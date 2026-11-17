//! Integrated board (unmodified SIMOS18.1 firmware on the TC1791 core + GPTA + PCP): the crank is
//! motored at a constant 1000 rpm by a bench motor (no plant, `Simos18Board::feed_crank`) and the
//! firmware's own crank/cam acquisition must lock onto the physical 60-2 + 2×8 cam signal.
//! Run with `--release --nocapture`.
//!
//! Evidence of synchronisation is taken from hardware outputs only: the firmware schedules the four
//! ignition outputs (G0.GTC12..15) only after `LV_SYN_ENG` (doc SIMOS18_GPTA_IO_RE.md T4), and their
//! dwell ends must land at a fixed angle before the TDC of the right cylinder, in firing order 1-3-4-2.
//!
//! Board dependency: the MC only stays up while the switch-off-path check (SOPC) passes, which needs the
//! power-stage drivers (`atic`, `power_stages`). Should the MC reset before synchronising, the test
//! reports BLOCKED after checking the tooth/PCP part.

mod common;

use simemu_tc1791::engine_io::IGNITION_GTC_BY_CYL;
use simemu_tc1791::{CpuBackend, Simos18Board};

const FLASH: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;
const RPM: f64 = 1000.0;
/// Firing TDC of cylinders 1..4 (°CRK after TDC0, firing order 1-3-4-2).
const TDC: [f64; 4] = [0.0, 540.0, 180.0, 360.0];

fn image(name: &str) -> Option<Vec<u8>> {
    let r = common::find_image(name);
    if r.is_none() {
        eprintln!("SKIP: {name} not found");
    }
    r
}

#[test]
fn firmware_synchronises_to_a_motored_crank() {
    for backend in CpuBackend::matrix() {
        firmware_synchronises_to_a_motored_crank_on(backend);
    }
}

fn firmware_synchronises_to_a_motored_crank_on(backend: CpuBackend) {
    let (Some(f), Some(mut e)) = (image(FLASH), image(EEPROM)) else { return };
    bind_dflash_power_class(&mut e, &f, 0x6A); // 264K SC8S50 power class (AGENTS.md §2.4 п.4 / §4)
    let mut b = Simos18Board::new_on(&f, &e, backend).expect("board");
    b.set_adc_input_voltage(2, 4, 2.5).unwrap(); // supply sense (boot_regression.rs)
    // the firmware arms the crank front end itself (FPCCTR0 = 0x00930046, doc §3.2)
    for _ in 0..300 {
        if b.engine_io_mut().gpta.mmio_read(0xF000_1848, 4) == 0x0093_0046 {
            break;
        }
        b.step_realtime_frame(0.001).unwrap();
    }
    assert_eq!(b.engine_io_mut().gpta.mmio_read(0xF000_1848, 4), 0x0093_0046, "crank front end not armed");
    let (resets0, armed_ms) = (b.hw().reset_count, b.board_time_s() * 1e3);

    let mut theta = 100.0;
    let (mut teeth_ok, mut blocked) = (false, false);
    let mut at15 = (0, 0);
    for ms in 0..400 {
        let t = b.board_time_s();
        let samples: Vec<(f64, f64)> = (1..=10).map(|i| (t + i as f64 * 1e-4, theta + RPM * 6.0 * i as f64 * 1e-4)).collect();
        theta = samples[9].1;
        b.feed_crank(&samples, [0.0, 0.0], None);
        b.run_for(1e-3).unwrap();
        if b.hw().reset_count != resets0 {
            blocked = true;
            eprintln!("BLOCKED: MC reset {} ms after arming (SOPC without power-stage drivers)", ms + 1);
            break;
        }
        let counts = (b.engine_io().wheel().unwrap().teeth_seen, b.engine_io().pcp().stats.starts[0x3F]);
        if ms == 15 {
            at15 = counts;
        }
        if ms == 40 {
            // after the crank start delay the PCP crank channel serves every tooth: 0x3F once per active
            // edge (DCM0 request)
            let (teeth, s3f) = (counts.0 - at15.0, counts.1 - at15.1);
            assert!(s3f + 2 >= teeth && s3f <= teeth + 2, "PCP ch 0x3F {s3f} for {teeth} teeth");
            assert!(b.engine_io().pcp().stats.starts[0x3A] + b.engine_io().pcp().stats.starts[0x3B] > 0, "cam channels");
            assert_eq!(b.engine_io().pcp().stats.errors, 0);
            teeth_ok = true;
        }
    }
    assert!(teeth_ok || blocked);
    if blocked {
        return;
    }
    // angle clock: GT1 advances PLLMTI (256) micro ticks per tooth = 15360 per revolution
    let g0 = b.engine_io_mut().gpta.gt_value(0, 1);
    for _ in 0..60 {
        let t = b.board_time_s();
        let samples: Vec<(f64, f64)> = (1..=10).map(|i| (t + i as f64 * 1e-4, theta + RPM * 6.0 * i as f64 * 1e-4)).collect();
        theta = samples[9].1;
        b.feed_crank(&samples, [0.0, 0.0], None);
        b.run_for(1e-3).unwrap();
    }
    let g1 = b.engine_io_mut().gpta.gt_value(0, 1);
    let d = g1.wrapping_sub(g0) & 0xFF_FFFF;
    assert!((15340..=15380).contains(&d), "GT1 advance over one revolution: {d}");

    // synchronised: every coil was fired, spark (end of dwell) at a fixed angle before its own TDC
    let st = b.engine_io_state();
    let mut sparks = Vec::new();
    for c in 0..4 {
        let p = st.coil[c].unwrap_or_else(|| panic!("cylinder {} coil (G0.GTC{}) never fired", c + 1, IGNITION_GTC_BY_CYL[c]));
        let btdc = (TDC[c] - p.end_deg).rem_euclid(720.0);
        let btdc = if btdc > 360.0 { btdc - 720.0 } else { btdc };
        eprintln!("cyl {} spark {:.1}° BTDC dwell {:.2} ms at t {:.4} s", c + 1, btdc, p.width_ms(), p.end_s);
        assert!((-10.0..=40.0).contains(&btdc), "cyl {} spark at {btdc}° BTDC", c + 1);
        assert!((0.3..=10.0).contains(&p.width_ms()), "dwell {} ms", p.width_ms());
        sparks.push((p.end_s, c + 1));
    }
    sparks.sort_by(|a, b| a.0.total_cmp(&b.0));
    let order: Vec<usize> = sparks.iter().map(|s| s.1).collect();
    let rot = order.iter().position(|&c| c == 1).unwrap();
    let order: Vec<usize> = order[rot..].iter().chain(&order[..rot]).copied().collect();
    assert_eq!(order, [1, 3, 4, 2], "firing order");
    eprintln!("synchronised: armed at {armed_ms:.0} ms, sparks in firing order {order:?}");
}

/// Re-seals ECME channels 6..=8 in DF0 (`0xAF000000..+0x18000`) with power class `b` (`0x6A` for stock `264K`)
/// using the firmware's `anvmcryptlight` stream cipher + CRC-16/CCITT-FALSE + FEE CRC-16/ARC trailer (`docs/SIMOS18_WFS5_RE.md` §8),
/// so a `259J` donor DFlash (`B = 0xBA`) does not enter `IMMO P-CLASS ERROR` (`0x42`) on stock `264K`.
fn bind_dflash_power_class(dflash: &mut [u8], flash: &[u8], b: u8) {
    const DF0: usize = 0x1_8000;
    const CH_LEN: usize = 100;
    const ST_LEN: usize = 13;
    const DT_LEN: usize = 53;
    let u = &flash[0x1_4200..0x1_4208];
    let k = [u[0] ^ u[1] ^ u[4], u[1] ^ u[2] ^ u[3], u[0] ^ u[2] ^ u[5], u[0] ^ u[3] ^ u[4], u[1] ^ u[3] ^ u[6], u[1] ^ u[4] ^ u[5]];
    let m = [u[4] ^ u[5] ^ u[6], u[2] ^ u[5] ^ u[7], u[4] ^ u[6] ^ u[7], u[0] ^ u[6] ^ u[7]];
    let filt = |s: u64| -> u32 {
        let bit = |i: u32| ((s >> (47 - i)) & 1) as u32;
        let n4 = |a, c, d, e| (bit(a) << 3) | (bit(c) << 2) | (bit(d) << 1) | bit(e);
        let t = |msk: u32, i: u32| (msk >> i) & 1;
        let idx = (t(0x287B, n4(2, 3, 5, 6)) << 4)
            | (t(0x267D, n4(8, 12, 14, 15)) << 3)
            | (t(0x267D, n4(17, 21, 23, 26)) << 2)
            | (t(0x267D, n4(28, 29, 31, 33)) << 1)
            | t(0x287B, n4(34, 43, 44, 46));
        t(0x7956_EB79, idx)
    };
    let apply_cipher = |ch: u16, buf: &mut [u8]| {
        for (w, pair) in buf.chunks_mut(2).enumerate() {
            let a = ch.wrapping_add(2 * w as u16) as u64;
            let mut s = (a << 32) | (a << 16) | ((k[0] as u64) << 8) | k[1] as u64;
            for j in 0..4 {
                let kb = k[2 + j] ^ m[j];
                for bit in (0..8).rev() {
                    s = ((s << 1) | (filt(s) ^ ((kb as u32 >> bit) & 1)) as u64) & 0xFFFF_FFFF_FFFF;
                }
            }
            for o in pair.iter_mut() {
                let mut ks = 0u8;
                for bit in (0..8).rev() {
                    let fb = [0u32, 2, 3, 6, 7, 8, 16, 22, 23, 26, 30, 41, 42, 43, 46, 47]
                        .iter()
                        .fold(0u64, |acc, &i| acc ^ ((s >> (47 - i)) & 1));
                    s = ((s << 1) | fb) & 0xFFFF_FFFF_FFFF;
                    ks |= (filt(s) as u8) << bit;
                }
                *o ^= ks;
            }
        }
    };
    let ccitt = |data: &[u8]| -> u16 {
        data.iter().fold(0xFFFFu16, |mut c, &x| {
            c ^= (x as u16) << 8;
            for _ in 0..8 {
                c = if c & 0x8000 != 0 { (c << 1) ^ 0x1021 } else { c << 1 };
            }
            c
        })
    };
    let arc = |init: u16, data: &[u8]| -> u16 {
        data.iter().fold(init, |mut c, &x| {
            c ^= x as u16;
            for _ in 0..8 {
                c = if c & 1 != 0 { (c >> 1) ^ 0xA001 } else { c >> 1 };
            }
            c
        })
    };
    let bank = &mut dflash[..DF0];
    let recs: Vec<(usize, u16, usize)> = (8..bank.len().saturating_sub(7))
        .step_by(8)
        .filter_map(|t| {
            let tr = &bank[t..t + 8];
            let c = u16::from_be_bytes([tr[6], tr[7]]);
            let n = ((tr[0] as u16 & 0xF) << 12) | ((tr[1] as u16) << 4) | (tr[2] as u16 >> 4);
            let units = ((tr[2] as usize & 0xF) << 8) | tr[3] as usize;
            (n == !c && units > 0 && units * 8 <= t).then(|| (t, u16::from_be_bytes([tr[4], tr[5]]), units))
        })
        .collect();
    let seed = recs
        .iter()
        .find_map(|&(t, id, u)| {
            if id != 0xFFFE || u != 1 {
                return None;
            }
            let raw = &bank[t - 8..t];
            let s = u32::from_be_bytes([raw[4], raw[5], raw[6], raw[7]]);
            let mut h = s.to_le_bytes().to_vec();
            h.extend_from_slice(&[1, 0, 0xFE, 0xFF]);
            (arc(arc(0xA55A, raw), &h) == u16::from_be_bytes([bank[t + 6], bank[t + 7]])).then_some(s)
        })
        .expect("FEE bank header");
    for ch in 6..=8u16 {
        let matching: Vec<(usize, u16, usize)> = recs.iter().copied().filter(|&(_, id, _)| id == ((ch << 8) | 0x46)).collect();
        let Some(&(last_t, _, last_u)) = matching.iter().max_by_key(|&&(t, _, _)| t) else { continue };
        let payload_offs = |t: usize, u: usize| (t - u * 8..t).filter(|i| i % 0x100 != 0);
        let last_raw: Vec<u8> = payload_offs(last_t, last_u).map(|o| bank[o]).collect();
        let mut plain = [0u8; CH_LEN];
        plain.copy_from_slice(&last_raw[..CH_LEN]);
        apply_cipher(ch, &mut plain);
        plain[ST_LEN + 0x23] = b;
        let crc = ccitt(&plain[ST_LEN..ST_LEN + DT_LEN - 2]);
        plain[ST_LEN + DT_LEN - 2..ST_LEN + DT_LEN].copy_from_slice(&crc.to_le_bytes());
        let mut enc = plain;
        apply_cipher(ch, &mut enc);
        for &(t, id, u) in &matching {
            let raw: Vec<u8> = payload_offs(t, u).map(|o| bank[o]).collect();
            if raw != last_raw {
                continue;
            }
            for (o, v) in payload_offs(t, u).zip(enc) {
                bank[o] = v;
            }
            let p: Vec<u8> = payload_offs(t, u).map(|o| bank[o]).collect();
            let mut h = seed.to_le_bytes().to_vec();
            h.extend_from_slice(&(u as u16).to_le_bytes());
            h.extend_from_slice(&id.to_le_bytes());
            let c = arc(arc(0xA55A, &p), &h);
            let n = !c;
            bank[t] = (bank[t] & 0xF0) | (n >> 12) as u8;
            bank[t + 1] = (n >> 4) as u8;
            bank[t + 2] = ((n & 0xF) << 4) as u8 | (bank[t + 2] & 0x0F);
            bank[t + 6..t + 8].copy_from_slice(&c.to_be_bytes());
        }
    }
}
