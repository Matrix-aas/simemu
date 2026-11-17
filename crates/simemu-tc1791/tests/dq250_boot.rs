//! DQ250 (TC1766) mechatronic boot tests (run with `--release -- --nocapture` for the report).
//!
//! Images are looked up in `$SIMEMU_IMAGE_DIR`, `<workspace>/vault/firmwares/tcu/dq250/`, and the workspace root;
//! a test is skipped when its image is missing. `SIMEMU_DQ250_MS` sets the simulated time of
//! `dq250_boot` (default 3500 ms: CBOOT's CRC-32 pass over ASW + CAL takes ~2.5 s of simulated time).
//! * `dq250_boot`: the tuned file as read (stale CAL CRC) — CBOOT keeps the ASW blocked and
//!   serves UDS itself.
//! * `dq250_asw_can`: the 0D9300011R/4319 read and the tuned file after the post-programming CRC
//!   recomputation (`simemu_damos::apply_post_programming_dq250`, AGENTS.md §2.4 item 3) run the ASW,
//!   which transmits its periodic frames and answers UDS on 0x7E1 -> 0x7E9.
//! * `dq250_wakeup_and_wfs5`: the board's supply input (AIN31 / EMUX 0) keeps the ASW's network request (CAN and
//!   NM alive past 8 s, released at 0 V) and the IMO's WFS5 challenge 0x012 is answered from a lab-style EEPROM
//!   record (negative control: wrong ID_kombi).
//! * `dq250_bench_scenario` (ignored, `scripts/run_scenarios.sh`): 20 s bench timeline and DTCs by the image's table.

mod common;

use simemu_tc1791::{CanFrame, CpuBackend, Dq250TcuBoard};
use std::collections::BTreeMap;

/// DQ250 0D9300012F (SW 4930) image.
const IMAGE: &str = common::DQ250_012F;
/// DQ250 0D9300011R (SW 4319, 0x180000 bytes) stock image.
const IMAGE_4319: &str = common::DQ250_011R;

fn image_named(name: &str) -> Option<Vec<u8>> {
    common::find_image(name)
}

fn image() -> Option<Vec<u8>> {
    image_named(IMAGE)
}

/// CRC-32 register (reflected 0xEDB88320, init 0xFFFFFFFF, no final XOR) as computed by CBOOT
/// 0x80016330; a block is valid when the register ends at 0.
fn crc32_register(data: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { (c >> 1) ^ 0xEDB8_8320 } else { c >> 1 };
        }
    }
    c
}

/// One request frame on 0x7E1 (node 1), then `wait_s` of simulated time; the 0x7E9 frames sent meanwhile.
fn uds(b: &mut Dq250TcuBoard, data: [u8; 8], wait_s: f64) -> Vec<CanFrame> {
    let before = b.can_tx().len();
    let f = CanFrame { node: 1, mo_index: 0, arb_id: 0x7E1, extended: false, dlc: 8, data, timestamp_us: 0 };
    assert_eq!(b.inject_can_rx(&f), 1, "receive MO for 0x7E1");
    b.run_for(wait_s).expect("run");
    b.can_tx()[before..].iter().filter(|f| f.arb_id == 0x7E9).cloned().collect()
}

/// ReadDataByIdentifier F187 (ISO-TP first frame, flow control, consecutive frames) -> ASCII part number.
fn read_f187(b: &mut Dq250TcuBoard, wait_s: f64) -> String {
    let first = uds(b, [0x03, 0x22, 0xF1, 0x87, 0x55, 0x55, 0x55, 0x55], wait_s);
    println!("UDS 22 F187 -> {:02X?}", first.iter().map(|f| (f.arb_id, f.data)).collect::<Vec<_>>());
    assert_eq!(first.len(), 1);
    assert_eq!((first[0].node, first[0].arb_id), (1, 0x7E9));
    assert_eq!(&first[0].data[..5], &[0x10, 0x0E, 0x62, 0xF1, 0x87], "ISO-TP first frame, positive response");
    let rest = uds(b, [0x30, 0, 0, 0, 0, 0, 0, 0], wait_s);
    let mut payload = first[0].data[5..].to_vec();
    for f in &rest {
        payload.extend_from_slice(&f.data[1..]);
    }
    payload.truncate(0x0E - 3);
    String::from_utf8_lossy(&payload).to_string()
}

/// UDS request on 0x7E1 (single frame) and the complete ISO-TP answer from 0x7E9 (flow control sent on the
/// first frame), polled in 5 ms steps for at most `timeout_s`.
fn uds_request(b: &mut Dq250TcuBoard, req: &[u8], timeout_s: f64) -> Option<Vec<u8>> {
    let mut data = [0x55u8; 8];
    data[0] = req.len() as u8;
    data[1..=req.len()].copy_from_slice(req);
    let mut seen = b.can_tx().len();
    b.inject_can_rx(&CanFrame { node: 1, mo_index: 0, arb_id: 0x7E1, extended: false, dlc: 8, data, timestamp_us: 0 });
    let (mut out, mut want) = (Vec::new(), usize::MAX);
    let mut t = 0.0;
    while t < timeout_s {
        b.run_for(0.005).ok()?;
        t += 0.005;
        let frames: Vec<[u8; 8]> = b.can_tx()[seen..].iter().filter(|f| f.arb_id == 0x7E9).map(|f| f.data).collect();
        seen = b.can_tx().len();
        for d in frames {
            match d[0] >> 4 {
                0 if d[1] == 0x7F && d[3] == 0x78 => {} // response pending
                0 => return Some(d[1..=(d[0] & 0xF) as usize].to_vec()),
                1 => {
                    want = (((d[0] & 0xF) as usize) << 8) | d[1] as usize;
                    out.extend_from_slice(&d[2..]);
                    let fc = CanFrame { node: 1, mo_index: 0, arb_id: 0x7E1, extended: false, dlc: 8, data: [0x30, 0, 0, 0, 0, 0, 0, 0], timestamp_us: 0 };
                    b.inject_can_rx(&fc);
                }
                2 => out.extend_from_slice(&d[1..]),
                _ => {}
            }
            if out.len() >= want {
                out.truncate(want);
                return Some(out);
            }
        }
    }
    None
}

/// One UDS exchange as a tester does it (ISO-TP both ways: a request longer than 7 bytes goes as first frame,
/// waits for the ECU's flow control, then consecutive frames; a multi-frame answer gets our flow control), while
/// `tick` runs the board in 5 ms steps (chassis frames etc.). Response-pending (7F xx 78) keeps waiting. `None` on
/// timeout.
fn uds_xfer(b: &mut Dq250TcuBoard, req: &[u8], timeout_s: f64, tick: &mut dyn FnMut(&mut Dq250TcuBoard)) -> Option<Vec<u8>> {
    let tx = |b: &mut Dq250TcuBoard, data: [u8; 8]| {
        b.inject_can_rx(&CanFrame { node: 1, mo_index: 0, arb_id: 0x7E1, extended: false, dlc: 8, data, timestamp_us: 0 });
    };
    let mut seen = b.can_tx().len();
    let mut cf_left: Vec<[u8; 8]> = Vec::new();
    if req.len() <= 7 {
        let mut d = [0x55u8; 8];
        d[0] = req.len() as u8;
        d[1..=req.len()].copy_from_slice(req);
        tx(b, d);
    } else {
        let mut d = [0x55u8; 8];
        d[0] = 0x10 | (req.len() >> 8) as u8;
        d[1] = req.len() as u8;
        d[2..8].copy_from_slice(&req[..6]);
        tx(b, d);
        for (k, c) in req[6..].chunks(7).enumerate() {
            let mut d = [0x55u8; 8];
            d[0] = 0x20 | ((k + 1) & 0xF) as u8;
            d[1..1 + c.len()].copy_from_slice(c);
            cf_left.push(d);
        }
    }
    let (mut out, mut want) = (Vec::new(), usize::MAX);
    let mut t = 0.0;
    while t < timeout_s {
        tick(b);
        b.run_for(0.005).ok()?;
        t += 0.005;
        let frames: Vec<[u8; 8]> = b.can_tx()[seen..].iter().filter(|f| f.arb_id == 0x7E9).map(|f| f.data).collect();
        seen = b.can_tx().len();
        for d in frames {
            match d[0] >> 4 {
                3 if !cf_left.is_empty() => {
                    for f in std::mem::take(&mut cf_left) {
                        tx(b, f);
                        b.run_for(0.001).ok()?;
                    }
                }
                0 if d[1] == 0x7F && d[3] == 0x78 => {}
                0 => return Some(d[1..=(d[0] & 0xF) as usize].to_vec()),
                1 => {
                    want = (((d[0] & 0xF) as usize) << 8) | d[1] as usize;
                    out.extend_from_slice(&d[2..]);
                    tx(b, [0x30, 0, 0, 0, 0, 0, 0, 0]);
                }
                2 => out.extend_from_slice(&d[1..]),
                _ => {}
            }
            if out.len() >= want {
                out.truncate(want);
                return Some(out);
            }
        }
    }
    None
}

fn hexs(d: &[u8]) -> String {
    d.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
}

/// Exploration (`--ignored`): default DQ250 image (or `SIMEMU_DQ250_IMAGE`), lab-style IMMO record, chassis frames and the
/// cluster answering 0x012; after `SIMEMU_DQ250_MS` (default 4000) the requests of `SIMEMU_DQ250_REQS`
/// ("10 03;22 0600;..."; `key` = 27 04 with seed + 0x0010829B from the last 27 03 answer, `wait:ms` = run that long)
/// are sent one by one. `SIMEMU_DQ250_EEPROM` / `_EEPROM_OUT`: EEPROM image in / out. `SIMEMU_DQ250_PEEK=addr:n,..`
/// prints RAM at the end.
#[test]
#[ignore]
fn dq250_uds_probe() {
    let name = std::env::var("SIMEMU_DQ250_IMAGE").unwrap_or_else(|_| IMAGE.into());
    let mut img = image_named(&name).expect("image");
    simemu_damos::apply_post_programming_dq250(&mut img).expect("post-programming");
    let (mut e, cluster) = immo_eeprom([0x12, 0x34]);
    if let Ok(p) = std::env::var("SIMEMU_DQ250_EEPROM") {
        e = std::fs::read(p).unwrap();
    }
    let mut b = Dq250TcuBoard::new(&img, &e).expect("board");
    let ms: f64 = std::env::var("SIMEMU_DQ250_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(4000.0);
    run_with_chassis(&mut b, &cluster, ms / 1000.0);
    let mut enc = simemu_bridge::MqbChassisEncoder::default();
    let c = simemu_bridge::ChassisInputs { selector_code: 8, drive_profile: 2, terminal_15: true, ..Default::default() };
    let mut seen = b.can_tx().len();
    let mut tick = |b: &mut Dq250TcuBoard| {
        let mut rx: Vec<CanFrame> = Vec::new();
        for f in &b.can_tx()[seen..] {
            enc.on_tcu_frame(f.arb_id, &f.data);
            if f.arb_id == 0x012 {
                rx.push(CanFrame { node: 1, mo_index: 0, arb_id: 0x013, extended: false, dlc: 8, data: cluster.answer_challenge(&f.data), timestamp_us: 0 });
            }
        }
        seen = b.can_tx().len();
        if b.cpu.hw().sim_time_s % 0.01 < 0.005 {
            for (id, dlc, data) in enc.encode(&c) {
                rx.push(CanFrame { node: 1, mo_index: 0, arb_id: id, extended: false, dlc, data, timestamp_us: 0 });
            }
        }
        for f in &rx {
            b.inject_can_rx(f);
        }
    };
    let mut seed = 0u32;
    for r in std::env::var("SIMEMU_DQ250_REQS").unwrap_or_default().split(';').map(str::trim).filter(|s| !s.is_empty()) {
        if let Some(w) = r.strip_prefix("wait:") {
            for _ in 0..(w.parse::<f64>().unwrap() / 5.0) as usize {
                tick(&mut b);
                b.run_for(0.005).unwrap();
            }
            continue;
        }
        let req: Vec<u8> = if r == "key" {
            let k = seed.wrapping_add(0x0010_829B);
            [vec![0x27, 0x04], k.to_be_bytes().to_vec()].concat()
        } else {
            r.split_whitespace().flat_map(|w| (0..w.len()).step_by(2).map(move |i| u8::from_str_radix(&w[i..i + 2], 16).unwrap())).collect()
        };
        let ans = uds_xfer(&mut b, &req, 3.0, &mut tick);
        println!("t={:6.2} > {} < {}", b.cpu.hw().sim_time_s, hexs(&req), ans.as_deref().map_or("(timeout)".into(), hexs));
        if let Some(a) = ans.filter(|a| a.len() == 6 && a[0] == 0x67 && a[1] == 0x03) {
            seed = u32::from_be_bytes([a[2], a[3], a[4], a[5]]);
        }
    }
    for spec in std::env::var("SIMEMU_DQ250_PEEK").unwrap_or_default().split(',').filter(|s| !s.is_empty()) {
        let (a, n) = spec.split_once(':').unwrap_or((spec, "16"));
        let a = u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap();
        println!("{a:08X}: {}", hexs(&b.cpu.read_mem(a, n.parse().unwrap()).unwrap()));
    }
    let p = b.periph();
    let w: Vec<String> = p.eeprom.log.iter().filter(|l| l.0 == 2).map(|(_, a, n)| format!("{a:04X}+{n}")).collect();
    println!("EEPROM writes: {}", w.join(" "));
    if let Ok(path) = std::env::var("SIMEMU_DQ250_EEPROM_OUT") {
        std::fs::write(&path, &p.eeprom.mem).unwrap();
    }
    println!("resets {:?} traps {}", b.cpu.hw().reset_log, b.cpu.hw().trap_count);
}

#[test]
fn dq250_boot() {
    for backend in CpuBackend::matrix() {
        dq250_boot_on(backend);
    }
}

fn dq250_boot_on(backend: CpuBackend) {
    let Some(img) = image() else {
        eprintln!("SKIP: {IMAGE} not found");
        return;
    };
    let ms: f64 = std::env::var("SIMEMU_DQ250_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(3500.0);
    let mut b = Dq250TcuBoard::new_on(&img, &[], backend).expect("board");
    println!("[{backend:?}] image: {:?}", b.info);
    let stats = b.run_for(ms / 1000.0).expect("emulation error");
    let hw = b.cpu.hw();
    println!(
        "pc=0x{:08X} insns={} sim={:.3}s wall={:.2}s fcpu={} resets={:?} irqs={} traps={}",
        b.pc(),
        hw.total_instructions,
        hw.sim_time_s,
        stats.wall_time_s,
        hw.scu.fcpu_hz(),
        hw.reset_log,
        hw.irq_dispatch_count,
        hw.trap_count
    );
    println!(
        "wdt: mode={:?} services={} access_errors={} endinit_dropped={}",
        hw.scu.wdt_mode, hw.scu.wdt_services, hw.scu.wdt_access_errors, hw.scu.endinit_dropped
    );
    println!("isa fixes: {:?}", hw.fixes.sites.keys().map(|a| format!("{a:08X}")).collect::<Vec<_>>());
    let ms_hits: BTreeMap<&str, u64> = b.milestones().into_iter().collect();
    println!("milestones: {ms_hits:?}");
    let p = b.periph();
    println!("eeprom: cmds {:?} accesses {:?}", p.eeprom.commands, p.eeprom.log.iter().map(|(c, a, n)| format!("{}{a:04X}+{n}", if *c == 3 { 'R' } else { 'W' })).collect::<Vec<_>>());
    println!("eeprom[0..0x60] after CBOOT: {:02X?}", &p.eeprom.mem[..0x60]);
    println!("spi transfers: {}, CAN tx: {}", p.spi_log.len(), b.can_tx().len());
    println!("pending SRCs: {:?}", b.pending_requests().iter().map(|(a, v)| format!("{a:08X}={v:04X}")).collect::<Vec<_>>());
    let mut reads: Vec<_> = p.plain_reads.iter().collect();
    reads.sort_by(|a, b| b.1.cmp(a.1));
    println!("plain-register reads: {:?}", reads.iter().take(20).map(|(a, c)| format!("{a:08X}x{c}")).collect::<Vec<_>>());

    // SBOOT -> CBOOT without resets or traps; SBOOT and CBOOT read the SPI EEPROM.
    assert!(ms_hits["SBOOT_START"] >= 1 && ms_hits["CBOOT_START"] >= 1, "{ms_hits:?}");
    assert_eq!(hw.reset_count, 0, "{:?}", hw.reset_log);
    assert_eq!(hw.trap_count, 0);
    assert_eq!(hw.scu.wdt_access_errors, 0);
    assert!(p.eeprom.commands.get(&0x03).copied().unwrap_or(0) >= 4, "EEPROM READ commands");

    // CBOOT verifies the ASW (0x80040000 + 0x130000) and CAL (0x80020000 + 0x20000) blocks with
    // CRC-32. The ASW is intact; the tuned CAL is not, so with a blank EEPROM (no stored
    // programming status) the unmodified CBOOT keeps the application blocked.
    assert_eq!(crc32_register(&img[0x4_0000..0x17_0000]), 0, "ASW block CRC");
    let cal_ok = crc32_register(&img[0x2_0000..0x4_0000]) == 0;
    println!("CAL block CRC valid: {cal_ok}");
    if !cal_ok {
        assert_eq!(ms_hits["ASW_START"], 0, "CBOOT must not start an application with a CAL CRC error");
        assert_eq!(p.eeprom.mem[0] & 0x0E, 0x0E, "CBOOT records the invalid blocks in its EEPROM status");
    }

    // The bootloader serves UDS on CAN node 1 (0x7E1 -> 0x7E9): ReadDataByIdentifier F187.
    let part = read_f187(&mut b, 0.1);
    println!("F187 = {part:?}");
    assert_eq!(part, "02E927770AN");
}

#[test]
fn dq250_asw_can() {
    for backend in CpuBackend::matrix() {
        for (name, sw) in [(IMAGE_4319, "0D9300011R "), (IMAGE, "0D9300012F ")] {
            dq250_asw_can_on(backend, name, sw);
        }
    }
}

fn dq250_asw_can_on(backend: CpuBackend, name: &str, sw: &str) {
    let Some(mut img) = image_named(name) else {
        eprintln!("SKIP: {name} not found");
        return;
    };
    // Stale block CRC words are recomputed with an explicit notice; nothing else is written.
    let log = simemu_damos::apply_post_programming_dq250(&mut img).expect("post-programming");
    println!("[{backend:?}] {name}: post-programming {log:?}");
    assert_eq!(log.len(), (name == IMAGE) as usize, "only the tuned file's CAL CRC is stale");
    let mut b = Dq250TcuBoard::new_on(&img, &[], backend).expect("board");
    let stats = b.run_for(4.5).expect("emulation error");
    let hw = b.cpu.hw();
    let ms_hits: BTreeMap<&str, u64> = b.milestones().into_iter().collect();
    println!(
        "milestones {ms_hits:?} fcpu {} irqs {} resets {:?} traps {} adc0 conversions {} wall {:.2}s",
        hw.scu.fcpu_hz(),
        hw.irq_dispatch_count,
        hw.reset_log,
        hw.trap_count,
        b.periph().adc0.conversions,
        stats.wall_time_s
    );
    assert_eq!(ms_hits["ASW_START"], 1, "{ms_hits:?}");
    assert_eq!((hw.reset_count, hw.trap_count), (0, 0), "{:?}", hw.reset_log);

    // Periodic application frames on node 1 (CBOOT sends nothing unsolicited).
    let mut ids: BTreeMap<u32, Vec<u64>> = BTreeMap::new();
    for f in b.can_tx().iter().filter(|f| f.node == 1) {
        ids.entry(f.arb_id).or_default().push(f.timestamp_us);
    }
    println!("CAN TX ids: {:?}", ids.iter().map(|(id, t)| format!("{id:03X} x{}", t.len())).collect::<Vec<_>>());
    for id in [0x0AD, 0x0AE, 0x128, 0x12D, 0x394, 0x3C8] {
        assert!(ids.get(&id).map_or(0, |t| t.len()) >= 10, "no periodic 0x{id:03X}");
    }
    let t = &ids[&0x0AD];
    let period_ms = (t[t.len() - 1] - t[0]) as f64 / (t.len() - 1) as f64 / 1000.0;
    println!("0x0AD: {} frames, mean period {period_ms:.2} ms", t.len());
    assert!((9.0..11.0).contains(&period_ms), "0x0AD period {period_ms} ms");

    // UDS on the application: ReadDataByIdentifier F187 (ISO-TP N_Bs is shorter than 100 ms: the flow
    // control follows within 20 ms) and DiagnosticSessionControl extended.
    let part = read_f187(&mut b, 0.02);
    println!("F187 = {part:?}");
    assert_eq!(part, sw);
    let r = uds(&mut b, [0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA], 0.02);
    println!("UDS 10 03 -> {:02X?}", r.iter().map(|f| f.data).collect::<Vec<_>>());
    assert_eq!(&r[0].data[..3], &[0x06, 0x50, 0x03], "positive response, extended session");
}

/// Board constants against the calibrations (VERIFIED-S): the gear sets the two TCU images are calibrated for
/// (`tcu_dq250::gear_set`: 0D9300011R = S3 0D9 set with 62/13 and 62/18 finals, 0D9300012F = 02E set with 70/16 and 70/21),
/// and the travel-sensor characteristic the board's sensor model inverts (`dq250_io::TRAVEL_MV/_CMM` = CAL 0x80030400 / 0x80030424 of
/// 0D9300011R; the mV axis also at 0x8003479C of 0D9300012F).
#[test]
fn dq250_calibration_constants() {
    use simemu_tc1791::dq250_io::{TRAVEL_CMM, TRAVEL_MV};
    use simemu_tc1791::tcu_dq250::gear_set;
    let rd = |f: &[u8], a: usize, n: usize| (0..n).map(|i| i16::from_le_bytes([f[a - 0x8000_0000 + 2 * i], f[a - 0x8000_0000 + 2 * i + 1]]) as f64).collect::<Vec<_>>();
    if let Some(f) = image_named(IMAGE_4319) {
        let g = gear_set(&f).expect("gear set 4319");
        assert_eq!((g.ratios[0], g.ratios[5], g.finals[0], g.finals[5]), (38.0 / 13.0, 26.0 / 32.0, 62.0 / 13.0, 62.0 / 18.0));
        assert_eq!(g.reverse, (27.0 * 22.0 / (14.0 * 13.0), 62.0 / 18.0));
        assert_eq!(rd(&f, 0x8003_0400, 17), TRAVEL_MV);
        assert_eq!(rd(&f, 0x8003_0424, 17), TRAVEL_CMM);
    }
    if let Some(f) = image() {
        let g = gear_set(&f).expect("gear set 4930");
        assert_eq!((g.ratios[0], g.ratios[5], g.finals[0], g.finals[5]), (45.0 / 13.0, 31.0 / 41.0, 70.0 / 16.0, 70.0 / 21.0));
        assert_eq!(g.reverse, (33.0 * 22.0 / (14.0 * 13.0), 70.0 / 21.0));
        assert_eq!(rd(&f, 0x8003_479C, 17), TRAVEL_MV);
    }
}

/// Lab-style WFS5 data record (K1 = 00 01 .. 0F, B 0x5C, ID_kombi 12 34) in the three copies of an erased part.
fn immo_eeprom(id_kombi: [u8; 2]) -> (Vec<u8>, simemu_tc1791::Wfs5ImmoPartner) {
    use simemu_tc1791::tcu_dq250::{write_wfs5_data_record, EEPROM_DEFAULT_SIZE, IMMO_DATA_COPIES, IMMO_DATA_LEN};
    let key: [u8; 16] = core::array::from_fn(|i| i as u8);
    let mut e = vec![0xFF; EEPROM_DEFAULT_SIZE];
    let mut rec = [0u8; IMMO_DATA_LEN];
    write_wfs5_data_record(&mut rec, &key, "LABEMU0123456789A", 0x5C, [0x12, 0x34], 1);
    for o in IMMO_DATA_COPIES {
        e[o..o + IMMO_DATA_LEN].copy_from_slice(&rec);
    }
    let p = simemu_tc1791::Wfs5ImmoPartner::from_mqb_slots_tcu(&[0; 16], &[0; 16], 0, id_kombi, [0; 2], "").with_key(key, 0x5C);
    (e, p)
}

/// `secs` of bench time in 10 ms steps: chassis frames (BCM, ESP, selector in P) and the virtual cluster answering
/// every 0x012 challenge; returns the 0x012 frames seen.
fn run_with_chassis(b: &mut Dq250TcuBoard, cluster: &simemu_tc1791::Wfs5ImmoPartner, secs: f64) -> Vec<[u8; 8]> {
    let mut enc = simemu_bridge::MqbChassisEncoder::default();
    let c = simemu_bridge::ChassisInputs { selector_code: 8, drive_profile: 2, terminal_15: true, ..Default::default() };
    let (mut seen, mut challenges) = (b.can_tx().len(), Vec::new());
    for _ in 0..(secs / 0.01).round() as usize {
        let mut rx: Vec<CanFrame> = Vec::new();
        for f in &b.can_tx()[seen..] {
            enc.on_tcu_frame(f.arb_id, &f.data);
            if f.arb_id == 0x012 {
                challenges.push(f.data);
                rx.push(CanFrame { node: 1, mo_index: 0, arb_id: 0x013, extended: false, dlc: 8, data: cluster.answer_challenge(&f.data), timestamp_us: 0 });
            }
        }
        seen = b.can_tx().len();
        for (id, dlc, data) in enc.encode(&c) {
            rx.push(CanFrame { node: 1, mo_index: 0, arb_id: id, extended: false, dlc, data, timestamp_us: 0 });
        }
        for f in &rx {
            b.inject_can_rx(f);
        }
        b.run_for(0.01).expect("run");
    }
    challenges
}

/// The board's supply input keeps the ASW awake and the IMO talks WFS5 (task: CAN alive, immobiliser):
/// * with the supply at battery level (default 13.8 V) the periodic frames and the NM frames continue past the
///   ~8 s at which the ASW used to fall silent, and the IMO sends 0x012 challenges;
/// * the cluster's answer computed from the EEPROM record's K1 / B / ID_kombi stops the challenges; a wrong
///   ID_kombi does not (negative control);
/// * with the supply input at 0 V the ASW releases the bus (no periodic frames in the last 2 s).
#[test]
fn dq250_wakeup_and_wfs5() {
    for backend in CpuBackend::matrix() {
        for name in [IMAGE_4319, IMAGE] {
            let Some(mut img) = image_named(name) else {
                eprintln!("SKIP: {name} not found");
                continue;
            };
            simemu_damos::apply_post_programming_dq250(&mut img).expect("post-programming");
            let periodic_after = |b: &Dq250TcuBoard, id: u32, t_us: u64| b.can_tx().iter().filter(|f| f.arb_id == id && f.timestamp_us > t_us).count();
            // accepted answer
            let (e, cluster) = immo_eeprom([0x12, 0x34]);
            let mut b = Dq250TcuBoard::new_on(&img, &e, backend).expect("board");
            let ch = run_with_chassis(&mut b, &cluster, 10.0);
            let (ad, nm) = (periodic_after(&b, 0x0AD, 9_000_000), periodic_after(&b, 0x1B00_0077, 8_000_000));
            println!("[{backend:?}] {name}: challenges {} {:02X?}, 0x0AD in the last 1 s: {ad}, NM after 8 s: {nm}", ch.len(), ch.first());
            assert!(ad >= 90, "periodic CAN alive at 10 s");
            assert!(nm >= 5, "NM continues (local network request)");
            assert!(!ch.is_empty() && ch.len() <= 2, "challenge sent and answered: {}", ch.len());
            // wrong ID_kombi: the IMO keeps asking
            let (e, wrong) = immo_eeprom([0x12, 0x35]);
            let mut b = Dq250TcuBoard::new_on(&img, &e, backend).expect("board");
            let ch = run_with_chassis(&mut b, &wrong, 6.0);
            println!("[{backend:?}] {name}: wrong ID_kombi -> {} challenges", ch.len());
            assert!(ch.len() >= 20, "rejected answers: the challenge repeats");
            // supply input off: bus released
            let mut b = Dq250TcuBoard::new_on(&img, &e, backend).expect("board");
            b.set_kl15_v(0.0);
            b.run_for(10.0).expect("run");
            let ad = periodic_after(&b, 0x0AD, 8_000_000);
            println!("[{backend:?}] {name}: supply input 0 V -> 0x0AD after 8 s: {ad}, 0x012: {}", periodic_after(&b, 0x012, 0));
            assert_eq!(ad, 0, "no network request without the supply input");
        }
    }
}

/// Bench scenario (`--ignored`, `scripts/run_scenarios.sh`): the DQ250 alone on the bench as far as the board is
/// emulated today — supply input at battery level, lab-style IMMO record, chassis frames (BCM, ESP standstill,
/// selector P for 4 s, then D) and the virtual cluster answering 0x012. Prints a state timeline and the DTCs with
/// the image's own SAE table. Asserts what is verified: no reset/trap, CAN and NM alive for the whole run, WFS5
/// accepted (one challenge), UDS answers. Sensors, valve drivers and solenoids are not emulated yet (no engine or
/// ECU frames either), so the TCU stays without gear: its DTCs are listed, not asserted.
#[test]
#[ignore]
fn dq250_bench_scenario() {
    for backend in CpuBackend::matrix() {
        let name = std::env::var("SIMEMU_DQ250_IMAGE").unwrap_or_else(|_| IMAGE.into());
        let Some(mut img) = image_named(&name) else {
            println!("SKIP: {name} not found");
            return;
        };
        println!("[{backend:?}] {name}: post-programming {:?}", simemu_damos::apply_post_programming_dq250(&mut img).expect("post-programming"));
        let (e, cluster) = immo_eeprom([0x12, 0x34]);
        let mut b = Dq250TcuBoard::new_on(&img, &e, backend).expect("board");
        let mut enc = simemu_bridge::MqbChassisEncoder::default();
        let (mut seen, mut challenges, mut accepted_at) = (0, 0, None);
        for step in 0..2000 {
            let t = step as f64 * 0.01;
            let c = simemu_bridge::ChassisInputs { selector_code: if t < 4.0 { 8 } else { 5 }, drive_profile: 2, terminal_15: true, ..Default::default() };
            let mut rx = Vec::new();
            for f in &b.can_tx()[seen..] {
                enc.on_tcu_frame(f.arb_id, &f.data);
                if f.arb_id == 0x012 {
                    challenges += 1;
                    accepted_at = Some(t);
                    rx.push(CanFrame { node: 1, mo_index: 0, arb_id: 0x013, extended: false, dlc: 8, data: cluster.answer_challenge(&f.data), timestamp_us: 0 });
                }
            }
            seen = b.can_tx().len();
            for (id, dlc, data) in enc.encode(&c) {
                rx.push(CanFrame { node: 1, mo_index: 0, arb_id: id, extended: false, dlc, data, timestamp_us: 0 });
            }
            for f in &rx {
                b.inject_can_rx(f);
            }
            b.run_for(0.01).expect("run");
            if step % 200 == 199 {
                let last = |id: u32| b.can_tx().iter().rev().find(|f| f.arb_id == id).map(|f| f.data);
                let n_since = |id: u32| b.can_tx().iter().filter(|f| f.arb_id == id && f.timestamp_us as f64 > (t - 1.0) * 1e6).count();
                println!(
                    "t={:5.2}s selector {} | 0x0AD x{} {:02X?} | 0x0AE {:02X?} | NM x{} | 0x012 total {challenges}",
                    t + 0.01,
                    c.selector_code,
                    n_since(0x0AD),
                    last(0x0AD),
                    last(0x0AE),
                    n_since(0x1B00_0077)
                );
            }
        }
        let hw = b.cpu.hw();
        let alive = b.can_tx().iter().filter(|f| f.arb_id == 0x0AD && f.timestamp_us > 19_000_000).count();
        println!("resets {:?} traps {} irqs {} | WFS5: {challenges} challenge(s), last at {accepted_at:?} s | 0x0AD in the last 1 s: {alive}", hw.reset_log, hw.trap_count, hw.irq_dispatch_count);
        assert_eq!((hw.reset_count, hw.trap_count), (0, 0));
        assert!(alive >= 90 && (1..=2).contains(&challenges), "CAN alive and WFS5 accepted");
        for mask in [0x09u8, 0x04] {
            let r = uds_request(&mut b, &[0x19, 0x02, mask], 2.0).expect("19 02 answer");
            assert_eq!(&r[..2], &[0x59, 0x02]);
            let list: Vec<String> = r[3..]
                .as_chunks::<4>()
                .0
                .iter()
                .map(|d| {
                    let dtc = u32::from_be_bytes([0, d[0], d[1], d[2]]);
                    format!("{:06X}={}/{:02X}", dtc, b.dtc_sae_code(dtc).unwrap_or_else(|| "?".into()), d[3])
                })
                .collect();
            println!("DTC (19 02 {mask:02X}): {}", list.join(" "));
        }
    }
}

/// Exploration helper (`--ignored`): probes (`SIMEMU_DQ250_PROBES=addr,..`), memory peek
/// (`SIMEMU_DQ250_PEEK=addr,..`, with the flash source of RAM code), CAN MO list and raw CAN
/// requests (`SIMEMU_DQ250_UDS=node:id:hex8,..`). Also: `SIMEMU_DQ250_EEPROM`/`_EEPROM_OUT` (EEPROM image in/out),
/// `SIMEMU_DQ250_AIN=n:volts,..`, `SIMEMU_DQ250_CHASSIS` (chassis frames every 10 ms; `_KL15OFF`, `_NM=id:hex8`,
/// `_IMMO=key:B:idkombi` = cluster answering 0x012), `SIMEMU_DQ250_TRACE=lo-hi` (+`_READS`, `_RAW=insns`: register
/// accesses with block PC), `SIMEMU_DQ250_DUMP` (LDRAM to file), `SIMEMU_DQ250_DTC=mask` (19 02), `SIMEMU_DQ250_GPTA`
/// (GPTA0/1 input routing and LTC configuration), SPI transfers grouped by port latches.
#[test]
#[ignore]
fn dq250_explore() {
    let name = std::env::var("SIMEMU_DQ250_IMAGE").unwrap_or_else(|_| IMAGE_4319.into());
    let mut img = image_named(&name).expect("image");
    if std::env::var("SIMEMU_DQ250_POSTPROG").is_ok() {
        println!("post-programming: {:?}", simemu_damos::apply_post_programming_dq250(&mut img));
    }
    let ms: f64 = std::env::var("SIMEMU_DQ250_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(300.0);
    let eeprom = std::env::var("SIMEMU_DQ250_EEPROM").map(|p| std::fs::read(p).unwrap()).unwrap_or_default();
    let mut b = Dq250TcuBoard::new(&img, &eeprom).expect("board");
    let hex = |s: &str| u32::from_str_radix(s.trim().trim_start_matches("0x"), 16).unwrap();
    let list = |var: &str| -> Vec<u32> { std::env::var(var).map(|l| l.split(',').map(hex).collect()).unwrap_or_default() };
    let probes = list("SIMEMU_DQ250_PROBES");
    // `SIMEMU_DQ250_RPM=input,shaft1,shaft2,output`: the speed sensors at these speeds (rest of the mechatronic at rest).
    if let Ok(v) = std::env::var("SIMEMU_DQ250_RPM") {
        let r: Vec<f64> = v.split(',').map(|x| x.parse().unwrap()).collect();
        let i = simemu_tc1791::Dq250Inputs { battery_v: 13.8, kl15: true, atf_c: 25.0, clutch_oil_c: 25.0, input_rpm: r[0], shaft_rpm: [r[1], r[2]], output_rpm: r[3], ..Default::default() };
        b.set_inputs(&i);
    }
    for spec in std::env::var("SIMEMU_DQ250_AIN").unwrap_or_default().split(',').filter(|s| !s.is_empty()) {
        let (i, v) = spec.split_once(':').unwrap();
        b.set_ain(i.parse().unwrap(), v.parse().unwrap());
    }
    for &a in &probes {
        b.cpu.add_probe(a).unwrap();
        b.cpu.add_probe(a ^ 0x2000_0000).unwrap();
    }
    let log_range = std::env::var("SIMEMU_DQ250_LOG").ok().map(|s| {
        let (lo, hi) = s.split_once('-').unwrap();
        (hex(lo), hex(hi))
    });
    if log_range.is_some() {
        *b.cpu.event_log_mut() = Some(Vec::new());
    }
    let trace_range = std::env::var("SIMEMU_DQ250_TRACE").ok().map(|s| {
        let (lo, hi) = s.split_once('-').unwrap();
        (hex(lo), hex(hi))
    });
    b.set_io_trace(trace_range, std::env::var("SIMEMU_DQ250_TRACE_READS").is_ok());
    let r = if std::env::var("SIMEMU_DQ250_CHASSIS").is_ok() {
        // Chassis modules (BCM terminal status with E2E, ESP, selector in P) every 10 ms.
        let mut enc = simemu_bridge::MqbChassisEncoder::default();
        // Virtual cluster answering 0x012 (`SIMEMU_DQ250_IMMO=key32hex:B:idkombi4hex`).
        let immo = std::env::var("SIMEMU_DQ250_IMMO").ok().map(|v| {
            let p: Vec<&str> = v.split(':').collect();
            let bytes = |h: &str| (0..h.len()).step_by(2).map(|i| u8::from_str_radix(&h[i..i + 2], 16).unwrap()).collect::<Vec<u8>>();
            let id = bytes(p[2]);
            simemu_tc1791::Wfs5ImmoPartner::from_mqb_slots_tcu(&[0; 16], &[0; 16], 0, [id[0], id[1]], [0; 2], "")
                .with_key(bytes(p[0]).try_into().unwrap(), u8::from_str_radix(p[1], 16).unwrap())
        });
        let c = simemu_bridge::ChassisInputs {
            selector_code: 8,
            drive_profile: 2,
            terminal_15: std::env::var("SIMEMU_DQ250_KL15OFF").is_err(),
            ..Default::default()
        };
        let mut r = b.run_for(0.01);
        let mut seen = 0;
        for k in 1..(ms / 10.0) as usize {
            let mut answers = Vec::new();
            for f in &b.can_tx()[seen..] {
                enc.on_tcu_frame(f.arb_id, &f.data);
                if let (0x012, Some(p)) = (f.arb_id, immo.as_ref()) {
                    answers.push(p.answer_challenge(&f.data));
                }
            }
            for data in answers {
                b.inject_can_rx(&CanFrame { node: 1, mo_index: 0, arb_id: 0x013, extended: false, dlc: 8, data, timestamp_us: 0 });
            }
            seen = b.can_tx().len();
            for (id, dlc, data) in enc.encode(&c) {
                b.inject_can_rx(&CanFrame { node: 1, mo_index: 0, arb_id: id, extended: false, dlc, data, timestamp_us: 0 });
            }
            if let Ok(nm) = std::env::var("SIMEMU_DQ250_NM") {
                // Another node's NM frame every 200 ms (`id:hex8`).
                let (id, d) = nm.split_once(':').unwrap();
                let mut data = [0u8; 8];
                for i in 0..8 {
                    data[i] = u8::from_str_radix(&d[2 * i..2 * i + 2], 16).unwrap();
                }
                if k % 20 == 0 {
                    b.inject_can_rx(&CanFrame { node: 1, mo_index: 0, arb_id: hex(id), extended: true, dlc: 8, data, timestamp_us: 0 });
                }
            }
            r = b.run_for(0.01);
        }
        r
    } else if std::env::var("SIMEMU_DQ250_KL15").is_ok() {
        // Klemmen_Status_01 0x3C0 (ZAS_Kl_S + ZAS_Kl_15, no E2E fields filled) every 100 ms.
        let f = CanFrame { node: 1, mo_index: 0, arb_id: 0x3C0, extended: false, dlc: 4, data: [0, 0, 3, 0, 0, 0, 0, 0], timestamp_us: 0 };
        let mut r = b.run_for(0.1);
        for _ in 1..(ms / 100.0) as usize {
            b.inject_can_rx(&f);
            r = b.run_for(0.1);
        }
        r
    } else {
        b.run_for(ms / 1000.0)
    };
    let pc = b.pc();
    println!("pc=0x{pc:08X} {r:?}");
    if let Ok(path) = std::env::var("SIMEMU_DQ250_DUMP") {
        std::fs::write(&path, b.cpu.read_mem(0xD000_0000, 0xE000).unwrap()).unwrap();
    }
    if let Ok(path) = std::env::var("SIMEMU_DQ250_EEPROM_OUT") {
        std::fs::write(&path, &b.periph().eeprom.mem).unwrap();
    }
    if let (Some((lo, hi)), Some(log)) = (log_range, b.cpu.event_log_mut().take()) {
        let max: usize = std::env::var("SIMEMU_DQ250_LOG_N").ok().and_then(|s| s.parse().ok()).unwrap_or(300);
        for (t, k, a, v) in log.iter().filter(|e| e.1 == 2 || (lo..=hi).contains(&e.2)).take(max) {
            println!("ev {t} {} {a:08X} {v:X}", ["R", "W", "IRQ"][*k as usize]);
        }
    }
    if let Ok(from) = std::env::var("SIMEMU_DQ250_TRACE_RAW") {
        let from: u64 = from.parse().unwrap();
        let raw: Vec<_> = b.periph().trace_log.iter().filter(|e| e.0 >= from && e.4 != 0).collect();
        for &&(n, pc, a, v, m) in raw.iter().rev().take(2000).rev() {
            println!("raw W@{n} pc {pc:08X} {a:08X} <- {v:X} m {m:X}");
        }
    }
    {
        let mut seen = std::collections::BTreeMap::new();
        for &(n, pc, a, v, m) in &b.periph().trace_log {
            let e = seen.entry((pc, a, m == 0)).or_insert((n, 0u64, v));
            e.1 += 1;
            e.2 = v;
        }
        for ((pc, a, rd), (n, c, v)) in seen {
            println!("io {} pc {pc:08X} {a:08X} first@{n} x{c} last {v:X}", if rd { "R" } else { "W" });
        }
    }
    for &a in &probes {
        for x in [a, a ^ 0x2000_0000] {
            if let Some(p) = b.cpu.probe(x).filter(|p| p.hits > 0) {
                println!("probe {x:08X}: hits {} d0-d15 {:08X?} a0-a15 {:08X?}", p.hits, &p.last_regs[..16], &p.last_regs[16..]);
            }
        }
    }
    for a in std::iter::once(pc & !0x1F).chain(list("SIMEMU_DQ250_PEEK")) {
        let m = b.cpu.read_mem(a, 32).unwrap();
        let src = img.windows(32).position(|w| w == &m[..]);
        println!("{a:08X}: {:02x?} flash_src={:?}", m, src.map(|o| format!("{:08X}", 0x8000_0000 + o)));
    }
    let hw = b.cpu.hw();
    println!(
        "ICR {:08X} irqs {} traps {} resets {:?} milestones {:?}",
        b.cpu.read_reg(unicorn_tricore_sys::TriCoreReg::ICR),
        hw.irq_dispatch_count,
        hw.trap_count,
        hw.reset_log,
        b.milestones()
    );
    println!("pending SRCs: {:X?} cpu: {:X?} | GPTA input edges applied {}", b.pending_requests(), hw.pending_cpu_request(), hw.engine_io.edges_applied);
    let srcs: Vec<String> = (0xF000_0000u32..0xF010_0600)
        .step_by(4)
        .filter(|&a| (a & 0xFF) >= 0xF0 || (0xF000_1F68..0xF000_2000).contains(&a) || (0xF000_2F68..0xF000_3000).contains(&a))
        .filter(|&a| !(0xF000_4000..0xF000_40C0).contains(&a))
        .filter_map(|a| {
            let v = b.cpu.read_mem(a, 4).ok().map(|m| u32::from_le_bytes(m[..4].try_into().unwrap()))?;
            (v & 0x1000 != 0 && v & 0xFFFF_0000 == 0).then(|| format!("{a:08X}={v:04X}"))
        })
        .collect();
    println!("enabled SRCs (SRE): {srcs:?}");
    // Call chain from the CSA list: (PCXI, saved A11, upper context?)
    use unicorn_tricore_sys::TriCoreReg as R;
    let mut pcxi = b.cpu.read_reg(R::PCXI);
    print!("PSW {:08X} A11 {:08X} chain:", b.cpu.read_reg(R::PSW), b.cpu.read_reg(R::A11));
    for _ in 0..16 {
        if pcxi & 0xF_FFFF == 0 {
            break;
        }
        let ea = ((pcxi >> 16) & 0xF) << 28 | (pcxi & 0xFFFF) << 6;
        let w: Vec<u32> = b.cpu.read_mem(ea, 64).unwrap().chunks(4).map(|c| u32::from_le_bytes(c.try_into().unwrap())).collect();
        let upper = pcxi & (1 << 22) != 0;
        // upper context: PCXI, PSW, A10, A11, ...; lower: PCXI, A11, ...
        print!(" [{}cpn{:X} ra={:08X}{}]", if upper { "U " } else { "L " }, pcxi >> 24, if upper { w[3] } else { w[1] }, if upper { format!(" psw={:08X}", w[1]) } else { String::new() });
        pcxi = w[0];
    }
    println!();
    let p = b.periph();
    println!("eeprom: cmds {:?} accesses {:?}", p.eeprom.commands, p.eeprom.log.iter().map(|(c, a, n)| format!("{}{a:04X}+{n}", if *c == 3 { 'R' } else { 'W' })).collect::<Vec<_>>());
    {
        type SpiGroupKey = (u8, u8, [u16; 6]);
        let mut g: BTreeMap<SpiGroupKey, (u64, Vec<(u32, u32)>)> = BTreeMap::new();
        for x in &p.spi_log {
            let e = g.entry((x.ssc, x.bits, x.port_out)).or_default();
            e.0 += 1;
            if e.1.len() < 12 {
                e.1.push((x.tx, x.rx));
            }
        }
        for ((ssc, bits, po), (n, sample)) in g {
            println!("spi ssc{ssc} {bits}b ports {po:04X?} x{n}: {:X?}", sample);
        }
    }
    let mut reads: Vec<_> = p.plain_reads.iter().collect();
    reads.sort_by(|a, b| b.1.cmp(a.1));
    println!("plain-register reads: {:?}", reads.iter().take(40).map(|(a, c)| format!("{a:08X}x{c}")).collect::<Vec<_>>());
    println!("plain-register writes: {:?}", p.last_written.iter().map(|(a, v)| format!("{a:08X}={v:X}")).collect::<Vec<_>>());
    let mut ids: BTreeMap<(u8, u32), (usize, [u8; 8])> = BTreeMap::new();
    for f in b.can_tx() {
        let e = ids.entry((f.node, f.arb_id)).or_insert((0, f.data));
        e.0 += 1;
        e.1 = f.data;
    }
    println!("CAN TX {} frames: {}", b.can_tx().len(), ids.iter().map(|((n, id), (c, d))| format!("n{n} {id:03X} x{c} {d:02X?}")).collect::<Vec<_>>().join("; "));
    println!("CAN NCR {:X?} NSR {:X?}", hw.can.ncr, hw.can.nsr);
    if std::env::var("SIMEMU_DQ250_GPTA").is_ok() {
        let g = &hw.engine_io.gpta;
        for k in 0..2 {
            for n in 0..32 {
                let l = g.gtc_input(k, n);
                if l != simemu_tc1791::gpta::Line::None {
                    println!("gpta{k} GTC{n:02} <- {l:?}");
                }
            }
            for n in 0..64 {
                let (l, ctr) = (g.ltc_input(k, n), g.ltc_ctr(k, n));
                if l != simemu_tc1791::gpta::Line::None || ctr != 0 {
                    println!("gpta{k} LTC{n:02} <- {l:?} ctr {ctr:04X} out {}", g.ltc_out(k, n));
                }
            }
        }
    }
    for n in 0..hw.can.num_mo {
        let m = hw.can.mo[n];
        if m.stat & 0x20 != 0 {
            let id = if m.ar & 0x2000_0000 != 0 { m.ar & 0x1FFF_FFFF } else { (m.ar >> 18) & 0x7FF };
            println!("MO{n}: list {} id {id:03X} amr {:08X} dir {} stat {:03X}", hw.can.mo_list[n], m.amr, (m.stat >> 11) & 1, m.stat);
        }
    }
    if std::env::var("SIMEMU_DQ250_DTC").is_ok() {
        let r = uds_request(&mut b, &[0x19, 0x02, u8::from_str_radix(&std::env::var("SIMEMU_DQ250_DTC").unwrap(), 16).unwrap_or(0xFF)], 2.0);
        println!("19 02 FF -> {:02X?}", r.as_ref().map(|r| r.len()));
        if let Some(r) = r {
            for d in r[3..].as_chunks::<4>().0 {
                println!("DTC {:02X}{:02X}{:02X} status {:02X}", d[0], d[1], d[2], d[3]);
            }
        }
    }
    for spec in std::env::var("SIMEMU_DQ250_UDS").unwrap_or_default().split(',').filter(|s| !s.is_empty()) {
        let parts: Vec<&str> = spec.split(':').collect();
        let mut data = [0u8; 8];
        for (i, c) in (0..parts[2].len()).step_by(2).enumerate() {
            data[i] = u8::from_str_radix(&parts[2][c..c + 2], 16).unwrap();
        }
        let f = CanFrame { node: parts[0].parse().unwrap(), mo_index: 0, arb_id: hex(parts[1]), extended: false, dlc: 8, data, timestamp_us: 0 };
        let before = b.can_tx().len();
        println!("inject {:03X} {:02X?} -> accepted by {} MO", f.arb_id, f.data, b.inject_can_rx(&f));
        let _ = b.run_for(std::env::var("SIMEMU_DQ250_UDS_WAIT_MS").ok().and_then(|s| s.parse::<f64>().ok()).unwrap_or(100.0) / 1000.0);
        for f in &b.can_tx()[before..] {
            println!("tx n{} {:03X} {:02X?}", f.node, f.arb_id, &f.data[..f.dlc as usize]);
        }
    }
}
