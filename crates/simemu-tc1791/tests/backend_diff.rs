//! CAN TX streams of the Unicorn and the native CPU backend under identical stimuli (run with
//! `--release --nocapture`). Both run the unmodified firmware on the same board models. With exact
//! Unicorn timing (patched library, ARCHITECTURE.md §3.1) the backends deliver every peripheral
//! access and interrupt at the same instruction (`backend_lockstep.rs` finds the first event that
//! differs), so the streams, boot milestones and end counters must be identical; a stock library
//! keeps block/quantum-granular timing and only the same ID set with counts within 2 % (+2 frames)
//! is asserted. The report lists per CAN ID the frame counts, the frames whose payload differs at
//! the same position and the timestamp offsets. `SIMEMU_DIFF_MS` sets the simulated time (default
//! 3000). `SIMEMU_DIFF_DUMP=<hex id>` prints both streams of one ID; `SIMEMU_DIFF_SPLIT=<ms>` runs
//! every 10 ms ECU frame in two calls (moves the frame boundaries; exact timing keeps both backends
//! equal under it too).

mod common;

use simemu_tc1791::{CanFrame, CpuBackend, Dq200TcuBoard, Simos18Board, Wfs5ImmoPartner};
use std::collections::BTreeMap;

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;
const TCU_FLASH: &str = common::DQ200_FLASH;
const TCU_EEPROM: &str = common::DQ200_EEPROM;

fn find(name: &str) -> Option<Vec<u8>> {
    common::find_image(name)
}

fn frames() -> u64 {
    std::env::var("SIMEMU_DIFF_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(3000) / 10
}

/// The ECU stimulus of `perf_determinism.rs`. Also returns the boot milestones (name, STM ticks) and
/// the end counters (instructions, interrupts, peripheral reads, writes).
fn ecu(flash: &[u8], eeprom: &[u8], backend: CpuBackend) -> (Vec<CanFrame>, Vec<(&'static str, u64)>, [u64; 4]) {
    let mut b = Simos18Board::new_on(flash, eeprom, backend).unwrap();
    b.set_adc_input_voltage(2, 4, 2.5).unwrap();
    let mut out = Vec::new();
    for i in 0..frames() {
        if i % 10 == 5 {
            let f = CanFrame { node: 0, mo_index: 0, arb_id: 0x3C0, extended: false, dlc: 4, data: [0, 0, 0x03, 0, 0, 0, 0, 0], timestamp_us: 0 };
            b.inject_can_rx(&f);
        }
        // SIMEMU_DIFF_SPLIT=<ms>: run each 10 ms frame in two calls (moves Unicorn quantum cuts)
        match std::env::var("SIMEMU_DIFF_SPLIT").ok().and_then(|s| s.parse::<f64>().ok()) {
            Some(ms) => {
                b.run_for(ms * 1e-3).unwrap();
                b.run_for(0.010 - ms * 1e-3).unwrap();
            }
            None => {
                b.step_realtime_frame(0.010).unwrap();
            }
        }
        out.extend(b.take_can_tx());
    }
    let hw = b.hw();
    assert_eq!(hw.reset_count, 0, "{backend:?}: {:?}", hw.reset_log);
    (out, hw.milestones.iter().map(|m| (m.name, m.stm_ticks)).collect(), [hw.total_instructions, hw.irq_dispatch_count, hw.mmio_read_count, hw.mmio_write_count])
}

/// The TCU stimulus of `perf_determinism.rs` plus a virtual instrument cluster answering WFS5.
fn tcu(flash: &[u8], eeprom: &[u8], backend: CpuBackend) -> (Vec<CanFrame>, [u64; 4]) {
    let mut t = Dq200TcuBoard::new_on(flash, eeprom, backend).unwrap();
    t.inject_full_mechatronic_plant(13.85, 52.0, 40.0, [0.0; 4], 0.0, 0.0, 780.0, 0.0, 0.0);
    let immo = Wfs5ImmoPartner::from_dq200_images(flash, eeprom);
    let mut out = Vec::new();
    for _ in 0..frames() {
        t.cpu.run_for(0.010).unwrap();
        for f in t.cpu.take_can_tx() {
            if f.arb_id == 0x012 {
                let a = CanFrame { node: 0, mo_index: 0, arb_id: 0x013, extended: false, dlc: 8, data: immo.answer_challenge(&f.data), timestamp_us: 0 };
                t.cpu.inject_can_rx(&a);
            }
            out.push(f);
        }
    }
    let hw = t.cpu.hw();
    assert_eq!(hw.reset_count, 0, "{backend:?}");
    (out, [hw.total_instructions, hw.irq_dispatch_count, hw.mmio_read_count, hw.mmio_write_count])
}

#[derive(Default, Debug)]
struct IdDiff {
    unicorn: usize,
    native: usize,
    /// Frames at the same position (n-th frame of the ID) with different payloads.
    payload_diffs: usize,
    first_diff: Option<(usize, [u8; 8], [u8; 8])>,
    /// Largest |t_native - t_unicorn| over the common positions (µs).
    max_dt_us: u64,
}

type FramePair<'a> = (Vec<&'a CanFrame>, Vec<&'a CanFrame>);

fn compare(name: &str, u: &[CanFrame], n: &[CanFrame]) -> BTreeMap<(u8, u32), IdDiff> {
    let mut by: BTreeMap<(u8, u32), FramePair<'_>> = BTreeMap::new();
    for f in u {
        by.entry((f.node, f.arb_id)).or_default().0.push(f);
    }
    for f in n {
        by.entry((f.node, f.arb_id)).or_default().1.push(f);
    }
    let mut out = BTreeMap::new();
    println!("== {name}: Unicorn {} frames, native {} frames", u.len(), n.len());
    for (k, (a, b)) in by {
        let mut d = IdDiff { unicorn: a.len(), native: b.len(), ..Default::default() };
        for (i, (x, y)) in a.iter().zip(&b).enumerate() {
            if x.data[..x.dlc as usize] != y.data[..y.dlc as usize] {
                d.payload_diffs += 1;
                d.first_diff.get_or_insert((i, x.data, y.data));
            }
            d.max_dt_us = d.max_dt_us.max(x.timestamp_us.abs_diff(y.timestamp_us));
        }
        println!(
            "  n{}:{:03X} count U/N {:4}/{:4}  payload diffs {:4}  max |dt| {:6} us  first diff {:02X?}",
            k.0, k.1, d.unicorn, d.native, d.payload_diffs, d.max_dt_us, d.first_diff
        );
        // SIMEMU_DIFF_DUMP=<hex id>: both streams of that ID side by side
        if std::env::var("SIMEMU_DIFF_DUMP").ok().and_then(|v| u32::from_str_radix(&v, 16).ok()) == Some(k.1) {
            for (x, y) in a.iter().zip(&b) {
                println!("    {:8} {:02X?} | {:8} {:02X?}", x.timestamp_us, &x.data[..x.dlc as usize], y.timestamp_us, &y.data[..y.dlc as usize]);
            }
        }
        out.insert(k, d);
    }
    out
}

fn assert_counts(name: &str, d: &BTreeMap<(u8, u32), IdDiff>) {
    for (k, x) in d {
        assert!(x.unicorn > 0 && x.native > 0, "{name}: ID n{}:{:03X} only on one backend ({x:?})", k.0, k.1);
        let tol = x.unicorn.max(x.native) / 50 + 2;
        assert!(x.unicorn.abs_diff(x.native) <= tol, "{name}: ID n{}:{:03X} counts {x:?}", k.0, k.1);
    }
}

/// Whether the linked Unicorn has exact instruction timing (then the backends must be identical).
fn exact() -> bool {
    Simos18Board::new_on(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], CpuBackend::Unicorn).unwrap().hw().exact_timing
}

#[test]
fn can_streams_unicorn_vs_native() {
    if !CpuBackend::Unicorn.is_available() {
        eprintln!("SKIP: built without `--features unicorn` (pure-Rust native default)");
        return;
    }
    let exact = exact();
    if let (Some(f), Some(e)) = (find(FACTORY), find(EEPROM)) {
        let ((u, mu, cu), (n, mn, cn)) = (ecu(&f, &e, CpuBackend::Unicorn), ecu(&f, &e, CpuBackend::Native));
        for ((name, tu), (_, tn)) in mu.iter().zip(&mn) {
            println!("  milestone {name:24} STM ticks U {tu:9} N {tn:9}  (N/U {:.4})", *tn as f64 / (*tu).max(1) as f64);
        }
        println!("  end counters (insns, irqs, mmio r, mmio w) U {cu:?} N {cn:?}");
        let d = compare("SIMOS18.1 SC800S50", &u, &n);
        assert_counts("SIMOS18.1", &d);
        if exact {
            assert_eq!((&mu, cu), (&mn, cn), "SIMOS18.1: milestones / counters differ");
            assert_eq!(u, n, "SIMOS18.1: CAN streams differ");
        }
    } else {
        eprintln!("SKIP: SC800S50 images not found");
    }
    if let (Some(f), Some(e)) = (find(TCU_FLASH), find(TCU_EEPROM)) {
        let ((u, cu), (n, cn)) = (tcu(&f, &e, CpuBackend::Unicorn), tcu(&f, &e, CpuBackend::Native));
        println!("  end counters (insns, irqs, mmio r, mmio w) U {cu:?} N {cn:?}");
        let d = compare("DQ200_G2", &u, &n);
        assert_counts("DQ200_G2", &d);
        if exact {
            assert_eq!(cu, cn, "DQ200_G2: counters differ");
            assert_eq!(u, n, "DQ200_G2: CAN streams differ");
        }
    } else {
        eprintln!("SKIP: DQ200_G2 images not found");
    }
}
