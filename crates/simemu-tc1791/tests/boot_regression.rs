//! Boot regression over every available SIMOS18.1 image (run with `--release --nocapture`).
//!
//! Images are looked up in `$SIMEMU_IMAGE_DIR`, `<workspace>/images`, `<workspace>/build` and the
//! workspace root; missing images are skipped. `SIMEMU_BOOT_MS` sets the simulated time per image
//! (default 3000 ms).

mod common;

use simemu_tc1791::{CpuBackend, ResetKind, Simos18Board};

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;

fn find(name: &str) -> Option<Vec<u8>> {
    common::find_image(name)
}

/// All matches of `pat` in `img`, as PFlash addresses.
fn search(img: &[u8], pat: &[u8]) -> Vec<u32> {
    img.windows(pat.len()).enumerate().filter(|(_, w)| *w == pat).map(|(i, _)| {
        let i = i as u32;
        if i < 0x20_0000 { 0x8000_0000 + i } else { 0x8080_0000 + i - 0x20_0000 }
    }).collect()
}

struct SumCheck {
    name: &'static str,
    cmp_hi: u32,
    result: u32,
}

/// Locate the firmware's own ROM/CAL 64-bit checksum comparisons by code signature
/// (`mov d3,#0 ; ne d15,d4,d2 ; or.ne d15,d5,d3` after `movh.a a15 ; ld.w d2,[a15]off`).
fn sum_checks(img: &[u8]) -> Vec<SumCheck> {
    let tail = [0x82, 0x03, 0x0b, 0x24, 0x10, 0xf1, 0x0b, 0x35, 0x80, 0xf2];
    let mut v = Vec::new();
    for a in search(img, &tail) {
        let off = |x: u32| if x >= 0x8080_0000 { (x - 0x8080_0000 + 0x20_0000) as usize } else { (x - 0x8000_0000) as usize };
        let o = off(a);
        if o < 8 {
            continue;
        }
        let movh = &img[o - 8..o - 4];
        let name = match (movh, &img[o - 4..o]) {
            ([0x91, 0x40, 0x00, 0xf8], [0x19, 0xf2, 0x40, 0x50]) => "ROM(PFlash0)",
            ([0x91, 0x00, 0x08, 0xf8], [0x19, 0xf2, 0x40, 0x00]) => "CAL(PFlash1)",
            _ => "sum",
        };
        // result: `mov d0,#1 ; mov d0,d0` follows within 0x30 bytes
        let res = img[o..(o + 0x40).min(img.len())].windows(4).position(|w| w == [0x82, 0x10, 0x02, 0x00]);
        if let Some(r) = res {
            v.push(SumCheck { name, cmp_hi: a + 2, result: a + r as u32 + 2 });
        }
    }
    v
}

struct Row {
    name: String,
    text: String,
    error: bool,
    resets: u32,
    can_tx: usize,
    irqs: u64,
    /// (evaluations, mismatch flag) of every located ROM/CAL checksum comparison.
    sums: Vec<(u64, u32)>,
}

fn boot(name: &str, flash: &[u8], eeprom: &[u8], ms: u64, backend: CpuBackend) -> Row {
    let name = &format!("{name} [{backend:?}]");
    let mut b = Simos18Board::new_on(flash, eeprom, backend).expect("board");
    // Bench stimulus: supply sense. Evidence: SC800S50 keeps CAN silent while ADC2.CH4 < ~1.0 V
    // (bisected over all 48 analog inputs); 2.5 V corresponds to a powered ECU.
    b.set_adc_input_voltage(2, 4, 2.5).unwrap();
    let checks = sum_checks(flash);
    for c in &checks {
        b.add_probe(c.cmp_hi).unwrap();
        b.add_probe(c.result).unwrap();
    }
    let mut stats = None;
    let mut err = None;
    let mut total = simemu_tc1791::RunStats::default();
    for _ in 0..ms / 10 {
        match b.step_realtime_frame(0.010) {
            Ok(s) => {
                total.instructions += s.instructions;
                total.wall_time_s += s.wall_time_s;
                total.sim_time_s += s.sim_time_s;
                stats = Some(s);
            }
            Err(e) => {
                err = Some(e);
                break;
            }
        }
    }
    let hw = b.hw();
    let pc = stats.map_or(0, |s| s.pc);
    let mut t = String::new();
    use std::fmt::Write;
    let _ = writeln!(t, "=== {} ===", name);
    if let Some(e) = &err {
        let _ = writeln!(t, "  ERROR: {}", e);
    }
    let pcnow = b.read_reg(unicorn_tricore_sys::TriCoreReg::PC);
    let _ = writeln!(
        t,
        "  sim {:.3}s in {:.2}s wall: {:.1} MIPS, RTF {:.2}; final PC 0x{:08X} ({})",
        total.sim_time_s,
        total.wall_time_s,
        total.instructions as f64 / total.wall_time_s.max(1e-9) / 1e6,
        total.sim_time_s / total.wall_time_s.max(1e-9),
        pcnow,
        Simos18Board::classify_region(pcnow)
    );
    let _ = pc;
    let resets: Vec<String> = hw.reset_log.iter().take(8).map(|(k, t, n)| format!("{:?}@{}ticks/{}insn", k, t, n)).collect();
    let _ = writeln!(t, "  resets: {} (sw {}) {:?} swrst-blocks {:08X?}", hw.reset_count, hw.sw_reset_count, resets, &hw.swrst_log[..hw.swrst_log.len().min(8)]);
    let ms_list: Vec<String> = hw.milestones.iter().take(40).map(|m| format!("r{}:{}@{:08X}", m.reset_cycle, m.name, m.pc)).collect();
    let _ = writeln!(t, "  milestones: {}", ms_list.join(" "));
    let _ = writeln!(
        t,
        "  irqs {} traps {} | rtos tasks {} isrs {} | WDT services {} access-errors {} | ENDINIT drops {} (last 0x{:08X}) csfr-drops {}",
        hw.irq_dispatch_count,
        hw.trap_count,
        hw.rtos_task_counts.len(),
        hw.rtos_isr_counts.len(),
        hw.scu.wdt_services,
        hw.scu.wdt_access_errors,
        hw.scu.endinit_dropped,
        hw.scu.last_endinit_drop,
        hw.fixes.endinit_csfr_dropped
    );
    let _ = writeln!(
        t,
        "  clocks: fCPU {} MHz (PLLCON0 0x{:08X}, PLLCON1 0x{:08X}), FPIDIV {}, STM_CLC 0x{:X} -> {} CPU cycles/STM tick; PCP-routed requests pending: {}",
        hw.scu.fcpu_hz() / 1_000_000,
        hw.scu.pllcon0,
        hw.scu.pllcon1,
        hw.scu.fpidiv(),
        hw.stm.clc,
        hw.stm.cycles_per_tick(),
        hw.pending_pcp_requests()
    );
    let mut ids: Vec<(u8, u32)> = hw.can_tx_queue.iter().map(|f| (f.node, f.arb_id)).collect();
    ids.sort_unstable();
    ids.dedup();
    let idtxt: Vec<String> = ids.iter().map(|(n, i)| format!("n{}:{:03X}", n, i)).collect();
    let _ = writeln!(t, "  CAN TX {} frames, {} IDs: {}", hw.can_tx_queue.len(), ids.len(), idtxt.join(" "));
    let _ = writeln!(
        t,
        "  translator fixes: sites {:?} hits {:?} | flash cmds {} dflash prog {} erase {} sqer {}",
        hw.fixes.sites_by_kind, hw.fixes.hits, hw.flashc.commands, hw.flashc.dflash_pages_programmed, hw.flashc.dflash_erases, hw.flashc.sequence_errors
    );
    for c in &checks {
        let p1 = b.probe(c.cmp_hi).unwrap();
        let p2 = b.probe(c.result).unwrap();
        let _ = writeln!(
            t,
            "  {} check @0x{:08X}: evaluated {}x, computed hi 0x{:08X} vs ref 0x{:08X}, mismatch flag d0={} ({} hits)",
            c.name, c.cmp_hi, p1.hits, p1.last_regs[4], p1.last_regs[2], p2.last_regs[0], p2.hits
        );
    }
    let sums = checks
        .iter()
        .map(|c| {
            let p = b.probe(c.result).unwrap();
            (p.hits, p.last_regs[0])
        })
        .collect();
    let hw = b.hw();
    Row {
        name: name.into(),
        text: t,
        error: err.is_some(),
        resets: hw.reset_count,
        can_tx: hw.can_tx_queue.len(),
        irqs: hw.irq_dispatch_count,
        sums,
    }
}

#[test]
fn boot_all_images() {
    let ms: u64 = std::env::var("SIMEMU_BOOT_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(3000);
    let Some(eeprom) = find(EEPROM) else {
        eprintln!("EEPROM dump not found, skipping");
        return;
    };
    let factory = find(FACTORY);
    let mut images: Vec<(String, Vec<u8>)> = Vec::new();
    if let Some(f) = &factory {
        images.push(("SC800S50 stock FRF (8V0906264K__0003)".into(), f.clone()));
    }
    for n in ["8V0906264H__0005.bin", "8V0906264J__0003.bin", "8V0906264F__0003.bin"] {
        if let Some(img) = find(n) {
            images.push((n.into(), img));
        }
    }
    let only = std::env::var("SIMEMU_ONLY").ok();
    let mut rows = Vec::new();
    for (name, img) in images {
        if only.as_deref().is_some_and(|o| !name.contains(o)) {
            continue;
        }
        for backend in CpuBackend::matrix() {
            let r = boot(&name, &img, &eeprom, ms, backend);
            println!("{}", r.text);
            rows.push(r);
        }
    }
    let _ = ResetKind::PowerOn;
    // The bench software (SC800S50, factory immobiliser) must boot natively: no emulator error, no
    // reset (the switch-off path check passes against the ATIC driver models), timer interrupts
    // and CAN traffic, and its own ROM/CAL checksum tests passing.
    for r in rows.iter().filter(|r| r.name.starts_with("SC800S50")) {
        assert!(!r.error, "{}", r.text);
        assert_eq!(r.resets, 0, "{}", r.text);
        assert!(r.irqs > 1000 && r.can_tx > 100, "{}", r.text);
        // The background ROM/CAL test needs ~1 s.
        if ms >= 3000 {
            assert_eq!(r.sums.len(), 2, "{}", r.text);
            assert!(r.sums.iter().all(|&(hits, flag)| hits > 0 && flag == 0), "ROM/CAL test: {}", r.text);
        }
    }
}
