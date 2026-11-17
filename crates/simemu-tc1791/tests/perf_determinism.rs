//! Throughput + determinism check (run with `--release --nocapture`).
//!
//! Runs the SC800S50 bench image (TC1791), the DQ200_G2 image (TC1784) and the DQ250 image (TC1766)
//! for `SIMEMU_DET_MS`
//! (default 3000) ms of simulated time in 10 ms frames with fixed stimuli and prints a digest of
//! everything the firmware produced: every CAN TX frame with its timestamp, per-frame instruction,
//! reset, interrupt, trap and MMIO counters, milestones, RTOS dispatch counts, final CPU registers
//! and the contents of all guest RAMs and the DFlash. Two emulator builds execute the firmware
//! identically iff their digests are equal; backend instrumentation (translator-fix hits, quanta)
//! is reported but not digested, so the Unicorn and the native backend must give the same digest:
//! asserted when both run and the linked Unicorn has exact instruction timing (patched library,
//! ARCHITECTURE.md §3.1). `SIMEMU_DET_EXPECT_ECU` / `SIMEMU_DET_EXPECT_TCU` / `SIMEMU_DET_EXPECT_DQ250`
//! (hex) assert a Unicorn digest, the same names with `_NATIVE` a native-core digest (each backend
//! runs per `CpuBackend::matrix`, i.e. `SIMEMU_CPU`; the native run is repeated and must reproduce its
//! digest). Images are looked up like `boot_regression.rs`.

mod common;

use simemu_tc1791::{CanFrame, CpuBackend, Dq200TcuBoard, Dq250TcuBoard, Simos18Board};
use unicorn_tricore_sys::TriCoreReg;

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;
const TCU_FLASH: &str = common::DQ200_FLASH;
const TCU_EEPROM: &str = common::DQ200_EEPROM;
const DQ250_FLASH: &str = common::DQ250_012F;

fn find(name: &str) -> Option<Vec<u8>> {
    common::find_image(name)
}

/// FNV-1a 64.
struct Digest(u64);
impl Digest {
    fn bytes(&mut self, b: &[u8]) {
        for &x in b {
            self.0 = (self.0 ^ x as u64).wrapping_mul(0x100_0000_01b3);
        }
    }
    fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
}

fn frame(d: &mut Digest, f: &CanFrame) {
    d.bytes(&[f.node, f.mo_index, f.extended as u8, f.dlc]);
    d.bytes(&f.data);
    d.u64(f.arb_id as u64);
    d.u64(f.timestamp_us);
}

fn state(d: &mut Digest, b: &Simos18Board) {
    let hw = b.hw();
    for v in [hw.total_instructions, hw.reset_count as u64, hw.sw_reset_count as u64, hw.irq_dispatch_count, hw.trap_count, hw.mmio_read_count, hw.mmio_write_count, hw.stm_ticks()] {
        d.u64(v);
    }
}

fn finish(d: &mut Digest, b: &Simos18Board) {
    state(d, b);
    let hw = b.hw();
    for i in 0..16 {
        d.u64(b.read_reg(TriCoreReg::d(i)) as u64);
        d.u64(b.read_reg(TriCoreReg::a(i)) as u64);
    }
    // architectural PSW (Unicorn's raw PSW word carries stale cached status flags)
    d.u64(b.psw() as u64);
    for r in [TriCoreReg::PC, TriCoreReg::PCXI, TriCoreReg::ICR, TriCoreReg::FCX, TriCoreReg::LCX, TriCoreReg::BIV, TriCoreReg::BTV, TriCoreReg::ISP] {
        d.u64(b.read_reg(r) as u64);
    }
    for buf in [&hw.dspr, &hw.pspr, &hw.lmu, &hw.dflash] {
        d.bytes(buf);
    }
    for m in &hw.milestones {
        d.bytes(m.name.as_bytes());
        d.u64(((m.reset_cycle as u64) << 32) | m.pc as u64);
        d.u64(m.stm_ticks);
    }
    for (k, v) in hw.rtos_task_counts.iter().chain(hw.rtos_isr_counts.iter()) {
        d.u64(*k as u64);
        d.u64(*v);
    }
    for v in [hw.scu.wdt_services, hw.scu.wdt_access_errors, hw.scu.endinit_dropped, hw.flashc.commands, hw.flashc.dflash_pages_programmed, hw.flashc.dflash_erases] {
        d.u64(v);
    }
    for (k, t, n) in &hw.reset_log {
        d.u64(*k as u64);
        d.u64(*t);
        d.u64(*n);
    }
}

struct Run {
    digest: u64,
    insns: u64,
    sim: f64,
    wall: f64,
    can_tx: usize,
}

fn report(name: &str, r: &Run, b: &Simos18Board) {
    let hw = b.hw();
    println!(
        "{name}: digest {:016x} | {} insns, sim {:.3} s, wall {:.3} s -> {:.1} MIPS, RTF {:.3} | CAN TX {} | irqs {} traps {} resets {} | mmio r/w {}/{} | quanta {} (early {}) | inline acct {} exact {} | tlb fills {} | translations {} | fix sites {:?} hits {:?}",
        r.digest,
        r.insns,
        r.sim,
        r.wall,
        r.insns as f64 / r.wall / 1e6,
        r.sim / r.wall,
        r.can_tx,
        hw.irq_dispatch_count,
        hw.trap_count,
        hw.reset_count,
        hw.mmio_read_count,
        hw.mmio_write_count,
        hw.quanta,
        hw.quanta_stopped_early,
        hw.inline_block_accounting,
        hw.exact_timing,
        hw.tlb_fills,
        hw.tb_translations,
        hw.fixes.sites_by_kind,
        hw.fixes.hits
    );
}

fn check(name: &str, var: &str, digest: u64) {
    if let Ok(want) = std::env::var(var) {
        assert_eq!(format!("{digest:016x}"), want.trim(), "{name} digest differs from {var}");
    }
}

fn frames() -> u64 {
    std::env::var("SIMEMU_DET_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(3000) / 10
}

/// Run `run` on every backend of [`CpuBackend::matrix`]. Each backend has its own digest
/// (`<var>` for Unicorn, `<var>_NATIVE`); the native digest must also repeat run to run, and with
/// exact Unicorn timing both backends must agree.
fn each(name: &str, var: &str, run: impl Fn(CpuBackend) -> u64) {
    let mut digests = Vec::new();
    for backend in CpuBackend::matrix() {
        let digest = run(backend);
        if backend == CpuBackend::Native {
            check(name, &format!("{var}_NATIVE"), digest);
            assert_eq!(run(backend), digest, "{name}: native digest not reproducible");
        } else {
            check(name, var, digest);
        }
        digests.push(digest);
    }
    if digests.len() == 2 {
        let probe = Simos18Board::new_on(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], CpuBackend::Unicorn).unwrap();
        if probe.hw().exact_timing {
            assert_eq!(digests[0], digests[1], "{name}: Unicorn and native digests differ");
        } else {
            println!("{name}: linked Unicorn has no exact instruction timing, backends not compared");
        }
    }
}

fn ecu_on(flash: &[u8], eeprom: &[u8], backend: CpuBackend) -> u64 {
    let mut b = Simos18Board::new_on(flash, eeprom, backend).unwrap();
    b.set_adc_input_voltage(2, 4, 2.5).unwrap();
    let mut d = Digest(0xcbf2_9ce4_8422_2325);
    let mut r = Run { digest: 0, insns: 0, sim: 0.0, wall: 0.0, can_tx: 0 };
    for i in 0..frames() {
        if i % 10 == 5 {
            // Klemmen_Status_01-like stimulus on node 0 (delivered only if a receive MO matches).
            let f = CanFrame { node: 0, mo_index: 0, arb_id: 0x3C0, extended: false, dlc: 4, data: [0, 0, 0x03, 0, 0, 0, 0, 0], timestamp_us: 0 };
            d.u64(b.inject_can_rx(&f) as u64);
        }
        let s = b.step_realtime_frame(0.010).unwrap();
        r.insns += s.instructions;
        r.sim += s.sim_time_s;
        r.wall += s.wall_time_s;
        d.u64(i);
        state(&mut d, &b);
        for f in b.take_can_tx() {
            r.can_tx += 1;
            frame(&mut d, &f);
        }
    }
    finish(&mut d, &b);
    r.digest = d.0;
    report(&format!("ECU TC1791 SC800S50 [{backend:?}]"), &r, &b);
    r.digest
}

#[test]
fn ecu_tc1791_determinism() {
    let (Some(flash), Some(eeprom)) = (find(FACTORY), find(EEPROM)) else {
        eprintln!("SC800S50 images not found, skipping");
        return;
    };
    each("ECU", "SIMEMU_DET_EXPECT_ECU", |k| ecu_on(&flash, &eeprom, k));
}

fn tcu_on(flash: &[u8], eeprom: &[u8], backend: CpuBackend) -> u64 {
    let mut t = Dq200TcuBoard::new_on(flash, eeprom, backend).unwrap();
    t.inject_full_mechatronic_plant(13.85, 52.0, 40.0, [0.0; 4], 0.0, 0.0, 780.0, 0.0, 0.0);
    let mut d = Digest(0xcbf2_9ce4_8422_2325);
    let mut r = Run { digest: 0, insns: 0, sim: 0.0, wall: 0.0, can_tx: 0 };
    for i in 0..frames() {
        let s = t.cpu.run_for(0.010).unwrap();
        r.insns += s.instructions;
        r.sim += s.sim_time_s;
        r.wall += s.wall_time_s;
        d.u64(i);
        state(&mut d, &t.cpu);
        for f in t.cpu.take_can_tx() {
            r.can_tx += 1;
            frame(&mut d, &f);
        }
    }
    finish(&mut d, &t.cpu);
    r.digest = d.0;
    report(&format!("TCU TC1784 DQ200_G2 [{backend:?}]"), &r, &t.cpu);
    r.digest
}

#[test]
fn tcu_tc1784_determinism() {
    let (Some(flash), Some(eeprom)) = (find(TCU_FLASH), find(TCU_EEPROM)) else {
        eprintln!("DQ200_G2 images not found, skipping");
        return;
    };
    each("TCU", "SIMEMU_DET_EXPECT_TCU", |k| tcu_on(&flash, &eeprom, k));
}

fn dq250_on(flash: &[u8], backend: CpuBackend) -> u64 {
    let mut t = Dq250TcuBoard::new_on(flash, &[], backend).unwrap();
    let mut d = Digest(0xcbf2_9ce4_8422_2325);
    let mut r = Run { digest: 0, insns: 0, sim: 0.0, wall: 0.0, can_tx: 0 };
    for i in 0..frames() {
        let s = t.run_for(0.010).unwrap();
        r.insns += s.instructions;
        r.sim += s.sim_time_s;
        r.wall += s.wall_time_s;
        d.u64(i);
        state(&mut d, &t.cpu);
        for f in t.cpu.take_can_tx() {
            r.can_tx += 1;
            frame(&mut d, &f);
        }
    }
    finish(&mut d, &t.cpu);
    for (_, hits) in t.milestones() {
        d.u64(hits);
    }
    r.digest = d.0;
    report(&format!("TCU TC1766 DQ250 [{backend:?}]"), &r, &t.cpu);
    r.digest
}

#[test]
fn tcu_tc1766_determinism() {
    let Some(flash) = find(DQ250_FLASH) else {
        eprintln!("DQ250 image not found, skipping");
        return;
    };
    each("DQ250", "SIMEMU_DET_EXPECT_DQ250", |k| dq250_on(&flash, k));
}
