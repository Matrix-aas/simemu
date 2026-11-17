//! PCP2 + GPTA + crank wheel against the real SIMOS18.1 PCP image (run with `--release`).
//!
//! 1. The whole PCP image of the copy table at 0x80125B00 decodes into valid PCP2 instructions with sane
//!    control flow.
//! 2. The unmodified firmware is booted on the board core with the bench supply stimulus until it arms the
//!    crank front end itself (FPCCTR0 = 0x00930046, ~120 ms). The board's integrated PCP and GPTA (state
//!    written by the TriCore drivers: contexts, pointer lists, ENSD constants, crank start fields; PCP_CS/ICR;
//!    GPTA registers incl. the multiplexer array) and the SRC nodes are then taken over and a 60-2 + 2 cam
//!    signal is fed through GPTA0 -> SRC nodes (TOS=PCP) -> PCP channels 0x3F/0x3E/0x3A/0x3B without the CPU.

mod common;

use simemu_tc1791::crank_wheel::{CrankWheel, WheelGeometry};
use simemu_tc1791::gpta::Gpta;
use simemu_tc1791::pcp::{self, decode, FpiBus, Insn, Pcp};
use simemu_tc1791::{CpuBackend, Simos18Board};
use std::collections::{BTreeMap, HashMap};

const FLASH: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;

/// Firmware images: `$SIMEMU_IMAGE_DIR`, else the workspace root or any ancestor directory (worktrees).
fn image(name: &str) -> Option<Vec<u8>> {
    let r = common::find_image(name);
    if r.is_none() {
        eprintln!("SKIP: {name} not found");
    }
    r
}

/// Copy table at 0x80125B00: {dst, src, len, flag} × n, terminated by dst = 0 (VERIFIED-S).
fn copy_table(f: &[u8]) -> Vec<(u32, u32, u32)> {
    let w = |a: usize| u32::from_le_bytes(f[a..a + 4].try_into().unwrap());
    (0..16).map(|i| 0x12_5B00 + 16 * i).map(|o| (w(o), w(o + 4), w(o + 8))).take_while(|e| e.0 != 0).collect()
}

fn pcp_with_image(f: &[u8]) -> (Pcp, Vec<(u16, u16)>) {
    let mut p = Pcp::new();
    let tab = copy_table(f);
    p.load_copy_table(&tab, |a| f[(a - 0x8000_0000) as usize]);
    let segs = tab
        .iter()
        .filter(|e| e.0 >= pcp::CMEM_BASE)
        .map(|e| (((e.0 - pcp::CMEM_BASE) / 2) as u16, ((e.0 - pcp::CMEM_BASE + e.2) / 2) as u16))
        .collect();
    (p, segs)
}

#[test]
fn pcp_image_decodes_into_valid_instructions() {
    let Some(f) = image(FLASH) else { return };
    let (p, segs) = pcp_with_image(&f);
    assert_eq!(segs.len(), 6);
    let inside = |pc: i32| segs.iter().any(|&(a, b)| (a as i32..b as i32).contains(&pc));
    let (mut n, mut invalid, mut exits) = (0, 0, 0);
    for &(a, b) in &segs {
        let mut pc = a;
        while pc < b {
            let (w, w2) = (p.cmem[pc as usize], p.cmem[pc as usize + 1]);
            n += 1;
            let Some(i) = decode(w, w2) else {
                invalid += 1;
                pc += 1;
                continue;
            };
            let next = pc as i32 + i.len() as i32;
            let target = match i {
                Insn::Jl { off } => Some(next + off as i32),
                Insn::Jc { off, .. } => Some(next + off as i32),
                Insn::JcA { addr, .. } => Some(addr as i32),
                Insn::Exit { .. } => {
                    exits += 1;
                    None
                }
                _ => None,
            };
            if let Some(t) = target {
                assert!(inside(t), "branch at {pc:#06x} to {t:#06x}: {}", pcp::disasm(pc, w, w2));
            }
            pc = next as u16;
        }
    }
    eprintln!("PCP image: {n} instruction slots, {invalid} invalid, {exits} EXIT");
    assert_eq!(n, 5165);
    assert_eq!(invalid, 0);
    // Channel start PCs of the boot contexts (CR7[31:16], doc §4) are inside the copied code.
    for pc in [0x0450, 0x0AB5, 0x03EC, 0x14A0, 0x11C8, 0x10DA, 0x0330, 0x0225, 0x1324, 0x0F8D, 0x1888] {
        assert!(inside(pc), "{pc:#x}");
    }
}

// ------------------------------------------------------------------------------------------------

/// Board stand-in: GPTA, SRC nodes, flat memory for everything else.
struct Bus {
    g: Gpta,
    src: BTreeMap<u32, u32>,
    mem: HashMap<u32, u32>,
    /// FPI writes per address (diagnostics).
    writes: BTreeMap<u32, u64>,
    other: BTreeMap<u32, u64>,
}

const SRR: u32 = 1 << 13;

impl Bus {
    fn is_src(a: u32) -> bool {
        (0xF000_1F68..0xF000_2000).contains(&a) || (0xF000_2768..0xF000_2800).contains(&a) || (0xF000_2FC0..0xF000_3000).contains(&a)
    }
}

impl FpiBus for Bus {
    fn fpi_read(&mut self, a: u32, size: u8) -> u32 {
        if Bus::is_src(a & !3) {
            return self.src.get(&(a & !3)).copied().unwrap_or(0) >> ((a & 3) * 8);
        }
        if Gpta::contains(a) {
            return self.g.mmio_read(a, size);
        }
        *self.other.entry(a).or_default() += 1;
        self.mem.get(&(a & !3)).copied().unwrap_or(0) >> ((a & 3) * 8)
    }
    fn fpi_write(&mut self, a: u32, v: u32, size: u8) {
        *self.writes.entry(a).or_default() += 1;
        if Bus::is_src(a & !3) {
            // SETR (15) / CLRR (14) semantics, configuration bits stored
            let v = v << ((a & 3) * 8);
            let e = self.src.entry(a & !3).or_default();
            *e = (*e & !0x1CFF) | (v & 0x1CFF);
            if v & 0x8000 != 0 {
                *e |= SRR;
            }
            if v & 0x4000 != 0 {
                *e &= !SRR;
            }
            return;
        }
        if Gpta::contains(a) {
            return self.g.mmio_write(a, v, size);
        }
        *self.other.entry(a).or_default() += 1;
        self.mem.insert(a & !3, v);
    }
    fn pcp_pending(&mut self) -> Option<u8> {
        self.src.values().filter(|&&v| v & (SRR | 0x1000) == SRR | 0x1000 && (v >> 10) & 3 == 1).map(|&v| v as u8).max()
    }
    fn pcp_ack(&mut self, srpn: u8) {
        for v in self.src.values_mut() {
            if *v & SRR != 0 && (*v >> 10) & 3 == 1 && *v as u8 == srpn {
                *v &= !SRR;
            }
        }
    }
}

struct Rig {
    p: Pcp,
    bus: Bus,
    pram0: Vec<u32>,
}

/// Firmware state after boot + crank front-end init, taken over from the board's integrated GPTA/PCP
/// (`engine_io`): the PCP and GPTA are moved out and driven without the CPU.
fn rig(f: &[u8], e: &[u8], backend: CpuBackend) -> Rig {
    let mut b = Simos18Board::new_on(f, e, backend).expect("board");
    // bench supply sense (as in boot_regression.rs): ADC2.CH4 = 2.5 V
    b.set_adc_input_voltage(2, 4, 2.5).unwrap();
    let mut armed = false;
    for _ in 0..300 {
        b.step_realtime_frame(0.010).expect("run");
        if b.engine_io_mut().gpta.mmio_read(0xF000_1848, 4) == 0x0093_0046 {
            armed = true;
            break;
        }
    }
    assert!(armed, "firmware did not arm the crank front end (FPCCTR0)");
    let src: BTreeMap<u32, u32> = (0xF000_1F68..0xF000_3000).step_by(4).filter(|&a| Bus::is_src(a)).map(|a| (a, b.hw().seg_f000.get(a))).collect();
    let io = b.engine_io_mut();
    let mut g = std::mem::take(&mut io.gpta);
    let p = std::mem::take(io.pcp_mut());
    assert_eq!(p.cs, 0x21, "PCP_CS after boot (EN, EIE, resume mode, full context)");
    // the crank channel context written by the TriCore init (doc §4: CR7 0x04500940)
    assert_eq!(p.pram[0x7E0 / 4], 0x0450_0940);
    assert_eq!(p.pram[0x7C0 / 4], 0x0AB5_0940);
    // crank start fields written by 0x80089768.. (runtime block +0x68 = 1)
    assert_eq!(p.pram[0x968 / 4], 1);
    g.take_requests();
    g.take_outputs();
    let pram0 = p.pram.clone();
    Rig { p, bus: Bus { g, src, mem: HashMap::new(), writes: BTreeMap::new(), other: BTreeMap::new() }, pram0 }
}

/// Run `secs` of engine rotation at the speed profile `rpm(t)`; PCP clock `f_pcp`.
/// Returns GT0.GT1 sampled at every active crank edge.
fn spin(r: &mut Rig, w: &mut CrankWheel, t0: f64, secs: f64, theta: &mut f64, rpm: impl Fn(f64) -> f64, f_pcp: f64) -> Vec<(f64, u32)> {
    let mut gt1 = Vec::new();
    let dt = 2e-6;
    let steps = (secs / dt).round() as u64;
    for i in 1..=steps {
        let t = t0 + i as f64 * dt;
        *theta += rpm(t) * 6.0 * dt;
        let edges = w.advance(t, *theta, 0.0, 0.0);
        if edges.iter().any(|e| e.line == 24 && !e.level) {
            gt1.push((*theta, r.bus.g.gt_value(0, 1)));
        }
        let end = (t * r.bus.g.f_gpta_hz as f64).round() as u64;
        let now = r.bus.g.now();
        r.bus.g.advance(end - now, &edges);
        for a in r.bus.g.take_requests() {
            if let Some(v) = r.bus.src.get_mut(&a) {
                *v |= SRR;
            }
        }
        r.p.run((f_pcp * dt) as u64, &mut r.bus);
    }
    gt1
}

#[test]
fn crank_signal_runs_the_pcp_crank_channels() {
    let (Some(f), Some(e)) = (image(FLASH), image(EEPROM)) else { return };
    for backend in CpuBackend::matrix() {
        crank_signal_on(&f, &e, backend);
    }
}

fn crank_signal_on(f: &[u8], e: &[u8], backend: CpuBackend) {
    let mut r = rig(f, e, backend);
    let f_pcp = 180e6; // HYP: PCP clock = fCPU (TC1791 data sheet: fPCP <= 200 MHz)
    let geo = WheelGeometry::from_flash(f).unwrap();
    let mut theta = 100.0;
    let mut w = CrankWheel::new(geo, r.bus.g.f_gpta_hz as f64, 0.0, theta);
    let init = w.initial_edges();
    r.bus.g.advance(0, &init);

    // cranking at 200 rpm for 2 revolutions, then 800 rpm for 4 revolutions
    let t_crank = 2.0 * 60.0 / 200.0;
    spin(&mut r, &mut w, 0.0, t_crank, &mut theta, |_| 200.0, f_pcp);
    let teeth_crank = w.teeth_seen;
    let s3f_a = r.p.stats.starts[0x3F];
    let cam_a = r.p.stats.starts[0x3A] + r.p.stats.starts[0x3B];
    let samples = spin(&mut r, &mut w, t_crank, 4.0 * 60.0 / 800.0, &mut theta, |_| 800.0, f_pcp);

    let d: Vec<String> = samples.windows(2).rev().take(64).rev().map(|p| format!("{:.0}:{}", p[1].0 % 360.0, p[1].1.wrapping_sub(p[0].1) & 0xFF_FFFF)).collect();
    eprintln!("GT1 increments per tooth (theta mod 360 : delta): {}", d.join(" "));
    let regs: Vec<String> = [0xF000_18C0u32, 0xF000_18C4, 0xF000_18C8, 0xF000_18CC, 0xF000_18D0, 0xF000_1848, 0xF000_188C]
        .iter()
        .map(|&a| format!("{a:08x}={:#x}", r.bus.g.mmio_read(a, 4)))
        .collect();
    eprintln!("PLL/FPC0/DCM0 after run: {}", regs.join(" "));
    let starts: Vec<String> = (0..64).filter(|&c| r.p.stats.starts[c] > 0).map(|c| format!("{c:#x}:{}", r.p.stats.starts[c])).collect();
    eprintln!("teeth {} | PCP channel starts {} | instructions {} cycles {} errors {}", w.teeth_seen, starts.join(" "), r.p.stats.instructions, r.p.stats.cycles, r.p.stats.errors);
    eprintln!("PCP FPI writes (addr: count): {:x?}", r.bus.writes);
    let changed: Vec<_> = (0x900 / 4..0xB00 / 4).filter(|&i| r.p.pram[i] != r.pram0[i]).map(|i| (0xF005_0000 + 4 * i as u32, r.pram0[i], r.p.pram[i])).collect();
    eprintln!("crank PRAM changes (addr, before, after): {:x?}", changed);

    assert_eq!(r.p.stats.errors, 0, "PCP error exit, PCP_ES = {:#x}", r.p.es);
    assert!(r.bus.other.is_empty(), "PCP touched non-GPTA/SRC/PRAM addresses: {:x?}", r.bus.other);

    // 1. ch 0x3F serves the DCM0 falling-edge request of every tooth (DCMCTR0 = 0x129: FRE)
    let teeth = w.teeth_seen - teeth_crank;
    let s3f = r.p.stats.starts[0x3F] - s3f_a;
    assert!(s3f + 2 >= teeth && s3f <= teeth + 2, "ch3F {s3f} for {teeth} teeth");
    // cam channels: one start per cam edge (both edges, GTC28/GTC26 capture) — 8 edges / 720° per armed cam
    // (1 cam channel on single-VVT CJHA/CJJG `264K`, 2 cam channels on dual-VVT CJXC `259J`).
    let n_cams = [0x3Ausize, 0x3B].into_iter().filter(|&c| r.p.stats.starts[c] > 0).count();
    assert!((1..=2).contains(&n_cams), "at least one cam channel must be armed");
    let cams = r.p.stats.starts[0x3A] + r.p.stats.starts[0x3B] - cam_a;
    assert_eq!(cams, 2 * 8 * n_cams as u64, "4 revolutions = 2 cycles ({n_cams} cam channel(s))");

    // 2. tooth period measured by the PCP from the GTC31 GT0 time stamps: 800 rpm -> 1.25 ms = 7031.25
    //    GT0 ticks (5.625 MHz). Crank block +0x14 (field meaning HYP: last tooth period).
    let per = r.p.pram[0x914 / 4];
    assert!((7030..=7032).contains(&per), "tooth period {per}");
    // 3. PLL reprogrammed every tooth: PLLREV = 16 * period - PLLMTI (fGPTA = 16 * GT0 clock), PLLSTP = -256
    assert!(r.bus.writes.get(&0xF000_18D0).copied().unwrap_or(0) + 2 >= teeth, "PLLREV writes");
    assert_eq!(r.bus.g.mmio_read(0xF000_18D0, 4), 16 * per - 0x100);
    assert_eq!(r.bus.g.mmio_read(0xF000_18CC, 4), 0xFF00);
    // 4. crank filter adapted to speed: FPCCTR0.CMP = c_fac_fil_crk_tooth_act (3/16, DAMOS @0x80805A19)
    //    x period, in CLK2 ticks (GT0/4)
    let cmp = r.bus.g.mmio_read(0xF000_1848, 4) & 0xFFFF;
    assert_eq!(cmp, per * 3 / 16 / 4, "FPCCTR0.CMP");
    // 5. DCM0 tooth timeout compare = 4 tooth periods (fGPTA ticks)
    assert_eq!(r.bus.g.mmio_read(0xF000_188C, 4), 4 * 16 * per);
    // 6. angle clock: GT1 advances exactly PLLMTI = 256 micro ticks per regular tooth, and GPTA1.GT1 follows
    //    the GPTA0 PLL (CKBCTR1.DFA03 = 1)
    let regular: Vec<u32> = samples.windows(2).skip(samples.len() - 50).filter(|p| {
        let a = p[1].0.rem_euclid(360.0);
        !(250.0..320.0).contains(&a) // skip the gap and the teeth after it (sync not completed, see below)
    }).map(|p| p[1].1.wrapping_sub(p[0].1) & 0xFF_FFFF).collect();
    assert!(regular.len() > 30 && regular.iter().all(|&x| x == 256), "{regular:?}");
    assert_eq!(r.bus.g.gt_value(1, 1), r.bus.g.gt_value(0, 1));
    // 7. crank acquisition state advanced by the PCP (+0x68: 1 = started by 0x80089768 -> 2)
    assert_eq!(r.p.pram[0x968 / 4], 2);
}
