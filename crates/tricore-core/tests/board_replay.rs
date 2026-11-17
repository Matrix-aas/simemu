//! Record/replay differential test: firmware on the Unicorn board (`simemu-tc1791`) is the
//! recorder, the native core replays in lockstep (one board translation block per step).
//! Design and limits: `tricore_core::replay` module docs.
//!
//! ```text
//! cargo test -p tricore-core --release --test board_replay -- --ignored --nocapture
//! SIMEMU_REPLAY_INSNS=50000000   instructions to replay (default 2000000)
//! SIMEMU_REPLAY_SKIP=N           run the board N instructions before syncing (SIMOS only)
//! SIMEMU_REPLAY_RAM_EVERY=256    compare RAM every N blocks
//! SIMEMU_REPLAY_RAM_FROM=N       compare RAM after every block from board count N on
//! SIMEMU_IMAGE_DIR=...           extra directory searched for the firmware images
//! SIMEMU_REPLAY_SHOW_KNOWN=1     print every divergence classified as a known defect
//! ```
//!
//! The board logs every CPU peripheral access (`Tc1791HardwareState::mmio_log`), so blocks that
//! read peripherals replay too: the native core gets the recorded read values and its peripheral
//! writes are checked against the board's. Only board-injected traps, resets and double interrupt
//! dispatches are resynchronised (`Sync`). Divergences that a documented Unicorn defect explains
//! ([`classify`]) are counted, resynchronised and reported; any other divergence fails the test.

use simemu_tc1791::{flash::DFLASH_BANK, Dq200TcuBoard, Simos18Board};
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::Instant;
use tricore_core::arch::{csa_ea, psw};
use tricore_core::replay::{native, play, trace_from_bytes, trace_to_bytes, ArchState, Event, ReplayBus, Replayer, TraceOp};
use tricore_core::{Arch, Cpu, Memory};
use unicorn_tricore_sys::TriCoreReg;

const FLASH: &str = "8V0906264K__0003.bin";
const EEPROM: &str = "simos18_dflash.bin";
const DQ200_FLASH: &str = "0CW300043H__0002.bin";
const DQ200_EEPROM: &str = "0CW300043H_eeprom.bin";

fn find(name: &str) -> Option<Vec<u8>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut bases = Vec::new();
    if let Some(env_dir) = std::env::var_os("SIMEMU_IMAGE_DIR") {
        bases.extend(std::env::split_paths(&env_dir));
    }
    bases.extend([root.clone(), root.join("../../..")]);
    let subdirs = [
        "",
        "vault/firmwares/ecu/bench",
        "vault/firmwares/ecu/stock",
        "vault/firmwares/ecu/custom",
        "vault/firmwares/tcu/dq200",
        "vault/firmwares/tcu/dq250",
        "vault/firmwares",
        "firmwares/ecu/bench",
        "firmwares/ecu/stock",
        "firmwares/tcu/dq200",
        "firmwares/tcu/dq250",
        "build",
    ];
    let dirs: Vec<PathBuf> = bases
        .into_iter()
        .flat_map(|b| subdirs.into_iter().map(move |s| if s.is_empty() { b.clone() } else { b.join(s) }))
        .collect();
    if let Some(exact) = dirs.iter().map(|d| d.join(name)).find(|p| p.is_file()).and_then(|p| std::fs::read(p).ok()) {
        return Some(exact);
    }
    let want: Option<(&str, &[u64])> = match name {
        EEPROM => Some(("ecu/bench", &[0x4_0000, 0x1_8000])),
        DQ200_FLASH => Some(("tcu/dq200", &[0x2A_0000])),
        DQ200_EEPROM => Some(("tcu/dq200", &[0x2000])),
        _ => None,
    };
    let (hint, sizes) = want?;
    for d in dirs {
        if !d.to_string_lossy().replace('\\', "/").ends_with(hint) {
            continue;
        }
        let Ok(rd) = std::fs::read_dir(&d) else { continue };
        let mut cands: Vec<PathBuf> = rd
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.is_file() && std::fs::metadata(p).is_ok_and(|m| sizes.contains(&m.len())))
            .collect();
        cands.sort();
        if let Some(first) = cands.into_iter().next() {
            if let Ok(b) = std::fs::read(first) {
                return Some(b);
            }
        }
    }
    if name == EEPROM {
        return std::fs::read(root.join("rom/dflash.bin")).ok();
    }
    None
}

/// The harness steps the board one translation block at a time (`run_instructions(1)` runs the
/// whole first block of a quantum only with block-granular Unicorn timing): no exact instruction
/// timing for the boards of this file (ARCHITECTURE.md §3.1; the CPU comparison does not need it).
fn block_steps() {
    std::env::set_var("SIMEMU_UC_EXACT", "0");
}

fn env_u64(k: &str, default: u64) -> u64 {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// Raw PSW word plus the folded Unicorn PSW_USB_* cache.
///
/// Unicorn 2.1.4 keeps C/V/SV/AV/SAV in separate cache registers (flag = bit 31, C = non-zero) and
/// `uc_reg_read(PSW)` returns the raw word, so fold them like QEMU's `psw_read`
/// (target/tricore/helper.c). Unlike `psw_read` this keeps raw bit 24 (PSW.RM[0]), which
/// `psw_read` masks away (0x06FFFFFF) — that defect is classified separately.
fn arch_psw(b: &Simos18Board) -> u32 {
    let f = |r, sh: u32| (b.read_reg(r) >> 31) << sh;
    (b.read_reg(TriCoreReg::PSW) & 0x07FF_FFFF)
        | ((b.read_reg(TriCoreReg::PSW_USB_C) != 0) as u32) << 31
        | f(TriCoreReg::PSW_USB_V, 30)
        | f(TriCoreReg::PSW_USB_SV, 29)
        | f(TriCoreReg::PSW_USB_AV, 28)
        | f(TriCoreReg::PSW_USB_SAV, 27)
}

fn board_state(b: &Simos18Board) -> ArchState {
    let mut s = ArchState::default();
    for i in 0..16 {
        s.d[i] = b.read_reg(TriCoreReg::d(i as u32));
        s.a[i] = b.read_reg(TriCoreReg::a(i as u32));
    }
    s.pc = b.read_reg(TriCoreReg::PC);
    s.psw = arch_psw(b);
    s.pcxi = b.read_reg(TriCoreReg::PCXI);
    s.fcx = b.read_reg(TriCoreReg::FCX);
    s.lcx = b.read_reg(TriCoreReg::LCX);
    s.icr = b.read_reg(TriCoreReg::ICR);
    s.isp = b.read_reg(TriCoreReg::ISP);
    s.btv = b.read_reg(TriCoreReg::BTV);
    s.biv = b.read_reg(TriCoreReg::BIV);
    s
}

/// Native memory with the TC1791 map of `Simos18Board::map_memory`: 4 MB PFlash (two banks,
/// cached/non-cached views), DFlash banks, BROM, DSPR, PSPR, LMU.
fn simos_layout(flash: Vec<u8>, dflash: Vec<u8>, brom: Vec<u8>, dspr: Vec<u8>, pspr: Vec<u8>, lmu: Vec<u8>) -> Memory {
    let mut m = Memory::new();
    let fl = m.add_owned(flash, false);
    for base in [0x8000_0000u32, 0xA000_0000] {
        m.map(base, fl, 0, 0x20_0000);
        m.map(base + 0x80_0000, fl, 0x20_0000, 0x20_0000);
    }
    let df = m.add_owned(dflash, false);
    m.map(0xAF00_0000, df, 0, DFLASH_BANK);
    m.map(0xAF08_0000, df, DFLASH_BANK, DFLASH_BANK);
    let n = brom.len();
    let br = m.add_owned(brom, false);
    m.map(0xAFFF_C000, br, 0, n);
    m.map(0x8FFF_C000, br, 0, n);
    for (base, v) in [(0xD000_0000u32, dspr), (0xC000_0000, pspr)] {
        let n = v.len();
        let r = m.add_owned(v, true);
        m.map(base, r, 0, n);
    }
    let n = lmu.len();
    let lm = m.add_owned(lmu, true);
    m.map(0xB000_0000, lm, 0, n);
    m.map(0x9000_0000, lm, 0, n);
    m
}

/// Copies of the board's buffers.
fn simos_memory(b: &Simos18Board) -> Memory {
    let hw = b.hw();
    simos_layout(hw.flash.to_vec(), hw.dflash.to_vec(), hw.brom.to_vec(), hw.dspr.to_vec(), hw.pspr.to_vec(), hw.lmu.to_vec())
}

/// The flash image and zeroed RAM (the trace's initial `Mem` ops fill RAM, DFlash and BROM).
fn simos_blank(flash: &[u8]) -> Memory {
    simos_layout(flash[..0x40_0000].to_vec(), vec![0; 2 * DFLASH_BANK], vec![0; 0x4000], vec![0; 0x2_0000], vec![0; 0x8000], vec![0; 0x2_0000])
}

fn simos_ram(b: &Simos18Board) -> Vec<(u32, &[u8])> {
    let hw = b.hw();
    let (b0, b1) = hw.dflash.split_at(DFLASH_BANK);
    vec![(0xD000_0000, &hw.dspr[..]), (0xC000_0000, &hw.pspr[..]), (0xB000_0000, &hw.lmu[..]), (0xAF08_0000, &b1[..DFLASH_BANK]), (0xAF00_0000, b0)]
}

/// Native memory with the TC1784 map of `Simos18Board::map_memory` (BoardVariant::Dq200Tc1784),
/// including the zero-filled `uc_mem_map` RAM windows the board puts over unmodelled address space.
fn dq200_memory(b: &Simos18Board) -> Memory {
    let hw = b.hw();
    let mut m = Memory::new();
    let fl = m.add_owned(hw.flash.to_vec(), false);
    m.map(0x8000_0000, fl, 0, 0x40_0000);
    m.map(0xA000_0000, fl, 0, 0x40_0000);
    let df = m.add_owned(hw.dflash.to_vec(), false);
    m.map(0xAF00_0000, df, 0, hw.dflash.len());
    m.map(0x8F00_0000, df, 0, hw.dflash.len());
    let br = m.add_owned(hw.brom.to_vec(), false);
    m.map(0xAFFF_0000, br, 0, hw.brom.len());
    m.map(0x8FFF_0000, br, 0, hw.brom.len());
    let ds = m.add_owned(hw.dspr.to_vec(), true);
    m.map(0xD000_0000, ds, 0, hw.dspr.len());
    let ps = m.add_owned(hw.pspr.to_vec(), true);
    m.map(0xD400_0000, ps, 0, hw.pspr.len());
    m.map(0xC000_0000, ps, 0, hw.pspr.len());
    let lm = m.add_owned(hw.lmu.to_vec(), true);
    m.map(0xB000_0000, lm, 0, hw.lmu.len());
    m.map(0x9000_0000, lm, 0, hw.lmu.len());
    for (a, l) in [(0x0000_0000u32, 0x1_0000usize), (0xF040_0000, 0x07A0_0000), (0xF810_0000, 0x006F_0000), (0xF880_0000, 0x0780_0000)] {
        let r = m.add_owned(vec![0; l], true);
        m.map(a, r, 0, l);
    }
    m
}

fn dq200_ram(b: &Simos18Board) -> Vec<(u32, &[u8])> {
    let hw = b.hw();
    vec![(0xD000_0000, &hw.dspr[..]), (0xD400_0000, &hw.pspr[..]), (0xB000_0000, &hw.lmu[..]), (0xAF00_0000, &hw.dflash[..])]
}

/// A divergence explained by a documented Unicorn 2.1.4 / board defect. `None` = unexplained.
/// `before` is the (agreed) state before the block, `board`/`native` the states after it.
fn classify(before: &ArchState, board: &ArchState, native: &ArchState) -> Option<&'static str> {
    let diff = board.diff(native);
    let only = |names: &[&str]| diff.iter().all(|(n, _, _)| names.contains(&n.as_str()));
    if diff.is_empty() {
        return None;
    }
    // QEMU psw_read() (target/tricore/helper.c) clears PSW with 0x06FFFFFF before folding the
    // cached status bits, which also drops PSW[24] (RM[0], TC1.6 FPU rounding mode). Every CALL,
    // RET, RFE, SVLCX, BISR, MFCR PSW and trap entry writes the result back. The manual keeps it
    // (V2 CALL p. 3-63, RET p. 3-311 keeps PSW[25:24]).
    if only(&["PSW"]) && board.psw ^ native.psw == 1 << 24 && native.psw & 1 << 24 != 0 {
        return Some("U1 psw_read() masks PSW with 0x06FFFFFF: PSW[24] (RM[0]) cleared");
    }
    // Board interrupt entry (simemu-tc1791 stm_icu.rs dispatch_pending_irq) sets PSW.CDE; V1 §5.3
    // (p. 5-7) leaves it unchanged (only trap entry, §6.2.5, sets CDE).
    if only(&["PSW"]) && board.psw ^ native.psw == psw::CDE && board.psw & psw::CDE != 0 && before.icr != board.icr {
        return Some("B1 board interrupt entry sets PSW.CDE (V1 5.3 leaves it)");
    }
    None
}

/// (moves or transfers so far, destination address) of every board DMA channel (SIMOS SscDma and
/// TC1784 DMA models, both public). DMA moves are board-side RAM writes the native core cannot see.
fn dma_state(b: &Simos18Board) -> Vec<(u64, u32)> {
    let hw = b.hw();
    let mut v: Vec<(u64, u32)> = hw.spi.dma.ch.iter().map(|c| (c.transfers, c.dadr)).collect();
    v.extend(hw.tc1784.dma.ch.iter().map(|c| (c.moves, c.dadr)));
    v
}

/// Guest page (64 KB) of a DMA destination address, as the native core maps it. TC1784 DMA
/// reaches DSPR through its global address 0xE84xxxxx as well (`tc1784_io::dma_ram`).
fn dma_page(addr: u32) -> u32 {
    let addr = if addr >> 20 == 0xE84 { 0xD000_0000 | (addr & 0xF_FFFF) } else { addr };
    addr & !0xFFFF
}

/// After the native core ran the block: take the board's bytes of `page` where the native CPU did
/// not write (`pre` = native page before the block). Those are the board DMA's writes; bytes both
/// sides wrote are left for the RAM comparison. Returns the written runs (address, bytes).
fn merge_dma_page(cpu: &mut Cpu<ReplayBus>, ram: &[(u32, &[u8])], page: u32, pre: &[u8]) -> Vec<(u32, Vec<u8>)> {
    let Some((base, bytes)) = ram.iter().find(|(a, r)| page >= *a && (page - a) < r.len() as u32) else { return Vec::new() };
    let off = (page - base) as usize;
    let board = &bytes[off..(off + pre.len()).min(bytes.len())];
    let mut post = vec![0u8; board.len()];
    cpu.read_bytes(page, &mut post);
    let mut runs: Vec<(u32, Vec<u8>)> = Vec::new();
    for i in 0..board.len() {
        if post[i] == pre[i] && board[i] != post[i] {
            let a = page + i as u32;
            match runs.last_mut() {
                Some((s, v)) if *s + v.len() as u32 == a => v.push(board[i]),
                _ => runs.push((a, vec![board[i]])),
            }
        }
    }
    for (a, v) in &runs {
        cpu.write_bytes(*a, v).unwrap();
    }
    runs
}

/// 4 KB chunks of `ram` (board) that differ from native memory.
fn diff_chunks(cpu: &Cpu<ReplayBus>, ram: &[(u32, &[u8])]) -> Vec<(u32, Vec<u8>)> {
    let mut out = Vec::new();
    let mut have = [0u8; 4096];
    for (a, r) in ram {
        for (i, w) in r.chunks(4096).enumerate() {
            let x = a + (i * 4096) as u32;
            if !cpu.read_bytes(x, &mut have[..w.len()]) || &have[..w.len()] != w {
                out.push((x, w.to_vec()));
            }
        }
    }
    out
}

/// Unicorn defects that are decided by the encoding alone: the first such instruction among the
/// `n` instructions the board executed from `start` (a Unicorn TB is straight-line code).
fn defect_site(cpu: &Cpu<ReplayBus>, start: u32, n: u64) -> Option<(&'static str, u32)> {
    let rm = (cpu.psw >> 24) & 3;
    let mut pc = start;
    for _ in 0..n {
        let mut b = [0u8; 4];
        if !cpu.read_bytes(pc, &mut b[..2]) {
            return None;
        }
        cpu.read_bytes(pc + 2, &mut b[2..]);
        let w = u32::from_le_bytes(b);
        let lo = w as u16;
        // SRO LD.H D[15], [A[b]]off4 (op1 8CH): EA = A[b] + zero_ext(2 * off4) (V2 p. 3-160);
        // Unicorn translate.c decode_sro_opc passes `address` unscaled for OPC1_16_SRO_LD_H.
        if lo & 0xFF == 0x8C && (lo >> 8) & 0xF != 0 {
            return Some(("U3 SRO LD.H D15,[A[b]]off4 uses off4 instead of 2*off4 (V2 p. 3-160)", pc));
        }
        // UPDFL (RR 4BH op2 0CH): PSW[31:24] = (PSW[31:24] & ~D[a][15:8]) | (D[a][7:0] & D[a][15:8])
        // (V2 UPDFL). Unicorn's helper_updfl (fpu_helper.c) never updates RM and clears the
        // flags outside the mask.
        if lo & 0xFF == 0x4B && (w >> 20) & 0xFF == 0x0C {
            return Some(("U5 UPDFL ignores the RM mask bits and clears unmasked flags", pc));
        }
        // RET (SR 9000H, SYS 0DH op2 06H) keeps PSW[25:24] (V2 RET p. 3-311); Unicorn's
        // helper_ret tests TRICORE_FEATURE_13 (set on every model) and restores RM from the CSA.
        if lo == 0x9000 || w & 0x0FC0_00FF == 0x0180_000D {
            return Some(("U6 RET restores PSW[25:24] (RM) from the CSA", pc));
        }
        // FPU ops (RR 4BH, RRR 6BH) round per PSW.RM (V1 11.3.2). Unicorn's fpu_set_state passes
        // `PSW & 0x03000000` unshifted as the softfloat rounding mode (and TriCore RM 01/10 =
        // +inf/-inf are softfloat 2/1), so every RM != 00 lands in round_canonical's
        // g_assert_not_reached() default (a no-op in release builds).
        if rm != 0 && matches!(lo & 0xFF, 0x4B | 0x6B) {
            return Some(("U4 FPU rounding with PSW.RM != 00 (fpu_set_state passes RM unshifted)", pc));
        }
        pc = pc.wrapping_add(tricore_core::fmt::insn_len(lo));
    }
    None
}

/// The word at `addr` in the board's RAM ranges.
fn ram_word(ram: &[(u32, &[u8])], addr: u32) -> Option<u32> {
    let (base, r) = ram.iter().find(|(a, r)| addr >= *a && addr - a + 4 <= r.len() as u32)?;
    let o = (addr - base) as usize;
    Some(u32::from_le_bytes(r[o..o + 4].try_into().unwrap()))
}

#[derive(Default)]
struct Report {
    known: BTreeMap<&'static str, (u64, u32, u64)>,
    taints: u64,
    pipn_mirrored: u64,
    dma_merges: u64,
    /// B3 overcount per block start address.
    b3_sites: BTreeMap<u32, u64>,
    insns: u64,
    native_insns: u64,
    board_s: f64,
    native_s: f64,
}

/// Trace recording options of [`record_replay`].
struct TraceRec<'t> {
    ops: &'t mut Vec<TraceOp>,
    /// Emit a register checkpoint every N board blocks (0: none, free-running benchmark trace).
    check_every: u64,
    /// Read-only images the native memory of a blank player lacks (BROM stub): (base, bytes).
    extra_init: Vec<(u32, Vec<u8>)>,
}

type RamFn = fn(&Simos18Board) -> Vec<(u32, &[u8])>;

/// Lockstep record/replay of `insns` instructions. Returns the report or the first unexplained
/// divergence. With `trace`, also records a [`TraceOp`] stream that re-executes the same run.
fn record_replay(
    board: &mut Simos18Board,
    cpu: &mut Cpu<ReplayBus>,
    insns: u64,
    ram_of: RamFn,
    mut trace: Option<TraceRec>,
) -> Result<(Report, Replayer), String> {
    let ram_every = env_u64("SIMEMU_REPLAY_RAM_EVERY", 256).max(1);
    // compare RAM after every block from this board instruction count on (bisecting)
    let ram_from = env_u64("SIMEMU_REPLAY_RAM_FROM", u64::MAX);
    let mut rp = Replayer::new();
    let mut rep = Report::default();
    // native instructions executed = cpu.icount - adj (the board's counter jumps, see B3)
    let mut adj = board.hw().total_instructions;
    macro_rules! emit {
        ($($op:expr),*) => { if let Some(t) = trace.as_mut() { $( t.ops.push($op); )* } };
    }
    macro_rules! sync {
        ($st:expr) => {{
            let b: &Simos18Board = board;
            let ram = ram_of(b);
            if trace.is_some() {
                let chunks = diff_chunks(cpu, &ram);
                emit!(TraceOp::RunTo(cpu.icount - adj), TraceOp::State($st));
                for (a, c) in chunks {
                    emit!(TraceOp::Mem(a, c));
                }
            }
            adj += b.hw().total_instructions - cpu.icount;
            rp.step(cpu, &Event::Sync { icount: b.hw().total_instructions, state: $st, ram: &ram }).unwrap();
        }};
    }
    {
        let st = board_state(board);
        let ram = ram_of(board);
        if let Some(t) = trace.as_mut() {
            t.ops.push(TraceOp::State(st));
            for (a, r) in ram.iter().map(|(a, r)| (*a, r.to_vec())).chain(t.extra_init.drain(..)) {
                for (i, c) in r.chunks(4096).enumerate() {
                    if c.iter().any(|&x| x != 0) {
                        t.ops.push(TraceOp::Mem(a + (i * 4096) as u32, c.to_vec()));
                    }
                }
            }
        }
        rp.step(cpu, &Event::Sync { icount: board.hw().total_instructions, state: st, ram: &ram }).unwrap();
        cpu.flush_code_cache();
    }
    *board.hw().mmio_log.borrow_mut() = Some(Vec::new());
    let start = board.hw().total_instructions;
    let mut k = 0u64;
    while board.hw().total_instructions - start < insns {
        k += 1;
        let hw = board.hw();
        let (t0, irqs0, traps0, resets0) = (hw.total_instructions, hw.irq_dispatch_count, hw.trap_count, hw.reset_count);
        let fixes0: u64 = hw.fixes.hits.iter().sum();
        let dma0 = dma_state(board);
        let before = ArchState::of(cpu);
        cpu.bus.endinit = hw.scu.endinit;
        let tb = Instant::now();
        board.run_instructions(1).map_err(|e| format!("board step: {e}"))?;
        rep.board_s += tb.elapsed().as_secs_f64();
        let hw = board.hw();
        let log = hw.mmio_log.borrow_mut().as_mut().map(std::mem::take).unwrap_or_default();
        let irqs = hw.irq_dispatch_count - irqs0;
        if hw.trap_count != traps0 || hw.reset_count != resets0 || irqs > 1 {
            rep.taints += 1;
            sync!(board_state(board));
            continue;
        }
        let tn = Instant::now();
        let dma1 = dma_state(board);
        // Board DMA wrote RAM during this step (at the boundary or from an MMIO write inside the
        // block): snapshot the destination pages before the native block, merge after it.
        let mut dma_pages: Vec<(u32, Vec<u8>)> = Vec::new();
        for (x, y) in dma0.iter().zip(&dma1).filter(|(x, y)| x.0 != y.0) {
            for p in [dma_page(x.1), dma_page(y.1)] {
                if !dma_pages.iter().any(|(q, _)| *q == p) {
                    let mut pre = vec![0u8; 0x1_0000];
                    if cpu.read_bytes(p, &mut pre) {
                        dma_pages.push((p, pre));
                    }
                }
            }
        }
        if irqs == 1 {
            let pipn = ((hw.last_tb_pc.wrapping_sub(cpu.biv)) >> 5) as u8;
            emit!(TraceOp::RunTo(cpu.icount - adj), TraceOp::Irq(pipn));
            rp.step(cpu, &Event::Irq { icount: t0, pipn }).map_err(|d| format!("{d:?}"))?;
            // B2: board interrupt entry saved the raw Unicorn PSW word (stale C/V/SV/AV/SAV, see
            // arch_psw) into the CSA; RFE later restored those stale flags into the interrupted
            // code. Compare with what the board saved; adopt it if it differs.
            let ea = csa_ea(cpu.pcxi) + 4;
            let native_saved = cpu.ld32(ea).unwrap();
            let board_saved = ram_word(&ram_of(board), ea).expect("CSA in board RAM");
            if native_saved != board_saved {
                cpu.st32(ea, board_saved).unwrap();
                emit!(TraceOp::Mem(ea, board_saved.to_le_bytes().to_vec()));
                let e = rep.known.entry("B2 board interrupt entry saves raw PSW (stale USB flags) in the CSA").or_insert((0, before.pc, t0));
                e.0 += 1;
            }
        }
        for &(w, addr, size, value, endinit) in &log {
            emit!(TraceOp::Access { write: w, addr, size, value, endinit });
            let ev = if w {
                Event::Write { icount: t0, addr, size, value, endinit }
            } else {
                Event::Read { icount: t0, addr, size, value, endinit }
            };
            rp.step(cpu, &ev).unwrap();
        }
        let flash_written = log.iter().any(|&(w, a, ..)| w && matches!(a >> 28, 0x8 | 0xA));
        // the instructions the board just executed start at the native PC (after any interrupt entry)
        let site = defect_site(cpu, cpu.pc, hw.total_instructions - t0);
        let ram = if k.is_multiple_of(ram_every) || t0 >= ram_from || site.is_some() { ram_of(board) } else { Vec::new() };
        let board_st = board_state(board);
        // ICR.PIPN is the board ICU's arbitration output, republished at the boundary that starts
        // every step; it is not CPU state. Mirror the board's value before the native block runs
        // (MFCR ICR inside the block reads it). If the board's PIPN is not what its ICU published,
        // an MTCR ICR in the block has overwritten the read-only field (U2); that value is then
        // mirrored, and the native MTCR leaves PIPN alone, so both agree after the block.
        let pipn = ((board_st.icr >> 16) & 0xFF) as u8;
        if pipn != hw.pipn_published {
            rep.known.entry("U2 MTCR ICR writes the read-only ICR.PIPN").or_insert((0, before.pc, t0)).0 += 1;
        }
        if (cpu.icr >> 16) as u8 != pipn {
            rep.pipn_mirrored += 1;
            emit!(TraceOp::RunTo(cpu.icount - adj), TraceOp::Pipn(pipn));
            cpu.set_pipn(pipn);
        }
        if hw.fixes.hits.iter().sum::<u64>() != fixes0 {
            // An isa_fix hook executed the instruction and wrote PC: Unicorn abandons the rest of
            // the translation block, but hook_block already counted all of it. Run the native core
            // to the board's PC and adopt the board's (over)count.
            let n = hw.total_instructions - cpu.icount;
            let mut k = 0;
            while cpu.pc != board_st.pc && k < n {
                cpu.run(1);
                k += 1;
            }
            if k < n {
                let e = rep.known.entry("B3 isa_fix hook writes PC mid-TB: board counts the abandoned rest of the TB").or_insert((0, before.pc, t0));
                e.0 += n - k;
                *rep.b3_sites.entry(before.pc).or_insert(0) += n - k;
            }
            adj += hw.total_instructions - cpu.icount;
            cpu.icount = hw.total_instructions;
        }
        let ev = Event::Check { icount: hw.total_instructions, state: board_st, ram: &ram };
        let mut merged: Vec<(u32, Vec<u8>)> = Vec::new();
        let r = (|| {
            Replayer::run_to(cpu, hw.total_instructions)?;
            if !dma_pages.is_empty() {
                let ram = ram_of(board);
                for (p, pre) in &dma_pages {
                    let runs = merge_dma_page(cpu, &ram, *p, pre);
                    rep.dma_merges += !runs.is_empty() as u64;
                    merged.extend(runs);
                }
            }
            rp.step(cpu, &ev)
        })();
        if !merged.is_empty() {
            emit!(TraceOp::RunTo(cpu.icount - adj));
            for (a, v) in merged {
                emit!(TraceOp::Mem(a, v));
            }
        }
        if let Err(d) = r {
            match classify(&before, &board_st, &d.native).or(site.map(|s| s.0)) {
                Some(name) => {
                    let e = rep.known.entry(name).or_insert((0, before.pc, t0));
                    e.0 += 1;
                    if std::env::var("SIMEMU_REPLAY_SHOW_KNOWN").is_ok() {
                        let mut bytes = [0u8; 24];
                        cpu.read_bytes(before.pc, &mut bytes);
                        eprintln!("{name}: block {:#010x} (bytes {bytes:02x?}), site {site:x?}: {}\n  before {before:x?}", before.pc, d.what);
                    }
                    sync!(board_st);
                }
                None => {
                    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../build");
                    for (a, r) in ram_of(board) {
                        let _ = std::fs::write(dir.join(format!("replay_dump_{a:08x}.bin")), r);
                    }
                    let mut bytes = [0u8; 16];
                    cpu.read_bytes(before.pc, &mut bytes);
                    return Err(format!(
                        "divergence at board block {k} pc {:#010x} (bytes {bytes:02x?}), icount {} -> {}: {}\nmmio {:x?}\nbefore {:x?}\nboard  {:x?}\nnative {:x?}\nstats {:?}",
                        before.pc, t0, d.icount, d.what, log, before, board_st, d.native, rp.stats
                    ));
                }
            }
        } else if trace.as_ref().is_some_and(|t| t.check_every > 0 && k.is_multiple_of(t.check_every)) {
            emit!(TraceOp::RunTo(cpu.icount - adj), TraceOp::Check(board_st));
        }
        if flash_written {
            // Flash programming completes inside the board's store hook: take the new DFlash.
            let ram: Vec<(u32, &[u8])> = ram_of(board).into_iter().filter(|r| r.0 >> 24 == 0xAF).collect();
            let chunks = diff_chunks(cpu, &ram);
            if !chunks.is_empty() {
                emit!(TraceOp::RunTo(cpu.icount - adj));
            }
            for (a, c) in chunks {
                cpu.write_bytes(a, &c).map_err(|e| format!("dflash sync at {e:#x}"))?;
                emit!(TraceOp::Mem(a, c));
            }
        }
        rep.native_s += tn.elapsed().as_secs_f64();
    }
    emit!(TraceOp::RunTo(cpu.icount - adj), TraceOp::Check(ArchState::of(cpu)));
    rep.insns = board.hw().total_instructions - start;
    rep.native_insns = cpu.icount - adj;
    Ok((rep, rp))
}

fn print_report(name: &str, rep: &Report, rp: &Replayer) {
    eprintln!(
        "{name}: {} instructions, {} verified exactly, {} blocks resynchronised (board traps/resets), ICR.PIPN mirrored {}x, DMA pages merged {}x, stats {:?}",
        rep.insns, rp.stats.verified_insns, rep.taints, rep.pipn_mirrored, rep.dma_merges, rp.stats
    );
    eprintln!("{name}: board {:.1} s, native+compare {:.1} s", rep.board_s, rep.native_s);
    for (k, (n, pc, ic)) in &rep.known {
        eprintln!("{name}: known defect x{n} (first at pc {pc:#010x}, icount {ic}): {k}");
    }
    let mut b3: Vec<_> = rep.b3_sites.iter().collect();
    b3.sort_by(|a, b| b.1.cmp(a.1));
    eprintln!("{name}: B3 hottest blocks (pc, overcounted instructions): {:x?}", &b3[..b3.len().min(8)]);
}

#[test]
#[ignore = "needs the SIMOS18.1 images; run with --ignored"]
fn simos18_boot_replays_on_native_core() {
    let (Some(flash), Some(eeprom)) = (find(FLASH), find(EEPROM)) else {
        eprintln!("SKIP: firmware images not found");
        return;
    };
    block_steps();
    let mut board = Simos18Board::new_on(&flash, &eeprom, simemu_tc1791::CpuBackend::Unicorn).expect("board");
    let skip = env_u64("SIMEMU_REPLAY_SKIP", 0);
    if skip > 0 {
        board.run_instructions(skip).expect("skip");
    }
    let mut cpu = native(Arch::Tc16, simos_memory(&board));
    let insns = env_u64("SIMEMU_REPLAY_INSNS", 2_000_000);
    match record_replay(&mut board, &mut cpu, insns, simos_ram, None) {
        Ok((rep, rp)) => print_report("SIMOS18.1", &rep, &rp),
        Err(e) => panic!("{e}"),
    }
    let hw = board.hw();
    let names: Vec<&str> = hw.milestones.iter().map(|m| m.name).collect();
    eprintln!("SIMOS18.1: board milestones {names:?}");
    eprintln!("SIMOS18.1: RTOS task dispatches {}, ISR dispatches {}", hw.rtos_task_counts.values().sum::<u64>(), hw.rtos_isr_counts.values().sum::<u64>());
}

#[test]
#[ignore = "needs the DQ200_G2 images; run with --ignored"]
fn dq200_boot_replays_on_native_core() {
    let (Some(flash), Some(eeprom)) = (find(DQ200_FLASH), find(DQ200_EEPROM)) else {
        eprintln!("SKIP: firmware images not found");
        return;
    };
    block_steps();
    let mut tcu = Dq200TcuBoard::new_on(&flash, &eeprom, simemu_tc1791::CpuBackend::Unicorn).expect("board");
    let board = &mut tcu.cpu;
    let mut cpu = native(Arch::Tc131, dq200_memory(board));
    let insns = env_u64("SIMEMU_REPLAY_INSNS", 2_000_000);
    match record_replay(board, &mut cpu, insns, dq200_ram, None) {
        Ok((rep, rp)) => print_report("DQ200_G2", &rep, &rp),
        Err(e) => panic!("{e}"),
    }
    let names: Vec<&str> = board.hw().milestones.iter().map(|m| m.name).collect();
    eprintln!("DQ200_G2: board milestones {names:?}, CAN frames sent {}", board.hw().can_tx_queue.len());
}

const SEGMENT_NAME: &str = "simos18_segment.trace";

/// Records the regression segment for [`simos18_recorded_segment_replays`]: the SIMOS18.1 FACTORY
/// image from SIMEMU_REPLAY_SKIP (default 3 550 000, just before the first interrupts and DMA
/// transfers) for SIMEMU_REPLAY_INSNS (default 600 000) board instructions, with a register
/// checkpoint every 128 blocks.
#[test]
#[ignore = "re-records vault/firmwares/ecu/bench/simos18_segment.trace; needs the SIMOS18.1 images"]
fn record_simos18_segment() {
    let (Some(flash), Some(eeprom)) = (find(FLASH), find(EEPROM)) else {
        eprintln!("SKIP: firmware images not found");
        return;
    };
    block_steps();
    let mut board = Simos18Board::new_on(&flash, &eeprom, simemu_tc1791::CpuBackend::Unicorn).expect("board");
    board.run_instructions(env_u64("SIMEMU_REPLAY_SKIP", 3_550_000)).expect("skip");
    let mut cpu = native(Arch::Tc16, simos_memory(&board));
    let mut ops = Vec::new();
    let brom = board.hw().brom.to_vec();
    let rec = TraceRec { ops: &mut ops, check_every: 128, extra_init: vec![(0xAFFF_C000, brom)] };
    let (rep, rp) = record_replay(&mut board, &mut cpu, env_u64("SIMEMU_REPLAY_INSNS", 600_000), simos_ram, Some(rec)).unwrap();
    print_report("SIMOS18.1 segment", &rep, &rp);
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../vault/firmwares/ecu/bench")
        .join(SEGMENT_NAME);
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    let bytes = trace_to_bytes(&ops);
    std::fs::write(&path, &bytes).unwrap();
    eprintln!("wrote {} ops, {} bytes to {}", ops.len(), bytes.len(), path.display());
}

/// Regression: replays the recorded SIMOS18.1 segment (interrupt entries, DMA-fed RAM, MMIO-driven
/// boot code, known-defect resyncs) on the native core alone and checks every register checkpoint
/// and the peripheral access sequence. Needs only the flash image and recorded segment in `vault/`.
#[test]
fn simos18_recorded_segment_replays() {
    let (Some(flash), Some(bytes)) = (find(FLASH), find(SEGMENT_NAME)) else {
        eprintln!("SKIP: flash image or {SEGMENT_NAME} not found");
        return;
    };
    let ops = trace_from_bytes(&bytes).unwrap();
    let checks = ops.iter().filter(|o| matches!(o, TraceOp::Check(_))).count();
    let mut cpu = native(Arch::Tc16, simos_blank(&flash));
    let n = play(&mut cpu, &ops).unwrap_or_else(|d| panic!("segment diverged: {d:?}"));
    assert!(cpu.bus.reads.is_empty() && cpu.bus.writes.is_empty(), "unconsumed peripheral accesses");
    eprintln!("segment: {n} instructions, {checks} checkpoints");
    assert!(checks > 500 && n > 500_000, "{checks} checkpoints, {n} instructions");
    // the harness must notice a wrong result: corrupt one checkpoint in the middle
    let mut bad = ops.clone();
    let i = bad.iter().enumerate().filter(|(_, o)| matches!(o, TraceOp::Check(_))).nth(checks / 2).unwrap().0;
    if let TraceOp::Check(s) = &mut bad[i] {
        s.d[2] ^= 1;
    }
    let mut cpu = native(Arch::Tc16, simos_blank(&flash));
    assert!(play(&mut cpu, &bad).is_err(), "corrupted checkpoint not detected");
}

/// Native core vs Unicorn board on the same SIMOS18.1 workload (release build):
/// records SIMEMU_REPLAY_INSNS (default 50 M) board instructions from reset as a free-running
/// trace, plays it on a fresh native core SIMEMU_BENCH_REPS times (default 3) and runs a fresh
/// board freely for the same count. SIMEMU_BENCH_LOOP=1 replays forever (for `sample`),
/// SIMEMU_BENCH_TRACE=file caches the recorded trace, SIMEMU_BENCH_NO_BOARD=1 skips the board run.
#[test]
#[ignore = "benchmark; needs the SIMOS18.1 images; run with --release --ignored"]
fn bench_native_vs_unicorn() {
    let (Some(flash), Some(eeprom)) = (find(FLASH), find(EEPROM)) else {
        eprintln!("SKIP: firmware images not found");
        return;
    };
    block_steps();
    let insns = env_u64("SIMEMU_REPLAY_INSNS", 50_000_000);
    let mut ops = Vec::new();
    // SIMEMU_BENCH_TRACE=file: reuse a recorded trace (written on the first run)
    let cache = std::env::var("SIMEMU_BENCH_TRACE").ok().map(PathBuf::from);
    if let Some(b) = cache.as_ref().and_then(|p| std::fs::read(p).ok()) {
        ops = trace_from_bytes(&b).unwrap();
    } else {
        let mut board = Simos18Board::new_on(&flash, &eeprom, simemu_tc1791::CpuBackend::Unicorn).expect("board");
        let mut cpu = native(Arch::Tc16, simos_memory(&board));
        let brom = board.hw().brom.to_vec();
        let rec = TraceRec { ops: &mut ops, check_every: 0, extra_init: vec![(0xAFFF_C000, brom)] };
        let (rep, rp) = record_replay(&mut board, &mut cpu, insns, simos_ram, Some(rec)).unwrap();
        print_report("bench recording", &rep, &rp);
        if let Some(p) = &cache {
            std::fs::write(p, trace_to_bytes(&ops)).unwrap();
        }
    }
    let runs = ops.iter().filter(|o| matches!(o, TraceOp::RunTo(_))).count();
    eprintln!("trace: {} ops, {runs} RunTo", ops.len());
    let reps = env_u64("SIMEMU_BENCH_REPS", 3);
    let looping = std::env::var("SIMEMU_BENCH_LOOP").is_ok();
    let mut best = f64::MAX;
    let mut n = 0;
    let mut i = 0;
    while looping || i < reps {
        i += 1;
        let mut cpu = native(Arch::Tc16, simos_blank(&flash));
        let t = Instant::now();
        n = play(&mut cpu, &ops).unwrap_or_else(|d| panic!("trace diverged: {d:?}"));
        let dt = t.elapsed().as_secs_f64();
        best = best.min(dt);
        eprintln!("native: {n} instructions in {dt:.3} s = {:.1} MIPS ({} blocks cached)", n as f64 / dt / 1e6, cpu.cached_blocks());
    }
    if std::env::var("SIMEMU_BENCH_NO_BOARD").is_ok() {
        return;
    }
    let mut board = Simos18Board::new_on(&flash, &eeprom, simemu_tc1791::CpuBackend::Unicorn).expect("board");
    let st = board.run_instructions(insns).expect("board run");
    eprintln!(
        "unicorn board: {} instructions in {:.3} s = {:.1} MIPS",
        st.instructions, st.wall_time_s, st.mips
    );
    eprintln!("native best: {:.1} MIPS = {:.2}x the board", n as f64 / best / 1e6, (n as f64 / best) / (st.instructions as f64 / st.wall_time_s));
}
