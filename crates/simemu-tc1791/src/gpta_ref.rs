//! Reference GPTA model: `gpta.rs` as of commit 5b173b2, before the routing bitmask / due-mask / lazy-PLL
//! caches. Test oracle only: the differential tests in `gpta.rs` (`tests::diff_*`) run it in lockstep with
//! [`crate::gpta::Gpta`] and require identical observable behaviour. It shares the public types.
#![allow(dead_code)]

use crate::gpta::{src_addr, CellKind, InputEdge, Line, OutputEvent, F_GPTA_DEFAULT, GPTA0_BASE, GPTA1_BASE, LTCA2_BASE};

const BASES: [u32; 3] = [GPTA0_BASE, GPTA1_BASE, LTCA2_BASE];

/// Source of a clock bus line (CDU, Figure 24-24 p.24-36, CKBCTR p.24-173).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Clk {
    /// fGPTA / 2^n.
    Div(u32),
    /// PLL output (compensated micro ticks) of kernel k.
    Pll(u8),
    /// DCM output of (kernel, dcm).
    Dcm(u8, u8),
    /// Not modelled (uncompensated PLL, FPC prescaler outputs).
    Off,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Consumer {
    Gt(u8, u8),
    Ltc(u8, u8),
}

#[derive(Default, Clone)]
struct Fpc {
    ctr: u32,
    tim: u32,
    /// Signal_Output.Level (SOL)
    sol: bool,
    /// Signal_Filtered
    sf: bool,
    /// Edge seen at the input since the last filter tick: Some(true) rising.
    edge: Option<bool>,
    /// Last processed filter tick + 1 (ticks before this are done).
    next_ok: u64,
}

#[derive(Default, Clone)]
struct Dcm {
    ctr: u32,
    /// Timer value at tick `t0` (increments every fGPTA tick).
    tim0: u32,
    t0: u64,
    cav: u32,
    cov: u32,
    /// Compare already evaluated for ticks < cmp_ok.
    cmp_ok: u64,
}

#[derive(Default, Clone)]
struct Pll {
    ctr: u32,
    mti: u32,
    cnt: u32,
    stp: u32,
    rev: u32,
    dtr: u32,
    /// Next tick to evaluate (all ticks < t are done).
    t: u64,
    event: bool,
}

#[derive(Default, Clone)]
struct Gt {
    ctr: u32,
    rev: u32,
    tim: u32,
    /// Div clocks: tick up to which `tim` is valid.
    t: u64,
    /// Div clocks: last tick whose timer-event equality compares have been evaluated. Separate from `t`: a capture or
    /// a routed input at the same tick syncs the timer first, and the compare at that tick must still run.
    cmp_t: u64,
}

#[derive(Default, Clone)]
struct Cell {
    ctr: u32,
    x: u32,
    cen: bool,
    out: bool,
    input: bool,
    mode_out: u8,
    ev: bool,
    oia: bool,
    /// GTCXR write protection (CAC & OSM after compare, p.24-179).
    wp: bool,
    /// LTC timer: reset requested for the next input pulse.
    rtm: bool,
    /// LTC timer: select line value (SLO).
    slv: bool,
    /// LTC timer on a Div clock: tick up to which `x` is valid.
    t: u64,
}

#[derive(Clone)]
struct Kernel {
    fpc: [Fpc; 6],
    fpcstat: u32,
    pdlctr: u32,
    dcm: [Dcm; 4],
    pll: Pll,
    ckbctr: u32,
    gt: [Gt; 2],
    gtc: Vec<Cell>,
    ltc: Vec<Cell>,
    srs: [u32; 4],
    srnr: u32,
    mractl: u32,
    mra: Vec<u32>,
    mra_fill: usize,
    mra_out: u32,
    gtc_src: [Line; 32],
    ltc_src: [Line; 64],
    pins: u64,
    /// Registers without modelled behaviour (CLC, FDR, ID, DBGCTR, MMXCTR..): raw storage by word offset.
    raw: [u32; 0x200],
}

impl Kernel {
    fn new() -> Self {
        Self {
            fpc: Default::default(),
            fpcstat: 0,
            pdlctr: 0,
            dcm: Default::default(),
            pll: Pll::default(),
            ckbctr: 0xFFFF, // reset value p.24-173
            gt: Default::default(),
            gtc: vec![Cell::default(); 32],
            ltc: vec![Cell::default(); 64],
            srs: [0; 4],
            srnr: 0,
            mractl: 0,
            mra: Vec::new(),
            mra_fill: 0,
            mra_out: 0,
            gtc_src: [Line::None; 32],
            ltc_src: [Line::None; 64],
            pins: 0,
            raw: [0; 0x200],
        }
    }
}

const LTC_MODE_CAPTURE: u32 = 0;
const LTC_MODE_COMPARE: u32 = 1;
/// No event scheduled.
const NEVER: u64 = u64::MAX;

/// TIM change of one pure filter tick ([`Gpta::fpc_pure`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FpcCount {
    Up,
    Down,
    /// Delayed mode with the reset bit (FPCCTR bit 24): TIM returns to 0 in one tick.
    Reset,
}
/// Recompute every cached next-event value on use and assert it (debug builds: the GPTA unit tests
/// check the caches against the plain computation).
const VERIFY_CACHE: bool = cfg!(debug_assertions);
/// LTC cell below the first timer cell of its kernel (no chain).
const NO_HEAD: u8 = u8::MAX;

/// The three GPTA kernels.
#[derive(Clone)]
pub struct Gpta {
    pub f_gpta_hz: u64,
    now: u64,
    k: [Kernel; 3],
    edctr: u32,
    reqs: Vec<u32>,
    outs: Vec<OutputEvent>,
    routing_dirty: bool,
    /// Consumers of the PLL outputs of GPTA0/GPTA1 and of the DCM outputs.
    pll_cons: [Vec<Consumer>; 2],
    dcm_cons: [[Vec<Consumer>; 4]; 2],
    /// (kernel, gt) running on a Div clock with its divider log2.
    div_gts: Vec<(u8, u8, u32)>,
    /// (kernel, timer cell) LTC timers on a Div clock line.
    div_chains: Vec<(u8, u8, u32)>,
    /// Compare-mode GTCs hooked to (kernel, gt).
    cmp_cells: [[Vec<u8>; 2]; 2],
    /// Counters for tests/diagnostics.
    pub pll_pulses: [u64; 2],
    /// End (exclusive) of the chain of each LTC timer cell, per kernel (cached with the routing: LTC mode
    /// changes set `routing_dirty`).
    chain_end: [[u8; 64]; 3],
    /// Timer cell heading the chain of each LTC cell ([`NO_HEAD`] before the first timer), cached with
    /// the routing like `chain_end`.
    chain_head: [[u8; 64]; 3],
    /// Cached next event tick of each arithmetically advanced LTC timer chain, before the "not in the
    /// past" clamp ([`Gpta::chain_next`]); [`NEVER`] = no event. Valid while the chain's bit in
    /// `chain_dirty` is clear: every mutation of a cell of the chain sets it ([`Gpta::touch`]). Lazy
    /// syncs keep the value (the pulses they add only move the chain towards the same event), except an
    /// overdue chain, which `sync_chain` marks.
    chain_next: [[u64; 64]; 3],
    chain_dirty: [u64; 3],
    /// Timer cells of `div_chains` per kernel, their index in it, and a min tree over their cached
    /// events (leaves at `len / 2 + index`, root at 1; [`Gpta::chains_min`]).
    div_mask: [u64; 3],
    /// Level-gated chains of `div_chains` (the others run on a clock bus line).
    div_gated: [u64; 3],
    chain_slot: [[u8; 64]; 3],
    chain_tree: Vec<u64>,
    /// Earliest due tick per category (FPC, DCM, PLL, GT) at the last `next_internal`.
    due: [u64; 4],
    /// Kernel whose clock bus and global timers LTCA2 uses: GPTA1 on TC1796/TC1791, GPTA0 on
    /// devices without GPTA1 ([`Gpta::tc1784`]).
    ltca2_host: usize,
    /// MRADIN words per kernel (TC1791 firmware: 54/54/32, VERIFIED-T doc Appendix B).
    mra_words: [usize; 3],
    /// Cell outputs that drive a port pin which is also read back as GPTA input line INn of every
    /// kernel: (kernel, kind, cell, n). Supplied by the board (port alternate function + OMCR).
    pin_loops: Vec<(u8, CellKind, u8, u8)>,
    /// Evaluate every busy filter tick with `fpc_step` (reference behaviour for the differential test).
    fpc_eager: bool,
    /// No internal event before this tick: lower bound of `next_internal` from the state left by the
    /// last full [`Gpta::advance`] (0 = unknown, after a register write). An advance without input edges
    /// that ends before it only moves `now` ([`Gpta::advance`]).
    quiet_until: u64,
    /// `now` moved without `sync_all(now)` + `fpc_catch_up_all(now + 1)`; [`Gpta::settle`] does them
    /// where the counters are observed or changed (see [`Gpta::advance`]).
    unsettled: bool,
    /// Every advance takes the full path (reference behaviour for the differential test).
    settle_eager: bool,
}

impl Default for Gpta {
    fn default() -> Self {
        Self::new(F_GPTA_DEFAULT)
    }
}

fn lane_mask(size: u8) -> u32 {
    match size {
        1 => 0xFF,
        2 => 0xFFFF,
        _ => 0xFFFF_FFFF,
    }
}

impl Gpta {
    pub fn new(f_gpta_hz: u64) -> Self {
        Self {
            f_gpta_hz,
            now: 0,
            k: [Kernel::new(), Kernel::new(), Kernel::new()],
            edctr: 0,
            reqs: Vec::new(),
            outs: Vec::new(),
            routing_dirty: true,
            pll_cons: Default::default(),
            dcm_cons: Default::default(),
            div_gts: Vec::new(),
            div_chains: Vec::new(),
            cmp_cells: Default::default(),
            pll_pulses: [0; 2],
            chain_end: [[64; 64]; 3],
            chain_head: [[NO_HEAD; 64]; 3],
            chain_next: [[NEVER; 64]; 3],
            chain_dirty: [u64::MAX; 3],
            div_mask: [0; 3],
            div_gated: [0; 3],
            chain_slot: [[NO_HEAD; 64]; 3],
            chain_tree: vec![NEVER; 2],
            due: [0; 4],
            ltca2_host: 1,
            mra_words: [54, 54, 32],
            pin_loops: Vec::new(),
            fpc_eager: false,
            quiet_until: 0,
            unsettled: false,
            settle_eager: false,
        }
    }

    /// Board wiring: cell outputs fed back as pin inputs (see the `pin_loops` field).
    pub fn set_pin_loops(&mut self, loops: &[(u8, CellKind, u8, u8)]) {
        if self.pin_loops != loops {
            self.pin_loops = loops.to_vec();
            self.quiet_until = 0;
        }
    }

    /// TC1784 (AUDO Future, DQ200_G2 `0CW300043H`): GPTA0 + a 32-cell LTCA2, no GPTA1. Evidence from
    /// the firmware's own GPTA driver (0x800E4348): 54 GPTA0 MRADIN words (loop bound 0x36 at
    /// 0x800E43A4), 24 LTCA2 words (0x18 at 0x800E4628), 32 LTCA2 cells configured (0x800E4656)
    /// with 8 SRCs 0xF0002FE0..FFC (0x800E46C2), no access to 0xF0002000..0xF00027FF. With no GPTA1,
    /// LTCA2 must take its clock bus from GPTA0 (VERIFIED-T: LIMCR routes the LTCA2 timers LTC22 to
    /// CLK2 and LTC26 to CLK0; only GPTA0.CKBCTR is programmed).
    pub fn tc1784(f_gpta_hz: u64) -> Self {
        Self { ltca2_host: 0, mra_words: [54, 54, 24], ..Self::new(f_gpta_hz) }
    }

    /// TC1766 (DQ250 `0D9300011R`): the firmware writes exactly 52 GPTA0 MRADIN words between the array reset
    /// (MRACTL = 2) and the enable (0x3401) — 28 OMCR words (output groups 13..0) + 16 LIMCR + 8 GIMCR; decoded with 52
    /// the input routes fall on the cells the firmware configures (VERIFIED-T, docs/DQ250_IO_MAP.md §2).
    pub fn tc1766(f_gpta_hz: u64) -> Self {
        Self { mra_words: [52, 54, 32], ..Self::new(f_gpta_hz) }
    }

    /// Current GPTA time (ticks of fGPTA).
    pub fn now(&self) -> u64 {
        self.now
    }

    /// Tick of the next internal event (cell compare, timer overflow, filter/PLL tick) at or
    /// before `cap`, `None` if there is none: lets a board end its execution quantum there, so
    /// the service requests of that event reach the CPU at its time and not at the next boundary.
    pub fn next_event_tick(&mut self, cap: u64) -> Option<u64> {
        if self.quiet_until > cap && !self.settle_eager {
            return None; // next_internal >= quiet_until (see `advance`)
        }
        self.rebuild_routing();
        let t = self.next_internal(cap);
        (t <= cap).then_some(t)
    }

    /// Seconds -> GPTA ticks.
    pub fn ticks(&self, seconds: f64) -> u64 {
        (seconds * self.f_gpta_hz as f64).round() as u64
    }

    /// Append the pending requests ([`Gpta::take_requests`]) and output transitions
    /// ([`Gpta::take_outputs`]) to the caller's buffers (no allocation once they have grown).
    pub fn drain_into(&mut self, reqs: &mut Vec<u32>, outs: &mut Vec<OutputEvent>) {
        reqs.append(&mut self.reqs);
        outs.append(&mut self.outs);
    }

    /// Requests or output transitions are waiting to be taken.
    pub fn pending(&self) -> bool {
        !self.reqs.is_empty() || !self.outs.is_empty()
    }

    /// CLC and FDR of GPTA0 (plain storage; no lazily advanced state involved).
    pub fn clock_regs(&self) -> (u32, u32) {
        (self.k[0].raw[0], self.k[0].raw[3])
    }

    /// SRC addresses whose SRR the board must set (in trigger order).
    pub fn take_requests(&mut self) -> Vec<u32> {
        std::mem::take(&mut self.reqs)
    }

    /// Cell output transitions since the last call.
    pub fn take_outputs(&mut self) -> Vec<OutputEvent> {
        std::mem::take(&mut self.outs)
    }

    /// True for GPTA addresses handled here (everything in the three kernels except the SRC registers).
    pub fn contains(addr: u32) -> bool {
        Self::split(addr).is_some()
    }

    fn split(addr: u32) -> Option<(usize, u32)> {
        let k = match addr {
            0xF000_1800..=0xF000_1FFF => 0,
            0xF000_2000..=0xF000_27FF => 1,
            0xF000_2800..=0xF000_2FFF => 2,
            _ => return None,
        };
        let off = addr - BASES[k];
        let first_src = if k == 2 { 0x7FC - 4 * 15 } else { 0x7FC - 4 * 37 };
        if off >= first_src {
            return None;
        }
        Some((k, off))
    }

    /// Input routing of GTCn (decoded GIMCR) — for tests and diagnostics.
    pub fn gtc_input(&self, kernel: usize, n: usize) -> Line {
        self.k[kernel].gtc_src[n]
    }

    /// Input routing of LTCn (decoded LIMCR).
    pub fn ltc_input(&self, kernel: usize, n: usize) -> Line {
        self.k[kernel].ltc_src[n]
    }

    /// Multiplexer register array words of kernel `k` in FIFO write order and MRACTL.MAEN (p.24-110).
    pub fn mra(&self, k: usize) -> (&[u32], bool) {
        (&self.k[k].mra, self.k[k].mractl & 1 != 0)
    }

    /// Output line level of LTCn of `kernel` (the cell's OUT bit).
    pub fn ltc_out(&self, kernel: usize, n: usize) -> bool {
        self.k[kernel].ltc[n].out
    }

    /// Stored control bits of LTCn of `kernel` (MOD in bits 1:0), without read side effects.
    pub fn ltc_ctr(&self, kernel: usize, n: usize) -> u32 {
        self.k[kernel].ltc[n].ctr
    }

    /// Current global timer value (GTTIMm) of a GPTA kernel.
    pub fn gt_value(&mut self, kernel: usize, m: usize) -> u32 {
        self.settle();
        self.sync_gt(kernel, m, self.now);
        self.k[kernel].gt[m].tim
    }

    // ============================================================================================
    // Register interface
    // ============================================================================================

    /// Read with byte lanes (`size` 1/2/4, natural alignment).
    pub fn mmio_read(&mut self, addr: u32, size: u8) -> u32 {
        let Some((k, off)) = Self::split(addr) else { return 0 };
        let w = self.reg_read(k, off & !3);
        (w >> ((off & 3) * 8)) & lane_mask(size)
    }

    /// Write with byte lanes (`val` is the naturally sized datum).
    pub fn mmio_write(&mut self, addr: u32, val: u32, size: u8) {
        let Some((k, off)) = Self::split(addr) else { return };
        let sh = (off & 3) * 8;
        let mask = lane_mask(size) << sh;
        self.reg_write(k, off & !3, (val << sh) & mask, mask);
    }

    /// Board convention (`cpu_hooks::periph_read`): 32-bit aligned address -> register word.
    pub fn read_word(&mut self, addr: u32) -> Option<u32> {
        let (k, off) = Self::split(addr)?;
        Some(self.reg_read(k, off & !3))
    }

    /// Board convention (`cpu_hooks::periph_write`): aligned address, lane-shifted value, byte-lane mask.
    pub fn write_lanes(&mut self, addr: u32, v: u32, m: u32) -> bool {
        let Some((k, off)) = Self::split(addr) else { return false };
        self.reg_write(k, off & !3, v & m, m);
        true
    }

    fn reg_read(&mut self, k: usize, off: u32) -> u32 {
        if k < 2 && (0x048..=0x077).contains(&off) {
            self.settle(); // FPC TIM: pure filter ticks (every other lazy value is synced by its read below)
        }
        let now = self.now;
        if k == 2 && !matches!(off, 0x008 | 0x020..=0x02C | 0x038..=0x040 | 0x200..=0x3FF) {
            return self.k[2].raw[off as usize / 4];
        }
        match off {
            0x008 => 0x0029_C000,
            0x010..=0x02C => self.k[k].srs[((off - 0x10) / 8) as usize],
            0x030 => self.k[k].srnr,
            0x038 => {
                let kk = &self.k[k];
                let full = kk.mra_fill >= self.mra_words[k];
                kk.mractl & 1 | (full as u32) << 2 | ((kk.mra_fill as u32) & 0x3F) << 8
            }
            0x040 => self.k[k].mra_out,
            0x044 => self.k[k].fpcstat,
            0x048..=0x077 => {
                let f = ((off - 0x48) / 8) as usize;
                if off & 4 == 0 { self.k[k].fpc[f].ctr } else { self.k[k].fpc[f].tim }
            }
            0x078 => self.k[k].pdlctr,
            0x080..=0x0BF => {
                let d = ((off - 0x80) / 16) as usize;
                match off & 0xF {
                    0x0 => self.k[k].dcm[d].ctr & !0x40,
                    0x4 => self.dcm_tim(k, d, now),
                    0x8 => self.k[k].dcm[d].cav,
                    _ => self.k[k].dcm[d].cov,
                }
            }
            0x0C0..=0x0D4 => {
                self.pll_ff(k, now);
                let p = &self.k[k].pll;
                match off {
                    0x0C0 => p.ctr,
                    0x0C4 => p.mti,
                    0x0C8 => p.cnt,
                    0x0CC => p.stp,
                    0x0D0 => p.rev,
                    _ => p.dtr,
                }
            }
            0x0D8 => self.k[k].ckbctr,
            0x0E0..=0x0FF => {
                let m = ((off - 0xE0) / 16) as usize;
                match off & 0xF {
                    0x0 => self.k[k].gt[m].ctr,
                    0x4 => self.k[k].gt[m].rev,
                    0x8 => {
                        self.sync_gt(k, m, now);
                        self.k[k].gt[m].tim
                    }
                    _ => 0,
                }
            }
            0x100..=0x1FF => {
                let n = ((off - 0x100) / 8) as usize;
                let c = &mut self.k[k].gtc[n];
                if off & 4 == 0 {
                    (c.ctr & !(0x8000 | 0x4000 | 0x400)) | (c.out as u32) << 15 | (c.cen as u32) << 10
                } else {
                    c.wp = false; // released by a read access (p.24-60)
                    c.x
                }
            }
            0x200..=0x3FF => {
                let n = ((off - 0x200) / 8) as usize;
                if off & 4 != 0 {
                    self.sync_chain_of(k, n, now);
                    return self.k[k].ltc[n].x;
                }
                let sel = self.ltc_select_in(k, n);
                let c = &self.k[k].ltc[n];
                let mut v = c.ctr & !(0x8000 | 0x4000 | 0x400);
                if c.ctr & 2 != 0 {
                    v = (v & !(0x40 | 0x80)) | (c.slv as u32) << 6; // SLO; CUDCLR reads 0
                } else {
                    v = (v & !0x200) | (sel as u32) << 9;
                }
                v | (c.out as u32) << 15 | (c.cen as u32) << 10
            }
            0x400 if k == 0 => self.edctr,
            _ => self.k[k].raw[off as usize / 4],
        }
    }

    fn reg_write(&mut self, k: usize, off: u32, v: u32, m: u32) {
        if !(0x010..=0x02C).contains(&off) {
            // service request flags (SRSCx/SRSSx) touch no counter and no event time
            self.settle();
            self.quiet_until = 0;
        }
        let now = self.now;
        let merge = |old: u32| (old & !m) | (v & m);
        if k == 2 && !matches!(off, 0x020..=0x02C | 0x038..=0x03C | 0x200..=0x3FF) {
            let e = &mut self.k[2].raw[off as usize / 4];
            *e = merge(*e);
            return;
        }
        match off {
            // SRSCx write-1-to-clear, SRSSx write-1-to-set (p.24-206..214)
            0x010..=0x02C => {
                let g = ((off - 0x10) / 8) as usize;
                let bits = v & m;
                if off & 4 == 0 {
                    self.k[k].srs[g] &= !bits;
                } else {
                    for b in 0..32 {
                        if bits & (1 << b) != 0 {
                            self.flag(k, g, b);
                        }
                    }
                }
            }
            0x030 => self.k[k].srnr = merge(self.k[k].srnr) & 0xFFFF,
            0x038 => {
                let old = self.k[k].mractl;
                let val = merge(old);
                if val & 2 != 0 && val & 1 == 0 {
                    self.k[k].mra_fill = 0; // WCRES (p.24-192)
                }
                self.k[k].mractl = val & 1;
                if old & 1 == 0 && val & 1 != 0 {
                    self.decode_mra(k);
                }
            }
            0x03C if m == 0xFFFF_FFFF => {
                let len = self.mra_words[k];
                let kk = &mut self.k[k];
                if kk.mractl & 1 == 0 && kk.mra_fill < len {
                    if kk.mra.len() < len {
                        kk.mra.resize(len, 0);
                    }
                    kk.mra_out = kk.mra[kk.mra_fill];
                    kk.mra[kk.mra_fill] = v;
                    kk.mra_fill += 1;
                }
            }
            0x044 => self.k[k].fpcstat = merge(self.k[k].fpcstat) & 0x3F3F,
            0x048..=0x077 => {
                let f = ((off - 0x48) / 8) as usize;
                if off & 4 == 0 {
                    self.k[k].fpc[f].ctr = merge(self.k[k].fpc[f].ctr) & 0x01FF_FFFF;
                } else {
                    self.k[k].fpc[f].tim = merge(self.k[k].fpc[f].tim) & 0xFFFF;
                }
                self.k[k].fpc[f].next_ok = self.k[k].fpc[f].next_ok.max(now);
            }
            0x078 => self.k[k].pdlctr = merge(self.k[k].pdlctr) & 0x77,
            0x080..=0x0BF => {
                let d = ((off - 0x80) / 16) as usize;
                match off & 0xF {
                    0x0 => {
                        let val = merge(self.k[k].dcm[d].ctr) & 0x3FF;
                        self.k[k].dcm[d].ctr = val & !0x40;
                        if val & 0x40 != 0 {
                            self.dcm_out(k, d, now); // QCK: immediate output pulse (p.24-162)
                        }
                    }
                    0x4 => {
                        let cur = self.dcm_tim(k, d, now);
                        let dc = &mut self.k[k].dcm[d];
                        dc.tim0 = merge(cur) & 0xFF_FFFF;
                        dc.t0 = now;
                    }
                    0x8 => self.k[k].dcm[d].cav = merge(self.k[k].dcm[d].cav) & 0xFF_FFFF,
                    _ => {
                        self.k[k].dcm[d].cov = merge(self.k[k].dcm[d].cov) & 0xFF_FFFF;
                        self.k[k].dcm[d].cmp_ok = now;
                    }
                }
            }
            0x0C0..=0x0D4 => {
                self.pll_ff(k, now);
                let p = &mut self.k[k].pll;
                match off {
                    0x0C0 => p.ctr = (p.ctr & !m) | (v & m) & 0x1F,
                    0x0C4 => p.mti = merge(p.mti) & 0xFFFF,
                    0x0C8 => p.cnt = merge(p.cnt) & 0xFFFF,
                    0x0CC => p.stp = merge(p.stp) & 0xFFFF,
                    0x0D0 => p.rev = merge(p.rev) & 0xFF_FFFF,
                    _ => p.dtr = merge(p.dtr) & 0x1FF_FFFF,
                }
            }
            0x0D8 => {
                self.sync_all(now);
                self.k[k].ckbctr = merge(self.k[k].ckbctr) & 0x1F_FFFF;
                self.routing_dirty = true;
                self.fpc_restart_all();
            }
            0x0E0..=0x0FF => {
                let g = ((off - 0xE0) / 16) as usize;
                self.sync_gt(k, g, now);
                let t = &mut self.k[k].gt[g];
                match off & 0xF {
                    0x0 => {
                        t.ctr = merge(t.ctr) & 0xFF;
                        self.routing_dirty = true;
                    }
                    0x4 => t.rev = merge(t.rev) & 0xFF_FFFF,
                    0x8 => {
                        t.tim = merge(t.tim) & 0xFF_FFFF;
                        t.t = now;
                        t.cmp_t = now;
                    }
                    _ => {}
                }
            }
            0x100..=0x1FF => {
                let n = ((off - 0x100) / 8) as usize;
                if off & 4 == 0 {
                    self.gtc_ctr_write(k, n, v, m);
                } else {
                    self.gtc_xr_write(k, n, merge(self.k[k].gtc[n].x) & 0xFF_FFFF);
                }
            }
            0x200..=0x3FF => {
                let n = ((off - 0x200) / 8) as usize;
                if off & 4 == 0 {
                    self.ltc_ctr_write(k, n, v, m);
                } else {
                    let x = merge(self.k[k].ltc[n].x) & 0xFFFF;
                    self.ltc_xr_write(k, n, x);
                }
            }
            0x400 if k == 0 => {
                self.sync_all(now);
                self.edctr = merge(self.edctr) & 0x70F; // EDCTR p.24-268
                self.fpc_restart_all();
                self.chain_dirty = [u64::MAX; 3]; // kernel enables gate every chain
            }
            _ => {
                let e = &mut self.k[k].raw[off as usize / 4];
                *e = merge(*e);
            }
        }
    }

    // ============================================================================================
    // Multiplexer register array (p.24-110, Figure 24-71; field layouts p.24-198..205)
    // ============================================================================================

    /// MRA word order (`mra_words` per kernel): OMCR..., LIMCRH7, LIMCRL7, ..., LIMCRL0, GIMCRH3,
    /// GIMCRL3, ..., GIMCRL0 (first word written ends at the top, p.24-110). A shorter LTCA2 array
    /// (TC1784: 32 cells) simply has no LIMCR words for g >= 4.
    fn decode_mra(&mut self, k: usize) {
        let w = self.k[k].mra.clone();
        let n = w.len();
        let (lim, gim): (&[u32], &[u32]) = if k == 2 {
            (&w[n.saturating_sub(16)..], &[])
        } else {
            (&w[n.saturating_sub(24)..n.saturating_sub(8)], &w[n.saturating_sub(8)..])
        };
        let kk = &mut self.k[k];
        kk.gtc_src = [Line::None; 32];
        kk.ltc_src = [Line::None; 64];
        // GIMCR: words = [H3, L3, H2, L2, H1, L1, H0, L0]; field n: EN bit 7, GIMG [6:4], GIML [2:0]
        for (i, &word) in gim.iter().enumerate() {
            let g = 3 - i / 2;
            let hi = i % 2 == 0;
            for j in 0..4 {
                let b = (word >> (8 * j)) & 0xFF;
                if b & 0x80 == 0 {
                    continue;
                }
                let (grp, l) = ((b >> 4) & 7, (b & 7) as u8);
                let cell = 8 * g + if hi { 4 } else { 0 } + j;
                kk.gtc_src[cell] = match grp {
                    0 => Line::In(8 * g as u8 + l),
                    1 => Line::In(32 + 8 * g as u8 + l),
                    2 => Line::Ltc(8 * g as u8 + l),
                    3 => Line::Ltc(32 + 8 * g as u8 + l),
                    4 if l < 6 => Line::Fpc(l),
                    4 => Line::Int(l - 6),
                    _ => Line::None,
                };
            }
        }
        // LIMCR: words = [H7, L7, ..., H0, L0]; LIMG n: 0 IOG(g%4), 1 IOG(4+g%4), 2 GTCG(g%4), 3 CLK, 4 PDL/INT
        for (i, &word) in lim.iter().enumerate() {
            let g = 7 - i / 2;
            let hi = i % 2 == 0;
            for j in 0..4 {
                let b = (word >> (8 * j)) & 0xFF;
                if b & 0x80 == 0 {
                    continue;
                }
                let (grp, l) = ((b >> 4) & 7, (b & 7) as u8);
                let cell = 8 * g + if hi { 4 } else { 0 } + j;
                let q = (g % 4) as u8;
                kk.ltc_src[cell] = match grp {
                    0 => Line::In(8 * q + l),
                    1 => Line::In(32 + 8 * q + l),
                    2 => Line::Gtc(8 * q + l),
                    3 => Line::Clk(l),
                    4 if l < 4 => Line::Pdl(l),
                    4 => Line::Int(l - 4),
                    _ => Line::None,
                };
            }
        }
        self.routing_dirty = true;
    }

    // ============================================================================================
    // Clock bus and routing caches
    // ============================================================================================

    /// Source of clock line `l` of kernel `k` (LTCA2 uses the GPTA1 clock bus, p.24-256).
    fn clk(&self, k: usize, l: u8) -> Clk {
        let k = if k == 2 { self.ltca2_host } else { k };
        let c = self.k[k].ckbctr;
        let other = 1 - k as u8;
        match l {
            0 => Clk::Div(0),
            1 => Clk::Pll(k as u8),
            2 => match c & 0xF {
                n @ 0..=13 => Clk::Div(n),
                14 => Clk::Pll(other),
                _ => Clk::Dcm(k as u8, 3),
            },
            3 => match (c >> 16) & 3 {
                0 => Clk::Dcm(k as u8, 2),
                1 => Clk::Pll(other),
                _ => Clk::Off,
            },
            4 => match (c >> 4) & 0xF {
                15 => Clk::Dcm(k as u8, 1),
                n => Clk::Div(n),
            },
            5 => Clk::Dcm(k as u8, 0),
            6 => match (c >> 8) & 0xF {
                15 => Clk::Off,
                n => Clk::Div(n),
            },
            _ => match (c >> 12) & 0xF {
                15 => Clk::Off,
                n => Clk::Div(n),
            },
        }
    }

    fn gt_running(&self, k: usize, m: usize) -> bool {
        k < 2 && self.edctr & (1 << (2 * k + m)) != 0 && self.edctr & (1 << (8 + k)) != 0
    }

    fn kernel_on(&self, k: usize) -> bool {
        self.edctr & (1 << (8 + k)) != 0
    }

    fn is_timer(c: &Cell) -> bool {
        c.ctr & 2 != 0
    }

    fn rebuild_routing(&mut self) {
        if !self.routing_dirty {
            return;
        }
        self.routing_dirty = false;
        self.chain_dirty = [u64::MAX; 3];
        for k in 0..3 {
            let mut end = 64u8;
            for i in (0..64).rev() {
                self.chain_end[k][i] = end;
                if Self::is_timer(&self.k[k].ltc[i]) {
                    end = i as u8;
                }
            }
            let mut head = NO_HEAD;
            for i in 0..64 {
                if Self::is_timer(&self.k[k].ltc[i]) {
                    head = i as u8;
                }
                self.chain_head[k][i] = head;
            }
        }
        // cleared in place: no allocation once the lists have grown
        self.pll_cons.iter_mut().chain(self.dcm_cons.iter_mut().flatten()).for_each(Vec::clear);
        self.div_gts.clear();
        self.div_chains.clear();
        self.div_gated = [0; 3];
        self.cmp_cells.iter_mut().flatten().for_each(Vec::clear);
        fn add(s: &mut Gpta, src: Clk, c: Consumer) {
            match src {
                Clk::Pll(p) => s.pll_cons[p as usize].push(c),
                Clk::Dcm(dk, d) => s.dcm_cons[dk as usize][d as usize].push(c),
                _ => {}
            }
        }
        for k in 0..2 {
            for m in 0..2 {
                let src = self.clk(k, ((self.k[k].gt[m].ctr >> 4) & 7) as u8);
                match src {
                    Clk::Div(d) => self.div_gts.push((k as u8, m as u8, d)),
                    _ => add(self, src, Consumer::Gt(k as u8, m as u8)),
                }
            }
            for n in 0..32 {
                let c = &self.k[k].gtc[n];
                if c.ctr & 2 != 0 {
                    self.cmp_cells[k][(c.ctr & 1) as usize].push(n as u8);
                }
            }
        }
        for k in 0..3 {
            for n in 0..63 {
                if !Self::is_timer(&self.k[k].ltc[n]) {
                    continue;
                }
                if let Line::Clk(l) = self.k[k].ltc_src[n] {
                    match self.clk(k, l) {
                        Clk::Div(d) => self.div_chains.push((k as u8, n as u8, d)),
                        src => add(self, src, Consumer::Ltc(k as u8, n as u8)),
                    }
                } else if self.chain_clock(k, n).is_some() {
                    self.div_chains.push((k as u8, n as u8, 0)); // level-gated fGPTA
                    self.div_gated[k] |= 1 << n;
                }
            }
        }
        self.div_mask = [0; 3];
        self.chain_slot = [[NO_HEAD; 64]; 3];
        for (i, &(k, ti, _)) in self.div_chains.iter().enumerate() {
            self.div_mask[k as usize] |= 1 << ti;
            self.chain_slot[k as usize][ti as usize] = i as u8;
        }
        let leaves = self.div_chains.len().next_power_of_two();
        self.chain_tree.clear();
        self.chain_tree.resize(2 * leaves, NEVER);
    }

    /// Bring the cached events of all dirty chains of `div_chains` up to date (and the min tree).
    fn refresh_chains(&mut self) {
        for k in 0..3 {
            let mut m = self.chain_dirty[k] & self.div_mask[k];
            while m != 0 {
                let ti = m.trailing_zeros() as usize;
                m &= m - 1;
                let d = self.div_chains[self.chain_slot[k][ti] as usize].2;
                self.chain_next(k, ti, d);
            }
        }
    }

    /// Earliest cached event of all arithmetically advanced chains ([`Gpta::chain_next`], before the
    /// clamp), [`NEVER`] if none: root of a min tree over `div_chains` updated with each cache entry.
    fn chains_min(&mut self) -> u64 {
        if self.routing_dirty {
            let mut m = NEVER;
            for i in 0..self.div_chains.len() {
                let (k, ti, d) = self.div_chains[i];
                m = m.min(self.chain_next(k as usize, ti as usize, d));
            }
            return m;
        }
        self.refresh_chains();
        if VERIFY_CACHE {
            let fresh = self.div_chains.clone().iter().map(|&(k, ti, _)| self.chain_next[k as usize][ti as usize]).min().unwrap_or(NEVER);
            assert_eq!(self.chain_tree[1], fresh, "stale chain tree now {}", self.now);
        }
        self.chain_tree[1]
    }

    /// Index of the first chain of `div_chains` at or after `from` whose cached event is <= `t`
    /// (cache refreshed by the caller).
    fn chain_first_due(&self, from: usize, t: u64) -> Option<usize> {
        let leaves = self.chain_tree.len() / 2;
        if from >= self.div_chains.len() {
            return None;
        }
        // walk up from the leaf until a right sibling subtree holds a due chain, then down
        let mut n = leaves + from;
        if self.chain_tree[n] > t {
            loop {
                if n == 1 {
                    return None;
                }
                if n.is_multiple_of(2) && self.chain_tree[n + 1] <= t {
                    n += 1;
                    break;
                }
                n /= 2;
            }
        }
        while n < leaves {
            n = if self.chain_tree[2 * n] <= t { 2 * n } else { 2 * n + 1 };
        }
        Some(n - leaves)
    }

    // ============================================================================================
    // Time advance
    // ============================================================================================

    /// Advance the model by `ticks` fGPTA cycles, applying `edges` (sorted by `t`, absolute GPTA ticks,
    /// `t` <= end; earlier edges are applied at the current time).
    ///
    /// Lazily advanced counters (GT, PLL, LTC timer chains, pure filter ticks) are handled across calls
    /// exactly as between the events inside one advance: synced where an event needs them, by the read
    /// of a GT/LTC/PLL register itself, and to `now` by [`Gpta::settle`] before a register write, a
    /// filter TIM read or an input edge at `now` (lazy syncs compose, so consecutive advances equal one
    /// advance split at the call boundaries; checked against syncing at every end by
    /// `lazy_settle_matches_settling_every_advance`). Without input edges, an interval that ends before
    /// `quiet_until` (no event can be due) only moves `now`.
    pub fn advance(&mut self, ticks: u64, edges: &[InputEdge]) {
        let end = self.now + ticks;
        if edges.is_empty() && end < self.quiet_until && !self.settle_eager {
            self.now = end;
            self.unsettled = true;
            return;
        }
        if edges.first().is_some_and(|e| e.t <= self.now) {
            // an edge at the current tick acts on the state after that tick (captures, filter ticks)
            self.settle();
        }
        let mut ei = 0;
        loop {
            self.rebuild_routing();
            let te = edges.get(ei).map(|e| e.t.max(self.now));
            let ti = self.next_internal(NEVER - 1);
            let t = te.unwrap_or(u64::MAX).min(ti).min(end);
            if t == end && te.is_none_or(|x| x > end) && ti > end {
                self.now = end;
                self.rest(ti);
                break;
            }
            self.now = t;
            // 1. input edges at t
            let ei0 = ei;
            while let Some(e) = edges.get(ei) {
                if e.t.max(self.now) != t {
                    break;
                }
                ei += 1;
                self.pin(e.kernel as usize, e.line, e.level, t);
            }
            self.process_tick(t, ei > ei0);
            if t == end && ei >= edges.len() {
                let ti = self.next_internal(NEVER - 1);
                if ti > end {
                    self.rest(ti);
                    break;
                }
            }
        }
    }

    /// End of a full advance at `now` with nothing due before `quiet` (`next_internal` computed at
    /// this or an earlier `now`: it only grows while nothing happens, so it bounds the next event).
    fn rest(&mut self, quiet: u64) {
        self.quiet_until = quiet;
        self.unsettled = true;
        if self.settle_eager {
            self.settle(); // reference: the counters synced at the end of every advance
        }
    }

    /// Bring the lazily advanced counters to `now` (`sync_all(now)` + `fpc_catch_up_all(now + 1)`, what
    /// the end of every advance did before).
    #[inline]
    fn settle(&mut self) {
        if self.unsettled {
            self.unsettled = false;
            let now = self.now;
            self.sync_all(now);
            self.fpc_catch_up_all(now + 1);
        }
    }

    /// Earliest internal event strictly after the already-processed state, capped at `cap` (+1 means none).
    fn next_internal(&mut self, cap: u64) -> u64 {
        let mut best = cap + 1;
        let now = self.now;
        let mut due = [NEVER; 4];
        for k in 0..2 {
            if !self.kernel_on(k) {
                continue;
            }
            // FPC filter ticks
            for f in 0..6 {
                if let Some(t) = self.fpc_next(k, f) {
                    due[0] = due[0].min(t);
                }
            }
            // DCM compares
            for d in 0..4 {
                let dc = &self.k[k].dcm[d];
                let from = dc.cmp_ok.max(now);
                let cur = self.dcm_tim(k, d, from);
                let dist = (dc.cov.wrapping_sub(cur)) & 0xFF_FFFF;
                due[1] = due[1].min(from + dist as u64);
            }
            // PLL pulses
            if let Some(t) = self.pll_next(k) {
                due[2] = due[2].min(t);
            }
        }
        best = best.min(due[0]).min(due[1]).min(due[2]);
        // GT on Div clocks: compares and overflow
        for i in 0..self.div_gts.len() {
            let (k, m, d) = self.div_gts[i];
            let (k, m) = (k as usize, m as usize);
            if !self.gt_running(k, m) {
                continue;
            }
            let g = &self.k[k].gt[m];
            let div = 1u64 << d;
            let base = g.t / div;
            let mut n = 0x100_0000 - g.tim as u64; // pulses to overflow
            for &c in &self.cmp_cells[k][m] {
                let cell = &self.k[k].gtc[c as usize];
                if cell.cen {
                    n = n.min(gt_dist(g.tim, cell.x, g.rev));
                }
            }
            // an overdue pulse (counter clock line re-routed / kernel re-enabled) is due at the next clock tick
            due[3] = due[3].min(((base + n) * div).max(now.div_ceil(div) * div));
        }
        best = best.min(due[3]);
        self.due = due;
        // LTC timers on Div clocks: the earliest cached event is exact unless a chain is overdue
        // (then each chain is clamped to its next clock tick)
        let m = self.chains_min();
        if m >= now {
            best = best.min(m);
        } else {
            for i in 0..self.div_chains.len() {
                let (k, ti, d) = self.div_chains[i];
                let raw = self.chain_next(k as usize, ti as usize, d);
                if raw != NEVER {
                    let div = 1u64 << d;
                    best = best.min(raw.max(now.div_ceil(div) * div));
                }
            }
        }
        best.max(now)
    }

    /// Next event tick of the Div-clocked (or level-gated, `d` = 0) timer chain `ti` before the "not in
    /// the past" clamp: `(t / div + chain_dist) · div` while the kernel is on and the chain runs, else
    /// [`NEVER`]. Cached (see `chain_next`).
    #[inline]
    fn chain_next(&mut self, k: usize, ti: usize, d: u32) -> u64 {
        if self.chain_dirty[k] >> ti & 1 == 0 && !self.routing_dirty {
            if VERIFY_CACHE {
                let fresh = if !self.kernel_on(k) || !self.chain_running(k, ti) {
                    NEVER
                } else {
                    self.chain_dist(k, ti).map_or(NEVER, |n| ((self.k[k].ltc[ti].t >> d) + n) << d)
                };
                assert_eq!(fresh, self.chain_next[k][ti], "stale chain cache k{k} ti{ti} now {}", self.now);
            }
            return self.chain_next[k][ti];
        }
        let raw = if !self.kernel_on(k) || !self.chain_running(k, ti) {
            NEVER
        } else {
            self.chain_dist(k, ti).map_or(NEVER, |n| ((self.k[k].ltc[ti].t >> d) + n) << d)
        };
        if !self.routing_dirty {
            self.chain_next[k][ti] = raw;
            self.chain_dirty[k] &= !(1 << ti);
            let slot = self.chain_slot[k][ti];
            if slot != NO_HEAD {
                // min tree over `div_chains` (stop once an ancestor keeps its value)
                let mut n = self.chain_tree.len() / 2 + slot as usize;
                self.chain_tree[n] = raw;
                while n > 1 {
                    n /= 2;
                    let m = self.chain_tree[2 * n].min(self.chain_tree[2 * n + 1]);
                    if self.chain_tree[n] == m {
                        break;
                    }
                    self.chain_tree[n] = m;
                }
            }
        }
        raw
    }

    /// A cell of the chain containing LTC `n` changed: its cached next event must be recomputed.
    #[inline]
    fn touch(&mut self, k: usize, n: usize) {
        let h = self.chain_head[k][n];
        if h != NO_HEAD {
            self.chain_dirty[k] |= 1 << h;
        }
    }

    /// Process everything due at tick `t` (edges already applied).
    /// `pins`: input edges were applied at `t`. Until something happens at `t`, a step whose category
    /// has nothing due at `t` per the preceding `next_internal` (`due`) is skipped: it would only
    /// re-evaluate unchanged state (lazy syncs compose exactly).
    fn process_tick(&mut self, t: u64, pins: bool) {
        let mut active = pins;
        // 2. FPC filter ticks
        for k in 0..2 {
            if !self.kernel_on(k) {
                continue;
            }
            if active || self.due[0] <= t {
                for f in 0..6 {
                    if self.fpc_next(k, f) == Some(t) {
                        self.fpc_step(k, f, t);
                        active = true;
                    }
                }
            }
            // 3. DCM compare
            if active || self.due[1] <= t {
                for d in 0..4 {
                    let dc = &self.k[k].dcm[d];
                    if dc.cmp_ok <= t && self.dcm_tim(k, d, t) == dc.cov {
                        self.k[k].dcm[d].cmp_ok = t + 1;
                        self.flag(k, 0, 3 * d as u32 + 2);
                        active = true;
                    }
                }
            }
        }
        // 4. PLL ticks (input events were flagged by DCM outputs at t)
        if active || self.due[2] <= t {
            for k in 0..2 {
                let ev = self.k[k].pll.event;
                self.pll_ff(k, t);
                if ev || self.pll_next(k) == Some(t) {
                    self.pll_tick(k, t);
                    active = true;
                }
            }
        }
        // 5. GTs on Div clocks: sync and evaluate equality compares on the timer event at t
        for i in 0..self.div_gts.len() {
            if !(active || self.due[3] <= t) {
                break;
            }
            let (k, m, d) = self.div_gts[i];
            let (k, m) = (k as usize, m as usize);
            if !self.gt_running(k, m) || !t.is_multiple_of(1u64 << d) || self.k[k].gt[m].cmp_t >= t {
                continue;
            }
            self.sync_gt(k, m, t);
            self.k[k].gt[m].cmp_t = t;
            self.gt_compares(k, m, t);
        }
        // 6. LTC timers on Div clocks with an event pulse at t
        // In `div_chains` order; a pulse may change later chains, so the cache is refreshed before
        // looking for the next due one.
        if self.chains_min() > t {
            return;
        }
        let mut i = 0;
        while i < self.div_chains.len() {
            let j = if self.routing_dirty {
                i
            } else {
                self.refresh_chains();
                match self.chain_first_due(i, t) {
                    Some(j) => j,
                    None => break,
                }
            };
            i = j + 1;
            let (k, ti, d) = self.div_chains[j];
            let div = 1u64 << d;
            if !t.is_multiple_of(div) {
                continue;
            }
            let (k, ti) = (k as usize, ti as usize);
            if self.k[k].ltc[ti].t >= t {
                continue;
            }
            let raw = self.chain_next(k, ti, d);
            if raw <= t {
                // pulses to the event: n = chain_dist (raw = (t0 / div + n) · div)
                let n = (raw >> d) - (self.k[k].ltc[ti].t >> d);
                self.k[k].ltc[ti].x = self.k[k].ltc[ti].x.wrapping_add(n as u32 - 1) & 0xFFFF;
                self.k[k].ltc[ti].t = t;
                self.ltc_pulse(k, ti, t);
            }
        }
    }

    /// Bring all lazily advanced counters up to `t` (no events may be due before `t`).
    fn sync_all(&mut self, t: u64) {
        for k in 0..2 {
            for m in 0..2 {
                self.sync_gt(k, m, t);
                let g = &mut self.k[k].gt[m];
                g.cmp_t = g.cmp_t.max(t);
            }
            self.pll_ff(k, t);
        }
        for i in 0..self.div_chains.len() {
            let (k, ti, d) = self.div_chains[i];
            let (k, ti) = (k as usize, ti as usize);
            if !self.routing_dirty && self.div_gated[k] >> ti & 1 == 0 {
                self.sync_chain_at(k, ti, d, true, t); // clock bus line: always running
            } else {
                self.sync_chain(k, ti, t);
            }
        }
    }

    // ============================================================================================
    // Pins and FPC (p.24-10..18, algorithm p.24-115..120)
    // ============================================================================================

    fn pin(&mut self, k: usize, line: u8, level: bool, t: u64) {
        if k >= 3 || line >= 64 {
            return;
        }
        let old = self.k[k].pins >> line & 1 != 0;
        if old == level {
            return;
        }
        // the filters counted with the old input level up to here
        for f in 0..6 {
            self.fpc_catch_up(k, f, t);
        }
        let kk = &mut self.k[k];
        kk.pins = (kk.pins & !(1 << line)) | (level as u64) << line;
        // FPC inputs (fixed assignment Table 24-9 p.24-94: SINk_m = IN[2k + 12m])
        for (f, fp) in self.k[k].fpc.iter_mut().enumerate() {
            let ips = (fp.ctr >> 19) & 7;
            if ips < 4 && 2 * f as u32 + 12 * ips == line as u32 {
                fp.edge = Some(level);
            }
        }
        self.route_level(k, Line::In(line), level, t);
    }

    /// Deliver a level change on a routable line to GTC/LTC inputs of kernel `k`.
    fn route_level(&mut self, k: usize, line: Line, level: bool, t: u64) {
        for n in 0..32 {
            if self.k[k].gtc_src[n] == line {
                self.gtc_in(k, n, level, t);
            }
        }
        for n in 0..64 {
            if self.k[k].ltc_src[n] == line {
                self.ltc_in(k, n, level, t);
            }
        }
    }

    fn fpc_input(&self, k: usize, f: usize) -> bool {
        let ips = (self.k[k].fpc[f].ctr >> 19) & 7;
        match ips {
            0..=3 => self.k[k].pins >> (2 * f as u32 + 12 * ips) & 1 != 0,
            5 => self.k[k].fpc[(f + 5) % 6].sol,
            _ => false,
        }
    }

    /// Filter clock divider log2 (CIN0 = fGPTA, CIN2/CIN3 = CLK2/CLK3 when prescaled fGPTA).
    fn fpc_div(&self, k: usize, f: usize) -> Option<u32> {
        match (self.k[k].fpc[f].ctr >> 22) & 3 {
            0 => Some(0),
            1 => None,
            l => match self.clk(k, l as u8) {
                Clk::Div(d) => Some(d),
                _ => None,
            },
        }
    }

    /// Next filter tick of FPC `f` that must be evaluated with [`Gpta::fpc_step`]. Filter ticks that
    /// only count TIM up or down ([`Gpta::fpc_pure`]) are not events: they are applied arithmetically
    /// by [`Gpta::fpc_catch_up`] before anything depends on them.
    fn fpc_next(&self, k: usize, f: usize) -> Option<u64> {
        let fp = &self.k[k].fpc[f];
        let mode = (fp.ctr >> 16) & 7;
        if mode >= 6 {
            return None;
        }
        let busy = fp.tim != 0 || fp.sol != self.fpc_input(k, f) || fp.edge.is_some();
        if !busy {
            return None;
        }
        let div = 1u64 << self.fpc_div(k, f)?;
        let (pure, count) = self.fpc_pure(k, f);
        if pure == 0 {
            let from = fp.next_ok.max(self.now);
            return Some(from.div_ceil(div) * div);
        }
        // up-counting ends in an event (output / SF change); down-counting ends idle
        (count == FpcCount::Up).then(|| fp.next_ok.div_ceil(div) * div + pure * div)
    }

    /// Filter ticks of FPC `f`, from its next one on, that only count TIM (no edge pending, input
    /// constant; algorithm p.24-115..120 as in [`Gpta::fpc_step`]): their number and direction. After
    /// up-counting comes an event tick (TIM reached the threshold), after down-counting the filter is
    /// idle. Busy filters tick only while counting this way between events, so a pure tick changes
    /// nothing but TIM and the processed-tick mark `next_ok`; the pure ticks of a filter run on
    /// consecutive filter clocks from `next_ok`.
    fn fpc_pure(&self, k: usize, f: usize) -> (u64, FpcCount) {
        let fp = &self.k[k].fpc[f];
        if self.fpc_eager || fp.edge.is_some() {
            return (0, FpcCount::Up);
        }
        let cmp = fp.ctr & 0xFFFF;
        let delayed = match (fp.ctr >> 16) & 7 {
            0 => true,
            4 => !fp.sf,
            5 => fp.sf,
            _ => false,
        };
        let inp = self.fpc_input(k, f);
        if delayed && fp.tim < cmp {
            if fp.sol != inp {
                ((cmp - fp.tim) as u64, FpcCount::Up)
            } else if fp.tim == 0 {
                (0, FpcCount::Up)
            } else if fp.ctr & (1 << 24) != 0 {
                (1, FpcCount::Reset)
            } else {
                (fp.tim as u64, FpcCount::Down)
            }
        } else if !delayed && fp.tim != 0 && fp.tim < cmp {
            ((cmp - fp.tim) as u64, FpcCount::Up)
        } else {
            (0, FpcCount::Up)
        }
    }

    /// Apply the pure filter ticks of FPC `f` before tick `upto` (exclusive). Pure ticks are deferred
    /// only inside [`Gpta::advance`]: it applies them before an input edge, before the filter's or its
    /// chained successor's next evaluated tick, and up to its end tick (so register accesses between
    /// advances see the state of the tick-by-tick model).
    fn fpc_catch_up(&mut self, k: usize, f: usize, upto: u64) {
        if k >= 2 || !self.kernel_on(k) || (self.k[k].fpc[f].ctr >> 16) & 7 >= 6 {
            return;
        }
        let (pure, count) = self.fpc_pure(k, f);
        if pure == 0 {
            return;
        }
        let Some(d) = self.fpc_div(k, f) else { return };
        let div = 1u64 << d;
        let fp = &mut self.k[k].fpc[f];
        let first = fp.next_ok.div_ceil(div) * div;
        if first >= upto {
            return;
        }
        let n = ((upto - 1 - first) / div + 1).min(pure);
        match count {
            FpcCount::Up => fp.tim += n as u32,
            FpcCount::Down => fp.tim -= n as u32,
            FpcCount::Reset => fp.tim = 0,
        }
        fp.next_ok = first + (n - 1) * div + 1;
    }

    /// [`Gpta::fpc_catch_up`] of every filter of the GPTA kernels.
    fn fpc_catch_up_all(&mut self, upto: u64) {
        for k in 0..2 {
            for f in 0..6 {
                self.fpc_catch_up(k, f, upto);
            }
        }
    }

    /// A filter may start ticking now (kernel enabled, filter clock re-routed): its first tick is the next
    /// filter clock at or after `now`, as `fpc_next` schedules it (the pure ticks run from `next_ok`).
    fn fpc_restart_all(&mut self) {
        let now = self.now;
        for kk in self.k.iter_mut().take(2) {
            for fp in &mut kk.fpc {
                fp.next_ok = fp.next_ok.max(now);
            }
        }
    }

    fn fpc_step(&mut self, k: usize, f: usize, t: u64) {
        self.fpc_catch_up(k, f, t);
        // a filter chained to this one (input IPS = 5) sees its new output only from the next tick of
        // the filter order: bring its pure ticks up to here with the old input first
        let nf = (f + 1) % 6;
        if (self.k[k].fpc[nf].ctr >> 19) & 7 == 5 {
            self.fpc_catch_up(k, nf, if nf > f { t } else { t + 1 });
        }
        let inp = self.fpc_input(k, f);
        let fp = &mut self.k[k].fpc[f];
        fp.next_ok = t + 1;
        let edge = fp.edge.take();
        let mode = (fp.ctr >> 16) & 7;
        let cmp = fp.ctr & 0xFFFF;
        let delayed = match mode {
            0 => true,
            4 => !fp.sf,
            5 => fp.sf,
            _ => false,
        };
        let mut out: Option<bool> = None;
        let mut glitch = None;
        if delayed {
            if fp.tim >= cmp {
                if cmp == 0 {
                    if fp.sol != inp {
                        fp.sol = inp;
                        fp.sf = inp;
                        out = Some(inp);
                    }
                } else {
                    fp.sol = !fp.sol;
                    fp.sf = fp.sol;
                    out = Some(fp.sol);
                }
                fp.tim = 0;
            } else {
                if fp.tim != 0 {
                    glitch = edge;
                }
                if fp.sol != inp {
                    fp.tim += 1;
                } else if fp.tim != 0 {
                    if fp.ctr & (1 << 24) != 0 { fp.tim = 0 } else { fp.tim -= 1 }
                }
            }
        } else if fp.tim == 0 {
            if fp.sol != inp {
                fp.sol = inp;
                out = Some(inp);
                let bypass = cmp == 0 || (mode == 2 && !inp) || (mode == 3 && inp);
                if bypass { fp.sf = inp } else { fp.tim = 1 }
            }
        } else if fp.tim >= cmp {
            fp.tim = 0;
            fp.sf = fp.sol;
        } else {
            fp.tim += 1;
            glitch = edge;
        }
        if let Some(r) = glitch {
            self.k[k].fpcstat |= 1 << (f + if r { 0 } else { 8 });
        }
        if let Some(level) = out {
            self.fpc_out(k, f, level, t);
        }
    }

    /// FPC output transition: PDL feed-through to DCMs, FPC/INT group (GTC inputs), PDL bus (LTC inputs),
    /// chained FPC inputs.
    fn fpc_out(&mut self, k: usize, f: usize, level: bool, t: u64) {
        let pdl = self.k[k].pdlctr;
        let dcm = match f {
            0 if pdl & 1 == 0 => Some(0),
            2 if pdl & 2 == 0 => Some(1),
            3 if pdl & 0x10 == 0 => Some(2),
            5 if pdl & 0x20 == 0 => Some(3),
            _ => None,
        };
        if let Some(d) = dcm {
            self.dcm_in(k, d, level, t);
        }
        self.route_level(k, Line::Fpc(f as u8), level, t);
        if let Some(p) = [0u8, 2, 3, 5].iter().position(|&x| x as usize == f) {
            self.route_level(k, Line::Pdl(p as u8), level, t);
        }
        let nf = (f + 1) % 6;
        if (self.k[k].fpc[nf].ctr >> 19) & 7 == 5 {
            self.k[k].fpc[nf].edge = Some(level);
        }
    }

    // ============================================================================================
    // DCM (p.24-24..27, algorithm p.24-125)
    // ============================================================================================

    fn dcm_tim(&self, k: usize, d: usize, t: u64) -> u32 {
        let dc = &self.k[k].dcm[d];
        (dc.tim0 as u64 + t.saturating_sub(dc.t0)) as u32 & 0xFF_FFFF
    }

    fn dcm_in(&mut self, k: usize, d: usize, rising: bool, t: u64) {
        let tim = self.dcm_tim(k, d, t);
        let ctr = self.k[k].dcm[d].ctr;
        let (rca, oca) = (ctr & 1 != 0, ctr & 2 != 0);
        let (zero, clock) = if rising { (ctr & 4 != 0, ctr & 0x10 != 0) } else { (ctr & 8 != 0, ctr & 0x20 != 0) };
        self.flag(k, 0, 3 * d as u32 + if rising { 0 } else { 1 });
        let dc = &mut self.k[k].dcm[d];
        if rca == rising {
            dc.cav = tim;
        } else if oca {
            dc.cov = tim;
        }
        if zero {
            dc.tim0 = 0;
            dc.t0 = t + 1;
        }
        if clock {
            self.dcm_out(k, d, t);
        }
    }

    /// DCM output pulse: PLL trigger input and clock bus lines.
    fn dcm_out(&mut self, k: usize, d: usize, t: u64) {
        if self.k[k].pll.ctr & 3 == d as u32 {
            self.k[k].pll.event = true;
        }
        self.rebuild_routing();
        let cons = std::mem::take(&mut self.dcm_cons[k][d]);
        for &c in &cons {
            self.consume_pulse(c, t);
        }
        self.dcm_cons[k][d] = cons;
    }

    // ============================================================================================
    // PLL (p.24-28..34, algorithm p.24-128)
    // ============================================================================================

    fn pll_m(p: &Pll) -> u64 {
        0x1_0000 - p.stp as u64
    }

    fn neg(dtr: u32) -> bool {
        dtr & 0x100_0000 != 0
    }

    /// Next tick at which the PLL emits a pulse or reloads its counter (absent input events).
    fn pll_next(&self, k: usize) -> Option<u64> {
        let p = &self.k[k].pll;
        let aen = p.ctr & 4 != 0;
        let pen = p.ctr & 8 != 0;
        let t = p.t.max(self.now);
        if p.event {
            return Some(t);
        }
        if p.cnt == 0 {
            return ((pen || !aen) && p.mti != 0).then_some(t);
        }
        if pen || Self::neg(p.dtr) {
            return Some(t);
        }
        // DTR is valid at p.t; it turns negative after floor(DTR/m)+1 ticks
        Some((p.t + p.dtr as u64 / Self::pll_m(p) + 1).max(self.now))
    }

    /// Advance the delta register through the pulse-free ticks [pll.t, t).
    fn pll_ff(&mut self, k: usize, t: u64) {
        let p = &mut self.k[k].pll;
        if t <= p.t {
            return;
        }
        let mut n = t - p.t;
        p.t = t;
        let m = Self::pll_m(p) as i64;
        let rev = p.rev as i64;
        let mut d = if Self::neg(p.dtr) { p.dtr as i64 - (1 << 25) } else { p.dtr as i64 };
        // bring into the rotation range [-m, rev)
        while n > 0 && (d >= rev.max(0) || d < -m) {
            if d >= 0 {
                let k2 = ((d - rev).max(0) / m + 1).min(n as i64);
                d -= k2 * m;
                n -= k2 as u64;
            } else {
                let k2 = if rev > 0 { ((-m - d) / rev + 1).min(n as i64) } else { n as i64 };
                d += k2 * rev;
                n -= k2 as u64;
            }
        }
        if n > 0 && rev + m > 0 {
            let y = (d + m) as i128 + n as i128 * rev as i128;
            d = (y % (rev + m) as i128) as i64 - m;
        }
        p.dtr = (d.rem_euclid(1 << 25)) as u32;
    }

    fn pll_tick(&mut self, k: usize, t: u64) {
        let p = &mut self.k[k].pll;
        let aen = p.ctr & 4 != 0;
        if aen && p.event {
            p.ctr |= 8; // Perform_End
        }
        p.event = false;
        let pen = p.ctr & 8 != 0;
        if p.cnt == 0 && (pen || !aen) {
            p.cnt = p.mti;
            p.ctr &= !8;
        }
        let pen = p.ctr & 8 != 0;
        let mut pulse = false;
        let mut done = false;
        if p.cnt != 0 && (pen || Self::neg(p.dtr)) {
            pulse = true;
            p.cnt -= 1;
            done = p.cnt == 0;
        }
        // 25-bit two's-complement delta register (wrapping, as release builds always computed it)
        p.dtr = if Self::neg(p.dtr) {
            p.dtr.wrapping_add(p.rev) & 0x1FF_FFFF
        } else {
            p.dtr.wrapping_add(0xFFFF_0000 | p.stp) & 0x1FF_FFFF
        };
        p.t = t + 1;
        if done {
            self.flag(k, 0, 12);
        }
        if pulse {
            self.pll_pulses[k] += 1;
            self.rebuild_routing();
            let cons = std::mem::take(&mut self.pll_cons[k]);
            for &c in &cons {
                self.consume_pulse(c, t);
            }
            self.pll_cons[k] = cons;
        }
    }

    fn consume_pulse(&mut self, c: Consumer, t: u64) {
        match c {
            Consumer::Gt(k, m) => {
                let (k, m) = (k as usize, m as usize);
                if self.gt_running(k, m) {
                    self.gt_add(k, m, 1);
                    self.gt_compares(k, m, t);
                }
            }
            Consumer::Ltc(k, n) => self.ltc_pulse(k as usize, n as usize, t),
        }
    }

    // ============================================================================================
    // Global timers (p.24-37.., algorithm p.24-130)
    // ============================================================================================

    fn gt_div(&self, k: usize, m: usize) -> Option<u32> {
        match self.clk(k, ((self.k[k].gt[m].ctr >> 4) & 7) as u8) {
            Clk::Div(d) => Some(d),
            _ => None,
        }
    }

    fn sync_gt(&mut self, k: usize, m: usize, t: u64) {
        if k > 1 {
            return;
        }
        let Some(d) = self.gt_div(k, m) else {
            self.k[k].gt[m].t = self.k[k].gt[m].t.max(t);
            return;
        };
        let g = &self.k[k].gt[m];
        if t <= g.t {
            return;
        }
        let n = if self.gt_running(k, m) { (t >> d) - (g.t >> d) } else { 0 };
        self.k[k].gt[m].t = t;
        if n > 0 {
            self.gt_add(k, m, n);
        }
    }

    fn gt_add(&mut self, k: usize, m: usize, mut n: u64) {
        loop {
            let g = &mut self.k[k].gt[m];
            let room = 0x100_0000 - g.tim as u64;
            if n < room {
                g.tim += n as u32;
                return;
            }
            n -= room;
            g.tim = g.rev;
            self.flag(k, 0, 13 + m as u32);
        }
    }

    /// Equality compares on a timer event of GT(k,m) (GTC algorithm p.24-131).
    fn gt_compares(&mut self, k: usize, m: usize, t: u64) {
        let tim = self.k[k].gt[m].tim;
        for i in 0..self.cmp_cells[k][m].len() {
            let n = self.cmp_cells[k][m][i] as usize;
            let c = &self.k[k].gtc[n];
            if c.cen && c.x == tim {
                self.gtc_compare_event(k, n, t);
            }
        }
    }

    /// TGE flag (p.24-170): bit SCO+9 of (GT - X); greater-or-equal when that bit is 0.
    fn tge(&self, k: usize, m: usize, x: u32) -> bool {
        let g = &self.k[k].gt[m];
        let bit = (g.ctr & 0xF) + 9;
        let diff = (g.tim as u64 | 0x100_0000).wrapping_sub(x as u64) & 0x1FF_FFFF;
        if bit >= 24 {
            g.tim >= x
        } else {
            diff >> bit & 1 == 0
        }
    }

    // ============================================================================================
    // GTC (p.24-55..66, algorithm p.24-131..136, registers p.24-175..179)
    // ============================================================================================

    fn gtc_ctr_write(&mut self, k: usize, n: usize, v: u32, m: u32) {
        let c = &mut self.k[k].gtc[n];
        let val = (c.ctr & !m) | (v & m);
        let old = c.ctr;
        c.ctr = val & 0x39FF; // OUT, OIA, CEN not stored
        if m & 0x100 != 0 {
            c.cen = val & 0x100 == 0; // enabled by writing EOA = 0 (p.24-62)
        }
        if (old ^ c.ctr) & 3 != 0 {
            self.routing_dirty = true; // compare mode / timer select: `cmp_cells`
        }
        if v & m & 0x4000 != 0 {
            self.k[k].gtc[n].oia = true;
            self.gtc_mux(k, n, n, self.now);
        }
    }

    fn gtc_xr_write(&mut self, k: usize, n: usize, x: u32) {
        let now = self.now;
        let c = &mut self.k[k].gtc[n];
        if c.wp {
            return;
        }
        c.x = x;
        // Compare on X write access: equality or GES/TGE (algorithm p.24-133)
        if c.ctr & 2 != 0 && c.cen {
            let m = (c.ctr & 1) as usize;
            let ges = c.ctr & 0x10 != 0;
            self.sync_gt(k, m, now);
            if self.k[k].gt[m].tim == x || (ges && self.tge(k, m, x)) {
                self.gtc_compare_event(k, n, now);
            }
        }
    }

    fn gtc_compare_event(&mut self, k: usize, n: usize, t: u64) {
        let ctr = self.k[k].gtc[n].ctr;
        if ctr & 0x20 != 0 {
            // capture after compare: selected or alternate timer (CAT)
            let m = ((ctr & 1) ^ ((ctr >> 6) & 1)) as usize;
            self.sync_gt(k, m, t);
            self.k[k].gtc[n].x = self.k[k].gt[m].tim;
            if ctr & 4 != 0 {
                self.k[k].gtc[n].wp = true;
            }
        }
        self.gtc_event(k, n, t);
    }

    /// Common event tail: request, one-shot, output/links.
    fn gtc_event(&mut self, k: usize, n: usize, t: u64) {
        let c = &mut self.k[k].gtc[n];
        c.ev = true;
        if c.ctr & 4 != 0 {
            c.cen = false;
        }
        self.flag(k, 1, n as u32);
        self.gtc_mux(k, n, n, t);
    }

    fn gtc_in(&mut self, k: usize, n: usize, level: bool, t: u64) {
        let c = &mut self.k[k].gtc[n];
        let old = c.input;
        c.input = level;
        if old == level || c.ctr & 2 != 0 || !c.cen {
            return;
        }
        let want = if level { c.ctr & 0x10 != 0 } else { c.ctr & 0x20 != 0 };
        if !want {
            return;
        }
        let m = (c.ctr & 1) as usize;
        self.sync_gt(k, m, t);
        self.k[k].gtc[n].x = self.k[k].gt[m].tim;
        self.gtc_event(k, n, t);
    }

    /// Manage_Mux for GTCs `from..` (links propagate upwards) — `to` is the highest cell with an event.
    fn gtc_mux(&mut self, k: usize, from: usize, to: usize, t: u64) {
        let mut mi = if from == 0 { 0 } else { self.k[k].gtc[from - 1].mode_out };
        for n in from..32 {
            let c = &mut self.k[k].gtc[n];
            let ocm = (c.ctr >> 11) & 7;
            let local = (c.ev || c.oia) && ocm & 3 != 0;
            let old = c.out;
            let mo = if local {
                set_data(&mut c.out, (ocm & 3) as u8);
                if c.ctr & 0x80 == 0 { (ocm & 3) as u8 } else if ocm & 4 != 0 { mi } else { 0 }
            } else if ocm & 4 != 0 {
                set_data(&mut c.out, mi);
                mi
            } else {
                0
            };
            if c.ctr & 0x100 != 0 && mi != 0 {
                c.cen = true;
                c.ctr &= !0x100;
            }
            c.mode_out = mo;
            c.ev = false;
            c.oia = false;
            let out = c.out;
            if out != old {
                self.output_event(k, CellKind::Gtc, n, out, t);
                self.route_level(k, Line::Gtc(n as u8), out, t);
            }
            mi = mo;
            if n >= to && mo == 0 {
                break;
            }
        }
    }

    // ============================================================================================
    // LTC (p.24-67..78, algorithm p.24-137..143, registers p.24-180..190)
    // ============================================================================================

    /// Index of the timer cell feeding Y_In/Select_In of cell `n` (nearest preceding timer).
    fn ltc_timer_of(&self, k: usize, n: usize) -> Option<usize> {
        (0..n).rev().find(|&i| Self::is_timer(&self.k[k].ltc[i]))
    }

    fn ltc_select_in(&self, k: usize, n: usize) -> bool {
        self.ltc_timer_of(k, n).is_some_and(|ti| self.k[k].ltc[ti].slv)
    }

    fn ltc_y_in(&self, k: usize, n: usize) -> u32 {
        self.ltc_timer_of(k, n).map_or(0, |ti| self.k[k].ltc[ti].x)
    }

    /// Cells following timer `ti` up to the next timer cell.
    fn chain(&self, k: usize, ti: usize) -> std::ops::Range<usize> {
        if self.routing_dirty {
            let end = (ti + 1..64).find(|&i| Self::is_timer(&self.k[k].ltc[i])).unwrap_or(64);
            return ti + 1..end;
        }
        ti + 1..self.chain_end[k][ti] as usize
    }

    fn ltc_active(c: &Cell, sel: bool) -> bool {
        (sel && c.ctr & 0x20 != 0) || (!sel && c.ctr & 0x10 != 0)
    }

    /// Input pulses until the next event of the chain of timer `ti` (None = never).
    fn chain_dist(&self, k: usize, ti: usize) -> Option<u64> {
        let c = &self.k[k].ltc[ti];
        if !c.cen && c.ctr & 4 != 0 {
            return None;
        }
        if c.rtm || c.x == 0xFFFF {
            return Some(1);
        }
        let mut n = (0xFFFF - c.x) as u64;
        for j in self.chain(k, ti) {
            let cj = &self.k[k].ltc[j];
            if cj.ctr & 3 == LTC_MODE_COMPARE && cj.cen && Self::ltc_active(cj, c.slv) {
                let dd = (cj.x.wrapping_sub(c.x) & 0xFFFF) as u64;
                if dd != 0 {
                    n = n.min(dd);
                }
            }
        }
        Some(n.max(1))
    }

    /// Clock of an arithmetically advanced LTC timer chain: (divider log2, running). A prescaled-fGPTA
    /// clock bus line, or fGPTA gated by a level-sensitive data input (ILM = 1 on a pin or cell output,
    /// PEN = 0: "the external signal operates as gating signal for the cell", Table 24-3 p.24-73; AIL =
    /// bit 5 selects active low, p.24-181).
    fn chain_clock(&self, k: usize, ti: usize) -> Option<(u32, bool)> {
        let c = &self.k[k].ltc[ti];
        match self.k[k].ltc_src[ti] {
            Line::Clk(l) => match self.clk(k, l) {
                Clk::Div(d) => Some((d, true)),
                _ => None,
            },
            Line::None => None,
            _ if c.ctr & 0x112 == 0x102 => Some((0, c.input != (c.ctr & 0x20 != 0))),
            _ => None,
        }
    }

    fn chain_running(&self, k: usize, ti: usize) -> bool {
        self.chain_clock(k, ti).is_some_and(|c| c.1)
    }

    /// Bring a Div-clocked timer to `t` (pulses in (c.t, t], none of them an event).
    fn sync_chain(&mut self, k: usize, ti: usize, t: u64) {
        if let Some((d, run)) = self.chain_clock(k, ti) {
            self.sync_chain_at(k, ti, d, run, t);
        } else {
            self.k[k].ltc[ti].t = self.k[k].ltc[ti].t.max(t);
        }
    }

    /// [`Gpta::sync_chain`] with the chain clock (`chain_clock`) known.
    fn sync_chain_at(&mut self, k: usize, ti: usize, d: u32, run: bool, t: u64) {
        let t0 = self.k[k].ltc[ti].t;
        if t <= t0 {
            return;
        }
        let n = if run { (t >> d) - (t0 >> d) } else { 0 };
        let mut add = 0;
        if n > 0 {
            // pulses before the chain's next event (chain_dist - 1): from the cached event while the
            // kernel is on (then the chain runs, so the cache holds its event)
            let max = if self.kernel_on(k) {
                let raw = self.chain_next(k, ti, d);
                if raw == NEVER { u64::MAX } else { (raw >> d) - (t0 >> d) - 1 }
            } else {
                self.chain_dist(k, ti).map_or(u64::MAX, |x| x - 1)
            };
            if n > max {
                self.touch(k, ti); // overdue: the clamped pulses move the event
            }
            add = n.min(max);
        }
        let c = &mut self.k[k].ltc[ti];
        c.x = (c.x as u64 + add) as u32 & 0xFFFF;
        c.t = t;
    }

    fn sync_chain_of(&mut self, k: usize, n: usize, t: u64) {
        if let Some(ti) = if Self::is_timer(&self.k[k].ltc[n]) { Some(n) } else { self.ltc_timer_of(k, n) } {
            self.sync_chain(k, ti, t);
        }
    }

    /// One input pulse of timer cell `ti` with full chain processing (Timer()/Compare()/Manage_Mux()).
    fn ltc_pulse(&mut self, k: usize, ti: usize, t: u64) {
        self.touch(k, ti);
        let c = &mut self.k[k].ltc[ti];
        if c.ctr & 2 == 0 || (!c.cen && c.ctr & 4 != 0) {
            return;
        }
        if c.rtm {
            c.rtm = false;
            c.x = 0xFFFF;
            if c.ctr & 0x200 != 0 {
                c.slv = !c.slv; // coherent update (p.24-182)
                c.ctr &= !0x200;
            }
        } else {
            c.x = (c.x + 1) & 0xFFFF;
        }
        c.t = c.t.max(t);
        let x = c.x;
        let mut lo = 64;
        let mut hi = 0;
        if x == 0xFFFF && c.cen {
            self.ltc_event(k, ti);
            lo = ti;
            hi = ti;
        }
        let (y, sel) = (x, self.k[k].ltc[ti].slv);
        let chain = self.chain(k, ti);
        for j in chain.clone() {
            let cj = &self.k[k].ltc[j];
            if cj.ctr & 3 == LTC_MODE_COMPARE && cj.cen && Self::ltc_active(cj, sel) && cj.x == y {
                self.ltc_event(k, j);
                lo = lo.min(j);
                hi = hi.max(j);
            }
        }
        self.ltc_event_in(k, ti, chain);
        if lo <= hi {
            self.ltc_mux(k, lo, hi, t);
        }
    }

    /// Event_Out propagation towards the timer (reset request in Reset Timer Mode).
    fn ltc_event_in(&mut self, k: usize, ti: usize, chain: std::ops::Range<usize>) {
        let sel = self.k[k].ltc[ti].slv;
        let mut eo = false;
        for j in chain.rev() {
            let cj = &self.k[k].ltc[j];
            eo = match cj.ctr & 3 {
                LTC_MODE_CAPTURE => cj.ev,
                _ if Self::ltc_active(cj, sel) => cj.ev,
                _ => eo,
            };
        }
        if eo && self.k[k].ltc[ti].ctr & 3 == 3 {
            self.k[k].ltc[ti].rtm = true;
            self.touch(k, ti);
        }
    }

    fn ltc_event(&mut self, k: usize, n: usize) {
        self.touch(k, n);
        let c = &mut self.k[k].ltc[n];
        c.ev = true;
        if c.ctr & 4 != 0 {
            c.cen = false;
        }
        let (g, b) = if n < 32 { (2, n as u32) } else { (3, n as u32 - 32) };
        self.flag(k, g, b);
    }

    fn ltc_in(&mut self, k: usize, n: usize, level: bool, t: u64) {
        if self.k[k].ltc[n].ctr & 0x112 == 0x102 && !matches!(self.k[k].ltc_src[n], Line::Clk(_)) {
            // level-gated timer: count the gate interval up to the edge, then switch the gate
            self.sync_chain(k, n, t);
            let c = &mut self.k[k].ltc[n];
            c.input = level;
            c.t = c.t.max(t);
            self.touch(k, n);
            return;
        }
        let c = &mut self.k[k].ltc[n];
        let old = c.input;
        c.input = level;
        if old == level || c.ctr & 0x100 != 0 {
            return; // level-sensitive inputs: only clock lines are modelled (as pulses)
        }
        let want = if level { c.ctr & 0x10 != 0 } else { c.ctr & 0x20 != 0 };
        if !want {
            return;
        }
        match c.ctr & 3 {
            LTC_MODE_CAPTURE if c.cen => {
                if let Some(ti) = self.ltc_timer_of(k, n) {
                    self.sync_chain(k, ti, t.saturating_sub(1));
                }
                self.k[k].ltc[n].x = self.ltc_y_in(k, n);
                self.ltc_event(k, n);
                if let Some(ti) = self.ltc_timer_of(k, n) {
                    let ch = self.chain(k, ti);
                    self.ltc_event_in(k, ti, ch);
                }
                self.ltc_mux(k, n, n, t);
            }
            2 | 3 if !matches!(self.k[k].ltc_src[n], Line::Clk(_)) => self.ltc_pulse(k, n, t),
            _ => {}
        }
    }

    fn ltc_ctr_write(&mut self, k: usize, n: usize, v: u32, m: u32) {
        let now = self.now;
        self.sync_chain_of(k, n, now);
        let c = &mut self.k[k].ltc[n];
        let old = c.ctr;
        let val = (old & !m) | (v & m);
        let timer = val & 2 != 0;
        let mut ctr = val & 0x3BFF & !0x400;
        if timer {
            // CUD (rwh) / CUDCLR (w), p.24-181: writing CUD = 1 sets it; it is cleared only by
            // CUDCLR = 1 with CUD = 0 or by hardware at the next reset overflow. A plain write of
            // CUD = 0 keeps a pending request (SC800S50 sets CUD by read-modify-write without CUDCLR,
            // PWM duty update 0x800A88E4).
            ctr = if val & 0x80 != 0 { ctr & !0x80 } else { ctr | (old & 0x200) };
            if m & 0x40 != 0 {
                c.slv = val & 0x40 != 0;
            }
            c.cen = true;
            if old & 2 == 0 {
                c.t = now; // a new timer starts counting now (lazy Div-clock sync reference)
            }
        } else if m & 0x80 != 0 {
            c.cen = val & 0x80 == 0;
        }
        c.ctr = ctr;
        self.touch(k, n);
        if (old ^ ctr) & 0x133 != 0 {
            self.routing_dirty = true;
        }
        if v & m & 0x4000 != 0 {
            self.k[k].ltc[n].oia = true;
            self.ltc_mux(k, n, n, now);
        }
    }

    fn ltc_xr_write(&mut self, k: usize, n: usize, x: u32) {
        let now = self.now;
        self.sync_chain_of(k, n, now);
        self.k[k].ltc[n].x = x;
        self.touch(k, n);
        let c = &self.k[k].ltc[n];
        if Self::is_timer(c) {
            let mut lo = 64;
            let mut hi = 0;
            if x == 0xFFFF && c.cen {
                self.ltc_event(k, n);
                lo = n;
                hi = n;
            }
            let sel = self.k[k].ltc[n].slv;
            let ch = self.chain(k, n);
            for j in ch.clone() {
                let cj = &self.k[k].ltc[j];
                if cj.ctr & 3 == LTC_MODE_COMPARE && cj.cen && Self::ltc_active(cj, sel) && cj.x == x {
                    self.ltc_event(k, j);
                    lo = lo.min(j);
                    hi = hi.max(j);
                }
            }
            self.ltc_event_in(k, n, ch);
            if lo <= hi {
                self.ltc_mux(k, lo, hi, now);
            }
        } else if c.ctr & 3 == LTC_MODE_COMPARE && c.cen && Self::ltc_active(c, self.ltc_select_in(k, n)) && x == self.ltc_y_in(k, n) {
            self.ltc_event(k, n);
            if let Some(ti) = self.ltc_timer_of(k, n) {
                let ch = self.chain(k, ti);
                self.ltc_event_in(k, ti, ch);
            }
            self.ltc_mux(k, n, n, now);
        }
    }

    fn ltc_mux(&mut self, k: usize, from: usize, to: usize, t: u64) {
        let mut mi = if from == 0 { 0 } else { self.k[k].ltc[from - 1].mode_out };
        for n in from..64 {
            let c = &mut self.k[k].ltc[n];
            let ocm = (c.ctr >> 11) & 7;
            let byp = c.ctr & 3 < 2 && c.ctr & 0x40 != 0;
            let old = c.out;
            let mo = if c.ev || c.oia {
                set_data(&mut c.out, (ocm & 3) as u8);
                if !byp { (ocm & 3) as u8 } else if ocm & 4 != 0 { mi } else { 0 }
            } else if ocm & 4 != 0 {
                set_data(&mut c.out, mi);
                mi
            } else {
                0
            };
            let enable = c.ctr & 3 < 2 && c.ctr & 0x80 != 0 && mi != 0;
            if enable {
                c.cen = true;
                c.ctr &= !0x80;
            }
            c.mode_out = mo;
            c.ev = false;
            c.oia = false;
            let out = c.out;
            if enable {
                self.touch(k, n);
            }
            if out != old {
                self.output_event(k, CellKind::Ltc, n, out, t);
                self.route_level(k, Line::Ltc(n as u8), out, t);
            }
            mi = mo;
            if n >= to && mo == 0 {
                break;
            }
        }
    }

    fn output_event(&mut self, k: usize, kind: CellKind, n: usize, level: bool, t: u64) {
        let gk = if k == 2 { self.ltca2_host } else { k };
        self.sync_gt(gk, 0, t);
        self.sync_gt(gk, 1, t);
        let (gt0, gt1) = (self.k[gk].gt[0].tim, self.k[gk].gt[1].tim);
        self.outs.push(OutputEvent { t, kernel: k as u8, kind, cell: n as u8, level, gt0, gt1 });
        if let Some(&(.., line)) = self.pin_loops.iter().find(|l| (l.0 as usize, l.1, l.2 as usize) == (k, kind, n)) {
            for kk in 0..3 {
                self.pin(kk, line, level, t);
            }
        }
    }

    // ============================================================================================
    // Service requests (p.24-112..114, Table 24-14)
    // ============================================================================================

    /// Set request flag `bit` of SRS group `g` and, if the source is enabled, trigger its node.
    fn flag(&mut self, k: usize, g: usize, bit: u32) {
        self.k[k].srs[g] |= 1 << bit;
        let kk = &self.k[k];
        let node = match (k, g) {
            (2, 2) => (enabled_ltc(&kk.ltc[bit as usize]), bit as usize / 4),
            (2, 3) => (enabled_ltc(&kk.ltc[32 + bit as usize]), (32 + bit as usize) / 4),
            (2, _) => return,
            (_, 0) => match bit {
                0..=11 => {
                    let d = (bit / 3) as usize;
                    let en = kk.dcm[d].ctr >> (7 + bit % 3) & 1 != 0;
                    (en, d)
                }
                12 => (kk.pll.ctr & 0x10 != 0, 4),
                13 | 14 => (kk.gt[(bit - 13) as usize].ctr & 0x80 != 0, 5),
                _ => return,
            },
            (_, 1) => {
                let n = bit as usize;
                let redirect = n % 2 == 1 && kk.srnr >> (n / 2) & 1 != 0;
                (kk.gtc[n].ctr & 8 != 0, if redirect { 22 + n / 2 } else { 6 + n / 2 })
            }
            (_, 2) => (enabled_ltc(&kk.ltc[bit as usize]), 22 + bit as usize / 4),
            _ => (enabled_ltc(&kk.ltc[32 + bit as usize]), 22 + (32 + bit as usize) / 4),
        };
        if node.0 {
            self.reqs.push(src_addr(k, node.1));
        }
    }
}

fn enabled_ltc(c: &Cell) -> bool {
    c.ctr & 8 != 0
}

/// Set_Data_Out (p.24-135/142).
fn set_data(out: &mut bool, mode: u8) {
    match mode {
        1 => *out = !*out,
        2 => *out = false,
        3 => *out = true,
        _ => {}
    }
}

/// Timer pulses from `tim` until a 24-bit global timer with reload `rev` equals `x` (>= 1).
fn gt_dist(tim: u32, x: u32, rev: u32) -> u64 {
    if x > tim {
        (x - tim) as u64
    } else if x >= rev {
        (0x100_0000 - tim) as u64 + (x - rev) as u64
    } else {
        u64::MAX / 4
    }
}

