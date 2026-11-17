//! GPTA0 / GPTA1 / LTCA2 (General Purpose Timer Array) model for the TC1791 (SIMOS18.1).
//!
//! Source: TC1796 User's Manual V2.0, chapter 24 (`vault/docs/infineon/tc1796_um_v2.0_2007_07.pdf`); page labels
//! below are the manual's (e.g. p.24-128). Cell behaviour follows the pseudo code of section 24.2.6
//! (p.24-115..24-147), register layouts section 24.3 (p.24-150..24-216), I/O line sharing 24.2.4
//! (p.24-90..24-111), service request groups Table 24-14 (p.24-113/114), EDCTR p.24-268.
//!
//! The pseudo code is specified "every GPTA clock"; this model is event driven: counters clocked by a
//! prescaled fGPTA line are advanced arithmetically and every tick at which something observable happens
//! (input edge, filter clock while a filter is busy, DCM compare, PLL micro tick, compare match, LTC
//! timer event) is evaluated with the pseudo-code semantics. Supported: FPC filter modes 0..5, PDL
//! feed-through (MUX = TSE = 0), DCM, PLL (incl. AEN/PEN compensation), clock bus (CKBCTR), GT0/GT1 (incl.
//! GT1 of GPTA1 driven by the GPTA0 PLL), GTC capture/compare/one-shot/capture-after-compare/GES/OIA/
//! EOA/M0-M1 action links, LTC capture/compare/free-running/reset timers with select lines and coherent
//! update, service request flags (SRSCx/SRSSx/SRNR) and groups, multiplexer register array (GIMCR/LIMCR
//! input routing decoded; OMCR kept raw). Not modelled: FPC prescaler modes 6/7, filter clocks that are not
//! a prescaled fGPTA, PDL forward/backward/3-sensor modes, uncompensated PLL clock lines, LTC63 bit-reverse
//! mode, LTC prescaler mode (PEN), OMCR output-pin mapping (outputs are
//! reported per cell), DBGCTR.
//!
//! Integration: the SRC registers (`base + 0x7FC - 4*node`) belong to the board's interrupt unit; this
//! model reports triggered nodes through [`Gpta::take_requests`] (board sets SRR). All other addresses
//! of `0xF0001800..0xF0002FFF` ([`Gpta::contains`]) go to [`Gpta::read_word`]/[`Gpta::write_lanes`] (board
//! `periph_read`/`periph_write` convention) or [`Gpta::mmio_read`]/[`Gpta::mmio_write`] (natural size).
//! Register accesses act at [`Gpta::now`]; the board calls [`Gpta::advance`] with the elapsed fGPTA ticks
//! (90 MHz on SIMOS18.1) and the pin edges of the interval, then drains requests and output events.

pub const GPTA0_BASE: u32 = 0xF000_1800;
pub const GPTA1_BASE: u32 = 0xF000_2000;
pub const LTCA2_BASE: u32 = 0xF000_2800;
/// fGPTA assumed by SIMOS18.1 (fFPI 90 MHz, FDR divider 1; GT0 = fGPTA/16 = 5.625 MHz firmware constant).
pub const F_GPTA_DEFAULT: u64 = 90_000_000;

const BASES: [u32; 3] = [GPTA0_BASE, GPTA1_BASE, LTCA2_BASE];

/// Service request control register of `node` in `kernel` (p.24-273).
pub fn src_addr(kernel: usize, node: usize) -> u32 {
    BASES[kernel] + 0x7FC - 4 * node as u32
}

/// An input pin edge: GPTA input line `line` (IN0..IN55) of `kernel` changes to `level` at GPTA tick `t`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InputEdge {
    pub t: u64,
    pub kernel: u8,
    pub line: u8,
    pub level: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CellKind {
    Gtc,
    Ltc,
}

/// A cell output (GTCkOUT / LTCkOUT) transition — read-only decode tap for actuator models.
/// `gt0`/`gt1` are the kernel's global timers at the transition (LTCA2: GPTA1's): GT0 = 5.625 MHz time base,
/// GT1 = angle base in PLL micro ticks (1/256 tooth = 6/256 °CRK on SIMOS18.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OutputEvent {
    /// fGPTA tick of the transition.
    pub t: u64,
    pub kernel: u8,
    pub kind: CellKind,
    pub cell: u8,
    pub level: bool,
    pub gt0: u32,
    pub gt1: u32,
}

/// Cell input routing decoded from the multiplexer register array.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Line {
    #[default]
    None,
    /// GPTA input line IN0..IN55.
    In(u8),
    /// FPC level output SOLk (FPC/INT group).
    Fpc(u8),
    /// External input INTk.
    Int(u8),
    /// GTC output.
    Gtc(u8),
    /// LTC output.
    Ltc(u8),
    /// Clock bus line CLKk.
    Clk(u8),
    /// PDL bus line PDLk.
    Pdl(u8),
}

const ROUTE_LINES: usize = 185;

impl Line {
    #[inline]
    const fn route_idx(self) -> usize {
        match self {
            Line::In(n) => (n & 63) as usize,
            Line::Gtc(n) => 64 + (n & 31) as usize,
            Line::Ltc(n) => 96 + (n & 63) as usize,
            Line::Fpc(n) => 160 + (n & 7) as usize,
            Line::Pdl(n) => 168 + (n & 3) as usize,
            Line::Int(n) => 172 + (n & 3) as usize,
            Line::Clk(n) => 176 + (n & 7) as usize,
            Line::None => 184,
        }
    }
}

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
    gtc_route: [u32; ROUTE_LINES],
    ltc_route: [u64; ROUTE_LINES],
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
            gtc_route: [0; ROUTE_LINES],
            ltc_route: [0; ROUTE_LINES],
            pins: 0,
            raw: [0; 0x200],
        }
    }
}

const LTC_MODE_CAPTURE: u32 = 0;
const LTC_MODE_COMPARE: u32 = 1;
/// No event scheduled.
const NEVER: u64 = u64::MAX;

const DUE_FPC: u8 = 1 << 0;
const DUE_DCM: u8 = 1 << 1;
const DUE_PLL: u8 = 1 << 2;
const DUE_GT: u8 = 1 << 3;
const DUE_ALL: u8 = DUE_FPC | DUE_DCM | DUE_PLL | DUE_GT;

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
    /// Compare-mode enabled GTCs (`MOD` compare and `CEN = 1`) hooked to `(kernel, gt)`, bit `n` = `GTCn`.
    cmp_mask: [[u32; 2]; 2],
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
    due_valid: u8,
    /// Kernel whose clock bus and global timers LTCA2 uses: GPTA1 on TC1796/TC1791, GPTA0 on
    /// devices without GPTA1 ([`Gpta::tc1784`]).
    ltca2_host: usize,
    /// MRADIN words per kernel (TC1791 firmware: 54/54/32, VERIFIED-T doc Appendix B).
    mra_words: [usize; 3],
    mra_gen: [u32; 3],
    /// Cell outputs that drive a port pin which is also read back as GPTA input line INn of every
    /// kernel: (kernel, kind, cell, n). Supplied by the board (port alternate function + OMCR).
    pin_loops: Vec<(u8, CellKind, u8, u8)>,
    /// Evaluate every busy filter tick with `fpc_step` (reference behaviour for the differential test).
    fpc_eager: bool,
    /// No internal event before this tick: lower bound of `next_internal` from the state left by the
    /// last full [`Gpta::advance`] (0 = unknown, after a register write). An advance without input edges
    /// that ends before it only moves `now` ([`Gpta::advance`]).
    quiet_until: u64,
    /// `quiet_until` is the next event itself (a lower bound only after a register write, an overdue clamp).
    quiet_exact: bool,
    /// The last `next_internal` clamped an overdue counter's event to the next clock tick.
    next_clamped: bool,
    /// `now` moved without `sync_all(now)` + `fpc_catch_up_all(now + 1)`; [`Gpta::settle`] does them
    /// where the counters are observed or changed (see [`Gpta::advance`]).
    unsettled: bool,
    /// Every advance takes the full path (reference behaviour for the differential test).
    settle_eager: bool,
    /// Per PLL: the tick whose (event-free, not yet emitted) pulse [`Gpta::pll_sync`] may emit when it brings
    /// the PLL to that tick, [`NEVER`] if none. The reference emits every pulse at its tick in step 4 of
    /// [`Gpta::process_tick`]: input edges, filter and DCM events of that tick (and GPTA1's PLL while GPTA0's
    /// pulses are consumed) come before it, steps 5/6 after it, and an advance ending at a tick includes its
    /// pulse. Only the first sync after that point may emit it: a pulse made due later at the same tick by a
    /// register write (kernel enable, DTR) belongs to the next advance.
    pll_vis: [u64; 2],
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
            cmp_mask: [[0; 2]; 2],
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
            due_valid: 0,
            ltca2_host: 1,
            mra_words: [54, 54, 32],
            mra_gen: [0; 3],
            pin_loops: Vec::new(),
            fpc_eager: false,
            quiet_until: 0,
            quiet_exact: false,
            next_clamped: false,
            unsettled: false,
            settle_eager: false,
            pll_vis: [NEVER; 2],
        }
    }

    /// Board wiring: cell outputs fed back as pin inputs (see the `pin_loops` field).
    pub fn set_pin_loops(&mut self, loops: &[(u8, CellKind, u8, u8)]) {
        if self.pin_loops != loops {
            self.pin_loops = loops.to_vec();
            self.quiet_until = 0;
            self.quiet_exact = false;
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
        // every PLL micro tick is an internal event here, also the pulses emitted lazily (`pll_safe_pulses`)
        let pulse = self.pll_pulse_next();
        let ev = self.next_event_bearing(cap);
        ev.into_iter().chain(pulse.filter(|&t| t <= cap)).min()
    }

    /// [`Gpta::next_event_tick`] without the event-free PLL pulses.
    fn next_event_bearing(&mut self, cap: u64) -> Option<u64> {
        if !self.settle_eager {
            if self.quiet_until > cap {
                return None; // next_internal >= quiet_until (see `advance`)
            }
            if self.quiet_exact && !self.routing_dirty {
                return (self.quiet_until <= cap).then_some(self.quiet_until);
            }
        }
        self.rebuild_routing();
        let t = self.next_internal(NEVER - 1);
        self.quiet_until = t;
        self.quiet_exact = !self.next_clamped;
        (t <= cap).then_some(t)
    }

    /// Next PLL micro tick (pulse or counter reload) of an enabled kernel after `now`, the ones at `now` done.
    fn pll_pulse_next(&mut self) -> Option<u64> {
        let now = self.now;
        let mut best = None;
        for k in 0..2 {
            if !self.kernel_on(k) {
                continue;
            }
            self.pll_sync(k, now);
            let p = &self.k[k].pll;
            let (aen, pen) = (p.ctr & 4 != 0, p.ctr & 8 != 0);
            let t = p.t.max(now);
            let next = if p.event {
                Some(t)
            } else if p.cnt == 0 {
                ((pen || !aen) && p.mti != 0).then_some(t)
            } else if pen || Self::neg(p.dtr) {
                Some(t)
            } else {
                Some((p.t + p.dtr as u64 / Self::pll_m(p) + 1).max(now)) // DTR turns negative then
            };
            best = best.into_iter().chain(next).min();
        }
        best
    }

    /// Seconds -> GPTA ticks.
    pub fn ticks(&self, seconds: f64) -> u64 {
        if !(seconds.is_finite() && seconds > 0.0) {
            return 0;
        }
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
    #[inline(always)]
    pub fn contains(addr: u32) -> bool {
        Self::split(addr).is_some()
    }

    #[inline(always)]
    fn split(addr: u32) -> Option<(usize, u32)> {
        let rel = addr.wrapping_sub(0xF000_1800);
        if rel >= 0x1800 {
            return None;
        }
        let k = (rel >> 11) as usize;
        let off = rel & 0x7FF;
        let first_src = if k == 2 { 0x7FC - 4 * 15 } else { 0x7FC - 4 * 37 };
        if off >= first_src {
            return None;
        }
        Some((k, off))
    }

    /// Input routing of GTCn (decoded GIMCR) — for tests and diagnostics.
    pub fn gtc_input(&self, kernel: usize, n: usize) -> Line {
        self.k.get(kernel).and_then(|k| k.gtc_src.get(n)).copied().unwrap_or(Line::None)
    }

    /// Input routing of LTCn (decoded LIMCR).
    pub fn ltc_input(&self, kernel: usize, n: usize) -> Line {
        self.k.get(kernel).and_then(|k| k.ltc_src.get(n)).copied().unwrap_or(Line::None)
    }

    /// Multiplexer register array words of kernel `k` in FIFO write order and MRACTL.MAEN (p.24-110).
    pub fn mra(&self, k: usize) -> (&[u32], bool) {
        match self.k.get(k) {
            Some(kk) => (&kk.mra, kk.mractl & 1 != 0),
            None => (&[], false),
        }
    }

    #[inline]
    pub fn mra_gen(&self, k: usize) -> u32 {
        self.mra_gen.get(k).copied().unwrap_or(0)
    }

    #[inline]
    pub fn mmxctr(&self) -> [u32; 4] {
        let r = &self.k[0].raw;
        let b = 0x700 / 4;
        [r[b], r[b + 1], r[b + 2], r[b + 3]]
    }

    /// Output line level of LTCn of `kernel` (the cell's OUT bit).
    pub fn ltc_out(&self, kernel: usize, n: usize) -> bool {
        self.k.get(kernel).and_then(|k| k.ltc.get(n)).is_some_and(|c| c.out)
    }

    /// Stored control bits of LTCn of `kernel` (MOD in bits 1:0), without read side effects.
    pub fn ltc_ctr(&self, kernel: usize, n: usize) -> u32 {
        self.k.get(kernel).and_then(|k| k.ltc.get(n)).map_or(0, |c| c.ctr)
    }

    /// Current global timer value (GTTIMm) of a GPTA kernel.
    pub fn gt_value(&mut self, kernel: usize, m: usize) -> u32 {
        if kernel >= 2 || m >= 2 {
            return 0;
        }
        self.settle();
        self.sync_gt(kernel, m, self.now);
        self.k[kernel].gt[m].tim
    }

    // ============================================================================================
    // Register interface
    // ============================================================================================

    /// Read with byte lanes (`size` 1/2/4, including cross-word unaligned accesses).
    pub fn mmio_read(&mut self, addr: u32, size: u8) -> u32 {
        let Some((k, off)) = Self::split(addr) else { return 0 };
        let boff = off & 3;
        let w0 = self.reg_read(k, off & !3);
        let sh = boff * 8;
        let m = lane_mask(size);
        if size <= 4 && boff + size as u32 > 4 {
            let w1 = self.read_word(addr.wrapping_sub(boff).wrapping_add(4)).unwrap_or(0);
            ((w0 >> sh) | (w1 << ((4 - boff) * 8))) & m
        } else {
            (w0 >> sh) & m
        }
    }

    /// Write with byte lanes (`val` is the naturally sized datum, including cross-word accesses).
    pub fn mmio_write(&mut self, addr: u32, val: u32, size: u8) {
        let Some((k, off)) = Self::split(addr) else { return };
        let boff = off & 3;
        let sh = boff * 8;
        let lane = lane_mask(size);
        let val32 = val & lane;
        let m0 = lane << sh;
        self.reg_write(k, off & !3, (val32 << sh) & m0, m0);
        if size <= 4 && boff + size as u32 > 4 {
            let hi_bytes = boff + size as u32 - 4;
            let m1 = (1u32 << (hi_bytes * 8)) - 1;
            let v1 = (val32 >> ((4 - boff) * 8)) & m1;
            self.write_lanes(addr.wrapping_sub(boff).wrapping_add(4), v1, m1);
        }
    }

    /// Side-effect-free host peek of a 32-bit aligned GPTA register word through `&self`.
    pub fn peek_word(&self, addr: u32) -> Option<u32> {
        let (k, off) = Self::split(addr)?;
        let off = off & !3;
        let now = self.now;
        if k == 2 && !matches!(off, 0x008 | 0x020..=0x02C | 0x038..=0x040 | 0x200..=0x3FF) {
            return Some(self.k[2].raw[off as usize / 4]);
        }
        Some(match off {
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
                        let g = &self.k[k].gt[m];
                        if let Clk::Div(d) = self.clk(k, ((g.ctr >> 4) & 7) as u8) {
                            if self.gt_running(k, m) && now > g.t {
                                let n = (now >> d) - (g.t >> d);
                                let room = 0x100_0000 - g.tim as u64;
                                if n < room {
                                    return Some(g.tim + n as u32);
                                }
                                let span = (0x100_0000 - g.rev as u64).max(1);
                                return Some((g.rev as u64 + (n - room) % span) as u32 & 0xFF_FFFF);
                            }
                        }
                        g.tim
                    }
                    _ => 0,
                }
            }
            0x100..=0x1FF => {
                let n = ((off - 0x100) / 8) as usize;
                let c = &self.k[k].gtc[n];
                if off & 4 == 0 {
                    (c.ctr & !(0x8000 | 0x4000 | 0x400)) | (c.out as u32) << 15 | (c.cen as u32) << 10
                } else {
                    c.x
                }
            }
            0x200..=0x3FF => {
                let n = ((off - 0x200) / 8) as usize;
                let c = &self.k[k].ltc[n];
                if off & 4 != 0 {
                    if Self::is_timer(c) {
                        if let Some((d, true)) = self.chain_clock(k, n) {
                            if now > c.t {
                                let max = self.chain_dist(k, n).map_or(u64::MAX, |x| x - 1);
                                let add = ((now >> d) - (c.t >> d)).min(max);
                                return Some((c.x as u64 + add) as u32 & 0xFFFF);
                            }
                        }
                    }
                    return Some(c.x);
                }
                let sel = self.ltc_select_in(k, n);
                let mut v = c.ctr & !(0x8000 | 0x4000 | 0x400);
                if c.ctr & 2 != 0 {
                    v = (v & !(0x40 | 0x80)) | (c.slv as u32) << 6;
                } else {
                    v = (v & !0x200) | (sel as u32) << 9;
                }
                v | (c.out as u32) << 15 | (c.cen as u32) << 10
            }
            0x400 if k == 0 => self.edctr,
            _ => self.k[k].raw[off as usize / 4],
        })
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
                self.pll_sync(k, now);
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
        if !matches!(off, 0x010..=0x030 | 0x044) {
            if !matches!(off, 0x100..=0x1FF) {
                self.quiet_exact = false;
            }
            if !matches!(off, 0x048..=0x077 | 0x080..=0x0BF | 0x0C0..=0x0D4 | 0x100..=0x3FF) {
                self.due_valid = 0;
            }
        }
        if !matches!(off, 0x010..=0x030 | 0x044 | 0x080..=0x0BF | 0x0C0..=0x0D4 | 0x100..=0x3FF) {
            // service request flags (SRSCx/SRSSx), SRNR and FPCSTAT touch no counter and no event time;
            // DCM, PLL and GTC/LTC writes sync only their own state and update quiet_until locally
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
                self.mra_gen[k] = self.mra_gen[k].wrapping_add(1);
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
                    self.mra_gen[k] = self.mra_gen[k].wrapping_add(1);
                }
            }
            0x044 => self.k[k].fpcstat = merge(self.k[k].fpcstat) & 0x3F3F,
            0x048..=0x077 => {
                self.due_valid &= !DUE_FPC;
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
                self.due_valid &= !DUE_DCM;
                match off & 0xF {
                    0x0 => {
                        let val = merge(self.k[k].dcm[d].ctr) & 0x3FF;
                        if val & 0x40 != 0 {
                            self.settle();
                            self.quiet_until = 0;
                            self.k[k].dcm[d].ctr = val & !0x40;
                            self.due_valid = 0;
                            self.dcm_out(k, d, now); // QCK: immediate output pulse (p.24-162)
                        } else {
                            self.k[k].dcm[d].ctr = val;
                        }
                    }
                    0x4 => {
                        let cur = self.dcm_tim(k, d, now);
                        let dc = &mut self.k[k].dcm[d];
                        dc.tim0 = merge(cur) & 0xFF_FFFF;
                        dc.t0 = now;
                        let (from, cov) = (dc.cmp_ok.max(now), dc.cov);
                        if self.kernel_on(k) {
                            let dist = (cov.wrapping_sub(self.dcm_tim(k, d, from))) & 0xFF_FFFF;
                            self.quiet_until = self.quiet_until.min(from + dist as u64);
                        }
                    }
                    0x8 => self.k[k].dcm[d].cav = merge(self.k[k].dcm[d].cav) & 0xFF_FFFF,
                    _ => {
                        self.k[k].dcm[d].cov = merge(self.k[k].dcm[d].cov) & 0xFF_FFFF;
                        self.k[k].dcm[d].cmp_ok = now;
                        if self.kernel_on(k) {
                            let cur = self.dcm_tim(k, d, now);
                            let dist = (self.k[k].dcm[d].cov.wrapping_sub(cur)) & 0xFF_FFFF;
                            self.quiet_until = self.quiet_until.min(now + dist as u64);
                        }
                    }
                }
            }
            0x0C0..=0x0D4 => {
                self.pll_sync(k, now);
                self.due_valid &= !(DUE_PLL | DUE_GT);
                let p = &mut self.k[k].pll;
                match off {
                    0x0C0 => p.ctr = (p.ctr & !m) | (v & m) & 0x1F,
                    0x0C4 => p.mti = merge(p.mti) & 0xFFFF,
                    0x0C8 => p.cnt = merge(p.cnt) & 0xFFFF,
                    0x0CC => p.stp = merge(p.stp) & 0xFFFF,
                    0x0D0 => p.rev = merge(p.rev) & 0xFF_FFFF,
                    _ => p.dtr = merge(p.dtr) & 0x1FF_FFFF,
                }
                if !self.routing_dirty && self.kernel_on(k) && !(0..2).any(|j| !self.kernel_on(j) && self.pll_live(j)) {
                    if let Some(ev) = self.pll_next(k) {
                        self.quiet_until = self.quiet_until.min(ev);
                    }
                } else {
                    self.quiet_until = 0;
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
        kk.gtc_route = [0; ROUTE_LINES];
        kk.ltc_route = [0; ROUTE_LINES];
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
                let line = match grp {
                    0 => Line::In(8 * g as u8 + l),
                    1 => Line::In(32 + 8 * g as u8 + l),
                    2 => Line::Ltc(8 * g as u8 + l),
                    3 => Line::Ltc(32 + 8 * g as u8 + l),
                    4 if l < 6 => Line::Fpc(l),
                    4 => Line::Int(l - 6),
                    _ => Line::None,
                };
                kk.gtc_src[cell] = line;
                if line != Line::None {
                    kk.gtc_route[line.route_idx()] |= 1u32 << cell;
                }
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
                let line = match grp {
                    0 => Line::In(8 * q + l),
                    1 => Line::In(32 + 8 * q + l),
                    2 => Line::Gtc(8 * q + l),
                    3 => Line::Clk(l),
                    4 if l < 4 => Line::Pdl(l),
                    4 => Line::Int(l - 4),
                    _ => Line::None,
                };
                kk.ltc_src[cell] = line;
                if line != Line::None {
                    kk.ltc_route[line.route_idx()] |= 1u64 << cell;
                }
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
        self.due_valid = 0;
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
    /// of a GT/LTC/PLL register itself, and to `now` by `Gpta::settle` before a register write, a
    /// filter TIM read or an input edge at `now` (lazy syncs compose, so consecutive advances equal one
    /// advance split at the call boundaries; checked against syncing at every end by
    /// `lazy_settle_matches_settling_every_advance`). Without input edges, an interval that ends before
    /// `quiet_until` (no event can be due) only moves `now`.
    pub fn advance(&mut self, ticks: u64, edges: &[InputEdge]) {
        let end = self.now + ticks;
        if edges.is_empty() && end < self.quiet_until && !self.settle_eager {
            self.now = end;
            self.unsettled = true;
            self.pll_vis = [end; 2];
            return;
        }
        if edges.first().is_some_and(|e| e.t <= self.now) {
            // an edge at the current tick acts on the state after that tick (captures, filter ticks)
            self.settle();
        }
        let mut ei = 0;
        loop {
            let te = edges.get(ei).map(|e| e.t.max(self.now));
            let (ti, exact) = if self.quiet_exact && !self.routing_dirty && !self.settle_eager {
                (self.quiet_until, true)
            } else {
                self.rebuild_routing();
                (self.next_internal(NEVER - 1), !self.next_clamped)
            };
            let t = te.unwrap_or(u64::MAX).min(ti).min(end);
            if t == end && te.is_none_or(|x| x > end) && ti > end {
                self.now = end;
                self.rest(ti, exact);
                break;
            }
            self.quiet_exact = false;
            self.now = t;
            self.pll_vis = [NEVER; 2];
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
                    self.rest(ti, !self.next_clamped);
                    break;
                }
            }
        }
    }

    /// End of a full advance at `now` with nothing due before `quiet` (`next_internal` computed at
    /// this or an earlier `now`: it only grows while nothing happens, so it bounds the next event).
    /// `exact`: `quiet` is the next event itself (not clamped from an overdue counter, see `next_clamped`).
    fn rest(&mut self, quiet: u64, exact: bool) {
        self.quiet_until = quiet;
        self.quiet_exact = exact;
        self.unsettled = true;
        self.pll_vis = [self.now; 2];
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
        self.rebuild_routing();
        let mut best = cap + 1;
        let now = self.now;
        let mut clamped = false;
        let eager = self.settle_eager || self.fpc_eager;
        let v_fpc = !eager && (self.due_valid & DUE_FPC != 0) && now <= self.due[0];
        let v_dcm = !eager && (self.due_valid & DUE_DCM != 0) && now <= self.due[1];
        let v_pll = !eager && (self.due_valid & DUE_PLL != 0) && now <= self.due[2];
        let v_gt = !eager && (self.due_valid & DUE_GT != 0) && now <= self.due[3];
        if v_fpc && v_dcm && v_pll && v_gt && !VERIFY_CACHE {
            best = best.min(self.due[0]).min(self.due[1]).min(self.due[2]).min(self.due[3]);
        } else {
            let mut due = self.due;
            let mut valid = self.due_valid;
            let can_cache = cap == NEVER - 1 && !eager;
            if !v_fpc || VERIFY_CACHE {
                let mut d0 = NEVER;
                for k in 0..2 {
                    if self.kernel_on(k) {
                        for f in 0..6 {
                            if let Some(t) = self.fpc_next(k, f) {
                                d0 = d0.min(t);
                            }
                        }
                    }
                }
                if v_fpc {
                    assert_eq!(self.due[0], d0, "stale DUE_FPC now {now}");
                }
                due[0] = d0;
                if can_cache {
                    valid |= DUE_FPC;
                }
            }
            if !v_dcm || VERIFY_CACHE {
                let mut d1 = NEVER;
                let mut dcm_ok = can_cache;
                for k in 0..2 {
                    if self.kernel_on(k) {
                        for d in 0..4 {
                            let dc = &self.k[k].dcm[d];
                            let from = dc.cmp_ok.max(now);
                            if from < dc.t0 {
                                dcm_ok = false;
                            }
                            let cur = self.dcm_tim(k, d, from);
                            let dist = (dc.cov.wrapping_sub(cur)) & 0xFF_FFFF;
                            d1 = d1.min(from + dist as u64);
                        }
                    }
                }
                if v_dcm {
                    assert_eq!(self.due[1], d1, "stale DUE_DCM now {now}");
                }
                due[1] = d1;
                if dcm_ok {
                    valid |= DUE_DCM;
                } else {
                    valid &= !DUE_DCM;
                }
            }
            if !v_pll || VERIFY_CACHE {
                let mut d2 = NEVER;
                for k in 0..2 {
                    if self.kernel_on(k) {
                        if let Some(t) = self.pll_next(k) {
                            d2 = d2.min(t);
                        }
                    }
                }
                if v_pll {
                    assert_eq!(self.due[2], d2, "stale DUE_PLL now {now}");
                }
                due[2] = d2;
                if can_cache {
                    valid |= DUE_PLL;
                }
            }
            if !v_gt || VERIFY_CACHE {
                let mut d3 = NEVER;
                let mut gt_ok = can_cache;
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
                    let mut mask = self.cmp_mask[k][m];
                    while mask != 0 {
                        let c = mask.trailing_zeros() as usize;
                        mask &= mask - 1;
                        let cell = &self.k[k].gtc[c];
                        n = n.min(gt_dist(g.tim, cell.x, g.rev));
                    }
                    // an overdue pulse (counter clock line re-routed / kernel re-enabled) is due at the next clock tick
                    let raw_ev = (base + n) * div;
                    let clamped_ev = now.div_ceil(div) * div;
                    if raw_ev < clamped_ev {
                        gt_ok = false;
                        clamped = true;
                    }
                    d3 = d3.min(raw_ev.max(clamped_ev));
                }
                if v_gt {
                    assert_eq!(self.due[3], d3, "stale DUE_GT now {now}");
                }
                due[3] = d3;
                if gt_ok {
                    valid |= DUE_GT;
                } else {
                    valid &= !DUE_GT;
                }
            }
            self.due = due;
            self.due_valid = valid;
            best = best.min(due[0]).min(due[1]).min(due[2]).min(due[3]);
        }
        // LTC timers on Div clocks: the earliest cached event is exact unless a chain is overdue
        // (then each chain is clamped to its next clock tick)
        let m = self.chains_min();
        if m >= now {
            best = best.min(m);
        } else {
            clamped = true;
            for i in 0..self.div_chains.len() {
                let (k, ti, d) = self.div_chains[i];
                let raw = self.chain_next(k as usize, ti as usize, d);
                if raw != NEVER {
                    let div = 1u64 << d;
                    best = best.min(raw.max(now.div_ceil(div) * div));
                }
            }
        }
        // an overdue counter's event (clamped above) moves when a read syncs that counter: then the result is only
        // a lower bound of the next event
        self.next_clamped = clamped || best < now;
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
        if pins || self.due_valid != DUE_ALL || t >= self.due[0] || t >= self.due[1] || t >= self.due[2] || t >= self.due[3] {
            let mut active = pins;
            // 2. FPC filter ticks
            if (self.due_valid & DUE_FPC == 0) || self.due[0] <= t {
                self.due_valid &= !DUE_FPC;
                for k in 0..2 {
                    if !self.kernel_on(k) {
                        continue;
                    }
                    for f in 0..6 {
                        if self.fpc_next(k, f) == Some(t) {
                            self.fpc_step(k, f, t);
                            active = true;
                        }
                    }
                }
            }
            // 3. DCM compare
            if (self.due_valid & DUE_DCM == 0) || self.due[1] <= t {
                self.due_valid &= !DUE_DCM;
                for k in 0..2 {
                    if !self.kernel_on(k) {
                        continue;
                    }
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
            if active || (self.due_valid & DUE_PLL == 0) || self.due[2] <= t {
                self.due_valid &= !DUE_PLL;
                // The step runs for both PLLs when something happened at t or an enabled kernel's PLL has a micro
                // tick at t; a disabled kernel's PLL is not scheduled itself (`next_internal`), so it ticks only then.
                let mut step = active;
                for k in 0..2 {
                    if self.kernel_on(k) {
                        self.pll_ff(k, t);
                        step |= self.pll_due_at(k, t);
                    }
                }
                for k in 0..2 {
                    self.pll_ff(k, t);
                    if step && self.pll_due_at(k, t) {
                        self.pll_tick(k, t);
                    }
                    self.pll_vis[k] = t;
                }
            }
            self.pll_vis = [t; 2];
            // 5. GTs on Div clocks: sync and evaluate equality compares on the timer event at t
            if (self.due_valid & DUE_GT == 0) || self.due[3] <= t {
                self.due_valid &= !DUE_GT;
                for i in 0..self.div_gts.len() {
                    let (k, m, d) = self.div_gts[i];
                    let (k, m) = (k as usize, m as usize);
                    if !self.gt_running(k, m) || !t.is_multiple_of(1u64 << d) || self.k[k].gt[m].cmp_t >= t {
                        continue;
                    }
                    self.sync_gt(k, m, t);
                    self.k[k].gt[m].cmp_t = t;
                    self.gt_compares(k, m, t);
                }
            }
        }
        self.pll_vis = [t; 2];
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
            self.pll_sync(k, t);
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
        self.due_valid &= !DUE_FPC;
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
        let idx = line.route_idx();
        let mut gm = self.k[k].gtc_route[idx];
        while gm != 0 {
            let n = gm.trailing_zeros() as usize;
            gm &= gm - 1;
            self.gtc_in(k, n, level, t);
        }
        let mut lm = self.k[k].ltc_route[idx];
        while lm != 0 {
            let n = lm.trailing_zeros() as usize;
            lm &= lm - 1;
            self.ltc_in(k, n, level, t);
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
        for kk in &mut self.k {
            for fp in &mut kk.fpc {
                fp.next_ok = fp.next_ok.max(now);
            }
        }
    }

    fn fpc_step(&mut self, k: usize, f: usize, t: u64) {
        self.due_valid &= !DUE_FPC;
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
        self.due_valid &= !DUE_DCM;
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
        self.due_valid &= !(DUE_PLL | DUE_GT);
        if self.k[k].pll.ctr & 3 == d as u32 {
            self.pll_ff(k, t);
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

    /// Number of upcoming PLL pulses that neither complete the MTI burst (`cnt == 0`) nor trigger a
    /// consumer event (GT overflow or GTC equality compare). During these pulses each pulse only
    /// decrements `cnt` and increments the running GT consumers.
    fn pll_safe_pulses(&self, k: usize) -> u64 {
        let p = &self.k[k].pll;
        if self.routing_dirty || p.event || p.ctr & 8 != 0 || p.cnt <= 1 {
            return 0;
        }
        // a disabled kernel's PLL that can still tick does so at the micro ticks of the enabled one (step 4 of
        // `process_tick`): those must stay events
        if (0..2).any(|j| !self.kernel_on(j) && self.pll_live(j)) {
            return 0;
        }
        let m = Self::pll_m(p) as i64;
        let rev = p.rev as i64;
        let d = if Self::neg(p.dtr) { p.dtr as i64 - (1 << 25) } else { p.dtr as i64 };
        if rev <= 0 || rev + m > 0x100_0000 || d < -m || d >= rev {
            return 0;
        }
        let mut safe = (p.cnt - 1) as u64;
        for &c in &self.pll_cons[k] {
            match c {
                Consumer::Gt(gk, gm) => {
                    let (gk, gm) = (gk as usize, gm as usize);
                    if self.gt_running(gk, gm) {
                        let g = &self.k[gk].gt[gm];
                        let mut n = 0x100_0000 - g.tim as u64;
                        let mut mask = self.cmp_mask[gk][gm];
                        while mask != 0 {
                            let ci = mask.trailing_zeros() as usize;
                            mask &= mask - 1;
                            let cell = &self.k[gk].gtc[ci];
                            n = n.min(gt_dist(g.tim, cell.x, g.rev));
                        }
                        safe = safe.min(n.saturating_sub(1));
                    }
                }
                Consumer::Ltc(_, _) => return 0,
            }
        }
        safe
    }

    /// The PLL can still emit a micro tick (pulse, counter reload or input event) without a register write.
    fn pll_live(&self, k: usize) -> bool {
        let p = &self.k[k].pll;
        p.event || p.cnt != 0 || ((p.ctr & 8 != 0 || p.ctr & 4 == 0) && p.mti != 0)
    }

    /// PLL `k`, brought to `t` by `pll_ff`, has its micro tick at `t`: an input event (also right after the tick
    /// `t` was done), else a pulse or counter reload not done yet (`p.t <= t`).
    fn pll_due_at(&self, k: usize, t: u64) -> bool {
        let p = &self.k[k].pll;
        let (aen, pen) = (p.ctr & 4 != 0, p.ctr & 8 != 0);
        p.event || (p.t <= t && ((p.cnt == 0 && (pen || !aen) && p.mti != 0) || (p.cnt != 0 && (pen || Self::neg(p.dtr)))))
    }

    /// Next tick at which the PLL emits an event-bearing pulse or reloads its counter (absent input events).
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
        let safe = self.pll_safe_pulses(k);
        if safe == 0 {
            if pen || Self::neg(p.dtr) {
                return Some(t);
            }
            // DTR is valid at p.t; it turns negative after floor(DTR/m)+1 ticks
            return Some((p.t + p.dtr as u64 / Self::pll_m(p) + 1).max(self.now));
        }
        let m = Self::pll_m(p) as i128;
        let rev = p.rev as i128;
        let d = (if Self::neg(p.dtr) { p.dtr as i64 - (1 << 25) } else { p.dtr as i64 }) as i128;
        let ticks = ((d + safe as i128 * (rev + m)).div_euclid(m) + 1) as u64;
        Some((p.t + ticks).max(self.now))
    }

    /// Advance the delta register (and any event-free pulses) through the ticks [pll.t, t).
    fn pll_ff(&mut self, k: usize, t: u64) {
        if t <= self.k[k].pll.t {
            return;
        }
        self.due_valid &= !DUE_PLL;
        self.rebuild_routing();
        // a PLL of a disabled kernel is not scheduled (`next_internal`): its pulses are emitted only by
        // `process_tick` steps that coincide with them, never lazily
        let on = self.kernel_on(k);
        let p = &mut self.k[k].pll;
        let mut n = t - p.t;
        p.t = t;
        let m = Self::pll_m(p) as i64;
        let rev = p.rev as i64;
        let mut d = if Self::neg(p.dtr) { p.dtr as i64 - (1 << 25) } else { p.dtr as i64 };
        let mut pulses = 0u64;
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
                pulses += k2 as u64;
            }
        }
        if n > 0 && rev + m > 0 {
            let y = (d + m) as i128 + n as i128 * rev as i128;
            let non_pulses = (y / (rev + m) as i128) as u64;
            pulses += n - non_pulses;
            d = (y % (rev + m) as i128) as i64 - m;
        }
        p.dtr = (d.rem_euclid(1 << 25)) as u32;
        let emitted = if on { pulses.min(p.cnt as u64) } else { 0 };
        if emitted > 0 {
            p.cnt -= emitted as u32;
            self.pll_pulses[k] += emitted;
            for i in 0..self.pll_cons[k].len() {
                if let Consumer::Gt(gk, gm) = self.pll_cons[k][i] {
                    let (gk, gm) = (gk as usize, gm as usize);
                    if self.gt_running(gk, gm) {
                        self.gt_add(gk, gm, emitted);
                    }
                }
            }
        }
    }

    /// Bring the PLL state to tick `t`, including a safe pulse that falls on `t` itself when the reference has
    /// emitted it by now (`pll_vis`).
    fn pll_sync(&mut self, k: usize, t: u64) {
        self.pll_ff(k, t);
        if self.pll_vis[k] != t {
            return;
        }
        self.pll_vis[k] = NEVER;
        if !self.kernel_on(k) {
            return;
        }
        let p = &self.k[k].pll;
        if p.t == t && p.cnt > 1 && !p.event && p.ctr & 8 == 0 && Self::neg(p.dtr) {
            self.pll_ff(k, t + 1);
        }
    }

    fn pll_tick(&mut self, k: usize, t: u64) {
        self.due_valid &= !DUE_PLL;
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

    fn sync_gt(&mut self, k: usize, m: usize, t: u64) {
        if k > 1 {
            return;
        }
        match self.clk(k, ((self.k[k].gt[m].ctr >> 4) & 7) as u8) {
            Clk::Div(d) => {
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
            Clk::Pll(p) => {
                self.pll_sync(p as usize, t);
                self.k[k].gt[m].t = self.k[k].gt[m].t.max(t);
            }
            _ => {
                self.k[k].gt[m].t = self.k[k].gt[m].t.max(t);
            }
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
            self.invalidate_gt_due(k, m);
            self.flag(k, 0, 13 + m as u32);
        }
    }

    /// Equality compares on a timer event of GT(k,m) (GTC algorithm p.24-131).
    fn gt_compares(&mut self, k: usize, m: usize, t: u64) {
        let tim = self.k[k].gt[m].tim;
        let mut mask = self.cmp_mask[k][m];
        while mask != 0 {
            let n = mask.trailing_zeros() as usize;
            mask &= mask - 1;
            let c = &self.k[k].gtc[n];
            if c.cen && c.x == tim {
                self.gtc_compare_event(k, n, t);
                mask |= self.cmp_mask[k][m] & !((2u32 << n).wrapping_sub(1));
            }
        }
    }

    #[inline]
    fn invalidate_gt_due(&mut self, k: usize, m: usize) {
        self.quiet_exact = false;
        if self.routing_dirty {
            self.due_valid &= !(DUE_PLL | DUE_GT);
            return;
        }
        match self.clk(k, ((self.k[k].gt[m].ctr >> 4) & 7) as u8) {
            Clk::Div(_) => self.due_valid &= !DUE_GT,
            Clk::Pll(_) => self.due_valid &= !DUE_PLL,
            _ => {}
        }
    }

    #[inline]
    fn update_cmp_mask(&mut self, k: usize, n: usize) {
        let c = &self.k[k].gtc[n];
        let bit = 1u32 << n;
        let (old0, old1) = (self.cmp_mask[k][0], self.cmp_mask[k][1]);
        self.cmp_mask[k][0] &= !bit;
        self.cmp_mask[k][1] &= !bit;
        if c.ctr & 2 != 0 && c.cen {
            self.cmp_mask[k][(c.ctr & 1) as usize] |= bit;
        }
        if self.cmp_mask[k][0] != old0 {
            self.invalidate_gt_due(k, 0);
        }
        if self.cmp_mask[k][1] != old1 {
            self.invalidate_gt_due(k, 1);
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
        let now = self.now;
        let c = &mut self.k[k].gtc[n];
        let val = (c.ctr & !m) | (v & m);
        let old = c.ctr;
        let old_cmp = old & 2 != 0 && c.cen;
        c.ctr = val & 0x39FF; // OUT, OIA, CEN not stored
        if m & 0x100 != 0 {
            c.cen = val & 0x100 == 0; // enabled by writing EOA = 0 (p.24-62)
        }
        let new_cmp = c.ctr & 2 != 0 && c.cen;
        let gt_m = (c.ctr & 1) as usize;
        let newly_enabled = new_cmp && (!old_cmp || ((old ^ c.ctr) & 1 != 0));
        self.update_cmp_mask(k, n);
        if newly_enabled {
            self.sync_gt(k, gt_m, now);
        }
        if v & m & 0x4000 != 0 {
            self.settle();
            self.k[k].gtc[n].oia = true;
            self.gtc_mux(k, n, n, now);
            self.quiet_until = 0;
            self.quiet_exact = false;
        } else if newly_enabled {
            self.update_quiet_for_gtc(k, n);
        }
    }

    fn gtc_xr_write(&mut self, k: usize, n: usize, x: u32) {
        let now = self.now;
        let c = &mut self.k[k].gtc[n];
        if c.wp {
            return;
        }
        let old_x = c.x;
        c.x = x;
        // Compare on X write access: equality or GES/TGE (algorithm p.24-133)
        if c.ctr & 2 != 0 && c.cen {
            let m = (c.ctr & 1) as usize;
            let ges = c.ctr & 0x10 != 0;
            if x != old_x {
                self.invalidate_gt_due(k, m);
            }
            self.sync_gt(k, m, now);
            if self.k[k].gt[m].tim == x || (ges && self.tge(k, m, x)) {
                self.settle();
                self.gtc_compare_event(k, n, now);
                self.quiet_until = 0;
                self.quiet_exact = false;
                return;
            }
            if x != old_x {
                self.update_quiet_for_gtc(k, n);
            }
        }
    }

    fn update_quiet_for_gtc(&mut self, k: usize, n: usize) {
        if self.routing_dirty {
            self.quiet_until = 0;
            return;
        }
        let c = &self.k[k].gtc[n];
        if c.ctr & 2 == 0 || !c.cen {
            return;
        }
        let m = (c.ctr & 1) as usize;
        if !self.gt_running(k, m) {
            return;
        }
        let g = &self.k[k].gt[m];
        let dist = gt_dist(g.tim, c.x, g.rev);
        match self.clk(k, ((g.ctr >> 4) & 7) as u8) {
            Clk::Div(d) => {
                let div = 1u64 << d;
                let base = g.t / div;
                let ev = ((base + dist) * div).max(self.now.div_ceil(div) * div);
                self.quiet_until = self.quiet_until.min(ev);
            }
            Clk::Pll(p) => {
                let pi = p as usize;
                let pl = &self.k[pi].pll;
                let m_pll = Self::pll_m(pl) as i64;
                let rev = pl.rev as i64;
                let d = if Self::neg(pl.dtr) { pl.dtr as i64 - (1 << 25) } else { pl.dtr as i64 };
                let ok_kernels = self.kernel_on(pi) && !(0..2).any(|j| !self.kernel_on(j) && self.pll_live(j));
                if ok_kernels && !pl.event && pl.ctr & 8 == 0 && pl.cnt > 1 && dist > 1 && rev > 0 && rev + m_pll <= 0x100_0000 && d >= -m_pll && d < rev {
                    let safe = (dist - 1).min((pl.cnt - 1) as u64);
                    let ticks = ((d as i128 + safe as i128 * (rev + m_pll) as i128).div_euclid(m_pll as i128) + 1) as u64;
                    let ev = (pl.t + ticks).max(self.now);
                    self.quiet_until = self.quiet_until.min(ev);
                } else if ok_kernels {
                    if let Some(ev) = self.pll_next(pi) {
                        self.quiet_until = self.quiet_until.min(ev);
                    } else {
                        self.quiet_until = 0;
                    }
                } else {
                    self.quiet_until = 0;
                }
            }
            _ => self.quiet_until = 0,
        }
    }

    fn gtc_compare_event(&mut self, k: usize, n: usize, t: u64) {
        let ctr = self.k[k].gtc[n].ctr;
        self.invalidate_gt_due(k, (ctr & 1) as usize);
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
            self.update_cmp_mask(k, n);
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
            let enable_linked = c.ctr & 0x100 != 0 && mi != 0;
            let gt_m = (c.ctr & 1) as usize;
            if enable_linked {
                c.cen = true;
                c.ctr &= !0x100;
            }
            c.mode_out = mo;
            c.ev = false;
            c.oia = false;
            let out = c.out;
            if enable_linked {
                self.due_valid &= !(DUE_PLL | DUE_GT);
                self.update_cmp_mask(k, n);
                self.sync_gt(k, gt_m, t);
            }
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
        if !self.routing_dirty && !Self::is_timer(&self.k[k].ltc[n]) {
            let h = self.chain_head[k][n];
            return (h != NO_HEAD).then_some(h as usize);
        }
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
        if !self.routing_dirty && self.div_gated[k] >> ti & 1 == 0 {
            let slot = self.chain_slot[k][ti];
            if slot != NO_HEAD {
                let d = self.div_chains[slot as usize].2;
                self.sync_chain_at(k, ti, d, true, t);
                return;
            }
        }
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
        if !self.routing_dirty {
            let h = self.chain_head[k][n];
            if h != NO_HEAD {
                self.sync_chain(k, h as usize, t);
            }
            return;
        }
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
        if ((self.k[k].ltc[n].ctr ^ v) & m) & 2 != 0 {
            self.settle();
            self.quiet_until = 0;
        }
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
        if (old ^ ctr) & 2 != 0 || (timer && (old ^ ctr) & 0x130 != 0) {
            self.routing_dirty = true;
        }
        if v & m & 0x4000 != 0 {
            self.settle();
            self.k[k].ltc[n].oia = true;
            self.ltc_mux(k, n, n, now);
            self.quiet_until = 0;
        } else {
            self.update_quiet_for_ltc(k, n);
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
                self.settle();
                self.ltc_mux(k, lo, hi, now);
                self.quiet_until = 0;
                return;
            }
        } else if c.ctr & 3 == LTC_MODE_COMPARE && c.cen && Self::ltc_active(c, self.ltc_select_in(k, n)) && x == self.ltc_y_in(k, n) {
            self.settle();
            self.ltc_event(k, n);
            if let Some(ti) = self.ltc_timer_of(k, n) {
                let ch = self.chain(k, ti);
                self.ltc_event_in(k, ti, ch);
            }
            self.ltc_mux(k, n, n, now);
            self.quiet_until = 0;
            return;
        }
        self.update_quiet_for_ltc(k, n);
    }

    fn update_quiet_for_ltc(&mut self, k: usize, n: usize) {
        if self.routing_dirty {
            self.quiet_until = 0;
            return;
        }
        let ti = self.chain_head[k][n];
        if ti == NO_HEAD {
            return;
        }
        let ti = ti as usize;
        let slot = self.chain_slot[k][ti];
        if slot == NO_HEAD {
            self.quiet_until = 0;
            return;
        }
        let d = self.div_chains[slot as usize].2;
        let raw = self.chain_next(k, ti, d);
        if raw != NEVER {
            let div = 1u64 << d;
            let ev = raw.max(self.now.div_ceil(div) * div);
            self.quiet_until = self.quiet_until.min(ev);
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

#[cfg(test)]
mod tests {
    use super::*;

    fn g0(off: u32) -> u32 {
        GPTA0_BASE + off
    }

    fn run_edges(g: &mut Gpta, ticks: u64, edges: &[InputEdge]) {
        g.advance(ticks, edges);
    }

    /// Filter chain wrap-around (FPC0 input = FPC5 output): at a tick where FPC5 switches, FPC0 has
    /// already been evaluated with the old input (filter order 0..5), lazily or not.
    #[test]
    fn lazy_filter_chain_wraps_from_fpc5_to_fpc0() {
        let run = |eager: bool| {
            let mut g = Gpta { fpc_eager: eager, ..Default::default() };
            g.mmio_write(g0(0x400), 0x70F, 4);
            g.mmio_write(g0(0x48 + 8 * 5), 0x0001_0000, 4); // FPC5: MOD 1 (immediate), CMP 0, IN10, fGPTA
            g.mmio_write(g0(0x48), 0x0028_000A, 4); // FPC0: MOD 0 (delayed), CMP 10, IPS 5 (FPC5), fGPTA
            let e = |t, level| InputEdge { t, kernel: 0, line: 10, level };
            g.advance(107, &[e(100, true), e(105, false)]);
            let mid = g.mmio_read(g0(0x4C), 4);
            g.advance(100, &[]);
            (mid, g.mmio_read(g0(0x4C), 4), g.mmio_read(g0(0x44), 4))
        };
        assert_eq!(run(false), run(true));
    }

    /// Lazily applied pure filter ticks (`fpc_catch_up`) against evaluating every busy filter tick
    /// (`fpc_eager`): random filter modes/thresholds/clocks/chaining, pin edges, TIM and control writes,
    /// clock-bus and kernel-enable changes; every register read, DCM capture (exact edge tick) and
    /// service request must agree.
    #[test]
    fn lazy_filter_ticks_match_tick_by_tick_evaluation() {
        let mut rng = 0x2545_F491_4F6C_DD1Du64;
        let mut rnd = |n: u64| {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            rng % n
        };
        for _case in 0..400 {
            let (mut lazy, mut eager) = (Gpta::default(), Gpta::default());
            eager.fpc_eager = true;
            let mut ops: Vec<(u32, u32)> = vec![(0xD8, 0x31_C846), (0x400, 0x70F)];
            for f in 0..6u32 {
                // CMP small, MOD 0..5, IPS 0..2 or 5 (chained), CIN 0/2/3, reset bit random
                let ips = [0, 1, 2, 5, 5][rnd(5) as usize];
                let cin = [0, 2, 3][rnd(3) as usize];
                let ctr = rnd(12) as u32 | (rnd(6) as u32) << 16 | ips << 19 | cin << 22 | (rnd(2) as u32) << 24;
                ops.push((0x48 + 8 * f, ctr));
            }
            for d in 0..4u32 {
                ops.push((0x80 + 16 * d, 0x183 | (rnd(4) as u32) << 4)); // RCA+OCA captures, requests on
            }
            for (off, v) in ops {
                lazy.mmio_write(g0(off), v, 4);
                eager.mmio_write(g0(off), v, 4);
            }
            let mut levels = 0u64;
            for step in 0..400 {
                match rnd(10) {
                    0 => {
                        let off = 0x4C + 8 * rnd(6) as u32; // TIM write
                        let v = rnd(14) as u32;
                        lazy.mmio_write(g0(off), v, 4);
                        eager.mmio_write(g0(off), v, 4);
                    }
                    1 => {
                        let off = 0x48 + 8 * rnd(6) as u32; // control write
                        let v = rnd(12) as u32 | (rnd(6) as u32) << 16 | (rnd(3) as u32) << 19 | ([0, 2, 3][rnd(3) as usize]) << 22;
                        lazy.mmio_write(g0(off), v, 4);
                        eager.mmio_write(g0(off), v, 4);
                    }
                    2 => {
                        let v = [0x31_C846u32, 0x31_C842, 0x31_C840, 0x31_C84F][rnd(4) as usize];
                        lazy.mmio_write(g0(0xD8), v, 4);
                        eager.mmio_write(g0(0xD8), v, 4);
                    }
                    3 if step % 7 == 0 => {
                        let v = [0x70Fu32, 0x60F][rnd(2) as usize]; // GPTA0 kernel off / on
                        lazy.mmio_write(g0(0x400), v, 4);
                        eager.mmio_write(g0(0x400), v, 4);
                    }
                    _ => {
                        // a burst of edges on the filter input lines (IN 2f + 12 IPS)
                        let ticks = 1 + rnd(3000);
                        let t0 = lazy.now();
                        let mut edges = Vec::new();
                        let mut t = t0;
                        for _ in 0..rnd(4) {
                            t += 1 + rnd(ticks / 2 + 1);
                            if t > t0 + ticks {
                                break;
                            }
                            let line = (2 * rnd(6) + 12 * rnd(3)) as u8;
                            levels ^= 1 << line;
                            edges.push(InputEdge { t, kernel: 0, line, level: levels >> line & 1 != 0 });
                        }
                        lazy.advance(ticks, &edges);
                        eager.advance(ticks, &edges);
                    }
                }
                assert_eq!(lazy.take_requests(), eager.take_requests(), "requests, step {step}");
                if step % 3 == 0 {
                    for off in (0x44..0xC0).step_by(4) {
                        assert_eq!(lazy.mmio_read(g0(off), 4), eager.mmio_read(g0(off), 4), "reg {off:#x}, step {step}");
                    }
                }
            }
        }
    }

    /// Lazy settling and the quiet fast path of `advance` against syncing every lazily advanced counter at
    /// the end of every advance (`settle_eager`, the behaviour before): random LTC timer chains (clock-bus,
    /// edge-clocked and level-gated timers with compare cells and output links), GTC compares/captures on
    /// the global timers, filters and DCMs, pin edges, register writes and reads, advances from 0 ticks to
    /// many events, on GPTA0/GPTA1 and the TC1784 GPTA0/LTCA2. Every request, output transition (with its
    /// GT0/GT1 stamps), register read and `next_event_tick` must agree.
    #[test]
    fn lazy_settle_matches_settling_every_advance() {
        let mut rng = 0x9E37_79B9_7F4A_7C15u64;
        let mut rnd = move |n: u64| {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            rng % n.max(1)
        };
        let mut fast = 0u64;
        for case in 0..300 {
            let tc1784 = case % 3 == 2;
            let (mut lazy, mut eager) = if tc1784 { (Gpta::tc1784(40_000_000), Gpta::tc1784(40_000_000)) } else { (Gpta::default(), Gpta::default()) };
            eager.settle_eager = true;
            let kernels = if tc1784 { [0usize, 2] } else { [0, 1] };
            let mut ops: Vec<(u32, u32)> = vec![(GPTA0_BASE + 0xD8, [0x31_C846u32, 0xC846, 0x5, 0x31_C842][rnd(4) as usize])];
            if !tc1784 {
                ops.push((GPTA1_BASE + 0xD8, [0x1C846u32, 0xC846, 0x31_C846][rnd(3) as usize]));
            }
            let mut lines = vec![10u8, 24];
            let mut ltc_regs = Vec::new();
            let mut gtc_regs = Vec::new();
            for &k in &kernels {
                let base = BASES[k];
                let n = if tc1784 && k == 2 { 24 } else { 54 };
                let mut mra = vec![0u32; n];
                let lim = if k == 2 { n - 16 } else { n - 24 };
                let groups = if k == 2 { 4 } else { 8 };
                for _ in 0..1 + rnd(3) {
                    let c0 = 8 * rnd(groups) as u32 + 4 * rnd(2) as u32; // timer cell, then 3 compare cells
                    let (g, hi, j) = (c0 / 8, c0 % 8 >= 4, c0 % 4);
                    let clocked = rnd(3) != 0;
                    let byte = if clocked { 0xB0 | [0u32, 2, 4, 6, 7][rnd(5) as usize] } else { 0x80 | rnd(8) as u32 };
                    if !clocked {
                        lines.push((8 * (g % 4) + (byte & 7)) as u8);
                    }
                    mra[lim + 2 * (7 - g as usize) + !hi as usize] |= byte << (8 * j);
                    let tctr = if clocked { [0x0Bu32, 0x110B, 0x5903, 0x0A, 0x903, 0x02][rnd(6) as usize] } else { [0x902u32, 0x90A, 0x0B, 0x13, 0x0A][rnd(5) as usize] };
                    ops.push((base + 0x204 + 8 * c0, rnd(0x300) as u32));
                    for c in c0 + 1..c0 + 4 {
                        ops.push((base + 0x204 + 8 * c, [rnd(0x300), 0xFFFE, rnd(0x40)][rnd(3) as usize] as u32));
                        ops.push((base + 0x200 + 8 * c, [0x2031u32, 0x3831, 0x3011, 0x3021, 0x31, 0x1839, 0x2039, 0x3839][rnd(8) as usize]));
                        ltc_regs.push(base + 0x200 + 8 * c);
                    }
                    ops.push((base + 0x200 + 8 * c0, tctr));
                    ltc_regs.push(base + 0x200 + 8 * c0);
                }
                if k != 2 {
                    for _ in 0..rnd(4) {
                        let c = 8 + rnd(16) as u32;
                        if rnd(2) == 0 {
                            ops.push((base + 0x104 + 8 * c, rnd(0x400) as u32));
                            ops.push((base + 0x100 + 8 * c, [0x1802u32, 0x1806, 0x1812, 0x180A, 0x180B][rnd(5) as usize]));
                        } else {
                            let l = rnd(8) as u32;
                            mra[n - 8 + 2 * (3 - c as usize / 8) + (c % 8 < 4) as usize] |= (0x80 | l) << (8 * (c % 4));
                            lines.push((8 * (c / 8) + l) as u8);
                            ops.push((base + 0x100 + 8 * c, [0x28u32, 0x18, 0x38, 0x2A][rnd(4) as usize]));
                        }
                        gtc_regs.push(base + 0x100 + 8 * c);
                    }
                    ops.push((base + 0xE0, (0x4E | 0x80) & [0xFF, 0x7F][rnd(2) as usize]));
                    ops.push((base + 0xF0, [0x2E, 0x6E][rnd(2) as usize]));
                    for f in 0..6u32 {
                        if rnd(3) == 0 {
                            let ctr = rnd(12) as u32 | (rnd(6) as u32) << 16 | (rnd(3) as u32) << 19 | ([0, 2, 3][rnd(3) as usize]) << 22;
                            ops.push((base + 0x48 + 8 * f, ctr));
                        }
                    }
                    for d in 0..4u32 {
                        ops.push((base + 0x80 + 16 * d, 0x183 | (rnd(4) as u32) << 4));
                    }
                }
                ops.push((base + 0x38, 2));
                ops.extend(mra.iter().map(|&w| (base + 0x3C, w)));
                ops.push((base + 0x38, 1));
            }
            ops.push((GPTA0_BASE + 0x400, if tc1784 { 0x503 } else { 0x70F }));
            for (a, v) in ops {
                lazy.mmio_write(a, v, 4);
                eager.mmio_write(a, v, 4);
            }
            let mut levels = [0u64; 3];
            for step in 0..300 {
                let both = |g: &mut Gpta, a: u32, v: u32| g.mmio_write(a, v, 4);
                match rnd(24) {
                    0 if !ltc_regs.is_empty() => {
                        let a = ltc_regs[rnd(ltc_regs.len() as u64) as usize] + 4;
                        let v = rnd(0x400) as u32;
                        both(&mut lazy, a, v);
                        both(&mut eager, a, v);
                    }
                    1 if !ltc_regs.is_empty() => {
                        // compare-cell rewrite or the firmware's coherent duty update (CUD by read-modify-write)
                        let a = ltc_regs[rnd(ltc_regs.len() as u64) as usize];
                        let v = (lazy.mmio_read(a, 4) | (0x200 * rnd(2) as u32)) & !0x8000;
                        assert_eq!(v, (eager.mmio_read(a, 4) | (0x200 * (v >> 9 & 1))) & !0x8000, "CTR {a:#x}, step {step}");
                        both(&mut lazy, a, v);
                        both(&mut eager, a, v);
                    }
                    2 if !gtc_regs.is_empty() => {
                        let a = gtc_regs[rnd(gtc_regs.len() as u64) as usize] + 4;
                        let v = rnd(0x800) as u32;
                        both(&mut lazy, a, v);
                        both(&mut eager, a, v);
                    }
                    3 => {
                        let a = BASES[kernels[0]] + 0xE8;
                        let v = rnd(0x1000) as u32;
                        both(&mut lazy, a, v);
                        both(&mut eager, a, v);
                    }
                    4 if step % 5 == 0 => {
                        let v = [0x31_C846u32, 0xC846, 0x5, 0x31_C840][rnd(4) as usize];
                        both(&mut lazy, GPTA0_BASE + 0xD8, v);
                        both(&mut eager, GPTA0_BASE + 0xD8, v);
                    }
                    5 if step % 7 == 0 => {
                        let v = [0x70Fu32, 0x60F, 0x503, 0x103][rnd(4) as usize];
                        both(&mut lazy, GPTA0_BASE + 0x400, v);
                        both(&mut eager, GPTA0_BASE + 0x400, v);
                    }
                    6 => {
                        for off in [0x10u32, 0x18, 0x20, 0x28] {
                            both(&mut lazy, BASES[kernels[1]] + off, !0);
                            both(&mut eager, BASES[kernels[1]] + off, !0);
                        }
                    }
                    7 => {
                        for &a in ltc_regs.iter().chain(&gtc_regs) {
                            for a in [a, a + 4] {
                                assert_eq!(lazy.mmio_read(a, 4), eager.mmio_read(a, 4), "reg {a:#x}, step {step}");
                            }
                        }
                        for k in kernels {
                            for off in (0x44..0x100).step_by(4) {
                                let a = BASES[k] + off;
                                assert_eq!(lazy.mmio_read(a, 4), eager.mmio_read(a, 4), "reg {a:#x}, step {step}");
                            }
                        }
                    }
                    r => {
                        let ticks = match r % 4 {
                            0 => rnd(2),
                            1 => 1 + rnd(40),
                            2 => 1 + rnd(2000),
                            _ => 1 + rnd(60_000),
                        };
                        let t0 = lazy.now();
                        let mut edges = Vec::new();
                        if rnd(3) == 0 {
                            let mut t = t0;
                            for _ in 0..1 + rnd(4) {
                                t += rnd(ticks / 2 + 1);
                                if t > t0 + ticks {
                                    break;
                                }
                                let k = kernels[rnd(2) as usize];
                                let line = lines[rnd(lines.len() as u64) as usize];
                                levels[k] ^= 1 << line;
                                edges.push(InputEdge { t, kernel: k as u8, line, level: levels[k] >> line & 1 != 0 });
                            }
                        }
                        fast += (edges.is_empty() && t0 + ticks < lazy.quiet_until) as u64;
                        lazy.advance(ticks, &edges);
                        eager.advance(ticks, &edges);
                    }
                }
                assert_eq!(lazy.take_requests(), eager.take_requests(), "requests, case {case} step {step}");
                assert_eq!(lazy.take_outputs(), eager.take_outputs(), "outputs, case {case} step {step}");
                let cap = lazy.now() + rnd(4000);
                assert_eq!(lazy.next_event_tick(cap), eager.next_event_tick(cap), "next event, case {case} step {step}");
            }
            for &k in &kernels {
                for m in 0..2 {
                    if k < 2 {
                        assert_eq!(lazy.gt_value(k, m), eager.gt_value(k, m));
                    }
                }
            }
        }
        assert!(fast > 10_000, "fast path exercised only {fast} times");
    }

    #[test]
    fn global_timer_prescaled_and_overflow() {
        let mut g = Gpta::default();
        g.mmio_write(g0(0xD8), 0xC846, 4); // CLK4 = fGPTA/16
        g.mmio_write(g0(0xE0), 0x4E | 0x80, 4); // GT0 on CLK4, overflow request enabled
        g.mmio_write(g0(0x400), 0x70F, 4);
        g.advance(16 * 1000, &[]);
        assert_eq!(g.gt_value(0, 0), 1000);
        g.mmio_write(g0(0xE8), 0xFF_FFFE, 4);
        g.advance(32, &[]);
        assert_eq!(g.gt_value(0, 0), 0);
        assert_eq!(g.mmio_read(g0(0x10), 4) & (1 << 13), 1 << 13);
        assert_eq!(g.take_requests(), vec![src_addr(0, 5)]);
    }

    #[test]
    fn fpc_immediate_falling_filter_and_dcm_period() {
        // FPC0: CMP 70, MOD 3 (rising unfiltered, falling immediate), IPS 2 (IN24), CLK2 = fGPTA/64
        let mut g = Gpta::default();
        g.mmio_write(g0(0xD8), 0xC846, 4);
        g.mmio_write(g0(0x400), 0x70F, 4);
        g.mmio_write(g0(0x48), 0x0093_0046, 4);
        g.mmio_write(g0(0x80), 0x129, 4); // DCM0: RCA, FZE, FCK, FRE
        let e = |t, level| InputEdge { t, kernel: 0, line: 24, level };
        // idle high, then a falling edge with a 20 us glitch-high and a long low
        run_edges(&mut g, 10_000, &[e(0, true)]);
        g.take_requests();
        run_edges(&mut g, 20_000, &[e(12_000, false), e(13_000, true), e(13_200, false), e(16_000, true)]);
        // falling passed immediately (at the next /64 tick), rising at 13000 is inside the 70*64 lockout
        // => output rises only after the lockout (and input low again at 13200 -> no rise until 16000)
        let falls = g.mmio_read(g0(0x14), 4) & 2;
        assert_eq!(falls, 2);
        // glitch flags: rising+falling edge seen during the delay
        assert_ne!(g.mmio_read(g0(0x44), 4) & 0x0101, 0);
        // DCM0 captured the low time on the rising edge (~16000-12032)
        let cav = g.mmio_read(g0(0x88), 4);
        // falling seen at the next /64 filter tick (12032), 70+1 filter ticks lockout, rising at 16576
        assert_eq!(cav, 16576 - 12033);
        assert_eq!(g.take_requests(), vec![src_addr(0, 0)]); // FRE only
    }

    #[test]
    fn pll_generates_mti_pulses_per_period_and_gt1_counts() {
        let mut g = Gpta::default();
        g.mmio_write(g0(0xD8), 0xC846, 4);
        g.mmio_write(g0(0x400), 0x70F, 4);
        g.mmio_write(g0(0xF0), 0x1E, 4); // GT1 on CLK1 = local PLL
        g.mmio_write(GPTA1_BASE + 0xD8, 0x1C846, 4); // G1 CLK3 = PLL of GPTA0
        g.mmio_write(GPTA1_BASE + 0xF0, 0x3E, 4);
        g.mmio_write(g0(0x80), 0x129, 4);
        g.mmio_write(g0(0x48), 0x0093_0000, 4); // no filtering (CMP 0)
        g.mmio_write(g0(0xC4), 0x100, 4);
        g.mmio_write(g0(0xCC), 0xFF00, 4);
        let period = 9000u64; // 100 us teeth
        g.mmio_write(g0(0xD0), (period - 256) as u32, 4);
        g.mmio_write(g0(0xC0), 0x14, 4); // REN, AEN, MUX 0 (DCM0)
        let mut edges = vec![InputEdge { t: 0, kernel: 0, line: 24, level: true }];
        for i in 1..=10u64 {
            edges.push(InputEdge { t: i * period, kernel: 0, line: 24, level: false });
            edges.push(InputEdge { t: i * period + 3000, kernel: 0, line: 24, level: true });
        }
        g.advance(10 * period + 5000, &edges);
        let gt1 = g.gt_value(0, 1);
        // AEN: exactly MTI micro ticks per tooth period from the first falling edge at 9000 on;
        // the run ends 5000 ticks into the 10th period
        assert!((9 * 256 + 140..9 * 256 + 145).contains(&gt1), "gt1 {gt1}");
        assert_eq!(g.gt_value(1, 1), gt1, "GPTA1.GT1 mirrors GPTA0 PLL");
        assert!(g.take_requests().contains(&src_addr(0, 4)));
    }

    #[test]
    fn pll_compensation_aen_stops_after_mti_and_catches_up() {
        let mut g = Gpta::default();
        g.mmio_write(g0(0x400), 0x70F, 4);
        g.mmio_write(g0(0xF0), 0x1E, 4);
        g.mmio_write(g0(0xC4), 0x100, 4);
        g.mmio_write(g0(0xCC), 0xFF00, 4);
        g.mmio_write(g0(0xD0), 9000 - 256, 4);
        g.mmio_write(g0(0xC0), 0x4, 4); // AEN
        g.mmio_write(g0(0x80), 0x20, 4); // FCK: pulse on falling edge
        let e = |t, level| InputEdge { t, kernel: 0, line: 24, level };
        g.mmio_write(g0(0x48), 0x0010_0000, 4); // IPS 2, delayed filter CMP 0 = bypass on fGPTA
        // accelerating input: the 2nd tooth arrives after ~4000 of the predicted 9000 ticks ->
        // PEN: the remaining micro ticks are issued at fGPTA, then a new period of MTI starts
        g.advance(4300, &[e(0, true), e(50, false), e(2000, true), e(4050, false)]);
        let v = g.gt_value(0, 1);
        assert!((256..=275).contains(&v), "gt1 {v}");
        // decelerating input (no further edge): exactly MTI pulses for the 2nd period, then the PLL stops
        g.advance(100_000, &[]);
        assert_eq!(g.gt_value(0, 1), 512);
    }

    #[test]
    fn gtc_capture_and_compare_with_links() {
        let mut g = Gpta::default();
        g.mmio_write(g0(0xD8), 0xC846, 4);
        g.mmio_write(g0(0xE0), 0x4E, 4);
        g.mmio_write(g0(0x400), 0x70F, 4);
        // MRA: route GTC28 <- FPC3, GTC30 <- FPC0, GTC31 <- IN24 (words of the SIMOS boot)
        g.mmio_write(g0(0x38), 2, 4);
        for i in 0..54 {
            let w = match i {
                46 => 0x80C0_C3C3,
                47 => 0xC5C5_0000,
                _ => 0,
            };
            g.mmio_write(g0(0x3C), w, 4);
        }
        g.mmio_write(g0(0x38), 1, 4);
        assert_eq!(g.gtc_input(0, 28), Line::Fpc(3));
        assert_eq!(g.gtc_input(0, 26), Line::Fpc(5));
        assert_eq!(g.gtc_input(0, 30), Line::Fpc(0));
        assert_eq!(g.gtc_input(0, 31), Line::In(24));
        // GTC31: capture GT0 on falling edges of IN24 with request
        g.mmio_write(g0(0x1F8), 0x28, 4);
        g.advance(1600, &[InputEdge { t: 0, kernel: 0, line: 24, level: true }, InputEdge { t: 1600, kernel: 0, line: 24, level: false }]);
        assert_eq!(g.mmio_read(g0(0x1FC), 4), 100);
        assert_eq!(g.take_requests(), vec![src_addr(0, 21)]);
        // GTC12 compare GT0 at 150, set output; GTC13 (OCM=100 link) follows; GTC14 hold
        g.mmio_write(g0(0x160), 0x1802 | 0x0004, 4); // compare GT0, OSM, OCM=011 (set)
        g.mmio_write(g0(0x164), 150, 4);
        g.mmio_write(g0(0x168), 0x2000, 4); // GTC13: capture, no edge (inactive), OCM=100
        g.advance(16 * 60, &[]);
        let o = g.take_outputs();
        assert_eq!(o.len(), 2, "{o:?}");
        assert_eq!((o[0].cell, o[0].level, o[0].t), (12, true, 150 * 16));
        assert_eq!((o[1].cell, o[1].level), (13, true));
        // one-shot: disabled after the event
        assert_eq!(g.mmio_read(g0(0x160), 4) & 0x400, 0);
        // OIA clear on GTC12
        g.mmio_write(g0(0x160), 0x5000, 4);
        assert!(!g.take_outputs()[0].level);
        // GES compare on XR write: X in the past -> immediate event
        g.mmio_write(g0(0x170), 0x1812, 4);
        g.mmio_write(g0(0x174), 10, 4);
        assert!(g.take_outputs().iter().any(|o| o.cell == 14 && o.level));
    }

    /// A capture on the same GT at the very tick of a compare syncs the timer first; the compare must still fire (it
    /// was lost until GT0 wrapped, ≈ 3 s at 5.625 MHz, stalling an SC800S50 ignition coil cell, G0.GTC14).
    #[test]
    fn capture_at_the_compare_tick_does_not_swallow_the_compare() {
        let mut g = Gpta::default();
        g.mmio_write(g0(0xD8), 0xC846, 4);
        g.mmio_write(g0(0xE0), 0x4E, 4);
        g.mmio_write(g0(0x400), 0x70F, 4);
        g.mmio_write(g0(0x38), 2, 4);
        for i in 0..54 {
            g.mmio_write(g0(0x3C), [0, 0x80C0_C3C3, 0xC5C5_0000][(i == 46) as usize + 2 * (i == 47) as usize], 4);
        }
        g.mmio_write(g0(0x38), 1, 4);
        g.mmio_write(g0(0x1F8), 0x28, 4); // GTC31: capture GT0 on falling edges of IN24
        g.mmio_write(g0(0x160), 0x1806, 4); // GTC12: compare GT0, one-shot, set output
        g.mmio_write(g0(0x164), 150, 4);
        g.advance(3200, &[InputEdge { t: 0, kernel: 0, line: 24, level: true }, InputEdge { t: 2400, kernel: 0, line: 24, level: false }]);
        assert_eq!(g.mmio_read(g0(0x1FC), 4), 150, "captured at the compare tick");
        let o = g.take_outputs();
        assert!(o.iter().any(|o| o.cell == 12 && o.level && o.t == 2400), "{o:?}");
    }

    #[test]
    fn ltc_pwm_reset_timer_period_and_duty() {
        // G1 quad: LTC24 reset timer (CLK4 = 5.625 MHz), LTC25 period compare XR = P-2,
        // LTC26 set at duty, LTC27 output: pattern from the SIMOS boot (0x590B/0x2031/0x3011/0x3021).
        let mut g = Gpta::default();
        let b = GPTA1_BASE;
        g.mmio_write(b + 0xD8, 0x1C846, 4);
        g.mmio_write(g0(0x400), 0x70F, 4);
        // LIMCR: LTC24 <- CLK4 (LIMCRL3 byte0 = 0xB4) ; position of LIMCRL3 = word 30 + 2*(7-3) + 1 = 39
        g.mmio_write(b + 0x38, 2, 4);
        for i in 0..54 {
            g.mmio_write(b + 0x3C, if i == 39 { 0xB4 } else { 0 }, 4);
        }
        g.mmio_write(b + 0x38, 1, 4);
        assert_eq!(g.ltc_input(1, 24), Line::Clk(4));
        let p = 5625u32;
        g.mmio_write(b + 0x200 + 24 * 8, 0x110B, 4); // reset timer, ILM, OCM=010 clear, REN
        g.mmio_write(b + 0x200 + 25 * 8, 0x2031, 4);
        g.mmio_write(b + 0x204 + 25 * 8, p - 2, 4);
        g.mmio_write(b + 0x200 + 26 * 8, 0x3831, 4); // compare (both select levels), set
        g.mmio_write(b + 0x204 + 26 * 8, 1000, 4);
        g.mmio_write(b + 0x200 + 27 * 8, 0x2031, 4); // link follower
        g.mmio_write(b + 0x204 + 27 * 8, 0xFFFE, 4);
        g.advance(16 * p as u64 * 5, &[]);
        let outs: Vec<_> = g.take_outputs().into_iter().filter(|o| o.cell == 27).collect();
        let rises: Vec<u64> = outs.iter().filter(|o| o.level).map(|o| o.t).collect();
        let falls: Vec<u64> = outs.iter().filter(|o| !o.level).map(|o| o.t).collect();
        assert!(rises.len() >= 4 && falls.len() >= 4, "{outs:?}");
        assert_eq!(rises[2] - rises[1], 16 * p as u64, "period");
        let hi = falls.iter().find(|&&f| f > rises[1]).unwrap() - rises[1];
        assert_eq!(hi, 16 * (p as u64 - 1001), "high time = period - duty edge");
        assert!(g.take_requests().iter().all(|&a| a == src_addr(1, 28)));
    }

    /// Duty update of the SC800S50 BSW PWM driver (0x800A88A4..0x800A88E8): the new edge goes to the compare
    /// cell of the inactive select level (SLO = 0 -> k+3 with SOH), then CUD is set by a read-modify-write of
    /// the timer CTR *without* CUDCLR (bit 9 set, bit 7 clear). The select line must toggle at the next reset
    /// overflow (p.24-181) so the new duty takes effect; before the fix such writes were ignored and every
    /// PWM channel kept its boot duty (A/D-monitoring test voltage stuck, doc §12.3).
    #[test]
    fn ltc_pwm_duty_update_by_coherent_update_without_cudclr() {
        let mut g = Gpta::default();
        let b = GPTA1_BASE;
        g.mmio_write(b + 0xD8, 0x1C846, 4);
        g.mmio_write(g0(0x400), 0x70F, 4);
        g.mmio_write(b + 0x38, 2, 4);
        for i in 0..54 {
            g.mmio_write(b + 0x3C, if i == 39 { 0xB4 } else { 0 }, 4); // LTC24 <- CLK4 (16 fGPTA ticks)
        }
        g.mmio_write(b + 0x38, 1, 4);
        let (p, d0, d1) = (0x1689u32, 1000u32, 2000u32);
        let ctr = |n: u32| b + 0x200 + n * 8;
        // SC800S50 LTCA2 LTC56 quad: timer 0x5903, period 0x2031 (XR = P - 2), SOL cell 0x3011, SOH cell 0x3021
        g.mmio_write(ctr(25) + 4, p - 2, 4);
        g.mmio_write(ctr(26) + 4, d0, 4);
        g.mmio_write(ctr(26), 0x3011, 4);
        g.mmio_write(ctr(27), 0x3021, 4);
        g.mmio_write(ctr(25), 0x2031, 4);
        g.mmio_write(ctr(24) + 4, 0, 4);
        g.mmio_write(ctr(24), 0x5903, 4);
        let high = |g: &mut Gpta, periods: u64| {
            g.advance(16 * p as u64 * periods, &[]);
            let o: Vec<_> = g.take_outputs().into_iter().filter(|o| o.cell == 27).collect();
            let r = o.iter().rev().find(|e| e.level).map(|e| e.t).unwrap();
            let f = o.iter().rev().find(|e| !e.level && e.t < r).map(|e| e.t).unwrap();
            let r0 = o.iter().rev().find(|e| e.level && e.t < f).map(|e| e.t).unwrap();
            (f - r0, r - r0)
        };
        assert_eq!(high(&mut g, 4), (16 * (d0 as u64 + 1), 16 * p as u64), "boot duty on the SOL cell");
        // firmware update: SLO = 0 -> write the SOH cell, then CTR |= CUD (read-modify-write, CUDCLR = 0)
        assert_eq!(g.mmio_read(ctr(24), 4) & 0x40, 0);
        g.mmio_write(ctr(27) + 4, d1, 4);
        let v = g.mmio_read(ctr(24), 4);
        g.mmio_write(ctr(24), v | 0x200, 4);
        assert_eq!(g.mmio_read(ctr(24), 4) & 0x200, 0x200, "CUD set by writing 1");
        assert_eq!(high(&mut g, 4), (16 * (d1 as u64 + 1), 16 * p as u64), "new duty after the coherent update");
        let v = g.mmio_read(ctr(24), 4);
        assert_eq!((v & 0x40, v & 0x200), (0x40, 0), "select line toggled, CUD cleared by hardware");
        // CUDCLR = 1 with CUD = 0 clears a pending request; a plain CUD = 0 write keeps it
        g.mmio_write(ctr(24), v | 0x200, 4);
        g.mmio_write(ctr(24), v & !0x200, 4);
        assert_eq!(g.mmio_read(ctr(24), 4) & 0x200, 0x200);
        g.mmio_write(ctr(24), (v & !0x200) | 0x80, 4);
        assert_eq!(g.mmio_read(ctr(24), 4) & 0x200, 0);
    }

    /// SIMOS18.1 KL50 start-request counter: G1.LTC60 free-running with ILM = 1 on IN26 counts fGPTA
    /// while the pin is high; its overflow toggles OUT27, which the pin loop feeds back to IN27 where
    /// G1.LTC61 counts both edges (configuration traced from SC800S50: CTR 0x902 / 0x032, LIMCRH7).
    #[test]
    fn ltc_level_gated_timer_and_pin_loop_count_high_time() {
        let mut g = Gpta::default();
        let b = GPTA1_BASE;
        g.mmio_write(g0(0x400), 0x70F, 4);
        // LIMCRH7 (MRA word 30): LTC60 <- IN26 (0x82), LTC61 <- IN27 (0x83)
        g.mmio_write(b + 0x38, 2, 4);
        for i in 0..54 {
            g.mmio_write(b + 0x3C, if i == 30 { 0x8382 } else { 0 }, 4);
        }
        g.mmio_write(b + 0x38, 1, 4);
        assert_eq!((g.ltc_input(1, 60), g.ltc_input(1, 61)), (Line::In(26), Line::In(27)));
        g.mmio_write(b + 0x200 + 60 * 8, 0x902, 4);
        g.mmio_write(b + 0x200 + 61 * 8, 0x032, 4);
        g.set_pin_loops(&[(1, CellKind::Ltc, 60, 27)]);
        let edge = |t, level| InputEdge { t, kernel: 1, line: 26, level };
        let count = |g: &mut Gpta| g.mmio_read(b + 0x204 + 61 * 8, 4) << 16 | g.mmio_read(b + 0x204 + 60 * 8, 4);
        g.advance(1000, &[]);
        assert_eq!(count(&mut g), 0, "released wire: no counting");
        g.advance(250_000, &[edge(1000, true), edge(201_000, false)]);
        assert_eq!(count(&mut g), 200_000, "counts fGPTA ticks while high, high word via OUT27 -> IN27");
        g.advance(500_000, &[edge(400_000, true), edge(410_000, false)]);
        assert_eq!(count(&mut g), 210_000);
    }

    #[test]
    fn tc1784_ltca2_on_gpta0_clock_bus_and_late_timer() {
        let mut g = Gpta::tc1784(40_000_000);
        g.mmio_write(g0(0xD8), 0x5, 4); // CKBCTR (DQ200 value): CLK2 = fGPTA/32
        g.mmio_write(g0(0x400), 0x503, 4); // GPTA0 + LTCA2 clocks on
        g.advance(1_000_000, &[]); // the timer is created long after time 0
        let b = LTCA2_BASE;
        g.mmio_write(b + 0x38, 2, 4);
        for i in 0..24 {
            g.mmio_write(b + 0x3C, if i == 18 { 0x00B2_0000 } else { 0 }, 4); // LIMCRH2: LTC22 <- CLK2
        }
        g.mmio_write(b + 0x38, 1, 4);
        assert_eq!(g.ltc_input(2, 22), Line::Clk(2));
        g.mmio_write(b + 0x200 + 22 * 8, 0x0B, 4); // reset timer, REN
        g.mmio_write(b + 0x200 + 23 * 8, 0x31, 4); // period compare on both select levels
        g.mmio_write(b + 0x204 + 23 * 8, 99, 4);
        // timer events at CLK2 pulses 100, 201, 302 (period XR + 2)
        g.advance(32 * 101 * 3, &[]);
        assert_eq!(g.take_requests(), vec![src_addr(2, 5); 3]);
    }

    #[test]
    fn srs_set_clear_and_srnr_redirect() {
        let mut g = Gpta::default();
        g.mmio_write(g0(0x1F8), 0x0008, 4); // GTC31 REN
        g.mmio_write(g0(0x1C), 1 << 31, 4); // SRSS1: software set -> request
        assert_eq!(g.take_requests(), vec![src_addr(0, 21)]);
        g.mmio_write(g0(0x30), 1 << 15, 4); // SRNR: redirect GTC31 to LTC group 37
        g.mmio_write(g0(0x1C), 1 << 31, 4);
        assert_eq!(g.take_requests(), vec![src_addr(0, 37)]);
        g.mmio_write(g0(0x18), 0xFFFF_FFFF, 4);
        assert_eq!(g.mmio_read(g0(0x18), 4), 0);
        // 16-bit write only touches its lanes
        g.mmio_write(g0(0x14), 0x1000, 2);
        assert_eq!(g.mmio_read(g0(0x10), 4), 0x1000);
        g.mmio_write(g0(0x12), 0x0000, 2);
        assert_eq!(g.mmio_read(g0(0x10), 4), 0x1000);
    }
}

/// The model in lockstep with the reference ([`crate::gpta_ref`]: this model before the routing bitmasks, the
/// due-mask early-outs and the lazily emitted PLL pulses). Every request, output transition (with its GT0/GT1
/// stamps), register read and next event must agree, and so must the whole register file read on clones (which
/// leaves the lazy state of the pair undisturbed).
#[cfg(test)]
mod diff_tests {
    use super::*;
    use crate::gpta_ref::Gpta as Ref;

    struct Pair {
        a: Gpta,
        b: Ref,
    }

    impl Pair {
        /// 0 = TC1791, 1 = TC1784 (GPTA0 + LTCA2), 2 = TC1766 (52 MRA words).
        fn new(dev: u64) -> Self {
            match dev {
                1 => Self { a: Gpta::tc1784(40_000_000), b: Ref::tc1784(40_000_000) },
                2 => Self { a: Gpta::tc1766(F_GPTA_DEFAULT), b: Ref::tc1766(F_GPTA_DEFAULT) },
                _ => Self { a: Gpta::default(), b: Ref::default() },
            }
        }

        fn w(&mut self, addr: u32, v: u32) {
            self.a.write_lanes(addr, v, !0);
            self.b.write_lanes(addr, v, !0);
        }

        fn r(&mut self, addr: u32, ctx: &str) -> u32 {
            let (x, y) = (self.a.read_word(addr), self.b.read_word(addr));
            assert_eq!(x, y, "read {addr:#x}, {ctx}");
            x.unwrap_or(0)
        }

        fn adv(&mut self, ticks: u64, edges: &[InputEdge], ctx: &str) {
            self.a.advance(ticks, edges);
            self.b.advance(ticks, edges);
            assert_eq!(self.a.take_requests(), self.b.take_requests(), "requests, {ctx}");
            assert_eq!(self.a.take_outputs(), self.b.take_outputs(), "outputs, {ctx}");
        }

        fn loops(&mut self, l: &[(u8, CellKind, u8, u8)]) {
            self.a.set_pin_loops(l);
            self.b.set_pin_loops(l);
        }

        /// Next event tick, on the pair itself (`own`) or on clones.
        fn next(&mut self, cap: u64, own: bool, ctx: &str) {
            if own {
                assert_eq!(self.a.next_event_tick(cap), self.b.next_event_tick(cap), "next event, {ctx}");
            } else {
                assert_eq!(self.a.clone().next_event_tick(cap), self.b.clone().next_event_tick(cap), "next event, {ctx}");
            }
        }

        /// Every register of the three kernels, read on clones.
        fn regs(&self, ctx: &str) {
            let (mut a, mut b) = (self.a.clone(), self.b.clone());
            for base in BASES {
                for addr in (base..base + 0x800).step_by(4).filter(|&a| Gpta::contains(a)) {
                    assert_eq!(a.read_word(addr), b.read_word(addr), "register {addr:#x}, {ctx}");
                }
            }
        }
    }

    /// Seed and case count of a random differential test; `SIMEMU_GPTA_DIFF_SEED` / `SIMEMU_GPTA_DIFF_CASES`
    /// override them for longer searches.
    fn knobs(seed: u64, cases: u64) -> (u64, u64) {
        let env = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<u64>().ok());
        (env("SIMEMU_GPTA_DIFF_SEED").unwrap_or(seed), env("SIMEMU_GPTA_DIFF_CASES").unwrap_or(cases))
    }

    fn rng(seed: u64) -> impl FnMut(u64) -> u64 {
        let mut s = seed;
        move |n| {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s % n.max(1)
        }
    }

    /// MRA word of the GIMCR byte of GTC `c` (`n` words, the 8 GIMCR words last) and of the LIMCR byte of LTC
    /// `c` (`lim` = first LIMCR word).
    fn gim_word(n: usize, c: u32) -> usize {
        n - 8 + 2 * (3 - c as usize / 8) + (c % 8 < 4) as usize
    }
    fn lim_word(lim: usize, c: u32) -> usize {
        lim + 2 * (7 - c as usize / 8) + (c % 8 < 4) as usize
    }

    /// Random cell structures on all three devices: LTC timer chains (clock bus incl. the PLL and DCM lines,
    /// edge-clocked, level-gated) with compare cells and output links, GTC compares/captures on GT0/GT1 (GT1 on
    /// the local PLL, GPTA1.GT1 on the GPTA0 PLL), FPC outputs routed to captures, DCM0 driving the PLL from a
    /// periodic input with and without AEN, a pin loop; random register writes, reads and advances.
    #[test]
    fn diff_random_cells_match_reference() {
        let (seed, cases) = knobs(0xD1FF_5EED_0BAD_CAFE, 240);
        let mut rnd = rng(seed);
        for case in 0..cases {
            let dev = case % 3;
            let mut p = Pair::new(dev);
            let kernels = if dev == 1 { [0usize, 2] } else { [0, 1] };
            let pll = rnd(3) != 0 && dev != 1;
            let ctx0 = format!("case {case} dev {dev} pll {pll}");
            // No CKBCTR here moves a GT from a PLL clock line to a prescaler line: both models then count prescaler
            // pulses back to the GT's last sync (the GT algorithm p.24-130 counts only events of the selected clock)
            // and `advance` loops forever on the overdue event (step 5 skips it, `cmp_t == now`). A defect of both
            // models, not reached by the SC800S50 / DQ250 traces; not fixed here.
            let mut ops: Vec<(u32, u32)> = vec![(GPTA0_BASE + 0xD8, [0x31_C846u32, 0xC846, 0x5, 0x31_C842, 0x2_C846][rnd(5) as usize])];
            if dev != 1 {
                ops.push((GPTA1_BASE + 0xD8, [0x1C846u32, 0xC846, 0x31_C846][rnd(3) as usize]));
            }
            let mut lines = vec![10u8, 24];
            let mut ltc_regs = Vec::new();
            let mut gtc_regs = Vec::new();
            let mut gt1_cmp = Vec::new();
            for &k in &kernels {
                let base = BASES[k];
                let n = match (dev, k) {
                    (1, 2) => 24,
                    (2, 0) => 52,
                    _ => 54,
                };
                let mut mra = vec![0u32; n];
                let lim = if k == 2 { n - 16 } else { n - 24 };
                let groups = if k == 2 { 4 } else { 8 };
                for _ in 0..1 + rnd(3) {
                    let c0 = 8 * rnd(groups) as u32 + 4 * rnd(2) as u32; // timer cell, then 3 compare cells
                    let clocked = rnd(3) != 0;
                    let byte = if clocked { 0xB0 | rnd(8) as u32 } else { 0x80 | rnd(8) as u32 };
                    if !clocked {
                        lines.push((8 * (c0 / 8 % 4) + (byte & 7)) as u8);
                    }
                    mra[lim_word(lim, c0)] |= byte << (8 * (c0 % 4));
                    let tctr = if clocked { [0x0Bu32, 0x110B, 0x5903, 0x0A, 0x903, 0x02][rnd(6) as usize] } else { [0x902u32, 0x90A, 0x0B, 0x13, 0x0A][rnd(5) as usize] };
                    ops.push((base + 0x204 + 8 * c0, rnd(0x300) as u32));
                    for c in c0 + 1..c0 + 4 {
                        ops.push((base + 0x204 + 8 * c, [rnd(0x300), 0xFFFE, rnd(0x40)][rnd(3) as usize] as u32));
                        ops.push((base + 0x200 + 8 * c, [0x2031u32, 0x3831, 0x3011, 0x3021, 0x31, 0x1839, 0x2039, 0x3839][rnd(8) as usize]));
                        ltc_regs.push(base + 0x200 + 8 * c);
                    }
                    ops.push((base + 0x200 + 8 * c0, tctr));
                    ltc_regs.push(base + 0x200 + 8 * c0);
                }
                if k != 2 {
                    for _ in 0..rnd(6) {
                        let c = 8 + rnd(24) as u32;
                        match rnd(4) {
                            0 => {
                                ops.push((base + 0x104 + 8 * c, rnd(0x400) as u32));
                                ops.push((base + 0x100 + 8 * c, [0x1802u32, 0x1806, 0x1812, 0x180A, 0x180B, 0x0807, 0x1803][rnd(7) as usize]));
                            }
                            1 if pll => {
                                ops.push((base + 0x104 + 8 * c, 1 + rnd(0x200) as u32));
                                ops.push((base + 0x100 + 8 * c, [0x0803u32, 0x0807, 0x180B, 0x1813, 0x2907][rnd(5) as usize]));
                                gt1_cmp.push(base + 0x100 + 8 * c);
                            }
                            _ => {
                                // capture of GT0 or GT1: from a pin or from an FPC output (GIMG 4)
                                let fpc = rnd(2) == 0;
                                let l = if fpc { rnd(6) as u32 } else { rnd(8) as u32 };
                                mra[gim_word(n, c)] |= (if fpc { 0xC0 } else { 0x80 } | l) << (8 * (c % 4));
                                if !fpc {
                                    lines.push((8 * (c / 8) + l) as u8);
                                }
                                ops.push((base + 0x100 + 8 * c, [0x28u32, 0x18, 0x38, 0x2A, 0x29, 0x39, 0x0C31][rnd(7) as usize]));
                            }
                        }
                        gtc_regs.push(base + 0x100 + 8 * c);
                    }
                    ops.push((base + 0xE0, (0x4E | 0x80) & [0xFF, 0x7F][rnd(2) as usize]));
                    // GT1: local PLL (CLK1), GPTA1 also the GPTA0 PLL (CLK3 with CKBCTR bit 16), else a prescaler line
                    let gt1 = if pll { [0x9Eu32, 0x1E][rnd(2) as usize] | if k == 1 && rnd(2) == 0 { 0x20 } else { 0 } } else { [0x2E, 0x6E][rnd(2) as usize] };
                    ops.push((base + 0xF0, gt1));
                    for f in 0..6u32 {
                        let pll_in = pll && k == 0 && f == 0;
                        if rnd(3) == 0 || pll_in {
                            let ips = if pll_in { 2 } else { rnd(3) as u32 };
                            let ctr = rnd(12) as u32 | (rnd(6) as u32) << 16 | ips << 19 | ([0, 2, 3][rnd(3) as usize]) << 22;
                            ops.push((base + 0x48 + 8 * f, ctr));
                        }
                    }
                    for d in 0..4u32 {
                        ops.push((base + 0x80 + 16 * d, 0x183 | (rnd(4) as u32) << 4));
                    }
                }
                ops.push((base + 0x38, 2));
                ops.extend(mra.iter().map(|&w| (base + 0x3C, w)));
                ops.push((base + 0x38, 1));
            }
            let period = 300 + rnd(20_000);
            if pll {
                let mti = 1 + rnd(300) as u32;
                ops.push((GPTA0_BASE + 0x80, 0x129)); // DCM0: RCA, FZE, FCK (PLL event on falling edges), FRE
                ops.push((GPTA0_BASE + 0xC4, mti));
                ops.push((GPTA0_BASE + 0xCC, 0x1_0000 - mti));
                ops.push((GPTA0_BASE + 0xD0, (period as u32).saturating_sub(mti).max(1)));
                ops.push((GPTA0_BASE + 0xC0, [0x14u32, 0x10, 0x04, 0x00][rnd(4) as usize]));
            }
            ops.push((GPTA0_BASE + 0x400, if dev == 1 { 0x503 } else { 0x70F }));
            for (a, v) in ops {
                p.w(a, v);
            }
            if rnd(4) == 0 {
                p.loops(&[(kernels[1] as u8, CellKind::Ltc, 1 + rnd(30) as u8, 27)]);
                lines.push(27);
            }
            let mut levels = [0u64; 3];
            let mut tooth = p.a.now();
            for step in 0..260 {
                let ctx = format!("{ctx0} step {step}");
                match rnd(30) {
                    0 if !ltc_regs.is_empty() => {
                        let a = ltc_regs[rnd(ltc_regs.len() as u64) as usize] + 4;
                        p.w(a, rnd(0x400) as u32);
                    }
                    1 if !ltc_regs.is_empty() => {
                        // compare-cell rewrite or the firmware's coherent duty update (CUD by read-modify-write)
                        let a = ltc_regs[rnd(ltc_regs.len() as u64) as usize];
                        let v = (p.r(a, &ctx) | (0x200 * rnd(2) as u32)) & !0x8000;
                        p.w(a, v);
                    }
                    2 if !gtc_regs.is_empty() => {
                        let a = gtc_regs[rnd(gtc_regs.len() as u64) as usize] + 4;
                        p.w(a, rnd(0x800) as u32);
                    }
                    3 if !gt1_cmp.is_empty() => {
                        // re-arm a GT1 (angle) compare a little ahead of the timer (EOA = 0 enables it)
                        let a = gt1_cmp[rnd(gt1_cmp.len() as u64) as usize];
                        let gt = p.r(a & !0x7FF | 0xF8, &ctx);
                        p.w(a + 4, (gt + rnd(40) as u32) & 0xFF_FFFF);
                        let ctr = p.r(a, &ctx) & 0x3BFF & !0x100;
                        p.w(a, ctr);
                    }
                    4 if pll => {
                        // period estimate (REV) as a tooth ISR writes it, sometimes MTI or the mode too
                        p.w(GPTA0_BASE + 0xD0, (period as u32 + rnd(400) as u32).saturating_sub(200).max(1));
                        match rnd(6) {
                            0 => p.w(GPTA0_BASE + 0xC4, 1 + rnd(300) as u32),
                            1 => p.w(GPTA0_BASE + 0xC0, [0x14u32, 0x10, 0x04, 0x00][rnd(4) as usize]),
                            _ => {}
                        }
                    }
                    5 => {
                        let a = BASES[kernels[0]] + [0xE8u32, 0xF8][rnd(2) as usize];
                        p.w(a, rnd(0x1000) as u32);
                    }
                    6 if step % 5 == 0 => {
                        let v = [0x31_C846u32, 0xC846, 0x5, 0x31_C840, 0x2_C846][rnd(5) as usize];
                        p.w(GPTA0_BASE + 0xD8, v);
                    }
                    7 if step % 9 == 0 => {
                        let v = [0x70Fu32, 0x60F, 0x503, 0x103, 0x70F, 0x70F][rnd(6) as usize];
                        p.w(GPTA0_BASE + 0x400, v);
                    }
                    8 => {
                        for off in [0x10u32, 0x18, 0x20, 0x28] {
                            p.w(BASES[kernels[1]] + off, !0);
                        }
                    }
                    9 => p.regs(&ctx),
                    10 => {
                        for &a in ltc_regs.iter().chain(&gtc_regs) {
                            p.r(a, &ctx);
                            p.r(a + 4, &ctx);
                        }
                        for off in [0xC0u32, 0xC8, 0xD4, 0xE8, 0xF8] {
                            p.r(GPTA0_BASE + off, &ctx);
                        }
                    }
                    r => {
                        let ticks = match r % 4 {
                            0 => rnd(2),
                            1 => 1 + rnd(40),
                            2 => 1 + rnd(2000),
                            _ => 1 + rnd(60_000),
                        };
                        let (t0, end) = (p.a.now(), p.a.now() + ticks);
                        let mut edges = Vec::new();
                        // periodic PLL input (IN24 -> FPC0 -> DCM0) with jitter and missing teeth
                        while pll && tooth + period <= end {
                            tooth += period + rnd(period / 8 + 1) - period / 16;
                            if tooth <= end && rnd(12) != 0 {
                                edges.push(InputEdge { t: tooth.max(t0), kernel: 0, line: 24, level: false });
                                edges.push(InputEdge { t: (tooth + period / 3).min(end).max(t0), kernel: 0, line: 24, level: true });
                            }
                        }
                        if rnd(3) == 0 {
                            let mut t = t0;
                            for _ in 0..1 + rnd(4) {
                                t += rnd(ticks / 2 + 1);
                                if t > end {
                                    break;
                                }
                                let k = kernels[rnd(2) as usize];
                                let line = lines[rnd(lines.len() as u64) as usize];
                                levels[k] ^= 1 << line;
                                edges.push(InputEdge { t, kernel: k as u8, line, level: levels[k] >> line & 1 != 0 });
                            }
                        }
                        edges.sort_by_key(|e| e.t);
                        p.adv(ticks, &edges, &ctx);
                    }
                }
                let cap = p.a.now() + rnd(4000);
                p.next(cap, rnd(8) == 0, &ctx);
            }
            p.regs(&format!("{ctx0} end"));
        }
    }

    /// SIMOS18-like crank front end from cranking to 8000 rpm: 60-2 wheel on IN24 -> FPC0 -> DCM0 -> PLL (AEN,
    /// 256 micro ticks per tooth, 3 x 256 over the gap, REV from the measured period as the tooth ISR writes it),
    /// GT1 on the PLL and GPTA1.GT1 on the GPTA0 PLL, one-shot angle compares on GT1 re-armed ahead of the timer,
    /// captures of GT0/GT1 on the crank pin, the filter output and the cam pin, an LTC chain clocked by the PLL
    /// line (no lazy pulses) and one on fGPTA/16, firmware-like register traffic between the advances.
    #[test]
    fn diff_crank_wheel_pll_matches_reference() {
        let (seed, cases) = knobs(0xC4A2_0060_0002_8000, 16);
        let mut rnd = rng(seed);
        for case in 0..cases {
            let mut p = Pair::new(0);
            let n = 54;
            let mut mra = vec![0u32; n];
            mra[gim_word(n, 31)] |= 0x80 << 24; // GTC31 <- IN24 (crank pin)
            mra[gim_word(n, 30)] |= 0xC0 << 16; // GTC30 <- FPC0 (filtered crank)
            mra[gim_word(n, 29)] |= 0x82 << 8; // GTC29 <- IN26 (cam pin)
            mra[lim_word(n - 24, 16)] |= 0xB1; // LTC16 <- CLK1 (local PLL)
            mra[lim_word(n - 24, 40)] |= 0xB4; // LTC40 <- CLK4 (fGPTA/16)
            let pll_ltc = case % 2 == 0;
            let mut ops = vec![
                (GPTA0_BASE + 0xD8, 0x1_C846u32),
                (GPTA1_BASE + 0xD8, 0x1_C846),
                (GPTA0_BASE + 0xE0, 0xCE), // GT0 on CLK4, overflow request
                (GPTA0_BASE + 0xF0, 0x1E), // GT1 on the PLL
                (GPTA1_BASE + 0xF0, 0x3E), // GPTA1.GT1 on the GPTA0 PLL (CLK3)
                (GPTA0_BASE + 0x48, 0x0093_0000 | [0u32, 2, 4][rnd(3) as usize]), // FPC0: IN24, MOD 3, CLK2
                (GPTA0_BASE + 0x80, 0x129),
                (GPTA0_BASE + 0xC4, 0x100),
                (GPTA0_BASE + 0xCC, 0xFF00),
                (GPTA0_BASE + 0xD0, 15_000 - 256),
                (GPTA0_BASE + 0xC0, 0x14),
                (GPTA0_BASE + 0x1F8, 0x29), // GTC31: capture GT1 on falling edges, request
                (GPTA0_BASE + 0x1F0, 0x0C31), // GTC30: capture GT1 on both edges, toggle output
                (GPTA0_BASE + 0x1E8, 0x38), // GTC29: capture GT0 on both edges, request
            ];
            for c in 8..16u32 {
                ops.push((GPTA0_BASE + 0x104 + 8 * c, 1 + rnd(2000) as u32));
                ops.push((GPTA0_BASE + 0x100 + 8 * c, [0x0807u32, 0x080F, 0x1807, 0x2003][rnd(4) as usize]));
            }
            if pll_ltc {
                ops.extend([(GPTA0_BASE + 0x200 + 8 * 16, 0x0B), (GPTA0_BASE + 0x204 + 8 * 17, 700), (GPTA0_BASE + 0x200 + 8 * 17, 0x2031)]);
                ops.extend([(GPTA0_BASE + 0x204 + 8 * 18, 200), (GPTA0_BASE + 0x200 + 8 * 18, 0x3831), (GPTA0_BASE + 0x200 + 8 * 19, 0x2031)]);
            }
            ops.extend([(GPTA0_BASE + 0x200 + 8 * 40, 0x110B), (GPTA0_BASE + 0x204 + 8 * 41, 5623), (GPTA0_BASE + 0x200 + 8 * 41, 0x2031)]);
            ops.extend([(GPTA0_BASE + 0x204 + 8 * 42, 1000), (GPTA0_BASE + 0x200 + 8 * 42, 0x3831), (GPTA0_BASE + 0x200 + 8 * 43, 0x2031)]);
            ops.push((GPTA0_BASE + 0x38, 2));
            ops.extend(mra.iter().map(|&w| (GPTA0_BASE + 0x3C, w)));
            ops.push((GPTA0_BASE + 0x38, 1));
            ops.push((GPTA0_BASE + 0x400, 0x70F));
            for (a, v) in ops {
                p.w(a, v);
            }
            let (rpm0, rpm1) = (150.0 + rnd(8000) as f64, 150.0 + rnd(8000) as f64);
            let teeth = 240u64;
            let mut t = p.a.now() + 1000;
            let mut edges = std::collections::VecDeque::new();
            for i in 0..teeth {
                let rpm = rpm0 + (rpm1 - rpm0) * i as f64 / teeth as f64;
                let period = (90e6 / rpm) as u64 + rnd(64); // 60 teeth per revolution
                let tooth = i % 60;
                if tooth < 58 {
                    edges.push_back(InputEdge { t, kernel: 0, line: 24, level: false });
                    edges.push_back(InputEdge { t: t + period / 2, kernel: 0, line: 24, level: true });
                }
                if tooth == 20 {
                    edges.push_back(InputEdge { t: t + period / 3, kernel: 0, line: 26, level: (i / 60) % 2 == 0 });
                }
                t += period;
            }
            let mut sorted: Vec<_> = edges.into_iter().collect();
            sorted.sort_by_key(|e| e.t);
            let mut edges: std::collections::VecDeque<_> = sorted.into();
            let mut step = 0u64;
            let mut last_period = 15_000u64;
            while !edges.is_empty() {
                step += 1;
                let ctx = format!("crank case {case} rpm {rpm0:.0}->{rpm1:.0} step {step} now {}", p.a.now());
                let span = 1 + rnd(last_period / 3 + 1);
                let end = p.a.now() + span;
                let mut batch = Vec::new();
                while edges.front().is_some_and(|e| e.t <= end) {
                    batch.push(edges.pop_front().unwrap());
                }
                let falling = batch.iter().any(|e| e.line == 24 && !e.level);
                p.adv(span, &batch, &ctx);
                if falling {
                    // tooth ISR: period from the DCM capture, REV and MTI (3 x over the gap), angle reads
                    let cav = p.r(GPTA0_BASE + 0x88, &ctx) as u64;
                    if cav > 512 {
                        last_period = cav;
                    }
                    let gap = cav > 2 * last_period.min(cav) && cav > 2 * 11_000;
                    p.w(GPTA0_BASE + 0xC4, if gap { 0x300 } else { 0x100 });
                    p.w(GPTA0_BASE + 0xD0, (last_period as u32).saturating_sub(256).max(1));
                    p.r(GPTA0_BASE + 0xF8, &ctx);
                    p.r(GPTA0_BASE + 0x1FC, &ctx);
                    p.r(GPTA0_BASE + 0xC8, &ctx);
                }
                if rnd(4) == 0 {
                    // re-arm an angle compare a little ahead of GT1 (EOA = 0)
                    let c = 8 + rnd(8) as u32;
                    let gt1 = p.r(GPTA0_BASE + 0xF8, &ctx);
                    p.w(GPTA0_BASE + 0x104 + 8 * c, (gt1 + 1 + rnd(600) as u32) & 0xFF_FFFF);
                    let ctr = p.r(GPTA0_BASE + 0x100 + 8 * c, &ctx) & 0x3BFF & !0x100;
                    p.w(GPTA0_BASE + 0x100 + 8 * c, ctr);
                }
                if rnd(20) == 0 {
                    p.regs(&ctx);
                }
                let cap = p.a.now() + rnd(20_000);
                p.next(cap, rnd(16) == 0, &ctx);
            }
            p.regs(&format!("crank case {case} end"));
        }
    }

    /// GPTA0 PLL in free run (no AEN, MTI 0x100 reloaded, m = 256, REV 768: a pulse every 4 ticks) clocking GT1
    /// and GPTA1.GT1 (CLK3), DCM0 <- FPC0 <- IN24 as its input, GTC31 capturing GT1 on rising IN24 edges.
    fn free_pll(p: &mut Pair, ctr: u32) {
        let mut mra = vec![0u32; 54];
        mra[gim_word(54, 31)] = 0x80 << 24;
        let mut ops = vec![(GPTA0_BASE + 0xD8, 0xC846u32), (GPTA1_BASE + 0xD8, 0x1_C846), (GPTA0_BASE + 0x400, 0x70F)];
        ops.extend([(0xF0, 0x1E), (0x48, 0x0013_0000), (0x80, 0x129), (0xC4, 0x100), (0xCC, 0xFF00), (0xD0, 768), (0xC0, ctr), (0x1F8, 0x19)].map(|(o, v)| (GPTA0_BASE + o, v)));
        ops.push((GPTA1_BASE + 0xF0, 0x3E));
        ops.push((GPTA0_BASE + 0x38, 2));
        ops.extend(mra.iter().map(|&w| (GPTA0_BASE + 0x3C, w)));
        ops.push((GPTA0_BASE + 0x38, 1));
        for (a, v) in ops {
            p.w(a, v);
        }
    }

    /// Tick of the next GPTA0 PLL pulse (GT1 steps), found on a clone of the reference.
    fn next_pulse(p: &Pair) -> u64 {
        let mut g = p.b.clone();
        let gt = g.mmio_read(GPTA0_BASE + 0xF8, 4);
        while g.mmio_read(GPTA0_BASE + 0xF8, 4) == gt {
            g.advance(1, &[]);
        }
        g.now()
    }

    /// An input edge at the tick of a PLL micro tick is captured before the pulse (input edges are step 1, the
    /// PLL step 4 of `process_tick`), also when the pulse is emitted lazily. The studio DQ250 preparation diverged
    /// here (SC8O40 knock window GTC26/GTC27 on an FPC line: GT1 stamp and capture one ahead).
    #[test]
    fn diff_capture_at_a_pll_pulse_tick_precedes_the_pulse() {
        let mut p = Pair::new(0);
        free_pll(&mut p, 0);
        p.adv(1000, &[], "free run");
        let t = next_pulse(&p);
        p.adv(t - 1 - p.a.now(), &[], "to the tick before the pulse");
        let gt = p.r(GPTA0_BASE + 0xF8, "GT1 before");
        p.adv(1, &[InputEdge { t, kernel: 0, line: 24, level: true }], "edge at the pulse tick");
        assert_eq!(p.r(GPTA0_BASE + 0x1FC, "capture"), gt, "captured before the pulse");
        assert_eq!(p.r(GPTA0_BASE + 0xF8, "GT1 after"), gt + 1);
    }

    /// A PLL input event at a tick whose micro tick is done already (an edge at `now` after an advance that
    /// ended on a pulse) is taken at that tick: the PLL step runs again (AEN: PEN, pulse at full speed).
    #[test]
    fn diff_pll_event_at_a_tick_already_done() {
        let mut p = Pair::new(0);
        free_pll(&mut p, 0x04);
        let e = |t, level| InputEdge { t, kernel: 0, line: 24, level };
        p.adv(1000, &[e(100, true), e(200, false)], "the first event starts the PLL");
        let t = next_pulse(&p);
        p.adv(t - p.a.now(), &[e(t - 2, true)], "advance ending on a pulse");
        p.adv(1, &[e(t, false)], "falling edge at that tick");
        p.regs("after the event");
        p.adv(40, &[], "PEN pulses");
        p.regs("after the PEN pulses");
    }

    /// The PLL of a disabled kernel (EDCTR G0EN = 0) is not scheduled; the reference still ticks it whenever the
    /// PLL step runs for something else (here GPTA1 input edges) at a tick where it is due, and GPTA1.GT1 on the
    /// GPTA0 PLL counts those pulses. Enabling the kernel at a tick with a pulse due leaves that pulse to the next
    /// advance.
    #[test]
    fn diff_pll_of_a_disabled_kernel() {
        let mut p = Pair::new(0);
        free_pll(&mut p, 0);
        for (o, v) in [(0xC4u32, 0x100u32), (0xCC, 0xFF00), (0xD0, 1000), (0xC0, 0)] {
            p.w(GPTA1_BASE + o, v); // GPTA1 PLL in free run as well: its micro ticks run the PLL step
        }
        p.adv(1000, &[], "free run");
        p.w(GPTA0_BASE + 0x400, 0x60F); // GPTA0 clock off, GPTA1/LTCA2 on
        for i in 0..300u64 {
            let t = p.a.now() + 1 + i % 5;
            p.adv(t - p.a.now(), &[InputEdge { t, kernel: 1, line: 3, level: i % 2 == 0 }], &format!("edge {i}"));
        }
        p.regs("GPTA0 off");
        for i in 0..8u64 {
            p.w(GPTA0_BASE + 0x400, 0x70F);
            p.regs(&format!("enabled {i}"));
            p.adv(1 + i, &[], "enabled");
            p.w(GPTA0_BASE + 0x400, 0x60F);
            p.adv(3, &[], "disabled");
        }
        p.regs("end");
    }

    /// `next_event_tick` names every PLL micro tick, also the ones the model emits lazily (the TC1784 board ends
    /// its quantum there).
    #[test]
    fn diff_next_event_names_every_pll_micro_tick() {
        let mut p = Pair::new(0);
        free_pll(&mut p, 0);
        p.adv(1000, &[], "free run");
        let t = next_pulse(&p);
        assert_eq!(p.a.clone().next_event_tick(t + 100), Some(t));
        p.next(t + 100, true, "next pulse");
    }

    /// An LTC chain whose clock line becomes a prescaler line is overdue: its event is clamped to `now` until a
    /// read syncs the chain and moves it. The cached next event must not outlive that read.
    #[test]
    fn diff_next_event_after_an_overdue_chain_is_synced() {
        let mut p = Pair::new(0);
        let mut mra = vec![0u32; 54];
        mra[lim_word(30, 8)] = 0xB2; // LTC8 <- CLK2
        for (a, v) in [(0xD8u32, 0xC84F), (0x400, 0x70F), (0x204 + 8 * 9, 100), (0x200 + 8 * 9, 0x2031), (0x200 + 8 * 8, 0x0B)] {
            p.w(GPTA0_BASE + a, v);
        }
        p.w(GPTA0_BASE + 0x38, 2);
        for w in mra {
            p.w(GPTA0_BASE + 0x3C, w);
        }
        p.w(GPTA0_BASE + 0x38, 1);
        p.adv(10_000, &[], "CLK2 on DCM3: the chain stands");
        p.w(GPTA0_BASE + 0xD8, 0xC840); // CLK2 = fGPTA
        let now = p.a.now();
        p.next(now + 1000, true, "overdue chain");
        p.r(GPTA0_BASE + 0x204 + 8 * 8, "timer read syncs the chain");
        p.next(now + 1000, true, "after the sync");
        p.adv(500, &[], "running");
    }

    /// Replays a recorded board trace (`SIMEMU_GPTA_TRACE_FILE`; lines `D <dq250>`, `W <addr> <v> <mask>`,
    /// `R <addr>`, `A <ticks> [<t> <kernel> <line> <level>]*`, `L [<k> <g|l> <cell> <line>]*`, numbers hex)
    /// through the pair. Skipped without a trace.
    #[test]
    fn diff_replay_recorded_trace() {
        let Ok(path) = std::env::var("SIMEMU_GPTA_TRACE_FILE") else { return eprintln!("SKIP: no SIMEMU_GPTA_TRACE_FILE") };
        let probe_every: u64 = std::env::var("SIMEMU_GPTA_PROBE_EVERY").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
        let text = std::io::BufRead::lines(std::io::BufReader::new(std::fs::File::open(&path).unwrap()));
        let hex = |s: &str| u64::from_str_radix(s, 16).unwrap();
        let mut p = Pair::new(0);
        for (i, line) in text.enumerate() {
            let line = line.unwrap();
            let f: Vec<&str> = line.split(' ').collect();
            let ctx = format!("event {i}: {line}");
            match f[0] {
                "D" => p = Pair::new(if f[1] == "1" { 2 } else { 0 }),
                "W" => {
                    p.a.write_lanes(hex(f[1]) as u32, hex(f[2]) as u32, hex(f[3]) as u32);
                    p.b.write_lanes(hex(f[1]) as u32, hex(f[2]) as u32, hex(f[3]) as u32);
                }
                "R" => {
                    p.r(hex(f[1]) as u32, &ctx);
                }
                "A" => {
                    let edges: Vec<InputEdge> = f[2..]
                        .chunks(4)
                        .map(|e| InputEdge { t: hex(e[0]), kernel: e[1].parse().unwrap(), line: e[2].parse().unwrap(), level: e[3] == "1" })
                        .collect();
                    p.adv(hex(f[1]), &edges, &ctx);
                }
                "L" => {
                    let l: Vec<(u8, CellKind, u8, u8)> = f[1..]
                        .chunks(4)
                        .map(|e| (e[0].parse().unwrap(), if e[1] == "g" { CellKind::Gtc } else { CellKind::Ltc }, e[2].parse().unwrap(), e[3].parse().unwrap()))
                        .collect();
                    p.loops(&l);
                }
                _ => panic!("bad trace line {i}: {line}"),
            }
            if probe_every != 0 && (i as u64).is_multiple_of(probe_every) {
                p.regs(&ctx);
            }
        }
        p.regs("end of trace");
    }
}
