//! Engine I/O of the SIMOS18.1 board: the GPTA ([`crate::gpta`]) and PCP2 ([`crate::pcp`]) wired into the
//! TC1791 core, the crank/cam sensor inputs ([`crate::crank_wheel`]) and read-only output taps.
//!
//! Time: everything follows the CPU instruction clock of the board.
//! * fGPTA = fFPI / FDR divider: fFPI = fCPU / (CCUCON0.FPIDIV + 1) (as for the STM, `stm_icu`), FDR = GPTA0
//!   `0xF000180C` (AUDO fractional divider: DM=1 normal ÷(1024−STEP), DM=2 fractional ×STEP/1024, else off).
//!   SIMOS18.1: FDR 0x43FF -> fGPTA = fFPI = 90 MHz (doc SIMOS18_GPTA_IO_RE.md §2).
//! * fPCP = fCPU / (`CCUCON0[19:16]` + 1). HYP: the TC1791 data sheet (p.84/117/121) lists fPCP as its own
//!   clock (240 MHz parts: fPCP = fCPU/2 = 120 MHz; 200 MHz parts: fPCP = fCPU) but its register is in the
//!   (unavailable) TC1791 UM; SIMOS18.1 writes CCUCON0 = 0x00010001 -> 90 MHz with this decode.
//! * The GPTA is advanced lazily to the current CPU time on every GPTA register access and at every
//!   quantum boundary; quanta are cut at the next sensor edge so DCM/GTC captures and the PCP response to
//!   them start exactly at the edge. The PCP runs in parallel with the CPU: it gets the PCP cycles of the
//!   elapsed CPU time while it has work (quanta are cut to [`PCP_SLICE_INSNS`] then), and it cannot bank
//!   idle time beyond [`PCP_IDLE_BANK`] cycles (its arbitration/start latency).
//!
//! Service requests: GPTA requests set SRR in the SoC SRC registers (`stm_icu`); TOS=CPU requests reach
//! the ICU as before, TOS=PCP requests are arbitrated by the PCP through [`FpiBus::pcp_pending`]. The PCP's
//! own CPU-routed nodes (PCP_SRC0/1, SRC4..8 with TOS=CPU) are mirrored into the register file at their
//! FPI addresses so the existing ICU arbitrates and acknowledges them; acknowledges are copied back.
//!
//! Sensors: the sensor wheels are engine hardware. [`Simos18Board::feed_sensor_edges`] takes timestamped
//! electrical sensor edges from the plant (single source of truth in the studio: `simemu-physics`
//! `crank_train`, 60-2 + 2×8 cam edges) and drives the GPTA0 input pins they are wired to.
//! [`Simos18Board::feed_crank`] is the bench variant without a plant (test motor): a board-side
//! [`CrankWheel`] with the same EA888 geometry turns a crank angle trajectory into edges.
//!
//! Outputs: every GPTA cell output transition is recorded per cell ([`CellTap`]) with board time and the
//! crank angle at that instant; GTC transitions are also kept in a bounded log for analysis. The decoding
//! of cells into actuator quantities lives in [`EngineIoState`] (see its field docs for the evidence).
//! The GPTA output lines that the MSCs serialise to the ATIC140 stages are fed to the power stages as MSC
//! alternate inputs after every output change ([`EngineIo::msc_altin`]: OMCR + MMXCTR decode).

use crate::crank_wheel::{CrankWheel, WheelGeometry, LINE_CAM0, LINE_CAM1, LINE_CRANK};
use crate::gpta::{CellKind, Gpta, InputEdge, OutputEvent};
use crate::gpta_trace::GptaTrace;
use crate::memory::{BoardVariant, Tc1791HardwareState};
use crate::pcp::{FpiBus, Pcp, PCP_REGS};
use crate::stm_icu::{CYCLES_PER_INSN, SRC_SRE, SRC_SRPN, SRC_SRR, SRC_TOS};
use crate::Simos18Board;
use serde::Serialize;
use std::collections::VecDeque;

/// PCP cycles an idle PCP may use at once after a request appears (arbitration + context restore latency).
pub const PCP_IDLE_BANK: u64 = 64;
/// CPU instructions per quantum while the PCP has work (its writes become visible with this granularity).
pub const PCP_SLICE_INSNS: u64 = 128;
/// Board time resolution of sensor edges (ns).
const NS_PER_S: f64 = 1e9;
const GTC_LOG_LEN: usize = 8192;
const TRAJ_LEN: usize = 8192;
const GTC_HIST_LEN: usize = 16;

/// Board wiring of the ignition outputs: the coil of cylinder 1..4 is driven by G0.GTC15/12/14/13.
/// Evidence: ignition channel table 0x8004F930 (ch0..3 = G0.GTC15..12, doc §6.1) and the firmware's own
/// scheduling at a motored 1000 rpm: each cell's dwell ends 6.7° before TDC 0/540/180/360 °CRK
/// (cylinders 1/2/3/4 of the 1-3-4-2 firing order), so the harness must connect them this way.
pub const IGNITION_GTC_BY_CYL: [u8; 4] = [15, 12, 14, 13];
/// Ignition output polarity: idle (and parked, CTR 0x5808) high, dwell = low phase, spark = rising edge.
/// Evidence: the low phase is the short one (1.28 ms at 1000 rpm cranking vs 118.7 ms high).
pub const IGNITION_ACTIVE_LEVEL: bool = false;

/// Board wiring of the injector driver timing inputs (ATIC121 channel inputs): the injector of cylinder 1..4
/// is switched by G0.GTC04/17/16/05 (GPTA0 OUT0/OUT55/OUT54/OUT6, pins P2.8/P9.7/P9.6/P2.14, doc §12).
/// Evidence: the injection output configuration 0x8005D88C parks the cells in logical-cylinder (firing)
/// order {4, 16, 5, 17} = cylinders 1-3-4-2 (CTR 0x5800 at 0x8005D890, VERIFIED-S); PCP channels 0x29
/// (G0.GTC20/21, bank of GTC04/16) and 0x28 (G0.GTC22/23, bank of GTC05/17) carry the cell addresses in
/// their PRAM blocks 0xE20/0xF20 (VERIFIED-T); while cranking every cell pulses at 85° and 43° before
/// exactly one cylinder's firing TDC of [`IGNITION_GTC_BY_CYL`]'s coils (VERIFIED-T, `studio::engine_run`).
pub const INJECTOR_GTC_BY_CYL: [u8; 4] = [4, 17, 16, 5];
/// Injector request polarity: parked high (CTR 0x5800 = OIA, OCM set), active = low phase (0.7..1 ms while
/// cranking vs. hundreds of ms high, VERIFIED-T).
pub const INJECTOR_ACTIVE_LEVEL: bool = false;

/// GPTA input line of the start request wire KL50 (pin P4.2 = IN26 of GPTA0/GPTA1/LTCA2, DS p.39).
/// Evidence (SC800S50): the hardware start-request memory of FR p.8619/8620 ("Hardware-Startwunsch-
/// speicher") is G1.LTC60, a free-running timer with ILM = 1 (CTR 0x902, LIMCR source IN26) that counts
/// fGPTA while the pin is high; its overflow toggles OUT27 (P4.3) which G1.LTC61 (IN27, both edges)
/// counts. The BSW reads both XRs (0x800A5B1C: (LTC61.XR << 16 | LTC60.XR) · 1000 / 90 MHz = ms), the
/// ENSS start-request logic 0x801AD304 compares successive readings (c_t_str_swi_st_req_diff_min, 6 ms)
/// and writes the discharged value back (0x800A5BA0). Active level high (AIL = 0): the counter must not
/// run with the wire released.
pub const LINE_KL50: u8 = 26;

/// GPTA input line of the oil level / temperature sender G266 (pin P3.4 = IN12, DS p.36): its pulse train is measured by
/// the G1.LTC40 quad (CLK7 period/duty capture; ISR 0x8008DFE8 → BSW 0x8008DCBC, SC800S50). VERIFIED-T: a square wave on
/// IN12 moves the ENLU variables, on the other three CLK7 quads' lines it does not (docs/SIMOS18_GPTA_IO_RE.md §12.9);
/// the firmware decodes the TOG frame of `simemu_physics::sensors::OilLevelSender` (`tests/oil_sensors.rs`).
pub const LINE_G266: u8 = 12;

/// Port pins driven by a GPTA output line and read back as the pin's own GPTA input line: (kernel of
/// the driving output, line). P4.3 = GPTA1 OUT27: IOCR0 PC3 = 0xA (alternate output 2, VERIFIED-T),
/// DS p.39 (O2 = OUT27 of GPTA1, input IN27 of GPTA0/GPTA1/LTCA2). SC800S50 routes G1.LTC60 to OUT27
/// (OMCR) and counts its overflow toggles with G1.LTC61 (LIMCR IN27).
const GPTA_LOOPBACK_PINS: [(usize, u8); 1] = [(1, 27)];

/// ECU-internal test voltage of the level-2 "monitoring of A/D conversion" (FR p.4939, ECM2 17.5): a
/// duty-cycle varying PWM, first-order filtered in hardware, converted by the MC (VP_MC_AN_DIG_MON) and by
/// the monitoring unit (VP_MU_AN_DIG_MON); the firmware requires the MC value to move between its 40 ms
/// samples (alive check, c_vp_min_thd_alv_chk_mon), to stay in range and to agree with the MU value
/// within c_vp_acq_conv_hys_mon (0.31 V). A stuck node sets LV_ERR_VP_ALV_CHK_MON (0xD001C097) and the
/// ADC error 0xD001C033, whose fault reaction requests cylinder cut-off (0xD001C0CF -> LV_TQ_SCC_REQ
/// 0xD000169D -> LV_AUTH_TQ_PAT -> NR_PAT_SCC = 4: no injection at all, VERIFIED-T).
/// PWM: ACTION_INFR_SetPwmDucyADCMon = 0x800629A4 -> BSW PWM 0x800A8758 channel 0x14; PWM table
/// *(0xD0000744) = 0x80042400, entry 0x14 = cell 0x258 = LTCA2 LTC56 quad, output cell LTC59 (VERIFIED-S/T).
pub const ADC_MON_PWM_CELL: (u8, u8) = (2, 59);
/// MC conversion: VP_ADC_MC = 0x8006E2D8 -> analog getter 0x800A70FC(id 1) = ADC0 result buffer 1 =
/// ADC0.CH14 (buffer i = CH(15 - i), `sensors_io`). VERIFIED-S.
pub const ADC_MON_ADC: (usize, usize) = (0, 14);
/// PWM high level = port supply VDDP (TC1791 DS p.87: 3.3 V typ.). Consistent with the firmware's range
/// check c_vp_max_thd_rng_mon = 3.15 V at the largest duty c_pwm_max_thd_ducy_mon = 91 % (3.0 V); a 5 V
/// level would read 4.55 V and fail it. UNVERIFIED: exact level, drop under load.
pub const ADC_MON_VOH: f64 = 3.3;
/// RC time constant of the filter. UNVERIFIED (no schematic): well above the PWM period (ripple far below
/// the 0.31 V MC/MU tolerance) and below the 40 ms duty step of the firmware.
pub const ADC_MON_TAU_S: f64 = 0.010;

/// First-order RC low-pass driven by a digital output (exact for piecewise-constant input).
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct RcNode {
    /// Node voltage (V).
    pub v: f64,
    t_ns: u64,
    level: bool,
}

impl RcNode {
    #[inline]
    pub fn advance(&mut self, t_ns: u64) {
        if t_ns > self.t_ns {
            let dt = t_ns - self.t_ns;
            let target = if self.level { ADC_MON_VOH } else { 0.0 };
            let x = dt as f64 * (1e-9 / ADC_MON_TAU_S);
            let alpha = if x <= 0.01 {
                x * (1.0 - 0.5 * x * (1.0 - (1.0 / 3.0) * x * (1.0 - 0.25 * x)))
            } else {
                1.0 - (-x).exp()
            };
            self.v += (target - self.v) * alpha;
            self.t_ns = t_ns;
        }
    }

    pub fn drive(&mut self, t_ns: u64, level: bool) {
        self.advance(t_ns);
        self.level = level;
    }
}

/// Last transitions of one GPTA cell output (read-only tap).
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct CellTap {
    pub level: bool,
    pub transitions: u64,
    pub last_rise_ns: u64,
    pub last_fall_ns: u64,
    /// Unwrapped crank angle (°CRK, TDC0 = 0 mod 720) at the last rising/falling transition; NaN when the
    /// crank trajectory is unknown at that time.
    pub last_rise_deg: f64,
    pub last_fall_deg: f64,
    /// Durations of the last completed high and low phases.
    pub high_ns: u64,
    pub low_ns: u64,
}

/// One GTC output transition (log for analysis/tests).
#[derive(Clone, Copy, Debug, Serialize)]
pub struct GtcEvent {
    pub t_ns: u64,
    pub kernel: u8,
    pub cell: u8,
    pub level: bool,
    pub deg: f64,
    pub gt1: u32,
}

pub struct EngineIo {
    pub gpta: Gpta,
    /// `None` only while the PCP is executing (it is moved out to borrow the board as its FPI bus).
    pcp: Option<Box<Pcp>>,
    synced_insns: u64,
    /// Board time of `gpta.now()`.
    synced_ns: u64,
    cyc_frac: u64,
    pcp_synced_insns: u64,
    pcp_frac: u64,
    pcp_credit: u64,
    pcp_busy: bool,
    /// Pending sensor edges, `t` in ns of board time, sorted.
    edges: VecDeque<InputEdge>,
    wheel: Option<CrankWheel>,
    /// Last level applied per GPTA0 input line (lines 0..63, bit set = known / high).
    pins_known: u64,
    pins_level: u64,
    /// Crank trajectory samples (board ns, unwrapped °CRK) for the angle of output events.
    traj: VecDeque<(u64, f64)>,
    taps: Vec<CellTap>,
    pub gtc_log: VecDeque<GtcEvent>,
    /// Last transitions per GTC cell (index kernel·32 + cell).
    gtc_hist: Vec<VecDeque<GtcEvent>>,
    /// SRC nodes with SRE & SRR & TOS = PCP (maintained by `src_put`, like `src_active` for the CPU).
    pcp_active: Vec<u32>,
    /// Decoded output multiplexer of GPTA0/1, keyed by the MRA generation it was decoded from.
    omcr: [(u32, OmcrTable); 2],
    omcr_hi_gtc: [u32; 2],
    omcr_hi_ltc: [u64; 2],
    altin_key: ([u32; 4], u32, u32),
    scratch_reqs: Vec<u32>,
    scratch_outs: Vec<OutputEvent>,
    scratch_edges: Vec<InputEdge>,
    /// Sensor edges delivered to the GPTA (diagnostics).
    pub edges_applied: u64,
    /// MRA generations the pin loopbacks were last derived from (GPTA0, GPTA1).
    loop_mra_gen: [u32; 2],
    /// A/D-monitoring test voltage (RC node behind [`ADC_MON_PWM_CELL`], see its docs).
    pub adc_mon: RcNode,
    /// Recorder of the GPTA stimulus (`SIMEMU_GPTA_TRACE`, [`crate::gpta_trace`]); opened at the first sync.
    gpta_trace: Option<Box<GptaTrace>>,
    gpta_trace_tried: bool,
}

type OmcrTable = Vec<Option<(CellKind, u8)>>;

static EMPTY_CELL_TAP: CellTap = CellTap {
    level: false,
    transitions: 0,
    last_rise_ns: 0,
    last_fall_ns: 0,
    high_ns: 0,
    low_ns: 0,
    last_rise_deg: f64::NAN,
    last_fall_deg: f64::NAN,
};

static EMPTY_GTC_EVENTS: VecDeque<GtcEvent> = VecDeque::new();

fn tap_index_checked(kernel: u8, kind: CellKind, cell: u8) -> Option<usize> {
    if kernel >= 3 {
        return None;
    }
    match kind {
        CellKind::Gtc if cell < 32 => Some(kernel as usize * 96 + cell as usize),
        CellKind::Ltc if cell < 64 => Some(kernel as usize * 96 + 32 + cell as usize),
        _ => None,
    }
}

fn gtc_hist_index_checked(kernel: u8, cell: u8) -> Option<usize> {
    (kernel < 3 && cell < 32).then(|| kernel as usize * 32 + cell as usize)
}

impl EngineIo {
    pub fn new() -> Self {
        Self {
            gpta: Gpta::default(),
            pcp: Some(Box::new(Pcp::new())),
            synced_insns: 0,
            synced_ns: 0,
            cyc_frac: 0,
            pcp_synced_insns: 0,
            pcp_frac: 0,
            pcp_credit: 0,
            pcp_busy: false,
            edges: VecDeque::new(),
            wheel: None,
            pins_known: 0,
            pins_level: 0,
            traj: VecDeque::new(),
            taps: vec![CellTap { last_rise_deg: f64::NAN, last_fall_deg: f64::NAN, ..Default::default() }; 3 * 96],
            gtc_log: VecDeque::new(),
            gtc_hist: vec![VecDeque::new(); 3 * 32],
            pcp_active: Vec::new(),
            omcr: [(u32::MAX, Vec::new()), (u32::MAX, Vec::new())],
            omcr_hi_gtc: [0; 2],
            omcr_hi_ltc: [0; 2],
            altin_key: ([u32::MAX; 4], u32::MAX, u32::MAX),
            scratch_reqs: Vec::with_capacity(16),
            scratch_outs: Vec::with_capacity(16),
            scratch_edges: Vec::with_capacity(16),
            edges_applied: 0,
            loop_mra_gen: [u32::MAX; 2],
            adc_mon: RcNode::default(),
            gpta_trace: None,
            gpta_trace_tried: false,
        }
    }

    /// Re-derive the pin loopbacks ([`GPTA_LOOPBACK_PINS`]) after the output multiplexer changed.
    fn update_pin_loops(&mut self) {
        let gens = [self.gpta.mra_gen(0), self.gpta.mra_gen(1)];
        if self.loop_mra_gen == gens {
            return;
        }
        self.loop_mra_gen = gens;
        let mut loops = Vec::new();
        for &(k, line) in &GPTA_LOOPBACK_PINS {
            if let (true, Some((kind, c))) = (self.gpta.mra(k).1, self.omcr_src(k).get(line as usize).copied().flatten()) {
                loops.push((k as u8, kind, c, line));
            }
        }
        if let Some(tr) = self.gpta_trace.as_mut() {
            tr.loops(&loops);
        }
        self.gpta.set_pin_loops(&loops);
    }

    pub fn pcp(&self) -> &Pcp {
        self.pcp.as_ref().expect("PCP not executing")
    }

    pub fn pcp_mut(&mut self) -> &mut Pcp {
        self.pcp.as_mut().expect("PCP not executing")
    }

    /// Output tap of a cell (`kernel` 0 = GPTA0, 1 = GPTA1, 2 = LTCA2).
    pub fn tap(&self, kernel: u8, kind: CellKind, cell: u8) -> &CellTap {
        tap_index_checked(kernel, kind, cell)
            .and_then(|i| self.taps.get(i))
            .unwrap_or(&EMPTY_CELL_TAP)
    }

    /// Sensor wheel (after the first [`Simos18Board::feed_crank`]).
    pub fn wheel(&self) -> Option<&CrankWheel> {
        self.wheel.as_ref()
    }

    /// Unwrapped crank angle at board time `t_ns` from the fed trajectory (linear), NaN if unknown.
    pub fn angle_at(&self, t_ns: u64) -> f64 {
        let len = self.traj.len();
        if len >= 2 {
            let (t0, a0) = self.traj[len - 2];
            let (t1, a1) = self.traj[len - 1];
            if t_ns >= t0 {
                if t_ns < t1 && t1 > t0 {
                    return a0 + (a1 - a0) * (t_ns - t0) as f64 / (t1 - t0) as f64;
                } else if t_ns == t1 {
                    return a1;
                }
                return f64::NAN;
            }
        }
        let i = self.traj.partition_point(|s| s.0 <= t_ns);
        match (i.checked_sub(1).and_then(|j| self.traj.get(j)), self.traj.get(i)) {
            (Some(&(t0, a0)), Some(&(t1, a1))) if t1 > t0 => a0 + (a1 - a0) * (t_ns - t0) as f64 / (t1 - t0) as f64,
            (Some(&(t0, a0)), None) if t_ns == t0 => a0,
            _ => f64::NAN,
        }
    }

    fn record(&mut self, ev: &OutputEvent, t_ns: u64) {
        let Some(idx) = tap_index_checked(ev.kernel, ev.kind, ev.cell) else { return };
        let deg = self.angle_at(t_ns);
        let tap = &mut self.taps[idx];
        if tap.transitions > 0 && tap.level != ev.level {
            if ev.level {
                tap.low_ns = t_ns.saturating_sub(tap.last_fall_ns);
            } else {
                tap.high_ns = t_ns.saturating_sub(tap.last_rise_ns);
            }
        }
        tap.level = ev.level;
        tap.transitions += 1;
        if ev.level {
            tap.last_rise_ns = t_ns;
            tap.last_rise_deg = deg;
        } else {
            tap.last_fall_ns = t_ns;
            tap.last_fall_deg = deg;
        }
        if ev.kind == CellKind::Gtc {
            let e = GtcEvent { t_ns, kernel: ev.kernel, cell: ev.cell, level: ev.level, deg, gt1: ev.gt1 };
            if self.gtc_log.len() == GTC_LOG_LEN {
                self.gtc_log.pop_front();
            }
            self.gtc_log.push_back(e);
            let h = &mut self.gtc_hist[ev.kernel as usize * 32 + ev.cell as usize];
            if h.len() == GTC_HIST_LEN {
                h.pop_front();
            }
            h.push_back(e);
        }
    }
}

/// One active phase of an output (board time s, unwrapped crank angle °CRK at both edges).
#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
pub struct Pulse {
    pub start_s: f64,
    pub start_deg: f64,
    pub end_s: f64,
    pub end_deg: f64,
}

impl Pulse {
    pub fn width_ms(&self) -> f64 {
        (self.end_s - self.start_s) * 1e3
    }
}

/// Read-only decode of the GPTA engine outputs (board view, no actuator physics): what the firmware drives
/// on the timing pins. Power-stage gating (ATIC140/121 enables, relays) is not part of this view.
#[derive(Clone, Debug, Default, Serialize)]
pub struct EngineIoState {
    pub t_s: f64,
    /// Latest crank angle of the fed trajectory (unwrapped °CRK), NaN when unknown.
    pub crank_deg: f64,
    /// Last completed coil dwell of cylinder 1..4 ([`IGNITION_GTC_BY_CYL`]): end = spark.
    pub coil: [Option<Pulse>; 4],
    /// Dwell in progress.
    pub coil_on: [bool; 4],
    /// Recent complete injector requests of cylinder 1..4 ([`INJECTOR_GTC_BY_CYL`], oldest first).
    pub injector: [Vec<Pulse>; 4],
    /// Recent complete pulses of the angle-scheduled outputs of the driver 0x800DC188 (doc §6.2: G1.GTC08..15,
    /// G0.GTC24, G1.GTC24, G1.GTC26), both polarities: (kernel, cell, high pulses, low pulses). G1.GTC26 =
    /// HPFP valve N276; G1.GTC08..15 are the valve-lift (AVS) actuator channels: they are scheduled only by the
    /// VVL controller 0x800CAB24 (c_n_min_vvl_ctl, c_ctr_abc_max_*_susp) through 0x800BBE74/0x800BBF20
    /// (VERIFIED-S), not by the injection driver.
    pub angle_outputs: Vec<(u8, u8, Vec<Pulse>, Vec<Pulse>)>,
}

impl EngineIo {
    /// Bookkeeping of PCP-routed requests on every SRC register update.
    #[inline]
    pub(crate) fn note_src(&mut self, addr: u32, v: u32) {
        let candidate = v & (SRC_SRE | SRC_SRR) == SRC_SRE | SRC_SRR && (v & SRC_TOS) >> 10 == 1;
        match self.pcp_active.iter().position(|&a| a == addr) {
            Some(i) if !candidate => {
                self.pcp_active.swap_remove(i);
            }
            None if candidate => self.pcp_active.push(addr),
            _ => {}
        }
    }

    /// Complete phases of `level` of a GTC cell from its recent transitions.
    pub fn gtc_pulses(&self, kernel: u8, cell: u8, level: bool) -> Vec<Pulse> {
        let Some(idx) = gtc_hist_index_checked(kernel, cell) else { return Vec::new() };
        let h = &self.gtc_hist[idx];
        h.iter()
            .zip(h.iter().skip(1))
            .filter(|(a, b)| a.level == level && b.level != level)
            .map(|(a, b)| Pulse { start_s: a.t_ns as f64 / NS_PER_S, start_deg: a.deg, end_s: b.t_ns as f64 / NS_PER_S, end_deg: b.deg })
            .collect()
    }

    /// Most recent complete phase of `level` of a GTC cell without allocating a `Vec`.
    pub fn last_gtc_pulse(&self, kernel: u8, cell: u8, level: bool) -> Option<Pulse> {
        let h = &self.gtc_hist[gtc_hist_index_checked(kernel, cell)?];
        h.iter()
            .zip(h.iter().skip(1))
            .rev()
            .find(|(a, b)| a.level == level && b.level != level)
            .map(|(a, b)| Pulse { start_s: a.t_ns as f64 / NS_PER_S, start_deg: a.deg, end_s: b.t_ns as f64 / NS_PER_S, end_deg: b.deg })
    }

    /// Latest crank angle of the fed trajectory (unwrapped °CRK), NaN when unknown.
    pub fn latest_crank_deg(&self) -> f64 {
        self.traj.back().map_or(f64::NAN, |s| s.1)
    }

    /// Recent transitions of a GTC cell, oldest first (an event may repeat the level it already had).
    pub fn gtc_events(&self, kernel: u8, cell: u8) -> &VecDeque<GtcEvent> {
        gtc_hist_index_checked(kernel, cell)
            .and_then(|i| self.gtc_hist.get(i))
            .unwrap_or(&EMPTY_GTC_EVENTS)
    }

    /// Current output level of a cell.
    pub fn level(&self, kernel: u8, kind: CellKind, cell: u8) -> bool {
        self.tap(kernel, kind, cell).level
    }

    /// Decode at board time `t_s`.
    pub fn state(&self, t_s: f64) -> EngineIoState {
        let mut st = EngineIoState { t_s, crank_deg: self.latest_crank_deg(), ..Default::default() };
        for (c, &cell) in IGNITION_GTC_BY_CYL.iter().enumerate() {
            st.coil[c] = self.last_gtc_pulse(0, cell, IGNITION_ACTIVE_LEVEL);
            let tap = self.tap(0, CellKind::Gtc, cell);
            st.coil_on[c] = tap.transitions > 1 && tap.level == IGNITION_ACTIVE_LEVEL;
        }
        for (c, &cell) in INJECTOR_GTC_BY_CYL.iter().enumerate() {
            st.injector[c] = self.gtc_pulses(0, cell, INJECTOR_ACTIVE_LEVEL);
        }
        let cells = (8..16).map(|c| (1u8, c)).chain([(0, 24), (1, 24), (1, 26)]);
        st.angle_outputs = cells.map(|(k, c)| (k, c, self.gtc_pulses(k, c, true), self.gtc_pulses(k, c, false))).collect();
        st
    }
}

impl Default for EngineIo {
    fn default() -> Self {
        Self::new()
    }
}

/// Source cell of every GPTA0/GPTA1 output line OUT[0..112) from the output multiplexer control registers
/// (TC1796 UM 24.2.4.3, p.24-97..99 / 24-194..197): OMCRHg/OMCRLg for I/O groups g = 0..6 (OUT[8g..8g+7])
/// and output groups g = 7..13 (OUT[56+8(g-7)..]); byte n of OMCRL (OMCRH) controls line n (4+n): OML [2:0]
/// = line of the selected 8-cell group, OMG [5:4] = 0 GTC group g%4, 1 LTC group g%4, 2 LTC group 4+g%4,
/// 3 reserved (output forced 0). The MRA FIFO is written OMCRH(top) .. OMCRL0, LIMCR, GIMCR (p.24-110);
/// SIMOS18.1 writes 54 words = 30 OMCR words (one group above OG6, not used here) + 16 LIMCR + 8 GIMCR.
/// Evidence for this decode on SC800S50: OUT0/OUT6/OUT54/OUT55 (the GPTA0 pins P2.8/P2.14/P9.6/P9.7, doc
/// §6.3) come out as G0.GTC04/05/16/17 — exactly the four GTCs the firmware parks at 0x5800 — and OG6
/// lines OUT108..111 as G0.GTC15/14/13/12, the ignition cells.
/// The TC1766 array has 52 words: 28 OMCR words, the top one for group 13 ([`Gpta::tc1766`]).
fn omcr_sources(mra: &[u32]) -> Vec<Option<(CellKind, u8)>> {
    let mut v = vec![None; 112];
    if mra.len() < 52 {
        return v;
    }
    let n_omcr = mra.len() - 24;
    for (i, &w) in mra[..n_omcr].iter().enumerate() {
        let g = n_omcr / 2 - 1 - i / 2;
        if g > 13 {
            continue;
        }
        let base = if g < 7 { 8 * g } else { 56 + 8 * (g - 7) } + if i % 2 == 0 { 4 } else { 0 };
        let q = (g % 4) as u8;
        for j in 0..4 {
            let b = (w >> (8 * j)) as u8;
            let l = b & 7;
            v[base + j] = match (b >> 4) & 3 {
                0 => Some((CellKind::Gtc, 8 * q + l)),
                1 => Some((CellKind::Ltc, 8 * q + l)),
                2 => Some((CellKind::Ltc, 32 + 8 * q + l)),
                _ => None,
            };
        }
    }
    v
}

impl EngineIo {
    /// Level of output line OUT[line] of GPTA0 (`kernel` 0) or GPTA1 (1); low when the multiplexer array is
    /// disabled or the line is unrouted (p.24-111).
    pub fn out_line(&mut self, kernel: usize, line: usize) -> bool {
        self.gpta.mra(kernel).1 && self.omcr_src(kernel).get(line).copied().flatten().is_some_and(|(kind, c)| self.level(kernel as u8, kind, c))
    }

    /// Output lines `OUT[n]` of GPTA0 (`kernel` 0) / GPTA1 (1) that the output multiplexer routes cell
    /// (`kind`, `cell`) to. LTCA2 (2): none — its own output multiplexer words (the first 16 of its 32 MRA words) are
    /// not decoded.
    pub fn out_lines_of(&mut self, kernel: usize, kind: CellKind, cell: u8) -> Vec<usize> {
        let src = self.omcr_src(kernel);
        (0..src.len()).filter(|&n| src[n] == Some((kind, cell))).collect()
    }

    fn omcr_src(&mut self, k: usize) -> &[Option<(CellKind, u8)>] {
        if k > 1 {
            return &[];
        }
        let gen = self.gpta.mra_gen(k);
        if self.omcr[k].0 != gen || self.omcr[k].1.is_empty() {
            let mra = self.gpta.mra(k).0;
            let src = omcr_sources(mra);
            let mut gtc = 0u32;
            let mut ltc = 0u64;
            for &(kind, c) in src.iter().skip(56).flatten() {
                match kind {
                    CellKind::Gtc if c < 32 => gtc |= 1u32 << c,
                    CellKind::Ltc if c < 64 => ltc |= 1u64 << c,
                    _ => {}
                }
            }
            self.omcr_hi_gtc[k] = gtc;
            self.omcr_hi_ltc[k] = ltc;
            self.omcr[k] = (gen, src);
        }
        &self.omcr[k].1
    }

    /// MSC0/MSC1 alternate inputs ALTINL (bits 15:0) / ALTINH (bits 31:16) from the GPTA-to-MSC multiplexer
    /// (GPTA0 MMXCTR00/01/10/11 at +0x700..0x70C, TC1796 UM p.24-257..261, Table 24-29: MSC0 ALTINL.n =
    /// OUT[56+n], ALTINH.n = OUT[72+n]; MSC1 ALTINL.n = OUT[88+n], ALTINH.n = OUT[104+n], n < 8; field 0 =
    /// GPTA0, 1 = GPTA1, 2 = LTCA2). UNVERIFIED/absent: LTCA2-sourced lines read low (the layout of the
    /// LTCA2 output multiplexer words is not decoded).
    pub fn msc_altin(&mut self) -> [u32; 2] {
        let mmx = self.gpta.mmxctr();
        let maen = [self.gpta.mra(0).1, self.gpta.mra(1).1];
        self.omcr_src(0);
        self.omcr_src(1);
        let mut alt = [0u32; 2];
        for (m, a) in alt.iter_mut().enumerate() {
            for half in 0..2 {
                let reg = mmx[2 * m + half];
                for n in 0..if m == 1 && half == 1 { 8 } else { 16 } {
                    let line = 56 + 32 * m + 16 * half + n;
                    let k = ((reg >> (2 * n)) & 3) as usize;
                    let on = k < 2 && maen[k] && self.omcr[k].1[line].is_some_and(|(kind, c)| self.level(k as u8, kind, c));
                    *a |= (on as u32) << (16 * half + n);
                }
            }
        }
        alt
    }
}

/// A request is pending on one of the PCP's own nodes routed to the PCP (p.11-33).
#[inline]
fn pcp_internal_pending(pcp: &Pcp) -> bool {
    let s = &pcp.src;
    if (s[2] | s[3] | s[4] | s[5] | s[6] | s[7] | s[8] | s[9] | s[10] | s[11]) & SRC_SRR == 0 {
        return false;
    }
    (2..12).any(|i| {
        let v = s[i];
        let tos_pcp = matches!(i, 2 | 3 | 9..=11) || (v >> 10) & 3 == 1;
        tos_pcp && v & SRC_SRR != 0
    })
}

/// Clock ratio of the GPTA from GPTA0 FDR (AUDO fractional divider): `(num, den)` such that
/// fGPTA = fFPI · num / den, `None` when the module clock is off.
fn fdr_ratio(fdr: u32) -> Option<(u64, u64)> {
    let step = (fdr & 0x3FF) as u64;
    match (fdr >> 14) & 3 {
        1 => Some((1, 1024 - step)),
        2 if step > 0 => Some((step, 1024)),
        _ => None,
    }
}

impl Tc1791HardwareState {
    /// GPTA + PCP run on the SIMOS18.1 (TC1791) and DQ250 (TC1766, same AUDO GPTA/PCP2 register map,
    /// TC1796 UM ch. 11/24) boards. The SIMOS18-only wiring below (A/D-monitoring RC node, knock-window
    /// FADC gate, MSC alternate inputs) is not connected on the DQ250 board.
    #[inline]
    fn eio_on(&self) -> bool {
        self.variant != BoardVariant::Dq200Tc1784
    }

    fn abs_insns(&self) -> u64 {
        self.total_instructions + self.quantum_executed
    }

    /// Bring the GPTA up to the current CPU time: apply the sensor edges that are due, advance the timer
    /// cells, set SRR of the requested service request nodes and record output transitions.
    pub(crate) fn engine_io_sync(&mut self) {
        let now_insns = self.abs_insns();
        if !self.engine_io.gpta_trace_tried {
            self.engine_io.gpta_trace_tried = true;
            self.engine_io.gpta_trace = GptaTrace::open(self.variant == BoardVariant::Dq250Tc1766);
        }
        if now_insns <= self.engine_io.synced_insns && self.engine_io.edges.is_empty() {
            return;
        }
        let now_ns = self.now_ns();
        let fpi_div = self.scu.fpidiv() as u64 + 1;
        let io = &mut self.engine_io;
        if now_insns <= io.synced_insns && io.edges.front().is_none_or(|e| e.t > now_ns) {
            return;
        }
        let d = now_insns.saturating_sub(io.synced_insns);
        io.synced_insns = now_insns;
        let (prev_ns, t0) = (io.synced_ns, io.gpta.now());
        io.synced_ns = now_ns.max(prev_ns);
        let ticks = match fdr_ratio(io.gpta.clock_regs().1) {
            Some((num, den)) => {
                let c = io.cyc_frac + d * CYCLES_PER_INSN * num;
                let unit = den * fpi_div;
                io.cyc_frac = c % unit;
                c / unit
            }
            None => 0,
        };
        let span_ns = io.synced_ns - prev_ns;
        io.update_pin_loops();
        if io.edges.front().is_some_and(|e| e.t <= now_ns) {
            let n = io.edges.partition_point(|e| e.t <= now_ns);
            let mut batch = std::mem::take(&mut io.scratch_edges);
            batch.clear();
            for e in io.edges.drain(..n) {
                if e.line >= 64 {
                    continue;
                }
                let dt = e.t.saturating_sub(prev_ns);
                let k = if span_ns == 0 { ticks } else { (dt as u128 * ticks as u128 / span_ns as u128) as u64 };
                io.pins_known |= 1 << e.line;
                io.pins_level = (io.pins_level & !(1 << e.line)) | (e.level as u64) << e.line;
                batch.push(InputEdge { t: t0 + k.min(ticks), ..e });
            }
            io.edges_applied += batch.len() as u64;
            if let Some(tr) = io.gpta_trace.as_mut() {
                tr.advance(ticks, &batch);
            }
            io.gpta.advance(ticks, &batch);
            batch.clear();
            io.scratch_edges = batch;
        } else {
            if let Some(tr) = io.gpta_trace.as_mut() {
                tr.advance(ticks, &[]);
            }
            io.gpta.advance(ticks, &[]);
        }
        let simos = self.variant == BoardVariant::Simos18Tc1791;
        let dq250 = self.variant == BoardVariant::Dq250Tc1766;
        // MSC alternate inputs (`msc_altin`) are taken up after a sync with output transitions. They change only with a
        // transition of a cell on an MMXCTR line (>= 56, `omcr_hi_*`) or with MMXCTR / the multiplexer array
        // (`altin_key`); otherwise the recomputation would give the same words.
        let altin_moved = simos && io.altin_key != (io.gpta.mmxctr(), io.gpta.mra_gen(0), io.gpta.mra_gen(1));
        let mut has_k01_out = false;
        if io.gpta.pending() {
            let mut reqs = std::mem::take(&mut io.scratch_reqs);
            let mut outs = std::mem::take(&mut io.scratch_outs);
            reqs.clear();
            outs.clear();
            io.gpta.drain_into(&mut reqs, &mut outs);
            let (hi_gtc, hi_ltc, synced_ns) = (io.omcr_hi_gtc, io.omcr_hi_ltc, io.synced_ns);
            if !outs.is_empty() {
                has_k01_out = altin_moved;
                for &ev in &outs {
                    if simos && !has_k01_out && ev.kernel < 2 {
                        let k = ev.kernel as usize;
                        has_k01_out = match ev.kind {
                            CellKind::Gtc => (hi_gtc[k] >> ev.cell) & 1 != 0,
                            CellKind::Ltc => (hi_ltc[k] >> ev.cell) & 1 != 0,
                        };
                    }
                    let k = ev.t.saturating_sub(t0);
                    let t_ns = if ticks == 0 {
                        synced_ns
                    } else {
                        prev_ns + (k as u128 * span_ns as u128 / ticks as u128) as u64
                    };
                    self.engine_io.record(&ev, t_ns);
                    if dq250 && ev.kernel == 0 && ev.kind == CellKind::Ltc {
                        self.dq250_valve_edge(ev.cell, ev.level, t_ns);
                    }
                    if !simos {
                        continue;
                    }
                    if (ev.kernel, ev.kind, ev.cell) == (ADC_MON_PWM_CELL.0, CellKind::Ltc, ADC_MON_PWM_CELL.1) {
                        self.engine_io.adc_mon.drive(t_ns, ev.level);
                    }
                    // Knock window → FADC gating line (GPTA0 OUT26 → GS7, `knock_fadc` module docs).
                    if (ev.kernel, ev.kind, ev.cell) == (crate::knock_fadc::GATE_CELL.0, CellKind::Gtc, crate::knock_fadc::GATE_CELL.1) {
                        self.fadc.gate_edge(crate::knock_fadc::GATE_LINE, t_ns, ev.level);
                    }
                }
            }
            for &a in &reqs {
                self.src_set_srr(a);
            }
            reqs.clear();
            outs.clear();
            self.engine_io.scratch_reqs = reqs;
            self.engine_io.scratch_outs = outs;
        }
        if !simos {
            return;
        }
        // A/D-monitoring test voltage: the RC node feeds ADC0.CH14 of the MC and the MU's own converter.
        let io = &mut self.engine_io;
        io.adc_mon.advance(io.synced_ns);
        let v = io.adc_mon.v as f32;
        self.adc.volts[ADC_MON_ADC.0][ADC_MON_ADC.1] = v;
        self.spi.mu.adc_volts = v;
        if has_k01_out {
            // GPTA outputs serialised by the MSCs to the ATIC140 stages (alternate data inputs)
            let io = &mut self.engine_io;
            io.altin_key = (io.gpta.mmxctr(), io.gpta.mra_gen(0), io.gpta.mra_gen(1));
            self.spi.ps.msc_altin = io.msc_altin();
        }
        if !self.fadc.gate_q.is_empty()
            || self.fadc.next_conv.iter().any(|c| c.is_some_and(|t| self.sim_time_s - t >= 5e-4))
        {
            self.knock_fadc_update();
        }
    }


    fn pcp_routed_pending(&self) -> Option<u8> {
        self.engine_io.pcp_active.iter().map(|&a| (self.src_get(a) & SRC_SRPN) as u8).max()
    }

    /// PCP_SRCx FPI address (SRC0 at PCP_REGS + 0xFC, p.11-67).
    fn pcp_src_addr(i: usize) -> u32 {
        PCP_REGS + 0xFC - 4 * i as u32
    }

    /// Copy ICU acknowledges of mirrored PCP CPU requests back into the PCP.
    fn pcp_cpu_acks_in(&mut self, pcp: &mut Pcp) {
        let s = &pcp.src;
        if (s[0] | s[1] | s[4] | s[5] | s[6] | s[7] | s[8]) & SRC_SRR == 0 {
            return;
        }
        for i in [0, 1, 4, 5, 6, 7, 8] {
            if pcp.src[i] & SRC_SRR != 0 {
                let a = Self::pcp_src_addr(i);
                let mirrored = self.seg_f000.get(a);
                if mirrored & (SRC_SRR | SRC_SRE) == SRC_SRE {
                    pcp.ack_cpu_request(i);
                }
            }
        }
    }

    /// Mirror the PCP's CPU-routed request nodes into the SRC register file for the ICU.
    fn pcp_cpu_requests_out(&mut self, pcp: &mut Pcp) {
        if !pcp.cpu_irq_dirty {
            return;
        }
        pcp.cpu_irq_dirty = false;
        for i in [0, 1, 4, 5, 6, 7, 8] {
            let v = pcp.src[i];
            let tos_cpu = i < 2 || (v >> 10) & 3 == 0;
            let m = if tos_cpu { v & (SRC_SRPN | SRC_SRE | SRC_SRR) } else { 0 };
            let a = Self::pcp_src_addr(i);
            if self.seg_f000.get(a) != m {
                self.src_put(a, m);
                if m & (SRC_SRE | SRC_SRR) == SRC_SRE | SRC_SRR {
                    self.stop_requested = true;
                }
            }
        }
    }

    /// Quantum boundary: GPTA to now, then the PCP runs for the PCP cycles of the elapsed CPU time.
    pub(crate) fn engine_io_boundary(&mut self) {
        if !self.eio_on() {
            return;
        }
        self.engine_io_sync();
        let now = self.abs_insns();
        let pcp_div = ((self.scu.ccucon0 >> 16) & 0xF) as u64 + 1;
        let Some(mut pcp) = self.engine_io.pcp.take() else { return };
        let io = &mut self.engine_io;
        let c = io.pcp_frac + now.saturating_sub(io.pcp_synced_insns) * CYCLES_PER_INSN;
        io.pcp_synced_insns = now;
        io.pcp_frac = c % pcp_div;
        let elapsed = c / pcp_div;
        let credit = if io.pcp_busy { io.pcp_credit + elapsed } else { elapsed.min(PCP_IDLE_BANK) };
        if !io.pcp_busy && io.pcp_active.is_empty() && pcp.channel.is_none() && !pcp_internal_pending(&pcp) {
            io.pcp = Some(pcp);
            return;
        }
        self.pcp_cpu_acks_in(&mut pcp);
        let used = pcp.run(credit, self);
        let busy = pcp.channel.is_some() || pcp_internal_pending(&pcp) || (pcp.cs & 1 != 0 && !self.engine_io.pcp_active.is_empty());
        self.pcp_cpu_requests_out(&mut pcp);
        let io = &mut self.engine_io;
        io.pcp_busy = busy;
        io.pcp_credit = if busy { credit.saturating_sub(used) } else { 0 };
        io.pcp = Some(pcp);
    }

    /// Instructions until the next engine-I/O event that must start a quantum.
    pub(crate) fn engine_io_insns_to_next_event(&self) -> u64 {
        if !self.eio_on() {
            return u64::MAX;
        }
        let io = &self.engine_io;
        let mut best = if io.pcp_busy { PCP_SLICE_INSNS } else { u64::MAX };
        if let Some(e) = io.edges.front() {
            let dt = e.t.saturating_sub(self.now_ns()) as u128;
            let insns = (dt * self.clk_f as u128).div_ceil(1_000_000_000u128 * CYCLES_PER_INSN as u128);
            best = best.min((insns as u64).max(1));
        }
        best
    }

    /// Hardware reset of GPTA and PCP; the sensor pin levels persist (they are driven from outside).
    pub(crate) fn engine_io_reset(&mut self) {
        let (now, now_ns) = (self.abs_insns(), self.now_ns());
        let io = &mut self.engine_io;
        io.gpta = if self.variant == BoardVariant::Dq250Tc1766 { Gpta::tc1766(crate::gpta::F_GPTA_DEFAULT) } else { Gpta::default() };
        // the new GPTA counts its MRA generations from 0 again: drop what was keyed by the old ones
        io.omcr = [(u32::MAX, Vec::new()), (u32::MAX, Vec::new())];
        io.loop_mra_gen = [u32::MAX; 2];
        io.altin_key = ([u32::MAX; 4], u32::MAX, u32::MAX);
        if io.gpta_trace.is_some() {
            io.gpta_trace = GptaTrace::open(self.variant == BoardVariant::Dq250Tc1766); // a trace per GPTA instance
        }
        io.pcp = Some(Box::new(Pcp::new()));
        io.synced_insns = now;
        io.pcp_synced_insns = now;
        io.synced_ns = now_ns;
        io.cyc_frac = 0;
        io.pcp_frac = 0;
        io.pcp_credit = 0;
        io.pcp_busy = false;
        io.adc_mon.drive(now_ns, false); // GPTA outputs inactive after reset: the RC node discharges
        for line in 0..64u8 {
            if io.pins_known & (1 << line) != 0 {
                for kernel in 0..3 {
                    io.edges.push_front(InputEdge { t: 0, kernel, line, level: io.pins_level & (1 << line) != 0 });
                }
            }
        }
    }

    /// Side-effect-free peek for GPTA and PCP registers.
    pub(crate) fn engine_io_peek(&self, a: u32) -> Option<u32> {
        if !self.eio_on() {
            return None;
        }
        if Gpta::contains(a) {
            return self.engine_io.gpta.peek_word(a);
        }
        if Pcp::contains(a) {
            return self.engine_io.pcp.as_ref()?.read_word(a);
        }
        None
    }

    /// Peripheral read hook for GPTA (except SRC nodes) and PCP addresses.
    pub(crate) fn engine_io_read(&mut self, a: u32) -> Option<u32> {
        if !self.eio_on() {
            return None;
        }
        if Gpta::contains(a) {
            if self.engine_io.pcp.is_some() {
                self.engine_io_sync();
            }
            let io = &mut self.engine_io;
            if let Some(tr) = io.gpta_trace.as_mut() {
                tr.read(a);
            }
            return io.gpta.read_word(a);
        }
        if Pcp::contains(a) {
            let mut pcp = self.engine_io.pcp.take()?;
            self.pcp_cpu_acks_in(&mut pcp);
            let v = pcp.read_word(a);
            self.engine_io.pcp = Some(pcp);
            return v;
        }
        None
    }

    /// Peripheral write hook for GPTA (except SRC nodes) and PCP addresses.
    pub(crate) fn engine_io_write(&mut self, a: u32, v: u32, m: u32) -> bool {
        if !self.eio_on() || !(Gpta::contains(a) || Pcp::contains(a)) {
            return false;
        }
        if self.scu.endinit_blocks(a) {
            return true;
        }
        if Gpta::contains(a) {
            if self.engine_io.pcp.is_some() {
                self.engine_io_sync();
            }
            let io = &mut self.engine_io;
            if let Some(tr) = io.gpta_trace.as_mut() {
                tr.write(a, v, m);
            }
            return io.gpta.write_lanes(a, v, m);
        }
        let Some(mut pcp) = self.engine_io.pcp.take() else { return true };
        pcp.write_lanes(a, v, m);
        if (PCP_REGS..PCP_REGS + 0x100).contains(&a) {
            self.pcp_cpu_requests_out(&mut pcp);
            self.stop_requested = true; // PCP_CS/ICR/SRC changes: let the PCP see them at the next boundary
        }
        self.engine_io.pcp = Some(pcp);
        true
    }
}

/// The PCP's FPI bus: peripheral MMIO + SoC memories (`dspr`/`pspr`/`lmu`/`flash`; PRAM/CMEM are
/// served inside the PCP). Unmapped accesses and writes to read-only Flash return `None`/`false`
/// so the PCP raises `PCP_ES.BER`.
impl FpiBus for Tc1791HardwareState {
    #[inline]
    fn fpi_read(&mut self, addr: u32, size: u8) -> u32 {
        self.fpi_try_read(addr, size).unwrap_or(0)
    }

    #[inline]
    fn fpi_write(&mut self, addr: u32, val: u32, size: u8) {
        let _ = self.fpi_try_write(addr, val, size);
    }

    #[inline]
    fn fpi_try_read(&mut self, addr: u32, size: u8) -> Option<u32> {
        let a = addr & !3;
        let single_word = (addr & 3) + size as u32 <= 4;
        if single_word && Gpta::contains(a) && self.engine_io.gpta_trace.is_none() {
            self.mmio_read_count += 1;
            let v = self.engine_io.gpta.read_word(a).unwrap_or(0);
            let sh = (addr & 3) * 8;
            let m = if size >= 4 { u32::MAX } else { (1u32 << (size * 8)) - 1 };
            let r = (v >> sh) & m;
            if let Some(log) = self.event_log.as_mut() {
                log.push((self.total_instructions + self.quantum_executed, 0, addr, r as u64));
            }
            return Some(r);
        }
        if single_word && crate::stm_icu::is_src(a) {
            self.mmio_read_count += 1;
            let v = self.src_get(a);
            let sh = (addr & 3) * 8;
            let m = if size >= 4 { u32::MAX } else { (1u32 << (size * 8)) - 1 };
            let r = (v >> sh) & m;
            if let Some(log) = self.event_log.as_mut() {
                log.push((self.total_instructions + self.quantum_executed, 0, addr, r as u64));
            }
            return Some(r);
        }
        if crate::memory::is_mmio(addr) {
            return Some(self.mmio_read(addr, size as u32) as u32);
        }
        let s = self.sys_mem_slice(addr, size as usize)?;
        Some(s.iter().rev().fold(0, |acc, &b| (acc << 8) | b as u32))
    }

    #[inline]
    fn fpi_try_write(&mut self, addr: u32, val: u32, size: u8) -> bool {
        let a = addr & !3;
        let single_word = (addr & 3) + size as u32 <= 4;
        if single_word && Gpta::contains(a) && (a & 0x7FF) >= 0x10 && self.engine_io.gpta_trace.is_none() {
            self.mmio_write_count += 1;
            if let Some(log) = self.event_log.as_mut() {
                log.push((self.total_instructions + self.quantum_executed, 1, addr, val as u64));
            }
            let sh = (addr & 3) * 8;
            let lane = if size >= 4 { u32::MAX } else { (1u32 << (size * 8)) - 1 };
            let m = lane << sh;
            let v = (val & lane) << sh;
            self.engine_io.gpta.write_lanes(a, v, m);
            return true;
        }
        if single_word && crate::stm_icu::is_src(a) {
            self.mmio_write_count += 1;
            if let Some(log) = self.event_log.as_mut() {
                log.push((self.total_instructions + self.quantum_executed, 1, addr, val as u64));
            }
            let sh = (addr & 3) * 8;
            let lane = if size >= 4 { u32::MAX } else { (1u32 << (size * 8)) - 1 };
            let m = lane << sh;
            let v = (val & lane) << sh;
            self.src_write(a, v, m);
            return true;
        }
        if crate::memory::is_mmio(addr) {
            self.mmio_write(addr, size as u32, val as u64);
            return true;
        }
        let Some(s) = self.sys_ram_mut(addr, size as usize) else {
            return false;
        };
        s.copy_from_slice(&val.to_le_bytes()[..size as usize]);
        self.mark_ram_write_code_dirty(addr, size as u32);
        true
    }
    #[inline]
    fn pcp_pending(&mut self) -> Option<u8> {
        self.pcp_routed_pending()
    }
    fn pcp_ack(&mut self, srpn: u8) {
        let mut i = 0;
        while i < self.engine_io.pcp_active.len() {
            let a = self.engine_io.pcp_active[i];
            let v = self.src_get(a);
            if (v & SRC_SRPN) as u8 == srpn {
                self.src_put(a, v & !SRC_SRR);
            } else {
                i += 1;
            }
        }
    }
}

/// Electrical level of a sensor input at the ECU connector after an edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorSignal {
    /// Crank speed sensor G28 (CPDD: low pulse at each tooth's active edge).
    Crank(bool),
    /// Intake cam Hall sensor G40.
    CamIntake(bool),
    /// Exhaust cam Hall sensor G300.
    CamExhaust(bool),
    /// Start request wire KL50 (ignition lock / start button), high = start requested ([`LINE_KL50`]).
    StartRequest(bool),
    /// Exhaust AVS switching diagnosis, comparator output of diagnosis channel 0 (cylinder indices 0/2) / 1 (1/3): high
    /// while the recoil voltage of a returning actuator pin exceeds the threshold ([`LINES_VVL_DIAG`]).
    VvlRecoil(u8, bool),
    /// Oil level / temperature sender G266 output ([`LINE_G266`]).
    OilLevelSender(bool),
}

/// AVS recoil diagnosis inputs: channel 0 = P4.12 (IN36 → GPTA1 FPC0 → DCM0), channel 1 = P4.8 (IN32 → GPTA1 FPC4 → DCM3),
/// active high (SC800S50 driver cfg 0x80042C04 / init 0x800F81A4, FPC IPS read back, DS p.34–35; VERIFIED-S/T,
/// docs/SIMOS18_COMBUSTION.md §6b).
pub const LINES_VVL_DIAG: [(u8, u8); 2] = [(36, 12), (32, 8)];

impl SensorSignal {
    /// Board wiring: GPTA input line of the sensor pin (a port pin feeds INn of GPTA0, GPTA1 and LTCA2,
    /// DS p.39). See [`crate::crank_wheel`] for the evidence: FPC0 IN24 crank, FPC3 IN30 cam 0, FPC5 IN34
    /// cam 1 (cam 0 = intake is HYP, doc §3.3); [`LINE_KL50`] start request.
    fn pin(self) -> (u8, bool) {
        match self {
            SensorSignal::Crank(l) => (LINE_CRANK, l),
            SensorSignal::CamIntake(l) => (LINE_CAM0, l),
            SensorSignal::CamExhaust(l) => (LINE_CAM1, l),
            SensorSignal::StartRequest(l) => (LINE_KL50, l),
            SensorSignal::VvlRecoil(ch, l) => (LINES_VVL_DIAG[ch as usize & 1].0, l),
            SensorSignal::OilLevelSender(l) => (LINE_G266, l),
        }
    }
}

impl EngineIo {
    /// Input lines with a known level and their levels as of the last sync (bit n = line INn).
    pub(crate) fn input_levels(&self) -> (u64, u64) {
        (self.pins_known, self.pins_level)
    }

    pub(crate) fn push_edge(&mut self, e: InputEdge) {
        if self.edges.back().is_none_or(|x| x.t <= e.t) {
            self.edges.push_back(e);
            return;
        }
        let i = self.edges.partition_point(|x| x.t <= e.t);
        self.edges.insert(i, e);
    }

    fn push_angle(&mut self, ns: u64, deg: f64) {
        if !deg.is_finite() || self.traj.back().is_some_and(|&(t0, _)| ns <= t0) {
            return;
        }
        if self.traj.len() == TRAJ_LEN {
            self.traj.pop_front();
        }
        self.traj.push_back((ns, deg));
    }
}

impl Simos18Board {
    pub fn engine_io(&self) -> &EngineIo {
        &self.hw().engine_io
    }

    pub fn engine_io_mut(&mut self) -> &mut EngineIo {
        &mut self.hw_mut().engine_io
    }

    /// Brings the GPTA up to the current board time (as the next quantum boundary would at this same time): the output
    /// transitions up to now are recorded. Without it, transitions of the last quantum before a co-simulation window end
    /// are recorded only in the next window (back-dated), i.e. after the outputs have been decoded for the plant.
    pub fn sync_engine_io(&mut self) {
        self.hw_mut().engine_io_sync();
    }

    /// Board time (s) at the current instruction.
    pub fn board_time_s(&self) -> f64 {
        self.hw().now_ns() as f64 / NS_PER_S
    }

    /// Sensor edges from the plant: (board time s, level after the edge). Edges in the past are applied at
    /// the current time; the board cuts its execution quanta at future edges.
    /// The KL50 pin is also read as a port input (P4.2 via the BSW getter 0x800A5B04, ENSS init 0x800ECF94:
    /// LV_ST_REQ_KL50 = pin level): its port level follows immediately (port inputs are not time-scheduled).
    pub fn feed_sensor_edges(&mut self, edges: &[(f64, SensorSignal)]) {
        for &(t, s) in edges {
            if !t.is_finite() {
                continue;
            }
            let (line, level) = s.pin();
            if let SensorSignal::StartRequest(l) = s {
                self.set_port_input(4, 2, Some(l));
            }
            if let SensorSignal::VvlRecoil(ch, l) = s {
                self.set_port_input(4, LINES_VVL_DIAG[ch as usize & 1].1, Some(l));
            }
            let io = &mut self.hw_mut().engine_io;
            for kernel in 0..3 {
                io.push_edge(InputEdge { t: (t.max(0.0) * NS_PER_S) as u64, kernel, line, level });
            }
        }
    }

    /// Crank angle sample of the plant (board time s, unwrapped °CRK, 0 = firing TDC of cylinder 1): only
    /// used to report the crank angle of output transitions ([`CellTap`], [`GtcEvent`]).
    pub fn feed_crank_angle(&mut self, t: f64, deg: f64) {
        if t.is_finite() && deg.is_finite() {
            self.hw_mut().engine_io.push_angle((t.max(0.0) * NS_PER_S) as u64, deg);
        }
    }

    /// Decoded engine outputs at the current board time.
    pub fn engine_io_state(&self) -> EngineIoState {
        self.engine_io().state(self.board_time_s())
    }

    /// Bench crank drive without a plant (test motor): the crank angle trajectory as (board time s, unwrapped
    /// °CRK) samples, linear in between, turned into sensor edges by a board-side [`CrankWheel`]
    /// (EA888 geometry unless `geo` is given; the plant's wheel must be the same — see the studio glue
    /// test). `cam_shift_deg`: intake/exhaust phase (°CRK, positive = later).
    pub fn feed_crank(&mut self, samples: &[(f64, f64)], cam_shift_deg: [f64; 2], geo: Option<WheelGeometry>) {
        if !(cam_shift_deg[0].is_finite() && cam_shift_deg[1].is_finite()) {
            return;
        }
        let Some(&(t0, a0)) = samples.iter().find(|&&(t, a)| t.is_finite() && a.is_finite()) else { return };
        let io = &mut self.hw_mut().engine_io;
        if io.wheel.is_none() {
            let mut w = CrankWheel::new(geo.unwrap_or_else(WheelGeometry::simos18_sc8s50), NS_PER_S, t0, a0);
            w.advance(t0, a0, cam_shift_deg[0], cam_shift_deg[1]);
            for e in w.initial_edges() {
                io.push_edge(e);
            }
            io.wheel = Some(w);
            io.push_angle((t0.max(0.0) * NS_PER_S) as u64, a0);
        }
        let Some(mut w) = io.wheel.take() else { return };
        for &(t, a) in samples {
            if !(t.is_finite() && a.is_finite()) {
                continue;
            }
            let ns = (t.max(0.0) * NS_PER_S) as u64;
            if io.traj.back().is_some_and(|&(tl, _)| ns <= tl) {
                continue;
            }
            for e in w.advance(t, a, cam_shift_deg[0], cam_shift_deg[1]) {
                io.push_edge(e);
            }
            io.push_angle(ns, a);
        }
        io.wheel = Some(w);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gpta::GPTA0_BASE;

    fn board() -> Simos18Board {
        Simos18Board::new_variant(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791).unwrap()
    }

    /// GPTA0 multiplexer array of 54 words, all zero but OMCRL7 (word 15, byte 0 = OUT56): `out56`.
    fn program_mra(hw: &mut Tc1791HardwareState, out56: u32) {
        hw.engine_io_write(0xF000_1838, 2, !0);
        for i in 0..54 {
            hw.engine_io_write(0xF000_183C, if i == 15 { out56 } else { 0 }, !0);
        }
        hw.engine_io_write(0xF000_1838, 1, !0);
    }

    /// A GPTA reset restarts the MRA generations at 0: the output multiplexer decode keyed by them must not
    /// survive it (same number of MRA writes, other contents).
    #[test]
    fn gpta_reset_drops_the_output_multiplexer_decode() {
        let mut b = board();
        let hw = b.hw_mut();
        program_mra(hw, 0x10); // OUT56 <- LTC24 (OMG 1)
        assert_eq!(hw.engine_io.omcr_src(0)[56], Some((CellKind::Ltc, 24)));
        hw.engine_io_reset();
        program_mra(hw, 0x00); // OUT56 <- GTC24
        assert_eq!(hw.engine_io.omcr_src(0)[56], Some((CellKind::Gtc, 24)));
    }

    /// The MSC alternate inputs (`msc_altin`) are taken up after a sync with output transitions: a new MMXCTR
    /// routing at the next transition of any cell, a cell on an MMXCTR line (OUT56..) with its own transition.
    #[test]
    fn msc_alternate_inputs_follow_at_the_next_output_transition() {
        let mut b = board();
        let hw = b.hw_mut();
        let sync = |hw: &mut Tc1791HardwareState| {
            hw.total_instructions += 1;
            hw.engine_io_sync();
        };
        program_mra(hw, 0x00); // OUT56 <- GTC24; MMXCTR = 0: OUT56 -> MSC0 alternate input 0 from GPTA0
        hw.engine_io_write(GPTA0_BASE + 0x100 + 8 * 24, 0x5800, !0); // GTC24: OIA with OCM = 11 (set)
        sync(hw);
        assert_eq!(hw.spi.ps.msc_altin[0] & 1, 1, "GTC24 high on OUT56");
        hw.engine_io_write(0xF000_1F00, 3, !0); // MMXCTR: OUT56 from no kernel
        sync(hw);
        assert_eq!(hw.spi.ps.msc_altin[0] & 1, 1, "MMXCTR taken up with the next output transition only");
        hw.engine_io_write(GPTA0_BASE + 0x100, 0x5800, !0); // GTC0 (OUT line < 56) set
        sync(hw);
        assert_eq!(hw.spi.ps.msc_altin[0] & 1, 0);
    }

    #[test]
    fn ltca2_has_no_decoded_output_lines() {
        let mut io = EngineIo::new();
        assert!(io.out_lines_of(2, CellKind::Ltc, 31).is_empty());
        assert!(!io.out_line(2, 0));
    }

    /// The A/D-monitoring RC node: exact first-order step response and the mean of a fast PWM (the firmware's
    /// duty ramp only reaches the converters through it; the ripple must stay far below the 0.31 V MC/MU
    /// tolerance).
    #[test]
    fn adc_monitor_rc_node_follows_the_pwm_duty() {
        let tau_ns = (ADC_MON_TAU_S * 1e9) as u64;
        let mut n = RcNode::default();
        n.drive(0, true);
        n.advance(tau_ns);
        assert!((n.v - ADC_MON_VOH * (1.0 - (-1.0f64).exp())).abs() < 1e-9);
        // 25 % duty at 15.6 kHz (LTCA2 LTC56 quad on CLK0: 5769 ticks of 90 MHz) for 10 τ
        let (period, high) = (64_100u64, 16_025u64);
        let mut t = tau_ns;
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        while t < 11 * tau_ns {
            n.drive(t, true);
            n.drive(t + high, false);
            t += period;
            n.advance(t);
            if t > 9 * tau_ns {
                lo = lo.min(n.v);
                hi = hi.max(n.v);
            }
        }
        assert!((0.5 * (lo + hi) - 0.25 * ADC_MON_VOH).abs() < 0.02, "mean {lo}..{hi}");
        assert!(hi - lo < 0.02, "ripple {}", hi - lo);
    }
}
