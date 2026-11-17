//! FADC (fast A/D converter) of the TC1784, modelled after the TC1796 FADC kernel
//! (`vault/docs/infineon/tc1796_um_v2.0_2007_07.pdf`, chapter 26; no TC1784 manual is available).
//! The TC1784 places the kernel at `0xF0100400` (the TC1796 at `0xF0100300`); the offsets are the
//! TC1796 ones (UM Table 26-8 / 18-28), which the DQ200_G2 firmware confirms: its driver
//! `0x800E2AB0` writes CLC +0x00, FDR +0x0C, GCR +0x1C, NCTR +0x18, CFGRx +0x20, ACRx +0x30,
//! FCRn +0x60 (only if non-zero), SRC3..0 +0xF0..+0xFC, and its SR1 ISR (`0x800E40E8` ->
//! `0x800ED4E0`, SRPN 0x23 at +0xF8 = FADC_SRC1, UM Table 26-11) reads RCH0/RCH1 at +0x40/+0x44.
//!
//! Modelled (UM section in brackets): four channels with the analog input stage and gain
//! (26.1.1, eq. 26.1), 21-cycle conversion time (eq. 26.2), channel timers with CTF/CTREL and the
//! reload at conversion start (26.1.4), gating modes 00/01 and trigger by FMR.SCRF / neighbour
//! channel trigger NCTR (26.1.3, 26.1.7), static/rotating priority CRPRIO (26.1.5.2), CRSR/FMR
//! flags, channel service requests to SR0..3 (26.1.9), multiplexer test mode, RESWEN, fFADC from
//! FDR normal/fractional divider (eq. 26.3/26.4).
//! Not modelled (not used by this firmware, VERIFIED-T register trace `dq200_write_trace`
//! `DQ_TRACE=F0100400,F01004FF`): the data reduction filters (FCRn never written, filter
//! registers read 0), external trigger inputs `TS[7:0]` (TM = 00 in every CFGR), the gating inputs
//! `GS[7:0]` (GM = 01 "always enabled"; GM = 10/11 see the gating line low — UNVERIFIED), offset/gain
//! calibration (GCR.CALMODE = 0), dynamic priority assignment and DMA requests.
//!
//! Event-driven: the board asks for the next event time ([`Fadc::next_event`]) and advances the
//! model to the current instruction count ([`Fadc::advance`]); there is no per-instruction work.

/// Kernel base of the TC1784 FADC (see module doc).
pub const FADC_BASE: u32 = 0xF010_0400;
/// fFADC cycles per conversion incl. sampling and storing (UM eq. 26.2).
pub const CONV_CYCLES: f64 = 21.0;
/// FADC reference VFAREF. TC1796: 3.3 V (UM 26.1, Table 26-1). The DQ200 firmware converts its
/// pump current-sense samples with 3300 mV / 1023 codes (config table `0x8013ABE0`: 0xCE4, 0x3FF),
/// i.e. it assumes the same reference on the TC1784 (VERIFIED-S for the firmware's assumption).
pub const VFAREF: f64 = 3.3;

const CRSR: usize = 0x10 / 4;
const NCTR: usize = 0x18 / 4;
const GCR: usize = 0x1C / 4;
const CFGR: usize = 0x20 / 4;
const ACR: usize = 0x30 / 4;
const RCH: usize = 0x40 / 4;
/// GCR bits that are stored (the others are write-only actions, read as 0).
const GCR_RW: u32 = 0x0F3F_0000;

#[derive(Clone, Debug)]
pub struct Fadc {
    /// Register file, word index = offset / 4 (0x00..0xEC; the SRCs are owned by the core).
    regs: [u32; 60],
    /// Instruction time of the next channel-timer trigger per channel (None = timer not running or
    /// parked at zero until its requested conversion starts).
    timer_next: [Option<f64>; 4],
    /// Channel being converted, its sampled result and the completion time (instructions).
    busy: Option<(usize, u32, f64)>,
    /// Instructions per fFADC cycle (0 = clock off) and per fCLC = fSYS cycle ([`Fadc::set_clock`]).
    insns_per_tick: f64,
    insns_per_fsys: f64,
    /// Pin voltages FAINxP / FAINxN per channel (board side).
    pub volts: [[f32; 2]; 4],
    pub conversions: [u64; 4],
}

impl Default for Fadc {
    fn default() -> Self {
        let mut regs = [0; 60];
        regs[0] = 3; // CLC reset value: module disabled (UM 26.3.3.1)
        regs[2] = 0x0027_C001; // ID: MODNUM 0x0027, 32-bit module, revision 1 (UM 26.2.1; revision UNVERIFIED)
        Self { regs, timer_next: [None; 4], busy: None, insns_per_tick: 0.0, insns_per_fsys: 0.0, volts: [[0.0; 2]; 4], conversions: [0; 4] }
    }
}

/// Service request line (0..3) raised by an event, to be routed to FADC_SRCn by the board.
pub type SrLine = usize;

impl Fadc {
    /// Reset state with the board's pin voltages.
    pub fn with_inputs(volts: [[f32; 2]; 4]) -> Self {
        Self { volts, ..Self::default() }
    }

    fn running(&self) -> bool {
        self.regs[0] & 1 == 0 && self.regs[GCR] & (1 << 21) != 0 && self.insns_per_tick > 0.0
    }

    /// fFADC from FDR (UM eq. 26.3/26.4) given the instructions per fSYS (= fFPI) cycle.
    pub fn set_clock(&mut self, insns_per_fsys: f64, now: f64) {
        let fdr = self.regs[3];
        let step = (fdr & 0x3FF) as f64;
        let n = match (fdr >> 14) & 3 {
            1 => 1024.0 - step,
            2 if step > 0.0 => 1024.0 / step,
            _ => 0.0,
        };
        let t = insns_per_fsys * n;
        self.insns_per_fsys = insns_per_fsys;
        if t != self.insns_per_tick {
            self.insns_per_tick = t;
            for x in 0..4 {
                self.retime(x, now);
            }
        }
    }

    /// Channel-timer period in instructions, None if the timer cannot trigger (UM 26.1.4: divider
    /// CTREL + 1 for fCLC / fCT >= 2, CTREL + 2 for fCT = fCLC; CTREL = 0 generates no event).
    fn timer_period(&self, x: usize) -> Option<f64> {
        let c = self.regs[CFGR + x];
        let (gm, ctm, ctf, ctrel) = ((c >> 6) & 3, (c >> 10) & 3, (c >> 12) & 7, (c >> 16) & 0xFF);
        // gating: 00 off, 01 always, 10 = GS high, 11 = GS low; GS lines read 0 (not connected).
        let gate = matches!(gm, 1 | 3);
        if !self.running() || ctrel == 0 || !matches!(ctf, 1..=6) || !(ctm == 1 || (ctm == 2 && gate)) || (ctm == 1 && gm == 0) {
            return None;
        }
        let div = [0.0, 1.0, 4.0, 16.0, 64.0, 256.0, 1024.0][ctf as usize];
        let t_ct = div * self.insns_per_tick;
        let n = ctrel as f64 + if t_ct < 2.0 * self.insns_per_fsys { 2.0 } else { 1.0 };
        Some(n * t_ct)
    }

    fn retime(&mut self, x: usize, now: f64) {
        self.timer_next[x] = self.timer_period(x).map(|p| now + p);
    }

    fn gate_open(&self, x: usize) -> bool {
        matches!((self.regs[CFGR + x] >> 6) & 3, 1 | 3)
    }

    /// Clocked with `insns_per_fsys` already and no event due by `now`: [`Fadc::set_clock`] and
    /// [`Fadc::advance`] would change nothing.
    pub fn quiet(&self, insns_per_fsys: f64, now: f64) -> bool {
        insns_per_fsys == self.insns_per_fsys && self.next_event().is_none_or(|t| t > now)
    }

    /// Earliest pending event (instruction time).
    pub fn next_event(&self) -> Option<f64> {
        let t = self.timer_next.iter().flatten().copied().fold(f64::INFINITY, f64::min);
        let t = self.busy.map_or(t, |b| t.min(b.2));
        t.is_finite().then_some(t)
    }

    /// 10-bit result of channel `x` from its pins (UM eq. 26.1: VM = GAIN((VP - VFAREFM) +
    /// (VFAREFM - VN)), -VFAREF -> 0, 0 -> 0x200, +VFAREF -> 0x3FF; a disabled input is at VFAREFM).
    fn sample(&self, x: usize) -> u32 {
        if self.regs[GCR] & (1 << 20) != 0 {
            return 0x200; // multiplexer test mode (UM 26.1.1)
        }
        let acr = self.regs[ACR + x];
        let m = VFAREF / 2.0;
        let vp = if acr & 4 != 0 { self.volts[x][0] as f64 } else { m };
        let vn = if acr & 8 != 0 { self.volts[x][1] as f64 } else { m };
        let vm = ((1u32 << (acr & 3)) as f64 * (vp - vn)).clamp(-VFAREF, VFAREF);
        ((vm / VFAREF + 1.0) * 512.0).round().clamp(0.0, 1023.0) as u32
    }

    /// Start the highest-priority requested conversion if the converter is idle (UM 26.1.5.2).
    fn arbitrate(&mut self, now: f64) {
        if self.busy.is_some() || !self.running() {
            return;
        }
        let crf = self.regs[CRSR] & 0xF;
        if crf == 0 {
            return;
        }
        let prio = ((self.regs[GCR] >> 16) & 3) as usize;
        let Some(x) = (0..4).map(|i| (prio + i) & 3).find(|&x| crf & (1 << x) != 0) else { return };
        self.regs[CRSR] = (self.regs[CRSR] & !(1 << x)) | 1 << (8 + x);
        self.busy = Some((x, self.sample(x), now + CONV_CYCLES * self.insns_per_tick));
        // the channel timer restarts from CTREL when its requested conversion starts (26.1.4)
        if self.timer_next[x].is_none() {
            self.retime(x, now);
        }
        // neighbour channel triggers (NCTR.ENxy: conversion start of x requests y, 26.1.7)
        let nctr = self.regs[NCTR] >> (8 * x);
        for y in (0..4).filter(|&y| y != x && nctr & (1 << y) != 0) {
            if self.gate_open(y) {
                self.regs[CRSR] |= 1 << y;
            }
        }
    }

    /// Process every event up to instruction time `now`; returns a bit mask of the service request
    /// lines SR0..SR3 raised meanwhile.
    pub fn advance(&mut self, now: f64) -> u32 {
        let mut srs = 0u32;
        loop {
            let busy_t = self.busy.map_or(f64::INFINITY, |b| b.2);
            let (tx, tt) = (0..4).filter_map(|x| self.timer_next[x].map(|t| (x, t))).fold((4, f64::INFINITY), |a, b| if b.1 < a.1 { b } else { a });
            let t = busy_t.min(tt);
            if t > now {
                break;
            }
            if busy_t <= tt {
                let (x, code, _) = self.busy.take().unwrap();
                self.regs[RCH + x] = code;
                self.regs[CRSR] = (self.regs[CRSR] & !(1 << (8 + x))) | 1 << (16 + x);
                self.conversions[x] += 1;
                let cfg = self.regs[CFGR + x];
                if cfg & (1 << 31) != 0 {
                    srs |= 1 << ((cfg >> 28) & 3);
                }
                self.arbitrate(busy_t);
            } else {
                // timer reached zero: request (gate permitting) and park until the conversion starts
                self.timer_next[tx] = None;
                if self.gate_open(tx) {
                    self.regs[CRSR] |= 1 << tx;
                }
                self.arbitrate(tt);
            }
        }
        srs
    }

    pub fn read(&self, off: u32) -> u32 {
        match off {
            0x14 => 0, // FMR: write-only
            0x1C => self.regs[GCR] & GCR_RW,
            _ => self.regs.get(off as usize / 4).copied().unwrap_or(0),
        }
    }

    /// Register write (`v` already merged with the byte-lane mask by the caller where needed).
    pub fn write(&mut self, off: u32, v: u32, m: u32, now: f64) -> u32 {
        let i = off as usize / 4;
        let mut srs = 0;
        if i >= self.regs.len() {
            return 0;
        }
        let merged = (self.regs[i] & !m) | (v & m);
        let w = v & m;
        match off {
            0x08 | 0x10 => {} // ID, CRSR: read-only
            0x00 => self.regs[0] = (merged & 0x3D) | (merged & 1) << 1, // DISS follows DISR
            0x14 => {
                let c = &mut self.regs[CRSR];
                *c |= (w >> 8) & 0xF; // SCRFx
                *c &= !(w & 0xF); // RCRFx wins
                *c |= ((w >> 24) & 0x3F & !((w >> 16) & 0x3F)) << 16; // SIRQx/SIRQFn
                *c &= !(((w >> 16) & 0x3F) << 16); // RIRQx/RIRQFn
                for x in 0..4 {
                    if w & (1 << (24 + x)) != 0 && w & (1 << (16 + x)) == 0 && self.regs[CFGR + x] & (1 << 31) != 0 {
                        srs |= 1 << ((self.regs[CFGR + x] >> 28) & 3);
                    }
                }
            }
            0x1C => {
                self.regs[GCR] = merged & GCR_RW;
                for x in (0..4).filter(|x| w & (1 << x) != 0) {
                    self.retime(x, now); // RCTx
                }
                if self.regs[GCR] & (1 << 21) == 0 {
                    self.busy = None;
                }
            }
            0x40..=0x4C => {
                if self.regs[GCR] & (1 << 19) != 0 {
                    self.regs[i] = merged & 0x3FF; // RESWEN
                }
            }
            _ => self.regs[i] = merged,
        }
        if matches!(off, 0x00 | 0x1C | 0x20..=0x2C) {
            for x in 0..4 {
                // a timer parked at zero (its request pending) restarts when that conversion starts
                let parked = self.regs[CRSR] & (1 << x) != 0;
                match (self.timer_period(x), self.timer_next[x]) {
                    (None, _) => self.timer_next[x] = None,
                    (Some(_), None) if !parked => self.retime(x, now),
                    _ => {}
                }
            }
        }
        self.arbitrate(now);
        srs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The DQ200_G2 configuration (trace of `0x800E2AB0` and `0x800ED498`, fFPI 60 MHz, fCPU 120
    /// MHz): channel 0 timer-driven at fFADC = 60 MHz * 18 / 1024, CTREL 0xAF -> one conversion per
    /// 176 fFADC cycles (5.99 kHz); channel 1 neighbour-triggered by channel 0 (NCTR = 2) and raising
    /// SR1 after both conversions (2 x 21 cycles); both inputs single-ended FAINxP - VFAREF/2 at gain 2
    /// (0 V -> 0, 3.3 V -> 1023, UM Table 26-1).
    #[test]
    fn dq200_configuration_converts_ch0_ch1_at_6khz_and_raises_sr1() {
        let mut f = Fadc::default();
        let ins_per_fsys = 2.0; // fCPU / fFPI
        let mut t = 0.0;
        for (off, v) in [(0x00, 0), (0x0C, 0x8012), (0x1C, 0x0020_0000), (0x30, 5), (0x20, 0x00AF_1445), (0x34, 5), (0x24, 0x9001_0045), (0x38, 4), (0x28, 0x0001_0000), (0x3C, 4), (0x2C, 0xB001_0000), (0x18, 2)] {
            f.write(off, v, !0, t);
            f.set_clock(ins_per_fsys, t);
        }
        f.volts[0][0] = 1.0;
        f.volts[1][0] = 2.88;
        f.write(0x20, 0x1400, 0xFF00, t); // 0x800ED498: CTM = 01, CTF = fFADC (unchanged)
        let tick = ins_per_fsys * 1024.0 / 18.0;
        let mut sr1 = Vec::new();
        while t < 120e6 * 0.01 {
            t = f.next_event().unwrap();
            if f.advance(t) & 2 != 0 {
                sr1.push(t);
            }
        }
        let n = sr1.len();
        let period = (sr1[n - 1] - sr1[n - 2]) / tick;
        assert!((period - 176.0).abs() < 1e-6, "{period} fFADC cycles");
        assert!((sr1[0] / tick - (176.0 + 42.0)).abs() < 1e-6, "first SR1 after timer + 2 conversions: {}", sr1[0] / tick);
        assert_eq!(f.read(0x40), (1.0f64 / 3.3 * 1024.0).round() as u32);
        assert_eq!(f.read(0x44), (2.88f64 / 3.3 * 1024.0).round() as u32);
        assert_eq!(f.conversions[2] + f.conversions[3], 0);
        assert_eq!(f.read(0x10) & 0x0003_0000, 0x0003_0000, "IRQ0/IRQ1 set");
        f.write(0x14, 0x0003_0000, !0, t);
        assert_eq!(f.read(0x10) & 0x0003_0000, 0, "RIRQ clears");
    }

    /// Cost of the FADC model (diagnostic): ns per simulated second of the DQ200 configuration
    /// (~18k events/s) and the share of real time.
    #[test]
    #[ignore = "diagnostic"]
    fn fadc_cost() {
        let mut f = Fadc::default();
        for (off, v) in [(0x00, 0), (0x0C, 0x8012), (0x1C, 0x0020_0000), (0x30, 5), (0x20, 0x00AF_1445), (0x34, 5), (0x24, 0x9001_0045), (0x18, 2)] {
            f.write(off, v, !0, 0.0);
            f.set_clock(2.0, 0.0);
        }
        let secs = 20.0;
        let t0 = std::time::Instant::now();
        let mut srs = 0u64;
        while let Some(t) = f.next_event().filter(|&t| t < secs * 120e6) {
            srs += f.advance(t).count_ones() as u64;
        }
        let ns = t0.elapsed().as_nanos() as f64 / secs;
        println!("FADC: {} conversions, {srs} SR in {secs} s -> {:.0} ns per simulated s = {:.4} % of real time", f.conversions[0] + f.conversions[1], ns, ns / 1e7);
    }

    /// Table 26-1: single-ended and differential results at gain 1 and 2.
    #[test]
    fn input_stage_matches_um_table_26_1() {
        let mut f = Fadc::default();
        let mut code = |acr: u32, p: f32, n: f32| {
            f.regs[ACR] = acr;
            f.volts[0] = [p, n];
            f.sample(0)
        };
        assert_eq!(code(0b1000, 9.9, 0.0), 768);
        assert_eq!(code(0b1000, 9.9, 3.3), 256);
        assert_eq!(code(0b1001, 0.0, 0.0), 1023);
        assert_eq!(code(0b0100, 0.0, 9.9), 256);
        assert_eq!(code(0b0100, 3.3, 9.9), 768);
        assert_eq!(code(0b0101, 0.0, 9.9), 0);
        assert_eq!(code(0b1100, 1.65, 0.0), 768);
        assert_eq!(code(0b1100, 0.0, 3.3), 0);
        assert_eq!(code(0b1101, 1.65, 0.0), 1023);
    }
}
