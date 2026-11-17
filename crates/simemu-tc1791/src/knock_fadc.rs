//! FADC (fast A/D converter, base 0xF010_0400) and the knock-sensor input it samples.
//!
//! SIMOS18.1 acquires knock "by soft" (FR KNCK 53.2, p.22133–22140): the raw sensor signal is sampled inside an
//! angle window per cylinder and band-pass filtered + integrated by the CPU (FPU). The hardware path, as the
//! unmodified SC800S50 programs it (docs/SIMOS18_GPTA_IO_RE.md §12.7; VERIFIED-T on the native core unless noted):
//!
//! * One knock sensor on **FADC channel 1**, single-ended FAIN1P − VFAREF/2 (ACR1 = 0x5: GAIN ×2, ENP = 1, ENN = 0;
//!   TC1796 UM p.26-5/26-6: 0 V → code 0, 3.3 V → 1023).
//! * **Window gate**: CFGR1 = 0x30631887: GSEL 7, GM = 10 (channel timer and conversions only while gate GS7 is
//!   high), CTM = 10, CTF = 1 (fCT = fFADC = 90 MHz, FDR 0x43FF), CTREL 99, INP 3. GS7 = GPTA0 OUT26 ← G0.GTC24
//!   (OMCR decode; GS4..7 = OUT1/9/18/26 per TC1796 UM Fig. 26-16 — HYP for the TC1791). G0.GTC24 is high in the
//!   pre-window (−9.7…+8.3° ATDC) and the main window (+10.1…+43.1° ATDC = `ip_knkwb`/`ip_knkwd`) of every segment.
//! * **Data reduction** (UM 26.1.6): FCR0 = 0x3512 (INSEL ch 1, ADDL 2, MAVL 1): every 3 conversions FRR0 = CRR0 +
//!   IRR10; FCR1 = 0xB202 (INSEL = FRR0, ADDL 2, IEN, INP 3): every 3 FRR0 values FRR1 = their sum → a 12-tap FIR
//!   `[1,1,1,2,2,2,2,2,2,1,1,1]` (gain 18) and 9:1 decimation; each FRR1 raises service request SR3.
//! * **DMA channel 11** (CHCR 0x00302000: PRSEL 2, continuous, 16 bit; SADR = FRR1 0xF0100494, DADR = window buffer,
//!   destination +2) moves every FRR1 to RAM; at TCOUNT = 0 it interrupts (DMA_SRC6). SR3 → ch 11 / PRSEL 2 is HYP
//!   (no TC1791 DMA request table; ch 11 is the only channel whose source is FRR1).
//! * The firmware reads the buffer ~18 µs after the gate falls (window-end callback 0x800AC300 → count from DADR),
//!   so the conversions are produced lazily up to "now" at every engine-I/O sync and FADC/DMA access — only while
//!   the gate is open, exactly at the converter's own conversion instants.
//!
//! Channel-timer divider: UM p.26-11/26-44: CTREL + 2 when fCLC/fCT = 1 (here 90/90 MHz) → 891 kHz conversions,
//! 99.0 kHz output; the firmware's filter tables assume 100 kHz (CTREL + 1). The UM rule is applied (UNVERIFIED
//! for the TC1791: a 1 % shift of the firmware's band-pass centres).
//!
//! Analog front end of the board (UNVERIFIED knobs: no schematic): the piezo sensor is AC-coupled onto a bias of
//! VFAREF/2 (the firmware initialises its filter input to 511.5 codes = mid-scale, KNCK p.22137) with gain
//! [`KNOCK_FE_GAIN`]. The sensor signal arrives from the plant as damped sinusoids ([`PinAtom`]).

use crate::memory::Tc1791HardwareState;
use std::collections::VecDeque;

pub const FADC_BASE: u32 = 0xF010_0400;
/// Register offsets (TC1796 UM Table 26-8).
const CRSR: u32 = 0x10;
const FMR: u32 = 0x14;
const GCR: u32 = 0x1C;
const CFGR0: u32 = 0x20;
const ACR0: u32 = 0x30;
const RCH0: u32 = 0x40;
const FCR0: u32 = 0x60;
const FDR: u32 = 0x0C;
/// FADC reference voltage.
pub const VFAREF: f64 = 3.3;
/// Gating source line of the knock window and the GPTA cell that drives it (kernel 0 = GPTA0, GTC24).
pub const GATE_LINE: usize = 7;
pub const GATE_CELL: (u8, u8) = (0, 24);
/// FADC service-request line → (DMA channel, CHCR.PRSEL). HYP (see module docs).
const SR_TO_DMA: [Option<(usize, u32)>; 4] = [None, None, None, Some((11, 2))];
/// Knock input stage of the board: gain from sensor terminals to FAIN1P, bias = VFAREF/2. UNVERIFIED.
pub const KNOCK_FE_GAIN: f64 = 1.0;
/// Channel timer preload while stopped (UM p.26-11).
const TIMER_PRELOAD: f64 = 4.0;
const MAX_ATOMS: usize = 1024;

/// One damped sinusoid at an input pin, board time in seconds: v(t) = v·e^(−α(t−t0))·sin(ω(t−t0) + φ).
#[derive(Debug, Clone, Copy, Default)]
pub struct PinAtom {
    pub t0: f64,
    pub t_end: f64,
    pub w: f64,
    pub alpha: f64,
    pub phi: f64,
    pub v: f64,
}

/// Signal at one FADC input pin: bias + gain·(sum of atoms). Bounded ring (allocation only while it grows).
#[derive(Debug, Clone)]
pub struct PinSignal {
    atoms: VecDeque<PinAtom>,
    pub bias_v: f64,
    pub gain: f64,
}

impl Default for PinSignal {
    fn default() -> Self {
        Self { atoms: VecDeque::with_capacity(MAX_ATOMS), bias_v: VFAREF / 2.0, gain: KNOCK_FE_GAIN }
    }
}

impl PinSignal {
    pub fn push(&mut self, a: PinAtom) {
        if !(a.t0.is_finite() && a.t_end.is_finite() && a.w.is_finite() && a.alpha.is_finite() && a.phi.is_finite() && a.v.is_finite() && a.alpha >= 0.0) {
            return;
        }
        if self.atoms.len() == MAX_ATOMS {
            self.atoms.pop_front();
        }
        self.atoms.push_back(PinAtom { v: a.v * self.gain, ..a });
    }

    /// Drops atoms that ended before `t` (they cannot contribute to later conversions).
    pub fn prune(&mut self, t: f64) {
        self.atoms.retain(|a| a.t_end >= t);
    }

    pub fn len(&self) -> usize {
        self.atoms.len()
    }

    pub fn is_empty(&self) -> bool {
        self.atoms.is_empty()
    }

    /// Voltage at `t` (direct evaluation).
    pub fn eval(&self, t: f64) -> f64 {
        self.bias_v
            + self.atoms.iter().filter(|a| t >= a.t0 && t < a.t_end).map(|a| a.v * (-a.alpha * (t - a.t0)).exp() * (a.w * (t - a.t0) + a.phi).sin()).sum::<f64>()
    }

    /// `out[k]` = voltage at `t_first + k·ts` (one complex rotation per live atom and sample).
    pub fn fill(&self, t_first: f64, ts: f64, out: &mut [f64]) {
        out.iter_mut().for_each(|x| *x = self.bias_v);
        let n = out.len();
        if n == 0 || ts <= 0.0 {
            return;
        }
        let t_last = t_first + (n - 1) as f64 * ts;
        for a in &self.atoms {
            if a.t0 > t_last || a.t_end < t_first {
                continue;
            }
            let i0 = if a.t0 > t_first { ((a.t0 - t_first) / ts).ceil() as usize } else { 0 };
            if i0 >= n {
                continue;
            }
            let i_end = (((a.t_end - t_first) / ts).floor() as usize).saturating_add(1).min(n);
            if i0 >= i_end {
                continue;
            }
            let x = t_first + i0 as f64 * ts - a.t0;
            let m = a.v * (-a.alpha * x).exp();
            if i_end - i0 == 1 {
                out[i0] += m * (a.w * x + a.phi).sin();
                continue;
            }
            let (s, c) = (a.w * x + a.phi).sin_cos();
            let (mut re, mut im) = (c * m, s * m);
            let d = (-a.alpha * ts).exp();
            let (ds, dc) = (a.w * ts).sin_cos();
            let (rr, ri) = (dc * d, ds * d);
            for o in out[i0..i_end].iter_mut() {
                *o += im;
                (re, im) = (re * rr - im * ri, re * ri + im * rr);
            }
        }
    }
}

/// Filter block state (UM 26.1.6).
#[derive(Debug, Clone, Copy, Default)]
struct Filter {
    crr: u32,
    /// Inputs added into CRR since the last intermediate result.
    count: u32,
    irr: [u32; 3],
    frr: u32,
}

impl Filter {
    /// One filter input; returns the new final result when an intermediate result completes.
    fn feed(&mut self, v: u32, fcr: u32) -> Option<u32> {
        let addl = fcr & 7;
        let mavl = ((fcr >> 4) & 3) as usize;
        self.crr += v;
        self.count += 1;
        if self.count <= addl {
            return None;
        }
        self.frr = self.crr + self.irr[..mavl].iter().sum::<u32>();
        self.irr = [self.crr, self.irr[0], self.irr[1]];
        self.crr = 0;
        self.count = 0;
        Some(self.frr)
    }
}

/// Dynamic state of the FADC (configuration lives in the plain register file).
#[derive(Debug, Clone, Default)]
pub struct Fadc {
    filt: [Filter; 2],
    rch: [u32; 4],
    /// Gating line levels and their pending transitions (board ns).
    gate: [bool; 8],
    pub(crate) gate_q: VecDeque<(u64, usize, bool)>,
    /// Board time (s) up to which conversions have been produced, and the next conversion start per channel while
    /// its timer runs.
    synced_s: f64,
    pub(crate) next_conv: [Option<f64>; 4],
    /// Analog inputs FAIN0P..FAIN3P.
    pub inputs: [PinSignal; 4],
    scratch: Vec<f64>,
    busy: bool,
    pub conversions: u64,
    pub filter_outputs: u64,
}

impl Fadc {
    /// GPTA output edge on gating line `line` at board time `t_ns`.
    pub fn gate_edge(&mut self, line: usize, t_ns: u64, level: bool) {
        if line < self.gate.len() {
            self.gate_q.push_back((t_ns, line, level));
        }
    }

    /// SoC reset: kernel state cleared; inputs (board wiring) stay.
    pub fn soc_reset(&mut self) {
        let inputs = std::mem::take(&mut self.inputs);
        *self = Self { inputs, ..Default::default() };
    }
}

impl Tc1791HardwareState {
    fn fadc_reg(&self, off: u32) -> u32 {
        self.seg_f000.get(FADC_BASE + off)
    }

    /// Converter input clock fFADC (FDR normal divider mode; other modes → fFPI).
    fn ffadc_hz(&self) -> f64 {
        let fdr = self.fadc_reg(FDR);
        let fpi = self.ffpi_hz() as f64;
        match (fdr >> 14) & 3 {
            1 => fpi / (1024 - (fdr & 0x3FF)) as f64,
            2 => fpi * (fdr & 0x3FF) as f64 / 1024.0,
            _ => fpi,
        }
    }

    /// Conversion period of channel `x`'s timer, s (`None`: timer off).
    fn fadc_period_s(&self, x: usize) -> Option<f64> {
        let cfgr = self.fadc_reg(CFGR0 + 4 * x as u32);
        let (ctf, ctrel) = ((cfgr >> 12) & 7, ((cfgr >> 16) & 0xFF) as f64);
        if ctf == 0 || ctf == 7 || ctrel == 0.0 || (cfgr >> 10) & 3 == 0 {
            return None;
        }
        let div = [1.0, 1.0, 4.0, 16.0, 64.0, 256.0, 1024.0][ctf as usize];
        let fct = self.ffadc_hz() / div;
        if !(fct.is_finite() && fct > 0.0) {
            return None;
        }
        // fCLC = fSYS (= fFPI here) and fCLC/fCT = 1 → CTREL + 2, else CTREL + 1 (UM p.26-44).
        let extra = if (self.ffpi_hz() as f64 - fct).abs() < 1.0 { 2.0 } else { 1.0 };
        Some((ctrel + extra) / fct)
    }

    /// Gating condition of channel `x` (GM).
    fn fadc_enabled(&self, x: usize) -> bool {
        let cfgr = self.fadc_reg(CFGR0 + 4 * x as u32);
        let anon = self.fadc_reg(GCR) & (1 << 21) != 0;
        anon && match (cfgr >> 6) & 3 {
            1 => true,
            2 => self.fadc.gate[(cfgr & 7) as usize],
            3 => !self.fadc.gate[(cfgr & 7) as usize],
            _ => false,
        }
    }

    /// Conversion result of channel `x` for pin voltage `v` (ACRx input configuration and gain, UM Table 26-1).
    fn fadc_code(&self, x: usize, v: f64) -> u32 {
        let gcr = self.fadc_reg(GCR);
        if (gcr >> 24) & 3 == 1 && ((gcr >> 26) & 3) as usize == x {
            return 0x200; // offset calibration: the channel amplifier inputs are shorted (mid-scale ± own offset)
        }
        let acr = self.fadc_reg(ACR0 + 4 * x as u32);
        let gain = [1.0, 2.0, 4.0, 8.0][(acr & 3) as usize];
        let half = VFAREF / 2.0;
        let diff = match (acr >> 2) & 3 {
            1 => v - half, // FAINxP − VFAREF/2
            2 => half - v, // VFAREF/2 − FAINxN (only P inputs are wired in this model)
            3 => v - half, // FAINxP − FAINxN with FAINxN at the bias
            _ => 0.0,
        };
        (511.5 + diff * gain * 1023.0 / (2.0 * VFAREF)).round().clamp(0.0, 1023.0) as u32
    }

    /// One conversion result of channel `x` into RCHx and the filter blocks; DMA requests of finished filter outputs.
    #[inline]
    fn fadc_result_with_fcr(&mut self, x: usize, code: u32, fcr: [u32; 2]) {
        let f = &mut self.fadc;
        f.rch[x] = code;
        f.conversions += 1;
        let takes = |fcr: u32| matches!((fcr >> 8) & 7, 1) || (fcr >> 8) & 7 == 4 + x as u32;
        let mut done = [None, None];
        if takes(fcr[0]) {
            done[0] = f.filt[0].feed(code, fcr[0]);
        }
        let in1 = match (fcr[1] >> 8) & 7 {
            2 => done[0],
            _ if takes(fcr[1]) => Some(code),
            _ => None,
        };
        if let Some(v) = in1 {
            done[1] = f.filt[1].feed(v, fcr[1]);
        }
        f.filter_outputs += (done[0].is_some() as u64) + (done[1].is_some() as u64);
        for n in 0..2 {
            if done[n].is_some() && fcr[n] & 0x8000 != 0 {
                if let Some((ch, line)) = SR_TO_DMA[((fcr[n] >> 12) & 3) as usize] {
                    self.spi.dma.hw_request(ch, line);
                    self.dma_run();
                }
            }
        }
    }

    fn fadc_result(&mut self, x: usize, code: u32) {
        let fcr = [self.seg_f000.get(FADC_BASE + FCR0), self.seg_f000.get(FADC_BASE + FCR0 + 0x20)];
        self.fadc_result_with_fcr(x, code, fcr);
    }

    /// Produces every conversion due up to now (lazily, only while a channel is gated on).
    pub(crate) fn knock_fadc_update(&mut self) {
        if self.fadc.busy || self.variant != crate::memory::BoardVariant::Simos18Tc1791 {
            return;
        }
        let now = self.now_ns() as f64 * 1e-9;
        let has_gate = self.fadc.gate_q.front().is_some_and(|&(t, _, _)| (t as f64) * 1e-9 <= now);
        if !has_gate && self.fadc.next_conv.iter().all(|c| c.is_none_or(|t| t >= now)) {
            self.fadc.synced_s = now;
            return;
        }
        self.fadc.busy = true;
        let mut had_gate_close = false;
        loop {
            // next gate transition (or now)
            let (t_stop, ev) = match self.fadc.gate_q.front() {
                Some(&(t, line, level)) if (t as f64) * 1e-9 <= now => ((t as f64) * 1e-9, Some((line, level))),
                _ => (now, None),
            };
            for x in 0..4 {
                if self.fadc.next_conv[x].is_some() {
                    self.fadc_convert_until(x, t_stop);
                }
            }
            let Some((line, level)) = ev else { break };
            self.fadc.gate_q.pop_front();
            if line < self.fadc.gate.len() {
                self.fadc.gate[line] = level;
            }
            if !level {
                had_gate_close = true;
            }
            for x in 0..4 {
                self.fadc_arm(x, t_stop);
            }
        }
        self.fadc.synced_s = now;
        if had_gate_close {
            for p in self.fadc.inputs.iter_mut() {
                p.prune(now - 0.01);
            }
        }
        self.fadc.busy = false;
    }

    /// (Re)start or stop channel `x`'s timer at `t` according to its gating condition.
    fn fadc_arm(&mut self, x: usize, t: f64) {
        let on = self.fadc_enabled(x);
        match (on, self.fadc.next_conv[x]) {
            (true, None) => {
                if self.fadc_period_s(x).is_some() {
                    // the stopped timer holds 04H and counts down at fCT before the first trigger
                    let cfgr = self.fadc_reg(CFGR0 + 4 * x as u32);
                    let ct = [1.0, 1.0, 4.0, 16.0, 64.0, 256.0, 1024.0][((cfgr >> 12) & 7).min(6) as usize] / self.ffadc_hz();
                    self.fadc.next_conv[x] = Some(t + TIMER_PRELOAD * ct);
                }
            }
            (false, Some(_)) => self.fadc.next_conv[x] = None,
            _ => {}
        }
    }

    /// Conversions of channel `x` with start time < `t_stop`.
    fn fadc_convert_until(&mut self, x: usize, t_stop: f64) {
        let (Some(t0), Some(p)) = (self.fadc.next_conv[x], self.fadc_period_s(x)) else { return };
        if t0 >= t_stop {
            return;
        }
        let total_n = ((t_stop - t0) / p).ceil().max(1.0) as usize;
        let skip = total_n.saturating_sub(4096);
        let t_start = t0 + skip as f64 * p;
        let n = total_n - skip;
        let mut buf = std::mem::take(&mut self.fadc.scratch);
        buf.resize(n, 0.0);
        self.fadc.inputs[x].fill(t_start, p, &mut buf);
        let fcr = [self.seg_f000.get(FADC_BASE + FCR0), self.seg_f000.get(FADC_BASE + FCR0 + 0x20)];
        for &v in &buf {
            let code = self.fadc_code(x, v);
            self.fadc_result_with_fcr(x, code, fcr);
        }
        self.fadc.scratch = buf;
        self.fadc.next_conv[x] = Some(t0 + total_n as f64 * p);
    }

    /// Side-effect-free FADC register peek.
    pub(crate) fn fadc_peek(&self, a: u32) -> Option<u32> {
        let off = a.wrapping_sub(FADC_BASE);
        let f = &self.fadc;
        match off {
            0x40..=0x4C => Some(f.rch[((off - RCH0) / 4) as usize]),
            0x64 | 0x84 => Some(f.filt[((off - 0x64) / 0x20) as usize].crr),
            0x68 | 0x88 => Some(f.filt[((off - 0x68) / 0x20) as usize].irr[0]),
            0x6C => Some(f.filt[0].irr[1]),
            0x70 => Some(f.filt[0].irr[2]),
            0x74 | 0x94 => Some(f.filt[((off - 0x74) / 0x20) as usize].frr),
            0x60..=0x74 | 0x80..=0x94 => None,
            FMR => Some(0),
            _ => None,
        }
    }

    /// FADC register read (`None` = plain register file).
    pub(crate) fn fadc_read(&mut self, a: u32) -> Option<u32> {
        let off = a.wrapping_sub(FADC_BASE);
        match off {
            0x40..=0x4C | 0x64..=0x74 | 0x84..=0x94 => {
                self.knock_fadc_update();
                self.fadc_peek(a)
            }
            FMR => Some(0),
            _ => None,
        }
    }

    /// FADC register write (`false` = plain register file stores it).
    pub(crate) fn fadc_write(&mut self, a: u32, v: u32, m: u32) -> bool {
        let off = a.wrapping_sub(FADC_BASE);
        if !(CRSR..0xF0).contains(&off) {
            return false;
        }
        self.knock_fadc_update();
        let now = self.now_ns() as f64 * 1e-9;
        let merged = (self.seg_f000.get(a) & !m) | (v & m);
        match off {
            GCR => {
                // RCD (8) and RSTFn (9 + n) are write-1 triggers that read 0.
                for (n, filt) in self.fadc.filt.iter_mut().enumerate() {
                    if v & m & (1 << (9 + n)) != 0 {
                        *filt = Filter::default();
                    }
                }
                self.seg_f000.set(a, merged & !0x700);
            }
            FMR => {
                for x in 0..4 {
                    if v & m & (1 << (8 + x)) != 0 {
                        // software-requested conversion: completes within one conversion time (≈ 233 ns)
                        let vin = self.fadc.inputs[x].eval(now);
                        let code = self.fadc_code(x, vin);
                        self.fadc_result(x, code);
                    }
                }
            }
            0x40..=0x4C => {
                if self.fadc_reg(GCR) & (1 << 19) != 0 {
                    self.fadc.rch[((off - RCH0) / 4) as usize] = merged & 0x3FF;
                }
            }
            0x64..=0x74 | 0x84..=0x94 => {}
            _ => self.seg_f000.set(a, merged),
        }
        for x in 0..4 {
            if !self.fadc_enabled(x) {
                self.fadc.next_conv[x] = None;
            } else {
                self.fadc_arm(x, now);
            }
        }
        true
    }
}

impl crate::Simos18Board {
    /// Knock-sensor signal at FADC input `ch` (FAINchP): damped sinusoids in board time, sensor-terminal volts (the
    /// board's input stage applies its gain and bias).
    pub fn feed_fadc_input(&mut self, ch: usize, atoms: impl IntoIterator<Item = PinAtom>) {
        let now = self.hw().now_ns() as f64 * 1e-9;
        let Some(p) = self.hw_mut().fadc.inputs.get_mut(ch) else {
            return;
        };
        p.prune(now - 0.01);
        atoms.into_iter().for_each(|a| p.push(a));
    }

    /// FADC state (diagnostics: conversion and filter-output counters).
    pub fn fadc(&self) -> &Fadc {
        &self.hw().fadc
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::BoardVariant;
    use crate::Simos18Board;

    /// The knock configuration exactly as SC800S50 writes it: a 7 kHz sine of 100 mV at FAIN1P is converted only
    /// while gate GS7 is high, reduced 9:1 by the two filter blocks (12-tap FIR, gain 18) and every FRR1 is moved by
    /// DMA channel 11 into RAM; the samples carry the bias (18 × mid-scale) and the sine at the FIR's gain.
    #[test]
    fn gated_conversions_filters_and_dma_as_programmed_for_knock() {
        let mut b = Simos18Board::new_variant(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791).unwrap();
        let hw = b.hw_mut();
        let w = |hw: &mut Tc1791HardwareState, off: u32, v: u32| assert!(hw.fadc_write(FADC_BASE + off, v, 0xFFFF_FFFF) || off == FDR);
        hw.seg_f000.set(FADC_BASE + FDR, 0x43FF);
        // DMA ch 11 as the knock driver programs it: FRR1 → 0xD0010BF8, 16 bit, destination +2, TREL 400
        let ch = 0x80 + 0x20 * 11;
        hw.spi.dma.write(ch + 0x04, 0x0030_2000 | 400, 0xFFFF_FFFF); // CHCR
        hw.spi.dma.write(ch + 0x0C, 0xF080, 0xFFFF_FFFF); // ADRCR
        hw.spi.dma.write(ch + 0x10, FADC_BASE + 0x94, 0xFFFF_FFFF); // SADR
        hw.spi.dma.write(ch + 0x14, 0xD001_0BF8, 0xFFFF_FFFF); // DADR
        hw.spi.dma.write(0x1C, 1 << 11, 0xFFFF_FFFF); // HTREQ
        w(hw, GCR, 0x0028_0600);
        w(hw, ACR0 + 4, 0x5);
        w(hw, FCR0, 0x3512);
        w(hw, FCR0 + 0x20, 0xB202);
        w(hw, CFGR0 + 4, 0x3063_1887);
        let (f, amp) = (7_000.0, 0.1);
        hw.fadc.inputs[1].push(PinAtom { t0: 0.0, t_end: 1.0, w: 2.0 * std::f64::consts::PI * f, alpha: 0.0, phi: 0.0, v: amp });
        // gate closed: nothing converts
        hw.sim_time_s = 1e-3;
        hw.knock_fadc_update();
        assert_eq!(hw.fadc.conversions, 0);
        // gate open for 1 ms (from 1.0 ms to 2.0 ms)
        hw.fadc.gate_edge(GATE_LINE, 1_000_000, true);
        hw.fadc.gate_edge(GATE_LINE, 2_000_000, false);
        hw.sim_time_s = 3e-3;
        hw.knock_fadc_update();
        // CTREL 99 + 2 at fCT = fCLC (fFPI of this unbooted board)
        let period = hw.fadc_period_s(1).unwrap();
        assert!((period * hw.ffpi_hz() as f64 - 101.0).abs() < 1e-6);
        let expect = (1e-3 / period).ceil() as u64;
        assert!((hw.fadc.conversions as i64 - expect as i64).abs() <= 1, "{} conversions vs {expect}", hw.fadc.conversions);
        let samples = hw.fadc.conversions / 9;
        let dadr = hw.spi.dma.read(ch + 0x14).unwrap();
        assert_eq!((dadr - 0xD001_0BF8) / 2, samples as u32, "one DMA move per FRR1");
        let buf: Vec<f64> = (0..samples as usize).map(|k| u16::from_le_bytes([hw.dspr[0x10BF8 + 2 * k], hw.dspr[0x10BF9 + 2 * k]]) as f64).collect();
        // after the filters have filled (2 samples), mean = 18 × 511.5, amplitude = 18 × 100 mV · 310 codes/V · |H_FIR|
        let tail = &buf[2..];
        let mean = tail.iter().sum::<f64>() / tail.len() as f64;
        let pk = tail.iter().map(|x| (x - mean).abs()).fold(0.0, f64::max);
        assert!((mean / (18.0 * 511.5) - 1.0).abs() < 0.01, "mean {mean}");
        let codes_per_v = 1023.0 / VFAREF;
        let taps = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0];
        let wt = 2.0 * std::f64::consts::PI * f * period;
        let (re, im) = taps.iter().enumerate().fold((0.0, 0.0), |(r, i), (k, &c)| (r + c * (wt * k as f64).cos(), i + c * (wt * k as f64).sin()));
        let h = re.hypot(im);
        assert!((pk / (h * amp * codes_per_v) - 1.0).abs() < 0.08, "peak {pk} vs {}", h * amp * codes_per_v);
        // after the gate closed nothing more is produced
        let n = hw.fadc.conversions;
        hw.sim_time_s = 5e-3;
        hw.knock_fadc_update();
        assert_eq!(hw.fadc.conversions, n);
    }

    /// Cost of the conversion engine (`cargo test --release -p simemu-tc1791 fadc_cost -- --ignored --nocapture`): one
    /// second of gated conversions at the knock rate with 40 live vibration atoms (≈ a 6000 rpm block at full load).
    #[test]
    #[ignore]
    fn fadc_cost() {
        let mut p = PinSignal::default();
        for k in 0..40 {
            let f = 5_000.0 + 400.0 * k as f64;
            p.push(PinAtom { t0: 0.0, t_end: 2.0, w: 2.0 * std::f64::consts::PI * f, alpha: 300.0, phi: 0.1 * k as f64, v: 0.01 });
        }
        let ts = 101.0 / 90e6;
        let n = (0.3 / ts) as usize; // ≈ 30 % of the time inside a window
        let mut buf = vec![0.0; 1024];
        let t0 = std::time::Instant::now();
        let mut acc = 0.0;
        for c in 0..n / 1024 {
            p.fill(c as f64 * 1024.0 * ts, ts, &mut buf);
            acc += buf[17];
        }
        let dt = t0.elapsed().as_secs_f64();
        eprintln!("{} conversions × 40 atoms in {:.2} ms → {:.2} % of real time ({acc:.3})", n, dt * 1e3, dt * 100.0);
    }

    /// Offset calibration (GCR.CALMODE = 01 for the channel) answers mid-scale, as the firmware's boot loop expects.
    #[test]
    fn offset_calibration_reads_mid_scale() {
        let mut b = Simos18Board::new_variant(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791).unwrap();
        let hw = b.hw_mut();
        hw.fadc_write(FADC_BASE + GCR, 0x0528_0000, 0xFFFF_FFFF);
        hw.fadc_write(FADC_BASE + FMR, 0x200, 0xFFFF_FFFF);
        assert_eq!(hw.fadc_read(FADC_BASE + RCH0 + 4), Some(0x200));
    }

    #[test]
    fn fdr_step_zero_disables_divider_and_oob_setters_do_not_panic() {
        let mut b = Simos18Board::new_variant(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791).unwrap();
        // OOB setters must be safe no-ops
        b.feed_fadc_input(99, [PinAtom { t0: 0.0, t_end: 1.0, w: 1000.0, alpha: 10.0, phi: 0.0, v: 0.1 }]);
        let hw = b.hw_mut();
        hw.fadc.gate_edge(99, 1_000, true);
        // Fractional divider DM = 2 (0x8000) with STEP = 0 -> fFADC = 0 -> fct = 0 (must not divide by zero!)
        hw.seg_f000.set(FADC_BASE + FDR, 0x8000);
        hw.fadc_write(FADC_BASE + GCR, 0x0028_0600, u32::MAX);
        hw.fadc_write(FADC_BASE + CFGR0 + 4, 0x3063_1887, u32::MAX);
        assert_eq!(hw.fadc_period_s(1), None);
        hw.fadc.gate_edge(GATE_LINE, 1_000_000, true);
        hw.sim_time_s = 0.01;
        hw.knock_fadc_update();
        assert_eq!(hw.fadc.conversions, 0);
    }
}
