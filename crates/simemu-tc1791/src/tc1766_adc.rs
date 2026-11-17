//! ADC0 of the TC1766 (DQ250 board): the AUDO-NG ADC of the TC1796 User's Manual V2.0, chapter 25
//! (`vault/docs/infineon/tc1796_um_v2.0_2007_07.pdf`, page labels 25-n), register map p.18-66/25-55..
//! (`F010 0400H` = ADC0_CLC .. `F010 05FCH` = ADC0_SRC0).
//!
//! What the DQ250 ASW programs (VERIFIED-T, MMIO trace of 0D9300011R/4319, driver 0x8007EEE0 /
//! ISR 0x8007FBE0 on ADC0_SRC2 SRPN 0x13): CON = 0x10001 (CTC 1), FDR = 0x826D (fractional,
//! STEP 0x26D), CHCON1..15 = 0x0280040A/0x02800400 (12 bit, INP 2, ENCH, LCC 000), CHCON0 = 0x02C0040A
//! (LCC 100: service request on result write), TTC = 0xFFFF, TCON = 0x40050000 (TSEN, TRLD 5) and
//! SCON = 8 (TRS) once per cycle; the ISR reads MSS0, clears the bit by writing 1 and copies CHSTAT0..15
//! (low halfword = left-aligned RESULT) to its table 0xD00050FC (0x800814DC).
//!
//! Modelled (TC1796 UM): conversion request source "Timer" (p.25-8..25-10: TR/TSEN/TRLD, period
//! TRLD·20/fADC, TCRP ← TTC at timer 0, highest channel first, AP.TP set/cleared by the arbiter,
//! clearing AP.TP clears TCRP, MSS1.MSRT), SCON TRS/TRC/QENS/QENC, conversion time
//! tS + (40|48|56)·tBC + 2·tADC with tBC = (CTC+1)/fADC, tS = 4·(STC+2)·tBC (Table 25-6, p.25-33..37),
//! CHSTATn (RESULT left aligned, CRS = timer, CHNR; p.25-63/64), channel request LCC = 100, ENCH, INP,
//! MSS0/MSS1 write-1-to-clear (Table 25-10), SRNP timer node, SR0..SR3 → ADC0_SRC0..3, STAT.BUSY/CHNRCC,
//! TSTAT.TIMER, TCRP. Not modelled: the other request sources (auto-scan, queue, SW0, external event,
//! injection — not used by this firmware), the external multiplexer (EMUX: the input line is the same
//! for every EMUX setting unless the board puts an external multiplexer on it, [`Adc0::emux`]), CHCONn.REF (bits 9:8 select [`Adc0::vref`]), limit checking (LCC 001..011, 101..111; LCCONx = 0 here),
//! SRTEST, arbitration lock (ALB = 0 here), synchronisation of two ADCs, SR4..SR7 (no SRC on the TC1796).

pub const ADC0_BASE: u32 = 0xF010_0400;
/// Analog reference voltage VAREF = 3.3 V: the DQ250 ASW converts its board voltages with
/// `mV = code/4096 · 3300 · gain/512` (0x8007142C, gain table 0xD0006140; VERIFIED-S), i.e. full scale 3.3 V
/// times a divider ratio gain/512 (6.0, 4.0, 2.0, 1.586, 1.0). No schematic: UNVERIFIED by data sheet.
pub const VAREF: f32 = 3.3;
/// SCU_CON.AN7TM pulls AN7 low "via a resistor of max. 900 Ω" (TC1796 UM p.5-60, VERIFIED-DS for the
/// TC1796; same SCU_CON bit 6 read by the DQ250 ASW at 0x80063D48).
pub const AN7_PULLDOWN_OHMS: f32 = 900.0;
/// Source resistance of the board node on AN7 (UNVERIFIED: no schematic; calibration knob).
pub const AN7_SOURCE_OHMS: f32 = 10_000.0;

const CHCON: u32 = 0x010;
const AP: u32 = 0x084;
const TTC: u32 = 0x08C;
const SCON: u32 = 0x098;
const TCON: u32 = 0x114;
const CON: u32 = 0x120;
const CHSTAT: u32 = 0x130;
const TSTAT: u32 = 0x1B0;
const STAT: u32 = 0x1B4;
const TCRP: u32 = 0x1B8;
const MSS0: u32 = 0x1D0;
const MSS1: u32 = 0x1D4;
const SRNP: u32 = 0x1DC;
const AP_TP: u32 = 1 << 6;
const TCON_TR: u32 = 1 << 31;
const TCON_TSEN: u32 = 1 << 30;

/// ADC0 state. Time is board time in ns.
pub struct Adc0 {
    /// Register file, word index = offset / 4 (0x000..0x1FC); modelled registers are kept here too.
    regs: [u32; 0x80],
    /// Analog input voltage per input line AIN0..31 (CHCONn.GRPS selects `AIN[n]` or `AIN[16+n]`, p.25-61).
    pub volts: [f32; 32],
    /// Lines behind an external analog multiplexer: voltage per EMUX code (CHCONn.EMUX drives the
    /// EMUX0/1 lines, p.25-41/25-60); `None` = the line is wired straight to the pin ([`Adc0::volts`]).
    pub emux: [Option<[f32; 4]>; 32],
    /// Reference voltage per CHCONn.REF code (bits 9:8): 0 = VAREF; the board decides what the alternate references
    /// are (DQ250: [`crate::dq250_io::SENSOR_REF_V`] on REF 1/2, HYP).
    pub vref: [f32; 4],
    /// Time at which the running timer reaches 0.
    timer_zero: Option<u64>,
    /// Timer period (ns).
    timer_period: u64,
    /// Channel being converted and the end of its conversion.
    conv: Option<(u8, u64)>,
    pub conversions: u64,
    /// SCU_CON.AN7TM (set by the board before [`Adc0::update`]).
    pub an7_test: bool,
}

impl Default for Adc0 {
    fn default() -> Self {
        let mut regs = [0; 0x80];
        regs[(CON / 4) as usize] = 1; // CON reset value 0000 0001H (p.25-82)
        Self { regs, volts: [0.0; 32], emux: [None; 32], vref: [VAREF; 4], timer_zero: None, timer_period: 0, conv: None, conversions: 0, an7_test: false }
    }
}

impl Adc0 {
    pub fn with_inputs(volts: [f32; 32], emux: [Option<[f32; 4]>; 32]) -> Self {
        Self { volts, emux, ..Self::default() }
    }

    pub fn contains(a: u32) -> bool {
        (ADC0_BASE..ADC0_BASE + 0x1F0).contains(&a) // SRC0..3 (0x1F0..0x1FC) belong to the interrupt unit
    }

    fn r(&self, off: u32) -> u32 {
        self.regs[(off / 4) as usize]
    }

    fn set(&mut self, off: u32, v: u32) {
        self.regs[(off / 4) as usize] = v;
    }

    /// fADC from ADC0_FDR (AUDO fractional divider: normal mode fIN/(1024-STEP), fractional mode
    /// fIN·STEP/1024), `None` when the divider is off.
    fn f_adc(&self, f_sys: u64) -> Option<f64> {
        let fdr = self.r(0x00C);
        let step = (fdr & 0x3FF) as f64;
        match (fdr >> 14) & 3 {
            1 => Some(f_sys as f64 / (1024.0 - step)),
            2 if step > 0.0 => Some(f_sys as f64 * step / 1024.0),
            _ => None,
        }
    }

    fn conv_ns(&self, ch: u8, f_adc: f64) -> u64 {
        let chcon = self.r(CHCON + 4 * ch as u32);
        let t_bc = ((self.r(CON) & 0xFF) + 1) as f64 / f_adc;
        let t_s = 4.0 * ((chcon & 0xFF) + 2) as f64 * t_bc;
        let n_bc = [48.0, 56.0, 40.0, 56.0][((chcon >> 10) & 3) as usize];
        ((t_s + n_bc * t_bc + 2.0 / f_adc) * 1e9).ceil() as u64
    }

    /// Register read (word aligned).
    pub fn read(&self, a: u32, now: u64, f_sys: u64) -> u32 {
        let off = a.wrapping_sub(ADC0_BASE);
        if off >= 0x200 {
            return 0;
        }
        match off {
            TCON => (self.r(TCON) & !TCON_TR) | if self.timer_zero.is_some() { TCON_TR } else { 0 },
            TSTAT => match (self.timer_zero, self.f_adc(f_sys)) {
                (Some(z), Some(f)) => (((z.saturating_sub(now)) as f64 * f / 20e9).ceil() as u32).min(0x3FFF),
                _ => 0,
            },
            STAT => self.conv.map_or(0, |(ch, _)| 1 << 15 | 1 << 8 | ch as u32), // BUSY, CHTSCC = timer
            SCON => 0,
            _ => self.r(off),
        }
    }

    /// Register write with byte-lane mask (`v`, `m` already in lane position).
    pub fn write(&mut self, a: u32, v: u32, m: u32, now: u64, f_sys: u64) {
        let off = a.wrapping_sub(ADC0_BASE);
        if off >= 0x200 {
            return;
        }
        let merged = (self.r(off) & !m) | (v & m);
        let w = v & m;
        match off {
            MSS0 | MSS1 => self.set(off, self.r(off) & !w), // write 1 clears (Table 25-10, SRTEST = 0)
            AP => {
                if merged & AP_TP == 0 {
                    self.set(TCRP, 0); // clearing AP.TP clears TCRP (p.25-10)
                }
                self.set(AP, merged & 0xDB);
            }
            SCON => {
                if w & 0x1 != 0 {
                    self.set(CON, self.r(CON) & !(1 << 15));
                } else if w & 0x2 != 0 {
                    self.set(CON, self.r(CON) | 1 << 15);
                }
                if w & 0x4 != 0 {
                    self.timer_zero = None;
                } else if w & 0x8 != 0 {
                    self.start_timer(now, f_sys);
                }
            }
            TCON => self.set(TCON, merged & !TCON_TR), // TR is rh: only SCON.TRS/TRC and TSEN change it
            CHSTAT..=0x16C | TSTAT | STAT | TCRP => {} // read-only
            _ => self.set(off, merged),
        }
    }

    fn start_timer(&mut self, now: u64, f_sys: u64) {
        let Some(f) = self.f_adc(f_sys) else { return };
        let trld = ((self.r(TCON) >> 16) & 0x3FFF).max(1) as f64;
        self.timer_period = ((trld * 20.0 / f) * 1e9).ceil().max(1.0) as u64;
        self.timer_zero = Some(now + self.timer_period);
    }

    /// Next time something happens (timer zero or end of conversion).
    pub fn next_event(&self) -> Option<u64> {
        match (self.timer_zero, self.conv.map(|c| c.1)) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// Advance to `now`; returns the SR lines (0..7) activated, in order.
    pub fn update(&mut self, now: u64, f_sys: u64) -> Vec<u8> {
        let mut srs = Vec::new();
        let Some(f) = self.f_adc(f_sys) else { return srs };
        loop {
            let t_conv = self.conv.map(|c| c.1).filter(|&t| t <= now);
            let t_timer = self.timer_zero.filter(|&t| t <= now);
            match (t_conv, t_timer) {
                (Some(tc), tt) if tt.is_none_or(|t| tc <= t) => {
                    let (ch, _) = self.conv.take().unwrap();
                    self.finish(ch, &mut srs);
                    self.next_conversion(tc, f);
                }
                (_, Some(t)) => {
                    // timer = 0: TCRP <- TTC, AP.TP, MSS1.MSRT, reload or stop (TSEN) (p.25-8/9)
                    self.set(TCRP, self.r(TCRP) | (self.r(TTC) & 0xFFFF));
                    if self.r(TCRP) != 0 {
                        self.set(AP, self.r(AP) | AP_TP);
                    }
                    self.set(MSS1, self.r(MSS1) | 1);
                    let srnp = self.r(SRNP);
                    if srnp & 1 != 0 {
                        srs.push(((srnp >> 1) & 7) as u8);
                    }
                    self.timer_zero = if self.r(TCON) & TCON_TSEN != 0 { None } else { Some(t + self.timer_period) };
                    if self.conv.is_none() {
                        self.next_conversion(t, f);
                    }
                }
                _ => break,
            }
        }
        srs
    }

    /// Start the conversion of the highest pending timer channel at `t`, if any.
    fn next_conversion(&mut self, t: u64, f: f64) {
        let tcrp = self.r(TCRP) & 0xFFFF;
        if tcrp == 0 || self.r(AP) & AP_TP == 0 {
            self.set(AP, self.r(AP) & !AP_TP);
            return;
        }
        let ch = 31 - tcrp.leading_zeros();
        self.set(TCRP, tcrp & !(1 << ch));
        self.conv = Some((ch as u8, t + self.conv_ns(ch as u8, f)));
    }

    fn finish(&mut self, ch: u8, srs: &mut Vec<u8>) {
        self.conversions += 1;
        let chcon = self.r(CHCON + 4 * ch as u32);
        let line = ch as usize + 16 * ((chcon >> 14) & 1) as usize;
        let mut v = self.emux[line].map_or(self.volts[line], |e| e[((chcon >> 12) & 3) as usize]);
        if line == 7 && self.an7_test {
            v *= AN7_PULLDOWN_OHMS / (AN7_PULLDOWN_OHMS + AN7_SOURCE_OHMS);
        }
        let vref = self.vref[((chcon >> 8) & 3) as usize];
        let ratio = if v.is_finite() && vref.is_finite() && vref > 0.0 {
            (v / vref).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let code12 = (ratio * 4095.0).round() as u32;
        let result = match (chcon >> 10) & 3 {
            0 => (code12 >> 2) << 2, // 10 bit, RESULT[11:2]
            2 => (code12 >> 4) << 4, // 8 bit, RESULT[11:4]
            _ => code12,             // 12 bit
        };
        let st = result | (chcon & 0x3000) << 4 | (chcon & 0x4000) << 4 | 1 << 20 | (ch as u32) << 24;
        self.set(CHSTAT + 4 * ch as u32, st);
        if (chcon >> 20) & 7 == 0b100 {
            self.set(MSS0, self.r(MSS0) | 1 << ch);
            if chcon & 1 << 23 != 0 {
                srs.push(((chcon >> 24) & 7) as u8);
            }
        }
    }
}

/// ADC0 service request line SRn → ADC0_SRCn (SR0..SR3, TC1796 UM p.25-119); SR4..7 have no SRC.
pub fn sr_src(n: u8) -> Option<u32> {
    (n < 4).then(|| ADC0_BASE + 0x1FC - 4 * n as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timer_sequence_converts_all_channels_and_requests_sr2() {
        let mut a = Adc0::default();
        let f = 80_000_000;
        let w = |a: &mut Adc0, off: u32, v: u32| a.write(ADC0_BASE + off, v, u32::MAX, 0, f);
        w(&mut a, 0x00C, 0x826D);
        w(&mut a, CON, 0x10001);
        for ch in 0..16 {
            w(&mut a, CHCON + 4 * ch, if ch == 0 { 0x02C0_040A } else { 0x0280_0400 });
        }
        a.volts[1] = VAREF / 2.0;
        w(&mut a, TCON, 0x4005_0000);
        w(&mut a, TTC, 0xFFFF);
        w(&mut a, SCON, 8);
        assert!(a.read(ADC0_BASE + TCON, 0, f) & TCON_TR != 0);
        let srs = a.update(1_000_000, f);
        assert_eq!(srs, vec![2], "channel 0 (LCC 100, INP 2) is converted last");
        assert_eq!(a.conversions, 16);
        assert_eq!(a.read(ADC0_BASE + MSS0, 0, f), 1);
        assert_eq!(a.read(ADC0_BASE + CHSTAT + 4, 0, f) & 0xFFF, 2048);
        assert_eq!(a.read(ADC0_BASE + CHSTAT + 4, 0, f) >> 24, 1);
        assert_eq!(a.read(ADC0_BASE + AP, 0, f) & AP_TP, 0, "arbiter cleared TP");
        assert_eq!(a.read(ADC0_BASE + TCON, 0, f) & TCON_TR, 0, "TSEN stopped the timer");
        w(&mut a, MSS0, 1);
        assert_eq!(a.read(ADC0_BASE + MSS0, 0, f), 0);
        assert_eq!(sr_src(2), Some(0xF010_05F4));
    }

    #[test]
    fn external_multiplexer_selects_the_line_by_emux() {
        let mut a = Adc0::default();
        let f = 80_000_000;
        a.emux[31] = Some([VAREF / 2.0, 0.0, 0.0, VAREF]);
        let w = |a: &mut Adc0, off: u32, v: u32| a.write(ADC0_BASE + off, v, u32::MAX, 0, f);
        w(&mut a, 0x00C, 0x826D);
        w(&mut a, CON, 0x10001);
        for (emux, code) in [(0u32, 2048), (3, 4095)] {
            w(&mut a, CHCON + 4 * 15, 0x0280_4400 | emux << 12); // GRPS 1 (AIN31), 12 bit
            w(&mut a, TCON, 0x4005_0000);
            w(&mut a, TTC, 0x8000);
            w(&mut a, SCON, 8);
            a.update(a.next_event().unwrap() + 1_000_000, f);
            assert_eq!(a.read(ADC0_BASE + CHSTAT + 4 * 15, 0, f) & 0xFFF, code, "EMUX {emux}");
        }
    }

    #[test]
    fn oob_mmio_and_non_finite_voltages_do_not_panic() {
        let mut a = Adc0::default();
        let f = 80_000_000;
        assert_eq!(a.read(ADC0_BASE + 0x400, 0, f), 0);
        a.write(ADC0_BASE + 0x400, 0xDEAD_BEEF, u32::MAX, 0, f);
        a.volts[0] = f32::NAN;
        a.vref[1] = 0.0;
        a.vref[2] = f32::NAN;
        a.write(ADC0_BASE + 0x00C, 0x826D, u32::MAX, 0, f);
        a.write(ADC0_BASE + CON, 0x10001, u32::MAX, 0, f);
        a.write(ADC0_BASE + CHCON, 0x0280_0500, u32::MAX, 0, f); // REF = 1 (vref = 0.0)
        a.write(ADC0_BASE + TCON, 0x4005_0000, u32::MAX, 0, f);
        a.write(ADC0_BASE + TTC, 0xFFFF, u32::MAX, 0, f);
        a.write(ADC0_BASE + SCON, 8, u32::MAX, 0, f);
        a.update(1_000_000, f);
        assert_eq!(a.read(ADC0_BASE + CHSTAT, 0, f) & 0xFFF, 0);
    }
}
