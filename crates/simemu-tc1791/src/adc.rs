//! ADC0..ADC2 analog front-end: per-channel input voltages converted by an emulated auto-scan
//! into the result registers the firmware reads.
//!
//! Register usage verified from the firmware's ADC driver (init 0x8010E000, service 0x800A68A0,
//! result readout 0x800A6944):
//! * `+0x090` channel selection mask (auto-scan)        * `+0x098` bit 9 = load event (start)
//! * `+0x100 + 4*ch` CHCTRch, bits 15:12 = result register index
//! * `+0x180 + 4*r` RESRr, the firmware stores the low halfword
//! * `+0x070` bit 17 = auto-scan source event flag (polled), cleared by writing 1 to `+0x074` bit 17
//!
//! UNVERIFIED (no TC1791 user manual available): result width (12-bit right aligned assumed; the
//! firmware writes INPCR0 = 0x709 whose conversion-mode field could not be decoded), the RESR
//! flag layout (VF bit 31, CHNR bits 24:20) and the conversion time.

use crate::memory::Tc1791HardwareState;

pub const ADC_BASES: [u32; 3] = [0xF010_1000, 0xF010_1400, 0xF010_1800];
/// Analog reference voltage (VAREF).
pub const ADC_VAREF: f32 = 5.0;
/// Result resolution in bits (UNVERIFIED, see module docs).
pub const ADC_RESULT_BITS: u32 = 12;
/// Conversion time per channel in nanoseconds (UNVERIFIED).
pub const ADC_CONV_NS_PER_CHANNEL: u64 = 1_000;

#[derive(Default)]
pub struct AdcState {
    /// Input voltage per kernel/channel.
    pub volts: [[f32; 16]; 3],
    /// Instruction count at which the running auto-scan completes.
    pub done_at: [Option<u64>; 3],
    pub evflags: [u32; 3],
    pub scans: [u64; 3],
}

impl AdcState {
    pub fn raw(v: f32) -> u32 {
        let full = (1u32 << ADC_RESULT_BITS) - 1;
        ((v / ADC_VAREF).clamp(0.0, 1.0) * full as f32).round() as u32
    }
}

fn kernel_of(addr: u32) -> Option<(usize, u32)> {
    ADC_BASES
        .iter()
        .position(|&b| (b..b + 0x400).contains(&addr))
        .map(|k| (k, addr - ADC_BASES[k]))
}

impl Tc1791HardwareState {
    #[inline]
    fn now_insns(&self) -> u64 {
        self.total_instructions + self.quantum_executed
    }

    fn adc_complete(&mut self, k: usize) {
        self.adc.done_at[k] = None;
        let base = ADC_BASES[k];
        let sel = self.seg_f000.get(base + 0x90) & 0xFFFF;
        for ch in 0..16u32 {
            if sel & (1 << ch) == 0 {
                continue;
            }
            let resreg = (self.seg_f000.get(base + 0x100 + 4 * ch) >> 12) & 0xF;
            let raw = AdcState::raw(self.adc.volts[k][ch as usize]);
            self.seg_f000.set(base + 0x180 + 4 * resreg, 0x8000_0000 | (ch << 20) | raw);
        }
        self.adc.evflags[k] |= 1 << 17;
        self.adc.scans[k] += 1;
    }

    /// Complete any conversion whose time has come (called at boundaries and on register reads).
    #[inline]
    pub(crate) fn adc_update(&mut self) {
        if self.adc.done_at == [None; 3] {
            return;
        }
        let now = self.now_insns();
        for k in 0..3 {
            if self.adc.done_at[k].is_some_and(|t| now >= t) {
                self.adc_complete(k);
            }
        }
    }

    #[inline]
    pub(crate) fn insns_to_next_adc_event(&self) -> u64 {
        if self.adc.done_at == [None; 3] {
            return u64::MAX;
        }
        let now = self.now_insns();
        self.adc.done_at.iter().flatten().map(|&t| t.saturating_sub(now).max(1)).min().unwrap_or(u64::MAX)
    }

    pub(crate) fn adc_peek(&self, addr: u32) -> Option<u32> {
        let (k, off) = kernel_of(addr)?;
        match off {
            0x70 => Some(self.adc.evflags[k]),
            0x74 => Some(0),
            0x98 => Some(self.seg_f000.get(addr) & !(1 << 9)),
            _ => None,
        }
    }

    pub(crate) fn adc_read(&mut self, addr: u32) -> Option<u32> {
        let (k, off) = kernel_of(addr)?;
        self.adc_update();
        let _ = (k, off);
        self.adc_peek(addr)
    }

    pub(crate) fn adc_write(&mut self, addr: u32, val: u32, mask: u32) -> bool {
        let Some((k, off)) = kernel_of(addr) else { return false };
        self.adc_update();
        match off {
            0x70 | 0x180..=0x1BC => true, // status/results are hardware-owned
            0x74 => {
                self.adc.evflags[k] &= !(val & mask);
                true
            }
            0x98 => {
                let v = (self.seg_f000.get(addr) & !mask) | (val & mask);
                self.seg_f000.set(addr, v & !(1 << 9));
                if val & mask & (1 << 9) != 0 {
                    let n = (self.seg_f000.get(addr - 0x98 + 0x90) & 0xFFFF).count_ones().max(1) as u64;
                    let insns = (n * ADC_CONV_NS_PER_CHANNEL * self.scu.fcpu_hz() / 1_000_000_000).max(1);
                    self.adc.done_at[k] = Some(self.now_insns() + insns);
                    self.stop_requested = true;
                }
                true
            }
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::BoardVariant;
    use crate::Simos18Board;

    #[test]
    fn autoscan_converts_selected_channels() {
        let mut b = Simos18Board::new_variant(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791)
            .unwrap();
        b.set_adc_input_voltage(0, 3, 2.5).unwrap();
        let hw = b.hw_mut();
        let base = ADC_BASES[0];
        hw.seg_f000.set(base + 0x90, 1 << 3);
        hw.seg_f000.set(base + 0x100 + 12, 7 << 12); // ch3 -> RESR7
        hw.adc_write(base + 0x98, 0x301, 0xFFFF_FFFF);
        assert_eq!(hw.adc_read(base + 0x70), Some(0), "conversion takes time");
        hw.total_instructions += 10_000;
        assert_eq!(hw.adc_read(base + 0x70).unwrap() & (1 << 17), 1 << 17);
        let r = hw.seg_f000.get(base + 0x180 + 28);
        assert_eq!(r & 0xFFFF, AdcState::raw(2.5));
        assert_eq!((r >> 20) & 0x1F, 3);
        hw.adc_write(base + 0x74, 1 << 17, 0xFFFF_FFFF);
        assert_eq!(hw.adc_read(base + 0x70), Some(0));
    }
}
