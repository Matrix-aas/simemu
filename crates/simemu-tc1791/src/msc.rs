//! TC1791 Micro Second Channel MSC0/MSC1: serial downstream (data + command frames) to the
//! ATIC140 power stages and the upstream receiver (UD0..3) for their replies.
//!
//! No TC1791 user manual is available; offsets/bits follow the AUDO MSC kernel as the SC800S50
//! firmware programs it (dynamic trace, MSC1 at +0x100):
//!
//! | off  | reg   | evidence |
//! |------|-------|----------|
//! | 0x0C | FDR   | 0x80110748 writes 0x43FE (normal divider, f_MSC = f_FPI / 2) |
//! | 0x10 | USR   | 0x8011090C writes 0x0C: `UFT [0] = 0` (12-bit frames), `URR [3:1] = 6` |
//! | 0x14 | DSC   | 0x8011081A 0x001A2A80: `NBC [21:16] = 26` command bits, `NDBH [12:8] = 10`, `NDBL [7:3] = 16`; `TM [0]` set later (data repetition); `CP [1]` polled at 0x800ADCF8 |
//! | 0x1C | DD    | per-channel software outputs (0x800A7ADC, pin ids 0x1xxx / 0x2xxx) |
//! | 0x20 | DC    | command frames (0x800ADCF4) |
//! | 0x24 / 0x28 | DSDSL / DSDSH | data source per data bit (0x8011078A / 0x801107AA) |
//! | 0x30 | UD0   | DMA source of the upstream buffer (ch10 SADR 0xF0000830, ch12 0xF0000930); PERR = bit 22 is checked at 0x800A8014 |
//! | 0x40 | ICR   | 0x80110904 writes 0x6000 (receive-data request enabled) |
//! | 0x44 / 0x48 / 0x4C | ISR / ISC / OCR | stored |
//!
//! Upstream frames go to UD0 (label 0) and request the DMA: MSC0 -> channel 10 (PRSEL 6), MSC1 ->
//! channel 12 (PRSEL 14), inferred from the firmware's channel programming (0x80112138,
//! 0x800A9342) like the SSC routing in [`crate::dma`]. UD format: `DATA [7:0]`, `V [16]`, `P [17]`
//! (even parity of DATA), `PERR [22]`.
//!
//! Not modelled: emergency stop, selection bits/ENSEL phases, data-frame timing (data frames are
//! evaluated when the outputs are read), upstream parity errors, labels 1..3, interrupt routing
//! of ISR events (the firmware uses DMA only). UNVERIFIED: the downstream bit time (2 / f_MSC),
//! the URR divider table (4 << (URR - 1)), the DSDSL/DSDSH source encoding (00 = DD, else ALTIN).

use std::collections::VecDeque;

pub const MSC_BASE: u32 = 0xF000_0800;
pub const MSC_STRIDE: u32 = 0x100;
pub const MSC_COUNT: usize = 2;
/// (DMA channel, request line) of the upstream receive request of MSC0 / MSC1.
pub const MSC_DMA: [(usize, u32); MSC_COUNT] = [(10, 6), (12, 14)];

pub const CLC: u32 = 0x00;
pub const ID: u32 = 0x08;
pub const FDR: u32 = 0x0C;
pub const USR: u32 = 0x10;
pub const DSC: u32 = 0x14;
pub const DSS: u32 = 0x18;
pub const DD: u32 = 0x1C;
pub const DC: u32 = 0x20;
pub const DSDSL: u32 = 0x24;
pub const DSDSH: u32 = 0x28;
pub const ESR: u32 = 0x2C;
pub const UD0: u32 = 0x30;
pub const ICR: u32 = 0x40;
pub const ISR: u32 = 0x44;
pub const ISC: u32 = 0x48;
pub const OCR: u32 = 0x4C;

const DSC_CP: u32 = 1 << 1;

#[inline]
pub fn decode(addr: u32) -> Option<(usize, u32)> {
    let rel = addr.wrapping_sub(MSC_BASE);
    let n = (rel / MSC_STRIDE) as usize;
    let off = rel % MSC_STRIDE;
    (n < MSC_COUNT && off <= OCR).then_some((n, off))
}

#[derive(Debug, Clone, Default)]
pub struct Msc {
    pub clc: u32,
    pub fdr: u32,
    pub usr: u32,
    pub dsc: u32,
    pub dss: u32,
    pub dd: u32,
    pub dc: u32,
    pub dsdsl: u32,
    pub dsdsh: u32,
    dd_mask: u32,
    alt_mask: u32,
    pub esr: u32,
    pub ud: [u32; 4],
    pub icr: u32,
    pub isr: u32,
    pub ocr: u32,
    /// Command frame being shifted out: (word, end time).
    cmd: Option<(u32, u64)>,
    /// Upstream frames on the wire: (byte, arrival time).
    up: VecDeque<(u8, u64)>,
    pub commands: u64,
    pub upstream_frames: u64,
}

impl Msc {
    pub fn f_msc_hz(&self, f_fpi_hz: u64) -> u64 {
        let step = (self.fdr & 0x3FF) as u64;
        match (self.fdr >> 14) & 3 {
            1 => f_fpi_hz / (1024 - step),
            2 => f_fpi_hz * step / 1024,
            _ => 0,
        }
    }

    fn ns(&self, cycles: u64, f_fpi_hz: u64) -> Option<u64> {
        let f = self.f_msc_hz(f_fpi_hz);
        (f != 0).then(|| (cycles * 1_000_000_000).div_ceil(f))
    }

    /// Duration of one upstream frame (start + 8 data + parity + stop, 12 or 16 bit).
    fn upstream_frame_ns(&self, f_fpi_hz: u64) -> Option<u64> {
        let urr = (self.usr >> 1) & 7;
        let bits = if self.usr & 1 != 0 { 16 } else { 12 };
        if urr == 0 {
            return None; // receiver off
        }
        self.ns(bits * (4u64 << (urr - 1)), f_fpi_hz)
    }

    pub fn read(&self, off: u32) -> Option<u32> {
        Some(match off {
            CLC => (self.clc & !2) | ((self.clc & 1) << 1),
            ID => 0,
            FDR => self.fdr,
            USR => self.usr,
            DSC => self.dsc | if self.cmd.is_some() { DSC_CP } else { 0 },
            DSS => self.dss,
            DD => self.dd,
            DC => self.dc,
            DSDSL => self.dsdsl,
            DSDSH => self.dsdsh,
            ESR => self.esr,
            0x30 | 0x34 | 0x38 | 0x3C => self.ud[((off - UD0) / 4) as usize],
            ICR => self.icr,
            ISR => self.isr,
            ISC => 0,
            OCR => self.ocr,
            _ => return None,
        })
    }

    pub fn write(&mut self, off: u32, val: u32, mask: u32, now_ns: u64, f_fpi_hz: u64) -> bool {
        let merge = |old: u32| (old & !mask) | (val & mask);
        match off {
            CLC => self.clc = merge(self.clc),
            FDR => self.fdr = merge(self.fdr),
            USR => self.usr = merge(self.usr),
            DSC => {
                self.dsc = merge(self.dsc) & !DSC_CP;
                self.update_masks();
            }
            DSS => {}
            DD => self.dd = merge(self.dd),
            DC => {
                self.dc = merge(self.dc);
                let nbc = ((self.dsc >> 16) & 0x3F) as u64;
                let ppd = ((self.dsc >> 24) & 0x1F) as u64;
                // start + selection bit + NBC bits + passive phase, 2 f_MSC cycles per bit
                if let Some(t) = self.ns(2 * (nbc + 2 + ppd), f_fpi_hz) {
                    if self.clc & 1 == 0 && self.cmd.is_none() {
                        self.cmd = Some((self.dc, now_ns + t));
                    }
                }
            }
            DSDSL => {
                self.dsdsl = merge(self.dsdsl);
                self.update_masks();
            }
            DSDSH => {
                self.dsdsh = merge(self.dsdsh);
                self.update_masks();
            }
            ESR => self.esr = merge(self.esr),
            ICR => self.icr = merge(self.icr),
            ISC => {
                let v = val & mask;
                self.isr &= !(v & 0xFFFF);
                self.isr |= v >> 16;
            }
            OCR => self.ocr = merge(self.ocr),
            ID | 0x30 | 0x34 | 0x38 | 0x3C | ISR => {}
            _ => return false,
        }
        true
    }

    /// The command frame whose transmission has ended by `now` (bits beyond NBC are not sent).
    pub fn take_command(&mut self, now_ns: u64) -> Option<(u32, u64)> {
        match self.cmd {
            Some((w, t)) if t <= now_ns => {
                self.cmd = None;
                self.commands += 1;
                let nbc = (self.dsc >> 16) & 0x3F;
                let mask = if nbc >= 32 { u32::MAX } else { (1u32 << nbc) - 1 };
                Some((w & mask, t))
            }
            _ => None,
        }
    }

    /// A device starts sending `bytes` upstream at `start_ns`.
    pub fn queue_upstream(&mut self, bytes: &[u8], start_ns: u64, f_fpi_hz: u64) {
        let Some(dt) = self.upstream_frame_ns(f_fpi_hz) else { return };
        let mut t = self.up.back().map_or(start_ns, |&(_, t)| t.max(start_ns));
        for &b in bytes {
            t += dt;
            self.up.push_back((b, t));
        }
    }

    /// Load the next due upstream frame into UD0. Returns true when a frame was received and
    /// the receive-data request is enabled (one DMA request).
    pub fn receive_next(&mut self, now_ns: u64) -> Option<bool> {
        match self.up.front() {
            Some(&(b, t)) if t <= now_ns => {
                self.up.pop_front();
                self.upstream_frames += 1;
                let parity = b.count_ones() & 1;
                self.ud[0] = b as u32 | 1 << 16 | parity << 17;
                Some(self.icr >> 14 & 3 != 0)
            }
            _ => None,
        }
    }

    #[inline]
    pub fn next_event_ns(&self) -> Option<u64> {
        match (self.cmd, self.up.front()) {
            (Some((_, a)), Some(&(_, b))) => Some(a.min(b)),
            (Some((_, a)), None) => Some(a),
            (None, Some(&(_, b))) => Some(b),
            (None, None) => None,
        }
    }

    fn update_masks(&mut self) {
        let ndbl = ((self.dsc >> 3) & 0x1F).min(16);
        let ndbh = ((self.dsc >> 8) & 0x1F).min(16);
        let mut dd_m = 0u32;
        let mut alt_m = 0u32;
        for k in 0..ndbl {
            if (self.dsdsl >> (2 * k)) & 3 == 0 {
                dd_m |= 1 << k;
            } else {
                alt_m |= 1 << k;
            }
        }
        for k in 0..ndbh {
            if (self.dsdsh >> (2 * k)) & 3 == 0 {
                dd_m |= 1 << (16 + k);
            } else {
                alt_m |= 1 << (16 + k);
            }
        }
        self.dd_mask = dd_m;
        self.alt_mask = alt_m;
    }

    /// Downstream data bits (`SRL = DD[15:0]`, `SRH = DD[25:16]`); bits whose DSDSL/DSDSH source is
    /// not DD take the alternate input `altin` (GPTA outputs).
    #[inline]
    pub fn data_bits(&self, altin: u32) -> u32 {
        (self.dd & self.dd_mask) | (altin & self.alt_mask)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FPI: u64 = 90_000_000;

    #[test]
    fn command_frame_timing_and_upstream_reception_as_programmed_by_the_firmware() {
        let mut m = Msc::default();
        // 0x80110748.. (MSC1)
        for (off, v) in [(FDR, 0x43FE), (USR, 0x0C), (DSC, 0x001A_2A80), (ICR, 0x6000)] {
            assert!(m.write(off, v, !0, 0, FPI));
        }
        assert_eq!(m.f_msc_hz(FPI), 45_000_000);
        m.write(DC, 0x0200_000B | 0xF000_0000, !0, 1000, FPI);
        assert_ne!(m.read(DSC).unwrap() & DSC_CP, 0, "command pending");
        assert_eq!(m.take_command(1000), None);
        let (w, t) = m.take_command(10_000).unwrap();
        assert_eq!(w, 0x0200_000B, "26 command bits");
        assert_eq!(m.read(DSC).unwrap() & DSC_CP, 0);
        m.queue_upstream(&[0xA5, 0x01], t, FPI);
        // 12-bit frame at f_MSC / 128: 34.1 us
        assert_eq!(m.receive_next(t + 34_000), None);
        assert_eq!(m.receive_next(t + 34_200), Some(true));
        assert_eq!(m.read(UD0).unwrap(), 0xA5 | 1 << 16);
        assert_eq!(m.receive_next(t + 70_000), Some(true));
        assert_eq!(m.read(UD0).unwrap() & 0xFF, 1);
        assert_eq!(decode(0xF000_0930), Some((1, UD0)));
        assert_eq!(decode(0xF000_09FC), None, "SRCs belong to the interrupt system");
    }

    #[test]
    fn data_frame_bits_follow_dd_and_source_selection() {
        let mut m = Msc::default();
        m.write(DSC, 0x001A_2A80, !0, 0, FPI);
        m.write(DD, 0x0300_0801, !0, 0, FPI);
        assert_eq!(m.data_bits(0), 0x0300_0801 & 0x03FF_FFFF);
        m.write(DSDSL, 0b01, !0, 0, FPI); // bit 0 from ALTIN
        assert_eq!(m.data_bits(0) & 1, 0);
        assert_eq!(m.data_bits(1) & 1, 1);
    }
}
