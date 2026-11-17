//! TC1791 High-Speed Synchronous Serial Channels SSC0..SSC3 (master mode, DMA-driven).
//!
//! No TC1791 user manual is available; the register map is the AUDO SSC kernel as the
//! SIMOS18.1 SC800S50 firmware programs it (dynamic trace of the unmodified firmware):
//!
//! | off  | reg   | firmware evidence |
//! |------|-------|-------------------|
//! | 0x00 | CLC   | 0x8010878E / 0x800E0766 write 0 and read back (DISS must follow DISR) |
//! | 0x0C | FDR   | 0x800A931C writes 0x43FF (DM=1 normal divider, STEP=0x3FF -> f_SSC = f_FPI) |
//! | 0x10 | CON   | 0x800A8D62..8D8C: read, clear EN(15) (programming mode), write BR, write EN=1,MS=1,HB=1,BM=0xF |
//! | 0x14 | BR    | 0x800A8D7A: 0x2C (1 MHz at f_FPI = 90 MHz) / 0xB3 |
//! | 0x18 | SSOC  | 0x800A8EF2 sets OEN (bits 15:8) for the job's SLSO line; 0x800A8F90 clears bits 15:8 at job end |
//! | 0x1C | SSOTC | 0x800A8EE0 (lead/trail/inactive delays; not timed here) |
//! | 0x20 | TB    | DMA destination (DADR of the TX channel, 0x800A9342) |
//! | 0x24 | RB    | DMA source (SADR of the RX channel, 0x800A935E) |
//! | 0x28 | STAT  | 0x800A8D2E polls BSY (bit 12) before every job |
//! | 0x2C | EFM   | error flag modification (CLR bits 11:8, SET bits 15:12) |
//!
//! Module bases: SSC_n = 0xF031_0000 + 0x100 * n (the job-start routine 0x800A8CE8 indexes
//! 0xF031_0000 + (bus << 8)). Requests: TIR when TB is moved into the shift register, RIR when
//! a received frame is loaded into RB, EIR on receive error (RB overwritten while unread with
//! CON.REN). In SIMOS18 these lines are consumed by the DMA (no SSC SRC is programmed by the
//! firmware), see [`crate::dma`].
//!
//! Not modelled: slave mode (MS=0; the firmware only uses master mode), SSOTC lead/trail/inactive
//! delays, phase/baud error detection (slave-only), the RX/TX FIFOs, the PISEL input selection
//! (there is only one MRST source per device here).

pub const SSC_BASE: u32 = 0xF031_0000;
pub const SSC_STRIDE: u32 = 0x100;
pub const SSC_COUNT: usize = 4;

pub const CLC: u32 = 0x00;
pub const PISEL: u32 = 0x04;
pub const ID: u32 = 0x08;
pub const FDR: u32 = 0x0C;
pub const CON: u32 = 0x10;
pub const BR: u32 = 0x14;
pub const SSOC: u32 = 0x18;
pub const SSOTC: u32 = 0x1C;
pub const TB: u32 = 0x20;
pub const RB: u32 = 0x24;
pub const STAT: u32 = 0x28;
pub const EFM: u32 = 0x2C;

pub const CON_BM: u32 = 0x000F;
pub const CON_HB: u32 = 1 << 4;
pub const CON_LB: u32 = 1 << 7;
pub const CON_REN: u32 = 1 << 9;
pub const CON_MS: u32 = 1 << 14;
pub const CON_EN: u32 = 1 << 15;

pub const STAT_RE: u32 = 1 << 9;
pub const STAT_BSY: u32 = 1 << 12;
const STAT_ERR: u32 = 0x0F00;

/// SSC index and register offset of a TC1791 SSC address.
#[inline]
pub fn decode(addr: u32) -> Option<(usize, u32)> {
    let rel = addr.wrapping_sub(SSC_BASE);
    let n = (rel / SSC_STRIDE) as usize;
    (n < SSC_COUNT).then_some((n, rel % SSC_STRIDE))
}

/// A device on the SPI bus of one SSC. Words are presented MSB first (wire order), whatever
/// CON.HB is.
pub trait SpiSlave {
    /// The set of active slave-select lines (bit n = SLSOn) of SSC `ssc` changed to `lines`
    /// (0 = all deselected).
    fn select(&mut self, ssc: usize, lines: u8, now_ns: u64);
    /// One frame of `bits` bits was shifted out on MTSR while `lines` were selected (`lines` may
    /// be 0: devices with a port-pin chip select decide themselves); returns the MRST bits the
    /// selected device shifted in during that frame (all ones when nobody drives MRST: pull-up).
    fn shift(&mut self, ssc: usize, lines: u8, mtsr: u32, bits: u32, now_ns: u64) -> u32;
}

/// Request lines pulsed by an SSC (consumed by the DMA / interrupt routing of the board).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct SscRequests {
    pub tir: bool,
    pub rir: bool,
    pub eir: bool,
}

impl SscRequests {
    pub fn any(&self) -> bool {
        self.tir || self.rir || self.eir
    }
    fn or(&mut self, o: SscRequests) {
        self.tir |= o.tir;
        self.rir |= o.rir;
        self.eir |= o.eir;
    }
}

#[derive(Debug, Clone, Copy)]
struct Frame {
    data: u32,
    bits: u32,
    done_ns: u64,
}

#[derive(Debug, Clone)]
pub struct Ssc {
    pub index: usize,
    clc: u32,
    pisel: u32,
    fdr: u32,
    con: u32,
    br: u32,
    ssoc: u32,
    ssotc: u32,
    tb: u32,
    tb_full: bool,
    rb: u32,
    rb_unread: bool,
    err: u32,
    frame: Option<Frame>,
    /// Slave-select lines currently driven active (as seen by the slaves).
    cs: u8,
    /// Frames completed since reset (diagnostics).
    pub frames: u64,
}

impl Ssc {
    pub fn new(index: usize) -> Self {
        Self {
            index,
            clc: 0,
            pisel: 0,
            fdr: 0,
            con: 0,
            br: 0,
            ssoc: 0,
            ssotc: 0,
            tb: 0,
            tb_full: false,
            rb: 0,
            rb_unread: false,
            err: 0,
            frame: None,
            cs: 0,
            frames: 0,
        }
    }

    /// Bits per frame (CON.BM + 1; BM = 0 is reserved and treated as 1 bit).
    pub fn bits(&self) -> u32 {
        (self.con & CON_BM) + 1
    }

    /// Kernel clock from the fractional divider (FDR.DM 1 = normal, 2 = fractional; 0/3 = off).
    pub fn f_ssc_hz(&self, f_fpi_hz: u64) -> u64 {
        let step = (self.fdr & 0x3FF) as u64;
        match (self.fdr >> 14) & 3 {
            1 => f_fpi_hz / (1024 - step),
            2 => f_fpi_hz * step / 1024,
            _ => 0,
        }
    }

    /// Duration of one frame in ns (master mode baud rate = f_SSC / (2 * (BR + 1))); `None`
    /// while the kernel clock is off.
    pub fn frame_ns(&self, f_fpi_hz: u64) -> Option<u64> {
        let f = self.f_ssc_hz(f_fpi_hz);
        (f != 0).then(|| (self.bits() as u64 * 2 * ((self.br & 0xFFFF) as u64 + 1) * 1_000_000_000).div_ceil(f))
    }

    pub fn busy(&self) -> bool {
        self.frame.is_some()
    }

    /// Slave-select lines enabled by SSOC.OEN.
    fn oen(&self) -> u8 {
        (self.ssoc >> 8) as u8
    }

    pub fn peek(&self, off: u32) -> Option<u32> {
        Some(match off {
            CLC => (self.clc & !2) | ((self.clc & 1) << 1),
            PISEL => self.pisel,
            FDR => self.fdr,
            CON => self.con,
            BR => self.br,
            SSOC => self.ssoc,
            SSOTC => self.ssotc,
            TB => self.tb,
            RB => self.rb,
            STAT => self.err | if self.busy() { STAT_BSY } else { 0 },
            EFM => 0,
            _ => return None,
        })
    }

    pub fn read(&mut self, off: u32) -> Option<u32> {
        if off == RB {
            self.rb_unread = false;
        }
        self.peek(off)
    }

    /// Register write with byte-lane mask. A TB write starts a frame if the shift register is
    /// free (TIR is returned in that case).
    pub fn write(&mut self, off: u32, val: u32, mask: u32, now_ns: u64, f_fpi_hz: u64) -> Option<SscRequests> {
        let merge = |old: u32| (old & !mask) | (val & mask);
        match off {
            CLC => self.clc = merge(self.clc),
            PISEL => self.pisel = merge(self.pisel),
            FDR => self.fdr = merge(self.fdr),
            CON => self.con = merge(self.con) & 0xFFFF,
            BR => self.br = merge(self.br) & 0xFFFF,
            SSOC => self.ssoc = merge(self.ssoc) & 0xFFFF,
            SSOTC => self.ssotc = merge(self.ssotc),
            TB => {
                self.tb = merge(self.tb) & 0xFFFF;
                self.tb_full = true;
                return Some(self.try_start(now_ns, f_fpi_hz));
            }
            EFM => {
                let v = val & mask;
                self.err &= !(v & STAT_ERR);
                self.err |= (v >> 4) & STAT_ERR;
            }
            ID | RB | STAT => {}
            _ => return None,
        }
        Some(SscRequests::default())
    }

    fn try_start(&mut self, now_ns: u64, f_fpi_hz: u64) -> SscRequests {
        if self.frame.is_some() || !self.tb_full || self.con & (CON_EN | CON_MS) != (CON_EN | CON_MS) || self.clc & 1 != 0 {
            return SscRequests::default();
        }
        let Some(dur) = self.frame_ns(f_fpi_hz) else { return SscRequests::default() };
        let bits = self.bits();
        let mask = if bits >= 32 { u32::MAX } else { (1 << bits) - 1 };
        let mut data = self.tb & mask;
        if self.con & CON_HB == 0 {
            data = data.reverse_bits() >> (32 - bits); // LSB first on the wire
        }
        self.tb_full = false;
        self.frame = Some(Frame { data, bits, done_ns: now_ns + dur });
        SscRequests { tir: true, ..Default::default() }
    }

    /// End time of the frame being shifted.
    pub fn next_event_ns(&self) -> Option<u64> {
        self.frame.map(|f| f.done_ns)
    }

    /// True if `advance(now_ns, ...)` has work to do (a frame finished or CS needs releasing).
    #[inline]
    pub fn has_due(&self, now_ns: u64) -> bool {
        self.frame.map_or(self.cs != 0, |f| now_ns >= f.done_ns)
    }

    /// Complete the current frame if it is due at `now_ns` (one frame per call, so that the DMA
    /// can refill TB in between). Returns the request lines pulsed.
    pub fn advance(&mut self, now_ns: u64, f_fpi_hz: u64, bus: &mut dyn SpiSlave) -> SscRequests {
        let mut req = SscRequests::default();
        let Some(f) = self.frame else {
            if self.cs != 0 {
                // Transfer ended earlier (or OEN was dropped): release the select lines.
                self.cs = 0;
                bus.select(self.index, 0, now_ns);
            }
            return req;
        };
        if now_ns < f.done_ns {
            return req;
        }
        self.frame = None;
        let lines = self.oen();
        if lines != self.cs {
            self.cs = lines;
            bus.select(self.index, lines, f.done_ns);
        }
        // The bus is asked even with no SLSO active: devices may be selected by port pins.
        let miso = if self.con & CON_LB != 0 { f.data } else { bus.shift(self.index, lines, f.data, f.bits, f.done_ns) };
        let mask = if f.bits >= 32 { u32::MAX } else { (1 << f.bits) - 1 };
        let mut rx = miso & mask;
        if self.con & CON_HB == 0 {
            rx = rx.reverse_bits() >> (32 - f.bits);
        }
        if self.rb_unread {
            self.err |= STAT_RE;
            req.eir = self.con & CON_REN != 0;
        }
        self.rb = rx;
        self.rb_unread = true;
        self.frames += 1;
        req.rir = true;
        // A word already waiting in TB is shifted immediately (continuous transfer, select
        // stays active); otherwise the transfer ends and the select lines are released.
        req.or(self.try_start(f.done_ns, f_fpi_hz));
        if self.frame.is_none() && self.cs != 0 {
            self.cs = 0;
            bus.select(self.index, 0, f.done_ns);
        }
        req
    }

    /// Selected lines as seen by the slaves (diagnostics/tests).
    pub fn selected(&self) -> u8 {
        self.cs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Echo slave that records traffic and answers `!mtsr`.
    #[derive(Default)]
    struct Probe {
        log: Vec<(u8, u32)>,
        selects: Vec<u8>,
    }
    impl SpiSlave for Probe {
        fn select(&mut self, _ssc: usize, lines: u8, _now: u64) {
            self.selects.push(lines);
        }
        fn shift(&mut self, _ssc: usize, lines: u8, mtsr: u32, bits: u32, _now: u64) -> u32 {
            self.log.push((lines, mtsr));
            !mtsr & ((1 << bits) - 1)
        }
    }

    const FPI: u64 = 90_000_000;

    fn ssc() -> Ssc {
        let mut s = Ssc::new(1);
        // Exactly the firmware's programming sequence (S50 0x800A931C, 0x800A8D7A, 0x800A8D8C, 0x800A8EF2).
        s.write(FDR, 0x43FF, !0, 0, FPI);
        s.write(CON, 0x4000, !0, 0, FPI);
        s.write(BR, 0x2C, !0, 0, FPI);
        s.write(CON, 0xC01F, !0, 0, FPI);
        s.write(SSOC, 0x0400, !0, 0, FPI);
        s
    }

    #[test]
    fn frame_timing_flags_and_requests() {
        let mut s = ssc();
        let mut p = Probe::default();
        assert_eq!(s.frame_ns(FPI), Some(16_000), "16 bit at 1 MHz");
        assert_eq!(s.read(STAT).unwrap() & STAT_BSY, 0);
        let r = s.write(TB, 0x7078, 0xFFFF, 1000, FPI).unwrap();
        assert!(r.tir && !r.rir, "TB moved to the shift register -> TIR");
        assert_ne!(s.read(STAT).unwrap() & STAT_BSY, 0, "BSY while shifting");
        // second word waits in TB
        let r = s.write(TB, 0x8CB4, 0xFFFF, 2000, FPI).unwrap();
        assert!(!r.any(), "shift register busy: no TIR yet");
        assert!(!s.advance(16_999, FPI, &mut p).any(), "not done before 16 us");
        let r = s.advance(17_000, FPI, &mut p);
        assert!(r.rir && r.tir, "frame 1 received, frame 2 started from TB");
        assert_eq!(p.selects, vec![0x04], "SLSO2 asserted once for the whole transfer");
        assert_eq!(s.read(RB), Some(!0x7078 & 0xFFFF));
        let r = s.advance(33_000, FPI, &mut p);
        assert!(r.rir && !r.tir && !r.eir);
        assert_eq!(p.log, vec![(0x04, 0x7078), (0x04, 0x8CB4)]);
        assert_eq!(p.selects, vec![0x04, 0], "deselected when TB is empty at frame end");
        assert_eq!(s.read(STAT).unwrap() & (STAT_BSY | STAT_RE), 0, "idle, RB was read in time");
        s.write(EFM, 1 << 13, !0, 0, FPI);
        assert_eq!(s.read(STAT).unwrap() & STAT_RE, STAT_RE, "EFM.SETRE");
        s.write(EFM, 1 << 9, !0, 0, FPI);
        assert_eq!(s.read(STAT).unwrap() & STAT_RE, 0, "EFM.CLRRE");
    }

    #[test]
    fn receive_error_requests_eir_when_enabled_and_lsb_first_order() {
        let mut s = ssc();
        let mut p = Probe::default();
        s.write(CON, 0xC20F, !0, 0, FPI); // EN MS REN, LSB first (HB=0), 16 bit
        s.write(TB, 0x0001, 0xFFFF, 0, FPI);
        s.advance(16_000, FPI, &mut p);
        assert_eq!(p.log[0].1, 0x8000, "LSB first: bit 0 is the first bit on the wire");
        assert_eq!(s.read(RB), Some(0xFFFE), "and the received word is re-ordered the same way");
        s.write(TB, 0x0002, 0xFFFF, 20_000, FPI);
        s.advance(36_000, FPI, &mut p); // RB read -> no error
        s.write(TB, 0x0003, 0xFFFF, 40_000, FPI);
        let r = s.advance(56_000, FPI, &mut p);
        assert!(r.eir, "RB overwritten while unread with REN set");
    }

    #[test]
    fn no_transfer_in_programming_mode_or_without_kernel_clock() {
        let mut s = ssc();
        s.write(CON, 0x400F, !0, 0, FPI);
        assert!(!s.write(TB, 1, 0xFFFF, 0, FPI).unwrap().tir);
        s.write(CON, 0xC00F, !0, 0, FPI);
        s.write(FDR, 0, !0, 0, FPI);
        assert!(!s.write(TB, 1, 0xFFFF, 0, FPI).unwrap().tir);
        assert_eq!(decode(0xF031_0128), Some((1, STAT)));
        assert_eq!(decode(0xF031_0400), None);
    }
}
