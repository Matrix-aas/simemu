//! Context Save Areas, call-depth counting, trap entry and interrupt entry.
//!
//! * CSA layout / list handling: V2 CALL p. 3-63, RET p. 3-311, RFE p. 3-313, SVLCX p. 3-375,
//!   RSLCX p. 3-316, BISR p. 3-43 (identical RTL in V131-2; PCXI/FCX/LCX layouts V1 p. 3-9/4-12).
//! * `cdc_increment`/`cdc_decrement`/`cdc_zero`: V2 Table 1-9 (p. 1-8), PSW.CDC encoding V1 p. 3-6.
//! * Trap entry: V1 §6.2.3 (return address) and §6.2.5 (initial state, FCU special case, p. 6-4/6-5);
//!   FCD after a hardware context save: V1 p. 6-8.
//! * Interrupt entry: V1 §5.3 (p. 5-7); vector `BIV | PIPN << 5` (V1 p. 5-16; TC1.6 V1.0 and TC1.3.1
//!   have no BIV.VSS vector-spacing bit — that is a TC1.6P/AURIX addition).
//! * TC1.3.1: V131-1 §5.3 (p. 5-8) and §6.2.5 and the V131-2 CALL/RET/RFE/SVLCX/RSLCX/BISR RTL are
//!   identical to TC1.6, so nothing here depends on `Cpu::arch`; instruction availability differs
//!   (`crate::groups::TC16_ONLY_MNEMONICS`) and is handled by the group decoders.

use crate::arch::{csa_ea, icr, pcxi, psw, Trap};
use crate::bus::Bus;
use crate::cpu::Cpu;

/// A register slot of a context save area.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    Pcxi,
    Psw,
    D(u8),
    A(u8),
}

use Slot::{Pcxi as X, Psw as P};
const fn d(n: u8) -> Slot {
    Slot::D(n)
}
const fn a(n: u8) -> Slot {
    Slot::A(n)
}

/// Word order of an upper context (V2 CALL p. 3-63).
pub const UPPER: [Slot; 16] =
    [X, P, a(10), a(11), d(8), d(9), d(10), d(11), a(12), a(13), a(14), a(15), d(12), d(13), d(14), d(15)];
/// Word order of a lower context (V2 SVLCX p. 3-375). LDLCX/STLCX use the same order (words 0/1
/// are PCXI/A11 placeholders there, see their pages).
pub const LOWER: [Slot; 16] =
    [X, a(11), a(2), a(3), d(0), d(1), d(2), d(3), a(4), a(5), a(6), a(7), d(4), d(5), d(6), d(7)];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Upper,
    Lower,
}

impl<B: Bus> Cpu<B> {
    fn slot(&self, s: Slot) -> u32 {
        match s {
            Slot::Pcxi => self.pcxi,
            Slot::Psw => self.psw,
            Slot::D(n) => self.d[n as usize],
            Slot::A(n) => self.a[n as usize],
        }
    }

    /// Context save common to CALL, SVLCX, BISR and trap/interrupt entry:
    ///
    /// ```text
    /// if (FCX == 0) trap(FCU);  tmp_FCX = FCX;  EA = {FCXS, 6'b0, FCXO, 6'b0};  new_FCX = M(EA, word);
    /// M(EA, 16 * word) = {context};  PCXI.PCPN = ICR.CCPN;  PCXI.PIE = ICR.IE;  PCXI.UL = upper;
    /// PCXI[19:0] = FCX[19:0];  FCX[19:0] = new_FCX[19:0];
    /// ```
    ///
    /// Returns `Ok(true)` when `tmp_FCX == LCX` — the caller raises FCD after completing the
    /// operation. PCXI bits other than `PCPN`/`PIE`/`UL`/`[19:0]` are preserved. Bus errors on the CSA
    /// propagate as the access trap without modifying registers.
    ///
    /// Inlined fast path for a CSA in plain RAM without translated code (callers pass a constant
    /// `kind`); everything else is [`Cpu::save_context_full`].
    #[inline(always)]
    pub fn save_context(&mut self, kind: Kind) -> Result<bool, Trap> {
        let fcx = self.fcx;
        if fcx & pcxi::LINK != 0 {
            if let Some(p) = self.mem.host_w_plain(csa_ea(fcx), 64) {
                // SAFETY: host_w_plain validated the 64-byte writable range.
                let new_fcx = unsafe { self.save_words(kind, p) };
                self.save_finish(kind, fcx, new_fcx);
                return Ok(fcx == self.lcx);
            }
        }
        self.save_context_full(kind)
    }

    /// Read the link word of the CSA at `p`, then write the context words (UPPER / LOWER order).
    ///
    /// # Safety
    /// `p` must address 64 writable bytes.
    #[inline(always)]
    unsafe fn save_words(&self, kind: Kind, p: *mut u8) -> u32 {
        let (d, a, x) = (&self.d, &self.a, self.pcxi);
        let w: [u32; 16] = if kind == Kind::Upper {
            // UPPER order
            [x, self.psw, a[10], a[11], d[8], d[9], d[10], d[11], a[12], a[13], a[14], a[15], d[12], d[13], d[14], d[15]]
        } else {
            // LOWER order
            [x, a[11], a[2], a[3], d[0], d[1], d[2], d[3], a[4], a[5], a[6], a[7], d[4], d[5], d[6], d[7]]
        };
        let mut buf = [0u8; 64];
        for (i, v) in w.iter().enumerate() {
            buf[4 * i..4 * i + 4].copy_from_slice(&v.to_le_bytes());
        }
        // SAFETY: by the caller.
        unsafe {
            let new_fcx = u32::from_le_bytes(*(p as *const [u8; 4]));
            *(p as *mut [u8; 64]) = buf;
            new_fcx
        }
    }

    /// PCXI / FCX update after a context save from free-list head `fcx` with link `new_fcx`.
    #[inline(always)]
    fn save_finish(&mut self, kind: Kind, fcx: u32, new_fcx: u32) {
        if kind == Kind::Lower && self.arch == crate::arch::Arch::Tc13 {
            // TriCore 1.3 SVLCX/BISR: PCXI.PCPN and PCXI.PIE keep their values (Arch::Tc13).
            self.pcxi = (self.pcxi & !(pcxi::UL | pcxi::LINK)) | (fcx & pcxi::LINK);
            self.fcx = (fcx & !pcxi::LINK) | (new_fcx & pcxi::LINK);
            return;
        }
        let keep = self.pcxi & !(0xFF << pcxi::PCPN_SHIFT | pcxi::PIE | pcxi::UL | pcxi::LINK);
        self.pcxi = keep
            | (self.icr & icr::CCPN) << pcxi::PCPN_SHIFT
            | if self.icr & icr::IE != 0 { pcxi::PIE } else { 0 }
            | if kind == Kind::Upper { pcxi::UL } else { 0 }
            | (fcx & pcxi::LINK);
        self.fcx = (fcx & !pcxi::LINK) | (new_fcx & pcxi::LINK);
    }

    /// [`Cpu::save_context`] for any CSA location.
    #[inline(never)]
    pub fn save_context_full(&mut self, kind: Kind) -> Result<bool, Trap> {
        let fcx = self.fcx;
        if fcx & pcxi::LINK == 0 {
            return Err(Trap::FCU);
        }
        let ea = csa_ea(fcx);
        let order = if kind == Kind::Upper { &UPPER } else { &LOWER };
        let new_fcx = if let Some(p) = self.csa_host(ea) {
            // Fast path: a CSA is 64-byte aligned (its EA has 6 zero bits), so in RAM it is one
            // host slice; same loads/stores in the same order as the slow path below.
            // SAFETY: csa_host validated the 64-byte writable range.
            unsafe { self.save_words(kind, p) }
        } else {
            let new_fcx = self.ld32(ea)?;
            for (i, &s) in order.iter().enumerate() {
                let v = self.slot(s);
                self.st32(ea + 4 * i as u32, v)?;
            }
            new_fcx
        };
        self.save_finish(kind, fcx, new_fcx);
        Ok(fcx == self.lcx)
    }

    /// Context restore common to RET, RFE (upper) and RSLCX (lower):
    ///
    /// ```text
    /// if (PCXI[19:0] == 0) trap(CSU);  if (PCXI.UL != upper) trap(CTYP);
    /// EA = {PCXS, 6'b0, PCXO, 6'b0};  {new_PCXI, ...} = M(EA, 16 * word);
    /// M(EA, word) = FCX;  FCX[19:0] = PCXI[19:0];  PCXI = new_PCXI;
    /// ```
    ///
    /// All registers of the context except PCXI and PSW are written; the loaded PSW is returned so
    /// that the caller applies its own rule (RET keeps `PSW[25:24]` on TC1.3.1/TC1.6, RFE restores it whole). All loads
    /// complete before any register changes, so a bus error leaves the state untouched.
    ///
    /// Inlined fast path for a CSA in plain RAM without translated code (callers pass a constant
    /// `kind`); everything else is [`Cpu::restore_context_full`].
    #[inline(always)]
    pub fn restore_context(&mut self, kind: Kind) -> Result<u32, Trap> {
        let link = self.pcxi & pcxi::LINK;
        if link != 0 && (self.pcxi & pcxi::UL != 0) == (kind == Kind::Upper) {
            if let Some(p) = self.mem.host_w_plain(csa_ea(self.pcxi), 64) {
                // SAFETY: host_w_plain validated the 64-byte writable range.
                let w = unsafe { self.restore_words(p) };
                return Ok(self.restore_finish(kind, link, w));
            }
        }
        self.restore_context_full(kind)
    }

    /// Read the 16 words of the CSA at `p`, then write FCX into its first word.
    ///
    /// # Safety
    /// `p` must address 64 writable bytes.
    #[inline(always)]
    unsafe fn restore_words(&self, p: *mut u8) -> [u32; 16] {
        let mut w = [0u32; 16];
        // SAFETY: by the caller.
        unsafe {
            let b = *(p as *const [u8; 64]);
            for (i, v) in w.iter_mut().enumerate() {
                *v = u32::from_le_bytes(b[4 * i..4 * i + 4].try_into().unwrap());
            }
            *(p as *mut [u8; 4]) = self.fcx.to_le_bytes();
        }
        w
    }

    /// [`Cpu::restore_context`] for any CSA location.
    #[inline(never)]
    pub fn restore_context_full(&mut self, kind: Kind) -> Result<u32, Trap> {
        let link = self.pcxi & pcxi::LINK;
        if link == 0 {
            return Err(Trap::CSU);
        }
        if (self.pcxi & pcxi::UL != 0) != (kind == Kind::Upper) {
            return Err(Trap::CTYP);
        }
        let ea = csa_ea(self.pcxi);
        let mut w = [0u32; 16];
        if let Some(p) = self.csa_host(ea) {
            // SAFETY: csa_host validated the 64-byte writable range.
            w = unsafe { self.restore_words(p) };
        } else {
            for (i, v) in w.iter_mut().enumerate() {
                *v = self.ld32(ea + 4 * i as u32)?;
            }
            self.st32(ea, self.fcx)?;
        }
        Ok(self.restore_finish(kind, link, w))
    }

    /// Register, FCX and PCXI updates of a context restore of words `w`; returns the loaded PSW
    /// (upper) or the current one (lower).
    #[inline(always)]
    fn restore_finish(&mut self, kind: Kind, link: u32, w: [u32; 16]) -> u32 {
        let mut new_psw = self.psw;
        let (d, a) = (&mut self.d, &mut self.a);
        if kind == Kind::Upper {
            // UPPER order: PCXI, PSW, A10, A11, D8-D11, A12-A15, D12-D15
            new_psw = w[1];
            (a[10], a[11]) = (w[2], w[3]);
            d[8..12].copy_from_slice(&w[4..8]);
            a[12..16].copy_from_slice(&w[8..12]);
            d[12..16].copy_from_slice(&w[12..16]);
        } else {
            // LOWER order: PCXI, A11, A2, A3, D0-D3, A4-A7, D4-D7
            (a[11], a[2], a[3]) = (w[1], w[2], w[3]);
            d[0..4].copy_from_slice(&w[4..8]);
            a[4..8].copy_from_slice(&w[8..12]);
            d[4..8].copy_from_slice(&w[12..16]);
        }
        self.fcx = (self.fcx & !pcxi::LINK) | link;
        self.pcxi = w[0];
        new_psw
    }

    fn cdc_fields(&self) -> Option<(u32, u32)> {
        let cdc = self.psw & psw::CDC;
        if cdc == psw::CDC {
            return None; // 1111111B: counting disabled
        }
        let ones = (cdc << 25).leading_ones(); // leading ones of the 7-bit field
        let width = 6 - ones; // 0cccccc -> 6 bits ... 1111110 -> 0 bits
        Some((cdc, width))
    }

    /// `cdc_increment()` (V2 Table 1-9): false if CDC = 1111111B, else increment CDC.COUNT and
    /// return true on overflow (the count wraps within its field).
    pub fn cdc_increment(&mut self) -> bool {
        let Some((cdc, width)) = self.cdc_fields() else { return false };
        let mask = (1u32 << width) - 1;
        let count = (cdc & mask) + 1;
        self.psw = (self.psw & !mask) | (count & mask);
        count > mask
    }

    /// `cdc_decrement()` (V2 Table 1-9): false if CDC = 1111111B, else decrement CDC.COUNT and
    /// return true on underflow.
    pub fn cdc_decrement(&mut self) -> bool {
        let Some((cdc, width)) = self.cdc_fields() else { return false };
        let mask = (1u32 << width) - 1;
        let count = cdc & mask;
        self.psw = (self.psw & !mask) | (count.wrapping_sub(1) & mask);
        count == 0
    }

    /// `cdc_zero()` (V2 Table 1-9).
    pub fn cdc_zero(&self) -> bool {
        match self.cdc_fields() {
            None => true,
            Some((cdc, width)) => cdc & ((1u32 << width) - 1) == 0,
        }
    }

    /// Supervisor-mode PSW/stack switch shared by trap and interrupt entry.
    fn enter_supervisor_stack(&mut self) {
        if self.psw & psw::IS == 0 {
            self.a[10] = self.isp;
        }
    }

    /// Take a trap (V1 §6.2.5). `ret` is the return address (see [`crate::Flow::Trap`]).
    pub fn enter_trap(&mut self, trap: Trap, ret: u32) {
        let vector = (self.btv & !1) | (trap.class as u32) << 5;
        if trap == Trap::FCD {
            // SYSCON.FCDSF (bit 0): set whenever an FCD trap is generated (V1 p. 6-8, p. 3-13).
            self.syscon |= 1;
        }
        let fcd = if trap == Trap::FCU {
            Err(Trap::FCU)
        } else {
            self.save_context(Kind::Upper)
        };
        match fcd {
            Ok(fcd) => {
                self.a[11] = ret;
                self.d[15] = trap.tin as u32;
                self.enter_supervisor_stack();
                self.psw = (self.psw & !(psw::PRS | psw::IO | psw::CDC | psw::GW))
                    | psw::IS
                    | psw::IO_SUPERVISOR
                    | psw::CDE;
                self.icr &= !icr::IE;
                self.pc = vector;
                if fcd && trap != Trap::FCD {
                    // FCD after the entry's context save: its return address is the handler entry.
                    self.enter_trap(Trap::FCD, vector);
                }
            }
            Err(_) => {
                // FCU (V1 p. 6-5): no context save; only these states are guaranteed. Any error
                // during the context save (bus error, MEM, ...) is also FCU (V1 p. 6-9 "FCU").
                let t = Trap::FCU;
                self.d[15] = t.tin as u32;
                self.enter_supervisor_stack();
                self.psw = (self.psw & !(psw::PRS | psw::IO)) | psw::IO_SUPERVISOR;
                self.icr &= !icr::IE;
                self.pc = (self.btv & !1) | (t.class as u32) << 5;
            }
        }
    }

    /// Asynchronous trap from the system (NMI, external bus error): taken at the current
    /// instruction boundary, returning to `self.pc`.
    pub fn raise_async_trap(&mut self, trap: Trap) {
        let pc = self.pc;
        self.enter_trap(trap, pc);
    }

    /// Interrupt entry at the current instruction boundary (V1 §5.3) if `ICR.IE == 1` and
    /// `pipn > ICR.CCPN`. Returns `Ok(true)` when the interrupt was taken (the caller acknowledges
    /// the service request), `Ok(false)` when it is masked, `Err(trap)` when the context save failed
    /// and the trap was entered instead (FCU: the request stays pending).
    ///
    /// Note: V1 §5.3 clears PSW.CDC but does not list PSW.CDE (unlike trap entry, §6.2.5, which sets
    /// CDE = 1); CDE is therefore left unchanged (the Unicorn board does the same since B1 was fixed).
    pub fn take_interrupt(&mut self, pipn: u8) -> Result<bool, Trap> {
        let p = pipn as u32;
        if self.icr & icr::IE == 0 || p <= self.icr & icr::CCPN {
            return Ok(false);
        }
        let ret = self.pc;
        let fcd = match self.save_context(Kind::Upper) {
            Ok(f) => f,
            Err(_) => {
                // any context-save error is FCU (V1 p. 6-9)
                self.enter_trap(Trap::FCU, ret);
                return Err(Trap::FCU);
            }
        };
        self.a[11] = ret;
        self.enter_supervisor_stack();
        self.psw = (self.psw & !(psw::PRS | psw::IO | psw::CDC | psw::GW)) | psw::IS | psw::IO_SUPERVISOR;
        self.icr = (self.icr & !(icr::IE | icr::CCPN)) | p;
        let vector = (self.biv & !1) | p << 5;
        self.pc = vector;
        if fcd {
            self.enter_trap(Trap::FCD, vector);
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Arch, Memory, NullBus};

    /// DSPR with a free list of 4 CSAs at 0xD000_0400.. (64-byte aligned, linked).
    fn cpu() -> Cpu<NullBus> {
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x1_0000], true);
        m.map(0xD000_0000, r, 0, 0x1_0000);
        let mut c = Cpu::new(Arch::Tc16, m, NullBus);
        let link = |ea: u32| ((ea >> 12) & 0xF_0000) | ((ea >> 6) & 0xFFFF);
        for i in 0..4u32 {
            let ea = 0xD000_0400 + 64 * i;
            let next = if i < 3 { link(ea + 64) } else { 0 };
            c.st32(ea, next).unwrap();
        }
        c.fcx = link(0xD000_0400);
        c.lcx = link(0xD000_0400 + 64 * 2);
        c
    }

    #[test]
    fn upper_save_restore_roundtrip() {
        let mut c = cpu();
        for i in 0..16 {
            c.d[i] = 0x100 + i as u32;
            c.a[i] = 0x200 + i as u32;
        }
        c.icr = 0x0000_0105;
        c.psw = 0x0300_0B80 | psw::C;
        let fcx0 = c.fcx;
        assert_eq!(c.save_context(Kind::Upper), Ok(false));
        assert_eq!(c.pcxi & pcxi::LINK, fcx0 & pcxi::LINK);
        assert_eq!(c.pcxi >> 24, 5);
        assert_ne!(c.pcxi & pcxi::PIE, 0);
        assert_ne!(c.pcxi & pcxi::UL, 0);
        assert_eq!(c.ld32(0xD000_0408).unwrap(), 0x20A, "A10 at word 2");
        c.d[8] = 0;
        let p = c.restore_context(Kind::Upper).unwrap();
        assert_eq!(p, 0x0300_0B80 | psw::C);
        assert_eq!(c.d[8], 0x108);
        assert_eq!(c.fcx, fcx0, "CSA returned to the free list");
        assert_eq!(c.restore_context(Kind::Lower), Err(Trap::CSU), "PCXI restored to 0");
    }

    #[test]
    fn tc13_lower_save_keeps_pcpn_and_pie() {
        for (arch, pcpn, pie) in [(Arch::Tc131, 0xE, true), (Arch::Tc13, 0, false)] {
            let mut c = cpu();
            c.arch = arch;
            c.pcxi = 0;
            c.icr = 0x0000_010E;
            assert_eq!(c.save_context(Kind::Lower), Ok(false));
            assert_eq!((c.pcxi >> 24, c.pcxi & pcxi::PIE != 0, c.pcxi & pcxi::UL), (pcpn, pie, 0), "{arch:?}");
            assert_ne!(c.pcxi & pcxi::LINK, 0);
        }
    }

    #[test]
    fn fcd_when_last_csa_is_used() {
        let mut c = cpu();
        assert_eq!(c.save_context(Kind::Lower), Ok(false));
        assert_eq!(c.save_context(Kind::Lower), Ok(false));
        assert_eq!(c.save_context(Kind::Lower), Ok(true), "third save uses the CSA named by LCX");
        assert_eq!(c.save_context(Kind::Lower), Ok(false));
        assert_eq!(c.save_context(Kind::Lower), Err(Trap::FCU));
    }

    #[test]
    fn interrupt_entry() {
        let mut c = cpu();
        c.pc = 0x8000_1234;
        c.biv = 0x8000_2000;
        c.isp = 0xD000_F000;
        c.a[10] = 0xD000_8000;
        c.psw = psw::CDE; // user stack (the reset PSW has IS = 1)
        c.icr = icr::IE | 3;
        assert_eq!(c.take_interrupt(3), Ok(false), "PIPN must exceed CCPN");
        assert_eq!(c.take_interrupt(9), Ok(true));
        assert_eq!(c.pc, 0x8000_2000 | 9 << 5);
        assert_eq!(c.a[11], 0x8000_1234);
        assert_eq!(c.a[10], 0xD000_F000);
        assert_eq!(c.icr & (icr::IE | icr::CCPN), 9);
        assert_eq!((c.pcxi >> 24, c.pcxi & pcxi::PIE != 0), (3, true));
        assert_eq!(c.psw & (psw::IS | psw::IO | psw::CDC), psw::IS | psw::IO_SUPERVISOR);
        assert_eq!(c.take_interrupt(200), Ok(false), "IE = 0 inside the ISR");
    }

    #[test]
    fn trap_entry_and_fcu() {
        let mut c = cpu();
        c.btv = 0x8000_0100;
        c.pc = 0x8000_4000;
        c.enter_trap(Trap::ALN, 0x8000_3FFC);
        assert_eq!((c.pc, c.a[11], c.d[15]), (0x8000_0100 | 2 << 5, 0x8000_3FFC, 4));
        assert_ne!(c.psw & psw::CDE, 0);
        c.fcx = 0;
        c.enter_trap(Trap::SOVF, 0);
        assert_eq!((c.pc, c.d[15]), (0x8000_0100 | 3 << 5, 4), "FCU replaces the trap");
    }

    /// V1 p. 6-8 / 3-13: "the FCDSF flag in the SYSCON register is set whenever an FCD trap is
    /// generated" (same text V131-1 p. 6-10).
    #[test]
    fn fcd_sets_syscon_fcdsf() {
        let mut c = cpu();
        c.btv = 0x8000_0100;
        assert_eq!(c.save_context(Kind::Lower), Ok(false));
        assert_eq!(c.save_context(Kind::Lower), Ok(false));
        c.enter_trap(Trap::ALN, 0x8000_3FFC); // saves into the CSA named by LCX -> FCD
        assert_eq!((c.pc, c.d[15]), (0x8000_0100 | 3 << 5, 1), "FCD entered after the ALN entry");
        assert_eq!(c.syscon & 1, 1, "SYSCON.FCDSF");
    }

    /// V1 p. 6-9 "FCU": "The FCU trap is also taken if any error is encountered during a context
    /// save or restore operation" — also for the context save of trap and interrupt entry.
    #[test]
    fn csa_bus_error_on_entry_is_fcu() {
        let mut c = cpu();
        c.btv = 0x8000_0100;
        c.fcx = 0x0005_0000; // CSA at 0x5000_0000: unmapped, the save fails with a bus error
        c.enter_trap(Trap::ALN, 0x8000_3FFC);
        assert_eq!((c.pc, c.d[15]), (0x8000_0100 | 3 << 5, 4), "FCU, not the bus error trap");
        let mut c = cpu();
        c.btv = 0x8000_0100;
        c.fcx = 0x0005_0000;
        c.icr = icr::IE;
        assert_eq!(c.take_interrupt(5), Err(Trap::FCU));
        assert_eq!((c.pc, c.d[15]), (0x8000_0100 | 3 << 5, 4));
    }

    #[test]
    fn call_depth_counter() {
        let mut c = cpu();
        c.psw = 0b0111110; // 6-bit counter at 62
        assert!(!c.cdc_increment());
        assert!(c.cdc_increment(), "63 -> overflow");
        c.psw = 0b1111110; // 0-bit counter: every call overflows
        assert!(c.cdc_increment());
        c.psw = 0b1111111;
        assert!(!c.cdc_increment() && !c.cdc_decrement() && c.cdc_zero());
        c.psw = 0b1000000; // 5-bit counter at 0
        assert!(c.cdc_zero());
        assert!(c.cdc_decrement(), "underflow");
    }
}
