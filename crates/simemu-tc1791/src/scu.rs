//! System Control Unit: PLL/oscillator status, reset control, watchdog timer (WDT) with the
//! ENDINIT protection mechanism, and the GPIO port input/output model.
//!
//! WDT protocol as used by the firmware (SBOOT 0x80007510 / 0x800076C0) and the AUDO-family WDT:
//! * password access: write WDT_CON0 with WDTPW = current password, WDTHPW1 = 1111b,
//!   `WDTHPW0 = WDT_CON1[3:2]` (DR/IR), WDTLCK = 0 -> unlocks one modify access;
//! * modify access: WDTLCK = 1, WDTHPW1 = 1111b -> updates WDTREL/WDTPW/ENDINIT and re-locks;
//!   setting ENDINIT = 1 services the WDT (reload with WDTREL), clearing it enters time-out mode
//!   (reload with 0xFFFC);
//! * any other write is an access error -> prewarning -> reset.
//!
//! The counter is clocked with fFPI/16384 (WDT_CON1.IR = 0) or fFPI/256 (IR = 1) and counts up;
//! an overflow (0xFFFF -> 0) is a time-out -> prewarning -> reset.

use crate::memory::Tc1791HardwareState;
use crate::stm_icu::{CYCLES_PER_INSN, FOSC_HZ};

pub const SCU_BASE: u32 = 0xF000_0500;
pub const WDT_CON0: u32 = 0xF000_05F0;
pub const WDT_CON1: u32 = 0xF000_05F4;
pub const WDT_SR: u32 = 0xF000_05F8;
pub const SCU_RSTSTAT: u32 = 0xF000_0550;
pub const SCU_SWRSTCON: u32 = 0xF000_0560;
pub const SCU_CHIPID: u32 = 0xF000_0640;

/// SCU_CHIPID (`0xF000_0640`, Infineon TC1791 DS v1.1 Tables 4 & 10, pp. 74, 76 — VERIFIED-DS):
/// * `0x8700_9502`: SAK-TC1791F-512F240EP AB-step (SIMOS18.1 ECU: EP=1, 240 MHz=0, 4 MB=7, CHID=0x95, CHREV=0x02).
/// * `0x9600_AA02`: SAK-TC1791S-384F200EP AB-step (`06K907425B` H13 ECU: EP=1, 200 MHz=1, 3 MB=6, CHID=0xAA, CHREV=0x02).
///
/// Firmware `0x80112486` (`0x8011249E..A2`) explicitly maps CHID `0xAA -> 0x95` and masks `& 0xFF` (VERIFIED-S).
pub const CHIPID_VALUE: u32 = 0x8700_9502;
pub const CHIPID_TC1791S_384_AB: u32 = 0x9600_AA02;

/// Duration of WDT prewarning mode before the reset, in WDT counter clocks. UNVERIFIED (no
/// TC1791 user manual available); the NMI that hardware raises on entering prewarning is not
/// modelled.
pub const WDT_PREWARNING_COUNTS: u32 = 256;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetKind {
    PowerOn,
    Software,
    Watchdog,
}

impl ResetKind {
    /// SCU_RSTSTAT value after this reset (PORST = bit 16, WDT = bit 3, SW = bit 4).
    pub fn rststat(self) -> u32 {
        match self {
            ResetKind::PowerOn => 0x0001_0000,
            ResetKind::Software => 0x0000_0010,
            ResetKind::Watchdog => 0x0000_0008,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WdtMode {
    Normal,
    TimeOut,
    Prewarning,
    Disabled,
}

pub struct ScuWdt {
    pub rststat: u32,
    pub osccon: u32,
    pub pllcon0: u32,
    pub pllcon1: u32,
    pub pll_findis: bool,
    pub pllerco0: u32,
    pub pllerco1: u32,
    pub pller_findis: bool,
    pub ccucon0: u32,
    pub ccucon1: u32,

    /// WDT_CON0 without the hardware password fields: `REL[31:16] | PW[15:8] | ENDINIT[0]`.
    pub wdt_rel: u32,
    pub wdt_pw: u32,
    pub endinit: bool,
    pub wdt_con1: u32,
    pub wdt_unlocked: bool,
    pub wdt_mode: WdtMode,
    pub wdt_tim: u32,
    pub wdt_is: bool,
    pub wdt_ae: bool,
    pub wdt_oe: bool,
    /// CPU cycles accumulated towards the next WDT count.
    pub wdt_frac: u64,
    pub wdt_prewarn_left: u32,
    pub wdt_access_errors: u64,
    pub wdt_services: u64,

    /// Writes dropped because ENDINIT = 1 (count, last address).
    pub endinit_dropped: u64,
    pub last_endinit_drop: u32,

    /// External logic level of input pins per port (P0..P17); `None` bits follow the pull device.
    pub port_ext_level: [u16; 18],
    pub port_ext_driven: [u16; 18],
}

impl ScuWdt {
    pub fn new(reset: ResetKind) -> Self {
        Self {
            rststat: reset.rststat(),
            osccon: 0,
            pllcon0: 0,
            pllcon1: 0,
            pll_findis: false,
            pllerco0: 0,
            pllerco1: 0,
            pller_findis: false,
            ccucon0: 0,
            ccucon1: 0,
            // State handed over to the user start address: WDT running in normal mode with the
            // reset reload value, ENDINIT set, password 0.
            wdt_rel: 0xFFFC,
            wdt_pw: 0,
            endinit: true,
            wdt_con1: 0,
            wdt_unlocked: false,
            wdt_mode: WdtMode::Normal,
            wdt_tim: 0xFFFC,
            wdt_is: false,
            wdt_ae: false,
            wdt_oe: false,
            wdt_frac: 0,
            wdt_prewarn_left: 0,
            wdt_access_errors: 0,
            wdt_services: 0,
            endinit_dropped: 0,
            last_endinit_drop: 0,
            port_ext_level: [0; 18],
            port_ext_driven: [0; 18],
        }
    }

    /// CPU clock from the PLL configuration (see `stm_icu` module docs).
    #[inline]
    pub fn fcpu_hz(&self) -> u64 {
        let (c0, c1) = (self.pllcon0 as u64, self.pllcon1 as u64);
        if c0 & 1 != 0 {
            FOSC_HZ / (((c1 >> 16) & 0x7F) + 1)
        } else {
            let n = ((c0 >> 9) & 0x7F) + 1;
            let p = ((c0 >> 24) & 0xF) + 1;
            let k2 = (c1 & 0x7F) + 1;
            FOSC_HZ * n / (p * k2)
        }
    }

    /// CCUCON0.FPIDIV: fFPI = fCPU / (FPIDIV + 1).
    #[inline]
    pub fn fpidiv(&self) -> u32 {
        self.ccucon0 & 0xF
    }

    /// Is ENDINIT currently set? Records the dropped write at `addr` when blocking.
    /// Used by variant-specific register blocks (`TC1784`/`TC1766`) that already verified `addr`
    /// is ENDINIT-protected on that MCU.
    pub fn endinit_active(&mut self, addr: u32) -> bool {
        if !self.endinit {
            return false;
        }
        self.endinit_dropped += 1;
        self.last_endinit_drop = addr;
        true
    }

    /// Is `addr` an ENDINIT-protected register on TC1791 and ENDINIT currently set?
    pub fn endinit_blocks(&mut self, addr: u32) -> bool {
        if !is_endinit_protected(addr) {
            return false;
        }
        self.endinit_active(addr)
    }

    pub(crate) fn con0_read(&self) -> u32 {
        (self.wdt_rel << 16)
            | (self.wdt_pw << 8)
            | if self.wdt_unlocked { 0 } else { 2 }
            | self.endinit as u32
    }

    pub(crate) fn sr_read(&self) -> u32 {
        (self.wdt_tim << 16)
            | ((self.wdt_mode == WdtMode::Prewarning) as u32) << 5
            | ((self.wdt_mode == WdtMode::TimeOut) as u32) << 4
            | ((self.wdt_mode == WdtMode::Disabled) as u32) << 3
            | (self.wdt_is as u32) << 2
            | (self.wdt_oe as u32) << 1
            | self.wdt_ae as u32
    }

    fn access_error(&mut self) {
        self.wdt_ae = true;
        self.wdt_access_errors += 1;
        self.enter_prewarning();
        self.wdt_unlocked = false;
    }

    fn enter_prewarning(&mut self) {
        if self.wdt_mode != WdtMode::Prewarning {
            self.wdt_mode = WdtMode::Prewarning;
            self.wdt_prewarn_left = WDT_PREWARNING_COUNTS;
        }
    }

    pub(crate) fn con0_write(&mut self, v: u32) {
        let hpw1 = (v >> 4) & 0xF;
        let hpw0 = (v >> 2) & 3;
        let lck = v & 2 != 0;
        if self.wdt_mode == WdtMode::Prewarning {
            return; // a reset is pending; accesses have no effect
        }
        if !self.wdt_unlocked {
            let pw = (v >> 8) & 0xFF;
            if !lck && hpw1 == 0xF && hpw0 == (self.wdt_con1 >> 2) & 3 && pw == self.wdt_pw {
                self.wdt_unlocked = true;
            } else {
                self.access_error();
            }
            return;
        }
        // modify access
        self.wdt_unlocked = false;
        if !lck || hpw1 != 0xF {
            self.access_error();
            return;
        }
        self.wdt_rel = v >> 16;
        self.wdt_pw = (v >> 8) & 0xFF;
        let new_endinit = v & 1 != 0;
        if new_endinit {
            // service / leave time-out mode
            self.wdt_services += 1;
            self.wdt_is = self.wdt_con1 & 4 != 0;
            self.wdt_ae = false;
            self.wdt_oe = false;
            if self.wdt_con1 & 8 != 0 {
                self.wdt_mode = WdtMode::Disabled;
            } else {
                self.wdt_mode = WdtMode::Normal;
                self.wdt_tim = self.wdt_rel;
            }
        } else if self.endinit || self.wdt_mode != WdtMode::TimeOut {
            self.wdt_mode = WdtMode::TimeOut;
            self.wdt_tim = 0xFFFC;
        }
        self.endinit = new_endinit;
    }

    /// Advance the WDT by `insns` CPU instructions; returns `true` when a WDT reset is due.
    pub fn wdt_advance(&mut self, insns: u64) -> bool {
        if self.wdt_mode == WdtMode::Disabled {
            return false;
        }
        let div: u64 = if self.wdt_is { 256 } else { 16384 };
        // WDT counts = CPU cycles / ((FPIDIV+1) * div)
        let total = self.wdt_frac + insns * CYCLES_PER_INSN;
        let unit = (self.fpidiv() as u64 + 1) * div;
        let mut counts = total / unit;
        self.wdt_frac = total % unit;
        while counts > 0 {
            if self.wdt_mode == WdtMode::Prewarning {
                let step = counts.min(self.wdt_prewarn_left as u64) as u32;
                self.wdt_prewarn_left -= step;
                self.wdt_tim = (self.wdt_tim + step) & 0xFFFF;
                if self.wdt_prewarn_left == 0 {
                    return true;
                }
                counts -= step as u64;
                continue;
            }
            let to_overflow = 0x1_0000 - self.wdt_tim as u64;
            if counts < to_overflow {
                self.wdt_tim += counts as u32;
                counts = 0;
            } else {
                counts -= to_overflow;
                self.wdt_tim = 0;
                self.wdt_oe = true;
                self.enter_prewarning();
            }
        }
        false
    }
}

/// ENDINIT-protected peripheral registers (AUDO family): module clock control registers of every
/// module the firmware enables, the SCU clock-generation registers, WDT_CON1 and the port pad
/// driver registers. Writes while ENDINIT = 1 are ignored by hardware.
#[inline]
pub fn is_endinit_protected(addr: u32) -> bool {
    match addr {
        0xF000_0200
        | 0xF000_0800
        | 0xF000_0900
        | 0xF000_0A00
        | 0xF000_1800
        | 0xF000_3C00
        | 0xF000_4000
        | 0xF004_3F00
        | 0xF010_0400
        | 0xF010_1000
        | 0xF010_1400
        | 0xF010_1800
        | 0xF031_0000
        | 0xF031_0100
        | 0xF031_0200
        | 0xF031_0300
        | 0xF032_1000
        | 0xF000_0510
        | 0xF000_0518
        | 0xF000_051C
        | 0xF000_0528
        | 0xF000_052C
        | 0xF000_0530
        | 0xF000_0534
        | WDT_CON1 => true,
        0xF000_0C00..=0xF000_17FF | 0xF030_0000..=0xF030_05FF => matches!(addr & 0xFF, 0x40 | 0x44),
        _ => false,
    }
}

/// Port index for a port register address (P0..P11 @ 0xF0000C00, P12..P17 @ 0xF0300000).
fn port_index(addr: u32) -> Option<usize> {
    match addr {
        0xF000_0C00..=0xF000_17FF => Some(((addr - 0xF000_0C00) >> 8) as usize),
        0xF030_0000..=0xF030_05FF => Some(12 + ((addr - 0xF030_0000) >> 8) as usize),
        _ => None,
    }
}

impl Tc1791HardwareState {
    pub(crate) fn scu_read(&self, addr: u32) -> Option<u32> {
        let s = &self.scu;
        Some(match addr {
            0xF000_0510 => s.osccon | 0x0000_0102, // PLLLV | PLLHV: oscillator in PLL input range
            0xF000_0514 => pll_stat(s.pllcon0, s.pll_findis),
            0xF000_0518 => s.pllcon0 & !0x30,
            0xF000_051C => s.pllcon1,
            0xF000_0524 => pll_stat(s.pllerco0, s.pller_findis),
            0xF000_0528 => s.pllerco0 & !0x30,
            0xF000_052C => s.pllerco1,
            0xF000_0530 => s.ccucon0,
            0xF000_0534 => s.ccucon1,
            SCU_RSTSTAT => s.rststat,
            SCU_SWRSTCON => 0,
            WDT_CON0 => s.con0_read(),
            WDT_CON1 => s.wdt_con1,
            WDT_SR => s.sr_read(),
            SCU_CHIPID => CHIPID_VALUE,
            _ => {
                let p = port_index(addr)?;
                if addr & 0xFF == 0x24 {
                    return Some(self.port_in(p, addr));
                }
                if addr & 0xFF == 0x04 {
                    return Some(0); // OMR is write-only
                }
                return None;
            }
        })
    }

    pub(crate) fn scu_write(&mut self, addr: u32, val: u32, mask: u32) -> bool {
        if (0xF000_0500..0xF000_0700).contains(&addr) && self.scu.endinit_blocks(addr) {
            return true;
        }
        let merge = |old: u32| (old & !mask) | (val & mask);
        match addr {
            0xF000_0510 => self.scu.osccon = merge(self.scu.osccon),
            0xF000_0518 => {
                let v = merge(self.scu.pllcon0);
                if v & 0x10 != 0 {
                    self.scu.pll_findis = true;
                }
                if v & 0x20 != 0 {
                    self.scu.pll_findis = false;
                }
                self.scu.pllcon0 = v & !0x30;
            }
            0xF000_051C => self.scu.pllcon1 = merge(self.scu.pllcon1),
            0xF000_0528 => {
                let v = merge(self.scu.pllerco0);
                if v & 0x10 != 0 {
                    self.scu.pller_findis = true;
                }
                if v & 0x20 != 0 {
                    self.scu.pller_findis = false;
                }
                self.scu.pllerco0 = v & !0x30;
            }
            0xF000_052C => self.scu.pllerco1 = merge(self.scu.pllerco1),
            0xF000_0530 => {
                // FPIDIV changes the STM rate: settle elapsed time at the old rate first.
                self.settle_stm();
                self.scu.ccucon0 = merge(self.scu.ccucon0);
                self.stm.fpidiv = self.scu.fpidiv();
                self.stop_requested = true;
            }
            0xF000_0534 => self.scu.ccucon1 = merge(self.scu.ccucon1),
            0xF000_0514 | 0xF000_0524 | SCU_RSTSTAT | SCU_CHIPID | WDT_SR => {}
            SCU_SWRSTCON => {
                if val & mask & 2 != 0 {
                    self.reset_request = Some(ResetKind::Software);
                    if self.swrst_log.len() < 64 {
                        self.swrst_log.push(self.last_tb_pc);
                    }
                    self.stop_requested = true;
                }
            }
            WDT_CON0 => {
                // Byte-wise writes cannot form a valid password sequence on hardware either.
                let v = if mask == 0xFFFF_FFFF { val } else { merge(self.scu.con0_read()) };
                self.scu.con0_write(v);
            }
            WDT_CON1 => self.scu.wdt_con1 = merge(self.scu.wdt_con1) & 0xC,
            _ => {
                let Some(p) = port_index(addr) else { return false };
                match addr & 0xFF {
                    0x04 => {
                        // OMR: PSx (bits 15:0) set, PCx (31:16) clear; both -> toggle.
                        let v = val & mask;
                        let set = v & 0xFFFF;
                        let clr = v >> 16;
                        let base = addr & !0xFF;
                        let mut out = self.port_reg(base);
                        out ^= set & clr;
                        out |= set & !clr;
                        out &= !(clr & !set);
                        self.port_reg_set(base, out & 0xFFFF);
                        return true;
                    }
                    0x24 => return true, // IN is read-only
                    0x40 | 0x44 => {
                        if self.scu.endinit_blocks(addr) {
                            return true;
                        }
                        let _ = p;
                        return false;
                    }
                    _ => return false,
                }
            }
        }
        true
    }

    fn port_reg(&self, a: u32) -> u32 {
        self.seg_f000.get(a)
    }

    fn port_reg_set(&mut self, a: u32, v: u32) {
        self.seg_f000.set(a, v);
    }

    /// Pn_IN: outputs read back the output latch, inputs read the external level or the
    /// configured pull device (floating inputs read 1).
    pub(crate) fn port_in(&self, p: usize, addr: u32) -> u32 {
        let base = addr & !0xFF;
        let out = self.seg_f000.get(base);
        let mut v = 0u32;
        for pin in 0..16u32 {
            let iocr = self.seg_f000.get(base + 0x10 + (pin / 4) * 4);
            let pc = (iocr >> ((pin % 4) * 8 + 4)) & 0xF;
            let bit = if pc & 0x8 != 0 {
                (out >> pin) & 1
            } else if self.scu.port_ext_driven[p] & (1 << pin) != 0 {
                ((self.scu.port_ext_level[p] >> pin) & 1) as u32
            } else if pc & 0x3 == 0x1 {
                0 // pull-down
            } else {
                1 // pull-up or floating
            };
            v |= bit << pin;
        }
        v
    }
}

fn pll_stat(con0: u32, findis: bool) -> u32 {
    let vcobyp = con0 & 1;
    let pwd = (con0 >> 1) & 1;
    let lock = (!findis && pwd == 0) as u32;
    vcobyp | (pwd << 1) | (lock << 2) | ((findis as u32) << 3) | 0x30
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unlock(w: &mut ScuWdt) {
        let pw = (w.con0_read() & 0xFFFF_FF01) | 0xF0 | (w.wdt_con1 & 0xC);
        w.con0_write(pw);
    }

    #[test]
    fn wdt_password_then_modify_sequence() {
        let mut w = ScuWdt::new(ResetKind::PowerOn);
        unlock(&mut w);
        assert!(w.wdt_unlocked);
        assert_eq!(w.con0_read() & 2, 0, "LCK reads 0 while unlocked");
        // modify: REL = 0xFF86, ENDINIT = 0 -> time-out mode
        w.con0_write(0xFF86_00F2);
        assert!(!w.endinit);
        assert_eq!(w.wdt_mode, WdtMode::TimeOut);
        assert_eq!(w.wdt_tim, 0xFFFC);
        unlock(&mut w);
        w.con0_write(0xFF86_00F3);
        assert!(w.endinit);
        assert_eq!(w.wdt_mode, WdtMode::Normal);
        assert_eq!(w.wdt_tim, 0xFF86);
        assert_eq!(w.wdt_access_errors, 0);
    }

    #[test]
    fn wdt_wrong_password_is_access_error_and_resets() {
        let mut w = ScuWdt::new(ResetKind::PowerOn);
        w.con0_write(0xFFFC_55F0); // wrong WDTPW
        assert!(w.wdt_ae);
        assert_eq!(w.wdt_mode, WdtMode::Prewarning);
        // the prewarning period ends in a reset
        let insns = (WDT_PREWARNING_COUNTS as u64) * 16384 + 10;
        assert!(w.wdt_advance(insns));
    }

    #[test]
    fn wdt_timeout_without_service_resets() {
        let mut w = ScuWdt::new(ResetKind::PowerOn);
        // 4 counts to overflow from 0xFFFC at fFPI/16384
        let per_count = 16384; // FPIDIV = 0 after reset
        assert!(!w.wdt_advance(3 * per_count));
        assert!(!w.wdt_advance(per_count + 1));
        assert!(w.wdt_oe);
        assert_eq!(w.wdt_mode, WdtMode::Prewarning);
        assert!(w.wdt_advance(WDT_PREWARNING_COUNTS as u64 * per_count));
    }

    #[test]
    fn endinit_protects_clc() {
        let mut w = ScuWdt::new(ResetKind::PowerOn);
        for &clc in &[
            0xF000_0200, // STM_CLC
            0xF000_0800, // MSC0_CLC
            0xF000_0900, // MSC1_CLC
            0xF000_3C00, // DMA_CLC
            0xF031_0000, // SSC0_CLC
            0xF031_0100, // SSC1_CLC
            0xF031_0200, // SSC2_CLC
            0xF031_0300, // SSC3_CLC
        ] {
            let before = w.endinit_dropped;
            assert!(is_endinit_protected(clc), "0x{clc:08X} must be ENDINIT-protected");
            assert_eq!(w.endinit_dropped, before, "is_endinit_protected must not increment counter");
            assert!(w.endinit_blocks(clc), "0x{clc:08X} must block under ENDINIT=1");
        }
        assert_eq!(w.endinit_dropped, 8);
        assert!(!w.endinit_blocks(0xF000_0230));
        unlock(&mut w);
        w.con0_write(0xFFFC_00F2);
        assert!(!w.endinit_blocks(0xF000_3C00));
        assert!(!w.endinit_blocks(0xF031_0300));
    }
}
