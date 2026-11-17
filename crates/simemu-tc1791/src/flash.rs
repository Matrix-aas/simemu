//! Flash command interface (PMU/FLASHx command state machine).
//!
//! The flash arrays are mapped read/execute-only and marked read-only in Unicorn, so guest stores
//! never modify array content directly: every store is delivered to `Tc1791HardwareState::flash_command`
//! (via `UC_HOOK_MEM_WRITE_PROT` on Unicorn, or `HwBus::write` on `tricore-core`) and discarded by
//! the TLB. Command cycles follow the AUDO flash command set of the TC1796 UM V2.0 §7.2.5
//! (p.7-14..7-25, Tables 7-4..7-15; no TC1791 user manual is available — the SC800S50 driver issues
//! exactly these cycles, VERIFIED-T). Addresses are relative to the array base: PFlash0 0xA0000000,
//! PFlash1 0xA0800000, DFlash bank 0 0xAF000000, bank 1 0xAF080000 (the TC1796 has its DFlash banks
//! at 0xAFE00000/0xAFE10000):
//!
//! | command                  | cycles                                                   | UM                   |
//! |--------------------------|----------------------------------------------------------|----------------------|
//! | Reset to Read            | 5554:F0 (at any time, also inside a sequence)            | p.7-15               |
//! | Clear Status             | 5554:F5                                                  | p.7-25               |
//! | Enter Page Mode          | 5554:50 (PFlash) / 5554:5D (DFlash)                      | p.7-15               |
//! | Load Page                | 55F0:data (low word), 55F4:data (high word)              | p.7-16               |
//! | Write Page               | 5554:AA, AAA8:55, 5554:A0, PA:AA                         | p.7-18               |
//! | Erase Sector             | 5554:AA, AAA8:55, 5554:80, 5554:AA, AAA8:55, SA:30 / 40  | p.7-20               |
//! | Disable Write Protection | 5554:AA, AAA8:55, 553C:UCB, AAA8:PW0, AAA8:PW1, 5558:05  | p.7-23 (Table 7-12)  |
//! | Disable Read Protection  | 5554:AA, AAA8:55, 553C:00, AAA8:PW0, AAA8:PW1, 5558:08   | p.7-24 (Table 7-13)  |
//! | Resume Protection        | 5554:5E                                                  | p.7-24 (Table 7-14)  |
//!
//! Erased flash reads as 0x00 (consistent with the unused area of the DFlash dump); programming
//! sets bits. DFlash page = 128 bytes; erase unit = one 96 KB DFlash bank (the TC1796 erases a whole
//! DFLASH bank, UM p.7-20; the TC1791 bank size is UNVERIFIED). PFlash page = 256 bytes; logical
//! sectors are 8×16 KB + 128 KB + 7×256 KB per 2 MB bank (TC1791_CAN_BSL `sector_map_tc1791`, code
//! and the CBOOT driver at 0x8003494E, VERIFIED-T; the physical-sector map for SA:40 is UNVERIFIED).
//! Read unlock = UCB 0 + 08H, write unlock = UCB 0/1 + 05H (UM Tables 7-12/7-13 and SBOOT
//! 0x800098F8, VERIFIED-T). Erased UCBs give no installed protection, the default here. A bench
//! snapshot is separate because its BIN has no physical UCB dump: `from_image` uses the UID and
//! recovered passwords at 0x80014200..1B and the PROCON0 value saved by SBOOT at D001F594.
//! Its installed-but-disabled read protection is UNVERIFIED against BROM and a UCB readout.

use crate::memory::Tc1791HardwareState;
use std::collections::VecDeque;

pub const DFLASH_PAGE: usize = 128;
pub const PFLASH_PAGE: usize = 256;
pub const PFLASH_BANK: usize = 0x20_0000;
pub const DFLASH_BANK: usize = 0x1_8000;
/// UNVERIFIED (see module docs).
pub const DFLASH_ERASE_UNIT: usize = DFLASH_BANK;

// FLASH_FSR bit positions (TC1796 UM V2.0 §7.2.11.2, p.7-43..7-49).
pub const FSR_PBUSY: u32 = 1 << 0;
pub const FSR_PROG: u32 = 1 << 4;
pub const FSR_ERASE: u32 = 1 << 5;
pub const FSR_PFPAGE: u32 = 1 << 6;
pub const FSR_DFPAGE: u32 = 1 << 7;
pub const FSR_PFOPER: u32 = 1 << 8;
pub const FSR_DFOPER: u32 = 1 << 9;
pub const FSR_SQER: u32 = 1 << 10;
pub const FSR_PROER: u32 = 1 << 11;
pub const FSR_PROIN: u32 = 1 << 16;
pub const FSR_RPROIN: u32 = 1 << 18;
pub const FSR_RPRODIS: u32 = 1 << 19;
pub const FSR_WPROIN0: u32 = 1 << 21;
pub const FSR_WPROIN1: u32 = 1 << 22;
pub const FSR_WPROIN2: u32 = 1 << 23;
pub const FSR_WPRODIS0: u32 = 1 << 25;
pub const FSR_WPRODIS1: u32 = 1 << 26;
pub const FCON_RPA: u32 = 1 << 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashArray {
    PFlash0,
    PFlash1,
    DFlash,
}

#[derive(Clone, Copy)]
struct ProtectionReset {
    fsr: [u32; 2],
    procon: [[u32; 3]; 2],
    passwords: [Option<[u32; 2]>; 2],
}

pub struct FlashCtrl {
    /// FSR of FLASH0 (PFlash0 + DFlash) and FLASH1 (PFlash1).
    pub fsr: [u32; 2],
    pub fcon: [u32; 2],
    /// FLASH0 MARP (`[0]`, 0xF8002018) and MARD (`[1]`, 0xF800201C), reset 0x0000_8000 (UM p.7-51/7-52).
    pub margin: [u32; 2],
    /// FLASH1 MARP (0xF8004018), reset 0x0000_8000.
    pub marp1: u32,
    pub procon: [[u32; 3]; 2],
    reset_protection: ProtectionReset,
    password_rejected: [bool; 2],
    seq: [(u32, u32); 6],
    seq_len: u8,
    seq_array: Option<FlashArray>,
    page_mode: Option<FlashArray>,
    page_buf: Vec<u8>,
    pub dflash_pages_programmed: u64,
    pub dflash_erases: u64,
    pub pflash_pages_programmed: u64,
    pub pflash_erases: u64,
    pub sequence_errors: u64,
    pub commands: u64,
    /// Bounded host-side trace `(address, size, value)`; never readable or writable by the guest.
    pub cycles: VecDeque<(u32, u32, u32)>,
}

impl Default for FlashCtrl {
    fn default() -> Self {
        let reset_protection = ProtectionReset { fsr: [0; 2], procon: [[0; 3]; 2], passwords: [None; 2] };
        Self {
            fsr: reset_protection.fsr,
            fcon: [0x0007_4F08; 2],
            margin: [0x0000_8000; 2],
            marp1: 0x0000_8000,
            procon: reset_protection.procon,
            reset_protection,
            password_rejected: [false; 2],
            seq: [(0, 0); 6],
            seq_len: 0,
            seq_array: None,
            page_mode: None,
            page_buf: Vec::new(),
            dflash_pages_programmed: 0,
            dflash_erases: 0,
            pflash_pages_programmed: 0,
            pflash_erases: 0,
            sequence_errors: 0,
            commands: 0,
            cycles: VecDeque::new(),
        }
    }
}

impl FlashCtrl {
    pub fn from_image(image: &[u8]) -> Self {
        let mut fc = Self::default();
        let Some(uid) = image.get(0x1_4200..0x1_420C) else { return fc };
        if uid.iter().all(|&b| b == 0 || b == 0xFF) {
            return fc;
        }
        let (Some(pw0), Some(pw1)) = (image.get(0x1_420C..0x1_4210), image.get(0x1_4210..0x1_4214)) else {
            return fc;
        };
        let pw0 = u32::from_le_bytes(pw0.try_into().unwrap());
        let pw1 = u32::from_le_bytes(pw1.try_into().unwrap());
        fc.reset_protection = ProtectionReset {
            fsr: [FSR_PROIN | FSR_RPROIN | FSR_RPRODIS; 2],
            procon: [[0x8000_0000, 0, 0]; 2],
            passwords: [Some([pw0, pw1]), None],
        };
        fc.reset();
        fc
    }

    pub fn reset(&mut self) {
        let p = self.reset_protection;
        *self = Self { fsr: p.fsr, procon: p.procon, reset_protection: p, ..Self::default() };
    }

    pub fn debug_page_state(&self) -> (Option<FlashArray>, usize, Vec<(u32, u32)>) {
        (self.page_mode, self.page_buf.len(), self.seq[..self.seq_len as usize].to_vec())
    }

    fn module(a: FlashArray) -> usize {
        if a == FlashArray::PFlash1 { 1 } else { 0 }
    }

    fn pflash_locked(&self, array: FlashArray, off: u32) -> bool {
        let m = Self::module(array);
        if self.fsr[m] & FSR_RPROIN != 0 && self.fsr[m] & FSR_RPRODIS == 0 {
            return true;
        }
        let sector = match off {
            0..=0x3FFF => 0,
            0x4000..=0x7FFF => 1,
            0x8000..=0xBFFF => 2,
            0xC000..=0xFFFF => 3,
            0x10000..=0x13FFF => 4,
            0x14000..=0x17FFF => 5,
            0x18000..=0x1BFFF => 6,
            0x1C000..=0x1FFFF => 7,
            0x20000..=0x3FFFF => 8,
            0x40000..=0x7FFFF => 9,
            0x80000..=0xBFFFF => 10,
            0xC0000..=0xFFFFF => 11,
            0x100000..=0x13FFFF => 12,
            0x140000..=0x17FFFF => 13,
            0x180000..=0x1BFFFF => 14,
            0x1C0000..=0x1FFFFF => 15,
            _ => usize::MAX,
        };
        let lock = |user| self.procon[m][user] & (1 << sector.min(15)) != 0;
        self.fsr[m] & FSR_WPROIN2 != 0 && lock(2)
            || self.fsr[m] & FSR_WPROIN0 != 0 && lock(0) && self.fsr[m] & FSR_WPRODIS0 == 0
            || self.fsr[m] & FSR_WPROIN1 != 0 && lock(1) && self.fsr[m] & (FSR_WPRODIS0 | FSR_WPRODIS1) == 0
    }
}

impl Tc1791HardwareState {
    /// Classify a guest flash address: `(array, offset from array base)`.
    fn flash_locate(&self, addr: u32) -> Option<(FlashArray, u32)> {
        match addr {
            0x8000_0000..=0x801F_FFFF => Some((FlashArray::PFlash0, addr - 0x8000_0000)),
            0xA000_0000..=0xA01F_FFFF => Some((FlashArray::PFlash0, addr - 0xA000_0000)),
            0x8080_0000..=0x809F_FFFF => Some((FlashArray::PFlash1, addr - 0x8080_0000)),
            0xA080_0000..=0xA09F_FFFF => Some((FlashArray::PFlash1, addr - 0xA080_0000)),
            0xAF00_0000..=0xAF0F_FFFF => Some((FlashArray::DFlash, addr - 0xAF00_0000)),
            _ => None,
        }
    }

    /// DFlash image offset for an array offset (bank 0 @ +0, bank 1 @ +0x80000).
    fn dflash_index(off: u32) -> Option<usize> {
        let off = off as usize;
        match off {
            0..=0x17FFF => Some(off),
            0x8_0000..=0x9_7FFF => Some(DFLASH_BANK + off - 0x8_0000),
            _ => None,
        }
    }

    /// Mark both cached (`0x80..`) and non-cached (`0xA0..`) PFlash views dirty for code-cache
    /// invalidation and end the active quantum before any translated block in the modified range runs.
    fn note_pflash_modified(&mut self, array: FlashArray, start: usize, len: usize) {
        let bank_off = if array == FlashArray::PFlash1 { 0x0080_0000u32 } else { 0 };
        let start = start as u32;
        let len = len as u32;
        for seg in [0x8000_0000u32, 0xA000_0000] {
            self.pflash_code_dirty.push((seg + bank_off + start, len));
        }
        self.tb_cache.fill(crate::cpu_hooks::TbCacheEntry::EMPTY);
        self.stop_requested = true;
    }

    /// A guest store hit a flash array: interpret it as a command cycle.
    pub(crate) fn flash_command(&mut self, addr: u32, size: u32, value: u64) {
        let Some((array, off)) = self.flash_locate(addr) else { return };
        let fc = &mut self.flashc;
        fc.commands += 1;
        let m = FlashCtrl::module(array);
        let data = value as u32;
        let cmd_byte = data & 0xFF;
        let cyc = off & 0xFFFF;
        if fc.cycles.len() == 4096 {
            fc.cycles.pop_front();
        }
        fc.cycles.push_back((addr, size, data));

        // Reset to Read (UM p.7-15): also inside a command sequence, which it aborts without a
        // sequence error; aborts Page Mode and clears all error flags (busy flags unaffected).
        if cyc == 0x5554 && cmd_byte == 0xF0 {
            fc.seq_len = 0;
            fc.seq_array = None;
            fc.page_mode = None;
            fc.fsr[m] &= !(FSR_PFPAGE | FSR_DFPAGE | FSR_PFOPER | FSR_DFOPER | FSR_SQER | FSR_PROER | 0xF000);
            return;
        }
        // Single-cycle commands
        if fc.seq_len == 0 && cyc == 0x5554 {
            match cmd_byte {
                0xF5 => {
                    fc.fsr[m] &= !(FSR_PROG | FSR_ERASE | FSR_PFOPER | FSR_DFOPER | FSR_SQER | FSR_PROER | 0xF000);
                    return;
                }
                // UM Table 7-14: resume both protections, never their installed flags.
                0x5E => {
                    if array != FlashArray::DFlash {
                        fc.fsr[m] &= !(FSR_RPRODIS | FSR_WPRODIS0 | FSR_WPRODIS1);
                    }
                    return;
                }
                0x50 | 0x5D => {
                    let want = if cmd_byte == 0x5D { FlashArray::DFlash } else { array };
                    fc.page_mode = Some(want);
                    fc.page_buf.clear();
                    fc.fsr[m] |= if want == FlashArray::DFlash { FSR_DFPAGE } else { FSR_PFPAGE };
                    return;
                }
                _ => {}
            }
        }
        // Load page
        if fc.page_mode.is_some() && (cyc == 0x55F0 || cyc == 0x55F4) {
            let n = (size as usize).min(8).min(PFLASH_PAGE.saturating_sub(fc.page_buf.len()));
            fc.page_buf.extend_from_slice(&value.to_le_bytes()[..n]);
            return;
        }

        // UM §7.2.5: all cycles of a multi-cycle command sequence must target the same Flash array/module.
        if fc.seq_len > 0 && fc.seq_array != Some(array) {
            let prev_m = FlashCtrl::module(fc.seq_array.unwrap_or(array));
            fc.seq_len = 0;
            fc.seq_array = None;
            fc.fsr[prev_m] |= FSR_SQER;
            fc.fsr[m] |= FSR_SQER;
            fc.sequence_errors += 1;
            return;
        }
        fc.seq_array = Some(array);
        let idx = fc.seq_len as usize;
        if idx < fc.seq.len() {
            fc.seq[idx] = (cyc, data);
            fc.seq_len += 1;
        } else {
            fc.seq_len = 0;
            fc.seq_array = None;
            fc.fsr[m] |= FSR_SQER;
            fc.sequence_errors += 1;
            return;
        }
        const WRITE: [(u32, u32); 3] = [(0x5554, 0xAA), (0xAAA8, 0x55), (0x5554, 0xA0)];
        const ERASE: [(u32, u32); 5] = [(0x5554, 0xAA), (0xAAA8, 0x55), (0x5554, 0x80), (0x5554, 0xAA), (0xAAA8, 0x55)];
        let seq = fc.seq;
        let n = fc.seq_len as usize;
        let cmd_matches = |slice: &[(u32, u32)], pat: &[(u32, u32)]| {
            slice.len() == pat.len() && slice.iter().zip(pat).all(|(&(c, d), &(pc, pd))| c == pc && (d & 0xFF) == pd)
        };
        let prefix_ok = |pat: &[(u32, u32)]| n <= pat.len() + 1 && cmd_matches(&seq[..n.min(pat.len())], &pat[..n.min(pat.len())]);
        let unlock_prefix = n >= 3
            && cmd_matches(&seq[..2], &[(0x5554, 0xAA), (0xAAA8, 0x55)])
            && seq[2].0 == 0x553C
            && (n == 3 || seq[3].0 == 0xAAA8)
            && (n <= 4 || seq[4].0 == 0xAAA8)
            && (n <= 5 || seq[5].0 == 0x5558);
        if unlock_prefix {
            if n == 6 {
                fc.seq_len = 0;
                fc.seq_array = None;
                self.flash_unlock(array, seq[2].1 & 0xFF, seq[3].1, seq[4].1, seq[5].1 & 0xFF);
            }
            return;
        }
        if n == 4 && cmd_matches(&seq[..3], &WRITE) && (seq[3].1 & 0xFF) == 0xAA {
            fc.seq_len = 0;
            fc.seq_array = None;
            self.flash_write_page(array, off);
        } else if n == 6 && cmd_matches(&seq[..5], &ERASE) {
            fc.seq_len = 0;
            fc.seq_array = None;
            self.flash_erase(array, off, seq[5].1 & 0xFF);
        } else if !(prefix_ok(&WRITE) || prefix_ok(&ERASE)) {
            fc.seq_len = 0;
            fc.seq_array = None;
            fc.fsr[m] |= FSR_SQER;
            fc.sequence_errors += 1;
        }
    }

    fn flash_unlock(&mut self, array: FlashArray, ucb: u32, pw0: u32, pw1: u32, mode: u32) {
        let m = FlashCtrl::module(array);
        let (user, mask) = match (ucb, mode) {
            (0, 0x08) => (0, FSR_RPRODIS),
            (0, 0x05) => (0, FSR_WPRODIS0),
            (1, 0x05) => (1, FSR_WPRODIS1),
            _ => {
                self.flashc.fsr[m] |= FSR_PROER;
                return;
            }
        };
        if self.flashc.password_rejected[user] {
            self.flashc.fsr[m] |= FSR_PROER;
            return;
        }
        if self.flashc.reset_protection.passwords[user] == Some([pw0, pw1]) {
            self.flashc.fsr[m] = (self.flashc.fsr[m] & !FSR_PROER) | mask;
        } else {
            self.flashc.fsr[m] |= FSR_PROER;
            self.flashc.password_rejected[user] = true;
        }
    }

    fn flash_write_page(&mut self, array: FlashArray, off: u32) {
        let m = FlashCtrl::module(array);
        let buf = std::mem::take(&mut self.flashc.page_buf);
        let mode = self.flashc.page_mode.take();
        self.flashc.fsr[m] &= !(FSR_DFPAGE | FSR_PFPAGE | FSR_PBUSY);
        if mode != Some(array) {
            self.flashc.fsr[m] |= FSR_SQER;
            self.flashc.sequence_errors += 1;
            return;
        }
        // UM p.7-19: if read/write protection is active for the sector, Page Mode is terminated,
        // programming is not started, and FSR.PROER is set.
        if array != FlashArray::DFlash && self.flashc.pflash_locked(array, off) {
            self.flashc.fsr[m] |= FSR_PROER;
            return;
        }
        match array {
            FlashArray::DFlash => {
                let page = off & !(DFLASH_PAGE as u32 - 1);
                let Some(idx) = Self::dflash_index(page) else {
                    self.flashc.fsr[m] |= FSR_SQER;
                    return;
                };
                let Some(dst) = self.dflash.get_mut(idx..idx + DFLASH_PAGE) else {
                    self.flashc.fsr[m] |= FSR_SQER;
                    return;
                };
                for (d, s) in dst.iter_mut().zip(buf.iter()) {
                    *d |= *s;
                }
                self.flashc.dflash_pages_programmed += 1;
            }
            FlashArray::PFlash0 | FlashArray::PFlash1 => {
                let page = off & !(PFLASH_PAGE as u32 - 1);
                let base = if array == FlashArray::PFlash1 { PFLASH_BANK } else { 0 };
                let idx = base + page as usize;
                if let Some(dst) = self.flash.get_mut(idx..idx + PFLASH_PAGE) {
                    for (d, s) in dst.iter_mut().zip(buf.iter()) {
                        *d |= *s;
                    }
                } else {
                    self.flashc.fsr[m] |= FSR_SQER;
                    return;
                }
                self.flashc.pflash_pages_programmed += 1;
                self.note_pflash_modified(array, page as usize, PFLASH_PAGE);
            }
        }
        self.flashc.fsr[m] |= FSR_PROG;
    }

    fn pflash_sector(off: u32) -> Option<(usize, usize)> {
        const SECTORS: [u32; 16] = [
            0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000,
            0x20000,
            0x40000, 0x40000, 0x40000, 0x40000, 0x40000, 0x40000, 0x40000,
        ];
        let mut start = 0u32;
        for size in SECTORS {
            if off >= start && off < start + size {
                return Some((start as usize, size as usize));
            }
            start += size;
        }
        None
    }

    /// Last cycle of Erase Sector: `SA:40` erases a DFlash bank (UM Table 7-10), `SA:30` / `SA:40`
    /// erases a PFlash sector.
    fn flash_erase(&mut self, array: FlashArray, off: u32, data: u32) {
        let m = FlashCtrl::module(array);
        if array != FlashArray::DFlash && self.flashc.pflash_locked(array, off) {
            self.flashc.fsr[m] |= FSR_PROER;
            return;
        }
        match array {
            FlashArray::DFlash if data == 0x40 => {
                let Some(idx) = Self::dflash_index(off) else {
                    self.flashc.fsr[m] |= FSR_SQER;
                    return;
                };
                let start = idx / DFLASH_ERASE_UNIT * DFLASH_ERASE_UNIT;
                let Some(dst) = self.dflash.get_mut(start..start + DFLASH_ERASE_UNIT) else {
                    self.flashc.fsr[m] |= FSR_SQER;
                    return;
                };
                dst.fill(0);
                self.flashc.dflash_erases += 1;
            }
            FlashArray::PFlash0 | FlashArray::PFlash1 if data == 0x30 || data == 0x40 => {
                let Some((start, size)) = Self::pflash_sector(off) else {
                    self.flashc.fsr[m] |= FSR_SQER;
                    return;
                };
                let base = if array == FlashArray::PFlash1 { PFLASH_BANK } else { 0 };
                if let Some(dst) = self.flash.get_mut(base + start..base + start + size) {
                    dst.fill(0);
                } else {
                    self.flashc.fsr[m] |= FSR_SQER;
                    return;
                }
                self.flashc.pflash_erases += 1;
                self.note_pflash_modified(array, start, size);
            }
            _ => {
                self.flashc.fsr[m] |= FSR_SQER;
                self.flashc.sequence_errors += 1;
                return;
            }
        }
        self.flashc.fsr[m] |= FSR_ERASE;
    }

    /// FLASH0/FLASH1 control, margin, protection and status register reads.
    pub(crate) fn flash_reg_read(&self, addr: u32) -> Option<u32> {
        let m = usize::from(addr >= 0xF800_4000);
        let rpa = (self.flashc.fsr[m] & FSR_RPROIN != 0 && self.flashc.fsr[m] & FSR_RPRODIS == 0) as u32 * FCON_RPA;
        match addr {
            0xF800_2010 => Some(self.flashc.fsr[0]),
            0xF800_4010 => Some(self.flashc.fsr[1]),
            0xF800_2014 | 0xF800_4014 => Some((self.flashc.fcon[m] & !FCON_RPA) | rpa),
            0xF800_2018 => Some(self.flashc.margin[0]),
            0xF800_201C => Some(self.flashc.margin[1]),
            0xF800_4018 => Some(self.flashc.marp1),
            0xF800_2020 => Some(self.flashc.procon[0][0]),
            0xF800_2024 => Some(self.flashc.procon[0][1]),
            0xF800_2028 => Some(self.flashc.procon[0][2]),
            0xF800_4020 => Some(self.flashc.procon[1][0]),
            0xF800_4024 => Some(self.flashc.procon[1][1]),
            0xF800_4028 => Some(self.flashc.procon[1][2]),
            _ => None,
        }
    }

    pub(crate) fn flash_reg_write(&mut self, addr: u32, value: u32, mask: u32) -> bool {
        match addr {
            0xF800_2018 | 0xF800_201C => {
                let index = usize::from(addr == 0xF800_201C);
                let allowed = if index == 0 { 0x0000_800F } else { 0x0000_801F };
                let mask = mask & allowed;
                self.flashc.margin[index] = (self.flashc.margin[index] & !mask) | (value & mask);
                return true;
            }
            0xF800_4018 => {
                let mask = mask & 0x0000_800F;
                self.flashc.marp1 = (self.flashc.marp1 & !mask) | (value & mask);
                return true;
            }
            0xF800_2014 | 0xF800_4014 => {}
            _ => return false,
        }
        let m = usize::from(addr >= 0xF800_4000);
        let old = self.flashc.fcon[m];
        let mut new = (old & !mask) | (value & mask & !FCON_RPA);
        if self.flashc.fsr[m] & FSR_RPROIN != 0 && self.flashc.fsr[m] & FSR_RPRODIS == 0 {
            new |= old & (3 << 17);
        }
        self.flashc.fcon[m] = new;
        true
    }
}

#[cfg(test)]
mod tests {
    use crate::memory::BoardVariant;
    use crate::{CpuBackend, Simos18Board};
    use unicorn_tricore_sys::TriCoreReg;

    fn board() -> Simos18Board {
        let flash = vec![0u8; 0x40_0000];
        let eep = vec![0u8; 0x4_0000];
        Simos18Board::new_variant(&flash, &eep, BoardVariant::Simos18Tc1791).unwrap()
    }

    fn cmd(b: &mut Simos18Board, a: u32, v: u32) {
        b.hw_mut().flash_command(a, 4, v as u64);
    }

    #[test]
    fn dflash_program_and_erase() {
        let mut b = board();
        cmd(&mut b, 0xAF00_5554, 0x5D);
        for i in 0..16u32 {
            cmd(&mut b, 0xAF00_55F0, 0x0403_0201 + i);
            cmd(&mut b, 0xAF00_55F4, 0x0807_0605);
        }
        cmd(&mut b, 0xAF00_5554, 0xAA);
        cmd(&mut b, 0xAF00_AAA8, 0x55);
        cmd(&mut b, 0xAF00_5554, 0xA0);
        cmd(&mut b, 0xAF00_0100, 0xAA);
        let hw = b.hw();
        assert_eq!(hw.flashc.dflash_pages_programmed, 1);
        assert_eq!(&hw.dflash[0x100..0x108], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(hw.dflash[0x100 + 8], 2);
        // guest-visible through the mapping
        assert_eq!(b.read_mem(0xAF00_0100, 4).unwrap(), vec![1, 2, 3, 4]);

        // erase sector (bank 0): the DFLASH form ends with SA:40 (UM Table 7-10); SA:30 is the
        // PFLASH logical-sector form and a sequence error on the DFlash
        let erase = |b: &mut Simos18Board, sa: u32, last: u32| {
            for (a, v) in [(0x5554, 0xAA), (0xAAA8, 0x55), (0x5554, 0x80), (0x5554, 0xAA), (0xAAA8, 0x55)] {
                cmd(b, sa + a, v);
            }
            cmd(b, sa, last);
        };
        erase(&mut b, 0xAF00_0000, 0x30);
        assert_eq!((b.hw().flashc.dflash_erases, b.hw().dflash[0x100]), (0, 1));
        assert!(b.hw().flashc.fsr[0] & super::FSR_SQER != 0);
        cmd(&mut b, 0xAF00_5554, 0xF5);
        erase(&mut b, 0xAF00_0000, 0x40);
        assert_eq!(b.hw().flashc.dflash_erases, 1);
        assert!(b.hw().flashc.fsr[0] & super::FSR_ERASE != 0);
        assert!(b.hw().dflash[..0x1_8000].iter().all(|&x| x == 0));
        assert_eq!(b.hw().flashc.sequence_errors, 1);
    }

    /// SC800S50 boot (VERIFIED-T): erase DFlash bank 1, Reset to Read + Clear Status on both modules,
    /// Resume Protection on both — no sequence error.
    #[test]
    fn firmware_boot_sequence_is_accepted() {
        let mut b = board();
        b.hw_mut().dflash[0x1_8000 + 5] = 0x77;
        for (a, v) in [(0x5554, 0xF5), (0x5554, 0xAA), (0xAAA8, 0x55), (0x5554, 0x80), (0x5554, 0xAA), (0xAAA8, 0x55), (0x0000, 0x40)] {
            cmd(&mut b, 0xAF08_0000 + a, v);
        }
        assert_eq!(b.hw().dflash[0x1_8000 + 5], 0, "bank 1 erased");
        for base in [0xA000_0000u32, 0xA080_0000] {
            cmd(&mut b, base + 0x5554, 0xF0);
            cmd(&mut b, base + 0x5554, 0xF5);
        }
        for base in [0xA000_0000u32, 0xA080_0000] {
            cmd(&mut b, base + 0x5554, 0x5E);
        }
        let fc = &b.hw().flashc;
        assert_eq!((fc.sequence_errors, fc.fsr[0] & super::FSR_SQER, fc.fsr[1] & super::FSR_SQER), (0, 0, 0));
        cmd(&mut b, 0xAF00_5554, 0x5D);
        for i in 0..16 {
            cmd(&mut b, 0xAF00_55F0, i);
        }
        for (a, v) in [(0x5554, 0xAA), (0xAAA8, 0x55), (0x5554, 0xA0), (0x0100, 0xAA)] {
            cmd(&mut b, 0xAF00_0000 + a, v);
        }
        assert_eq!(b.hw().flashc.dflash_pages_programmed, 1);
        let expected: Vec<u8> = (0u32..16).flat_map(u32::to_le_bytes).collect();
        assert_eq!(&b.hw().dflash[0x100..0x140], expected.as_slice());
    }

    /// Reset to Read inside a command sequence aborts it without a sequence error and clears the
    /// error flags (UM p.7-15).
    #[test]
    fn reset_to_read_aborts_a_sequence() {
        let mut b = board();
        cmd(&mut b, 0xAF00_5554, 0xAA);
        cmd(&mut b, 0xAF00_1234, 0x99); // SQER
        cmd(&mut b, 0xAF00_5554, 0xAA);
        cmd(&mut b, 0xAF00_AAA8, 0x55);
        cmd(&mut b, 0xAF00_5554, 0xF0);
        assert_eq!(b.hw().flashc.fsr[0] & super::FSR_SQER, 0);
        assert_eq!(b.hw().flashc.sequence_errors, 1);
        // a new sequence starts from scratch
        cmd(&mut b, 0xAF00_5554, 0x5D);
        assert!(b.hw().flashc.fsr[0] & super::FSR_DFPAGE != 0);
    }

    #[test]
    fn bad_sequence_sets_sqer_and_flash_is_untouched() {
        let mut b = board();
        cmd(&mut b, 0xAF00_5554, 0xAA);
        cmd(&mut b, 0xAF00_1234, 0x99);
        assert!(b.hw().flashc.fsr[0] & super::FSR_SQER != 0);
        // Reset to Read does not change array content
        cmd(&mut b, 0xA000_5554, 0xF0);
        cmd(&mut b, 0xA080_5554, 0xF0);
        assert!(b.hw().flash[0x5554..0x5558].iter().all(|&x| x == 0));
    }

    #[test]
    fn pflash_program_and_erase() {
        let mut b = board();
        for (base, index, last) in [(0xA000_0000u32, 0x100usize, 0x30u32), (0xA080_0000, 0x20_0100, 0x40)] {
            b.hw_mut().flash[index] = 0x08;
            cmd(&mut b, base + 0x5554, 0x50);
            cmd(&mut b, base + 0x55F0, 0x06);
            cmd(&mut b, base + 0x55F4, 0x00);
            for (a, v) in [(0x5554, 0xAA), (0xAAA8, 0x55), (0x5554, 0xA0), (0x0100, 0xAA)] {
                cmd(&mut b, base + a, v);
            }
            assert_eq!(b.hw().flash[index], 0x0E);
            for (a, v) in [(0x5554, 0xAA), (0xAAA8, 0x55), (0x5554, 0x80), (0x5554, 0xAA), (0xAAA8, 0x55)] {
                cmd(&mut b, base + a, v);
            }
            cmd(&mut b, base + 0x0100, last);
            assert!(b.hw().flash[index - 0x100..index - 0x100 + 0x4000].iter().all(|&x| x == 0));
        }
        let fc = &b.hw().flashc;
        assert_eq!(fc.pflash_pages_programmed, 2);
        assert_eq!(fc.pflash_erases, 2);
        assert_eq!(fc.sequence_errors, 0);
    }

    #[test]
    fn margin_register_reset_and_write_lanes() {
        let mut b = board();
        assert_eq!(b.hw().flash_reg_read(0xF800_2018), Some(0x8000));
        assert_eq!(b.hw().flash_reg_read(0xF800_4018), Some(0x8000));
        assert_eq!(b.read_mem(0xF800_201D, 1).unwrap(), [0x80]);
        b.hw_mut().mmio_write(0xF800_201D, 1, 0);
        assert_eq!(b.hw().flashc.margin[1], 0);
        b.hw_mut().mmio_write(0xF800_201C, 4, u32::MAX as u64);
        assert_eq!(b.hw().flashc.margin[1], 0x801F);
        b.hw_mut().mmio_write(0xF800_2018, 4, u32::MAX as u64);
        assert_eq!(b.hw().flashc.margin[0], 0x800F);
        b.hw_mut().mmio_write(0xF800_4018, 4, u32::MAX as u64);
        assert_eq!(b.hw().flashc.marp1, 0x800F);
        b.hw_mut().flashc.reset();
        assert_eq!(b.read_mem(0xF800_201D, 1).unwrap(), [0x80]);
        assert_eq!(b.hw().flash_reg_read(0xF800_4018), Some(0x8000));
    }

    #[test]
    fn password_unlock_and_resume_protection() {
        let mut b = board();
        b.hw_mut().flash[0x1_4200] = 1;
        for (off, value) in [(0x1_420C_usize, 0x1122_3344_u32), (0x1_4210, 0x5566_7788)] {
            b.hw_mut().flash[off..off + 4].copy_from_slice(&value.to_le_bytes());
        }
        b.hw_mut().flashc = super::FlashCtrl::from_image(&b.hw().flash);
        for base in [0xA000_0000, 0xA080_0000] {
            cmd(&mut b, base + 0x5554, 0x5E);
        }
        assert_eq!(b.hw().flashc.fsr, [super::FSR_PROIN | super::FSR_RPROIN; 2]);
        assert!(b.hw().flash_reg_read(0xF800_2014).unwrap() & super::FCON_RPA != 0);
        for (addr, expected) in [(0xF800_2020, 0x8000_0000), (0xF800_2024, 0), (0xF800_4020, 0x8000_0000), (0xF800_4024, 0)] {
            assert_eq!(b.hw().flash_reg_read(addr), Some(expected));
        }
        for (a, v) in [(0x5554, 0xAA), (0xAAA8, 0x55), (0x5554, 0x80), (0x5554, 0xAA), (0xAAA8, 0x55), (0x0100, 0x40)] {
            cmd(&mut b, 0xA000_0000 + a, v);
        }
        assert_eq!(b.hw().flashc.pflash_erases, 0);
        assert!(b.hw().flashc.fsr[0] & super::FSR_PROER != 0);

        for (ucb, mode) in [(0, 8), (0, 5)] {
            for (a, v) in [(0x5554, 0xAA), (0xAAA8, 0x55), (0x553C, ucb), (0xAAA8, 0x1122_3344), (0xAAA8, 0x5566_7788), (0x5558, mode)] {
                cmd(&mut b, 0xA000_0000 + a, v);
            }
        }
        assert_eq!(
            b.hw().flashc.fsr[0] & (super::FSR_PROIN | super::FSR_RPROIN | super::FSR_RPRODIS | super::FSR_WPRODIS0 | super::FSR_PROER),
            super::FSR_PROIN | super::FSR_RPROIN | super::FSR_RPRODIS | super::FSR_WPRODIS0
        );
        assert_eq!(b.hw().flash_reg_read(0xF800_2014).unwrap() & super::FCON_RPA, 0);
        for (a, v) in [(0x5554, 0xAA), (0xAAA8, 0x55), (0x5554, 0x80), (0x5554, 0xAA), (0xAAA8, 0x55), (0x0100, 0x40)] {
            cmd(&mut b, 0xA000_0000 + a, v);
        }
        assert_eq!(b.hw().flashc.pflash_erases, 1);

        cmd(&mut b, 0xA000_5554, 0x5E);
        for (a, v) in [(0x5554, 0xAA), (0xAAA8, 0x55), (0x553C, 0), (0xAAA8, 0), (0xAAA8, 0x5566_7788), (0x5558, 8)] {
            cmd(&mut b, 0xA000_0000 + a, v);
        }
        assert!(b.hw().flashc.fsr[0] & super::FSR_PROER != 0);
        assert_eq!(b.hw().flashc.fsr[0] & super::FSR_RPRODIS, 0);
        for (a, v) in [(0x5554, 0xAA), (0xAAA8, 0x55), (0x553C, 0), (0xAAA8, 0x1122_3344), (0xAAA8, 0x5566_7788), (0x5558, 8)] {
            cmd(&mut b, 0xA000_0000 + a, v);
        }
        assert!(b.hw().flashc.fsr[0] & super::FSR_PROER != 0, "another password attempt needs a reset");
        assert_eq!(b.hw().flashc.fsr[0] & super::FSR_RPRODIS, 0);
        b.hw_mut().flashc.reset();
        assert!(b.hw().flashc.fsr[0] & super::FSR_RPRODIS != 0);
        assert_eq!(b.hw().flashc.fsr[0] & super::FSR_PROER, 0);
        cmd(&mut b, 0xA000_5554, 0x5E);
        assert_eq!(b.hw().flashc.fsr[0] & super::FSR_RPRODIS, 0);
    }

    /// Guest code running in PSPR programs and erases PFlash0, PFlash1, and DFlash banks 0/1 at
    /// runtime and immediately reads and executes at both cached (`0x80..`) and non-cached
    /// (`0xA0..` / `0xAF..`) aliases within one multi-instruction run, on both `Native` and `Unicorn`.
    #[test]
    fn guest_pflash_and_dflash_runtime_modify_parity_across_backends() {
        // PSPR routine @ 0xC000_0000:
        //   0x00: ld.a a4, [a12]0     (99 C4 00 00)
        //   0x04: jz.a a4, +10        (BD 04 0A 00 -> 0xC000_0018)
        //   0x08: ld.w d4, [a12]4     (19 C4 04 00)
        //   0x0C: st.w [a4]0, d4      (59 44 00 00)  -- guest flash command cycle
        //   0x10: lea  a12, [a12]8    (D9 CC 08 00)
        //   0x14: j    0xC000_0000    (1D FF F6 FF)
        //   0x18: ld.w d8, [a6]0      (19 68 00 00)  -- guest readback at alias 0
        //   0x1C: ld.w d9, [a7]0      (19 79 00 00)  -- guest readback at alias 1
        //   0x20: jaz  a5, +6         (BD 05 06 00 -> 0xC000_002C, skip jli for DFlash data-only)
        //   0x24: jli  a6             (2D 06 20 00)  -- execute at cached PFlash alias
        //   0x28: st.w [a14]0, d2     (59 E2 00 00)
        //   0x2C: jz.a a5, +6         (BD 05 06 00 -> 0xC000_0038)
        //   0x30: jli  a7             (2D 07 20 00)  -- execute at uncached PFlash alias
        //   0x34: st.w [a14]4, d2     (59 E2 04 00)
        //   0x38: j    .              (3C 00)
        const PSPR_DRIVER: &[u8] = &[
            0x99, 0xC4, 0x00, 0x00,
            0xBD, 0x04, 0x0A, 0x00,
            0x19, 0xC4, 0x04, 0x00,
            0x59, 0x44, 0x00, 0x00,
            0xD9, 0xCC, 0x08, 0x00,
            0x1D, 0xFF, 0xF6, 0xFF,
            0x19, 0x68, 0x00, 0x00,
            0x19, 0x79, 0x00, 0x00,
            0xBD, 0x05, 0x06, 0x00,
            0x2D, 0x06, 0x20, 0x00,
            0x59, 0xE2, 0x00, 0x00,
            0xBD, 0x05, 0x06, 0x00,
            0x2D, 0x07, 0x20, 0x00,
            0x59, 0xE2, 0x04, 0x00,
            0x3C, 0x00,
        ];
        // Leaf function `mov d2, #imm4 ; ji a11`
        let leaf = |imm4: u8| [0x82u8, (imm4 << 4) | 0x02, 0xDC, 0x0B];

        let run_driver = |b: &mut Simos18Board, cmds: &[(u32, u32)], alias0: u32, alias1: u32, exec: bool| -> (u32, u32, u32, u32) {
            let mut table = Vec::with_capacity((cmds.len() + 1) * 8);
            for &(a, v) in cmds {
                table.extend_from_slice(&a.to_le_bytes());
                table.extend_from_slice(&v.to_le_bytes());
            }
            table.extend_from_slice(&0u64.to_le_bytes());
            b.write_mem(0xD000_2000, &table).unwrap();
            b.write_mem(0xD000_3000, &[0u8; 8]).unwrap();
            b.write_reg(TriCoreReg::A12, 0xD000_2000);
            b.write_reg(TriCoreReg::A14, 0xD000_3000);
            b.write_reg(TriCoreReg::A6, alias0);
            b.write_reg(TriCoreReg::A7, alias1);
            b.write_reg(TriCoreReg::A5, if exec { 1 } else { 0 });
            b.write_reg(TriCoreReg::D2, 0xDEAD);
            b.write_reg(TriCoreReg::D8, 0);
            b.write_reg(TriCoreReg::D9, 0);
            b.write_reg(TriCoreReg::PC, 0xC000_0000);
            b.run_instructions(800).unwrap();
            let res = b.read_mem(0xD000_3000, 8).unwrap();
            let r0 = u32::from_le_bytes(res[0..4].try_into().unwrap());
            let r1 = u32::from_le_bytes(res[4..8].try_into().unwrap());
            (b.read_reg(TriCoreReg::D8), b.read_reg(TriCoreReg::D9), r0, r1)
        };

        for backend in CpuBackend::matrix() {
            let mut flash_img = vec![0u8; 0x40_0000];
            // Initial PFlash0 @ +0x100: `mov d2, #3 ; ji a11`; PFlash1 sector 1 @ +0x20_4100: `mov d2, #4 ; ji a11`
            flash_img[0x100..0x104].copy_from_slice(&leaf(3));
            flash_img[0x20_4100..0x20_4104].copy_from_slice(&leaf(4));
            let mut b = Simos18Board::new_on(&flash_img, &vec![0u8; 0x4_0000], backend).unwrap();
            b.write_mem(0xC000_0000, PSPR_DRIVER).unwrap();

            // 1. Warm up TB / block caches at BOTH cached and uncached aliases of PFlash0 and PFlash1!
            let init_w0 = u32::from_le_bytes(leaf(3));
            let init_w1 = u32::from_le_bytes(leaf(4));
            assert_eq!(run_driver(&mut b, &[], 0x8000_0100, 0xA000_0100, true), (init_w0, init_w0, 3, 3), "{backend:?} warmup PFlash0");
            assert_eq!(run_driver(&mut b, &[], 0x8080_4100, 0xA080_4100, true), (init_w1, init_w1, 4, 4), "{backend:?} warmup PFlash1");

            // 2. Guest erases PFlash0 sector 0 (SA:30) and programs `mov d2, #7 ; ji a11`, then immediately
            //    reads AND executes at 0x8000_0100 and 0xA000_0100 in the same run!
            let new_w0 = u32::from_le_bytes(leaf(7));
            let cmds_pf0 = [
                (0xA000_5554, 0xAA), (0xA000_AAA8, 0x55), (0xA000_5554, 0x80),
                (0xA000_5554, 0xAA), (0xA000_AAA8, 0x55), (0xA000_0000, 0x30),
                (0xA000_5554, 0x50),
                (0xA000_55F0, new_w0),
                (0xA000_55F4, 0xCAFE_BEEF),
                (0xA000_5554, 0xAA), (0xA000_AAA8, 0x55), (0xA000_5554, 0xA0), (0xA000_0100, 0xAA),
            ];
            assert_eq!(
                run_driver(&mut b, &cmds_pf0, 0x8000_0100, 0xA000_0100, true),
                (new_w0, new_w0, 7, 7),
                "{backend:?} runtime PFlash0 erase+program must update data & code at cached+uncached aliases"
            );

            // 3. Guest erases PFlash1 sector 1 (0xA080_4000, SA:40) and programs `mov d2, #6 ; ji a11`!
            let new_w1 = u32::from_le_bytes(leaf(6));
            let cmds_pf1 = [
                (0xA080_5554, 0xAA), (0xA080_AAA8, 0x55), (0xA080_5554, 0x80),
                (0xA080_5554, 0xAA), (0xA080_AAA8, 0x55), (0xA080_4000, 0x40),
                (0xA080_5554, 0x50),
                (0xA080_55F0, new_w1),
                (0xA080_55F4, 0),
                (0xA080_5554, 0xAA), (0xA080_AAA8, 0x55), (0xA080_5554, 0xA0), (0xA080_4100, 0xAA),
            ];
            assert_eq!(
                run_driver(&mut b, &cmds_pf1, 0x8080_4100, 0xA080_4100, true),
                (new_w1, new_w1, 6, 6),
                "{backend:?} runtime PFlash1 erase+program must update data & code at cached+uncached aliases"
            );

            // 4. Guest programs DFlash bank 0 (0xAF00_0200) and bank 1 (0xAF08_0200), reads back, then erases!
            let cmds_df_prog = [
                (0xAF00_5554, 0x5D), (0xAF00_55F0, 0x1357_2468), (0xAF00_55F4, 0),
                (0xAF00_5554, 0xAA), (0xAF00_AAA8, 0x55), (0xAF00_5554, 0xA0), (0xAF00_0200, 0xAA),
                (0xAF08_5554, 0x5D), (0xAF08_55F0, 0xDEAD_C0DE), (0xAF08_55F4, 0),
                (0xAF08_5554, 0xAA), (0xAF08_AAA8, 0x55), (0xAF08_5554, 0xA0), (0xAF08_0200, 0xAA),
            ];
            assert_eq!(
                run_driver(&mut b, &cmds_df_prog, 0xAF00_0200, 0xAF08_0200, false),
                (0x1357_2468, 0xDEAD_C0DE, 0, 0),
                "{backend:?} runtime DFlash program readback"
            );
            let cmds_df_erase = [
                (0xAF00_5554, 0xAA), (0xAF00_AAA8, 0x55), (0xAF00_5554, 0x80),
                (0xAF00_5554, 0xAA), (0xAF00_AAA8, 0x55), (0xAF00_0000, 0x40),
                (0xAF08_5554, 0xAA), (0xAF08_AAA8, 0x55), (0xAF08_5554, 0x80),
                (0xAF08_5554, 0xAA), (0xAF08_AAA8, 0x55), (0xAF08_0000, 0x40),
            ];
            assert_eq!(
                run_driver(&mut b, &cmds_df_erase, 0xAF00_0200, 0xAF08_0200, false),
                (0, 0, 0, 0),
                "{backend:?} runtime DFlash erase readback"
            );
            assert_eq!(b.hw().flashc.sequence_errors, 0, "{backend:?}");
        }
    }

    #[test]
    fn interleaved_array_sequence_and_oob_dflash_raise_sqer() {
        use super::{FSR_PROG, FSR_SQER};
        let mut b = board();
        // Start unlock on PFlash0, then write cycle 2 on PFlash1 -> must abort with SQER on both modules!
        cmd(&mut b, 0xA000_5554, 0xAA);
        cmd(&mut b, 0xA080_AAA8, 0x55);
        assert_ne!(b.hw().flashc.fsr[0] & FSR_SQER, 0, "interleaved PFlash0->PFlash1 must set FSR_SQER on module 0");
        assert_ne!(b.hw().flashc.fsr[1] & FSR_SQER, 0, "interleaved PFlash0->PFlash1 must set FSR_SQER on module 1");
        assert_eq!(b.hw().flashc.seq_len, 0);

        cmd(&mut b, 0xA000_5554, 0xF5); // Clear Status
        cmd(&mut b, 0xA080_5554, 0xF5);

        // Out-of-bounds DFlash page program (0xAF02_0000 > 96 KB DF0): must set SQER, not PROG!
        let writes0 = b.hw().flashc.dflash_pages_programmed;
        for (a, d) in [
            (0xAF00_5554, 0x5D),
            (0xAF00_55F0, 0x1234_5678),
            (0xAF00_55F4, 0),
            (0xAF00_5554, 0xAA),
            (0xAF00_AAA8, 0x55),
            (0xAF00_5554, 0xA0),
            (0xAF02_0000, 0xAA),
        ] {
            cmd(&mut b, a, d);
        }
        assert_ne!(b.hw().flashc.fsr[0] & FSR_SQER, 0, "OOB DFlash write must set FSR_SQER");
        assert_eq!(b.hw().flashc.fsr[0] & FSR_PROG, 0, "OOB DFlash write must not claim FSR_PROG");
        assert_eq!(b.hw().flashc.dflash_pages_programmed, writes0);
    }
}
