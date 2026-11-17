//! Generic corrections for TriCore instructions that Unicorn 2.1.4's TCG translator gets wrong.
//!
//! Each fix produces exactly the ISA-specified result (TriCore 1.6 architecture manual); firmware
//! bytes are never modified. Sites are discovered dynamically: the block hook decodes every
//! translation block before it executes, and when it finds an affected encoding at an address
//! without a fix hook it stops before the block runs; at the next quantum boundary a single-address
//! code hook (`begin == end`) is installed and the cached block is invalidated. The hook re-decodes
//! the bytes at PC (RAM code may change), computes the result in Rust, writes the destination
//! registers and sets PC to the next instruction (Unicorn resumes at the written PC).
//!
//! Unicorn 2.1.4 defects corrected (each proven by a unit test below):
//! * RCRW `IMASK` (0xD7/op2 1) and RCRW `INSERT` (0xD7/op2 0): `decode_rcrw_insert` swaps the
//!   position register (bits 27:24) and the destination (bits 31:28); `imask e2,..,d15,..` even
//!   writes `cpu_gpr_d[16]` (out of bounds). Fixed upstream in QEMU.
//! * RRRR `DEXTR` (0x17/op2 4) with a != b: shift by `32 - pos` is undefined for pos = 0 (host
//!   shifts mod 32, giving `D[a] | D[b]` instead of `D[a]`).
//! * RRPW `DEXTR` (0x77) with pos = 0 and a != b: immediate shift right by 32 (same defect).
//! * `BISR` (RC 0xAD/op2 0, SC 0xE0): `ICR |= const9` instead of `ICR.CCPN = const9`.
//! * `DISABLE D[a]` (SYS 0x0D/op2 0x0F): not decoded -> illegal-opcode trap.
//! * `MFCR`/`MTCR` (0x4D/0xCD) on CSFRs that Unicorn does not implement (e.g. PCON0/PCON1/PCON2,
//!   DCON0/DCON2): MFCR leaves the destination unchanged, MTCR is dropped. Modelled here, with the
//!   self-clearing PCON1.PCINV/PBINV invalidate bits.
//! * BO `LD.BU D[a], [+A[b]]off10` (0x09/op2 0x11, pre-increment): the loaded byte is
//!   sign-extended instead of zero-extended (breaks the firmware's AES/DFlash decryption loops).
//! * `MTCR` to ENDINIT-protected CSFRs (BIV, BTV, ISP, COMPAT, BMACON, SMACON, MIECON — the `E`
//!   entries of Unicorn's own csfr.def) is executed by Unicorn regardless of ENDINIT; hardware
//!   ignores it while ENDINIT = 1.
//! * `MFCR`/`MTCR` ICR (CSFR 0xFE2C): ENABLE/DISABLE/RESTORE update ICR through the TCG global
//!   `cpu_ICR`, which TCG keeps in a host register while a later op of the block still uses it;
//!   MFCR/MTCR access `env->ICR` with raw `tcg_gen_ld/st`. So in `enable; mfcr dX, ICR; disable`
//!   (one block) MFCR reads the stale IE, and in `enable; mtcr ICR, dX; enable` the MTCR is lost.
//!   The compiler's DVSTEP wrapper (`mfcr d1,ICR; disable; dvinit..dvstep; jz.t d1,8; enable`)
//!   back to back then never re-enables interrupts. A hook call at the site is the fix: TCG
//!   writes globals back and drops their register copies before calling a hook (helper flags 0),
//!   so Unicorn's own MFCR/MTCR then operate on the architectural ICR.
//! * `MTCR ICR` also writes the read-only ICR.PIPN field (csfr.def lists ICR as `A`), so the
//!   firmware could overwrite the ICU's arbitration result; the hook executes MTCR ICR itself.
//! * TriCore 1.3 cores ([`FixState::tc13`], TC1766 / DQ250): SVLCX (SYS 0DH op2 08H) and BISR keep
//!   PCXI.PCPN/PIE, while Unicorn implements the TC1.3.1/TC1.6 RTL that copies ICR.CCPN/IE into them
//!   (`tricore_core::Arch::Tc13`; UNVERIFIED against a TC1.3 manual, derived from the DQ250 OS).
//! * SRO `LD.H D15, [A[b]]off4` (0x8C): `decode_sro_opc` uses `EA = A[b] + off4` instead of
//!   `A[b] + 2 * off4`. The other 16-bit offset forms (SRO/SLRO/SSRO/SC LD/ST) scale correctly.
//! * RR `FTOU` (4BH op2 12H), `FTOQ31` (11H), `FTOQ31Z` (18H), `Q31TOF` (15H), n = 1: not decoded by
//!   `decode_rr_divide` (falls to its default, IOPC = class 2 TIN 1). FTOU is a TC1.3 instruction (V131
//!   ISA p. 3-479); both DQ250 images execute it (0D9300011R/4319 0x800A637C, 0D9300012F/4930
//!   0x800938B8, input-shaft speed) and halted in their class-2 handler. The hook executes them with the
//!   native core's implementation (`tricore_core::groups::fpu_div::conversion`: result, PSW flags, PSW.RM;
//!   no CAE trap, as for Unicorn's own FPU instructions).
//!
//! A SIMEMU-patched Unicorn fixes some of these in the translator (`uc_ctl` 0x5152,
//! [`FixState::lib`]); their sites are then not hooked. The FPU-state defects of the same library
//! are fixed only there and remain with stock Unicorn: `psw_read` drops `PSW[24]` (`RM[0]`) on every
//! CALL/RET/RFE/SVLCX/MFCR PSW/trap entry, `fpu_set_state` hands the unshifted PSW.RM to softfloat
//! (every rounding mode other than nearest-even is wrong), `UPDFL` never updates PSW.RM and
//! clears the flags outside its mask, and `RET` restores PSW.RM from the CSA (TC1.3.1/TC1.6 keep
//! it).
//!
//! Not a translator defect but handled by the same site machinery ([`irq_site`]): ENABLE, RESTORE,
//! RFE (and the MTCR ICR / BISR fixes) end the quantum right after the instruction when a pending
//! interrupt becomes deliverable, as the native core does, instead of up to `QUANTUM_MAX_INSNS` later.
//!
//! A hook that executes the instruction writes PC, which makes Unicorn abandon the rest of the
//! translation block that the block hook has already counted; the hook takes those instructions
//! back out of the quantum (blocks with fix sites always run the block hook, see
//! `cpu_hooks::classify_block`, which records the block end).

use crate::memory::Tc1791HardwareState;
use crate::stm_icu::csa_ea;

const ICR_PIPN: u32 = 0x00FF_0000;
use std::collections::{BTreeMap, HashMap};
use std::ffi::{c_int, c_void};
use unicorn_tricore_sys::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fix {
    ImaskRcrw { c: u32, d: u32, width: u32, const4: u32 },
    InsertRcrw { c: u32, a: u32, d: u32, width: u32, const4: u32 },
    DextrRrrr { c: u32, a: u32, b: u32, d: u32 },
    DextrRrpwPos0 { c: u32, a: u32 },
    Bisr { const9: u32, len: u32 },
    /// SVLCX on a TriCore 1.3 core (only reported when [`FixState::tc13`]).
    SvlcxTc13,
    DisableD { a: u32 },
    Mfcr { c: u32, csfr: u32 },
    Mtcr { a: u32, csfr: u32 },
    MtcrProtected { csfr: u32 },
    LdBuPreInc { a: u32, b: u32, off: i32 },
    IcrSync,
    MtcrIcr { a: u32 },
    LdHSro { b: u32, off4: u32 },
    /// RR FTOU/FTOQ31/FTOQ31Z/Q31TOF (op1 4BH, n = 1): `D[c]` from `D[a]`, `D[b]`.
    FpuConv { op2: u32, a: u32, b: u32, c: u32 },
    /// Interrupt-latency sites (not translator defects, see [`irq_site`]): `ENABLE`,
    /// `RESTORE D[a]`, `RFE` (32-bit and 16-bit form).
    Enable,
    Restore { a: u32 },
    Rfe,
}

impl Fix {
    pub fn len(&self) -> u32 {
        match self {
            Fix::Bisr { len, .. } => *len,
            Fix::LdHSro { .. } => 2,
            _ => 4,
        }
    }

    pub fn is_empty(&self) -> bool {
        false
    }

    fn kind(&self) -> usize {
        match self {
            Fix::ImaskRcrw { .. } => 0,
            Fix::InsertRcrw { .. } => 1,
            Fix::DextrRrrr { .. } => 2,
            Fix::DextrRrpwPos0 { .. } => 3,
            Fix::Bisr { .. } | Fix::SvlcxTc13 => 4,
            Fix::DisableD { .. } => 5,
            Fix::Mfcr { .. } => 6,
            Fix::Mtcr { .. } => 7,
            Fix::MtcrProtected { .. } => 8,
            Fix::LdBuPreInc { .. } => 9,
            Fix::IcrSync | Fix::MtcrIcr { .. } | Fix::Enable | Fix::Restore { .. } | Fix::Rfe => 10,
            Fix::LdHSro { .. } => 11,
            Fix::FpuConv { .. } => 12,
        }
    }
}

/// Kind 10 counts both the ICR sync/PIPN fixes and the exact-interrupt-latency stops of
/// [`irq_site`] (the oracle's `KNOWN_WRONG` mirrors this list; the latency sites are not defects
/// and stay `trustworthy`, they are not returned by [`decode`]).
pub const FIX_KIND_NAMES: [&str; 13] = [
    "imask_rcrw", "insert_rcrw", "dextr_rrrr", "dextr_rrpw_pos0", "bisr", "disable_d", "mfcr_csfr",
    "mtcr_csfr", "mtcr_endinit", "ld_bu_preinc", "icr", "ld_h_sro", "fpu_conv",
];

/// Instructions after which a pending interrupt can become deliverable (they set ICR.IE or lower
/// ICR.CCPN): ENABLE (SYS 0DH op2 0CH), `RESTORE D[a]` (op2 0EH), RFE (op2 07H, SR 8000H), MTCR ICR
/// (RLC CDH, const16 FE2CH). The board delivers interrupts at quantum boundaries; the native core
/// ends its quantum right after such an instruction when an interrupt wins (V1 5.3: the ICU
/// request is taken at the next instruction boundary), but Unicorn kept executing up to the quantum
/// end (`QUANTUM_MAX_INSNS`, 16.7 us at 120 MHz) — enough to reorder the DQ200 OS tasks and trip
/// its safety software's 10 ms task supervision (docs/DQ200_IO_MAP.md §5.3). With a hook here
/// Unicorn stops at the same boundary. BISR is a fix site of [`decode`] already and gets the same
/// check; so is MTCR ICR with a stock library (a patched one translates it, but the interrupt check
/// is still needed: SC800S50 0x8008D594 `mtcr ICR, d15` unmasks the SSC/DMA interrupt).
pub fn irq_site(lo: u16, hi: u16) -> Option<Fix> {
    if lo & 1 == 0 {
        return (lo == 0x8000).then_some(Fix::Rfe);
    }
    let w = lo as u32 | (hi as u32) << 16;
    if w & 0xFF == 0xCD && (w >> 12) & 0xFFFF == 0xFE2C {
        return Some(Fix::MtcrIcr { a: (w >> 8) & 0xF });
    }
    if w & 0xFF != 0x0D {
        return None;
    }
    match (w >> 22) & 0x3F {
        0x0C => Some(Fix::Enable),
        0x0E => Some(Fix::Restore { a: (w >> 8) & 0xF }),
        0x07 => Some(Fix::Rfe),
        _ => None,
    }
}

/// [`decode`] (translator defects the linked library needs fixed) or an [`irq_site`].
fn site(lo: u16, hi: u16, fixes: &FixState) -> Option<Fix> {
    decode(lo, hi)
        .filter(|f| fixes.needed(f))
        .or_else(|| (fixes.tc13 && (lo, hi) == (0x000D, 0x0200)).then_some(Fix::SvlcxTc13))
        .or_else(|| irq_site(lo, hi))
}

/// CSFR offsets that Unicorn 2.1.4 implements in `gen_mfcr`/`gen_mtcr` (csfr.def + PSW).
fn unicorn_implements_csfr(off: u32) -> bool {
    matches!(
        off,
        0xFE00 | 0xFE04 | 0xFE08 | 0xFE14 | 0xFE18 | 0xFE1C | 0xFE20 | 0xFE24 | 0xFE28 | 0xFE2C
            | 0xFE38 | 0xFE3C | 0x9400 | 0x8000 | 0x8004 | 0x800C | 0x8010 | 0x8014 | 0x8018
            | 0x9004 | 0x900C | 0x9020 | 0x9024 | 0x9028 | 0x9044 | 0x9210 | 0x9214 | 0x9218
            | 0xFD00 | 0xFD08 | 0xFD0C | 0xFD10 | 0xFD20 | 0xFD24 | 0xFD40 | 0xFD44 | 0xFD48
            | 0xFC00 | 0xFC04 | 0xFC08 | 0xFC0C | 0xFC10 | 0xFC14
            | 0xE000 | 0xE080 | 0xE100 | 0xE180 | 0xE200 | 0xE280 | 0xE300 | 0xE380
    ) || ((0xC000..=0xDC1C).contains(&off) && off & 0x3FF <= 0x1C && off & 3 == 0)
}

/// ENDINIT-protected CSFRs: the `E` entries of QEMU/Unicorn `csfr.def` (TriCore 1.6 manual).
fn csfr_endinit_protected(off: u32) -> bool {
    matches!(off, 0xFE20 | 0xFE24 | 0xFE28 | 0x9400 | 0x9004 | 0x900C | 0x9044)
}

/// Decode an instruction (`lo` = first halfword, `hi` = second halfword) into a fix, if Unicorn
/// mistranslates it.
pub fn decode(lo: u16, hi: u16) -> Option<Fix> {
    if lo & 1 == 0 {
        // 16-bit
        return match lo & 0xFF {
            0xE0 => Some(Fix::Bisr { const9: (lo >> 8) as u32, len: 2 }),
            0x8C if (lo >> 8) & 0xF != 0 => Some(Fix::LdHSro { b: (lo >> 12) as u32, off4: (lo >> 8) as u32 & 0xF }),
            _ => None,
        };
    }
    let w = lo as u32 | (hi as u32) << 16;
    let c = w >> 28;
    let d = (w >> 24) & 0xF;
    let a = (w >> 8) & 0xF;
    let b = (w >> 12) & 0xF;
    match w & 0xFF {
        0xD7 => {
            let width = (w >> 16) & 0x1F;
            let const4 = (w >> 12) & 0xF;
            match (w >> 21) & 7 {
                1 => Some(Fix::ImaskRcrw { c, d, width, const4 }),
                0 => Some(Fix::InsertRcrw { c, a, d, width, const4 }),
                _ => None,
            }
        }
        0x17 if (w >> 21) & 7 == 4 && a != b => Some(Fix::DextrRrrr { c, a, b, d }),
        0x77 if (w >> 23) & 0x1F == 0 && a != b => Some(Fix::DextrRrpwPos0 { c, a }),
        0x09 if (w >> 22) & 0x3F == 0x11 => {
            let off10 = (((w >> 28) & 0xF) << 6) | ((w >> 16) & 0x3F);
            Some(Fix::LdBuPreInc { a, b, off: ((off10 << 22) as i32) >> 22 })
        }
        0xAD if (w >> 21) & 0x7F == 0 => Some(Fix::Bisr { const9: (w >> 12) & 0x1FF, len: 4 }),
        0x0D if (w >> 22) & 0x3F == 0x0F => Some(Fix::DisableD { a }),
        0x4B if (w >> 16) & 3 == 1 && matches!((w >> 20) & 0xFF, 0x11 | 0x12 | 0x15 | 0x18) => {
            Some(Fix::FpuConv { op2: (w >> 20) & 0xFF, a, b, c })
        }
        0x4D if (w >> 12) & 0xFFFF == 0xFE2C => Some(Fix::IcrSync),
        0xCD if (w >> 12) & 0xFFFF == 0xFE2C => Some(Fix::MtcrIcr { a }),
        0x4D => {
            let csfr = (w >> 12) & 0xFFFF;
            (!unicorn_implements_csfr(csfr)).then_some(Fix::Mfcr { c, csfr })
        }
        0xCD => {
            let csfr = (w >> 12) & 0xFFFF;
            if !unicorn_implements_csfr(csfr) {
                Some(Fix::Mtcr { a, csfr })
            } else if csfr_endinit_protected(csfr) {
                Some(Fix::MtcrProtected { csfr })
            } else {
                None
            }
        }
        _ => None,
    }
}

#[derive(Default)]
pub struct FixState {
    /// Installed fix sites (address -> hook handle).
    pub sites: HashMap<u32, uc_hook>,
    /// Discovered sites awaiting installation at the next quantum boundary.
    pub pending: Vec<u32>,
    pub hits: [u64; 13],
    pub sites_by_kind: [u64; 13],
    /// `SIMEMU_FIX_*` mask of the linked Unicorn (`uc_ctl_simemu_isa_fixes`, 0 = stock).
    pub lib: u32,
    /// End address of the block whose block hook ran last (see `hook_isa_fix`).
    pub tb_end: u32,
    /// CSFRs not implemented by Unicorn (offset -> value).
    pub csfr: BTreeMap<u32, u32>,
    pub endinit_csfr_dropped: u64,
    /// (CSFR offset, PC) of the most recent MTCR dropped because ENDINIT = 1.
    pub last_endinit_csfr_drop: (u32, u32),
    /// Quanta ended right after an ICR change because an interrupt became deliverable ([`irq_site`]).
    pub irq_stops: u64,
    /// TriCore 1.3 core: SVLCX/BISR keep PCXI.PCPN/PIE (`tricore_core::Arch::Tc13`).
    pub tc13: bool,
}

impl FixState {
    /// Whether `f` needs a hook with the linked Unicorn (a patched one translates some correctly).
    pub fn needed(&self, f: &Fix) -> bool {
        let fixed_by = match f {
            Fix::LdHSro { .. } => SIMEMU_FIX_SRO_LD_H,
            Fix::IcrSync | Fix::MtcrIcr { .. } => SIMEMU_FIX_ICR,
            _ => 0,
        };
        self.lib & fixed_by == 0 || fixed_by == 0
    }

    /// Reset values of the modelled CSFRs. PCON2/DCON2 report the TC1791 cache/scratchpad sizes
    /// in KB (datasheet: 16 KB ICACHE, 32 KB PSPR, 16 KB DCACHE, 128 KB DSPR).
    pub fn reset_csfrs(&mut self) {
        self.csfr.clear();
        self.csfr.insert(0x9208, (32 << 16) | 16); // PCON2
        self.csfr.insert(0x9000, (128 << 16) | 16); // DCON2
    }

    fn csfr_read(&self, off: u32) -> u32 {
        self.csfr.get(&off).copied().unwrap_or(0)
    }

    fn csfr_write(&mut self, off: u32, v: u32) {
        match off {
            0x9208 | 0x9000 => {}                     // PCON2/DCON2 read-only
            0x9204 => {
                self.csfr.insert(off, v & !0x3); // PCON1: PCINV/PBINV complete immediately
            }
            _ => {
                self.csfr.insert(off, v);
            }
        }
    }
}

impl Tc1791HardwareState {
    /// Decode the instructions of a translation block, calling `site` for every instruction that
    /// needs a fix. Returns the instruction count, `None` if the bytes are not host-visible.
    fn decode_block(&self, addr: u32, size: u32, mut on_site: impl FnMut(u32, Fix)) -> Option<u64> {
        let code = self.code_bytes(addr)?;
        let end = (size as usize).min(code.len());
        let mut off = 0usize;
        let mut n = 0u64;
        while off + 1 < end {
            let lo = u16::from_le_bytes([code[off], code[off + 1]]);
            let len = if lo & 1 == 0 { 2 } else { 4 };
            let hi = if len == 4 && off + 3 < code.len() {
                u16::from_le_bytes([code[off + 2], code[off + 3]])
            } else {
                0
            };
            if let Some(f) = site(lo, hi, &self.fixes) {
                on_site(addr.wrapping_add(off as u32), f);
            }
            off += len;
            n += 1;
        }
        Some(n.max(1))
    }

    fn is_new_site(&self, a: u32) -> bool {
        !self.fixes.sites.contains_key(&a) && !self.fixes.pending.contains(&a)
    }

    /// Decode all instructions of a translation block: returns the instruction count and queues
    /// fix sites that are not yet hooked.
    pub(crate) fn scan_block(&mut self, addr: u32, size: u32) -> (u64, bool) {
        let mut new_sites: [(u32, usize); 4] = [(0, 0); 4];
        let mut found = 0usize;
        let n = self.decode_block(addr, size, |a, f| {
            if found < new_sites.len() && self.is_new_site(a) {
                new_sites[found] = (a, f.kind());
                found += 1;
            }
        });
        let Some(n) = n else {
            return (size.div_ceil(4).max(1) as u64, false);
        };
        for &(a, k) in &new_sites[..found] {
            self.fixes.pending.push(a);
            self.fixes.sites_by_kind[k] += 1;
        }
        (n, found > 0)
    }

    /// Side-effect-free counterpart of [`Self::scan_block`]: the instruction count, `None` if the
    /// block contains a fix site (a hooked one needs the block end, see [`hook_isa_fix`]) or its
    /// bytes are not host-visible.
    pub(crate) fn block_count_if_known(&self, addr: u32, size: u32) -> Option<u64> {
        let mut site = false;
        let n = self.decode_block(addr, size, |_, _| site = true)?;
        (!site).then_some(n)
    }

    /// Instructions in `[from, to)` (code bytes host-visible), 0 if none or not visible.
    pub(crate) fn insns_between(&self, from: u32, to: u32) -> u64 {
        let Some(code) = self.code_bytes(from) else { return 0 };
        let end = (to.saturating_sub(from) as usize).min(code.len());
        let (mut off, mut n) = (0usize, 0u64);
        while off + 1 < end {
            off += if code[off] & 1 == 0 { 2 } else { 4 };
            n += 1;
        }
        n
    }
}

#[inline]
fn rd(uc: *mut uc_engine, r: TriCoreReg) -> u32 {
    let mut v: u32 = 0;
    // SAFETY: valid engine; register id from the enum.
    unsafe { uc_reg_read(uc, r as c_int, (&mut v as *mut u32).cast()) };
    v
}

#[inline]
fn wr(uc: *mut uc_engine, r: TriCoreReg, v: u32) {
    // SAFETY: as above.
    unsafe { uc_reg_write(uc, r as c_int, (&v as *const u32).cast()) };
}

/// Architectural PSW: Unicorn keeps C/V/SV/AV/SAV (= FS..FU) in its PSW_USB_* cache, the raw PSW word
/// has stale bits 31:27 (QEMU `psw_read`; `Simos18Board::psw`).
fn psw_read(uc: *mut uc_engine) -> u32 {
    let f = |r, sh: u32| (rd(uc, r) >> 31) << sh;
    (rd(uc, TriCoreReg::PSW) & 0x07FF_FFFF)
        | ((rd(uc, TriCoreReg::PSW_USB_C) != 0) as u32) << 31
        | f(TriCoreReg::PSW_USB_V, 30)
        | f(TriCoreReg::PSW_USB_SV, 29)
        | f(TriCoreReg::PSW_USB_AV, 28)
        | f(TriCoreReg::PSW_USB_SAV, 27)
}

/// QEMU `psw_write`: the raw word and the PSW_USB_* cache.
fn psw_write(uc: *mut uc_engine, v: u32) {
    wr(uc, TriCoreReg::PSW, v);
    for (r, bit) in [(TriCoreReg::PSW_USB_C, 31), (TriCoreReg::PSW_USB_V, 30), (TriCoreReg::PSW_USB_SV, 29), (TriCoreReg::PSW_USB_AV, 28), (TriCoreReg::PSW_USB_SAV, 27)] {
        wr(uc, r, (v >> bit & 1) << 31);
    }
}

/// End the quantum after the current instruction if an interrupt is deliverable with `icr`
/// (what the native core does on its `IrqCheck` stop).
fn stop_if_irq_wins(hw: &mut Tc1791HardwareState, icr: u32) {
    if hw.irq_wins(icr) {
        hw.stop_requested = true;
        hw.fixes.irq_stops += 1;
    }
}

/// Execute `fix` at `pc` with ISA semantics. Returns the next PC, or `None` to let Unicorn
/// execute the instruction itself (cases that it handles correctly, e.g. traps).
pub(crate) fn execute(uc: *mut uc_engine, hw: &mut Tc1791HardwareState, pc: u32, fix: Fix) -> Option<u32> {
    let d = |n: u32| rd(uc, TriCoreReg::d(n));
    // Privileged instructions require PSW.IO == 2 (Supervisor, TriCore V1 §6.3.2 Class 1 TIN 1 PRIV).
    if matches!(
        fix,
        Fix::Bisr { .. }
            | Fix::DisableD { .. }
            | Fix::Mtcr { .. }
            | Fix::MtcrProtected { .. }
            | Fix::MtcrIcr { .. }
            | Fix::Enable
            | Fix::Restore { .. }
    ) && (rd(uc, TriCoreReg::PSW) >> 10) & 3 != 2
    {
        return None;
    }
    match fix {
        Fix::ImaskRcrw { c, d: dp, width, const4 } => {
            let pos = d(dp) & 0x1F;
            let mask = (((1u64 << width) - 1) << pos) as u32;
            let lo = ((const4 as u64) << pos) as u32;
            let c = c & !1;
            wr(uc, TriCoreReg::d(c), lo);
            wr(uc, TriCoreReg::d(c + 1), mask);
        }
        Fix::InsertRcrw { c, a, d: dp, width, const4 } => {
            let pos = d(dp) & 0x1F;
            let mask = (((1u64 << width) - 1) << pos) as u32;
            let v = (d(a) & !mask) | (((const4 as u64) << pos) as u32 & mask);
            wr(uc, TriCoreReg::d(c), v);
        }
        Fix::DextrRrrr { c, a, b, d: dp } => {
            let pos = d(dp) & 0x1F;
            let wide = ((d(a) as u64) << 32) | d(b) as u64;
            wr(uc, TriCoreReg::d(c), ((wide << pos) >> 32) as u32);
        }
        Fix::DextrRrpwPos0 { c, a } => {
            let v = d(a);
            wr(uc, TriCoreReg::d(c), v);
        }
        Fix::Bisr { .. } | Fix::SvlcxTc13 => {
            let fcx = rd(uc, TriCoreReg::FCX);
            if fcx & 0x000F_FFFF == 0 || fcx == rd(uc, TriCoreReg::LCX) {
                return None; // FCU/FCD trap: Unicorn raises it
            }
            let ea = csa_ea(fcx);
            let mut link = [0u8; 4];
            // SAFETY: valid engine, buffer of matching size.
            if unsafe { uc_mem_read(uc, ea as u64, link.as_mut_ptr().cast(), 4) } != UC_ERR_OK {
                return None;
            }
            let new_fcx = u32::from_le_bytes(link);
            let pcxi = rd(uc, TriCoreReg::PCXI);
            let lower = [
                pcxi,
                rd(uc, TriCoreReg::A11),
                rd(uc, TriCoreReg::A2),
                rd(uc, TriCoreReg::A3),
                d(0),
                d(1),
                d(2),
                d(3),
                rd(uc, TriCoreReg::A4),
                rd(uc, TriCoreReg::A5),
                rd(uc, TriCoreReg::A6),
                rd(uc, TriCoreReg::A7),
                d(4),
                d(5),
                d(6),
                d(7),
            ];
            let mut bytes = [0u8; 64];
            for (i, w) in lower.iter().enumerate() {
                bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
            }
            // SAFETY: as above.
            if unsafe { uc_mem_write(uc, ea as u64, bytes.as_ptr().cast(), 64) } != UC_ERR_OK {
                return None;
            }
            let icr = rd(uc, TriCoreReg::ICR);
            let new_pcxi = if hw.fixes.tc13 {
                (pcxi & 0xFFB0_0000) | (fcx & 0x000F_FFFF)
            } else {
                ((icr & 0xFF) << 24) | ((icr >> 8) & 1) << 23 | (pcxi & 0x0030_0000) | (fcx & 0x000F_FFFF)
            };
            wr(uc, TriCoreReg::PCXI, new_pcxi);
            wr(uc, TriCoreReg::FCX, (fcx & 0xFFF0_0000) | (new_fcx & 0x000F_FFFF));
            let Fix::Bisr { const9, len } = fix else { return Some(pc.wrapping_add(4)) };
            let icr = (icr & !0xFF) | 0x100 | (const9 & 0xFF);
            wr(uc, TriCoreReg::ICR, icr);
            stop_if_irq_wins(hw, icr);
            return Some(pc.wrapping_add(len));
        }
        Fix::DisableD { a } => {
            let icr = rd(uc, TriCoreReg::ICR);
            wr(uc, TriCoreReg::d(a), (icr >> 8) & 1);
            wr(uc, TriCoreReg::ICR, icr & !0x100);
        }
        Fix::Mfcr { c, csfr } => {
            let v = hw.fixes.csfr_read(csfr);
            wr(uc, TriCoreReg::d(c), v);
        }
        Fix::Mtcr { a, csfr } => {
            if csfr_endinit_protected(csfr) && hw.scu.endinit {
                hw.fixes.endinit_csfr_dropped += 1;
                hw.fixes.last_endinit_csfr_drop = (csfr, pc);
            } else {
                let v = d(a);
                hw.fixes.csfr_write(csfr, v);
            }
        }
        Fix::LdBuPreInc { a, b, off } => {
            let ea = rd(uc, TriCoreReg::a(b)).wrapping_add(off as u32);
            let v = if hw.is_mmio(ea) {
                hw.mmio_read_logged(ea, 1) as u8
            } else {
                let mut byte = [0u8; 1];
                // SAFETY: valid engine, 1-byte buffer.
                if unsafe { uc_mem_read(uc, ea as u64, byte.as_mut_ptr().cast(), 1) } != UC_ERR_OK {
                    return None; // unmapped: Unicorn raises the proper exception
                }
                byte[0]
            };
            wr(uc, TriCoreReg::a(b), ea);
            wr(uc, TriCoreReg::d(a), v as u32);
        }
        Fix::IcrSync => {
            // The hook call itself synchronised cpu_ICR; Unicorn executes the MFCR.
            hw.fixes.hits[fix.kind()] += 1;
            return None;
        }
        Fix::MtcrIcr { a } => {
            if (rd(uc, TriCoreReg::PSW) >> 10) & 3 != 2 {
                return None; // not supervisor mode: Unicorn's (non-)handling of the privilege trap
            }
            // ICR.PIPN (23:16) is read-only; the hook call synchronised cpu_ICR as for MFCR.
            let icr = rd(uc, TriCoreReg::ICR);
            let icr = (icr & ICR_PIPN) | (d(a) & !ICR_PIPN);
            if !hw.fixes.needed(&fix) && !hw.irq_wins(icr) {
                return None; // translated correctly and nothing to deliver: Unicorn executes it
            }
            wr(uc, TriCoreReg::ICR, icr);
            stop_if_irq_wins(hw, icr);
        }
        Fix::Enable | Fix::Restore { .. } => {
            let ie = if let Fix::Restore { a } = fix { (d(a) & 1) << 8 } else { 0x100 };
            let icr = (rd(uc, TriCoreReg::ICR) & !0x100) | ie;
            if !hw.irq_wins(icr) {
                return None; // nothing to deliver: Unicorn executes the instruction
            }
            wr(uc, TriCoreReg::ICR, icr);
            stop_if_irq_wins(hw, icr);
        }
        Fix::Rfe => {
            // RFE restores ICR.IE = PCXI.PIE and ICR.CCPN = PCXI.PCPN (the layout of the board's
            // interrupt entry, `stm_icu`). RFE ends the translation block, so Unicorn executes it
            // and the next block start sees the stop request: the quantum ends right after RFE.
            let pcxi = rd(uc, TriCoreReg::PCXI);
            stop_if_irq_wins(hw, ((pcxi >> 23) & 1) << 8 | pcxi >> 24);
            return None;
        }
        Fix::LdHSro { b, off4 } => {
            let ea = rd(uc, TriCoreReg::a(b)).wrapping_add(2 * off4);
            let v = if hw.is_mmio(ea) {
                hw.mmio_read_logged(ea, 2) as u16
            } else {
                let mut h = [0u8; 2];
                // SAFETY: valid engine, 2-byte buffer.
                if unsafe { uc_mem_read(uc, ea as u64, h.as_mut_ptr().cast(), 2) } != UC_ERR_OK {
                    return None; // unmapped: Unicorn raises the exception (at its own EA)
                }
                u16::from_le_bytes(h)
            };
            wr(uc, TriCoreReg::D15, v as i16 as i32 as u32);
        }
        Fix::FpuConv { op2, a, b, c } => {
            let (res, psw) = tricore_core::groups::fpu_div::conversion(op2, d(a), d(b), psw_read(uc))?;
            wr(uc, TriCoreReg::d(c), res);
            psw_write(uc, psw);
        }
        Fix::MtcrProtected { csfr } => {
            if !hw.scu.endinit {
                return None;
            }
            hw.fixes.endinit_csfr_dropped += 1;
            hw.fixes.last_endinit_csfr_drop = (csfr, pc);
        }
    }
    Some(pc.wrapping_add(fix.len()))
}

/// Code hook installed on each fix site.
pub(crate) extern "C" fn hook_isa_fix(uc: *mut uc_engine, address: u64, _size: u32, ud: *mut c_void) {
    // SAFETY: `ud` is the board's hardware state; only one callback runs at a time and no Rust
    // borrow of the state is live while Unicorn executes.
    let hw = unsafe { &mut *(ud as *mut Tc1791HardwareState) };
    let pc = address as u32;
    let Some(code) = hw.code_bytes(pc) else { return };
    if code.len() < 2 {
        return;
    }
    let lo = u16::from_le_bytes([code[0], code[1]]);
    let hi = if code.len() >= 4 { u16::from_le_bytes([code[2], code[3]]) } else { 0 };
    let Some(fix) = site(lo, hi, &hw.fixes) else { return };
    let next = pc.wrapping_add(fix.len());
    // `hook_block` always counts the full block up front (`quantum_executed += n`) before any
    // fix-hooked block runs (even when `exact_timing` is enabled, because `classify_block` marks
    // blocks with fix sites slow). Subtract the unexecuted tail `[next .. tb_end)` when `execute`
    // writes `PC = next` and abandons the rest of the TB; add it back if `execute` defers (`None`).
    let tail = hw.insns_between(next, hw.fixes.tb_end).min(hw.quantum_executed);
    hw.quantum_executed -= tail;
    if let Some(next) = execute(uc, hw, pc, fix) {
        hw.fixes.hits[fix.kind()] += 1;
        wr(uc, TriCoreReg::PC, next);
        if hw.stop_requested {
            // An interrupt became deliverable: end the quantum before the next instruction.
            // SAFETY: valid engine inside a callback.
            unsafe { uc_emu_stop(uc) };
        }
    } else {
        hw.quantum_executed += tail;
    }
}

#[cfg(all(test, feature = "unicorn"))]
mod tests {
    use super::*;
    use crate::memory::BoardVariant;
    use crate::Simos18Board;

    const CODE: u32 = 0xC000_0000;

    /// Run exactly one instruction on a bare Unicorn TriCore (no fixes) and return D0..D15.
    fn raw_unicorn(insn: &[u8], regs: &[(TriCoreReg, u32)]) -> Result<[u32; 16], uc_err> {
        raw_unicorn_mem(insn, regs, &[], &mut [], CODE + insn.len() as u32)
    }

    /// [`raw_unicorn`] with DSPR contents `mem` (address, bytes), the registers `out` read back
    /// and an explicit end address.
    fn raw_unicorn_mem(insn: &[u8], regs: &[(TriCoreReg, u32)], mem: &[(u32, &[u8])], out: &mut [(TriCoreReg, u32)], until: u32) -> Result<[u32; 16], uc_err> {
        unsafe {
            let mut uc: *mut uc_engine = std::ptr::null_mut();
            assert_eq!(uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc), UC_ERR_OK);
            assert_eq!(uc_mem_map(uc, CODE as u64, 0x4000, UC_PROT_ALL as u32), UC_ERR_OK);
            assert_eq!(uc_mem_map(uc, 0xD000_0000, 0x4000, UC_PROT_ALL as u32), UC_ERR_OK);
            uc_mem_write(uc, CODE as u64, insn.as_ptr().cast(), insn.len());
            for (a, b) in mem {
                uc_mem_write(uc, *a as u64, b.as_ptr().cast(), b.len());
            }
            for &(r, v) in regs {
                wr(uc, r, v);
            }
            let e = uc_emu_start(uc, CODE as u64, until as u64, 0, 0);
            let mut d = [0u32; 16];
            for (i, o) in d.iter_mut().enumerate() {
                *o = rd(uc, TriCoreReg::d(i as u32));
            }
            for (r, v) in out.iter_mut() {
                *v = rd(uc, *r);
            }
            uc_close(uc);
            if e != UC_ERR_OK {
                return Err(e);
            }
            Ok(d)
        }
    }

    /// `SIMEMU_FIX_*` mask of the linked Unicorn (0 = stock).
    fn lib_fixes() -> u32 {
        unsafe {
            let mut uc: *mut uc_engine = std::ptr::null_mut();
            uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc);
            let m = uc_ctl_simemu_isa_fixes(uc);
            uc_close(uc);
            m
        }
    }

    /// Run the instruction on the board core (fix discovery active) and return D0..D15.
    fn fixed(insn: &[u8], regs: &[(TriCoreReg, u32)]) -> (Simos18Board, [u32; 16]) {
        let mut b = Simos18Board::new_variant_on(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791, crate::CpuBackend::Unicorn)
            .unwrap();
        b.write_mem(CODE, insn).unwrap();
        for &(r, v) in regs {
            b.write_reg(r, v);
        }
        b.run_until(CODE, CODE + insn.len() as u32).unwrap();
        let mut out = [0u32; 16];
        for (i, o) in out.iter_mut().enumerate() {
            *o = b.read_reg(TriCoreReg::d(i as u32));
        }
        (b, out)
    }

    #[test]
    fn imask_rcrw_register_swap_bug() {
        // imask e2, #1, d0, #1 (firmware 0x800A30D8): E2 = {1 << D0, 1 << D0}
        let insn = 0x2021_10D7u32.to_le_bytes();
        let regs = [(TriCoreReg::D0, 5), (TriCoreReg::D1, 0x1111), (TriCoreReg::D2, 9), (TriCoreReg::D3, 0x3333)];
        let raw = raw_unicorn(&insn, &regs).unwrap();
        assert_ne!((raw[0], raw[1], raw[2], raw[3]), (5, 0x1111, 1 << 5, 1 << 5), "unicorn bug reproduced");
        assert_eq!((raw[0], raw[1]), (1 << 9, 1 << 9), "unicorn uses D2 as position and writes E0");
        let (_, f) = fixed(&insn, &regs);
        assert_eq!((f[0], f[1], f[2], f[3]), (5, 0x1111, 1 << 5, 1 << 5));
        // imask e2, #2, d15, #2: Unicorn writes cpu_gpr_d[16]
        let insn = 0x2F22_20D7u32.to_le_bytes();
        let regs = [(TriCoreReg::D15, 4), (TriCoreReg::D2, 0)];
        let (_, f) = fixed(&insn, &regs);
        assert_eq!((f[2], f[3], f[15]), (2 << 4, 3 << 4, 4));
    }

    #[test]
    fn ld_bu_pre_increment_sign_extension_bug() {
        // ld.bu d2, [+a4]1  → BO: off10=1 (bits 21:16 = 1), op2 0x11, b=4, a=2, op1 0x09
        let w: u32 = (0x11 << 22) | (1 << 16) | (4 << 12) | (2 << 8) | 0x09;
        let insn = w.to_le_bytes();
        let regs = [(TriCoreReg::A4, 0xD000_0010), (TriCoreReg::D2, 0)];
        // byte at 0xD0000011 = 0x9C (negative as i8)
        let raw = unsafe {
            let mut uc: *mut uc_engine = std::ptr::null_mut();
            assert_eq!(uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc), UC_ERR_OK);
            uc_mem_map(uc, CODE as u64, 0x4000, UC_PROT_ALL as u32);
            uc_mem_map(uc, 0xD000_0000, 0x4000, UC_PROT_ALL as u32);
            uc_mem_write(uc, CODE as u64, insn.as_ptr().cast(), 4);
            uc_mem_write(uc, 0xD000_0011, [0x9Cu8].as_ptr().cast(), 1);
            for &(r, v) in &regs {
                wr(uc, r, v);
            }
            uc_emu_start(uc, CODE as u64, (CODE + 4) as u64, 0, 0);
            let v = (rd(uc, TriCoreReg::D2), rd(uc, TriCoreReg::A4));
            uc_close(uc);
            v
        };
        assert_eq!(raw, (0xFFFF_FF9C, 0xD000_0011), "unicorn sign-extends LD.BU pre-increment");
        let mut b = Simos18Board::new_variant_on(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791, crate::CpuBackend::Unicorn).unwrap();
        b.write_mem(CODE, &insn).unwrap();
        b.write_mem(0xD000_0011, &[0x9C]).unwrap();
        for &(r, v) in &regs {
            b.write_reg(r, v);
        }
        b.run_until(CODE, CODE + 4).unwrap();
        assert_eq!((b.read_reg(TriCoreReg::D2), b.read_reg(TriCoreReg::A4)), (0x9C, 0xD000_0011));
    }

    #[test]
    fn insert_rcrw_register_swap_bug() {
        // insert d3, d4, #5, d6, #4
        let insn = 0x3604_54D7u32.to_le_bytes();
        let regs = [(TriCoreReg::D3, 0), (TriCoreReg::D4, 0xFFFF_FFFF), (TriCoreReg::D6, 8)];
        let want = !(0xFu32 << 8) | (5 << 8);
        let raw = raw_unicorn(&insn, &regs).unwrap();
        assert_ne!(raw[3], want, "unicorn bug reproduced");
        let (_, f) = fixed(&insn, &regs);
        assert_eq!((f[3], f[6]), (want, 8));
    }

    #[test]
    fn dextr_rrrr_pos_zero_bug() {
        // dextr d4, d3, d2, d1 (firmware 0x8008450A) with D1 = 0 -> D4 = D3
        let insn = 0x4180_2317u32.to_le_bytes();
        let regs = [(TriCoreReg::D1, 0), (TriCoreReg::D2, 0x0000_00F0), (TriCoreReg::D3, 0x1234_0000)];
        let raw = raw_unicorn(&insn, &regs).unwrap();
        assert_ne!(raw[4], 0x1234_0000, "unicorn bug reproduced");
        let (_, f) = fixed(&insn, &regs);
        assert_eq!(f[4], 0x1234_0000);
        // non-zero position is still the 64-bit funnel shift
        let regs = [(TriCoreReg::D1, 8), (TriCoreReg::D2, 0xAB00_0000), (TriCoreReg::D3, 0x0012_3456)];
        let (_, f) = fixed(&insn, &regs);
        assert_eq!(f[4], 0x1234_56AB);
    }

    #[test]
    fn dextr_rrpw_pos_zero() {
        // dextr d5, d6, d7, #0 -> D5 = D6
        let insn = 0x5000_7677u32.to_le_bytes();
        let regs = [(TriCoreReg::D6, 0xCAFE_0001), (TriCoreReg::D7, 0x0000_0F0F)];
        let raw = raw_unicorn(&insn, &regs).unwrap();
        assert_eq!(raw[5], 0xCAFE_0F0F, "unicorn bug reproduced: D6 | D7 instead of D6");
        let (_, f) = fixed(&insn, &regs);
        assert_eq!(f[5], 0xCAFE_0001);
    }

    #[test]
    fn bisr_sets_ccpn_instead_of_or() {
        // bisr #5 with ICR.CCPN = 3: ISA -> CCPN = 5, Unicorn -> 3 | 5 = 7
        let insn = [0xE0u8, 0x05];
        let link = (0xD000_0040u32 >> 6) & 0xFFFF | ((0xD000_0040u32 >> 12) & 0xF_0000);
        let regs = [(TriCoreReg::ICR, 0x103), (TriCoreReg::FCX, link), (TriCoreReg::LCX, 0)];
        let unicorn_ccpn = unsafe {
            let mut uc: *mut uc_engine = std::ptr::null_mut();
            uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc);
            uc_mem_map(uc, CODE as u64, 0x4000, UC_PROT_ALL as u32);
            uc_mem_map(uc, 0xD000_0000, 0x4000, UC_PROT_ALL as u32);
            uc_mem_write(uc, CODE as u64, insn.as_ptr().cast(), 2);
            for &(r, v) in &regs {
                wr(uc, r, v);
            }
            uc_emu_start(uc, CODE as u64, (CODE + 2) as u64, 0, 0);
            let icr = rd(uc, TriCoreReg::ICR);
            uc_close(uc);
            icr & 0xFF
        };
        assert_eq!(unicorn_ccpn, 7, "unicorn bug reproduced");
        let (b, _) = fixed(&insn, &regs);
        let icr = b.read_reg(TriCoreReg::ICR);
        assert_eq!(icr & 0x1FF, 0x105);
        let pcxi = b.read_reg(TriCoreReg::PCXI);
        assert_eq!(pcxi >> 24, 3, "PCXI.PCPN = old CCPN");
        assert_eq!(pcxi & PCXI_UL_BIT, 0, "lower context");
        assert_eq!(pcxi & 0xF_FFFF, link);
    }
    const PCXI_UL_BIT: u32 = 1 << 22;

    #[test]
    fn mfcr_unimplemented_csfr_and_pcon1_self_clear() {
        // mtcr #0x9204(PCON1), d15 ; mfcr d14, #0x9204
        let mtcr = (0x9204u32 << 12) | (15 << 8) | 0xCD;
        let mfcr = (14u32 << 28) | (0x9204 << 12) | 0x4D;
        let mut insn = mtcr.to_le_bytes().to_vec();
        insn.extend_from_slice(&mfcr.to_le_bytes());
        let regs = [(TriCoreReg::D15, 3), (TriCoreReg::D14, 0xDEAD)];
        let raw = raw_unicorn(&insn, &regs).unwrap();
        assert_eq!(raw[14], 0xDEAD, "unicorn leaves MFCR destination unchanged");
        let (_, f) = fixed(&insn, &regs);
        assert_eq!(f[14], 0, "PCINV/PBINV self-clear");
    }

    #[test]
    fn disable_d_is_executed() {
        // disable d3 (TriCore 1.6): D3 = ICR.IE, ICR.IE = 0
        let insn = 0x03C0_030Du32.to_le_bytes();
        let regs = [(TriCoreReg::ICR, 0x100), (TriCoreReg::D3, 7)];
        // Unicorn raises an illegal-opcode trap (vectoring to the unmapped BTV in this harness)
        assert!(raw_unicorn(&insn, &regs).is_err(), "unicorn does not decode DISABLE D[a]");
        let (b, f) = fixed(&insn, &regs);
        assert_eq!(f[3], 1);
        assert_eq!(b.read_reg(TriCoreReg::ICR) & 0x100, 0);
    }

    #[test]
    fn icr_mfcr_mtcr_see_enable_disable_in_same_block() {
        const ENABLE: u32 = 0x0300_000D;
        const DISABLE: u32 = 0x0340_000D;
        const MFCR_D1_ICR: u32 = 0x1FE2_C04D; // mfcr d1, #0xFE2C
        const MTCR_ICR_D0: u32 = 0x0FE2_C0CD; // mtcr #0xFE2C, d0
        let code = |w: &[u32]| w.iter().flat_map(|w| w.to_le_bytes()).collect::<Vec<u8>>();
        // (sequence in one block, ICR before, D0) -> (D1, ICR after) per ISA. The bug needs a
        // later ICR op in the block (otherwise TCG stores the global right away).
        let cases = [
            // back-to-back DVSTEP wrappers: `enable` ends one, `mfcr d1, ICR; disable` opens the next
            (code(&[ENABLE, MFCR_D1_ICR, DISABLE]), 0x000, 0, (0x100, 0x000)),
            (code(&[DISABLE, MFCR_D1_ICR, ENABLE]), 0x105, 0, (0x005, 0x105)),
            (code(&[ENABLE, MTCR_ICR_D0, ENABLE]), 0x000, 0x003, (0, 0x103)),
            (code(&[DISABLE, MTCR_ICR_D0, DISABLE]), 0x100, 0x105, (0, 0x005)),
        ];
        for (insn, icr, d0, want) in cases {
            let raw = unsafe {
                let mut uc: *mut uc_engine = std::ptr::null_mut();
                uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc);
                uc_mem_map(uc, CODE as u64, 0x4000, UC_PROT_ALL as u32);
                uc_mem_write(uc, CODE as u64, insn.as_ptr().cast(), insn.len());
                for (r, v) in [(TriCoreReg::PSW, 0xB80), (TriCoreReg::ICR, icr), (TriCoreReg::D0, d0), (TriCoreReg::D1, 0)] {
                    wr(uc, r, v);
                }
                uc_emu_start(uc, CODE as u64, (CODE + insn.len() as u32) as u64, 0, 0);
                let v = (rd(uc, TriCoreReg::D1), rd(uc, TriCoreReg::ICR));
                uc_close(uc);
                v
            };
            if lib_fixes() & SIMEMU_FIX_ICR == 0 {
                assert_ne!(raw, want, "unicorn bug reproduced: {insn:02X?}");
            } else {
                assert_eq!(raw, want, "fixed in the patched library: {insn:02X?}");
            }
            let (b, f) = fixed(&insn, &[(TriCoreReg::ICR, icr), (TriCoreReg::D0, d0), (TriCoreReg::D1, 0)]);
            assert_eq!((f[1], b.read_reg(TriCoreReg::ICR)), want, "{insn:02X?}");
        }
    }

    /// U3: SRO `LD.H D15, [A[b]]off4` (V2 LD.H: EA = A[b] + zero_ext(2 * off4), sign-extended).
    #[test]
    fn sro_ld_h_scales_off4() {
        let insn = 0x438Cu16.to_le_bytes(); // ld.h d15, [a4]6 (SRO: b = 15:12, off4 = 11:8 = 3)
        let data: &[u8] = &[0, 0x11, 0x22, 0x33, 0x44, 0x55, 0x34, 0x92];
        let regs = [(TriCoreReg::A4, 0xD000_0000), (TriCoreReg::D15, 0)];
        let raw = raw_unicorn_mem(&insn, &regs, &[(0xD000_0000, data)], &mut [], CODE + 2).unwrap();
        let want = 0xFFFF_9234;
        if lib_fixes() & SIMEMU_FIX_SRO_LD_H == 0 {
            assert_eq!(raw[15], 0x4433, "unicorn bug reproduced: EA = A[b] + off4");
        } else {
            assert_eq!(raw[15], want, "fixed in the patched library");
        }
        let mut b = Simos18Board::new_variant_on(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791, crate::CpuBackend::Unicorn).unwrap();
        b.write_mem(CODE, &insn).unwrap();
        b.write_mem(0xD000_0000, data).unwrap();
        for &(r, v) in &regs {
            b.write_reg(r, v);
        }
        b.run_until(CODE, CODE + 2).unwrap();
        assert_eq!(b.read_reg(TriCoreReg::D15), want);
        // off4 = 0 is translated correctly and needs no hook
        assert_eq!(decode(0x408C, 0), None);
    }

    /// U2: MTCR ICR leaves the read-only ICR.PIPN alone.
    #[test]
    fn mtcr_icr_keeps_pipn() {
        let insn = 0x0FE2_C0CDu32.to_le_bytes(); // mtcr #0xFE2C, d0
        let regs = [(TriCoreReg::ICR, 0x0005_0103), (TriCoreReg::D0, 0x00AA_0107)];
        let mut icr = [(TriCoreReg::ICR, 0)];
        raw_unicorn_mem(&insn, &regs, &[], &mut icr, CODE + 4).unwrap();
        if lib_fixes() & SIMEMU_FIX_ICR == 0 {
            assert_eq!(icr[0].1, 0x00AA_0107, "unicorn bug reproduced: PIPN written");
        } else {
            assert_eq!(icr[0].1, 0x0005_0107, "fixed in the patched library");
        }
        let (b, _) = fixed(&insn, &regs);
        assert_eq!(b.read_reg(TriCoreReg::ICR), 0x0005_0107);
    }

    /// U1: `psw_read` (here via MFCR PSW) keeps PSW[24] = RM[0]. Not correctable from hooks: with
    /// stock Unicorn the board shows the defect too.
    #[test]
    fn psw_read_keeps_rm0() {
        let insn = 0x0FE0_404Du32.to_le_bytes(); // mfcr d0, #0xFE04 (PSW)
        let regs = [(TriCoreReg::PSW, 0x0300_0B80), (TriCoreReg::D0, 0)];
        let raw = raw_unicorn(&insn, &regs).unwrap();
        let (_, f) = fixed(&insn, &regs);
        if lib_fixes() & SIMEMU_FIX_PSW_RM == 0 {
            assert_eq!((raw[0], f[0]), (0x0200_0B80, 0x0200_0B80), "unicorn bug reproduced: RM 11 -> 10");
        } else {
            assert_eq!((raw[0], f[0]), (0x0300_0B80, 0x0300_0B80));
        }
    }

    /// U4: FPU results follow PSW.RM (V1 11.3.2). 1.5 * (1 + 2^-23) lies exactly between two
    /// floats: nearest-even rounds up to 0x3FC00002, round-toward-zero (RM = 11) gives 0x3FC00001.
    /// Not correctable from hooks: with stock Unicorn the board shows the defect too.
    #[test]
    fn fpu_rounding_follows_psw_rm() {
        let code = |w: &[u32]| w.iter().flat_map(|w| w.to_le_bytes()).collect::<Vec<u8>>();
        // mtcr #0xFE04, d3 ; mul.f d2, d0, d1
        let insn = code(&[0x0FE0_43CD, 0x2041_104B]);
        let regs = [(TriCoreReg::D0, 0x3FC0_0000), (TriCoreReg::D1, 0x3F80_0001), (TriCoreReg::D3, 0x0300_0B80)];
        let raw = raw_unicorn(&insn, &regs).unwrap();
        let (_, f) = fixed(&insn, &regs);
        if lib_fixes() & SIMEMU_FIX_FPU_RM == 0 {
            assert_ne!(raw[2], 0x3FC0_0001, "unicorn bug reproduced");
            assert_eq!(f[2], raw[2], "board shows the library defect");
        } else {
            assert_eq!((raw[2], f[2]), (0x3FC0_0001, 0x3FC0_0001));
            let (_, f) = fixed(&insn, &[(TriCoreReg::D0, 0x3FC0_0000), (TriCoreReg::D1, 0x3F80_0001), (TriCoreReg::D3, 0x0000_0B80)]);
            assert_eq!(f[2], 0x3FC0_0002, "RM = 00: nearest-even");
        }
    }

    /// U5: UPDFL sets PSW[31:24] = (PSW[31:24] & ~D[a][15:8]) | (D[a][7:0] & D[a][15:8]) (V2 UPDFL),
    /// including RM. Fixed only in the patched library.
    #[test]
    fn updfl_updates_masked_psw_bits_only() {
        let code = |w: &[u32]| w.iter().flat_map(|w| w.to_le_bytes()).collect::<Vec<u8>>();
        // updfl d3 ; mfcr d0, #0xFE04 (PSW)
        let insn = code(&[0x80C1_034B, 0x0FE0_404D]);
        // RM 11 -> 01 (mask 0x03, value 0x01); FS (C) and FX outside the mask stay set
        let regs = [(TriCoreReg::PSW, 0x0700_0B80), (TriCoreReg::PSW_USB_C, 1 << 31), (TriCoreReg::D3, 0x0301)];
        let raw = raw_unicorn(&insn, &regs).unwrap();
        let (_, f) = fixed(&insn, &regs);
        assert_eq!(f[0], raw[0], "no hook: the board shows the library's result");
        if lib_fixes() & SIMEMU_FIX_UPDFL == 0 {
            assert_ne!(raw[0] & 0x8700_0000, 0x8500_0000, "unicorn bug reproduced: {:#010x}", raw[0]);
        } else {
            assert_eq!(raw[0], 0x8500_0B80);
        }
    }

    /// U6: RET keeps PSW[25:24] (RM) on TC1.3.1/TC1.6 (V2 RET). Fixed only in the patched library.
    #[test]
    fn ret_keeps_psw_rm() {
        let code = |w: &[u32]| w.iter().flat_map(|w| w.to_le_bytes()).collect::<Vec<u8>>();
        // call +12 ; mfcr d0, PSW ; (end) nop nop ; callee: mtcr PSW, d3 (RM 11 -> 00) ; ret
        let insn = code(&[0x0006_006D, 0x0FE0_404D, 0, 0x0FE0_43CD, 0x9000]);
        let link = (0xD000_0040u32 >> 6) & 0xFFFF | ((0xD000_0040u32 >> 12) & 0xF_0000);
        let regs = [(TriCoreReg::PSW, 0x0300_0B80), (TriCoreReg::FCX, link), (TriCoreReg::D3, 0x0B81)]; // callee CDC = 1
        let raw = raw_unicorn_mem(&insn, &regs, &[], &mut [], CODE + 8).unwrap()[0];
        let mut b = Simos18Board::new_variant_on(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791, crate::CpuBackend::Unicorn).unwrap();
        b.write_mem(CODE, &insn).unwrap();
        for &(r, v) in &regs {
            b.write_reg(r, v);
        }
        b.run_until(CODE, CODE + 8).unwrap();
        assert_eq!(b.read_reg(TriCoreReg::D0), raw, "no hook: the board shows the library's result");
        if lib_fixes() & SIMEMU_FIX_RET_RM == 0 {
            assert_ne!(raw & 0x0300_0000, 0, "unicorn bug reproduced: RM restored from the CSA {raw:#010x}");
        } else {
            assert_eq!(raw & 0x0300_0000, 0, "RM = 00 set by the callee survives RET: {raw:#010x}");
        }
    }

    /// B3: a hook that executes an instruction mid-block makes Unicorn abandon the rest of the
    /// block; only the executed instructions are counted.
    #[test]
    fn fix_hook_mid_block_counts_executed_instructions() {
        // nop ; dextr d4, d3, d2, d1 (D1 = 0: fix site) ; nop ; nop ; nop
        let mut insn = vec![0u8, 0];
        insn.extend(0x4180_2317u32.to_le_bytes());
        insn.extend([0u8; 6]);
        let (b, f) = fixed(&insn, &[(TriCoreReg::D1, 0), (TriCoreReg::D3, 0x1234_0000)]);
        assert_eq!(f[4], 0x1234_0000);
        assert_eq!(b.hw().fixes.hits[2], 1);
        assert_eq!(b.hw().total_instructions, 5);
    }

    /// A pending interrupt is taken at the instruction boundary where ENABLE / RESTORE / RFE makes
    /// it deliverable, on both backends (native core: `IrqCheck` stop; Unicorn: [`irq_site`] hook).
    /// The interrupted code counts in D15 (`add d15, #1`); the ISR copies D15 to D4 as its first
    /// instruction, so D4 = instructions of the interrupted code run past the unmasking point.
    /// Without the [`irq_site`] hooks Unicorn ran on to the quantum end (D4 > 0 or ISR not reached).
    #[test]
    fn interrupt_taken_right_after_unmasking() {
        const ADD: [u8; 2] = 0x1FC2u16.to_le_bytes(); // add d15, #1
        const BIV: u32 = 0xC000_1000;
        let words = |w: &[u32]| w.iter().flat_map(|w| w.to_le_bytes()).collect::<Vec<u8>>();
        let run = |backend, main: &[u8], icr: u32, prios: &[u32], vectors: &[(u32, Vec<u8>)]| {
            let mut b = Simos18Board::new_variant_on(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791, backend).unwrap();
            let mut code = main.to_vec();
            (0..24).for_each(|_| code.extend(ADD));
            b.write_mem(CODE, &code).unwrap();
            for (p, v) in vectors {
                b.write_mem(BIV + (p << 5), v).unwrap();
            }
            let csa = 0xD000_1000u32;
            let link = |ea: u32| ((ea >> 12) & 0xF_0000) | ((ea >> 6) & 0xFFFF);
            for k in 0..3 {
                b.write_mem(csa + 64 * k, &link(csa + 64 * (k + 1)).to_le_bytes()).unwrap();
            }
            for (r, v) in [
                (TriCoreReg::FCX, link(csa)),
                (TriCoreReg::LCX, 0),
                (TriCoreReg::BIV, BIV),
                (TriCoreReg::ISP, 0xD000_8000),
                (TriCoreReg::A10, 0xD000_4000),
                (TriCoreReg::PSW, 0x0000_0B80),
                (TriCoreReg::ICR, icr),
                (TriCoreReg::D15, 0),
                (TriCoreReg::D4, 0xFFFF),
                (TriCoreReg::PC, CODE),
            ] {
                b.write_reg(r, v);
            }
            for (k, p) in prios.iter().enumerate() {
                b.hw_mut().mmio_write(0xF7E0_FFF0 + 4 * k as u32, 4, (crate::stm_icu::SRC_SETR | crate::stm_icu::SRC_SRE | p) as u64);
            }
            b.run_instructions(40).unwrap();
            b.read_reg(TriCoreReg::D4)
        };
        // mov d4, d15 (SRR 02H; D4 is lower context, which RFE does not restore) ; j . (SB 3CH)
        let copy = vec![0x02u8, 0xF4, 0x3C, 0x00];
        for backend in [crate::CpuBackend::Unicorn, crate::CpuBackend::Native] {
            // enable ; add... with a priority-9 request pending (IE = 0, CCPN = 2)
            assert_eq!(run(backend, &words(&[0x0300_000D]), 0x002, &[9], &[(9, copy.clone())]), 0, "{backend:?}: ENABLE");
            // mov d0, #1 (SRC 82H) ; restore d0 ; add...
            let mut main = vec![0x82, 0x10];
            main.extend(words(&[0x0380_000D]));
            assert_eq!(run(backend, &main, 0x002, &[9], &[(9, copy.clone())]), 0, "{backend:?}: RESTORE");
            // priority 9 taken first (IE, CCPN 2), its ISR is a lone RFE; the pending priority 5
            // must follow right after the RFE, before any instruction of the interrupted code
            let rfe = words(&[0x01C0_000D]);
            assert_eq!(run(backend, &[], 0x102, &[9, 5], &[(9, rfe.clone()), (5, copy.clone())]), 0, "{backend:?}: RFE");
            assert_eq!(run(backend, &[], 0x102, &[9, 5], &[(9, 0x8000u16.to_le_bytes().to_vec()), (5, copy.clone())]), 0, "{backend:?}: RFE (16-bit)");
            // mov d0, #0x102 (RLC 3BH) ; mtcr ICR, d0 (IE, CCPN 2) ; add... — hooked with a patched
            // library too (it translates MTCR ICR correctly, the interrupt check is still needed)
            assert_eq!(run(backend, &words(&[0x0010_203B, 0x0FE2_C0CD]), 0x002, &[9], &[(9, copy.clone())]), 0, "{backend:?}: MTCR ICR");
            // bisr #2 (RC ADH; IE, CCPN 2) ; add...
            assert_eq!(run(backend, &words(&[0x0000_20AD]), 0x002, &[9], &[(9, copy.clone())]), 0, "{backend:?}: BISR");
        }
    }

    /// Exact instruction timing (patched Unicorn, `uc_ctl_simemu_exact`; always on the native core):
    /// a quantum ends after exactly its budget even inside a translation block; a peripheral read
    /// sees the time of its own instruction, not of the block end; a store that requests an
    /// interrupt is followed by the interrupt at the next instruction, not at the block end.
    #[test]
    fn peripheral_access_timing_is_exact() {
        let lib_exact = Simos18Board::new_variant_on(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791, crate::CpuBackend::Unicorn)
            .unwrap()
            .hw()
            .exact_timing;
        let words = |w: &[u32]| w.iter().flat_map(|w| w.to_le_bytes()).collect::<Vec<u8>>();
        let nops = |n: usize| vec![0u8; 2 * n];
        const BIV: u32 = 0xC000_1000;
        for backend in [crate::CpuBackend::Unicorn, crate::CpuBackend::Native] {
            if backend == crate::CpuBackend::Unicorn && !lib_exact {
                continue; // stock library: block-granular timing (ARCHITECTURE.md §3.1)
            }
            let new = |code: &[u8]| {
                let mut b = Simos18Board::new_variant_on(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791, backend).unwrap();
                b.write_mem(CODE, code).unwrap();
                b.write_reg(TriCoreReg::PC, CODE);
                b
            };
            // 20 NOPs, one block: a 7-instruction budget stops after the 7th
            let mut b = new(&nops(20));
            b.run_instructions(7).unwrap();
            assert_eq!((b.hw().total_instructions, b.read_reg(TriCoreReg::PC)), (7, CODE + 14), "{backend:?}: exact budget");
            // 5 NOPs ; ld.w d1, [a15] (STM TIM0; BO 09H/24H) ; 5 NOPs ; st.w [a14], d2 (CPU SRC0 SETR,
            // BO 89H/24H) ; add d15, #1 ... — STM runs at fCPU after reset (1 tick per instruction)
            let mut code = nops(5);
            code.extend(words(&[0x0900_F109]));
            code.extend(nops(5));
            code.extend(words(&[0x0900_E289]));
            (0..24).for_each(|_| code.extend(0x1FC2u16.to_le_bytes()));
            let mut b = new(&code);
            // ISR 9: mov d4, d15 ; j .
            b.write_mem(BIV + (9 << 5), &[0x02, 0xF4, 0x3C, 0x00]).unwrap();
            let csa = 0xD000_1000u32;
            let link = |ea: u32| ((ea >> 12) & 0xF_0000) | ((ea >> 6) & 0xFFFF);
            b.write_mem(csa, &link(csa + 64).to_le_bytes()).unwrap();
            for (r, v) in [
                (TriCoreReg::FCX, link(csa)),
                (TriCoreReg::LCX, 0),
                (TriCoreReg::BIV, BIV),
                (TriCoreReg::ISP, 0xD000_8000),
                (TriCoreReg::ICR, 0x100),
                (TriCoreReg::A15, 0xF000_0210),
                (TriCoreReg::A14, 0xF7E0_FFF0),
                (TriCoreReg::D2, crate::stm_icu::SRC_SETR | crate::stm_icu::SRC_SRE | 9),
                (TriCoreReg::D15, 0),
                (TriCoreReg::D4, 0xFFFF),
            ] {
                b.write_reg(r, v);
            }
            b.run_instructions(40).unwrap();
            assert_eq!(b.read_reg(TriCoreReg::D1), 5, "{backend:?}: STM read at its own instruction");
            assert_eq!(b.read_reg(TriCoreReg::D4), 0, "{backend:?}: interrupt right after the requesting store");
        }
    }

    /// U7: FTOU / FTOQ31 / FTOQ31Z / Q31TOF are not decoded by Unicorn 2.1.4 (IOPC, class 2 TIN 1 —
    /// the DQ250 trap at 4319 0x800A637C / 4930 0x800938B8). Expected values per V131 ISA p. 3-479
    /// (FTOU) and V2 p. 3-399/3-400/3-409; PSW.RM V1 Table 11-3 (00 nearest-even, 01 toward +inf).
    #[test]
    fn fpu_conversions_unicorn_does_not_decode() {
        let rr = |op2: u32, b: u32| (2u32 << 28 | op2 << 20 | 1 << 16 | b << 12 | 1 << 8 | 0x4B).to_le_bytes();
        let ftou = rr(0x12, 0); // ftou d2, d1
        // (insn, D1, D3, PSW) -> (D2, PSW)
        let cases = [
            (ftou, 0x4111_4F4C, 0, 0x0000_0A80, 9, 0x8400_0A80),          // firmware operand 9.08: inexact
            (ftou, 0xBE80_0000, 0, 0x0000_0B80, 0, 0xC000_0B80),          // -0.25 < 0.0: 0, FI
            (ftou, 0x4F80_0000, 0, 0x0000_0B80, 0xFFFF_FFFF, 0xC000_0B80), // 2^32 > 2^32 - 1
            (ftou, 0x4020_0000, 0, 0x0000_0B80, 2, 0x8400_0B80),          // 2.5, RM 00: tie to even
            (ftou, 0x4020_0000, 0, 0x0100_0B80, 3, 0x8500_0B80),          // 2.5, RM 01: toward +inf
            (ftou, 0x4080_0000, 0, 0x0000_0B80, 4, 0x0000_0B80),          // exact: no flag
            (rr(0x15, 3), 0x4000_0000, 1, 0x0000_0B80, 0x3F80_0000, 0x0000_0B80), // q31tof 0.5 * 2^1
            (rr(0x11, 3), 0x3EAA_AAAB, 7, 0x0000_0B80, 0x0055_5556, 0x8400_0B80), // ftoq31 5592405.5: even
            (rr(0x18, 3), 0x3EAA_AAAB, 7, 0x0000_0B80, 0x0055_5555, 0x8400_0B80), // ftoq31z: toward zero
            (rr(0x11, 3), 0x3F80_0000, 0, 0x0000_0B80, 0x7FFF_FFFF, 0xC000_0B80), // 1.0 is not a Q31
        ];
        for (insn, d1, d3, psw, want, want_psw) in cases {
            let regs = [(TriCoreReg::D1, d1), (TriCoreReg::D3, d3), (TriCoreReg::PSW, psw)];
            assert!(raw_unicorn(&insn, &regs).is_err(), "unicorn bug reproduced: {insn:02X?} traps");
            let (b, f) = fixed(&insn, &regs);
            assert_eq!((f[2], b.psw()), (want, want_psw), "{insn:02X?} D1 {d1:#010x}");
        }
        assert_eq!(decode(0x014B, 0x2111), Some(Fix::FpuConv { op2: 0x11, a: 1, b: 0, c: 2 }));
        assert_eq!(decode(0x014B, 0x2131), None, "FTOIZ is translated by Unicorn");
    }

    #[test]
    fn decode_ignores_correct_forms() {
        assert!(matches!(decode(0x2317, 0x4180), Some(Fix::DextrRrrr { .. })));
        // dextr with a == b (rotate) is translated correctly by Unicorn
        assert_eq!(decode(0x3317, 0x4180), None);
        // mfcr of an implemented CSFR (FCX); ICR needs the TCG-global sync hook
        assert_eq!(decode(0x804D, 0xFFE3), None);
        assert_eq!(decode(0xC04D, 0xFFE2), Some(Fix::IcrSync));
    }

    #[test]
    fn privileged_fixes_do_not_bypass_user_mode_io_privilege() {
        let mut b = Simos18Board::new_variant_on(
            &vec![0u8; 0x40_0000],
            &vec![0u8; 0x4_0000],
            crate::memory::BoardVariant::Simos18Tc1791,
            crate::CpuBackend::Unicorn,
        )
        .unwrap();
        // PSW with IO = 01b (User-1 mode, bits 11:10 = 0x0400) instead of Supervisor (10b = 0x0800)
        b.set_psw(0x0000_0480);
        b.write_reg(TriCoreReg::D1, 0x0000_0105);
        b.write_reg(TriCoreReg::ICR, 0);
        let uc = b.uc;
        let hw = b.hw_mut();
        for fix in [
            Fix::MtcrIcr { a: 1 },
            Fix::Mtcr { a: 1, csfr: 0x9004 },
            Fix::MtcrProtected { csfr: 0xFE00 },
            Fix::Bisr { const9: 7, len: 4 },
            Fix::DisableD { a: 2 },
            Fix::Enable,
            Fix::Restore { a: 1 },
        ] {
            assert_eq!(execute(uc, hw, 0x8000_0000, fix), None, "user-mode {fix:?} must defer to CPU MIP trap");
        }
        assert_eq!(b.read_reg(TriCoreReg::ICR), 0, "user-mode MTCR/BISR/ENABLE must not mutate ICR");
    }
}
