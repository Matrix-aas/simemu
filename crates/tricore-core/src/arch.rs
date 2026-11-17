//! Architecture constants: core variants, traps, PSW/ICR/PCXI fields and the CSFR table.
//!
//! References (in `vault/docs/infineon/`):
//! * `tc16_architecture_vol1_a4.pdf` — TriCore V1.6 Core Architecture, User Manual Vol. 1, V1.0 2012-05 ("V1")
//! * `tcarch_vol2_a4.pdf` — TriCore V1.6 Instruction Set, User Manual Vol. 2, V1.0 2012-05 ("V2")
//! * `tc_v131_corearchitecture_v138.pdf` / `Tc_v131_instructionset_v138.pdf` — TriCore 1 V1.3 & V1.3.1
//!   Vol. 1 / Vol. 2, V1.3.8 2008-01 ("V131-1", "V131-2")

/// Core architecture revision. Selects instruction availability and the few behavioural deltas.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Arch {
    /// TriCore 1.3 (TC1766 / AUDO-NG, DQ250 TCU). Shares the TriCore 1 non-TC1.6 encoding space
    /// with [`Arch::Tc131`] (rejecting TC1.6-only BOL/MOV/MULMS.H/DIV/CACHEI.I forms; MMU `0x75`
    /// raises `UOPC`, `DVADJ` uses V131-2 p. 3-111), with the TriCore 1.3 (`1.3.0`) deltas:
    /// * `SVLCX` and `BISR` leave `PCXI.PCPN`/`PCXI.PIE` unchanged (`Cpu::save_finish`; UNVERIFIED
    ///   against the separate V131-2 addendum, derived from the DQ250 OS, `MILESTONE.md` §2.3);
    /// * `RET` restores the full `PSW` word from the CSA instead of preserving `PSW[25:24]`
    ///   (V131-2 p. 3-369 `1.3.0` row);
    /// * `DVINIT.B/.BU/.H/.HU` compute full 8-/16-bit quotient-range overflow in `PSW.V`/`SV`
    ///   (V131-2 pp. 3-114, 3-116 `1.3.0` rows);
    /// * `RFM` does not touch `DBGTCR.DTA` (V131-2 p. 3-373 `1.3.1` row; V131-1 Table 13-1).
    Tc13,
    /// TriCore 1.3.1 (TC1784 / AUDO Future, DQ200_G2 TCU).
    Tc131,
    /// TriCore 1.6 (TC1791 / AUDO MAX, SIMOS18.1).
    Tc16,
}

/// A trap: class and Trap Identification Number (V1 Table 6-1, p. 6-1..6-2).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Trap {
    pub class: u8,
    pub tin: u8,
}

impl Trap {
    pub const fn new(class: u8, tin: u8) -> Self {
        Self { class, tin }
    }
    // Class 0: MMU
    pub const VAF: Trap = Trap::new(0, 0);
    pub const VAP: Trap = Trap::new(0, 1);
    // Class 1: internal protection
    pub const PRIV: Trap = Trap::new(1, 1);
    pub const MPR: Trap = Trap::new(1, 2);
    pub const MPW: Trap = Trap::new(1, 3);
    pub const MPX: Trap = Trap::new(1, 4);
    pub const MPP: Trap = Trap::new(1, 5);
    pub const MPN: Trap = Trap::new(1, 6);
    pub const GRWP: Trap = Trap::new(1, 7);
    // Class 2: instruction errors
    pub const IOPC: Trap = Trap::new(2, 1);
    pub const UOPC: Trap = Trap::new(2, 2);
    pub const OPD: Trap = Trap::new(2, 3);
    pub const ALN: Trap = Trap::new(2, 4);
    pub const MEM: Trap = Trap::new(2, 5);
    // Class 3: context management
    pub const FCD: Trap = Trap::new(3, 1);
    pub const CDO: Trap = Trap::new(3, 2);
    pub const CDU: Trap = Trap::new(3, 3);
    pub const FCU: Trap = Trap::new(3, 4);
    pub const CSU: Trap = Trap::new(3, 5);
    pub const CTYP: Trap = Trap::new(3, 6);
    pub const NEST: Trap = Trap::new(3, 7);
    // Class 4: system bus and peripheral errors
    pub const PSE: Trap = Trap::new(4, 1);
    pub const DSE: Trap = Trap::new(4, 2);
    pub const DAE: Trap = Trap::new(4, 3);
    pub const CAE: Trap = Trap::new(4, 4);
    pub const PIE: Trap = Trap::new(4, 5);
    pub const DIE: Trap = Trap::new(4, 6);
    pub const TAE: Trap = Trap::new(4, 7);
    // Class 5: assertion
    pub const OVF: Trap = Trap::new(5, 1);
    pub const SOVF: Trap = Trap::new(5, 2);
    // Class 7: NMI
    pub const NMI: Trap = Trap::new(7, 0);

    /// Class 6: system call, TIN = SYSCALL const9 (V1 Table 6-1 note 1).
    pub const fn sys(tin: u8) -> Self {
        Trap::new(6, tin)
    }

    /// Asynchronous traps (V1 Table 6-1 "Synch./Asynch." column): DAE, CAE, DIE, TAE, NMI.
    pub const fn is_async(&self) -> bool {
        matches!((self.class, self.tin), (4, 3) | (4, 4) | (4, 6) | (4, 7) | (7, 0))
    }
}

/// PSW fields (V1 p. 3-5..3-7).
pub mod psw {
    pub const C: u32 = 1 << 31;
    pub const V: u32 = 1 << 30;
    pub const SV: u32 = 1 << 29;
    pub const AV: u32 = 1 << 28;
    pub const SAV: u32 = 1 << 27;
    /// FPU rounding mode `PSW[25:24]`; preserved by RET on TC1.3.1/TC1.6 (V131-2 p. 3-369, V2 p. 3-311).
    pub const RM: u32 = 3 << 24;
    pub const PRS: u32 = 3 << 12;
    pub const IO: u32 = 3 << 10;
    pub const IO_SUPERVISOR: u32 = 2 << 10;
    pub const IS: u32 = 1 << 9;
    pub const GW: u32 = 1 << 8;
    pub const CDE: u32 = 1 << 7;
    pub const CDC: u32 = 0x7F;
    /// Reset value (V1 p. 3-5).
    pub const RESET: u32 = 0x0000_0B80;
}

/// ICR fields (V1 p. 5-14..5-15). Same layout on TC1.3.1.
pub mod icr {
    pub const CCPN: u32 = 0xFF;
    pub const IE: u32 = 1 << 8;
    pub const PIPN_SHIFT: u32 = 16;
    pub const PIPN: u32 = 0xFF << 16;
}

/// PCXI fields (V1 p. 3-9 / 4-12: `PCPN [31:24]`, `PIE 23`, `UL 22`, `PCXS [19:16]`, `PCXO [15:0]`). Same layout on
/// TC1.3.1; FCX/LCX use the same 20-bit link-word format in `[19:0]`.
pub mod pcxi {
    pub const PCPN_SHIFT: u32 = 24;
    pub const PIE: u32 = 1 << 23;
    pub const UL: u32 = 1 << 22;
    pub const LINK: u32 = 0x000F_FFFF;
}

/// Effective address of the CSA named by a link word: `{PCXS/FCXS, 6'b0, PCXO/FCXO, 6'b0}`
/// (V2 CALL p. 3-63).
#[inline]
pub const fn csa_ea(link: u32) -> u32 {
    ((link & 0x000F_0000) << 12) | ((link & 0x0000_FFFF) << 6)
}

/// CSFR offsets of the registers the core keeps as named fields (V1 Table 13-2).
pub mod csfr {
    pub const PCXI: u16 = 0xFE00;
    pub const PSW: u16 = 0xFE04;
    pub const PC: u16 = 0xFE08;
    pub const SYSCON: u16 = 0xFE14;
    pub const CPU_ID: u16 = 0xFE18;
    pub const BIV: u16 = 0xFE20;
    pub const BTV: u16 = 0xFE24;
    pub const ISP: u16 = 0xFE28;
    pub const ICR: u16 = 0xFE2C;
    pub const FCX: u16 = 0xFE38;
    pub const LCX: u16 = 0xFE3C;
    pub const PCON1: u16 = 0x9204;
}

/// One entry of the CSFR table.
#[derive(Clone, Copy, Debug)]
pub struct CsfrDef {
    pub offset: u16,
    pub name: &'static str,
    /// Only writable while the system is in the initialisation state (V1 p. 3-1 "ENDINIT Protection").
    pub endinit: bool,
    /// Bits that are writable with MTCR (read-only registers: 0).
    pub write_mask: u32,
}

const fn r(offset: u16, name: &'static str) -> CsfrDef {
    CsfrDef { offset, name, endinit: false, write_mask: u32::MAX }
}
const fn e(offset: u16, name: &'static str) -> CsfrDef {
    CsfrDef { offset, name, endinit: true, write_mask: u32::MAX }
}
const fn ro(offset: u16, name: &'static str) -> CsfrDef {
    CsfrDef { offset, name, endinit: false, write_mask: 0 }
}
const fn m(offset: u16, name: &'static str, write_mask: u32) -> CsfrDef {
    CsfrDef { offset, name, endinit: false, write_mask }
}

/// CSFRs stored in the generic CSFR file (everything of V1 Table 13-2 except the GPRs and the
/// registers held as named [`crate::Cpu`] fields). ENDINIT list: V1 p. 3-1 (BTV, BIV, ISP, BMACON,
/// COMPAT, MIECON, PMA0, SMACON). An offset not in this table (nor a named register) is undefined:
/// MTCR has no effect, MFCR returns undefined data — modelled as 0 (V1 p. 3-18).
///
/// Implementation-specific registers (PCON0..2, DCON0..2, PMA0, ...) follow the TC1791 data sheet;
/// their reset values are supplied by the board (`Cpu::csfr_set_reset`).
pub const CSFR_TABLE: &[CsfrDef] = &[
    // Memory protection (V1 ch. 8)
    r(0xC000, "DPR0_0L"), r(0xC004, "DPR0_0U"), r(0xC008, "DPR0_1L"), r(0xC00C, "DPR0_1U"),
    r(0xC010, "DPR0_2L"), r(0xC014, "DPR0_2U"), r(0xC018, "DPR0_3L"), r(0xC01C, "DPR0_3U"),
    r(0xC400, "DPR1_0L"), r(0xC404, "DPR1_0U"), r(0xC408, "DPR1_1L"), r(0xC40C, "DPR1_1U"),
    r(0xC410, "DPR1_2L"), r(0xC414, "DPR1_2U"), r(0xC418, "DPR1_3L"), r(0xC41C, "DPR1_3U"),
    r(0xC800, "DPR2_0L"), r(0xC804, "DPR2_0U"), r(0xC808, "DPR2_1L"), r(0xC80C, "DPR2_1U"),
    r(0xC810, "DPR2_2L"), r(0xC814, "DPR2_2U"), r(0xC818, "DPR2_3L"), r(0xC81C, "DPR2_3U"),
    r(0xCC00, "DPR3_0L"), r(0xCC04, "DPR3_0U"), r(0xCC08, "DPR3_1L"), r(0xCC0C, "DPR3_1U"),
    r(0xCC10, "DPR3_2L"), r(0xCC14, "DPR3_2U"), r(0xCC18, "DPR3_3L"), r(0xCC1C, "DPR3_3U"),
    r(0xD000, "CPR0_0L"), r(0xD004, "CPR0_0U"), r(0xD008, "CPR0_1L"), r(0xD00C, "CPR0_1U"),
    r(0xD010, "CPR0_2L"), r(0xD014, "CPR0_2U"), r(0xD018, "CPR0_3L"), r(0xD01C, "CPR0_3U"),
    r(0xD400, "CPR1_0L"), r(0xD404, "CPR1_0U"), r(0xD408, "CPR1_1L"), r(0xD40C, "CPR1_1U"),
    r(0xD410, "CPR1_2L"), r(0xD414, "CPR1_2U"), r(0xD418, "CPR1_3L"), r(0xD41C, "CPR1_3U"),
    r(0xD800, "CPR2_0L"), r(0xD804, "CPR2_0U"), r(0xD808, "CPR2_1L"), r(0xD80C, "CPR2_1U"),
    r(0xD810, "CPR2_2L"), r(0xD814, "CPR2_2U"), r(0xD818, "CPR2_3L"), r(0xD81C, "CPR2_3U"),
    r(0xDC00, "CPR3_0L"), r(0xDC04, "CPR3_0U"), r(0xDC08, "CPR3_1L"), r(0xDC0C, "CPR3_1U"),
    r(0xDC10, "CPR3_2L"), r(0xDC14, "CPR3_2U"), r(0xDC18, "CPR3_3L"), r(0xDC1C, "CPR3_3U"),
    r(0xE000, "DPM0"), r(0xE080, "DPM1"), r(0xE100, "DPM2"), r(0xE180, "DPM3"),
    r(0xE200, "CPM0"), r(0xE280, "CPM1"), r(0xE300, "CPM2"), r(0xE380, "CPM3"),
    r(0xE400, "TPS_CON"), r(0xE404, "TPS_TIMER0"), r(0xE408, "TPS_TIMER1"),
    // MMU (V1 ch. 9; TC1.3.1 only in practice)
    r(0x8000, "MMU_CON"), r(0x8004, "MMU_ASI"), r(0x800C, "MMU_TVA"), r(0x8010, "MMU_TPA"),
    r(0x8014, "MMU_TPX"), r(0x8018, "MMU_TFA"), r(0x8020, "MMU_TFAS"),
    // Memory configuration / integrity (V1 ch. 14, implementation specific)
    r(0x9000, "DCON2"), e(0x9004, "BMACON"), r(0x9008, "DCON1"), e(0x900C, "SMACON"),
    r(0x9010, "DSTR"), r(0x9018, "DATR"), r(0x901C, "DEADD"), r(0x9020, "DIEAR"), r(0x9024, "DIETR"),
    r(0x9028, "CCDIER"), r(0x9040, "DCON0"), e(0x9044, "MIECON"),
    r(0x9200, "PSTR"), r(0x9204, "PCON1"), r(0x9208, "PCON2"), r(0x920C, "PCON0"),
    r(0x9210, "PIEAR"), r(0x9214, "PIETR"), r(0x9218, "CCPIER"),
    e(0x9400, "COMPAT"),
    e(0x801C, "PMA0"),
    // Debug (V1 ch. 12)
    r(0xFD00, "DBGSR"), r(0xFD08, "EXEVT"), r(0xFD0C, "CREVT"), r(0xFD10, "SWEVT"),
    r(0xF000, "TR0EVT"), r(0xF004, "TR0ADR"), r(0xF008, "TR1EVT"), r(0xF00C, "TR1ADR"),
    r(0xF010, "TR2EVT"), r(0xF014, "TR2ADR"), r(0xF018, "TR3EVT"), r(0xF01C, "TR3ADR"),
    r(0xF020, "TR4EVT"), r(0xF024, "TR4ADR"), r(0xF028, "TR5EVT"), r(0xF02C, "TR5ADR"),
    r(0xF030, "TR6EVT"), r(0xF034, "TR6ADR"), r(0xF038, "TR7EVT"), r(0xF03C, "TR7ADR"),
    r(0xFD30, "TRIG_ACC"), r(0xFD40, "DMS"), r(0xFD44, "DCX"), r(0xFD48, "DBGTCR"),
    // FPU (V1 §11.6 p. 11-11..11-14): FPU_TRAP_CON only FIE..FXE [22:18] are rw (TST/RM/FI..FX
    // rh; TCL write-only, see `Cpu::csfr_write`); the capture registers are rh.
    m(0xA000, "FPU_TRAP_CON", 0x1F << 18), ro(0xA004, "FPU_TRAP_PC"), ro(0xA008, "FPU_TRAP_OPC"),
    ro(0xA010, "FPU_TRAP_SRC1"), ro(0xA014, "FPU_TRAP_SRC2"), ro(0xA018, "FPU_TRAP_SRC3"),
    // Identification
    ro(0xFE18, "CPU_ID"),
];

/// `CSFR_INDEX[offset >> 2]` = index in [`CSFR_TABLE`] + 1 (0: not in the table), built at
/// compile time: MFCR/MTCR are frequent (interrupt locks), a linear search is not.
static CSFR_INDEX: [u8; 0x4000] = {
    let mut t = [0u8; 0x4000];
    let mut i = 0;
    while i < CSFR_TABLE.len() {
        assert!(CSFR_TABLE[i].offset & 3 == 0 && i < 255);
        t[(CSFR_TABLE[i].offset >> 2) as usize] = i as u8 + 1;
        i += 1;
    }
    t
};

/// Index of `offset` in [`CSFR_TABLE`].
#[inline]
pub fn csfr_index(offset: u16) -> Option<usize> {
    if offset & 3 != 0 {
        return None;
    }
    match CSFR_INDEX[(offset >> 2) as usize] {
        0 => None,
        i => Some(i as usize - 1),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn csfr_table_has_unique_offsets() {
        for (i, d) in CSFR_TABLE.iter().enumerate() {
            assert_eq!(csfr_index(d.offset), Some(i), "duplicate CSFR {}", d.name);
        }
    }

    #[test]
    fn csa_effective_address() {
        // segment 0xD, offset 0x0001 -> 0xD000_0040
        assert_eq!(csa_ea(0x000D_0001), 0xD000_0040);
    }
}
