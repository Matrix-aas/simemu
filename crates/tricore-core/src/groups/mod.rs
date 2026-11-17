//! Instruction groups. Each module implements the decode/execute contract of `crate::op` for a
//! disjoint set of mnemonics; together they cover every mnemonic of the TriCore 1.6 and 1.3.1
//! instruction-set manuals exactly once (checked below).
//!
//! | file | group | scope |
//! |---|---|---|
//! | `alu.rs` | A | integer ALU, compare, logic, shift, move, address arithmetic |
//! | `bitmul.rs` | B | bit logic (*.T), bit-field, count-leading, MUL/MADD/MSUB, IXMAX/IXMIN |
//! | `ldst.rs` | C | loads/stores, all addressing modes, LEA, LDMST, SWAP.W, context load/store, cache ops |
//! | `ctrl.rs` | D | branches, loops, call/return, context, traps, interrupt enable, MFCR/MTCR, sync |
//! | `fpu_div.rs` | E | FPU, PACK/UNPACK, integer division, TC1.3.1 MMU (TLB*) |

pub mod alu;
pub mod bitmul;
pub mod ctrl;
pub mod fpu_div;
pub mod ldst;

/// Every mnemonic of V2 "List of Instructions by Shortname" (TriCore V1.6 Vol. 2, V1.0 2012-05).
pub const TC16_MNEMONICS: &[&str] = &[
    "ABS", "ABS.B", "ABS.H", "ABSDIF", "ABSDIF.B", "ABSDIF.H", "ABSDIFS", "ABSDIFS.H", "ABSS",
    "ABSS.H", "ADD", "ADD.A", "ADD.B", "ADD.F", "ADD.H", "ADDC", "ADDI", "ADDIH", "ADDIH.A", "ADDS",
    "ADDS.H", "ADDS.HU", "ADDS.U", "ADDSC.A", "ADDSC.AT", "ADDX", "AND", "AND.AND.T", "AND.ANDN.T",
    "AND.EQ", "AND.GE", "AND.GE.U", "AND.LT", "AND.LT.U", "AND.NE", "AND.NOR.T", "AND.OR.T",
    "AND.T", "ANDN", "ANDN.T", "BISR", "BMERGE", "BSPLIT", "CACHEA.I", "CACHEA.W", "CACHEA.WI",
    "CACHEI.I", "CACHEI.W", "CACHEI.WI", "CADD", "CADDN", "CALL", "CALLA", "CALLI", "CLO", "CLO.H",
    "CLS", "CLS.H", "CLZ", "CLZ.H", "CMOV", "CMOVN", "CMP.F", "CSUB", "CSUBN", "DEBUG", "DEXTR",
    "DISABLE", "DIV", "DIV.F", "DIV.U", "DSYNC", "DVADJ", "DVINIT", "DVINIT.B", "DVINIT.BU",
    "DVINIT.H", "DVINIT.HU", "DVINIT.U", "DVSTEP", "DVSTEP.U", "ENABLE", "EQ", "EQ.A", "EQ.B",
    "EQ.H", "EQ.W", "EQANY.B", "EQANY.H", "EQZ.A", "EXTR", "EXTR.U", "FCALL", "FCALLA", "FCALLI",
    "FRET", "FTOI", "FTOIZ", "FTOQ31", "FTOQ31Z", "FTOU", "FTOUZ", "GE", "GE.A", "GE.U", "IMASK",
    "INS.T", "INSERT", "INSN.T", "ISYNC", "ITOF", "IXMAX", "IXMAX.U", "IXMIN", "IXMIN.U", "J", "JA",
    "JEQ", "JEQ.A", "JGE", "JGE.U", "JGEZ", "JGTZ", "JI", "JL", "JLA", "JLEZ", "JLI", "JLT",
    "JLT.U", "JLTZ", "JNE", "JNE.A", "JNED", "JNEI", "JNZ", "JNZ.A", "JNZ.T", "JZ", "JZ.A", "JZ.T",
    "LD.A", "LD.B", "LD.BU", "LD.D", "LD.DA", "LD.H", "LD.HU", "LD.Q", "LD.W", "LDLCX", "LDMST",
    "LDUCX", "LEA", "LOOP", "LOOPU", "LT", "LT.A", "LT.B", "LT.BU", "LT.H", "LT.HU", "LT.U", "LT.W",
    "LT.WU", "MADD", "MADD.F", "MADD.H", "MADD.Q", "MADD.U", "MADDM.H", "MADDMS.H", "MADDR.H",
    "MADDR.Q", "MADDRS.H", "MADDRS.Q", "MADDS", "MADDS.H", "MADDS.Q", "MADDS.U", "MADDSU.H",
    "MADDSUM.H", "MADDSUMS.H", "MADDSUR.H", "MADDSURS.H", "MADDSUS.H", "MAX", "MAX.B", "MAX.BU",
    "MAX.H", "MAX.HU", "MAX.U", "MFCR", "MIN", "MIN.B", "MIN.BU", "MIN.H", "MIN.HU", "MIN.U", "MOV",
    "MOV.A", "MOV.AA", "MOV.D", "MOV.U", "MOVH", "MOVH.A", "MSUB", "MSUB.F", "MSUB.H", "MSUB.Q",
    "MSUB.U", "MSUBAD.H", "MSUBADM.H", "MSUBADMS.H", "MSUBADR.H", "MSUBADRS.H", "MSUBADS.H",
    "MSUBM.H", "MSUBMS.H", "MSUBR.H", "MSUBR.Q", "MSUBRS.H", "MSUBRS.Q", "MSUBS", "MSUBS.H",
    "MSUBS.Q", "MSUBS.U", "MTCR", "MUL", "MUL.F", "MUL.H", "MUL.Q", "MUL.U", "MULM.H", "MULMS.H",
    "MULR.H", "MULR.Q", "MULS", "MULS.U", "NAND", "NAND.T", "NE", "NE.A", "NEZ.A", "NOP", "NOR",
    "NOR.T", "NOT", "OR", "OR.AND.T", "OR.ANDN.T", "OR.EQ", "OR.GE", "OR.GE.U", "OR.LT", "OR.LT.U",
    "OR.NE", "OR.NOR.T", "OR.OR.T", "OR.T", "ORN", "ORN.T", "PACK", "PARITY", "Q31TOF", "QSEED.F",
    "RESTORE", "RET", "RFE", "RFM", "RSLCX", "RSTV", "RSUB", "RSUBS", "RSUBS.U", "SAT.B", "SAT.BU",
    "SAT.H", "SAT.HU", "SEL", "SELN", "SH", "SH.AND.T", "SH.ANDN.T", "SH.EQ", "SH.GE", "SH.GE.U",
    "SH.H", "SH.LT", "SH.LT.U", "SH.NAND.T", "SH.NE", "SH.NOR.T", "SH.OR.T", "SH.ORN.T",
    "SH.XNOR.T", "SH.XOR.T", "SHA", "SHA.H", "SHAS", "ST.A", "ST.B", "ST.D", "ST.DA", "ST.H",
    "ST.Q", "ST.T", "ST.W", "STLCX", "STUCX", "SUB", "SUB.A", "SUB.B", "SUB.F", "SUB.H", "SUBC",
    "SUBS", "SUBS.H", "SUBS.HU", "SUBS.U", "SUBX", "SVLCX", "SWAP.W", "SYSCALL", "TRAPSV", "TRAPV",
    "UNPACK", "UPDFL", "UTOF", "XNOR", "XNOR.T", "XOR", "XOR.EQ", "XOR.GE", "XOR.GE.U", "XOR.LT",
    "XOR.LT.U", "XOR.NE", "XOR.T"
];

/// Mnemonics of V131-2 (TriCore 1 V1.3 & V1.3.1 Vol. 2, V1.3.8) that are absent from the TC1.6 manual.
pub const TC131_ONLY_MNEMONICS: &[&str] = &[
    "TLBDEMAP", "TLBFLUSH.A", "TLBFLUSH.B", "TLBMAP", "TLBPROBE.A", "TLBPROBE.I"
];

/// Mnemonics of the TC1.6 manual that are absent from V131-2 (not part of the `Arch::Tc131` ISA).
pub const TC16_ONLY_MNEMONICS: &[&str] = &[
    "CACHEI.I", "DIV", "DIV.U", "FCALL", "FCALLA", "FCALLI", "FRET", "MULMS.H", "RESTORE"
];

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    #[test]
    fn every_manual_mnemonic_belongs_to_exactly_one_group() {
        let mut owner: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
        for (g, list) in [
            ("alu", alu::MNEMONICS),
            ("bitmul", bitmul::MNEMONICS),
            ("ldst", ldst::MNEMONICS),
            ("ctrl", ctrl::MNEMONICS),
            ("fpu_div", fpu_div::MNEMONICS),
        ] {
            for m in list {
                owner.entry(m).or_default().push(g);
            }
        }
        let manual: Vec<&str> = TC16_MNEMONICS.iter().chain(TC131_ONLY_MNEMONICS).copied().collect();
        for m in &manual {
            assert_eq!(owner.get(m).map(|v| v.len()), Some(1), "{m} owned by {:?}", owner.get(m));
        }
        for m in owner.keys() {
            assert!(manual.contains(m), "{m} is not a manual mnemonic");
        }
    }
}
