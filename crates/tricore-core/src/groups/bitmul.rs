//! Group B — single-bit logic (BIT format: AND.T ... XOR.T, accumulating AND./OR. *.T, SH.*.T, INS.T,
//! INSN.T), bit-field (EXTR, EXTR.U, INSERT, IMASK, DEXTR, BMERGE, BSPLIT, PARITY), count leading
//! (CLO/CLS/CLZ incl. .H), multiply and multiply-accumulate (MUL*, MADD*, MSUB* integer, .U, .Q, .H,
//! rounding/saturating/M variants) and DSP index search (IXMAX/IXMIN).
//!
//! Sources: V2 = TriCore V1.6 Vol. 2 (`vault/docs/infineon/tcarch_vol2_a4.pdf`), V131-2 = TriCore 1 V1.3 &
//! V1.3.1 Vol. 2 (`vault/docs/infineon/Tc_v131_instructionset_v138.pdf`). Each instruction class below
//! cites both. TC1.3.1 vs TC1.6: the encoding boxes of all 266 group-B encodings of V131-2 (glyph font
//! with a code offset of 29: '1' is 0x14, 'a' is 'D') decode to exactly the op1/op2/fixed fields of V2,
//! and the RTL and status-flag text of every page is identical apart from line wrapping. The
//! only delta is MULMS.H, which exists in V2 only (`TC16_ONLY_MNEMONICS`).
//!
//! Decoding policy:
//! * fields the encoding box gives a value for (op2, `n = 0H` of BMERGE/BSPLIT/PARITY/IXMAX*) must
//!   match; reserved `-` fields are ignored ("Read value is undefined", V2 Table 1-7 p. 1-5);
//! * `E[x]` operands: V2 Table 1-7 defines only even codes ("`E[0]` = 0000B, `E[2]` = 0010B, and so on");
//!   odd codes are undefined and this core uses the even register of the pair (bit 0 ignored);
//! * MAC/MUL `n`: "n should only take the values 0 or 1, any other value returns an undefined result"
//!   (e.g. V2 p. 3-183): n = 2/3 execute the RTL literally (`<< n`; the -1 * -1 case needs n == 1);
//! * results the manual calls undefined (EXTR width 0 or pos + width > 32, INSERT/IMASK
//!   pos + width > 32, DEXTR `D[d]` > 31) evaluate the RTL literally with 64-bit intermediates,
//!   truncated to 32 bits; EXTR/EXTR.U with width 0 give 0.
//!
//! Status flags: arithmetic RTL is evaluated with infinite precision (i128); `overflow` and
//! `advanced_overflow` are taken from the unsaturated result as the pages state.
//! None of these instructions branch or trap, so no operation ends a block.

use crate::flags::{leading_signs, set_v_av};
use crate::fmt::{self, word};
use crate::{Arch, Bus, Cpu, Flow};

/// Mnemonics owned by this group (manual spelling).
pub const MNEMONICS: &[&str] = &[
    "AND.AND.T", "AND.ANDN.T", "AND.NOR.T", "AND.OR.T", "AND.T", "ANDN.T", "BMERGE", "BSPLIT",
    "CLO", "CLO.H", "CLS", "CLS.H", "CLZ", "CLZ.H", "DEXTR", "EXTR", "EXTR.U", "IMASK", "INS.T",
    "INSERT", "INSN.T", "IXMAX", "IXMAX.U", "IXMIN", "IXMIN.U", "MADD", "MADD.H", "MADD.Q",
    "MADD.U", "MADDM.H", "MADDMS.H", "MADDR.H", "MADDR.Q", "MADDRS.H", "MADDRS.Q", "MADDS",
    "MADDS.H", "MADDS.Q", "MADDS.U", "MADDSU.H", "MADDSUM.H", "MADDSUMS.H", "MADDSUR.H",
    "MADDSURS.H", "MADDSUS.H", "MSUB", "MSUB.H", "MSUB.Q", "MSUB.U", "MSUBAD.H", "MSUBADM.H",
    "MSUBADMS.H", "MSUBADR.H", "MSUBADRS.H", "MSUBADS.H", "MSUBM.H", "MSUBMS.H", "MSUBR.H",
    "MSUBR.Q", "MSUBRS.H", "MSUBRS.Q", "MSUBS", "MSUBS.H", "MSUBS.Q", "MSUBS.U", "MUL", "MUL.H",
    "MUL.Q", "MUL.U", "MULM.H", "MULMS.H", "MULR.H", "MULR.Q", "MULS", "MULS.U", "NAND.T", "NOR.T",
    "OR.AND.T", "OR.ANDN.T", "OR.NOR.T", "OR.OR.T", "OR.T", "ORN.T", "PARITY", "SH.AND.T",
    "SH.ANDN.T", "SH.NAND.T", "SH.NOR.T", "SH.OR.T", "SH.ORN.T", "SH.XNOR.T", "SH.XOR.T", "XNOR.T",
    "XOR.T"
];

/// Boolean function of `x = D[a][pos1]` and `y = D[b][pos2]` (BIT format).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitFn {
    And,
    Or,
    Nor,
    AndN,
    Nand,
    OrN,
    Xnor,
    Xor,
}

/// How the boolean result reaches `D[c]`: `zero_ext` (AND.T ...), `D[c][0] AND r` (AND.*.T),
/// `D[c][0] OR r` (OR.*.T) or `{D[c][30:0], r}` (SH.*.T).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitAcc {
    Set,
    And,
    Or,
    Sh,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CountFn {
    Clo,
    Cls,
    Clz,
    CloH,
    ClsH,
    ClzH,
}

/// Bit-field value: `D[b]` or zero-extended const4.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Val {
    Reg(u8),
    Imm(u8),
}

/// Bit-field position and width: immediates (xxPW), `E[d][4:0]` / `E[d][36:32]` (RRRR, RCRR) or
/// `D[d][4:0]` and an immediate width (xxRW).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pw {
    Imm { pos: u8, width: u8 },
    E(u8),
    D { d: u8, width: u8 },
}

/// Second multiplier operand: `D[b]` or const9, sign- or zero-extended at decode time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Src {
    Reg(u8),
    Imm(i16),
}

/// Q-format product shape (V2 §2.4.7 p. 2-10; MADD.Q p. 3-187, MUL.Q p. 3-274).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QKind {
    /// 32 * 32, upper word -> 32.
    W32Up,
    /// 32 * 32 -> 64.
    W32,
    /// 32 * 16L, `>> 16` -> 32.
    L32Up,
    /// 32 * 16L -> 64.
    L32,
    U32Up,
    U32,
    /// 16L * 16L -> 32.
    LL,
    /// 16L * 16L, `<< 16` -> 64.
    LLE,
    UU,
    UUE,
    /// 16L * 16L rounded -> upper half of 32 (MULR.Q, MADDR.Q, ...).
    RLL,
    RUU,
}

/// Packed half-word multiply shape (V2 §2.4.8 p. 2-10).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HKind {
    /// Two 32-bit words in `E[c]` (MUL.H, MADD.H, MADDS.H, MSUB.H, MADDSU.H, MSUBAD.H ...).
    Packed,
    /// Sum of both products `<< 16` into a 64-bit accumulator (MULM.H, MADDM.H, MADDSUM.H ...).
    Multi,
    /// Two rounded 16-bit results in `D[c]`, accumulator halves from `D[d]` (MULR.H, MADDR.H ...).
    Round,
    /// As `Round`, accumulator words from `E[d]` (MADDR.H/MSUBR.H `D[c], E[d]` ... UL, op1 43H/63H).
    RoundE,
}

/// MAC option bits of [`BitMulOp::Int`], [`BitMulOp::Q`] and [`BitMulOp::H`].
pub mod mf {
    /// Has an accumulator operand (MADD*, MSUB*).
    pub const ACC: u8 = 1;
    /// Product subtracted (Q, Int); for `.H`: the word-1 product is subtracted.
    pub const SUB: u8 = 2;
    /// `.H`: the word-0 product is subtracted.
    pub const SUB0: u8 = 4;
    /// Saturating (`ssov`/`suov`).
    pub const SAT: u8 = 8;
    /// Int: unsigned operands.
    pub const UNS: u8 = 16;
    /// Int: 64-bit destination and accumulator (E registers).
    pub const WIDE: u8 = 32;
    /// PSW unchanged (MULMS.H).
    pub const KEEP: u8 = 64;
}

/// Predecoded group-B operation. Register numbers are `u8`; E-register operands hold the even
/// register number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitMulOp {
    Bit { f: BitFn, acc: BitAcc, c: u8, a: u8, p1: u8, b: u8, p2: u8 },
    InsT { inv: bool, c: u8, a: u8, p1: u8, b: u8, p2: u8 },
    Count { f: CountFn, c: u8, a: u8 },
    Bmerge { c: u8, a: u8, b: u8 },
    Bsplit { c: u8, a: u8 },
    Parity { c: u8, a: u8 },
    /// Position from `pw` (width unused).
    Dextr { c: u8, a: u8, b: u8, pw: Pw },
    Extr { signed: bool, c: u8, a: u8, pw: Pw },
    Insert { c: u8, a: u8, v: Val, pw: Pw },
    Imask { c: u8, v: Val, pw: Pw },
    Ix { max: bool, uns: bool, c: u8, d: u8, b: u8 },
    /// `MUL D[c], D[a], D[b]/const9` (32-bit signed result; fast path of `Int`).
    Mul { c: u8, a: u8, s: Src },
    /// MUL (E), MULS, MUL.U, MULS.U, MADD*, MSUB* integer forms (flags: [`mf`]).
    Int { f: u8, c: u8, d: u8, a: u8, s: Src },
    Q { k: QKind, f: u8, n: u8, c: u8, d: u8, a: u8, b: u8 },
    /// `mode = op2[1:0]`: 0 UL, 1 LU, 2 LL, 3 UU.
    H { k: HKind, mode: u8, f: u8, n: u8, c: u8, d: u8, a: u8, b: u8 },
}

use BitMulOp as O;

/// Decode a group-B instruction (`hi` ignored for 16-bit encodings).
pub fn decode(lo: u16, hi: u16, _pc: u32, arch: Arch) -> Option<BitMulOp> {
    if lo & 1 == 0 {
        // MUL D[a], D[b] (SRR, op1 E2H): V2 p. 3-269/3-270, V131-2 p. 3-324.
        if lo & 0xFF == 0xE2 {
            let s = fmt::Srr::from(lo);
            return Some(O::Mul { c: s.a as u8, a: s.a as u8, s: Src::Reg(s.b as u8) });
        }
        return None;
    }
    let w = word(lo, hi);
    let r = |x: u32| x as u8;
    let e = |x: u32| (x & !1) as u8;
    Some(match w as u8 {
        // ---- BIT format, V2 Table 1-6: c pos2 op2 pos1 b a op1 ----
        // AND.T/OR.T/NOR.T/ANDN.T (87H): V2 p. 3-40, 3-304, 3-292, 3-42; V131-2 p. 3-66, 3-362, 3-350, 3-68.
        // NAND.T/ORN.T/XNOR.T/XOR.T (07H): V2 p. 3-286, 3-306, 3-384, 3-392; V131-2 p. 3-344, 3-364, 3-456, 3-465.
        // AND.AND.T/AND.OR.T/AND.NOR.T/AND.ANDN.T (47H): V2 p. 3-32; V131-2 p. 3-58.
        // OR.AND.T/OR.OR.T/OR.NOR.T/OR.ANDN.T (C7H): V2 p. 3-296; V131-2 p. 3-354.
        // SH.AND.T/SH.OR.T/SH.NOR.T/SH.ANDN.T (27H), SH.NAND.T/SH.ORN.T/SH.XNOR.T/SH.XOR.T (A7H):
        // V2 p. 3-335/3-336; V131-2 p. 3-399.
        op1 @ (0x87 | 0x07 | 0x47 | 0xC7 | 0x27 | 0xA7) => {
            let i = fmt::Bit::from(w);
            use BitFn::*;
            let f = match (op1, i.op2) {
                (0x87 | 0x47 | 0xC7 | 0x27, 0) => And,
                (0x87 | 0x47 | 0xC7 | 0x27, 1) => Or,
                (0x87 | 0x47 | 0xC7 | 0x27, 2) => Nor,
                (0x87 | 0x47 | 0xC7 | 0x27, _) => AndN,
                (_, 0) => Nand,
                (_, 1) => OrN,
                (_, 2) => Xnor,
                _ => Xor,
            };
            let acc = match op1 {
                0x87 | 0x07 => BitAcc::Set,
                0x47 => BitAcc::And,
                0xC7 => BitAcc::Or,
                _ => BitAcc::Sh,
            };
            O::Bit { f, acc, c: r(i.c), a: r(i.a), p1: r(i.pos1), b: r(i.b), p2: r(i.pos2) }
        }
        // INS.T (op2 0) / INSN.T (op2 1): V2 p. 3-111, V131-2 p. 3-137.
        0x67 => {
            let i = fmt::Bit::from(w);
            if i.op2 > 1 {
                return None;
            }
            O::InsT { inv: i.op2 == 1, c: r(i.c), a: r(i.a), p1: r(i.pos1), b: r(i.b), p2: r(i.pos2) }
        }
        // ---- RR: count leading (0FH) ----
        // CLO 1CH, CLO.H 7DH, CLS 1DH, CLS.H 7EH, CLZ 1BH, CLZ.H 7CH: V2 p. 3-68..3-73,
        // V131-2 p. 3-97..3-102. Fields b and n are reserved.
        0x0F => {
            let i = fmt::Rr::from(w);
            let f = match i.op2 {
                0x1C => CountFn::Clo,
                0x7D => CountFn::CloH,
                0x1D => CountFn::Cls,
                0x7E => CountFn::ClsH,
                0x1B => CountFn::Clz,
                0x7C => CountFn::ClzH,
                _ => return None,
            };
            O::Count { f, c: r(i.c), a: r(i.a) }
        }
        // ---- RR 4BH: BMERGE 01H (V2 p. 3-45, V131-2 p. 3-71), BSPLIT 09H (V2 p. 3-46, V131-2
        // p. 3-72), PARITY 02H (V2 p. 3-309, V131-2 p. 3-368); n = 0H in all three boxes ----
        0x4B => {
            let i = fmt::Rr::from(w);
            match (i.op2, i.n) {
                (0x01, 0) => O::Bmerge { c: r(i.c), a: r(i.a), b: r(i.b) },
                (0x09, 0) => O::Bsplit { c: e(i.c), a: r(i.a) },
                (0x02, 0) => O::Parity { c: r(i.c), a: r(i.a) },
                _ => return None,
            }
        }
        // ---- bit fields: DEXTR V2 p. 3-79 / V131-2 p. 3-108; EXTR, EXTR.U V2 p. 3-100/3-101 /
        // V131-2 p. 3-130; IMASK V2 p. 3-109 / V131-2 p. 3-135; INSERT V2 p. 3-112/3-113 /
        // V131-2 p. 3-138 ----
        // RRPW 77H op2 0: DEXTR D[c], D[a], D[b], pos (width field reserved).
        0x77 => {
            let i = fmt::Rrpw::from(w);
            if i.op2 != 0 {
                return None;
            }
            O::Dextr { c: r(i.c), a: r(i.a), b: r(i.b), pw: Pw::Imm { pos: r(i.pos), width: 0 } }
        }
        // RRPW 37H: op2 0 INSERT D[b], 1 IMASK E[c], D[b], 2 EXTR, 3 EXTR.U.
        0x37 => {
            let i = fmt::Rrpw::from(w);
            let pw = Pw::Imm { pos: r(i.pos), width: r(i.width) };
            match i.op2 {
                0 => O::Insert { c: r(i.c), a: r(i.a), v: Val::Reg(r(i.b)), pw },
                1 => O::Imask { c: e(i.c), v: Val::Reg(r(i.b)), pw },
                op2 => O::Extr { signed: op2 == 2, c: r(i.c), a: r(i.a), pw },
            }
        }
        // RRRR 17H: op2 0 INSERT D[b], E[d]; 2 EXTR E[d]; 3 EXTR.U E[d]; 4 DEXTR D[d].
        0x17 => {
            let i = fmt::Rrrr::from(w);
            match i.op2 {
                0 => O::Insert { c: r(i.c), a: r(i.a), v: Val::Reg(r(i.b)), pw: Pw::E(e(i.d)) },
                2 | 3 => O::Extr { signed: i.op2 == 2, c: r(i.c), a: r(i.a), pw: Pw::E(e(i.d)) },
                4 => O::Dextr { c: r(i.c), a: r(i.a), b: r(i.b), pw: Pw::D { d: r(i.d), width: 0 } },
                _ => return None,
            }
        }
        // RRRW 57H: op2 0 INSERT, 1 IMASK, 2 EXTR, 3 EXTR.U with D[d], width.
        0x57 => {
            let i = fmt::Rrrw::from(w);
            let pw = Pw::D { d: r(i.d), width: r(i.width) };
            match i.op2 {
                0 => O::Insert { c: r(i.c), a: r(i.a), v: Val::Reg(r(i.b)), pw },
                1 => O::Imask { c: e(i.c), v: Val::Reg(r(i.b)), pw },
                2 | 3 => O::Extr { signed: i.op2 == 2, c: r(i.c), a: r(i.a), pw },
                _ => return None,
            }
        }
        // RCRR 97H op2 0: INSERT D[c], D[a], const4, E[d].
        0x97 => {
            let i = fmt::Rcrr::from(w);
            if i.op2 != 0 {
                return None;
            }
            O::Insert { c: r(i.c), a: r(i.a), v: Val::Imm(r(i.const4)), pw: Pw::E(e(i.d)) }
        }
        // RCPW B7H: op2 0 INSERT const4, pos, width; 1 IMASK E[c], const4, pos, width (a reserved).
        0xB7 => {
            let i = fmt::Rcpw::from(w);
            let pw = Pw::Imm { pos: r(i.pos), width: r(i.width) };
            match i.op2 {
                0 => O::Insert { c: r(i.c), a: r(i.a), v: Val::Imm(r(i.const4)), pw },
                1 => O::Imask { c: e(i.c), v: Val::Imm(r(i.const4)), pw },
                _ => return None,
            }
        }
        // RCRW D7H: op2 0 INSERT const4, D[d], width; 1 IMASK E[c], const4, D[d], width.
        // (Unicorn swaps the c and d fields of both: crate::testutil::oracle::KNOWN_WRONG.)
        0xD7 => {
            let i = fmt::Rcrw::from(w);
            let pw = Pw::D { d: r(i.d), width: r(i.width) };
            match i.op2 {
                0 => O::Insert { c: r(i.c), a: r(i.a), v: Val::Imm(r(i.const4)), pw },
                1 => O::Imask { c: e(i.c), v: Val::Imm(r(i.const4)), pw },
                _ => return None,
            }
        }
        // ---- RRR 6BH: IXMAX 0AH, IXMAX.U 0BH (V2 p. 3-115, V131-2 p. 3-142), IXMIN 08H, IXMIN.U 09H
        // (V2 p. 3-117, V131-2 p. 3-144); n = 0H, a reserved ----
        0x6B => {
            let i = fmt::Rrr::from(w);
            if i.n != 0 || !(0x08..=0x0B).contains(&i.op2) {
                return None;
            }
            O::Ix { max: i.op2 >= 0x0A, uns: i.op2 & 1 != 0, c: e(i.c), d: e(i.d), b: r(i.b) }
        }
        // ---- integer multiply: MUL/MULS V2 p. 3-269/3-270, V131-2 p. 3-324; MUL.U/MULS.U V2 p. 3-277,
        // V131-2 p. 3-333. RC 53H (op2 [27:21]) const9, RR2 73H (op2 [27:16]) D[b] ----
        0x53 | 0x73 => {
            let (c, a, op2, s) = if w as u8 == 0x53 {
                let i = fmt::Rc::from(w);
                let s = |uns: bool| Src::Imm(if uns { i.const9 as i16 } else { fmt::sext(i.const9, 9) as i32 as i16 });
                (i.c, i.a, i.op2, s(matches!(i.op2, 2 | 4)))
            } else {
                let i = fmt::Rr2::from(w);
                let op2 = match i.op2 {
                    0x0A => 1,
                    0x6A => 3,
                    0x8A => 5,
                    0x68 => 2,
                    0x88 => 4,
                    _ => return None,
                };
                (i.c, i.a, op2, Src::Reg(r(i.b)))
            };
            use mf::*;
            let f = match op2 {
                1 => return Some(O::Mul { c: r(c), a: r(a), s }),
                3 => WIDE,
                5 => SAT,
                2 => UNS | WIDE,
                4 => UNS | SAT,
                _ => return None,
            };
            let c = if f & WIDE != 0 { e(c) } else { r(c) };
            O::Int { f, c, d: 0, a: r(a), s }
        }
        // ---- MADD/MADDS (13H RCR, 03H RRR2) V2 p. 3-180/3-181, V131-2 p. 3-221; MADD.U/MADDS.U
        // V2 p. 3-193/3-194, V131-2 p. 3-235; MSUB/MSUBS (33H RCR, 23H RRR2) V2 p. 3-233/3-234,
        // V131-2 p. 3-282; MSUB.U/MSUBS.U V2 p. 3-246/3-247, V131-2 p. 3-296 ----
        op1 @ (0x13 | 0x33 | 0x03 | 0x23) => {
            use mf::*;
            let (c, d, a, op2, s) = if op1 & 0x10 != 0 {
                let i = fmt::Rcr::from(w);
                let uns = matches!(i.op2, 2 | 4 | 6);
                let k = if uns { i.const9 as i16 } else { fmt::sext(i.const9, 9) as i32 as i16 };
                (i.c, i.d, i.a, i.op2, Src::Imm(k))
            } else {
                let i = fmt::Rrr2::from(w);
                let op2 = match i.op2 {
                    0x0A => 1,
                    0x6A => 3,
                    0x8A => 5,
                    0xEA => 7,
                    0x68 => 2,
                    0x88 => 4,
                    0xE8 => 6,
                    _ => return None,
                };
                (i.c, i.d, i.a, op2, Src::Reg(r(i.b)))
            };
            let f = match op2 {
                1 => 0,
                3 => WIDE,
                5 => SAT,
                7 => SAT | WIDE,
                2 => UNS | WIDE,
                4 => UNS | SAT,
                6 => UNS | SAT | WIDE,
                _ => return None,
            } | ACC
                | if op1 & 0x20 != 0 { SUB } else { 0 };
            let (c, d) = if f & WIDE != 0 { (e(c), e(d)) } else { (r(c), r(d)) };
            O::Int { f, c, d, a: r(a), s }
        }
        // ---- Q format, RRR1 43H (MADD.Q/MADDS.Q V2 p. 3-187..3-191, V131-2 p. 3-228; MADDR.Q/
        // MADDRS.Q V2 p. 3-202, V131-2 p. 3-247; MADDR.H/MADDRS.H D[c], E[d] UL V2 p. 3-199/3-200,
        // V131-2 p. 3-242) and 63H (MSUB.Q/MSUBS.Q V2 p. 3-240..3-244, V131-2 p. 3-289; MSUBR.Q/
        // MSUBRS.Q V2 p. 3-266, V131-2 p. 3-321; MSUBR.H/MSUBRS.H D[c], E[d] UL V2 p. 3-263/3-264,
        // V131-2 p. 3-316) ----
        op1 @ (0x43 | 0x63) => {
            use mf::*;
            let i = fmt::Rrr1::from(w);
            let sub = if op1 == 0x63 { SUB } else { 0 };
            let sat = if i.op2 & 0x20 != 0 { SAT } else { 0 };
            if i.op2 & 0x1F == 0x1E {
                let f = ACC | sat | if sub != 0 { SUB | SUB0 } else { 0 };
                return Some(O::H { k: HKind::RoundE, mode: 0, f, n: r(i.n), c: r(i.c), d: e(i.d), a: r(i.a), b: r(i.b) });
            }
            let k = match i.op2 & 0x1F {
                0x02 => QKind::W32Up,
                0x1B => QKind::W32,
                0x01 => QKind::L32Up,
                0x19 => QKind::L32,
                0x00 => QKind::U32Up,
                0x18 => QKind::U32,
                0x05 => QKind::LL,
                0x1D => QKind::LLE,
                0x04 => QKind::UU,
                0x1C => QKind::UUE,
                0x07 => QKind::RLL,
                0x06 => QKind::RUU,
                _ => return None,
            };
            let d = if q_wide(k) { e(i.d) } else { r(i.d) };
            let c = if q_wide(k) { e(i.c) } else { r(i.c) };
            O::Q { k, f: ACC | sub | sat, n: r(i.n), c, d, a: r(i.a), b: r(i.b) }
        }
        // ---- MUL.Q (V2 p. 3-274/3-275, V131-2 p. 3-330) and MULR.Q (V2 p. 3-284, V131-2 p. 3-341):
        // RR1 93H, op2 [27:18] ----
        0x93 => {
            let i = fmt::Rr1::from(w);
            let k = match i.op2 {
                0x02 => QKind::W32Up,
                0x1B => QKind::W32,
                0x01 => QKind::L32Up,
                0x19 => QKind::L32,
                0x00 => QKind::U32Up,
                0x18 => QKind::U32,
                0x05 => QKind::LL,
                0x04 => QKind::UU,
                0x07 => QKind::RLL,
                0x06 => QKind::RUU,
                _ => return None,
            };
            let c = if q_wide(k) { e(i.c) } else { r(i.c) };
            O::Q { k, f: 0, n: r(i.n), c, d: 0, a: r(i.a), b: r(i.b) }
        }
        // ---- packed half-word MAC, RRR1 op2 [23:18] = kind[5:2] | mode[1:0]:
        // 83H MADD.H/MADDS.H V2 p. 3-183, MADDM.H/MADDMS.H p. 3-195, MADDR.H/MADDRS.H p. 3-198
        //     (V131-2 p. 3-224, 3-238, 3-242);
        // A3H MSUB.H/MSUBS.H V2 p. 3-236, MSUBM.H/MSUBMS.H p. 3-259, MSUBR.H/MSUBRS.H p. 3-262
        //     (V131-2 p. 3-285, 3-312, 3-316);
        // C3H MADDSU.H/MADDSUS.H V2 p. 3-204, MADDSUM.H/MADDSUMS.H p. 3-208, MADDSUR.H/MADDSURS.H
        //     p. 3-211 (V131-2 p. 3-249, 3-253, 3-257);
        // E3H MSUBAD.H/MSUBADS.H V2 p. 3-248, MSUBADM.H/MSUBADMS.H p. 3-252, MSUBADR.H/MSUBADRS.H
        //     p. 3-255 (V131-2 p. 3-299, 3-303, 3-307) ----
        op1 @ (0x83 | 0xA3 | 0xC3 | 0xE3) => {
            use mf::*;
            let i = fmt::Rrr1::from(w);
            let (k, sat) = h_kind(i.op2)?;
            let signs = match op1 {
                0x83 => 0,
                0xA3 => SUB | SUB0,
                0xC3 => SUB0,
                _ => SUB,
            };
            let (c, d) = match k {
                HKind::Round => (r(i.c), r(i.d)),
                _ => (e(i.c), e(i.d)),
            };
            O::H { k, mode: (i.op2 & 3) as u8, f: ACC | signs | sat, n: r(i.n), c, d, a: r(i.a), b: r(i.b) }
        }
        // ---- packed half-word multiply, RR1 B3H op2 [27:18]: MUL.H 18H..1BH (V2 p. 3-272, V131-2
        // p. 3-327), MULM.H 1CH..1FH (V2 p. 3-279, V131-2 p. 3-335), MULR.H 0CH..0FH (V2 p. 3-282,
        // V131-2 p. 3-338), MULMS.H 3CH..3FH (V2 p. 3-281, TC1.6 only) ----
        0xB3 => {
            let i = fmt::Rr1::from(w);
            if i.op2 > 0x3F {
                return None;
            }
            let (k, sat) = h_kind(i.op2)?;
            let f = match (k, sat) {
                (_, 0) => 0,
                // MULMS.H: V2 p. 3-281 lists only the encodings and "C/V/SV/AV/SAV status flag is
                // not set" ("deprecated in TC2"; absent from V131-2 and the TC1.8 manual). Saturating
                // the MULM.H result to 64 bits is the identity (|result| < 2^50), so the value equals
                // MULM.H and the PSW is left unchanged.
                (HKind::Multi, _) if arch == Arch::Tc16 => mf::KEEP,
                _ => return None,
            };
            let c = if k == HKind::Round { r(i.c) } else { e(i.c) };
            O::H { k, mode: (i.op2 & 3) as u8, f, n: r(i.n), c, d: 0, a: r(i.a), b: r(i.b) }
        }
        _ => return None,
    })
}

/// `.H` kind from a 6-bit op2 (V2 MADD.H family pages): op2[4:2] = 110B packed (18H..1BH),
/// 111B multi-precision (1CH..1FH), 011B rounded (0CH..0FH); op2[5] (+20H) is the saturating variant.
fn h_kind(op2: u32) -> Option<(HKind, u8)> {
    let sat = if op2 & 0x20 != 0 { mf::SAT } else { 0 };
    let k = match (op2 >> 2) & 7 {
        6 => HKind::Packed,
        7 => HKind::Multi,
        3 => HKind::Round,
        _ => return None,
    };
    Some((k, sat))
}

#[inline(always)]
const fn q_wide(k: QKind) -> bool {
    matches!(k, QKind::W32 | QKind::L32 | QKind::U32 | QKind::LLE | QKind::UUE)
}

pub fn ends_block(_op: &BitMulOp) -> bool {
    false
}

// ---- helpers ------------------------------------------------------------------------------------

#[inline(always)]
fn get_e<B: Bus>(cpu: &Cpu<B>, d: u8) -> u64 {
    cpu.d[d as usize] as u64 | (cpu.d[d as usize + 1] as u64) << 32
}

#[inline(always)]
fn set_e<B: Bus>(cpu: &mut Cpu<B>, c: u8, v: u64) {
    cpu.d[c as usize] = v as u32;
    cpu.d[c as usize + 1] = (v >> 32) as u32;
}

/// Result outside the signed `w`-bit range (`overflow` of the 32-/64-bit status flag RTL).
#[inline(always)]
fn sov(r: i128, w: u32) -> bool {
    r < -(1i128 << (w - 1)) || r >= 1i128 << (w - 1)
}

/// `advanced_overflow = result[w-1] ^ result[w-2]`.
#[inline(always)]
fn aov(r: i128, w: u32) -> bool {
    ((r >> (w - 1)) ^ (r >> (w - 2))) & 1 != 0
}

#[inline(always)]
fn ssov(r: i128, w: u32) -> i128 {
    r.clamp(-(1i128 << (w - 1)), (1i128 << (w - 1)) - 1)
}

/// `(pos, width)` of a bit-field operand: `E[d][4:0]`, `E[d][36:32]` and `D[d][4:0]` (V2 p. 3-100).
#[inline(always)]
fn pos_width<B: Bus>(cpu: &Cpu<B>, pw: Pw) -> (u32, u32) {
    match pw {
        Pw::Imm { pos, width } => (pos as u32, width as u32),
        Pw::E(d) => (cpu.d[d as usize & 15] & 31, cpu.d[(d as usize + 1) & 15] & 31),
        Pw::D { d, width } => (cpu.d[d as usize & 15] & 31, width as u32),
    }
}

/// `(2^width - 1) << pos`, truncated to 32 bits (V2 INSERT/IMASK p. 3-109/3-112).
#[inline(always)]
fn field_mask(pos: u32, width: u32) -> u32 {
    ((((1u64 << width) - 1) << pos) & 0xFFFF_FFFF) as u32
}

/// Signed 16 * 16 product of the MAC pages with the -1 * -1 special case (V2 §2.4.2 p. 2-9):
/// `sc ? 7FFFFFFFH : (x * y) << n`, sc = both operands 8000H and n == 1. Returns (value, sc).
#[inline(always)]
fn mul16(x: i16, y: i16, n: u32) -> (i64, bool) {
    if n == 1 && x == i16::MIN && y == i16::MIN {
        (0x7FFF_FFFF, true)
    } else {
        (((x as i64) * (y as i64)) << n, false)
    }
}

/// Signed half-word operands of the two `.H` products: mode 0 UL, 1 LU, 2 LL, 3 UU (V2 MADD.H p. 3-183..3-184):
/// returns ((word-1 operands), (word-0 operands)).
#[inline(always)]
fn h_operands(x: u32, y: u32, mode: u8) -> ((i16, i16), (i16, i16)) {
    let (xl, xu, yl, yu) = (x as i16, (x >> 16) as i16, y as i16, (y >> 16) as i16);
    match mode {
        0 => ((xu, yu), (xl, yl)),
        1 => ((xu, yl), (xl, yu)),
        2 => ((xu, yl), (xl, yl)),
        _ => ((xl, yu), (xu, yu)),
    }
}

// ---- execute ------------------------------------------------------------------------------------

pub fn exec<B: Bus>(cpu: &mut Cpu<B>, op: &BitMulOp, pc: u32, len: u32) -> Flow {
    exec_inline(cpu, op, pc, len)
}

/// [`exec`] forced inline: the hot-instruction dispatch (`crate::op::exec_hot`) calls it with a
/// constant variant so that the match folds away.
#[inline(always)]
pub(crate) fn exec_inline<B: Bus>(cpu: &mut Cpu<B>, op: &BitMulOp, _pc: u32, _len: u32) -> Flow {
    match *op {
        O::Bit { f, acc, c, a, p1, b, p2 } => {
            let x = cpu.d[a as usize & 15] >> p1 & 1;
            let y = cpu.d[b as usize & 15] >> p2 & 1;
            let r = match f {
                BitFn::And => x & y,
                BitFn::Or => x | y,
                BitFn::Nor => (x | y) ^ 1,
                BitFn::AndN => x & (y ^ 1),
                BitFn::Nand => (x & y) ^ 1,
                BitFn::OrN => x | (y ^ 1),
                BitFn::Xnor => (x ^ y) ^ 1,
                BitFn::Xor => x ^ y,
            };
            let dc = cpu.d[c as usize & 15];
            cpu.d[c as usize & 15] = match acc {
                BitAcc::Set => r,
                BitAcc::And => dc & !(r ^ 1),
                BitAcc::Or => dc | r,
                BitAcc::Sh => dc << 1 | r,
            };
        }
        O::InsT { inv, c, a, p1, b, p2 } => {
            let bit = (cpu.d[b as usize & 15] >> p2 & 1) ^ inv as u32;
            cpu.d[c as usize & 15] = cpu.d[a as usize & 15] & !(1 << p1) | bit << p1;
        }
        O::Count { f, c, a } => {
            let x = cpu.d[a as usize & 15];
            let h = |g: fn(u16) -> u32| g((x >> 16) as u16) << 16 | g(x as u16);
            cpu.d[c as usize & 15] = match f {
                CountFn::Clo => x.leading_ones(),
                CountFn::Clz => x.leading_zeros(),
                CountFn::Cls => leading_signs(x) - 1,
                CountFn::CloH => h(|v| v.leading_ones()),
                CountFn::ClzH => h(|v| v.leading_zeros()),
                CountFn::ClsH => h(|v| if (v as i16) < 0 { v.leading_ones() - 1 } else { v.leading_zeros() - 1 }),
            };
        }
        O::Bmerge { c, a, b } => {
            // D[c][2i+1] = D[a][i], D[c][2i] = D[b][i] for i in 0..16 (V2 p. 3-45).
            let (x, y) = (cpu.d[a as usize & 15], cpu.d[b as usize & 15]);
            let mut v = 0;
            for i in 0..16 {
                v |= (x >> i & 1) << (2 * i + 1) | (y >> i & 1) << (2 * i);
            }
            cpu.d[c as usize & 15] = v;
        }
        O::Bsplit { c, a } => {
            // E[c][15:0] = even bits of D[a], E[c][47:32] = odd bits; other bits 0 (V2 p. 3-46).
            let x = cpu.d[a as usize & 15];
            let (mut ev, mut od) = (0, 0);
            for i in 0..16 {
                ev |= (x >> (2 * i) & 1) << i;
                od |= (x >> (2 * i + 1) & 1) << i;
            }
            cpu.d[c as usize & 15] = ev;
            cpu.d[c as usize + 1] = od;
        }
        O::Parity { c, a } => {
            // Each byte of D[c] = {7'b0, XOR of the bits of that byte of D[a]} (V2 p. 3-309).
            let x = cpu.d[a as usize & 15];
            let mut v = 0;
            for i in 0..4 {
                v |= (((x >> (8 * i)) as u8).count_ones() & 1) << (8 * i);
            }
            cpu.d[c as usize & 15] = v;
        }
        O::Dextr { c, a, b, pw } => {
            // D[c] = ({D[a], D[b]} << pos)[63:32], pos = pos or D[d][4:0] (V2 p. 3-79).
            let (pos, _) = pos_width(cpu, pw);
            let v = (cpu.d[a as usize & 15] as u64) << 32 | cpu.d[b as usize & 15] as u64;
            cpu.d[c as usize & 15] = ((v << pos) >> 32) as u32;
        }
        O::Extr { signed, c, a, pw } => {
            // D[c] = sign_ext/zero_ext((D[a] >> pos)[width-1:0]) (V2 p. 3-100).
            let (pos, width) = pos_width(cpu, pw);
            let v = (cpu.d[a as usize & 15] >> pos) & field_mask(0, width);
            cpu.d[c as usize & 15] = if signed && width > 0 { fmt::sext(v, width) } else { v };
        }
        O::Insert { c, a, v, pw } => {
            // mask = (2^width - 1) << pos; D[c] = (D[a] & ~mask) | ((value << pos) & mask) (V2 p. 3-112).
            let (pos, width) = pos_width(cpu, pw);
            let val = match v {
                Val::Reg(b) => cpu.d[b as usize & 15],
                Val::Imm(k) => k as u32,
            };
            let m = field_mask(pos, width);
            cpu.d[c as usize & 15] = cpu.d[a as usize & 15] & !m | (val << pos) & m;
        }
        O::Imask { c, v, pw } => {
            // E[c][63:32] = (2^width - 1) << pos; E[c][31:0] = value << pos (V2 p. 3-109).
            let (pos, width) = pos_width(cpu, pw);
            let val = match v {
                Val::Reg(b) => cpu.d[b as usize & 15],
                Val::Imm(k) => k as u32,
            };
            cpu.d[c as usize & 15] = val << pos;
            cpu.d[c as usize + 1] = field_mask(pos, width);
        }
        O::Ix { max, uns, c, d, b } => {
            // V2 p. 3-115..3-118: E[d] = {00H, value, index, working index}; compare D[b] halves.
            let (lo, hi) = (cpu.d[d as usize & 15], cpu.d[d as usize + 1]);
            let (wi, idx, cur) = (lo as u16, (lo >> 16) as u16, hi as u16);
            let y = cpu.d[b as usize & 15];
            let (v0, v1) = (y as u16, (y >> 16) as u16);
            let key = |v: u16| -> i32 {
                let k = if uns { v as i32 } else { v as i16 as i32 };
                if max {
                    k
                } else {
                    -k
                }
            };
            // IXMIN is IXMAX on negated keys: `<=`/`<` become `>=`/`>`.
            let (nv, ni) = if key(v0) >= key(v1) && key(v0) > key(cur) {
                (v0, wi)
            } else if key(v1) > key(v0) && key(v1) > key(cur) {
                (v1, wi.wrapping_add(1))
            } else {
                (cur, idx)
            };
            cpu.d[c as usize & 15] = (ni as u32) << 16 | wi.wrapping_add(2) as u32;
            cpu.d[c as usize + 1] = nv as u32;
        }
        O::Mul { c, a, s } => {
            // MUL D[c] (V2 p. 3-269): result = D[a] * D[b]/sign_ext(const9); D[c] = result[31:0];
            // 32-bit V/SV/AV/SAV update (p. 3-270).
            let y = match s {
                Src::Reg(b) => cpu.d[b as usize & 15] as i32,
                Src::Imm(k) => k as i32,
            };
            let p = cpu.d[a as usize & 15] as i32 as i64 * y as i64;
            cpu.d[c as usize & 15] = crate::flags::ov32(&mut cpu.psw, p);
        }
        O::Int { f, c, d, a, s } => {
            use mf::*;
            let uns = f & UNS != 0;
            let wide = f & WIDE != 0;
            let x = cpu.d[a as usize & 15];
            let y = match s {
                Src::Reg(b) => cpu.d[b as usize & 15],
                Src::Imm(k) => k as i32 as u32,
            };
            let p = if uns { (x as u64 * y as u64) as i128 } else { x as i32 as i128 * y as i32 as i128 };
            let acc = match (f & ACC != 0, wide, uns) {
                (false, _, _) => 0,
                (true, true, true) => get_e(cpu, d) as i128,
                (true, true, false) => get_e(cpu, d) as i64 as i128,
                (true, false, true) => cpu.d[d as usize & 15] as i128,
                (true, false, false) => cpu.d[d as usize & 15] as i32 as i128,
            };
            let r = if f & SUB != 0 { acc - p } else { acc + p };
            let w = if wide { 64 } else { 32 };
            let ov = if uns { r < 0 || r >> w != 0 } else { sov(r, w) };
            set_v_av(&mut cpu.psw, ov, aov(r, w));
            let v = match (f & SAT != 0, uns) {
                (false, _) => r,
                (true, false) => ssov(r, w),
                (true, true) => r.clamp(0, (1i128 << w) - 1),
            };
            if wide {
                set_e(cpu, c, v as u64);
            } else {
                cpu.d[c as usize & 15] = v as u32;
            }
        }
        O::Q { k, f, n, c, d, a, b } => exec_q(cpu, k, f, n as u32, c, d, a, b),
        O::H { k, mode, f, n, c, d, a, b } => exec_h(cpu, k, mode, f, n as u32, c, d, a, b),
    }
    Flow::Next
}

/// MUL.Q/MULR.Q (V2 p. 3-274, 3-284), MADD(S).Q/MADDR(S).Q (p. 3-187, 3-202), MSUB(S).Q/
/// MSUBR(S).Q (p. 3-240, 3-266).
#[allow(clippy::too_many_arguments)]
#[inline(never)]
fn exec_q<B: Bus>(cpu: &mut Cpu<B>, k: QKind, f: u8, n: u32, c: u8, d: u8, a: u8, b: u8) {
    use mf::*;
    let (x, y) = (cpu.d[a as usize], cpu.d[b as usize]);
    let xs = x as i32 as i128;
    let (xl, xu, yl, yu) = (x as i16, (x >> 16) as i16, y as i16, (y >> 16) as i16);
    let (m, sc) = match k {
        QKind::W32Up => (((xs * y as i32 as i128) << n) >> 32, false),
        QKind::W32 => ((xs * y as i32 as i128) << n, false),
        QKind::L32Up => (((xs * yl as i128) << n) >> 16, false),
        QKind::L32 => ((xs * yl as i128) << n, false),
        QKind::U32Up => (((xs * yu as i128) << n) >> 16, false),
        QKind::U32 => ((xs * yu as i128) << n, false),
        QKind::LL | QKind::RLL => {
            let (m, sc) = mul16(xl, yl, n);
            (m as i128, sc)
        }
        QKind::UU | QKind::RUU => {
            let (m, sc) = mul16(xu, yu, n);
            (m as i128, sc)
        }
        // 64-bit 16 * 16 forms add `mul_res << 16` (V2 p. 3-188).
        QKind::LLE => ((mul16(xl, yl, n).0 as i128) << 16, false),
        QKind::UUE => ((mul16(xu, yu, n).0 as i128) << 16, false),
    };
    let wide = q_wide(k);
    let round = matches!(k, QKind::RLL | QKind::RUU);
    let w = if wide { 64 } else { 32 };
    if f & ACC == 0 {
        if round {
            // MULR.Q: result = sc ? 7FFFFFFFH : (prod << n) + 8000H; D[c] = {result[31:16], 16'b0};
            // V cleared, AV/SAV from result.
            let r = if sc { 0x7FFF_FFFF } else { m + 0x8000 };
            set_v_av(&mut cpu.psw, false, aov(r, 32));
            cpu.d[c as usize] = r as u32 & 0xFFFF_0000;
            return;
        }
        // MUL.Q: full 32-/64-bit V/SV/AV/SAV update.
        set_v_av(&mut cpu.psw, sov(m, w), aov(m, w));
        if wide {
            set_e(cpu, c, m as u64);
        } else {
            cpu.d[c as usize] = m as u32;
        }
        return;
    }
    let acc = if wide { get_e(cpu, d) as i64 as i128 } else { cpu.d[d as usize] as i32 as i128 };
    let mut r = if f & SUB != 0 { acc - m } else { acc + m };
    if round {
        r += 0x8000;
    }
    set_v_av(&mut cpu.psw, sov(r, w), aov(r, w));
    let v = if f & SAT != 0 { ssov(r, w) } else { r };
    if wide {
        set_e(cpu, c, v as u64);
    } else {
        cpu.d[c as usize] = if round { v as u32 & 0xFFFF_0000 } else { v as u32 };
    }
}

/// Packed half-word multiply family: MUL.H/MULM.H/MULMS.H/MULR.H (V2 p. 3-272, 3-279, 3-281, 3-282)
/// and the MADD*.H/MSUB*.H/MADDSU*.H/MSUBAD*.H MAC pages (see `decode`).
#[allow(clippy::too_many_arguments)]
#[inline(never)]
fn exec_h<B: Bus>(cpu: &mut Cpu<B>, k: HKind, mode: u8, f: u8, n: u32, c: u8, d: u8, a: u8, b: u8) {
    use mf::*;
    let ((x1, y1), (x0, y0)) = h_operands(cpu.d[a as usize], cpu.d[b as usize], mode);
    let (m1, sc1) = mul16(x1, y1, n);
    let (m0, sc0) = mul16(x0, y0, n);
    let s1 = if f & SUB != 0 { -m1 } else { m1 };
    let s0 = if f & SUB0 != 0 { -m0 } else { m0 };
    let acc = f & ACC != 0;
    let sat = f & SAT != 0;
    match k {
        HKind::Packed => {
            // result_wordi = E[d] word i +/- mul_resi; E[c] = {(ssov)(w1), (ssov)(w0)}.
            let (w1, w0) = if acc {
                (cpu.d[d as usize + 1] as i32 as i64 + s1, cpu.d[d as usize] as i32 as i64 + s0)
            } else {
                (m1, m0)
            };
            let (w1, w0) = (w1 as i128, w0 as i128);
            let av = aov(w1, 32) || aov(w0, 32);
            // MUL.H (p. 3-273): V cleared, SV unchanged, AV and SAV from the words. The manual's SAV
            // row reads "then PSW.SV = 1 else PSW.SV = PSW.SV" (same in V131-2 p. 3-328 and the TC1.8
            // manual) while its SV row says "Not set by this instruction": the row label (SAV) is
            // taken as authoritative.
            set_v_av(&mut cpu.psw, acc && (sov(w1, 32) || sov(w0, 32)), av);
            let (v1, v0) = if sat { (ssov(w1, 32), ssov(w0, 32)) } else { (w1, w0) };
            cpu.d[c as usize] = v0 as u32;
            cpu.d[c as usize + 1] = v1 as u32;
        }
        HKind::Multi => {
            // result = E[d] + ((+/-result_word1 +/- result_word0) << 16); MULM.H: (w1 + w0) << 16.
            let r = if acc { get_e(cpu, d) as i64 as i128 + (((s1 + s0) as i128) << 16) } else { ((m1 + m0) as i128) << 16 };
            if f & KEEP == 0 {
                if acc {
                    set_v_av(&mut cpu.psw, sov(r, 64), aov(r, 64));
                } else {
                    // MULM.H (p. 3-280): V and AV cleared, SV and SAV unchanged.
                    set_v_av(&mut cpu.psw, false, false);
                }
            }
            let v = if sat { ssov(r, 64) } else { r };
            set_e(cpu, c, v as u64);
        }
        HKind::Round | HKind::RoundE => {
            // result_halfwordi = acc_i +/- mul_resi + 8000H; D[c] = {hw1[31:16], hw0[31:16]}.
            let (h1, h0) = if acc {
                let (a1, a0) = if k == HKind::RoundE {
                    (cpu.d[d as usize + 1] as i32, cpu.d[d as usize] as i32)
                } else {
                    let v = cpu.d[d as usize];
                    ((v & 0xFFFF_0000) as i32, (v << 16) as i32)
                };
                (a1 as i64 + s1 + 0x8000, a0 as i64 + s0 + 0x8000)
            } else {
                // MULR.H (p. 3-282): sc ? 7FFFFFFFH : (prod << n) + 8000H.
                (if sc1 { 0x7FFF_FFFF } else { m1 + 0x8000 }, if sc0 { 0x7FFF_FFFF } else { m0 + 0x8000 })
            };
            let (h1, h0) = (h1 as i128, h0 as i128);
            // MULR.H (p. 3-283): V cleared, AV/SAV from the half-words.
            set_v_av(&mut cpu.psw, acc && (sov(h1, 32) || sov(h0, 32)), aov(h1, 32) || aov(h0, 32));
            let (v1, v0) = if sat { (ssov(h1, 32), ssov(h0, 32)) } else { (h1, h0) };
            cpu.d[c as usize] = (v1 as u32 & 0xFFFF_0000) | (v0 as u32) >> 16;
        }
    }
}

/// Hand-computed vectors from the manual pages (edge cases, the Unicorn defects, policy choices).
/// Exhaustive per-encoding checks against a literal RTL transcription and against Unicorn are in
/// `tests/bitmul.rs`.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::arch::psw::{AV, SAV, SV, V};
    use crate::testutil::{oracle, run, run_arch, Setup, TRAP_BASE};
    use crate::NullBus;

    const FLAGS: u32 = V | SV | AV | SAV;

    fn one(w: u32, setup: impl FnOnce(&mut Cpu<NullBus>)) -> Cpu<NullBus> {
        run(&w.to_le_bytes(), 1, |c| {
            c.psw = 0;
            setup(c)
        })
    }

    #[test]
    fn extr_extr_u_rrpw() {
        // extr.u d2, d1, #4, #8 (V2 p. 3-100): (12345678H >> 4)[7:0] = 67H
        assert_eq!(one(0x2268_0137, |c| c.d[1] = 0x1234_5678).d[2], 0x67);
        // extr d2, d1, #4, #8: F8H sign-extended
        assert_eq!(one(0x2248_0137, |c| c.d[1] = 0xF80).d[2], 0xFFFF_FFF8);
        // the same bytes on the framework oracle (Unicorn is trustworthy for EXTR)
        if unicorn_tricore_sys::LINKED {
            let mut s = Setup::default();
            s.d[1] = 0xF80;
            assert_eq!(oracle::run(&0x2248_0137u32.to_le_bytes(), 1, &s).unwrap().d[2], 0xFFFF_FFF8);
        }
    }

    #[test]
    fn mul_flags() {
        // mul d3, d1, #-2 (RC, V2 p. 3-269): 40000000H * -2 = -2^31: no overflow, AV (bit31 ^ bit30)
        let c = one(0x303F_E153, |c| c.d[1] = 0x4000_0000);
        assert_eq!((c.d[3], c.psw & FLAGS), (0x8000_0000, AV | SAV));
        // 40000001H * -2 overflows: result[31:0], V and SV
        let c = one(0x303F_E153, |c| c.d[1] = 0x4000_0001);
        assert_eq!((c.d[3], c.psw & FLAGS), (0x7FFF_FFFE, V | SV | AV | SAV));
        // mul d2, d3 (SRR E2H): D[a] = D[a] * D[b]
        let c = run(&0x32E2u16.to_le_bytes(), 1, |c| (c.d[2], c.d[3]) = (7, -3i32 as u32));
        assert_eq!((c.d[2], c.pc), (-21i32 as u32, crate::testutil::CODE + 2));
        // mul e2, d1, d2 (RR2 6AH): 64-bit result never overflows (V cleared, SV kept); 2^62 sets AV
        let c = one(0x206A_2173, |c| (c.d[1], c.d[2], c.psw) = (0x8000_0000, 0x8000_0000, V | SV));
        assert_eq!((c.d[2], c.d[3], c.psw & FLAGS), (0, 0x4000_0000, SV | AV | SAV));
    }

    #[test]
    fn unsigned_saturation() {
        // madds.u d4, d5, d6, d7 (RRR2 88H, V2 p. 3-193): FFFFFFF0H + 2 * 10H saturates to FFFFFFFFH
        let c = one(0x4588_7603, |c| (c.d[5], c.d[6], c.d[7]) = (0xFFFF_FFF0, 2, 0x10));
        assert_eq!((c.d[4], c.psw & FLAGS), (0xFFFF_FFFF, V | SV));
        // msubs.u d4, d5, d6, d7 (V2 p. 3-246): 5 - 6 < 0 saturates to 0, overflow
        let c = one(0x4588_7623, |c| (c.d[5], c.d[6], c.d[7]) = (5, 2, 3));
        assert_eq!((c.d[4], c.psw & FLAGS), (0, V | SV));
    }

    #[test]
    fn madds_64_saturation() {
        // madds e2, e4, d6, #1 (RCR 07H, V2 p. 3-181): 7FFF..FH + 1 = 2^63 saturates; AV = r[63] ^ r[62]
        let c = one(0x24E0_1613, |c| (c.d[4], c.d[5], c.d[6]) = (0xFFFF_FFFF, 0x7FFF_FFFF, 1));
        assert_eq!((c.d[2], c.d[3], c.psw & FLAGS), (0xFFFF_FFFF, 0x7FFF_FFFF, V | SV | AV | SAV));
    }

    #[test]
    fn q_format_special_cases() {
        // mul.h e2, d0, d1ul, #1 (V2 p. 3-272): 8000H * 8000H = 7FFFFFFFH; V cleared, SV kept,
        // AV/SAV from the words (the SAV row of p. 3-273 says "PSW.SV": taken as SAV)
        let c = one(0x2061_10B3, |c| (c.d[0], c.d[1], c.psw) = (0x8000_0002, 0x8000_0003, V | SV));
        assert_eq!((c.d[3], c.d[2], c.psw & FLAGS), (0x7FFF_FFFF, 12, SV | AV | SAV));
        // mulr.q d3, d1u, d2u, #1 (V2 p. 3-284): sc -> 7FFFFFFFH without the rounding constant
        let c = one(0x3019_2193, |c| (c.d[1], c.d[2]) = (0x8000_1234, 0x8000_5678));
        assert_eq!((c.d[3], c.psw & FLAGS), (0x7FFF_0000, AV | SAV));
        let c = one(0x3019_2193, |c| (c.d[1], c.d[2]) = (0x4000_0000, 0x4000_0000));
        assert_eq!((c.d[3], c.psw & FLAGS), (0x2000_0000, 0));
        // maddr.q d3, d4, d1u, d2u, #1 (V2 p. 3-202): D[d] + 7FFFFFFFH + 8000H overflows
        let c = one(0x3419_2143, |c| (c.d[1], c.d[2], c.d[4]) = (0x8000_0000, 0x8000_0000, 0));
        assert_eq!((c.d[3], c.psw & FLAGS), (0x8000_0000, V | SV | AV | SAV));
        // maddrs.q (op2 26H): ssov(result, 32)[31:16]
        let c = one(0x3499_2143, |c| (c.d[1], c.d[2], c.d[4]) = (0x8000_0000, 0x8000_0000, 0));
        assert_eq!(c.d[3], 0x7FFF_0000);
        // mul.q d3, d1, d2, #1 (V2 p. 3-274): 32 * 32 has no -1 * -1 special case: 2^31 overflows
        let c = one(0x3009_2193, |c| (c.d[1], c.d[2]) = (0x8000_0000, 0x8000_0000));
        assert_eq!((c.d[3], c.psw & FLAGS), (0x8000_0000, V | SV | AV | SAV));
        // msub.q d3, d4, d1, d2, #0 (V2 p. 3-240): D[d] - ((1 * 1) >> 32) = 0. Unicorn computes
        // (D[d] * 2^32 - 1) >> 32 = FFFFFFFFH (tests/bitmul.rs UNICORN_WRONG).
        let c = one(0x3408_2163, |c| (c.d[1], c.d[2], c.d[4]) = (1, 1, 0));
        assert_eq!(c.d[3], 0);
    }

    #[test]
    fn mulm_and_deprecated_mulms() {
        // mulm.h e2, d0, d1ll, #1 (V2 p. 3-279): ((2*4 << 1) + (3*4 << 1)) << 16; V, AV cleared
        let c = one(0x2079_10B3, |c| (c.d[0], c.d[1], c.psw) = (0x0002_0003, 4, FLAGS));
        assert_eq!((c.d[2], c.d[3], c.psw & FLAGS), (0x0028_0000, 0, SV | SAV));
        // mulms.h (op2 3EH, V2 p. 3-281, TC1.6 only): same value, no status flag set
        let c = one(0x20F9_10B3, |c| (c.d[0], c.d[1], c.psw) = (0x0002_0003, 4, FLAGS));
        assert_eq!((c.d[2], c.d[3], c.psw & FLAGS), (0x0028_0000, 0, FLAGS));
        // not a TC1.3.1 instruction: illegal opcode trap (class 2)
        let c = run_arch(&0x20F9_10B3u32.to_le_bytes(), 1, Arch::Tc131, |_| {});
        assert_eq!(c.pc, TRAP_BASE | 2 << 5);
    }

    #[test]
    fn ixmax_ixmin() {
        // ixmax e2, e4, d6 (V2 p. 3-115): E[d] = {value 5, index 3, working index 10H}
        let e4 = |c: &mut Cpu<NullBus>, d6: u32| (c.d[4], c.d[5], c.d[6]) = (0x0003_0010, 5, d6);
        let c = one(0x24A0_606B, |c| e4(c, 0x0007_0005));
        assert_eq!((c.d[2], c.d[3]), (0x0011_0012, 7));
        // ties keep the current maximum (priority: current, first, second)
        let c = one(0x24A0_606B, |c| e4(c, 0x0005_0005));
        assert_eq!((c.d[2], c.d[3]), (0x0003_0012, 5));
        // ixmin.u: FFFFH is large; ixmin: FFFFH is -1
        let c = one(0x2490_606B, |c| e4(c, 0xFFFF_0001));
        assert_eq!((c.d[2], c.d[3]), (0x0010_0012, 1));
        let c = one(0x2480_606B, |c| e4(c, 0xFFFF_0001));
        assert_eq!((c.d[2], c.d[3]), (0x0011_0012, 0xFFFF));
    }

    #[test]
    fn bit_fields_unicorn_gets_wrong() {
        // imask e2, #1, d0, #1 (RCRW D7H, firmware): E[c] = {1 << D[d], 1 << D[d]}
        let c = one(0x2021_10D7, |c| c.d[0] = 5);
        assert_eq!((c.d[2], c.d[3]), (0x20, 0x20));
        // insert d3, d1, #0AH, d4, #4 (RCRW)
        let c = one(0x3404_A1D7, |c| (c.d[1], c.d[4]) = (0xFFFF_FFFF, 8));
        assert_eq!(c.d[3], 0xFFFF_FAFF);
        // insert d2, d0, #0FH, e4 (RCRR 97H): E[4] = {width 4, pos 8}
        let c = one(0x2400_F097, |c| (c.d[0], c.d[4], c.d[5]) = (0x1234_5678, 8, 4));
        assert_eq!(c.d[2], 0x1234_5F78);
        // insert d3, d1, #0FH, #4, #0 (RCPW, width 0: mask 0; aborts Unicorn)
        assert_eq!(one(0x3200_F1B7, |c| c.d[1] = 0xCAFE).d[3], 0xCAFE);
        // dextr d3, d1, d2, #0 / #8 (RRPW 77H) and dextr d3, d1, d2, d4 (RRRR)
        let dx = |w: u32, d4: u32| one(w, |c| (c.d[1], c.d[2], c.d[4]) = (0x1122_3344, 0x5566_7788, d4)).d[3];
        assert_eq!(dx(0x3000_2177, 0), 0x1122_3344);
        assert_eq!(dx(0x3400_2177, 0), 0x2233_4455);
        assert_eq!(dx(0x3480_2117, 0), 0x1122_3344);
        assert_eq!(dx(0x3480_2117, 4), 0x1223_3445);
    }

    #[test]
    fn bit_logic() {
        // ins.t d3, d1, #5, d2, #7 / insn.t (V2 p. 3-111)
        assert_eq!(one(0x3385_2167, |c| (c.d[1], c.d[2]) = (0, 0x80)).d[3], 0x20);
        assert_eq!(one(0x33A5_2167, |c| (c.d[1], c.d[2]) = (0x20, 0x80)).d[3], 0);
        // sh.xor.t d3, d1, #0, d2, #0 (V2 p. 3-336): {D[c][30:0], 1 ^ 0}
        assert_eq!(one(0x3060_21A7, |c| (c.d[1], c.d[2], c.d[3]) = (1, 0, 0x8000_0001)).d[3], 3);
        // and.nor.t d3, d1, #0, d2, #0 (V2 p. 3-32): bit 0 ANDed, other bits kept
        assert_eq!(one(0x3040_2147, |c| (c.d[1], c.d[2], c.d[3]) = (0, 0, u32::MAX)).d[3], u32::MAX);
        assert_eq!(one(0x3040_2147, |c| (c.d[1], c.d[2], c.d[3]) = (1, 0, u32::MAX)).d[3], u32::MAX - 1);
        // or.andn.t d3, d1, #0, d2, #0 (V2 p. 3-296)
        assert_eq!(one(0x3060_21C7, |c| (c.d[1], c.d[2], c.d[3]) = (1, 0, 0)).d[3], 1);
    }

    #[test]
    fn count_split_merge_parity() {
        let cnt = |op2: u32, x: u32| one(0x2000_010F | op2 << 20, |c| c.d[1] = x).d[2];
        // CLS (V2 p. 3-70): leading_signs - 1
        assert_eq!([cnt(0x1D, 0), cnt(0x1D, u32::MAX), cnt(0x1D, 0xFFFF), cnt(0x1D, 0x8000_0000)], [31, 31, 15, 0]);
        assert_eq!([cnt(0x7E, 0x0000_FFFF), cnt(0x7E, 0x4000_C000)], [0x000F_000F, 1]);
        assert_eq!([cnt(0x1C, 0xF000_0000), cnt(0x1B, 1), cnt(0x7C, 0x0001_0000), cnt(0x7D, 0xFFFF_8000)], [4, 31, 0x000F_0010, 0x0010_0001]);
        // bsplit e2, d5 (V2 p. 3-46) and bmerge d4, d3, d2 (p. 3-45) round-trip
        let c = run(&[0x2090_054Bu32.to_le_bytes(), 0x4010_234Bu32.to_le_bytes()].concat(), 2, |c| c.d[5] = 0xAAAA_5555);
        assert_eq!((c.d[2], c.d[3], c.d[4]), (0x00FF, 0xFF00, 0xAAAA_5555));
        // parity d2, d1 (V2 p. 3-309)
        assert_eq!(one(0x2020_014B, |c| c.d[1] = 0x0103_07FF).d[2], 0x0100_0100);
        // n = 0H is part of the BMERGE/BSPLIT/PARITY/IXMAX boxes
        assert!(decode(0x014B, 0x2021, 0, Arch::Tc16).is_none());
    }

    #[test]
    fn policy_for_undefined_fields() {
        // E[c] with an odd code: the even register of the pair (mul e3, d1, d2 writes E[2])
        let c = one(0x306A_2173, |c| (c.d[1], c.d[2]) = (0x10000, 0x10000));
        assert_eq!((c.d[2], c.d[3]), (0, 1));
        // n = 2 executes the RTL literally: no -1 * -1 case, (4000_0000H << 2)[31:0] = 0, overflow
        // (mul.q d3, d1l, d2l, #2)
        let c = one(0x3016_2193, |c| (c.d[1], c.d[2]) = (0x8000, 0x8000));
        assert_eq!((c.d[3], c.psw & V), (0, V));
        // extr.u with width 0 returns 0
        assert_eq!(one(0x2260_0137, |c| c.d[1] = u32::MAX).d[2], 0);
    }

    #[test]
    fn no_op_ends_a_block() {
        let ops = [0x2268_0137u32, 0x303F_E153, 0x2061_10B3, 0x2400_F097];
        for w in ops {
            assert!(!ends_block(&decode(w as u16, (w >> 16) as u16, 0, Arch::Tc16).unwrap()));
        }
    }
}
