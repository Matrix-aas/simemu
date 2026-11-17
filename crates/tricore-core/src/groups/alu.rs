//! Group A — integer ALU: add/subtract (incl. saturating, carry, packed .B/.H), absolute value/difference,
//! min/max/saturate, conditional arithmetic (CADD/CSUB/SEL/CMOV), logic, compares (incl. accumulating
//! AND./OR./XOR./SH. compares and packed EQ/LT/EQANY), shifts (SH/SHA/SHAS), moves (MOV*, MOVH*),
//! address arithmetic (ADD.A, ADDSC.A/AT, ADDIH.A, SUB.A, EQ.A, NE.A, LT.A, GE.A, EQZ.A, NEZ.A,
//! MOV.A/AA/D) and RSTV.
//!
//! Sources: V2 = `vault/docs/infineon/tcarch_vol2_a4.pdf` (TriCore V1.6 Vol. 2, V1.0 2012-05), V131-2 =
//! `vault/docs/infineon/Tc_v131_instructionset_v138.pdf` (TriCore 1 V1.3/V1.3.1 Vol. 2, V1.3.8 2008-01).
//! Encodings are the boxes of the V2 pages cited per decoder arm (also in
//! `docs/tricore_core/isa_tc16_encodings.json`); V131-2 has the same syntax lines and RTL for every
//! mnemonic of this group except the four TC1.6-only `MOV E[..]` forms (absent from V131-2 MOV
//! p. 3-273/3-274), which therefore decode only for `Arch::Tc16`. V131-2 encoding boxes are images; the
//! rendered ADDSC.A/AT (p. 3-52), CADD (p. 3-86), RSTV (p. 3-376), SAT.B (p. 3-381) and MOV boxes were
//! checked against V2 and are identical.
//!
//! Semantics notes (where the manual text needs reading):
//! * Status flags follow each page's "Status Flags" block: `overflow` and `advanced_overflow` are judged
//!   on the infinite-precision `result` before saturation (ssov/suov, V2 Table 1-9 p. 1-8);
//!   V/AV are overwritten, SV/SAV sticky. Packed forms OR the per-lane conditions.
//! * GE.U/LT.U RC: V2 p. 3-106/3-175 zero-extend const9. The V131-2 RTL (p. 3-132/3-213) says
//!   `sign_ext(const9) // unsigned` while its own description text says "zero-extended (GE.U)"; the
//!   description (and every accumulating .U form on both manuals) wins, so both archs zero-extend.
//! * EQANY.H (V2 p. 3-97): the RTL `result_halfword1 OR result_halfword1` is a typo for hw1 OR hw0
//!   (description: "logical OR of the Boolean results from each comparison").
//! * SHA.H (V2 p. 3-339): the RTL builds `msk` from `D[a][31]` for both half-words; the description fills
//!   with "the sign-bit of the respective half-word", which is what is implemented.
//! * SHA (V2 p. 3-337): right shifts cannot overflow (the RTL's `msk | D[a] >> n` is the sign-filled
//!   value); V = 0 for them, C = OR of the shifted-out bits, C = 0 for a zero shift.
//! * MOV `E[c]` with an odd c: V1 p. 6-7 "The OPD trap may be raised for instructions that take an
//!   even-odd register pair as an operand, if the operand specifier is odd" (otherwise implementation
//!   defined). It is raised here (class 2 TIN 3), as Unicorn/QEMU do for the 32-bit forms.
//! * Address-register writes to A0/A1/A8/A9 while PSW.GW = 0 raise GRWP (class 1 TIN 7, V1 p. 6-7),
//!   like the load/store group does for LD.A/LEA and base-register updates.
//! * Fields shown as `-` in the encoding boxes are ignored.

use crate::arch::psw;
use crate::flags::{carry, ov32, set_v_av, ssov, suov};
use crate::fmt::{sext, word, Rc, Rcr, Rlc, Rr, Rrr, Srrs, Sys};
use crate::{Arch, Bus, Cpu, Flow, Trap};

/// Mnemonics owned by this group (manual spelling).
pub const MNEMONICS: &[&str] = &[
    "ABS", "ABS.B", "ABS.H", "ABSDIF", "ABSDIF.B", "ABSDIF.H", "ABSDIFS", "ABSDIFS.H", "ABSS",
    "ABSS.H", "ADD", "ADD.A", "ADD.B", "ADD.H", "ADDC", "ADDI", "ADDIH", "ADDIH.A", "ADDS",
    "ADDS.H", "ADDS.HU", "ADDS.U", "ADDSC.A", "ADDSC.AT", "ADDX", "AND", "AND.EQ", "AND.GE",
    "AND.GE.U", "AND.LT", "AND.LT.U", "AND.NE", "ANDN", "CADD", "CADDN", "CMOV", "CMOVN", "CSUB",
    "CSUBN", "EQ", "EQ.A", "EQ.B", "EQ.H", "EQ.W", "EQANY.B", "EQANY.H", "EQZ.A", "GE", "GE.A",
    "GE.U", "LT", "LT.A", "LT.B", "LT.BU", "LT.H", "LT.HU", "LT.U", "LT.W", "LT.WU", "MAX", "MAX.B",
    "MAX.BU", "MAX.H", "MAX.HU", "MAX.U", "MIN", "MIN.B", "MIN.BU", "MIN.H", "MIN.HU", "MIN.U",
    "MOV", "MOV.A", "MOV.AA", "MOV.D", "MOV.U", "MOVH", "MOVH.A", "NAND", "NE", "NE.A", "NEZ.A",
    "NOR", "NOT", "OR", "OR.EQ", "OR.GE", "OR.GE.U", "OR.LT", "OR.LT.U", "OR.NE", "ORN", "RSTV",
    "RSUB", "RSUBS", "RSUBS.U", "SAT.B", "SAT.BU", "SAT.H", "SAT.HU", "SEL", "SELN", "SH", "SH.EQ",
    "SH.GE", "SH.GE.U", "SH.H", "SH.LT", "SH.LT.U", "SH.NE", "SHA", "SHA.H", "SHAS", "SUB", "SUB.A",
    "SUB.B", "SUB.H", "SUBC", "SUBS", "SUBS.H", "SUBS.HU", "SUBS.U", "SUBX", "XNOR", "XOR",
    "XOR.EQ", "XOR.GE", "XOR.GE.U", "XOR.LT", "XOR.LT.U", "XOR.NE"
];

/// Comparison of `D[a]` with the second operand (EQ/NE/LT/LT.U/GE/GE.U).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cc {
    Eq,
    Ne,
    Lt,
    LtU,
    Ge,
    GeU,
}

/// What a comparison does with `D[c]`: plain (EQ...), or accumulate into `D[c][0]` (AND.EQ..., OR.EQ...,
/// XOR.EQ...) or shift into `D[c]` (SH.EQ...).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Acc {
    Set,
    And,
    Or,
    Xor,
    Sh,
}

/// Operation kind. Immediate and register forms of a mnemonic share a kind; RC/RLC/SRC/SC immediates
/// are extended at decode time as the page prescribes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum K {
    Abs,
    AbsS,
    AbsDif,
    AbsDifS,
    Add,
    AddS,
    AddSU,
    AddC,
    AddX,
    Sub,
    SubS,
    SubSU,
    SubC,
    SubX,
    Rsub,
    RsubS,
    RsubSU,
    CAdd,
    CAddN,
    CSub,
    CSubN,
    Sel,
    SelN,
    AbsB,
    AbsH,
    AbsSH,
    AbsDifB,
    AbsDifH,
    AbsDifSH,
    AddB,
    AddH,
    AddSH,
    AddSHU,
    SubB,
    SubH,
    SubSH,
    SubSHU,
    Max,
    MaxU,
    Min,
    MinU,
    MaxB,
    MaxBU,
    MaxH,
    MaxHU,
    MinB,
    MinBU,
    MinH,
    MinHU,
    SatB,
    SatBU,
    SatH,
    SatHU,
    Cmp(Cc, Acc),
    EqB,
    EqH,
    EqW,
    LtB,
    LtBU,
    LtH,
    LtHU,
    LtW,
    LtWU,
    EqAnyB,
    EqAnyH,
    And,
    AndN,
    Nand,
    Nor,
    Or,
    OrN,
    Xnor,
    Xor,
    Sh,
    ShH,
    Sha,
    ShaH,
    ShaS,
    Mov,
    /// `E[c] = sign_ext(src)`
    MovE,
    /// `E[c] = {D[a], D[b]}`
    MovE2,
    /// MOV `E[c]` with an odd pair specifier: OPD trap
    Opd,
    AddA,
    SubA,
    MovA,
    MovAA,
    MovD,
    /// `A[c] = A[b] + (D[a] << imm)`
    AddScA,
    AddScAt,
    EqA,
    NeA,
    LtA,
    GeA,
    EqzA,
    NezA,
    Rstv,
}

/// A predecoded group-A instruction. `y` (second source) is `imm` when `is_imm`, else register `b`;
/// `d` is the condition register of CADD/CSUB/SEL (and of the 16-bit CMOV forms, `D[15]`).
#[derive(Clone, Copy, Debug)]
pub struct AluOp {
    pub(crate) k: K,
    pub(crate) c: u8,
    pub(crate) a: u8,
    pub(crate) b: u8,
    pub(crate) d: u8,
    pub(crate) is_imm: bool,
    pub(crate) imm: u32,
}

const fn rr(k: K, c: u32, a: u32, b: u32) -> AluOp {
    AluOp { k, c: c as u8, a: a as u8, b: b as u8, d: 0, is_imm: false, imm: 0 }
}

const fn ri(k: K, c: u32, a: u32, imm: u32) -> AluOp {
    AluOp { k, c: c as u8, a: a as u8, b: 0, d: 0, is_imm: true, imm }
}

impl AluOp {
    const fn cond(mut self, d: u32) -> Self {
        self.d = d as u8;
        self
    }

    /// MOV E[c] with an odd register-pair specifier becomes an OPD trap (module docs).
    fn pair(self) -> Self {
        if matches!(self.k, K::MovE | K::MovE2) && self.c & 1 != 0 {
            AluOp { k: K::Opd, ..self }
        } else {
            self
        }
    }
}

const CC: [Cc; 6] = [Cc::Eq, Cc::Ne, Cc::Lt, Cc::LtU, Cc::Ge, Cc::GeU];

/// Compare op2 blocks shared by RR 0BH and RC 8BH: EQ 10H.., AND.EQ 20H.., OR.EQ 27H.., XOR.EQ 2FH..,
/// SH.EQ 37H.., each ordered EQ, NE, LT, LT.U, GE, GE.U (V2 p. 3-34..3-39, 3-92, 3-106, 3-174, 3-287,
/// 3-298..3-303, 3-328..3-334, 3-386..3-391).
fn cmp_op2(op2: u32) -> Option<(Cc, Acc)> {
    for (base, acc) in [(0x10, Acc::Set), (0x20, Acc::And), (0x27, Acc::Or), (0x2F, Acc::Xor), (0x37, Acc::Sh)] {
        if (base..base + 6).contains(&op2) {
            return Some((CC[(op2 - base) as usize], acc));
        }
    }
    None
}

pub fn decode(lo: u16, hi: u16, _pc: u32, arch: Arch) -> Option<AluOp> {
    use K::*;
    if lo & 1 == 0 {
        return decode16(lo, arch);
    }
    let w = word(lo, hi);
    let tc16 = arch == Arch::Tc16;
    match w as u8 {
        // RR 0BH: arithmetic, compare, packed, MOV (pages per arm)
        0x0B => {
            let f = Rr::from(w);
            if let Some((cc, acc)) = cmp_op2(f.op2) {
                return Some(rr(Cmp(cc, acc), f.c, f.a, f.b));
            }
            let k = match f.op2 {
                0x00 => Add,     // V2 p. 3-12
                0x02 => AddS,    // p. 3-22
                0x03 => AddSU,   // p. 3-26
                0x04 => AddX,    // p. 3-29
                0x05 => AddC,    // p. 3-18
                0x08 => Sub,     // p. 3-364
                0x0A => SubS,    // p. 3-370
                0x0B => SubSU,   // p. 3-370
                0x0C => SubX,    // p. 3-374
                0x0D => SubC,    // p. 3-369
                0x0E => AbsDif,  // p. 3-5
                0x0F => AbsDifS, // p. 3-8
                0x18 => Min,     // p. 3-220
                0x19 => MinU,
                0x1A => Max, // p. 3-215
                0x1B => MaxU,
                0x1C => Abs,  // p. 3-2
                0x1D => AbsS, // p. 3-10
                0x1F => Mov,  // p. 3-224
                0x40 => AddB, // p. 3-16
                0x48 => SubB, // p. 3-367
                0x4E => AbsDifB, // p. 3-6
                0x50 => EqB,  // p. 3-95
                0x52 => LtB,  // p. 3-177
                0x53 => LtBU,
                0x56 => EqAnyB, // p. 3-97
                0x58 => MinB,   // p. 3-222
                0x59 => MinBU,
                0x5A => MaxB, // p. 3-217
                0x5B => MaxBU,
                0x5C => AbsB,  // p. 3-3
                0x5E => SatB,  // p. 3-320
                0x5F => SatBU, // p. 3-321
                0x60 => AddH,  // p. 3-16
                0x62 => AddSH, // p. 3-24
                0x63 => AddSHU,
                0x68 => SubH,  // p. 3-367
                0x6A => SubSH, // p. 3-372
                0x6B => SubSHU,
                0x6E => AbsDifH,  // p. 3-6
                0x6F => AbsDifSH, // p. 3-9
                0x70 => EqH,      // p. 3-95
                0x72 => LtH,      // p. 3-178
                0x73 => LtHU,
                0x76 => EqAnyH, // p. 3-97
                0x78 => MinH,   // p. 3-223
                0x79 => MinHU,
                0x7A => MaxH, // p. 3-218
                0x7B => MaxHU,
                0x7C => AbsH,          // p. 3-3
                0x7D => AbsSH,         // p. 3-11
                0x7E => SatH,          // p. 3-322
                0x7F => SatHU,         // p. 3-323
                0x80 if tc16 => MovE,  // p. 3-224, TC1.6 only
                0x81 if tc16 => MovE2, // p. 3-224, TC1.6 only
                0x90 => EqW,           // p. 3-95
                0x92 => LtW,           // p. 3-179
                0x93 => LtWU,
                _ => return None,
            };
            Some(rr(k, f.c, f.a, f.b).pair())
        }
        // RC 8BH: same arithmetic/compare op2 values with const9 (sign-extended unless the page says
        // zero_ext: the unsigned compares and MIN.U/MAX.U)
        0x8B => {
            let f = Rc::from(w);
            let (s9, z9) = (sext(f.const9, 9), f.const9);
            if let Some((cc, acc)) = cmp_op2(f.op2) {
                let v = if matches!(cc, Cc::LtU | Cc::GeU) { z9 } else { s9 };
                return Some(ri(Cmp(cc, acc), f.c, f.a, v));
            }
            let (k, v) = match f.op2 {
                0x00 => (Add, s9),     // p. 3-12
                0x02 => (AddS, s9),    // p. 3-22
                0x03 => (AddSU, s9),   // p. 3-26 (sign_ext, then unsigned)
                0x04 => (AddX, s9),    // p. 3-29
                0x05 => (AddC, s9),    // p. 3-18
                0x08 => (Rsub, s9),    // p. 3-318
                0x0A => (RsubS, s9),   // p. 3-319
                0x0B => (RsubSU, s9),  // p. 3-319 (sign_ext, then unsigned)
                0x0E => (AbsDif, s9),  // p. 3-5
                0x0F => (AbsDifS, s9), // p. 3-8
                0x18 => (Min, s9),     // p. 3-220
                0x19 => (MinU, z9),
                0x1A => (Max, s9), // p. 3-215
                0x1B => (MaxU, z9),
                0x56 => (EqAnyB, s9), // p. 3-97
                0x76 => (EqAnyH, s9),
                _ => return None,
            };
            Some(ri(k, f.c, f.a, v))
        }
        // RR 0FH / RC 8FH: logic and shifts (const9 zero-extended; shifts use const9[5:0] / [4:0])
        0x0F | 0x8F => {
            let (is_rc, f, g) = (w & 0x80 != 0, Rr::from(w), Rc::from(w));
            let k = match if is_rc { g.op2 } else { f.op2 } {
                0x00 => Sh,   // p. 3-326
                0x01 => Sha,  // p. 3-337
                0x02 => ShaS, // p. 3-341
                0x08 => And,  // p. 3-30
                0x09 => Nand, // p. 3-285
                0x0A => Or,   // p. 3-294
                0x0B => Nor,  // p. 3-291
                0x0C => Xor,  // p. 3-385
                0x0D => Xnor, // p. 3-383
                0x0E => AndN, // p. 3-41
                0x0F => OrN,  // p. 3-305
                0x40 => ShH,  // p. 3-331
                0x41 => ShaH, // p. 3-339
                _ => return None,
            };
            Some(if is_rc { ri(k, g.c, g.a, g.const9) } else { rr(k, f.c, f.a, f.b) })
        }
        // RR 01H: address arithmetic/compare/move
        0x01 => {
            let f = Rr::from(w);
            let k = match f.op2 {
                0x00 => MovAA,   // p. 3-228
                0x01 => AddA,    // p. 3-15
                0x02 => SubA,    // p. 3-366
                0x40 => EqA,     // p. 3-94
                0x41 => NeA,     // p. 3-288
                0x42 => LtA,     // p. 3-176
                0x43 => GeA,     // p. 3-108
                0x48 => EqzA,    // p. 3-99
                0x49 => NezA,    // p. 3-289
                0x4C => MovD,    // p. 3-229
                0x60 => AddScA,  // p. 3-27
                0x62 => AddScAt, // p. 3-27
                0x63 => MovA,    // p. 3-227
                _ => return None,
            };
            let mut op = rr(k, f.c, f.a, f.b);
            op.imm = f.n; // ADDSC.A shift amount
            Some(op)
        }
        // RRR 2BH: CADD/CADDN/CSUB/CSUBN/SEL/SELN (p. 3-59, 3-61, 3-76, 3-77, 3-324, 3-325)
        0x2B => {
            let f = Rrr::from(w);
            let k = [CAdd, CAddN, CSub, CSubN, Sel, SelN].get(f.op2 as usize).copied()?;
            Some(rr(k, f.c, f.a, f.b).cond(f.d))
        }
        // RCR ABH: CADD 0, CADDN 1, SEL 4, SELN 5 with sign-extended const9 (same pages)
        0xAB => {
            let f = Rcr::from(w);
            let k = match f.op2 {
                0 => CAdd,
                1 => CAddN,
                4 => Sel,
                5 => SelN,
                _ => return None,
            };
            Some(ri(k, f.c, f.a, sext(f.const9, 9)).cond(f.d))
        }
        // RLC: ADDI 1BH p. 3-19, ADDIH 9BH p. 3-20, ADDIH.A 11H p. 3-21, MOV 3BH / MOV E FBH p. 3-224,
        // MOV.U BBH p. 3-230, MOVH 7BH p. 3-231, MOVH.A 91H p. 3-232
        0x1B | 0x9B | 0x11 | 0x3B | 0xFB | 0xBB | 0x7B | 0x91 => {
            let f = Rlc::from(w);
            let (k16, s16, h16) = (f.const16, sext(f.const16, 16), f.const16 << 16);
            Some(match f.op1 {
                0x1B => ri(Add, f.c, f.a, s16),
                0x9B => ri(Add, f.c, f.a, h16),
                0x11 => ri(AddA, f.c, f.a, h16),
                0x3B => ri(Mov, f.c, 0, s16),
                0xFB if tc16 => ri(MovE, f.c, 0, s16).pair(),
                0xBB => ri(Mov, f.c, 0, k16),
                0x7B => ri(Mov, f.c, 0, h16),
                0x91 => ri(MovA, f.c, 0, h16),
                _ => return None,
            })
        }
        // SYS 2FH op2 00H: RSTV p. 3-317
        0x2F if Sys::from(w).op2 == 0 => Some(rr(Rstv, 0, 0, 0)),
        _ => None,
    }
}

/// 16-bit encodings (V2 Table 1-5 formats SRC/SRR/SC/SR/SRRS; pages as for the 32-bit forms).
fn decode16(h: u16, arch: Arch) -> Option<AluOp> {
    use K::*;
    let op1 = h as u8;
    let (a, b) = (((h >> 8) & 15) as u32, ((h >> 12) & 15) as u32);
    let (k4, k8) = (sext(b, 4), (h >> 8) as u32);
    // SRRS (op1 = bits [5:0]): ADDSC.A A[a], A[b], D[15], n — A[a] = A[b] + (D[15] << n), p. 3-27
    if op1 & 0x3F == 0x10 {
        let f = Srrs::from(h);
        let mut op = rr(AddScA, f.a, 15, f.b);
        op.imm = f.n;
        return Some(op);
    }
    Some(match op1 {
        // SRC (const4 sign-extended except MOV.A)
        0xC2 => ri(Add, a, a, k4),                // ADD D[a], const4         p. 3-12
        0x92 => ri(Add, a, 15, k4),               // ADD D[a], D[15], const4
        0x9A => ri(Add, 15, a, k4),               // ADD D[15], D[a], const4
        0xB0 => ri(AddA, a, a, k4),               // ADD.A A[a], const4       p. 3-15
        0x8A => ri(CAdd, a, a, k4).cond(15),      // CADD D[a], D[15], const4 p. 3-59
        0xCA => ri(CAddN, a, a, k4).cond(15),     // CADDN                    p. 3-61
        // CMOV D[a] = D[15] != 0 ? src : D[a]  ==  SELN D[a], D[15], D[a], src   p. 3-74
        0xAA => ri(SelN, a, a, k4).cond(15),
        // CMOVN D[a] = D[15] == 0 ? src : D[a]  ==  SEL D[a], D[15], D[a], src   p. 3-75
        0xEA => ri(Sel, a, a, k4).cond(15),
        0xBA => ri(Cmp(Cc::Eq, Acc::Set), 15, a, k4), // EQ D[15], D[a], const4 p. 3-92
        0xFA => ri(Cmp(Cc::Lt, Acc::Set), 15, a, k4), // LT D[15], D[a], const4 p. 3-174
        0x82 => ri(Mov, a, 0, k4),                    // MOV D[a], const4       p. 3-225
        0xD2 if arch == Arch::Tc16 => ri(MovE, a, 0, k4).pair(), // MOV E[a], const4 (TC1.6 only)
        0xA0 => ri(MovA, a, 0, b),                    // MOV.A A[a], const4 (zero_ext) p. 3-227
        0x06 => ri(Sh, a, a, k4),                     // SH D[a], const4        p. 3-326
        0x86 => ri(Sha, a, a, k4),                    // SHA D[a], const4       p. 3-338
        // SRR
        0x42 => rr(Add, a, a, b),              // ADD D[a], D[b]         p. 3-13
        0x12 => rr(Add, a, 15, b),             // ADD D[a], D[15], D[b]
        0x1A => rr(Add, 15, a, b),             // ADD D[15], D[a], D[b]
        0x30 => rr(AddA, a, a, b),             // ADD.A A[a], A[b]       p. 3-15
        0x22 => rr(AddS, a, a, b),             // ADDS D[a], D[b]        p. 3-22
        0x26 => rr(And, a, a, b),              // AND D[a], D[b]         p. 3-30
        0x2A => rr(SelN, a, a, b).cond(15),    // CMOV D[a], D[15], D[b]  (see 0xAA)
        0x6A => rr(Sel, a, a, b).cond(15),     // CMOVN D[a], D[15], D[b] (see 0xEA)
        0x3A => rr(Cmp(Cc::Eq, Acc::Set), 15, a, b), // EQ D[15], D[a], D[b]
        0x7A => rr(Cmp(Cc::Lt, Acc::Set), 15, a, b), // LT D[15], D[a], D[b]
        0x02 => rr(Mov, a, 0, b),              // MOV D[a], D[b]         p. 3-225
        0x60 => rr(MovA, a, 0, b),             // MOV.A A[a], D[b]       p. 3-227
        0x40 => rr(MovAA, a, 0, b),            // MOV.AA A[a], A[b]      p. 3-228
        0x80 => rr(MovD, a, 0, b),             // MOV.D D[a], A[b]       p. 3-229
        0xA6 => rr(Or, a, a, b),               // OR D[a], D[b]          p. 3-294
        0xA2 => rr(Sub, a, a, b),              // SUB D[a], D[b]         p. 3-364
        0x52 => rr(Sub, a, 15, b),             // SUB D[a], D[15], D[b]
        0x5A => rr(Sub, 15, a, b),             // SUB D[15], D[a], D[b]
        0x62 => rr(SubS, a, a, b),             // SUBS D[a], D[b]        p. 3-370
        0xC6 => rr(Xor, a, a, b),              // XOR D[a], D[b]         p. 3-385
        // SC (const8 zero-extended)
        0x16 => ri(And, 15, 15, k8),           // AND D[15], const8      p. 3-30
        0x96 => ri(Or, 15, 15, k8),            // OR D[15], const8       p. 3-294
        0xDA => ri(Mov, 15, 0, k8),            // MOV D[15], const8      p. 3-224
        0x20 => ri(SubA, 10, 10, k8),          // SUB.A A[10], const8    p. 3-366
        // SR (op2 in [15:12])
        0x46 if b == 0 => ri(Nor, a, a, 0),    // NOT D[a] = ~(D[a] | 0)  p. 3-293
        0x32 => match b {
            0 => rr(SatB, a, a, 0),            // SAT.B D[a]   p. 3-320
            1 => rr(SatBU, a, a, 0),           // SAT.BU D[a]  p. 3-321
            2 => rr(SatH, a, a, 0),            // SAT.H D[a]   p. 3-322
            3 => rr(SatHU, a, a, 0),           // SAT.HU D[a]  p. 3-323
            5 => ri(Rsub, a, a, 0),            // RSUB D[a] = 0 - D[a]  p. 3-318
            _ => return None,
        },
        _ => return None,
    })
}

/// Group-A instructions never transfer control or need re-arbitration; only the OPD form traps.
pub fn ends_block(op: &AluOp) -> bool {
    op.k == K::Opd
}

#[inline(always)]
fn s(x: u32) -> i64 {
    x as i32 as i64
}

#[inline(always)]
fn u(x: u32) -> i64 {
    x as i64
}

/// `result[31] ^ result[30]` of an infinite-precision result (advanced overflow, V2 p. 1-8).
#[inline(always)]
fn aov32(r: i64) -> bool {
    ((r >> 31) ^ (r >> 30)) & 1 != 0
}

/// Saturating 32-bit result: `ssov`/`suov(result, 32)` with V/AV judged on the unsaturated result.
#[inline(always)]
fn sat32(p: &mut u32, r: i64, unsigned: bool) -> u32 {
    let v = if unsigned { suov(r, 32) } else { ssov(r, 32) };
    set_v_av(p, v != r, aov32(r));
    v as u32
}

/// Lane-wise operation on the `w`-bit lanes (8 or 16) of `x` and `y`, extended as signed or unsigned.
/// `f` returns the infinite-precision lane result. Returns the packed lane results (saturated to the
/// lane range when `sat`, else truncated), whether any lane left the lane range (overflow) and whether
/// any lane had `result[w-1] ^ result[w-2]` (advanced overflow).
#[inline(always)]
fn lanes(x: u32, y: u32, w: u32, unsigned: bool, sat: bool, f: impl Fn(i64, i64) -> i64) -> (u32, bool, bool) {
    let m = (1u32 << w) - 1;
    let (lo, hi) = if unsigned { (0, m as i64) } else { (-(1i64 << (w - 1)), (1i64 << (w - 1)) - 1) };
    let ext = |v: u32| if unsigned { (v & m) as i64 } else { sext(v & m, w) as i32 as i64 };
    let (mut r, mut ov, mut aov) = (0u32, false, false);
    let mut sh = 0;
    while sh < 32 {
        let res = f(ext(x >> sh), ext(y >> sh));
        ov |= res < lo || res > hi;
        aov |= ((res >> (w - 1)) ^ (res >> (w - 2))) & 1 != 0;
        let v = if sat { res.clamp(lo, hi) } else { res };
        r |= (v as u32 & m) << sh;
        sh += w;
    }
    (r, ov, aov)
}

/// [`lanes`] with the packed V/SV/AV/SAV update.
#[inline(always)]
fn lanes_flags(p: &mut u32, x: u32, y: u32, w: u32, unsigned: bool, sat: bool, f: impl Fn(i64, i64) -> i64) -> u32 {
    let (r, ov, aov) = lanes(x, y, w, unsigned, sat, f);
    set_v_av(p, ov, aov);
    r
}

/// Lane-wise map without flags (MIN/MAX/EQ/LT packed): `f` returns the lane value (all-ones = -1).
#[inline(always)]
fn lanes_map(x: u32, y: u32, w: u32, unsigned: bool, f: impl Fn(i64, i64) -> i64) -> u32 {
    lanes(x, y, w, unsigned, false, f).0
}

#[inline(always)]
fn compare(cc: Cc, x: u32, y: u32) -> bool {
    match cc {
        Cc::Eq => x == y,
        Cc::Ne => x != y,
        Cc::Lt => (x as i32) < (y as i32),
        Cc::LtU => x < y,
        Cc::Ge => (x as i32) >= (y as i32),
        Cc::GeU => x >= y,
    }
}

#[inline(always)]
fn mask(b: bool) -> i64 {
    -(b as i64)
}

/// Shift count: sign-extension of the low `bits` bits (SH/SHA/SHAS: 6 = -32..31, SH.H/SHA.H: 5).
#[inline(always)]
fn count(y: u32, bits: u32) -> i32 {
    sext(y & ((1 << bits) - 1), bits) as i32
}

#[inline(always)]
pub fn exec<B: Bus>(cpu: &mut Cpu<B>, op: &AluOp, pc: u32, _len: u32) -> Flow {
    use K::*;
    let (c, a, b) = (op.c as usize & 15, op.a as usize & 15, op.b as usize & 15);
    let x = cpu.d[a];
    let y = if op.is_imm { op.imm } else { cpu.d[b] };
    let p = &mut cpu.psw;
    let r = match op.k {
        // ---- 32-bit arithmetic: V2 p. 3-2..3-29, 3-318..3-319, 3-364..3-374 --------------------
        Abs => ov32(p, s(y).abs()),
        AbsS => sat32(p, s(y).abs(), false),
        AbsDif => ov32(p, (s(x) - s(y)).abs()),
        AbsDifS => sat32(p, (s(x) - s(y)).abs(), false),
        Add => ov32(p, s(x) + s(y)),
        AddS => sat32(p, s(x) + s(y), false),
        AddSU => sat32(p, u(x) + u(y), true),
        AddC | AddX => {
            let cin = if op.k == AddC { *p >> 31 } else { 0 };
            let r = ov32(p, s(x) + s(y) + cin as i64);
            *p = (*p & !psw::C) | (carry(x, y, cin) as u32) << 31;
            r
        }
        Sub => ov32(p, s(x) - s(y)),
        SubS => sat32(p, s(x) - s(y), false),
        SubSU => sat32(p, u(x) - u(y), true),
        // SUBC: D[a] - D[b] + C - 1, carry(D[a], ~D[b], C); SUBX: carry(D[a], ~D[b], 1)
        SubC | SubX => {
            let cin = if op.k == SubC { *p >> 31 } else { 1 };
            let r = ov32(p, s(x) - s(y) + cin as i64 - 1);
            *p = (*p & !psw::C) | (carry(x, !y, cin) as u32) << 31;
            r
        }
        Rsub => ov32(p, s(y) - s(x)),
        RsubS => sat32(p, s(y) - s(x), false),
        RsubSU => sat32(p, u(y) - u(x), true),
        // ---- conditional: V2 p. 3-59..3-77, 3-324..3-325 (flags only written when executed) ----
        CAdd | CAddN | CSub | CSubN => {
            let cond = (cpu.d[op.d as usize & 15] != 0) == matches!(op.k, CAdd | CSub);
            if !cond {
                x
            } else if matches!(op.k, CAdd | CAddN) {
                ov32(&mut cpu.psw, s(x) + s(y))
            } else {
                ov32(&mut cpu.psw, s(x) - s(y))
            }
        }
        Sel => if cpu.d[op.d as usize & 15] != 0 { x } else { y },
        SelN => if cpu.d[op.d as usize & 15] == 0 { x } else { y },
        // ---- packed arithmetic: V2 p. 3-3, 3-6, 3-9, 3-11, 3-16, 3-24, 3-367, 3-372 -----------
        AbsB => lanes_flags(p, y, 0, 8, false, false, |v, _| v.abs()),
        AbsH => lanes_flags(p, y, 0, 16, false, false, |v, _| v.abs()),
        AbsSH => lanes_flags(p, y, 0, 16, false, true, |v, _| v.abs()),
        AbsDifB => lanes_flags(p, x, y, 8, false, false, |v, w| (v - w).abs()),
        AbsDifH => lanes_flags(p, x, y, 16, false, false, |v, w| (v - w).abs()),
        AbsDifSH => lanes_flags(p, x, y, 16, false, true, |v, w| (v - w).abs()),
        AddB => lanes_flags(p, x, y, 8, false, false, |v, w| v + w),
        AddH => lanes_flags(p, x, y, 16, false, false, |v, w| v + w),
        AddSH => lanes_flags(p, x, y, 16, false, true, |v, w| v + w),
        AddSHU => lanes_flags(p, x, y, 16, true, true, |v, w| v + w),
        SubB => lanes_flags(p, x, y, 8, false, false, |v, w| v - w),
        SubH => lanes_flags(p, x, y, 16, false, false, |v, w| v - w),
        SubSH => lanes_flags(p, x, y, 16, false, true, |v, w| v - w),
        SubSHU => lanes_flags(p, x, y, 16, true, true, |v, w| v - w),
        // ---- min/max/saturate: V2 p. 3-215..3-223, 3-320..3-323 --------------------------------
        Max => (s(x).max(s(y))) as u32,
        MaxU => x.max(y),
        Min => (s(x).min(s(y))) as u32,
        MinU => x.min(y),
        MaxB => lanes_map(x, y, 8, false, i64::max),
        MaxBU => lanes_map(x, y, 8, true, i64::max),
        MaxH => lanes_map(x, y, 16, false, i64::max),
        MaxHU => lanes_map(x, y, 16, true, i64::max),
        MinB => lanes_map(x, y, 8, false, i64::min),
        MinBU => lanes_map(x, y, 8, true, i64::min),
        MinH => lanes_map(x, y, 16, false, i64::min),
        MinHU => lanes_map(x, y, 16, true, i64::min),
        SatB => s(x).clamp(-0x80, 0x7F) as u32,
        SatBU => x.min(0xFF),
        SatH => s(x).clamp(-0x8000, 0x7FFF) as u32,
        SatHU => x.min(0xFFFF),
        // ---- compares: V2 p. 3-34..3-39, 3-92..3-97, 3-106, 3-174..3-179, 3-287, 3-298..3-303,
        //      3-328..3-334, 3-386..3-391 ------------------------------------------------------
        Cmp(cc, acc) => {
            let t = compare(cc, x, y) as u32;
            let old = cpu.d[c];
            match acc {
                Acc::Set => t,
                Acc::And => old & (!1 | t),
                Acc::Or => old | t,
                Acc::Xor => old ^ t,
                Acc::Sh => old << 1 | t,
            }
        }
        EqB => lanes_map(x, y, 8, true, |v, w| mask(v == w)),
        EqH => lanes_map(x, y, 16, true, |v, w| mask(v == w)),
        EqW => mask(x == y) as u32,
        LtB => lanes_map(x, y, 8, false, |v, w| mask(v < w)),
        LtBU => lanes_map(x, y, 8, true, |v, w| mask(v < w)),
        LtH => lanes_map(x, y, 16, false, |v, w| mask(v < w)),
        LtHU => lanes_map(x, y, 16, true, |v, w| mask(v < w)),
        LtW => mask((x as i32) < (y as i32)) as u32,
        LtWU => mask(x < y) as u32,
        EqAnyB => (lanes_map(x, y, 8, true, |v, w| (v == w) as i64) != 0) as u32,
        EqAnyH => (lanes_map(x, y, 16, true, |v, w| (v == w) as i64) != 0) as u32,
        // ---- logic: V2 p. 3-30, 3-41, 3-285, 3-291, 3-293, 3-294, 3-305, 3-383, 3-385 -----------
        And => x & y,
        AndN => x & !y,
        Nand => !(x & y),
        Nor => !(x | y),
        Or => x | y,
        OrN => x | !y,
        Xnor => !(x ^ y),
        Xor => x ^ y,
        // ---- shifts: V2 p. 3-326, 3-331, 3-337..3-341 -------------------------------------------
        Sh => {
            let n = count(y, 6);
            if n >= 0 { x << n } else { (u(x) >> -n) as u32 }
        }
        ShH => {
            let n = count(y, 5);
            let h = |v: u32| (if n >= 0 { (v & 0xFFFF) << n } else { (v & 0xFFFF) >> -n }) & 0xFFFF;
            h(x) | h(x >> 16) << 16
        }
        Sha => {
            let n = count(y, 6);
            let (r, cy, v) = if n >= 0 {
                let wide = s(x) << n;
                (wide as u32, n != 0 && x >> (32 - n) != 0, wide != wide as i32 as i64)
            } else {
                ((s(x) >> -n) as u32, u(x) & ((1i64 << -n) - 1) != 0, false)
            };
            set_v_av(p, v, (r ^ r << 1) >> 31 != 0);
            *p = (*p & !psw::C) | (cy as u32) << 31;
            r
        }
        ShaH => {
            let n = count(y, 5);
            let h = |v: u32| {
                let v = v as u16 as i16 as i32;
                (if n >= 0 { v << n } else { v >> -n }) as u32 & 0xFFFF
            };
            h(x) | h(x >> 16) << 16
        }
        ShaS => {
            let n = count(y, 6);
            sat32(p, if n >= 0 { s(x) << n } else { s(x) >> -n }, false)
        }
        // ---- moves: V2 p. 3-224..3-232 ------------------------------------------------------------
        Mov => y,
        MovE | MovE2 => {
            let (lo, hi) = if op.k == MovE { (y, (y as i32 >> 31) as u32) } else { (y, x) };
            cpu.d[c] = lo;
            cpu.d[c | 1] = hi;
            return Flow::Next;
        }
        Opd => return Flow::trap(Trap::OPD, pc),
        MovD => cpu.a[b],
        // ---- address registers: V2 p. 3-15, 3-21, 3-27, 3-94, 3-99, 3-108, 3-176, 3-227..3-232,
        //      3-288, 3-289, 3-366 ------------------------------------------------------------------
        AddA | SubA | MovA | MovAA | AddScA | AddScAt => {
            // GRWP: write to A0/A1/A8/A9 while PSW.GW = 0 (V1 p. 6-7, PSW.GW p. 3-6)
            if 0x0303 >> c & 1 != 0 && *p & psw::GW == 0 {
                return Flow::trap(Trap::GRWP, pc);
            }
            let ya = if op.is_imm { op.imm } else { cpu.a[b] };
            cpu.a[c] = match op.k {
                AddA => cpu.a[a].wrapping_add(ya),
                SubA => cpu.a[a].wrapping_sub(ya),
                MovA => y,
                MovAA => ya,
                AddScA => cpu.a[b].wrapping_add(x << op.imm),
                _ => cpu.a[b].wrapping_add((x as i32 >> 3) as u32) & !3,
            };
            return Flow::Next;
        }
        EqA => (cpu.a[a] == cpu.a[b]) as u32,
        NeA => (cpu.a[a] != cpu.a[b]) as u32,
        LtA => (cpu.a[a] < cpu.a[b]) as u32,
        GeA => (cpu.a[a] >= cpu.a[b]) as u32,
        EqzA => (cpu.a[a] == 0) as u32,
        NezA => (cpu.a[a] != 0) as u32,
        // ---- RSTV: V2 p. 3-317 ---------------------------------------------------------------------
        Rstv => {
            *p &= !(psw::V | psw::SV | psw::AV | psw::SAV);
            return Flow::Next;
        }
    };
    cpu.d[c] = r;
    Flow::Next
}


#[cfg(test)]
mod tests {
    //! (1) `manual_vectors`: at least one hand-computed vector per encoding from the V2 page RTL (the
    //!     test checks that every row of [`ENCODINGS`] is exercised);
    //! (2) `decoder_claims_exactly_the_manual_encodings`: exhaustive 16-bit / sampled 32-bit check
    //!     that the decoder accepts exactly the manual's encodings (TC1.6-only forms only on TC1.6);
    //! (3) `differential_vs_unicorn_*`: random fields and edge-biased operands, full register/PSW/PC/
    //!     PCXI/ICR/FCX state compared with bare Unicorn (`SIMEMU_ALU_DIFF_N` samples per encoding;
    //!     default 500 in debug, 5000 in release builds).
    use super::*;
    use crate::testutil::{self, link, oracle, Setup, CODE, CSA_BASE, DATA, TRAP_BASE};
    use crate::NullBus;
    use std::collections::HashSet;

    type C = Cpu<NullBus>;

    #[allow(clippy::upper_case_acronyms)]
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum F {
        RR,
        RC,
        RCR,
        RRR,
        RLC,
        SYS,
        SRC,
        SRR,
        SC,
        SR,
        SRRS,
    }

    type Enc = (&'static str, &'static str, F, u8, Option<u8>, &'static str);

    /// Every encoding of this group's mnemonics in V2 (mnemonic, operands, format, op1, op2, page),
    /// generated from `docs/tricore_core/isa_tc16_encodings.json`.
    const ENCODINGS: &[Enc] = &[
        ("ABS", "D[c],D[b]", F::RR, 0x0B, Some(0x1C), "3-2"),
        ("ABS.B", "D[c],D[b]", F::RR, 0x0B, Some(0x5C), "3-3"),
        ("ABS.H", "D[c],D[b]", F::RR, 0x0B, Some(0x7C), "3-3"),
        ("ABSDIF", "D[c],D[a],const9", F::RC, 0x8B, Some(0x0E), "3-5"),
        ("ABSDIF", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x0E), "3-5"),
        ("ABSDIF.B", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x4E), "3-6"),
        ("ABSDIF.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x6E), "3-6"),
        ("ABSDIFS", "D[c],D[a],const9", F::RC, 0x8B, Some(0x0F), "3-8"),
        ("ABSDIFS", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x0F), "3-8"),
        ("ABSDIFS.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x6F), "3-9"),
        ("ABSS", "D[c],D[b]", F::RR, 0x0B, Some(0x1D), "3-10"),
        ("ABSS.H", "D[c],D[b]", F::RR, 0x0B, Some(0x7D), "3-11"),
        ("ADD", "D[c],D[a],const9", F::RC, 0x8B, Some(0x00), "3-12"),
        ("ADD", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x00), "3-12"),
        ("ADD", "D[a],const4", F::SRC, 0xC2, None, "3-12"),
        ("ADD", "D[a],D[15],const4", F::SRC, 0x92, None, "3-12"),
        ("ADD", "D[15],D[a],const4", F::SRC, 0x9A, None, "3-12"),
        ("ADD", "D[a],D[b]", F::SRR, 0x42, None, "3-13"),
        ("ADD", "D[a],D[15],D[b]", F::SRR, 0x12, None, "3-13"),
        ("ADD", "D[15],D[a],D[b]", F::SRR, 0x1A, None, "3-13"),
        ("ADD.A", "A[c],A[a],A[b]", F::RR, 0x01, Some(0x01), "3-15"),
        ("ADD.A", "A[a],const4", F::SRC, 0xB0, None, "3-15"),
        ("ADD.A", "A[a],A[b]", F::SRR, 0x30, None, "3-15"),
        ("ADD.B", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x40), "3-16"),
        ("ADD.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x60), "3-16"),
        ("ADDC", "D[c],D[a],const9", F::RC, 0x8B, Some(0x05), "3-18"),
        ("ADDC", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x05), "3-18"),
        ("ADDI", "D[c],D[a],const16", F::RLC, 0x1B, None, "3-19"),
        ("ADDIH", "D[c],D[a],const16", F::RLC, 0x9B, None, "3-20"),
        ("ADDIH.A", "A[c],A[a],const16", F::RLC, 0x11, None, "3-21"),
        ("ADDS", "D[c],D[a],const9", F::RC, 0x8B, Some(0x02), "3-22"),
        ("ADDS", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x02), "3-22"),
        ("ADDS", "D[a],D[b]", F::SRR, 0x22, None, "3-22"),
        ("ADDS.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x62), "3-24"),
        ("ADDS.HU", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x63), "3-24"),
        ("ADDS.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x03), "3-26"),
        ("ADDS.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x03), "3-26"),
        ("ADDSC.A", "A[c],A[b],D[a],n", F::RR, 0x01, Some(0x60), "3-27"),
        ("ADDSC.A", "A[a],A[b],D[15],n", F::SRRS, 0x10, None, "3-27"),
        ("ADDSC.AT", "A[c],A[b],D[a]", F::RR, 0x01, Some(0x62), "3-27"),
        ("ADDX", "D[c],D[a],const9", F::RC, 0x8B, Some(0x04), "3-29"),
        ("ADDX", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x04), "3-29"),
        ("AND", "D[c],D[a],const9", F::RC, 0x8F, Some(0x08), "3-30"),
        ("AND", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x08), "3-30"),
        ("AND", "D[15],const8", F::SC, 0x16, None, "3-30"),
        ("AND", "D[a],D[b]", F::SRR, 0x26, None, "3-30"),
        ("AND.EQ", "D[c],D[a],const9", F::RC, 0x8B, Some(0x20), "3-34"),
        ("AND.EQ", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x20), "3-34"),
        ("AND.GE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x24), "3-35"),
        ("AND.GE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x24), "3-35"),
        ("AND.GE.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x25), "3-35"),
        ("AND.GE.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x25), "3-35"),
        ("AND.LT", "D[c],D[a],const9", F::RC, 0x8B, Some(0x22), "3-37"),
        ("AND.LT", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x22), "3-37"),
        ("AND.LT.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x23), "3-37"),
        ("AND.LT.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x23), "3-37"),
        ("AND.NE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x21), "3-39"),
        ("AND.NE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x21), "3-39"),
        ("ANDN", "D[c],D[a],const9", F::RC, 0x8F, Some(0x0E), "3-41"),
        ("ANDN", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x0E), "3-41"),
        ("CADD", "D[c],D[d],D[a],const9", F::RCR, 0xAB, Some(0x00), "3-59"),
        ("CADD", "D[c],D[d],D[a],D[b]", F::RRR, 0x2B, Some(0x00), "3-59"),
        ("CADD", "D[a],D[15],const4", F::SRC, 0x8A, None, "3-59"),
        ("CADDN", "D[c],D[d],D[a],const9", F::RCR, 0xAB, Some(0x01), "3-61"),
        ("CADDN", "D[c],D[d],D[a],D[b]", F::RRR, 0x2B, Some(0x01), "3-61"),
        ("CADDN", "D[a],D[15],const4", F::SRC, 0xCA, None, "3-61"),
        ("CMOV", "D[a],D[15],const4", F::SRC, 0xAA, None, "3-74"),
        ("CMOV", "D[a],D[15],D[b]", F::SRR, 0x2A, None, "3-74"),
        ("CMOVN", "D[a],D[15],const4", F::SRC, 0xEA, None, "3-75"),
        ("CMOVN", "D[a],D[15],D[b]", F::SRR, 0x6A, None, "3-75"),
        ("CSUB", "D[c],D[d],D[a],D[b]", F::RRR, 0x2B, Some(0x02), "3-76"),
        ("CSUBN", "D[c],D[d],D[a],D[b]", F::RRR, 0x2B, Some(0x03), "3-77"),
        ("EQ", "D[c],D[a],const9", F::RC, 0x8B, Some(0x10), "3-92"),
        ("EQ", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x10), "3-92"),
        ("EQ", "D[15],D[a],const4", F::SRC, 0xBA, None, "3-92"),
        ("EQ", "D[15],D[a],D[b]", F::SRR, 0x3A, None, "3-92"),
        ("EQ.A", "D[c],A[a],A[b]", F::RR, 0x01, Some(0x40), "3-94"),
        ("EQ.B", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x50), "3-95"),
        ("EQ.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x70), "3-95"),
        ("EQ.W", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x90), "3-95"),
        ("EQANY.B", "D[c],D[a],const9", F::RC, 0x8B, Some(0x56), "3-97"),
        ("EQANY.B", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x56), "3-97"),
        ("EQANY.H", "D[c],D[a],const9", F::RC, 0x8B, Some(0x76), "3-97"),
        ("EQANY.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x76), "3-97"),
        ("EQZ.A", "D[c],A[a]", F::RR, 0x01, Some(0x48), "3-99"),
        ("GE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x14), "3-106"),
        ("GE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x14), "3-106"),
        ("GE.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x15), "3-106"),
        ("GE.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x15), "3-106"),
        ("GE.A", "D[c],A[a],A[b]", F::RR, 0x01, Some(0x43), "3-108"),
        ("LT", "D[c],D[a],const9", F::RC, 0x8B, Some(0x12), "3-174"),
        ("LT", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x12), "3-174"),
        ("LT", "D[15],D[a],const4", F::SRC, 0xFA, None, "3-174"),
        ("LT", "D[15],D[a],D[b]", F::SRR, 0x7A, None, "3-174"),
        ("LT.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x13), "3-175"),
        ("LT.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x13), "3-175"),
        ("LT.A", "D[c],A[a],A[b]", F::RR, 0x01, Some(0x42), "3-176"),
        ("LT.B", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x52), "3-177"),
        ("LT.BU", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x53), "3-177"),
        ("LT.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x72), "3-178"),
        ("LT.HU", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x73), "3-178"),
        ("LT.W", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x92), "3-179"),
        ("LT.WU", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x93), "3-179"),
        ("MAX", "D[c],D[a],const9", F::RC, 0x8B, Some(0x1A), "3-215"),
        ("MAX", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x1A), "3-215"),
        ("MAX.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x1B), "3-215"),
        ("MAX.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x1B), "3-215"),
        ("MAX.B", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x5A), "3-217"),
        ("MAX.BU", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x5B), "3-217"),
        ("MAX.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x7A), "3-218"),
        ("MAX.HU", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x7B), "3-218"),
        ("MIN", "D[c],D[a],const9", F::RC, 0x8B, Some(0x18), "3-220"),
        ("MIN", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x18), "3-220"),
        ("MIN.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x19), "3-220"),
        ("MIN.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x19), "3-220"),
        ("MIN.B", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x58), "3-222"),
        ("MIN.BU", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x59), "3-222"),
        ("MIN.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x78), "3-223"),
        ("MIN.HU", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x79), "3-223"),
        ("MOV", "D[c],const16", F::RLC, 0x3B, None, "3-224"),
        ("MOV", "E[c],const16", F::RLC, 0xFB, None, "3-224"),
        ("MOV", "D[c],D[b]", F::RR, 0x0B, Some(0x1F), "3-224"),
        ("MOV", "E[c],D[b]", F::RR, 0x0B, Some(0x80), "3-224"),
        ("MOV", "E[c],D[a],D[b]", F::RR, 0x0B, Some(0x81), "3-224"),
        ("MOV", "D[15],const8", F::SC, 0xDA, None, "3-224"),
        ("MOV", "D[a],const4", F::SRC, 0x82, None, "3-225"),
        ("MOV", "E[a],const4", F::SRC, 0xD2, None, "3-225"),
        ("MOV", "D[a],D[b]", F::SRR, 0x02, None, "3-225"),
        ("MOV.A", "A[c],D[b]", F::RR, 0x01, Some(0x63), "3-227"),
        ("MOV.A", "A[a],const4", F::SRC, 0xA0, None, "3-227"),
        ("MOV.A", "A[a],D[b]", F::SRR, 0x60, None, "3-227"),
        ("MOV.AA", "A[c],A[b]", F::RR, 0x01, Some(0x00), "3-228"),
        ("MOV.AA", "A[a],A[b]", F::SRR, 0x40, None, "3-228"),
        ("MOV.D", "D[c],A[b]", F::RR, 0x01, Some(0x4C), "3-229"),
        ("MOV.D", "D[a],A[b]", F::SRR, 0x80, None, "3-229"),
        ("MOV.U", "D[c],const16", F::RLC, 0xBB, None, "3-230"),
        ("MOVH", "D[c],const16", F::RLC, 0x7B, None, "3-231"),
        ("MOVH.A", "A[c],const16", F::RLC, 0x91, None, "3-232"),
        ("NAND", "D[c],D[a],const9", F::RC, 0x8F, Some(0x09), "3-285"),
        ("NAND", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x09), "3-285"),
        ("NE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x11), "3-287"),
        ("NE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x11), "3-287"),
        ("NE.A", "D[c],A[a],A[b]", F::RR, 0x01, Some(0x41), "3-288"),
        ("NEZ.A", "D[c],A[a]", F::RR, 0x01, Some(0x49), "3-289"),
        ("NOR", "D[c],D[a],const9", F::RC, 0x8F, Some(0x0B), "3-291"),
        ("NOR", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x0B), "3-291"),
        ("NOT", "D[a]", F::SR, 0x46, Some(0x00), "3-293"),
        ("OR", "D[c],D[a],const9", F::RC, 0x8F, Some(0x0A), "3-294"),
        ("OR", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x0A), "3-294"),
        ("OR", "D[15],const8", F::SC, 0x96, None, "3-294"),
        ("OR", "D[a],D[b]", F::SRR, 0xA6, None, "3-294"),
        ("OR.EQ", "D[c],D[a],const9", F::RC, 0x8B, Some(0x27), "3-298"),
        ("OR.EQ", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x27), "3-298"),
        ("OR.GE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x2B), "3-299"),
        ("OR.GE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x2B), "3-299"),
        ("OR.GE.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x2C), "3-299"),
        ("OR.GE.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x2C), "3-299"),
        ("OR.LT", "D[c],D[a],const9", F::RC, 0x8B, Some(0x29), "3-301"),
        ("OR.LT", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x29), "3-301"),
        ("OR.LT.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x2A), "3-301"),
        ("OR.LT.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x2A), "3-301"),
        ("OR.NE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x28), "3-303"),
        ("OR.NE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x28), "3-303"),
        ("ORN", "D[c],D[a],const9", F::RC, 0x8F, Some(0x0F), "3-305"),
        ("ORN", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x0F), "3-305"),
        ("RSTV", "", F::SYS, 0x2F, Some(0x00), "3-317"),
        ("RSUB", "D[c],D[a],const9", F::RC, 0x8B, Some(0x08), "3-318"),
        ("RSUB", "D[a]", F::SR, 0x32, Some(0x05), "3-318"),
        ("RSUBS", "D[c],D[a],const9", F::RC, 0x8B, Some(0x0A), "3-319"),
        ("RSUBS.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x0B), "3-319"),
        ("SAT.B", "D[c],D[a]", F::RR, 0x0B, Some(0x5E), "3-320"),
        ("SAT.B", "D[a]", F::SR, 0x32, Some(0x00), "3-320"),
        ("SAT.BU", "D[c],D[a]", F::RR, 0x0B, Some(0x5F), "3-321"),
        ("SAT.BU", "D[a]", F::SR, 0x32, Some(0x01), "3-321"),
        ("SAT.H", "D[c],D[a]", F::RR, 0x0B, Some(0x7E), "3-322"),
        ("SAT.H", "D[a]", F::SR, 0x32, Some(0x02), "3-322"),
        ("SAT.HU", "D[c],D[a]", F::RR, 0x0B, Some(0x7F), "3-323"),
        ("SAT.HU", "D[a]", F::SR, 0x32, Some(0x03), "3-323"),
        ("SEL", "D[c],D[d],D[a],const9", F::RCR, 0xAB, Some(0x04), "3-324"),
        ("SEL", "D[c],D[d],D[a],D[b]", F::RRR, 0x2B, Some(0x04), "3-324"),
        ("SELN", "D[c],D[d],D[a],const9", F::RCR, 0xAB, Some(0x05), "3-325"),
        ("SELN", "D[c],D[d],D[a],D[b]", F::RRR, 0x2B, Some(0x05), "3-325"),
        ("SH", "D[c],D[a],const9", F::RC, 0x8F, Some(0x00), "3-326"),
        ("SH", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x00), "3-326"),
        ("SH", "D[a],const4", F::SRC, 0x06, None, "3-326"),
        ("SH.EQ", "D[c],D[a],const9", F::RC, 0x8B, Some(0x37), "3-328"),
        ("SH.EQ", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x37), "3-328"),
        ("SH.GE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x3B), "3-329"),
        ("SH.GE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x3B), "3-329"),
        ("SH.GE.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x3C), "3-329"),
        ("SH.GE.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x3C), "3-329"),
        ("SH.H", "D[c],D[a],const9", F::RC, 0x8F, Some(0x40), "3-331"),
        ("SH.H", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x40), "3-331"),
        ("SH.LT", "D[c],D[a],const9", F::RC, 0x8B, Some(0x39), "3-332"),
        ("SH.LT", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x39), "3-332"),
        ("SH.LT.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x3A), "3-332"),
        ("SH.LT.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x3A), "3-332"),
        ("SH.NE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x38), "3-334"),
        ("SH.NE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x38), "3-334"),
        ("SHA", "D[c],D[a],const9", F::RC, 0x8F, Some(0x01), "3-337"),
        ("SHA", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x01), "3-337"),
        ("SHA", "D[a],const4", F::SRC, 0x86, None, "3-338"),
        ("SHA.H", "D[c],D[a],const9", F::RC, 0x8F, Some(0x41), "3-339"),
        ("SHA.H", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x41), "3-339"),
        ("SHAS", "D[c],D[a],const9", F::RC, 0x8F, Some(0x02), "3-341"),
        ("SHAS", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x02), "3-341"),
        ("SUB", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x08), "3-364"),
        ("SUB", "D[a],D[b]", F::SRR, 0xA2, None, "3-364"),
        ("SUB", "D[a],D[15],D[b]", F::SRR, 0x52, None, "3-364"),
        ("SUB", "D[15],D[a],D[b]", F::SRR, 0x5A, None, "3-364"),
        ("SUB.A", "A[c],A[a],A[b]", F::RR, 0x01, Some(0x02), "3-366"),
        ("SUB.A", "A[10],const8", F::SC, 0x20, None, "3-366"),
        ("SUB.B", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x48), "3-367"),
        ("SUB.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x68), "3-367"),
        ("SUBC", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x0D), "3-369"),
        ("SUBS", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x0A), "3-370"),
        ("SUBS", "D[a],D[b]", F::SRR, 0x62, None, "3-370"),
        ("SUBS.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x0B), "3-370"),
        ("SUBS.H", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x6A), "3-372"),
        ("SUBS.HU", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x6B), "3-372"),
        ("SUBX", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x0C), "3-374"),
        ("XNOR", "D[c],D[a],const9", F::RC, 0x8F, Some(0x0D), "3-383"),
        ("XNOR", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x0D), "3-383"),
        ("XOR", "D[c],D[a],const9", F::RC, 0x8F, Some(0x0C), "3-385"),
        ("XOR", "D[c],D[a],D[b]", F::RR, 0x0F, Some(0x0C), "3-385"),
        ("XOR", "D[a],D[b]", F::SRR, 0xC6, None, "3-385"),
        ("XOR.EQ", "D[c],D[a],const9", F::RC, 0x8B, Some(0x2F), "3-386"),
        ("XOR.EQ", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x2F), "3-386"),
        ("XOR.GE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x33), "3-387"),
        ("XOR.GE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x33), "3-387"),
        ("XOR.GE.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x34), "3-387"),
        ("XOR.GE.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x34), "3-387"),
        ("XOR.LT", "D[c],D[a],const9", F::RC, 0x8B, Some(0x31), "3-389"),
        ("XOR.LT", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x31), "3-389"),
        ("XOR.LT.U", "D[c],D[a],const9", F::RC, 0x8B, Some(0x32), "3-389"),
        ("XOR.LT.U", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x32), "3-389"),
        ("XOR.NE", "D[c],D[a],const9", F::RC, 0x8B, Some(0x30), "3-391"),
        ("XOR.NE", "D[c],D[a],D[b]", F::RR, 0x0B, Some(0x30), "3-391"),
    ];

    /// Encodings that V131-2 does not have (the TC1.6 `MOV E[..]` forms).
    fn tc16_only(e: &Enc) -> bool {
        e.0 == "MOV" && e.1.contains("E[")
    }

    // ---- encoders: V2 Table 1-5 / 1-6 field layouts ------------------------------------------------

    fn w32(w: u32) -> Vec<u8> {
        w.to_le_bytes().to_vec()
    }
    fn w16(h: u32) -> Vec<u8> {
        (h as u16).to_le_bytes().to_vec()
    }
    fn rr(op1: u32, op2: u32, c: u32, a: u32, b: u32) -> Vec<u8> {
        rrn(op1, op2, c, a, b, 0)
    }
    fn rrn(op1: u32, op2: u32, c: u32, a: u32, b: u32, n: u32) -> Vec<u8> {
        w32(c << 28 | op2 << 20 | n << 16 | b << 12 | a << 8 | op1)
    }
    fn rc(op1: u32, op2: u32, c: u32, a: u32, k9: i32) -> Vec<u8> {
        w32(c << 28 | op2 << 21 | (k9 as u32 & 0x1FF) << 12 | a << 8 | op1)
    }
    fn rcr(op1: u32, op2: u32, c: u32, d: u32, a: u32, k9: i32) -> Vec<u8> {
        w32(c << 28 | d << 24 | op2 << 21 | (k9 as u32 & 0x1FF) << 12 | a << 8 | op1)
    }
    fn rrr(op1: u32, op2: u32, c: u32, d: u32, a: u32, b: u32) -> Vec<u8> {
        w32(c << 28 | d << 24 | op2 << 20 | b << 12 | a << 8 | op1)
    }
    fn rlc(op1: u32, c: u32, a: u32, k16: u32) -> Vec<u8> {
        w32(c << 28 | (k16 & 0xFFFF) << 12 | a << 8 | op1)
    }
    fn sys(op1: u32, op2: u32) -> Vec<u8> {
        w32(op2 << 22 | op1)
    }
    fn src(op1: u32, a: u32, k4: i32) -> Vec<u8> {
        w16((k4 as u32 & 15) << 12 | a << 8 | op1)
    }
    fn srr(op1: u32, a: u32, b: u32) -> Vec<u8> {
        w16(b << 12 | a << 8 | op1)
    }
    fn sc(op1: u32, k8: u32) -> Vec<u8> {
        w16(k8 << 8 | op1)
    }
    fn sr(op1: u32, op2: u32, a: u32) -> Vec<u8> {
        w16(op2 << 12 | a << 8 | op1)
    }
    fn srrs(a: u32, b: u32, n: u32) -> Vec<u8> {
        w16(b << 12 | a << 8 | n << 6 | 0x10)
    }

    /// (op1, op2) of an instruction, as listed in [`ENCODINGS`].
    fn key(b: &[u8]) -> (u8, Option<u8>) {
        let op1 = b[0];
        if op1 & 1 == 0 {
            let h = u16::from_le_bytes([b[0], b[1]]);
            return match op1 {
                0x46 | 0x32 => (op1, Some((h >> 12) as u8)),
                _ if op1 & 0x3F == 0x10 => (0x10, None),
                _ => (op1, None),
            };
        }
        let w = u32::from_le_bytes(b[..4].try_into().unwrap());
        let op2 = match op1 {
            0x0B | 0x0F | 0x01 => Some(w >> 20 & 0xFF),
            0x8B | 0x8F => Some(w >> 21 & 0x7F),
            0x2B => Some(w >> 20 & 0xF),
            0xAB => Some(w >> 21 & 7),
            0x2F => Some(w >> 22 & 0x3F),
            _ => None,
        };
        (op1, op2.map(|v| v as u8))
    }

    const USB: u32 = psw::C | psw::V | psw::SV | psw::AV | psw::SAV;
    const CY: u32 = psw::C;
    const OV: u32 = psw::V | psw::SV;
    const AO: u32 = psw::AV | psw::SAV;

    fn i(v: i32) -> u32 {
        v as u32
    }

    #[derive(Default)]
    struct Vectors {
        seen: HashSet<(u8, Option<u8>)>,
    }

    impl Vectors {
        /// Execute one instruction with PSW user bits `usb` after `set`; record its encoding.
        #[track_caller]
        fn run(&mut self, insn: Vec<u8>, usb: u32, set: impl FnOnce(&mut C)) -> C {
            self.seen.insert(key(&insn));
            let c = testutil::run(&insn, 1, |c| {
                c.psw = psw::RESET | usb;
                set(c)
            });
            assert_eq!(c.pc, CODE + insn.len() as u32, "{insn:02x?} did not retire normally");
            c
        }
    }

    /// D[r] == v and PSW user bits == usb.
    #[track_caller]
    fn d(c: &C, r: usize, v: u32, usb: u32) {
        assert_eq!((c.d[r], c.psw & USB), (v, usb), "D[{r}], PSW.USB");
    }

    /// A[r] == v and PSW user bits == usb.
    #[track_caller]
    fn a(c: &C, r: usize, v: u32, usb: u32) {
        assert_eq!((c.a[r], c.psw & USB), (v, usb), "A[{r}], PSW.USB");
    }

    /// Expected values computed by hand from the RTL and Status Flags of the cited V2 pages.
    #[test]
    fn manual_vectors() {
        let mut t = Vectors::default();

        // ABS, ABS.B, ABS.H, ABSS, ABSS.H (p. 3-2..3-11): the operand is D[b]
        d(&t.run(rr(0x0B, 0x1C, 3, 0, 1), 0, |c| c.d[1] = i(-5)), 3, 5, 0);
        d(&t.run(rr(0x0B, 0x1C, 3, 0, 1), 0, |c| c.d[1] = 0x8000_0000), 3, 0x8000_0000, OV | AO);
        d(&t.run(rr(0x0B, 0x5C, 3, 0, 1), 0, |c| c.d[1] = 0x80FF_7F05), 3, 0x8001_7F05, OV | AO);
        d(&t.run(rr(0x0B, 0x5C, 3, 0, 1), 0, |c| c.d[1] = 0x40C0_0102), 3, 0x4040_0102, AO);
        d(&t.run(rr(0x0B, 0x7C, 3, 0, 1), 0, |c| c.d[1] = 0x8000_FFFF), 3, 0x8000_0001, OV | AO);
        d(&t.run(rr(0x0B, 0x1D, 3, 0, 1), 0, |c| c.d[1] = 0x8000_0000), 3, 0x7FFF_FFFF, OV | AO);
        d(&t.run(rr(0x0B, 0x1D, 3, 0, 1), USB, |c| c.d[1] = i(-7)), 3, 7, CY | psw::SV | psw::SAV);
        d(&t.run(rr(0x0B, 0x7D, 3, 0, 1), 0, |c| c.d[1] = 0x8000_FFF9), 3, 0x7FFF_0007, OV | AO);

        // ABSDIF, ABSDIF.B/.H, ABSDIFS, ABSDIFS.H (p. 3-5..3-9)
        d(&t.run(rc(0x8B, 0x0E, 3, 1, -3), 0, |c| c.d[1] = 10), 3, 13, 0);
        d(&t.run(rc(0x8B, 0x0E, 3, 1, -1), 0, |c| c.d[1] = 0x7FFF_FFFF), 3, 0x8000_0000, OV | AO);
        d(&t.run(rr(0x0B, 0x0E, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (3, 10)), 3, 7, 0);
        d(&t.run(rr(0x0B, 0x4E, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x0180_1000, 0x037F_0800)), 3, 0x02FF_0800, OV);
        d(&t.run(rr(0x0B, 0x6E, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x0005_7FFF, 0x0009_8000)), 3, 0x0004_FFFF, OV);
        d(&t.run(rc(0x8B, 0x0F, 3, 1, 1), 0, |c| c.d[1] = 0x8000_0000), 3, 0x7FFF_FFFF, OV | AO);
        d(&t.run(rr(0x0B, 0x0F, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (i(-2), 5)), 3, 7, 0);
        d(&t.run(rr(0x0B, 0x6F, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0003, 0x7FFF_0001)), 3, 0x7FFF_0002, OV);

        // ADD (p. 3-12): RC, RR and the six 16-bit forms
        d(&t.run(rc(0x8B, 0x00, 3, 1, -2), 0, |c| c.d[1] = 5), 3, 3, 0);
        d(&t.run(rc(0x8B, 0x00, 3, 1, 1), 0, |c| c.d[1] = 0x7FFF_FFFF), 3, 0x8000_0000, OV | AO);
        d(&t.run(rr(0x0B, 0x00, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x4000_0000, 0)), 3, 0x4000_0000, AO);
        d(&t.run(src(0xC2, 4, -8), 0, |c| c.d[4] = 10), 4, 2, 0);
        d(&t.run(src(0x92, 4, 7), 0, |c| c.d[15] = 1), 4, 8, 0);
        d(&t.run(src(0x9A, 4, -1), 0, |c| c.d[4] = 0), 15, 0xFFFF_FFFF, 0);
        d(&t.run(srr(0x42, 4, 5), 0, |c| (c.d[4], c.d[5]) = (1, 2)), 4, 3, 0);
        d(&t.run(srr(0x12, 4, 5), 0, |c| (c.d[15], c.d[5]) = (7, 3)), 4, 10, 0);
        d(&t.run(srr(0x1A, 4, 5), 0, |c| (c.d[4], c.d[5]) = (100, i(-1))), 15, 99, 0);

        // ADD.A (p. 3-15), ADDIH.A (p. 3-21), ADDSC.A / ADDSC.AT (p. 3-27): no flags
        a(&t.run(rr(0x01, 0x01, 4, 2, 3), USB, |c| (c.a[2], c.a[3]) = (0x1000, 0xFFFF_FFF0)), 4, 0x0FF0, USB);
        a(&t.run(src(0xB0, 3, -2), 0, |c| c.a[3] = 0x100), 3, 0xFE, 0);
        a(&t.run(srr(0x30, 3, 4), 0, |c| (c.a[3], c.a[4]) = (1, 2)), 3, 3, 0);
        a(&t.run(rlc(0x11, 3, 1, 0xD000), 0, |c| c.a[1] = 0x1234), 3, 0xD000_1234, 0);
        a(&t.run(rrn(0x01, 0x60, 4, 3, 2, 2), 0, |c| (c.a[2], c.d[3]) = (0x1000, 5)), 4, 0x1014, 0);
        a(&t.run(srrs(4, 2, 3), 0, |c| (c.a[2], c.d[15]) = (0x1000, 2)), 4, 0x1010, 0);
        a(&t.run(rr(0x01, 0x62, 4, 3, 2), 0, |c| (c.a[2], c.d[3]) = (0x1001, i(-9))), 4, 0x0FFC, 0);

        // ADD.B / ADD.H (p. 3-16)
        d(&t.run(rr(0x0B, 0x40, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x7FFF_8001, 0x0101_FF01)), 3, 0x8000_7F02, OV | AO);
        d(&t.run(rr(0x0B, 0x60, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x7FFF_0001, 0x0001_0002)), 3, 0x8000_0003, OV | AO);

        // ADDC (p. 3-18), ADDX (p. 3-29): carry out to PSW.C
        d(&t.run(rc(0x8B, 0x05, 3, 1, 0), CY, |c| c.d[1] = 0xFFFF_FFFF), 3, 0, CY);
        d(&t.run(rr(0x0B, 0x05, 3, 1, 2), CY, |c| (c.d[1], c.d[2]) = (0x7FFF_FFFF, 0)), 3, 0x8000_0000, OV | AO);
        d(&t.run(rc(0x8B, 0x04, 3, 1, 1), 0, |c| c.d[1] = 0xFFFF_FFFF), 3, 0, CY);
        d(&t.run(rr(0x0B, 0x04, 3, 1, 2), CY, |c| (c.d[1], c.d[2]) = (0x8000_0000, 0x8000_0000)), 3, 0, CY | OV);

        // ADDI (p. 3-19), ADDIH (p. 3-20)
        d(&t.run(rlc(0x1B, 3, 1, 0x8000), 0, |c| c.d[1] = 5), 3, 0xFFFF_8005, 0);
        d(&t.run(rlc(0x9B, 3, 1, 1), 0, |c| c.d[1] = 0x7FFF_0000), 3, 0x8000_0000, OV | AO);

        // ADDS (p. 3-22), ADDS.H/.HU (p. 3-24), ADDS.U (p. 3-26)
        d(&t.run(rc(0x8B, 0x02, 3, 1, 0xFF), 0, |c| c.d[1] = 0x7FFF_FFF0), 3, 0x7FFF_FFFF, OV | AO);
        d(&t.run(rr(0x0B, 0x02, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, i(-1))), 3, 0x8000_0000, OV | AO);
        d(&t.run(srr(0x22, 1, 2), 0, |c| (c.d[1], c.d[2]) = (1, 2)), 1, 3, 0);
        d(&t.run(rr(0x0B, 0x62, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x7FFF_8000, 0x0001_FFFF)), 3, 0x7FFF_8000, OV | AO);
        d(&t.run(rr(0x0B, 0x63, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xFFFF_4000, 0x0001_0001)), 3, 0xFFFF_4001, OV | AO);
        d(&t.run(rc(0x8B, 0x03, 3, 1, -1), 0, |c| c.d[1] = 5), 3, 0xFFFF_FFFF, OV);
        d(&t.run(rr(0x0B, 0x03, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, 0x4000_0000)), 3, 0xC000_0000, 0);

        // AND (p. 3-30), ANDN (p. 3-41), NAND (p. 3-285), NOR (p. 3-291), NOT (p. 3-293), OR (p. 3-294),
        // ORN (p. 3-305), XNOR (p. 3-383), XOR (p. 3-385): const9/const8 zero-extended, no flags
        d(&t.run(rc(0x8F, 0x08, 3, 1, 0x1FF), USB, |c| c.d[1] = 0xFFFF_FFFF), 3, 0x1FF, USB);
        d(&t.run(rr(0x0F, 0x08, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xF0F0, 0xFF00)), 3, 0xF000, 0);
        d(&t.run(sc(0x16, 0xF0), 0, |c| c.d[15] = 0x1234), 15, 0x30, 0);
        d(&t.run(srr(0x26, 1, 2), 0, |c| (c.d[1], c.d[2]) = (6, 3)), 1, 2, 0);
        d(&t.run(rc(0x8F, 0x0E, 3, 1, 0x0F0), 0, |c| c.d[1] = 0xFFFF), 3, 0xFF0F, 0);
        d(&t.run(rr(0x0F, 0x0E, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xFF, 0xF)), 3, 0xF0, 0);
        d(&t.run(rc(0x8F, 0x09, 3, 1, 0x0F), 0, |c| c.d[1] = 0xFF), 3, 0xFFFF_FFF0, 0);
        d(&t.run(rr(0x0F, 0x09, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xFFFF_FFFF, 0xFFFF_FFFF)), 3, 0, 0);
        d(&t.run(rc(0x8F, 0x0B, 3, 1, 0x100), 0, |c| c.d[1] = 0), 3, 0xFFFF_FEFF, 0);
        d(&t.run(rr(0x0F, 0x0B, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xF0, 0x0F)), 3, 0xFFFF_FF00, 0);
        d(&t.run(sr(0x46, 0, 1), USB, |c| c.d[1] = 0x0F0F_0F0F), 1, 0xF0F0_F0F0, USB);
        d(&t.run(rc(0x8F, 0x0A, 3, 1, 0x1FF), 0, |c| c.d[1] = 0x1000), 3, 0x11FF, 0);
        d(&t.run(rr(0x0F, 0x0A, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xF0, 0x0F)), 3, 0xFF, 0);
        d(&t.run(sc(0x96, 0x80), 0, |c| c.d[15] = 0x100), 15, 0x180, 0);
        d(&t.run(srr(0xA6, 1, 2), 0, |c| (c.d[1], c.d[2]) = (1, 2)), 1, 3, 0);
        d(&t.run(rc(0x8F, 0x0F, 3, 1, 0x1FF), 0, |c| c.d[1] = 0), 3, 0xFFFF_FE00, 0);
        d(&t.run(rr(0x0F, 0x0F, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (1, 0xFFFF_FFFE)), 3, 1, 0);
        d(&t.run(rc(0x8F, 0x0D, 3, 1, 0x0F), 0, |c| c.d[1] = 0xFF), 3, 0xFFFF_FF0F, 0);
        d(&t.run(rr(0x0F, 0x0D, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x1234, 0x1234)), 3, 0xFFFF_FFFF, 0);
        d(&t.run(rc(0x8F, 0x0C, 3, 1, 0x1FF), 0, |c| c.d[1] = 0xFF), 3, 0x100, 0);
        d(&t.run(rr(0x0F, 0x0C, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xF0, 0xFF)), 3, 0x0F, 0);
        d(&t.run(srr(0xC6, 1, 2), 0, |c| (c.d[1], c.d[2]) = (3, 5)), 1, 6, 0);

        // Accumulating compares: AND.xx (p. 3-34..3-39), OR.xx (p. 3-298..3-303), XOR.xx (p. 3-386..3-391),
        // SH.xx (p. 3-328..3-334). Unsigned forms zero-extend const9: 0x1FF is 511, not -1.
        d(&t.run(rc(0x8B, 0x20, 3, 1, -5), 0, |c| (c.d[3], c.d[1]) = (0xFFFF_FFFF, i(-5))), 3, 0xFFFF_FFFF, 0);
        d(&t.run(rc(0x8B, 0x20, 3, 1, -4), 0, |c| (c.d[3], c.d[1]) = (0xFFFF_FFFF, i(-5))), 3, 0xFFFF_FFFE, 0);
        d(&t.run(rr(0x0B, 0x20, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0x10, 1, 1)), 3, 0x10, 0);
        d(&t.run(rc(0x8B, 0x24, 3, 1, -2), 0, |c| (c.d[3], c.d[1]) = (3, i(-1))), 3, 3, 0);
        d(&t.run(rr(0x0B, 0x24, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (1, i(-1), 0)), 3, 0, 0);
        d(&t.run(rc(0x8B, 0x25, 3, 1, 0x1FF), 0, |c| (c.d[3], c.d[1]) = (1, 0x1FF)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x25, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (1, 0xFFFF_FFFF, 1)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x22, 3, 1, -2), 0, |c| (c.d[3], c.d[1]) = (1, i(-3))), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x22, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (1, 0x8000_0000, 0)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x23, 3, 1, 0x1FF), 0, |c| (c.d[3], c.d[1]) = (1, 0x200)), 3, 0, 0);
        d(&t.run(rr(0x0B, 0x23, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (1, 1, 0x8000_0000)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x21, 3, 1, 5), 0, |c| (c.d[3], c.d[1]) = (7, 5)), 3, 6, 0);
        d(&t.run(rr(0x0B, 0x21, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (7, 5, 6)), 3, 7, 0);
        d(&t.run(rc(0x8B, 0x27, 3, 1, -1), 0, |c| (c.d[3], c.d[1]) = (0x10, i(-1))), 3, 0x11, 0);
        d(&t.run(rr(0x0B, 0x27, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0x10, 1, 2)), 3, 0x10, 0);
        d(&t.run(rc(0x8B, 0x2B, 3, 1, -2), 0, |c| (c.d[3], c.d[1]) = (0, i(-1))), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x2B, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0, i(-1), 0)), 3, 0, 0);
        d(&t.run(rc(0x8B, 0x2C, 3, 1, 0x1FF), 0, |c| (c.d[3], c.d[1]) = (0, 0x1FF)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x2C, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0, 0xFFFF_FFFF, 0)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x29, 3, 1, -2), 0, |c| (c.d[3], c.d[1]) = (0, i(-3))), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x29, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (2, 0, 0)), 3, 2, 0);
        d(&t.run(rc(0x8B, 0x2A, 3, 1, 0x1FF), 0, |c| (c.d[3], c.d[1]) = (0, 0x200)), 3, 0, 0);
        d(&t.run(rr(0x0B, 0x2A, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0, 1, 0xFFFF_FFFF)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x28, 3, 1, 4), 0, |c| (c.d[3], c.d[1]) = (0, 3)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x28, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0, 3, 3)), 3, 0, 0);
        d(&t.run(rc(0x8B, 0x2F, 3, 1, -1), 0, |c| (c.d[3], c.d[1]) = (1, i(-1))), 3, 0, 0);
        d(&t.run(rr(0x0B, 0x2F, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (1, 1, 2)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x33, 3, 1, -1), 0, |c| (c.d[3], c.d[1]) = (1, 0)), 3, 0, 0);
        d(&t.run(rr(0x0B, 0x33, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0, i(-1), 0)), 3, 0, 0);
        d(&t.run(rc(0x8B, 0x34, 3, 1, 0x1FF), 0, |c| (c.d[3], c.d[1]) = (0, 0x1FF)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x34, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0, 0xFFFF_FFFF, 0)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x31, 3, 1, -2), 0, |c| (c.d[3], c.d[1]) = (1, i(-3))), 3, 0, 0);
        d(&t.run(rr(0x0B, 0x31, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0, 0x8000_0000, 0)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x32, 3, 1, 0x1FF), 0, |c| (c.d[3], c.d[1]) = (1, 0x200)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x32, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (1, 0, 1)), 3, 0, 0);
        d(&t.run(rc(0x8B, 0x30, 3, 1, 4), 0, |c| (c.d[3], c.d[1]) = (0, 3)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x30, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0x10, 3, 3)), 3, 0x10, 0);
        d(&t.run(rc(0x8B, 0x37, 3, 1, -2), 0, |c| (c.d[3], c.d[1]) = (0x8000_0001, i(-2))), 3, 3, 0);
        d(&t.run(rr(0x0B, 0x37, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (1, 1, 2)), 3, 2, 0);
        d(&t.run(rc(0x8B, 0x3B, 3, 1, -1), 0, |c| (c.d[3], c.d[1]) = (0, 0)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x3B, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (1, i(-1), 0)), 3, 2, 0);
        d(&t.run(rc(0x8B, 0x3C, 3, 1, 0x1FF), 0, |c| (c.d[3], c.d[1]) = (0, 0x1FF)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x3C, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0, 0xFFFF_FFFF, 1)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x39, 3, 1, -2), 0, |c| (c.d[3], c.d[1]) = (0, i(-3))), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x39, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0xFFFF_FFFF, 0, 0)), 3, 0xFFFF_FFFE, 0);
        d(&t.run(rc(0x8B, 0x3A, 3, 1, 0x1FF), 0, |c| (c.d[3], c.d[1]) = (0, 0x200)), 3, 0, 0);
        d(&t.run(rr(0x0B, 0x3A, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0, 1, 0x8000_0000)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x38, 3, 1, 4), 0, |c| (c.d[3], c.d[1]) = (0, 3)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x38, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0x4000_0000, 3, 3)), 3, 0x8000_0000, 0);

        // CADD (p. 3-59), CADDN (p. 3-61), CSUB (p. 3-76), CSUBN (p. 3-77): flags only when executed
        d(&t.run(rcr(0xAB, 0, 3, 4, 1, -1), 0, |c| (c.d[4], c.d[1]) = (1, 10)), 3, 9, 0);
        d(&t.run(rcr(0xAB, 0, 3, 4, 1, -1), USB, |c| (c.d[4], c.d[1]) = (0, 10)), 3, 10, USB);
        d(&t.run(rrr(0x2B, 0, 3, 4, 1, 2), 0, |c| (c.d[4], c.d[1], c.d[2]) = (2, 0x7FFF_FFFF, 1)), 3, 0x8000_0000, OV | AO);
        d(&t.run(src(0x8A, 1, 3), 0, |c| (c.d[15], c.d[1]) = (5, 1)), 1, 4, 0);
        d(&t.run(rcr(0xAB, 1, 3, 4, 1, 5), 0, |c| (c.d[4], c.d[1]) = (0, 10)), 3, 15, 0);
        d(&t.run(rrr(0x2B, 1, 3, 4, 1, 2), OV, |c| (c.d[4], c.d[1], c.d[2]) = (1, 10, 5)), 3, 10, OV);
        d(&t.run(src(0xCA, 1, -1), 0, |c| (c.d[15], c.d[1]) = (0, 1)), 1, 0, 0);
        d(&t.run(rrr(0x2B, 2, 3, 4, 1, 2), 0, |c| (c.d[4], c.d[1], c.d[2]) = (1, 0x8000_0000, 1)), 3, 0x7FFF_FFFF, OV | AO);
        d(&t.run(rrr(0x2B, 3, 3, 4, 1, 2), 0, |c| (c.d[4], c.d[1], c.d[2]) = (0, 5, 7)), 3, 0xFFFF_FFFE, 0);

        // CMOV (p. 3-74), CMOVN (p. 3-75), SEL (p. 3-324), SELN (p. 3-325)
        d(&t.run(src(0xAA, 1, -3), 0, |c| (c.d[15], c.d[1]) = (1, 9)), 1, 0xFFFF_FFFD, 0);
        d(&t.run(srr(0x2A, 1, 2), 0, |c| (c.d[15], c.d[1], c.d[2]) = (0, 9, 5)), 1, 9, 0);
        d(&t.run(src(0xEA, 1, 7), 0, |c| (c.d[15], c.d[1]) = (0, 9)), 1, 7, 0);
        d(&t.run(srr(0x6A, 1, 2), 0, |c| (c.d[15], c.d[1], c.d[2]) = (0, 9, 5)), 1, 5, 0);
        d(&t.run(rcr(0xAB, 4, 3, 4, 1, -7), 0, |c| (c.d[4], c.d[1]) = (0, 1)), 3, 0xFFFF_FFF9, 0);
        d(&t.run(rrr(0x2B, 4, 3, 4, 1, 2), 0, |c| (c.d[4], c.d[1], c.d[2]) = (1, 0x11, 0x22)), 3, 0x11, 0);
        d(&t.run(rcr(0xAB, 5, 3, 4, 1, -7), 0, |c| (c.d[4], c.d[1]) = (0, 0x11)), 3, 0x11, 0);
        d(&t.run(rrr(0x2B, 5, 3, 4, 1, 2), 0, |c| (c.d[4], c.d[1], c.d[2]) = (1, 0x11, 0x22)), 3, 0x22, 0);

        // EQ (p. 3-92), NE (p. 3-287), GE/GE.U (p. 3-106), LT/LT.U (p. 3-174/3-175)
        d(&t.run(rc(0x8B, 0x10, 3, 1, -1), 0, |c| c.d[1] = i(-1)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x10, 3, 1, 2), 0, |c| (c.d[3], c.d[1], c.d[2]) = (0xFFFF, 1, 2)), 3, 0, 0);
        d(&t.run(src(0xBA, 1, -8), 0, |c| c.d[1] = i(-8)), 15, 1, 0);
        d(&t.run(srr(0x3A, 1, 2), 0, |c| (c.d[1], c.d[2]) = (5, 5)), 15, 1, 0);
        d(&t.run(rc(0x8B, 0x11, 3, 1, 0x1FF), 0, |c| c.d[1] = 0x1FF), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x11, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (7, 7)), 3, 0, 0);
        d(&t.run(rc(0x8B, 0x14, 3, 1, -1), 0, |c| c.d[1] = i(-1)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x14, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (i(-1), 0)), 3, 0, 0);
        d(&t.run(rc(0x8B, 0x15, 3, 1, 0x1FF), 0, |c| c.d[1] = 0x1FF), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x15, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xFFFF_FFFF, 0)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x12, 3, 1, -255), 0, |c| c.d[1] = i(-256)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x12, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0, i(-1))), 3, 0, 0);
        d(&t.run(src(0xFA, 1, -1), 0, |c| c.d[1] = i(-2)), 15, 1, 0);
        d(&t.run(srr(0x7A, 1, 2), 0, |c| (c.d[1], c.d[2]) = (1, 2)), 15, 1, 0);
        d(&t.run(rc(0x8B, 0x13, 3, 1, 0x1FF), 0, |c| c.d[1] = 0x200), 3, 0, 0);
        d(&t.run(rr(0x0B, 0x13, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0, 0xFFFF_FFFF)), 3, 1, 0);

        // EQ.A (p. 3-94), NE.A (p. 3-288), LT.A (p. 3-176), GE.A (p. 3-108), EQZ.A (p. 3-99),
        // NEZ.A (p. 3-289): address comparisons are unsigned
        d(&t.run(rr(0x01, 0x40, 3, 1, 2), 0, |c| (c.a[1], c.a[2]) = (0x1234, 0x1234)), 3, 1, 0);
        d(&t.run(rr(0x01, 0x41, 3, 1, 2), 0, |c| (c.a[1], c.a[2]) = (1, 2)), 3, 1, 0);
        d(&t.run(rr(0x01, 0x42, 3, 1, 2), 0, |c| (c.a[1], c.a[2]) = (1, 0x8000_0000)), 3, 1, 0);
        d(&t.run(rr(0x01, 0x43, 3, 1, 2), 0, |c| (c.a[1], c.a[2]) = (0x8000_0000, 1)), 3, 1, 0);
        d(&t.run(rr(0x01, 0x48, 3, 1, 0), 0, |c| c.a[1] = 0), 3, 1, 0);
        d(&t.run(rr(0x01, 0x49, 3, 1, 0), 0, |c| (c.a[1], c.d[3]) = (0, 5)), 3, 0, 0);

        // EQ.B/.H/.W (p. 3-95), EQANY.B/.H (p. 3-97), LT.B/.BU (p. 3-177), LT.H/.HU (p. 3-178),
        // LT.W/.WU (p. 3-179)
        d(&t.run(rr(0x0B, 0x50, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x1122_3344, 0x1100_3300)), 3, 0xFF00_FF00, 0);
        d(&t.run(rr(0x0B, 0x70, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x1111_2222, 0x1111_2223)), 3, 0xFFFF_0000, 0);
        d(&t.run(rr(0x0B, 0x90, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (5, 5)), 3, 0xFFFF_FFFF, 0);
        d(&t.run(rc(0x8B, 0x56, 3, 1, 0xFF), 0, |c| c.d[1] = 0x1200_5678), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x56, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x0102_0304, 0x0506_0704)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x76, 3, 1, -2), 0, |c| c.d[1] = 0x1234_FFFE), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x76, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x1234_5678, 0x0000_5678)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x76, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x1234_5678, 0x5678_1234)), 3, 0, 0);
        d(&t.run(rr(0x0B, 0x52, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x807F_0100, 0x7F80_01FF)), 3, 0xFF00_0000, 0);
        d(&t.run(rr(0x0B, 0x53, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x807F_0100, 0x7F80_01FF)), 3, 0x00FF_00FF, 0);
        d(&t.run(rr(0x0B, 0x72, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x0001_8000, 0x0002_7FFF)), 3, 0xFFFF_FFFF, 0);
        d(&t.run(rr(0x0B, 0x73, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x0001_8000, 0x0002_7FFF)), 3, 0xFFFF_0000, 0);
        d(&t.run(rr(0x0B, 0x92, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, 0)), 3, 0xFFFF_FFFF, 0);
        d(&t.run(rr(0x0B, 0x93, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, 0)), 3, 0, 0);

        // MAX/MAX.U (p. 3-215), MAX.B/.BU (p. 3-217), MAX.H/.HU (p. 3-218), MIN* (p. 3-220..3-223)
        d(&t.run(rc(0x8B, 0x1A, 3, 1, -3), 0, |c| c.d[1] = i(-5)), 3, 0xFFFF_FFFD, 0);
        d(&t.run(rr(0x0B, 0x1A, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, 1)), 3, 1, 0);
        d(&t.run(rc(0x8B, 0x1B, 3, 1, 0x1FF), 0, |c| c.d[1] = 0x100), 3, 0x1FF, 0);
        d(&t.run(rr(0x0B, 0x1B, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, 1)), 3, 0x8000_0000, 0);
        d(&t.run(rr(0x0B, 0x5A, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x807F_05FF, 0x7F80_0600)), 3, 0x7F7F_0600, 0);
        d(&t.run(rr(0x0B, 0x5B, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x807F_05FF, 0x7F80_0600)), 3, 0x8080_06FF, 0);
        d(&t.run(rr(0x0B, 0x7A, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0001, 0x7FFF_FFFF)), 3, 0x7FFF_0001, 0);
        d(&t.run(rr(0x0B, 0x7B, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0001, 0x7FFF_FFFF)), 3, 0x8000_FFFF, 0);
        d(&t.run(rc(0x8B, 0x18, 3, 1, -3), 0, |c| c.d[1] = 5), 3, 0xFFFF_FFFD, 0);
        d(&t.run(rr(0x0B, 0x18, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, 1)), 3, 0x8000_0000, 0);
        d(&t.run(rc(0x8B, 0x19, 3, 1, 0x1FF), 0, |c| c.d[1] = 0x8000_0000), 3, 0x1FF, 0);
        d(&t.run(rr(0x0B, 0x19, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, 1)), 3, 1, 0);
        d(&t.run(rr(0x0B, 0x58, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x807F_05FF, 0x7F80_0600)), 3, 0x8080_05FF, 0);
        d(&t.run(rr(0x0B, 0x59, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x807F_05FF, 0x7F80_0600)), 3, 0x7F7F_0500, 0);
        d(&t.run(rr(0x0B, 0x78, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0001, 0x7FFF_FFFF)), 3, 0x8000_FFFF, 0);
        d(&t.run(rr(0x0B, 0x79, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0001, 0x7FFF_FFFF)), 3, 0x7FFF_0001, 0);

        // MOV (p. 3-224/3-225), MOV.U (p. 3-230), MOVH (p. 3-231)
        d(&t.run(rlc(0x3B, 3, 0, 0x8000), 0, |_| ()), 3, 0xFFFF_8000, 0);
        let c = t.run(rlc(0xFB, 2, 0, 0xFFFE), 0, |_| ());
        assert_eq!((c.d[2], c.d[3]), (0xFFFF_FFFE, 0xFFFF_FFFF));
        d(&t.run(rr(0x0B, 0x1F, 3, 0, 2), 0, |c| c.d[2] = 0x1234), 3, 0x1234, 0);
        let c = t.run(rr(0x0B, 0x80, 4, 0, 1), 0, |c| c.d[1] = 0x8000_0000);
        assert_eq!((c.d[4], c.d[5]), (0x8000_0000, 0xFFFF_FFFF));
        let c = t.run(rr(0x0B, 0x81, 4, 6, 3), 0, |c| (c.d[6], c.d[3]) = (0xAAAA, 0xBBBB));
        assert_eq!((c.d[4], c.d[5]), (0xBBBB, 0xAAAA));
        let c = t.run(rr(0x0B, 0x81, 4, 4, 5), 0, |c| (c.d[4], c.d[5]) = (1, 2)); // E[4] = {D[4], D[5]}: swap
        assert_eq!((c.d[4], c.d[5]), (2, 1));
        d(&t.run(sc(0xDA, 0xFF), 0, |_| ()), 15, 0xFF, 0);
        d(&t.run(src(0x82, 1, -1), 0, |_| ()), 1, 0xFFFF_FFFF, 0);
        let c = t.run(src(0xD2, 2, -8), 0, |_| ());
        assert_eq!((c.d[2], c.d[3]), (0xFFFF_FFF8, 0xFFFF_FFFF));
        let c = t.run(src(0xD2, 2, 7), 0, |c| c.d[3] = 5);
        assert_eq!((c.d[2], c.d[3]), (7, 0));
        d(&t.run(srr(0x02, 1, 2), 0, |c| c.d[2] = 0x55), 1, 0x55, 0);
        d(&t.run(rlc(0xBB, 3, 0, 0x8000), 0, |_| ()), 3, 0x8000, 0);
        d(&t.run(rlc(0x7B, 3, 0, 0x8001), 0, |_| ()), 3, 0x8001_0000, 0);

        // MOV.A (p. 3-227: SRC const4 zero-extended), MOV.AA (p. 3-228), MOV.D (p. 3-229), MOVH.A (p. 3-232)
        a(&t.run(rr(0x01, 0x63, 3, 0, 2), 0, |c| c.d[2] = 0x1234), 3, 0x1234, 0);
        a(&t.run(src(0xA0, 3, 0xF), 0, |_| ()), 3, 15, 0);
        a(&t.run(srr(0x60, 3, 2), 0, |c| c.d[2] = 0xD000_0000), 3, 0xD000_0000, 0);
        a(&t.run(rr(0x01, 0x00, 3, 0, 2), 0, |c| c.a[2] = 0xA000_0000), 3, 0xA000_0000, 0);
        a(&t.run(srr(0x40, 3, 2), 0, |c| c.a[2] = 0xA000_0004), 3, 0xA000_0004, 0);
        d(&t.run(rr(0x01, 0x4C, 3, 0, 2), 0, |c| c.a[2] = 0x1234_5678), 3, 0x1234_5678, 0);
        d(&t.run(srr(0x80, 3, 2), 0, |c| c.a[2] = 0x8765_4321), 3, 0x8765_4321, 0);
        a(&t.run(rlc(0x91, 3, 0, 0x8001), 0, |_| ()), 3, 0x8001_0000, 0);

        // RSTV (p. 3-317)
        let c = t.run(sys(0x2F, 0), USB, |_| ());
        assert_eq!(c.psw & USB, CY);

        // RSUB (p. 3-318), RSUBS/RSUBS.U (p. 3-319)
        d(&t.run(rc(0x8B, 0x08, 3, 1, 3), 0, |c| c.d[1] = 5), 3, 0xFFFF_FFFE, 0);
        d(&t.run(rc(0x8B, 0x08, 3, 1, 0), 0, |c| c.d[1] = 0x8000_0000), 3, 0x8000_0000, OV | AO);
        d(&t.run(sr(0x32, 5, 1), 0, |c| c.d[1] = 7), 1, 0xFFFF_FFF9, 0);
        d(&t.run(rc(0x8B, 0x0A, 3, 1, 0), 0, |c| c.d[1] = 0x8000_0000), 3, 0x7FFF_FFFF, OV | AO);
        d(&t.run(rc(0x8B, 0x0B, 3, 1, 3), 0, |c| c.d[1] = 5), 3, 0, OV);
        d(&t.run(rc(0x8B, 0x0B, 3, 1, -1), 0, |c| c.d[1] = 5), 3, 0xFFFF_FFFA, 0);

        // SAT.B (p. 3-320), SAT.BU (p. 3-321), SAT.H (p. 3-322), SAT.HU (p. 3-323): no flags
        d(&t.run(rr(0x0B, 0x5E, 3, 1, 0), USB, |c| c.d[1] = i(-200)), 3, 0xFFFF_FF80, USB);
        d(&t.run(sr(0x32, 0, 1), 0, |c| c.d[1] = 300), 1, 0x7F, 0);
        d(&t.run(rr(0x0B, 0x5F, 3, 1, 0), 0, |c| c.d[1] = 0xFFFF_FFFF), 3, 0xFF, 0);
        d(&t.run(sr(0x32, 1, 1), 0, |c| c.d[1] = 0x80), 1, 0x80, 0);
        d(&t.run(rr(0x0B, 0x7E, 3, 1, 0), 0, |c| c.d[1] = 0x12345), 3, 0x7FFF, 0);
        d(&t.run(sr(0x32, 2, 1), 0, |c| c.d[1] = i(-40000)), 1, 0xFFFF_8000, 0);
        d(&t.run(rr(0x0B, 0x7F, 3, 1, 0), 0, |c| c.d[1] = 0x8000_0000), 3, 0xFFFF, 0);
        d(&t.run(sr(0x32, 3, 1), 0, |c| c.d[1] = 0x1234), 1, 0x1234, 0);

        // SH (p. 3-326): 6-bit count, -32 clears; SRC count is sign_ext(const4); no flags
        d(&t.run(rc(0x8F, 0x00, 3, 1, 0x1E0), USB, |c| c.d[1] = 0x8000_0001), 3, 0, USB);
        d(&t.run(rc(0x8F, 0x00, 3, 1, 0x01F), 0, |c| c.d[1] = 1), 3, 0x8000_0000, 0);
        d(&t.run(rc(0x8F, 0x00, 3, 1, 0x1C1), 0, |c| c.d[1] = 1), 3, 2, 0);
        d(&t.run(rc(0x8F, 0x00, 3, 1, 0x03F), 0, |c| c.d[1] = 0x8000_0000), 3, 0x4000_0000, 0);
        d(&t.run(rr(0x0F, 0x00, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, i(-4))), 3, 0x0800_0000, 0);
        d(&t.run(src(0x06, 1, -8), 0, |c| c.d[1] = 0x8000_0000), 1, 0x0080_0000, 0);
        d(&t.run(src(0x06, 1, 7), 0, |c| c.d[1] = 1), 1, 0x80, 0);

        // SH.H (p. 3-331): 5-bit count per half-word, -16 clears
        d(&t.run(rc(0x8F, 0x40, 3, 1, 0x10), 0, |c| c.d[1] = 0x8001_0003), 3, 0, 0);
        d(&t.run(rc(0x8F, 0x40, 3, 1, 0x1F), 0, |c| c.d[1] = 0x8001_0003), 3, 0x4000_0001, 0);
        d(&t.run(rc(0x8F, 0x40, 3, 1, 1), 0, |c| c.d[1] = 0x8001_0003), 3, 0x0002_0006, 0);
        d(&t.run(rr(0x0F, 0x40, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8001_0003, 0xFFFF_FFF0)), 3, 0, 0);

        // SHA (p. 3-337/3-338): C = OR of shifted-out bits (0 on zero shifts), V on left shifts only
        d(&t.run(rc(0x8F, 0x01, 3, 1, 1), 0, |c| c.d[1] = 0x4000_0000), 3, 0x8000_0000, OV | AO);
        d(&t.run(rc(0x8F, 0x01, 3, 1, 4), 0, |c| c.d[1] = 0xF000_0001), 3, 0x10, CY | OV);
        d(&t.run(rc(0x8F, 0x01, 3, 1, 0x3F), 0, |c| c.d[1] = 0x8000_0001), 3, 0xC000_0000, CY);
        d(&t.run(rc(0x8F, 0x01, 3, 1, 0x20), 0, |c| c.d[1] = 0x8000_0000), 3, 0xFFFF_FFFF, CY);
        d(&t.run(rc(0x8F, 0x01, 3, 1, 0), CY | OV, |c| c.d[1] = 0x4000_0000), 3, 0x4000_0000, psw::SV | AO);
        d(&t.run(rr(0x0F, 0x01, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xC000_0000, 1)), 3, 0x8000_0000, CY | AO);
        d(&t.run(src(0x86, 1, -8), 0, |c| c.d[1] = 0x180), 1, 1, CY);

        // SHA.H (p. 3-339): per-half-word sign fill, no flags
        d(&t.run(rc(0x8F, 0x41, 3, 1, 0x1F), USB, |c| c.d[1] = 0x8000_4000), 3, 0xC000_2000, USB);
        d(&t.run(rc(0x8F, 0x41, 3, 1, 0x10), 0, |c| c.d[1] = 0x8000_7FFF), 3, 0xFFFF_0000, 0);
        d(&t.run(rr(0x0F, 0x41, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x1234_8765, 4)), 3, 0x2340_7650, 0);

        // SHAS (p. 3-341): saturating left shift, C untouched
        d(&t.run(rc(0x8F, 0x02, 3, 1, 1), CY, |c| c.d[1] = 0x4000_0000), 3, 0x7FFF_FFFF, CY | OV | AO);
        d(&t.run(rr(0x0F, 0x02, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0xC000_0000, 2)), 3, 0x8000_0000, OV);
        d(&t.run(rr(0x0F, 0x02, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, 0x20)), 3, 0xFFFF_FFFF, 0);

        // SUB (p. 3-364), SUB.A (p. 3-366), SUB.B/.H (p. 3-367), SUBC (p. 3-369), SUBS/SUBS.U (p. 3-370),
        // SUBS.H/.HU (p. 3-372), SUBX (p. 3-374)
        d(&t.run(rr(0x0B, 0x08, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0000, 1)), 3, 0x7FFF_FFFF, OV | AO);
        d(&t.run(srr(0xA2, 1, 2), 0, |c| (c.d[1], c.d[2]) = (5, 7)), 1, 0xFFFF_FFFE, 0);
        d(&t.run(srr(0x52, 1, 2), 0, |c| (c.d[15], c.d[2]) = (10, 3)), 1, 7, 0);
        d(&t.run(srr(0x5A, 1, 2), 0, |c| (c.d[1], c.d[2]) = (3, 10)), 15, 0xFFFF_FFF9, 0);
        a(&t.run(rr(0x01, 0x02, 3, 1, 2), 0, |c| (c.a[1], c.a[2]) = (0x1000, 0x10)), 3, 0xFF0, 0);
        a(&t.run(sc(0x20, 0xFF), 0, |c| c.a[10] = 0x1000), 10, 0xF01, 0);
        d(&t.run(rr(0x0B, 0x48, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_7F05, 0x0101_FF03)), 3, 0x7FFF_8002, OV | AO);
        d(&t.run(rr(0x0B, 0x68, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_0005, 0x0001_0003)), 3, 0x7FFF_0002, OV | AO);
        d(&t.run(rr(0x0B, 0x0D, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0, 0)), 3, 0xFFFF_FFFF, 0);
        d(&t.run(rr(0x0B, 0x0D, 3, 1, 2), CY, |c| (c.d[1], c.d[2]) = (5, 3)), 3, 2, CY);
        d(&t.run(rr(0x0B, 0x0A, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x7FFF_FFFF, i(-1))), 3, 0x7FFF_FFFF, OV | AO);
        d(&t.run(srr(0x62, 1, 2), 0, |c| (c.d[1], c.d[2]) = (5, 3)), 1, 2, 0);
        d(&t.run(rr(0x0B, 0x0B, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (3, 5)), 3, 0, OV);
        d(&t.run(rr(0x0B, 0x6A, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x8000_7FFF, 0x0001_FFFF)), 3, 0x8000_7FFF, OV | AO);
        d(&t.run(rr(0x0B, 0x6B, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (0x0001_8000, 0x0002_0001)), 3, 0x0000_7FFF, OV | AO);
        d(&t.run(rr(0x0B, 0x0C, 3, 1, 2), 0, |c| (c.d[1], c.d[2]) = (1, 1)), 3, 0, CY);
        d(&t.run(rr(0x0B, 0x0C, 3, 1, 2), CY, |c| (c.d[1], c.d[2]) = (0, 1)), 3, 0xFFFF_FFFF, 0);

        let all: HashSet<_> = ENCODINGS.iter().map(|e| (e.3, e.4)).collect();
        assert_eq!(all.len(), ENCODINGS.len(), "(op1, op2) identifies an encoding");
        let mut missing: Vec<_> = all.difference(&t.seen).collect();
        missing.sort();
        assert!(missing.is_empty(), "encodings without a manual vector: {missing:02x?}");
        assert!(t.seen.is_subset(&all));
    }

    /// TC1.3.1 has no `MOV E[..]` forms: they are illegal opcodes there (V131-2 MOV p. 3-273); the rest
    /// of the group behaves identically.
    #[test]
    fn tc131_differences() {
        for insn in [rlc(0xFB, 2, 0, 1), rr(0x0B, 0x80, 2, 0, 1), rr(0x0B, 0x81, 2, 1, 3), src(0xD2, 2, 1)] {
            let c = testutil::run_arch(&insn, 1, Arch::Tc131, |_| ());
            assert_eq!((c.pc, c.a[11]), (TRAP_BASE | 2 << 5, CODE), "{insn:02x?}: IOPC on TC1.3.1");
            assert_eq!(c.d[2], 0);
        }
        // GE.U const9 is zero-extended on TC1.3.1 too (V131-2 p. 3-132 description; see module docs)
        let c = testutil::run_arch(&rc(0x8B, 0x15, 3, 1, 0x1FF), 1, Arch::Tc131, |c| c.d[1] = 0x1FF);
        assert_eq!(c.d[3], 1);
    }

    /// MOV E[c] with an odd c raises OPD (class 2, TIN 3; V1 p. 6-7) and writes nothing.
    #[test]
    fn odd_register_pair_raises_opd() {
        for insn in [rlc(0xFB, 3, 0, 1), rr(0x0B, 0x80, 3, 0, 1), rr(0x0B, 0x81, 3, 1, 2), src(0xD2, 3, 1)] {
            let c = testutil::run(&insn, 1, |c| (c.d[3], c.d[4]) = (0x33, 0x44));
            assert_eq!((c.pc, c.a[11], c.d[15]), (TRAP_BASE | 2 << 5, CODE, 3), "{insn:02x?}: OPD, TIN in D[15]");
            assert_eq!((c.d[3], c.d[4], c.icount), (0x33, 0x44, 1));
            assert!(ends_block(&decode(u16::from_le_bytes([insn[0], insn[1]]), *insn.get(2).unwrap_or(&0) as u16 | (*insn.get(3).unwrap_or(&0) as u16) << 8, 0, Arch::Tc16).unwrap()));
        }
    }

    /// GRWP (class 1 TIN 7, V1 p. 6-7): "A program attempted to modify one of the global address
    /// registers (A[0], A[1], A[8] or A[9]) when it did not have permission to do so" (PSW.GW = 0,
    /// V1 p. 3-6). Applies to every address-register write of this group, as it does to LD.A/LEA.
    #[test]
    fn global_address_register_write_protection() {
        let writes = [
            srr(0x40, 8, 2),              // MOV.AA A8, A2
            srr(0x60, 8, 2),              // MOV.A A8, D2
            src(0xA0, 8, 3),              // MOV.A A8, const4
            srr(0x30, 8, 2),              // ADD.A A8, A2
            src(0xB0, 8, 1),              // ADD.A A8, const4
            srrs(8, 2, 1),                // ADDSC.A A8, A2, D15, 1
            rr(0x01, 0x00, 8, 0, 2),      // MOV.AA (RR)
            rr(0x01, 0x01, 8, 2, 3),      // ADD.A (RR)
            rr(0x01, 0x02, 8, 2, 3),      // SUB.A (RR)
            rr(0x01, 0x60, 8, 2, 3),      // ADDSC.A (RR)
            rr(0x01, 0x62, 8, 2, 3),      // ADDSC.AT
            rr(0x01, 0x63, 8, 0, 2),      // MOV.A (RR)
            rlc(0x91, 8, 0, 0x1234),      // MOVH.A
            rlc(0x11, 8, 2, 0x1234),      // ADDIH.A
        ];
        for insn in writes {
            let c = testutil::run(&insn, 1, |c| (c.psw, c.a[8]) = (psw::RESET & !psw::GW, 0xA8));
            assert_eq!((c.pc, c.d[15], c.a[8]), (TRAP_BASE | 1 << 5, 7, 0xA8), "{insn:02x?}: GRWP");
            let c = testutil::run(&insn, 1, |c| (c.psw, c.a[8], c.a[2], c.d[2]) = (psw::RESET, 0xA8, 0x100, 0x100));
            assert_ne!(c.a[8], 0xA8, "{insn:02x?}: written with PSW.GW = 1");
        }
        // A10 is not a global register: SUB.A A10, const8 never traps
        let c = testutil::run(&sc(0x20, 4), 1, |c| c.psw = psw::RESET & !psw::GW);
        assert_eq!(c.pc, CODE + 2);
    }

    #[test]
    fn op_stays_small() {
        assert!(std::mem::size_of::<AluOp>() <= 12, "{}", std::mem::size_of::<AluOp>());
    }

    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 16) as u32
        }

        fn pick(&mut self, v: &[u32]) -> u32 {
            v[self.next() as usize % v.len()]
        }

        /// Register value biased towards overflow/saturation boundaries of words, half-words, bytes.
        fn val(&mut self) -> u32 {
            const EDGE: [u32; 16] = [
                0, 1, 2, 0xFFFF_FFFF, 0x7FFF_FFFF, 0x8000_0000, 0x8000_0001, 0x4000_0000, 0xC000_0000, 0x7FFF,
                0x8000, 0xFFFF, 0xFFFF_8000, 0x7FFF_8000, 0x8000_7FFF, 0x0001_0000,
            ];
            const L8: [u32; 8] = [0, 1, 0x7F, 0x80, 0x81, 0xFF, 0x40, 0xC0];
            const L16: [u32; 8] = [0, 1, 0x7FFF, 0x8000, 0x8001, 0xFFFF, 0x4000, 0xC000];
            match self.next() % 8 {
                0 | 1 => self.pick(&EDGE),
                2 => (self.next() % 129).wrapping_sub(64),
                3 => (0..4).fold(0, |v, k| v | self.pick(&L8) << (8 * k)),
                4 => self.pick(&L16) | self.pick(&L16) << 16,
                _ => self.next(),
            }
        }

        fn k9(&mut self) -> u32 {
            if self.next() & 1 == 0 {
                self.pick(&[0, 1, 0x1FF, 0x100, 0xFF, 0x20, 0x1F, 0x3F, 0x10, 0x1E0, 0x1C0, 0x30])
            } else {
                self.next() & 0x1FF
            }
        }
    }

    /// A random instance of encoding `e`: every register/immediate field random.
    fn random_insn(r: &mut Rng, e: &Enc) -> Vec<u8> {
        let (op1, op2) = (e.3 as u32, e.4.unwrap_or(0) as u32);
        let mut f = || r.next() & 15;
        let (c, a, b, d) = (f(), f(), f(), f());
        match e.2 {
            F::RR => rrn(op1, op2, c, a, b, r.next() & 3),
            F::RC => rc(op1, op2, c, a, r.k9() as i32),
            F::RCR => rcr(op1, op2, c, d, a, r.k9() as i32),
            F::RRR => rrr(op1, op2, c, d, a, b),
            F::RLC => rlc(op1, c, a, if r.next() & 1 == 0 { r.pick(&[0, 1, 0x7FFF, 0x8000, 0xFFFF]) } else { r.next() }),
            F::SYS => sys(op1, op2),
            F::SRC => src(op1, a, b as i32),
            F::SRR => srr(op1, a, b),
            F::SC => sc(op1, r.next() & 0xFF),
            F::SR => sr(op1, op2, a),
            F::SRRS => srrs(a, b, r.next() & 3),
        }
    }

    /// Persistent bare-Unicorn engine (CODE/DATA and the CSA list of `testutil::cpu_with`, BTV, ISP)
    /// wrapped in [`oracle::Engine`] so engine lifecycle and `uc_emu_start` are synchronised across
    /// threads on Windows.
    struct Uc(oracle::Engine);

    use std::ffi::c_int;
    use unicorn_tricore_sys::*;

    impl Uc {
        fn new(model: Option<c_int>) -> Uc {
            let u = oracle::Engine::new_mapped(model).expect("unicorn open");
            u.wr(TriCoreReg::BTV, TRAP_BASE);
            u.wr(TriCoreReg::ISP, DATA + 0x7000);
            Uc(u)
        }

        /// Execute `insn` at CODE once from state `s`. With `until` the run stops when PC reaches it
        /// (cheap); otherwise after one instruction (`count = 1` makes Unicorn flush its translation
        /// cache on every start, ~1 ms) — used when the instruction is expected to trap.
        fn run(&self, insn: &[u8], s: &Setup, until: Option<u32>) -> Result<oracle::Out, uc_err> {
            self.0.mem_write(CODE as u64, insn);
            self.0.remove_cache(CODE as u64, CODE as u64 + 8);
            for k in 0..8u32 {
                let next = if k < 7 { link(CSA_BASE + 64 * (k + 1)) } else { 0 };
                self.0.mem_write((CSA_BASE + 64 * k) as u64, &next.to_le_bytes());
            }
            for i in 0..16 {
                self.0.wr(TriCoreReg::d(i), s.d[i as usize]);
                self.0.wr(TriCoreReg::a(i), s.a[i as usize]);
            }
            self.0.wr_psw(s.psw);
            for (r, v) in [(TriCoreReg::PCXI, s.pcxi), (TriCoreReg::ICR, s.icr), (TriCoreReg::FCX, s.fcx), (TriCoreReg::LCX, s.lcx)] {
                self.0.wr(r, v);
            }
            let e = self.0.emu_start(CODE as u64, until.unwrap_or(0) as u64, 0, until.is_none() as usize);
            if e != UC_ERR_OK {
                return Err(e);
            }
            let (mut d, mut a) = ([0; 16], [0; 16]);
            for i in 0..16 {
                d[i as usize] = self.0.rd(TriCoreReg::d(i));
                a[i as usize] = self.0.rd(TriCoreReg::a(i));
            }
            Ok(oracle::Out {
                d,
                a,
                psw: self.0.rd_psw(),
                pcxi: self.0.rd(TriCoreReg::PCXI),
                icr: self.0.rd(TriCoreReg::ICR),
                fcx: self.0.rd(TriCoreReg::FCX),
                lcx: self.0.rd(TriCoreReg::LCX),
                pc: self.0.rd(TriCoreReg::PC),
                data: Vec::new(),
            })
        }
    }

    /// Native vs Unicorn after one instruction; `None` when equal, or when Unicorn is a known-bad
    /// oracle (`oracle::KNOWN_WRONG`), or for the one implementation-defined case where QEMU is
    /// inconsistent with itself: 16-bit `MOV E[a], const4` with an odd `a` writes D[a]/D[a+1] there
    /// (no CHECK_REG_PAIR), while its 32-bit forms raise OPD like this core (V1 p. 6-7).
    fn compare(uc: &Uc, insn: &[u8], s: &Setup, arch: Arch) -> Option<String> {
        let hi = if insn.len() == 4 { u16::from_le_bytes([insn[2], insn[3]]) } else { 0 };
        if !oracle::trustworthy(u16::from_le_bytes([insn[0], insn[1]]), hi) || (insn[0] == 0xD2 && insn[1] & 1 != 0) {
            return None;
        }
        let n = testutil::run_arch(insn, 1, arch, |c| s.apply(c));
        let next = CODE + insn.len() as u32;
        let o = match uc.run(insn, s, (n.pc == next).then_some(next)) {
            Ok(o) => o,
            Err(e) => return Some(format!("unicorn error {e}")),
        };
        let mut diff = Vec::new();
        for r in 0..16 {
            if n.d[r] != o.d[r] {
                diff.push(format!("D{r} {:#010x}/{:#010x} (in {:#010x})", n.d[r], o.d[r], s.d[r]));
            }
            if n.a[r] != o.a[r] {
                diff.push(format!("A{r} {:#010x}/{:#010x} (in {:#010x})", n.a[r], o.a[r], s.a[r]));
            }
        }
        if n.psw != o.psw {
            diff.push(format!("PSW {:#010x}/{:#010x} (in {:#010x})", n.psw, o.psw, s.psw));
        }
        for (name, x, y) in [("PC", n.pc, o.pc), ("PCXI", n.pcxi, o.pcxi), ("ICR", n.icr, o.icr), ("FCX", n.fcx, o.fcx)] {
            if x != y {
                diff.push(format!("{name} {x:#010x}/{y:#010x}"));
            }
        }
        (!diff.is_empty()).then(|| format!("native/unicorn: {}", diff.join(", ")))
    }

    /// Random differential run over every encoding; returns the mismatches (at most 3 per encoding).
    fn differential(arch: Arch, model: Option<c_int>, seed: u64) -> Vec<String> {
        let default = if cfg!(debug_assertions) { 500 } else { 5000 };
        let samples = std::env::var("SIMEMU_ALU_DIFF_N").ok().and_then(|v| v.parse().ok()).unwrap_or(default);
        let out = std::sync::Mutex::new(Vec::new());
        std::thread::scope(|scope| {
            for (k, chunk) in ENCODINGS.chunks(12).enumerate() {
                let out = &out;
                scope.spawn(move || {
                    let uc = Uc::new(model);
                    let mut r = Rng(seed ^ (k as u64 + 1).wrapping_mul(0x9E37_79B9_7F4A_7C15));
                    for e in chunk.iter().filter(|e| arch == Arch::Tc16 || !tc16_only(e)) {
                        let mut found = 0;
                        for _ in 0..samples {
                            let insn = random_insn(&mut r, e);
                            let mut s = Setup::default();
                            for i in 0..16 {
                                (s.d[i], s.a[i]) = (r.val(), r.val());
                            }
                            s.psw = psw::RESET | (r.next() & USB);
                            (s.fcx, s.lcx) = (link(CSA_BASE), link(CSA_BASE + 64 * 6));
                            if let Some(m) = compare(&uc, &insn, &s, arch) {
                                out.lock().unwrap().push(format!("{} {} ({:?}, p. {}) {insn:02x?}: {m}", e.0, e.1, e.2, e.5));
                                found += 1;
                                if found == 3 {
                                    break;
                                }
                            }
                        }
                    }
                });
            }
        });
        out.into_inner().unwrap()
    }

    #[test]
    fn differential_vs_unicorn_tc16() {
        if !unicorn_tricore_sys::LINKED {
            return;
        }
        let m = differential(Arch::Tc16, None, 0x5EED_A1D5);
        assert!(m.is_empty(), "{} mismatches:\n{}", m.len(), m.join("\n"));
    }

    #[test]
    fn differential_vs_unicorn_tc131() {
        if !unicorn_tricore_sys::LINKED {
            return;
        }
        let m = differential(Arch::Tc131, Some(UC_CPU_TRICORE_TC1797), 0x1311);
        assert!(m.is_empty(), "{} mismatches:\n{}", m.len(), m.join("\n"));
    }

    /// Exhaustive over 16-bit encodings, sampled over 32-bit ones: the decoder accepts exactly the
    /// encodings of [`ENCODINGS`] (minus the TC1.6-only forms on TC1.3.1).
    #[test]
    fn decoder_claims_exactly_the_manual_encodings() {
        let mut table: HashSet<(u8, Option<u8>)> = ENCODINGS.iter().map(|e| (e.3, e.4)).collect();
        for arch in [Arch::Tc16, Arch::Tc131] {
            if arch == Arch::Tc131 {
                for e in ENCODINGS.iter().filter(|e| tc16_only(e)) {
                    table.remove(&(e.3, e.4));
                }
            }
            for h in (0..=u16::MAX).step_by(2) {
                let want = table.contains(&key(&h.to_le_bytes()));
                assert_eq!(decode(h, 0, 0, arch).is_some(), want, "16-bit {h:#06x} ({arch:?})");
            }
            let mut r = Rng(0xDEC0DE);
            for op1 in (1..=0xFFu32).step_by(2) {
                for _ in 0..4096 {
                    let w = r.next() << 8 | op1;
                    let want = table.contains(&key(&w.to_le_bytes()));
                    assert_eq!(decode(w as u16, (w >> 16) as u16, 0, arch).is_some(), want, "32-bit {w:#010x} ({arch:?})");
                }
            }
            // every encoding with random fields
            for e in ENCODINGS.iter().filter(|e| arch == Arch::Tc16 || !tc16_only(e)) {
                for _ in 0..64 {
                    let b = random_insn(&mut r, e);
                    let hi = if b.len() == 4 { u16::from_le_bytes([b[2], b[3]]) } else { 0 };
                    assert!(decode(u16::from_le_bytes([b[0], b[1]]), hi, 0, arch).is_some(), "{e:?} {b:02x?}");
                }
            }
        }
    }
}
