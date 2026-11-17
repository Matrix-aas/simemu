//! Group E — single-precision FPU, PACK/UNPACK, integer division and the TC1.3.1 MMU instructions.
//!
//! Manuals: V2 = `tcarch_vol2_a4.pdf` (TC1.6 Vol. 2), V1 = `tc16_architecture_vol1_a4.pdf`,
//! V131-1/V131-2 = the TriCore 1 V1.3/V1.3.1 Vol. 1/Vol. 2 (V1.3.8). All encodings are 32-bit; there
//! are no 16-bit forms in this group. The TC1.3.1 encoding boxes are glyph-encoded in the PDF
//! (ASCII - 1DH); decoded they are identical to V2 for every shared mnemonic.
//!
//! | class | format / op1 | op2 (bits) / n | manual |
//! |---|---|---|---|
//! | DIV, DIV.U (TC1.6 only) | RR 4BH | 20H/21H `[27:20]`, n=1 | V2 p. 3-84..3-85 |
//! | DVINIT, .U, .B, .BU, .H, .HU | RR 4BH | 1AH 0AH 5AH 4AH 3AH 2AH, n=0 | V2 p. 3-86..3-88, V131-2 p. 3-113..3-116 |
//! | DVSTEP, DVSTEP.U | RRR 6BH | 0FH/0EH `[23:20]`, n=0 | V2 p. 3-89, V131-2 p. 3-118..3-119 |
//! | DVADJ | RRR 6BH | 0DH, n=0 | V2 p. 3-82 (TC1.6), V131-2 p. 3-111 (TC1.3/1.3.1: different RTL) |
//! | PACK / UNPACK | RRR 6BH 00H n=0 / RR 4BH 08H n=0 | | V2 p. 3-307 / 3-381, V131-2 p. 3-365 / 3-453 |
//! | ADD.F SUB.F MADD.F MSUB.F | RRR 6BH | 02H 03H 06H 07H, n=1 | V2 p. 3-394, 3-411, 3-404, 3-406 |
//! | CMP.F DIV.F MUL.F | RR 4BH | 00H 05H 04H, n=1 | V2 p. 3-395, 3-396, 3-408 |
//! | FTOI FTOIZ FTOU FTOUZ FTOQ31 FTOQ31Z | RR 4BH | 10H 13H 12H 17H 11H 18H, n=1 | V2 p. 3-397..3-402 |
//! | ITOF UTOF Q31TOF QSEED.F UPDFL | RR 4BH | 14H 16H 15H 19H 0CH, n=1 | V2 p. 3-403, 3-413, 3-409, 3-410, 3-412 |
//! | TLBDEMAP TLBFLUSH.A/.B TLBMAP TLBPROBE.A/.I (TC1.3/1.3.1) | RR 75H | 00H 04H 05H 40H 08H 09H | V131-2 p. 3-497..3-504 |
//!
//! Fields marked `-` in the encoding boxes are ignored (UPDFL: V2 shows `a` in `[31:28]`, V131-2 `-`).
//!
//! Semantics decisions where the manuals disagree with each other (each one cited at its code):
//! * Denormal operands are replaced by signed zero *before* classification, results are rounded to
//!   the IEEE format with gradual underflow and *then* a denormal result is replaced by signed zero
//!   (V1 §11.2.2 p. 11-2 and §11.3.2 p. 11-7, which explicitly reject flushing before rounding; the
//!   V2 RTL `denorm_to_zero(precise_result)` before `ieee754_round` is the simplified form).
//! * FU = non-zero unrounded result with magnitude < 2^-126 (V1 §11.2.4 p. 11-4; the RTL omits
//!   "non-zero"). FX is suppressed when FI or FZ is set for every FPU instruction (V1 §11.4 p. 11-8,
//!   V131-1 p. 11-9 "FX can be asserted ... so long as FI and FZ are negated"; some pages omit it).
//! * Invalid operations (ADD_NAN/MUL_NAN/DIV_NAN/SQRT_NAN results) set FI (V1 Table 11-5 p. 11-8).
//! * FV is set by every instruction returning a floating-point result whose rounded magnitude
//!   reaches 2^128 (V1 §11.4 "FV" p. 11-9) — this includes Q31TOF with a large exponent adjustment,
//!   whose page says "FV not set"; 0/0 asserts FI only (V1 Table 11-5 note 4).
//! * Odd register-pair specifiers raise OPD (V1 p. 6-7 permits it; implementation defined).
//! * MMU instructions on a core without MMU raise UOPC (V131-1 §10.8 p. 10-8; TC1784 has no MMU).

use crate::arch::{psw, Arch, Trap, CSFR_TABLE};
use crate::fmt::{sext, word, Rr, Rrr};
use crate::{Bus, Cpu, Flow};

/// Mnemonics owned by this group (manual spelling).
pub const MNEMONICS: &[&str] = &[
    "ADD.F", "CMP.F", "DIV", "DIV.F", "DIV.U", "DVADJ", "DVINIT", "DVINIT.B", "DVINIT.BU",
    "DVINIT.H", "DVINIT.HU", "DVINIT.U", "DVSTEP", "DVSTEP.U", "FTOI", "FTOIZ", "FTOQ31", "FTOQ31Z",
    "FTOU", "FTOUZ", "ITOF", "MADD.F", "MSUB.F", "MUL.F", "PACK", "Q31TOF", "QSEED.F", "SUB.F",
    "TLBDEMAP", "TLBFLUSH.A", "TLBFLUSH.B", "TLBMAP", "TLBPROBE.A", "TLBPROBE.I", "UNPACK", "UPDFL",
    "UTOF"
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Div,
    DivU,
    DvInit,
    DvInitU,
    DvInitB,
    DvInitBU,
    DvInitH,
    DvInitHU,
    DvStep,
    DvStepU,
    DvAdj,
    Pack,
    Unpack,
    AddF,
    SubF,
    MaddF,
    MsubF,
    CmpF,
    DivF,
    MulF,
    Ftoi,
    Ftoiz,
    Ftou,
    Ftouz,
    Ftoq31,
    Ftoq31z,
    Itof,
    Utof,
    Q31tof,
    QseedF,
    Updfl,
    /// Odd register-pair specifier: OPD trap.
    Opd,
    /// TLB* on a core without MMU: UOPC trap.
    NoMmu,
}

/// One predecoded instruction: register fields `a [11:8]`, `b [15:12]`, `d [27:24]` (RRR) and `c`
/// `[31:28]`; `op2` is kept for the FPU trap capture (FPU_TRAP_OPC.OPC).
#[derive(Clone, Copy, Debug)]
pub struct FpuDivOp {
    pub kind: Kind,
    pub op2: u8,
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
}

pub fn decode(lo: u16, hi: u16, _pc: u32, arch: Arch) -> Option<FpuDivOp> {
    use Kind::*;
    if lo & 1 == 0 {
        return None;
    }
    let w = word(lo, hi);
    // `odd`: a register-pair specifier is odd (OPD).
    let (kind, op2, a, b, c, d, odd) = match w & 0xFF {
        0x4B => {
            let r = Rr::from(w);
            let k = match (r.op2, r.n) {
                (0x20, 1) if arch == Arch::Tc16 => Div,
                (0x21, 1) if arch == Arch::Tc16 => DivU,
                (0x1A, 0) => DvInit,
                (0x0A, 0) => DvInitU,
                (0x5A, 0) => DvInitB,
                (0x4A, 0) => DvInitBU,
                (0x3A, 0) => DvInitH,
                (0x2A, 0) => DvInitHU,
                (0x08, 0) => Unpack,
                (0x00, 1) => CmpF,
                (0x05, 1) => DivF,
                (0x04, 1) => MulF,
                (0x10, 1) => Ftoi,
                (0x13, 1) => Ftoiz,
                (0x12, 1) => Ftou,
                (0x17, 1) => Ftouz,
                (0x11, 1) => Ftoq31,
                (0x18, 1) => Ftoq31z,
                (0x14, 1) => Itof,
                (0x16, 1) => Utof,
                (0x15, 1) => Q31tof,
                (0x19, 1) => QseedF,
                (0x0C, 1) => Updfl,
                _ => return None,
            };
            let odd = r.c & 1 != 0 && matches!(k, Div | DivU | DvInit | DvInitU | DvInitB | DvInitBU | DvInitH | DvInitHU | Unpack);
            (k, r.op2, r.a, r.b, r.c, 0, odd)
        }
        0x6B => {
            let r = Rrr::from(w);
            let k = match (r.op2, r.n) {
                (0x0F, 0) => DvStep,
                (0x0E, 0) => DvStepU,
                (0x0D, 0) => DvAdj,
                (0x00, 0) => Pack,
                (0x02, 1) => AddF,
                (0x03, 1) => SubF,
                (0x06, 1) => MaddF,
                (0x07, 1) => MsubF,
                _ => return None,
            };
            let odd = match k {
                DvStep | DvStepU | DvAdj => (r.c | r.d) & 1 != 0,
                Pack => r.d & 1 != 0,
                _ => false,
            };
            (k, r.op2, r.a, r.b, r.c, r.d, odd)
        }
        // V131-2 §3.3 p. 3-497..3-504: RR, op1 75H, op2 [27:20]; [19:16] don't care (TriCore 1: TC1.3 & TC1.3.1).
        0x75 if arch != Arch::Tc16 => {
            let r = Rr::from(w);
            match r.op2 {
                0x00 | 0x04 | 0x05 | 0x40 | 0x08 | 0x09 => (NoMmu, r.op2, r.a, r.b, r.c, 0, false),
                _ => return None,
            }
        }
        _ => return None,
    };
    let kind = if odd { Opd } else { kind };
    Some(FpuDivOp { kind, op2: op2 as u8, a: a as u8, b: b as u8, c: c as u8, d: d as u8 })
}

pub fn ends_block(op: &FpuDivOp) -> bool {
    matches!(op.kind, Kind::Opd | Kind::NoMmu)
}

#[inline(always)]
pub fn exec<B: Bus>(cpu: &mut Cpu<B>, op: &FpuDivOp, pc: u32, len: u32) -> Flow {
    use Kind::*;
    let (a, b, c, d) = (op.a as usize, op.b as usize, op.c as usize, op.d as usize);
    let (va, vb, vd) = (cpu.d[a], cpu.d[b], cpu.d[d]);
    let ed = vd as u64 | (cpu.d[(d + 1) & 15] as u64) << 32;
    let rm = (cpu.psw >> 24) & 3;
    let (res, raised) = match op.kind {
        Div | DivU | DvInit | DvInitU | DvInitB | DvInitBU | DvInitH | DvInitHU => {
            let (e, ovf) = divide_init(cpu.arch, op.kind, va, vb);
            set_e(cpu, c, e);
            // V2 p. 3-85 / 3-88: V = overflow, SV |= overflow, AV = 0, SAV unchanged.
            let mut p = cpu.psw & !(psw::V | psw::AV);
            if ovf {
                p |= psw::V | psw::SV;
            }
            cpu.psw = p;
            return Flow::Next;
        }
        DvStep => {
            let e = dvstep(ed, vb);
            set_e(cpu, c, e);
            return Flow::Next;
        }
        DvStepU => {
            let e = dvstep_u(ed, vb);
            set_e(cpu, c, e);
            return Flow::Next;
        }
        DvAdj => {
            let e = if cpu.arch != Arch::Tc16 { dvadj_131(ed, vb) } else { dvadj_16(ed, vb) };
            set_e(cpu, c, e);
            return Flow::Next;
        }
        Pack => {
            cpu.d[c] = pack(ed, va, cpu.psw & psw::C != 0);
            return Flow::Next;
        }
        Unpack => {
            set_e(cpu, c, unpack(va));
            return Flow::Next;
        }
        Opd => return Flow::trap(Trap::OPD, pc),
        NoMmu => return Flow::trap(Trap::UOPC, pc),
        // ---- FPU (D[c] = result) ----
        AddF => add_f(va, vd, rm),
        SubF => add_f(va ^ SIGN, vd, rm), // D[d] - D[a]: the sign flip keeps NaN classes
        MaddF => madd_f(va, vb, vd, false, rm),
        MsubF => madd_f(va, vb, vd, true, rm),
        MulF => mul_f(va, vb, rm),
        DivF => div_f(va, vb, rm),
        CmpF => cmp_f(va, vb),
        Ftoi => ftoi(va, rm),
        Ftoiz => ftoi(va, RZ),
        Ftou => ftou(va, rm),
        Ftouz => ftou(va, RZ),
        Ftoq31 => ftoq31(va, vb, rm),
        Ftoq31z => ftoq31(va, vb, RZ),
        Itof => itof(va as i32 as i64, rm),
        Utof => itof(va as i64, rm),
        Q31tof => q31tof(va, vb, rm),
        QseedF => qseed(va),
        Updfl => {
            // V2 p. 3-412: PSW[31:24] = (PSW[31:24] & ~D[a][15:8]) | (D[a][7:0] & D[a][15:8]).
            let mask = (va >> 8) & 0xFF;
            cpu.psw = (cpu.psw & !(mask << 24)) | ((va & mask) << 24);
            (0, (va & mask & 0x7C) << 24) // exceptions it asserts (FI..FX), for the CAE trap
        }
    };
    let raised = if op.kind == Updfl {
        raised
    } else {
        cpu.d[c] = res;
        let (psw, raised) = set_flags(cpu.psw, raised);
        cpu.psw = psw;
        raised
    };
    // CAE only for an enabled exception (FIE..FXE = FPU_TRAP_CON[22:18] <-> FI..FX = PSW[30:26])
    // while FPU_TRAP_CON.TST is clear (V1 §11.5 p. 11-10).
    let con = cpu.csfrs[TRAP_CON];
    if con & 1 == 0 && (raised >> 8) & con & (0x1F << 18) != 0 {
        fpu_trap(cpu, op, pc, rm, raised, va, vb, vd);
        return Flow::fault(Trap::CAE, pc, len);
    }
    Flow::Next
}

/// PSW after an FPU instruction that raised `raised` (FI..FX): FX only while FI and FZ are negated, FS =
/// any exception, FI..FX sticky (V1 §11.4 p. 11-8). Returns (PSW, the exceptions asserted).
#[inline(always)]
fn set_flags(psw: u32, raised: u32) -> (u32, u32) {
    let raised = if raised & (FI | FZ) != 0 { raised & !FX } else { raised };
    ((psw & !FS) | raised | if raised != 0 { FS } else { 0 }, raised)
}

/// FTOU, FTOQ31, FTOQ31Z and Q31TOF (RR 4BH, n = 1, `op2` 12H/11H/18H/15H) as [`exec`] executes them:
/// `(D[c], PSW)` from `D[a]`, `D[b]` and `PSW` (RM and flags). Unicorn 2.1.4 does not decode these four and raises
/// IOPC instead; `simemu_tc1791::isa_fix` executes them with this function. The CAE trap (FPU_TRAP_CON)
/// is not considered — as for every FPU instruction Unicorn translates.
pub fn conversion(op2: u32, a: u32, b: u32, psw: u32) -> Option<(u32, u32)> {
    let rm = (psw >> 24) & 3;
    let (res, raised) = match op2 {
        0x12 => ftou(a, rm),
        0x11 => ftoq31(a, b, rm),
        0x18 => ftoq31(a, b, RZ),
        0x15 => q31tof(a, b, rm),
        _ => return None,
    };
    Some((res, set_flags(psw, raised).0))
}

#[inline(always)]
fn set_e<B: Bus>(cpu: &mut Cpu<B>, c: usize, e: u64) {
    cpu.d[c] = e as u32;
    cpu.d[c + 1] = (e >> 32) as u32;
}

// ---- integer division (V2 p. 3-82..3-89, V131-2 p. 3-111..3-119) ------------------------------------

/// Signed quotient-range overflow for TriCore 1.3 (`1.3.0`, V131-2 pp. 3-114, 3-116) on
/// `DVINIT.B` (`sh = 7`) and `DVINIT.H` (`sh = 15`).
#[inline(always)]
fn dvinit_signed_ovf_130(qsign: bool, a: u32, b: u32, sh: u32) -> bool {
    let ua = (a as i32).unsigned_abs() as u64;
    let ub = (b as i32).unsigned_abs() as u64;
    let sig = ua >> sh;
    let base = ua & ((1u64 << sh) - 1);
    if qsign && ub != 0 {
        (sig == ub && base >= ub) || sig > ub
    } else {
        sig >= ub
    }
}

/// DIV/DIV.U (V2 p. 3-84) and the DVINIT group (V2 p. 3-86..3-88; V131-2 pp. 3-114..3-116 `1.3.0`
/// vs `1.3.1` overflow rules). Returns E[c] and the overflow condition.
#[inline(always)]
fn divide_init(arch: Arch, k: Kind, a: u32, b: u32) -> (u64, bool) {
    let qsign = (a ^ b) >> 31 != 0;
    let sa = a as i32 as i64 as u64;
    match k {
        Kind::Div => {
            let (x, y) = (a as i32, b as i32);
            let (q, r) = if y == 0 {
                (if x >= 0 { 0x7FFF_FFFF } else { 0x8000_0000 }, 0)
            } else if y == -1 && x == i32::MIN {
                (0x7FFF_FFFF, 0)
            } else {
                ((x / y) as u32, (x % y) as u32) // C semantics: remainder takes the dividend's sign
            };
            (q as u64 | (r as u64) << 32, y == 0 || (y == -1 && x == i32::MIN))
        }
        Kind::DivU => {
            let (q, r) = a.checked_div(b).map_or((0xFFFF_FFFF, 0), |q| (q, a % b));
            (q as u64 | (r as u64) << 32, b == 0)
        }
        Kind::DvInit => (sa, b == 0 || (b == 0xFFFF_FFFF && a == 0x8000_0000)),
        Kind::DvInitU => (a as u64, b == 0),
        Kind::DvInitB => {
            let ovf = if arch == Arch::Tc13 {
                dvinit_signed_ovf_130(qsign, a, b, 7)
            } else {
                b == 0 || (b == 0xFFFF_FFFF && a == 0xFFFF_FF80)
            };
            (sa << 24 | if qsign { 0xFF_FFFF } else { 0 }, ovf)
        }
        Kind::DvInitBU => {
            let ovf = if arch == Arch::Tc13 { (a >> 8) >= b } else { b == 0 };
            ((a as u64) << 24, ovf)
        }
        Kind::DvInitH => {
            let ovf = if arch == Arch::Tc13 {
                dvinit_signed_ovf_130(qsign, a, b, 15)
            } else {
                b == 0 || (b == 0xFFFF_FFFF && a == 0xFFFF_8000)
            };
            (sa << 16 | if qsign { 0xFFFF } else { 0 }, ovf)
        }
        Kind::DvInitHU => {
            let ovf = if arch == Arch::Tc13 { (a >> 16) >= b } else { b == 0 };
            ((a as u64) << 16, ovf)
        }
        _ => unreachable!(),
    }
}

/// DVSTEP (V2 p. 3-89). `remainder` is wider than 32 bits in the RTL (`E[c] = {remainder[31:0], ..}`
/// truncates it at the end): kept in i64 so `remainder << 1` cannot wrap.
#[inline(always)]
fn dvstep(e: u64, b: u32) -> u64 {
    let dividend_sign = e >> 63 != 0;
    let quotient_sign = dividend_sign != (b >> 31 != 0);
    let addend = if quotient_sign { b as i32 as i64 } else { -(b as i32 as i64) };
    let mut dq = e as u32;
    let mut rem = (e >> 32) as i32 as i64;
    for _ in 0..8 {
        rem = (rem << 1) | (dq >> 31) as i64;
        dq <<= 1;
        let temp = rem + addend;
        let take = (temp < 0) == dividend_sign;
        if take {
            rem = temp;
        }
        dq |= (if take { !quotient_sign } else { quotient_sign }) as u32;
    }
    (rem as u32 as u64) << 32 | dq as u64
}

/// DVSTEP.U (V2 p. 3-90): unsigned restoring step, 33-bit remainder before the compare.
#[inline(always)]
fn dvstep_u(e: u64, b: u32) -> u64 {
    let mut dq = e as u32;
    let mut rem = e >> 32; // < 2^33 after the shift
    for _ in 0..8 {
        rem = (rem << 1) | (dq >> 31) as u64;
        dq <<= 1;
        let ge = rem >= b as u64;
        if ge {
            rem -= b as u64;
        }
        dq |= ge as u32;
    }
    (rem as u32 as u64) << 32 | dq as u64
}

/// DVADJ, TC1.6 RTL (V2 p. 3-82). The overflow case is architecturally `64'bx`; the non-overflow
/// formula is returned for it.
fn dvadj_16(e: u64, b: u32) -> u64 {
    let hi = (e >> 32) as u32;
    let x_sign = e >> 63 != 0;
    let q_sign = x_sign != (b >> 31 != 0);
    let eq_pos = x_sign && hi == b;
    let eq_neg = x_sign && hi == b.wrapping_neg();
    let q = e as u32;
    let quotient = if (q_sign && !eq_neg) || eq_pos { q.wrapping_add(1) } else { q };
    let remainder = if eq_pos || eq_neg { 0 } else { hi };
    (remainder as u64) << 32 | quotient as u64
}

/// DVADJ, TC1.3.1 RTL (V131-2 p. 3-111).
fn dvadj_131(e: u64, b: u32) -> u64 {
    let hi = (e >> 32) as u32;
    let q = e as u32;
    let qneg = q >> 31 != 0;
    if (hi as i32 as i64).abs() == (b as i32 as i64).abs() && e >> 63 != 0 {
        if qneg { q as u64 } else { q.wrapping_add(1) as u64 }
    } else if qneg {
        (hi as u64) << 32 | q.wrapping_add(1) as u64
    } else {
        e
    }
}

/// PACK (V2 p. 3-307, V131-2 identical): PSW.C is the extra sticky bit of the round-to-nearest flag.
fn pack(e: u64, a: u32, carry: bool) -> u32 {
    let int_exp = (e >> 32) as i32;
    let m = e as u32;
    let rnd = (m >> 7) & 1 != 0 && ((m >> 8) & 1 != 0 || m & 0x7F != 0 || carry);
    let top = m >> 31 != 0;
    let (exp, frac) = if !top && int_exp == 255 {
        (255, (m >> 8) & 0x7F_FFFF) // infinity or NaN
    } else if top && int_exp >= 127 {
        (255, 0) // overflow -> infinity
    } else if (top && int_exp <= -128) || m == 0 {
        (0, 0) // underflow -> zero, or zero
    } else {
        let temp_exp = if top { (int_exp + 128) as u32 } else { 0 };
        let ef = ((((temp_exp & 0xFF) << 23) | ((m >> 8) & 0x7F_FFFF)) + rnd as u32) & 0x7FFF_FFFF;
        (ef >> 23, ef & 0x7F_FFFF)
    };
    (a & SIGN) | exp << 23 | frac
}

/// UNPACK (V2 p. 3-381, V131-2 identical).
fn unpack(a: u32) -> u64 {
    let exp = (a >> 23) & 0xFF;
    let frac = a & 0x7F_FFFF;
    let (ie, im): (i32, u32) = match (exp, frac) {
        (255, _) => (255, frac << 7),
        (0, 0) => (-127, 0),
        (0, _) => (-126, frac << 7),
        _ => (exp as i32 - 127, 1 << 30 | frac << 7),
    };
    (ie as u32 as u64) << 32 | im as u64
}

// ---- FPU (V2 §3.2 p. 3-393..3-413, syntax V2 Table 1-11 p. 1-11, V1 ch. 11) --------------------------

const SIGN: u32 = 1 << 31;
// PSW exception flags (V1 Table 11-4 p. 11-8): FS, FI, FV, FZ, FU, FX = PSW[31:26].
const FS: u32 = 1 << 31;
const FI: u32 = 1 << 30;
const FV: u32 = 1 << 29;
const FZ: u32 = 1 << 28;
const FU: u32 = 1 << 27;
const FX: u32 = 1 << 26;
// PSW.RM (V1 Table 11-3 p. 11-6).
const RN: u32 = 0;
const RU: u32 = 1;
const RD: u32 = 2;
const RZ: u32 = 3;
// V2 Table 1-11.
const QUIET_NAN: u32 = 0x7FC0_0000;
const ADD_NAN: u32 = 0x7FC0_0001;
const MUL_NAN: u32 = 0x7FC0_0002;
const SQRT_NAN: u32 = 0x7FC0_0004;
const DIV_NAN: u32 = 0x7FC0_0008;
const INF: u32 = 0x7F80_0000;

/// FPU trap capture registers (V1 §11.6 p. 11-11..11-16), indices into `Cpu::csfrs`.
const fn csfr_idx(off: u16) -> usize {
    let mut i = 0;
    while CSFR_TABLE[i].offset != off {
        i += 1;
    }
    i
}
const TRAP_CON: usize = csfr_idx(0xA000);
const TRAP_PC: usize = csfr_idx(0xA004);
const TRAP_OPC: usize = csfr_idx(0xA008);
const TRAP_SRC1: usize = csfr_idx(0xA010);
const TRAP_SRC2: usize = csfr_idx(0xA014);
const TRAP_SRC3: usize = csfr_idx(0xA018);

/// FPU CAE trap (V1 §11.5 p. 11-10, V131-1 §11.5 p. 11-13): capture PC/opcode/operands/flags/RM
/// (V1 §11.6 p. 11-11..11-16), set TST and take the asynchronous CAE trap (class 4 TIN 4) after the
/// instruction completed. SRC1/2/3 = D[a]/D[b]/D[d] (the s1/s2/s3 fields, V2 Table 1-7).
#[cold]
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn fpu_trap<B: Bus>(cpu: &mut Cpu<B>, op: &FpuDivOp, pc: u32, rm: u32, raised: u32, va: u32, vb: u32, vd: u32) {
    let con = cpu.csfrs[TRAP_CON];
    cpu.csfrs[TRAP_CON] = (con & !(0x1F << 26) & !(3 << 8)) | (raised & (0x1F << 26)) | rm << 8 | 1;
    cpu.csfrs[TRAP_PC] = pc;
    let rr = matches!(op.kind, Kind::AddF | Kind::SubF | Kind::MaddF | Kind::MsubF) as u32 ^ 1;
    cpu.csfrs[TRAP_OPC] = (op.c as u32) << 16 | rr << 8 | op.op2 as u32;
    (cpu.csfrs[TRAP_SRC1], cpu.csfrs[TRAP_SRC2], cpu.csfrs[TRAP_SRC3]) = (va, vb, vd);
}

/// An operand after denormal substitution (V1 §11.2.2): value = (-1)^neg · m · 2^e for `Num`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Cls {
    Zero,
    Num,
    Inf,
    Nan,
}
#[derive(Clone, Copy)]
struct F {
    cls: Cls,
    neg: bool,
    m: u64,
    e: i32,
}

#[inline(always)]
fn unp(x: u32) -> F {
    let neg = x >> 31 != 0;
    let (be, f) = ((x >> 23) & 0xFF, x & 0x7F_FFFF);
    let cls = match be {
        0 => Cls::Zero,
        0xFF if f == 0 => Cls::Inf,
        0xFF => Cls::Nan,
        _ => Cls::Num,
    };
    F { cls, neg, m: (f | 0x80_0000) as u64, e: be as i32 - 150 }
}

/// `is_s_nan` (V2 Table 1-11).
#[inline(always)]
fn is_snan(x: u32) -> bool {
    x & 0x7FC0_0000 == 0x7F80_0000 && x & 0x3F_FFFF != 0
}
#[inline(always)]
fn is_nan(x: u32) -> bool {
    x & 0x7FFF_FFFF > INF
}

/// Result for NaN operands: QUIET_NAN, FI if one is signalling (V1 §11.2.3 p. 11-3).
#[inline(always)]
fn nan_result(xs: &[u32]) -> (u32, u32) {
    (QUIET_NAN, if xs.iter().any(|&x| is_snan(x)) { FI } else { 0 })
}

#[inline(always)]
fn signed(neg: bool, mag: u32) -> u32 {
    if neg { mag | SIGN } else { mag }
}

/// Round-up decision for a magnitude whose discarded part is `rest` (`half` = one half ulp).
#[inline(always)]
fn round_up(rm: u32, neg: bool, odd: bool, rest: u128, half: u128, sticky: bool) -> bool {
    let inexact = rest != 0 || sticky;
    match rm {
        RN => rest > half || (rest == half && (sticky || odd)),
        RU => inexact && !neg,
        RD => inexact && neg,
        _ => false,
    }
}

/// Round (-1)^neg · (sig + sticky·ε) · 2^exp (sig != 0) to single precision per `rm` with gradual
/// underflow, then substitute signed zero for a denormal result (V1 §11.3 p. 11-6..11-7). Returns the
/// bits and FV/FU/FX: FV when the rounded result with unbounded exponent reaches 2^128 (the result is
/// then ∞ or the largest finite number per V1 §11.4 "FV"), FU for a non-zero unrounded magnitude
/// below 2^-126 (V1 §11.2.4), FX when the result differs from the unrounded value.
#[inline(always)]
fn round_pack(neg: bool, exp: i32, sig: u64, sticky: bool, rm: u32) -> (u32, u32) {
    let lz = sig.leading_zeros();
    let (sig, exp) = (sig << lz, exp - lz as i32);
    let e = exp + 63; // exponent of the leading one
    if e >= -126 {
        let rest = (sig & ((1 << 40) - 1)) as u128;
        let mut m = sig >> 40;
        let fx = if rest != 0 || sticky { FX } else { 0 };
        m += round_up(rm, neg, m & 1 != 0, rest, 1 << 39, sticky) as u64;
        let e = if m == 1 << 24 {
            m >>= 1;
            e + 1
        } else {
            e
        };
        if e > 127 {
            let inf = match rm {
                RN => true,
                RU => !neg,
                RD => neg,
                _ => false,
            };
            return (signed(neg, if inf { INF } else { 0x7F7F_FFFF }), FV | FX);
        }
        (signed(neg, ((e + 127) as u32) << 23 | (m as u32 & 0x7F_FFFF)), fx)
    } else {
        // Tiny: round on the denormal grid (unit 2^-149); only the carry into 2^-126 survives.
        let sh = ((-149 - exp) as u32).min(127); // > 40 here
        let w = sig as u128;
        let k = (w >> sh) as u64;
        let up = round_up(rm, neg, k & 1 != 0, w & ((1u128 << sh) - 1), 1u128 << (sh - 1), sticky);
        let min_normal = k + up as u64 == 1 << 23;
        (signed(neg, if min_normal { 0x0080_0000 } else { 0 }), FU | FX)
    }
}

/// Exact (-1)^sx·mx·2^ex + (-1)^sy·my·2^ey, rounded (either magnitude may be zero). An exactly
/// cancelling sum is +0, or -0 when rounding toward -∞ (IEEE-754 §6.3).
#[inline(always)]
fn add_round(sx: bool, mx: u64, ex: i32, sy: bool, my: u64, ey: i32, rm: u32) -> (u32, u32) {
    if mx == 0 || my == 0 {
        return match (mx, my) {
            (0, 0) => (signed(if sx == sy { sx } else { rm == RD }, 0), 0),
            (0, _) => round_pack(sy, ey, my, false, rm),
            _ => round_pack(sx, ex, mx, false, rm),
        };
    }
    let (lx, ly) = (mx.leading_zeros(), my.leading_zeros());
    let (x, y) = ((sx, mx << lx, ex - lx as i32), (sy, my << ly, ey - ly as i32));
    let ((sb, mb, eb), (ss, ms, es)) = if (x.2, x.1) >= (y.2, y.1) { (x, y) } else { (y, x) };
    let diff = (eb - es) as u32;
    let small = (ms as u128) << 62;
    let (sm, mut sticky) = if diff >= 127 { (0, true) } else { (small >> diff, small & ((1u128 << diff) - 1) != 0) };
    let big = (mb as u128) << 62;
    let sum = if sb == ss { big + sm } else { big - sm - sticky as u128 };
    if sum == 0 {
        return (signed(rm == RD, 0), 0);
    }
    let top = 128 - sum.leading_zeros();
    let (sig, exp) = if top > 64 {
        let sh = top - 64;
        sticky |= sum & ((1u128 << sh) - 1) != 0;
        ((sum >> sh) as u64, eb - 62 + sh as i32)
    } else {
        (sum as u64, eb - 62)
    };
    round_pack(sb, exp, sig, sticky, rm)
}

/// ADD.F D[c] = D[a] + D[d] (V2 p. 3-394); SUB.F calls it with D[a] negated (V2 p. 3-411).
#[inline(always)]
fn add_f(a: u32, d: u32, rm: u32) -> (u32, u32) {
    if is_nan(a) || is_nan(d) {
        return nan_result(&[a, d]);
    }
    let (x, y) = (unp(a), unp(d));
    match (x.cls, y.cls) {
        (Cls::Inf, Cls::Inf) if x.neg != y.neg => (ADD_NAN, FI),
        (Cls::Inf, _) => (signed(x.neg, INF), 0),
        (_, Cls::Inf) => (signed(y.neg, INF), 0),
        _ => {
            let mx = if x.cls == Cls::Zero { 0 } else { x.m };
            let my = if y.cls == Cls::Zero { 0 } else { y.m };
            add_round(x.neg, mx, x.e, y.neg, my, y.e, rm)
        }
    }
}

/// MUL.F (V2 p. 3-408).
#[inline(always)]
fn mul_f(a: u32, b: u32, rm: u32) -> (u32, u32) {
    if is_nan(a) || is_nan(b) {
        return nan_result(&[a, b]);
    }
    let (x, y) = (unp(a), unp(b));
    let neg = x.neg != y.neg;
    match (x.cls, y.cls) {
        (Cls::Inf, Cls::Zero) | (Cls::Zero, Cls::Inf) => (MUL_NAN, FI),
        (Cls::Inf, _) | (_, Cls::Inf) => (signed(neg, INF), 0),
        (Cls::Zero, _) | (_, Cls::Zero) => (signed(neg, 0), 0),
        _ => round_pack(neg, x.e + y.e, x.m * y.m, false, rm),
    }
}

/// MADD.F D[c] = D[d] + D[a]·D[b] / MSUB.F D[c] = D[d] - D[a]·D[b], fused (V2 p. 3-404/3-406).
/// Invalid-∞ rule per V1 Table 11-5 (the RTL's second ADD_NAN condition tests D[b] where D[d] is
/// meant).
#[inline(always)]
fn madd_f(a: u32, b: u32, d: u32, sub: bool, rm: u32) -> (u32, u32) {
    if is_nan(a) || is_nan(b) || is_nan(d) {
        return nan_result(&[a, b, d]);
    }
    let (x, y, z) = (unp(a), unp(b), unp(d));
    let pneg = (x.neg != y.neg) != sub;
    match (x.cls, y.cls) {
        (Cls::Inf, Cls::Zero) | (Cls::Zero, Cls::Inf) => (MUL_NAN, FI),
        (Cls::Inf, _) | (_, Cls::Inf) => {
            if z.cls == Cls::Inf && z.neg != pneg {
                (ADD_NAN, FI)
            } else {
                (signed(pneg, INF), 0)
            }
        }
        _ if z.cls == Cls::Inf => (signed(z.neg, INF), 0),
        _ => {
            let p = if x.cls == Cls::Zero || y.cls == Cls::Zero { 0 } else { x.m * y.m };
            let mz = if z.cls == Cls::Zero { 0 } else { z.m };
            add_round(pneg, p, x.e + y.e, z.neg, mz, z.e, rm)
        }
    }
}

/// DIV.F (V2 p. 3-396). x/±0 with finite non-zero x: ∞ and FZ; ∞/0 is exact (V1 §11.4 "FZ").
fn div_f(a: u32, b: u32, rm: u32) -> (u32, u32) {
    if is_nan(a) || is_nan(b) {
        return nan_result(&[a, b]);
    }
    let (x, y) = (unp(a), unp(b));
    let neg = x.neg != y.neg;
    match (x.cls, y.cls) {
        (Cls::Inf, Cls::Inf) | (Cls::Zero, Cls::Zero) => (DIV_NAN, FI),
        (Cls::Inf, _) => (signed(neg, INF), 0),
        (_, Cls::Inf) | (Cls::Zero, _) => (signed(neg, 0), 0),
        (_, Cls::Zero) => (signed(neg, INF), FZ),
        _ => {
            let n = x.m << 40;
            round_pack(neg, x.e - y.e - 40, n / y.m, n % y.m != 0, rm)
        }
    }
}

/// CMP.F (V2 p. 3-395): no denormal substitution; ±0 compare equal.
#[inline(always)]
fn cmp_f(a: u32, b: u32) -> (u32, u32) {
    let key = |x: u32| if x & SIGN != 0 { -((x & !SIGN) as i64) } else { x as i64 };
    let denorm = |x: u32| x & 0x7F80_0000 == 0 && x & 0x7F_FFFF != 0;
    let mut r = if is_nan(a) || is_nan(b) {
        8
    } else {
        let (ka, kb) = (key(a), key(b));
        (ka < kb) as u32 | ((ka == kb) as u32) << 1 | ((ka > kb) as u32) << 2
    };
    r |= (denorm(a) as u32) << 4 | (denorm(b) as u32) << 5;
    (r, if is_snan(a) || is_snan(b) { FI } else { 0 })
}

/// m·2^s (m != 0, m < 2^24) as an integer magnitude rounded per `rm`: (truncated, rounded, inexact).
/// Truncated values >= 2^39 saturate to 2^40, beyond every target range.
#[inline(always)]
fn to_fixed(neg: bool, m: u64, s: i32, rm: u32) -> (u64, u64, bool) {
    if s >= 16 {
        return (1 << 40, 1 << 40, false);
    }
    if s >= 0 {
        return (m << s, m << s, false);
    }
    let sh = ((-s) as u32).min(127);
    let w = m as u128;
    let t = (w >> sh) as u64;
    let rest = w & ((1u128 << sh) - 1);
    (t, t + round_up(rm, neg, t & 1 != 0, rest, 1u128 << (sh - 1), false) as u64, rest != 0)
}

/// Signed conversion with the manual's range checks on the unrounded value: FTOI/FTOIZ (V2 p. 3-397/
/// 3-398) and FTOQ31/FTOQ31Z (V2 p. 3-399/3-400, upper bound q_real(7FFFFFFFH), lower bound -1.0).
#[inline(always)]
fn to_signed(x: F, s: i32, rm: u32) -> (u32, u32) {
    let (t, r, inexact) = to_fixed(x.neg, x.m, s, rm);
    if !x.neg && (t > 0x7FFF_FFFF || (t == 0x7FFF_FFFF && inexact)) {
        (0x7FFF_FFFF, FI)
    } else if x.neg && (t > 0x8000_0000 || (t == 0x8000_0000 && inexact)) {
        (0x8000_0000, FI)
    } else {
        (if x.neg { (r as u32).wrapping_neg() } else { r as u32 }, if inexact { FX } else { 0 })
    }
}

#[inline(always)]
fn ftoi(a: u32, rm: u32) -> (u32, u32) {
    let x = unp(a);
    match x.cls {
        Cls::Nan => (0, FI),
        Cls::Inf => (if x.neg { 0x8000_0000 } else { 0x7FFF_FFFF }, FI),
        Cls::Zero => (0, 0),
        Cls::Num => to_signed(x, x.e, rm),
    }
}

/// FTOU/FTOUZ (V2 p. 3-401/3-402): any value < 0.0 gives 0 and FI.
#[inline(always)]
fn ftou(a: u32, rm: u32) -> (u32, u32) {
    let x = unp(a);
    match x.cls {
        Cls::Nan => (0, FI),
        Cls::Inf | Cls::Num if x.neg => (0, FI),
        Cls::Inf => (0xFFFF_FFFF, FI),
        Cls::Zero => (0, 0),
        Cls::Num => {
            let (t, r, inexact) = to_fixed(false, x.m, x.e, rm);
            if t > 0xFFFF_FFFF || (t == 0xFFFF_FFFF && inexact) {
                (0xFFFF_FFFF, FI)
            } else {
                (r as u32, if inexact { FX } else { 0 })
            }
        }
    }
}

/// FTOQ31/FTOQ31Z: D[a]·2^-D[b][8:0] to Q31. FX compares the adjusted value (the RTL's
/// `f_real(D[a]) != q_real(result)` drops the adjustment).
fn ftoq31(a: u32, b: u32, rm: u32) -> (u32, u32) {
    let x = unp(a);
    let adj = sext(b & 0x1FF, 9) as i32;
    match x.cls {
        Cls::Nan => (0, FI),
        Cls::Inf => (if x.neg { 0x8000_0000 } else { 0x7FFF_FFFF }, FI),
        Cls::Zero => (0, 0),
        Cls::Num => to_signed(x, x.e - adj + 31, rm),
    }
}

/// ITOF/UTOF (V2 p. 3-403/3-413).
#[inline(always)]
fn itof(v: i64, rm: u32) -> (u32, u32) {
    if v == 0 { (0, 0) } else { round_pack(v < 0, 0, v.unsigned_abs(), false, rm) }
}

/// Q31TOF (V2 p. 3-409): q_real(D[a])·2^D[b][8:0].
fn q31tof(a: u32, b: u32, rm: u32) -> (u32, u32) {
    let v = a as i32 as i64;
    let adj = sext(b & 0x1FF, 9) as i32;
    if v == 0 { (0, 0) } else { round_pack(v < 0, adj - 31, v.unsigned_abs(), false, rm) }
}

/// QSEED.F significands, indexed by {(E-1)[0], M[22:17]}. The manual only bounds the precision
/// (approx_inv_sqrt: >= 6.75 bits, V2 Table 1-11); this table was measured on TriCore hardware
/// (QEMU target/tricore/fpu_helper.c, `target_qseed_significand_table`) and meets that bound
/// (checked by `qseed_table_meets_manual_precision`).
const QSEED: [u8; 128] = [
    253, 252, 245, 244, 239, 238, 231, 230, 225, 224, 217, 216, 211, 210, 205, 204, 201, 200, 195, 194,
    189, 188, 185, 184, 179, 178, 175, 174, 169, 168, 165, 164, 161, 160, 157, 156, 153, 152, 149, 148,
    145, 144, 141, 140, 137, 136, 133, 132, 131, 130, 127, 126, 123, 122, 121, 120, 117, 116, 115, 114,
    111, 110, 109, 108, 103, 102, 99, 98, 93, 92, 89, 88, 83, 82, 79, 78, 75, 74, 71, 70, 67, 66, 63,
    62, 59, 58, 55, 54, 53, 52, 49, 48, 45, 44, 43, 42, 39, 38, 37, 36, 33, 32, 31, 30, 27, 26, 25, 24,
    23, 22, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2,
];

/// QSEED.F (V2 p. 3-410): ±0 (after denormal substitution) -> ±∞, negative -> SQRT_NAN with FI,
/// +∞ -> +0.
fn qseed(a: u32) -> (u32, u32) {
    if is_nan(a) {
        return nan_result(&[a]);
    }
    let x = unp(a);
    match x.cls {
        Cls::Zero => (signed(x.neg, INF), 0),
        _ if x.neg => (SQRT_NAN, FI),
        Cls::Inf => (0, 0),
        _ => {
            let em1 = ((a >> 23) & 0xFF) - 1;
            let idx = ((em1 & 1) << 6 | (a >> 17) & 0x3F) as usize;
            ((0xBD - em1 / 2) << 23 | (QSEED[idx] as u32) << 15, 0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::{oracle, run, run_arch, CODE, TRAP_BASE};
    use crate::NullBus;

    fn rr(op2: u32, n: u32, a: u32, b: u32, c: u32) -> u32 {
        0x4B | a << 8 | b << 12 | n << 16 | op2 << 20 | c << 28
    }
    fn rrr(op2: u32, n: u32, a: u32, b: u32, c: u32, d: u32) -> u32 {
        0x6B | a << 8 | b << 12 | n << 16 | op2 << 20 | d << 24 | c << 28
    }
    fn one(w: u32, setup: impl FnOnce(&mut Cpu<NullBus>)) -> Cpu<NullBus> {
        run(&w.to_le_bytes(), 1, setup)
    }
    fn f(x: f32) -> u32 {
        x.to_bits()
    }
    fn e(c: &Cpu<NullBus>, n: usize) -> u64 {
        c.d[n] as u64 | (c.d[n + 1] as u64) << 32
    }
    fn fpu(w: u32, a: u32, b: u32, d: u32, rm: u32) -> (u32, u32) {
        let c = one(w, |c| {
            (c.d[1], c.d[2], c.d[3]) = (a, b, d);
            c.psw = rm << 24;
        });
        assert_eq!(c.pc, CODE + 4, "no trap expected");
        (c.d[4], c.psw >> 26 << 26)
    }
    // d4 = op(d1 [, d2 | d3])
    const ADDF: u32 = 0x4321_016B; // add.f d4, d3, d1 (V2 p. 3-394 box; rizin agrees)
    fn addf() -> u32 {
        rrr(0x02, 1, 1, 0, 4, 3)
    }

    #[test]
    fn encodings_match_manual_boxes() {
        // add.f d4, d3, d1 built from the V2 p. 3-394 box
        assert_eq!(addf(), ADDF);
        let k = |w: u32, arch| decode(w as u16, (w >> 16) as u16, 0, arch).map(|o| o.kind);
        use Kind::*;
        for (w, kind) in [
            (rr(0x20, 1, 1, 2, 4), Div), (rr(0x21, 1, 1, 2, 4), DivU), (rr(0x1A, 0, 1, 2, 4), DvInit),
            (rr(0x0A, 0, 1, 2, 4), DvInitU), (rr(0x5A, 0, 1, 2, 4), DvInitB), (rr(0x4A, 0, 1, 2, 4), DvInitBU),
            (rr(0x3A, 0, 1, 2, 4), DvInitH), (rr(0x2A, 0, 1, 2, 4), DvInitHU), (rrr(0x0F, 0, 0, 2, 4, 6), DvStep),
            (rrr(0x0E, 0, 0, 2, 4, 6), DvStepU), (rrr(0x0D, 0, 0, 2, 4, 6), DvAdj), (rrr(0x00, 0, 1, 0, 4, 6), Pack),
            (rr(0x08, 0, 1, 0, 4), Unpack), (rrr(0x02, 1, 1, 0, 4, 3), AddF), (rrr(0x03, 1, 1, 0, 4, 3), SubF),
            (rrr(0x06, 1, 1, 2, 4, 3), MaddF), (rrr(0x07, 1, 1, 2, 4, 3), MsubF), (rr(0x00, 1, 1, 2, 4), CmpF),
            (rr(0x05, 1, 1, 2, 4), DivF), (rr(0x04, 1, 1, 2, 4), MulF), (rr(0x10, 1, 1, 0, 4), Ftoi),
            (rr(0x13, 1, 1, 0, 4), Ftoiz), (rr(0x12, 1, 1, 0, 4), Ftou), (rr(0x17, 1, 1, 0, 4), Ftouz),
            (rr(0x11, 1, 1, 2, 4), Ftoq31), (rr(0x18, 1, 1, 2, 4), Ftoq31z), (rr(0x14, 1, 1, 0, 4), Itof),
            (rr(0x16, 1, 1, 0, 4), Utof), (rr(0x15, 1, 1, 2, 4), Q31tof), (rr(0x19, 1, 1, 0, 4), QseedF),
            (rr(0x0C, 1, 1, 0, 1), Updfl),
        ] {
            assert_eq!(k(w, Arch::Tc16), Some(kind), "{w:#010x}");
            let tc131 = if matches!(kind, Div | DivU) { None } else { Some(kind) };
            assert_eq!(k(w, Arch::Tc131), tc131, "{w:#010x} on TC1.3.1");
        }
        // don't-care bits [19:18] and unused register fields
        assert_eq!(k(rr(0x04, 1, 1, 2, 4) | 3 << 18, Arch::Tc16), Some(MulF));
        // wrong n, unused op2
        assert_eq!(k(rr(0x04, 0, 1, 2, 4), Arch::Tc16), None);
        assert_eq!(k(rr(0x1A, 1, 1, 2, 4), Arch::Tc16), None);
        assert_eq!(k(rrr(0x02, 0, 1, 0, 4, 3), Arch::Tc16), None);
        // odd pairs -> OPD
        assert_eq!(k(rr(0x1A, 0, 1, 2, 5), Arch::Tc16), Some(Opd));
        assert_eq!(k(rrr(0x0F, 0, 0, 2, 4, 7), Arch::Tc16), Some(Opd));
        assert_eq!(k(rrr(0x00, 0, 1, 0, 4, 3), Arch::Tc16), Some(Opd), "PACK E[d]");
        // TLB* (V131-2 p. 3-497..3-504): TC1.3.1 only, bits [19:16] don't care
        for op2 in [0x00, 0x04, 0x05, 0x40, 0x08, 0x09] {
            let w = 0x75 | 2 << 8 | op2 << 20 | 0xF << 16;
            assert_eq!(k(w, Arch::Tc131), Some(NoMmu));
            assert_eq!(k(w, Arch::Tc16), None);
        }
        assert_eq!(k(0x75 | 0x41 << 20, Arch::Tc131), None);
        assert_eq!(k(0x0000, Arch::Tc16), None, "no 16-bit forms");
    }

    // ---- integer division ----

    #[test]
    fn div_and_div_u() {
        let w = rr(0x20, 1, 1, 2, 4);
        let c = one(w, |c| (c.d[1], c.d[2], c.psw) = ((-7i32) as u32, 2, psw::AV | psw::SAV));
        assert_eq!(e(&c, 4), ((-1i32) as u32 as u64) << 32 | (-3i32) as u32 as u64, "-7/2 = -3 r -1");
        assert_eq!(c.psw & (psw::V | psw::SV | psw::AV | psw::SAV), psw::SAV, "AV cleared, SAV kept");
        let c = one(w, |c| (c.d[1], c.d[2]) = (5, 0));
        assert_eq!((e(&c, 4), c.psw & (psw::V | psw::SV)), (0x7FFF_FFFF, psw::V | psw::SV));
        let c = one(w, |c| (c.d[1], c.d[2]) = (0x8000_0001, 0));
        assert_eq!(e(&c, 4), 0x8000_0000);
        let c = one(w, |c| (c.d[1], c.d[2]) = (0x8000_0000, u32::MAX));
        assert_eq!((e(&c, 4), c.psw & psw::V), (0x7FFF_FFFF, psw::V));
        let w = rr(0x21, 1, 1, 2, 4);
        let c = one(w, |c| (c.d[1], c.d[2], c.psw) = (0xFFFF_FFFF, 0x10, psw::V));
        assert_eq!((e(&c, 4), c.psw & psw::V), (0xF_0000_0000_u64 | 0x0FFF_FFFF, 0));
        let c = one(w, |c| (c.d[1], c.d[2]) = (9, 0));
        assert_eq!((e(&c, 4), c.psw & psw::V), (0xFFFF_FFFF, psw::V));
        // TC1.3.1 has no DIV: IOPC
        let c = run_arch(&rr(0x20, 1, 1, 2, 4).to_le_bytes(), 1, Arch::Tc131, |_| ());
        assert_eq!(c.pc, TRAP_BASE | 2 << 5);
        assert_eq!(c.d[15], 1);
    }

    /// Full DVINIT/DVSTEP/DVADJ sequences against exact division (V2 p. 3-82..3-90).
    fn seq(arch: Arch, init: u32, step: u32, steps: usize, adj: bool, a: u32, b: u32) -> (u64, bool) {
        let mut code = vec![rr(init, 0, 1, 2, 4)];
        code.extend(std::iter::repeat_n(rrr(step, 0, 0, 2, 4, 4), steps));
        if adj {
            code.push(rrr(0x0D, 0, 0, 2, 4, 4));
        }
        let bytes: Vec<u8> = code.iter().flat_map(|w| w.to_le_bytes()).collect();
        let c = run_arch(&bytes, code.len() as u64, arch, |c| (c.d[1], c.d[2]) = (a, b));
        (e(&c, 4), c.psw & psw::V != 0)
    }

    #[test]
    fn divide_sequences_match_exact_division() {
        let mut x = 0x9E37_79B9u32;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x
        };
        for arch in [Arch::Tc16, Arch::Tc131, Arch::Tc13] {
            for i in 0..3000 {
                let (mut a, mut b) = (next(), next());
                if i % 3 == 0 {
                    b >>= next() % 32;
                }
                if i % 5 == 0 {
                    a = (a as i32 / ((b | 1) as i32)).wrapping_mul(b as i32) as u32; // exact multiples
                }
                if b == 0 || (a == 0x8000_0000 && b == u32::MAX) {
                    continue;
                }
                let (q, r) = ((a as i32).wrapping_div(b as i32), (a as i32).wrapping_rem(b as i32));
                assert_eq!(seq(arch, 0x1A, 0x0F, 4, true, a, b), ((r as u32 as u64) << 32 | q as u32 as u64, false), "{arch:?} {a:#x}/{b:#x}");
                assert_eq!(seq(arch, 0x0A, 0x0E, 4, false, a, b), (((a % b) as u64) << 32 | (a / b) as u64, false), "{a:#x}/u{b:#x}");
                // 16- and 8-bit quotient variants with operands whose quotient fits
                let (a16, b16) = ((a as i32 >> 8) as u32, ((b as i32) >> 16) as u32 | 0x100);
                let q16 = (a16 as i32) / (b16 as i32);
                if (-0x8000..0x8000).contains(&q16) {
                    let r16 = (a16 as i32) % (b16 as i32);
                    assert_eq!(seq(arch, 0x3A, 0x0F, 2, true, a16, b16).0, (r16 as u32 as u64) << 32 | q16 as u32 as u64);
                }
                let (au, bu) = (a >> 9, b >> 16 | 0x80);
                assert_eq!(seq(arch, 0x2A, 0x0E, 2, false, au, bu).0, (((au % bu) as u64) << 32) | (au / bu) as u64);
                let (a8, b8) = ((a as i32 >> 16) as u32, (b as i32 >> 7) as u32 | 0x0100);
                let q8 = (a8 as i32) / (b8 as i32);
                if b8 != 0 && (-0x80..0x80).contains(&q8) {
                    let r8 = (a8 as i32) % (b8 as i32);
                    assert_eq!(seq(arch, 0x5A, 0x0F, 1, true, a8, b8).0, (r8 as u32 as u64) << 32 | q8 as u32 as u64);
                }
                let (au8, bu8) = (a >> 17, b >> 8 | 0x0100);
                assert_eq!(seq(arch, 0x4A, 0x0E, 1, false, au8, bu8).0, (((au8 % bu8) as u64) << 32) | (au8 / bu8) as u64);
            }
        }
        // 64/32 signed: DVSTEP x4 + DVADJ on a 64-bit dividend
        let mut rng = Rng(0xC0FF_EE11);
        for _ in 0..20000 {
            let (n, b, q, rem) = signed_64_32(&mut rng);
            let mut ev = n as u64;
            for _ in 0..4 {
                ev = dvstep(ev, b as u32);
            }
            let want = (rem as u32 as u64) << 32 | q as u32 as u64;
            assert_eq!(dvadj_16(ev, b as u32), want, "{n:#x}/{b:#x}");
            assert_eq!(dvadj_131(ev, b as u32), want, "{n:#x}/{b:#x} (TC1.3.1 DVADJ)");
        }
        // 64/32 unsigned with a divisor >= 2^31 needs the 33-bit remainder of DVSTEP.U
        let (hi, lo, dv) = (0xC000_0000u32, 0x1234_5678u32, 0xF000_0001u32);
        let mut ev = (hi as u64) << 32 | lo as u64;
        for _ in 0..4 {
            ev = dvstep_u(ev, dv);
        }
        let n = (hi as u128) << 32 | lo as u128;
        assert_eq!(ev, ((n % dv as u128) as u64) << 32 | (n / dv as u128) as u64);
    }

    #[test]
    fn dvinit_layouts_and_overflow() {
        let c = one(rr(0x5A, 0, 1, 2, 4), |c| (c.d[1], c.d[2]) = (0xFFFF_FF85, 3));
        assert_eq!(e(&c, 4), 0xFFFF_FFFF_85FF_FFFF, "sign_ext(D[a]) << 24 | ones (signs differ)");
        let c = one(rr(0x5A, 0, 1, 2, 4), |c| (c.d[1], c.d[2]) = (0xFFFF_FF80, u32::MAX));
        assert_eq!((e(&c, 4), c.psw & psw::V), (0xFFFF_FFFF_8000_0000, psw::V));
        let c = one(rr(0x4A, 0, 1, 2, 4), |c| (c.d[1], c.d[2]) = (0x8000_0001, 0));
        assert_eq!((e(&c, 4), c.psw & psw::V), (0x80_0000_0100_0000, psw::V));
        let c = one(rr(0x3A, 0, 1, 2, 4), |c| (c.d[1], c.d[2]) = (0x1234, 0xFFFF_FFF0));
        assert_eq!(e(&c, 4), 0x1234_FFFF);
        let c = one(rr(0x3A, 0, 1, 2, 4), |c| (c.d[1], c.d[2]) = (0xFFFF_8000, u32::MAX));
        assert_eq!(c.psw & (psw::V | psw::SV), psw::V | psw::SV);
        let c = one(rr(0x2A, 0, 1, 2, 4), |c| (c.d[1], c.d[2]) = (0xABCD_1234, 7));
        assert_eq!((e(&c, 4), c.psw & psw::V), (0xABCD_1234_0000, 0));
        let c = one(rr(0x1A, 0, 1, 2, 4), |c| (c.d[1], c.d[2]) = (0x8000_0000, u32::MAX));
        assert_eq!((e(&c, 4), c.psw & psw::V), (0xFFFF_FFFF_8000_0000, psw::V));
        let c = one(rr(0x0A, 0, 1, 2, 4), |c| (c.d[1], c.d[2], c.psw) = (0x8000_0000, 1, psw::AV));
        assert_eq!((e(&c, 4), c.psw & (psw::V | psw::AV)), (0x8000_0000, 0));
    }

    #[test]
    fn dvadj_corner_cases() {
        // -6 / 3: DVINIT+4×DVSTEP leaves remainder -3 (== -divisor) and quotient -3 in ones complement
        let mut ev = (-6i64) as u64;
        for _ in 0..4 {
            ev = dvstep(ev, 3);
        }
        assert_eq!(dvadj_16(ev, 3), (-2i64 as u32) as u64);
        assert_eq!(dvadj_131(ev, 3), (-2i64 as u32) as u64);
        // TC1.3.1 formula of V131-2 p. 3-111 on a hand-made input
        assert_eq!(dvadj_131(0xFFFF_FFFD_0000_0005, 3), 6);
        assert_eq!(dvadj_131(0x0000_0001_FFFF_FFFC, 7), 0x0000_0001_FFFF_FFFD);
        assert_eq!(dvadj_131(0x0000_0001_0000_0004, 7), 0x0000_0001_0000_0004);
    }

    #[test]
    fn odd_pairs_raise_opd_and_tlb_raises_uopc() {
        let c = one(rr(0x1A, 0, 1, 2, 5), |_| ());
        assert_eq!((c.pc, c.d[15], c.a[11]), (TRAP_BASE | 2 << 5, 3, CODE));
        for arch in [Arch::Tc131, Arch::Tc13] {
            for op2 in [0x00, 0x04, 0x05, 0x40, 0x08, 0x09] {
                let c = run_arch(&(0x75u32 | 2 << 8 | op2 << 20).to_le_bytes(), 1, arch, |_| ());
                assert_eq!((c.pc, c.d[15], c.a[11], c.icount), (TRAP_BASE | 2 << 5, 2, CODE, 1), "{arch:?} TLB op2 {op2:#x}");
            }
        }
    }

    #[test]
    fn tc13_dvadj_and_dvinit_130_overflow() {
        // V131-2 p. 3-111: DVADJ on TC1.3 and TC1.3.1 uses dvadj_131 (6), whereas TC1.6 uses dvadj_16 (5).
        let dvadj_w = rrr(0x0D, 0, 0, 2, 4, 4).to_le_bytes();
        for (arch, want) in [(Arch::Tc13, 6u64), (Arch::Tc131, 6u64), (Arch::Tc16, 5u64)] {
            let c = run_arch(&dvadj_w, 1, arch, |c| {
                c.d[4] = 5;
                c.d[5] = (-3i32) as u32;
                c.d[2] = 3;
            });
            assert_eq!(e(&c, 4), want, "DVADJ on {arch:?}");
        }

        // V131-2 pp. 3-114, 3-116: `1.3.0` (`Arch::Tc13`) checks full 8-/16-bit quotient overflow,
        // whereas `1.3.1` (`Arch::Tc131`) only checks div-by-zero / MIN / -1.
        let run_ovf = |arch: Arch, op2: u32, a: u32, b: u32| -> bool {
            let w = rr(op2, 0, 1, 2, 4).to_le_bytes();
            let c = run_arch(&w, 1, arch, |c| (c.d[1], c.d[2]) = (a, b));
            assert_eq!((c.psw & psw::V != 0), (c.psw & psw::SV != 0));
            c.psw & psw::V != 0
        };
        // DVINIT.B (0x5A): signed i8 range -128..=127
        assert!(!run_ovf(Arch::Tc13, 0x5A, 127, 1));
        assert!(run_ovf(Arch::Tc13, 0x5A, 128, 1));
        assert!(!run_ovf(Arch::Tc131, 0x5A, 128, 1));
        assert!(!run_ovf(Arch::Tc13, 0x5A, (-128i32) as u32, 1));
        assert!(run_ovf(Arch::Tc13, 0x5A, (-129i32) as u32, 1));
        assert!(run_ovf(Arch::Tc13, 0x5A, (-128i32) as u32, u32::MAX));
        assert!(!run_ovf(Arch::Tc13, 0x5A, (-257i32) as u32, 2));
        assert!(run_ovf(Arch::Tc13, 0x5A, (-258i32) as u32, 2));
        assert!(run_ovf(Arch::Tc13, 0x5A, 0, 0));

        // DVINIT.BU (0x4A): unsigned u8 range 0..=255
        assert!(!run_ovf(Arch::Tc13, 0x4A, 255, 1));
        assert!(run_ovf(Arch::Tc13, 0x4A, 256, 1));
        assert!(!run_ovf(Arch::Tc131, 0x4A, 256, 1));
        assert!(run_ovf(Arch::Tc13, 0x4A, 0, 0));

        // DVINIT.H (0x3A): signed i16 range -32768..=32767
        assert!(!run_ovf(Arch::Tc13, 0x3A, 32767, 1));
        assert!(run_ovf(Arch::Tc13, 0x3A, 32768, 1));
        assert!(!run_ovf(Arch::Tc131, 0x3A, 32768, 1));
        assert!(!run_ovf(Arch::Tc13, 0x3A, (-32768i32) as u32, 1));
        assert!(run_ovf(Arch::Tc13, 0x3A, (-32769i32) as u32, 1));

        // DVINIT.HU (0x2A): unsigned u16 range 0..=65535
        assert!(!run_ovf(Arch::Tc13, 0x2A, 65535, 1));
        assert!(run_ovf(Arch::Tc13, 0x2A, 65536, 1));
        assert!(!run_ovf(Arch::Tc131, 0x2A, 65536, 1));
    }

    #[test]
    fn pack_unpack() {
        // UNPACK 1.5f: exponent 0, mantissa 2.30 = 0x6000_0000
        let c = one(rr(0x08, 0, 1, 0, 4), |c| c.d[1] = f(1.5));
        assert_eq!(e(&c, 4), 0x6000_0000);
        for (x, ev) in [(0u32, 0xFFFF_FF81_0000_0000u64), (0x0000_0001, 0xFFFF_FF82_0000_0080), (INF, 0xFF_0000_0000), (0xFF80_0001, 0xFF_0000_0080)] {
            assert_eq!(unpack(x), ev, "{x:#x}");
        }
        // PACK: biased exponent = int_exp + 128, sign from D[a][31]: (-1, 0x8000_0000) -> -1.0
        let w = rrr(0x00, 0, 1, 0, 4, 6);
        let c = one(w, |c| (c.d[6], c.d[7], c.d[1]) = (0x8000_0000, (-1i32) as u32, SIGN));
        assert_eq!(c.d[4], f(-1.0));
        // round to nearest even with PSW.C as sticky
        let m = 0x8000_0180u32; // bit 8 = 1, bit 7 = 1 -> round up
        assert_eq!(pack(m as u64, 0, false), 128 << 23 | 2);
        assert_eq!(pack(0x80u64 | 0x8000_0000, 0, false), 128 << 23, "tie, even: no round");
        assert_eq!(pack(0x80u64 | 0x8000_0000, 0, true), 128 << 23 | 1, "PSW.C sticky");
        assert_eq!(pack((127u64) << 32 | 0x8000_0000, 0, false), INF, "overflow");
        assert_eq!(pack(((-128i32) as u32 as u64) << 32 | 0x8000_0000, 0, false), 0, "underflow");
        assert_eq!(pack(255u64 << 32 | 0x0000_1200, 0, false), INF | 0x12, "NaN payload");
        assert_eq!(pack(5u64 << 32 | 0x4000_0000, 0, false), 0x40_0000, "denormal: temp_exp 0");
        // normals round-trip through UNPACK (2.30), then mantissa << 1 (1.31) with exponent - 1
        for x in [1.0f32, -3.75, 1e-30, 6.0e37] {
            let u = unpack(x.to_bits());
            let ex = ((u >> 32) as i32 - 1) as u32 as u64;
            assert_eq!(pack(ex << 32 | ((u as u32) << 1) as u64, x.to_bits(), false), x.to_bits());
        }
    }

    // ---- FPU ----

    #[test]
    fn add_sub_special_cases() {
        let sub = rrr(0x03, 1, 1, 0, 4, 3);
        assert_eq!(fpu(addf(), f(1.0), 0, f(2.0), RN), (f(3.0), 0));
        assert_eq!(fpu(sub, f(1.0), 0, f(2.0), RN), (f(1.0), 0), "D[d] - D[a]");
        assert_eq!(fpu(addf(), INF, 0, INF | SIGN, RN), (ADD_NAN, FS | FI));
        assert_eq!(fpu(sub, INF, 0, INF, RN), (ADD_NAN, FS | FI));
        assert_eq!(fpu(addf(), 0x7F80_0001, 0, f(1.0), RN), (QUIET_NAN, FS | FI), "sNaN");
        assert_eq!(fpu(addf(), QUIET_NAN | 5, 0, f(1.0), RN), (QUIET_NAN, 0), "qNaN not propagated");
        assert_eq!(fpu(addf(), INF, 0, f(1.0), RN), (INF, 0), "∞ is exact");
        assert_eq!(fpu(addf(), f(1.0), 0, f(-1.0), RN), (0, 0));
        assert_eq!(fpu(addf(), f(1.0), 0, f(-1.0), RD), (SIGN, 0), "-0 toward -∞");
        assert_eq!(fpu(addf(), 1, 0, SIGN, RN), (0, 0), "denormal + -0 = +0 + -0");
        assert_eq!(fpu(addf(), f(1.0), 0, f(1e-10), RN), (f(1.0), FS | FX));
        assert_eq!(fpu(addf(), f(1.0), 0, f(1e-10), RU), (f(1.0) + 1, FS | FX));
        assert_eq!(fpu(addf(), f(f32::MAX), 0, f(f32::MAX), RN), (INF, FS | FV | FX));
        assert_eq!(fpu(addf(), f(f32::MAX), 0, f(f32::MAX), RZ), (f(f32::MAX), FS | FV | FX));
        assert_eq!(fpu(addf(), f(-f32::MAX), 0, f(-f32::MAX), RU), (f(-f32::MAX), FS | FV | FX));
        // flags are sticky except FS; ALU-flag view of the same bits is overwritten
        let c = one(addf(), |c| (c.d[1], c.d[3], c.psw) = (f(1.0), f(2.0), FS | FU | psw::IS));
        assert_eq!(c.psw, FU | psw::IS);
    }

    #[test]
    fn underflow_rounds_before_zero_substitution() {
        // V1 §11.3.2: the unrounded tiny result is rounded on the IEEE grid first, then a denormal
        // result is replaced by zero. 2^-63 · (2 - 2^-23)·2^-64 = 2^-126·(1 - 2^-24) lies exactly
        // halfway between the largest denormal and 2^-126.
        let mul = rr(0x04, 1, 1, 2, 4);
        let (a, b) = (f(2f32.powi(-63)), 0x3FFF_FFFF - (64 << 23)); // (2 - 2^-23)·2^-64
        assert_eq!(fpu(mul, a, b, 0, RN), (0x0080_0000, FS | FU | FX), "tie -> even -> 2^-126");
        assert_eq!(fpu(mul, a, b, 0, RU), (0x0080_0000, FS | FU | FX));
        assert_eq!(fpu(mul, a, b, 0, RZ), (0, FS | FU | FX), "largest denormal -> +0");
        assert_eq!(fpu(mul, a | SIGN, b, 0, RD), (0x8080_0000, FS | FU | FX));
        assert_eq!(fpu(mul, a | SIGN, b, 0, RU), (SIGN, FS | FU | FX), "-denormal -> -0");
        // just below the tie: RN rounds down to a denormal -> zero
        assert_eq!(fpu(mul, a, b - 1, 0, RN), (0, FS | FU | FX));
        // 2^-70 · 2^-70 = 2^-140 is a denormal even after rounding up
        assert_eq!(fpu(mul, f(2f32.powi(-70)), f(-2f32.powi(-70)), 0, RN), (SIGN, FS | FU | FX));
        assert_eq!(fpu(mul, f(2f32.powi(-70)), f(2f32.powi(-70)), 0, RU), (0, FS | FU | FX));
        // exact cancellation is not tiny: no FU
        assert_eq!(fpu(addf(), 0x0080_0000, 0, 0x8080_0000, RN), (0, 0));
    }

    #[test]
    fn mul_div_madd() {
        let mul = rr(0x04, 1, 1, 2, 4);
        let div = rr(0x05, 1, 1, 2, 4);
        let madd = rrr(0x06, 1, 1, 2, 4, 3);
        let msub = rrr(0x07, 1, 1, 2, 4, 3);
        assert_eq!(fpu(mul, f(1.5), f(-2.0), 0, RN), (f(-3.0), 0));
        assert_eq!(fpu(mul, INF, 0x0000_0010, 0, RN), (MUL_NAN, FS | FI), "∞ · denormal(=0)");
        assert_eq!(fpu(mul, INF | SIGN, f(2.0), 0, RN), (INF | SIGN, 0));
        assert_eq!(fpu(div, f(1.0), f(3.0), 0, RN), (f(1.0 / 3.0), FS | FX));
        assert_eq!(fpu(div, f(1.0), f(3.0), 0, RU), (f(1.0 / 3.0), FS | FX), "RN already rounded up");
        assert_eq!(fpu(div, f(1.0), f(3.0), 0, RZ), (f(1.0 / 3.0) - 1, FS | FX));
        assert_eq!(fpu(div, f(-2.0), f(3.0), 0, RD), (f(-2.0 / 3.0), FS | FX));
        assert_eq!(fpu(div, f(-2.0), f(3.0), 0, RU), (f(-2.0 / 3.0) - 1, FS | FX));
        assert_eq!(fpu(div, f(-1.0), 0, 0, RN), (INF | SIGN, FS | FZ));
        assert_eq!(fpu(div, f(1.0), 0x8000_0001, 0, RN), (INF | SIGN, FS | FZ), "denormal divisor is -0");
        assert_eq!(fpu(div, 0, SIGN, 0, RN), (DIV_NAN, FS | FI), "0/0 is FI only");
        assert_eq!(fpu(div, INF, INF | SIGN, 0, RN), (DIV_NAN, FS | FI));
        assert_eq!(fpu(div, INF, 0, 0, RN), (INF, 0), "∞/0 exact, no FZ");
        assert_eq!(fpu(div, f(2.0), INF, 0, RN), (0, 0));
        // fused: 1 + (1+2^-23)(1-2^-23) = 2 - 2^-46 -> rounds to 2.0 (inexact) only once
        let e = 2f32.powi(-23);
        assert_eq!(fpu(madd, f(1.0 + e), f(1.0 - e), f(-1.0), RN), (f(-2f32.powi(-46)), 0), "exact fused");
        assert_eq!(fpu(madd, f(1.0 + e), f(1.0 - e), f(1.0), RZ), (f(2.0) - 1, FS | FX));
        assert_eq!(fpu(msub, f(2.0), f(3.0), f(10.0), RN), (f(4.0), 0), "D[d] - D[a]·D[b]");
        assert_eq!(fpu(madd, INF, f(2.0), INF | SIGN, RN), (ADD_NAN, FS | FI), "∞·x + -∞ (V1 Table 11-5)");
        assert_eq!(fpu(msub, INF, f(2.0), INF, RN), (ADD_NAN, FS | FI));
        assert_eq!(fpu(msub, INF, f(2.0), INF | SIGN, RN), (INF | SIGN, 0));
        assert_eq!(fpu(madd, 0, INF, f(1.0), RN), (MUL_NAN, FS | FI));
        assert_eq!(fpu(madd, f(1.0), f(1.0), INF, RN), (INF, 0));
        assert_eq!(fpu(madd, f(1.0), 0, SIGN, RN), (0, 0), "+0 + -0");
        assert_eq!(fpu(madd, f(-1.0), 0, SIGN, RN), (SIGN, 0), "-0 + -0");
    }

    #[test]
    fn compare() {
        let cmp = rr(0x00, 1, 1, 2, 4);
        assert_eq!(fpu(cmp, f(1.0), f(2.0), 0, RN), (1, 0));
        assert_eq!(fpu(cmp, 0, SIGN, 0, RN), (2, 0), "+0 == -0");
        assert_eq!(fpu(cmp, f(-1.0), f(-2.0), 0, RN), (4, 0));
        assert_eq!(fpu(cmp, 2, 1, 0, RN), (4 | 16 | 32, 0), "denormals compared, not substituted");
        assert_eq!(fpu(cmp, QUIET_NAN, f(1.0), 0, RN), (8, 0));
        assert_eq!(fpu(cmp, 0x7F80_0001, f(1.0), 0, RN), (8, FS | FI));
        assert_eq!(fpu(cmp, INF | SIGN, f(-1e38), 0, RN), (1, 0));
    }

    #[test]
    fn conversions() {
        let ftoi = rr(0x10, 1, 1, 0, 4);
        let ftoiz = rr(0x13, 1, 1, 0, 4);
        let ftou = rr(0x12, 1, 1, 0, 4);
        let ftouz = rr(0x17, 1, 1, 0, 4);
        let itof = rr(0x14, 1, 1, 0, 4);
        let utof = rr(0x16, 1, 1, 0, 4);
        assert_eq!(fpu(ftoi, f(2.5), 0, 0, RN), (2, FS | FX), "ties to even");
        assert_eq!(fpu(ftoi, f(3.5), 0, 0, RN), (4, FS | FX));
        assert_eq!(fpu(ftoi, f(-2.5), 0, 0, RD), ((-3i32) as u32, FS | FX));
        assert_eq!(fpu(ftoi, f(-2.5), 0, 0, RU), ((-2i32) as u32, FS | FX));
        assert_eq!(fpu(ftoiz, f(-2.9), 0, 0, RN), ((-2i32) as u32, FS | FX));
        assert_eq!(fpu(ftoi, f(2147483648.0), 0, 0, RN), (0x7FFF_FFFF, FS | FI));
        assert_eq!(fpu(ftoi, f(-2147483648.0), 0, 0, RN), (0x8000_0000, 0), "-2^31 in range");
        assert_eq!(fpu(ftoi, f(-2147483904.0), 0, 0, RN), (0x8000_0000, FS | FI));
        assert_eq!(fpu(ftoiz, QUIET_NAN, 0, 0, RN), (0, FS | FI), "FX suppressed by FI");
        assert_eq!(fpu(ftoi, 1, 0, 0, RU), (0, 0), "denormal is +0: exact");
        assert_eq!(fpu(ftou, f(-0.25), 0, 0, RN), (0, FS | FI), "< 0.0 is invalid");
        assert_eq!(fpu(ftou, SIGN, 0, 0, RN), (0, 0), "-0 is not negative");
        assert_eq!(fpu(ftou, f(4294967040.0), 0, 0, RN), (0xFFFF_FF00, 0));
        assert_eq!(fpu(ftou, f(4294967296.0), 0, 0, RN), (0xFFFF_FFFF, FS | FI));
        assert_eq!(fpu(ftouz, f(7.99), 0, 0, RU), (7, FS | FX));
        assert_eq!(fpu(ftouz, INF | SIGN, 0, 0, RN), (0, FS | FI));
        assert_eq!(fpu(itof, 0x8000_0000, 0, 0, RN), (f(-2147483648.0), 0));
        assert_eq!(fpu(itof, 0x7FFF_FFFF, 0, 0, RN), (f(2147483648.0), FS | FX));
        assert_eq!(fpu(itof, 0x7FFF_FFFF, 0, 0, RZ), (f(2147483520.0), FS | FX));
        assert_eq!(fpu(utof, 0xFFFF_FFFF, 0, 0, RN), (f(4294967296.0), FS | FX));
        assert_eq!(fpu(utof, 0, 0, 0, RD), (0, 0), "+0 in every mode");
    }

    #[test]
    fn q31_conversions() {
        let ftoq = rr(0x11, 1, 1, 2, 4);
        let ftoqz = rr(0x18, 1, 1, 2, 4);
        let qtof = rr(0x15, 1, 1, 2, 4);
        assert_eq!(fpu(ftoq, f(0.5), 0, 0, RN), (0x4000_0000, 0));
        assert_eq!(fpu(ftoq, f(-1.0), 0, 0, RN), (0x8000_0000, 0), "-1.0 in range");
        assert_eq!(fpu(ftoq, f(1.0), 0, 0, RN), (0x7FFF_FFFF, FS | FI));
        assert_eq!(fpu(ftoq, f(8.0), 4, 0, RN), (0x4000_0000, 0), "8 · 2^-4 = 0.5");
        assert_eq!(fpu(ftoq, f(0.25), 0x1FF, 0, RN), (0x4000_0000, 0), "D[b][8:0] = -1 -> ·2");
        assert_eq!(fpu(ftoq, f(0.25), 0xFFFF_FEFF, 0, RN), (0, FS | FX), "only [8:0]: 255 -> 2^-257");
        // -(1 + 2^-23)·2^-9 = -(2^22 + 0.5) in Q31 units
        let x = 0xBB00_0001;
        assert_eq!(fpu(ftoqz, x, 0, 0, RD), (0xFFC0_0000, FS | FX), "toward zero, PSW.RM ignored");
        assert_eq!(fpu(ftoq, x, 0, 0, RD), (0xFFBF_FFFF, FS | FX));
        assert_eq!(fpu(ftoq, x, 0, 0, RN), (0xFFC0_0000, FS | FX), "tie -> even");
        assert_eq!(fpu(ftoq, f(0.5), 1, 0, RN), (0x2000_0000, 0));
        assert_eq!(fpu(ftoq, QUIET_NAN, 0, 0, RN), (0, FS | FI));
        assert_eq!(fpu(qtof, 0x4000_0000, 0, 0, RN), (f(0.5), 0));
        assert_eq!(fpu(qtof, 0x8000_0000, 3, 0, RN), (f(-8.0), 0));
        assert_eq!(fpu(qtof, 0x7FFF_FFFF, 0, 0, RN), (f(1.0), FS | FX));
        assert_eq!(fpu(qtof, 0x4000_0000, 0x100, 0, RN), (0, FS | FU | FX), "2^-1·2^-256");
        assert_eq!(fpu(qtof, 0x4000_0000, 0xFF, 0, RN), (INF, FS | FV | FX), "2^254 overflows");
    }

    #[test]
    fn qseed_and_updfl() {
        let q = rr(0x19, 1, 1, 0, 4);
        assert_eq!(fpu(q, f(1.0), 0, 0, RN), (0x3F7E_8000, 0));
        assert_eq!(fpu(q, SIGN | 1, 0, 0, RN), (INF | SIGN, 0), "-denormal is -0 -> -∞");
        assert_eq!(fpu(q, f(-4.0), 0, 0, RN), (SQRT_NAN, FS | FI));
        assert_eq!(fpu(q, INF, 0, 0, RN), (0, 0));
        assert_eq!(fpu(q, 0x7F80_0001, 0, 0, RN), (QUIET_NAN, FS | FI));
        // UPDFL: V2 p. 3-412 example 0301H selects round toward +∞ and keeps the flags
        let w = rr(0x0C, 1, 1, 0, 0);
        let c = one(w, |c| (c.d[1], c.psw) = (0x0301, FI | FX | psw::IS));
        assert_eq!(c.psw, FI | FX | 1 << 24 | psw::IS);
        let c = one(w, |c| (c.d[1], c.psw) = (0xFC80, FI | FX | 2 << 24));
        assert_eq!(c.psw, FS | 2 << 24, "mask FS..FX: FS set, FI..FX cleared, RM kept");
    }

    #[test]
    fn qseed_table_meets_manual_precision() {
        // approx_inv_sqrt >= 6.75 bits (V2 Table 1-11): check every table entry over its interval.
        for e in [127u32, 128] {
            for m in 0..64u32 {
                for k in 0..=16u32 {
                    let x = f32::from_bits(e << 23 | m << 17 | (k * 0x1FFFF / 16));
                    let (r, _) = qseed(x.to_bits());
                    let err = (f32::from_bits(r) as f64 * (x as f64).sqrt() - 1.0).abs();
                    assert!(err <= 2f64.powf(-6.75), "x={x} err={err}");
                }
            }
        }
    }

    #[test]
    fn fpu_cae_trap_captures_and_inhibits() {
        let mul = rr(0x04, 1, 1, 2, 4);
        let con = TRAP_CON;
        // FXE enabled, TST clear: 1/3 is inexact -> CAE (class 4 TIN 4), return address = next insn
        let c = one(rr(0x05, 1, 1, 2, 4), |c| {
            (c.d[1], c.d[2], c.d[4]) = (f(1.0), f(3.0), 0x1111);
            c.csfrs[con] = 1 << 18;
            c.psw = (RU << 24) | psw::IS;
        });
        assert_eq!((c.pc, c.d[15], c.a[11], c.icount), (TRAP_BASE | 4 << 5, 4, CODE + 4, 1));
        assert_eq!(c.d[4], f(1.0 / 3.0), "result written before the trap");
        assert_eq!(c.csfrs[con], 1 << 18 | FX | RU << 8 | 1);
        assert_eq!(c.csfrs[TRAP_PC], CODE);
        assert_eq!(c.csfrs[TRAP_OPC], 4 << 16 | 1 << 8 | 0x05);
        assert_eq!((c.csfrs[TRAP_SRC1], c.csfrs[TRAP_SRC2]), (f(1.0), f(3.0)));
        // TST set: no trap
        let c = one(mul, |c| {
            (c.d[1], c.d[2]) = (f(1.1), f(3.3));
            c.csfrs[con] = 1 << 18 | 1;
        });
        assert_eq!(c.pc, CODE + 4);
        // enabled exception not raised: no trap; RRR format bit
        let c = one(rrr(0x02, 1, 1, 0, 4, 3), |c| {
            (c.d[1], c.d[3]) = (INF, INF | SIGN);
            c.csfrs[con] = 1 << 22;
        });
        assert_eq!((c.pc, c.csfrs[TRAP_OPC]), (TRAP_BASE | 4 << 5, 4 << 16 | 0x02));
        assert_eq!(c.csfrs[con] & (0x1F << 26), FI, "FX suppressed by FI is not captured");
        // UPDFL can raise a trap by setting an enabled flag
        let c = one(rr(0x0C, 1, 1, 0, 0), |c| {
            c.d[1] = 0x1010; // set FZ
            c.csfrs[con] = 1 << 20;
        });
        assert_eq!(c.pc, TRAP_BASE | 4 << 5);
    }

    // ---- differential tests against bare Unicorn ----

    /// Floating-point operands biased toward the interesting classes.
    fn fp_operand(r: &mut impl FnMut() -> u32) -> u32 {
        const SPECIAL: [u32; 16] = [
            0, SIGN, INF, INF | SIGN, QUIET_NAN, 0x7F80_0001, 0x0000_0001, 0x807F_FFFF, 0x0080_0000,
            0x8080_0000, 0x7F7F_FFFF, 0xFF7F_FFFF, 0x3F80_0000, 0xBF80_0000, 0x4F00_0000, 0xCF00_0000,
        ];
        let x = r();
        match x % 8 {
            0 => SPECIAL[(x >> 8) as usize % 16],
            1 => (x & 0x8000_0000) | (r() & 0x007F_FFFF) | ((120 + (x >> 4) % 16) << 23), // near 1.0
            2 => (x & 0x8000_0000) | (r() & 0x007F_FFFF) | ((1 + (x >> 4) % 4) << 23), // near 2^-126
            3 => (x & 0x8000_0000) | (r() & 0x007F_FFFF) | ((150 + (x >> 4) % 8) << 23), // integers
            _ => r(),
        }
    }

    /// Bare Unicorn with the PSW user bits synchronised via [`oracle::Engine`]. PSW.RM is not
    /// usable on stock Unicorn (raw PSW writes skip `fpu_set_state`, which would anyway pass
    /// `PSW & 0x3000000` unshifted as the softfloat mode): FPU comparisons run with RM = 00.
    struct Uc(oracle::Engine);
    impl Uc {
        fn new(w: u32) -> Uc {
            let eng = oracle::Engine::new_mapped(None).expect("unicorn open");
            eng.mem_write(CODE as u64, &w.to_le_bytes());
            Uc(eng)
        }
        /// Execute the instruction once from `d[1..=3]`, `psw`; `None` if it did not retire normally.
        fn one(&self, d: [u32; 3], p: u32) -> Option<(u32, u32, u32)> {
            use unicorn_tricore_sys::TriCoreReg as R;
            for (i, v) in d.iter().enumerate() {
                self.0.wr(R::d(i as u32 + 1), *v);
            }
            self.0.wr(R::d(4), 0xDEAD_0004);
            self.0.wr(R::d(5), 0xDEAD_0005);
            self.0.wr(R::BTV, TRAP_BASE);
            self.0.wr_psw(p);
            let e = self.0.emu_start(CODE as u64, 0, 0, 1);
            if e != unicorn_tricore_sys::UC_ERR_OK || self.0.rd(R::PC) != CODE + 4 {
                return None;
            }
            Some((self.0.rd(R::d(4)), self.0.rd(R::d(5)), self.0.rd_psw()))
        }
    }

    /// Native core, same register convention as [`Uc::one`].
    fn native(w: u32, d: [u32; 3], p: u32) -> (u32, u32, u32) {
        let c = run(&w.to_le_bytes(), 1, |c| {
            (c.d[1], c.d[2], c.d[3], c.d[4], c.d[5]) = (d[0], d[1], d[2], 0xDEAD_0004, 0xDEAD_0005);
            c.psw = p;
        });
        assert_eq!(c.pc, CODE + 4);
        (c.d[4], c.d[5], c.psw)
    }

    struct Rng(u32);
    impl Rng {
        fn next(&mut self) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            self.0
        }
    }

    /// A valid intermediate state of a 32/32 divide sequence (DVINIT + `k` DVSTEPs) or, for the
    /// unsigned step, a 64/32 dividend {hi < divisor, lo}: d = [divisor, E[d] low, E[d] high].
    fn divide_state(r: &mut Rng, signed: bool, steps: usize) -> [u32; 3] {
        if signed && r.next().is_multiple_of(2) {
            // 64/32: n = q·b + rem, |rem| < |b|, rem has the dividend's sign, q fits in 32 bits
            let (n, b, _, _) = signed_64_32(r);
            let mut ev = n as u64;
            for _ in 0..steps {
                ev = dvstep(ev, b as u32);
            }
            return [b as u32, ev as u32, (ev >> 32) as u32];
        }
        loop {
            let (a, b) = (r.next(), r.next() >> (r.next() % 32));
            if b == 0 || (a == 0x8000_0000 && b == u32::MAX) {
                continue;
            }
            let mut ev = if signed {
                divide_init(Arch::Tc16, Kind::DvInit, a, b).0
            } else if r.next().is_multiple_of(2) {
                (((r.next() % b) as u64) << 32) | a as u64
            } else {
                a as u64
            };
            for _ in 0..steps {
                ev = if signed { dvstep(ev, b) } else { dvstep_u(ev, b) };
            }
            return [b, ev as u32, (ev >> 32) as u32];
        }
    }

    /// PSW.RM for every rounding instruction, checked against exact host f64 values (Unicorn cannot
    /// check RM != 00). `exact` must be exact in f64; the result must be the IEEE rounding of it.
    fn rounded_ok(rm: u32, exact: f64, r: u32) -> bool {
        let x = f32::from_bits(r) as f64;
        let up = f32::from_bits(if r & SIGN == 0 { r + 1 } else { r - 1 }) as f64; // next toward +∞
        let dn = f32::from_bits(if r & SIGN == 0 { r - 1 } else { r + 1 }) as f64;
        let ceil = x >= exact && dn < exact;
        let floor = x <= exact && up > exact;
        match rm {
            RN => r == (exact as f32).to_bits(),
            RU => ceil || x == exact,
            RD => floor || x == exact,
            _ => if exact >= 0.0 { floor || x == exact } else { ceil || x == exact },
        }
    }
    fn int_rounded_ok(rm: u32, exact: f64, r: i64) -> bool {
        let want = match rm {
            RN => exact.round_ties_even(),
            RU => exact.ceil(),
            RD => exact.floor(),
            _ => exact.trunc(),
        };
        r as f64 == want
    }

    #[test]
    fn rounding_modes_match_exact_arithmetic() {
        let mut rng = Rng(0x00DD_BA11);
        let normal = |r: &mut Rng, lo: u32, span: u32| (r.next() & SIGN) | (r.next() & 0x7F_FFFF) | (lo + r.next() % span) << 23;
        let in_range = |v: f64| v != 0.0 && v.abs() >= 2f64.powi(-126) && v.abs() < 2f64.powi(127);
        for i in 0..200_000 {
            let rm = i as u32 % 4;
            let (a, b) = (normal(&mut rng, 100, 56), normal(&mut rng, 100, 56));
            let (fa, fb) = (f32::from_bits(a) as f64, f32::from_bits(b) as f64);
            // MUL.F: the f64 product of two f32 is exact
            let (r, fl) = mul_f(a, b, rm);
            assert!(rounded_ok(rm, fa * fb, r), "mul {a:#x} {b:#x} rm{rm} -> {r:#x}");
            assert_eq!(fl & FX != 0, f32::from_bits(r) as f64 != fa * fb);
            // ADD.F: exact in f64 while the exponents differ by <= 28
            let c = (b & !(0xFF << 23)) | ((((a >> 23) & 0xFF) + rng.next() % 29 - 14).clamp(1, 254)) << 23;
            let fc = f32::from_bits(c) as f64;
            if in_range(fa + fc) {
                let (r, _) = add_f(a, c, rm);
                assert!(rounded_ok(rm, fa + fc, r), "add {a:#x} {c:#x} rm{rm} -> {r:#x}");
            }
            // DIV.F: bracket the quotient through exact products q·b (f64)
            let (q, _) = div_f(a, b, rm);
            let fq = f32::from_bits(q) as f64;
            if in_range(fa / fb) {
                let (up, dn) = (f32::from_bits(if q & SIGN == 0 { q + 1 } else { q - 1 }) as f64, f32::from_bits(if q & SIGN == 0 { q - 1 } else { q + 1 }) as f64);
                let cmp = |v: f64| (v * fb).partial_cmp(&fa).map(|o| if fb < 0.0 { o.reverse() } else { o }).unwrap(); // sign of v - a/b
                use std::cmp::Ordering::*;
                let ok = match rm {
                    RN => q == ((fa / fb) as f32).to_bits(), // f64 -> f32 double rounding is innocuous for ÷
                    RU => cmp(fq) != Less && cmp(dn) == Less,
                    RD => cmp(fq) != Greater && cmp(up) == Greater,
                    _ => if fa / fb > 0.0 { cmp(fq) != Greater && cmp(up) == Greater } else { cmp(fq) != Less && cmp(dn) == Less },
                };
                assert!(ok || cmp(fq) == Equal, "div {a:#x} {b:#x} rm{rm} -> {q:#x}");
            }
            // ITOF/UTOF/Q31TOF: integers and scaled integers are exact in f64
            let v = rng.next();
            assert!(rounded_ok(rm, v as i32 as f64, itof(v as i32 as i64, rm).0) || v == 0);
            assert!(rounded_ok(rm, v as f64, itof(v as i64, rm).0) || v == 0);
            let adj = rng.next() % 64;
            if v != 0 {
                assert!(rounded_ok(rm, v as i32 as f64 * 2f64.powi(adj as i32 - 31), q31tof(v, adj, rm).0));
            }
            // FTOI/FTOU/FTOQ31 inside the target range
            let x = normal(&mut rng, 110, 47);
            let fx = f32::from_bits(x) as f64;
            let (r, _) = ftoi(x, rm);
            if fx.abs() < 2f64.powi(31) {
                assert!(int_rounded_ok(rm, fx, r as i32 as i64), "ftoi {x:#x} rm{rm} -> {r:#x}");
            }
            let (r, fl) = ftou(x, rm);
            if fx > 0.0 && fx < 2f64.powi(32) {
                assert!(int_rounded_ok(rm, fx, r as i64), "ftou {x:#x} rm{rm} -> {r:#x}");
            } else if fx < 0.0 {
                assert_eq!((r, fl), (0, FI));
            }
            let sh = rng.next() % 40;
            let q31 = fx * 2f64.powi(31 - sh as i32);
            let (r, _) = ftoq31(x, sh, rm);
            if q31 > -2f64.powi(31) && q31 < 2f64.powi(31) - 1.0 {
                assert!(int_rounded_ok(rm, q31, r as i32 as i64), "ftoq31 {x:#x} >>{sh} rm{rm} -> {r:#x}");
            }
        }
    }

    /// A signed 64-bit dividend with a 32-bit quotient: (n, b, q, rem) with n = q·b + rem (truncating).
    fn signed_64_32(r: &mut Rng) -> (i64, i32, i32, i32) {
        loop {
            let b = (r.next() as i32) >> (r.next() % 31);
            let q = (r.next() as i32) >> (r.next() % 31);
            if b == 0 || b == i32::MIN || q == i32::MIN {
                continue;
            }
            let qb = q as i64 * b as i64;
            let m = (r.next() as i64) % (b as i64).abs();
            let rem = if qb < 0 { -m } else { m };
            return (qb + rem, b, q, rem as i32);
        }
    }

    /// Randomised differential test against bare Unicorn for every encoding Unicorn implements.
    /// Mismatches must fall into one of the documented Unicorn defects (`unicorn_defect`).
    #[test]
    fn differential_vs_unicorn() {
        if !unicorn_tricore_sys::LINKED {
            return;
        }
        let insns: &[(&str, u32)] = &[
            ("DIV", rr(0x20, 1, 1, 2, 4)), ("DIV.U", rr(0x21, 1, 1, 2, 4)), ("DVINIT", rr(0x1A, 0, 1, 2, 4)),
            ("DVINIT.U", rr(0x0A, 0, 1, 2, 4)), ("DVINIT.B", rr(0x5A, 0, 1, 2, 4)), ("DVINIT.BU", rr(0x4A, 0, 1, 2, 4)),
            ("DVINIT.H", rr(0x3A, 0, 1, 2, 4)), ("DVINIT.HU", rr(0x2A, 0, 1, 2, 4)), ("DVSTEP", rrr(0x0F, 0, 0, 1, 4, 2)),
            ("DVSTEP.U", rrr(0x0E, 0, 0, 1, 4, 2)), ("DVADJ", rrr(0x0D, 0, 0, 1, 4, 2)), ("PACK", rrr(0x00, 0, 1, 0, 4, 2)),
            ("UNPACK", rr(0x08, 0, 1, 0, 4)), ("ADD.F", rrr(0x02, 1, 1, 0, 4, 3)), ("SUB.F", rrr(0x03, 1, 1, 0, 4, 3)),
            ("MADD.F", rrr(0x06, 1, 1, 2, 4, 3)), ("MSUB.F", rrr(0x07, 1, 1, 2, 4, 3)), ("CMP.F", rr(0x00, 1, 1, 2, 4)),
            ("DIV.F", rr(0x05, 1, 1, 2, 4)), ("MUL.F", rr(0x04, 1, 1, 2, 4)), ("FTOI", rr(0x10, 1, 1, 0, 4)),
            ("FTOIZ", rr(0x13, 1, 1, 0, 4)), ("FTOU", rr(0x12, 1, 1, 0, 4)), ("FTOUZ", rr(0x17, 1, 1, 0, 4)),
            ("FTOQ31", rr(0x11, 1, 1, 2, 4)), ("FTOQ31Z", rr(0x18, 1, 1, 2, 4)), ("ITOF", rr(0x14, 1, 1, 0, 4)),
            ("UTOF", rr(0x16, 1, 1, 0, 4)), ("Q31TOF", rr(0x15, 1, 1, 2, 4)), ("QSEED.F", rr(0x19, 1, 1, 0, 4)),
            ("UPDFL", rr(0x0C, 1, 1, 0, 1)),
        ];
        let n: usize = std::env::var("FPU_DIFF_N").ok().and_then(|s| s.parse().ok()).unwrap_or(4000);
        let mut report = Vec::new();
        let mut unexplained = Vec::new();
        // not decoded by Unicorn 2.1.4 (they trap below); `simemu_tc1791::isa_fix` executes them with `conversion`
        let undecoded = ["FTOU", "FTOQ31", "FTOQ31Z", "Q31TOF"];
        for &(name, w) in insns {
            assert_eq!(oracle::trustworthy(w as u16, (w >> 16) as u16), !undecoded.contains(&name), "{name}: known Unicorn defect?");
            let uc = Uc::new(w);
            let mut rng = Rng(0x1234_5678 ^ w);
            let fp = (w >> 16) & 3 == 1 && name != "DIV" && name != "DIV.U";
            let (mut ok, mut known) = (0usize, std::collections::BTreeMap::<&str, usize>::new());
            let mut trapped = 0;
            for i in 0..n {
                let d = match name {
                    "DVSTEP" | "DVSTEP.U" => divide_state(&mut rng, name == "DVSTEP", i % 4),
                    "DVADJ" => divide_state(&mut rng, true, 4),
                    "ITOF" | "UTOF" | "Q31TOF" | "UPDFL" => [rng.next(), rng.next() >> (rng.next() % 32), rng.next()],
                    _ if fp => {
                        let mut r = || rng.next();
                        [fp_operand(&mut r), fp_operand(&mut r), fp_operand(&mut r)]
                    }
                    _ => [rng.next(), rng.next() >> (rng.next() % 32), rng.next()],
                };
                // random user/FPU flags; RM = 00 for the FPU (see `Uc`)
                let p = (rng.next() & if fp { 0xFC00_0000 } else { 0xFF00_0000 }) | psw::IS | psw::IO_SUPERVISOR;
                let Some(theirs) = uc.one(d, p) else {
                    trapped += 1;
                    continue;
                };
                let mine = native(w, d, p);
                if mine == theirs {
                    ok += 1;
                } else if let Some(why) = unicorn_defect(name, d, p, mine, theirs) {
                    *known.entry(why).or_default() += 1;
                } else {
                    unexplained.push(format!("{name}: d1..d3={d:08x?} psw={p:08x} native={mine:08x?} unicorn={theirs:08x?}"));
                }
            }
            assert_eq!(trapped == n, undecoded.contains(&name), "{name}: {trapped} of {n} trapped");
            report.push(if trapped == n {
                format!("{name:9} not implemented by Unicorn 2.1.4 (traps)")
            } else {
                format!("{name:9} {ok} identical, {trapped} trapped, known Unicorn defects: {known:?}")
            });
        }
        eprintln!("{}", report.join("\n"));
        assert!(unexplained.is_empty(), "{} unexplained differences, first ones:\n{}", unexplained.len(), unexplained[..unexplained.len().min(20)].join("\n"));
    }

    /// Documented disagreements where Unicorn (QEMU 5 TriCore + softfloat) deviates from the manual.
    fn unicorn_defect(name: &str, d: [u32; 3], p: u32, mine: (u32, u32, u32), theirs: (u32, u32, u32)) -> Option<&'static str> {
        match name {
            // QEMU helper_dvstep_u: `temp = (remainder & 0xffffffff) - r2` drops bit 32 of the
            // shifted remainder, wrong whenever the divisor is >= 2^31 and the remainder reaches it.
            "DVSTEP.U" => {
                let (b, mut dq, mut rem) = (d[0] as i64, d[1], d[2] as i64);
                for _ in 0..8 {
                    rem = (rem << 1) | (dq >> 31) as i64;
                    dq <<= 1;
                    let temp = (rem & 0xFFFF_FFFF) - b;
                    if temp >= 0 {
                        rem = temp;
                    }
                    dq |= (temp >= 0) as u32;
                }
                (theirs == (dq, rem as u32, p) && d[0] >= 1 << 31).then_some("DVSTEP.U: remainder bit 32 dropped")
            }
            // QEMU helper_updfl ignores the mask for FS..FU (writes value & mask) and never updates RM.
            "UPDFL" => {
                let (v, m) = (d[0] & 0xFF, (d[0] >> 8) & 0xFF);
                let fx = (p & !(((m >> 2) & 1) << 26)) | ((v & m & 4) << 24);
                let q = (fx & 0x07FF_FFFF & !(0x1F << 27)) | ((v & m & 0xF8) << 24);
                (theirs.2 == q && mine.0 == theirs.0).then_some("UPDFL: mask ignored for FS..FU, RM not written")
            }
            // V1 §11.3.2 p. 11-7: an unrounded result just below 2^-126 that is nearer to 2^-126 than
            // to the largest denormal rounds to 2^-126; softfloat's flush-to-zero flushes it first.
            "ADD.F" | "SUB.F" | "MUL.F" | "DIV.F" | "MADD.F" | "MSUB.F" => (mine.0 & !SIGN == 0x0080_0000
                && theirs.0 == mine.0 & SIGN
                && mine.2 == theirs.2)
                .then_some("tiny result flushed to zero before rounding"),
            _ => None,
        }
    }
}
