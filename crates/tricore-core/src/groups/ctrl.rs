//! Group D — control flow and system: J/JA/JI/JL/JLA/JLI, all conditional jumps (JEQ/JNE/JGE/JLT
//! incl. .U/.A, JZ/JNZ incl. .A/.T, JGEZ/JGTZ/JLEZ/JLTZ, JNED/JNEI), LOOP/LOOPU, CALL/CALLA/CALLI,
//! FCALL*/FRET, RET, RFE, RFM, SVLCX, RSLCX, BISR, RESTORE, SYSCALL, TRAPV/TRAPSV, DISABLE, ENABLE,
//! ISYNC, DSYNC, MFCR, MTCR, DEBUG, NOP.
//!
//! References: V2 = `vault/docs/infineon/tcarch_vol2_a4.pdf` (TriCore V1.6 Vol. 2), V1 = TriCore V1.6
//! Vol. 1, V131-1/V131-2 = TriCore 1 V1.3 & V1.3.1 Vol. 1/2 (V1.3.8). Page numbers are the printed
//! "3-NNN" page numbers of the instruction pages.
//!
//! TC1.3.1 vs TC1.6 (V131-2 vs V2): the 75 TC1.3.1 encodings of this group are bit-identical to their
//! TC1.6 counterparts (checked on the rendered V131-2 encoding boxes, e.g. JEQ p. 3-148, DISABLE
//! p. 3-109) and so are their RTL (RET p. 3-369 "1.3.1" rows, RFE p. 3-371, RFM p. 3-373). TC1.6 adds
//! 11 encodings that decode as invalid (IOPC) on `Arch::Tc13`/`Arch::Tc131`: FCALL, FCALLA, FCALLI, FRET (SR, SYS),
//! RESTORE, DISABLE `D[a]`, and the "disp4 + 16" forms of JEQ/JNE (SBC 9EH/DEH, SBR BEH/FEH).
//! The only behavioural delta is MTCR to PC (FE08H), see [`CtrlOp::Mtcr`].
//!
//! Privilege (V2 Table 1-14 p. 1-14, V131-2 §2.6 p. 2-19): BISR, MTCR, RFM need Supervisor mode
//! (`PSW.IO == 10B`, the test the RFM RTL spells out); ENABLE, DISABLE, RESTORE need User-1 or
//! Supervisor mode (`PSW.IO != 00B`; the reserved value 11B is not User-0 and is accepted). Otherwise
//! PRIV (V1 Table 6-1 class 1 TIN 1), which ranks above all context traps (V1 Table 6-2 p. 6-12).
//!
//! Context-trap order: where the instruction RTL and the priority table V1 Table 6-2 disagree for
//! simultaneous traps (RET: CDU vs CSU; RFE: CTYP vs NEST) the RTL order of the instruction page is
//! used. Any memory error during a context save/restore raises FCU (V1 "FCU" p. 6-9: "The FCU trap is
//! also taken if any error is encountered during a context save or restore operation").
//!
//! All conditional branches, calls, returns and system instructions end a predecoded block; NOP,
//! ISYNC, DSYNC and MFCR do not (ISYNC needs no pipeline model: stores into translated code are
//! invalidated by the core before the next instruction).

use crate::arch::{icr, pcxi, psw, Arch, Trap};
use crate::context::Kind;
use crate::fmt::{sext, word, Brn, B};
use crate::op::StopReason;
use crate::{Bus, Cpu, Flow};

/// Mnemonics owned by this group (manual spelling).
pub const MNEMONICS: &[&str] = &[
    "BISR", "CALL", "CALLA", "CALLI", "DEBUG", "DISABLE", "DSYNC", "ENABLE", "FCALL", "FCALLA",
    "FCALLI", "FRET", "ISYNC", "J", "JA", "JEQ", "JEQ.A", "JGE", "JGE.U", "JGEZ", "JGTZ", "JI",
    "JL", "JLA", "JLEZ", "JLI", "JLT", "JLT.U", "JLTZ", "JNE", "JNE.A", "JNED", "JNEI", "JNZ",
    "JNZ.A", "JNZ.T", "JZ", "JZ.A", "JZ.T", "LOOP", "LOOPU", "MFCR", "MTCR", "NOP", "RESTORE",
    "RET", "RFE", "RFM", "RSLCX", "SVLCX", "SYSCALL", "TRAPSV", "TRAPV"
];

/// Debug CSFRs used by DEBUG/RFM (V1 ch. 12: DBGSR p. 12-13, DCX p. 12-25, DBGTCR p. 12-26).
const DBGSR: u16 = 0xFD00;
const DCX: u16 = 0xFD44;
const DBGTCR: u16 = 0xFD48;
/// DBGSR.DE (bit 0) and DBGTCR.DTA (bit 0).
const DE: u32 = 1;
const DTA: u32 = 1;

/// Branch condition `x OP y` on 32-bit operands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cond {
    Eq,
    Ne,
    /// signed
    Lt,
    /// signed
    Ge,
    /// signed (JGTZ)
    Gt,
    /// signed (JLEZ)
    Le,
    LtU,
    GeU,
}

impl Cond {
    #[inline(always)]
    fn test(self, x: u32, y: u32) -> bool {
        match self {
            Cond::Eq => x == y,
            Cond::Ne => x != y,
            Cond::Lt => (x as i32) < (y as i32),
            Cond::Ge => (x as i32) >= (y as i32),
            Cond::Gt => (x as i32) > (y as i32),
            Cond::Le => (x as i32) <= (y as i32),
            Cond::LtU => x < y,
            Cond::GeU => x >= y,
        }
    }
}

/// Predecoded operations of this group. Targets of PC-relative and absolute branches are resolved at
/// decode time (`t`); `imm` is the const4 already sign- or zero-extended (`imm as i32 as u32`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CtrlOp {
    /// NOP (V2 p. 3-290), ISYNC (p. 3-114), DSYNC (p. 3-81): no architectural effect here.
    Nop,
    /// J, JA, LOOPU: `PC = t`.
    J { t: u32 },
    /// JL, JLA: `A[11] = PC + 4; PC = t`.
    Jl { t: u32 },
    /// JI: `PC = {A[a][31:1], 0}`.
    Ji { a: u8 },
    /// JLI: `A[11] = PC + 4; PC = {A[a][31:1], 0}`.
    Jli { a: u8 },
    /// `if (D[a] c D[b]) PC = t`.
    JDd { c: Cond, a: u8, b: u8, t: u32 },
    /// `if (D[a] c imm) PC = t`.
    JDi { c: Cond, a: u8, imm: i8, t: u32 },
    /// JEQ.A / JNE.A: `if ((A[a] == A[b]) != ne) PC = t`.
    JAa { ne: bool, a: u8, b: u8, t: u32 },
    /// JZ.A / JNZ.A: `if ((A[a] == 0) != ne) PC = t`.
    JAz { ne: bool, a: u8, t: u32 },
    /// JZ.T / JNZ.T: `if (D[a][n] == nz) PC = t`.
    JBit { nz: bool, a: u8, n: u8, t: u32 },
    /// JNEI (`inc`) / JNED with `D[b]`: `if (D[a] != D[b]) PC = t; D[a] = D[a] +/- 1`.
    JStepD { inc: bool, a: u8, b: u8, t: u32 },
    /// JNEI / JNED with const4.
    JStepI { inc: bool, a: u8, imm: i8, t: u32 },
    /// LOOP: `if (A[b] != 0) PC = t; A[b] = A[b] - 1`.
    Loop { b: u8, t: u32 },
    /// CALL, CALLA.
    Call { t: u32 },
    /// `CALLI A[a]`.
    Calli { a: u8 },
    /// FCALL, FCALLA (TC1.6).
    Fcall { t: u32 },
    /// `FCALLI A[a]` (TC1.6).
    Fcalli { a: u8 },
    Ret,
    /// FRET (TC1.6).
    Fret,
    Rfe,
    Rfm,
    Svlcx,
    Rslcx,
    /// BISR `const9[7:0]` / `const8`.
    Bisr { ccpn: u8 },
    Syscall { tin: u8 },
    Trapv,
    Trapsv,
    Enable,
    Disable,
    /// `DISABLE D[a]` (TC1.6).
    DisableD { a: u8 },
    /// `RESTORE D[a]` (TC1.6).
    Restore { a: u8 },
    Debug,
    /// `MFCR D[c], const16`.
    Mfcr { c: u8, off: u16 },
    /// `MTCR const16, D[a]`. PC (FE08H): on TC1.6 "if the core is not in halt a write will have no
    /// effect" (V1 p. 3-4); on TC1.3.1 the PC register is plainly rw (V131-1 p. 3-5), so the write is a
    /// control transfer.
    Mtcr { a: u8, off: u16 },
}

#[inline(always)]
fn rel(pc: u32, disp: u32, bits: u32) -> u32 {
    pc.wrapping_add(sext(disp, bits) << 1)
}

/// `{disp24[23:20], 7'b0, disp24[19:0], 1'b0}` (V2 JA p. 3-120).
#[inline(always)]
fn abs24(disp24: u32) -> u32 {
    (disp24 >> 20) << 28 | (disp24 & 0xF_FFFF) << 1
}

#[inline(always)]
fn r4(v: u32) -> u8 {
    (v & 0xF) as u8
}

pub fn decode(lo: u16, hi: u16, pc: u32, arch: Arch) -> Option<CtrlOp> {
    let tc16 = arch == Arch::Tc16;
    if lo & 1 == 0 {
        decode16(lo, pc, tc16)
    } else {
        decode32(word(lo, hi), pc, tc16)
    }
}

/// 16-bit encodings (V2 Table 1-5 formats SB, SBC, SBR, SBRN, SC, SR).
fn decode16(h: u16, pc: u32, tc16: bool) -> Option<CtrlOp> {
    use CtrlOp::*;
    use Cond::*;
    let w = h as u32;
    let f12 = r4(w >> 12); // SR op2 / SBC const4 / SBR b / SBRN n
    let f8 = r4(w >> 8); // SR a / SB.. disp4
    // SB: PC + sign_ext(disp8) * 2.
    let d8 = rel(pc, w >> 8, 8);
    // SBC/SBR/SBRN: PC + zero_ext(disp4) * 2, the second JEQ/JNE forms zero_ext(disp4 + 16) * 2.
    let d4 = pc.wrapping_add((f8 as u32) << 1);
    let d4p16 = pc.wrapping_add((f8 as u32 + 16) << 1);
    let c4 = sext(f12 as u32, 4) as i8;
    Some(match h as u8 {
        // SR 00H: NOP op2 0 (p. 3-290), FRET 7 (p. 3-105), RFE 8 (p. 3-313), RET 9 (p. 3-311),
        // DEBUG A (p. 3-78).
        0x00 => match f12 {
            0x0 => Nop,
            0x7 if tc16 => Fret,
            0x8 => Rfe,
            0x9 => Ret,
            0xA => Debug,
            _ => return None,
        },
        // JI A[a] (SR DCH op2 0, p. 3-128).
        0xDC if f12 == 0 => Ji { a: f8 },
        // J disp8 (SB 3CH, p. 3-119); CALL disp8 (SB 5CH, p. 3-63).
        0x3C => J { t: d8 },
        0x5C => Call { t: d8 },
        // JNZ D[15], disp8 (SB EEH, p. 3-141); JZ D[15], disp8 (SB 6EH, p. 3-144).
        0xEE => JDi { c: Ne, a: 15, imm: 0, t: d8 },
        0x6E => JDi { c: Eq, a: 15, imm: 0, t: d8 },
        // JEQ D[15], const4 (SBC 1EH / 9EH, p. 3-121); JNE (SBC 5EH / DEH, p. 3-136).
        0x1E => JDi { c: Eq, a: 15, imm: c4, t: d4 },
        0x9E if tc16 => JDi { c: Eq, a: 15, imm: c4, t: d4p16 },
        0x5E => JDi { c: Ne, a: 15, imm: c4, t: d4 },
        0xDE if tc16 => JDi { c: Ne, a: 15, imm: c4, t: d4p16 },
        // JEQ D[15], D[b] (SBR 3EH / BEH, p. 3-121); JNE (SBR 7EH / FEH, p. 3-136).
        0x3E => JDd { c: Eq, a: 15, b: f12, t: d4 },
        0xBE if tc16 => JDd { c: Eq, a: 15, b: f12, t: d4p16 },
        0x7E => JDd { c: Ne, a: 15, b: f12, t: d4 },
        0xFE if tc16 => JDd { c: Ne, a: 15, b: f12, t: d4p16 },
        // JGEZ (SBR CEH, p. 3-126), JGTZ (4EH, p. 3-127), JLEZ (8EH, p. 3-131), JLTZ (0EH, p. 3-135),
        // JNZ D[b] (F6H, p. 3-141), JZ D[b] (76H, p. 3-144).
        0xCE => JDi { c: Ge, a: f12, imm: 0, t: d4 },
        0x4E => JDi { c: Gt, a: f12, imm: 0, t: d4 },
        0x8E => JDi { c: Le, a: f12, imm: 0, t: d4 },
        0x0E => JDi { c: Lt, a: f12, imm: 0, t: d4 },
        0xF6 => JDi { c: Ne, a: f12, imm: 0, t: d4 },
        0x76 => JDi { c: Eq, a: f12, imm: 0, t: d4 },
        // JNZ.A A[b] (SBR 7CH, p. 3-142); JZ.A A[b] (SBR BCH, p. 3-145).
        0x7C => JAz { ne: true, a: f12, t: d4 },
        0xBC => JAz { ne: false, a: f12, t: d4 },
        // LOOP A[b], disp4 (SBR FCH, p. 3-172): PC + {27'b111..1, disp4, 0}.
        0xFC => Loop { b: f12, t: pc.wrapping_add(0xFFFF_FFE0 | (f8 as u32) << 1) },
        // JNZ.T D[15], n (SBRN AEH, p. 3-143); JZ.T D[15], n (SBRN 2EH, p. 3-146).
        0xAE => JBit { nz: true, a: 15, n: f12, t: d4 },
        0x2E => JBit { nz: false, a: 15, n: f12, t: d4 },
        // BISR const8 (SC E0H, p. 3-43).
        0xE0 => Bisr { ccpn: (w >> 8) as u8 },
        _ => return None,
    })
}

/// 32-bit encodings (V2 Table 1-6 formats B, BRC, BRN, BRR, RC, RLC, RR, SYS).
fn decode32(w: u32, pc: u32, tc16: bool) -> Option<CtrlOp> {
    use CtrlOp::*;
    use Cond::*;
    let a = r4(w >> 8);
    let b = r4(w >> 12); // BRR b / BRC const4
    let brop2 = w >> 31 != 0; // BRC/BRR/BRN op2 [31]
    let t15 = rel(pc, (w >> 16) & 0x7FFF, 15); // PC + sign_ext(disp15) * 2
    let c4s = sext(b as u32, 4) as i8;
    let c4u = b as i8;
    let d24 = B::from(w).disp24;
    let (rel24, a24) = (rel(pc, d24, 24), abs24(d24));
    Some(match w as u8 {
        // B: J (1DH, p. 3-119), JA (9DH, p. 3-120), JL (5DH, p. 3-129), JLA (DDH, p. 3-130),
        // CALL (6DH, p. 3-63), CALLA (EDH, p. 3-65), FCALL (61H, p. 3-102), FCALLA (E1H, p. 3-103).
        0x1D => J { t: rel24 },
        0x9D => J { t: a24 },
        0x5D => Jl { t: rel24 },
        0xDD => Jl { t: a24 },
        0x6D => Call { t: rel24 },
        0xED => Call { t: a24 },
        0x61 if tc16 => Fcall { t: rel24 },
        0xE1 if tc16 => Fcall { t: a24 },
        // RR 2DH, op2 [27:20]: CALLI 00H (p. 3-66), FCALLI 01H (p. 3-104), JLI 02H (p. 3-132),
        // JI 03H (p. 3-128).
        0x2D => match (w >> 20) & 0xFF {
            0x00 => Calli { a },
            0x01 if tc16 => Fcalli { a },
            0x02 => Jli { a },
            0x03 => Ji { a },
            _ => return None,
        },
        // BRC DFH / BRR 5FH: JEQ op2 0, JNE op2 1 (p. 3-121, 3-136), const4 sign-extended.
        0xDF => JDi { c: if brop2 { Ne } else { Eq }, a, imm: c4s, t: t15 },
        0x5F => JDd { c: if brop2 { Ne } else { Eq }, a, b, t: t15 },
        // BRC FFH / BRR 7FH: JGE op2 0 (sign_ext const4), JGE.U op2 1 (zero_ext) (p. 3-124).
        0xFF => if brop2 { JDi { c: GeU, a, imm: c4u, t: t15 } } else { JDi { c: Ge, a, imm: c4s, t: t15 } },
        0x7F => JDd { c: if brop2 { GeU } else { Ge }, a, b, t: t15 },
        // BRC BFH / BRR 3FH: JLT op2 0, JLT.U op2 1 (p. 3-133).
        0xBF => if brop2 { JDi { c: LtU, a, imm: c4u, t: t15 } } else { JDi { c: Lt, a, imm: c4s, t: t15 } },
        0x3F => JDd { c: if brop2 { LtU } else { Lt }, a, b, t: t15 },
        // BRC 9FH / BRR 1FH: JNEI op2 0 (p. 3-140), JNED op2 1 (p. 3-139).
        0x9F => JStepI { inc: !brop2, a, imm: c4s, t: t15 },
        0x1F => JStepD { inc: !brop2, a, b, t: t15 },
        // BRR 7DH: JEQ.A op2 0 (p. 3-123), JNE.A op2 1 (p. 3-138).
        0x7D => JAa { ne: brop2, a, b, t: t15 },
        // BRR BDH: JZ.A op2 0 (p. 3-145), JNZ.A op2 1 (p. 3-142), register A[a].
        0xBD => JAz { ne: brop2, a, t: t15 },
        // BRR FDH: LOOP A[b] op2 0 (p. 3-172), LOOPU op2 1 (p. 3-173).
        0xFD => if brop2 { J { t: t15 } } else { Loop { b, t: t15 } },
        // BRN 6FH (op1 = bits [6:0], n = {[7], [15:12]}): JZ.T op2 0 (p. 3-146), JNZ.T op2 1 (p. 3-143).
        0x6F | 0xEF => JBit { nz: brop2, a, n: Brn::from(w).n as u8, t: t15 },
        // SYS 0DH, op2 [27:22].
        0x0D => match (w >> 22) & 0x3F {
            0x00 => Nop,                   // NOP p. 3-290
            0x03 if tc16 => Fret,          // FRET p. 3-105
            0x04 => Debug,                 // DEBUG p. 3-78
            0x05 => Rfm,                   // RFM p. 3-315
            0x06 => Ret,                   // RET p. 3-311
            0x07 => Rfe,                   // RFE p. 3-313
            0x08 => Svlcx,                 // SVLCX p. 3-375
            0x09 => Rslcx,                 // RSLCX p. 3-316
            0x0C => Enable,                // ENABLE p. 3-91
            0x0D => Disable,               // DISABLE p. 3-80
            0x0E if tc16 => Restore { a }, // RESTORE p. 3-310
            0x0F if tc16 => DisableD { a }, // DISABLE D[a] p. 3-80
            0x12 | 0x13 => Nop,            // DSYNC p. 3-81, ISYNC p. 3-114
            0x14 => Trapv,                 // TRAPV p. 3-380
            0x15 => Trapsv,                // TRAPSV p. 3-379
            _ => return None,
        },
        // RC ADH, op2 [27:21]: BISR 00H (p. 3-43, const9[7:0]), SYSCALL 04H (p. 3-378, const9[7:0]).
        0xAD => match (w >> 21) & 0x7F {
            0x00 => Bisr { ccpn: (w >> 12) as u8 },
            0x04 => Syscall { tin: (w >> 12) as u8 },
            _ => return None,
        },
        // RLC: MFCR D[c], const16 (4DH, p. 3-219); MTCR const16, D[a] (CDH, p. 3-268).
        0x4D => Mfcr { c: r4(w >> 28), off: (w >> 12) as u16 },
        0xCD => Mtcr { a, off: (w >> 12) as u16 },
        _ => return None,
    })
}

pub fn ends_block(op: &CtrlOp) -> bool {
    !matches!(op, CtrlOp::Nop | CtrlOp::Mfcr { .. })
}

#[inline(always)]
fn supervisor<B: Bus>(cpu: &Cpu<B>) -> bool {
    cpu.psw & psw::IO == psw::IO_SUPERVISOR
}

#[inline(always)]
fn user1<B: Bus>(cpu: &Cpu<B>) -> bool {
    cpu.psw & psw::IO != 0
}

/// Trap raised by a failed context save/restore: the context traps themselves, FCU for memory errors.
#[inline]
fn ctx_trap(e: Trap) -> Trap {
    if e.class == 3 {
        e
    } else {
        Trap::FCU
    }
}

#[inline(always)]
fn stop(next: u32) -> Flow {
    Flow::Stop { next, why: StopReason::IrqCheck }
}

#[inline(always)]
fn set_ie<B: Bus>(cpu: &mut Cpu<B>, on: bool) {
    cpu.icr = if on { cpu.icr | icr::IE } else { cpu.icr & !icr::IE };
}

pub fn exec<B: Bus>(cpu: &mut Cpu<B>, op: &CtrlOp, pc: u32, len: u32) -> Flow {
    exec_inline(cpu, op, pc, len)
}

/// [`exec`] forced inline: the hot-instruction dispatch (`crate::op::exec_hot`) calls it with a
/// constant variant so that the match folds away.
#[inline(always)]
pub(crate) fn exec_inline<B: Bus>(cpu: &mut Cpu<B>, op: &CtrlOp, pc: u32, len: u32) -> Flow {
    use CtrlOp::*;
    let next = pc.wrapping_add(len);
    let br = |taken: bool, t: u32| if taken { Flow::Jump(t) } else { Flow::Next };
    match *op {
        Nop => Flow::Next,
        J { t } => {
            #[cfg(feature = "trace-data-stores")]
            {
                cpu.bus.call_execute(pc, t, next);
            }
            Flow::Jump(t)
        }
        Jl { t } => {
            #[cfg(feature = "trace-data-stores")]
            {
                cpu.bus.call_execute(pc, t, next);
            }
            cpu.a[11] = next;
            Flow::Jump(t)
        }
        Ji { a } => {
            let t = cpu.a[a as usize & 15] & !1;
            #[cfg(feature = "trace-data-stores")]
            {
                cpu.bus.call_execute(pc, t, next);
            }
            Flow::Jump(t)
        }
        // V2 JLI p. 3-132 RTL order: `A[11] = PC + 4; PC = {A[a][31:1], 0}` - with a = 11 the jump
        // goes to the new A[11] (CALLI/FCALLI read A[a] before writing A[11]; Unicorn agrees on both).
        Jli { a } => {
            cpu.a[11] = next;
            #[cfg(feature = "trace-data-stores")]
            {
                cpu.bus.call_execute(pc, cpu.a[a as usize & 15] & !1, next);
            }
            Flow::Jump(cpu.a[a as usize & 15] & !1)
        }
        JDd { c, a, b, t } => br(c.test(cpu.d[a as usize & 15], cpu.d[b as usize & 15]), t),
        JDi { c, a, imm, t } => br(c.test(cpu.d[a as usize & 15], imm as i32 as u32), t),
        JAa { ne, a, b, t } => br((cpu.a[a as usize & 15] == cpu.a[b as usize & 15]) != ne, t),
        JAz { ne, a, t } => br((cpu.a[a as usize & 15] == 0) != ne, t),
        JBit { nz, a, n, t } => br((cpu.d[a as usize & 15] >> n & 1 != 0) == nz, t),
        // V2 JNED p. 3-139 / JNEI p. 3-140: compare first, then the unconditional decrement/increment.
        JStepD { inc, a, b, t } => {
            let taken = cpu.d[a as usize & 15] != cpu.d[b as usize & 15];
            step(cpu, a, inc);
            br(taken, t)
        }
        JStepI { inc, a, imm, t } => {
            let taken = cpu.d[a as usize & 15] != imm as i32 as u32;
            step(cpu, a, inc);
            br(taken, t)
        }
        // V2 LOOP p. 3-172.
        Loop { b, t } => {
            // GRWP: A0/A1/A8/A9 written while PSW.GW = 0 (V1 p. 6-7)
            if 0x0303 >> b & 1 != 0 && cpu.psw & psw::GW == 0 {
                return Flow::trap(Trap::GRWP, pc);
            }
            let r = &mut cpu.a[b as usize & 15];
            let taken = *r != 0;
            *r = r.wrapping_sub(1);
            br(taken, t)
        }
        Call { t } => call(cpu, pc, next, t),
        Calli { a } => {
            let t = cpu.a[a as usize & 15] & !1;
            call(cpu, pc, next, t)
        }
        Fcall { t } => fcall(cpu, pc, len, next, t),
        Fcalli { a } => {
            let t = cpu.a[a as usize & 15] & !1;
            fcall(cpu, pc, len, next, t)
        }
        Ret => ret(cpu, pc),
        Fret => fret(cpu, pc, len),
        Rfe => rfe(cpu, pc),
        Rfm => rfm(cpu, pc, len),
        // V2 SVLCX p. 3-375: FCD after the completed save returns to the next instruction (V1 p. 6-8).
        Svlcx => match cpu.save_context(Kind::Lower) {
            Ok(false) => Flow::Next,
            Ok(true) => Flow::Trap { trap: Trap::FCD, ret: next },
            Err(e) => Flow::trap(ctx_trap(e), pc),
        },
        // V2 RSLCX p. 3-316: restores A[11], A[2..7], D[0..7], PCXI; PSW untouched.
        Rslcx => match cpu.restore_context(Kind::Lower) {
            Ok(_) => Flow::Next,
            Err(e) => Flow::trap(ctx_trap(e), pc),
        },
        // V2 BISR p. 3-43.
        Bisr { ccpn } => {
            if !supervisor(cpu) {
                return Flow::trap(Trap::PRIV, pc);
            }
            match cpu.save_context(Kind::Lower) {
                Ok(fcd) => {
                    cpu.icr = (cpu.icr & !(icr::IE | icr::CCPN)) | icr::IE | ccpn as u32;
                    if fcd {
                        Flow::Trap { trap: Trap::FCD, ret: next }
                    } else {
                        stop(next)
                    }
                }
                Err(e) => Flow::trap(ctx_trap(e), pc),
            }
        }
        // V2 SYSCALL p. 3-378 / V1 p. 6-11: TIN = const9[7:0], return address = next instruction.
        Syscall { tin } => Flow::Trap { trap: Trap::sys(tin), ret: next },
        // V2 TRAPV p. 3-380 / TRAPSV p. 3-379: synchronous, return address = this instruction (V1 §6.2.3).
        Trapv => {
            if cpu.psw & psw::V != 0 {
                Flow::trap(Trap::OVF, pc)
            } else {
                Flow::Next
            }
        }
        Trapsv => {
            if cpu.psw & psw::SV != 0 {
                Flow::trap(Trap::SOVF, pc)
            } else {
                Flow::Next
            }
        }
        // V2 ENABLE p. 3-91, DISABLE p. 3-80, RESTORE p. 3-310.
        Enable | Disable | DisableD { .. } | Restore { .. } if !user1(cpu) => Flow::trap(Trap::PRIV, pc),
        Enable => {
            set_ie(cpu, true);
            stop(next)
        }
        Disable => {
            set_ie(cpu, false);
            stop(next)
        }
        DisableD { a } => {
            cpu.d[a as usize & 15] = (cpu.icr & icr::IE != 0) as u32;
            set_ie(cpu, false);
            stop(next)
        }
        Restore { a } => {
            let on = cpu.d[a as usize & 15] & 1 != 0;
            set_ie(cpu, on);
            stop(next)
        }
        // V2 DEBUG p. 3-78: a debug event if DBGSR.DE, else NOP. The instruction retires; the caller
        // decides what the debug event does (halt, breakpoint trap, ...).
        Debug => {
            if cpu.csfr_read(DBGSR, pc) & DE != 0 {
                Flow::Stop { next, why: StopReason::Debug }
            } else {
                Flow::Next
            }
        }
        // V2 MFCR p. 3-219: any privilege level; PC reads as the address of this instruction.
        Mfcr { c, off } => {
            cpu.d[c as usize & 15] = cpu.csfr_read(off, pc);
            Flow::Next
        }
        // V2 MTCR p. 3-268.
        Mtcr { a, off } => {
            if !supervisor(cpu) {
                return Flow::trap(Trap::PRIV, pc);
            }
            let v = cpu.d[a as usize & 15];
            match off {
                crate::arch::csfr::PC => {
                    if cpu.arch != Arch::Tc16 {
                        Flow::Jump(v & !1)
                    } else {
                        Flow::Next
                    }
                }
                crate::arch::csfr::ICR => {
                    cpu.csfr_write(off, v);
                    stop(next)
                }
                _ => {
                    cpu.csfr_write(off, v);
                    Flow::Next
                }
            }
        }
    }
}

#[inline(always)]
fn step<B: Bus>(cpu: &mut Cpu<B>, a: u8, inc: bool) {
    let r = &mut cpu.d[a as usize & 15];
    *r = if inc { r.wrapping_add(1) } else { r.wrapping_sub(1) };
}

/// CALL / CALLA / CALLI (V2 p. 3-63, 3-65, 3-66).
///
/// The CSA receives the caller's PSW with CDE forced to 1 ("When the PSW is saved, the CDE bit is
/// forced to '1'", V2 p. 3-63; V1 §12.6 p. 12-9), while the callee continues with the incremented
/// call depth and CDE = 1. This is what makes the counter "restored to its previous value on
/// returns" (V1 §4.3); the RTL line order (cdc_increment before the store) read literally would save
/// the incremented count and leak one count per call/return pair. A CDO trap leaves PSW unchanged
/// (precise synchronous trap).
#[inline]
fn call<B: Bus>(cpu: &mut Cpu<B>, pc: u32, ret: u32, t: u32) -> Flow {
    if cpu.fcx & pcxi::LINK == 0 {
        return Flow::trap(Trap::FCU, pc);
    }
    let psw0 = cpu.psw;
    if psw0 & psw::CDE != 0 && cpu.cdc_increment() {
        cpu.psw = psw0;
        return Flow::trap(Trap::CDO, pc);
    }
    let live = cpu.psw | psw::CDE;
    cpu.psw = psw0 | psw::CDE;
    let fcd = match cpu.save_context(Kind::Upper) {
        Ok(f) => f,
        Err(e) => {
            cpu.psw = psw0;
            return Flow::trap(ctx_trap(e), pc);
        }
    };
    cpu.psw = live;
    #[cfg(feature = "trace-data-stores")]
    {
        cpu.bus.call_execute(pc, t, ret);
    }
    cpu.a[11] = ret;
    if fcd {
        // V1 p. 6-8: the call completes, then FCD is taken with the call target as return address.
        Flow::Trap { trap: Trap::FCD, ret: t }
    } else {
        Flow::Jump(t)
    }
}

/// RET (V2 p. 3-311; V131-2 p. 3-369): TC1.3.1/TC1.6 keep PSW[25:24], whereas TC1.3 (`1.3.0` row)
/// restores the entire PSW word from the CSA.
#[inline]
fn ret<B: Bus>(cpu: &mut Cpu<B>, pc: u32) -> Flow {
    let psw0 = cpu.psw;
    if psw0 & psw::CDE != 0 && cpu.cdc_decrement() {
        cpu.psw = psw0;
        return Flow::trap(Trap::CDU, pc);
    }
    let t = cpu.a[11] & !1;
    match cpu.restore_context(Kind::Upper) {
        Ok(p) => {
            cpu.psw = if cpu.arch == Arch::Tc13 {
                p
            } else {
                (p & !psw::RM) | (psw0 & psw::RM)
            };
            Flow::Jump(t)
        }
        Err(e) => {
            cpu.psw = psw0;
            Flow::trap(ctx_trap(e), pc)
        }
    }
}

/// RFE (V2 p. 3-313): CSU, CTYP, NEST checks in RTL order; ICR.IE/CCPN from the current PCXI; the
/// whole PSW comes from the CSA.
#[inline]
fn rfe<B: Bus>(cpu: &mut Cpu<B>, pc: u32) -> Flow {
    let x = cpu.pcxi;
    if x & pcxi::LINK == 0 {
        return Flow::trap(Trap::CSU, pc);
    }
    if x & pcxi::UL == 0 {
        return Flow::trap(Trap::CTYP, pc);
    }
    if cpu.psw & psw::CDE != 0 && !cpu.cdc_zero() {
        return Flow::trap(Trap::NEST, pc);
    }
    let t = cpu.a[11] & !1;
    match cpu.restore_context(Kind::Upper) {
        Ok(p) => {
            cpu.psw = p;
            restore_icr(cpu, x);
            stop(t)
        }
        Err(e) => Flow::trap(ctx_trap(e), pc),
    }
}

/// `ICR.IE = PCXI.PIE; ICR.CCPN = PCXI.PCPN`.
#[inline]
fn restore_icr<B: Bus>(cpu: &mut Cpu<B>, x: u32) {
    let ie = if x & pcxi::PIE != 0 { icr::IE } else { 0 };
    cpu.icr = (cpu.icr & !(icr::IE | icr::CCPN)) | ie | (x >> pcxi::PCPN_SHIFT) & icr::CCPN;
}

/// RFM (V2 p. 3-315; V131-2 p. 3-373): NOP unless DBGSR.DE; else PC = A[11], ICR from PCXI, then
/// {PCXI, PSW, A[10], A[11]} from the debug context area at DCX[31:6] (V1 p. 12-25), DBGTCR.DTA = 0.
fn rfm<B: Bus>(cpu: &mut Cpu<B>, pc: u32, len: u32) -> Flow {
    if !supervisor(cpu) {
        return Flow::trap(Trap::PRIV, pc);
    }
    if cpu.csfr_read(DBGSR, pc) & DE == 0 {
        return Flow::Next;
    }
    let ea = cpu.csfr_read(DCX, pc) & !0x3F;
    let mut w = [0u32; 4];
    for (i, v) in w.iter_mut().enumerate() {
        match cpu.ld32(ea.wrapping_add(4 * i as u32)) {
            Ok(x) => *v = x,
            Err(e) => return Flow::fault(e, pc, len),
        }
    }
    let t = cpu.a[11] & !1;
    let x = cpu.pcxi;
    restore_icr(cpu, x);
    cpu.pcxi = w[0];
    cpu.psw = w[1];
    cpu.a[10] = w[2];
    cpu.a[11] = w[3];
    if cpu.arch != Arch::Tc13 {
        let dbgtcr = cpu.csfr_read(DBGTCR, pc);
        cpu.csfr_write(DBGTCR, dbgtcr & !DTA);
    }
    stop(t)
}

/// FCALL / FCALLA / FCALLI (V2 p. 3-102..3-104): push A[11] at A[10] - 4. Address-register stores
/// must be word aligned (V1 Table 2-1 p. 2-4), else ALN.
#[inline]
fn fcall<B: Bus>(cpu: &mut Cpu<B>, pc: u32, len: u32, next: u32, t: u32) -> Flow {
    let ea = cpu.a[10].wrapping_sub(4);
    if ea & 3 != 0 {
        return Flow::trap(Trap::ALN, pc);
    }
    let r = cpu.st32(ea, cpu.a[11]);
    if let Err(e) = r {
        if !e.is_async() {
            return Flow::fault(e, pc, len);
        }
    }
    cpu.a[11] = next;
    cpu.a[10] = ea;
    match r {
        // An asynchronous store error is reported after the instruction completed: the return
        // address is the next instruction that would have executed, the call target.
        Err(e) => Flow::Trap { trap: e, ret: t },
        Ok(()) => {
            #[cfg(feature = "trace-data-stores")]
            {
                cpu.bus.call_execute(pc, t, next);
            }
            Flow::Jump(t)
        }
    }
}

/// FRET (V2 p. 3-105): PC = A[11]; A[11] = M(A[10]); A[10] += 4. Word-aligned load (V1 Table 2-1).
#[inline]
fn fret<B: Bus>(cpu: &mut Cpu<B>, pc: u32, len: u32) -> Flow {
    let ea = cpu.a[10];
    if ea & 3 != 0 {
        return Flow::trap(Trap::ALN, pc);
    }
    match cpu.ld32(ea) {
        Ok(v) => {
            let t = cpu.a[11] & !1;
            cpu.a[11] = v;
            cpu.a[10] = ea.wrapping_add(4);
            Flow::Jump(t)
        }
        Err(e) => Flow::fault(e, pc, len),
    }
}

#[cfg(test)]
mod tests {
    use super::{CtrlOp as O, *};
    use crate::testutil::{oracle, *};
    use crate::{Exit, Memory, NullBus};

    /// Address of the instruction under test (inside CODE, below the trap table at CODE + 0x2000).
    const AT: u32 = CODE + 0x1000;

    // ---- encoders: V2 Table 1-5 (16-bit) / Table 1-6 (32-bit) field positions ---------------------

    fn b(op1: u32, disp24: u32) -> u32 {
        op1 | (disp24 >> 16 & 0xFF) << 8 | (disp24 & 0xFFFF) << 16
    }
    /// BRC and BRR share the layout (the [15:12] field is const4 or b).
    fn br(op1: u32, op2: u32, a: u32, f12: u32, disp15: u32) -> u32 {
        op1 | a << 8 | f12 << 12 | (disp15 & 0x7FFF) << 16 | op2 << 31
    }
    fn brn(op2: u32, a: u32, n: u32, disp15: u32) -> u32 {
        0x6F | (n >> 4) << 7 | a << 8 | (n & 15) << 12 | (disp15 & 0x7FFF) << 16 | op2 << 31
    }
    fn rr(op2: u32, a: u32) -> u32 {
        0x2D | a << 8 | op2 << 20
    }
    fn sys(op2: u32, a: u32) -> u32 {
        0x0D | a << 8 | op2 << 22
    }
    fn rc(op2: u32, const9: u32) -> u32 {
        0xAD | const9 << 12 | op2 << 21
    }
    fn rlc(op1: u32, c: u32, const16: u32, a: u32) -> u32 {
        op1 | a << 8 | const16 << 12 | c << 28
    }
    /// SB / SC: [15:8]; SBC, SBR, SBRN, SR: [15:12] and [11:8].
    fn s8(op1: u16, v8: u16) -> u16 {
        op1 | v8 << 8
    }
    fn s44(op1: u16, hi: u16, lo: u16) -> u16 {
        op1 | lo << 8 | hi << 12
    }

    fn w32(w: u32) -> Vec<u8> {
        w.to_le_bytes().to_vec()
    }
    fn w16(h: u16) -> Vec<u8> {
        h.to_le_bytes().to_vec()
    }

    /// Native core with `code` at [`AT`], `f` applied, `n` instructions executed.
    fn exec_arch(code: &[u8], n: u64, arch: Arch, f: impl FnOnce(&mut Cpu<NullBus>)) -> (Cpu<NullBus>, Exit) {
        let mut c = cpu_with(&[], arch);
        c.write_bytes(AT, code).unwrap();
        c.pc = AT;
        f(&mut c);
        let e = c.run(n);
        (c, e)
    }
    fn one(code: &[u8], f: impl FnOnce(&mut Cpu<NullBus>)) -> Cpu<NullBus> {
        exec_arch(code, 1, Arch::Tc16, f).0
    }
    fn vector(class: u32) -> u32 {
        TRAP_BASE | class << 5
    }
    fn word_at(c: &Cpu<NullBus>, ea: u32) -> u32 {
        let mut b = [0u8; 4];
        assert!(c.read_bytes(ea, &mut b));
        u32::from_le_bytes(b)
    }

    // ---- decode ---------------------------------------------------------------------------------

    /// Firmware words decoded by rizin 0.9.1 (capstone, an independent decoder) from the SIMOS18.1
    /// factory image, with the address they were found at, versus our decoder.
    #[test]
    fn decode_matches_independent_disassembler() {
        use Cond::*;
        let cases: &[(u32, u32, O)] = &[
            (0x801202c0, 0x00e0, O::Bisr { ccpn: 0 }),
            (0x80123fd8, 0x000014ad, O::Bisr { ccpn: 1 }),
            (0x80120734, 0xa45c, O::Call { t: 0x8012067c }),
            (0x80100268, 0xa602fc6d, O::Call { t: 0x80094e6c }),
            (0x8010d5e0, 0xfdbaff6d, O::Call { t: 0x8010d154 }),
            (0x801216e0, 0xd00101ed, O::Call { t: 0x3a002 }),
            (0x80101c62, 0x00000f2d, O::Calli { a: 15 }),
            (0x801244a0, 0xa300, O::Debug),
            (0x8012829c, 0x0340000d, O::Disable),
            (0x80108512, 0x0480000d, O::Nop),
            (0x801282f6, 0x0300000d, O::Enable),
            (0x80121a90, 0xd0010261, O::Fcall { t: 0x8017ba92 }),
            (0x80121620, 0xd00101e1, O::Fcall { t: 0x3a002 }),
            (0x80120890, 0x7500, O::Fret),
            (0x8010851a, 0x04c0000d, O::Nop),
            (0x8010019e, 0x083c, O::J { t: 0x801001ae }),
            (0x80100926, 0xfcb7ff1d, O::J { t: 0x80100294 }),
            (0x80121c60, 0xd001029d, O::J { t: 0x5a002 }),
            (0x80100194, 0x161e, O::JDi { c: Eq, a: 15, imm: 1, t: 0x801001a0 }),
            (0x801001c6, 0x000b16df, O::JDi { c: Eq, a: 6, imm: 1, t: 0x801001dc }),
            (0x80101860, 0x00210f5f, O::JDd { c: Eq, a: 15, b: 0, t: 0x801018a2 }),
            (0x80101c6e, 0x0010fc7d, O::JAa { ne: false, a: 12, b: 15, t: 0x80101c8e }),
            (0x80100222, 0x0006207f, O::JDd { c: Ge, a: 0, b: 2, t: 0x8010022e }),
            (0x801001c2, 0x805ef87f, O::JDd { c: GeU, a: 8, b: 15, t: 0x8010027e }),
            (0x80121500, 0x01ce, O::JDi { c: Ge, a: 0, imm: 0, t: 0x80121502 }),
            (0x8011d4b0, 0x1e4e, O::JDi { c: Gt, a: 1, imm: 0, t: 0x8011d4cc }),
            (0x80101ca8, 0x0fdc, O::Ji { a: 15 }),
            (0x8020bc42, 0x0032002d, O::Ji { a: 0 }),
            (0x80121a50, 0xd001025d, O::Jl { t: 0x8017ba52 }),
            (0x801215e0, 0xd00101dd, O::Jl { t: 0x3a002 }),
            (0x8011d408, 0x068e, O::JDi { c: Le, a: 0, imm: 0, t: 0x8011d414 }),
            (0x80101a8a, 0x7db6f93f, O::JDd { c: Lt, a: 9, b: 15, t: 0x801015f6 }),
            (0x80101a4e, 0x80070b3f, O::JDd { c: LtU, a: 11, b: 0, t: 0x80101a5c }),
            (0x801094d6, 0x240e, O::JDi { c: Lt, a: 2, imm: 0, t: 0x801094de }),
            (0x80100198, 0x3b5e, O::JDi { c: Ne, a: 15, imm: 3, t: 0x801001ae }),
            (0x8010093c, 0x81511fdf, O::JDi { c: Ne, a: 15, imm: 1, t: 0x80100bde }),
            (0x80101c8a, 0x8004fc7d, O::JAa { ne: true, a: 12, b: 15, t: 0x80101c92 }),
            (0x80120e30, 0xd001011f, O::JStepD { inc: false, a: 1, b: 0, t: 0x8011ae32 }),
            (0x801eb446, 0xfeeb0f9f, O::JStepI { inc: false, a: 15, imm: 0, t: 0x801eb21c }),
            (0x80200a24, 0x4e3f2f1f, O::JStepD { inc: true, a: 15, b: 2, t: 0x801fa6a2 }),
            (0x801001fe, 0x13f6, O::JDi { c: Ne, a: 1, imm: 0, t: 0x80100204 }),
            (0x801015ac, 0x08ee, O::JDi { c: Ne, a: 15, imm: 0, t: 0x801015bc }),
            (0x8010204c, 0x247c, O::JAz { ne: true, a: 2, t: 0x80102054 }),
            (0x8011168c, 0x80140fbd, O::JAz { ne: true, a: 15, t: 0x801116b4 }),
            (0x8010107c, 0x06ae, O::JBit { nz: true, a: 15, n: 0, t: 0x80101088 }),
            (0x80101116, 0x80070fef, O::JBit { nz: true, a: 15, n: 16, t: 0x80101124 }),
            (0x801002ae, 0x8576, O::JDi { c: Eq, a: 8, imm: 0, t: 0x801002b8 }),
            (0x801001a6, 0x046e, O::JDi { c: Eq, a: 15, imm: 0, t: 0x801001ae }),
            (0x80101504, 0x29bc, O::JAz { ne: false, a: 2, t: 0x80101516 }),
            (0x80101c44, 0x002f0cbd, O::JAz { ne: false, a: 12, t: 0x80101ca2 }),
            (0x80104da8, 0x672e, O::JBit { nz: false, a: 15, n: 6, t: 0x80104db6 }),
            (0x8010e0e8, 0x00031fef, O::JBit { nz: false, a: 15, n: 17, t: 0x8010e0ee }),
            (0x80100290, 0x2ffc, O::Loop { b: 2, t: 0x8010028e }),
            (0x801001b2, 0x7fef20fd, O::Loop { b: 2, t: 0x80100190 }),
            (0x80122fb0, 0xd00107fd, O::J { t: 0x8011cfb2 }),
            (0x8010edce, 0x0fe0004d, O::Mfcr { c: 0, off: 0xfe00 }),
            (0x80108850, 0xffe0004d, O::Mfcr { c: 15, off: 0xfe00 }),
            (0x80108516, 0x09044fcd, O::Mtcr { a: 15, off: 0x9044 }),
            (0x8010000a, 0x0000, O::Nop),
            (0x80122dc8, 0x0000000d, O::Nop),
            (0x80100008, 0x9000, O::Ret),
            (0x80121e74, 0x8000, O::Rfe),
            (0x80108478, 0x0240000d, O::Rslcx),
            (0x80108464, 0x0200000d, O::Svlcx),
            (0x80201820, 0x808da0ad, O::Syscall { tin: 0xda }),
        ];
        for &(pc, w, ref want) in cases {
            let got = decode(w as u16, (w >> 16) as u16, pc, Arch::Tc16);
            assert_eq!(got.as_ref(), Some(want), "{w:#010x} at {pc:#x}");
        }
    }

    /// Every encoding of V2 (86) decodes on TC1.6; the 11 TC1.6-only ones are invalid on TC1.3.1 and
    /// the other 75 decode identically (V131-2 encoding boxes).
    #[test]
    fn decode_all_86_encodings_and_tc131_subset() {
        let enc32: &[(u32, bool)] = &[
            (rc(0, 5), true),                // BISR RC
            (b(0x6D, 4), true),              // CALL B
            (b(0xED, 4), true),              // CALLA
            (rr(0, 3), true),                // CALLI
            (sys(4, 0), true),               // DEBUG SYS
            (sys(0x0D, 0), true),            // DISABLE
            (sys(0x0F, 3), false),           // DISABLE D[a]
            (sys(0x12, 0), true),            // DSYNC
            (sys(0x0C, 0), true),            // ENABLE
            (b(0x61, 4), false),             // FCALL
            (b(0xE1, 4), false),             // FCALLA
            (rr(1, 3), false),               // FCALLI
            (sys(3, 0), false),              // FRET SYS
            (sys(0x13, 0), true),            // ISYNC
            (b(0x1D, 4), true),              // J B
            (b(0x9D, 4), true),              // JA
            (br(0xDF, 0, 1, 2, 3), true),    // JEQ BRC
            (br(0x5F, 0, 1, 2, 3), true),    // JEQ BRR
            (br(0x7D, 0, 1, 2, 3), true),    // JEQ.A
            (br(0xFF, 0, 1, 2, 3), true),    // JGE BRC
            (br(0x7F, 0, 1, 2, 3), true),    // JGE BRR
            (br(0xFF, 1, 1, 2, 3), true),    // JGE.U BRC
            (br(0x7F, 1, 1, 2, 3), true),    // JGE.U BRR
            (rr(3, 3), true),                // JI RR
            (b(0x5D, 4), true),              // JL
            (b(0xDD, 4), true),              // JLA
            (rr(2, 3), true),                // JLI
            (br(0xBF, 0, 1, 2, 3), true),    // JLT BRC
            (br(0x3F, 0, 1, 2, 3), true),    // JLT BRR
            (br(0xBF, 1, 1, 2, 3), true),    // JLT.U BRC
            (br(0x3F, 1, 1, 2, 3), true),    // JLT.U BRR
            (br(0xDF, 1, 1, 2, 3), true),    // JNE BRC
            (br(0x5F, 1, 1, 2, 3), true),    // JNE BRR
            (br(0x7D, 1, 1, 2, 3), true),    // JNE.A
            (br(0x9F, 1, 1, 2, 3), true),    // JNED BRC
            (br(0x1F, 1, 1, 2, 3), true),    // JNED BRR
            (br(0x9F, 0, 1, 2, 3), true),    // JNEI BRC
            (br(0x1F, 0, 1, 2, 3), true),    // JNEI BRR
            (br(0xBD, 1, 1, 0, 3), true),    // JNZ.A BRR
            (brn(1, 1, 17, 3), true),        // JNZ.T BRN
            (br(0xBD, 0, 1, 0, 3), true),    // JZ.A BRR
            (brn(0, 1, 17, 3), true),        // JZ.T BRN
            (br(0xFD, 0, 0, 2, 3), true),    // LOOP BRR
            (br(0xFD, 1, 0, 0, 3), true),    // LOOPU
            (rlc(0x4D, 3, 0xFE04, 0), true), // MFCR
            (rlc(0xCD, 0, 0xFE2C, 3), true), // MTCR
            (sys(0, 0), true),               // NOP SYS
            (sys(0x0E, 3), false),           // RESTORE
            (sys(6, 0), true),               // RET SYS
            (sys(7, 0), true),               // RFE SYS
            (sys(5, 0), true),               // RFM
            (sys(9, 0), true),               // RSLCX
            (sys(8, 0), true),               // SVLCX
            (rc(4, 5), true),                // SYSCALL
            (sys(0x15, 0), true),            // TRAPSV
            (sys(0x14, 0), true),            // TRAPV
        ];
        let enc16: &[(u16, bool)] = &[
            (s8(0xE0, 5), true),         // BISR SC
            (s8(0x5C, 4), true),         // CALL SB
            (s44(0x00, 0xA, 0), true),   // DEBUG SR
            (s44(0x00, 7, 0), false),    // FRET SR
            (s8(0x3C, 4), true),         // J SB
            (s44(0x1E, 2, 3), true),     // JEQ SBC
            (s44(0x9E, 2, 3), false),    // JEQ SBC +16
            (s44(0x3E, 2, 3), true),     // JEQ SBR
            (s44(0xBE, 2, 3), false),    // JEQ SBR +16
            (s44(0xCE, 2, 3), true),     // JGEZ
            (s44(0x4E, 2, 3), true),     // JGTZ
            (s44(0xDC, 0, 3), true),     // JI SR
            (s44(0x8E, 2, 3), true),     // JLEZ
            (s44(0x0E, 2, 3), true),     // JLTZ
            (s44(0x5E, 2, 3), true),     // JNE SBC
            (s44(0xDE, 2, 3), false),    // JNE SBC +16
            (s44(0x7E, 2, 3), true),     // JNE SBR
            (s44(0xFE, 2, 3), false),    // JNE SBR +16
            (s8(0xEE, 4), true),         // JNZ SB
            (s44(0xF6, 2, 3), true),     // JNZ SBR
            (s44(0x7C, 2, 3), true),     // JNZ.A SBR
            (s44(0xAE, 2, 3), true),     // JNZ.T SBRN
            (s8(0x6E, 4), true),         // JZ SB
            (s44(0x76, 2, 3), true),     // JZ SBR
            (s44(0xBC, 2, 3), true),     // JZ.A SBR
            (s44(0x2E, 2, 3), true),     // JZ.T SBRN
            (s44(0xFC, 2, 3), true),     // LOOP SBR
            (s44(0x00, 0, 0), true),     // NOP SR
            (s44(0x00, 9, 0), true),     // RET SR
            (s44(0x00, 8, 0), true),     // RFE SR
        ];
        assert_eq!(enc32.len() + enc16.len(), 86);
        assert_eq!(enc32.iter().filter(|e| e.1).count() + enc16.iter().filter(|e| e.1).count(), 75);
        let all = enc32.iter().map(|&(w, t)| (w, t)).chain(enc16.iter().map(|&(h, t)| (h as u32, t)));
        for (w, in131) in all {
            let (lo, hi) = (w as u16, (w >> 16) as u16);
            let d16 = decode(lo, hi, AT, Arch::Tc16).unwrap_or_else(|| panic!("{w:#x} not decoded"));
            assert_eq!(decode(lo, hi, AT, Arch::Tc131), in131.then_some(d16), "{w:#x} on TC1.3.1");
        }
        // reserved op2 values of the shared op1 spaces are not claimed
        for w in [sys(1, 0), sys(0x10, 0), sys(0x3F, 0), rr(4, 0), rc(1, 0), rc(5, 0)] {
            assert_eq!(decode(w as u16, (w >> 16) as u16, AT, Arch::Tc16), None, "{w:#x}");
        }
        for op2 in [1u16, 6, 0xB, 0xF] {
            assert_eq!(decode(s44(0x00, op2, 0), 0, AT, Arch::Tc16), None);
        }
        assert_eq!(decode(s44(0xDC, 1, 0), 0, AT, Arch::Tc16), None);
    }

    // ---- branches (manual RTL) ------------------------------------------------------------------

    /// One case: encoding, register setup, expected PC (and nothing else changes except as noted).
    fn branch(code: Vec<u8>, set: impl FnOnce(&mut Cpu<NullBus>), want: u32) -> Cpu<NullBus> {
        let c = one(&code, set);
        assert_eq!(c.pc, want, "{:x?}", code);
        assert_eq!(c.icount, 1);
        c
    }

    #[test]
    fn unconditional_jumps() {
        let n = AT + 4;
        // J disp24 (p. 3-119): PC + sign_ext(disp24) * 2, backwards across the region start too.
        branch(w32(b(0x1D, 0x40)), |_| {}, AT + 0x80);
        branch(w32(b(0x1D, 0xFF_F800)), |_| {}, AT - 0x1000);
        // J disp8 (SB): sign_ext(disp8) * 2
        branch(w16(s8(0x3C, 0x7F)), |_| {}, AT + 0xFE);
        branch(w16(s8(0x3C, 0x80)), |_| {}, AT - 0x100);
        // JA (p. 3-120): {disp24[23:20], 7'b0, disp24[19:0], 0}
        branch(w32(b(0x9D, 0xC0_0123)), |_| {}, 0xC000_0246);
        // JL / JLA (p. 3-129/3-130): A[11] = PC + 4
        let c = branch(w32(b(0x5D, 0x10)), |_| {}, AT + 0x20);
        assert_eq!(c.a[11], n);
        let c = branch(w32(b(0xDD, 0xC0_0800)), |_| {}, 0xC000_1000);
        assert_eq!(c.a[11], n);
        // JI RR / SR (p. 3-128): bit 0 cleared
        branch(w32(rr(3, 4)), |c| c.a[4] = CODE + 0x1235, CODE + 0x1234);
        branch(w16(s44(0xDC, 0, 4)), |c| c.a[4] = CODE + 0x0101, CODE + 0x0100);
        // JLI (p. 3-132): A[11] is written first, so JLI A[11] continues at PC + 4
        let c = branch(w32(rr(2, 4)), |c| c.a[4] = CODE + 0x0301, CODE + 0x0300);
        assert_eq!(c.a[11], n);
        branch(w32(rr(2, 11)), |c| c.a[11] = CODE + 0x0301, n);
        // LOOPU (p. 3-173)
        branch(w32(br(0xFD, 1, 0, 0, 0x7FF0)), |_| {}, AT - 0x20);
    }

    #[test]
    fn compare_branches_32bit() {
        let t = AT + 0x40; // disp15 = 0x20
        let nt = AT + 4;
        let neg = (-1i32) as u32;
        // JEQ BRC (p. 3-121): const4 sign-extended (0xF -> -1)
        branch(w32(br(0xDF, 0, 3, 0xF, 0x20)), |c| c.d[3] = neg, t);
        branch(w32(br(0xDF, 0, 3, 0xF, 0x20)), |c| c.d[3] = 15, nt);
        // JNE BRC
        branch(w32(br(0xDF, 1, 3, 0x7, 0x20)), |c| c.d[3] = 7, nt);
        branch(w32(br(0xDF, 1, 3, 0x7, 0x20)), |c| c.d[3] = 8, t);
        // JEQ / JNE BRR
        branch(w32(br(0x5F, 0, 3, 4, 0x20)), |c| (c.d[3], c.d[4]) = (9, 9), t);
        branch(w32(br(0x5F, 1, 3, 4, 0x20)), |c| (c.d[3], c.d[4]) = (9, 9), nt);
        // JGE BRC signed, sign_ext(const4): -2 >= -8 taken; JGE.U zero_ext: 0xFFFFFFFE >= 8 taken
        branch(w32(br(0xFF, 0, 1, 0x8, 0x20)), |c| c.d[1] = (-2i32) as u32, t);
        branch(w32(br(0xFF, 0, 1, 0x7, 0x20)), |c| c.d[1] = (-2i32) as u32, nt);
        branch(w32(br(0xFF, 1, 1, 0xF, 0x20)), |c| c.d[1] = 14, nt);
        branch(w32(br(0xFF, 1, 1, 0xF, 0x20)), |c| c.d[1] = (-2i32) as u32, t);
        // JGE / JGE.U BRR
        branch(w32(br(0x7F, 0, 1, 2, 0x20)), |c| (c.d[1], c.d[2]) = (neg, 1), nt);
        branch(w32(br(0x7F, 1, 1, 2, 0x20)), |c| (c.d[1], c.d[2]) = (neg, 1), t);
        // JLT / JLT.U BRC (p. 3-133)
        branch(w32(br(0xBF, 0, 1, 0x8, 0x20)), |c| c.d[1] = (-9i32) as u32, t);
        branch(w32(br(0xBF, 0, 1, 0x8, 0x20)), |c| c.d[1] = (-8i32) as u32, nt);
        branch(w32(br(0xBF, 1, 1, 0x8, 0x20)), |c| c.d[1] = 7, t);
        branch(w32(br(0xBF, 1, 1, 0x8, 0x20)), |c| c.d[1] = (-9i32) as u32, nt);
        // JLT / JLT.U BRR
        branch(w32(br(0x3F, 0, 1, 2, 0x20)), |c| (c.d[1], c.d[2]) = (0x8000_0000, 0x7FFF_FFFF), t);
        branch(w32(br(0x3F, 1, 1, 2, 0x20)), |c| (c.d[1], c.d[2]) = (0x8000_0000, 0x7FFF_FFFF), nt);
        // JEQ.A / JNE.A (p. 3-123, 3-138)
        branch(w32(br(0x7D, 0, 5, 6, 0x20)), |c| (c.a[5], c.a[6]) = (0x1234, 0x1234), t);
        branch(w32(br(0x7D, 1, 5, 6, 0x20)), |c| (c.a[5], c.a[6]) = (0x1234, 0x1234), nt);
        // JZ.A / JNZ.A BRR (A[a], p. 3-145/3-142)
        branch(w32(br(0xBD, 0, 5, 0, 0x20)), |c| c.a[5] = 0, t);
        branch(w32(br(0xBD, 1, 5, 0, 0x20)), |c| c.a[5] = 0, nt);
        // disp15 negative: sign_ext(0x4000) * 2 = -0x8000 (outside CODE: no fetch after the budget)
        branch(w32(br(0xBD, 1, 5, 0, 0x4000)), |c| c.a[5] = 1, AT.wrapping_sub(0x8000));
        // JZ.T / JNZ.T BRN with n[4] in bit 7 (p. 3-146/3-143)
        branch(w32(brn(0, 2, 31, 0x20)), |c| c.d[2] = 0x7FFF_FFFF, t);
        branch(w32(brn(1, 2, 31, 0x20)), |c| c.d[2] = 0x7FFF_FFFF, nt);
        branch(w32(brn(1, 2, 16, 0x20)), |c| c.d[2] = 0x0001_0000, t);
    }

    #[test]
    fn step_branches_and_loop() {
        let t = AT + 0x40;
        let nt = AT + 4;
        // JNED BRC (p. 3-139): compare with the old value, decrement unconditionally
        let c = branch(w32(br(0x9F, 1, 1, 0, 0x20)), |c| c.d[1] = 0, nt);
        assert_eq!(c.d[1], u32::MAX);
        let c = branch(w32(br(0x9F, 1, 1, 0, 0x20)), |c| c.d[1] = 5, t);
        assert_eq!(c.d[1], 4);
        // JNEI BRC (p. 3-140): sign_ext(const4), increment wraps
        let c = branch(w32(br(0x9F, 0, 1, 0xF, 0x20)), |c| c.d[1] = u32::MAX, nt);
        assert_eq!(c.d[1], 0);
        // JNED / JNEI BRR; with a == b the compare sees equal values
        let c = branch(w32(br(0x1F, 1, 1, 2, 0x20)), |c| (c.d[1], c.d[2]) = (3, 4), t);
        assert_eq!(c.d[1], 2);
        let c = branch(w32(br(0x1F, 0, 1, 1, 0x20)), |c| c.d[1] = 3, nt);
        assert_eq!(c.d[1], 4);
        let c = branch(w32(br(0x1F, 0, 1, 2, 0x20)), |c| (c.d[1], c.d[2]) = (7, 8), t);
        assert_eq!(c.d[1], 8);
        // LOOP BRR (p. 3-172): A[b], decrement unconditionally
        let c = branch(w32(br(0xFD, 0, 0, 6, 0x7FF0)), |c| c.a[6] = 2, AT - 0x20);
        assert_eq!(c.a[6], 1);
        let c = branch(w32(br(0xFD, 0, 0, 6, 0x7FF0)), |c| c.a[6] = 0, nt);
        assert_eq!(c.a[6], u32::MAX);
        // LOOP SBR: PC + {27 ones, disp4, 0}: disp4 = 0 -> -32, disp4 = 15 -> -2
        let c = branch(w16(s44(0xFC, 6, 0)), |c| c.a[6] = 1, AT - 32);
        assert_eq!(c.a[6], 0);
        branch(w16(s44(0xFC, 6, 15)), |c| c.a[6] = 1, AT - 2);
        branch(w16(s44(0xFC, 6, 15)), |c| c.a[6] = 0, AT + 2);
    }

    /// LOOP modifies A[b]: on a global address register (A0/A1/A8/A9) with PSW.GW = 0 that is a GRWP
    /// trap (class 1 TIN 7, V1 p. 6-7), synchronous, A[b] and PC unchanged.
    #[test]
    fn loop_on_global_register_is_grwp() {
        for insn in [w32(br(0xFD, 0, 0, 8, 0x7FF0)), w16(s44(0xFC, 8, 0))] {
            let c = one(&insn, |c| (c.psw, c.a[8]) = (psw::RESET & !psw::GW, 2));
            assert_eq!((c.pc, c.a[11], c.d[15], c.a[8]), (vector(1), AT, 7, 2), "{insn:02x?}");
            let c = one(&insn, |c| (c.psw, c.a[8]) = (psw::RESET, 2));
            assert_eq!(c.a[8], 1, "{insn:02x?}: PSW.GW = 1");
        }
    }

    #[test]
    fn compare_branches_16bit() {
        let nt = AT + 2;
        let t = |disp4: u32| AT + 2 * disp4;
        let neg = (-1i32) as u32;
        // JEQ D[15], const4 (SBC 1EH: zero_ext(disp4) * 2; 9EH: zero_ext(disp4 + 16) * 2)
        branch(w16(s44(0x1E, 0xF, 3)), |c| c.d[15] = neg, t(3));
        branch(w16(s44(0x9E, 0xF, 3)), |c| c.d[15] = neg, t(19));
        branch(w16(s44(0x9E, 0xF, 3)), |c| c.d[15] = 15, nt);
        // JNE D[15], const4 (5EH / DEH)
        branch(w16(s44(0x5E, 2, 15)), |c| c.d[15] = 3, t(15));
        branch(w16(s44(0xDE, 2, 15)), |c| c.d[15] = 3, t(31));
        branch(w16(s44(0xDE, 2, 15)), |c| c.d[15] = 2, nt);
        // JEQ / JNE D[15], D[b] (3EH / BEH, 7EH / FEH)
        branch(w16(s44(0x3E, 4, 1)), |c| (c.d[15], c.d[4]) = (5, 5), t(1));
        branch(w16(s44(0xBE, 4, 1)), |c| (c.d[15], c.d[4]) = (5, 5), t(17));
        branch(w16(s44(0x7E, 4, 1)), |c| (c.d[15], c.d[4]) = (5, 5), nt);
        branch(w16(s44(0xFE, 4, 1)), |c| (c.d[15], c.d[4]) = (5, 6), t(17));
        // JGEZ / JGTZ / JLEZ / JLTZ D[b] (signed against 0)
        branch(w16(s44(0xCE, 4, 2)), |c| c.d[4] = 0, t(2));
        branch(w16(s44(0xCE, 4, 2)), |c| c.d[4] = 0x8000_0000, nt);
        branch(w16(s44(0x4E, 4, 2)), |c| c.d[4] = 0, nt);
        branch(w16(s44(0x4E, 4, 2)), |c| c.d[4] = 0x7FFF_FFFF, t(2));
        branch(w16(s44(0x8E, 4, 2)), |c| c.d[4] = 0, t(2));
        branch(w16(s44(0x8E, 4, 2)), |c| c.d[4] = 1, nt);
        branch(w16(s44(0x0E, 4, 2)), |c| c.d[4] = neg, t(2));
        branch(w16(s44(0x0E, 4, 2)), |c| c.d[4] = 0, nt);
        // JZ / JNZ D[b] (SBR 76H / F6H) and D[15], disp8 (SB 6EH / EEH, sign_ext(disp8))
        branch(w16(s44(0x76, 4, 2)), |c| c.d[4] = 0, t(2));
        branch(w16(s44(0xF6, 4, 2)), |c| c.d[4] = 0, nt);
        branch(w16(s8(0x6E, 0xF0)), |c| c.d[15] = 0, AT - 0x20);
        branch(w16(s8(0xEE, 0x10)), |c| c.d[15] = 1, AT + 0x20);
        branch(w16(s8(0xEE, 0x10)), |c| c.d[15] = 0, nt);
        // JZ.A / JNZ.A A[b] (BCH / 7CH)
        branch(w16(s44(0xBC, 4, 2)), |c| c.a[4] = 0, t(2));
        branch(w16(s44(0x7C, 4, 2)), |c| c.a[4] = 0, nt);
        // JZ.T / JNZ.T D[15], n (2EH / AEH)
        branch(w16(s44(0x2E, 15, 2)), |c| c.d[15] = 0x7FFF, t(2));
        branch(w16(s44(0xAE, 15, 2)), |c| c.d[15] = 0x7FFF, nt);
        branch(w16(s44(0xAE, 3, 2)), |c| c.d[15] = 8, t(2));
    }

    // ---- calls and returns ------------------------------------------------------------------------

    fn csa(i: u32) -> u32 {
        CSA_BASE + 64 * i
    }

    #[test]
    fn call_saves_upper_context_and_counts_depth() {
        // CALL disp24 (p. 3-63)
        let c = one(&w32(b(0x6D, 0x10)), |c| {
            c.psw = 0x0000_0B80 | psw::C | 5; // CDE = 1, 6-bit counter at 5
            c.icr = icr::IE | 0x22;
            c.pcxi = 0x0030_0000; // bits 21:20 preserved
            for i in 0..16 {
                c.d[i] = 0x100 + i as u32;
                c.a[i] = 0x200 + i as u32;
            }
            c.a[10] = DATA + 0x6000;
        });
        assert_eq!(c.pc, AT + 0x20);
        assert_eq!(c.a[11], AT + 4);
        assert_eq!(c.psw, 0x0000_0B80 | psw::C | 6, "live PSW: count + 1, CDE = 1");
        assert_eq!(c.pcxi, 0x22 << 24 | pcxi::PIE | pcxi::UL | 0x0030_0000 | link(csa(0)));
        assert_eq!(c.fcx, link(csa(1)));
        let saved: Vec<u32> = (0..16).map(|i| word_at(&c, csa(0) + 4 * i)).collect();
        assert_eq!(
            saved,
            [0x0030_0000, 0x0B80 | psw::C | 5, DATA + 0x6000, 0x20B, 0x108, 0x109, 0x10A, 0x10B, 0x20C, 0x20D, 0x20E, 0x20F, 0x10C, 0x10D, 0x10E, 0x10F]
        );
        // CALL disp8 (SB), CALLA (p. 3-65), CALLI (p. 3-66; target read before A[11] changes)
        let c = one(&w16(s8(0x5C, 0x08)), |_| {});
        assert_eq!((c.pc, c.a[11]), (AT + 0x10, AT + 2));
        let c = one(&w32(b(0xED, 0xC0_0400)), |_| {});
        assert_eq!((c.pc, c.a[11]), (CODE + 0x800, AT + 4));
        let c = one(&w32(rr(0, 11)), |c| c.a[11] = CODE + 0x0103);
        assert_eq!((c.pc, c.a[11]), (CODE + 0x0102, AT + 4));
    }

    #[test]
    fn call_depth_one_shot_and_traps() {
        let call = w32(b(0x6D, 0x10));
        // CDE = 0: no counting, the CSA gets CDE forced to 1, the callee runs with CDE = 1 (V1 §12.6).
        let c = one(&call, |c| c.psw = 0x0B00 | 0x3F);
        assert_eq!(c.psw, 0x0B80 | 0x3F);
        assert_eq!(word_at(&c, csa(0) + 4), 0x0B80 | 0x3F);
        // 6-bit counter at 63 -> CDO (class 3 TIN 2), PSW unchanged in the trap's CSA
        let c = one(&call, |c| c.psw = 0x0B80 | 0x3F);
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(3), 2, AT));
        assert_eq!(word_at(&c, csa(0) + 4), 0x0B80 | 0x3F, "precise trap: PSW not incremented");
        // call trace mode 1111110B traps every call; 1111111B never counts
        let c = one(&call, |c| c.psw = 0x0B80 | 0x7E);
        assert_eq!(c.pc, vector(3));
        let c = one(&call, |c| c.psw = 0x0B80 | 0x7F);
        assert_eq!((c.pc, c.psw), (AT + 0x20, 0x0B80 | 0x7F));
        // FCX = 0 -> FCU (no context saved, D[15] = 4)
        let c = one(&call, |c| c.fcx = 0);
        assert_eq!((c.pc, c.d[15]), (vector(3), 4));
        // FCX == LCX: the call completes, then FCD with return address = call target (V1 p. 6-8)
        let c = one(&call, |c| c.lcx = c.fcx);
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(3), 1, AT + 0x20));
        assert_eq!(word_at(&c, csa(1) + 12), AT + 4, "the FCD entry saved the callee's A[11]");
        assert_eq!(c.icount, 1);
        // CSA in unmapped memory -> FCU
        let c = one(&call, |c| c.fcx = 0x000E_0000);
        assert_eq!((c.pc, c.d[15]), (vector(3), 4));
    }

    #[test]
    fn ret_restores_caller() {
        // CALL to AT + 0x20 then RET SR (p. 3-311) / RET SYS
        for ret in [w16(s44(0x00, 9, 0)), w32(sys(6, 0))] {
            let mut code = w32(b(0x6D, 0x10));
            code.resize(0x20, 0);
            code.extend(&ret);
            let (c, _) = exec_arch(&code, 2, Arch::Tc16, |c| {
                c.psw = 0x0B80 | psw::V | psw::SV | 1 << 24;
                for (i, reg) in c.d.iter_mut().enumerate().skip(8) {
                    *reg = i as u32;
                }
                c.a[12] = 0xAAAA;
                c.pcxi = 0x1234;
            });
            assert_eq!(c.pc, AT + 4);
            assert_eq!(c.psw, 0x0B80 | psw::V | psw::SV | 1 << 24);
            assert_eq!((c.d[8], c.d[15], c.a[12], c.a[11]), (8, 15, 0xAAAA, 0));
            assert_eq!((c.pcxi, c.fcx), (0x1234, link(csa(0))));
        }
        // PSW[25:24] (RM) are kept from the callee; everything else comes from the CSA
        let ret = w16(s44(0x00, 9, 0));
        let c = one(&ret, |c| {
            c.psw = 0x0B81 | 2 << 24;
            c.pcxi = pcxi::UL | link(csa(3));
            c.write_bytes(csa(3) + 4, &(0x8000_0B80u32 | 1 << 24).to_le_bytes()).unwrap();
            c.a[11] = CODE + 0x0201;
        });
        assert_eq!((c.pc, c.psw), (CODE + 0x0200, 0x8000_0B80 | 2 << 24));
        assert_eq!(c.fcx, link(csa(3)));
        assert_eq!(word_at(&c, csa(3)), link(csa(0)), "CSA linked back to the free list");
        // V131-2 p. 3-369 (`1.3.0` row): TC1.3 restores the entire PSW from the CSA (including [25:24]).
        let (c13, _) = exec_arch(&ret, 1, Arch::Tc13, |c| {
            c.psw = 0x0B81 | 2 << 24;
            c.pcxi = pcxi::UL | link(csa(3));
            c.write_bytes(csa(3) + 4, &(0x8000_0B80u32 | 1 << 24).to_le_bytes()).unwrap();
            c.a[11] = CODE + 0x0201;
        });
        assert_eq!((c13.pc, c13.psw), (CODE + 0x0200, 0x8000_0B80 | 1 << 24));
        let (c131, _) = exec_arch(&ret, 1, Arch::Tc131, |c| {
            c.psw = 0x0B81 | 2 << 24;
            c.pcxi = pcxi::UL | link(csa(3));
            c.write_bytes(csa(3) + 4, &(0x8000_0B80u32 | 1 << 24).to_le_bytes()).unwrap();
            c.a[11] = CODE + 0x0201;
        });
        assert_eq!((c131.pc, c131.psw), (CODE + 0x0200, 0x8000_0B80 | 2 << 24));

        // V131-2 p. 3-373 (`1.3.1` row): RFM clears DBGTCR.DTA on TC1.3.1/TC1.6, untouched on TC1.3.
        let rfm_w = w32(sys(5, 0));
        for (arch, want_dta) in [(Arch::Tc13, 1u32), (Arch::Tc131, 0u32), (Arch::Tc16, 0u32)] {
            let (cr, _) = exec_arch(&rfm_w, 1, arch, |c| {
                c.csfr_set_reset(DBGSR, DE);
                c.csfr_set_reset(DCX, DATA);
                c.csfr_set_reset(DBGTCR, DTA);
                c.a[11] = CODE + 0x0300;
            });
            assert_eq!(cr.csfr_read(DBGTCR, 0) & DTA, want_dta, "RFM DBGTCR.DTA on {arch:?}");
        }
        // CDU: count 0 with CDE = 1 (PSW unchanged); CSU: PCXI = 0; CTYP: lower context
        let c = one(&ret, |c| (c.psw, c.pcxi) = (0x0B80, pcxi::UL | link(csa(3))));
        assert_eq!((c.pc, c.d[15], word_at(&c, csa(0) + 4)), (vector(3), 3, 0x0B80));
        let c = one(&ret, |c| (c.psw, c.pcxi) = (0x0B81, 0));
        assert_eq!((c.pc, c.d[15], word_at(&c, csa(0) + 4)), (vector(3), 5, 0x0B81));
        let c = one(&ret, |c| (c.psw, c.pcxi) = (0x0B81, link(csa(3))));
        assert_eq!((c.pc, c.d[15]), (vector(3), 6));
        // CDE = 0: no decrement, no CDU
        let c = one(&ret, |c| (c.psw, c.pcxi) = (0x0B00, pcxi::UL | link(csa(3))));
        assert_ne!(c.pc, vector(3));
    }

    #[test]
    fn fast_call_and_return() {
        // FCALL (p. 3-102): M(A[10] - 4) = A[11]; A[11] = PC + 4; A[10] -= 4
        let c = one(&w32(b(0x61, 0x10)), |c| (c.a[10], c.a[11]) = (DATA + 0x100, 0xCAFE_0000));
        assert_eq!((c.pc, c.a[11], c.a[10]), (AT + 0x20, AT + 4, DATA + 0xFC));
        assert_eq!(word_at(&c, DATA + 0xFC), 0xCAFE_0000);
        // FCALLA (p. 3-103), FCALLI (p. 3-104; a = 11 reads the old A[11])
        let c = one(&w32(b(0xE1, 0xC0_0400)), |c| c.a[10] = DATA + 0x100);
        assert_eq!((c.pc, c.a[10]), (CODE + 0x800, DATA + 0xFC));
        let c = one(&w32(rr(1, 11)), |c| (c.a[10], c.a[11]) = (DATA + 0x100, CODE + 0x0301));
        assert_eq!((c.pc, c.a[11], word_at(&c, DATA + 0xFC)), (CODE + 0x300, AT + 4, CODE + 0x0301));
        // FRET SR / SYS (p. 3-105)
        for fret in [w16(s44(0x00, 7, 0)), w32(sys(3, 0))] {
            let c = one(&fret, |c| {
                (c.a[10], c.a[11]) = (DATA + 0xFC, CODE + 0x0401);
                c.write_bytes(DATA + 0xFC, &0x1357_9BDFu32.to_le_bytes()).unwrap();
            });
            assert_eq!((c.pc, c.a[11], c.a[10]), (CODE + 0x400, 0x1357_9BDF, DATA + 0x100));
        }
        // address-register word accesses must be word aligned (V1 Table 2-1): ALN (class 2 TIN 4)
        let c = one(&w32(b(0x61, 0x10)), |c| c.a[10] = DATA + 0x102);
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(2), 4, AT));
        let c = one(&w16(s44(0x00, 7, 0)), |c| c.a[10] = DATA + 0x102);
        assert_eq!((c.pc, c.d[15]), (vector(2), 4));
        // FRET from unmapped memory: DSE (synchronous, returns to the FRET)
        let c = one(&w16(s44(0x00, 7, 0)), |c| c.a[10] = 0x1000_0000);
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(4), 2, AT));
    }

    #[test]
    fn rfe_returns_from_trap_handler() {
        // SYSCALL #0x1AB (p. 3-378): class 6, TIN = const9[7:0], return address = next instruction
        let mut code = w32(rc(4, 0x1AB));
        code.resize((TRAP_BASE + (6 << 5) - AT) as usize, 0);
        code.extend(w16(s44(0x00, 8, 0))); // RFE SR at the class-6 vector
        let (c, e) = exec_arch(&code, 1, Arch::Tc16, |c| {
            c.psw = 0x0000_0080 | psw::C | 1 << 10; // user-1, user stack
            c.icr = icr::IE | 7;
        });
        assert_eq!(e, Exit::Budget);
        assert_eq!((c.pc, c.d[15], c.a[11]), (TRAP_BASE + (6 << 5), 0xAB, AT + 4));
        assert_eq!(c.icr & icr::IE, 0);
        let mut c = c;
        // RFE (p. 3-313): ICR from PCXI, whole PSW from the CSA, PC = A[11]; stops for re-arbitration
        assert_eq!(c.run(1), Exit::Stopped(crate::StopReason::IrqCheck));
        assert_eq!((c.pc, c.psw, c.icr & (icr::IE | icr::CCPN)), (AT + 4, 0x0000_0080 | psw::C | 1 << 10, icr::IE | 7));
        assert_eq!(c.a[10], DATA + 0x6000, "user stack back");
        assert_eq!((c.fcx, c.pcxi), (link(csa(0)), 0));
        // RFE SYS form; traps: CSU, CTYP, NEST (CDE = 1 and count != 0)
        let rfe = w32(sys(7, 0));
        let ctx = |c: &mut Cpu<NullBus>| {
            c.pcxi = 0x09 << 24 | pcxi::UL | link(csa(3));
            c.a[11] = CODE + 0x0501;
            c.write_bytes(csa(3) + 4, &0x0B80u32.to_le_bytes()).unwrap();
        };
        let (c, e) = exec_arch(&rfe, 1, Arch::Tc16, |c| {
            ctx(c);
            c.psw = 0x0B80;
        });
        assert_eq!((e, c.pc, c.icr & (icr::IE | icr::CCPN)), (Exit::Stopped(crate::StopReason::IrqCheck), CODE + 0x500, 9));
        let c = one(&rfe, |c| (c.pcxi, c.psw) = (0, 0x0B80));
        assert_eq!((c.pc, c.d[15]), (vector(3), 5));
        let c = one(&rfe, |c| (c.pcxi, c.psw) = (link(csa(3)), 0x0B80));
        assert_eq!((c.pc, c.d[15]), (vector(3), 6));
        let c = one(&rfe, |c| {
            ctx(c);
            c.psw = 0x0B81;
        });
        assert_eq!((c.pc, c.d[15]), (vector(3), 7));
        let c = one(&rfe, |c| {
            ctx(c);
            c.psw = 0x0B01; // CDE = 0: no NEST check
        });
        assert_eq!(c.pc, CODE + 0x500);
        let c = one(&rfe, |c| {
            ctx(c);
            c.psw = 0x0BFF; // CDC disabled: cdc_zero() is true
        });
        assert_eq!(c.pc, CODE + 0x500);
    }

    #[test]
    fn lower_context_save_restore() {
        let sv = w32(sys(8, 0));
        // SVLCX (p. 3-375) / RSLCX (p. 3-316) round trip
        let mut code = sv.clone();
        code.extend(w32(sys(9, 0)));
        let (c, _) = exec_arch(&code, 1, Arch::Tc16, |c| {
            for (i, reg) in c.d.iter_mut().enumerate().take(8) {
                *reg = 0x10 + i as u32;
            }
            for (i, reg) in c.a.iter_mut().enumerate().take(8).skip(2) {
                *reg = 0x20 + i as u32;
            }
            c.a[11] = 0x2B;
            c.icr = icr::IE | 3;
            c.pcxi = 0x1111;
        });
        assert_eq!(c.pc, AT + 4);
        assert_eq!(c.pcxi, 3 << 24 | pcxi::PIE | link(csa(0)));
        let saved: Vec<u32> = (0..16).map(|i| word_at(&c, csa(0) + 4 * i)).collect();
        assert_eq!(saved, [0x1111, 0x2B, 0x22, 0x23, 0x10, 0x11, 0x12, 0x13, 0x24, 0x25, 0x26, 0x27, 0x14, 0x15, 0x16, 0x17]);
        let mut c = c;
        c.d[0] = 0;
        c.a[11] = 0;
        c.psw |= psw::C;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!((c.d[0], c.a[11], c.pcxi, c.fcx), (0x10, 0x2B, 0x1111, link(csa(0))));
        assert_ne!(c.psw & psw::C, 0, "RSLCX leaves PSW alone");
        // SVLCX: FCU, FCD (return address = next instruction)
        let c = one(&sv, |c| c.fcx = 0);
        assert_eq!((c.pc, c.d[15]), (vector(3), 4));
        let c = one(&sv, |c| c.lcx = c.fcx);
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(3), 1, AT + 4));
        // RSLCX: CSU, CTYP (upper context)
        let rs = w32(sys(9, 0));
        let c = one(&rs, |c| c.pcxi = 0);
        assert_eq!((c.pc, c.d[15]), (vector(3), 5));
        let c = one(&rs, |c| c.pcxi = pcxi::UL | link(csa(3)));
        assert_eq!((c.pc, c.d[15]), (vector(3), 6));
    }

    #[test]
    fn bisr_saves_lower_and_sets_priority() {
        // BISR const9 (RC, p. 3-43): only const9[7:0] is used
        for (code, ccpn) in [(w32(rc(0, 0x1FE)), 0xFE), (w16(s8(0xE0, 0x7E)), 0x7E)] {
            let (c, e) = exec_arch(&code, 1, Arch::Tc16, |c| {
                c.icr = 0x0005_0003; // PIPN 5, CCPN 3, IE 0
                c.a[11] = 0xB0B;
                c.d[7] = 0x77;
            });
            assert_eq!(e, Exit::Stopped(crate::StopReason::IrqCheck));
            assert_eq!(c.pc, AT + code.len() as u32);
            assert_eq!(c.icr, 0x0005_0000 | icr::IE | ccpn);
            assert_eq!(c.pcxi, 3 << 24 | link(csa(0)), "PCPN = old CCPN, PIE = old IE, UL = 0");
            assert_eq!((word_at(&c, csa(0) + 4), word_at(&c, csa(0) + 60)), (0xB0B, 0x77));
        }
        let bisr = w16(s8(0xE0, 1));
        // Supervisor only (Table 1-14): PRIV (class 1 TIN 1) in User-1
        let c = one(&bisr, |c| c.psw = 0x0B80 & !psw::IO | 1 << 10);
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(1), 1, AT));
        // FCU, FCD
        let c = one(&bisr, |c| c.fcx = 0);
        assert_eq!((c.pc, c.d[15]), (vector(3), 4));
        let c = one(&bisr, |c| c.lcx = c.fcx);
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(3), 1, AT + 2));
    }

    // ---- traps, interrupt enable, system ----------------------------------------------------------

    #[test]
    fn trapv_trapsv_and_syscall() {
        // TRAPV (p. 3-380): OVF class 5 TIN 1, return address = the TRAPV itself (synchronous)
        let c = one(&w32(sys(0x14, 0)), |c| c.psw |= psw::V);
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(5), 1, AT));
        let c = one(&w32(sys(0x14, 0)), |c| c.psw |= psw::SV);
        assert_eq!(c.pc, AT + 4);
        // TRAPSV (p. 3-379): SOVF class 5 TIN 2
        let c = one(&w32(sys(0x15, 0)), |c| c.psw |= psw::SV);
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(5), 2, AT));
        let c = one(&w32(sys(0x15, 0)), |c| c.psw |= psw::V);
        assert_eq!(c.pc, AT + 4);
        // SYSCALL (p. 3-378)
        let c = one(&w32(rc(4, 0x0FF)), |_| {});
        assert_eq!((c.pc, c.d[15], c.a[11]), (vector(6), 0xFF, AT + 4));
    }

    #[test]
    fn interrupt_enable_instructions() {
        let irq = Exit::Stopped(crate::StopReason::IrqCheck);
        let user0 = |c: &mut Cpu<NullBus>| c.psw &= !psw::IO;
        let user1 = |c: &mut Cpu<NullBus>| c.psw = c.psw & !psw::IO | 1 << 10;
        // ENABLE (p. 3-91)
        let (c, e) = exec_arch(&w32(sys(0x0C, 0)), 1, Arch::Tc16, |c| {
            user1(c);
            c.icr = 0x0003_0005;
        });
        assert_eq!((e, c.pc, c.icr), (irq, AT + 4, 0x0003_0005 | icr::IE));
        // DISABLE (p. 3-80)
        let (c, e) = exec_arch(&w32(sys(0x0D, 0)), 1, Arch::Tc16, |c| c.icr = icr::IE | 5);
        assert_eq!((e, c.icr), (irq, 5));
        // DISABLE D[a]: D[a] = old IE
        let (c, e) = exec_arch(&w32(sys(0x0F, 3)), 1, Arch::Tc16, |c| (c.icr, c.d[3]) = (icr::IE | 5, 0xFFFF));
        assert_eq!((e, c.icr, c.d[3]), (irq, 5, 1));
        let c = one(&w32(sys(0x0F, 3)), |c| (c.icr, c.d[3]) = (5, 0xFFFF));
        assert_eq!((c.icr, c.d[3]), (5, 0));
        // RESTORE D[a] (p. 3-310): IE = D[a][0]
        let (c, e) = exec_arch(&w32(sys(0x0E, 3)), 1, Arch::Tc16, |c| (c.icr, c.d[3]) = (5, 3));
        assert_eq!((e, c.icr), (irq, icr::IE | 5));
        let c = one(&w32(sys(0x0E, 3)), |c| (c.icr, c.d[3]) = (icr::IE | 5, 2));
        assert_eq!(c.icr, 5);
        // User-0: PRIV for all four (Table 1-14)
        for w in [sys(0x0C, 0), sys(0x0D, 0), sys(0x0F, 3), sys(0x0E, 3)] {
            let c = one(&w32(w), |c| {
                user0(c);
                c.icr = 5;
            });
            assert_eq!((c.pc, c.d[15], c.a[11]), (vector(1), 1, AT), "{w:#x}");
        }
    }

    #[test]
    fn nop_sync_and_debug() {
        for code in [w16(0x0000), w32(sys(0, 0)), w32(sys(0x12, 0)), w32(sys(0x13, 0)), w16(s44(0x00, 0xA, 0)), w32(sys(4, 0))] {
            let (c, e) = exec_arch(&code, 1, Arch::Tc16, |c| c.psw = 0x0B80 | psw::C);
            assert_eq!((e, c.pc, c.psw, c.icount), (Exit::Budget, AT + code.len() as u32, 0x0B80 | psw::C, 1), "{code:x?}");
        }
        // DEBUG with DBGSR.DE = 1 is a debug event (the instruction retires)
        for code in [w16(s44(0x00, 0xA, 0)), w32(sys(4, 0))] {
            let (c, e) = exec_arch(&code, 5, Arch::Tc16, |c| c.csfr_set_reset(DBGSR, DE));
            assert_eq!((e, c.pc, c.icount), (Exit::Stopped(crate::StopReason::Debug), AT + code.len() as u32, 1));
        }
        // NOP does not end a block: 3 NOPs then a J are one block
        let mut code = vec![0u8; 6];
        code.extend(w32(b(0x1D, 0xFF_FFFD)));
        let (c, _) = exec_arch(&code, 4, Arch::Tc16, |_| {});
        assert_eq!((c.pc, c.cached_blocks()), (AT, 1));
    }

    #[test]
    fn rfm_returns_from_debug_monitor() {
        let rfm = w32(sys(5, 0));
        let dcx = DATA + 0x200;
        let set = |c: &mut Cpu<NullBus>| {
            c.csfr_set_reset(DBGSR, DE);
            c.csfr_set_reset(DCX, dcx | 0x3F); // reserved [5:0] ignored
            c.csfr_set_reset(DBGTCR, DTA);
            for (i, v) in [0x1234u32, 0x0B80 | psw::C, DATA + 0x5000, 0xC0DE].iter().enumerate() {
                c.write_bytes(dcx + 4 * i as u32, &v.to_le_bytes()).unwrap();
            }
            c.pcxi = 0x07 << 24 | pcxi::PIE;
            c.a[11] = CODE + 0x0601;
            c.icr = 0;
        };
        let (c, e) = exec_arch(&rfm, 1, Arch::Tc16, set);
        assert_eq!(e, Exit::Stopped(crate::StopReason::IrqCheck));
        assert_eq!((c.pc, c.pcxi, c.psw, c.a[10], c.a[11]), (CODE + 0x600, 0x1234, 0x0B80 | psw::C, DATA + 0x5000, 0xC0DE));
        assert_eq!((c.icr, c.csfr_read(DBGTCR, 0)), (icr::IE | 7, 0));
        // DBGSR.DE = 0: NOP
        let c = one(&rfm, |c| {
            set(c);
            c.csfr_set_reset(DBGSR, 0);
        });
        assert_eq!((c.pc, c.pcxi), (AT + 4, 0x07 << 24 | pcxi::PIE));
        // Supervisor only (RTL: PSW.IO != 10B -> PRIV)
        let c = one(&rfm, |c| {
            set(c);
            c.psw = 0x0B80 | psw::IO;
        });
        assert_eq!((c.pc, c.d[15]), (vector(1), 1));
    }

    #[test]
    fn mfcr_mtcr() {
        // MFCR (p. 3-219): any privilege; PC reads as this instruction; undefined offsets read 0
        let c = one(&w32(rlc(0x4D, 3, 0xFE04, 0)), |c| c.psw = 0x0000_0080 | psw::C);
        assert_eq!(c.d[3], 0x0000_0080 | psw::C);
        let c = one(&w32(rlc(0x4D, 3, 0xFE08, 0)), |_| {});
        assert_eq!(c.d[3], AT);
        let c = one(&w32(rlc(0x4D, 3, 0xFE2C, 0)), |c| c.icr = 0x0012_0105);
        assert_eq!(c.d[3], 0x0012_0105);
        let c = one(&w32(rlc(0x4D, 3, 0x1234, 0)), |c| c.d[3] = 9);
        assert_eq!(c.d[3], 0);
        // MTCR (p. 3-268): ICR stops for re-arbitration, PIPN read-only
        let (c, e) = exec_arch(&w32(rlc(0xCD, 0, 0xFE2C, 3)), 1, Arch::Tc16, |c| (c.icr, c.d[3]) = (0x0012_0000, 0x00FF_0109));
        assert_eq!((e, c.pc, c.icr), (Exit::Stopped(crate::StopReason::IrqCheck), AT + 4, 0x0012_0109));
        let (c, e) = exec_arch(&w32(rlc(0xCD, 0, 0xFE04, 3)), 1, Arch::Tc16, |c| c.d[3] = 0x0000_0B81);
        assert_eq!((e, c.pc, c.psw), (Exit::Budget, AT + 4, 0x0B81));
        let c = one(&w32(rlc(0xCD, 0, 0xFE24, 3)), |c| c.d[3] = 0x8000_0101);
        assert_eq!(c.btv, 0x8000_0100);
        // Supervisor only: PRIV in User-1 (register unchanged)
        let c = one(&w32(rlc(0xCD, 0, 0xFE38, 3)), |c| {
            c.psw = c.psw & !psw::IO | 1 << 10;
            c.d[3] = 0;
        });
        assert_eq!((c.pc, c.d[15]), (vector(1), 1));
        assert_ne!(c.fcx, 0, "FCX not written");
        // PC: TC1.6 write has no effect while running (V1 p. 3-4); TC1.3.1 PC is rw (V131-1 p. 3-5)
        let mtpc = w32(rlc(0xCD, 0, 0xFE08, 3));
        let (c, _) = exec_arch(&mtpc, 1, Arch::Tc16, |c| c.d[3] = CODE + 0x0701);
        assert_eq!(c.pc, AT + 4);
        let (c, _) = exec_arch(&mtpc, 1, Arch::Tc131, |c| c.d[3] = CODE + 0x0701);
        assert_eq!(c.pc, CODE + 0x700);
        // ENDINIT-protected CSFRs are not written while ENDINIT = 1
        struct Locked;
        impl Bus for Locked {
            fn read(&mut self, _: u32, _: u8) -> Result<u64, crate::BusError> {
                Err(crate::BusError)
            }
            fn write(&mut self, _: u32, _: u8, _: u64) -> Result<(), crate::BusError> {
                Err(crate::BusError)
            }
            fn endinit(&self) -> bool {
                true
            }
        }
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x1000], true);
        m.map(CODE, r, 0, 0x1000);
        let mut c = Cpu::new(Arch::Tc16, m, Locked);
        c.write_bytes(CODE, &w32(rlc(0xCD, 0, 0xFE20, 3))).unwrap();
        (c.pc, c.biv, c.d[3]) = (CODE, 0x100, 0x8000_0000);
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!((c.pc, c.biv), (CODE + 4, 0x100));
    }

    #[test]
    fn tc131_only_lacks_tc16_additions() {
        // FCALL on TC1.3.1 is an illegal opcode (IOPC, class 2 TIN 1)
        let (c, _) = exec_arch(&w32(b(0x61, 0x10)), 1, Arch::Tc131, |_| {});
        assert_eq!((c.pc, c.d[15]), (vector(2), 1));
        // the shared encodings behave identically
        let (c, _) = exec_arch(&w16(s44(0x1E, 0xF, 3)), 1, Arch::Tc131, |c| c.d[15] = u32::MAX);
        assert_eq!(c.pc, AT + 6);
    }

    // ---- differential tests against bare Unicorn ------------------------------------------------

    struct Rng(u32);
    impl Rng {
        fn next(&mut self) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            self.0
        }
        fn below(&mut self, n: u32) -> u32 {
            self.next() % n
        }
    }

    /// Base setup shared by the native core and Unicorn: registers of `cpu_with` plus the free CSA list.
    fn base_setup() -> Setup {
        let c = cpu_with(&[], Arch::Tc16);
        let mut s = Setup::from_cpu(&c);
        for i in 0..8u32 {
            let next = if i < 7 { link(csa(i + 1)) } else { 0 };
            s.mem.push((csa(i), next.to_le_bytes().to_vec()));
        }
        s.pc = AT;
        s
    }

    /// Run `code` at `s.pc` for `n` instructions on both cores and compare the architectural state
    /// (D, A, PC, PSW, PCXI, FCX, LCX, ICR, DATA memory). `Err` if Unicorn failed.
    ///
    /// Random PSW values (registers and CSA images) keep PSW[31:24] zero: Unicorn 2.1.4 stores a PSW
    /// register write raw, without distributing the user status bits to its separate flag fields, so
    /// the first helper that reads the PSW (MFCR, CALL's context save, ...) sees C/V/SV/AV/SAV and
    /// PSW[24] cleared (0xB300_08FF -> 0x0200_08FF; probed). The oracle therefore cannot check the
    /// status bits / RM handling of CALL/RET/RFE; the unit tests above do.
    fn differential(code: &[u8], n: u64, s: &Setup) -> Result<(), String> {
        let mut s = s.clone();
        s.mem.push((s.pc, code.to_vec()));
        let o = oracle::run(&[], n as usize, &s).map_err(|e| format!("unicorn error {e}"))?;
        let mut c = cpu_with(&[], Arch::Tc16);
        s.apply(&mut c);
        c.pc = s.pc;
        c.run(n);
        let mut data = vec![0u8; DATA_SIZE as usize];
        assert!(c.read_bytes(DATA, &mut data));
        let mine = (c.d, c.a, c.pc, c.psw, c.pcxi, c.fcx, c.lcx, c.icr);
        let theirs = (o.d, o.a, o.pc, o.psw, o.pcxi, o.fcx, o.lcx, o.icr);
        if mine != theirs {
            return Err(format!("{code:02x?}\n native {mine:x?}\n unicorn {theirs:x?}\n setup {s:x?}"));
        }
        if data != o.data {
            let i = data.iter().zip(&o.data).position(|(x, y)| x != y).unwrap();
            return Err(format!("{code:02x?}: DATA differs at {:#x}\n setup {s:x?}", DATA + i as u32));
        }
        Ok(())
    }

    fn halves(code: &[u8]) -> (u16, u16) {
        let h = |i: usize| code.get(i..i + 2).map_or(0, |x| u16::from_le_bytes([x[0], x[1]]));
        (h(0), h(2))
    }

    /// A random 32-bit value biased towards the interesting compare cases.
    fn operand(r: &mut Rng, other: u32) -> u32 {
        match r.below(6) {
            0 => other,
            1 => other.wrapping_add(1),
            2 => other.wrapping_sub(1),
            3 => (r.below(32) as i32 - 16) as u32,
            4 => [0, 0x8000_0000, 0x7FFF_FFFF, u32::MAX][r.below(4) as usize],
            _ => r.next(),
        }
    }

    /// A disp24 / disp15 reaching AT +/- 4 KB (both signs), so the target stays inside CODE. Large
    /// displacements are covered by the unit tests and the firmware decode cross-check.
    fn near24(r: &mut Rng) -> u32 {
        (r.below(0x1000) as i32 - 0x800) as u32 & 0xFF_FFFF
    }
    fn near15(r: &mut Rng) -> u32 {
        near24(r) & 0x7FFF
    }

    /// All branch encodings: random fields (targets kept inside CODE so Unicorn can fetch them) and
    /// random, compare-biased operands.
    #[test]
    fn differential_branches_vs_unicorn() {
        if !unicorn_tricore_sys::LINKED {
            return;
        }
        let mut r = Rng(0x2545_F491);
        type Gen = fn(&mut Rng) -> Vec<u8>;
        let gens: &[(&str, Gen)] = &[
            ("J", |r| w32(b(0x1D, near24(r)))),
            ("JA", |r| w32(b(0x9D, 0xC0_0000 | r.below(0x1000)))),
            ("JL", |r| w32(b(0x5D, near24(r)))),
            ("JLA", |r| w32(b(0xDD, 0xC0_0000 | r.below(0x1000)))),
            ("J16", |r| w16(s8(0x3C, r.below(256) as u16))),
            ("JI", |r| w32(rr(3, r.below(16)))),
            ("JI16", |r| w16(s44(0xDC, 0, r.below(16) as u16))),
            ("JLI", |r| w32(rr(2, r.below(16)))),
            ("JEQ/JNE BRC", |r| w32(br(0xDF, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JEQ/JNE BRR", |r| w32(br(0x5F, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JGE BRC", |r| w32(br(0xFF, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JGE BRR", |r| w32(br(0x7F, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JLT BRC", |r| w32(br(0xBF, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JLT BRR", |r| w32(br(0x3F, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JNED/JNEI BRC", |r| w32(br(0x9F, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JNED/JNEI BRR", |r| w32(br(0x1F, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JEQ.A/JNE.A", |r| w32(br(0x7D, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JZ.A/JNZ.A", |r| w32(br(0xBD, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("LOOP/LOOPU", |r| w32(br(0xFD, r.below(2), r.below(16), r.below(16), near15(r)))),
            ("JZ.T/JNZ.T", |r| w32(brn(r.below(2), r.below(16), r.below(32), near15(r)))),
            ("SB JZ/JNZ", |r| w16(s8([0x6E, 0xEE][r.below(2) as usize], r.below(256) as u16))),
            ("SBC", |r| w16(s44([0x1E, 0x9E, 0x5E, 0xDE][r.below(4) as usize], r.below(16) as u16, r.below(16) as u16))),
            ("SBR", |r| {
                let ops = [0x3E, 0xBE, 0x7E, 0xFE, 0xCE, 0x4E, 0x8E, 0x0E, 0xF6, 0x76, 0x7C, 0xBC, 0xFC];
                w16(s44(ops[r.below(ops.len() as u32) as usize], r.below(16) as u16, r.below(16) as u16))
            }),
            ("SBRN", |r| w16(s44([0x2E, 0xAE][r.below(2) as usize], r.below(16) as u16, r.below(16) as u16))),
        ];
        let mut runs = 0;
        for (name, gen) in gens {
            let mut ok = 0;
            while ok < 400 {
                let code = gen(&mut r);
                let (lo, hi) = halves(&code);
                let Some(op) = decode(lo, hi, AT, Arch::Tc16) else { panic!("{name}: {code:x?} not decoded") };
                assert!(oracle::trustworthy(lo, hi));
                let mut s = base_setup();
                for i in 0..16 {
                    s.d[i] = r.next();
                    // address registers: mostly code addresses (JI/JLI targets), some small values
                    s.a[i] = if r.below(4) == 0 { r.below(4) } else { CODE + r.below(0x2000) };
                }
                let (a, bb) = (r.below(16) as usize, r.below(16) as usize);
                s.d[a] = operand(&mut r, s.d[bb]);
                s.a[a] = if r.below(3) == 0 { s.a[bb] } else { s.a[a] };
                // keep the taken target inside CODE (Unicorn fetches it before stopping)
                let t = match op {
                    O::J { t } | O::Jl { t } | O::JDd { t, .. } | O::JDi { t, .. } | O::JAa { t, .. } | O::JAz { t, .. }
                    | O::JBit { t, .. } | O::JStepD { t, .. } | O::JStepI { t, .. } | O::Loop { t, .. } => t,
                    O::Ji { a } | O::Jli { a } => s.a[a as usize] & !1,
                    _ => unreachable!(),
                };
                if !(CODE..CODE + 0x2000).contains(&t) {
                    continue;
                }
                // make the compare operand of the instruction itself interesting
                if let O::JDd { a, b, .. } | O::JStepD { a, b, .. } = op {
                    s.d[a as usize] = operand(&mut r, s.d[b as usize]);
                }
                if let O::JDi { a, imm, .. } | O::JStepI { a, imm, .. } = op {
                    s.d[a as usize] = operand(&mut r, imm as i32 as u32);
                }
                if let O::JAa { a, b, .. } = op {
                    if r.below(2) == 0 {
                        s.a[a as usize] = s.a[b as usize];
                    }
                }
                if let O::JAz { a, .. } | O::Loop { b: a, .. } = op {
                    if r.below(2) == 0 {
                        s.a[a as usize] = r.below(2);
                    }
                }
                differential(&code, 1, &s).unwrap_or_else(|e| panic!("{name}: {e}"));
                ok += 1;
                runs += 1;
            }
        }
        assert!(runs >= 9600);
    }

    /// CALL/CALLA/CALLI followed by RET (both forms), FCALL/FCALLA/FCALLI followed by FRET, with
    /// random register contents, PSW (CDE = 1, no CDO), ICR and PCXI.
    #[test]
    fn differential_calls_vs_unicorn() {
        if !unicorn_tricore_sys::LINKED {
            return;
        }
        let mut r = Rng(0x9E37_79B9);
        for i in 0..1200 {
            let mut s = base_setup();
            for k in 0..16 {
                s.d[k] = r.next();
                s.a[k] = r.next();
            }
            s.a[10] = DATA + 0x6000 - 4 * r.below(64);
            s.a[11] = r.next();
            // CDE = 1, counter not at its overflow value; USB, RM random
            let cdc = [r.below(63), 0x40 | r.below(31), 0x7F, 0x70 | r.below(7)][r.below(4) as usize];
            s.psw = (r.next() & 0x0000_0F00) | psw::CDE | cdc;
            s.icr = r.next() & 0x1FF;
            s.pcxi = r.next() & 0xFFF0_0000;
            let callee = 0x100 + 2 * r.below(0x700); // target offset from CODE
            let fast = i % 2 == 1;
            let call = match (i / 2) % 3 {
                0 => {
                    let disp = (callee as i32 - (AT - CODE) as i32) >> 1;
                    w32(b(if fast { 0x61 } else { 0x6D }, disp as u32 & 0xFF_FFFF))
                }
                1 => w32(b(if fast { 0xE1 } else { 0xED }, 0xC0_0000 | callee >> 1)),
                _ => {
                    let a = [2, 4, 7, 12, 15][r.below(5) as usize];
                    s.a[a] = CODE + callee + r.below(2);
                    w32(rr(fast as u32, a as u32))
                }
            };
            let ret = if fast {
                [w16(s44(0x00, 7, 0)), w32(sys(3, 0))][r.below(2) as usize].clone()
            } else {
                [w16(s44(0x00, 9, 0)), w32(sys(6, 0))][r.below(2) as usize].clone()
            };
            s.mem.push((CODE + callee, ret));
            differential(&call, 1, &s).unwrap_or_else(|e| panic!("call #{i}: {e}"));
            differential(&call, 2, &s).unwrap_or_else(|e| panic!("call+ret #{i}: {e}"));
        }
    }

    /// RET / RFE / RSLCX from random CSA contents, SVLCX with random lower registers.
    #[test]
    fn differential_context_vs_unicorn() {
        if !unicorn_tricore_sys::LINKED {
            return;
        }
        let mut r = Rng(0x1234_5679);
        for i in 0..1500 {
            let mut s = base_setup();
            for k in 0..16 {
                s.d[k] = r.next();
                s.a[k] = r.next();
            }
            s.a[11] = CODE + r.below(0x2000);
            s.icr = r.next() & 0x1FF;
            let kind = i % 4;
            let upper = kind < 2;
            // the previous context in CSA 5 with random content; PCXI points to it
            let mut ctx: Vec<u8> = (0..16).flat_map(|_| r.next().to_le_bytes()).collect();
            if kind == 0 || kind == 1 {
                // restored PSW: random except PSW[31:24] (see `differential`)
                ctx[4..8].copy_from_slice(&(r.next() & 0x00FF_FFFF).to_le_bytes());
            }
            s.mem.push((csa(5), ctx));
            s.pcxi = (r.next() & 0xFF30_0000) | if upper { pcxi::UL } else { 0 } | link(csa(5));
            s.pcxi |= r.next() & pcxi::PIE;
            let code = match kind {
                0 => {
                    // RET: CDE = 1 with count > 0, or CDE = 0, or counting disabled
                    s.psw = [psw::CDE | (1 + r.below(62)), r.below(64), psw::CDE | 0x7F][r.below(3) as usize] | (r.next() & 0x0000_0F00);
                    [w16(s44(0x00, 9, 0)), w32(sys(6, 0))][r.below(2) as usize].clone()
                }
                1 => {
                    // RFE: no NEST (count 0 or CDE = 0)
                    s.psw = [psw::CDE, r.below(64), psw::CDE | 0x40][r.below(3) as usize] | (r.next() & 0x0000_0F00);
                    [w16(s44(0x00, 8, 0)), w32(sys(7, 0))][r.below(2) as usize].clone()
                }
                2 => {
                    s.psw = r.next() & 0x0000_0FFF;
                    w32(sys(9, 0)) // RSLCX
                }
                _ => {
                    s.psw = r.next() & 0x0000_0FFF;
                    s.pcxi = r.next();
                    w32(sys(8, 0)) // SVLCX
                }
            };
            differential(&code, 1, &s).unwrap_or_else(|e| panic!("#{i}: {e}"));
        }
    }

    /// ENABLE, DISABLE, NOP (both), ISYNC, DSYNC, MFCR/MTCR on CSFRs Unicorn implements and that are
    /// not ENDINIT protected (not ICR: `oracle::KNOWN_WRONG`, stale cached ICR). Not TRAPV/TRAPSV: Unicorn 2.1.4 never advances PC past a TRAPV/TRAPSV
    /// that does not trap (the instruction re-executes forever), so it is no oracle for them.
    #[test]
    fn differential_system_vs_unicorn() {
        if !unicorn_tricore_sys::LINKED {
            return;
        }
        let mut r = Rng(0xDEAD_BEEF);
        for i in 0..2000 {
            let mut s = base_setup();
            for k in 0..16 {
                s.d[k] = r.next();
                s.a[k] = r.next();
            }
            s.psw = (r.next() & 0x0000_037F) | psw::IO_SUPERVISOR | psw::CDE;
            s.icr = r.next() & 0x1FF;
            s.pcxi = r.next();
            let reg = r.below(16);
            let code = match i % 9 {
                0 => w32(sys(0x0C, 0)),
                1 => w32(sys(0x0D, 0)),
                2 => [w16(0), w32(sys(0, 0))][r.below(2) as usize].clone(),
                3 => w32(sys(0x12 + r.below(2), 0)),
                4..=6 => {
                    let off = [0xFE00, 0xFE04, 0xFE38, 0xFE3C, 0xFE24][r.below(5) as usize];
                    w32(rlc(0x4D, reg, off, 0))
                }
                _ => {
                    // writable bits only: PSW, PCXI, FCX/LCX link words
                    let (off, mask) = [(0xFE04, 0xFFFF_FFFF), (0xFE00, 0xFFFF_FFFF), (0xFE38, 0xF_FFFF), (0xFE3C, 0xF_FFFF)]
                        [r.below(4) as usize];
                    s.d[reg as usize] &= mask;
                    w32(rlc(0xCD, 0, off, reg))
                }
            };
            let (lo, hi) = halves(&code);
            assert!(oracle::trustworthy(lo, hi), "{code:x?}");
            differential(&code, 1, &s).unwrap_or_else(|e| panic!("#{i}: {e}"));
        }
    }

    /// Cases where the manual and Unicorn 2.1.4 disagree (Unicorn bug candidates; each is a
    /// differential mismatch). The native core follows the manual, see the unit tests above.
    #[test]
    fn unicorn_disagreements_are_known() {
        if !unicorn_tricore_sys::LINKED {
            return;
        }
        type F = fn(&mut Setup);
        let cases: &[(&str, Vec<u8>, F)] = &[
            // CSA PSW image: CDE forced to 1 (V2 p. 3-63); Unicorn saves CDE = 0
            ("CALL with PSW.CDE = 0", w32(b(0x6D, 0x10)), |s| s.psw = 0x0B05),
            // trap entry: upper context saved, then D[15] = TIN (V1 §6.2.5); Unicorn saves D[15] = TIN
            ("CDO trap entry", w32(b(0x6D, 0x10)), |s| s.psw = 0x0BBF),
            // precise trap: Unicorn keeps the decremented call depth when RET raises CSU
            ("RET -> CSU after cdc_decrement", w16(s44(0, 9, 0)), |s| (s.pcxi, s.psw) = (0, 0x0B81)),
            // FCD return address = next instruction / call target (V1 p. 6-8); Unicorn A[11] = 0
            ("CALL -> FCD", w32(b(0x6D, 0x10)), |s| s.lcx = s.fcx),
            ("SVLCX -> FCD", w32(sys(8, 0)), |s| s.lcx = s.fcx),
            // SYS trap (V2 p. 3-378): Unicorn does not vector, it just continues
            ("SYSCALL", w32(rc(4, 0x12)), |_| {}),
            // TRAPV with PSW.V = 1: no OVF trap, PC not advanced
            ("TRAPV taken", w32(sys(0x14, 0)), |s| s.psw |= psw::V),
            // TRAPV with PSW.V = 0: PC not advanced (re-executes forever)
            ("TRAPV not taken", w32(sys(0x14, 0)), |_| {}),
            // RFM with DBGSR.DE = 0 is a NOP (V2 p. 3-315); Unicorn does not advance PC
            ("RFM, DE = 0", w32(sys(5, 0)), |_| {}),
            // privilege (V2 Table 1-14): Unicorn never raises PRIV
            ("ENABLE in User-0", w32(sys(0x0C, 0)), |s| s.psw = 0x0380),
            ("MTCR PCXI in User-1", w32(rlc(0xCD, 0, 0xFE00, 3)), |s| (s.psw, s.d[3]) = (0x0780, 0x105)),
            ("BISR in User-1", w16(s8(0xE0, 3)), |s| s.psw = 0x0780),
            // address-register word store must be word aligned (V1 Table 2-1): no ALN in Unicorn
            ("FCALL, A[10] - 4 not word aligned", w32(b(0x61, 0x10)), |s| s.a[10] = DATA + 0x102),
        ];
        for (name, code, f) in cases {
            let mut s = base_setup();
            f(&mut s);
            assert!(differential(code, 1, &s).is_err(), "{name}: Unicorn now agrees with the manual");
        }
    }
}
