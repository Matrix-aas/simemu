//! Central decoded-operation type and the decode/execute dispatchers.
//!
//! This file is owned by the framework. Instruction groups never edit it: each group module in
//! `crate::groups` provides
//!
//! ```ignore
//! pub enum XOp { .. }                                   // the group's predecoded operations
//! pub const MNEMONICS: &[&str];                          // mnemonics the group owns (manual names)
//! pub fn decode(lo: u16, hi: u16, pc: u32, arch: Arch) -> Option<XOp>;
//! pub fn ends_block(op: &XOp) -> bool;                    // may change PC / raise a trap / stop
//! pub fn exec<B: Bus>(cpu: &mut Cpu<B>, op: &XOp, pc: u32, len: u32) -> Flow;
//! ```
//!
//! and the dispatchers below call them in a fixed order.

use crate::arch::{Arch, Trap};
use crate::bus::Bus;
use crate::cpu::Cpu;
use crate::fmt::insn_len;
use crate::groups::{alu, bitmul, ctrl, fpu_div, ldst};

/// One predecoded instruction. Keep `Op` within 16 bytes (checked by a test): groups store register
/// numbers as `u8` and pre-extend immediates at decode time.
#[derive(Clone, Copy, Debug)]
pub enum Op {
    Alu(alu::AluOp),
    BitMul(bitmul::BitMulOp),
    LdSt(ldst::LdStOp),
    Ctrl(ctrl::CtrlOp),
    FpuDiv(fpu_div::FpuDivOp),
    /// No group decodes this encoding: IOPC trap when executed (V1 Table 6-1, class 2 TIN 1).
    Invalid,
}

/// Result of executing one instruction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flow {
    /// Continue with the next sequential instruction (`pc + len`).
    Next,
    /// Transfer control to the target (taken branch, call, return, ...). Ends the block.
    Jump(u32),
    /// Take a trap. `ret` is the return address written to `A[11]`: the PC of the trapping instruction
    /// for synchronous traps, the next instruction for SYS and asynchronous traps, the target for
    /// FCD after CALL (V1 §6.2.3 p. 6-4). The instruction counts as executed.
    Trap { trap: Trap, ret: u32 },
    /// Retire the instruction, continue at `next` but return from [`Cpu::run`] first so the caller
    /// can act (interrupt re-arbitration after ENABLE/MTCR ICR/BISR/RFE, WAIT, DEBUG halt, ...).
    Stop { next: u32, why: StopReason },
}

impl Flow {
    /// Synchronous trap raised by the instruction at `pc`.
    #[inline(always)]
    pub const fn trap(trap: Trap, pc: u32) -> Flow {
        Flow::Trap { trap, ret: pc }
    }

    /// Trap reported by a memory access of the instruction at `pc` (`len` bytes long): synchronous
    /// traps (DSE, ALN, ...) return to `pc`, asynchronous ones (DAE) to the next instruction.
    #[inline(always)]
    pub const fn fault(trap: Trap, pc: u32, len: u32) -> Flow {
        Flow::Trap { trap, ret: if trap.is_async() { pc.wrapping_add(len) } else { pc } }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    /// Interrupt enable / priority changed: the caller must re-arbitrate pending interrupts.
    IrqCheck,
    /// The core is halted until an interrupt (implementation-specific idle).
    Wait,
    /// DEBUG instruction or debug event without a debugger.
    Debug,
}

/// A decoded instruction of a block: one record of the threaded code (32 bytes).
#[derive(Clone, Copy, Debug)]
pub struct Insn {
    pub op: Op,
    /// Address of the instruction.
    pub pc: u32,
    pub len: u8,
    /// [`hot`] id of the operation (0: generic dispatch through [`exec`]).
    pub hot: u8,
    /// Position in its block (the number of block instructions before it).
    pub idx: u8,
    /// Threaded-code handler of the core that cached the block (`crate::cpu`); 0 until then.
    pub(crate) h: usize,
}

/// Decode the instruction at `pc` (`hi` is ignored for 16-bit encodings).
#[inline]
pub fn decode(lo: u16, hi: u16, pc: u32, arch: Arch) -> Insn {
    let len = insn_len(lo) as u8;
    let op = if let Some(o) = alu::decode(lo, hi, pc, arch) {
        Op::Alu(o)
    } else if let Some(o) = bitmul::decode(lo, hi, pc, arch) {
        Op::BitMul(o)
    } else if let Some(o) = ldst::decode(lo, hi, pc, arch) {
        Op::LdSt(o)
    } else if let Some(o) = ctrl::decode(lo, hi, pc, arch) {
        Op::Ctrl(o)
    } else if let Some(o) = fpu_div::decode(lo, hi, pc, arch) {
        Op::FpuDiv(o)
    } else {
        Op::Invalid
    };
    Insn { op, pc, len, hot: hot::id(&op), idx: 0, h: 0 }
}

/// Hot-instruction dispatch. The firmware's dynamic mix is broad (no opcode above 6%) and blocks
/// are short (4.6 instructions in the SIMOS18.1 workload), so the generic two-level dispatch
/// (group, then kind) costs two hard-to-predict indirect branches per instruction. The most
/// frequent (group, kind[, mode]) combinations get an id at decode time; [`exec_hot`] dispatches on
/// it once and calls the group's executor with that kind as a constant, which folds the group's own
/// match away. Semantics stay entirely in the group modules.
pub mod hot {
    use super::Op;
    use crate::groups::alu::K;
    use crate::groups::bitmul::BitMulOp as B;
    use crate::groups::ctrl::{Cond, CtrlOp as C};
    use crate::groups::ldst::{Kind as L, Mode as M};

    /// Conditional branches are the most frequent class: one id per condition.
    fn cond(c: Cond, base: u8) -> u8 {
        base + match c {
            Cond::Eq => 0,
            Cond::Ne => 1,
            Cond::Lt => 2,
            Cond::Ge => 3,
            Cond::Gt => 4,
            Cond::Le => 5,
            Cond::LtU => 6,
            Cond::GeU => 7,
        }
    }

    macro_rules! ids {
        ($($name:ident),* $(,)?) => { ids!(@ 1u8, $($name,)*); };
        (@ $n:expr, $name:ident, $($rest:ident,)*) => { pub const $name: u8 = $n; ids!(@ $n + 1, $($rest,)*); };
        (@ $n:expr,) => {};
    }
    ids!(
        LDW_B, LDW_P, LDW_A, LDBU_B, LDBU_P, LDHU_B, LDH_B, LDB_B, LDA_B, LDA_A, LDD_B,
        STW_B, STW_P, STW_A, STB_B, STB_P, STH_B, STA_B, STD_B, LEA_B, LEA_A,
        MOV, ADD, SUB, AND, OR, XOR, SH, SHA, MOVA, MOVAA, MOVD, ADDA, SUBA, ADDSCA, SEL, SELN, CMP,
        MAX, MIN, MAXU, MINU, EQA, NEA, EQZA, NEZA,
        LTA, GEA, RSUB, CADD, LDBU_PRE, LDHU_P, STH_P, MUL, CALLI,
        CMP_EQ, CMP_NE, CMP_LT, CMP_LTU, CMP_GE, CMP_GEU,
        NOP, J, JL, JI, JAA_EQ, JAA_NE, JAZ_Z, JAZ_NZ, JBIT_Z, JBIT_NZ, LOOP, CALL, RET,
        JDD_EQ, JDD_NE, JDD_LT, JDD_GE, JDD_GT, JDD_LE, JDD_LTU, JDD_GEU,
        JDI_EQ, JDI_NE, JDI_LT, JDI_GE, JDI_GT, JDI_LE, JDI_LTU, JDI_GEU,
        EXTR, INSERT,
        ADD_F, SUB_F, MUL_F, MADD_F, MSUB_F, CMP_F, DIV_F,
        FTOI, FTOIZ, FTOU, FTOUZ, ITOF, UTOF, DIV, DIV_U, DVSTEP, DVSTEP_U,
        FPU,
    );

    /// Number of ids (0 included).
    pub const COUNT: usize = FPU as usize + 1;

    /// The operation never accesses memory or the bus: it cannot observe the instruction count nor
    /// set the core's attention flag (code writes, bus stop requests). Its handler skips both.
    pub const fn pure(id: u8) -> bool {
        !(mem(id) || matches!(id, 0 | CALLI | CALL | RET))
    }

    /// Ids with a side-effect-free fast path (`crate::op::try_fast`): plain loads/stores and LEA.
    pub const fn fast(id: u8) -> bool {
        mem(id) || matches!(id, LEA_B | LEA_A)
    }

    /// Plain loads and stores.
    pub const fn mem(id: u8) -> bool {
        matches!(
            id,
            LDW_B | LDW_P | LDW_A | LDBU_B | LDBU_P | LDHU_B | LDH_B | LDB_B | LDA_B | LDA_A | LDD_B | STW_B | STW_P
                | STW_A | STB_B | STB_P | STH_B | STA_B | STD_B | LDBU_PRE | LDHU_P | STH_P
        )
    }

    pub fn id(op: &Op) -> u8 {
        match op {
            Op::LdSt(o) => match (o.kind, o.mode) {
                (L::LdW, M::Base) => LDW_B,
                (L::LdW, M::Post) => LDW_P,
                (L::LdW, M::Abs) => LDW_A,
                (L::LdBu, M::Base) => LDBU_B,
                (L::LdBu, M::Post) => LDBU_P,
                (L::LdBu, M::Pre) => LDBU_PRE,
                (L::LdHu, M::Post) => LDHU_P,
                (L::StH, M::Post) => STH_P,
                (L::LdHu, M::Base) => LDHU_B,
                (L::LdH, M::Base) => LDH_B,
                (L::LdB, M::Base) => LDB_B,
                (L::LdA, M::Base) => LDA_B,
                (L::LdA, M::Abs) => LDA_A,
                (L::LdD, M::Base) => LDD_B,
                (L::StW, M::Base) => STW_B,
                (L::StW, M::Post) => STW_P,
                (L::StW, M::Abs) => STW_A,
                (L::StB, M::Base) => STB_B,
                (L::StB, M::Post) => STB_P,
                (L::StH, M::Base) => STH_B,
                (L::StA, M::Base) => STA_B,
                (L::StD, M::Base) => STD_B,
                (L::Lea, M::Base) => LEA_B,
                (L::Lea, M::Abs) => LEA_A,
                _ => 0,
            },
            Op::Alu(o) => match o.k {
                K::Mov => MOV,
                K::Add => ADD,
                K::Sub => SUB,
                K::And => AND,
                K::Or => OR,
                K::Xor => XOR,
                K::Sh => SH,
                K::Sha => SHA,
                K::MovA => MOVA,
                K::MovAA => MOVAA,
                K::MovD => MOVD,
                K::AddA => ADDA,
                K::SubA => SUBA,
                K::AddScA => ADDSCA,
                K::Sel => SEL,
                K::SelN => SELN,
                K::Cmp(cc, crate::groups::alu::Acc::Set) => {
                    use crate::groups::alu::Cc;
                    match cc {
                        Cc::Eq => CMP_EQ,
                        Cc::Ne => CMP_NE,
                        Cc::Lt => CMP_LT,
                        Cc::LtU => CMP_LTU,
                        Cc::Ge => CMP_GE,
                        Cc::GeU => CMP_GEU,
                    }
                }
                K::Cmp(..) => CMP,
                K::LtA => LTA,
                K::GeA => GEA,
                K::Rsub => RSUB,
                K::CAdd => CADD,
                K::Max => MAX,
                K::Min => MIN,
                K::MaxU => MAXU,
                K::MinU => MINU,
                K::EqA => EQA,
                K::NeA => NEA,
                K::EqzA => EQZA,
                K::NezA => NEZA,
                _ => 0,
            },
            Op::Ctrl(o) => match o {
                C::Nop => NOP,
                C::J { .. } => J,
                C::Jl { .. } => JL,
                C::Ji { .. } => JI,
                C::JDd { c, .. } => cond(*c, JDD_EQ),
                C::JDi { c, .. } => cond(*c, JDI_EQ),
                C::JAa { ne, .. } => if *ne { JAA_NE } else { JAA_EQ },
                C::JAz { ne, .. } => if *ne { JAZ_NZ } else { JAZ_Z },
                C::JBit { nz, .. } => if *nz { JBIT_NZ } else { JBIT_Z },
                C::Calli { .. } => CALLI,
                C::Loop { .. } => LOOP,
                C::Call { .. } => CALL,
                C::Ret => RET,
                _ => 0,
            },
            Op::BitMul(o) => match o {
                B::Extr { .. } => EXTR,
                B::Insert { .. } => INSERT,
                B::Mul { .. } => MUL,
                _ => 0,
            },
            Op::FpuDiv(o) => {
                use crate::groups::fpu_div::Kind as F;
                match o.kind {
                    F::AddF => ADD_F,
                    F::SubF => SUB_F,
                    F::MulF => MUL_F,
                    F::MaddF => MADD_F,
                    F::MsubF => MSUB_F,
                    F::CmpF => CMP_F,
                    F::DivF => DIV_F,
                    F::Ftoi => FTOI,
                    F::Ftoiz => FTOIZ,
                    F::Ftou => FTOU,
                    F::Ftouz => FTOUZ,
                    F::Itof => ITOF,
                    F::Utof => UTOF,
                    F::Div => DIV,
                    F::DivU => DIV_U,
                    F::DvStep => DVSTEP,
                    F::DvStepU => DVSTEP_U,
                    _ => FPU,
                }
            }
            Op::Invalid => 0,
        }
    }
}

/// A record's operation does not have the variant its hot id stands for: impossible, [`decode`]
/// derives the id from the operation ([`hot::id`]) and records are never modified afterwards. No
/// panic path, so the threaded-code handlers stay leaf functions.
macro_rules! mismatch {
    () => {{
        debug_assert!(false, "hot id does not match the operation");
        // SAFETY: see the macro documentation.
        unsafe { std::hint::unreachable_unchecked() }
    }};
}

/// Execute one predecoded instruction through its [`hot`] id (falls back to [`exec`]).
#[inline(always)]
pub fn exec_hot<B: Bus>(cpu: &mut Cpu<B>, insn: &Insn, pc: u32, len: u32) -> Flow {
    exec_id(insn.hot, cpu, insn, pc, len)
}

/// Fast path of the hot load/store ids (`ldst::exec_fast`): `Some(())` = executed and falls
/// through; `None` = not a hot load/store or not applicable, nothing changed (run [`exec_hot`]).
#[inline(always)]
pub fn try_fast<B: Bus>(id: u8, cpu: &mut Cpu<B>, insn: &Insn) -> Option<()> {
    use crate::groups::ldst::{Kind as L, LdStOp, Mode as M};
    macro_rules! f {
        ($k:ident, $m:ident) => {{
            let Op::LdSt(o) = &insn.op else { mismatch!() };
            ldst::exec_fast(cpu, &LdStOp { kind: L::$k, mode: M::$m, ..*o })
        }};
    }
    match id {
        hot::LDW_B => f!(LdW, Base),
        hot::LDW_P => f!(LdW, Post),
        hot::LDW_A => f!(LdW, Abs),
        hot::LDBU_B => f!(LdBu, Base),
        hot::LDBU_P => f!(LdBu, Post),
        hot::LDBU_PRE => f!(LdBu, Pre),
        hot::LDHU_P => f!(LdHu, Post),
        hot::STH_P => f!(StH, Post),
        hot::LDHU_B => f!(LdHu, Base),
        hot::LDH_B => f!(LdH, Base),
        hot::LDB_B => f!(LdB, Base),
        hot::LDA_B => f!(LdA, Base),
        hot::LDA_A => f!(LdA, Abs),
        hot::LDD_B => f!(LdD, Base),
        hot::STW_B => f!(StW, Base),
        hot::STW_P => f!(StW, Post),
        hot::STW_A => f!(StW, Abs),
        hot::STB_B => f!(StB, Base),
        hot::STB_P => f!(StB, Post),
        hot::STH_B => f!(StH, Base),
        hot::STA_B => f!(StA, Base),
        hot::STD_B => f!(StD, Base),
        hot::LEA_B => f!(Lea, Base),
        hot::LEA_A => f!(Lea, Abs),
        _ => None,
    }
}

/// [`exec_hot`] with the id passed separately: with a constant `id` the dispatch folds away (one
/// threaded-code handler per id, `crate::cpu`).
#[inline(always)]
pub fn exec_id<B: Bus>(id: u8, cpu: &mut Cpu<B>, insn: &Insn, pc: u32, len: u32) -> Flow {
    use crate::groups::alu::{AluOp, K};
    use crate::groups::bitmul::BitMulOp as Bm;
    use crate::groups::ctrl::CtrlOp as C;
    use crate::groups::ldst::{Kind as L, LdStOp, Mode as M};
    // Rebuild the operation with its kind (and mode) as constants: the inlined executor folds.
    macro_rules! ldst {
        ($k:ident, $m:ident) => {{
            let Op::LdSt(o) = &insn.op else { mismatch!() };
            ldst::exec(cpu, &LdStOp { kind: L::$k, mode: M::$m, ..*o }, pc, len)
        }};
    }
    macro_rules! alu {
        ($k:ident) => {{
            let Op::Alu(o) = &insn.op else { mismatch!() };
            alu::exec(cpu, &AluOp { k: K::$k, ..*o }, pc, len)
        }};
    }
    // Variants rebuilt from all of their fields: rustc turns such a rebuild into a copy of the
    // record's operation, so the executor's match only folds with the variant asserted.
    macro_rules! ctrl {
        ($v:ident { $($f:ident),* }) => {{
            let Op::Ctrl(o) = &insn.op else { mismatch!() };
            // SAFETY: `hot::id` gave this id to this variant only.
            unsafe { std::hint::assert_unchecked(matches!(o, C::$v { .. })) };
            ctrl::exec_inline(cpu, o, pc, len)
        }};
        ($v:ident) => {{
            ctrl::exec_inline(cpu, &C::$v, pc, len)
        }};
    }
    macro_rules! bm {
        ($v:ident) => {{
            let Op::BitMul(o) = &insn.op else { mismatch!() };
            // SAFETY: `hot::id` gave this id to this variant only.
            unsafe { std::hint::assert_unchecked(matches!(o, Bm::$v { .. })) };
            bitmul::exec_inline(cpu, o, pc, len)
        }};
    }
    macro_rules! cmp {
        ($cc:ident) => {{
            use crate::groups::alu::{Acc, Cc};
            let Op::Alu(o) = &insn.op else { mismatch!() };
            alu::exec(cpu, &AluOp { k: K::Cmp(Cc::$cc, Acc::Set), ..*o }, pc, len)
        }};
    }
    macro_rules! flag {
        ($v:ident, $flag:ident, $val:expr, { $($f:ident),* }) => {{
            let Op::Ctrl(C::$v { $($f,)* .. }) = insn.op else { mismatch!() };
            ctrl::exec_inline(cpu, &C::$v { $flag: $val, $($f),* }, pc, len)
        }};
    }
    macro_rules! jdd {
        ($v:ident, $cc:ident, $y:ident) => {{
            let Op::Ctrl(C::$v { a, $y, t, .. }) = insn.op else { mismatch!() };
            ctrl::exec_inline(cpu, &C::$v { c: crate::groups::ctrl::Cond::$cc, a, $y, t }, pc, len)
        }};
    }
    macro_rules! fpu {
        ($k:ident) => {{
            let Op::FpuDiv(o) = &insn.op else { mismatch!() };
            fpu_div::exec(cpu, &fpu_div::FpuDivOp { kind: fpu_div::Kind::$k, ..*o }, pc, len)
        }};
    }
    match id {
        hot::LDW_B => ldst!(LdW, Base),
        hot::LDW_P => ldst!(LdW, Post),
        hot::LDW_A => ldst!(LdW, Abs),
        hot::LDBU_B => ldst!(LdBu, Base),
        hot::LDBU_P => ldst!(LdBu, Post),
        hot::LDBU_PRE => ldst!(LdBu, Pre),
        hot::LDHU_P => ldst!(LdHu, Post),
        hot::STH_P => ldst!(StH, Post),
        hot::LDHU_B => ldst!(LdHu, Base),
        hot::LDH_B => ldst!(LdH, Base),
        hot::LDB_B => ldst!(LdB, Base),
        hot::LDA_B => ldst!(LdA, Base),
        hot::LDA_A => ldst!(LdA, Abs),
        hot::LDD_B => ldst!(LdD, Base),
        hot::STW_B => ldst!(StW, Base),
        hot::STW_P => ldst!(StW, Post),
        hot::STW_A => ldst!(StW, Abs),
        hot::STB_B => ldst!(StB, Base),
        hot::STB_P => ldst!(StB, Post),
        hot::STH_B => ldst!(StH, Base),
        hot::STA_B => ldst!(StA, Base),
        hot::STD_B => ldst!(StD, Base),
        hot::LEA_B => ldst!(Lea, Base),
        hot::LEA_A => ldst!(Lea, Abs),
        hot::MOV => alu!(Mov),
        hot::ADD => alu!(Add),
        hot::SUB => alu!(Sub),
        hot::AND => alu!(And),
        hot::OR => alu!(Or),
        hot::XOR => alu!(Xor),
        hot::SH => alu!(Sh),
        hot::SHA => alu!(Sha),
        hot::MOVA => alu!(MovA),
        hot::MOVAA => alu!(MovAA),
        hot::MOVD => alu!(MovD),
        hot::ADDA => alu!(AddA),
        hot::SUBA => alu!(SubA),
        hot::ADDSCA => alu!(AddScA),
        hot::SEL => alu!(Sel),
        hot::SELN => alu!(SelN),
        hot::CMP => {
            let Op::Alu(o) = &insn.op else { mismatch!() };
            alu::exec(cpu, o, pc, len)
        }
        hot::CMP_EQ => cmp!(Eq),
        hot::CMP_NE => cmp!(Ne),
        hot::CMP_LT => cmp!(Lt),
        hot::CMP_LTU => cmp!(LtU),
        hot::CMP_GE => cmp!(Ge),
        hot::CMP_GEU => cmp!(GeU),
        hot::LTA => alu!(LtA),
        hot::GEA => alu!(GeA),
        hot::RSUB => alu!(Rsub),
        hot::CADD => alu!(CAdd),
        hot::MAX => alu!(Max),
        hot::MIN => alu!(Min),
        hot::MAXU => alu!(MaxU),
        hot::MINU => alu!(MinU),
        hot::EQA => alu!(EqA),
        hot::NEA => alu!(NeA),
        hot::EQZA => alu!(EqzA),
        hot::NEZA => alu!(NezA),
        hot::NOP => Flow::Next,
        hot::J => ctrl!(J { t }),
        hot::JL => ctrl!(Jl { t }),
        hot::JI => ctrl!(Ji { a }),
        hot::JDD_EQ => jdd!(JDd, Eq, b),
        hot::JDD_NE => jdd!(JDd, Ne, b),
        hot::JDD_LT => jdd!(JDd, Lt, b),
        hot::JDD_GE => jdd!(JDd, Ge, b),
        hot::JDD_GT => jdd!(JDd, Gt, b),
        hot::JDD_LE => jdd!(JDd, Le, b),
        hot::JDD_LTU => jdd!(JDd, LtU, b),
        hot::JDD_GEU => jdd!(JDd, GeU, b),
        hot::JDI_EQ => jdd!(JDi, Eq, imm),
        hot::JDI_NE => jdd!(JDi, Ne, imm),
        hot::JDI_LT => jdd!(JDi, Lt, imm),
        hot::JDI_GE => jdd!(JDi, Ge, imm),
        hot::JDI_GT => jdd!(JDi, Gt, imm),
        hot::JDI_LE => jdd!(JDi, Le, imm),
        hot::JDI_LTU => jdd!(JDi, LtU, imm),
        hot::JDI_GEU => jdd!(JDi, GeU, imm),
        hot::JAA_EQ => flag!(JAa, ne, false, { a, b, t }),
        hot::JAA_NE => flag!(JAa, ne, true, { a, b, t }),
        hot::JAZ_Z => flag!(JAz, ne, false, { a, t }),
        hot::JAZ_NZ => flag!(JAz, ne, true, { a, t }),
        hot::JBIT_Z => flag!(JBit, nz, false, { a, n, t }),
        hot::JBIT_NZ => flag!(JBit, nz, true, { a, n, t }),
        hot::CALLI => ctrl!(Calli { a }),
        hot::LOOP => ctrl!(Loop { b, t }),
        hot::CALL => ctrl!(Call { t }),
        hot::RET => ctrl!(Ret),
        hot::EXTR => bm!(Extr),
        hot::MUL => bm!(Mul),
        hot::INSERT => bm!(Insert),
        hot::ADD_F => fpu!(AddF),
        hot::SUB_F => fpu!(SubF),
        hot::MUL_F => fpu!(MulF),
        hot::MADD_F => fpu!(MaddF),
        hot::MSUB_F => fpu!(MsubF),
        hot::CMP_F => fpu!(CmpF),
        hot::DIV_F => fpu!(DivF),
        hot::FTOI => fpu!(Ftoi),
        hot::FTOIZ => fpu!(Ftoiz),
        hot::FTOU => fpu!(Ftou),
        hot::FTOUZ => fpu!(Ftouz),
        hot::ITOF => fpu!(Itof),
        hot::UTOF => fpu!(Utof),
        hot::DIV => fpu!(Div),
        hot::DIV_U => fpu!(DivU),
        hot::DVSTEP => fpu!(DvStep),
        hot::DVSTEP_U => fpu!(DvStepU),
        hot::FPU => {
            let Op::FpuDiv(o) = &insn.op else { mismatch!() };
            fpu_div::exec(cpu, o, pc, len)
        }
        _ => exec(cpu, &insn.op, pc, len),
    }
}

/// Number of groups that claim an encoding (must be 0 or 1; checked exhaustively by a test).
pub fn claims(lo: u16, hi: u16, pc: u32, arch: Arch) -> usize {
    alu::decode(lo, hi, pc, arch).is_some() as usize
        + bitmul::decode(lo, hi, pc, arch).is_some() as usize
        + ldst::decode(lo, hi, pc, arch).is_some() as usize
        + ctrl::decode(lo, hi, pc, arch).is_some() as usize
        + fpu_div::decode(lo, hi, pc, arch).is_some() as usize
}

impl Op {
    /// Whether the block must end after this instruction.
    #[inline]
    pub fn ends_block(&self) -> bool {
        match self {
            Op::Alu(o) => alu::ends_block(o),
            Op::BitMul(o) => bitmul::ends_block(o),
            Op::LdSt(o) => ldst::ends_block(o),
            Op::Ctrl(o) => ctrl::ends_block(o),
            Op::FpuDiv(o) => fpu_div::ends_block(o),
            Op::Invalid => true,
        }
    }
}

/// Execute one predecoded instruction located at `pc`.
#[inline(always)]
pub fn exec<B: Bus>(cpu: &mut Cpu<B>, op: &Op, pc: u32, len: u32) -> Flow {
    match op {
        Op::Alu(o) => alu::exec(cpu, o, pc, len),
        Op::BitMul(o) => bitmul::exec(cpu, o, pc, len),
        Op::LdSt(o) => ldst::exec(cpu, o, pc, len),
        Op::Ctrl(o) => ctrl::exec(cpu, o, pc, len),
        Op::FpuDiv(o) => fpu_div::exec(cpu, o, pc, len),
        Op::Invalid => Flow::trap(Trap::IOPC, pc),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn op_size_budget() {
        assert!(std::mem::size_of::<Op>() <= 16, "Op is {} bytes", std::mem::size_of::<Op>());
        assert!(std::mem::size_of::<Insn>() <= 32);
    }

    /// Every 16-bit encoding and a deterministic sample of 32-bit encodings (all op1 values) is
    /// claimed by at most one group, for both architectures.
    #[test]
    fn groups_never_overlap() {
        for arch in [Arch::Tc13, Arch::Tc131, Arch::Tc16] {
            for lo in (0..=u16::MAX).step_by(2) {
                assert!(claims(lo, 0, 0x8000_0000, arch) <= 1, "16-bit {lo:#06x} claimed twice ({arch:?})");
            }
            let mut x = 0x1234_5678u32;
            for op1 in (1..=0xFFu32).step_by(2) {
                for _ in 0..4096 {
                    x ^= x << 13;
                    x ^= x >> 17;
                    x ^= x << 5;
                    let w = (x & !0xFF) | op1;
                    let n = claims(w as u16, (w >> 16) as u16, 0x8000_0000, arch);
                    assert!(n <= 1, "32-bit {w:#010x} claimed {n} times ({arch:?})");
                }
            }
        }
    }

    /// The threaded-code handlers skip the instruction count and the attention check for
    /// `hot::pure` ids and run `try_fast` for `hot::fast` ids: no pure operation may access memory,
    /// and every fast id must be a plain load/store or LEA (all 16-bit and sampled 32-bit encodings).
    #[test]
    fn hot_id_classes_match_operations() {
        use crate::groups::ctrl::CtrlOp as C;
        use crate::groups::ldst::Kind as L;
        for arch in [Arch::Tc13, Arch::Tc131, Arch::Tc16] {
            let mut x = 0x2468_ACE1u32;
            let words = (0..=u16::MAX).step_by(2).map(|lo| lo as u32).chain((0..1 << 20).map(|_| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                x | 1
            }));
            for w in words {
                let i = decode(w as u16, (w >> 16) as u16, 0x8000_0000, arch);
                if hot::fast(i.hot) {
                    assert!(matches!(i.op, Op::LdSt(o) if o.mode != crate::groups::ldst::Mode::Rev && o.mode != crate::groups::ldst::Mode::Circ), "{w:#x}: {:?}", i.op);
                }
                if hot::pure(i.hot) {
                    let ok = match i.op {
                        Op::LdSt(o) => o.kind == L::Lea,
                        Op::Ctrl(c) => !matches!(c, C::Call { .. } | C::Calli { .. } | C::Ret | C::Fcall { .. } | C::Fcalli { .. } | C::Fret | C::Rfe | C::Rfm | C::Svlcx | C::Rslcx | C::Bisr { .. }),
                        Op::Invalid => false,
                        _ => true,
                    };
                    assert!(ok, "{w:#x}: pure id {} for {:?}", i.hot, i.op);
                }
            }
        }
    }
}
