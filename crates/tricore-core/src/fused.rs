//! Whole-block superinstructions (Ertl & Gregg 2003; Casey et al. 2007), chosen from the dynamic
//! block profiles of the firmware (`docs/PERFORMANCE.md` §8): a block whose hot-id sequence matches
//! a pattern gets one handler for all of its instructions instead of one per instruction.
//!
//! Semantics are those of the per-instruction handlers, in the same order: every instruction runs
//! its own fast path or executor ([`step`]); an instruction whose fast path does not apply hands
//! over to the full executor at its own record ([`slow`]), which continues the block with the
//! per-instruction handlers. A block that branches back to its own start runs again in place while
//! [`chain`] would continue there (whole block within the budget, no hook stop), so the instruction
//! count, budget ends and everything the firmware observes stay exactly those of the
//! per-instruction path. Nothing is skipped: every iteration executes every instruction.

use crate::bus::Bus;
use crate::cpu::{chain, done, next, slow, Cpu, Handler};
use crate::flags::ov32;
use crate::groups::alu::K;
use crate::groups::ctrl::{Cond, CtrlOp};
use crate::groups::ldst::{self, Kind, Mode};
use crate::op::{self, hot, Flow, Insn, Op};

/// One instruction of a fused block: `Ok` = fell through, `Err(None)` = its fast path does not
/// apply (nothing changed), `Err(Some(flow))` = it ended the block (branch, trap).
#[inline(always)]
fn step<B: Bus, const ID: u8>(cpu: &mut Cpu<B>, i: &Insn) -> Result<(), Option<Flow>> {
    // no generic or out-of-line executors (they would keep a stack slot alive), no bus users
    const { assert!(ID != 0 && ID != hot::FPU && (hot::fast(ID) || hot::pure(ID))) };
    #[cfg(feature = "trace-data-stores")]
    if !hot::pure(ID) {
        cpu.exec_pc = i.pc;
    }
    if hot::fast(ID) {
        return op::try_fast(ID, cpu, i).ok_or(None);
    }
    match op::exec_id(ID, cpu, i, i.pc, i.len as u32) {
        Flow::Next => Ok(()),
        f => Err(Some(f)),
    }
}

/// The executing block (`len` instructions) branched back to its own start: true (and the block's
/// instruction count advanced) if [`chain`] would continue there — the whole block fits the budget
/// and the block has no hook stop.
#[inline(always)]
fn again<B: Bus>(cpu: &mut Cpu<B>, len: u64) -> bool {
    let hook = cpu.cache.block(cpu.t_blk).hook;
    let base = cpu.t_base + len;
    if !hook && cpu.t_end - base >= len {
        cpu.t_base = base;
        true
    } else {
        false
    }
}

macro_rules! fused {
    ($($(#[$doc:meta])* $name:ident = [$($id:ident),+];)*) => {
        $(
            $(#[$doc])*
            #[allow(non_snake_case)]
            fn $name<B: Bus>(cpu: &mut Cpu<B>, i: &Insn) {
                let r = i as *const Insn;
                let len = [$(hot::$id),+].len();
                loop {
                    let mut k = 0;
                    $(
                        // SAFETY: `select` matched the block's whole id sequence: records 0..len exist.
                        let ik = unsafe { &*r.add(k) };
                        match step::<B, { hot::$id }>(cpu, ik) {
                            Ok(()) => {}
                            Err(None) => return slow(cpu, ik),
                            Err(Some(Flow::Jump(t))) if t == i.pc && again(cpu, len as u64) => continue,
                            Err(Some(flow)) => return done(cpu, ik, flow, true),
                        }
                        k += 1;
                    )+
                    debug_assert_eq!(k, len);
                    // the last instruction fell through to the block's `end` record
                    // SAFETY: as above.
                    return next(cpu, unsafe { &*r.add(len - 1) });
                }
            }
        )*

        /// Superinstruction for a block whose instructions have exactly these hot ids.
        fn select_seq<B: Bus>(ids: &[u8]) -> Option<Handler<B>> {
            $(if ids == [$(hot::$id),+] {
                return Some($name::<B>);
            })*
            None
        }
    };
}

fused! {
    /// SIMOS18.1 software delay loop `ld.hu d15,[a4]; add d15,#-1; st.h [a4],d15; ld.hu d15,[a4];
    /// jnz d15` (0x800FB2D4, 49 % of the instructions of `perf_determinism`); see [`countdown`].
    DELAY = [LDHU_B, ADD, STH_B, LDHU_B, JDI_NE];
    /// SIMOS18.1 table-driven CRC-32 (0x8001F552, 9 %).
    CRC_S18 = [SH, LDBU_P, SH, XOR, ADDSCA, ADD, LDW_B, XOR, JDI_NE];
    /// SIMOS18.1 word search loop (0x80101FFE / 0x80102020, 5 %).
    SEARCH = [LDW_P, CMP_EQ, SEL, ADD, JDD_GE];
    /// DQ200_G2 table-driven CRC (0x800EDCC8, 15 %).
    CRC_DQ200 = [EXTR, MOVA, LDBU_P, XOR, LEA_B, ADDSCA, SH, LDW_B, XOR, ADD, JDI_GEU];
    /// DQ250 bootloader CRC (0xA001634C, 14 % of its boot).
    CRC_DQ250 = [LDW_B, EXTR, LDBU_P, XOR, AND, ADDSCA, SH, LDW_B, XOR, STW_B, ADD, JDD_LTU];
}

/// Superinstruction for a cached block (its records without the `end` record), if one applies.
pub(crate) fn select<B: Bus>(insns: &[Insn]) -> Option<Handler<B>> {
    if countdown_shape(insns) {
        return Some(countdown::<B>);
    }
    let ids: Vec<u8> = insns.iter().map(|i| i.hot).collect();
    select_seq::<B>(&ids)
}

/// `ld.hu d,[a]o; add d,d,#y; st.h [a]o,d; ld.hu d,[a]o; jne d,#c,<start>`: one halfword counter
/// in memory, stepped until it equals `c` (the [`DELAY`] block with all operands related).
fn countdown_shape(insns: &[Insn]) -> bool {
    let [i0, i1, i2, i3, i4] = insns else { return false };
    let ids = [i0.hot, i1.hot, i2.hot, i3.hot, i4.hot];
    if ids != [hot::LDHU_B, hot::ADD, hot::STH_B, hot::LDHU_B, hot::JDI_NE] {
        return false;
    }
    let (Op::LdSt(l0), Op::Alu(add), Op::LdSt(st), Op::LdSt(l1), Op::Ctrl(CtrlOp::JDi { c: Cond::Ne, a: j, t, .. })) =
        (i0.op, i1.op, i2.op, i3.op, i4.op)
    else {
        return false;
    };
    let same = |o: &ldst::LdStOp, k: Kind| o.kind == k && o.mode == Mode::Base && o.chk == 0 && (o.a, o.b, o.off) == (l0.a, l0.b, l0.off);
    same(&l0, Kind::LdHu)
        && same(&st, Kind::StH)
        && same(&l1, Kind::LdHu)
        && add.k == K::Add
        && add.is_imm
        && (add.c, add.a) == (l0.a, l0.a)
        && j == l0.a
        && t == i0.pc
}

/// [`countdown_shape`] block. The address checks of the three accesses (EA, MEM/ALN/MPN/MPP, host
/// memory without translated code) depend only on `A[a]`, the offset and PSW.IO, which the block
/// does not write: they are made once, then every iteration performs the load, the ADD with its
/// PSW update, the store, the load and the compare, exactly as the per-instruction handlers. If
/// the checks fail, [`DELAY`] runs the block instruction by instruction.
fn countdown<B: Bus>(cpu: &mut Cpu<B>, i: &Insn) {
    // SAFETY: `countdown_shape` matched the block's five records.
    let r = unsafe { std::slice::from_raw_parts(i as *const Insn, 5) };
    let (Op::LdSt(ld), Op::Alu(add), Op::Ctrl(CtrlOp::JDi { imm, .. })) = (&r[0].op, &r[1].op, &r[4].op) else {
        return DELAY(cpu, i);
    };
    let ea = match ldst::fast(cpu, ld, 2, 2) {
        Some((ea, None)) => ea,
        _ => return DELAY(cpu, i),
    };
    let p = match (cpu.mem.host_r(ea, 2), cpu.mem.host_w_plain(ea, 2)) {
        (Some(p), Some(q)) if std::ptr::eq(p, q) => q,
        _ => return DELAY(cpu, i),
    };
    let (d, y, c) = (ld.a as usize & 15, add.imm as i32 as i64, *imm as i32 as u32);
    loop {
        // ld.hu d,[a]o — D[d] is overwritten by the ADD below
        // SAFETY: p was validated for 2-byte accesses (host_r / host_w_plain).
        let v = u16::from_le_bytes(unsafe { *(p as *const [u8; 2]) }) as u32;
        #[cfg(feature = "trace-data-stores")]
        cpu.bus.data_read(r[0].pc, ea, 2, v as u64);
        // add d,d,#y (V/AV overwritten, SV/SAV sticky: as alu::exec)
        let s = ov32(&mut cpu.psw, v as i32 as i64 + y);
        // st.h [a]o,d
        #[cfg(feature = "trace-data-stores")]
        cpu.bus.data_write(r[2].pc, ea, 2, (s as u16) as u64);
        // SAFETY: as above.
        unsafe { *(p as *mut [u8; 2]) = (s as u16).to_le_bytes() };
        // ld.hu d,[a]o
        // SAFETY: as above.
        let w = u16::from_le_bytes(unsafe { *(p as *const [u8; 2]) }) as u32;
        #[cfg(feature = "trace-data-stores")]
        cpu.bus.data_read(r[3].pc, ea, 2, w as u64);
        cpu.d[d] = w;
        // jne d,#c,<start>
        if w == c {
            return next(cpu, &r[4]);
        }
        if !again(cpu, 5) {
            return chain(cpu, 5, i.pc);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{Arch, Cpu, Exit, Memory, NullBus};

    /// The SIMOS18.1 delay loop (bytes of 0x800FB2D4) followed by `j .`, counter in another page:
    /// one long `run` (threaded code, chaining, [`super::countdown`] in place) ends in exactly the
    /// state of single-instruction steps (`run(1)`: the per-instruction path), budget ends included.
    #[test]
    fn delay_loop_matches_single_steps() {
        // ld.hu d15,[a4]; add d15,#-1; st.h [a4],d15; ld.hu d15,[a4]; jnz d15,<start>; j .
        let code = [0x09, 0x4F, 0xC0, 0x08, 0xC2, 0xFF, 0xB4, 0x4F, 0x09, 0x4F, 0xC0, 0x08, 0xEE, 0xFA, 0x3C, 0x00];
        let mut pc = 0xD000_0000;
        let block: Vec<_> = [4u32, 2, 2, 4, 2]
            .iter()
            .map(|&len| {
                let o = (pc - 0xD000_0000) as usize;
                let (lo, hi) = (u16::from_le_bytes([code[o], code[o + 1]]), u16::from_le_bytes([code[o + 2], code[o + 3]]));
                let i = crate::op::decode(lo, hi, pc, Arch::Tc16);
                pc += len;
                i
            })
            .collect();
        assert!(super::countdown_shape(&block), "the loop uses the countdown superinstruction");
        let make = |n: u16| {
            let mut m = Memory::new();
            let r = m.add_owned(vec![0; 0x2_0000], true);
            m.map(0xD000_0000, r, 0, 0x2_0000);
            let mut c = Cpu::new(Arch::Tc16, m, NullBus);
            c.write_bytes(0xD000_0000, &code).unwrap();
            c.write_bytes(0xD001_0000, &n.to_le_bytes()).unwrap();
            c.a[4] = 0xD001_0000;
            c.pc = 0xD000_0000;
            c
        };
        for (n, budgets) in [(700u16, [5000u64, 1234, 3]), (0, [400_000, 3, 2])] {
            let (mut a, mut b) = (make(n), make(n));
            for budget in budgets {
                assert_eq!(a.run(budget), Exit::Budget);
                for _ in 0..budget {
                    assert_eq!(b.run(1), Exit::Budget);
                }
                let mem = |c: &Cpu<NullBus>| {
                    let mut m = [0u8; 2];
                    assert!(c.read_bytes(0xD001_0000, &mut m));
                    m
                };
                assert_eq!((a.icount, a.pc, a.psw, a.d, a.a, mem(&a)), (b.icount, b.pc, b.psw, b.d, b.a, mem(&b)), "n {n}, budget {budget}");
            }
        }
    }

    #[test]
    #[cfg(feature = "trace-data-stores")]
    fn delay_loop_notifies_observers_with_exact_pcs() {
        #[derive(Default)]
        struct Obs(Vec<(bool, u32, u32, u8, u64)>);
        impl crate::Bus for Obs {
            fn read(&mut self, _: u32, _: u8) -> Result<u64, crate::BusError> { Err(crate::BusError) }
            fn write(&mut self, _: u32, _: u8, _: u64) -> Result<(), crate::BusError> { Err(crate::BusError) }
            fn data_read(&mut self, pc: u32, addr: u32, size: u8, val: u64) {
                self.0.push((false, pc, addr, size, val));
            }
            fn data_write(&mut self, pc: u32, addr: u32, size: u8, val: u64) {
                self.0.push((true, pc, addr, size, val));
            }
        }
        let code = [0x09, 0x4F, 0xC0, 0x08, 0xC2, 0xFF, 0xB4, 0x4F, 0x09, 0x4F, 0xC0, 0x08, 0xEE, 0xFA, 0x3C, 0x00];
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x2_0000], true);
        m.map(0xD000_0000, r, 0, 0x2_0000);
        let mut c = Cpu::new(Arch::Tc16, m, Obs::default());
        c.write_bytes(0xD000_0000, &code).unwrap();
        c.write_bytes(0xD001_0000, &2u16.to_le_bytes()).unwrap();
        c.a[4] = 0xD001_0000;
        c.pc = 0xD000_0000;
        assert_eq!(c.run(10), Exit::Budget);
        assert_eq!(
            c.bus.0,
            vec![
                (false, 0xD000_0000, 0xD001_0000, 2, 2),
                (true, 0xD000_0006, 0xD001_0000, 2, 1),
                (false, 0xD000_0008, 0xD001_0000, 2, 1),
                (false, 0xD000_0000, 0xD001_0000, 2, 1),
                (true, 0xD000_0006, 0xD001_0000, 2, 0),
                (false, 0xD000_0008, 0xD001_0000, 2, 0),
            ]
        );
    }
}
