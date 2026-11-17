//! Group C — every load/store and memory instruction: LD.*/ST.* (all addressing modes: absolute,
//! base+short/long offset, pre/post-increment, circular, bit-reverse, and the 16-bit SLR/SLRO/SRO/SSR/
//! SSRO/SC forms), LEA, LDMST, SWAP.W, ST.T, LDLCX/LDUCX/STLCX/STUCX, CACHEA.*/CACHEI.*.
//!
//! Manuals (`vault/docs/infineon/`): V1 = TriCore V1.6 Vol. 1, V2 = Vol. 2 (instruction pages cited per
//! class below), V131-1/V131-2 = TriCore 1 V1.3 & V1.3.1 Vol. 1/2 (V1.3.8).
//!
//! * Addressing modes: V1 §2.5 (p. 2-7..2-11) and the RTL of each instruction page. Circular:
//!   `EA = A[b] + index`, `new_index = index + off10; new_index < 0 ? new_index + length :
//!   new_index % length`, `A[b+1] = {length, new_index[15:0]}`; LD.W/LD.D/ST.W/ST.D access the buffer
//!   in halfwords and LD.DA/ST.DA in words, each wrapping at `length` (V1 Figure 2-6 p. 2-10).
//!   Bit-reverse: `A[b+1] = {incr, reverse16(reverse16(index) + reverse16(incr))}`.
//! * Traps, checked in the priority order of V1 Table 6-2 (p. 6-12): PRIV (CACHEA.I/CACHEI.I outside
//!   Supervisor mode) > GRWP (write to A0/A1/A8/A9 while PSW.GW = 0, V1 p. 3-7/6-7) > MEM (offset
//!   leaves the base segment, or the datum spans a segment boundary, V1 p. 6-7) > ALN (V1 Table 2-1
//!   p. 2-4; circular buffer base not 8-byte aligned, length not a multiple of the data size, or
//!   index >= length, V1 p. 2-10) > MPN (EA = 0) > MPP (User-0 access to a peripheral segment) > DSE
//!   (load bus error) / DAE (store bus error, asynchronous: the instruction completes).
//!   A trapping load/store changes no register; a DAE store keeps its address-register update.
//! * Odd register-pair specifiers (`E[a]`, `P[a]`, `P[b]`) raise OPD (V1 p. 6-7: "may be raised"; the
//!   alternative is implementation defined, so the architected trap is used).
//! * Unspecified result (V1 p. 2-7): when the loaded register is also updated by the addressing mode
//!   (`ld.a a2, [a2+]4`), the loaded value wins; a store of the updated register stores the original.
//! * TC1.3.1 deltas (V131-2, encodings verified on the rendered pages, e.g. LD.W p. 3-198/3-199,
//!   ST.T p. 3-427, STLCX p. 3-432): no BOL forms of LD.B/LD.BU/LD.H/LD.HU/ST.A/ST.B/ST.H and no
//!   CACHEI.I (IOPC via `Op::Invalid`); double-word LD.D/ST.D may be halfword aligned only in cacheable
//!   memory or the DSPR (V131-1 p. 2-4; default attributes V131-1 Table 11 p. 8-3), TC1.6 only requires
//!   word alignment in peripheral segments (V1 p. 2-4, PMA0 V1 p. 8-6).
//! * Not modelled: the range-based protection system (MPR/MPW with SYSCON.PROTEN = 1, V1 ch. 9) and
//!   the data cache (CACHE* only perform their address-register updates; memory is always coherent).
//! * Unicorn 2.1.4 defects (tests `matches_unicorn_on_random_operands`, `unicorn_disagreements`): BO
//!   LD.BU pre-increment sign-extends (`testutil::oracle::KNOWN_WRONG`); SRO `LD.H D15, [A[b]]off4`
//!   adds off4 instead of 2 * off4 (V2 p. 3-160; firmware uses this form); CACHEI.I raises IOPC;
//!   circular LD.W/LD.D/ST.W that run off the buffer end access past it instead of wrapping (V1
//!   Figure 2-6); a circular index update with
//!   |offset| > length borrows from the length field; none of PRIV/GRWP/MEM/ALN/MPN/OPD is raised.
//! * Hot path: Abs/Base/Post/Pre accesses that raise no trap skip the ordered checks (`fast`).

use crate::arch::{psw, Arch, Trap, CSFR_TABLE};
use crate::context::{Slot, LOWER, UPPER};
use crate::flags::reverse16;
use crate::fmt::{sext, word, Abs, Absb, Bo, Bol, Sc, Slr, Slro, Sro, Ssr, Ssro};
use crate::{Bus, Cpu, Flow};

/// Mnemonics owned by this group (manual spelling).
pub const MNEMONICS: &[&str] = &[
    "CACHEA.I", "CACHEA.W", "CACHEA.WI", "CACHEI.I", "CACHEI.W", "CACHEI.WI", "LD.A", "LD.B",
    "LD.BU", "LD.D", "LD.DA", "LD.H", "LD.HU", "LD.Q", "LD.W", "LDLCX", "LDMST", "LDUCX", "LEA",
    "ST.A", "ST.B", "ST.D", "ST.DA", "ST.H", "ST.Q", "ST.T", "ST.W", "STLCX", "STUCX", "SWAP.W"
];

/// What an instruction transfers. Load/store order follows BO `op2[3:0]` (V2 LD.B p. 3-150 .. LD.Q
/// p. 3-162: B, BU, H, HU, W, D, A, DA, Q).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    LdB, LdBu, LdH, LdHu, LdW, LdD, LdA, LdDa, LdQ,
    StB, StH, StW, StD, StA, StDa, StQ,
    Lea, SwapW, Ldmst,
    /// `a` = bit value, `b` = bpos3.
    StT,
    Ldlcx, Lducx, Stlcx, Stucx,
    CacheAW, CacheAWI, CacheAI, CacheIW, CacheII, CacheIWI,
    /// Valid encoding with an odd register-pair specifier: OPD trap.
    Opd,
}

/// Addressing mode, normalised: 16-bit implicit-base forms are `Base` with a pre-scaled offset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// `EA = off` (the absolute address, precomputed).
    Abs,
    /// `EA = A[b] + off`.
    Base,
    /// `EA = A[b]; A[b] = EA + off`.
    Post,
    /// `EA = A[b] + off; A[b] = EA`.
    Pre,
    /// `P[b]` bit-reverse.
    Rev,
    /// `P[b]` circular, `off` = index increment.
    Circ,
}

/// One predecoded load/store (12 bytes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LdStOp {
    pub kind: Kind,
    pub mode: Mode,
    /// Data/address register (or ST.T bit value).
    pub a: u8,
    /// Base register (or ST.T bit position).
    pub b: u8,
    /// `GLOBAL` / `SUPERVISOR` checks needed before execution.
    pub chk: u8,
    pub off: i32,
}

/// Writes A0/A1/A8/A9: GRWP when PSW.GW = 0.
const GLOBAL: u8 = 1;
/// Supervisor-mode only: PRIV otherwise.
const SUPERVISOR: u8 = 2;

impl LdStOp {
    const fn new(kind: Kind, mode: Mode, a: u32, b: u32, off: i32) -> Self {
        Self { kind, mode, a: a as u8, b: b as u8, chk: 0, off }
    }

    fn finish(mut self) -> Self {
        use Kind::*;
        let pair_b = matches!(self.mode, Mode::Rev | Mode::Circ);
        if (matches!(self.kind, LdD | StD | LdDa | StDa | Ldmst) && self.a & 1 != 0) || (pair_b && self.b & 1 != 0) {
            return Self { kind: Opd, ..self };
        }
        let global = |r: u8| matches!(r, 0 | 1 | 8 | 9);
        if (matches!(self.kind, LdA | LdDa | Lea) && global(self.a))
            || (matches!(self.mode, Mode::Post | Mode::Pre) && global(self.b))
            || (pair_b && global(self.b + 1))
        {
            self.chk |= GLOBAL;
        }
        if matches!(self.kind, CacheAI | CacheII) {
            self.chk |= SUPERVISOR;
        }
        self
    }
}

pub fn decode(lo: u16, hi: u16, _pc: u32, arch: Arch) -> Option<LdStOp> {
    let op = if lo & 1 == 0 { decode16(lo)? } else { decode32(word(lo, hi), arch)? };
    Some(op.finish())
}

fn decode32(w: u32, arch: Arch) -> Option<LdStOp> {
    use Kind::*;
    match w as u8 {
        // ABS, op2 = [27:26]: LD.B/BU/H/HU p. 3-150/3-151/3-158/3-160, LD.Q p. 3-162, LD.W/D/A/DA
        // p. 3-164/3-154/3-147/3-156, ST.B/H p. 3-346/3-353, ST.Q p. 3-356, ST.W/D/A/DA
        // p. 3-359/3-349/3-343/3-351, LEA p. 3-171, SWAP.W p. 3-376, LDMST p. 3-168,
        // STLCX/STUCX/LDLCX/LDUCX p. 3-362/3-363/3-167/3-170. EA = {off18[17:14], 14'b0, off18[13:0]}.
        0x05 | 0x15 | 0x25 | 0x45 | 0x65 | 0x85 | 0xA5 | 0xC5 | 0xE5 => {
            let f = Abs::from(w);
            let k = match (f.op1, f.op2) {
                (0x05, 0) => LdB,
                (0x05, 1) => LdBu,
                (0x05, 2) => LdH,
                (0x05, 3) => LdHu,
                (0x45, 0) => LdQ,
                (0x85, 0) => LdW,
                (0x85, 1) => LdD,
                (0x85, 2) => LdA,
                (0x85, 3) => LdDa,
                (0x25, 0) => StB,
                (0x25, 2) => StH,
                (0x65, 0) => StQ,
                (0xA5, 0) => StW,
                (0xA5, 1) => StD,
                (0xA5, 2) => StA,
                (0xA5, 3) => StDa,
                (0xC5, 0) => Lea,
                (0xE5, 0) => SwapW,
                (0xE5, 1) => Ldmst,
                (0x15, 0) => Stlcx,
                (0x15, 1) => Stucx,
                (0x15, 2) => Ldlcx,
                (0x15, 3) => Lducx,
                _ => return None,
            };
            Some(LdStOp::new(k, Mode::Abs, f.a, 0, f.ea() as i32))
        }
        // ABSB ST.T p. 3-358: M(EA, byte) = (M(EA, byte) AND ~(1 << bpos3)) | (b << bpos3).
        0xD5 => {
            let f = Absb::from(w);
            (f.op2 == 0).then(|| LdStOp::new(StT, Mode::Abs, f.b, f.bpos3, f.ea() as i32))
        }
        0x09 | 0x29 | 0x49 | 0x69 | 0x89 | 0xA9 => bo(Bo::from(w), arch),
        // BOL: EA = A[b] + sign_ext(off16). LD.W p. 3-165, ST.W p. 3-360, LD.A p. 3-148, LEA p. 3-171
        // (TC1.3.1 too); TC1.6 only: LD.BU p. 3-152, LD.B p. 3-151, LD.HU p. 3-161, LD.H p. 3-159,
        // ST.A p. 3-344, ST.B p. 3-347, ST.H p. 3-354 (absent from V131-2).
        op1 => {
            let k = match op1 {
                0x19 => LdW,
                0x59 => StW,
                0x99 => LdA,
                0xD9 => Lea,
                _ if arch != Arch::Tc16 => return None,
                0x39 => LdBu,
                0x79 => LdB,
                0xB9 => LdHu,
                0xC9 => LdH,
                0xB5 => StA,
                0xE9 => StB,
                0xF9 => StH,
                _ => return None,
            };
            let f = Bol::from(w);
            Some(LdStOp::new(k, Mode::Base, f.a, f.b, sext(f.off16, 16) as i32))
        }
    }
}

/// BO: op2[5:4] selects the mode (0x09/0x49/0x89: 0 post-increment, 1 pre-increment, 2 base + short
/// offset; 0x29/0x69/0xA9: 0 bit-reverse, 1 circular), op2[3:0] the operation (V2 encoding boxes of
/// LD.* p. 3-147..3-165, ST.* p. 3-343..3-360, CACHEA.* p. 3-47..3-52, CACHEI.* p. 3-53..3-58,
/// SWAP.W p. 3-376, LDMST p. 3-168, LEA p. 3-171, LDLCX/LDUCX/STLCX/STUCX p. 3-167/3-170/3-362/3-363).
fn bo(f: Bo, arch: Arch) -> Option<LdStOp> {
    use Kind::*;
    let indexed = f.op1 & 0x20 != 0;
    let mode = match (indexed, f.op2 >> 4) {
        (false, 0) => Mode::Post,
        (false, 1) => Mode::Pre,
        (false, 2) => Mode::Base,
        (true, 0) => Mode::Rev,
        (true, 1) => Mode::Circ,
        _ => return None,
    };
    let n = f.op2 & 0xF;
    let k = match f.op1 & !0x20 {
        0x09 => [LdB, LdBu, LdH, LdHu, LdW, LdD, LdA, LdDa, LdQ].get(n as usize).copied()?,
        0x89 => match n {
            0x0 => StB,
            0x2 => StH,
            0x4 => StW,
            0x5 => StD,
            0x6 => StA,
            0x7 => StDa,
            0x8 => StQ,
            0xC => CacheAW,
            0xD => CacheAWI,
            0xE => CacheAI,
            0xA if !indexed && arch == Arch::Tc16 => CacheII,
            0xB if !indexed => CacheIW,
            0xF if !indexed => CacheIWI,
            _ => return None,
        },
        _ => match (n, mode) {
            (0, _) => SwapW,
            (1, _) => Ldmst,
            (4, Mode::Base) => Ldlcx,
            (5, Mode::Base) => Lducx,
            (6, Mode::Base) => Stlcx,
            (7, Mode::Base) => Stucx,
            (8, Mode::Base) => Lea,
            _ => return None,
        },
    };
    let off = if mode == Mode::Rev { 0 } else { sext(f.off10, 10) as i32 };
    Some(LdStOp::new(k, mode, f.a, f.b, off))
}

/// 16-bit forms (same encodings in V131-2). Offsets are zero-extended and scaled by the access size.
fn decode16(h: u16) -> Option<LdStOp> {
    use Kind::*;
    use Mode::{Base, Post};
    let op = |k: Kind, m: Mode, a: u32, b: u32, off: u32| Some(LdStOp::new(k, m, a, b, off as i32));
    let (sc, slr, slro, sro, ssr, ssro) =
        (Sc::from(h), Slr::from(h), Slro::from(h), Sro::from(h), Ssr::from(h), Ssro::from(h));
    match h as u8 {
        // SC: M(A[10] + zero_ext(4 * const8), word). LD.W p. 3-165, ST.W p. 3-360, LD.A p. 3-148,
        // ST.A p. 3-344; data register D15/A15.
        0x58 => op(LdW, Base, 15, 10, 4 * sc.const8),
        0x78 => op(StW, Base, 15, 10, 4 * sc.const8),
        0xD8 => op(LdA, Base, 15, 10, 4 * sc.const8),
        0xF8 => op(StA, Base, 15, 10, 4 * sc.const8),
        // SLR: X[c] = M(A[b]); post-increment form A[b] += size. LD.BU p. 3-152/3-153, LD.H p. 3-159,
        // LD.W p. 3-165, LD.A p. 3-148.
        0x14 => op(LdBu, Base, slr.c, slr.b, 0),
        0x04 => op(LdBu, Post, slr.c, slr.b, 1),
        0x94 => op(LdH, Base, slr.c, slr.b, 0),
        0x84 => op(LdH, Post, slr.c, slr.b, 2),
        0x54 => op(LdW, Base, slr.c, slr.b, 0),
        0x44 => op(LdW, Post, slr.c, slr.b, 4),
        0xD4 => op(LdA, Base, slr.c, slr.b, 0),
        0xC4 => op(LdA, Post, slr.c, slr.b, 4),
        // SLRO: X[c] = M(A[15] + zero_ext(size * off4)). LD.BU p. 3-153, LD.H p. 3-159, LD.W p. 3-166,
        // LD.A p. 3-149.
        0x08 => op(LdBu, Base, slro.c, 15, slro.off4),
        0x88 => op(LdH, Base, slro.c, 15, 2 * slro.off4),
        0x48 => op(LdW, Base, slro.c, 15, 4 * slro.off4),
        0xC8 => op(LdA, Base, slro.c, 15, 4 * slro.off4),
        // SRO: D15/A15 <-> M(A[b] + zero_ext(size * off4)). LD.BU p. 3-153, LD.H p. 3-160, LD.W p. 3-166,
        // LD.A p. 3-149, ST.B p. 3-347, ST.H p. 3-354, ST.W p. 3-360, ST.A p. 3-344.
        0x0C => op(LdBu, Base, 15, sro.b, sro.off4),
        0x8C => op(LdH, Base, 15, sro.b, 2 * sro.off4),
        0x4C => op(LdW, Base, 15, sro.b, 4 * sro.off4),
        0xCC => op(LdA, Base, 15, sro.b, 4 * sro.off4),
        0x2C => op(StB, Base, 15, sro.b, sro.off4),
        0xAC => op(StH, Base, 15, sro.b, 2 * sro.off4),
        0x6C => op(StW, Base, 15, sro.b, 4 * sro.off4),
        0xEC => op(StA, Base, 15, sro.b, 4 * sro.off4),
        // SSR: M(A[b]) = X[a]; post-increment form A[b] += size. ST.B p. 3-347, ST.H p. 3-354,
        // ST.W p. 3-360/3-361, ST.A p. 3-344/3-345.
        0x34 => op(StB, Base, ssr.a, ssr.b, 0),
        0x24 => op(StB, Post, ssr.a, ssr.b, 1),
        0xB4 => op(StH, Base, ssr.a, ssr.b, 0),
        0xA4 => op(StH, Post, ssr.a, ssr.b, 2),
        0x74 => op(StW, Base, ssr.a, ssr.b, 0),
        0x64 => op(StW, Post, ssr.a, ssr.b, 4),
        0xF4 => op(StA, Base, ssr.a, ssr.b, 0),
        0xE4 => op(StA, Post, ssr.a, ssr.b, 4),
        // SSRO: M(A[15] + zero_ext(size * off4)) = X[a]. ST.B p. 3-348, ST.H p. 3-355, ST.W p. 3-361,
        // ST.A p. 3-345.
        0x28 => op(StB, Base, ssro.a, 15, ssro.off4),
        0xA8 => op(StH, Base, ssro.a, 15, 2 * ssro.off4),
        0x68 => op(StW, Base, ssro.a, 15, 4 * ssro.off4),
        0xE8 => op(StA, Base, ssro.a, 15, 4 * ssro.off4),
        _ => None,
    }
}

/// Only the always-trapping operand error ends a block; memory traps are handled by `Cpu::run`.
pub fn ends_block(op: &LdStOp) -> bool {
    op.kind == Kind::Opd
}

#[inline(always)]
pub fn exec<B: Bus>(cpu: &mut Cpu<B>, op: &LdStOp, pc: u32, len: u32) -> Flow {
    match step(cpu, op) {
        Ok(()) => Flow::Next,
        Err(t) => Flow::fault(t, pc, len),
    }
}

#[inline(always)]
fn step<B: Bus>(cpu: &mut Cpu<B>, op: &LdStOp) -> Result<(), Trap> {
    use Kind::*;
    if op.chk != 0 {
        mode_checks(cpu, op.chk)?;
    }
    let a = op.a as usize;
    match op.kind {
        // Loads: V2 LD.B/LD.BU p. 3-150, LD.H/LD.HU p. 3-158, LD.Q p. 3-162, LD.W p. 3-164,
        // LD.A p. 3-147, LD.D p. 3-154, LD.DA p. 3-156. Extension per page: LD.B/LD.H sign_ext,
        // LD.BU/LD.HU zero_ext, LD.Q {M(EA, halfword), 16'h0000}.
        LdB => {
            let v = load(cpu, op, 1, 1, 1)?;
            cpu.d[a] = v as u8 as i8 as u32;
        }
        LdBu => {
            let v = load(cpu, op, 1, 1, 1)?;
            cpu.d[a] = v as u8 as u32;
        }
        LdH => {
            let v = load(cpu, op, 2, 2, 2)?;
            cpu.d[a] = v as u16 as i16 as u32;
        }
        LdHu => {
            let v = load(cpu, op, 2, 2, 2)?;
            cpu.d[a] = v as u16 as u32;
        }
        LdQ => {
            let v = load(cpu, op, 2, 2, 2)?;
            cpu.d[a] = (v as u32) << 16;
        }
        LdW => {
            let v = load(cpu, op, 4, 2, 2)?;
            cpu.d[a] = v as u32;
        }
        LdA => {
            let v = load(cpu, op, 4, 4, 4)?;
            cpu.a[a] = v as u32;
        }
        LdD => {
            let v = load(cpu, op, 8, 2, 2)?;
            (cpu.d[a], cpu.d[a + 1]) = (v as u32, (v >> 32) as u32);
        }
        LdDa => {
            let v = load(cpu, op, 8, 4, 4)?;
            (cpu.a[a], cpu.a[a + 1]) = (v as u32, (v >> 32) as u32);
        }
        // Stores: ST.B p. 3-346 (D[a][7:0]), ST.H p. 3-353 (D[a][15:0]), ST.Q p. 3-356 (D[a][31:16]),
        // ST.W p. 3-359, ST.A p. 3-343, ST.D p. 3-349 (E[a]), ST.DA p. 3-351 (P[a]).
        StB => store(cpu, op, 1, 1, 1, cpu.d[a] as u64)?,
        StH => store(cpu, op, 2, 2, 2, cpu.d[a] as u64)?,
        StQ => store(cpu, op, 2, 2, 2, (cpu.d[a] >> 16) as u64)?,
        StW => store(cpu, op, 4, 2, 2, cpu.d[a] as u64)?,
        StA => store(cpu, op, 4, 4, 4, cpu.a[a] as u64)?,
        StD => store(cpu, op, 8, 2, 2, cpu.d[a] as u64 | (cpu.d[a + 1] as u64) << 32)?,
        StDa => store(cpu, op, 8, 4, 4, cpu.a[a] as u64 | (cpu.a[a + 1] as u64) << 32)?,
        // LEA p. 3-171: A[a] = EA[31:0]; no memory access, so no MEM/ALN/protection checks.
        Lea => {
            cpu.a[a] = match op.mode {
                Mode::Abs => op.off as u32,
                _ => cpu.a[op.b as usize].wrapping_add(op.off as u32),
            };
        }
        // LDMST p. 3-168: M(EA, word) = (M(EA, word) & ~E[a][63:32]) | (E[a][31:0] & E[a][63:32]);
        // word aligned (V1 Table 2-1).
        Ldmst => {
            let x = addr(cpu, op, 4, 4, 4, true)?;
            let (value, mask) = (cpu.d[a], cpu.d[a + 1]);
            let m = cpu.ld32(x.ea)?;
            let r = cpu.st32(x.ea, (m & !mask) | (value & mask));
            x.commit(cpu);
            r?;
        }
        // SWAP.W p. 3-376: tmp = M(EA, word); M(EA, word) = D[a]; D[a] = tmp.
        SwapW => {
            let x = addr(cpu, op, 4, 4, 4, true)?;
            let m = cpu.ld32(x.ea)?;
            let r = cpu.st32(x.ea, cpu.d[a]);
            x.commit(cpu);
            cpu.d[a] = m;
            r?;
        }
        StT => {
            let x = addr(cpu, op, 1, 1, 1, true)?;
            let (m, bit) = (cpu.ld8(x.ea)?, 1u8 << op.b);
            cpu.st8(x.ea, if op.a != 0 { m | bit } else { m & !bit })?;
        }
        // STLCX p. 3-362: M(EA, 16-word) = {PCXI, A[11], A[2:3], D[0:3], A[4:7], D[4:7]};
        // STUCX p. 3-363: M(EA, 16-word) = {PCXI, PSW, A[10:11], D[8:11], A[12:15], D[12:15]};
        // EA 16-word aligned (V1 Table 2-1). Word 0 at the lowest address (`crate::context` order).
        Stlcx | Stucx => {
            let x = addr(cpu, op, 64, 64, 64, true)?;
            for (i, &s) in (if op.kind == Stlcx { &LOWER } else { &UPPER }).iter().enumerate() {
                let v = match s {
                    Slot::Pcxi => cpu.pcxi,
                    Slot::Psw => cpu.psw,
                    Slot::D(n) => cpu.d[n as usize],
                    Slot::A(n) => cpu.a[n as usize],
                };
                cpu.st32(x.ea + 4 * i as u32, v)?;
            }
        }
        // LDLCX p. 3-167: {dummy, dummy, A[2:3], D[0:3], A[4:7], D[4:7]} = M(EA, 16-word);
        // LDUCX p. 3-170: {dummy, dummy, A[10:11], D[8:11], A[12:15], D[12:15]} = M(EA, 16-word).
        // PCXI, PSW, FCX (and A[11] for LDLCX) are unaffected (V131-2 §1.11.2).
        Ldlcx | Lducx => {
            let x = addr(cpu, op, 64, 64, 64, true)?;
            let mut w = [0u32; 16];
            for (i, v) in w.iter_mut().enumerate() {
                *v = cpu.ld32(x.ea + 4 * i as u32)?;
            }
            for (&s, &v) in (if op.kind == Ldlcx { &LOWER } else { &UPPER }).iter().zip(&w).skip(2) {
                match s {
                    Slot::D(n) => cpu.d[n as usize] = v,
                    Slot::A(n) => cpu.a[n as usize] = v,
                    Slot::Pcxi | Slot::Psw => {}
                }
            }
        }
        // CACHEA.* p. 3-47..3-52, CACHEI.* p. 3-53..3-58: "Any address register updates associated with
        // the addressing mode are always performed regardless of the cache operation."
        // ponytail: no data-cache model, so writeback/invalidate are no-ops (memory always coherent);
        // add a cache model if firmware ever depends on CACHEA.I discarding dirty lines.
        CacheAW | CacheAWI | CacheAI | CacheIW | CacheII | CacheIWI => {
            addr(cpu, op, 1, 1, 1, false)?.commit(cpu);
        }
        Opd => return Err(Trap::OPD),
    }
    Ok(())
}

/// PRIV (CACHEA.I p. 3-47, CACHEI.I p. 3-55: "can only be executed in Supervisor mode") before GRWP
/// (V1 Table 6-2 priorities 11, 12; PSW.GW V1 p. 3-7).
#[inline(never)]
fn mode_checks<B: Bus>(cpu: &Cpu<B>, chk: u8) -> Result<(), Trap> {
    if chk & SUPERVISOR != 0 && cpu.psw & psw::IO != psw::IO_SUPERVISOR {
        return Err(Trap::PRIV);
    }
    if chk & GLOBAL != 0 && cpu.psw & psw::GW == 0 {
        return Err(Trap::GRWP);
    }
    Ok(())
}

/// Effective address of an access and the address-register update of its addressing mode.
struct Ea {
    ea: u32,
    wb: Option<(usize, u32)>,
    /// Circular buffer (base, index, length) for accesses that may wrap.
    circ: Option<(u32, u32, u32)>,
}

impl Ea {
    #[inline(always)]
    fn commit<B: Bus>(&self, cpu: &mut Cpu<B>) {
        if let Some((r, v)) = self.wb {
            cpu.a[r] = v;
        }
    }
}

/// Address computation and the MEM/ALN/MPN/MPP checks of an access of `size` bytes, alignment `align`;
/// circular accesses are performed in `part`-byte pieces. `access = false` (cache operations): no
/// memory is accessed, only the addressing-mode rules apply.
#[inline(always)]
fn addr<B: Bus>(cpu: &Cpu<B>, op: &LdStOp, size: u32, align: u32, part: u32, access: bool) -> Result<Ea, Trap> {
    let b = op.b as usize;
    let off = op.off as u32;
    let (ea, base, mut wb, circ) = match op.mode {
        Mode::Abs => (off, None, None, None),
        Mode::Base => {
            let base = cpu.a[b];
            (base.wrapping_add(off), Some(base), None, None)
        }
        Mode::Post => {
            let ea = cpu.a[b];
            (ea, None, Some((b, ea.wrapping_add(off))), None)
        }
        Mode::Pre => {
            let ea = cpu.a[b].wrapping_add(off);
            (ea, Some(cpu.a[b]), Some((b, ea)), None)
        }
        Mode::Rev => {
            let (base, p) = (cpu.a[b], cpu.a[b + 1]);
            let (idx, incr) = (p & 0xFFFF, p >> 16);
            let new = reverse16(reverse16(idx) + reverse16(incr));
            (base.wrapping_add(idx), Some(base), Some((b + 1, incr << 16 | new)), None)
        }
        Mode::Circ => {
            let (base, p) = (cpu.a[b], cpu.a[b + 1]);
            let (idx, len) = (p & 0xFFFF, p >> 16);
            (base.wrapping_add(idx), Some(base), None, Some((base, idx, len)))
        }
    };
    if access {
        let n = if circ.is_some() { part } else { size };
        if base.is_some_and(|base| (base ^ ea) >> 28 != 0) || (ea ^ ea.wrapping_add(n - 1)) >> 28 != 0 {
            return Err(Trap::MEM);
        }
    }
    if ea & (align - 1) != 0 || (size == 8 && align == 2 && ea & 3 != 0 && dword_needs_word(cpu, ea)) {
        return Err(Trap::ALN);
    }
    if let Some((base, idx, len)) = circ {
        if base & 7 != 0 || len % size != 0 || idx >= len {
            return Err(Trap::ALN);
        }
        let new = idx as i32 + op.off;
        let new = if new < 0 { new + len as i32 } else { new % len as i32 };
        wb = Some((b + 1, len << 16 | (new as u32 & 0xFFFF)));
    }
    if access {
        if ea == 0 {
            return Err(Trap::MPN);
        }
        if cpu.psw & psw::IO == 0 && peripheral(cpu, ea) {
            return Err(Trap::MPP);
        }
    }
    Ok(Ea { ea, wb, circ })
}

/// PMA0 (CSFR 801CH, V1 p. 8-6): ATT[1][n] = bit 16 + n, ATT[0][n] = bit n; 10B = peripheral space.
pub(crate) const PMA0: usize = {
    let mut i = 0;
    while CSFR_TABLE[i].offset != 0x801C {
        i += 1;
    }
    i
};

/// Segment with the peripheral-space attribute. TC1.6: segment F always, others per PMA0 (the board
/// sets its reset value C000_03FFH = segments E, F); TC1.3.1: E and F (fixed, V131-1 Table 11 p. 8-3).
#[inline(always)]
fn peripheral<B: Bus>(cpu: &Cpu<B>, ea: u32) -> bool {
    let seg = ea >> 28;
    let res = (cpu.periph_mask >> seg) & 1 != 0;
    debug_assert_eq!(
        res,
        match cpu.arch {
            Arch::Tc13 | Arch::Tc131 => seg >= 0xE,
            Arch::Tc16 => {
                let pma = cpu.csfrs[PMA0];
                seg == 0xF || ((pma >> (16 + seg)) & 1 == 1 && (pma >> seg) & 1 == 0)
            }
        }
    );
    res
}

/// Double-word accesses that must be word aligned (V1 p. 2-4: peripheral space; V131-1 p. 2-4:
/// everything except cacheable memory and the DSPR, with the default TC1.3.1 attributes: segments
/// 0-9 and C cacheable, DSPR at D0000000H-D3FFFFFFH, V131-1 Table 11/13 p. 8-3/8-4).
fn dword_needs_word<B: Bus>(cpu: &Cpu<B>, ea: u32) -> bool {
    match cpu.arch {
        Arch::Tc16 => peripheral(cpu, ea),
        // ponytail: TC1784 attributes assumed equal to the architecture defaults; take them from the
        // board if its data sheet differs.
        Arch::Tc13 | Arch::Tc131 => !matches!(ea >> 28, 0..=9 | 0xC) && !(0xD000_0000..0xD400_0000).contains(&ea),
    }
}

/// Addresses of the pieces of a circular access that may wrap: `A[b] + (index + k * part) % length`
/// (LD.W p. 3-164, LD.D p. 3-154, LD.DA p. 3-156, ST.W p. 3-359, ST.D p. 3-349, ST.DA p. 3-351).
fn circ_parts((base, idx, len): (u32, u32, u32), size: u32, part: u32) -> Result<[u32; 4], Trap> {
    let mut eas = [0; 4];
    for k in 0..size / part {
        let ea = base.wrapping_add((idx + k * part) % len);
        if (ea ^ base) >> 28 != 0 || (ea ^ ea.wrapping_add(part - 1)) >> 28 != 0 {
            return Err(Trap::MEM);
        }
        eas[k as usize] = ea;
    }
    Ok(eas)
}

/// The common case: an Abs/Base/Post/Pre access that raises none of MEM/ALN/MPN/MPP. Returns the EA
/// and the address-register update; `None` sends the access through the full checks of [`addr`].
#[inline(always)]
pub(crate) fn fast<B: Bus>(cpu: &Cpu<B>, op: &LdStOp, size: u32, align: u32) -> Option<(u32, Option<(usize, u32)>)> {
    // register fields are 4 bits wide (the mask only drops the bounds check)
    let b = op.b as usize & 15;
    let off = op.off as u32;
    let (ea, base, wb) = match op.mode {
        Mode::Abs => (off, off, None),
        Mode::Base => (cpu.a[b].wrapping_add(off), cpu.a[b], None),
        Mode::Post => (cpu.a[b], cpu.a[b], Some((b, cpu.a[b].wrapping_add(off)))),
        Mode::Pre => (cpu.a[b].wrapping_add(off), cpu.a[b], Some((b, cpu.a[b].wrapping_add(off)))),
        Mode::Rev | Mode::Circ => return None,
    };
    // halfword-aligned double words take the slow path (`dword_needs_word`)
    let mask = if size == 8 { 3 } else { align - 1 };
    let ok = ((base ^ ea) | (ea ^ ea.wrapping_add(size - 1))) >> 28 == 0
        && ea & mask == 0
        && ea != 0
        && (cpu.psw & psw::IO != 0 || !peripheral(cpu, ea));
    ok.then_some((ea, wb))
}

/// Fast path of the plain loads/stores (`step` restricted to Abs/Base/Post/Pre accesses of host
/// memory that raise no trap, need no mode check, reach no peripheral and overwrite no translated
/// code): same effects in the same order as `step`. `None` = not applicable, nothing changed — the
/// caller then runs [`exec`]. Contains no out-of-line call, so a threaded-code handler that inlines
/// it keeps its own final call a tail call.
#[inline(always)]
pub(crate) fn exec_fast<B: Bus>(cpu: &mut Cpu<B>, op: &LdStOp) -> Option<()> {
    use Kind::*;
    if op.chk != 0 {
        return None;
    }
    let a = op.a as usize & 15;
    macro_rules! ld {
        ($n:literal, $align:literal, $t:ty) => {{
            let (ea, wb) = fast(cpu, op, $n, $align)?;
            let p = cpu.mem.host_r(ea, $n)?;
            // SAFETY: host_r validated the range.
            let v = <$t>::from_le_bytes(unsafe { *(p as *const [u8; $n]) });
            #[cfg(feature = "trace-data-stores")]
            cpu.bus.data_read(cpu.exec_pc, ea, $n, v as u64);
            if let Some((r, x)) = wb {
                cpu.a[r & 15] = x;
            }
            v
        }};
    }
    macro_rules! st {
        ($n:literal, $align:literal, $v:expr) => {{
            let v = $v;
            let (ea, wb) = fast(cpu, op, $n, $align)?;
            #[cfg(feature = "trace-data-stores")]
            {
                cpu.bus.data_write(cpu.exec_pc, ea, $n, v as u64);
            }
            let p = cpu.mem.host_w_plain(ea, $n)?;
            // SAFETY: host_w_plain validated the range.
            unsafe { *(p as *mut [u8; $n]) = v.to_le_bytes() };
            if let Some((r, x)) = wb {
                cpu.a[r & 15] = x;
            }
        }};
    }
    match op.kind {
        LdB => cpu.d[a] = ld!(1, 1, u8) as i8 as u32,
        LdBu => cpu.d[a] = ld!(1, 1, u8) as u32,
        LdH => cpu.d[a] = ld!(2, 2, u16) as i16 as u32,
        LdHu => cpu.d[a] = ld!(2, 2, u16) as u32,
        LdW => cpu.d[a] = ld!(4, 2, u32),
        LdA => cpu.a[a] = ld!(4, 4, u32),
        LdD => {
            let v = ld!(8, 2, u64);
            (cpu.d[a], cpu.d[(a + 1) & 15]) = (v as u32, (v >> 32) as u32);
        }
        StB => st!(1, 1, cpu.d[a] as u8),
        StH => st!(2, 2, cpu.d[a] as u16),
        StW => st!(4, 2, cpu.d[a]),
        StA => st!(4, 4, cpu.a[a]),
        StD => st!(8, 2, cpu.d[a] as u64 | (cpu.d[(a + 1) & 15] as u64) << 32),
        Lea => {
            cpu.a[a] = match op.mode {
                Mode::Abs => op.off as u32,
                _ => cpu.a[op.b as usize & 15].wrapping_add(op.off as u32),
            }
        }
        _ => return None,
    }
    Some(())
}

/// Load `size` bytes (zero-extended). Registers change only if every access succeeded.
#[inline(always)]
fn load<B: Bus>(cpu: &mut Cpu<B>, op: &LdStOp, size: u32, align: u32, part: u32) -> Result<u64, Trap> {
    if let Some((ea, wb)) = fast(cpu, op, size, align) {
        let v = match size {
            1 => cpu.ld8(ea)? as u64,
            2 => cpu.ld16(ea)? as u64,
            4 => cpu.ld32(ea)? as u64,
            _ => cpu.ld64(ea)?,
        };
        if let Some((r, x)) = wb {
            cpu.a[r] = x;
        }
        return Ok(v);
    }
    load_checked(cpu, op, size, align, part)
}

#[inline(never)]
fn load_checked<B: Bus>(cpu: &mut Cpu<B>, op: &LdStOp, size: u32, align: u32, part: u32) -> Result<u64, Trap> {
    let x = addr(cpu, op, size, align, part, true)?;
    let v = match x.circ {
        Some(c) if part < size => {
            let eas = circ_parts(c, size, part)?;
            let mut v = 0u64;
            for k in 0..size / part {
                let ea = eas[k as usize];
                let p = if part == 2 { cpu.ld16(ea)? as u64 } else { cpu.ld32(ea)? as u64 };
                v |= p << (8 * part * k);
            }
            v
        }
        _ => match size {
            1 => cpu.ld8(x.ea)? as u64,
            2 => cpu.ld16(x.ea)? as u64,
            4 => cpu.ld32(x.ea)? as u64,
            _ => cpu.ld64(x.ea)?,
        },
    };
    x.commit(cpu);
    Ok(v)
}

/// Store the low `size` bytes of `v`. A bus error (DAE) is asynchronous: the instruction completes,
/// so the address-register update is kept.
#[inline(always)]
fn store<B: Bus>(cpu: &mut Cpu<B>, op: &LdStOp, size: u32, align: u32, part: u32, v: u64) -> Result<(), Trap> {
    if let Some((ea, wb)) = fast(cpu, op, size, align) {
        let r = match size {
            1 => cpu.st8(ea, v as u8),
            2 => cpu.st16(ea, v as u16),
            4 => cpu.st32(ea, v as u32),
            _ => cpu.st64(ea, v),
        };
        if let Some((r, x)) = wb {
            cpu.a[r] = x;
        }
        return r;
    }
    store_checked(cpu, op, size, align, part, v)
}

#[inline(never)]
fn store_checked<B: Bus>(cpu: &mut Cpu<B>, op: &LdStOp, size: u32, align: u32, part: u32, v: u64) -> Result<(), Trap> {
    let x = addr(cpu, op, size, align, part, true)?;
    let r = match x.circ {
        Some(c) if part < size => {
            let eas = circ_parts(c, size, part)?;
            let mut r = Ok(());
            for k in 0..size / part {
                let (ea, p) = (eas[k as usize], v >> (8 * part * k));
                r = if part == 2 { cpu.st16(ea, p as u16) } else { cpu.st32(ea, p as u32) };
                if r.is_err() {
                    break;
                }
            }
            r
        }
        _ => match size {
            1 => cpu.st8(x.ea, v as u8),
            2 => cpu.st16(x.ea, v as u16),
            4 => cpu.st32(x.ea, v as u32),
            _ => cpu.st64(x.ea, v),
        },
    };
    x.commit(cpu);
    r
}

#[cfg(test)]
mod tests {
    use super::Kind::*;
    use super::*;
    use crate::testutil::{cpu_with, link, oracle, run, run_arch, Setup, CODE, CODE_SIZE, CSA_BASE, DATA, DATA_SIZE, IRQ_BASE, TRAP_BASE};
    use crate::{Memory, NullBus};

    type M = Mode;
    /// (mnemonic, format, mode, op1, op2 or FFH, V2 page).
    type Row = (&'static str, &'static str, Mode, u8, u8, &'static str);

    /// Every group-C encoding of V2, generated from `docs/tricore_core/isa_tc16_encodings.json` (the
    /// machine-extracted encoding boxes of `tcarch_vol2_a4.pdf`).
    const MANUAL: &[Row] = &[
    ("CACHEA.I", "BO", M::Base, 0x89, 0x2E, "3-47"),
    ("CACHEA.I", "BO", M::Rev, 0xA9, 0x0E, "3-47"),
    ("CACHEA.I", "BO", M::Circ, 0xA9, 0x1E, "3-47"),
    ("CACHEA.I", "BO", M::Post, 0x89, 0x0E, "3-47"),
    ("CACHEA.I", "BO", M::Pre, 0x89, 0x1E, "3-48"),
    ("CACHEA.W", "BO", M::Base, 0x89, 0x2C, "3-49"),
    ("CACHEA.W", "BO", M::Rev, 0xA9, 0x0C, "3-49"),
    ("CACHEA.W", "BO", M::Circ, 0xA9, 0x1C, "3-49"),
    ("CACHEA.W", "BO", M::Post, 0x89, 0x0C, "3-49"),
    ("CACHEA.W", "BO", M::Pre, 0x89, 0x1C, "3-50"),
    ("CACHEA.WI", "BO", M::Base, 0x89, 0x2D, "3-51"),
    ("CACHEA.WI", "BO", M::Rev, 0xA9, 0x0D, "3-51"),
    ("CACHEA.WI", "BO", M::Circ, 0xA9, 0x1D, "3-51"),
    ("CACHEA.WI", "BO", M::Post, 0x89, 0x0D, "3-51"),
    ("CACHEA.WI", "BO", M::Pre, 0x89, 0x1D, "3-52"),
    ("CACHEI.W", "BO", M::Base, 0x89, 0x2B, "3-53"),
    ("CACHEI.W", "BO", M::Post, 0x89, 0x0B, "3-53"),
    ("CACHEI.W", "BO", M::Pre, 0x89, 0x1B, "3-53"),
    ("CACHEI.I", "BO", M::Base, 0x89, 0x2A, "3-55"),
    ("CACHEI.I", "BO", M::Post, 0x89, 0x0A, "3-55"),
    ("CACHEI.I", "BO", M::Pre, 0x89, 0x1A, "3-55"),
    ("CACHEI.WI", "BO", M::Base, 0x89, 0x2F, "3-57"),
    ("CACHEI.WI", "BO", M::Post, 0x89, 0x0F, "3-57"),
    ("CACHEI.WI", "BO", M::Pre, 0x89, 0x1F, "3-57"),
    ("LD.A", "ABS", M::Abs, 0x85, 0x02, "3-147"),
    ("LD.A", "BO", M::Base, 0x09, 0x26, "3-147"),
    ("LD.A", "BO", M::Rev, 0x29, 0x06, "3-147"),
    ("LD.A", "BO", M::Circ, 0x29, 0x16, "3-147"),
    ("LD.A", "BO", M::Post, 0x09, 0x06, "3-148"),
    ("LD.A", "BO", M::Pre, 0x09, 0x16, "3-148"),
    ("LD.A", "BOL", M::Base, 0x99, 0xFF, "3-148"),
    ("LD.A", "SC", M::Base, 0xD8, 0xFF, "3-148"),
    ("LD.A", "SLR", M::Base, 0xD4, 0xFF, "3-148"),
    ("LD.A", "SLR", M::Post, 0xC4, 0xFF, "3-148"),
    ("LD.A", "SLRO", M::Base, 0xC8, 0xFF, "3-149"),
    ("LD.A", "SRO", M::Base, 0xCC, 0xFF, "3-149"),
    ("LD.B", "ABS", M::Abs, 0x05, 0x00, "3-150"),
    ("LD.B", "BO", M::Base, 0x09, 0x20, "3-150"),
    ("LD.B", "BO", M::Rev, 0x29, 0x00, "3-150"),
    ("LD.B", "BO", M::Circ, 0x29, 0x10, "3-150"),
    ("LD.B", "BO", M::Post, 0x09, 0x00, "3-151"),
    ("LD.B", "BO", M::Pre, 0x09, 0x10, "3-151"),
    ("LD.B", "BOL", M::Base, 0x79, 0xFF, "3-151"),
    ("LD.BU", "ABS", M::Abs, 0x05, 0x01, "3-151"),
    ("LD.BU", "BO", M::Base, 0x09, 0x21, "3-151"),
    ("LD.BU", "BO", M::Rev, 0x29, 0x01, "3-151"),
    ("LD.BU", "BO", M::Circ, 0x29, 0x11, "3-152"),
    ("LD.BU", "BO", M::Post, 0x09, 0x01, "3-152"),
    ("LD.BU", "BO", M::Pre, 0x09, 0x11, "3-152"),
    ("LD.BU", "BOL", M::Base, 0x39, 0xFF, "3-152"),
    ("LD.BU", "SLR", M::Base, 0x14, 0xFF, "3-152"),
    ("LD.BU", "SLR", M::Post, 0x04, 0xFF, "3-153"),
    ("LD.BU", "SLRO", M::Base, 0x08, 0xFF, "3-153"),
    ("LD.BU", "SRO", M::Base, 0x0C, 0xFF, "3-153"),
    ("LD.D", "ABS", M::Abs, 0x85, 0x01, "3-154"),
    ("LD.D", "BO", M::Base, 0x09, 0x25, "3-154"),
    ("LD.D", "BO", M::Rev, 0x29, 0x05, "3-154"),
    ("LD.D", "BO", M::Circ, 0x29, 0x15, "3-154"),
    ("LD.D", "BO", M::Post, 0x09, 0x05, "3-155"),
    ("LD.D", "BO", M::Pre, 0x09, 0x15, "3-155"),
    ("LD.DA", "ABS", M::Abs, 0x85, 0x03, "3-156"),
    ("LD.DA", "BO", M::Base, 0x09, 0x27, "3-156"),
    ("LD.DA", "BO", M::Rev, 0x29, 0x07, "3-156"),
    ("LD.DA", "BO", M::Circ, 0x29, 0x17, "3-156"),
    ("LD.DA", "BO", M::Post, 0x09, 0x07, "3-157"),
    ("LD.DA", "BO", M::Pre, 0x09, 0x17, "3-157"),
    ("LD.H", "ABS", M::Abs, 0x05, 0x02, "3-158"),
    ("LD.H", "BO", M::Base, 0x09, 0x22, "3-158"),
    ("LD.H", "BO", M::Rev, 0x29, 0x02, "3-158"),
    ("LD.H", "BO", M::Circ, 0x29, 0x12, "3-158"),
    ("LD.H", "BO", M::Post, 0x09, 0x02, "3-159"),
    ("LD.H", "BO", M::Pre, 0x09, 0x12, "3-159"),
    ("LD.H", "BOL", M::Base, 0xC9, 0xFF, "3-159"),
    ("LD.H", "SLR", M::Base, 0x94, 0xFF, "3-159"),
    ("LD.H", "SLR", M::Post, 0x84, 0xFF, "3-159"),
    ("LD.H", "SLRO", M::Base, 0x88, 0xFF, "3-159"),
    ("LD.H", "SRO", M::Base, 0x8C, 0xFF, "3-160"),
    ("LD.HU", "ABS", M::Abs, 0x05, 0x03, "3-160"),
    ("LD.HU", "BO", M::Base, 0x09, 0x23, "3-160"),
    ("LD.HU", "BO", M::Rev, 0x29, 0x03, "3-160"),
    ("LD.HU", "BO", M::Circ, 0x29, 0x13, "3-160"),
    ("LD.HU", "BO", M::Post, 0x09, 0x03, "3-161"),
    ("LD.HU", "BO", M::Pre, 0x09, 0x13, "3-161"),
    ("LD.HU", "BOL", M::Base, 0xB9, 0xFF, "3-161"),
    ("LD.Q", "ABS", M::Abs, 0x45, 0x00, "3-162"),
    ("LD.Q", "BO", M::Base, 0x09, 0x28, "3-162"),
    ("LD.Q", "BO", M::Rev, 0x29, 0x08, "3-162"),
    ("LD.Q", "BO", M::Circ, 0x29, 0x18, "3-162"),
    ("LD.Q", "BO", M::Post, 0x09, 0x08, "3-162"),
    ("LD.Q", "BO", M::Pre, 0x09, 0x18, "3-163"),
    ("LD.W", "ABS", M::Abs, 0x85, 0x00, "3-164"),
    ("LD.W", "BO", M::Base, 0x09, 0x24, "3-164"),
    ("LD.W", "BO", M::Rev, 0x29, 0x04, "3-164"),
    ("LD.W", "BO", M::Circ, 0x29, 0x14, "3-164"),
    ("LD.W", "BO", M::Post, 0x09, 0x04, "3-165"),
    ("LD.W", "BO", M::Pre, 0x09, 0x14, "3-165"),
    ("LD.W", "BOL", M::Base, 0x19, 0xFF, "3-165"),
    ("LD.W", "SC", M::Base, 0x58, 0xFF, "3-165"),
    ("LD.W", "SLR", M::Base, 0x54, 0xFF, "3-165"),
    ("LD.W", "SLR", M::Post, 0x44, 0xFF, "3-165"),
    ("LD.W", "SLRO", M::Base, 0x48, 0xFF, "3-166"),
    ("LD.W", "SRO", M::Base, 0x4C, 0xFF, "3-166"),
    ("LDLCX", "ABS", M::Abs, 0x15, 0x02, "3-167"),
    ("LDLCX", "BO", M::Base, 0x49, 0x24, "3-167"),
    ("LDMST", "ABS", M::Abs, 0xE5, 0x01, "3-168"),
    ("LDMST", "BO", M::Base, 0x49, 0x21, "3-168"),
    ("LDMST", "BO", M::Rev, 0x69, 0x01, "3-168"),
    ("LDMST", "BO", M::Circ, 0x69, 0x11, "3-168"),
    ("LDMST", "BO", M::Post, 0x49, 0x01, "3-169"),
    ("LDMST", "BO", M::Pre, 0x49, 0x11, "3-169"),
    ("LDUCX", "ABS", M::Abs, 0x15, 0x03, "3-170"),
    ("LDUCX", "BO", M::Base, 0x49, 0x25, "3-170"),
    ("LEA", "ABS", M::Abs, 0xC5, 0x00, "3-171"),
    ("LEA", "BO", M::Base, 0x49, 0x28, "3-171"),
    ("LEA", "BOL", M::Base, 0xD9, 0xFF, "3-171"),
    ("ST.A", "ABS", M::Abs, 0xA5, 0x02, "3-343"),
    ("ST.A", "BO", M::Base, 0x89, 0x26, "3-343"),
    ("ST.A", "BO", M::Rev, 0xA9, 0x06, "3-343"),
    ("ST.A", "BO", M::Circ, 0xA9, 0x16, "3-343"),
    ("ST.A", "BO", M::Post, 0x89, 0x06, "3-344"),
    ("ST.A", "BO", M::Pre, 0x89, 0x16, "3-344"),
    ("ST.A", "BOL", M::Base, 0xB5, 0xFF, "3-344"),
    ("ST.A", "SC", M::Base, 0xF8, 0xFF, "3-344"),
    ("ST.A", "SRO", M::Base, 0xEC, 0xFF, "3-344"),
    ("ST.A", "SSR", M::Base, 0xF4, 0xFF, "3-344"),
    ("ST.A", "SSR", M::Post, 0xE4, 0xFF, "3-345"),
    ("ST.A", "SSRO", M::Base, 0xE8, 0xFF, "3-345"),
    ("ST.B", "ABS", M::Abs, 0x25, 0x00, "3-346"),
    ("ST.B", "BO", M::Base, 0x89, 0x20, "3-346"),
    ("ST.B", "BO", M::Rev, 0xA9, 0x00, "3-346"),
    ("ST.B", "BO", M::Circ, 0xA9, 0x10, "3-346"),
    ("ST.B", "BO", M::Post, 0x89, 0x00, "3-347"),
    ("ST.B", "BO", M::Pre, 0x89, 0x10, "3-347"),
    ("ST.B", "BOL", M::Base, 0xE9, 0xFF, "3-347"),
    ("ST.B", "SRO", M::Base, 0x2C, 0xFF, "3-347"),
    ("ST.B", "SSR", M::Base, 0x34, 0xFF, "3-347"),
    ("ST.B", "SSR", M::Post, 0x24, 0xFF, "3-347"),
    ("ST.B", "SSRO", M::Base, 0x28, 0xFF, "3-348"),
    ("ST.D", "ABS", M::Abs, 0xA5, 0x01, "3-349"),
    ("ST.D", "BO", M::Base, 0x89, 0x25, "3-349"),
    ("ST.D", "BO", M::Rev, 0xA9, 0x05, "3-349"),
    ("ST.D", "BO", M::Circ, 0xA9, 0x15, "3-349"),
    ("ST.D", "BO", M::Post, 0x89, 0x05, "3-350"),
    ("ST.D", "BO", M::Pre, 0x89, 0x15, "3-350"),
    ("ST.DA", "ABS", M::Abs, 0xA5, 0x03, "3-351"),
    ("ST.DA", "BO", M::Base, 0x89, 0x27, "3-351"),
    ("ST.DA", "BO", M::Rev, 0xA9, 0x07, "3-351"),
    ("ST.DA", "BO", M::Circ, 0xA9, 0x17, "3-351"),
    ("ST.DA", "BO", M::Post, 0x89, 0x07, "3-352"),
    ("ST.DA", "BO", M::Pre, 0x89, 0x17, "3-352"),
    ("ST.H", "ABS", M::Abs, 0x25, 0x02, "3-353"),
    ("ST.H", "BO", M::Base, 0x89, 0x22, "3-353"),
    ("ST.H", "BO", M::Rev, 0xA9, 0x02, "3-353"),
    ("ST.H", "BO", M::Circ, 0xA9, 0x12, "3-353"),
    ("ST.H", "BO", M::Post, 0x89, 0x02, "3-354"),
    ("ST.H", "BO", M::Pre, 0x89, 0x12, "3-354"),
    ("ST.H", "BOL", M::Base, 0xF9, 0xFF, "3-354"),
    ("ST.H", "SRO", M::Base, 0xAC, 0xFF, "3-354"),
    ("ST.H", "SSR", M::Base, 0xB4, 0xFF, "3-354"),
    ("ST.H", "SSR", M::Post, 0xA4, 0xFF, "3-354"),
    ("ST.H", "SSRO", M::Base, 0xA8, 0xFF, "3-355"),
    ("ST.Q", "ABS", M::Abs, 0x65, 0x00, "3-356"),
    ("ST.Q", "BO", M::Base, 0x89, 0x28, "3-356"),
    ("ST.Q", "BO", M::Rev, 0xA9, 0x08, "3-356"),
    ("ST.Q", "BO", M::Circ, 0xA9, 0x18, "3-356"),
    ("ST.Q", "BO", M::Post, 0x89, 0x08, "3-356"),
    ("ST.Q", "BO", M::Pre, 0x89, 0x18, "3-357"),
    ("ST.T", "ABSB", M::Abs, 0xD5, 0x00, "3-358"),
    ("ST.W", "ABS", M::Abs, 0xA5, 0x00, "3-359"),
    ("ST.W", "BO", M::Base, 0x89, 0x24, "3-359"),
    ("ST.W", "BO", M::Rev, 0xA9, 0x04, "3-359"),
    ("ST.W", "BO", M::Circ, 0xA9, 0x14, "3-359"),
    ("ST.W", "BO", M::Post, 0x89, 0x04, "3-360"),
    ("ST.W", "BO", M::Pre, 0x89, 0x14, "3-360"),
    ("ST.W", "BOL", M::Base, 0x59, 0xFF, "3-360"),
    ("ST.W", "SC", M::Base, 0x78, 0xFF, "3-360"),
    ("ST.W", "SRO", M::Base, 0x6C, 0xFF, "3-360"),
    ("ST.W", "SSR", M::Base, 0x74, 0xFF, "3-360"),
    ("ST.W", "SSR", M::Post, 0x64, 0xFF, "3-361"),
    ("ST.W", "SSRO", M::Base, 0x68, 0xFF, "3-361"),
    ("STLCX", "ABS", M::Abs, 0x15, 0x00, "3-362"),
    ("STLCX", "BO", M::Base, 0x49, 0x26, "3-362"),
    ("STUCX", "ABS", M::Abs, 0x15, 0x01, "3-363"),
    ("STUCX", "BO", M::Base, 0x49, 0x27, "3-363"),
    ("SWAP.W", "ABS", M::Abs, 0xE5, 0x00, "3-376"),
    ("SWAP.W", "BO", M::Base, 0x49, 0x20, "3-376"),
    ("SWAP.W", "BO", M::Rev, 0x69, 0x00, "3-376"),
    ("SWAP.W", "BO", M::Circ, 0x69, 0x10, "3-376"),
    ("SWAP.W", "BO", M::Post, 0x49, 0x00, "3-377"),
    ("SWAP.W", "BO", M::Pre, 0x49, 0x10, "3-377"),
    ];

    fn kind_of(m: &str) -> Kind {
        match m {
            "LD.B" => LdB,
            "LD.BU" => LdBu,
            "LD.H" => LdH,
            "LD.HU" => LdHu,
            "LD.W" => LdW,
            "LD.D" => LdD,
            "LD.A" => LdA,
            "LD.DA" => LdDa,
            "LD.Q" => LdQ,
            "ST.B" => StB,
            "ST.H" => StH,
            "ST.W" => StW,
            "ST.D" => StD,
            "ST.A" => StA,
            "ST.DA" => StDa,
            "ST.Q" => StQ,
            "LEA" => Lea,
            "SWAP.W" => SwapW,
            "LDMST" => Ldmst,
            "ST.T" => StT,
            "LDLCX" => Ldlcx,
            "LDUCX" => Lducx,
            "STLCX" => Stlcx,
            "STUCX" => Stucx,
            "CACHEA.W" => CacheAW,
            "CACHEA.WI" => CacheAWI,
            "CACHEA.I" => CacheAI,
            "CACHEI.W" => CacheIW,
            "CACHEI.I" => CacheII,
            "CACHEI.WI" => CacheIWI,
            _ => panic!("{m}"),
        }
    }

    /// (data size, alignment), V1 Table 2-1 p. 2-4, written out independently of the implementation.
    fn shape(k: Kind) -> (u32, u32) {
        match k {
            LdB | LdBu | StB | StT => (1, 1),
            LdH | LdHu | LdQ | StH | StQ => (2, 2),
            LdW | StW => (4, 2),
            LdA | StA | SwapW | Ldmst => (4, 4),
            LdD | StD => (8, 2),
            LdDa | StDa => (8, 4),
            Ldlcx | Lducx | Stlcx | Stucx => (64, 64),
            _ => (1, 1),
        }
    }

    fn pair_a(k: Kind) -> bool {
        matches!(k, LdD | StD | LdDa | StDa | Ldmst)
    }

    /// Address registers written or stored as data by the operation (A[a], A[a+1]).
    fn a_operands(op: &LdStOp) -> Vec<usize> {
        let a = op.a as usize;
        match op.kind {
            LdA | StA | Lea => vec![a],
            LdDa | StDa => vec![a, a + 1],
            _ => vec![],
        }
    }

    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 16) as u32
        }
        fn below(&mut self, n: u32) -> u32 {
            self.next() % n
        }
    }

    // ---- encoders (field layouts of V2 Table 1-5/1-6 p. 1-3/1-4) --------------------------------------

    fn bo(op1: u8, op2: u8, a: u32, b: u32, off10: i32) -> u32 {
        let o = off10 as u32 & 0x3FF;
        (o >> 6) << 28 | (op2 as u32) << 22 | (o & 0x3F) << 16 | b << 12 | a << 8 | op1 as u32
    }

    fn bol(op1: u8, a: u32, b: u32, off16: i32) -> u32 {
        let o = off16 as u32 & 0xFFFF;
        ((o >> 6) & 0xF) << 28 | (o >> 10) << 22 | (o & 0x3F) << 16 | b << 12 | a << 8 | op1 as u32
    }

    fn off18(ea: u32) -> u32 {
        assert_eq!(ea & 0x0FFF_C000, 0, "{ea:#x} is not absolute-addressable");
        let o = (ea >> 28) << 14 | (ea & 0x3FFF);
        ((o >> 6) & 0xF) << 28 | ((o >> 10) & 0xF) << 22 | (o & 0x3F) << 16 | (o >> 14) << 12
    }

    fn abs(op1: u8, op2: u8, a: u32, ea: u32) -> u32 {
        off18(ea) | (op2 as u32) << 26 | a << 8 | op1 as u32
    }

    fn st_t(ea: u32, bpos: u32, bit: u32) -> u32 {
        off18(ea) | bit << 11 | bpos << 8 | 0xD5
    }

    fn h16(op1: u8, hi: u32, mid: u32) -> u32 {
        hi << 12 | mid << 8 | op1 as u32
    }

    fn bytes(w: u32) -> Vec<u8> {
        if w & 1 == 1 {
            w.to_le_bytes().to_vec()
        } else {
            (w as u16).to_le_bytes().to_vec()
        }
    }

    /// Encode `row` with register fields `a`/`b`, raw offset field `off` (off10/off16/const8/off4) and
    /// absolute address `ea` (ABS/ABSB).
    fn encode(row: &Row, a: u32, b: u32, off: i32, ea: u32) -> u32 {
        let (_, f, _, op1, op2, _) = *row;
        match f {
            "ABS" => abs(op1, op2, a, ea),
            "ABSB" => st_t(ea, b & 7, a & 1),
            "BO" => bo(op1, op2, a, b, off),
            "BOL" => bol(op1, a, b, off),
            "SC" => (off as u32 & 0xFF) << 8 | op1 as u32,
            "SLR" | "SSR" => h16(op1, b, a),
            "SLRO" | "SSRO" => h16(op1, off as u32 & 0xF, a),
            "SRO" => h16(op1, b, off as u32 & 0xF),
            _ => unreachable!("{f}"),
        }
    }

    fn dec(w: u32, arch: Arch) -> Option<LdStOp> {
        decode(w as u16, (w >> 16) as u16, CODE, arch)
    }

    // ---- decoding -------------------------------------------------------------------------------------

    #[test]
    fn decodes_every_manual_encoding() {
        assert_eq!(MANUAL.len(), 190);
        let tc16_only_bol = [0x39, 0x79, 0xB9, 0xC9, 0xB5, 0xE9, 0xF9];
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
        for row in MANUAL {
            let (m, f, mode, op1, _, page) = *row;
            let k = kind_of(m);
            let absent_131 = m == "CACHEI.I" || (f == "BOL" && tc16_only_bol.contains(&op1));
            for _ in 0..256 {
                let mut a = rng.below(16);
                let mut b = rng.below(16);
                if pair_a(k) {
                    a &= !1;
                }
                if matches!(mode, M::Rev | M::Circ) {
                    b &= !1;
                }
                let w = encode(row, a, b, rng.next() as i32, DATA | rng.below(0x4000));
                let op = dec(w, Arch::Tc16).unwrap_or_else(|| panic!("{m} {f} p. {page}: {w:#010x} not decoded"));
                assert_eq!((op.kind, op.mode), (k, mode), "{m} {f} p. {page}: {w:#010x}");
                let o131 = dec(w, Arch::Tc131);
                assert_eq!(o131.is_none(), absent_131, "{m} {f} TC1.3.1: {w:#010x}");
                if !absent_131 {
                    assert_eq!(o131, Some(op));
                }
            }
        }
    }

    /// Within the op1 values this group uses, exactly the manual's op2 values decode; 16-bit: exactly the
    /// 36 manual op1 values.
    #[test]
    fn decodes_nothing_else() {
        let listed = |op1: u8, op2: u8| MANUAL.iter().any(|r| r.3 == op1 && r.4 == op2);
        for op1 in [0x09u8, 0x29, 0x49, 0x69, 0x89, 0xA9] {
            for op2 in 0..64u8 {
                for w in [bo(op1, op2, 2, 4, 0), bo(op1, op2, 15, 14, -1) ^ 0x0F00] {
                    assert_eq!(dec(w, Arch::Tc16).is_some(), listed(op1, op2), "BO {op1:#04x}/{op2:#04x}");
                }
            }
        }
        for op1 in [0x05u8, 0x15, 0x25, 0x45, 0x65, 0x85, 0xA5, 0xC5, 0xE5, 0xD5] {
            for op2 in 0..4u8 {
                let w = abs(op1, op2, 3, 0xD000_1234);
                assert_eq!(dec(w, Arch::Tc16).is_some(), listed(op1, op2), "ABS {op1:#04x}/{op2}");
            }
        }
        let short: Vec<u8> = MANUAL.iter().filter(|r| r.4 == 0xFF && r.3 & 1 == 0).map(|r| r.3).collect();
        assert_eq!(short.len(), 36);
        for op1 in (0..=0xFFu8).step_by(2) {
            for hi in [0x00u16, 0xA5, 0xFF] {
                let h = hi << 8 | op1 as u16;
                assert_eq!(decode(h, 0, 0, Arch::Tc16).is_some(), short.contains(&op1), "16-bit {h:#06x}");
            }
        }
    }

    /// Firmware encodings with rizin's (capstone) disassembly — an independent decoder — transcribed
    /// into (kind, mode, a, b, off). 16-bit: DQ200_G2 ASW, one per opcode.
    #[test]
    fn decodes_firmware_examples_like_rizin() {
        /// (bytes, rizin text, kind, mode, a, b, off)
        type Example = (&'static [u8], &'static str, Kind, Mode, u8, u8, i32);
        #[rustfmt::skip]
        let cases: &[Example] = &[
            (&[0x04, 0x5f], "ld.bu d15, [a5+]", LdBu, M::Post, 15, 5, 1),
            (&[0x24, 0xff], "st.b [a15+], d15", StB, M::Post, 15, 15, 1),
            (&[0xd4, 0x00], "ld.a a0, [a0]", LdA, M::Base, 0, 0, 0),
            (&[0xd8, 0x00], "ld.a a15, [sp]#0", LdA, M::Base, 15, 10, 0),
            (&[0x8c, 0x07], "ld.h d15, [a0]#0xe", LdH, M::Base, 15, 0, 14),
            (&[0x64, 0x08], "st.w [a0+], d8", StW, M::Post, 8, 0, 4),
            (&[0x68, 0x08], "st.w [a15]#0, d8", StW, M::Base, 8, 15, 0),
            (&[0x84, 0x08], "ld.h d8, [a0+]", LdH, M::Post, 8, 0, 2),
            (&[0xcc, 0x08], "ld.a a15, [a0]#0x20", LdA, M::Base, 15, 0, 0x20),
            (&[0xe4, 0x08], "st.a [a0+], a8", StA, M::Post, 8, 0, 4),
            (&[0xf4, 0x0b], "st.a [a0], a11", StA, M::Base, 11, 0, 0),
            (&[0x44, 0x00], "ld.w d0, [a0+]", LdW, M::Post, 0, 0, 4),
            (&[0xc8, 0xe4], "ld.a a4, [a15]#0x38", LdA, M::Base, 4, 15, 0x38),
            (&[0x78, 0x34], "st.w [sp]#0xd0, d15", StW, M::Base, 15, 10, 0xd0),
            (&[0x08, 0x00], "ld.bu d0, [a15]#0", LdBu, M::Base, 0, 15, 0),
            (&[0x74, 0xff], "st.w [a15], d15", StW, M::Base, 15, 15, 0),
            (&[0x6c, 0xf1], "st.w [a15]#4, d15", StW, M::Base, 15, 15, 4),
            (&[0x48, 0x1f], "ld.w d15, [a15]#4", LdW, M::Base, 15, 15, 4),
            (&[0x54, 0xf8], "ld.w d8, [a15]", LdW, M::Base, 8, 15, 0),
            (&[0x14, 0xff], "ld.bu d15, [a15]", LdBu, M::Base, 15, 15, 0),
            (&[0x0c, 0x28], "ld.bu d15, [a2]#8", LdBu, M::Base, 15, 2, 8),
            (&[0x2c, 0x48], "st.b [a4]#8, d15", StB, M::Base, 15, 4, 8),
            (&[0x34, 0xf1], "st.b [a15], d1", StB, M::Base, 1, 15, 0),
            (&[0x28, 0x29], "st.b [a15]#2, d9", StB, M::Base, 9, 15, 2),
            (&[0xc4, 0xed], "ld.a a13, [a14+]", LdA, M::Post, 13, 14, 4),
            (&[0xb4, 0xcf], "st.h [a12], d15", StH, M::Base, 15, 12, 0),
            (&[0x4c, 0xc1], "ld.w d15, [a12]#4", LdW, M::Base, 15, 12, 4),
            (&[0xac, 0x21], "st.h [a2]#2, d15", StH, M::Base, 15, 2, 2),
            (&[0x88, 0x11], "ld.h d1, [a15]#2", LdH, M::Base, 1, 15, 2),
            (&[0xf8, 0x02], "st.a [sp]#8, a15", StA, M::Base, 15, 10, 8),
            (&[0xa8, 0x06], "st.h [a15]#0, d6", StH, M::Base, 6, 15, 0),
            (&[0xec, 0x09], "st.a [a0]#0x24, a15", StA, M::Base, 15, 0, 0x24),
            (&[0xe8, 0x75], "st.a [a15]#0x1c, a5", StA, M::Base, 5, 15, 0x1c),
            (&[0x58, 0x60], "ld.w d15, [sp]#0x180", LdW, M::Base, 15, 10, 0x180),
            (&[0x94, 0x60], "ld.h d0, [a6]", LdH, M::Base, 0, 6, 0),
            (&[0xa4, 0x98], "st.h [a9+], d8", StH, M::Post, 8, 9, 2),
            (&[0xd9, 0xee, 0xa8, 0x50], "lea a14, [a14]#0x968", Lea, M::Base, 14, 14, 0x968),
            (&[0xd9, 0xff, 0x7c, 0x18], "lea a15, [a15]#-0x7b84", Lea, M::Base, 15, 15, -0x7b84),
            (&[0x49, 0xff, 0x08, 0x0a], "lea a15, [a15]#8", Lea, M::Base, 15, 15, 8),
            (&[0xc5, 0xfc, 0x10, 0x80], "lea a12, #0xf0000210", Lea, M::Abs, 12, 0, 0xf000_0210u32 as i32),
            (&[0x89, 0x40, 0x04, 0x09], "st.w [a4]#4, d0", StW, M::Base, 0, 4, 4),
            (&[0x09, 0xd1, 0x04, 0x05], "ld.w d1, [+a13]#4", LdW, M::Pre, 1, 13, 4),
            (&[0x09, 0x2f, 0x43, 0x04], "ld.bu d15, [+a2]#3", LdBu, M::Pre, 15, 2, 3),
            (&[0x85, 0xf8, 0x10, 0x80], "ld.w d8, #0xf0000210", LdW, M::Abs, 8, 0, 0xf000_0210u32 as i32),
            (&[0x09, 0xfa, 0xc0, 0x08], "ld.hu d10, [a15]#0", LdHu, M::Base, 10, 15, 0),
            (&[0x89, 0xc0, 0x04, 0x05], "st.w [+a12]#4, d0", StW, M::Pre, 0, 12, 4),
            (&[0x09, 0x2f, 0x90, 0x01], "ld.a a15, [a2+]#0x10", LdA, M::Post, 15, 2, 0x10),
            (&[0x19, 0xf0, 0x10, 0x02], "ld.w d0, [a15]#0x2010", LdW, M::Base, 0, 15, 0x2010),
            (&[0xa5, 0xff, 0xc4, 0xc0], "st.w #0xf0000f04, d15", StW, M::Abs, 15, 0, 0xf000_0f04u32 as i32),
            (&[0x59, 0xc4, 0x20, 0xd5], "st.w [a12]#0x5360, d4", StW, M::Base, 4, 12, 0x5360),
            (&[0x09, 0x00, 0x02, 0x00], "ld.b d0, [a0+]#2", LdB, M::Post, 0, 0, 2),
            (&[0x05, 0x80, 0xff, 0xff], "ld.hu d0, #0x80003fff", LdHu, M::Abs, 0, 0, 0x8000_3fffu32 as i32),
            (&[0x25, 0xff, 0x58, 0x50], "st.b #0xf0000558, d15", StB, M::Abs, 15, 0, 0xf000_0558u32 as i32),
            (&[0x19, 0xff, 0xfc, 0xff], "ld.w d15, [a15]#-4", LdW, M::Base, 15, 15, -4),
            (&[0x09, 0xd4, 0x66, 0xb9], "ld.d e4, [a13]#-0x11a", LdD, M::Base, 4, 13, -0x11a),
            (&[0x09, 0x20, 0x48, 0x01], "ld.d e0, [a2+]#8", LdD, M::Post, 0, 2, 8),
            (&[0x89, 0x40, 0x48, 0x01], "st.d [a4+]#8, e0", StD, M::Post, 0, 4, 8),
            // capstone drops bit 0 of an odd pair specifier (prints p12 for a = 13, p0 for a = 1); the
            // core raises OPD (module docs)
            (&[0x85, 0x0d, 0x8b, 0x0d], "ld.da p12, #0x180b", Opd, M::Abs, 13, 0, 0x180b),
            (&[0xa5, 0x01, 0x0d, 0xfd], "st.da #0x13cd, p0", Opd, M::Abs, 1, 0, 0x13cd),
            (&[0x85, 0x0c, 0x8b, 0x0d], "ld.da p12, #0x180b", LdDa, M::Abs, 12, 0, 0x180b),
            (&[0x49, 0xf2, 0x40, 0x08], "ldmst [a15]#0, e2", Ldmst, M::Base, 2, 15, 0),
            (&[0xe5, 0x1d, 0x03, 0x33], "swap.w #0x100030c3, d13", SwapW, M::Abs, 13, 0, 0x1000_30c3),
            (&[0x69, 0x6a, 0x00, 0x00], "swap.w [p6+r], d10", SwapW, M::Rev, 10, 6, 0),
            // rizin prints `#0` for the bit value, but ABSB b is bit 11 (V2 ST.T p. 3-358), here 1
            (&[0xd5, 0x8e, 0xb0, 0x60], "st.t #0x800009b0, #6, #0", StT, M::Abs, 1, 6, 0x8000_09b0u32 as i32),
            (&[0xd5, 0x00, 0x9a, 0x00], "st.t #0x81a, #0, #0", StT, M::Abs, 0, 0, 0x81a),
            (&[0x15, 0x40, 0x2c, 0x00], "stlcx #0x4000002c", Stlcx, M::Abs, 0, 0, 0x4000_002c),
            (&[0x15, 0x26, 0xdb, 0x04], "stucx #0x20000c1b", Stucx, M::Abs, 6, 0, 0x2000_0c1b),
            (&[0x15, 0x0d, 0x1b, 0x0d], "lducx #0x101b", Lducx, M::Abs, 13, 0, 0x101b),
            (&[0x89, 0x41, 0x52, 0x53], "cachea.wi [a4+]#0x152", CacheAWI, M::Post, 1, 4, 0x152),
            (&[0x89, 0x11, 0x12, 0x23], "cachea.w [a1+]#0x92", CacheAW, M::Post, 1, 1, 0x92),
            (&[0x89, 0x0d, 0xbf, 0xe6], "cachei.i [+a0]#-0x41", CacheII, M::Pre, 13, 0, -0x41),
            (&[0x89, 0x0a, 0x90, 0x0a], "cachei.i [a0]#0x10", CacheII, M::Base, 10, 0, 0x10),
            (&[0x09, 0x83, 0x2c, 0x1a], "ld.q d3, [a8]#0x6c", LdQ, M::Base, 3, 8, 0x6c),
            (&[0x65, 0x85, 0x4e, 0x00], "st.q #0x8000040e, d5", StQ, M::Abs, 5, 0, 0x8000_040eu32 as i32),
            (&[0x79, 0x8c, 0xd3, 0x4e], "ld.b d12, [a8]#-0x12ed", LdB, M::Base, 12, 8, -0x12ed),
            (&[0x09, 0x99, 0x19, 0xc4], "ld.b d9, [+a9]#-0xe7", LdB, M::Pre, 9, 9, -0xe7),
        ];
        for &(b, text, kind, mode, a, base, off) in cases {
            let w = if b.len() == 4 { u32::from_le_bytes(b.try_into().unwrap()) } else { u16::from_le_bytes([b[0], b[1]]) as u32 };
            let op = dec(w, Arch::Tc16).unwrap_or_else(|| panic!("{text} not decoded"));
            assert_eq!((op.kind, op.mode, op.a, op.b, op.off), (kind, mode, a, base, off), "{text}");
        }
        // capstone prints `cachea.w [p0+c]` for the odd pair specifier b = 1 (a9 15 32 27); the core
        // raises OPD instead (module docs).
        assert_eq!(dec(0x2732_15A9, Arch::Tc16).unwrap().kind, Opd);
    }

    // ---- execution: random cases, the manual RTL as reference, Unicorn as second oracle ---------------

    /// A reusable core laid out like `testutil::cpu_with` whose DATA is a buffer the test reads and
    /// writes directly (`Memory::add_raw`), so a random case costs no new 2 MB page table.
    struct Rig {
        c: Cpu<NullBus>,
        data: *mut u8,
    }

    impl Rig {
        fn new(arch: Arch) -> Self {
            let data = Box::into_raw(vec![0u8; DATA_SIZE as usize].into_boxed_slice()) as *mut u8;
            let mut m = Memory::new();
            let code = m.add_owned(vec![0; CODE_SIZE as usize], true);
            m.map(CODE, code, 0, CODE_SIZE as usize);
            // SAFETY: the buffer lives until `drop` and is only accessed between `run` calls.
            let r = unsafe { m.add_raw(data, DATA_SIZE as usize, true) };
            m.map(DATA, r, 0, DATA_SIZE as usize);
            let mut c = Cpu::new(arch, m, NullBus);
            (c.btv, c.biv, c.isp) = (TRAP_BASE, IRQ_BASE, DATA + 0x7000);
            Rig { c, data }
        }

        fn data(&mut self) -> &mut [u8] {
            // SAFETY: see `new`; the core is not running and no other slice of the buffer is alive.
            unsafe { std::slice::from_raw_parts_mut(self.data, DATA_SIZE as usize) }
        }

        /// Place `w` at CODE and restore the CSA free list and context registers of `cpu_with`.
        fn load(&mut self, w: u32) {
            self.c.write_bytes(CODE, &bytes(w)).unwrap();
            for i in 0..8u32 {
                let ea = CSA_BASE + 64 * i;
                let next = if i < 7 { link(ea + 64) } else { 0 };
                let o = (ea - DATA) as usize;
                self.data()[o..o + 4].copy_from_slice(&next.to_le_bytes());
            }
            let c = &mut self.c;
            (c.fcx, c.lcx, c.pcxi, c.icr, c.pc) = (link(CSA_BASE), link(CSA_BASE + 64 * 6), 0, 0, CODE);
        }

        /// Registers and DATA before execution.
        fn snapshot(&mut self) -> Setup {
            let mut s = Setup::from_cpu(&self.c);
            s.mem = vec![(DATA, self.data().to_vec())];
            s
        }

        /// Execute the instruction at CODE.
        fn step(&mut self) -> State {
            let n = self.c.icount;
            assert_eq!(self.c.run(1), crate::Exit::Budget);
            assert_eq!(self.c.icount, n + 1);
            let data = self.data().to_vec();
            State { d: self.c.d, a: self.c.a, pc: self.c.pc, psw: self.c.psw, data }
        }
    }

    impl Drop for Rig {
        fn drop(&mut self) {
            // SAFETY: allocated in `new`; the core never runs again.
            unsafe { drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(self.data, DATA_SIZE as usize))) }
        }
    }

    /// Set up a random, trap-free instance of `row` on `rig`: the instruction word and its decoding.
    fn gen(rig: &mut Rig, row: &Row, rng: &mut Rng) -> (u32, LdStOp) {
        loop {
            if let Some(x) = try_gen(rig, row, rng) {
                return x;
            }
        }
    }

    fn try_gen(rig: &mut Rig, row: &Row, rng: &mut Rng) -> Option<(u32, LdStOp)> {
        let (m, f, mode, ..) = *row;
        let k = kind_of(m);
        let (size, align) = shape(k);
        let mut a = rng.below(16);
        let mut b = rng.below(16);
        if pair_a(k) {
            a &= !1;
        }
        if matches!(mode, M::Rev | M::Circ) {
            b &= !1;
        }
        // circular buffer first: the offset must stay below the length (V1 p. 2-9) and the index is
        // size aligned, so that no access wraps (wrapping is covered by dedicated tests)
        let cbase = DATA + 0x800 + (rng.below(0x4000) & !7);
        let clen = size * (1 + rng.below(64));
        let cidx = size * rng.below(clen / size);
        let off = match f {
            "BO" if mode == M::Circ => {
                let lim = (clen as i32 - 1).min(511);
                rng.below(2 * lim as u32 + 1) as i32 - lim
            }
            "BO" => rng.below(1024) as i32 - 512,
            "BOL" => rng.below(0x10000) as i32 - 0x8000,
            "SC" => rng.below(256) as i32,
            _ => rng.below(16) as i32,
        };
        let abs_ea = DATA + (rng.below(0x4000 - 64) & !(align - 1));
        let w = encode(row, a, b, off, abs_ea);
        let op = dec(w, rig.c.arch)?;
        rig.load(w);
        let c = &mut rig.c;
        for i in 0..16 {
            c.d[i] = rng.next();
            c.a[i] = rng.next();
        }
        // Supervisor mode, PSW.GW = 1, random sticky status bits (must be preserved)
        c.psw = psw::RESET | (rng.next() & 0xF800_0000);
        let ea = DATA + 0x800 + (rng.below(0x6000) & !(align - 1));
        let ob = op.b as usize;
        let (first, updated): (u32, Vec<usize>) = match op.mode {
            M::Abs => (op.off as u32, vec![]),
            M::Base | M::Pre => {
                let base = ea.wrapping_sub(op.off as u32);
                if base >> 28 != DATA >> 28 {
                    return None;
                }
                c.a[ob] = base;
                (ea, if op.mode == M::Pre { vec![ob] } else { vec![] })
            }
            M::Post => {
                c.a[ob] = ea;
                (ea, vec![ob])
            }
            M::Rev => {
                let idx = rng.below(0x4000) & !(align - 1);
                c.a[ob] = cbase;
                c.a[ob + 1] = rng.next() << 16 | idx;
                (cbase + idx, vec![ob, ob + 1])
            }
            M::Circ => {
                c.a[ob] = cbase;
                c.a[ob + 1] = clen << 16 | cidx;
                (cbase, vec![ob, ob + 1])
            }
        };
        // the manual leaves a register that is both data operand and address update undefined
        if a_operands(&op).iter().any(|r| updated.contains(r)) {
            return None;
        }
        let lo = ((first & !7).saturating_sub(0x100).max(DATA) - DATA) as usize;
        let fill: Vec<u8> = (0..0x800).map(|_| rng.next() as u8).collect();
        rig.data()[lo..lo + 0x800].copy_from_slice(&fill);
        Some((w, op))
    }

    struct State {
        d: [u32; 16],
        a: [u32; 16],
        pc: u32,
        psw: u32,
        data: Vec<u8>,
    }

    fn data_of(c: &Cpu<NullBus>) -> Vec<u8> {
        let mut v = vec![0; DATA_SIZE as usize];
        assert!(c.read_bytes(DATA, &mut v));
        v
    }

    /// The V2 RTL of `op` transcribed literally (pages cited in the implementation), applied to the
    /// state `c` (registers and DATA). Covers trap-free executions only.
    fn reference(c: &Setup, op: &LdStOp, len: u32) -> State {
        let (mut d, mut ar) = (c.d, c.a);
        let mut mem = c.mem[0].1.clone();
        let (a, b) = (op.a as usize, op.b as usize);
        let rev = |x: u32| (0..16).fold(0u32, |r, i| r | ((x >> i) & 1) << (15 - i));
        let off = op.off as u32;
        let (ea, update) = match op.mode {
            M::Abs => (off, None),
            M::Base => (ar[b].wrapping_add(off), None),
            M::Post => (ar[b], Some((b, ar[b].wrapping_add(off)))),
            M::Pre => (ar[b].wrapping_add(off), Some((b, ar[b].wrapping_add(off)))),
            M::Rev => {
                let (index, incr) = (ar[b + 1] & 0xFFFF, ar[b + 1] >> 16);
                (ar[b] + index, Some((b + 1, incr << 16 | rev(rev(index) + rev(incr)))))
            }
            M::Circ => {
                let (index, length) = (ar[b + 1] & 0xFFFF, ar[b + 1] >> 16);
                let n = index as i32 + op.off;
                let n = if n < 0 { n + length as i32 } else { n % length as i32 };
                (ar[b] + index, Some((b + 1, length << 16 | (n as u32 & 0xFFFF))))
            }
        };
        // address of byte i of the datum: circular LD.W/LD.D/ST.W/ST.D wrap per halfword, LD.DA/ST.DA
        // per word (EA2 = A[b] + (index + 2) % length, ...)
        let at = |i: u32| -> usize {
            let addr = if op.mode == M::Circ {
                let (base, index, length) = (c.a[b], c.a[b + 1] & 0xFFFF, c.a[b + 1] >> 16);
                let g = match op.kind {
                    LdW | LdD | StW | StD => 2,
                    LdDa | StDa => 4,
                    _ => 64,
                };
                base + (index + i / g * g) % length + i % g
            } else {
                ea + i
            };
            (addr - DATA) as usize
        };
        let rd = |mem: &Vec<u8>, n: u32| (0..n).fold(0u64, |v, i| v | (mem[at(i)] as u64) << (8 * i));
        let wr = |mem: &mut Vec<u8>, n: u32, v: u64| (0..n).for_each(|i| mem[at(i)] = (v >> (8 * i)) as u8);
        if let Some((r, v)) = update {
            ar[r] = v;
        }
        let word = |mem: &mut Vec<u8>, i: u32, v: u32| {
            let o = (ea - DATA + 4 * i) as usize;
            mem[o..o + 4].copy_from_slice(&v.to_le_bytes());
        };
        let rword = |mem: &Vec<u8>, i: u32| {
            let o = (ea - DATA + 4 * i) as usize;
            u32::from_le_bytes(mem[o..o + 4].try_into().unwrap())
        };
        match op.kind {
            LdB => d[a] = rd(&mem, 1) as u8 as i8 as i32 as u32,
            LdBu => d[a] = rd(&mem, 1) as u32,
            LdH => d[a] = rd(&mem, 2) as u16 as i16 as i32 as u32,
            LdHu => d[a] = rd(&mem, 2) as u32,
            LdQ => d[a] = (rd(&mem, 2) as u32) << 16,
            LdW => d[a] = rd(&mem, 4) as u32,
            LdA => ar[a] = rd(&mem, 4) as u32,
            LdD => {
                let v = rd(&mem, 8);
                (d[a], d[a + 1]) = (v as u32, (v >> 32) as u32);
            }
            LdDa => {
                let v = rd(&mem, 8);
                (ar[a], ar[a + 1]) = (v as u32, (v >> 32) as u32);
            }
            StB => wr(&mut mem, 1, c.d[a] as u64),
            StH => wr(&mut mem, 2, c.d[a] as u64),
            StQ => wr(&mut mem, 2, (c.d[a] >> 16) as u64),
            StW => wr(&mut mem, 4, c.d[a] as u64),
            StA => wr(&mut mem, 4, c.a[a] as u64),
            StD => wr(&mut mem, 8, c.d[a] as u64 | (c.d[a + 1] as u64) << 32),
            StDa => wr(&mut mem, 8, c.a[a] as u64 | (c.a[a + 1] as u64) << 32),
            Lea => ar[a] = ea,
            SwapW => {
                let t = rd(&mem, 4) as u32;
                wr(&mut mem, 4, c.d[a] as u64);
                d[a] = t;
            }
            Ldmst => {
                let m = rd(&mem, 4) as u32;
                wr(&mut mem, 4, ((m & !c.d[a + 1]) | (c.d[a] & c.d[a + 1])) as u64);
            }
            StT => {
                let m = rd(&mem, 1) as u8;
                wr(&mut mem, 1, ((m & !(1 << op.b)) | (op.a << op.b)) as u64);
            }
            Stlcx => {
                let ctx = [c.pcxi, c.a[11], c.a[2], c.a[3], c.d[0], c.d[1], c.d[2], c.d[3], c.a[4], c.a[5], c.a[6], c.a[7], c.d[4], c.d[5], c.d[6], c.d[7]];
                (0..16).for_each(|i| word(&mut mem, i, ctx[i as usize]));
            }
            Stucx => {
                let ctx = [c.pcxi, c.psw, c.a[10], c.a[11], c.d[8], c.d[9], c.d[10], c.d[11], c.a[12], c.a[13], c.a[14], c.a[15], c.d[12], c.d[13], c.d[14], c.d[15]];
                (0..16).for_each(|i| word(&mut mem, i, ctx[i as usize]));
            }
            Ldlcx => {
                (ar[2], ar[3]) = (rword(&mem, 2), rword(&mem, 3));
                (0..4).for_each(|i| d[i] = rword(&mem, 4 + i as u32));
                (0..4).for_each(|i| ar[4 + i] = rword(&mem, 8 + i as u32));
                (0..4).for_each(|i| d[4 + i] = rword(&mem, 12 + i as u32));
            }
            Lducx => {
                (ar[10], ar[11]) = (rword(&mem, 2), rword(&mem, 3));
                (0..4).for_each(|i| d[8 + i] = rword(&mem, 4 + i as u32));
                (0..4).for_each(|i| ar[12 + i] = rword(&mem, 8 + i as u32));
                (0..4).for_each(|i| d[12 + i] = rword(&mem, 12 + i as u32));
            }
            CacheAW | CacheAWI | CacheAI | CacheIW | CacheII | CacheIWI | Opd => {}
        }
        State { d, a: ar, pc: CODE + len, psw: c.psw, data: mem }
    }

    fn diff(x: &State, y: &State) -> Option<String> {
        let mut s = String::new();
        for i in 0..16 {
            if x.d[i] != y.d[i] {
                s += &format!(" d{i} {:#x}/{:#x}", x.d[i], y.d[i]);
            }
            if x.a[i] != y.a[i] {
                s += &format!(" a{i} {:#x}/{:#x}", x.a[i], y.a[i]);
            }
        }
        if x.pc != y.pc {
            s += &format!(" pc {:#x}/{:#x}", x.pc, y.pc);
        }
        if x.psw != y.psw {
            s += &format!(" psw {:#x}/{:#x}", x.psw, y.psw);
        }
        if let Some(i) = (x.data != y.data).then(|| x.data.iter().zip(&y.data).position(|(p, q)| p != q)).flatten() {
            s += &format!(" mem[{:#x}] {:#x}/{:#x}", DATA + i as u32, x.data[i], y.data[i]);
        }
        (!s.is_empty()).then_some(s)
    }

    #[test]
    fn every_encoding_follows_the_manual_rtl() {
        let mut rng = Rng(0x0123_4567_89AB_CDEF);
        for arch in [Arch::Tc16, Arch::Tc131] {
            let mut rig = Rig::new(arch);
            for row in MANUAL {
                if dec(encode(row, 0, 0, 0, DATA), arch).is_none() {
                    continue; // not a TC1.3.1 instruction
                }
                for _ in 0..128 {
                    let (w, op) = gen(&mut rig, row, &mut rng);
                    let s = rig.snapshot();
                    let want = reference(&s, &op, if w & 1 == 1 { 4 } else { 2 });
                    if let Some(e) = diff(&rig.step(), &want) {
                        panic!("{} {} {:?} p. {} {arch:?} {w:#010x}: native/manual{e}", row.0, row.1, row.2, row.5);
                    }
                }
            }
        }
    }

    /// (mnemonic, format) rows where Unicorn 2.1.4 contradicts the manual (reported, not compared):
    /// * CACHEI.I (V2 p. 3-55): not implemented, Unicorn raises IOPC;
    /// * LD.H D15, [A[b]]off4 (SRO, V2 p. 3-160): Unicorn adds off4 instead of 2 * off4 (ST.H SRO is
    ///   scaled correctly; rizin/capstone also scale: `8c07 ld.h d15, [a0]#0xe`).
    const UNICORN_DIFFERS: &[(&str, &str)] = &[("CACHEI.I", "BO"), ("LD.H", "SRO")];

    /// Randomised differential test against bare Unicorn (default TriCore model) for every encoding it
    /// is trustworthy for. `SIMEMU_LDST_CASES` sets the number of cases per encoding (default 256).
    #[test]
    fn matches_unicorn_on_random_operands() {
        if !unicorn_tricore_sys::LINKED {
            return;
        }
        let n: usize = std::env::var("SIMEMU_LDST_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(256);
        let mut rng = Rng(0xFEED_FACE_CAFE_BEEF);
        let mut rig = Rig::new(Arch::Tc16);
        let (mut total, mut skipped) = (0, 0);
        let mut bad: Vec<(&Row, usize, String)> = Vec::new();
        for row in MANUAL {
            let (mut count, mut first) = (0, String::new());
            for _ in 0..n {
                let (w, op) = gen(&mut rig, row, &mut rng);
                if !oracle::trustworthy(w as u16, (w >> 16) as u16) {
                    skipped += 1;
                    continue;
                }
                if op.kind == Stucx {
                    // oracle limitation: uc_reg_write(PSW) does not update Unicorn's split PSW status
                    // flags, and STUCX stores psw_read(); the native side is checked by the RTL test
                    rig.c.psw &= 0x07FF_FFFF;
                }
                let s = rig.snapshot();
                let mine = rig.step();
                total += 1;
                let e = match oracle::run(&bytes(w), 1, &s) {
                    Ok(o) => diff(&mine, &State { d: o.d, a: o.a, pc: o.pc, psw: o.psw, data: o.data }),
                    Err(e) => Some(format!(" unicorn error {e}")),
                };
                if let Some(e) = e {
                    count += 1;
                    if first.is_empty() {
                        first = format!("{w:#010x}: native/unicorn{e}");
                    }
                }
            }
            if count > 0 {
                bad.push((row, count, first));
            }
        }
        println!("{total} cases compared, {skipped} skipped (KNOWN_WRONG)");
        for (row, count, first) in &bad {
            println!("{} {} {:?} (p. {}): {count} mismatches, e.g. {first}", row.0, row.1, row.2, row.5);
        }
        let unexpected: Vec<_> = bad.iter().filter(|(row, ..)| !UNICORN_DIFFERS.contains(&(row.0, row.1))).collect();
        assert!(unexpected.is_empty(), "{unexpected:?}");
        assert!(total > 40_000 || n < 256, "{total} cases");
    }

    // ---- literal manual semantics, traps, TC1.3.1 deltas ------------------------------------------------

    const BUF: u32 = DATA + 0x100;

    fn exec1(w: u32, setup: impl FnOnce(&mut Cpu<NullBus>)) -> Cpu<NullBus> {
        run(&bytes(w), 1, setup)
    }

    fn put(c: &mut Cpu<NullBus>, addr: u32, b: &[u8]) {
        c.write_bytes(addr, b).unwrap();
    }

    fn peek(c: &Cpu<NullBus>, addr: u32, n: usize) -> Vec<u8> {
        let mut v = vec![0; n];
        assert!(c.read_bytes(addr, &mut v));
        v
    }

    /// The trap entered by the single executed instruction (vector `BTV | class << 5`, TIN in D[15]).
    fn trapped(c: &Cpu<NullBus>) -> Option<Trap> {
        (TRAP_BASE..TRAP_BASE + 0x100)
            .contains(&c.pc)
            .then(|| Trap::new(((c.pc - TRAP_BASE) >> 5) as u8, c.d[15] as u8))
    }

    const PATTERN: [u8; 16] = [0x80, 0x81, 0xFE, 0x7F, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC];

    fn ld(op2: u8, a: u32, off: i32) -> Cpu<NullBus> {
        exec1(bo(0x09, op2, a, 2, off), |c| {
            c.a[2] = BUF;
            put(c, BUF, &PATTERN);
        })
    }

    #[test]
    #[cfg(feature = "trace-data-stores")]
    fn fast_load_observer_preserves_memory() {
        #[derive(Default)]
        struct Observer(Vec<(u32, u32, u8, u64)>);
        impl Bus for Observer {
            fn read(&mut self, _: u32, _: u8) -> Result<u64, crate::BusError> { Err(crate::BusError) }
            fn write(&mut self, _: u32, _: u8, _: u64) -> Result<(), crate::BusError> { Err(crate::BusError) }
            fn data_read(&mut self, pc: u32, addr: u32, size: u8, value: u64) {
                self.0.push((pc, addr, size, value));
            }
        }
        let mut m = crate::Memory::new();
        let ram = m.add_owned(vec![0xA7; 0x1000], true);
        m.map(0xD000_0000, ram, 0, 0x1000);
        let mut c = Cpu::new(Arch::Tc16, m, Observer::default());
        c.a[2] = 0xD000_0004;
        c.exec_pc = 0x8000_0042;
        let op = LdStOp { kind: Kind::LdBu, mode: Mode::Base, a: 0, b: 2, chk: 0, off: 1 };
        assert!(exec_fast(&mut c, &op).is_some());
        assert_eq!(c.d[0], 0xA7);
        assert_eq!(c.bus.0, [(0x8000_0042, 0xD000_0005, 1, 0xA7)]);
    }

    #[test]
    fn load_widths_and_extension() {
        assert_eq!(ld(0x20, 1, 0).d[1], 0xFFFF_FF80, "LD.B sign_ext");
        assert_eq!(ld(0x21, 1, 0).d[1], 0x80, "LD.BU zero_ext");
        assert_eq!(ld(0x22, 1, 0).d[1], 0xFFFF_8180, "LD.H sign_ext");
        assert_eq!(ld(0x23, 1, 0).d[1], 0x8180, "LD.HU zero_ext");
        assert_eq!(ld(0x28, 1, 0).d[1], 0x8180_0000, "LD.Q {{M(EA, halfword), 0}}");
        assert_eq!(ld(0x22, 1, 2).d[1], 0x7FFE, "LD.H positive");
        assert_eq!(ld(0x24, 1, 2).d[1], 0x2211_7FFE, "LD.W, halfword aligned (V1 Table 2-1)");
        let c = ld(0x25, 4, 2);
        assert_eq!((c.d[4], c.d[5]), (0x2211_7FFE, 0x6655_4433), "LD.D: low word in the even register");
        let c = ld(0x27, 4, 4);
        assert_eq!((c.a[4], c.a[5]), (0x4433_2211, 0x8877_6655), "LD.DA");
        assert_eq!(ld(0x26, 3, 4).a[3], 0x4433_2211, "LD.A");
        assert_eq!(ld(0x26, 2, 4).a[2], 0x4433_2211, "LD.A A[b] as destination (no update): loaded value");
    }

    fn st(op2: u8, a: u32, off: i32, setup: impl FnOnce(&mut Cpu<NullBus>)) -> Vec<u8> {
        let c = exec1(bo(0x89, op2, a, 2, off), |c| {
            c.a[2] = BUF;
            setup(c);
        });
        assert_eq!(trapped(&c), None);
        peek(&c, BUF, 12)
    }

    #[test]
    fn store_widths() {
        let d = |c: &mut Cpu<NullBus>| {
            (c.d[4], c.d[5]) = (0x89AB_CDEF, 0x0123_4567);
            (c.a[4], c.a[5]) = (0x1122_3344, 0x5566_7788);
        };
        assert_eq!(st(0x20, 4, 1, d)[..4], [0, 0xEF, 0, 0], "ST.B D[a][7:0], byte aligned");
        assert_eq!(st(0x22, 4, 2, d)[..4], [0, 0, 0xEF, 0xCD], "ST.H D[a][15:0]");
        assert_eq!(st(0x28, 4, 0, d)[..4], [0xAB, 0x89, 0, 0], "ST.Q D[a][31:16]");
        assert_eq!(st(0x24, 4, 2, d)[..6], [0, 0, 0xEF, 0xCD, 0xAB, 0x89], "ST.W halfword aligned");
        assert_eq!(st(0x25, 4, 2, d)[..10], [0, 0, 0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01], "ST.D E[a]");
        assert_eq!(st(0x26, 4, 4, d)[4..8], [0x44, 0x33, 0x22, 0x11], "ST.A");
        assert_eq!(st(0x27, 4, 4, d)[4..12], [0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55], "ST.DA P[a]");
    }

    #[test]
    fn addressing_mode_updates() {
        // post-increment: EA = A[b]; A[b] = EA + off (V1 §2.5.4)
        let c = ld(0x04, 1, -8);
        assert_eq!((c.d[1], c.a[2]), (0x7FFE_8180, BUF - 8));
        // pre-increment: EA = A[b] + off; A[b] = EA (V1 §2.5.3)
        let c = ld(0x14, 1, 4);
        assert_eq!((c.d[1], c.a[2]), (0x4433_2211, BUF + 4));
        // base + short offset leaves A[b] alone
        assert_eq!(ld(0x24, 1, 4).a[2], BUF);
        // BOL: sign_ext(off16); firmware `lea a15, [a15]#-0x1450` (d9 ff b0 ee)
        let c = exec1(0xEEB0_FFD9, |c| c.a[15] = 0xD000_8000);
        assert_eq!(c.a[15], 0xD000_8000 - 0x1450);
        let c = exec1(bol(0x19, 3, 2, -0x8000), |c| {
            c.a[2] = BUF + 0x8000;
            put(c, BUF, &PATTERN);
        });
        assert_eq!(c.d[3], 0x7FFE_8180, "LD.W BOL off16 = -32768");
        // ABS: EA = {off18[17:14], 14'b0, off18[13:0]} (V1 Figure 2-3)
        let c = exec1(abs(0x85, 0, 7, BUF), |c| put(c, BUF, &PATTERN));
        assert_eq!(c.d[7], 0x7FFE_8180);
        assert_eq!(exec1(abs(0xC5, 0, 3, 0xF000_3FFC), |_| ()).a[3], 0xF000_3FFC, "LEA ABS");
        // 16-bit forms
        let c = exec1(0x0258, |c| {
            c.a[10] = BUF - 8;
            put(c, BUF, &PATTERN);
        });
        assert_eq!(c.d[15], 0x7FFE_8180, "LD.W D15, [A10]4*const8 (SC)");
        let c = exec1(0x2584, |c| {
            c.a[2] = BUF;
            put(c, BUF, &PATTERN);
        });
        assert_eq!((c.d[5], c.a[2]), (0xFFFF_8180, BUF + 2), "LD.H D[c], [A[b]+] (SLR) adds 2");
        let c = exec1(0x2504, |c| {
            c.a[2] = BUF;
            put(c, BUF, &PATTERN);
        });
        assert_eq!((c.d[5], c.a[2]), (0x80, BUF + 1), "LD.BU D[c], [A[b]+] (SLR) adds 1");
        let c = exec1(0x3588, |c| {
            c.a[15] = BUF - 6;
            put(c, BUF, &PATTERN);
        });
        assert_eq!(c.d[5], 0xFFFF_8180, "LD.H D[c], [A15]2*off4 (SLRO)");
        let c = exec1(0x220C, |c| {
            c.a[2] = BUF - 2;
            put(c, BUF, &PATTERN);
        });
        assert_eq!(c.d[15], 0x80, "LD.BU D15, [A[b]]off4 (SRO), unscaled");
        let c = exec1(0x34E8, |c| {
            c.a[15] = BUF - 12;
            c.a[4] = 0xCAFE_F00D;
        });
        assert_eq!(peek(&c, BUF, 4), [0x0D, 0xF0, 0xFE, 0xCA], "ST.A [A15]4*off4, A[a] (SSRO)");
        let c = exec1(0x2464, |c| {
            c.a[2] = BUF;
            c.d[4] = 0x0403_0201;
        });
        assert_eq!((peek(&c, BUF, 4), c.a[2]), (vec![1, 2, 3, 4], BUF + 4), "ST.W [A[b]+], D[a] (SSR)");
        let c = exec1(0x0378, |c| {
            c.a[10] = BUF - 12;
            c.d[15] = 0x0403_0201;
        });
        assert_eq!(peek(&c, BUF, 4), [1, 2, 3, 4], "ST.W [A10]4*const8, D15 (SC)");
    }

    /// Circular index update examples of V1 p. 2-9 (25 halfwords, L = 50).
    #[test]
    fn circular_index_manual_examples() {
        for (index, off, new) in [(48, 2, 0), (48, 4, 2), (4, -8, 46)] {
            let c = exec1(bo(0x29, 0x12, 1, 2, off), |c| {
                c.a[2] = BUF;
                c.a[3] = 50 << 16 | index;
            });
            assert_eq!(trapped(&c), None);
            assert_eq!(c.a[3], 50 << 16 | new, "I = {index}, offset {off}");
        }
    }

    /// V1 Figure 2-6 p. 2-10 and the circular RTL of LD.W/LD.D/LD.DA/ST.W/ST.D/ST.DA: accesses that run
    /// off the end of the buffer wrap to its start, in halfwords (data registers) or words (address
    /// registers).
    #[test]
    fn circular_accesses_wrap_around() {
        // 8 halfwords b0..b7 = 0xB0B0.. 0xB7B7; L = 16
        let buf: Vec<u8> = (0..8u8).flat_map(|i| [0xB0 + i, 0xB0 + i]).collect();
        let circ = |w: u32, index: u32, setup: &dyn Fn(&mut Cpu<NullBus>)| {
            exec1(w, |c| {
                put(c, BUF, &buf);
                c.a[2] = BUF;
                c.a[3] = 16 << 16 | index;
                setup(c);
            })
        };
        let c = circ(bo(0x29, 0x14, 1, 2, 2), 14, &|_| ());
        assert_eq!((c.d[1], c.a[3]), (0xB0B0_B7B7, 16 << 16), "LD.W at element n: {{b0, bn}}");
        let c = circ(bo(0x29, 0x15, 4, 2, 4), 12, &|_| ());
        assert_eq!((c.d[4], c.d[5]), (0xB7B7_B6B6, 0xB1B1_B0B0), "LD.D at n-1");
        let c = circ(bo(0x29, 0x15, 4, 2, 4), 14, &|_| ());
        assert_eq!((c.d[4], c.d[5]), (0xB0B0_B7B7, 0xB2B2_B1B1), "LD.D at n: halfword granularity");
        let c = circ(bo(0x29, 0x17, 4, 2, 8), 12, &|_| ());
        assert_eq!((c.a[4], c.a[5], c.a[3]), (0xB7B7_B6B6, 0xB1B1_B0B0, 16 << 16 | 4), "LD.DA: words");
        let c = circ(bo(0xA9, 0x14, 1, 2, 2), 14, &|c| c.d[1] = 0x1111_2222);
        assert_eq!(peek(&c, BUF, 16)[..2], [0x11, 0x11]);
        assert_eq!(peek(&c, BUF, 16)[14..], [0x22, 0x22], "ST.W wraps");
        let c = circ(bo(0xA9, 0x15, 4, 2, 2), 12, &|c| (c.d[4], c.d[5]) = (0x2222_1111, 0x4444_3333));
        assert_eq!(peek(&c, BUF, 16), [0x33, 0x33, 0x44, 0x44, 0xB2, 0xB2, 0xB3, 0xB3, 0xB4, 0xB4, 0xB5, 0xB5, 0x11, 0x11, 0x22, 0x22]);
        let c = circ(bo(0xA9, 0x17, 4, 2, 2), 12, &|c| (c.a[4], c.a[5]) = (0x2222_1111, 0x4444_3333));
        assert_eq!(peek(&c, BUF, 4), [0x33, 0x33, 0x44, 0x44], "ST.DA wraps per word");
        assert_eq!(c.a[3], 16 << 16 | 14);
    }

    /// V1 Table 2-4 p. 2-11 (1024-point FFT, 16-bit values, M = 0400H): index sequence 0, 1024, 512,
    /// 1536, 256. The table's fourth result 0000010001100000B is a typo (reverse(1536) + reverse(1024) =
    /// 60H + 20H = 80H, reversed 100H).
    #[test]
    fn bit_reverse_fft_sequence() {
        let w = bo(0x29, 0x02, 1, 2, 0);
        let mut c = cpu_with(&[bytes(w), bytes(w), bytes(w), bytes(w)].concat(), Arch::Tc16);
        c.a[2] = BUF;
        c.a[3] = 0x0400 << 16;
        for want in [1024, 512, 1536, 256] {
            c.run(1);
            assert_eq!(c.a[3], 0x0400 << 16 | want);
        }
        assert_eq!(c.icount, 4);
    }

    fn aln(w: u32, setup: impl FnOnce(&mut Cpu<NullBus>)) -> Option<Trap> {
        let c = exec1(w, |c| {
            c.a[2] = BUF;
            setup(c);
        });
        let t = trapped(&c);
        if t.is_some_and(|t| !t.is_async()) {
            assert_eq!(c.a[11], CODE, "synchronous: returns to the instruction");
            assert_eq!(c.a[2], BUF, "a trapping access changes no register");
        }
        t
    }

    #[test]
    fn alignment_rules() {
        let a = Some(Trap::ALN);
        assert_eq!(aln(bo(0x09, 0x20, 1, 2, 1), |_| ()), None, "LD.B byte aligned");
        assert_eq!(aln(bo(0x09, 0x22, 1, 2, 1), |_| ()), a, "LD.H");
        assert_eq!(aln(bo(0x09, 0x28, 1, 2, 3), |_| ()), a, "LD.Q");
        assert_eq!(aln(bo(0x09, 0x24, 1, 2, 1), |_| ()), a, "LD.W odd");
        assert_eq!(aln(bo(0x09, 0x24, 1, 2, 2), |_| ()), None, "LD.W halfword aligned");
        assert_eq!(aln(bo(0x09, 0x26, 1, 2, 2), |_| ()), a, "LD.A word aligned");
        assert_eq!(aln(bo(0x09, 0x27, 4, 2, 2), |_| ()), a, "LD.DA word aligned");
        assert_eq!(aln(bo(0x09, 0x25, 4, 2, 2), |_| ()), None, "LD.D halfword aligned (not peripheral)");
        assert_eq!(aln(bo(0x09, 0x25, 4, 2, 1), |_| ()), a, "LD.D odd");
        assert_eq!(aln(bo(0x89, 0x26, 1, 2, 2), |_| ()), a, "ST.A");
        assert_eq!(aln(bo(0x89, 0x22, 1, 2, 1), |_| ()), a, "ST.H");
        assert_eq!(aln(bo(0x49, 0x20, 1, 2, 2), |_| ()), a, "SWAP.W word aligned");
        assert_eq!(aln(bo(0x49, 0x21, 4, 2, 2), |_| ()), a, "LDMST word aligned");
        assert_eq!(aln(bo(0x49, 0x26, 0, 2, 0x20), |_| ()), a, "STLCX 16-word aligned");
        assert_eq!(aln(bo(0x49, 0x25, 0, 2, -0x40), |_| ()), None, "LDUCX aligned");
        assert_eq!(aln(st_t(BUF + 1, 3, 1), |_| ()), None, "ST.T byte aligned");
        assert_eq!(aln(0x2154, |_| ()), None, "LD.W D[c], [A[b]] (SLR) aligned");
        // LD.D/ST.D in a peripheral segment must be word aligned (V1 p. 2-4); the access then reaches
        // the (empty) bus: DSE / DAE
        assert_eq!(aln(bo(0x09, 0x25, 4, 3, 2), |c| c.a[3] = 0xF000_0000), a);
        assert_eq!(aln(bo(0x09, 0x25, 4, 3, 4), |c| c.a[3] = 0xF000_0000), Some(Trap::DSE));
        assert_eq!(aln(bo(0x89, 0x25, 4, 3, 2), |c| c.a[3] = 0xE000_0000), Some(Trap::DAE), "PMA0 = 0: E not peripheral");
        let pma = |c: &mut Cpu<NullBus>| c.csfr_set_reset(0x801C, 0xC000_03FF);
        assert_eq!(aln(bo(0x89, 0x25, 4, 3, 2), |c| (pma(c), c.a[3] = 0xE000_0000).1), a, "PMA0 reset: E peripheral");
    }

    /// Circular-buffer restrictions (V1 p. 2-10): 64-bit aligned start, length a multiple of the data
    /// size, index < length; otherwise ALN.
    #[test]
    fn circular_buffer_restrictions() {
        let ldw = bo(0x29, 0x14, 1, 2, 4);
        let t = |base: u32, len: u32, idx: u32| {
            let c = exec1(ldw, |c| {
                c.a[2] = base;
                c.a[3] = len << 16 | idx;
            });
            assert_eq!(c.a[3], len << 16 | if trapped(&c).is_some() { idx } else { (idx + 4) % len });
            trapped(&c)
        };
        assert_eq!(t(BUF, 16, 4), None);
        assert_eq!(t(BUF + 4, 16, 4), Some(Trap::ALN), "base not 64-bit aligned");
        assert_eq!(t(BUF, 18, 4), Some(Trap::ALN), "length not a multiple of 4");
        assert_eq!(t(BUF, 16, 16), Some(Trap::ALN), "index >= length");
        assert_eq!(t(BUF, 0, 0), Some(Trap::ALN), "length 0");
        assert_eq!(t(BUF, 16, 2), None, "halfword aligned index (LD.W)");
        // cache operations follow the same addressing-mode rules
        let c = exec1(bo(0xA9, 0x1C, 0, 2, 4), |c| {
            c.a[2] = BUF;
            c.a[3] = 8 << 16 | 8;
        });
        assert_eq!(trapped(&c), Some(Trap::ALN));
    }

    #[test]
    fn mem_traps() {
        let m = Some(Trap::MEM);
        // different segment: base + offset leaves segment D (V1 p. 6-7)
        assert_eq!(aln(bo(0x09, 0x24, 1, 3, -4), |c| c.a[3] = DATA), m, "base + short offset");
        assert_eq!(aln(bol(0x19, 1, 3, -4), |c| c.a[3] = DATA), m, "base + long offset");
        assert_eq!(aln(bo(0x09, 0x14, 1, 3, -4), |c| c.a[3] = DATA), m, "pre-increment");
        assert_eq!(aln(0x3058, |c| c.a[10] = 0xCFFF_FF80), m, "SC: A10 + 4 * const8");
        assert_eq!(aln(0x3058, |c| c.a[10] = 0xCFFF_FF00), Some(Trap::DSE), "same segment: reaches the bus");
        // segment crossing: the datum spans DFFFFFFFH/E0000000H
        assert_eq!(aln(bo(0x09, 0x24, 1, 3, 0xE), |c| c.a[3] = 0xDFFF_FFF0), m, "word");
        assert_eq!(aln(bo(0x09, 0x25, 4, 3, 0xC), |c| c.a[3] = 0xDFFF_FFF0), m, "double-word");
        // post-increment: the EA is A[b] itself, the update may leave the segment
        let c = exec1(bo(0x09, 0x04, 1, 3, 8), |c| {
            c.a[3] = DATA + 0xFFFC;
            put(c, DATA + 0xFFFC, &[1, 2, 3, 4]);
        });
        assert_eq!((trapped(&c), c.d[1], c.a[3]), (None, 0x0403_0201, DATA + 0x1_0004));
        // LEA does not access memory: no MEM trap
        assert_eq!(exec1(bo(0x49, 0x28, 1, 3, -4), |c| c.a[3] = DATA).a[1], DATA - 4);
    }

    #[test]
    fn protection_traps_and_bus_errors() {
        // MPN: load/store to effective address zero (V1 p. 6-7)
        assert_eq!(aln(0x3054, |c| c.a[3] = 0), Some(Trap::MPN), "LD.W D[0], [A3]");
        assert_eq!(aln(bo(0x89, 0x24, 1, 3, 4), |c| c.a[3] = 0xFFFF_FFFC), Some(Trap::MEM), "MEM outranks MPN");
        // MPP: User-0 access to a peripheral segment (V1 p. 6-7); User-1 reaches the bus
        assert_eq!(aln(bo(0x09, 0x24, 1, 3, 0), |c| (c.psw = 0, c.a[3] = 0xF000_0000).1), Some(Trap::MPP));
        assert_eq!(aln(bo(0x09, 0x24, 1, 3, 0), |c| (c.psw = 1 << 10, c.a[3] = 0xF000_0000).1), Some(Trap::DSE));
        assert_eq!(aln(bo(0x09, 0x24, 1, 2, 0), |c| c.psw = 0), None, "User-0 RAM access");
        // DSE (load) is synchronous: no register update, return to the load
        let c = exec1(bo(0x09, 0x04, 1, 3, 4), |c| c.a[3] = 0x5000_0000);
        assert_eq!((trapped(&c), c.a[3], c.a[11]), (Some(Trap::DSE), 0x5000_0000, CODE));
        // DAE (store) is asynchronous: the instruction completes, return to the next one
        let c = exec1(bo(0x89, 0x04, 1, 3, 4), |c| c.a[3] = 0x5000_0000);
        assert_eq!((trapped(&c), c.a[3], c.a[11]), (Some(Trap::DAE), 0x5000_0004, CODE + 4));
        let c = exec1(bo(0x49, 0x00, 1, 3, 4), |c| (c.a[3], c.d[1]) = (0x5000_0000, 7));
        assert_eq!((trapped(&c), c.d[1]), (Some(Trap::DSE), 7), "SWAP.W load fails first");
    }

    #[test]
    fn global_register_write_protection_and_privilege() {
        let no_gw = |c: &mut Cpu<NullBus>| c.psw &= !psw::GW;
        let g = Some(Trap::GRWP);
        assert_eq!(aln(abs(0xC5, 0, 0, BUF), no_gw), g, "LEA A0");
        assert_eq!(aln(abs(0xC5, 0, 3, BUF), no_gw), None, "LEA A3");
        assert_eq!(aln(bo(0x09, 0x26, 9, 2, 0), no_gw), g, "LD.A A9");
        assert_eq!(aln(bo(0x09, 0x27, 8, 2, 0), no_gw), g, "LD.DA P8 = A8/A9");
        assert_eq!(aln(bo(0x09, 0x04, 1, 1, 4), |c| (no_gw(c), c.a[1] = BUF).1), g, "post-increment of A1");
        assert_eq!(aln(bo(0x29, 0x14, 1, 0, 4), |c| (no_gw(c), c.a[0] = BUF, c.a[1] = 8 << 16).2), g, "P0: A1 update");
        assert_eq!(aln(bo(0x09, 0x24, 1, 1, 0), |c| (no_gw(c), c.a[1] = BUF).1), None, "A1 only read");
        assert_eq!(aln(abs(0xC5, 0, 0, BUF), |_| ()), None, "GW = 1");
        let c = exec1(abs(0xC5, 0, 8, BUF), no_gw);
        assert_eq!((c.a[11], c.a[8] == BUF), (CODE, false), "GRWP: synchronous, A8 unchanged");
        // CACHEA.I / CACHEI.I are Supervisor only; PRIV outranks GRWP (V1 Table 6-2)
        let user1 = |c: &mut Cpu<NullBus>| c.psw = (c.psw & !psw::IO) | 1 << 10;
        assert_eq!(aln(bo(0x89, 0x2E, 0, 2, 0), user1), Some(Trap::PRIV), "CACHEA.I");
        assert_eq!(aln(bo(0x89, 0x2A, 0, 2, 0), user1), Some(Trap::PRIV), "CACHEI.I");
        assert_eq!(aln(bo(0x89, 0x2C, 0, 2, 0), user1), None, "CACHEA.W is not privileged");
        assert_eq!(aln(bo(0x89, 0x0E, 0, 0, 4), |c| (user1(c), no_gw(c)).1), Some(Trap::PRIV));
    }

    #[test]
    fn odd_register_pairs_raise_opd() {
        for w in [bo(0x09, 0x25, 3, 2, 0), bo(0x89, 0x27, 5, 2, 0), bo(0x49, 0x21, 1, 2, 0), bo(0x29, 0x14, 1, 3, 0), abs(0x85, 1, 7, BUF)] {
            let op = dec(w, Arch::Tc16).unwrap();
            assert!(op.kind == Opd && ends_block(&op), "{w:#010x}");
            let c = exec1(w, |_| ());
            assert_eq!((trapped(&c), c.a[11]), (Some(Trap::OPD), CODE));
        }
    }

    #[test]
    fn context_store_and_load() {
        let setup = |c: &mut Cpu<NullBus>| {
            for i in 0..16 {
                c.d[i] = 0xD0 + i as u32;
                c.a[i] = 0xA0 + i as u32;
            }
            c.pcxi = 0x0012_3456;
            c.psw = psw::RESET | psw::C;
        };
        let words = |c: &Cpu<NullBus>| -> Vec<u32> { peek(c, BUF, 64).chunks(4).map(|w| u32::from_le_bytes(w.try_into().unwrap())).collect() };
        let c = exec1(abs(0x15, 0, 0, BUF), setup);
        let l: Vec<u32> = [0x0012_3456, 0xAB, 0xA2, 0xA3, 0xD0, 0xD1, 0xD2, 0xD3, 0xA4, 0xA5, 0xA6, 0xA7, 0xD4, 0xD5, 0xD6, 0xD7].into();
        assert_eq!(words(&c), l, "STLCX {{PCXI, A[11], A[2:3], D[0:3], A[4:7], D[4:7]}}");
        let c = exec1(bo(0x49, 0x27, 0, 2, 0x40), |c| (setup(c), c.a[2] = BUF - 0x40).1);
        let u: Vec<u32> = [0x0012_3456, psw::RESET | psw::C, 0xAA, 0xAB, 0xD8, 0xD9, 0xDA, 0xDB, 0xAC, 0xAD, 0xAE, 0xAF, 0xDC, 0xDD, 0xDE, 0xDF].into();
        assert_eq!(words(&c), u, "STUCX {{PCXI, PSW, A[10:11], D[8:11], A[12:15], D[12:15]}}");
        let ctx: Vec<u8> = (0..16u32).flat_map(|i| (0x100 + i).to_le_bytes()).collect();
        for (w, lower) in [(abs(0x15, 2, 0, BUF), true), (abs(0x15, 3, 0, BUF), false)] {
            let c = exec1(w, |c| {
                setup(c);
                put(c, BUF, &ctx);
            });
            assert_eq!((c.pcxi, c.psw, c.fcx), (0x0012_3456, psw::RESET | psw::C, crate::testutil::link(crate::testutil::CSA_BASE)));
            if lower {
                assert_eq!((c.a[2], c.a[3], c.d[0], c.a[4], c.d[7], c.a[11]), (0x102, 0x103, 0x104, 0x108, 0x10F, 0xAB), "LDLCX");
                assert_eq!((c.d[8], c.a[10]), (0xD8, 0xAA));
            } else {
                assert_eq!((c.a[10], c.a[11], c.d[8], c.a[12], c.d[15]), (0x102, 0x103, 0x104, 0x108, 0x10F), "LDUCX");
                assert_eq!((c.d[0], c.a[2]), (0xD0, 0xA2));
            }
        }
    }

    #[test]
    fn ldmst_swap_and_store_bit() {
        let c = exec1(bo(0x49, 0x21, 4, 2, 0), |c| {
            c.a[2] = BUF;
            put(c, BUF, &0x1234_5678u32.to_le_bytes());
            (c.d[4], c.d[5]) = (0xAAAA_AAAA, 0x0F0F_00FF);
        });
        assert_eq!(peek(&c, BUF, 4), 0x1A3A_56AAu32.to_le_bytes(), "LDMST (M & ~mask) | (value & mask)");
        let c = exec1(bo(0x49, 0x10, 3, 2, 4), |c| {
            c.a[2] = BUF - 4;
            put(c, BUF, &0x1234_5678u32.to_le_bytes());
            c.d[3] = 0xCAFE_BABE;
        });
        assert_eq!((c.d[3], peek(&c, BUF, 4), c.a[2]), (0x1234_5678, 0xCAFE_BABEu32.to_le_bytes().to_vec(), BUF), "SWAP.W");
        let c = exec1(st_t(BUF + 1, 6, 1), |c| put(c, BUF, &[0x00, 0x81]));
        assert_eq!(peek(&c, BUF, 2), [0x00, 0xC1], "ST.T set");
        let c = exec1(st_t(BUF + 1, 7, 0), |c| put(c, BUF, &[0x00, 0x81]));
        assert_eq!(peek(&c, BUF, 2), [0x00, 0x01], "ST.T clear");
    }

    #[test]
    fn cache_operations_only_update_addresses() {
        let c = exec1(bo(0x89, 0x0C, 0, 2, 8), |c| (c.a[2] = BUF, put(c, BUF, &PATTERN)).0);
        assert_eq!((c.a[2], peek(&c, BUF, 16)), (BUF + 8, PATTERN.to_vec()), "CACHEA.W post-increment");
        assert_eq!(exec1(bo(0x89, 0x1F, 0, 2, -8), |c| c.a[2] = 0x1234).a[2], 0x122C, "CACHEI.WI pre-increment, no checks");
        assert_eq!(exec1(bo(0x89, 0x2B, 0, 2, -8), |c| c.a[2] = 0).a[2], 0, "CACHEI.W base: no update, no MPN");
        let c = exec1(bo(0xA9, 0x0E, 0, 2, 0), |c| (c.a[2], c.a[3]) = (BUF, 0x0400 << 16));
        assert_eq!(c.a[3], 0x0400 << 16 | 0x400, "CACHEA.I bit-reverse");
        let c = exec1(bo(0xA9, 0x1D, 0, 2, -4), |c| (c.a[2], c.a[3]) = (BUF, 16 << 16));
        assert_eq!(c.a[3], 16 << 16 | 12, "CACHEA.WI circular");
    }

    /// TC1.3.1 (V131-2): no TC1.6 BOL byte/halfword/ST.A forms, no CACHEI.I; LD.D/ST.D halfword aligned
    /// only in cacheable memory or the DSPR (V131-1 p. 2-4).
    #[test]
    fn tc131_differences() {
        for w in [bol(0x39, 1, 2, 0), bol(0xB5, 1, 2, 0), bo(0x89, 0x2A, 0, 2, 0)] {
            let c = run_arch(&bytes(w), 1, Arch::Tc131, |c| c.a[2] = BUF);
            assert_eq!(trapped(&c), Some(Trap::IOPC), "{w:#010x}");
            assert_eq!(trapped(&run(&bytes(w), 1, |c| c.a[2] = BUF)), None);
        }
        let core = |arch: Arch, base: u32| {
            let mut m = Memory::new();
            let code = m.add_owned(bytes(bo(0x09, 0x25, 4, 2, 2)), true);
            let ram = m.add_owned(vec![0; 0x100], true);
            let csa = m.add_owned(vec![0; 0x40], true);
            m.map(CODE, code, 0, 4);
            m.map(base, ram, 0, 0x100);
            m.map(0xB000_0000, csa, 0, 0x40);
            let mut c = Cpu::new(arch, m, NullBus);
            // one free CSA (link word 0) so that the trap entry succeeds
            (c.pc, c.a[2], c.btv, c.fcx) = (CODE, base, 0x7000_0000, link(0xB000_0000));
            c.run(1);
            c.pc
        };
        let ok = CODE + 4;
        let aln = 0x7000_0000 | 2 << 5;
        assert_eq!(core(Arch::Tc131, 0xA000_0000), aln, "TC1.3.1 non-cacheable segment A");
        assert_eq!(core(Arch::Tc131, 0x8000_0000), ok, "TC1.3.1 cacheable segment 8");
        assert_eq!(core(Arch::Tc131, 0xD000_0000), ok, "TC1.3.1 DSPR");
        assert_eq!(core(Arch::Tc131, 0xD400_0000), aln, "TC1.3.1 PSPR image in segment D");
        assert_eq!(core(Arch::Tc16, 0xA000_0000), ok, "TC1.6: only peripheral space needs word alignment");
    }

    /// Unicorn 2.1.4 defect (`oracle::KNOWN_WRONG`): BO LD.BU pre-increment sign-extends; the manual
    /// zero-extends (V2 p. 3-152). Firmware: `ld.bu d15, [+a2]#3` (09 2f 43 04).
    #[test]
    fn ld_bu_pre_increment_zero_extends() {
        let c = exec1(0x0443_2F09, |c| {
            c.a[2] = BUF - 3;
            put(c, BUF, &[0x9C]);
        });
        assert_eq!((c.d[15], c.a[2]), (0x9C, BUF));
        assert!(!oracle::trustworthy(0x2F09, 0x0443));
    }

    /// Cases where the manual (and the native core) and bare Unicorn differ, beyond `KNOWN_WRONG`.
    /// The native result is asserted; Unicorn's is printed (`--nocapture`) for the record.
    #[test]
    fn unicorn_disagreements() {
        let buf: Vec<u8> = (0..16u8).map(|i| 0xB0 + i).collect();
        let probe = |name: &str, w: u32, setup: &dyn Fn(&mut Cpu<NullBus>), check: &dyn Fn(&Cpu<NullBus>)| {
            let mut c = cpu_with(&bytes(w), Arch::Tc16);
            put(&mut c, BUF, &buf);
            setup(&mut c);
            let mut s = Setup::from_cpu(&c);
            s.mem = vec![(DATA, data_of(&c))];
            c.run(1);
            check(&c);
            let mine = State { d: c.d, a: c.a, pc: c.pc, psw: c.psw, data: data_of(&c) };
            let verdict = match oracle::run(&bytes(w), 1, &s) {
                Ok(o) => diff(&mine, &State { d: o.d, a: o.a, pc: o.pc, psw: o.psw, data: o.data }).unwrap_or_else(|| " (agrees)".into()),
                Err(e) => format!(" unicorn error {e}"),
            };
            println!("{name}: native/unicorn{verdict}");
        };
        let circ = |len: u32, idx: u32| move |c: &mut Cpu<NullBus>| (c.a[2], c.a[3]) = (BUF, len << 16 | idx);
        probe("LD.W circular at index L-2 (wraps per halfword)", bo(0x29, 0x14, 1, 2, 2), &circ(16, 14), &|c| assert_eq!(c.d[1], 0xB1B0_BFBE));
        probe("LD.D circular at index L-2 (halfword granularity)", bo(0x29, 0x15, 4, 2, 2), &circ(16, 14), &|c| assert_eq!((c.d[4], c.d[5]), (0xB1B0_BFBE, 0xB5B4_B3B2)));
        probe("ST.W circular at index L-2", bo(0xA9, 0x14, 1, 2, 2), &|c| (circ(16, 14)(c), c.d[1] = 0x1111_2222).1, &|c| assert_eq!(peek(c, BUF, 2), [0x11, 0x11]));
        probe("circular index update with |offset| > L (new_index stays negative)", bo(0x29, 0x12, 1, 2, -40), &circ(16, 2), &|c| assert_eq!(c.a[3], 16 << 16 | (2u32.wrapping_sub(40).wrapping_add(16) & 0xFFFF)));
        probe("circular index >= length", bo(0x29, 0x12, 1, 2, 2), &circ(16, 16), &|c| assert_eq!(trapped(c), Some(Trap::ALN)));
        probe("LD.W odd address", bo(0x09, 0x24, 1, 2, 1), &|c| c.a[2] = BUF, &|c| assert_eq!(trapped(c), Some(Trap::ALN)));
        probe("LD.A halfword address", bo(0x09, 0x26, 1, 2, 2), &|c| c.a[2] = BUF, &|c| assert_eq!(trapped(c), Some(Trap::ALN)));
        probe("LD.W base + offset into another segment", bo(0x09, 0x24, 1, 2, -4), &|c| c.a[2] = DATA, &|c| assert_eq!(trapped(c), Some(Trap::MEM)));
        probe("LEA A0 with PSW.GW = 0", abs(0xC5, 0, 0, BUF), &|c| c.psw &= !psw::GW, &|c| assert_eq!(trapped(c), Some(Trap::GRWP)));
        probe("CACHEA.I in User-1 mode", bo(0x89, 0x2E, 0, 2, 0), &|c| (c.a[2], c.psw) = (BUF, 0x0580), &|c| assert_eq!(trapped(c), Some(Trap::PRIV)));
        probe("LD.D with odd pair E[3]", bo(0x09, 0x25, 3, 2, 0), &|c| c.a[2] = BUF, &|c| assert_eq!(trapped(c), Some(Trap::OPD)));
        probe("LD.H D15, [A2]2*off4 (SRO, off4 = 3)", h16(0x8C, 2, 3), &|c| c.a[2] = BUF, &|c| assert_eq!(c.d[15], 0xFFFF_B7B6));
        probe("CACHEI.I [A2]0 in Supervisor mode", bo(0x89, 0x2A, 0, 2, 0), &|c| c.a[2] = BUF, &|c| assert_eq!(trapped(c), None));
        probe("LDLCX leaves A11", abs(0x15, 2, 0, BUF), &|c| c.a[11] = 0x1234, &|c| assert_eq!(c.a[11], 0x1234));
        probe("LDUCX leaves PSW", abs(0x15, 3, 0, BUF), &|_| (), &|c| assert_eq!(c.psw, psw::RESET));
    }

    #[test]
    fn tc13_and_tc131_reject_tc16_only_bol_encodings() {
        for op1 in [0x19u8, 0x59, 0x99, 0xD9] {
            let w = bol(op1, 2, 3, 0x1234);
            for arch in [Arch::Tc13, Arch::Tc131, Arch::Tc16] {
                assert!(dec(w, arch).is_some(), "BOL {op1:#04x} must decode on {arch:?}");
            }
        }
        for op1 in [0x39u8, 0x79, 0xB9, 0xC9, 0xB5, 0xE9, 0xF9] {
            let w = bol(op1, 2, 3, 0x1234);
            assert!(dec(w, Arch::Tc16).is_some(), "BOL {op1:#04x} must decode on Tc16");
            for arch in [Arch::Tc13, Arch::Tc131] {
                assert!(dec(w, arch).is_none(), "TC1.6-only BOL {op1:#04x} decoded on {arch:?}");
            }
        }
    }

    /// Throughput of predecoded straight-line load/store code (release: `cargo test -p tricore-core
    /// --release --lib ldst_throughput -- --ignored --nocapture`).
    #[test]
    #[ignore]
    fn ldst_throughput() {
        let mips = |name: &str, seq: &[u32]| {
            let unit: Vec<u8> = seq.iter().flat_map(|&w| bytes(w)).collect();
            let reps = 0x1F00 / unit.len();
            let n = (reps * seq.len()) as u64;
            let mut c = cpu_with(&unit.repeat(reps), Arch::Tc16);
            let t = std::time::Instant::now();
            let mut total = 0u64;
            while t.elapsed().as_secs_f64() < 0.5 {
                (c.pc, c.a[10], c.a[2], c.a[3], c.a[4], c.a[15], c.a[6]) = (CODE, BUF, BUF, BUF, BUF, BUF, BUF);
                c.run(n);
                total += n;
            }
            assert_eq!(c.icount, total);
            println!("{name:<44} {:>5.0} MIPS", total as f64 / t.elapsed().as_secs_f64() / 1e6);
        };
        let (lea, ldw_sc, stw_post, ldbu_bol) = (bol(0xD9, 6, 6, 4), 0x0258, h16(0x64, 15, 2), bol(0x39, 1, 3, 0x100));
        let (sth_bo, lda_slro, ldd) = (bo(0x89, 0x22, 1, 4, 2), h16(0xC8, 1, 5), bo(0x09, 0x25, 4, 3, 8));
        mips("lea a6, [a6]4 (BOL)", &[lea]);
        mips("ld.w d15, [a10]8 (SC)", &[ldw_sc]);
        mips("ld.bu d1, [a3]0x100 (BOL)", &[ldbu_bol]);
        mips("st.w [a15+], d2 (SSR)", &[stw_post]);
        mips("st.h [a4]2, d1 (BO)", &[sth_bo]);
        mips("ld.d e4, [a3]8 (BO)", &[ldd]);
        mips("mix: SC/SSR/BOL/BO/SLRO/LEA", &[ldw_sc, stw_post, ldbu_bol, sth_bo, lda_slro, lea]);
    }
}
