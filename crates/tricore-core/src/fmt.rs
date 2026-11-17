//! Field extractors for every TriCore instruction format.
//!
//! Source: V2 (TriCore V1.6 Vol. 2) Table 1-5 "16-bit Opcode Formats" (p. 1-3), Table 1-6 "32-bit
//! Opcode Formats" (p. 1-4) and Table 1-7 "Opcode Field Definitions" (p. 1-5); identical in V131-2
//! §2.2. Register fields are returned as register numbers (0..=15); `s1` is named `a`, `s2` `b`,
//! `s3` `d`, `d` `c`, following the operand names the instruction pages use (`D[a]`, `D[b]`, `D[d]`,
//! `D[c]`). Immediates are returned raw (unsigned); use [`sext`] where the instruction page says
//! `sign_ext`.
//!
//! Bit 0 of op1 distinguishes the instruction length: 0 = 16-bit, 1 = 32-bit (V2 p. 1-3/1-4).

/// Instruction length in bytes from the first halfword.
#[inline(always)]
pub const fn insn_len(lo: u16) -> u32 {
    if lo & 1 == 0 {
        2
    } else {
        4
    }
}

/// The 32-bit instruction word from its two halfwords (little-endian fetch).
#[inline(always)]
pub const fn word(lo: u16, hi: u16) -> u32 {
    lo as u32 | (hi as u32) << 16
}

/// Sign-extend the low `bits` bits of `v`.
#[inline(always)]
pub const fn sext(v: u32, bits: u32) -> u32 {
    (((v << (32 - bits)) as i32) >> (32 - bits)) as u32
}

#[inline(always)]
const fn f(w: u32, lo: u32, n: u32) -> u32 {
    (w >> lo) & ((1u32 << n) - 1)
}

macro_rules! format32 {
    ($(#[$m:meta])* $name:ident { $($field:ident : $lo:literal, $n:literal);* $(;)? }) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub struct $name { pub op1: u8, $(pub $field: u32),* }
        impl $name {
            #[inline(always)]
            pub const fn from(w: u32) -> Self {
                Self { op1: (w & 0xFF) as u8, $($field: f(w, $lo, $n)),* }
            }
        }
    };
}

macro_rules! format16 {
    ($(#[$m:meta])* $name:ident { $($field:ident : $lo:literal, $n:literal);* $(;)? }) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub struct $name { pub op1: u8, $(pub $field: u32),* }
        impl $name {
            #[inline(always)]
            pub const fn from(h: u16) -> Self {
                let w = h as u32;
                Self { op1: (w & 0xFF) as u8, $($field: f(w, $lo, $n)),* }
            }
        }
    };
}

// ---- 32-bit formats (V2 Table 1-6, p. 1-4) -------------------------------------------------------

/// ABS: absolute addressing. `off18 = {[31:28]→[9:6], [25:22]→[13:10], [21:16]→[5:0], [15:12]→[17:14]}`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Abs {
    pub op1: u8,
    pub op2: u32,
    pub off18: u32,
    pub a: u32,
}
impl Abs {
    #[inline(always)]
    pub const fn from(w: u32) -> Self {
        Self { op1: (w & 0xFF) as u8, op2: f(w, 26, 2), off18: off18(w), a: f(w, 8, 4) }
    }
    /// Effective address `{off18[17:14], 14'b0, off18[13:0]}` (V2 LD.W ABS form).
    #[inline(always)]
    pub const fn ea(&self) -> u32 {
        ((self.off18 >> 14) << 28) | (self.off18 & 0x3FFF)
    }
}

#[inline(always)]
const fn off18(w: u32) -> u32 {
    f(w, 16, 6) | f(w, 28, 4) << 6 | f(w, 22, 4) << 10 | f(w, 12, 4) << 14
}

/// ABSB: absolute bit addressing (ST.T). Same off18 layout as ABS; `b` bit 11, `bpos3 [10:8]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Absb {
    pub op1: u8,
    pub op2: u32,
    pub off18: u32,
    pub b: u32,
    pub bpos3: u32,
}
impl Absb {
    #[inline(always)]
    pub const fn from(w: u32) -> Self {
        Self { op1: (w & 0xFF) as u8, op2: f(w, 26, 2), off18: off18(w), b: f(w, 11, 1), bpos3: f(w, 8, 3) }
    }
    #[inline(always)]
    pub const fn ea(&self) -> u32 {
        ((self.off18 >> 14) << 28) | (self.off18 & 0x3FFF)
    }
}

/// B: `disp24 = {[15:8]→[23:16], [31:16]→[15:0]}`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct B {
    pub op1: u8,
    pub disp24: u32,
}
impl B {
    #[inline(always)]
    pub const fn from(w: u32) -> Self {
        Self { op1: (w & 0xFF) as u8, disp24: f(w, 16, 16) | f(w, 8, 8) << 16 }
    }
}

format32!(/// BIT: `c [31:28], pos2 [27:23], op2 [22:21], pos1 [20:16], b [15:12], a [11:8]`.
    Bit { c: 28, 4; pos2: 23, 5; op2: 21, 2; pos1: 16, 5; b: 12, 4; a: 8, 4 });

/// BO: base + offset. `off10 = {[31:28]→[9:6], [21:16]→[5:0]}`, `op2 [27:22], b [15:12], a [11:8]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bo {
    pub op1: u8,
    pub op2: u32,
    pub off10: u32,
    pub b: u32,
    pub a: u32,
}
impl Bo {
    #[inline(always)]
    pub const fn from(w: u32) -> Self {
        Self { op1: (w & 0xFF) as u8, op2: f(w, 22, 6), off10: f(w, 16, 6) | f(w, 28, 4) << 6, b: f(w, 12, 4), a: f(w, 8, 4) }
    }
}

/// BOL: base + long offset. `off16 = {[27:22]→[15:10], [31:28]→[9:6], [21:16]→[5:0]}`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bol {
    pub op1: u8,
    pub off16: u32,
    pub b: u32,
    pub a: u32,
}
impl Bol {
    #[inline(always)]
    pub const fn from(w: u32) -> Self {
        Self { op1: (w & 0xFF) as u8, off16: f(w, 16, 6) | f(w, 28, 4) << 6 | f(w, 22, 6) << 10, b: f(w, 12, 4), a: f(w, 8, 4) }
    }
}

format32!(/// BRC: `op2 [31], disp15 [30:16], const4 [15:12], a [11:8]`.
    Brc { op2: 31, 1; disp15: 16, 15; const4: 12, 4; a: 8, 4 });

/// BRN: `op2 [31], disp15 [30:16], n = {[7]→[4], [15:12]→[3:0]}, a [11:8]`; `op1` is bits `[6:0]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Brn {
    pub op1: u8,
    pub op2: u32,
    pub disp15: u32,
    pub n: u32,
    pub a: u32,
}
impl Brn {
    #[inline(always)]
    pub const fn from(w: u32) -> Self {
        Self { op1: (w & 0x7F) as u8, op2: f(w, 31, 1), disp15: f(w, 16, 15), n: f(w, 12, 4) | f(w, 7, 1) << 4, a: f(w, 8, 4) }
    }
}

format32!(/// BRR: `op2 [31], disp15 [30:16], b [15:12], a [11:8]`.
    Brr { op2: 31, 1; disp15: 16, 15; b: 12, 4; a: 8, 4 });
format32!(/// RC: `c [31:28], op2 [27:21], const9 [20:12], a [11:8]`.
    Rc { c: 28, 4; op2: 21, 7; const9: 12, 9; a: 8, 4 });
format32!(/// RCPW: `c [31:28], pos [27:23], op2 [22:21], width [20:16], const4 [15:12], a [11:8]`.
    Rcpw { c: 28, 4; pos: 23, 5; op2: 21, 2; width: 16, 5; const4: 12, 4; a: 8, 4 });
format32!(/// RCR: `c [31:28], d [27:24], op2 [23:21], const9 [20:12], a [11:8]`.
    Rcr { c: 28, 4; d: 24, 4; op2: 21, 3; const9: 12, 9; a: 8, 4 });
format32!(/// RCRR: `c [31:28], d [27:24], op2 [23:21], const4 [15:12], a [11:8]`.
    Rcrr { c: 28, 4; d: 24, 4; op2: 21, 3; const4: 12, 4; a: 8, 4 });
format32!(/// RCRW: `c [31:28], d [27:24], op2 [23:21], width [20:16], const4 [15:12], a [11:8]`.
    Rcrw { c: 28, 4; d: 24, 4; op2: 21, 3; width: 16, 5; const4: 12, 4; a: 8, 4 });
format32!(/// RLC: `c [31:28], const16 [27:12], a [11:8]`.
    Rlc { c: 28, 4; const16: 12, 16; a: 8, 4 });
format32!(/// RR: `c [31:28], op2 [27:20], n [17:16], b [15:12], a [11:8]`.
    Rr { c: 28, 4; op2: 20, 8; n: 16, 2; b: 12, 4; a: 8, 4 });
format32!(/// RR1: `c [31:28], op2 [27:18], n [17:16], b [15:12], a [11:8]`.
    Rr1 { c: 28, 4; op2: 18, 10; n: 16, 2; b: 12, 4; a: 8, 4 });
format32!(/// RR2: `c [31:28], op2 [27:16], b [15:12], a [11:8]`.
    Rr2 { c: 28, 4; op2: 16, 12; b: 12, 4; a: 8, 4 });
format32!(/// RRPW: `c [31:28], pos [27:23], op2 [22:21], width [20:16], b [15:12], a [11:8]`.
    Rrpw { c: 28, 4; pos: 23, 5; op2: 21, 2; width: 16, 5; b: 12, 4; a: 8, 4 });
format32!(/// RRR: `c [31:28], d [27:24], op2 [23:20], n [17:16], b [15:12], a [11:8]`.
    Rrr { c: 28, 4; d: 24, 4; op2: 20, 4; n: 16, 2; b: 12, 4; a: 8, 4 });
format32!(/// RRR1: `c [31:28], d [27:24], op2 [23:18], n [17:16], b [15:12], a [11:8]`.
    Rrr1 { c: 28, 4; d: 24, 4; op2: 18, 6; n: 16, 2; b: 12, 4; a: 8, 4 });
format32!(/// RRR2: `c [31:28], d [27:24], op2 [23:16], b [15:12], a [11:8]`.
    Rrr2 { c: 28, 4; d: 24, 4; op2: 16, 8; b: 12, 4; a: 8, 4 });
format32!(/// RRRR: `c [31:28], d [27:24], op2 [23:21], b [15:12], a [11:8]`.
    Rrrr { c: 28, 4; d: 24, 4; op2: 21, 3; b: 12, 4; a: 8, 4 });
format32!(/// RRRW: `c [31:28], d [27:24], op2 [23:21], width [20:16], b [15:12], a [11:8]`.
    Rrrw { c: 28, 4; d: 24, 4; op2: 21, 3; width: 16, 5; b: 12, 4; a: 8, 4 });
format32!(/// SYS: `op2 [27:22], a [11:8]` (s1/d).
    Sys { op2: 22, 6; a: 8, 4 });

// ---- 16-bit formats (V2 Table 1-5, p. 1-3) -------------------------------------------------------

format16!(/// SB: `disp8 [15:8]`.
    Sb { disp8: 8, 8 });
format16!(/// SBC: `const4 [15:12], disp4 [11:8]`.
    Sbc { const4: 12, 4; disp4: 8, 4 });
format16!(/// SBR: `b [15:12]` (s2), `disp4 [11:8]`.
    Sbr { b: 12, 4; disp4: 8, 4 });
format16!(/// SBRN: `n [15:12], disp4 [11:8]`.
    Sbrn { n: 12, 4; disp4: 8, 4 });
format16!(/// SC: `const8 [15:8]`.
    Sc { const8: 8, 8 });
format16!(/// SLR: `b [15:12]` (s2), `c [11:8]` (d).
    Slr { b: 12, 4; c: 8, 4 });
format16!(/// SLRO: `off4 [15:12], c [11:8]` (d).
    Slro { off4: 12, 4; c: 8, 4 });
format16!(/// SR: `op2 [15:12], a [11:8]` (s1/d).
    Sr { op2: 12, 4; a: 8, 4 });
format16!(/// SRC: `const4 [15:12], a [11:8]` (s1/d).
    Src { const4: 12, 4; a: 8, 4 });
format16!(/// SRO: `b [15:12]` (s2), `off4 [11:8]`.
    Sro { b: 12, 4; off4: 8, 4 });
format16!(/// SRR: `b [15:12]` (s2), `a [11:8]` (s1/d).
    Srr { b: 12, 4; a: 8, 4 });
format16!(/// SSR: `b [15:12]` (s2), `a [11:8]` (s1).
    Ssr { b: 12, 4; a: 8, 4 });
format16!(/// SSRO: `off4 [15:12], a [11:8]` (s1).
    Ssro { off4: 12, 4; a: 8, 4 });

/// SRRS: `b [15:12]` (s2), `a [11:8]` (s1/d), `n [7:6]`; `op1` is bits `[5:0]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Srrs {
    pub op1: u8,
    pub b: u32,
    pub a: u32,
    pub n: u32,
}
impl Srrs {
    #[inline(always)]
    pub const fn from(h: u16) -> Self {
        let w = h as u32;
        Self { op1: (w & 0x3F) as u8, b: f(w, 12, 4), a: f(w, 8, 4), n: f(w, 6, 2) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manual_examples() {
        // ABS d3, d1 — RR: c=3, op2=1C, b=1, op1=0B (V2 p. 3-2)
        let r = Rr::from(0x31C0_100B);
        assert_eq!((r.c, r.op2, r.n, r.b, r.a, r.op1), (3, 0x1C, 0, 1, 0, 0x0B));
        // imask e2, #1, d0, #1 (isa_fix.rs firmware example): RCRW c=2, d=0, op2=1, width=1, const4=1
        let r = Rcrw::from(0x2021_10D7);
        assert_eq!((r.c, r.d, r.op2, r.width, r.const4, r.op1), (2, 0, 1, 1, 1, 0xD7));
        // jnz.t d15, #31 encodes n[4] in bit 7 and op1 in [6:0]
        let b = Brn::from(0x8000_F0EF);
        assert_eq!((b.op1, b.op2, b.n, b.a), (0x6F, 1, 31, 0));
        // movh.a a15, #0x8006 (91 60 00 f8): RLC c=15, const16=0x8006
        let r = Rlc::from(0xF800_6091);
        assert_eq!((r.c, r.const16), (15, 0x8006));
        // lea a15, [a15]#-0x1450 (d9 ff b0 ee): BOL off16 = 0xEBB0
        let b = Bol::from(0xEEB0_FFD9);
        assert_eq!((b.a, b.b, sext(b.off16, 16) as i32), (15, 15, -0x1450));
        // st.w #0xf0000c10, d13 (a5 fd d0 00): ABS EA 0xF0000C10
        assert_eq!(Abs::from(0x00D0_FDA5).ea(), 0xF000_0C10);
        // addsc.a: SRRS n in [7:6]
        let s = Srrs::from(0x21D0);
        assert_eq!((s.op1, s.n, s.a, s.b), (0x10, 3, 1, 2));
    }
}
