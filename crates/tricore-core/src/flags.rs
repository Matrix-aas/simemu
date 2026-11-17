//! PSW user-status-bit helpers and the RTL functions of V2 Table 1-9 (p. 1-8).
//!
//! Status bit semantics: V2 §1.5 / §2.3 (p. 2-8) and the "Status Flags" block of each instruction
//! page. The standard 32-bit arithmetic update (e.g. ADD, V2 p. 3-12) is
//!
//! ```text
//! overflow = (result > 7FFFFFFFH) OR (result < -80000000H);   PSW.V = overflow;  PSW.SV |= overflow
//! advanced_overflow = result[31] ^ result[30];                 PSW.AV = adv;      PSW.SAV |= adv
//! ```

use crate::arch::psw;

/// Set V/SV and AV/SAV from the two conditions (V, AV overwritten; SV, SAV sticky).
#[inline(always)]
pub fn set_v_av(p: &mut u32, overflow: bool, advanced_overflow: bool) {
    let mut v = *p & !(psw::V | psw::AV);
    if overflow {
        v |= psw::V | psw::SV;
    }
    if advanced_overflow {
        v |= psw::AV | psw::SAV;
    }
    *p = v;
}

/// Standard flag update for a 32-bit result computed with infinite precision; returns the
/// architectural (wrapped) 32-bit result.
#[inline(always)]
pub fn ov32(p: &mut u32, result: i64) -> u32 {
    let r = result as u32;
    set_v_av(p, result != r as i32 as i64, (r ^ (r << 1)) & 0x8000_0000 != 0);
    r
}

/// `ssov(x, y)`: saturate `x` to a signed `y`-bit range (V2 Table 1-9).
#[inline(always)]
pub fn ssov(x: i64, y: u32) -> i64 {
    let max_pos = (1i64 << (y - 1)) - 1;
    let max_neg = -(1i64 << (y - 1));
    x.clamp(max_neg, max_pos)
}

/// `suov(x, y)`: saturate `x` to an unsigned `y`-bit range (V2 Table 1-9).
#[inline(always)]
pub fn suov(x: i64, y: u32) -> i64 {
    x.clamp(0, (1i64 << y) - 1)
}

/// `carry(a, b, c)`: bit 32 of the unsigned sum (V2 Table 1-9).
#[inline(always)]
pub fn carry(a: u32, b: u32, c: u32) -> bool {
    (a as u64 + b as u64 + c as u64) >> 32 != 0
}

/// `round16(x)` = `{(x + 32'h00008000)[31:16], 16'h0000}` (V2 Table 1-9).
#[inline(always)]
pub fn round16(x: u32) -> u32 {
    x.wrapping_add(0x8000) & 0xFFFF_0000
}

/// `reverse16(n)`: bit-reverse the low 16 bits (V2 Table 1-9).
#[inline(always)]
pub fn reverse16(n: u32) -> u32 {
    (n as u16).reverse_bits() as u32
}

/// `leading_signs(x)`: number of leading bits equal to bit 31, the sign bit included (V2 CLS p. 3-70:
/// `result = leading_signs(D[a]) - 1`).
#[inline(always)]
pub fn leading_signs(x: u32) -> u32 {
    (if (x as i32) < 0 { !x } else { x }).leading_zeros()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_flag_semantics() {
        let mut p = 0;
        assert_eq!(ov32(&mut p, 0x7FFF_FFFF + 1), 0x8000_0000);
        assert_eq!(p & (psw::V | psw::SV | psw::AV | psw::SAV), psw::V | psw::SV | psw::AV | psw::SAV);
        // V and AV are overwritten, SV and SAV stay set
        assert_eq!(ov32(&mut p, 1), 1);
        assert_eq!(p & (psw::V | psw::SV | psw::AV | psw::SAV), psw::SV | psw::SAV);
        assert_eq!(ov32(&mut p, -0x8000_0000), 0x8000_0000);
        assert_eq!(p & psw::V, 0, "-2^31 is representable");
    }

    #[test]
    fn saturation() {
        assert_eq!(ssov(0x1_0000_0000, 32), 0x7FFF_FFFF);
        assert_eq!(ssov(-0x9000, 16), -0x8000);
        assert_eq!(suov(-1, 8), 0);
        assert_eq!(suov(300, 8), 255);
        assert_eq!(leading_signs(0), 32);
        assert_eq!(leading_signs(u32::MAX), 32);
        assert_eq!(leading_signs(0x4000_0000), 1);
    }
}
