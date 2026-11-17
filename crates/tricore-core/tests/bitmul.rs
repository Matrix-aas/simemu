//! Group B (`groups/bitmul.rs`) instruction tests.
//!
//! * `table_matches_decoder`: the 270 encodings of the V2 encoding boxes (table below, generated from
//!   `docs/tricore_core/isa_tc16_encodings.json` plus the MULMS.H UU box the extractor missed) are
//!   exactly the encodings the decoder accepts, for TC1.6 and TC1.3.1 (no MULMS.H).
//! * `every_encoding_matches_manual_rtl`: random instances of every encoding with random and edge
//!   operands against [`reference`], a literal transcription of the RTL and status-flag text of each
//!   page, keyed by mnemonic and operand syntax (independent of the decoder's tables).
//! * `unicorn_differential`: the same instances against bare Unicorn 2.1.4 (persistent engine,
//!   single-stepped), skipping the encodings `simemu_tc1791::isa_fix` knows Unicorn mistranslates.
//!   Disagreements must be listed in [`UNICORN_WRONG`] with the manual reason.
//! * `firmware_words_decode_as_rizin_names`: group-B words of both firmware images decode, and to the
//!   mnemonic rizin/capstone gives them.
//!
//! Only defined behaviour is compared: n in {0, 1}, EXTR width > 0, pos + width <= 32, DEXTR D[d] <= 31.
//!
//! ```text
//! cargo test -p tricore-core --release --test bitmul -- --nocapture
//! BITMUL_TRACE=1   print each instance before it runs on Unicorn (finds the word when Unicorn aborts)
//! ```

use std::collections::BTreeMap;
use std::ffi::c_int;
use tricore_core::groups::bitmul;
use tricore_core::{Arch, Cpu, Exit, Memory, NullBus};
use unicorn_tricore_sys::*;

/// One encoding box of V2: mnemonic, operand syntax, format, op1, op2, V2 page.
struct E {
    mn: &'static str,
    syn: &'static str,
    fmt: &'static str,
    op1: u8,
    op2: u16,
    page: &'static str,
}

#[rustfmt::skip]
const ENC: &[E] = &[
    E { mn: "AND.AND.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x47, op2: 0x00, page: "3-32" },
    E { mn: "AND.ANDN.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x47, op2: 0x03, page: "3-32" },
    E { mn: "AND.NOR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x47, op2: 0x02, page: "3-32" },
    E { mn: "AND.OR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x47, op2: 0x01, page: "3-32" },
    E { mn: "AND.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x87, op2: 0x00, page: "3-40" },
    E { mn: "ANDN.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x87, op2: 0x03, page: "3-42" },
    E { mn: "BMERGE", syn: "D[c], D[a], D[b]", fmt: "RR", op1: 0x4B, op2: 0x01, page: "3-45" },
    E { mn: "BSPLIT", syn: "E[c], D[a]", fmt: "RR", op1: 0x4B, op2: 0x09, page: "3-46" },
    E { mn: "CLO", syn: "D[c], D[a]", fmt: "RR", op1: 0x0F, op2: 0x1C, page: "3-68" },
    E { mn: "CLO.H", syn: "D[c], D[a]", fmt: "RR", op1: 0x0F, op2: 0x7D, page: "3-69" },
    E { mn: "CLS", syn: "D[c], D[a]", fmt: "RR", op1: 0x0F, op2: 0x1D, page: "3-70" },
    E { mn: "CLS.H", syn: "D[c], D[a]", fmt: "RR", op1: 0x0F, op2: 0x7E, page: "3-71" },
    E { mn: "CLZ", syn: "D[c], D[a]", fmt: "RR", op1: 0x0F, op2: 0x1B, page: "3-72" },
    E { mn: "CLZ.H", syn: "D[c], D[a]", fmt: "RR", op1: 0x0F, op2: 0x7C, page: "3-73" },
    E { mn: "DEXTR", syn: "D[c], D[a], D[b], pos", fmt: "RRPW", op1: 0x77, op2: 0x00, page: "3-79" },
    E { mn: "DEXTR", syn: "D[c], D[a], D[b], D[d]", fmt: "RRRR", op1: 0x17, op2: 0x04, page: "3-79" },
    E { mn: "EXTR", syn: "D[c], D[a], pos, width", fmt: "RRPW", op1: 0x37, op2: 0x02, page: "3-100" },
    E { mn: "EXTR", syn: "D[c], D[a], E[d]", fmt: "RRRR", op1: 0x17, op2: 0x02, page: "3-100" },
    E { mn: "EXTR", syn: "D[c], D[a], D[d], width", fmt: "RRRW", op1: 0x57, op2: 0x02, page: "3-100" },
    E { mn: "EXTR.U", syn: "D[c], D[a], pos, width", fmt: "RRPW", op1: 0x37, op2: 0x03, page: "3-100" },
    E { mn: "EXTR.U", syn: "D[c], D[a], E[d]", fmt: "RRRR", op1: 0x17, op2: 0x03, page: "3-100" },
    E { mn: "EXTR.U", syn: "D[c], D[a], D[d], width", fmt: "RRRW", op1: 0x57, op2: 0x03, page: "3-101" },
    E { mn: "IMASK", syn: "E[c], const4, pos, width", fmt: "RCPW", op1: 0xB7, op2: 0x01, page: "3-109" },
    E { mn: "IMASK", syn: "E[c], const4, D[d], width", fmt: "RCRW", op1: 0xD7, op2: 0x01, page: "3-109" },
    E { mn: "IMASK", syn: "E[c], D[b], pos, width", fmt: "RRPW", op1: 0x37, op2: 0x01, page: "3-109" },
    E { mn: "IMASK", syn: "E[c], D[b], D[d], width", fmt: "RRRW", op1: 0x57, op2: 0x01, page: "3-109" },
    E { mn: "INS.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x67, op2: 0x00, page: "3-111" },
    E { mn: "INSN.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x67, op2: 0x01, page: "3-111" },
    E { mn: "INSERT", syn: "D[c], D[a], const4, pos, width", fmt: "RCPW", op1: 0xB7, op2: 0x00, page: "3-112" },
    E { mn: "INSERT", syn: "D[c], D[a], const4, E[d]", fmt: "RCRR", op1: 0x97, op2: 0x00, page: "3-112" },
    E { mn: "INSERT", syn: "D[c], D[a], const4, D[d], width", fmt: "RCRW", op1: 0xD7, op2: 0x00, page: "3-112" },
    E { mn: "INSERT", syn: "D[c], D[a], D[b], pos, width", fmt: "RRPW", op1: 0x37, op2: 0x00, page: "3-112" },
    E { mn: "INSERT", syn: "D[c], D[a], D[b], E[d]", fmt: "RRRR", op1: 0x17, op2: 0x00, page: "3-113" },
    E { mn: "INSERT", syn: "D[c], D[a], D[b], D[d], width", fmt: "RRRW", op1: 0x57, op2: 0x00, page: "3-113" },
    E { mn: "IXMAX", syn: "E[c], E[d], D[b]", fmt: "RRR", op1: 0x6B, op2: 0x0A, page: "3-115" },
    E { mn: "IXMAX.U", syn: "E[c], E[d], D[b]", fmt: "RRR", op1: 0x6B, op2: 0x0B, page: "3-115" },
    E { mn: "IXMIN", syn: "E[c], E[d], D[b]", fmt: "RRR", op1: 0x6B, op2: 0x08, page: "3-117" },
    E { mn: "IXMIN.U", syn: "E[c], E[d], D[b]", fmt: "RRR", op1: 0x6B, op2: 0x09, page: "3-117" },
    E { mn: "MADD", syn: "D[c], D[d], D[a], const9", fmt: "RCR", op1: 0x13, op2: 0x01, page: "3-180" },
    E { mn: "MADD", syn: "E[c], E[d], D[a], const9", fmt: "RCR", op1: 0x13, op2: 0x03, page: "3-180" },
    E { mn: "MADD", syn: "D[c], D[d], D[a], D[b]", fmt: "RRR2", op1: 0x03, op2: 0x0A, page: "3-180" },
    E { mn: "MADD", syn: "E[c], E[d], D[a], D[b]", fmt: "RRR2", op1: 0x03, op2: 0x6A, page: "3-180" },
    E { mn: "MADDS", syn: "D[c], D[d], D[a], const9", fmt: "RCR", op1: 0x13, op2: 0x05, page: "3-180" },
    E { mn: "MADDS", syn: "E[c], E[d], D[a], const9", fmt: "RCR", op1: 0x13, op2: 0x07, page: "3-181" },
    E { mn: "MADDS", syn: "D[c], D[d], D[a], D[b]", fmt: "RRR2", op1: 0x03, op2: 0x8A, page: "3-181" },
    E { mn: "MADDS", syn: "E[c], E[d], D[a], D[b]", fmt: "RRR2", op1: 0x03, op2: 0xEA, page: "3-181" },
    E { mn: "MADD.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0x83, op2: 0x1A, page: "3-183" },
    E { mn: "MADD.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0x83, op2: 0x19, page: "3-183" },
    E { mn: "MADD.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x83, op2: 0x18, page: "3-183" },
    E { mn: "MADD.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0x83, op2: 0x1B, page: "3-184" },
    E { mn: "MADDS.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0x83, op2: 0x3A, page: "3-184" },
    E { mn: "MADDS.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0x83, op2: 0x39, page: "3-184" },
    E { mn: "MADDS.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x83, op2: 0x38, page: "3-185" },
    E { mn: "MADDS.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0x83, op2: 0x3B, page: "3-185" },
    E { mn: "MADD.Q", syn: "D[c], D[d], D[a], D[b], n", fmt: "RRR1", op1: 0x43, op2: 0x02, page: "3-187" },
    E { mn: "MADD.Q", syn: "E[c], E[d], D[a], D[b], n", fmt: "RRR1", op1: 0x43, op2: 0x1B, page: "3-187" },
    E { mn: "MADD.Q", syn: "D[c], D[d], D[a], D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x01, page: "3-187" },
    E { mn: "MADD.Q", syn: "E[c], E[d], D[a], D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x19, page: "3-187" },
    E { mn: "MADD.Q", syn: "D[c], D[d], D[a], D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x00, page: "3-187" },
    E { mn: "MADD.Q", syn: "E[c], E[d], D[a], D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x18, page: "3-188" },
    E { mn: "MADD.Q", syn: "D[c], D[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x05, page: "3-188" },
    E { mn: "MADD.Q", syn: "E[c], E[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x1D, page: "3-188" },
    E { mn: "MADD.Q", syn: "D[c], D[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x04, page: "3-188" },
    E { mn: "MADD.Q", syn: "E[c], E[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x1C, page: "3-189" },
    E { mn: "MADDS.Q", syn: "D[c], D[d], D[a], D[b], n", fmt: "RRR1", op1: 0x43, op2: 0x22, page: "3-189" },
    E { mn: "MADDS.Q", syn: "E[c], E[d], D[a], D[b], n", fmt: "RRR1", op1: 0x43, op2: 0x3B, page: "3-189" },
    E { mn: "MADDS.Q", syn: "D[c], D[d], D[a], D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x21, page: "3-189" },
    E { mn: "MADDS.Q", syn: "E[c], E[d], D[a], D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x39, page: "3-189" },
    E { mn: "MADDS.Q", syn: "D[c], D[d], D[a], D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x20, page: "3-189" },
    E { mn: "MADDS.Q", syn: "E[c], E[d], D[a], D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x38, page: "3-190" },
    E { mn: "MADDS.Q", syn: "D[c], D[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x25, page: "3-190" },
    E { mn: "MADDS.Q", syn: "E[c], E[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x3D, page: "3-190" },
    E { mn: "MADDS.Q", syn: "D[c], D[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x24, page: "3-190" },
    E { mn: "MADDS.Q", syn: "E[c], E[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x3C, page: "3-190" },
    E { mn: "MADD.U", syn: "E[c], E[d], D[a], const9", fmt: "RCR", op1: 0x13, op2: 0x02, page: "3-193" },
    E { mn: "MADD.U", syn: "E[c], E[d], D[a], D[b]", fmt: "RRR2", op1: 0x03, op2: 0x68, page: "3-193" },
    E { mn: "MADDS.U", syn: "D[c], D[d], D[a], const9", fmt: "RCR", op1: 0x13, op2: 0x04, page: "3-193" },
    E { mn: "MADDS.U", syn: "E[c], E[d], D[a], const9", fmt: "RCR", op1: 0x13, op2: 0x06, page: "3-193" },
    E { mn: "MADDS.U", syn: "D[c], D[d], D[a], D[b]", fmt: "RRR2", op1: 0x03, op2: 0x88, page: "3-193" },
    E { mn: "MADDS.U", syn: "E[c], E[d], D[a], D[b]", fmt: "RRR2", op1: 0x03, op2: 0xE8, page: "3-194" },
    E { mn: "MADDM.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0x83, op2: 0x1E, page: "3-195" },
    E { mn: "MADDM.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0x83, op2: 0x1D, page: "3-195" },
    E { mn: "MADDM.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x83, op2: 0x1C, page: "3-195" },
    E { mn: "MADDM.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0x83, op2: 0x1F, page: "3-196" },
    E { mn: "MADDMS.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0x83, op2: 0x3E, page: "3-196" },
    E { mn: "MADDMS.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0x83, op2: 0x3D, page: "3-196" },
    E { mn: "MADDMS.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x83, op2: 0x3C, page: "3-196" },
    E { mn: "MADDMS.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0x83, op2: 0x3F, page: "3-197" },
    E { mn: "MADDR.H", syn: "D[c], D[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0x83, op2: 0x0E, page: "3-198" },
    E { mn: "MADDR.H", syn: "D[c], D[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0x83, op2: 0x0D, page: "3-198" },
    E { mn: "MADDR.H", syn: "D[c], D[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x83, op2: 0x0C, page: "3-198" },
    E { mn: "MADDR.H", syn: "D[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x43, op2: 0x1E, page: "3-199" },
    E { mn: "MADDR.H", syn: "D[c], D[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0x83, op2: 0x0F, page: "3-199" },
    E { mn: "MADDRS.H", syn: "D[c], D[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0x83, op2: 0x2E, page: "3-199" },
    E { mn: "MADDRS.H", syn: "D[c], D[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0x83, op2: 0x2D, page: "3-200" },
    E { mn: "MADDRS.H", syn: "D[c], D[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x83, op2: 0x2C, page: "3-200" },
    E { mn: "MADDRS.H", syn: "D[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x43, op2: 0x3E, page: "3-200" },
    E { mn: "MADDRS.H", syn: "D[c], D[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0x83, op2: 0x2F, page: "3-201" },
    E { mn: "MADDR.Q", syn: "D[c], D[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x07, page: "3-202" },
    E { mn: "MADDR.Q", syn: "D[c], D[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x06, page: "3-202" },
    E { mn: "MADDRS.Q", syn: "D[c], D[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x43, op2: 0x27, page: "3-202" },
    E { mn: "MADDRS.Q", syn: "D[c], D[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x43, op2: 0x26, page: "3-202" },
    E { mn: "MADDSU.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xC3, op2: 0x1A, page: "3-204" },
    E { mn: "MADDSU.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xC3, op2: 0x19, page: "3-204" },
    E { mn: "MADDSU.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xC3, op2: 0x18, page: "3-204" },
    E { mn: "MADDSU.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xC3, op2: 0x1B, page: "3-205" },
    E { mn: "MADDSUS.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xC3, op2: 0x3A, page: "3-205" },
    E { mn: "MADDSUS.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xC3, op2: 0x39, page: "3-205" },
    E { mn: "MADDSUS.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xC3, op2: 0x38, page: "3-206" },
    E { mn: "MADDSUS.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xC3, op2: 0x3B, page: "3-206" },
    E { mn: "MADDSUM.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xC3, op2: 0x1E, page: "3-208" },
    E { mn: "MADDSUM.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xC3, op2: 0x1D, page: "3-208" },
    E { mn: "MADDSUM.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xC3, op2: 0x1C, page: "3-208" },
    E { mn: "MADDSUM.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xC3, op2: 0x1F, page: "3-209" },
    E { mn: "MADDSUMS.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xC3, op2: 0x3E, page: "3-209" },
    E { mn: "MADDSUMS.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xC3, op2: 0x3D, page: "3-209" },
    E { mn: "MADDSUMS.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xC3, op2: 0x3C, page: "3-210" },
    E { mn: "MADDSUMS.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xC3, op2: 0x3F, page: "3-210" },
    E { mn: "MADDSUR.H", syn: "D[c], D[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xC3, op2: 0x0E, page: "3-211" },
    E { mn: "MADDSUR.H", syn: "D[c], D[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xC3, op2: 0x0D, page: "3-211" },
    E { mn: "MADDSUR.H", syn: "D[c], D[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xC3, op2: 0x0C, page: "3-211" },
    E { mn: "MADDSUR.H", syn: "D[c], D[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xC3, op2: 0x0F, page: "3-212" },
    E { mn: "MADDSURS.H", syn: "D[c], D[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xC3, op2: 0x2E, page: "3-212" },
    E { mn: "MADDSURS.H", syn: "D[c], D[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xC3, op2: 0x2D, page: "3-212" },
    E { mn: "MADDSURS.H", syn: "D[c], D[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xC3, op2: 0x2C, page: "3-213" },
    E { mn: "MADDSURS.H", syn: "D[c], D[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xC3, op2: 0x2F, page: "3-213" },
    E { mn: "MSUB", syn: "D[c], D[d], D[a], const9", fmt: "RCR", op1: 0x33, op2: 0x01, page: "3-233" },
    E { mn: "MSUB", syn: "E[c], E[d], D[a], const9", fmt: "RCR", op1: 0x33, op2: 0x03, page: "3-233" },
    E { mn: "MSUB", syn: "D[c], D[d], D[a], D[b]", fmt: "RRR2", op1: 0x23, op2: 0x0A, page: "3-233" },
    E { mn: "MSUB", syn: "E[c], E[d], D[a], D[b]", fmt: "RRR2", op1: 0x23, op2: 0x6A, page: "3-233" },
    E { mn: "MSUBS", syn: "D[c], D[d], D[a], const9", fmt: "RCR", op1: 0x33, op2: 0x05, page: "3-233" },
    E { mn: "MSUBS", syn: "E[c], E[d], D[a], const9", fmt: "RCR", op1: 0x33, op2: 0x07, page: "3-234" },
    E { mn: "MSUBS", syn: "D[c], D[d], D[a], D[b]", fmt: "RRR2", op1: 0x23, op2: 0x8A, page: "3-234" },
    E { mn: "MSUBS", syn: "E[c], E[d], D[a], D[b]", fmt: "RRR2", op1: 0x23, op2: 0xEA, page: "3-234" },
    E { mn: "MSUB.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xA3, op2: 0x1A, page: "3-236" },
    E { mn: "MSUB.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xA3, op2: 0x19, page: "3-236" },
    E { mn: "MSUB.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xA3, op2: 0x18, page: "3-236" },
    E { mn: "MSUB.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xA3, op2: 0x1B, page: "3-237" },
    E { mn: "MSUBS.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xA3, op2: 0x3A, page: "3-237" },
    E { mn: "MSUBS.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xA3, op2: 0x39, page: "3-237" },
    E { mn: "MSUBS.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xA3, op2: 0x38, page: "3-238" },
    E { mn: "MSUBS.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xA3, op2: 0x3B, page: "3-238" },
    E { mn: "MSUB.Q", syn: "D[c], D[d], D[a], D[b], n", fmt: "RRR1", op1: 0x63, op2: 0x02, page: "3-240" },
    E { mn: "MSUB.Q", syn: "E[c], E[d], D[a], D[b], n", fmt: "RRR1", op1: 0x63, op2: 0x1B, page: "3-240" },
    E { mn: "MSUB.Q", syn: "D[c], D[d], D[a], D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x01, page: "3-240" },
    E { mn: "MSUB.Q", syn: "E[c], E[d], D[a], D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x19, page: "3-240" },
    E { mn: "MSUB.Q", syn: "D[c], D[d], D[a], D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x00, page: "3-241" },
    E { mn: "MSUB.Q", syn: "E[c], E[d], D[a], D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x18, page: "3-241" },
    E { mn: "MSUB.Q", syn: "D[c], D[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x05, page: "3-241" },
    E { mn: "MSUB.Q", syn: "E[c], E[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x1D, page: "3-241" },
    E { mn: "MSUB.Q", syn: "D[c], D[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x04, page: "3-241" },
    E { mn: "MSUB.Q", syn: "E[c], E[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x1C, page: "3-242" },
    E { mn: "MSUBS.Q", syn: "D[c], D[d], D[a], D[b], n", fmt: "RRR1", op1: 0x63, op2: 0x22, page: "3-242" },
    E { mn: "MSUBS.Q", syn: "E[c], E[d], D[a], D[b], n", fmt: "RRR1", op1: 0x63, op2: 0x3B, page: "3-242" },
    E { mn: "MSUBS.Q", syn: "D[c], D[d], D[a], D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x21, page: "3-242" },
    E { mn: "MSUBS.Q", syn: "E[c], E[d], D[a], D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x39, page: "3-242" },
    E { mn: "MSUBS.Q", syn: "D[c], D[d], D[a], D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x20, page: "3-242" },
    E { mn: "MSUBS.Q", syn: "E[c], E[d], D[a], D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x38, page: "3-243" },
    E { mn: "MSUBS.Q", syn: "D[c], D[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x25, page: "3-243" },
    E { mn: "MSUBS.Q", syn: "E[c], E[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x3D, page: "3-243" },
    E { mn: "MSUBS.Q", syn: "D[c], D[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x24, page: "3-243" },
    E { mn: "MSUBS.Q", syn: "E[c], E[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x3C, page: "3-244" },
    E { mn: "MSUB.U", syn: "E[c], E[d], D[a], const9", fmt: "RCR", op1: 0x33, op2: 0x02, page: "3-246" },
    E { mn: "MSUB.U", syn: "E[c], E[d], D[a], D[b]", fmt: "RRR2", op1: 0x23, op2: 0x68, page: "3-246" },
    E { mn: "MSUBS.U", syn: "D[c], D[d], D[a], const9", fmt: "RCR", op1: 0x33, op2: 0x04, page: "3-246" },
    E { mn: "MSUBS.U", syn: "E[c], E[d], D[a], const9", fmt: "RCR", op1: 0x33, op2: 0x06, page: "3-246" },
    E { mn: "MSUBS.U", syn: "D[c], D[d], D[a], D[b]", fmt: "RRR2", op1: 0x23, op2: 0x88, page: "3-246" },
    E { mn: "MSUBS.U", syn: "E[c], E[d], D[a], D[b]", fmt: "RRR2", op1: 0x23, op2: 0xE8, page: "3-247" },
    E { mn: "MSUBAD.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xE3, op2: 0x1A, page: "3-248" },
    E { mn: "MSUBAD.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xE3, op2: 0x19, page: "3-248" },
    E { mn: "MSUBAD.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xE3, op2: 0x18, page: "3-248" },
    E { mn: "MSUBAD.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xE3, op2: 0x1B, page: "3-249" },
    E { mn: "MSUBADS.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xE3, op2: 0x3A, page: "3-249" },
    E { mn: "MSUBADS.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xE3, op2: 0x39, page: "3-249" },
    E { mn: "MSUBADS.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xE3, op2: 0x38, page: "3-250" },
    E { mn: "MSUBADS.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xE3, op2: 0x3B, page: "3-250" },
    E { mn: "MSUBADM.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xE3, op2: 0x1E, page: "3-252" },
    E { mn: "MSUBADM.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xE3, op2: 0x1D, page: "3-252" },
    E { mn: "MSUBADM.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xE3, op2: 0x1C, page: "3-252" },
    E { mn: "MSUBADM.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xE3, op2: 0x1F, page: "3-253" },
    E { mn: "MSUBADMS.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xE3, op2: 0x3E, page: "3-253" },
    E { mn: "MSUBADMS.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xE3, op2: 0x3D, page: "3-253" },
    E { mn: "MSUBADMS.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xE3, op2: 0x3C, page: "3-254" },
    E { mn: "MSUBADMS.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xE3, op2: 0x3F, page: "3-254" },
    E { mn: "MSUBADR.H", syn: "D[c], D[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xE3, op2: 0x0E, page: "3-255" },
    E { mn: "MSUBADR.H", syn: "D[c], D[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xE3, op2: 0x0D, page: "3-255" },
    E { mn: "MSUBADR.H", syn: "D[c], D[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xE3, op2: 0x0C, page: "3-255" },
    E { mn: "MSUBADR.H", syn: "D[c], D[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xE3, op2: 0x0F, page: "3-256" },
    E { mn: "MSUBADRS.H", syn: "D[c], D[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xE3, op2: 0x2E, page: "3-256" },
    E { mn: "MSUBADRS.H", syn: "D[c], D[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xE3, op2: 0x2D, page: "3-256" },
    E { mn: "MSUBADRS.H", syn: "D[c], D[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xE3, op2: 0x2C, page: "3-257" },
    E { mn: "MSUBADRS.H", syn: "D[c], D[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xE3, op2: 0x2F, page: "3-257" },
    E { mn: "MSUBM.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xA3, op2: 0x1E, page: "3-259" },
    E { mn: "MSUBM.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xA3, op2: 0x1D, page: "3-259" },
    E { mn: "MSUBM.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xA3, op2: 0x1C, page: "3-259" },
    E { mn: "MSUBM.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xA3, op2: 0x1F, page: "3-260" },
    E { mn: "MSUBMS.H", syn: "E[c], E[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xA3, op2: 0x3E, page: "3-260" },
    E { mn: "MSUBMS.H", syn: "E[c], E[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xA3, op2: 0x3D, page: "3-260" },
    E { mn: "MSUBMS.H", syn: "E[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xA3, op2: 0x3C, page: "3-261" },
    E { mn: "MSUBMS.H", syn: "E[c], E[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xA3, op2: 0x3F, page: "3-261" },
    E { mn: "MSUBR.H", syn: "D[c], D[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xA3, op2: 0x0E, page: "3-262" },
    E { mn: "MSUBR.H", syn: "D[c], D[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xA3, op2: 0x0D, page: "3-262" },
    E { mn: "MSUBR.H", syn: "D[c], D[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xA3, op2: 0x0C, page: "3-262" },
    E { mn: "MSUBR.H", syn: "D[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x63, op2: 0x1E, page: "3-263" },
    E { mn: "MSUBR.H", syn: "D[c], D[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xA3, op2: 0x0F, page: "3-263" },
    E { mn: "MSUBRS.H", syn: "D[c], D[d], D[a], D[b] LL, n", fmt: "RRR1", op1: 0xA3, op2: 0x2E, page: "3-263" },
    E { mn: "MSUBRS.H", syn: "D[c], D[d], D[a], D[b] LU, n", fmt: "RRR1", op1: 0xA3, op2: 0x2D, page: "3-264" },
    E { mn: "MSUBRS.H", syn: "D[c], D[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0xA3, op2: 0x2C, page: "3-264" },
    E { mn: "MSUBRS.H", syn: "D[c], E[d], D[a], D[b] UL, n", fmt: "RRR1", op1: 0x63, op2: 0x3E, page: "3-264" },
    E { mn: "MSUBRS.H", syn: "D[c], D[d], D[a], D[b] UU, n", fmt: "RRR1", op1: 0xA3, op2: 0x2F, page: "3-265" },
    E { mn: "MSUBR.Q", syn: "D[c], D[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x07, page: "3-266" },
    E { mn: "MSUBR.Q", syn: "D[c], D[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x06, page: "3-266" },
    E { mn: "MSUBRS.Q", syn: "D[c], D[d], D[a] L, D[b] L, n", fmt: "RRR1", op1: 0x63, op2: 0x27, page: "3-266" },
    E { mn: "MSUBRS.Q", syn: "D[c], D[d], D[a] U, D[b] U, n", fmt: "RRR1", op1: 0x63, op2: 0x26, page: "3-266" },
    E { mn: "MUL", syn: "D[c], D[a], const9", fmt: "RC", op1: 0x53, op2: 0x01, page: "3-269" },
    E { mn: "MUL", syn: "E[c], D[a], const9", fmt: "RC", op1: 0x53, op2: 0x03, page: "3-269" },
    E { mn: "MUL", syn: "D[c], D[a], D[b]", fmt: "RR2", op1: 0x73, op2: 0x0A, page: "3-269" },
    E { mn: "MUL", syn: "E[c], D[a], D[b]", fmt: "RR2", op1: 0x73, op2: 0x6A, page: "3-269" },
    E { mn: "MUL", syn: "D[a], D[b]", fmt: "SRR", op1: 0xE2, op2: 0x00, page: "3-269" },
    E { mn: "MULS", syn: "D[c], D[a], const9", fmt: "RC", op1: 0x53, op2: 0x05, page: "3-270" },
    E { mn: "MULS", syn: "D[c], D[a], D[b]", fmt: "RR2", op1: 0x73, op2: 0x8A, page: "3-270" },
    E { mn: "MUL.H", syn: "E[c], D[a], D[b] LL, n", fmt: "RR1", op1: 0xB3, op2: 0x1A, page: "3-272" },
    E { mn: "MUL.H", syn: "E[c], D[a], D[b] LU, n", fmt: "RR1", op1: 0xB3, op2: 0x19, page: "3-272" },
    E { mn: "MUL.H", syn: "E[c], D[a], D[b] UL, n", fmt: "RR1", op1: 0xB3, op2: 0x18, page: "3-272" },
    E { mn: "MUL.H", syn: "E[c], D[a], D[b] UU, n", fmt: "RR1", op1: 0xB3, op2: 0x1B, page: "3-273" },
    E { mn: "MUL.Q", syn: "D[c], D[a], D[b], n", fmt: "RR1", op1: 0x93, op2: 0x02, page: "3-274" },
    E { mn: "MUL.Q", syn: "E[c], D[a], D[b], n", fmt: "RR1", op1: 0x93, op2: 0x1B, page: "3-274" },
    E { mn: "MUL.Q", syn: "D[c], D[a], D[b] L, n", fmt: "RR1", op1: 0x93, op2: 0x01, page: "3-274" },
    E { mn: "MUL.Q", syn: "E[c], D[a], D[b] L, n", fmt: "RR1", op1: 0x93, op2: 0x19, page: "3-274" },
    E { mn: "MUL.Q", syn: "D[c], D[a], D[b] U, n", fmt: "RR1", op1: 0x93, op2: 0x00, page: "3-274" },
    E { mn: "MUL.Q", syn: "E[c], D[a], D[b] U, n", fmt: "RR1", op1: 0x93, op2: 0x18, page: "3-275" },
    E { mn: "MUL.Q", syn: "D[c], D[a] L, D[b] L, n", fmt: "RR1", op1: 0x93, op2: 0x05, page: "3-275" },
    E { mn: "MUL.Q", syn: "D[c], D[a] U, D[b] U, n", fmt: "RR1", op1: 0x93, op2: 0x04, page: "3-275" },
    E { mn: "MUL.U", syn: "E[c], D[a], const9", fmt: "RC", op1: 0x53, op2: 0x02, page: "3-277" },
    E { mn: "MUL.U", syn: "E[c], D[a], D[b]", fmt: "RR2", op1: 0x73, op2: 0x68, page: "3-277" },
    E { mn: "MULS.U", syn: "D[c], D[a], const9", fmt: "RC", op1: 0x53, op2: 0x04, page: "3-277" },
    E { mn: "MULS.U", syn: "D[c], D[a], D[b]", fmt: "RR2", op1: 0x73, op2: 0x88, page: "3-277" },
    E { mn: "MULM.H", syn: "E[c], D[a], D[b] LL, n", fmt: "RR1", op1: 0xB3, op2: 0x1E, page: "3-279" },
    E { mn: "MULM.H", syn: "E[c], D[a], D[b] LU, n", fmt: "RR1", op1: 0xB3, op2: 0x1D, page: "3-279" },
    E { mn: "MULM.H", syn: "E[c], D[a], D[b] UL, n", fmt: "RR1", op1: 0xB3, op2: 0x1C, page: "3-279" },
    E { mn: "MULM.H", syn: "E[c], D[a], D[b] UU, n", fmt: "RR1", op1: 0xB3, op2: 0x1F, page: "3-280" },
    E { mn: "MULMS.H", syn: "E[c], D[a], D[b] LL, n", fmt: "RR1", op1: 0xB3, op2: 0x3E, page: "3-281" },
    E { mn: "MULMS.H", syn: "E[c], D[a], D[b] LU, n", fmt: "RR1", op1: 0xB3, op2: 0x3D, page: "3-281" },
    E { mn: "MULMS.H", syn: "E[c], D[a], D[b] UL, n", fmt: "RR1", op1: 0xB3, op2: 0x3C, page: "3-281" },
    E { mn: "MULR.H", syn: "D[c], D[a], D[b] LL, n", fmt: "RR1", op1: 0xB3, op2: 0x0E, page: "3-282" },
    E { mn: "MULR.H", syn: "D[c], D[a], D[b] LU, n", fmt: "RR1", op1: 0xB3, op2: 0x0D, page: "3-282" },
    E { mn: "MULR.H", syn: "D[c], D[a], D[b] UL, n", fmt: "RR1", op1: 0xB3, op2: 0x0C, page: "3-282" },
    E { mn: "MULR.H", syn: "D[c], D[a], D[b] UU, n", fmt: "RR1", op1: 0xB3, op2: 0x0F, page: "3-283" },
    E { mn: "MULR.Q", syn: "D[c], D[a] L, D[b] L, n", fmt: "RR1", op1: 0x93, op2: 0x07, page: "3-284" },
    E { mn: "MULR.Q", syn: "D[c], D[a] U, D[b] U, n", fmt: "RR1", op1: 0x93, op2: 0x06, page: "3-284" },
    E { mn: "NAND.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x07, op2: 0x00, page: "3-286" },
    E { mn: "NOR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x87, op2: 0x02, page: "3-292" },
    E { mn: "OR.AND.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0xC7, op2: 0x00, page: "3-296" },
    E { mn: "OR.ANDN.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0xC7, op2: 0x03, page: "3-296" },
    E { mn: "OR.NOR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0xC7, op2: 0x02, page: "3-296" },
    E { mn: "OR.OR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0xC7, op2: 0x01, page: "3-296" },
    E { mn: "OR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x87, op2: 0x01, page: "3-304" },
    E { mn: "ORN.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x07, op2: 0x01, page: "3-306" },
    E { mn: "PARITY", syn: "D[c], D[a]", fmt: "RR", op1: 0x4B, op2: 0x02, page: "3-309" },
    E { mn: "SH.AND.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x27, op2: 0x00, page: "3-335" },
    E { mn: "SH.ANDN.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x27, op2: 0x03, page: "3-335" },
    E { mn: "SH.NAND.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0xA7, op2: 0x00, page: "3-335" },
    E { mn: "SH.NOR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x27, op2: 0x02, page: "3-335" },
    E { mn: "SH.OR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x27, op2: 0x01, page: "3-336" },
    E { mn: "SH.ORN.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0xA7, op2: 0x01, page: "3-336" },
    E { mn: "SH.XNOR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0xA7, op2: 0x02, page: "3-336" },
    E { mn: "SH.XOR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0xA7, op2: 0x03, page: "3-336" },
    E { mn: "XNOR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x07, op2: 0x02, page: "3-384" },
    E { mn: "XOR.T", syn: "D[c], D[a], pos1, D[b], pos2", fmt: "BIT", op1: 0x07, op2: 0x03, page: "3-392" },
    E { mn: "MULMS.H", syn: "E[c], D[a], D[b] UU, n", fmt: "RR1", op1: 0xB3, op2: 0x3F, page: "3-281" },
];

/// Encodings for which Unicorn 2.1.4 disagrees with the manual: (mnemonic, syntax prefix, reason).
/// Filled from the differential run; each entry was checked against the page RTL.
const UNICORN_WRONG: &[(&str, &str, &str)] = &[
    (
        "INSERT",
        "D[c], D[a], const4, E[d]",
        "RCRR 97H: Unicorn swaps the c and d fields (reads E[c], writes D[d]) and traps when c is odd \
         (the RCRW defect of KNOWN_WRONG, also present in RCRR); V2 p. 3-112",
    ),
    (
        "MSUB.Q",
        "D[c], D[d], D[a], D[b]",
        "32*32 / 16*32 -> 32 forms: Unicorn rounds the full-precision difference, (D[d] * 2^k - P) >> k; \
         the RTL is D[d] - (P >> k) (V2 p. 3-240, V131-2 p. 3-289, TC1.8 identical): 1 LSB apart \
         whenever the discarded low bits of P are non-zero",
    ),
    ("MSUBS.Q", "D[c], D[d], D[a], D[b]", "as MSUB.Q (V2 p. 3-242)"),
    (
        "MULS.U",
        "D[c], D[a], D[b]",
        "Unicorn 2.1.4 helper_muls_u uses a signed 64-bit overflow check on LP64, missing unsigned \
         products >= 2^63 and returning the low 32 bits with V = 0 instead of saturating to 0xFFFFFFFF \
         (V2 p. 3-284)",
    ),
    ("MULMS.H", "E[c]", "not implemented by Unicorn (trap); encodings listed in V2 p. 3-281 (TC1.6 only)"),
];

/// Group-B words from rizin 0.9.1 linear sweeps of the firmware images (`pD` over the SIMOS18.1
/// FACTORY flash 0x80000000..0x80200000 and the DQ200_G2 flash 0x80000000..0x802A0000; includes
/// data decoded as code): up to 3 per (image, mnemonic, op1, op2), with rizin's mnemonic.
#[rustfmt::skip]
const FIRMWARE: &[(&str, &str, u32)] = &[
    ("SIMOS", "MSUB", 0x31373033),
    ("SIMOS", "MSUB", 0x34373133),
    ("SIMOS", "MSUB", 0x38313433),
    ("SIMOS", "MSUB.U", 0x4A530033),
    ("SIMOS", "NAND.T", 0x00000007),
    ("SIMOS", "NAND.T", 0x09050007),
    ("SIMOS", "ORN.T", 0x00200007),
    ("SIMOS", "SH.AND.T", 0x00800027),
    ("SIMOS", "MADD.Q", 0x00000043),
    ("SIMOS", "AND.OR.T", 0x00200047),
    ("SIMOS", "IMASK", 0x00200057),
    ("SIMOS", "INS.T", 0x00800067),
    ("SIMOS", "IXMIN.U", 0x0090006B),
    ("SIMOS", "OR.T", 0x00200087),
    ("SIMOS", "MUL.Q", 0x00000093),
    ("SIMOS", "NAND.T", 0x00800207),
    ("SIMOS", "MUL.Q", 0x00000093),
    ("SIMOS", "MUL", 0xF020CF53),
    ("SIMOS", "INSERT", 0xF1810FB7),
    ("SIMOS", "MUL", 0xF020CF53),
    ("SIMOS", "EXTR.U", 0xF2E80437),
    ("SIMOS", "INSERT", 0x060110B7),
    ("SIMOS", "INSERT", 0x060100B7),
    ("SIMOS", "MUL", 0xF020C453),
    ("SIMOS", "EXTR.U", 0xC0680037),
    ("SIMOS", "EXTR.U", 0xD0680037),
    ("SIMOS", "CLZ", 0x31B0000F),
    ("SIMOS", "EXTR", 0x50480737),
    ("SIMOS", "EXTR", 0x60480737),
    ("SIMOS", "CLZ", 0x11B0040F),
    ("SIMOS", "INSERT", 0xF085F037),
    ("SIMOS", "MUL", 0x2FE2),
    ("SIMOS", "MUL", 0x2FE2),
    ("SIMOS", "MUL", 0x2FE2),
    ("SIMOS", "OR.T", 0x00200187),
    ("SIMOS", "OR.T", 0x00200187),
    ("SIMOS", "INSERT", 0xF0020F37),
    ("SIMOS", "INS.T", 0xF3820F67),
    ("SIMOS", "INS.T", 0x02030F67),
    ("SIMOS", "INSERT", 0x00021037),
    ("SIMOS", "OR.NOR.T", 0x054DDAC7),
    ("SIMOS", "MADDS.Q", 0x85E7EB43),
    ("SIMOS", "AND.ANDN.T", 0xEBF29747),
    ("SIMOS", "EXTR", 0xA04CD737),
    ("SIMOS", "MSUBM.H", 0xD77B0BA3),
    ("SIMOS", "IMASK", 0xE22BCE37),
    ("SIMOS", "INSERT", 0x00004F97),
    ("SIMOS", "IMASK", 0x32353137),
    ("SIMOS", "MADDRS.H", 0x85BE1283),
    ("SIMOS", "SH.XNOR.T", 0x9BDC06A7),
    ("SIMOS", "AND.OR.T", 0xD5A79147),
    ("SIMOS", "INSN.T", 0x14292967),
    ("SIMOS", "MADD", 0x53380D13),
    ("SIMOS", "MSUB.H", 0xC76C51A3),
    ("SIMOS", "OR.ANDN.T", 0xFFFA8CC7),
    ("SIMOS", "INSERT", 0x90057357),
    ("SIMOS", "MADDS", 0x768AE203),
    ("SIMOS", "IMASK", 0x423F3A37),
    ("SIMOS", "AND.OR.T", 0x41B3C047),
    ("SIMOS", "MADD", 0x917D3B13),
    ("SIMOS", "MADD", 0x313BB713),
    ("SIMOS", "MSUBRS.Q", 0xE198E663),
    ("SIMOS", "SH.NOR.T", 0x484B2927),
    ("SIMOS", "MADD.U", 0xE15A2D13),
    ("SIMOS", "MADDS.U", 0x54DA3313),
    ("SIMOS", "AND.T", 0x3F00A987),
    ("SIMOS", "MSUBADMS.H", 0xFFF059E3),
    ("SIMOS", "SH.XNOR.T", 0x59D742A7),
    ("SIMOS", "OR.NOR.T", 0x0ECA73C7),
    ("SIMOS", "SH.ORN.T", 0x4DBCCEA7),
    ("SIMOS", "MSUBRS.H", 0x5AB4DAA3),
    ("SIMOS", "XOR.T", 0x9061E007),
    ("SIMOS", "OR.OR.T", 0xFCA60CC7),
    ("SIMOS", "IMASK", 0x4DBD69B7),
    ("SIMOS", "MADD", 0xEB20E913),
    ("SIMOS", "SH.NAND.T", 0x610802A7),
    ("SIMOS", "MADD.U", 0x975FEC13),
    ("SIMOS", "OR.OR.T", 0x12B131C7),
    ("SIMOS", "SH.ANDN.T", 0x5FEC8027),
    ("SIMOS", "MSUBR.H", 0x70797263),
    ("SIMOS", "MSUB.Q", 0x31722063),
    ("SIMOS", "MSUB.Q", 0x31722063),
    ("SIMOS", "MUL", 0x200A7F73),
    ("SIMOS", "MADD", 0x220A0903),
    ("SIMOS", "MUL", 0x100A9073),
    ("SIMOS", "MUL", 0x200A7073),
    ("SIMOS", "MADD", 0x010A0703),
    ("SIMOS", "MADD", 0xF30AF903),
    ("SIMOS", "EXTR.U", 0xFF610057),
    ("SIMOS", "XNOR.T", 0xDECA8A07),
    ("SIMOS", "CLZ", 0x31B0000F),
    ("SIMOS", "OR.NOR.T", 0x48D0C6C7),
    ("SIMOS", "SH.AND.T", 0xDD8B8B27),
    ("SIMOS", "DEXTR", 0x34867077),
    ("SIMOS", "MSUBRS.Q", 0x4D9B3063),
    ("SIMOS", "SH.XOR.T", 0xB07DABA7),
    ("SIMOS", "MADDSUM.H", 0x857130C3),
    ("SIMOS", "AND.T", 0x18197087),
    ("SIMOS", "AND.T", 0x00000387),
    ("SIMOS", "INSERT", 0x030F0017),
    ("SIMOS", "ORN.T", 0x00200007),
    ("SIMOS", "SH.AND.T", 0x00800027),
    ("SIMOS", "IMASK", 0x00200057),
    ("SIMOS", "IXMIN.U", 0x0090006B),
    ("SIMOS", "EXTR.U", 0xFF610057),
    ("SIMOS", "EXTR.U", 0xFF610057),
    ("SIMOS", "MSUB", 0xFF0A3223),
    ("SIMOS", "MSUB", 0xFF0A1023),
    ("SIMOS", "DEXTR", 0x34867077),
    ("SIMOS", "MSUBRS.Q", 0x4D9B3063),
    ("SIMOS", "SH.XOR.T", 0xB07DABA7),
    ("SIMOS", "MADDSUM.H", 0x857130C3),
    ("SIMOS", "AND.AND.T", 0x00000847),
    ("SIMOS", "SH.NOR.T", 0xCF497627),
    ("SIMOS", "MADDSUR.H", 0x5430D6C3),
    ("SIMOS", "MSUBM.H", 0x117A2DA3),
    ("SIMOS", "MSUBS.Q", 0xE9EDC863),
    ("SIMOS", "SH.NOR.T", 0xF7D44F27),
    ("SIMOS", "SH.XNOR.T", 0xFFD2CAA7),
    ("SIMOS", "INSERT", 0x26097857),
    ("SIMOS", "MSUBMS.H", 0xD2FD8CA3),
    ("SIMOS", "NOR.T", 0xF6D9A887),
    ("SIMOS", "MSUBADRS.H", 0x92BDCCE3),
    ("SIMOS", "MADDS", 0xDEF13C13),
    ("SIMOS", "MSUB", 0x426D1C33),
    ("SIMOS", "MSUBS.Q", 0x40843063),
    ("SIMOS", "OR.OR.T", 0x44A474C7),
    ("SIMOS", "ANDN.T", 0x4CE47C87),
    ("SIMOS", "INSN.T", 0x83B96067),
    ("SIMOS", "MSUBM.H", 0xCB7D28A3),
    ("SIMOS", "XOR.T", 0x5C646C07),
    ("SIMOS", "MADDS.H", 0x1CE02C83),
    ("SIMOS", "DEXTR", 0x34867077),
    ("SIMOS", "SH.XOR.T", 0xB07DABA7),
    ("SIMOS", "MADDSUM.H", 0x857130C3),
    ("SIMOS", "INSERT", 0x1003F517),
    ("SIMOS", "SH.NAND.T", 0x000100A7),
    ("SIMOS", "INSERT", 0x00008017),
    ("SIMOS", "INSERT", 0x000009D7),
    ("SIMOS", "INSERT", 0x000009D7),
    ("SIMOS", "AND.NOR.T", 0x03480347),
    ("SIMOS", "SH.ORN.T", 0x04A804A7),
    ("SIMOS", "DEXTR", 0x80809E17),
    ("SIMOS", "MADDS.Q", 0x80809E43),
    ("SIMOS", "MSUBS.Q", 0x80809F63),
    ("SIMOS", "IXMIN", 0x8080A06B),
    ("SIMOS", "DEXTR", 0x8080A117),
    ("SIMOS", "MSUBS.Q", 0x8080A163),
    ("SIMOS", "MSUBS.U", 0x8080A233),
    ("SIMOS", "MSUBS.Q", 0x8080A263),
    ("SIMOS", "MSUB.Q", 0x10623E63),
    ("SIMOS", "INSN.T", 0x3DB3DB67),
    ("SIMOS", "MADDS", 0xAFA83E13),
    ("SIMOS", "AND.AND.T", 0x77153E47),
    ("SIMOS", "MSUBS.U", 0x53D43D33),
    ("SIMOS", "MADDS", 0xAFA83E13),
    ("SIMOS", "AND.AND.T", 0x77153E47),
    ("SIMOS", "SH.ANDN.T", 0x3FFE3C27),
    ("SIMOS", "MSUBS", 0x3FED4E33),
    ("SIMOS", "XOR.T", 0x0FF83F07),
    ("SIMOS", "SH.ANDN.T", 0x3FFE3C27),
    ("SIMOS", "MSUBS", 0x3FED4E33),
    ("SIMOS", "MADDR.Q", 0xD4183F43),
    ("SIMOS", "AND.NOR.T", 0x4C410047),
    ("SIMOS", "MUL.U", 0x00474353),
    ("SIMOS", "MUL.U", 0x00504353),
    ("SIMOS", "AND.NOR.T", 0x485F4247),
    ("SIMOS", "MUL.U", 0x00474353),
    ("SIMOS", "EXTR", 0x414D5F57),
    ("SIMOS", "EXTR", 0x414D5F57),
    ("SIMOS", "EXTR", 0x414D5F57),
    ("SIMOS", "MSUB.U", 0x56490033),
    ("SIMOS", "MADD.Q", 0x00004543),
    ("SIMOS", "MADD.Q", 0x00005243),
    ("SIMOS", "MSUB.U", 0x55460033),
    ("SIMOS", "MUL", 0x706F7453),
    ("SIMOS", "INSERT", 0x00000057),
    ("SIMOS", "MSUB.Q", 0x73654463),
    ("SIMOS", "MADD.Q", 0x6C764143),
    ("SIMOS", "MADD.Q", 0x6C764143),
    ("SIMOS", "MADD.Q", 0x6C764143),
    ("SIMOS", "MADD.Q", 0x44616843),
    ("SIMOS", "MSUB.Q", 0xEC740363),
    ("SIMOS", "MUL.Q", 0xD0001393),
    ("SIMOS", "INSERT", 0xD0001397),
    ("SIMOS", "INSERT", 0xD000ED97),
    ("SIMOS", "SH.NAND.T", 0xD00013A7),
    ("SIMOS", "MSUBMS.H", 0xBBF404A3),
    ("SIMOS", "OR.AND.T", 0x2B1E9EC7),
    ("SIMOS", "MADDS.U", 0x6A9B2113),
    ("SIMOS", "MSUB.Q", 0x03640363),
    ("SIMOS", "MSUB.Q", 0xD000C763),
    ("SIMOS", "OR.AND.T", 0x2B1E9EC7),
    ("SIMOS", "SH.OR.T", 0x03210027),
    ("SIMOS", "OR.AND.T", 0x000007C7),
    ("SIMOS", "MSUB.Q", 0x00000463),
    ("SIMOS", "IMASK", 0x59295857),
    ("SIMOS", "MSUB.Q", 0x13641663),
    ("SIMOS", "MSUB", 0x71737033),
    ("SIMOS", "MSUB.Q", 0x00000063),
    ("SIMOS", "NOR.T", 0x02C60087),
    ("SIMOS", "SH.OR.T", 0x00280027),
    ("SIMOS", "IMASK", 0x01B801B7),
    ("SIMOS", "MSUBADS.H", 0x00E000E3),
    ("SIMOS", "SH.ORN.T", 0x003F02A7),
    ("SIMOS", "SH.OR.T", 0x02280227),
    ("SIMOS", "MADDS.U", 0x24881713),
    ("SIMOS", "MADDMS.H", 0x6BFA4183),
    ("SIMOS", "AND.ANDN.T", 0x43741447),
    ("SIMOS", "MSUB", 0xDD7B1333),
    ("SIMOS", "MSUBRS.Q", 0x989E3963),
    ("SIMOS", "MADDMS.H", 0xFFFF0083),
    ("SIMOS", "MSUBADS.H", 0x00E400E3),
    ("SIMOS", "IMASK", 0x02B802B7),
    ("SIMOS", "MSUBS", 0x05B10233),
    ("SIMOS", "MADDRS.H", 0x05B10383),
    ("SIMOS", "INSERT", 0x000211D7),
    ("SIMOS", "MSUBADS.H", 0x80E480E3),
    ("SIMOS", "MSUBADS.H", 0x80E480E3),
    ("SIMOS", "MADDS.Q", 0x00800043),
    ("SIMOS", "MADDSU.H", 0x006100C3),
    ("SIMOS", "MSUBS.Q", 0x00E10063),
    ("SIMOS", "MSUBS.Q", 0x02F40263),
    ("SIMOS", "MSUBS.Q", 0x03950163),
    ("SIMOS", "ORN.T", 0x00200007),
    ("SIMOS", "IXMIN.U", 0x0090006B),
    ("SIMOS", "MSUB.Q", 0x7B777C63),
    ("SIMOS", "MADD.U", 0x975FEC13),
    ("SIMOS", "MSUB.H", 0x9E6495A3),
    ("SIMOS", "MUL.H", 0x206F85B3),
    ("SIMOS", "MSUBS.Q", 0x3B84E363),
    ("SIMOS", "MADDSU.H", 0xDF60EFC3),
    ("SIMOS", "IMASK", 0xCF31C2D7),
    ("SIMOS", "MSUBRS.Q", 0xA39CEC63),
    ("SIMOS", "MSUB.Q", 0x7B777C63),
    ("SIMOS", "NOR.T", 0x0FC88087),
    ("SIMOS", "ANDN.T", 0x86EC8087),
    ("SIMOS", "ANDN.T", 0x86F48087),
    ("SIMOS", "DEXTR", 0x60804517),
    ("SIMOS", "MULS", 0xF08AF073),
    ("SIMOS", "MULS", 0x008AF073),
    ("SIMOS", "MSUB", 0x220A0F23),
    ("SIMOS", "MUL.U", 0x00680F73),
    ("SIMOS", "MUL.U", 0x00680F73),
    ("SIMOS", "MUL.U", 0x00684073),
    ("SIMOS", "MULS.U", 0x00802053),
    ("SIMOS", "MULS.U", 0x10802153),
    ("SIMOS", "MULS.U", 0x00802053),
    ("SIMOS", "MUL", 0x406A5473),
    ("SIMOS", "MUL", 0x406A5473),
    ("SIMOS", "MADD.U", 0xCA686503),
    ("SIMOS", "MULS", 0x008A0F73),
    ("SIMOS", "IMASK", 0x242110D7),
    ("SIMOS", "IMASK", 0x042110D7),
    ("SIMOS", "MULS.U", 0xF088FF73),
    ("SIMOS", "MULS.U", 0x4088F273),
    ("SIMOS", "MULS.U", 0x4088F073),
    ("SIMOS", "MADDS", 0x00F28013),
    ("SIMOS", "MADDS.U", 0x07CA8013),
    ("SIMOS", "MSUBS.U", 0x43889333),
    ("SIMOS", "MSUBS.U", 0x43889333),
    ("SIMOS", "MADDS.Q", 0xE8808543),
    ("SIMOS", "MSUBS", 0x9DE50733),
    ("SIMOS", "MADDS.U", 0x60805213),
    ("SIMOS", "XNOR.T", 0x32C20807),
    ("SIMOS", "MADD.Q", 0x53735243),
    ("SIMOS", "XNOR.T", 0x52C45407),
    ("SIMOS", "MSUBS.H", 0x9CE011A3),
    ("SIMOS", "MSUB.Q", 0x60085263),
    ("SIMOS", "MADD.Q", 0x6E088143),
    ("SIMOS", "MSUBS.Q", 0x78E05463),
    ("SIMOS", "MSUBM.H", 0xE37CACA3),
    ("DQ200", "INSERT", 0xF0030FB7),
    ("DQ200", "INSERT", 0xF08700B7),
    ("DQ200", "INSERT", 0xF0040FB7),
    ("DQ200", "EXTR.U", 0x2E640F37),
    ("DQ200", "EXTR.U", 0xF3700F37),
    ("DQ200", "EXTR.U", 0xF0E10F37),
    ("DQ200", "INSERT", 0xF081F037),
    ("DQ200", "MADD", 0xF50A4303),
    ("DQ200", "MUL", 0xF0214253),
    ("DQ200", "MUL", 0xF0214253),
    ("DQ200", "MUL", 0xF0214053),
    ("DQ200", "INS.T", 0x2F84F267),
    ("DQ200", "MADD", 0x500A4A03),
    ("DQ200", "MUL", 0xF0E2),
    ("DQ200", "MADD", 0x2F0A6B03),
    ("DQ200", "INS.T", 0xFF81AF67),
    ("DQ200", "INS.T", 0xFF81AF67),
    ("DQ200", "MUL", 0x01E2),
    ("DQ200", "INSERT", 0xF085F037),
    ("DQ200", "INSERT", 0xF085F037),
    ("DQ200", "EXTR", 0x00480037),
    ("DQ200", "AND.AND.T", 0x17003247),
    ("DQ200", "SH.ANDN.T", 0xFF610127),
    ("DQ200", "SH.ANDN.T", 0xFF620127),
    ("DQ200", "ANDN.T", 0xFF610187),
    ("DQ200", "ANDN.T", 0xFF630187),
    ("DQ200", "NAND.T", 0x00010007),
    ("DQ200", "IMASK", 0x5C312237),
    ("DQ200", "AND.T", 0x769FFC87),
    ("DQ200", "MADDS.U", 0x4381FD13),
    ("DQ200", "XOR.T", 0x0FF75007),
    ("DQ200", "ANDN.T", 0x45EEB487),
    ("DQ200", "MUL", 0x98E2),
    ("DQ200", "MSUB", 0x2E251033),
    ("DQ200", "MADDS.U", 0x8F8D7A13),
    ("DQ200", "MSUBS", 0xBBBB3333),
    ("DQ200", "IMASK", 0x44393637),
    ("DQ200", "INSERT", 0x000000D7),
    ("DQ200", "SH.AND.T", 0x1495C627),
    ("DQ200", "AND.T", 0xA7898A87),
    ("DQ200", "INSERT", 0xFA0B6917),
    ("DQ200", "NAND.T", 0x6A1C4407),
    ("DQ200", "IMASK", 0x643E75B7),
    ("DQ200", "OR.NOR.T", 0xCC5CFEC7),
    ("DQ200", "MSUBADM.H", 0x8A78B8E3),
    ("DQ200", "SH.ORN.T", 0xC03CF2A7),
    ("DQ200", "AND.T", 0xC41CF687),
    ("DQ200", "MSUBR.H", 0x8238B0A3),
    ("DQ200", "OR.NOR.T", 0x6A4E7BC7),
    ("DQ200", "MSUBAD.H", 0x2C6A3DE3),
    ("DQ200", "INSERT", 0xD001FFD7),
    ("DQ200", "NAND.T", 0x00000007),
    ("DQ200", "AND.AND.T", 0x17003247),
    ("DQ200", "IMASK", 0x39363737),
    ("DQ200", "MADDS", 0xFFF2A013),
    ("DQ200", "MADDS", 0xFFF8A013),
    ("DQ200", "MADDS", 0xFFE0A013),
    ("DQ200", "EXTR", 0xB0500B37),
    ("DQ200", "IMASK", 0x643E75B7),
    ("DQ200", "OR.NOR.T", 0xCC5CFEC7),
    ("DQ200", "MSUBADM.H", 0x8A78B8E3),
    ("DQ200", "SH.ORN.T", 0xC03CF2A7),
    ("DQ200", "MSUBR.H", 0x8238B0A3),
    ("DQ200", "MSUBAD.H", 0x2C6A3DE3),
    ("DQ200", "SH.AND.T", 0x00020027),
    ("DQ200", "MSUBR.H", 0x56341BA3),
    ("DQ200", "OR.ANDN.T", 0x467F73C7),
    ("DQ200", "AND.ANDN.T", 0xB4F1FC47),
    ("DQ200", "ORN.T", 0x83393507),
    ("DQ200", "MADD.Q", 0x00000243),
    ("DQ200", "MADD.Q", 0x6F6F4243),
    ("DQ200", "IMASK", 0x30303357),
    ("DQ200", "MSUB", 0x47204833),
    ("DQ200", "AND.NOR.T", 0x4D5F3247),
    ("DQ200", "MSUBADS.H", 0x00E300E3),
    ("DQ200", "XNOR.T", 0x1B580007),
    ("DQ200", "MADD", 0x00200013),
    ("DQ200", "MADD", 0x00200013),
    ("DQ200", "MSUB.U", 0x004D0033),
    ("DQ200", "IXMAX", 0x03A2036B),
    ("DQ200", "INSN.T", 0x00340067),
    ("DQ200", "DEXTR", 0x00000777),
    ("DQ200", "SH.NAND.T", 0x271000A7),
    ("DQ200", "MSUBS.U", 0x00CC0333),
    ("DQ200", "MSUB", 0x02660333),
    ("DQ200", "MSUB", 0x02660333),
    ("DQ200", "MSUB", 0x02660333),
    ("DQ200", "OR.AND.T", 0xFF9C00C7),
    ("DQ200", "ORN.T", 0x00300207),
    ("DQ200", "ORN.T", 0x00200207),
    ("DQ200", "INSERT", 0x00100017),
    ("DQ200", "IXMAX", 0x03A2036B),
    ("DQ200", "IXMAX", 0x03A2036B),
    ("DQ200", "MSUB.U", 0x00470033),
    ("DQ200", "MSUB.Q", 0x10000063),
    ("DQ200", "MSUBS.Q", 0x00960063),
    ("DQ200", "IMASK", 0x00B700B7),
    ("DQ200", "IMASK", 0x30303357),
    ("DQ200", "MSUB", 0x47204833),
    ("DQ200", "AND.NOR.T", 0x4D5F3247),
    ("DQ200", "MSUBADS.H", 0x00E300E3),
    ("DQ200", "MADD", 0x00200013),
    ("DQ200", "MSUB.U", 0x004D0033),
    ("DQ200", "MSUBS", 0x00F00033),
    ("DQ200", "INSN.T", 0x00340067),
    ("DQ200", "MSUBS.U", 0x00CC0333),
    ("DQ200", "OR.AND.T", 0xFF9C00C7),
    ("DQ200", "SH.XNOR.T", 0x005A00A7),
    ("DQ200", "INSERT", 0x00100017),
    ("DQ200", "MSUB.Q", 0x10000063),
    ("DQ200", "MSUBS.Q", 0x00960063),
    ("DQ200", "IMASK", 0x30303357),
    ("DQ200", "AND.NOR.T", 0x4D5F3247),
    ("DQ200", "MSUBADS.H", 0x00E300E3),
    ("DQ200", "SH.ORN.T", 0x012C00A7),
    ("DQ200", "MSUBS.U", 0x00CC0333),
    ("DQ200", "OR.AND.T", 0xFF9C00C7),
    ("DQ200", "SH.XNOR.T", 0x005A00A7),
    ("DQ200", "MSUB.Q", 0x10000063),
    ("DQ200", "MSUBS.Q", 0x00960063),
    ("DQ200", "XNOR.T", 0xEFCAF407),
    ("DQ200", "MSUBS.H", 0xF9E6F9A3),
    ("DQ200", "XOR.T", 0xFFE20007),
    ("DQ200", "MADDSUS.H", 0x14EC14C3),
    ("DQ200", "MADDSUS.H", 0x14EC14C3),
    ("DQ200", "MSUBR.H", 0x02790263),
    ("DQ200", "MADD.Q", 0x03630343),
    ("DQ200", "MADD.Q", 0x03630343),
    ("DQ200", "MADD.Q", 0x03630343),
    ("DQ200", "OR.OR.T", 0x043705C7),
    ("DQ200", "MSUB.H", 0x036B03A3),
    ("DQ200", "XOR.T", 0xFFE20007),
    ("DQ200", "IMASK", 0xFE20FDD7),
    ("DQ200", "MADDSUS.H", 0x14EC14C3),
    ("DQ200", "MSUBR.H", 0x02790263),
    ("DQ200", "XNOR.T", 0xEFCAF407),
    ("DQ200", "MSUBS.H", 0xF9E6F9A3),
    ("DQ200", "MSUBS", 0xFFE2FF33),
    ("DQ200", "MSUBR.H", 0x02790263),
    ("DQ200", "OR.OR.T", 0x043705C7),
    ("DQ200", "MSUB.H", 0x036B03A3),
    ("DQ200", "MADD.U", 0x01450113),
    ("DQ200", "MSUBS", 0xFFE2FF33),
    ("DQ200", "MSUBS.U", 0x03990333),
    ("DQ200", "MSUBADS.H", 0x03ED03E3),
    ("DQ200", "OR.T", 0x03AF0387),
    ("DQ200", "IXMIN.U", 0x2993EA6B),
    ("DQ200", "MSUBADS.H", 0x29E429E3),
    ("DQ200", "MADDS.Q", 0x7A8A2A43),
    ("DQ200", "SH.OR.T", 0xEB28EB27),
    ("DQ200", "MSUB.Q", 0xEB64EB63),
    ("DQ200", "NOR.T", 0x52D2EB87),
    ("DQ200", "AND.ANDN.T", 0x07770747),
    ("DQ200", "EXTR.U", 0x07620757),
    ("DQ200", "INSN.T", 0x27240767),
    ("DQ200", "NOR.T", 0x09452787),
    ("DQ200", "AND.AND.T", 0x07160947),
    ("DQ200", "MADD.H", 0x08680883),
    ("DQ200", "MUL.Q", 0x00100293),
    ("DQ200", "DEXTR", 0x01900177),
    ("DQ200", "MSUBRS.H", 0x02BC02A3),
    ("DQ200", "SH.AND.T", 0x26952527),
    ("DQ200", "MADDS.U", 0x0DCD0A13),
    ("DQ200", "MADDS.U", 0x018B0113),
    ("DQ200", "MSUBADS.H", 0x03E403E3),
    ("DQ200", "OR.ANDN.T", 0x00F500C7),
    ("DQ200", "EXTR", 0x07D00137),
    ("DQ200", "MSUB.Q", 0x00630063),
    ("DQ200", "MADDRS.H", 0x03BB0383),
    ("DQ200", "MSUBS.U", 0x428F4333),
    ("DQ200", "SH.NOR.T", 0x4FD15327),
    ("DQ200", "MSUB.Q", 0x66146D63),
    ("DQ200", "MSUBS.U", 0x009A0133),
    ("DQ200", "MSUBS.Q", 0x03890363),
    ("DQ200", "SH.NAND.T", 0x0D000CA7),
    ("DQ200", "MADD.U", 0x034B0313),
    ("DQ200", "SH.OR.T", 0x2C232927),
    ("DQ200", "EXTR", 0x00490057),
    ("DQ200", "MSUBRS.H", 0xC0B8AFA3),
    ("DQ200", "OR.T", 0xB5AA9887),
    ("DQ200", "EXTR.U", 0x6B676057),
    ("DQ200", "EXTR", 0x57575757),
    ("DQ200", "EXTR", 0x57575757),
    ("DQ200", "INSERT", 0x50005E57),
    ("DQ200", "MSUBRS.H", 0xC0B8AFA3),
    ("DQ200", "MSUBRS.H", 0xC0B8AFA3),
    ("DQ200", "IXMIN.U", 0x0092006B),
    ("DQ200", "SH.OR.T", 0x002F0027),
    ("DQ200", "IXMIN.U", 0x0092006B),
    ("DQ200", "AND.ANDN.T", 0x8C735A47),
    ("DQ200", "MADDR.H", 0x98795E43),
    ("DQ200", "OR.T", 0x33333387),
    ("DQ200", "EXTR.U", 0x00730057),
    ("DQ200", "MUL.U", 0x00530053),
    ("DQ200", "MUL.U", 0x00530053),
    ("DQ200", "SH.NOR.T", 0x004A0027),
    ("DQ200", "MUL", 0x00770053),
    ("DQ200", "MSUBS", 0x00B27F33),
    ("DQ200", "MADD.U", 0x00488013),
    ("DQ200", "MADD", 0x027C8013),
    ("DQ200", "MSUB.H", 0x9E6495A3),
    ("DQ200", "INSERT", 0x6116C8D7),
    ("DQ200", "INSERT", 0xF50FC457),
    ("DQ200", "MUL.H", 0x206F85B3),
    ("DQ200", "MSUBS.Q", 0x3B84E363),
    ("DQ200", "MADDSU.H", 0xDF60EFC3),
    ("DQ200", "IMASK", 0xCF31C2D7),
    ("DQ200", "MSUBRS.Q", 0xA39CEC63),
    ("DQ200", "MSUB.Q", 0x7B777C63),
    ("DQ200", "AND.OR.T", 0xD4ADF047),
    ("DQ200", "OR.OR.T", 0x12B131C7),
    ("DQ200", "SH.ANDN.T", 0x5FEC8027),
    ("DQ200", "MSUB.Q", 0x65676163),
    ("DQ200", "MSUBADS.H", 0xE5E7E1E3),
    ("DQ200", "MSUB", 0xF20A7323),
    ("DQ200", "MSUB", 0x110AE923),
    ("DQ200", "MSUB", 0xF10ABF23),
    ("DQ200", "CLZ", 0xF1B00F0F),
    ("DQ200", "IMASK", 0x252110D7),
    ("DQ200", "MUL", 0x300A0173),
    ("DQ200", "MUL", 0x300A2173),
    ("DQ200", "MUL", 0x100A2573),
    ("DQ200", "DEXTR", 0x01800177),
    ("DQ200", "MADDMS.H", 0x00F30083),
    ("DQ200", "MUL.U", 0x00550053),
    ("DQ200", "MSUB.Q", 0x00720063),
    ("DQ200", "MSUBRS.H", 0x00B500A3),
    ("DQ200", "MSUBADS.H", 0x08EB08E3),
    ("DQ200", "INSERT", 0x26097857),
    ("DQ200", "MSUBMS.H", 0xD2FD8CA3),
    ("DQ200", "NOR.T", 0xF6D9A887),
    ("DQ200", "MSUBADRS.H", 0x92BDCCE3),
    ("DQ200", "MADD.H", 0x936C8183),
    ("DQ200", "SH.NAND.T", 0x000000A7),
    ("DQ200", "MADD", 0x00638013),
    ("DQ200", "MADDS", 0x00B28013),
    ("DQ200", "MADDS.H", 0x93EC8183),
    ("DQ200", "MADDR.H", 0x943C8183),
    ("DQ200", "CLO", 0x01C1310F),
    ("DQ200", "CLO", 0x01C3370F),
    ("DQ200", "CLO", 0x01C3090F),
    ("DQ200", "MADDRS.H", 0x03BB0383),
    ("DQ200", "MADD", 0xB17C8013),
    ("DQ200", "MADDS.U", 0xB2D08013),
    ("DQ200", "MADDS.U", 0xC4D48013),
    ("DQ200", "MADDS", 0xB5B08013),
    ("DQ200", "MADDS", 0xB7A08013),
    ("DQ200", "INSERT", 0x00000097),
    ("DQ200", "MADD.Q", 0x006E0043),
    ("DQ200", "MUL", 0x00720053),
    ("DQ200", "MSUB.Q", 0x00740063),
    ("DQ200", "MSUBADM.H", 0x8A78B8E3),
    ("DQ200", "MSUBR.H", 0x8238B0A3),
    ("DQ200", "MSUBAD.H", 0x2C6A3DE3),
    ("DQ200", "MSUB.H", 0x9E6495A3),
    ("DQ200", "MUL.H", 0x206F85B3),
    ("DQ200", "MSUBS.Q", 0x3B84E363),
    ("DQ200", "MADDSU.H", 0xDF60EFC3),
    ("DQ200", "MSUBRS.Q", 0xA39CEC63),
];

const CODE: u32 = 0xC000_0000;
const CODE_SIZE: usize = 0x1_0000;
const PSW_BASE: u32 = 0x0000_0B80;
const V: u32 = 1 << 30;
const SV: u32 = 1 << 29;
const AV: u32 = 1 << 28;
const SAV: u32 = 1 << 27;

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn u32(&mut self) -> u32 {
        (self.next() >> 16) as u32
    }
    fn below(&mut self, n: u32) -> u32 {
        (self.next() % n as u64) as u32
    }
    /// Operand value: uniform, or built from the edge cases of the pages (8000H, 7FFFH, -1, 0, ...).
    fn operand(&mut self) -> u32 {
        const W: [u32; 14] = [
            0, 1, 0xFFFF_FFFF, 0x8000_0000, 0x7FFF_FFFF, 0x4000_0000, 0xC000_0000, 0x8000_8000, 0x7FFF_7FFF,
            0x0000_8000, 0x8000_0001, 0xFFFF_0000, 0x0000_FFFF, 0x3FFF_C000,
        ];
        const H: [u32; 8] = [0x8000, 0x7FFF, 0xFFFF, 0, 1, 0x4000, 0xC000, 0x8001];
        match self.below(8) {
            0 => W[self.below(W.len() as u32) as usize],
            1 => H[self.below(8) as usize] << 16 | H[self.below(8) as usize],
            2 => (self.below(9) as i32 - 4) as u32,
            3 => H[self.below(8) as usize] << 16 | self.u32() & 0xFFFF,
            _ => self.u32(),
        }
    }
}

/// (lowest bit, width) of op2 per format (V2 Table 1-6 p. 1-4).
fn op2_field(fmt: &str) -> (u32, u32) {
    match fmt {
        "BIT" | "RRPW" | "RCPW" => (21, 2),
        "RR" => (20, 8),
        "RR1" => (18, 10),
        "RR2" => (16, 12),
        "RRR" => (20, 4),
        "RRR1" => (18, 6),
        "RRR2" => (16, 8),
        "RRRR" | "RRRW" | "RCR" | "RCRR" | "RCRW" => (21, 3),
        "RC" => (21, 7),
        "SRR" => (0, 0),
        f => panic!("format {f}"),
    }
}

fn is_extr(e: &E) -> bool {
    e.mn.starts_with("EXTR")
}

/// The instruction word is in the defined range of its page (EXTR: width > 0 and pos + width <= 32,
/// INSERT/IMASK: pos + width <= 32 for immediate pos/width).
fn defined(e: &E, w: u32) -> bool {
    let (pos, width) = ((w >> 23) & 31, (w >> 16) & 31);
    match e.fmt {
        "RRPW" | "RCPW" if e.mn != "DEXTR" => pos + width <= 32 && (width > 0 || !is_extr(e)),
        "RRRW" => width > 0 || !is_extr(e),
        _ => true,
    }
}

/// A random instance of encoding `e`.
fn instance(e: &E, rng: &mut Rng) -> u32 {
    if e.fmt == "SRR" {
        return rng.u32() & 0xFF00 | e.op1 as u32;
    }
    let (lo, bits) = op2_field(e.fmt);
    loop {
        let mut w = rng.u32() & !(((1u32 << bits) - 1) << lo) & !0xFF | e.op1 as u32 | (e.op2 as u32) << lo;
        if e.syn.starts_with("E[c]") {
            w &= !(1 << 28);
        }
        if e.syn.contains("E[d]") {
            w &= !(1 << 24);
        }
        if e.syn.ends_with(", n") {
            w &= !(1 << 17);
        }
        if matches!(e.mn, "BMERGE" | "BSPLIT" | "PARITY" | "IXMAX" | "IXMAX.U" | "IXMIN" | "IXMIN.U") {
            w &= !(3 << 16);
        }
        if defined(e, w) {
            return w;
        }
    }
}

/// Register values for one case: random operands, then register-held positions/widths restricted to
/// their defined range.
fn operands(e: &E, w: u32, rng: &mut Rng) -> ([u32; 16], u32) {
    let mut d = [0u32; 16];
    for v in d.iter_mut() {
        *v = rng.operand();
    }
    let psw = rng.u32() & 0xF800_0000 | PSW_BASE;
    let fd = ((w >> 24) & 15) as usize;
    let width = (w >> 16) & 31;
    match (e.mn, e.fmt) {
        ("DEXTR", "RRRR") => d[fd] = rng.below(32),
        (_, "RRRW" | "RCRW") => d[fd] = d[fd] & !31 | rng.below(33 - width),
        (_, "RRRR" | "RCRR") => {
            let pos = rng.below(32);
            let maxw = (32 - pos).min(31);
            let wd = if is_extr(e) { 1 + rng.below(maxw) } else { rng.below(maxw + 1) };
            d[fd] = d[fd] & !31 | pos;
            d[fd + 1] = d[fd + 1] & !31 | wd;
        }
        _ => {}
    }
    (d, psw)
}

// ---- literal RTL of the manual pages ----------------------------------------------------------

fn bit(v: u32, i: u32) -> bool {
    v >> i & 1 == 1
}

/// `result[w-1] ^ result[w-2]`.
fn adv(r: i128, w: u32) -> bool {
    ((r >> (w - 1)) ^ (r >> (w - 2))) & 1 == 1
}

fn out_s(r: i128, w: u32) -> bool {
    r > (1i128 << (w - 1)) - 1 || r < -(1i128 << (w - 1))
}

fn ssov(r: i128, w: u32) -> i128 {
    let (max_pos, max_neg) = ((1i128 << (w - 1)) - 1, -(1i128 << (w - 1)));
    if r > max_pos {
        max_pos
    } else if r < max_neg {
        max_neg
    } else {
        r
    }
}

fn suov(r: i128, w: u32) -> i128 {
    let max_pos = (1i128 << w) - 1;
    if r > max_pos {
        max_pos
    } else if r < 0 {
        0
    } else {
        r
    }
}

/// `if (overflow) V = 1, SV = 1 else V = 0; if (advanced_overflow) AV = 1, SAV = 1 else AV = 0`.
fn status(psw: &mut u32, overflow: bool, advanced: bool) {
    *psw &= !(V | AV);
    if overflow {
        *psw |= V | SV;
    }
    if advanced {
        *psw |= AV | SAV;
    }
}

fn leading(v: u32, bits: u32, one: bool) -> u32 {
    let mut k = 0;
    for i in (0..bits).rev() {
        if bit(v, i) != one {
            break;
        }
        k += 1;
    }
    k
}

fn h16(v: u32, upper: bool) -> i128 {
    if upper {
        (v >> 16) as i16 as i128
    } else {
        v as i16 as i128
    }
}

/// Execute `e` (instance `w`) on registers `d`/`psw` exactly as its page reads.
fn reference(e: &E, w: u32, d0: &[u32; 16], psw0: u32) -> ([u32; 16], u32) {
    let mut d = *d0;
    let mut psw = psw0;
    let c = ((w >> 28) & 15) as usize;
    let dd = ((w >> 24) & 15) as usize;
    let a = ((w >> 8) & 15) as usize;
    let b = ((w >> 12) & 15) as usize;
    let n = (w >> 16) & 3;
    let (mn, syn) = (e.mn, e.syn);
    let e_get = |d: &[u32; 16], r: usize| d[r] as u64 | (d[r + 1] as u64) << 32;
    let e_set = |d: &mut [u32; 16], r: usize, v: u64| {
        d[r] = v as u32;
        d[r + 1] = (v >> 32) as u32;
    };

    if e.fmt == "BIT" {
        // V2 p. 3-32, 3-40, 3-42, 3-111, 3-286, 3-292, 3-296, 3-304, 3-306, 3-335, 3-384, 3-392.
        let (p1, p2) = ((w >> 16) & 31, (w >> 23) & 31);
        let (x, y) = (bit(d[a], p1), bit(d[b], p2));
        let f = |op: &str| match op {
            "AND" => x && y,
            "ANDN" => x && !y,
            "NAND" => !(x && y),
            "NOR" => !(x || y),
            "OR" => x || y,
            "ORN" => x || !y,
            "XNOR" => !(x ^ y),
            "XOR" => x ^ y,
            o => panic!("{o}"),
        };
        let parts: Vec<&str> = mn.split('.').collect();
        d[c] = match parts.as_slice() {
            ["INS", "T"] => d[a] & !(1 << p1) | (y as u32) << p1,
            ["INSN", "T"] => d[a] & !(1 << p1) | (!y as u32) << p1,
            [op, "T"] => f(op) as u32,
            ["SH", op, "T"] => d[c] << 1 | f(op) as u32,
            ["AND", op, "T"] => d[c] & !1 | (bit(d[c], 0) && f(op)) as u32,
            ["OR", op, "T"] => d[c] & !1 | (bit(d[c], 0) || f(op)) as u32,
            _ => panic!("{mn}"),
        };
        return (d, psw);
    }

    match mn {
        // V2 p. 3-68 .. 3-73.
        "CLO" => d[c] = leading(d[a], 32, true),
        "CLZ" => d[c] = leading(d[a], 32, false),
        "CLS" => d[c] = leading(d[a], 32, bit(d[a], 31)) - 1,
        "CLO.H" => d[c] = leading(d[a] >> 16, 16, true) << 16 | leading(d[a] & 0xFFFF, 16, true),
        "CLZ.H" => d[c] = leading(d[a] >> 16, 16, false) << 16 | leading(d[a] & 0xFFFF, 16, false),
        "CLS.H" => {
            let (hi, lo) = (d[a] >> 16, d[a] & 0xFFFF);
            d[c] = (leading(hi, 16, bit(hi, 15)) - 1) << 16 | (leading(lo, 16, bit(lo, 15)) - 1);
        }
        // V2 p. 3-45: D[c][31:24] = {D[a][15], D[b][15], ...}.
        "BMERGE" => {
            let mut v = 0;
            for i in 0..16 {
                v |= (bit(d[a], i) as u32) << (2 * i + 1) | (bit(d[b], i) as u32) << (2 * i);
            }
            d[c] = v;
        }
        // V2 p. 3-46: E[c][7:0] = {D[a][14], ..., D[a][0]}, E[c][39:32] = {D[a][15], ..., D[a][1]}.
        "BSPLIT" => {
            let (mut lo, mut hi) = (0u32, 0u32);
            for i in 0..16 {
                lo |= (bit(d[a], 2 * i) as u32) << i;
                hi |= (bit(d[a], 2 * i + 1) as u32) << i;
            }
            d[c] = lo;
            d[c + 1] = hi;
        }
        // V2 p. 3-309.
        "PARITY" => {
            let mut v = 0;
            for byte in 0..4 {
                let mut p = false;
                for i in 0..8 {
                    p ^= bit(d[a], 8 * byte + i);
                }
                v |= (p as u32) << (8 * byte);
            }
            d[c] = v;
        }
        // V2 p. 3-79: D[c] = ({D[a], D[b]} << pos)[63:32].
        "DEXTR" => {
            let pos = if e.fmt == "RRPW" { (w >> 23) & 31 } else { d[dd] & 31 };
            d[c] = ((((d[a] as u128) << 32 | d[b] as u128) << pos) >> 32) as u32;
        }
        // V2 p. 3-100/3-101.
        "EXTR" | "EXTR.U" => {
            let (pos, width) = match e.fmt {
                "RRPW" => ((w >> 23) & 31, (w >> 16) & 31),
                "RRRR" => (d[dd] & 31, d[dd + 1] & 31),
                _ => (d[dd] & 31, (w >> 16) & 31),
            };
            let v = (d[a] >> pos) as u64 & ((1u64 << width) - 1);
            d[c] = if mn == "EXTR" && v >> (width - 1) & 1 == 1 { (v | !((1u64 << width) - 1)) as u32 } else { v as u32 };
        }
        // V2 p. 3-112/3-113: mask = (2^width - 1) << pos; D[c] = (D[a] & ~mask) | ((value << pos) & mask).
        "INSERT" => {
            let value = if e.fmt.starts_with("RC") { (w >> 12) & 15 } else { d[b] };
            let (pos, width) = match e.fmt {
                "RCPW" | "RRPW" => ((w >> 23) & 31, (w >> 16) & 31),
                "RCRR" | "RRRR" => (d[dd] & 31, d[dd + 1] & 31),
                _ => (d[dd] & 31, (w >> 16) & 31),
            };
            let mask = (((1u64 << width) - 1) << pos) as u32;
            d[c] = d[a] & !mask | ((value as u64) << pos) as u32 & mask;
        }
        // V2 p. 3-109: E[c][63:32] = (2^width - 1) << pos; E[c][31:0] = value << pos.
        "IMASK" => {
            let value = if e.fmt.starts_with("RC") { (w >> 12) & 15 } else { d[b] };
            let pos = if e.fmt.ends_with("PW") { (w >> 23) & 31 } else { d[dd] & 31 };
            let width = (w >> 16) & 31;
            d[c] = ((value as u64) << pos) as u32;
            d[c + 1] = (((1u64 << width) - 1) << pos) as u32;
        }
        // V2 p. 3-115..3-118.
        "IXMAX" | "IXMAX.U" | "IXMIN" | "IXMIN.U" => {
            let k = |v: u32| if mn.ends_with(".U") { (v & 0xFFFF) as i64 } else { v as u16 as i16 as i64 };
            let (ed0, ed1, db) = (d[dd], d[dd + 1], d[b]);
            let (first, second, cur) = (k(db), k(db >> 16), k(ed1));
            let wi = ed0 & 0xFFFF;
            let (val, idx) = if mn.starts_with("IXMAX") {
                if first >= second && first > cur {
                    (db & 0xFFFF, wi)
                } else if second > first && second > cur {
                    (db >> 16, wi + 1)
                } else {
                    (ed1 & 0xFFFF, ed0 >> 16)
                }
            } else if first <= second && first < cur {
                (db & 0xFFFF, wi)
            } else if second < first && second < cur {
                (db >> 16, wi + 1)
            } else {
                (ed1 & 0xFFFF, ed0 >> 16)
            };
            d[c] = (idx & 0xFFFF) << 16 | (wi + 2) & 0xFFFF;
            d[c + 1] = val;
        }
        // V2 p. 3-180 (MADD/MADDS), 3-193 (MADD.U/MADDS.U), 3-233 (MSUB/MSUBS), 3-246 (MSUB.U/MSUBS.U),
        // 3-269 (MUL/MULS), 3-277 (MUL.U/MULS.U).
        "MUL" | "MULS" | "MUL.U" | "MULS.U" | "MADD" | "MADDS" | "MADD.U" | "MADDS.U" | "MSUB" | "MSUBS" | "MSUB.U"
        | "MSUBS.U" => {
            let uns = mn.ends_with(".U");
            let base = mn.trim_end_matches(".U");
            let sat = base.ends_with('S');
            let base = base.trim_end_matches('S');
            let wide = syn.starts_with("E[c]");
            let val = |v: u32| if uns { v as i128 } else { v as i32 as i128 };
            let (x, y, c) = match e.fmt {
                "SRR" => (val(d[a]), val(d[b]), a),
                "RC" | "RCR" => {
                    let k9 = (w >> 12) & 0x1FF;
                    (val(d[a]), if uns { k9 as i128 } else { ((k9 << 23) as i32 >> 23) as i128 }, c)
                }
                _ => (val(d[a]), val(d[b]), c),
            };
            let acc = match (base, wide) {
                ("MUL", _) => 0,
                (_, true) => if uns { e_get(&d, dd) as i128 } else { e_get(&d, dd) as i64 as i128 },
                (_, false) => val(d[dd]),
            };
            let result = if base == "MSUB" { acc - x * y } else { acc + x * y };
            let wbits = if wide { 64 } else { 32 };
            let overflow = if uns { result > (1i128 << wbits) - 1 || result < 0 } else { out_s(result, wbits) };
            status(&mut psw, overflow, adv(result, wbits));
            let v = if !sat { result } else if uns { suov(result, wbits) } else { ssov(result, wbits) };
            if wide {
                e_set(&mut d, c, v as u64);
            } else {
                d[c] = v as u32;
            }
        }
        // V2 p. 3-187 (MADD.Q/MADDS.Q), 3-202 (MADDR.Q/MADDRS.Q), 3-240 (MSUB.Q/MSUBS.Q), 3-266 (MSUBR.Q/
        // MSUBRS.Q), 3-274 (MUL.Q), 3-284 (MULR.Q).
        m if m.ends_with(".Q") => {
            let base = m.trim_end_matches(".Q");
            let mul = base.starts_with("MUL");
            let round = base.contains('R');
            let sat = !mul && base.ends_with('S');
            let wide = syn.starts_with("E[c]");
            let wbits = if wide { 64 } else { 32 };
            let a32 = d[a] as i32 as i128;
            let (prod, sc) = if syn.contains("D[a] L, D[b] L") || syn.contains("D[a] U, D[b] U") {
                let up = syn.contains("D[a] U");
                let (x, y) = (h16(d[a], up), h16(d[b], up));
                let sc = x == -0x8000 && y == -0x8000 && n == 1;
                let mul_res = if sc { 0x7FFF_FFFF } else { (x * y) << n };
                (if wide { mul_res << 16 } else { mul_res }, sc)
            } else if syn.contains("D[b] L") || syn.contains("D[b] U") {
                let p = (a32 * h16(d[b], syn.contains("D[b] U"))) << n;
                (if wide { p } else { p >> 16 }, false)
            } else {
                let p = (a32 * d[b] as i32 as i128) << n;
                (if wide { p } else { p >> 32 }, false)
            };
            if mul && round {
                // result = sc ? 7FFFFFFFH : ((D[a][15:0] * D[b][15:0]) << n) + 8000H; V cleared.
                let result = if sc { 0x7FFF_FFFF } else { prod + 0x8000 };
                d[c] = (result as u32 >> 16) << 16;
                psw &= !V;
                if adv(result, 32) {
                    psw |= AV | SAV;
                } else {
                    psw &= !AV;
                }
            } else {
                let acc = if mul {
                    0
                } else if wide {
                    e_get(&d, dd) as i64 as i128
                } else {
                    d[dd] as i32 as i128
                };
                let mut result = if base.starts_with("MSUB") { acc - prod } else { acc + prod };
                if round {
                    result += 0x8000;
                }
                status(&mut psw, out_s(result, wbits), adv(result, wbits));
                let v = if sat { ssov(result, wbits) } else { result };
                if wide {
                    e_set(&mut d, c, v as u64);
                } else if round {
                    d[c] = (v as u32 >> 16) << 16;
                } else {
                    d[c] = v as u32;
                }
            }
        }
        // Packed half-word pages: V2 p. 3-183, 3-195, 3-198, 3-204, 3-208, 3-211, 3-236, 3-248, 3-252,
        // 3-255, 3-259, 3-262, 3-272, 3-279, 3-281, 3-282.
        m if m.ends_with(".H") => {
            let base = &m[..m.len() - 2];
            // '+'/'-' applied to mul_res1 / mul_res0, the page family, saturation.
            let (s1, s0, fam, sat) = match base {
                "MUL" | "MADD" => ('+', '+', 'P', false),
                "MADDS" => ('+', '+', 'P', true),
                "MULM" | "MADDM" => ('+', '+', 'M', false),
                "MULMS" | "MADDMS" => ('+', '+', 'M', true),
                "MULR" | "MADDR" => ('+', '+', 'R', false),
                "MADDRS" => ('+', '+', 'R', true),
                "MADDSU" => ('+', '-', 'P', false),
                "MADDSUS" => ('+', '-', 'P', true),
                "MADDSUM" => ('+', '-', 'M', false),
                "MADDSUMS" => ('+', '-', 'M', true),
                "MADDSUR" => ('+', '-', 'R', false),
                "MADDSURS" => ('+', '-', 'R', true),
                "MSUB" => ('-', '-', 'P', false),
                "MSUBS" => ('-', '-', 'P', true),
                "MSUBM" => ('-', '-', 'M', false),
                "MSUBMS" => ('-', '-', 'M', true),
                "MSUBR" => ('-', '-', 'R', false),
                "MSUBRS" => ('-', '-', 'R', true),
                "MSUBAD" => ('-', '+', 'P', false),
                "MSUBADS" => ('-', '+', 'P', true),
                "MSUBADM" => ('-', '+', 'M', false),
                "MSUBADMS" => ('-', '+', 'M', true),
                "MSUBADR" => ('-', '+', 'R', false),
                "MSUBADRS" => ('-', '+', 'R', true),
                o => panic!("{o}"),
            };
            let mul = base.starts_with("MUL");
            let mode = &syn[syn.find("D[b] ").unwrap() + 5..][..2];
            // (D[a] half, D[b] half) of product 1 and product 0, true = [31:16].
            let ((a1, b1), (a0, b0)) = match mode {
                "LL" => ((true, false), (false, false)),
                "LU" => ((true, false), (false, true)),
                "UL" => ((true, true), (false, false)),
                "UU" => ((false, true), (true, true)),
                o => panic!("{o}"),
            };
            let (x1, y1, x0, y0) = (h16(d[a], a1), h16(d[b], b1), h16(d[a], a0), h16(d[b], b0));
            let sc1 = x1 == -0x8000 && y1 == -0x8000 && n == 1;
            let sc0 = x0 == -0x8000 && y0 == -0x8000 && n == 1;
            let mul_res1 = if sc1 { 0x7FFF_FFFF } else { (x1 * y1) << n };
            let mul_res0 = if sc0 { 0x7FFF_FFFF } else { (x0 * y0) << n };
            let ap = |acc: i128, s: char, m: i128| if s == '+' { acc + m } else { acc - m };
            match fam {
                'P' => {
                    let (r1, r0) = if mul {
                        (mul_res1, mul_res0)
                    } else {
                        (ap(d[dd + 1] as i32 as i128, s1, mul_res1), ap(d[dd] as i32 as i128, s0, mul_res0))
                    };
                    let aov = adv(r1, 32) || adv(r0, 32);
                    if mul {
                        // MUL.H p. 3-273: V cleared, SV not set, AV and SAV from the words.
                        psw &= !V;
                        psw = if aov { psw | AV | SAV } else { psw & !AV };
                    } else {
                        status(&mut psw, out_s(r1, 32) || out_s(r0, 32), aov);
                    }
                    let (v1, v0) = if sat { (ssov(r1, 32), ssov(r0, 32)) } else { (r1, r0) };
                    d[c] = v0 as u32;
                    d[c + 1] = v1 as u32;
                }
                'M' => {
                    let ed = e_get(&d, dd) as i64 as i128;
                    let result = match base {
                        "MULM" | "MULMS" => (mul_res1 + mul_res0) << 16,
                        "MADDM" | "MADDMS" => ed + ((mul_res1 + mul_res0) << 16),
                        "MSUBM" | "MSUBMS" => ed - ((mul_res1 + mul_res0) << 16),
                        "MADDSUM" | "MADDSUMS" => ed + ((mul_res1 - mul_res0) << 16),
                        _ => ed - ((mul_res1 - mul_res0) << 16),
                    };
                    match base {
                        // MULM.H p. 3-280: V and AV cleared.
                        "MULM" => psw &= !(V | AV),
                        // MULMS.H p. 3-281: no status flag set.
                        "MULMS" => {}
                        _ => status(&mut psw, out_s(result, 64), adv(result, 64)),
                    }
                    let v = if sat { ssov(result, 64) } else { result };
                    e_set(&mut d, c, v as u64);
                }
                _ => {
                    let (h1, h0) = if mul {
                        (
                            if sc1 { 0x7FFF_FFFF } else { mul_res1 + 0x8000 },
                            if sc0 { 0x7FFF_FFFF } else { mul_res0 + 0x8000 },
                        )
                    } else {
                        let (acc1, acc0) = if syn.contains("E[d]") {
                            (d[dd + 1] as i32 as i128, d[dd] as i32 as i128)
                        } else {
                            ((d[dd] & 0xFFFF_0000) as i32 as i128, (d[dd] << 16) as i32 as i128)
                        };
                        (ap(acc1, s1, mul_res1) + 0x8000, ap(acc0, s0, mul_res0) + 0x8000)
                    };
                    let aov = adv(h1, 32) || adv(h0, 32);
                    if mul {
                        // MULR.H p. 3-283: V cleared, AV/SAV from the half-words.
                        psw &= !V;
                        psw = if aov { psw | AV | SAV } else { psw & !AV };
                    } else {
                        status(&mut psw, out_s(h1, 32) || out_s(h0, 32), aov);
                    }
                    let (v1, v0) = if sat { (ssov(h1, 32), ssov(h0, 32)) } else { (h1, h0) };
                    d[c] = (v1 as u32 >> 16) << 16 | v0 as u32 >> 16;
                }
            }
        }
        _ => panic!("no reference for {mn}"),
    }
    (d, psw)
}

// ---- engines ------------------------------------------------------------------------------------

fn native_cpu(code: &[u8], arch: Arch) -> Cpu<NullBus> {
    let mut m = Memory::new();
    let r = m.add_owned(vec![0; CODE_SIZE], true);
    m.map(CODE, r, 0, CODE_SIZE);
    let mut c = Cpu::new(arch, m, NullBus);
    c.write_bytes(CODE, code).unwrap();
    c
}

fn native_step(c: &mut Cpu<NullBus>, pc: u32, d: &[u32; 16], psw: u32) -> ([u32; 16], u32, u32) {
    c.d = *d;
    c.psw = psw;
    c.pc = pc;
    assert_eq!(c.run(1), Exit::Budget);
    (c.d, c.psw, c.pc)
}

/// Bare Unicorn with the code region mapped, kept open across cases (opening an engine per case, as
/// `testutil::oracle::run` does, costs about 0.6 ms).
struct Uc(*mut uc_engine);

impl Uc {
    fn new(code: &[u8]) -> Uc {
        let mut uc = std::ptr::null_mut();
        // SAFETY: plain Unicorn C API use; `code` outlives the copy into guest memory.
        unsafe {
            assert_eq!(uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc), UC_ERR_OK);
            assert_eq!(uc_mem_map(uc, CODE as u64, CODE_SIZE, UC_PROT_ALL as u32), UC_ERR_OK);
            assert_eq!(uc_mem_write(uc, CODE as u64, code.as_ptr().cast(), code.len()), UC_ERR_OK);
            let btv = CODE + 0xF000;
            uc_reg_write(uc, TriCoreReg::BTV as c_int, (&btv as *const u32).cast());
        }
        Uc(uc)
    }

    fn step(&mut self, pc: u32, d: &[u32; 16], psw: u32) -> Result<([u32; 16], u32, u32), uc_err> {
        let mut out = [0u32; 16];
        let (mut psw_o, mut pc_o) = (0u32, 0u32);
        // SAFETY: valid engine; register ids from the enum; pointers to live locals.
        unsafe {
            for (i, v) in d.iter().enumerate() {
                uc_reg_write(self.0, TriCoreReg::d(i as u32) as c_int, (v as *const u32).cast());
            }
            // Unicorn keeps C/V/SV/AV/SAV in the QEMU `PSW_USB_*` caches (flag in bit 31, C nonzero)
            // and reads/writes `PSW` raw, so the status bits go through the cache registers.
            uc_reg_write(self.0, TriCoreReg::PSW as c_int, (&psw as *const u32).cast());
            for (r, v) in [
                (TriCoreReg::PSW_USB_C, psw & 1 << 31),
                (TriCoreReg::PSW_USB_V, (psw & V) << 1),
                (TriCoreReg::PSW_USB_SV, (psw & SV) << 2),
                (TriCoreReg::PSW_USB_AV, (psw & AV) << 3),
                (TriCoreReg::PSW_USB_SAV, (psw & SAV) << 4),
            ] {
                uc_reg_write(self.0, r as c_int, (&v as *const u32).cast());
            }
            let e = uc_emu_start(self.0, pc as u64, 0, 0, 1);
            if e != UC_ERR_OK {
                return Err(e);
            }
            for (i, o) in out.iter_mut().enumerate() {
                uc_reg_read(self.0, TriCoreReg::d(i as u32) as c_int, (o as *mut u32).cast());
            }
            uc_reg_read(self.0, TriCoreReg::PSW as c_int, (&mut psw_o as *mut u32).cast());
            let mut usb = [0u32; 5];
            for (k, r) in [TriCoreReg::PSW_USB_C, TriCoreReg::PSW_USB_V, TriCoreReg::PSW_USB_SV, TriCoreReg::PSW_USB_AV, TriCoreReg::PSW_USB_SAV]
                .into_iter()
                .enumerate()
            {
                uc_reg_read(self.0, r as c_int, (&mut usb[k] as *mut u32).cast());
            }
            psw_o = psw_o & 0x07FF_FFFF
                | ((usb[0] != 0) as u32) << 31
                | (usb[1] & 1 << 31) >> 1
                | (usb[2] & 1 << 31) >> 2
                | (usb[3] & 1 << 31) >> 3
                | (usb[4] & 1 << 31) >> 4;
            uc_reg_read(self.0, TriCoreReg::PC as c_int, (&mut pc_o as *mut u32).cast());
        }
        Ok((out, psw_o, pc_o))
    }
}

impl Drop for Uc {
    fn drop(&mut self) {
        // SAFETY: engine opened in `new`.
        unsafe { uc_close(self.0) };
    }
}

/// Instances that make Unicorn 2.1.4 abort the process (SIGILL), so they cannot be compared
/// in-process: INSERT RCPW/RRPW with width 0 (a zero-length deposit in its translator). V2 p. 3-112
/// defines them: mask = 0, D[c] = D[a]. IMASK with width 0 executes normally.
fn unicorn_aborts(e: &E, w: u32) -> bool {
    e.mn == "INSERT" && matches!(e.fmt, "RCPW" | "RRPW") && (w >> 16) & 31 == 0
}

/// `insns` instances of every encoding available on `arch`, laid out in 4-byte slots.
fn layout(arch: Arch, insns: usize, rng: &mut Rng) -> (Vec<u8>, Vec<(usize, u32)>) {
    let mut code = Vec::new();
    let mut slots = Vec::new();
    for (i, e) in ENC.iter().enumerate() {
        if arch == Arch::Tc131 && e.mn == "MULMS.H" {
            continue;
        }
        for _ in 0..insns {
            let w = instance(e, rng);
            slots.push((i, w));
            code.extend_from_slice(&w.to_le_bytes());
        }
    }
    assert!(code.len() <= CODE_SIZE - 0x1000);
    (code, slots)
}

fn len_of(w: u32) -> u32 {
    if w & 1 == 0 {
        2
    } else {
        4
    }
}

fn show(d: &[u32; 16], psw: u32) -> String {
    let regs: Vec<String> = d.iter().map(|v| format!("{v:08x}")).collect();
    format!("d=[{}] psw={psw:08x}", regs.join(" "))
}

// ---- tests --------------------------------------------------------------------------------------

#[test]
fn table_matches_decoder() {
    assert_eq!(ENC.len(), 270);
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let mut by_op1: BTreeMap<u8, Vec<u16>> = BTreeMap::new();
    for e in ENC {
        by_op1.entry(e.op1).or_default().push(e.op2);
        for _ in 0..64 {
            let w = instance(e, &mut rng);
            for arch in [Arch::Tc16, Arch::Tc131] {
                let got = bitmul::decode(w as u16, (w >> 16) as u16, CODE, arch);
                let want = !(arch == Arch::Tc131 && e.mn == "MULMS.H");
                assert_eq!(got.is_some(), want, "{} {} ({}) {w:08x} {arch:?}", e.mn, e.syn, e.page);
            }
        }
    }
    // Completeness: per op1, every op2 value outside the table is rejected; fixed n = 0H enforced.
    for (&op1, op2s) in &by_op1 {
        let e = ENC.iter().find(|e| e.op1 == op1).unwrap();
        let (lo, bits) = op2_field(e.fmt);
        for op2 in 0..1u32 << bits {
            for _ in 0..16 {
                let w = rng.u32() & !(((1u32 << bits) - 1) << lo) & !0xFF | op1 as u32 | op2 << lo;
                let w = if e.fmt == "SRR" { w & 0xFFFF } else { w };
                let some = bitmul::decode(w as u16, (w >> 16) as u16, CODE, Arch::Tc16).is_some();
                let fixed_n0 = matches!(op1, 0x4B | 0x6B);
                let want = op2s.contains(&(op2 as u16)) && (!fixed_n0 || (w >> 16) & 3 == 0);
                assert_eq!(some, want, "op1 {op1:02x} op2 {op2:x} word {w:08x}");
            }
        }
    }
}

#[test]
fn firmware_words_decode_as_rizin_names() {
    let mut bad = Vec::new();
    for &(img, mn, w) in FIRMWARE {
        let arch = if img == "SIMOS" { Arch::Tc16 } else { Arch::Tc131 };
        let decoded = bitmul::decode(w as u16, (w >> 16) as u16, CODE, arch).is_some();
        let op1 = w as u8;
        if matches!(op1, 0x4B | 0x6B) && (w >> 16) & 3 != 0 {
            // The boxes of BMERGE/BSPLIT/PARITY/IXMAX* fix n = 0H; capstone ignores the field (seen
            // only in data decoded as code). Rejected by the decoder.
            assert!(!decoded, "{img} {w:08x}");
            continue;
        }
        let manual = ENC.iter().find(|e| {
            let (lo, bits) = op2_field(e.fmt);
            e.op1 == op1 && (bits == 0 || (w >> lo) & ((1 << bits) - 1) == e.op2 as u32)
        });
        if !decoded || manual.map(|e| e.mn) != Some(mn) {
            bad.push(format!("{img} {w:08x} rizin {mn} manual {:?} decoded {decoded}", manual.map(|e| e.mn)));
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

#[test]
fn every_encoding_matches_manual_rtl() {
    let mut rng = Rng(0x2545_F491_4F6C_DD1D);
    for arch in [Arch::Tc16, Arch::Tc131] {
        let (code, slots) = layout(arch, 24, &mut rng);
        let mut cpu = native_cpu(&code, arch);
        for (k, &(i, w)) in slots.iter().enumerate() {
            let e = &ENC[i];
            let pc = CODE + 4 * k as u32;
            for _ in 0..48 {
                let (d, psw) = operands(e, w, &mut rng);
                let (nd, npsw, npc) = native_step(&mut cpu, pc, &d, psw);
                let (rd, rpsw) = reference(e, w, &d, psw);
                assert_eq!(npc, pc + len_of(w));
                assert!(
                    nd == rd && npsw == rpsw,
                    "{} {} (V2 p. {}) word {w:08x} {arch:?}\n in:  {}\n got: {}\n ref: {}",
                    e.mn,
                    e.syn,
                    e.page,
                    show(&d, psw),
                    show(&nd, npsw),
                    show(&rd, rpsw)
                );
            }
        }
    }
}

#[test]
fn unicorn_differential() {
    if !unicorn_tricore_sys::LINKED {
        return;
    }
    let mut rng = Rng(0xD1B5_4A32_D192_ED03);
    let (code, slots) = layout(Arch::Tc16, 16, &mut rng);
    let mut cpu = native_cpu(&code, Arch::Tc16);
    let mut uc = Uc::new(&code);
    // encoding index -> (cases, mismatches, first mismatch, unicorn errors)
    let mut stats: BTreeMap<usize, (u32, u32, Option<String>, u32)> = BTreeMap::new();
    for (k, &(i, w)) in slots.iter().enumerate() {
        if !simemu_tc1791::isa_fix::decode(w as u16, (w >> 16) as u16).is_none() {
            continue; // Unicorn defect documented in testutil::oracle::KNOWN_WRONG
        }
        let e = &ENC[i];
        if unicorn_aborts(e, w) {
            continue;
        }
        let pc = CODE + 4 * k as u32;
        if std::env::var("BITMUL_TRACE").is_ok() {
            eprintln!("{} {} {w:08x}", e.mn, e.syn);
        }
        for _ in 0..64 {
            let (d, psw) = operands(e, w, &mut rng);
            let (nd, npsw, npc) = native_step(&mut cpu, pc, &d, psw);
            let s = stats.entry(i).or_default();
            s.0 += 1;
            match uc.step(pc, &d, psw) {
                Err(_) => s.3 += 1,
                Ok((ud, upsw, upc)) => {
                    if ud != nd || upsw != npsw || upc != npc {
                        s.1 += 1;
                        if s.2.is_none() {
                            s.2 = Some(format!(
                                "word {w:08x}\n   in:      {}\n   native:  {} pc {npc:08x}\n   unicorn: {} pc {upc:08x}",
                                show(&d, psw),
                                show(&nd, npsw),
                                show(&ud, upsw)
                            ));
                        }
                    }
                }
            }
        }
    }
    let mut unexpected = Vec::new();
    let mut total = 0;
    for (i, (cases, bad, first, errs)) in &stats {
        let e = &ENC[*i];
        total += cases;
        let listed = UNICORN_WRONG.iter().find(|(m, s, _)| *m == e.mn && e.syn.starts_with(s));
        if *bad > 0 || *errs > 0 {
            println!(
                "{:<11} {:<36} {:>4}/{:<4} mismatches, {:>4} unicorn errors{}",
                e.mn,
                e.syn,
                bad,
                cases,
                errs,
                listed.map(|l| format!(" [listed: {}]", l.2)).unwrap_or_default()
            );
            if let Some(f) = first {
                println!("   {f}");
            }
            if listed.is_none() {
                unexpected.push(format!("{} {}", e.mn, e.syn));
            }
        }
    }
    println!("{total} cases over {} encodings", stats.len());
    assert!(unexpected.is_empty(), "unlisted Unicorn disagreements: {unexpected:#?}");
}
