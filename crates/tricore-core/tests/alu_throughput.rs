//! Interpreter throughput on a predecoded block of group-A instructions, mixed roughly like the
//! firmware census (`docs/tricore_core/insn_census.md`: mov, mov.u, movh.a, addsc.a, add, mov.aa, sha,
//! and, or, sh, sub, xor, ne, mov.a, add.a, eq, max, sat.h, cmov, ...).
//!
//! `cargo test -p tricore-core --release --test alu_throughput -- --ignored --nocapture`

use tricore_core::{Arch, Cpu, Exit, Memory, NullBus};

const CODE: u32 = 0xC000_0000;

/// 16 instructions (32-bit and 16-bit forms); the block repeats them to the 64-instruction limit.
const MIX: &[u32] = &[
    0x1002,      // mov d0, d1             (SRR 02H)
    0x8123_40BB, // mov.u d8, #0x1234      (RLC BBH)
    0xF800_6091, // movh.a a15, #0x8006    (RLC 91H)
    0x3130,      // add.a a1, a3           (SRR 30H)
    0x3600_2101, // addsc.a a3, a2, d1, #0 (RR 01H/60H)
    0x4242,      // add d2, d4             (SRR 42H)
    0x403F_A18F, // sha d4, d1, #-6        (RC 8FH/01H)
    0x5026,      // and d0, d5             (SRR 26H)
    0x5144_708F, // or d5, d0, #0x47       (RC 8FH/0AH)
    0x6A06,      // sh d10, #6             (SRC 06H)
    0x10A2,      // sub d0, d1             (SRR A2H)
    0x7110_200B, // ne d7, d0, d2          (RR 0BH/11H)
    0x81A0_A00B, // max d8, d0, d10        (RR 0BH/1AH)
    0x07E0_010B, // sat.h d0, d1           (RR 0BH/7EH)
    0x52AA,      // cmov d2, d15, #5       (SRC AAH)
    0x6182,      // mov d1, #6             (SRC 82H)
];

fn build() -> Vec<u8> {
    let mut b = Vec::new();
    while b.len() < 64 * 3 {
        for &w in MIX {
            if w & 1 == 1 {
                b.extend_from_slice(&w.to_le_bytes());
            } else {
                b.extend_from_slice(&(w as u16).to_le_bytes());
            }
        }
    }
    b
}

#[test]
#[ignore]
fn group_a_block_throughput() {
    let mut m = Memory::new();
    let code = m.add_owned(vec![0; 0x4000], true);
    m.map(CODE, code, 0, 0x4000);
    let mut c = Cpu::new(Arch::Tc16, m, NullBus);
    c.write_bytes(CODE, &build()).unwrap();
    let rounds = 2_000_000u64;
    let t = std::time::Instant::now();
    for _ in 0..rounds {
        c.pc = CODE;
        assert_eq!(c.run(64), Exit::Budget);
    }
    let dt = t.elapsed().as_secs_f64();
    assert_eq!(c.icount, rounds * 64);
    assert_eq!(c.cached_blocks(), 1, "one predecoded 64-instruction block");
    eprintln!("group A block: {:.0} MIPS ({} instructions in {dt:.3} s)", c.icount as f64 / dt / 1e6, c.icount);
}
