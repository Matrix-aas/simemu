//! PCP2 — Peripheral Control Processor interpreter (TC1791 / SIMOS18.1).
//!
//! Sources (page labels are the manual's own, e.g. "p.11-89"):
//! * Semantics: TC1796 User's Manual V2.0 (2007-07), chapter 11 "Peripheral Control Processor"
//!   (`vault/docs/infineon/tc1796_um_v2.0_2007_07.pdf`). The TC1791 data sheet (p.8) states the same PCP2 core with
//!   16 KB PRAM and 32 KB CMEM; the firmware programs the TC1796 register layout (PCP_CS = 0x21 at
//!   0xF0043F10, PCP_ICR at 0xF0043F20, VERIFIED-T).
//! * Binary encoding: the UM does not list opcodes. Encodings follow the HighTec/GNU binutils TriCore port
//!   (`opcodes/tricore-opc.c` `pcp_opcodes[]` + `tricore-dis.c` `decode_pcp_insn`). Cross-check: the complete
//!   SIMOS18.1 PCP image (copy table 0x80125B00, 6 CMEM segments, 5165 instruction slots) decodes with zero
//!   invalid opcodes and every direct branch target lands inside a copied segment (`tests/pcp_gpta_crank.rs`).
//!
//! Integration: the board owns the SoC service request nodes. PCP-routed requests (TOS = PCP) reach the
//! interpreter through [`FpiBus::pcp_pending`]/[`FpiBus::pcp_ack`]; every FPI access of a channel program
//! (`.F`/`.IF`/COPY) goes through [`FpiBus::fpi_read`]/[`FpiBus::fpi_write`], except accesses to the PCP's own
//! PRAM/CMEM, which are served internally. TriCore accesses to `0xF0043F00..0xF0043FFF`,
//! `0xF0050000..0xF0053FFF`, `0xF0060000..0xF0067FFF` ([`Pcp::contains`]) go to [`Pcp::read_word`]/
//! [`Pcp::write_lanes`] (board `periph_read`/`periph_write` convention) or [`Pcp::mmio_read`]/[`Pcp::mmio_write`].
//! The board runs [`Pcp::run`] after each time quantum with the elapsed PCP clock cycles as budget.

use std::fmt;

/// PCP register block (TC1796 UM Table 11-16, p.11-125).
pub const PCP_REGS: u32 = 0xF004_3F00;
/// Parameter RAM (p.11-125; 16 KB on TC1791 per data sheet p.8).
pub const PRAM_BASE: u32 = 0xF005_0000;
pub const PRAM_SIZE: usize = 0x4000;
/// Code memory (p.11-125; 32 KB on TC1791 per data sheet p.8).
pub const CMEM_BASE: u32 = 0xF006_0000;
pub const CMEM_SIZE: usize = 0x8000;

/// FPI bus as seen by the PCP (implemented by the board).
pub trait FpiBus {
    /// Read `size` bytes (1, 2 or 4) at `addr`; result zero-extended.
    fn fpi_read(&mut self, addr: u32, size: u8) -> u32;
    /// Write the low `size` bytes of `val` at `addr`.
    fn fpi_write(&mut self, addr: u32, val: u32, size: u8);
    /// Fallible FPI read (`None` raises `PCP_ES.BER`, TC1796 UM §11.4.4 p.11-39).
    fn fpi_try_read(&mut self, addr: u32, size: u8) -> Option<u32> {
        Some(self.fpi_read(addr, size))
    }
    /// Fallible FPI write (`false` raises `PCP_ES.BER`, TC1796 UM §11.4.4 p.11-39).
    fn fpi_try_write(&mut self, addr: u32, val: u32, size: u8) -> bool {
        self.fpi_write(addr, val, size);
        true
    }
    /// PCP interrupt arbitration bus (p.11-33): highest SRPN among the SoC service request nodes with
    /// TOS = PCP, SRE = 1 and SRR = 1 (not cleared).
    fn pcp_pending(&mut self) -> Option<u8>;
    /// Acknowledge of the winning node with this SRPN: clear its SRR (p.11-33).
    fn pcp_ack(&mut self, srpn: u8);
}

// ------------------------------------------------------------------------------------------------
// Decoder
// ------------------------------------------------------------------------------------------------

/// Group 1 (`.F`, FPI indirect) operations, bits `[12:9]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FOp { Add, Sub, Comp, And, Or, Xor, Ld, St, Xch }
/// Group 2 (`.PI`, PRAM DPTR + offset6) operations, bits `[12:9]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PiOp { Add, Sub, Comp, Mclr, And, Mset, Or, Xor, Ld, St, Xch }
/// Group 3 (register, conditional on cc_A) operations, bits `[12:9]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ROp { Add, Sub, Comp, Neg, Not, And, Or, Xor, LdP, StP, Mov, Inb, Pri }
/// Group 4 (immediate) operations, bits `[12:9]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IOp { Add, Sub, Comp, Shr, Shl, Rr, Rl, Set, Clr, Ld, InbI, Chkb }
/// Group 5 (FPI + immediate) operations, bits `[12:10]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FiOp { SetF, ClrF, LdIf, StIf }
/// Group 6 (complex maths) operations, bits `[12:9]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MOp { Dinit, Dstep, Minit, Mstep32, Mstep64 }

/// One decoded PCP2 instruction. `size` fields are the encoded SIZE code (0 = byte, 1 = half, 2 = word).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Insn {
    Nop,
    Copy { dst: u8, src: u8, cnc: u8, cnt0: u8, size: u8 },
    Bcopy { dst: u8, src: u8, cnc: u8, cnt0: u8 },
    Exit { st: bool, int: bool, ep: bool, ec: bool, cc: u8 },
    F { op: FOp, rb: u8, ra: u8, size: u8 },
    Pi { op: PiOp, ra: u8, off: u8 },
    R { op: ROp, rb: u8, ra: u8, cc: u8 },
    I { op: IOp, ra: u8, imm: u8 },
    Ldl { upper: bool, ra: u8, imm: u16 },
    Fi { op: FiOp, ra: u8, imm: u8, size: u8 },
    M { op: MOp, rb: u8, ra: u8 },
    Jl { off: i16 },
    Jc { off: i8, cc: u8 },
    JcA { addr: u16, cc: u8 },
    JcI { ra: u8, cc: u8 },
    JcIa { ra: u8, cc: u8 },
    Debug { dac: bool, rta: bool, eda: bool, sdb: bool, cc: u8 },
}

impl Insn {
    /// Length in 16-bit slots.
    pub fn len(&self) -> u16 {
        matches!(self, Insn::Ldl { .. } | Insn::JcA { .. }) as u16 + 1
    }

    pub fn is_empty(&self) -> bool {
        false
    }
}

/// Decode the instruction word `w` (and the following word `w2` for 32-bit forms).
/// `None` = invalid opcode (IOP error, p.11-39).
pub fn decode(w: u16, w2: u16) -> Option<Insn> {
    let b = |n: u16| (w >> n) & 1 != 0;
    let rb = ((w >> 6) & 7) as u8;
    let ra = ((w >> 3) & 7) as u8;
    let op4 = (w >> 9) & 0xF;
    Some(match w >> 13 {
        0 => match (w >> 11) & 3 {
            _ if w == 0 => Insn::Nop,
            // COPY: dst [10:9], src [8:7], cnc [6:5], cnt0 [4:2], size [1:0]
            1 => {
                let (d, s, c, z) = ((w >> 9) & 3, (w >> 7) & 3, (w >> 5) & 3, w & 3);
                if d == 3 || s == 3 || c == 3 || z == 3 {
                    return None;
                }
                Insn::Copy { dst: d as u8, src: s as u8, cnc: c as u8, cnt0: ((w >> 2) & 7) as u8, size: z as u8 }
            }
            // EXIT: st [10], int [9], ep [8], ec [7], cc_B [3:0]; [6:4] must be 0
            2 => {
                if w & 0x0070 != 0 {
                    return None;
                }
                Insn::Exit { st: b(10), int: b(9), ep: b(8), ec: b(7), cc: (w & 0xF) as u8 }
            }
            // BCOPY: dst [10:9], src [8:7], cnc [6:5], cnt0 [3:2]; [4],[1:0] must be 0
            3 => {
                let (d, s, c, n) = ((w >> 9) & 3, (w >> 7) & 3, (w >> 5) & 3, (w >> 2) & 3);
                if w & 0x0013 != 0 || d == 3 || s == 3 || c == 3 || n == 1 {
                    return None;
                }
                Insn::Bcopy { dst: d as u8, src: s as u8, cnc: c as u8, cnt0: n as u8 }
            }
            _ => return None,
        },
        1 => {
            let op = match op4 {
                0 => FOp::Add, 1 => FOp::Sub, 2 => FOp::Comp, 5 => FOp::And, 7 => FOp::Or,
                8 => FOp::Xor, 9 => FOp::Ld, 0xA => FOp::St, 0xB => FOp::Xch,
                _ => return None,
            };
            if b(2) || w & 3 == 3 {
                return None;
            }
            Insn::F { op, rb, ra, size: (w & 3) as u8 }
        }
        2 => {
            let op = match op4 {
                0 => PiOp::Add, 1 => PiOp::Sub, 2 => PiOp::Comp, 4 => PiOp::Mclr, 5 => PiOp::And,
                6 => PiOp::Mset, 7 => PiOp::Or, 8 => PiOp::Xor, 9 => PiOp::Ld, 0xA => PiOp::St,
                0xB => PiOp::Xch,
                _ => return None,
            };
            Insn::Pi { op, ra: rb, off: (w & 0x3F) as u8 }
        }
        3 => {
            let op = match op4 {
                0 => ROp::Add, 1 => ROp::Sub, 2 => ROp::Comp, 3 => ROp::Neg, 4 => ROp::Not,
                5 => ROp::And, 7 => ROp::Or, 8 => ROp::Xor, 9 => ROp::LdP, 0xA => ROp::StP,
                0xC => ROp::Mov, 0xD => ROp::Inb, 0xE => ROp::Pri,
                _ => return None,
            };
            Insn::R { op, rb, ra, cc: (w & 7) as u8 }
        }
        4 => {
            let imm = (w & 0x3F) as u8;
            if op4 == 8 || op4 == 9 {
                if imm != 0 {
                    return None;
                }
                return Some(Insn::Ldl { upper: op4 == 8, ra: rb, imm: w2 });
            }
            let op = match op4 {
                0 => IOp::Add, 1 => IOp::Sub, 2 => IOp::Comp, 4 => IOp::Shr, 5 => IOp::Shl,
                6 => IOp::Rr, 7 => IOp::Rl, 0xA => IOp::Set, 0xB => IOp::Clr, 0xC => IOp::Ld,
                0xD => IOp::InbI, 0xE => IOp::Chkb,
                _ => return None,
            };
            // imm5 forms: bit 5 must be 0 (CHKB uses bit 5 as S/C)
            if matches!(op, IOp::Shr | IOp::Shl | IOp::Rr | IOp::Rl | IOp::Set | IOp::Clr | IOp::InbI) && imm & 0x20 != 0 {
                return None;
            }
            Insn::I { op, ra: rb, imm }
        }
        5 => {
            let op = match (w >> 10) & 7 {
                3 => FiOp::SetF, 4 => FiOp::ClrF, 5 => FiOp::LdIf, 6 => FiOp::StIf,
                _ => return None,
            };
            let size = (((w >> 5) & 1) | ((w >> 8) & 2)) as u8;
            if size == 3 {
                return None;
            }
            Insn::Fi { op, ra: rb, imm: (w & 0x1F) as u8, size }
        }
        6 => {
            let op = match op4 {
                0 => MOp::Dinit, 1 => MOp::Dstep, 2 => MOp::Minit, 3 => MOp::Mstep32, 4 => MOp::Mstep64,
                _ => return None,
            };
            if w & 7 != 0 {
                return None;
            }
            Insn::M { op, rb, ra }
        }
        _ => {
            let cc = ((w >> 6) & 0xF) as u8;
            match (w >> 10) & 7 {
                0 => Insn::Jl { off: (((w & 0x3FF) as i16) << 6) >> 6 },
                1 => Insn::Jc { off: (((w & 0x3F) as i8) << 2) >> 2, cc },
                2 if w & 0x3F == 0 => Insn::JcA { addr: w2, cc },
                4 if w & 7 == 0 => Insn::JcI { ra, cc },
                5 if w & 7 == 0 => Insn::JcIa { ra, cc },
                7 if w & 0x30 == 0 => Insn::Debug { dac: b(3), rta: b(2), eda: b(1), sdb: b(0), cc },
                _ => return None,
            }
        }
    })
}

const CC: [&str; 16] = ["uc", "z", "nz", "v", "ult", "ugt", "slt", "sgt", "n", "nn", "nv", "uge", "sge", "sle", "cnz", "cnn"];
const PM: [&str; 3] = ["", "+", "-"];
const SZ: [u32; 3] = [8, 16, 32];

/// Disassemble the instruction at halfword address `pc` (jump targets are absolute halfword PCs).
pub fn disasm(pc: u16, w: u16, w2: u16) -> String {
    match decode(w, w2) {
        None => format!(".hword {w:#06x}"),
        Some(i) => Disasm { pc, i }.to_string(),
    }
}

struct Disasm {
    pc: u16,
    i: Insn,
}

impl fmt::Display for Disasm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let next = self.pc as i32 + 1;
        let lc = |s: &str| s.to_ascii_lowercase();
        match self.i {
            Insn::Nop => write!(f, "nop"),
            Insn::Copy { dst, src, cnc, cnt0, size } => write!(f, "copy dst{}, src{}, cnc={cnc}, cnt0={}, size={}", PM[dst as usize], PM[src as usize], if cnt0 == 0 { 8 } else { cnt0 }, SZ[size as usize]),
            Insn::Bcopy { dst, src, cnc, cnt0 } => write!(f, "bcopy dst{}, src{}, cnc={cnc}, cnt0={cnt0}", PM[dst as usize], PM[src as usize]),
            Insn::Exit { st, int, ep, ec, cc } => write!(f, "exit st={}, int={}, ep={}, ec={}, cc_{}", st as u8, int as u8, ep as u8, ec as u8, CC[cc as usize]),
            Insn::F { op, rb, ra, size } => write!(f, "{}.f r{rb}, [r{ra}], size={}", lc(&format!("{op:?}")), SZ[size as usize]),
            Insn::Pi { op, ra, off } => write!(f, "{}.pi r{ra}, [{off:#x}]", lc(&format!("{op:?}"))),
            Insn::R { op: ROp::LdP, rb, ra, cc } => write!(f, "ld.p r{rb}, [r{ra}], cc_{}", CC[cc as usize]),
            Insn::R { op: ROp::StP, rb, ra, cc } => write!(f, "st.p r{rb}, [r{ra}], cc_{}", CC[cc as usize]),
            Insn::R { op, rb, ra, cc } => write!(f, "{} r{rb}, r{ra}, cc_{}", lc(&format!("{op:?}")), CC[cc as usize]),
            Insn::I { op: IOp::Chkb, ra, imm } => write!(f, "chkb r{ra}, {}, {}", imm & 0x1F, if imm & 0x20 != 0 { "set" } else { "clr" }),
            Insn::I { op, ra, imm } => {
                let n = match op {
                    IOp::Add => "add.i", IOp::Sub => "sub.i", IOp::Comp => "comp.i", IOp::Ld => "ld.i", IOp::InbI => "inb.i",
                    IOp::Shr => "shr", IOp::Shl => "shl", IOp::Rr => "rr", IOp::Rl => "rl", IOp::Set => "set", IOp::Clr => "clr",
                    IOp::Chkb => unreachable!(),
                };
                write!(f, "{n} r{ra}, {imm:#x}")
            }
            Insn::Ldl { upper, ra, imm } => write!(f, "ldl.{} r{ra}, {imm:#06x}", if upper { "iu" } else { "il" }),
            Insn::Fi { op, ra, imm, size } => {
                let n = match op { FiOp::SetF => "set.f", FiOp::ClrF => "clr.f", FiOp::LdIf => "ld.if", FiOp::StIf => "st.if" };
                write!(f, "{n} [r{ra}], {imm:#x}, size={}", SZ[size as usize])
            }
            Insn::M { op, rb, ra } => write!(f, "{} r{rb}, r{ra}", lc(&format!("{op:?}"))),
            Insn::Jl { off } => write!(f, "jl {:#06x}", next + off as i32),
            Insn::Jc { off, cc } => write!(f, "jc {:#06x}, cc_{}", next + off as i32, CC[cc as usize]),
            Insn::JcA { addr, cc } => write!(f, "jc.a {addr:#06x}, cc_{}", CC[cc as usize]),
            Insn::JcI { ra, cc } => write!(f, "jc.i r{ra}, cc_{}", CC[cc as usize]),
            Insn::JcIa { ra, cc } => write!(f, "jc.ia r{ra}, cc_{}", CC[cc as usize]),
            Insn::Debug { dac, rta, eda, sdb, cc } => write!(f, "debug dac={}, rta={}, eda={}, sdb={}, cc_{}", dac as u8, rta as u8, eda as u8, sdb as u8, CC[cc as usize]),
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Register fields
// ------------------------------------------------------------------------------------------------

// R7 (p.11-10)
const Z: u32 = 1 << 0;
const N: u32 = 1 << 1;
const C: u32 = 1 << 2;
const V: u32 = 1 << 3;
const CNZ: u32 = 1 << 4;
const IEN: u32 = 1 << 5;
const CEN: u32 = 1 << 6;
// PCP_CS (p.11-56..58)
const CS_EN: u32 = 1 << 0;
const CS_RST: u32 = 1 << 1;
const CS_RS: u32 = 1 << 2;
const CS_RCB: u32 = 1 << 4;
const CS_PPE: u32 = 1 << 8;
const CS_CWE: u32 = 1 << 16;
// PCP_ES (p.11-59)
const ES_BER: u32 = 1 << 0;
const ES_IOP: u32 = 1 << 1;
const ES_DCR: u32 = 1 << 2;
const ES_IAE: u32 = 1 << 3;
const ES_DBE: u32 = 1 << 4;
const ES_CWD: u32 = 1 << 6;
const ES_PPC: u32 = 1 << 7;
// PCP_SRCx (p.11-67..71)
const SRC_SRE: u32 = 1 << 12;
const SRC_SRR: u32 = 1 << 13;
const SRC_RRQ: u32 = 1 << 28;

/// Statistics for tests and the integrating board.
#[derive(Debug, Clone)]
pub struct PcpStats {
    /// Channel invocations per SRPN (new requests and resumptions).
    pub starts: Vec<u64>,
    pub instructions: u64,
    pub cycles: u64,
    pub errors: u64,
    pub suspends: u64,
}

impl Default for PcpStats {
    fn default() -> Self {
        Self { starts: vec![0; 256], instructions: 0, cycles: 0, errors: 0, suspends: 0 }
    }
}

/// What a channel start was triggered by.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Req {
    /// New request with SRPN; `internal` = one of the PCP's own nodes (index).
    New { srpn: u8, internal: Option<usize> },
    /// Resumption of a suspended channel from extended node `node` (p.11-20).
    Resume { node: usize },
}

/// The PCP2 core with its memories and register block.
pub struct Pcp {
    /// PRAM as 32-bit words (FPI-visible little-endian at `PRAM_BASE`).
    pub pram: Vec<u32>,
    /// CMEM as 16-bit instruction slots (FPI-visible little-endian at `CMEM_BASE`).
    pub cmem: Vec<u16>,
    pub clc: u32,
    pub cs: u32,
    pub es: u32,
    pub icr: u32,
    pub itr: u32,
    pub ssr: u32,
    /// PCP_SRC0..11 (p.11-67..71).
    pub src: [u32; 12],
    /// R0..R7.
    pub r: [u32; 8],
    /// Program counter (halfword address in CMEM).
    pub pc: u16,
    /// Channel currently executing (SRPN of the invoking request = channel number).
    pub channel: Option<u8>,
    /// Channel whose context is still in the GPRs (context-load optimisation, p.11-24 / p.11-28).
    last: Option<u8>,
    /// Registers written since the last restore (context-save optimisation, p.11-24).
    dirty: u8,
    /// Instructions fetched since channel start (channel watchdog, p.11-39).
    fetched: u32,
    /// Pre-decoded CMEM cache: `(w, w2, decode(w, w2))` per 16-bit slot.
    decoded: Vec<(u16, u16, Option<Insn>)>,
    /// True when arbitration state may have changed since the last suspension check.
    irq_dirty: bool,
    /// True when CPU-targeted `src` nodes `[0, 1, 4..=8]` may have changed since the last ICU sync.
    pub(crate) cpu_irq_dirty: bool,
    /// DEBUG instruction allowed (PCP in debug mode, p.11-50). Off: DEBUG is an IOP error.
    pub debug_mode: bool,
    pub stats: PcpStats,
    /// Trace hook: (channel, pc, instruction word) of every executed instruction when enabled.
    pub trace: Option<Vec<(u8, u16, u16)>>,
}

impl Default for Pcp {
    fn default() -> Self {
        Self::new()
    }
}

impl Pcp {
    pub fn new() -> Self {
        let mut p = Self {
            pram: vec![0; PRAM_SIZE / 4],
            cmem: vec![0; CMEM_SIZE / 2],
            clc: 0,
            cs: 0,
            es: 0,
            icr: 0,
            itr: 0,
            ssr: 0,
            src: [0; 12],
            r: [0; 8],
            pc: 0,
            channel: None,
            last: None,
            dirty: 0,
            fetched: 0,
            decoded: vec![(0, 0, Some(Insn::Nop)); CMEM_SIZE / 2],
            irq_dirty: true,
            cpu_irq_dirty: true,
            debug_mode: false,
            stats: PcpStats::default(),
            trace: None,
        };
        p.reset_srcs();
        p
    }

    fn reset_srcs(&mut self) {
        self.cpu_irq_dirty = true;
        // Reset values p.11-67..70: SRC0/1/4..8 = 0x1000, SRC2/3/9..11 = 0x1400.
        for (i, s) in self.src.iter_mut().enumerate() {
            *s = if matches!(i, 2 | 3 | 9..=11) { 0x1400 } else { 0x1000 };
        }
    }

    // --------------------------------------------------------------------------------------------
    // FPI slave side (TriCore / DMA accesses)
    // --------------------------------------------------------------------------------------------

    /// True if `addr` belongs to the PCP (register block, PRAM or CMEM).
    pub fn contains(addr: u32) -> bool {
        (PCP_REGS..PCP_REGS + 0x100).contains(&addr)
            || (PRAM_BASE..PRAM_BASE + PRAM_SIZE as u32).contains(&addr)
            || (CMEM_BASE..CMEM_BASE + CMEM_SIZE as u32).contains(&addr)
    }

    fn word_read(&self, a: u32) -> u32 {
        if (PRAM_BASE..PRAM_BASE + PRAM_SIZE as u32).contains(&a) {
            self.pram[((a - PRAM_BASE) >> 2) as usize]
        } else if (CMEM_BASE..CMEM_BASE + CMEM_SIZE as u32).contains(&a) {
            let i = ((a - CMEM_BASE) >> 1) as usize & !1;
            self.cmem[i] as u32 | (self.cmem[i + 1] as u32) << 16
        } else if (PCP_REGS..PCP_REGS + 0x100).contains(&a) {
            self.reg_read(a - PCP_REGS)
        } else {
            0
        }
    }

    /// Read from the PCP address space. The UM requires 32-bit accesses (p.11-48/49); narrower and
    /// cross-word reads return the addressed lanes.
    pub fn mmio_read(&self, addr: u32, size: u8) -> u32 {
        let a = addr & !3;
        let boff = addr & 3;
        let w0 = self.word_read(a);
        let sh = boff * 8;
        let m = lane_mask(size);
        if size <= 4 && boff + size as u32 > 4 {
            let w1 = self.word_read(a.wrapping_add(4));
            ((w0 >> sh) | (w1 << ((4 - boff) * 8))) & m
        } else {
            (w0 >> sh) & m
        }
    }

    fn mmio_write_word(&mut self, a: u32, v: u32, m: u32) {
        if (PRAM_BASE..PRAM_BASE + PRAM_SIZE as u32).contains(&a) {
            let w = &mut self.pram[((a - PRAM_BASE) >> 2) as usize];
            *w = (*w & !m) | v;
            if self.channel.is_none() {
                // External CPU write to PRAM may alter a channel's CSA: force reload on next start.
                self.last = None;
            }
        } else if (CMEM_BASE..CMEM_BASE + CMEM_SIZE as u32).contains(&a) {
            let i = ((a - CMEM_BASE) >> 1) as usize;
            let old = self.cmem[i] as u32 | (self.cmem[i + 1] as u32) << 16;
            let n = (old & !m) | v;
            self.cmem[i] = n as u16;
            self.cmem[i + 1] = (n >> 16) as u16;
        } else if (PCP_REGS..PCP_REGS + 0x100).contains(&a) {
            let old = self.reg_read(a - PCP_REGS);
            self.reg_write(a - PCP_REGS, (old & !m) | v);
        }
    }

    /// Write to the PCP address space (lane-aware merge for narrow and cross-word writes).
    pub fn mmio_write(&mut self, addr: u32, val: u32, size: u8) {
        let a = addr & !3;
        let boff = addr & 3;
        let sh = boff * 8;
        let lane = lane_mask(size);
        let val32 = val & lane;
        let m0 = lane << sh;
        let v0 = (val32 << sh) & m0;
        self.mmio_write_word(a, v0, m0);
        if size <= 4 && boff + size as u32 > 4 {
            let hi_bytes = boff + size as u32 - 4;
            let m1 = (1u32 << (hi_bytes * 8)) - 1;
            let v1 = (val32 >> ((4 - boff) * 8)) & m1;
            self.mmio_write_word(a.wrapping_add(4), v1, m1);
        }
    }

    /// Board convention (`cpu_hooks::periph_read`): 32-bit aligned address -> word.
    pub fn read_word(&self, addr: u32) -> Option<u32> {
        Self::contains(addr).then(|| self.word_read(addr & !3))
    }

    /// Board convention (`cpu_hooks::periph_write`): aligned address, lane-shifted value, byte-lane mask.
    pub fn write_lanes(&mut self, addr: u32, v: u32, m: u32) -> bool {
        if !Self::contains(addr) {
            return false;
        }
        let a = addr & !3;
        let old = self.word_read(a);
        self.mmio_write(a, (old & !m) | (v & m), 4);
        true
    }

    fn reg_read(&self, off: u32) -> u32 {
        match off {
            0x00 => self.clc,
            0x08 => 0x0020_C000, // PCP_ID MODNUM 0x0020, MODTYPE 0xC0 (p.11-54); MODREV unknown
            0x10 => (self.cs & !(CS_RST | CS_RS)) | if self.channel.is_some() { CS_RS } else { 0 },
            0x14 => self.es,
            0x20 => self.icr,
            0x24 => self.itr,
            0x28 => 0x0000_03E4, // PCP_ICON fixed (p.11-64)
            0x2C => self.ssr,
            0xD0..=0xFC if off & 3 == 0 => self.src[((0xFC - off) / 4) as usize],
            _ => 0,
        }
    }

    fn reg_write(&mut self, off: u32, v: u32) {
        self.irq_dirty = true;
        self.cpu_irq_dirty = true;
        match off {
            0x00 => self.clc = v & 0x8000,
            0x10 => {
                if v & CS_RST != 0 {
                    self.soft_reset();
                } else {
                    self.cs = v & !(CS_RST | CS_RS | 1 << 3);
                }
            }
            // ICR: only PARBCYC/PONECYC are writable (p.11-61/62)
            0x20 => self.icr = (self.icr & !0x0700_0000) | (v & 0x0700_0000),
            0x24 => self.itr = v & 0x000F_00FF,
            // SRC4..8: TOS programmable at configuration time (p.11-69)
            0xDC..=0xEC if off & 3 == 0 => {
                let i = ((0xFC - off) / 4) as usize;
                self.src[i] = (self.src[i] & !0x0C00) | (v & 0x0C00);
            }
            _ => {}
        }
    }

    /// PCP_CS.RST soft reset (p.11-126): halt channel, reset control registers, clear SRR/RRQ only.
    fn soft_reset(&mut self) {
        self.cpu_irq_dirty = true;
        self.channel = None;
        self.last = None;
        self.cs = 0;
        self.es = 0;
        self.icr = 0;
        self.itr = 0;
        self.ssr = 0;
        for s in &mut self.src {
            *s &= !(SRC_SRR | SRC_RRQ | 0x00FF_00FF);
        }
    }

    // --------------------------------------------------------------------------------------------
    // Service request queues (p.11-33..37)
    // --------------------------------------------------------------------------------------------

    fn node_tos(&self, i: usize) -> u32 {
        match i {
            0 | 1 => 0,
            2 | 3 | 9..=11 => 1,
            _ => (self.src[i] >> 10) & 3,
        }
    }

    /// Post an implicit request to the queue for `tos` (0 = CPU, 1 = PCP). Returns false when the queue is
    /// full (the PCP stalls, p.11-36).
    fn post(&mut self, tos: u32, srpn: u8) -> bool {
        self.irq_dirty = true;
        if tos == 0 {
            self.cpu_irq_dirty = true;
        }
        let order: &[usize] = if tos == 0 { &[0, 1, 4, 5, 6, 7, 8] } else { &[2, 3, 4, 5, 6, 7, 8, 9, 10, 11] };
        for &i in order {
            if self.node_tos(i) == tos && self.src[i] & SRC_SRR == 0 {
                self.src[i] = (self.src[i] & !0x00FF_00FF) | SRC_SRR | srpn as u32;
                if i >= 9 {
                    self.src[i] |= (srpn as u32) << 16; // SRCN = SRPN for EXIT requests (p.11-71)
                }
                return true;
            }
        }
        self.ssr = 0x8000 | (self.channel.unwrap_or(0) as u32) << 16 | tos << 8 | srpn as u32;
        false
    }

    /// Highest pending implicit request of the PCP itself to the CPU (TOS = 0): `(node, SRPN)`.
    /// The board's ICU arbitrates it like any SRN and calls [`Pcp::ack_cpu_request`] when taken.
    pub fn pending_cpu_request(&self) -> Option<(usize, u8)> {
        (0..12)
            .filter(|&i| self.node_tos(i) == 0 && self.src[i] & (SRC_SRR | SRC_SRE) == SRC_SRR | SRC_SRE)
            .map(|i| (i, self.src[i] as u8))
            .max_by_key(|&(_, p)| p)
    }

    pub fn ack_cpu_request(&mut self, node: usize) {
        self.cpu_irq_dirty = true;
        self.src[node] &= !SRC_SRR;
    }

    /// Winner on the PCP bus among the PCP's own nodes: `(node, SRPN)`.
    fn internal_pending(&self) -> Option<(usize, u8)> {
        if (self.src[2] | self.src[3] | self.src[4] | self.src[5] | self.src[6] | self.src[7] | self.src[8] | self.src[9] | self.src[10] | self.src[11]) & SRC_SRR == 0 {
            return None;
        }
        (2..12)
            .filter(|&i| self.node_tos(i) == 1 && self.src[i] & SRC_SRR != 0)
            .map(|i| (i, self.src[i] as u8))
            .max_by_key(|&(_, p)| p)
    }

    /// PICU arbitration (p.11-33): highest SRPN among external and internal PCP-bus requests.
    /// When the internal PCP queue is full only internal nodes take part (p.11-36).
    fn arbitrate<B: FpiBus>(&mut self, bus: &mut B) -> Option<(u8, Req)> {
        let int = self.internal_pending();
        let ext = if int.is_some() && [2usize, 3, 9, 10, 11].iter().all(|&i| self.src[i] & SRC_SRR != 0) {
            None
        } else {
            bus.pcp_pending()
        };
        let pick_int = match (int, ext) {
            (Some((_, pi)), Some(pe)) => pi >= pe,
            (Some(_), None) => true,
            (None, Some(_)) => false,
            (None, None) => return None,
        };
        if pick_int {
            let (i, p) = int.unwrap();
            if self.src[i] & SRC_RRQ != 0 {
                Some((p, Req::Resume { node: i }))
            } else {
                Some((p, Req::New { srpn: p, internal: Some(i) }))
            }
        } else {
            let p = ext.unwrap();
            Some((p, Req::New { srpn: p, internal: None }))
        }
    }

    fn take(&mut self, req: Req, bus: &mut impl FpiBus) {
        match req {
            Req::New { srpn, internal: None } => bus.pcp_ack(srpn),
            Req::New { internal: Some(i), .. } | Req::Resume { node: i } => self.src[i] &= !(SRC_SRR | SRC_RRQ),
        }
    }

    // --------------------------------------------------------------------------------------------
    // Contexts (p.11-11..24)
    // --------------------------------------------------------------------------------------------

    /// Number of registers in the selected context model (PCP_CS.CS, p.11-57).
    fn ctx_regs(&self) -> usize {
        match (self.cs >> 6) & 3 {
            0 => 8,
            1 => 4,
            _ => 2,
        }
    }

    /// PRAM word index of CRk for `ch`. The CSA region of a channel holds CR7 at the lowest address and
    /// CR(8-n) at the highest (Figure 11-3, p.11-16; VERIFIED-T: SIMOS18 contexts have PC|DPTR|flags in word 0).
    fn cr_index(&self, ch: u8, k: usize) -> usize {
        ch as usize * self.ctx_regs() + (7 - k)
    }

    fn restore(&mut self, ch: u8) {
        let first = 8 - self.ctx_regs();
        for k in first..8 {
            let i = self.cr_index(ch, k);
            self.r[k] = self.pram.get(i).copied().unwrap_or(0);
        }
        self.r[7] &= 0xFFFF;
    }

    fn save(&mut self, ch: u8, pc: u16) {
        let first = 8 - self.ctx_regs();
        for k in first..7 {
            if self.dirty & (1 << k) != 0 {
                let i = self.cr_index(ch, k);
                if let Some(w) = self.pram.get_mut(i) {
                    *w = self.r[k];
                }
            }
        }
        let i = self.cr_index(ch, 7);
        if let Some(w) = self.pram.get_mut(i) {
            *w = (pc as u32) << 16 | (self.r[7] & 0xFFFF);
        }
        self.last = Some(ch);
        self.channel = None;
        self.pc = pc;
    }

    /// Context restore + checks for a channel start (p.11-17..20, p.11-28).
    fn start(&mut self, req: Req) {
        let (ch, cppn) = match req {
            Req::New { srpn, .. } => {
                // PRAM partitioning check (p.11-38): MCN = 2*PPS.
                if self.cs & CS_PPE != 0 && srpn as u32 > 2 * ((self.cs >> 9) & 0x7F) {
                    self.channel = Some(srpn);
                    self.error(ES_PPC);
                    return;
                }
                if self.last != Some(srpn) {
                    self.restore(srpn);
                    self.pc = (self.pram.get(self.cr_index(srpn, 7)).copied().unwrap_or(0) >> 16) as u16;
                }
                if self.cs & CS_RCB != 0 {
                    self.pc = 2 * srpn as u16;
                }
                (srpn, (self.r[6] >> 24) as u8)
            }
            Req::Resume { node } => {
                let ch = (self.src[node] >> 16) as u8;
                if self.last != Some(ch) {
                    self.restore(ch);
                    self.pc = (self.pram.get(self.cr_index(ch, 7)).copied().unwrap_or(0) >> 16) as u16;
                }
                (ch, self.src[node] as u8)
            }
        };
        self.icr = (self.icr & !0xFF) | cppn as u32;
        self.channel = Some(ch);
        self.last = Some(ch);
        self.dirty = 0;
        self.fetched = 0;
        self.irq_dirty = true;
        self.stats.starts[ch as usize] += 1;
        if self.r[7] & CEN == 0 {
            self.error(ES_DCR);
        }
    }

    /// Error exit (p.11-30): disable channel, record PC/channel/cause, save context, CPU request.
    fn error(&mut self, cause: u32) {
        let ch = self.channel.unwrap_or(0);
        self.stats.errors += 1;
        self.set_r(7, self.r[7] & !CEN);
        self.es = (self.pc as u32) << 16 | (ch as u32) << 8 | cause;
        let pc = self.pc;
        self.save(ch, pc);
        let esr = (self.cs >> 24) as u8;
        if cause != ES_BER && esr != 0 {
            self.post(0, esr);
        }
    }

    // --------------------------------------------------------------------------------------------
    // Execution
    // --------------------------------------------------------------------------------------------

    /// Run for up to `budget` PCP clock cycles (cycle counts per Table 11-15, p.11-108..111).
    /// Returns the cycles consumed (0 if the PCP is idle with no pending request).
    pub fn run<B: FpiBus>(&mut self, budget: u64, bus: &mut B) -> u64 {
        let mut used = 0u64;
        self.irq_dirty = true;
        while used < budget {
            if self.channel.is_none() {
                if self.cs & CS_EN == 0 {
                    break;
                }
                let Some((_, req)) = self.arbitrate(bus) else { break };
                self.take(req, bus);
                self.start(req);
                used += self.exit_cycles();
                continue;
            }
            // Suspension in favour of a higher-priority request (p.11-28, p.11-35).
            if self.r[7] & IEN != 0 && self.irq_dirty {
                self.irq_dirty = false;
                if let Some((p, req)) = self.arbitrate(bus) {
                    let cppn = (self.icr & 0xFF) as u8;
                    let free_ext = (9..12).find(|&i| self.src[i] & SRC_SRR == 0);
                    let free_pcp = [2usize, 3, 9, 10, 11].iter().filter(|&&i| self.src[i] & SRC_SRR == 0).count();
                    if p > cppn && free_pcp >= 2 {
                        if let Some(node) = free_ext {
                            let ch = self.channel.unwrap();
                            let pc = self.pc;
                            self.save(ch, pc);
                            self.src[node] = (self.src[node] & !0x00FF_00FF) | SRC_SRR | SRC_RRQ | (ch as u32) << 16 | cppn as u32;
                            self.stats.suspends += 1;
                            self.take(req, bus);
                            self.start(req);
                            used += 2 * self.exit_cycles();
                            continue;
                        }
                    }
                }
            }
            used += self.step(bus);
        }
        self.stats.cycles += used;
        used
    }

    #[inline]
    fn exit_cycles(&self) -> u64 {
        match self.ctx_regs() {
            8 => 9,
            4 => 7,
            _ => 6,
        }
    }

    #[inline]
    fn set_r(&mut self, n: u8, v: u32) {
        if n == 7 {
            if v & IEN != 0 && self.r[7] & IEN == 0 {
                self.irq_dirty = true;
            }
            self.r[7] = v & 0xFFFF;
        } else {
            self.r[n as usize] = v;
        }
        self.dirty |= 1 << n;
    }

    /// Flag update; applied after the result write so it wins over bits moved into R7 (p.11-9 note).
    #[inline]
    fn flags(&mut self, mask: u32, val: u32) {
        if mask & val & IEN != 0 && self.r[7] & IEN == 0 {
            self.irq_dirty = true;
        }
        self.r[7] = (self.r[7] & !mask) | (val & mask);
        self.dirty |= 0x80;
    }

    #[inline]
    fn nz(v: u32) -> u32 {
        ((v >> 31 != 0) as u32 * N) | ((v == 0) as u32 * Z)
    }

    #[inline]
    fn add_flags(&mut self, a: u32, b: u32) -> u32 {
        let (r, c) = a.overflowing_add(b);
        let v = ((a ^ r) & (b ^ r)) >> 31 != 0;
        self.flags(N | Z | V | C, Self::nz(r) | (v as u32 * V) | (c as u32 * C));
        r
    }

    /// `a - b`: C = borrow (unsigned a < b, "cc_ULT", Table 11-12 p.11-73), V = signed overflow.
    #[inline]
    fn sub_flags(&mut self, a: u32, b: u32) -> u32 {
        let r = a.wrapping_sub(b);
        let v = ((a ^ b) & (a ^ r)) >> 31 != 0;
        self.flags(N | Z | V | C, Self::nz(r) | (v as u32 * V) | ((a < b) as u32 * C));
        r
    }

    /// Condition codes, Table 11-12 (p.11-73).
    #[inline(always)]
    fn cond(&self, cc: u8) -> bool {
        if cc == 0 {
            return true;
        }
        let f = self.r[7];
        let (z, n, c, v, cnz) = (f & Z != 0, f & N != 0, f & C != 0, f & V != 0, f & CNZ != 0);
        match cc {
            0 => true,
            1 => z,
            2 => !z,
            3 => v,
            4 => c,
            5 => !(c || z),
            6 => n ^ v,
            7 => !((n ^ v) || z),
            8 => n,
            9 => !n,
            10 => !v,
            11 => !c,
            12 => !(n ^ v),
            13 => (n ^ v) || z,
            14 => cnz,
            _ => !cnz,
        }
    }

    #[inline(always)]
    fn pram_addr(&self, off: u32) -> usize {
        ((self.r[7] >> 8 & 0xFF) << 6 | (off & 0x3F)) as usize
    }

    #[inline(always)]
    fn pram_rd(&mut self, off: u32) -> Option<u32> {
        let i = self.pram_addr(off);
        let v = self.pram.get(i).copied();
        if v.is_none() {
            self.error(ES_BER);
        }
        v
    }

    #[inline(always)]
    fn pram_wr(&mut self, off: u32, v: u32) -> bool {
        let i = self.pram_addr(off);
        // PRAM partitioning (p.11-38): no writes into the CSA.
        if self.cs & CS_PPE != 0 && i < (2 * ((self.cs >> 9) & 0x7F) as usize + 1) * self.ctx_regs() {
            self.error(ES_PPC);
            return false;
        }
        match self.pram.get_mut(i) {
            Some(w) => {
                *w = v;
                true
            }
            None => {
                self.error(ES_BER);
                false
            }
        }
    }

    /// FPI read by the PCP. Own PRAM/CMEM are served internally (word accesses only, p.11-48).
    #[inline]
    fn fpi_rd(&mut self, bus: &mut impl FpiBus, a: u32, size: u8) -> Option<u32> {
        if Self::contains(a) {
            if size != 4 || (PCP_REGS..PCP_REGS + 0x100).contains(&a) {
                self.error(ES_BER); // byte/half PRAM access or own registers (p.11-45, p.11-48)
                return None;
            }
            return Some(self.word_read(a));
        }
        let Some(v) = bus.fpi_try_read(a, size) else {
            self.error(ES_BER);
            return None;
        };
        Some(v)
    }

    #[inline]
    fn fpi_wr(&mut self, bus: &mut impl FpiBus, a: u32, v: u32, size: u8) -> bool {
        let v = v & lane_mask(size);
        if Self::contains(a) {
            if size != 4 || (PCP_REGS..PCP_REGS + 0x100).contains(&a) {
                self.error(ES_BER);
                return false;
            }
            self.mmio_write_word(a & !3, v, !0);
            return true;
        }
        if !bus.fpi_try_write(a, v, size) {
            self.error(ES_BER);
            return false;
        }
        if !crate::gpta::Gpta::contains(a) {
            self.irq_dirty = true;
        }
        true
    }

    /// Jump to an absolute halfword address; IAE on overflow or outside CMEM (p.11-39, p.11-46).
    #[inline]
    fn jump(&mut self, target: i32) {
        if !(0..=0xFFFF).contains(&target) || target as usize >= self.cmem.len() {
            self.error(ES_IAE);
        } else {
            self.pc = target as u16;
        }
    }

    /// Execute one instruction. Returns cycles.
    #[inline(always)]
    fn step<B: FpiBus>(&mut self, bus: &mut B) -> u64 {
        let pc = self.pc;
        let idx = pc as usize;
        if idx >= self.cmem.len() {
            self.error(ES_IAE);
            return 1;
        }
        let w = self.cmem[idx];
        let w2 = self.cmem.get(idx + 1).copied().unwrap_or(0);
        let ins = match self.decoded.get(idx).copied() {
            Some((cw, cw2, Some(ins))) if cw == w && cw2 == w2 => ins,
            Some((cw, cw2, None)) if cw == w && cw2 == w2 => {
                self.error(ES_IOP);
                return 1;
            }
            _ => {
                let ins = decode(w, w2);
                if let Some(slot) = self.decoded.get_mut(idx) {
                    *slot = (w, w2, ins);
                }
                let Some(ins) = ins else {
                    self.error(ES_IOP);
                    return 1;
                };
                ins
            }
        };
        if let Some(t) = self.trace.as_mut() {
            t.push((self.channel.unwrap_or(0), pc, w));
        }
        let len = ins.len();
        self.fetched += len as u32;
        // Channel watchdog (p.11-39): threshold 16 * CWT instructions.
        if self.cs & CS_CWE != 0 && self.fetched > 16 * ((self.cs >> 17) & 0x7F) {
            self.error(ES_CWD);
            return 1;
        }
        if len == 2 && idx + 1 >= self.cmem.len() {
            self.error(ES_IAE);
            return 1;
        }
        let next = pc as i32 + len as i32;
        if next > 0xFFFF {
            self.error(ES_IAE);
            return 1;
        }
        self.pc = next as u16;
        self.stats.instructions += 1;
        self.exec(ins, next, bus)
    }

    #[inline(always)]
    fn exec<B: FpiBus>(&mut self, ins: Insn, next: i32, bus: &mut B) -> u64 {
        let sizeb = |s: u8| 1u8 << s;
        match ins {
            Insn::Nop => 1,
            // COPY p.11-85, counter operation Figure 11-13 p.11-77
            Insn::Copy { dst, src, cnc, cnt0, size } => {
                let n = sizeb(size);
                let mut cycles = 0;
                loop {
                    let mut c0 = if cnt0 == 0 { 8 } else { cnt0 as u32 };
                    while c0 > 0 {
                        let Some(v) = self.fpi_rd(bus, self.r[4], n) else { return cycles + 8 };
                        if !self.fpi_wr(bus, self.r[5], v, n) {
                            return cycles + 8;
                        }
                        self.r[4] = step_ptr(self.r[4], src, n as u32);
                        self.r[5] = step_ptr(self.r[5], dst, n as u32);
                        self.dirty |= 0x30;
                        c0 -= 1;
                        cycles += 8;
                    }
                    if cnc == 0 || !self.dec_cnt1() || cnc == 1 {
                        break;
                    }
                }
                cycles
            }
            // BCOPY p.11-81, Figure 11-14 p.11-78; block of 8/2/4 words (Table 11-13 p.11-74)
            Insn::Bcopy { dst, src, cnc, cnt0 } => {
                let words = match cnt0 { 0 => 8, 2 => 2, _ => 4 };
                let mut cycles = 0;
                loop {
                    for i in 0..words {
                        let Some(v) = self.fpi_rd(bus, self.r[4].wrapping_add(4 * i), 4) else { return cycles + 8 };
                        if !self.fpi_wr(bus, self.r[5].wrapping_add(4 * i), v, 4) {
                            return cycles + 8;
                        }
                    }
                    self.r[4] = step_ptr(self.r[4], src, 4 * words);
                    self.r[5] = step_ptr(self.r[5], dst, 4 * words);
                    self.dirty |= 0x30;
                    cycles += 4 + 2 * words as u64;
                    if cnc == 0 || !self.dec_cnt1() || cnc == 1 {
                        break;
                    }
                }
                cycles
            }
            // EXIT p.11-89
            Insn::Exit { st, int, ep, ec, cc } => {
                let ch = self.channel.unwrap();
                if int && self.cond_after_ec(ec, cc) {
                    let tos = (self.r[6] >> 14) & 1; // R6.TOS upper bit forced to 0 (p.11-8)
                    let srpn = (self.r[6] >> 16) as u8;
                    if !self.post(tos, srpn) {
                        self.pc = (next - 1) as u16; // stall: re-execute EXIT later (p.11-36)
                        return 1;
                    }
                }
                if ec {
                    self.dec_cnt1();
                }
                if st {
                    self.set_r(7, self.r[7] & !CEN);
                }
                let save_pc = if ep && self.cs & CS_RCB == 0 { next as u16 } else { 2 * ch as u16 };
                self.save(ch, save_pc);
                self.exit_cycles()
            }
            Insn::F { op, rb, ra, size } => {
                let a = self.r[ra as usize];
                let n = sizeb(size);
                match op {
                    FOp::St => {
                        self.fpi_wr(bus, a, self.r[rb as usize], n);
                        return 5;
                    }
                    FOp::Xch => {
                        let Some(m) = self.fpi_rd(bus, a, n) else { return 8 };
                        let old = self.r[rb as usize];
                        if self.fpi_wr(bus, a, old, n) {
                            self.set_r(rb, m);
                            self.flags(N | Z, Self::nz(self.r[rb as usize]));
                        }
                        return 8;
                    }
                    _ => {}
                }
                let Some(m) = self.fpi_rd(bus, a, n) else { return 8 };
                let b = self.r[rb as usize];
                match op {
                    FOp::Add => {
                        let r = self.add_flags(b, m);
                        self.set_r(rb, r);
                    }
                    FOp::Sub => {
                        let r = self.sub_flags(b, m);
                        self.set_r(rb, r);
                    }
                    // COMP.F p.11-84: Flags(R[b] - sign_ext(FPI[R[a]]))
                    FOp::Comp => {
                        let m = sign_ext(m, 8 * n as u32);
                        self.sub_flags(b, m);
                    }
                    FOp::And | FOp::Or | FOp::Xor | FOp::Ld => {
                        let r = match op {
                            FOp::And => b & m,
                            FOp::Or => b | m,
                            FOp::Xor => b ^ m,
                            _ => m,
                        };
                        self.set_r(rb, r);
                        self.flags(N | Z, Self::nz(self.r[rb as usize]));
                    }
                    FOp::St | FOp::Xch => unreachable!(),
                }
                8
            }
            Insn::Pi { op, ra, off } => {
                let o = off as u32;
                let rv = self.r[ra as usize];
                if op == PiOp::St {
                    self.pram_wr(o, rv);
                    return 2;
                }
                let Some(m) = self.pram_rd(o) else { return 2 };
                match op {
                    PiOp::Add => {
                        let r = self.add_flags(rv, m);
                        self.set_r(ra, r);
                    }
                    PiOp::Sub => {
                        let r = self.sub_flags(rv, m);
                        self.set_r(ra, r);
                    }
                    PiOp::Comp => {
                        self.sub_flags(rv, m);
                    }
                    // MCLR.PI / MSET.PI p.11-99: R[a] = R[a] op PRAM; PRAM = R[a]
                    PiOp::Mclr | PiOp::Mset => {
                        let r = if op == PiOp::Mclr { rv & m } else { rv | m };
                        if self.pram_wr(o, r) {
                            self.set_r(ra, r);
                            self.flags(N | Z, Self::nz(self.r[ra as usize]));
                        }
                        return 4;
                    }
                    // XCH.PI p.11-105
                    PiOp::Xch => {
                        if self.pram_wr(o, rv) {
                            self.set_r(ra, m);
                            self.flags(N | Z, Self::nz(self.r[ra as usize]));
                        }
                    }
                    PiOp::And | PiOp::Or | PiOp::Xor | PiOp::Ld => {
                        let r = match op {
                            PiOp::And => rv & m,
                            PiOp::Or => rv | m,
                            PiOp::Xor => rv ^ m,
                            _ => m,
                        };
                        self.set_r(ra, r);
                        self.flags(N | Z, Self::nz(self.r[ra as usize]));
                    }
                    PiOp::St => unreachable!(),
                }
                2
            }
            Insn::R { op, rb, ra, cc } => {
                let cyc = match op {
                    ROp::LdP => 2,
                    ROp::StP => 4,
                    _ => 1,
                };
                // cc_A false => NOP, no flag update (p.11-41)
                if !self.cond(cc) {
                    return cyc;
                }
                let (b, a) = (self.r[rb as usize], self.r[ra as usize]);
                match op {
                    ROp::Add => {
                        let r = self.add_flags(b, a);
                        self.set_r(rb, r);
                    }
                    ROp::Sub => {
                        let r = self.sub_flags(b, a);
                        self.set_r(rb, r);
                    }
                    ROp::Comp => {
                        self.sub_flags(b, a);
                    }
                    // NEG p.11-96: R[b] = -R[a], flags of 0 - R[a]
                    ROp::Neg => {
                        let r = self.sub_flags(0, a);
                        self.set_r(rb, r);
                    }
                    ROp::LdP => {
                        let Some(m) = self.pram_rd(a & 0x3F) else { return cyc };
                        self.set_r(rb, m);
                        self.flags(N | Z, Self::nz(self.r[rb as usize]));
                    }
                    ROp::StP => {
                        self.pram_wr(a & 0x3F, b);
                    }
                    // INB p.11-88: R[b][R[a][4:0]] = C, no flags
                    ROp::Inb => {
                        let bit = 1u32 << (a & 0x1F);
                        let r = if self.r[7] & C != 0 { b | bit } else { b & !bit };
                        self.set_r(rb, r);
                    }
                    _ => {
                        let r = match op {
                            ROp::Not => !a,
                            ROp::And => b & a,
                            ROp::Or => b | a,
                            ROp::Xor => b ^ a,
                            ROp::Mov => a,
                            // PRI p.11-98: position of the most significant 1, 0x20 if zero
                            ROp::Pri => {
                                if a == 0 {
                                    0x20
                                } else {
                                    31 - a.leading_zeros()
                                }
                            }
                            _ => unreachable!(),
                        };
                        self.set_r(rb, r);
                        self.flags(N | Z, Self::nz(self.r[rb as usize]));
                    }
                }
                cyc
            }
            Insn::I { op, ra, imm } => {
                let x = self.r[ra as usize];
                let i = imm as u32;
                let n = i & 0x1F;
                match op {
                    IOp::Add => {
                        let r = self.add_flags(x, i);
                        self.set_r(ra, r);
                    }
                    IOp::Sub => {
                        let r = self.sub_flags(x, i);
                        self.set_r(ra, r);
                    }
                    // COMP.I p.11-84: sign-extended imm6
                    IOp::Comp => {
                        self.sub_flags(x, sign_ext(i, 6));
                    }
                    IOp::Ld => {
                        self.set_r(ra, i);
                        self.flags(N | Z, Self::nz(i));
                    }
                    // SHL p.11-101: C = last bit shifted out of bit 31
                    IOp::Shl => {
                        let r = if n >= 32 { 0 } else { x << n };
                        let c = n != 0 && (x >> (32 - n)) & 1 != 0;
                        self.set_r(ra, r);
                        self.flags(N | Z | C, Self::nz(self.r[ra as usize]) | (c as u32 * C));
                    }
                    // SHR p.11-102
                    IOp::Shr => {
                        let r = x >> n;
                        self.set_r(ra, r);
                        self.flags(N | Z, Self::nz(self.r[ra as usize]));
                    }
                    // RL p.11-100: C = last bit shifted out of bit 31
                    IOp::Rl => {
                        let r = x.rotate_left(n);
                        let c = n != 0 && r & 1 != 0;
                        self.set_r(ra, r);
                        self.flags(N | Z | C, Self::nz(self.r[ra as usize]) | (c as u32 * C));
                    }
                    // RR p.11-100: N, Z only (C unchanged — firmware relies on it, see module tests)
                    IOp::Rr => {
                        let r = x.rotate_right(n);
                        self.set_r(ra, r);
                        self.flags(N | Z, Self::nz(self.r[ra as usize]));
                    }
                    IOp::Set => self.set_r(ra, x | 1 << n),
                    IOp::Clr => self.set_r(ra, x & !(1 << n)),
                    IOp::InbI => {
                        let r = if self.r[7] & C != 0 { x | 1 << n } else { x & !(1 << n) };
                        self.set_r(ra, r);
                    }
                    // CHKB p.11-83: C = (R[a][imm5] == S/C)
                    IOp::Chkb => {
                        let bit = (x >> n) & 1;
                        let want = (i >> 5) & 1;
                        self.flags(C, (bit == want) as u32 * C);
                    }
                }
                1
            }
            // LDL.IL / LDL.IU p.11-93
            Insn::Ldl { upper, ra, imm } => {
                let x = self.r[ra as usize];
                let r = if upper { (x & 0xFFFF) | (imm as u32) << 16 } else { (x & 0xFFFF_0000) | imm as u32 };
                self.set_r(ra, r);
                self.flags(N | Z, Self::nz(self.r[ra as usize]));
                1
            }
            Insn::Fi { op, ra, imm, size } => {
                let n = sizeb(size);
                let base = self.r[ra as usize];
                match op {
                    // SET.F / CLR.F p.11-83, 11-101: locked RMW on [R[a]]
                    FiOp::SetF | FiOp::ClrF => {
                        let Some(m) = self.fpi_rd(bus, base, n) else { return 8 };
                        let bit = 1u32 << imm;
                        let v = if op == FiOp::SetF { m | bit } else { m & !bit };
                        self.fpi_wr(bus, base, v, n);
                        8
                    }
                    // LD.IF p.11-91: R0 = zext(FPI[R[a] + offset5])
                    FiOp::LdIf => {
                        let Some(m) = self.fpi_rd(bus, base.wrapping_add(imm as u32), n) else { return 8 };
                        self.set_r(0, m);
                        self.flags(N | Z, Self::nz(m));
                        8
                    }
                    // ST.IF p.11-103: FPI[R[a] + offset5] = R0
                    FiOp::StIf => {
                        self.fpi_wr(bus, base.wrapping_add(imm as u32), self.r[0], n);
                        5
                    }
                }
            }
            Insn::M { op, rb, ra } => {
                let (b, a, r0) = (self.r[rb as usize], self.r[ra as usize], self.r[0]);
                match op {
                    // DINIT p.11-87
                    MOp::Dinit => {
                        self.set_r(0, 0);
                        self.flags(V | Z, ((a == 0) as u32 * V) | ((b == 0 && a != 0) as u32 * Z));
                        1
                    }
                    // DSTEP p.11-88. Z: the UM lists Z without defining it; HYP: Z = (quotient Rb == 0),
                    // by analogy with DINIT ("flag a zero result"). The SIMOS image never tests Z after DSTEP.
                    MOp::Dstep => {
                        let r0n = (r0 << 8).wrapping_add(b >> 24);
                        let (q, rem) = r0n.checked_div(a).map_or((0, r0n), |q| (q, r0n % a));
                        let bn = (b << 8).wrapping_add(q);
                        self.set_r(rb, bn);
                        self.set_r(0, rem);
                        self.flags(Z, (bn == 0) as u32 * Z);
                        10
                    }
                    // MINIT p.11-93
                    MOp::Minit => {
                        self.set_r(0, 0);
                        self.flags(Z, (a == 0 || b == 0) as u32 * Z);
                        1
                    }
                    // MSTEP32 p.11-95
                    MOp::Mstep32 => {
                        let bn = b.rotate_left(8);
                        let r = (r0 << 8).wrapping_add((bn & 0xFF).wrapping_mul(a));
                        self.set_r(rb, bn);
                        self.set_r(0, r);
                        self.flags(Z, (r == 0) as u32 * Z);
                        10
                    }
                    // MSTEP64 p.11-95 (40-bit temp)
                    MOp::Mstep64 => {
                        let t = (r0 as u64 + a as u64 * (b & 0xFF) as u64) & 0xFF_FFFF_FFFF;
                        let r = (t >> 8) as u32;
                        let bn = (b >> 8) | ((t & 0xFF) as u32) << 24;
                        self.set_r(rb, bn);
                        self.set_r(0, r);
                        self.flags(Z, (r == 0) as u32 * Z);
                        10
                    }
                }
            }
            // JL p.11-91: NextPC + sext(offset10) (p.11-46)
            Insn::Jl { off } => {
                self.jump(next + off as i32);
                4
            }
            // JC p.11-90 (target relative to NextPC, p.11-46/47)
            Insn::Jc { off, cc } => {
                if self.cond(cc) {
                    self.jump(next + off as i32);
                    4
                } else {
                    2
                }
            }
            Insn::JcA { addr, cc } => {
                if self.cond(cc) {
                    self.jump(addr as i32);
                    4
                } else {
                    2
                }
            }
            Insn::JcI { ra, cc } => {
                if self.cond(cc) {
                    self.jump(next + self.r[ra as usize] as u16 as i16 as i32);
                    4
                } else {
                    2
                }
            }
            Insn::JcIa { ra, cc } => {
                if self.cond(cc) {
                    self.jump(self.r[ra as usize] as u16 as i32);
                    4
                } else {
                    2
                }
            }
            // DEBUG p.11-86: only legal in debug mode, otherwise IOP error.
            Insn::Debug { dac, rta, eda: _, sdb, cc } => {
                if !self.debug_mode {
                    self.pc = (next - 1) as u16;
                    self.error(ES_IOP);
                    return 1;
                }
                if !self.cond(cc) {
                    return 2;
                }
                let ch = self.channel.unwrap();
                self.es = ((next as u32) & 0xFFFF) << 16 | (ch as u32) << 8 | ES_DBE;
                if dac {
                    self.cs &= !CS_EN;
                }
                if sdb {
                    let mut pc = next as u16;
                    if rta {
                        pc -= 1;
                    } else {
                        self.set_r(7, self.r[7] & !CEN);
                    }
                    self.save(ch, pc);
                    return self.exit_cycles();
                }
                2
            }
        }
    }

    /// EXIT: the interrupt condition is evaluated after the optional CNT1 decrement (operation order p.11-89).
    fn cond_after_ec(&mut self, ec: bool, cc: u8) -> bool {
        if !ec {
            return self.cond(cc);
        }
        let saved = (self.r[6], self.r[7], self.dirty);
        self.dec_cnt1();
        let c = self.cond(cc);
        (self.r[6], self.r[7], self.dirty) = saved;
        c
    }

    /// CNT1 = CNT1 - 1, CN1Z updated (Table 11-14 note 1, p.11-108). Returns true while CNT1 != 0.
    fn dec_cnt1(&mut self) -> bool {
        let c = (self.r[6] & 0xFFF).wrapping_sub(1) & 0xFFF;
        self.set_r(6, (self.r[6] & !0xFFF) | c);
        self.flags(CNZ, (c == 0) as u32 * CNZ);
        c != 0
    }

    /// Load a CMEM image from a flash copy table (dst, src, len, flag) as written by the firmware's
    /// startup copy loop; `read` returns flash bytes for an address. Only PRAM/CMEM destinations are taken.
    pub fn load_copy_table(&mut self, entries: &[(u32, u32, u32)], read: impl Fn(u32) -> u8) {
        for &(dst, src, len) in entries {
            for i in 0..len {
                let a = dst + i;
                if Self::contains(a) && a >= PRAM_BASE {
                    self.mmio_write(a, read(src + i) as u32, 1);
                }
            }
        }
    }
}

fn lane_mask(size: u8) -> u32 {
    match size {
        1 => 0xFF,
        2 => 0xFFFF,
        _ => 0xFFFF_FFFF,
    }
}

fn sign_ext(v: u32, bits: u32) -> u32 {
    if bits >= 32 {
        v
    } else {
        ((v << (32 - bits)) as i32 >> (32 - bits)) as u32
    }
}

/// SRC+/- / DST+/- pointer update (Table 11-13, p.11-75/76).
fn step_ptr(p: u32, mode: u8, n: u32) -> u32 {
    match mode {
        1 => p.wrapping_add(n),
        2 => p.wrapping_sub(n),
        _ => p,
    }
}

// ------------------------------------------------------------------------------------------------
// Unit tests: one per instruction class, against the UM pseudo-code.
// ------------------------------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    /// Flat FPI memory + a list of PCP-routed SRNs `(srpn, pending)`.
    #[derive(Default)]
    struct Bus {
        mem: HashMap<u32, u8>,
        req: Vec<u8>,
        writes: Vec<(u32, u32, u8)>,
    }

    impl FpiBus for Bus {
        fn fpi_read(&mut self, a: u32, size: u8) -> u32 {
            (0..size as u32).fold(0, |v, i| v | (*self.mem.get(&(a + i)).unwrap_or(&0) as u32) << (8 * i))
        }
        fn fpi_write(&mut self, a: u32, v: u32, size: u8) {
            self.writes.push((a, v, size));
            for i in 0..size as u32 {
                self.mem.insert(a + i, (v >> (8 * i)) as u8);
            }
        }
        fn pcp_pending(&mut self) -> Option<u8> {
            self.req.iter().copied().max()
        }
        fn pcp_ack(&mut self, srpn: u8) {
            self.req.retain(|&r| r != srpn);
        }
    }

    /// Assemble raw words at PC 0x100 for channel `ch` (full context, resume mode, DPTR 0x10, CEN).
    fn setup(code: &[u16]) -> (Pcp, Bus) {
        let mut p = Pcp::new();
        p.cs = 0x21;
        for (i, &w) in code.iter().enumerate() {
            p.cmem[0x100 + i] = w;
        }
        let ch = 5u8;
        p.pram[ch as usize * 8] = 0x0100_1040; // CR7: PC 0x100, DPTR 0x10, CEN
        p.pram[ch as usize * 8 + 1] = 0x0500_0000; // CR6: CPPN 5
        let bus = Bus { req: vec![ch], ..Default::default() };
        (p, bus)
    }

    fn run(code: &[u16]) -> (Pcp, Bus) {
        let (mut p, mut b) = setup(code);
        p.run(10_000, &mut b);
        (p, b)
    }

    const EXIT: u16 = 0x1100; // exit ep=1

    #[test]
    fn decode_matches_binutils_table() {
        // (word, expected text) — opcode/mask pairs from binutils pcp_opcodes[]
        let cases: &[(u16, u16, &str)] = &[
            (0x537b, 0, "ld.pi r5, [0x3b]"),
            (0x3222, 0, "ld.f r0, [r4], size=32"),
            (0x9c29, 0, "chkb r0, 9, set"),
            (0xeac0, 0x0462, "jc.a 0x0462, cc_uge"),
            (0x9300, 0x0004, "ldl.il r4, 0x0004"),
            (0x3529, 0, "st.f r4, [r5], size=16"),
            (0x362a, 0, "xch.f r0, [r5], size=32"),
            (0x6ee8, 0, "or r3, r5, cc_uc"),
            (0xf498, 0, "jc.ia r3, cc_nz"),
            (0x1100, 0, "exit st=0, int=0, ep=1, ec=0, cc_uc"),
            (0xaf09, 0, "set.f [r4], 0x9, size=32"),
            (0xb340, 0, "clr.f [r5], 0x0, size=32"),
            (0xc468, 0, "minit r1, r5"),
            (0xc668, 0, "mstep32 r1, r5"),
            (0xc2d0, 0, "dstep r3, r2"),
            (0x9bc5, 0, "inb.i r7, 0x5"),
            (0x0000, 0, "nop"),
            (0x0800 | 1 << 9 | 1 << 7 | 2, 0, "copy dst+, src+, cnc=0, cnt0=8, size=32"),
        ];
        for &(w, w2, s) in cases {
            assert_eq!(disasm(0x450, w, w2), s, "{w:#06x}");
        }
        // JC relative to NextPC with sign extension: 0xe7ff at 0x100 = jc 0x100 (offset -1), cc_uc
        assert_eq!(disasm(0x100, 0xe43f, 0), "jc 0x0100, cc_uc");
        assert_eq!(disasm(0x100, 0xe3ff, 0), "jl 0x0100");
        // invalid encodings
        for w in [0x1070u16, 0x2004, 0x6c00, 0xc001, 0xe801, 0xf830, 0x1801] {
            assert!(decode(w, 0).is_none(), "{w:#06x}");
        }
    }

    #[test]
    fn arithmetic_flags() {
        // ld.i r1,0x3f ; ldl.iu r1,0x7fff ; ld.i r2,1 ; add r1,r2 (overflow) ; exit
        let (p, _) = run(&[0x9040, 0x7fff, 0x9240, 0xffff, 0x9881, 0x6050, EXIT]);
        assert_eq!(p.r[1], 0x8000_0000);
        assert_eq!(p.r[7] & (N | Z | V | C), N | V);
        // ld.i r1,0 ; sub.i r1,1 => 0xffffffff, borrow C=1, N=1
        let (p, _) = run(&[0x9840, 0x8241, EXIT]);
        assert_eq!(p.r[1], 0xFFFF_FFFF);
        assert_eq!(p.r[7] & (N | Z | V | C), N | C);
        // comp.i r1,0x3f compares with sign-extended -1: 5 - (-1): no borrow? 5 < 0xffffffff => C=1
        let (p, _) = run(&[0x9845, 0x847f, EXIT]);
        assert_eq!(p.r[1], 5);
        assert_eq!(p.r[7] & C, C);
        // neg r2, r1 with r1 = 5 => -5, C (borrow from 0)
        let (p, _) = run(&[0x9845, 0x6688, EXIT]);
        assert_eq!(p.r[2], (-5i32) as u32);
        assert_eq!(p.r[7] & (N | C), N | C);
    }

    #[test]
    fn conditional_register_ops_are_nops_when_false() {
        // ld.i r1,1 ; ld.i r2,2 ; comp.i r1,1 (Z=1) ; mov r1,r2,cc_nz (skipped) ; mov r3,r2,cc_z ; exit
        let (p, _) = run(&[0x9841, 0x9882, 0x8441, 0x7852, 0x78d1, EXIT]);
        assert_eq!(p.r[1], 1);
        assert_eq!(p.r[3], 2);
        assert_eq!(p.r[7] & Z, 0); // MOV updated flags (2 != 0)
    }

    #[test]
    fn shifts_rotates_and_bits() {
        // ld.i r1,0x21 ; shl r1,4 ; ldl.iu r1,0x8000 ; shl r1,1 (C=1)
        let (p, _) = run(&[0x9861, 0x8a44, 0x9040, 0x8000, 0x8a41, EXIT]);
        assert_eq!(p.r[1], 0x0000_0420);
        assert_eq!(p.r[7] & C, C);
        // rr keeps C: set C via shl, then rr r1,8 must not touch C
        let (p, _) = run(&[0x9841, 0x9040, 0x8000, 0x8a41, 0x8c48, EXIT]);
        assert_eq!(p.r[7] & C, C);
        // rl r1,1 with bit31 set => C=1, bit0 = 1
        let (p, _) = run(&[0x9841, 0x9040, 0x8000, 0x8e41, EXIT]);
        assert_eq!(p.r[1], 0x0000_0003);
        assert_eq!(p.r[7] & C, C);
        // set r1,31 ; clr r1,31 ; chkb r1,0,clr ; inb.i r2,7
        let (p, _) = run(&[0x9840, 0x945f, 0x965f, 0x9c40, 0x9a87, EXIT]);
        assert_eq!(p.r[1], 0);
        assert_eq!(p.r[2], 0x80);
        // pri r2, r1 with r1 = 0x20 -> 5; with 0 -> 0x20
        let (p, _) = run(&[0x9860, 0x7c88, EXIT]);
        assert_eq!(p.r[2], 5);
        let (p, _) = run(&[0x9840, 0x7c88, EXIT]);
        assert_eq!(p.r[2], 0x20);
    }

    #[test]
    fn r7_writes_keep_upper_zero_and_flags_win() {
        // ldl.il r1,0xff07 ; mov r7,r1 => 0xff04 (Z,N recomputed: positive, non-zero; C stays from value)
        let (p, _) = run(&[0x9240, 0xff07, 0x79c8, EXIT]);
        // R7 lower = 0xff04 at EXIT time; CEN (bit 6) is set by the value
        assert_eq!(p.pram[5 * 8] & 0xFFFF, 0xff04);
    }

    #[test]
    fn pram_ops_use_dptr() {
        // DPTR 0x10 -> PRAM word 0x400 + off
        let (mut p, mut b) = setup(&[0x5242, 0x4243, 0x5442, 0x4c41, 0x4841, 0x5641, EXIT]);
        p.pram[0x402] = 10;
        p.pram[0x403] = 3;
        p.pram[0x401] = 0xF0;
        p.run(1000, &mut b);
        // ld.pi r1,[2]=10 ; sub.pi r1,[3] => 7 ; st.pi r1,[2] ; mset.pi r1,[1] => 0xF7 ; mclr.pi r1,[1] => 0xF7 ; xch.pi r1,[1]
        assert_eq!(p.pram[0x402], 7);
        assert_eq!(p.pram[0x401], 0xF7);
        assert_eq!(p.r[1], 0xF7);
        // ld.p / st.p use R[a][5:0]
        let (mut p, mut b) = setup(&[0x9845, 0x7288, 0x8081, 0x7488, EXIT]);
        p.pram[0x405] = 41;
        p.run(1000, &mut b);
        assert_eq!(p.pram[0x405], 42);
    }

    #[test]
    fn fpi_ops_and_own_pram_via_fpi() {
        // r4 = 0xF0001000 ; ld.i r0,0x2a ; st.if [r4],4,size32 ; ld.f r1,[r4]+... ; set.f [r4],3 ; ld.if [r4],4
        // st.if [r4], 4, size=32 = 101 110 1 100 0 00100 = 0xbb04
        let (mut p, mut b) = setup(&[0x9100, 0xf000, 0x9300, 0x1000, 0x982a, 0xbb04, EXIT]);
        p.run(1000, &mut b);
        assert_eq!(b.writes, vec![(0xF000_1004, 0x2a, 4)]);
        // XCH.F on own PRAM address goes to PRAM, not to the bus
        let (mut p, mut b) = setup(&[0x9300, 0x0010, 0x9100, 0xf005, 0x9801, 0x3622, EXIT]);
        p.pram[4] = 0;
        p.run(1000, &mut b);
        assert_eq!(p.pram[4], 1);
        assert_eq!(p.r[0], 0);
        assert!(b.writes.is_empty());
        // 16-bit store: st.f r0,[r4],size=16
        let (mut p, mut b) = setup(&[0x9100, 0xf000, 0x9300, 0x1810, 0x9804, 0x3421, EXIT]);
        p.run(1000, &mut b);
        assert_eq!(b.writes, vec![(0xF000_1810, 4, 2)]);
    }

    #[test]
    fn multiply_and_divide_sequences() {
        // r1 = 0x25, r5 = 1000 ; rr r1,8 ; minit r1,r5 ; mstep32 r1,r5 => r0 = 0x25*1000
        let (p, _) = run(&[0x9865, 0x9340, 0x03e8, 0x8c48, 0xc468, 0xc668, EXIT]);
        assert_eq!(p.r[0], 37 * 1000);
        // 32x32 via mstep64 x4: r1 = 0x12345678, r2 = 1000
        let (mut p, mut b) = setup(&[0xc450, 0xc850, 0xc850, 0xc850, 0xc850, EXIT]);
        p.r = [0, 0x1234_5678, 1000, 0, 0, 0, 0x0500_0000, 0x1040];
        p.last = Some(5); // keep preset registers (context-load optimisation)
        p.pc = 0x100;
        p.run(1000, &mut b);
        let full = 0x1234_5678u64 * 1000;
        assert_eq!(p.r[0], (full >> 32) as u32);
        assert_eq!(p.r[1], full as u32);
        // 32/32 divide: dinit r3,r2 ; 4x dstep r3,r2 => r3 = quotient, r0 = remainder
        let (mut p, mut b) = setup(&[0xc0d0, 0xc2d0, 0xc2d0, 0xc2d0, 0xc2d0, EXIT]);
        p.r = [0, 0, 7, 1_000_003, 0, 0, 0x0500_0000, 0x1040];
        p.last = Some(5);
        p.pc = 0x100;
        p.run(1000, &mut b);
        assert_eq!(p.r[3], 1_000_003 / 7);
        assert_eq!(p.r[0], 1_000_003 % 7);
    }

    #[test]
    fn jumps() {
        // jc +1 skips one; jl back; jc.i computed; jc.ia absolute
        // 0x100: ld.i r1,2 ; jc.i r1 -> 0x102+2 = 0x104 ; ld.i r2,9 ; ld.i r2,9 ; ld.i r3,7 ; exit
        let (p, _) = run(&[0x9842, 0xf008, 0x9889, 0x9889, 0x98c7, EXIT]);
        assert_eq!(p.r[2], 0);
        assert_eq!(p.r[3], 7);
        // jc.a not-taken (cc_z with Z=0) then taken
        let (p, _) = run(&[0x9841, 0xe840, 0x0105, 0xe800, 0x0106, 0x98c1, EXIT]);
        assert_eq!(p.r[3], 0);
        // jump outside CMEM -> IAE error exit, channel disabled
        let (p, _) = run(&[0x9240, 0x7fff, 0xf408]);
        assert_eq!(p.es & 0xFF, ES_IAE);
        assert_eq!(p.pram[5 * 8] & CEN, 0);
    }

    #[test]
    fn context_save_restore_and_exit_modes() {
        // Resume mode, EP=1: saved PC = NextPC, R7 lower kept, dirty registers saved
        let (p, _) = run(&[0x98c7, EXIT, 0x9881]);
        assert_eq!(p.pram[5 * 8] >> 16, 0x102);
        assert_eq!(p.pram[5 * 8 + 7 - 3], 7); // CR3
        assert_eq!(p.stats.starts[5], 1);
        // EP=0: PC = channel entry 2*SRPN
        let (p, _) = run(&[0x1000]);
        assert_eq!(p.pram[5 * 8] >> 16, 10);
        // ST=1 disables the channel; next request => DCR error
        let (mut p, mut b) = setup(&[0x1500]);
        p.run(100, &mut b);
        assert_eq!(p.pram[5 * 8] & CEN, 0);
        b.req.push(5);
        p.last = None;
        p.run(100, &mut b);
        assert_eq!(p.es & 0xFF, ES_DCR);
        // small context model: CR7 at word 0 of a 4-word region
        let (mut p, mut b) = setup(&[EXIT]);
        p.cs = 0x61;
        p.pram[5 * 4] = 0x0100_1040;
        p.run(100, &mut b);
        assert_eq!(p.pram[5 * 4] >> 16, 0x101);
    }

    #[test]
    fn exit_interrupt_and_cnt1() {
        // R6: TOS=0 (CPU), SRPN 0x33, CNT1=1 ; exit ec=1,int=1,cc_cnz -> request posted (CNT1 -> 0)
        let (mut p, mut b) = setup(&[0x1380 | 0xE]);
        p.pram[5 * 8 + 1] = 0x0533_0001;
        p.run(100, &mut b);
        assert_eq!(p.pending_cpu_request(), Some((0, 0x33)));
        assert_eq!(p.pram[5 * 8 + 1] & 0xFFF, 0);
        assert_ne!(p.pram[5 * 8] & CNZ, 0);
        // TOS=1: self-request on the PCP bus -> channel 0x33 runs next
        let (mut p, mut b) = setup(&[0x1380, EXIT]);
        p.pram[5 * 8 + 1] = 0x0533_4001;
        p.pram[0x33 * 8] = 0x0101_1040;
        p.run(100, &mut b);
        assert_eq!(p.stats.starts[0x33], 1);
    }

    #[test]
    fn copy_with_counters() {
        // copy dst+, src+, cnc=2, cnt0=2, size=32 with CNT1=3 => 6 words
        let w = 0x0800 | 1 << 9 | 1 << 7 | 2 << 5 | 2 << 2 | 2;
        let (mut p, mut b) = setup(&[w, EXIT]);
        p.pram[5 * 8 + 1] = 0x0500_0003;
        p.pram[5 * 8 + 3] = 0xD000_0000; // CR4 = src
        p.pram[5 * 8 + 2] = 0xD000_1000; // CR5 = dst
        for i in 0..24 {
            b.mem.insert(0xD000_0000 + i, i as u8);
        }
        p.run(1000, &mut b);
        assert_eq!(b.writes.len(), 6);
        assert_eq!(b.writes[5], (0xD000_1014, 0x1716_1514, 4));
        assert_eq!(p.pram[5 * 8 + 3], 0xD000_0018);
    }

    #[test]
    fn priority_and_suspension() {
        // Channel 5 (IEN, CPPN 5) loops; request 9 arrives -> 5 suspended into SRC9..11, 9 runs, 5 resumes.
        let (mut p, mut b) = setup(&[0x0000, 0x0000, 0x0000, EXIT]);
        p.pram[5 * 8] = 0x0100_1060; // IEN|CEN
        p.cmem[0x200] = EXIT;
        p.pram[9 * 8] = 0x0200_1040;
        p.pram[9 * 8 + 1] = 0x0900_0000;
        p.run(3, &mut b); // starts ch5, executes a couple of NOPs
        b.req.push(9);
        p.run(1000, &mut b);
        assert_eq!(p.stats.suspends, 1);
        assert_eq!(p.stats.starts[9], 1);
        assert_eq!(p.stats.starts[5], 2);
        assert_eq!(p.pram[5 * 8] >> 16, 0x104);
        // Non-interruptible channel (IEN=0) is not suspended
        let (mut p, mut b) = setup(&[0x0000, 0x0000, 0x0000, EXIT]);
        p.pram[9 * 8] = 0x0200_1040;
        p.cmem[0x200] = EXIT;
        p.run(3, &mut b);
        b.req.push(9);
        p.run(1000, &mut b);
        assert_eq!(p.stats.suspends, 0);
    }

    #[test]
    fn invalid_opcode_and_debug() {
        let (p, _) = run(&[0xffff]);
        assert_eq!(p.es & 0xFF, ES_IOP);
        assert_eq!(p.es >> 16, 0x100);
        let (p, _) = run(&[0xfc01]); // debug sdb=1 outside debug mode -> IOP
        assert_eq!(p.es & 0xFF, ES_IOP);
    }

    #[test]
    fn mmio_register_block() {
        let mut p = Pcp::new();
        p.mmio_write(PCP_REGS + 0x10, 0x21, 4);
        p.mmio_write(PCP_REGS + 0x20, 0x0100_0000, 4);
        assert_eq!(p.mmio_read(PCP_REGS + 0x10, 4), 0x21);
        assert_eq!(p.mmio_read(PCP_REGS + 0x20, 4), 0x0100_0000);
        assert_eq!(p.mmio_read(PCP_REGS + 0xF4, 4), 0x1400); // PCP_SRC2 reset
        p.mmio_write(CMEM_BASE + 0x8A0, 0x5331_537b, 4);
        assert_eq!(p.cmem[0x450], 0x537b);
        assert_eq!(p.cmem[0x451], 0x5331);
        p.mmio_write(PRAM_BASE + 0x7E2, 0x0450, 2);
        assert_eq!(p.pram[0x7E0 / 4], 0x0450_0000);
    }

    #[test]
    fn pram_mmio_write_invalidates_cached_context_and_handles_cross_word() {
        // Channel 5 (full context, set up by `setup`): `add.i r1, #1` (`0x8041`) + `exit` (`EXIT`)
        let (mut p, mut b) = setup(&[0x8041, EXIT]);
        let r1_idx = 5 * 8 + 6; // CR1 = R1 of channel 5
        p.pram[r1_idx] = 10;
        p.run(100, &mut b);
        assert_eq!(p.pram[r1_idx], 11);
        assert_eq!(p.last, Some(5));

        // External CPU writes new R1 = 50 into PRAM while channel is idle (`channel.is_none()`):
        p.mmio_write(PRAM_BASE + r1_idx as u32 * 4, 50, 4);
        assert_eq!(p.last, None, "CPU PRAM write must invalidate cached self.last");

        p.pram[5 * 8] = 0x0100_1040; // reset ch 5 PC to 0x100
        b.req.push(5);
        p.run(100, &mut b);
        assert_eq!(p.pram[r1_idx], 51, "channel 5 must reload updated R1 = 50 from PRAM");

        // Cross-word PRAM access and out-of-range PCP_REGS read/write
        p.mmio_write(PRAM_BASE + 2, 0xAABB_CCDD, 4);
        assert_eq!(p.mmio_read(PRAM_BASE + 2, 4), 0xAABB_CCDD);
        assert_eq!(p.mmio_read(0xF004_0000, 4), 0, "address below PCP_REGS must not underflow u32");
    }
}
