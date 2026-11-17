//! Test helpers for instruction-group unit tests (`#[cfg(test)]` only).
//!
//! (a) [`run`] executes raw instruction bytes on the native core with a given register/memory
//!     setup — no assembler needed (take the bytes from the manual encoding box, a `fmt` struct, or
//!     a rizin disassembly of the firmware);
//! (b) [`oracle`] runs the same bytes on bare Unicorn for comparison, guarded by
//!     [`oracle::KNOWN_WRONG`] (Unicorn translator defects that must never be used as reference).
//!
//! ```ignore
//! use crate::testutil::*;
//! let c = run(&0x31C0_100Bu32.to_le_bytes(), 1, |c| c.d[1] = (-5i32) as u32); // abs d3, d1
//! assert_eq!(c.d[3], 5);
//! let o = oracle::run(&0x31C0_100Bu32.to_le_bytes(), 1, &Setup::from_cpu(&c0)).unwrap();
//! ```

pub mod oracle;

use crate::{Arch, Cpu, Memory, NullBus};

/// Code is placed at the start of a 16 KB writable region (PSPR address of the TC1791).
pub const CODE: u32 = 0xC000_0000;
pub const CODE_SIZE: u32 = 0x4000;
/// 64 KB of data RAM (DSPR address of the TC1791).
pub const DATA: u32 = 0xD000_0000;
pub const DATA_SIZE: u32 = 0x1_0000;
/// Trap vector table base inside the code region (each class vector is filled with NOPs by
/// default, so a trap is visible as `pc == TRAP_BASE | class << 5`).
pub const TRAP_BASE: u32 = CODE + 0x2000;
/// Interrupt vector table base inside the code region.
pub const IRQ_BASE: u32 = CODE + 0x3000;
/// A free CSA list of 8 entries is linked at `CSA_BASE` (FCX points to the first, LCX to the
/// seventh), so CALL/RET/SVLCX/BISR work out of the box.
pub const CSA_BASE: u32 = DATA + 0x8000;

/// Link word (FCX/PCXI format) of a 64-byte aligned CSA address.
pub const fn link(ea: u32) -> u32 {
    ((ea >> 12) & 0xF_0000) | ((ea >> 6) & 0xFFFF)
}

/// A core with CODE/DATA mapped, `bytes` at `CODE`, PC = CODE, BTV/BIV/ISP/FCX/LCX set up.
pub fn cpu_with(bytes: &[u8], arch: Arch) -> Cpu<NullBus> {
    assert!(bytes.len() <= 0x2000, "code overlaps the trap table");
    let mut m = Memory::new();
    let code = m.add_owned(vec![0; CODE_SIZE as usize], true);
    let data = m.add_owned(vec![0; DATA_SIZE as usize], true);
    m.map(CODE, code, 0, CODE_SIZE as usize);
    m.map(DATA, data, 0, DATA_SIZE as usize);
    let mut c = Cpu::new(arch, m, NullBus);
    c.write_bytes(CODE, bytes).unwrap();
    for i in 0..8u32 {
        let ea = CSA_BASE + 64 * i;
        let next = if i < 7 { link(ea + 64) } else { 0 };
        c.write_bytes(ea, &next.to_le_bytes()).unwrap();
    }
    c.fcx = link(CSA_BASE);
    c.lcx = link(CSA_BASE + 64 * 6);
    c.btv = TRAP_BASE;
    c.biv = IRQ_BASE;
    c.isp = DATA + 0x7000;
    c.a[10] = DATA + 0x6000;
    c.pc = CODE;
    c
}

/// Execute exactly `n` instructions of `bytes` (TC1.6) after `setup` and return the core.
pub fn run(bytes: &[u8], n: u64, setup: impl FnOnce(&mut Cpu<NullBus>)) -> Cpu<NullBus> {
    run_arch(bytes, n, Arch::Tc16, setup)
}

/// [`run`] for a given architecture.
pub fn run_arch(bytes: &[u8], n: u64, arch: Arch, setup: impl FnOnce(&mut Cpu<NullBus>)) -> Cpu<NullBus> {
    let mut c = cpu_with(bytes, arch);
    setup(&mut c);
    let exit = c.run(n);
    assert!(matches!(exit, crate::Exit::Budget | crate::Exit::Stopped(_)), "{exit:?}");
    c
}

/// The register/memory state the oracle comparison works on.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Setup {
    pub d: [u32; 16],
    pub a: [u32; 16],
    pub psw: u32,
    pub pcxi: u32,
    pub icr: u32,
    pub fcx: u32,
    pub lcx: u32,
    pub pc: u32,
    /// (address, bytes) written before execution; addresses must lie in CODE or DATA.
    pub mem: Vec<(u32, Vec<u8>)>,
}

impl Setup {
    /// Snapshot of the registers of a native core (memory is not copied).
    pub fn from_cpu(c: &Cpu<NullBus>) -> Self {
        Self { d: c.d, a: c.a, psw: c.psw, pcxi: c.pcxi, icr: c.icr, fcx: c.fcx, lcx: c.lcx, pc: c.pc, mem: Vec::new() }
    }

    /// Apply to a native core.
    pub fn apply(&self, c: &mut Cpu<NullBus>) {
        (c.d, c.a, c.psw, c.pcxi, c.icr, c.fcx, c.lcx) = (self.d, self.a, self.psw, self.pcxi, self.icr, self.fcx, self.lcx);
        for (addr, b) in &self.mem {
            c.write_bytes(*addr, b).unwrap();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn harness_runs_and_traps_on_unimplemented() {
        // RR 0BH with op2 FFH is undefined in the manual (V2 Table 3-1): IOPC trap (class 2).
        let c = run(&0x0FF0_000Bu32.to_le_bytes(), 1, |c| c.psw = 0);
        assert_eq!(c.icount, 1);
        assert_eq!(c.pc, TRAP_BASE | 2 << 5);
        assert_eq!(c.a[11], CODE);
        assert_eq!(c.a[10], DATA + 0x7000, "ISP loaded because PSW.IS was 0");
    }
}
