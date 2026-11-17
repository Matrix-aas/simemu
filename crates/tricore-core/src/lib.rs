//! Native Infineon TriCore CPU core (TriCore 1.6 for the TC1791/SIMOS18.1, TriCore 1.3.1 for the
//! TC1784/DQ200_G2) — no Unicorn/QEMU.
//!
//! * [`Cpu`] — architectural state (D0-15, A0-15, PC, PSW, PCXI, FCX, LCX, ICR, ISP, BTV, BIV,
//!   SYSCON, other CSFRs via [`arch::CSFR_TABLE`]), exact instruction counter, [`Cpu::run`] that
//!   stops exactly on the budget or when an instruction/the bus requests an interrupt check.
//! * [`Memory`] — host slices for flash/RAM (64 KB page table, aliases, read-only flash, code
//!   granule tracking for self-modifying code); [`Bus`] — peripheral fallback.
//! * [`op`] — central decode/execute dispatch over the per-group operation enums in [`groups`];
//!   blocks are predecoded once and cached by PC (`cache`) and run as threaded code: one handler
//!   per hot operation, chained to the next record and to linked successor blocks (`cpu`), with
//!   whole-block superinstructions chosen from firmware profiles (`fused`). `docs/PERFORMANCE.md` §8.
//! * [`context`] — CSA save/restore, call-depth counter, trap and interrupt entry.
//! * [`fmt`] — field extractors for every instruction format; [`flags`] — PSW status helpers.
//! * [`replay`] — record/replay differential harness against the Unicorn board.
//!
//! Manuals (`vault/docs/infineon/`): `tc16_architecture_vol1_a4.pdf` (V1), `tcarch_vol2_a4.pdf` (V2),
//! `tc_v131_corearchitecture_v138.pdf` (V131-1), `Tc_v131_instructionset_v138.pdf` (V131-2).
//! The instruction census of the firmware images is `docs/tricore_core/insn_census.md`.
//!
//! Interrupts: the board's ICU arbitrates; between `run` calls it publishes the winner with
//! [`Cpu::set_pipn`] and calls [`Cpu::take_interrupt`] (instruction-boundary delivery, V1 §5.3).

pub mod arch;
pub mod bus;
mod cache;
mod fused;
pub mod context;
pub mod cpu;
pub mod flags;
pub mod fmt;
pub mod groups;
pub mod mem;
pub mod op;
pub mod replay;
#[cfg(test)]
pub mod testutil;

pub use arch::{Arch, Trap};
pub use bus::{Bus, BusError, NullBus};
pub use cpu::{Cpu, Exit};
pub use mem::{Memory, RegionId};
pub use op::{Flow, StopReason};
