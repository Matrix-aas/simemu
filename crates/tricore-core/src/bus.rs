//! Peripheral fallback interface.
//!
//! Flash and RAM are host slices registered in [`crate::Memory`] and accessed by the core directly.
//! Everything else — peripheral segments, and stores into read-only memory (flash command cycles) —
//! goes through this trait. The board implements it with its register models (today the Unicorn
//! MMIO callbacks in `simemu-tc1791/src/cpu_hooks.rs`: `mmio_read`/`mmio_write`/`flash_command`).

/// The addressed location does not respond (unmapped / bus error).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BusError;

pub trait Bus {
    /// Read `size` (1, 2, 4 or 8) bytes at `addr`, little-endian, zero-extended.
    fn read(&mut self, addr: u32, size: u8) -> Result<u64, BusError>;

    /// Write the low `size` (1, 2, 4 or 8) bytes of `value` at `addr`.
    fn write(&mut self, addr: u32, size: u8, value: u64) -> Result<(), BusError>;

    /// System ENDINIT state (watchdog). While `true`, MTCR to ENDINIT-protected CSFRs has no effect
    /// (V1 p. 3-1). Implementations without a watchdog model leave the CSFRs writable.
    fn endinit(&self) -> bool {
        false
    }

    /// Polled after every peripheral access: `true` makes [`crate::Cpu::run`] return
    /// [`crate::Exit::Requested`] at the next instruction boundary (e.g. a software-triggered service
    /// request that the caller must arbitrate now).
    fn stop_requested(&mut self) -> bool {
        false
    }

    /// Called before every peripheral access with the exact number of instructions retired before
    /// the accessing one (the [`crate::Cpu::icount`] it executes at) and the start address of the
    /// executing block, so that time-driven peripheral models can catch up to the access.
    fn clock(&mut self, _icount: u64, _block_pc: u32) {}

    /// Asked with the new ICR after an instruction that requests interrupt re-arbitration
    /// ([`crate::StopReason::IrqCheck`]: ENABLE, DISABLE, RESTORE, MTCR ICR, BISR, RFE): `false`
    /// when no pending request would be taken now, so [`crate::Cpu::run`] continues instead of
    /// returning [`crate::Exit::Stopped`]. Must not change state. The default always returns.
    fn irq_check(&mut self, _icr: u32) -> bool {
        true
    }

    /// Observation only, called after a successful direct RAM load with `trace-data-stores`.
    fn data_read(&mut self, _pc: u32, _addr: u32, _size: u8, _value: u64) {}

    /// Observation only, called before every guest data store, including direct RAM stores.
    fn data_write(&mut self, _pc: u32, _addr: u32, _size: u8, _value: u64) {}

    /// Observation of a completed call or a tail-call jump, before taking its target.
    fn call_execute(&mut self, _pc: u32, _target: u32, _return: u32) {}

    /// Asked when code at `pc` is decoded: `true` makes `pc` start a block and [`crate::Cpu::run`]
    /// return [`crate::Exit::Hook`] before executing it (once per arrival). Call
    /// [`crate::Cpu::flush_code_cache`] when the answer changes for already decoded code.
    fn hook(&mut self, _pc: u32) -> bool {
        false
    }
}

/// A bus without peripherals: every access fails (tests, pure-memory replays).
#[derive(Default, Clone, Copy, Debug)]
pub struct NullBus;

impl Bus for NullBus {
    fn read(&mut self, _addr: u32, _size: u8) -> Result<u64, BusError> {
        Err(BusError)
    }
    fn write(&mut self, _addr: u32, _size: u8, _value: u64) -> Result<(), BusError> {
        Err(BusError)
    }
}
