//! Native CPU backend: the board around `tricore_core::Cpu` instead of Unicorn/QEMU-TCG.
//!
//! * Memory: the board's `HostBuf`s are mapped zero-copy as core regions with the same map as
//!   `Simos18Board::map_memory` (flash/DFlash/BROM read-only, RAM writable; stores into RAM code
//!   drop the predecoded blocks). Stores into read-only memory go to `flash_command`, the four
//!   peripheral segments to `mmio_read`/`mmio_write` (8-byte accesses as two words, like Unicorn).
//! * Time: the core reports the exact instruction count of every peripheral access (`Bus::clock`),
//!   so peripheral models see `quantum_executed` per instruction instead of per block.
//! * Interrupts: the board's SRC arbitration result is delivered with the core's own interrupt entry
//!   (`Cpu::take_interrupt`) at the instruction boundary where it became pending: quanta end exactly
//!   at timer/peripheral events, after a peripheral access that requests a stop, and after every
//!   instruction that changes ICR (ENABLE/DISABLE/RESTORE/MTCR ICR/RFE/BISR). Traps are taken by the
//!   core itself.
//! * Code hooks (probes, SC800S50 signature hooks, first entry of an execution region) make the core
//!   stop before the hooked address (`Bus::hook`, `Exit::Hook`).
//!
//! Selected per board with [`CpuBackend`] (`SIMEMU_CPU=native|unicorn`, default `native`).

use crate::cpu_hooks::{region_of, Probe};
use crate::memory::{
    BoardVariant, DataWatchEvent, DataWatchKind, MilestoneEvent, Tc1791HardwareState,
    CALL_TRACE_CAP, WATCH_EVENT_CAP,
};
use crate::stm_icu::SRC_SRR;
use crate::{tcu_dq250, Simos18Board, S50_SIG_HOOKS};
use std::ptr::NonNull;
use tricore_core::{Arch, Bus, BusError, Cpu, Exit, Memory, StopReason};
use unicorn_tricore_sys::TriCoreReg;

/// CPU emulation engine of a board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CpuBackend {
    /// Unicorn 2.1 (QEMU TCG) with the board's translator fixes (`isa_fix`).
    Unicorn,
    /// `tricore-core`: predecoded-block interpreter implementing TC1.6 / TC1.3.1 per the manuals.
    Native,
}

impl CpuBackend {
    /// Returns `true` if this CPU backend was compiled into the binary (`Native` is always built in;
    /// `Unicorn` requires Cargo feature `unicorn`).
    pub const fn is_available(self) -> bool {
        match self {
            CpuBackend::Native => true,
            CpuBackend::Unicorn => unicorn_tricore_sys::LINKED,
        }
    }

    /// `SIMEMU_CPU=native|unicorn`; unset or anything else selects [`CpuBackend::Native`].
    pub fn from_env() -> Self {
        match std::env::var("SIMEMU_CPU").ok().as_deref().map(str::trim) {
            Some(s) if s.eq_ignore_ascii_case("unicorn") => CpuBackend::Unicorn,
            _ => CpuBackend::Native,
        }
    }

    /// Backends a test matrix runs: `Native` by default (pure Rust), or `[Unicorn, Native]` when
    /// compiled with `--features unicorn` (overridden by `SIMEMU_CPU`).
    pub fn matrix() -> Vec<Self> {
        match std::env::var("SIMEMU_CPU").ok().as_deref().map(str::trim) {
            Some(s) if s.eq_ignore_ascii_case("native") => vec![CpuBackend::Native],
            Some(s) if s.eq_ignore_ascii_case("unicorn") => {
                if unicorn_tricore_sys::LINKED {
                    vec![CpuBackend::Unicorn]
                } else {
                    Vec::new()
                }
            }
            _ if unicorn_tricore_sys::LINKED => vec![CpuBackend::Unicorn, CpuBackend::Native],
            _ => vec![CpuBackend::Native],
        }
    }
}

use crate::memory::is_mmio;

/// Parse an inclusive hex watchpoint address or `start:end` range (with optional `0x` prefixes).
pub fn parse_watch_range(part: &str) -> Option<(u32, u32)> {
    let (start, end) = part.split_once(':').unwrap_or((part, part));
    let parse = |value: &str| {
        let value = value.trim();
        let digits = value.strip_prefix("0x").or_else(|| value.strip_prefix("0X")).unwrap_or(value);
        u32::from_str_radix(digits, 16).ok()
    };
    let start = parse(start)?;
    let end = parse(end)?;
    (start <= end).then_some((start, end))
}

#[inline]
fn range_overlaps(ranges: &[(u32, u32)], addr: u32, size: u8) -> bool {
    let end = addr.saturating_add(size.saturating_sub(1) as u32);
    ranges.iter().any(|&(lo, hi)| addr <= hi && end >= lo)
}

/// The board as seen by the core: peripheral segments and flash command cycles.
pub(crate) struct HwBus {
    hw: NonNull<Tc1791HardwareState>,
    /// Core instruction count at the start of the running quantum.
    q_start: u64,
    /// Read-only mapped ranges `[lo, hi)` (flash, DFlash, BROM): stores are flash command cycles.
    rom: Vec<(u32, u32)>,
    /// Hooked code addresses (probes, signature hooks), sorted.
    hooks: Vec<u32>,
    data_watch_enabled: bool,
    call_trace_enabled: bool,
}

impl HwBus {
    #[inline]
    fn hw(&mut self) -> &mut Tc1791HardwareState {
        // SAFETY: the state outlives the core (the board drops the core first) and is not borrowed
        // elsewhere while the core runs (same contract as the Unicorn callbacks' user data).
        unsafe { self.hw.as_mut() }
    }

    fn mmio_read(&mut self, addr: u32, size: u32) -> u64 {
        let hw = self.hw();
        let v = hw.mmio_read(addr, size);
        let endinit = hw.scu.endinit;
        if let Some(log) = hw.mmio_log.get_mut() {
            log.push((false, addr, size as u8, v, endinit));
        }
        v
    }

    fn mmio_write(&mut self, addr: u32, size: u32, value: u64) {
        let hw = self.hw();
        hw.mmio_write(addr, size, value);
        let endinit = hw.scu.endinit;
        if let Some(log) = hw.mmio_log.get_mut() {
            log.push((true, addr, size as u8, value, endinit));
        }
    }
}

impl Bus for HwBus {
    fn read(&mut self, addr: u32, size: u8) -> Result<u64, BusError> {
        if !is_mmio(addr) {
            self.hw().last_bad_access = Some((unicorn_tricore_sys::UC_MEM_READ_UNMAPPED, addr));
            return Err(BusError);
        }
        Ok(if size == 8 {
            self.mmio_read(addr, 4) | self.mmio_read(addr.wrapping_add(4), 4) << 32
        } else {
            self.mmio_read(addr, size as u32)
        })
    }

    fn write(&mut self, addr: u32, size: u8, value: u64) -> Result<(), BusError> {
        if is_mmio(addr) {
            if size == 8 {
                self.mmio_write(addr, 4, value & 0xFFFF_FFFF);
                self.mmio_write(addr.wrapping_add(4), 4, value >> 32);
            } else {
                self.mmio_write(addr, size as u32, value);
            }
            return Ok(());
        }
        if self.rom.iter().any(|&(lo, hi)| addr >= lo && addr < hi) {
            let hw = self.hw();
            hw.flash_command(addr, size as u32, value);
            let endinit = hw.scu.endinit;
            if let Some(log) = hw.mmio_log.get_mut() {
                log.push((true, addr, size, value, endinit));
            }
            return Ok(());
        }
        self.hw().last_bad_access = Some((unicorn_tricore_sys::UC_MEM_WRITE_UNMAPPED, addr));
        Err(BusError)
    }

    fn data_read(&mut self, pc: u32, addr: u32, size: u8, value: u64) {
        if !self.data_watch_enabled {
            return;
        }
        let hw = self.hw();
        if range_overlaps(&hw.watch_read_ranges, addr, size) {
            if hw.watch_events.len() == WATCH_EVENT_CAP {
                hw.watch_events.pop_front();
            }
            let instructions = hw.total_instructions + hw.quantum_executed;
            hw.watch_events.push_back(DataWatchEvent { kind: DataWatchKind::Read, pc, addr, size, value, instructions });
        }
    }

    fn data_write(&mut self, pc: u32, addr: u32, size: u8, value: u64) {
        if !self.data_watch_enabled {
            return;
        }
        let hw = self.hw();
        if range_overlaps(&hw.watch_write_ranges, addr, size) {
            if hw.watch_events.len() == WATCH_EVENT_CAP {
                hw.watch_events.pop_front();
            }
            let instructions = hw.total_instructions + hw.quantum_executed;
            hw.watch_events.push_back(DataWatchEvent { kind: DataWatchKind::Write, pc, addr, size, value, instructions });
        }
    }

    fn call_execute(&mut self, pc: u32, target: u32, ret: u32) {
        if !self.call_trace_enabled {
            return;
        }
        let hw = self.hw();
        let in_range = hw.call_trace_ranges.is_empty()
            || hw.call_trace_ranges.iter().any(|&(lo, hi)| (lo..=hi).contains(&target));
        if in_range
            && hw.block_sequence.len() < CALL_TRACE_CAP
            && !hw.block_sequence.iter().any(|&(from, to, _, _)| from == pc && to == target)
        {
            hw.block_sequence.push((pc, target, ret, hw.total_instructions + hw.quantum_executed));
        }
    }

    fn endinit(&self) -> bool {
        // SAFETY: see `hw`.
        unsafe { self.hw.as_ref() }.scu.endinit
    }

    fn stop_requested(&mut self) -> bool {
        let hw = self.hw();
        hw.stop_requested || hw.reset_request.is_some()
    }

    #[inline]
    fn clock(&mut self, icount: u64, block_pc: u32) {
        let q = icount - self.q_start;
        let hw = self.hw();
        hw.quantum_executed = q;
        hw.last_tb_pc = block_pc;
    }

    /// Same test as `native_quantum` makes after `Exit::Stopped(IrqCheck)` (state unchanged in
    /// between), so the core carries on itself when no interrupt wins.
    #[inline]
    fn irq_check(&mut self, icr: u32) -> bool {
        self.hw().irq_wins(icr)
    }

    fn hook(&mut self, pc: u32) -> bool {
        let hw = self.hw();
        if !hw.block_hit_ranges.is_empty()
            && hw.block_hit_ranges.iter().any(|&(lo, hi)| (lo..=hi).contains(&pc))
        {
            *hw.block_hits.entry(pc).or_insert(0) += 1;
        }
        let seen = hw.regions_seen;
        self.hooks.binary_search(&pc).is_ok() || seen & 1 << region_of(pc).0 == 0
    }
}

pub(crate) type NativeCpu = Cpu<HwBus>;

/// Owned native CPU core of a board.
pub(crate) type NativeCell = Box<NativeCpu>;

/// Map the board memories into a new core (layout of `Simos18Board::map_memory`).
pub(crate) fn new_core(hw: NonNull<Tc1791HardwareState>) -> NativeCell {
    // SAFETY: called by the constructor; the buffers are owned by the state and never move.
    let s = unsafe { &mut *hw.as_ptr() };
    let mut m = Memory::new();
    let mut rom = Vec::new();
    // SAFETY (all `add_raw`): HostBufs live as long as the state, which outlives the core.
    let region = |m: &mut Memory, buf: &crate::memory::HostBuf, writable: bool| unsafe { m.add_raw(buf.raw(), buf.len(), writable) };
    let mut ram_ranges = Vec::new();
    let mut map = |m: &mut Memory, rom: &mut Vec<(u32, u32)>, base: u32, r, off: usize, len: usize, writable: bool| {
        m.map(base, r, off, len);
        if writable {
            ram_ranges.push((base, base as u64 + len as u64));
        } else {
            rom.push((base, base + len as u32));
        }
    };
    let arch = match s.variant {
        BoardVariant::Simos18Tc1791 => {
            let fl = region(&mut m, &s.flash, false);
            for base in [0x8000_0000u32, 0xA000_0000] {
                map(&mut m, &mut rom, base, fl, 0, 0x20_0000, false);
                map(&mut m, &mut rom, base + 0x80_0000, fl, 0x20_0000, 0x20_0000, false);
            }
            let df = region(&mut m, &s.dflash, false);
            map(&mut m, &mut rom, 0xAF00_0000, df, 0, crate::flash::DFLASH_BANK, false);
            map(&mut m, &mut rom, 0xAF08_0000, df, crate::flash::DFLASH_BANK, crate::flash::DFLASH_BANK, false);
            let br = region(&mut m, &s.brom, false);
            map(&mut m, &mut rom, 0xAFFF_C000, br, 0, s.brom.len(), false);
            map(&mut m, &mut rom, 0x8FFF_C000, br, 0, s.brom.len(), false);
            let ds = region(&mut m, &s.dspr, true);
            map(&mut m, &mut rom, 0xD000_0000, ds, 0, s.dspr.len(), true);
            let ps = region(&mut m, &s.pspr, true);
            map(&mut m, &mut rom, 0xC000_0000, ps, 0, s.pspr.len(), true);
            let lm = region(&mut m, &s.lmu, true);
            map(&mut m, &mut rom, 0xB000_0000, lm, 0, s.lmu.len(), true);
            map(&mut m, &mut rom, 0x9000_0000, lm, 0, s.lmu.len(), true);
            Arch::Tc16
        }
        BoardVariant::Dq200Tc1784 => {
            let fl = region(&mut m, &s.flash, false);
            map(&mut m, &mut rom, 0x8000_0000, fl, 0, 0x40_0000, false);
            map(&mut m, &mut rom, 0xA000_0000, fl, 0, 0x40_0000, false);
            let df = region(&mut m, &s.dflash, false);
            map(&mut m, &mut rom, 0xAF00_0000, df, 0, s.dflash.len(), false);
            map(&mut m, &mut rom, 0x8F00_0000, df, 0, s.dflash.len(), false);
            let br = region(&mut m, &s.brom, false);
            map(&mut m, &mut rom, 0xAFFF_0000, br, 0, s.brom.len(), false);
            map(&mut m, &mut rom, 0x8FFF_0000, br, 0, s.brom.len(), false);
            let ds = region(&mut m, &s.dspr, true);
            map(&mut m, &mut rom, 0xD000_0000, ds, 0, s.dspr.len(), true);
            let ps = region(&mut m, &s.pspr, true);
            map(&mut m, &mut rom, 0xD400_0000, ps, 0, s.pspr.len(), true);
            map(&mut m, &mut rom, 0xC000_0000, ps, 0, s.pspr.len(), true);
            let lm = region(&mut m, &s.lmu, true);
            map(&mut m, &mut rom, 0xB000_0000, lm, 0, s.lmu.len(), true);
            map(&mut m, &mut rom, 0x9000_0000, lm, 0, s.lmu.len(), true);
            // zero-filled RAM over unmodelled address space (the board's `uc_mem_map` windows)
            for (a, l) in [(0x0000_0000u32, 0x1_0000usize), (0xF040_0000, 0x07A0_0000), (0xF810_0000, 0x006F_0000), (0xF880_0000, 0x0780_0000)] {
                let r = m.add_owned(vec![0; l], true);
                m.map(a, r, 0, l);
                ram_ranges.push((a, a as u64 + l as u64));
            }
            Arch::Tc131
        }
        BoardVariant::Dq250Tc1766 => {
            let fl = region(&mut m, &s.flash, false);
            map(&mut m, &mut rom, 0x8000_0000, fl, 0, tcu_dq250::PFLASH_SIZE, false);
            map(&mut m, &mut rom, 0xA000_0000, fl, 0, tcu_dq250::PFLASH_SIZE, false);
            let df = region(&mut m, &s.dflash, false);
            map(&mut m, &mut rom, tcu_dq250::DFLASH_BASE, df, 0, s.dflash.len(), false);
            let ds = region(&mut m, &s.dspr, true);
            map(&mut m, &mut rom, 0xD000_0000, ds, 0, s.dspr.len(), true);
            let ps = region(&mut m, &s.pspr, true);
            map(&mut m, &mut rom, 0xD400_0000, ps, 0, s.pspr.len(), true);
            Arch::Tc13
        }
    };
    s.ram_ranges = ram_ranges;
    let mut hooks: Vec<u32> = Vec::new();
    if s.variant == BoardVariant::Simos18Tc1791 {
        // Flash locations are verified now; PSPR locations when executed (as `install_hooks`).
        hooks.extend(S50_SIG_HOOKS.iter().filter(|h| h.addr >= 0xC000_0000 || s.code_bytes(h.addr).is_some_and(|b| b.len() >= 8 && b[..8] == h.sig)).map(|h| h.addr));
        hooks.sort_unstable();
    }
    Box::new(Cpu::new(
        arch,
        m,
        HwBus { hw, q_start: 0, rom, hooks, data_watch_enabled: false, call_trace_enabled: false },
    ))
}

impl Simos18Board {
    #[inline]
    pub(crate) fn nat(&self) -> Option<&NativeCpu> {
        self.native.as_deref()
    }

    #[inline]
    pub(crate) fn nat_mut(&mut self) -> Option<&mut NativeCpu> {
        self.native.as_deref_mut()
    }

    pub fn backend(&self) -> CpuBackend {
        if self.native.is_some() {
            CpuBackend::Native
        } else {
            CpuBackend::Unicorn
        }
    }

    /// Replace the inclusive `[lo, hi]` guest read watch ranges (`Native` + `trace-data-stores`).
    pub fn set_watch_read_ranges(&mut self, ranges: Vec<(u32, u32)>) {
        self.hw_mut().watch_read_ranges = ranges;
    }

    /// Replace the inclusive `[lo, hi]` guest write watch ranges (`Native` + `trace-data-stores`).
    pub fn set_watch_write_ranges(&mut self, ranges: Vec<(u32, u32)>) {
        self.hw_mut().watch_write_ranges = ranges;
    }

    /// Add an inclusive `[lo, hi]` guest read watch range (`Native` + `trace-data-stores`).
    pub fn add_read_watch_range(&mut self, lo: u32, hi: u32) {
        if lo <= hi {
            self.hw_mut().watch_read_ranges.push((lo, hi));
        }
    }

    /// Add an inclusive `[lo, hi]` guest write watch range (`Native` + `trace-data-stores`).
    pub fn add_write_watch_range(&mut self, lo: u32, hi: u32) {
        if lo <= hi {
            self.hw_mut().watch_write_ranges.push((lo, hi));
        }
    }

    /// Clear all configured data read/write watch ranges.
    pub fn clear_watch_ranges(&mut self) {
        let hw = self.hw_mut();
        hw.watch_read_ranges.clear();
        hw.watch_write_ranges.clear();
    }

    /// Enable or disable recording of matching `DataWatchEvent`s on `CpuBackend::Native`.
    pub fn set_data_watch_enabled(&mut self, enabled: bool) {
        if let Some(cpu) = self.nat_mut() {
            cpu.bus.data_watch_enabled = enabled;
        }
    }

    /// Enable or disable recording of call/jump edges in `block_sequence` on `CpuBackend::Native`.
    pub fn set_call_trace_enabled(&mut self, enabled: bool) {
        if let Some(cpu) = self.nat_mut() {
            cpu.bus.call_trace_enabled = enabled;
        }
    }

    /// Restrict `call_execute` edge recording to targets in `ranges` (empty = record all targets).
    pub fn set_call_trace_ranges(&mut self, ranges: Vec<(u32, u32)>) {
        self.hw_mut().call_trace_ranges = ranges;
    }

    /// Configure inclusive `[lo, hi]` PC ranges counted when decoded on `CpuBackend::Native`.
    pub fn set_block_hit_ranges(&mut self, ranges: Vec<(u32, u32)>) {
        self.hw_mut().block_hit_ranges = ranges;
        if let Some(cpu) = self.nat_mut() {
            cpu.flush_code_cache();
        }
    }

    /// Convenience toggle for both data watchpoint and call-edge observers on `CpuBackend::Native`.
    pub fn set_data_write_trace(&mut self, enabled: bool) {
        if let Some(cpu) = self.nat_mut() {
            cpu.bus.data_watch_enabled = enabled;
            cpu.bus.call_trace_enabled = enabled;
        }
    }

    /// Native core reset part of `Simos18Board::reset` (the registers are written by the caller):
    /// implementation-specific CSFR reset values and the code hooks' per-reset state.
    pub(crate) fn native_reset(&mut self) {
        let Some(c) = self.nat_mut() else { return };
        c.csfrs.iter_mut().for_each(|v| *v = 0);
        // PCON2/DCON2: TC1791 cache/scratchpad sizes in KB (as `FixState::reset_csfrs`).
        c.csfr_set_reset(0x9208, (32 << 16) | 16);
        c.csfr_set_reset(0x9000, (128 << 16) | 16);
        // region milestones restart: blocks are re-decoded with the region hooks
        c.flush_code_cache();
    }

    /// Add a code hook address (probe).
    pub(crate) fn native_add_hook(&mut self, addr: u32) {
        let Some(c) = self.nat_mut() else { return };
        if let Err(i) = c.bus.hooks.binary_search(&addr) {
            c.bus.hooks.insert(i, addr);
        }
        c.flush_code_cache();
    }

    /// Execute one quantum of at most `budget` instructions. Returns the instructions executed
    /// and an error report if the core cannot continue.
    pub(crate) fn native_quantum(&mut self, budget: u64) -> (u64, Option<String>) {
        self.flush_pflash_code_dirty();
        let c = self.nat_mut().expect("native backend");
        let q_start = c.icount;
        c.bus.q_start = q_start;
        let end = q_start + budget;
        loop {
            let c = self.nat_mut().expect("native backend");
            match c.run(end - c.icount) {
                Exit::Hook { pc } => {
                    let q = c.icount - q_start;
                    self.hw_mut().quantum_executed = q;
                    self.native_hook(pc);
                }
                // DEBUG event without a debugger: the instruction retired, carry on
                Exit::Stopped(StopReason::Debug) => {}
                // ICR changed (ENABLE/DISABLE/RESTORE/MTCR/RFE/BISR, mostly lock pairs): end the
                // quantum only if an interrupt is taken now
                Exit::Stopped(StopReason::IrqCheck) if !self.native_irq_wins() => {}
                Exit::FetchFault { pc } => {
                    let c = self.nat().expect("native backend");
                    let n = c.icount - q_start;
                    let hw = self.hw();
                    let e = format!(
                        "native core: no executable memory at PC=0x{pc:08X} ({}) nor at the trap vector, A11=0x{:08X}, PCXI=0x{:08X}, FCX=0x{:08X}, bad access {:?}",
                        Self::classify_region(pc),
                        c.a[11],
                        c.pcxi,
                        c.fcx,
                        hw.last_bad_access.map(|(t, a)| format!("type {} @ 0x{:08X}", t, a)),
                    );
                    return (n, Some(e));
                }
                Exit::Budget | Exit::Requested | Exit::Stopped(_) => {
                    let c = self.nat().expect("native backend");
                    return (c.icount - q_start, None);
                }
            }
        }
    }

    /// Whether the ICU would interrupt the core now (`ICR.IE` and a pending SRPN > `ICR.CCPN`).
    fn native_irq_wins(&self) -> bool {
        self.hw().irq_wins(self.nat().expect("native backend").icr)
    }

    /// The core stopped before the hooked address `pc`.
    fn native_hook(&mut self, pc: u32) {
        let (rid, name) = region_of(pc);
        let hw = self.hw_mut();
        if hw.regions_seen & 1 << rid == 0 {
            hw.regions_seen |= 1 << rid;
            let ticks = hw.stm_now();
            hw.milestones.push(MilestoneEvent { reset_cycle: hw.reset_count, name, pc, stm_ticks: ticks });
            // re-decode without the region hook (the resumed block is still reported as passed)
            self.nat_mut().expect("native backend").invalidate_code(pc, 2);
        }
        let c = self.nat().expect("native backend");
        let (a2, a11, a15) = (c.a[2], c.a[11], c.a[15]);
        if !self.hw().probes.is_empty() {
            let (d, a) = (c.d, c.a);
            let hw = self.hw_mut();
            if let Some(p) = hw.probes.iter_mut().find(|p: &&mut Probe| p.addr == pc) {
                p.hits += 1;
                p.last_regs[..16].copy_from_slice(&d);
                p.last_regs[16..].copy_from_slice(&a);
            }
        }
        let hw = self.hw_mut();
        if hw.variant == BoardVariant::Simos18Tc1791 {
            crate::sig_hit(hw, pc, |r| match r {
                TriCoreReg::A2 => a2,
                TriCoreReg::A11 => a11,
                TriCoreReg::A15 => a15,
                _ => 0,
            });
        }
    }

    /// ICU: publish the arbitration result in ICR.PIPN and take the winning interrupt through the
    /// core's interrupt entry (V1 §5.3). The SRC is acknowledged only when the entry succeeded.
    pub(crate) fn native_dispatch_irq(&mut self) -> bool {
        let pending = self.hw().pending_cpu_request();
        if pending.is_none() && self.hw().pipn_published == 0 {
            return false;
        }
        let pipn = pending.map_or(0, |(p, _)| p);
        self.hw_mut().pipn_published = pipn as u8;
        let c = self.nat_mut().expect("native backend");
        c.set_pipn(pipn as u8);
        let Some((pipn, src)) = pending else { return false };
        let vector = c.biv | pipn << 5;
        let pc = c.pc;
        match c.take_interrupt(pipn as u8) {
            Ok(false) => false,
            Ok(true) => {
                let fcd = c.pc != vector;
                let hw = self.hw_mut();
                let v = hw.src_get(src) & !SRC_SRR;
                hw.src_put(src, v);
                hw.irq_dispatch_count += 1;
                if let Some(log) = hw.event_log.as_mut() {
                    log.push((hw.total_instructions + hw.quantum_executed, 2, pc, pipn as u64));
                }
                hw.trap_count += fcd as u64;
                true
            }
            // context save failed (FCU trap entered): the request stays pending
            Err(_) => {
                self.hw_mut().trap_count += 1;
                true
            }
        }
    }

    pub(crate) fn native_read_reg(c: &NativeCpu, reg: TriCoreReg) -> u32 {
        use TriCoreReg as R;
        let n = reg as u32;
        match reg {
            R::A0 | R::A1 | R::A2 | R::A3 | R::A4 | R::A5 | R::A6 | R::A7 | R::A8 | R::A9 | R::A10 | R::A11 | R::A12 | R::A13 | R::A14 | R::A15 => c.a[(n - 1) as usize],
            R::D0 | R::D1 | R::D2 | R::D3 | R::D4 | R::D5 | R::D6 | R::D7 | R::D8 | R::D9 | R::D10 | R::D11 | R::D12 | R::D13 | R::D14 | R::D15 => c.d[(n - 17) as usize],
            R::PC => c.pc,
            R::PSW => c.psw,
            // Unicorn's cached status flags: C non-zero, V/SV/AV/SAV in bit 31
            R::PSW_USB_C => c.psw >> 31,
            R::PSW_USB_V => (c.psw << 1) & 1 << 31,
            R::PSW_USB_SV => (c.psw << 2) & 1 << 31,
            R::PSW_USB_AV => (c.psw << 3) & 1 << 31,
            R::PSW_USB_SAV => (c.psw << 4) & 1 << 31,
            R::PCXI => c.pcxi,
            R::ICR => c.icr,
            R::FCX => c.fcx,
            R::LCX => c.lcx,
            R::ISP => c.isp,
            R::BTV => c.btv,
            R::BIV => c.biv,
            R::SYSCON => c.syscon,
            R::CPU_ID => c.csfr_read(0xFE18, c.pc),
            R::COMPAT => c.csfr_read(0x9400, c.pc),
            R::Invalid => 0,
        }
    }

    pub(crate) fn native_write_reg(c: &mut NativeCpu, reg: TriCoreReg, v: u32) {
        use TriCoreReg as R;
        let n = reg as u32;
        let flag = |c: &mut NativeCpu, bit: u32, on: bool| c.psw = (c.psw & !(1 << bit)) | (on as u32) << bit;
        match reg {
            R::A0 | R::A1 | R::A2 | R::A3 | R::A4 | R::A5 | R::A6 | R::A7 | R::A8 | R::A9 | R::A10 | R::A11 | R::A12 | R::A13 | R::A14 | R::A15 => c.a[(n - 1) as usize] = v,
            R::D0 | R::D1 | R::D2 | R::D3 | R::D4 | R::D5 | R::D6 | R::D7 | R::D8 | R::D9 | R::D10 | R::D11 | R::D12 | R::D13 | R::D14 | R::D15 => c.d[(n - 17) as usize] = v,
            R::PC => c.pc = v & !1,
            R::PSW => c.psw = v,
            R::PSW_USB_C => flag(c, 31, v != 0),
            R::PSW_USB_V => flag(c, 30, v >> 31 != 0),
            R::PSW_USB_SV => flag(c, 29, v >> 31 != 0),
            R::PSW_USB_AV => flag(c, 28, v >> 31 != 0),
            R::PSW_USB_SAV => flag(c, 27, v >> 31 != 0),
            R::PCXI => c.pcxi = v,
            R::ICR => c.icr = v,
            R::FCX => c.fcx = v,
            R::LCX => c.lcx = v,
            R::ISP => c.isp = v,
            R::BTV => c.btv = v & !1,
            R::BIV => c.biv = v & !1,
            R::SYSCON => c.syscon = v,
            R::CPU_ID => c.csfr_set_reset(0xFE18, v),
            R::COMPAT => c.csfr_set_reset(0x9400, v),
            R::Invalid => {}
        }
    }

    /// Debugger read: memories directly, peripheral registers through the register models.
    pub(crate) fn native_read_mem(&self, addr: u32, len: usize) -> Result<Vec<u8>, String> {
        let mut buf = vec![0u8; len];
        if self.nat().expect("native backend").read_bytes(addr, &mut buf) {
            return Ok(buf);
        }
        for (i, b) in buf.iter_mut().enumerate() {
            let a = addr.wrapping_add(i as u32);
            if !self.nat().expect("native backend").read_bytes(a, std::slice::from_mut(b)) {
                if !is_mmio(a) {
                    return Err(format!("read of unmapped memory at 0x{a:08X}"));
                }
                *b = self.hw().mmio_peek(a);
            }
        }
        Ok(buf)
    }

    /// Debugger write: memories (read-only ones too) directly, dropping affected predecoded code;
    /// peripheral registers through the register models.
    pub(crate) fn native_write_mem(&mut self, addr: u32, data: &[u8]) -> Result<(), String> {
        let c = self.nat_mut().expect("native backend");
        if c.write_bytes(addr, data).is_ok() {
            return Ok(());
        }
        for (i, &b) in data.iter().enumerate() {
            let a = addr.wrapping_add(i as u32);
            let c = self.nat_mut().expect("native backend");
            if c.write_bytes(a, &[b]).is_err() {
                if !is_mmio(a) {
                    return Err(format!("write to unmapped memory at 0x{a:08X}"));
                }
                self.hw_mut().mmio_write(a, 1, b as u64);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn watch_range_parser_handles_addresses_ranges_and_invalid_input() {
        assert_eq!(parse_watch_range("D00108A4"), Some((0xD001_08A4, 0xD001_08A4)));
        assert_eq!(parse_watch_range("0xD0000400:D00004FF"), Some((0xD000_0400, 0xD000_04FF)));
        assert_eq!(parse_watch_range("2:1"), None);
        assert_eq!(parse_watch_range("not-an-address"), None);
    }

    #[test]
    fn probe_stops_before_execution_and_captures_registers() {
        let mut board = Simos18Board::new_on(&vec![0; 0x40_0000], &vec![0; 0x4_0000], CpuBackend::Native).unwrap();
        board.add_probe(0x8000_0000).unwrap();
        board.write_reg(TriCoreReg::PC, 0x8000_0000);
        board.write_reg(TriCoreReg::d(4), 0x1234_5678);
        let (executed, error) = board.native_quantum(1);
        assert_eq!((executed, error), (1, None));
        let probe = board.probe(0x8000_0000).unwrap();
        assert_eq!(probe.hits, 1);
        assert_eq!(probe.last_regs[4], 0x1234_5678);
        assert_eq!(board.read_reg(TriCoreReg::PC), 0x8000_0002);
    }

    #[test]
    #[cfg(feature = "trace-data-stores")]
    fn data_watch_and_call_trace_capture_configured_ranges() {
        let mut board = Simos18Board::new_on(&vec![0; 0x40_0000], &vec![0; 0x4_0000], CpuBackend::Native).unwrap();
        // PSPR @ 0xC000_0000:
        // st.w [a4]0, d4  (59 44 00 00)
        // ld.w d5, [a4]0  (19 45 00 00)
        // jl 0xC000_0020  (5D 00 0C 00 -> disp24 = +12 halfwords = +24 bytes from 0xC000_0008 -> 0xC000_0020)
        // j .             (3C 00)
        // @ 0xC000_0020: ji a11 (DC 0B)
        board.write_mem(0xC000_0000, &[0x59, 0x44, 0x00, 0x00, 0x19, 0x45, 0x00, 0x00, 0x5D, 0x00, 0x0C, 0x00, 0x3C, 0x00]).unwrap();
        board.write_mem(0xC000_0020, &[0xDC, 0x0B]).unwrap();
        board.write_reg(TriCoreReg::A4, 0xD000_0400);
        board.write_reg(TriCoreReg::D4, 0xCAFE_BABE);
        board.write_reg(TriCoreReg::PC, 0xC000_0000);
        board.add_write_watch_range(0xD000_0402, 0xD000_0402); // overlaps 4-byte store at 0xD000_0400
        board.add_read_watch_range(0xD000_0400, 0xD000_0403);
        board.set_call_trace_ranges(vec![(0xC000_0020, 0xC000_002F)]);
        board.set_data_write_trace(true);

        board.run_instructions(5).unwrap();
        assert_eq!(board.read_reg(TriCoreReg::D5), 0xCAFE_BABE);
        let events = board.take_watch_events();
        assert_eq!(events.len(), 2);
        assert_eq!(
            (events[0].kind, events[0].pc, events[0].addr, events[0].size, events[0].value),
            (DataWatchKind::Write, 0xC000_0000, 0xD000_0400, 4, 0xCAFE_BABE)
        );
        assert_eq!(
            (events[1].kind, events[1].pc, events[1].addr, events[1].size, events[1].value),
            (DataWatchKind::Read, 0xC000_0004, 0xD000_0400, 4, 0xCAFE_BABE)
        );
        let calls = board.take_block_sequence();
        assert_eq!(calls.len(), 1);
        assert_eq!((calls[0].0, calls[0].1, calls[0].2), (0xC000_0008, 0xC000_0020, 0xC000_000C));
    }
}
