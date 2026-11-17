//! Host-backed memory regions, `CanFrame`/`MilestoneEvent` and the shared hardware state that the
//! C-ABI callbacks operate on.
//!
//! Ownership/aliasing model: the state lives behind a raw pointer owned by the board
//! (`Box::into_raw`); every guest-visible RAM/flash buffer is a [`HostBuf`] (raw allocation) that is
//! mapped into Unicorn with `uc_mem_map_ptr`. Rust code only creates short-lived slices from those
//! raw pointers, never long-lived `Box` borrows that would conflict with Unicorn's writes.

use crate::adc::AdcState;
use crate::flash::FlashCtrl;
use crate::isa_fix::FixState;
use crate::multican::MultiCan;
use crate::scu::ScuWdt;
use crate::stm_icu::StmState;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, VecDeque};
use std::ops::{Deref, DerefMut};
use std::ptr::NonNull;

/// Upper bound on buffered host-side watchpoint events (`Tc1791HardwareState::watch_events`).
pub const WATCH_EVENT_CAP: usize = 65_536;
/// Upper bound on buffered host-side call-trace edges (`Tc1791HardwareState::block_sequence`).
pub const CALL_TRACE_CAP: usize = 65_536;

/// Peripheral MMIO segments mapped via `uc_mmio_map` in `Simos18Board::map_memory`.
pub(crate) const MMIO_SEGMENTS: [(u32, u32); 4] = [
    (0xF000_0000, 0x40_0000),
    (0xF7E0_0000, 0x02_0000),
    (0xF800_0000, 0x10_0000),
    (0xF87F_0000, 0x01_0000),
];

#[inline]
pub(crate) fn is_mmio(a: u32) -> bool {
    MMIO_SEGMENTS.iter().any(|&(b, l)| a.wrapping_sub(b) < l)
}

/// Access direction of a [`DataWatchEvent`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DataWatchKind {
    Read,
    Write,
}

/// Host-side observation of a guest data load or store matching a configured watch range
/// (recorded on `CpuBackend::Native` when built with the `trace-data-stores` feature).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct DataWatchEvent {
    pub kind: DataWatchKind,
    pub pc: u32,
    pub addr: u32,
    pub size: u8,
    pub value: u64,
    pub instructions: u64,
}

/// A CAN 2.0 frame on one of the four MultiCAN nodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct CanFrame {
    /// MultiCAN node index (0..=3) the frame is transmitted/received on.
    pub node: u8,
    pub mo_index: u8,
    pub arb_id: u32,
    pub extended: bool,
    /// Number of data bytes (0..=8). Classic-CAN DLC values 9..15 are reported as 8.
    pub dlc: u8,
    pub data: [u8; 8],
    pub timestamp_us: u64,
}

/// Milestone trace entry recorded during boot and RTOS execution.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MilestoneEvent {
    pub reset_cycle: u32,
    pub name: &'static str,
    pub pc: u32,
    pub stm_ticks: u64,
}

/// A heap buffer whose address never changes and that is only reached through its raw pointer.
pub struct HostBuf {
    ptr: NonNull<u8>,
    len: usize,
}

// SAFETY: `HostBuf` is an owned heap allocation; it is moved together with its board.
unsafe impl Send for HostBuf {}

impl HostBuf {
    pub fn new(len: usize, fill: u8) -> Self {
        let b: Box<[u8]> = vec![fill; len].into_boxed_slice();
        let len = b.len();
        let ptr = NonNull::new(Box::into_raw(b) as *mut u8).expect("non-null alloc");
        Self { ptr, len }
    }

    pub fn from_slice(data: &[u8], len: usize, fill: u8) -> Self {
        let mut b = Self::new(len, fill);
        let n = data.len().min(len);
        b[..n].copy_from_slice(&data[..n]);
        b
    }

    #[inline]
    pub(crate) fn raw(&self) -> *mut u8 {
        self.ptr.as_ptr()
    }
}

impl Deref for HostBuf {
    type Target = [u8];
    #[inline]
    fn deref(&self) -> &[u8] {
        // SAFETY: ptr/len describe a live allocation owned by `self`.
        unsafe { std::slice::from_raw_parts(self.ptr.as_ptr(), self.len) }
    }
}

impl DerefMut for HostBuf {
    #[inline]
    fn deref_mut(&mut self) -> &mut [u8] {
        // SAFETY: as above; `&mut self` guarantees no other Rust borrow exists.
        unsafe { std::slice::from_raw_parts_mut(self.ptr.as_ptr(), self.len) }
    }
}

impl Drop for HostBuf {
    fn drop(&mut self) {
        // SAFETY: reconstructs exactly the Box created in `new`.
        unsafe {
            drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(
                self.ptr.as_ptr(),
                self.len,
            )));
        }
    }
}

/// Which SoC/board the shared core emulates (memory map differs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardVariant {
    /// Continental SIMOS18.1, Infineon TC1791 (AUDO MAX, TriCore 1.6).
    Simos18Tc1791,
    /// Temic DQ200_G2 mechatronic, Infineon TC1784 (AUDO Future, TriCore 1.3.1), see `tcu_dq200`.
    Dq200Tc1784,
    /// Temic DQ250 (02E/0D9) mechatronic, Infineon TC1766 (AUDO-NG, TriCore 1.3), see `tcu_dq250`.
    Dq250Tc1766,
}

/// Default-semantics register file for one peripheral address segment.
pub struct MmioSeg {
    pub base: u32,
    pub regs: HostBuf,
}

impl MmioSeg {
    pub fn new(base: u32, len: usize) -> Self {
        Self { base, regs: HostBuf::new(len, 0) }
    }
    #[inline]
    pub fn contains(&self, addr: u32) -> bool {
        addr.wrapping_sub(self.base) < self.regs.len() as u32
    }
    #[inline]
    pub fn get(&self, addr: u32) -> u32 {
        let off = addr.wrapping_sub(self.base) as usize & !3;
        if off + 4 > self.regs.len() {
            return 0;
        }
        u32::from_le_bytes(self.regs[off..off + 4].try_into().unwrap())
    }
    #[inline]
    pub fn set(&mut self, addr: u32, val: u32) {
        let off = addr.wrapping_sub(self.base) as usize & !3;
        if off + 4 <= self.regs.len() {
            self.regs[off..off + 4].copy_from_slice(&val.to_le_bytes());
        }
    }
}

/// Record/replay trace entry `(is_write, addr, size, value, endinit_after)`.
pub type MmioLogEntry = (bool, u32, u8, u64, bool);

/// Shared hardware state accessed by the C-ABI MMIO/code/block callbacks.
pub struct Tc1791HardwareState {
    pub variant: BoardVariant,

    // ---- Guest memories (mapped zero-copy with uc_mem_map_ptr) ----
    /// Program flash image (4 MB). SIMOS: [0..2M] = PFlash0 @0x80000000, [2M..4M] = PFlash1 @0x80800000.
    /// DQ200: linear 4 MB @0x80000000.
    pub flash: HostBuf,
    /// Data flash. SIMOS: [0..0x18000] bank 0 @0xAF000000, [0x18000..0x30000] bank 1 @0xAF080000.
    /// DQ200: the 8 KB SPI EEPROM image (padded), see `tcu_dq200`.
    pub dflash: HostBuf,
    pub brom: HostBuf,
    pub dspr: HostBuf,
    pub pspr: HostBuf,
    pub lmu: HostBuf,

    // ---- Peripheral register files (uc_mmio_map, default "plain register" semantics) ----
    pub seg_f000: MmioSeg,
    pub seg_f7e0: MmioSeg,
    pub seg_f800: MmioSeg,
    pub seg_f87f: MmioSeg,

    // ---- Modeled peripherals ----
    pub stm: StmState,
    pub scu: ScuWdt,
    pub can: MultiCan,
    pub flashc: FlashCtrl,
    pub adc: AdcState,
    /// SSC0..3, DMA and the external monitoring unit (see `dma::SscDma`).
    pub spi: crate::dma::SscDma,
    pub fixes: FixState,

    // ---- Execution accounting (maintained by the block hook) ----
    pub(crate) quantum_budget: u64,
    pub(crate) quantum_executed: u64,
    /// Instructions of the current quantum already folded into the STM base.
    pub(crate) stm_folded: u64,
    pub total_instructions: u64,
    /// Simulated time since power-on (instructions converted with the live clock tree).
    pub sim_time_s: f64,
    /// Set by any callback that wants the current quantum to end at the next block boundary
    /// (e.g. a newly pending interrupt or a timer re-programming).
    pub(crate) stop_requested: bool,
    pub(crate) reset_request: Option<crate::scu::ResetKind>,
    /// Start address of the most recently entered translation block (diagnostics).
    pub last_tb_pc: u32,
    /// Translation blocks that issued SWRSTCON.SWRSTREQ (diagnostics, bounded).
    pub swrst_log: Vec<u32>,
    /// Last unmapped/protected access reported by Unicorn: (access type, address).
    pub last_bad_access: Option<(i32, u32)>,
    /// Bitmask of execution regions entered since the last reset (region milestones).
    pub(crate) regions_seen: u16,
    pub probes: Vec<crate::cpu_hooks::Probe>,
    /// SRCs currently requesting a CPU interrupt (SRE & SRR & TOS=CPU & SRPN != 0).
    pub(crate) src_active: Vec<u32>,
    /// ICR.PIPN last written by the arbiter (`dispatch_pending_irq`); nothing else may change it.
    pub pipn_published: u8,
    pub(crate) tb_cache: Vec<crate::cpu_hooks::TbCacheEntry>,
    /// Writable host-backed guest memories `[begin, end)` (RAM, `end` in `u64` so a segment ending
    /// at `0x1_0000_0000` does not wrap to `0`). Pages in here are handed to the TLB without
    /// execute permission until code is fetched from them (see `cpu_hooks::hook_tlb_fill`).
    pub(crate) ram_ranges: Vec<(u32, u64)>,
    /// Unicorn target page size - 1.
    pub(crate) page_mask: u32,
    /// RAM pages code has been fetched from: (page base, instruction count of the latest block
    /// executed there). Stores to these pages take Unicorn's self-modifying-code path; pages
    /// without code execution for a while are returned to plain data RAM (pure performance measure).
    pub(crate) code_pages: Vec<(u32, u64)>,
    /// Guest PFlash ranges `(addr, len)` modified at runtime (page program / sector erase) whose
    /// translated blocks must be invalidated at the next quantum boundary.
    pub(crate) pflash_code_dirty: Vec<(u32, u32)>,
    /// TLB fills served by `cpu_hooks::hook_tlb_fill` (diagnostics).
    pub tlb_fills: u64,
    /// Translation blocks Unicorn translated (inline block accounting; diagnostics).
    pub tb_translations: u64,
    /// Block accounting is done inline by a SIMEMU-patched Unicorn (see `cpu_hooks::classify_block`).
    pub inline_block_accounting: bool,
    /// Unicorn exact instruction timing (`uc_ctl_simemu_exact`): peripheral accesses see the exact
    /// instruction count and end their block, quanta end at exactly their budget (as the native core).
    pub exact_timing: bool,
    /// Block hook -> patched Unicorn: run the block about to start limited to this many instructions.
    pub(crate) truncate: u64,
    /// Clock segment (see `Tc1791HardwareState::time_at`): start time, start instruction count, fCPU.
    pub(crate) clk_t0: f64,
    pub(crate) clk_n0: u64,
    pub(crate) clk_f: u64,
    /// Number of execution quanta (uc_emu_start calls) and how they ended.
    pub quanta: u64,
    pub quanta_stopped_early: u64,
    /// Reset history: (kind, STM ticks, total instructions).
    pub reset_log: Vec<(crate::scu::ResetKind, u64, u64)>,

    // ---- Telemetry ----
    pub sw_reset_count: u32,
    pub reset_count: u32,
    pub milestones: Vec<MilestoneEvent>,
    pub rtos_task_counts: BTreeMap<u32, u64>,
    pub rtos_isr_counts: BTreeMap<u32, u64>,
    pub can_tx_queue: Vec<CanFrame>,
    pub mmio_read_count: u64,
    pub mmio_write_count: u64,
    /// Record/replay differential harness (`tricore-core/tests/board_replay.rs`): while `Some`,
    /// every CPU MMIO access and flash command store is appended as (is_write, addr, size, value,
    /// ENDINIT after the access). Observation only.
    pub mmio_log: std::cell::RefCell<Option<Vec<MmioLogEntry>>>,
    /// Backend comparison (`tests/backend_lockstep.rs`): while `Some`, every peripheral access and
    /// interrupt entry as (instruction count, kind 0 read / 1 write / 2 interrupt, address or
    /// interrupted PC, value or SRPN), and every quantum end (kind 3, instructions executed,
    /// budget). Observation only.
    pub event_log: Option<Vec<(u64, u8, u32, u64)>>,
    /// Inclusive `[lo, hi]` guest address ranges observed for data reads/writes (`Native` + `trace-data-stores`).
    pub watch_read_ranges: Vec<(u32, u32)>,
    pub watch_write_ranges: Vec<(u32, u32)>,
    /// Bounded ring buffer of observed data read/write events (never guest-accessible).
    pub watch_events: VecDeque<DataWatchEvent>,
    /// Optional inclusive `[lo, hi]` target PC filters for call/jump tracing (empty = all targets).
    pub call_trace_ranges: Vec<(u32, u32)>,
    /// Optional inclusive `[lo, hi]` PC ranges for block-entry counting (`block_hits`).
    pub block_hit_ranges: Vec<(u32, u32)>,
    pub block_hits: BTreeMap<u32, u64>,
    pub block_sequence: Vec<(u32, u32, u32, u64)>,
    pub irq_dispatch_count: u64,
    pub trap_count: u64,

    // ---- DQ200 (TC1784) board peripherals (SSC/EEPROM, DMA, ADC sources), see `tc1784_io` ----
    pub tc1784: crate::tc1784_io::Tc1784Io,

    /// TC1766-only peripherals of the DQ250 board (`Some` only for `BoardVariant::Dq250Tc1766`).
    pub dq250: Option<Box<crate::tcu_dq250::Tc1766Periph>>,
    /// SIMOS18.1 GPTA + PCP, crank/cam inputs and output taps (`engine_io`).
    pub engine_io: crate::engine_io::EngineIo,
    /// FADC (knock-sensor acquisition, `knock_fadc`).
    pub fadc: crate::knock_fadc::Fadc,
    /// SENT receiver module and the sensors on its inputs (`sent`).
    pub sent: crate::sent::Sent,
}

impl Tc1791HardwareState {
    #[inline]
    pub(crate) fn is_ram(&self, addr: u32) -> bool {
        let a = addr as u64;
        self.ram_ranges.iter().any(|&(b, e)| a >= b as u64 && a < e)
    }

    /// Host-backed non-MMIO readable slice `[addr, addr + len)` (RAM, PFlash, DFlash, BROM).
    pub(crate) fn sys_mem_slice(&self, addr: u32, len: usize) -> Option<&[u8]> {
        let (buf, off): (&[u8], usize) = match (self.variant, addr >> 24) {
            (_, 0xD0) => (&self.dspr, (addr & 0x00FF_FFFF) as usize),
            (BoardVariant::Simos18Tc1791 | BoardVariant::Dq200Tc1784, 0xC0)
            | (BoardVariant::Dq200Tc1784 | BoardVariant::Dq250Tc1766, 0xD4) => {
                (&self.pspr, (addr & 0x00FF_FFFF) as usize)
            }
            (BoardVariant::Simos18Tc1791 | BoardVariant::Dq200Tc1784, 0x90 | 0xB0) => {
                (&self.lmu, (addr & 0x00FF_FFFF) as usize)
            }
            (BoardVariant::Simos18Tc1791, 0xAF) => match addr & 0x00FF_FFFF {
                a @ 0x00_0000..=0x01_7FFF => (&self.dflash[..crate::flash::DFLASH_BANK], a as usize),
                a @ 0x08_0000..=0x09_7FFF => (&self.dflash[crate::flash::DFLASH_BANK..], (a - 0x08_0000) as usize),
                _ if (addr & 0x00FF_0000) == 0x00FF_0000 => (self.code_bytes(addr)?, 0),
                _ => return None,
            },
            (BoardVariant::Dq200Tc1784, 0x8F | 0xAF) if (addr & 0x00FF_0000) < 0x0010_0000 => {
                (&self.dflash, (addr & 0x000F_FFFF) as usize)
            }
            (BoardVariant::Dq250Tc1766, 0xAF) if (0xAFE0_0000..0xAFE0_0000 + self.dflash.len() as u32).contains(&addr) => {
                (&self.dflash, (addr - 0xAFE0_0000) as usize)
            }
            _ => (self.code_bytes(addr)?, 0),
        };
        buf.get(off..off.checked_add(len)?)
    }

    /// Host-backed writable RAM slice `[addr, addr + len)` (DSPR/LDRAM, PSPR/SPRAM, LMU).
    pub(crate) fn sys_ram_mut(&mut self, addr: u32, len: usize) -> Option<&mut [u8]> {
        let (buf, off): (&mut [u8], usize) = match (self.variant, addr >> 24) {
            (_, 0xD0) => (&mut self.dspr, (addr & 0x00FF_FFFF) as usize),
            (BoardVariant::Simos18Tc1791 | BoardVariant::Dq200Tc1784, 0xC0)
            | (BoardVariant::Dq200Tc1784 | BoardVariant::Dq250Tc1766, 0xD4) => {
                (&mut self.pspr, (addr & 0x00FF_FFFF) as usize)
            }
            (BoardVariant::Simos18Tc1791 | BoardVariant::Dq200Tc1784, 0x90 | 0xB0) => {
                (&mut self.lmu, (addr & 0x00FF_FFFF) as usize)
            }
            _ => return None,
        };
        buf.get_mut(off..off.checked_add(len)?)
    }

    /// Record a non-CPU (DMA / PCP) write into RAM `[addr, addr + len)` for code-cache invalidation
    /// when the affected region may hold cached instructions (all aliases included).
    pub(crate) fn mark_ram_write_code_dirty(&mut self, addr: u32, len: u32) {
        if len == 0 {
            return;
        }
        let mut push_one = |base: u32| {
            if self.tb_cache.is_empty() {
                // Native backend: coalesce contiguous/overlapping DMA writes so high-rate streams
                // (e.g. 891 kHz knock FADC DMA) record at most one span per quantum, then
                // `Cpu::invalidate_code` checks `F_CODE` / `r.code` in O(1).
                if let Some((last_base, last_len)) = self.pflash_code_dirty.last_mut() {
                    let last_end = (*last_base as u64) + (*last_len as u64);
                    let new_end = (base as u64) + (len as u64);
                    if (base as u64) >= (*last_base as u64) && (base as u64) <= last_end {
                        *last_len = (new_end.max(last_end) - (*last_base as u64)) as u32;
                        return;
                    }
                }
                self.pflash_code_dirty.push((base, len));
            } else {
                // Unicorn backend: only RAM pages in `code_pages` can hold translated TBs.
                let p0 = base & !self.page_mask;
                let p1 = base.wrapping_add(len - 1) & !self.page_mask;
                if self.code_pages.iter().any(|&(p, _)| p == p0 || p == p1) {
                    self.pflash_code_dirty.push((base, len));
                    self.tb_cache.fill(crate::cpu_hooks::TbCacheEntry::EMPTY);
                }
            }
        };
        push_one(addr);
        match (self.variant, addr >> 24) {
            (BoardVariant::Dq200Tc1784, 0xC0) => push_one(0xD400_0000 | (addr & 0x00FF_FFFF)),
            (BoardVariant::Dq200Tc1784, 0xD4) => push_one(0xC000_0000 | (addr & 0x00FF_FFFF)),
            (BoardVariant::Simos18Tc1791 | BoardVariant::Dq200Tc1784, 0x90) => push_one(0xB000_0000 | (addr & 0x00FF_FFFF)),
            (BoardVariant::Simos18Tc1791 | BoardVariant::Dq200Tc1784, 0xB0) => push_one(0x9000_0000 | (addr & 0x00FF_FFFF)),
            _ => {}
        }
    }

    /// Read-only view of the STM 64-bit counter (ticks of fSTM).
    pub fn stm_ticks(&self) -> u64 {
        self.stm_now()
    }

    #[inline]
    pub fn read_f000_u32(&self, addr: u32) -> u32 {
        self.seg_f000.get(addr)
    }

    /// Raw register-file store (no peripheral side effects except keeping SRC bookkeeping
    /// coherent). Used by legacy hooks.
    #[inline]
    pub fn write_f000_u32(&mut self, addr: u32, val: u32) {
        if crate::stm_icu::is_src(addr) {
            self.src_put(addr, val);
        } else {
            self.seg_f000.set(addr, val);
        }
    }

    #[inline]
    pub fn read_f7e0_u32(&self, addr: u32) -> u32 {
        self.seg_f7e0.get(addr)
    }

    #[inline]
    pub fn write_f7e0_u32(&mut self, addr: u32, val: u32) {
        if crate::stm_icu::is_src(addr) {
            self.src_put(addr, val);
        } else {
            self.seg_f7e0.set(addr, val);
        }
    }

    #[inline]
    pub fn read_dspr_u32(&self, addr: u32) -> u32 {
        read_le32(&self.dspr, (addr & 0x000F_FFFF) as usize)
    }

    #[inline]
    pub fn write_dspr_u32(&mut self, addr: u32, val: u32) {
        write_le32(&mut self.dspr, (addr & 0x000F_FFFF) as usize, val);
    }

    #[inline]
    pub fn read_lmu_u32(&self, addr: u32) -> u32 {
        read_le32(&self.lmu, (addr & 0x000F_FFFF) as usize)
    }

    #[inline]
    pub fn write_lmu_u32(&mut self, addr: u32, val: u32) {
        write_le32(&mut self.lmu, (addr & 0x000F_FFFF) as usize, val);
    }

    /// Host view of guest code bytes starting at `addr` (flash/PSPR/DSPR/BROM), for instruction
    /// decoding by the block hook. `None` if `addr` is outside known executable memories.
    pub(crate) fn code_bytes(&self, addr: u32) -> Option<&[u8]> {
        let (buf, off): (&[u8], u32) = match (self.variant, addr >> 24) {
            (BoardVariant::Simos18Tc1791, 0x80 | 0xA0) => {
                let a = addr & 0x00FF_FFFF;
                match a {
                    0x00_0000..=0x1F_FFFF => (&self.flash[..0x20_0000], a),
                    0x80_0000..=0x9F_FFFF => (&self.flash[0x20_0000..], a - 0x80_0000),
                    _ => return None,
                }
            }
            (BoardVariant::Dq200Tc1784 | BoardVariant::Dq250Tc1766, 0x80 | 0xA0) => (&self.flash[..], addr & 0x00FF_FFFF),
            (_, 0xC0) => (&self.pspr[..], addr & 0x00FF_FFFF),
            (_, 0xD0) => (&self.dspr[..], addr & 0x00FF_FFFF),
            (BoardVariant::Dq200Tc1784 | BoardVariant::Dq250Tc1766, 0xD4) => (&self.pspr[..], addr & 0x00FF_FFFF),
            (_, 0x8F | 0xAF) if (0x1_0000 - (addr & 0xFFFF)) as usize <= self.brom.len() && addr & 0x00FF_0000 == 0x00FF_0000 => {
                (&self.brom[..], (addr & 0xFFFF) - (0x1_0000 - self.brom.len() as u32))
            }
            _ => return None,
        };
        buf.get(off as usize..)
    }
}

#[inline]
pub(crate) fn read_le32(b: &[u8], off: usize) -> u32 {
    match b.get(off..off + 4) {
        Some(s) => u32::from_le_bytes([s[0], s[1], s[2], s[3]]),
        None => 0,
    }
}

#[inline]
pub(crate) fn write_le32(b: &mut [u8], off: usize, val: u32) {
    if let Some(s) = b.get_mut(off..off + 4) {
        s.copy_from_slice(&val.to_le_bytes());
    }
}
