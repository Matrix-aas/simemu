//! Infineon TriCore TC1791 (AUDO MAX, TriCore 1.6) SoC + Continental SIMOS18.1 board emulator.
//!
//! The firmware runs unmodified in Unicorn 2.1.x. Hardware is modelled at register level:
//! - [`memory`]   — guest memories (zero-copy host buffers), shared hardware state.
//! - [`stm_icu`]  — 64-bit STM, SRC registers, interrupt arbitration and TriCore interrupt entry.
//! - [`scu`]      — PLL/oscillator status, reset control, watchdog with ENDINIT protection, ports.
//! - [`multican`] — MultiCAN nodes, message objects, list panel.
//! - [`flash`]    — PMU/flash command state machine (DFlash program/erase).
//! - [`adc`]      — ADC0..2 auto-scan conversion of analog input voltages.
//! - [`isa_fix`]  — generic corrections for Unicorn TriCore translator defects.
//! - [`native`]   — the alternative CPU backend: `tricore-core` instead of Unicorn ([`CpuBackend`],
//!   `SIMEMU_CPU=native|unicorn`).
//! - [`ssc`], [`dma`], [`msc`] — serial channels, DMA and the Micro Second Channel.
//! - [`monitor_device`] — processor-monitoring unit (MU, a function block of ATIC139) on SSC1.
//! - [`atic`], [`atic139`], [`power_stages`] — ATIC140/121/111 power-stage drivers, ATIC139 LS/PS
//!   blocks, disable lines, loads and the read-only output view.
//! - [`pic_eot`] — engine-off timer (PIC12LF1822) on SSC0.
//! - [`atic142`] — lambda sensor interface ASIC (wide-band + binary sensor) on SSC1.
//! - [`cpu_hooks`]— Unicorn callbacks (MMIO, flash cycles, block accounting, probes).
//! - [`sensors_io`] — board-level analog/digital input API, live calibration, real-time stepping.
//! - [`tcu_dq200`] — DQ200_G2 (TC1784) TCU board.
//! - [`tcu_dq250`] — DQ250 (02E/0D9) TCU board, Infineon TC1766 (AUDO-NG).

pub mod adc;
pub mod atic;
pub mod atic139;
pub mod atic142;
pub mod cpu_hooks;
pub mod dma;
pub mod dq250_io;
pub mod engine_io;
pub mod knock_fadc;
pub mod flash;
pub mod isa_fix;
pub mod memory;
pub mod monitor_device;
pub mod msc;
pub mod multican;
pub mod native;
pub mod power_stages;
pub mod scu;
pub mod sent;
pub mod sensors_io;
pub mod crank_wheel;
pub mod gpta;
#[cfg(test)]
mod gpta_ref;
mod gpta_trace;
pub mod pcp;
pub mod pic_eot;
pub mod st95640;
pub mod ssc;
pub mod stm_icu;
pub mod tc1766_adc;
pub mod tc1784_io;
pub mod tcu_dq200;
pub mod tcu_dq250;

pub use cpu_hooks::Probe;
pub use memory::{BoardVariant, CanFrame, DataWatchEvent, DataWatchKind, MilestoneEvent, Tc1791HardwareState};
pub use native::{parse_watch_range, CpuBackend};
pub use scu::ResetKind;
pub use tcu_dq200::{Dq200LiveState, Dq200Outputs, Dq200TcuBoard, Dq200TcuImageInfo, Wfs5ImmoPartner};
pub use tcu_dq250::{Dq250ImageInfo, Dq250Inputs, Dq250Outputs, Dq250TcuBoard};

use memory::{HostBuf, MmioSeg};
use serde::Serialize;
use std::collections::BTreeMap;
use std::ffi::{c_int, c_void, CStr};
use std::ptr::NonNull;
use std::time::Instant;
use stm_icu::CYCLES_PER_INSN;
use unicorn_tricore_sys::*;

/// Maximum instructions per execution quantum. Interrupts are delivered at quantum boundaries,
/// so this bounds the interrupt latency (2000 instructions = 11 us at 180 MHz). Quanta are also
/// cut at the next STM compare / ADC completion so timer interrupts are exact.
pub const QUANTUM_MAX_INSNS: u64 = 2000;

/// A RAM page is returned to plain data RAM after this many instructions without code execution
/// from it (its translations are dropped; see `cpu_hooks::hook_tlb_fill`).
const RAM_CODE_FLUSH_INSNS: u64 = 1_000_000;

/// TC1791 start address after reset for "internal start from flash" (non-cached PFlash).
pub const SIMOS_RESET_VECTOR: u32 = 0xA000_0000;

/// Honest execution statistics of a run.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct RunStats {
    pub pc: u32,
    pub instructions: u64,
    /// Simulated time = instructions / (fCPU * IPC).
    pub sim_time_s: f64,
    pub wall_time_s: f64,
    /// sim_time / wall_time (< 1 means slower than real time).
    pub real_time_factor: f64,
    pub mips: f64,
    pub resets: u32,
    pub interrupts: u64,
}

pub struct Simos18Board {
    pub(crate) uc: *mut uc_engine,
    /// Owned (`Box::into_raw`) hardware state wrapped in `UnsafeCell`; Unicorn C callbacks and
    /// `HwBus` hold the inner `*mut Tc1791HardwareState` while emulation runs under `&mut self`.
    hw: NonNull<std::cell::UnsafeCell<Tc1791HardwareState>>,
    variant: BoardVariant,
    hooks: Vec<uc_hook>,
    /// 12-byte unique chip ID of the emulated device (SIMOS: the copy the supplier bootloader
    /// stored at flash 0x80014200 for this very chip).
    chip_uid: Option<[u8; 12]>,
    /// `Some`: the native core runs the firmware and `uc` is null (see [`native`]).
    native: Option<native::NativeCell>,
}

// SAFETY: the engine and the state are exclusively owned by the board and only used from the
// thread that currently owns it.
unsafe impl Send for Simos18Board {}

fn uc_check(err: uc_err, what: &str) -> Result<(), String> {
    if err == UC_ERR_OK {
        Ok(())
    } else {
        Err(format!("{} failed: {}", what, Simos18Board::strerror(err)))
    }
}

/// Code locations of the SC800S50 software that legacy observation/supervisor hooks are keyed to.
/// They are only active where the executed bytes equal the SC800S50 bytes, so other software
/// versions are never affected.
#[derive(Clone, Copy)]
enum SigAction {
    Milestone(&'static str),
    TaskDispatch,
    IsrDispatch,
    IsrExit,
}

struct SigHook {
    addr: u32,
    sig: [u8; 8],
    action: SigAction,
}

const S50_SIG_HOOKS: &[SigHook] = &[
    // RTA-OS kernel in PSPR (copied at runtime): checked when executed (hot path: first in list).
    SigHook { addr: 0xC000_2FFA, sig: S50_PSPR_SIG_ISR, action: SigAction::IsrDispatch },
    SigHook { addr: 0xC000_303C, sig: S50_PSPR_SIG_ISR_EXIT, action: SigAction::IsrExit },
    SigHook { addr: 0xC000_3300, sig: S50_PSPR_SIG_TASK, action: SigAction::TaskDispatch },
    SigHook { addr: 0xC000_3144, sig: S50_PSPR_SIG_START_OS, action: SigAction::Milestone("S50_RTOS_START_OS") },
    SigHook { addr: 0x8000_5354, sig: [0x91, 0x00, 0x00, 0xf8, 0xd9, 0xff, 0x80, 0xd1], action: SigAction::Milestone("S50_SBOOT_MAIN") },
    SigHook { addr: 0x8001_CAE8, sig: [0x6d, 0x00, 0xc0, 0x05, 0x0d, 0x00, 0xc0, 0x04], action: SigAction::Milestone("S50_CBOOT_ENTRY") },
    SigHook { addr: 0x8010_860C, sig: [0x91, 0x60, 0x00, 0xf8, 0xd9, 0xff, 0xb0, 0xee], action: SigAction::Milestone("S50_ASW_STARTUP") },
    SigHook { addr: 0x8008_077C, sig: [0x61, 0x00, 0x62, 0x1e, 0x82, 0x04, 0x40, 0x44], action: SigAction::Milestone("S50_FLASH_CHK_FAIL") },
    SigHook { addr: 0x8006_D120, sig: [0x20, 0x08, 0x02, 0x48, 0x49, 0xa4, 0x00, 0x0a], action: SigAction::Milestone("S50_MOC_RESET_REQ") },
    SigHook { addr: 0x8010_F034, sig: [0x91, 0x00, 00, 0xfb, 0xd9, 0xff, 0x38, 0x00], action: SigAction::Milestone("S50_ASW_CRASH_HANDLER") },
    SigHook { addr: 0x8010_E2B0, sig: [0x40, 0x4f, 0xa5, 0xdf, 0x74, 0xb8, 0x20, 0x20], action: SigAction::Milestone("S50_ASW_FADC_INIT") },
    SigHook { addr: 0x8011_24F4, sig: [0x40, 0x4f, 0xd9, 0x9c, 0x0c, 0x3e, 0xc8, 0x22], action: SigAction::Milestone("S50_ASW_GPTA_INIT") },
    SigHook { addr: 0x8011_44EC, sig: [0xbb, 0x00, 0x40, 0x8f, 0xc5, 0xf4, 0x3c, 0xb0], action: SigAction::Milestone("S50_ASW_STM_CMP0_INIT") },
];
const S50_PSPR_SIG_START_OS: [u8; 8] = [0x02, 0x48, 0x4d, 0x40, 0xe0, 0xff, 0xbb, 0x00];
const S50_PSPR_SIG_TASK: [u8; 8] = [0x2d, 0x02, 0x00, 0x00, 0x0d, 0x00, 0x40, 0x03];
const S50_PSPR_SIG_ISR: [u8; 8] = [0x2d, 0x0f, 0x00, 0x00, 0xf4, 0xcd, 0x0d, 0x00];
const S50_PSPR_SIG_ISR_EXIT: [u8; 8] = [0x0d, 0x00, 0x40, 0x02, 0x00, 0x00, 0x00, 0x80];

extern "C" fn hook_sig(uc: *mut uc_engine, address: u64, _size: u32, ud: *mut c_void) {
    // SAFETY: `ud` is the board's hardware state (see cpu_hooks module comment).
    let hw = unsafe { &mut *(ud as *mut Tc1791HardwareState) };
    // The block hook counted the whole block (`classify_block` keeps blocks with signature hooks on
    // the hook path, which records the block end): time stamps at the hooked instruction itself.
    let back = hw.insns_between(address as u32, hw.fixes.tb_end).min(hw.quantum_executed);
    hw.quantum_executed -= back;
    sig_hit(hw, address as u32, |r: TriCoreReg| {
        let mut v = 0u32;
        // SAFETY: valid engine.
        unsafe { uc_reg_read(uc, r as c_int, (&mut v as *mut u32).cast()) };
        v
    });
    hw.quantum_executed += back;
}

/// Signature hook at `pc` (any backend): verify the code bytes, then record the action.
fn sig_hit(hw: &mut Tc1791HardwareState, pc: u32, reg: impl Fn(TriCoreReg) -> u32) {
    let Some(h) = S50_SIG_HOOKS.iter().find(|h| h.addr == pc) else { return };
    match hw.code_bytes(pc) {
        Some(b) if b.len() >= 8 && b[..8] == h.sig => {}
        _ => return,
    }
    match h.action {
        SigAction::Milestone(name) => {
            let ticks = hw.stm_now();
            let pc_rec = if name == "S50_FLASH_CHK_FAIL" || name == "S50_MOC_RESET_REQ" || name == "S50_ASW_CRASH_HANDLER" {
                reg(TriCoreReg::A11)
            } else {
                pc
            };
            hw.milestones.push(MilestoneEvent { reset_cycle: hw.reset_count, name, pc: pc_rec, stm_ticks: ticks });
        }
        SigAction::TaskDispatch => *hw.rtos_task_counts.entry(reg(TriCoreReg::A2)).or_insert(0) += 1,
        SigAction::IsrDispatch => *hw.rtos_isr_counts.entry(reg(TriCoreReg::A15)).or_insert(0) += 1,
        SigAction::IsrExit => *hw.rtos_isr_counts.entry(0xC000_303C).or_insert(0) += 1,
    }
}

impl Simos18Board {
    pub fn new(flash_bytes: &[u8], eeprom_bytes: &[u8]) -> Result<Self, String> {
        Self::new_variant(flash_bytes, eeprom_bytes, BoardVariant::Simos18Tc1791)
    }

    /// [`Self::new`] on an explicit CPU backend.
    pub fn new_on(flash_bytes: &[u8], eeprom_bytes: &[u8], backend: CpuBackend) -> Result<Self, String> {
        Self::new_variant_on(flash_bytes, eeprom_bytes, BoardVariant::Simos18Tc1791, backend)
    }

    /// `simos18_code_hooks == false` selects the DQ200 (TC1784) memory map.
    pub fn new_with_mode(flash_bytes: &[u8], eeprom_bytes: &[u8], simos18_code_hooks: bool) -> Result<Self, String> {
        let v = if simos18_code_hooks { BoardVariant::Simos18Tc1791 } else { BoardVariant::Dq200Tc1784 };
        Self::new_variant(flash_bytes, eeprom_bytes, v)
    }

    /// CPU backend from `SIMEMU_CPU` ([`CpuBackend::from_env`]).
    pub fn new_variant(flash_bytes: &[u8], eeprom_bytes: &[u8], variant: BoardVariant) -> Result<Self, String> {
        Self::new_variant_on(flash_bytes, eeprom_bytes, variant, CpuBackend::from_env())
    }

    pub fn new_variant_on(flash_bytes: &[u8], eeprom_bytes: &[u8], variant: BoardVariant, backend: CpuBackend) -> Result<Self, String> {
        let dq250 = variant == BoardVariant::Dq250Tc1766;
        let flash_len = if dq250 { tcu_dq250::PFLASH_SIZE } else { 0x40_0000 };
        if flash_bytes.len() < flash_len {
            return Err(format!("Expected 0x{flash_len:X} byte Flash buffer, got {} bytes", flash_bytes.len()));
        }
        let mut uc: *mut uc_engine = std::ptr::null_mut();
        let mut page_size = 0x1000u32;
        if backend == CpuBackend::Unicorn {
            if !unicorn_tricore_sys::LINKED {
                return Err(
                    "Unicorn CPU backend is not compiled in (SIMEMU uses the pure-Rust `native` core by default; rebuild with `--features unicorn` for differential testing)".into(),
                );
            }
            // SAFETY: out-pointer is valid.
            uc_check(unsafe { uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc) }, "uc_open")?;
            // SAFETY: valid engine, out-pointer valid; TLB mode is selected before any emulation.
            let init_res = unsafe {
                uc_check(uc_ctl_get_page_size(uc, &mut page_size), "uc_ctl_get_page_size")
                    .and_then(|()| uc_check(uc_ctl_tlb_mode(uc, UC_TLB_VIRTUAL), "uc_ctl_tlb_mode"))
            };
            if let Err(e) = init_res {
                // SAFETY: `uc_open` succeeded above; close before returning so `uc` does not leak.
                unsafe { uc_close(uc) };
                return Err(e);
            }
        }

        let simos = variant == BoardVariant::Simos18Tc1791;
        let dflash = if dq250 {
            HostBuf::new(tcu_dq250::DFLASH_SIZE, 0)
        } else if simos {
            // Bank 0 = bytes[0..0x18000] @ 0xAF000000, bank 1 = bytes[0x18000..0x30000] @ 0xAF080000 (the
            // layout of `hw.dflash`, so a saved DFlash powers up again as it was). The 256 KB bench dump
            // holds bank 0 only; its bytes 0x18000.. are zero, so bank 1 comes up erased.
            HostBuf::from_slice(&eeprom_bytes[..eeprom_bytes.len().min(0x3_0000)], 0x3_0000, 0)
        } else {
            HostBuf::from_slice(eeprom_bytes, 0x10_0000, 0)
        };
        // BROM is not available: every word is the stub `mov d2,#0 ; ret` (82 02 00 90).
        let mut brom = HostBuf::new(if simos { 0x4000 } else { 0x1_0000 }, 0);
        for c in brom.as_chunks_mut::<4>().0 {
            *c = [0x82, 0x02, 0x00, 0x90];
        }
        let ram = |simos_len: usize, dq250_len: usize| {
            HostBuf::new(if simos { simos_len } else if dq250 { dq250_len } else { 0x10_0000 }, 0)
        };
        let state = Tc1791HardwareState {
            variant,
            flash: HostBuf::from_slice(&flash_bytes[..flash_len], flash_len, 0),
            dflash,
            brom,
            dspr: ram(0x2_0000, tcu_dq250::LDRAM_SIZE),
            pspr: ram(0x8000, tcu_dq250::SPRAM_SIZE),
            lmu: ram(0x2_0000, 0x1000),
            seg_f000: MmioSeg::new(0xF000_0000, 0x40_0000),
            seg_f7e0: MmioSeg::new(0xF7E0_0000, 0x2_0000),
            seg_f800: MmioSeg::new(0xF800_0000, 0x10_0000),
            seg_f87f: MmioSeg::new(0xF87F_0000, 0x1_0000),
            stm: Default::default(),
            scu: scu::ScuWdt::new(ResetKind::PowerOn),
            can: Default::default(),
            flashc: Default::default(),
            adc: Default::default(),
            spi: Default::default(),
            fixes: Default::default(),
            quantum_budget: 0,
            quantum_executed: 0,
            stm_folded: 0,
            total_instructions: 0,
            sim_time_s: 0.0,
            stop_requested: false,
            reset_request: None,
            last_tb_pc: 0,
            swrst_log: Vec::new(),
            last_bad_access: None,
            regions_seen: 0,
            probes: Vec::new(),
            src_active: Vec::new(),
            tb_cache: if uc.is_null() { Vec::new() } else { vec![cpu_hooks::TbCacheEntry::EMPTY; cpu_hooks::TB_CACHE_SIZE] },
            ram_ranges: Vec::new(),
            page_mask: page_size.max(1) - 1,
            code_pages: Vec::new(),
            pflash_code_dirty: Vec::new(),
            tlb_fills: 0,
            tb_translations: 0,
            inline_block_accounting: false,
            exact_timing: false,
            truncate: 0,
            clk_t0: 0.0,
            clk_n0: 0,
            clk_f: 0,
            pipn_published: 0,
            quanta: 0,
            quanta_stopped_early: 0,
            reset_log: Vec::new(),
            // Off: SSC/DMA/MU are modelled (`ssc`, `dma`, `monitor_device`); the legacy fake would
            // complete the same transfers a second time.
            sw_reset_count: 0,
            reset_count: 0,
            milestones: Vec::with_capacity(64),
            rtos_task_counts: BTreeMap::new(),
            rtos_isr_counts: BTreeMap::new(),
            can_tx_queue: Vec::with_capacity(1024),
            mmio_read_count: 0,
            mmio_write_count: 0,
            mmio_log: Default::default(),
            event_log: None,
            watch_read_ranges: Vec::new(),
            watch_write_ranges: Vec::new(),
            watch_events: std::collections::VecDeque::new(),
            call_trace_ranges: Vec::new(),
            block_hit_ranges: Vec::new(),
            block_hits: BTreeMap::new(),
            block_sequence: Vec::new(),
            irq_dispatch_count: 0,
            trap_count: 0,
            tc1784: tc1784_io::Tc1784Io::new(if simos { &[] } else { eeprom_bytes }),
            dq250: dq250.then(|| Box::new(tcu_dq250::Tc1766Periph::new(eeprom_bytes))),
            engine_io: Default::default(),
            fadc: Default::default(),
            sent: Default::default(),
        };
        let hw = NonNull::new(Box::into_raw(Box::new(std::cell::UnsafeCell::new(state)))).expect("non-null");
        let chip_uid = (variant == BoardVariant::Simos18Tc1791)
            .then(|| flash_bytes.get(0x1_4200..0x1_420C).and_then(|b| <[u8; 12]>::try_from(b).ok()))
            .flatten()
            .filter(|u| u.iter().any(|&b| b != 0 && b != 0xFF));
        let mut board = Self { uc, hw, variant, hooks: Vec::new(), chip_uid, native: None };
        if simos {
            board.hw_mut().flashc = flash::FlashCtrl::from_image(&flash_bytes[..flash_len]);
        }
        if backend == CpuBackend::Native {
            let inner = NonNull::new(std::cell::UnsafeCell::raw_get(hw.as_ptr())).expect("non-null");
            board.native = Some(native::new_core(inner));
        } else {
            board.map_memory()?;
            board.install_hooks()?;
            board.enable_inline_block_accounting();
            // SAFETY: valid engine, outside emulation.
            board.hw_mut().fixes.lib = unsafe { uc_ctl_simemu_isa_fixes(board.uc) };
            board.hw_mut().fixes.tc13 = variant == BoardVariant::Dq250Tc1766;
        }
        board.reset(ResetKind::PowerOn);
        board.hw_mut().reset_count = 0;
        board.hw_mut().reset_log.clear();
        if simos {
            // Switched 12 V inputs with the wire open: the ECU input stages hold the pins low (the
            // firmware reads a high level as active) — KL50 start request P4.2 (getter 0x800A5B04) and
            // starter feedback KL50R P1.9 (getter 0x800A5AFC, read into LV_ST_RESP by 0x800ECF28).
            board.set_port_input(4, 2, Some(false));
            board.set_port_input(1, 9, Some(false));
        }
        Ok(board)
    }

    pub(crate) fn strerror(err: uc_err) -> String {
        // SAFETY: uc_strerror returns a static C string or null.
        unsafe {
            let ptr = uc_strerror(err);
            if ptr.is_null() {
                format!("uc_err({})", err)
            } else {
                CStr::from_ptr(ptr).to_string_lossy().into_owned()
            }
        }
    }

    /// Read-only access to the hardware state.
    #[inline]
    pub fn hw(&self) -> &Tc1791HardwareState {
        // SAFETY: owned allocation, valid for the board's lifetime; `&self` prevents concurrent
        // emulation or `&mut` access (every `&self` method uses pure `&Tc1791HardwareState` reads).
        unsafe { &*self.hw.as_ref().get() }
    }

    #[inline]
    pub(crate) fn hw_mut(&mut self) -> &mut Tc1791HardwareState {
        // SAFETY: `&mut self` guarantees exclusivity outside of emulation.
        unsafe { self.hw.as_mut().get_mut() }
    }

    #[inline]
    fn hw_ptr(&self) -> *mut c_void {
        std::cell::UnsafeCell::raw_get(self.hw.as_ptr()).cast()
    }

    pub fn variant(&self) -> BoardVariant {
        self.variant
    }

    /// Diagnostics: the event recorder ([`Tc1791HardwareState::event_log`]); `Some` records.
    pub fn event_log_mut(&mut self) -> &mut Option<Vec<(u64, u8, u32, u64)>> {
        &mut self.hw_mut().event_log
    }

    /// Map `len` bytes of a host buffer at `addr`.
    fn map_raw(&mut self, addr: u32, ptr: *mut u8, len: usize, perms: u32) -> Result<(), String> {
        // SAFETY: `ptr..ptr+len` is a live HostBuf allocation owned by the state, which outlives
        // the engine (closed first in Drop).
        let err = unsafe { uc_mem_map_ptr(self.uc, addr as u64, len, perms, ptr.cast()) };
        uc_check(err, &format!("uc_mem_map_ptr(0x{:08X}, 0x{:X})", addr, len))?;
        if perms & UC_PROT_WRITE as u32 == 0 {
            // map_ptr does not mark the region read-only; protect() does, so stores are discarded
            // after the WRITE_PROT hook has seen them.
            // SAFETY: region just mapped.
            uc_check(unsafe { uc_mem_protect(self.uc, addr as u64, len, perms) }, "uc_mem_protect")?;
        } else {
            self.hw_mut().ram_ranges.push((addr, addr as u64 + len as u64));
        }
        Ok(())
    }

    /// `[lo, hi]` without the RAM ranges, as inclusive hook ranges. Memory hooks must not cover
    /// RAM: Unicorn keeps every page with an installed memory hook on the store slow path.
    fn outside_ram(&self, lo: u32, hi: u32) -> Vec<(u64, u64)> {
        let mut ram = self.hw().ram_ranges.clone();
        ram.sort_unstable();
        let (mut out, mut cur) = (Vec::new(), lo as u64);
        for (b, e) in ram {
            let b = b as u64;
            if e <= cur || b > hi as u64 {
                continue;
            }
            if b > cur {
                out.push((cur, b - 1));
            }
            cur = e;
        }
        if cur <= hi as u64 {
            out.push((cur, hi as u64));
        }
        out
    }

    fn map_memory(&mut self) -> Result<(), String> {
        let rx = (UC_PROT_READ | UC_PROT_EXEC) as u32;
        let ro = UC_PROT_READ as u32;
        let rwx = UC_PROT_ALL as u32;
        let hw = self.hw();
        let (fl, df, br, ds, ps, lm) = (hw.flash.raw(), hw.dflash.raw(), hw.brom.raw(), hw.dspr.raw(), hw.pspr.raw(), hw.lmu.raw());
        let (dflen, brlen, dslen, pslen, lmlen) = (hw.dflash.len(), hw.brom.len(), hw.dspr.len(), hw.pspr.len(), hw.lmu.len());
        match self.variant {
            BoardVariant::Simos18Tc1791 => {
                // PFlash0 / PFlash1, cached and non-cached views share one backing store.
                for base in [0x8000_0000u32, 0xA000_0000] {
                    self.map_raw(base, fl, 0x20_0000, rx)?;
                    // SAFETY: offset within the 4 MB flash allocation.
                    self.map_raw(base + 0x80_0000, unsafe { fl.add(0x20_0000) }, 0x20_0000, rx)?;
                }
                // DFlash bank 0 / bank 1 (non-cached only).
                self.map_raw(0xAF00_0000, df, flash::DFLASH_BANK, ro)?;
                // SAFETY: second half of the 0x30000 DFlash allocation.
                self.map_raw(0xAF08_0000, unsafe { df.add(flash::DFLASH_BANK) }, flash::DFLASH_BANK, ro)?;
                self.map_raw(0xAFFF_C000, br, brlen, rx)?;
                self.map_raw(0x8FFF_C000, br, brlen, rx)?;
                self.map_raw(0xD000_0000, ds, dslen, rwx)?;
                self.map_raw(0xC000_0000, ps, pslen, rwx)?;
                self.map_raw(0xB000_0000, lm, lmlen, rwx)?;
                self.map_raw(0x9000_0000, lm, lmlen, rwx)?;
            }
            BoardVariant::Dq200Tc1784 => {
                // TC1784 memory map with read-only PFlash/DFlash/BROM and writable DSPR/PSPR/LMU.
                self.map_raw(0x8000_0000, fl, 0x40_0000, rx)?;
                self.map_raw(0xA000_0000, fl, 0x40_0000, rx)?;
                self.map_raw(0xAF00_0000, df, dflen, ro)?;
                self.map_raw(0x8F00_0000, df, dflen, ro)?;
                self.map_raw(0xAFFF_0000, br, brlen, rx)?;
                self.map_raw(0x8FFF_0000, br, brlen, rx)?;
                self.map_raw(0xD000_0000, ds, dslen, rwx)?;
                self.map_raw(0xD400_0000, ps, pslen, rwx)?;
                self.map_raw(0xC000_0000, ps, pslen, rwx)?;
                self.map_raw(0xB000_0000, lm, lmlen, rwx)?;
                self.map_raw(0x9000_0000, lm, lmlen, rwx)?;
                for (a, l) in [(0x0000_0000u64, 0x1_0000usize), (0xF040_0000, 0x07A0_0000), (0xF810_0000, 0x006F_0000), (0xF880_0000, 0x0780_0000)] {
                    // SAFETY: valid engine.
                    uc_check(unsafe { uc_mem_map(self.uc, a, l, rwx) }, "uc_mem_map")?;
                    self.hw_mut().ram_ranges.push((a as u32, a + l as u64));
                }
            }
            BoardVariant::Dq250Tc1766 => {
                // TC1766: PFlash cached/non-cached views, DFlash, LDRAM (DMI), SPRAM (PMI).
                // Everything else is unmapped (bus error), as on the chip.
                for base in [0x8000_0000u32, 0xA000_0000] {
                    self.map_raw(base, fl, tcu_dq250::PFLASH_SIZE, rx)?;
                }
                self.map_raw(tcu_dq250::DFLASH_BASE, df, dflen, ro)?;
                self.map_raw(0xD000_0000, ds, dslen, rwx)?;
                self.map_raw(0xD400_0000, ps, pslen, rwx)?;
            }
        }
        // Peripheral segments: every access goes through the register model.
        let ud = self.hw_ptr();
        use cpu_hooks::*;
        let segs: [(u32, usize, uc_cb_mmio_read_t, uc_cb_mmio_write_t); 4] = [
            (0xF000_0000, 0x40_0000, mmio_read_f000, mmio_write_f000),
            (0xF7E0_0000, 0x2_0000, mmio_read_f7e0, mmio_write_f7e0),
            (0xF800_0000, 0x10_0000, mmio_read_f800, mmio_write_f800),
            (0xF87F_0000, 0x1_0000, mmio_read_f87f, mmio_write_f87f),
        ];
        for (base, len, r, w) in segs {
            // SAFETY: callbacks have the declared C ABI; `ud` outlives the engine.
            let err = unsafe { uc_mmio_map(self.uc, base as u64, len, Some(r), ud, Some(w), ud) };
            uc_check(err, &format!("uc_mmio_map(0x{:08X})", base))?;
        }
        Ok(())
    }

    fn add_hook(&mut self, kind: uc_hook_type, cb: *mut c_void, begin: u64, end: u64) -> Result<uc_hook, String> {
        if self.uc.is_null() {
            return Err("Unicorn hooks need the Unicorn CPU backend".into());
        }
        let mut hh: uc_hook = 0;
        // SAFETY: valid engine, callback with the ABI matching `kind`, user data = state.
        let err = unsafe { uc_hook_add_basic(self.uc, &mut hh, kind, cb, self.hw_ptr(), begin, end) };
        uc_check(err, &format!("uc_hook_add(type {kind:#x}, {begin:#x})"))?;
        self.hooks.push(hh);
        Ok(hh)
    }

    fn install_hooks(&mut self) -> Result<(), String> {
        self.add_hook(UC_HOOK_BLOCK, cpu_hooks::hook_block as *mut c_void, 1, 0)?;
        self.add_hook(UC_HOOK_TLB_FILL, cpu_hooks::hook_tlb_fill as *mut c_void, 1, 0)?;
        // RAM is always mapped and readable/writable/executable, so it can raise none of these
        // events; leaving it out of the hook ranges keeps RAM stores on the fast path.
        for (b, e) in self.outside_ram(0x8000_0000, 0xAFFF_FFFF) {
            self.add_hook(UC_HOOK_MEM_WRITE_PROT, cpu_hooks::hook_flash_write as *mut c_void, b, e)?;
        }
        for (b, e) in self.outside_ram(0, u32::MAX) {
            self.add_hook(
                UC_HOOK_MEM_READ_UNMAPPED | UC_HOOK_MEM_WRITE_UNMAPPED | UC_HOOK_MEM_FETCH_UNMAPPED | UC_HOOK_MEM_FETCH_PROT | UC_HOOK_MEM_READ_PROT,
                cpu_hooks::hook_bad_access as *mut c_void,
                b,
                e,
            )?;
        }
        if self.variant == BoardVariant::Simos18Tc1791 {
            for h in S50_SIG_HOOKS {
                // Flash locations are verified now; PSPR locations when executed.
                let ok = h.addr >= 0xC000_0000 || self.hw().code_bytes(h.addr).is_some_and(|b| b.len() >= 8 && b[..8] == h.sig);
                if ok {
                    self.add_hook(UC_HOOK_CODE, hook_sig as *mut c_void, h.addr as u64, h.addr as u64)?;
                }
            }
        }
        Ok(())
    }

    /// With a SIMEMU-patched Unicorn (`scripts/build_unicorn.py`), let the translated code do the
    /// block accounting of [`cpu_hooks::hook_block`] inline whenever its outcome is known; stock
    /// Unicorn rejects the control and every block keeps calling the hook. Same results either way.
    fn enable_inline_block_accounting(&mut self) {
        let hw = std::cell::UnsafeCell::raw_get(self.hw.as_ptr());
        // SAFETY: the pointers address fields of the boxed state, which outlives the engine.
        let cfg = unsafe {
            uc_simemu_bc {
                executed: std::ptr::addr_of_mut!((*hw).quantum_executed),
                limit: std::ptr::addr_of!((*hw).quantum_budget),
                stop: std::ptr::addr_of!((*hw).stop_requested).cast(),
                seen: std::ptr::addr_of!((*hw).regions_seen),
                last_pc: std::ptr::addr_of_mut!((*hw).last_tb_pc),
                classify: cpu_hooks::classify_block,
                user: hw.cast(),
            }
        };
        // SAFETY: valid engine, outside emulation; Unicorn copies `cfg`.
        let ok = unsafe { uc_ctl_simemu_block_counter(self.uc, &cfg) } == UC_ERR_OK;
        self.hw_mut().inline_block_accounting = ok;
        // Exact instruction timing, as the native core has it (`SIMEMU_UC_EXACT=0`: block-granular
        // timing of stock Unicorn, for comparisons). Stock Unicorn: unavailable, see ARCHITECTURE.md §3.1.
        if ok && std::env::var("SIMEMU_UC_EXACT").map_or(true, |v| v.trim() != "0") {
            // SAFETY: valid engine, outside emulation; `truncate` is a field of the boxed state.
            let exact = unsafe { uc_ctl_simemu_exact(self.uc, std::ptr::addr_of_mut!((*hw).truncate)) } == UC_ERR_OK;
            self.hw_mut().exact_timing = exact;
        }
    }

    /// Install an observation-only probe at `addr` (records hit count and registers).
    pub fn add_probe(&mut self, addr: u32) -> Result<(), String> {
        if self.native.is_some() {
            self.native_add_hook(addr);
        } else {
            self.add_hook(UC_HOOK_CODE, cpu_hooks::hook_probe as *mut c_void, addr as u64, addr as u64)?;
            // SAFETY: outside emulation.
            unsafe { uc_ctl_remove_cache(self.uc, addr as u64, addr as u64 + 4) };
        }
        self.hw_mut().probes.push(Probe { addr, hits: 0, last_regs: [0; 32] });
        Ok(())
    }

    /// Diagnostic, Unicorn backend only: log every data write into `[lo, hi]` to
    /// [`cpu_hooks::WRITE_WATCH`] (observation only, writes nothing).
    pub fn add_write_watch(&mut self, lo: u32, hi: u32) -> Result<(), String> {
        self.add_hook(UC_HOOK_MEM_WRITE, cpu_hooks::hook_write_watch as *mut c_void, lo as u64, hi as u64).map(|_| ())
    }

    pub fn probe(&self, addr: u32) -> Option<&Probe> {
        self.hw().probes.iter().find(|p| p.addr == addr)
    }

    #[inline]
    pub fn try_read_reg(&self, reg: TriCoreReg) -> Result<u32, String> {
        if let Some(c) = self.nat() {
            return Ok(Self::native_read_reg(c, reg));
        }
        let mut val: u32 = 0;
        // SAFETY: valid engine and out pointer.
        let err = unsafe { uc_reg_read(self.uc, reg as c_int, (&mut val as *mut u32).cast()) };
        uc_check(err, "uc_reg_read").map(|_| val)
    }

    /// Read a CPU register (register ids come from the enum, so failures cannot occur).
    #[inline]
    pub fn read_reg(&self, reg: TriCoreReg) -> u32 {
        let r = self.try_read_reg(reg);
        debug_assert!(r.is_ok(), "{r:?}");
        r.unwrap_or(0)
    }

    #[inline]
    pub(crate) fn write_reg(&mut self, reg: TriCoreReg, val: u32) {
        if let Some(c) = self.nat_mut() {
            return Self::native_write_reg(c, reg, val);
        }
        // SAFETY: valid engine and in pointer.
        let err = unsafe { uc_reg_write(self.uc, reg as c_int, (&val as *const u32).cast()) };
        debug_assert_eq!(err, UC_ERR_OK);
    }

    pub(crate) fn write_mem(&mut self, addr: u32, data: &[u8]) -> Result<(), String> {
        if self.native.is_some() {
            return self.native_write_mem(addr, data);
        }
        // SAFETY: valid engine; data slice valid for its length.
        let err = unsafe { uc_mem_write(self.uc, addr as u64, data.as_ptr().cast(), data.len()) };
        uc_check(err, &format!("uc_mem_write(0x{:08X}, {})", addr, data.len()))
    }

    pub(crate) fn read_u32(&self, addr: u32) -> u32 {
        if self.native.is_some()
            || memory::is_mmio(addr)
            || memory::is_mmio(addr.wrapping_add(3))
        {
            return self.read_mem(addr, 4).map_or(0, |b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]));
        }
        let mut b = [0u8; 4];
        // SAFETY: valid engine; non-MMIO address cannot invoke C MMIO callbacks; buffer of 4 bytes.
        unsafe { uc_mem_read(self.uc, addr as u64, b.as_mut_ptr().cast(), 4) };
        u32::from_le_bytes(b)
    }

    pub fn classify_region(pc: u32) -> &'static str {
        cpu_hooks::region_of(pc).1
    }

    /// Hardware reset of the SoC: CPU core registers/CSFRs, all SRCs, STM, MultiCAN, SCU/WDT,
    /// flash controller, ADC and peripheral register files return to their reset state. RAMs
    /// (DSPR/PSPR/LMU standby RAM) keep their contents. SCU_RSTSTAT reports `kind`.
    pub fn reset(&mut self, kind: ResetKind) {
        let reset_pc = match self.variant {
            BoardVariant::Simos18Tc1791 | BoardVariant::Dq250Tc1766 => SIMOS_RESET_VECTOR,
            BoardVariant::Dq200Tc1784 => 0x8000_0000,
        };
        let hw = self.hw_mut();
        let ticks = hw.stm_now();
        let total = hw.total_instructions;
        hw.reset_log.push((kind, ticks, total));
        hw.reset_count += 1;
        if kind == ResetKind::Software {
            hw.sw_reset_count += 1;
        }
        let (lvl, drv) = (hw.scu.port_ext_level, hw.scu.port_ext_driven);
        hw.scu = scu::ScuWdt::new(kind);
        hw.scu.port_ext_level = lvl;
        hw.scu.port_ext_driven = drv;
        hw.stm = Default::default();
        hw.can_reset();
        hw.flashc.reset();
        hw.adc.done_at = [None; 3];
        hw.adc.evflags = [0; 3];
        hw.spi.reset();
        for s in [&mut hw.seg_f000, &mut hw.seg_f7e0, &mut hw.seg_f800, &mut hw.seg_f87f] {
            s.regs.fill(0);
        }
        hw.src_reset_all();
        hw.tc1784_reset();
        hw.engine_io_reset();
        hw.fadc.soc_reset();
        hw.sent.soc_reset();
        if let Some(p) = hw.dq250.as_mut() {
            p.reset();
        }
        hw.fixes.reset_csfrs();
        hw.regions_seen = 0;
        hw.reset_request = None;
        hw.pipn_published = 0;
        hw.stop_requested = false;
        hw.quantum_executed = 0;
        hw.stm_folded = 0;
        hw.clock_rebase(); // the SCU is back at its reset clock
        for i in 0..16 {
            self.write_reg(TriCoreReg::d(i), 0);
            self.write_reg(TriCoreReg::a(i), 0);
        }
        self.set_psw(0x0000_0B80);
        self.write_reg(TriCoreReg::PCXI, 0);
        self.write_reg(TriCoreReg::ICR, 0);
        self.write_reg(TriCoreReg::BIV, 0);
        self.write_reg(TriCoreReg::BTV, 0xA000_0100);
        self.write_reg(TriCoreReg::ISP, 0x0000_0100);
        self.write_reg(TriCoreReg::FCX, 0);
        self.write_reg(TriCoreReg::LCX, 0);
        self.write_reg(TriCoreReg::SYSCON, 0);
        self.write_reg(TriCoreReg::PC, reset_pc);
        self.native_reset();
        self.flush_pflash_code_dirty();
        // Boot-ROM startup software: the application expects the device's unique chip ID at the
        // start of DSPR (it keys the DFlash `anvmcryptlight` records with it; see
        // docs/SIMOS18_WFS5_RE.md). No flash code writes it, so it is the BROM's job on every
        // reset. Behaviour inferred from the firmware's expectation — UNVERIFIED against a BROM dump.
        if let Some(uid) = self.chip_uid {
            let _ = self.write_mem(0xD000_0000, &uid);
        }
    }

    /// Invalidate cached translation blocks / predecoded code for any Flash or RAM ranges modified
    /// outside normal CPU stores (runtime Flash programming/erase or DMA/PCP code writes).
    pub(crate) fn flush_pflash_code_dirty(&mut self) {
        if self.hw().pflash_code_dirty.is_empty() {
            return;
        }
        let dirty = std::mem::take(&mut self.hw_mut().pflash_code_dirty);
        if let Some(c) = self.nat_mut() {
            for (addr, len) in dirty {
                c.invalidate_code(addr, len);
            }
        } else if !self.uc.is_null() {
            for (addr, len) in dirty {
                // SAFETY: outside emulation; drops translated TBs covering the modified range.
                unsafe { uc_ctl_remove_cache(self.uc, addr as u64, addr as u64 + len as u64) };
            }
        }
    }

    /// Work done between quanta: install discovered translator fixes, perform requested
    /// resets, complete conversions and arbitrate/take interrupts.
    fn boundary(&mut self) -> Result<(), String> {
        self.flush_pflash_code_dirty();
        if self.native.is_none() {
            self.install_fix_hooks()?;
            let hw = self.hw_mut();
            let now = hw.total_instructions;
            if hw.code_pages.iter().any(|c| now.saturating_sub(c.1) > RAM_CODE_FLUSH_INSNS) {
                // Return idle RAM code pages to plain data RAM: drop their translations, then the TLB
                // (refilled without execute permission, see `cpu_hooks::hook_tlb_fill`).
                let page = hw.page_mask as u64 + 1;
                let stale: Vec<u32> = hw.code_pages.iter().filter(|c| now.saturating_sub(c.1) > RAM_CODE_FLUSH_INSNS).map(|c| c.0).collect();
                hw.code_pages.retain(|c| now.saturating_sub(c.1) <= RAM_CODE_FLUSH_INSNS);
                for p in stale {
                    // SAFETY: outside emulation. Only drops translations; they are rebuilt on demand.
                    unsafe { uc_ctl_remove_cache(self.uc, p as u64, p as u64 + page) };
                }
                // SAFETY: outside emulation.
                unsafe { uc_ctl_flush_tlb(self.uc) };
            }
        }
        match self.variant {
            BoardVariant::Simos18Tc1791 => {
                self.hw_mut().spi_dma_update();
                if let Some(kind) = self.hw_mut().reset_request.take() {
                    self.reset(kind);
                }
                let hw = self.hw_mut();
                hw.adc_update();
                hw.sent_update();
            }
            BoardVariant::Dq200Tc1784 => {
                if let Some(kind) = self.hw_mut().reset_request.take() {
                    self.reset(kind);
                }
                let hw = self.hw_mut();
                hw.adc_update();
                hw.tc1784_update();
            }
            BoardVariant::Dq250Tc1766 => {
                if let Some(kind) = self.hw_mut().reset_request.take() {
                    self.reset(kind);
                }
                let hw = self.hw_mut();
                hw.adc_update();
                hw.dq250_update();
            }
        }
        self.hw_mut().engine_io_boundary();
        self.dispatch_pending_irq();
        Ok(())
    }

    /// Hook the translator-fix sites discovered by the block hook (Unicorn, outside emulation).
    fn install_fix_hooks(&mut self) -> Result<(), String> {
        let pending = std::mem::take(&mut self.hw_mut().fixes.pending);
        for a in pending {
            let hh = self.add_hook(UC_HOOK_CODE, isa_fix::hook_isa_fix as *mut c_void, a as u64, a as u64)?;
            self.hw_mut().fixes.sites.insert(a, hh);
            // SAFETY: outside emulation.
            uc_check(unsafe { uc_ctl_remove_cache(self.uc, a as u64, a as u64 + 4) }, "uc_ctl_remove_cache")?;
        }
        Ok(())
    }

    /// Fold an executed quantum into the hardware timers.
    fn account(&mut self, executed: u64) {
        let hw = self.hw_mut();
        let n = executed.saturating_sub(hw.stm_folded);
        let (prev, cur) = hw.stm.advance(n);
        hw.stm_folded = 0;
        hw.total_instructions += executed;
        hw.sim_time_s = hw.time_at(hw.total_instructions);
        hw.clock_rebase();
        hw.quantum_executed = 0;
        hw.stm_events(prev, cur);
        if hw.scu.wdt_advance(executed) && hw.reset_request.is_none() {
            hw.reset_request = Some(ResetKind::Watchdog);
        }
    }

    fn error_report(&self, err: uc_err) -> String {
        let pc = self.read_reg(TriCoreReg::PC);
        let hw = self.hw();
        format!(
            "Unicorn stopped at PC=0x{:08X} ({}), last block 0x{:08X}, A11=0x{:08X}, PCXI=0x{:08X}, FCX=0x{:08X}, bad access {:?}: {}",
            pc,
            Self::classify_region(pc),
            hw.last_tb_pc,
            self.read_reg(TriCoreReg::A11),
            self.read_reg(TriCoreReg::PCXI),
            self.read_reg(TriCoreReg::FCX),
            hw.last_bad_access.map(|(t, a)| format!("type {} @ 0x{:08X}", t, a)),
            Self::strerror(err)
        )
    }

    /// Execute exactly `n` guest instructions in quanta of at most [`QUANTUM_MAX_INSNS`] (Unicorn
    /// without exact timing: block granular, a block that would exceed the budget is deferred to the
    /// next call unless it is the first of the call).
    pub fn run_instructions(&mut self, n: u64) -> Result<RunStats, String> {
        self.run_inner(n, None, f64::INFINITY)
    }

    /// Execute guest code for `seconds` of simulated time (time follows the firmware-configured
    /// clock tree, so it is exact even while the PLL is being reprogrammed).
    pub fn run_for(&mut self, seconds: f64) -> Result<RunStats, String> {
        self.run_inner(u64::MAX, None, seconds)
    }

    pub(crate) fn run_inner(&mut self, n: u64, until: Option<u32>, seconds: f64) -> Result<RunStats, String> {
        let t0 = Instant::now();
        let start = self.hw().total_instructions;
        let t_start = self.hw().sim_time_s;
        let (resets0, irqs0) = (self.hw().reset_count, self.hw().irq_dispatch_count);
        let mut idle_rounds = 0u32;
        while self.hw().total_instructions - start < n && self.hw().sim_time_s - t_start < seconds {
            self.boundary()?;
            if let Some(u) = until {
                if self.read_reg(TriCoreReg::PC) == u {
                    break;
                }
            }
            let left = n - (self.hw().total_instructions - start);
            let hw = self.hw_mut();
            let t_left = seconds - (hw.sim_time_s - t_start);
            let by_time = if t_left.is_finite() {
                ((t_left * hw.clk_f as f64 / CYCLES_PER_INSN as f64).ceil() as u64).max(1)
            } else {
                u64::MAX
            };
            let variant_next = match hw.variant {
                BoardVariant::Simos18Tc1791 => hw.insns_to_next_spi_event(),
                BoardVariant::Dq200Tc1784 => hw.tc1784_insns_to_next_event(),
                BoardVariant::Dq250Tc1766 => hw.dq250_insns_to_next_event(),
            };
            let budget = QUANTUM_MAX_INSNS
                .min(left)
                .min(by_time)
                .min(hw.insns_to_next_stm_event())
                .min(hw.insns_to_next_adc_event())
                .min(variant_next)
                .min(hw.engine_io_insns_to_next_event())
                .max(1);
            hw.quantum_budget = budget;
            hw.quantum_executed = 0;
            hw.stm_folded = 0;
            hw.stop_requested = false;
            hw.last_bad_access = None;
            let (executed, err) = if self.native.is_some() {
                // `until`: single instructions, so that the boundary sees every PC
                let (n, e) = self.native_quantum(if until.is_some() { 1 } else { budget });
                (n, e.map_or(Ok(()), |e| Err(Ok(e))))
            } else {
                let mut pc = self.read_reg(TriCoreReg::PC);
                let err = loop {
                    // SAFETY: valid engine; no Rust borrow of the state is held across this call.
                    let err = unsafe { uc_emu_start(self.uc, pc as u64, until.unwrap_or(0) as u64, 0, 0) };
                    // Stopped only to hook newly found fix sites (before their block ran): hook them
                    // and resume the same quantum. Exact timing needs the quantum ends of the native
                    // core, which has no such stop (peripherals are brought up to date at quantum ends).
                    let hw = self.hw();
                    if err != UC_ERR_OK || !hw.exact_timing || hw.fixes.pending.is_empty() || hw.stop_requested || hw.reset_request.is_some() || hw.quantum_executed >= budget {
                        break err;
                    }
                    self.install_fix_hooks()?;
                    pc = self.read_reg(TriCoreReg::PC);
                };
                (self.hw().quantum_executed, if err == UC_ERR_OK { Ok(()) } else { Err(Err(err)) })
            };
            {
                let hw = self.hw_mut();
                hw.quanta += 1;
                if executed < budget {
                    hw.quanta_stopped_early += 1;
                }
            }
            self.account(executed);
            let hw = self.hw_mut();
            if let Some(log) = hw.event_log.as_mut() {
                log.push((hw.total_instructions, 3, executed as u32, budget));
            }
            match err {
                Ok(()) => {}
                Err(Ok(e)) => return Err(e),
                Err(Err(e)) => return Err(self.error_report(e)),
            }
            if executed == 0 && self.hw().fixes.pending.is_empty() && self.hw().reset_request.is_none() {
                idle_rounds += 1;
                if idle_rounds > 10_000 {
                    return Err(format!("CPU makes no progress at PC=0x{:08X}", self.read_reg(TriCoreReg::PC)));
                }
            } else {
                idle_rounds = 0;
            }
        }
        self.hw_mut().adc_update();
        let wall = t0.elapsed().as_secs_f64();
        let hw = self.hw();
        let instructions = hw.total_instructions - start;
        let sim = hw.sim_time_s - t_start;
        Ok(RunStats {
            pc: self.read_reg(TriCoreReg::PC),
            instructions,
            sim_time_s: sim,
            wall_time_s: wall,
            real_time_factor: if wall > 0.0 { sim / wall } else { 0.0 },
            mips: if wall > 0.0 { instructions as f64 / wall / 1e6 } else { 0.0 },
            resets: hw.reset_count - resets0,
            interrupts: hw.irq_dispatch_count - irqs0,
        })
    }

    /// Run from `from` until `until` is reached (test helper).
    #[cfg(test)]
    pub(crate) fn run_until(&mut self, from: u32, until: u32) -> Result<RunStats, String> {
        self.write_reg(TriCoreReg::PC, from);
        self.run_inner(1_000_000, Some(until), f64::INFINITY)
    }

    /// Power-on boot: (re)start from the reset vector and execute `total_instructions`.
    pub fn run_boot_sequence(&mut self, total_instructions: usize) -> Result<u32, String> {
        if self.hw().total_instructions != 0 {
            self.reset(ResetKind::PowerOn);
        }
        self.run_instructions(total_instructions as u64).map(|s| s.pc)
    }
}

impl Drop for Simos18Board {
    fn drop(&mut self) {
        // the native core maps the state's buffers too
        drop(self.native.take());
        // SAFETY: the engine is closed before the state (and the buffers it maps) is freed.
        unsafe {
            if !self.uc.is_null() {
                uc_close(self.uc);
            }
            drop(Box::from_raw(self.hw.as_ptr()));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_mem_mmio_is_pure_and_aliasing_safe_across_backends() {
        for backend in CpuBackend::matrix() {
            let mut b = Simos18Board::new_variant_on(
                &vec![0u8; 0x40_0000],
                &vec![0u8; 0x4_0000],
                BoardVariant::Simos18Tc1791,
                backend,
            )
            .unwrap();
            b.hw_mut().stm.advance(3_600);
            // Enable SSC0 (`CLC=0`, `FDR=0x43FF`, `CON=0xC01F`, `BR=1`) and complete a 16-bit frame
            // so `SSC0.RB` holds unread data (`0xFFFF` from idle MISO) before peeking!
            {
                let hw = b.hw_mut();
                hw.scu.endinit = false;
                hw.mmio_write(0xF031_0000, 4, 0); // SSC0_CLC = 0
                hw.scu.endinit = true;
                hw.mmio_write(0xF031_000C, 4, 0x43FF); // SSC0_FDR: f_SSC = f_FPI
                hw.mmio_write(0xF031_0014, 4, 1); // SSC0_BR = 1
                hw.mmio_write(0xF031_0010, 4, 0xC01F); // SSC0_CON: EN | MS | 16-bit
                hw.mmio_write(0xF031_0020, 4, 0x1234); // SSC0_TB -> starts 16-bit frame at t = 0
                hw.sim_time_s = 1e-4; // 100 us > frame_ns
                hw.spi_dma_update();
            }
            let reads_before = b.hw().mmio_read_count;

            // Hold an immutable `&Tc1791HardwareState` across `read_mem` and `read_u32` on MMIO:
            // previously `uc_mem_read` / `native_read_mem` created `&mut Tc1791HardwareState` and
            // triggered register side effects (STM.CAP latch, SSC.RB clear).
            let hw_ref = b.hw();
            let tim0_bytes = b.read_mem(0xF000_0210, 4).unwrap();
            let tim0_u32 = b.read_u32(0xF000_0210);
            let rb_u32 = b.read_u32(0xF031_0024);
            assert_eq!(u32::from_le_bytes(tim0_bytes.try_into().unwrap()), tim0_u32);
            assert!(tim0_u32 > 0, "expected tim0_u32 > 0");
            assert_eq!(rb_u32, 0xFFFF);
            assert_eq!(hw_ref.mmio_read_count, reads_before, "{backend:?}: peek must not count as guest MMIO read");
            assert_eq!(hw_ref.stm.cap, 0, "{backend:?}: peek of STM_TIM0 must not latch STM_CAP");
            // Prove `SSC0.RB` was NOT marked read by the peek: shifting a second word without reading
            // `RB` must raise receive-overflow `STAT.RE`!
            {
                let hw = b.hw_mut();
                assert_eq!(hw.mmio_read(0xF031_0028, 4) & (1 << 9), 0, "no RE before second frame");
                hw.mmio_write(0xF031_0020, 4, 0x5678);
                hw.sim_time_s = 2e-4;
                hw.spi_dma_update();
                assert_ne!(hw.mmio_read(0xF031_0028, 4) & (1 << 9), 0, "{backend:?}: peek must preserve rb_unread so overrun sets RE");
            }
        }
    }

    #[test]
    fn dq200_ram_range_at_4gb_boundary_does_not_wrap() {
        for backend in CpuBackend::matrix() {
            let b = Simos18Board::new_variant_on(
                &vec![0u8; 0x40_0000],
                &vec![0u8; 0x2000],
                BoardVariant::Dq200Tc1784,
                backend,
            )
            .unwrap();
            assert!(b.hw().is_ram(0xF880_0000), "{backend:?}: 0xF880_0000 + 0x0780_0000 must not wrap to 0");
            assert!(b.hw().is_ram(0xFFFF_FFFC), "{backend:?}");
            assert!(!b.hw().is_ram(0xF87F_0000), "{backend:?}: MMIO segment below 0xF880_0000 is not RAM");
        }
    }

    #[test]
    fn unaligned_cross_word_mmio_read_and_write() {
        let mut b = Simos18Board::new_variant_on(
            &vec![0u8; 0x40_0000],
            &vec![0u8; 0x4_0000],
            BoardVariant::Simos18Tc1791,
            CpuBackend::Native,
        )
        .unwrap();
        let hw = b.hw_mut();
        // Use unmodelled register words in `seg_f000` (outside SCU/GPIO/GPTA)
        hw.seg_f000.set(0xF000_0700, 0x1122_3344);
        hw.seg_f000.set(0xF000_0704, 0x5566_7788);
        assert_eq!(hw.mmio_read(0xF000_0702, 4), 0x7788_1122);
        assert_eq!(hw.mmio_read(0xF000_0703, 2), 0x8811);

        hw.mmio_write(0xF000_0702, 4, 0xCCDD_AABB);
        assert_eq!(hw.seg_f000.get(0xF000_0700), 0xAABB_3344);
        assert_eq!(hw.seg_f000.get(0xF000_0704), 0x5566_CCDD);
    }

    #[test]
    fn ram_code_invalidation_beyond_8_bytes_and_live_cal_uncached_alias() {
        // 12-byte RAM leaf function at PSPR 0xC000_0100: `nop ; nop ; mov d2, #N ; ji a11`
        // First 8 bytes (`0x0000_000D` × 2) stay identical; only bytes 8..12 change!
        let leaf = |imm4: u8| [0x82u8, (imm4 << 4) | 0x02, 0xDC, 0x0B]; // mov d2, #imm4 ; ji a11
        let make_fn = |imm4: u8| -> [u8; 12] {
            let mut out = [0u8; 12];
            out[0..4].copy_from_slice(&0x0000_000Du32.to_le_bytes()); // nop (32-bit)
            out[4..8].copy_from_slice(&0x0000_000Du32.to_le_bytes()); // nop (32-bit)
            out[8..12].copy_from_slice(&leaf(imm4));
            out
        };
        for backend in CpuBackend::matrix() {
            let mut flash = vec![0u8; 0x40_0000];
            // Caller at 0x8000_0000..0x8000_0008: `jli a6 ; nop` (`[0x2D, 0x06, 0x20, 0x00]`)
            flash[0..4].copy_from_slice(&[0x2D, 0x06, 0x20, 0x00]); // jli a6
            flash[4..8].copy_from_slice(&0x0000_000Du32.to_le_bytes()); // nop
            // CAL bank on Simos18Tc1791 starts at flash offset 0x20_0000 (`0x8080_0000` / `0xA080_0000`)
            flash[0x20_0000..0x20_0004].copy_from_slice(&leaf(3));

            let mut b = Simos18Board::new_variant_on(&flash, &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791, backend).unwrap();

            b.write_mem(0xC000_0100, &make_fn(5)).unwrap();
            b.write_reg(TriCoreReg::A6, 0xC000_0100);
            b.run_until(0x8000_0000, 0x8000_0004).unwrap();
            assert_eq!(b.read_reg(TriCoreReg::D2), 5, "{backend:?}: initial RAM function");

            // Modify ONLY bytes 8..12 via sys_ram_mut + mark_ram_write_code_dirty (simulating DMA/PCP write
            // AND testing Unicorn block_info full-block hash beyond byte 8)
            {
                let fn7 = make_fn(7);
                let hw = b.hw_mut();
                hw.sys_ram_mut(0xC000_0108, 4).unwrap().copy_from_slice(&fn7[8..12]);
                hw.mark_ram_write_code_dirty(0xC000_0108, 4);
            }
            b.write_reg(TriCoreReg::A6, 0xC000_0100);
            b.run_until(0x8000_0000, 0x8000_0004).unwrap();
            assert_eq!(b.read_reg(TriCoreReg::D2), 7, "{backend:?}: RAM code tail update must invalidate TB");

            // Test write_live_calibration invalidating uncached 0xA080_0000 alias too
            b.write_reg(TriCoreReg::A6, 0xA080_0000);
            b.run_until(0x8000_0000, 0x8000_0004).unwrap();
            assert_eq!(b.read_reg(TriCoreReg::D2), 3, "{backend:?}: initial uncached CAL");

            b.write_live_calibration(0, &leaf(6)).unwrap();
            b.write_reg(TriCoreReg::A6, 0xA080_0000);
            b.run_until(0x8000_0000, 0x8000_0004).unwrap();
            assert_eq!(b.read_reg(TriCoreReg::D2), 6, "{backend:?}: write_live_calibration must invalidate 0xA080_0000");
        }
    }
}
