//! Unicorn callbacks: peripheral MMIO dispatch (`uc_mmio_map`), flash command cycles
//! (`UC_HOOK_MEM_WRITE_PROT`), the block hook (instruction accounting, translator-fix discovery,
//! region milestones) and observation-only PC probes.

use crate::memory::{BoardVariant, MilestoneEvent, Tc1791HardwareState};
use crate::stm_icu::{is_src, STM_BASE};
use std::ffi::{c_int, c_uint, c_void};
use unicorn_tricore_sys::*;

/// Execution regions of the SIMOS18.1 flash layout (bootloader layout common to all SIMOS18.1
/// software versions). Used for generic "first entry" milestones.
pub fn region_of(pc: u32) -> (u8, &'static str) {
    match pc & 0xDFFF_FFFF {
        0x8000_0000..=0x8000_FFFF => (0, "SBOOT"),
        0x8001_0000..=0x8003_BFFF => (1, "CBOOT"),
        0x8003_C000..=0x801F_FFFF => (2, "ASW_PFLASH0"),
        0x8080_0000..=0x8087_FFFF => (3, "CAL_PFLASH1"),
        0x8088_0000..=0x809F_FFFF => (4, "ASW_PFLASH1"),
        0x8FFF_C000..=0x8FFF_FFFF => (5, "BROM"),
        _ => match pc >> 28 {
            0xC => (6, "PSPR_CODE"),
            0xD => (7, "DSPR_CODE"),
            _ => (8, "OTHER"),
        },
    }
}

impl Tc1791HardwareState {
    #[inline]
    fn seg_mut(&mut self, a: u32) -> Option<&mut crate::memory::MmioSeg> {
        [&mut self.seg_f000, &mut self.seg_f7e0, &mut self.seg_f800, &mut self.seg_f87f]
            .into_iter()
            .find(|s| s.contains(a))
    }

    pub(crate) fn is_mmio(&self, a: u32) -> bool {
        [&self.seg_f000, &self.seg_f7e0, &self.seg_f800, &self.seg_f87f].iter().any(|s| s.contains(a))
    }

    fn seg_get(&self, a: u32) -> u32 {
        for s in [&self.seg_f000, &self.seg_f7e0, &self.seg_f800, &self.seg_f87f] {
            if s.contains(a) {
                return s.get(a);
            }
        }
        0
    }

    /// Peripheral-specific read of an aligned word; `None` falls back to the register file.
    fn periph_read(&mut self, a: u32) -> Option<u32> {
        if let Some(v) = self.tc1784_read(a) {
            return Some(v);
        }
        if is_src(a) {
            return Some(self.src_get(a));
        }
        if self.variant == BoardVariant::Dq250Tc1766 {
            return self.dq250_read(a);
        }
        if let Some(v) = self.engine_io_read(a) {
            return Some(v);
        }
        match a {
            0xF000_0200..=0xF000_02FF => self.stm_read(a - STM_BASE),
            0xF000_0500..=0xF000_06FF | 0xF000_0C00..=0xF000_17FF | 0xF030_0000..=0xF030_05FF => {
                self.scu_read(a)
            }
            0xF000_4000..=0xF000_7FFF => self.can_read(a),
            0xF010_1000..=0xF010_1BFF => self.adc_read(a),
            0xF010_0400..=0xF010_04EF => self.fadc_read(a),
            crate::sent::SENT_BASE..=crate::sent::SENT_LAST => self.sent_read(a),
            0xF800_2000..=0xF800_4FFF => self.flash_reg_read(a),
            0xF031_0000..=0xF031_03FF => self.ssc_mmio_read(a),
            0xF000_3C00..=0xF000_3EDF => self.dma_mmio_read(a),
            0xF000_0800..=0xF000_09FF => self.msc_mmio_read(a),
            _ => None,
        }
    }

    fn periph_write(&mut self, a: u32, v: u32, m: u32) -> bool {
        if self.tc1784_write(a, v, m) {
            return true;
        }
        if is_src(a) {
            self.src_write(a, v, m);
            return true;
        }
        if self.variant == BoardVariant::Dq250Tc1766 {
            return self.dq250_write(a, v, m);
        }
        if self.engine_io_write(a, v, m) {
            return true;
        }
        match a {
            0xF000_0200..=0xF000_02FF => self.stm_write(a - STM_BASE, v, m),
            0xF000_0500..=0xF000_06FF | 0xF000_0C00..=0xF000_17FF | 0xF030_0000..=0xF030_05FF => {
                self.scu_write(a, v, m)
            }
            0xF000_4000..=0xF000_7FFF => self.can_write(a, v, m),
            0xF010_1000..=0xF010_1BFF => self.adc_write(a, v, m),
            0xF010_0400..=0xF010_04EF => self.fadc_write(a, v, m),
            crate::sent::SENT_BASE..=crate::sent::SENT_LAST => self.sent_write(a, v, m),
            0xF800_2000..=0xF800_4FFF => self.flash_reg_write(a, v, m),
            0xF031_0000..=0xF031_03FF => self.ssc_mmio_write(a, v, m),
            0xF000_3C00..=0xF000_3EDF => self.dma_mmio_write(a, v, m),
            0xF000_0800..=0xF000_09FF => self.msc_mmio_write(a, v, m),
            _ => false,
        }
    }

    pub(crate) fn periph_peek(&self, a: u32) -> Option<u32> {
        if let Some(v) = self.tc1784_peek(a) {
            return Some(v);
        }
        if is_src(a) {
            return Some(self.src_get(a));
        }
        if self.variant == BoardVariant::Dq250Tc1766 {
            return self.dq250_peek(a);
        }
        if let Some(v) = self.engine_io_peek(a) {
            return Some(v);
        }
        match a {
            0xF000_0200..=0xF000_02FF => self.stm_peek(a - STM_BASE),
            0xF000_0500..=0xF000_06FF | 0xF000_0C00..=0xF000_17FF | 0xF030_0000..=0xF030_05FF => {
                self.scu_read(a)
            }
            0xF000_4000..=0xF000_7FFF => self.can_read(a),
            0xF010_1000..=0xF010_1BFF => self.adc_peek(a),
            0xF010_0400..=0xF010_04EF => self.fadc_peek(a),
            crate::sent::SENT_BASE..=crate::sent::SENT_LAST => self.sent_peek(a),
            0xF800_2000..=0xF800_4FFF => self.flash_reg_read(a),
            0xF031_0000..=0xF031_03FF => self.ssc_mmio_peek(a),
            0xF000_3C00..=0xF000_3EDF => self.spi.dma.read(a - crate::dma::DMA_BASE),
            0xF000_0800..=0xF000_09FF => self.msc_mmio_read(a),
            _ => None,
        }
    }

    /// Side-effect-free host debugger peek of one MMIO byte through `&self`.
    pub(crate) fn mmio_peek(&self, addr: u32) -> u8 {
        let a = addr & !3;
        let v = self.periph_peek(a).unwrap_or_else(|| self.seg_get(a));
        (v >> ((addr & 3) * 8)) as u8
    }

    /// Guest MMIO read of `size` bytes (sub-word and cross-word unaligned reads extract the
    /// addressed byte lanes across `a` and `a + 4`).
    pub(crate) fn mmio_read(&mut self, addr: u32, size: u32) -> u64 {
        if size == 0 {
            return 0;
        }
        self.mmio_read_count += 1;
        let a = addr & !3;
        let boff = addr & 3;
        let v0 = self.periph_read(a).unwrap_or_else(|| self.seg_get(a));
        let sh = boff * 8;
        let m = if size >= 4 { u32::MAX } else { (1u32 << (size * 8)) - 1 };
        let r = if size <= 4 && boff + size > 4 {
            let a1 = a.wrapping_add(4);
            let v1 = self.periph_read(a1).unwrap_or_else(|| self.seg_get(a1));
            (((v0 >> sh) | (v1 << ((4 - boff) * 8))) & m) as u64
        } else {
            ((v0 >> sh) & m) as u64
        };
        if let Some(log) = self.event_log.as_mut() {
            log.push((self.total_instructions + self.quantum_executed, 0, addr, r));
        }
        r
    }

    /// A CPU read of a peripheral (the MMIO read callback; also used by `isa_fix`): recorded in
    /// `mmio_log` when that is on.
    pub(crate) fn mmio_read_logged(&mut self, addr: u32, size: u32) -> u64 {
        let v = self.mmio_read(addr, size);
        let endinit = self.scu.endinit;
        if let Some(log) = self.mmio_log.get_mut() {
            log.push((false, addr, size as u8, v, endinit));
        }
        v
    }

    fn mmio_write_word_lanes(&mut self, a: u32, v: u32, m: u32) {
        if !self.periph_write(a, v, m) && !self.scu.endinit_blocks(a) {
            if let Some(s) = self.seg_mut(a) {
                let old = s.get(a);
                s.set(a, (old & !m) | (v & m));
            }
        }
        if matches!(a, 0xF000_0518 | 0xF000_051C | 0xF000_0040) && self.scu.fcpu_hz() != self.clk_f {
            // new clock tree: end the quantum after this instruction, time is rebased there
            self.stop_requested = true;
        }
    }

    /// Guest MMIO write: sub-word and cross-word unaligned writes are presented to the register
    /// model as `(value, byte-lane mask)` so that set/clear/trigger semantics apply to the exact
    /// written byte lanes.
    pub(crate) fn mmio_write(&mut self, addr: u32, size: u32, value: u64) {
        if size == 0 {
            return;
        }
        self.mmio_write_count += 1;
        if let Some(log) = self.event_log.as_mut() {
            log.push((self.total_instructions + self.quantum_executed, 1, addr, value));
        }
        let a = addr & !3;
        let boff = addr & 3;
        let sh = boff * 8;
        let lane = if size >= 4 { u32::MAX } else { (1u32 << (size * 8)) - 1 };
        let val32 = (value as u32) & lane;
        let m0 = lane << sh;
        let v0 = val32 << sh;
        self.mmio_write_word_lanes(a, v0, m0);
        if size <= 4 && boff + size > 4 {
            let hi_bytes = boff + size - 4;
            let m1 = (1u32 << (hi_bytes * 8)) - 1;
            let v1 = val32 >> ((4 - boff) * 8);
            self.mmio_write_word_lanes(a.wrapping_add(4), v1, m1);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// C-ABI callbacks. All of them only touch the state through the `user_data` pointer; no Rust
// borrow of the state is alive while Unicorn executes, and nothing here can panic.
// ---------------------------------------------------------------------------------------------

#[inline]
unsafe fn state<'a>(ud: *mut c_void) -> &'a mut Tc1791HardwareState {
    &mut *(ud as *mut Tc1791HardwareState)
}

macro_rules! mmio_segment {
    ($read:ident, $write:ident, $base:expr) => {
        pub(crate) extern "C" fn $read(_uc: *mut uc_engine, off: u64, size: c_uint, ud: *mut c_void) -> u64 {
            // SAFETY: see module comment.
            let hw = unsafe { state(ud) };
            let addr = ($base as u32).wrapping_add(off as u32);
            if size == 8 {
                hw.mmio_read_logged(addr, 4) | (hw.mmio_read_logged(addr.wrapping_add(4), 4) << 32)
            } else {
                hw.mmio_read_logged(addr, size)
            }
        }
        pub(crate) extern "C" fn $write(uc: *mut uc_engine, off: u64, size: c_uint, value: u64, ud: *mut c_void) {
            // SAFETY: see module comment.
            let hw = unsafe { state(ud) };
            let addr = ($base as u32).wrapping_add(off as u32);
            if size == 8 {
                hw.mmio_write(addr, 4, value & 0xFFFF_FFFF);
                let endinit0 = hw.scu.endinit;
                if let Some(log) = hw.mmio_log.get_mut() {
                    log.push((true, addr, 4, value & 0xFFFF_FFFF, endinit0));
                }
                hw.mmio_write(addr.wrapping_add(4), 4, value >> 32);
                let endinit1 = hw.scu.endinit;
                if let Some(log) = hw.mmio_log.get_mut() {
                    log.push((true, addr.wrapping_add(4), 4, value >> 32, endinit1));
                }
            } else {
                hw.mmio_write(addr, size, value);
                let endinit = hw.scu.endinit;
                if let Some(log) = hw.mmio_log.get_mut() {
                    log.push((true, addr, size as u8, value, endinit));
                }
            }
            if hw.reset_request.is_some() {
                // SAFETY: valid engine; stopping from a callback is supported.
                unsafe { uc_emu_stop(uc) };
            }
        }
    };
}

mmio_segment!(mmio_read_f000, mmio_write_f000, 0xF000_0000u32);
mmio_segment!(mmio_read_f7e0, mmio_write_f7e0, 0xF7E0_0000u32);
mmio_segment!(mmio_read_f800, mmio_write_f800, 0xF800_0000u32);
mmio_segment!(mmio_read_f87f, mmio_write_f87f, 0xF87F_0000u32);

/// Stores into the (read-only mapped) flash arrays: command cycles for the flash controller.
/// Returning `true` lets Unicorn continue; the store itself is discarded (read-only region).
pub(crate) extern "C" fn hook_flash_write(
    _uc: *mut uc_engine,
    _t: uc_mem_type,
    address: u64,
    size: c_int,
    value: i64,
    ud: *mut c_void,
) -> bool {
    // SAFETY: see module comment.
    let hw = unsafe { state(ud) };
    hw.flash_command(address as u32, size.max(0) as u32, value as u64);
    let endinit = hw.scu.endinit;
    if let Some(log) = hw.mmio_log.get_mut() {
        log.push((true, address as u32, size.max(0) as u8, value as u64, endinit));
    }
    true
}

/// Unmapped/protected accesses: record for the error report and let Unicorn stop.
pub(crate) extern "C" fn hook_bad_access(
    _uc: *mut uc_engine,
    t: uc_mem_type,
    address: u64,
    _size: c_int,
    _value: i64,
    ud: *mut c_void,
) -> bool {
    // SAFETY: see module comment.
    let hw = unsafe { state(ud) };
    hw.last_bad_access = Some((t, address as u32));
    false
}

/// Executed before every translation block: accounts instructions, enforces the quantum budget,
/// discovers translator-fix sites and records region milestones.
pub(crate) extern "C" fn hook_block(uc: *mut uc_engine, address: u64, size: u32, ud: *mut c_void) {
    // SAFETY: see module comment.
    let hw = unsafe { state(ud) };
    let pc = address as u32;
    let (n, new_fix) = hw.block_info(pc, size);
    if !matches!(pc >> 28, 0x8 | 0xA) {
        hw.note_code_exec(pc);
    }
    let started = hw.quantum_executed > 0;
    let over = hw.quantum_executed + n > hw.quantum_budget;
    if over && hw.exact_timing && hw.quantum_executed < hw.quantum_budget && !new_fix && !(started && hw.stop_requested) {
        // The quantum ends inside this block: Unicorn re-runs it limited to the rest of the budget
        // (nothing of it has executed; the limited block is counted by its own prologue).
        hw.truncate = hw.quantum_budget - hw.quantum_executed;
        return;
    }
    if new_fix || (started && (hw.stop_requested || over)) {
        // The block has not executed yet: stopping here keeps the accounting exact.
        // SAFETY: valid engine.
        unsafe { uc_emu_stop(uc) };
        return;
    }
    let (rid, name) = region_of(pc);
    let bit = 1u16 << rid;
    if hw.regions_seen & bit == 0 {
        // time at the block's first instruction (as the native core stamps it)
        hw.regions_seen |= bit;
        let ticks = hw.stm_now();
        hw.milestones.push(MilestoneEvent { reset_cycle: hw.reset_count, name, pc, stm_ticks: ticks });
    }
    hw.quantum_executed += n;
    hw.last_tb_pc = pc;
    hw.fixes.tb_end = pc.wrapping_add(size);
}

/// Translation-time block classification for the inline block accounting of a SIMEMU-patched
/// Unicorn (`uc_ctl_simemu_block_counter`). The inline prologue does exactly what [`hook_block`]
/// does for a block of a known instruction count without fix sites outside RAM in an already
/// entered region: `quantum_executed += n; last_tb_pc = pc` unless a stop is requested or
/// the budget would be exceeded. Every other block keeps calling [`hook_block`].
pub(crate) extern "C" fn classify_block(ud: *mut c_void, pc: u64, size: u32, n: *mut u64, bit: *mut u32) -> u32 {
    // SAFETY: see module comment.
    let hw = unsafe { state(ud) };
    hw.tb_translations += 1;
    let pc = pc as u32;
    if !matches!(pc >> 28, 0x8 | 0xA) {
        return 1; // RAM code: the hook stamps the code page
    }
    let Some(count) = hw.block_count_if_known(pc, size) else { return 1 };
    if hw.variant == BoardVariant::Simos18Tc1791 && crate::S50_SIG_HOOKS.iter().any(|h| h.addr.wrapping_sub(pc) < size) {
        return 1; // signature hook inside: `hook_sig` needs the block end for its time stamp
    }
    // SAFETY: Unicorn's out-parameters for this call.
    unsafe {
        *n = count;
        *bit = 1 << region_of(pc).0;
    }
    0
}

/// TLB fill (`UC_TLB_VIRTUAL`): identity mapping with full permissions, except that RAM pages
/// get no execute permission until code is fetched from them.
///
/// Why: Unicorn routes every store through `notdirty_write` (full slow path) while the TLB
/// entry is executable, because that is how it detects self-modifying code. TriCore's own TLB
/// fill always grants execute, so with the default TLB every RAM store is slow. A data-only RAM
/// page is cleaned after its first store and then written by the inlined fast path. A page
/// becomes executable (and SMC-protected) on its first instruction fetch, before any
/// translation of it exists, so self-modifying code is still always detected. Guest-visible
/// permissions are unchanged: they are enforced by the memory mappings, not by this TLB.
pub(crate) extern "C" fn hook_tlb_fill(
    _uc: *mut uc_engine,
    vaddr: u64,
    t: uc_mem_type,
    e: *mut uc_tlb_entry,
    ud: *mut c_void,
) -> bool {
    // SAFETY: see module comment.
    let hw = unsafe { state(ud) };
    hw.tlb_fills += 1;
    let page = vaddr as u32 & !hw.page_mask;
    let mut perms = UC_PROT_ALL;
    if hw.is_ram(page) && !hw.code_pages.iter().any(|c| c.0 == page) {
        if t == UC_MEM_FETCH {
            let now = hw.total_instructions + hw.quantum_executed;
            hw.code_pages.push((page, now));
        } else {
            perms = UC_PROT_READ | UC_PROT_WRITE;
        }
    }
    // SAFETY: `e` is Unicorn's out-parameter for this call.
    unsafe { *e = uc_tlb_entry { paddr: vaddr, perms } };
    true
}

impl Tc1791HardwareState {
    /// Record that a block of RAM-resident code is entered.
    #[inline]
    fn note_code_exec(&mut self, pc: u32) {
        let page = pc & !self.page_mask;
        let now = self.total_instructions + self.quantum_executed;
        if let Some(c) = self.code_pages.iter_mut().find(|c| c.0 == page) {
            c.1 = now;
        }
    }
}

/// Per-block decode cache entry: block start, size in bytes, instruction count, first 8 code bytes.
#[derive(Clone, Copy)]
pub(crate) struct TbCacheEntry {
    pc: u32,
    size: u16,
    n: u16,
    sig: u64,
}

impl TbCacheEntry {
    pub(crate) const EMPTY: Self = Self { pc: u32::MAX, size: 0, n: 0, sig: 0 };
}

pub(crate) const TB_CACHE_SIZE: usize = 1 << 17;

impl Tc1791HardwareState {
    /// Instruction count of a block and whether it contains not-yet-hooked fix sites. Decoding
    /// is cached per block; the cache entry is validated against the block's first 8 bytes so
    /// rewritten RAM code is decoded again.
    #[inline]
    fn block_info(&mut self, pc: u32, size: u32) -> (u64, bool) {
        // Fibonacci hash: the memory aliases (PFlash0/PFlash1, cached/non-cached) differ only in
        // high address bits and would collide in a plain low-bits index.
        let idx = (pc.wrapping_mul(0x9E37_79B1) >> (32 - TB_CACHE_SIZE.trailing_zeros())) as usize;
        let e = self.tb_cache[idx];
        let hit = e.pc == pc && e.size as u32 == size;
        if hit && matches!(pc >> 24, 0x80 | 0xA0) {
            return (e.n as u64, false); // flash content never changes under a translation
        }
        let len = size as usize;
        let sig = match self.code_bytes(pc) {
            Some(b) if len > 0 && b.len() >= len => {
                // Hash the entire RAM block (not just the first 8 bytes) so self-modifying code
                // rewriting tail instructions invalidates the cached count and re-scans fix sites.
                let mut h = 0xCBF2_9CE4_8422_2325u64;
                let (chunks, rem) = b[..len].as_chunks::<8>();
                for &w in chunks {
                    h = (h ^ u64::from_le_bytes(w)).wrapping_mul(0x1000_0000_01B3);
                }
                for &byte in rem {
                    h = (h ^ byte as u64).wrapping_mul(0x1000_0000_01B3);
                }
                h
            }
            _ => return self.scan_block(pc, size),
        };
        if hit && e.sig == sig {
            return (e.n as u64, false);
        }
        let (n, found) = self.scan_block(pc, size);
        if !found && size <= u16::MAX as u32 {
            self.tb_cache[idx] = TbCacheEntry { pc, size: size as u16, n: n as u16, sig };
        }
        (n, found)
    }
}

/// Write-watch trace entry: `(PC, A11, address, value, instruction count)`.
pub type WriteWatchEntry = (u32, u32, u32, i64, u64);

/// Writes seen by `hook_write_watch`: `(PC, A11, address, value, instruction count)`. Diagnostic.
pub static WRITE_WATCH: std::sync::Mutex<Vec<WriteWatchEntry>> = std::sync::Mutex::new(Vec::new());

/// Observation-only data-write watchpoint (Unicorn, `Simos18Board::add_write_watch`).
pub(crate) extern "C" fn hook_write_watch(uc: *mut uc_engine, _t: uc_mem_type, address: u64, _size: c_int, value: i64, ud: *mut c_void) {
    // SAFETY: see module comment.
    let hw = unsafe { state(ud) };
    let (mut pc, mut ra) = (0u32, 0u32);
    // SAFETY: valid engine inside a callback.
    unsafe {
        uc_reg_read(uc, TriCoreReg::PC as c_int, (&mut pc as *mut u32).cast());
        uc_reg_read(uc, TriCoreReg::A11 as c_int, (&mut ra as *mut u32).cast());
    }
    let mut w = WRITE_WATCH.lock().unwrap_or_else(|e| e.into_inner());
    if w.len() < 1_000_000 {
        w.push((pc, ra, address as u32, value, hw.total_instructions + hw.quantum_executed));
    }
}

/// Observation-only PC probe: counts hits and snapshots D0..D15/A0..A15.
pub(crate) extern "C" fn hook_probe(uc: *mut uc_engine, address: u64, _size: u32, ud: *mut c_void) {
    // SAFETY: see module comment.
    let hw = unsafe { state(ud) };
    let pc = address as u32;
    let mut regs = [0u32; 32];
    for (i, r) in regs.iter_mut().enumerate() {
        let id = if i < 16 { TriCoreReg::d(i as u32) } else { TriCoreReg::a(i as u32 - 16) };
        // SAFETY: valid engine; failure leaves 0.
        unsafe { uc_reg_read(uc, id as c_int, (r as *mut u32).cast()) };
    }
    if let Some(p) = hw.probes.iter_mut().find(|p| p.addr == pc) {
        p.hits += 1;
        p.last_regs = regs;
    }
}

/// A probe registered with [`crate::Simos18Board::add_probe`].
#[derive(Debug, Clone)]
pub struct Probe {
    pub addr: u32,
    pub hits: u64,
    /// D0..D15 followed by A0..A15 at the most recent hit.
    pub last_regs: [u32; 32],
}
