//! System Timer (`STM`), Service Request Control registers (`SRC`) and the CPU interrupt
//! control unit (arbitration + TriCore interrupt entry).
//!
//! Clock tree — derived at run time from the firmware's own register writes (see
//! [`crate::scu::ScuWdt::fcpu_hz`]):
//! * fCPU = fOSC * (NDIV+1) / ((PDIV+1) * (K2DIV+1))  (SCU_PLLCON0/1, PLL normal mode)
//! * fCPU = fOSC / (K1DIV+1)                           (VCO bypass / prescaler mode)
//! * fFPI = fCPU / (CCUCON0.FPIDIV + 1)
//! * fSTM = fFPI / STM_CLC.RMC
//!
//! SC800S50 writes NDIV = 71, PDIV = 1, K2DIV = 3 (ramped 35 -> 11 -> 4 -> 3), CCUCON0 = 0x10001,
//! STM_CLC = 0x314: with the 20 MHz crystal fCPU = 180 MHz, fFPI = 90 MHz, fSTM = 30 MHz. The
//! RTA-OS counter reloads CMP0 by 30000 ticks = 1 ms, and the GPTA conversion constants
//! (x90/80 MHz) independently confirm fFPI = 90 MHz. The STM-ticks-per-instruction ratio,
//! `(FPIDIV+1) * RMC` CPU cycles per tick, does not depend on the crystal at all.

use crate::memory::Tc1791HardwareState;
use crate::Simos18Board;
use unicorn_tricore_sys::TriCoreReg;

/// Board crystal. 20 MHz is the only standard crystal for which the firmware's PLL setting gives
/// a VCO inside the AUDO range (720 MHz) and a 1 ms OS tick; used only to convert cycles to
/// seconds (STM/WDT ratios are crystal independent). Photo evidence: the crystal next to the
/// TC1791 on a SIMOS18.1 board (`06K907425B H13`, `Y6000`) is marked `20.000`
/// (`vault/hardware/simos18_06K907425B_H13/`, `docs/SIMOS18_BOARD_HW.md`).
pub const FOSC_HZ: u64 = 20_000_000;
/// Calibration constant: CPU cycles per executed instruction (IPC = 1 assumption; Unicorn does not
/// model the TriCore pipeline).
pub const CYCLES_PER_INSN: u64 = 1;

pub const STM_BASE: u32 = 0xF000_0200;
pub const STM_SRC1: u32 = 0xF000_02F8;
pub const STM_SRC0: u32 = 0xF000_02FC;

// SRC register fields (TriCore AUDO SRC layout).
pub const SRC_SRPN: u32 = 0x0000_00FF;
pub const SRC_TOS: u32 = 0x0000_0C00;
pub const SRC_SRE: u32 = 0x0000_1000;
pub const SRC_SRR: u32 = 0x0000_2000;
pub const SRC_CLRR: u32 = 0x0000_4000;
pub const SRC_SETR: u32 = 0x0000_8000;

/// Service request control registers that exist on this board, sorted.
///
/// Firmware evidence (SIMOS18.1 SC800S50): every address below is either written by the
/// firmware's SRC-configuration routines (0x800DDE48 CPU table, 0x800DDF22 PCP table,
/// 0x8008D534 enable helper, SBOOT 0x8000256E) with the SRC bit layout (CLRR|SRE|TOS|SRPN), or is
/// part of a contiguous SRC block of a module whose other SRCs are written that way
/// (GPTA0/1 SRC00..SRC37, DMA SRC0..7, MultiCAN SRC0..15, CPU SRC0..3 + SBSRC).
pub fn src_table() -> &'static [u32] {
    use std::sync::OnceLock;
    static T: OnceLock<Vec<u32>> = OnceLock::new();
    T.get_or_init(|| {
        let mut v: Vec<u32> = vec![STM_SRC1, STM_SRC0, 0xF000_06F4, 0xF000_09FC];
        v.extend((0..38).map(|i| 0xF000_1F68 + 4 * i)); // GPTA0 SRC37..SRC00
        v.extend((0..38).map(|i| 0xF000_2768 + 4 * i)); // GPTA1 SRC37..SRC00
        v.extend((0..11).map(|i| 0xF000_2FD4 + 4 * i)); // LTCA2 SRCs
        v.extend((0..8).map(|i| 0xF000_3EE0 + 4 * i)); // DMA SRC7..SRC0
        v.extend((0..16).map(|i| 0xF000_40C0 + 4 * i)); // MultiCAN SRC15..SRC0
        v.extend([0xF001_03E4, 0xF001_03EC, 0xF032_19F8, 0xF032_19FC]);
        v.extend([0xF7E0_FFBC, 0xF7E0_FFF0, 0xF7E0_FFF4, 0xF7E0_FFF8, 0xF7E0_FFFC]); // CPU SBSRC, SRC3..0
        v.sort_unstable();
        v.dedup();
        v
    })
}

#[inline]
pub fn is_src(addr: u32) -> bool {
    addr & 3 == 0
        && matches!(
            addr,
            STM_SRC1
                | STM_SRC0
                | 0xF000_06F4
                | 0xF000_09FC
                | 0xF000_1F68..=0xF000_1FFC
                | 0xF000_2768..=0xF000_27FC
                | 0xF000_2FD4..=0xF000_2FFC
                | 0xF000_3EE0..=0xF000_3EFC
                | 0xF000_40C0..=0xF000_40FC
                | 0xF001_03E4
                | 0xF001_03EC
                | 0xF032_19F8
                | 0xF032_19FC
                | 0xF7E0_FFBC
                | 0xF7E0_FFF0..=0xF7E0_FFFC
        )
}

/// MultiCAN SRC `n` (INT_O line n) address.
#[inline]
pub fn can_src(n: u32) -> u32 {
    0xF000_40FC - 4 * (n & 15)
}

/// 64-bit STM with compare units.
pub struct StmState {
    /// STM count at the start of the current quantum.
    pub base: u64,
    /// CPU cycles accumulated towards the next STM tick.
    pub frac: u64,
    pub clc: u32,
    /// SCU_CCUCON0.FPIDIV (FPI = CPU / (FPIDIV+1)), mirrored here because it sets the STM rate.
    pub fpidiv: u32,
    /// Captured TIM6 value (bits 63:32), latched on reads of TIM0..TIM5.
    pub cap: u32,
    pub cmp: [u32; 2],
    pub cmcon: u32,
    /// STM_ICR: CMP0EN(0) CMP0IR(1) CMP0OS(2) CMP1EN(4) CMP1IR(5) CMP1OS(6).
    pub icr: u32,
}

impl Default for StmState {
    fn default() -> Self {
        Self { base: 0, frac: 0, clc: 0x0000_0100, fpidiv: 0, cap: 0, cmp: [0; 2], cmcon: 0, icr: 0 }
    }
}

impl StmState {
    /// CPU cycles per STM tick (`(FPIDIV+1) * RMC`); 0 when the module clock is disabled.
    pub fn cycles_per_tick(&self) -> u64 {
        let rmc = ((self.clc >> 8) & 0xFF) as u64;
        if rmc == 0 || self.clc & 1 != 0 {
            0
        } else {
            (self.fpidiv as u64 + 1) * rmc
        }
    }

    /// STM value after `insns` instructions of the current quantum.
    #[inline]
    pub fn now(&self, insns: u64) -> u64 {
        match self.cycles_per_tick() {
            0 => self.base,
            c => self.base + (self.frac + insns * CYCLES_PER_INSN) / c,
        }
    }

    /// Advance by `insns` executed instructions, returning `(prev, cur)` STM values.
    pub fn advance(&mut self, insns: u64) -> (u64, u64) {
        let prev = self.base;
        let total = self.frac + insns * CYCLES_PER_INSN;
        if let Some(ticks) = total.checked_div(self.cycles_per_tick()) {
            self.base += ticks;
            self.frac = total % self.cycles_per_tick();
        }
        (prev, self.base)
    }

    /// Instructions until the STM reaches `target` (>= 1).
    pub fn insns_until(&self, target: u64) -> u64 {
        let c = self.cycles_per_tick();
        if c == 0 {
            return u64::MAX;
        }
        if target <= self.base {
            return 1;
        }
        let cycles = ((target - self.base) as u128) * c as u128 - self.frac as u128;
        let n = cycles.div_ceil(CYCLES_PER_INSN as u128);
        n.clamp(1, u64::MAX as u128) as u64
    }

    fn field(&self, i: usize) -> (u32, u32) {
        let shift = if i == 0 { 0 } else { 16 };
        let msize = (self.cmcon >> shift) & 0x1F;
        let mstart = (self.cmcon >> (shift + 8)) & 0x1F;
        (msize + 1, mstart)
    }

    /// Distance in field units from field value `p` to the next equality with CMPi (never 0).
    fn field_distance(&self, i: usize, p: u64) -> (u64, u32) {
        let (width, mstart) = self.field(i);
        let mask = if width >= 32 { 0xFFFF_FFFFu64 } else { (1u64 << width) - 1 };
        let target = self.cmp[i] as u64 & mask;
        let mut d = target.wrapping_sub(p & mask) & mask;
        if d == 0 {
            d = mask + 1;
        }
        (d, mstart)
    }

    /// Edge-triggered compare: CMPi matched while the counter moved from `prev` to `cur`.
    pub fn matched(&self, i: usize, prev: u64, cur: u64) -> bool {
        if cur <= prev {
            return false;
        }
        let (_, mstart) = self.field(i);
        let p = prev >> mstart;
        let c = cur >> mstart;
        let (d, _) = self.field_distance(i, p);
        c - p >= d
    }

    /// Absolute STM value of the next CMPi match after `cur`.
    pub fn next_match(&self, i: usize, cur: u64) -> u64 {
        let (_, mstart) = self.field(i);
        let p = cur >> mstart;
        let (d, _) = self.field_distance(i, p);
        (p + d) << mstart
    }
}

impl Tc1791HardwareState {
    /// Simulated time (s) after `n` instructions since power-on. Within a clock segment it is an
    /// exact function of `n`, so it does not depend on how execution was split into quanta (the
    /// CPU backends split differently).
    #[inline]
    pub(crate) fn time_at(&self, n: u64) -> f64 {
        self.clk_t0 + (n.saturating_sub(self.clk_n0) * CYCLES_PER_INSN) as f64 / self.clk_f.max(1) as f64
    }

    /// Start a new clock segment at the current instruction count if fCPU changed. Called at
    /// quantum boundaries only: a write that changes fCPU ends the quantum (`mmio_write`).
    pub(crate) fn clock_rebase(&mut self) {
        let f = self.scu.fcpu_hz();
        if f != self.clk_f {
            self.clk_t0 = self.time_at(self.total_instructions);
            self.clk_n0 = self.total_instructions;
            self.clk_f = f;
        }
    }

    /// Side-effect-free STM register read (`STM_CAP` is not latched).
    pub(crate) fn stm_peek(&self, off: u32) -> Option<u32> {
        let now = self.stm_now();
        Some(match off {
            0x00 => self.stm.clc,
            0x10..=0x24 => (now >> (4 * ((off - 0x10) / 4))) as u32,
            0x28 => (now >> 32) as u32,
            0x2C => self.stm.cap,
            0x30 => self.stm.cmp[0],
            0x34 => self.stm.cmp[1],
            0x38 => self.stm.cmcon,
            0x3C => self.stm.icr,
            0x40 => 0,
            _ => return None,
        })
    }

    /// STM register read (offset relative to STM_BASE). Reads never advance time.
    pub(crate) fn stm_read(&mut self, off: u32) -> Option<u32> {
        let now = self.stm_now();
        Some(match off {
            0x00 => self.stm.clc,
            0x10..=0x24 => {
                self.stm.cap = (now >> 32) as u32;
                (now >> (4 * ((off - 0x10) / 4))) as u32
            }
            0x28 => (now >> 32) as u32,
            0x2C => self.stm.cap,
            0x30 => self.stm.cmp[0],
            0x34 => self.stm.cmp[1],
            0x38 => self.stm.cmcon,
            0x3C => self.stm.icr,
            0x40 => 0,
            _ => return None,
        })
    }

    pub(crate) fn stm_write(&mut self, off: u32, val: u32, mask: u32) -> bool {
        let merge = |old: u32| (old & !mask) | (val & mask);
        match off {
            0x00 => {
                if self.scu.endinit_blocks(STM_BASE) {
                    return true;
                }
                // Changing the divider: settle elapsed time with the old rate first.
                self.settle_stm();
                self.stm.clc = merge(self.stm.clc);
            }
            0x30 => self.stm.cmp[0] = merge(self.stm.cmp[0]),
            0x34 => self.stm.cmp[1] = merge(self.stm.cmp[1]),
            0x38 => self.stm.cmcon = merge(self.stm.cmcon) & 0x1F1F_1F1F,
            // ICR: CMPxIR bits are read-only (hardware status), only EN/OS are writable.
            0x3C => {
                let w = merge(self.stm.icr) & 0x55;
                self.stm.icr = (self.stm.icr & 0x22) | w;
            }
            0x40 => {
                // TC1796 UM §15.2.3 (p.15-6, Fig. 15-3) & §15.3.4 (p.15-20): CMPxIRS sets
                // ICR.CMPxIR (overriding CMPxIRR if both are 1) without generating an interrupt
                // pulse at STMIR0/STMIR1.
                let v = val & mask;
                if v & 1 != 0 {
                    self.stm.icr &= !0x02;
                }
                if v & 2 != 0 {
                    self.stm.icr |= 0x02;
                }
                if v & 4 != 0 {
                    self.stm.icr &= !0x20;
                }
                if v & 8 != 0 {
                    self.stm.icr |= 0x20;
                }
            }
            0x10..=0x2C => {} // timer/capture registers are read-only
            _ => return false,
        }
        // Compare configuration changed: end the quantum so the next event is re-planned.
        self.stop_requested = true;
        true
    }

    /// Fold the instructions executed so far in this quantum into the STM base (used when the
    /// STM rate changes mid-quantum).
    pub(crate) fn settle_stm(&mut self) {
        let n = self.quantum_executed.saturating_sub(self.stm_folded);
        let (prev, cur) = self.stm.advance(n);
        self.stm_folded = self.quantum_executed;
        self.stm_events(prev, cur);
    }

    /// Current STM value including the instructions already executed in this quantum.
    #[inline]
    pub fn stm_now(&self) -> u64 {
        self.stm.now(self.quantum_executed.saturating_sub(self.stm_folded))
    }

    fn stm_set_ir(&mut self, i: usize) {
        let (ir, en, os) = if i == 0 { (0x02, 0x01, 0x04) } else { (0x20, 0x10, 0x40) };
        self.stm.icr |= ir;
        if self.stm.icr & en != 0 {
            let src = if self.stm.icr & os != 0 { STM_SRC1 } else { STM_SRC0 };
            self.src_set_srr(src);
        }
    }

    /// Process STM compare events for the time window `(prev, cur]`.
    ///
    /// Per TC1796 UM V2.0 §15.2.3 Fig. 15-3 (PDF p.923-924) and §15.3.4 `STM_ICR` (PDF p.935-936),
    /// a compare match always sets `STM_ICR.CMPxIR` (polled directly with `CMP0EN = 0` by e.g.
    /// DQ250 CBOOT), while `CMPxEN` gates the service request output `STMIR0`/`STMIR1` (`STM_SRCx.SRR`)
    /// inside [`Self::stm_set_ir`].
    pub(crate) fn stm_events(&mut self, prev: u64, cur: u64) {
        for i in 0..2 {
            if self.stm.matched(i, prev, cur) {
                self.stm_set_ir(i);
            }
        }
    }

    /// Instructions until the next STM compare match that can raise a CPU interrupt (comparator
    /// enabled and its SRC enabled for the CPU). Other matches only set flags; they are applied
    /// at the end of the quantum in which they occur.
    pub(crate) fn insns_to_next_stm_event(&self) -> u64 {
        let mut best = u64::MAX;
        for (i, en, os) in [(0usize, 0x01u32, 0x04u32), (1, 0x10, 0x40)] {
            if self.stm.icr & en == 0 {
                continue;
            }
            let src = self.src_get(if self.stm.icr & os != 0 { STM_SRC1 } else { STM_SRC0 });
            if src & SRC_SRE == 0 || src & SRC_TOS != 0 {
                continue;
            }
            let t = self.stm.next_match(i, self.stm.base);
            best = best.min(self.stm.insns_until(t));
        }
        best
    }

    // ------------------------------------------------------------------------------------------
    // SRC registers (single source of truth: the peripheral register files)
    // ------------------------------------------------------------------------------------------

    #[inline]
    pub(crate) fn src_get(&self, addr: u32) -> u32 {
        if self.seg_f7e0.contains(addr) {
            self.seg_f7e0.get(addr)
        } else {
            self.seg_f000.get(addr)
        }
    }

    #[inline]
    pub(crate) fn src_put(&mut self, addr: u32, v: u32) {
        if self.seg_f7e0.contains(addr) {
            self.seg_f7e0.set(addr, v);
        } else {
            self.seg_f000.set(addr, v);
        }
        // Maintain the set of CPU-routed requests that can win arbitration.
        let candidate = v & (SRC_SRE | SRC_SRR) == (SRC_SRE | SRC_SRR) && v & SRC_TOS == 0 && v & SRC_SRPN != 0;
        match self.src_active.iter().position(|&a| a == addr) {
            Some(i) if !candidate => {
                self.src_active.swap_remove(i);
            }
            None if candidate => self.src_active.push(addr),
            _ => {}
        }
        self.engine_io.note_src(addr, v);
    }

    /// Guest write to an SRC register (byte-lane aware).
    pub(crate) fn src_write(&mut self, addr: u32, val: u32, mask: u32) {
        let old = self.src_get(addr);
        let cfg_mask = (SRC_SRPN | SRC_TOS | SRC_SRE) & mask;
        let mut new = (old & !cfg_mask) | (val & cfg_mask);
        let v = val & mask;
        let (set, clr) = (v & SRC_SETR != 0, v & SRC_CLRR != 0);
        if set && !clr {
            new |= SRC_SRR;
        } else if clr && !set {
            new &= !SRC_SRR;
        }
        new &= SRC_SRPN | SRC_TOS | SRC_SRE | SRC_SRR;
        self.src_put(addr, new);
        if new & (SRC_SRE | SRC_SRR) == (SRC_SRE | SRC_SRR) {
            self.stop_requested = true;
        }
    }

    /// Hardware service request: set SRR.
    pub(crate) fn src_set_srr(&mut self, addr: u32) {
        let v = self.src_get(addr) | SRC_SRR;
        self.src_put(addr, v);
        if v & SRC_SRE != 0 {
            self.stop_requested = true;
        }
    }

    /// Highest-priority pending CPU service request: `(SRPN, src address)`.
    pub fn pending_cpu_request(&self) -> Option<(u32, u32)> {
        let mut best: Option<(u32, u32)> = None;
        for &a in &self.src_active {
            let pn = self.src_get(a) & SRC_SRPN;
            if best.is_none_or(|(b, _)| pn > b) {
                best = Some((pn, a));
            }
        }
        best
    }

    /// Whether the CPU takes a pending request with this ICR (`ICR.IE` and SRPN > `ICR.CCPN`).
    pub fn irq_wins(&self, icr: u32) -> bool {
        icr & ICR_IE != 0 && self.pending_cpu_request().is_some_and(|(p, _)| p > icr & 0xFF)
    }

    /// Service requests routed to the PCP (TOS != CPU) that are pending. The PCP is not emulated,
    /// so these stay pending and are never delivered to the TriCore.
    pub fn pending_pcp_requests(&self) -> usize {
        src_table()
            .iter()
            .filter(|&&a| {
                let v = self.src_get(a);
                v & SRC_SRR != 0 && v & SRC_TOS != 0
            })
            .count()
    }

    /// Reset all SRCs to their reset value (0).
    pub(crate) fn src_reset_all(&mut self) {
        for &a in src_table() {
            self.src_put(a, 0);
        }
        self.src_active.clear();
    }
}

// TriCore PSW/PCXI/ICR fields (TriCore 1.3/1.6 layout, identical to Unicorn's helpers).
const PSW_CDC: u32 = 0x7F;
const PSW_CDE: u32 = 1 << 7;
const PSW_GW: u32 = 1 << 8;
const PSW_IS: u32 = 1 << 9;
const PSW_IO: u32 = 3 << 10;
const PSW_PRS: u32 = 3 << 12;
const ICR_IE: u32 = 1 << 8;
const PCXI_UL: u32 = 1 << 22;
const PCXI_PIE: u32 = 1 << 23;

/// Effective address of a context save area from a link word (FCX/PCXI format).
#[inline]
pub fn csa_ea(link: u32) -> u32 {
    ((link & 0x000F_0000) << 12) | ((link & 0x0000_FFFF) << 6)
}

impl Simos18Board {
    /// Architectural PSW. Unicorn keeps C/V/SV/AV/SAV in the PSW_USB_* cache (flag = bit 31, C =
    /// any non-zero value) and `uc_reg_read(PSW)` returns the raw word with stale status bits;
    /// fold them like QEMU's `psw_read` (keeping `PSW[24]`, `RM[0]`).
    pub fn psw(&self) -> u32 {
        let f = |r, sh: u32| (self.read_reg(r) >> 31) << sh;
        (self.read_reg(TriCoreReg::PSW) & 0x07FF_FFFF)
            | ((self.read_reg(TriCoreReg::PSW_USB_C) != 0) as u32) << 31
            | f(TriCoreReg::PSW_USB_V, 30)
            | f(TriCoreReg::PSW_USB_SV, 29)
            | f(TriCoreReg::PSW_USB_AV, 28)
            | f(TriCoreReg::PSW_USB_SAV, 27)
    }

    /// Write the architectural PSW: the raw word and the PSW_USB_* cache, like `psw_write`.
    pub(crate) fn set_psw(&mut self, v: u32) {
        self.write_reg(TriCoreReg::PSW, v);
        self.write_reg(TriCoreReg::PSW_USB_C, v & 1 << 31);
        self.write_reg(TriCoreReg::PSW_USB_V, (v & 1 << 30) << 1);
        self.write_reg(TriCoreReg::PSW_USB_SV, (v & 1 << 29) << 2);
        self.write_reg(TriCoreReg::PSW_USB_AV, (v & 1 << 28) << 3);
        self.write_reg(TriCoreReg::PSW_USB_SAV, (v & 1 << 27) << 4);
    }

    /// Evaluate the ICU and take the highest-priority pending interrupt if `ICR.IE == 1` and
    /// `PIPN > CCPN`. Implements the TriCore interrupt entry (upper-context save into the free
    /// CSA list, PCXI/FCX link update, ISP switch, PSW/ICR update, vector to `BIV | PIPN << 5`).
    /// The winning SRC's SRR is acknowledged only after the context save succeeded.
    pub fn dispatch_pending_irq(&mut self) -> bool {
        if self.native.is_some() {
            return self.native_dispatch_irq();
        }
        let pending = self.hw().pending_cpu_request();
        if pending.is_none() && self.hw().pipn_published == 0 {
            return false;
        }
        let icr = self.read_reg(TriCoreReg::ICR);
        let pipn = pending.map_or(0, |(p, _)| p);
        // Keep ICR.PIPN coherent with the arbitration result.
        let icr_new = (icr & !0x00FF_0000) | (pipn << 16);
        if icr_new != icr {
            self.write_reg(TriCoreReg::ICR, icr_new);
        }
        self.hw_mut().pipn_published = pipn as u8;
        let Some((pipn, src_addr)) = pending else {
            return false;
        };
        if icr & ICR_IE == 0 || pipn <= (icr & 0xFF) {
            return false;
        }

        let fcx = self.read_reg(TriCoreReg::FCX);
        let pc = self.read_reg(TriCoreReg::PC);
        if fcx & 0x000F_FFFF == 0 {
            // No free context: FCU trap (class 3, TIN 4); the request stays pending.
            self.enter_trap(3, 4, pc, false);
            return true;
        }
        let ea = csa_ea(fcx);
        let new_fcx = self.read_u32(ea);
        let psw = self.psw();
        let pcxi = self.read_reg(TriCoreReg::PCXI);
        let upper = [
            pcxi,
            psw,
            self.read_reg(TriCoreReg::A10),
            self.read_reg(TriCoreReg::A11),
            self.read_reg(TriCoreReg::D8),
            self.read_reg(TriCoreReg::D9),
            self.read_reg(TriCoreReg::D10),
            self.read_reg(TriCoreReg::D11),
            self.read_reg(TriCoreReg::A12),
            self.read_reg(TriCoreReg::A13),
            self.read_reg(TriCoreReg::A14),
            self.read_reg(TriCoreReg::A15),
            self.read_reg(TriCoreReg::D12),
            self.read_reg(TriCoreReg::D13),
            self.read_reg(TriCoreReg::D14),
            self.read_reg(TriCoreReg::D15),
        ];
        let mut bytes = [0u8; 64];
        for (i, w) in upper.iter().enumerate() {
            bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        if self.write_mem(ea, &bytes).is_err() {
            // The CSA is not writable: on hardware this is a bus error during context save.
            self.enter_trap(4, 2, pc, false);
            return true;
        }

        let new_pcxi = ((icr & 0xFF) << 24)
            | if icr & ICR_IE != 0 { PCXI_PIE } else { 0 }
            | PCXI_UL
            | (fcx & 0x000F_FFFF);
        self.write_reg(TriCoreReg::PCXI, new_pcxi);
        self.write_reg(TriCoreReg::FCX, (fcx & 0xFFF0_0000) | (new_fcx & 0x000F_FFFF));
        self.write_reg(TriCoreReg::A11, pc);
        if psw & PSW_IS == 0 {
            let isp = self.read_reg(TriCoreReg::ISP);
            self.write_reg(TriCoreReg::A10, isp);
        }
        // V1 §5.3 (TC1.6 and TC1.3.1): CDC cleared, PSW.CDE unchanged (only trap entry, §6.2.5, sets it)
        let psw_new = (psw & !(PSW_CDC | PSW_GW | PSW_IO | PSW_PRS)) | PSW_IS | (2 << 10);
        self.set_psw(psw_new);
        self.write_reg(TriCoreReg::ICR, (icr_new & !(ICR_IE | 0xFF)) | pipn);
        let biv = self.read_reg(TriCoreReg::BIV);
        self.write_reg(TriCoreReg::PC, biv | (pipn << 5));

        // Acknowledge: clear SRR of the serviced request.
        let hw = self.hw_mut();
        let v = hw.src_get(src_addr) & !SRC_SRR;
        hw.src_put(src_addr, v);
        hw.irq_dispatch_count += 1;
        if let Some(log) = hw.event_log.as_mut() {
            log.push((hw.total_instructions + hw.quantum_executed, 2, pc, pipn as u64));
        }

        if fcx == self.read_reg(TriCoreReg::LCX) {
            // Last free context consumed: FCD trap, return address = ISR entry.
            let isr = biv | (pipn << 5);
            self.enter_trap(3, 5, isr, true);
        }
        true
    }

    /// TriCore trap entry (class/TIN), mirroring the architecture definition. `save_upper` is
    /// false for FCU (no context can be saved).
    pub(crate) fn enter_trap(&mut self, class: u32, tin: u32, ret_pc: u32, save_upper: bool) {
        let psw = self.psw();
        let icr = self.read_reg(TriCoreReg::ICR);
        if save_upper {
            let fcx = self.read_reg(TriCoreReg::FCX);
            if fcx & 0x000F_FFFF != 0 {
                let ea = csa_ea(fcx);
                let new_fcx = self.read_u32(ea);
                let regs = [
                    self.read_reg(TriCoreReg::PCXI),
                    psw,
                    self.read_reg(TriCoreReg::A10),
                    self.read_reg(TriCoreReg::A11),
                    self.read_reg(TriCoreReg::D8),
                    self.read_reg(TriCoreReg::D9),
                    self.read_reg(TriCoreReg::D10),
                    self.read_reg(TriCoreReg::D11),
                    self.read_reg(TriCoreReg::A12),
                    self.read_reg(TriCoreReg::A13),
                    self.read_reg(TriCoreReg::A14),
                    self.read_reg(TriCoreReg::A15),
                    self.read_reg(TriCoreReg::D12),
                    self.read_reg(TriCoreReg::D13),
                    self.read_reg(TriCoreReg::D14),
                    self.read_reg(TriCoreReg::D15),
                ];
                let mut bytes = [0u8; 64];
                for (i, w) in regs.iter().enumerate() {
                    bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
                }
                let _ = self.write_mem(ea, &bytes);
                let pcxi = ((icr & 0xFF) << 24)
                    | if icr & ICR_IE != 0 { PCXI_PIE } else { 0 }
                    | PCXI_UL
                    | (fcx & 0x000F_FFFF);
                self.write_reg(TriCoreReg::PCXI, pcxi);
                self.write_reg(TriCoreReg::FCX, (fcx & 0xFFF0_0000) | (new_fcx & 0x000F_FFFF));
            }
        }
        self.write_reg(TriCoreReg::D15, tin);
        self.write_reg(TriCoreReg::A11, ret_pc);
        if psw & PSW_IS == 0 {
            let isp = self.read_reg(TriCoreReg::ISP);
            self.write_reg(TriCoreReg::A10, isp);
        }
        let psw_new = (psw & !(PSW_CDC | PSW_GW | PSW_IO | PSW_PRS)) | PSW_IS | (2 << 10) | PSW_CDE;
        self.set_psw(psw_new);
        self.write_reg(TriCoreReg::ICR, icr & !ICR_IE);
        let btv = self.read_reg(TriCoreReg::BTV);
        self.write_reg(TriCoreReg::PC, btv | (class << 5));
        self.hw_mut().trap_count += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stm(cmp0: u32, cmcon: u32) -> StmState {
        StmState {
            clc: 0x300, // RMC = 3
            fpidiv: 1,  // FPI = CPU / 2
            cmp: [cmp0, 0],
            cmcon,
            ..Default::default()
        }
    }

    #[test]
    fn stm_compare_is_edge_triggered() {
        let s = stm(1000, 0x1F);
        assert!(!s.matched(0, 0, 999));
        assert!(s.matched(0, 0, 1000));
        assert!(s.matched(0, 999, 1000));
        // already equal at `prev`: no second event while the counter stays/passes
        assert!(!s.matched(0, 1000, 1001));
        assert!(!s.matched(0, 1000, 5000));
        // no level clause: counter far beyond CMP without passing it again
        assert!(!s.matched(0, 2000, 3000));
        // full 32-bit wrap brings the next equality
        assert!(s.matched(0, 2000, (1u64 << 32) + 1000));
    }

    #[test]
    fn stm_compare_zero_is_valid_and_wraps() {
        let s = stm(0, 0x1F);
        assert!(!s.matched(0, 5, 0xFFFF_FFFF));
        assert!(s.matched(0, 0xFFFF_FFF0, 0x1_0000_0000));
        assert_eq!(s.next_match(0, 0xFFFF_FFF0), 0x1_0000_0000);
    }

    #[test]
    fn stm_compare_respects_msize_mstart() {
        // MSIZE0 = 7 (8 bits), MSTART0 = 4: compare STM[11:4] with CMP0[7:0].
        let s = stm(0x12, 7 | (4 << 8));
        assert_eq!(s.next_match(0, 0), 0x120);
        assert!(s.matched(0, 0x11F, 0x120));
        assert!(!s.matched(0, 0x120, 0x12F)); // still equal: no re-trigger
        assert_eq!(s.next_match(0, 0x120), 0x1120); // next 8-bit wrap of the field
    }

    #[test]
    fn stm_time_base_is_instruction_proportional() {
        let mut s = stm(0, 0x1F);
        assert_eq!(s.cycles_per_tick(), 6);
        // (FPIDIV+1) * RMC = 6 instructions per STM tick
        let (p, c) = s.advance(6000);
        assert_eq!((p, c), (0, 1000));
        let (_, c) = s.advance(5);
        assert_eq!(c, 1000);
        assert_eq!(s.now(1), 1001);
        assert_eq!(s.insns_until(1002), 7);
    }

    #[test]
    fn compare_events_require_their_enable_bits() {
        let mut b = board();
        let hw = b.hw_mut();
        hw.stm.clc = 0x300;
        hw.stm.fpidiv = 1;
        hw.stm.cmcon = 0;
        hw.stm.cmp = [0, 0];
        hw.stm.icr = 0;
        // Fig. 15-3: compare match latches CMPxIR even with CMPxEN = 0, but never raises STMIRx (SRR).
        hw.stm_events(0, 1000);
        assert_eq!(hw.stm.icr & 0x22, 0x22);
        assert_eq!(hw.src_get(STM_SRC0) & SRC_SRR, 0);
        assert_eq!(hw.src_get(STM_SRC1) & SRC_SRR, 0);

        // Clear CMP0IR/CMP1IR via STM_ISRR, enable CMP0EN -> STM_SRC0.SRR raised on match.
        hw.stm_write(0x40, 0x05, u32::MAX);
        assert_eq!(hw.stm.icr & 0x22, 0);
        hw.stm.icr = 0x01;
        hw.stm_events(1000, 2000);
        assert_eq!(hw.stm.icr & 0x02, 0x02);
        assert_eq!(hw.src_get(STM_SRC0) & SRC_SRR, SRC_SRR);
        assert_eq!(hw.src_get(STM_SRC1) & SRC_SRR, 0);

        // Software set via STM_ISRR.CMP0IRS sets CMP0IR without pulsing STM_SRC0.SRR (UM §15.2.3 p.15-6).
        hw.src_write(STM_SRC0, SRC_CLRR, SRC_CLRR);
        hw.stm_write(0x40, 0x05, u32::MAX);
        hw.stm_write(0x40, 0x02, u32::MAX);
        assert_eq!(hw.stm.icr & 0x02, 0x02);
        assert_eq!(hw.src_get(STM_SRC0) & SRC_SRR, 0);

        // Enable CMP1EN routed to STMIR1 (CMP1OS = 1) -> STM_SRC1.SRR raised on match.
        hw.stm_write(0x40, 0x05, u32::MAX);
        hw.stm.icr = 0x50;
        hw.stm_events(2000, 3000);
        assert_eq!(hw.stm.icr & 0x20, 0x20);
        assert_eq!(hw.src_get(STM_SRC0) & SRC_SRR, 0);
        assert_eq!(hw.src_get(STM_SRC1) & SRC_SRR, SRC_SRR);
    }

    use crate::memory::BoardVariant;
    use crate::Simos18Board;

    fn board() -> Simos18Board {
        Simos18Board::new_variant(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791).unwrap()
    }

    #[test]
    fn src_set_clear_semantics_and_byte_lanes() {
        let mut b = board();
        let hw = b.hw_mut();
        let a = 0xF7E0_FFF0;
        hw.mmio_write(a, 4, (SRC_SETR | SRC_SRE | 5) as u64);
        assert_eq!(hw.src_get(a), SRC_SRR | SRC_SRE | 5, "SETR sets SRR, SETR itself reads 0");
        assert_eq!(hw.mmio_read(a, 4) as u32 & (SRC_SETR | SRC_CLRR), 0);
        hw.mmio_write(a, 4, (SRC_SRE | 7) as u64);
        assert_eq!(hw.src_get(a), SRC_SRR | SRC_SRE | 7, "config write keeps SRR");
        hw.mmio_write(a, 4, (SRC_SETR | SRC_CLRR | SRC_SRE | 7) as u64);
        assert_eq!(hw.src_get(a) & SRC_SRR, SRC_SRR, "SETR+CLRR together: no change");
        // byte write to SRC+1 (bits 15:8) with CLRR only
        hw.mmio_write(a + 1, 1, (SRC_CLRR >> 8) as u64);
        assert_eq!(hw.src_get(a), 7, "byte lane: CLRR clears SRR; SRE (same lane) written 0; SRPN kept");
        hw.mmio_write(a + 1, 1, ((SRC_SETR | SRC_SRE) >> 8) as u64);
        assert_eq!(hw.src_get(a), SRC_SRR | SRC_SRE | 7);
        assert_eq!(hw.pending_cpu_request(), Some((7, a)));
        // TOS != CPU (PCP) is never offered to the CPU
        hw.mmio_write(a, 4, (SRC_SRE | 0x400 | 7) as u64);
        assert_eq!(hw.pending_cpu_request(), None);
    }

    #[test]
    fn interrupt_entry_saves_upper_context_and_vectors() {
        let mut b = board();
        // code: `nop` x4 at PSPR; ISR vector table at 0xC0001000
        b.write_mem(0xC000_0000, &[0u8; 64]).unwrap();
        let csa = 0xD000_1000u32;
        let link = |ea: u32| ((ea >> 12) & 0xF_0000) | ((ea >> 6) & 0xFFFF);
        // free list: csa -> csa+64 -> 0
        b.write_mem(csa, &link(csa + 64).to_le_bytes()).unwrap();
        b.write_reg(TriCoreReg::FCX, link(csa));
        b.write_reg(TriCoreReg::LCX, 0);
        b.write_reg(TriCoreReg::BIV, 0xC000_1000);
        b.write_reg(TriCoreReg::ISP, 0xD000_8000);
        b.write_reg(TriCoreReg::A10, 0xD000_4000);
        b.write_reg(TriCoreReg::PSW, 0x0000_0B80 & !(1 << 9)); // IS = 0
        b.write_reg(TriCoreReg::ICR, 0x100 | 2); // IE, CCPN = 2
        b.write_reg(TriCoreReg::D8, 0x1234_5678);
        b.write_reg(TriCoreReg::PC, 0xC000_0000);
        b.hw_mut().mmio_write(0xF7E0_FFF0, 4, (SRC_SETR | SRC_SRE | 9) as u64);
        assert!(b.dispatch_pending_irq());
        assert_eq!(b.read_reg(TriCoreReg::PC), 0xC000_1000 | (9 << 5));
        assert_eq!(b.read_reg(TriCoreReg::A11), 0xC000_0000);
        assert_eq!(b.read_reg(TriCoreReg::A10), 0xD000_8000, "switched to ISP");
        let icr = b.read_reg(TriCoreReg::ICR);
        assert_eq!((icr & 0x1FF, (icr >> 16) & 0xFF), (9, 9), "IE=0, CCPN=PIPN");
        let pcxi = b.read_reg(TriCoreReg::PCXI);
        assert_eq!(pcxi >> 24, 2, "PCPN = old CCPN");
        assert_ne!(pcxi & PCXI_PIE, 0);
        assert_ne!(pcxi & PCXI_UL, 0);
        assert_eq!(pcxi & 0xF_FFFF, link(csa));
        assert_eq!(b.read_reg(TriCoreReg::FCX), link(csa + 64));
        let psw = b.read_reg(TriCoreReg::PSW);
        assert_eq!(psw & (PSW_IS | PSW_GW | PSW_IO), PSW_IS | (2 << 10));
        assert_eq!(b.read_u32(csa + 0x10), 0x1234_5678, "D8 saved in upper context");
        assert_eq!(b.read_u32(csa + 0x08), 0xD000_4000, "old A10 saved");
        assert_eq!(b.hw().src_get(0xF7E0_FFF0) & SRC_SRR, 0, "acknowledged");
        // a lower-or-equal priority request is not taken while CCPN = 9
        b.write_reg(TriCoreReg::ICR, b.read_reg(TriCoreReg::ICR) | 0x100);
        b.hw_mut().mmio_write(0xF7E0_FFF4, 4, (SRC_SETR | SRC_SRE | 9) as u64);
        assert!(!b.dispatch_pending_irq());
    }

    #[test]
    fn interrupt_with_empty_free_list_traps_fcu_and_stays_pending() {
        let mut b = board();
        b.write_reg(TriCoreReg::FCX, 0);
        b.write_reg(TriCoreReg::BTV, 0xC000_2000);
        b.write_reg(TriCoreReg::ICR, 0x100);
        b.write_reg(TriCoreReg::PC, 0xC000_0000);
        b.hw_mut().mmio_write(0xF7E0_FFF0, 4, (SRC_SETR | SRC_SRE | 3) as u64);
        assert!(b.dispatch_pending_irq());
        assert_eq!(b.read_reg(TriCoreReg::PC), 0xC000_2000 | (3 << 5), "context-management trap");
        assert_eq!(b.read_reg(TriCoreReg::D15), 4, "TIN = FCU");
        assert_ne!(b.hw().src_get(0xF7E0_FFF0) & SRC_SRR, 0, "request not acknowledged");
    }

    #[test]
    fn src_table_contains_core_requests() {
        for a in [STM_SRC0, STM_SRC1, 0xF7E0_FFF0, can_src(1), 0xF000_3EFC] {
            assert!(is_src(a), "{a:08X}");
        }
        assert!(!is_src(0xF000_0230));
    }
}
