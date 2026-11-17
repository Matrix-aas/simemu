//! TC1791 DMA controller (16 channels, one flat channel set) — the subset the SIMOS18.1 firmware
//! programs, with real data movement through a [`DmaBus`].
//!
//! Register map as used by the unmodified SC800S50 firmware (base 0xF000_3C00; SBOOT 0x80005514
//! writes CLC; the DMA driver works through the pointer at 0xD000_0648):
//!
//! | off            | reg     | evidence |
//! |----------------|---------|----------|
//! | 0x000          | CLC     | 0x80005514 writes 8 |
//! | 0x010          | CHRSTR  | 0x800A93B6 writes 0x800 (reset channel 11) |
//! | 0x014          | TRSR    | request state: bits 15:0 pending, 31:16 hardware request enabled |
//! | 0x018          | STREQ   | 0x800A8F3E writes 1 << ch (software request of the TX channel) |
//! | 0x01C          | HTREQ   | 0x800A8E7A/8EB4 write 1 << ch (enable hardware request); bits 31:16 disable |
//! | 0x020..0x028   | EER/ERRSR/CLRE | error enable/status/clear (bit n = channel n) |
//! | 0x054 / 0x058  | INTSR / INTCR  | channel interrupt flags ICHn (bit n) / clear |
//! | 0x080 + 0x20*n | CHSR, CHCR, CHICR, ADRCR, SADR, DADR, SHADR | 0x80112138 (CHCR), 0x80112148 (ADRCR), 0x80112172 (CHICR), 0x800A9342/935E (SADR/DADR) |
//! | 0x2E0..0x2FC   | DMA_SRC7..SRC0 | owned by the interrupt system (`stm_icu`); SRCn = 0xF0003EFC - 4n |
//!
//! Channel register layout (bit fields as the firmware writes them):
//! * CHCR: `TREL [8:0]`, `PRSEL [15:12]` (request input select: 0x00204000 = PRSEL 4 for the SSC
//!   channels 0..3, 0x00406000 = PRSEL 6, 0x0040E000 = PRSEL 14 ...), `BLKM [18:16]`, `RROAT [19]`,
//!   `CHMODE [20]`, `CHDW [22:21]` (0 = 8, 1 = 16, 2 = 32 bit).
//! * CHICR: `WRPSE [0]`, `WRPDE [1]`, `INTCT [3:2]`, `INTP [7:4]` (selects `DMA_SRC<INTP>`; e.g. ch1 INTP 2
//!   is paired with the enable of SRC 0xF0003EF4 = SRC2 at 0x8008D556), `WRPP [11:8]`, `IRDV [15:12]`.
//!   The SSC RX channels use INTCT = 10b, IRDV = 0: one interrupt when the transaction ends.
//! * ADRCR: `SMF [2:0]`, `INCS [3]`, `DMF [6:4]`, `INCD [7]`, `CBLS [11:8]`, `CBLD [15:12]`: 0xF080 = source
//!   fixed (SSC RB, CBLS = 0), destination +2 per move (RAM buffer, CBLD = 15); 0x0F08 = source +2,
//!   destination fixed (SSC TB, CBLD = 0). Address offset = (data width << SMF); upper bits
//!   `[31:CBL]` are frozen (circular buffer of 2^CBL bytes; CBL = 0 freezes all 32 bits, TC1796 UM
//!   §12.1.4.7 p.12-19 & §12.7.2 p.12-84..85 — VERIFIED-DS).
//!
//! Transaction semantics: a request with TCOUNT = 0 loads TCOUNT := max(TREL, 1). Each request
//! performs one transfer (2^BLKM-style move count, BLKM table below) and decrements TCOUNT; with
//! RROAT = 0 the request flag is cleared after every transfer (hardware-paced), with RROAT = 1 only
//! at the end of the transaction. At TCOUNT = 0 a single-mode channel (CHMODE = 0) drops its
//! hardware request enable — the driver re-enables it via HTREQ for every job, as observed.
//!
//! Not modelled (unused by the firmware): shadow address/SHCT, pattern detection, wrap-buffer
//! interrupts, move-engine access protection (the 0x30..0x7C registers are left to the plain
//! register file), bus-error interrupts (errors are only latched in ERRSR).

pub const DMA_BASE: u32 = 0xF000_3C00;
pub const DMA_CHANNELS: usize = 16;
/// DMA_SRC0; DMA_SRCn = DMA_SRC0 - 4 * n.
pub const DMA_SRC0: u32 = 0xF000_3EFC;

pub const CLC: u32 = 0x000;
pub const CHRSTR: u32 = 0x010;
pub const TRSR: u32 = 0x014;
pub const STREQ: u32 = 0x018;
pub const HTREQ: u32 = 0x01C;
pub const EER: u32 = 0x020;
pub const ERRSR: u32 = 0x024;
pub const CLRE: u32 = 0x028;
pub const INTSR: u32 = 0x054;
pub const INTCR: u32 = 0x058;
pub const CH_BASE: u32 = 0x080;
pub const CH_STRIDE: u32 = 0x20;

/// Service request node address for CHICR.INTP / WRPP value `n`.
#[inline]
pub fn src_addr(n: u32) -> u32 {
    DMA_SRC0 - 4 * (n & 7)
}

/// Memory/peripheral access of a DMA move. `bytes` is 1, 2 or 4; `None`/`false` = bus error.
pub trait DmaBus {
    fn dma_read(&mut self, addr: u32, bytes: u32) -> Option<u32>;
    fn dma_write(&mut self, addr: u32, bytes: u32, val: u32) -> bool;
}

#[derive(Debug, Default, Clone, Copy)]
pub struct DmaChannel {
    pub chcr: u32,
    pub chicr: u32,
    pub adrcr: u32,
    pub sadr: u32,
    pub dadr: u32,
    pub tcount: u32,
    /// TRSR.CHn: transaction request pending.
    pub pending: bool,
    /// Hardware request enabled (HTREQ.ECHn ... DCHn).
    pub htre: bool,
    /// Transfers performed since reset (diagnostics).
    pub transfers: u64,
}

impl DmaChannel {
    pub fn trel(&self) -> u32 {
        self.chcr & 0x1FF
    }
    pub fn prsel(&self) -> u32 {
        (self.chcr >> 12) & 0xF
    }
    fn moves_per_transfer(&self) -> u32 {
        [1, 2, 4, 8, 16, 3, 5, 9][((self.chcr >> 16) & 7) as usize]
    }
    fn rroat(&self) -> bool {
        self.chcr & (1 << 19) != 0
    }
    fn continuous(&self) -> bool {
        self.chcr & (1 << 20) != 0
    }
    pub fn width_bytes(&self) -> u32 {
        match (self.chcr >> 21) & 3 {
            0 => 1,
            1 => 2,
            _ => 4,
        }
    }
}

fn step(addr: u32, width: u32, factor: u32, inc: bool, cbl: u32) -> u32 {
    let off = width << factor;
    let mask = (1u32 << cbl) - 1;
    let low = if inc { addr.wrapping_add(off) } else { addr.wrapping_sub(off) };
    (addr & !mask) | (low & mask)
}

#[derive(Debug, Clone)]
pub struct Dma {
    pub clc: u32,
    pub eer: u32,
    pub errsr: u32,
    pub intsr: u32,
    pub ch: [DmaChannel; DMA_CHANNELS],
    pending_mask: u16,
}

impl Default for Dma {
    fn default() -> Self {
        Self { clc: 0, eer: 0, errsr: 0, intsr: 0, ch: [DmaChannel::default(); DMA_CHANNELS], pending_mask: 0 }
    }
}

/// Result of [`Dma::service`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct DmaOutcome {
    /// Bit n set: DMA_SRCn service request raised (set SRR).
    pub src_requests: u8,
    pub transfers: u32,
}

impl Dma {
    fn chan(off: u32) -> Option<(usize, u32)> {
        let rel = off.checked_sub(CH_BASE)?;
        let n = (rel / CH_STRIDE) as usize;
        (n < DMA_CHANNELS).then_some((n, rel % CH_STRIDE))
    }

    pub fn read(&self, off: u32) -> Option<u32> {
        if let Some((n, r)) = Self::chan(off) {
            let c = &self.ch[n];
            return Some(match r {
                0x00 => c.tcount,
                0x04 => c.chcr,
                0x08 => c.chicr,
                0x0C => c.adrcr,
                0x10 => c.sadr,
                0x14 => c.dadr,
                0x18 => 0, // SHADR: shadow transfers not modelled
                _ => return None,
            });
        }
        Some(match off {
            CLC => (self.clc & !2) | ((self.clc & 1) << 1),
            TRSR => self.ch.iter().enumerate().fold(0, |a, (i, c)| {
                a | (c.pending as u32) << i | (c.htre as u32) << (16 + i)
            }),
            CHRSTR | STREQ | HTREQ | CLRE | INTCR => 0,
            EER => self.eer,
            ERRSR => self.errsr,
            INTSR => self.intsr,
            _ => return None,
        })
    }

    /// Register write (value already shifted into its byte lanes, `mask` = written lanes).
    /// Returns false for offsets this model does not own.
    pub fn write(&mut self, off: u32, val: u32, mask: u32) -> bool {
        let v = val & mask;
        let merge = |old: u32| (old & !mask) | v;
        if let Some((n, r)) = Self::chan(off) {
            let c = &mut self.ch[n];
            match r {
                0x04 => c.chcr = merge(c.chcr),
                0x08 => c.chicr = merge(c.chicr),
                0x0C => c.adrcr = merge(c.adrcr),
                0x10 => c.sadr = merge(c.sadr),
                0x14 => c.dadr = merge(c.dadr),
                0x00 | 0x18 => {}
                _ => return false,
            }
            return true;
        }
        match off {
            CLC => self.clc = merge(self.clc),
            CHRSTR => {
                self.pending_mask &= !(v as u16);
                for (i, c) in self.ch.iter_mut().enumerate() {
                    if v & (1 << i) != 0 {
                        c.tcount = 0;
                        c.pending = false;
                        c.htre = false;
                    }
                }
            }
            STREQ => {
                self.pending_mask |= v as u16;
                for (i, c) in self.ch.iter_mut().enumerate() {
                    if v & (1 << i) != 0 {
                        c.pending = true;
                    }
                }
            }
            HTREQ => {
                for (i, c) in self.ch.iter_mut().enumerate() {
                    if v & (1 << i) != 0 {
                        c.htre = true;
                    }
                    if v & (1 << (16 + i)) != 0 {
                        c.htre = false;
                    }
                }
            }
            EER => self.eer = merge(self.eer),
            CLRE => self.errsr &= !v,
            INTCR => self.intsr &= !v,
            TRSR | ERRSR | INTSR => {}
            _ => return false,
        }
        true
    }

    /// Hardware request input `line` (the PRSEL value that selects it) of channel `ch`.
    #[inline]
    pub fn hw_request(&mut self, ch: usize, line: u32) {
        if let Some(c) = self.ch.get_mut(ch) {
            if c.htre && c.prsel() == line {
                c.pending = true;
                self.pending_mask |= 1 << ch;
            }
        }
    }

    #[inline]
    pub fn pending(&self) -> bool {
        self.clc & 1 == 0 && self.pending_mask != 0
    }

    /// Execute all pending transfers (highest channel number first). One transfer per pending
    /// request; RROAT channels run their whole transaction.
    pub fn service(&mut self, bus: &mut impl DmaBus) -> DmaOutcome {
        let mut out = DmaOutcome::default();
        if self.clc & 1 != 0 || self.pending_mask == 0 {
            return out; // module disabled or nothing pending
        }
        let mut m = self.pending_mask;
        while m != 0 {
            let n = 15 - m.leading_zeros() as usize;
            m &= !(1 << n);
            while self.ch[n].pending {
                if !self.transfer(n, bus, &mut out) {
                    break;
                }
            }
        }
        out
    }

    /// One transfer of channel `n`; false when the channel stops (bus error).
    fn transfer(&mut self, n: usize, bus: &mut impl DmaBus, out: &mut DmaOutcome) -> bool {
        let c = &mut self.ch[n];
        if c.tcount == 0 {
            c.tcount = c.trel().max(1);
        }
        let w = c.width_bytes();
        let a = c.adrcr;
        for _ in 0..c.moves_per_transfer() {
            let ok = match bus.dma_read(c.sadr, w) {
                Some(d) => bus.dma_write(c.dadr, w, d),
                None => false,
            };
            if !ok {
                c.pending = false;
                self.pending_mask &= !(1 << n);
                self.errsr |= 1 << n;
                return false;
            }
            c.sadr = step(c.sadr, w, a & 7, a & 8 != 0, (a >> 8) & 0xF);
            c.dadr = step(c.dadr, w, (a >> 4) & 7, a & 0x80 != 0, (a >> 12) & 0xF);
        }
        c.tcount -= 1;
        c.transfers += 1;
        out.transfers += 1;
        let intct = (c.chicr >> 2) & 3;
        let irdv = (c.chicr >> 12) & 0xF;
        if (intct == 2 && c.tcount == irdv) || (intct == 3 && c.tcount < irdv) {
            self.intsr |= 1 << n;
            out.src_requests |= 1 << ((c.chicr >> 4) & 7);
        }
        if c.tcount == 0 {
            c.pending = false;
            self.pending_mask &= !(1 << n);
            if !c.continuous() {
                c.htre = false;
            }
        } else if !c.rroat() {
            c.pending = false;
            self.pending_mask &= !(1 << n);
        }
        true
    }
}

// ---------------------------------------------------------------------------------------------
// SIMOS18.1 board integration: SSC0..3 + DMA + monitoring unit behind the core's MMIO dispatch.
// ---------------------------------------------------------------------------------------------

use crate::memory::Tc1791HardwareState;
use crate::monitor_device::MonitorDevice;
use crate::power_stages::{BoardSpiBus, PowerStages};
use crate::ssc::{self, Ssc, SscRequests, SSC_COUNT};
use crate::stm_icu::CYCLES_PER_INSN;

/// DMA request input (CHCR.PRSEL) on which the SSC requests arrive. Inferred from the firmware's
/// channel programming (no TC1791 DMA request table available): channels 0..3 are configured
/// with PRSEL 4 and ch0/ch1 read SSC0/SSC1.RB, ch2/ch3 write SSC0/SSC1.TB (S50 0x80112138,
/// 0x800A9342, 0x800A935E): ch n <- SSCn.RIR, ch 2+n <- SSCn.TIR.
pub const SSC_DMA_PRSEL: u32 = 4;

/// Serial peripherals of the board (one field of [`Tc1791HardwareState`]): SSC0..3, DMA, the
/// monitoring unit and the power-stage drivers with their MSC links.
pub struct SscDma {
    pub ssc: [Ssc; SSC_COUNT],
    pub dma: Dma,
    pub mu: MonitorDevice,
    /// Power-stage drivers and their MSC links (see `power_stages`).
    pub ps: PowerStages,
    /// SSC request lines raised while the DMA was busy: bit = DMA channel.
    lines: u32,
    active: bool,
}

impl Default for SscDma {
    fn default() -> Self {
        Self { ssc: std::array::from_fn(Ssc::new), dma: Dma::default(), mu: MonitorDevice::new(), ps: PowerStages::default(), lines: 0, active: false }
    }
}

impl SscDma {
    /// SoC reset: SSC/DMA registers to reset values; the MU sees the MC reset line.
    pub fn reset(&mut self) {
        self.ssc = std::array::from_fn(Ssc::new);
        self.dma = Dma::default();
        self.lines = 0;
        self.active = false;
        self.mu.on_mc_reset();
        self.ps.soc_reset();
    }

    fn route(&mut self, n: usize, req: SscRequests) {
        if n < 2 {
            if req.rir {
                self.lines |= 1 << n;
            }
            if req.tir {
                self.lines |= 1 << (2 + n);
            }
        }
    }
}

impl DmaBus for Tc1791HardwareState {
    fn dma_read(&mut self, addr: u32, bytes: u32) -> Option<u32> {
        if crate::memory::is_mmio(addr) {
            return Some(self.mmio_read(addr, bytes) as u32);
        }
        let s = self.sys_mem_slice(addr, bytes as usize)?;
        Some(s.iter().rev().fold(0, |a, &b| (a << 8) | b as u32))
    }

    fn dma_write(&mut self, addr: u32, bytes: u32, val: u32) -> bool {
        if crate::memory::is_mmio(addr) {
            self.mmio_write(addr, bytes, val as u64);
            return true;
        }
        let Some(s) = self.sys_ram_mut(addr, bytes as usize) else {
            return false; // flash/unmapped: bus error
        };
        s.copy_from_slice(&val.to_le_bytes()[..bytes as usize]);
        self.mark_ram_write_code_dirty(addr, bytes);
        true
    }
}

impl Tc1791HardwareState {
    /// Simulated time including the executed part of the running quantum.
    pub fn now_ns(&self) -> u64 {
        // An exact function of the absolute instruction count (`time_at`); the second term is 0.0
        // unless `sim_time_s` was moved from outside (unit tests).
        let n = self.total_instructions + self.quantum_executed;
        ((self.time_at(n) + (self.sim_time_s - self.time_at(self.total_instructions))) * 1e9) as u64
    }

    pub(crate) fn ffpi_hz(&self) -> u64 {
        self.scu.fcpu_hz() / (self.scu.fpidiv() as u64 + 1)
    }

    /// Side-effect-free peek of an SSC register.
    pub(crate) fn ssc_mmio_peek(&self, addr: u32) -> Option<u32> {
        let (n, off) = ssc::decode(addr)?;
        self.spi.ssc[n].peek(off)
    }

    /// MMIO read of an SSC register (`None`: not an SSC register this model owns).
    pub(crate) fn ssc_mmio_read(&mut self, addr: u32) -> Option<u32> {
        let (n, off) = ssc::decode(addr)?;
        if !self.spi.active {
            self.spi_dma_update();
        }
        self.spi.ssc[n].read(off)
    }

    pub(crate) fn ssc_mmio_write(&mut self, addr: u32, val: u32, mask: u32) -> bool {
        let Some((n, off)) = ssc::decode(addr) else { return false };
        if off == ssc::CLC && self.scu.endinit_blocks(addr) {
            return true;
        }
        let (now, fpi) = (self.now_ns(), self.ffpi_hz());
        let Some(req) = self.spi.ssc[n].write(off, val, mask, now, fpi) else { return false };
        self.spi.route(n, req);
        self.dma_run();
        true
    }

    pub(crate) fn dma_mmio_read(&mut self, addr: u32) -> Option<u32> {
        let off = addr.wrapping_sub(DMA_BASE);
        if matches!(off, TRSR | ERRSR | INTSR | 0x1E0..=0x1FC) {
            self.knock_fadc_update(); // DMA progress of the knock acquisition (ch 11 DADR) up to now
        }
        self.spi.dma.read(off)
    }

    pub(crate) fn dma_mmio_write(&mut self, addr: u32, val: u32, mask: u32) -> bool {
        let off = addr.wrapping_sub(DMA_BASE);
        if off == CLC && self.scu.endinit_blocks(addr) {
            return true;
        }
        if matches!(off, 0x1E0..=0x1FC)
            || (matches!(off, CHRSTR | STREQ | HTREQ | EER | CLRE | INTCR) && (val & mask) & ((1 << 11) | (1 << 27)) != 0)
        {
            self.knock_fadc_update();
        }
        let handled = self.spi.dma.write(off, val, mask);
        if handled {
            self.dma_run();
        }
        handled
    }

    /// Run the DMA until no request is pending. Moves go through the full MMIO model (a write to
    /// SSCn.TB starts the next frame); requests raised meanwhile are latched in `spi.lines`.
    pub(crate) fn dma_run(&mut self) {
        if self.spi.active {
            return;
        }
        self.spi.active = true;
        loop {
            let lines = std::mem::take(&mut self.spi.lines);
            for ch in 0..DMA_CHANNELS {
                if lines & (1 << ch) != 0 {
                    self.spi.dma.hw_request(ch, SSC_DMA_PRSEL);
                }
            }
            if !self.spi.dma.pending() {
                break;
            }
            let mut dma = std::mem::take(&mut self.spi.dma);
            let out = dma.service(self);
            self.spi.dma = dma;
            for n in 0..8 {
                if out.src_requests & (1 << n) != 0 {
                    self.src_set_srr(src_addr(n));
                }
            }
        }
        self.spi.active = false;
    }

    /// Bring SSC frames, DMA, the MU and the MSC links up to the current time (called at quantum
    /// boundaries and before SSC register reads). A MU reset request is turned into a SoC reset
    /// request.
    pub(crate) fn spi_dma_update(&mut self) {
        let now = self.now_ns();
        self.spi.mu.advance(now);
        if self.spi.lines != 0 || self.spi.dma.pending() || self.spi.ssc.iter().any(|s| s.has_due(now)) {
            let fpi = self.ffpi_hz();
            let lines = self.disable_lines();
            loop {
                let mut any = false;
                for n in 0..SSC_COUNT {
                    let SscDma { ssc, mu, ps, .. } = &mut self.spi;
                    let req = ssc[n].advance(now, fpi, &mut BoardSpiBus { mu, ps, lines });
                    if req.any() {
                        any = true;
                        self.spi.route(n, req);
                    }
                }
                self.dma_run();
                if !any {
                    break;
                }
            }
        }
        self.msc_update(now);
        if self.spi.mu.take_reset_request() && self.reset_request.is_none() {
            // The MU drives the MCU reset pin; mapped onto `ResetKind::PowerOn` (PORST).
            self.reset_request = Some(crate::scu::ResetKind::PowerOn);
            self.stop_requested = true;
        }
    }

    /// Instructions until the next SSC frame or MSC frame completes (quantum limit for exact timing).
    #[inline]
    pub(crate) fn insns_to_next_spi_event(&self) -> u64 {
        let mut best = None::<u64>;
        for s in &self.spi.ssc {
            if let Some(t) = s.next_event_ns() {
                best = Some(best.map_or(t, |b| b.min(t)));
            }
        }
        if let Some(t) = self.msc_next_event_ns() {
            best = Some(best.map_or(t, |b| b.min(t)));
        }
        let Some(t) = best else { return u64::MAX };
        let dt = t.saturating_sub(self.now_ns()) as u128;
        let insns = dt * self.clk_f as u128 / (1_000_000_000u128 * CYCLES_PER_INSN as u128);
        (insns as u64).max(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 64 KB RAM at 0xD000_0000 plus one 16-bit register at 0xF031_0120 that records writes.
    struct Bus {
        ram: Vec<u8>,
        reg_writes: Vec<u32>,
        reg_read_val: u32,
    }
    impl DmaBus for Bus {
        fn dma_read(&mut self, addr: u32, bytes: u32) -> Option<u32> {
            if addr == 0xF031_0124 {
                return Some(self.reg_read_val);
            }
            let o = addr.checked_sub(0xD000_0000)? as usize;
            let s = self.ram.get(o..o + bytes as usize)?;
            Some(s.iter().rev().fold(0, |a, &b| a << 8 | b as u32))
        }
        fn dma_write(&mut self, addr: u32, bytes: u32, val: u32) -> bool {
            if addr == 0xF031_0120 {
                self.reg_writes.push(val);
                return true;
            }
            let Some(o) = addr.checked_sub(0xD000_0000) else { return false };
            let Some(s) = self.ram.get_mut(o as usize..(o + bytes) as usize) else { return false };
            s.copy_from_slice(&val.to_le_bytes()[..bytes as usize]);
            true
        }
    }

    #[test]
    fn ssc_tx_and_rx_channels_as_programmed_by_the_firmware() {
        let mut ram = vec![0u8; 0x1_0000];
        for (i, w) in [0x7078u16, 0x8CB4, 0xBFE1].iter().enumerate() {
            ram[0xA54 + 2 * i..0xA56 + 2 * i].copy_from_slice(&w.to_le_bytes());
        }
        let mut bus = Bus { ram, reg_writes: vec![], reg_read_val: 0xA123 };
        let mut d = Dma::default();
        let all = u32::MAX;
        // Init (S50 0x80112138/48/72): ch3 = SSC1 TX, ch1 = SSC1 RX.
        d.write(CH_BASE + 3 * 0x20 + 0x04, 0x0020_4000, all);
        d.write(CH_BASE + 3 * 0x20 + 0x0C, 0x0000_0F08, all);
        d.write(CH_BASE + 3 * 0x20 + 0x14, 0xF031_0120, all);
        d.write(CH_BASE + 0x20 + 0x04, 0x0020_4000, all);
        d.write(CH_BASE + 0x20 + 0x0C, 0x0000_F080, all);
        d.write(CH_BASE + 0x20 + 0x08, 0x0000_0228, all);
        d.write(CH_BASE + 0x20 + 0x10, 0xF031_0124, all);
        // Job start (0x800A9342.. 0x800A8F3E): buffers, TREL 3, HTREQ, STREQ on TX.
        d.write(CH_BASE + 3 * 0x20 + 0x10, 0xD000_0A54, all);
        d.write(CH_BASE + 0x20 + 0x14, 0xD000_0AD0, all);
        d.write(CH_BASE + 3 * 0x20 + 0x04, 0x0020_4003, all);
        d.write(HTREQ, 1 << 3, all);
        d.write(CH_BASE + 0x20 + 0x04, 0x0020_4003, all);
        d.write(HTREQ, 1 << 1, all);
        d.write(STREQ, 1 << 3, all);
        assert_eq!(d.read(TRSR).unwrap(), (1 << 3) | (1 << 17) | (1 << 19));

        let o = d.service(&mut bus);
        assert_eq!((o.transfers, o.src_requests), (1, 0), "one TX move per request (RROAT = 0)");
        assert_eq!(bus.reg_writes, vec![0x7078]);
        assert_eq!(d.read(CH_BASE + 3 * 0x20).unwrap(), 2, "TCOUNT loaded with TREL and decremented");
        // Hardware requests: wrong request line is ignored, PRSEL 4 is taken.
        d.hw_request(3, 5);
        assert!(!d.pending());
        for i in 0..3 {
            d.hw_request(1, 4); // RIR of SSC1
            bus.reg_read_val = 0xA100 + i;
            let o = d.service(&mut bus);
            if i < 2 {
                d.hw_request(3, 4); // TIR of SSC1
                d.service(&mut bus);
                assert_eq!(o.src_requests, 0);
            } else {
                assert_eq!(o.src_requests, 1 << 2, "INTCT=10b, IRDV=0: SRC2 at the end of the transaction");
            }
        }
        assert_eq!(bus.reg_writes, vec![0x7078, 0x8CB4, 0xBFE1], "source incremented by 2, destination fixed");
        assert_eq!(&bus.ram[0xAD0..0xAD6], &[0x00, 0xA1, 0x01, 0xA1, 0x02, 0xA1], "RX: fixed source, destination +2");
        assert_eq!(d.read(INTSR).unwrap(), 1 << 1);
        assert_eq!(d.read(TRSR).unwrap(), 0, "single mode: hardware request enable dropped at TCOUNT = 0");
        assert_eq!(d.read(CH_BASE + 3 * 0x20 + 0x10).unwrap(), 0xD000_0A5A);
        d.write(INTCR, 1 << 1, all);
        assert_eq!(d.read(INTSR).unwrap(), 0);
        assert_eq!(src_addr(2), 0xF000_3EF4);
    }

    #[test]
    fn circular_buffer_wraps_inside_cbl_bits_and_bus_error_latches() {
        // TC1796 UM §12.7.2 p.12-85: CBL = 0 freezes [31:0]; CBL = 4 freezes [31:4] (2^4 = 16 B).
        assert_eq!(step(0xF031_0124, 1, 0, true, 0), 0xF031_0124, "CBL=0 freezes all 32 bits even for 8-bit moves");
        assert_eq!(step(0xD000_0FFE, 2, 0, true, 4), 0xD000_0FF0, "2^4 = 16-byte circular buffer");
        assert_eq!(step(0xD000_0000, 4, 1, false, 15), 0xD000_7FF8, "2^15 = 32 KB circular buffer [0x0000..0x7FFF], decrement by 4 << 1");
        assert_eq!(step(0xD000_8000, 4, 1, false, 15), 0xD000_FFF8, "2^15 = 32 KB circular buffer [0x8000..0xFFFF], bit 15 stays frozen");
        let mut d = Dma::default();
        let mut bus = Bus { ram: vec![0; 16], reg_writes: vec![], reg_read_val: 0 };
        d.write(CH_BASE + 0x04, 0x0040_0001, u32::MAX);
        d.write(CH_BASE + 0x10, 0x1234_0000, u32::MAX);
        d.write(STREQ, 1, u32::MAX);
        assert_eq!(d.service(&mut bus).transfers, 0);
        assert_eq!(d.read(ERRSR).unwrap(), 1);
    }
}
