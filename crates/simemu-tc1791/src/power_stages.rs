//! SIMOS18.1 board wiring of the power-stage drivers ([`crate::atic`]) to the TC1791 serial
//! interfaces, the processor-monitoring disable lines and the loads, plus the read-only output
//! view used as the source for the ECU outputs.
//!
//! Disable lines (FR p.5679 SOPC table; pin evidence = SC800S50 0x800812B0, the ECM3 switch-off
//! path driver, and the levels traced while ECM3 requests the lines in SOPC states 0x80/0x7F):
//!
//! | line    | source        | active  | drives              |
//! |---------|---------------|---------|---------------------|
//! | NDIS0   | MU            | low     | ATIC121 NDIS        |
//! | NDIS1   | MU            | low     | ATIC140 NDIS, ATIC111 EN |
//! | LS_DIS0 | P1.6 (0x16)   | high    | ATIC140 LS_DIS, ATIC121 DIS |
//! | MTC_DIS | P6.7 (0x67, via the ATIC111 driver 0x801F53D4 / table flag) | high | ATIC111 DIS |
//! | EN7_8   | P13.15 (0x20F)| low     | ATIC140 EN7_8 (starter) |
//!
//! Serial links: ATIC121 on SSC0/SLSO0, ATIC111 on SSC1/SLSO3 (SPI handles 0x0080 / 0x2180 of
//! the BSW SPI scheduler, frames traced on the select lines), the MU on SSC1/SLSO2,
//! ATIC140 #0 / #1 on MSC0 / MSC1 (DC writes 0x800ADCF4). The MU is a function block of ATIC139,
//! whose LS/PS blocks answer the 0xAAAA frames on the same select line ([`crate::atic139`]); the
//! engine-off timer PIC is on SSC0/SLSO6 ([`crate::pic_eot`]), the lambda sensor IC ATIC142 on SSC1/SLSO4
//! ([`crate::atic142`]).

use crate::atic::{Atic111, Atic111Pins, Atic121, Atic121Pins, Atic140, Atic140Pins, Load, ATIC140_CHANNELS};
use crate::atic139::Atic139;
use crate::atic142::{Atic142, ATIC142_SLSO, ATIC142_SSC};
use crate::pic_eot::{PicEot, EOT_SLSO, EOT_SSC};
use crate::memory::Tc1791HardwareState;
use crate::monitor_device::{MonitorDevice, MU_SLSO, MU_SSC};
use crate::msc::{Msc, MSC_COUNT, MSC_DMA};
use crate::ssc::SpiSlave;
use serde::Serialize;

pub const ATIC121_SSC: usize = 0;
pub const ATIC121_SLSO: u8 = 0;
pub const ATIC111_SSC: usize = 1;
pub const ATIC111_SLSO: u8 = 3;
/// (port, pin) of the MC switch-off paths.
pub const PIN_LS_DIS0: (usize, u32) = (1, 6);
pub const PIN_MTC_DIS: (usize, u32) = (6, 7);
pub const PIN_EN7_8: (usize, u32) = (13, 15);
/// Disable pins of the three ATIC111 H-bridges, high = disabled (driver config 0x8004F46C {PWM channel, direction
/// pin, disable pin, …} +4, byte +9 = 0 → active high; the enable routine 0x801F54F4 drives it low): bridge 0
/// throttle P6.7 (MTC_DIS), bridge 1 rotary coolant valve N493 P13.9, bridge 2 wastegate actuator V465 P4.9
/// (VERIFIED-S; VERIFIED-T: P6.7 high while P4.9 low and the wastegate PWM running). Which ATIC111 chain stage carries
/// which bridge (and so which stage's SPI status reflects which pin) is UNVERIFIED: the SPI status keeps MTC_DIS.
pub const PIN_BRIDGE_DIS: [(usize, u32); 3] = [(6, 7), (13, 9), (4, 9)];

/// Relays switched by two port pins each (relay driver 0x800AB5D0.., config table 0x80041990: {pin A,
/// pin B, feedback input} per relay; all pins set at init 0x801138A4.., switched off in the after-run
/// 1.9 s (relay 1) / 2.7 s (relay 0) after KL15 off, VERIFIED-T). The coil is energised when both pins are
/// high (driver state 1; A or B alone are its switch tests). Relay 0: P13.1 + P13.0 (feedback P13.3);
/// relay 1: P7.1 + P2.7 (feedback P7.0). FR PWSL: main relay (RLY_MAIN) + one power relay
/// (NC_NR_PWR_RLY = 1) — which pin pair is J271 and which J757 is UNVERIFIED.
pub const RELAY_PINS: [[(usize, u32); 2]; 2] = [[(13, 1), (13, 0)], [(7, 1), (2, 7)]];

/// Levels of the disable lines at the driver pins.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct DisableLines {
    pub ndis0: bool,
    pub ndis1: bool,
    pub ls_dis0: bool,
    pub mtc_dis: bool,
    pub en7_8: bool,
    /// Disable pins of the three H-bridges ([`PIN_BRIDGE_DIS`]).
    pub bridge_dis: [bool; 3],
}

impl DisableLines {
    /// UNVERIFIED for ATIC140 #0: wired like #1 except EN7_8 (tied enabled), the MC only checks #1.
    pub fn atic140(&self, n: usize) -> Atic140Pins {
        Atic140Pins { ndis: self.ndis1, ls_dis: self.ls_dis0, en78: n == 0 || self.en7_8 }
    }
    pub fn atic121(&self) -> Atic121Pins {
        Atic121Pins { ndis: self.ndis0, dis: self.ls_dis0 }
    }
    pub fn atic111(&self) -> Atic111Pins {
        Atic111Pins { en: self.ndis1, dis: self.mtc_dis }
    }
}

/// Driver chips, their links and loads. Chip state survives an MC reset (board supply).
#[derive(Debug, Clone, Default)]
pub struct PowerStages {
    pub atic140: [Atic140; MSC_COUNT],
    pub atic121: Atic121,
    pub atic111: Atic111,
    pub msc: [Msc; MSC_COUNT],
    /// MSC alternate data inputs (GPTA outputs), bit n = data bit n. Supplied by the GPTA model.
    pub msc_altin: [u32; MSC_COUNT],
    /// LS/PS function blocks of ATIC139 (its MU block is `SscDma::mu`).
    pub atic139: Atic139,
    /// Engine-off timer (KL30 supplied).
    pub eot: PicEot,
    /// Lambda sensor interface (wide-band upstream + binary downstream sensor).
    pub atic142: Atic142,
}

impl PowerStages {
    /// SoC reset: the MSC kernels return to reset values; the chips keep their configuration.
    pub fn soc_reset(&mut self) {
        self.msc = Default::default();
    }
}

/// SPI wiring as seen from the SSCs.
pub struct BoardSpiBus<'a> {
    pub mu: &'a mut MonitorDevice,
    pub ps: &'a mut PowerStages,
    pub lines: DisableLines,
}

impl SpiSlave for BoardSpiBus<'_> {
    fn select(&mut self, ssc: usize, lines: u8, _now_ns: u64) {
        match ssc {
            MU_SSC => {
                self.mu.select(lines & (1 << MU_SLSO) != 0);
                self.ps.atic111.select(lines & (1 << ATIC111_SLSO) != 0);
                if lines & (1 << ATIC142_SLSO) != 0 {
                    self.ps.atic142.select();
                }
            }
            ATIC121_SSC => self.ps.atic121.select(),
            _ => {}
        }
    }

    fn shift(&mut self, ssc: usize, lines: u8, mtsr: u32, bits: u32, now_ns: u64) -> u32 {
        let sel = |n: u8| lines & (1 << n) != 0;
        match (ssc, bits) {
            (MU_SSC, 16) if sel(MU_SLSO) && Atic139::claims(mtsr as u16) => self.ps.atic139.exchange(mtsr as u16) as u32,
            (MU_SSC, 16) if sel(MU_SLSO) => self.mu.exchange(mtsr as u16, now_ns) as u32,
            (ATIC142_SSC, 16) if sel(ATIC142_SLSO) => self.ps.atic142.exchange(mtsr as u16) as u32,
            (ATIC111_SSC, 16) if sel(ATIC111_SLSO) => self.ps.atic111.exchange(mtsr as u16, self.lines.atic111()) as u32,
            (ATIC121_SSC, 16) if sel(ATIC121_SLSO) => self.ps.atic121.exchange(mtsr as u16, self.lines.atic121()) as u32,
            (EOT_SSC, 8) if sel(EOT_SLSO) => self.ps.eot.exchange(mtsr as u8, now_ns) as u32,
            _ if lines == 0 => (1u64 << bits).wrapping_sub(1) as u32, // nobody drives MRST: pull-up
            _ => 0,                                                    // device not modelled
        }
    }
}

impl Tc1791HardwareState {
    #[inline]
    fn pin(&self, (port, pin): (usize, u32)) -> bool {
        if port >= 18 || pin >= 16 {
            return false;
        }
        let base = if port < 12 { 0xF000_0C00 + 0x100 * port as u32 } else { 0xF030_0000 + 0x100 * (port as u32 - 12) };
        let iocr = self.seg_f000.get(base + 0x10 + (pin / 4) * 4);
        let pc = (iocr >> ((pin % 4) * 8 + 4)) & 0xF;
        if pc & 0x8 != 0 {
            (self.seg_f000.get(base) >> pin) & 1 != 0
        } else if self.scu.port_ext_driven[port] & (1 << pin) != 0 {
            (self.scu.port_ext_level[port] >> pin) & 1 != 0
        } else {
            pc & 0x3 != 0x1
        }
    }

    /// Port pin configured as general-purpose output (IOCR PC = 0b1x00) with its output latch high.
    fn pin_driven_high(&self, (port, pin): (usize, u32)) -> bool {
        if port >= 18 || pin >= 16 {
            return false;
        }
        let base = if port < 12 { 0xF000_0C00 + 0x100 * port as u32 } else { 0xF030_0000 + 0x100 * (port as u32 - 12) };
        let pc = (self.seg_f000.get(base + 0x10 + (pin / 4) * 4) >> ((pin % 4) * 8 + 4)) & 0xF;
        pc & 0b1011 == 0b1000 && self.seg_f000.get(base) >> pin & 1 != 0
    }

    pub fn disable_lines(&self) -> DisableLines {
        let mu = &self.spi.mu;
        DisableLines {
            ndis0: mu.ndis0(),
            ndis1: mu.ndis1(),
            ls_dis0: self.pin(PIN_LS_DIS0),
            mtc_dis: self.pin(PIN_MTC_DIS),
            en7_8: self.pin(PIN_EN7_8),
            bridge_dis: PIN_BRIDGE_DIS.map(|p| self.pin(p)),
        }
    }

    pub(crate) fn msc_mmio_read(&self, addr: u32) -> Option<u32> {
        let (n, off) = crate::msc::decode(addr)?;
        self.spi.ps.msc[n].read(off)
    }

    pub(crate) fn msc_mmio_write(&mut self, addr: u32, val: u32, mask: u32) -> bool {
        let Some((n, off)) = crate::msc::decode(addr) else { return false };
        if off == crate::msc::CLC && self.scu.endinit_blocks(addr) {
            return true;
        }
        let (now, fpi) = (self.now_ns(), self.ffpi_hz());
        self.spi.ps.msc[n].write(off, val, mask, now, fpi)
    }

    /// Deliver finished MSC command frames to the ATIC140s and their due upstream frames to UD0
    /// (each one served by the DMA before the next arrives).
    pub(crate) fn msc_update(&mut self, now: u64) {
        for n in 0..MSC_COUNT {
            let ps = &mut self.spi.ps;
            ps.atic140[n].data = ps.msc[n].data_bits(ps.msc_altin[n]);
            ps.atic140[n].bridged = crate::atic::ATIC140_BRIDGED[n]; // board wiring
        }
        if self.msc_next_event_ns().is_none_or(|t| t > now) {
            return;
        }
        let fpi = self.ffpi_hz();
        let lines = self.disable_lines();
        for (n, &(ch, line)) in MSC_DMA.iter().enumerate() {
            let ps = &mut self.spi.ps;
            while let Some((w, t)) = ps.msc[n].take_command(now) {
                let reply = ps.atic140[n].command(w, lines.atic140(n));
                ps.msc[n].queue_upstream(&reply, t, fpi);
            }
            while let Some(req) = self.spi.ps.msc[n].receive_next(now) {
                if req {
                    self.spi.dma.hw_request(ch, line);
                    self.dma_run();
                }
            }
        }
    }

    #[inline]
    pub(crate) fn msc_next_event_ns(&self) -> Option<u64> {
        match (self.spi.ps.msc[0].next_event_ns(), self.spi.ps.msc[1].next_event_ns()) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        }
    }
}

/// One ATIC140 output stage.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct StageOut {
    /// Commanded on by the MSC data frame.
    pub commanded: bool,
    /// Conducting: commanded and not disabled by a hardware line.
    pub on: bool,
}

/// One ATIC111 H-bridge channel.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct BridgeOut {
    /// EN (MU NDIS1) high and the bridge's own disable pin ([`PIN_BRIDGE_DIS`]) low.
    pub enabled: bool,
    /// Mode bits 3:2 of the SPI command.
    pub mode: u8,
    pub command: u16,
}

/// Read-only view of the power-stage outputs (source for `EcuOutputs`, together with the GPTA
/// timing outputs: ignition dwell / injector on-time come from GPTA, the drivers gate them).
#[derive(Debug, Clone, Serialize)]
pub struct PowerStageOutputs {
    pub lines: DisableLines,
    /// ATIC140 #0 (MSC0) / #1 (MSC1), 26 stages each.
    pub atic140: [[StageOut; ATIC140_CHANNELS]; MSC_COUNT],
    /// Injector driver enabled by NDIS0 / LS_DIS0.
    pub injectors_enabled: bool,
    /// ATIC121 configuration registers as written over SPI (current profile, timing).
    pub atic121_regs: [u16; 32],
    pub bridges: [BridgeOut; 3],
    /// Port-driven relays energised ([`RELAY_PINS`]).
    pub relays: [bool; 2],
}

impl crate::Simos18Board {
    pub fn power_stage_outputs(&self) -> PowerStageOutputs {
        let hw = self.hw();
        let ps = &hw.spi.ps;
        let lines = hw.disable_lines();
        let atic140 = std::array::from_fn(|n| {
            let (c, pins) = (&ps.atic140[n], lines.atic140(n));
            std::array::from_fn(|k| StageOut { commanded: c.data >> k & 1 != 0, on: c.output_on(k, pins) })
        });
        PowerStageOutputs {
            lines,
            atic140,
            injectors_enabled: ps.atic121.enabled(lines.atic121()),
            atic121_regs: ps.atic121.regs,
            bridges: std::array::from_fn(|k| BridgeOut {
                enabled: ps.atic111.enabled(Atic111Pins { en: lines.ndis1, dis: lines.bridge_dis[k] }),
                mode: ps.atic111.mode(k),
                command: ps.atic111.cmd[k],
            }),
            relays: RELAY_PINS.map(|pins| pins.iter().all(|&p| hw.pin_driven_high(p))),
        }
    }

    /// Port pin driven high by its general-purpose output (false when not configured as output).
    pub fn port_output(&self, port: usize, pin: u32) -> bool {
        self.hw().pin_driven_high((port, pin))
    }

    /// Driver chips and loads (vehicle model: wire faults, load parameters; MSC ALTIN inputs).
    pub fn power_stages_mut(&mut self) -> &mut PowerStages {
        &mut self.hw_mut().spi.ps
    }

    /// Replace one load. `chip`: 0/1 = ATIC140 #0/#1, 2 = ATIC121, 3 = ATIC111.
    pub fn set_power_stage_load(&mut self, chip: usize, channel: usize, load: Load) -> Result<(), String> {
        let ps = self.power_stages_mut();
        let slot = match chip {
            0 | 1 => ps.atic140[chip].loads.get_mut(channel),
            2 => ps.atic121.loads.get_mut(channel),
            3 => ps.atic111.loads.get_mut(channel),
            _ => None,
        };
        *slot.ok_or_else(|| format!("no power stage {chip}/{channel}"))? = load;
        Ok(())
    }
}
