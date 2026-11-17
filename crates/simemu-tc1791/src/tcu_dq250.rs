//! Temic DQ250 (02E / 0D9, wet 6-speed DCT) mechatronic: Infineon TC1766 board.
//!
//! MCU identification (ASW `0D9300011R` / `4319` and `0D9300012F` / `4930`, HW `02E927770AQ`):
//! * PFlash image is 0x178000 bytes (1504 KB, the TC1766 PFlash size); erased flash reads 0x00
//!   (AUDO-NG ECC flash). A 0x180000-byte read (1.5 MB window) is accepted when its last 0x8000
//!   bytes are zero.
//! * The firmware's RAM test (0x800717B0) covers LDRAM 0xD0000000..0xD000DFFF (56 KB), SPRAM
//!   0xD4000000..0xD4003FFF (16 KB), PCP PRAM 0xF0050000..0xF0051FFF and PCP CMEM
//!   0xF0060000..0xF0062FFF: the TC1766 memory set (TC1796 has 64 KB LDRAM / 48 KB SPRAM).
//! * AUDO-NG peripheral map: SCU/WDT at 0xF0000000 (WDT_CON0/1 at +0x20/+0x24, PLL_CLC +0x40),
//!   STM 0xF0000200, ASC0 0xF0000A00, ports P0..P5 0xF0000C00.., GPTA0 0xF0001800, DMA
//!   0xF0003C00, MultiCAN 0xF0004000 with 2 nodes and message objects at +0x400 (0xF00044E0 = MO7),
//!   PCP 0xF0043F00, SSC0/1 0xF0100100/0xF0100200, FADC 0xF0100300, ADC0 0xF0100400 (AUDO-NG ADC
//!   with CHCON at +0x10..+0x4C), FLASH0 regs 0xF8002000 (MARP/MARD written at boot).
//! * Chip ID check at 0x80098F36: `(SCU_CHIPID & 0xFF00) == 0x8B00`.
//!
//! Firmware layout: SBOOT 0x80000000..0x80007FFF (trap table at 0, start 0x80004000, boot ID
//! "02E927770X 0909", "DQ250_MQB"), programming records 0x8000C000/0x8000C100, CBOOT
//! 0x80010000..0x80017FFF (start 0x80010200, "02E927770AN", "EV_TCMDQ250021"), calibration
//! 0x80030000..0x8003FFFF ("0D9300012F", "DQ250-6A", "49300D9300012F_4930_ODKW"), ASW
//! 0x80040000..0x800B3FFF (`ja 0x80040900`), ASW constants 0x800C0000..0x800C7FFF.
//!
//! Boot chain: SBOOT reads the EEPROM and enters CBOOT. CBOOT (main 0x80015138) starts the ASW
//! (`ji [0xA001BFD0]` = 0xA0040000, 0x8001511C) only if the EEPROM status record says the logical
//! blocks are valid, or else after a CRC-32 pass (0x8001113C -> 0x80016370: reflected 0xEDB88320,
//! init 0xFFFFFFFF, block valid when the register ends at 0) over ASW 0xA0040000+0x130000 and
//! CAL 0xA0020000+0x20000 (block table 0x800101C8; the CRC word is the last word of each block,
//! 0x8003FFFC / 0x8016FFFC, see `simemu_damos::apply_post_programming_dq250`). CBOOT itself serves UDS on CAN node 1 (0x7E1 -> 0x7E9) and polls the
//! STM (CMP0IR) from a loop copied to SPRAM.
//!
//! Board devices modelled here: SCU (reset, WDT via the shared [`crate::scu::ScuWdt`], PLL),
//! SSC0/SSC1 synchronous serial channels, the SPI EEPROM on SSC0 (chip select P2.8, 25xxx command
//! set READ/WRITE/RDSR/WREN, 32-byte pages, as driven by SBOOT 0x80003BD8..0x80003D02), ASC0,
//! ADC0 ([`crate::tc1766_adc`], board inputs [`BOARD_AIN`], external multiplexer on AIN31 [`BOARD_EMUX`]:
//! supply input [`Dq250TcuBoard::set_kl15_v`]) with SCU_CON.AN7TM, and the FADC (TC1796 kernel at 0xF0100300,
//! channel 0 on the AIN7 reference node). Shared models: STM, MultiCAN (AUDO-NG global register layout via
//! `can_tc1791_addr`), GPTA0/LTCA2 and PCP2 ([`crate::engine_io`], TC1796 UM register map; GPTA0 with the
//! TC1766's 52-word multiplexer array, [`crate::gpta::Gpta::tc1766`]). The mechatronic around the MCU
//! ([`crate::dq250_io`], [`Dq250TcuBoard::set_inputs`] / [`Dq250TcuBoard::decode_outputs`]): speed sensors as pin
//! waves on the GPTA input lines (`PinGen`), travel / clutch-pressure / oil-temperature sensors on ADC0 (sensor
//! supply on the alternate references), the watchdog companion chip on SSC1 / CS P2.1 and the SPI temperature
//! sensor on SSC0 / CS P2.9, the master supply and three high sides, and eleven valve coils (R-L, integrated per PWM
//! edge) with their current senses. Everything else in the peripheral space is a plain register file; not emulated:
//! DMA; the P5.10..14 strap (read 0, selects the pressure-sensor characteristic) is left open (UNVERIFIED).
//!
//! CPU core: TriCore 1.3 ([`tricore_core::Arch::Tc13`]; Unicorn: `isa_fix` `SvlcxTc13` and the BISR
//! fix): SVLCX/BISR keep PCXI.PCPN/PIE. The ASW OS (vector 1 = `bisr #0xE` + dispatcher 0x80041890,
//! task start by `svlcx`/`rfe` at 0x80041A08..0x80041A72) only runs its tasks at CCPN 0 that way; with
//! the TC1.3.1 RTL every task, the idle loop 0x800A8D54 included, runs at CCPN 0xE and ActivateTask
//! 0x8004E2B2 waits forever for its SRPN-1 dispatch request (MILESTONE.md §2.3).

use crate::memory::{BoardVariant, CanFrame, Tc1791HardwareState};
use crate::scu::ResetKind;
use crate::stm_icu::{CYCLES_PER_INSN, SRC_SRR, STM_BASE};
use crate::tc1766_adc::Adc0;
use crate::tc1784_io::fadc::Fadc;
use crate::{RunStats, Simos18Board};
use std::collections::BTreeMap;

pub const PFLASH_SIZE: usize = 0x17_8000;
/// Size of a full read by tools that dump a 1.5 MB window (zero padding past [`PFLASH_SIZE`]).
pub const PFLASH_READ_SIZE: usize = 0x18_0000;
/// DFlash (2 x 16 KB). Not referenced by this firmware; mapped erased. Base UNVERIFIED (AUDO-NG).
pub const DFLASH_BASE: u32 = 0xAFE0_0000;
pub const DFLASH_SIZE: usize = 0x8000;
/// LDRAM is 56 KB on the chip; Unicorn maps TriCore memory in 16 KB pages, so 0xD000E000..0xD000FFFF
/// (unimplemented on hardware, bus error) is backed by RAM here. ponytail: no bus error for that 8 KB.
pub const LDRAM_SIZE: usize = 0x1_0000;
pub const SPRAM_SIZE: usize = 0x4000;

pub const RST_REQ: u32 = 0xF000_0010;
pub const RST_SR: u32 = 0xF000_0014;
pub const WDT_CON0: u32 = 0xF000_0020;
pub const WDT_CON1: u32 = 0xF000_0024;
pub const WDT_SR: u32 = 0xF000_0028;
pub const PLL_CLC: u32 = 0xF000_0040;
pub const SCU_CHIPID: u32 = 0xF000_0074;
pub const SCU_CON: u32 = 0xF000_0050;
/// Port 0 input register.
pub const P0_IN: u32 = 0xF000_0C24;
/// CHID = 0x8B as required by the firmware's chip check (0x80098F36); CHREV UNVERIFIED.
pub const CHIPID_VALUE: u32 = 0x0000_8B00;
/// PLL_CLC after reset: VCO bypass (UNVERIFIED reset value).
pub const PLL_CLC_RESET: u32 = 0x0000_0020;

pub const SSC_BASES: [u32; 2] = [0xF010_0100, 0xF010_0200];
pub const FADC_BASE: u32 = 0xF010_0300;
pub const ASC0_BASE: u32 = 0xF000_0A00;

/// Chip select of the SPI EEPROM: P2.8, active low (SBOOT SSC table at 0x800001C4).
pub const EEPROM_CS: (usize, u32) = (2, 8);
/// SPI EEPROM size used when no image is supplied (no DQ250 EEPROM dump is available; the
/// firmware uses 16-bit addresses and 32-byte pages, the capacity is UNVERIFIED).
pub const EEPROM_DEFAULT_SIZE: usize = 0x2000;

/// Board voltages on ADC0 AIN0..31 (AIN16..31: multiplexer group 1). No DQ250 schematic is available: each value is
/// the nominal code of the window in which the firmware's ADC plausibility check (0x80095860, limits in
/// CAL 0x8003424C) accepts the channel, converted at VAREF 5 V. UNVERIFIED identities:
/// AIN0/AIN1 = 0xF3C ± 0xC8 (4.76 V, HYP: sensor supply rails, either one suffices), AIN2 = 0 ± 0xF
/// (HYP: analog ground reference), AIN7 = window [0x4D8, 0xBB7] without and <= 0x12E with the
/// SCU_CON.AN7TM pull-down (0x80095860..0x800959C6): driven at VAREF/2 (HYP: ratiometric divider).
/// AIN25 = entry 9 of the ASW's board-voltage table (0D9300012F: channel list 0xD0005FA0, 0x19 with gain 1.0, mV at
/// 0xD0003B2A, 0x80067048): events 0x167 / 0x165 (P0602 entries of the event table) report it below 1000 / 3100 mV
/// (0x800A4C04, CAL 0x80030042 / 0x80030040; VERIFIED-S) — they fired permanently with the line at 0 V, coding did not
/// change that. Driven at 3.2 V (UNVERIFIED identity, HYP: a 3.3 V rail monitor).
/// Other inputs are not driven (0 V), among them the other group-1 lines (AIN16..31, CHCONn.GRPS = 1 in the
/// driver's second phase, 0x8007EEE0 with d4 = 1) not set by the mechatronic model.
pub const BOARD_AIN: [f32; 32] = {
    let mut v = [0.0f32; 32];
    v[0] = 0xF3C as f32 / 4095.0 * crate::tc1766_adc::VAREF;
    v[1] = 0xF3C as f32 / 4095.0 * crate::tc1766_adc::VAREF;
    v[7] = REF_NODE_V;
    v[25] = 3.2;
    v
};

/// Battery voltage of the bench supply (V) used for the board's KL30 inputs until a plant drives them.
pub const KL30_DEFAULT_V: f32 = 13.8;

/// Divider of the board's battery-level inputs: the ASW converts AIN31/EMUX 0, AIN4, AIN6, AIN15 and AIN23 with
/// gain 3072/512 = 6.0 (0x8007142C, gain table 0xD0006140; VERIFIED-S); the circuit itself is UNVERIFIED.
pub const KL30_DIVIDER: f32 = 6.0;

/// AIN31 sits behind an external multiplexer (CHCON15 of the group-1 phase: EMUX 0 = 0x02804400 or EMUX 3 =
/// 0x02807400, toggled by flag 0xD0004102 at 0x80067FE0..0x800680B0; VERIFIED-S): EMUX 0 -> analog index 0x21
/// (converted 6:1 as a battery voltage; checked ≥ 0x745 = 9.0 V before the AIN7/FADC cross-check at
/// 0x800958AE and ≥ 0x6B4 = 8.3 V in the start-up power-stage test 0x8009D66E), EMUX 3 -> index 0x20
/// (converted at 0x8009E74C as T = 122.85·V − 232.85, HYP: board temperature in °C). EMUX 1/2 are not used.
pub const BOARD_EMUX: [Option<[f32; 4]>; 32] = {
    let mut e = [None; 32];
    e[31] = Some([KL30_DEFAULT_V / KL30_DIVIDER, 0.0, 0.0, BOARD_TEMP_V]);
    e
};

/// Board temperature input (AIN31/EMUX 3) at 25 °C by the firmware's conversion 0x8009E74C (UNVERIFIED sensor).
pub const BOARD_TEMP_V: f32 = (25.0 + 232.85) / 122.85;

/// Reference node on AIN7 and FADC FAIN0P (UNVERIFIED identity). With battery present the ASW requires AIN7 in
/// CAL 0x8003424C [0xAA2 ± 0x115] and the FADC channel-0 result (`RCH0 << 2`) within 0x200 of it
/// (0x800958C2..0x8009594C): the same node seen by both converters (HYP). Driven at the window centre.
pub const REF_NODE_V: f32 = 0xAA2 as f32 / 4095.0 * crate::tc1766_adc::VAREF;

/// FADC pins [FAINxP, FAINxN] per channel: channel 0 = the AIN7 reference node, the rest not driven.
pub const BOARD_FAIN: [[f32; 2]; 4] = [[REF_NODE_V, 0.0], [0.0; 2], [0.0; 2], [0.0; 2]];

/// Module clock control registers (ENDINIT-protected; DISS mirrors DISR).
const CLC_REGS: &[u32] = &[
    0xF000_0800, 0xF000_0A00, 0xF000_0B00, 0xF000_1800, 0xF000_3C00, 0xF000_4000, 0xF004_3F00,
    0xF010_0100, 0xF010_0200, 0xF010_0400, 0xF010_0600,
];

/// Service request control registers of TC1766 modules not in the shared SRC table
/// (ASC0, SSC0, SSC1, ADC0), proven by the firmware's CLRR/SRE writes.
fn is_tc1766_src(a: u32) -> bool {
    matches!(a, 0xF000_0AF0..=0xF000_0AFC | 0xF010_01F4..=0xF010_01FC | 0xF010_02F4..=0xF010_02FC | 0xF010_03F0..=0xF010_03FC | 0xF010_05F0..=0xF010_05FC)
        && a & 3 == 0
}

/// MultiCAN global registers of the AUDO-NG layout (TC1796 UM p.18-66: LIST0..7 at +0x100, MSPND0..7 at
/// +0x120, MSID0..7 at +0x140, MSIMASK/PANCTR/MCR/MITR at +0x160..+0x16C) mapped onto the shared model's
/// TC1791 layout (MSPND +0x140, MSID +0x180, MSIMASK/PANCTR/MCR/MITR +0x1C0..+0x1CC).
fn can_tc1791_addr(a: u32) -> u32 {
    match a.wrapping_sub(0xF000_4000) {
        0x120..=0x13C => a + 0x20,
        0x140..=0x15C => a + 0x40,
        0x160..=0x16C => a + 0x60,
        _ => a,
    }
}

/// 25xxx-family SPI EEPROM.
#[derive(Debug, Clone)]
pub struct SpiEeprom {
    pub mem: Vec<u8>,
    state: EepState,
    addr: usize,
    wel: bool,
    data_bytes: u32,
    pub read_bytes: u64,
    pub written_bytes: u64,
    pub commands: BTreeMap<u8, u64>,
    /// READ/WRITE accesses: (command, start address, data bytes), bounded.
    pub log: Vec<(u8, u16, u32)>,
    start: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EepState {
    Idle,
    Ignore,
    Status,
    WriteStatus,
    Addr { cmd: u8, n: u8 },
    Read,
    Write,
}

impl SpiEeprom {
    pub const PAGE: usize = 32;

    pub fn new(image: &[u8]) -> Self {
        let mem = if image.is_empty() { vec![0xFF; EEPROM_DEFAULT_SIZE] } else { image.to_vec() };
        Self {
            mem,
            state: EepState::Idle,
            addr: 0,
            wel: false,
            data_bytes: 0,
            read_bytes: 0,
            written_bytes: 0,
            commands: BTreeMap::new(),
            log: Vec::new(),
            start: 0,
        }
    }

    /// Chip select released: ends the command; a completed WRITE/WRSR clears WEL. The internal
    /// write cycle completes immediately (tWC not modelled: RDSR.WIP always reads 0).
    pub fn deselect(&mut self) {
        if matches!(self.state, EepState::Write | EepState::WriteStatus) && self.data_bytes > 0 {
            self.wel = false;
        }
        if matches!(self.state, EepState::Read | EepState::Write) && self.log.len() < 4096 {
            let cmd = if self.state == EepState::Read { 0x03 } else { 0x02 };
            self.log.push((cmd, self.start as u16, self.data_bytes));
        }
        self.state = EepState::Idle;
        self.data_bytes = 0;
    }

    /// One byte shifted while selected; returns the byte driven on MISO.
    pub fn shift(&mut self, tx: u8) -> u8 {
        let len = self.mem.len().max(1);
        match self.state {
            EepState::Idle => {
                *self.commands.entry(tx).or_insert(0) += 1;
                self.state = match tx {
                    0x06 => {
                        self.wel = true;
                        EepState::Ignore
                    }
                    0x04 => {
                        self.wel = false;
                        EepState::Ignore
                    }
                    0x05 => EepState::Status,
                    0x01 => EepState::WriteStatus,
                    0x03 | 0x02 => {
                        self.addr = 0;
                        EepState::Addr { cmd: tx, n: 0 }
                    }
                    _ => EepState::Ignore,
                };
                0xFF
            }
            EepState::Ignore => 0xFF,
            EepState::Status => (self.wel as u8) << 1,
            EepState::WriteStatus => {
                self.data_bytes += 1;
                0xFF
            }
            EepState::Addr { cmd, n } => {
                self.addr = (self.addr << 8) | tx as usize;
                self.state = if n == 0 { EepState::Addr { cmd, n: 1 } } else if cmd == 0x03 { EepState::Read } else { EepState::Write };
                self.start = self.addr;
                0xFF
            }
            EepState::Read => {
                let b = self.mem[self.addr % len];
                self.addr = self.addr.wrapping_add(1);
                self.read_bytes += 1;
                self.data_bytes += 1;
                b
            }
            EepState::Write => {
                if self.wel {
                    let page = self.addr & !(Self::PAGE - 1);
                    let a = (page | ((self.addr + self.data_bytes as usize) & (Self::PAGE - 1))) % len;
                    self.mem[a] = tx;
                    self.written_bytes += 1;
                }
                self.data_bytes += 1;
                0xFF
            }
        }
    }
}

/// One SSC transfer (diagnostics: identifies SPI devices by chip-select snapshot).
#[derive(Debug, Clone, Copy)]
pub struct SpiXfer {
    pub ssc: u8,
    pub bits: u8,
    pub tx: u32,
    pub rx: u32,
    /// P0..P5 output latches at the time of the transfer.
    pub port_out: [u16; 6],
    pub insns: u64,
}

/// A toothed wheel passing a Hall speed sensor on a GPTA input line: the line follows the wheel's angle (high over
/// the second half of each tooth pitch), so it toggles exactly when a tooth edge passes the sensor. A rotation gives
/// the pulse train (one 50 % pulse per tooth; the falling edge is the one the PCP captures), a standing wheel holds
/// the level, and a small oscillation inside one pitch (driveline lash in P) gives no edges at all — as at a real
/// sensor. A reversal re-crosses the last edge. ponytail: no magnetic hysteresis of the sensor (an oscillation
/// straddling an edge toggles it), add it with its datasheet value when one is known.
#[derive(Clone, Copy, Debug)]
struct PinGen {
    line: u8,
    /// Wheel angle in half tooth pitches at `t_ns` (edges are queued up to `t_ns`).
    pos: f64,
    t_ns: f64,
    /// Angular speed in half tooth pitches per ns (signed).
    rate: f64,
    level: bool,
}

impl PinGen {
    /// Line level at the current angle.
    fn level_at(pos: f64) -> bool {
        pos.floor().rem_euclid(2.0) == 1.0
    }

    /// Turns the wheel up to `horizon` (ns), reporting each level change (time ≥ `now`, level).
    fn advance(&mut self, now: f64, horizon: f64, edge: &mut dyn FnMut(f64, bool)) {
        if !(now.is_finite() && horizon.is_finite()) {
            return;
        }
        let mut push = |g: &mut Self, t: f64| {
            let level = Self::level_at(g.pos);
            if level != g.level {
                g.level = level;
                edge(t.max(now), level);
            }
        };
        if !self.pos.is_finite() {
            self.pos = 0.0;
        }
        push(self, self.t_ns);
        let mut count = 0usize;
        while self.t_ns < horizon && count < 16_384 {
            if !self.rate.is_finite() || self.rate == 0.0 {
                self.t_ns = horizon;
                break;
            }
            // Keep `pos` modulo 2.0 so `|b| <= 2.0` always and `+ 1e-9` never underflows ULP.
            self.pos = self.pos.rem_euclid(2.0);
            let b = if self.rate > 0.0 { self.pos.floor() + 1.0 } else { self.pos.ceil() - 1.0 };
            let tb = self.t_ns + (b - self.pos) / self.rate;
            if !tb.is_finite() || tb > horizon {
                self.pos = (self.pos + self.rate * (horizon - self.t_ns)).rem_euclid(2.0);
                self.t_ns = horizon;
                break;
            }
            let next_t = if tb > self.t_ns { tb } else { self.t_ns + (self.t_ns.abs() * f64::EPSILON).max(1.0) };
            self.pos = (b + 1e-9 * self.rate.signum()).rem_euclid(2.0);
            self.t_ns = next_t.min(horizon);
            push(self, self.t_ns);
            count += 1;
        }
        if self.t_ns < horizon {
            self.pos = (self.pos + self.rate * (horizon - self.t_ns)).rem_euclid(2.0);
            self.t_ns = horizon;
        }
    }
}

/// Edges of the pin generators are queued into the GPTA this far ahead of the board time (the quantum ends at the
/// first queued edge, so the horizon only bounds how late a new speed takes effect).
const PIN_LOOKAHEAD_NS: f64 = 200_000.0;

/// TC1766-specific peripheral state of the DQ250 board.
pub struct Tc1766Periph {
    /// Speed-sensor waves on GPTA input lines ([`Dq250TcuBoard::set_pin_wave`]).
    pins: Vec<PinGen>,
    pub pll_clc: u32,
    pub rst_req: u32,
    pub ssc_rb: [u32; 2],
    pub eeprom: SpiEeprom,
    eeprom_selected: bool,
    pub spi_log: Vec<SpiXfer>,
    pub asc0_tx: Vec<u8>,
    /// ADC0 ([`crate::tc1766_adc`]); `adc0.volts` are the board's analog inputs.
    pub adc0: Adc0,
    /// FADC at 0xF0100300 (TC1796 kernel and layout, [`crate::tc1784_io::fadc`]): the ASW programs FDR
    /// 0x826D, GCR 0x00200003 (ANON), CFGR0 0x00016440 (channel 0 timer, CTF 1024, CTREL 1, always
    /// gated), CFGR1 0x00016040, ACR0/1 = 5 (FAINxP single ended, gain 2) and polls RCH0 (0xF0100340,
    /// 0x8007EF2C: `(RCH0 & 0x3FF) << 2` into the analog table, index 0x22); no FADC interrupt is enabled
    /// (VERIFIED-T register trace of 4319).
    pub fadc: Fadc,
    /// Reads/writes of registers without a model (plain register file), per address.
    pub plain_reads: BTreeMap<u32, u64>,
    pub plain_writes: BTreeMap<u32, u64>,
    /// Last value written to each plain register (configuration snapshot for the IO map).
    pub last_written: BTreeMap<u32, u32>,
    pub sw_reset_requests: Vec<u32>,
    /// Diagnostic register-access trace: accesses inside `trace_range` are logged as
    /// (instruction count, block pc, address, value, lane mask; mask 0 = read), bounded.
    pub trace_range: Option<(u32, u32)>,
    pub trace_reads: bool,
    pub trace_log: Vec<(u64, u32, u32, u32, u32)>,
    /// Watchdog companion chip on SSC1 / CS P2.1 ([`crate::dq250_io::QaWatchdog`]).
    pub wd: crate::dq250_io::QaWatchdog,
    /// Temperature at the SPI temperature sensor on SSC0 / CS P2.9 (°C, [`crate::dq250_io::lm74_word`]).
    pub board_temp_c: f64,
    /// Valve coils behind the high sides ([`crate::dq250_io::VALVES`] order).
    pub valves: crate::dq250_io::ValveStage,
}

const SPI_LOG_MAX: usize = 1 << 16;

impl Tc1766Periph {
    pub fn new(eeprom: &[u8]) -> Self {
        Self {
            pins: Vec::new(),
            pll_clc: PLL_CLC_RESET,
            rst_req: 0,
            ssc_rb: [0; 2],
            eeprom: SpiEeprom::new(eeprom),
            eeprom_selected: false,
            spi_log: Vec::new(),
            asc0_tx: Vec::new(),
            adc0: {
                let mut a = Adc0::with_inputs(BOARD_AIN, BOARD_EMUX);
                a.vref = [crate::tc1766_adc::VAREF, crate::dq250_io::SENSOR_REF_V, crate::dq250_io::SENSOR_REF_V, crate::tc1766_adc::VAREF];
                a
            },
            fadc: Fadc::with_inputs(BOARD_FAIN),
            plain_reads: BTreeMap::new(),
            plain_writes: BTreeMap::new(),
            last_written: BTreeMap::new(),
            sw_reset_requests: Vec::new(),
            trace_range: None,
            trace_reads: false,
            trace_log: Vec::new(),
            wd: Default::default(),
            board_temp_c: 25.0,
            valves: Default::default(),
        }
    }

    /// Module reset (EEPROM content is non-volatile).
    pub fn reset(&mut self) {
        self.pll_clc = PLL_CLC_RESET;
        self.rst_req = 0;
        self.ssc_rb = [0; 2];
        self.eeprom.deselect();
        self.eeprom_selected = false;
        // inputs are driven from outside the MCU
        let vref = self.adc0.vref;
        self.adc0 = Adc0::with_inputs(self.adc0.volts, self.adc0.emux);
        self.adc0.vref = vref;
        self.fadc = Fadc::with_inputs(self.fadc.volts);
        // the watchdog chip keeps running across an MCU reset (its own supply); HYP
        for c in self.valves.coils.iter_mut() {
            c.cell_level = false;
        }
    }
}

/// PLL_CLC fields (TC1796-family layout, UNVERIFIED for TC1766): LOCK[0], RESLD[1], VCOBYP[5],
/// KDIV[11:8], PDIV[15:13], NDIV[22:16], OSCDISC[24]. Returns the equivalent TC1791
/// PLLCON0/PLLCON1 encoding consumed by [`crate::scu::ScuWdt::fcpu_hz`] (fCPU = fOSC*N/(P*K),
/// bypass: fOSC/K).
fn pll_to_scu(v: u32) -> (u32, u32) {
    let n = (v >> 16) & 0x7F;
    let p = (v >> 13) & 0x7;
    let k = (v >> 8) & 0xF;
    (((v >> 5) & 1) | (n << 9) | (p << 24), k | (k << 16))
}

impl Tc1791HardwareState {
    fn p(&mut self) -> &mut Tc1766Periph {
        self.dq250.as_mut().expect("DQ250 board state")
    }

    fn port_out(&self, port: usize) -> u16 {
        self.seg_f000.get(0xF000_0C00 + 0x100 * port as u32) as u16
    }

    fn plain_store(&mut self, a: u32, v: u32, m: u32) {
        let old = self.seg_f000.get(a);
        self.seg_f000.set(a, (old & !m) | (v & m));
    }

    fn f_sys(&self) -> u64 {
        self.scu.fcpu_hz() / (self.scu.fpidiv() as u64 + 1)
    }

    /// Quantum boundary: ADC0 timer/conversions up to now, its service requests to the SRCs.
    pub(crate) fn dq250_update(&mut self) {
        if self.dq250.is_none() {
            return;
        }
        self.dq250_valves();
        let (now, f) = (self.now_ns(), self.f_sys());
        let an7 = self.seg_f000.get(SCU_CON) & 1 << 6 != 0; // SCU_CON.AN7TM
        self.p().adc0.an7_test = an7;
        for sr in self.p().adc0.update(now, f) {
            if let Some(src) = crate::tc1766_adc::sr_src(sr) {
                self.src_set_srr(src);
            }
        }
        self.dq250_fadc_advance();
        self.dq250_pins(now as f64);
    }

    /// Level of a port output latch bit.
    fn latch(&self, (port, pin): (usize, u32)) -> bool {
        self.port_out(port) & (1 << pin) != 0
    }

    /// Valve power stages up to now: switch states from the GPTA cells / port latches, high sides and master supply;
    /// every coil integrated to now; the current senses and the switched-supply senses onto the ADC inputs. Called at
    /// quantum boundaries, ADC accesses and port writes; PWM edges in between come through
    /// [`Self::dq250_valve_edge`] at their exact time.
    pub(crate) fn dq250_valves(&mut self) {
        use crate::dq250_io::*;
        self.engine_io_sync();
        let now = self.now_ns();
        let master = self.latch(MASTER_ON) && !self.latch(MASTER_OFF) && self.p().wd.enabled();
        let hs = HIGH_SIDES.map(|(p, _, _)| master && self.latch(p));
        let levels: [bool; 11] = std::array::from_fn(|k| match VALVES[k].drive {
            ValveDrive::Pwm(cell) => self.engine_io.tap(0, crate::gpta::CellKind::Ltc, cell).level,
            ValveDrive::Port(p, b) => self.latch((p, b)),
        });
        let p = self.p();
        let v = p.valves.supply_v;
        let mut hs_i = [0.0f64; 3];
        for (k, c) in p.valves.coils.iter_mut().enumerate() {
            c.advance(now, 1e9, v);
            c.cell_level = levels[k];
            c.on = levels[k] && hs[VALVES[k].hs];
            hs_i[VALVES[k].hs] += c.i_a;
            if let Some(ain) = VALVES[k].sense {
                p.adc0.volts[ain] = (c.i_a as f32 * COIL_SENSE_V_PER_A).min(crate::tc1766_adc::VAREF);
            }
        }
        for (h, &(_, ain, gain)) in HIGH_SIDES.iter().enumerate() {
            p.adc0.volts[ain] = (hs_i[h] as f32 * gain).min(crate::tc1766_adc::VAREF);
        }
        for ain in MASTER_SENSE_AIN {
            p.adc0.volts[ain] = if master { v as f32 / KL30_DIVIDER } else { 0.0 };
        }
    }

    /// A GPTA0 LTC output transition at board time `t_ns` (from the GPTA sync): the coil of a PWM valve on that cell is
    /// integrated up to the edge and switched.
    pub(crate) fn dq250_valve_edge(&mut self, cell: u8, level: bool, t_ns: u64) {
        use crate::dq250_io::*;
        let Some(k) = VALVES.iter().position(|v| v.drive == ValveDrive::Pwm(cell)) else { return };
        let master = self.latch(MASTER_ON) && !self.latch(MASTER_OFF) && self.p().wd.enabled();
        let hs = master && self.latch(HIGH_SIDES[VALVES[k].hs].0);
        let p = self.p();
        let v = p.valves.supply_v;
        let c = &mut p.valves.coils[k];
        c.advance(t_ns, 1e9, v);
        c.cell_level = level;
        c.on = level && hs;
    }

    /// Queues the toothed wheels' edges up to `now + PIN_LOOKAHEAD_NS` into the GPTA (GPTA0 and LTCA2 see the same
    /// input lines): every tooth edge the wheel's angle crosses, at the time it crosses it.
    fn dq250_pins(&mut self, now: f64) {
        let mut edges = Vec::new();
        for g in &mut self.p().pins {
            let line = g.line;
            g.advance(now, now + PIN_LOOKAHEAD_NS, &mut |t, level| edges.push(crate::gpta::InputEdge { t: t as u64, kernel: 0, line, level }));
        }
        for e in edges {
            for kernel in 0..3 {
                self.engine_io.push_edge(crate::gpta::InputEdge { kernel, ..e });
            }
        }
    }

    /// FADC clock (fFADC from fSYS = fCPU / (FPIDIV + 1)) and events up to now. The ASW polls RCH0 and
    /// enables no FADC interrupt, so the FADC does not bound the quantum: its events are completed on
    /// access and at quantum boundaries. ponytail: an FADC SRC enabled later is served at the next
    /// quantum boundary, add the FADC to `dq250_insns_to_next_event` when a firmware enables one.
    fn dq250_fadc_advance(&mut self) {
        self.dq250_fadc_clock(false);
    }

    fn dq250_fadc_clock(&mut self, reclock: bool) {
        let now = (self.total_instructions + self.quantum_executed) as f64;
        let per_fsys = (self.scu.fpidiv() as f64 + 1.0) / CYCLES_PER_INSN as f64;
        let p = self.p();
        if reclock || !p.fadc.quiet(per_fsys, now) {
            p.fadc.set_clock(per_fsys, now);
            let srs = p.fadc.advance(now);
            self.dq250_fadc_raise(srs);
        }
    }

    fn dq250_fadc_raise(&mut self, srs: u32) {
        for n in 0..4 {
            if srs & (1 << n) != 0 {
                self.src_set_srr(FADC_BASE + 0xFC - 4 * n); // FADC_SRCn (TC1796 UM Table 26-11)
            }
        }
    }

    /// Instructions until the next ADC0 event.
    pub(crate) fn dq250_insns_to_next_event(&self) -> u64 {
        let Some(t) = self.dq250.as_ref().and_then(|p| p.adc0.next_event()) else { return u64::MAX };
        let dt = t.saturating_sub(self.now_ns()) as u128;
        ((dt * self.scu.fcpu_hz() as u128).div_ceil(1_000_000_000u128 * CYCLES_PER_INSN as u128) as u64).max(1)
    }

    /// MMIO read of a TC1766 board address; `None` = plain register file.
    fn dq250_trace(&mut self, a: u32, v: u32, m: u32) {
        let (insns, pc) = (self.total_instructions + self.quantum_executed, self.last_tb_pc);
        let p = self.p();
        if (m != 0 || p.trace_reads) && p.trace_range.is_some_and(|(lo, hi)| (lo..=hi).contains(&a)) && p.trace_log.len() < 400_000 {
            p.trace_log.push((insns, pc, a, v, m));
        }
    }

    /// Side-effect-free peek of a TC1766 register.
    pub(crate) fn dq250_peek(&self, a: u32) -> Option<u32> {
        if is_tc1766_src(a) {
            return Some(self.src_get(a));
        }
        if let Some(v) = self.engine_io_peek(a) {
            return Some(v);
        }
        let p = self.dq250.as_deref()?;
        if Adc0::contains(a) {
            return Some(p.adc0.read(a, self.now_ns(), self.f_sys()));
        }
        if (FADC_BASE..FADC_BASE + 0xF0).contains(&a) {
            return Some(p.fadc.read(a - FADC_BASE));
        }
        if CLC_REGS.contains(&a) {
            let v = self.seg_f000.get(a);
            return Some((v & !2) | ((v & 1) << 1));
        }
        match a {
            RST_SR => Some(match self.reset_log.last().map(|r| r.0) {
                Some(ResetKind::Software) => 1 << 29,
                Some(ResetKind::Watchdog) => 1 << 30,
                _ => 1 << 27,
            }),
            RST_REQ => Some(p.rst_req),
            WDT_CON0 => Some(self.scu.con0_read()),
            WDT_CON1 => Some(self.scu.wdt_con1),
            WDT_SR => Some(self.scu.sr_read()),
            PLL_CLC => {
                let v = p.pll_clc & !0x3;
                Some(v | (v & (1 << 24) == 0) as u32)
            }
            SCU_CHIPID => Some(CHIPID_VALUE),
            0xF000_0200..=0xF000_02FF => self.stm_peek(a - STM_BASE),
            P0_IN => {
                let (known, level) = self.engine_io.input_levels();
                let iocr = |pin: u32| (self.seg_f000.get(0xF000_0C10 + (pin / 4) * 4) >> ((pin % 4) * 8 + 4)) & 0x8;
                let wired = (0..16).filter(|&n| known & (1 << n) != 0 && iocr(n) == 0).fold(0u32, |m, n| m | 1 << n);
                self.scu_read(a).map(|v| (v & !wired) | (level as u32 & wired))
            }
            0xF000_0C00..=0xF000_17FF => self.scu_read(a),
            0xF000_4000..=0xF000_7FFF => self.can_read(can_tc1791_addr(a)),
            0xF800_2000..=0xF800_4FFF => self.flash_reg_read(a),
            _ if SSC_BASES.iter().any(|&b| a == b + 0x24) => {
                let i = ((a >> 8) & 3) as usize - 1;
                Some(p.ssc_rb[i])
            }
            _ if SSC_BASES.iter().any(|&b| a == b + 0x28) => Some(0),
            _ => None,
        }
    }

    pub(crate) fn dq250_read(&mut self, a: u32) -> Option<u32> {
        self.dq250_trace(a, 0, 0);
        if is_tc1766_src(a) {
            return Some(self.src_get(a));
        }
        if let Some(v) = self.engine_io_read(a) {
            return Some(v); // GPTA0/LTCA2 and PCP (crate::engine_io)
        }
        if Adc0::contains(a) {
            self.dq250_update();
            let (now, f) = (self.now_ns(), self.f_sys());
            return Some(self.p().adc0.read(a, now, f));
        }
        if (FADC_BASE..FADC_BASE + 0xF0).contains(&a) {
            self.dq250_fadc_advance();
            return Some(self.p().fadc.read(a - FADC_BASE));
        }
        if CLC_REGS.contains(&a) {
            let v = self.seg_f000.get(a);
            return Some((v & !2) | ((v & 1) << 1));
        }
        let r = match a {
            // Reset cause in bits 27..30, as decoded by the firmware at 0x8006F7A0
            // (RST_SR >> 24: 0x08 power-on, 0x10 hardware, 0x20 software, 0x40 watchdog).
            RST_SR => Some(match self.reset_log.last().map(|r| r.0) {
                Some(ResetKind::Software) => 1 << 29,
                Some(ResetKind::Watchdog) => 1 << 30,
                _ => 1 << 27,
            }),
            RST_REQ => Some(self.p().rst_req),
            WDT_CON0 => Some(self.scu.con0_read()),
            WDT_CON1 => Some(self.scu.wdt_con1),
            WDT_SR => Some(self.scu.sr_read()),
            PLL_CLC => {
                // Lock time is not modelled: locked whenever the oscillator is connected.
                let v = self.p().pll_clc & !0x3;
                Some(v | (v & (1 << 24) == 0) as u32)
            }
            SCU_CHIPID => Some(CHIPID_VALUE),
            0xF000_0200..=0xF000_02FF => self.stm_read(a - STM_BASE),
            // P0_IN: P0.n is the pin of GPTA0 input line INn (HYP, TC1766 pin table not in vault/docs/infineon), so the input
            // pins read the speed-sensor levels the GPTA sees; the PCP samples P0.0 / P0.2 at the output-speed captures
            // to tell the direction (docs/DQ250_FIRMWARE_SPEC.md §0, CMEM 0x13D/0x158).
            P0_IN => {
                let (known, level) = self.engine_io.input_levels();
                let iocr = |pin: u32| (self.seg_f000.get(0xF000_0C10 + (pin / 4) * 4) >> ((pin % 4) * 8 + 4)) & 0x8;
                let wired = (0..16).filter(|&n| known & (1 << n) != 0 && iocr(n) == 0).fold(0u32, |m, n| m | 1 << n);
                self.scu_read(a).map(|v| (v & !wired) | (level as u32 & wired))
            }
            0xF000_0C00..=0xF000_17FF => self.scu_read(a),
            0xF000_4000..=0xF000_7FFF => self.can_read(can_tc1791_addr(a)),
            0xF800_2000..=0xF800_4FFF => self.flash_reg_read(a),
            _ if SSC_BASES.iter().any(|&b| a == b + 0x24) => {
                let i = ((a >> 8) & 3) as usize - 1;
                Some(self.p().ssc_rb[i])
            }
            _ if SSC_BASES.iter().any(|&b| a == b + 0x28) => Some(0), // STAT: idle, no errors
            _ => None,
        };
        if r.is_none() {
            *self.p().plain_reads.entry(a).or_insert(0) += 1;
        }
        r
    }

    /// MMIO write of a TC1766 board address; `false` = plain register file.
    pub(crate) fn dq250_write(&mut self, a: u32, v: u32, m: u32) -> bool {
        self.dq250_trace(a, v, m);
        if is_tc1766_src(a) {
            self.src_write(a, v, m);
            return true;
        }
        if self.engine_io_write(a, v, m) {
            return true;
        }
        if (FADC_BASE..FADC_BASE + 0xF0).contains(&a) {
            // CLC / FDR are ENDINIT-protected (TC1796 UM Table 18-28)
            if !(matches!(a - FADC_BASE, 0x00 | 0x0C) && self.scu.endinit_active(a)) {
                let now = (self.total_instructions + self.quantum_executed) as f64;
                let srs = self.p().fadc.write(a - FADC_BASE, v, m, now);
                self.dq250_fadc_raise(srs);
                self.dq250_fadc_clock(true); // CLC/FDR change the clock
            }
            return true;
        }
        if Adc0::contains(a) {
            if a == crate::tc1766_adc::ADC0_BASE && self.scu.endinit_active(a) {
                return true;
            }
            self.dq250_update();
            let (now, f) = (self.now_ns(), self.f_sys());
            self.p().adc0.write(a, v, m, now, f);
            self.stop_requested = true; // a started timer/conversion bounds the quantum
            return true;
        }
        let protected = CLC_REGS.contains(&a) || matches!(a, RST_REQ | WDT_CON1 | PLL_CLC);
        if protected && self.scu.endinit_active(a) {
            return true;
        }
        let merge = |old: u32| (old & !m) | (v & m);
        match a {
            RST_REQ => {
                let w = merge(self.p().rst_req);
                self.p().rst_req = w;
                // Bit 0 is set in every reset request the firmware issues (0x5, 0x00200001,
                // 0x01220005) and clear in its initialisation writes (0): UNVERIFIED decode.
                if w & 1 != 0 {
                    let pc = self.last_tb_pc;
                    if self.p().sw_reset_requests.len() < 4096 {
                        self.p().sw_reset_requests.push(pc);
                    }
                    self.reset_request = Some(ResetKind::Software);
                    self.stop_requested = true;
                }
            }
            WDT_CON0 => {
                // Byte-wise writes cannot form a valid password sequence on hardware either.
                let w = if m == u32::MAX { v } else { merge(self.scu.con0_read()) };
                self.scu.con0_write(w);
            }
            WDT_CON1 => self.scu.wdt_con1 = merge(self.scu.wdt_con1) & 0xC,
            RST_SR | WDT_SR | SCU_CHIPID => {}
            PLL_CLC => {
                let w = merge(self.p().pll_clc) & !0x3; // LOCK read-only, RESLD reads 0
                self.p().pll_clc = w;
                let (c0, c1) = pll_to_scu(w);
                self.scu.pllcon0 = c0;
                self.scu.pllcon1 = c1;
            }
            0xF000_0200..=0xF000_02FF => return self.stm_write(a - STM_BASE, v, m) || {
                self.plain_store(a, v, m);
                true
            },
            0xF000_0C00..=0xF000_17FF => {
                if !self.scu_write(a, v, m) {
                    self.plain_store(a, v, m);
                }
                self.spi_cs_update();
                self.dq250_valves();
            }
            0xF000_4000..=0xF000_7FFF if self.can_write(can_tc1791_addr(a), v, m) => {}
            _ if a == ASC0_BASE + 0x20 => {
                // TBUF: the frame leaves immediately; transmit-buffer and transmit interrupts.
                self.plain_store(a, v, m);
                if self.p().asc0_tx.len() < 65_536 {
                    self.p().asc0_tx.push(v as u8);
                }
                self.src_set_srr(ASC0_BASE + 0xFC);
                self.src_set_srr(ASC0_BASE + 0xF0);
            }
            _ if SSC_BASES.iter().any(|&b| a == b + 0x20) => {
                self.plain_store(a, v, m);
                let i = ((a >> 8) & 3) as usize - 1;
                self.ssc_transfer(i, v & m);
            }
            _ => {
                if protected {
                    self.plain_store(a, v, m);
                    return true;
                }
                let p = self.p();
                *p.plain_writes.entry(a).or_insert(0) += 1;
                p.last_written.insert(a, v & m);
                return false;
            }
        }
        true
    }

    /// Chip-select edges of SPI devices (port output latch changes).
    fn spi_cs_update(&mut self) {
        let sel = self.port_out(EEPROM_CS.0) & (1 << EEPROM_CS.1) == 0
            && self.seg_f000.get(0xF000_0C00 + 0x100 * EEPROM_CS.0 as u32 + 0x18) & 0x80 != 0; // P2.8 output
        let p = self.p();
        if p.eeprom_selected && !sel {
            p.eeprom.deselect();
        }
        p.eeprom_selected = sel;
    }

    /// SSC master transfer: shifts `(CON.BM + 1)` bits MSB-first (CON.HB = 1 for every channel
    /// this firmware configures) to the selected device. The transfer completes immediately:
    /// TSRC (transmit buffer empty) and RSRC (receive buffer full) are requested at once.
    fn ssc_transfer(&mut self, i: usize, tx: u32) {
        let base = SSC_BASES[i];
        let con = self.seg_f000.get(base + 0x10);
        if con & 0x8000 == 0 {
            return; // programming mode: no transfer
        }
        let bits = (con & 0xF) + 1;
        let data = tx & ((1u64 << bits) - 1) as u32;
        let mut rx = 0u32;
        let nbytes = bits.div_ceil(8);
        for k in (0..nbytes).rev() {
            let b = (data >> (8 * k)) as u8;
            let r = if i == 0 && self.p().eeprom_selected { self.p().eeprom.shift(b) } else { 0 };
            rx = (rx << 8) | r as u32;
        }
        // 16-bit devices answering in-frame (docs/DQ250_IO_MAP.md §5): the watchdog chip (SSC1, CS P2.1) and the SPI
        // temperature sensor (SSC0, CS P2.9).
        let cs_low = |s: &Self, (port, pin): (usize, u32)| s.port_out(port) & (1 << pin) == 0;
        if bits == 16 && i == 1 && cs_low(self, crate::dq250_io::WD_CS) {
            rx = self.p().wd.frame(data as u16) as u32;
        } else if bits == 16 && i == 0 && cs_low(self, crate::dq250_io::LM74_CS) {
            rx = crate::dq250_io::lm74_word(self.p().board_temp_c) as u32;
        }
        let rx = rx & ((1u64 << bits) - 1) as u32;
        let port_out = [0, 1, 2, 3, 4, 5].map(|p| self.port_out(p));
        let insns = self.total_instructions + self.quantum_executed;
        let p = self.p();
        p.ssc_rb[i] = rx;
        if p.spi_log.len() < SPI_LOG_MAX {
            p.spi_log.push(SpiXfer { ssc: i as u8, bits: bits as u8, tx: data, rx, port_out, insns });
        }
        self.src_set_srr(base + 0xF4);
        self.src_set_srr(base + 0xF8);
    }
}

/// WFS5 (MQB immobiliser, partner 1: challenge `0x012`, answer `0x013`) records in the SPI EEPROM, located by
/// filling the part with random bytes and finding them in the IMO's RAM copies (VERIFIED-T, 4319): three copies
/// of the 53-byte data record at 0x844 / 0x87C / 0x8B4 (RAM 0xD0009CE0 / 0x9D18 / 0x9D50, each checked with
/// CRC-16/CCITT-FALSE over 0x33 bytes, CRC LE at +0x33: `0x800E2BB6..0x800E2C30`, CRC routine 0x800E1984) and
/// three copies of the 13-byte state record at 0x814 / 0x824 / 0x834 (RAM 0xD0009D88.., CRC over 11 bytes,
/// `0x800E296C..`), which the ASW initialises itself on an erased part. Data-record layout as in the DQ200 and
/// SIMOS (`[1..17)` K1, `[0x11..0x22)` VIN, `[0x23]` power class B, `[0x25..0x27)` ID_kombi, `[0x2F..0x33)`
/// write counter LE): a record with K1 = 00 01 .. 0F and B = 0x5C makes the unmodified ASW send challenges
/// computed with exactly that K1 and B (C4 = 9C F7 FE B7, WFS5 variant 3), and the cluster answer with the
/// record's ID_kombi stops the challenges, a wrong ID_kombi does not (VERIFIED-T, `tests/dq250_boot.rs`).
/// With the erased part the ASW uses K1 = FF.., B = 0xFF. ID_tcu in the challenge is 3B A0 on 4319 (source not
/// located; not needed to answer).
pub const IMMO_DATA_COPIES: [usize; 3] = [0x844, 0x87C, 0x8B4];
pub const IMMO_STATE_COPIES: [usize; 3] = [0x814, 0x824, 0x834];
pub const IMMO_DATA_LEN: usize = 53;

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection) as the IMO computes it (0x800E193C).
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    data.iter().fold(0xFFFFu16, |mut c, &b| {
        c ^= (b as u16) << 8;
        for _ in 0..8 {
            c = if c & 0x8000 != 0 { (c << 1) ^ 0x1021 } else { c << 1 };
        }
        c
    })
}

/// Fills one WFS5 data record (`d.len() >= 53`): K, VIN (up to 17 chars, space-padded if shorter), B, ID_kombi,
/// counter and CRC; the other bytes of `d` are kept.
pub fn write_wfs5_data_record(d: &mut [u8], key: &[u8; 16], vin: &str, b: u8, id_kombi: [u8; 2], counter: u32) {
    if d.len() < IMMO_DATA_LEN {
        return;
    }
    d[1..17].copy_from_slice(key);
    let v = vin.as_bytes();
    let n = v.len().min(17);
    d[0x11..0x22].fill(b' ');
    d[0x11..0x11 + n].copy_from_slice(&v[..n]);
    d[0x23] = b;
    d[0x25..0x27].copy_from_slice(&id_kombi);
    d[0x2F..0x33].copy_from_slice(&counter.to_le_bytes());
    let c = crc16_ccitt(&d[..0x33]);
    d[0x33..0x35].copy_from_slice(&c.to_le_bytes());
}

/// Identification strings of a DQ250 image.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct Dq250ImageInfo {
    pub flash_size: usize,
    /// SBOOT identification ("02E927770X  0909").
    pub sboot_id: String,
    /// Hardware part numbers found in the programming records / CBOOT.
    pub hw_part_numbers: Vec<String>,
    /// ASW/calibration identification ("49300D9300012F_4930_ODKW").
    pub asw_id: String,
    pub odx: String,
}

/// Printable ASCII run at `s` (at most `max` bytes), trimmed.
fn ascii_at(img: &[u8], s: usize, max: usize) -> String {
    let Some(tail) = img.get(s..) else {
        return String::new();
    };
    let len = tail.iter().take(max).position(|&c| !(0x20..0x7F).contains(&c)).unwrap_or_else(|| max.min(tail.len()));
    String::from_utf8_lossy(&tail[..len]).trim().to_string()
}

fn find_str(img: &[u8], prefix: &[u8], max: usize) -> Vec<String> {
    let mut v = Vec::new();
    let mut i = 0;
    while let Some(p) = img[i..].windows(prefix.len()).position(|w| w == prefix) {
        let s = i + p;
        let t = ascii_at(img, s, max);
        if !v.contains(&t) {
            v.push(t);
        }
        i = s + 1;
    }
    v
}

impl Dq250ImageInfo {
    pub fn from_bytes(img: &[u8]) -> Self {
        let first = |p: &[u8], n| find_str(img, p, n).into_iter().next().unwrap_or_default();
        Self {
            flash_size: img.len(),
            sboot_id: img.get(0x62C..0x63C).map(|b| String::from_utf8_lossy(b).trim().to_string()).unwrap_or_default(),
            hw_part_numbers: find_str(img, b"02E927", 12),
            // "<version>0D9300<index>_<version>_<odx>" ("49300D9300012F_4930_ODKW", "43190D9300011R_4319_RcCC")
            asw_id: (0..img.len().saturating_sub(10))
                .filter(|&i| &img[i + 4..i + 10] == b"0D9300")
                .map(|i| ascii_at(img, i, 40))
                .find(|s| s.contains('_'))
                .unwrap_or_default(),
            odx: first(b"EV_TCM", 20),
        }
    }
}

/// Code locations used as observation-only boot milestones (`Simos18Board::add_probe`).
pub const MILESTONES: &[(&str, u32)] = &[
    ("SBOOT_START", 0xA000_4000),
    ("CBOOT_START", 0x8001_0200),
    ("ASW_START", 0x8004_0900),
];

/// DTC number -> SAE code tables of the calibration (diagnostics only): a list of VAG DTC numbers (u16, the UDS
/// `19 02` DTC low word) followed by the same count of SAE codes (u16, top two bits P/C/B/U). Two such tables in CAL
/// (4319: 0x80038288 with 99 entries ending in 0x0000, 0x800387E0 with 150 entries; 4930: 0x8003999C /
/// 0x80039EF4): DTC 0x29FB / 0x29FC -> P0810 like the DQ200 table 0x80032AD0, 0x529B..0x52A0 -> P0731..P0735,
/// P0729 (gear ratio 1..6). Located by their first entries; VERIFIED-S by the matching SAE codes.
pub fn dtc_table(flash: &[u8]) -> Vec<(u16, u16)> {
    let Some(cal) = flash.get(0x2_0000..flash.len().min(0x4_0000)) else {
        return Vec::new();
    };
    let word_at = |o: usize| cal.get(o..o + 2).map(|b| u16::from_le_bytes([b[0], b[1]]));
    let words = |o: usize, n: usize| (0..n).filter_map(move |i| word_at(o + 2 * i));
    let find = |pat: &[u16]| {
        let b: Vec<u8> = pat.iter().flat_map(|w| w.to_le_bytes()).collect();
        cal.windows(b.len()).position(|w| w == &b[..])
    };
    let mut t = Vec::new();
    if let Some(a) = find(&[0, 0x529B, 0x529C]).map(|o| o + 2) {
        if let Some(n) = (0..0x200).find(|&i| word_at(a + 2 * i) == Some(0)) {
            if a + 4 * n + 2 <= cal.len() {
                t.extend(words(a, n).zip(words(a + 2 * n + 2, n)));
            }
        }
    }
    if let Some(b) = find(&[0x29FB, 0x29FC]) {
        if let Some(n) = (1..0x400).find(|&n| b + 4 * n <= cal.len() && word_at(b + 2 * n) == Some(0x0810) && word_at(b + 2 * n + 2) == Some(0x0810)) {
            t.extend(words(b, n).zip(words(b + 2 * n, n)));
        }
    }
    t
}

/// Gear set a DQ250 calibration is made for: forward ratios and final drives of gears 1..6, reverse (ratio, final).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GearSet {
    pub ratios: [f64; 6],
    pub finals: [f64; 6],
    pub reverse: (f64, f64),
}

/// Tooth table of the calibration (22 bytes: `01`, drive teeth of gears 1..6, reverse idler, `01`, driven teeth 1..6,
/// reverse teeth, final pinions [sel 0, sel 1], final wheels [sel 0, sel 1], reverse shaft pair), read by the ratio code
/// (0D9300011R 0x8009D0C8 -> CAL 0x80030600, 0D9300012F 0x8008B380 -> 0x80034638: ratio·1000 = driven · wheel ·
/// 1000 / (drive · pinion), final selector 0,0,0,0,1,1 = tables 0x800DCA3C / 0x800C3D34, reverse = t15·t0F·t13 /
/// (t07·t14·t11); VERIFIED-S). Located by its structure (unique in both CALs); `None` when absent.
pub fn gear_set(flash: &[u8]) -> Option<GearSet> {
    let cal = flash.get(0x2_0000..0x4_0000)?;
    let ok = |t: &[u8]| t[0] == 1 && t[8] == 1 && t[0x12] == t[0x13] && t[1..8].iter().chain(&t[9..0x16]).all(|&x| (10..=80).contains(&x));
    let t = cal.windows(0x16).find(|t| ok(t))?;
    let f = |x: u8| x as f64;
    let sel = [0usize, 0, 0, 0, 1, 1];
    Some(GearSet {
        ratios: std::array::from_fn(|g| f(t[9 + g]) / f(t[1 + g])),
        finals: sel.map(|s| f(t[0x12 + s]) / f(t[0x10 + s])),
        reverse: (f(t[0x15]) * f(t[0x0F]) / (f(t[0x07]) * f(t[0x14])), f(t[0x13]) / f(t[0x11])),
    })
}

/// SAE text of a code from [`dtc_table`] ("P0810", "U0100").
pub fn sae_text(code: u16) -> String {
    format!("{}{:04X}", ['P', 'C', 'B', 'U'][(code >> 14) as usize], code & 0x3FFF)
}

/// Physical state of the mechatronic's surroundings (plant side), applied as board signals by
/// [`Dq250TcuBoard::set_inputs`].
#[derive(Debug, Clone, Copy, Default)]
pub struct Dq250Inputs {
    /// Battery (terminal 30), V.
    pub battery_v: f32,
    /// Ignition on (terminal 15 / the board's supply input, [`Dq250TcuBoard::set_kl15_v`]).
    pub kl15: bool,
    /// Clutch input speed (engine side of both clutches), rpm.
    pub input_rpm: f64,
    /// Input shaft 1 / 2 speeds, rpm.
    pub shaft_rpm: [f64; 2],
    /// Output speed, rpm, signed (negative = rolling backwards: the A/B sensor phase follows the wheel).
    pub output_rpm: f64,
    /// Clutch K1 / K2 pressures, bar.
    pub clutch_bar: [f64; 2],
    /// Gearbox oil (ATF sump) temperature, °C.
    pub atf_c: f64,
    /// Clutch oil temperature, °C.
    pub clutch_oil_c: f64,
    /// Gear actuator (selector fork) travels in the plant's fork order, mm.
    pub fork_mm: [f64; 4],
}

/// Decoded valve outputs in the plant's wet-unit channel order (`simemu_physics::mechatronic::wet`: N88, N89, N90, N91,
/// N92, N215, N216, N217, N218, N233, N371). `None` = output stage not driven.
#[derive(Debug, Clone, Copy, Default)]
pub struct Dq250Outputs {
    pub duty: [Option<f64>; 11],
    /// Coil current, A (`None`: not measured by the board model, estimate from the duty).
    pub current_a: [Option<f64>; 11],
}

/// DQ250 TCU board: the TC1766 SoC running the unmodified PFlash image.
pub struct Dq250TcuBoard {
    pub cpu: Simos18Board,
    pub info: Dq250ImageInfo,
    /// [`dtc_table`] of the loaded image.
    pub dtcs: Vec<(u16, u16)>,
    /// [`gear_set`] of the loaded image.
    pub gears: Option<GearSet>,
}

impl Dq250TcuBoard {
    /// `flash` must be the 0x178000-byte PFlash image (a 0x180000-byte read with a zero tail is accepted); `eeprom` the SPI EEPROM content (empty:
    /// an erased 8 KB part, see [`EEPROM_DEFAULT_SIZE`]).
    pub fn new(flash: &[u8], eeprom: &[u8]) -> Result<Self, String> {
        Self::new_on(flash, eeprom, crate::CpuBackend::from_env())
    }

    pub fn new_on(flash: &[u8], eeprom: &[u8], backend: crate::CpuBackend) -> Result<Self, String> {
        // Readers that dump a 1.5 MB window (PCM Flasher: 0x180000) append 0x8000 bytes past the end of the
        // TC1766 PFlash; they are accepted only when they are all zero (nothing is mapped there).
        let flash = match flash.len() {
            PFLASH_SIZE => flash,
            PFLASH_READ_SIZE if flash[PFLASH_SIZE..].iter().all(|&b| b == 0) => &flash[..PFLASH_SIZE],
            n => {
                return Err(format!(
                    "DQ250 PFlash image must be 0x{PFLASH_SIZE:X} bytes (or 0x{PFLASH_READ_SIZE:X} with a zero tail), got 0x{n:X}"
                ))
            }
        };
        let mut cpu = Simos18Board::new_variant_on(flash, eeprom, BoardVariant::Dq250Tc1766, backend)?;
        cpu.hw_mut().engine_io.gpta = crate::gpta::Gpta::tc1766(crate::gpta::F_GPTA_DEFAULT);
        for &(_, a) in MILESTONES {
            cpu.add_probe(a)?;
            cpu.add_probe(a ^ 0x2000_0000)?;
        }
        let mut b = Self { cpu, info: Dq250ImageInfo::from_bytes(flash), dtcs: dtc_table(flash), gears: gear_set(flash) };
        // Mechatronic on a standing gearbox (bench): battery and ignition on, actuators and clutches at rest, 25 °C.
        b.set_inputs(&Dq250Inputs { battery_v: KL30_DEFAULT_V, kl15: true, atf_c: 25.0, clutch_oil_c: 25.0, ..Default::default() });
        Ok(b)
    }

    pub fn run_for(&mut self, seconds: f64) -> Result<RunStats, String> {
        self.cpu.run_for(seconds)
    }

    pub fn pc(&self) -> u32 {
        self.cpu.read_reg(unicorn_tricore_sys::TriCoreReg::PC)
    }

    pub fn periph(&self) -> &Tc1766Periph {
        self.cpu.hw().dq250.as_deref().expect("DQ250 board state")
    }

    /// Milestones reached (probe hit counts, cached + non-cached address).
    pub fn milestones(&self) -> Vec<(&'static str, u64)> {
        MILESTONES
            .iter()
            .map(|&(n, a)| (n, [a, a ^ 0x2000_0000].iter().filter_map(|&x| self.cpu.probe(x)).map(|p| p.hits).sum()))
            .collect()
    }

    /// SAE code of a UDS DTC (`19 02` 3-byte DTC) by the image's own table, `None` when not listed.
    pub fn dtc_sae_code(&self, dtc: u32) -> Option<String> {
        (dtc >> 16 == 0).then_some(())?;
        self.dtcs.iter().find(|e| e.0 as u32 == dtc).map(|e| sae_text(e.1))
    }

    pub fn can_tx(&self) -> &[CanFrame] {
        &self.cpu.hw().can_tx_queue
    }

    /// Supply input that wakes the ASW and keeps its network request (AIN31 behind the external multiplexer,
    /// EMUX 0, 6:1, [`BOARD_EMUX`]): with it at battery level the ASW keeps transmitting and sends WFS5 challenges;
    /// at 0 V it releases the bus after ~4.8 s (NM repeat phase only) — VERIFIED-T, `tests/dq250_boot.rs`.
    /// HYP: terminal 15 (the ASW reads only bit 20 of `Klemmen_Status_01`, 0x80074C8E, not ZAS_Kl_15).
    pub fn set_kl15_v(&mut self, volts: f32) {
        let p = self.cpu.hw_mut().dq250.as_mut().expect("DQ250 board state");
        if let Some(e) = p.adc0.emux[31].as_mut() {
            e[0] = volts / KL30_DIVIDER;
        }
    }

    /// Applies the plant state as the board's electrical inputs (sensor transfers: [`crate::dq250_io`]).
    pub fn set_inputs(&mut self, i: &Dq250Inputs) {
        use crate::dq250_io::*;
        self.set_kl15_v(if i.kl15 { i.battery_v } else { 0.0 });
        let speeds = [i.input_rpm, i.shaft_rpm[0], i.shaft_rpm[1], i.output_rpm, i.output_rpm];
        for (k, (&(a, b, teeth), rpm)) in SPEED_SENSORS.iter().zip(speeds).enumerate() {
            // Output B (IN0/IN1) reads the same wheel as output A (IN2/IN3) a quarter pitch ahead: turning forwards, B is
            // low when A falls — the firmware's "forward" (PCP CMEM 0x13D samples P0.0 = IN0 at the falling-edge capture
            // of IN2, table 0x800C43F6: low = +1; docs/DQ250_FIRMWARE_SPEC.md §0). Reverse rotation inverts it by itself.
            let phase = if k == 4 { 0.25 } else { 0.0 };
            self.set_wheel(a, rpm, teeth, phase);
            self.set_wheel(b, rpm, teeth, phase);
        }
        let p = self.cpu.hw_mut().dq250.as_mut().expect("DQ250 board state");
        for (f, &ain) in TRAVEL_SENSOR_AIN.iter().enumerate() {
            p.adc0.volts[ain] = travel_sensor_v(i.fork_mm[f]);
        }
        for (k, &ain) in PRESSURE_SENSOR_AIN.iter().enumerate() {
            p.adc0.volts[ain] = pressure_sensor_v(i.clutch_bar[k]);
        }
        p.adc0.volts[ATF_TEMP_AIN] = atf_temp_v(i.atf_c);
        // the mechatronic sits in the ATF: board sensor at the clutch-oil temperature (HYP)
        p.board_temp_c = i.clutch_oil_c;
        p.valves.supply_v = i.battery_v.max(0.0) as f64;
    }

    /// Valve coil resistances (plant order, at the coils' temperatures) and inductance.
    pub fn set_coil_params(&mut self, r_ohm: &[f64], l_h: f64) {
        let p = self.cpu.hw_mut().dq250.as_mut().expect("DQ250 board state");
        for (c, &r) in p.valves.coils.iter_mut().zip(r_ohm) {
            c.r_ohm = r;
            c.l_h = l_h;
        }
    }

    /// Valve outputs as the board drives them: per valve the coil current (exact R-L integral, 1 ms mean) and whether
    /// its stage is switched (`duty` = the share of the battery voltage the mean current corresponds to).
    pub fn decode_outputs(&self) -> Dq250Outputs {
        let p = self.periph();
        let mut o = Dq250Outputs::default();
        for (k, c) in p.valves.coils.iter().enumerate() {
            let driven = c.on || c.i_mean_a > 1e-3;
            o.duty[k] = driven.then(|| (c.i_mean_a * c.r_ohm / p.valves.supply_v.max(1.0)).clamp(0.0, 1.0));
            o.current_a[k] = Some(c.i_mean_a);
        }
        o
    }

    /// GPTA input line `line` follows a `teeth` wheel turning at `rpm` (signed) from now on (`PinGen`); the speed
    /// applies from the end of the edges already queued. A new wheel starts at angle `phase` (tooth pitches).
    pub fn set_wheel(&mut self, line: u8, rpm: f64, teeth: u32, phase: f64) {
        let now = self.cpu.hw().now_ns() as f64;
        let phase = if phase.is_finite() { phase.rem_euclid(1.0) } else { 0.0 };
        let new_rate = if rpm.is_finite() && teeth > 0 {
            rpm / 60.0 * teeth as f64 * 2.0 / 1e9
        } else {
            0.0
        };
        let new_rate = if new_rate.is_finite() { new_rate } else { 0.0 };
        let p = self.cpu.hw_mut().dq250.as_mut().expect("DQ250 board state");
        let g = match p.pins.iter().position(|g| g.line == line) {
            Some(i) => &mut p.pins[i],
            None => {
                let pos = (2.0 * phase + 1e-6).rem_euclid(2.0);
                p.pins.push(PinGen { line, pos, t_ns: now, rate: 0.0, level: false });
                p.pins.last_mut().expect("just pushed")
            }
        };
        if now > g.t_ns {
            if g.rate.is_finite() {
                g.pos = (g.pos + g.rate * (now - g.t_ns)).rem_euclid(2.0);
            }
            g.t_ns = now;
        }
        if g.rate == 0.0 && new_rate != 0.0 {
            g.t_ns = now;
        }
        g.rate = new_rate;
    }

    /// Drive the board's analog input line AIN`idx` (0..31) with `volts`.
    pub fn set_ain(&mut self, idx: usize, volts: f32) {
        if volts.is_finite() {
            if let Some(slot) = self.cpu.hw_mut().dq250.as_mut().expect("DQ250 board state").adc0.volts.get_mut(idx) {
                *slot = volts;
            }
        }
    }

    /// Diagnostic register-access trace, see [`Tc1766Periph::trace_log`].
    pub fn set_io_trace(&mut self, range: Option<(u32, u32)>, reads: bool) {
        let p = self.cpu.hw_mut().dq250.as_mut().expect("DQ250 board state");
        p.trace_range = range;
        p.trace_reads = reads;
    }

    pub fn inject_can_rx(&mut self, f: &CanFrame) -> usize {
        self.cpu.inject_can_rx(f)
    }

    /// SRCs with a pending request (SRR set): `(address, value)`.
    pub fn pending_requests(&self) -> Vec<(u32, u32)> {
        let hw = self.cpu.hw();
        let mut v: Vec<(u32, u32)> = crate::stm_icu::src_table()
            .iter()
            .copied()
            .chain((0xF000_0AF0..=0xF000_0AFC).step_by(4))
            .chain((0xF010_01F4..=0xF010_01FC).step_by(4))
            .chain((0xF010_02F4..=0xF010_02FC).step_by(4))
            .chain((0xF010_03F0..=0xF010_03FC).step_by(4))
            .chain((0xF010_05F0..=0xF010_05FC).step_by(4))
            .map(|a| (a, hw.src_get(a)))
            .filter(|&(_, s)| s & SRC_SRR != 0)
            .collect();
        v.dedup();
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eeprom_read_write_status_sequence() {
        let mut e = SpiEeprom::new(&[]);
        assert_eq!(e.mem.len(), EEPROM_DEFAULT_SIZE);
        // WRITE without WREN is ignored
        for b in [0x02, 0x00, 0x40, 0x11] {
            e.shift(b);
        }
        e.deselect();
        assert_eq!(e.mem[0x40], 0xFF);
        e.shift(0x06);
        e.deselect();
        e.shift(0x05);
        assert_eq!(e.shift(0), 0x02, "WEL set");
        e.deselect();
        // page write wraps inside the 32-byte page
        for b in [0x02, 0x00, 0x5E, 0xA1, 0xA2, 0xA3] {
            e.shift(b);
        }
        e.deselect();
        assert_eq!((e.mem[0x5E], e.mem[0x5F], e.mem[0x40]), (0xA1, 0xA2, 0xA3));
        e.shift(0x05);
        assert_eq!(e.shift(0), 0, "WEL cleared by the write");
        e.deselect();
        for b in [0x03, 0x00, 0x5E] {
            e.shift(b);
        }
        assert_eq!([e.shift(0), e.shift(0)], [0xA1, 0xA2]);
    }

    /// Toothed wheel at a Hall sensor: a steady rotation gives two edges per tooth at the right times; an
    /// oscillation inside one pitch gives none; a reversal across an edge toggles it back.
    #[test]
    fn toothed_wheel_edges_follow_the_angle() {
        let count = |g: &mut PinGen, t0: f64, t1: f64| {
            let mut v = Vec::new();
            g.advance(t0, t1, &mut |t, l| v.push((t, l)));
            v
        };
        // 600 rpm, 36 teeth: 360 teeth/s -> 720 edges/s, 1.389 ms apart
        let mut g = PinGen { line: 0, pos: 1e-6, t_ns: 0.0, rate: 600.0 / 60.0 * 36.0 * 2.0 / 1e9, level: false };
        let e = count(&mut g, 0.0, 1e9);
        assert_eq!(e.len(), 720);
        assert!((e[1].0 - e[0].0 - 1e9 / 720.0).abs() < 1.0 && e[0].1 && !e[1].1);
        // ±1° oscillation (pitch 10°) away from an edge: no edges
        let mut g = PinGen { line: 0, pos: 0.5, t_ns: 0.0, rate: 0.0, level: false };
        let r = (1.0 / 5.0) / 10e6; // 1° in 10 ms, in half pitches per ns
        for k in 0..20 {
            g.rate = if k % 2 == 0 { r } else { -r };
            assert!(count(&mut g, k as f64 * 10e6, (k + 1) as f64 * 10e6).is_empty(), "swing {k}");
        }
        // across an edge and back: two toggles
        let mut g = PinGen { line: 0, pos: 0.9, t_ns: 0.0, rate: 0.2 / 1e6, level: false };
        assert_eq!(count(&mut g, 0.0, 1e6).iter().map(|e| e.1).collect::<Vec<_>>(), [true]);
        g.rate = -g.rate;
        assert_eq!(count(&mut g, 1e6, 2e6).iter().map(|e| e.1).collect::<Vec<_>>(), [false]);
    }

    /// Output speed sensors as `set_inputs` places them (B a quarter pitch ahead of A): at every falling edge of A
    /// (the edge GTC02 captures, GTCCTR 0x428) B is low turning forwards and high turning backwards — the levels the
    /// PCP maps to +1 / −1 (docs/DQ250_FIRMWARE_SPEC.md §0).
    #[test]
    fn output_sensor_b_is_low_at_a_falling_edges_when_turning_forwards() {
        for (rate, b_level) in [(1e-6, false), (-1e-6, true)] {
            let wheel = |phase: f64| PinGen { line: 0, pos: 2.0 * phase + 1e-6, t_ns: 0.0, rate, level: PinGen::level_at(2.0 * phase + 1e-6) };
            let (mut a, mut b) = (wheel(0.0), wheel(0.25));
            let mut edges = Vec::new();
            a.advance(0.0, 2e7, &mut |t, l| edges.push((t, 0, l)));
            b.advance(0.0, 2e7, &mut |t, l| edges.push((t, 1, l)));
            edges.sort_by(|x, y| x.0.total_cmp(&y.0));
            let mut lb = PinGen::level_at(0.5 + 1e-6);
            let mut falls = 0;
            for (_, s, l) in edges {
                match (s, l) {
                    (1, l) => lb = l,
                    (0, false) => {
                        assert_eq!(lb, b_level, "rate {rate}");
                        falls += 1;
                    }
                    _ => {}
                }
            }
            assert!(falls >= 9, "rate {rate}: {falls}");
        }
    }

    #[test]
    fn pll_clc_to_clock() {
        // SBOOT value 0x00272926 with VCOBYP cleared: N = 40, P = 2, K = 10 -> 2 * fOSC.
        let mut s = crate::scu::ScuWdt::new(ResetKind::PowerOn);
        let (c0, c1) = pll_to_scu(0x0027_2926 & !0x20);
        s.pllcon0 = c0;
        s.pllcon1 = c1;
        assert_eq!(s.fcpu_hz(), 2 * crate::stm_icu::FOSC_HZ);
    }

    #[test]
    fn endinit_blocks_protected_regs_and_truncated_inputs_do_not_panic() {
        // Truncated flash / EEPROM inputs must not panic
        assert!(dtc_table(&[]).is_empty());
        assert!(dtc_table(&vec![0u8; 0x2_0100]).is_empty());
        let mut short_eep = [0u8; 16];
        write_wfs5_data_record(&mut short_eep, &[0xAA; 16], "LABEMUTEST0000001", 0x5C, [1, 2], 1);
        assert_eq!(short_eep, [0u8; 16], "short EEPROM buffer must be left untouched without panic");

        // PinGen at huge t_ns (where sub-ns step < 1 ULP of t_ns) must terminate cleanly without hanging!
        let mut g = PinGen { line: 0, pos: 0.0, t_ns: 1e16, rate: 1e-12, level: false };
        let mut edges = 0usize;
        g.advance(1e16, 1e16 + 1e6, &mut |_, _| edges += 1);
        assert!(edges <= 16_384);

        // ENDINIT=1 on Dq250Tc1766 blocks PLL_CLC, RST_REQ, WDT_CON1, FADC_CLC, ADC0_CLC, SSC0_CLC
        let mut b = Simos18Board::new_variant(
            &vec![0u8; 0x18_0000],
            &vec![0xFFu8; 0x2000],
            BoardVariant::Dq250Tc1766,
        )
        .unwrap();
        let hw = b.hw_mut();
        assert!(hw.scu.endinit);
        for &reg in &[PLL_CLC, RST_REQ, WDT_CON1, FADC_BASE, crate::tc1766_adc::ADC0_BASE, 0xF010_0100] {
            let before = hw.periph_peek(reg);
            hw.mmio_write(reg, 4, 0xDEAD_BEEF);
            assert_eq!(hw.periph_peek(reg), before, "DQ250 0x{reg:08X} must block under ENDINIT=1");
        }
        assert_eq!(hw.scu.endinit_dropped, 6);
    }
}
