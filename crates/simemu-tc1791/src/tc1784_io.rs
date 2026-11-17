//! TC1784 (AUDO Future) peripherals of the DQ200_G2 mechatronic board that differ from the
//! TC1791 models of the shared core: SSC0/SSC1 + the ST95640 SPI EEPROM, the DMA move engine
//! (subset used by the AUTOSAR Spi driver), the ADC request-source/event structure, the GPTA
//! (GPTA0 + LTCA2 of [`crate::gpta`], clocked from GPTA0_FDR, pin inputs from [`PinSignal`]
//! generators) and the module SRC registers that are not part of the core's TC1791 SRC table.
//! The PCP is not modelled: only SBOOT (0x80005744/0x80005768) touches PCP_CLC/PCP_CS, the
//! application never loads PRAM/CMEM and every SRC it configures routes to the CPU.
//!
//! Every behaviour below is derived from how the DQ200_G2 firmware (0CW300043H_0002) programs the
//! hardware; addresses are cited where they are used. Nothing here reads or writes firmware RAM
//! except DMA moves, which are bus transfers programmed by the firmware itself.

use crate::memory::{BoardVariant, Tc1791HardwareState};
use crate::st95640::St95640;

#[path = "fadc.rs"]
pub mod fadc;
#[path = "pump_bldc.rs"]
pub mod pump;

// ---- SSC (SBOOT init 0x800050CA: CLC +0x00, FDR +0x0C, CON +0x10, BR +0x14, TB +0x20, RB +0x24,
// STAT +0x28 (BSY bit 12 polled at 0x80005F32), EFM +0x2C; TSRC +0xF4, RSRC +0xF8, ESRC +0xFC) ----
pub const SSC_BASE: [u32; 2] = [0xF010_0100, 0xF010_0200];
const SSC_STAT_BSY: u32 = 1 << 12;

// ---- EEPROM chip select: P3.5 (SBOOT 0x80005F82: P3_OMR = 0x00200000 selects, 0x20 deselects).
pub const EEPROM_CS_PORT_BASE: u32 = 0xF000_0F00;
pub const EEPROM_CS_PIN: u32 = 5;

/// SPI slaves by chip select, from the AUTOSAR Spi job table 0x8013B90C (20-byte jobs: +4 BR,
/// +6 CS port<<8|pin, +9 polarity, +0xB data-width/phase bits) and the SSC each was seen on.
/// Modelled: the EEPROM, the pump driver, the monitor device and the two valve drivers; the chip
/// on CS P2.9 is UNIDENTIFIED (MISO reads all ones).
pub struct SpiDevice {
    pub name: &'static str,
    pub ssc: usize,
    pub port: u32,
    pub pin: u32,
}
pub const SPI_DEVICES: [SpiDevice; 6] = [
    SpiDevice { name: "ST95640 EEPROM (CS P3.5, jobs 5-8)", ssc: 0, port: 0xF000_0F00, pin: 5 },
    // V401 pump motor driver (`pump::BldcPump`): 16-bit LSB-first frames, chip UNIDENTIFIED.
    SpiDevice { name: "Pump motor driver (CS P3.6, jobs 0-3)", ssc: 0, port: 0xF000_0F00, pin: 6 },
    SpiDevice { name: "UNIDENTIFIED (CS P2.9, job 4)", ssc: 1, port: 0xF000_0E00, pin: 9 },
    // Valve driver chips ([`ValveDriver`]): their 32-bit words leave on SSC0 as two 16-bit halves
    // (low half first) while P3.7 (module 0) / P2.13 (module 1) is low (VERIFIED-T, SSC0 trace).
    SpiDevice { name: "Valve driver module 1 (SSC0, CS P2.13, jobs 9-11)", ssc: 0, port: 0xF000_0E00, pin: 13 },
    // Observed (VERIFIED-T): the 32-bit frames built by the driver 0x800DC060..0x800DD9F0 (frame
    // buffer 0xD00087A4) leave on SSC1 as two 16-bit halves (low half first) while P2.8 is low.
    SpiDevice { name: "Monitoring device, 32-bit protocol (SSC1, CS P2.8)", ssc: 1, port: 0xF000_0E00, pin: 8 },
    SpiDevice { name: "Valve driver module 0 (SSC0, CS P3.7)", ssc: 0, port: 0xF000_0F00, pin: 7 },
];
/// Index of the 32-bit monitoring device in [`SPI_DEVICES`].
pub const SPI_MONITOR: usize = 4;
/// Index of the pump motor driver in [`SPI_DEVICES`].
pub const SPI_PUMP: usize = 1;
/// [`SPI_DEVICES`] index of the valve driver chip of valve module 0 / 1.
pub const SPI_VALVE_DRIVER: [usize; 2] = [5, 3];

/// Valve driver chip of one valve module (4 channels; module 0 = Pwm ch [0, 1, 6, 5], module 1 =
/// [2, 4, 3, 7], `0x8013AEF0`). Two of them sit on SSC0 (CS P3.7 / P2.13); the firmware's driver
/// `0x800DEC0A..0x800DFE3E` + module state machine `0x800E0244` talk to them. The chip is
/// UNIDENTIFIED (no datasheet); the behaviour below is what the firmware checks (VERIFIED-S, REA
/// listing Evidence `ev_6f2627d7…`, Ghidra decompilation of the same functions):
/// - 32-bit words, bits 3:0 register (the status check compares this nibble), 26:4 data, 29:27 a
///   CRC-3 over bits 29:0 (`0x800DEBD8`, LFSR over 30 bits), sent low half first; the word shifted
///   out during word N answers word N-1 (init check `0x800DF62C` rotates the receive buffers by one).
/// - configuration at start-up (`0x800DEC0A`, config `0x8013AEF0`/`0x8013AF18`): words 0x00400010,
///   0x01030101, 0x00004102..0x00004105, then 0x07FFFFF6 (status read with all data bits set) and 7.
///   The reply to each of the first six must equal it in bits 26:0, otherwise error bits 0..5 of
///   `0xD0008E14+m`.
/// - status read: register 6 (`cfg + 0x18`), answered with bits 3:0 = 6 (checked, error 0x40 at
///   `0x800DF750`), bits 19:4 one fault nibble per channel (bit 0 / 1 / 3 / 2 debounced to regulator
///   status 2 / 2 / 3 / 2 at `0x800DFE3E`), bits 21:20 (`0x800DFDE0`), bits 26:22 (`0x800DFD16`,
///   `0x800DFD54`, `0x800DFD8C`); every set bit is a fault, 0 = healthy. A word of all ones (no chip
///   answering) leaves the module diagnosis in state 9 with every regulator status 4 (`0x800E0404`),
///   and then the actuator measurement table `0xD0007404` is never refreshed (`0x800C255E`) — the
///   firmware's sub-gearbox pressure `0xD0005188` stays 0 and no shift is ever released.
///
/// Modelled: a healthy chip (no faults, coils connected); fault bits from coil electrics are not
/// modelled (UNVERIFIED bit meanings). Replies carry no CRC (the firmware does not check it).
#[derive(Clone, Copy, Debug, Default)]
pub struct ValveDriver {
    reply: u32,
    half: u8,
    req: u32,
    pub words: u64,
}

impl ValveDriver {
    /// One 16-bit SSC frame while the chip is selected; returns the 16 MISO bits.
    fn xfer16(&mut self, tx: u32) -> u32 {
        let out = if self.half == 0 { self.reply & 0xFFFF } else { self.reply >> 16 };
        if self.half == 0 {
            self.req = tx & 0xFFFF;
            self.half = 1;
        } else {
            self.req |= (tx & 0xFFFF) << 16;
            self.half = 0;
            self.words += 1;
            self.reply = if self.req & 0xF == 6 { 6 } else { self.req & 0x07FF_FFFF };
        }
        out
    }
}

/// Port P5 (OUT, OMR) of the pump driver pins: P5.3 bridge enable, P5.9 commutation (see [`pump`]).
pub const P5_BASE: u32 = 0xF000_1100;
/// GPTA0 input line of the pump driver's zero-crossing output (GTC12, VERIFIED-T routing).
pub const PUMP_ZC_LINE: u8 = 47;
/// LTCA2 cell whose output is the bridge PWM (last cell of the LTC26..LTC30 chain, 0x800DA96C).
pub const PUMP_PWM_LTC: u8 = 30;

/// External 32-bit SPI device on SSC1 / CS P2.8 that the safety software (0x80041000..0x80045FFF)
/// uses as the external monitor of the valve power stages. The chip is UNIDENTIFIED (no datasheet);
/// everything below is derived from how the firmware drives and checks it:
/// - frame (VERIFIED-S, check 0x800DD8F6, dispatch 0x800DD78C): bit 0 = 1 read / 0 write, bits 5:1
///   register, bits 29:6 data, bits 31:30 a 2-bit counter the device advances with every reply; the
///   word shifted out during frame N answers frame N-1 (address, R/W bit, written data for writes,
///   register content for reads); all-ones / all-zero replies are rejected.
/// - identification (VERIFIED-S, 0x80042010 compares with the constants at 0x800460E0): after the
///   MCU writes 0xF60D07 to registers 15 and 19 they read back 0xF60D07, and register 14 (command
///   register: the MCU writes keyed one-hot commands 0x6D7502, 0xA1FE01, 0xA2FD02, 0xA3FB04,
///   0xA4F708, 0xA57F10) reads bit 2 = 1 once configured; a register 14 read-back of the written
///   value (bit 2 = 0) is answered with safety error 9 and a software reset (3x -> shut-off latched).
///   Modelled: written data with bit 2 set. UNVERIFIED meaning of the bit.
/// - register 8 (read once at start-up into 0xD00089DE): bits 5:4 = 10b expected by the shut-off
///   path test (0x800428EA, step 9; otherwise error 2). Modelled as a constant 0x20 (UNVERIFIED).
/// - two question/answer watchdog channels (VERIFIED-S): status registers 17 / 20 (read every cycle,
///   decoded at 0x800DD610: data bits 11:8 question, 7:4 error count, 3:1, 12, bit 0 = channel OK),
///   answer registers 18 / 21. The firmware answers `table[question]` with the table at
///   0x80046188, tracks that the question changes and that all 16 questions occur (0x8004354A /
///   0x80043604), and in its forced-failure mode sends wrong answers until the error count reaches 7
///   (0x80043894). Its shut-off path test (group 6 steps 0x0E..0x24, 0x80042998..0x80042CEA) sends
///   one inverted answer on one channel (0x8004342A) per phase and expects (power-stage supplies
///   `ADC[30]`/`ADC[31]` measured against 0.8 x KL30 at 0x80044838):
///   `ch0` wrong after 3 correct -> `ch0` bit 0 = 0, supplies off; `ch0` correct + `ch1` wrong (4th
///   answer) -> `ch0` bit 0 = 1, supplies ON; both correct, `ch1` wrong -> `ch1` bit 0 = 0, supplies off;
///   `ch0` wrong + `ch1` correct -> `ch1` bit 0 = 1, supplies ON.
///   An error counter per channel that starts at 8 after power-up, rises by 2 per wrong and falls by
///   1 per correct answer, with the supplies off while any counter is >= 7, is the simplest rule
///   that reproduces all four expectations and the firmware's threshold 7; bit 0 = last answer
///   correct; the question advances after a correct answer (all 16 in turn). Initial value, steps
///   and question order are fitted to these firmware expectations: UNVERIFIED (no datasheet).
/// - power-stage enable inputs from the MCU (VERIFIED-S): the safety software drives P3.9 = 1 /
///   P3.10 = 0 to enable (0x80043AD2 -> 0x800447B8) and P3.9 = 0 / P3.10 = 1 to disable
///   (0x80043AA8 / 0x80043B3E -> 0x800447CA). Modelled: the supply switch closes only with the
///   enable pattern on these pins (UNVERIFIED wiring: inputs of this device or of the switch).
/// - the MCU's own supply enable P1.13 (VERIFIED-S, supply test 0x80045972: P1_OMR clears P1.13
///   at 0x800459BE and expects both supplies off (0x80045A18, else error 0x1C), sets it again at
///   0x80045A30 and expects both on (0x80045A76)): an AND input of the same switch.
///
/// Registers not listed hold what the MCU last wrote (power-up 0).
#[derive(Clone, Debug, Default)]
pub struct MonitorDevice {
    /// Answer on the bus (otherwise MRST stays pulled up = all ones, as without a slave model).
    pub attached: bool,
    pub regs: [u32; 32],
    /// Reply prepared for the next frame.
    reply: u32,
    counter: u32,
    /// 16-bit half being shifted (0 = low half) and the request word being assembled.
    half: u8,
    req: u32,
    pub frames: u64,
    /// Watchdog channels 0 (regs 17/18) and 1 (regs 20/21).
    pub wd: [QaChannel; 2],
    /// Levels of the MCU's power-stage outputs: bit 0 P3.9 (enable), bit 1 P3.10 (disable), bit 2
    /// P1.13 (supply enable).
    pub pins: u8,
    /// Diagnostic: (request, reply sent with it, pins) of the first frames (bounded).
    pub log: Vec<(u32, u32, u8)>,
}

/// Correct answer per watchdog question, as the firmware computes it (table 0x80046188).
pub const QA_ANSWERS: [u8; 16] = [0x0E, 0x0A, 0x06, 0x02, 0x0F, 0x0B, 0x07, 0x03, 0x0C, 0x08, 0x04, 0x00, 0x0D, 0x09, 0x05, 0x01];

/// Watchdog error counter after power-up, its steps and the shut-off threshold (see
/// [`MonitorDevice`]; threshold 7 = the firmware's own limit at 0x800438AE, the rest UNVERIFIED).
pub const QA_ERRORS_INIT: u8 = 8;
pub const QA_ERRORS_WRONG: u8 = 2;
pub const QA_ERRORS_OFF: u8 = 7;

#[derive(Clone, Copy, Debug)]
pub struct QaChannel {
    pub question: u8,
    pub errors: u8,
    pub ok: bool,
    pub answers: u64,
}

impl Default for QaChannel {
    fn default() -> Self {
        Self { question: 0, errors: QA_ERRORS_INIT, ok: false, answers: 0 }
    }
}

impl QaChannel {
    fn answer(&mut self, a: u32) {
        self.answers += 1;
        if a & 0xF == QA_ANSWERS[self.question as usize] as u32 {
            self.errors = self.errors.saturating_sub(1);
            self.ok = true;
            self.question = (self.question + 1) & 0xF;
        } else {
            self.errors = (self.errors + QA_ERRORS_WRONG).min(15);
            self.ok = false;
        }
    }

    fn status(&self) -> u32 {
        (self.question as u32) << 8 | (self.errors as u32) << 4 | self.ok as u32
    }
}

impl MonitorDevice {
    /// The valve power-stage supply switch of the device is closed (`pins`: P3.9 / P3.10 levels).
    pub fn outputs_enabled(&self) -> bool {
        !self.attached || (self.pins == 0b101 && self.wd.iter().all(|c| c.errors < QA_ERRORS_OFF))
    }

    fn read_reg(&self, addr: u32) -> u32 {
        match addr {
            8 => 0x20,
            14 => self.regs[14] | 4,
            17 => self.wd[0].status(),
            20 => self.wd[1].status(),
            _ => self.regs[addr as usize],
        }
    }

    /// One 16-bit SSC frame while the chip is selected; returns the 16 MISO bits.
    fn xfer16(&mut self, tx: u32) -> u32 {
        let out = if self.half == 0 { self.reply & 0xFFFF } else { self.reply >> 16 };
        if self.half == 0 {
            self.req = tx & 0xFFFF;
            self.half = 1;
        } else {
            self.req |= (tx & 0xFFFF) << 16;
            self.half = 0;
            self.frames += 1;
            if self.log.len() < 4096 {
                self.log.push((self.req, self.reply, self.pins));
            }
            let (read, addr, data) = (self.req & 1 == 1, (self.req >> 1) & 0x1F, (self.req >> 6) & 0xFF_FFFF);
            if !read {
                self.regs[addr as usize] = data;
                match addr {
                    18 => self.wd[0].answer(data),
                    21 => self.wd[1].answer(data),
                    _ => {}
                }
            }
            let d = if read { self.read_reg(addr) } else { data } & 0xFF_FFFF;
            self.counter = (self.counter + 1) & 3;
            self.reply = self.counter << 30 | d << 6 | addr << 1 | read as u32;
        }
        out
    }

    /// Chip select seen high between frames: the next frame starts a new word.
    fn deselect(&mut self) {
        self.half = 0;
    }
}

// ---- DMA (Spi job start 0x800E5B02: channel n regs at 0xF0003C80 + 0x20n: CHCR +4, CHICR +8,
// ADRCR +0xC, SADR +0x10, DADR +0x14; STREQ 0xF0003C18, HTREQ 0xF0003C1C, INTCR 0xF0003C58).
// SSCk RX requests drive channel k, SSCk TX requests channel k+2 (0x800E5B32..0x800E5B42). ----
pub const DMA_BASE: u32 = 0xF000_3C00;
const DMA_CH: u32 = 0xF000_3C80;
pub const DMA_SRC0: u32 = 0xF000_3EFC;

// ---- ADC (driver 0x800E1A50..0x800E2810): kernel k at 0xF0101000 + 0x400k. Request source s
// (1 = auto-scan kernel 0, 3 = kernel 1, group table 0x8013B134/0x8013B150): source control
// +0x10+4s, CRPR (channel mask) block +0x90+0x20(s>>1), CRMR +0x98+0x20(s>>1) (bit 9 = load
// event, bits 1:0 ENGT), event flags +0x70 (bit 16+s, polled by the ISRs 0x800E22A2/0x800E2310),
// flag clear +0x74, CHCTRx +0x100 (bits 15:12 result register), INPCRx +0x50 (bits 9:8 DW: 0 =
// 10 bit, 1 = 12 bit, 2 = 8 bit; firmware right-shift table 0x8013B0F4 = [2,0,4] => results are
// left-aligned in 12 bits), RESRr +0x180, GLOBSTR +0x38 (busy bits 1:0, polled 0x800E2602). ----
pub const ADC_BASE: [u32; 2] = [0xF010_1000, 0xF010_1400];
/// Service request nodes of the ADC: source s raises SRC at `0xF01013FC - 4*s`. Evidence: the
/// firmware configures 0xF01013F8 = SRPN 0x22 (ISR 0x800E22A2 handles kernel 0 source 1) and
/// 0xF01013F0 = SRPN 0x21 (ISR 0x800E2310 handles kernel 1 source 3) at 0x800E3FB4; both kernels
/// were programmed with the same event node register value (+0x3C = 0x0A = sources 1 and 3).
/// UNVERIFIED: the hardware routing rule itself (no TC1784 manual available).
pub const ADC_SRC_NODES: [u32; 6] = [0xF010_13FC, 0xF010_13F8, 0xF010_13F4, 0xF010_13F0, 0xF010_13EC, 0xF010_13E8];
/// Conversion time per channel incl. sampling (UNVERIFIED; INPCR0.STC = 10 at fADC ~ fFPI).
pub const ADC_CONV_S_PER_CHANNEL: f64 = 1.0e-6;
pub const ADC_VAREF: f64 = 5.0;

/// FADC SRCs (0x800E401E..: 0xF01004F0..0xF01004FC, SRPN 0x23 at 0xF01004F8): SRC3..SRC0, service
/// request line n -> `0xF01004FC - 4n` (TC1796 UM Table 18-28 / 26-11).
const FADC_SRCS: [u32; 4] = [0xF010_04F0, 0xF010_04F4, 0xF010_04F8, 0xF010_04FC];
/// FADC analog inputs (FAINxP, x = channel) as driven by the DQ200 board. The firmware converts
/// channel 0 and channel 1 (single-ended FAINxP - VFAREF/2, gain 2; channels 2/3 powered down at
/// 0x800ED498) every 167 us and its SR1 ISR stores `RCH0 -> ADC[32]`, `RCH1 -> ADC[34]` (0x800ED4E0).
/// - Channel 0: current-sense output of the pump motor driver (HYP, strong: the ISR feeds it to the
///   accumulator 0x800D9BBE of the pump-driver module 0x800D9xxx, which averages it in two states
///   of a bit the driver toggles in the register image of the CS P3.6 SPI device (0x800D9C12) and
///   computes (mean_on - mean_off) * 10 * 3300 / 1023 (config 0x8013ABE0) = offset-compensated
///   amplifier output; the pump code (0x800EEC40) divides it by 10 into 0xD000B2F8).
/// - Channel 1: board rail checked by the safety software (check 7, 0x80045366: window 764..1023,
///   substitute 0x37D when KL30 is low). Identity UNVERIFIED; driven at the substitute (nominal) code.
pub const FADC_RAIL_CH1_V: f32 = (0x37D as f64 / 1024.0 * fadc::VFAREF) as f32;

/// SRC registers of TC1784 modules that the core's TC1791 table does not contain.
pub fn is_tc1784_src(a: u32) -> bool {
    ADC_SRC_NODES.contains(&a)
        || FADC_SRCS.contains(&a)
        || SSC_BASE.iter().any(|&b| (b + 0xF4..=b + 0xFC).contains(&a) && a & 3 == 0)
}

#[derive(Default, Clone)]
pub struct Ssc {
    /// Received byte shifted in by the last completed transfer.
    pub rb: u32,
    /// Byte currently in the shift register and its completion time (instructions).
    pub shifting: Option<(u32, u64)>,
    /// Transmit buffer written while the shifter is busy.
    pub tb_full: Option<u32>,
    pub frames: u64,
}

#[derive(Default, Clone, Copy)]
pub struct DmaCh {
    pub tcount: u32,
    pub sadr: u32,
    pub dadr: u32,
    pub moves: u64,
}

#[derive(Default)]
pub struct Dma {
    pub ch: [DmaCh; 8],
    /// HTREQ enabled channels (bits 7:0).
    pub hw_enabled: u32,
    pub intsr: u32,
    /// Moves to/from addresses this model does not reach (diagnostic).
    pub bus_errors: u64,
}

#[derive(Default, Clone, Copy)]
pub struct Adc1784 {
    /// (source, completion instruction count) of the running scan per kernel.
    pub running: [Option<(u32, u64)>; 2],
    pub scans: [u64; 2],
}

// ---- GPTA: GPTA0 @ 0xF0001800 + LTCA2 @ 0xF0002800 (no GPTA1), see `Gpta::tc1784`. Clock from the
// firmware's driver 0x800E4348: GPTA0_CLC = 0, GPTA0_FDR = 0x82AB (DM = 2 fractional, STEP = 683)
// -> fGPTA = fFPI * 683 / 1024 = 40.02 MHz at fFPI 60 MHz (the firmware's 3 kHz solenoid PWM period
// 0x341A + 2 = 13340 ticks and 6 kHz ADC trigger 0x1A0C + 2 = 6670 ticks are round at 40 MHz). ----
pub const GPTA0_BASE: u32 = crate::gpta::GPTA0_BASE;
/// LTC cells whose output transitions trigger the gated ADC scans (kernel 0: LTC58, kernel 1:
/// LTC60, both reset-timer/compare pairs programmed by the ADC driver 0x800E2692). UNVERIFIED: the
/// GPTA -> ADC trigger wiring of the TC1784 (no manual available).
const ADC_TRIGGER_LTC: [u8; 2] = [58, 60];

/// Valve PWM outputs in the order of the firmware's Pwm channels 0..7 (channel config table
/// 0x8013B884, 16-byte entries, word 1 bits 6:0 = GPTA0 LTC output cell, bits 20:19 = kernel 0):
/// the output cell of each 3 kHz generator (VERIFIED-S).
pub const VALVE_PWM_CELLS: [u8; 8] = [40, 15, 32, 20, 28, 44, 36, 24];
/// Current-sense ADC buffer index of each Pwm channel. Evidence (VERIFIED-S): the 3 kHz notification of
/// Pwm channel 0 (0x800EB430) runs, per valve, `current = 0x800ED908(k)`, PI controller
/// `0xD4000000(i, current)` and `Pwm_SetDutyCycle 0xD4000730(ch, duty)` with
/// (k, i, ch) = (0,0,5) (1,1,3) (4,2,0) (5,3,7) (6,4,6) (7,5,2) (2,6,1) (3,7,4); `0x800ED8C0` converts
/// `ADC[table 0x8013CF24[k]] = [28, 5, 7, 2, 27, 4, 29, 0][k]` into the current of sense k.
pub const VALVE_SHUNT_ADC: [usize; 8] = [27, 7, 0, 5, 2, 28, 29, 4];
/// Pin volts per coil ampere of the current-sense amplifiers: the firmware converts a 12-bit code
/// to 0.1 mA as `(code - 0) * 2501 >> 9` (offsets/gains copied from 0x8013CEB8/0x8013CEC8 by
/// 0x800ED780), i.e. 0.4885 mA per code = 2.4996 V/A at VAREF 5 V. The 0.1 mA unit is inferred from
/// the set-points the firmware commands (~9800 = 0.98 A, saturation 16000 = 1.6 A; UNVERIFIED unit).
pub const SHUNT_V_PER_A: f64 = 10_000.0 * 512.0 / 2501.0 * ADC_VAREF / 4095.0;
/// Port outputs read back on analog inputs: (port OUT register, pin, ADC buffer index). VERIFIED-S:
/// the safety software drives P2.1 from its shut-off state (0x80044EC6: P2_OMR set while the state
/// word 0xD001FF58 = 0x2AAAAAAA, reset otherwise) and checks the level on `ADC[23]` (see
/// [`ADC_TEST_PULLDOWN`]). What P2.1 switches on the board (shut-off path) is UNVERIFIED.
pub const OUTPUT_READBACKS: [(u32, u32, usize); 2] = [(0xF000_0E00, 1, 23), (0xF000_0E00, 1, 3)];
/// Port pins of the valve outputs: the eight pins the safety software forces to input while its
/// shut-off state is active (0x80045AF0: PC nibbles of P0_IOCR bytes 0xF0000C11/13/15/19/1B/1D/1F =
/// P0.1/3/5/9/11/13/15 and P1_IOCR8 byte 0xF0000D18 = P1.8, VERIFIED-S). (IOCR byte address, bit
/// offset of the 4-bit PC field). Which pin carries which Pwm channel is UNVERIFIED, so a valve is
/// driven only while all eight pins are outputs (true for the shut-off and for a uniform setup).
pub const VALVE_OUTPUT_PINS: [(u32, u32); 8] =
    [(0xF000_0C10, 12), (0xF000_0C10, 28), (0xF000_0C14, 12), (0xF000_0C18, 12), (0xF000_0C18, 28), (0xF000_0C1C, 12), (0xF000_0C1C, 28), (0xF000_0D18, 4)];
/// ADC kernel register +0x34 bit 4, read as a pull-down on the readback input. The safety test
/// (0x80044EC6 stimulus, 0x80044D22/0x80044F44 measurements, limits 0x8004603C) runs three phases:
/// P2.1 low with bit 4 clear -> `ADC[23]` in 1921..2175 (mid-scale); P2.1 high -> 3968..4095;
/// P2.1 low with bit 4 set (ADC1+0x34 = 0x30) -> 0..127. Modelled as a divider node at VAREF/2
/// that P2.1 pulls to VAREF and bit 4 discharges to 0 V. UNVERIFIED circuit (no schematic, no TC1784
/// ADC manual), inferred from these three expectations.
pub const ADC_TEST_PULLDOWN: u32 = 1 << 4;
/// Freewheel diode forward voltage across a switched-off coil. UNVERIFIED (typical silicon diode).
pub const COIL_FREEWHEEL_V: f64 = 0.7;

/// Valve coil driven by a low-side PWM switch: series R-L from the supply while the output is on,
/// freewheeling through the diode while it is off (the current decays to zero and stays there).
/// Integrated exactly between output transitions, so the sensed current carries the real PWM
/// ripple. R and L come from the plant (coil temperature), see `Dq200TcuBoard::set_coil_params`.
#[derive(Clone, Copy, Debug)]
pub struct CoilLoad {
    pub r_ohm: f64,
    pub l_h: f64,
    pub i_a: f64,
    /// Low-side switch conducting (PWM cell output high and the output pin driven).
    pub on: bool,
    /// Level of the GPTA output cell.
    pub cell_level: bool,
    /// Current averaged over the last [`COIL_MEAN_S`] (exact integral of the R-L response, PWM ripple
    /// removed): what the valve's armature force follows (the plant's spool model, `TcuOutputs::
    /// solenoid_current_a`).
    pub i_mean_a: f64,
    /// fGPTA tick of the last update.
    t: u64,
}

/// Averaging time of [`CoilLoad::i_mean_a`]: a few 3 kHz PWM periods, well below the spool's
/// mechanical response (`simemu_physics::hydraulics::SolenoidHw::spool_hz`, 40 Hz second order).
pub const COIL_MEAN_S: f64 = 1e-3;

impl Default for CoilLoad {
    fn default() -> Self {
        // UNVERIFIED defaults (typical automotive proportional valve: 5 Ω, 12 mH), replaced by the
        // plant's coil parameters when a plant is attached.
        Self { r_ohm: 5.0, l_h: 0.012, i_a: 0.0, on: false, cell_level: false, i_mean_a: 0.0, t: 0 }
    }
}

impl CoilLoad {
    pub fn advance(&mut self, t: u64, f_gpta: f64, v_supply: f64) {
        if t <= self.t || f_gpta <= 0.0 {
            self.t = self.t.max(t);
            return;
        }
        let dt = (t - self.t) as f64 / f_gpta;
        self.t = t;
        let u = if self.on { v_supply.max(0.0) } else { -COIL_FREEWHEEL_V };
        let r = self.r_ohm.max(1e-3);
        let tau = self.l_h.max(1e-6) / r;
        let i_inf = u / r;
        let i0 = self.i_a;
        let decay = (-dt / tau).exp();
        let i1 = i_inf + (i0 - i_inf) * decay;
        // Mean over the interval: i_inf + (i0 - i_inf) tau / dt (1 - e^{-dt/tau}); while freewheeling
        // the current stops at zero (time to zero t0 = tau ln(1 - i0 / i_inf)).
        let mean = if i1 >= 0.0 {
            i_inf + (i0 - i_inf) * tau / dt * (1.0 - decay)
        } else {
            let t0 = tau * (1.0 - i0 / i_inf).ln();
            (i_inf * t0 + (i0 - i_inf) * tau * (1.0 - (-t0 / tau).exp())) / dt
        };
        self.i_a = i1.max(0.0);
        let w = (-dt / COIL_MEAN_S).exp();
        self.i_mean_a = mean.max(0.0) + (self.i_mean_a - mean.max(0.0)) * w;
    }
}

/// A periodic digital signal at a GPTA input line (sensor output as seen by the pin).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PinSignal {
    /// Period in seconds; `<= 0` / non-finite = no edges (line held at `idle`).
    pub period_s: f64,
    /// High-time fraction 0..1 (0 or 1 = constant level).
    pub duty: f64,
    pub idle: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct PinGen {
    sig: PinSignal,
    level: bool,
    /// GPTA tick of the next edge (fractional, so periods do not drift); None = not running.
    next: Option<f64>,
}

/// Measured output of a GPTA cell (ticks of fGPTA).
#[derive(Clone, Copy, Debug, Default)]
pub struct CellPwm {
    pub level: bool,
    pub last_rise: Option<u64>,
    pub last_edge: u64,
    pub high: u64,
    pub period: u64,
    pub edges: u64,
}

impl CellPwm {
    /// Duty 0..1 at GPTA tick `now`: the last measured period while edges keep coming, otherwise the
    /// static level.
    pub fn duty(&self, now: u64) -> f64 {
        if self.period > 0 && now.saturating_sub(self.last_edge) <= 2 * self.period {
            (self.high as f64 / self.period as f64).clamp(0.0, 1.0)
        } else if self.level {
            1.0
        } else {
            0.0
        }
    }
}

pub struct Tc1784Io {
    pub eeprom: St95640,
    pub ssc: [Ssc; 2],
    pub dma: Dma,
    pub adc: Adc1784,
    /// Pin voltage per ADC kernel/channel.
    pub adc_volts: [[f32; 16]; 2],
    /// Last (tx, rx) SPI frames per [`SPI_DEVICES`] entry.
    pub spi_log: [std::collections::VecDeque<(u32, u32)>; 6],
    pub monitor: MonitorDevice,
    /// Valve driver chips of valve modules 0 / 1 ([`ValveDriver`]).
    pub valve_drivers: [ValveDriver; 2],
    /// Frames shifted while no known chip select was low: (tx, P2 OUT, P3 OUT) per SSC (diagnostic).
    pub spi_other: [std::collections::VecDeque<(u32, u32, u32)>; 2],
    eeprom_cs_low: bool,
    /// Diagnostic register-write trace: writes inside `trace_range` are logged as
    /// (block pc, address, value, lane mask) into `trace_log` (bounded).
    pub trace_range: Option<(u32, u32)>,
    pub trace_log: Vec<(u32, u32, u32, u32)>,
    pub gpta: crate::gpta::Gpta,
    /// Instruction count up to which the GPTA has been advanced, and the CPU-cycle remainder.
    gpta_at: u64,
    gpta_rem: u64,
    /// Signal generators per GPTA input line IN0..IN55.
    pins: [PinGen; 56],
    /// Earliest `PinGen::next` of the running generators as of the last full scan of `pin_edges`, and
    /// "a generator changed since" (`set_pin`, reset): while clean, no edge comes before `pin_next`.
    pin_next: f64,
    pins_dirty: bool,
    /// Edges of the interval being advanced and drained GPTA requests/outputs (reused buffers).
    edge_buf: Vec<crate::gpta::InputEdge>,
    req_buf: Vec<u32>,
    out_buf: Vec<crate::gpta::OutputEvent>,
    /// Output transitions per (kernel, LTC: 64 + cell / GTC: cell).
    pub cell_out: std::collections::HashMap<(u8, u8), CellPwm>,
    /// Copy of the `cell_out` entry of the pump bridge PWM cell (LTCA2 [`PUMP_PWM_LTC`]), read on every
    /// GPTA update.
    pump_pwm: CellPwm,
    /// Valve coils on the Pwm channels ([`VALVE_PWM_CELLS`]) and their supply voltage.
    pub coils: [CoilLoad; 8],
    /// Battery-side supply of the valve power stages; the coils and the supply sense inputs see it
    /// only while the monitor device's supply switch is closed ([`Tc1784Io::valve_supply_v`]).
    pub coil_supply_v: f64,
    /// ADC pin voltage of the two power-stage supply senses `ADC[30]`/`ADC[31]` with the switch closed.
    pub valve_supply_pin_v: f32,
    pub fadc: fadc::Fadc,
    /// V401 pump: driver chip on CS P3.6 + motor + gear-pump shaft.
    pub pump: pump::BldcPump,
}

impl Default for Tc1784Io {
    fn default() -> Self {
        Self::new(&[])
    }
}

impl Tc1784Io {
    pub fn new(eeprom_image: &[u8]) -> Self {
        Self {
            eeprom: St95640::new(eeprom_image),
            ssc: Default::default(),
            dma: Default::default(),
            adc: Default::default(),
            adc_volts: [[0.0; 16]; 2],
            spi_log: Default::default(),
            monitor: MonitorDevice { attached: true, ..Default::default() },
            valve_drivers: Default::default(),
            spi_other: Default::default(),
            eeprom_cs_low: false,
            trace_range: None,
            trace_log: Vec::new(),
            gpta: crate::gpta::Gpta::tc1784(40_000_000),
            gpta_at: 0,
            gpta_rem: 0,
            pins: [PinGen::default(); 56],
            pin_next: f64::INFINITY,
            pins_dirty: true,
            edge_buf: Vec::new(),
            req_buf: Vec::new(),
            out_buf: Vec::new(),
            cell_out: Default::default(),
            pump_pwm: CellPwm::default(),
            coils: [CoilLoad::default(); 8],
            coil_supply_v: 0.0,
            valve_supply_pin_v: 0.0,
            fadc: fadc::Fadc::default(),
            pump: pump::BldcPump::default(),
        }
    }

    /// Supply voltage behind the monitor device's power-stage switch (see [`MonitorDevice`]).
    pub fn valve_supply_v(&self) -> f64 {
        if self.monitor.outputs_enabled() { self.coil_supply_v } else { 0.0 }
    }

    /// Advance every valve coil to the current GPTA time and put its sensed current on the ADC pin.
    /// `driven`: the valve output pins are configured as outputs ([`VALVE_OUTPUT_PINS`]).
    fn sense_coils(&mut self, driven: bool) {
        let (now, f) = (self.gpta.now(), self.gpta.f_gpta_hz as f64);
        let v = self.valve_supply_v();
        for (c, &adc) in self.coils.iter_mut().zip(VALVE_SHUNT_ADC.iter()) {
            c.advance(now, f, v);
            c.on = c.cell_level && driven;
            self.adc_volts[adc / 16][adc % 16] = (c.i_a * SHUNT_V_PER_A).min(ADC_VAREF) as f32;
        }
    }

    /// Drive GPTA input line `line` (IN0..IN55) with a periodic signal from now on.
    pub fn set_pin(&mut self, line: u8, sig: PinSignal) {
        let Some(g) = self.pins.get_mut(line as usize) else {
            return;
        };
        if g.sig != sig {
            let running = sig.period_s.is_finite() && sig.period_s > 0.0 && sig.duty > 0.0 && sig.duty < 1.0;
            // a running generator keeps its phase; if in the low phase between pulses and the period
            // shortened (rapid acceleration from creep/cranking), rescale the remaining low wait so
            // the sensor does not stay silent for a multi-hundred-millisecond old period.
            if !running || g.next.is_none() {
                g.next = None;
            } else if !g.level
                && g.sig.period_s.is_finite()
                && g.sig.period_s > 0.0
                && sig.period_s < g.sig.period_s
            {
                if let Some(next) = g.next {
                    let now = self.gpta.now() as f64;
                    let rem = (next - now).max(0.0);
                    let ratio = (sig.period_s * (1.0 - sig.duty))
                        / (g.sig.period_s * (1.0 - g.sig.duty)).max(1e-12);
                    g.next = Some(now + (rem * ratio).max(1.0));
                }
            }
            g.sig = sig;
            self.pins_dirty = true;
        }
    }

    /// Measured output of a GPTA cell (`ltc` selects LTC/GTC).
    pub fn cell_pwm(&self, kernel: u8, ltc: bool, cell: u8) -> Option<CellPwm> {
        if cell >= 64 {
            return None;
        }
        self.cell_out.get(&(kernel, cell + if ltc { 64 } else { 0 })).copied()
    }

    /// Edges of all pin generators in the GPTA ticks (from, to], into `edge_buf`. Unless a generator
    /// changed, nothing happens before `pin_next` (the scan would emit nothing and keep every state).
    #[inline]
    fn pin_edges(&mut self, from: u64, to: u64, f_gpta: f64) {
        self.edge_buf.clear();
        if self.pins_dirty || self.pin_next <= to as f64 {
            self.scan_pins(from, to, f_gpta);
        }
    }

    fn scan_pins(&mut self, from: u64, to: u64, f_gpta: f64) {
        const MAX_PIN_EDGES_PER_SCAN: usize = 16_384;
        self.pins_dirty = false;
        if !f_gpta.is_finite() || f_gpta <= 0.0 {
            return;
        }
        let edges = &mut self.edge_buf;
        let to_f = to as f64;
        let mut first = f64::INFINITY;
        for (line, g) in self.pins.iter_mut().enumerate() {
            let s = g.sig;
            let running = s.period_s.is_finite() && s.period_s > 0.0 && s.duty > 0.0 && s.duty < 1.0;
            if !running {
                let level = if s.period_s.is_finite() && s.period_s > 0.0 { s.duty >= 1.0 } else { s.idle };
                if level != g.level {
                    g.level = level;
                    edges.push(crate::gpta::InputEdge { t: to, kernel: 0, line: line as u8, level });
                }
                continue;
            }
            let p = s.period_s * f_gpta;
            if !p.is_finite() || p <= 0.0 {
                continue;
            }
            let mut next = g.next.unwrap_or(from as f64 + 1.0);
            let mut emitted = 0usize;
            while next <= to_f {
                if emitted >= MAX_PIN_EDGES_PER_SCAN {
                    let full_p = p.max(2.0);
                    let skip = ((to_f - next) / full_p).floor().max(1.0);
                    let cand = next + skip * full_p;
                    next = if cand > to_f { cand } else { to_f + full_p.max((to_f.abs() * f64::EPSILON).max(1.0)) };
                    break;
                }
                g.level = !g.level;
                edges.push(crate::gpta::InputEdge {
                    t: (next as u64).max(from.saturating_add(1)),
                    kernel: 0,
                    line: line as u8,
                    level: g.level,
                });
                let step = (if g.level { p * s.duty } else { p * (1.0 - s.duty) }).max(1.0);
                let cand = next + step;
                next = if cand > next { cand } else { next + (next.abs() * f64::EPSILON).max(1.0) };
                emitted += 1;
            }
            g.next = Some(next);
            first = first.min(next);
        }
        self.pin_next = first;
        edges.sort_by_key(|e| e.t);
    }
}

impl Tc1791HardwareState {
    #[inline]
    fn io_now(&self) -> u64 {
        self.total_instructions + self.quantum_executed
    }

    #[inline]
    fn io_now_s(&self) -> f64 {
        self.io_now() as f64 / self.scu.fcpu_hz().max(1) as f64
    }

    #[inline]
    fn s_to_insns(&self, s: f64) -> u64 {
        ((s * self.scu.fcpu_hz() as f64).ceil() as u64).max(1)
    }

    /// SoC reset: on-chip modules return to idle; the external EEPROM keeps its contents and sees
    /// its chip select released (port pins become pulled-up inputs).
    pub(crate) fn tc1784_reset(&mut self) {
        let t = self.io_now_s();
        let io = &mut self.tc1784;
        io.ssc = Default::default();
        io.dma = Default::default();
        io.adc = Default::default();
        io.eeprom.set_cs(false, t);
        io.eeprom_cs_low = false;
        io.gpta = crate::gpta::Gpta::tc1784(40_000_000);
        io.gpta_rem = 0;
        let volts = io.fadc.volts;
        io.fadc = fadc::Fadc::default();
        io.fadc.volts = volts;
        io.cell_out.clear();
        io.pump_pwm = CellPwm::default();
        for c in io.coils.iter_mut() {
            *c = CoilLoad { r_ohm: c.r_ohm, l_h: c.l_h, ..Default::default() };
        }
        for g in io.pins.iter_mut() {
            g.next = None;
            g.level = false;
        }
        io.pins_dirty = true;
        io.pump.power_on_reset();
        io.pump.reset_time(0);
        self.tc1784.gpta_at = self.io_now();
    }

    /// CPU cycles -> fGPTA ticks as (numerator, denominator) from GPTA0_CLC/FDR and the FPI divider
    /// (TC1796 UM eq. 24.6/24.7, fGPTA = fSYS / (1024 - STEP) or fSYS * STEP / 1024); None = stopped.
    fn gpta_ratio(&self) -> Option<(u64, u64)> {
        let (clc, fdr) = self.tc1784.gpta.clock_regs();
        let step = (fdr & 0x3FF) as u64;
        let fpi = self.scu.fpidiv() as u64 + 1;
        match (clc & 1, (fdr >> 14) & 3) {
            (0, 1) => Some((1, fpi * (1024 - step))),
            (0, 2) if step > 0 => Some((step, fpi * 1024)),
            _ => None,
        }
    }

    /// Advance the GPTA to the current instruction count with the pin signals of the interval,
    /// then deliver its service requests and output transitions.
    fn gpta_update(&mut self) {
        let now = self.io_now();
        let d = now.saturating_sub(self.tc1784.gpta_at);
        self.tc1784.gpta_at = now;
        let Some((num, den)) = self.gpta_ratio() else {
            self.tc1784.gpta_rem = 0;
            return;
        };
        let total = self.tc1784.gpta_rem + d * crate::stm_icu::CYCLES_PER_INSN * num;
        let ticks = total / den;
        self.tc1784.gpta_rem = total % den;
        if ticks == 0 {
            return;
        }
        let f = (self.scu.fcpu_hz() * num / den) as f64;
        let io = &mut self.tc1784;
        io.gpta.f_gpta_hz = f as u64;
        let t0 = io.gpta.now();
        io.pin_edges(t0, t0 + ticks, f);
        // V401: the motor runs to the end of the interval; its zero-crossing edges enter IN47.
        let duty = io.pump_pwm.duty(t0);
        io.pump.advance(t0 + ticks, f, io.coil_supply_v, duty);
        if !io.pump.edges.is_empty() {
            io.edge_buf.extend(io.pump.edges.iter().map(|&(t, level)| crate::gpta::InputEdge { t, kernel: 0, line: PUMP_ZC_LINE, level }));
            io.edge_buf.sort_by_key(|e| e.t);
        }
        io.fadc.volts[0][0] = io.pump.sense_v();
        io.gpta.advance(ticks, &io.edge_buf);
        self.gpta_drain();
    }

    #[inline]
    fn gpta_drain(&mut self) {
        if self.tc1784.gpta.pending() {
            self.gpta_deliver();
        }
    }

    fn gpta_deliver(&mut self) {
        let io = &mut self.tc1784;
        let (mut reqs, mut outs) = (std::mem::take(&mut io.req_buf), std::mem::take(&mut io.out_buf));
        io.gpta.drain_into(&mut reqs, &mut outs);
        for &a in &reqs {
            self.src_set_srr(a);
        }
        for &o in &outs {
            let ltc = o.kind == crate::gpta::CellKind::Ltc;
            let m = self.tc1784.cell_out.entry((o.kernel, o.cell + if ltc { 64 } else { 0 })).or_default();
            if o.level {
                if let Some(r) = m.last_rise {
                    m.period = o.t - r;
                }
                m.last_rise = Some(o.t);
            } else if let Some(r) = m.last_rise {
                m.high = o.t - r;
            }
            m.level = o.level;
            m.last_edge = o.t;
            m.edges += 1;
            if ltc && o.kernel == 2 && o.cell == PUMP_PWM_LTC {
                self.tc1784.pump_pwm = *m;
            }
            if let Some(k) = VALVE_PWM_CELLS.iter().position(|&c| ltc && o.kernel == 0 && c == o.cell) {
                // UNVERIFIED polarity: output high = low-side switch conducting (closed-loop check:
                // the firmware's current controllers settle at their set-points with this sense).
                let driven = self.valve_pins_driven();
                let io = &mut self.tc1784;
                let f = io.gpta.f_gpta_hz as f64;
                let v = io.valve_supply_v();
                io.coils[k].advance(o.t, f, v);
                io.coils[k].cell_level = o.level;
                io.coils[k].on = o.level && driven;
            }
            if ltc && o.kernel == 0 {
                if let Some(k) = ADC_TRIGGER_LTC.iter().position(|&c| c == o.cell) {
                    self.adc1784_trigger(k);
                }
            }
        }
        reqs.clear();
        outs.clear();
        (self.tc1784.req_buf, self.tc1784.out_buf) = (reqs, outs);
    }

    fn gpta_access(a: u32) -> bool {
        crate::gpta::Gpta::contains(a) && !(0xF000_2000..0xF000_2800).contains(&a)
    }

    /// All valve output pins configured as outputs (IOCR PC field 1xxx), see [`VALVE_OUTPUT_PINS`].
    pub(crate) fn valve_pins_driven(&self) -> bool {
        VALVE_OUTPUT_PINS.iter().all(|&(a, sh)| (self.seg_f000.get(a) >> sh) & 0x8 != 0)
    }

    /// Side-effect-free peek of a TC1784 register.
    pub(crate) fn tc1784_peek(&self, a: u32) -> Option<u32> {
        if self.variant != BoardVariant::Dq200Tc1784 {
            return None;
        }
        if is_tc1784_src(a) {
            return Some(self.src_get(a));
        }
        if Self::gpta_access(a) {
            return self.tc1784.gpta.peek_word(a);
        }
        if (fadc::FADC_BASE..fadc::FADC_BASE + 0xF0).contains(&a) {
            return Some(self.tc1784.fadc.read(a - fadc::FADC_BASE));
        }
        match a {
            0xF87F_FD14 => Some(self.seg_f87f.get(a) & !1),
            _ if (SSC_BASE[0]..SSC_BASE[1] + 0x100).contains(&a) => {
                let k = ((a - SSC_BASE[0]) >> 8) as usize;
                match a - SSC_BASE[k] {
                    0x24 => Some(self.tc1784.ssc[k].rb),
                    0x28 => {
                        let s = &self.tc1784.ssc[k];
                        Some(self.seg_f000.get(a) & !SSC_STAT_BSY | if s.shifting.is_some() { SSC_STAT_BSY } else { 0 })
                    }
                    _ => None,
                }
            }
            _ if (DMA_CH..DMA_CH + 0x100).contains(&a) => {
                let n = ((a - DMA_CH) >> 5) as usize;
                match (a - DMA_CH) & 0x1F {
                    0x00 => Some((self.seg_f000.get(a) & !0x3FFF) | self.tc1784.dma.ch[n].tcount),
                    0x10 => Some(self.tc1784.dma.ch[n].sadr),
                    0x14 => Some(self.tc1784.dma.ch[n].dadr),
                    _ => None,
                }
            }
            0xF000_3C54 => Some(self.tc1784.dma.intsr),
            0xF000_3C10 | 0xF000_3C18 | 0xF000_3C1C | 0xF000_3C58 => Some(0),
            _ if (DMA_BASE..0xF000_3EE0).contains(&a) => Some(self.seg_f000.get(a)),
            _ if (ADC_BASE[0]..ADC_BASE[1] + 0x400).contains(&a) => {
                let k = ((a - ADC_BASE[0]) >> 10) as usize;
                match a - ADC_BASE[k] {
                    0x38 => Some(self.seg_f000.get(a) & !3 | self.tc1784.adc.running[k].is_some() as u32),
                    0x74 => Some(0),
                    0x98 | 0xB8 => Some(self.seg_f000.get(a) & !(1 << 9)),
                    _ => Some(self.seg_f000.get(a)),
                }
            }
            _ => None,
        }
    }

    /// TC1784 register read; `None` = handled by the shared core.
    pub(crate) fn tc1784_read(&mut self, a: u32) -> Option<u32> {
        if self.variant != BoardVariant::Dq200Tc1784 {
            return None;
        }
        self.tc1784_update();
        if let Some((lo, hi)) = self.tc1784.trace_range {
            if (lo..=hi).contains(&a) && self.tc1784.trace_log.len() < 200_000 {
                self.tc1784.trace_log.push((self.last_tb_pc, a, 0, 0)); // mask 0 = read
            }
        }
        if is_tc1784_src(a) {
            return Some(self.src_get(a));
        }
        if Self::gpta_access(a) {
            return self.tc1784.gpta.read_word(a);
        }
        if (fadc::FADC_BASE..fadc::FADC_BASE + 0xF0).contains(&a) {
            return Some(self.tc1784.fadc.read(a - fadc::FADC_BASE));
        }
        match a {
            0xF87F_FD14 => Some(self.seg_f87f.get(a) & !1), // PMI_CON1 invalidate bits self-clear
            _ if (SSC_BASE[0]..SSC_BASE[1] + 0x100).contains(&a) => {
                let k = ((a - SSC_BASE[0]) >> 8) as usize;
                match a - SSC_BASE[k] {
                    0x24 => Some(self.tc1784.ssc[k].rb),
                    0x28 => {
                        let s = &self.tc1784.ssc[k];
                        Some(self.seg_f000.get(a) & !SSC_STAT_BSY | if s.shifting.is_some() { SSC_STAT_BSY } else { 0 })
                    }
                    _ => None,
                }
            }
            _ if (DMA_CH..DMA_CH + 0x100).contains(&a) => {
                let n = ((a - DMA_CH) >> 5) as usize;
                match (a - DMA_CH) & 0x1F {
                    0x00 => Some((self.seg_f000.get(a) & !0x3FFF) | self.tc1784.dma.ch[n].tcount),
                    0x10 => Some(self.tc1784.dma.ch[n].sadr),
                    0x14 => Some(self.tc1784.dma.ch[n].dadr),
                    _ => None,
                }
            }
            0xF000_3C54 => Some(self.tc1784.dma.intsr),
            0xF000_3C10 | 0xF000_3C18 | 0xF000_3C1C | 0xF000_3C58 => Some(0),
            // other DMA registers: plain registers (keeps the TC1791 DMA model of the core out)
            _ if (DMA_BASE..0xF000_3EE0).contains(&a) => Some(self.seg_f000.get(a)),
            _ if (ADC_BASE[0]..ADC_BASE[1] + 0x400).contains(&a) => {
                let k = ((a - ADC_BASE[0]) >> 10) as usize;
                match a - ADC_BASE[k] {
                    0x38 => Some(self.seg_f000.get(a) & !3 | self.tc1784.adc.running[k].is_some() as u32),
                    0x74 => Some(0),
                    0x98 | 0xB8 => Some(self.seg_f000.get(a) & !(1 << 9)),
                    _ => Some(self.seg_f000.get(a)),
                }
            }
            _ => None,
        }
    }

    /// TC1784 register write; `false` = handled by the shared core.
    pub(crate) fn tc1784_write(&mut self, a: u32, v: u32, m: u32) -> bool {
        if self.variant != BoardVariant::Dq200Tc1784 {
            return false;
        }
        self.tc1784_update();
        if let Some((lo, hi)) = self.tc1784.trace_range {
            if (lo..=hi).contains(&a) && self.tc1784.trace_log.len() < 200_000 {
                self.tc1784.trace_log.push((self.last_tb_pc, a, v, m));
            }
        }
        if is_tc1784_src(a) {
            self.src_write(a, v, m);
            return true;
        }
        if Self::gpta_access(a) {
            if a == 0xF000_1800 && self.scu.endinit_active(a) {
                return true;
            }
            self.tc1784.gpta.write_lanes(a, v, m);
            self.gpta_drain();
            return true;
        }
        if (fadc::FADC_BASE..fadc::FADC_BASE + 0xF0).contains(&a) {
            let off = a - fadc::FADC_BASE;
            // CLC / FDR are ENDINIT-protected (TC1796 UM Table 18-28, write "SV, E")
            if matches!(off, 0x00 | 0x0C) && self.scu.endinit_active(a) {
                return true;
            }
            let now = self.io_now() as f64;
            let srs = self.tc1784.fadc.write(off, v, m, now);
            self.fadc_raise(srs);
            self.fadc_clock_and_advance(now); // CLC/FDR change the clock
            return true;
        }
        let merge = |old: u32| (old & !m) | (v & m);
        match a {
            // EEPROM chip select: let the port model update the latch, then sample the pin.
            _ if a == EEPROM_CS_PORT_BASE || a == EEPROM_CS_PORT_BASE + 4 => {
                if !self.scu_write(a, v, m) {
                    let old = self.seg_f000.get(a);
                    self.seg_f000.set(a, merge(old));
                }
                let low = self.seg_f000.get(EEPROM_CS_PORT_BASE) & (1 << EEPROM_CS_PIN) == 0;
                if low != self.tc1784.eeprom_cs_low {
                    self.tc1784.eeprom_cs_low = low;
                    let t = self.io_now_s();
                    self.tc1784.eeprom.set_cs(low, t);
                }
                true
            }
            // Pump driver pins: let the port model update the latch, then sample P5.3 / P5.9.
            _ if a == P5_BASE || a == P5_BASE + 4 => {
                if !self.scu_write(a, v, m) {
                    let old = self.seg_f000.get(a);
                    self.seg_f000.set(a, merge(old));
                }
                let out = self.seg_f000.get(P5_BASE);
                self.tc1784.pump.set_pins(out & (1 << 3) != 0, out & (1 << 9) != 0);
                true
            }
            _ if (SSC_BASE[0]..SSC_BASE[1] + 0x100).contains(&a) => {
                let k = ((a - SSC_BASE[0]) >> 8) as usize;
                match a - SSC_BASE[k] {
                    0x00 if self.scu.endinit_active(a) => true,
                    0x20 => {
                        self.ssc_tb_write(k, v & m);
                        true
                    }
                    0x24 | 0x28 => true,
                    _ => false,
                }
            }
            _ if (DMA_CH..DMA_CH + 0x100).contains(&a) => {
                let n = ((a - DMA_CH) >> 5) as usize;
                let old = self.seg_f000.get(a);
                self.seg_f000.set(a, merge(old));
                let c = &mut self.tc1784.dma.ch[n];
                match (a - DMA_CH) & 0x1F {
                    0x04 => c.tcount = merge(old) & 0x3FFF, // CHCR.TREL loads the transfer count
                    0x10 => c.sadr = merge(old),
                    0x14 => c.dadr = merge(old),
                    _ => {}
                }
                true
            }
            0xF000_3C10 => {
                // CHRSTR: channel reset completes immediately (polled at 0x800DDAF4), bits read 0
                for (n, ch) in self.tc1784.dma.ch.iter_mut().enumerate() {
                    if v & m & (1 << n) != 0 {
                        ch.tcount = 0;
                    }
                }
                true
            }
            0xF000_3C18 => {
                // STREQ: software transfer request
                for n in 0..8 {
                    if v & m & (1 << n) != 0 {
                        self.dma_move(n);
                    }
                }
                true
            }
            0xF000_3C1C => {
                // HTREQ: ECHx (7:0) enable, DCHx (23:16) disable hardware requests
                let w = v & m;
                self.tc1784.dma.hw_enabled = (self.tc1784.dma.hw_enabled | (w & 0xFF)) & !((w >> 16) & 0xFF);
                true
            }
            0xF000_3C58 => {
                self.tc1784.dma.intsr &= !(v & m & 0xFF);
                true
            }
            _ if (DMA_BASE..0xF000_3EE0).contains(&a) => {
                if a == DMA_BASE && self.scu.endinit_active(a) {
                    return true;
                }
                let old = self.seg_f000.get(a);
                self.seg_f000.set(a, merge(old));
                true
            }
            _ if (ADC_BASE[0]..ADC_BASE[1] + 0x400).contains(&a) => {
                let k = ((a - ADC_BASE[0]) >> 10) as usize;
                self.adc1784_write(k, a, a - ADC_BASE[k], v, m)
            }
            _ => false,
        }
    }

    // ------------------------------------------------------------------ SSC

    fn ssc_insns_per_frame(&self, k: usize) -> u64 {
        let base = SSC_BASE[k];
        let bits = (self.seg_f000.get(base + 0x10) & 0xF) as u64 + 1;
        let br = (self.seg_f000.get(base + 0x14) & 0xFFFF) as u64;
        // shift clock = fFPI / (2 * (BR + 1)); one FPI cycle = FPIDIV + 1 CPU cycles (IPC = 1)
        (bits * 2 * (br + 1) * (self.scu.fpidiv() as u64 + 1)).max(1)
    }

    fn ssc_tb_write(&mut self, k: usize, tb: u32) {
        if self.seg_f000.get(SSC_BASE[k] + 0x10) & (1 << 15) == 0 {
            return; // CON.EN = 0: programming mode, no transfer
        }
        if self.tc1784.ssc[k].shifting.is_some() {
            self.tc1784.ssc[k].tb_full = Some(tb);
        } else {
            self.ssc_start(k, tb);
        }
    }

    /// TB -> shift register: transmit buffer empty (TIR) request.
    fn ssc_start(&mut self, k: usize, tb: u32) {
        let done = self.io_now() + self.ssc_insns_per_frame(k);
        self.tc1784.ssc[k].shifting = Some((tb, done));
        self.stop_requested = true;
        self.src_set_srr(SSC_BASE[k] + 0xF4);
        self.dma_request(k + 2);
    }

    /// Shift complete: exchange with the selected slave, RB update and receive (RIR) request.
    fn ssc_complete(&mut self, k: usize) {
        let Some((tx, _)) = self.tc1784.ssc[k].shifting.take() else { return };
        let bits = (self.seg_f000.get(SSC_BASE[k] + 0x10) & 0xF) + 1;
        let ones = (1u32 << bits) - 1;
        // Slave selected by its chip-select pin (output latch low).
        let dev = SPI_DEVICES.iter().position(|d| d.ssc == k && self.seg_f000.get(d.port) & (1 << d.pin) == 0);
        if k == SPI_DEVICES[SPI_MONITOR].ssc && dev != Some(SPI_MONITOR) {
            self.tc1784.monitor.deselect();
        }
        for (m, &d) in SPI_VALVE_DRIVER.iter().enumerate() {
            if k == SPI_DEVICES[d].ssc && dev != Some(d) {
                self.tc1784.valve_drivers[m].half = 0; // chip select high between words
            }
        }
        let rx = match dev {
            Some(d) if bits == 16 && SPI_VALVE_DRIVER.contains(&d) => {
                self.tc1784.valve_drivers[(d == SPI_VALVE_DRIVER[1]) as usize].xfer16(tx)
            }
            // EEPROM: 8-bit multiples shifted MSB first (SSC0 CON.HB = 1 in every EEPROM job)
            Some(0) if bits.is_multiple_of(8) => {
                let t = self.io_now_s();
                (0..bits / 8).rev().fold(0u32, |acc, i| (acc << 8) | self.tc1784.eeprom.xfer((tx >> (8 * i)) as u8, t) as u32)
            }
            Some(SPI_PUMP) if bits == 16 => self.tc1784.pump.xfer16(tx),
            Some(SPI_MONITOR) if bits == 16 && self.tc1784.monitor.attached => {
                self.monitor_pins();
                self.tc1784.monitor.xfer16(tx)
            }
            _ => ones, // no slave model: MRST pulled up
        };
        let s = &mut self.tc1784.ssc[k];
        s.rb = rx;
        s.frames += 1;
        if let Some(d) = dev {
            let log = &mut self.tc1784.spi_log[d];
            log.push_back((tx & ones, rx));
            if log.len() > 64 {
                log.pop_front();
            }
        } else {
            let (p2, p3) = (self.seg_f000.get(0xF000_0E00), self.seg_f000.get(0xF000_0F00));
            let log = &mut self.tc1784.spi_other[k];
            log.push_back((tx & ones, p2, p3));
            if log.len() > 64 {
                log.pop_front();
            }
        }
        self.src_set_srr(SSC_BASE[k] + 0xF8);
        self.dma_request(k);
        if let Some(tb) = self.tc1784.ssc[k].tb_full.take() {
            self.ssc_start(k, tb);
        }
    }

    /// Sample the MCU's power-stage enable / disable outputs P3.9 / P3.10 for the monitor device.
    fn monitor_pins(&mut self) {
        let (p1, p3) = (self.seg_f000.get(0xF000_0D00), self.seg_f000.get(0xF000_0F00));
        self.tc1784.monitor.pins = ((p3 >> 9) & 1 | ((p3 >> 10) & 1) << 1 | ((p1 >> 13) & 1) << 2) as u8;
    }

    // ------------------------------------------------------------------ DMA

    fn dma_request(&mut self, n: usize) {
        if self.tc1784.dma.hw_enabled & (1 << n) != 0 {
            self.dma_move(n);
        }
    }

    /// One DMA move of channel `n` (CHCR.CHDW width, ADRCR address update) and transaction
    /// completion when the transfer count reaches 0.
    fn dma_move(&mut self, n: usize) {
        if self.tc1784.dma.ch[n].tcount == 0 {
            return;
        }
        let base = DMA_CH + 0x20 * n as u32;
        let chcr = self.seg_f000.get(base + 4);
        let adrcr = self.seg_f000.get(base + 0xC);
        let width = 1u32 << ((chcr >> 21) & 3).min(2);
        let c0 = self.tc1784.dma.ch[n];
        let val = self.dma_bus_read(c0.sadr, width);
        let step = |addr: u32, mf: u32, inc: bool, cbl: u32| {
            let d = width << mf;
            let mask = if cbl >= 16 { 0xFFFF } else { (1u32 << cbl) - 1 };
            let next = if inc { addr.wrapping_add(d) } else { addr.wrapping_sub(d) };
            (addr & !mask) | (next & mask)
        };
        let c = &mut self.tc1784.dma.ch[n];
        c.sadr = step(c.sadr, adrcr & 7, adrcr & 8 != 0, (adrcr >> 8) & 0xF);
        c.dadr = step(c.dadr, (adrcr >> 4) & 7, adrcr & 0x80 != 0, (adrcr >> 12) & 0xF);
        c.tcount -= 1;
        c.moves += 1;
        let done = c.tcount == 0;
        // The move retires (address update) before its write side effects: a write to SSC TB starts
        // the shifter, whose transmit request may immediately ask this channel for the next move.
        self.dma_bus_write(c0.dadr, width, val);
        if done {
            self.tc1784.dma.intsr |= 1 << n;
            // UNVERIFIED (no manual): CHICR.INTCT != 0 raises the DMA SRC selected by CHICR.INTP.
            let chicr = self.seg_f000.get(base + 8);
            if chicr & 0xC != 0 {
                self.src_set_srr(DMA_SRC0 - 4 * ((chicr >> 8) & 7));
            }
        }
    }

    fn dma_ram(&mut self, addr: u32) -> Option<(&mut [u8], usize)> {
        let off = (addr & 0x000F_FFFF) as usize;
        match addr >> 20 {
            // DSPR local (0xD00) and the global address the Spi driver programs (0xE84, 0x800E5B96)
            0xD00 | 0xE84 => Some((&mut self.dspr[..], off)),
            0xC00 | 0xD40 => Some((&mut self.pspr[..], off)),
            0x900 | 0xB00 => Some((&mut self.lmu[..], off)),
            _ => None,
        }
    }

    fn dma_bus_read(&mut self, addr: u32, width: u32) -> u32 {
        if let Some(k) = SSC_BASE.iter().position(|&b| addr == b + 0x24) {
            return self.tc1784.ssc[k].rb;
        }
        // PFlash (cached/non-cached view) is a readable bus slave: the Spi driver moves constant
        // TX data from flash (e.g. 0x8013CE84 for the CS P3.6 jobs).
        if matches!(addr >> 28, 0x8 | 0xA) {
            let off = (addr & 0x0FFF_FFFF) as usize;
            if let Some(src) = self.flash.get(off..off + width as usize) {
                let mut b = [0u8; 4];
                b[..width as usize].copy_from_slice(src);
                return u32::from_le_bytes(b);
            }
        }
        match self.dma_ram(addr) {
            Some((m, off)) if off + width as usize <= m.len() => {
                let mut b = [0u8; 4];
                b[..width as usize].copy_from_slice(&m[off..off + width as usize]);
                u32::from_le_bytes(b)
            }
            _ => {
                self.tc1784.dma.bus_errors += 1;
                0
            }
        }
    }

    fn dma_bus_write(&mut self, addr: u32, width: u32, val: u32) {
        if let Some(k) = SSC_BASE.iter().position(|&b| addr == b + 0x20) {
            self.ssc_tb_write(k, val);
            return;
        }
        match self.dma_ram(addr) {
            Some((m, off)) if off + width as usize <= m.len() => {
                m[off..off + width as usize].copy_from_slice(&val.to_le_bytes()[..width as usize]);
                let canonical = if addr >> 20 == 0xE84 { 0xD000_0000 | (addr & 0x000F_FFFF) } else { addr };
                self.mark_ram_write_code_dirty(canonical, width);
            }
            _ => self.tc1784.dma.bus_errors += 1,
        }
    }

    // ------------------------------------------------------------------ ADC

    fn adc1784_write(&mut self, k: usize, a: u32, off: u32, v: u32, m: u32) -> bool {
        let old = self.seg_f000.get(a);
        let new = (old & !m) | (v & m);
        match off {
            0x00 if self.scu.endinit_active(a) => return true, // CLC: ENDINIT-protected
            0x00 => return false,
            0x70 | 0x180..=0x1BC => {} // hardware owned
            0x74 => {
                let ef = self.seg_f000.get(a - 4) & !(v & m);
                self.seg_f000.set(a - 4, ef);
            }
            0x98 | 0xB8 => {
                let s = if off == 0x98 { 1 } else { 3 };
                self.seg_f000.set(a, new & !(1 << 9));
                if v & m & (1 << 9) != 0 {
                    self.adc1784_start(k, s);
                }
            }
            _ => self.seg_f000.set(a, new),
        }
        true
    }

    fn adc1784_start(&mut self, k: usize, s: u32) {
        let mask = self.seg_f000.get(ADC_BASE[k] + 0x90 + 0x20 * (s >> 1)) & 0xFFFF;
        let t = self.s_to_insns(mask.count_ones().max(1) as f64 * ADC_CONV_S_PER_CHANNEL);
        self.tc1784.adc.running[k] = Some((s, self.io_now() + t));
        self.stop_requested = true;
    }

    /// Hardware-gated scan (CRMR.ENGT = 01, driver 0x800E2692 with group flag bit 4): one scan per
    /// output transition of the GPTA0 trigger cell ([`ADC_TRIGGER_LTC`]).
    fn adc1784_trigger(&mut self, k: usize) {
        let s = if k == 0 { 1u32 } else { 3 };
        let crmr = self.seg_f000.get(ADC_BASE[k] + 0x98 + 0x20 * (s >> 1));
        if crmr & 3 == 1 && self.tc1784.adc.running[k].is_none() {
            self.adc1784_start(k, s);
        }
    }

    fn adc1784_complete(&mut self, k: usize) {
        let Some((s, _)) = self.tc1784.adc.running[k].take() else { return };
        let driven = self.valve_pins_driven();
        self.tc1784.sense_coils(driven);
        // Power-stage supply senses ADC[30]/ADC[31] (kernel 1 ch 14/15) behind the switch.
        self.monitor_pins();
        let vs = if self.tc1784.monitor.outputs_enabled() { self.tc1784.valve_supply_pin_v } else { 0.0 };
        self.tc1784.adc_volts[1][14] = vs;
        self.tc1784.adc_volts[1][15] = vs;
        // Output readbacks wired to analog inputs (port output latch high = VAREF at the pin).
        for &(port, pin, adc) in OUTPUT_READBACKS.iter() {
            let high = self.seg_f000.get(port) & (1 << pin) != 0;
            let pull_down = self.seg_f000.get(ADC_BASE[1] + 0x34) & ADC_TEST_PULLDOWN != 0;
            self.tc1784.adc_volts[adc / 16][adc % 16] = match (pull_down, high) {
                (true, _) => 0.0,
                (false, true) => ADC_VAREF as f32,
                (false, false) => (ADC_VAREF / 2.0) as f32,
            };
        }
        let base = ADC_BASE[k];
        let mask = self.seg_f000.get(base + 0x90 + 0x20 * (s >> 1)) & 0xFFFF;
        for ch in 0..16u32 {
            if mask & (1 << ch) == 0 {
                continue;
            }
            let chctr = self.seg_f000.get(base + 0x100 + 4 * ch);
            let resreg = (chctr >> 12) & 0xF;
            let icls = (chctr >> 10) & 3;
            let dw = (self.seg_f000.get(base + 0x50 + 4 * icls) >> 8) & 3;
            let bits = match dw {
                1 => 12,
                2 => 8,
                _ => 10,
            };
            let v = (self.tc1784.adc_volts[k][ch as usize] as f64 / ADC_VAREF).clamp(0.0, 1.0);
            let code = ((v * ((1u32 << bits) - 1) as f64).round() as u32) << (12 - bits);
            self.seg_f000.set(base + 0x180 + 4 * resreg, 0x8000_0000 | ch << 20 | code);
        }
        let ef = self.seg_f000.get(base + 0x70) | 1 << (16 + s);
        self.seg_f000.set(base + 0x70, ef);
        self.tc1784.adc.scans[k] += 1;
        if self.seg_f000.get(base + 0x3C) & (1 << s) != 0 {
            self.src_set_srr(ADC_SRC_NODES[s as usize]);
        }
    }

    // ------------------------------------------------------------------ FADC

    fn fadc_raise(&mut self, srs: u32) {
        for n in 0..4 {
            if srs & (1 << n) != 0 {
                self.src_set_srr(FADC_SRCS[3 - n]);
            }
        }
    }

    /// Clock the FADC from fSYS = fFPI and complete its events up to now.
    fn fadc_update(&mut self) {
        let now = self.io_now() as f64;
        if !self.tc1784.fadc.quiet(self.fadc_insns_per_fsys(), now) {
            self.fadc_clock_and_advance(now);
        }
    }

    fn fadc_insns_per_fsys(&self) -> f64 {
        (self.scu.fpidiv() as f64 + 1.0) / crate::stm_icu::CYCLES_PER_INSN as f64
    }

    fn fadc_clock_and_advance(&mut self, now: f64) {
        self.tc1784.fadc.set_clock(self.fadc_insns_per_fsys(), now);
        let srs = self.tc1784.fadc.advance(now);
        self.fadc_raise(srs);
    }

    // ------------------------------------------------------------------ time

    /// Complete every event whose time has come (called at quantum boundaries and on accesses).
    pub(crate) fn tc1784_update(&mut self) {
        if self.variant != BoardVariant::Dq200Tc1784 {
            return;
        }
        let now = self.io_now();
        for k in 0..2 {
            while matches!(self.tc1784.ssc[k].shifting, Some((_, t)) if now >= t) {
                self.ssc_complete(k);
            }
            if matches!(self.tc1784.adc.running[k], Some((_, t)) if now >= t) {
                self.adc1784_complete(k);
            }
        }
        self.gpta_update();
        self.fadc_update();
    }

    pub(crate) fn tc1784_insns_to_next_event(&mut self) -> u64 {
        if self.variant != BoardVariant::Dq200Tc1784 {
            return u64::MAX;
        }
        let now = self.io_now();
        let gpta = self.gpta_insns_to_next_event(now);
        let io = &self.tc1784;
        let ssc = io.ssc.iter().filter_map(|s| s.shifting.map(|(_, t)| t));
        let adc = io.adc.running.iter().flatten().map(|&(_, t)| t);
        let fadc = io.fadc.next_event().map(|t| t.ceil() as u64);
        ssc.chain(adc).chain(fadc).map(|t| t.saturating_sub(now).max(1)).min().unwrap_or(u64::MAX).min(gpta)
    }

    /// Instructions until the next GPTA internal event (`Gpta::next_event_tick`), looking at most
    /// one maximal quantum ahead. Cutting the quantum there delivers the GPTA service requests
    /// (Pwm period notifications, V401 commutation steps, timer channels) at their time on both CPU
    /// backends; at the next quantum boundary instead, their latency (up to `QUANTUM_MAX_INSNS`)
    /// depended on where each backend happened to end its quanta. ponytail: the V401 zero-crossing
    /// captures (IN47 edges from `pump`) are still timestamped exactly but raised at the boundary.
    fn gpta_insns_to_next_event(&mut self, now: u64) -> u64 {
        let Some((num, den)) = self.gpta_ratio() else { return u64::MAX };
        // GPTA ticks = (rem + insns * CYCLES_PER_INSN * num) / den since `gpta_at` (see `gpta_update`)
        let per_insn = crate::stm_icu::CYCLES_PER_INSN * num;
        let elapsed = self.tc1784.gpta_rem + now.saturating_sub(self.tc1784.gpta_at) * per_insn;
        let g = &mut self.tc1784.gpta;
        let base = g.now() + elapsed / den;
        let horizon = (crate::QUANTUM_MAX_INSNS * per_insn).div_ceil(den) + 1;
        let Some(t) = g.next_event_tick(base + horizon) else { return u64::MAX };
        let need = (t.saturating_sub(g.now()) * den).saturating_sub(self.tc1784.gpta_rem);
        (self.tc1784.gpta_at + need.div_ceil(per_insn)).saturating_sub(now).max(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `Gpta::next_event_tick` (the quantum cut of `gpta_insns_to_next_event`) names the tick at which
    /// a timer raises its request: GT0 on fGPTA/16 two counts before overflow -> 32 ticks ahead.
    #[test]
    fn gpta_next_event_is_the_request_tick() {
        let mut g = crate::gpta::Gpta::tc1784(40_000_000);
        let r = |off: u32| GPTA0_BASE + off;
        g.mmio_write(r(0xD8), 0xC846, 4); // CLK4 = fGPTA/16
        g.mmio_write(r(0xE0), 0x4E | 0x80, 4); // GT0 on CLK4, overflow request enabled
        g.mmio_write(r(0x400), 0x70F, 4);
        g.advance(16 * 10, &[]);
        g.mmio_write(r(0xE8), 0xFF_FFFE, 4);
        let now = g.now();
        assert_eq!(g.next_event_tick(now + 1000), Some(now + 32));
        assert_eq!(g.next_event_tick(now + 31), None, "beyond the horizon");
        g.advance(31, &[]);
        assert!(g.take_requests().is_empty());
        g.advance(1, &[]);
        assert_eq!(g.take_requests().len(), 1, "the request comes exactly at the predicted tick");
    }

    /// The coil follows the analytic R-L step response while switched on and freewheels to zero.
    #[test]
    fn coil_load_integrates_rl_exactly_and_freewheels_to_zero() {
        let f = 40e6;
        let mut c = CoilLoad { on: true, ..Default::default() };
        let (r, l, v) = (c.r_ohm, c.l_h, 13.8);
        // ten uneven steps reach the same current as one step of the summed time (exact integration)
        let mut t = 0u64;
        for k in 1..=10u64 {
            t += 1000 * k;
            c.advance(t, f, v);
        }
        let ts = t as f64 / f;
        let exact = v / r * (1.0 - (-ts * r / l).exp());
        assert!((c.i_a - exact).abs() < 1e-9, "{} vs {exact}", c.i_a);
        c.on = false;
        c.advance(t + (0.05 * f) as u64, f, v);
        assert_eq!(c.i_a, 0.0, "freewheeling current decays to zero and stays there");
    }

    /// 30 % PWM at 3 kHz: the mean current settles at (D·V − (1−D)·V_diode) / R with the small
    /// ripple of an L/R = 2.4 ms coil; the firmware's current sense reads it through SHUNT_V_PER_A.
    #[test]
    fn coil_load_pwm_mean_current_and_sense_scale() {
        let f = 40e6;
        let mut c = CoilLoad::default();
        let (period, high, v) = (13_340u64, 4_002u64, 13.8);
        let (mut t, mut peak, mut valley) = (0u64, 0.0, 0.0);
        for _ in 0..3000 {
            c.on = true;
            c.advance(t + high, f, v);
            peak = c.i_a;
            c.on = false;
            t += period;
            c.advance(t, f, v);
            valley = c.i_a;
        }
        let d = high as f64 / period as f64;
        let mean = (d * v - (1.0 - d) * COIL_FREEWHEEL_V) / c.r_ohm;
        let mid = 0.5 * (peak + valley); // segments ≈ linear at T << L/R
        assert!((mid - mean).abs() < 0.01 * mean, "{mid} vs {mean}");
        assert!((c.i_mean_a - mean).abs() < 0.01 * mean, "period mean {} vs {mean}", c.i_mean_a);
        let ripple = (v - c.r_ohm * mean) * high as f64 / f / c.l_h;
        assert!(((peak - valley) - ripple).abs() < 0.05 * ripple, "ripple {} vs {ripple}", peak - valley);
        // firmware: code * 2501 >> 9 = 0.1 mA (0x800ED8C0)
        let code = ((1.0 * SHUNT_V_PER_A) / ADC_VAREF * 4095.0).round() as u32;
        assert!(((code * 2501) >> 9).abs_diff(10_000) <= 3, "1 A -> {} x 0.1 mA", (code * 2501) >> 9);
    }

    /// Out-of-frame replies of the 32-bit monitoring device: frame N answers frame N-1 with its
    /// address, the written data (writes) or the register content (reads), and a counter that
    /// advances by one per reply (the checks of 0x800DD8F6).
    #[test]
    fn monitor_device_replies_out_of_frame_with_counter() {
        let mut m = MonitorDevice { attached: true, ..Default::default() };
        let word = |m: &mut MonitorDevice, w: u32| {
            let lo = m.xfer16(w & 0xFFFF);
            let hi = m.xfer16(w >> 16);
            lo | hi << 16
        };
        let write6 = 0x600 << 6 | 6 << 1; // write register 6 = 0x600
        let read6 = 6 << 1 | 1;
        let read24 = 24 << 1 | 1;
        word(&mut m, write6);
        let r1 = word(&mut m, read6); // answers the write
        let r2 = word(&mut m, read24); // answers the read of 6
        let r3 = word(&mut m, read24);
        assert_eq!(r1 & 0x3FFF_FFFF, write6, "write echoed");
        assert_eq!(r2 & 0x3FFF_FFFF, read6 | 0x600 << 6, "register content");
        assert_eq!(r3 & 0x3FFF_FFFF, read24, "unwritten register reads 0");
        assert_eq!(((r2 >> 30) + 4 - (r1 >> 30)) % 4, 1, "counter advances per reply");
        assert_eq!(((r3 >> 30) + 4 - (r2 >> 30)) % 4, 1);
    }

    /// Valve driver: the firmware's start-up words are echoed (bits 26:0) one word later, as
    /// `0x800DF62C` checks, and the status read (register 6) answers 6 with no fault bits
    /// (`0x800DF750` / `0x800DFE3E`).
    #[test]
    fn valve_driver_echoes_configuration_and_reports_healthy_status() {
        let mut v = ValveDriver::default();
        let word = |v: &mut ValveDriver, w: u32| v.xfer16(w & 0xFFFF) | v.xfer16(w >> 16) << 16;
        let init = [0x1840_0010u32, 0x3903_0101, 0x3000_4102, 0x2000_4103, 0x3800_4104, 0x2800_4105, 0x07FF_FFF6 | (3 << 27), 0x3800_0007];
        let mut prev = None;
        for w in init {
            let r = word(&mut v, w);
            if let Some(p) = prev.filter(|p: &u32| p & 0xF != 6) {
                assert_eq!(r & 0x07FF_FFFF, p & 0x07FF_FFFF, "echo of {p:08X}");
            }
            prev = Some(w);
        }
        word(&mut v, 0x2800_0006);
        let status = word(&mut v, 0x3800_0007);
        assert_eq!((status & 0xF, status >> 4), (6, 0));
    }

    /// The monitor device reproduces what the firmware's own checks expect: identification
    /// (0x80042010), register 8 (0x800428EA) and the four phases of the shut-off path test
    /// (0x80042998..0x80042CEA), with the supply switch also following P3.9/P3.10 and P1.13.
    #[test]
    fn monitor_device_meets_the_firmware_shutoff_path_test() {
        let mut m = MonitorDevice { attached: true, pins: 0b101, ..Default::default() };
        let word = |m: &mut MonitorDevice, w: u32| {
            let lo = m.xfer16(w & 0xFFFF);
            lo | m.xfer16(w >> 16) << 16
        };
        let wr = |reg: u32, d: u32| d << 6 | reg << 1;
        let rd = |reg: u32| reg << 1 | 1;
        word(&mut m, wr(15, 0xF6_0D07));
        word(&mut m, wr(14, 0x6D_7502));
        word(&mut m, rd(15));
        let r15 = word(&mut m, rd(14));
        let r14 = word(&mut m, rd(8));
        let r8 = word(&mut m, rd(17));
        assert_eq!((r15 >> 6) & 0xFF_FFFF, 0xF6_0D07);
        assert_eq!((r14 >> 8) & 1, 1, "register 14 bit 2 after configuration");
        assert_eq!((r8 >> 6) & 0x30, 0x20, "register 8 bits 5:4 = 10b");
        assert!(!m.outputs_enabled(), "outputs off after power-up until the watchdog is served");
        // the firmware's answer sequence: both correct, then the test phases
        let answer = |m: &mut MonitorDevice, ok0: bool, ok1: bool| {
            for (c, reg, ok) in [(0, 18, ok0), (1, 21, ok1)] {
                let a = QA_ANSWERS[m.wd[c].question as usize] as u32;
                word(m, wr(reg, if ok { a } else { !a & 0xF }));
            }
        };
        for _ in 0..3 {
            answer(&mut m, true, true);
        }
        assert!(m.outputs_enabled(), "served watchdog -> supplies on");
        answer(&mut m, false, true);
        assert!(!m.wd[0].ok && !m.outputs_enabled(), "ch0 wrong -> ch0 bit 0 = 0, supplies off");
        answer(&mut m, true, false);
        assert!(m.wd[0].ok && m.outputs_enabled(), "ch0 correct + ch1 wrong -> ch0 bit 0 = 1, supplies on");
        answer(&mut m, true, true);
        answer(&mut m, true, false);
        assert!(!m.wd[1].ok && !m.outputs_enabled(), "ch1 wrong -> ch1 bit 0 = 0, supplies off");
        answer(&mut m, false, true);
        assert!(m.wd[1].ok && m.outputs_enabled(), "ch0 wrong + ch1 correct -> ch1 bit 0 = 1, supplies on");
        m.pins = 0b001;
        assert!(!m.outputs_enabled(), "P1.13 low -> supplies off (0x800459BE)");
        m.pins = 0b110;
        assert!(!m.outputs_enabled(), "disable pattern on P3.9/P3.10 -> supplies off");
    }

    /// Cost of the coil-load model (diagnostic): ns per update and the share of real time at ~110k
    /// updates/s (8 coils x (2 PWM edges x 3 kHz + 2 ADC scans x 5.7 kHz)).
    #[test]
    #[ignore = "diagnostic"]
    fn coil_load_cost() {
        let mut c = [CoilLoad::default(); 8];
        let n = 2_000_000u64;
        let t0 = std::time::Instant::now();
        for k in 0..n {
            let x = &mut c[(k & 7) as usize];
            x.on = k & 8 != 0;
            x.advance(k * 700, 40e6, 13.8);
        }
        let ns = t0.elapsed().as_nanos() as f64 / n as f64;
        println!("CoilLoad::advance {ns:.1} ns/call -> {:.3} % of real time (i {:.3})", ns * 110e3 / 1e9 * 100.0, c[0].i_a);
    }

    #[test]
    fn endinit_blocks_clc_and_scan_pins_handles_oob_and_sub_ulp_duty() {
        let mut b = crate::Simos18Board::new_variant(
            &vec![0u8; 0x40_0000],
            &vec![0u8; 0x2000],
            crate::memory::BoardVariant::Dq200Tc1784,
        )
        .unwrap();
        {
            let hw = b.hw_mut();
            assert!(hw.scu.endinit);
            // ENDINIT=1 must block writes to GPTA0_CLC, FADC_CLC, SSC0_CLC, DMA_CLC, ADC0_CLC
            for &clc in &[0xF000_1800u32, 0xF010_0400, 0xF010_0100, 0xF000_3C00, 0xF010_1000] {
                let before = hw.periph_peek(clc);
                hw.mmio_write(clc, 4, 0xDEAD_BEEF);
                assert_eq!(hw.periph_peek(clc), before, "0x{clc:08X} write must be blocked under ENDINIT=1");
            }
            assert_eq!(hw.scu.endinit_dropped, 5);

            // OOB pin & cell indices must not panic or overflow u8
            hw.tc1784.set_pin(200, PinSignal { period_s: 1e-3, duty: 0.5, idle: false });
            assert!(hw.tc1784.cell_pwm(0, true, 250).is_none());

            // Sub-ULP duty and ultra-short period at large tick count must terminate in bounded edges!
            hw.tc1784.set_pin(0, PinSignal { period_s: 1e-9, duty: 1e-18, idle: false });
            let big = 1u64 << 54;
            hw.tc1784.scan_pins(big, big + 1_000_000, 40e6);
            assert!(hw.tc1784.edge_buf.len() <= 16_384);
        }
    }
}
