//! DQ250 0D9 mechatronic board (TC1766): the circuits around the MCU that the firmware talks to — speed and analog
//! sensors, the SPI watchdog companion chip, the SPI temperature sensor, the valve power stages and their current
//! senses. Wired into the board by [`crate::tcu_dq250`]; evidence and labels in `docs/DQ250_IO_MAP.md` (addresses are
//! the 0D9300011R / 4319 image unless noted). Nothing here reads or writes firmware RAM.

use crate::tc1784_io::CoilLoad;

/// Sensor supply on the alternate ADC references `VAREF[1]` / `VAREF[2]` (CHCONn.REF = 1 / 2: AIN22, AIN30 / AIN16..19).
/// HYP 5 V: the ASW converts exactly these channels as `mV = code · 5000 / 4095` (0x8008EA68, `FUN_80046c30`).
pub const SENSOR_REF_V: f32 = 5.0;

/// Speed sensors on GPTA0 input lines: (capture line, count line, teeth). Slot order of the firmware (snapshot
/// 0x800640BE..0x80064168, teeth `0xD00060F0` = [60, 36, 36, 36, 36] read at run time; VERIFIED-S/T): 0 clutch input
/// (engine side, replaced by the CAN engine speed under P0726: G182), 1 input shaft 1 (K1), 2 input shaft 2 (K2), 3 / 4
/// output speed A / B (`× 18/62` in the firmware: they sit on the output shaft of the 62:18 final). Both lines of a
/// pair carry the same sensor signal (HYP, as on the DQ200).
pub const SPEED_SENSORS: [(u8, u8, u32); 5] = [(8, 9, 60), (10, 11, 36), (6, 7, 36), (2, 3, 36), (0, 1, 36)];

/// Gear-actuator travel sensors AIN16..19 = actuators 0..3 (1/3, 4/2, 5/N, 6/R). Characteristic the firmware is
/// calibrated for (CAL 0x80030400 mV / 0x80030424 travel, 17 points, the same for all four; mV axis also at 0x8003479C in 0D9300012F): sensor output
/// mV -> travel 0.01 mm (unit HYP). VERIFIED-S.
pub const TRAVEL_SENSOR_AIN: [usize; 4] = [16, 17, 18, 19];
pub const TRAVEL_MV: [f64; 17] = [936., 989., 1116., 1307., 1913., 2084., 2265., 2397., 2470., 2549., 2676., 2905., 3272., 3946., 4142., 4171., 4700.];
pub const TRAVEL_CMM: [f64; 17] = [-1167., -1067., -967., -867., -567., -467., -317., -168., -67., 32., 183., 382., 583., 884., 1033., 1134., 1134.];

/// Travel sensor output voltage at `mm` of actuator travel: the inverse of the firmware's characteristic.
pub fn travel_sensor_v(mm: f64) -> f32 {
    let mm = if mm.is_finite() { mm } else { 0.0 };
    let c = (mm * 100.0).clamp(TRAVEL_CMM[0], TRAVEL_CMM[15]);
    let i = (1..16).find(|&i| c <= TRAVEL_CMM[i]).unwrap_or(15);
    let f = (c - TRAVEL_CMM[i - 1]) / (TRAVEL_CMM[i] - TRAVEL_CMM[i - 1]);
    ((TRAVEL_MV[i - 1] + f * (TRAVEL_MV[i] - TRAVEL_MV[i - 1])) / 1000.0) as f32
}

/// Clutch pressure sensors: AIN30 = K1 (sub-gearbox 1), AIN22 = K2. The firmware reads `p[0.01 bar] = (mV − 173) ·
/// 14434 / 32768 − 100`, clamped 0..2000, with the P5.10..14 strap at 0 (CAL 0x800304F0 holds the other gain/offset;
/// `FUN_80046c30`, VERIFIED-S/T): 400 mV at 0 bar, 227 mV/bar. Window 100..4900 mV (P0805 / P087A outside).
pub const PRESSURE_SENSOR_AIN: [usize; 2] = [30, 22];
pub fn pressure_sensor_v(bar: f64) -> f32 {
    let bar = if bar.is_finite() { bar.max(0.0) } else { 0.0 };
    ((400.0 + 227.0 * bar) / 1000.0).min(4.95) as f32
}

/// Gearbox oil temperature on AIN11 (ratio to VAREF, 0.01 %): curve CAL 0x800304C0 −50/0/50/100/150/200 °C ↔
/// 4454/5000/5442/5808/6114/6374 (VERIFIED-S; window 501..9499, P2741 outside). In 0D9300011R the ASW uses its thermal
/// model (fed by the SPI sensor [`lm74_word`]) instead (CAL 0x800304EE = 2).
pub const ATF_TEMP_AIN: usize = 11;
pub fn atf_temp_v(t_c: f64) -> f32 {
    const T: [f64; 6] = [-50., 0., 50., 100., 150., 200.];
    const R: [f64; 6] = [4454., 5000., 5442., 5808., 6114., 6374.];
    let t = (if t_c.is_finite() { t_c } else { 20.0 }).clamp(-50.0, 200.0);
    let i = (1..6).find(|&i| t <= T[i]).unwrap_or(5);
    let r = R[i - 1] + (t - T[i - 1]) / (T[i] - T[i - 1]) * (R[i] - R[i - 1]);
    (r / 10000.0 * crate::tc1766_adc::VAREF as f64) as f32
}

/// SSC0 16-bit device on CS P2.9: an LM74-type SPI temperature sensor (HYP, strong: the driver `FUN_8008add8` checks the
/// product ID 0x800F and writes continuous conversion, decodes `T = ((rx + 0x2CC0) >> 7) − 0x59` °C, valid only with
/// the low two bits set). Word in continuous conversion: 13-bit two's-complement temperature in 1/16 °C at bits 15:3,
/// bits 2:0 high (tri-state bits read as pulled up, HYP).
pub const LM74_CS: (usize, u32) = (2, 9);
pub fn lm74_word(t_c: f64) -> u16 {
    let t = (if t_c.is_finite() { t_c } else { 20.0 }).clamp(-55.0, 150.0);
    ((((t * 16.0).round() as i16) << 3) as u16) | 0b111
}

/// Watchdog companion chip on SSC1, CS P2.1 (channel 0x800DCA24: 16 bit, MSB first). The firmware reads each reply as
/// the answer to the frame it shifted (RX buffer k = reply to TX frame k, VERIFIED-S), so the chip answers in-frame.
/// Protocol as the firmware expects it (`FUN_8007f284`, `FUN_80071828`, `FUN_8009f1cc`; VERIFIED-S):
/// * `0xE800` -> bits 6:4 error counter, 3:0 question; `0xEA00` -> bits 7:6 = 11 (else the ASW re-initialises);
/// * four answer frames `0xF0xx` per question, response codes 3, 2, 1, 0 ([`QaWatchdog::answer`]); after 200 good
///   cycles the ASW sends one wrong set and expects the counter + 1;
/// * `0xD401` re-initialise, `0xD417` run, `0xD010` / `0xD0D0` mode (D0D0 before shut-down), `0xD276` / `0xD246`
///   register 0x12 (HYP: bits 5:4 = power-stage enables), `0xC800` / `0xEC00` / `0xF600` status reads (0 = normal).
///
/// UNVERIFIED (the chip's internals are not in the dump): power-on counter 0, the next question = previous + 1 (any
/// sequence satisfies the ASW), outputs enabled while register 0x12 bits 5:4 are set, no shut-down, counter < 7.
pub const WD_CS: (usize, u32) = (2, 1);

#[derive(Debug, Clone, Default)]
pub struct QaWatchdog {
    pub q: u8,
    pub cnt: u8,
    rc: u8,
    bad: bool,
    pub reg12: u8,
    pub shutdown: bool,
    pub good_cycles: u64,
    pub bad_cycles: u64,
}

impl QaWatchdog {
    /// Expected answer byte for response code `rc` (0..3) to question `q` (`FUN_8009f1cc` / `FUN_800bd0ec`).
    pub fn answer(rc: u8, q: u8) -> u8 {
        let b = |x: u8, i: u8| (x >> i) & 1;
        let (r0, r1) = (rc & 1, (rc >> 1) & 1);
        let (q0, q1, q2, q3) = (b(q, 0), b(q, 1), b(q, 2), b(q, 3));
        (q2 ^ r0) << 7
            | (q0 ^ r0) << 6
            | (q3 ^ r0) << 5
            | (q1 ^ r0) << 4
            | (q2 ^ q3 ^ q0 ^ r1) << 3
            | (q3 ^ q0 ^ q1 ^ r1) << 2
            | (q0 ^ q1 ^ q2 ^ r1) << 1
            | (q3 ^ q0 ^ r1)
    }

    /// Power-stage supplies switched on by the chip.
    pub fn enabled(&self) -> bool {
        self.reg12 & 0x30 == 0x30 && !self.shutdown && self.cnt < 7
    }

    /// One 16-bit frame while CS is low; returns the word shifted out.
    pub fn frame(&mut self, tx: u16) -> u16 {
        let (cmd, d) = ((tx >> 8) as u8, tx as u8);
        match cmd {
            0xE8 => ((self.cnt as u16) << 4) | self.q as u16,
            0xEA => 0xC0,
            0xF0 => {
                self.bad |= d != Self::answer(3 - self.rc, self.q);
                self.rc += 1;
                if self.rc == 4 {
                    if self.bad {
                        self.cnt = (self.cnt + 1).min(7);
                        self.bad_cycles += 1;
                    } else {
                        self.cnt = self.cnt.saturating_sub(1);
                        self.good_cycles += 1;
                    }
                    (self.rc, self.bad, self.q) = (0, false, (self.q + 1) & 0xF);
                }
                0
            }
            0xD4 if d == 0x01 => {
                (self.rc, self.bad, self.shutdown) = (0, false, false);
                0
            }
            0xD0 if d == 0xD0 => {
                self.shutdown = true;
                0
            }
            0xD2 => {
                self.reg12 = d;
                0
            }
            _ => 0,
        }
    }
}

/// How a valve's power stage is switched by the MCU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValveDrive {
    /// GPTA0 LTC cell output (PWM group's duty cell) on an alternate-function pin; output high = stage on (HYP).
    Pwm(u8),
    /// Port output latch (port, pin); high = on (`FUN_80080460`, pin table 0x800DAE60).
    Port(usize, u32),
}

/// One valve output in the plant's wet-unit channel order (`simemu_physics::mechatronic::wet`).
#[derive(Debug, Clone, Copy)]
pub struct Valve {
    pub name: &'static str,
    pub drive: ValveDrive,
    /// High-side switch 0..2 = P2.5 / P2.6 / P2.7 (sub-gearbox 1, sub-gearbox 2, common).
    pub hs: usize,
    /// AIN of the coil current sense (1.65 V/A, `I[0.1 mA] = code · 40010 >> 13`), if any.
    pub sense: Option<usize>,
}

/// Valve channels (0D9300011R): PWM cells from the 52-word OMCR decode (OUT21/23/25/27/73/75 <- LTC48/52/24/28/14/40),
/// port outputs, high sides and current senses (VERIFIED-S, docs/DQ250_IO_MAP.md §3). Function letters PCS A..F =
/// N215, N216, N217, N218, N233, N371 and shift A..E = N88, N90, N89, N91, N92 are HYP (SSP 308 naming order).
pub const VALVES: [Valve; 11] = [
    Valve { name: "N88", drive: ValveDrive::Port(0, 12), hs: 0, sense: None },
    Valve { name: "N89", drive: ValveDrive::Port(0, 14), hs: 1, sense: None },
    Valve { name: "N90", drive: ValveDrive::Port(0, 13), hs: 0, sense: None },
    Valve { name: "N91", drive: ValveDrive::Port(0, 15), hs: 1, sense: None },
    Valve { name: "N92", drive: ValveDrive::Port(2, 0), hs: 2, sense: None },
    Valve { name: "N215", drive: ValveDrive::Pwm(48), hs: 0, sense: Some(20) },
    Valve { name: "N216", drive: ValveDrive::Pwm(52), hs: 1, sense: Some(28) },
    Valve { name: "N217", drive: ValveDrive::Pwm(24), hs: 2, sense: Some(26) },
    Valve { name: "N218", drive: ValveDrive::Pwm(28), hs: 2, sense: Some(27) },
    Valve { name: "N233", drive: ValveDrive::Pwm(14), hs: 0, sense: None },
    Valve { name: "N371", drive: ValveDrive::Pwm(40), hs: 1, sense: None },
];

/// High-side switches (port, pin) and their supply-current senses (AIN, V/A): HS1 P2.5 -> AIN29, HS2 P2.6 -> AIN5
/// (1.6117 mA/code = 0.5 V/A), HS3 P2.7 -> AIN3 (0.8059 mA/code = 1.0 V/A). VERIFIED-S (units HYP).
pub const HIGH_SIDES: [((usize, u32), usize, f32); 3] = [((2, 5), 29, 0.5), ((2, 6), 5, 0.5), ((2, 7), 3, 1.0)];
/// Coil current sense of the four closed-loop PCS channels, V/A (0.4884 mA/code at VAREF 3.3 V).
pub const COIL_SENSE_V_PER_A: f32 = 1.65;
/// Master supply of the power stages: P3.10 high and P3.9 low (`FUN_800504d8`) and the watchdog chip's enable; its
/// output is read 6:1 on AIN4 and AIN6 (start-up test 0x8009D61C, VERIFIED-S/T). Which supply each of AIN4/AIN6
/// senses is not determined: both get the master supply.
pub const MASTER_ON: (usize, u32) = (3, 10);
pub const MASTER_OFF: (usize, u32) = (3, 9);
pub const MASTER_SENSE_AIN: [usize; 2] = [4, 6];

/// Valve coils of the board with their electrical state.
#[derive(Debug, Clone, Default)]
pub struct ValveStage {
    pub coils: [CoilLoad; 11],
    /// Battery voltage at the high sides, V.
    pub supply_v: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn watchdog_answers_match_the_firmware_vectors() {
        // Test vectors computed from the ASW's answer routine (fork A of docs/DQ250_IO_MAP.md §5).
        let row = |q| [3, 2, 1, 0].map(|rc| 0xF000 | QaWatchdog::answer(rc, q) as u16);
        assert_eq!(row(0), [0xF0FF, 0xF00F, 0xF0F0, 0xF000]);
        assert_eq!(row(1), [0xF0B0, 0xF040, 0xF0BF, 0xF04F]);
        assert_eq!(row(15), [0xF001, 0xF0F1, 0xF00E, 0xF0FE]);
        let mut w = QaWatchdog::default();
        assert_eq!(w.frame(0xE800), 0);
        for f in row(0) {
            w.frame(f);
        }
        assert_eq!((w.good_cycles, w.q, w.cnt), (1, 1, 0));
        for f in row(0) {
            w.frame(f); // answers to the old question: wrong
        }
        assert_eq!((w.bad_cycles, w.cnt), (1, 1));
        assert!(!w.enabled());
        w.frame(0xD276);
        assert!(w.enabled());
        w.frame(0xD246);
        assert!(!w.enabled());
    }

    #[test]
    fn sensor_transfers_invert_the_firmware_characteristics() {
        // travel: the firmware's own interpolation of the voltage gives the travel back
        for mm in [-11.0, -6.5, -0.5, 0.0, 3.0, 6.5, 9.0] {
            let mv = travel_sensor_v(mm) as f64 * 1000.0;
            let i = (1..17).find(|&i| mv <= TRAVEL_MV[i]).unwrap();
            let c = TRAVEL_CMM[i - 1] + (mv - TRAVEL_MV[i - 1]) / (TRAVEL_MV[i] - TRAVEL_MV[i - 1]) * (TRAVEL_CMM[i] - TRAVEL_CMM[i - 1]);
            assert!((c / 100.0 - mm).abs() < 1e-3, "{mm}: {c}");
        }
        for bar in [0.0, 5.0, 12.5] {
            let mv = pressure_sensor_v(bar) as f64 * 1000.0;
            let p = (mv - 173.0) * 14434.0 / 32768.0 - 100.0;
            assert!((p / 100.0 - bar).abs() < 0.01, "{bar}: {p}");
        }
        // LM74 word decoded as the driver does: ((rx + 0x2CC0) >> 7) − 0x59
        for t in [-20.0, 0.0, 25.0, 90.0, 120.0] {
            let rx = lm74_word(t) as i16 as i32;
            assert_eq!(((rx + 0x2CC0) >> 7) - 0x59, t as i32, "{t}");
        }
        assert!((atf_temp_v(0.0) - 0.5 * 3.3).abs() < 1e-6);
    }
}
