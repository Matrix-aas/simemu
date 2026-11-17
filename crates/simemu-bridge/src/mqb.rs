//! MQB rest-bus: the CAN frames of the car's modules that are not emulated (ESP, steering angle sensor / EPS, airbags,
//! gateway, energy manager, instrument cluster, BCM, climate control, driver door, ACC and its stalk, selector lever), as the
//! engine ECU and the TCU receive them on the drivetrain CAN. Only physical quantities of the plant, driver actions and a
//! real car's static defaults are encoded, never decisions of those modules (no ASR/ESP/EPS intervention, no ACC
//! acceleration request, no load-management or idle-speed request).
//!
//! Sources (labels as in `docs/SIMOS18_GPTA_IO_RE.md` §0):
//! - Which IDs the units receive: MultiCAN receive objects the firmwares configure (VERIFIED-T, SC8O40 264H / SC8S50
//!   264K, DQ200_G2 0CW300043H, DQ250 4930 / 4319; MILESTONE §2.10).
//! - Signal layout: opendbc `vw_mqb.dbc` / `vw_mqbevo.dbc` (DBC); MQB E2E = AUTOSAR profile 2, CRC-8H2F over bytes
//!   1..DLC-1 and the data ID of the counter value (opendbc `mqbcan.py`).
//! - Data IDs: the firmwares' own E2E tables (VERIFIED-S): DQ250 0D9300012F RX PDU table 0x800C4188 (byte 10 = index into
//!   the u16 data-ID arrays at 0x800C2BEC, CRC routine 0x8006C4C4), DQ200_G2 RX table 0x8013944C (data-ID pointer per
//!   CAN ID), SC8O40 0x800504E0..0x8005068F. Same values as opendbc where opendbc has them; ESP_07 0x91 and EPB_01 0x05
//!   are not in opendbc.
//! - Value conventions of Kombi_02 (init/invalid codes, QBit 0 = valid, standing time = engine standing time between
//!   ignition off and the next start, counted by the cluster from the CAN ignition state): Funktionsrahmen SC8L50Y0
//!   52.81 (FR p.19542–19551) and 28 ENSS "Engine off timer plausibility" (FR p.8741).
//! - Cycle times and which frames carry a counter / CRC: the engine ECU's MQB receive diagnosis table (FR p.21998–22030),
//!   VERIFIED-FR; FR data IDs: Airbag_02 0x44, EPB_01 0x05, TSG_FT_02, ESP_33 (same in the SC8O40 table).
//!
//! Not encoded (no timeout monitor of the ECU, DQ200 or DQ250 fails without them, VERIFIED-T, or no source for their
//! content): Einheiten_01, STS_01, Anhaenger_01, STH_01, PSD_04..06, Remotestart_FFB, network management (NMH_Gateway),
//! the Haldex controller.

const CRC8_H2F_TABLE: [u8; 256] = {
    let mut t = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u8;
        let mut k = 0;
        while k < 8 {
            c = if c & 0x80 != 0 { (c << 1) ^ 0x2F } else { c << 1 };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
};

/// MQB E2E CRC-8H2F (poly 0x2F, init/xorout 0xFF) over bytes 1..n followed by the per-message data-ID byte.
pub fn mqb_crc8_p2f(payload_bytes_1_to_7: &[u8], magic_id_byte: u8) -> u8 {
    let mut crc: u8 = 0xFF;
    for &b in payload_bytes_1_to_7 {
        crc = CRC8_H2F_TABLE[(crc ^ b) as usize];
    }
    crc = CRC8_H2F_TABLE[(crc ^ magic_id_byte) as usize];
    crc ^ 0xFF
}

/// Physical/driver state of the vehicle as the non-emulated modules would report it.
#[derive(Debug, Clone, Copy, Default)]
pub struct ChassisInputs {
    /// Wheel speeds FL, FR, RL, RR (km/h, magnitude).
    pub wheel_kmh: [f64; 4],
    pub vehicle_kmh: f64,
    /// The car rolls backwards (ESP_10 direction of travel).
    pub backward: bool,
    /// Longitudinal acceleration of the body, m/s² (ESP_02).
    pub accel_ms2: f64,
    pub brake_pressure_bar: f64,
    pub brake_pedal_pressed: bool,
    /// Selector lever raw position (`Waehlhebel_03.WH_SensorPos_roh`): P=8, R=7, N=6, D=5, S=12, M=10.
    /// Evidence: the DQ200_G2 firmware maps this code to its `Getriebe_11.GE_Fahrstufe` by the table
    /// `0x801382BC` = `01 08 08 08 08 08 07 06 05 00 0E 0E 09 09 0E 0F` (code 8 -> 5 P, 7 -> 6 R, 6 -> 7 N,
    /// 1..5 -> 8 D, 12/13 -> 9 S, 10/11/14 -> 14 tiptronic gate; `0x80072B38`) and by `0x8007240A`
    /// (7 -> state 1 = reverse: checked against driving direction -1 at `0x80072482`).
    pub selector_code: u8,
    /// Drive profile the driver selected (`Charisma_01` 0x385 target profile of every unit; 2 = normal, 3 = sport,
    /// UNVERIFIED values). It used to be sent in 0x392, which is ESP_07.
    pub drive_profile: u8,
    pub terminal_15: bool,
    pub terminal_50_request: bool,
    /// Engine running (the cluster's standing-time counter stops at a start).
    pub engine_running: bool,
    /// Four-wheel drive (`ESP_07.ESP_Quattro_Antrieb`).
    pub awd: bool,
    /// Alternator mechanical power, W, and utilisation, % (plant electrical model; BEM_05).
    pub alternator_w: f64,
    pub alternator_util_pct: f64,
    /// Outside air temperature, °C (plant ambient; BCM sensor raw in Gateway_72, cluster-filtered in Kombi_02).
    pub ambient_c: f64,
    /// Rolling radius of the tyres, m (ESP_20 / Kombi_03 circumference; 0 = the 205/55 R16 default).
    pub tyre_radius_m: f64,
    /// Odometer when the session began, km (the cluster adds the distance driven since).
    pub odometer_km: f64,
    /// Engine-off time before the session began (soak), s: the cluster's standing-time counter starts there.
    pub soak_s: f64,
}

/// Encoder period, s: [`MqbChassisEncoder::encode`] is called once per 10 ms of bus time.
const TICK_S: f64 = 0.01;
/// ABS sensor teeth (pole pairs) per wheel revolution, ESP_20 `ESP_Zaehnezahl`. UNVERIFIED (calibration knob).
pub const ABS_TEETH: u8 = 48;
/// Fuel in the tank, L, and its capacity (Octavia A7 50 L). Static default, UNVERIFIED: the plant has no tank.
pub const FUEL_L: f64 = 35.0;
pub const TANK_L: f64 = 50.0;
/// Clock of Diagnose_01 at the session start: 2026-11-04 12:00:00 (static default, UNVERIFIED; deterministic).
const CLOCK0: (u32, u32, u32, f64) = (2026, 11, 4, 12.0 * 3600.0);

/// E2E data IDs (one per counter value), VERIFIED-S in the firmwares (module docs).
mod data_id {
    pub const AIRBAG_01: [u8; 16] = [0x40; 16];
    pub const LWI_01: [u8; 16] = [0x86; 16];
    pub const ESP_21: [u8; 16] = [0xB4, 0xEF, 0xF8, 0x49, 0x1E, 0xE5, 0xC2, 0xC0, 0x97, 0x19, 0x3C, 0xC9, 0xF1, 0x98, 0xD6, 0x61];
    pub const ESP_02: [u8; 16] = [0xAA; 16];
    pub const ESP_05: [u8; 16] = [0x07; 16];
    pub const ESP_10: [u8; 16] = [0xAC; 16];
    pub const ESP_07: [u8; 16] = [0x91; 16];
    pub const ESP_20: [u8; 16] = [0xAC, 0xB3, 0xAB, 0xEB, 0x7A, 0xE1, 0x3B, 0xF7, 0x73, 0xBA, 0x7C, 0x9E, 0x06, 0x5F, 0x02, 0xD9];
    pub const KLEMMEN_STATUS_01: [u8; 16] = [0xC3; 16];
    /// FR p.22001 (ACU_2 static 0x44); SC8O40 0x800504F0.
    pub const AIRBAG_02: [u8; 16] = [0x44; 16];
    /// FR p.22018 (EPB_1_TCS static 0x05); DQ250 PDU table index 0x0F, DQ200 0x801393DC.
    pub const EPB_01: [u8; 16] = [0x05; 16];
    pub const ACC_06: [u8; 16] = [0x37, 0x7D, 0xF3, 0xA9, 0x18, 0x46, 0x6D, 0x4D, 0x3D, 0x71, 0x92, 0x9C, 0xE5, 0x32, 0x10, 0xB9];
    pub const GRA_ACC_01: [u8; 16] = [0x6A, 0x38, 0xB4, 0x27, 0x22, 0xEF, 0xE1, 0xBB, 0xF8, 0x80, 0x84, 0x49, 0xC7, 0x9E, 0x1E, 0x2B];
    /// FR p.22003 (DSC_LE_2); SC8O40 0x80050520.
    pub const TSG_FT_02: [u8; 16] = [0xC4, 0x6A, 0x69, 0x30, 0xCF, 0x61, 0x58, 0x51, 0x1B, 0x86, 0x99, 0xD3, 0xF6, 0x1D, 0x9A, 0x37];
    pub const ESP_33: [u8; 16] = [0x13, 0x21, 0x9B, 0x6A, 0x9A, 0x62, 0xD4, 0x65, 0x18, 0xF1, 0xAB, 0x16, 0x32, 0x89, 0xE7, 0x26];
}

/// (ID, DLC, period in 10 ms ticks) of every frame. Periods: the engine ECU's MQB receive diagnosis table (FR p.21998–22030,
/// "Recurrence"; its timeout symptom is set after 1.5 cycles, FR p.21866, so a slower frame is a fault), VERIFIED-FR;
/// Waehlhebel_03 (not received by the engine ECU) UNVERIFIED.
const FRAMES: [(u32, u8, u32); 31] = [
    (0x0B2, 8, 1),   // ESP_19 wheel speeds (TCS_19)
    (0x086, 8, 1),   // LWI_01 steering angle (PSTE_1_EPS)
    (0x0AF, 4, 1),   // Waehlhebel_03 selector lever
    (0x0FD, 8, 2),   // ESP_21 reference speed (TCS_21)
    (0x101, 8, 2),   // ESP_02 accelerations (TCS_2)
    (0x106, 8, 2),   // ESP_05 brake pressure (TCS_5)
    (0x116, 8, 2),   // ESP_10 wheel pulse counters (TCS_10)
    (0x104, 8, 2),   // EPB_01 (EPB_1_TCS)
    (0x122, 8, 2),   // ACC_06 (DCC_6)
    (0x12B, 8, 3),   // GRA_ACC_01 (CRU_1)
    (0x040, 8, 5),   // Airbag_01 (ACU_1)
    (0x30B, 8, 5),   // Kombi_01 (ICL_1)
    (0x365, 8, 5),   // BEM_05 (EGY_MNG_5)
    (0x3C0, 4, 10),  // Klemmen_Status_01 (EILU_1_VECU)
    (0x3DB, 8, 10),  // Gateway_72 (IF_CFT_72)
    (0x3DD, 8, 10),  // Gateway_74 (IF_CFT_74)
    (0x392, 8, 10),  // ESP_07 (TCS_7)
    (0x3B5, 8, 10),  // Klima_11 (CCU_11)
    (0x3E5, 8, 10),  // TSG_FT_02 driver door (DSC_LE_2)
    (0x3D6, 8, 10),  // Licht_hinten_01 (LAMP_RE_VECU)
    (0x520, 8, 20),  // Airbag_02 (ACU_2)
    (0x1A2, 8, 20),  // ESP_15 (TCS_15)
    (0x1AB, 8, 20),  // ESP_33 (TCS_33)
    (0x65E, 8, 50),  // OBD_Tankgeber_01 (FTL_SENS_1)
    (0x65D, 8, 100), // ESP_20 (TCS_20)
    (0x585, 8, 100), // Systeminfo_01 (SYS_INFO_1)
    (0x385, 8, 100), // Charisma_01 (SPT_1_IF)
    (0x65A, 8, 100), // BCM_01 (VECU_1)
    (0x6B2, 8, 100), // Diagnose_01 (DIAG_1)
    (0x6B7, 8, 100), // Kombi_02 (ICL_2)
    (0x6B8, 8, 100), // Kombi_03 (ICL_3)
];

fn finite_or(x: f64, default: f64) -> f64 {
    if x.is_finite() {
        x
    } else {
        default
    }
}

impl ChassisInputs {
    /// Replaces non-finite (`NaN`/`Inf`) fields with safe physical defaults and normalises speeds to
    /// non-negative magnitudes so a corrupt external telemetry sample can never poison encoder state
    /// or emit false MQB error sentinels.
    fn sanitized(&self) -> Self {
        Self {
            wheel_kmh: self.wheel_kmh.map(|w| finite_or(w, 0.0).abs()),
            vehicle_kmh: finite_or(self.vehicle_kmh, 0.0).abs(),
            backward: self.backward,
            accel_ms2: finite_or(self.accel_ms2, 0.0),
            brake_pressure_bar: finite_or(self.brake_pressure_bar, 0.0),
            brake_pedal_pressed: self.brake_pedal_pressed,
            selector_code: self.selector_code,
            drive_profile: self.drive_profile,
            terminal_15: self.terminal_15,
            terminal_50_request: self.terminal_50_request,
            engine_running: self.engine_running,
            awd: self.awd,
            alternator_w: finite_or(self.alternator_w, 0.0).max(0.0),
            alternator_util_pct: finite_or(self.alternator_util_pct, 0.0).clamp(0.0, 100.0),
            ambient_c: finite_or(self.ambient_c, 20.0),
            tyre_radius_m: if self.tyre_radius_m.is_finite()
                && (0.1..=0.65).contains(&self.tyre_radius_m)
            {
                self.tyre_radius_m
            } else {
                0.316
            },
            odometer_km: finite_or(self.odometer_km, 0.0).max(0.0),
            soak_s: finite_or(self.soak_s, 0.0).max(0.0),
        }
    }
}

/// Intel (little-endian) signal: `len` bits of `v` at bit `start`.
fn put(d: &mut [u8; 8], start: u32, len: u32, v: u64) {
    if len == 0 || start >= 64 {
        return;
    }
    let width = len.min(64 - start);
    let low_mask = if width >= 64 {
        u64::MAX
    } else {
        (1u64 << width) - 1
    };
    let m = low_mask << start;
    let mut w = u64::from_le_bytes(*d);
    w = (w & !m) | ((v << start) & m);
    *d = w.to_le_bytes();
}

/// Raw value of a physical value: `(x - offset) / factor`, rounded and clamped to `len` bits.
fn raw(x: f64, factor: f64, offset: f64, len: u32) -> u64 {
    if !x.is_finite() || !factor.is_finite() || factor == 0.0 || !offset.is_finite() || len == 0 {
        return 0;
    }
    let max_raw = if len >= 64 {
        u64::MAX as f64
    } else {
        ((1u64 << len) - 1) as f64
    };
    let scaled = ((x - offset) / factor).round();
    if !scaled.is_finite() {
        return if scaled.is_sign_positive() {
            max_raw as u64
        } else {
            0
        };
    }
    scaled.clamp(0.0, max_raw) as u64
}

fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = y - i64::from(m <= 2);
    let era = y.div_euclid(400);
    let yoe = (y - era * 400) as u64;
    let m_off = if m > 2 { m - 3 } else { m + 9 } as u64;
    let doy = (153 * m_off + 2) / 5 + u64::from(d) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe as i64
}

fn civil_from_days(z: i64) -> (u32, u32, u32) {
    let era = z.div_euclid(146_097);
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = (y + i64::from(m <= 2)).clamp(2000, 2127) as u32;
    (y, m as u32, d as u32)
}

/// Encodes the frames of the non-emulated modules, one call per 10 ms bus period.
#[derive(Default)]
pub struct MqbChassisEncoder {
    tick: u32,
    /// Message counter per entry of [`FRAMES`].
    counters: [u8; FRAMES.len()],
    /// Last `Getriebe_06` (0x128, TCU -> selector module) seen on the bus: bit 2 `GE_Test_Freigabe`,
    /// bytes 1/2 `GE_Testparameter_1/2` (layout: opendbc `vw_mqb.dbc`; firmware TX buffer `0xD00078F0`
    /// filled at `0x800CC20A..0x800CC26A`).
    getriebe_06: [u8; 3],
    /// Cluster: engine standing time, s (`None` until the first call: then the soak time).
    standing_s: Option<f64>,
    kl15: bool,
    /// Cluster: distance driven in this session, km.
    distance_km: f64,
    /// ESP: wheel pulses counted per wheel (FL, FR, RL, RR).
    pulses: [f64; 4],
}

impl MqbChassisEncoder {
    /// `ESP_21` E2E data-ID table (per counter value), confirmed in DQ200_G2 flash @ 0x8013942C.
    pub const ESP_21_DATA_IDS: [u8; 16] = data_id::ESP_21;

    /// A frame the TCU put on the bus: the selector module listens to `Getriebe_06` (0x128).
    pub fn on_tcu_frame(&mut self, id: u32, data: &[u8; 8]) {
        if id == 0x128 {
            self.getriebe_06.copy_from_slice(&data[..3]);
        }
    }

    /// Resets the physical trip/soak state after a bench plant reset (`Kombi_02` standing time re-latches `ChassisInputs::soak_s`
    /// on the next [`Self::encode`] call, session trip distance and wheel pulse accumulators clear) while advancing `tick` to the
    /// next 300-tick (3 s) super-period so all 31 cyclic frames emit immediately on the first slice with monotonic `BZ` counters.
    pub fn reset_soak(&mut self) {
        self.standing_s = None;
        self.kl15 = false;
        self.distance_km = 0.0;
        self.pulses = [0.0; 4];
        self.getriebe_06 = [0; 3];
        self.tick = self.tick.wrapping_add((300 - self.tick % 300) % 300);
    }

    /// Returns `(id, dlc, data)` of the frames due in this 10 ms bus period (each at its configured cycle).
    pub fn encode(&mut self, c: &ChassisInputs) -> Vec<(u32, u8, [u8; 8])> {
        let c = c.sanitized();
        self.update(&c);
        let tick = self.tick;
        self.tick = self.tick.wrapping_add(1);
        let mut out = Vec::new();
        for (i, &(id, dlc, period)) in FRAMES.iter().enumerate() {
            if !tick.is_multiple_of(period) {
                continue;
            }
            let bz = self.counters[i];
            self.counters[i] = (bz + 1) & 0x0F;
            out.push((id, dlc, self.frame(id, dlc, bz, &c)));
        }
        out
    }

    /// The cluster's and the ESP's own counters, advanced by one 10 ms period.
    fn update(&mut self, c: &ChassisInputs) {
        let s = self.standing_s.get_or_insert(c.soak_s.max(0.0));
        // counts from ignition off (CAN terminal 15) to the next engine start (FR p.8741 / p.19551)
        if self.kl15 && !c.terminal_15 {
            *s = 0.0;
        }
        if !c.engine_running {
            *s += TICK_S;
        }
        self.kl15 = c.terminal_15;
        self.distance_km += c.vehicle_kmh.abs() * TICK_S / 3600.0;
        let circ = circumference_m(c);
        for (p, v) in self.pulses.iter_mut().zip(c.wheel_kmh) {
            *p = (*p + v.abs() / 3.6 * TICK_S / circ * ABS_TEETH as f64) % 1001.0;
        }
    }

    fn frame(&self, id: u32, dlc: u8, bz: u8, c: &ChassisInputs) -> [u8; 8] {
        let mut d = [0u8; 8];
        let e2e = |d: &mut [u8; 8], ids: &[u8; 16]| {
            put(d, 8, 4, bz as u64);
            d[0] = mqb_crc8_p2f(&d[1..dlc as usize], ids[bz as usize]);
        };
        let kmh = |v: f64, lsb: f64| raw(v.clamp(0.0, 320.0), lsb, 0.0, 16);
        let standstill = c.vehicle_kmh.abs() < 0.1;
        let fuel_raw = raw(FUEL_L, 1.0, 0.0, 7);
        let circ_mm = ((circumference_m(c) * 1000.0).round() as u64).min(0xFFD);
        match id {
            // ESP_19: RL, RR, FL, FR at 0.0075 km/h/bit (no E2E).
            0x0B2 => {
                for (k, w) in [2, 3, 0, 1].into_iter().enumerate() {
                    put(&mut d, 16 * k as u32, 16, kmh(c.wheel_kmh[w], 0.0075));
                }
            }
            // LWI_01: steering wheel straight, angle/rate 0, sensor OK, QBits 0 = valid (the plant has no steering).
            0x086 => e2e(&mut d, &data_id::LWI_01),
            // Waehlhebel_03 (DLC 4, the DQ200_G2 COM table reads the same fields, signals 0x37/0x38/0xC2/0xC6/0xC3/0xC1/0x3A/
            // 0x39): WH_SensorPos_roh 4..7, WH_03_BZ 8..11, WH_SensorPos_roh_inv 12..15, WH_Testergebnis 16..23,
            // WH_Test_Aktiv 24, WH_Status 25..31 (0 = no fault; its bits are the TCU's events 6..13, `0x80073142`).
            // Test exchange with the TCU (state machine 0x527F, `0x80072CC4`/`0x80072E5A`): while `GE_Test_Freigabe` is set the
            // selector module reports `WH_Test_Aktiv` and returns `WH_Testergebnis` = `GE_Testparameter_1 ^ GE_Testparameter_2`
            // — the only answer the TCU accepts (`0x80072D6A..0x80072D80`); a missing / wrong answer is its event 18 (P0919)
            // / 19, and a test not completed in time is event 2 (P0914, counter `0xD000525D`, `0x80072F92`). What the real
            // module computes internally is not known beyond satisfying this check (UNVERIFIED).
            0x0AF => {
                let sel = c.selector_code & 0x0F;
                let [ge0, p1, p2] = self.getriebe_06;
                let test = ge0 & 0x04 != 0;
                d = [sel << 4, ((!sel & 0x0F) << 4) | bz, if test { p1 ^ p2 } else { 0 }, test as u8, 0, 0, 0, 0];
            }
            // ESP_21: BR_Eingriffsmoment 12|10 = 0 Nm (raw 509, offset −509: no brake intervention), ESP_v_Signal 32|16
            // 0.01 km/h, QBit 55 = 0 (valid).
            0x0FD => {
                put(&mut d, 12, 10, 509);
                put(&mut d, 32, 16, kmh(c.vehicle_kmh, 0.01));
                e2e(&mut d, &data_id::ESP_21);
            }
            // ESP_02: lateral acceleration 16|8 (0.01 g, −1.27) = 0, longitudinal 24|10 (0.03125 m/s², −16), yaw rate 0,
            // standstill flag 15, QBits 12..14 = 0 (valid).
            0x101 => {
                put(&mut d, 15, 1, standstill as u64);
                put(&mut d, 16, 8, raw(0.0, 0.01, -1.27, 8));
                put(&mut d, 24, 10, raw(c.accel_ms2.clamp(-16.0, 15.9), 0.03125, -16.0, 10));
                e2e(&mut d, &data_id::ESP_02);
            }
            // ESP_05: ESP_Bremsdruck 16|10 (0.3 bar, −30; DQ250 0D9300012F decodes raw·0.3 − 30 bar and treats 0x3FF and
            // QBit 12 as invalid, `0x80086328..0x800863B2`, VERIFIED-S), ESP_Fahrer_bremst 26 (DBC). The flag used to be
            // sent in bit 36 (ESP_HDC_Standby by the DBC).
            0x106 => {
                put(&mut d, 16, 10, raw(c.brake_pressure_bar.clamp(0.0, 276.0), 0.3, -30.0, 10));
                put(&mut d, 26, 1, c.brake_pedal_pressed as u64);
                e2e(&mut d, &data_id::ESP_05);
            }
            // ESP_10: wheel pulse counters VL 16|10, VR 26|10, HL 36|10, HR 46|10 (0..1000, wrapping), direction per wheel
            // 56.. (2 bit: 1 forward, 2 backward, 0 at standstill — HYP), QBits 12..15 = 0.
            0x116 => {
                for (k, p) in self.pulses.iter().enumerate() {
                    put(&mut d, 16 + 10 * k as u32, 10, *p as u64);
                    let dir = if c.wheel_kmh[k].abs() < 0.1 { 0 } else if c.backward { 2 } else { 1 };
                    put(&mut d, 56 + 2 * k as u32, 2, dir);
                }
                e2e(&mut d, &data_id::ESP_10);
            }
            // Airbag_01: no crash, no lamp, no fault.
            0x040 => e2e(&mut d, &data_id::AIRBAG_01),
            // Kombi_01: lamps off, Kombi_01_BZ 8|4 (no CRC), KBI_V_Digital 24|9 km/h, KBI_angez_Geschw 48|10 (0.32 km/h; the
            // needle's over-reading is not modelled), KBI_Tankwarnung 16 (reserve, 7 L UNVERIFIED).
            0x30B => {
                let v_kmh = c.vehicle_kmh.clamp(0.0, 320.0);
                put(&mut d, 8, 4, bz as u64);
                // KBI_Anzeigestatus_GRA 13|2 and _LIM 35|2 = 1, "set-speed display available" (FR p.19451–19452): the ECU's
                // speed limiter is coded and expects the cluster's release (LMVDes_bDispLim, U1122, VERIFIED-T).
                put(&mut d, 13, 2, 1);
                put(&mut d, 35, 2, 1);
                put(&mut d, 16, 1, (FUEL_L < 7.0) as u64);
                put(&mut d, 24, 9, raw(v_kmh, 1.0, 0.0, 9));
                put(&mut d, 48, 10, raw(v_kmh, 0.32, 0.0, 10));
            }
            // Klemmen_Status_01 (DLC 4): ZAS_Kl_S 16, ZAS_Kl_15 17, ZAS_Kl_X 18, ZAS_Kl_50 (start request) 19.
            0x3C0 => {
                let k = c.terminal_15 as u64;
                put(&mut d, 16, 3, k | k << 1 | k << 2);
                put(&mut d, 19, 1, c.terminal_50_request as u64);
                e2e(&mut d, &data_id::KLEMMEN_STATUS_01);
            }
            // Gateway_72: every routed message present (`*_alt` = 0), BCM1_Aussen_Temp_ungef 56|8 (0.5 °C, −50) = plant ambient,
            // BCM1_OBD_FStatus_ATemp 52|4 = 0 (no sensor fault, HYP), doors closed.
            0x3DB => put(&mut d, 56, 8, raw(c.ambient_c.clamp(-50.0, 75.0), 0.5, -50.0, 8)),
            // Gateway_74: routed messages present, no paddle / start-stop request.
            0x3DD => {}
            // ESP_07: tyre tolerance adaptation 0 % (raw 127, 0.0488 %, −6.2), no wheel-sensor fault, ESP_Quattro_Antrieb 13.
            0x392 => {
                put(&mut d, 13, 1, c.awd as u64);
                for k in 0..3 {
                    put(&mut d, 16 + 8 * k, 8, raw(0.0, 0.0488281, -6.20117, 8));
                }
                e2e(&mut d, &data_id::ESP_07);
            }
            // ESP_20: ESP_Zaehnezahl 16|8, BR_Reifenumfang 52|12 mm, its QBit 51 = 0.
            0x65D => {
                put(&mut d, 16, 8, ABS_TEETH as u64);
                put(&mut d, 52, 12, circ_mm);
                e2e(&mut d, &data_id::ESP_20);
            }
            // Systeminfo_01: no unit reports a fault, no roller/transport mode.
            0x585 => {}
            // Diagnose_01: KBI_Kilometerstand 8|20, clock UH_Jahr 28|7 (+2000), UH_Monat 35|4, UH_Tag 39|5, UH_Stunde 44|5,
            // UH_Minute 49|6, UH_Sekunde 55|6.
            0x6B2 => {
                put(&mut d, 8, 20, self.odometer(c));
                let (y0, mo0, day0, t0) = CLOCK0;
                let t = t0 + self.tick as f64 * TICK_S;
                let s = t as u64;
                let base_days = days_from_civil(y0 as i64, mo0, day0);
                let (y, mo, day) = civil_from_days(base_days + (s / 86_400) as i64);
                put(&mut d, 28, 7, (y - 2000) as u64);
                put(&mut d, 35, 4, mo as u64);
                put(&mut d, 39, 5, day as u64);
                put(&mut d, 44, 5, s / 3600 % 24);
                put(&mut d, 49, 6, s / 60 % 60);
                put(&mut d, 55, 6, s % 60);
            }
            // Kombi_02: KBI_Kilometerstand 0|20, KBI_Standzeit_02 20|17 s (0..0x1FFFC; 0x1FFFD..F = n/a, init, error), KBI_Inhalt_Tank
            // 40|7 L (0x7E init, 0x7F error), KBI_FStatus_Tank 47 = 0 (level OK), KBI_QBit_Aussen_Temp_gef 55 = 0 (valid),
            // KBI_Aussen_Temp_gef 56|8 (0.5 °C, −50; 0xFD..FF n/a, init, error). FR p.19542–19551.
            0x6B7 => {
                put(&mut d, 0, 20, self.odometer(c));
                put(&mut d, 20, 17, (self.standing_s.unwrap_or(0.0) as u64).min(0x1FFFC));
                put(&mut d, 40, 7, fuel_raw);
                put(&mut d, 56, 8, raw(c.ambient_c.clamp(-50.0, 75.0), 0.5, -50.0, 8));
            }
            // Kombi_03: KBI_Reifenumfang 0|12 mm, KBI_Tankfuellstand_Prozent 18|7, KBI_Tankinhalt_hochaufl 26|14 (0.01 L),
            // KBI_Max_Tankinhalt 40|8 (0.5 L), KBI_Reifenumfang_Sekundaer 48|12.
            0x6B8 => {
                put(&mut d, 0, 12, circ_mm);
                put(&mut d, 18, 7, raw(100.0 * FUEL_L / TANK_L, 1.0, 0.0, 7));
                put(&mut d, 26, 14, raw(FUEL_L, 0.01, 0.0, 14));
                put(&mut d, 40, 8, raw(TANK_L, 0.5, 0.0, 8));
                put(&mut d, 48, 12, circ_mm);
            }
            // Units the engine ECU's coding expects (bench DFlash; its timeout monitors COM_*_TOUT fail without them,
            // VERIFIED-T), idle: what they send while the driver uses none of their functions.
            // EPB_01: longitudinal acceleration 24|8 (%g, −128), no clutch pedal (DSG), brake released, no fault.
            0x104 => {
                put(&mut d, 24, 8, raw(c.accel_ms2 / 9.81 * 100.0, 1.0, -128.0, 8).min(0xFD));
                e2e(&mut d, &data_id::EPB_01);
            }
            // ACC_06 with ACC switched off: ACC_Status_ACC 0, no acceleration request (ACC_Sollbeschleunigung_02 = 3.01 m/s²,
            // the "no request" value), comfort band 0.2 m/s², jerk limits 0 — what openpilot sends for an inactive ACC
            // (opendbc `mqbcan.create_acc_accel_control`). ACC_Typ 58|2 = 2: the type the ECU's ACC coding accepts (with 0 its
            // coding check VLCAvl_stCodFail, P1647, fails; 0x8016A4A0..0x8016A854, the type lands in 0xD001AD00, VERIFIED-T);
            // what types 1..3 mean is not verified.
            0x122 => {
                put(&mut d, 16, 6, raw(0.2, 0.024, 0.0, 6));
                put(&mut d, 24, 11, raw(3.01, 0.005, -7.22, 11));
                put(&mut d, 35, 5, raw(0.2, 0.0625, 0.0, 5));
                put(&mut d, 58, 2, 2);
                e2e(&mut d, &data_id::ACC_06);
            }
            // GRA_ACC_01: the ACC stalk, no key pressed. Its configuration fields (FR p.14684–14700; positions: opendbc MEB
            // `GRA_ACC_01`): GRA_Hauptschalter 12 = 1 (latched main switch in ON, how the driver left it — with OFF the ECU
            // reports LMLev_bMainSwt, P0568), GRA_Typ_Hauptschalter 14 = 0 (latched), GRA_Codierung 22|2 = 2 ("ACC operating
            // unit"; the ECU checks it against its ACC coding: 0xD001AD57, `d15 == 2` at 0x8016A4E6, VERIFIED-S/T),
            // GRA_LIM_Taste_verfuegbar 25 = 1 (the ECU's speed limiter is coded: LMLev_bLevLim, U1122, without it),
            // GRA_Typ_Bedienteil 27|3 = 2 (6-position unit; 0 = init → LMLev_bLevCod, P0568; 1..3 all accepted, which one the
            // car has is UNVERIFIED).
            0x12B => {
                put(&mut d, 12, 1, 1);
                put(&mut d, 22, 2, 2);
                put(&mut d, 25, 1, 1);
                put(&mut d, 27, 3, 2);
                e2e(&mut d, &data_id::GRA_ACC_01)
            }
            // BEM_05: generator power BEM_P_Generator 16|8 (50 W; "mechanical generator power, the ECU computes engine load
            // from it", 0xFF = error, FR p.16063) and utilisation BEM_DFM 32|5 (3.225 %, +0.025) = the plant's alternator;
            // no load-management request (no idle-speed raise, no shut-off stage). Sent as 0 W the ECU under-fuels the
            // alternator load at cold idle and the engine stalls (cold_start_dq250_5, VERIFIED-T).
            0x365 => {
                put(&mut d, 16, 8, raw(c.alternator_w, 50.0, 0.0, 8).min(0xFE));
                put(&mut d, 32, 5, raw(c.alternator_util_pct.max(0.025), 3.225, 0.025, 5));
            }
            // Klima_11: climate control with the compressor off (no AC request, no compressor torque, no fan load).
            0x3B5 => {}
            // TSG_FT_02: driver door closed.
            0x3E5 => e2e(&mut d, &data_id::TSG_FT_02),
            // Licht_hinten_01: Licht_hinten_01_BZ 0|4 (no CRC), brake lights 11 = brake pedal, reversing lights 13 = selector R,
            // no lamp defect.
            0x3D6 => {
                put(&mut d, 0, 4, bz as u64);
                put(&mut d, 11, 1, c.brake_pedal_pressed as u64);
                put(&mut d, 13, 1, (c.selector_code == 7) as u64);
            }
            // Airbag_02: seat-belt / occupancy information all 0 (UNVERIFIED meaning).
            0x520 => e2e(&mut d, &data_id::AIRBAG_02),
            // ESP_15: content unknown (DBC: only CRC/BZ); the engine ECU monitors its timeout only (FR TCS_15). CRC data ID
            // unknown: byte 0 left 0 (UNVERIFIED).
            0x1A2 => put(&mut d, 8, 4, bz as u64),
            // ESP_33: content unknown (DBC: only CRC/BZ), E2E per FR TCS_33.
            0x1AB => e2e(&mut d, &data_id::ESP_33),
            // OBD_Tankgeber_01: fuel level sender raw resistances and status not modelled — all 0 (UNVERIFIED).
            0x65E => {}
            // Charisma_01: target drive profile of every unit = the driver's selection (`drive_profile`).
            0x385 => {
                for k in 0..16 {
                    if k != 3 {
                        put(&mut d, 4 * k, 4, (c.drive_profile & 0x0F) as u64);
                    }
                }
            }
            // BCM_01: no warning (brake pads, brake fluid, washer, coolant level sensors OK).
            0x65A => {}
            _ => unreachable!("frame {id:03X} not in FRAMES"),
        }
        d
    }

    fn odometer(&self, c: &ChassisInputs) -> u64 {
        raw(c.odometer_km + self.distance_km, 1.0, 0.0, 20).min(0xFFFFC)
    }
}

/// Rolling circumference, m (205/55 R16, r = 0.316 m, when the plant gives none).
fn circumference_m(c: &ChassisInputs) -> f64 {
    2.0 * std::f64::consts::PI * if c.tyre_radius_m > 0.1 { c.tyre_radius_m } else { 0.316 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc8h2f_check_value() {
        // CRC-8H2F over "123456789" = 0xDF (AUTOSAR). Our helper appends a data-ID byte, so feed
        // the first 8 bytes and use '9' as the "data ID".
        assert_eq!(mqb_crc8_p2f(b"12345678", b'9'), 0xDF);
    }

    /// Frames as the firmwares see them: ESP_21 and Motor_20-style E2E consistent (CRC over the counter's data ID),
    /// every frame at its own cycle with its own counter, signals where the DBC puts them.
    #[test]
    fn frames_cycles_counters_and_e2e() {
        let mut e = MqbChassisEncoder::default();
        let c = ChassisInputs { wheel_kmh: [50.0; 4], vehicle_kmh: 50.0, ambient_c: 20.0, terminal_15: true, odometer_km: 1234.0, soak_s: 300.0, ..Default::default() };
        let mut seen: std::collections::BTreeMap<u32, Vec<[u8; 8]>> = Default::default();
        for _ in 0..200 {
            for (id, dlc, d) in e.encode(&c) {
                assert!(dlc == 8 || matches!(id, 0x0AF | 0x3C0));
                seen.entry(id).or_default().push(d);
            }
        }
        for &(id, _, period) in &FRAMES {
            assert_eq!(seen[&id].len() as u32, 200_u32.div_ceil(period), "{id:03X} cycle");
        }
        // ESP_21: counter steps by one per frame, CRC with the data ID of that counter value, 50 km/h, 0 Nm intervention
        for (k, d) in seen[&0x0FD].iter().enumerate() {
            assert_eq!(d[1] & 0x0F, (k % 16) as u8);
            assert_eq!(d[0], mqb_crc8_p2f(&d[1..8], data_id::ESP_21[k % 16]));
            assert_eq!(u16::from_le_bytes([d[4], d[5]]), 5000);
            assert_eq!(u16::from_le_bytes([d[1], d[2]]) >> 4 & 0x3FF, 509);
        }
        // Klemmen_Status_01 over its 4 bytes
        let k = seen[&0x3C0][3];
        assert_eq!((k[0], k[2] & 0x0F), (mqb_crc8_p2f(&k[1..4], 0xC3), 0x07));
        // Kombi_02: odometer, standing time (soak, frozen? no: engine not running → counting), 20 °C = raw 140
        let k2 = seen[&0x6B7][1];
        let w = u64::from_le_bytes(k2);
        assert_eq!((w & 0xFFFFF, w >> 56), (1234, 140));
        assert!((((w >> 20) & 0x1FFFF) as f64 - 301.0).abs() < 1.5);
    }

    /// Standing time: frozen while the engine runs, restarts from 0 at ignition off.
    #[test]
    fn cluster_standing_time() {
        let mut e = MqbChassisEncoder::default();
        let mut c = ChassisInputs { terminal_15: true, soak_s: 100.0, ..Default::default() };
        e.encode(&c);
        c.engine_running = true;
        for _ in 0..500 {
            e.encode(&c);
        }
        assert!((e.standing_s.unwrap() - 100.01).abs() < 1e-6);
        c.engine_running = false;
        c.terminal_15 = false;
        for _ in 0..100 {
            e.encode(&c);
        }
        assert!((e.standing_s.unwrap() - 1.0).abs() < 1e-6);
    }

    /// Bench soak reset re-latches `Kombi_02` standing time from the new `soak_s` (even if KL15 was previously on) and
    /// preserves rolling E2E message counters.
    #[test]
    fn reset_soak_relatches_standing_time_and_keeps_counters() {
        let mut e = MqbChassisEncoder::default();
        let warm_run = ChassisInputs {
            terminal_15: true,
            engine_running: true,
            vehicle_kmh: 60.0,
            wheel_kmh: [60.0; 4],
            soak_s: 300.0,
            ..Default::default()
        };
        let mut last_esp_bz = 0u8;
        for _ in 0..50 {
            for (id, _, d) in e.encode(&warm_run) {
                if id == 0x0FD {
                    last_esp_bz = d[1] & 0x0F;
                }
            }
        }
        // Reset bench to a 12 h cold soak while KL15 is now off: standing_s must latch 43200 s (not wipe to 0 on the
        // prior `kl15 == true` edge) and ESP_21's 4-bit counter must step monotonically by +1.
        e.reset_soak();
        let cold_off = ChassisInputs { terminal_15: false, soak_s: 43_200.0, ..Default::default() };
        let frames = e.encode(&cold_off);
        let esp = frames.iter().find(|f| f.0 == 0x0FD).expect("ESP_21").2;
        assert_eq!(esp[1] & 0x0F, (last_esp_bz + 1) & 0x0F);
        assert!((e.standing_s.unwrap() - 43_200.01).abs() < 1e-6);

        // Reset again to a 5 min warm soak with KL15 kept on: all 31 frames (incl. Kombi_02 0x6B7) emit on the first slice.
        e.reset_soak();
        let warm_on = ChassisInputs { terminal_15: true, soak_s: 300.0, ..Default::default() };
        let first_slice = e.encode(&warm_on);
        assert_eq!(first_slice.len(), FRAMES.len(), "all cyclic frames must emit on the first slice after reset_soak");
        let kombi_02 = first_slice.iter().find(|f| f.0 == 0x6B7).expect("Kombi_02 on first slice").2;
        let w = u64::from_le_bytes(kombi_02);
        let reported_s = ((w >> 20) & 0x1FFFF) as f64;
        assert!((reported_s - 300.0).abs() <= 1.0, "expected ~300 s, got {reported_s}");
    }

    /// Selector module answers the TCU's Getriebe_06 test with Test_Aktiv and P1 ^ P2 (the check the
    /// DQ200_G2 firmware makes at 0x80072D6A), and nothing while no test is requested.
    #[test]
    fn selector_module_answers_the_getriebe_06_test() {
        let mut e = MqbChassisEncoder::default();
        let ci = ChassisInputs { selector_code: 8, ..Default::default() };
        let wh = |e: &mut MqbChassisEncoder| e.encode(&ci).into_iter().find(|f| f.0 == 0x0AF).unwrap();
        let (_, dlc, d) = wh(&mut e);
        assert_eq!((dlc, d[0] >> 4, d[1] >> 4, d[2], d[3]), (4, 8, 7, 0, 0));
        e.on_tcu_frame(0x128, &[0x85, 0x5A, 0x3C, 0, 0, 0, 0, 0]);
        let (_, _, d) = wh(&mut e);
        assert_eq!((d[2], d[3]), (0x5A ^ 0x3C, 1));
        e.on_tcu_frame(0x128, &[0x81, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(wh(&mut e).2[3], 0);
    }

    #[test]
    fn crc8h2f_table_matches_bitwise_reference() {
        fn bitwise(payload: &[u8], id: u8) -> u8 {
            let mut crc: u8 = 0xFF;
            for &b in payload.iter().chain(std::iter::once(&id)) {
                crc ^= b;
                for _ in 0..8 {
                    crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x2F } else { crc << 1 };
                }
            }
            crc ^ 0xFF
        }
        for id in 0..=255u8 {
            let p = [id, id.wrapping_mul(3), id ^ 0x5A, 0xA5, 0, 0xFF, id.rotate_left(3)];
            assert_eq!(mqb_crc8_p2f(&p, id), bitwise(&p, id));
        }
    }

    #[test]
    fn nan_inf_inputs_do_not_poison_state_or_emit_extreme_offsets() {
        let mut e = MqbChassisEncoder::default();
        let bad = ChassisInputs {
            wheel_kmh: [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::NAN],
            vehicle_kmh: f64::NAN,
            backward: false,
            accel_ms2: f64::NAN,
            brake_pressure_bar: f64::NAN,
            brake_pedal_pressed: false,
            selector_code: 8,
            drive_profile: 2,
            terminal_15: true,
            terminal_50_request: false,
            engine_running: false,
            awd: false,
            alternator_w: f64::NAN,
            alternator_util_pct: f64::NAN,
            ambient_c: f64::NAN,
            tyre_radius_m: f64::INFINITY,
            odometer_km: f64::NAN,
            soak_s: f64::INFINITY,
        };
        let frames: std::collections::BTreeMap<u32, [u8; 8]> =
            e.encode(&bad).into_iter().map(|(id, _, d)| (id, d)).collect();

        // ESP_02: 0.0 m/s² -> (0 - (-16)) / 0.03125 = 512 (not 0 -> -16 m/s²)
        let esp02 = u64::from_le_bytes(frames[&0x101]);
        assert_eq!((esp02 >> 24) & 0x3FF, 512);

        // ESP_05: 0.0 bar -> (0 - (-30)) / 0.3 = 100 (not 0 -> -30 bar)
        let esp05 = u64::from_le_bytes(frames[&0x106]);
        assert_eq!((esp05 >> 16) & 0x3FF, 100);

        // EPB_01: 0 %g -> 128 (not 0 -> -128 %g)
        assert_eq!(frames[&0x104][3], 128);

        // Gateway_72 & Kombi_02: default 20.0 °C -> (20 - (-50)) / 0.5 = 140
        assert_eq!(frames[&0x3DB][7], 140);
        assert_eq!(frames[&0x6B7][7], 140);

        // ESP_20: default 0.316 m circumference -> ~1985 mm (never 0xFFF invalid sentinel)
        let esp20 = u64::from_le_bytes(frames[&0x65D]);
        assert_eq!((esp20 >> 52) & 0xFFF, 1985);

        // Subsequent finite ticks must integrate cleanly without NaN state poisoning.
        let good = ChassisInputs {
            wheel_kmh: [36.0; 4],
            vehicle_kmh: 36.0,
            terminal_15: true,
            ..Default::default()
        };
        for _ in 0..100 {
            e.encode(&good);
        }
        assert!(e.distance_km.is_finite() && e.distance_km > 0.009);
        assert!(e.pulses.iter().all(|p| p.is_finite() && *p > 0.0));
        assert!(e.standing_s.unwrap().is_finite());
    }

    #[test]
    fn negative_speeds_and_extreme_values_stay_consistent_and_below_error_sentinels() {
        let mut e = MqbChassisEncoder::default();
        let rev = ChassisInputs {
            wheel_kmh: [-18.0; 4],
            vehicle_kmh: -18.0,
            backward: true,
            ambient_c: 120.0,
            accel_ms2: 50.0,
            tyre_radius_m: 5.0,
            ..Default::default()
        };
        let frames: std::collections::BTreeMap<u32, [u8; 8]> =
            e.encode(&rev).into_iter().map(|(id, _, d)| (id, d)).collect();

        // Reverse at -18 km/h: ESP_19 (0.0075), ESP_21 (0.01), Kombi_01 (1.0 & 0.32), and ESP_10 all agree on 18 km/h.
        assert_eq!(u16::from_le_bytes([frames[&0x0B2][0], frames[&0x0B2][1]]), 2400);
        assert_eq!(u16::from_le_bytes([frames[&0x0FD][4], frames[&0x0FD][5]]), 1800);
        let k1 = u64::from_le_bytes(frames[&0x30B]);
        assert_eq!((k1 >> 24) & 0x1FF, 18);
        assert_eq!((k1 >> 48) & 0x3FF, 56);
        assert_eq!(frames[&0x116][7] & 0x03, 2); // backward direction

        // Extreme finite clamps stay strictly below MQB init/error sentinels.
        assert_eq!(frames[&0x3DB][7], 250); // <= 0xFA (< 0xFD)
        assert_eq!(frames[&0x104][3], 0xFD); // < 0xFE

        let mut e2 = MqbChassisEncoder::default();
        let fast = ChassisInputs {
            vehicle_kmh: 999.0,
            wheel_kmh: [999.0; 4],
            ..Default::default()
        };
        let f2: std::collections::BTreeMap<u32, [u8; 8]> =
            e2.encode(&fast).into_iter().map(|(id, _, d)| (id, d)).collect();
        let k1_fast = u64::from_le_bytes(f2[&0x30B]);
        assert!(((k1_fast >> 24) & 0x1FF) < 0x1FE);
        assert!(((k1_fast >> 48) & 0x3FF) < 0x3FE);
    }

    #[test]
    fn diagnose_01_calendar_advances_across_month_and_year_boundaries() {
        let base = days_from_civil(2026, 11, 4);
        assert_eq!(civil_from_days(base), (2026, 11, 4));
        assert_eq!(civil_from_days(base + 26), (2026, 11, 30));
        assert_eq!(civil_from_days(base + 27), (2026, 12, 1));
        assert_eq!(civil_from_days(base + 58), (2027, 1, 1));

        let e = MqbChassisEncoder {
            tick: 28 * 8_640_000, // +28 days from 2026-11-04 12:00 -> 2026-12-02 12:00
            ..Default::default()
        };
        let d = e.frame(0x6B2, 8, 0, &ChassisInputs::default());
        let w = u64::from_le_bytes(d);
        let yr = ((w >> 28) & 0x7F) + 2000;
        let mo = (w >> 35) & 0x0F;
        let day = (w >> 39) & 0x1F;
        assert_eq!((yr, mo, day), (2026, 12, 2));
    }
}
