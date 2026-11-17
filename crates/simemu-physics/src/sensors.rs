//! Sensor transfer functions: physical quantity → electrical signal seen by the ECU pins.
//!
//! Analog engine sensors: the characteristic of sensor + ECU input circuit is the inverse of the conversion
//! the SC800S50 firmware applies to the pin voltage (VERIFIED-S: calibration name and CAL address at each
//! table; voltages in the firmware's 5 V / 32768 units). The tables describe the hardware the calibration
//! was made for: the plant holds them as data and never reads the firmware's calibration at run time (a
//! modified calibration does not change the sensors). Round trip through the unmodified firmware:
//! `studio::io::tests::sensor_characteristics_round_trip_through_the_firmware`.

/// Piecewise-linear interpolation in `t` (x ascending), clamped at both ends.
pub fn interp(x: f64, t: &[(f64, f64)]) -> f64 {
    if t.is_empty() {
        return 0.0;
    }
    if t.len() == 1 || !x.is_finite() {
        return t[0].1;
    }
    let i = t.partition_point(|p| p.0 < x).clamp(1, t.len() - 1);
    let ((x0, y0), (x1, y1)) = (t[i - 1], t[i]);
    if x1 == x0 {
        return y0;
    }
    y0 + (x.clamp(t[0].0, t[t.len() - 1].0) - x0) * (y1 - y0) / (x1 - x0)
}

/// Bilinear interpolation in the map `z[iy][ix]` over ascending axes `xs` / `ys`, clamped at the edges.
pub fn interp2<const N: usize>(x: f64, y: f64, xs: &[f64; N], ys: &[f64], z: &[[f64; N]]) -> f64 {
    let m = ys.len().min(z.len());
    if N == 0 || m == 0 {
        return 0.0;
    }
    let x = if x.is_finite() { x } else { xs[0] };
    let y = if y.is_finite() { y } else { ys[0] };
    let row_at = |r: &[f64; N]| {
        if N == 1 {
            return r[0];
        }
        let j = xs.partition_point(|&xv| xv < x).clamp(1, N - 1);
        let (x0, x1) = (xs[j - 1], xs[j]);
        if x1 == x0 {
            return r[j - 1];
        }
        r[j - 1] + (x.clamp(xs[0], xs[N - 1]) - x0) * (r[j] - r[j - 1]) / (x1 - x0)
    };
    if m == 1 {
        return row_at(&z[0]);
    }
    let i = ys[..m].partition_point(|&yv| yv < y).clamp(1, m - 1);
    let (y0, y1) = (ys[i - 1], ys[i]);
    let (z0, z1) = (row_at(&z[i - 1]), row_at(&z[i]));
    if y1 == y0 {
        return z0;
    }
    z0 + (y.clamp(ys[0], ys[m - 1]) - y0) * (z1 - z0) / (y1 - y0)
}

/// Sensor voltage for the physical value `y` from a firmware conversion table (voltage ascending, value
/// monotone): the inverse of the table, clamped to its end voltages.
pub fn inverse_v(y: f64, t: &[(f64, f64)]) -> f64 {
    let n = t.len();
    if n == 0 {
        return 0.0;
    }
    if n == 1 || !y.is_finite() {
        return t[0].0;
    }
    if t[0].1 <= t[n - 1].1 {
        let i = t.partition_point(|p| p.1 < y).clamp(1, n - 1);
        let ((v0, y0), (v1, y1)) = (t[i - 1], t[i]);
        if y1 == y0 {
            return v0;
        }
        v0 + (y.clamp(t[0].1, t[n - 1].1) - y0) * (v1 - v0) / (y1 - y0)
    } else {
        let i = t.partition_point(|p| p.1 > y).clamp(1, n - 1);
        let ((v0, y0), (v1, y1)) = (t[i - 1], t[i]);
        if y1 == y0 {
            return v0;
        }
        v0 + (y.clamp(t[n - 1].1, t[0].1) - y0) * (v1 - v0) / (y1 - y0)
    }
}

/// Intake-air temperature NTC (TMAP G42, ADC0.CH10): `ip_tlts_air_mes[0]` @0x80810ADA (axis @0x80810396),
/// V → °C.
pub const IAT_TABLE: [(f64, f64); 16] = [
    (0.1324, 140.25), (0.1649, 130.5), (0.2065, 120.0), (0.2609, 110.25), (0.3320, 100.5), (0.4236, 90.0),
    (0.7082, 70.5), (1.1909, 50.25), (1.9244, 30.0), (2.3668, 20.25), (2.8464, 10.5), (3.3093, -0.75),
    (3.7149, -11.25), (4.0421, -21.0), (4.2799, -30.75), (4.4400, -40.5),
];

/// Coolant temperature NTC (G62, ADC2.CH7): `ip_tlts_mes[0]` @0x8082357A (axis @0x80822BAE), V → °C.
pub const CLT_TABLE: [(f64, f64); 20] = [
    (0.2361, 152.2266), (0.3270, 141.2422), (0.3799, 134.8438), (0.5180, 120.7812), (0.6439, 109.5391),
    (1.0062, 87.7344), (1.2424, 78.6953), (1.5318, 69.9062), (1.9016, 60.1484), (2.1001, 55.4141),
    (2.5999, 42.9609), (2.9440, 34.2656), (3.3743, 24.0), (4.1136, 6.4219), (4.2621, 1.3828),
    (4.3938, -3.7266), (4.5135, -9.7344), (4.6091, -15.1016), (4.6909, -20.6094), (4.9001, -38.4375),
];

/// TMAP manifold pressure (ADC0.CH11): `ip_prs_im_mes` @0x808099E4, 0.3999 V → 10.0 kPa, 4.648 V → 300 kPa.
pub const MAP_TABLE: [(f64, f64); 2] = [(0.39993, 9.99985), (4.64798, 300.0039)];
/// Ambient pressure sensor on the ECU board (ADC1.CH3): `ip_amp_mes` @0x80809878, 0.2 V → 15 kPa,
/// 4.7 V → 115 kPa.
pub const BARO_TABLE: [(f64, f64); 2] = [(0.20004, 14.99978), (4.70001, 114.99832)];
/// Charge-air pressure upstream of the throttle (G31, ADC1.CH2 — VERIFIED-T: OBD PID 0x70 follows that pin
/// only): `ip_prs_up_thr_mes` @0x80809A18, 0.3999 V → 20 kPa, 4.65 V → 300 kPa.
pub const PUT_TABLE: [(f64, f64); 2] = [(0.39993, 19.99971), (4.64996, 300.0039)];
/// Low-pressure fuel sender G410 (ADC2.CH2): `ip_fup_efp` @0x808018AE, 0.498 V → 1000 hPa, 4.502 V →
/// 11000 hPa (bar here). The 1000 hPa at 0.5 V is read as the absolute pressure of a gauge sender at
/// sea level; gauge vs absolute sensing is UNVERIFIED.
pub const LP_FUEL_TABLE: [(f64, f64); 2] = [(0.49805, 1.000317), (4.50195, 11.000834)];

pub fn map_sensor_v(p_kpa: f64) -> f64 {
    inverse_v(p_kpa, &MAP_TABLE)
}

pub fn baro_sensor_v(p_kpa: f64) -> f64 {
    inverse_v(p_kpa, &BARO_TABLE)
}

pub fn put_sensor_v(p_kpa: f64) -> f64 {
    inverse_v(p_kpa, &PUT_TABLE)
}

pub fn iat_sensor_v(t_c: f64) -> f64 {
    inverse_v(t_c, &IAT_TABLE)
}

pub fn coolant_sensor_v(t_c: f64) -> f64 {
    inverse_v(t_c, &CLT_TABLE)
}

/// Gauge pressure `p_bar`, see [`LP_FUEL_TABLE`].
pub fn lp_fuel_sensor_v(p_bar: f64) -> f64 {
    inverse_v(p_bar + 1.0, &LP_FUEL_TABLE)
}

/// High-pressure rail sensor (ADC2.CH3): linear, offset `c_vp_fup_mes_ofs` 0.5 V @0x808021FE, slope
/// `c_fup_mes_slop` 7.0001 MPa/V @0x80801E64 (14.0 MPa at 2.5 V and 3.5 MPa at 1.0 V on OBD PID 0x23,
/// VERIFIED-T).
pub fn rail_sensor_v(p_mpa: f64) -> f64 {
    if !p_mpa.is_finite() {
        return 0.5;
    }
    (0.5 + p_mpa / 7.000096).clamp(0.0, 5.0)
}

/// Throttle position sensors G187 (rising, ADC0.CH5) / G188 (falling, ADC0.CH6) for the plate angle in degrees
/// from the lower mechanical stop: lower stop `c_vp_tps_ad_bol_1/2` 0.5 / 4.5 V @0x8080C098/9A, slope
/// `c_tps_slop` 21 °TPS/V @0x8080C062 on both tracks (limp-home `c_vp_tps_sp_lih_1` 0.8333 V @0x8080C0C6 =
/// `c_tps_lih_ini` 7.0 °TPS @0x8080C050).
pub fn throttle_sensors_v(deg: f64) -> (f64, f64) {
    let d = if deg.is_finite() { deg } else { 0.0 };
    ((0.5 + d / 21.0).clamp(0.0, 5.0), (4.5 - d / 21.0).clamp(0.0, 5.0))
}

/// Pedal sensors G79 / G185 (second track at half voltage): slopes `c_vp_pvs_slop_1/2_mon` 36.5625 / 73.125
/// %/V @0x8080FDD4/D8; released position 0.75 V UNVERIFIED (below the idle thresholds
/// `c_vp_pvs_is_thd_1/2_mon` 0.908 / 0.449 V @0x8080FDCC/D0).
pub fn pedal_sensors_v(pct: f64) -> (f64, f64) {
    let p = if pct.is_finite() { pct.clamp(0.0, 100.0) } else { 0.0 };
    let a = (0.75 + p / 36.5625).clamp(0.0, 5.0);
    (a, a / 2.0)
}

/// Bosch LSU 4.9 pump current (mA) vs λ (datasheet table, linear interpolation).
pub fn lsu49_pump_current_ma(lambda: f64) -> f64 {
    const T: [(f64, f64); 11] = [
        (0.65, -2.22), (0.70, -1.82), (0.80, -1.11), (0.85, -0.80), (0.90, -0.50), (1.00, 0.0),
        (1.18, 0.33), (1.43, 0.67), (1.70, 0.94), (2.42, 1.38), (10.0, 2.54),
    ];
    if !lambda.is_finite() {
        return 0.0;
    }
    let l = lambda.clamp(T[0].0, T[T.len() - 1].0);
    for w in T.windows(2) {
        let ((x0, y0), (x1, y1)) = (w[0], w[1]);
        if l <= x1 {
            return y0 + (l - x0) * (y1 - y0) / (x1 - x0);
        }
    }
    T[T.len() - 1].1
}

/// CJ125 interface output UA = 1.5 V + gain·Rshunt·Ip (Rshunt 61.9 Ω). The amplification (8 or 17)
/// is selected by the firmware over SPI on the real chip.
pub fn lsu_cj125_ua_v(lambda: f64, gain: f64) -> f64 {
    let g = if gain.is_finite() { gain } else { 0.0 };
    (1.5 + g * 61.9 * lsu49_pump_current_ma(lambda) / 1000.0).clamp(0.0, 5.0)
}

/// Source of a digital sensor edge produced by the plant.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum EdgeSource {
    /// Active (falling) edge of crank tooth 0..57 (tooth 0 = first after the 2-tooth gap, 78 °CRK
    /// before TDC0). `forward = false` when the crank turns backwards: a CPDD sensor then emits its
    /// long (≈90 µs) direction pulse instead of the short forward pulse.
    CrankTooth { tooth: u8, forward: bool },
    /// Cam Hall sensor edge, cam 0 = intake, 1 = exhaust; `level` = sensor output after the edge.
    Cam { cam: u8, level: bool },
}

/// Time-stamped sensor edge (absolute plant time, s).
#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct SensorEdge {
    pub t_s: f64,
    pub source: EdgeSource,
}

/// Piezo knock-sensor sensitivity in its flat band, mV/g (Bosch KS-class data sheet: 26 ± 8 mV/g).
pub const KNOCK_SENSOR_MV_PER_G: f64 = 26.0;
/// Structure-borne transfer from the first circumferential cylinder-pressure mode to the sensor
/// seat, g per bar, by cylinder (one sensor between cylinders 2 and 3). UNVERIFIED.
pub const KNOCK_TRANSFER_G_PER_BAR: [f64; 4] = [1.5, 2.0, 2.0, 1.5];

/// RMS knock-band output (mV) over a cylinder's window: mechanical background (piston slap,
/// valve seating, injectors; ∝ speed^1.2, UNVERIFIED) and the RMS cylinder-pressure oscillation.
pub fn knock_sensor_band_mv(osc_rms_bar: f64, rpm: f64, cyl: usize) -> f64 {
    let osc = if osc_rms_bar.is_finite() { osc_rms_bar } else { 0.0 };
    let n = if rpm.is_finite() { rpm.abs() } else { 0.0 };
    let bg_g = 0.25 * (n / 1000.0).powf(1.2);
    let knock_g = KNOCK_TRANSFER_G_PER_BAR[cyl.min(KNOCK_TRANSFER_G_PER_BAR.len() - 1)] * osc;
    KNOCK_SENSOR_MV_PER_G * (bg_g * bg_g + knock_g * knock_g).sqrt()
}

/// Oil level / temperature sender G266 — thermal "TOG" sender, temperature time ms → oil °C: `ip_toil_srv_intl[0]`
/// @0x8081FCE4 (axis @0x8081F0C8), SC8S50.
pub const G266_TEMP_TABLE: [(f64, f64); 13] = [
    (23.0, -40.0), (30.0, -17.0), (35.0, 0.0), (40.0, 17.0), (45.0, 33.0), (50.0, 50.0), (55.0, 67.0), (60.0, 83.0),
    (65.0, 100.0), (70.0, 117.0), (75.0, 133.0), (80.0, 150.0), (84.5, 165.0),
];
/// G266 cool-off time ms → oil level mm: `ip_loil_srv_intl[0]` @0x8081F716 (axis @0x8081F014), SC8S50 (a long cool-off
/// time is a low level: the heated element cools faster in oil than in air).
pub const G266_LEVEL_TABLE: [(f64, f64); 13] = [
    (150.0, 120.0), (225.0, 110.0), (300.0, 100.0), (375.0, 90.0), (450.0, 80.0), (525.0, 70.0), (600.0, 60.0),
    (675.0, 50.0), (750.0, 40.0), (825.0, 30.0), (900.0, 20.0), (915.0, 18.0), (1050.0, 0.0),
];
/// Reference pulse period of the G266 frame, ms (the firmware's nominal reference `c_t_sig_ref_dft_loil` 100 ms, window
/// `c_t_sig_h_ref_loil_min/max` 48–52 ms high).
const G266_REF_MS: f64 = 100.0;
/// Heating pulse, ms (the firmware only records it, T_HEAT_LOIL), output-low time after power-up, ms, and fast temperature
/// periods sent before the first level measurement. UNVERIFIED (no sender data sheet; chosen inside the firmware's windows).
const G266_HEAT_MS: f64 = 30.0;
const G266_POWER_ON_MS: f64 = 100.0;
const G266_START_PULSES: u32 = 10;

/// Oil level / temperature sender G266 (supplied from KL15, open-collector output against the ECU pull-up; ECU pin P3.4).
/// Frame as the SIMOS18 firmware decodes it (ENLU 27.3, SC800S50 0x801A36F0; FR p.7193–7208; the BSW captures at each
/// falling edge the high time just ended and the low time before it): the heating pulse (high), then during the cool-off
/// time a 50 % pulse (low, high) — alternately the reference (period 100 ms) and the temperature pulse (period =
/// temperature time) — and the output low until the cool-off time, counted from the end of heating, has passed. After
/// power-up only fast temperature periods (the firmware's start-phase detection: equal high/low times, period
/// 25–88 ms). Characteristics:
/// the inverses of the firmware's own sender curves [`G266_TEMP_TABLE`] / [`G266_LEVEL_TABLE`] (VERIFIED-S as the ECU's
/// model of the sender; the real sender's characteristic is HYP the same). High = output transistor off (pull-up).
#[derive(Debug, Clone, Default)]
pub struct OilLevelSender {
    powered: bool,
    /// Plant time of the next pin transition, s, and the segments (level, ms) still to send.
    t_next: f64,
    queue: std::collections::VecDeque<(bool, f64)>,
    start_left: u32,
    next_is_ref: bool,
}

impl OilLevelSender {
    /// Temperature time of the sender at `oil_c`, ms.
    pub fn temp_ms(oil_c: f64) -> f64 {
        inverse_v(oil_c, &G266_TEMP_TABLE)
    }

    /// Cool-off time of the sender at `level_mm`, ms.
    pub fn cool_ms(level_mm: f64) -> f64 {
        inverse_v(level_mm, &G266_LEVEL_TABLE)
    }

    /// Pin transitions (plant time s, level) up to `t`. The values are sampled when a frame starts.
    pub fn step(&mut self, t: f64, powered: bool, oil_c: f64, level_mm: f64) -> Vec<(f64, bool)> {
        if !t.is_finite() {
            return Vec::new();
        }
        if powered != self.powered {
            self.powered = powered;
            self.queue.clear();
            self.t_next = t;
            self.start_left = G266_START_PULSES;
            self.next_is_ref = true;
            if powered {
                self.queue.push_back((false, G266_POWER_ON_MS));
            } else {
                return vec![(t, true)];
            }
        }
        let mut v = Vec::new();
        while powered && self.t_next <= t {
            if v.len() >= 8192 {
                self.queue.clear();
                self.t_next = t;
                break;
            }
            if self.queue.is_empty() {
                let tt = Self::temp_ms(oil_c);
                if self.start_left > 0 {
                    self.start_left -= 1;
                    self.queue.extend([(true, 0.5 * tt), (false, 0.5 * tt)]);
                } else {
                    let p = if self.next_is_ref { G266_REF_MS } else { tt };
                    self.next_is_ref = !self.next_is_ref;
                    let low = (Self::cool_ms(level_mm) - p).max(1.0);
                    self.queue.extend([(true, G266_HEAT_MS), (false, 0.5 * p), (true, 0.5 * p), (false, low)]);
                }
            }
            let Some((level, ms)) = self.queue.pop_front() else { break };
            v.push((self.t_next, level));
            let dt_s = if ms.is_finite() { ms.max(1.0) * 1e-3 } else { 1e-3 };
            let next = self.t_next + dt_s;
            if next <= self.t_next {
                self.t_next = t + 1e-3;
                break;
            }
            self.t_next = next;
        }
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The frame carries what the firmware's decoder (0x801A36F0) reconstructs: reference period, temperature period
    /// (50 % pulses) and the cool-off time = end of one heating pulse to the start of the next.
    #[test]
    fn g266_frame_codes_temperature_and_level() {
        let mut s = OilLevelSender::default();
        let mut e = s.step(0.0, true, 90.0, 80.0);
        e.extend(s.step(10.0, true, 90.0, 80.0));
        let dur: Vec<f64> = e.windows(2).map(|w| (w[1].0 - w[0].0) * 1e3).collect();
        assert!(!e[0].1 && e.windows(2).all(|w| w[0].1 != w[1].1), "alternating levels from low");
        let tt = OilLevelSender::temp_ms(90.0);
        assert!((interp(tt, &G266_TEMP_TABLE) - 90.0).abs() < 1e-9 && (tt - 62.06).abs() < 0.01);
        // power-on low, 10 fast temperature periods, then reference / temperature frames
        let f = 1 + 2 * G266_START_PULSES as usize;
        assert!(dur[1..f].iter().all(|&d| (d - 0.5 * tt).abs() < 1e-6));
        let (r, t) = (&dur[f..f + 4], &dur[f + 4..f + 8]);
        assert!((r[0] - G266_HEAT_MS).abs() < 1e-6 && (t[0] - G266_HEAT_MS).abs() < 1e-6 && e[f].1, "frame starts with the heating pulse (high)");
        assert!((r[1] + r[2] - 100.0).abs() < 1e-6 && (r[1] - r[2]).abs() < 1e-9 && (t[1] + t[2] - tt).abs() < 1e-6);
        assert!((r[1] + r[2] + r[3] - OilLevelSender::cool_ms(80.0)).abs() < 1e-6 && (OilLevelSender::cool_ms(80.0) - 450.0).abs() < 1e-9);
        assert!((t[1] + t[2] + t[3] - 450.0).abs() < 1e-6);
        assert_eq!(s.step(10.5, false, 90.0, 80.0), vec![(10.5, true)], "unpowered: output released (pull-up)");
    }

    #[test]
    fn lambda_one_is_mid_rail() {
        assert!((lsu_cj125_ua_v(1.0, 17.0) - 1.5).abs() < 1e-12);
        assert!(lsu_cj125_ua_v(0.8, 8.0) < lsu_cj125_ua_v(0.9, 8.0));
        assert!(lsu_cj125_ua_v(1.2, 17.0) > 1.5);
    }

    /// Every characteristic is the inverse of its firmware table: value → V → table gives the value back
    /// across the table range.
    #[test]
    fn sensor_voltages_invert_the_firmware_tables() {
        type TableCase<'a> = (&'a str, &'a [(f64, f64)], fn(f64) -> f64);
        let tables: [TableCase<'_>; 5] = [
            ("IAT", &IAT_TABLE, iat_sensor_v),
            ("CLT", &CLT_TABLE, coolant_sensor_v),
            ("MAP", &MAP_TABLE, map_sensor_v),
            ("baro", &BARO_TABLE, baro_sensor_v),
            ("PUT", &PUT_TABLE, put_sensor_v),
        ];
        for (name, t, f) in tables {
            for w in t.windows(2) {
                let y = 0.5 * (w[0].1 + w[1].1);
                assert!((interp(f(y), t) - y).abs() < 1e-9, "{name} {y}");
            }
        }
        assert!(iat_sensor_v(-20.0) > iat_sensor_v(20.0) && iat_sensor_v(20.0) > iat_sensor_v(90.0));
        // 5 °C intake air ≈ 3.07 V, 90 °C coolant ≈ 0.97 V (the former generic NTC gave −31 °C / 80 °C there).
        assert!((iat_sensor_v(5.0) - 3.073).abs() < 0.01 && (coolant_sensor_v(90.0) - 0.969).abs() < 0.01);
        assert!((interp(lp_fuel_sensor_v(5.0), &LP_FUEL_TABLE) - 6.0).abs() < 1e-9);
        assert!((rail_sensor_v(14.0) - 2.5).abs() < 1e-4 && (rail_sensor_v(3.5) - 1.0).abs() < 1e-4);
    }

    #[test]
    fn redundant_tracks_are_complementary() {
        let (a, b) = throttle_sensors_v(37.0);
        assert!((a + b - 5.0).abs() < 1e-12);
        assert!((throttle_sensors_v(7.0).0 - 0.8333).abs() < 1e-3, "limp-home voltage of the calibration");
    }

    #[test]
    fn sensor_functions_and_g266_survive_nan_empty_and_large_time_jumps() {
        assert_eq!(interp(1.0, &[]), 0.0);
        assert_eq!(interp(f64::NAN, &[(0.0, 3.0), (10.0, 5.0)]), 3.0);
        assert_eq!(inverse_v(1.0, &[]), 0.0);
        assert!(map_sensor_v(f64::NAN).is_finite());
        assert!(coolant_sensor_v(f64::NAN).is_finite());
        assert!(rail_sensor_v(f64::NAN).is_finite());
        assert!(throttle_sensors_v(f64::NAN).0.is_finite());
        assert!(pedal_sensors_v(f64::NAN).0.is_finite());
        assert!(lsu_cj125_ua_v(f64::NAN, f64::NAN).is_finite());
        assert!(knock_sensor_band_mv(f64::NAN, f64::NAN, 99).is_finite());
        assert!(interp2(f64::NAN, f64::NAN, &[0.0, 1.0], &[0.0, 1.0], &[[1.0, 2.0], [3.0, 4.0]]).is_finite());

        let mut g = OilLevelSender::default();
        assert!(g.step(f64::NAN, true, 90.0, 80.0).is_empty());
        let _ = g.step(0.0, true, f64::NAN, f64::NAN);
        // Huge jump and ULP-stalled timestamp must terminate and cap memory cleanly.
        let big = g.step(1e9, true, 90.0, 80.0);
        assert!(big.len() <= 8192);
        let ulp = g.step(1e18, true, 90.0, 80.0);
        assert!(ulp.len() <= 8192);
    }
}
