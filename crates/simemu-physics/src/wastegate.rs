//! Electric wastegate actuator (EA888 Gen3: "Ladedrucksteller" V465 with its position sensor) and the flap.
//!
//! Hardware chain, referred to the actuator output shaft (angle θ from the flap-seated position):
//! * DC motor behind an H-bridge (ATIC111 on the ECU board): mean armature voltage = signed duty × supply;
//!   torque k·(V − k·ω)/R through the gearbox (k = motor constant × ratio × efficiency). Bridge disabled →
//!   open circuit, no torque; duty 0 with the bridge enabled → the bridge shorts the motor (eddy brake −k²ω/R).
//! * Return spring towards its rest end (`rest_open`), preload + rate.
//! * Exhaust gas on the flap: (p₃ − p₄)·A_port acting at the flap arm, through the linkage, pushes the flap open;
//!   the seat carries it while the actuator holds the flap shut.
//! * Coulomb friction (gears, linkage bushings in hot exhaust), end stops (seat, full open).
//! * Flow: the flap lifts off a port of diameter d on an arm r; curtain area π·d·r·sin φ saturates at the port
//!   area π·d²/4 → open fraction min(1, 4·r·sin φ / d) (most of the flow change happens in the first part of the
//!   travel, as on any swing-valve wastegate).
//! * Position sensor: contactless angle sensor on the output shaft, linear output between `sensor_v` at the two
//!   ends. SC800S50 reads it on ADC1.CH5 as POSN_BPA = (V − VP_BPA_AD_BOL)·FAC_BPA_SLOP: 0 % = flap seated (lower
//!   stop, high voltage), 100 % = open; the bench ECU's learned seat voltage is 3.931 V and its slope −33.33 %/V
//!   (`lc_vp_bpa_dif_tol_bol_con`: constant 3.0 V span) — VERIFIED-T in the firmware (docs/SIMOS18_GPTA_IO_RE.md
//!   §12.6). The firmware's own requirements on the hardware (same section): it holds the flap shut with a static
//!   −15…−25 % duty ("noise" mode), learns the seat at −40 % within 0.3 s, and its limp-home (95 % with zero duty)
//!   expects the de-energised actuator to go open.
//!
//! Every parameter is a profile field of the turbocharger (`TurboHardware::wg_actuator`); the values are
//! UNVERIFIED engineering estimates (no supplier data): e-WG actuators of this class reach ~3–4 N·m stall torque
//! at the output and travel end to end in ≈ 0.1–0.15 s at 13.5 V.

use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct WastegateActuatorHw {
    /// Armature resistance, Ω.
    pub r_ohm: f64,
    /// Torque constant at the output shaft (= back-EMF constant), N·m/A.
    pub k_nm_a: f64,
    /// Reflected inertia at the output shaft (rotor × ratio² dominates), kg·m².
    pub inertia_kgm2: f64,
    /// Coulomb friction at the output, N·m.
    pub friction_nm: f64,
    /// Output shaft travel between the seated flap and the open stop, rad.
    pub travel_rad: f64,
    /// Return spring: true = rests at the open stop (fail-safe open), false = at the seat. Preload, N·m, rate, N·m/rad.
    pub rest_open: bool,
    pub spring_preload_nm: f64,
    pub spring_rate_nm_rad: f64,
    /// Flap: port diameter, m, flap arm, m, flap angle at full travel, deg; linkage ratio (flap angle per output angle)
    /// is flap_full_deg / travel.
    pub port_d_m: f64,
    pub flap_arm_m: f64,
    pub flap_full_deg: f64,
    /// Position-sensor output at the seat and at the open stop, V.
    pub sensor_v: [f64; 2],
}

impl WastegateActuatorHw {
    /// Small-frame IHI IS12 / IS20 wastegate with the EA888 Gen3 e-actuator (UNVERIFIED values, see module docs).
    pub fn ea888_gen3_is12() -> Self {
        Self {
            r_ohm: 2.0,
            k_nm_a: 0.6,
            inertia_kgm2: 4.0e-3,
            friction_nm: 0.12,
            // Sensor 2.143 V/rad (UNVERIFIED). Seat = the bench ECU's learned VP_BPA_AD_BOL (3.931 V, learned on the reference
            // CJSA/IS12). Open stop 0.70 V (UNVERIFIED, inside the firmware's bounds): the stock IS12 calibration SC8O40
            // limits the position set point to the sensor voltage `c_vp_sp_bpa_max` 0.870 V (`lc_vp_sp_bpa_max` = 1,
            // 0x800BBEFC) and commands exactly that position when it opens the wastegate fully (cold part load), so the stop
            // must lie beyond it — at the former 0.931 V the position controller wound up to +100 % against the stop and set
            // PWM_BPA_H (P00AF, docs/SIMOS18_GPTA_IO_RE.md §12.6.1); a learned top limit below `c_vp_bpa_tol_sub` −
            // `c_vp_bpa_tol_abs_l` = 0.57 V is implausible to the same calibration; workshop data: open ≈ 0.2–0.8 V. Travel,
            // flap angle and spring preload are the former 1.4 rad / 32° / 0.2 N·m extended by the extra 0.231 V at the
            // same sensor gain, linkage ratio and seat torque (the flow saturates long before either stop).
            travel_rad: 1.5078,
            rest_open: true,
            spring_preload_nm: 0.1838,
            spring_rate_nm_rad: 0.15,
            port_d_m: 0.026,
            flap_arm_m: 0.024,
            flap_full_deg: 34.46,
            sensor_v: [3.931, 0.70],
        }
    }

    /// IS38: larger wastegate port and flap (UNVERIFIED).
    pub fn ea888_gen3_is38() -> Self {
        Self { port_d_m: 0.030, flap_arm_m: 0.027, ..Self::ea888_gen3_is12() }
    }

    fn port_area_m2(&self) -> f64 {
        std::f64::consts::PI / 4.0 * self.port_d_m * self.port_d_m
    }
}

/// Actuator state.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct WastegateActuator {
    /// Output shaft angle from the seat, rad, and its rate, rad/s.
    pub theta_rad: f64,
    pub omega: f64,
    /// Armature current, A (signed, + = closing torque).
    pub current_a: f64,
}

impl WastegateActuator {
    pub fn at_rest(hw: &WastegateActuatorHw) -> Self {
        Self { theta_rad: if hw.rest_open { hw.travel_rad } else { 0.0 }, ..Default::default() }
    }

    /// Position as a fraction of the travel, 0 = flap seated, 1 = open stop.
    pub fn position(&self, hw: &WastegateActuatorHw) -> f64 {
        (self.theta_rad / hw.travel_rad).clamp(0.0, 1.0)
    }

    /// Flow-area fraction of the wastegate port.
    pub fn open_fraction(&self, hw: &WastegateActuatorHw) -> f64 {
        let phi = (self.position(hw) * hw.flap_full_deg).to_radians();
        (4.0 * hw.flap_arm_m * phi.sin() / hw.port_d_m).clamp(0.0, 1.0)
    }

    /// Position-sensor output voltage.
    pub fn sensor_v(&self, hw: &WastegateActuatorHw) -> f64 {
        hw.sensor_v[0] + (hw.sensor_v[1] - hw.sensor_v[0]) * self.position(hw)
    }

    /// Output-shaft angle at which the port passes the flow fraction `f` (inverse of [`Self::open_fraction`]).
    pub fn theta_for_open_fraction(hw: &WastegateActuatorHw, f: f64) -> f64 {
        let s = (f.clamp(0.0, 1.0) * hw.port_d_m / (4.0 * hw.flap_arm_m)).min(1.0);
        s.asin().to_degrees() / hw.flap_full_deg * hw.travel_rad
    }

    /// Static armature voltage (+ = towards the seat) that balances spring and gas load at `theta` (no friction).
    pub fn hold_voltage(hw: &WastegateActuatorHw, theta: f64, dp_pa: f64) -> f64 {
        let to_rest = if hw.rest_open { hw.travel_rad - theta } else { -theta };
        let preload_dir = if hw.rest_open { 1.0 } else { -1.0 };
        let t_spring = preload_dir * hw.spring_preload_nm + hw.spring_rate_nm_rad * to_rest;
        let t_gas = dp_pa.max(0.0) * hw.port_area_m2() * hw.flap_arm_m * hw.flap_full_deg.to_radians() / hw.travel_rad;
        (t_spring + t_gas) * hw.r_ohm / hw.k_nm_a
    }

    /// Test-cell equipment only (plant tests driving the plant without an ECU; never used by the emulator): bridge duty
    /// of a feed-forward + P position servo that holds the output shaft at `th` (≤ 0 → driven shut; the angle for a wanted
    /// flow fraction: `EngineState::bench_wastegate_theta`).
    pub fn bench_servo_duty(&self, hw: &WastegateActuatorHw, th: f64, dp_pa: f64, supply_v: f64) -> f64 {
        if th <= 0.0 {
            return 1.0;
        }
        ((Self::hold_voltage(hw, th, dp_pa) + 30.0 * (self.theta_rad - th)) / supply_v.max(1.0)).clamp(-1.0, 1.0)
    }

    /// Advances by `dt`. `v_motor` = mean armature voltage with + driving towards the seat (`None` = bridge off),
    /// `dp_pa` = turbine inlet − outlet pressure on the flap.
    pub fn step(&mut self, hw: &WastegateActuatorHw, v_motor: Option<f64>, dp_pa: f64, dt: f64) {
        if !(dt > 0.0 && dt.is_finite()) {
            return;
        }
        if dt > 5e-4 {
            let n = (dt / 2e-4).ceil().clamp(1.0, 10_000.0) as usize;
            let h = dt / n as f64;
            for _ in 0..n {
                self.step_one(hw, v_motor, dp_pa, h);
            }
        } else {
            self.step_one(hw, v_motor, dp_pa, dt);
        }
    }

    fn step_one(&mut self, hw: &WastegateActuatorHw, v_motor: Option<f64>, dp_pa: f64, dt: f64) {
        let w = self.omega;
        // Sign convention inside: θ grows towards open; motor + = closing = −θ direction.
        self.current_a = v_motor.map_or(0.0, |v| (v + hw.k_nm_a * w) / hw.r_ohm);
        let t_motor = -hw.k_nm_a * self.current_a;
        let to_rest = if hw.rest_open { hw.travel_rad - self.theta_rad } else { -self.theta_rad };
        let preload_dir = if hw.rest_open { 1.0 } else { -1.0 };
        let t_spring = preload_dir * hw.spring_preload_nm + hw.spring_rate_nm_rad * to_rest;
        let ratio = hw.flap_full_deg.to_radians() / hw.travel_rad;
        let t_gas = dp_pa.max(0.0) * hw.port_area_m2() * hw.flap_arm_m * ratio;
        let t = t_motor + t_spring + t_gas;
        let at_seat = self.theta_rad <= 0.0 && t <= 0.0;
        let at_open = self.theta_rad >= hw.travel_rad && t >= 0.0;
        if at_seat || at_open || (w == 0.0 && t.abs() <= hw.friction_nm) {
            self.omega = 0.0;
            return;
        }
        let f = hw.friction_nm * if w != 0.0 { w.signum() } else { t.signum() };
        let mut w1 = w + (t - f) / hw.inertia_kgm2 * dt;
        if w != 0.0 && w1.signum() != w.signum() {
            w1 = 0.0;
        }
        let th = self.theta_rad + w1 * dt;
        if th <= 0.0 || th >= hw.travel_rad {
            (self.theta_rad, self.omega) = (th.clamp(0.0, hw.travel_rad), 0.0);
        } else {
            (self.theta_rad, self.omega) = (th, w1);
        }
    }
}

/// Wastegate flow over the actuator position as the firmware's boost-pressure actuator set-point model has it — the
/// IS12 wastegate as the stock CJSA calibration describes it (derived from the firmware model, not supplier data).
///
/// SC8O40 turbine control (FR 10.23/10.24; code 0x800C1DDC…0x800C1FE2, VERIFIED-S by disassembly, every step checked
/// against the firmware's RAM on the bench, VERIFIED-T):
/// * flow capacity x = `ip_flow_wg_con_1`(T₃) · `ip_psi_wg`(p₄/p₃) · p₃ / `c_prs_stnd_posn_sp_bpa_rgl` (200 hPa), in the
///   firmware's raw scaling `x = CON[s/m] · ψ · p₃[hPa] · 0.24272` — i.e. the flow of an ideal 2.023 cm² orifice divided
///   by `c_flow_wg_ref` 300 kg/h (CON = √(2κ/((κ−1)·287·T)), ψ the isentropic flow function, κ 1.35);
/// * required flow y = FLOW_WG_SP / 300 kg/h (the firmware also keeps a VVL/TIA-corrected copy; the correction was 1.0 at
///   every bench point — UNVERIFIED elsewhere);
/// * FAC_BPA = `ip_fac_bpa_sp[VVL]`(x, y) (1 = shut), position set point = `ip_posn_sp_bpa_rgl_conv`(FAC) = 100·(1 − FAC) %
///   in the calibration's sensor scale: POSN = (VP_BOL − VP)·`c_fac_bpa_slop_sub` (−26.5 %/V, `lc_fac_bpa_slop_sub` = 1).
///
/// The plant inverts it: the actuator's sensor voltage gives POSN → FAC, the gas state gives x, and the flow is the y at
/// which the map reaches that FAC (the map falls monotonically along y). Past the last y point the last segment is
/// extended to FAC 0; the result never exceeds the plant's own port orifice (`wastegate_cda_m2`, which also covers the
/// table's corner at x → 0, where the map has flow without a pressure difference). The AMP correction
/// `ip_fac_amp_posn_sp_bpa_rgl` (1.0 above 950 hPa) is not applied (UNVERIFIED at altitude).
#[derive(Debug)]
pub struct WastegateFlowMap {
    pub source: &'static str,
    /// Isentropic flow function ψ over p₄/p₃ (`ip_psi_wg`).
    pub psi: [(f64, f64); 8],
    /// √(2κ/((κ−1)·R·T)) over T₃ in K, s/m (`ip_flow_wg_con_1`).
    pub con: [(f64, f64); 8],
    /// `x = CON·ψ·p₃[hPa]·x_scale`.
    pub x_scale: f64,
    /// Flow of y = 1, kg/h (`c_flow_wg_ref`).
    pub y_ref_kg_h: f64,
    /// Position-sensor slope of the calibration, % per volt below the seat voltage (`c_fac_bpa_slop_sub`).
    pub slope_pct_per_v: f64,
    pub x_axis: [f64; 16],
    pub y_axis: [f64; 10],
    /// FAC_BPA, `fac[vvl][ix][iy]` (vvl 0 = standard exhaust lift, 1 = LFT_1).
    pub fac: [[[f64; 10]; 16]; 2],
}

impl WastegateFlowMap {
    /// Flow capacity x of the gas state (p in Pa, T in K).
    pub fn x(&self, p3_pa: f64, p4_pa: f64, t3_k: f64) -> f64 {
        let pr = p4_pa / p3_pa;
        // past the last table point ψ falls to 0 at p₄ = p₃
        let psi = if pr > 0.98999 { 0.050629 * ((1.0 - pr) / (1.0 - 0.98999)).clamp(0.0, 1.0) } else { crate::sensors::interp(pr, &self.psi) };
        crate::sensors::interp(t3_k, &self.con) * psi * p3_pa / 100.0 * self.x_scale
    }

    /// FAC_BPA column at x along the y axis (the firmware's lookup, clamped at the x axis ends), made non-increasing.
    fn column(&self, x: f64, vvl: usize) -> [f64; 10] {
        let i = self.x_axis.partition_point(|&a| a < x).clamp(1, 15) - 1;
        let f = ((x - self.x_axis[i]) / (self.x_axis[i + 1] - self.x_axis[i])).clamp(0.0, 1.0);
        let (a, b) = (&self.fac[vvl][i], &self.fac[vvl][i + 1]);
        let mut c = [0.0; 10];
        for j in 0..10 {
            c[j] = (a[j] + (b[j] - a[j]) * f).min(if j > 0 { c[j - 1] } else { 1.0 });
        }
        c
    }

    /// The firmware's FAC_BPA set point for the normalised flow `y` at capacity `x`.
    pub fn fac_at(&self, x: f64, y: f64, lft1: bool) -> f64 {
        let c = self.column(x, lft1 as usize);
        crate::sensors::interp(y, &std::array::from_fn::<_, 10, _>(|j| (self.y_axis[j], c[j])))
    }

    /// Normalised flow y at which the map reaches `fac` at capacity `x` (the inverse along y).
    pub fn y_at(&self, x: f64, fac: f64, lft1: bool) -> f64 {
        let (c, ys) = (self.column(x, lft1 as usize), &self.y_axis);
        if fac >= c[0] {
            return 0.0;
        }
        for j in 1..10 {
            if fac >= c[j] && c[j - 1] > c[j] {
                return ys[j - 1] + (c[j - 1] - fac) / (c[j - 1] - c[j]) * (ys[j] - ys[j - 1]);
            }
        }
        // wider open than the table: the last segment extended to FAC 0
        let slope = (ys[9] - ys[8]) / (c[8] - c[9]).max(1e-6);
        ys[9] + (c[9] - fac.max(0.0)) * slope
    }

    /// Wastegate mass flow, kg/s, at the actuator position-sensor voltage `v` (seat at `v_seat`).
    pub fn mass_flow(&self, v: f64, v_seat: f64, p3_pa: f64, p4_pa: f64, t3_k: f64, lft1: bool) -> f64 {
        if p3_pa <= p4_pa {
            return 0.0;
        }
        let fac = 1.0 - (v_seat - v) * self.slope_pct_per_v / 100.0;
        self.y_at(self.x(p3_pa, p4_pa, t3_k), fac, lft1) * self.y_ref_kg_h / 3600.0
    }
}

/// IHI IS12 wastegate as SC8O40 (`images/8V0906264H__0005.bin`) models it: `ip_fac_bpa_sp[0/1]` @0x80819880 / 0x808199C0
/// (x axis @0x80819572, y axis @0x808195C0), `ip_psi_wg` @0x80803564, `ip_flow_wg_con_1` @0x808034FE, `c_flow_wg_ref`
/// 300 kg/h @0x8080374E, `c_prs_stnd_posn_sp_bpa_rgl` 200 hPa @0x808037D4, `c_fac_bpa_slop_sub` −26.5 %/V @0x80802C28
/// (VERIFIED-S, values as stored).
pub const IS12_SC8O40_WASTEGATE: WastegateFlowMap = WastegateFlowMap {
    source: "SC8O40 ip_fac_bpa_sp (firmware model of the IS12 wastegate)",
    psi: [(0.53003, 0.243408), (0.62988, 0.238617), (0.72998, 0.221710), (0.80005, 0.200958), (0.87012, 0.169861), (0.93994, 0.120483), (0.96997, 0.086700), (0.98999, 0.050629)],
    con: [(373.156, 0.008487), (523.156, 0.007168), (673.156, 0.006319), (823.156, 0.005714), (973.156, 0.005256), (1073.156, 0.005005), (1173.156, 0.004787), (1323.0, 0.004499)],
    x_scale: 0.24272,
    y_ref_kg_h: 300.0,
    slope_pct_per_v: 26.49994,
    x_axis: [0.0, 0.10004, 0.15002, 0.20001, 0.22504, 0.25012, 0.30017, 0.40015, 0.50012, 0.60022, 0.70001, 0.80005, 0.90002, 1.10004, 1.35004, 1.60004],
    y_axis: [0.0, 0.02502, 0.05005, 0.20001, 0.35004, 0.65002, 0.80005, 0.95001, 1.25, 1.54999],
    fac: [
        [
            [1.0, 0.5, 0.29999, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.83398, 0.5, 0.09998, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.91998, 0.79999, 0.28198, 0.03302, 0.01599, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.94202, 0.86603, 0.4494, 0.065, 0.03198, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.95203, 0.89203, 0.56696, 0.16602, 0.03998, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.96503, 0.91498, 0.72919, 0.35376, 0.08002, 0.02002, 0.0, 0.0, 0.0],
            [1.0, 0.97504, 0.961, 0.87677, 0.55988, 0.17328, 0.11499, 0.07001, 0.02802, 0.03003],
            [1.0, 0.98199, 0.96399, 0.91168, 0.76349, 0.44458, 0.29498, 0.20892, 0.08899, 0.06],
            [1.0, 0.97998, 0.96503, 0.93036, 0.86462, 0.66229, 0.53833, 0.43756, 0.25098, 0.16101],
            [1.0, 0.98999, 0.96503, 0.93402, 0.89246, 0.76471, 0.67194, 0.58917, 0.39783, 0.25897],
            [1.0, 0.98999, 0.96399, 0.93597, 0.9007, 0.81757, 0.75024, 0.672, 0.48999, 0.349],
            [1.0, 0.98999, 0.966, 0.93701, 0.90399, 0.81805, 0.76282, 0.68695, 0.56, 0.42902],
            [1.0, 0.98999, 0.96802, 0.93799, 0.914, 0.81848, 0.76538, 0.70502, 0.57806, 0.448],
            [1.0, 0.98999, 0.97101, 0.94, 0.91498, 0.8194, 0.77051, 0.71112, 0.59961, 0.48859],
            [1.0, 0.98999, 0.97601, 0.94299, 0.91498, 0.8197, 0.77838, 0.7312, 0.63281, 0.53931],
            [1.0, 0.98999, 0.97998, 0.94501, 0.91498, 0.82001, 0.78564, 0.75134, 0.66602, 0.59003],
        ],
        [
            [1.0, 0.70001, 0.59998, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.84998, 0.76007, 0.06567, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.90503, 0.82336, 0.19275, 0.0553, 0.02289, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.94495, 0.88666, 0.47925, 0.17859, 0.04999, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.95996, 0.93549, 0.63062, 0.37347, 0.04999, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.96497, 0.95227, 0.72815, 0.49719, 0.15503, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.98566, 0.96716, 0.85132, 0.6524, 0.3161, 0.11578, 0.0, 0.0, 0.0],
            [1.0, 0.98438, 0.97754, 0.8996, 0.78394, 0.55939, 0.41949, 0.27972, 0.0, 0.0],
            [1.0, 0.98541, 0.97668, 0.92487, 0.85632, 0.69604, 0.61188, 0.52057, 0.35815, 0.13574],
            [1.0, 0.98401, 0.97491, 0.93292, 0.89191, 0.73999, 0.67798, 0.60663, 0.46387, 0.30396],
            [1.0, 0.98578, 0.97314, 0.93475, 0.8927, 0.75409, 0.67981, 0.6106, 0.47455, 0.3512],
            [1.0, 0.98596, 0.97339, 0.93665, 0.89355, 0.76141, 0.69147, 0.62158, 0.47455, 0.3512],
            [1.0, 0.98615, 0.97357, 0.94098, 0.89435, 0.76141, 0.69147, 0.62158, 0.47455, 0.3512],
            [1.0, 0.98657, 0.974, 0.94159, 0.89594, 0.76141, 0.69147, 0.62158, 0.47455, 0.3512],
            [1.0, 0.987, 0.97449, 0.94238, 0.89801, 0.76141, 0.69147, 0.62158, 0.47455, 0.3512],
            [1.0, 0.98749, 0.97504, 0.94318, 0.90002, 0.76141, 0.69147, 0.62158, 0.47455, 0.3512],
        ],
    ],
};

#[cfg(test)]
mod tests {
    use super::*;

    fn run(a: &mut WastegateActuator, hw: &WastegateActuatorHw, v: Option<f64>, dp: f64, s: f64) {
        for _ in 0..(s / 2e-4) as usize {
            a.step(hw, v, dp, 2e-4);
        }
    }

    /// Full voltage seats the flap in ≈ 0.1–0.2 s against 1.2 bar of exhaust pressure difference, the spring
    /// returns it with the bridge off, a constant partial duty holds an intermediate position, and the flow
    /// fraction saturates long before the open stop.
    #[test]
    fn actuator_closes_holds_and_springs_back() {
        let hw = WastegateActuatorHw::ea888_gen3_is38();
        let mut a = WastegateActuator::at_rest(&hw);
        assert_eq!(a.position(&hw), 1.0);
        assert_eq!(a.open_fraction(&hw), 1.0);
        let mut t = 0.0;
        while a.position(&hw) > 0.0 && t < 1.0 {
            a.step(&hw, Some(13.5), 1.2e5, 2e-4);
            t += 2e-4;
        }
        assert!((0.05..0.25).contains(&t), "seat reached after {t} s");
        // the firmware's static "noise" hold (−14.8 % at 13.5 V) keeps the flap seated at idle exhaust pressure
        run(&mut a, &hw, Some(0.148 * 13.5), 0.05e5, 0.3);
        assert_eq!(a.position(&hw), 0.0);
        // seat learning: −40 % from the open stop reaches the seat within the firmware's 0.3 s acquisition delay
        let mut b = WastegateActuator::at_rest(&hw);
        run(&mut b, &hw, Some(0.4 * 13.5), 0.0, 0.3);
        assert_eq!(b.position(&hw), 0.0);
        run(&mut a, &hw, Some(13.5), 1.2e5, 0.1);
        assert_eq!(a.position(&hw), 0.0, "held shut against the gas load");
        assert!((a.sensor_v(&hw) - hw.sensor_v[0]).abs() < 1e-12);
        run(&mut a, &hw, None, 1.2e5, 0.5);
        assert_eq!(a.position(&hw), 1.0, "spring + gas open it with the bridge off");
        run(&mut a, &hw, Some(0.12 * 13.5), 0.1e5, 0.5);
        let x = a.position(&hw);
        assert!(x > 0.05 && x < 0.95, "partial duty holds an intermediate position: {x}");
        let mid = WastegateActuator { theta_rad: 0.6 * hw.travel_rad, ..Default::default() };
        assert_eq!(mid.open_fraction(&hw), 1.0, "flow saturates well before the open stop");
        let near = WastegateActuator { theta_rad: 0.05 * hw.travel_rad, ..Default::default() };
        assert!(near.open_fraction(&hw) > 0.1 && near.open_fraction(&hw) < 0.5);
    }

    /// The forward lookup reproduces the firmware's own set point read from its RAM (bench, kickdown 4300 rpm: x 9895,
    /// y 32164 raw → FAC_BPA 37496 / 65536), the inverse returns the flow it started from, and the flow rises with the
    /// opening and with the pressure before the turbine.
    #[test]
    fn wastegate_map_matches_the_firmware_and_inverts() {
        let m = &IS12_SC8O40_WASTEGATE;
        // the firmware's x for its own state: TEG 1223.4 K, p₄/p₃ = 2228/4096, p₃ 2187 hPa
        let x = m.x(2187e2, 2187e2 * 2228.0 / 4096.0, 1223.4);
        assert!((x - 9895.0 / 16384.0).abs() < 0.004, "x {x}");
        let fac = m.fac_at(x, 32164.0 / 32768.0, false);
        assert!((fac - 37496.0 / 65536.0).abs() < 0.004, "FAC {fac}");
        assert!((m.y_at(x, fac, false) - 32164.0 / 32768.0).abs() < 1e-6);
        let (seat, p4, t) = (3.93, 1.1e5, 1200.0);
        let mut last = 0.0;
        for k in 0..=30 {
            let v = seat - 0.1 * k as f64;
            let w = m.mass_flow(v, seat, 2.2e5, p4, t, false);
            assert!(w >= last - 1e-12, "{v} V: {w} < {last}");
            last = w;
        }
        assert_eq!(m.mass_flow(seat, seat, 2.2e5, p4, t, false), 0.0, "seated");
        assert!(m.mass_flow(seat - 1.0, seat, 2.4e5, p4, t, false) > m.mass_flow(seat - 1.0, seat, 1.8e5, p4, t, false));
    }

    #[test]
    fn negative_zero_theta_and_large_dt_are_sign_consistent_and_stable() {
        let mut hw = WastegateActuatorHw::ea888_gen3_is12();
        hw.rest_open = false;
        let v_pos = WastegateActuator::hold_voltage(&hw, 0.0, 0.0);
        let v_neg = WastegateActuator::hold_voltage(&hw, -0.0, 0.0);
        assert_eq!(v_pos, v_neg);
        assert!(v_pos < 0.0, "normally-closed spring preload holds the seat without positive closing voltage");

        let mut a_neg = WastegateActuator { theta_rad: -0.0, ..Default::default() };
        a_neg.step(&hw, None, 0.0, 2e-4);
        assert_eq!(a_neg.theta_rad, 0.0);
        assert_eq!(a_neg.omega, 0.0);

        // Large dt (0.25 s) must sub-step stably to the open stop on a normally-open actuator.
        let hw_open = WastegateActuatorHw::ea888_gen3_is12();
        let mut b = WastegateActuator::default();
        b.step(&hw_open, None, 0.5e5, 0.25);
        assert_eq!(b.position(&hw_open), 1.0);
        assert!(b.omega.is_finite());
    }
}
