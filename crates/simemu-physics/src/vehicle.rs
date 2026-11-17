//! Tyres, brakes, body (pitch / load transfer / aero / grade), chassis-dyno rollers and the Haldex
//! AWD coupling hardware.
//!
//! Sources:
//! - Tyre: Magic Formula F = D·sin(C·atan(Bκ)) with load-dependent μ (pDx2-type term) and slip
//!   stiffness ∝ Fz; transient response by the single-contact-point carcass deflection model
//!   du/dt + |Vx|·u/σ = Vsx, κ' = u/σ (H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., 2012,
//!   §4.3.2, §7.2); carcass damping faded out above a few m/s for standstill stability (ibid. ch. 7).
//! - Rolling resistance: F = Fz·(c0 + c2·(V/100)²)·(p/p_ref)^−0.4 (pressure exponent as in
//!   SAE J2452), decreasing with tread temperature (≈ −0.5 %/K, bounded). Coefficients UNVERIFIED.
//! - Brakes: T = k·p_line·μ(T_disc)/μ_nom; pad μ rises slightly when warm and fades above
//!   ≈ 450 °C (A. Day, *Braking of Road Vehicles*, Butterworth-Heinemann 2014, ch. 3, 6);
//!   disc cooling = convection (∝ v^0.8) + radiation εσA(T⁴ − T∞⁴).
//! - Haldex Gen 5: electric pump with centrifugal pressure regulator, no control valve /
//!   accumulator (Haldex press release 2010 "Haldex wins Major Hydraulics and Pneumatics Award").

use crate::clutch::WetClutchHw;
use serde::{Deserialize, Serialize};

pub const G: f64 = 9.81;
const SIGMA_SB: f64 = 5.67e-8;

#[derive(Debug, Clone, Serialize)]
pub struct TyreHw {
    pub radius_m: f64,
    /// Wheel + tyre + disc + hub, per wheel.
    pub inertia_kgm2: f64,
    pub mu_peak: f64,
    pub load_sensitivity: f64,
    pub fz0_n: f64,
    pub c_shape: f64,
    /// C_Fκ / Fz.
    pub slip_stiffness_per_fz: f64,
    pub relaxation_m: f64,
    pub t_opt_c: f64,
    pub rr_c0: f64,
    pub rr_c2: f64,
    pub pressure_bar: f64,
    pub pressure_ref_bar: f64,
    pub c_tread_j_k: f64,
}

impl TyreHw {
    /// 225/45 R17-class summer tyre (Octavia A7). UNVERIFIED.
    pub fn octavia() -> Self {
        Self {
            radius_m: 0.3115,
            inertia_kgm2: 1.15,
            mu_peak: 1.05,
            load_sensitivity: -0.08,
            fz0_n: 4000.0,
            c_shape: 1.5,
            slip_stiffness_per_fz: 19.0,
            relaxation_m: 0.20,
            t_opt_c: 85.0,
            rr_c0: 0.0108,
            rr_c2: 0.0015,
            pressure_bar: 2.4,
            pressure_ref_bar: 2.4,
            c_tread_j_k: 3600.0,
        }
    }
    fn kappa_peak(&self) -> f64 {
        let b = self.slip_stiffness_per_fz / (self.c_shape * self.mu_peak);
        (std::f64::consts::PI / (2.0 * self.c_shape)).tan() / b
    }
}

/// Two tyres of one axle (straight-line, left = right).
#[derive(Debug, Clone, Serialize)]
pub struct AxleTyres {
    /// Carcass longitudinal deflection, m.
    pub u_m: f64,
    pub force_n: f64,
    pub tread_c: f64,
    /// Slip power dissipated in the contact patches, W.
    pub slip_power_w: f64,
}

impl AxleTyres {
    pub fn new(t: f64) -> Self {
        Self { u_m: 0.0, force_n: 0.0, tread_c: t, slip_power_w: 0.0 }
    }

    /// Updates the carcass deflection and returns the axle's longitudinal force (both tyres), N.
    /// `v_ground`: road (or roller) surface speed under the tyre relative to the hub, m/s.
    pub fn step(&mut self, hw: &TyreHw, fz_axle: f64, v_ground: f64, omega: f64, dt: f64) -> f64 {
        if !(dt > 0.0 && dt.is_finite()) {
            return self.force_n;
        }
        let vx = v_ground.abs();
        let v_s = omega * hw.radius_m - v_ground;
        let u_max = hw.relaxation_m * hw.kappa_peak();
        let u0 = self.u_m;
        self.u_m = ((self.u_m + v_s * dt) / (1.0 + vx * dt / hw.relaxation_m)).clamp(-u_max, u_max);
        let fz = (0.5 * fz_axle).max(0.0);
        if fz <= 0.0 {
            self.force_n = 0.0;
            return 0.0;
        }
        let dfz = (fz - hw.fz0_n) / hw.fz0_n;
        let ft = (1.0 - 4e-5 * (self.tread_c - hw.t_opt_c).powi(2)).max(0.7);
        let d = hw.mu_peak * (1.0 + hw.load_sensitivity * dfz) * ft * fz;
        let b = hw.slip_stiffness_per_fz * fz / (hw.c_shape * d);
        let mut kappa = self.u_m / hw.relaxation_m;
        if self.u_m.abs() >= u_max {
            // Sliding: force follows the actual slip ratio beyond the peak.
            let real = v_s / vx.max(0.5);
            if real.abs() > kappa.abs() && real * kappa > 0.0 {
                kappa = real;
            }
        }
        let fx = d * (hw.c_shape * (b * kappa).atan()).sin();
        // Low-speed carcass damping (ζ ≈ 0.5 of the wheel–tread mode), faded out above 3 m/s.
        let k_x = hw.slip_stiffness_per_fz * fz / hw.relaxation_m;
        let c_d = 2.0 * 0.5 * (k_x * hw.inertia_kgm2).sqrt() / hw.radius_m;
        let damp = c_d * (self.u_m - u0) / dt * (1.0 - vx / 3.0).max(0.0);
        self.force_n = 2.0 * (fx + damp);
        self.slip_power_w = (self.force_n * v_s).abs();
        self.force_n
    }

    pub fn rolling_resistance(&self, hw: &TyreHw, fz_axle: f64, v: f64) -> f64 {
        let vr = v.abs() * 3.6 / 100.0;
        let c = (hw.rr_c0 + hw.rr_c2 * vr * vr) * (hw.pressure_bar / hw.pressure_ref_bar).powf(-0.4) * (1.0 - 0.005 * (self.tread_c - 25.0)).clamp(0.8, 1.15);
        fz_axle.max(0.0) * c * (v / 0.2).tanh()
    }

    pub fn thermal(&mut self, hw: &TyreHw, f_rr: f64, v: f64, ambient_c: f64, dt: f64) {
        let heat = 0.8 * self.slip_power_w + 0.5 * (f_rr * v).abs();
        let g = 2.0 * (4.0 + 2.5 * v.abs().sqrt());
        self.tread_c += (heat - g * (self.tread_c - ambient_c)) / (2.0 * hw.c_tread_j_k) * dt;
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct BrakeHw {
    /// Axle brake torque per bar at μ_nominal [front, rear], Nm/bar.
    pub nm_per_bar: [f64; 2],
    pub mu_nominal: f64,
    /// Disc pair heat capacity [front, rear], J/K, and radiating area per disc, m².
    pub disc_c_j_k: [f64; 2],
    pub disc_area_m2: [f64; 2],
}

impl BrakeHw {
    /// ≈ 1 g at 100 bar, 70 / 30 split (previous calibration), 312 mm vented front discs.
    pub fn octavia() -> Self {
        Self { nm_per_bar: [29.8, 12.8], mu_nominal: 0.40, disc_c_j_k: [6900.0, 4100.0], disc_area_m2: [0.12, 0.08] }
    }
}

pub fn pad_mu(t_c: f64) -> f64 {
    let rise = 0.37 + 0.04 * (t_c / 200.0).clamp(0.0, 1.0);
    if t_c <= 450.0 {
        rise
    } else {
        (rise - 0.17 * (t_c - 450.0) / 250.0).max(0.22)
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Brakes {
    pub disc_c: [f64; 2],
    pub line_bar: f64,
    /// Torque actually absorbed in the last step per axle, Nm.
    pub torque_nm: [f64; 2],
}

impl Brakes {
    pub fn new(t: f64) -> Self {
        Self { disc_c: [t; 2], line_bar: 0.0, torque_nm: [0.0; 2] }
    }
    /// Maximum friction torque available on an axle, Nm.
    pub fn capacity(&self, hw: &BrakeHw, axle: usize) -> f64 {
        hw.nm_per_bar[axle] * self.line_bar * pad_mu(self.disc_c[axle]) / hw.mu_nominal
    }
    pub fn thermal(&mut self, hw: &BrakeHw, omega: [f64; 2], v: f64, ambient_c: f64, dt: f64) {
        for (a, &w_a) in omega.iter().enumerate() {
            let p = (self.torque_nm[a] * w_a).abs();
            let t = self.disc_c[a];
            let conv = 2.0 * (4.0 + 1.3 * v.abs().powf(0.8)) * (t - ambient_c);
            let rad = 2.0 * 0.55 * SIGMA_SB * hw.disc_area_m2[a] * ((t + 273.15).powi(4) - (ambient_c + 273.15).powi(4));
            self.disc_c[a] += (p - conv - rad) / hw.disc_c_j_k[a] * dt;
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct BodyHw {
    pub mass_kg: f64,
    pub drag_coefficient: f64,
    pub lift_coefficient: [f64; 2],
    pub frontal_area_m2: f64,
    pub wheelbase_m: f64,
    pub cg_height_m: f64,
    pub front_axle_share: f64,
    pub pitch_inertia_kgm2: f64,
    pub pitch_freq_hz: f64,
    pub pitch_damping_ratio: f64,
}

/// Sprung-mass pitch: I·θ̈ = m·a·h − Kθ − Cθ̇; suspension moment (Kθ + Cθ̇) is what transfers
/// load between the axles (steady state m·a·h/L).
#[derive(Debug, Clone, Serialize)]
pub struct Body {
    pub pitch_rad: f64,
    pub pitch_rate: f64,
    pub axle_load_n: [f64; 2],
}

impl Body {
    pub fn new(hw: &BodyHw) -> Self {
        let w = hw.mass_kg * G;
        Self { pitch_rad: 0.0, pitch_rate: 0.0, axle_load_n: [w * hw.front_axle_share, w * (1.0 - hw.front_axle_share)] }
    }

    /// Axle loads for grade `grade_rad`, longitudinal acceleration `a` and speed `v` (aero lift).
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, hw: &BodyHw, a: f64, v: f64, rho: f64, grade_rad: f64, fixed: bool, dt: f64) {
        let k = hw.pitch_inertia_kgm2 * (2.0 * std::f64::consts::PI * hw.pitch_freq_hz).powi(2);
        let c = 2.0 * hw.pitch_damping_ratio * (k * hw.pitch_inertia_kgm2).sqrt();
        let m_susp = if fixed { 0.0 } else { k * self.pitch_rad + c * self.pitch_rate };
        if !fixed {
            self.pitch_rate += (hw.mass_kg * a * hw.cg_height_m - m_susp) / hw.pitch_inertia_kgm2 * dt;
            self.pitch_rad += self.pitch_rate * dt;
        }
        let w = hw.mass_kg * G;
        let q = 0.5 * rho * v * v * hw.frontal_area_m2;
        let static_f = w * grade_rad.cos() * hw.front_axle_share - w * grade_rad.sin() * hw.cg_height_m / hw.wheelbase_m;
        let ff = static_f - m_susp / hw.wheelbase_m - q * hw.lift_coefficient[0];
        let fr = w * grade_rad.cos() - static_f + m_susp / hw.wheelbase_m - q * hw.lift_coefficient[1];
        self.axle_load_n = [ff.max(0.0), fr.max(0.0)];
    }
}

/// Bench equipment: roller dyno under the driven axle(s). Its absorber is the bench's own speed
/// controller (allowed: it is test equipment, not vehicle logic).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ChassisDynoHw {
    /// Equivalent translational mass of the rollers, kg.
    pub roller_mass_kg: f64,
    pub max_absorb_n: f64,
    /// Road-load simulation of a 4-quadrant dyno with axle-coupled rollers under both axles (all four
    /// wheels turn): the roller drive makes the roller surface move as the car would on a level road —
    /// test mass = vehicle mass, aerodynamic drag ½ρ·Cd·A·v² of the car, rolling resistance from the tyre
    /// model (as in coast-down-based dyno settings, e.g. UN ECE R83 Annex 4 / SAE J2263, with the inertia
    /// simulated electrically). The cell's road-speed fan blows at the roller speed.
    #[serde(default)]
    pub road_load: bool,
}

impl Default for ChassisDynoHw {
    fn default() -> Self {
        Self { roller_mass_kg: 1200.0, max_absorb_n: 12_000.0, road_load: false }
    }
}

impl ChassisDynoHw {
    /// 4WD road-load dyno (axle-coupled rollers, electric inertia simulation). Roller mass and drive
    /// rating UNVERIFIED (typical 48" single-roll 4WD emission dyno).
    pub fn road_load() -> Self {
        Self { roller_mass_kg: 1800.0, max_absorb_n: 15_000.0, road_load: true }
    }
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Rollers {
    pub speed_ms: f64,
    pub absorber_n: f64,
    integral: f64,
}

impl Rollers {
    /// Integrates the roller surface speed under tyre force `fx`. `target_kmh <= 0` = inertia run.
    pub fn step(&mut self, hw: &ChassisDynoHw, fx: f64, target_kmh: f64, dt: f64) {
        if target_kmh > 0.0 {
            let err = self.speed_ms - target_kmh / 3.6;
            self.integral = (self.integral + err * dt).clamp(0.0, hw.max_absorb_n / 6000.0);
            self.absorber_n = (4000.0 * err + 6000.0 * self.integral).clamp(0.0, hw.max_absorb_n);
        } else {
            self.absorber_n = 0.0;
        }
        let loss = 60.0 * (self.speed_ms / 0.5).tanh() + 0.4 * self.speed_ms * self.speed_ms;
        self.speed_ms = (self.speed_ms + (fx - self.absorber_n * (self.speed_ms / 0.05).tanh() - loss) / hw.roller_mass_kg * dt).max(0.0);
    }

    /// Road-load mode ([`ChassisDynoHw::road_load`]): the roller surface moves like the car on a level road under
    /// the net tyre force `fx` (rolling resistance already subtracted), the car's aerodynamic drag `f_aero` and
    /// test mass `mass_kg`; `absorber_n` is the force the roller drive takes (negative = motoring), saturated at
    /// its rating. Forward only (as `step`).
    pub fn step_road_load(&mut self, hw: &ChassisDynoHw, fx: f64, f_aero: f64, mass_kg: f64, dt: f64) {
        let a = (fx - f_aero) / mass_kg;
        // drive force = what the mechanical roller mass needs on top of the tyre force to follow the simulated car
        self.absorber_n = (fx - hw.roller_mass_kg * a).clamp(-hw.max_absorb_n, hw.max_absorb_n);
        self.speed_ms = (self.speed_ms + (fx - self.absorber_n) / hw.roller_mass_kg * dt).max(0.0);
    }
}

/// Haldex Gen 5 coupling: pump motor driven by the AWD controller (another ECU), centrifugal
/// pressure regulator, piston, wet multi-plate pack between the prop shaft and the rear diff.
#[derive(Debug, Clone, Serialize)]
pub struct HaldexHw {
    pub pump_max_rad_s: f64,
    pub pump_tau_s: f64,
    /// Regulated pressure = k·ω² (centrifugal regulator), capped.
    pub k_pressure: f64,
    pub p_max_pa: f64,
    pub fill_tau_s: f64,
    pub clutch: WetClutchHw,
    /// Prop-shaft speed / rear wheel speed (angle drive = rear final drive).
    pub ratio: f64,
}

impl HaldexHw {
    pub fn gen5() -> Self {
        Self { pump_max_rad_s: 420.0, pump_tau_s: 0.08, k_pressure: 30e5 / (400.0 * 400.0), p_max_pa: 30e5, fill_tau_s: 0.06, clutch: WetClutchHw::haldex(), ratio: 2.6 }
    }
}

/// Command from the AWD controller (J492). `None` = pump not driven = coupling open.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AwdOutputs {
    pub pump_motor_duty: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Haldex {
    pub pump_omega: f64,
    pub pressure_pa: f64,
    pub force_n: f64,
    pub torque_nm: f64,
    pub temp_c: f64,
    pub health: f64,
}

impl Haldex {
    pub fn new(t: f64) -> Self {
        Self { pump_omega: 0.0, pressure_pa: 0.0, force_n: 0.0, torque_nm: 0.0, temp_c: t, health: 1.0 }
    }

    /// Coupling capacity after this step's hydraulics, Nm (prop-shaft side).
    pub fn hydraulics(&mut self, hw: &HaldexHw, cmd: &AwdOutputs, v_batt: f64, slip_rad_s: f64, dt: f64) -> f64 {
        let target = cmd.pump_motor_duty.unwrap_or(0.0).clamp(0.0, 1.0) * hw.pump_max_rad_s * (v_batt / 13.5).min(1.1);
        self.pump_omega += (target - self.pump_omega) * (dt / hw.pump_tau_s).min(1.0);
        self.pressure_pa = (hw.k_pressure * self.pump_omega * self.pump_omega).min(hw.p_max_pa);
        let f_target = (self.pressure_pa * hw.clutch.piston_area() - hw.clutch.return_spring_n).max(0.0);
        self.force_n += (f_target - self.force_n) * (dt / hw.fill_tau_s).min(1.0);
        let v_slip = slip_rad_s.abs() * hw.clutch.r_eff();
        let mu = hw.clutch.mu(v_slip, self.temp_c, self.force_n / hw.clutch.face_area(), 0.0, 0.0, self.health);
        hw.clutch.surfaces * mu * self.force_n * hw.clutch.r_eff()
    }

    pub fn thermal(&mut self, slip_power_w: f64, ambient_c: f64, dt: f64) {
        self.temp_c += (slip_power_w - 6.0 * (self.temp_c - ambient_c)) / 1200.0 * dt;
        if self.temp_c > 300.0 {
            self.health = (self.health - 0.01 * ((self.temp_c - 300.0) / 40.0).powi(2) * dt).max(0.0);
        }
    }
}
