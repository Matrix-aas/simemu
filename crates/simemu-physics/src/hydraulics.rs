//! Shared hydraulic hardware primitives: fluids, current-driven solenoid valves, orifices,
//! quasi-static actuator pistons, gas accumulator, DC-motor and engine-driven pumps.
//!
//! Sources:
//! - Orifice equation Q = Cd·A·√(2Δp/ρ), Cd ≈ 0.61–0.62 for sharp-edged spool ports:
//!   H. E. Merritt, *Hydraulic Control Systems*, Wiley 1967, ch. 3; N. D. Manring,
//!   *Hydraulic Control Systems*, Wiley 2005, ch. 2–4 (spool valves, pumps, volumetric losses).
//! - Viscosity–temperature: Vogel equation ν = exp(a + b/(T + c)) (H. Vogel, Phys. Z. 22, 1921;
//!   same family as ASTM D341 / Walther). Constants fitted to typical DCT fluid data sheets
//!   (ν40 ≈ 30 mm²/s, ν100 ≈ 7 mm²/s) and 75W gear oil (ν40 ≈ 50, ν100 ≈ 9). UNVERIFIED per fluid.
//! - Gas-charged accumulator: polytropic law p·Vᵍⁿ = const, n ≈ 1.3–1.4 (Merritt §5; Manring §10).
//! - Coil RL dynamics L·di/dt = u − R(T)·i, copper α = 0.00393 1/K.
//!
//! Chambers are modelled as incompressible (massless piston, quasi-static force balance) so the
//! stiff oil bulk modulus never enters the explicit 0.2 ms integration. ponytail: compressibility
//! (pressure ringing, ~1 ms) is not resolved; add a compliant node if valve-pressure ripple matters.

use serde::Serialize;

pub const CD: f64 = 0.62;
pub const P_ATM: f64 = 101_325.0;
/// Spool overlap leakage area as a fraction of the full port area.
pub(crate) const LEAK_FRACTION: f64 = 2e-3;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Fluid {
    pub rho: f64,
    pub cp: f64,
    pub vogel_a: f64,
    pub vogel_b: f64,
    pub vogel_c: f64,
}

impl Fluid {
    /// Low-viscosity DCT/ATF fluid (also the DQ200 mechatronic fluid).
    pub const ATF: Fluid = Fluid { rho: 850.0, cp: 2000.0, vogel_a: -1.328, vogel_b: 638.4, vogel_c: 95.0 };
    /// 75W manual-gearbox oil (DQ200 gear set sump).
    pub const GEAR_OIL: Fluid = Fluid { rho: 870.0, cp: 2000.0, vogel_a: -1.658, vogel_b: 752.0, vogel_c: 95.0 };

    /// Kinematic viscosity, m²/s.
    pub fn nu(&self, t_c: f64) -> f64 {
        (self.vogel_a + self.vogel_b / (t_c.max(-40.0) + self.vogel_c)).exp() * 1e-6
    }
    /// Dynamic viscosity, Pa·s.
    pub fn mu(&self, t_c: f64) -> f64 {
        self.nu(t_c) * self.rho
    }
}

/// Flow coefficient k (m³/s per √Pa) of a sharp-edged orifice: Q = k·√Δp.
pub fn orifice_k(area_m2: f64, rho: f64) -> f64 {
    CD * area_m2.max(0.0) * (2.0 / rho).sqrt()
}

/// Pressure of a chamber connected to the supply `ps` through `ks` and to tank (0) through `kt`
/// when it must take in `q` (m³/s, negative = expelled). Closed-form inverse of
/// q = ks·sgn√(ps−p) − kt·sgn√p. Non-increasing in `q`. Cavitates at 0.
pub fn valve_chamber_pressure(q: f64, ps: f64, ks: f64, kt: f64) -> f64 {
    let ps = ps.max(0.0);
    let (ks, kt) = (ks.max(1e-15), kt.max(1e-15));
    let sp = ps.sqrt();
    if q >= ks * sp {
        return 0.0;
    }
    if q >= -kt * sp {
        // ks·√(ps−p) − kt·√p = q, w = √p ∈ [0, √ps]
        let a = ks * ks + kt * kt;
        let w = (-q * kt + (q * q * kt * kt - a * (q * q - ps * ks * ks)).max(0.0).sqrt()) / a;
        return w * w;
    }
    // p > ps (piston pushed back into the supply): ks·√(p−ps) + kt·√p = Q
    let qq = -q;
    let a = ks * ks - kt * kt;
    let b = 2.0 * qq * kt;
    let c = -qq * qq - ps * ks * ks;
    let (lo, hi) = (sp, qq / kt);
    let w = if a.abs() < 1e-30 {
        -c / b
    } else {
        let d = (b * b - 4.0 * a * c).max(0.0).sqrt();
        let r1 = (-b + d) / (2.0 * a);
        let r2 = (-b - d) / (2.0 * a);
        if r1 >= lo - 1e-9 && r1 <= hi + 1e-9 { r1 } else { r2 }
    };
    let w = w.clamp(lo, hi.max(lo));
    w * w
}

/// Chamber pressure behind a proportional pressure-reducing valve with set-point `p_set`, supply
/// `ps`, full port coefficient `k` and flow droop `droop` (Pa per m³/s). Non-increasing in `q`.
pub fn prv_chamber_pressure(q: f64, p_set: f64, ps: f64, k: f64, droop: f64) -> f64 {
    let k = k.max(1e-15);
    let p_set = p_set.clamp(0.0, ps.max(0.0));
    let p = if q >= 0.0 { (p_set - droop * q).min(ps - (q / k).powi(2)) } else { (p_set - droop * q).max((q / k).powi(2)) };
    p.max(0.0)
}

/// Coil and spool hardware of a current-driven solenoid valve. UNVERIFIED values (typical
/// automotive proportional solenoids: 3–6 Ω, 5–15 mH, ~1 A rated).
#[derive(Debug, Clone, Copy, Serialize)]
pub struct SolenoidHw {
    pub r20_ohm: f64,
    pub l_h: f64,
    /// Current that produces the full-scale magnetic force (normalised spool = 1).
    pub i_rated_a: f64,
    /// Spool + armature as a mass–spring–damper on the magnetic force: natural frequency (Hz) and damping
    /// ratio. UNVERIFIED (typical proportional solenoid valves: a few tens of Hz bandwidth). Second order, so that
    /// the TCU's current dither (DQ200: 125–150 Hz, ±0.1–0.28 A, set points `0xD000A3D8`) shakes the spool by a
    /// small fraction of its amplitude (as intended: break friction) while set-point changes pass in ≈ 5–10 ms. The
    /// former first-order 4 ms lag passed ≈ 1/4 of the dither to the clutch pressure (±12 bar at 250 bar/A).
    /// `TYPICAL` 40 Hz / ζ 0.7; the DQ200 valves 50 Hz / ζ 0.8 (`mechatronic::Dq200HydraulicsHw::dq200`).
    pub spool_hz: f64,
    pub spool_zeta: f64,
    pub coil_heat_cap_j_k: f64,
    pub coil_to_oil_w_k: f64,
}

impl SolenoidHw {
    pub const TYPICAL: SolenoidHw = SolenoidHw { r20_ohm: 5.0, l_h: 0.012, i_rated_a: 1.0, spool_hz: 40.0, spool_zeta: 0.7, coil_heat_cap_j_k: 25.0, coil_to_oil_w_k: 0.35 };
}

/// Coil insulation / winding failure temperature (class H enamel ≈ 180–220 °C).
const COIL_FAIL_C: f64 = 220.0;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Solenoid {
    pub current_a: f64,
    pub coil_c: f64,
    /// Normalised spool position = magnetic force / full-scale force (0..1.2), and its rate (1/s).
    pub spool: f64,
    pub spool_rate: f64,
    /// Spool seized (contamination, varnish from degraded fluid).
    pub stuck: bool,
    /// Winding burnt open.
    pub open_circuit: bool,
}

impl Solenoid {
    pub fn new(t_c: f64) -> Self {
        Self { current_a: 0.0, coil_c: t_c, spool: 0.0, spool_rate: 0.0, stuck: false, open_circuit: false }
    }

    /// Winding resistance at the present copper temperature.
    pub fn resistance_ohm(&self, hw: &SolenoidHw) -> f64 {
        hw.r20_ohm * (1.0 + 0.00393 * (self.coil_c - 20.0))
    }

    /// `duty`: PWM duty of the TCU low-side driver (None = not driven); averaged over the PWM period.
    /// ponytail: PWM ripple/dither (DQ200: 3 kHz, VERIFIED-S Pwm period 0x341A at 40 MHz) not resolved
    /// at 0.2 ms; current is the period mean. The DQ200 board model integrates the same coil exactly
    /// per PWM edge for the firmware's current sense (`simemu_tc1791::tc1784_io::CoilLoad`).
    /// `measured`: coil current from a board model that integrates the coil per PWM edge (DQ200:
    /// `simemu_tc1791::tc1784_io::CoilLoad`, period mean); it replaces the duty-based estimate.
    pub fn step(&mut self, hw: &SolenoidHw, duty: Option<f64>, measured: Option<f64>, v_supply: f64, oil_c: f64, dt: f64) {
        if !(dt > 0.0 && dt.is_finite()) {
            return;
        }
        if dt > 1e-3 {
            let n = (dt / 1e-3).ceil().clamp(1.0, 10_000.0) as usize;
            let h = dt / n as f64;
            for _ in 0..n {
                self.step_one(hw, duty, measured, v_supply, oil_c, h);
            }
        } else {
            self.step_one(hw, duty, measured, v_supply, oil_c, dt);
        }
    }

    fn step_one(&mut self, hw: &SolenoidHw, duty: Option<f64>, measured: Option<f64>, v_supply: f64, oil_c: f64, dt: f64) {
        let r = self.resistance_ohm(hw);
        let u = duty.unwrap_or(0.0).clamp(0.0, 1.0) * v_supply.max(0.0);
        if self.open_circuit {
            self.current_a = 0.0;
        } else if let Some(i) = measured {
            self.current_a = i.max(0.0);
        } else {
            // Implicit Euler of L di/dt = u − R i (freewheel diode keeps i ≥ 0).
            self.current_a = ((self.current_a + u / hw.l_h * dt) / (1.0 + r / hw.l_h * dt)).max(0.0);
        }
        let p_coil = self.current_a * self.current_a * r;
        self.coil_c += (p_coil - hw.coil_to_oil_w_k * (self.coil_c - oil_c)) / hw.coil_heat_cap_j_k * dt;
        if self.coil_c > COIL_FAIL_C {
            self.open_circuit = true;
        }
        if !self.stuck {
            // Semi-implicit (symplectic) Euler of s'' = ωn²(f − s) − 2ζωn s'; stable for ωn·dt ≪ 1 (0.05 at 40 Hz, 0.2 ms).
            let f = (self.current_a / hw.i_rated_a).clamp(0.0, 1.2);
            let wn = 2.0 * std::f64::consts::PI * hw.spool_hz;
            self.spool_rate = (self.spool_rate + wn * wn * (f - self.spool) * dt) / (1.0 + 2.0 * hw.spool_zeta * wn * dt);
            self.spool = (self.spool + self.spool_rate * dt).clamp(0.0, 1.2);
        }
    }
}

/// Valve set-point for a proportional pressure valve: rising (0 when de-energised) or falling
/// (maximum when de-energised, fail-safe main-pressure valves). 8 % magnetic preload dead band.
pub fn pressure_valve_setpoint(spool: f64, p_max: f64, rising: bool) -> f64 {
    let s = ((spool - 0.08) / 0.92).clamp(0.0, 1.0);
    p_max * if rising { s } else { 1.0 - s }
}

/// Port coefficients (k_supply, k_tank) of a proportional 3/3 flow valve: spool 0.5 = closed
/// centre (hold, ±0.04 overlap), above → supply, below → tank. Same law for a 3/2 switching valve
/// with `overlap = 0` and a steep `gain_band`.
pub fn flow_valve_k(spool: f64, k_max: f64, overlap: f64, gain_band: f64) -> (f64, f64) {
    let leak = LEAK_FRACTION * k_max;
    let open_s = ((spool - 0.5 - overlap) / gain_band).clamp(0.0, 1.0);
    let open_t = ((0.5 - overlap - spool) / gain_band).clamp(0.0, 1.0);
    (leak + k_max * open_s, leak + k_max * open_t)
}

/// How a chamber is fed.
#[derive(Debug, Clone, Copy)]
pub enum Feed {
    /// Orifices to the supply (ps, ks) and to tank (kt).
    Valve { ps: f64, ks: f64, kt: f64 },
    /// Pressure-reducing valve.
    Regulated { p_set: f64, ps: f64, k: f64, droop: f64 },
}

#[derive(Debug, Clone, Copy)]
pub struct Chamber {
    pub area_m2: f64,
    /// +1: pressure pushes the piston towards +x.
    pub dir: f64,
    pub feed: Feed,
}

impl Chamber {
    /// Pressure at piston velocity `v` (m/s).
    pub fn pressure(&self, v: f64) -> f64 {
        let q = self.dir * self.area_m2 * v;
        match self.feed {
            Feed::Valve { ps, ks, kt } => valve_chamber_pressure(q, ps, ks, kt),
            Feed::Regulated { p_set, ps, k, droop } => prv_chamber_pressure(q, p_set, ps, k, droop),
        }
    }
}

const V_MAX: f64 = 1.5;

/// Quasi-static piston: velocity `v` such that Σ dir·A·p(v) + f_ext(v) − c·v − Fc·tanh(v/1 mm/s) = 0.
/// `f_ext(v)` = all other forces along +x evaluated at the END-of-step position x + v·dt (backward
/// Euler on the spring/contact stiffness: unconditionally stable for restoring loads).
/// `stop_pos`/`stop_neg`: motion blocked in that direction (end stop, blocker ring, dog friction).
/// Returns (v, force on the stop at v = 0, positive along +x).
pub fn solve_piston(ch: &[Chamber], f_ext: &dyn Fn(f64) -> f64, c_visc: f64, f_coulomb: f64, stop_pos: bool, stop_neg: bool) -> (f64, f64) {
    let resid = |v: f64| ch.iter().map(|c| c.dir * c.area_m2 * c.pressure(v)).sum::<f64>() + f_ext(v) - c_visc * v - f_coulomb * (v / 1e-3).tanh();
    let r0 = resid(0.0);
    if (r0 >= 0.0 && stop_pos) || (r0 <= 0.0 && stop_neg) {
        return (0.0, r0);
    }
    let (mut lo, mut hi) = if r0 > 0.0 { (0.0, V_MAX) } else { (-V_MAX, 0.0) };
    for _ in 0..30 {
        let m = 0.5 * (lo + hi);
        if resid(m) > 0.0 {
            lo = m;
        } else {
            hi = m;
        }
    }
    (0.5 * (lo + hi), 0.0)
}

/// DC motor driving a fixed-displacement gear pump (Haldex pump; DQ200 V401 without a TCU board).
/// The DQ200 V401 is a sensorless brushless motor commutated by the TCU (`docs/DQ200_IO_MAP.md` §4);
/// with the TCU board attached its motor is integrated on the board and drives the gear pump through
/// [`DcPump::driven`]. Here: averaged DC-motor equivalent (back-EMF k·ω, R), UNVERIFIED values.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct DcPumpHw {
    pub r_ohm: f64,
    /// Back-EMF / torque constant, V·s/rad = N·m/A.
    pub k_motor: f64,
    pub inertia_kgm2: f64,
    pub displacement_m3_rev: f64,
    pub eta_hm: f64,
    /// Laminar slip leakage coefficient: Q_slip = c·Δp/μ, m³.
    pub slip_m3: f64,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct DcPump {
    pub omega: f64,
    pub current_a: f64,
    pub flow_m3_s: f64,
    /// Electrical power drawn, W.
    pub power_w: f64,
}

impl DcPumpHw {
    /// Gear-pump pressure torque per pascal of outlet pressure, D / (2π η_hm), m³.
    pub fn torque_per_pa(&self) -> f64 {
        self.displacement_m3_rev / (2.0 * std::f64::consts::PI * self.eta_hm)
    }

    /// Viscous drag of bearings and gears, N·m·s; scales with the fluid viscosity `mu`. UNVERIFIED.
    pub fn visc_nm_s(mu: f64) -> f64 {
        2e-6 * mu / 0.01
    }
}

impl DcPump {
    /// Pump shaft driven at `omega` by an external motor model (DQ200: the brushless V401 on the TCU
    /// board): delivery and the mechanical input power at outlet pressure `p_out`.
    pub fn driven(&mut self, hw: &DcPumpHw, omega: f64, p_out: f64, mu: f64) {
        self.omega = omega;
        self.current_a = 0.0;
        let w = omega.max(0.0);
        self.power_w = (hw.torque_per_pa() * p_out.max(0.0) + DcPumpHw::visc_nm_s(mu) * w) * w;
        let q_geo = hw.displacement_m3_rev * w / (2.0 * std::f64::consts::PI);
        self.flow_m3_s = (q_geo - hw.slip_m3 * p_out.max(0.0) / mu).max(0.0);
    }

    /// Implicit motor-speed update. `p_out` pump outlet pressure (Pa), `mu` fluid viscosity.
    pub fn step(&mut self, hw: &DcPumpHw, duty: Option<f64>, v_batt: f64, p_out: f64, mu: f64, dt: f64) {
        let u = duty.unwrap_or(0.0).clamp(0.0, 1.0) * v_batt;
        let t_load = hw.torque_per_pa() * p_out.max(0.0);
        let c_visc = DcPumpHw::visc_nm_s(mu);
        let k = hw.k_motor;
        let w = (self.omega * hw.inertia_kgm2 / dt + k * u / hw.r_ohm - t_load) / (hw.inertia_kgm2 / dt + k * k / hw.r_ohm + c_visc);
        self.omega = w.max(0.0);
        self.current_a = ((u - k * self.omega) / hw.r_ohm).max(0.0);
        self.power_w = u * self.current_a;
        let q_geo = hw.displacement_m3_rev * self.omega / (2.0 * std::f64::consts::PI);
        self.flow_m3_s = (q_geo - hw.slip_m3 * p_out.max(0.0) / mu).max(0.0);
    }
}

/// Nitrogen-charged accumulator: pressure from stored oil volume (polytropic n = 1.3).
/// Below the precharge the bladder is empty and only the line compliance remains.
pub fn accumulator_pressure(oil_m3: f64, precharge_pa: f64, gas_v0_m3: f64, line_compliance_m3: f64) -> f64 {
    if oil_m3 >= 0.0 {
        let vg = (gas_v0_m3 - oil_m3).max(0.05 * gas_v0_m3);
        precharge_pa * (gas_v0_m3 / vg).powf(1.3)
    } else {
        (precharge_pa * (1.0 + oil_m3 / line_compliance_m3)).max(0.0)
    }
}

/// d p / d V_oil of the accumulator (for the stability of the relief valve integration).
pub fn accumulator_stiffness(oil_m3: f64, precharge_pa: f64, gas_v0_m3: f64, line_compliance_m3: f64) -> f64 {
    if oil_m3 >= 0.0 {
        let vg = (gas_v0_m3 - oil_m3).max(0.05 * gas_v0_m3);
        1.3 * accumulator_pressure(oil_m3, precharge_pa, gas_v0_m3, line_compliance_m3) / vg
    } else {
        precharge_pa / line_compliance_m3
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chamber_inverse_is_consistent() {
        let (ps, ks, kt) = (50e5, 2e-7, 1e-7);
        for &p in &[0.0f64, 1e5, 20e5, 49e5, 50e5, 70e5, 200e5] {
            let q = ks * (ps - p).signum() * (ps - p).abs().sqrt() - kt * p.sqrt();
            let p2 = valve_chamber_pressure(q, ps, ks, kt);
            assert!((p2 - p).abs() < 1e3, "p {p} → q {q} → {p2}");
        }
    }

    #[test]
    fn viscosity_matches_fit_points() {
        let f = Fluid::ATF;
        assert!((f.nu(40.0) * 1e6 - 30.0).abs() < 3.0);
        assert!((f.nu(100.0) * 1e6 - 7.0).abs() < 0.5);
        assert!(f.nu(-20.0) > 50.0 * f.nu(90.0));
    }
}
