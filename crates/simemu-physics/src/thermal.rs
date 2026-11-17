//! Lumped thermal network: engine (block+head+jacket coolant), radiator, oil, catalyst.
//!
//! Heat sources come from the engine energy balance; coolant routing follows the ITM valve output
//! of the ECU or, when it is not driven, the wax fail-safe thermostat inside the ITM.

use crate::atmosphere::CP_EXH;
use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use serde::Serialize;

const C_ENGINE: f64 = 64e3; // J/K: ~100 kg Al/Fe + ~4 kg coolant in the jacket
const C_RADIATOR: f64 = 8e3; // J/K
const C_OIL: f64 = 9e3; // J/K: 4.5 l oil + sump
const C_CAT: f64 = 2.5e3; // J/K: close-coupled monolith + can
const CP_COOLANT: f64 = 3500.0; // J/(kg·K), G13 50 %
/// Radiator air-side conductance at `V_RADIATOR_REF_MS` (6 m/s core face velocity ≈ 72 km/h road speed), W/K; scales as
/// `(v_air / V_RADIATOR_REF_MS)^0.6` for louvered fins (Kays & London, Compact Heat Exchangers, fig. 10-92; Davenport
/// 1983) in series with coolant-side `UA_RADIATOR_LIQ`. UNVERIFIED (part geometry).
const UA_RADIATOR: f64 = 1500.0;
const UA_RADIATOR_LIQ: f64 = 4500.0;
const V_RADIATOR_REF_MS: f64 = 6.0;
/// Natural-convection buoyancy draft through the stationary radiator + condenser + intercooler stack at ΔT = 80 K
/// (`vehicle_speed_ms = 0`, `fan_duty = 0`), m/s (~1.8 kW passive rejection at standstill). UNVERIFIED.
const V_RADIATOR_NAT_MS: f64 = 0.05;
const UA_OIL_COOLER: f64 = 120.0; // W/K
const A_RADIATOR_M2: f64 = 0.4;
/// Wax fail-safe thermostat in the ITM: starts/finishes opening, °C. UNVERIFIED.
const WAX_OPEN_C: f64 = 105.0;
const WAX_FULL_C: f64 = 120.0;
const CAT_LIGHT_OFF_C: f64 = 250.0;

#[derive(Debug, Clone, Copy, Default)]
pub struct HeatInputs {
    /// Heat into the coolant jacket (head, liner, IEM, ring belt, gasket leak), W.
    pub wall_heat_w: f64,
    /// Heat into the oil (piston under-crown jets, turbo bearing), W.
    pub oil_heat_w: f64,
    pub friction_w: f64,
    pub exhaust_kg_s: f64,
    pub turbine_out_k: f64,
    pub unburned_fuel_w: f64,
    pub excess_air_kg_s: f64,
    pub coolant_pump_kg_s: f64,
    pub coolant_valve_cmd: Option<f64>,
    pub fan_duty: Option<f64>,
    pub vehicle_speed_ms: f64,
    pub air_density: f64,
    pub ambient_c: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct ThermalState {
    pub engine_c: f64,
    pub radiator_c: f64,
    pub oil_c: f64,
    pub cat_c: f64,
    pub radiator_valve_open: f64,
    pub radiator_heat_w: f64,
}

impl ThermalState {
    pub fn uniform(t_c: f64) -> Self {
        Self { engine_c: t_c, radiator_c: t_c, oil_c: t_c, cat_c: t_c, radiator_valve_open: 0.0, radiator_heat_w: 0.0 }
    }

    pub fn step(&mut self, h: &HeatInputs, dt: f64) {
        if !(dt > 0.0 && dt.is_finite()) {
            return;
        }
        // Sub-step for the nonlinear wax thermostat and radiator ε-NTU; each step solves the coupled
        // 3-node (engine, radiator, oil) conductance network implicitly via nalgebra::Matrix3.
        let n = (dt / 0.05).ceil().clamp(1.0, 10_000.0) as usize;
        let dt = dt / n as f64;
        for _ in 0..n {
            let amb = h.ambient_c;
            let wax = ((self.engine_c - WAX_OPEN_C) / (WAX_FULL_C - WAX_OPEN_C)).clamp(0.0, 1.0);
            let valve = h.coolant_valve_cmd.filter(|v| v.is_finite()).unwrap_or(0.0).clamp(0.0, 1.0).max(wax);
            self.radiator_valve_open = valve;
            let m_rad = h.coolant_pump_kg_s.max(0.0) * valve;
            let g_eng_rad = m_rad * CP_COOLANT;

            // Radiator: lumped core node (`C_RADIATOR` at `radiator_c` = G83) fed in series by `g_eng_rad = m_rad * CP_COOLANT`
            // and cooled by unmixed crossflow air `c_air * (1 - exp(-ua / c_air))` (Kays & London `C_r -> 0` on the lumped node,
            // avoiding double-counting `C_cool` and allowing J293 / ram air to cool the stagnant core during Nachlauf when
            // `m_rad == 0`), with louvered-fin air-side UA ∝ (ρ·v_air)^0.6.
            let rho_air = if h.air_density.is_finite() { h.air_density.max(0.0) } else { 0.0 };
            let fan = h.fan_duty.filter(|d| d.is_finite()).unwrap_or(0.0).clamp(0.0, 1.0);
            let v_spd = if h.vehicle_speed_ms.is_finite() { h.vehicle_speed_ms.abs() } else { 0.0 };
            let v_nat = V_RADIATOR_NAT_MS * ((self.radiator_c - amb).max(0.0) / 80.0).sqrt();
            let v_air = 0.30 * v_spd + 4.0 * fan + v_nat;
            let c_air = rho_air * A_RADIATOR_M2 * v_air * 1005.0;
            let g_rad_amb = if c_air > 1e-3 {
                let ua_air = UA_RADIATOR * ((rho_air / 1.2) * (v_air / V_RADIATOR_REF_MS)).max(1e-6).powf(0.6);
                // Stagnant coolant in flat Al tubes (~500 W/K) rising to turbulent forced flow `UA_RADIATOR_LIQ`. UNVERIFIED.
                let ua_liq = 500.0 + (UA_RADIATOR_LIQ - 500.0) * (m_rad / 0.5).clamp(0.0, 1.0);
                let ua = (ua_air * ua_liq) / (ua_air + ua_liq);
                c_air * (-(-ua / c_air).exp_m1())
            } else {
                0.0
            };

            // Enclosed engine bay + acoustic belly pan at standstill (~10 + 5 W/K), flushed by underhood airflow at speed. UNVERIFIED.
            let g_eng_amb = 10.0 * (1.0 + v_spd / 5.0);
            let g_oil_cool = UA_OIL_COOLER;
            let g_oil_amb = 5.0 * (1.0 + v_spd / 5.0);

            let to_coolant = h.wall_heat_w + 0.5 * h.friction_w;
            let to_oil = h.oil_heat_w + 0.5 * h.friction_w;

            let d0 = C_ENGINE + dt * (g_eng_rad + g_oil_cool + g_eng_amb);
            let e1 = -dt * g_eng_rad;
            let e2 = -dt * g_oil_cool;
            let d1 = C_RADIATOR + dt * (g_eng_rad + g_rad_amb);
            let d2 = C_OIL + dt * (g_oil_cool + g_oil_amb);
            let c00 = d1 * d2;
            let c01 = -e1 * d2;
            let c02 = -e2 * d1;
            let det = d0 * c00 + e1 * c01 + e2 * c02;
            let adj = Matrix3::new(
                c00,
                c01,
                c02,
                c01,
                d0 * d2 - e2 * e2,
                e1 * e2,
                c02,
                e1 * e2,
                d0 * d1 - e1 * e1,
            );
            let rhs = Vector3::new(
                C_ENGINE * self.engine_c + dt * (to_coolant + g_eng_amb * amb),
                C_RADIATOR * self.radiator_c + dt * (g_rad_amb * amb),
                C_OIL * self.oil_c + dt * (to_oil + g_oil_amb * amb),
            );
            let t_next = (adj * rhs) / det;
            self.engine_c = t_next[0];
            self.radiator_c = t_next[1];
            self.oil_c = t_next[2];
            self.radiator_heat_w = g_rad_amb * (self.radiator_c - amb);

            // Catalyst: convective heating by exhaust + oxidation of unburned fuel with excess O2.
            let t_gas_c = h.turbine_out_k - 273.15;
            let q_conv = h.exhaust_kg_s * CP_EXH * 0.9 * (t_gas_c - self.cat_c);
            let exo = if self.cat_c > CAT_LIGHT_OFF_C {
                h.unburned_fuel_w.min(h.excess_air_kg_s / 14.7 * 43.0e6) * 0.9
            } else {
                0.0
            };
            let q_cat_amb = 6.0 * (1.0 + v_spd / 10.0) * (self.cat_c - amb);
            self.cat_c += (q_conv + exo - q_cat_amb) / C_CAT * dt;
        }
    }
}

// ================================================================================================================
// Combustion-chamber component nodes (per cylinder, advanced on the crank-angle grid by `cylinder`)
// ================================================================================================================

/// Piston: three lumped nodes — crown centre, crown edge + top land, first ring groove (+ second land) — after the
/// node layout of MAHLE GmbH (ed.), "Pistons and Engine Testing", 2nd ed., Springer Vieweg 2016, ch. 3 (temperature
/// fields of gasoline pistons with cooling jets). Heat enters crown and edge from the gas (Hohenberg), flows radially to
/// the edge and axially to the ring belt, leaves through the rings into the liner and from the crown underside into the
/// oil jet / splash. Conductances from AlSi12CuNiMg (k ≈ 140 W/m·K) over the EA888 Ø82.5 mm crown (≈ 7 mm thick:
/// G ≈ 2πkt/ln(r₂/r₁) ≈ 7 W/K; land wall 4 mm over 8 mm: ≈ 20 W/K). Ring-pack and jet UA UNVERIFIED, chosen so that
/// rated power gives the MAHLE ranges for turbo-GDI pistons with jets (crown ≈ 280–320 °C, first groove ≈ 220–260 °C).
#[derive(Debug, Clone, Serialize)]
pub struct PistonThermalHw {
    /// Heat capacities of the crown, land and groove nodes, J/K.
    pub c_crown: f64,
    pub c_land: f64,
    pub c_groove: f64,
    /// Crown → land (radial) and land → groove (axial) conductances, W/K.
    pub g_crown_land: f64,
    pub g_land_groove: f64,
    /// Share of the piston's gas-side area that belongs to the crown-centre node (rest: edge + top land).
    pub crown_area_share: f64,
    /// Oil jet on the crown underside at full jet flow, splash/mist on the underside, W/K (to the oil).
    pub ua_jet: f64,
    pub ua_splash: f64,
    /// Ring belt → rings → liner, W/K.
    pub ua_ring: f64,
}

impl PistonThermalHw {
    pub fn ea888_gen3() -> Self {
        Self { c_crown: 90.0, c_land: 50.0, c_groove: 40.0, g_crown_land: 7.0, g_land_groove: 20.0, crown_area_share: 0.75, ua_jet: 8.5, ua_splash: 3.0, ua_ring: 8.0 }
    }

    /// Advances [crown, land, groove] (K) by `dt` with gas heat `q_crown`/`q_land` (J this step, film evaporation already
    /// subtracted), oil-jet effectiveness `jet` (0..1), oil and liner temperatures. Returns heat to (oil, coolant), J.
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub fn step(&self, t: &mut [f64; 3], q_crown: f64, q_land: f64, jet: f64, t_oil: f64, t_liner: f64, dt: f64) -> (f64, f64) {
        let g_oil = self.ua_jet * jet + self.ua_splash;
        let g_cl = self.g_crown_land;
        let g_lg = self.g_land_groove;
        let g_ring = self.ua_ring;
        let d0 = self.c_crown + dt * (g_cl + g_oil);
        let e0 = -dt * g_cl;
        let d1 = self.c_land + dt * (g_cl + g_lg);
        let e1 = -dt * g_lg;
        let d2 = self.c_groove + dt * (g_lg + g_ring);
        let c00 = d1 * d2 - e1 * e1;
        let c01 = -e0 * d2;
        let c02 = e0 * e1;
        let det = d0 * c00 + e0 * c01;
        let adj = Matrix3::new(
            c00,
            c01,
            c02,
            c01,
            d0 * d2,
            -d0 * e1,
            c02,
            -d0 * e1,
            d0 * d1 - e0 * e0,
        );
        let rhs = Vector3::new(
            self.c_crown * t[0] + q_crown + dt * g_oil * t_oil,
            self.c_land * t[1] + q_land,
            self.c_groove * t[2] + dt * g_ring * t_liner,
        );
        let t_next = (adj * rhs) / det;
        t[0] = t_next[0];
        t[1] = t_next[1];
        t[2] = t_next[2];
        let q_oil = g_oil * (t[0] - t_oil) * dt;
        let q_ring = g_ring * (t[2] - t_liner) * dt;
        (q_oil, q_ring)
    }
}

/// Exhaust valves of a cylinder (lumped): head/neck node and seat/margin node. The head takes the in-cylinder face heat
/// and most of the port convection during blowdown, the seat node the hot jet through the seat gap; it is cooled by the
/// seat insert while closed (the larger part of the exhaust-valve heat leaves through the seat — Heywood 1988 §12.6) and
/// by the stem/guide. Capacities from the 28 mm EA888 valve (SSP 606); conductances UNVERIFIED (the former single node's
/// seat 2.0 W/K and stem 0.4 W/K kept).
#[derive(Debug, Clone, Serialize)]
pub struct ValveThermalHw {
    pub c_head: f64,
    pub c_seat: f64,
    pub g_head_seat: f64,
    pub ua_seat_closed: f64,
    pub ua_stem: f64,
    /// Port film coefficient at 50 g/s per valve, W/(m²·K) (∝ ṁ^0.8).
    pub port_h_ref: f64,
}

impl ValveThermalHw {
    pub fn ea888_gen3_exhaust() -> Self {
        Self { c_head: 20.0, c_seat: 5.0, g_head_seat: 1.5, ua_seat_closed: 2.0, ua_stem: 0.4, port_h_ref: 1500.0 }
    }

    /// Advances [head, seat] by `dt`: `q_face` gas heat into the head face, `q_port` port convection (2:1 head:seat),
    /// `closed`, head/seat-insert temperature `t_wall`. Returns heat to the coolant, J.
    #[inline]
    pub fn step(&self, t: &mut [f64; 2], q_face: f64, q_port: f64, closed: bool, t_wall: f64, dt: f64) -> f64 {
        let g_hs = self.g_head_seat;
        let g_stem = self.ua_stem;
        let g_seat = if closed { self.ua_seat_closed } else { 0.0 };
        let d0 = self.c_head + dt * (g_hs + g_stem);
        let e0 = -dt * g_hs;
        let d1 = self.c_seat + dt * (g_hs + g_seat);
        let det = d0 * d1 - e0 * e0;
        let adj = Matrix2::new(d1, -e0, -e0, d0);
        let rhs = Vector2::new(
            self.c_head * t[0] + q_face + 0.67 * q_port + dt * g_stem * t_wall,
            self.c_seat * t[1] + 0.33 * q_port + dt * g_seat * t_wall,
        );
        let t_next = (adj * rhs) / det;
        t[0] = t_next[0];
        t[1] = t_next[1];
        (g_seat * (t[1] - t_wall) + g_stem * (t[0] - t_wall)) * dt
    }
}

/// Spark plug: insulator-nose tip and ground-electrode nodes. The heat range is the conductance of the insulator nose to
/// the shell (a "hot" plug has a long nose, a "cold" one a short nose). NGK / Bosch technical information: the nose should
/// run between the self-cleaning temperature ≈ 450–500 °C and the pre-ignition onset ≈ 850–950 °C, and one heat-range
/// step moves the tip by ≈ 70–100 K. Fouling: below ≈ 450 °C carbon deposits on the nose and shunts the secondary
/// voltage (misfire); above ≈ 500 °C it burns off.
#[derive(Debug, Clone, Serialize)]
pub struct SparkPlugHw {
    /// NGK-scale heat range (higher = colder).
    pub heat_range: f64,
    /// Gas-wetted area of the insulator nose and of the ground electrode, m².
    pub tip_area_m2: f64,
    pub gnd_area_m2: f64,
    /// Heat capacities, J/K.
    pub c_tip: f64,
    pub c_gnd: f64,
    /// Nose → shell conductance at heat range 7, W/K, and its factor per heat-range step (FITTED to ≈ 80 K/step at WOT).
    pub g_tip_hr7: f64,
    pub g_step: f64,
    /// Ground electrode → shell conductance, W/K.
    pub g_gnd: f64,
    /// Carbon-fouling time constant below the self-cleaning limit (stoichiometric homogeneous charge), s; burn-off time
    /// constant 100 K above the limit, s. UNVERIFIED.
    pub foul_tau_s: f64,
    pub burnoff_tau_s: f64,
}

impl SparkPlugHw {
    /// VW OE plug of the EA888 Gen3 (06K905601B = NGK PLFER7A8EG, heat range 7). Areas/capacities UNVERIFIED.
    pub fn ea888_oe() -> Self {
        Self { heat_range: 7.0, tip_area_m2: 40e-6, gnd_area_m2: 12e-6, c_tip: 0.12, c_gnd: 0.03, g_tip_hr7: 0.032, g_step: 1.3, g_gnd: 0.012, foul_tau_s: 1800.0, burnoff_tau_s: 5.0 }
    }

    #[inline]
    pub fn g_tip(&self) -> f64 {
        let d = self.heat_range - 7.0;
        if d == 0.0 {
            self.g_tip_hr7
        } else {
            self.g_tip_hr7 * self.g_step.powf(d)
        }
    }

    /// Advances [tip, ground] with gas heat `q_tip`/`q_gnd` (J) and the shell (head) temperature.
    #[inline]
    pub fn step(&self, t: &mut [f64; 2], q_tip: f64, q_gnd: f64, t_shell: f64, dt: f64) {
        t[0] += (q_tip - self.g_tip() * (t[0] - t_shell) * dt) / self.c_tip;
        t[1] += (q_gnd - self.g_gnd * (t[1] - t_shell) * dt) / self.c_gnd;
    }

    /// Carbon-fouling index (0 clean … 1 shorted) after `dt` at tip temperature `t_tip` K; `soot` ≥ 0 scales the deposit
    /// source (1 = stoichiometric homogeneous charge; more for rich pockets / misfires).
    #[inline]
    pub fn foul(&self, c: f64, t_tip: f64, soot: f64, dt: f64) -> f64 {
        let t_c = t_tip - 273.15;
        let grow = if t_c < 450.0 { soot * (1.0 - c) * dt / self.foul_tau_s } else { 0.0 };
        let burn = if t_c > 500.0 { c * (1.0 - (-dt / self.burnoff_tau_s * ((t_c - 500.0) / 100.0).min(3.0)).exp()) } else { 0.0 };
        (c + grow - burn).clamp(0.0, 1.0)
    }
}

/// Combustion-chamber deposit hot spot (the thickest carbon on the crown edge / exhaust side): an insulating layer of
/// thickness δ over the metal. Its surface runs hotter than the metal by ≈ q''·δ/k (CCD thermal conductivity
/// 0.1–0.5 W/m·K: Nakic, Assanis & White, SAE 940948; Anderson & Prakash, SAE 852111) — the thermal part of the
/// deposit-induced octane-requirement increase and a surface-ignition source.
#[derive(Debug, Clone, Serialize)]
pub struct DepositHw {
    pub k_w_mk: f64,
    /// Volumetric heat capacity of the deposit, J/(m³·K).
    pub rho_c: f64,
    /// Spot area, m² (cancels in the surface temperature; sets the node's own capacity).
    pub area_m2: f64,
}

impl DepositHw {
    pub fn ccd() -> Self {
        Self { k_w_mk: 0.3, rho_c: 1.5e6, area_m2: 1e-4 }
    }

    /// Advances the deposit-layer temperature `t` (K) on a substrate at `t_sub` with gas heat `q_gas` (J over the spot
    /// area) and thickness `delta_m`; without deposit the surface is the substrate.
    #[inline]
    pub fn step(&self, t: &mut f64, q_gas: f64, t_sub: f64, delta_m: f64, dt: f64) {
        if delta_m <= 1e-6 {
            *t = t_sub;
            return;
        }
        let c = self.rho_c * delta_m * self.area_m2;
        let g = 2.0 * self.k_w_mk / delta_m * self.area_m2; // layer centre → substrate: half-thickness resistance
        *t += q_gas / c - (g * dt / c).min(1.0) * (*t - t_sub);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cold_engine_without_heat_stays_at_ambient() {
        let mut t = ThermalState::uniform(5.0);
        t.step(&HeatInputs { ambient_c: 5.0, air_density: 1.27, ..Default::default() }, 60.0);
        assert!((t.engine_c - 5.0).abs() < 1e-9);
    }

    #[test]
    fn wax_thermostat_caps_coolant_without_itm_command() {
        // 30 kW into the coolant at highway speed, no ECU valve command: wax regulates ~105–120 °C.
        let mut t = ThermalState::uniform(90.0);
        let h = HeatInputs { wall_heat_w: 30e3, coolant_pump_kg_s: 0.6, vehicle_speed_ms: 30.0, air_density: 1.2, ambient_c: 20.0, ..Default::default() };
        for _ in 0..3600 {
            t.step(&h, 1.0);
        }
        assert!(t.engine_c > WAX_OPEN_C && t.engine_c < WAX_FULL_C, "engine {}", t.engine_c);
    }

    #[test]
    fn radiator_fan_cools_stagnant_core_during_nachlauf_and_small_valve_bleed_drops_g83() {
        // Stagnant Nachlauf (`coolant_pump_kg_s = 0`, `vehicle_speed_ms = 0`): J293 at 80 % must cool the radiator core
        // much faster than passive natural draft alone.
        let mut t_off = ThermalState::uniform(105.0);
        let mut t_fan = ThermalState::uniform(105.0);
        t_off.step(&HeatInputs { ambient_c: 25.0, air_density: 1.18, fan_duty: Some(0.0), ..Default::default() }, 30.0);
        t_fan.step(&HeatInputs { ambient_c: 25.0, air_density: 1.18, fan_duty: Some(0.8), ..Default::default() }, 30.0);
        assert!(
            t_off.radiator_c - t_fan.radiator_c > 35.0,
            "J293 must rapidly cool stagnant Nachlauf radiator: fan_off={:.1} °C vs fan_on={:.1} °C",
            t_off.radiator_c,
            t_fan.radiator_c
        );

        // Highway cruise in 0 °C air with tiny ITM bleed (`valve = 2 %`): G83 (`radiator_c`) must approach cold ambient,
        // not stall at `(T_eng + T_amb) / 2`.
        let mut t_hwy = ThermalState::uniform(90.0);
        let h_hwy = HeatInputs {
            wall_heat_w: 6e3,
            coolant_pump_kg_s: 1.0,
            coolant_valve_cmd: Some(0.02),
            vehicle_speed_ms: 30.0,
            air_density: 1.29,
            ambient_c: 0.0,
            ..Default::default()
        };
        for _ in 0..300 {
            t_hwy.step(&h_hwy, 1.0);
        }
        assert!(t_hwy.radiator_c < 15.0, "G83 at 2 % ITM bleed in 0 °C highway air must cool near ambient, got {:.1} °C", t_hwy.radiator_c);
    }
}
