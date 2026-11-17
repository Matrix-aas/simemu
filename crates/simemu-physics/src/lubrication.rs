//! Lubrication: two-stage variable-displacement vane pump, oil gallery network, crank journal
//! bearings and the turbocharger bearing.
//!
//! Gallery: consumers are laminar conductances (∝ clearance³/μ); the pump delivers
//! V_disp·n·(1 − aeration) and regulates down to the stage pressure selected by N428; a mechanical
//! relief caps cold pressure. Sump: when the oil level falls the pickup sucks air (aeration).
//!
//! Journal bearings (5 mains, 4 rods): 1-D radial film model, Ocvirk short-bearing load capacity
//! W_h = μ·ω·R·L³/(4C²)·ε/(1−ε²)²·√(π²(1−ε²)+16ε²), short-bearing squeeze film
//! W_s = π·μ·R·L³/(2C²)·(1+2ε²)/(1−ε²)^2.5·ε̇, Greenwood–Tripp asperity load
//! p_a = K·F_5/2(Λ), F_5/2 ≈ 4.4086e-5·(4−Λ)^6.804, Λ = h_min/σ. Oil fill fraction drains through the
//! hydrodynamic side leakage U·C·L·ε and refills from the gallery feed → oil starvation collapses the
//! film within about one revolution. Friction (Petroff + boundary μ·W_a) heats a lumped shell node
//! cooled by the oil through-flow; viscosity follows the shell temperature. Above the Sn melting point
//! (232 °C) of the AlSn20 lining the contact turns adhesive; Archard wear opens the clearance; at
//! 450 °C or with the lining worn through the bearing seizes.

use crate::atmosphere::oil_viscosity_cst;
use crate::profiles::{EngineProfile, JournalBearing};
use serde::Serialize;
use std::f64::consts::PI;

pub const N_BEARINGS: usize = 9;
const OIL_RHO: f64 = 850.0;
const OIL_CP: f64 = 2000.0;
/// High-shear viscosity relative to the low-shear kinematic value (5W-40 shear thinning). UNVERIFIED.
const SHEAR_THIN: f64 = 0.75;
/// Gallery conductances at 90 °C, m³/(s·Pa): one crank bearing at nominal clearance; head
/// (HLAs, cam bearings, phasers, chain tensioner); piston jets (4); turbo. UNVERIFIED.
const G_BEARING: f64 = 6.25e-11;
const G_HEAD: f64 = 3.6e-10;
const G_JETS: f64 = 2.5e-10;
const G_TURBO: f64 = 4.0e-11;
/// Piston-jet check valve cracking pressure, Pa. UNVERIFIED.
const JET_CRACK_PA: f64 = 1.8e5;
const RELIEF_PA: f64 = 6.5e5;
/// Oil held in galleries/head and drain-back hold-up per 1000 rpm, l; sump volume below which the
/// pickup starts to draw air, l. UNVERIFIED.
const OIL_IN_CIRCUIT_L: f64 = 1.2;
const OIL_HOLDUP_L_PER_KRPM: f64 = 0.15;
const PICKUP_COVER_L: f64 = 1.0;
/// Sump level at the oil level sender G266 with the engine filled to the MAX mark and stopped, mm, and its change per
/// litre in the sump (sump plan area ≈ 500 cm²). UNVERIFIED (no sump drawing); the firmware's own sender curve
/// `ip_loil_srv_intl` spans 0–120 mm.
const SUMP_FULL_MM: f64 = 100.0;
const SUMP_MM_PER_L: f64 = 20.0;
/// Drain time constant of the piston-jet gallery behind the closed N522 switching valve (leakage), s. UNVERIFIED.
const JET_GALLERY_DRAIN_S: f64 = 0.3;
/// Greenwood–Tripp constant (16√2/15)·π·(ηβσ)²·√(σ/β)·E' with ηβσ = 0.05, σ/β = 1e-3, E' = 110 GPa. UNVERIFIED.
const K_GT_PA: f64 = 5.9e7;
const MU_BOUNDARY: f64 = 0.08;
const MU_ADHESIVE: f64 = 0.35;
const T_SN_MELT_K: f64 = 505.15;
const T_SEIZE_K: f64 = 723.15;
const SHELL_C_J_K: f64 = 60.0;
const SHELL_G_W_K: f64 = 3.0;
const LINING_HARDNESS_PA: f64 = 0.45e9;
const ARCHARD_K: f64 = 2e-6;
const LINING_M: f64 = 0.25e-3;
/// Turbo bearing housing: heat capacity, conductance to the turbine side and to the water jacket. UNVERIFIED.
const TB_C: f64 = 300.0;
const TB_UA_HOT: f64 = 1.5;
const TB_UA_WATER: f64 = 10.0;
const TB_SEIZE_K: f64 = 623.15;

/// Dynamic viscosity of the film at `t_k`, Pa·s.
#[inline]
pub fn film_viscosity(t_k: f64) -> f64 {
    oil_viscosity_cst(t_k - 273.15) * 1e-6 * OIL_RHO * SHEAR_THIN
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Bearing {
    pub eps: f64,
    /// Oil fill fraction of the clearance (1 = fully flooded).
    pub fill: f64,
    pub t_k: f64,
    pub wear_m: f64,
    pub seized: bool,
    pub load_n: f64,
    /// Minimum film thickness in the running / last cycle, m.
    pub film_run_m: f64,
    pub film_last_m: f64,
    pub friction_w: f64,
}

impl Bearing {
    fn new(t_k: f64) -> Self {
        Self { eps: 0.3, fill: 1.0, t_k, wear_m: 0.0, seized: false, load_n: 0.0, film_run_m: f64::MAX, film_last_m: 0.0, friction_w: 0.0 }
    }
}

#[inline]
fn ocvirk(e: f64) -> f64 {
    let q = 1.0 - e * e;
    e / (q * q) * (PI * PI * q + 16.0 * e * e).sqrt()
}

#[inline]
fn squeeze(e: f64) -> f64 {
    let q = 1.0 - e * e;
    (1.0 + 2.0 * e * e) / (q * q * q.sqrt())
}

#[inline]
fn gt_pressure(lambda: f64) -> f64 {
    if lambda >= 4.0 { 0.0 } else { K_GT_PA * 4.4086e-5 * (4.0 - lambda).powf(6.804) }
}

#[derive(Debug, Clone, Serialize)]
pub struct Lubrication {
    pub p_gallery_pa: f64,
    /// Piston-jet gallery downstream of the switching valve (N522 de-energised = open), where the jet oil-pressure switch
    /// F447 sits, Pa.
    pub p_jets_pa: f64,
    /// Oil level in the sump (fill minus the oil held in the circuit) at the oil level sender, mm.
    pub sump_level_mm: f64,
    /// 0..4 main bearings, 5..8 rod bearings (cylinders 1..4).
    pub bearings: [Bearing; N_BEARINGS],
    pub oil_l: f64,
    pub aeration: f64,
    pub jets_flowing: bool,
    pub pump_power_w: f64,
    pub turbo_bearing_k: f64,
    pub turbo_fill: f64,
    pub turbo_seized: bool,
}

impl Lubrication {
    pub fn new(p: &EngineProfile, t_k: f64) -> Self {
        Self {
            p_gallery_pa: 0.0,
            p_jets_pa: 0.0,
            sump_level_mm: SUMP_FULL_MM,
            bearings: [Bearing::new(t_k); N_BEARINGS],
            oil_l: p.mech.oil_capacity_l,
            aeration: 0.0,
            jets_flowing: false,
            pump_power_w: 0.0,
            turbo_bearing_k: t_k,
            turbo_fill: 1.0,
            turbo_seized: false,
        }
    }

    pub fn geometry(p: &EngineProfile, b: usize) -> &JournalBearing {
        if b < 5 { &p.mech.main_bearing } else { &p.mech.rod_bearing }
    }

    /// Pump + gallery pressure, per sub-step.
    pub fn step_gallery(&mut self, p: &EngineProfile, rpm: f64, oil_c: f64, low_stage: bool, jets_enabled: bool, dt: f64) {
        let mu_ratio = (oil_viscosity_cst(oil_c) / oil_viscosity_cst(90.0)).max(0.05);
        let mut g = (G_HEAD + G_TURBO) / mu_ratio;
        for (b, br) in self.bearings.iter().enumerate() {
            let c0 = Self::geometry(p, b).clearance_m;
            g += G_BEARING * ((c0 + br.wear_m) / c0).powi(3) / mu_ratio;
        }
        let avail = self.oil_l - OIL_IN_CIRCUIT_L - OIL_HOLDUP_L_PER_KRPM * rpm.abs() / 1000.0;
        self.sump_level_mm = SUMP_FULL_MM + (avail - (p.mech.oil_capacity_l - OIL_IN_CIRCUIT_L)) * SUMP_MM_PER_L;
        self.aeration = ((PICKUP_COVER_L - avail) / PICKUP_COVER_L).clamp(0.0, 1.0);
        let q_max = p.mech.oil_pump_m3_rev * rpm.max(0.0) / 60.0 * 0.92 * (1.0 - self.aeration);
        let stage = if low_stage { 1.9e5 } else { 3.8e5 };
        let regulated = (stage * (1.0 + 0.5 * (mu_ratio - 1.0).max(0.0))).min(RELIEF_PA);
        let g_jets = G_JETS / mu_ratio;
        let mut target = (q_max / g).min(regulated);
        self.jets_flowing = false;
        if jets_enabled && target > JET_CRACK_PA {
            let with = ((q_max + g_jets * JET_CRACK_PA) / (g + g_jets)).min(regulated);
            if with > JET_CRACK_PA {
                target = with;
                self.jets_flowing = true;
            }
        }
        self.p_gallery_pa += (target - self.p_gallery_pa) * (dt / 0.02).min(1.0);
        // jet gallery: fed from the main gallery through the open switching valve, leaks down behind the closed one
        let (p_j, tau) = if jets_enabled { (self.p_gallery_pa, 0.02) } else { (0.0, JET_GALLERY_DRAIN_S) };
        self.p_jets_pa += (p_j - self.p_jets_pa) * (dt / tau).min(1.0);
        let q = g * self.p_gallery_pa + if self.jets_flowing { g_jets * (self.p_gallery_pa - JET_CRACK_PA).max(0.0) } else { 0.0 };
        self.pump_power_w = q * self.p_gallery_pa / 0.6;
    }

    /// One journal bearing over `dt` under radial load `load` (N) at journal speed `omega`.
    /// Returns the friction torque magnitude, Nm.
    pub fn step_bearing(&mut self, p: &EngineProfile, b: usize, load: f64, omega: f64, t_oil_k: f64, dt: f64) -> f64 {
        let mu_oil = (oil_viscosity_cst(t_oil_k - 273.15) / oil_viscosity_cst(90.0)).max(0.05);
        self.step_bearing_with_mu(p, b, load, omega, t_oil_k, mu_oil, dt)
    }

    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub fn step_bearing_with_mu(&mut self, p: &EngineProfile, b: usize, load: f64, omega: f64, t_oil_k: f64, mu_oil: f64, dt: f64) -> f64 {
        let geo = Self::geometry(p, b).clone();
        let sigma = p.mech.bearing_roughness_m;
        let p_sup = self.p_gallery_pa;
        let br = &mut self.bearings[b];
        let r = 0.5 * geo.diameter_m;
        let l = geo.width_m;
        let c = geo.clearance_m + br.wear_m;
        let mu = film_viscosity(br.t_k);
        let w = omega.abs();
        let u = w * r;
        br.load_n = load;

        // Oil fill: gallery feed vs hydrodynamic side leakage.
        let q_sup = G_BEARING * (c / geo.clearance_m).powi(3) / mu_oil * p_sup;
        let q_side = u * c * l * br.eps;
        br.fill = (br.fill + (q_sup - br.fill * q_side) / (2.0 * PI * r * l * c) * dt).clamp(0.0, 1.0);

        // Radial film dynamics, linearly-implicit (Rosenbrock) Euler step.
        let k_o = mu * w * r * l.powi(3) / (4.0 * c * c);
        let k_s = PI * mu * r * l.powi(3) / (2.0 * c * c);
        let a_c = r * l;
        let e_max = 1.0 - 0.05 * sigma / c;
        let fill = br.fill;
        let f = |e: f64| {
            let lam = c * (1.0 - e) / sigma;
            (load - fill * k_o * ocvirk(e) - a_c * gt_pressure(lam)) / ((fill + 1e-3) * k_s * squeeze(e))
        };
        let e0 = br.eps.min(e_max);
        let f0 = f(e0);
        let de = 1e-5;
        let fp = if e0 + de <= e_max { (f(e0 + de) - f0) / de } else { (f0 - f(e0 - de)) / de };
        let mut e1 = e0 + dt * f0 / (1.0 - dt * fp.min(0.0));
        if !e1.is_finite() {
            e1 = e0;
        }
        let clamped = e1 >= e_max;
        br.eps = e1.clamp(0.0, e_max);
        let h_min = c * (1.0 - br.eps);
        if omega > 1.0 {
            br.film_run_m = br.film_run_m.min(h_min);
        }

        // Friction: hydrodynamic shear + asperity contact (all remaining load when bottomed out).
        let lam = h_min / sigma;
        let w_hyd = fill * k_o * ocvirk(br.eps);
        // Asperities can never carry more than the applied load; when the film is squeezed to the
        // minimum, they carry everything the fluid cannot.
        let mut w_asp = (a_c * gt_pressure(lam)).min(load);
        if clamped {
            w_asp = w_asp.max(load - w_hyd);
        }
        let mu_b = if br.t_k > T_SN_MELT_K { MU_ADHESIVE } else { MU_BOUNDARY };
        let f_h = fill * mu * u * 2.0 * PI * r * l / (c * (1.0 - br.eps * br.eps).sqrt());
        let f_a = mu_b * w_asp;
        let p_fr = (f_h + f_a) * u;
        br.friction_w = p_fr;

        // Shell temperature and wear.
        // Shell node, integrated exactly over the step (the oil through-flow conductance can be large).
        let g = OIL_RHO * OIL_CP * q_sup.max(fill * q_side) + SHELL_G_W_K;
        let t_eq = t_oil_k + p_fr / g;
        br.t_k = t_eq + (br.t_k - t_eq) * (-g * dt / SHELL_C_J_K).exp();
        let k_w = if br.t_k > T_SN_MELT_K { 50.0 * ARCHARD_K } else { ARCHARD_K };
        br.wear_m = (br.wear_m + k_w * (w_asp / a_c) * u * dt / LINING_HARDNESS_PA).min(2.0 * LINING_M);
        if br.t_k > T_SEIZE_K || br.wear_m > LINING_M {
            br.seized = true;
        }
        (f_h + f_a) * r
    }

    /// Floating-ring turbo bearing: oil-fed, heat-soaked from the turbine. Returns the extra
    /// bearing friction power (W) and seizes when the oil cokes / bronze galls.
    pub fn step_turbo_bearing(&mut self, omega_t: f64, t_exh_k: f64, t_cool_k: f64, t_oil_k: f64, dt: f64) -> f64 {
        let q = G_TURBO * self.p_gallery_pa;
        // Bearing cavity drains in ~0.5 s without feed.
        self.turbo_fill = (self.turbo_fill + (q / 1.2e-5 - self.turbo_fill) * dt / 0.5).clamp(0.0, 1.0);
        let w = omega_t.abs();
        let base = 1.2e-6 * w * w + 2e-11 * w * w * w;
        let p_fr = base * (1.0 + 30.0 * (1.0 - self.turbo_fill).powi(2));
        let t = self.turbo_bearing_k;
        let cool = OIL_RHO * OIL_CP * q;
        let g = cool + TB_UA_HOT + TB_UA_WATER;
        let t_eq = (p_fr + TB_UA_HOT * t_exh_k + cool * t_oil_k + TB_UA_WATER * t_cool_k) / g;
        self.turbo_bearing_k = t_eq + (t - t_eq) * (-g * dt / TB_C).exp();
        if self.turbo_bearing_k > TB_SEIZE_K {
            self.turbo_seized = true;
        }
        p_fr
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_bearing_keeps_a_film_and_starved_one_collapses_and_heats() {
        let p = EngineProfile::ea888_18t_cjsa_is12();
        let mut l = Lubrication::new(&p, 373.0);
        l.p_gallery_pa = 4e5;
        let w = 4000.0 * PI / 30.0;
        let mut tq = 0.0;
        for i in 0..20_000 {
            let phase = (i as f64 * 1e-5 * w / 2.0).sin().max(0.0);
            tq = l.step_bearing(&p, 5, 2e3 + 4.5e4 * phase.powi(8), w, 373.0, 1e-5);
        }
        let b = l.bearings[5];
        assert!(b.film_run_m > 0.4e-6, "film {:e}", b.film_run_m);
        assert!(b.t_k < 400.0 && !b.seized && tq < 2.0, "{b:?} {tq}");

        l.p_gallery_pa = 0.0;
        for _ in 0..300_000 {
            l.step_bearing(&p, 5, 2e4, w, 373.0, 1e-5);
            if l.bearings[5].seized {
                break;
            }
        }
        assert!(l.bearings[5].fill < 0.1);
        assert!(l.bearings[5].seized, "{:?}", l.bearings[5]);
    }
}
