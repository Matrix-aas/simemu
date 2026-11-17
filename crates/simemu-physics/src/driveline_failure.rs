//! Strength and fatigue of driveline parts. Damage is accumulated with Palmgren–Miner's rule from
//! the stresses the simulated torques produce; nothing is scripted.
//!
//! - Gear tooth root bending, simplified ISO 6336-3: σ_F = F_t/(b·m_n)·Y_FS·Y_β with
//!   Y_FS ≈ 3.45 + 12.5/z (combined form/stress-correction factor, profile shift 0) and Y_β = 0.85.
//!   Case-carburised 16MnCr5/20MnCr5: σ_Flim = 460 MPa, Y_ST = 2 → σ_FE = 920 MPa; life factor
//!   Y_NT = 2.5 at N ≤ 10³ falling log-linearly to 1.0 at 3·10⁶ (ISO 6336-3, Eh/IF curve);
//!   static fracture above 2.5·σ_FE.
//! - Flank contact, ISO 6336-2: σ_H = Z_H·Z_E·Z_εβ·√(F_t/(b·d₁)·(u+1)/u), Z_H = 2.4,
//!   Z_E = 189.8 √MPa (steel/steel), Z_εβ = 0.8; σ_Hlim = 1500 MPa, Z_NT = 1.6 at ≤ 10⁵ → 1.0 at
//!   5·10⁷. Pitting raises mesh losses and root stress (pits are notches).
//! - Shafts / driveshafts / CV joints in torsion: τ = 16·T·d_o/(π(d_o⁴ − d_i⁴)); Basquin line from
//!   (10³ cycles, 0.9·τ_u) to (10⁶, 0.45·τ_u); fracture above τ_u.
//!
//! Geometry and material values are UNVERIFIED engineering estimates (no supplier drawings).

use serde::Serialize;
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct GearGeometry {
    /// Normal module, m.
    pub module_m: f64,
    pub face_width_m: f64,
    /// Input-shaft → output-shaft centre distance, m.
    pub center_distance_m: f64,
}

impl GearGeometry {
    pub fn pinion_radius(&self, ratio: f64) -> f64 {
        self.center_distance_m / (1.0 + ratio.abs())
    }
    pub fn pinion_teeth(&self, ratio: f64) -> f64 {
        (2.0 * self.pinion_radius(ratio) / self.module_m).max(8.0)
    }
}

/// Tooth-root bending stress of the pinion at input torque `t_nm`, MPa.
pub fn root_stress_mpa(t_nm: f64, g: &GearGeometry, ratio: f64) -> f64 {
    let r = ratio.abs().max(1e-6);
    let ft = t_nm.abs() / g.pinion_radius(r).max(1e-6);
    let y_fs = 3.45 + 12.5 / g.pinion_teeth(r);
    ft / (g.face_width_m.max(1e-6) * g.module_m.max(1e-6)) * y_fs * 0.85 / 1e6
}

/// Flank contact stress, MPa.
pub fn contact_stress_mpa(t_nm: f64, g: &GearGeometry, ratio: f64) -> f64 {
    let ra = ratio.abs().max(1e-6);
    let r1 = g.pinion_radius(ra).max(1e-6);
    let u = ra.max(1.0 / ra);
    let ft = t_nm.abs() / r1;
    // N/m² → MPa inside the root: Z_E in √MPa, F/(b·d) in N/mm².
    2.4 * 189.8 * 0.8 * (ft / (g.face_width_m.max(1e-6) * 1e3 * 2.0 * r1 * 1e3) * (u + 1.0) / u).sqrt()
}

const SIGMA_FE: f64 = 920.0;

/// Cycles to tooth-root fracture at stress σ (MPa); 0 = static fracture, ∞ = endurance.
pub fn bending_cycles(sigma: f64) -> f64 {
    let y = sigma / SIGMA_FE;
    if y >= 2.5 {
        0.0
    } else if y <= 1.0 {
        f64::INFINITY
    } else {
        10f64.powf(3.0 + (2.5 - y) / 1.5 * (3e6f64.log10() - 3.0))
    }
}

/// Cycles to pitting at contact stress σ_H (MPa).
pub fn contact_cycles(sigma_h: f64) -> f64 {
    let z = sigma_h / 1500.0;
    if z >= 1.6 {
        1e5 * (1.6 / z).powi(20)
    } else if z <= 1.0 {
        f64::INFINITY
    } else {
        10f64.powf(5.0 + (1.6 - z) / 0.6 * (5e7f64.log10() - 5.0))
    }
}

/// Maximum torsional shear stress of a (hollow) shaft, MPa.
pub fn shaft_shear_mpa(t_nm: f64, d_o: f64, d_i: f64) -> f64 {
    16.0 * t_nm.abs() * d_o / (PI * (d_o.powi(4) - d_i.powi(4))) / 1e6
}

/// Cycles to torsional fatigue failure (Basquin), 0 = static fracture.
pub fn torsion_cycles(tau: f64, tau_u: f64) -> f64 {
    if tau >= tau_u {
        0.0
    } else if tau <= 0.45 * tau_u {
        f64::INFINITY
    } else {
        let k = 3.0 / 2f64.log10();
        1e3 * (0.9 * tau_u / tau).powf(k)
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct ShaftHw {
    pub d_outer_m: f64,
    pub d_inner_m: f64,
    pub tau_ultimate_mpa: f64,
}

/// Miner damage state of a part loaded once per revolution.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct Damage {
    pub fatigue: f64,
    pub broken: bool,
}

impl Damage {
    /// `cycles` load cycles at a stress whose life is `n_f`.
    pub fn add(&mut self, cycles: f64, n_f: f64) {
        if self.broken {
            return;
        }
        if n_f <= 0.0 {
            self.broken = true;
            return;
        }
        if n_f.is_finite() {
            self.fatigue += cycles / n_f;
            if self.fatigue >= 1.0 {
                self.broken = true;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_gear_stress_and_life_are_plausible() {
        let g = GearGeometry { module_m: 2.25e-3, face_width_m: 22e-3, center_distance_m: 0.068 };
        let s250 = root_stress_mpa(250.0, &g, 3.765);
        let s400 = root_stress_mpa(400.0, &g, 3.765);
        assert!((800.0..1600.0).contains(&s250), "σ_F at 250 Nm {s250}");
        let (n250, n400) = (bending_cycles(s250), bending_cycles(s400));
        assert!(n250 > 1e5 && n400 < n250 / 10.0, "{n250} {n400}");
        assert_eq!(bending_cycles(3000.0), 0.0);
        let sh = contact_stress_mpa(250.0, &g, 3.765);
        assert!((900.0..2500.0).contains(&sh), "σ_H {sh}");
    }
}
