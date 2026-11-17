//! Slider-crank kinematics, connecting-rod structural limits, dual-mass flywheel spring and the
//! crank (60-2) / cam (2 × 8 edges) sensor wheels.
//!
//! Kinematics: offset slider crank. With crank radius r, rod length l, pin offset e and local crank
//! angle φ from TDC: s(φ) = r·cosφ + √(l² − (r·sinφ − e)²), piston travel from TDC x = s_TDC − s.
//! Crank torque from a piston force F (positive = pushing the piston away from the head) follows from
//! virtual work: T = F·dx/dφ. Reciprocating mass m adds J(φ) = m·(dx/dφ)² to the crank inertia and the
//! velocity torque −m·x'·x''·ω² (Lagrange). Rod angle β: sinβ = (r·sinφ − e)/l.

use crate::materials::ROD_36MNVS4;
use crate::profiles::EngineProfile;
use crate::sensors::{EdgeSource, SensorEdge};
use serde::Serialize;
use std::f64::consts::PI;

/// Firing TDC of cylinders 1..4 in the 720° cycle (firing order 1-3-4-2).
pub const FIRING_OFFSET_DEG: [f64; 4] = [0.0, 540.0, 180.0, 360.0];
/// Falling (active) edge of the first tooth after the gap, °CRK relative to TDC0 (FR NC_OFS_TDC0_REF_CRK = 78).
pub const CRANK_TOOTH0_DEG: f64 = -78.0;
/// Cam Hall edges, °CRK after TDC0 at phaser rest (FR baseline, `docs/SIMOS18_GPTA_IO_RE.md` §5.1) and the
/// level after each edge (UNVERIFIED polarity: high between the listed pairs).
pub const CAM_EDGES_DEG: [[(f64, bool); 8]; 2] = [
    [(2.0, false), (46.0, true), (80.0, false), (226.0, true), (260.0, false), (406.0, true), (542.0, false), (586.0, true)],
    [(33.0, false), (179.0, true), (213.0, false), (359.0, true), (495.0, false), (539.0, true), (675.0, false), (719.0, true)],
];

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Kinematics {
    pub r: f64,
    pub l: f64,
    pub e: f64,
    /// Piston area, m².
    pub area: f64,
    pub v_clear: f64,
    pub v_disp: f64,
    pub bore: f64,
    pub stroke: f64,
    s_tdc: f64,
    /// Reciprocating (piston group + rod small end) and rotating (rod big end) masses, kg.
    pub m_rec: f64,
    pub m_rot: f64,
}

/// Piston position and derivatives w.r.t. crank angle (rad) plus the rod angle.
#[derive(Debug, Clone, Copy, Default)]
pub struct Slider {
    pub x: f64,
    pub dx: f64,
    pub ddx: f64,
    pub sin_b: f64,
    pub cos_b: f64,
    pub sin_phi: f64,
    pub cos_phi: f64,
}

impl Kinematics {
    pub fn new(p: &EngineProfile) -> Self {
        let m = &p.mech;
        let r = 0.5 * p.stroke_m;
        let area = PI / 4.0 * p.bore_m * p.bore_m;
        let v_disp = area * p.stroke_m;
        Self {
            r,
            l: m.rod_length_m,
            e: m.pin_offset_m,
            area,
            v_clear: v_disp / (p.compression_ratio - 1.0),
            v_disp,
            bore: p.bore_m,
            stroke: p.stroke_m,
            s_tdc: ((m.rod_length_m + r).powi(2) - m.pin_offset_m.powi(2)).sqrt(),
            m_rec: m.piston_mass_kg + m.rod_small_end_share * m.rod_mass_kg,
            m_rot: (1.0 - m.rod_small_end_share) * m.rod_mass_kg,
        }
    }

    /// `phi` local crank angle (rad, 0 = TDC); `short` = permanent rod shortening (bent rod), m.
    #[inline]
    pub fn eval(&self, phi: f64, short: f64) -> Slider {
        let (sn, cs) = phi.sin_cos();
        let l = self.l - short;
        let q = self.r * sn - self.e;
        let qd = self.r * cs;
        let qdd = -self.r * sn;
        let w = (l * l - q * q).max(1e-6).sqrt();
        let s = self.r * cs + w;
        let sd = -self.r * sn - q * qd / w;
        let sdd = -self.r * cs - (qd * qd + q * qdd) / w - q * q * qd * qd / (w * w * w);
        Slider { x: self.s_tdc - s, dx: -sd, ddx: -sdd, sin_b: q / l, cos_b: w / l, sin_phi: sn, cos_phi: cs }
    }

    #[inline]
    pub fn volume(&self, s: &Slider) -> f64 {
        self.v_clear + self.area * s.x
    }
}

/// Compressive load at which the rod shank yields/buckles (Johnson parabola; Euler beyond the
/// transition slenderness), reduced for an existing bend by the secant-type factor 1/(1 + e·c/k²).
/// Effective column length 0.75·rod length between the eyes (UNVERIFIED).
pub fn rod_critical_load(p: &EngineProfile, bend_m: f64) -> f64 {
    let m = &p.mech;
    let e_mod = ROD_36MNVS4.youngs_pa;
    let sy = ROD_36MNVS4.yield_pa(120.0);
    let k = m.rod_gyration_radius_m;
    let slender2 = (0.75 * m.rod_length_m / k).powi(2);
    let euler = PI * PI * e_mod / slender2;
    let s_cr = if sy < 0.5 * euler { sy * (1.0 - sy * slender2 / (4.0 * PI * PI * e_mod)) } else { euler };
    s_cr * m.rod_shank_area_m2 / (1.0 + bend_m * m.rod_fibre_m / (k * k))
}

/// Dual-mass-flywheel arc-spring torque (primary → secondary), with hard end stops.
pub fn dmf_torque(p: &EngineProfile, twist: f64, rel_omega: f64) -> f64 {
    let m = &p.mech;
    let over = (twist.abs() - m.dmf_travel_rad).max(0.0) * twist.signum();
    m.dmf_rate_nm_rad * twist + 50.0 * m.dmf_rate_nm_rad * over + m.dmf_damping_nms * rel_omega
}

/// Emits crank-tooth edges crossed between unwrapped crank angles `th0` → `th1` (deg).
pub fn crank_edges(th0: f64, th1: f64, t0: f64, dt: f64, out: &mut Vec<SensorEdge>) {
    if !th0.is_finite() || !th1.is_finite() || !t0.is_finite() || !(dt >= 0.0 && dt.is_finite()) || th1 == th0 {
        return;
    }
    let idx = |th: f64| ((th - CRANK_TOOTH0_DEG) / 6.0).floor();
    let (f0, f1) = (idx(th0), idx(th1));
    if (f1 - f0).abs() > 4096.0 {
        return;
    }
    let (n0, n1) = (f0 as i64, f1 as i64);
    if n0 == n1 {
        return;
    }
    let mut emit = |j: i64, forward: bool| {
        let tooth = j.rem_euclid(60);
        if tooth < 58 {
            let a = CRANK_TOOTH0_DEG + 6.0 * j as f64;
            out.push(SensorEdge { t_s: t0 + dt * ((a - th0) / (th1 - th0)).clamp(0.0, 1.0), source: EdgeSource::CrankTooth { tooth: tooth as u8, forward } });
        }
    };
    if n1 > n0 {
        for j in (n0 + 1)..=n1 {
            emit(j, true);
        }
    } else {
        for j in ((n1 + 1)..=n0).rev() {
            emit(j, false);
        }
    }
}

/// Emits cam Hall edges for cam `k` crossed between cam-referred unwrapped angles `psi0` → `psi1`
/// (crank deg; ψ = crank angle minus the phaser shift).
pub fn cam_edges(k: usize, psi0: f64, psi1: f64, t0: f64, dt: f64, out: &mut Vec<SensorEdge>) {
    if k >= CAM_EDGES_DEG.len() || !psi0.is_finite() || !psi1.is_finite() || !t0.is_finite() || !(dt >= 0.0 && dt.is_finite()) || psi1 == psi0 {
        return;
    }
    if (psi1 - psi0).abs() > 720.0 * 64.0 {
        return;
    }
    let start = out.len();
    for &(e, level) in &CAM_EDGES_DEG[k] {
        let c0 = ((psi0 - e) / 720.0).floor() as i64;
        let c1 = ((psi1 - e) / 720.0).floor() as i64;
        if c1 > c0 {
            for c in (c0 + 1)..=c1 {
                let a = e + 720.0 * c as f64;
                out.push(SensorEdge {
                    t_s: t0 + dt * ((a - psi0) / (psi1 - psi0)).clamp(0.0, 1.0),
                    source: EdgeSource::Cam { cam: k as u8, level },
                });
            }
        } else if c1 < c0 {
            for c in ((c1 + 1)..=c0).rev() {
                let a = e + 720.0 * c as f64;
                out.push(SensorEdge {
                    t_s: t0 + dt * ((a - psi0) / (psi1 - psi0)).clamp(0.0, 1.0),
                    source: EdgeSource::Cam { cam: k as u8, level: !level },
                });
            }
        }
    }
    if out.len() - start > 1 {
        out[start..].sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slider_crank_matches_geometry_and_numeric_derivatives() {
        let p = EngineProfile::ea888_18t_cjsa_is12();
        let k = Kinematics::new(&p);
        let tdc = k.eval(0.0, 0.0);
        let bdc = k.eval(PI, 0.0);
        assert!(tdc.x.abs() < 1e-5, "x(TDC) {}", tdc.x);
        assert!((bdc.x - p.stroke_m).abs() < 1e-4, "stroke {}", bdc.x);
        let eps = (k.v_clear + k.v_disp) / k.v_clear;
        assert!((eps - p.compression_ratio).abs() < 1e-9);
        for &phi in &[0.3, 1.2, 2.5, 4.0] {
            let h = 1e-5;
            let a = k.eval(phi - h, 0.0);
            let b = k.eval(phi + h, 0.0);
            let s = k.eval(phi, 0.0);
            assert!(((b.x - a.x) / (2.0 * h) - s.dx).abs() < 1e-6);
            assert!(((b.dx - a.dx) / (2.0 * h) - s.ddx).abs() < 1e-5);
        }
    }

    #[test]
    fn crank_wheel_has_58_edges_per_rev_and_cam_8_per_cycle() {
        let mut v = Vec::new();
        let mut th = 0.0;
        while th < 720.0 {
            crank_edges(th, th + 0.7, 0.0, 1.0, &mut v);
            cam_edges(0, th, th + 0.7, 0.0, 1.0, &mut v);
            th += 0.7;
        }
        let teeth = v.iter().filter(|e| matches!(e.source, EdgeSource::CrankTooth { .. })).count();
        let cams = v.iter().filter(|e| matches!(e.source, EdgeSource::Cam { .. })).count();
        assert_eq!(teeth, 116);
        assert_eq!(cams, 8);
        let mut r = Vec::new();
        crank_edges(10.0, 3.0, 0.0, 1.0, &mut r);
        assert!(r.iter().all(|e| matches!(e.source, EdgeSource::CrankTooth { forward: false, .. })));
    }

    #[test]
    fn stock_rod_buckles_near_twice_normal_peak_force() {
        let p = EngineProfile::ea888_18t_cjsa_is12();
        let f = rod_critical_load(&p, 0.0);
        let p_equiv_bar = f / Kinematics::new(&p).area / 1e5;
        assert!((160.0..230.0).contains(&p_equiv_bar), "{p_equiv_bar}");
        assert!(rod_critical_load(&p, 1e-3) < 0.85 * f);
    }

    #[test]
    fn edge_generators_handle_multi_cycle_reverse_and_non_finite_inputs() {
        let mut v = Vec::new();
        // Multi-cycle jump (2 full 720° cam cycles = 16 edges) ordered chronologically.
        cam_edges(0, 0.0, 1440.0, 1.0, 0.2, &mut v);
        assert_eq!(v.len(), 16);
        assert!(v.windows(2).all(|w| w[0].t_s <= w[1].t_s));

        // Reverse rotation over one cycle emits 8 inverted edges in non-decreasing timestamp order.
        let mut rev = Vec::new();
        cam_edges(1, 720.0, 0.0, 2.0, 0.1, &mut rev);
        assert_eq!(rev.len(), 8);
        assert!(rev.windows(2).all(|w| w[0].t_s <= w[1].t_s));

        // Out-of-bounds cam index, NaN/Inf, negative dt, or astronomical jumps must not panic or OOM.
        let mut bad = Vec::new();
        cam_edges(2, 0.0, 720.0, 0.0, 0.1, &mut bad);
        cam_edges(0, f64::NAN, 720.0, 0.0, 0.1, &mut bad);
        cam_edges(0, 0.0, 1e15, 0.0, 0.1, &mut bad);
        crank_edges(f64::NAN, 100.0, 0.0, 0.1, &mut bad);
        crank_edges(0.0, 100.0, 0.0, -0.1, &mut bad);
        crank_edges(0.0, 1e15, 0.0, 0.1, &mut bad);
        assert!(bad.is_empty());
    }
}
