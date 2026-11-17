//! Starter motor and starter battery.
//!
//! Starter: 12 V permanent-magnet DC motor with planetary reduction and a pre-engaged pinion on a roller
//! overrunning clutch (Bosch "DW"/Valeo "D7" class, 1.1 kW, the size fitted to the 1.4–2.0 TSI MQB engines).
//! Armature circuit `V_th − I·(R_th + R_cable + R_m) = k·ω_m`, torque `k·I − T_friction(ω_m)`. The clutch
//! transmits torque only while the pinion drives the ring gear; when the crank runs ahead (engine fires,
//! expansion stroke) the motor freewheels and spins on its own inertia.
//!
//! Battery: lead-acid starter battery with open-circuit voltage OCV(SOC, T) and an effective internal
//! resistance R(T, SOC) (ohmic + short-term polarisation as seen in a 10 s high-rate discharge).
//!
//! Evidence / assumptions (UNVERIFIED unless stated):
//! * pinion 9 teeth: Bosch 1.1 kW 12 V permanent-magnet gear-reduction starters are catalogued with 9 pinion
//!   teeth; ring gear 132 teeth (VAG DMF ring gear, UNVERIFIED) → 14.67:1; planetary 4.6:1 (Bosch PM
//!   reduction starters 3.4..5.6:1).
//! * motor constants chosen so that the starter delivers its 1.1 kW rating with a battery at −20 °C (the
//!   rating condition of starter data sheets) and runs at ~60 A / ~4250 pinion rpm without load at 12.7 V
//!   (no-load test class of PM reduction starters): k = 5.5 mN·m/A, R_m = 14 mΩ (armature + brushes).
//! * battery 68 Ah / 380 A (EN) AGM class (MQB start-stop fitment). EN 50342-1 cold-cranking test: I_CC for
//!   10 s at −18 °C with ≥ 7.5 V at the terminals → effective R(−18 °C) ≤ (12.8 − 7.5)/380 = 14 mΩ; a good
//!   battery reaches ~8 V → 12.5 mΩ. Resistance roughly halves from −18 °C to 25 °C (electrolyte
//!   conductivity; grid resistance does not scale) → 6.9 mΩ at 25 °C.
//! * OCV(SOC) 11.85 V empty .. 12.75 V full (rested, 25 °C), +1.2 mV/K (6 cells × 0.2 mV/K).
//! * main cable 25 mm² Cu ~1.5 m (1.0 mΩ) + engine/body ground strap + solenoid contacts → 2.5 mΩ.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct StarterParams {
    /// EMF / torque constant, V·s/rad = N·m/A.
    pub k: f64,
    /// Armature + brush resistance, Ω.
    pub r_m: f64,
    /// Main cable, ground return and solenoid contacts, Ω.
    pub r_cable: f64,
    /// Overall reduction armature → crank (planetary × ring/pinion).
    pub ratio: f64,
    /// Gear-train efficiency (planetary + pinion/ring mesh).
    pub eta: f64,
    /// Armature + planetary inertia at the armature, kg·m².
    pub j_armature: f64,
    /// Armature friction at the armature: Coulomb (brushes, bearings), N·m, and viscous (windage, gears), N·m·s/rad.
    pub t_coulomb: f64,
    pub b_viscous: f64,
}

/// 1.1 kW PM planetary-reduction starter (see module docs).
pub const STARTER_1_1KW: StarterParams = StarterParams {
    k: 5.5e-3,
    r_m: 0.014,
    r_cable: 0.0025,
    ratio: 4.6 * 132.0 / 9.0,
    eta: 0.85,
    j_armature: 2.5e-4,
    t_coulomb: 0.05,
    b_viscous: 1.4e-4,
};

/// Starter motor state.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Starter {
    /// Armature speed, rad/s.
    pub omega_m: f64,
    /// Armature current, A.
    pub current_a: f64,
    /// Pinion locked to the ring gear through the overrunning clutch (driving).
    pub engaged: bool,
}

impl Starter {
    fn friction(p: &StarterParams, w: f64) -> f64 {
        if w == 0.0 { 0.0 } else { w.signum() * (p.t_coulomb + p.b_viscous * w.abs()) }
    }

    fn current(p: &StarterParams, v_th: f64, r_th: f64, w: f64) -> f64 {
        ((v_th - p.k * w) / (r_th + p.r_cable + p.r_m)).max(0.0)
    }

    /// Drive torque at the crank (N·m) and the inertia the engaged starter adds to the crank (kg·m²) at crank
    /// speed `w_crank`, for a supply with Thevenin source `v_th` / `r_th` (relay closed = `on`).
    pub fn drive(&mut self, p: &StarterParams, on: bool, v_th: f64, r_th: f64, w_crank: f64) -> (f64, f64) {
        self.engaged = false;
        if !on {
            self.current_a = 0.0;
            return (0.0, 0.0);
        }
        let w_lock = p.ratio * w_crank;
        if self.omega_m >= w_lock - 1e-6 {
            let i = Self::current(p, v_th, r_th, w_lock);
            let t_m = p.k * i - Self::friction(p, w_lock);
            if t_m > 0.0 {
                self.engaged = true;
                self.current_a = i;
                return (p.eta * p.ratio * t_m, p.j_armature * p.ratio * p.ratio);
            }
        }
        // Crank ahead of the pinion: the roller clutch freewheels.
        self.current_a = Self::current(p, v_th, r_th, self.omega_m);
        (0.0, 0.0)
    }

    /// Advances the armature after the crank step: locked to the crank while engaged, otherwise spinning on
    /// its own torque (or running down with the relay open).
    pub fn follow(&mut self, p: &StarterParams, w_crank: f64, dt: f64) {
        if self.engaged {
            self.omega_m = p.ratio * w_crank;
            return;
        }
        let w = self.omega_m;
        let t = p.k * self.current_a - Self::friction(p, w);
        let w1 = w + t / p.j_armature * dt;
        self.omega_m = if w1 * w < 0.0 && self.current_a == 0.0 { 0.0 } else { w1 };
    }
}

/// Lead-acid starter battery.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Battery {
    pub capacity_ah: f64,
    /// State of charge 0..1.
    pub soc: f64,
    /// Effective internal resistance at 25 °C and full charge, Ω.
    pub r25_ohm: f64,
}

impl Default for Battery {
    fn default() -> Self {
        Self { capacity_ah: 68.0, soc: 0.9, r25_ohm: 6.9e-3 }
    }
}

impl Battery {
    /// Rested open-circuit voltage, V.
    pub fn ocv(&self, t_c: f64) -> f64 {
        11.85 + 0.90 * self.soc.clamp(0.0, 1.0) + 1.2e-3 * (t_c - 25.0)
    }

    /// Effective internal resistance, Ω (rises in the cold and when discharged).
    pub fn r(&self, t_c: f64) -> f64 {
        let f_t = if t_c < 25.0 { 1.0 + 0.019 * (25.0 - t_c) } else { (1.0 - 0.005 * (t_c - 25.0)).max(0.8) };
        self.r25_ohm * f_t * (1.0 + 0.6 * (1.0 - self.soc.clamp(0.0, 1.0)))
    }

    /// Coulomb counting: `i_a` > 0 discharges.
    pub fn draw(&mut self, i_a: f64, dt: f64) {
        self.soc = (self.soc - i_a * dt / (3600.0 * self.capacity_ah)).clamp(0.0, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Datasheet-class characteristic: no-load ~60 A / ~4250 pinion rpm, stall several hundred amps,
    /// 1.1 kW peak with the battery at −20 °C, and a breakaway torque at the crank far above the
    /// compression torque of a 1.8 l four-cylinder (≈ 70..90 N·m peak, see the bench cranking test).
    #[test]
    fn characteristic_matches_a_1_1_kw_reduction_starter() {
        let p = STARTER_1_1KW;
        let b = Battery { soc: 1.0, ..Default::default() };
        let (v, r) = (b.ocv(25.0), b.r(25.0));
        // free running
        let mut s = Starter::default();
        for _ in 0..20_000 {
            s.current_a = Starter::current(&p, v, r, s.omega_m);
            s.follow(&p, 0.0, 1e-4);
        }
        let pinion_rpm = s.omega_m * 30.0 / std::f64::consts::PI / 4.6;
        assert!((3800.0..4700.0).contains(&pinion_rpm), "no-load pinion {pinion_rpm}");
        assert!((40.0..90.0).contains(&s.current_a), "no-load current {}", s.current_a);
        // stall (breakaway)
        let mut s = Starter::default();
        let (t, j) = s.drive(&p, true, v, r, 0.0);
        assert!((400.0..650.0).contains(&s.current_a), "stall current {}", s.current_a);
        assert!(t > 140.0 && j > 0.5, "stall torque at the crank {t}, reflected inertia {j}");
        assert!(v - r * s.current_a > 8.5, "battery at inrush {}", v - r * s.current_a);
        // rated power with the battery at -20 °C
        let (vc, rc) = (b.ocv(-20.0), b.r(-20.0));
        let pmax = (0..400)
            .map(|n| {
                let w = n as f64 * 5.0;
                let i = Starter::current(&p, vc, rc, w);
                (p.k * i - Starter::friction(&p, w)) * w
            })
            .fold(0.0, f64::max);
        assert!((950.0..1300.0).contains(&pmax), "cold max power {pmax}");
        // EN 50342-1 cold cranking: >= 7.5 V after 10 s at I_CC = 380 A, -18 °C
        assert!(b.ocv(-18.0) - 380.0 * b.r(-18.0) >= 7.5);
    }
}
