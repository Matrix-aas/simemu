//! Rotary coolant valve of the EA888 Gen3 thermal-management module (actuator N493 with its SENT position sensor).
//!
//! Hardware chain (angle θ of rotary slide 1 from its lower mechanical stop):
//! * DC motor behind ATIC111 bridge 1 of the ECU (mean armature voltage = signed duty × supply, + = towards larger θ),
//!   worm gear to the valve: torque k·(V − k·ω)/R at the output, Coulomb friction of the slide seals; the worm is
//!   self-locking, so the de-energised valve stays where it is. Mechanical stops at 0° and [`RotaryValveHw::travel_deg`]
//!   — FR ENTE 34.22 (p.9082): the firmware normalises its sensor between the lower (0°) and upper (263°) stop and
//!   learns both by driving into them (`c_rvcpwm_thd_bol_ad` 30 % for `c_t_rvc_bol` 1 s, stall = gradient below
//!   `c_dig_rvc_bol_grd` 15 counts).
//! * Position sensor: contactless angle sensor with a SAE J2716 output, 12-bit fast channel (D1..D3, the only part the
//!   firmware reads, INFR `0x800A5D9C`), linear between [`RotaryValveHw::sensor_counts`] at the two stops — the
//!   firmware's own default adaptation values `c_dig_rvc_bol` 145 / `c_dig_rvc_tol` 3948 (accepted 90..200 /
//!   3890..4000), i.e. what a nominal sensor delivers.
//! * Coolant routing: only the radiator branch is modelled — open fraction falling linearly from 1 at
//!   [`RotaryValveHw::radiator_open_deg`] to 0 at [`RotaryValveHw::radiator_closed_deg`] (low angles = maximum cooling:
//!   the firmware's limp-home set point `c_rvc_sp_lih` is 4°). The wax fail-safe thermostat stays in `thermal`.
//!
//! Every number except the stops and the sensor counts is an UNVERIFIED estimate (no supplier data): the valve moves
//! end to end in ≈ 1 s at 13.5 V and full duty and holds against its seals with ≈ 15 % duty.

use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct RotaryValveHw {
    /// Armature resistance, Ω; torque constant at the valve shaft, N·m/A; reflected inertia, kg·m²; Coulomb friction, N·m.
    pub r_ohm: f64,
    pub k_nm_a: f64,
    pub inertia_kgm2: f64,
    pub friction_nm: f64,
    /// Angle of the upper mechanical stop, deg (lower stop = 0°).
    pub travel_deg: f64,
    /// SENT fast-channel value at the lower / upper stop.
    pub sensor_counts: [f64; 2],
    /// Radiator branch fully open at / closed from this angle, deg.
    pub radiator_open_deg: f64,
    pub radiator_closed_deg: f64,
    /// Angle of a new / unpowered car (radiator branch closed; the firmware parks and adapts the valve itself).
    pub rest_deg: f64,
}

impl RotaryValveHw {
    pub fn ea888_gen3() -> Self {
        Self {
            r_ohm: 3.0,
            k_nm_a: 2.5,
            inertia_kgm2: 2.0e-3,
            friction_nm: 1.5,
            travel_deg: 263.0,
            sensor_counts: [145.0, 3948.0],
            radiator_open_deg: 0.0,
            radiator_closed_deg: 85.0,
            rest_deg: 160.0,
        }
    }
}

impl Default for RotaryValveHw {
    fn default() -> Self {
        Self::ea888_gen3()
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct RotaryValve {
    pub theta_deg: f64,
    /// Shaft speed, deg/s.
    pub omega_deg_s: f64,
}

impl RotaryValve {
    pub fn at_rest(hw: &RotaryValveHw) -> Self {
        Self { theta_deg: hw.rest_deg, omega_deg_s: 0.0 }
    }

    /// Advances by `dt`; `v_motor` = mean armature voltage (+ = towards larger θ), `None` = bridge disabled.
    pub fn step(&mut self, hw: &RotaryValveHw, v_motor: Option<f64>, dt: f64) {
        let w = self.omega_deg_s.to_radians();
        let (drive, damp) = match v_motor {
            Some(v) => (hw.k_nm_a * v / hw.r_ohm, hw.k_nm_a * hw.k_nm_a / hw.r_ohm),
            None => (0.0, 0.0),
        };
        // implicit in the back-EMF term (electromechanical time constant ≈ 1 ms ≈ dt)
        let free = (w + dt / hw.inertia_kgm2 * drive) / (1.0 + dt * damp / hw.inertia_kgm2);
        let stick = hw.friction_nm * dt / hw.inertia_kgm2;
        let w_new = if free.abs() <= stick { 0.0 } else { free - stick * free.signum() };
        let mut th = self.theta_deg + w_new.to_degrees() * dt;
        let mut w_new = w_new.to_degrees();
        if th <= 0.0 || th >= hw.travel_deg {
            th = th.clamp(0.0, hw.travel_deg);
            w_new = 0.0;
        }
        self.theta_deg = th;
        self.omega_deg_s = w_new;
    }

    /// Radiator branch open fraction.
    pub fn radiator_fraction(&self, hw: &RotaryValveHw) -> f64 {
        ((hw.radiator_closed_deg - self.theta_deg) / (hw.radiator_closed_deg - hw.radiator_open_deg)).clamp(0.0, 1.0)
    }

    /// 12-bit SENT fast-channel value of the position sensor.
    pub fn sensor_value(&self, hw: &RotaryValveHw) -> u16 {
        let [a, b] = hw.sensor_counts;
        (a + (b - a) * self.theta_deg / hw.travel_deg).round().clamp(0.0, 4095.0) as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valve_travels_holds_and_stops_at_the_end() {
        let hw = RotaryValveHw::ea888_gen3();
        let mut v = RotaryValve { theta_deg: 0.0, omega_deg_s: 0.0 };
        let mut t_end = None;
        for i in 0..3000 {
            v.step(&hw, Some(13.5), 1e-3);
            if v.theta_deg >= hw.travel_deg && t_end.is_none() {
                t_end = Some(i as f64 * 1e-3);
            }
        }
        let t = t_end.expect("reaches the upper stop");
        assert!((0.5..2.0).contains(&t), "end to end in {t} s");
        assert_eq!(v.sensor_value(&hw), 3948);
        // self-locking: no drift without current, and 10 % duty does not break the seals loose
        for _ in 0..1000 {
            v.step(&hw, None, 1e-3);
        }
        let th = v.theta_deg;
        for _ in 0..1000 {
            v.step(&hw, Some(-1.35), 1e-3);
        }
        assert_eq!(v.theta_deg, th);
        assert_eq!(RotaryValve { theta_deg: 0.0, ..v }.radiator_fraction(&hw), 1.0);
        assert_eq!(RotaryValve { theta_deg: 160.0, ..v }.radiator_fraction(&hw), 0.0);
    }
}
