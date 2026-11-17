//! Test-cell equipment interlock — bench hardware, NOT engine-ECU logic.
//!
//! Opt-in and OFF by default, so the physics is free to destroy the engine. When armed and a
//! limit is crossed it does only what a real dyno-cell interlock does: latches, lifts the
//! throttle pedal (a driver input) and aborts the sweep program. It never touches firmware
//! outputs or physics state. The latch is released only by an explicit reset.

use serde::{Deserialize, Serialize};
use simemu_physics::PlantTelemetry;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyLimits {
    pub enabled: bool,
    pub rpm_max: Option<f64>,
    pub egt_max_c: Option<f64>,
    pub coolant_max_c: Option<f64>,
    pub oil_max_c: Option<f64>,
    pub oil_min_bar: Option<f64>,
    /// Oil pressure is only checked above this engine speed (no pressure at rest / cranking).
    pub oil_check_above_rpm: f64,
    /// Highest per-cylinder knock integral (Livengood–Wu, ≥ 1 = auto-ignition).
    pub knock_max: Option<f64>,
}

impl Default for SafetyLimits {
    fn default() -> Self {
        Self {
            enabled: false,
            rpm_max: Some(7000.0),
            egt_max_c: Some(980.0),
            coolant_max_c: Some(115.0),
            oil_max_c: Some(140.0),
            oil_min_bar: Some(1.0),
            oil_check_above_rpm: 1500.0,
            knock_max: Some(1.0),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Trip {
    pub t_s: f64,
    pub channel: &'static str,
    pub value: f64,
    pub limit: f64,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct BenchSafety {
    pub limits: SafetyLimits,
    pub trip: Option<Trip>,
    pub trips_total: u64,
}

fn first_violation(l: &SafetyLimits, t: &PlantTelemetry) -> Option<Trip> {
    let knock = t.knock_integral.iter().copied().fold(f64::NAN, f64::max);
    let over = |channel, value: f64, lim: Option<f64>| lim.filter(|&m| value > m).map(|limit| (channel, value, limit));
    over("plant.rpm", t.rpm, l.rpm_max)
        .or_else(|| over("plant.egt_c", t.egt_c, l.egt_max_c))
        .or_else(|| over("plant.coolant_c", t.coolant_c, l.coolant_max_c))
        .or_else(|| over("plant.oil_c", t.oil_c, l.oil_max_c))
        .or_else(|| over("plant.knock_integral", knock, l.knock_max))
        .or_else(|| {
            l.oil_min_bar
                .filter(|&m| t.rpm > l.oil_check_above_rpm && t.oil_bar < m)
                .map(|limit| ("plant.oil_bar", t.oil_bar, limit))
        })
        .map(|(channel, value, limit)| Trip { t_s: t.sim_time_s, channel, value, limit })
}

impl BenchSafety {
    /// Returns true while the interlock is latched (the caller then lifts the pedal / stops the sweep).
    pub fn check(&mut self, t: &PlantTelemetry) -> bool {
        if !self.limits.enabled {
            return false;
        }
        if self.trip.is_none() {
            self.trip = first_violation(&self.limits, t);
            self.trips_total += u64::from(self.trip.is_some());
        }
        self.trip.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trips_only_when_armed_and_latches() {
        let mut s = BenchSafety::default();
        let mut t = PlantTelemetry { rpm: 7200.0, oil_bar: 3.0, ..Default::default() };
        assert!(!s.check(&t), "off by default");
        s.limits.enabled = true;
        assert!(s.check(&t));
        assert_eq!(s.trip.as_ref().unwrap().channel, "plant.rpm");
        t.rpm = 3000.0;
        assert!(s.check(&t), "latched until reset");
        s.trip = None;
        t.oil_bar = 0.3;
        assert!(s.check(&t) && s.trip.as_ref().unwrap().channel == "plant.oil_bar");
    }
}
