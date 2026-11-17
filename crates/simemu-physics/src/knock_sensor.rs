//! Engine-block vibration at the knock-sensor seats and the piezo knock sensors.
//!
//! Source → structure → sensor, all hardware:
//! * **Knock.** End-gas autoignition rings the combustion chamber's acoustic modes. For a cylindrical chamber
//!   the mode frequencies are f_mn = α_mn·c/(π·B) (Draper, "Pressure waves accompanying detonation in the
//!   internal combustion engine", J. Aeronaut. Sci. 5, 1938; α₁₀ = 1.841, α₂₀ = 3.054, α₀₁ = 3.832), c = √(γ·R·T)
//!   of the burned gas — ≈ 6.5–7 kHz for the first circumferential mode of the 82.5 mm EA888 bore. Each mode
//!   decays with the plant's knock envelope time constant (`cylinder::KNOCK_DECAY_S`) and reaches the sensor
//!   seat through the block with a gain in g per bar of pressure oscillation that falls with the distance
//!   between the cylinder and the sensor.
//! * **Mechanical noise** (what a knock detector must not call knock): valve seating at the end of the closing
//!   ramp (seating velocity ∝ crank speed), piston slap after firing TDC (secondary piston motion, ∝ √ peak
//!   pressure and speed), DI needle closing (∝ √ΔP), and random impacts of the rotating machinery (chain,
//!   bearings, gears; a Poisson process whose rate is ∝ speed). Each impact rings the block's structural modes
//!   (impulse response e^(−t/τ)·sin(ωt) per mode).
//! * **Sensor.** KS4-class piezo accelerometer: output ∝ acceleration, 26 ± 8 mV/g at 5 kHz, main resonance
//!   30 kHz, 3–25 kHz range, 1150 pF (Bosch Motorsport "Knock Sensor KS4-P" data sheet). The resonance is applied
//!   per mode as the second-order gain/phase at the mode frequency.
//!
//! Representation: every excitation is a damped sinusoid ("atom") {t₀, ω, 1/τ, φ, volts at each sensor}; the
//! sensor voltage at any time is the sum of the live atoms. Nothing is synthesised at a fixed rate: the ECU's
//! converter asks for the samples it actually takes ([`BlockVibration::sample_into`], phasor recursion per atom).
//! The atom store is a fixed ring → no allocation while running.
//!
//! All magnitudes are `UNVERIFIED` knobs ([`KnockSensorHw`]); their only anchors are the sensor data sheet and
//! the requirement that the ECU's own calibrated detection works (noise level, threshold factors), see the tests.

use serde::Serialize;
use std::f64::consts::PI;

/// Draper's roots α_mn of J'_m for the knock modes (1,0), (2,0), (0,1), (3,0), (1,1) and their relative excitation.
/// UNVERIFIED weights (the first circumferential mode dominates for a near-wall end-gas pocket).
const KNOCK_MODES: [(f64, f64); 5] = [(1.841, 1.0), (3.054, 0.5), (3.832, 0.35), (4.201, 0.3), (5.331, 0.25)];
/// Ratio of specific heats of the burned gas at the knock event.
const GAMMA_BURNED: f64 = 1.28;
const R_BURNED: f64 = 288.0;
/// Atoms are dropped after this many decay time constants (e^-6 = 0.25 %).
const LIFE_TAUS: f64 = 6.0;
const RING: usize = 512;

/// Knock-sensor hardware of an engine (profile data).
#[derive(Debug, Clone, Serialize)]
pub struct KnockSensorHw {
    /// Number of sensors fitted (1 or 2) and their positions along the crank axis in cylinder pitches
    /// (0 = cylinder 1, 3 = cylinder 4).
    pub sensors: usize,
    pub sensor_pos: [f64; 2],
    /// Sensitivity in the flat band, V/g, main resonance, Hz, and its quality factor.
    pub v_per_g: f64,
    pub resonance_hz: f64,
    pub resonance_q: f64,
    /// Structure-borne transfer at the source position, g per bar of first-mode pressure oscillation.
    pub knock_g_per_bar: f64,
    /// Transfer attenuation per cylinder pitch of distance between source and sensor.
    pub attenuation_per_pitch: f64,
    /// Decay time constant of the chamber modes, s.
    pub knock_decay_s: f64,
    /// Structural modes of the block rung by mechanical impacts: (Hz, Q, relative weight).
    pub block_modes: [(f64, f64, f64); 6],
    /// Valve seating: closing-ramp height expressed as seating velocity per crank speed, m/s per rad/s, and the
    /// block response per m/s of seating velocity, g.
    pub seat_ramp_m: f64,
    pub seat_g_per_m_s: f64,
    /// Piston slap after firing TDC (the piston crosses to the major-thrust side once the gas force has turned the rod):
    /// g at 3000 rpm and 60 bar, angle after TDC, °CA. The cyclic events dominate the background inside a knock window,
    /// so the window energy repeats from cycle to cycle and the random impacts only add a small spread.
    pub slap_g: f64,
    pub slap_deg: f64,
    /// DI needle closing, g at 10 MPa ΔP.
    pub injector_g: f64,
    /// Rotating-machinery impacts: mean count per crank revolution and their mean peak g (uniform 0.5…1.5 × mean:
    /// many comparable small impacts → a near-Gaussian background, as a knock detector sees it).
    pub shots_per_rev: f64,
    pub shot_g: f64,
}

impl KnockSensorHw {
    /// EA888 Gen3: one KS4-class sensor on the intake side of the block between cylinders 2 and 3 (SC800S50 reads one
    /// knock input, FADC ch 1; the position is UNVERIFIED). Block modes, transfer and noise magnitudes UNVERIFIED,
    /// sized against the firmware's own calibration (docs/SIMOS18_GPTA_IO_RE.md §12.7): at 3000 rpm full load the
    /// in-band background at the sensor is ≈ 1.5–2 g (≈ 40–50 mV), which the firmware's gain adaptation brings to its
    /// noise target 0.5–1 V and keeps above its sensor-diagnosis floor (`ip_knks_err_bas` 0.12–0.31 V), and ≈ 1 bar of
    /// first-mode knock in cylinder 2/3 reaches about twice the background amplitude (light knock = detection limit).
    pub fn ea888_gen3() -> Self {
        Self {
            sensors: 1,
            sensor_pos: [1.5, 1.5],
            v_per_g: 0.026,
            resonance_hz: 30_000.0,
            resonance_q: 12.0,
            knock_g_per_bar: 4.0,
            attenuation_per_pitch: 0.75,
            knock_decay_s: 0.8e-3, // = cylinder::KNOCK_DECAY_S (the plant's knock envelope)
            // a cast-iron block responds broadband through the knock range (3–25 kHz): six damped modes
            block_modes: [(5_200.0, 8.0, 0.8), (6_900.0, 8.0, 1.0), (9_500.0, 8.0, 0.8), (13_500.0, 8.0, 0.7), (17_500.0, 8.0, 0.6), (22_000.0, 8.0, 0.5)],
            seat_ramp_m: 0.6e-3,
            seat_g_per_m_s: 12.0,
            slap_g: 8.0,
            slap_deg: 12.0,
            injector_g: 2.0,
            shots_per_rev: 24.0,
            shot_g: 1.0,
        }
    }

    /// Volts at sensor `s` per g at the source position `pos` (cylinder pitches).
    fn gain(&self, s: usize, pos: f64) -> f64 {
        self.v_per_g * self.attenuation_per_pitch.powf((self.sensor_pos[s] - pos).abs())
    }

    /// Piezo resonance: second-order gain and phase at `f` Hz.
    fn resonance(&self, f: f64) -> (f64, f64) {
        let r = f / self.resonance_hz;
        let (re, im) = (1.0 - r * r, r / self.resonance_q);
        let m2 = re * re + im * im;
        (1.0 / m2.sqrt(), -im.atan2(re))
    }
}

/// One damped sinusoid at the sensor terminals: v_s(t) = v[s]·e^(−α(t−t0))·sin(ω(t−t0) + φ) for t0 ≤ t < t_end.
/// True when `target` (0..720) was passed going forward from `a` to `b`.
fn crossed(a: f64, b: f64, target: f64) -> bool {
    let step = (b - a).rem_euclid(720.0);
    let d = (target - a).rem_euclid(720.0);
    step > 0.0 && step < 360.0 && d > 0.0 && d <= step
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct VibAtom {
    pub t0: f64,
    pub t_end: f64,
    pub w: f64,
    pub alpha: f64,
    pub phi: f64,
    /// Peak volts at each sensor.
    pub v: [f64; 2],
}

/// Vibration field of the block: the live atoms and the impact generator state.
#[derive(Debug, Clone)]
pub struct BlockVibration {
    atoms: Box<[VibAtom; RING]>,
    head: usize,
    len: usize,
    rng: u64,
    next_shot_s: f64,
    /// Atoms pushed since the last [`BlockVibration::mark_read`].
    unread: usize,
    /// Knock events emitted (diagnostics), per cylinder.
    pub knock_events: [u64; 4],
}

impl Default for BlockVibration {
    fn default() -> Self {
        Self { atoms: Box::new([VibAtom::default(); RING]), head: 0, len: 0, rng: 0x9E37_79B9_7F4A_7C15, next_shot_s: 0.0, unread: 0, knock_events: [0; 4] }
    }
}

impl BlockVibration {
    fn rand(&mut self) -> f64 {
        // xorshift64*: deterministic, cheap.
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        (self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }

    #[allow(clippy::too_many_arguments)]
    fn push(&mut self, hw: &KnockSensorHw, t0: f64, f_hz: f64, tau: f64, phi: f64, g: f64, pos: f64) {
        let (h, dphi) = hw.resonance(f_hz);
        let v = std::array::from_fn(|s| if s < hw.sensors { g * h * hw.gain(s, pos) } else { 0.0 });
        let a = VibAtom { t0, t_end: t0 + LIFE_TAUS * tau, w: 2.0 * PI * f_hz, alpha: 1.0 / tau, phi: phi + dphi, v };
        let i = (self.head + self.len) % RING;
        self.atoms[i] = a;
        if self.len < RING {
            self.len += 1;
        } else {
            self.head = (self.head + 1) % RING; // full: the oldest atom gives way
        }
        self.unread = (self.unread + 1).min(self.len);
    }

    /// Atoms created since the last [`Self::mark_read`] (oldest first): the ECU board keeps its own copy of the signal
    /// at its input pins and samples it at its converter's conversion times.
    pub fn unread(&self) -> impl Iterator<Item = &VibAtom> {
        (self.len - self.unread..self.len).map(move |k| &self.atoms[(self.head + k) % RING])
    }

    pub fn mark_read(&mut self) {
        self.unread = 0;
    }

    /// Impulse on the block at `pos` (cylinder pitches) at time `t`, `g` peak response: rings every block mode.
    pub fn impact(&mut self, hw: &KnockSensorHw, t: f64, pos: f64, g: f64) {
        if g <= 0.0 {
            return;
        }
        for (f, q, w) in hw.block_modes {
            let jitter = 0.85 + 0.3 * self.rand();
            self.push(hw, t, f, q / (PI * f), 0.0, g * w * jitter, pos);
        }
    }

    /// Knock in cylinder `cyl` at `t`: first-mode pressure oscillation `amp_pa`, burned gas at `t_gas_k`.
    pub fn knock(&mut self, hw: &KnockSensorHw, t: f64, cyl: usize, amp_pa: f64, t_gas_k: f64, bore_m: f64) {
        self.knock_events[cyl] += 1;
        let c = (GAMMA_BURNED * R_BURNED * t_gas_k.max(800.0)).sqrt();
        let g1 = hw.knock_g_per_bar * amp_pa / 1e5;
        for (alpha, w) in KNOCK_MODES {
            let phi = 2.0 * PI * self.rand();
            self.push(hw, t, alpha * c / (PI * bore_m), hw.knock_decay_s, phi, g1 * w, cyl as f64);
        }
    }

    /// Valve seating of cylinder `cyl` (closing ramp at crank speed `omega`).
    pub fn valve_seat(&mut self, hw: &KnockSensorHw, t: f64, cyl: usize, omega: f64) {
        self.impact(hw, t, cyl as f64, hw.seat_g_per_m_s * hw.seat_ramp_m * omega.abs());
    }

    /// Piston slap of cylinder `cyl` after its firing TDC with peak pressure `p_max_pa`.
    pub fn piston_slap(&mut self, hw: &KnockSensorHw, t: f64, cyl: usize, omega: f64, p_max_pa: f64) {
        let g = hw.slap_g * (p_max_pa.max(0.0) / 60e5).sqrt() * (omega.abs() / (3000.0 * PI / 30.0));
        self.impact(hw, t, cyl as f64, g);
    }

    /// DI needle seating of cylinder `cyl` at rail − cylinder pressure `dp_pa`.
    pub fn injector_close(&mut self, hw: &KnockSensorHw, t: f64, cyl: usize, dp_pa: f64) {
        self.impact(hw, t, cyl as f64, hw.injector_g * (dp_pa.max(0.0) / 1e7).sqrt());
    }

    /// Events of cylinder `cyl` in one crank step of the plant (local angle `phi0` → `phi1`, 0..720 from its firing
    /// TDC, ending at time `t`): knock that started in the step (`knock_pa` > 0, burned gas `t_gas_k`), seating of
    /// its intake/exhaust valves at their closing angles `closes_deg`, piston slap at `slap_deg` after firing TDC
    /// (`p_cyl_pa` = cylinder pressure there), and a DI needle that closed at `t_inj_close` against `dp_inj_pa`.
    #[allow(clippy::too_many_arguments)]
    pub fn cylinder_step(&mut self, hw: &KnockSensorHw, t: f64, cyl: usize, phi0: f64, phi1: f64, omega: f64, closes_deg: [f64; 2], p_cyl_pa: f64, knock_pa: f64, t_gas_k: f64, bore_m: f64, inj_close: Option<(f64, f64)>) {
        if knock_pa > 0.0 {
            self.knock(hw, t, cyl, knock_pa, t_gas_k, bore_m);
        }
        for close in closes_deg {
            if crossed(phi0, phi1, close.rem_euclid(720.0)) {
                self.valve_seat(hw, t, cyl, omega);
            }
        }
        if crossed(phi0, phi1, hw.slap_deg) {
            self.piston_slap(hw, t, cyl, omega, p_cyl_pa);
        }
        if let Some((tc, dp)) = inj_close {
            self.injector_close(hw, tc, cyl, dp);
        }
    }

    /// Rotating-machinery impacts from `t0` to `t1` at crank speed `omega` (Poisson process).
    pub fn background(&mut self, hw: &KnockSensorHw, t0: f64, t1: f64, omega: f64) {
        let rate = hw.shots_per_rev * omega.abs() / (2.0 * PI);
        if rate <= 0.0 {
            self.next_shot_s = t1;
            return;
        }
        if self.next_shot_s < t0 {
            self.next_shot_s = t0 - (1.0 - self.rand()).ln() / rate;
        }
        while self.next_shot_s < t1 {
            let (t, pos) = (self.next_shot_s, 3.0 * self.rand());
            let g = hw.shot_g * (0.5 + self.rand());
            self.impact(hw, t, pos, g);
            self.next_shot_s += -(1.0 - self.rand()).ln() / rate;
        }
    }

    /// Drops atoms that have decayed before `t`.
    pub fn prune(&mut self, t: f64) {
        while self.len > self.unread && self.atoms[self.head].t_end < t {
            self.head = (self.head + 1) % RING;
            self.len -= 1;
        }
    }

    /// Live atoms (diagnostics).
    pub fn live(&self) -> usize {
        self.len
    }

    /// Sensor `s` output voltage at `t` (signal only, around 0 V).
    pub fn sensor_v(&self, s: usize, t: f64) -> f64 {
        let mut v = 0.0;
        for k in 0..self.len {
            let a = &self.atoms[(self.head + k) % RING];
            if t >= a.t0 && t < a.t_end {
                let x = t - a.t0;
                v += a.v[s] * (-a.alpha * x).exp() * (a.w * x + a.phi).sin();
            }
        }
        v
    }

    /// Adds sensor `s`'s voltage at `t_first + n·ts` to `out[n]` (n = 0..out.len()), one complex rotation per atom
    /// and sample.
    pub fn sample_into(&self, s: usize, t_first: f64, ts: f64, out: &mut [f64]) {
        let n = out.len();
        if n == 0 {
            return;
        }
        let t_last = t_first + (n - 1) as f64 * ts;
        for k in 0..self.len {
            let a = &self.atoms[(self.head + k) % RING];
            if a.v[s] == 0.0 || a.t0 > t_last || a.t_end < t_first {
                continue;
            }
            let i0 = if a.t0 > t_first { ((a.t0 - t_first) / ts).ceil() as usize } else { 0 };
            if i0 >= n {
                continue;
            }
            let x = t_first + i0 as f64 * ts - a.t0;
            let m = a.v[s] * (-a.alpha * x).exp();
            let (mut im, mut re) = (a.w * x + a.phi).sin_cos();
            (re, im) = (re * m, im * m);
            let d = (-a.alpha * ts).exp();
            let (ds, dc) = (a.w * ts).sin_cos();
            let (rr, ri) = (dc * d, ds * d);
            let i_end = (((a.t_end - t_first) / ts).floor() as usize + 1).min(n);
            for o in out[i0..i_end].iter_mut() {
                *o += im;
                (re, im) = (re * rr - im * ri, re * ri + im * rr);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Phasor recursion equals direct evaluation, and expired atoms vanish.
    #[test]
    fn batch_sampling_equals_direct_evaluation() {
        let hw = KnockSensorHw::ea888_gen3();
        let mut b = BlockVibration::default();
        b.knock(&hw, 1.0e-3, 1, 2e5, 2400.0, 0.0825);
        b.valve_seat(&hw, 1.3e-3, 2, 600.0);
        b.background(&hw, 0.0, 3e-3, 600.0);
        let ts = 1.0 / 80_000.0;
        let mut out = vec![0.0; 300];
        b.sample_into(0, 0.5e-3, ts, &mut out);
        for (i, &v) in out.iter().enumerate() {
            let d = b.sensor_v(0, 0.5e-3 + i as f64 * ts);
            assert!((v - d).abs() < 1e-9 + 1e-6 * d.abs(), "sample {i}: {v} vs {d}");
        }
        assert!(out.iter().any(|v| v.abs() > 0.01));
        b.prune(1.0);
        assert!(b.live() > 0, "undelivered atoms are kept");
        assert!(b.unread().count() > 0);
        b.mark_read();
        b.prune(1.0);
        assert_eq!(b.live(), 0);
        assert_eq!(b.sensor_v(0, 1.0), 0.0);
    }

    /// Knock rings at Draper's first mode: c = √(γRT) at 2400 K over an 82.5 mm bore → ≈ 6.6–6.9 kHz; the sensor
    /// nearest the knocking cylinder sees the larger signal; 1 bar of first-mode oscillation gives the order of
    /// tens of mV (KS4: 26 mV/g).
    #[test]
    fn knock_mode_frequency_and_transfer() {
        let mut hw = KnockSensorHw::ea888_gen3();
        hw.sensors = 2;
        hw.sensor_pos = [0.5, 2.5];
        let mut b = BlockVibration::default();
        b.knock(&hw, 0.0, 0, 1e5, 2400.0, 0.0825);
        let ts = 1.0 / 400_000.0;
        let mut s0 = vec![0.0; 400];
        let mut s1 = vec![0.0; 400];
        b.sample_into(0, 0.0, ts, &mut s0);
        b.sample_into(1, 0.0, ts, &mut s1);
        let peak = |v: &[f64]| v.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak(&s0) > 1.5 * peak(&s1), "near sensor {} far {}", peak(&s0), peak(&s1));
        assert!((0.05..0.3).contains(&peak(&s0)), "{} V", peak(&s0));
        let f1 = 1.841 * (GAMMA_BURNED * R_BURNED * 2400.0f64).sqrt() / (PI * 0.0825);
        assert!((6_500.0..7_000.0).contains(&f1), "{f1}");
        // dominant spectral line of the first millisecond = Draper's first mode
        let amp = |f: f64| {
            let (mut re, mut im) = (0.0, 0.0);
            for (i, v) in s0.iter().enumerate() {
                let ph = 2.0 * PI * f * i as f64 * ts;
                re += v * ph.cos();
                im += v * ph.sin();
            }
            re.hypot(im)
        };
        let f_peak = (30..250).map(|k| k as f64 * 100.0).fold((0.0, 0.0), |m, f| if amp(f) > m.1 { (f, amp(f)) } else { m }).0;
        assert!((f_peak / f1 - 1.0).abs() < 0.05, "{f_peak} Hz vs {f1}");
    }

    /// The background is rate-driven: more impacts at higher speed, deterministic for a given history.
    #[test]
    fn background_scales_with_speed_and_is_deterministic() {
        let hw = KnockSensorHw::ea888_gen3();
        let rms = |omega: f64| {
            let mut b = BlockVibration::default();
            let mut acc = 0.0;
            let ts = 1.0 / 50_000.0;
            let mut out = vec![0.0; 50];
            for k in 0..200 {
                let t0 = k as f64 * 1e-3;
                b.background(&hw, t0, t0 + 1e-3, omega);
                out.iter_mut().for_each(|x| *x = 0.0);
                b.sample_into(0, t0, ts, &mut out);
                acc += out.iter().map(|x| x * x).sum::<f64>();
                b.prune(t0);
            }
            (acc / 10_000.0).sqrt()
        };
        let (lo, hi) = (rms(100.0), rms(600.0));
        assert!(hi > 2.0 * lo, "{lo} {hi}");
        assert_eq!(rms(300.0), rms(300.0));
    }
}
