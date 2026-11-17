//! The synthesizer: plant records → 48 kHz pressure at the listener positions.
//!
//! 1. **Resampling.** Crank frames arrive every crank step (≤ 1 °CA / ≤ 50 µs). Every channel is taken as
//!    piecewise linear between frame ends and box-averaged over each 1/48 000 s sample: mass and impulse are
//!    conserved, nothing above the plant's own resolution is invented (a 20 kHz frame rate at idle limits the
//!    flow/pressure-driven sources to ≈ 10 kHz; impacts are placed with sub-sample timing and their spectrum
//!    comes from the structure they excite).
//! 2. **Sources** (one signal each = radiated pressure at 1 m, Pa): exhaust and intake by transfer-matrix duct
//!    acoustics (`duct`), driven by the plant's own turbine-outlet and compressor-inlet mass flows; structure-
//!    borne sources as forces/impulses into modal models of the engine structure (J.-M. Adrien, "The missing
//!    link: modal synthesis", MIT Press 1991; K. van den Doel, P. Kry, D. Pai, "FoleyAutomatic", SIGGRAPH 2001)
//!    radiating with a plate radiation efficiency (F. Fahy, P. Gardonio, "Sound and Structural Vibration",
//!    2nd ed. 2007, ch. 3); aero-acoustic jets by Lighthill's U⁸ law (M. J. Lighthill, Proc. R. Soc. A 211
//!    (1952) 564); fan noise by Sharland's U⁶ dipole law (I. J. Sharland, J. Sound Vib. 1 (1964) 302); rotor
//!    tones at blade-passing frequency (J. M. Tyler, T. G. Sofrin, SAE Trans. 70 (1962) 309).
//! 3. **Listener paths**: spherical spreading, propagation delay, mass-law panel transmission and leakage.
//!
//! Every constant without a data-sheet value is a field of [`Params`] and marked UNVERIFIED.

use crate::conv::{Convolver, BLOCK as B};
use crate::duct::{Duct, ExhaustGeometry, IntakeGeometry};
use crate::tap::{Config, Record, Slow};
use simemu_physics::acoustic_probe::{CrankFrame, Impact, ImpactKind};
use std::f64::consts::PI;

pub const FS: f64 = 48_000.0;
const T: f64 = 1.0 / FS;
const EX_FIR: usize = 8192;
const IN_FIR: usize = 4096;
const C0: f32 = 343.0;

/// Sound sources (mixer channels).
pub const N_SRC: usize = 16;
pub mod src {
    pub const EXHAUST: usize = 0;
    pub const INTAKE: usize = 1;
    pub const TURBO: usize = 2;
    pub const COMBUSTION: usize = 3;
    pub const KNOCK: usize = 4;
    pub const VALVETRAIN: usize = 5;
    pub const INJECTORS: usize = 6;
    pub const HPFP: usize = 7;
    pub const CHAIN: usize = 8;
    pub const MOUNTS: usize = 9;
    pub const STARTER: usize = 10;
    pub const ALTERNATOR: usize = 11;
    pub const FAN: usize = 12;
    pub const CLUTCH: usize = 13;
    pub const GEARS: usize = 14;
    pub const PUMP: usize = 15;
}
/// (key, Russian, English) names; `CYL_RESOLVED[s]` = the cylinder mask applies.
pub const SRC_NAMES: [(&str, &str, &str); N_SRC] = [
    ("exhaust", "Выхлоп (срез трубы)", "Exhaust (tailpipe)"),
    ("intake", "Впуск (заборник)", "Intake (snorkel)"),
    ("turbo", "Турбокомпрессор / перепуск", "Turbocharger / diverter"),
    ("combustion", "Сгорание (блок)", "Combustion (block)"),
    ("knock", "Детонация", "Knock ringing"),
    ("valvetrain", "ГРМ: клапаны", "Valvetrain"),
    ("injectors", "Форсунки", "Injectors"),
    ("hpfp", "ТНВД", "HPFP"),
    ("chain", "Цепь ГРМ", "Timing chain"),
    ("mounts", "Опоры (гул в салоне)", "Mounts (cabin boom)"),
    ("starter", "Стартер", "Starter"),
    ("alternator", "Генератор", "Alternator"),
    ("fan", "Вентилятор радиатора", "Radiator fan"),
    ("clutch", "Сцепления DQ200", "DQ200 clutches"),
    ("gears", "Шестерни / синхронизаторы", "Gears / synchros"),
    ("pump", "Насос мехатроника", "Mechatronic pump"),
];
pub const CYL_RESOLVED: [bool; N_SRC] = [true, true, false, true, true, true, true, false, false, false, false, false, false, false, false, false];

/// Listener positions; index 3 = stethoscope (source signals without a path).
pub const N_LIST: usize = 4;
pub const LISTENER_NAMES: [(&str, &str, &str); N_LIST] = [
    ("exterior", "Снаружи у трубы (0.5 м, 45°)", "Exterior at tailpipe (0.5 m, 45°)"),
    ("cabin", "Салон (место водителя)", "Cabin (driver's ear)"),
    ("bay", "Моторный отсек (капот открыт)", "Engine bay (hood open)"),
    ("stethoscope", "Стетоскоп (источник напрямую)", "Stethoscope (source direct)"),
];

/// Mixer: per-source gain (mute/solo resolved by the UI), cylinder weights, master.
#[derive(Debug, Clone, Copy)]
pub struct Mix {
    pub gain: [f32; N_SRC],
    pub cyl: [f32; 4],
    pub master: f32,
}

impl Default for Mix {
    fn default() -> Self {
        Self { gain: [1.0; N_SRC], cyl: [1.0; 4], master: 1.0 }
    }
}

/// Modal model of one radiating structure: (frequency Hz, Q, modal mass kg) per mode, radiating area m²,
/// plate critical frequency Hz (radiation efficiency σ = 1/√(1 + (f_c/f)⁴)).
#[derive(Debug, Clone, Copy)]
pub struct Structure {
    pub modes: &'static [(f32, f32, f32)],
    pub area_m2: f32,
    pub f_crit: f32,
}

/// UNVERIFIED: no modal survey of the EA888 Gen3 / DQ200 housings is available; modal densities and masses are
/// typical of cast-aluminium powertrain housings (first block modes 200–600 Hz, head/covers stiffer and lighter).
pub const BLOCK: Structure = Structure {
    modes: &[(240.0, 8.0, 30.0), (520.0, 10.0, 15.0), (830.0, 14.0, 10.0), (1180.0, 16.0, 8.0), (1560.0, 18.0, 6.0), (1980.0, 20.0, 5.0), (2450.0, 22.0, 5.0), (2950.0, 24.0, 4.0), (3500.0, 26.0, 4.0), (4100.0, 28.0, 3.5), (4750.0, 30.0, 3.0), (5450.0, 30.0, 3.0), (6200.0, 32.0, 3.0), (7000.0, 32.0, 3.0), (7900.0, 35.0, 3.0), (8900.0, 35.0, 3.0)],
    area_m2: 0.5,
    f_crit: 2500.0,
};
pub const HEAD: Structure = Structure {
    modes: &[(650.0, 12.0, 6.0), (980.0, 14.0, 4.0), (1350.0, 16.0, 3.0), (1800.0, 18.0, 2.5), (2300.0, 20.0, 2.0), (2900.0, 22.0, 2.0), (3550.0, 24.0, 1.8), (4300.0, 26.0, 1.6), (5100.0, 28.0, 1.5), (6000.0, 30.0, 1.4), (7000.0, 30.0, 1.3), (8200.0, 32.0, 1.2), (9500.0, 35.0, 1.2), (11000.0, 35.0, 1.2)],
    area_m2: 0.25,
    f_crit: 3000.0,
};
pub const COVER: Structure = Structure {
    modes: &[(450.0, 10.0, 4.0), (800.0, 12.0, 3.0), (1250.0, 15.0, 2.0), (1750.0, 18.0, 1.5), (2400.0, 20.0, 1.2), (3200.0, 22.0, 1.0), (4100.0, 24.0, 1.0), (5200.0, 26.0, 1.0), (6500.0, 28.0, 1.0)],
    area_m2: 0.12,
    f_crit: 3500.0,
};
pub const STARTER: Structure = Structure {
    modes: &[(700.0, 10.0, 1.5), (1300.0, 12.0, 1.0), (2100.0, 15.0, 0.8), (3100.0, 18.0, 0.6), (4400.0, 20.0, 0.5), (6000.0, 22.0, 0.5)],
    area_m2: 0.04,
    f_crit: 4000.0,
};
pub const GEARBOX: Structure = Structure {
    modes: &[(380.0, 10.0, 12.0), (690.0, 12.0, 8.0), (1050.0, 15.0, 6.0), (1500.0, 18.0, 4.0), (2050.0, 20.0, 3.0), (2700.0, 22.0, 3.0), (3500.0, 24.0, 2.5), (4500.0, 26.0, 2.0), (5800.0, 28.0, 2.0)],
    area_m2: 0.35,
    f_crit: 2800.0,
};

/// Model constants. Everything here is UNVERIFIED unless the doc says otherwise (knobs of the model).
#[derive(Debug, Clone, Copy)]
pub struct Params {
    pub exhaust: ExhaustGeometry,
    pub intake: IntakeGeometry,
    /// Fraction of the first-mode knock pressure that acts as a net force on the liner/head (the mode is
    /// antisymmetric; the knock sensor sees the local wall motion).
    pub knock_force_frac: f32,
    /// Ratio of specific heats and gas constant of burned gas for the knock-mode sound speed (C. S. Draper,
    /// J. Aeronaut. Sci. 5 (1938) 219: f = 1.841·c/(π·B), the first circumferential mode).
    pub knock_gamma: f32,
    /// The gas force is reacted inside the engine (head bolts ↔ main bearings): only its local, short-wavelength
    /// part deforms the radiating walls. Modelled as coupling `comb_coupling` × first-order high-pass at
    /// `comb_coupling_hz` (stand-in for the measured structure-attenuation curve of the engine, which is not
    /// available).
    pub comb_coupling: f32,
    pub comb_coupling_hz: f32,
    /// Cam-bearing ↔ valve-seat load transfer is a self-equilibrated force pair: effective net fraction.
    pub valve_force_frac: f32,
    /// HPFP plunger load is reacted between the pump flange and the cam bearing: effective net fraction.
    pub hpfp_force_frac: f32,
    pub valve_restitution: f32,
    /// Injector armature/needle: moving mass kg, opening impact velocity m/s, closing velocity at 10 MPa ΔP.
    pub inj_mass_kg: f32,
    pub inj_open_ms: f32,
    pub inj_close_ms_10mpa: f32,
    /// HPFP N276 armature mass / impact velocity at 13.5 V; inlet plate and outlet ball impulses.
    pub hpfp_arm_kg: f32,
    pub hpfp_arm_ms: f32,
    pub hpfp_plate_ns: f32,
    pub hpfp_ball_ns_10mpa: f32,
    /// Roller-to-plunger lever: plunger force = HPFP crank torque / this (mean dh/dθ_crank = lift/(π/2)), m/rad.
    pub hpfp_lever_m: f32,
    /// Timing chain: crank sprocket teeth, sprocket pitch radius m, link mass kg, fraction of the polygonal
    /// impact that reaches the cover, tension reference Nm.
    pub chain_teeth: f32,
    pub chain_radius_m: f32,
    pub chain_link_kg: f32,
    pub chain_frac: f32,
    pub chain_tension_nm: f32,
    /// Cabin pressure per Nm of block roll torque below ~100 Hz (mounts + body), Pa/Nm.
    pub mount_pa_per_nm: f32,
    pub mount_lp_hz: f32,
    /// Starter: planetary ratio, pinion teeth, pinion pitch radius m, motor torque constant Nm/A, mesh
    /// transmission-error fraction, commutator segments, brush impulse per ampere N·s/A, pinion throw / retract
    /// impulses N·s. Ratio and teeth as `simemu_physics::starter` (4.6, 9).
    pub st_planetary: f32,
    pub st_pinion_teeth: f32,
    pub st_pinion_r_m: f32,
    pub st_k_nm_a: f32,
    pub st_te_frac: f32,
    pub st_segments: f32,
    pub st_brush_ns_a: f32,
    pub st_engage_ns: f32,
    pub st_release_ns: f32,
    /// Compressor: main/splitter blade counts, BPF level at 300 m/s tip speed (∝ U⁴), broadband level at
    /// 300 m/s (∝ U³), rotor residual unbalance kg·m (ISO 1940 class) and bearing-housing mass kg.
    pub comp_blades: f32,
    pub comp_bpf_pa_300: f32,
    pub comp_bb_pa_300: f32,
    pub turbo_unbalance_kgm: f32,
    pub turbo_housing_kg: f32,
    /// The plant's compressor is a quasi-steady map solved every 0.2 ms without inlet-duct inertia; near zero
    /// shaft speed its flow chatters from one sub-step to the next (a solver effect, not sound). Its flow is
    /// used as the intake source only below this cut-off (2nd-order Butterworth), where a quasi-steady map is
    /// valid (surge cycles and firing-order pulsations stay; rotor noise is the turbo source).
    pub comp_flow_lp_hz: f32,
    /// Lighthill constant of jet mixing noise (3e-5…1e-4 for subsonic cold jets).
    pub lighthill_k: f32,
    /// The throttle and diverter jets blow inside the intake ducts: their sound leaves through the duct wall
    /// (normal-incidence mass law, m″ of a 3 mm PA6-GF30 wall ≈ 4.5 kg/m²).
    pub duct_wall_kg_m2: f32,
    /// Alternator: magnetic tone per ampere at orders 36 and 72 (12 poles × 3 phases, 36-slot stator), Pa/A at 1 m.
    pub alt_pa_per_a: f32,
    /// Radiator fan: blades, diameter m, max speed rpm, spin-up time constant s, Sharland constant.
    pub fan_blades: f32,
    pub fan_dia_m: f32,
    pub fan_max_rpm: f32,
    pub fan_tau_s: f32,
    pub fan_sharland_k: f32,
    /// Dry-clutch friction noise per √W of slip power, Pa/√W at 1 m (force spectrum into the gearbox housing).
    pub clutch_n_per_sqrt_w: f32,
    /// Gear mesh force per Nm of mesh torque (transmission-error fraction / pitch radius), N/Nm.
    pub mesh_n_per_nm: f32,
    /// Sleeve (+ fork) mass for engagement clunks, kg; impulse per unit dog-health loss, N·s.
    pub sleeve_kg: f32,
    pub dog_clash_ns: f32,
    /// Synchro cone friction noise force per √W, N/√W.
    pub synchro_n_per_sqrt_w: f32,
    /// Mechatronic pump: gear teeth, pressure-ripple force per Pa, motor commutator force per A.
    pub pump_teeth: f32,
    pub pump_n_per_pa: f32,
    pub pump_n_per_a: f32,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            exhaust: ExhaustGeometry::default(),
            intake: IntakeGeometry::default(),
            knock_force_frac: 0.4,
            knock_gamma: 1.3,
            comb_coupling: 0.3,
            comb_coupling_hz: 2000.0,
            valve_force_frac: 0.15,
            hpfp_force_frac: 0.05,
            valve_restitution: 0.4,
            inj_mass_kg: 3.0e-3,
            inj_open_ms: 0.8,
            inj_close_ms_10mpa: 0.35,
            hpfp_arm_kg: 6.0e-3,
            hpfp_arm_ms: 1.2,
            hpfp_plate_ns: 2.0e-3,
            hpfp_ball_ns_10mpa: 6.0e-4,
            hpfp_lever_m: 3.2e-3 / (PI as f32 / 2.0),
            chain_teeth: 21.0,
            chain_radius_m: 0.0267,
            chain_link_kg: 5.0e-3,
            chain_frac: 0.15,
            chain_tension_nm: 15.0,
            mount_pa_per_nm: 4.0e-4,
            mount_lp_hz: 100.0,
            st_planetary: 4.6,
            st_pinion_teeth: 9.0,
            st_pinion_r_m: 0.0113,
            st_k_nm_a: 5.5e-3,
            st_te_frac: 0.03,
            st_segments: 22.0,
            st_brush_ns_a: 2.0e-6,
            st_engage_ns: 0.12,
            st_release_ns: 0.05,
            comp_blades: 6.0,
            comp_bpf_pa_300: 0.05,
            comp_bb_pa_300: 0.03,
            turbo_unbalance_kgm: 2.0e-8,
            turbo_housing_kg: 3.0,
            comp_flow_lp_hz: 300.0,
            lighthill_k: 3.0e-5,
            duct_wall_kg_m2: 4.5,
            alt_pa_per_a: 2.0e-4,
            fan_blades: 7.0,
            fan_dia_m: 0.38,
            fan_max_rpm: 2800.0,
            fan_tau_s: 1.5,
            fan_sharland_k: 3.0e-5,
            clutch_n_per_sqrt_w: 0.6,
            mesh_n_per_nm: 0.4,
            sleeve_kg: 0.35,
            dog_clash_ns: 40.0,
            synchro_n_per_sqrt_w: 0.8,
            pump_teeth: 9.0,
            pump_n_per_pa: 4.0e-7,
            pump_n_per_a: 0.05,
        }
    }
}

/// Listener path of one source class: distance m, extra gain (shielding), mass-law corner Hz (0 = open air),
/// leakage fraction (apertures/sealing) bypassing the panel.
#[derive(Debug, Clone, Copy)]
pub struct PathSpec {
    pub r_m: f32,
    pub gain: f32,
    pub mass_law_hz: f32,
    pub leak: f32,
}

/// Source classes by location: tailpipe, intake orifice, engine structure, turbo, front end, gearbox, mounts.
const CLASS: [usize; N_SRC] = [0, 1, 3, 2, 2, 2, 2, 2, 2, 6, 2, 4, 4, 5, 5, 5];
/// UNVERIFIED geometry of the listener positions (ISO 5130 exterior point; driver's ear; 0.5 m over the
/// engine with the hood open). Mass-law corner f = ρc/(π·m″): floor/rear panels m″ ≈ 12 kg/m², dash ≈ 15 kg/m².
pub const PATHS: [[PathSpec; 7]; 3] = [
    [
        PathSpec { r_m: 0.5, gain: 1.0, mass_law_hz: 0.0, leak: 1.0 },
        PathSpec { r_m: 4.4, gain: 0.5, mass_law_hz: 2000.0, leak: 0.3 },
        PathSpec { r_m: 4.2, gain: 0.5, mass_law_hz: 1500.0, leak: 0.3 },
        PathSpec { r_m: 4.3, gain: 0.5, mass_law_hz: 1500.0, leak: 0.3 },
        PathSpec { r_m: 4.5, gain: 0.7, mass_law_hz: 0.0, leak: 1.0 },
        PathSpec { r_m: 3.8, gain: 0.6, mass_law_hz: 2000.0, leak: 0.3 },
        PathSpec { r_m: 1.0, gain: 0.0, mass_law_hz: 0.0, leak: 1.0 },
    ],
    [
        PathSpec { r_m: 3.0, gain: 1.0, mass_law_hz: 11.0, leak: 0.01 },
        PathSpec { r_m: 1.8, gain: 1.0, mass_law_hz: 9.0, leak: 0.01 },
        PathSpec { r_m: 1.5, gain: 1.0, mass_law_hz: 9.0, leak: 0.015 },
        PathSpec { r_m: 1.6, gain: 1.0, mass_law_hz: 9.0, leak: 0.01 },
        PathSpec { r_m: 2.5, gain: 1.0, mass_law_hz: 9.0, leak: 0.01 },
        PathSpec { r_m: 1.2, gain: 1.0, mass_law_hz: 11.0, leak: 0.015 },
        PathSpec { r_m: 1.0, gain: 1.0, mass_law_hz: 0.0, leak: 1.0 },
    ],
    [
        PathSpec { r_m: 4.5, gain: 1.0, mass_law_hz: 0.0, leak: 1.0 },
        PathSpec { r_m: 0.6, gain: 1.0, mass_law_hz: 0.0, leak: 1.0 },
        PathSpec { r_m: 0.5, gain: 1.0, mass_law_hz: 0.0, leak: 1.0 },
        PathSpec { r_m: 0.5, gain: 1.0, mass_law_hz: 0.0, leak: 1.0 },
        PathSpec { r_m: 0.8, gain: 1.0, mass_law_hz: 0.0, leak: 1.0 },
        PathSpec { r_m: 0.9, gain: 1.0, mass_law_hz: 0.0, leak: 1.0 },
        PathSpec { r_m: 1.0, gain: 0.0, mass_law_hz: 0.0, leak: 1.0 },
    ],
];

// ---- small DSP blocks --------------------------------------------------------------------------------

/// Biquad, transposed direct form II.
#[derive(Debug, Clone, Copy, Default)]
struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl Biquad {
    /// Band-pass with 0 dB peak (R. Bristow-Johnson, "Audio EQ cookbook").
    fn bandpass(f: f64, q: f64) -> Self {
        let w = 2.0 * PI * f.clamp(1.0, 0.45 * FS) / FS;
        let al = w.sin() / (2.0 * q.max(0.05));
        let a0 = 1.0 + al;
        Self { b0: (al / a0) as f32, b1: 0.0, b2: (-al / a0) as f32, a1: (-2.0 * w.cos() / a0) as f32, a2: ((1.0 - al) / a0) as f32, ..Default::default() }
    }
    fn lowpass(f: f64, q: f64) -> Self {
        let w = 2.0 * PI * f.clamp(1.0, 0.45 * FS) / FS;
        let al = w.sin() / (2.0 * q.max(0.05));
        let a0 = 1.0 + al;
        let c = w.cos();
        Self { b0: ((1.0 - c) / 2.0 / a0) as f32, b1: ((1.0 - c) / a0) as f32, b2: ((1.0 - c) / 2.0 / a0) as f32, a1: (-2.0 * c / a0) as f32, a2: ((1.0 - al) / a0) as f32, ..Default::default() }
    }
    fn highpass(f: f64, q: f64) -> Self {
        let w = 2.0 * PI * f.clamp(1.0, 0.45 * FS) / FS;
        let al = w.sin() / (2.0 * q.max(0.05));
        let a0 = 1.0 + al;
        let c = w.cos();
        Self { b0: ((1.0 + c) / 2.0 / a0) as f32, b1: (-(1.0 + c) / a0) as f32, b2: ((1.0 + c) / 2.0 / a0) as f32, a1: (-2.0 * c / a0) as f32, a2: ((1.0 - al) / a0) as f32, ..Default::default() }
    }
    /// Keeps the state, replaces the coefficients.
    fn retune(&mut self, n: Biquad) {
        let (z1, z2) = (self.z1, self.z2);
        *self = n;
        self.z1 = z1;
        self.z2 = z2;
    }
    #[inline]
    fn run(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}

/// Force → velocity of each mode (s/m / (s² + ω/Q s + ω²)) → radiated pressure at 1 m:
/// p = ρc·√(σ·S/(4π))·v (radiated power ρcσS⟨v²⟩ spread over a sphere).
struct Bank {
    bq: Vec<Biquad>,
    gain: Vec<f32>,
}

impl Bank {
    fn new(s: &Structure) -> Self {
        let rho_c = 1.2 * 343.0;
        let (mut bq, mut gain) = (Vec::new(), Vec::new());
        for &(f, q, m) in s.modes {
            if f as f64 >= 0.45 * FS {
                continue;
            }
            bq.push(Biquad::bandpass(f as f64, q as f64));
            let sigma = 1.0 / (1.0 + (s.f_crit / f).powi(4)).sqrt();
            gain.push(q / (m * 2.0 * PI as f32 * f) * rho_c * (sigma * s.area_m2 / (4.0 * PI as f32)).sqrt());
        }
        Self { bq, gain }
    }
    fn run(&mut self, force: &[f32; B], out: &mut [f32; B]) {
        out.fill(0.0);
        for (b, g) in self.bq.iter_mut().zip(&self.gain) {
            for i in 0..B {
                out[i] += g * b.run(force[i]);
            }
        }
    }
}

struct Path {
    delay: Vec<f32>,
    pos: usize,
    d: usize,
    lp_a: f32,
    lp_y: f32,
    gain: f32,
    leak: f32,
}

impl Path {
    fn new(s: &PathSpec) -> Self {
        let d = ((s.r_m / C0) as f64 * FS).round() as usize;
        Self {
            delay: vec![0.0; (d + 1).next_power_of_two()],
            pos: 0,
            d,
            lp_a: if s.mass_law_hz > 0.0 { (-2.0 * PI * s.mass_law_hz as f64 / FS).exp() as f32 } else { 0.0 },
            lp_y: 0.0,
            gain: s.gain / s.r_m.max(0.05),
            leak: s.leak,
        }
    }
    #[inline]
    fn run(&mut self, x: f32) -> f32 {
        let mask = self.delay.len() - 1;
        self.delay[self.pos & mask] = x;
        let xd = self.delay[(self.pos + self.delay.len() - self.d) & mask];
        self.pos = self.pos.wrapping_add(1);
        self.lp_y = (1.0 - self.lp_a) * xd + self.lp_a * self.lp_y;
        self.gain * (self.leak * xd + (1.0 - self.leak) * self.lp_y)
    }
}

/// Deterministic white noise (xorshift64*), unit variance.
struct Noise(u64);

impl Noise {
    #[inline]
    fn next(&mut self) -> f32 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        let u = (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40) as f32 / (1u64 << 24) as f32;
        (2.0 * u - 1.0) * 1.732_050_8
    }
}

/// Band-limited noise shaped by a band-pass at `f` (Q ≈ 1); `rms` = target RMS.
struct ColoredNoise {
    bq: Biquad,
    f: f64,
}

impl ColoredNoise {
    fn new(f: f64) -> Self {
        Self { bq: Biquad::bandpass(f, 1.0), f }
    }
    fn tune(&mut self, f: f64) {
        let f = f.clamp(40.0, 0.4 * FS);
        if (f / self.f - 1.0).abs() > 0.03 {
            self.f = f;
            self.bq.retune(Biquad::bandpass(f, 1.0));
        }
    }
    /// White noise through a 0 dB-peak band-pass of bandwidth f/Q keeps a power fraction ≈ (π/2)·BW/(fs/2).
    fn norm(&self) -> f32 {
        (1.0 / (PI / 2.0 * self.f / (FS / 2.0)).sqrt()) as f32
    }
}

/// Lighthill jet: pressure RMS at 1 m from an orifice flow `m` kg/s driven by `dp` Pa from gas of density
/// `rho_up`, sound speed `c_up`; returns (p_rms, peak frequency St = 0.2).
fn jet(k: f32, m: f32, dp: f32, rho_up: f32, c_up: f32, rho0: f32) -> (f32, f32) {
    if m <= 1e-6 || dp <= 0.0 {
        return (0.0, 1000.0);
    }
    let u = (2.0 * dp / rho_up).sqrt().min(c_up);
    let a = m / (rho_up * u);
    let d = (4.0 * a / PI as f32).sqrt();
    let w = k * rho0 * u.powi(8) * d * d / C0.powi(5);
    ((rho0 * C0 * w / (4.0 * PI as f32)).sqrt(), 0.2 * u / d.max(1e-3))
}

/// Normal-incidence mass-law transmission of a limp wall of `m2` kg/m² at `f` Hz (Fahy & Gardonio §5.4).
fn wall(m2: f32, f: f32, rho0: f32) -> f32 {
    1.0 / (1.0 + (PI as f32 * f * m2 / (2.0 * rho0 * C0)).powi(2)).sqrt()
}

// ---- resampler channels ----------------------------------------------------------------------------

const CH_P: usize = 0;
const CH_K: usize = 4;
const CH_T: usize = 8;
const CH_EX: usize = 12;
const CH_IN: usize = 16;
const CH_VN: usize = 20;
const CH_MOUT: usize = 24;
const CH_MTHR: usize = 25;
const CH_MC: usize = 26;
const CH_MDV: usize = 27;
const CH_TGAS: usize = 28;
const CH_CAM: usize = 29;
const CH_HPFP: usize = 30;
const CH_OMEGA: usize = 31;
const CH_TURBO: usize = 32;
const CH_STW: usize = 33;
const CH_STA: usize = 34;
const CH_PEM: usize = 35;
const CH_TEM: usize = 36;
const CH_PIM: usize = 37;
const CH_TIM: usize = 38;
const CH_PB: usize = 39;
const CH_P4: usize = 40;
const CH_RAIL: usize = 41;
const CH_VBAT: usize = 42;
const CH_PRC: usize = 43;
const NCH: usize = 44;

fn channels(f: &CrankFrame) -> [f32; NCH] {
    let mut v = [0.0; NCH];
    v[CH_P..CH_P + 4].copy_from_slice(&f.p_cyl);
    v[CH_K..CH_K + 4].copy_from_slice(&f.knock_pa);
    v[CH_T..CH_T + 4].copy_from_slice(&f.t_cyl);
    v[CH_EX..CH_EX + 4].copy_from_slice(&f.m_ex);
    v[CH_IN..CH_IN + 4].copy_from_slice(&f.m_in);
    v[CH_VN..CH_VN + 4].copy_from_slice(&f.valve_n);
    v[CH_MOUT] = f.m_out;
    v[CH_MTHR] = f.m_thr;
    v[CH_MC] = f.m_c;
    v[CH_MDV] = f.m_dv;
    v[CH_TGAS] = f.t_gas_nm;
    v[CH_CAM] = f.cam_nm;
    v[CH_HPFP] = f.hpfp_nm;
    v[CH_OMEGA] = f.omega;
    v[CH_TURBO] = f.turbo_rad_s;
    v[CH_STW] = f.starter_rad_s;
    v[CH_STA] = f.starter_a;
    v[CH_PEM] = f.p_em;
    v[CH_TEM] = f.t_em;
    v[CH_PIM] = f.p_im;
    v[CH_TIM] = f.t_im;
    v[CH_PB] = f.p_boost;
    v[CH_P4] = f.p4;
    v[CH_RAIL] = f.rail_pa;
    v[CH_VBAT] = f.battery_v;
    v[CH_PRC] = f.pr_c_rel;
    for x in &mut v {
        if !x.is_finite() {
            *x = 0.0;
        }
    }
    v
}

/// Impulse (N·s) into the structure of source `src` at plant time `t`; `cyl` for the cylinder mask (4 = none).
#[derive(Debug, Clone, Copy)]
struct Ev {
    t: f64,
    src: u8,
    cyl: u8,
    j: f32,
}

/// One rendered block.
pub struct BlockOut {
    /// Plant time of the first sample, s.
    pub t0: f64,
    /// Pressure at the listener positions (and the stethoscope), Pa, after the mixer.
    pub out: [[f32; B]; N_LIST],
    /// Mean-square pressure of each source at each position before the mixer gain, Pa².
    pub level: [[f32; N_SRC]; N_LIST],
}

pub struct Synth {
    pub params: Params,
    pub mix: Mix,
    cfg: Config,
    // resampler
    started: bool,
    t_cur: f64,
    blk_t0: f64,
    v_prev: [f32; NCH],
    acc: [f32; NCH],
    fill: usize,
    blk: Box<[[f32; B]; NCH]>,
    // frame-domain state
    crank_unw: f64,
    crank_prev: f32,
    chain_next: f64,
    st_angle: f64,
    st_tooth_next: f64,
    st_seg_next: f64,
    st_on: bool,
    events: Vec<Ev>,
    slow: Slow,
    slow_prev: Option<Slow>,
    synchro_w: f32,
    // DSP
    ex_duct: Duct,
    in_duct: Duct,
    ex_conv: Convolver,
    in_conv: Convolver,
    ir: Vec<f32>,
    ir_t: f64,
    ir_key: [f32; 6],
    ex_rc: f32,
    in_rc: f32,
    banks: Vec<Bank>,
    knock_ph: [f64; 4],
    comb_hp_a: f32,
    comb_hp_x: f32,
    comb_hp_y: f32,
    ph: [f64; 10],
    noise: Noise,
    nz: [ColoredNoise; 6],
    mount_hp: Biquad,
    mc_lp: Biquad,
    mount_lp: Biquad,
    fan_rpm: f32,
    paths: Vec<Path>,
    x: Box<[[f32; B]; N_SRC]>,
    force: Box<[[f32; B]; N_SRC]>,
    tmp: [f32; B],
    out: Box<BlockOut>,
}

/// Bank index per source (sources without a structure use direct radiation).
const BANK_OF: [Option<usize>; N_SRC] = [None, None, None, Some(0), Some(1), Some(2), Some(3), Some(4), Some(5), None, Some(6), None, None, Some(7), Some(8), Some(9)];

impl Synth {
    pub fn new(params: Params) -> Self {
        let banks = vec![
            Bank::new(&BLOCK),
            Bank::new(&BLOCK),
            Bank::new(&HEAD),
            Bank::new(&HEAD),
            Bank::new(&HEAD),
            Bank::new(&COVER),
            Bank::new(&STARTER),
            Bank::new(&GEARBOX),
            Bank::new(&GEARBOX),
            Bank::new(&GEARBOX),
        ];
        let mut paths = Vec::with_capacity(3 * N_SRC);
        for row in &PATHS {
            for &cls in &CLASS {
                paths.push(Path::new(&row[cls]));
            }
        }
        Self {
            params,
            mix: Mix::default(),
            cfg: Config { piston_area_m2: 5.35e-3, bore_m: 0.0825, valve_mass_kg: [0.085, 0.078], exhaust_volume_m3: 1e-3, intake_volume_m3: 2.6e-3, compressor_tip_m: 0.0245 },
            started: false,
            t_cur: 0.0,
            blk_t0: 0.0,
            v_prev: [0.0; NCH],
            acc: [0.0; NCH],
            fill: 0,
            blk: Box::new([[0.0; B]; NCH]),
            crank_unw: 0.0,
            crank_prev: 0.0,
            chain_next: 0.0,
            st_angle: 0.0,
            st_tooth_next: 0.0,
            st_seg_next: 0.0,
            st_on: false,
            events: Vec::with_capacity(8192),
            slow: Slow { p_amb: 101_325.0, t_amb: 293.0, rho_amb: 1.2, t4_k: 293.0, t_cat_k: 293.0, ..Default::default() },
            slow_prev: None,
            synchro_w: 0.0,
            ex_duct: Duct::new(EX_FIR),
            in_duct: Duct::new(IN_FIR),
            ex_conv: Convolver::new(EX_FIR),
            in_conv: Convolver::new(IN_FIR),
            ir: vec![0.0; EX_FIR],
            ir_t: f64::NEG_INFINITY,
            ir_key: [0.0; 6],
            ex_rc: 0.0,
            in_rc: 0.0,
            banks,
            knock_ph: [0.0; 4],
            comb_hp_a: (1.0 / (1.0 + 2.0 * PI * params.comb_coupling_hz as f64 / FS)) as f32,
            comb_hp_x: 0.0,
            comb_hp_y: 0.0,
            ph: [0.0; 10],
            noise: Noise(0x9E37_79B9_7F4A_7C15),
            nz: [ColoredNoise::new(3000.0), ColoredNoise::new(2000.0), ColoredNoise::new(2000.0), ColoredNoise::new(1500.0), ColoredNoise::new(800.0), ColoredNoise::new(2500.0)],
            mount_hp: Biquad::highpass(15.0, 0.707),
            mc_lp: Biquad::lowpass(params.comp_flow_lp_hz as f64, 0.707),
            mount_lp: Biquad::lowpass(100.0, 0.707),
            fan_rpm: 0.0,
            paths,
            x: Box::new([[0.0; B]; N_SRC]),
            force: Box::new([[0.0; B]; N_SRC]),
            tmp: [0.0; B],
            out: Box::new(BlockOut { t0: 0.0, out: [[0.0; B]; N_LIST], level: [[0.0; N_SRC]; N_LIST] }),
        }
    }

    /// Feeds one record; `sink` receives every completed block.
    pub fn push(&mut self, r: &Record, sink: &mut impl FnMut(&BlockOut)) {
        match r {
            Record::None => {}
            Record::Config(c) => {
                self.cfg = *c;
                self.ir_t = f64::NEG_INFINITY;
                self.mount_lp = Biquad::lowpass(self.params.mount_lp_hz as f64, 0.707);
            }
            Record::Impact(i) => self.impact(i),
            Record::Slow(s) => self.on_slow(s),
            Record::Crank(f) => self.frame(f, sink),
        }
    }

    fn queue(&mut self, e: Ev) {
        if self.events.len() < self.events.capacity() && e.t.is_finite() && e.j.is_finite() {
            self.events.push(e);
        }
    }

    fn impact(&mut self, i: &Impact) {
        if !i.t_s.is_finite() || !i.mag.is_finite() {
            return;
        }
        let p = &self.params;
        let (src, j) = match i.kind {
            ImpactKind::InjectorOpen => (src::INJECTORS, p.inj_mass_kg * p.inj_open_ms),
            ImpactKind::InjectorClose => (src::INJECTORS, p.inj_mass_kg * p.inj_close_ms_10mpa * (i.mag / 1e7).max(0.0).sqrt()),
            ImpactKind::HpfpValveClose => (src::HPFP, p.hpfp_arm_kg * p.hpfp_arm_ms * i.mag / 13.5),
            ImpactKind::HpfpValveOpen => (src::HPFP, p.hpfp_plate_ns),
            ImpactKind::HpfpOutletOpen => (src::HPFP, p.hpfp_ball_ns_10mpa * (i.mag / 1e7).max(0.0).sqrt()),
            ImpactKind::ValveSeat => (src::VALVETRAIN, self.cfg.valve_mass_kg[i.sub.min(1) as usize] * i.mag * (1.0 + p.valve_restitution)),
        };
        let cyl = if matches!(i.kind, ImpactKind::InjectorOpen | ImpactKind::InjectorClose | ImpactKind::ValveSeat) { i.cyl } else { 4 };
        self.queue(Ev { t: i.t_s, src: src as u8, cyl, j });
    }

    fn on_slow(&mut self, s: &Slow) {
        if !s.t_s.is_finite() {
            return;
        }
        let p = self.params;
        if let Some(o) = self.slow_prev {
            // Dog engagement clunk: sleeve (+ fork) arrested by the dog-tooth back-taper.
            let new = s.engaged & !o.engaged;
            for gi in 0..8 {
                if new & (1 << gi) != 0 {
                    self.queue(Ev { t: s.t_s, src: src::GEARS as u8, cyl: 4, j: p.sleeve_kg * s.sleeve_ms[gi].max(o.sleeve_ms[gi]).max(0.02) * 1.5 });
                }
            }
            let dt = s.t_s - o.t_s;
            self.synchro_w = if dt > 0.0 { ((s.synchro_j - o.synchro_j) as f64 / dt).max(0.0) as f32 } else { 0.0 };
            if !self.synchro_w.is_finite() {
                self.synchro_w = 0.0;
            }
            if s.dog_health < o.dog_health - 1e-7 {
                self.queue(Ev { t: s.t_s, src: src::GEARS as u8, cyl: 4, j: p.dog_clash_ns * (o.dog_health - s.dog_health) });
            }
        }
        self.slow_prev = Some(*s);
        self.slow = *s;
    }

    fn frame(&mut self, f: &CrankFrame, sink: &mut impl FnMut(&BlockOut)) {
        if !f.t_s.is_finite() || !f.h.is_finite() || !f.crank_deg.is_finite() {
            return;
        }
        let v1 = channels(f);
        let t1 = f.t_s;
        let h = (f.h as f64).max(1e-9);
        let t0 = t1 - h;
        if !self.started || t0 < self.t_cur - 1e-3 || t0 > self.t_cur + 0.05 {
            // First frame, plant reset (time went back) or a gap: re-anchor the audio clock.
            let back = self.started && t0 < self.t_cur - 1e-3;
            let hi = if back { (self.t_cur - 1e-3).min(t1 + 1.0) } else { t1 + 1.0 };
            self.started = true;
            self.t_cur = t0;
            self.blk_t0 = t0 - self.fill as f64 * T;
            self.acc = [0.0; NCH];
            self.v_prev = v1;
            self.ir_t = f64::NEG_INFINITY;
            self.slow_prev = None;
            self.synchro_w = 0.0;
            self.events.retain(|e| e.t >= t0 && e.t <= hi);
            self.crank_prev = f.crank_deg;
            self.st_on = f.starter_on;
        }
        self.frame_events(f, t0, h);
        let v0 = self.v_prev;
        let mut a = t0.max(self.t_cur);
        while t1 >= self.t_cur + T {
            let b = self.t_cur + T;
            self.integrate(&v0, &v1, t0, h, a, b);
            for k in 0..NCH {
                self.blk[k][self.fill] = self.acc[k] * FS as f32;
            }
            self.acc = [0.0; NCH];
            self.fill += 1;
            self.t_cur = b;
            a = b;
            if self.fill == B {
                self.render();
                sink(&self.out);
                self.fill = 0;
                self.blk_t0 = self.t_cur;
            }
        }
        self.integrate(&v0, &v1, t0, h, a, t1);
        self.v_prev = v1;
    }

    #[inline]
    fn integrate(&mut self, v0: &[f32; NCH], v1: &[f32; NCH], t0: f64, h: f64, a: f64, b: f64) {
        if b <= a {
            return;
        }
        let w = (b - a) as f32;
        let frac = ((0.5 * (a + b) - t0) / h) as f32;
        for k in 0..NCH {
            self.acc[k] += w * (v0[k] + (v1[k] - v0[k]) * frac);
        }
    }

    /// Events implied by the crank and starter kinematics inside one frame (chain meshing, pinion meshing,
    /// commutation, pinion throw / retract).
    fn frame_events(&mut self, f: &CrankFrame, t0: f64, h: f64) {
        let p = self.params;
        let d = ((f.crank_deg - self.crank_prev) as f64 + 360.0).rem_euclid(720.0) - 360.0;
        self.crank_prev = f.crank_deg;
        let th0 = self.crank_unw;
        let th1 = th0 + d;
        self.crank_unw = th1;
        if d > 0.0 && p.chain_teeth > 0.0 {
            let pitch = 360.0 / p.chain_teeth as f64;
            if self.chain_next < th0 || self.chain_next > th0 + 2.0 * pitch {
                self.chain_next = (th0 / pitch).ceil() * pitch;
            }
            let w = f.omega.max(0.0);
            // Polygonal action: chordal speed fluctuation v·π/Z of the link meeting the sprocket.
            let v_rel = w * p.chain_radius_m * PI as f32 / p.chain_teeth;
            let j = p.chain_frac * p.chain_link_kg * v_rel * (1.0 + f.cam_nm.abs() / p.chain_tension_nm.max(1e-3));
            while self.chain_next <= th1 {
                let t = t0 + h * (self.chain_next - th0) / d;
                self.queue(Ev { t, src: src::CHAIN as u8, cyl: 4, j });
                self.chain_next += pitch;
            }
        }
        if f.starter_on != self.st_on {
            let j = if f.starter_on { p.st_engage_ns } else { p.st_release_ns };
            self.queue(Ev { t: t0, src: src::STARTER as u8, cyl: 4, j });
            self.st_on = f.starter_on;
        }
        let a0 = self.st_angle;
        let a1 = a0 + if f.starter_rad_s.is_finite() { f.starter_rad_s.max(0.0) as f64 } else { 0.0 } * h;
        self.st_angle = a1;
        if f.starter_on && a1 > a0 && p.st_planetary > 0.0 && p.st_pinion_teeth > 0.0 && p.st_segments > 0.0 {
            let tooth = 2.0 * PI * p.st_planetary as f64 / p.st_pinion_teeth as f64; // armature angle per pinion tooth
            if self.st_tooth_next < a0 || self.st_tooth_next > a0 + 2.0 * tooth {
                self.st_tooth_next = (a0 / tooth).ceil() * tooth;
            }
            let f_mesh = (f.starter_rad_s / tooth as f32).max(20.0);
            let force = p.st_k_nm_a * f.starter_a * p.st_planetary / p.st_pinion_r_m.max(1e-4);
            let j_mesh = p.st_te_frac * force / f_mesh + if f.starter_engaged { 0.0 } else { 2e-4 };
            while self.st_tooth_next <= a1 {
                let t = t0 + h * (self.st_tooth_next - a0) / (a1 - a0);
                self.queue(Ev { t, src: src::STARTER as u8, cyl: 4, j: j_mesh });
                self.st_tooth_next += tooth;
            }
            let seg = 2.0 * PI / p.st_segments as f64;
            if self.st_seg_next < a0 || self.st_seg_next > a0 + 2.0 * seg {
                self.st_seg_next = (a0 / seg).ceil() * seg;
            }
            while self.st_seg_next <= a1 {
                let t = t0 + h * (self.st_seg_next - a0) / (a1 - a0);
                self.queue(Ev { t, src: src::STARTER as u8, cyl: 4, j: p.st_brush_ns_a * f.starter_a });
                self.st_seg_next += seg;
            }
        }
    }

    fn mean(&self, ch: usize) -> f32 {
        self.blk[ch].iter().sum::<f32>() / B as f32
    }

    /// Rebuilds the duct filters for the current gas state (every 0.1 s, only if it moved).
    fn update_ducts(&mut self) {
        let s = self.slow;
        let p4 = self.mean(CH_P4).max(80e3);
        let r_filter = 2.0 * simemu_physics::engine::AIR_FILTER_K as f32 * s.air_kg_s.max(0.0) * s.rho_amb;
        let (p_em, t_em) = (self.mean(CH_PEM).max(5e4), self.mean(CH_TEM).max(250.0));
        let q_em = self.mean(CH_MOUT).max(1e-5) / (p_em / (288.0 * t_em));
        let r_turb = (2.0 * (p_em - p4).max(50.0) / q_em).clamp(1e3, 1e9);
        let key = [s.t4_k, s.t_cat_k, p4 / 1000.0, r_filter / 100.0, r_turb.ln() * 10.0, s.exhaust_kg_s * 1000.0];
        let moved = key.iter().zip(&self.ir_key).any(|(a, b)| (a - b).abs() > 0.03 * b.abs().max(10.0));
        let first = self.ir_t == f64::NEG_INFINITY;
        if !first && (!moved || self.blk_t0 - self.ir_t < 0.1) {
            return;
        }
        self.ir_key = key;
        self.ir_t = self.blk_t0;
        let c_em = self.cfg.exhaust_volume_m3 as f64 / (1.33 * p_em as f64);
        self.params.exhaust.configure(&mut self.ex_duct, s.t4_k as f64, s.t_cat_k as f64, s.t_amb as f64, p4 as f64, s.rho_amb as f64, s.exhaust_kg_s as f64, r_turb as f64, c_em);
        self.ex_duct.build_ir(FS, &mut self.ir[..EX_FIR]);
        self.ex_conv.set_ir(&self.ir[..EX_FIR]);
        self.params.intake.configure(&mut self.in_duct, s.t_amb as f64, s.p_amb as f64, r_filter as f64);
        self.in_duct.build_ir(FS, &mut self.ir[..IN_FIR]);
        self.in_conv.set_ir(&self.ir[..IN_FIR]);
    }

    fn render(&mut self) {
        let p = self.params;
        let s = self.slow;
        let w = self.mix.cyl;
        let all = w.iter().all(|&x| x >= 0.999);
        self.update_ducts();
        for f in self.force.iter_mut() {
            f.fill(0.0);
        }
        for x in self.x.iter_mut() {
            x.fill(0.0);
        }
        let rho0 = s.rho_amb.max(0.5);

        // ---- Exhaust: turbine-outlet flow of the plant (or masked port flows through a linear manifold) ----
        {
            let (p_em, t_em, p4) = (self.mean(CH_PEM).max(5e4), self.mean(CH_TEM).max(250.0), self.mean(CH_P4).max(5e4));
            let rho4 = p4 / (288.0 * s.t4_k.max(250.0));
            let rho_em = p_em / (288.0 * t_em);
            let q_out = self.mean(CH_MOUT).max(1e-4) / rho_em;
            let tau = (2.0 * (p_em - p4).max(100.0) / q_out * self.cfg.exhaust_volume_m3 / (1.33 * p_em)).clamp(1e-5, 0.05);
            let k = 1.0 - (-(T as f32) / tau).exp();
            for i in 0..B {
                let q = if all {
                    self.blk[CH_MOUT][i] / rho4
                } else {
                    let x = (0..4).map(|c| w[c] * self.blk[CH_EX + c][i]).sum::<f32>() / rho_em;
                    self.ex_rc += (x - self.ex_rc) * k;
                    self.ex_rc * rho_em / rho4
                };
                self.tmp[i] = q;
            }
            let (tmp, xs) = (&self.tmp, &mut self.x[src::EXHAUST]);
            self.ex_conv.process(tmp, xs);
        }

        // ---- Intake orifice: compressor-inlet flow of the plant (or masked valve flows through the plenum) ----
        {
            let (p_im, t_im, p_b) = (self.mean(CH_PIM).max(5e3), self.mean(CH_TIM).max(230.0), self.mean(CH_PB).max(5e4));
            let rho_im = p_im / (287.0 * t_im);
            let q_thr = self.mean(CH_MTHR).max(1e-4) / rho_im;
            let tau = (2.0 * (p_b - p_im).max(100.0) / q_thr * self.cfg.intake_volume_m3 / (1.4 * p_im)).clamp(1e-5, 0.2);
            let k = 1.0 - (-(T as f32) / tau).exp();
            for i in 0..B {
                self.tmp[i] = if all {
                    -self.mc_lp.run(self.blk[CH_MC][i]) / rho0
                } else {
                    let x = (0..4).map(|c| w[c] * self.blk[CH_IN + c][i]).sum::<f32>() / rho_im;
                    self.in_rc += (x - self.in_rc) * k;
                    -self.in_rc
                };
            }
            let (tmp, xs) = (&self.tmp, &mut self.x[src::INTAKE]);
            self.in_conv.process(tmp, xs);
            // Throttle jet (hiss on overrun / idle): Lighthill, radiated from the throttle body.
            let (pj, fj) = jet(p.lighthill_k, self.mean(CH_MTHR), p_b - p_im, p_b / (287.0 * t_im), (1.4 * 287.0 * t_im).sqrt(), rho0);
            let nz = &mut self.nz[0];
            nz.tune(fj as f64);
            let g = pj * wall(p.duct_wall_kg_m2, fj, rho0) * nz.norm();
            for i in 0..B {
                self.x[src::INTAKE][i] += g * nz.bq.run(self.noise.next());
            }
        }

        // ---- Turbocharger: blade-pass and shaft tones, broadband, diverter-valve jet ----
        {
            let wt = self.mean(CH_TURBO).max(0.0);
            let u = wt * self.cfg.compressor_tip_m;
            let un = u / 300.0;
            let f_sh = wt as f64 / (2.0 * PI);
            let prc = self.mean(CH_PRC);
            let surge = 1.0 + 4.0 * ((prc - 0.85) / 0.15).clamp(0.0, 1.0);
            let a_bpf = p.comp_bpf_pa_300 * un.powi(4);
            let a_bpf2 = 0.5 * a_bpf;
            // Unbalance force U·ω² on a mass-controlled bearing housing → radiated by the compressor housing.
            let v_h = p.turbo_unbalance_kgm * wt * wt / (p.turbo_housing_kg * wt.max(1.0));
            let a_1x = 1.2 * 343.0 * (0.05f32 / (4.0 * PI as f32)).sqrt() * v_h;
            let fr = [f_sh * p.comp_blades as f64, f_sh * 2.0 * p.comp_blades as f64, f_sh];
            let amps = [a_bpf, a_bpf2, a_1x];
            for (k, (&f, &a)) in fr.iter().zip(&amps).enumerate() {
                let fade = (((0.45 * FS - f) / 2000.0).clamp(0.0, 1.0)) as f32;
                if a * fade <= 0.0 {
                    continue;
                }
                let dph = 2.0 * PI * f / FS;
                for i in 0..B {
                    self.ph[k] += dph;
                    self.x[src::TURBO][i] += a * fade * (self.ph[k].sin() as f32) * std::f32::consts::SQRT_2;
                }
                self.ph[k] %= 2.0 * PI;
            }
            let bb = p.comp_bb_pa_300 * un.powi(3) * surge;
            let nz = &mut self.nz[1];
            nz.tune((0.25 * f_sh * p.comp_blades as f64).clamp(500.0, 8000.0));
            let g = bb * nz.norm();
            for i in 0..B {
                self.x[src::TURBO][i] += g * nz.bq.run(self.noise.next());
            }
            let t_b = 300.0f32;
            let (pj, fj) = jet(p.lighthill_k, self.mean(CH_MDV), self.mean(CH_PB) - s.p_amb, self.mean(CH_PB) / (287.0 * t_b), (1.4f32 * 287.0 * t_b).sqrt(), rho0);
            let nz = &mut self.nz[2];
            nz.tune(fj as f64);
            let g = pj * wall(p.duct_wall_kg_m2, fj, rho0) * nz.norm();
            for i in 0..B {
                self.x[src::TURBO][i] += g * nz.bq.run(self.noise.next());
            }
        }

        // ---- Structure-borne engine sources: forces into the modal banks ----
        let a_p = self.cfg.piston_area_m2;
        let bore = self.cfg.bore_m.max(0.01);
        for i in 0..B {
            let mut fc = 0.0;
            let mut fk = 0.0;
            let mut fv = 0.0;
            for (c, (&wc, kph)) in w.iter().zip(&mut self.knock_ph).enumerate() {
                fc += wc * (self.blk[CH_P + c][i] - s.p_amb);
                let t = self.blk[CH_T + c][i].max(300.0);
                let f1 = 1.841 * (p.knock_gamma.max(1.0) * 288.0 * t).sqrt() / (PI as f32 * bore);
                if (f1 as f64) < 0.45 * FS {
                    *kph = (*kph + 2.0 * PI * f1 as f64 / FS) % (2.0 * PI);
                    fk += wc * self.blk[CH_K + c][i] * kph.sin() as f32;
                }
                fv += wc * self.blk[CH_VN + c][i];
            }
            let fcx = fc * a_p;
            // First-order high-pass (coupling of the internally reacted gas force).
            let hp = self.comb_hp_a * (self.comb_hp_y + fcx - self.comb_hp_x);
            self.comb_hp_x = fcx;
            self.comb_hp_y = hp;
            self.force[src::COMBUSTION][i] = p.comb_coupling * hp;
            self.force[src::KNOCK][i] = fk * a_p * p.knock_force_frac;
            self.force[src::VALVETRAIN][i] = fv * p.valve_force_frac;
            self.force[src::HPFP][i] = p.hpfp_force_frac * self.blk[CH_HPFP][i] / p.hpfp_lever_m.max(1e-4);
            self.force[src::CHAIN][i] = self.blk[CH_CAM][i] / p.chain_radius_m.max(1e-4) * p.chain_frac;
        }
        // Clutch slip friction (broadband force ∝ √P into the bell housing).
        {
            let pw: f32 = (0..2).map(|k| (s.clutch_slip[k] * s.clutch_nm[k]).abs()).sum();
            let g = p.clutch_n_per_sqrt_w * pw.sqrt() * self.nz[3].norm();
            for i in 0..B {
                self.force[src::CLUTCH][i] += g * self.nz[3].bq.run(self.noise.next());
            }
        }
        // Synchroniser cone friction (grinding) from the rate of synchroniser work.
        if self.synchro_w > 0.0 {
            let g = p.synchro_n_per_sqrt_w * self.synchro_w.sqrt() * self.nz[5].norm();
            for i in 0..B {
                self.force[src::GEARS][i] += g * self.nz[5].bq.run(self.noise.next());
            }
        }
        // Gear mesh: transmission-error force at the mesh frequency and its 2nd harmonic.
        for sh in 0..2 {
            let z = s.mesh_teeth[sh];
            if z <= 0.0 {
                continue;
            }
            let f = (z * s.shaft_rad_s[sh].abs()) as f64 / (2.0 * PI);
            if f >= 0.45 * FS {
                continue;
            }
            let a = p.mesh_n_per_nm * s.mesh_nm[sh].abs();
            let h2 = if 2.0 * f < 0.45 * FS { 0.4f32 } else { 0.0 };
            let dph = 2.0 * PI * f / FS;
            for f_out in &mut self.force[src::GEARS] {
                self.ph[3 + sh] += dph;
                *f_out += a * (self.ph[3 + sh].sin() as f32 + h2 * (2.0 * self.ph[3 + sh]).sin() as f32);
            }
            self.ph[3 + sh] %= 2.0 * PI;
        }
        // Mechatronic gear pump: pressure ripple at the tooth frequency + motor commutation.
        {
            let teeth = p.pump_teeth.max(1.0);
            let f_p = (teeth * s.pump_rad_s.max(0.0)) as f64 / (2.0 * PI);
            let f_m = f_p * 12.0 / teeth as f64;
            let a = if f_p < 0.45 * FS { p.pump_n_per_pa * s.supply_pa.max(0.0) * (s.pump_rad_s > 1.0) as u8 as f32 } else { 0.0 };
            let a_m = if f_m < 0.45 * FS { p.pump_n_per_a * s.pump_a.max(0.0) } else { 0.0 };
            let (d1, d2) = (if f_p < 0.45 * FS { 2.0 * PI * f_p / FS } else { 0.0 }, if f_m < 0.45 * FS { 2.0 * PI * f_m / FS } else { 0.0 });
            for f_out in &mut self.force[src::PUMP] {
                self.ph[5] += d1;
                self.ph[6] += d2;
                *f_out += a * self.ph[5].sin() as f32 + a_m * self.ph[6].sin() as f32;
            }
            self.ph[5] %= 2.0 * PI;
            self.ph[6] %= 2.0 * PI;
        }
        // Impulses.
        let t_end = self.blk_t0 + B as f64 * T;
        let (t0b, ev) = (self.blk_t0, &mut self.events);
        let force = &mut self.force;
        ev.retain(|e| {
            if e.t >= t_end {
                return true;
            }
            let wgt = if (e.cyl as usize) < 4 { w[e.cyl as usize] } else { 1.0 };
            let x = (((e.t - t0b) * FS).max(0.0)) as f32;
            let i = (x as usize).min(B - 1);
            let fr = (x - i as f32).min(1.0);
            let a = e.j * wgt * FS as f32;
            let f = &mut force[(e.src as usize).min(N_SRC - 1)];
            f[i] += a * (1.0 - fr);
            if i + 1 < B {
                f[i + 1] += a * fr;
            } else {
                f[i] += a * fr;
            }
            false
        });
        for ((&bank, f), x) in BANK_OF.iter().zip(self.force.iter()).zip(self.x.iter_mut()) {
            if let Some(b) = bank {
                self.banks[b].run(f, x);
            }
        }

        // ---- Mounts: block roll torque → body → cabin pressure (low frequency) ----
        for i in 0..B {
            let tq = self.mount_lp.run(self.mount_hp.run(self.blk[CH_TGAS][i]));
            self.x[src::MOUNTS][i] = p.mount_pa_per_nm * tq;
        }

        // ---- Alternator magnetic orders 36 / 72 ----
        {
            let f = s.alt_rad_s.max(0.0) as f64 / (2.0 * PI);
            let a = p.alt_pa_per_a * s.alt_a.max(0.0);
            for (k, ord) in [(7usize, 36.0), (8, 72.0)] {
                let fq = f * ord;
                if fq > 0.45 * FS || a <= 0.0 {
                    continue;
                }
                let dph = 2.0 * PI * fq / FS;
                let amp = if ord > 40.0 { 0.5 * a } else { a };
                for i in 0..B {
                    self.ph[k] += dph;
                    self.x[src::ALTERNATOR][i] += amp * self.ph[k].sin() as f32;
                }
                self.ph[k] %= 2.0 * PI;
            }
        }

        // ---- Radiator fan: motor lag, blade-pass tone, Sharland broadband ----
        {
            let target = s.fan_duty * p.fan_max_rpm;
            self.fan_rpm += (target - self.fan_rpm) * (1.0 - (-(B as f64 * T) as f32 / p.fan_tau_s.max(1e-3)).exp());
            let u = self.fan_rpm / 60.0 * PI as f32 * p.fan_dia_m;
            let area = PI as f32 / 4.0 * p.fan_dia_m * p.fan_dia_m;
            let wpow = p.fan_sharland_k * rho0 * u.powi(6) * area / C0.powi(3);
            let prms = (rho0 * C0 * wpow / (4.0 * PI as f32)).sqrt();
            let f_bpf = (self.fan_rpm / 60.0 * p.fan_blades) as f64;
            let nz = &mut self.nz[4];
            nz.tune((2.0 * f_bpf).max(200.0));
            let g = prms * nz.norm();
            let (dph, tone) = if f_bpf < 0.45 * FS { (2.0 * PI * f_bpf / FS, 0.7 * prms) } else { (0.0, 0.0) };
            for out_s in &mut self.x[src::FAN] {
                self.ph[9] += dph;
                *out_s = g * nz.bq.run(self.noise.next()) + tone * self.ph[9].sin() as f32;
            }
            self.ph[9] %= 2.0 * PI;
        }

        // ---- Listener mix ----
        let o = &mut *self.out;
        o.t0 = self.blk_t0;
        for l in 0..N_LIST {
            o.out[l].fill(0.0);
            o.level[l].fill(0.0);
        }
        for sx in 0..N_SRC {
            let g = self.mix.gain[sx] * self.mix.master;
            let x = &self.x[sx];
            let mut e = 0.0;
            for (o3, &xi) in o.out[3].iter_mut().zip(x.iter()) {
                *o3 += g * xi;
                e += xi * xi;
            }
            o.level[3][sx] = e / B as f32;
            for l in 0..3 {
                let path = &mut self.paths[l * N_SRC + sx];
                let mut e = 0.0;
                for (ol, &xi) in o.out[l].iter_mut().zip(x.iter()) {
                    let y = path.run(xi);
                    e += y * y;
                    *ol += g * y;
                }
                o.level[l][sx] = e / B as f32;
            }
        }
    }
}
