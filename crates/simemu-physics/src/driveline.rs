//! Dual-clutch gear train, differential, driveshafts, wheels and longitudinal vehicle.
//!
//! The gearbox only reacts to [`TcuOutputs`] (solenoid PWM duties, pump motor) through the
//! [`Mechatronic`] hydraulics. Which gear is engaged is a *consequence* of sleeve positions, which
//! follow the actuator pistons against detents, synchroniser blocker rings and dog teeth.
//!
//! Rotational bodies (explicit symplectic Euler at the 0.2 ms plant sub-step):
//! DMF secondary + clutch drum (owned by the engine model) — clutch K1/K2 — input shafts 1/2 —
//! engaged gear meshes (torsional spring + backlash per gear, input-referred) — output shafts +
//! final drive + differential carrier (one body) — driveshafts (spring + backlash) — wheels —
//! tyre carcass (relaxation) — vehicle body (or chassis-dyno rollers). AWD: Haldex coupling
//! between the carrier and the rear axle.
//!
//! Synchroniser: cone torque T = n·μ·F_ax·R_c/sin α between idler and output shaft (reaction
//! applied to both sides); the blocker ring prevents sleeve travel while slipping only if
//! n·μ·R_c/sin α ≥ R_b(1 − μ_b tan β)/(μ_b + tan β) (R. J. Socin, L. K. Walters, SAE 680008;
//! D. Häggström, PhD thesis KTH 2016 "Synchronizer mechanism"; Naunheimer et al. 2011, ch. 9).
//! Dog clash when the sleeve meets the dog teeth with speed difference. Mesh losses: load-dependent
//! sliding loss per mesh (N. E. Anderson, S. H. Loewenthal, NASA TP-1622 / J. Mech. Des. 104
//! (1982) 767) + churning of the dipping ring gear (C. Changenet, P. Velex, J. Mech. Des. 129
//! (2007) 128–133) + rolling bearings (SKF/Palmgren M0 = 1e-7·f0·(ν·n)^(2/3)·d_m³).

use crate::clutch::{ClutchEnv, ClutchHw, ClutchPack};
use crate::driveline_failure::{self as fail, Damage, GearGeometry, ShaftHw};
use crate::hydraulics::Fluid;
pub use crate::mechatronic::TcuOutputs;
use crate::mechatronic::{MechLoads, Mechatronic};
use crate::profiles::DrivetrainProfile;
use crate::telemetry::PlantTelemetry;
use crate::vehicle::{AxleTyres, Body, Brakes, ChassisDynoHw, Haldex, Rollers, G};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

pub const REVERSE: u8 = 0xFF;
const LOCK_TOL_RAD_S: f64 = 0.5;
/// Idler/hub speed difference below which the blocker ring indexes, rad/s.
const SYNC_INDEX_RAD_S: f64 = 1.0;
/// Speed difference above which dog chamfers bounce instead of meshing, rad/s.
const DOG_CLASH_RAD_S: f64 = 6.0;
/// Axial force that rams dogs through regardless of speed difference, N.
const DOG_FORCE_THROUGH_N: f64 = 2500.0;
const FORK_DETENT_N: f64 = 60.0;

pub fn gear_index(g: u8) -> Option<usize> {
    match g {
        1..=7 => Some(g as usize - 1),
        REVERSE => Some(7),
        _ => None,
    }
}
fn gear_code(i: usize) -> u8 {
    if i == 7 {
        REVERSE
    } else {
        i as u8 + 1
    }
}
fn sgn(x: f64) -> f64 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}
fn dead_zone(x: f64, lash: f64) -> f64 {
    if x > lash {
        x - lash
    } else if x < -lash {
        x + lash
    } else {
        0.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum Selector {
    #[default]
    P,
    R,
    N,
    D,
    S,
    M,
}

#[derive(Debug, Clone, Serialize)]
pub struct SynchroHw {
    /// Number of friction cones per gear (1..7, R).
    pub cones: [f64; 8],
    pub cone_radius_m: f64,
    pub sin_cone_angle: f64,
    pub mu_cone: f64,
    /// Blocker indexing torque per axial force R_b(1 − μ_b tan β)/(μ_b + tan β), m.
    pub index_ratio_m: f64,
    pub ring_c_j_k: f64,
    pub ring_g_w_k: f64,
    /// Friction energy that wears a ring to its limit, J.
    pub life_j: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct GearTrainHw {
    /// Input shaft + fixed gears + always-meshing idlers (reflected), without the clutch hub.
    pub shaft_inertia_kgm2: [f64; 2],
    /// Output shafts + final drive + differential, referred to the carrier.
    pub output_inertia_kgm2: f64,
    /// Engaged path stiffness (input shaft torsion + meshes), referred to the input shaft.
    pub mesh_stiffness_nm_rad: f64,
    /// Half backlash of an engaged path at the input shaft, rad.
    pub mesh_lash_rad: f64,
    pub mesh_damping_ratio: f64,
    /// Efficiency of gear mesh + final drive mesh under load.
    pub mesh_efficiency: f64,
    /// Both driveshafts with CV joints, at the wheel.
    pub halfshaft_stiffness_nm_rad: f64,
    pub halfshaft_lash_rad: f64,
    pub gears: [GearGeometry; 8],
    pub input_shafts: [ShaftHw; 2],
    /// One driveshaft (per side).
    pub halfshaft: ShaftHw,
    pub cv_joint_strength_nm: f64,
    pub synchro: SynchroHw,
    pub dog_radius_m: f64,
    pub dog_mu: f64,
    pub pawl_stiffness_nm_rad: f64,
    pub pawl_lash_rad: f64,
    /// Pawl tooth strength, in carrier (axle) torque, Nm.
    pub pawl_strength_nm: f64,
    /// Above this carrier speed the pawl cam ejects the pawl (ratchets), rad/s.
    pub pawl_engage_max_rad_s: f64,
    pub oil: Fluid,
    pub ring_radius_m: f64,
    pub ring_immersed_m2: f64,
    pub ring_face_m: f64,
    pub oil_volume_m3: f64,
    pub immersion_ratio: f64,
    pub case_c_j_k: f64,
    pub case_g_w_k: f64,
    pub case_g_per_ms: f64,
    /// Oil-to-coolant heat exchanger (wet units), W/K.
    pub cooler_w_k: f64,
}

impl GearTrainHw {
    /// DQ200 (0CW). Geometry/stiffness UNVERIFIED engineering estimates.
    pub fn dq200() -> Self {
        let g = |m: f64, b: f64, a: f64| GearGeometry { module_m: m * 1e-3, face_width_m: b * 1e-3, center_distance_m: a };
        Self {
            shaft_inertia_kgm2: [0.011, 0.009],
            output_inertia_kgm2: 0.25,
            mesh_stiffness_nm_rad: 9000.0,
            mesh_lash_rad: 0.004,
            mesh_damping_ratio: 0.06,
            mesh_efficiency: 0.975,
            halfshaft_stiffness_nm_rad: 14000.0,
            halfshaft_lash_rad: 0.004,
            gears: [g(2.25, 25.0, 0.068), g(2.25, 22.0, 0.068), g(2.0, 20.0, 0.068), g(2.0, 18.0, 0.068), g(1.75, 17.0, 0.060), g(1.75, 16.0, 0.060), g(1.75, 15.0, 0.060), g(2.0, 18.0, 0.060)],
            input_shafts: [ShaftHw { d_outer_m: 0.026, d_inner_m: 0.0, tau_ultimate_mpa: 900.0 }, ShaftHw { d_outer_m: 0.042, d_inner_m: 0.032, tau_ultimate_mpa: 900.0 }],
            halfshaft: ShaftHw { d_outer_m: 0.026, d_inner_m: 0.0, tau_ultimate_mpa: 1100.0 },
            cv_joint_strength_nm: 3500.0,
            synchro: SynchroHw {
                cones: [3.0, 3.0, 2.0, 1.0, 1.0, 1.0, 1.0, 2.0],
                cone_radius_m: 0.035,
                sin_cone_angle: (6.5f64).to_radians().sin(),
                mu_cone: 0.10,
                index_ratio_m: 0.018,
                ring_c_j_k: 25.0,
                ring_g_w_k: 0.6,
                life_j: 50e6,
            },
            dog_radius_m: 0.045,
            dog_mu: 0.15,
            pawl_stiffness_nm_rad: 60_000.0,
            pawl_lash_rad: 0.012,
            pawl_strength_nm: 6000.0,
            pawl_engage_max_rad_s: 4.5,
            oil: Fluid::GEAR_OIL,
            ring_radius_m: 0.09,
            ring_immersed_m2: 0.012,
            ring_face_m: 0.025,
            oil_volume_m3: 1.7e-3,
            immersion_ratio: 0.25,
            case_c_j_k: 25_000.0,
            case_g_w_k: 10.0,
            case_g_per_ms: 1.5,
            cooler_w_k: 0.0,
        }
    }
    /// DQ250 (02E): heavier gear set in ATF, ATF cooler on the engine coolant. UNVERIFIED.
    pub fn dq250() -> Self {
        let g = |m: f64, b: f64, a: f64| GearGeometry { module_m: m * 1e-3, face_width_m: b * 1e-3, center_distance_m: a };
        Self {
            shaft_inertia_kgm2: [0.010, 0.008],
            output_inertia_kgm2: 0.30,
            mesh_stiffness_nm_rad: 11000.0,
            gears: [g(2.5, 26.0, 0.075), g(2.5, 24.0, 0.075), g(2.25, 22.0, 0.075), g(2.25, 20.0, 0.075), g(2.0, 19.0, 0.066), g(2.0, 18.0, 0.066), g(2.0, 17.0, 0.066), g(2.25, 20.0, 0.066)],
            input_shafts: [ShaftHw { d_outer_m: 0.030, d_inner_m: 0.0, tau_ultimate_mpa: 900.0 }, ShaftHw { d_outer_m: 0.048, d_inner_m: 0.036, tau_ultimate_mpa: 900.0 }],
            halfshaft: ShaftHw { d_outer_m: 0.028, d_inner_m: 0.0, tau_ultimate_mpa: 1100.0 },
            cv_joint_strength_nm: 4200.0,
            oil: Fluid::ATF,
            ring_immersed_m2: 0.02,
            oil_volume_m3: 6.5e-3,
            immersion_ratio: 0.35,
            case_c_j_k: 32_000.0,
            cooler_w_k: 60.0,
            ..Self::dq200()
        }
    }
    pub fn dq381() -> Self {
        Self { case_c_j_k: 34_000.0, ..Self::dq250() }
    }
}

/// Environment of one plant sub-step.
#[derive(Debug, Clone, Copy)]
pub struct DriveEnv {
    pub battery_v: f64,
    pub ambient_c: f64,
    pub air_density: f64,
    pub coolant_c: f64,
    pub selector: Selector,
    pub brake_pct: f64,
    /// Haldex pump duty from the AWD controller (another ECU); `None` = not driven.
    pub awd_pump_duty: Option<f64>,
    /// Chassis-dyno rollers (hardware, absorber target km/h; ≤ 0 = inertia run).
    pub rollers: Option<(ChassisDynoHw, f64)>,
}

impl Default for DriveEnv {
    fn default() -> Self {
        Self { battery_v: 13.5, ambient_c: 20.0, air_density: 1.2, coolant_c: 20.0, selector: Selector::N, brake_pct: 0.0, awd_pump_duty: None, rollers: None }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct DrivelineState {
    pub mech: Mechatronic,
    pub clutches: ClutchPack,
    pub clutch_torque_nm: [f64; 2],
    pub clutch_capacity_nm: [f64; 2],
    pub clutch_slip_rad_s: [f64; 2],
    pub clutch_locked: [bool; 2],
    pub slip_angle: [f64; 2],
    /// Sleeve positions (shift-fork travel sensors), mm.
    pub fork_mm: [f64; 4],
    pub shaft_omega: [f64; 2],
    /// Differential carrier speed, rad/s.
    pub carrier_omega: f64,
    pub gear_engaged: [bool; 8],
    pub gear_twist: [f64; 8],
    /// Torque through each engaged mesh at the input shaft, Nm.
    pub gear_torque_nm: [f64; 8],
    pub gear_damage: [Damage; 8],
    pub gear_pitting: [Damage; 8],
    pub dog_health: [f64; 8],
    pub synchro_energy_j: [f64; 8],
    pub synchro_c: [f64; 8],
    pub shaft_damage: [Damage; 2],
    pub driveshaft_damage: Damage,
    pub halfshaft_twist: f64,
    pub halfshaft_torque_nm: f64,
    pub pawl_engaged: bool,
    pub pawl_twist: f64,
    pub pawl_torque_nm: f64,
    pub pawl_damage: Damage,
    /// Front / rear axle wheel speeds, rad/s.
    pub wheel_omega: [f64; 2],
    pub tyres: [AxleTyres; 2],
    pub brakes: Brakes,
    pub body: Body,
    pub vehicle_speed_ms: f64,
    /// Cooling air speed over the car: the road speed, or the road-speed fan of a road-load dyno cell.
    pub airflow_ms: f64,
    pub accel_ms2: f64,
    pub road_grade_pct: f64,
    pub rollers: Rollers,
    pub haldex: Haldex,
    /// Gearbox case / gear-oil (wet: ATF sump) temperature.
    pub case_c: f64,
    pub gear_loss_w: f64,
    // per-step cache
    omega_drive: f64,
    t_ext: [f64; 2],
    t_carrier_gears: f64,
    sync_torque_in: [f64; 8],
    mech_heat_w: f64,
}

impl DrivelineState {
    pub fn new(d: &DrivetrainProfile, t: f64) -> Self {
        Self {
            mech: Mechatronic::new(&d.mechatronic, t),
            clutches: ClutchPack::new(&d.clutch, t),
            clutch_torque_nm: [0.0; 2],
            clutch_capacity_nm: [0.0; 2],
            clutch_slip_rad_s: [0.0; 2],
            clutch_locked: [false; 2],
            slip_angle: [0.0; 2],
            fork_mm: [0.0; 4],
            shaft_omega: [0.0; 2],
            carrier_omega: 0.0,
            gear_engaged: [false; 8],
            gear_twist: [0.0; 8],
            gear_torque_nm: [0.0; 8],
            gear_damage: [Damage::default(); 8],
            gear_pitting: [Damage::default(); 8],
            dog_health: [1.0; 8],
            synchro_energy_j: [0.0; 8],
            synchro_c: [t; 8],
            shaft_damage: [Damage::default(); 2],
            driveshaft_damage: Damage::default(),
            halfshaft_twist: 0.0,
            halfshaft_torque_nm: 0.0,
            pawl_engaged: false,
            pawl_twist: 0.0,
            pawl_torque_nm: 0.0,
            pawl_damage: Damage::default(),
            wheel_omega: [0.0; 2],
            tyres: [AxleTyres::new(t), AxleTyres::new(t)],
            brakes: Brakes::new(t),
            body: Body::new(&d.body),
            vehicle_speed_ms: 0.0,
            airflow_ms: 0.0,
            accel_ms2: 0.0,
            road_grade_pct: 0.0,
            rollers: Rollers::default(),
            haldex: Haldex::new(t),
            case_c: t,
            gear_loss_w: 0.0,
            omega_drive: 0.0,
            t_ext: [0.0; 2],
            t_carrier_gears: 0.0,
            sync_torque_in: [0.0; 8],
            mech_heat_w: 0.0,
        }
    }

    /// Inertia of the clutch drum / central plate that rides on the DMF secondary, kg·m².
    pub fn drive_inertia(d: &DrivetrainProfile) -> f64 {
        d.clutch.drive_inertia()
    }

    /// Car rolling at `v` m/s with every rotating part synchronous (test / scenario helper).
    pub fn set_rolling(&mut self, d: &DrivetrainProfile, v: f64) {
        self.vehicle_speed_ms = v;
        let w = v / d.tyre.radius_m;
        self.wheel_omega = [w, w];
        self.carrier_omega = w;
        for gi in 0..8 {
            if self.gear_engaged[gi] {
                if let Some((_, _, s)) = d.gear_fork(gear_code(gi)) {
                    self.shaft_omega[s] = w * d.total_ratio(gear_code(gi));
                }
            }
        }
    }

    /// Places sleeve `fork` fully into gear `g` (scenario helper).
    pub fn force_engage(&mut self, d: &DrivetrainProfile, g: u8) {
        if let (Some((f, side, _)), Some(gi)) = (d.gear_fork(g), gear_index(g)) {
            self.fork_mm[f] = side * d.fork_full_mm;
            self.gear_engaged[gi] = true;
            self.gear_twist[gi] = 0.0;
        }
    }

    fn clutch_env(&self, d: &DrivetrainProfile, env: &DriveEnv, omega_drum: f64) -> ClutchEnv {
        ClutchEnv {
            ambient_c: env.ambient_c,
            vehicle_speed_ms: self.vehicle_speed_ms,
            omega_drum,
            omega_hub: self.shaft_omega,
            oil_c: if matches!(d.clutch, ClutchHw::Wet(_)) { self.case_c } else { self.mech.oil_c },
            atf_degradation: self.mech.atf_degradation,
            cool_flow_m3_s: self.mech.cool_flow_m3_s,
            balance_fill: self.mech.balance_fill,
        }
    }

    /// Gear dog-engaged on input shaft `s`, if any (the first one if several).
    pub fn engaged_on_shaft(&self, d: &DrivetrainProfile, s: usize) -> Option<u8> {
        (0..8).find(|&gi| self.gear_engaged[gi] && !self.gear_damage[gi].broken && d.gear_fork(gear_code(gi)).map(|x| x.2) == Some(s)).map(gear_code)
    }

    /// Gear currently transmitting the clutch torque, for display.
    pub fn driving_gear(&self, d: &DrivetrainProfile) -> Option<u8> {
        let k = if self.clutch_torque_nm[0].abs() >= self.clutch_torque_nm[1].abs() { 0 } else { 1 };
        if self.clutch_torque_nm[k].abs() < 1.0 {
            return None;
        }
        self.engaged_on_shaft(d, k)
    }

    /// Speed difference idler − output shaft referred to the input shaft (= twist rate of gear g).
    fn gear_slip(&self, d: &DrivetrainProfile, gi: usize) -> f64 {
        let g = gear_code(gi);
        let s = d.gear_fork(g).map(|x| x.2).unwrap_or(0);
        self.shaft_omega[s] - d.total_ratio(g) * self.carrier_omega
    }

    fn synchro_mu(&self, d: &DrivetrainProfile, gi: usize) -> f64 {
        let h = &d.train.synchro;
        let wear = (self.synchro_energy_j[gi] / h.life_j.max(1.0)).clamp(0.0, 1.0);
        let t = self.synchro_c[gi];
        let fade = if t < 200.0 { 1.0 } else { (1.0 - (t - 200.0) / 300.0).max(0.4) };
        h.mu_cone * (1.0 - 0.6 * wear) * fade
    }

    /// Blocker ring holds the sleeve while slipping only if cone torque ≥ indexing torque.
    fn blocker_holds(&self, d: &DrivetrainProfile, gi: usize) -> bool {
        let h = &d.train.synchro;
        h.cones[gi] * self.synchro_mu(d, gi) * h.cone_radius_m / h.sin_cone_angle.max(1e-4) >= h.index_ratio_m
    }

    /// Sleeve stops in each direction (blocker ring, dog chamfers, end stops) and dog friction.
    fn fork_stops(&self, d: &DrivetrainProfile) -> [(bool, bool, f64); 4] {
        let tol = 1e-3;
        std::array::from_fn(|f| {
            let (_, plus, minus) = d.forks[f];
            let y = self.fork_mm[f];
            let blocked = |g: u8, side: f64| -> bool {
                let ya = y * side;
                if ya >= d.fork_travel_mm - tol {
                    return true;
                }
                let Some(gi) = gear_index(g) else { return ya >= d.free_side_mm - tol };
                let ratio = d.gear_ratio(g).abs().max(1e-6);
                let dw = (self.gear_slip(d, gi) / ratio).abs();
                if (ya - d.fork_sync_mm).abs() < tol && dw > SYNC_INDEX_RAD_S && self.blocker_holds(d, gi) {
                    return true;
                }
                let dog_thresh = if self.blocker_holds(d, gi) { SYNC_INDEX_RAD_S } else { DOG_CLASH_RAD_S };
                (ya - d.fork_dog_mm).abs() < tol && dw > dog_thresh && self.mech.fork_contact_n[f].abs() < DOG_FORCE_THROUGH_N
            };
            let mut fc = 0.0;
            for (g, side) in [(plus, 1.0), (minus, -1.0)] {
                if let Some(gi) = gear_index(g) {
                    if self.gear_engaged[gi] && y * side > 0.0 {
                        fc = d.train.dog_mu * (self.gear_torque_nm[gi] * d.gear_ratio(g)).abs() / d.train.dog_radius_m;
                    }
                }
            }
            (blocked(plus, 1.0), blocked(minus, -1.0), fc)
        })
    }

    /// Computes the clutch torques for this sub-step after stepping the mechatronic.
    /// Steps the mechatronic and forks while the gearbox is uncoupled from the engine (engine-dyno stand).
    /// `omega_pump`: hydraulic pump input speed on wet units (rad/s) when a TCU stays powered in the bench session.
    #[allow(clippy::too_many_arguments)]
    pub fn step_uncoupled(&mut self, d: &DrivetrainProfile, tcu: &TcuOutputs, omega_pump: f64, brake_pct: f64, battery_v: f64, ambient_c: f64, dt: f64) {
        if !(dt > 0.0 && dt.is_finite()) {
            return;
        }
        let omega_pump = if omega_pump.is_finite() { omega_pump.max(0.0) } else { 0.0 };
        let brake_frac = if brake_pct.is_finite() { (brake_pct / 100.0).clamp(0.0, 1.0) } else { 0.0 };
        self.omega_drive = omega_pump;
        self.shaft_omega = [0.0; 2];
        self.carrier_omega = 0.0;
        self.wheel_omega = [0.0; 2];
        self.vehicle_speed_ms = 0.0;
        self.rollers.speed_ms = 0.0;
        self.rollers.absorber_n = 0.0;
        self.brakes.line_bar = 120.0 * brake_frac;
        self.brakes.torque_nm = [0.0; 2];
        self.airflow_ms = 0.0;
        self.accel_ms2 = 0.0;
        self.clutch_slip_rad_s = [0.0; 2];
        self.clutch_torque_nm = [0.0; 2];
        self.clutch_locked = [false; 2];
        self.gear_torque_nm = [0.0; 8];
        self.gear_twist = [0.0; 8];
        self.halfshaft_torque_nm = 0.0;
        self.halfshaft_twist = 0.0;
        if !tcu.any_driven() && self.mech.clutch_x_m.iter().all(|&x| x <= 1e-6) {
            self.mech_heat_w = 0.0;
            return;
        }
        let env = DriveEnv {
            battery_v,
            ambient_c,
            air_density: 1.2,
            coolant_c: self.case_c,
            selector: Selector::P,
            brake_pct: 0.0,
            awd_pump_duty: None,
            rollers: None,
        };
        let cenv = self.clutch_env(d, &env, omega_pump);
        let fl = Fluid::ATF;
        let stops = self.fork_stops(d);
        let fork_y = self.fork_mm.map(|v| v * 1e-3);
        let full = d.fork_full_mm * 1e-3;
        {
            let clutches = &self.clutches;
            let mech = &mut self.mech;
            let damping = [0, 1].map(|k| clutches.piston_damping(&d.clutch, k, mech.clutch_x_m[k], &fl, cenv.oil_c));
            let cf = |k: usize, x: f64| clutches.piston_force(&d.clutch, k, x, &cenv, &fl);
            let ff = |_f: usize, y: f64| -FORK_DETENT_N * (2.0 * PI * y / full).sin();
            let loads = MechLoads {
                clutch_force: &cf,
                clutch_damping: damping,
                clutch_sep_limit_n: [d.clutch.separation_limit_n(0), d.clutch.separation_limit_n(1)],
                clutch_friction_n: [0, 1].map(|k| d.clutch.actuator_friction_n(k, mech.clutch_x_m[k])),
                clutch_area: [d.clutch.actuator_area(0), d.clutch.actuator_area(1)],
                clutch_stroke: [d.clutch.stroke_max(0), d.clutch.stroke_max(1)],
                fork_force: &ff,
                fork_stops: stops,
                fork_y,
                fork_shaft: d.forks.map(|f| f.0),
            };
            self.mech_heat_w = mech.step(&d.mechatronic, tcu, battery_v, omega_pump, self.case_c, &loads, dt);
        }
        for k in 0..2 {
            self.clutches.set_travel(&d.clutch, k, self.mech.clutch_x_m[k]);
        }
        self.clutch_capacity_nm = [0, 1].map(|k| self.clutches.static_capacity(&d.clutch, k, &cenv));
        self.update_forks(d, dt);
    }

    /// `omega_in`, `t_in`, `j_in`: speed of, net torque on (excluding the gearbox) and inertia of
    /// the clutch drive side (DMF secondary incl. [`Self::drive_inertia`]).
    /// Returns the torque the gearbox takes off that body (clutches + wet-unit oil pump), Nm.
    #[allow(clippy::too_many_arguments)]
    pub fn clutch_torques(&mut self, d: &DrivetrainProfile, tcu: &TcuOutputs, env: &DriveEnv, omega_in: f64, t_in: f64, j_in: f64, dt: f64) -> f64 {
        self.omega_drive = omega_in;
        let cenv = self.clutch_env(d, env, omega_in);
        let fl = Fluid::ATF;
        let stops = self.fork_stops(d);
        let fork_y = self.fork_mm.map(|v| v * 1e-3);
        let full = d.fork_full_mm * 1e-3;
        {
            let clutches = &self.clutches;
            let mech = &mut self.mech;
            let damping = [0, 1].map(|k| clutches.piston_damping(&d.clutch, k, mech.clutch_x_m[k], &fl, cenv.oil_c));
            let cf = |k: usize, x: f64| clutches.piston_force(&d.clutch, k, x, &cenv, &fl);
            let ff = |_f: usize, y: f64| -FORK_DETENT_N * (2.0 * PI * y / full).sin();
            let loads = MechLoads {
                clutch_force: &cf,
                clutch_damping: damping,
                clutch_sep_limit_n: [d.clutch.separation_limit_n(0), d.clutch.separation_limit_n(1)],
                clutch_friction_n: [0, 1].map(|k| d.clutch.actuator_friction_n(k, mech.clutch_x_m[k])),
                clutch_area: [d.clutch.actuator_area(0), d.clutch.actuator_area(1)],
                clutch_stroke: [d.clutch.stroke_max(0), d.clutch.stroke_max(1)],
                fork_force: &ff,
                fork_stops: stops,
                fork_y,
                fork_shaft: d.forks.map(|f| f.0),
            };
            self.mech_heat_w = mech.step(&d.mechatronic, tcu, env.battery_v, omega_in, self.case_c, &loads, dt);
        }
        for k in 0..2 {
            self.clutches.set_travel(&d.clutch, k, self.mech.clutch_x_m[k]);
        }
        self.gear_set_torques(d, dt);

        // Clutch stick/slip with every other torque known (complementarity over ≤ 2 contacts).
        let j_k = [0, 1].map(|k| d.train.shaft_inertia_kgm2[k] + d.clutch.hub_inertia(k));
        let slip = [0, 1].map(|k| omega_in - self.shaft_omega[k]);
        self.clutch_slip_rad_s = slip;
        let cap = [0, 1].map(|k| self.clutches.static_capacity(&d.clutch, k, &cenv));
        self.clutch_capacity_nm = cap;
        let mut torque = [0.0f64; 2];
        let mut cand = [false; 2];
        for k in 0..2 {
            if slip[k].abs() <= LOCK_TOL_RAD_S && cap[k] > 0.5 {
                cand[k] = true;
            } else {
                torque[k] = self.clutches.slip_torque(&d.clutch, k, slip[k], self.slip_angle[k], &cenv, &fl);
            }
        }
        let t_e = t_in - self.mech.pump_torque_nm;
        let mut locked = [false; 2];
        for _ in 0..3 {
            if !cand[0] && !cand[1] {
                break;
            }
            let j = j_in + (0..2).filter(|&k| cand[k]).map(|k| j_k[k]).sum::<f64>();
            let tt = t_e - (0..2).filter(|&k| !cand[k]).map(|k| torque[k]).sum::<f64>() + (0..2).filter(|&k| cand[k]).map(|k| self.t_ext[k]).sum::<f64>();
            let a = tt / j;
            let req = [0, 1].map(|k| j_k[k] * a - self.t_ext[k]);
            let worst = (0..2).filter(|&k| cand[k] && req[k].abs() > cap[k]).max_by(|&x, &y| (req[x].abs() / cap[x]).total_cmp(&(req[y].abs() / cap[y])));
            match worst {
                Some(k) => {
                    cand[k] = false;
                    torque[k] = cap[k] * sgn(req[k]);
                }
                None => {
                    for k in 0..2 {
                        if cand[k] {
                            torque[k] = req[k];
                            locked[k] = true;
                        }
                    }
                    break;
                }
            }
        }
        self.clutch_torque_nm = torque;
        self.clutch_locked = locked;
        torque[0] + torque[1] + self.mech.pump_torque_nm
    }

    /// Mesh spring torques, synchroniser cone torques and drags for the current state.
    fn gear_set_torques(&mut self, d: &DrivetrainProfile, dt: f64) {
        let tr = &d.train;
        let nu = tr.oil.nu(self.case_c) * 1e6;
        let mut t_ext = [0.0f64; 2];
        let mut t_car = 0.0;
        let j_k = [0, 1].map(|k| tr.shaft_inertia_kgm2[k] + d.clutch.hub_inertia(k));
        for gi in 0..8 {
            let g = gear_code(gi);
            self.sync_torque_in[gi] = 0.0;
            self.gear_torque_nm[gi] = 0.0;
            let Some((f, side, s)) = d.gear_fork(g) else { continue };
            let i_g = d.total_ratio(g);
            let r_g = d.gear_ratio(g);
            let slip = self.gear_slip(d, gi);
            if self.gear_engaged[gi] && !self.gear_damage[gi].broken {
                let phi = self.gear_twist[gi];
                let in_contact = phi.abs() > tr.mesh_lash_rad;
                let c = 2.0 * tr.mesh_damping_ratio * (tr.mesh_stiffness_nm_rad * j_k[s]).sqrt();
                let tau = tr.mesh_stiffness_nm_rad * dead_zone(phi, tr.mesh_lash_rad) + if in_contact { c * slip } else { 0.0 };
                let pitted = if self.gear_pitting[gi].broken { 0.02 } else { 0.0 };
                let eta = (tr.mesh_efficiency - pitted).clamp(0.1, 1.0);
                let forward = tau * self.shaft_omega[s] >= 0.0;
                self.gear_torque_nm[gi] = tau;
                t_ext[s] -= tau;
                t_car += tau * i_g * if forward { eta } else { 1.0 / eta };
            }
            // Synchroniser cone: sleeve pressed against the blocker ring (or grinding at the dogs).
            let ya = self.fork_mm[f] * side;
            let fax = self.mech.fork_contact_n[f] * side;
            if ya >= d.fork_sync_mm - 1e-3 && ya < d.fork_engaged_mm && fax > 0.0 && !self.gear_engaged[gi] {
                let h = &tr.synchro;
                let t_cone = h.cones[gi] * self.synchro_mu(d, gi) * fax * h.cone_radius_m / h.sin_cone_angle.max(1e-4);
                let t_in_max = t_cone / r_g.abs().max(1e-6);
                let s0 = (t_in_max * dt / j_k[s]).max(0.2);
                let t_in = -t_in_max * (slip / s0).clamp(-1.0, 1.0);
                self.sync_torque_in[gi] = t_in;
                t_ext[s] += t_in;
                t_car -= i_g * t_in;
            }
        }
        // Rolling-bearing drag (SKF M0, 4 bearings Ø45 per shaft, f0 = 2).
        for (te, &sw) in t_ext.iter_mut().zip(&self.shaft_omega) {
            let n = sw.abs() * 30.0 / PI;
            let m0 = 4.0 * 1e-7 * 2.0 * (nu * n).max(2000.0).powf(2.0 / 3.0) * 45f64.powi(3) * 1e-3;
            *te -= m0 * (sw / 0.5).tanh();
        }
        self.t_ext = t_ext;
        self.t_carrier_gears = t_car;
    }

    /// Churning of the dipping ring gear (Changenet & Velex 2007), Nm on the carrier.
    fn churning_torque(&self, d: &DrivetrainProfile) -> f64 {
        let tr = &d.train;
        let w = self.carrier_omega.abs();
        if w < 0.1 {
            return 0.0;
        }
        let (r, dia) = (tr.ring_radius_m, 2.0 * tr.ring_radius_m);
        let nu = tr.oil.nu(self.case_c);
        let fr = w * w * r / G;
        let re = w * r * r / nu;
        let v0 = tr.oil_volume_m3 / dia.powi(3);
        let cm = if re < 6000.0 {
            1.366 * tr.immersion_ratio.powf(0.45) * v0.powf(0.1) * fr.powf(-0.6) * re.powf(-0.21)
        } else {
            3.644 * tr.immersion_ratio.powf(0.1) * v0.powf(-0.35) * fr.powf(-0.88) * (tr.ring_face_m / dia).powf(0.85)
        };
        0.5 * tr.oil.rho * w * w * r.powi(3) * tr.ring_immersed_m2 * cm * sgn(self.carrier_omega)
    }

    /// Integrates shafts, carrier, wheels, vehicle, forks, temperatures and damage.
    /// `omega_in_after`: drive-side speed after the engine model integrated it.
    pub fn integrate(&mut self, d: &DrivetrainProfile, env: &DriveEnv, omega_in_after: f64, dt: f64) {
        if !(dt > 0.0 && dt.is_finite()) {
            return;
        }
        let tr = &d.train;
        let j_k = [0, 1].map(|k| tr.shaft_inertia_kgm2[k] + d.clutch.hub_inertia(k));
        for (k, &jk) in j_k.iter().enumerate() {
            if self.clutch_locked[k] {
                self.shaft_omega[k] = omega_in_after;
            } else {
                self.shaft_omega[k] += (self.clutch_torque_nm[k] + self.t_ext[k]) / jk * dt;
            }
            self.slip_angle[k] = (self.slip_angle[k] + (self.omega_drive - self.shaft_omega[k]) * dt) % (2.0 * PI);
        }

        // Tyres (forces from the current state), road or rollers under each axle.
        let r = d.tyre.radius_m;
        // Synchronised 4WD roller bench (Maha MSR / ElsaWin MQB requirement): both axles sit on linked rollers so
        // non-driven wheels turn at the roller surface speed instead of standing at 0 km/h.
        let ground = [if env.rollers.is_some() { self.rollers.speed_ms } else { self.vehicle_speed_ms }; 2];
        let fx = [0, 1].map(|a| self.tyres[a].step(&d.tyre, self.body.axle_load_n[a], ground[a], self.wheel_omega[a], dt));

        // Driveshafts (both sides; an open differential cannot drive either side if one is broken).
        let th = self.halfshaft_twist;
        let rel = self.carrier_omega - self.wheel_omega[0];
        let j_wf = 2.0 * d.tyre.inertia_kgm2;
        let c_hs = 2.0 * 0.05 * (tr.halfshaft_stiffness_nm_rad * tr.output_inertia_kgm2).sqrt();
        let t_hs = if self.driveshaft_damage.broken {
            0.0
        } else {
            tr.halfshaft_stiffness_nm_rad * dead_zone(th, tr.halfshaft_lash_rad) + if th.abs() > tr.halfshaft_lash_rad { c_hs * rel } else { 0.0 }
        };
        self.halfshaft_torque_nm = t_hs;

        // Parking pawl (mechanical, selector cable).
        if env.selector == Selector::P && !self.pawl_damage.broken {
            if !self.pawl_engaged {
                if self.carrier_omega.abs() <= tr.pawl_engage_max_rad_s {
                    self.pawl_engaged = true;
                    self.pawl_twist = 0.0;
                } else {
                    // Ratcheting: the cam ejects the pawl; tip wear per tooth pass.
                    self.pawl_damage.fatigue += 2e-4 * self.carrier_omega.abs() * dt;
                }
            }
        } else {
            self.pawl_engaged = false;
        }
        let t_pawl = if self.pawl_engaged {
            let c = 2.0 * 0.1 * (tr.pawl_stiffness_nm_rad * tr.output_inertia_kgm2).sqrt();
            tr.pawl_stiffness_nm_rad * dead_zone(self.pawl_twist, tr.pawl_lash_rad) + if self.pawl_twist.abs() > tr.pawl_lash_rad { c * self.carrier_omega } else { 0.0 }
        } else {
            0.0
        };
        self.pawl_torque_nm = t_pawl;

        // Carrier (output shafts + final drive + differential) and Haldex coupling to rear axle.
        let t_churn = self.churning_torque(d);
        let nu = tr.oil.nu(self.case_c) * 1e6;
        let n_c = self.carrier_omega.abs() * 30.0 / PI;
        let t_bear_out = 4.0 * 1e-7 * 2.0 * (nu * n_c * 4.0).max(2000.0).powf(2.0 / 3.0) * 50f64.powi(3) * 1e-3 * (self.carrier_omega / 0.5).tanh();
        let t_car_0 = self.t_carrier_gears - t_hs - t_pawl - t_churn - t_bear_out;

        let mut t_rear = 0.0;
        if let Some(hx) = &d.haldex {
            let dw = self.carrier_omega - self.wheel_omega[1];
            let cap = self.haldex.hydraulics(
                hx,
                &crate::vehicle::AwdOutputs { pump_motor_duty: env.awd_pump_duty },
                env.battery_v,
                dw * hx.ratio,
                dt,
            );
            let j_front = tr.output_inertia_kgm2;
            let j_rear = 2.0 * d.tyre.inertia_kgm2;
            let w_car_free = self.carrier_omega + t_car_0 / j_front * dt;
            let w_rear_free = self.wheel_omega[1] - fx[1] * r / j_rear * dt;
            let need = (w_car_free - w_rear_free) / (dt * (1.0 / j_front + 1.0 / j_rear));
            t_rear = need.clamp(-cap * hx.ratio, cap * hx.ratio);
            self.haldex.torque_nm = t_rear / hx.ratio;
            self.haldex.thermal((t_rear * dw).abs(), env.ambient_c, dt);
        }

        let t_car = t_car_0 - t_rear;
        self.carrier_omega += t_car / tr.output_inertia_kgm2 * dt;

        // Wheels with Coulomb brakes (exact stick/slip per step).
        self.brakes.line_bar = 120.0 * (env.brake_pct / 100.0).clamp(0.0, 1.0);
        let drive = [t_hs, t_rear];
        let j_w = [j_wf, 2.0 * d.tyre.inertia_kgm2];
        for a in 0..2 {
            let free = self.wheel_omega[a] + (drive[a] - fx[a] * r) / j_w[a] * dt;
            let cap = self.brakes.capacity(&d.brakes, a);
            let dw = cap * dt / j_w[a];
            let new = if free.abs() <= dw { 0.0 } else { free - sgn(free) * dw };
            self.brakes.torque_nm[a] = (free - new) * j_w[a] / dt;
            self.wheel_omega[a] = new;
        }

        // Twists with the new speeds (symplectic).
        for gi in 0..8 {
            if self.gear_engaged[gi] {
                self.gear_twist[gi] += self.gear_slip(d, gi) * dt;
            }
        }
        self.halfshaft_twist += (self.carrier_omega - self.wheel_omega[0]) * dt;
        if self.pawl_engaged {
            self.pawl_twist += self.carrier_omega * dt;
        }

        // Body or rollers.
        let b = &d.body;
        let grade = (self.road_grade_pct / 100.0).atan();
        let v = self.vehicle_speed_ms;
        let rr = [0, 1].map(|a| self.tyres[a].rolling_resistance(&d.tyre, self.body.axle_load_n[a], ground[a]));
        if let Some((hw, target)) = env.rollers {
            let f_roll: f64 = (0..2).map(|a| fx[a] - rr[a]).sum();
            if hw.road_load {
                let vr = self.rollers.speed_ms;
                let f_aero = 0.5 * env.air_density * b.drag_coefficient * b.frontal_area_m2 * vr * vr;
                self.rollers.step_road_load(&hw, f_roll, f_aero, b.mass_kg, dt);
                self.airflow_ms = self.rollers.speed_ms;
            } else {
                self.airflow_ms = 0.0;
                self.rollers.step(&hw, f_roll, target, dt);
            }
            self.vehicle_speed_ms = 0.0;
            self.accel_ms2 = 0.0;
            self.body.step(b, 0.0, 0.0, env.air_density, 0.0, true, dt);
        } else {
            let f_aero = 0.5 * env.air_density * b.drag_coefficient * b.frontal_area_m2 * v * v.abs();
            let a = (fx[0] + fx[1] - f_aero - rr[0] - rr[1] - b.mass_kg * G * grade.sin()) / b.mass_kg;
            self.vehicle_speed_ms = v + a * dt;
            self.airflow_ms = self.vehicle_speed_ms;
            self.accel_ms2 = a;
            self.body.step(b, a, v, env.air_density, grade, false, dt);
        }

        self.update_forks(d, dt);
        self.thermal_and_damage(d, env, fx, rr, ground, dt);
    }

    fn update_forks(&mut self, d: &DrivetrainProfile, dt: f64) {
        for f in 0..4 {
            let (s_fork, plus, minus) = d.forks[f];
            let y0 = self.fork_mm[f];
            let mut y = (y0 + self.mech.fork_v[f] * 1e3 * dt).clamp(-d.fork_travel_mm, d.fork_travel_mm);
            for (g, side) in [(plus, 1.0), (minus, -1.0)] {
                let Some(gi) = gear_index(g) else {
                    if y * side > d.free_side_mm {
                        y = side * d.free_side_mm; // no gear on this side: sleeve end stop
                    }
                    continue;
                };
                let s = if side < 0.0 { d.fork_minus_shaft[f].unwrap_or(s_fork) } else { s_fork };
                let (ya0, ya) = (y0 * side, y * side);
                let dw = (self.gear_slip(d, gi) / d.gear_ratio(g).abs().max(1e-6)).abs();
                // Blocker ring.
                if ya0 <= d.fork_sync_mm && ya > d.fork_sync_mm && dw > SYNC_INDEX_RAD_S && self.blocker_holds(d, gi) {
                    y = side * d.fork_sync_mm;
                }
                // Dog chamfers (or blocker ring if the fork reversed between sync and dog): clash / grinding, or rammed through.
                let ya = y * side;
                let dog_thresh = if self.blocker_holds(d, gi) { SYNC_INDEX_RAD_S } else { DOG_CLASH_RAD_S };
                if ya0 <= d.fork_dog_mm && ya > d.fork_dog_mm && dw > dog_thresh {
                    if self.mech.fork_contact_n[f].abs() < DOG_FORCE_THROUGH_N {
                        y = side * d.fork_dog_mm;
                    } else if !self.blocker_holds(d, gi) {
                        self.dog_health[gi] -= 1e-4 * dw * dw;
                    }
                }
                let ya = y * side;
                if !self.gear_engaged[gi] && ya0 <= d.fork_engaged_mm && ya > d.fork_engaged_mm && dw > DOG_CLASH_RAD_S && self.mech.fork_contact_n[f].abs() < DOG_FORCE_THROUGH_N {
                    y = side * d.fork_engaged_mm;
                }
                if !self.blocker_holds(d, gi) && (y * side - d.fork_dog_mm).abs() < 1e-3 && dw > DOG_CLASH_RAD_S {
                    self.dog_health[gi] -= 2e-5 * dw * self.mech.fork_contact_n[f].abs() * dt;
                }
                self.dog_health[gi] = self.dog_health[gi].max(0.0);
                // Once the sleeve splines slide past `fork_engaged_mm` into the dog tooth slots, they remain
                // rotationally meshed until the sleeve withdraws past the dog tooth tips (`fork_dog_mm`).
                let threshold = if self.gear_engaged[gi] { d.fork_dog_mm } else { d.fork_engaged_mm };
                let engaged = y * side >= threshold;
                if engaged && !self.gear_engaged[gi] {
                    self.gear_twist[gi] = 0.0;
                    if !self.clutch_locked[s] && dw <= DOG_CLASH_RAD_S {
                        self.shaft_omega[s] = d.total_ratio(g) * self.carrier_omega;
                    }
                }
                // Rounded dogs (lost back-taper) let the sleeve walk out under load.
                if engaged && self.dog_health[gi] < 0.5 && (self.gear_torque_nm[gi] * d.gear_ratio(g)).abs() > 1500.0 * self.dog_health[gi] {
                    y = side * (d.fork_dog_mm - 0.1);
                }
                self.gear_engaged[gi] = y * side >= threshold;
            }
            self.fork_mm[f] = y;
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn thermal_and_damage(&mut self, d: &DrivetrainProfile, env: &DriveEnv, fx: [f64; 2], rr: [f64; 2], ground: [f64; 2], dt: f64) {
        let tr = &d.train;
        // Clutches.
        let p_slip = [0, 1].map(|k| (self.clutch_torque_nm[k] * (self.omega_drive - self.shaft_omega[k])).abs());
        let cenv = self.clutch_env(d, env, self.omega_drive);
        let clutch_to_oil = self.clutches.thermal_step(&d.clutch, p_slip, &cenv, &Fluid::ATF, dt);

        // Synchroniser rings: friction heat, wear energy.
        let mut p_sync = 0.0;
        for gi in 0..8 {
            if d.gear_fork(gear_code(gi)).is_some() {
                let p = (self.sync_torque_in[gi] * self.gear_slip(d, gi)).abs();
                p_sync += p;
                let hot = 1.0 + (self.synchro_c[gi] - 150.0).max(0.0) / 50.0;
                self.synchro_energy_j[gi] += p * hot * dt;
                let h = &tr.synchro;
                self.synchro_c[gi] += (p - h.ring_g_w_k * (self.synchro_c[gi] - self.case_c)) / h.ring_c_j_k * dt;
            }
        }

        // Gear-set losses → case.
        let mut p_mesh = 0.0;
        for gi in 0..8 {
            if let Some((_, _, s)) = d.gear_fork(gear_code(gi)) {
                p_mesh += (1.0 - tr.mesh_efficiency) * (self.gear_torque_nm[gi] * self.shaft_omega[s]).abs();
            }
        }
        let p_churn = (self.churning_torque(d) * self.carrier_omega).abs();
        self.gear_loss_w = p_mesh + p_churn;
        let v = self.vehicle_speed_ms.abs().max(if env.rollers.is_some() { self.rollers.speed_ms } else { 0.0 });
        let q_in = self.gear_loss_w + p_sync + self.mech_heat_w + clutch_to_oil;
        let q_out = (tr.case_g_w_k + tr.case_g_per_ms * v) * (self.case_c - env.ambient_c) + tr.cooler_w_k * (self.case_c - env.coolant_c);
        self.case_c += (q_in - q_out) / tr.case_c_j_k * dt;

        for a in 0..2 {
            self.tyres[a].thermal(&d.tyre, rr[a], ground[a], env.ambient_c, dt);
        }
        let _ = fx;
        self.brakes.thermal(&d.brakes, self.wheel_omega, v, env.ambient_c, dt);

        // Fatigue / overload (Miner).
        for gi in 0..8 {
            let g = gear_code(gi);
            let Some((_, _, s)) = d.gear_fork(g) else { continue };
            let tau = self.gear_torque_nm[gi];
            if tau == 0.0 {
                continue;
            }
            let geo = &tr.gears[gi];
            let ratio = d.gear_ratio(g);
            let cycles = self.shaft_omega[s].abs() * dt / (2.0 * PI);
            let notch = if self.gear_pitting[gi].broken { 1.3 } else { 1.0 };
            self.gear_damage[gi].add(cycles, fail::bending_cycles(fail::root_stress_mpa(tau, geo, ratio) * notch));
            self.gear_pitting[gi].add(cycles, fail::contact_cycles(fail::contact_stress_mpa(tau, geo, ratio)));
        }
        for k in 0..2 {
            let t = self.clutch_torque_nm[k].abs().max(self.t_ext[k].abs());
            let sh = &tr.input_shafts[k];
            let cycles = self.shaft_omega[k].abs() * dt / (2.0 * PI);
            self.shaft_damage[k].add(cycles, fail::torsion_cycles(fail::shaft_shear_mpa(t, sh.d_outer_m, sh.d_inner_m), sh.tau_ultimate_mpa));
            if self.shaft_damage[k].broken {
                // A sheared input shaft disconnects its clutch from every gear on it.
                for (gi, gd) in self.gear_damage.iter_mut().enumerate() {
                    if d.gear_fork(gear_code(gi)).map(|x| x.2) == Some(k) {
                        gd.broken = true;
                    }
                }
            }
        }
        let t_side = 0.5 * self.halfshaft_torque_nm.abs();
        let cyc = self.wheel_omega[0].abs() * dt / (2.0 * PI);
        let hs = &tr.halfshaft;
        self.driveshaft_damage.add(cyc, fail::torsion_cycles(fail::shaft_shear_mpa(t_side, hs.d_outer_m, hs.d_inner_m), hs.tau_ultimate_mpa));
        self.driveshaft_damage.add(cyc, fail::torsion_cycles(t_side, tr.cv_joint_strength_nm));
        if self.pawl_engaged && self.pawl_torque_nm.abs() > tr.pawl_strength_nm * (1.0 - self.pawl_damage.fatigue).max(0.2) {
            self.pawl_damage.broken = true;
            self.pawl_engaged = false;
        }
    }

    /// Human-readable list of failed parts.
    pub fn failures(&self, d: &DrivetrainProfile) -> Vec<String> {
        let mut v = Vec::new();
        let name = |gi: usize| if gi == 7 { "R".to_string() } else { (gi + 1).to_string() };
        for gi in 0..8 {
            if self.gear_damage[gi].broken {
                v.push(format!("gear {} tooth fracture", name(gi)));
            } else if self.gear_pitting[gi].broken {
                v.push(format!("gear {} flank pitting", name(gi)));
            }
            if self.dog_health[gi] < 0.5 {
                v.push(format!("gear {} dog teeth rounded", name(gi)));
            }
            if d.gear_fork(gear_code(gi)).is_some() && !self.blocker_holds(d, gi) {
                v.push(format!("gear {} synchroniser no longer blocks", name(gi)));
            }
        }
        for k in 0..2 {
            if self.shaft_damage[k].broken {
                v.push(format!("input shaft {} fractured", k + 1));
            }
            let (h, glaze, _, warp) = self.clutches.condition()[k];
            if h < 0.5 {
                v.push(format!("K{} friction lining destroyed", k + 1));
            } else if glaze > 0.2 {
                v.push(format!("K{} lining glazed", k + 1));
            }
            if warp > 0.05 {
                v.push(format!("K{} plates warped", k + 1));
            }
        }
        if self.driveshaft_damage.broken {
            v.push("driveshaft / CV joint fractured".into());
        }
        if self.pawl_damage.broken {
            v.push("parking pawl broken".into());
        }
        if self.mech.solenoids.iter().any(|s| s.open_circuit) {
            v.push("solenoid coil burnt open".into());
        }
        if self.mech.pump_health < 0.9 {
            v.push("oil pump cavitation erosion".into());
        }
        if self.mech.atf_degradation > 0.5 {
            v.push("ATF oxidised".into());
        }
        if self.haldex.health < 0.5 {
            v.push("Haldex clutch pack burnt".into());
        }
        v
    }

    pub fn fill_telemetry(&self, d: &DrivetrainProfile, t: &mut PlantTelemetry) {
        let rpm = 30.0 / PI;
        let r = d.tyre.radius_m;
        let gc = |g: Option<u8>| match g {
            Some(REVERSE) => -1,
            Some(g) => g as i8,
            None => 0,
        };
        let (temps, surf) = self.clutches.temps();
        let cond = self.clutches.condition();
        t.gearbox = d.slug;
        t.vehicle_kmh = self.vehicle_speed_ms * 3.6;
        t.wheel_kmh_driven = self.wheel_omega[0] * r * 3.6;
        t.wheel_kmh = [self.wheel_omega[0] * r * 3.6, self.wheel_omega[0] * r * 3.6, self.wheel_omega[1] * r * 3.6, self.wheel_omega[1] * r * 3.6];
        t.accel_ms2 = self.accel_ms2;
        t.shaft_rpm = self.shaft_omega.map(|w| w * rpm);
        t.output_shaft_rpm = [d.final_drives[0], d.final_drives[(d.gears.max(1) - 1).min(6)]].map(|f| self.carrier_omega * rpm * f);
        t.shaft_gear = [gc(self.engaged_on_shaft(d, 0)), gc(self.engaged_on_shaft(d, 1))];
        t.driving_gear = gc(self.driving_gear(d));
        t.fork_mm = self.fork_mm;
        t.clutch_travel_mm = self.mech.clutch_x_m.map(|x| x * 1e3);
        t.clutch_kiss_mm = [0, 1].map(|k| self.clutches.kiss_point_m(&d.clutch, k) * 1e3);
        t.clutch_capacity_nm = self.clutch_capacity_nm;
        t.clutch_torque_nm = self.clutch_torque_nm;
        t.clutch_slip_rpm = self.clutch_slip_rad_s.map(|w| w * rpm);
        t.clutch_temp_c = temps;
        t.clutch_surface_c = surf;
        t.clutch_bar = self.mech.clutch_pa.map(|p| p / 1e5);
        t.clutch_wear_mm = cond.map(|c| c.2);
        t.clutch_health = cond.map(|c| c.0);
        t.gearbox_hydraulic_bar = self.mech.supply_pa / 1e5;
        t.sub_gearbox_bar = self.mech.sub_pa.map(|p| p / 1e5);
        t.solenoid_current_a = std::array::from_fn(|i| self.mech.solenoids[i].current_a);
        t.solenoid_names = crate::mechatronic::solenoid_names(&d.mechatronic);
        t.pump_rpm = self.mech.pump.omega * rpm;
        t.pump_current_a = self.mech.pump.current_a;
        t.pump_flow_lpm = self.mech.pump_flow_m3_s * 6e4;
        t.mechatronic_oil_c = self.mech.oil_c;
        t.gearbox_case_c = self.case_c;
        t.cooling_lpm = (self.mech.cool_flow_m3_s[0] + self.mech.cool_flow_m3_s[1]) * 6e4;
        t.brake_bar = self.brakes.line_bar;
        t.brake_disc_c = self.brakes.disc_c;
        t.tyre_force_n = [self.tyres[0].force_n, self.tyres[1].force_n];
        t.tyre_tread_c = [self.tyres[0].tread_c, self.tyres[1].tread_c];
        t.axle_load_n = self.body.axle_load_n;
        t.pitch_deg = self.body.pitch_rad.to_degrees();
        t.driveshaft_torque_nm = self.halfshaft_torque_nm;
        t.haldex_bar = self.haldex.pressure_pa / 1e5;
        t.haldex_torque_nm = self.haldex.torque_nm;
        t.roller_kmh = self.rollers.speed_ms * 3.6;
        t.roller_absorber_n = self.rollers.absorber_n;
        t.driveline_failures = self.failures(d);
    }
}
