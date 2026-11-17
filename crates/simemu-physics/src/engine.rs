//! EA888 Gen3 engine plant: crank-angle-resolved cylinders, valvetrain, crank train with dual-mass
//! flywheel, lubrication, fuel system, air path and turbocharger, and the physical failure chain.
//!
//! Every input a real ECU decides arrives through [`EcuOutputs`] at hardware level (coil dwell/spark
//! angle, injector pulses, HPFP valve closing angle, PWM duties, relays). `None` = not driven: the
//! hardware falls back to its de-energised state (throttle on the limp-home spring, normally-open
//! HPFP valve, no spark, no injection, phasers locked at rest, jets on, flaps open, ...).
//!
//! Integration: the plant calls [`EngineState::step`] every sub-step (0.2 ms). Inside, the crank
//! angle is advanced in adaptive steps of ≤ 1 °CA and ≤ 50 µs; cylinders, valves, manifolds, rail,
//! crank/DMF and bearings are integrated on that grid. Compressor, boost volume, turbo speed and
//! actuators run on the sub-step.

use crate::acoustic_probe::{CrankFrame, ImpactKind, Probe};
use crate::atmosphere::{oil_viscosity_cst, orifice_flow, CP_AIR, CP_EXH, GAMMA_AIR, GAMMA_EXH, R_AIR};
use crate::crank_train::{self, Kinematics, Slider, FIRING_OFFSET_DEG};
use crate::cylinder::{CylIn, Cylinder, AIR, CP_E, FUEL, PROD};
use crate::lubrication::{film_viscosity, Lubrication, N_BEARINGS};
use crate::knock_sensor::BlockVibration;
use crate::wastegate::WastegateActuator;
use crate::materials::{in713c_rupture_h, WHEEL_AA2618, WHEEL_IN713C};
use crate::profiles::{EngineProfile, EtcActuator};
use crate::starter::{Starter, STARTER_1_1KW};
use crate::sensors::SensorEdge;
use crate::valvetrain::Valvetrain;
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

pub use crate::cylinder::{AFR_STOICH, FUEL_DENSITY_KG_M3, LHV_J_KG};
pub const FUEL_BULK_MODULUS_PA: f64 = 1.2e9;
/// Exhaust back-pressure coefficient of cat + silencers, Pa/(kg/s)². ~17 kPa at 0.17 kg/s.
pub const EXHAUST_SYSTEM_K: f64 = 6.0e5;
/// Air-filter pressure-drop coefficient, Pa/(kg/s)². ~3 kPa at 0.17 kg/s.
pub const AIR_FILTER_K: f64 = 1.0e5;
const DIVERTER_CDA_M2: f64 = 2.5e-4;
const COMPRESSOR_INDUCER_CDA_M2: f64 = 1.0e-3;
/// HPFP: plunger stroke, m (area from `hpfp_stroke_cc`, UNVERIFIED); plunger TDC at this crank angle
/// (+180·k, + exhaust cam shift): SC8S50 `c_ang_ref_vcv` = 0 °CRK ("relative crank angle displacement
/// between TDC on cyl. 0 and TDC of the single-piston pump", DAMOS @0x8080860E) — the 4-lobe pump cam
/// puts a plunger TDC on every cylinder TDC.
pub const HPFP_LIFT_M: f64 = 3.2e-3;
pub const HPFP_TDC_DEG: f64 = 0.0;
// N276 quantity-control valve (normally open; FUSL 43.15, FR p.12026 "currentless open VCV") and the pump: delivered
// fuel per stroke = efficiency × displaced volume after the inlet valve closed − internal loss (the plunger first
// compresses the loss volume before the outlet opens) — profile fields `hpfp_*` (the calibration's pump model).
// Valve sequence (Bosch-type digital inlet valve, Reif (ed.), "Gasoline Engine Management", 2015, HDP5): current
// on → after the closing delay the armature has pulled the needle off the inlet plate; the plate then closes as
// soon as the plunger moves up (on its own during the down-stroke the suction keeps it open); once closed it is held
// by the pump-chamber pressure until plunger TDC even without current ("self locking", `c_fup_min_lock_vcv`),
// below that rail pressure only as long as the armature is held. Current off → the armature returns after
// `hpfp_vcv_release_ms`.
/// Port-flap actuator (vacuum diaphragm switched by N316) time constant, s. UNVERIFIED.
const PORT_FLAP_TAU_S: f64 = 0.25;
/// Liner temperature at which the ring / skirt film thicknesses of `piston_friction` hold (warm engine: coolant 90 °C + 30 K), K.
const MU_LINER_REF_K: f64 = 393.15;
/// Tumble flaps closed: intake flow coefficient multiplier. UNVERIFIED.
const TUMBLE_FLOW_FACTOR: f64 = 0.75;
const CRANK_STEP_RAD: f64 = 1.0 * PI / 180.0;
const MAX_STEP_S: f64 = 50e-6;
/// Exhaust manifold gas: R and cv, J/(kg·K).
const R_E: f64 = 288.0;
const CV_E: f64 = CP_E - R_E;
/// Centrifugal stress shape factor of a bladed disc, σ = K·ρ·U_tip² (bore hoop stress). UNVERIFIED.
const WHEEL_SHAPE_K: f64 = 0.31;
/// Hydrolock: smallest gas pocket the liquid leaves before the piston stalls on it, m³.
const LIQUID_MIN_GAS_M3: f64 = 1.5e-6;
/// A bent rod this short has folded / the big end has let go. UNVERIFIED.
const ROD_COLLAPSE_M: f64 = 6e-3;

/// Ignition coil command: coil current flows for `dwell_ms`, the spark is the end of dwell at
/// `spark_btdc_deg` before the cylinder's firing TDC (negative = after TDC).
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct CoilCmd {
    pub dwell_ms: f64,
    pub spark_btdc_deg: f64,
}

/// One DI pulse: electrical start at `soi_btdc_deg` before the cylinder's firing TDC (e.g. 300 =
/// early intake stroke), electrical duration `width_ms` (opening dead time included).
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct InjPulse {
    pub soi_btdc_deg: f64,
    pub width_ms: f64,
}

/// One active phase of a timing output in plant time, s: the ECU's own edges (co-simulation: board edge time + the fixed
/// actuator delay of the lock-step window). `off_s` = ∞ while the phase had not ended at the last look.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Drive {
    pub on_s: f64,
    pub off_s: f64,
}

/// The timing outputs as the edges the ECU drove (event-accurate): per output the active phases that end after the plant's
/// current time (≤ 3, oldest first). When present the plant runs the DI and MPI injectors, the coils and N276 from these
/// and ignores the per-cycle angle commands of [`EcuOutputs`] (which then only summarise the last cycle for display).
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct TimedOutputs {
    /// Injector current (electrical pulse) of the DI / MPI injector of cylinder 1..4.
    pub di: [[Option<Drive>; 3]; 4],
    pub mpi: [[Option<Drive>; 3]; 4],
    /// Coil primary current of cylinder 1..4 (dwell); the spark is its end.
    pub coil: [[Option<Drive>; 3]; 4],
    /// N276 coil current.
    pub hpfp: [Option<Drive>; 3],
}

/// One injector pulse in progress: electrical start (plant time) and width (∞ = end not known yet), s.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub(crate) struct InjSlot {
    pub on_s: f64,
    pub w_s: f64,
}

impl InjSlot {
    /// Electrical time since the start at `t`, within the pulse.
    pub fn tau(&self, t: f64) -> f64 {
        (t - self.on_s).clamp(0.0, self.w_s)
    }
}

/// The pulses of one injector: ≤ 3 in progress, and the start of the last timed drive taken (each drive starts once).
#[derive(Debug, Clone, Copy, Serialize)]
pub(crate) struct Injector {
    pub slots: [Option<InjSlot>; 3],
    taken_on_s: f64,
}

impl Default for Injector {
    fn default() -> Self {
        Self { slots: [None; 3], taken_on_s: f64::NEG_INFINITY }
    }
}

impl Injector {
    /// Starts / updates the pulses for the step [t, t + h): a per-cycle angle command starts when the crank (cylinder
    /// angle `phi0`..`phi1`) passes its start angle; a timed drive starts at its own time (once) and takes its end when
    /// it has been seen. A pulse no longer listed among the drives stops. Returns the slots started in this step.
    #[allow(clippy::too_many_arguments)]
    pub fn schedule(&mut self, pulses: &[Option<InjPulse>; 3], timed: Option<&[Option<Drive>; 3]>, phi0: f64, phi1: f64, omega: f64, t: f64, h: f64) -> [bool; 3] {
        let mut started = [false; 3];
        match timed {
            Some(drives) => {
                for slot in self.slots.iter_mut() {
                    if slot.is_some_and(|s| !drives.iter().flatten().any(|d| d.on_s == s.on_s)) {
                        *slot = None;
                    }
                }
                for d in drives.iter().flatten() {
                    if let Some(s) = self.slots.iter_mut().flatten().find(|s| s.on_s == d.on_s) {
                        s.w_s = d.off_s - d.on_s;
                    } else if d.on_s > self.taken_on_s && d.on_s < t + h {
                        self.taken_on_s = d.on_s;
                        if let Some(j) = self.slots.iter().position(Option::is_none) {
                            self.slots[j] = Some(InjSlot { on_s: d.on_s, w_s: d.off_s - d.on_s });
                            started[j] = true;
                        }
                    }
                }
            }
            None if omega > 0.0 => {
                for (j, p) in pulses.iter().enumerate() {
                    if let Some(p) = p.filter(|p| p.width_ms > 0.0 && crate::cylinder::crossed(phi0, phi1, (720.0 - p.soi_btdc_deg).rem_euclid(720.0))) {
                        self.slots[j] = Some(InjSlot { on_s: t, w_s: p.width_ms * 1e-3 });
                        started[j] = true;
                    }
                }
            }
            None => {}
        }
        started
    }
}

/// Decoded hardware outputs of the engine ECU. `None` = not driven by the firmware.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EcuOutputs {
    /// Timing outputs as edges ([`TimedOutputs`]); `None` = per-cycle angle commands (physics tests standing in for an ECU).
    #[serde(default)]
    pub timed: Option<TimedOutputs>,
    /// Main relay J271 (powers coils, injector booster, ETC bridge, solenoids).
    pub main_relay: Option<bool>,
    /// Starter relays J906/J907.
    pub starter_relay: Option<bool>,
    /// Throttle motor H-bridge: signed duty −1..+1 of the battery voltage (+ = opening torque); `None` =
    /// bridge disabled (motor open, plate on its springs).
    pub etc_duty: Option<f64>,
    /// Ignition coil per cylinder 1..4 (GPTA0 GTC12..15 active level = dwell, falling edge = spark).
    pub ignition: [Option<CoilCmd>; 4],
    /// Up to three DI pulses per cylinder 1..4 and cycle (ATIC121 timing inputs GPTA0 GTC04/17/16/05).
    pub injection: [[Option<InjPulse>; 3]; 4],
    /// HPFP quantity-control valve N276 current-on angle, °CRK after TDC0 (applied to every 180 °CA
    /// pump lobe). The normally-open valve only delivers from closing until plunger TDC.
    pub hpfp_close_deg: Option<f64>,
    /// N276 current-off angle of the same pulse, °CRK, ≥ `hpfp_close_deg` (unwrapped: on + width). `None` = current
    /// held until the following plunger TDC.
    #[serde(default)]
    pub hpfp_off_deg: Option<f64>,
    /// Up to three MPI (port) injector pulses per cylinder 1..4 and cycle, like [`Self::injection`].
    #[serde(default)]
    pub port_injection: [[Option<InjPulse>; 3]; 4],
    /// In-tank pump control J538 duty 0..1.
    pub lpfp_duty: Option<f64>,
    /// Electric wastegate actuator V465 H-bridge: signed duty −1..+1 of the supply (+ = torque towards the seat,
    /// i.e. closing); `None` = bridge disabled (motor open, flap on its spring and the exhaust gas).
    pub wastegate_duty: Option<f64>,
    /// Diverter valve N249 energised (open).
    pub diverter_open: Option<bool>,
    /// Oil pressure control valve N428 energised (low-pressure stage). De-energised = high stage.
    pub oil_pump_low_stage: Option<bool>,
    /// Cam phaser solenoids N205 (intake) / N318 (exhaust) PWM duty 0..1 (≈ 0.5 = hold).
    pub cam_phaser_duty: [Option<f64>; 2],
    /// Piston-cooling-jet valve N522 energised = jets switched off. UNVERIFIED polarity.
    pub piston_jets_off: Option<bool>,
    /// Port (tumble) flaps commanded closed. SC800S50 drives N316 (ATIC140#1 stage 11) with !PORT_DEAC: energised =
    /// flaps open (VERIFIED-S, `0x8018756E`); the decode inverts it.
    pub tumble_flaps_closed: Option<bool>,
    /// Exhaust AVS actuator coils per cylinder 1..4 energised, [to LFT_1 (small lobe), to standard (large lobe)]
    /// (`profiles::AvsHw`); `None` = not driven (pins stay as they are, the cam pieces stay put).
    #[serde(default)]
    pub avs_actuator: [[Option<bool>; 2]; 4],
    /// Rotary coolant valve actuator N493 (ATIC111 bridge 1): signed duty −1..+1 of the supply (+ = towards larger
    /// valve angle); `None` = bridge disabled (worm gear self-locking, the valve stays put).
    pub rotary_valve_duty: Option<f64>,
    /// Radiator fan J293 PWM 0..1.
    pub fan_duty: Option<f64>,
    /// Alternator regulator voltage set-point commanded over LIN, V (regulator default 14.2 V).
    pub alternator_setpoint_v: Option<f64>,
    /// Oxygen-sensor heater PWM duty 0..1 of the low-side switch on the battery: [upstream wide-band, downstream
    /// binary] (`None` = switch off).
    pub lambda_heater_duty: [Option<f64>; 2],
}

impl EcuOutputs {
    pub fn powered(&self) -> bool {
        self.main_relay.unwrap_or(false)
    }

    /// True when the firmware drives any output.
    pub fn any_driven(&self) -> bool {
        self.timed.is_some()
            || self.main_relay.is_some()
            || self.starter_relay.is_some()
            || self.etc_duty.is_some()
            || self.ignition.iter().any(Option::is_some)
            || self.injection.iter().flatten().any(Option::is_some)
            || self.hpfp_close_deg.is_some()
            || self.hpfp_off_deg.is_some()
            || self.port_injection.iter().flatten().any(Option::is_some)
            || self.lpfp_duty.is_some()
            || self.wastegate_duty.is_some()
            || self.diverter_open.is_some()
            || self.oil_pump_low_stage.is_some()
            || self.cam_phaser_duty.iter().any(Option::is_some)
            || self.piston_jets_off.is_some()
            || self.tumble_flaps_closed.is_some()
            || self.avs_actuator.iter().flatten().any(Option::is_some)
            || self.rotary_valve_duty.is_some()
            || self.fan_duty.is_some()
            || self.alternator_setpoint_v.is_some()
            || self.lambda_heater_duty.iter().any(Option::is_some)
    }
}

/// Steady charge temperature in the intake manifold (where the air temperature sensor sits): air at `t_in_k` through
/// the throttle at `m_air_kg_s` picks up heat from the manifold wall, which sits between the head flange (`t_head_k`,
/// coolant temperature) and the charge: series conductance head → wall (`intake_wall_head_ua_w_k`) → air (UA ∝ ṁ^0.8,
/// turbulent forced convection, Dittus–Boelter; Heywood 1988 §12.3). At idle the wall stays near the head temperature
/// and the slow charge heats up; at high flow the charge cools the wall and the pick-up is limited by the flange.
/// ε = 1 − exp(−UA_series/(ṁ·cp)).
pub fn manifold_air_temp_k(p: &EngineProfile, m_air_kg_s: f64, t_in_k: f64, t_head_k: f64) -> f64 {
    // Below idle mass flow (`intake_wall_ua_ref_kg_s`), natural convection inside the plenum bounds the heat-transfer
    // effectiveness so stagnant air does not asymptotically reach the 90 °C head temperature as ṁ → 0.
    let m = m_air_kg_s.max(p.intake_wall_ua_ref_kg_s);
    let ua_air = p.intake_wall_ua_w_k * (m / p.intake_wall_ua_ref_kg_s).powf(0.8);
    let ua = 1.0 / (1.0 / ua_air + 1.0 / p.intake_wall_head_ua_w_k);
    t_in_k + (1.0 - (-ua / (m * CP_AIR)).exp()) * (t_head_k - t_in_k)
}

/// Fuel in one intake port: x–τ wall films on the intake-valve back and the port wall (Aquino 1981, Hires & Overington
/// 1981; [`crate::profiles::PortFilmHw`]) and the port charge (airborne spray + film vapour) that the next intake event
/// draws in; plus the MPI pulse in progress.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct PortFuel {
    /// Liquid film on the intake-valve back + port wall (total), kg, the two films, and the two films by fuel
    /// pseudo-component (`combustion::FUEL_PSEUDO`: light / mid / heavy).
    pub film_kg: f64,
    pub films_kg: [f64; 2],
    pub films_c: [[f64; 3]; 2],
    /// Fuel in the port charge (vapour + airborne droplets), kg.
    pub vapour_kg: f64,
    /// Injected in the last completed cycle, kg (telemetry).
    pub injected_last_kg: f64,
    injected_kg: f64,
    /// Runner air drawn into the cylinder so far in the current intake event, kg.
    drawn_kg: f64,
    inj: Injector,
}

impl PortFuel {
    /// Draws the port charge with `m_air` kg of runner air entering the cylinder in this step (`m_air` ≤ 0: no intake
    /// flow, the event is over); `m_runner` = air in the runner between injector and valve, kg. Plug flow: the runner
    /// slug that holds the port fuel (injected onto the valve back and into the runner while the valve was shut) is the
    /// first air drawn at the next opening, the fuel spread evenly in it; once the slug is in, fuel reaching the charge
    /// later (film vapour, open-valve injection) is drawn at once. The earlier well-mixed runner (exponential wash-out,
    /// each event taking m_air/m_runner of what is left) held back ≈ 25–30 % of the airborne fuel for later events, which
    /// the firmware's port model has no term for: it puts (1 − Σx)·m_MPI of an injection into the cylinder in the next
    /// event (FMSP 41.19, FR p.11663; docs/SIMOS18_GPTA_IO_RE.md §12.10.4) — λ 2.2–2.7 at its DI → MPI switches and 1.2–1.4
    /// on load steps (docs/SIMOS18_COMBUSTION.md §14). Returns the fuel drawn, kg.
    fn induct(&mut self, m_air: f64, m_runner: f64) -> f64 {
        if m_air <= 0.0 {
            self.drawn_kg = 0.0;
            return 0.0;
        }
        let left = m_runner - self.drawn_kg;
        let mut dm = if left > m_air { self.vapour_kg * m_air / left } else { self.vapour_kg };
        self.drawn_kg += m_air;
        self.vapour_kg -= dm;
        if self.vapour_kg < 1e-24 {
            dm += self.vapour_kg;
            self.vapour_kg = 0.0;
        }
        dm
    }

    /// Advances the port over [t, t + h): MPI pulses ([`InjSlot::schedule`]: angle commands starting in (phi0, phi1]
    /// (cylinder angle, firing TDC = 0/720) or timed drives) flow `q_kg_s` once the dead time has passed, split onto the
    /// films (fractions `x`, fuel pseudo-components `wc`); component c of film i loses the fraction `evap[i][c]`
    /// (= 1 − exp(−h/τ_ic)) per step. Returns the fuel injected in this step, kg.
    #[allow(clippy::too_many_arguments)]
    fn step(&mut self, pulses: &[Option<InjPulse>; 3], timed: Option<&[Option<Drive>; 3]>, phi0: f64, phi1: f64, omega: f64, t: f64, q_kg_s: f64, dead_s: f64, x: [f64; 2], wc: [f64; 3], evap: [[f64; 3]; 2], h: f64) -> f64 {
        self.inj.schedule(pulses, timed, phi0, phi1, omega, t, h);
        if crate::cylinder::crossed(phi0, phi1, 180.0) {
            self.injected_last_kg = self.injected_kg;
            self.injected_kg = 0.0;
        }
        let mut dm = 0.0;
        for slot in self.inj.slots.iter_mut() {
            let Some(s) = *slot else { continue };
            let (a, b) = (s.tau(t), s.tau(t + h));
            dm += q_kg_s * (b - a.max(dead_s)).max(0.0);
            if t + h >= s.on_s + s.w_s {
                *slot = None;
            }
        }
        if dm == 0.0 && self.film_kg == 0.0 {
            return 0.0;
        }
        self.vapour_kg += (1.0 - x[0] - x[1]) * dm;
        for i in 0..2 {
            for c in 0..3 {
                self.films_c[i][c] += x[i] * wc[c] * dm;
                let mut ev = self.films_c[i][c] * evap[i][c];
                self.films_c[i][c] -= ev;
                if self.films_c[i][c] < 1e-24 {
                    ev += self.films_c[i][c];
                    self.films_c[i][c] = 0.0;
                }
                self.vapour_kg += ev;
            }
            self.films_kg[i] = self.films_c[i].iter().sum();
        }
        self.film_kg = self.films_kg[0] + self.films_kg[1];
        self.injected_kg += dm;
        dm
    }
}

/// Boundary conditions for one engine step.
#[derive(Debug, Clone, Copy)]
pub struct EngineEnv {
    /// Plant time at the start of the step, s (sensor-edge time stamps).
    pub t_s: f64,
    pub p_amb_pa: f64,
    pub t_amb_k: f64,
    pub vehicle_speed_ms: f64,
    pub coolant_c: f64,
    pub oil_c: f64,
    pub battery_v: f64,
    /// Supply seen by the starter circuit (battery terminals): Thevenin source voltage and resistance with
    /// the other loads and the alternator accounted for (`starter` module).
    pub supply_v_th: f64,
    pub supply_r_th: f64,
    pub fuel_ron: f64,
    /// Mechanical torque absorbed by the alternator, Nm (from the electrical model).
    pub alternator_torque_nm: f64,
    /// Ethanol mass fraction of the fuel in the tank (0 = gasoline).
    pub fuel_ethanol: f64,
}

/// Sub-step averaged results (thermal model, telemetry).
#[derive(Debug, Clone, Default, Serialize)]
pub struct EngineFlows {
    /// Cycle-averaged gross indicated torque (compression + expansion strokes), Nm.
    pub torque_indicated_nm: f64,
    /// Mean friction torque (bearings, piston group, rest), Nm (filtered over ~2 cycles).
    pub torque_friction_nm: f64,
    /// Cycle-averaged gas-exchange (pumping) torque loss, Nm.
    pub torque_pumping_nm: f64,
    /// Accessory torque (alternator, oil pump, HPFP drive), Nm.
    pub torque_aux_nm: f64,
    /// Mean camshaft drive torque (valve springs, valve gas forces, lash adjusters), Nm.
    pub torque_valvetrain_nm: f64,
    pub torque_starter_nm: f64,
    pub starter_current_a: f64,
    /// Fresh air inducted by the cylinders, through the throttle, through the compressor, kg/s.
    pub air_cyl_kg_s: f64,
    pub air_throttle_kg_s: f64,
    pub air_compressor_kg_s: f64,
    pub exhaust_kg_s: f64,
    pub fuel_kg_s: f64,
    /// Mean trapped fresh air per cylinder and cycle, mg.
    pub air_per_cyl_mg: f64,
    pub lambda_cyl: [f64; 4],
    pub fuel_per_cyl_mg: [f64; 4],
    /// Livengood–Wu integral reached in the last cycle (≥ 1 = end-gas autoignition).
    pub knock_integral: [f64; 4],
    pub firing: [bool; 4],
    pub wall_heat_w: f64,
    pub oil_heat_w: f64,
    pub friction_power_w: f64,
    pub unburned_fuel_w: f64,
    pub excess_air_kg_s: f64,
    pub turbine_out_k: f64,
    pub compressor_out_k: f64,
    pub compressor_power_w: f64,
    pub turbine_power_w: f64,
    /// Pressure after the turbine (catalyst inlet) in the last step, Pa.
    pub turbine_out_pa: f64,
    pub coolant_pump_kg_s: f64,
    pub blowby_kg_s: f64,
    /// Heat released by unburned fuel oxidising in the exhaust manifold (afterburning), W.
    pub afterburn_w: f64,
}

/// Turbocharger rotor mechanics.
#[derive(Debug, Clone, Default, Serialize)]
pub struct TurboMech {
    /// Bore hoop stress / UTS at wheel temperature: compressor (AA2618) and turbine (IN713C).
    pub stress_ratio_c: f64,
    pub stress_ratio_t: f64,
    /// Low-cycle fatigue (compressor bore) and creep (turbine) life consumed.
    pub lcf_damage: f64,
    pub creep_damage: f64,
    pub burst: bool,
    /// Wheel metal temperatures (compressor, turbine), °C — lagged gas temperatures (τ ≈ 3 s).
    pub t_wheel_c: [f64; 2],
    sig_ext: f64,
    sig_other: f64,
    rising: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct EngineState {
    /// Crank (DMF primary) speed, rad/s; may go negative briefly (compression bounce).
    pub omega_rad_s: f64,
    /// Crank angle within the 720° cycle, cylinder 1 firing TDC at 0°.
    pub crank_deg: f64,
    /// Unwrapped crank angle, deg.
    pub crank_total_deg: f64,
    /// DMF secondary (gearbox side) speed, rad/s, and arc-spring twist primary − secondary, rad.
    pub omega2_rad_s: f64,
    pub dmf_twist_rad: f64,
    pub p_intake_pa: f64,
    pub p_boost_pa: f64,
    pub p_exhaust_pa: f64,
    pub t_boost_k: f64,
    pub t_intake_k: f64,
    pub t_exhaust_k: f64,
    /// Exhaust-manifold gas masses [air, fuel, products], kg.
    pub exh_m: [f64; 3],
    pub turbo_omega_rad_s: f64,
    /// Throttle plate angle from the lower mechanical stop, deg, and its rate, rad/s.
    pub throttle_deg: f64,
    pub throttle_rate_rad_s: f64,
    pub wastegate_open: f64,
    /// Electric wastegate actuator (output shaft, motor current).
    pub wg: WastegateActuator,
    /// Port (tumble) flap shaft position, 0 = open, 1 = closed.
    pub port_flap_closed: f64,
    /// Crankcase gas the ventilation returns into the intake manifold behind the throttle (blow-by), not air: its fuel
    /// and combustion products in the manifold, kg. The cylinders induct them with the manifold gas.
    pub im_pcv_kg: [f64; 2],
    pub rail_pa: f64,
    /// Low-pressure fuel gauge pressure, Pa.
    pub lp_fuel_pa: f64,
    /// Main oil gallery pressure (gauge), Pa.
    pub oil_pressure_pa: f64,
    /// λ of the mixed exhaust-manifold gas leaving towards the turbine (input of
    /// [`crate::lambda_sensor::ExhaustSensors`], which adds the transport to the sensors).
    pub lambda_exh: f64,
    /// Turbine-inlet thermocouple (τ ≈ 0.15 s lag of the pulsating manifold gas temperature), K.
    pub egt_k: f64,
    pub cyl: [Cylinder; 4],
    pub vt: Valvetrain,
    pub lube: Lubrication,
    pub turbo: TurboMech,
    /// Crank seized / destroyed: cannot rotate.
    pub locked: bool,
    /// Starter motor (armature speed, current, clutch state).
    pub starter: Starter,
    /// Peak-to-peak crank speed over the last 720°, rpm.
    pub speed_ripple_rpm: f64,
    /// Chronological physical failure log.
    pub failures: Vec<String>,
    pub flows: EngineFlows,
    pub kin: Kinematics,
    /// Sensor edges produced during the last plant step (cleared by the plant every step).
    #[serde(skip)]
    pub edges: Vec<SensorEdge>,
    /// Read-only acoustic probe (disabled by default; see `acoustic_probe`).
    #[serde(skip)]
    pub probe: Probe,
    /// Block vibration at the knock-sensor seats (`knock_sensor`; the ECU board samples it).
    #[serde(skip)]
    pub vib: BlockVibration,
    #[serde(skip)]
    sl: [Slider; 4],
    cam_psi: [f64; 2],
    hpfp_psi: f64,
    hpfp_closed: bool,
    /// Plunger volume still to be displaced before the outlet valve opens in this stroke, m³.
    hpfp_loss_m3: f64,
    /// N276 coil current on, time since its last current-on / current-off edge, s.
    vcv_current: bool,
    vcv_since_on: f64,
    vcv_since_off: f64,
    /// Intake ports of cylinders 1..4 (MPI fuel, wall film).
    pub port: [PortFuel; 4],
    /// Fuel dissolved in the engine oil (liner wetting scraped down by the rings), kg.
    pub oil_fuel_kg: f64,
    cam_torque_nm: f64,
    bearing_fric_nm: f64,
    hpfp_torque_nm: f64,
    fric_filt_nm: f64,
    w_min: f64,
    w_max: f64,
    reported: [u16; 5],
}

/// Piston ring pack + skirt friction force (N, magnitude): boundary/mixed ring friction with gas
/// loading behind the top ring (Stribeck-like μ(v)), viscous ring and skirt shear, skirt side thrust.
/// Ring tension, contact widths and film thicknesses UNVERIFIED.
#[inline]
fn piston_friction(p_gauge: f64, side_n: f64, v_p: f64, mu_oil: f64, bore: f64) -> f64 {
    let v = v_p.abs();
    let mu_b = 0.02 + 0.10 * (-v / 0.3).exp();
    let n_ring = 250.0 + p_gauge.max(0.0) * PI * bore * 1.0e-3;
    let shear = mu_oil * v * (PI * bore * 3e-3 / 2e-6 + 2.0e-3 / 10e-6);
    mu_b * n_ring + shear + 0.03 * side_n.abs()
}

impl EngineState {
    pub fn new(p: &EngineProfile, p_amb_pa: f64, t_amb_k: f64) -> Self {
        let kin = Kinematics::new(p);
        let seed = p.mech.cyl_spread.seed;
        let cyl = std::array::from_fn(|c| Cylinder::new(&kin, (-FIRING_OFFSET_DEG[c]).rem_euclid(720.0), p_amb_pa, t_amb_k, seed ^ (0x1000 + c as u64)));
        let sl = std::array::from_fn(|c| kin.eval((-FIRING_OFFSET_DEG[c]).rem_euclid(720.0).to_radians(), 0.0));
        let m_exh = p_amb_pa * p.exhaust_volume_m3 / (R_E * t_amb_k);
        Self {
            omega_rad_s: 0.0,
            crank_deg: 0.0,
            crank_total_deg: 0.0,
            omega2_rad_s: 0.0,
            dmf_twist_rad: 0.0,
            p_intake_pa: p_amb_pa,
            p_boost_pa: p_amb_pa,
            p_exhaust_pa: p_amb_pa,
            t_boost_k: t_amb_k,
            t_intake_k: t_amb_k,
            t_exhaust_k: t_amb_k,
            exh_m: [m_exh, 0.0, 0.0],
            turbo_omega_rad_s: 0.0,
            throttle_deg: p.throttle_limp_home_deg,
            throttle_rate_rad_s: 0.0,
            wastegate_open: WastegateActuator::at_rest(&p.turbo.wg_actuator).open_fraction(&p.turbo.wg_actuator),
            wg: WastegateActuator::at_rest(&p.turbo.wg_actuator),
            port_flap_closed: 0.0,
            im_pcv_kg: [0.0; 2],
            rail_pa: p_amb_pa,
            lp_fuel_pa: 0.0,
            oil_pressure_pa: 0.0,
            lambda_exh: 10.0,
            egt_k: t_amb_k,
            cyl,
            vt: Valvetrain::new(p, &kin),
            lube: Lubrication::new(p, t_amb_k),
            turbo: TurboMech { t_wheel_c: [t_amb_k - 273.15; 2], ..Default::default() },
            locked: false,
            starter: Starter::default(),
            speed_ripple_rpm: 0.0,
            failures: Vec::new(),
            flows: EngineFlows::default(),
            kin,
            edges: Vec::new(),
            probe: Probe::default(),
            vib: BlockVibration::default(),
            sl,
            cam_psi: [0.0, 0.0],
            hpfp_psi: 0.0,
            hpfp_closed: false,
            hpfp_loss_m3: 0.0,
            vcv_current: false,
            vcv_since_on: f64::INFINITY,
            vcv_since_off: f64::INFINITY,
            port: [PortFuel::default(); 4],
            oil_fuel_kg: 0.0,
            cam_torque_nm: 0.0,
            bearing_fric_nm: 0.0,
            hpfp_torque_nm: 0.0,
            fric_filt_nm: 0.0,
            w_min: 0.0,
            w_max: 0.0,
            reported: [0; 5],
        }
    }

    pub fn rpm(&self) -> f64 {
        self.omega_rad_s * 30.0 / PI
    }

    /// Test-cell equipment only (plant tests without an ECU; never used by the emulator): actuator angle at which the
    /// wastegate passes the fraction `f` of its fully open flow at the present gas state, by the plant's own flow law
    /// (with `TurboHardware::wastegate_map` the port orifice no longer sets the flow at part opening).
    pub fn bench_wastegate_theta(&self, tb: &crate::profiles::TurboHardware, f: f64) -> f64 {
        let hw = &tb.wg_actuator;
        let (p3, p4, t3) = (self.p_exhaust_pa, self.flows.turbine_out_pa, self.t_exhaust_k);
        let Some(wm) = tb.wastegate_map.filter(|_| f > 0.0 && p3 > p4) else { return WastegateActuator::theta_for_open_fraction(hw, f) };
        let lft1 = self.vt.avs.iter().map(|c| c.pos).sum::<f64>() >= 2.0;
        let flow = |th: f64| {
            let a = WastegateActuator { theta_rad: th, ..Default::default() };
            orifice_flow(tb.wastegate_cda_m2 * a.open_fraction(hw), p3, t3, p4, t3, GAMMA_EXH).min(wm.mass_flow(a.sensor_v(hw), hw.sensor_v[0], p3, p4, t3, lft1))
        };
        let target = f.min(1.0) * flow(hw.travel_rad);
        let (mut lo, mut hi) = (0.0, hw.travel_rad);
        for _ in 0..30 {
            let mid = 0.5 * (lo + hi);
            if flow(mid) < target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        hi
    }

    pub fn turbo_rpm(&self) -> f64 {
        self.turbo_omega_rad_s * 30.0 / PI
    }

    /// Sets crank and DMF secondary to the same speed (test benches / initial conditions).
    pub fn set_speed(&mut self, omega: f64) {
        let w = if omega.is_finite() { omega } else { 0.0 };
        self.omega_rad_s = w;
        self.omega2_rad_s = w;
        self.dmf_twist_rad = 0.0;
    }

    fn fail(&mut self, t: f64, slot: usize, bit: u16, msg: String) {
        if self.reported[slot] & bit == 0 {
            self.reported[slot] |= bit;
            self.failures.push(format!("t={t:.2} s: {msg}"));
        }
    }

    fn fail_with(&mut self, t: f64, slot: usize, bit: u16, lock: bool, msg: impl FnOnce() -> String) {
        if self.reported[slot] & bit == 0 {
            self.reported[slot] |= bit;
            self.failures.push(format!("t={t:.2} s: {}", msg()));
        }
        if lock {
            self.locked = true;
        }
    }

    /// Electronic throttle: DC motor at mean bridge voltage `v_motor` (`None` = bridge off, no current; at 0 V
    /// the bridge shorts the motor) + gear against the limp-home spring pair, Coulomb friction, stops.
    fn step_throttle(&mut self, etc: &EtcActuator, lh_deg: f64, v_motor: Option<f64>, dt: f64) {
        if !(dt > 0.0 && dt.is_finite()) {
            return;
        }
        if dt > 5e-4 {
            let n = (dt / 2e-4).ceil().clamp(1.0, 10_000.0) as usize;
            let h = dt / n as f64;
            for _ in 0..n {
                self.step_throttle_one(etc, lh_deg, v_motor, h);
            }
        } else {
            self.step_throttle_one(etc, lh_deg, v_motor, dt);
        }
    }

    fn step_throttle_one(&mut self, etc: &EtcActuator, lh_deg: f64, v_motor: Option<f64>, dt: f64) {
        let w = self.throttle_rate_rad_s;
        let t_motor = v_motor.map_or(0.0, |v| etc.k_nm_a * (v - etc.k_nm_a * w) / etc.r_ohm);
        let x = (self.throttle_deg - lh_deg).to_radians();
        let t_spring = if x > 0.0 {
            -(etc.close_preload_nm + etc.close_rate_nm_rad * x)
        } else if x < 0.0 {
            etc.open_preload_nm - etc.open_rate_nm_rad * x
        } else {
            0.0
        };
        let t = t_motor + t_spring;
        // At rest it moves only once the drive beats the friction (at limp-home also the preload of the side
        // it moves into).
        let (lo, hi) = if x == 0.0 { (-etc.open_preload_nm, etc.close_preload_nm) } else { (0.0, 0.0) };
        if w == 0.0 && t >= lo - etc.friction_nm && t <= hi + etc.friction_nm {
            return;
        }
        let f = etc.friction_nm * if w != 0.0 { w.signum() } else { t.signum() };
        let mut w1 = w + (t - f) / etc.inertia_kgm2 * dt;
        if w != 0.0 && w1.signum() != w.signum() {
            w1 = 0.0; // friction stops it; no reversal within one step
        }
        let mut th = self.throttle_deg + w1.to_degrees() * dt;
        // Crossing limp-home against a drive weaker than the far side's preload parks the plate there.
        if x * (th - lh_deg) < 0.0 {
            let far = if th > lh_deg { etc.close_preload_nm } else { etc.open_preload_nm };
            if t_motor.abs() < far + etc.friction_nm {
                (th, w1) = (lh_deg, 0.0);
            }
        }
        if th <= 0.0 || th >= etc.max_deg {
            (th, w1) = (th.clamp(0.0, etc.max_deg), 0.0);
        }
        self.throttle_deg = th;
        self.throttle_rate_rad_s = w1;
    }

    /// Advances the engine by `dt` with `load_nm` braking the DMF secondary (clutch / absorber) and
    /// `extra_j2` additional inertia on the secondary (dyno rotor).
    pub fn step(&mut self, p: &EngineProfile, a: &EcuOutputs, env: &EngineEnv, load_nm: f64, extra_j2: f64, dt: f64) {
        if !(dt > 0.0 && dt.is_finite()) {
            return;
        }
        if !self.omega_rad_s.is_finite() {
            self.omega_rad_s = 0.0;
        }
        if !self.omega2_rad_s.is_finite() {
            self.omega2_rad_s = 0.0;
        }
        let powered = a.powered();
        let n_rpm = self.rpm().abs();
        let t_amb = env.t_amb_k;
        let t_cool_k = env.coolant_c + 273.15;
        let t_oil_k = env.oil_c + 273.15;
        let kin = self.kin;
        let mut f = EngineFlows::default();

        // ---- Actuators (first-order electromechanical lags) ----------------------------
        // H-bridge supplied through the main relay: mean motor voltage = duty × battery voltage.
        let v_etc = a.etc_duty.filter(|_| powered).map(|d| d.clamp(-1.0, 1.0) * env.battery_v);
        self.step_throttle(&p.etc, p.throttle_limp_home_deg, v_etc, dt);

        // ---- Compressor (sub-step) -------------------------------------------------------
        let tb = &p.turbo;
        let turbo_alive = !self.turbo.burst && !self.lube.turbo_seized;
        let m_c_prev = self.flows.air_compressor_kg_s.max(0.0);
        let gp = &p.gas_path;
        let p1 = env.p_amb_pa - gp.air_filter_k * m_c_prev * m_c_prev;
        // Compressor inlet: ambient air, plus engine-bay air for an open filter (heat soak).
        let t1 = t_amb + gp.inlet_rise_k(t_amb, env.coolant_c, env.vehicle_speed_ms);
        let pr_c = (self.p_boost_pa + gp.charge_air_dp_rel_k * m_c_prev * m_c_prev) / p1;
        let nr = if turbo_alive { self.turbo_rpm() / tb.n_ref_rpm } else { 0.0 };
        let mut eta_c = tb.eta_compressor;
        let choke = tb.choke_flow_ref_kg_s * nr;
        let mut pr_max = 1.0 + (tb.pr_zero_flow_ref - 1.0) * nr * nr;
        let corr = (p1 / 101_325.0) / (t1 / 298.15).sqrt();
        let m_c = if let Some(map) = tb.map {
            // Table map (`compressor_map`): flow at this pressure ratio and speed, standardised by √(T_ref/T₁) and
            // volumetric at the inlet; the stationary wheel passes air like the generic model.
            let fac = (map.t_ref_k / t1).sqrt();
            let n_std = nr * tb.n_ref_rpm * fac;
            pr_max = map.pr_zero_flow(n_std);
            let flow = |q: f64| q / fac * p1 / (R_AIR * t1);
            match map.q_std(pr_c.max(1.0), n_std) {
                Some(q) if pr_c > 1.0 => {
                    eta_c = (map.eta(pr_c, q) - tb.eta_loss).max(0.2);
                    flow(q)
                }
                q if pr_c <= 1.0 => flow(q.unwrap_or(0.0)) + orifice_flow(COMPRESSOR_INDUCER_CDA_M2, p1, t1, self.p_boost_pa, t1, GAMMA_AIR),
                _ => orifice_flow(1e-4, p1, t1, self.p_boost_pa, self.t_boost_k, GAMMA_AIR),
            }
        } else if pr_c <= 1.0 {
            choke * corr + orifice_flow(COMPRESSOR_INDUCER_CDA_M2, p1, t1, self.p_boost_pa, t1, GAMMA_AIR)
        } else if pr_c < pr_max {
            let x = (pr_c - 1.0) / (pr_max - 1.0);
            let flow_frac = (1.0 - x * x).sqrt();
            eta_c = (tb.eta_compressor * (1.0 - 1.2 * (flow_frac - 0.6).powi(2))).max(0.35);
            choke * corr * flow_frac
        } else {
            orifice_flow(1e-4, p1, t1, self.p_boost_pa, self.t_boost_k, GAMMA_AIR)
        };
        let t_c_out = if pr_c > 1.0 { t1 * (1.0 + (pr_c.powf((GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0) / eta_c) } else { t1 };
        let p_comp = m_c.max(0.0) * CP_AIR * (t_c_out - t1);
        f.air_compressor_kg_s = m_c;
        f.compressor_out_k = t_c_out;
        let probe_on = self.probe.enabled;
        let mut pf = CrankFrame { m_c: m_c as f32, pr_c_rel: (pr_c / pr_max) as f32, ..Default::default() };
        f.compressor_power_w = p_comp;

        // ---- Intercooler / charge temperatures ------------------------------------------
        let eps_ic = p.intercooler_effectiveness * (0.55 + 0.45 * (env.vehicle_speed_ms / 25.0).min(1.0));
        let t_ic_out = t_c_out - eps_ic * (t_c_out - t_amb);
        self.t_boost_k += (t_ic_out - self.t_boost_k) * (dt / 3.0).min(1.0);
        let t_im_target = manifold_air_temp_k(p, self.flows.air_throttle_kg_s, self.t_boost_k, t_cool_k);
        self.t_intake_k += (t_im_target - self.t_intake_k) * (dt / 1.0).min(1.0);

        let cda_thr = (crate::sensors::interp(self.throttle_deg, p.throttle_cda_cm2) - p.throttle_deposit_cm2).max(0.0) * 1e-4;
        let dv_open = powered && a.diverter_open.unwrap_or(false);

        // ---- Low-pressure fuel ------------------------------------------------------------
        let lp_target = match (powered, a.lpfp_duty) {
            (true, Some(d)) => 0.6e6 * d.clamp(0.0, 1.0),
            _ => 0.0,
        };
        self.lp_fuel_pa += (lp_target - self.lp_fuel_pa) * (dt / 0.3).min(1.0);
        let feed = ((self.lp_fuel_pa - 0.1e6) / 0.2e6).clamp(0.0, 1.0);

        // ---- Lubrication network and phasers -----------------------------------------------
        let low_stage = powered && a.oil_pump_low_stage.unwrap_or(false);
        let jets_enabled = !(powered && a.piston_jets_off.unwrap_or(false));
        self.lube.step_gallery(p, n_rpm, env.oil_c, low_stage, jets_enabled, dt);
        let p_oil = self.lube.p_gallery_pa;
        self.oil_pressure_pa = p_oil;
        let shift0 = [self.vt.cam_shift_deg(p, 0), self.vt.cam_shift_deg(p, 1)];
        for (k, g) in [&p.mech.intake, &p.mech.exhaust].into_iter().enumerate() {
            self.vt.phaser[k].step(&g.phaser, g.phaser_range_deg, a.cam_phaser_duty[k], powered, p_oil, env.battery_v, env.oil_c, dt);
        }
        let shift1 = [self.vt.cam_shift_deg(p, 0), self.vt.cam_shift_deg(p, 1)];
        let jet = if self.lube.jets_flowing { ((p_oil - 1.8e5) / 2e5).clamp(0.0, 1.0).sqrt() } else { 0.0 };
        // Fuel in the oil evaporates from the hot sump (light ends first; τ ≈ 30 min at 90 °C, halving per +15 K —
        // UNVERIFIED order of magnitude of the oil-dilution recovery seen in GDI drive-cycle studies).
        self.oil_fuel_kg *= (-dt / (1800.0 * (-(env.oil_c - 90.0) / 15.0).exp())).exp();
        let oil_dilution = self.oil_fuel_kg / (self.lube.oil_l.max(0.5) * 0.85);
        let (afr, lhv, latent) = crate::combustion::fuel_blend(env.fuel_ethanol);
        // Coolant warms along the block jacket from its inlet: per-cylinder offset from the mean (wall heat over ṁ·cp).
        let sp = &p.mech.cyl_spread;
        let jacket_dt = self.flows.wall_heat_w.max(0.0) / (self.flows.coolant_pump_kg_s.max(0.05) * 3500.0);
        let t_cool_cyl: [f64; 4] = std::array::from_fn(|c| t_cool_k + jacket_dt * (((c as f64 - sp.coolant_inlet_pos).abs() + 0.5) / 4.0 - 0.5));
        // Port (tumble) flaps: pneumatic actuator behind N316 moves the flap shaft with a lag; de-energised = open
        // (UNVERIFIED rest position).
        let flap_cmd = if powered && a.tumble_flaps_closed.unwrap_or(false) { 1.0 } else { 0.0 };
        self.port_flap_closed += (flap_cmd - self.port_flap_closed) * (dt / PORT_FLAP_TAU_S).min(1.0);
        let tumble = self.port_flap_closed > 0.5;
        let mu_oil_ratio = (oil_viscosity_cst(env.oil_c) / oil_viscosity_cst(90.0)).clamp(0.5, 20.0);
        // Ring and skirt run as hydrodynamic slider bearings: a more viscous oil thickens their film as √(μ·U/load), so
        // the viscous force grows as √μ (Addison, PhD thesis Nottingham 2015, rings/skirt n = 0.5 after Sandoval & Heywood
        // SAE 2003-01-0725; Shayler et al. 0.3 for the piston group), not ∝ μ at fixed films. The film thicknesses of
        // `piston_friction` are the warm ones (liner 120 °C, mixed lubrication near the composite roughness), kept as the
        // floor: hotter oil cannot thin them further, warm friction is unchanged. Was 51× warm at −20 °C, now ≈ 7×.
        let mu_liner = {
            let (mu, mu_ref) = (film_viscosity(t_cool_k + 30.0), film_viscosity(MU_LINER_REF_K));
            if mu > mu_ref { (mu * mu_ref).sqrt() } else { mu }
        };
        let p_coolant = 1.0e5 + 0.6e5 * ((env.coolant_c - 20.0) / 80.0).clamp(0.0, 1.0);

        // ---- Starter / accessories ------------------------------------------------------
        // Starter relays J906/J907 in series feed the solenoid; the motor model is in `starter`.
        let starter_on = a.starter_relay.unwrap_or(false);
        let (mut st_t_e, mut st_i_e) = (0.0, 0.0);
        let nk = n_rpm / 1000.0;
        // Unmodelled friction (timing chain, balance shafts, valvetrain rolling, seals, water pump), FMEP kPa. UNVERIFIED
        // shape. The speed-independent 28 kPa it began with was exactly what the warm engine had too much against the
        // firmware's own loss model (SC8O40 `ip_tqfr_vvl_cam_port_h`, TQ_LOSS_ENG over UDS at idle and 2100 rpm:
        // +3.9 / +4.2 Nm) and is gone (docs/SIMOS18_COMBUSTION.md §18).
        let fmep_rest = (7.0 * nk + 1.6 * nk * nk) * mu_oil_ratio.powf(0.3) + 20.0 * (-n_rpm / 300.0).exp();
        let t_rest = fmep_rest * 1e3 * kin.v_disp * 4.0 / (4.0 * PI);
        let w_abs = self.omega_rad_s.abs().max(20.0);
        let t_pump = self.lube.pump_power_w / w_abs;
        let t_alt = env.alternator_torque_nm;
        f.coolant_pump_kg_s = (0.4 * n_rpm / 1000.0).min(2.5);

        // ---- Turbine / exhaust boundary -----------------------------------------------------
        let cda_t = if self.turbo.burst { 2.0 * tb.turbine_cda_m2 } else { tb.turbine_cda_m2 };
        let m_out_prev = self.flows.exhaust_kg_s.max(0.0);
        let p4 = env.p_amb_pa + p.gas_path.exhaust_k * m_out_prev * m_out_prev;
        // Electric wastegate actuator (H-bridge through the main relay; + = towards the seat) against its spring and
        // the exhaust pressure difference on the flap (`wastegate`).
        let v_wg = a.wastegate_duty.filter(|_| powered).map(|d| d.clamp(-1.0, 1.0) * env.battery_v);
        self.wg.step(&tb.wg_actuator, v_wg, self.p_exhaust_pa - p4, dt);
        self.wastegate_open = self.wg.open_fraction(&tb.wg_actuator);
        let cda_total = cda_t + tb.wastegate_cda_m2 * self.wastegate_open;
        let ua_iem = 10.0 * (m_out_prev.max(1e-3) / 0.05).powf(0.8);
        let v_rail = p.rail_volume_cc * 1e-6;
        let a_pl = p.hpfp_stroke_cc * 1e-6 / HPFP_LIFT_M;
        let vcv_pull_s = crate::sensors::interp(env.battery_v, p.hpfp_vcv_close_delay_ms) * 1e-3;
        let (vcv_rel_s, vcv_lock_pa) = (p.hpfp_vcv_release_ms * 1e-3, p.hpfp_vcv_lock_mpa * 1e6);
        // MPI: supply = low-pressure system; dead time vs battery; port film time constant at the head temperature.
        let mpi_dead_s = crate::sensors::interp(env.battery_v, p.mpi_dead_time_ms) * 1e-3;
        let film = &p.port_film;
        let film_x = film.x.map(|x| (x * (1.0 + film.ivd_factor)).min(0.45));
        let film_tau_s = film.tau_comp_s(env.coolant_c, self.p_intake_pa / 1e3, self.flows.air_cyl_kg_s);
        let fuel_wc = crate::combustion::fuel_components(env.fuel_ethanol);
        let mut m_mpi = 0.0;
        let inj_q = p.injector_flow_cc_s * 1e-6;
        let dead_s = p.injector_dead_time_ms * 1e-3;
        let j2 = p.mech.dmf_secondary_kgm2 + extra_j2;

        // ---- Block vibration: rotating-machinery impacts (knock-sensor background) --------
        self.vib.background(&p.knock, env.t_s, env.t_s + dt, self.omega_rad_s);
        self.vib.prune(env.t_s - 0.05);

        // ---- Crank-angle loop -----------------------------------------------------------
        let n_sub = (self.omega_rad_s.abs() * dt / CRANK_STEP_RAD).ceil().max((dt / MAX_STEP_S).ceil()).clamp(1.0, 100_000.0) as usize;
        let h = dt / n_sub as f64;
        let film_evap = film_tau_s.map(|f| f.map(|tau| 1.0 - (-h / tau).exp()));
        let mu_oil_b = (oil_viscosity_cst(t_oil_k - 273.15) / oil_viscosity_cst(90.0)).max(0.05);
        let (mut m_thr_sum, mut m_dv_sum, mut m_out_sum, mut e_turb, mut fric_e, mut m_cyl_air) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let (mut q_cool, mut q_oil, mut v_fuel_sum, mut m_bb_sum, mut fuel_out, mut air_out, mut t4_e) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let mut aux_e = 0.0;
        let mut cam_e = 0.0;
        let mut ab_e = 0.0;
        for j in 0..n_sub {
            let t_now = env.t_s + j as f64 * h;
            let w = self.omega_rad_s;

            // Torques at the current state.
            let mut j_eff = p.crank_inertia_kgm2;
            let mut t_gas = 0.0;
            let mut t_ring = 0.0;
            for c in 0..4 {
                let s = self.sl[c];
                let pg = self.cyl[c].p - env.p_amb_pa;
                let fg = pg * kin.area;
                t_gas += fg * s.dx - kin.m_rec * s.dx * s.ddx * w * w;
                j_eff += kin.m_rec * s.dx * s.dx;
                let side = fg * s.sin_b / s.cos_b;
                t_ring += piston_friction(pg, side, s.dx * w, mu_liner, kin.bore) * s.dx.abs();
            }
            let t_spring = crank_train::dmf_torque(p, self.dmf_twist_rad, w - self.omega2_rad_s);
            pf.t_gas_nm = t_gas as f32;
            let mut t_drive = t_gas - t_spring - self.cam_torque_nm - self.hpfp_torque_nm;
            let t_fric = t_ring + self.bearing_fric_nm + t_rest + t_pump + t_alt;
            // Starter through the overrunning clutch: it carries torque only while the pinion drives, i.e.
            // while the clutch torque of the coupled system stays positive; otherwise the crank runs ahead.
            let (t_st, j_st) = self.starter.drive(&STARTER_1_1KW, starter_on, env.supply_v_th, env.supply_r_th, w);
            if j_st > 0.0 {
                let t_net = t_drive - if w == 0.0 { 0.0 } else { t_fric * w.signum() };
                if j_eff * t_st > j_st * t_net {
                    t_drive += t_st;
                    j_eff += j_st;
                    st_t_e += t_st * h;
                } else {
                    self.starter.engaged = false;
                }
            }
            if self.locked {
                self.omega_rad_s = 0.0;
            } else if w == 0.0 {
                if t_drive.abs() > t_fric {
                    self.omega_rad_s = (t_drive - t_fric * t_drive.signum()) / j_eff * h;
                }
            } else {
                let w1 = w + (t_drive - t_fric * w.signum()) / j_eff * h;
                self.omega_rad_s = if w1 * w < 0.0 { 0.0 } else { w1 };
            }
            let w = self.omega_rad_s;
            self.starter.follow(&STARTER_1_1KW, w, h);
            st_i_e += self.starter.current_a * h;
            fric_e += (t_ring + self.bearing_fric_nm + t_rest) * h;
            aux_e += (t_pump + t_alt + self.hpfp_torque_nm) * h;
            cam_e += self.cam_torque_nm * h;
            self.omega2_rad_s += (t_spring - load_nm) / j2 * h;
            self.dmf_twist_rad += (w - self.omega2_rad_s) * h;

            // Crank angle and sensor wheels.
            let th0 = self.crank_total_deg;
            let th1 = th0 + w.to_degrees() * h;
            self.crank_total_deg = th1;
            let cyc_prev = self.crank_deg;
            self.crank_deg = th1.rem_euclid(720.0);
            let frac_j = (j + 1) as f64 / n_sub as f64;
            let cam_shift_at = |k: usize| {
                if shift1[k] == shift0[k] || j + 1 == n_sub {
                    shift1[k]
                } else {
                    shift0[k] + (shift1[k] - shift0[k]) * frac_j
                }
            };
            let e0 = self.edges.len();
            crank_train::crank_edges(th0, th1, t_now, h, &mut self.edges);
            for k in 0..2 {
                let psi = th1 - cam_shift_at(k);
                crank_train::cam_edges(k, self.cam_psi[k], psi, t_now, h, &mut self.edges);
                self.cam_psi[k] = psi;
            }
            if self.edges.len() - e0 > 1 {
                self.edges[e0..].sort_by(|x, y| x.t_s.total_cmp(&y.t_s));
            }
            if w == 0.0 {
                self.speed_ripple_rpm = 0.0;
                self.w_min = 0.0;
                self.w_max = 0.0;
            } else {
                self.w_min = self.w_min.min(w);
                self.w_max = self.w_max.max(w);
                if w > 0.0 && self.crank_deg < cyc_prev {
                    self.speed_ripple_rpm = (self.w_max - self.w_min) * 30.0 / PI;
                    self.w_min = w;
                    self.w_max = w;
                    for b in self.lube.bearings.iter_mut() {
                        b.film_last_m = b.film_run_m;
                        b.film_run_m = f64::MAX;
                    }
                }
            }

            // HPFP plunger (4-lobe cam on the exhaust camshaft) and N276 valve.
            let shift_ex = cam_shift_at(1);
            let psi1 = th1 - HPFP_TDC_DEG + 90.0 - shift_ex;
            let psi0 = self.hpfp_psi;
            self.hpfp_psi = psi1;
            let lift = |x: f64| 0.5 * HPFP_LIFT_M * (1.0 - (2.0 * PI * x / 180.0).cos());
            let (pm0, pm1) = (psi0.rem_euclid(180.0), psi1.rem_euclid(180.0));
            // N276 coil current: timed drives as the ECU switched them, or the decoded feed [on, off) repeated on every
            // pump lobe (no off angle = held to TDC).
            let current = match (powered, a.hpfp_close_deg) {
                (true, _) if a.timed.is_some() => a.timed.iter().flat_map(|t| t.hpfp.iter().flatten()).any(|d| d.on_s <= t_now + h && t_now + h < d.off_s),
                (true, Some(on)) => {
                    let on_p = (on - HPFP_TDC_DEG + 90.0 - shift_ex).rem_euclid(180.0);
                    let width = a.hpfp_off_deg.map_or((90.0 - on_p).rem_euclid(180.0), |off| off - on);
                    width >= 180.0 || (pm1 - on_p).rem_euclid(180.0) < width
                }
                _ => false,
            };
            if current && !self.vcv_current {
                self.vcv_since_on = 0.0;
            } else if !current && self.vcv_current {
                self.vcv_since_off = 0.0;
            }
            self.vcv_current = current;
            self.vcv_since_on += h;
            self.vcv_since_off += h;
            // Armature pulled in `t_pull` after current-on, back `t_rel` after the current is off (at the earliest
            // `t_rel` after it arrived).
            let retracted = self.vcv_since_on >= vcv_pull_s && (current || self.vcv_since_off < vcv_rel_s || self.vcv_since_on - vcv_pull_s < vcv_rel_s);
            let rising = pm1 < 90.0 && psi1 > psi0;
            if retracted && rising && !self.hpfp_closed {
                self.probe.impact(t_now + h, ImpactKind::HpfpValveClose, 0, 0, env.battery_v);
                self.hpfp_closed = true;
                let fup_hpa = (self.rail_pa - env.p_amb_pa) / 100.0;
                self.hpfp_loss_m3 = p.hpfp_loss.mg_at(w.abs() * 30.0 / PI, fup_hpa) * 1e-6 / FUEL_DENSITY_KG_M3;
            } else if self.hpfp_closed && !retracted && self.rail_pa - env.p_amb_pa < vcv_lock_pa {
                self.hpfp_closed = false; // not self-locking yet: the returning needle pushes the plate open
            }
            if pm0 < 90.0 && pm1 >= 90.0 || psi1 - psi0 >= 180.0 {
                if self.hpfp_closed {
                    self.probe.impact(t_now + h, ImpactKind::HpfpValveOpen, 0, 0, self.rail_pa);
                }
                self.hpfp_closed = false;
            }
            let mut v_pump = 0.0;
            let hpfp_out_prev = self.probe.hpfp_out;
            self.hpfp_torque_nm = 0.0;
            if self.hpfp_closed {
                let dh = if th1 > th0 && (pm1 - pm0).abs() < 90.0 { (lift(pm1) - lift(pm0)).max(0.0) } else { 0.0 };
                if dh > 0.0 {
                    let dv = a_pl * dh * p.hpfp_efficiency * feed;
                    let lost = dv.min(self.hpfp_loss_m3);
                    self.hpfp_loss_m3 -= lost;
                    v_pump = dv - lost;
                    if v_pump > 0.0 && !self.probe.hpfp_out {
                        self.probe.impact(t_now + h, ImpactKind::HpfpOutletOpen, 0, 0, self.rail_pa);
                    }
                    let dth = (th1 - th0).to_radians().max(1e-9);
                    self.hpfp_torque_nm = (self.rail_pa - self.lp_fuel_pa).max(0.0) * a_pl * (dh / dth).min(HPFP_LIFT_M);
                }
            }

            // Intake manifold: throttle and diverter on this step.
            let m_thr_raw = orifice_flow(cda_thr, self.p_boost_pa, self.t_boost_k, self.p_intake_pa, self.t_intake_k, GAMMA_AIR) * h;
            let dp_thr = self.p_boost_pa - self.p_intake_pa;
            let m_thr_eq = 0.5 * dp_thr.abs() / (R_AIR * 0.5 * (self.t_boost_k + self.t_intake_k) * (1.0 / p.intake_volume_m3 + 1.0 / p.boost_volume_m3));
            let m_thr = m_thr_raw.clamp(-m_thr_eq, m_thr_eq);
            m_thr_sum += m_thr;
            let dv0 = m_dv_sum;
            if dv_open {
                m_dv_sum += orifice_flow(DIVERTER_CDA_M2, self.p_boost_pa, self.t_boost_k, p1, t_amb, GAMMA_AIR) * h;
            }

            // Cylinders + valves.
            let m_im = (self.p_intake_pa * p.intake_volume_m3 / (R_AIR * self.t_intake_k)).max(1e-9);
            let [f_pcv, p_pcv] = self.im_pcv_kg.map(|m| (m / m_im).clamp(0.0, 0.5));
            let x_im = [1.0 - f_pcv - p_pcv, f_pcv, p_pcv];
            let m_em: f64 = self.exh_m.iter().sum();
            let x_em = if m_em > 0.0 { [self.exh_m[0] / m_em, self.exh_m[1] / m_em, self.exh_m[2] / m_em] } else { [1.0, 0.0, 0.0] };
            let mut d_im = m_thr;
            let mut d_em = [0.0; 3];
            let mut e_em = 0.0;
            let mut cam_t = 0.0;
            let mut throw = [Vector2::zeros(); 4];
            let mut rod_load = [0.0f64; 4];
            let mut v_inj = 0.0;
            let close_in = {
                let (_, d, centre, _) = self.vt.lobe(p, 0, 0);
                centre + 0.5 * d
            };
            let dp_mpi = (self.lp_fuel_pa + env.p_amb_pa - self.p_intake_pa).max(0.0);
            let q_mpi_base = p.mpi_injector_mg_ms * 1e-3 * (dp_mpi / (p.mpi_ref_dp_kpa * 1e3)).sqrt();
            let m_runner = self.p_intake_pa * film.port_volume_m3 / (R_AIR * self.t_intake_k);
            for c in 0..4 {
                let phi0 = (th0 - FIRING_OFFSET_DEG[c]).rem_euclid(720.0);
                let phi1 = (th1 - FIRING_OFFSET_DEG[c]).rem_euclid(720.0);
                let mut s = kin.eval(phi1.to_radians(), self.cyl[c].rod_short_m);
                let mut v1 = kin.volume(&s);
                // Hydrolock: the liquid cannot be compressed; the rod gives way, the crank pays.
                let v_liq = self.cyl[c].liquid_volume();
                if v_liq > 0.0 && v1 - v_liq < LIQUID_MIN_GAS_M3 {
                    let delta = (v_liq + LIQUID_MIN_GAS_M3 - v1) / kin.area;
                    let e = crank_train::rod_critical_load(p, self.cyl[c].rod_short_m) * delta;
                    let ke = 0.5 * j_eff * self.omega_rad_s * self.omega_rad_s;
                    self.omega_rad_s = self.omega_rad_s.signum() * (2.0 * (ke - e).max(0.0) / j_eff).sqrt();
                    self.cyl[c].rod_short_m += delta;
                    s = kin.eval(phi1.to_radians(), self.cyl[c].rod_short_m);
                    v1 = kin.volume(&s);
                    self.fail(t_now, c, 1 << 0, format!("cyl {}: hydrolock — liquid in the chamber, connecting rod bent", c + 1));
                }
                let w = self.omega_rad_s;
                let p_port = [self.p_intake_pa, self.p_exhaust_pa];
                let avs_on = a.avs_actuator[c].map(|x| powered && x.unwrap_or(false));
                cam_t += self.vt.step_cylinder(p, c, phi0, phi1, w, &s, self.cyl[c].p, p_port, p_oil, avs_on, env.battery_v, h, t_now + h);
                if crate::cylinder::atdc(phi0) < 180.0 && crate::cylinder::atdc(phi1) >= 180.0 {
                    for v in self.vt.valve[c].iter_mut() {
                        v.float_last_m = v.float_run_m;
                        v.float_run_m = 0.0;
                    }
                }
                let lift_in = self.vt.flow_lift(p, c, 0);
                let lift_ex = self.vt.flow_lift(p, c, 1);
                let cda_in = self.vt.geo[0].cda(&p.mech.intake, lift_in) * if tumble { TUMBLE_FLOW_FACTOR } else { 1.0 } * sp.port_flow[c];
                let cda_ex = self.vt.geo[1].cda(&p.mech.exhaust, lift_ex);
                let cin = CylIn {
                    dt: h,
                    omega: w,
                    phi0,
                    phi1,
                    s,
                    v1,
                    cda_in,
                    cda_ex,
                    in_open: self.vt.valve[c][0].x > 0.0,
                    ex_open: self.vt.valve[c][1].x > 0.0,
                    p_im: self.p_intake_pa,
                    t_im: self.t_intake_k,
                    v_im: p.intake_volume_m3,
                    x_im,
                    p_em: self.p_exhaust_pa,
                    t_em: self.t_exhaust_k,
                    x_em,
                    v_em: p.exhaust_volume_m3,
                    p_cc: env.p_amb_pa,
                    p_coolant,
                    t_coolant_k: t_cool_cyl[c],
                    t_oil_k,
                    rail_pa: self.rail_pa,
                    coil: if powered { a.ignition[c] } else { None },
                    pulses: if powered { a.injection[c] } else { [None; 3] },
                    coil_timed: a.timed.as_ref().map(|t| if powered { t.coil[c] } else { [None; 3] }),
                    di_timed: a.timed.as_ref().map(|t| if powered { t.di[c] } else { [None; 3] }),
                    battery_v: env.battery_v,
                    ron: env.fuel_ron,
                    tumble,
                    jet,
                    injector_m3_s_10mpa: inj_q * sp.di_flow[c],
                    dead_s,
                    time_s: t_now,
                    afr,
                    lhv,
                    latent,
                    w_eth: env.fuel_ethanol,
                    deposit_m: p.mech.deposit_m * sp.deposit[c],
                    ring_factor: sp.ring_leak[c],
                    oil_dilution,
                };
                let o = self.cyl[c].step(&kin, &p.mech, &cin);
                let close_ex = {
                    let (_, d, centre, _) = self.vt.lobe(p, c, 1);
                    centre + 0.5 * d
                };
                let closes = [close_in, close_ex];
                let (p_c, t_c) = (self.cyl[c].p, self.cyl[c].t);
                let inj = o.inj_close.map(|x| (t_now + x, (self.rail_pa - p_c).max(0.0)));
                self.vib.cylinder_step(&p.knock, t_now + h, c, phi0, phi1, w, closes, p_c, o.knock_pa, t_c, kin.bore, inj);
                // Port injection (supply = low-pressure system, ΔP to the manifold) and induction of the port charge
                // with the runner slug drawn from the port (plug flow, `PortFuel::induct`).
                let pulses_mpi = if powered { a.port_injection[c] } else { [None; 3] };
                let timed_mpi = a.timed.as_ref().map(|t| if powered { t.mpi[c] } else { [None; 3] });
                let q_mpi = q_mpi_base * sp.mpi_flow[c];
                m_mpi += self.port[c].step(&pulses_mpi, timed_mpi.as_ref(), phi0, phi1, w, t_now, q_mpi, mpi_dead_s, film_x, fuel_wc, film_evap, h);
                let dm = self.port[c].induct(o.m_from_im, m_runner);
                if dm > 0.0 {
                    self.cyl[c].induct_port_fuel(dm, self.t_intake_k);
                }
                if probe_on {
                    let cy = &self.cyl[c];
                    pf.p_cyl[c] = cy.p as f32;
                    pf.t_cyl[c] = cy.t as f32;
                    pf.knock_pa[c] = cy.knock_amp_pa as f32;
                    pf.m_ex[c] = ((o.m_to_em[0] + o.m_to_em[1] + o.m_to_em[2]) / h) as f32;
                    pf.m_in[c] = (o.m_in_valve / h) as f32;
                    let vl = &self.vt.valve[c];
                    pf.valve_n[c] = (p.mech.intake.count * vl[0].cam_force_n + p.mech.exhaust.count * vl[1].cam_force_n) as f32;
                    let dp_inj = (self.rail_pa - cy.p).max(0.0);
                    if let Some(x) = o.inj_open {
                        self.probe.impact(t_now + x, ImpactKind::InjectorOpen, c, 0, self.rail_pa);
                    }
                    if let Some(x) = o.inj_close {
                        self.probe.impact(t_now + x, ImpactKind::InjectorClose, c, 0, dp_inj);
                    }
                    for (k, vlk) in vl.iter().enumerate() {
                        if vlk.seat_hit_v > 0.0 {
                            self.probe.impact(t_now + h, ImpactKind::ValveSeat, c, k as u8, vlk.seat_hit_v);
                        }
                    }
                }
                d_im += o.m_bb - o.m_from_im;
                self.im_pcv_kg[0] = (self.im_pcv_kg[0] + o.m_bb_x[FUEL] - o.m_from_im * x_im[FUEL]).max(0.0);
                self.im_pcv_kg[1] = (self.im_pcv_kg[1] + o.m_bb_x[PROD] - o.m_from_im * x_im[PROD]).max(0.0);
                m_cyl_air += o.m_from_im;
                m_bb_sum += o.m_bb;
                for (dem, &m_out) in d_em.iter_mut().zip(&o.m_to_em) {
                    *dem += m_out;
                }
                e_em += o.e_to_em;
                q_cool += o.q_coolant;
                q_oil += o.q_oil;
                v_inj += o.v_fuel;
                self.oil_fuel_kg += o.fuel_to_oil;

                // Connecting-rod force, buckling, crank-pin and main-bearing loads.
                let cy = &mut self.cyl[c];
                let fp = (cy.p - env.p_amb_pa) * kin.area - kin.m_rec * s.ddx * w * w;
                let f_rod = fp / s.cos_b;
                cy.record_rod_force(f_rod);
                let f_cr = crank_train::rod_critical_load(p, cy.rod_short_m);
                if f_rod > f_cr {
                    let v_gas = (cy.v - cy.liquid_volume()).max(1e-7);
                    cy.rod_short_m += (v_gas / kin.area * (f_rod / f_cr - 1.0)).min(1e-3);
                }
                let (sn, cs) = (s.sin_phi, s.cos_phi);
                throw[c] = Vector2::new(-f_rod * s.cos_b, f_rod * s.sin_b);
                let cf = kin.m_rot * kin.r * w * w;
                rod_load[c] = (throw[c] + Vector2::new(cf * cs, cf * sn)).norm();
                self.sl[c] = s;
            }
            self.cam_torque_nm = cam_t;

            // Bearings.
            let mut fr = 0.0;
            for b in 0..N_BEARINGS {
                let load = if b < 5 {
                    let l = if b > 0 { throw[b - 1] } else { Vector2::zeros() };
                    let r = if b < 4 { throw[b] } else { Vector2::zeros() };
                    0.5 * (l + r).norm()
                } else {
                    rod_load[b - 5]
                };
                fr += self.lube.step_bearing_with_mu(p, b, load, self.omega_rad_s, t_oil_k, mu_oil_b, h);
            }
            self.bearing_fric_nm = fr;

            // Rail.
            v_fuel_sum += v_inj;
            let q_leak = 1e-9 * (self.rail_pa / 1e7) * h;
            self.rail_pa += FUEL_BULK_MODULUS_PA / v_rail * (v_pump - v_inj - q_leak);
            self.rail_pa = self.rail_pa.clamp(env.p_amb_pa + self.lp_fuel_pa, p.rail_prv_mpa * 1e6);

            // Intake manifold (isothermal plenum).
            self.p_intake_pa = (self.p_intake_pa + R_AIR * self.t_intake_k / p.intake_volume_m3 * d_im).max(1_000.0);

            // Exhaust manifold: species, energy, turbine + wastegate outflow, IEM heat to coolant.
            let m_em: f64 = self.exh_m.iter().sum();
            let mut e = m_em * CV_E * self.t_exhaust_k + e_em;
            for (exh, &dem) in self.exh_m.iter_mut().zip(&d_em) {
                *exh = (*exh + dem).max(0.0);
            }
            let m_em: f64 = self.exh_m.iter().sum::<f64>().max(1e-9);
            // Afterburning: unburned fuel (partial burns, misfires, crevice HC) with the oxygen of lean cylinders / the
            // excess air oxidises in the hot manifold (Westbrook–Dryer global rate) and heats the turbine gas.
            if self.exh_m[FUEL] > 0.0 && self.exh_m[AIR] > 0.0 {
                let fr = crate::combustion::afterburn_fraction(self.t_exhaust_k, self.p_exhaust_pa, self.exh_m[FUEL] / m_em, 0.232 * self.exh_m[AIR] / m_em, h);
                let dm = (fr * self.exh_m[FUEL]).min(self.exh_m[AIR] / afr);
                if dm > 0.0 {
                    self.exh_m[FUEL] -= dm;
                    self.exh_m[AIR] -= afr * dm;
                    self.exh_m[PROD] += (1.0 + afr) * dm;
                    e += lhv * dm;
                    ab_e += lhv * dm;
                }
            }
            // Turbine + wastegate flow from the manifold's own gas pressure, both ways: when the cylinders have drawn the
            // manifold below the downstream pressure (overlap back-flow at low load) gas of the manifold's composition
            // returns at the turbine-outlet temperature (before, the outflow was evaluated at a pressure floored at
            // 0.9·p_amb with no return flow, which held the idle manifold below ambient).
            let t_down = self.flows.turbine_out_k.max(t_amb);
            let t_gas = (e / (m_em * CV_E)).clamp(t_amb.min(250.0), 2500.0);
            let p_gas = m_em * R_E * t_gas / p.exhaust_volume_m3;
            // the quasi-steady back-pressure k·ṁ² of the downstream system only acts on forward flow; returning gas comes
            // from the tailpipe side at ambient pressure
            let p_dn = if p_gas >= p4 { p4 } else { env.p_amb_pa.min(p4) };
            let cap = 0.5 * (p_gas - p_dn).abs() * p.exhaust_volume_m3 / (R_E * t_gas.max(t_down));
            // forward flow: turbine nozzle + wastegate (the port orifice, limited to the calibration's own model when the
            // turbocharger has one, `wastegate_map`)
            let (m_fwd, tur_share) = match tb.wastegate_map {
                Some(wm) if p_gas > p_dn => {
                    let lft1 = self.vt.avs.iter().map(|c| c.pos).sum::<f64>() >= 2.0;
                    let m_t = orifice_flow(cda_t, p_gas, t_gas, p_dn, t_down, GAMMA_EXH);
                    let m_w = orifice_flow(tb.wastegate_cda_m2 * self.wastegate_open, p_gas, t_gas, p_dn, t_down, GAMMA_EXH)
                        .min(wm.mass_flow(self.wg.sensor_v(&tb.wg_actuator), tb.wg_actuator.sensor_v[0], p_gas, p_dn, t_gas, lft1));
                    (m_t + m_w, m_t / (m_t + m_w).max(1e-12))
                }
                _ => (orifice_flow(cda_total, p_gas, t_gas, p_dn, t_down, GAMMA_EXH), cda_t / cda_total),
            };
            let dm = if p_gas >= p4 || p_gas < p_dn { (m_fwd * h).clamp(-cap, cap.min(0.5 * m_em)) } else { 0.0 };
            let m_out = dm.max(0.0);
            if dm < 0.0 {
                for m_s in &mut self.exh_m {
                    *m_s -= dm * *m_s / m_em;
                }
                e -= CP_E * t_down * dm;
            }
            let m_em: f64 = self.exh_m.iter().sum::<f64>().max(1e-9);
            let t_e = t_gas;
            e -= CP_E * t_e * m_out;
            let q_iem = ua_iem * (t_e - t_cool_k).max(0.0) * h;
            e -= q_iem;
            q_cool += q_iem;
            fuel_out += m_out * self.exh_m[FUEL] / m_em;
            air_out += m_out * self.exh_m[AIR] / m_em;
            for m_s in &mut self.exh_m {
                *m_s *= 1.0 - m_out / m_em;
            }
            let m_em = (m_em - m_out).max(1e-9);
            self.t_exhaust_k = (e / (m_em * CV_E)).clamp(t_amb.min(250.0), 2500.0);
            self.p_exhaust_pa = (m_em * R_E * self.t_exhaust_k / p.exhaust_volume_m3).max(0.3 * env.p_amb_pa);
            m_out_sum += m_out;
            let m_t = m_out * tur_share;
            let pr_t = (self.p_exhaust_pa / p4).max(1.0);
            let dh_frac = 1.0 - pr_t.powf(-(GAMMA_EXH - 1.0) / GAMMA_EXH);
            let c_s = (2.0 * CP_EXH * t_e * dh_frac).sqrt().max(1.0);
            let sr = self.turbo_omega_rad_s.max(200.0) * tb.turbine_tip_radius_m / c_s / 0.7;
            let eta_t = (tb.eta_turbine * (2.0 * sr - sr * sr)).max(0.0);
            let de = if turbo_alive { m_t * CP_EXH * t_e * eta_t * dh_frac } else { 0.0 };
            e_turb += de;
            t4_e += (m_out * t_e - de / CP_EXH).max(0.0);
            if probe_on {
                self.probe.hpfp_out = v_pump > 0.0 || (hpfp_out_prev && self.hpfp_closed);
                pf.t_s = t_now + h;
                pf.h = h as f32;
                pf.crank_deg = self.crank_deg as f32;
                pf.omega = self.omega_rad_s as f32;
                pf.m_out = (m_out / h) as f32;
                pf.m_thr = (m_thr / h) as f32;
                pf.m_dv = ((m_dv_sum - dv0) / h) as f32;
                pf.p_em = self.p_exhaust_pa as f32;
                pf.t_em = self.t_exhaust_k as f32;
                pf.p_im = self.p_intake_pa as f32;
                pf.t_im = self.t_intake_k as f32;
                pf.p_boost = self.p_boost_pa as f32;
                pf.p4 = p4 as f32;
                pf.cam_nm = self.cam_torque_nm as f32;
                pf.hpfp_nm = self.hpfp_torque_nm as f32;
                pf.rail_pa = self.rail_pa as f32;
                pf.battery_v = env.battery_v as f32;
                pf.turbo_rad_s = self.turbo_omega_rad_s as f32;
                pf.starter_rad_s = self.starter.omega_m as f32;
                pf.starter_a = self.starter.current_a as f32;
                pf.starter_on = starter_on;
                pf.starter_engaged = self.starter.engaged;
                self.probe.frames.push(pf);
            }
        }
        let sum_ms = |x: f64| x / dt;

        // ---- Boost volume, turbo rotor -------------------------------------------------------
        self.p_boost_pa += R_AIR * self.t_boost_k / p.boost_volume_m3 * (m_c * dt - m_thr_sum - m_dv_sum);
        self.p_boost_pa = self.p_boost_pa.max(1_000.0);
        let p_turb = sum_ms(e_turb);
        f.turbine_power_w = p_turb;
        let tb_fric = self.lube.step_turbo_bearing(self.turbo_omega_rad_s, self.t_exhaust_k, t_cool_k, t_oil_k, dt);
        if turbo_alive && !self.lube.turbo_seized {
            let w = self.turbo_omega_rad_s;
            let dw = (p_turb - p_comp - tb_fric) / (tb.inertia_kgm2 * w.max(200.0));
            self.turbo_omega_rad_s = (w + dw * dt).max(0.0);
        } else {
            self.turbo_omega_rad_s = 0.0;
        }
        self.egt_k += (self.t_exhaust_k - self.egt_k) * (dt / 0.15).min(1.0);
        self.step_turbo_mech(p, f.compressor_out_k, env.t_s, dt);

        // ---- Flows / diagnostics -------------------------------------------------------------
        f.air_throttle_kg_s = sum_ms(m_thr_sum);
        f.air_cyl_kg_s = sum_ms(m_cyl_air);
        f.exhaust_kg_s = sum_ms(m_out_sum);
        f.turbine_out_pa = p4;
        f.fuel_kg_s = sum_ms(v_fuel_sum) * FUEL_DENSITY_KG_M3 + sum_ms(m_mpi);
        f.turbine_out_k = if m_out_sum > 1e-12 { t4_e / m_out_sum } else { self.t_exhaust_k };
        f.unburned_fuel_w = sum_ms(fuel_out) * lhv;
        f.excess_air_kg_s = sum_ms(air_out);
        f.wall_heat_w = sum_ms(q_cool);
        f.oil_heat_w = sum_ms(q_oil) + tb_fric;
        f.blowby_kg_s = sum_ms(m_bb_sum);
        f.afterburn_w = sum_ms(ab_e);
        self.fric_filt_nm += (sum_ms(fric_e) - self.fric_filt_nm) * (dt / 0.05).min(1.0);
        f.torque_friction_nm = self.fric_filt_nm;
        f.torque_aux_nm = sum_ms(aux_e);
        f.torque_valvetrain_nm = sum_ms(cam_e);
        f.friction_power_w = sum_ms(fric_e) * self.omega_rad_s.abs() + self.lube.pump_power_w;
        let m_em: f64 = self.exh_m.iter().sum::<f64>().max(1e-12);
        let air_o = self.exh_m[AIR] + self.exh_m[PROD] * afr / (afr + 1.0);
        let fuel_o = self.exh_m[FUEL] + self.exh_m[PROD] / (afr + 1.0);
        let lam_mix = if fuel_o > 1e-6 * m_em { air_o / (afr * fuel_o) } else { 10.0 };
        self.lambda_exh = lam_mix.min(10.0);

        let mut gross = 0.0;
        let mut pump = 0.0;
        let mut air = 0.0;
        let mut n_air = 0.0;
        let max_age = if self.omega_rad_s.abs() > 1.0 {
            (4.5 * PI / self.omega_rad_s.abs()).clamp(0.15, 2.0)
        } else {
            0.0
        };
        for c in 0..4 {
            let l = self.cyl[c].last;
            let recent = max_age > 0.0 && env.t_s - l.t_end_s < max_age && l.t_end_s > 0.0;
            f.firing[c] = recent && l.burned;
            f.knock_integral[c] = l.lw;
            f.lambda_cyl[c] = l.lambda.min(10.0);
            f.fuel_per_cyl_mg[c] = if recent { l.fuel_kg * 1e6 } else { 0.0 };
            if recent {
                gross += l.gross_imep_pa;
                pump += l.pmep_pa;
                air += l.air_kg;
                n_air += 1.0;
            }
        }
        f.torque_indicated_nm = gross * kin.v_disp / (4.0 * PI);
        f.torque_pumping_nm = pump * kin.v_disp / (4.0 * PI);
        f.air_per_cyl_mg = if n_air > 0.0 { air / n_air * 1e6 } else { 0.0 };
        f.torque_starter_nm = st_t_e / dt;
        f.starter_current_a = st_i_e / dt;
        self.flows = f;
        self.check_failures(env.t_s);
    }

    fn step_turbo_mech(&mut self, p: &EngineProfile, t_c_out_k: f64, t: f64, dt: f64) {
        let tb = &p.turbo;
        let w = self.turbo_omega_rad_s;
        let k = (dt / 3.0).min(1.0);
        self.turbo.t_wheel_c[0] += (t_c_out_k - 273.15 - self.turbo.t_wheel_c[0]) * k;
        self.turbo.t_wheel_c[1] += (self.egt_k - 273.15 - 50.0 - self.turbo.t_wheel_c[1]) * k;
        let [tc, tt] = self.turbo.t_wheel_c;
        let sig_c = WHEEL_SHAPE_K * WHEEL_AA2618.density * (w * tb.compressor_tip_radius_m).powi(2);
        let sig_t = WHEEL_SHAPE_K * WHEEL_IN713C.density * (w * tb.turbine_tip_radius_m).powi(2);
        let m = &mut self.turbo;
        m.stress_ratio_c = sig_c / WHEEL_AA2618.uts_pa(tc).max(1.0);
        m.stress_ratio_t = sig_t / WHEEL_IN713C.uts_pa(tt).max(1.0);
        // Rain-flow-lite: each reversal of the compressor bore stress closes a half cycle.
        const HYST: f64 = 5e6;
        if m.rising {
            if sig_c > m.sig_ext {
                m.sig_ext = sig_c;
            } else if sig_c < m.sig_ext - HYST {
                m.lcf_damage += 0.5 * WHEEL_AA2618.cycle_damage(m.sig_ext, m.sig_other, tc);
                m.sig_other = m.sig_ext;
                m.sig_ext = sig_c;
                m.rising = false;
            }
        } else if sig_c < m.sig_ext {
            m.sig_ext = sig_c;
        } else if sig_c > m.sig_ext + HYST {
            m.sig_other = m.sig_ext;
            m.sig_ext = sig_c;
            m.rising = true;
        }
        // Creep acts at the hot blade roots, where the centrifugal stress is about half the bore hoop stress. UNVERIFIED.
        if sig_t > 1e6 {
            m.creep_damage += dt / 3600.0 / in713c_rupture_h(tt + 273.15, 0.5 * sig_t / 1e6);
        }
        if !m.burst && (m.stress_ratio_c >= 1.0 || m.stress_ratio_t >= 1.0 || m.lcf_damage >= 1.0 || m.creep_damage >= 1.0) {
            m.burst = true;
            let msg = format!(
                "turbocharger wheel burst at {:.0} rpm (compressor σ/UTS {:.2}, turbine σ/UTS {:.2}, LCF {:.2}, creep {:.2})",
                w * 30.0 / PI,
                m.stress_ratio_c,
                m.stress_ratio_t,
                m.lcf_damage,
                m.creep_damage
            );
            self.fail(t, 4, 1 << 0, msg);
        }
    }

    fn check_failures(&mut self, t: f64) {
        for c in 0..4 {
            let (land_failed, piston_damage, t_piston_k, gasket_leak_m2, rod_short_m, rod_broken, exv_creep, t_exv_k) = {
                let cy = &self.cyl[c];
                (cy.land_failed, cy.piston_damage, cy.t_piston_k, cy.gasket_leak_m2, cy.rod_short_m, cy.rod_broken, cy.exv_creep, cy.t_exv_k)
            };
            if land_failed {
                self.fail_with(t, c, 1 << 1, false, || {
                    format!("cyl {}: piston ring land fractured / crown failed (Miner D={:.2}, crown {:.0} °C)", c + 1, piston_damage, t_piston_k - 273.15)
                });
            }
            if gasket_leak_m2 > 0.0 {
                self.fail_with(t, c, 1 << 2, false, || {
                    format!("cyl {}: head gasket fire ring breached (leak {:.1} mm²)", c + 1, gasket_leak_m2 * 1e6)
                });
            }
            if rod_short_m > 0.05e-3 {
                self.fail_with(t, c, 1 << 3, false, || {
                    format!("cyl {}: connecting rod buckled ({:.2} mm)", c + 1, rod_short_m * 1e3)
                });
            }
            if rod_broken || rod_short_m > ROD_COLLAPSE_M {
                self.fail_with(t, c, 1 << 4, true, || {
                    format!("cyl {}: connecting rod fractured — engine destroyed", c + 1)
                });
            }
            if exv_creep >= 1.0 {
                self.vt.valve[c][1].broken = true;
                self.fail_with(t, c, 1 << 5, false, || {
                    format!("cyl {}: exhaust valve head crept off (T {:.0} °C)", c + 1, t_exv_k - 273.15)
                });
            }
            for k in 0..2 {
                let v = self.vt.valve[c][k];
                let name = if k == 0 { "intake" } else { "exhaust" };
                if v.bend_m > 0.02e-3 {
                    self.fail_with(t, c, 1 << (6 + k), false, || {
                        format!("cyl {}: {name} valve hit the piston and bent ({:.2} mm)", c + 1, v.bend_m * 1e3)
                    });
                }
                if v.broken {
                    self.fail_with(t, c, 1 << (8 + k), false, || {
                        format!("cyl {}: {name} valve head broke off", c + 1)
                    });
                }
            }
            if self.vt.valve[c].iter().any(|v| v.float_last_m > 0.1e-3) {
                let sep_mm = self.vt.valve[c].iter().map(|v| v.float_last_m).fold(0.0, f64::max) * 1e3;
                self.fail_with(t, c, 1 << 10, false, || {
                    format!("cyl {}: valve float ({sep_mm:.2} mm separation)", c + 1)
                });
            }
        }
        for b in 0..N_BEARINGS {
            if self.lube.bearings[b].seized {
                let br = self.lube.bearings[b];
                self.fail_with(t, 4, 1 << (1 + b), true, || {
                    let name = if b < 5 { format!("main bearing {}", b + 1) } else { format!("rod bearing cyl {}", b - 4) };
                    format!("{name} seized ({:.0} °C, wear {:.0} µm) — crank locked", br.t_k - 273.15, br.wear_m * 1e6)
                });
            }
        }
        if self.lube.turbo_seized {
            self.fail_with(t, 4, 1 << 11, false, || "turbocharger shaft bearing seized (oil starvation)".to_string());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn env() -> EngineEnv {
        EngineEnv { t_s: 0.0, p_amb_pa: 101_325.0, t_amb_k: 298.15, vehicle_speed_ms: 30.0, coolant_c: 90.0, oil_c: 95.0, battery_v: 14.2, supply_v_th: 14.2, supply_r_th: 0.007, fuel_ron: 95.0, alternator_torque_nm: 0.0, fuel_ethanol: 0.0 }
    }

    #[test]
    fn undriven_engine_stays_at_rest_and_is_unfuelled() {
        let p = EngineProfile::ea888_18t_cjsa_is12();
        let mut e = EngineState::new(&p, 101_325.0, 298.15);
        let a = EcuOutputs::default();
        for _ in 0..5000 {
            e.step(&p, &a, &env(), 0.0, 0.0, 2e-4);
        }
        assert_eq!(e.rpm(), 0.0);
        assert!(e.rail_pa < 2e5, "rail must stay at low pressure without the ECU");
        assert!((e.throttle_deg - p.throttle_limp_home_deg).abs() < 1e-6);
        assert!(e.cyl.iter().all(|c| c.m[FUEL] == 0.0));
    }

    /// Each engine's DI injector delivers the fuel mass its own calibration's characteristic `ip_ti_4_l` assigns to
    /// the electrical time (ΔP hPa, mass mg, ti ms), within 3 %: CJXC = 8V0906259J map @0x8084A214, CJSA = SC8O40
    /// 264H map @0x80849D92.
    #[test]
    fn injector_matches_the_calibrated_characteristic() {
        let cases = [
            (EngineProfile::ea888_20t_cjxc_is38(), [(50005.0f64, 20.0079, 2.052), (100005.0, 20.0079, 1.444), (150005.0, 49.9985, 2.916), (70001.0, 12.5049, 1.092)]),
            (EngineProfile::ea888_18t_cjsa_is12(), [(50005.0f64, 20.008, 2.732), (100005.0, 20.008, 1.904), (150005.0, 49.998, 3.896), (70001.0, 12.505, 1.444)]),
        ];
        for (p, pts) in cases {
            for (dp_hpa, m_mg, ti_ms) in pts {
                let q = p.injector_flow_cc_s * 1e-6 * (dp_hpa * 100.0 / 1e7).sqrt();
                let m = FUEL_DENSITY_KG_M3 * q * (ti_ms - p.injector_dead_time_ms) * 1e-3 * 1e6;
                assert!((m / m_mg - 1.0).abs() < 0.03, "{}: {dp_hpa} hPa {ti_ms} ms: {m:.2} mg vs {m_mg} mg", p.engine_code);
            }
        }
    }

    /// Manifold heat pick-up reproduces the reference warm-idle reading (16 kg/h, 12.7 °C ambient, 88 °C coolant → IAT
    /// 39.7 °C at warm idle) and fades at high flow (50 g/s, low-speed full load: < 1 K per 10 K).
    #[test]
    fn manifold_heat_pickup_matches_the_warm_idle_log() {
        let p = EngineProfile::ea888_18t_cjsa_is12();
        let iat = manifold_air_temp_k(&p, 16.0 / 3600.0, 273.15 + 12.7, 273.15 + 88.0) - 273.15;
        assert!((iat - 39.7).abs() < 2.0, "idle IAT {iat:.1} °C");
        let eps_wot = (manifold_air_temp_k(&p, 0.05, 300.0, 310.0) - 300.0) / 10.0;
        assert!(eps_wot > 0.02 && eps_wot < 0.1, "low-speed full-load pick-up ε {eps_wot:.3}");
    }

    fn hpfp_run(p: &EngineProfile, rpm: f64, rail_mpa: f64, cmd: impl Fn(&mut EcuOutputs), cycles: f64) -> f64 {
        let mut e = EngineState::new(p, 101_325.0, 298.15);
        e.set_speed(rpm * PI / 30.0);
        e.rail_pa = rail_mpa * 1e6;
        e.lp_fuel_pa = 0.6e6;
        let mut a = EcuOutputs { main_relay: Some(true), lpfp_duty: Some(1.0), ..Default::default() };
        cmd(&mut a);
        let mut env = env();
        env.battery_v = 14.0;
        let dt = 1e-4;
        for _ in 0..(cycles * 120.0 / rpm / dt) as usize {
            e.set_speed(rpm * PI / 30.0);
            e.step(p, &a, &env, 0.0, 0.0, dt);
            env.t_s += dt;
        }
        e.rail_pa / 1e6
    }

    /// Timed drives (co-simulation) act at their own times on the plant's crank-angle timeline, not at the angles of the
    /// previous cycle: N276 current delivers only once it is on, a DI pulse puts its q·(w − dead time) into the cylinder
    /// when it is driven, and the coil fires the cylinder at the end of its dwell.
    #[test]
    fn timed_drives_act_at_their_own_time() {
        let p = EngineProfile::ea888_18t_cjsa_is12();
        let rpm = 1200.0;
        let mut e = EngineState::new(&p, 101_325.0, 298.15);
        // pistons as warm as the 90 °C coolant of `env()` (a 25 °C crown keeps a share of the pulse as a wall film, so the
        // lean single pulse would not light: this test is about the drive timing)
        e.cyl.iter_mut().for_each(|c| c.set_component_temps(470.0, 850.0));
        e.set_speed(rpm * PI / 30.0);
        e.rail_pa = 10e6;
        e.lp_fuel_pa = 0.6e6;
        let d = |on_s: f64, off_s: f64| Some(Drive { on_s, off_s });
        let mut timed = TimedOutputs { hpfp: [d(0.02, 0.25), None, None], ..Default::default() };
        timed.di[0] = [d(0.155, 0.157), None, None];
        timed.coil[0] = [d(0.195, 0.1985), None, None];
        let a = EcuOutputs { main_relay: Some(true), lpfp_duty: Some(1.0), timed: Some(timed), ..Default::default() };
        let mut env = env();
        let dt: f64 = 1e-4;
        let fuel = |e: &EngineState| e.cyl[0].m[FUEL] + e.cyl[0].spray_kg + e.cyl[0].film_kg + e.cyl[0].liner_film_kg;
        let (mut rail_before, mut rail_inj, mut fuel_before, mut fuel_after, mut spark_deg) = (0.0, 0.0, 0.0, 0.0, f64::NAN);
        for _ in 0..(0.25 / dt).round() as usize {
            e.set_speed(rpm * PI / 30.0);
            let t0 = env.t_s;
            e.step(&p, &a, &env, 0.0, 0.0, dt);
            env.t_s += dt;
            if (t0..env.t_s).contains(&0.0195) {
                rail_before = e.rail_pa;
            }
            if (t0..env.t_s).contains(&0.1545) {
                fuel_before = fuel(&e);
                rail_inj = e.rail_pa;
            }
            if (t0..env.t_s).contains(&0.1575) {
                fuel_after = fuel(&e);
            }
            if (t0..env.t_s).contains(&0.1985) {
                spark_deg = crate::cylinder::atdc(e.crank_deg);
            }
        }
        assert!((rail_before - 10e6).abs() < 0.05e6, "no delivery before N276 is on: {:.2} MPa", rail_before / 1e6);
        assert!(e.rail_pa > 12e6, "delivery once it is on: {:.2} MPa", e.rail_pa / 1e6);
        let q = p.injector_flow_cc_s * 1e-6 * ((rail_inj - 1e5) / 1e7).sqrt();
        let want = FUEL_DENSITY_KG_M3 * q * (2e-3 - p.injector_dead_time_ms * 1e-3);
        assert!(fuel_before < 0.01 * want, "nothing before the pulse: {fuel_before:e}");
        assert!(((fuel_after - fuel_before) / want - 1.0).abs() < 0.1, "pulse mass {:.2} mg vs {:.2} mg", (fuel_after - fuel_before) * 1e6, want * 1e6);
        assert!((e.cyl[0].last.ign_deg - spark_deg).abs() < 1.0, "spark at the end of the dwell: {:.1} vs {spark_deg:.1} °ATDC", e.cyl[0].last.ign_deg);
    }

    /// N276 (normally open): the zero-delivery pulse the SC800S50 firmware sends at idle (VERIFIED-T, ≈ 1300 rpm:
    /// current on 22.5 °CRK before plunger TDC, off 6 °CRK later) must not deliver — the armature arrives after TDC,
    /// in the suction stroke, and returns before BDC — while the same current-on with the current held across BDC
    /// (start / maximum mode) delivers the full stroke, and a closing in the up-stroke delivers the part after it.
    #[test]
    fn hpfp_valve_closes_only_in_the_up_stroke() {
        let p = EngineProfile::ea888_20t_cjxc_is38();
        let lead = 22.5;
        let zero = hpfp_run(&p, 1300.0, 3.0, |a| (a.hpfp_close_deg, a.hpfp_off_deg) = (Some(180.0 - lead), Some(180.0 - lead + 6.1)), 20.0);
        assert!((zero - 3.0).abs() < 0.05, "min-delivery pulse must leave the rail alone: {zero:.2} MPa");
        let held = hpfp_run(&p, 1300.0, 3.0, |a| (a.hpfp_close_deg, a.hpfp_off_deg) = (Some(180.0 - lead), Some(180.0 - lead + 150.0)), 2.0);
        assert!(held > 15.0, "current held across BDC = full stroke: {held:.2} MPa");
        // Closing 30 ° before TDC (current on 30 ° + the delay earlier, short pulse): a partial stroke after the
        // self-locking pressure is passed.
        let d = crate::sensors::interp(14.0, p.hpfp_vcv_close_delay_ms) * 1e-3 * 1300.0 * 6.0;
        let part = hpfp_run(&p, 1300.0, 3.0, |a| (a.hpfp_close_deg, a.hpfp_off_deg) = (Some(150.0 - d), Some(150.0 - d + 6.1)), 2.0);
        assert!(part > 3.5 && part < held, "partial delivery: {part:.2} MPa");
    }

    /// Port injection: a pulse of width w delivers q·√(ΔP/ΔP_ref)·(w − dead time) (`ip_fac_mff_ti_stnd_h_rng_mpi`,
    /// `ip_ti_add_dly_mpi`, `ip_fac_ti_prs_mpi`); the film takes the fraction x and evaporates with τ; in steady state
    /// everything injected reaches the cylinders (x–τ: the film only delays it).
    #[test]
    fn port_injection_follows_the_calibrated_characteristic_and_the_x_tau_film() {
        let p = EngineProfile::ea888_18t_cjsa_is12();
        let mut port = PortFuel::default();
        let q = p.mpi_injector_mg_ms * 1e-3; // at ΔP_ref
        let dead = crate::sensors::interp(14.0, p.mpi_dead_time_ms) * 1e-3;
        let pulse = [Some(InjPulse { soi_btdc_deg: 400.0, width_ms: 3.58 }), None, None];
        let (h, w) = (2e-5, 800.0f64.to_radians() * 6.0);
        let mut phi = 0.0;
        let (mut inj, mut out) = (0.0, 0.0);
        let tau = p.port_film.tau_ref_s;
        let x = p.port_film.x;
        for k in 0..(3.0 / h) as usize {
            let phi1 = (phi + w.to_degrees() * h).rem_euclid(720.0);
            inj += port.step(&pulse, None, phi, phi1, w, k as f64 * h, q, dead, x, [1.0, 0.0, 0.0], tau.map(|t| [1.0 - (-h / t).exp(); 3]), h);
            phi = phi1;
            if k == 0 {
                continue;
            }
            // intake stroke draws the whole port charge
            if (360.0..540.0).contains(&phi) {
                out += port.vapour_kg;
                port.vapour_kg = 0.0;
            }
        }
        let per_pulse_mg = (3.58e-3 - dead) * q * 1e6;
        assert!((port.injected_last_kg * 1e6 / per_pulse_mg - 1.0).abs() < 0.01, "{} vs {per_pulse_mg}", port.injected_last_kg * 1e6);
        // steady films Σ x_i·m·τ_i/T_cycle (the slow film needs ≈ 3 τ = 3.6 s; 3 s run → within 30 %)
        let film_ss = (x[0] * tau[0] + x[1] * tau[1] * (1.0 - (-3.0 / tau[1]).exp())) * per_pulse_mg / 0.15;
        assert!((port.film_kg * 1e6 / film_ss - 1.0).abs() < 0.3, "film {:.2} mg vs x·m·τ/T {film_ss:.2} mg", port.film_kg * 1e6);
        assert!(((out + port.film_kg + port.vapour_kg) / inj - 1.0).abs() < 1e-9, "mass conserved");
    }

    /// Port films by fuel pseudo-component: at the firmware-anchored reference (coolant 90 °C, 35 kPa, 3 g/s) the
    /// mass-weighted τ of each film is its `tau_ref_s` (steady film mass x·m·τ unchanged), the light ends leave first and
    /// the heavy ones last, and a cold port holds the heavy ends far longer.
    #[test]
    fn port_film_components_keep_the_reference_film_and_split_by_volatility() {
        let f = crate::profiles::PORT_FILM_EA888_GEN3;
        let w = crate::combustion::fuel_components(0.0);
        let warm = f.tau_comp_s(f.t_ref_c, f.p_ref_kpa, f.flow_ref_kg_s);
        for (i, (&wm, &tau_ref)) in warm.iter().zip(&f.tau_ref_s).enumerate() {
            let mean: f64 = w.iter().zip(&wm).map(|(&wc, &tc)| wc * tc).sum();
            assert!((mean / tau_ref - 1.0).abs() < 1e-9, "film {i}: {mean} vs {tau_ref}");
            assert!(wm[0] < wm[1] && wm[1] < wm[2], "film {i}: {wm:?}");
        }
        let cold = f.tau_comp_s(20.0, f.p_ref_kpa, f.flow_ref_kg_s);
        assert!(cold[1][2] > 10.0 * warm[1][2], "port wall heavy ends at 20 °C: {:.1} s vs {:.1} s", cold[1][2], warm[1][2]);
    }

    /// Plug flow of the runner charge: an intake event that draws at least the runner's air takes all the port fuel in
    /// that event (the firmware's port model, FMSP 41.19), one that draws only part of it takes that share; the next
    /// event starts a new slug.
    #[test]
    fn port_charge_goes_in_with_the_runner_slug() {
        let mut port = PortFuel { vapour_kg: 10e-6, ..Default::default() };
        let draw = |port: &mut PortFuel, air_mg: f64| (0..air_mg as usize).map(|_| port.induct(1e-6, 90e-6)).sum::<f64>() * 1e6;
        assert!((draw(&mut port, 120.0) - 10.0).abs() < 1e-9, "the whole charge with a 120 mg event through a 90 mg runner");
        port.induct(0.0, 90e-6); // valve shut
        port.vapour_kg = 9e-6;
        assert!((draw(&mut port, 60.0) - 6.0).abs() < 1e-9, "two thirds of the slug drawn: two thirds of its fuel");
    }

    /// Timed injector drives (co-simulation): fuel flows from the electrical start + dead time to the end at the drive's
    /// own times, whatever the crank does; a pulse whose end is seen only later (∞ first, the real end in a later step)
    /// delivers exactly what the complete pulse does, and a drive still listed after its end is not injected twice.
    #[test]
    fn timed_injector_drive_flows_at_its_own_time() {
        let q = 2e-3; // kg/s
        let (dead, h) = (0.4e-3, 1e-4);
        let none = [None; 3];
        let run = |seen_end_at: f64| {
            let mut port = PortFuel::default();
            let mut first = None;
            let mut total = 0.0;
            for k in 0..100 {
                let t = k as f64 * h;
                let off = if t >= seen_end_at { 6.0e-3 } else { f64::INFINITY };
                let d = [Some(Drive { on_s: 2.05e-3, off_s: off }), None, None];
                let m = port.step(&none, Some(&d), 0.0, 0.0, 0.0, t, q, dead, [0.0; 2], [1.0, 0.0, 0.0], [[0.0; 3]; 2], h);
                if m > 0.0 && first.is_none() {
                    first = Some(t);
                }
                total += m;
            }
            (first, total)
        };
        let (first, known) = run(0.0);
        assert!((known / (q * (6.0e-3 - 2.05e-3 - dead)) - 1.0).abs() < 1e-9, "{known}");
        assert!(first.is_some_and(|t| (t - 2.4e-3).abs() < 1.5 * h), "starts at on + dead time: {first:?}");
        let (_, late) = run(4.0e-3);
        assert!((late / known - 1.0).abs() < 1e-9, "end seen late: {late} vs {known}");
    }

    /// Throttle actuator: full voltage opens the plate in well under 0.1 s, the springs return it to limp-home
    /// with the bridge off, a small duty holds part throttle, and a negative one drives it onto the lower stop.
    #[test]
    fn throttle_actuator_opens_returns_and_holds() {
        let p = EngineProfile::ea888_18t_cjsa_is12();
        let mut e = EngineState::new(&p, 101_325.0, 298.15);
        let lh = p.throttle_limp_home_deg;
        let run = |e: &mut EngineState, v: Option<f64>, s: f64| {
            for _ in 0..(s / 2e-4) as usize {
                e.step_throttle(&p.etc, lh, v, 2e-4);
            }
        };
        run(&mut e, Some(13.5), 0.08);
        assert_eq!(e.throttle_deg, p.etc.max_deg, "full open within 80 ms");
        run(&mut e, None, 0.3);
        assert_eq!(e.throttle_deg, lh, "return spring brings it back to limp-home");
        run(&mut e, Some(0.18 * 13.5), 0.5);
        assert!(e.throttle_deg > lh + 5.0 && e.throttle_deg < 60.0, "18 % holds part throttle: {}", e.throttle_deg);
        run(&mut e, Some(-0.5 * 13.5), 0.2);
        assert_eq!(e.throttle_deg, 0.0, "lower stop");
        run(&mut e, Some(0.0), 0.3);
        assert_eq!(e.throttle_deg, lh, "limp-home spring opens it again");
    }
}
