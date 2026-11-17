//! EA888 Gen3 + DSG hardware plant.
//!
//! Pure hardware physics. The plant is driven exclusively by decoded firmware outputs
//! ([`EcuOutputs`], [`TcuOutputs`]) and by the driver / test-bench equipment ([`DriverInputs`],
//! [`BenchMode`]). It never decides throttle, spark, fuel, boost, gear or clutch pressure.
//!
//! Integration: fixed sub-step [`DT_SUB`] (0.2 ms) for the air path, clutches and tyres; inside each
//! sub-step the engine advances crank-angle resolved (<= 1 degCA / <= 50 us). The gearbox is coupled
//! to the DMF secondary. The thermal network is advanced once per call with energy-averaged heat flows.

pub mod acoustic_probe;
pub mod atmosphere;
pub mod bench_config;
pub mod clutch;
pub mod combustion;
pub mod compressor_map;
pub mod crank_train;
pub mod cooling_valve;
pub mod cylinder;
pub mod driveline;
pub mod driveline_failure;
pub mod engine;
pub mod hydraulics;
pub mod knock_sensor;
pub mod lambda_sensor;
pub mod lubrication;
pub mod materials;
pub mod mechatronic;
pub mod profiles;
pub mod sensors;
pub mod starter;
pub mod telemetry;
pub mod thermal;
pub mod valvetrain;
pub mod vehicle;
pub mod wastegate;

pub use atmosphere::{Ambient, AmbientState};
pub use driveline::{DriveEnv, DrivelineState, Selector, TcuOutputs, REVERSE};
pub use vehicle::{AwdOutputs, ChassisDynoHw};
pub use engine::{CoilCmd, Drive, EcuOutputs, EngineEnv, EngineState, InjPulse, TimedOutputs};
pub use sensors::{EdgeSource, SensorEdge};
pub use profiles::{DrivetrainPresetId, DrivetrainProfile, EnginePresetId, EngineProfile};
pub use telemetry::{PlantTelemetry, PS_PER_KW};
pub use thermal::{HeatInputs, ThermalState};

use serde::{Deserialize, Serialize};

pub const DT_SUB: f64 = 2e-4;
/// Effective charge-acceptance resistance of a near-full lead-acid battery (absorbs ~17 A at 14.2 V).
const BATTERY_R_CHARGE: f64 = 0.08;
pub const ALTERNATOR_RATIO: f64 = 2.7;
const ALTERNATOR_MAX_A: f64 = 140.0;
const ALTERNATOR_EFFICIENCY: f64 = 0.6;
/// Full-speed electrical input power of the single 300 W MQB radiator fan unit J293 (`c_state_ecf_pow_sub[0] = 2` in
/// `SC8O40` @ `0x808066A8` / `SC8S50` @ `0x808066F6` = `2H = 300W`, FR ENTE 34.21 p. 9062 & CUSF 11.56 p. 3347);
/// intermediate speeds and DC-link current scaling follow `duty³` around nominal `RADIATOR_FAN_NOM_V` (UNVERIFIED).
const RADIATOR_FAN_MAX_W: f64 = 300.0;
const RADIATOR_FAN_NOM_V: f64 = 13.5;
const RADIATOR_FAN_MIN_V: f64 = 6.0;
const DYNO_ROTOR_INERTIA: f64 = 0.30;
const DYNO_MAX_ABSORB_NM: f64 = 800.0;
/// Face velocity of the test cell's cooling fan over the vehicle's front-end heat exchangers (intercooler, radiator)
/// while the car does not move (engine / AC / roller dyno), m/s. A turbo engine on a test cell runs with its charge air
/// cooled as in the vehicle (SAE J1937: the cell's charge-air cooling reproduces the in-vehicle cooler); 25 m/s is the
/// ram-air speed at which `EngineProfile::intercooler_effectiveness` ("at speed") applies in full (`engine.rs`,
/// intercooler). The fan speed itself is UNVERIFIED (no cell data); 0 = no fan.
pub const TEST_CELL_FAN_MS: f64 = 25.0;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PlantActuators {
    pub ecu: EcuOutputs,
    pub tcu: TcuOutputs,
    /// AWD coupling controller (Haldex J492), a separate ECU.
    #[serde(default)]
    pub awd: AwdOutputs,
}

/// What the human (or the bench operator) does. Physical inputs only.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DriverInputs {
    pub pedal_pct: f64,
    pub brake_pct: f64,
    pub selector: Selector,
    /// Terminal 15 (ignition on).
    pub ignition_on: bool,
    /// Start button held (start request to the ECU; the ECU decides whether to crank).
    pub start_button: bool,
}

/// Test-bench equipment.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum BenchMode {
    /// Car on the road, gearbox connected.
    Road,
    /// Engine on a 4-quadrant AC dynamometer (motoring and absorbing) on the DMF secondary, gearbox removed: a speed
    /// loop of finite stiffness (PI on the secondary speed, rotor inertia, ±`DYNO_MAX_ABSORB_NM`), so combustion
    /// pulsations and misfires still show in the crank speed (what the firmware's roughness / misfire detection reads),
    /// unlike [`VirtualPlant::motoring_rpm`] which pins the crank speed.
    AcDyno { target_rpm: f64 },
    /// Engine on an eddy-current absorber with the gearbox removed. The absorber's own speed
    /// controller holds `target_rpm` (it can only brake, never motor).
    EngineDyno { target_rpm: f64 },
    /// Whole car strapped on a roller dyno; the absorber holds `target_kmh` (≤ 0 = inertia run).
    ChassisDyno { target_kmh: f64 },
    /// Whole car on a 4WD road-load chassis dyno ([`ChassisDynoHw::road_load`]): the rollers move as the car would
    /// on a level road (vehicle mass, aerodynamic drag and tyre rolling resistance of the build).
    RoadLoadDyno,
}

pub struct VirtualPlant {
    pub engine_profile: EngineProfile,
    pub drivetrain_profile: DrivetrainProfile,
    pub ambient: Ambient,
    pub fuel_ron: f64,
    /// Ethanol mass fraction of the fuel in the tank.
    pub fuel_ethanol: f64,
    pub engine: EngineState,
    pub driveline: DrivelineState,
    pub thermal: ThermalState,
    /// Rotary coolant valve of the thermal-management module (actuator N493 + SENT position sensor).
    pub rotary_valve: cooling_valve::RotaryValve,
    pub rotary_valve_hw: cooling_valve::RotaryValveHw,
    pub driver: DriverInputs,
    pub bench: BenchMode,
    pub sim_time_s: f64,
    pub battery_v: f64,
    pub battery: starter::Battery,
    /// Starter supply (Thevenin source V, Ω) for the next engine step.
    supply_th: (f64, f64),
    pub alternator_a: f64,
    /// Test-bench laboratory supply on the battery terminals (stiff voltage source), V. `None` = the
    /// vehicle battery.
    pub bench_supply_v: Option<f64>,
    /// Test-bench motoring drive coupled to the crank: holds this speed, rpm (`None` = not coupled).
    pub motoring_rpm: Option<f64>,
    /// Test-cell cooling fan, m/s ([`TEST_CELL_FAN_MS`]).
    pub test_cell_fan_ms: f64,
    pub dyno_absorber_nm: f64,
    dyno_integral: f64,
    last_actuators: PlantActuators,
    /// Exhaust oxygen sensors (transport, catalyst oxygen storage, heated cells).
    pub exhaust_sensors: lambda_sensor::ExhaustSensors,
}

impl VirtualPlant {
    pub fn new(engine: EngineProfile, drivetrain: DrivetrainProfile, ambient: Ambient) -> Self {
        let st = ambient.state();
        let driveline = DrivelineState::new(&drivetrain, ambient.temp_c);
        let e = EngineState::new(&engine, st.p_kpa * 1e3, st.t_k);
        Self {
            engine_profile: engine,
            drivetrain_profile: drivetrain,
            ambient,
            fuel_ron: 95.0,
            fuel_ethanol: 0.0,
            engine: e,
            driveline,
            thermal: ThermalState::uniform(ambient.temp_c),
            rotary_valve: cooling_valve::RotaryValve::at_rest(&cooling_valve::RotaryValveHw::default()),
            rotary_valve_hw: Default::default(),
            driver: DriverInputs::default(),
            bench: BenchMode::Road,
            sim_time_s: 0.0,
            battery_v: starter::Battery::default().ocv(ambient.temp_c),
            battery: starter::Battery::default(),
            supply_th: (starter::Battery::default().ocv(ambient.temp_c), starter::Battery::default().r(ambient.temp_c)),
            alternator_a: 0.0,
            bench_supply_v: None,
            motoring_rpm: None,
            test_cell_fan_ms: TEST_CELL_FAN_MS,
            dyno_absorber_nm: 0.0,
            dyno_integral: 0.0,
            last_actuators: PlantActuators::default(),
            exhaust_sensors: lambda_sensor::ExhaustSensors::new(ambient.temp_c),
        }
    }

    /// Cold-soaked car at ambient temperature, engine stopped. Bench settings, fuel and the
    /// driver's controls are kept.
    pub fn reset(&mut self) {
        let (ron, eth, bench, driver, supply, fan) = (self.fuel_ron, self.fuel_ethanol, self.bench, self.driver.clone(), self.bench_supply_v, self.test_cell_fan_ms);
        *self = Self::new(self.engine_profile.clone(), self.drivetrain_profile.clone(), self.ambient);
        self.bench_supply_v = supply;
        self.test_cell_fan_ms = fan;
        self.fuel_ron = ron;
        self.fuel_ethanol = eth;
        self.bench = bench;
        self.driver = driver;
    }

    /// Engine soaked at `t_c` after running and a short stop (the warm scenarios' and studio's warm-car state): coolant,
    /// oil, radiator, catalyst, bearing shells and the combustion-chamber parts (pistons, valves, plugs, deposits) at `t_c`.
    /// Setting only the coolant left the pistons at the plant's ambient — a cold crown under a hot engine, which keeps part
    /// of a hot start's fuel as a wall film (`cylinder` films).
    pub fn soak(&mut self, t_c: f64) {
        let t_k = t_c + 273.15;
        self.thermal = ThermalState::uniform(t_c);
        self.engine.lube.bearings.iter_mut().for_each(|b| b.t_k = t_k);
        self.engine.cyl.iter_mut().for_each(|c| c.set_component_temps_uniform(t_k));
    }

    /// Current the alternator can deliver at the present engine speed, A.
    fn alternator_i_max(&self) -> f64 {
        let n_alt = self.engine.rpm() * ALTERNATOR_RATIO;
        if n_alt > 1000.0 { ALTERNATOR_MAX_A * (1.0 - (-(n_alt - 1000.0) / 2500.0).exp()) } else { 0.0 }
    }

    /// The alternator as the energy manager reports it on CAN (BEM_05): mechanical power taken from the crank, W, and
    /// utilisation (`BEM_DFM`: current over what it could deliver at this speed), %.
    pub fn alternator_report(&self) -> (f64, f64) {
        let i_max = self.alternator_i_max();
        let util = if i_max > 1.0 { (100.0 * self.alternator_a / i_max).clamp(0.0, 100.0) } else { 0.0 };
        (self.battery_v * self.alternator_a / ALTERNATOR_EFFICIENCY, util)
    }

    /// Belt-driven alternator with internal field regulator + lead-acid battery (`starter::Battery`) or a
    /// bench supply. Returns the alternator shaft torque at the crank; leaves the Thevenin source the starter
    /// circuit sees for the next step in `supply_th`.
    fn step_electrical(&mut self, set_v: f64, starter_a: f64, main_relay_on: bool, fan_duty: f64, dt: f64) -> f64 {
        let i_max = self.alternator_i_max();
        let t_batt = self.ambient.temp_c;
        let (ocv, r) = (self.battery.ocv(t_batt), self.battery.r(t_batt));
        let base = if self.driver.ignition_on {
            25.0
        } else if main_relay_on {
            3.0
        } else {
            0.03
        };
        let fan_a = if fan_duty > 0.0 && self.battery_v.is_finite() && self.battery_v >= RADIATOR_FAN_MIN_V {
            let d = fan_duty.clamp(0.0, 1.0);
            (RADIATOR_FAN_MAX_W / RADIATOR_FAN_NOM_V) * (d * d * d) * (self.battery_v / RADIATOR_FAN_NOM_V).clamp(0.0, 1.15)
        } else {
            0.0
        };
        let other = base + fan_a;
        let load = other + starter_a;
        let wanted = load + ((set_v - ocv) / BATTERY_R_CHARGE).max(0.0);
        // Field-current lag of the regulator, bounded by instantaneous back-EMF capability `i_max`.
        self.alternator_a = (self.alternator_a + (wanted.min(i_max) - self.alternator_a) * (dt / 0.1).min(1.0)).min(i_max);
        let net = self.alternator_a - load;
        if let Some(v) = self.bench_supply_v {
            self.battery_v = v;
            self.supply_th = (v, 0.0);
        } else {
            self.battery_v = if net >= 0.0 { (ocv + BATTERY_R_CHARGE * net).min(set_v.max(ocv)) } else { ocv + r * net };
            self.battery.draw(-net, dt);
            self.supply_th = (ocv + r * (self.alternator_a - other).min(0.0), r);
        }
        let w = self.engine.omega_rad_s;
        if w > 1.0 { self.battery_v * self.alternator_a / (ALTERNATOR_EFFICIENCY * w) } else { 0.0 }
    }

    /// Air speed over the front-end heat exchangers, m/s: ram air of the moving car (road, road-load dyno), the
    /// test-cell fan while the car stands on a dyno.
    fn cooling_air_ms(&self) -> f64 {
        match self.bench {
            BenchMode::EngineDyno { .. } | BenchMode::AcDyno { .. } | BenchMode::ChassisDyno { .. } => self.driveline.airflow_ms.max(self.test_cell_fan_ms),
            BenchMode::Road | BenchMode::RoadLoadDyno => self.driveline.airflow_ms,
        }
    }

    pub fn ambient_state(&self) -> AmbientState {
        self.ambient.state()
    }

    /// Advances the plant by `dt` seconds.
    pub fn step(&mut self, dt: f64, act: &PlantActuators) {
        if !(dt > 0.0 && dt.is_finite()) {
            return;
        }
        let amb = self.ambient.state();
        let n = (dt / DT_SUB).round().clamp(1.0, 100_000.0) as usize;
        let h = dt / n as f64;
        let fan_kl30_ok = self.battery_v.is_finite() && self.battery_v >= RADIATOR_FAN_MIN_V;
        let eff_fan = act.ecu.fan_duty.filter(|d| d.is_finite() && fan_kl30_ok).map(|d| d.clamp(0.0, 1.0));
        let mut heat = HeatInputs { ambient_c: self.ambient.temp_c, air_density: amb.rho, coolant_valve_cmd: None, fan_duty: eff_fan, ..Default::default() };
        let w = 1.0 / n as f64;
        // The ECU/TCU are powered from terminal 30 and decide themselves what to drive (incl. after-run);
        // the plant never gates their outputs by ignition state.
        let ecu = &act.ecu;
        let tcu = &act.tcu;
        let ecu_powered = ecu.powered();
        let main_relay_on = ecu.main_relay == Some(true);
        let set_v = ecu.alternator_setpoint_v.unwrap_or(14.2).clamp(12.0, 15.5);
        let fan_duty = eff_fan.unwrap_or(0.0);

        self.engine.edges.clear();
        for i in 0..n {
            if let Some(rpm) = self.motoring_rpm {
                self.engine.set_speed(rpm * std::f64::consts::PI / 30.0);
            }
            let alt_torque = self.step_electrical(set_v, self.engine.flows.starter_current_a, main_relay_on, fan_duty, h);
            let env = EngineEnv {
                t_s: self.sim_time_s + i as f64 * h,
                p_amb_pa: amb.p_kpa * 1e3,
                t_amb_k: amb.t_k,
                vehicle_speed_ms: self.cooling_air_ms(),
                coolant_c: self.thermal.engine_c,
                oil_c: self.thermal.oil_c,
                battery_v: self.battery_v,
                supply_v_th: self.supply_th.0,
                supply_r_th: self.supply_th.1,
                fuel_ron: self.fuel_ron,
                alternator_torque_nm: alt_torque,
                fuel_ethanol: self.fuel_ethanol,
            };
            let env_e = env;
            match self.bench {
                BenchMode::EngineDyno { target_rpm } => {
                    // Absorber on the DMF secondary (gearbox removed).
                    let err = self.engine.omega2_rad_s - target_rpm * std::f64::consts::PI / 30.0;
                    self.dyno_integral = (self.dyno_integral + err * h).clamp(0.0, DYNO_MAX_ABSORB_NM / 40.0);
                    self.dyno_absorber_nm = (8.0 * err + 40.0 * self.dyno_integral).clamp(0.0, DYNO_MAX_ABSORB_NM);
                    self.engine.step(&self.engine_profile, ecu, &env, self.dyno_absorber_nm, DYNO_ROTOR_INERTIA, h);
                    self.driveline.step_uncoupled(&self.drivetrain_profile, tcu, self.engine.omega2_rad_s, self.driver.brake_pct, self.battery_v, self.ambient.temp_c, h);
                }
                BenchMode::AcDyno { target_rpm } => {
                    // positive = absorbing, negative = motoring
                    let err = self.engine.omega2_rad_s - target_rpm * std::f64::consts::PI / 30.0;
                    let lim = DYNO_MAX_ABSORB_NM / 40.0;
                    self.dyno_integral = (self.dyno_integral + err * h).clamp(-lim, lim);
                    self.dyno_absorber_nm = (8.0 * err + 40.0 * self.dyno_integral).clamp(-DYNO_MAX_ABSORB_NM, DYNO_MAX_ABSORB_NM);
                    self.engine.step(&self.engine_profile, ecu, &env, self.dyno_absorber_nm, DYNO_ROTOR_INERTIA, h);
                    self.driveline.step_uncoupled(&self.drivetrain_profile, tcu, self.engine.omega2_rad_s, self.driver.brake_pct, self.battery_v, self.ambient.temp_c, h);
                }
                BenchMode::Road | BenchMode::ChassisDyno { .. } | BenchMode::RoadLoadDyno => {
                    self.dyno_absorber_nm = 0.0;
                    self.dyno_integral = 0.0;
                    let d = &self.drivetrain_profile;
                    let env = DriveEnv {
                        battery_v: self.battery_v,
                        ambient_c: self.ambient.temp_c,
                        air_density: amb.rho,
                        coolant_c: self.thermal.engine_c,
                        selector: self.driver.selector,
                        brake_pct: self.driver.brake_pct,
                        awd_pump_duty: act.awd.pump_motor_duty,
                        rollers: match self.bench {
                            BenchMode::ChassisDyno { target_kmh } => Some((ChassisDynoHw::default(), target_kmh)),
                            BenchMode::RoadLoadDyno => Some((ChassisDynoHw::road_load(), 0.0)),
                            _ => None,
                        },
                    };
                    // The gearbox input (clutch drum / central plate) sits on the DMF secondary.
                    let e = &self.engine;
                    let j_drive = DrivelineState::drive_inertia(d);
                    let j_sec = self.engine_profile.mech.dmf_secondary_kgm2;
                    let t_sec = crank_train::dmf_torque(&self.engine_profile, e.dmf_twist_rad, e.omega_rad_s - e.omega2_rad_s);
                    let t_gbx = self.driveline.clutch_torques(d, tcu, &env, e.omega2_rad_s, t_sec, j_sec + j_drive, h);
                    self.engine.step(&self.engine_profile, ecu, &env_e, t_gbx, j_drive, h);
                    self.driveline.integrate(d, &env, self.engine.omega2_rad_s, h);
                }
            }
            if let Some(n) = self.motoring_rpm {
                self.engine.set_speed(n * std::f64::consts::PI / 30.0);
            }
            self.rotary_valve.step(
                &self.rotary_valve_hw,
                ecu.rotary_valve_duty.filter(|_| ecu_powered).map(|d| d.clamp(-1.0, 1.0) * self.battery_v),
                h,
            );
            let f = &self.engine.flows;
            heat.wall_heat_w += f.wall_heat_w * w;
            heat.oil_heat_w += f.oil_heat_w * w;
            heat.friction_w += f.friction_power_w * w;
            heat.exhaust_kg_s += f.exhaust_kg_s * w;
            heat.turbine_out_k += f.turbine_out_k * w;
            heat.unburned_fuel_w += f.unburned_fuel_w * w;
            heat.excess_air_kg_s += f.excess_air_kg_s * w;
            heat.coolant_pump_kg_s += f.coolant_pump_kg_s * w;
        }
        heat.vehicle_speed_ms = self.cooling_air_ms();
        heat.coolant_valve_cmd = Some(self.rotary_valve.radiator_fraction(&self.rotary_valve_hw));
        if heat.exhaust_kg_s < 1e-6 {
            heat.turbine_out_k = amb.t_k;
        }
        // Wide-band probe upstream of the turbine (SSP 606): manifold gas pressure and temperature; catalyst and the
        // downstream probe after it. UNVERIFIED: sensor boss at coolant temperature (lower bound).
        let gas = lambda_sensor::ExhaustGas {
            lambda: self.engine.lambda_exh,
            mass_flow_kg_s: heat.exhaust_kg_s,
            temp_k: self.engine.t_exhaust_k,
            pressure_pa: self.engine.p_exhaust_pa,
            temp_cat_k: heat.turbine_out_k,
        };
        let heater_duty = ecu.lambda_heater_duty.map(|d| d.filter(|_| ecu_powered));
        self.exhaust_sensors.step(&gas, heater_duty, self.battery_v, self.thermal.engine_c, dt);
        self.thermal.step(&heat, dt);
        self.sim_time_s += dt;
        self.last_actuators = act.clone();
    }

    pub fn telemetry(&self) -> PlantTelemetry {
        let e = &self.engine;
        let f = &e.flows;
        let amb = self.ambient.state();
        let rpm = e.rpm();
        // A crank at rest delivers no output torque (friction is then only a static reaction).
        let brake = if e.omega_rad_s > 0.0 {
            f.torque_indicated_nm - f.torque_friction_nm - f.torque_pumping_nm - f.torque_aux_nm
        } else {
            0.0
        };
        let power_kw = brake * e.omega_rad_s / 1000.0;
        let state = if e.locked {
            "seized"
        } else if f.firing.iter().any(|&x| x) && rpm > 300.0 {
            "running"
        } else if f.starter_current_a > 0.0 {
            "cranking"
        } else if rpm > 1.0 {
            "spinning"
        } else {
            "stopped"
        };
        let a = &self.last_actuators;
        let ecu_connected = a.ecu.any_driven();
        let tcu_connected = a.tcu.any_driven();
        let cyl = &e.cyl;
        let vt = &e.vt;
        let lu = &e.lube;
        let per = |g: &dyn Fn(usize) -> f64| -> [f64; 4] { std::array::from_fn(g) };
        let bear = |g: &dyn Fn(&lubrication::Bearing) -> f64| -> [f64; 9] { std::array::from_fn(|b| g(&lu.bearings[b])) };
        let cylinder_health = per(&|c| {
            let cy = &cyl[c];
            let v = &vt.valve[c];
            if cy.land_failed || cy.rod_broken || v.iter().any(|x| x.broken) {
                return 0.0;
            }
            let bend = v.iter().map(|x| x.bend_m).fold(0.0, f64::max) / 2.5e-3;
            1.0 - cy.piston_damage.max(cy.rod_damage).max(cy.gasket_damage).max(bend).max(cy.rod_short_m / 6e-3).clamp(0.0, 1.0)
        });
        let bearing_health = if lu.bearings.iter().any(|b| b.seized) { 0.0 } else { 1.0 - lu.bearings.iter().map(|b| b.wear_m / 0.25e-3).fold(0.0, f64::max).min(1.0) };
        let turbo_damage = e.turbo.lcf_damage.max(e.turbo.creep_damage).min(1.0);
        let mut t = PlantTelemetry {
            sim_time_s: self.sim_time_s,
            engine_state: state,
            rpm,
            crank_deg: e.crank_deg,
            torque_brake_nm: brake,
            torque_indicated_nm: f.torque_indicated_nm,
            torque_friction_nm: f.torque_friction_nm,
            torque_pumping_nm: f.torque_pumping_nm,
            power_kw,
            power_ps: power_kw * PS_PER_KW,
            dyno_absorber_nm: self.dyno_absorber_nm,
            baro_kpa: amb.p_kpa,
            ambient_c: self.ambient.temp_c,
            map_kpa: e.p_intake_pa / 1e3,
            put_kpa: e.p_boost_pa / 1e3,
            exhaust_manifold_kpa: e.p_exhaust_pa / 1e3,
            iat_c: e.t_intake_k - 273.15,
            charge_air_c: e.t_boost_k - 273.15,
            compressor_out_c: f.compressor_out_k - 273.15,
            throttle_pct: e.throttle_deg / 90.0 * 100.0,
            wastegate_open_pct: e.wastegate_open * 100.0,
            turbo_rpm: e.turbo_rpm(),
            maf_kg_h: f.air_throttle_kg_s * 3600.0,
            air_mg_stroke: f.air_per_cyl_mg,
            fuel_mg_stroke: f.fuel_per_cyl_mg,
            lambda_sensor: self.exhaust_sensors.lambda_up_gas,
            lambda_down: self.exhaust_sensors.lambda_down,
            lsu_tip_c: self.exhaust_sensors.up.t_c,
            lsf_tip_c: self.exhaust_sensors.down.t_c,
            lsu_heater_w: self.exhaust_sensors.up.heater_w,
            lambda_cyl: f.lambda_cyl,
            firing: f.firing,
            knock_integral: f.knock_integral,
            rail_mpa: e.rail_pa / 1e6,
            lp_fuel_bar: e.lp_fuel_pa / 1e5,
            egt_c: e.egt_k - 273.15,
            turbine_out_c: f.turbine_out_k - 273.15,
            cat_c: self.thermal.cat_c,
            cyl_peak_bar: per(&|c| cyl[c].last.p_max_pa / 1e5),
            cyl_peak_deg: per(&|c| cyl[c].last.p_max_deg),
            imep_bar: per(&|c| cyl[c].last.imep_pa / 1e5),
            ca50_deg: per(&|c| cyl[c].last.ca50_deg),
            knock_amp_bar: per(&|c| cyl[c].last.knock_amp_pa / 1e5),
            knock_sensor_mv: per(&|c| sensors::knock_sensor_band_mv(cyl[c].last.knock_rms_bar, rpm, c)),
            crank_speed_ripple_rpm: e.speed_ripple_rpm,
            residual_frac: per(&|c| cyl[c].last.residual),
            cam_phase_deg: [vt.phaser[0].angle_deg, vt.phaser[1].angle_deg],
            combustion: {
                let l = |c: usize| &cyl[c].last;
                let cel = |k: f64| k - 273.15;
                telemetry::CombustionTelemetry {
                    ign_deg: per(&|c| l(c).ign_deg),
                    ca10_deg: per(&|c| l(c).ca10_deg),
                    ca90_deg: per(&|c| l(c).ca90_deg),
                    u_prime_ms: per(&|c| l(c).u_prime),
                    s_l_ms: per(&|c| l(c).s_l),
                    stratification: per(&|c| l(c).stratification),
                    t_end_gas_c: per(&|c| cel(l(c).t_u_max)),
                    t_burned_c: per(&|c| cel(l(c).t_b_max)),
                    knock_unburned: per(&|c| l(c).knock_unburned),
                    p_local_bar: per(&|c| l(c).p_local_max_pa / 1e5),
                    no_ppm: per(&|c| l(c).no_ppm),
                    co_pct: per(&|c| l(c).co_pct),
                    hc_ppm: per(&|c| l(c).hc_ppm),
                    soot_mg: per(&|c| l(c).soot_mg),
                    piston_land_c: per(&|c| cel(cyl[c].t_land_k)),
                    piston_groove_c: per(&|c| cel(cyl[c].t_groove_k)),
                    exhaust_seat_c: per(&|c| cel(cyl[c].t_exv_seat_k)),
                    plug_tip_c: per(&|c| cel(cyl[c].t_plug_tip_k)),
                    plug_ground_c: per(&|c| cel(cyl[c].t_plug_gnd_k)),
                    deposit_c: per(&|c| cel(cyl[c].t_deposit_k)),
                    plug_fouling: per(&|c| cyl[c].plug_fouling),
                    land_stress_ratio: per(&|c| l(c).land_stress_ratio),
                    land_stress_max: per(&|c| cyl[c].land_stress_max),
                    preign_events: std::array::from_fn(|c| cyl[c].preign_events),
                    lspi_events: std::array::from_fn(|c| cyl[c].lspi_events),
                    knock_events: std::array::from_fn(|c| cyl[c].knock_events),
                    misfire_events: std::array::from_fn(|c| cyl[c].misfire_events),
                    preignition: std::array::from_fn(|c| l(c).preignition),
                    preign_deg: per(&|c| l(c).preign_deg),
                    oil_fuel_pct: 100.0 * e.oil_fuel_kg / (lu.oil_l.max(0.5) * 0.85),
                    afterburn_kw: f.afterburn_w / 1e3,
                }
            },
            dmf_secondary_rpm: e.omega2_rad_s * 30.0 / std::f64::consts::PI,
            piston_crown_c: per(&|c| cyl[c].t_piston_k - 273.15),
            exhaust_valve_c: per(&|c| cyl[c].t_exv_k - 273.15),
            piston_damage: per(&|c| cyl[c].piston_damage),
            ring_land_failed: std::array::from_fn(|c| cyl[c].land_failed),
            blowby_l_min: f.blowby_kg_s / amb.rho * 60_000.0,
            valve_float_mm: per(&|c| vt.valve[c].iter().map(|v| v.float_last_m).fold(0.0, f64::max) * 1e3),
            hla_pumpup_mm: per(&|c| vt.valve[c].iter().map(|v| v.hla_m).fold(0.0, f64::max) * 1e3),
            valve_bend_mm: std::array::from_fn(|c| [vt.valve[c][0].bend_m * 1e3, vt.valve[c][1].bend_m * 1e3]),
            rod_bend_mm: per(&|c| cyl[c].rod_short_m * 1e3),
            rod_damage: per(&|c| cyl[c].rod_damage),
            head_gasket_damage: per(&|c| cyl[c].gasket_damage),
            head_gasket_leak_mm2: per(&|c| cyl[c].gasket_leak_m2 * 1e6),
            cylinder_liquid_ml: per(&|c| cyl[c].liquid_volume() * 1e6),
            exhaust_valve_creep: per(&|c| cyl[c].exv_creep),
            bearing_min_film_um: bear(&|b| if b.film_last_m < 1.0 { b.film_last_m * 1e6 } else { 0.0 }),
            bearing_c: bear(&|b| b.t_k - 273.15),
            bearing_wear_um: bear(&|b| b.wear_m * 1e6),
            bearing_oil_fill: bear(&|b| b.fill),
            oil_level_l: lu.oil_l,
            oil_aeration: lu.aeration,
            turbo_stress_ratio: [e.turbo.stress_ratio_c, e.turbo.stress_ratio_t],
            turbo_bearing_c: lu.turbo_bearing_k - 273.15,
            turbo_damage,
            failures: e.failures.clone(),
            coolant_c: self.thermal.engine_c,
            radiator_c: self.thermal.radiator_c,
            radiator_valve_pct: self.thermal.radiator_valve_open * 100.0,
            rotary_valve_deg: self.rotary_valve.theta_deg,
            rotary_valve_sensor: self.rotary_valve.sensor_value(&self.rotary_valve_hw),
            oil_c: self.thermal.oil_c,
            oil_bar: e.oil_pressure_pa / 1e5,
            oil_jet_bar: lu.p_jets_pa / 1e5,
            oil_level_mm: lu.sump_level_mm,
            battery_v: self.battery_v,
            starter_current_a: f.starter_current_a,
            cylinder_health,
            bearing_health,
            turbo_health: if e.turbo.burst || lu.turbo_seized { 0.0 } else { 1.0 - turbo_damage },
            ecu_outputs: a.ecu.clone(),
            tcu_outputs: a.tcu.clone(),
            ecu_outputs_connected: ecu_connected,
            wastegate_pos_pct: e.wg.position(&self.engine_profile.turbo.wg_actuator) * 100.0,
            wastegate_sensor_v: e.wg.sensor_v(&self.engine_profile.turbo.wg_actuator),
            wastegate_motor_a: e.wg.current_a,
            port_flap_sensor_v: { let v = self.engine_profile.port_flap_sensor_v; v[0] + (v[1] - v[0]) * e.port_flap_closed },
            tcu_outputs_connected: tcu_connected,
            ..Default::default()
        };
        self.driveline.fill_telemetry(&self.drivetrain_profile, &mut t);
        t
    }
}

#[cfg(test)]
mod bench_tests;
#[cfg(test)]
mod driveline_tests;

#[cfg(test)]
mod fan_electrical_tests {
    use super::*;

    #[test]
    fn radiator_fan_loads_alternator_and_draws_battery_during_afterrun() {
        let mut p_off = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient::default());
        p_off.driver.ignition_on = true;
        p_off.motoring_rpm = Some(2000.0);
        let mut act = PlantActuators::default();
        act.ecu.main_relay = Some(true);
        act.ecu.fan_duty = Some(0.0);
        p_off.step(0.5, &act);
        let alt_idle = p_off.alternator_a;

        let mut p_full = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient::default());
        p_full.driver.ignition_on = true;
        p_full.motoring_rpm = Some(2000.0);
        act.ecu.fan_duty = Some(1.0);
        p_full.step(0.5, &act);
        assert!(p_full.alternator_a - alt_idle > 20.0, "300 W J293 fan at 14.2 V must add ~23 A: idle={alt_idle:.1} A, full={:.1} A", p_full.alternator_a);

        // Nachlauf (ignition off, engine stopped, main relay held, fan at 80 %): draws KL87 (3 A) + J293 from KL30 battery.
        let mut p_nl = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient::default());
        let soc0 = p_nl.battery.soc;
        act.ecu.fan_duty = Some(0.8);
        p_nl.step(10.0, &act);
        assert!(p_nl.battery.soc < soc0, "afterrun fan must discharge battery: {soc0} -> {}", p_nl.battery.soc);
        assert!(p_nl.battery_v < p_nl.battery.ocv(p_nl.ambient.temp_c), "battery terminal voltage must sag under fan load");

        // Unpowered KL30 (`bench_supply_v = 0 V`) and NaN fan duty must not draw current or poison state.
        let mut p_dead = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient::default());
        p_dead.bench_supply_v = Some(0.0);
        p_dead.battery_v = 0.0;
        p_dead.thermal.radiator_c = 100.0;
        act.ecu.fan_duty = Some(1.0);
        p_dead.step(1.0, &act);
        let rad_unpowered = p_dead.thermal.radiator_c;
        let mut p_ref = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient::default());
        p_ref.thermal.radiator_c = 100.0;
        let mut act_none = PlantActuators::default();
        act_none.ecu.fan_duty = Some(f64::NAN);
        p_ref.step(1.0, &act_none);
        assert!((rad_unpowered - p_ref.thermal.radiator_c).abs() < 1e-6, "0 V KL30 must not spin J293");
        assert!(p_ref.battery_v.is_finite() && p_ref.battery.soc.is_finite());
    }
}
