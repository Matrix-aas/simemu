//! Plant → synthesizer records. [`Tap::collect`] runs on the plant thread after every `VirtualPlant::step`:
//! it drains the engine's acoustic probe (impacts first, then crank frames, so every impact is queued before
//! the audio block that contains it is rendered) and adds one [`Slow`] record with the sub-step-rate
//! quantities of the rest of the car. It only reads plant state (and empties the probe vectors).

use simemu_physics::acoustic_probe::{CrankFrame, Impact};
use simemu_physics::{DrivetrainProfile, EngineProfile, PlantActuators, VirtualPlant, REVERSE};

/// Quantities sampled once per plant step (1 ms in the studio).
#[derive(Debug, Clone, Copy, Default)]
pub struct Slow {
    pub t_s: f64,
    pub p_amb: f32,
    pub t_amb: f32,
    pub rho_amb: f32,
    /// Turbine-outlet gas and catalyst temperatures, K; mean exhaust and compressor mass flow, kg/s.
    pub t4_k: f32,
    pub t_cat_k: f32,
    pub exhaust_kg_s: f32,
    pub air_kg_s: f32,
    /// Alternator output current, A, and rotor speed, rad/s.
    pub alt_a: f32,
    pub alt_rad_s: f32,
    /// Radiator fan PWM 0..1 (ECU output J293).
    pub fan_duty: f32,
    /// Clutch slip speeds, rad/s, and transmitted torques, Nm.
    pub clutch_slip: [f32; 2],
    pub clutch_nm: [f32; 2],
    /// Per input shaft: speed rad/s, pinion teeth of the engaged gear (0 = none), mesh torque Nm.
    pub shaft_rad_s: [f32; 2],
    pub mesh_teeth: [f32; 2],
    pub mesh_nm: [f32; 2],
    /// Engaged gears (bit gi) and, per gear, the sleeve speed of its fork, m/s.
    pub engaged: u8,
    pub sleeve_ms: [f32; 8],
    /// Σ dog health (decreases on dog clash) and Σ synchroniser friction work, J.
    pub dog_health: f32,
    pub synchro_j: f32,
    /// Mechatronic pump motor speed rad/s, current A; accumulator pressure Pa.
    pub pump_rad_s: f32,
    pub pump_a: f32,
    pub supply_pa: f32,
}

/// Constants of the installed hardware the synthesizer needs (from the plant profiles).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Config {
    pub piston_area_m2: f32,
    pub bore_m: f32,
    pub valve_mass_kg: [f32; 2],
    pub exhaust_volume_m3: f32,
    pub intake_volume_m3: f32,
    pub compressor_tip_m: f32,
}

impl Config {
    pub fn from_profile(p: &EngineProfile) -> Self {
        Self {
            piston_area_m2: (std::f64::consts::PI / 4.0 * p.bore_m * p.bore_m) as f32,
            bore_m: p.bore_m as f32,
            valve_mass_kg: [p.mech.intake.moving_mass_kg as f32, p.mech.exhaust.moving_mass_kg as f32],
            exhaust_volume_m3: p.exhaust_volume_m3 as f32,
            intake_volume_m3: p.intake_volume_m3 as f32,
            compressor_tip_m: p.turbo.compressor_tip_radius_m as f32,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub enum Record {
    #[default]
    None,
    Crank(CrankFrame),
    Impact(Impact),
    Slow(Slow),
    Config(Config),
}

#[derive(Default)]
pub struct Tap {
    cfg: Option<Config>,
}

fn gear_code(gi: usize) -> u8 {
    if gi == 7 { REVERSE } else { gi as u8 + 1 }
}

impl Tap {
    /// Turns the probe off and frees nothing (capacity is kept for the next activation).
    pub fn disable(plant: &mut VirtualPlant) {
        let pr = &mut plant.engine.probe;
        pr.enabled = false;
        pr.frames.clear();
        pr.impacts.clear();
    }

    /// Drains the probe into `push` and appends a [`Slow`] record. Returns the number of dropped records
    /// (`push` returned false: consumer behind).
    pub fn collect(&mut self, plant: &mut VirtualPlant, act: &PlantActuators, push: &mut impl FnMut(Record) -> bool) -> usize {
        let mut dropped = 0;
        let mut put = |r: Record| {
            if !push(r) {
                dropped += 1;
            }
        };
        let cfg = Config::from_profile(&plant.engine_profile);
        if self.cfg != Some(cfg) {
            self.cfg = Some(cfg);
            put(Record::Config(cfg));
        }
        let pr = &mut plant.engine.probe;
        for i in pr.impacts.drain(..) {
            put(Record::Impact(i));
        }
        for f in pr.frames.drain(..) {
            put(Record::Crank(f));
        }
        pr.enabled = true;
        put(Record::Slow(slow(plant, act)));
        dropped
    }
}

fn slow(p: &VirtualPlant, act: &PlantActuators) -> Slow {
    let amb = p.ambient_state();
    let d: &DrivetrainProfile = &p.drivetrain_profile;
    let dl = &p.driveline;
    let mut s = Slow {
        t_s: p.sim_time_s,
        p_amb: (amb.p_kpa * 1e3) as f32,
        t_amb: amb.t_k as f32,
        rho_amb: amb.rho as f32,
        t4_k: p.engine.flows.turbine_out_k.max(amb.t_k) as f32,
        t_cat_k: (p.thermal.cat_c + 273.15) as f32,
        exhaust_kg_s: p.engine.flows.exhaust_kg_s as f32,
        air_kg_s: p.engine.flows.air_compressor_kg_s as f32,
        alt_a: p.alternator_a as f32,
        alt_rad_s: (p.engine.omega_rad_s * simemu_physics::ALTERNATOR_RATIO) as f32,
        fan_duty: act.ecu.fan_duty.unwrap_or(0.0).clamp(0.0, 1.0) as f32,
        clutch_slip: dl.clutch_slip_rad_s.map(|x| x as f32),
        clutch_nm: dl.clutch_torque_nm.map(|x| x as f32),
        shaft_rad_s: dl.shaft_omega.map(|x| x as f32),
        dog_health: dl.dog_health.iter().sum::<f64>() as f32,
        synchro_j: dl.synchro_energy_j.iter().sum::<f64>() as f32,
        pump_rad_s: dl.mech.pump.omega as f32,
        pump_a: dl.mech.pump.current_a as f32,
        supply_pa: dl.mech.supply_pa as f32,
        ..Default::default()
    };
    for sh in 0..2 {
        if let Some(code) = dl.engaged_on_shaft(d, sh) {
            let gi = if code == REVERSE { 7 } else { (code as usize).saturating_sub(1).min(6) };
            s.mesh_teeth[sh] = d.train.gears[gi].pinion_teeth(d.gear_ratio(code)) as f32;
            s.mesh_nm[sh] = dl.gear_torque_nm[gi] as f32;
        }
    }
    for gi in 0..8 {
        if dl.gear_engaged[gi] {
            s.engaged |= 1 << gi;
        }
        if let Some((f, _, _)) = d.gear_fork(gear_code(gi)) {
            s.sleeve_ms[gi] = dl.mech.fork_v[f].abs() as f32;
        }
    }
    s
}
