//! Serializable snapshot of the plant for UIs and bridges. Physical quantities only.
//! Per-cylinder arrays are indexed by cylinder 1..4; bearing arrays: 0..4 = main bearings 1..5,
//! 5..8 = rod bearings of cylinders 1..4. "Last cycle" values refer to each cylinder's last
//! completed 720° cycle (boundary 180 °CA after its firing TDC).

use crate::driveline::TcuOutputs;
use crate::engine::EcuOutputs;
use serde::Serialize;

pub const PS_PER_KW: f64 = 1.359_621_6;

/// Per-cylinder combustion diagnostics of the last cycle and the hot-part / pre-ignition state (`cylinder`).
#[derive(Debug, Clone, Serialize, Default)]
pub struct CombustionTelemetry {
    /// Ignition angle (spark or pre-ignition), 10 % and 90 % burn angles, °ATDC.
    pub ign_deg: [f64; 4],
    pub ca10_deg: [f64; 4],
    pub ca90_deg: [f64; 4],
    /// Turbulence intensity and laminar flame speed at ignition, m/s; stratification index (unmixed vapour share).
    pub u_prime_ms: [f64; 4],
    pub s_l_ms: [f64; 4],
    pub stratification: [f64; 4],
    /// Peak end-gas and burned-zone temperatures, °C.
    pub t_end_gas_c: [f64; 4],
    pub t_burned_c: [f64; 4],
    /// Unburned share at knock onset (0 = no knock).
    pub knock_unburned: [f64; 4],
    /// Peak local wall pressure incl. knock oscillation, bar.
    pub p_local_bar: [f64; 4],
    /// Engine-out NO ppm, CO % vol, HC ppm C1, soot index mg/cycle.
    pub no_ppm: [f64; 4],
    pub co_pct: [f64; 4],
    pub hc_ppm: [f64; 4],
    pub soot_mg: [f64; 4],
    /// Piston top land and first ring groove, exhaust-valve seat, plug insulator tip / ground electrode, deposit
    /// surface, °C; plug carbon fouling 0..1.
    pub piston_land_c: [f64; 4],
    pub piston_groove_c: [f64; 4],
    pub exhaust_seat_c: [f64; 4],
    pub plug_tip_c: [f64; 4],
    pub plug_ground_c: [f64; 4],
    pub deposit_c: [f64; 4],
    pub plug_fouling: [f64; 4],
    /// Ring-land peak stress / UTS in the last cycle and the highest seen.
    pub land_stress_ratio: [f64; 4],
    pub land_stress_max: [f64; 4],
    /// Event counters since start: pre-ignitions (hot surface + LSPI), LSPI, knocking cycles, misfires.
    pub preign_events: [u32; 4],
    pub lspi_events: [u32; 4],
    pub knock_events: [u32; 4],
    pub misfire_events: [u32; 4],
    /// Pre-ignition in the last cycle and its angle, °ATDC.
    pub preignition: [bool; 4],
    pub preign_deg: [f64; 4],
    /// Fuel in the engine oil, % mass; exhaust-manifold afterburning, kW.
    pub oil_fuel_pct: f64,
    pub afterburn_kw: f64,
}

#[derive(Debug, Clone, Serialize, Default)]
pub struct PlantTelemetry {
    pub sim_time_s: f64,
    /// "stopped" | "cranking" | "spinning" | "running" | "seized".
    pub engine_state: &'static str,
    /// Crank speed, rpm.
    pub rpm: f64,
    /// Crank angle in the 720° cycle, deg (cylinder 1 firing TDC = 0).
    pub crank_deg: f64,
    /// Crank output torque = indicated − friction − pumping − accessories (what a dyno measures), Nm.
    pub torque_brake_nm: f64,
    /// Gross indicated torque (compression + expansion work of the last cycles), Nm.
    pub torque_indicated_nm: f64,
    /// Mean mechanical friction torque (bearings, piston group, rest), Nm.
    pub torque_friction_nm: f64,
    /// Gas-exchange (pumping) torque loss, Nm.
    pub torque_pumping_nm: f64,
    pub power_kw: f64,
    pub power_ps: f64,
    /// Engine-dyno absorber torque, Nm.
    pub dyno_absorber_nm: f64,

    pub baro_kpa: f64,
    pub ambient_c: f64,
    pub map_kpa: f64,
    pub put_kpa: f64,
    pub exhaust_manifold_kpa: f64,
    pub iat_c: f64,
    pub charge_air_c: f64,
    pub compressor_out_c: f64,
    pub throttle_pct: f64,
    pub wastegate_open_pct: f64,
    pub turbo_rpm: f64,
    /// Air mass flow through the throttle (MAF-equivalent), kg/h.
    pub maf_kg_h: f64,
    /// Mean trapped fresh air per cylinder and cycle, mg.
    pub air_mg_stroke: f64,
    /// Fuel injected per cylinder in its last cycle, mg.
    pub fuel_mg_stroke: [f64; 4],
    /// λ of the exhaust gas at the upstream (wide-band) sensor.
    pub lambda_sensor: f64,
    /// λ of the gas at the downstream (post-catalyst) sensor.
    pub lambda_down: f64,
    /// Ceramic tip temperatures of the upstream / downstream sensor, °C, and the upstream heater power, W.
    pub lsu_tip_c: f64,
    pub lsf_tip_c: f64,
    pub lsu_heater_w: f64,
    /// In-cylinder λ at spark, last cycle.
    pub lambda_cyl: [f64; 4],
    /// Cylinder burned in its last cycle (not misfired) and the cycle is recent.
    pub firing: [bool; 4],
    /// Livengood–Wu end-gas integral reached in the last cycle (≥ 1 = autoignition).
    pub knock_integral: [f64; 4],
    pub rail_mpa: f64,
    pub lp_fuel_bar: f64,
    /// Exhaust manifold gas temperature (turbine inlet), °C.
    pub egt_c: f64,
    pub turbine_out_c: f64,
    pub cat_c: f64,

    // ---- Crank-angle-resolved combustion diagnostics (last cycle) --------------------------
    /// Peak mean cylinder pressure, bar abs, and its crank angle, °ATDC.
    pub cyl_peak_bar: [f64; 4],
    pub cyl_peak_deg: [f64; 4],
    /// Net indicated mean effective pressure, bar.
    pub imep_bar: [f64; 4],
    /// 50 % burn angle, °ATDC.
    pub ca50_deg: [f64; 4],
    /// Peak first circumferential knock-mode amplitude, bar.
    pub knock_amp_bar: [f64; 4],
    /// Knock-sensor band RMS in each cylinder's window (5–70 °ATDC), mV.
    pub knock_sensor_mv: [f64; 4],
    /// Peak-to-peak crank speed over the last 720°, rpm (misfire / roughness signature).
    pub crank_speed_ripple_rpm: f64,
    /// Residual (burned) gas fraction at intake valve closing.
    pub residual_frac: [f64; 4],
    /// Intake cam advance / exhaust cam retard from the locked rest position, °CA.
    pub cam_phase_deg: [f64; 2],
    /// Quasi-dimensional combustion, hot parts, pre-ignition and emissions per cylinder.
    pub combustion: CombustionTelemetry,
    /// DMF secondary (gearbox side) speed, rpm.
    pub dmf_secondary_rpm: f64,

    // ---- Component temperatures and health -----------------------------------------------
    pub piston_crown_c: [f64; 4],
    pub exhaust_valve_c: [f64; 4],
    /// Ring-land fatigue (Miner) damage 0..1.
    pub piston_damage: [f64; 4],
    pub ring_land_failed: [bool; 4],
    /// Crankcase blow-by at ambient density, l/min.
    pub blowby_l_min: f64,
    /// Largest valve/follower separation in the last cycle (intake or exhaust), mm.
    pub valve_float_mm: [f64; 4],
    /// HLA pump-up (extension beyond zero lash), max of the cylinder's valves, mm.
    pub hla_pumpup_mm: [f64; 4],
    /// Permanent valve stem bend [intake, exhaust], mm.
    pub valve_bend_mm: [[f64; 2]; 4],
    /// Plastic connecting-rod shortening, mm, and rod fatigue damage 0..1.
    pub rod_bend_mm: [f64; 4],
    pub rod_damage: [f64; 4],
    /// Head-gasket bead/fire-ring damage 0..1 and breach area, mm².
    pub head_gasket_damage: [f64; 4],
    pub head_gasket_leak_mm2: [f64; 4],
    /// Liquid (coolant + fuel film) in the combustion chamber, ml.
    pub cylinder_liquid_ml: [f64; 4],
    /// Exhaust-valve creep life consumed 0..1.
    pub exhaust_valve_creep: [f64; 4],
    /// Minimum oil-film thickness in the last revolution pair, µm (mains 1..5, rods 1..4).
    pub bearing_min_film_um: [f64; 9],
    pub bearing_c: [f64; 9],
    pub bearing_wear_um: [f64; 9],
    pub bearing_oil_fill: [f64; 9],
    pub oil_level_l: f64,
    /// Air fraction drawn by the oil pickup 0..1.
    pub oil_aeration: f64,
    /// Turbo wheel bore stress / UTS at wheel temperature (compressor, turbine).
    pub turbo_stress_ratio: [f64; 2],
    pub turbo_bearing_c: f64,
    /// Turbo life consumed (max of compressor LCF and turbine creep) 0..1.
    pub turbo_damage: f64,
    /// Chronological list of physical failure events.
    pub failures: Vec<String>,

    pub coolant_c: f64,
    pub radiator_c: f64,
    pub radiator_valve_pct: f64,
    /// Rotary coolant valve angle from its lower stop, deg, and the 12-bit SENT value of its position sensor.
    pub rotary_valve_deg: f64,
    pub rotary_valve_sensor: u16,
    pub oil_c: f64,
    /// Main gallery oil pressure (gauge), bar.
    pub oil_bar: f64,
    /// Piston-jet gallery pressure (gauge, at the switch F447), bar; sump oil level at the sender G266, mm.
    pub oil_jet_bar: f64,
    pub oil_level_mm: f64,
    pub battery_v: f64,
    pub starter_current_a: f64,

    pub gearbox: &'static str,
    pub vehicle_kmh: f64,
    pub wheel_kmh_driven: f64,
    /// Wheel-speed sensors FL, FR, RL, RR (straight line: left = right).
    pub wheel_kmh: [f64; 4],
    pub accel_ms2: f64,
    pub pitch_deg: f64,
    pub axle_load_n: [f64; 2],
    pub tyre_force_n: [f64; 2],
    pub tyre_tread_c: [f64; 2],
    pub shaft_rpm: [f64; 2],
    /// Output shaft speeds (differential carrier speed × final drive of the shaft carrying 1st / top gear), rpm.
    pub output_shaft_rpm: [f64; 2],
    /// Gear dog-engaged on each input shaft (−1 = reverse, 0 = none).
    pub shaft_gear: [i8; 2],
    pub driving_gear: i8,
    /// Sleeve / fork travel sensors, mm.
    pub fork_mm: [f64; 4],
    /// Clutch actuator (DQ200: G617/G618) or piston travel, mm, and the physical kiss point.
    pub clutch_travel_mm: [f64; 2],
    pub clutch_kiss_mm: [f64; 2],
    pub clutch_bar: [f64; 2],
    pub clutch_capacity_nm: [f64; 2],
    pub clutch_torque_nm: [f64; 2],
    pub clutch_slip_rpm: [f64; 2],
    /// Plate bulk temperature and friction-interface (flash) temperature.
    pub clutch_temp_c: [f64; 2],
    pub clutch_surface_c: [f64; 2],
    pub clutch_wear_mm: [f64; 2],
    pub clutch_health: [f64; 2],
    /// Accumulator (DQ200, G270) or main line pressure (wet), bar.
    pub gearbox_hydraulic_bar: f64,
    pub sub_gearbox_bar: [f64; 2],
    /// Coil currents seen by the TCU's current shunts, A.
    pub solenoid_current_a: [f64; 12],
    pub solenoid_names: [&'static str; 12],
    pub pump_rpm: f64,
    pub pump_current_a: f64,
    pub pump_flow_lpm: f64,
    pub cooling_lpm: f64,
    /// Hydraulic fluid temperature (DQ200 G510 / wet ATF sump).
    pub mechatronic_oil_c: f64,
    pub gearbox_case_c: f64,
    pub driveshaft_torque_nm: f64,
    pub haldex_bar: f64,
    pub haldex_torque_nm: f64,
    pub brake_bar: f64,
    pub brake_disc_c: [f64; 2],
    pub roller_kmh: f64,
    pub roller_absorber_n: f64,
    pub driveline_failures: Vec<String>,

    /// Summary integrity 0..1 per cylinder (1 − worst of piston, rod, gasket, valve damage).
    pub cylinder_health: [f64; 4],
    /// 1 − worst bearing lining wear fraction (0 when any bearing seized).
    pub bearing_health: f64,
    pub turbo_health: f64,

    /// Actuator commands as received from the firmware boards (`None` = not driven).
    pub ecu_outputs: EcuOutputs,
    pub tcu_outputs: TcuOutputs,
    pub ecu_outputs_connected: bool,
    pub tcu_outputs_connected: bool,

    /// Electric wastegate actuator: position (0 = flap seated, 100 = open stop), position-sensor output, V, and
    /// armature current, A (+ = closing torque).
    pub wastegate_pos_pct: f64,
    pub wastegate_sensor_v: f64,
    pub wastegate_motor_a: f64,
    /// Port-flap position sensor output, V.
    pub port_flap_sensor_v: f64,
}
