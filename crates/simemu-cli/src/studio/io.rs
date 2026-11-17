//! Pin-level glue between the physics plant and the emulated boards.
//!
//! Inputs: plant sensors → board peripherals. Outputs: board peripherals → plant actuators.
//! Nothing here may synthesise a command the firmware did not produce.
//!
//! Timing outputs (DI and MPI injectors, coils, N276) reach the plant as their edges ([`TimedOutputs`], delay
//! [`ACTUATOR_DELAY_S`]); the per-cycle angle summaries next to them are for display and logs.
//!
//! Engine ECU, decoded so far: ignition = GPTA0 GTC12..15 coil timing (`simemu_tc1791::engine_io`) gated
//! by the ATIC140 enables (`power_stage_outputs().lines`: NDIS1 high, LS_DIS0 low); DI injection = the
//! ATIC121 channel inputs G0.GTC04/17/16/05 (`engine_io::INJECTOR_GTC_BY_CYL`) gated by the ATIC121 enables;
//! the two port-driven main relays; the starter relays (ATIC140#1 bits 6/7); the fuel pump control signal
//! (BSW PWM channel 0xE); the HPFP valve N276 (G1.GTC26 current feed → `hpfp_close_deg` / `hpfp_off_deg`); the MPI
//! port injectors (G1.GTC00/03/02/01 → `port_injection`); the throttle motor (ATIC111 bridge 0: PWM G1.LTC27 +
//! direction P14.4 → `etc_duty`); the radiator cooling fan controller J293 (BSW PWM channel 0xF, G0.LTC15 →
//! `fan_duty` in `io_air`). Inputs: KL15, KL50, KL50R, brake light / test switch, P/N interlock, injector boost
//! voltage sense, analog sensors incl. the second pedal conversion, the flap potentiometer and the radiator-outlet
//! temperature (see [`feed_ecu`]).
//! The exhaust AVS actuators are ATIC140 stages 16..19 of both chips (`io_air`); G1.GTC08..15 are their timing and
//! diagnosis-window channels, not pins. Their switching diagnosis (pin recoil → P4.12 / P4.8) is fed by
//! [`feed_crank_cam`]. Not decoded (stay `None` = de-energised): the remaining solenoid PWM stages.
//!
//! Crank/cam: the sensor wheels are engine hardware, so the plant (`simemu_physics::crank_train`) is the
//! single source of the edges; this glue only adds the sensor electronics (CPDD pulse width) and the
//! harness (sensor → ECU input). `wheel_geometry_matches_board_wheel` pins the plant wheel to the
//! firmware's NC constants as read by `simemu_tc1791::crank_wheel`.

use simemu_physics::crank_train::FIRING_OFFSET_DEG;
use simemu_physics::{BenchMode, CoilCmd, Drive, DriverInputs, EcuOutputs, EdgeSource, InjPulse, PlantTelemetry, Selector, TcuOutputs, TimedOutputs, VirtualPlant};
use simemu_tc1791::atic142::{BinaryCell, LinearCell};
use simemu_tc1791::crank_wheel::WheelGeometry;
use simemu_tc1791::engine_io::{Pulse, SensorSignal, IGNITION_ACTIVE_LEVEL, IGNITION_GTC_BY_CYL, INJECTOR_ACTIVE_LEVEL, INJECTOR_GTC_BY_CYL};
use simemu_tc1791::gpta::CellKind;
use simemu_tc1791::{Dq200TcuBoard, Simos18Board};

/// KL15 input divider of the ECU (FR p.23942 NC_V_IGK_BRI_RATIO).
const KL15_DIVIDER: f64 = 5.701171875;
/// Brake light switch input divider. UNVERIFIED: the firmware only fixes the ADC threshold (3.5 V,
/// c_vp_thd_bls_act), a 12 V switch output must land above it.
const BLS_DIVIDER: f64 = 3.0;

/// Injector boost converter (ECU board, ATIC121 DC/DC) output sense: ADC2.CH1. VERIFIED-T: VBOOST
/// (RAM 0xD0019580, u8, 0.5 V/bit, diagnosis 0x801FD3D4 against c_vboost_mmv_diag_1/2/3 = 30/50/75 V) reads
/// 45.5 V at 2.5 V and 18.0 V at 1.0 V on this channel and nothing else moves it → divider 18.2 V/V.
const VBOOST_ADC: (u8, u8) = (2, 1);
const VBOOST_DIVIDER: f64 = 18.2;
/// After-main-relay supply VBR / KL87 sense (FR PWSL 64.4, p.23947: V_RLY_MAIN_MES_H_RES): ADC2.CH5. VERIFIED-S:
/// ACTION_INFR_GetVRlyMainBas 0x800A66A0 reads logical input 0x8007 = ADC2 buffer index 7 = CH5 (KL15 0x8008 = index 8
/// = CH4); the 10-sample average 0x801074A8 → 0xD001518A is the input of the main-relay ON plausibility 0x801DF674
/// (fails below c_v_rly_main_plaus_on_diag 5.0 V → RLY_MAIN_PLAUS_ON, which inhibits FID 0x3CD = lambda signal
/// acquisition). UNVERIFIED: divider NC_V_RLY_MAIN_BRI_RATIO taken equal to the KL15 one.
const KL87_ADC: (u8, u8) = (2, 5);
const KL87_DIVIDER: f64 = KL15_DIVIDER;
/// 5 V sensor reference supplies A/B/C (OBD P0642/P0652/P0698 = VCC_L_0/1/2, IDX_ERR 0x136..0x138) read back by the
/// MC: ACTION_INFR_GetVccBas 0x800A65F4 reads logical inputs 0xE / 0xD / 0xC = ADC0 buffer indices 14 / 13 / 12 =
/// ADC0.CH1 / CH2 / CH3 (VERIFIED-S), monitored against 4.57..5.48 V (c_vcc_pvs_min/max_mon). They were never fed:
/// VCC_L_0..2 failed as soon as the main-relay plausibility stopped masking them. UNVERIFIED: the supplies are on
/// whenever the ECU runs (their switch-off, e.g. by the ATIC139 PS outputs, is not modelled).
const VCC_ADC: [(u8, u8); 3] = [(0, 1), (0, 2), (0, 3)];
/// Second conversion of the pedal track G185 (PVS_2) for the A/D-converter plausibility CONV_PLAUS (ECOP 20.12.3, FR
/// p.5889; check 0x808C84B8: |ACQ_MC_2 − ACQ_MC_1| ≤ c_v_max_acq_conv_plaus 0.298 V): ACQ_MC_1 = getter id 0x800C =
/// ADC2.CH0 (0x808C5CAC), ACQ_MC_2 = id 7 = ADC0.CH8 = `pedal_pvs2` (0x8006E5E4) — VERIFIED-S/T (fed: error gone).
const PVS2_ADC2: (u8, u8) = (2, 0);
/// Radiator-outlet coolant temperature sensor G83 (LTS index 1, "TCR"): getter 0x800A5BE0 index 1 = id 0x4004 =
/// ADC1.CH6 (acquisition 0x808E558C, diagnosis LTS_EL_L/H_TCR 0x808E5A28) — VERIFIED-S/T. Its table `ip_tlts_mes[1]`
/// @0x808235A2 is within 1.6 °C of G62's, so the G62 characteristic is used (≤ 1.6 °C error).
const TCR_ADC: (u8, u8) = (1, 6);
const VCC_V: f64 = 2.5;
/// Regulated boost voltage. UNVERIFIED value (no ATIC121 data): Continental GDI drivers boost to ~65 V; the
/// firmware brackets it with its own "low" < 50 V and "high" > 75 V diagnosis thresholds.
const VBOOST_SET_V: f64 = 65.0;
/// Lowest input voltage at which the boost converter still regulates. UNVERIFIED.
const VBOOST_MIN_IN_V: f64 = 6.0;

/// Boost converter output: regulated while it is supplied through the main relay (KL87) with enough input
/// voltage; otherwise the output capacitor sits at the input voltage minus the boost diode (0.7 V) or
/// discharges with the supply off.
pub fn injector_boost_v(relay_on: bool, vb: f64) -> f64 {
    match relay_on {
        true if vb >= VBOOST_MIN_IN_V => VBOOST_SET_V,
        true => (vb - 0.7).max(0.0),
        false => 0.0,
    }
}

/// Feeds the engine ECU's inputs from the plant through the sensors' transfer functions and the ECU
/// input circuits. Analog channels: the board's analog map (firmware evidence in `sensors_io`); crank,
/// cam and KL50 go through the GPTA ([`feed_crank_cam`], [`feed_start_request`]).
/// Brake pedal: light switch F (12 V when pressed) on the analog BLS input and the brake test switch
/// (normally closed: 12 V while released) on P9.4 (getter 0x800A55D4: LV_PIN_BTS = !P9.4).
/// KL15: the ignition switch output (battery voltage while `ignition_on`).
/// KL50R: starter feedback P1.9 (getter 0x800A5AFC -> LV_ST_RESP, relay diagnosis FR p.8643).
/// P/N: the starter interlock input P0.3 (ACTION_INFR_GetPnCluSwi2 = 0x800A6880, LV_PN_CLU_SWI_2 = !P0.3,
/// FR p.25941: P/N signal of automatic transmissions, interlock switch of manual ones) is pulled to
/// ground in P and N. In the car the mechatronic drives it from its selector-shaft position; its output
/// pin is not decoded on the TCU board, so the lever position switches it here (UNVERIFIED path). On an
/// engine test cell ([`BenchMode::EngineDyno`] / [`BenchMode::AcDyno`], gearbox uncoupled) the test-stand
/// harness holds P0.3 grounded continuously so starter interlock ENSS releases KL50 and `LV_PN_CLU_SWI_2`
/// remains plausible with `Getriebe_11.GE_Fahrstufe = P` (`STATE_GLV_ETCU == PRKG`, FR TRSM 73.21.2.3
/// p.26092, avoiding `P0850` `PN_SCP_OC`; note that an MT-coded ECU with `LV_AT == 0` would instead open
/// P0.3 once running per FR p.26012).
pub fn feed_ecu(board: &mut Simos18Board, t: &PlantTelemetry, d: &DriverInputs, bench: BenchMode) {
    use simemu_physics::sensors as s;
    let (pvs1, pvs2) = s::pedal_sensors_v(d.pedal_pct);
    let (tps1, tps2) = s::throttle_sensors_v(t.throttle_pct * 0.9);
    let brake = d.brake_pct > 2.0;
    let signals: [(&str, f64); 13] = [
        ("pedal_pvs1", pvs1),
        ("pedal_pvs2", pvs2),
        ("map", s::map_sensor_v(t.map_kpa)),
        ("baro", s::baro_sensor_v(t.baro_kpa)),
        ("boost_pressure", s::put_sensor_v(t.put_kpa)),
        ("coolant_temp", s::coolant_sensor_v(t.coolant_c)),
        ("intake_air_temp", s::iat_sensor_v(t.iat_c)),
        ("throttle_tps1", tps1),
        ("throttle_tps2", tps2),
        ("rail_pressure", s::rail_sensor_v(t.rail_mpa)),
        ("fuel_low_pressure", s::lp_fuel_sensor_v(t.lp_fuel_bar)),
        ("kl15", if d.ignition_on { t.battery_v / KL15_DIVIDER } else { 0.0 }),
        ("brake_light_switch", if brake { t.battery_v / BLS_DIVIDER } else { 0.0 }),
    ];
    for (name, v) in signals {
        board.set_analog_signal(name, v as f32);
    }
    let relay_on = board.power_stage_outputs().relays.iter().all(|&r| r);
    let _ = board.set_adc_input_voltage(VBOOST_ADC.0, VBOOST_ADC.1, (injector_boost_v(relay_on, t.battery_v) / VBOOST_DIVIDER) as f32);
    let _ = board.set_adc_input_voltage(KL87_ADC.0, KL87_ADC.1, (if relay_on { t.battery_v } else { 0.0 } / KL87_DIVIDER) as f32);
    for (k, ch) in VCC_ADC {
        let _ = board.set_adc_input_voltage(k, ch, VCC_V as f32);
    }
    let _ = board.set_adc_input_voltage(PVS2_ADC2.0, PVS2_ADC2.1, pvs2 as f32);
    let _ = board.set_adc_input_voltage(TCR_ADC.0, TCR_ADC.1, s::coolant_sensor_v(t.radiator_c) as f32);
    board.set_port_input(9, 4, Some(!brake));
    let on_engine_dyno = matches!(bench, BenchMode::EngineDyno { .. } | BenchMode::AcDyno { .. });
    let pn_interlock = on_engine_dyno || matches!(d.selector, Selector::P | Selector::N);
    board.set_port_input(0, 3, Some(!pn_interlock));
    // KL50R: starter terminal 50 (energised through both starter relays) fed back on P1.9.
    board.set_port_input(1, 9, Some(t.starter_current_a > 0.0));
    crate::studio::io_air::feed(board, t);
}

/// Feeds the DQ200 mechatronic's sensors from the plant.
pub fn feed_tcu(tcu: &mut Dq200TcuBoard, t: &PlantTelemetry) {
    tcu.inject_full_mechatronic_plant(
        t.battery_v as f32,
        t.gearbox_hydraulic_bar as f32,
        t.mechatronic_oil_c as f32,
        t.fork_mm.map(|x| x as f32),
        t.clutch_travel_mm[0] as f32,
        t.clutch_travel_mm[1] as f32,
        t.rpm as f32,
        t.shaft_rpm[0] as f32,
        t.shaft_rpm[1] as f32,
    );
}

/// Powers the engine ECU up and runs its boot code for `insns` instructions. The sensors are wired and
/// supplied before the processor leaves reset, so every input line already carries the plant's level at
/// board time 0: applying the crank/cam levels only after the firmware has armed its GPTA input captures
/// (as the first plant exchange would) produces edges no sensor generated — the firmware then counted a
/// phantom cam edge, synchronised once and dropped the engine speed again (`N` → 0) within 0.5 s.
pub fn power_on_ecu(board: &mut Simos18Board, link: &mut EcuSensorLink, plant: &VirtualPlant, insns: usize) -> Result<u32, String> {
    // Hardware reset only releases when terminal 15 (KL15) wakes the board's power-supply ASIC, so the KL15 ADC
    // input is at battery voltage during the boot sequence even if the bench session still has `ignition_on = false`.
    let mut d = plant.driver.clone();
    d.ignition_on = true;
    feed_ecu(board, &plant.telemetry(), &d, plant.bench);
    feed_lambda_sensors(board, &plant.exhaust_sensors);
    feed_crank_cam(board, link, plant, 0.0);
    feed_start_request(board, link, plant);
    let res = board.run_boot_sequence(insns)?;
    link.t_off = Some(board.board_time_s() - plant.sim_time_s);
    Ok(res)
}

/// Time base link between the plant and the engine ECU board for the crank/cam signals.
#[derive(Debug, Clone, Default)]
pub struct EcuSensorLink {
    /// Board time − plant time (s); `None` until the first exchange (and after an ECU power cycle).
    pub t_off: Option<f64>,
    /// Level last applied to the KL50 start-request wire (`None` before the first exchange).
    pub kl50: Option<bool>,
    /// AVS pin recoils already passed to the ECU per cylinder and pin, and the pending end of the recoil pulse per
    /// diagnosis channel (board time).
    pub avs_recoils: [[u32; 2]; 4],
    pub vvl_low_at: [Option<f64>; 2],
    /// Oil level / temperature sender G266 (frame generator state; `io_air::feed_oil_sender`).
    pub g266: simemu_physics::sensors::OilLevelSender,
}

/// Length of the comparator pulse of one AVS pin recoil, s. UNVERIFIED (the firmware accepts a pulse that stays high for
/// `ip_t_min_vvl_diag` 1.5 ms after its 0.2 ms input filter; the induced-voltage amplitude against the comparator threshold
/// P10.2 is not modelled — every recoil passes).
const VVL_RECOIL_S: f64 = 3e-3;

/// Exhaust AVS switching diagnosis (docs/SIMOS18_COMBUSTION.md §6b): a pin pushed home by its groove's run-out induces a
/// voltage in its actuator coil; the ECU's comparator of diagnosis channel (logical cylinder index & 1) passes it to
/// P4.12 / P4.8 while the firmware enables that channel (P1.0 / P1.1 high, VERIFIED-S 0x800BB860).
fn feed_avs_recoil(board: &mut Simos18Board, link: &mut EcuSensorLink, plant: &VirtualPlant) {
    let until = link.board_time(plant.sim_time_s);
    let mut v = Vec::new();
    for (ch, low_at) in link.vvl_low_at.iter_mut().enumerate() {
        if let Some(t) = low_at.filter(|&t| t <= until) {
            v.push((t, SensorSignal::VvlRecoil(ch as u8, false)));
            *low_at = None;
        }
    }
    for (c, piece) in plant.engine.vt.avs.iter().enumerate() {
        for k in 0..2 {
            if piece.recoils[k] == link.avs_recoils[c][k] {
                continue;
            }
            link.avs_recoils[c][k] = piece.recoils[k];
            let ch = (FIRING_OFFSET_DEG[c] / 180.0).round() as usize & 1;
            if board.port_output(1, ch as u32) && link.vvl_low_at[ch].is_none() && piece.recoil_t_s[k] > plant.sim_time_s - 0.01 {
                let t = link.board_time(piece.recoil_t_s[k]);
                v.push((t, SensorSignal::VvlRecoil(ch as u8, true)));
                link.vvl_low_at[ch] = Some(t + VVL_RECOIL_S);
            }
        }
    }
    if !v.is_empty() {
        board.feed_sensor_edges(&v);
    }
}

/// Start request wire KL50 (ignition lock / start button → ECU pin P4.2, `engine_io::LINE_KL50`): the
/// driver's switch, applied at the board time of the current plant instant. With the switch open the
/// ECU input stage holds the pin low (the firmware reads a high pin as a start request).
pub fn feed_start_request(board: &mut Simos18Board, link: &mut EcuSensorLink, plant: &VirtualPlant) {
    let level = plant.driver.start_button;
    if link.kl50 != Some(level) {
        link.kl50 = Some(level);
        board.feed_sensor_edges(&[(link.board_time(plant.sim_time_s), SensorSignal::StartRequest(level))]);
    }
}

impl EcuSensorLink {
    pub fn board_time(&self, plant_t: f64) -> f64 {
        plant_t + self.t_off.unwrap_or(0.0)
    }
}

/// Crank/cam sensor edges of the last plant step → ECU inputs. Call right after `plant.step`; the board
/// then runs up to `link.board_time(plant.sim_time_s)`. The crank sensor (CPDD G28) pulls its output low
/// at each tooth's active edge for 45 µs forward / 100 µs backward (`WheelGeometry` pulse widths, DAMOS
/// window c_t_crk_sho_level_* 15–62.2 / 62.4–150 µs); the Hall cam sensors follow the plant's levels.
pub fn feed_crank_cam(board: &mut Simos18Board, link: &mut EcuSensorLink, plant: &VirtualPlant, step_s: f64) {
    let geo = WheelGeometry::simos18_sc8s50();
    let e = &plant.engine;
    if link.t_off.is_none() {
        // first exchange: plant at the start of the step that was just taken, sensor levels from its state
        let t0 = plant.sim_time_s - step_s;
        link.t_off = Some(board.board_time_s() - t0);
        let th0 = e.crank_deg;
        let shift = |k: usize| e.vt.cam_shift_deg(&plant.engine_profile, k);
        board.feed_sensor_edges(&[
            (board.board_time_s(), SensorSignal::Crank(true)),
            (board.board_time_s(), SensorSignal::CamIntake(geo.cam_level(&geo.cam_in_edges_deg, th0, shift(0)))),
            (board.board_time_s(), SensorSignal::CamExhaust(geo.cam_level(&geo.cam_ex_edges_deg, th0, shift(1)))),
        ]);
    }
    let mut v = Vec::with_capacity(2 * e.edges.len());
    for ed in &e.edges {
        let t = link.board_time(ed.t_s);
        match ed.source {
            EdgeSource::CrankTooth { forward, .. } => {
                v.push((t, SensorSignal::Crank(false)));
                v.push((t + if forward { geo.pulse_fwd_s } else { geo.pulse_back_s }, SensorSignal::Crank(true)));
            }
            EdgeSource::Cam { cam: 0, level } => v.push((t, SensorSignal::CamIntake(level))),
            EdgeSource::Cam { level, .. } => v.push((t, SensorSignal::CamExhaust(level))),
        }
    }
    board.feed_sensor_edges(&v);
    board.feed_crank_angle(link.board_time(plant.sim_time_s), unwrapped_crank_deg(board, e.crank_deg));
    feed_avs_recoil(board, link, plant);
    crate::studio::io_air::feed_oil_sender(board, link, plant);
}

/// Continues the board's unwrapped crank angle with the plant's 0..720° angle (the crank moves < 360°
/// per exchange step).
fn unwrapped_crank_deg(board: &Simos18Board, deg720: f64) -> f64 {
    let last = board.engine_io().latest_crank_deg();
    if last.is_nan() {
        return deg720;
    }
    let d = (deg720 - last.rem_euclid(720.0) + 360.0).rem_euclid(720.0) - 360.0;
    last + d
}

/// Oxygen-sensor cells → lambda sensor IC ATIC142 (SSC1/SLSO4, `simemu_tc1791::atic142`): the plant's cell state
/// at the chip pins, every exchange step.
pub fn feed_lambda_sensors(board: &mut Simos18Board, s: &simemu_physics::lambda_sensor::ExhaustSensors) {
    let (w, b) = (s.wideband(), s.binary());
    let c = &mut board.power_stages_mut().atic142;
    c.lin = LinearCell { ip_lim_ma: w.ip_lim_ma, e_exh_v: w.e_exh_v, ri_ohm: w.ri_ohm, rp_ohm: w.rp_ohm };
    c.bin = BinaryCell { e_v: b.e_v, ri_ohm: b.ri_ohm };
}

/// Oxygen-sensor heaters: BSW PWM channel 5 = upstream (wide-band) heater, channel 9 = downstream (binary) heater —
/// VERIFIED-S: the ASW duty functions 0x8019B258 / 0x8019B2CC compare the battery voltage with
/// c_vb_max_prot_lsh_lsl (0x808043A3) / c_vb_max_prot_lsh_ls (0x808043A2) and call the INFR setters 0x800A59E4
/// (channel 5) / 0x800A59D0 (channel 9); duty = (d + 1)/256 of the 8-bit V_efc²/V_b² value. Channel table
/// 0x80042400: channel 5 = cell 0x230 = LTCA2 LTC16 quad, channel 9 = cell 0x158 = G1.LTC56 quad; the output is
/// the fourth cell of the quad (as channel 0 → G1.LTC27, channel 0xE → G1.LTC11). Low = heater on (VERIFIED-T: the
/// firmware's duty byte 0xD000C56C = 5 during the dew-point phase, (5+1)/256 = 2.3 %, is the low fraction of
/// L2.LTC19). UNVERIFIED: the power stage behind the cells (assumed an ATIC140 low-side stage gated like the fuel
/// pump signal).
const LSH_PWM_CELLS: [(u8, u8); 2] = [(2, 19), (1, 59)];

/// Fuel pump control unit J538 PWM signal: BSW PWM channel 0xE (ACTION_INFR_SetPwmEfp = 0x800A6338,
/// SetFrqPwmEfp = 0x800A6320: 20/100 Hz, FR p.12143) = G1.LTC8 quad, duty cell G1.LTC11 -> GPTA1 OUT77
/// (OMCR) -> MSC0 ALTINH.5 (MMXCTR01 field 5 = GPTA1) -> ATIC140#0 stage 21 (low-side switch of the
/// signal line). VERIFIED-S/T: the channel table *(0xD0000744) entry 0xE holds cell 0x128.
/// UNVERIFIED: that J538 takes the on-time of the switch as its pump-speed demand.
const EFP_PWM_CELL: (u8, u8) = (1, 11);

/// Slowest PWM the firmware runs on an LTC quad: G0.LTC4 quad (N80 purge), CLK6, period 35156 = 10 Hz
/// (docs/SIMOS18_GPTA_IO_RE.md §6.4). A completed phase longer than this is a static level, not a PWM phase.
const MAX_PWM_PERIOD_NS: u64 = 100_000_000;

/// On-time (high) fraction of an LTC PWM output over its last completed period; a PWM output toggles at
/// least once per period, so an output quiet for more than 1.5 periods or with a completed phase longer
/// than [`MAX_PWM_PERIOD_NS`] is static (0 / 100 %).
pub(crate) fn ltc_duty(board: &Simos18Board, (kernel, cell): (u8, u8)) -> f64 {
    tap_duty(board.engine_io().tap(kernel, CellKind::Ltc, cell), (board.board_time_s() * 1e9) as u64, MAX_PWM_PERIOD_NS)
}

/// Alias for [`ltc_duty`] (all LTC PWM channels use [`MAX_PWM_PERIOD_NS`]).
pub(crate) fn ltc_duty_bounded(board: &Simos18Board, (kernel, cell): (u8, u8)) -> f64 {
    ltc_duty(board, (kernel, cell))
}

fn tap_duty(tap: &simemu_tc1791::engine_io::CellTap, now_ns: u64, max: u64) -> f64 {
    let last_ns = tap.last_rise_ns.max(tap.last_fall_ns);
    let period = tap.high_ns + tap.low_ns;
    if tap.transitions < 3 || period == 0 || tap.high_ns.max(tap.low_ns) > max || now_ns.saturating_sub(last_ns) > period + period / 2 {
        return if tap.level { 1.0 } else { 0.0 };
    }
    tap.high_ns as f64 / period as f64
}

/// Throttle motor H-bridge = ATIC111 bridge 0 (driver config 0x8004F46C: BSW PWM channel 0, direction pin id
/// 0x304 = P14.4, disable pin 0x67 = P6.7 MTC_DIS; duty setter 0x801F5854 called by THRO
/// ACTION_INFR_SetPwmDucyEtc, FR p.25114). PWM: channel 0 = G1.LTC24 quad (table 0x80042400, 1 kHz), output
/// cell G1.LTC27 → GPTA1 OUT25 (OMCR) = P4.1; high = on (VERIFIED-T: 0x00FB/0x4000 = 1.5 % commanded → 15 µs
/// high per 1 ms; +100 % → static high). Direction: P14.4 high = PWM_ETC ≥ 0 = opening torque (bridge cfg
/// byte 8 = 1: not inverted), switched by the PWM-period ISR 0x800AC0C4 one period after a sign change
/// (VERIFIED-S/T: P14 OUT bit 4 alternates with the ±100 % TPS start check).
const ETC_PWM_CELL: (u8, u8) = (1, 27);
const ETC_DIR_PIN: (usize, u32) = (14, 4);

/// HPFP quantity-control valve N276 (normally open): angle-output channel 9 = G1.GTC26 (channel table
/// 0x800418CC, 17 sub-events = the VCV peak/hold profile, FR p.11983 "Fuel pressure actuator control").
/// The cell idles high (0x5800, polarity byte 1) like the coil outputs, so the current feed is the low
/// phase; its start is the closing command. VERIFIED-T at 1000 rpm motored: current on 55 °CRK after
/// each pump TDC (k·180 °CRK, `c_ang_ref_vcv` = 0) = 35 ° before plunger BDC, off 19 ° before TDC — the
/// maximum-delivery start phase (LV_MFP_START_REQ) closes the valve before BDC.
const HPFP_VCV_CELL: (u8, u8) = (1, 26);
/// MPI (port) injectors of cylinders 1..4: G1.GTC00/03/02/01, active low (MILESTONE §5 п.8, separate agent: injection
/// driver reaches RUN, cells idle high like the DI timing inputs). The low-side stage behind them and its enable are
/// not identified: gated like the coils by the ATIC140 enables (NDIS1 high, LS_DIS0 low) — HYP.
const MPI_GTC_BY_CYL: [u8; 4] = [0, 3, 2, 1];

/// A spark older than this many crank degrees or seconds is no longer a command (engine stopped / the
/// firmware stopped firing).
const SPARK_HOLD_DEG: f64 = 1440.0;
const SPARK_HOLD_S: f64 = 1.0;

/// Fixed actuator delay of the co-simulation, s. In a lock-step window the plant steps first (its sensor edges must exist
/// before the ECU runs over them, so sensor timing is exact), then the ECU runs the same interval: an edge the ECU drives
/// in a window can reach the plant only in the next one. The injectors (DI, MPI) and N276 therefore reach the plant exactly
/// one window after the ECU drove them, every edge by the same delay (pulse widths and spacing, i.e. the fuel masses, kept):
/// 1 ms = 6 °CRK at 1000 rpm, 36 °CRK at 6000 rpm. The coils are the exception (see `coil_drives`): their discharge is
/// angle-critical, so the end of a dwell in progress is taken from the coil's previous dwell length — on time when the
/// firmware keeps it, ≤ one window late otherwise. Level and duty outputs (relays, H-bridges, PWM duties, AVS coils) are
/// sampled at the window end and held over the next window (zero-order hold, ≤ the same delay).
pub const ACTUATOR_DELAY_S: f64 = crate::studio::session::EXCHANGE_S;

/// Active phases (`active` level) of a GTC timing output as plant-time drives: the board's edge times shifted by
/// [`ACTUATOR_DELAY_S`], the complete phases and the one in progress (end ∞ until it is seen), ≤ 3, oldest first.
fn gtc_drives(board: &Simos18Board, link: &EcuSensorLink, (kernel, cell): (u8, u8), active: bool, now: f64) -> [Option<Drive>; 3] {
    let to_plant = |t_ns: u64| t_ns as f64 / 1e9 - link.t_off.unwrap_or(0.0) + ACTUATOR_DELAY_S;
    drives_of(board.engine_io().gtc_events(kernel, cell).iter().map(|e| (e.t_ns, e.level)), active, to_plant, now)
}

/// Phases of `active` level from an output's transitions (board ns, level), oldest first: a phase runs from the first event
/// that drives the pin to `active` to the first that drives it back — an event repeating the level the pin already has is
/// no edge, and the first event seen only gives the level (history start, the cell's idle level at init). Open = end ∞.
fn phases(events: impl Iterator<Item = (u64, bool)>, active: bool, to_plant: impl Fn(u64) -> f64) -> Vec<Drive> {
    let mut v: Vec<Drive> = Vec::new();
    let mut level: Option<bool> = None;
    for (t_ns, l) in events {
        if level.is_some_and(|x| x != l) {
            if l == active {
                v.push(Drive { on_s: to_plant(t_ns), off_s: f64::INFINITY });
            } else if let Some(d) = v.last_mut() {
                d.off_s = to_plant(t_ns);
            }
        }
        level = Some(l);
    }
    v
}

/// The last 3 phases that end after `now − ACTUATOR_DELAY_S` (a phase that ended within the last window may still be ahead
/// of the plant; one the plant has already passed is ignored by it).
fn recent(v: &[Drive], now: f64) -> [Option<Drive>; 3] {
    let mut out = [None; 3];
    let min_off = now - ACTUATOR_DELAY_S;
    let mut n = 0usize;
    for d in v.iter().rev().filter(|d| d.off_s > min_off).take(3) {
        out[n] = Some(*d);
        n += 1;
    }
    out[..n].reverse();
    out
}

/// [`phases`] → [`recent`].
fn drives_of(events: impl Iterator<Item = (u64, bool)>, active: bool, to_plant: impl Fn(u64) -> f64, now: f64) -> [Option<Drive>; 3] {
    recent(&phases(events, active, to_plant), now)
}

/// Coil dwell phases of a cylinder as plant-time drives, without the window delay: the discharge (spark) is the end of the
/// dwell and the plant fires at that time on its own crank. A complete dwell is late by ≤ one window when it is seen; a
/// dwell in progress ends after the coil's previous dwell length (the firmware's dwell time follows the supply voltage, so
/// consecutive dwells of a coil are equal at steady supply) unless that time has already passed — then the real end is
/// awaited. The plant fires each dwell once ([`simemu_physics::TimedOutputs::coil`]).
fn coil_drives(board: &Simos18Board, link: &EcuSensorLink, cell: u8, now: f64) -> [Option<Drive>; 3] {
    let to_plant = |t_ns: u64| t_ns as f64 / 1e9 - link.t_off.unwrap_or(0.0);
    let mut v = phases(board.engine_io().gtc_events(0, cell).iter().map(|e| (e.t_ns, e.level)), IGNITION_ACTIVE_LEVEL, to_plant);
    predict_dwell_end(&mut v, now);
    recent(&v, now)
}

/// A dwell in progress (last phase open) ends after the previous dwell's length, unless that is already past.
fn predict_dwell_end(v: &mut [Drive], now: f64) {
    if let [.., prev, last] = v {
        let w = prev.off_s - prev.on_s;
        if last.off_s.is_infinite() && w.is_finite() && last.on_s + w > now {
            last.off_s = last.on_s + w;
        }
    }
}

/// Timing outputs as drives for the plant (event-accurate, [`ACTUATOR_DELAY_S`]), gated by the power stages as the
/// per-cycle summary of [`decode_ecu_outputs`].
fn decode_timed(board: &Simos18Board, link: &EcuSensorLink, injectors_enabled: bool, stages_enabled: bool) -> TimedOutputs {
    let now = board.board_time_s() - link.t_off.unwrap_or(0.0);
    let mut t = TimedOutputs::default();
    for c in 0..4 {
        if injectors_enabled {
            t.di[c] = gtc_drives(board, link, (0, INJECTOR_GTC_BY_CYL[c]), INJECTOR_ACTIVE_LEVEL, now);
        }
        if stages_enabled {
            t.mpi[c] = gtc_drives(board, link, (1, MPI_GTC_BY_CYL[c]), false, now);
            t.coil[c] = coil_drives(board, link, IGNITION_GTC_BY_CYL[c], now);
        }
    }
    t.hpfp = gtc_drives(board, link, HPFP_VCV_CELL, false, now);
    t
}

/// Per-cycle summary of one injector for display and logs (the plant runs [`TimedOutputs`]): the pulses (≤ 3) of the
/// cylinder's latest complete engine cycle — the injector requests that precede the most recent firing TDC the crank has
/// passed — as start angle before that TDC and electrical width. A cycle without pulses (fuel cut-off, cylinder cut-off, a
/// DI → MPI switch) is reported empty, not as a repeat of the previous one.
fn injection_pulses(pulses: &[Pulse], firing_deg: f64, now_s: f64, now_deg: f64) -> [Option<InjPulse>; 3] {
    let mut out = [None; 3];
    // firing TDC (unwrapped) that ends the cycle a pulse belongs to
    let tdc = |p: &Pulse| firing_deg + 720.0 * ((p.start_deg - firing_deg) / 720.0).ceil();
    let complete = |p: &&Pulse| p.start_deg.is_finite() && tdc(p) <= now_deg;
    let Some(last) = pulses.iter().rev().find(complete).filter(|p| now_s - p.end_s < SPARK_HOLD_S && now_deg - tdc(p) < 720.0) else { return out };
    let last_tdc = tdc(last);
    let mut n = 0usize;
    for p in pulses.iter().rev().filter(complete).filter(|p| tdc(p) == last_tdc).take(3) {
        out[n] = Some(InjPulse { soi_btdc_deg: last_tdc - p.start_deg, width_ms: p.width_ms() });
        n += 1;
    }
    out[..n].reverse();
    out
}

/// Decodes engine-ECU hardware outputs.
/// * ignition: last dwell of each cylinder's coil output (GPTA0 GTC15/12/14/13, active low) → dwell and
///   spark angle before the cylinder's firing TDC, while the ATIC140 ignition/low-side stages are enabled
///   (NDIS1 high from the monitoring unit, LS_DIS0 low; GTC12/13 reach ATIC140#1 stages 23/22 through
///   GPTA0 OUT111/110 → MSC1 ALTINH.7/.6, `engine_io::EngineIo::msc_altin`);
/// * relays: the two port-driven relays (`power_stages::RELAY_PINS`) supply the engine loads, the plant's
///   `main_relay` is on while both are energised (which one is J271 / J757 is UNVERIFIED); the starter
///   relays J906/J907 are ATIC140#1 stages 6 and 7 in series (ENSS starter relay control, gated by EN7_8);
/// * everything else behind the power stages (pumps, valves, H-bridges, injector driver enable)
///   stays `None` until the power-stage driver models provide the stage states (see module docs).
pub fn decode_ecu_outputs(board: &Simos18Board, link: &EcuSensorLink) -> EcuOutputs {
    let eio = board.engine_io();
    let t_s = board.board_time_s();
    let crank_deg = eio.latest_crank_deg();
    let ps = board.power_stage_outputs();
    let lines = ps.lines;
    let stages_enabled = lines.ndis1 && !lines.ls_dis0;
    let dir = if board.port_output(ETC_DIR_PIN.0, ETC_DIR_PIN.1) { 1.0 } else { -1.0 };
    let mut o = EcuOutputs {
        main_relay: Some(ps.relays.iter().all(|&r| r)),
        starter_relay: Some(ps.atic140[1][6].on && ps.atic140[1][7].on),
        lpfp_duty: Some(if stages_enabled { ltc_duty(board, EFP_PWM_CELL) } else { 0.0 }),
        // ATIC111 output stage enabled by EN (MU NDIS1) and not disabled by DIS (MTC_DIS).
        etc_duty: ps.bridges[0].enabled.then(|| dir * ltc_duty(board, ETC_PWM_CELL)),
        lambda_heater_duty: LSH_PWM_CELLS.map(|c| stages_enabled.then(|| 1.0 - ltc_duty(board, c))),
        ..Default::default()
    };
    // N276 current feed (latest low phase of the VCV cell: on and off angle), while it is still being repeated.
    if let Some(p) = eio
        .last_gtc_pulse(HPFP_VCV_CELL.0, HPFP_VCV_CELL.1, false)
        .filter(|p| t_s - p.end_s < SPARK_HOLD_S && crank_deg - p.end_deg < SPARK_HOLD_DEG)
    {
        let on = p.start_deg.rem_euclid(720.0);
        o.hpfp_close_deg = Some(on);
        o.hpfp_off_deg = Some(on + (p.end_deg - p.start_deg));
    }
    for (c, &cell) in MPI_GTC_BY_CYL.iter().enumerate().filter(|_| stages_enabled) {
        o.port_injection[c] = injection_pulses(&eio.gtc_pulses(1, cell, false), FIRING_OFFSET_DEG[c], t_s, crank_deg);
    }
    // DI injectors: the ATIC121 switches a channel while its timing input is active and its NDIS (MU NDIS0)
    // / DIS (LS_DIS0) inputs enable the stages.
    for (c, &cell) in INJECTOR_GTC_BY_CYL.iter().enumerate().filter(|_| ps.injectors_enabled) {
        o.injection[c] = injection_pulses(&eio.gtc_pulses(0, cell, INJECTOR_ACTIVE_LEVEL), FIRING_OFFSET_DEG[c], t_s, crank_deg);
    }
    for (c, &cell) in IGNITION_GTC_BY_CYL.iter().enumerate().filter(|_| lines.ndis1 && !lines.ls_dis0) {
        let Some(p) = eio.last_gtc_pulse(0, cell, IGNITION_ACTIVE_LEVEL) else { continue };
        let d_deg = crank_deg - p.end_deg;
        if t_s - p.end_s > SPARK_HOLD_S || !d_deg.is_finite() || d_deg >= SPARK_HOLD_DEG {
            continue;
        }
        let btdc = (FIRING_OFFSET_DEG[c] - p.end_deg).rem_euclid(720.0);
        o.ignition[c] = Some(CoilCmd { dwell_ms: p.width_ms(), spark_btdc_deg: if btdc > 360.0 { btdc - 720.0 } else { btdc } });
    }
    o.timed = Some(decode_timed(board, link, ps.injectors_enabled, stages_enabled));
    crate::studio::io_air::decode(board, &ps, &mut o);
    o
}

/// Decodes TCU hardware outputs: duties measured on the GPTA PWM output cells of the DQ200
/// (`Dq200TcuBoard::decode_outputs`).
pub fn decode_tcu_outputs(tcu: &Dq200TcuBoard) -> TcuOutputs {
    let o = tcu.decode_outputs();
    TcuOutputs {
        solenoid_duty: o.solenoid_duty,
        pump_motor_duty: o.pump_motor_duty,
        pump_shaft_rad_s: o.pump_shaft_rad_s,
        solenoid_current_a: Some(o.solenoid_current_a),
    }
}

#[cfg(test)]
mod tests {
    use simemu_physics::crank_train::{CAM_EDGES_DEG, CRANK_TOOTH0_DEG};
    use simemu_tc1791::crank_wheel::WheelGeometry;

    /// The RVC trace of §12.9.1: 1 kHz PWM, 12.58 s static low (stage off), then static high = 100 %.
    #[test]
    fn ltc_duty_reads_static_high_after_a_long_pause_as_full_duty() {
        let ms = 1_000_000u64;
        let mut tap = simemu_tc1791::engine_io::CellTap { level: true, transitions: 147_793, high_ns: 132_628, ..Default::default() };
        (tap.last_fall_ns, tap.last_rise_ns, tap.low_ns) = (74_516 * ms, 87_098 * ms, 12_582 * ms);
        assert_eq!(super::tap_duty(&tap, 87_100 * ms, super::MAX_PWM_PERIOD_NS), 1.0);
        // a running PWM is still read from its last period
        (tap.level, tap.high_ns, tap.low_ns, tap.last_fall_ns, tap.last_rise_ns) = (false, 250_000, 750_000, 87_099 * ms + 250_000, 87_099 * ms);
        assert_eq!(super::tap_duty(&tap, 87_099 * ms + 500_000, super::MAX_PWM_PERIOD_NS), 0.25);
    }

    /// Plant → ECU sensor path end to end: the plant's own crank/cam edges (engine held at 1000 rpm by a
    /// bench motor, `set_speed` every exchange step) synchronise the unmodified firmware, and the decoded
    /// coil outputs land at the same angle before each cylinder's own firing TDC of the plant.
    /// The engine rests until the firmware has armed its crank front end (FPCCTR0 = 0x00930046, as in a
    /// real start: ECU up with KL15, then cranking). A "flying start" — crank already turning while the
    /// front end is armed — synchronises or not depending on the angle at arming: with the corrected
    /// TriCore cores (native, patched Unicorn) starting at 0/30/150/180 °CRK from power-on the firmware
    /// fires one coil and drops N to 0, any angle works once the crank starts after arming (VERIFIED-T).
    #[test]
    fn plant_edges_synchronise_firmware_and_sparks_decode_per_cylinder() {
        use super::*;
        use crate::studio::session::{S50_EEPROM, S50_FLASH};
        use simemu_physics::{Ambient, DrivetrainProfile, EngineProfile, PlantActuators};
        let Some(root) = crate::paths::test_workspace_root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let flash = crate::paths::read_firmware(&root, S50_FLASH).unwrap();
        let eeprom = crate::paths::read_firmware(&root, S50_EEPROM).unwrap();
        let nvm = crate::immo_lab::ecu_binding(true, &flash, &eeprom).nvm;
        let mut board = Simos18Board::new(&flash, &nvm).unwrap();
        let mut plant = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient::default());
        plant.driver.ignition_on = true;
        let (mut link, mut act) = (EcuSensorLink::default(), PlantActuators::default());
        let omega = 1000.0 * std::f64::consts::PI / 30.0;
        let mut armed = false;
        for _ in 0..1000 {
            feed_ecu(&mut board, &plant.telemetry(), &plant.driver, plant.bench);
            armed |= board.engine_io_mut().gpta.mmio_read(0xF000_1848, 4) == 0x0093_0046;
            plant.engine.set_speed(if armed { omega } else { 0.0 });
            plant.step(1e-3, &act);
            feed_crank_cam(&mut board, &mut link, &plant, 1e-3);
            let dt = link.board_time(plant.sim_time_s) - board.board_time_s();
            board.run_for(dt.max(0.0)).unwrap();
            board.sync_engine_io();
            act.ecu = decode_ecu_outputs(&board, &link);
            // first cycle on all four coils (start ignition angle; the angle moves once the start is over)
            if act.ecu.ignition.iter().all(Option::is_some) {
                break;
            }
        }
        assert_eq!(board.hw().reset_count, 0);
        for (c, coil) in act.ecu.ignition.iter().enumerate() {
            let coil = coil.unwrap_or_else(|| panic!("cylinder {} not fired", c + 1));
            eprintln!("cyl {} spark {:.1}° BTDC dwell {:.2} ms", c + 1, coil.spark_btdc_deg, coil.dwell_ms);
            assert!((0.0..20.0).contains(&coil.spark_btdc_deg), "cyl {} {:?}", c + 1, coil);
            assert!((0.3..10.0).contains(&coil.dwell_ms));
        }
        let a: Vec<f64> = act.ecu.ignition.iter().map(|c| c.unwrap().spark_btdc_deg).collect();
        assert!(a.iter().all(|x| (x - a[0]).abs() < 1.0), "same start angle on every cylinder: {a:?}");
    }

    /// Injector requests → plant pulses: the pulses of the latest cycle whose firing TDC has passed, as angle
    /// before that TDC (cylinder 1: TDC at 0/720/1440 °CRK); a cycle still in progress is not reported, stale
    /// commands expire like sparks.
    #[test]
    fn injection_pulses_report_the_last_complete_cycle() {
        use super::*;
        let p = |s: f64, e: f64| Pulse { start_s: s / 1000.0, start_deg: s, end_s: e / 1000.0, end_deg: e };
        // two pulses per cycle at 85° and 43° BTDC (start-time pattern of SC800S50, studio::engine_run)
        let pulses = [p(635.0, 636.5), p(677.0, 677.6), p(1355.0, 1356.5)];
        let w = |x: &Option<InjPulse>| x.map(|q| (q.soi_btdc_deg, (q.width_ms * 1e3).round()));
        let r = injection_pulses(&pulses, 0.0, 1.4, 1400.0);
        assert_eq!(r.iter().map(w).collect::<Vec<_>>(), vec![Some((85.0, 1500.0)), Some((43.0, 600.0)), None], "cycle ending at 720");
        let r = injection_pulses(&pulses, 0.0, 1.45, 1445.0);
        assert_eq!(r.iter().map(w).collect::<Vec<_>>(), vec![Some((85.0, 1500.0)), None, None], "cycle ending at 1440");
        assert!(injection_pulses(&pulses, 0.0, 3.0, 1445.0).iter().all(Option::is_none), "expired");
        // the cycle ending at 2160 had no pulses (fuel cut-off / injection mode switch): nothing is repeated
        assert!(injection_pulses(&pulses, 0.0, 2.17, 2170.0).iter().all(Option::is_none), "no pulses in the last cycle");
    }

    /// Timing output → plant drives: edges keep their board times + the fixed window delay, a repeated level is no edge,
    /// the phase in progress is open (∞), phases the plant has passed drop out, at most the last three are kept.
    #[test]
    fn timing_output_phases_become_delayed_drives() {
        use super::*;
        let ms = |x: f64| (x * 1e6) as u64;
        let to_plant = |t: u64| t as f64 / 1e9 + ACTUATOR_DELAY_S;
        // idle high, low 1.0..1.5 ms (a repeated low at 1.2 ms), low 3.0..3.4 ms, low from 5.0 ms (in progress)
        let ev = [(0, true), (ms(1.0), false), (ms(1.2), false), (ms(1.5), true), (ms(3.0), false), (ms(3.4), true), (ms(5.0), false)];
        let d = drives_of(ev.into_iter(), false, to_plant, 5.1e-3);
        let got: Vec<(f64, f64)> = d.iter().flatten().map(|d| ((d.on_s * 1e4).round() / 10.0, (d.off_s * 1e4).round() / 10.0)).collect();
        assert_eq!(got, vec![(4.0, 4.4), (6.0, f64::INFINITY)], "phase 1 passed by the plant, phase 2 kept, phase 3 open");
        let d = drives_of(ev.into_iter(), false, to_plant, 1.6e-3);
        assert_eq!(d[0].map(|d| ((d.on_s * 1e4).round(), (d.off_s * 1e4).round())), Some((20.0, 25.0)), "starts at the first low, not the repeat");
        // the first event is the level, not an edge
        assert!(drives_of([(ms(1.0), false), (ms(2.0), true)].into_iter(), false, to_plant, 0.0).iter().all(Option::is_none));
        // coil: a dwell in progress ends after the previous dwell length (2 ms), unless that time has passed
        let coil = [(0, true), (ms(10.0), false), (ms(12.0), true), (ms(30.0), false)];
        let mut v = phases(coil.into_iter(), false, |t| t as f64 / 1e9);
        predict_dwell_end(&mut v, 31e-3);
        assert_eq!(v.last().map(|d| (d.off_s * 1e4).round()), Some(320.0));
        let mut v = phases(coil.into_iter(), false, |t| t as f64 / 1e9);
        predict_dwell_end(&mut v, 33e-3);
        assert!(v.last().is_some_and(|d| d.off_s.is_infinite()), "longer than the previous dwell: wait for the real end");
    }

    /// One wheel: the plant's sensor wheels (edges fed to the ECU) equal the board-side geometry, which is
    /// checked against the firmware's NC constants by `crank_wheel::tests::geometry_matches_flash`.
    #[test]
    fn wheel_geometry_matches_board_wheel() {
        let g = WheelGeometry::simos18_sc8s50();
        assert_eq!((g.teeth, g.missing), (60, 2));
        assert_eq!(CRANK_TOOTH0_DEG, -g.ref_to_tdc0_deg);
        for (k, edges) in [g.cam_in_edges_deg, g.cam_ex_edges_deg].iter().enumerate() {
            for (i, &e) in edges.iter().enumerate() {
                let level = (i % 2 == 0) == g.cam_edge0_rising;
                assert!(CAM_EDGES_DEG[k].contains(&(e, level)), "cam {k} edge {e}° level {level}");
            }
        }
    }
}
