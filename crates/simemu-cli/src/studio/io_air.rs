//! Pin-level glue for the charge-air, valve-timing and knock hardware of the engine ECU (companion of `io.rs`).
//!
//! Decoded outputs (SC800S50 evidence: docs/SIMOS18_GPTA_IO_RE.md §12.6):
//! * Electric wastegate actuator V465 = ATIC111 bridge 2 (driver config 0x8004F46C entry 2: BSW PWM channel 3, direction
//!   P13.6, disable P4.9). The CHRA position controller (FR 9.26, 1 ms) ends in INFR SetPwmDucyBpa 0x800A5448 →
//!   H-bridge SetDuty 0x801F5854(handle 2, PWM_BPA/2): + = towards open. PWM = LTCA2 LTC32 quad, output cell L2.LTC35,
//!   1 kHz, high = on (VERIFIED-T: −14.86 % → 0.148 ms high per 1 ms); P13.6 high = positive duty (VERIFIED-T).
//!   Plant convention: `EcuOutputs::wastegate_duty` + = towards the seat → the sign is inverted here.
//! * Diverter (recirculation) valve N249 = ATIC140#1 stage 10 (INFR SetCmdRcl 0x800A549C, MSC1 data bit 10;
//!   LV_CMD_RCL_OPEN, FR 9.7), energised = open (VERIFIED-S).
//!
//! * Cam phaser solenoids N205 (intake) / N318 (exhaust) (docs/SIMOS18_GPTA_IO_RE.md §12.8): BSW PWM channels 0x11 /
//!   0x12 (handle table 0x80042C7C) = G0.LTC24 quad → output **G0.LTC27** / LTCA2 LTC28 quad → output **L2.LTC31**,
//!   150 Hz (`ip_frq_pwm_vcp`), **low = current on** (VERIFIED-T: low fraction of G0.LTC27 = the firmware's hold duty
//!   within 0.1 %; L2.LTC31 VERIFIED-S by its CTR class), load = ATIC140#1 stage 0 / 1 (electrical diagnosis ids
//!   0x100 / 0x101). Tapped at the cells (the TC1791 GPTA→MSC encoding is UNVERIFIED) and gated like the other
//!   ATIC140 stages (NDIS1 high, LS_DIS0 low). Index 0 = intake = cam 0 (VERIFIED-T).
//! * ATIC140#1 stages (MSC1 data bits, DIO 0x2000 + n; VERIFIED-S): 2 = N428 oil-pump low-pressure stage
//!   (LV_PUMP_OIL_EL_SP, energised = low stage), 4 = N522 piston-cooling jets (`lc_oil_2_pist_ivs` = 1: energised =
//!   jets off), 11 = N316 port flaps (!PORT_DEAC: energised = flaps open).
//!
//! * Oil-pressure switches (ENLU 27.12, FR p.7348: GetPoilSwiH / GetPoilSwiL, 100 ms; `c_idx_cod_poil_det_conf` 0x7FF
//!   enables both pin evaluations): INFR 0x800A5A78 reads **P6.14** → 0xD0000E4A, 0x800A5A70 reads **P7.5** → 0xD0000E4B,
//!   both the raw pin level (ACQ 0x808DF9E0; VERIFIED-S). The codeword's pin bits (FR p.7341: bit 8 swaps H/L, bits 9/10
//!   invert H/L; all set in SC8S50 and SC8O40) give LV_POIL_SWI_L (0xD0000E3B) = !P6.14 and LV_POIL_SWI_H (0xD0000E37) =
//!   !P7.5 (OPM 0x808DF7A8, init 0x800E9C24; VERIFIED-S): P6.14 = reduced-pressure switch F378, P7.5 = oil pressure switch
//!   F22, both closing to ground at pressure ("switch active"); switch points UNVERIFIED (part data). The piston-jet switch F447
//!   (P6.15, getter 0x800A5A80 inverted) follows the plant's jet-gallery pressure (`OIL_SWITCH_JETS`).
//! * Oil level / temperature sender G266 on P3.4 ([`feed_oil_sender`]): TOG pulse train of
//!   `simemu_physics::sensors::OilLevelSender` from the sump oil temperature and level (firmware decode VERIFIED-T,
//!   `simemu-tc1791/tests/oil_sensors.rs`).
//!
//! Inputs: wastegate position sensor → ADC1.CH5 (logical 0x4005, getter 0x800A6990; VERIFIED-S/T), 0–5 V ratiometric;
//! port-flap position sensor → ADC2.CH8 (logical 0x8005, `0x801801A6` → `ip_v_port_1_cor`; VERIFIED-S).
//!
//! * Rotary coolant valve N493 = ATIC111 bridge 1 (driver config 0x8004F47C: BSW PWM channel 1, direction P4.5 not
//!   inverted, disable P13.9; ENTE 34.25, FR p.9095): PWM = G1.LTC28 quad, output cell G1.LTC31 (→ P4.4), high = on like
//!   bridges 0/2 (same driver 0x801F5854). Direction: P4.5 high (the firmware's positive duty) turns the valve towards a
//!   *smaller* angle (plant convention `EcuOutputs::rotary_valve_duty` + = larger angle) — VERIFIED-T by the closed loop:
//!   with this sign RVC_AV (0xD0012574) follows the firmware's set point within ±1° while the valve travels 170° → 58°;
//!   with the opposite sign the controller drove the valve into the stops at 100 % duty.
//! * Its position sensor → SENT input 0 (logical SENT channel 0 → hw channel 0, table 0x800426E0; INFR
//!   GetDigRvcSens 0x800A5D9C reads D1..D3 as a 12-bit value, MSN first, status nibble bit 0 = sensor error).
//! * Radiator cooling fan controller J293 (ECF, FR ENTE 34.20/34.21 pp. 9058–9076, CUSF 11.56 pp. 3340–3350): BSW PWM
//!   channel 0xF (table 0x80042400 entry 0x800425A4, REA `ev_828823eb…`; setters `SetPwmDucyEcf` 0x800A6200
//!   `ev_f0ee78f7…` and `SetPwmFrqEcf` 0x800A63C8 `ev_9d60cc6d…`) = G0.LTC12 quad (CLK6 = 351.56 kHz, period 0x1B77 =
//!   7031 → 50.0 Hz), output cell **G0.LTC15**, **high = low-side stage on** like EFP channel 0xE (VERIFIED-T: standby
//!   `26/256` = 10.156 % sets G0.LTC15 high fraction = 0.1016); tapped at the cell and gated by the ATIC140 enables
//!   (`ndis1` && `!ls_dis0`, physical stage / MSC routing UNVERIFIED). J293 control curve (`fans_nengcorfan1var2_m_vw`,
//!   `fans_rfan1devoutadj_t_vw`, `fans_rrgnmaxcodem_c_vw` in SC8S50 264K, VERIFIED-S): ≤ 10.16 % (`26/256`, incl. 0 %
//!   `ECF_POW_AD` pulse) = 0 %, 89.84 % (`230/256`..`253/256`) = 100 %, static 100 % (> 99.5 %, no PWM edges) = 0 %
//!   (`ecf_pwm_to_fan_duty`).

use crate::studio::io::EcuSensorLink;
use simemu_physics::{EcuOutputs, PlantTelemetry, VirtualPlant};
use simemu_tc1791::engine_io::SensorSignal;
use simemu_tc1791::knock_fadc::PinAtom;
use simemu_tc1791::sent::SentSensor;
use simemu_tc1791::Simos18Board;

/// Wastegate actuator PWM output cell (LTCA2 LTC35) and direction pin P13.6.
const WG_PWM_CELL: (u8, u8) = (2, 35);
const WG_DIR_PIN: (usize, u32) = (13, 6);
/// ATIC111 bridge index of the wastegate actuator.
const WG_BRIDGE: usize = 2;
/// Wastegate position sensor input (ADC kernel, channel).
const WG_POS_ADC: (u8, u8) = (1, 5);
/// N249: ATIC140 #1 stage 10.
const N249_STAGE: (usize, usize) = (1, 10);
/// ATIC140 #1 stages: N428 oil-pump stage, N522 piston jets, N316 port flaps.
const N428_STAGE: usize = 2;
const N522_STAGE: usize = 4;
const N316_STAGE: usize = 11;
/// Cam phaser solenoid PWM output cells [intake N205, exhaust N318] (low = current on).
const VCP_PWM_CELLS: [(u8, u8); 2] = [(0, 27), (2, 31)];
/// Intake-manifold flap potentiometer G336: raw pin value 0xD0011954 (follows the pin, VERIFIED-T) = getter id 0x8005 = ADC2.CH8 (0x80180188..),
/// window c_v_port_min/max_diag 0.20/4.80 V (PORT_FB_EL_L/H_1, 0x808C6000). The firmware inverts the pin voltage
/// (ip_v_port_1_cor: 4.995 − V) before comparing with c_v_port_open_ini 3.80 V / c_v_port_close_ini 1.20 V
/// (0x800DB2B6) → pin 1.20 V open, 3.80 V closed — VERIFIED-S (`EngineProfile::port_flap_sensor_v`).
const PORT_FLAP_ADC: (u8, u8) = (2, 8);
/// Oil-pressure switches: (port, pin, closing pressure in bar gauge) — reduced-pressure switch F378 on P6.14 (0.5–0.8 bar)
/// and oil pressure switch F22 on P7.5 (2.3–3.0 bar); mid-band values of the EA888 Gen3 replacement parts, UNVERIFIED.
const OIL_SWITCHES: [(usize, u8, f64); 2] = [(6, 14, 0.65), (7, 5, 2.65)];
/// Piston-jet oil-pressure switch F447 ("oil pressure switch level 3", 06L919081, on the jet gallery behind N522): P6.15,
/// getter 0x800A5A80 returns !pin = LV_POIL_SWI_PIST (1 = pressure, FR p.7443) → closes to ground at pressure. Switch
/// point 0.3–0.6 bar (aftermarket equivalents of 06L919081; 0.45 bar mid-band), UNVERIFIED.
const OIL_SWITCH_JETS: (usize, u8, f64) = (6, 15, 0.45);
/// Exhaust AVS actuators per logical cylinder index (firing order 1-3-4-2) [to STND, to LFT_1] = (ATIC140 chip, stage):
/// low-side stages 16..19 switched by software MSC data bits (pin table 0x80042C40 → 0x8009B250 → 0x800A7ADC; DiagEl
/// adapter 0x800A4D4C labels ACR 0 = to STND, 1 = to LFT_1; stage on = coil energised). VERIFIED-S (SC800S50,
/// docs/SIMOS18_COMBUSTION.md §6b); index = firing order: HYP (c_crk_vvl_acr steps 180 ° per index).
const AVS_STAGES: [[(usize, usize); 2]; 4] = [[(1, 17), (0, 17)], [(1, 16), (0, 19)], [(0, 18), (0, 16)], [(1, 18), (1, 19)]];
/// Rotary coolant valve N493: PWM output cell G1.LTC31, direction pin P4.5, ATIC111 bridge 1, SENT input 0.
const RVC_PWM_CELL: (u8, u8) = (1, 31);
const RVC_DIR_PIN: (usize, u32) = (4, 5);
const RVC_BRIDGE: usize = 1;
const RVC_SENT_CH: usize = 0;
/// Radiator cooling fan controller J293 (ECF): BSW PWM channel 0xF (G0.LTC12 quad, 50 Hz), output cell G0.LTC15
/// (high = ATIC140 low-side stage on, same polarity as EFP channel 0xE; VERIFIED-S/T, REA `ev_828823eb…`, `ev_f0ee78f7…`).
const ECF_PWM_CELL: (u8, u8) = (0, 15);
/// J293 PWM transfer characteristic (`fans_nengcorfan1var2_m_vw` / `fans_rfan1devoutadj_t_vw` / `fans_rrgnmaxcodem_c_vw`
/// in SC8S50, `c_ecfpwm_man` = 9.998 % in SC8O40/SC8S50; VERIFIED-S): 26/256 (10.156 %) = 0 % speed standby (deadband
/// 10.5 % absorbs timer quantization 715/7031 = 10.169 %), 230/256 (89.844 %) = 100 % speed, ≤ 253/256 (98.828 %) max
/// valid command, > 99.5 % static on = off (ground short / incomplete first 50 Hz period).
const ECF_PWM_OFF: f64 = 26.0 / 256.0;
const ECF_PWM_MIN: f64 = 0.105;
const ECF_PWM_MAX: f64 = 230.0 / 256.0;
const ECF_PWM_STATIC_ON: f64 = 0.995;

/// Converts the low-side switch on-time fraction on the J293 control wire (`0.0..=1.0`) to normalized fan speed (`0.0..=1.0`).
pub(crate) fn ecf_pwm_to_fan_duty(pwm: f64) -> f64 {
    if !(ECF_PWM_MIN..=ECF_PWM_STATIC_ON).contains(&pwm) {
        0.0
    } else {
        ((pwm - ECF_PWM_OFF) / (ECF_PWM_MAX - ECF_PWM_OFF)).clamp(0.0, 1.0)
    }
}
/// SENT unit time of the valve's position sensor, ns. UNVERIFIED (the firmware's nominal tick on this channel is
/// 3.03 µs, CFDR 0x9F1 — `simemu_tc1791::sent`).
const RVC_SENT_TICK_NS: f64 = 3000.0;

/// SENT frame of the valve position sensor: fast channel 1 = the 12-bit position in D1..D3 (MSN first, what the
/// firmware reads), fast channel 2 = its complement 4095 − x in D4..D6 with the nibble order reversed (SAE J2716
/// dual-channel convention; the firmware ignores D4..D6 — UNVERIFIED for this sensor), status nibble 0 (no error).
fn rvc_sent_frame(x: u16) -> SentSensor {
    let (x, y) = (x.min(4095), 4095 - x.min(4095));
    let n = |v: u16, k: u32| ((v >> (4 * k)) & 0xF) as u8;
    SentSensor { tick_ns: RVC_SENT_TICK_NS, status: 0, data: [n(x, 2), n(x, 1), n(x, 0), n(y, 0), n(y, 1), n(y, 2)] }
}

/// FADC input of each plant knock sensor: sensor 0 → channel 1 (SC800S50 knock driver config 0x80042388: one input,
/// FADC ch 1, VERIFIED-S/T); a second sensor of a two-sensor engine → channel 2 (configured by the firmware as the
/// second knock group but never started — HYP).
const KNOCK_FADC_CH: [usize; 2] = [1, 2];

/// Block vibration produced by the plant step that just ran → the knock-sensor inputs of the ECU (board time). Call
/// after `plant.step`, before the board runs to the new plant time (the session does, next to `io::feed_crank_cam`).
pub fn feed_knock(board: &mut Simos18Board, link: &EcuSensorLink, plant: &mut VirtualPlant) {
    let off = link.board_time(0.0);
    for (s, &ch) in KNOCK_FADC_CH.iter().enumerate().take(plant.engine_profile.knock.sensors.min(2)) {
        let atoms = plant.engine.vib.unread().map(|a| PinAtom { t0: a.t0 + off, t_end: a.t_end + off, w: a.w, alpha: a.alpha, phi: a.phi, v: a.v[s] });
        board.feed_fadc_input(ch, atoms);
    }
    plant.engine.vib.mark_read();
}

/// Oil level / temperature sender G266 (`simemu_physics::sensors::OilLevelSender`, supplied on the bench while the ECU
/// is powered) → its pulse train on P3.4 (`engine_io::LINE_G266`), from the sump oil temperature and level. Call after
/// `plant.step`, like the crank edges.
pub fn feed_oil_sender(board: &mut Simos18Board, link: &mut EcuSensorLink, plant: &VirtualPlant) {
    let e = link.g266.step(plant.sim_time_s, true, plant.thermal.oil_c, plant.engine.lube.sump_level_mm);
    if !e.is_empty() {
        let v: Vec<_> = e.into_iter().map(|(t, l)| (link.board_time(t), SensorSignal::OilLevelSender(l))).collect();
        board.feed_sensor_edges(&v);
    }
}

/// Plant sensors of this subsystem → ECU inputs (called with the plant state before its step, like `io::feed_ecu`).
pub fn feed(board: &mut Simos18Board, t: &PlantTelemetry) {
    let _ = board.set_adc_input_voltage(WG_POS_ADC.0, WG_POS_ADC.1, t.wastegate_sensor_v as f32);
    let _ = board.set_adc_input_voltage(PORT_FLAP_ADC.0, PORT_FLAP_ADC.1, t.port_flap_sensor_v as f32);
    for (port, pin, bar) in OIL_SWITCHES {
        board.set_port_input(port, pin, Some(t.oil_bar < bar));
    }
    board.set_port_input(OIL_SWITCH_JETS.0, OIL_SWITCH_JETS.1, Some(t.oil_jet_bar < OIL_SWITCH_JETS.2));
    board.set_sent_sensor(RVC_SENT_CH, Some(rvc_sent_frame(t.rotary_valve_sensor)));
}

/// ECU outputs of this subsystem → plant actuators.
pub fn decode(board: &Simos18Board, ps: &simemu_tc1791::power_stages::PowerStageOutputs, o: &mut EcuOutputs) {
    let opening = board.port_output(WG_DIR_PIN.0, WG_DIR_PIN.1);
    o.wastegate_duty = ps.bridges[WG_BRIDGE].enabled.then(|| {
        let d = crate::studio::io::ltc_duty(board, WG_PWM_CELL);
        if opening { -d } else { d }
    });
    o.diverter_open = Some(ps.atic140[N249_STAGE.0][N249_STAGE.1].on);
    let st = |k: usize| ps.atic140[1][k].on;
    o.oil_pump_low_stage = Some(st(N428_STAGE));
    o.piston_jets_off = Some(st(N522_STAGE));
    o.tumble_flaps_closed = Some(!st(N316_STAGE));
    let stages_enabled = ps.lines.ndis1 && !ps.lines.ls_dis0;
    o.rotary_valve_duty = ps.bridges[RVC_BRIDGE].enabled.then(|| {
        let d = crate::studio::io::ltc_duty_bounded(board, RVC_PWM_CELL);
        if board.port_output(RVC_DIR_PIN.0, RVC_DIR_PIN.1) { -d } else { d }
    });
    o.cam_phaser_duty = VCP_PWM_CELLS.map(|c| stages_enabled.then(|| 1.0 - crate::studio::io::ltc_duty(board, c)));
    o.fan_duty = stages_enabled.then(|| ecf_pwm_to_fan_duty(crate::studio::io::ltc_duty(board, ECF_PWM_CELL)));
    // AVS: plant order [to LFT_1, to STND] per physical cylinder
    let on = |(chip, k): (usize, usize)| Some(ps.atic140[chip][k].on);
    o.avs_actuator = std::array::from_fn(|c| {
        let [stnd, lft1] = AVS_STAGES[(simemu_physics::crank_train::FIRING_OFFSET_DEG[c] / 180.0).round() as usize];
        [on(lft1), on(stnd)]
    });
}

#[cfg(test)]
mod tests {
    use super::ecf_pwm_to_fan_duty;

    #[test]
    fn j293_pwm_transfer_curve_matches_fans_calibration() {
        // 0 % (`ECF_POW_AD` variant pulse / idle high) and standby `715 / 7031 = 10.169 %` (`c_ecfpwm_man`) stay exact 0.
        assert_eq!(ecf_pwm_to_fan_duty(0.0), 0.0);
        assert_eq!(ecf_pwm_to_fan_duty(26.0 / 256.0), 0.0);
        assert_eq!(ecf_pwm_to_fan_duty(715.0 / 7031.0), 0.0);
        // Lowest active Nachlauf step (`38 / 256 = 14.844 %`), 12.5 % map point (`51 / 256`), and midpoint (`50 %`).
        assert!((ecf_pwm_to_fan_duty(38.0 / 256.0) - 12.0 / 204.0).abs() < 1e-9);
        assert!((ecf_pwm_to_fan_duty(51.0 / 256.0) - 25.0 / 204.0).abs() < 1e-9);
        assert!((ecf_pwm_to_fan_duty(0.5) - 0.5).abs() < 1e-9);
        // Full speed (`230 / 256 = 89.844 %`) up to ceiling `fans_rrgnmaxcodem_c_vw` (`253 / 256 = 98.828 %`) = 1.0.
        assert_eq!(ecf_pwm_to_fan_duty(230.0 / 256.0), 1.0);
        assert_eq!(ecf_pwm_to_fan_duty(253.0 / 256.0), 1.0);
        // Static low (100 % low = wire short / incomplete first 50 Hz cycle) and NaN must never spin the fan.
        assert_eq!(ecf_pwm_to_fan_duty(1.0), 0.0);
        assert_eq!(ecf_pwm_to_fan_duty(f64::NAN), 0.0);
    }

    /// End-to-end check on unmodified SIMOS18.1 firmware (`8V0906264H__0005.bin`):
    /// 1. At 85 °C coolant / 60 °C radiator (`G83`), `G0.LTC15` outputs the ~10.16 % 50 Hz standby PWM → `fan_duty == Some(0.0)`.
    /// 2. After starting a hot engine (117 °C coolant > `fans_tcoengushi_c_vw` 116.25 °C, 110 °C radiator `G83`), `ENTE`
    ///    holds standby during the post-start alternator stabilization window (`c_t_ast_ecf_act_not_prot = 5.0 s`) and then
    ///    commands active cooling on `G0.LTC15` (`fan_duty > 0.20`).
    #[test]
    fn firmware_drives_radiator_fan_pwm_when_hot() {
        let Some(root) = crate::paths::test_workspace_root() else {
            eprintln!("SKIP: images not found");
            return;
        };
        if !crate::paths::has_firmware(&root, crate::paths::DEFAULT_ECU_FLASH) {
            eprintln!("SKIP: {} not found", crate::paths::DEFAULT_ECU_FLASH);
            return;
        }
        let mut s = crate::studio::session::StudioSession::with_build(
            &root,
            crate::studio::bench_build::BuildState::from_config(simemu_physics::bench_config::BenchConfig::default()),
        )
        .expect("session");
        if !s.immo_lab {
            s.apply_preset(&crate::studio::session::PresetRequest {
                action: "immo_lab".into(),
                engine_slug: None,
                drivetrain_slug: None,
                fuel_ron: None,
                ambient_temp_c: None,
                altitude_m: None,
                humidity_pct: None,
                image: None,
            })
            .expect("immo_lab");
        }
        s.plant.ambient.temp_c = 30.0;
        s.plant.soak(85.0);
        s.plant.thermal.radiator_c = 60.0;
        s.eot_clock_s = 300.0;
        s.power_cycle_ecu().expect("boot at 85 °C");
        s.plant.driver.ignition_on = true;
        s.step(0.5);
        let standby_on = crate::studio::io::ltc_duty(&s.board, super::ECF_PWM_CELL);
        assert!(
            (0.095..=0.105).contains(&standby_on),
            "expected ~10.16 % standby switch on-time on G0.LTC15, got {:.4}",
            standby_on
        );
        assert_eq!(s.actuators.ecu.fan_duty, Some(0.0));

        // Boot on a hot engine (117 °C coolant, 110 °C radiator outlet G83), let KL15/crank front-end arm (0.5 s), then
        // motor at 1200 rpm past `c_t_ast_ecf_act_not_prot = 5.0 s`.
        s.plant.soak(117.0);
        s.plant.thermal.radiator_c = 110.0;
        s.power_cycle_ecu().expect("boot at 117 °C");
        s.plant.driver.ignition_on = true;
        s.step(0.5);
        s.plant.motoring_rpm = Some(1200.0);
        for _ in 0..14 {
            s.plant.thermal.engine_c = 117.0;
            s.plant.thermal.radiator_c = 110.0;
            s.step(0.5);
        }
        let hot_duty = s.actuators.ecu.fan_duty.expect("stages enabled");
        assert!(hot_duty > 0.20, "expected active J293 fan command at 117 °C / 110 °C G83 after 5 s T_AST, got {hot_duty:.3}");
    }

    /// Nachlauf radiator-fan afterrun (`fans_tiaftrunthd_c_vw = 60 s` minimum prior engine run, `fans_rfan1aftrunvar2_m_vw`
    /// ambient/coolant map): after > 60 s running at 30 °C ambient and 117 °C coolant, switching KL15 off keeps KL87 closed
    /// and drives J293 above standby while the engine is stopped.
    #[test]
    #[ignore = "slow: >60 s engine run required by fans_tiaftrunthd_c_vw before Nachlauf fan activates"]
    fn firmware_drives_radiator_fan_during_nachlauf_after_60s_hot_run() {
        let Some(root) = crate::paths::test_workspace_root() else {
            eprintln!("SKIP: images not found");
            return;
        };
        if !crate::paths::has_firmware(&root, crate::paths::DEFAULT_ECU_FLASH) {
            eprintln!("SKIP: {} not found", crate::paths::DEFAULT_ECU_FLASH);
            return;
        }
        let mut s = crate::studio::session::StudioSession::with_build(
            &root,
            crate::studio::bench_build::BuildState::from_config(simemu_physics::bench_config::BenchConfig::default()),
        )
        .expect("session");
        if !s.immo_lab {
            s.apply_preset(&crate::studio::session::PresetRequest {
                action: "immo_lab".into(),
                engine_slug: None,
                drivetrain_slug: None,
                fuel_ron: None,
                ambient_temp_c: None,
                altitude_m: None,
                humidity_pct: None,
                image: None,
            })
            .expect("immo_lab");
        }
        s.plant.ambient.temp_c = 30.0;
        s.warm_car();
        s.power_cycle_ecu().expect("boot warm");
        s.plant.driver.ignition_on = true;
        s.step(0.5);
        s.plant.driver.brake_pct = 50.0;
        s.plant.driver.start_button = true;
        s.step(2.5);
        s.plant.driver.start_button = false;
        s.plant.driver.brake_pct = 0.0;
        assert!(s.plant.telemetry().rpm > 500.0, "engine must start, got {:.0} rpm", s.plant.telemetry().rpm);
        for _ in 0..62 {
            s.plant.thermal.engine_c = 117.0;
            s.plant.thermal.radiator_c = 110.0;
            s.step(1.0);
        }
        s.plant.driver.ignition_on = false;
        s.step(2.0);
        assert!(s.plant.telemetry().rpm < 50.0);
        assert_eq!(s.actuators.ecu.main_relay, Some(true), "ECU must hold KL87 main relay during Nachlauf");
        let afterrun_duty = s.actuators.ecu.fan_duty.unwrap_or(0.0);
        assert!(afterrun_duty > 0.05, "expected Nachlauf fan duty > 5 % after 62 s hot run, got {afterrun_duty:.3}");
    }

    /// Natural closed-loop stationary warmup (no forced thermal writes): starting from the default 90 °C warm Octavia A7 FL
    /// (`5 °C` ambient, `Road` in `N` at `0 km/h`) and revving on the pedal saturates natural radiator draft, opens the ITM
    /// rotary coolant valve N493, heats `G83`, and causes unmodified SIMOS18.1 `FANS` to turn on the radiator fan J293
    /// (`fan_duty > 0.15`) and regulate coolant temperature without boiling.
    #[test]
    #[ignore = "slow: natural closed-loop stationary rev-warmup until ITM N493 saturates and FANS engages J293"]
    fn firmware_turns_on_radiator_fan_naturally_during_stationary_warmup() {
        let Some(root) = crate::paths::test_workspace_root() else {
            eprintln!("SKIP: images not found");
            return;
        };
        if !crate::paths::has_firmware(&root, crate::paths::DEFAULT_ECU_FLASH) {
            eprintln!("SKIP: {} not found", crate::paths::DEFAULT_ECU_FLASH);
            return;
        }
        let mut s = crate::studio::session::StudioSession::with_build(
            &root,
            crate::studio::bench_build::BuildState::from_config(simemu_physics::bench_config::BenchConfig::default()),
        )
        .expect("session");
        if !s.immo_lab {
            s.apply_preset(&crate::studio::session::PresetRequest {
                action: "immo_lab".into(),
                engine_slug: None,
                drivetrain_slug: None,
                fuel_ron: None,
                ambient_temp_c: None,
                altitude_m: None,
                humidity_pct: None,
                image: None,
            })
            .expect("immo_lab");
        }
        s.warm_car();
        s.power_cycle_ecu().expect("boot warm");
        s.plant.driver.ignition_on = true;
        s.step(0.5);
        s.plant.driver.brake_pct = 50.0;
        s.plant.driver.start_button = true;
        s.step(2.5);
        s.plant.driver.start_button = false;
        s.plant.driver.brake_pct = 0.0;
        assert!(s.plant.telemetry().rpm > 500.0, "engine must start");

        // Blip/hold throttle at standstill in N like an operator warming the engine (8 s rev, 2 s idle so VAG's 20 s
        // stationary pedal-hold limiter resets), then let it idle once hot so ITM + FANS hand off from N493 to J293.
        s.plant.driver.selector = simemu_physics::Selector::N;
        let mut max_fan = 0.0_f64;
        for sec in 1..=180 {
            s.plant.driver.pedal_pct = if s.plant.telemetry().coolant_c < 112.0 && sec % 10 < 8 { 40.0 } else { 0.0 };
            s.step(1.0);
            let tel = s.plant.telemetry();
            let fan = tel.ecu_outputs.fan_duty.unwrap_or(0.0);
            max_fan = max_fan.max(fan);
            if sec % 5 == 0 || fan > 0.05 {
                eprintln!(
                    "t={sec:3}s rpm={:4.0} TCO={:5.1}°C G83={:5.1}°C TOIL={:5.1}°C N493={:5.1}° ({:3.0}%) fan={:.3}",
                    tel.rpm, tel.coolant_c, tel.radiator_c, tel.oil_c, tel.rotary_valve_deg, tel.radiator_valve_pct, fan
                );
            }
            if max_fan > 0.15 && sec >= 30 {
                break;
            }
        }
        assert!(max_fan > 0.15, "SIMOS18.1 must turn on J293 naturally during stationary rev-warmup, max_fan={max_fan:.3}");
        assert!(s.plant.telemetry().coolant_c < 120.0, "coolant must not boil: {:.1} °C", s.plant.telemetry().coolant_c);
    }
}
