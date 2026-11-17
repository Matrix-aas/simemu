//! DQ200_G2 mechatronic (TC1784) ↔ plant: the electrical side of the valve coils.
//!
//! Sensors and decoded outputs are exchanged by `io::{feed_tcu, decode_tcu_outputs}`. The board also
//! needs each valve coil's resistance (copper temperature, plant) and inductance: its coil-load model
//! (`simemu_tc1791::tc1784_io::CoilLoad`) integrates the coil current per PWM edge at board time
//! resolution and puts it on the current-sense inputs the firmware's 3 kHz current controllers read.
//! A 10 ms plant exchange cannot close a 3 kHz current loop, so the coil electrics live with the board.
//! Docs: `docs/DQ200_IO_MAP.md`.

use simemu_physics::mechatronic::MechatronicHw;
use simemu_physics::{PlantTelemetry, TcuOutputs, VirtualPlant};
use simemu_tc1791::tcu_dq200::VALVE_PWM_CHANNEL_OF_SOLENOID;
use simemu_tc1791::{Dq200TcuBoard, Dq250Inputs, Dq250TcuBoard, Simos18Board};

/// MultiCAN node of the DQ250 board wired to the vehicle's drivetrain CAN.
pub const DQ250_CAN_NODE: u8 = 1;

/// The TCU board of the bench build's gearbox: DQ200_G2 (TC1784) or DQ250 0D9 (TC1766).
#[allow(clippy::large_enum_variant)]
pub enum TcuBoard {
    Dq200(Dq200TcuBoard),
    Dq250(Dq250TcuBoard),
}

impl TcuBoard {
    pub fn cpu(&self) -> &Simos18Board {
        match self {
            Self::Dq200(t) => &t.cpu,
            Self::Dq250(t) => &t.cpu,
        }
    }

    pub fn cpu_mut(&mut self) -> &mut Simos18Board {
        match self {
            Self::Dq200(t) => &mut t.cpu,
            Self::Dq250(t) => &mut t.cpu,
        }
    }

    /// A frame from the vehicle bus (the session's node 0) into the board: the DQ250 has the drivetrain CAN on MultiCAN
    /// node 1 (its periodic frames and UDS are there, `tests/dq250_boot.rs`), the DQ200 on node 0.
    pub fn inject_can_rx(&mut self, f: &simemu_tc1791::CanFrame) -> usize {
        match self {
            Self::Dq200(t) => t.cpu.inject_can_rx(f),
            Self::Dq250(t) => t.cpu.inject_can_rx(&simemu_tc1791::CanFrame { node: DQ250_CAN_NODE, ..*f }),
        }
    }

    /// Frames the board transmitted since the last call, on the session's vehicle bus (node 0).
    pub fn take_can_tx(&mut self) -> Vec<simemu_tc1791::CanFrame> {
        let mut v = self.cpu_mut().take_can_tx();
        for f in &mut v {
            f.node = 0;
        }
        v
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn dq200(&self) -> Option<&Dq200TcuBoard> {
        match self {
            Self::Dq200(t) => Some(t),
            Self::Dq250(_) => None,
        }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn dq200_mut(&mut self) -> Option<&mut Dq200TcuBoard> {
        match self {
            Self::Dq200(t) => Some(t),
            Self::Dq250(_) => None,
        }
    }

    pub fn dq250(&self) -> Option<&Dq250TcuBoard> {
        match self {
            Self::Dq250(t) => Some(t),
            Self::Dq200(_) => None,
        }
    }

    /// The DQ250's SPI EEPROM as the firmware left it: (image id, content, bytes written since power-on). `None` for the
    /// DQ200 (it runs from its EEPROM dump).
    pub fn nvm(&self) -> Option<(&str, &[u8], u64)> {
        let t = self.dq250()?;
        let e = &t.periph().eeprom;
        Some((t.info.asw_id.as_str(), &e.mem, e.written_bytes))
    }

    /// This board belongs to gearbox `d` (DQ250 0D9 board for a `0D9…` TCU, DQ200 board otherwise).
    pub fn fits(&self, d: &simemu_physics::DrivetrainProfile) -> bool {
        matches!(self, Self::Dq250(_)) == d.tcu_code.starts_with("0D9")
    }

    pub fn sw_id(&self) -> String {
        match self {
            Self::Dq200(t) => t.info.sw_version_tag.clone(),
            Self::Dq250(t) => t.info.asw_id.clone(),
        }
    }

    pub fn region(&self, pc: u32) -> &'static str {
        match self {
            Self::Dq200(_) => Dq200TcuBoard::classify_region(pc),
            Self::Dq250(_) => match pc & 0x0FFF_FFFF {
                0x0000_0000..=0x0000_7FFF => "SBOOT",
                0x0001_0000..=0x0001_7FFF => "CBOOT",
                0x0004_0000..=0x0017_7FFF => "ASW",
                _ => "RAM/other",
            },
        }
    }

    /// Plant state → the board's sensor inputs (and the DQ200 valve coil R/L, `coils`).
    pub fn feed(&mut self, t: &PlantTelemetry, d: &simemu_physics::DriverInputs, coils: Option<(Vec<f64>, f64)>) {
        match self {
            Self::Dq200(b) => {
                crate::studio::io::feed_tcu(b, t);
                if let Some((r, l)) = coils.and_then(|(r, l)| Some((<[f64; 8]>::try_from(r).ok()?, l))) {
                    b.set_coil_params(r, l);
                }
            }
            Self::Dq250(b) => {
                b.set_inputs(&dq250_inputs(t, d));
                if let Some((r, l)) = coils {
                    b.set_coil_params(&r, l);
                }
            }
        }
    }

    /// Decoded firmware outputs for the plant.
    pub fn decode(&self) -> TcuOutputs {
        match self {
            Self::Dq200(b) => crate::studio::io::decode_tcu_outputs(b),
            Self::Dq250(b) => {
                let o = b.decode_outputs();
                let mut out = TcuOutputs::default();
                out.solenoid_duty[..11].copy_from_slice(&o.duty);
                if o.current_a.iter().any(Option::is_some) {
                    let mut c = [0.0; simemu_physics::mechatronic::SOLENOID_CHANNELS];
                    for (k, i) in o.current_a.iter().enumerate() {
                        c[k] = i.unwrap_or(0.0);
                    }
                    out.solenoid_current_a = Some(c);
                }
                out
            }
        }
    }
}

/// DQ250 mechatronic inputs from the plant: speeds, clutch pressures, temperatures, fork travels, selector cable.
/// Output speed: the output shaft of the 5/6/R final drive — the firmware converts the output sensors with that final's
/// pinion/wheel teeth (`× 18/62` on 0D9300011R, docs/DQ250_IO_MAP.md §2).
pub fn dq250_inputs(t: &PlantTelemetry, d: &simemu_physics::DriverInputs) -> Dq250Inputs {
    Dq250Inputs {
        battery_v: t.battery_v as f32,
        kl15: d.ignition_on,
        input_rpm: t.dmf_secondary_rpm,
        shaft_rpm: t.shaft_rpm,
        output_rpm: t.output_shaft_rpm[1],
        clutch_bar: t.clutch_bar,
        atf_c: t.mechatronic_oil_c,
        clutch_oil_c: t.mechatronic_oil_c,
        fork_mm: t.fork_mm,
    }
}

/// Coil R (at the plant's coil temperature) and L of the DQ200 valves, per firmware Pwm channel, for
/// `Dq200TcuBoard::set_coil_params` (latched at each co-simulation window start and handed to the TCU
/// worker with the other inputs).
/// Wet units (DQ250): R per plant channel (`mechatronic::wet` order) for `Dq250TcuBoard::set_coil_params`.
pub fn coil_params(plant: &VirtualPlant) -> Option<(Vec<f64>, f64)> {
    let s = &plant.driveline.mech.solenoids;
    match &plant.drivetrain_profile.mechatronic {
        MechatronicHw::Dq200(h) => {
            let mut r = vec![h.solenoid.r20_ohm; 8];
            for (i, &ch) in VALVE_PWM_CHANNEL_OF_SOLENOID.iter().enumerate() {
                r[ch] = s[i].resistance_ohm(&h.solenoid);
            }
            Some((r, h.solenoid.l_h))
        }
        MechatronicHw::Wet(h) => Some(((0..11).map(|i| s[i].resistance_ohm(&h.solenoid)).collect(), h.solenoid.l_h)),
    }
}

#[cfg(test)]
mod tests {
    use crate::studio::session::{StudioSession, S50_FLASH};
    use simemu_physics::mechatronic::MechatronicHw;
    use simemu_physics::{BenchMode, Selector};
    use std::path::PathBuf;
    use std::time::Instant;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    /// The board's V401 motor works against the same gear pump the plant uses for delivery.
    #[test]
    fn board_pump_matches_plant_pump() {
        let MechatronicHw::Dq200(h) = simemu_physics::DrivetrainProfile::dq200_fwd().mechatronic else { panic!("DQ200 profile") };
        assert!((h.pump.torque_per_pa() - simemu_tc1791::tcu_dq200::PUMP_TORQUE_PER_PA).abs() < 1e-15);
        assert!((simemu_physics::hydraulics::DcPumpHw::visc_nm_s(0.01) - simemu_tc1791::tcu_dq200::PUMP_VISC_NM_S).abs() < 1e-15);
    }

    /// The plant's sub-gearbox valves N436/N440 are the firmware's own p -> I table read backwards
    /// (CAL 0x80030E60: pressure 0.01 bar, +0x18: current 0.1 mA; `0x80071D1C`).
    #[test]
    fn plant_sub_valve_is_the_firmware_p_i_table() {
        let Some(root) = root() else { return };
        let Ok(f) = crate::paths::read_firmware(&root, crate::paths::DEFAULT_DQ200_FLASH) else { return };
        let rd = |a: usize| i16::from_le_bytes([f[a - 0x8000_0000], f[a - 0x8000_0000 + 1]]) as f64;
        let MechatronicHw::Dq200(h) = simemu_physics::DrivetrainProfile::dq200_fwd().mechatronic else { panic!("DQ200 profile") };
        for (i, &(amp, pa)) in h.sub_valve_curve.iter().enumerate() {
            assert!((rd(0x8003_0E78 + 2 * i) / 1e4 - amp).abs() < 1e-9 && (rd(0x8003_0E60 + 2 * i) * 1e3 - pa).abs() < 1e-6, "point {i}");
        }
    }

    /// The studio decodes DQ200 fault-memory records with the firmware's own table (UI field `dtc_sae`): the
    /// VAG numbers seen on the bench map to U0002 (event 46), P0810 (event 63) and P0602 (event 51).
    #[test]
    fn dq200_dtcs_decode_with_the_firmware_table() {
        let Some(root) = root() else { return };
        let d = crate::studio::session::dq200_dtc_sae(&root, &["0029B5 status 0x0C".into(), "0029C6 status 0x2E".into(), "0029BB status 0x2F".into(), "123456 status 0x00".into()]);
        assert_eq!(d, ["U0002 (event 46)", "P0810 (event 63)", "P0602 (event 51)", "?"]);
    }

    fn env_f(k: &str, d: f64) -> f64 {
        std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d)
    }

    /// TCU-in-the-loop bench scenario on the studio session (ECU + TCU + plant, firmware unmodified):
    /// car on the chassis dyno (inertia run), KL15 on, brake pressed, selector P; the engine is turned
    /// by the bench motoring drive at `SIMEMU_TCU_MOTOR_RPM` (default 800) from 1 s — test-bench
    /// equipment providing engine speed physically while the ECU's own idle is not stable yet
    /// (MILESTONE §2.1); the ECU still runs and reports what its sensors see on CAN. Selector D at
    /// `SIMEMU_TCU_D_S` (default 4 s); from `SIMEMU_TCU_GO_S` (default D + 3 s) the brake is released,
    /// the pedal pressed (`SIMEMU_TCU_PEDAL`, 30 %) and the motoring drive ramped (`SIMEMU_TCU_RAMP`
    /// 300 rpm/s up to `SIMEMU_TCU_RPM_MAX` 3500 rpm) — the drive pins the crank speed, so the engine
    /// speed does not fall during an upshift as a combusting engine's would. Run `SIMEMU_TCU_S`
    /// (default 19 s; with the clutch characteristic the TCU learned, `simemu_physics::clutch::DryClutchHw::dq200`, the
    /// drive reaches 6th by then — 3rd before): `SIMEMU_CPU=native cargo test --release -p simemu-cli dq200_bench_scenario --
    /// --ignored --nocapture` (`SIMEMU_TCU_DRV=1` prints the drive-away trace).
    ///
    /// Asserted (firmware decisions only, observed on its RAM/CAN): WFS5 accepted; main state 1 -> 3
    /// -> 4 -> 5 with the safety software releasing the outputs within 0.5 s and keeping them; V401
    /// synchronised and the accumulator above 40 bar; in P (before D) 1st and R engaged (gear status
    /// `0xD0005370` = 0x0801), clutches open (< 4.45 mm) and both input shafts standing (< 50 rpm);
    /// main state 9 after D; launch on K1 with slip (input shaft 1 below engine speed while the rollers
    /// turn) to > 5 km/h; upshifts 1 -> 2 -> 3 in the TCU's own `WBA_03.WBA_eing_Gang_02` (0x394
    /// byte 3); no selector-test faults (P0914 / P0919), no clutch-position faults of state 4 (P0810
    /// events 0x74/0x75), no TCU reset; every stored DTC decodes through the firmware's event tables.
    #[test]
    #[ignore = "slow: ~20 s of ECU+TCU+plant co-simulation"]
    fn dq200_bench_scenario() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let mut s = StudioSession::new(&root).expect("session");
        assert!(s.tcu.is_some(), "TCU not booted: {:?}", s.tcu_error);
        let (t_end, t_d, rpm) = (env_f("SIMEMU_TCU_S", 19.0), env_f("SIMEMU_TCU_D_S", 4.0), env_f("SIMEMU_TCU_MOTOR_RPM", 800.0));
        s.plant.bench = BenchMode::ChassisDyno { target_kmh: 0.0 };
        s.plant.driver.ignition_on = true;
        s.plant.driver.brake_pct = 30.0;
        s.plant.driver.selector = Selector::P;
        // Diagnostics: `SIMEMU_TCU_TRACE=lo,hi` (peripheral access trace, hex), `SIMEMU_TCU_WATCH=addr[:n],..`
        // (DSPR bytes printed every step), `SIMEMU_TCU_DT` (print step, s).
        let hex = |s: &str| u32::from_str_radix(s.trim().trim_start_matches("0x"), 16).unwrap();
        if let Ok(r) = std::env::var("SIMEMU_TCU_TRACE") {
            let (lo, hi) = r.split_once(',').unwrap();
            s.tcu.as_mut().unwrap().dq200_mut().unwrap().set_io_trace(Some((hex(lo), hex(hi))));
        }
        let watch: Vec<(u32, usize)> = std::env::var("SIMEMU_TCU_WATCH").unwrap_or_default().split(',').filter(|x| !x.is_empty())
            .map(|a| a.split_once(':').map_or_else(|| (hex(a), 1), |(a, n)| (hex(a), n.parse().unwrap()))).collect();
        {
            // Diagnostic overrides of the UNVERIFIED V401 motor constants (`PUMP_KE`, `PUMP_R`, ...).
            let hw = &mut s.tcu.as_mut().unwrap().dq200_mut().unwrap().pump_mut().hw;
            hw.ke_ll = env_f("PUMP_KE", hw.ke_ll);
            hw.r_ll_ohm = env_f("PUMP_R", hw.r_ll_ohm);
            hw.l_ll_h = env_f("PUMP_L", hw.l_ll_h);
            hw.inertia_kgm2 = env_f("PUMP_J", hw.inertia_kgm2);
            hw.friction_nm = env_f("PUMP_FR", hw.friction_nm);
        }
        if let MechatronicHw::Dq200(h) = &mut s.plant.drivetrain_profile.mechatronic {
            // Diagnostic override of the UNVERIFIED clutch-valve flow coefficient (`CLUTCH_K`, m³/(s·√Pa)).
            h.clutch_valve_k = env_f("CLUTCH_K", h.clutch_valve_k);
        }
        // `SIMEMU_TCU_WWATCH=lo,hi[;lo,hi..]` (Unicorn only): data writes into [lo, hi], printed at the end.
        for r in std::env::var("SIMEMU_TCU_WWATCH").unwrap_or_default().split(';').filter(|x| !x.is_empty()) {
            let (lo, hi) = r.split_once(',').unwrap();
            s.tcu.as_mut().unwrap().dq200_mut().unwrap().cpu.add_write_watch(hex(lo), hex(hi)).expect("write watch needs SIMEMU_CPU=unicorn");
        }
        let mut probes: Vec<u32> = std::env::var("SIMEMU_TCU_PROBES").unwrap_or_default().split(',').filter(|x| !x.is_empty()).map(hex).collect();
        // `SIMEMU_TCU_VEC=1`: count entries of every interrupt vector (BIV 0x80040000, 32 bytes each).
        if std::env::var("SIMEMU_TCU_VEC").is_ok() {
            probes.extend((1..256u32).map(|p| 0x8004_0000 + 32 * p));
        }
        for &p in &probes {
            s.tcu.as_mut().unwrap().dq200_mut().unwrap().cpu.add_probe(p).unwrap();
        }
        let wall = Instant::now();
        let (mut t, dt) = (0.0, env_f("SIMEMU_TCU_DT", 0.1));
        let resets0 = s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().reset_log.len();
        let mut resets_seen = resets0;
        let mut main_before_motor = 0u8;
        let mut max_main = 0u8;
        let mut state3_s: Option<f64> = None;
        let mut released_s: Option<f64> = None;
        let mut max_coil_a = 0f64;
        let (mut pump_sync_s, mut pump_off_s, mut max_bar) = (None::<f64>, None::<f64>, 0f32);
        let (mut max_clutch_mm, mut max_shaft_rpm) = (0f64, 0f64);
        let (mut p_engaged_s, mut state9_s) = (None::<f64>, None::<f64>);
        let (mut max_roller_kmh, mut max_k1_slip_rpm, mut gear_seq) = (0f64, 0f64, Vec::<u8>::new());
        // Drive-away: brake released, pedal pressed and the motoring drive ramped from `t_go`.
        let (t_go, pedal, ramp, rpm_max) = (env_f("SIMEMU_TCU_GO_S", t_d + 3.0), env_f("SIMEMU_TCU_PEDAL", 30.0), env_f("SIMEMU_TCU_RAMP", 300.0), env_f("SIMEMU_TCU_RPM_MAX", 3500.0));
        while t < t_end - 1e-9 {
            if t >= 1.0 {
                s.plant.motoring_rpm = Some(if t >= t_go { (rpm + ramp * (t - t_go)).min(rpm_max) } else { rpm });
            }
            if t >= t_d {
                s.plant.driver.selector = Selector::D;
            }
            if t >= t_go {
                s.plant.driver.brake_pct = 0.0;
                s.plant.driver.pedal_pct = pedal;
            }
            s.step(dt);
            t += dt;
            if t >= t_go - 0.5 && std::env::var("SIMEMU_TCU_DRV").is_ok() {
                let tel = s.plant.telemetry();
                let m = &s.plant.driveline.mech;
                let d = &s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().dspr;
                let u16_at = |o: usize| u16::from_le_bytes([d[o], d[o + 1]]);
                let wba = s.can_log.iter().rev().find(|e| e.src == "TCU" && e.id == "394").map(|e| e.data.clone()).unwrap_or_default();
                println!(
                    "{t:7.2} DRV v {:.1} km/h roll {:.1} km/h eng {:.0} rpm | gears {:04X} tgt {:04X} main {} | K x {:?} mm p {:?} bar | shafts {:.0}/{:.0} rpm | WBA_03 {wba}",
                    tel.vehicle_kmh,
                    tel.roller_kmh,
                    tel.rpm,
                    u16_at(0x5370),
                    u16_at(0x5376),
                    d[0x7EC0],
                    m.clutch_x_m.map(|x| (x * 1e4).round() / 10.0),
                    m.clutch_pa.map(|p| (p / 1e4).round() / 10.0),
                    u16_at(0x7536) as f64 / 10.0,
                    u16_at(0x753A) as f64 / 10.0,
                );
            }
            let tcu = s.tcu.as_ref().unwrap().dq200().unwrap();
            let l = tcu.read_live_state();
            let o = tcu.decode_outputs();
            let cur = tcu.coil_currents();
            {
                let hw = tcu.cpu.hw();
                if hw.reset_log.len() > resets_seen {
                    println!("{t:7.3} TCU reset {:?}, SWRST pcs {:X?}, traps {}", &hw.reset_log[resets_seen..], hw.swrst_log, hw.trap_count);
                    resets_seen = hw.reset_log.len();
                }
            }
            if t < 1.0 {
                main_before_motor = main_before_motor.max(l.main_state);
            }
            max_main = max_main.max(l.main_state);
            let pump_state = tcu.cpu.hw().dspr[0x3400];
            max_bar = max_bar.max(l.system_pressure_bar);
            {
                let m = &s.plant.driveline.mech;
                let d = &tcu.cpu.hw().dspr;
                let shaft = |o: usize| u16::from_le_bytes([d[o], d[o + 1]]) as f64 / 10.0;
                // P phase: clutches open, input shafts standing, 1st + R engaged (gear status 0xD0005370).
                if t < t_d {
                    max_clutch_mm = m.clutch_x_m.iter().fold(max_clutch_mm, |a, &x| a.max(x * 1e3));
                    if t > 3.0 {
                        max_shaft_rpm = max_shaft_rpm.max(shaft(0x7536)).max(shaft(0x753A));
                    }
                    if u16::from_le_bytes([d[0x5370], d[0x5371]]) == 0x0801 && l.main_state == 5 {
                        p_engaged_s.get_or_insert(t);
                    }
                }
                if t >= t_d && l.main_state == 9 {
                    state9_s.get_or_insert(t);
                }
                // Launch: rollers turning while input shaft 1 (G501) slips below the engine speed.
                let tel = s.plant.telemetry();
                max_roller_kmh = max_roller_kmh.max(tel.roller_kmh);
                if t >= t_go && tel.roller_kmh > 1.0 {
                    max_k1_slip_rpm = max_k1_slip_rpm.max(tel.rpm - shaft(0x7536));
                }
                // Engaged gear as the TCU reports it on CAN (WBA_03 byte 3 low nibble, opendbc vw_mqb.dbc).
                if let Some(g) = s.can_log.iter().rev().find(|e| e.src == "TCU" && e.id == "394").and_then(|e| u8::from_str_radix(e.data.split(' ').nth(3)?, 16).ok()) {
                    if t >= t_go && gear_seq.last() != Some(&(g & 0xF)) {
                        gear_seq.push(g & 0xF);
                    }
                }
            }
            if pump_state == 6 {
                pump_sync_s.get_or_insert(t);
            }
            if pump_sync_s.is_some() && pump_state == 2 && l.system_pressure_bar > 50.0 {
                pump_off_s.get_or_insert(t);
            }
            if l.main_state >= 3 && state3_s.is_none() {
                state3_s = Some(t);
            }
            if l.safety_word == 0x5555_5555 {
                released_s.get_or_insert(t);
                max_coil_a = cur.iter().copied().fold(max_coil_a, f64::max);
            } else {
                assert!(released_s.is_none(), "safety shut-off after release at {t:.1} s (errors {})", l.safety_errors);
                assert!(o.solenoid_duty.iter().all(Option::is_none), "valve driven during shut-off at {t:.1} s");
                assert!(cur.iter().all(|&i| i < 1e-3), "coil current during shut-off at {t:.1} s: {cur:?}");
            }
            if std::env::var("SIMEMU_TCU_HYD").is_ok() {
                let m = &s.plant.driveline.mech;
                println!(
                    "{t:7.3} HYD p {:.1} bar acc {:.1} cm3 pump {:.0} rad/s q {:.1} cm3/s sub {:?} cl {:?} mm x {:?} fork v {:?} spool {:?}",
                    m.supply_pa / 1e5,
                    m.acc_oil_m3 * 1e6,
                    m.pump.omega,
                    m.pump_flow_m3_s * 1e6,
                    m.sub_pa.map(|p| (p / 1e4).round() / 10.0),
                    m.clutch_pa.map(|p| (p / 1e4).round() / 10.0),
                    m.clutch_x_m.map(|x| (x * 1e4).round() / 10.0),
                    m.fork_v.map(|v| (v * 1e4).round() / 10.0),
                    m.solenoids.iter().take(8).map(|s| (s.spool * 100.0).round()).collect::<Vec<_>>(),
                );
            }
            if std::env::var("SIMEMU_TCU_IRQS").is_ok() {
                let hw = tcu.cpu.hw();
                println!("{t:7.3} I irqs {} insns {} quanta {} early {} irq_stops {}", hw.irq_dispatch_count, hw.total_instructions, hw.quanta, hw.quanta_stopped_early, hw.fixes.irq_stops);
            }
            if std::env::var("SIMEMU_TCU_PROBE_STEP").is_ok() {
                for &p in &probes {
                    let pr = tcu.cpu.probe(p).unwrap();
                    println!("{t:7.3} P {p:08X} hits {} D {:X?}", pr.hits, &pr.last_regs[..16]);
                }
            }
            if !watch.is_empty() {
                let d = &tcu.cpu.hw().dspr;
                let v: Vec<String> = watch.iter().map(|&(a, n)| {
                    let o = (a & 0xF_FFFF) as usize;
                    format!("{a:X}={:X}", (0..n).fold(0u32, |acc, k| acc | (d[o + k] as u32) << (8 * k)))
                }).collect();
                println!("{t:7.3} W {}", v.join(" "));
            }
            if ((t * 10.0).round() as u64).is_multiple_of(5) && (t * 10.0 - (t * 10.0).round()).abs() < 1e-6 {
                let tel = s.plant.telemetry();
                let hw = tcu.cpu.hw();
                let pm = &hw.tc1784.pump;
                let pump = format!(
                    "st {} sync {}{} {:.0} rpm {:.1} A ({:.1} A dc) duty {:?} edges comm {} zc {}",
                    hw.dspr[0x3400],
                    hw.dspr[0x832A],
                    hw.dspr[0x832B],
                    pm.omega * 30.0 / std::f64::consts::PI,
                    pm.i_a,
                    pm.i_dc,
                    o.pump_motor_duty.map(|d| (d * 1000.0).round() / 10.0),
                    pm.comm_edges,
                    pm.zc_edges
                );
                println!(
                    "{t:5.1} s main {} gear {}/{} sel {} | eng {:.0} rpm, G182 sees engine, run-state {} | p fw {:.1} bar (ok {}) plant {:.1} bar | pump {} | safety {:08X} err {} | valves {:?} I {:?} mA",
                    l.main_state,
                    l.istgang,
                    l.zielgang,
                    l.waehlhebel,
                    tel.rpm,
                    l.engine_running,
                    l.system_pressure_bar,
                    l.pressure_ok,
                    tel.gearbox_hydraulic_bar,
                    pump,
                    l.safety_word,
                    l.safety_errors,
                    o.solenoid_duty.iter().take(8).map(|d| d.map(|x| (x * 100.0).round())).collect::<Vec<_>>(),
                    cur.map(|i| (i * 1e3).round()),
                );
            }
        }
        let wall_s = wall.elapsed().as_secs_f64();
        let l = s.tcu.as_ref().unwrap().dq200().unwrap().read_live_state();
        println!("ECU+TCU+plant: {t_end:.1} s simulated in {wall_s:.1} s wall (incl. session boot) -> RTF {:.2}", t_end / wall_s);
        println!("WFS5 TCU challenges {} answers {} authorised {}", s.immo.tcu_challenges, s.immo.tcu_answers, l.wfs5_authorized);
        println!("main state: before motoring {main_before_motor}, max {max_main}, >=3 at {state3_s:?} s; outputs released at {released_s:?} s, max coil current {max_coil_a:.2} A");
        let forks: Vec<i16> = (0..4).map(|f| {
            let d = &s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().dspr;
            i16::from_le_bytes([d[0x7E60 + 2 * f], d[0x7E61 + 2 * f]])
        }).collect();
        println!("V401: synchronised at {pump_sync_s:?} s, max pressure {max_bar:.1} bar, switched off at the upper threshold at {pump_off_s:?} s; forks {forks:?} x 0.01 mm");
        {
            let hw = s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw();
            println!("TCU resets {:?} (kind, STM ticks, insns), SWRST pcs {:X?}, traps {}", hw.reset_log, hw.swrst_log, hw.trap_count);
        }
        if std::env::var("SIMEMU_TCU_SPI").is_ok() {
            let io = &s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().tc1784;
            for (d, log) in simemu_tc1791::tc1784_io::SPI_DEVICES.iter().zip(io.spi_log.iter()) {
                println!("spi {}: {:X?}", d.name, log);
            }
            for (k, log) in io.spi_other.iter().enumerate() {
                println!("spi SSC{k} without known CS (tx, P2, P3): {:X?}", log);
            }
        }
        if std::env::var("SIMEMU_TCU_CANLOG").is_ok() {
            for e in &s.can_log {
                println!("can {:9.1} ms {} {} {}", e.t_ms, e.src, e.id, e.data);
            }
        }
        if let Ok(path) = std::env::var("SIMEMU_TCU_DUMP") {
            std::fs::write(&path, &s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().dspr[..]).unwrap();
            std::fs::write(format!("{path}.pspr"), &s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().pspr[..]).unwrap();
        }
        for &p in &probes {
            let pr = s.tcu.as_ref().unwrap().dq200().unwrap().cpu.probe(p).unwrap();
            if pr.hits > 0 || !(0x8004_0000..=0x8004_2000).contains(&p) {
                println!("probe 0x{p:08X} hits {} D {:X?} A {:X?}", pr.hits, &pr.last_regs[..16], &pr.last_regs[16..]);
            }
        }
        if std::env::var("SIMEMU_TCU_WWATCH").is_ok() {
            let mut seen = std::collections::HashSet::new();
            for &(pc, ra, a, v, n) in simemu_tc1791::cpu_hooks::WRITE_WATCH.lock().unwrap().iter() {
                if seen.insert((pc, a, v)) {
                    println!("wwatch insn {n} pc {pc:08X} a11 {ra:08X} [{a:08X}] <- {v:X}");
                }
            }
        }
        if std::env::var("SIMEMU_TCU_TRACE").is_ok() {
            let mut seen = std::collections::HashSet::new();
            let all = std::env::var("SIMEMU_TCU_TRACE_ALL").is_ok();
            for &(pc, a, v, m) in s.tcu.as_ref().unwrap().dq200().unwrap().io_trace() {
                if seen.insert((pc, a, v & m)) || all {
                    println!("trace pc {pc:08X} [{a:08X}] {} {:08X} (m {m:08X})", if m == 0 { "->" } else { "<-" }, v);
                }
            }
        }
        if std::env::var("SIMEMU_TCU_PUMP_DIAG").is_ok() {
            let hw = s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw();
            let g = &hw.tc1784.gpta;
            for n in [10usize, 12] {
                println!("GPTA0 GTC{n}: input {:?}", g.gtc_input(0, n));
            }
            for n in 20..32usize {
                println!("LTCA2 LTC{n}: CTR {:08X} out {} in {:?}", g.ltc_ctr(2, n), g.ltc_out(2, n), g.ltc_input(2, n));
            }
            let mut cells: Vec<_> = hw.tc1784.cell_out.iter().map(|(k, m)| (*k, m.edges, m.period)).collect();
            cells.sort();
            println!("cell outputs (kernel, 64+LTC/GTC, edges, period): {cells:?}");
        }
        let dtcs = s.tcu.as_mut().unwrap().dq200_mut().unwrap().read_dtcs().expect("UDS 19 02");
        for (dtc, st, sae) in &dtcs {
            println!("DTC {dtc:06X} {sae} status {st:02X}");
        }
        assert!(l.wfs5_authorized, "WFS5 0x013 not accepted");
        assert!(main_before_motor >= 2, "TCU should leave state 1 on the ECU/chassis CAN");
        assert!(max_main >= 4, "pressure-sufficient state 4 not reached (max {max_main})");
        assert!(pump_sync_s.is_some(), "V401 never synchronised (pump state 6)");
        assert!(max_bar >= 40.0, "accumulator not charged: {max_bar:.1} bar");
        assert!(t_end < 15.0 || pump_off_s.is_some(), "firmware never switched the pump off at its upper threshold");
        assert!(released_s.is_some_and(|t| t <= 0.5), "safety software did not release the outputs: {released_s:?}");
        assert!(max_coil_a > 0.5, "no valve current under firmware control ({max_coil_a:.2} A)");
        assert!(dtcs.iter().all(|d| d.2 != "?"), "DTC without SAE code: {dtcs:X?}");
        println!("P phase: 1st + R engaged in state 5 at {p_engaged_s:?} s, clutch travel max {max_clutch_mm:.2} mm, input shafts max {max_shaft_rpm:.0} rpm after 3 s");
        println!("D: state 9 at {state9_s:?} s; drive-away: rollers max {max_roller_kmh:.1} km/h, K1 slip max {max_k1_slip_rpm:.0} rpm, WBA_03 gears {gear_seq:?}");
        assert!(p_engaged_s.is_some(), "1st and R not engaged in P (main state 5)");
        assert!(max_clutch_mm < 4.45, "clutch closed in P: {max_clutch_mm:.2} mm");
        assert!(max_shaft_rpm < 50.0, "open clutches drag the input shafts: {max_shaft_rpm:.0} rpm");
        assert!(state9_s.is_some(), "main state 9 not reached in D (max {max_main})");
        assert!(max_roller_kmh > 5.0 && max_k1_slip_rpm > 200.0, "no launch with K1 slip: {max_roller_kmh:.1} km/h, slip {max_k1_slip_rpm:.0} rpm");
        let pos = |g: u8| gear_seq.iter().position(|&x| x == g);
        assert!(matches!((pos(1), pos(2), pos(3)), (Some(a), Some(b), Some(c)) if a < b && b < c), "no 1 -> 2 -> 3 upshifts: {gear_seq:?}");
        // Selector test (events 2/18) and the state-4 clutch-position monitor (events 0x74/0x75).
        let bad = [0x298A, 0x299A, 0x29FB, 0x29FC];
        assert!(dtcs.iter().all(|d| !bad.contains(&(d.0 & 0xFFFF))), "selector-test / clutch-position faults: {dtcs:X?}");
        let resets = &s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().reset_log;
        assert_eq!(resets.len(), resets0, "TCU reset during the run: {resets:?}");
    }

    /// Intel (little-endian) signal of a CAN payload: `len` bits from bit `start`.
    fn sig(d: &[u8; 8], start: u32, len: u32) -> i64 {
        ((u64::from_le_bytes(*d) >> start) & ((1u64 << len) - 1)) as i64
    }

    /// Stage 3 IS38 car (`octavia_is38`) on the 4WD road-load dyno with ECU image `SIMEMU_IMAGE` (default stock SC8O40) and the DQ250
    /// board (`SIMEMU_DQ250_IMAGE`) on the gear set of its own calibration, warm engine, gearbox at `SIMEMU_DRIVE_ATF_C`
    /// (70 °C); `None` (SKIP printed) when an image is missing.
    fn cjsa_dq250_session() -> Option<StudioSession> {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return None;
        };
        let image = std::env::var("SIMEMU_IMAGE").unwrap_or_else(|_| "8V0906264H__0005.bin".into());
        if !crate::paths::has_firmware(&root, &image) {
            eprintln!("SKIP: {image} not found (scripts/frf_to_bin.py)");
            return None;
        }
        let mut s = StudioSession::new(&root).expect("session");
        s.bench_build.active = simemu_physics::bench_config::preset("octavia_is38").expect("octavia_is38");
        // these scenarios use 0D9300012F unless `SIMEMU_DQ250_IMAGE` names another (the studio default is 4319)
        if std::env::var("SIMEMU_DQ250_IMAGE").is_err() {
            s.bench_build.active.tcu_image = Some(crate::studio::session::DQ250_012F.into());
        }
        s.plant = s.bench_build.new_plant().expect("plant");
        s.plant.soak(90.0);
        // warm gearbox (ATF sump = case) after a short stop
        s.plant.driveline.case_c = env_f("SIMEMU_DRIVE_ATF_C", 70.0);
        s.plant.driveline.mech.oil_c = s.plant.driveline.case_c;
        s.plant.bench = BenchMode::RoadLoadDyno;
        s.eot_clock_s = 300.0;
        s.load_image(&image).expect("image");
        s.power_cycle_tcu();
        if s.tcu.is_none() && s.tcu_error.as_deref().is_some_and(|e| e.contains("not found")) {
            eprintln!("SKIP: {:?}", s.tcu_error);
            return None;
        }
        assert!(s.tcu.as_ref().is_some_and(|t| t.dq250().is_some()), "DQ250 board: {:?}", s.tcu_error);
        // The gearbox hardware the TCU calibration is made for (its tooth table, `tcu_dq250::gear_set`):
        // 0D9300012F matches the build's `dq250_0d9_awd`; the 0D9300011R read is for the S3 gear set.
        let gs = s.tcu.as_ref().unwrap().dq250().unwrap().gears.expect("gear set in the TCU calibration");
        let d = s.plant.drivetrain_profile.clone().with_gear_set(gs.ratios, gs.finals, gs.reverse);
        if d.gear_ratios != s.plant.drivetrain_profile.gear_ratios || d.final_drives != s.plant.drivetrain_profile.final_drives {
            println!("plant gear set from the TCU calibration: {gs:?}");
            s.plant.drivetrain_profile = d;
        }
        // `SIMEMU_DQ250_NVM_DIR`: the TCU EEPROM is kept there (continue from a saved workshop state)
        if let Ok(d) = std::env::var("SIMEMU_DQ250_NVM_DIR") {
            s.set_nvm_dir(d.into());
        }
        println!("ECU {} | TCU {} | build {} | {}", s.image_label, s.tcu.as_ref().unwrap().sw_id(), s.bench_build.active.name, s.immo.binding);
        // Diagnostic overrides of the gear-valve hydraulics (`WetHydraulicsHw::mux_energised` / `plus_valves_0x2e`).
        if let MechatronicHw::Wet(h) = &mut s.plant.drivetrain_profile.mechatronic {
            if let Ok(v) = std::env::var("SIMEMU_DQ250_MUX") {
                // four digits, one per actuator: 1 = connected while N92 is energised
                h.mux_energised = std::array::from_fn(|f| v.as_bytes().get(f) == Some(&b'1'));
            }
            if let Ok(v) = std::env::var("SIMEMU_DQ250_PLUS_2E") {
                // four digits, one per actuator: 1 = its +0x2E valve pushes towards + travel
                h.plus_valves_0x2e = std::array::from_fn(|f| v.as_bytes().get(f) != Some(&b'0'));
            }
        }
        Some(s)
    }

    /// One tester job through the session (as `/api/uds`), stepping the co-simulation until it is answered (at most
    /// `max_s`); the status afterwards.
    fn uds_job(s: &mut StudioSession, service: &str, value: Option<u32>, rid: Option<u16>, max_s: f64) -> crate::studio::uds::UdsStatus {
        let r = crate::studio::session::UdsRequest { target: "tcu".into(), service: service.into(), data: Vec::new(), value, rid };
        s.uds_request(&r).expect("uds request");
        for _ in 0..(max_s / 0.01) as usize {
            s.step(0.01);
            if !s.uds_tcu.status.pending {
                break;
            }
        }
        for x in s.uds_tcu.status.log.iter().filter(|_| service != "routine_status") {
            println!("{:7.2}   UDS > {:<24} < {:<24} {}", s.plant.sim_time_s, x.request, x.response, x.note);
        }
        s.uds_tcu.status.clone()
    }

    /// TCU RAM bytes (observation only).
    fn tcu_ram(s: &StudioSession, a: u32, n: usize) -> Vec<u8> {
        s.tcu.as_ref().unwrap().cpu().read_mem(a, n).unwrap_or_default()
    }

    /// Ignition on (if off), engine started with the start button and idling for 5 s.
    fn start_engine(s: &mut StudioSession) {
        if !s.plant.driver.ignition_on {
            s.plant.driver.ignition_on = true;
            s.step(1.0);
        }
        s.plant.driver.start_button = true;
        let mut running = false;
        for _ in 0..500 {
            s.step(0.01);
            let tel = s.plant.telemetry();
            if tel.rpm > 600.0 && tel.starter_current_a <= 0.0 {
                running = true;
                break;
            }
        }
        s.plant.driver.start_button = false;
        assert!(running, "engine did not start");
        s.step(5.0);
    }

    /// Ignition off as in a workshop after coding / basic settings: the TCU runs its after-run (it stores what it
    /// learnt in its EEPROM) and releases the bus; `SIMEMU_DQ250_AFTERRUN_S` (default 15 s) later its supply goes
    /// (board power-down, the EEPROM kept), then ignition on and engine start.
    fn key_off_on(s: &mut StudioSession) {
        s.plant.driver.ignition_on = false;
        let w0 = s.tcu.as_ref().and_then(super::TcuBoard::nvm).map_or(0, |n| n.2);
        s.step(env_f("SIMEMU_DQ250_AFTERRUN_S", 15.0));
        let w1 = s.tcu.as_ref().and_then(super::TcuBoard::nvm).map_or(0, |n| n.2);
        println!("{:7.2} ignition off: the TCU wrote {} EEPROM bytes in its after-run", s.plant.sim_time_s, w1 - w0);
        s.power_cycle_tcu();
        start_engine(s);
    }

    /// Workshop timeline row: plant and the firmware's adaptation state (TCU RAM, observation only: 0xD00059DF step,
    /// 0xD00059D3 abort code, 0xD00059B0 violated preconditions, 0xD0004122 routine state; docs/DQ250_IO_MAP.md §8).
    fn workshop_row(s: &StudioSession, tag: &str) {
        let tel = s.plant.telemetry();
        let r = |a, n| tcu_ram(s, a, n);
        let o = &s.actuators.tcu;
        let amps: Vec<String> = (0..11).map(|k| o.solenoid_duty[k].map_or("-".into(), |_| format!("{:.2}", o.solenoid_current_a.map_or(0.0, |c| c[k])))).collect();
        println!(
            "{:7.2} {tag:6} | {:5.0} rpm | out {:.0} rpm wheels {:?} km/h | shafts {:?} | line {:.1} bar clutch {:?} bar {:?} Nm | forks {:?} mm | adapt step {:02X} abort {:02X} viol {:04X} routine {:02X} | EEPROM written {} B | coil A {}",
            s.plant.sim_time_s,
            tel.rpm,
            tel.output_shaft_rpm[1],
            tel.wheel_kmh.map(|x| (x * 10.0).round() / 10.0),
            tel.shaft_rpm.map(|x| x.round()),
            tel.gearbox_hydraulic_bar,
            tel.clutch_bar.map(|x| (x * 10.0).round() / 10.0),
            tel.clutch_torque_nm.map(|x| (x * 10.0).round() / 10.0),
            tel.fork_mm.map(|x| (x * 10.0).round() / 10.0),
            r(0xD000_59DF, 1)[0],
            r(0xD000_59D3, 1)[0],
            u16::from_le_bytes([r(0xD000_59B0, 2)[0], r(0xD000_59B0, 2)[1]]),
            r(0xD000_4122, 1)[0],
            s.tcu.as_ref().and_then(super::TcuBoard::nvm).map_or(0, |n| n.2),
            amps.join(" ")
        );
        // `SIMEMU_TCU_WATCH=addr[:n],..`: more TCU RAM (observation only)
        for w in std::env::var("SIMEMU_TCU_WATCH").unwrap_or_default().split(',').filter(|x| !x.is_empty()) {
            let (a, n) = w.split_once(':').unwrap_or((w, "2"));
            let a = u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap();
            print!("  {a:08X}={:02X?}", r(a, n.parse().unwrap()));
        }
        if std::env::var("SIMEMU_TCU_WATCH").is_ok() {
            println!();
        }
        // `SIMEMU_DQ250_TQ=1`: torque between the units — ECU `Motor_11.MO_Mom_Ist_Summe` 22|10, TCU `Getriebe_12.
        // GE_Aufnahmemoment` 48|10 (Nm, −509) — next to the plant's crank output and gearbox load (observation only)
        if std::env::var("SIMEMU_DQ250_TQ").is_ok() {
            let f = |src: &'static str, id: u32, b: u32| s.can_last.get(&(src, id)).map(|f| ((u64::from_le_bytes(f.data) >> b) & 0x3FF) as i64 - 509);
            println!(
                "        torque: ECU MO_Mom_Ist_Summe {:?} Nm, TCU GE_Aufnahmemoment {:?} Nm | plant brake {:.1} Nm, gearbox (clutches + pump) {:.1} Nm",
                f("ECU", 0x0A7, 22),
                f("TCU", 0x0AE, 48),
                tel.torque_brake_nm,
                tel.clutch_torque_nm.iter().sum::<f64>() + s.plant.driveline.mech.pump_torque_nm
            );
        }
    }

    /// The workshop part of [`dq250_workshop_scenario`] (engine idling in P with the brake): coding `coding` (`2E F198` +
    /// `2E 0600`), ignition cycle, then each basic setting of `basic` polled with `22 0102` until it leaves 0xC0 and ended
    /// with `31 02`. The tester is bench equipment; the firmware decides and stores everything. Returns (routine, status).
    fn workshop(s: &mut StudioSession, coding: u32, basic: &[u16]) -> Vec<(u16, Option<u8>)> {
        let st = uds_job(s, "write_coding", Some(coding), None, 3.0);
        assert_eq!(st.coding, Some(coding), "coding not accepted: {}", st.note);
        key_off_on(s);
        let st = uds_job(s, "read_coding", None, None, 2.0);
        assert_eq!(st.coding, Some(coding), "coding not kept over the ignition cycle");
        let max_s = env_f("SIMEMU_DQ250_BASIC_S", 120.0);
        let mut results = Vec::new();
        for &rid in basic {
            // A basic setting the firmware aborted (status 0x60, e.g. 0D9300011R/4319 0x0380 abort 0x0E when idle torque
            // oscillates beyond ±3.0 Nm right after gear engagement, docs/DQ250_FIRMWARE_SPEC.md §11.6) is ended and
            // started again up to 3 attempts, as a technician does (`dq250_prep::BASIC_ATTEMPTS`); the firmware decides.
            let mut attempt = 1usize;
            let status = loop {
                let st = uds_job(s, "basic_setting", None, Some(rid), 3.0);
                workshop_row(s, "start");
                let mut t = 0.0;
                let mut status = st.routine_status;
                while status == Some(0xC0) && t < max_s {
                    for _ in 0..5 {
                        s.step(0.1);
                        t += 0.1;
                        workshop_row(s, "basic");
                    }
                    status = uds_job(s, "routine_status", None, None, 1.0).routine_status;
                }
                println!(
                    "basic setting 0x{rid:04X} (attempt {attempt}): status {:02X?} ({}) after {t:.1} s",
                    status,
                    status.map_or("?", crate::studio::uds::routine_status_text)
                );
                if status != Some(0x60) || attempt == 3 {
                    break status;
                }
                uds_job(s, "routine_stop", None, Some(rid), 2.0);
                attempt += 1;
                s.step(2.0);
            };
            results.push((rid, status));
            // `SIMEMU_TCU_PEEK=addr:n,..`: TCU RAM as int16 after each basic setting (observation only)
            for w in std::env::var("SIMEMU_TCU_PEEK").unwrap_or_default().split(',').filter(|x| !x.is_empty()) {
                let (a, n) = w.split_once(':').unwrap_or((w, "2"));
                let a = u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap();
                let m = tcu_ram(s, a, n.parse().unwrap());
                println!("  peek {a:08X}: {:?}", m.chunks(2).map(|c| i16::from_le_bytes([c[0], *c.get(1).unwrap_or(&0)])).collect::<Vec<_>>());
            }
            // A technician reads "completed" and ends the routine a few seconds later. The firmware may still be
            // moving a fork back to neutral when it reports 0x10; stopping at once can leave the fork mid-travel,
            // and the next basic setting then fails (0x04D4 status 0x60 with a fork parked at -5.5 mm).
            s.step(5.0);
            // the tester ends the routine (the firmware accepts the next one only then: NRC 0x24 otherwise)
            uds_job(s, "routine_stop", None, Some(rid), 2.0);
        }
        results
    }

    /// Workshop on the DQ250 as a VW tester does it, through the session's UDS tester only (bench equipment), the
    /// firmware deciding everything. No EEPROM dump exists: the erased part, the firmware writes its records itself.
    /// Engine started and idling in P, brake held; coding `SIMEMU_DQ250_CODING` (default 24 = "000024")
    /// through `write_coding`; ignition cycle ([`key_off_on`]: the TCU's after-run stores its EEPROM);
    /// basic settings `SIMEMU_DQ250_BASIC` (default `0380,04D4`) one after the other, each polled with `22 0102` until it
    /// leaves 0xC0 (running) and ended with `31 02`, with a timeline of the plant and of the firmware's adaptation state
    /// (TCU RAM, observation only: 0xD00059DF step, 0xD00059D3 abort code, 0xD00059B0 violated preconditions;
    /// docs/DQ250_IO_MAP.md §8); ignition cycle; a short drive (D with brake, creep, 30 % pedal) and the DTCs.
    ///
    /// Asserted (VERIFIED-T, native): coding accepted and kept over both ignition cycles; with the default sequence both
    /// basic settings completed by the firmware (status 0x10) and the learnt gear positions still valid after the
    /// ignition cycle (0xD00059B0 bits 5..12 except 9 clear); no TCU reset / trap. The short drive is printed, not
    /// asserted (VERIFIED-T after the sixth pass: D on the brake at 600–700 rpm without a stall, creep, 1 → 2 → 3 at 30 %;
    /// the whole drive is [`dq250_full_drive_scenario`]). `SIMEMU_DQ250_NVM_DIR` keeps the EEPROM in a directory,
    /// `SIMEMU_DQ250_HOLD=N:20,D:20` holds selector positions, `SIMEMU_TCU_WATCH` / `SIMEMU_TCU_PEEK` print TCU RAM.
    #[test]
    #[ignore = "slow: ECU + DQ250 + plant co-simulation"]
    fn dq250_workshop_scenario() {
        let Some(mut s) = cjsa_dq250_session() else { return };
        let coding = env_f("SIMEMU_DQ250_CODING", 24.0) as u32;
        let basic: Vec<u16> = std::env::var("SIMEMU_DQ250_BASIC").unwrap_or_else(|_| "0380,04D4".into()).split(',').filter(|x| !x.is_empty()).map(|x| u16::from_str_radix(x, 16).unwrap()).collect();
        let row = workshop_row;
        // ignition, start, idle in P with the brake held
        let d = &mut s.plant.driver;
        (d.selector, d.brake_pct) = (Selector::P, 60.0);
        start_engine(&mut s);
        // `SIMEMU_DQ250_IDLE_TRACE=n`: n rows every 10 ms at idle
        for _ in 0..env_f("SIMEMU_DQ250_IDLE_TRACE", 0.0) as usize {
            s.step(0.01);
            let m = &s.plant.driveline;
            println!("        carrier {:+.3} rad/s pawl {:+.4} rad {:+.0} Nm | shafts {:+.2} {:+.2} rad/s | clutch T {:+.2} {:+.2} Nm | halfshaft {:+.4} rad", m.carrier_omega, m.pawl_twist, m.pawl_torque_nm, m.shaft_omega[0], m.shaft_omega[1], m.clutch_torque_nm[0], m.clutch_torque_nm[1], m.halfshaft_twist);
        }
        row(&s, "idle");
        let results = workshop(&mut s, coding, &basic);
        // `SIMEMU_DQ250_HOLD=sel:secs,..`: selector positions held at idle with the brake (e.g. N:20,D:20)
        for h in std::env::var("SIMEMU_DQ250_HOLD").unwrap_or_default().split(',').filter(|x| !x.is_empty()) {
            let (sel, secs) = h.split_once(':').unwrap();
            s.plant.driver.selector = match sel { "N" => Selector::N, "D" => Selector::D, "R" => Selector::R, _ => Selector::P };
            for _ in 0..(secs.parse::<f64>().unwrap() * 2.0) as usize {
                s.step(0.5);
                row(&s, sel);
            }
        }
        s.plant.driver.selector = Selector::P;
        key_off_on(&mut s);
        let coding_after = uds_job(&mut s, "read_coding", None, None, 2.0).coding;
        println!("coding after the ignition cycle: {coding_after:?}");
        row(&s, "after");
        // learnt gear positions survive the ignition cycle: 0xD00059B0 bits 5..12 ("gear position not adapted", bit 9 =
        // the N side of 5/N, without a gear) clear (observation only)
        let viol = u16::from_le_bytes(tcu_ram(&s, 0xD000_59B0, 2).try_into().unwrap());
        // short drive: D with the brake, creep, 30 % pedal (`SIMEMU_DQ250_DRIVE_S`, default 12 s)
        let (mut gears, mut creep_kmh, mut max_kmh) = (Vec::<i64>::new(), 0f64, 0f64);
        s.plant.driver.selector = Selector::D;
        let drive_s = env_f("SIMEMU_DQ250_DRIVE_S", 12.0);
        let mut t = 0.0;
        while t < drive_s {
            let d = &mut s.plant.driver;
            (d.brake_pct, d.pedal_pct) = if t < 2.0 { (60.0, 0.0) } else if t < 6.0 { (0.0, 0.0) } else { (0.0, 30.0) };
            s.step(0.01);
            t += 0.01;
            let v = s.plant.telemetry().roller_kmh;
            if (2.0..6.0).contains(&t) {
                creep_kmh = creep_kmh.max(v);
            }
            max_kmh = max_kmh.max(v);
            let g = sig(&s.can_last.get(&("TCU", 0x394)).map(|f| f.data).unwrap_or([0; 8]), 24, 4);
            if gears.last() != Some(&g) {
                gears.push(g);
            }
            if ((t * 100.0).round() as u64).is_multiple_of(100) {
                row(&s, "drive");
            }
        }
        s.plant.driver = simemu_physics::DriverInputs { ignition_on: true, selector: Selector::P, brake_pct: 60.0, ..s.plant.driver.clone() };
        s.step(3.0);
        let st = uds_job(&mut s, "read_dtc", None, None, 3.0);
        println!("TCU DTCs: {:?} = {:?}", st.dtcs, st.dtc_sae);
        let hw = s.tcu.as_ref().unwrap().cpu().hw();
        println!("drive: TCU gears on WBA_03 {gears:?}, creep max {creep_kmh:.1} km/h, max {max_kmh:.1} km/h | TCU resets {:?} traps {}", hw.reset_log, hw.trap_count);
        assert!(hw.reset_log.is_empty() && hw.trap_count == 0 && s.tcu_error.is_none(), "TCU reset/trap/error: {:?} {:?}", hw.reset_log, s.tcu_error);
        assert_eq!(coding_after, Some(coding), "coding lost over the ignition cycle");
        if std::env::var("SIMEMU_DQ250_BASIC").is_err() {
            // default workshop sequence: both basic settings completed by the firmware (0x10), results kept
            assert!(results.iter().all(|r| r.1 == Some(0x10)), "basic settings not completed: {results:02X?}");
            assert_eq!(viol & 0x1DE0, 0, "learnt gear positions lost over the ignition cycle (0xD00059B0 = {viol:04X})");
        }
    }

    /// DQ250 in the loop: the Stage 3 IS38 car (bench build `octavia_is38`: CJSA + IS38 + DQ250 0D9 + Haldex 5) on the 4WD road-load
    /// chassis dyno, ECU image `SIMEMU_IMAGE` (default stock SC8O40 `images/8V0906264H__0005.bin`), TCU image
    /// `SIMEMU_DQ250_IMAGE` (default 0D9300012F, see `StudioSession::boot_dq250`) on the gear set of its own
    /// calibration (`tcu_dq250::gear_set`), lab immobiliser binding for both. Driver inputs only: ignition, start, idle in
    /// P, D with the brake held (`SIMEMU_DRIVE_D_S`, 10 s), brake released (creep), pedal `SIMEMU_DRIVE_PEDAL1` (30 %) to
    /// `SIMEMU_DRIVE_V1` (50 km/h), `SIMEMU_DRIVE_PEDAL2` (70 %) to `SIMEMU_DRIVE_VMAX` (100 km/h), lift off, brake, P —
    /// within `SIMEMU_DRIVE_S` (30 s). Prints a timeline (TCU CAN: `WBA_03.WBA_eing_Gang_02`, `Getriebe_11` `GE_Fahrstufe`
    /// / `GE_Zielgang` / torque request; plant: line / clutch pressures, shafts, actuator travels, valve coil currents) and
    /// the TCU DTCs by the image's own table.
    ///
    /// Asserted (firmware decisions, VERIFIED-T on 0D9300011R and 0D9300012F, native core — on Unicorn the ASW halts once
    /// the speed sensors run, MILESTONE §2.3; docs/DQ250_IO_MAP.md §7): WFS5 accepted, no
    /// TCU reset / trap; in P the firmware preselects R and 2nd and the mechatronic completes it; in D it takes R out,
    /// engages 1st (reported as gear 1 on `WBA_03`), creeps on K1 above 3 km/h within 4 s and, under pedal, shifts up (a gear above
    /// 1st on `WBA_03`): the firmware closes K1 (its mode 9) and hands over to K2 once it sees the car rolling forwards
    /// (output-speed direction from the port P0 levels, docs/DQ250_FIRMWARE_SPEC.md §0).
    /// Diagnostics: `SIMEMU_DRIVE_TRACE=t0,t1` (rows every 10 ms), `SIMEMU_TCU_WATCH=addr[:n],..` (TCU RAM),
    /// `SIMEMU_TCU_PROBES=addr,..` (code probes), `SIMEMU_DQ250_MUX` / `SIMEMU_DQ250_PLUS_2E` (gear-valve hydraulics, 4 digits).
    /// Run: `SIMEMU_CPU=native cargo test --release -p simemu-cli dq250_drive_scenario -- --ignored --nocapture`.
    #[test]
    #[ignore = "slow: ECU + DQ250 + plant co-simulation"]
    fn dq250_drive_scenario() {
        let Some(mut s) = cjsa_dq250_session() else { return };
        // `SIMEMU_TCU_WATCH=addr[:n],..`: TCU RAM bytes (hex) printed with every row (observation only).
        let hex = |s: &str| u32::from_str_radix(s.trim().trim_start_matches("0x"), 16).unwrap();
        // `SIMEMU_TCU_PROBES=addr,..`: code probes (observation only), printed as hits / d4 / a11 with every row.
        let probes: Vec<u32> = std::env::var("SIMEMU_TCU_PROBES").unwrap_or_default().split(',').filter(|x| !x.is_empty()).map(hex).collect();
        for &a in &probes {
            s.tcu.as_mut().unwrap().cpu_mut().add_probe(a).unwrap();
        }
        let watch: Vec<(u32, usize)> = std::env::var("SIMEMU_TCU_WATCH").unwrap_or_default().split(',').filter(|x| !x.is_empty())
            .map(|a| a.split_once(':').map_or_else(|| (hex(a), 1), |(a, n)| (hex(a), n.parse().unwrap()))).collect();
        // `SIMEMU_DRIVE_TRACE=t0,t1`: a row every 10 ms in that window.
        let tr = std::env::var("SIMEMU_DRIVE_TRACE").ok().and_then(|v| v.split_once(',').map(|(a, b)| (a.parse().unwrap_or(0.0), b.parse().unwrap_or(0.0)))).unwrap_or((-1.0, -1.0));
        let (t_d, v1, vmax, t_max) = (env_f("SIMEMU_DRIVE_D_S", 10.0), env_f("SIMEMU_DRIVE_V1", 50.0), env_f("SIMEMU_DRIVE_VMAX", 100.0), env_f("SIMEMU_DRIVE_S", 30.0));
        let (p1, p2) = (env_f("SIMEMU_DRIVE_PEDAL1", 30.0), env_f("SIMEMU_DRIVE_PEDAL2", 70.0));
        let (mut t, dt) = (0.0, 0.01);
        let (mut phase, mut phase_t) = ("ignition", 0.0);
        let (mut running_s, mut max_kmh, mut gears) = (None::<f64>, 0f64, Vec::<i64>::new());
        // firmware gear requests completed by the mechatronic (travel past its ±6.50 mm engagement thresholds)
        let (mut p_r_2nd, mut d_first, mut creep_kmh, mut top_gear) = (None::<f64>, None::<f64>, 0f64, 0i64);
        let mut max_clutch = [(0f64, 0f64); 2];
        while t < t_max - 1e-9 && phase != "done" {
            let tel = s.plant.telemetry();
            let v = tel.roller_kmh;
            let np = match phase {
                "ignition" if t >= 1.0 => "start",
                "start" if running_s.is_some() || t >= 6.0 => "idle",
                "idle" if t >= t_d => "hold_d",
                "hold_d" if t >= t_d + 2.0 => "creep",
                "creep" if t >= t_d + 6.0 => "pedal1",
                "pedal1" if v >= v1 => "pedal2",
                "pedal2" if v >= vmax => "coast",
                "coast" if t - phase_t >= 5.0 => "brake",
                "brake" if v < 0.5 && t - phase_t > 1.0 => "park",
                "park" if t - phase_t >= 3.0 => "done",
                p => p,
            };
            if np != phase {
                (phase, phase_t) = (np, t);
                println!("{t:7.2} --- {phase}");
            }
            let d = &mut s.plant.driver;
            d.ignition_on = true;
            d.start_button = phase == "start";
            d.selector = if ["hold_d", "creep", "pedal1", "pedal2", "coast", "brake"].contains(&phase) { Selector::D } else { Selector::P };
            d.brake_pct = match phase {
                "ignition" | "start" | "idle" | "hold_d" => 60.0,
                "brake" | "park" | "done" => 40.0,
                _ => 0.0,
            };
            d.pedal_pct = match phase {
                "pedal1" => p1,
                "pedal2" => p2,
                _ => 0.0,
            };
            s.step(dt);
            t += dt;
            let tel = s.plant.telemetry();
            if running_s.is_none() && tel.rpm > 600.0 && tel.starter_current_a <= 0.0 {
                running_s = Some(t);
            }
            max_kmh = max_kmh.max(tel.roller_kmh);
            for (k, mc) in max_clutch.iter_mut().enumerate() {
                if tel.clutch_bar[k] > mc.0 {
                    *mc = (tel.clutch_bar[k], t);
                }
            }
            // `SIMEMU_DQ250_DUMP_AT=t,..`: TCU LDRAM at these times into /tmp/dq250_ram_<t>.bin (observation only)
            for at in std::env::var("SIMEMU_DQ250_DUMP_AT").unwrap_or_default().split(',').filter_map(|x| x.parse::<f64>().ok()) {
                if (t - at).abs() < dt / 2.0 {
                    let ram = s.tcu.as_ref().unwrap().cpu().read_mem(0xD000_0000, 0xE000).unwrap();
                    std::fs::write(format!("/tmp/dq250_ram_{at}.bin"), ram).unwrap();
                }
            }
            let fr = |id: u32| s.can_last.get(&("TCU", id)).map(|f| f.data).unwrap_or([0; 8]);
            let (g11, wba) = (fr(0x0AD), fr(0x394));
            let m11 = s.can_last.get(&("ECU", 0x0A7)).map(|f| f.data).unwrap_or([0; 8]);
            let gear = sig(&wba, 24, 4);
            let f = tel.fork_mm;
            if phase == "idle" && f[3] <= -6.5 && f[1] <= -6.5 {
                p_r_2nd.get_or_insert(t); // P: R (6/R actuator −) and 2nd (4/2 actuator −)
            }
            if ["hold_d", "creep"].contains(&phase) && f[0] >= 6.5 && f[3] > -6.5 && gear == 1 {
                d_first.get_or_insert(t); // D: R out, 1st in, reported on WBA_03
            }
            if phase == "creep" {
                creep_kmh = creep_kmh.max(tel.roller_kmh);
            }
            if d_first.is_some() && (1..=6).contains(&gear) {
                top_gear = top_gear.max(gear); // highest gear the TCU reports after taking 1st in D
            }
            if gears.last() != Some(&gear) {
                gears.push(gear);
            }
            if ((t * 100.0).round() as u64).is_multiple_of(50) || (tr.0..tr.1).contains(&t) {
                let o = &s.actuators.tcu;
                let duty: Vec<String> = (0..11).map(|k| o.solenoid_duty[k].map_or("-".into(), |_| format!("{:.2}", o.solenoid_current_a.map_or(0.0, |c| c[k])))).collect();
                println!(
                    "{t:7.2} {phase:7} | {:5.0} rpm {:5.1} km/h | plant gear {} shafts {:?} | TCU gear {gear} Fahrstufe {} Zielgang {} | torque: TCU request {} Nm st {}/{}, ECU actual {} Nm | line {:.1} bar clutch {:?} bar | forks {:?} mm | coil A {}",
                    tel.rpm,
                    tel.roller_kmh,
                    tel.driving_gear,
                    tel.shaft_rpm.map(|x| x.round()),
                    sig(&g11, 42, 4),
                    sig(&g11, 60, 4),
                    sig(&g11, 12, 10) - 509,
                    sig(&g11, 56, 2),
                    sig(&g11, 50, 3),
                    sig(&m11, 22, 10) - 509,
                    tel.gearbox_hydraulic_bar,
                    tel.clutch_bar.map(|x| (x * 10.0).round() / 10.0),
                    tel.fork_mm.map(|x| (x * 10.0).round() / 10.0),
                    duty.join(" ")
                );
                for &a in &probes {
                    if let Some(p) = s.tcu.as_ref().unwrap().cpu().probe(a) {
                        println!("        probe {a:08X} hits {} d4 {:X} a11 {:08X}", p.hits, p.last_regs[4], p.last_regs[16 + 11]);
                    }
                }
                if !watch.is_empty() {
                    let c = s.tcu.as_ref().unwrap().cpu();
                    let w: Vec<String> = watch.iter().map(|&(a, n)| format!("{a:08X}:{:02X?}", c.read_mem(a, n).unwrap_or_default())).collect();
                    println!("        watch {}", w.join(" "));
                }
            }
        }
        // TCU fault memory over the session's tester (19 02 FF), decoded with the image's table.
        s.uds_request(&crate::studio::session::UdsRequest { target: "tcu".into(), service: "read_dtc".into(), data: Vec::new(), value: None, rid: None }).expect("uds");
        for _ in 0..200 {
            s.step(0.01);
            if !s.uds_tcu.status.pending {
                break;
            }
        }
        let st = &s.uds_tcu.status;
        println!("TCU DTCs: {:?} = {:?} ({})", st.dtcs, st.dtc_sae, st.note);
        println!("max clutch pressure (bar, at s): K1 {:?} K2 {:?}", max_clutch[0], max_clutch[1]);
        let hw = s.tcu.as_ref().unwrap().cpu().hw();
        println!(
            "engine running at {running_s:?} s, max {max_kmh:.1} km/h, TCU gears {gears:?} | TCU resets {:?} traps {} | WFS5 TCU challenges {} | TCU error {:?}",
            hw.reset_log, hw.trap_count, s.immo.tcu_challenges, s.tcu_error
        );
        assert!(s.tcu_error.is_none() && hw.trap_count == 0, "TCU board error {:?}", s.tcu_error);
        assert!(running_s.is_some(), "engine did not start");
        assert!((1..=3).contains(&s.immo.tcu_challenges), "WFS5: TCU challenges {} (accepted = 1..3)", s.immo.tcu_challenges);
        println!("P: R + 2nd engaged at {p_r_2nd:?} s | D: 1st engaged (WBA_03 gear 1) at {d_first:?} s | creep max {creep_kmh:.1} km/h | highest TCU gear after D {top_gear} (upshifts: {})", if top_gear >= 2 { "yes" } else { "not reached" });
        assert!(hw.reset_log.is_empty(), "TCU reset: {:?}", hw.reset_log);
        assert!(p_r_2nd.is_some(), "P: the firmware's R + 2nd preselection not completed");
        assert!(d_first.is_some(), "D: 1st not engaged / not reported");
        assert!(creep_kmh > 3.0, "no creep on K1: {creep_kmh:.1} km/h"); // clutch at the firmware characteristic: ≈ 0.3 m/s²
        assert!(top_gear >= 2, "no upshift: highest TCU gear after D {top_gear}");
    }

    /// A whole drive on the DQ250 the way a driver does it, on an EEPROM the firmware prepared itself: unless
    /// `SIMEMU_DQ250_NVM_DIR` already holds this image's EEPROM (it is saved there otherwise), the workshop first codes the
    /// TCU (`SIMEMU_DQ250_CODING`, default 000024) and runs the basic settings 0x0380 / 0x04D4 ([`workshop`]), then an
    /// ignition cycle lets the after-run store them. Without
    /// them the firmware limits the target gear to 2nd (0x8008CF3C: NVM blocks 9 / 0xB not valid at power-up →
    /// 0xD0003380 ^ 0xD00059CE ≠ 0xFF0 → max 2, event 0x163; VERIFIED-S, docs/DQ250_FIRMWARE_SPEC.md §5.4).
    /// Driver: P → D with the brake → creep → 30 % to 30 km/h → 50 % to 60 km/h → cruise ≈ 55 km/h with a light pedal →
    /// kickdown → light pedal (up to top gear) → coast 15 s (800–1800 rpm in top gear: where the firmware's in-drive
    /// kiss-point adaptation may run, docs/DQ250_FIRMWARE_SPEC.md §4) → brake to a stop → creep → R (the road-load dyno's
    /// rollers do not turn backwards: R creep torque against them) → P.
    ///
    /// Asserted: no TCU reset / trap / board error, no engine stall caused by the gearbox (rpm < 300 with the clutches
    /// carrying > 15 Nm; idle dips with open or nearly open clutches are printed), no engine that stopped by itself (it is
    /// restarted by the driver in P and the drive goes on, then the test fails), no fuelled cylinder leaner than λ 1.7 in
    /// the 0.5 s after the ECU's DI → MPI switches (SC8O40 images), top gear `SIMEMU_DQ250_MIN_TOP` (default 6)
    /// reported on `WBA_03`, braking from top gear downshifts to 2nd or 1st, creep after the stop, R engaged with K1 above
    /// its zero-torque pressure, no gearbox (P, without the module self-tests P06xx of the erased EEPROM) fault stored during
    /// the drive (pending / confirmed, read with `19 02` before and after; status 0x02 alone is the firmware's undebounced first
    /// report), the engine below 6500 rpm under kickdown
    /// (no clutch slip to the ECU's limiter), on 0D9300012F the line-pressure adaptation of the clutch free while cruising
    /// learnt (0xD0003C34/36 > 0, docs/DQ250_FIRMWARE_SPEC.md §4). Printed (0D9300012F RAM layout): the timeline with the TCU mode (0xD0005E13), the kiss-point adaptations
    /// (0xD0003C85 / 0xD0003C62 / 0xD0003C34; 0xD00041D3 / 0xD00059E8, docs/DQ250_FIRMWARE_SPEC.md §3–4) and the DTCs.
    #[test]
    #[ignore = "slow: ECU + DQ250 + workshop + plant co-simulation"]
    fn dq250_full_drive_scenario() {
        let Some(mut s) = cjsa_dq250_session() else { return };
        let min_top = env_f("SIMEMU_DQ250_MIN_TOP", 6.0) as i64;
        // `SIMEMU_DQ250_NVM_DIR` holding this image's EEPROM (e.g. from an earlier run of this scenario): no workshop
        let id = s.tcu.as_ref().and_then(super::TcuBoard::nvm).map(|n| n.0.to_string()).unwrap_or_default();
        let safe: String = id.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '-' }).collect();
        let prepared = s.tcu_nvm.dir.as_ref().is_some_and(|d| d.join(format!("tcu_{safe}.eeprom.bin")).exists());
        // `SIMEMU_DRIVE_TRACE=t0,t1[;t0,t1..]` (session time): a row every 10 ms with the engine and the TCU torque request
        let tr: Vec<(f64, f64)> = std::env::var("SIMEMU_DRIVE_TRACE").unwrap_or_default().split(';').filter_map(|v| v.split_once(',').map(|(a, b)| (a.parse().unwrap_or(0.0), b.parse().unwrap_or(0.0)))).collect();
        // `SIMEMU_TCU_WATCH=addr[:n],..`: TCU RAM bytes printed in the trace rows (observation only)
        let hex = |s: &str| u32::from_str_radix(s.trim().trim_start_matches("0x"), 16).unwrap();
        let watch: Vec<(u32, usize)> = std::env::var("SIMEMU_TCU_WATCH").unwrap_or_default().split(',').filter(|x| !x.is_empty())
            .map(|a| a.split_once(':').map_or_else(|| (hex(a), 1), |(a, n)| (hex(a), n.parse().unwrap()))).collect();
        let d = &mut s.plant.driver;
        (d.selector, d.brake_pct) = (Selector::P, 60.0);
        start_engine(&mut s);
        if !prepared {
            let res = workshop(&mut s, env_f("SIMEMU_DQ250_CODING", 24.0) as u32, &[0x0380, 0x04D4]);
            assert!(res.iter().all(|r| r.1 == Some(0x10)), "basic settings not completed: {res:02X?}");
            key_off_on(&mut s);
        }
        // stored TCU faults before the drive (workshop history of the erased mechatronic: P287B, P062F, P0606, P0607 ...)
        let stored_before: Vec<String> = tcu_dtcs(&mut s).into_iter().filter(|d| d.2 & 0x0C != 0).map(|d| d.0).collect();
        // `SIMEMU_TCU_PROBES=addr,..`: code probes on the TCU of the drive (observation only), a row (d4, plant, the
        // watch) whenever one is hit; `SIMEMU_TCU_COUNT=addr,..`: only counted (task rates), totals printed at the end
        let probes: Vec<u32> = std::env::var("SIMEMU_TCU_PROBES").unwrap_or_default().split(',').filter(|x| !x.is_empty()).map(hex).collect();
        let counts: Vec<u32> = std::env::var("SIMEMU_TCU_COUNT").unwrap_or_default().split(',').filter(|x| !x.is_empty()).map(hex).collect();
        for &a in probes.iter().chain(&counts) {
            s.tcu.as_mut().unwrap().cpu_mut().add_probe(a).unwrap();
        }
        let mut probe_hits = vec![0u64; probes.len()];
        // TCU RAM observations use the 0D9300012F layout (docs/DQ250_FIRMWARE_SPEC.md); other images print zeros
        let is_4930 = s.tcu.as_ref().unwrap().sw_id().contains("0D9300012F");
        let ram8 = |s: &StudioSession, a: u32| if is_4930 { tcu_ram(s, a, 1)[0] } else { 0 };
        let ram16 = |s: &StudioSession, a: u32, n: usize| -> Vec<i16> {
            if !is_4930 {
                return vec![0; n];
            }
            tcu_ram(s, a, 2 * n).chunks(2).map(|c| i16::from_le_bytes([c[0], c[1]])).collect()
        };
        let (t0, dt) = (s.plant.sim_time_s, 0.01);
        let (mut phase, mut pt) = ("p_idle", t0);
        let mut gears: Vec<(f64, i64)> = Vec::new();
        let (mut top_gear, mut top_at, mut gear_before_kd, mut kd_min_gear) = (0i64, None::<f64>, 0i64, 9i64);
        let (mut brake_min_gear, mut creep2_kmh, mut rev_k1_bar, mut rev_gear) = (9i64, 0f64, 0f64, false);
        let mut kd_max_rpm = 0f64;
        // engine dips below 300 rpm: (time, phase, clutch torque K1 + K2) — `gearbox_stall` when the clutches load the engine
        let (mut min_rpm, mut dips, mut in_dip, mut gearbox_stall) = (f64::MAX, Vec::<(f64, &str, f64)>::new(), false, None::<f64>);
        let mut modes = Vec::<(f64, u8)>::new();
        // engine stopped by itself (engine/ECU side, see `dips`): the driver restarts it in P and goes on with the phase
        let (mut restarts, mut restarting) = (Vec::<(f64, &str)>::new(), false);
        // the ECU's DI → MPI switches (SC8O40 `IDX_CMB_MOD_CUR` 0xD000C46B, 0 = OPP_1 → 8 = OPP_MPI_1; observation only):
        // (time, phase, leanest fuelled cylinder in the next 0.5 s)
        let o40 = s.image_label.contains("SC8O40");
        let (mut cmb_mode, mut to_mpi) = (None::<u8>, Vec::<(f64, &str, f64)>::new());
        loop {
            let t = s.plant.sim_time_s;
            let tel = s.plant.telemetry();
            if restarting {
                let d = &mut s.plant.driver;
                (d.selector, d.brake_pct, d.pedal_pct, d.start_button) = (Selector::P, 60.0, 0.0, true);
                if tel.rpm > 600.0 && tel.starter_current_a <= 0.0 {
                    (restarting, d.start_button, pt) = (false, false, t);
                    println!("{t:7.2} --- engine restarted, back to {phase}");
                }
                s.step(dt);
                continue;
            }
            if tel.rpm < 50.0 && t - t0 > 1.0 && phase != "done" {
                restarting = true;
                restarts.push((t, phase));
                println!("{t:7.2} !!! engine stopped in {phase}: restart");
                continue;
            }
            let (v, el) = (tel.roller_kmh, t - pt);
            let np = match phase {
                "p_idle" if el >= 3.0 => "d_hold",
                "d_hold" if el >= 3.0 => "creep",
                "creep" if el >= 5.0 => "p30",
                "p30" if v >= 30.0 || el >= 15.0 => "p50",
                "p50" if v >= 60.0 || el >= 15.0 => "cruise",
                "cruise" if el >= 25.0 => "kickdown",
                "kickdown" if v >= 110.0 || el >= 8.0 => "lift",
                "lift" if el >= 10.0 => "coast",
                "coast" if el >= 15.0 => "brake",
                "brake" if v < 0.3 && el > 1.0 => "stop",
                "stop" if el >= 2.0 => "creep2",
                "creep2" if el >= 5.0 => "stop2",
                "stop2" if v.abs() < 0.3 && el >= 2.0 => "r_hold",
                "r_hold" if el >= 3.0 => "reverse",
                "reverse" if el >= 5.0 => "park",
                "park" if el >= 3.0 => "done",
                p => p,
            };
            if np != phase {
                (phase, pt) = (np, t);
                println!("{t:7.2} --- {phase}");
            }
            if phase == "done" || t - t0 > 200.0 {
                break;
            }
            let d = &mut s.plant.driver;
            d.selector = match phase {
                "p_idle" | "park" => Selector::P,
                "r_hold" | "reverse" => Selector::R,
                _ => Selector::D,
            };
            d.brake_pct = match phase {
                "p_idle" | "d_hold" | "stop" | "stop2" | "r_hold" | "park" => 60.0,
                "brake" => 30.0,
                _ => 0.0,
            };
            // the driver holds ≈ 50 km/h with a light foot while cruising (4th at ≈ 1700 rpm)
            d.pedal_pct = match phase {
                "p30" => 30.0,
                "p50" => 50.0,
                "cruise" => (12.0 + 3.0 * (50.0 - v)).clamp(0.0, 35.0),
                "kickdown" => 100.0,
                "lift" => 15.0,
                _ => 0.0,
            };
            s.step(dt);
            let tel = s.plant.telemetry();
            for (k, &a) in probes.iter().enumerate() {
                let Some(p) = s.tcu.as_ref().unwrap().cpu().probe(a) else { continue };
                if p.hits != probe_hits[k] {
                    let w: Vec<String> = watch.iter().map(|&(a, n)| format!("{a:08X}:{:02X?}", tcu_ram(&s, a, n))).collect();
                    println!(
                        "        {t:7.2} probe {a:08X} +{} d4 {:X} | {phase} {:5.0} rpm shafts {:?} | clutch {:?} bar line {:.1} | forks {:?} v {:?} contact {:?} | {}",
                        p.hits - probe_hits[k],
                        p.last_regs[4],
                        tel.rpm,
                        tel.shaft_rpm.map(|x| x.round()),
                        tel.clutch_bar.map(|x| (x * 10.0).round() / 10.0),
                        tel.gearbox_hydraulic_bar,
                        tel.fork_mm.map(|x| (x * 100.0).round() / 100.0),
                        s.plant.driveline.mech.fork_v.map(|x| (x * 1e3).round() / 1e3),
                        s.plant.driveline.mech.fork_contact_n.map(|x| x.round()),
                        w.join(" ")
                    );
                    probe_hits[k] = p.hits;
                }
            }
            let wba = s.can_last.get(&("TCU", 0x394)).map(|f| f.data).unwrap_or([0; 8]);
            let gear = sig(&wba, 24, 4);
            if gears.last().map(|g| g.1) != Some(gear) {
                gears.push((t, gear));
            }
            let mode = ram8(&s, 0xD000_5E13);
            if modes.last().map(|m| m.1) != Some(mode) {
                modes.push((t, mode));
            }
            if o40 {
                let m = s.board.read_mem(0xD000_C46B, 1).unwrap_or_default().first().copied();
                if cmb_mode == Some(0) && m == Some(8) {
                    to_mpi.push((t, phase, 0.0));
                }
                cmb_mode = m;
                if let Some(x) = to_mpi.last_mut().filter(|x| t - x.0 < 0.5) {
                    x.2 = x.2.max(lean_peak(&tel));
                }
            }
            if (1..=6).contains(&gear) && top_gear < gear {
                (top_gear, top_at) = (gear, Some(t));
            }
            match phase {
                "cruise" => gear_before_kd = gear,
                "kickdown" => {
                    kd_max_rpm = kd_max_rpm.max(tel.rpm);
                    if (1..=6).contains(&gear) {
                        kd_min_gear = kd_min_gear.min(gear);
                    }
                }
                "brake" | "stop" if (1..=6).contains(&gear) => brake_min_gear = brake_min_gear.min(gear),
                "creep2" => creep2_kmh = creep2_kmh.max(tel.roller_kmh),
                // the road-load dyno's rollers do not turn backwards (`ChassisDynoHw`, speed ≥ 0): R shows as the 6/R
                // sleeve in R (−travel past the firmware's −6.50 mm class threshold) with K1 above its zero-torque 2.7 bar
                "reverse" => {
                    rev_k1_bar = rev_k1_bar.max(tel.clutch_bar[0]);
                    rev_gear |= tel.fork_mm[3] <= -6.5;
                }
                _ => {}
            }
            min_rpm = min_rpm.min(tel.rpm);
            let load = tel.clutch_torque_nm[0].abs() + tel.clutch_torque_nm[1].abs();
            if tel.rpm < 300.0 && !in_dip {
                in_dip = true;
                dips.push((t, phase, load));
                println!("{t:7.2} !!! engine below 300 rpm ({:.0}) in {phase}, clutch torque {:?} Nm", tel.rpm, tel.clutch_torque_nm.map(|x| x.round()));
            }
            if in_dip && load > 15.0 {
                gearbox_stall.get_or_insert(t);
            }
            in_dip &= tel.rpm < 500.0;
            if tr.iter().any(|w| (w.0..w.1).contains(&t)) {
                let g11 = s.can_last.get(&("TCU", 0x0AD)).map(|f| f.data).unwrap_or([0; 8]);
                println!(
                    "        {t:7.2} {:5.0} rpm thr {:4.1} % MAP {:4.0} kPa rail {:4.1} MPa λ {:?} fuel {:?} mg ign {:?} | T ind {:5.1} fric {:5.1} brake {:5.1} Nm | clutch T {:?} Nm | TCU request {} Nm st {}/{}",
                    tel.rpm,
                    tel.throttle_pct,
                    tel.map_kpa,
                    tel.rail_mpa,
                    tel.lambda_cyl.map(|x| (x * 100.0).round() / 100.0),
                    tel.fuel_mg_stroke.map(|x| (x * 10.0).round() / 10.0),
                    tel.combustion.ign_deg.map(|x| x.round()),
                    tel.torque_indicated_nm,
                    tel.torque_friction_nm,
                    tel.torque_brake_nm,
                    tel.clutch_torque_nm.map(|x| x.round()),
                    sig(&g11, 12, 10) - 509,
                    sig(&g11, 56, 2),
                    sig(&g11, 50, 3)
                );
                if !watch.is_empty() {
                    let w: Vec<String> = watch.iter().map(|&(a, n)| format!("{a:08X}:{:02X?}", tcu_ram(&s, a, n))).collect();
                    let m = &s.plant.driveline.mech;
                    let o = &s.actuators.tcu;
                    println!(
                        "                watch coil A {:?} duty {:?} | shafts {:?} | sub {:?} bar forks {:?} mm v {:?} m/s chambers {:?} bar | {}",
                        o.solenoid_current_a.map(|c| c.map(|x| (x * 1000.0).round() / 1000.0)),
                        o.solenoid_duty.map(|d| d.map(|x| (x * 1000.0).round() / 1000.0)),
                        tel.shaft_rpm.map(|x| x.round()),
                        m.sub_pa.map(|p| (p / 1e4).round() / 10.0),
                        tel.fork_mm.map(|x| (x * 100.0).round() / 100.0),
                        m.fork_v.map(|x| (x * 1e3).round() / 1e3),
                        m.fork_chamber_pa.map(|c| c.map(|p| (p / 1e4).round() / 10.0)),
                        w.join(" ")
                    );
                }
            }
            if ((t * 100.0).round() as u64).is_multiple_of(50) {
                let o = &s.actuators.tcu;
                let amps: Vec<String> = (0..11).map(|k| o.solenoid_duty[k].map_or("-".into(), |_| format!("{:.2}", o.solenoid_current_a.map_or(0.0, |c| c[k])))).collect();
                println!(
                    "{t:7.2} {phase:8} | {:5.0} rpm {:6.1} km/h pedal {:3.0} | plant gear {} | TCU gear {gear} mode {mode} sub {} | clutch {:?} bar line {:.1} | forks {:?} | coil A {} | kiss adapt {:02X} viol {:04X} 5826 {} 3C34 {:?} | p0 adapt {:02X} 59E8 {:?}",
                    tel.rpm,
                    tel.roller_kmh,
                    s.plant.driver.pedal_pct,
                    tel.driving_gear,
                    ram8(&s, 0xD000_5E10),
                    tel.clutch_bar.map(|x| (x * 10.0).round() / 10.0),
                    tel.gearbox_hydraulic_bar,
                    tel.fork_mm.map(|x| (x * 10.0).round() / 10.0),
                    amps.join(" "),
                    ram8(&s, 0xD000_3C85),
                    ram16(&s, 0xD000_3C62, 1)[0],
                    ram16(&s, 0xD000_5826, 1)[0],
                    ram16(&s, 0xD000_3C34, 4),
                    ram8(&s, 0xD000_41D3),
                    [ram16(&s, 0xD000_59E8, 4), ram16(&s, 0xD000_59F4, 4)]
                );
            }
        }
        let dtcs = tcu_dtcs(&mut s);
        let st = &s.uds_tcu.status;
        println!("TCU DTCs: {:?} = {:?} ({})", st.dtcs, st.dtc_sae, st.note);
        // stored during the drive (pending / confirmed, status bits 2/3) gearbox (P) faults; status 0x02 alone is the
        // firmware's undebounced first report (0D9300012F 0x80067D76: bit 1 from 0xD00057DE, set by every raw fail report).
        // P06xx = the control module's own self-tests, here the NVM blocks no factory EEPROM provides (no dump exists): factory
        // valve characteristics, blocks 4/5 → P0607 event 0x171 at every power-up (`FUN_800a7144`), P0606 event 0x170
        // (0x80049072); stored a few seconds after power-up, not by the drive.
        let new_stored: Vec<&(String, String, u8)> =
            dtcs.iter().filter(|d| d.1.starts_with('P') && !d.1.starts_with("P06") && d.2 & 0x0C != 0 && !stored_before.contains(&d.0)).collect();
        let line_adapt = ram16(&s, 0xD000_3C34, 4);
        println!("stored before the drive {stored_before:?} | stored in the drive {new_stored:?} | kickdown max {kd_max_rpm:.0} rpm | line-pressure adaptation 0xD0003C34 {line_adapt:?}");
        println!("TCU gears (t, WBA_03): {:?}", gears.iter().map(|(t, g)| format!("{t:.1}:{g}")).collect::<Vec<_>>());
        println!("TCU modes (t, 0xD0005E13): {:?}", modes.iter().map(|(t, m)| format!("{t:.1}:{m}")).collect::<Vec<_>>());
        println!(
            "top gear {top_gear} at {top_at:?} s | kickdown from {gear_before_kd} to {kd_min_gear} | braking down to {brake_min_gear} | creep after stop {creep2_kmh:.1} km/h | R: sleeve in R {rev_gear}, K1 max {rev_k1_bar:.1} bar | min rpm {min_rpm:.0}, dips below 300 rpm (t, phase, clutch Nm) {dips:.1?}, restarts {restarts:.1?}"
        );
        println!("ECU DI → MPI switches (t, phase, leanest fuelled cylinder λ in the next 0.5 s): {to_mpi:.2?}");
        for &a in &counts {
            let n = s.tcu.as_ref().unwrap().cpu().probe(a).map_or(0, |p| p.hits);
            println!("probe {a:08X}: {n} hits in {:.1} s since power-up of this TCU", s.plant.sim_time_s);
        }
        let hw = s.tcu.as_ref().unwrap().cpu().hw();
        assert!(hw.reset_log.is_empty() && hw.trap_count == 0 && s.tcu_error.is_none(), "TCU reset/trap/error: {:?} {} {:?}", hw.reset_log, hw.trap_count, s.tcu_error);
        assert!(gearbox_stall.is_none(), "the clutches dragged the engine below 300 rpm at {gearbox_stall:?} s: {dips:.1?}");
        assert!(restarts.is_empty(), "the engine stopped by itself and needed a restart: {restarts:.1?}");
        // port fuel enters with the next intake event as the firmware's model has it (`PortFuel::induct`): the well-mixed
        // runner gave λ 2.2–2.7 here (docs/SIMOS18_COMBUSTION.md §14)
        assert!(to_mpi.iter().all(|x| x.2 <= 1.7), "lean collapse after the ECU's DI → MPI switch: {to_mpi:.2?}");
        assert!(top_gear >= min_top, "top gear {top_gear} < {min_top}");
        assert!(brake_min_gear <= 2, "braking from top gear: lowest gear {brake_min_gear}");
        assert!(creep2_kmh > 3.0, "no creep after the stop: {creep2_kmh:.1} km/h");
        assert!(rev_gear && rev_k1_bar > 2.8, "R: sleeve in R {rev_gear}, K1 max {rev_k1_bar:.1} bar (creep needs > 2.7 bar)");
        assert!(new_stored.is_empty(), "gearbox faults stored during the drive: {new_stored:?}");
        // the clutches hold the engine under kickdown: with N215/N216 saturating at 12 bar K2 slipped and the engine ran
        // into the ECU's limiter (≈ 6800 rpm) in 2nd
        assert!(kd_max_rpm < 6500.0, "kickdown: engine up to {kd_max_rpm:.0} rpm (clutch slip to the limiter)");
        if is_4930 {
            // line-pressure adaptation of the clutch whose sub-gearbox was free while cruising on 4th (docs/DQ250_FIRMWARE_SPEC.md §4)
            assert!(line_adapt[0] > 0 && line_adapt[1] > 0, "line-pressure adaptation not learnt: 0xD0003C34 {line_adapt:?}");
        }
    }

    /// The studio's own path, as driven by hand: a fresh studio session (empty state directory), ECU image
    /// `SIMEMU_IMAGE` (default stock SC8O40 264H), bench build `octavia_is38` chosen and started like the UI does
    /// (`/api/build` preset + start), TCU image the studio's default (the 0D9300011R read; `SIMEMU_DQ250_IMAGE` for
    /// another), the erased mechatronic prepared by the studio itself (`studio::dq250_prep`), «Тёплый старт» (README:
    /// DQ250 in studio, step 4; `SIMEMU_STUDIO_COLD=1` skips it: the open cold-engagement case), the studio's default
    /// bench mode (road). Driver: ignition, start, P, D with the brake, creep, 30 % pedal to 50 km/h, 60 % to 90 km/h, lift,
    /// brake to a stop, P.
    ///
    /// Asserted: the preparation completes; no TCU reset / trap; the car drives off and creeps; upshifts to 4th or higher,
    /// downshifts to 2nd or 1st when braking; no gearbox (P, without the module self-tests P06xx of the erased EEPROM) fault
    /// stored during the drive (`19 02` before and after; status 0x02 alone is the undebounced first report); both clutch
    /// linings intact. `scripts/run_scenarios.sh`: `dq250_studio_default` (stock) and `dq250_studio_s50_repro` (bench
    /// SC8S50 + 0D9300012F).
    #[test]
    #[ignore = "slow: studio session, DQ250 preparation and a drive (ECU + TCU + plant)"]
    fn dq250_studio_default_drive_scenario() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let image = std::env::var("SIMEMU_IMAGE").unwrap_or_else(|_| "8V0906264H__0005.bin".into());
        if !crate::paths::has_firmware(&root, &image) || StudioSession::dq250_image_path(&root, None).is_err() {
            eprintln!("SKIP: {image} or the DQ250 image not found");
            return;
        }
        let dir = std::env::temp_dir().join(format!("simemu_studio_default_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut s = StudioSession::new(&root).expect("session");
        s.tcu_prep.auto = true;
        s.set_nvm_dir(dir.clone());
        s.load_image(&image).expect("image");
        s.build_route("POST", br#"{"action":"preset","preset":"octavia_is38"}"#).expect("preset");
        s.build_route("POST", br#"{"action":"start"}"#).expect("start");
        let t0 = std::time::Instant::now();
        while s.tcu_prep.running() && t0.elapsed().as_secs() < 1200 {
            std::thread::sleep(std::time::Duration::from_millis(100));
            s.poll_tcu_prep();
        }
        let rec = s.tcu_prep.record.clone().expect("preparation record");
        println!("ECU {} | TCU {} | build {} | bench {:?} | {}", s.image_label, s.tcu.as_ref().unwrap().sw_id(), s.bench_build.active.name, s.plant.bench, rec.summary());
        assert!(rec.ok, "preparation failed: {:?}", rec.failed);
        let cold = std::env::var("SIMEMU_STUDIO_COLD").is_ok_and(|v| v == "1");
        if !cold {
            s.apply_preset(&crate::studio::session::PresetRequest { action: "warm_soak".into(), engine_slug: None, drivetrain_slug: None, fuel_ron: None, ambient_temp_c: None, altitude_m: None, humidity_pct: None, image: None }).expect("warm soak");
        }
        let d = &mut s.plant.driver;
        (d.selector, d.brake_pct) = (Selector::P, 60.0);
        start_engine(&mut s);
        let stored_before: Vec<String> = tcu_dtcs(&mut s).into_iter().filter(|d| d.2 & 0x0C != 0).map(|d| d.0).collect();
        let (t0, dt) = (s.plant.sim_time_s, 0.01);
        let (mut phase, mut pt) = ("p_idle", t0);
        let (mut gears, mut top, mut brake_min, mut creep_kmh) = (Vec::<i64>::new(), 0i64, 9i64, 0f64);
        loop {
            let t = s.plant.sim_time_s;
            let v = s.plant.telemetry().vehicle_kmh.max(s.plant.telemetry().roller_kmh);
            let el = t - pt;
            let np = match phase {
                "p_idle" if el >= 3.0 => "d_hold",
                "d_hold" if el >= 3.0 => "creep",
                "creep" if el >= 4.0 => "p30",
                "p30" if v >= 50.0 || el >= 25.0 => "p60",
                "p60" if v >= 90.0 || el >= 20.0 => "lift",
                "lift" if el >= 5.0 => "brake",
                "brake" if v < 0.3 && el > 1.0 => "park",
                "park" if el >= 3.0 => "done",
                p => p,
            };
            if np != phase {
                (phase, pt) = (np, t);
                println!("{t:7.2} --- {phase}");
            }
            if phase == "done" || t - t0 > 150.0 {
                break;
            }
            let d = &mut s.plant.driver;
            d.selector = if matches!(phase, "p_idle" | "park") { Selector::P } else { Selector::D };
            d.brake_pct = match phase {
                "p_idle" | "d_hold" | "park" => 60.0,
                "brake" => 35.0,
                _ => 0.0,
            };
            d.pedal_pct = match phase {
                "p30" => 30.0,
                "p60" => 60.0,
                _ => 0.0,
            };
            s.step(dt);
            let tel = s.plant.telemetry();
            let gear = sig(&s.can_last.get(&("TCU", 0x394)).map(|f| f.data).unwrap_or([0; 8]), 24, 4);
            if gears.last() != Some(&gear) {
                gears.push(gear);
            }
            if (1..=6).contains(&gear) {
                top = top.max(gear);
                if phase == "brake" {
                    brake_min = brake_min.min(gear);
                }
            }
            if phase == "creep" {
                creep_kmh = creep_kmh.max(tel.vehicle_kmh.max(tel.roller_kmh));
            }
            if ((t * 100.0).round() as u64).is_multiple_of(100) {
                println!(
                    "{t:7.2} {phase:6} | {:5.0} rpm {:5.1} km/h | TCU gear {gear} plant {:?} | clutch {:?} bar line {:.1} | forks {:?} | ATF {:.0} °C",
                    tel.rpm,
                    tel.vehicle_kmh.max(tel.roller_kmh),
                    tel.driving_gear,
                    tel.clutch_bar.map(|x| (x * 10.0).round() / 10.0),
                    tel.gearbox_hydraulic_bar,
                    tel.fork_mm.map(|x| (x * 10.0).round() / 10.0),
                    tel.gearbox_case_c
                );
            }
        }
        let dtcs = tcu_dtcs(&mut s);
        let new_stored: Vec<&(String, String, u8)> =
            dtcs.iter().filter(|d| d.1.starts_with('P') && !d.1.starts_with("P06") && d.2 & 0x0C != 0 && !stored_before.contains(&d.0)).collect();
        let current: Vec<&(String, String, u8)> = dtcs.iter().filter(|d| d.2 & 0x0F != 0).collect();
        let tel = s.plant.telemetry();
        println!("TCU DTCs with a fail bit (DTC, SAE, status): {current:?}");
        println!("stored before the drive {stored_before:?} | stored in the drive {new_stored:?}");
        println!("TCU gears on WBA_03 {gears:?} | top {top} | braking down to {brake_min} | creep {creep_kmh:.1} km/h | clutch health {:?} | driveline {:?}", tel.clutch_health, tel.driveline_failures);
        let hw = s.tcu.as_ref().unwrap().cpu().hw();
        assert!(hw.reset_log.is_empty() && hw.trap_count == 0 && s.tcu_error.is_none(), "TCU reset/trap/error: {:?} {} {:?}", hw.reset_log, hw.trap_count, s.tcu_error);
        assert!(creep_kmh > 2.0, "no creep: {creep_kmh:.1} km/h");
        assert!(top >= 4, "upshifts: top gear {top} ({gears:?})");
        assert!(brake_min <= 2, "downshifts when braking: lowest gear {brake_min}");
        assert!(new_stored.is_empty(), "gearbox faults stored during the drive: {new_stored:?}");
        assert!(tel.clutch_health.iter().all(|&h| h > 0.9), "clutch linings: {:?}", tel.clutch_health);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// TCU fault memory over the session's tester (`19 02`), decoded with the image's table: (DTC hex, SAE code, status).
    fn tcu_dtcs(s: &mut StudioSession) -> Vec<(String, String, u8)> {
        s.uds_request(&crate::studio::session::UdsRequest { target: "tcu".into(), service: "read_dtc".into(), data: Vec::new(), value: None, rid: None }).expect("uds");
        for _ in 0..200 {
            s.step(0.01);
            if !s.uds_tcu.status.pending {
                break;
            }
        }
        let st = &s.uds_tcu.status;
        st.dtcs
            .iter()
            .zip(&st.dtc_sae)
            .filter_map(|(d, sae)| {
                let (hex, status) = d.split_once(" status 0x")?;
                Some((hex.to_string(), sae.clone(), u8::from_str_radix(status, 16).ok()?))
            })
            .collect()
    }

    /// Highest in-cylinder λ at spark of the cylinders that were fuelled in their last cycle (fuel cut-off excluded).
    fn lean_peak(tel: &simemu_physics::PlantTelemetry) -> f64 {
        (0..4).filter(|&c| tel.fuel_mg_stroke[c] > 1.0).map(|c| tel.lambda_cyl[c]).fold(0.0, f64::max)
    }

    /// Warm idle of the DQ250 build (`octavia_is38`: CJSA + IS38 + DQ250 0D9 AWD, stock SC8O40 by default) through
    /// the selector positions and the load steps a driver makes at standstill, then a short drive-off and a stop after an
    /// overrun cut-off (the ECU may idle on DI there and go back to MPI, docs/SIMOS18_COMBUSTION.md §14): P 10 s →
    /// D with the brake 10 s → R with the brake 10 s → P 5 s → D, 40 % pedal 8 s → coast 3 s → brake to a stop → D with
    /// the brake 3 s → creep 6 s → D with the brake 15 s → P 5 s. Driver inputs only.
    ///
    /// Asserted: the engine never stops (no restart), ≥ 500 rpm in every idle phase after 1 s, and in idle phases no
    /// fuelled cylinder leaner than λ 1.7 (the plant's lean flame limit is ≈ 1.8). Printed: the minimum speed and the
    /// leanest cylinder per phase, the ECU's combustion-mode changes (SC8O40 RAM `IDX_CMB_MOD_CUR` 0xD000C46B, observation
    /// only). The DI → MPI switch itself is guarded by [`dq250_full_drive_scenario`].
    #[test]
    #[ignore = "slow: ECU + DQ250 + plant co-simulation"]
    fn dq250_idle_scenario() {
        let Some(mut s) = cjsa_dq250_session() else { return };
        let o40 = s.image_label.contains("SC8O40");
        let d = &mut s.plant.driver;
        (d.selector, d.brake_pct) = (Selector::P, 60.0);
        start_engine(&mut s);
        let plan: [(&str, Selector, f64, f64, f64); 11] = [
            ("p", Selector::P, 60.0, 0.0, 10.0),
            ("d_hold", Selector::D, 60.0, 0.0, 10.0),
            ("r_hold", Selector::R, 60.0, 0.0, 10.0),
            ("p2", Selector::P, 60.0, 0.0, 5.0),
            ("drive", Selector::D, 0.0, 40.0, 8.0),
            ("coast", Selector::D, 0.0, 0.0, 3.0),
            ("brake", Selector::D, 40.0, 0.0, 30.0),
            ("d_stop", Selector::D, 60.0, 0.0, 3.0),
            ("creep", Selector::D, 0.0, 0.0, 6.0),
            ("d_stop2", Selector::D, 60.0, 0.0, 15.0),
            ("p_end", Selector::P, 60.0, 0.0, 5.0),
        ];
        let mut stopped = None::<(f64, &str)>;
        let mut modes = Vec::<(f64, u8)>::new();
        let mut worst = Vec::<(&str, f64, f64)>::new();
        for (name, sel, brake, pedal, secs) in plan {
            let d = &mut s.plant.driver;
            (d.selector, d.brake_pct, d.pedal_pct) = (sel, brake, pedal);
            let t0 = s.plant.sim_time_s;
            let (mut min_rpm, mut lean) = (f64::MAX, 0f64);
            while s.plant.sim_time_s - t0 < secs {
                s.step(0.01);
                let tel = s.plant.telemetry();
                let t = s.plant.sim_time_s;
                if tel.rpm < 50.0 {
                    stopped.get_or_insert((t, name));
                    break;
                }
                if o40 {
                    let m = s.board.read_mem(0xD000_C46B, 1).unwrap_or_default().first().copied().unwrap_or(0);
                    if modes.last().map(|x| x.1) != Some(m) {
                        modes.push((t, m));
                    }
                }
                if name == "brake" && tel.roller_kmh < 0.3 && t - t0 > 1.0 {
                    break;
                }
                let idle = !matches!(name, "drive" | "coast" | "brake" | "creep");
                if t - t0 > 1.0 && idle {
                    min_rpm = min_rpm.min(tel.rpm);
                    lean = lean.max(lean_peak(&tel));
                }
            }
            let tel = s.plant.telemetry();
            if matches!(name, "drive" | "coast" | "brake" | "creep") {
                println!("{:7.2} {name:7} {:5.0} rpm {:5.1} km/h", s.plant.sim_time_s, tel.rpm, tel.roller_kmh);
            } else {
                println!("{:7.2} {name:7} min {min_rpm:5.0} rpm, leanest fuelled cylinder λ {lean:.2}", s.plant.sim_time_s);
                worst.push((name, min_rpm, lean));
            }
            if stopped.is_some() {
                break;
            }
        }
        println!("ECU combustion mode (t, IDX_CMB_MOD_CUR; 8 = OPP_MPI_1, 0 = OPP_1 DI): {:?}", modes.iter().map(|(t, m)| format!("{t:.1}:{m}")).collect::<Vec<_>>());
        assert!(stopped.is_none(), "engine stopped at {stopped:?}");
        for (name, min_rpm, lean) in worst {
            assert!(min_rpm >= 500.0, "{name}: idle down to {min_rpm:.0} rpm");
            assert!(lean <= 1.7, "{name}: a fuelled cylinder at λ {lean:.2} (lean collapse)");
        }
    }

    /// Cold start of the DQ250 build (`octavia_is38`, stock SC8O40 by default) after an overnight soak at
    /// `SIMEMU_COLD_AMB` (default 20 °C; ambient, coolant, oil, ATF and the engine-off timer 12 h): ignition, start button,
    /// 30 s in P. Asserted: the engine starts and keeps running on its own (no stop, no restart), ≥ 600 rpm from
    /// `SIMEMU_COLD_FROM_S` (default 3 s) after the start button on.
    /// Printed: speed, the ECU's combustion mode (SC8O40 `IDX_CMB_MOD_CUR` 0xD000C46B: 25–29 start, 16 catalyst heating,
    /// observation only), λ and the cylinder films every 0.5 s.
    #[test]
    #[ignore = "slow: ECU + DQ250 + plant co-simulation"]
    fn cold_start_dq250_scenario() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let image = std::env::var("SIMEMU_IMAGE").unwrap_or_else(|_| "8V0906264H__0005.bin".into());
        if !crate::paths::has_firmware(&root, &image) {
            eprintln!("SKIP: {image} not found (scripts/frf_to_bin.py)");
            return;
        }
        let amb = env_f("SIMEMU_COLD_AMB", 20.0);
        let from_s = env_f("SIMEMU_COLD_FROM_S", 3.0);
        let mut s = StudioSession::new(&root).expect("session");
        s.bench_build.active = simemu_physics::bench_config::preset("octavia_is38").expect("octavia_is38");
        s.bench_build.active.ambient.temp_c = amb;
        s.plant = s.bench_build.new_plant().expect("plant");
        s.plant.bench = BenchMode::RoadLoadDyno;
        s.eot_clock_s = 12.0 * 3600.0;
        s.load_image(&image).expect("image");
        s.power_cycle_tcu();
        let o40 = s.image_label.contains("SC8O40");
        println!("ECU {} | build {} | ambient {amb} °C | coolant {:.1} °C", s.image_label, s.bench_build.active.name, s.plant.telemetry().coolant_c);
        let d = &mut s.plant.driver;
        (d.selector, d.brake_pct, d.ignition_on) = (Selector::P, 60.0, true);
        s.step(1.0);
        s.plant.driver.start_button = true;
        let t0 = s.plant.sim_time_s;
        let (mut running, mut stopped, mut min_rpm) = (None::<f64>, None::<f64>, f64::MAX);
        while s.plant.sim_time_s - t0 < 30.0 {
            s.step(0.01);
            let t = s.plant.sim_time_s - t0;
            let tel = s.plant.telemetry();
            if running.is_none() && tel.rpm > 600.0 && tel.starter_current_a <= 0.0 {
                running = Some(t);
                s.plant.driver.start_button = false;
            }
            if running.is_some() && tel.rpm < 50.0 {
                stopped = Some(t);
                break;
            }
            if t >= from_s {
                min_rpm = min_rpm.min(tel.rpm);
            }
            if ((t * 100.0).round() as u64).is_multiple_of(50) {
                let mode = if o40 { s.board.read_mem(0xD000_C46B, 1).unwrap_or_default().first().copied().unwrap_or(0) } else { 0 };
                let films: Vec<f64> = s.plant.engine.cyl.iter().map(|c| ((c.film_kg + c.liner_film_kg) * 1e7).round() / 10.0).collect();
                println!(
                    "{t:6.2} {:5.0} rpm | mode {mode} | λ {:?} | fuel {:?} mg air {:.0} mg | films {films:?} mg | coolant {:.1} °C",
                    tel.rpm,
                    tel.lambda_cyl.map(|x| (x * 100.0).round() / 100.0),
                    tel.fuel_mg_stroke.map(|x| (x * 10.0).round() / 10.0),
                    tel.air_mg_stroke,
                    tel.coolant_c
                );
            }
        }
        println!("running at {running:?} s, stopped at {stopped:?} s, min {min_rpm:.0} rpm from {from_s} s");
        assert!(running.is_some(), "the engine did not start");
        assert!(stopped.is_none(), "the engine stopped at {stopped:?} s after the start button");
        assert!(min_rpm >= 600.0, "cold idle down to {min_rpm:.0} rpm");
    }
}
