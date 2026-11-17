//! Whole-car drive scenario: the unmodified SIMOS18.1 (stock SC8O40 `images/8V0906264H__0005.bin`) and DQ200_G2
//! firmwares drive the Octavia A7 FL build (`octavia_fl`: CJSA + IS12 + DQ200 FWD) on a 4WD road-load chassis
//! dyno (`BenchMode::RoadLoadDyno`: vehicle mass, aerodynamic drag and tyre rolling resistance of the build). No bench
//! motoring drive: engine torque comes from the ECU's own control; shift torque interventions travel over the real CAN
//! between the two boards (TCU `Getriebe_11/12` → ECU, ECU `Motor_11/12/20` → TCU). Lab immobiliser binding for both
//! units (AGENTS.md §2.4 п.5).
//!
//! Driver inputs only (brake, selector, pedal, ignition, start button): ignition, start (button held until the
//! engine runs on its own, ≤ 5 s), warm idle, D with the brake held, brake released (creep), pedal 30 % up to
//! `SIMEMU_DRIVE_V1` (50 km/h) then 70 % up to `SIMEMU_DRIVE_VMAX` (100 km/h), lift off, brake to a stop, selector P.
//! Run: `SIMEMU_CPU=native cargo test --release -p simemu-cli vehicle_drive_scenario -- --ignored --nocapture`.
//! Read-only diagnostics: `SIMEMU_DRIVE_TRACE=t0,t1` (rows every 10 ms in the window), `SIMEMU_DRIVE_HYD=1` (mechatronic
//! hydraulics per row), `SIMEMU_DRIVE_CYL=1` (combustion per row in the window), `SIMEMU_TCU_WATCH=addr[:n|:wN],..`
//! (TCU DSPR bytes / signed words per row), `SIMEMU_DRIVE_OBD=t,..` (ECU load values over UDS next to the plant's torque
//! split at those times). Driver knobs: `SIMEMU_DRIVE_PEDAL1/2` (%), `SIMEMU_DRIVE_BRAKE` (% in the
//! stop, default 40 ≈ 0.5 g), `SIMEMU_DRIVE_BENCH=road` (road model instead of the dyno).
//!
//! Signals (opendbc `vw_mqb.dbc`, Intel byte order; the receivers are the firmwares themselves):
//! `Motor_11` 0x0A7 `MO_Mom_Soll_Roh` 12|10, `MO_Mom_Ist_Summe` 22|10 (Nm, offset −509); `Motor_20` 0x121
//! `MO_Solldrehz_Leerlauf` 40|8 (10 rpm); `Getriebe_11` 0x0AD `GE_MMom_Soll_02` 12|10 (Nm, −509), `GE_Fahrstufe` 42|4,
//! `GE_MMom_Status_02` 50|3, `GE_Status_Kraftschluss` 53|3, `GE_MMom_Status` 56|2, `GE_Zielgang` 60|4; `Getriebe_12` 0x0AE
//! `GE_Freigabe_Synchro` 14, `GE_Synchro_Wunschdrehz` 15|9 (25 rpm), `GE_Anheb_Solldrehz_Leerlauf` 40|8 (10 rpm),
//! `GE_Aufnahmemoment` 48|10 (Nm, −509); `WBA_03` 0x394 `WBA_eing_Gang_02` 24|4.

#[cfg(test)]
mod tests {
    use crate::studio::session::{StudioSession, S50_FLASH};
    use simemu_physics::{BenchMode, Selector};
    use std::path::PathBuf;

    const IMAGE: &str = "8V0906264H__0005.bin";
    const DQ200_FLASH: &str = crate::paths::DEFAULT_DQ200_FLASH;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    fn env_f(k: &str, d: f64) -> f64 {
        std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d)
    }

    /// Intel (little-endian) signal of a CAN payload: `len` bits from bit `start`.
    fn sig(d: &[u8; 8], start: u32, len: u32) -> i64 {
        ((u64::from_le_bytes(*d) >> start) & ((1u64 << len) - 1)) as i64
    }

    /// Fault memory of the ECU (UDS 19 02 FF through the session's tester): (DTC, status) per record.
    fn read_ecu_dtcs(s: &mut StudioSession) -> Vec<(u32, u8)> {
        let f = s.uds_ecu.request("ReadDTCInformation", &[0x19, 0x02, 0xFF]);
        s.board.inject_can_rx(&f);
        for _ in 0..400 {
            s.step(0.01);
            if !s.uds_ecu.status.pending {
                break;
            }
        }
        s.uds_ecu.status.dtcs.iter().filter_map(|d| {
            let (code, st) = d.split_once(" status 0x")?;
            Some((u32::from_str_radix(code, 16).ok()?, u8::from_str_radix(st, 16).ok()?))
        }).collect()
    }

    /// ReadDataByIdentifier of OBD DIDs (0xF4xx = mode-01 PID xx) through the session's tester: the data bytes of each DID
    /// (`len` each), `None` without a positive answer.
    fn read_obd(s: &mut StudioSession, dids: &[(u16, usize)]) -> Option<Vec<Vec<u8>>> {
        let mut req = vec![0x22];
        dids.iter().for_each(|(d, _)| req.extend(d.to_be_bytes()));
        let f = s.uds_ecu.request("ReadDataByIdentifier", &req);
        s.board.inject_can_rx(&f);
        for _ in 0..50 {
            s.step(0.01);
            if !s.uds_ecu.status.pending {
                break;
            }
        }
        let r: Vec<u8> = s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
        let mut i = 1;
        let mut out = Vec::new();
        for &(d, n) in dids {
            (r.first() == Some(&0x62) && r.get(i..i + 2) == Some(&d.to_be_bytes()[..])).then_some(())?;
            out.push(r.get(i + 2..i + 2 + n)?.to_vec());
            i += 2 + n;
        }
        Some(out)
    }

    /// The ECU's own load and torque split (OBD PIDs and VW measured values over UDS) next to the plant's at time `t`
    /// (observation only).
    fn print_torque_split(s: &mut StudioSession, t: f64) {
        let f = &s.plant.engine.flows;
        let plant = format!(
            "plant: brake {:.1} Nm = indicated {:.1} − friction {:.1} − pumping {:.1} − aux {:.1} Nm | air {:.0} mg/stroke | MAP {:.0} kPa",
            s.plant.telemetry().torque_brake_nm, f.torque_indicated_nm, f.torque_friction_nm, f.torque_pumping_nm, f.torque_aux_nm, f.air_per_cyl_mg, s.plant.telemetry().map_kpa
        );
        // PID 04 calculated load %, 43 absolute load %, 0B MAP kPa, 0C rpm/4, 62 actual torque % (−125), 63 reference torque Nm,
        // 8E friction torque % (−125)
        let a = read_obd(s, &[(0xF404, 1), (0xF443, 2), (0xF40B, 1)]);
        let b = read_obd(s, &[(0xF462, 1), (0xF463, 2), (0xF48E, 1)]);
        println!("OBD at {t:.2} s: load/abs load/MAP {a:?} | torque %/ref Nm/friction % {b:?} | {plant}");
        // VW measured values (FR p.4025/4044/4045): TQ_ENG, TQ_LOSS_ENG, TQ_LOSS_ADD (0.1 Nm), IGA_AV_MV (0.01 °)
        if let Some(v) = read_obd(s, &[(0x437C, 2), (0x209C, 2), (0x209B, 2)]).zip(read_obd(s, &[(0x2004, 2), (0x2032, 2)])) {
            let w = |b: &[u8], signed: bool| if signed { i16::from_be_bytes([b[0], b[1]]) as f64 } else { u16::from_be_bytes([b[0], b[1]]) as f64 };
            let (tq, loss, add) = (w(&v.0[0], true) / 10.0, w(&v.0[1], false) / 10.0, w(&v.0[2], false) / 10.0);
            println!("  ECU DIDs: TQ_ENG {tq:.1} Nm, TQ_LOSS_ENG {loss:.1} Nm, TQ_LOSS_ADD {add:.1} Nm → TQ_ENG + TQ_LOSS_ENG {:.1} Nm | IGA_AV_MV {:.1} ° | MAF_CYL {} kg/h", tq + loss, w(&v.1[0], true) / 100.0, w(&v.1[1], false));
        }
        // Idle-speed controller I-share TQ_DIF_I_IS 0x437D, loss adaptations TQ_DIF_IS_AD 0x2945 / _CONV 0x437E (0.1 Nm),
        // A/C compressor TQ_LOSS_ACC 0x2017 (0.01 Nm), generator load GEN_LOAD_EGY_MNG_CAN 0x2050 (0.01 %) — FR p.4025/4044
        let i16s = |d: u16, s: &mut StudioSession| read_obd(s, &[(d, 2)]).map(|v| i16::from_be_bytes([v[0][0], v[0][1]]) as f64);
        let (i_is, ad, ad_conv, acc, gen) = (i16s(0x437D, s), i16s(0x2945, s), i16s(0x437E, s), i16s(0x2017, s), i16s(0x2050, s));
        let (alt_w, alt_pct) = s.plant.alternator_report();
        let w_crank = s.plant.engine.omega_rad_s.max(1.0);
        println!(
            "  ECU idle/loss: TQ_DIF_I_IS {:?} TQ_DIF_IS_AD {:?} TQ_DIF_IS_AD_CONV {:?} (0.1 Nm) TQ_LOSS_ACC {:?} (0.01 Nm) GEN_LOAD_EGY_MNG_CAN {:?} (0.01 %) | plant alternator {alt_w:.0} W {:.1} Nm {alt_pct:.0} %",
            i_is, ad, ad_conv, acc, gen, alt_w / w_crank
        );
        if let (Some(a), Some(b)) = (a, b) {
            let r = u16::from_be_bytes([b[1][0], b[1][1]]) as f64;
            let pct = |x: u8| x as f64 - 125.0;
            println!(
                "  ECU: calculated load {:.1} %, absolute load {:.1} %, MAP {} kPa | actual torque {:.0} % = {:.1} Nm, friction {:.0} % = {:.1} Nm (reference {r} Nm)",
                a[0][0] as f64 / 2.55, u16::from_be_bytes([a[1][0], a[1][1]]) as f64 / 2.55, a[2][0], pct(b[0][0]), pct(b[0][0]) * r / 100.0, pct(b[2][0]), pct(b[2][0]) * r / 100.0
            );
        }
    }

    /// Driver phases.
    #[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
    enum Phase {
        Ignition,
        Start,
        Idle,
        HoldD,
        Creep,
        Pedal1,
        Pedal2,
        Coast,
        Brake,
        Park,
        Done,
    }

    /// One shift torque intervention seen on CAN (TCU `GE_MMom_Status` ≠ 0) and the ECU's / engine's answer.
    #[derive(Debug, Default, Clone)]
    struct Intervention {
        t0: f64,
        t1: f64,
        gear: i64,
        target_gear: i64,
        /// Lowest `GE_MMom_Soll_02` the TCU requested, Nm.
        req_min_nm: i64,
        /// ECU `MO_Mom_Ist_Summe` 100 ms before and lowest during, Nm.
        ecu_before_nm: i64,
        ecu_min_nm: i64,
        /// Plant brake torque 100 ms before and lowest during, Nm.
        eng_before_nm: f64,
        eng_min_nm: f64,
        /// Mean decoded spark advance (° BTDC) 100 ms before and lowest during.
        spark_before: f64,
        spark_min: f64,
        /// Most cylinders without a DI or MPI pulse in one 10 ms sample (cylinder cut-off).
        cyl_cut_max: usize,
    }

    /// Warm restarts on the Octavia A7 FL build (studio QA: the first start of a session ran, later ones fired to 500–830 rpm with
    /// λ 0.5–0.65 and died, the battery fell to 7.7 V while cranking): ignition on, start button until the engine runs on its
    /// own (≤ 5 s), idle, ignition off for `SIMEMU_RESTART_OFF_S` (default 10 s), three times in one session. Driver inputs
    /// only.
    #[test]
    #[ignore = "slow: ~70 s of ECU + TCU + plant co-simulation"]
    fn engine_restart_scenario() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        if !crate::paths::has_firmware(&root, IMAGE) {
            eprintln!("SKIP: {IMAGE} not found (scripts/frf_to_bin.py)");
            return;
        }
        let mut s = StudioSession::new(&root).expect("session");
        // `SIMEMU_RESTART_STUDIO=1`: the studio's own start-up state (its default image and build) instead of `octavia_fl`
        let studio = std::env::var_os("SIMEMU_RESTART_STUDIO").is_some();
        if !studio {
            s.bench_build.active = simemu_physics::bench_config::preset("octavia_fl").expect("octavia_fl");
            s.plant = s.bench_build.new_plant().expect("plant");
        }
        s.plant.soak(90.0);
        s.eot_clock_s = 300.0;
        if !studio {
            s.load_image(IMAGE).expect("image");
        } else {
            s.power_cycle_ecu().expect("ECU");
        }
        println!("image {} | build {}", s.image_label, s.bench_build.active.name);
        let off_s = env_f("SIMEMU_RESTART_OFF_S", 10.0);
        // `SIMEMU_RESTART_TAP_S`: start button released after this time (a tap, as on the studio button) instead of held until
        // the engine runs
        let tap_s = env_f("SIMEMU_RESTART_TAP_S", 5.0);
        let resets0 = s.board.hw().reset_count;
        let dt = 0.01;
        let mut t = 0.0;
        // (cranking time to run s, battery min while cranking V, min idle rpm after it, λ sensor range in the first 2 s)
        type AttemptStat = (Option<f64>, f64, f64, (f64, f64));
        let mut report: Vec<AttemptStat> = Vec::new();
        let film = |s: &StudioSession| s.plant.engine.port.iter().map(|p| p.film_kg).sum::<f64>() * 1e6;
        for attempt in 0..3 {
            let d = &mut s.plant.driver;
            (d.ignition_on, d.brake_pct, d.selector) = (true, 60.0, Selector::P);
            let (mut started, mut v_min, mut rpm_min, mut lam) = (None::<f64>, f64::MAX, f64::MAX, (f64::MAX, 0f64));
            let t_key = t;
            while t < t_key + 16.0 {
                let since = t - t_key;
                s.plant.driver.start_button = since >= 1.0 && started.is_none() && since < 1.0 + tap_s;
                s.step(dt);
                t += dt;
                let tel = s.plant.telemetry();
                if s.plant.driver.start_button {
                    v_min = v_min.min(tel.battery_v);
                }
                if started.is_none() && since >= 1.0 && tel.rpm > 600.0 && tel.starter_current_a <= 0.0 {
                    started = Some(since - 1.0);
                }
                if let Some(t0) = started {
                    if since - 1.0 - t0 > 0.5 {
                        rpm_min = rpm_min.min(tel.rpm);
                    }
                    if since - 1.0 - t0 < 2.0 && tel.lambda_sensor > 0.0 {
                        lam = (lam.0.min(tel.lambda_sensor), lam.1.max(tel.lambda_sensor));
                    }
                }
                if ((t * 100.0).round() as u64).is_multiple_of(50) {
                    let fuel: f64 = tel.fuel_mg_stroke.iter().sum::<f64>() / 4.0;
                    println!(
                        "{t:6.2} attempt {attempt} | {:5.0} rpm | Vbatt {:5.2} V starter {:4.0} A | λ {:.2} | fuel {fuel:5.1} mg air {:4.0} mg | port film {:.1} mg | KL15 {} KL50 {}",
                        tel.rpm, tel.battery_v, tel.starter_current_a, tel.lambda_sensor, tel.air_mg_stroke, film(&s), s.plant.driver.ignition_on as u8, s.plant.driver.start_button as u8
                    );
                }
            }
            report.push((started, v_min, rpm_min, lam));
            // key off: the engine stops, the ECU runs its after-run
            s.plant.driver.ignition_on = false;
            s.plant.driver.start_button = false;
            let t_off = t;
            while t < t_off + off_s {
                s.step(dt);
                t += dt;
                if ((t * 100.0).round() as u64).is_multiple_of(100) {
                    let tel = s.plant.telemetry();
                    println!("{t:6.2} key off   | {:5.0} rpm | Vbatt {:5.2} V | main relay {:?} | port film {:.1} mg", tel.rpm, tel.battery_v, s.actuators.ecu.main_relay, film(&s));
                }
            }
        }
        println!("\n=== engine_restart_scenario ({:?}) ===", s.board.backend());
        for (i, (st, v, rpm, lam)) in report.iter().enumerate() {
            println!("attempt {i}: running after {st:?} s of cranking, battery min {v:.2} V while cranking, min idle rpm {rpm:.0}, λ (sensor) {:.2}..{:.2} in the first 2 s", lam.0, lam.1);
        }
        let resets = s.board.hw().reset_count - resets0;
        println!("ECU resets {resets}; ECU error {:?}, TCU error {:?}", s.ecu_error, s.tcu_error);
        assert!(s.ecu_error.is_none(), "{:?}", s.ecu_error);
        for (i, (st, v, rpm, _)) in report.iter().enumerate() {
            assert!(st.is_some_and(|x| x < 5.0), "attempt {i}: no start ({st:?})");
            assert!(*rpm > 500.0, "attempt {i}: idle fell to {rpm:.0} rpm (stall)");
            // the starter's inrush pulls a healthy battery to ≈ 7–8 V for some ms (below ≈ 6 V the control units brown out)
            assert!(*v > 7.0, "attempt {i}: battery {v:.2} V while cranking");
        }
    }

    /// TCU RAM, read only: signed 16-bit word at `addr` (DSPR).
    fn tcu_i16(s: &StudioSession, addr: u32) -> i16 {
        let d = &s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().dspr;
        let o = (addr & 0xF_FFFF) as usize;
        i16::from_le_bytes([d[o], d[o + 1]])
    }

    /// Steady-gear clutch slip verification on the Octavia A7 FL build (`octavia_fl`, stock SC8O40 264H + DQ200_G2): ignition, start, D, pedal steps through 1 → 2 → 3 → 4 at
    /// part load, then a kickdown. Driver inputs only. Per gear (WBA_03) and pedal, in steady windows (gear and pedal
    /// unchanged 1 s before and 0.3 s after the sample, pedal pressed, and the driving clutch synchronised once since the
    /// gear came in — launch and shift slip are the firmware's choice): slip of the clutch that drives (engine − its input
    /// shaft), the firmware's clutch torque request and travel target (`0xD0006020`, `0xD0006092/9A`, observation only)
    /// against the plant's travel, capacity and pressure; the system pressure margin over the P0841 condition (36.0 bar,
    /// docs/DQ200_IO_MAP.md §13.3).
    ///
    /// Bound: in a steady gear the DQ200_G2 asks the driving clutch for the engine torque (part load, micro-slip) or more
    /// (kickdown: 1.5–1.8×) through its own clutch model, and the plant now has that clutch (EEPROM characteristic,
    /// `clutch::DryClutchHw::dq200`). 95 % of the steady samples below 30 rpm — the micro-slip the firmware then keeps is
    /// ≈ 5 rpm on average; the slip threshold of the firmware itself (filtered slip `0xD0007D48` from `0xD00079A8/AC`,
    /// `0x800D4360`) was not located, so 30 rpm is a margin, not a firmware constant. The fault memory may hold only the
    /// known history of the start (U0002, U0103) and P0602 ×4 (docs/DQ200_IO_MAP.md §11.5). Before the fix the steady 3rd
    /// slipped p95 118 rpm and 4th 129 rpm (docs/DQ200_IO_MAP.md §14). Rows also carry the ECU's `MO_Mom_Ist_Summe` and the
    /// plant's clutch torque; `SIMEMU_DRIVE_OBD=t,..` prints the ECU's torque split over UDS, `SIMEMU_DRIVE_TRACE=t0,t1` rows
    /// every 10 ms (docs/SIMOS18_COMBUSTION.md §18).
    #[test]
    #[ignore = "slow: ~60 s of ECU + TCU + plant co-simulation"]
    fn dq200_slip_scenario() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        if !crate::paths::has_firmware(&root, IMAGE) {
            eprintln!("SKIP: {IMAGE} not found (scripts/frf_to_bin.py)");
            return;
        }
        let mut s = StudioSession::new(&root).expect("session");
        s.bench_build.active = simemu_physics::bench_config::preset("octavia_fl").expect("octavia_fl");
        s.plant = s.bench_build.new_plant().expect("plant");
        s.plant.soak(90.0);
        s.plant.bench = BenchMode::Road;
        s.eot_clock_s = 300.0;
        s.load_image(IMAGE).expect("image");
        assert!(s.tcu.is_some(), "TCU: {:?}", s.tcu_error);
        println!("image {} | build {} | {:?}", s.image_label, s.bench_build.active.name, s.plant.bench);
        let tcu_resets0 = s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().reset_log.len();
        let (pedal1, kick_s) = (env_f("SIMEMU_SLIP_PEDAL", 25.0), env_f("SIMEMU_SLIP_KICK_S", 4.0));
        let (mut t, dt) = (0.0, 0.01);
        let mut running = None::<f64>;
        let (mut t_d, mut t4, mut t_kick) = (None::<f64>, None::<f64>, None::<f64>);
        #[derive(Clone, Copy)]
        struct Row {
            t: f64,
            gear: i64,
            pedal: f64,
            /// the driving clutch has synchronised (|slip| < 10 rpm) since this gear came in
            synced: bool,
            k: usize,
            slip: f64,
            req: f64,
            target: f64,
            travel: f64,
            cap: f64,
            bar: f64,
            eng: f64,
            /// ECU `Motor_11.MO_Mom_Ist_Summe` (Nm) and the torque the plant's driving clutch carries (Nm)
            ecu: f64,
            clu: f64,
        }
        let mut rows: Vec<Row> = Vec::new();
        // `SIMEMU_DRIVE_OBD=t,..`: the ECU's torque split (UDS DIDs) next to the plant's at these times (observation only)
        let mut obd_at: Vec<f64> = std::env::var("SIMEMU_DRIVE_OBD").unwrap_or_default().split(',').filter_map(|x| x.parse().ok()).collect();
        // `SIMEMU_DRIVE_TRACE=t0,t1`: rows every 10 ms in the window
        let (tr0, tr1) = std::env::var("SIMEMU_DRIVE_TRACE").ok().and_then(|v| v.split_once(',').map(|(a, b)| (a.parse().unwrap_or(0.0), b.parse().unwrap_or(0.0)))).unwrap_or((-1.0, -1.0));
        let (mut p_min, mut p_min_t) = (f64::MAX, 0.0);
        let mut min_rpm = f64::MAX;
        while t < 75.0 {
            let tel = s.plant.telemetry();
            if running.is_none() && tel.rpm > 600.0 && tel.starter_current_a <= 0.0 {
                running = Some(t);
            }
            let gear = sig(&s.can_last.get(&("TCU", 0x394)).map(|f| f.data).unwrap_or([0; 8]), 24, 4);
            if t_d.is_none() && running.is_some_and(|r| t > r + 4.0) {
                t_d = Some(t);
            }
            if t4.is_none() && gear == 4 {
                t4 = Some(t);
            }
            if t_kick.is_none() && t4.is_some_and(|t4| t > t4 + 6.0) {
                t_kick = Some(t);
            }
            let d = &mut s.plant.driver;
            d.ignition_on = true;
            d.start_button = t >= 1.0 && running.is_none();
            d.selector = if t_d.is_some() { Selector::D } else { Selector::P };
            d.brake_pct = match t_d {
                Some(td) if t > td + 2.0 => 0.0,
                _ => 60.0,
            };
            d.pedal_pct = match (t_d, t_kick) {
                (_, Some(tk)) if t < tk + kick_s => 100.0,
                (_, Some(_)) => 0.0,
                (Some(td), None) if t > td + 3.0 => pedal1,
                _ => 0.0,
            };
            if t_kick.is_some_and(|tk| t > tk + kick_s + 2.0) {
                break;
            }
            s.step(dt);
            t += dt;

            if obd_at.first().is_some_and(|&x| t >= x) {
                obd_at.remove(0);
                print_torque_split(&mut s, t);
            }
            let tel = s.plant.telemetry();
            if t_d.is_some() {
                min_rpm = min_rpm.min(tel.rpm);
                let p = tcu_i16(&s, 0xD000_7E30) as f64 / 100.0;
                if p < p_min {
                    (p_min, p_min_t) = (p, t);
                }
            }
            let gear = sig(&s.can_last.get(&("TCU", 0x394)).map(|f| f.data).unwrap_or([0; 8]), 24, 4);
            if (1..=7).contains(&gear) {
                // odd gears on K1 (input shaft 1), even on K2
                let k = if gear % 2 == 1 { 0 } else { 1 };
                let slip = tel.clutch_slip_rpm[k];
                let synced = rows.last().is_some_and(|r| r.gear == gear && r.synced) || slip.abs() < 10.0;
                rows.push(Row {
                    t,
                    gear,
                    pedal: s.plant.driver.pedal_pct,
                    synced,
                    k,
                    slip,
                    req: tcu_i16(&s, 0xD000_6020 + 2 * k as u32) as f64 / 10.0,
                    target: tcu_i16(&s, 0xD000_6092 + 8 * k as u32) as f64 / 100.0,
                    travel: tel.clutch_travel_mm[k],
                    cap: tel.clutch_capacity_nm[k],
                    bar: tel.clutch_bar[k],
                    eng: tel.torque_brake_nm,
                    ecu: (sig(&s.can_last.get(&("ECU", 0x0A7)).map(|f| f.data).unwrap_or([0; 8]), 22, 10) - 509) as f64,
                    clu: tel.clutch_torque_nm[k],
                });
            }
            if ((t * 100.0).round() as u64).is_multiple_of(50) || (tr0..tr1).contains(&t) {
                if let Some(r) = rows.last().filter(|r| (r.t - t).abs() < 1e-6) {
                    println!(
                        "{t:6.2} gear {} K{} slip {:5.0} rpm | req {:5.0} Nm target {:5.2} mm | travel {:5.2} mm cap {:4.0} Nm {:4.1} bar | eng {:4.0} Nm clutch {:4.0} ECU {:4.0} | {:5.0} rpm {:5.1} km/h ped {:3.0} | sys {:.1} bar",
                        r.gear, r.k + 1, r.slip, r.req, r.target, r.travel, r.cap, r.bar, r.eng, r.clu, r.ecu, tel.rpm, tel.vehicle_kmh, r.pedal, tcu_i16(&s, 0xD000_7E30) as f64 / 100.0
                    );
                }
            }
        }
        let steady = |i: usize| {
            let r0 = rows[i];
            r0.pedal > 0.0
                && r0.synced
                && rows.iter().filter(|r| (r0.t - 1.0..=r0.t + 0.3).contains(&r.t)).all(|r| r.gear == r0.gear && r.pedal == r0.pedal)
                && rows.last().is_some_and(|r| r.t >= r0.t + 0.3)
                && rows.first().is_some_and(|r| r.t <= r0.t - 1.0)
        };
        println!("\n=== dq200_slip_scenario ({:?}) === start {running:?} D {t_d:?} 4th {t4:?} kickdown {t_kick:?}", s.board.backend());
        // (gear, pedal %, samples, p95 |slip| rpm)
        let mut stats: Vec<(i64, f64, usize, f64)> = Vec::new();
        for g in 1..=7 {
            for pedal in [pedal1, 100.0] {
                let mut v: Vec<Row> = (0..rows.len()).filter(|&i| rows[i].gear == g && rows[i].pedal == pedal && steady(i)).map(|i| rows[i]).collect();
                if v.is_empty() {
                    continue;
                }
                let n = v.len() as f64;
                let mean = |f: fn(&Row) -> f64| v.iter().map(f).sum::<f64>() / n;
                let mean_slip = mean(|r| r.slip.abs());
                let line = format!(
                    "request {:.0} Nm, target {:.2} mm, travel {:.2} mm, capacity {:.0} Nm, {:.1} bar, engine {:.0} Nm, clutch {:.0} Nm, ECU MO_Mom_Ist_Summe {:.0} Nm",
                    mean(|r| r.req), mean(|r| r.target), mean(|r| r.travel), mean(|r| r.cap), mean(|r| r.bar), mean(|r| r.eng), mean(|r| r.clu), mean(|r| r.ecu)
                );
                v.sort_by(|a, b| a.slip.abs().total_cmp(&b.slip.abs()));
                let p95 = v[(0.95 * (n - 1.0)) as usize].slip.abs();
                let over = v.iter().filter(|r| r.slip.abs() > 30.0).count() as f64 / n;
                println!("gear {g} (K{}) pedal {pedal:.0} %: {n} steady samples, slip mean {mean_slip:.1} p95 {p95:.1} rpm, > 30 rpm {:.0} % | {line}", v[0].k + 1, 100.0 * over);
                stats.push((g, pedal, v.len(), p95));
            }
        }
        let tcu_resets = s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().reset_log.len() - tcu_resets0;
        println!("system pressure (TCU 0xD0007E30) min {p_min:.2} bar at {p_min_t:.2} s: margin {:.2} bar over the P0841 condition (36.0 bar)", p_min - 36.0);
        let tcu_dtcs = s.tcu.as_mut().unwrap().dq200_mut().unwrap().read_dtcs().expect("TCU UDS 19 02");
        println!("min engine speed after D {min_rpm:.0} rpm; TCU resets {tcu_resets}; TCU fault memory {tcu_dtcs:X?}");
        assert!(s.tcu_error.is_none() && tcu_resets == 0, "TCU error {:?}, resets {tcu_resets}", s.tcu_error);
        assert!(min_rpm > 500.0, "engine speed fell to {min_rpm:.0} rpm");
        for g in 2..=4 {
            assert!(stats.iter().any(|s| s.0 == g && s.1 == pedal1 && s.2 >= 50), "no steady gear {g} at {pedal1} %: {stats:?}");
        }
        assert!(stats.iter().any(|s| s.1 == 100.0 && s.2 >= 50), "no steady gear at kickdown: {stats:?}");
        for &(g, pedal, n, p95) in stats.iter().filter(|s| s.2 >= 50) {
            assert!(p95 < 30.0, "gear {g} at {pedal} %: the driving clutch slips {p95:.0} rpm (p95 of {n} steady samples)");
        }
        let known = ["U0002", "U0103", "P0602"];
        assert!(tcu_dtcs.iter().all(|d| known.contains(&d.2.as_str())), "TCU faults from the drive: {tcu_dtcs:X?}");
    }

    #[test]
    #[ignore = "slow: ~80 s of ECU + TCU + plant co-simulation"]
    fn vehicle_drive_scenario() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        if !crate::paths::has_firmware(&root, IMAGE) {
            eprintln!("SKIP: {IMAGE} not found (scripts/frf_to_bin.py)");
            return;
        }
        let mut s = StudioSession::new(&root).expect("session");
        // Stock Octavia A7 FL build (`octavia_fl`: the plant re-created from the build, as a bench start does).
        s.bench_build.active = simemu_physics::bench_config::preset("octavia_fl").expect("octavia_fl");
        s.plant = s.bench_build.new_plant().expect("plant");
        // Warm engine (hot restart after a short stop): coolant/oil 90 °C, engine-off timer 300 s.
        s.plant.soak(90.0);
        // `SIMEMU_DRIVE_BENCH=road`: the same drive on the road model instead of the road-load dyno.
        let road = std::env::var("SIMEMU_DRIVE_BENCH").is_ok_and(|v| v == "road");
        s.plant.bench = if road { BenchMode::Road } else { BenchMode::RoadLoadDyno };
        let kmh = |t: &simemu_physics::PlantTelemetry| if road { t.vehicle_kmh } else { t.roller_kmh };
        s.eot_clock_s = 300.0;
        s.load_image(IMAGE).expect("image");
        assert!(s.tcu.is_some(), "TCU: {:?}", s.tcu_error);
        println!("image {} | build {} | {}", s.image_label, s.bench_build.active.name, s.immo.binding);
        let ecu_resets0 = s.board.hw().reset_count;
        let tcu_resets0 = s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().reset_log.len();
        let ecu_err = {
            let flash = crate::paths::read_firmware(&root, IMAGE).unwrap();
            let off = s.damos.parameters.iter().find(|p| p.id == "c_err_dtc_cus_aeb_oc_0").map(|p| p.cpu_address as usize);
            off.map(|a| crate::studio::tune_check::ecu_error_table(&flash, a - 0x8080_0000)).unwrap_or_default()
        };

        let (t_d, creep_s) = (env_f("SIMEMU_DRIVE_D_S", 10.0), env_f("SIMEMU_DRIVE_CREEP_S", 4.0));
        let (v1, vmax, t_max) = (env_f("SIMEMU_DRIVE_V1", 50.0), env_f("SIMEMU_DRIVE_VMAX", 100.0), env_f("SIMEMU_DRIVE_S", 150.0));
        let (p1, p2, brake) = (env_f("SIMEMU_DRIVE_PEDAL1", 30.0), env_f("SIMEMU_DRIVE_PEDAL2", 70.0), env_f("SIMEMU_DRIVE_BRAKE", 40.0));
        // `SIMEMU_DRIVE_OBD=t,..`: the ECU's own torque / load values (OBD, UDS 22 F4xx) next to the plant's at these times
        let mut obd_at: Vec<f64> = std::env::var("SIMEMU_DRIVE_OBD").unwrap_or_default().split(',').filter_map(|x| x.parse().ok()).collect();
        let (tr0, tr1) = std::env::var("SIMEMU_DRIVE_TRACE").ok().and_then(|v| v.split_once(',').map(|(a, b)| (a.parse().unwrap_or(0.0), b.parse().unwrap_or(0.0)))).unwrap_or((-1.0, -1.0));
        let hex = |s: &str| u32::from_str_radix(s.trim().trim_start_matches("0x"), 16).unwrap();
        // `addr:n` = n bytes (hex), `addr:wN` = N signed 16-bit words.
        let watch: Vec<(u32, usize, bool)> = std::env::var("SIMEMU_TCU_WATCH").unwrap_or_default().split(',').filter(|x| !x.is_empty())
            .map(|a| match a.split_once(':') {
                None => (hex(a), 1, false),
                Some((a, n)) => n.strip_prefix('w').map_or_else(|| (hex(a), n.parse().unwrap(), false), |w| (hex(a), w.parse().unwrap(), true)),
            })
            .collect();

        // `SIMEMU_TCU_WWATCH=lo,hi[;lo,hi..]` (Unicorn only): TCU data writes into [lo, hi], printed at the end (writer pc,
        // return address a11).
        for r in std::env::var("SIMEMU_TCU_WWATCH").unwrap_or_default().split(';').filter(|x| !x.is_empty()) {
            let (lo, hi) = r.split_once(',').unwrap();
            s.tcu.as_mut().unwrap().dq200_mut().unwrap().cpu.add_write_watch(hex(lo), hex(hi)).expect("write watch needs SIMEMU_CPU=unicorn");
        }
        let mut phase = Phase::Ignition;
        let mut phase_t = 0.0;
        let mut phases: Vec<(f64, Phase)> = vec![(0.0, phase)];
        let (mut t, dt) = (0.0, 0.01);
        let mut running_s: Option<f64> = None;
        // stall = crank below 100 rpm for 0.3 s while the driver expects the engine to run; dips below 400 rpm reported
        let (mut stall_s, mut min_rpm_driving, mut low_since, mut dips) = (None::<f64>, f64::MAX, None::<f64>, Vec::<(f64, f64)>::new());
        // lowest engine speed while creeping; braking: (start s, speed km/h) and the time the car came to rest
        let (mut min_rpm_creep, mut brake_from, mut stop_s) = (f64::MAX, None::<(f64, f64)>, None::<f64>);
        let (mut state9_s, mut launch_s) = (None::<f64>, None::<f64>);
        let mut max_kmh = 0f64;
        let mut gear_seq: Vec<(f64, i64)> = Vec::new();
        let mut interventions: Vec<Intervention> = Vec::new();
        let mut open: Option<Intervention> = None;
        let (mut idle_raise_max, mut nsp_max_creep, mut sync_reqs) = (0i64, 0i64, 0u32);
        // (plant Nm, ECU MO_Mom_Ist_Summe, spark) of the last 100 ms
        let mut hist: std::collections::VecDeque<(f64, i64, f64)> = Default::default();
        let mut rows: Vec<String> = Vec::new();
        let wall = std::time::Instant::now();
        while t < t_max - 1e-9 && phase != Phase::Done {
            let v = kmh(&s.plant.telemetry());
            let np = match phase {
                Phase::Ignition if t >= 1.0 => Phase::Start,
                Phase::Start if running_s.is_some() || t >= 6.0 => Phase::Idle,
                Phase::Idle if t >= t_d => Phase::HoldD,
                Phase::HoldD if t >= t_d + 2.0 => Phase::Creep,
                Phase::Creep if t >= t_d + 2.0 + creep_s => Phase::Pedal1,
                Phase::Pedal1 if v >= v1 => Phase::Pedal2,
                Phase::Pedal2 if v >= vmax => Phase::Coast,
                Phase::Coast if t - phase_t >= 5.0 => Phase::Brake,
                Phase::Brake if v < 0.5 && t - phase_t > 1.0 => Phase::Park,
                Phase::Park if t - phase_t >= 3.0 => Phase::Done,
                p => p,
            };
            if np != phase {
                (phase, phase_t) = (np, t);
                phases.push((t, phase));
            }
            let d = &mut s.plant.driver;
            d.ignition_on = true;
            d.start_button = phase == Phase::Start;
            d.selector = if (Phase::HoldD..=Phase::Brake).contains(&phase) { Selector::D } else { Selector::P };
            d.brake_pct = match phase {
                Phase::Ignition | Phase::Start | Phase::Idle | Phase::HoldD => 60.0,
                Phase::Brake | Phase::Park | Phase::Done => brake,
                _ => 0.0,
            };
            d.pedal_pct = match phase {
                Phase::Pedal1 => p1,
                Phase::Pedal2 => p2,
                _ => 0.0,
            };
            s.step(dt);
            t += dt;

            // ---- observation (read-only) ----
            if obd_at.first().is_some_and(|&x| t >= x) {
                obd_at.remove(0);
                print_torque_split(&mut s, t);
            }
            let tel = s.plant.telemetry();
            let v = kmh(&tel);
            max_kmh = max_kmh.max(v);
            if running_s.is_none() && tel.rpm > 600.0 && tel.starter_current_a <= 0.0 {
                running_s = Some(t);
            }
            if phase == Phase::Creep {
                min_rpm_creep = min_rpm_creep.min(tel.rpm);
            }
            if phase == Phase::Brake {
                brake_from.get_or_insert((t, v));
            }
            if brake_from.is_some() && v < 0.5 {
                stop_s.get_or_insert(t);
            }
            if running_s.is_some() && phase < Phase::Park {
                min_rpm_driving = min_rpm_driving.min(tel.rpm);
                let t0 = if tel.rpm < 100.0 { *low_since.get_or_insert(t) } else { low_since.take().map_or(t, |_| t) };
                if t - t0 >= 0.3 {
                    stall_s.get_or_insert(t0);
                }
                if tel.rpm < 400.0 {
                    match dips.last_mut() {
                        Some(d) if t - d.0 < 1.0 => d.1 = d.1.min(tel.rpm),
                        _ => dips.push((t, tel.rpm)),
                    }
                }
            }
            let fr = |src: &'static str, id: u32| s.can_last.get(&(src, id)).map(|f| f.data).unwrap_or([0; 8]);
            let (m11, m20, g11, g12, wba) = (fr("ECU", 0x0A7), fr("ECU", 0x121), fr("TCU", 0x0AD), fr("TCU", 0x0AE), fr("TCU", 0x394));
            let (m11_ist, m11_soll) = (sig(&m11, 22, 10) - 509, sig(&m11, 12, 10) - 509);
            let (g_soll, g_st, g_st2, g_zg) = (sig(&g11, 12, 10) - 509, sig(&g11, 56, 2), sig(&g11, 50, 3), sig(&g11, 60, 4));
            let (anheb, sync_ena, sync_n) = (sig(&g12, 40, 8) * 10, sig(&g12, 14, 1), sig(&g12, 15, 9) * 25);
            let nsp = sig(&m20, 40, 8) * 10;
            let gear = sig(&wba, 24, 4);
            let main_state = s.tcu.as_ref().unwrap().dq200().unwrap().read_live_state().main_state;
            if main_state == 9 && state9_s.is_none() && phase >= Phase::HoldD {
                state9_s = Some(t);
            }
            if launch_s.is_none() && v > 2.0 {
                launch_s = Some(t);
            }
            if phase >= Phase::HoldD && gear_seq.last().map(|g| g.1) != Some(gear) {
                gear_seq.push((t, gear));
            }
            if phase == Phase::Creep {
                idle_raise_max = idle_raise_max.max(anheb);
                nsp_max_creep = nsp_max_creep.max(nsp);
            }
            sync_reqs += (sync_ena == 1) as u32;
            let o = &s.actuators.ecu;
            let sparks: Vec<f64> = o.ignition.iter().flatten().map(|c| c.spark_btdc_deg).collect();
            let spark = if sparks.is_empty() { f64::NAN } else { sparks.iter().sum::<f64>() / sparks.len() as f64 };
            let cut = (0..4).filter(|&c| o.injection[c].iter().all(Option::is_none) && o.port_injection[c].iter().all(Option::is_none)).count();
            if g_st != 0 {
                let iv = open.get_or_insert_with(|| {
                    let (e, m, sp) = hist.front().copied().unwrap_or((tel.torque_brake_nm, m11_ist, spark));
                    Intervention { t0: t, gear, target_gear: g_zg, req_min_nm: i64::MAX, ecu_before_nm: m, ecu_min_nm: i64::MAX, eng_before_nm: e, eng_min_nm: f64::MAX, spark_before: sp, spark_min: f64::MAX, ..Default::default() }
                });
                iv.t1 = t;
                iv.req_min_nm = iv.req_min_nm.min(g_soll);
                iv.ecu_min_nm = iv.ecu_min_nm.min(m11_ist);
                iv.eng_min_nm = iv.eng_min_nm.min(tel.torque_brake_nm);
                if spark.is_finite() {
                    iv.spark_min = iv.spark_min.min(spark);
                }
                iv.cyl_cut_max = iv.cyl_cut_max.max(cut);
            } else if let Some(iv) = open.take() {
                interventions.push(iv);
            }
            hist.push_back((tel.torque_brake_nm, m11_ist, spark));
            if hist.len() > 10 {
                hist.pop_front();
            }
            let traced = (tr0..tr1).contains(&t);
            if ((t * 100.0).round() as u64).is_multiple_of(100) || traced {
                let row = format!(
                    "{t:6.2} {:8} | {:5.0} rpm {:5.1} km/h | gear {:2} zg {:2} main {:2} | slip K1 {:5.0} K2 {:5.0} rpm | eng {:4.0} Nm, ECU ist {:4} soll {:4} | TCU req {:4} st {} st2 {} | idle +{:4} N_SP {:4} | sync {} {:4} | spark {:5.1} | ped {:3.0} brk {:3.0}",
                    format!("{phase:?}"),
                    tel.rpm,
                    v,
                    gear,
                    g_zg,
                    main_state,
                    tel.clutch_slip_rpm[0],
                    tel.clutch_slip_rpm[1],
                    tel.torque_brake_nm,
                    m11_ist,
                    m11_soll,
                    g_soll,
                    g_st,
                    g_st2,
                    anheb,
                    nsp,
                    sync_ena,
                    sync_n,
                    spark,
                    s.plant.driver.pedal_pct,
                    s.plant.driver.brake_pct,
                );
                println!("{row}");
                if !traced {
                    rows.push(row);
                }
                if std::env::var_os("SIMEMU_DRIVE_HYD").is_some() {
                    let m = &s.plant.driveline.mech;
                    println!(
                        "        H sup {:.1} bar pump {:.0} rpm duty {:?} vb {:.1} V {:.1} cm3/s used {:?} cm3/s sub {:?} cl {:?} bar x {:?} mm I {:?} A",
                        m.supply_pa / 1e5,
                        m.pump.omega * 30.0 / std::f64::consts::PI,
                        s.actuators.tcu.pump_motor_duty.map(|d| (d * 100.0).round()),
                        tel.battery_v,
                        m.pump_flow_m3_s * 1e6,
                        m.consumer_m3_s.map(|q| (q * 1e7).round() / 10.0),
                        m.sub_pa.map(|p| (p / 1e4).round() / 10.0),
                        m.clutch_pa.map(|p| (p / 1e4).round() / 10.0),
                        m.clutch_x_m.map(|x| (x * 1e4).round() / 10.0),
                        m.solenoids.iter().take(8).map(|s| (s.current_a * 100.0).round() / 100.0).collect::<Vec<_>>(),
                    );
                }
                if !watch.is_empty() {
                    let dspr = &s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().dspr;
                    let w: Vec<String> = watch.iter().map(|&(a, n, words)| {
                        let o = (a & 0xF_FFFF) as usize;
                        if words {
                            format!("{a:X}={:?}", (0..n).map(|k| i16::from_le_bytes([dspr[o + 2 * k], dspr[o + 2 * k + 1]])).collect::<Vec<_>>())
                        } else {
                            format!("{a:X}={:02X?}", &dspr[o..o + n])
                        }
                    }).collect();
                    println!("        W {}", w.join(" "));
                }
                if traced && std::env::var_os("SIMEMU_DRIVE_CYL").is_some() {
                    println!(
                        "        MAP {:.0} kPa rail {:.1} MPa air {:.0} mg fuel {:?} lam {:?} imep {:?} ign °ATDC plant {:?} ECU {:?} | inj {:?} mpi {:?}",
                        tel.map_kpa,
                        tel.rail_mpa,
                        tel.air_mg_stroke,
                        tel.fuel_mg_stroke.map(|x| (x * 10.0).round() / 10.0),
                        tel.lambda_cyl.map(|x| (x * 100.0).round() / 100.0),
                        tel.imep_bar.map(|x| (x * 10.0).round() / 10.0),
                        s.plant.engine.cyl.iter().map(|c| (c.last.ign_deg * 10.0).round() / 10.0).collect::<Vec<_>>(),
                        o.ignition.map(|c| c.map(|c| (-c.spark_btdc_deg * 10.0).round() / 10.0)),
                        o.injection[0].iter().flatten().map(|p| (p.soi_btdc_deg.round(), (p.width_ms * 100.0).round() / 100.0)).collect::<Vec<_>>(),
                        o.port_injection[0].iter().flatten().map(|p| (p.soi_btdc_deg.round(), (p.width_ms * 100.0).round() / 100.0)).collect::<Vec<_>>(),
                    );
                }
            }
        }
        if let Some(iv) = open.take() {
            interventions.push(iv);
        }
        let wall_s = wall.elapsed().as_secs_f64();
        println!("\n=== vehicle_drive_scenario ({:?}, {:?}) ===", s.board.backend(), s.plant.bench);
        println!("drive {t:.1} s simulated in {wall_s:.1} s wall (session boot excluded) -> RTF {:.2}", t / wall_s);
        println!("phases: {:?}", phases.iter().map(|(t, p)| format!("{t:.2} {p:?}")).collect::<Vec<_>>());
        println!("engine running on its own at {running_s:?} s; min rpm while driving {min_rpm_driving:.0}; stall (< 100 rpm for 0.3 s) {stall_s:?}; dips below 400 rpm (t, min): {dips:?}");
        println!("TCU main state 9 in D at {state9_s:?} s, launch (> 2 km/h) at {launch_s:?} s, max {max_kmh:.1} km/h");
        // mean deceleration from the brake application to rest
        let brake_g = brake_from.zip(stop_s).map(|((t0, v0), t1)| (v0 - 0.5) / 3.6 / (t1 - t0) / 9.81);
        println!("creep: min {min_rpm_creep:.0} rpm; braking ({brake} % pedal) from {brake_from:?} (s, km/h) to rest at {stop_s:?} s: mean {:.2} g", brake_g.unwrap_or(f64::NAN));
        println!("creep: TCU idle-speed raise (Getriebe_12 GE_Anheb_Solldrehz_Leerlauf) max {idle_raise_max} rpm, ECU idle set point (Motor_20) max {nsp_max_creep} rpm");
        println!("gears (WBA_03 WBA_eing_Gang_02): {:?}", gear_seq.iter().map(|(t, g)| format!("{t:.2}:{g}")).collect::<Vec<_>>());
        println!("speed-sync requests (Getriebe_12 GE_Freigabe_Synchro = 1): {sync_reqs} samples of 10 ms");
        println!("torque interventions (Getriebe_11 GE_MMom_Status != 0): {}", interventions.len());
        for iv in &interventions {
            println!(
                "  {:6.2}..{:6.2} s gear {} -> {} | TCU request min {:4} Nm | ECU MO_Mom_Ist {:4} -> {:4} Nm | plant {:5.0} -> {:5.0} Nm | spark {:5.1} -> {:5.1} °BTDC | cut cylinders max {}",
                iv.t0, iv.t1, iv.gear, iv.target_gear, iv.req_min_nm, iv.ecu_before_nm, iv.ecu_min_nm, iv.eng_before_nm, iv.eng_min_nm, iv.spark_before, iv.spark_min, iv.cyl_cut_max
            );
        }
        println!("timeline (1 s):");
        for r in &rows {
            println!("  {r}");
        }
        let ecu_resets = s.board.hw().reset_count - ecu_resets0;
        let tcu_resets = s.tcu.as_ref().unwrap().dq200().unwrap().cpu.hw().reset_log.len() - tcu_resets0;
        println!("resets: ECU {ecu_resets}, TCU {tcu_resets}; ECU error {:?}, TCU error {:?}", s.ecu_error, s.tcu_error);

        if std::env::var("SIMEMU_TCU_WWATCH").is_ok() {
            let mut seen = std::collections::HashSet::new();
            for &(pc, ra, a, v, n) in simemu_tc1791::cpu_hooks::WRITE_WATCH.lock().unwrap().iter() {
                if seen.insert((pc, ra, a, v)) {
                    println!("wwatch insn {n} pc {pc:08X} a11 {ra:08X} [{a:08X}] <- {v:X}");
                }
            }
        }
        // ---- fault memories, decoded with each firmware's own tables ----
        let ecu_dtcs = read_ecu_dtcs(&mut s);
        println!("ECU fault memory: {} records ({}); image tables: {} errors", ecu_dtcs.len(), s.uds_ecu.status.note, ecu_err.len());
        for &(dtc, st) in ecu_dtcs.iter().filter(|d| d.1 & 0x2F != 0) {
            // VW UDS DTC = 0x00 + customer DTC (16 bit) — e.g. MCC_ATIC_139_LS ↔ 0x5653 (docs/SIMOS18_BOARD_HW.md §3)
            let names: Vec<String> = ecu_err.iter().filter(|e| e.2 as u32 == dtc & 0xFFFF).map(|e| format!("{} (OBD {:04X})", e.0, e.1)).collect();
            println!("  ECU DTC {dtc:06X} status {st:02X}: {names:?}");
        }
        let tcu_flash = crate::paths::read_firmware(&root, DQ200_FLASH).unwrap_or_default();
        let tcu_event = |dtc: u32| -> Option<usize> {
            let b = (simemu_tc1791::tcu_dq200::DTC_VAG_TABLE - 0x8000_0000) as usize;
            (0..simemu_tc1791::tcu_dq200::DTC_EVENTS).find(|&i| tcu_flash.get(b + 2 * i..b + 2 * i + 2).is_some_and(|w| u16::from_le_bytes([w[0], w[1]]) as u32 == dtc & 0xFFFF))
        };
        let tcu_dtcs = s.tcu.as_mut().unwrap().dq200_mut().unwrap().read_dtcs().expect("TCU UDS 19 02");
        println!("TCU fault memory: {} records (event = index in the firmware's DTC table 0x80032AD0):", tcu_dtcs.len());
        for (dtc, st, sae) in &tcu_dtcs {
            println!("  TCU DTC {dtc:06X} {sae} event {:?} status {st:02X}", tcu_event(*dtc));
        }

        // ---- assertions: firmware decisions observed on CAN / RAM / plant ----
        assert!(s.ecu_error.is_none() && s.tcu_error.is_none(), "board error: {:?} / {:?}", s.ecu_error, s.tcu_error);
        assert!(ecu_resets == 0 && tcu_resets == 0, "reset during the drive: ECU {ecu_resets} TCU {tcu_resets}");
        assert!(running_s.is_some(), "engine did not start");
        assert!(stall_s.is_none(), "engine stalled at {stall_s:?} s");
        assert!(state9_s.is_some(), "TCU not ready in D (main state 9)");
        assert!(launch_s.is_some(), "no drive-away");
        let pos = |g: i64| gear_seq.iter().position(|x| x.1 == g);
        let ups: Vec<Option<usize>> = (1..=5).map(pos).collect();
        assert!(ups.iter().all(Option::is_some) && ups.windows(2).all(|w| w[0] < w[1]), "no 1 -> 5 upshift sequence: {gear_seq:?}");
        assert!(max_kmh >= vmax - 5.0, "top speed {max_kmh:.1} km/h");
        assert!(phases.iter().any(|p| p.1 == Phase::Park), "the car did not come to a stop");
        assert!(
            interventions.iter().any(|iv| iv.ecu_min_nm < iv.ecu_before_nm - 15 && iv.eng_min_nm < iv.eng_before_nm - 15.0),
            "no shift torque intervention realised by the ECU: {interventions:?}"
        );
        // Creep without the engine dip of the one-cycle-late injection (docs/DQ200_IO_MAP.md §12.5: ≈ 190 rpm).
        assert!(min_rpm_creep > 500.0, "engine speed fell to {min_rpm_creep:.0} rpm while creeping");
        // A realistic stop (≥ 0.45 g) without stalling (asserted above).
        assert!(brake_g.is_some_and(|g| g >= 0.45 || brake < 40.0), "braking {brake_g:?} g with {brake} % pedal");
        // Clutch-position (P0810), pressure range/performance (P0841, supervision `0x8006E89E`, event 154) and low-pressure
        // (P0868) faults were plant mismatches (clutch actuator, valve, hydraulic balance); none may be stored.
        let bad = ["P0810", "P0841", "P0868"];
        assert!(tcu_dtcs.iter().all(|d| !bad.contains(&d.2.as_str())), "clutch-position / pressure faults on the TCU: {tcu_dtcs:X?}");
    }
}
