//! Network fault scenario: the rest of the car's CAN (`simemu_bridge::MqbChassisEncoder`, the modules that are not
//! emulated) must leave no lost-communication / invalid-data DTC in the ECU and the TCU after a drive.
//!
//! `SIMEMU_RESTBUS_BUILD` = bench build preset (default `bench_s50`, the studio's baseline; `octavia_fl` = DQ200 car,
//! `octavia_is38` = DQ250 4WD car); the ECU runs the stock image of the build's engine (`session::stock_ecu_image`), as the
//! studio does. Drive: start, idle in P, D on the brake, creep, 30 % pedal to 40 km/h, coast, brake to a stop, P; then
//! `19 02 0F` on both units.
//!
//! The memory is not cleared before the drive: the DQ250 starts from an erased EEPROM that the studio prepares
//! (`studio::dq250_prep`), everything in its memory is from this run; for the ECU and the DQ200 (NVM from the dumps) the
//! memory is read at ignition on, and a record that is pending/confirmed there without failing in this cycle (status bits
//! 2/3 without 0/1) is history. Clearing works as on a car — OBD `04` on the functional address (`clear_dtc_scenario`,
//! `MILESTONE.md` §1.1); UDS `14` is not implemented by any of the firmwares.
//!
//! Network DTC = an OBD U-code by the module's own table (ECU: `tune_check::ecu_error_table`; DQ250: `tcu_dq250::dtc_table`;
//! DQ200: its CAL table, plus the VAG numbers the DQ250 table marks as U-codes — 0x29BB / 0x52E4 / 0x52E7, which the DQ200
//! table reports as P0602). Asserted after the drive: no network DTC that failed in this cycle (status bit 0 or 1) or became
//! pending/confirmed during the run (bit 2 or 3, not history); the ECU's outside air temperature (OBD PID 0x46, from
//! Kombi_02 / Gateway_72) within 1 °C of the plant's ambient.
//!
//! Run: `SIMEMU_CPU=native cargo test --release -p simemu-cli network_dtc_scenario -- --ignored --nocapture`.

#[cfg(test)]
mod tests {
    use crate::studio::session::{stock_ecu_image, StudioSession, UdsRequest, S50_FLASH};
    use crate::studio::tcu_io::TcuBoard;
    use simemu_physics::{BenchMode, Selector};
    use std::path::PathBuf;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    /// The bench build of `preset`, its DQ250 image replaced by `SIMEMU_DQ250_IMAGE` (a file name as in the studio) if set.
    /// For `octavia_is38` (1.8 TSI `CJSA` `8V0906264H` `SC8O40`), defaults to `0D9300012F` (`SW 4930`,
    /// `DQ250_FLASH`) when `SIMEMU_DQ250_IMAGE` is unset, matching `tcu_io::cjsa_dq250_session` and `dq250_prep`:
    /// `8V0906264H` configures CAN RX MO 44 for `Getriebe_15` (`0x0B1`) and monitors its 2.5 s timeout in `0x801cbac6`
    /// (`U0101` `005496` `COM_TCU_TOUT`, VERIFIED-S), which `0D9300012F` (`SW 4930`) transmits at MultiCAN TX table entry 15
    /// (`0x800c0ab6`, built by `0x8007e86c`, VERIFIED-S), whereas Audi S3 2.0 TFSI `0D9300011R` (`SW 4319`, `0x800d9ad8`) has
    /// only 18 TX entries and does not transmit `0x0B1` (VERIFIED-S).
    fn audit_build(preset: &str) -> simemu_physics::bench_config::BenchConfig {
        let mut build = simemu_physics::bench_config::preset(preset).expect("preset");
        if let Ok(i) = std::env::var("SIMEMU_DQ250_IMAGE") {
            build.tcu_image = Some(i);
        } else if matches!(preset, "octavia_is38" | "cjsa_is38_dq250" | "a3_8v_quattro") {
            build.tcu_image = Some(crate::studio::session::DQ250_012F.into());
        }
        build
    }

    /// One DTC record: VAG/customer number, status, decoded text, network.
    struct Dtc {
        num: u16,
        status: u8,
        text: String,
        net: bool,
    }

    /// `19 02 0F` of `target` (failed now / this cycle, pending, confirmed), decoded. Not `FF`: SC8O40 answers at most 63
    /// records (255 bytes, VERIFIED-T), most of them only "test not completed" (0x50), which could hide a fault.
    fn dtcs(s: &mut StudioSession, target: &str, dq250_table: &[(u16, u16)]) -> Vec<Dtc> {
        let req = UdsRequest { target: target.into(), service: "raw".into(), data: vec![0x19, 0x02, 0x0F], value: None, rid: None };
        s.uds_request(&req).expect("uds");
        for _ in 0..300 {
            s.step(0.01);
            if !(if target == "ecu" { &s.uds_ecu } else { &s.uds_tcu }).status.pending {
                break;
            }
        }
        dtcs_from_status(s, target, dq250_table)
    }

    /// The records of the last `19 02` answer of `target`, decoded ([`dtcs`]).
    fn dtcs_from_status(s: &StudioSession, target: &str, dq250_table: &[(u16, u16)]) -> Vec<Dtc> {
        let st = if target == "ecu" { s.uds_ecu.status.clone() } else { s.uds_tcu.status.clone() };
        let ecu = crate::studio::tune_check::ecu_errors(s);
        st.dtcs
            .iter()
            .enumerate()
            .map(|(i, d)| {
                let num = d.get(..6).and_then(|x| u32::from_str_radix(x, 16).ok()).unwrap_or(0) as u16;
                let status = d.rsplit("0x").next().and_then(|x| u8::from_str_radix(x, 16).ok()).unwrap_or(0);
                let (code, name) = if target == "ecu" {
                    let e: Vec<_> = ecu.iter().filter(|e| e.2 == num).collect();
                    (e.first().map_or("?".into(), |e| simemu_tc1791::tcu_dq250::sae_text(e.1)), e.iter().map(|e| e.0.as_str()).collect::<Vec<_>>().join("/"))
                } else {
                    let own = st.dtc_sae.get(i).map(|x| x.split(' ').next().unwrap_or("?").to_string()).unwrap_or_else(|| "?".into());
                    let global = dq250_table.iter().find(|e| e.0 == num).map(|e| simemu_tc1791::tcu_dq250::sae_text(e.1));
                    match global {
                        Some(g) if g.starts_with('U') && !own.starts_with('U') => (format!("{g} (own table {own})"), String::new()),
                        _ => (own, String::new()),
                    }
                };
                Dtc { num, status, net: code.starts_with('U'), text: format!("{d} {code} {name}") }
            })
            .collect()
    }

    /// Ambient air temperature as the ECU reports it to a scan tool (OBD PID 0x46 as UDS DID 0xF446: A − 40 °C).
    fn ecu_ambient_c(s: &mut StudioSession) -> Option<f64> {
        let req = UdsRequest { target: "ecu".into(), service: "raw".into(), data: vec![0x22, 0xF4, 0x46], value: None, rid: None };
        s.uds_request(&req).ok()?;
        for _ in 0..100 {
            s.step(0.01);
            if !s.uds_ecu.status.pending {
                break;
            }
        }
        let r: Vec<u8> = s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
        (r.get(..3) == Some(&[0x62, 0xF4, 0x46][..])).then(|| r.get(3).map(|&a| a as f64 - 40.0)).flatten()
    }

    #[test]
    #[ignore = "slow: ECU + TCU + plant co-simulation (DQ250: with the workshop preparation)"]
    fn network_dtc_scenario() {
        let Some(root) = root() else { return eprintln!("SKIP: {S50_FLASH} not found") };
        let preset = std::env::var("SIMEMU_RESTBUS_BUILD").unwrap_or_else(|_| "bench_s50".into());
        let build = audit_build(&preset);
        let image = stock_ecu_image(build.engine);
        if !crate::paths::has_firmware(&root, image) {
            return eprintln!("SKIP: {image} not found (scripts/frf_to_bin.py)");
        }
        let mut s = StudioSession::with_build(&root, crate::studio::bench_build::BuildState::from_config(build)).expect("session");
        s.warm_car();
        s.plant.bench = BenchMode::RoadLoadDyno;
        let dir = std::env::temp_dir().join(format!("simemu_restbus_{}_{preset}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        s.tcu_prep.auto = true;
        s.set_nvm_dir(dir.clone());
        s.load_image(image).expect("image");
        let t0 = std::time::Instant::now();
        while s.tcu_prep.running() && t0.elapsed().as_secs() < 1800 {
            std::thread::sleep(std::time::Duration::from_millis(100));
            s.poll_tcu_prep();
        }
        if let Some(r) = &s.tcu_prep.record {
            println!("DQ250 preparation: {}", r.summary());
        }
        let dq250_table = match s.tcu.as_ref().and_then(TcuBoard::dq250) {
            Some(t) => t.dtcs.clone(),
            None => StudioSession::dq250_image_path(&root, None).ok().and_then(|p| std::fs::read(p).ok()).map(|f| simemu_tc1791::tcu_dq250::dtc_table(&f)).unwrap_or_default(),
        };
        println!("ECU {} | TCU {} | build {preset}", s.image_label, s.tcu.as_ref().map_or("-".into(), |t| t.sw_id()));

        let d = &mut s.plant.driver;
        (d.ignition_on, d.selector, d.brake_pct) = (true, Selector::P, 60.0);
        s.step(1.0);
        // history: pending/confirmed at ignition on without failing in this cycle
        let mut history: Vec<(&str, u16)> = Vec::new();
        for t in ["ecu", "tcu"] {
            for d in dtcs(&mut s, t, &dq250_table) {
                println!("  {t} at ignition on: {}", d.text);
                if d.status & 0x0C != 0 && d.status & 0x03 == 0 {
                    history.push((t, d.num));
                }
            }
        }
        s.plant.driver.start_button = true;
        for _ in 0..500 {
            s.step(0.01);
            let t = s.plant.telemetry();
            if t.rpm > 600.0 && t.starter_current_a <= 0.0 {
                break;
            }
        }
        s.plant.driver.start_button = false;
        s.step(5.0);
        s.plant.driver.selector = Selector::D;
        s.step(2.0);
        s.plant.driver.brake_pct = 0.0;
        s.step(3.0);
        s.plant.driver.pedal_pct = 30.0;
        for _ in 0..2500 {
            s.step(0.01);
            if s.plant.telemetry().roller_kmh >= 40.0 {
                break;
            }
        }
        let vmax = s.plant.telemetry().roller_kmh;
        s.plant.driver.pedal_pct = 0.0;
        s.step(3.0);
        s.plant.driver.brake_pct = 40.0;
        for _ in 0..1500 {
            s.step(0.01);
            if s.plant.telemetry().roller_kmh < 0.5 {
                break;
            }
        }
        s.plant.driver.selector = Selector::P;
        s.step(3.0);
        println!("drive: {vmax:.0} km/h, t = {:.1} s, rpm {:.0}", s.plant.sim_time_s, s.plant.telemetry().rpm);
        let (amb, taa) = (s.plant.telemetry().ambient_c, ecu_ambient_c(&mut s));
        println!("outside air: plant {amb:.1} °C, ECU (OBD 0x46) {taa:?} °C");

        let mut bad = Vec::new();
        for t in ["ecu", "tcu"] {
            let l = dtcs(&mut s, t, &dq250_table);
            println!("{t}: {} DTC records", l.len());
            // DQ250 UDS 19 02 sets bit 1 (0x02, testFailedThisOperationCycle) on the first raw un-debounced report
            // before the 100-cycle (2.0 s) debounce queue 0xD00046E4 (docs/DQ250_FIRMWARE_SPEC.md §11.1, §12.3).
            let fail_mask = if t == "tcu" { 0x01 } else { 0x03 };
            for d in &l {
                let hist = history.contains(&(t, d.num));
                let now = d.status & fail_mask != 0 || (d.status & 0x0C != 0 && !hist);
                let tag = match (d.net, now, hist) {
                    (true, true, _) => "NETWORK",
                    (true, false, true) => "network (history)",
                    (true, false, false) => "network (not failed)",
                    (false, true, _) => "fault",
                    (false, false, true) => "history",
                    _ => "-",
                };
                println!("  {t} {tag:21} {}", d.text);
                if d.net && now {
                    bad.push(format!("{t} {}", d.text));
                }
            }
        }
        let _ = std::fs::remove_dir_all(&dir);
        assert!(bad.is_empty(), "network DTCs after the drive: {bad:#?}");
        assert!(taa.is_some_and(|t| (t - amb).abs() <= 1.0), "ECU outside air {taa:?} °C, plant {amb:.1} °C");
    }

    /// Fault audit of the studio path: the whole car of a bench build with the stock ECU image (`SIMEMU_IMAGE` overrides),
    /// the build's TCU, studio preparation, then start (warm, or after a 12 h soak at `SIMEMU_AUDIT_COLD` °C), idle, D/R
    /// creep, 30 % to 50 km/h, kickdown to 110 km/h, coast, 40 % cruise, brake to a stop, idle in D and P. `19 02 0F` of both
    /// units at ignition on (history) and after the drive. Asserted: no record that failed in this cycle or became
    /// pending/confirmed in it, any kind (not only network), no stall, no ECU/TCU reset or trap.
    ///
    /// `SIMEMU_AUDIT_BUILD` = bench build preset (default `bench_s50`); `SIMEMU_DQ250_IMAGE` = DQ250 image of the build.
    #[test]
    #[ignore = "slow: ECU + TCU + plant co-simulation"]
    fn fault_audit_scenario() {
        let Some(root) = root() else { return eprintln!("SKIP: {S50_FLASH} not found") };
        let preset = std::env::var("SIMEMU_AUDIT_BUILD").unwrap_or_else(|_| "bench_s50".into());
        let cold: Option<f64> = std::env::var("SIMEMU_AUDIT_COLD").ok().and_then(|v| v.parse().ok());
        let mut build = audit_build(&preset);
        if let Some(a) = cold {
            build.ambient.temp_c = a;
        }
        let image = std::env::var("SIMEMU_IMAGE").unwrap_or_else(|_| stock_ecu_image(build.engine).into());
        if !crate::paths::has_firmware(&root, &image) {
            return eprintln!("SKIP: {image} not found (scripts/frf_to_bin.py)");
        }
        let mut s = StudioSession::with_build(&root, crate::studio::bench_build::BuildState::from_config(build)).expect("session");
        s.plant.bench = BenchMode::RoadLoadDyno;
        let dir = std::env::temp_dir().join(format!("simemu_audit_{}_{preset}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        // `SIMEMU_AUDIT_STATE=dir`: start from the studio state saved there (prepared EEPROMs), saved there after a
        // preparation; every run copies it, so the drive never changes it
        let keep = std::env::var("SIMEMU_AUDIT_STATE").ok().map(PathBuf::from);
        let copy = |from: &std::path::Path, to: &std::path::Path| {
            let _ = std::fs::create_dir_all(to);
            for e in std::fs::read_dir(from).into_iter().flatten().flatten() {
                let _ = std::fs::copy(e.path(), to.join(e.file_name()));
            }
        };
        if let Some(k) = &keep {
            copy(k, &dir);
        }
        s.tcu_prep.auto = true;
        s.set_nvm_dir(dir.clone());
        s.load_image(&image).expect("image");
        let prepared = s.tcu_prep.running();
        let t0 = std::time::Instant::now();
        while s.tcu_prep.running() && t0.elapsed().as_secs() < 1800 {
            std::thread::sleep(std::time::Duration::from_millis(100));
            s.poll_tcu_prep();
        }
        if let Some(r) = &s.tcu_prep.record {
            println!("TCU preparation: {}", r.summary());
        }
        if let (Some(k), true) = (&keep, prepared) {
            copy(&dir, k);
        }
        // both units power up on the soaked car
        match cold {
            Some(a) => {
                s.plant.soak(a);
                (s.plant.driveline.case_c, s.plant.driveline.mech.oil_c) = (a, a);
                s.eot_clock_s = 12.0 * 3600.0;
                s.reset_chassis_soak();
            }
            None => s.warm_car(),
        }
        s.power_cycle_ecu().expect("ECU power-up");
        if s.tcu.is_some() {
            s.power_cycle_tcu();
        }
        let dq250_table = match s.tcu.as_ref().and_then(TcuBoard::dq250) {
            Some(t) => t.dtcs.clone(),
            None => StudioSession::dq250_image_path(&root, None).ok().and_then(|p| std::fs::read(p).ok()).map(|f| simemu_tc1791::tcu_dq250::dtc_table(&f)).unwrap_or_default(),
        };
        let units: Vec<&str> = if s.tcu.is_some() { vec!["ecu", "tcu"] } else { vec!["ecu"] };
        println!("ECU {} | TCU {} | build {preset} | ambient {:.1} °C | coolant {:.1} °C", s.image_label, s.tcu.as_ref().map_or("-".into(), |t| t.sw_id()), s.plant.ambient.temp_c, s.plant.telemetry().coolant_c);

        let d = &mut s.plant.driver;
        (d.ignition_on, d.selector, d.brake_pct) = (true, Selector::P, 60.0);
        s.step(1.0);
        let mut history: Vec<(&str, u16)> = Vec::new();
        for &t in &units {
            for d in dtcs(&mut s, t, &dq250_table) {
                println!("  {t} at ignition on: {}", d.text);
                if d.status & 0x0C != 0 && d.status & 0x03 == 0 {
                    history.push((t, d.num));
                }
            }
        }
        let resets0 = (s.board.hw().reset_log.len(), s.tcu.as_ref().map_or(0, |t| t.cpu().hw().reset_log.len()));

        s.plant.driver.start_button = true;
        let ts = s.plant.sim_time_s;
        while s.plant.sim_time_s - ts < 10.0 {
            s.step(0.01);
            let t = s.plant.telemetry();
            if t.rpm > 600.0 && t.starter_current_a <= 0.0 {
                break;
            }
        }
        s.plant.driver.start_button = false;
        let start_s = s.plant.sim_time_s - ts;
        // (phase, selector, pedal %, brake %, ends when: min time s, speed km/h reached (> 0: rising, < 0: falling below), max time s)
        let plan: &[(&str, Selector, f64, f64, f64, f64, f64)] = &[
            ("idle P", Selector::P, 0.0, 60.0, 30.0, 0.0, 30.0),
            ("D brake", Selector::D, 0.0, 60.0, 3.0, 0.0, 3.0),
            ("creep D", Selector::D, 0.0, 0.0, 5.0, 0.0, 5.0),
            ("stop", Selector::D, 0.0, 40.0, 0.5, -0.3, 10.0),
            ("R brake", Selector::R, 0.0, 60.0, 3.0, 0.0, 3.0),
            ("creep R", Selector::R, 0.0, 0.0, 4.0, 0.0, 4.0),
            ("stop R", Selector::R, 0.0, 40.0, 0.5, -0.3, 10.0),
            ("D brake 2", Selector::D, 0.0, 60.0, 2.0, 0.0, 2.0),
            ("30 %", Selector::D, 30.0, 0.0, 1.0, 50.0, 40.0),
            ("kickdown", Selector::D, 100.0, 0.0, 1.0, 110.0, 30.0),
            ("coast", Selector::D, 0.0, 0.0, 8.0, 0.0, 8.0),
            ("40 %", Selector::D, 40.0, 0.0, 10.0, 0.0, 10.0),
            ("brake", Selector::D, 0.0, 35.0, 0.5, -0.3, 40.0),
            ("idle D", Selector::D, 0.0, 60.0, 5.0, 0.0, 5.0),
            ("park", Selector::P, 0.0, 60.0, 5.0, 0.0, 5.0),
        ];
        let (mut stall, mut vmax, mut rpm_max) = (None::<(f64, &str)>, 0f64, 0f64);
        // `SIMEMU_AUDIT_TRACE=t0,t1`: a row every 10 ms in that window (plant time)
        let trace = std::env::var("SIMEMU_AUDIT_TRACE").ok().and_then(|v| v.split_once(',').and_then(|(a, b)| Some((a.parse().ok()?, b.parse().ok()?)))).unwrap_or((-1.0, -1.0));
        let hex = |x: &str| u32::from_str_radix(x.trim().trim_start_matches("0x"), 16).unwrap();
        let watch: Vec<(u32, usize)> = std::env::var("SIMEMU_TCU_WATCH").unwrap_or_default().split(',').filter(|x| !x.is_empty())
            .map(|a| a.split_once(':').map_or_else(|| (hex(a), 1), |(a, n)| (hex(a), n.parse().unwrap()))).collect();
        // `SIMEMU_TCU_PROBES=addr,..`: TCU code probes (observation only), hits and D15/D4 at the last hit per trace row
        let probes: Vec<u32> = std::env::var("SIMEMU_TCU_PROBES").unwrap_or_default().split(',').filter(|x| !x.is_empty()).map(hex).collect();
        if let Some(t) = s.tcu.as_mut() {
            for &a in &probes {
                t.cpu_mut().add_probe(a).expect("probe");
            }
        }
        for &(name, sel, pedal, brake, t_min, v_end, t_max) in plan {
            let tp = s.plant.sim_time_s;
            let d = &mut s.plant.driver;
            (d.selector, d.pedal_pct, d.brake_pct) = (sel, pedal, brake);
            loop {
                s.step(0.01);
                let tel = s.plant.telemetry();
                let (el, v) = (s.plant.sim_time_s - tp, tel.vehicle_kmh.abs().max(tel.roller_kmh.abs()));
                (vmax, rpm_max) = (vmax.max(v), rpm_max.max(tel.rpm));
                if tel.rpm < 300.0 && stall.is_none() {
                    stall = Some((s.plant.sim_time_s, name));
                }
                if (trace.0..trace.1).contains(&s.plant.sim_time_s) {
                    // observation only: plant gearbox state, TCU coil currents and RAM (`SIMEMU_TCU_WATCH=addr[:n],..`)
                    let o = &s.actuators.tcu;
                    let amps: Vec<String> = (0..11).map(|k| o.solenoid_duty[k].map_or("-".into(), |_| format!("{:.2}", o.solenoid_current_a.map_or(0.0, |c| c[k])))).collect();
                    let w: Vec<String> = s.tcu.as_ref().map_or(Vec::new(), |t| watch.iter().map(|&(a, n)| format!("{a:08X}:{:02X?}", t.cpu().read_mem(a, n).unwrap_or_default())).collect());
                    println!(
                        "{:7.2} {name:9} | {:5.0} rpm | shafts {:?} out {:.0} | line {:.1} clutch {:?} bar {:?} Nm | forks {:?} | coil A {} | {}",
                        s.plant.sim_time_s,
                        tel.rpm,
                        tel.shaft_rpm.map(|x| x.round()),
                        tel.output_shaft_rpm[1],
                        tel.gearbox_hydraulic_bar,
                        tel.clutch_bar.map(|x| (x * 10.0).round() / 10.0),
                        tel.clutch_torque_nm.map(|x| (x * 10.0).round() / 10.0),
                        tel.fork_mm.map(|x| (x * 100.0).round() / 100.0),
                        amps.join(" "),
                        w.join(" ")
                    );
                    for &a in &probes {
                        if let Some(p) = s.tcu.as_ref().and_then(|t| t.cpu().probe(a)) {
                            println!("        probe {a:08X} hits {} d15 {:X} d4 {:X}", p.hits, p.last_regs[15], p.last_regs[4]);
                        }
                    }
                }
                if ((s.plant.sim_time_s * 100.0).round() as u64).is_multiple_of(200) {
                    println!(
                        "{:7.2} {name:9} | {:5.0} rpm {:5.1} km/h | gear {} | clutch {:?} | coolant {:.0} °C ATF {:.0} °C",
                        s.plant.sim_time_s,
                        tel.rpm,
                        v,
                        tel.driving_gear,
                        tel.clutch_bar.map(|x| (x * 10.0).round() / 10.0),
                        tel.coolant_c,
                        tel.gearbox_case_c
                    );
                }
                let reached = (v_end > 0.0 && v >= v_end) || (v_end < 0.0 && v < -v_end);
                if el >= t_max || (el >= t_min && (v_end == 0.0 || reached)) {
                    break;
                }
            }
        }
        let tel = s.plant.telemetry();
        println!(
            "drive: started in {start_s:.1} s, max {vmax:.0} km/h / {rpm_max:.0} rpm, end {:.0} rpm | clutch health {:?} | driveline failures {:?}",
            tel.rpm, tel.clutch_health, tel.driveline_failures
        );

        let mut bad = Vec::new();
        for &t in &units {
            let l = dtcs(&mut s, t, &dq250_table);
            println!("{t}: {} DTC records", l.len());
            let fail_mask = if t == "tcu" { 0x01 } else { 0x03 };
            for d in &l {
                let hist = history.contains(&(t, d.num));
                let now = d.status & fail_mask != 0 || (d.status & 0x0C != 0 && !hist);
                println!("  {t} {:8} {}", if now { "FAULT" } else if hist { "history" } else { "-" }, d.text);
                if now {
                    bad.push(format!("{t} {}", d.text));
                }
            }
        }
        let resets = (s.board.hw().reset_log.len() - resets0.0, s.tcu.as_ref().map_or(0, |t| t.cpu().hw().reset_log.len()) - resets0.1);
        let _ = std::fs::remove_dir_all(&dir);
        println!("AUDIT {preset} cold {cold:?}: {} faults, stall {stall:?}, resets ECU/TCU {resets:?}, errors {:?} {:?}", bad.len(), s.ecu_error, s.tcu_error);
        assert!(stall.is_none(), "engine stalled: {stall:?}");
        assert!(resets == (0, 0) && s.ecu_error.is_none() && s.tcu_error.is_none(), "resets {resets:?}, errors {:?} {:?}", s.ecu_error, s.tcu_error);
        assert!(bad.is_empty(), "faults in this drive: {bad:#?}");
    }

    /// The studio tester clears the fault memory of the ECU and the TCU the way a scan tool does (`clear_dtc`: OBD `04` on
    /// the functional address 0x7DF; UDS `14` is answered `7F 14 11` by all three firmwares): with the ignition on and the
    /// engine off, `19 02 2F` before (records carried in the NVM: ECU DFlash and DQ200 EEPROM dumps, DQ250 on an erased
    /// EEPROM; not `FF`: SC8O40 answers at most 63 records), `clear_dtc` on each target (positive answer `44`), `19 02 2F`
    /// again. Asserted: both clears answered positively and no stored record (status bit 3) is left that is not failing
    /// right now (bit 0) — history is gone; what fails now is stored again by the firmware (e.g. DQ250 P062F on the
    /// erased EEPROM), printed.
    ///
    /// Builds (`SIMEMU_CLEAR_BUILDS`, default `octavia_fl,octavia_is38`): SC8O40 + DQ200_G2 and SC8O40 + DQ250 (default image).
    #[test]
    #[ignore = "slow: ECU + TCU + plant co-simulation"]
    fn clear_dtc_scenario() {
        let Some(root) = root() else { return eprintln!("SKIP: {S50_FLASH} not found") };
        let builds = std::env::var("SIMEMU_CLEAR_BUILDS").unwrap_or_else(|_| "octavia_fl,octavia_is38".into());
        let mut bad = Vec::new();
        for preset in builds.split(',').filter(|p| !p.is_empty()) {
            let build = audit_build(preset);
            let image = stock_ecu_image(build.engine);
            if !crate::paths::has_firmware(&root, image) {
                eprintln!("SKIP {preset}: {image} not found");
                continue;
            }
            let mut s = StudioSession::with_build(&root, crate::studio::bench_build::BuildState::from_config(build)).expect("session");
            s.warm_car();
            s.load_image(image).expect("image");
            if s.tcu.is_none() {
                bad.push(format!("{preset}: no TCU"));
                continue;
            }
            let dq250_table = s.tcu.as_ref().and_then(TcuBoard::dq250).map(|t| t.dtcs.clone()).unwrap_or_default();
            println!("== {preset}: ECU {} | TCU {}", s.image_label, s.tcu.as_ref().map_or("-".into(), |t| t.sw_id()));
            let d = &mut s.plant.driver;
            (d.ignition_on, d.selector, d.brake_pct) = (true, Selector::P, 60.0);
            s.step(3.0);
            let read = |s: &mut StudioSession, t: &str| -> Vec<Dtc> {
                let req = UdsRequest { target: t.into(), service: "raw".into(), data: vec![0x19, 0x02, 0x2F], value: None, rid: None };
                s.uds_request(&req).expect("uds");
                for _ in 0..300 {
                    s.step(0.01);
                    if !(if t == "ecu" { &s.uds_ecu } else { &s.uds_tcu }).status.pending {
                        break;
                    }
                }
                dtcs_from_status(s, t, &dq250_table)
            };
            let mut before = Vec::new();
            for t in ["ecu", "tcu"] {
                let l = read(&mut s, t);
                let stored: Vec<u16> = l.iter().filter(|d| d.status & 0x08 != 0).map(|d| d.num).collect();
                println!("  {t} before: {} records, stored {}", l.len(), stored.len());
                for d in l.iter().filter(|d| d.status & 0x0F != 0) {
                    println!("    {}", d.text);
                }
                before.push((t, stored));
            }
            for t in ["ecu", "tcu"] {
                let req = UdsRequest { target: t.into(), service: "clear_dtc".into(), data: Vec::new(), value: None, rid: None };
                s.uds_request(&req).expect("uds");
                for _ in 0..500 {
                    s.step(0.01);
                    if !(if t == "ecu" { &s.uds_ecu } else { &s.uds_tcu }).status.pending {
                        break;
                    }
                }
                let st = if t == "ecu" { &s.uds_ecu.status } else { &s.uds_tcu.status };
                println!("  {t} clear: {} -> {} ({})", st.request, st.response_hex, st.note);
                if !st.response_hex.starts_with("44") {
                    bad.push(format!("{preset} {t}: clear answered {}", st.response_hex));
                }
            }
            s.step(1.0);
            for (t, stored) in &before {
                let l = read(&mut s, t);
                println!("  {t} after: {} records", l.len());
                for d in l.iter().filter(|d| d.status & 0x0F != 0) {
                    println!("    {}", d.text);
                    if d.status & 0x09 == 0x08 {
                        bad.push(format!("{preset} {t} history left after the clear: {}{}", d.text, if stored.contains(&d.num) { " (stored before)" } else { "" }));
                    }
                }
            }
        }
        assert!(bad.is_empty(), "{bad:#?}");
    }

    /// Reproducer of the operator bench workflow on `octavia_fl` (`8V0906264H__0005.bin` + DQ200_G2):
    /// standby with ignition off, then ignition on, start button, short idle in P, release brake and rev gas
    /// to 35 % and 80 % in P, return to idle in P, shift to D, release brake and accelerate at 35 % gas,
    /// brake to a stop, read DTCs (`19 02 2F`) on both ECU and TCU.
    #[test]
    #[ignore = "slow: ECU + TCU + plant co-simulation"]
    fn user_workflow_rev_and_drive_scenario() {
        let Some(root) = root() else { return eprintln!("SKIP: {S50_FLASH} not found") };
        let build = audit_build("octavia_fl");
        let image = stock_ecu_image(build.engine);
        if !crate::paths::has_firmware(&root, image) {
            return eprintln!("SKIP: {image} not found");
        }
        let mut s = StudioSession::with_build(&root, crate::studio::bench_build::BuildState::from_config(build)).expect("session");
        s.warm_car();
        if let Ok(t_c) = std::env::var("SIMEMU_SOAK_C").and_then(|v| v.parse::<f64>().map_err(|_| std::env::VarError::NotPresent)) {
            s.plant.soak(t_c);
            (s.plant.driveline.case_c, s.plant.driveline.mech.oil_c) = (t_c, t_c);
            s.power_cycle_ecu().expect("power_cycle_ecu");
        }
        s.load_image(image).expect("image");
        // Standby with ignition off for 160 s before operator turns on ignition (reproducing the t = 155.58 s session)
        for _ in 0..160 {
            s.step(1.0);
        }
        let iat_standby = s.plant.telemetry().iat_c;
        println!("standby 160 s: iat_c = {iat_standby:.1} °C, immo ECU {}->{} TCU {}->{}", s.immo.ecu_challenges, s.immo.ecu_answers, s.immo.tcu_challenges, s.immo.tcu_answers);
        assert!(iat_standby < 60.0, "IAT heat-soak alarm during standby: {iat_standby:.1} °C");
        let d = &mut s.plant.driver;
        (d.ignition_on, d.selector, d.brake_pct) = (true, Selector::P, 60.0);
        s.step(1.0);
        assert_eq!((s.immo.ecu_challenges, s.immo.ecu_answers), (1, 1), "ECU WFS5 must authorize on first challenge");
        s.plant.driver.start_button = true;
        let ts = s.plant.sim_time_s;
        while s.plant.sim_time_s - ts < 8.0 {
            s.step(0.01);
            let t = s.plant.telemetry();
            if t.rpm > 600.0 && t.starter_current_a <= 0.0 {
                break;
            }
        }
        s.plant.driver.start_button = false;
        let phases: &[(&str, Selector, f64, f64, f64)] = &[
            ("idle_p_1", Selector::P, 0.0, 60.0, 3.0),
            ("rev_p_35", Selector::P, 35.0, 0.0, 4.0),
            ("rev_p_80", Selector::P, 80.0, 0.0, 2.5),
            ("idle_p_2", Selector::P, 0.0, 0.0, 4.0),
            ("d_brake", Selector::D, 0.0, 60.0, 2.0),
            ("d_drive_35", Selector::D, 35.0, 0.0, 6.0),
            ("d_coast", Selector::D, 0.0, 0.0, 2.0),
            ("d_stop", Selector::D, 0.0, 50.0, 4.0),
        ];
        let mut stall = None;
        let mut max_rpm_p = 0.0f64;
        let mut max_kmh_d = 0.0f64;
        let mut max_knock_idle_part = 0.0f64;
        for &(name, sel, pedal, brake, dur) in phases {
            let t0 = s.plant.sim_time_s;
            let d = &mut s.plant.driver;
            (d.selector, d.pedal_pct, d.brake_pct) = (sel, pedal, brake);
            while s.plant.sim_time_s - t0 < dur {
                s.step(0.01);
                let tel = s.plant.telemetry();
                if sel == Selector::P {
                    max_rpm_p = max_rpm_p.max(tel.rpm);
                } else {
                    max_kmh_d = max_kmh_d.max(tel.vehicle_kmh);
                }
                if pedal <= 40.0 {
                    for &k in &tel.knock_integral {
                        max_knock_idle_part = max_knock_idle_part.max(k);
                    }
                }
                if tel.rpm < 400.0 && stall.is_none() {
                    stall = Some((s.plant.sim_time_s, name, tel.rpm));
                }
                if ((s.plant.sim_time_s * 10.0).round() as i64) % 5 == 0 && ((s.plant.sim_time_s * 100.0).round() as i64) % 50 == 0 {
                    println!(
                        "t {:5.1}s {:10} | rpm {:4.0} km/h {:4.1} gear {:?} | thr {:4.1}% wg_duty {:?} wg_pos {:4.1}% | knk {:?} λ {:?} misf {:?} foul {:?}",
                        s.plant.sim_time_s,
                        name,
                        tel.rpm,
                        tel.vehicle_kmh,
                        tel.driving_gear,
                        tel.throttle_pct,
                        s.actuators.ecu.wastegate_duty.map(|x| (x * 100.0).round() / 100.0),
                        tel.wastegate_open_pct,
                        tel.knock_integral.map(|x| (x * 100.0).round() / 100.0),
                        tel.lambda_cyl.map(|x| (x * 100.0).round() / 100.0),
                        tel.combustion.misfire_events,
                        tel.combustion.plug_fouling.map(|x| (x * 1000.0).round() / 1000.0),
                    );
                }
            }
        }
        let ecu_dtcs = dtcs(&mut s, "ecu", &[]);
        let tcu_dtcs = dtcs(&mut s, "tcu", &[]);
        let ecu_bad: Vec<_> = ecu_dtcs.iter().filter(|d| d.status & 0x2F != 0).map(|d| &d.text).collect();
        let tcu_bad: Vec<_> = tcu_dtcs.iter().filter(|d| d.status & 0x2F != 0).map(|d| &d.text).collect();
        let dl_fail = s.plant.telemetry().driveline_failures;
        println!("max_rpm_p={max_rpm_p:.0}, max_kmh_d={max_kmh_d:.1}, max_knock_idle_part={max_knock_idle_part:.3}, stall={stall:?}, dl_fail={dl_fail:?}, ecu_bad={ecu_bad:?}, tcu_bad={tcu_bad:?}");
        assert!(stall.is_none(), "Engine stalled: {stall:?}");
        assert!(max_rpm_p > 2500.0, "RPM did not rise in P: {max_rpm_p:.0}");
        assert!(max_kmh_d > 25.0, "Car did not accelerate in D: {max_kmh_d:.1}");
        assert!(max_knock_idle_part < 0.25, "Knock integral jumped too high at idle/part load: {max_knock_idle_part:.3}");
        assert!(dl_fail.is_empty(), "Driveline failures: {dl_fail:?}");
        assert!(ecu_bad.is_empty(), "ECU DTCs: {ecu_bad:#?}");
        assert!(tcu_bad.is_empty(), "TCU DTCs: {tcu_bad:#?}");
    }
}
