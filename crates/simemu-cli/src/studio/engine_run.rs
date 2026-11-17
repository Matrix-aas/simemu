//! Closed-loop engine start scenario on the studio session (firmware outputs → plant → sensors →
//! firmware): ignition on, brake, start request (KL50 wire), then every stage is reported from
//! observables only (CAN, decoded hardware outputs, plant state, read-only RAM/probe taps, the firmware's
//! fault memory). Run:
//! `SIMEMU_CPU=native cargo test --release -p simemu-cli engine_start_scenario -- --ignored --nocapture`.
//! `SIMEMU_START_S` (default 40 s), `SIMEMU_KL50_END` (default 3 s), `SIMEMU_COLD` (cold-soaked engine),
//! `SIMEMU_ENGINE_OFF_S` (engine-off timer PIC at power-up, default 300 s warm / 12 h cold), `SIMEMU_IMAGE`
//! (e.g. `images/8V0906264H__0005.bin`; RAM-address diagnostics and the ATIC142/heater checks are SC800S50-only),
//! bench configuration `SIMEMU_ENGINE` (engine profile slug, e.g. `cjxc`, `cjsa`) / `SIMEMU_TURBO` (`is12`, `is20`,
//! `is38`) — independent of the image; `SIMEMU_IDLE_TARGETS=1` asserts the firmware's idle targets (N_SP_IS ± 80 rpm
//! for ≥ 30 s, closed loop |STFT| < 10 %), `SIMEMU_ERRM=1` prints the failed ERRM errors every second (SC800S50 RAM),
//! `SIMEMU_ATIC142=1` (lambda IC control words, driver state, sensor/heater state every 0.1 s),
//! test-bench equipment: `SIMEMU_BENCH_SUPPLY=13.8` (laboratory supply on the battery terminals instead of
//! the car battery), `SIMEMU_MOTOR=t0,rpm` (motoring drive coupled to the crank from `t0`),
//! `SIMEMU_ADC=kernel.channel:volts,…` (voltage source on an input the plant does not drive).
//! Read-only diagnostics: `SIMEMU_PROBES=pc,…` (execution counts + last registers; `SIMEMU_PROBE_TRACE=1`
//! prints D4/D5/D8 every 0.1 s with the ETC PWM cell G1.LTC27 and P14 OUT), `SIMEMU_PEEK=addr:len,…`
//! (RAM every 0.1 s), `SIMEMU_RAM_DUMP=dir` (DSPR snapshot every 0.1 s), `SIMEMU_IO_REPORT=1` (GPTA1
//! output lines of the ATIC111 PWM cells, PCP channel starts, every GPTA cell that switched;
//! `SIMEMU_GTC_LOG=k.cell,…` adds their transitions with crank angle), `SIMEMU_UDS_DID=F40F,F405,…` (tester
//! ReadDataByIdentifier, three DIDs per request, one request every 0.1 s in turn), `SIMEMU_CYL_REPORT=1`
//! (plant per-cylinder air/fuel/λ/IMEP, throttle, ATIC111 bridge words).

#[cfg(test)]
mod tests {
    use crate::studio::session::{StudioSession, S50_FLASH};
    use std::path::PathBuf;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    /// Read-only firmware observables (SC800S50 RAM, docs/SIMOS18_WFS5_RE.md §5, docs/SIMOS18_GPTA_IO_RE.md §12).
    const IMMO_ANSWER_OK: u32 = 0xD001_0428; // both MAC halves and S bit 0 of the cluster's 0x011 answer
    const IMMO_RX: u32 = 0x8009_B644; // CAN receive handler of the 0x011 answer
    const OBD_N: u32 = 0xD001_2408; // PID 0x0C buffer, rpm·4
    const VBOOST: u32 = 0xD001_9580; // injector boost voltage, 0.5 V/bit
    /// PCP channels of the injection banks (G0.GTC20/21 -> injectors of cylinders 1/3, G0.GTC22/23 -> 4/2) and
    /// of the HPFP valve N276 (G1.GTC26).
    const PCP_INJ: [usize; 2] = [0x29, 0x28];
    const PCP_VCV: usize = 0x0E;
    /// A/D-monitoring error flags of the level-2 monitoring (ECM2 17.5, FR p.4939): LV_ERR_CONV_MON /
    /// LV_ERR_VP_ALV_CHK_MON; set, they cut every cylinder (engine_io::ADC_MON_PWM_CELL docs).
    const ERR_CONV_MON: u32 = 0xD001_C033;
    const ERR_VP_ALV_CHK_MON: u32 = 0xD001_C097;
    /// ATIC142 driver struct (`*(0xD0000884)`, docs/SIMOS18_BOARD_HW.md §6): +0x26 state, +0x27 configuration
    /// state (STATE_LAMB_IC_CONF_INI), +0x2A / +0x2B communication / configuration error flags.
    const ATIC142_DEV: u32 = 0xD000_832C;
    /// EGCP I/O image written by ACTION_INFR_GetWrafMeasLin (0x800A5878, destinations a4..a7 at entry, VERIFIED-T):
    /// IPLSL_MMV_SEG (366.39 nA/bit), V_R_IT_LSL, VSLSL, VPLSL (152.588 µV/bit), i16 each.
    const WRAF_IO: u32 = 0xD000_040C;

    #[derive(Default, Debug)]
    struct Stages {
        ecu_resets: u32,
        immo_challenges: u64,
        relays_on_s: Option<f64>,
        starter_on_s: Option<f64>,
        starter_off_s: Option<f64>,
        crank_s: Option<f64>,
        crank_150_s: Option<f64>,
        fuel_pump_on_s: Option<f64>,
        first_spark_s: Option<f64>,
        rail_4mpa_s: Option<f64>,
        first_injection_s: Option<f64>,
        running_s: Option<f64>,
        max_rpm: f64,
        sparks: [u32; 4],
        injections: [u32; 4],
        min_cranking_v: f64,
        /// Longest interval the engine ran on its own (starter off, > 600 rpm), s.
        self_run_s: f64,
        /// Plant samples every 0.1 s inside the idle window [`IDLE_WINDOW_S`]: rpm, exhaust λ, throttle °,
        /// rail MPa.
        idle: Vec<[f64; 4]>,
        /// rpm every 0.1 s inside [`LONG_IDLE_S`].
        long_idle: Vec<f64>,
        /// OBD (UDS 0x22, read-only, every 0.5 s): fuel-system status PID 0x03 seen (bit set per status value),
        /// short-term fuel trims while in closed loop (%), firmware λ (PID 0x34) vs plant cell λ once the sensor
        /// reads (pump controller on).
        fuel_status_seen: u8,
        stft_closed_loop: Vec<f64>,
        lambda_fw_vs_plant: Vec<(f64, f64)>,
        /// Heater duty decoded from the GPTA cells vs the firmware's own duty bytes, samples [up, down] that agree /
        /// were compared.
        heater_match: [u32; 2],
        heater_checks: [u32; 2],
        /// Wide-band tip temperature every 0.1 s inside [`LONG_IDLE_S`].
        lsu_tip_c: Vec<f64>,
        /// Latest Motor_20 (0x121) frame the ECU sent and the latest firmware measured values (UDS 0x22).
        motor20: Vec<u8>,
        fw: FwValues,
        /// Inside [`FW_IDLE_WINDOW`], every 0.5 s: plant rpm, firmware idle set-point (Motor_20), FUP, FUP_SP (MPa).
        fw_idle: Vec<(f64, f64, f64, f64)>,
        /// Every 0.5 s while the engine runs on its own (from 3 s): time, firmware FUP, FUP_SP, plant rail (MPa).
        fup_track: Vec<(f64, f64, f64, f64)>,
        /// First time the firmware reported an MPI pulse width (TI_1_MES_MPI > 0); port fuel seen in the plant.
        mpi_released_s: Option<f64>,
        port_fuel_seen: bool,
    }

    #[derive(Default, Debug)]
    struct FwValues {
        fup_sp_mpa: f64,
        fup_mpa: f64,
        ti_mpi_ms: f64,
        toil_c: f64,
        tia_c: f64,
        pump_vol_pct: f64,
        maf_g_s: f64,
        map_kpa: f64,
        load_pct: f64,
        tq_eng_nm: f64,
        tq_req_nm: f64,
        iga_deg: f64,
    }

    impl Stages {
        /// Idle speed set-point the firmware broadcasts: Motor_20 (0x121) `MO_Solldrehz_Leerlauf` = N_SP_IS, 8 bit,
        /// 10 rpm/bit, 0xFF = error (FR p.18057/18069, ByPass_Motor_20_MO_Solldrehz_Leerlauf).
        fn n_sp_is(&self) -> f64 {
            match self.motor20.get(MOTOR20_N_SP_IS_BYTE) {
                Some(&b) if b != 0xFF => b as f64 * 10.0,
                _ => f64::NAN,
            }
        }
    }

    /// Byte of Motor_20 carrying MO_Solldrehz_Leerlauf (VERIFIED-T: see `engine_start_scenario` output).
    const MOTOR20_N_SP_IS_BYTE: usize = 5;
    /// Warm idle checked against the firmware's own targets (≥ 30 s of the 40 s run).
    const FW_IDLE_WINDOW: std::ops::Range<f64> = 10.0..40.0;

    /// ERRM status array of SC800S50 (docs/SIMOS18_BOARD_HW.md §7): 2 bytes per IDX_ERR at 0xD00125E4; IDX_ERR = position
    /// of `c_err_clas_<name>` in its (contiguous, UBYTE) DAMOS table. Read-only. Failed = byte 1 bit 3 (HYP from observed
    /// values: confirmed failures read 1F 7B, a debounce in progress 08 03, tested OK 08 05 — bit 1 is also set while a
    /// test is still running).
    const ERRM_STATUS: u32 = 0xD001_25E4;

    fn errm_failed(s: &StudioSession) -> Vec<String> {
        let mut cls: Vec<(u32, &str)> = s.damos.parameters.iter().filter(|p| p.id.starts_with("c_err_clas_")).map(|p| (p.cpu_address, &p.id[11..])).collect();
        cls.sort();
        let n = cls.windows(2).position(|w| w[1].0 != w[0].0 + 1).map_or(cls.len(), |k| k + 1);
        let st = s.board.read_mem(ERRM_STATUS, 2 * n).unwrap_or_default();
        (0..n).filter(|&i| st.get(2 * i + 1).is_some_and(|b| b & 0x08 != 0)).map(|i| cls[i].1.to_uppercase()).collect()
    }

    /// Data bytes of each DID in a positive ReadDataByIdentifier answer (`None` if it is not the answer to them).
    fn did_values(r: &[u8], dids: &[(u16, usize)]) -> Option<Vec<Vec<u8>>> {
        if r.first() != Some(&0x62) {
            return None;
        }
        let mut i = 1;
        let mut out = Vec::new();
        for &(d, n) in dids {
            if r.get(i..i + 2)? != d.to_be_bytes() {
                return None;
            }
            out.push(r.get(i + 2..i + 2 + n)?.to_vec());
            i += 2 + n;
        }
        Some(out)
    }

    /// Window after the start in which the firmware holds the engine on its own (after-start idle).
    const IDLE_WINDOW_S: std::ops::Range<f64> = 5.0..10.0;
    /// Lasting idle: the engine must still idle here (≥ 30 s on its own after the start).
    const LONG_IDLE_S: std::ops::Range<f64> = 30.0..40.0;
    /// Firmware duty bytes of the λ heaters (SC800S50: ASW 0x8019B258 / 0x8019B2CC, docs/SIMOS18_BOARD_HW.md §6):
    /// upstream 0xD000C56C, downstream 0xD000C56B; duty = (b + 1)/256.
    const LSH_DUTY_BYTES: [u32; 2] = [0xD000_C56C, 0xD000_C56B];

    /// Warm engine (hot restart): coolant/oil at 90 °C.
    fn warm_engine(p: &mut simemu_physics::VirtualPlant) {
        p.soak(90.0);
    }

    /// ReadDataByIdentifier of up to three OBD DIDs (0xF4xx = mode-01 PID xx) through the session's tester;
    /// returns the data bytes of each DID (`len` = data length of each).
    fn read_obd(s: &mut StudioSession, dids: &[(u16, usize)]) -> Vec<Vec<u8>> {
        let mut req = vec![0x22];
        dids.iter().for_each(|(d, _)| req.extend(d.to_be_bytes()));
        let f = s.uds_ecu.request("ReadDataByIdentifier", &req);
        s.board.inject_can_rx(&f);
        for _ in 0..100 {
            s.step(0.01);
            if !s.uds_ecu.status.pending {
                break;
            }
        }
        let r: Vec<u8> = s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
        assert_eq!(r.first(), Some(&0x62), "ReadDataByIdentifier {req:02X?}: {:?}", s.uds_ecu.status.response_hex);
        let mut i = 1;
        dids.iter()
            .map(|&(d, n)| {
                assert_eq!(&r[i..i + 2], &d.to_be_bytes(), "DID order in {r:02X?}");
                i += 2 + n;
                r[i - n..i].to_vec()
            })
            .collect()
    }

    /// Analog sensor characteristics (simemu_physics::sensors, derived from the firmware's own conversion
    /// tables) round trip through the unmodified firmware: plant value → sensor voltage → ADC → the
    /// firmware's linearisation → its OBD value (UDS 0x22 F4xx), engine stopped at ignition on.
    #[test]
    fn sensor_characteristics_round_trip_through_the_firmware() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let mut s = StudioSession::new(&root).expect("session");
        warm_engine(&mut s.plant);
        s.plant.engine.rail_pa = 8.0e6;
        s.plant.driver.ignition_on = true;
        for _ in 0..150 {
            s.step(0.01);
        }
        let t = s.plant.telemetry();
        let v = read_obd(&mut s, &[(0xF405, 1), (0xF40F, 1), (0xF40B, 1)]);
        let w = read_obd(&mut s, &[(0xF433, 1), (0xF423, 2), (0xF470, 10)]);
        let x = read_obd(&mut s, &[(0xF411, 1), (0xF449, 1), (0xF44A, 1)]);
        let t2 = s.plant.telemetry();
        let (clt, iat) = (v[0][0] as f64 - 40.0, v[1][0] as f64 - 40.0);
        let (map, baro) = (v[2][0] as f64, w[0][0] as f64);
        let rail = u16::from_be_bytes([w[1][0], w[1][1]]) as f64 * 0.01;
        let put = u16::from_be_bytes([w[2][3], w[2][4]]) as f64 / 32.0;
        let pct = |b: u8| b as f64 * 100.0 / 255.0;
        let (tps1, pvs1, pvs2) = (pct(x[0][0]), pct(x[1][0]), pct(x[2][0]));
        let (s_tps1, _) = simemu_physics::sensors::throttle_sensors_v(t2.throttle_pct * 0.9);
        let (s_pvs1, s_pvs2) = simemu_physics::sensors::pedal_sensors_v(s.plant.driver.pedal_pct);
        eprintln!(
            "firmware: CLT {clt} °C IAT {iat} °C MAP {map} kPa baro {baro} kPa rail {rail:.2} MPa PUT {put:.2} kPa TPS1 {tps1:.1} % PVS {pvs1:.1}/{pvs2:.1} %\n\
             plant:    CLT {:.1} °C IAT {:.1} °C MAP {:.1} kPa baro {:.1} kPa rail {:.2} MPa PUT {:.2} kPa TPS1 {:.1} % PVS {:.1}/{:.1} %",
            t.coolant_c, t.iat_c, t.map_kpa, t.baro_kpa, t2.rail_mpa, t.put_kpa, s_tps1 / 5.0 * 100.0, s_pvs1 / 5.0 * 100.0, s_pvs2 / 5.0 * 100.0
        );
        // OBD resolution: 1 °C, 1 kPa, 10 kPa (rail), 1/32 kPa (PUT), 1/255 (TPS / pedal = sensor voltage / 5 V).
        assert!((clt - t.coolant_c).abs() <= 1.5, "coolant {clt} vs {}", t.coolant_c);
        assert!((iat - t.iat_c).abs() <= 1.5, "intake air {iat} vs {}", t.iat_c);
        assert!((map - t.map_kpa).abs() <= 1.5 && (baro - t.baro_kpa).abs() <= 1.5, "MAP {map} / baro {baro} vs {} / {}", t.map_kpa, t.baro_kpa);
        assert!((put - t.put_kpa).abs() <= 1.0, "PUT {put} vs {}", t.put_kpa);
        assert!((rail - t2.rail_mpa).abs() <= 0.15, "rail {rail} vs {}", t2.rail_mpa);
        assert!((tps1 - s_tps1 / 5.0 * 100.0).abs() <= 1.0, "TPS1 {tps1}");
        // PID 0x4A reports the half-voltage track G185 doubled (OBD convention, HYP).
        assert!((pvs1 - s_pvs1 / 5.0 * 100.0).abs() <= 0.6 && (pvs2 - 2.0 * s_pvs2 / 5.0 * 100.0).abs() <= 0.6, "pedal {pvs1}/{pvs2}");
    }

    /// Reads the firmware's fault memory with UDS 0x19 02 2F (the session's tester).
    fn read_dtcs(s: &mut StudioSession) -> Vec<String> {
        let f = s.uds_ecu.request("ReadDTCInformation", &[0x19, 0x02, 0x2F]);
        s.board.inject_can_rx(&f);
        for _ in 0..400 {
            s.step(0.01);
            if !s.uds_ecu.status.pending {
                break;
            }
        }
        s.uds_ecu.status.dtcs.clone()
    }

    #[test]
    #[ignore]
    fn engine_start_scenario() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let mut s = StudioSession::new(&root).expect("session");
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        // Bench configuration (any engine / turbo with any image): SIMEMU_ENGINE = engine profile slug
        // (`EngineProfile::from_slug`, e.g. `cjxc`, `cjsa`), SIMEMU_TURBO = turbo slug (`is12`, `is20`, `is38`).
        if let Ok(e) = std::env::var("SIMEMU_ENGINE") {
            s.plant.engine_profile = simemu_physics::EngineProfile::from_slug(&e).unwrap_or_else(|| panic!("unknown engine {e}"));
        }
        if let Ok(t) = std::env::var("SIMEMU_TURBO") {
            s.plant.engine_profile.turbo = simemu_physics::EngineProfile::turbo_from_slug(&t).unwrap_or_else(|| panic!("unknown turbo {t}"));
        }
        s.plant.reset();
        // Bench state before the ECU wakes up: engine temperature and the engine-off timer of the previous stop
        // (hot restart: a short stop; cold start: an overnight soak), then the battery is connected (power-up).
        let cold = std::env::var("SIMEMU_COLD").is_ok();
        if !cold {
            warm_engine(&mut s.plant);
        }
        s.eot_clock_s = env("SIMEMU_ENGINE_OFF_S", if cold { 12.0 * 3600.0 } else { 300.0 });
        let image = std::env::var("SIMEMU_IMAGE").unwrap_or_else(|_| s.image_file.clone());
        if image == s.image_file {
            s.power_cycle_ecu().expect("power-up");
        } else {
            s.load_image(&image).expect("image");
        }
        eprintln!("image {} | engine-off timer at power-up {} s", s.image_label, s.eot_clock_s);
        let resets0 = s.board.hw().reset_count;
        s.plant.bench_supply_v = std::env::var("SIMEMU_BENCH_SUPPLY").ok().and_then(|v| v.parse().ok());
        let motor: Option<(f64, f64)> = std::env::var("SIMEMU_MOTOR").ok().and_then(|v| {
            let mut it = v.split(',').filter_map(|x| x.parse().ok());
            Some((it.next()?, it.next()?))
        });
        s.board.add_probe(IMMO_RX).expect("probe");
        // Diagnostics (observation only): SIMEMU_PROBES=pc,pc,… counts executions (+ last D/A registers),
        // SIMEMU_PEEK=addr:len,… dumps RAM every 0.1 s. Hex without 0x.
        let hex = |x: &str| u32::from_str_radix(x.trim().trim_start_matches("0x"), 16).ok();
        let probes: Vec<u32> = std::env::var("SIMEMU_PROBES").unwrap_or_default().split(',').filter_map(hex).collect();
        let peeks: Vec<(u32, usize)> = std::env::var("SIMEMU_PEEK")
            .unwrap_or_default()
            .split(',')
            .filter_map(|p| p.split_once(':').and_then(|(a, n)| Some((hex(a)?, n.parse().ok()?))))
            .collect();
        for &p in &probes {
            s.board.add_probe(p).expect("probe");
        }
        // Bench stimulus on inputs the plant does not drive: SIMEMU_ADC=kernel.channel:volts,…
        for (k, ch, v) in std::env::var("SIMEMU_ADC").unwrap_or_default().split(',').filter_map(|x| {
            let (kc, v) = x.split_once(':')?;
            let (k, c) = kc.split_once('.')?;
            Some((k.parse().ok()?, c.parse().ok()?, v.parse().ok()?))
        }) {
            s.board.set_adc_input_voltage(k, ch, v).expect("ADC channel");
        }
        s.plant.driver.ignition_on = true;
        s.plant.driver.selector = simemu_physics::Selector::P;
        let mut st = Stages { min_cranking_v: f64::MAX, ..Default::default() };
        let wall = std::time::Instant::now();
        let (secs, kl50_end) = (env("SIMEMU_START_S", 40.0), env("SIMEMU_KL50_END", 3.0));
        let s50 = s.image_label.starts_with("SC8S50");
        let mut rpm_trace = Vec::new();
        let mut spark_end = [0.0f64; 4];
        let mut inj_end = [0.0f64; 4];
        let mut self_run_from: Option<f64> = None;
        // Immobiliser over CAN (any image): time of the last 0x010 challenge the virtual cluster answered.
        let (mut immo_seen, mut last_challenge_s) = (0u64, f64::NAN);
        // firmware heater duty bytes of the last 200 ms (one per 10 ms slice), newest first
        let mut heater_hist = [[0u8; 20]; 2];
        let rd8 = |s: &StudioSession, a: u32| s.board.read_mem(a, 1).map(|b| b[0]).unwrap_or(0);
        for i in 0..(secs / 0.01) as usize {
            let t = (i + 1) as f64 * 0.01;
            // DSG car: brake pressed (start plausibility, FR p.8613), then the start request (KL50).
            s.plant.driver.brake_pct = if t >= 0.5 { 60.0 } else { 0.0 };
            s.plant.driver.start_button = (1.0..kl50_end).contains(&t);
            s.plant.motoring_rpm = motor.and_then(|(t0, n)| (t >= t0).then_some(n));
            s.step(0.01);
            let tel = s.plant.telemetry();
            let o = s.actuators.ecu.clone();
            for k in 0..2 {
                heater_hist[k].rotate_right(1);
                heater_hist[k][0] = rd8(&s, LSH_DUTY_BYTES[k]);
            }
            st.max_rpm = st.max_rpm.max(tel.rpm);
            if s.immo.ecu_challenges > immo_seen {
                (immo_seen, last_challenge_s) = (s.immo.ecu_challenges, t);
            }
            if o.main_relay == Some(true) {
                st.relays_on_s.get_or_insert(t);
            }
            if o.lpfp_duty.unwrap_or(0.0) > 0.1 {
                st.fuel_pump_on_s.get_or_insert(t);
            }
            if o.starter_relay == Some(true) {
                st.starter_on_s.get_or_insert(t);
                st.min_cranking_v = st.min_cranking_v.min(tel.battery_v);
            } else if st.starter_on_s.is_some() {
                st.starter_off_s.get_or_insert(t);
            }
            if tel.rpm > 30.0 {
                st.crank_s.get_or_insert(t);
            }
            if tel.rpm > 150.0 {
                st.crank_150_s.get_or_insert(t);
            }
            if tel.rail_mpa > 4.0 {
                st.rail_4mpa_s.get_or_insert(t);
            }
            let io = s.board.engine_io_state();
            for (c, se) in spark_end.iter_mut().enumerate() {
                if let Some(p) = io.coil[c].filter(|p| p.end_s != *se) {
                    *se = p.end_s;
                    st.sparks[c] += 1;
                    st.first_spark_s.get_or_insert(t);
                }
            }
            for (c, ie) in inj_end.iter_mut().enumerate() {
                if let Some(p) = io.injector[c].last().filter(|p| p.end_s != *ie) {
                    *ie = p.end_s;
                    st.injections[c] += 1;
                }
            }
            if o.injection.iter().flatten().any(Option::is_some) {
                st.first_injection_s.get_or_insert(t);
            }
            st.port_fuel_seen |= s.plant.engine.port.iter().any(|p| p.injected_last_kg > 0.0);
            let pcp = &s.board.engine_io().pcp().stats.starts;
            if tel.engine_state == "running" {
                st.running_s.get_or_insert(t);
            }
            if o.starter_relay != Some(true) && tel.rpm > 600.0 {
                let t0 = *self_run_from.get_or_insert(t);
                st.self_run_s = st.self_run_s.max(t - t0);
            } else {
                self_run_from = None;
            }
            if i % 10 == 9 {
                rpm_trace.push(tel.rpm.round());
                if LONG_IDLE_S.contains(&t) {
                    st.long_idle.push(tel.rpm);
                    st.lsu_tip_c.push(tel.lsu_tip_c);
                }
                // Heater outputs: the decoded low fraction of the two PWM cells vs the firmware's duty bytes. The cell is
                // measured over its last completed period (10 ms upstream, 100 ms downstream), so the byte it shows is
                // one of those written during the last two periods.
                for (k, n) in [(0usize, 3usize), (1, 20)] {
                    let b = heater_hist[k][0];
                    if let (true, Some(d)) = (s50 && b > 0 && b < 0xFD, o.lambda_heater_duty[k]) {
                        st.heater_checks[k] += 1;
                        st.heater_match[k] += heater_hist[k][..n].iter().any(|&b| (d - (b as f64 + 1.0) / 256.0).abs() < 0.01) as u32;
                    }
                }
                if IDLE_WINDOW_S.contains(&t) {
                    st.idle.push([tel.rpm, tel.lambda_sensor, tel.throttle_pct * 0.9, tel.rail_mpa]);
                }
                let n_fw = s.board.read_mem(OBD_N, 2).map(|b| u16::from_le_bytes([b[0], b[1]])).unwrap_or(0);
                if let Some(m) = s.can_log.iter().rev().find(|f| f.src == "ECU" && f.id == "121") {
                    st.motor20 = m.data.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
                }
                eprintln!(
                    "  fw: N_SP_IS {:.0} rpm MAF {:.2} g/s (plant {:.2}) MAP {:.1} kPa (plant {:.1}) load {:.1} % TQ_ENG {:.1} req {:.1} Nm IGA {:.1} ° | Motor_20 [{}] | FUP_SP {:.1} FUP {:.1} MPa PUMP_VOL {:.1} % TI MPI {:.2} ms | TOIL {:.0} °C TIA {:.1} °C | plant IAT {:.1} °C port film {:.1} mg MPI {:?}",
                    st.n_sp_is(),
                    st.fw.maf_g_s,
                    tel.maf_kg_h / 3.6,
                    st.fw.map_kpa,
                    tel.map_kpa,
                    st.fw.load_pct,
                    st.fw.tq_eng_nm,
                    st.fw.tq_req_nm,
                    st.fw.iga_deg,
                    st.motor20.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" "),
                    st.fw.fup_sp_mpa,
                    st.fw.fup_mpa,
                    st.fw.pump_vol_pct,
                    st.fw.ti_mpi_ms,
                    st.fw.toil_c,
                    st.fw.tia_c,
                    tel.iat_c,
                    s.plant.engine.port.iter().map(|p| p.film_kg).sum::<f64>() * 1e6,
                    o.port_injection[0].iter().flatten().map(|p| (p.soi_btdc_deg.round(), (p.width_ms * 100.0).round() / 100.0)).collect::<Vec<_>>(),
                );
                eprintln!(
                    "t {t:.2}s rpm {:.0} ({}) N_fw {:.0} kl50 {} starter {:?} {:.0} A vb {:.1} V | immo ok {} | sparks {:?} inj {:?} {:?} | rail {:.1} MPa N276 {:?} | VBOOST {:.1} V | ADC mon err {}{} | PCP inj {:?} vcv {} | lp {:.1} bar MAP {:.0} kPa",
                    tel.rpm,
                    tel.engine_state,
                    n_fw as f64 / 4.0,
                    s.plant.driver.start_button as u8,
                    o.starter_relay,
                    tel.starter_current_a,
                    tel.battery_v,
                    rd8(&s, IMMO_ANSWER_OK),
                    st.sparks,
                    st.injections,
                    o.injection[0].iter().flatten().map(|p| (p.soi_btdc_deg.round(), (p.width_ms * 100.0).round() / 100.0)).collect::<Vec<_>>(),
                    tel.rail_mpa,
                    (o.hpfp_close_deg, o.hpfp_off_deg),
                    rd8(&s, VBOOST) as f64 * 0.5,
                    rd8(&s, ERR_CONV_MON),
                    rd8(&s, ERR_VP_ALV_CHK_MON),
                    PCP_INJ.map(|c| pcp[c]),
                    pcp[PCP_VCV],
                    tel.lp_fuel_bar,
                    tel.map_kpa,
                );
                // λ control observed over OBD (tester ReadDataByIdentifier, read-only): PID 0x03 fuel system status,
                // 0x34 O2 sensor 1 equivalence ratio + current, 0x06 short-term fuel trim — one request every 0.5 s,
                // the answer is read 0.1 s later.
                if std::env::var_os("SIMEMU_UDS_DID").is_none() {
                    let r: Vec<u8> = s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
                    if i % 50 == 9 && r.len() >= 14 && r[..3] == [0x62, 0xF4, 0x03] && r[5..7] == [0xF4, 0x34] && r[11..13] == [0xF4, 0x06] {
                        let status = r[3];
                        st.fuel_status_seen |= status;
                        let lambda_fw = u16::from_be_bytes([r[7], r[8]]) as f64 * 2.0 / 65536.0;
                        let ip_fw_ma = u16::from_be_bytes([r[9], r[10]]) as f64 / 256.0 - 128.0;
                        if status == 2 {
                            st.stft_closed_loop.push((r[13] as f64 - 128.0) * 100.0 / 128.0);
                        }
                        // a ready sensor: plausible λ and a pump current (the firmware answers λ 0 / −128 mA before)
                        if (0.4..4.0).contains(&lambda_fw) && ip_fw_ma.abs() > 0.02 && ip_fw_ma.abs() < 10.0 {
                            st.lambda_fw_vs_plant.push((lambda_fw, s.plant.exhaust_sensors.lambda_up_cell));
                        }
                    }
                    // VW measured values (DID table of the image, names: log_tools::channels): FUP_SP, FUP, TI_1_MES_MPI[0]
                    // and TOIL, TIA, PUMP_VOL_VCV.
                    let fw_b: [(u16, usize); 3] = [(0x293B, 2), (0x2027, 2), (0x13AC, 2)];
                    let fw_c: [(u16, usize); 3] = [(0x202F, 2), (0x1001, 1), (0x209A, 2)];
                    // MAF_CYL (the firmware air mass flow, kg/h), MAP (G71, 0.1 kPa), OBD calculated load.
                    let fw_d: [(u16, usize); 3] = [(0x2032, 2), (0x39C0, 2), (0xF404, 1)];
                    // TQ_ENG (firmware torque estimate), TQ_DRIV_REQ, IGA_AV_MV (log_tools::channels).
                    let fw_e: [(u16, usize); 3] = [(0x437C, 2), (0x4380, 2), (0x2004, 2)];
                    let i16v = |b: &[u8]| i16::from_be_bytes([b[0], b[1]]) as f64;
                    if i % 50 == 19 {
                        if let Some(v) = did_values(&r, &fw_e) {
                            (st.fw.tq_eng_nm, st.fw.tq_req_nm, st.fw.iga_deg) = (i16v(&v[0]) / 10.0, i16v(&v[1]) / 10.0, i16v(&v[2]) / 100.0);
                        }
                    }
                    let w16 = |b: &[u8]| u16::from_be_bytes([b[0], b[1]]) as f64;
                    if i % 50 == 29 {
                        if let Some(v) = did_values(&r, &fw_b) {
                            (st.fw.fup_sp_mpa, st.fw.fup_mpa, st.fw.ti_mpi_ms) = (w16(&v[0]) / 100.0, w16(&v[1]) / 100.0, w16(&v[2]) / 250.0);
                        }
                    }
                    if i % 50 == 39 {
                        if let Some(v) = did_values(&r, &fw_c) {
                            (st.fw.toil_c, st.fw.tia_c, st.fw.pump_vol_pct) = ((w16(&v[0]) - 2731.4) / 10.0, v[1][0] as f64 * 0.75 - 48.0, w16(&v[2]) / 100.0);
                        }
                    }
                    if i % 50 == 49 {
                        if let Some(v) = did_values(&r, &fw_d) {
                            (st.fw.maf_g_s, st.fw.map_kpa, st.fw.load_pct) = (w16(&v[0]) / 3.6, w16(&v[1]) / 10.0, v[2][0] as f64 * 100.0 / 255.0);
                        }
                    }
                    let req = match i % 50 {
                        49 => vec![0x22, 0xF4, 0x03, 0xF4, 0x34, 0xF4, 0x06],
                        9 => fw_e.iter().fold(vec![0x22], |mut q, d| { q.extend(d.0.to_be_bytes()); q }),
                        39 => fw_d.iter().fold(vec![0x22], |mut q, d| { q.extend(d.0.to_be_bytes()); q }),
                        19 => fw_b.iter().fold(vec![0x22], |mut q, d| { q.extend(d.0.to_be_bytes()); q }),
                        29 => fw_c.iter().fold(vec![0x22], |mut q, d| { q.extend(d.0.to_be_bytes()); q }),
                        _ => vec![],
                    };
                    if !req.is_empty() {
                        let f = s.uds_ecu.request("ReadDataByIdentifier", &req);
                        s.board.inject_can_rx(&f);
                    }
                    if i % 50 == 29 && st.fw.ti_mpi_ms > 0.0 {
                        st.mpi_released_s.get_or_insert(t);
                    }
                    // FUP / FUP_SP are refreshed at slot 29; skip the first 2 s on its own: after the start the firmware ramps
                    // FUP_SP down faster than injection can empty the rail (min delivery, the pump cannot remove fuel)
                    if i % 50 == 29 && t >= 3.0 && self_run_from.is_some_and(|t0| t - t0 > 2.0) && st.fw.fup_sp_mpa > 0.0 {
                        st.fup_track.push((t, st.fw.fup_mpa, st.fw.fup_sp_mpa, tel.rail_mpa));
                    }
                    if FW_IDLE_WINDOW.contains(&t) && i % 50 == 39 {
                        st.fw_idle.push((tel.rpm, st.n_sp_is(), st.fw.fup_mpa, st.fw.fup_sp_mpa));
                    }
                }
                if let Ok(dids) = std::env::var("SIMEMU_UDS_DID") {
                    // ReadDataByIdentifier through the session's tester: the comma-separated DIDs in groups of
                    // three (one single-frame request) every 0.1 s in turn; the answer is printed one step later.
                    eprintln!("  uds {}: {}", s.uds_ecu.status.request, s.uds_ecu.status.response_hex);
                    let dids: Vec<u16> = dids.split(',').filter_map(|d| u16::from_str_radix(d.trim(), 16).ok()).collect();
                    let groups: Vec<&[u16]> = dids.chunks(3).collect();
                    if !groups.is_empty() {
                        let mut req = vec![0x22];
                        groups[(i / 10) % groups.len()].iter().for_each(|d| req.extend(d.to_be_bytes()));
                        let f = s.uds_ecu.request("ReadDataByIdentifier", &req);
                        s.board.inject_can_rx(&f);
                    }
                }
                if s50 && std::env::var_os("SIMEMU_ERRM").is_some() && i % 100 == 99 {
                    eprintln!("  errm failed: {:?}", errm_failed(&s));
                }
                if std::env::var_os("SIMEMU_CYL_REPORT").is_some() {
                    let r = |v: [f64; 4]| v.map(|x| (x * 100.0).round() / 100.0);
                    let br = s.board.power_stage_outputs().bridges;
                    eprintln!(
                        "  cyl: air {:.1} mg (maf {:.2} kg/h) fuel {:?} mg lambda {:?} exhaust {:.2} firing {:?} imep {:?} bar peak {:?} bar | throttle {:.1} % pedal {:.0} | ATIC111 {:?} | spark {:?} | Nm ind {:.1} fric {:.1} pump {:.1} aux {:.1} | residual {:.3} T_ivc {:.0} K cam {:?} | port flaps {:.2} MAP {:.1} kPa misfires {:?}",
                        tel.air_mg_stroke,
                        tel.maf_kg_h,
                        r(tel.fuel_mg_stroke),
                        r(tel.lambda_cyl),
                        tel.lambda_sensor,
                        tel.firing,
                        r(tel.imep_bar),
                        r(tel.cyl_peak_bar),
                        tel.throttle_pct,
                        s.plant.driver.pedal_pct,
                        br.map(|b| (b.enabled, format!("{:04X}", b.command))),
                        o.ignition.map(|c| c.map(|c| ((c.spark_btdc_deg * 10.0).round() / 10.0, (c.dwell_ms * 100.0).round() / 100.0))),
                        tel.torque_indicated_nm,
                        tel.torque_friction_nm,
                        tel.torque_pumping_nm,
                        s.plant.engine.flows.torque_aux_nm,
                        s.plant.engine.cyl[0].last.residual,
                        0.0,
                        tel.cam_phase_deg,
                        s.plant.engine.port_flap_closed,
                        tel.map_kpa,
                        s.plant.engine.cyl.iter().map(|c| c.misfire_events).collect::<Vec<_>>(),
                    );
                }
                if std::env::var_os("SIMEMU_ATIC142").is_some() {
                    let c = &s.board.hw().spi.ps.atic142;
                    let d = s.board.read_mem(ATIC142_DEV, 0x38).unwrap_or_default();
                    let io = s.board.read_mem(WRAF_IO, 8).unwrap_or_default();
                    let h = |k: usize| i16::from_le_bytes([io[2 * k], io[2 * k + 1]]);
                    eprintln!(
                        "  atic142: ctl 2={:04X} 3={:04X} 8={:04X} 9={:04X} A={:04X} B={:04X} C={:04X} conf dl {} | dev state {} conf {} com_err {} conf_err {} | fw IPLSL {:.3} mA V_R_IT {:.3} V VSLSL {:.3} V VPLSL {:.3} V | cell Ip {:.3} mA Ri {:.0} Ω E {:.2} V, bin E {:.2} V Ri {:.0} Ω",
                        c.ctl[2], c.ctl[3], c.ctl[8], c.ctl[9], c.ctl[0xA], c.ctl[0xB], c.ctl[0xC], c.conf_downloads, d[0x26], d[0x27], d[0x2A], d[0x2B],
                        h(0) as f64 * 366.39e-6, h(1) as f64 * 152.588e-6, h(2) as f64 * 152.588e-6, h(3) as f64 * 152.588e-6,
                        c.lin.ip_lim_ma, c.lin.ri_ohm, c.lin.e_exh_v, c.bin.e_v, c.bin.ri_ohm
                    );
                    let taps: Vec<String> = [(1u8, 56u8), (1, 57), (1, 58), (1, 59), (2, 16), (2, 17), (2, 18), (2, 19)]
                        .iter()
                        .map(|&(k, c)| {
                            let t = s.board.engine_io().tap(k, simemu_tc1791::gpta::CellKind::Ltc, c);
                            format!("K{k}.{c} {}:{:.3}/{:.3}", t.level as u8, t.high_ns as f64 / 1e6, t.low_ns as f64 / 1e6)
                        })
                        .collect();
                    eprintln!("  heater cells (high/low ms): {taps:?} fw duty bytes {:?}", s.board.read_mem(0xD000_C56B, 2).unwrap_or_default());
                    eprintln!(
                        "  sensors: heater duty {:?} | LSU tip {:.0} °C {:.1} W, LSF tip {:.0} °C | λ gas up {:.3} down {:.3}",
                        o.lambda_heater_duty.map(|d| d.map(|x| (x * 1000.0).round() / 1000.0)), tel.lsu_tip_c, tel.lsu_heater_w, tel.lsf_tip_c, tel.lambda_sensor, tel.lambda_down
                    );
                }
                if std::env::var_os("SIMEMU_PROBE_TRACE").is_some() {
                    for &p in &probes {
                        let pr = s.board.probe(p).expect("probe");
                        eprintln!("  probe {p:08X} hits {} d4 {:X} d5 {:X} d8 {:X}", pr.hits, pr.last_regs[4], pr.last_regs[5], pr.last_regs[8]);
                    }
                    let t = s.board.engine_io().tap(1, simemu_tc1791::gpta::CellKind::Ltc, 27);
                    eprintln!("  G1.LTC27 level {} high {:.3} ms low {:.3} ms | P14 OUT {:02X}", t.level as u8, t.high_ns as f64 / 1e6, t.low_ns as f64 / 1e6, s.board.read_mem(0xF030_0200, 1).map(|b| b[0]).unwrap_or(0));
                }
                if let Ok(dir) = std::env::var("SIMEMU_RAM_DUMP") {
                    // Read-only snapshot of the data scratchpad (DSPR 0xD0000000, 128 KB) every 0.1 s.
                    let b = s.board.read_mem(0xD000_0000, 0x2_0000).unwrap_or_default();
                    let _ = std::fs::write(format!("{dir}/dspr_{:05}.bin", (t * 100.0).round() as u32), b);
                }
                for &(a, n) in &peeks {
                    let b = s.board.read_mem(a, n).unwrap_or_default();
                    eprintln!("  peek {a:08X}: {}", b.iter().map(|x| format!("{x:02X}")).collect::<Vec<_>>().join(" "));
                }
            }
        }
        if std::env::var("SIMEMU_IO_REPORT").is_ok() {
            for (k, c) in [(1usize, 24u8), (1, 25), (1, 26), (1, 27), (1, 28), (1, 29), (1, 30), (1, 31)] {
                let lines = s.board.engine_io_mut().out_lines_of(k, simemu_tc1791::gpta::CellKind::Ltc, c);
                eprintln!("  OMCR: G{k}.LTC{c} -> OUT{lines:?}");
            }
            let io = s.board.engine_io();
            let st = &io.pcp().stats.starts;
            eprintln!("PCP channel starts: {:?}", (0..st.len()).filter(|&c| st[c] > 0).map(|c| (format!("{c:#04x}"), st[c])).collect::<Vec<_>>());
            if let Ok(cells) = std::env::var("SIMEMU_GTC_LOG") {
                let cells: Vec<(u8, u8)> = cells.split(',').filter_map(|x| x.split_once('.')).filter_map(|(k, c)| Some((k.parse().ok()?, c.parse().ok()?))).collect();
                for e in io.gtc_log.iter().filter(|e| cells.contains(&(e.kernel, e.cell))) {
                    eprintln!("  gtc K{}.{:02} t {:.6} {} deg {:.1} ({:.1})", e.kernel, e.cell, e.t_ns as f64 * 1e-9, e.level as u8, e.deg, e.deg.rem_euclid(720.0));
                }
            }
            for k in 0..3u8 {
                for (kind, n) in [(simemu_tc1791::gpta::CellKind::Gtc, 32u8), (simemu_tc1791::gpta::CellKind::Ltc, 64)] {
                    for c in 0..n {
                        let t = io.tap(k, kind, c);
                        if t.transitions > 0 {
                            eprintln!("  tap K{k} {kind:?}{c:02}: {} transitions, level {}, high {:.3} ms low {:.3} ms", t.transitions, t.level as u8, t.high_ns as f64 / 1e6, t.low_ns as f64 / 1e6);
                        }
                    }
                }
            }
        }
        for &p in &probes {
            let pr = s.board.probe(p).expect("probe");
            let r = |k: std::ops::Range<usize>| pr.last_regs[k].iter().map(|x| format!("{x:X}")).collect::<Vec<_>>().join(" ");
            eprintln!("probe {p:08X} hits {} | d {} | a {}", pr.hits, r(0..16), r(16..32));
        }
        st.ecu_resets = s.board.hw().reset_count - resets0;
        st.immo_challenges = s.immo.ecu_challenges;
        let immo_rx_runs = s.board.probe(IMMO_RX).map_or(0, |p| p.hits);
        let immo_ok = rd8(&s, IMMO_ANSWER_OK);
        eprintln!("{secs:.1} s simulated in {:.1} s wall ({:?} CPU); stages: {st:#?}", wall.elapsed().as_secs_f64(), s.board.backend());
        eprintln!("rpm every 0.1 s: {rpm_trace:?}");
        let dtcs = read_dtcs(&mut s);
        let failed: Vec<&String> = dtcs.iter().filter(|d| d.rsplit("0x").next().and_then(|x| u8::from_str_radix(x, 16).ok()).is_some_and(|st| st & 0x2F != 0)).collect();
        eprintln!("ECU fault memory: {} entries ({}), failed/pending/confirmed: {failed:?}", dtcs.len(), s.uds_ecu.status.note);
        let errm = if s50 { errm_failed(&s) } else { Vec::new() };
        eprintln!("ERRM failed errors (SC800S50 RAM): {errm:?}");
        assert!(s.ecu_error.is_none(), "{:?}", s.ecu_error);
        assert_eq!(st.ecu_resets, 0);
        assert!(s.ecu_can_tx_total > 0, "ECU silent on CAN");
        // Immobiliser: the cluster's 0x011 answer reaches the firmware's receive handler and passes its check.
        // Immobiliser, image-independent (CAN): the ECU challenged on 0x010, the cluster answered on 0x011 and the ECU
        // stopped challenging (it accepted the answer). SC800S50 additionally: its receive handler ran and the RAM check
        // flag 0xD0010428 is set (docs/SIMOS18_WFS5_RE.md §5).
        eprintln!(
            "immobiliser: binding {:?}, challenges {} answers {} last challenge at {last_challenge_s:.2} s",
            s.immo.binding, s.immo.ecu_challenges, s.immo.ecu_answers
        );
        assert!(s.immo.ecu_challenges > 0 && s.immo.ecu_answers > 0, "immobiliser: no 0x010 challenge answered ({:?})", s.immo);
        assert!(secs - last_challenge_s > 2.0, "immobiliser: the ECU keeps challenging (last at {last_challenge_s:.2} s)");
        if s50 {
            assert!(immo_rx_runs > 0 && immo_ok == 1, "immobiliser answer: handler ran {immo_rx_runs}×, check {immo_ok}");
        }
        // Reached so far (firmware decisions only): relays at KL15, fuel pump pre-run, starter on the KL50
        // request (not before), cranking without a hang on the first compression, synchronised spark on every
        // cylinder while cranking, rail pressure from the firmware-driven HPFP valve.
        assert!(st.relays_on_s.is_some(), "main relays");
        assert!(st.fuel_pump_on_s.is_some_and(|t| t < 1.0), "fuel pump pre-run at ignition on");
        if motor.is_none() && kl50_end > 1.3 {
            let on = st.starter_on_s.expect("starter");
            // SC8S50 energises the starter 0.1–0.3 s after KL50; SC8O40 at ≈ 0.6 s (its own start-request debounce, not decoded)
            assert!(on > 1.0 && on < if s50 { 1.3 } else { kl50_end }, "starter must follow the KL50 request: {on}");
            assert!(st.crank_150_s.is_some_and(|t| t - on < 0.6), "starter hangs: 150 rpm at {:?}", st.crank_150_s);
            assert!(st.sparks.iter().all(|&n| n >= 2), "sparks per cylinder while cranking {:?}", st.sparks);
            assert!(st.rail_4mpa_s.is_some(), "HPFP: rail pressure never above the 4 MPa high-pressure-start threshold");
            // DI injection (ATIC121 inputs G0.GTC04/17/16/05) on every cylinder once the rail is up, and the
            // plant burns it: the engine fires on its own (doc §12.3).
            assert!(st.injections.iter().all(|&n| n >= 2), "injections per cylinder while cranking {:?}", st.injections);
            assert!(st.first_injection_s.is_some_and(|t| t > st.rail_4mpa_s.unwrap()), "injection only after the high-pressure-start threshold");
            assert!(st.running_s.is_some() && st.max_rpm > 400.0, "first combustions: running at {:?}, max {:.0} rpm", st.running_s, st.max_rpm);
            // Fuel system (GPTA doc §12.6): while the engine runs on its own the rail follows the firmware's set-point
            // (FUP_SP vs FUP over UDS, both from the firmware) within ±1 MPa, the N276 power-stage diagnosis passes
            // (VCV_OC: bridged-load model of ATIC140#1 stages 24/25), and once the firmware releases MPI (TI_1_MES_MPI > 0)
            // the decoded port pulses reach the plant.
            // At the DI → MPI switch the firmware moves FUP_SP onto the measured pressure (the rail can only fall by
            // leakage without DI consumption); the second after the release is the set-point transition, not tracking.
            let switching = |t: f64| st.mpi_released_s.is_some_and(|r| (t - r).abs() < 1.0);
            // Likewise right after the start: the rail is still at its cranking pressure and the firmware ramps FUP_SP
            // down faster than injection can drain it (observed 12.1 vs 11.0 MPa at the first sample, < 0.3 MPa 1 s later),
            // and generally while the firmware still lowers FUP_SP (by > 0.1 MPa to the next sample).
            let t0 = st.fup_track.first().map_or(f64::MAX, |x| x.0); // first sample of unaided running
            let settling = |t: f64| t < t0 + 1.0;
            let ramping = |k: usize| st.fup_track.get(k + 1).is_some_and(|n| n.2 < st.fup_track[k].2 - 0.1);
            let rail_dev = st.fup_track.iter().enumerate().filter(|&(k, x)| !switching(x.0) && !settling(x.0) && !ramping(k)).map(|(_, &(_, fup, sp, _))| (fup - sp).abs()).fold(0.0, f64::max);
            eprintln!("rail while running (t, FUP, FUP_SP, plant MPa): {:?}", st.fup_track.iter().map(|x| [x.0, x.1, x.2, (x.3 * 10.0).round() / 10.0]).collect::<Vec<_>>());
            assert!(st.fup_track.len() >= 2 && rail_dev <= 1.0, "rail vs firmware set-point: {} samples (engine on its own {:.1} s), max deviation {rail_dev:.2} MPa", st.fup_track.len(), st.self_run_s);
            if st.mpi_released_s.is_some() {
                assert!(st.port_fuel_seen, "MPI released by the firmware at {:?} s but no port fuel reached the plant", st.mpi_released_s);
            }
            // Idle targets (firmware's own): >= 30 s within ±80 rpm of N_SP_IS (Motor_20), fuel-system status 2 with
            // |STFT| < 10 %, rail ±1 MPa. Reported always, asserted with SIMEMU_IDLE_TARGETS=1: the speed target is reached on
            // 259J/`cjxc` and 264H/`cjsa` since the quasi-dimensional combustion (docs/SIMOS18_COMBUSTION.md); STFT sits at
            // +12…14 % (the firmware's air-charge model gives 12 % less air than the plant inducts, ibid. §6).
            let on_target = st.fw_idle.iter().filter(|(n, sp, _, _)| sp.is_finite() && (n - sp).abs() <= 80.0).count();
            let stft_ok = !st.stft_closed_loop.is_empty() && st.stft_closed_loop.iter().rev().take(10).all(|x| x.abs() < 10.0);
            eprintln!(
                "idle targets: {on_target}/{} samples (0.5 s) within ±80 rpm of N_SP_IS, status 2 + |STFT| < 10 %: {stft_ok}, rail max dev {rail_dev:.2} MPa, self-run {:.1} s | samples (rpm, N_SP_IS): {:?}",
                st.fw_idle.len(),
                st.self_run_s,
                st.fw_idle.iter().map(|x| (x.0.round(), x.1)).collect::<Vec<_>>()
            );
            let strict = std::env::var_os("SIMEMU_IDLE_TARGETS").is_some();
            assert!(st.self_run_s >= 2.0, "self-sustained running only {:.1} s", st.self_run_s);
            if strict && secs >= LONG_IDLE_S.end {
                assert!(on_target >= 60, "warm idle on the firmware's target for >= 30 s: {on_target} samples");
                assert!(stft_ok && st.fuel_status_seen & 2 != 0, "closed loop with |STFT| < 10 %: {:?}", st.stft_closed_loop);
            }
            // (checked after the idle targets are reported)
            if s50 {
                // Failed errors left are bench conditions or owned by other models: chassis ECUs (COM_*: the rest-bus
                // `simemu_bridge::mqb` sends what the ECU's coding expects, MILESTONE §2.10), the wastegate position sensor (VP_BPA_L_1)
                // and cam phasers (CAM_STAT_VCP_*), and the rotary coolant valve's SENT sensor (RVC_DIG_COM).
                // docs/SIMOS18_GPTA_IO_RE.md §12.6. The oil level/temperature sender G266 and the oil-pressure switches are
                // emulated (§12.11): no OIL_SENS_* / POIL_* error may be failed.
                let unexplained: Vec<&String> = errm.iter().filter(|e| !(e.starts_with("COM_") || e.starts_with("VP_BPA_") || e.starts_with("CAM_STAT_VCP") || e.as_str() == "RVC_DIG_COM")).collect();
                assert!(unexplained.is_empty(), "unexplained failed errors {unexplained:?}");
            }
            // After-start idle held by the firmware alone (doc §12.4) and the λ chain (docs/SIMOS18_BOARD_HW.md §6, GPTA doc
            // §12.5): asserted with SIMEMU_IDLE_TARGETS=1.
            if strict && secs >= IDLE_WINDOW_S.end {
                let col = |k: usize| st.idle.iter().map(move |x| x[k]);
                let (rpm_min, rpm_max) = (col(0).fold(f64::MAX, f64::min), col(0).fold(0.0, f64::max));
                let lambda = col(1).sum::<f64>() / st.idle.len() as f64;
                eprintln!("idle window {IDLE_WINDOW_S:?} s: {rpm_min:.0}..{rpm_max:.0} rpm, mean λ {lambda:.2}, throttle {:?}°, rail {:?} MPa", col(2).map(|x| (x * 10.0).round() / 10.0).collect::<Vec<_>>(), col(3).map(|x| x.round()).collect::<Vec<_>>());
                assert!(st.self_run_s >= 5.0, "self-sustained running only {:.1} s", st.self_run_s);
                assert!(rpm_min > 600.0 && rpm_max < 1500.0, "after-start idle {rpm_min:.0}..{rpm_max:.0} rpm");
                assert!(col(2).all(|x| (1.0..6.0).contains(&x)), "throttle under ETC control at an idle opening");
            }
            if strict && secs >= LONG_IDLE_S.end {
                let (lo, hi) = (st.long_idle.iter().cloned().fold(f64::MAX, f64::min), st.long_idle.iter().cloned().fold(0.0, f64::max));
                let tip = st.lsu_tip_c.iter().sum::<f64>() / st.lsu_tip_c.len().max(1) as f64;
                // the firmware's reading is segment-averaged and filtered, the plant value instantaneous: median ratio
                // within ±10 %, most samples within 20 %
                let agree = st.lambda_fw_vs_plant.iter().filter(|(f, p)| (f / p - 1.0).abs() < 0.2).count();
                let mut ratios: Vec<f64> = st.lambda_fw_vs_plant.iter().map(|(f, p)| f / p).collect();
                ratios.sort_by(|a, b| a.total_cmp(b));
                let median = ratios.get(ratios.len() / 2).copied().unwrap_or(0.0);
                eprintln!(
                    "lasting idle {LONG_IDLE_S:?} s: {lo:.0}..{hi:.0} rpm, self-run {:.1} s | heater decode {:?}/{:?} | LSU tip {tip:.0} °C | fuel status seen {:#04x}, closed-loop STFT {:?} % | firmware λ vs plant {agree}/{} within 20 %, median ratio {median:.3}",
                    st.self_run_s,
                    st.heater_match,
                    st.heater_checks,
                    st.fuel_status_seen,
                    st.stft_closed_loop.iter().map(|x| x.round()).collect::<Vec<_>>(),
                    st.lambda_fw_vs_plant.len(),
                );
                assert!(st.self_run_s >= 30.0, "self-sustained running only {:.1} s", st.self_run_s);
                assert!(lo > 700.0 && hi < 1600.0, "lasting idle {lo:.0}..{hi:.0} rpm");
                if s50 {
                    let dev = s.board.read_mem(ATIC142_DEV, 0x38).unwrap_or_default();
                    assert_eq!((dev[0x26], dev[0x27]), (1, 3), "ATIC142 accepted (state 1), configuration SUCCESS (3)");
                    for k in 0..2 {
                        assert!(st.heater_checks[k] > 50 && st.heater_match[k] * 100 >= st.heater_checks[k] * 85, "heater {k}: decode {:?}/{:?}", st.heater_match, st.heater_checks);
                    }
                    assert!((tip - 785.0).abs() < 30.0, "LSU tip {tip:.0} °C under the firmware's heater control");
                }
                assert!(st.lambda_fw_vs_plant.len() >= 10 && agree * 10 >= st.lambda_fw_vs_plant.len() * 6 && (0.9..1.1).contains(&median), "firmware λ reading tracks the plant");
            }
        }
        assert!(st.max_rpm > 150.0, "cranking");
    }
}
