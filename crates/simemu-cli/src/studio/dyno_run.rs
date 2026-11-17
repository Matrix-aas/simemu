//! Engine-dyno scenario: the unmodified firmware runs the engine at wide-open throttle on the test cell and is
//! judged only through its own observables (UDS 0x22 DIDs) and the plant: boost pressure against the firmware's
//! set point, the electric wastegate position against its set point, cam phasing against the cam set points, and
//! per-cylinder knock retard after knock is induced physically (fuel switched to a low-octane grade on the test
//! cell's fuel conditioning unit). Run:
//! `SIMEMU_CPU=native cargo test --release -p simemu-cli engine_dyno_wot_scenario -- --ignored --nocapture`.
//! `SIMEMU_DYNO_RPM` (default 3000), `SIMEMU_DYNO_S` (default 40), `SIMEMU_KNOCK_T` (fuel switch time, default
//! 25 s), `SIMEMU_KNOCK_RON` (default 78), `SIMEMU_ENGINE` (plant engine slug, default `2.0t_is38` = EA888 Gen3
//! CJXC + IS38 — the bench image's calibration family; any engine/turbo profile can be chosen; with `SIMEMU_BUILD` the
//! bench build's car is used), `SIMEMU_ENGINE_C` (coolant/oil temperature of the soaked engine, default 90),
//! `SIMEMU_DRIVETRAIN` (the car the engine came from). With `SIMEMU_IMAGE` or `SIMEMU_DRIVETRAIN` the car's
//! four-wheel-drive flag is coded into the ECU before the run (`tests::code_ecu_drivetrain`; `SIMEMU_ECU_CODING=dump`
//! keeps the dump's coding). `SIMEMU_CELL_FAN_MS`: test-cell cooling fan (default `simemu_physics::TEST_CELL_FAN_MS`).
//!
//! Bench (test-cell equipment and driver only, AGENTS.md §2.2): warm engine, start like `engine_run` (ignition,
//! brake, KL50), gearbox removed (no TCU), a 4-quadrant AC dyno in speed mode holds the engine speed
//! (`VirtualPlant::motoring_rpm`, ramped), pedal 100 %.

#[cfg(test)]
mod tests {
    use crate::studio::session::{StudioSession, S50_FLASH};
    use simemu_physics::{BenchMode, DrivetrainProfile, EngineProfile, Selector};
    use std::path::PathBuf;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    /// Firmware observables read over UDS 0x22 (DIDs as listed in `log_tools::channels`; SimosTools scaling).
    struct Did {
        name: &'static str,
        did: u16,
        signed: bool,
        scale: f64,
        offset: f64,
    }

    const DIDS: &[Did] = &[
        Did { name: "PUT", did: 0x202A, signed: false, scale: 0.1, offset: 0.0 },
        Did { name: "PUT_SP", did: 0x2029, signed: false, scale: 0.1, offset: 0.0 },
        Did { name: "POSN_BPA", did: 0x39A2, signed: false, scale: 0.01, offset: 0.0 },
        Did { name: "POSN_SP_BPA", did: 0x39A3, signed: false, scale: 0.01, offset: 0.0 },
        Did { name: "IGA_AV", did: 0x2004, signed: true, scale: 0.01, offset: 0.0 },
        Did { name: "KNK1", did: 0x200A, signed: true, scale: 0.01, offset: 0.0 },
        Did { name: "KNK2", did: 0x200B, signed: true, scale: 0.01, offset: 0.0 },
        Did { name: "KNK3", did: 0x200C, signed: true, scale: 0.01, offset: 0.0 },
        Did { name: "KNK4", did: 0x200D, signed: true, scale: 0.01, offset: 0.0 },
        Did { name: "CAM_IN", did: 0x201E, signed: true, scale: 0.1, offset: 0.0 },
        Did { name: "CAM_EX", did: 0x201A, signed: true, scale: 0.1, offset: 0.0 },
        Did { name: "CAM_IN_SP", did: 0x201D, signed: true, scale: 0.1, offset: 0.0 },
        Did { name: "CAM_EX_SP", did: 0x2019, signed: true, scale: 0.1, offset: 0.0 },
        Did { name: "TOIL", did: 0x202F, signed: false, scale: 0.1, offset: -273.14 },
        // Full-load operating point (FR p.3987–4045 DID chapter; scaling as `log_tools::channels`)
        Did { name: "TIA", did: 0x1001, signed: false, scale: 0.75, offset: -48.0 },
        Did { name: "TIA_CHA", did: 0x1041, signed: true, scale: 0.005859375, offset: 144.0 },
        Did { name: "MAF_CYL", did: 0x2032, signed: false, scale: 1.0, offset: 0.0 },
        Did { name: "MAP_SP", did: 0x39C1, signed: false, scale: 0.1, offset: 0.0 },
        Did { name: "TQ_ENG", did: 0x437C, signed: true, scale: 0.1, offset: 0.0 },
        Did { name: "TQ_REQ", did: 0x4380, signed: true, scale: 0.1, offset: 0.0 },
        Did { name: "TCO", did: 0x11CD, signed: false, scale: 1.0, offset: -40.0 },
        Did { name: "LAM_SP", did: 0xF444, signed: false, scale: 1.0 / 32768.0, offset: 0.0 },
        Did { name: "TQI_POW_MAX", did: 0x203F, signed: false, scale: 0.1, offset: 0.0 },
        Did { name: "TQ_LOSS_ENG", did: 0x209C, signed: false, scale: 0.1, offset: 0.0 },
        Did { name: "TQ_LOSS_ADD", did: 0x209B, signed: false, scale: 0.1, offset: 0.0 },
        Did { name: "IGA_BAS_COR_MV", did: 0x2949, signed: true, scale: 0.01, offset: 0.0 },
    ];

    /// Read-only knock observables of SC800S50 (RAM, docs/SIMOS18_GPTA_IO_RE.md §12.7): KNKS[seg] and NL[seg] (u16,
    /// 5/65536 V), segment order = firing order (seg 0..3 = cylinders 1, 3, 4, 2).
    const KNKS: u32 = 0xD001_4274;
    const NL: u32 = 0xD001_42B2;
    const SEG_CYL: [usize; 4] = [0, 2, 3, 1];

    /// CAM_VCP [intake, exhaust] actual and set point (i16, 1/128 °CRK; docs/SIMOS18_GPTA_IO_RE.md §12.8).
    const CAM_VCP: u32 = 0xD001_56B0;
    const CAM_SP: u32 = 0xD001_569C;

    fn cam_ram(s: &StudioSession, base: u32) -> [f64; 2] {
        let b = s.board.read_mem(base, 4).unwrap_or(vec![0; 4]);
        [i16::from_le_bytes([b[0], b[1]]) as f64 / 128.0, i16::from_le_bytes([b[2], b[3]]) as f64 / 128.0]
    }

    fn knock_ram(s: &StudioSession, base: u32) -> [f64; 4] {
        let b = s.board.read_mem(base, 8).unwrap_or(vec![0; 8]);
        let mut v = [0.0; 4];
        for seg in 0..4 {
            v[SEG_CYL[seg]] = u16::from_le_bytes([b[2 * seg], b[2 * seg + 1]]) as f64 * 5.0 / 65536.0;
        }
        v
    }

    /// One ReadDataByIdentifier per call in rotation; returns (index, value) when an answer arrived.
    struct Poller {
        next: usize,
        pending: Option<(usize, f64)>,
        last: Vec<Option<f64>>,
    }

    impl Poller {
        fn poll(&mut self, s: &mut StudioSession, t: f64) {
            if let Some((k, t0)) = self.pending {
                if s.uds_ecu.status.pending && t - t0 < 0.5 {
                    return;
                }
                self.pending = None;
                let r: Vec<u8> = s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
                let d = &DIDS[k];
                if r.len() >= 4 && r[0] == 0x62 && r[1..3] == d.did.to_be_bytes() {
                    let raw = match (r.len(), d.signed) {
                        (4, _) => r[3] as f64, // one-byte DIDs (TIA, TCO)
                        (_, true) => i16::from_be_bytes([r[3], r[4]]) as f64,
                        _ => u16::from_be_bytes([r[3], r[4]]) as f64,
                    };
                    self.last[k] = Some(raw * d.scale + d.offset);
                }
            }
            let k = self.next % DIDS.len();
            self.next = k + 1;
            let did = DIDS[k].did.to_be_bytes();
            let f = s.uds_ecu.request("ReadDataByIdentifier", &[0x22, did[0], did[1]]);
            s.board.inject_can_rx(&f);
            self.pending = Some((k, t));
        }

        fn get(&self, name: &str) -> Option<f64> {
            DIDS.iter().position(|d| d.name == name).and_then(|k| self.last[k])
        }
    }

    type WotRow = (f64, Option<f64>, Option<f64>, f64, f64, [Option<f64>; 4], [f64; 2], [f64; 2], [u64; 4]);

    /// Samples every 0.5 s of the WOT phase: (t, firmware PUT/PUT_SP over UDS, RAM POSN_BPA/POSN_SP_BPA, knock retard
    /// per cylinder over UDS, CAM_VCP/set point [intake, exhaust] from RAM, plant knock-event counts).
    #[derive(Default)]
    struct Report {
        rows: Vec<WotRow>,
        /// Firmware oil temperature (DID 0x202F) of each row.
        toil: Vec<Option<f64>>,
        resets: u32,
    }

    /// SC800S50 RAM (read-only observation, docs/SIMOS18_GPTA_IO_RE.md §12.6): POSN_BPA / POSN_SP_BPA, i16, 100/16384 %.
    const POSN_BPA: u32 = 0xD001_1D8A;
    const POSN_SP_BPA: u32 = 0xD001_1D96;

    fn ram_i16(s: &StudioSession, a: u32) -> f64 {
        s.board.read_mem(a, 2).map(|b| i16::from_le_bytes([b[0], b[1]]) as f64).unwrap_or(f64::NAN)
    }

    /// One tester request, answered within 1 s (a `0x78` "response pending" is returned as it stands).
    fn uds(s: &mut StudioSession, req: &[u8]) -> Vec<u8> {
        let f = s.uds_ecu.request("tester", req);
        s.board.inject_can_rx(&f);
        for _ in 0..100 {
            s.step(0.01);
            if !s.uds_ecu.status.pending {
                break;
            }
        }
        s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect()
    }

    /// Workshop coding of the engine ECU for the drivetrain on the bench (tester over UDS, as VCDS does it; the firmware
    /// checks the value and takes it itself). Only the four-wheel-drive cell is touched: coding cell 23 = byte 5 bit 1 of
    /// the long coding, DID 0x0600 (264H cell table built at `0x801BADAC`; FR p.24789: LV_VAR_4WD = VAR_CODE[5] cell 23).
    /// The bench ECU comes from a Golf 7 R 4Motion (AGENTS.md §3.1) and is coded 4WD; SC8O40 then takes the CJSB 4×4
    /// full-load torque (`ip_tq_pow_max_at[·][2]` 280 Nm via `id_idx_tq_pow_max_at`[DCT_4WD]) instead of CJSA's 250 Nm.
    /// Sequence (VERIFIED-T): EOL session 0x40 (writes are refused with NRC 0x31 in 0x03), repair-shop code 0xF198, the
    /// coding; the variant variables are taken at the next KL15 off→on (FR p.24789; CALID transmission letter J/K, DID
    /// 0xF804). The firmware writes the coding to its DFlash itself (`0x78`, then `6E 06 00` within ~1 s; test
    /// `ecu_coding_survives_power_cycle`).
    fn code_ecu_drivetrain(s: &mut StudioSession, awd: bool) -> String {
        let cur = uds(s, &[0x22, 0x06, 0x00]);
        if cur.len() < 13 || cur[..3] != [0x62, 0x06, 0x00] {
            return format!("ECU coding: not readable ({cur:02X?})");
        }
        if (cur[8] & 0x02 != 0) == awd {
            return format!("ECU coding {:02X?}: four-wheel-drive cell already {}", &cur[3..13], awd as u8);
        }
        let mut code = cur[3..13].to_vec();
        code[5] ^= 0x02;
        for r in [&[0x10, 0x03][..], &[0x10, 0x01], &[0x10, 0x40], &[0x2E, 0xF1, 0x98, 0, 0, 0, 0, 0, 1]] {
            uds(s, r);
        }
        let w = uds(s, &[[0x2E, 0x06, 0x00].as_slice(), &code].concat());
        if w.first() != Some(&0x6E) && w != [0x7F, 0x2E, 0x78] {
            uds(s, &[0x10, 0x01]);
            return format!("ECU coding {:02X?}: the firmware refused {code:02X?} ({w:02X?}), coding unchanged", &cur[3..13]);
        }
        // KL15 off → after-run → on: the firmware re-reads its variant coding
        s.plant.driver.ignition_on = false;
        (0..500).for_each(|_| s.step(0.01));
        s.plant.driver.ignition_on = true;
        (0..100).for_each(|_| s.step(0.01));
        let calid = uds(s, &[0x22, 0xF8, 0x04]);
        format!(
            "ECU coding {:02X?} → {code:02X?} (4WD cell {}; write answer {w:02X?}); CALID {:?}",
            &cur[3..13],
            awd as u8,
            String::from_utf8_lossy(calid.get(4..).unwrap_or_default())
        )
    }

    /// The engine ECU stores a workshop coding in its DFlash itself: `2E 0600` (EOL session, F198 first) is answered
    /// positively once the firmware's NVM write completed on the emulated flash (it answered `0x78` for ever while the
    /// flash model rejected its `5554:5E` / `SA:40` cycles), and the new coding is read back after KL15 off→on and after
    /// a power cycle that keeps the DFlash (bench battery disconnected and reconnected). Image `SIMEMU_IMAGE` (default
    /// the stock SC8O40 `images/8V0906264H__0005.bin`; the bench SC8S50 refuses its 4WD → FWD recoding with NRC 0x22).
    #[test]
    #[ignore]
    fn ecu_coding_survives_power_cycle() {
        let Some(root) = root_or_skip() else { return };
        let img = std::env::var("SIMEMU_IMAGE").unwrap_or_else(|_| "8V0906264H__0005.bin".into());
        if !crate::paths::has_firmware(&root, &img) {
            eprintln!("SKIP: {img} not found");
            return;
        }
        let mut s = StudioSession::new(&root).expect("session");
        s.tcu = None;
        s.cluster_tcu = None;
        s.load_image(&img).expect("image");
        s.plant.driver.ignition_on = true;
        s.plant.driver.selector = Selector::P;
        (0..300).for_each(|_| s.step(0.01));
        let read = |s: &mut StudioSession| uds(s, &[0x22, 0x06, 0x00]);
        let cur = read(&mut s);
        assert_eq!(cur.get(..3), Some(&[0x62, 0x06, 0x00][..]), "coding readable: {cur:02X?}");
        let mut code = cur[3..].to_vec();
        code[5] ^= 0x02; // four-wheel-drive cell (see `code_ecu_drivetrain`)
        for r in [&[0x10, 0x03][..], &[0x10, 0x01], &[0x10, 0x40], &[0x2E, 0xF1, 0x98, 0, 0, 0, 0, 0, 1]] {
            uds(&mut s, r);
        }
        let writes = s.board.hw().flashc.dflash_pages_programmed;
        let mut w = uds(&mut s, &[[0x2E, 0x06, 0x00].as_slice(), &code].concat());
        for _ in 0..1000 {
            if !s.uds_ecu.status.pending {
                break;
            }
            s.step(0.01);
            w = s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
        }
        let fc = &s.board.hw().flashc;
        eprintln!("write answer {w:02X?}; DFlash pages {} erases {} sequence errors {}", fc.dflash_pages_programmed - writes, fc.dflash_erases, fc.sequence_errors);
        assert_eq!(w, [0x6E, 0x06, 0x00], "coding write completes");
        assert!(fc.dflash_pages_programmed > writes, "the firmware wrote the coding to DFlash");
        assert_eq!(fc.sequence_errors, 0);
        uds(&mut s, &[0x10, 0x01]);
        s.plant.driver.ignition_on = false;
        (0..500).for_each(|_| s.step(0.01));
        s.plant.driver.ignition_on = true;
        (0..300).for_each(|_| s.step(0.01));
        assert_eq!(read(&mut s)[3..], code[..], "after KL15 off→on");
        // battery off and on: the board is rebuilt from the DFlash the firmware left
        s.plant.driver.ignition_on = false;
        (0..500).for_each(|_| s.step(0.01));
        s.ecu_nvm = s.board.hw().dflash.to_vec();
        s.power_cycle_ecu().expect("power-up");
        s.plant.driver.ignition_on = true;
        (0..300).for_each(|_| s.step(0.01));
        assert_eq!(read(&mut s)[3..], code[..], "after a power cycle");
    }

    /// The bench run: warm-up state `t_engine` (°C), start, AC dyno to `rpm_hold`, pedal 100 % from 10 s, fuel switch to
    /// `ron_knock` at `t_knock`, `secs` total.
    fn run(root: &std::path::Path, t_engine: f64, rpm_hold: f64, secs: f64, t_knock: f64, ron_knock: f64) -> Report {
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        let slug = std::env::var("SIMEMU_ENGINE").unwrap_or_else(|_| "2.0t_is38".into());
        let mut s = StudioSession::new(root).expect("session");
        s.tcu = None;
        s.cluster_tcu = None;
        if std::env::var_os("SIMEMU_BUILD").is_none() {
            s.plant.engine_profile = EngineProfile::from_slug(&slug).expect("engine profile"); // else the bench build's car
        }
        if let Ok(d) = std::env::var("SIMEMU_DRIVETRAIN") {
            s.plant.drivetrain_profile = DrivetrainProfile::from_slug(&d).expect("drivetrain profile"); // the car the engine came from
        }
        s.plant.reset();
        s.plant.soak(t_engine);
        s.plant.bench = BenchMode::EngineDyno { target_rpm: 9000.0 };
        s.plant.test_cell_fan_ms = env("SIMEMU_CELL_FAN_MS", s.plant.test_cell_fan_ms);
        s.eot_clock_s = 300.0;
        match std::env::var("SIMEMU_IMAGE") { Ok(img) => s.load_image(&img).expect("image"), Err(_) => s.power_cycle_ecu().expect("power-up") };
        s.plant.driver.ignition_on = true;
        s.plant.driver.selector = Selector::P;
        // Another software in the bench ECU (`SIMEMU_IMAGE`) or another car (`SIMEMU_DRIVETRAIN`): the workshop codes the ECU
        // for the car, as after reflashing; the bench image keeps the coding of the car it was read from.
        let recode = std::env::var_os("SIMEMU_IMAGE").is_some() || std::env::var_os("SIMEMU_DRIVETRAIN").is_some();
        if recode && std::env::var("SIMEMU_ECU_CODING").map_or(true, |v| v != "dump") {
            let awd = s.plant.drivetrain_profile.awd;
            eprintln!("{}", code_ecu_drivetrain(&mut s, awd));
        }
        let mut poll = Poller { next: 0, pending: None, last: vec![None; DIDS.len()] };
        let wall = std::time::Instant::now();
        let mut motor: Option<f64> = None;
        let mut rep = Report::default();
        let mut fired = false;
        for i in 0..(secs / 0.01) as usize {
            let t = (i + 1) as f64 * 0.01;
            s.plant.driver.brake_pct = if (0.5..8.0).contains(&t) { 60.0 } else { 0.0 };
            // the operator holds the start request from 1 s until the engine runs on its own (≤ 5 s of cranking)
            fired |= s.plant.engine.rpm() > 600.0;
            s.plant.driver.start_button = (1.0..6.0).contains(&t) && !fired;
            if t >= 8.0 {
                // AC dyno in speed mode: picks the engine up at its own speed and ramps it at 500 rpm/s.
                let n = motor.get_or_insert(s.plant.engine.rpm().max(700.0));
                *n = (*n + 500.0 * 0.01).min(rpm_hold);
                s.plant.motoring_rpm = Some(*n);
            }
            s.plant.driver.pedal_pct = if t >= 10.0 { 100.0 } else { 0.0 };
            if (t - t_knock).abs() < 0.005 {
                s.plant.fuel_ron = ron_knock;
                eprintln!("t {t:.2}: test-cell fuel switched to RON {ron_knock}");
            }
            s.step(0.01);
            poll.poll(&mut s, t);
            if i % 50 == 49 {
                let tel = s.plant.telemetry();
                let o = &s.actuators.ecu;
                let g = |n: &str| poll.get(n).map_or("  -  ".to_string(), |v| format!("{v:6.1}"));
                eprintln!(
                    "t {t:5.1} rpm {:5.0} tq {:5.0} MAP {:5.1} PUT {:5.1}/{} sp {} | WG pos {:5.1}% sp {} fw {} duty {:?} | spark {:?} | knk {}/{}/{}/{} KNKS {:.2?} NL {:.2?} plant {:.1?} ev {:?} | cam {:.1?} fw RAM/{:.1?} sp {:.1?}/ duty {:?} | λ {:.2} rail {:.1} fails {}",
                    tel.rpm,
                    tel.torque_brake_nm,
                    tel.map_kpa,
                    tel.put_kpa,
                    g("PUT"),
                    g("PUT_SP"),
                    tel.wastegate_pos_pct,
                    g("POSN_SP_BPA"),
                    g("POSN_BPA"),
                    o.wastegate_duty.map(|d| (d * 100.0).round() / 100.0),
                    o.ignition.map(|c| c.map(|c| (c.spark_btdc_deg * 10.0).round() / 10.0)),
                    g("KNK1"),
                    g("KNK2"),
                    g("KNK3"),
                    g("KNK4"),
                    knock_ram(&s, KNKS),
                    knock_ram(&s, NL),
                    tel.knock_amp_bar,
                    s.plant.engine.vib.knock_events,
                    tel.cam_phase_deg,
                    cam_ram(&s, CAM_VCP),
                    cam_ram(&s, CAM_SP),
                    o.cam_phaser_duty,
                    tel.lambda_sensor,
                    tel.rail_mpa,
                    tel.failures.len()
                );
                eprintln!("    firmware TOIL {} °C (plant oil {:.1} °C)", g("TOIL"), tel.oil_c);
                let spark = o.ignition.iter().flatten().map(|c| c.spark_btdc_deg).sum::<f64>() / 4.0;
                eprintln!(
                    "    op: IAT plant {:.1} fw {} | charge air plant {:.1} (compr out {:.1}) fw {} | air plant {:.0} mg fw {} | MAP_SP {} | TQ fw {} req {} | TCO fw {} | λ SP {} | spark {spark:.1} | TQI_POW_MAX {} loss eng {} add {} | IGA_BAS_COR_MV {} | cam DID in {}/{} ex {}/{}",
                    tel.iat_c,
                    g("TIA"),
                    tel.charge_air_c,
                    tel.compressor_out_c,
                    g("TIA_CHA"),
                    tel.air_mg_stroke,
                    poll.get("MAF_CYL").map_or("  -  ".into(), |v| format!("{:.0}", v / 3.6 * 1e3 / (tel.rpm.max(1.0) / 30.0))),
                    g("MAP_SP"),
                    g("TQ_ENG"),
                    g("TQ_REQ"),
                    g("TCO"),
                    poll.get("LAM_SP").map_or("  -  ".into(), |v| format!("{v:.3}")),
                    g("TQI_POW_MAX"),
                    g("TQ_LOSS_ENG"),
                    g("TQ_LOSS_ADD"),
                    g("IGA_BAS_COR_MV"),
                    g("CAM_IN"),
                    g("CAM_IN_SP"),
                    g("CAM_EX"),
                    g("CAM_EX_SP"),
                );
                if t >= 10.0 {
                    let knk = ["KNK1", "KNK2", "KNK3", "KNK4"].map(|n| poll.get(n));
                    let q = 100.0 / 16384.0;
                    rep.toil.push(poll.get("TOIL"));
                    rep.rows.push((t, poll.get("PUT"), poll.get("PUT_SP"), ram_i16(&s, POSN_BPA) * q, ram_i16(&s, POSN_SP_BPA) * q, knk, cam_ram(&s, CAM_VCP), cam_ram(&s, CAM_SP), s.plant.engine.vib.knock_events));
                }
            }
        }
        eprintln!("{secs} s simulated in {:.1} s wall", wall.elapsed().as_secs_f64());
        rep.resets = s.board.hw().reset_count;
        let f = s.uds_ecu.request("ReadDTCInformation", &[0x19, 0x02, 0x2F]);
        s.board.inject_can_rx(&f);
        for _ in 0..400 {
            s.step(0.01);
            if !s.uds_ecu.status.pending {
                break;
            }
        }
        eprintln!("fault memory: {:?}", s.uds_ecu.status.dtcs);
        rep
    }

    fn root_or_skip() -> Option<std::path::PathBuf> {
        let r = root();
        if r.is_none() {
            eprintln!("SKIP: {S50_FLASH} not found");
        }
        r
    }

    /// Warm engine (90 °C), 3000 rpm WOT: the firmware closes the wastegate and its boost and position controllers
    /// follow their set points; at RON 95 the emulated knock sensor never makes it retard (no false knock), after the
    /// fuel switch to RON 78 the plant knocks and the firmware retards ignition per cylinder.
    #[test]
    #[ignore]
    fn engine_dyno_wot_boost_and_knock() {
        let Some(root) = root_or_skip() else { return };
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        let (t_knock, ron) = (env("SIMEMU_KNOCK_T", 26.0), env("SIMEMU_KNOCK_RON", 78.0));
        let r = run(&root, env("SIMEMU_ENGINE_C", 90.0), env("SIMEMU_DYNO_RPM", 3000.0), env("SIMEMU_DYNO_S", 34.0), t_knock, ron);
        assert_eq!(r.resets, 0, "no ECU reset");
        let win = |a: f64, b: f64| r.rows.iter().filter(move |x| x.0 >= a && x.0 < b);
        // boost: firmware PUT vs PUT_SP (UDS), wastegate position vs its set point (RAM), steady WOT before the switch
        let put: Vec<f64> = win(18.0, t_knock).filter_map(|x| Some((x.1? - x.2?).abs() / x.2?)).collect();
        let wg: Vec<f64> = win(18.0, t_knock).map(|x| (x.3 - x.4).abs()).collect();
        let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len().max(1) as f64;
        eprintln!("boost |PUT−SP|/SP mean {:.3}, wastegate |POSN−SP| mean {:.2} %", mean(&put), mean(&wg));
        assert!(!put.is_empty() && mean(&put) < 0.06, "boost follows its set point: {put:?}");
        assert!(mean(&wg) < 5.0, "wastegate follows its set point: {wg:?}");
        // knock
        let retard = |a: f64, b: f64| win(a, b).flat_map(|x| x.5.iter().flatten().copied()).fold(0.0f64, f64::min);
        let (before, after) = (retard(14.0, t_knock), retard(t_knock + 1.0, t_knock + 8.0));
        let ev = |t: f64| win(t, t + 0.6).next().map(|x| x.8).unwrap_or_default();
        let (e0, e1) = (ev(t_knock - 1.0), ev(t_knock + 7.0));
        eprintln!("knock retard before {before:.1}°, after {after:.1}°; plant knock events in the 8 s after the switch {:?}", std::array::from_fn::<u64, 4, _>(|c| e1[c] - e0[c]));
        assert!(before > -0.8, "no false knock at RON 95: {before}");
        assert!(after <= -1.5, "knock control retards after the fuel switch: {after}");
    }

    /// Cam phasing: the firmware's cam set points are reached and held. Run with the oil at 30 °C. The firmware takes the
    /// solenoid temperature for its hold-duty map `ip_pwm_hld_vcp` from TOIL (FR VVTI 79.15), and on SC8S50 TOIL is its
    /// oil-temperature *model* (CLF_TOIL_SRV_INTL_DI_CONF bit 0: the sender is never used for TOIL, code 0x808DF3E4),
    /// initialised from TOIL_STOP stored in the ECU's NVM (24 °C in the bench DFlash) — see
    /// [`engine_dyno_cam_phasing_hot`] for the warm engine. Must pass on both CPU cores.
    #[test]
    #[ignore]
    fn engine_dyno_cam_phasing() {
        let Some(root) = root_or_skip() else { return };
        let r = run(&root, 30.0, 3000.0, 22.0, 99.0, 95.0);
        assert_eq!(r.resets, 0, "no ECU reset");
        let ok = r.rows.iter().filter(|x| x.0 >= 14.0).filter(|x| (x.6[0] - x.7[0]).abs() < 2.0).count();
        let n = r.rows.iter().filter(|x| x.0 >= 14.0).count();
        eprintln!("intake cam within 2° of its set point in {ok}/{n} samples");
        assert!(n > 0 && ok * 10 >= n * 8, "intake cam set point reached and held");
    }

    /// Hot engine (plant at 90 °C): the firmware's TOIL model starts from its stored TOIL_STOP (≈ 30 °C) and warms with
    /// the running engine (docs/SIMOS18_GPTA_IO_RE.md §12.9); once it has, the hold duty fits the hot coil and the intake
    /// cam holds its set point. `SIMEMU_DYNO_S` (default 240) WOT seconds at 3000 rpm; judged over the last 30 s.
    #[test]
    #[ignore]
    fn engine_dyno_cam_phasing_hot() {
        let Some(root) = root_or_skip() else { return };
        let secs = std::env::var("SIMEMU_DYNO_S").ok().and_then(|v| v.parse().ok()).unwrap_or(240.0);
        let r = run(&root, 90.0, 3000.0, secs, 1e9, 95.0);
        assert_eq!(r.resets, 0, "no ECU reset");
        let last: Vec<usize> = (0..r.rows.len()).filter(|&i| r.rows[i].0 >= secs - 30.0).collect();
        let ok = last.iter().filter(|&&i| (r.rows[i].6[0] - r.rows[i].7[0]).abs() < 3.0).count();
        let toil = last.iter().filter_map(|&i| r.toil[i]).fold(f64::NAN, f64::min);
        eprintln!("last 30 s: firmware TOIL ≥ {toil:.0} °C, intake cam within 3° of its set point in {ok}/{} samples", last.len());
        assert!(toil > 70.0, "firmware oil model warmed up: {toil}");
        assert!(!last.is_empty() && ok * 10 >= last.len() * 8, "intake cam set point held on the warm engine");
    }

    /// Part-load air charge, firmware against plant (docs/SIMOS18_COMBUSTION.md §6.4): the warm engine (90 °C) on the AC
    /// dyno at `SIMEMU_PL_POINTS="rpm,pedal%;…"` (default 1250…2500 rpm at 15/25 % pedal), each held 10 s; over the last
    /// 4 s the firmware's air mass flow MAF_CYL (DID 0x2032, kg/h) against the plant's trapped air, with MAP (DID 0x39C0),
    /// STFT (OBD PID 0x06 via DID 0xF406) and the cam phases, flap and AVS state the firmware set. Prints one row per
    /// point; asserts nothing (measurement).
    #[test]
    #[ignore]
    fn part_load_air_vs_firmware() {
        let Some(root) = root_or_skip() else { return };
        let pts: Vec<(f64, f64)> = std::env::var("SIMEMU_PL_POINTS")
            .ok()
            .map(|v| v.split(';').map(|c| { let x: Vec<f64> = c.split(',').map(|y| y.trim().parse().unwrap()).collect(); (x[0], x[1]) }).collect())
            .unwrap_or(vec![(1250.0, 15.0), (1250.0, 25.0), (1500.0, 15.0), (1500.0, 25.0), (2000.0, 15.0), (2000.0, 25.0), (2500.0, 15.0), (2500.0, 25.0)]);
        let slug = std::env::var("SIMEMU_ENGINE").unwrap_or_else(|_| "2.0t_is38".into());
        let mut s = StudioSession::new(&root).expect("session");
        s.tcu = None;
        s.cluster_tcu = None;
        s.plant.engine_profile = EngineProfile::from_slug(&slug).expect("engine profile");
        s.plant.reset();
        s.plant.soak(90.0);
        s.plant.bench = BenchMode::EngineDyno { target_rpm: 9000.0 };
        s.eot_clock_s = 300.0;
        match std::env::var("SIMEMU_IMAGE") { Ok(img) => s.load_image(&img).expect("image"), Err(_) => s.power_cycle_ecu().expect("power-up") };
        s.plant.driver.ignition_on = true;
        s.plant.driver.selector = Selector::P;
        // DIDs: MAF_CYL, MAP, STFT; TQ_ENG (i16, 0.1 Nm), TQ_LOSS_ENG (|·|, 0.1 Nm), IGA_AV_MV (i16, 0.01 °) — FR p.4025/4044
        let dids: [u16; 6] = [0x2032, 0x39C0, 0xF406, 0x437C, 0x209C, 0x2004];
        let mut last = [f64::NAN; 6];
        let (mut k_did, mut pending) = (0usize, None::<(usize, f64)>);
        let (mut fired, mut motor) = (false, None::<f64>);
        let t_start = 10.0;
        let total = t_start + 10.0 * pts.len() as f64;
        let mut acc: Vec<[f64; 19]> = vec![[0.0; 19]; pts.len()];
        for i in 0..(total / 0.01) as usize {
            let t = (i + 1) as f64 * 0.01;
            s.plant.driver.brake_pct = if (0.5..8.0).contains(&t) { 60.0 } else { 0.0 };
            fired |= s.plant.engine.rpm() > 600.0;
            s.plant.driver.start_button = (1.0..6.0).contains(&t) && !fired;
            let k = ((t - t_start) / 10.0).floor();
            let (rpm, pedal) = if k >= 0.0 { pts[(k as usize).min(pts.len() - 1)] } else { (900.0, 0.0) };
            if t >= 8.0 {
                let n = motor.get_or_insert(s.plant.engine.rpm().max(700.0));
                *n += (rpm - *n).clamp(-5.0, 5.0);
                s.plant.motoring_rpm = Some(*n);
            }
            s.plant.driver.pedal_pct = pedal;
            s.step(0.01);
            // UDS: one ReadDataByIdentifier in flight, rotating
            if let Some((j, t0)) = pending {
                if !s.uds_ecu.status.pending || t - t0 > 0.5 {
                    pending = None;
                    let r: Vec<u8> = s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
                    if r.len() >= 4 && r[0] == 0x62 && r[1..3] == dids[j].to_be_bytes() {
                        last[j] = match (r.len() >= 5, dids[j]) {
                            (true, 0x437C | 0x2004) => i16::from_be_bytes([r[3], r[4]]) as f64,
                            (true, _) => u16::from_be_bytes([r[3], r[4]]) as f64,
                            _ => r[3] as f64,
                        };
                    }
                }
            }
            if pending.is_none() {
                let d = dids[k_did % dids.len()].to_be_bytes();
                let f = s.uds_ecu.request("ReadDataByIdentifier", &[0x22, d[0], d[1]]);
                s.board.inject_can_rx(&f);
                pending = Some((k_did % dids.len(), t));
                k_did += 1;
            }
            let ph = (t - t_start) - 10.0 * k;
            if k >= 0.0 && ph >= 6.0 && i % 10 == 0 && last.iter().all(|x| x.is_finite()) {
                let tel = s.plant.telemetry();
                let ram = s.board.read_mem(0xD001_3EA8, 2).map(|b| u16::from_le_bytes([b[0], b[1]]) as f64 / 32.0).unwrap_or(f64::NAN);
                if i % 200 == 0 { eprintln!("    t {t:.1} MAF_CYL DID {} kg/h, RAM 0xD0013EA8 {ram:.2} kg/h", last[0]); }
                let maf = if (ram - last[0]).abs() < 1.5 { ram } else { last[0] };
                let mg_fw = maf / 3.6 * 1e3 / (tel.rpm / 30.0); // kg/h → mg/stroke (2 strokes per rev of a 4-cylinder)
                let cam = cam_ram(&s, CAM_VCP);
                let a = &mut acc[k as usize];
                let spark = s.actuators.ecu.ignition.iter().flatten().map(|c| c.spark_btdc_deg).sum::<f64>() / 4.0;
                let lam = tel.lambda_cyl.iter().sum::<f64>() / 4.0;
                for (j, v) in [
                    1.0, tel.air_mg_stroke, mg_fw, last[1] / 10.0, (last[2] - 128.0) * 100.0 / 128.0, cam[0], cam[1], s.plant.engine.port_flap_closed, s.plant.engine.vt.avs[0].pos,
                    tel.torque_brake_nm, tel.torque_indicated_nm, tel.torque_friction_nm + tel.torque_pumping_nm + s.plant.engine.flows.torque_aux_nm, last[3] / 10.0, last[4] / 10.0, last[5] / 100.0, spark, lam,
                    tel.torque_friction_nm, tel.torque_pumping_nm,
                ]
                .into_iter()
                .enumerate()
                {
                    a[j] += v;
                }
            }
        }
        // Torque: plant brake / gross indicated / losses (friction + pumping + aux) against the firmware's TQ_ENG, TQ_LOSS_ENG
        // and its indicated sum TQ_ENG + TQ_LOSS_ENG; IGA_AV_MV against the spark the plant received; λ of the cylinders.
        eprintln!("pl,engine,rpm,pedal,plant_mg,fw_mg,fw/plant,map_kpa,stft_pct,cam_in,cam_ex,flaps_closed,avs_lft1,tq_plant,tqi_plant,loss_plant,tq_fw,loss_fw,tqi_fw,tqi_fw/plant,iga_fw,spark_plant,lambda,fric_plant,pump_plant");
        for ((rpm, pedal), a) in pts.iter().zip(&acc) {
            let n = a[0].max(1.0);
            let m: Vec<f64> = a.iter().map(|x| x / n).collect();
            eprintln!(
                "pl,{slug},{rpm:.0},{pedal:.0},{:.1},{:.1},{:.3},{:.1},{:+.1},{:.1},{:.1},{:.2},{:.2},{:.1},{:.1},{:.1},{:.1},{:.1},{:.1},{:.3},{:.1},{:.1},{:.3},{:.1},{:.1}",
                m[1], m[2], m[2] / m[1], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[12] + m[13], (m[12] + m[13]) / m[10], m[14], m[15], m[16], m[17], m[18]
            );
        }
    }
}
