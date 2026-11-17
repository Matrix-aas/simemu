//! Log replay on the virtual engine test cell: a real car log drives the bench, the unmodified firmware
//! runs the engine, and the simulated channels are compared with the logged ones.
//!
//! `simemu replay <log.csv> [--image <bin>] [--profile <slug>]
//!  [--from s] [--to s] [--preroll s]` (`SIMEMU_REPLAY_TRACE=1`: the start every 0.1 s).
//! Images with stale block CRCs / ECM3 sums have them recomputed by `load_image` as a flashing tool does.
//!
//! Bench set-up (test-cell equipment and driver only — AGENTS.md §2.2):
//! - Engine on the test cell with the gearbox removed (`BenchMode::EngineDyno`, eddy absorber idle) and a
//!   4-quadrant AC dyno on the DMF secondary in speed mode (`BenchMode::AcDyno`, finite-stiffness loop) that holds the
//!   logged engine speed — the standard way to replay a road trace on an engine dyno when the vehicle,
//!   gearbox and TCU are not available. The firmware sees the logged speed on its crank sensor and decides
//!   throttle, boost, fuel and spark itself; it never gets a value from the log.
//! - Driver: pedal from the log; brake held while the logged vehicle speed is < 1 km/h (a DSG car cannot
//!   stand still in D otherwise — inferred, the log has no brake channel); selector D after the start.
//! - Start: ignition on, brake, start request (KL50) like `engine_run`, then the firmware starts the engine
//!   on its own during the pre-roll; the replay begins at `--from` of the log after `--preroll` seconds.
//! - Not reproduced, by construction: vehicle speed and gear on CAN (the ECU sees a stationary car without a
//!   TCU: the DQ250 firmware cannot run until its SPI EEPROM dump exists), driveline torque, road load. A
//!   chassis-dyno replay with the DQ250 + Haldex profile needs that TCU and is refused rather than faked.
//!
//! The emulated tester reads the logger's own DIDs (`log_tools::channels`) over UDS 0x22, one DID per
//! request in rotation, so firmware-internal channels (spark, knock retard, λ set point, …) are compared
//! through the same firmware path and scaling as the car log. Plant channels are sampled every 10 ms.

use crate::log_tools::channels::{by_name, Channel, CHANNELS};
use crate::log_tools::csv_log::{interp, LogData};
use crate::log_tools::report::{err_stats, fmt_num, html_page, svg_plot, Series, Span};
use crate::log_tools::{opt, out_dir, STOCK_264H};
use crate::studio::session::StudioSession;
use simemu_physics::{Ambient, BenchMode, DrivetrainProfile, EngineProfile, PlantTelemetry, Selector, ThermalState};
use std::fmt::Write;
use std::path::{Path, PathBuf};

pub const DEFAULT_PROFILE: &str = "cjsa_is38_dq250_awd";
/// Speed slew limit of the AC test-cell dyno, rpm/s. UNVERIFIED (transient engine dynos reach
/// 10 000–20 000 rpm/s; the log's fastest change is ≈ −5000 rpm/s in an upshift).
const DYNO_SLEW_RPM_S: f64 = 10_000.0;

/// `engine+drivetrain` slugs, or a named combination.
pub fn profile(slug: &str) -> Result<(EngineProfile, DrivetrainProfile), String> {
    let (e, d) = match slug {
        DEFAULT_PROFILE => ("cjsa_is38", "dq250_0d9_awd"),
        s => s.split_once('+').ok_or_else(|| format!("unknown profile {s} (use {DEFAULT_PROFILE} or <engine>+<drivetrain>)"))?,
    };
    Ok((EngineProfile::from_slug(e).ok_or(format!("unknown engine {e}"))?, DrivetrainProfile::from_slug(d).ok_or(format!("unknown drivetrain {d}"))?))
}

/// Firmware-internal channels the tester polls (the plant has no counterpart for them, or the firmware's
/// own reading is wanted next to the plant value).
const TESTER_CHANNELS: &[&str] = &[
    "Ign Avg", "Knock Cyl 1", "Knock Cyl 2", "Knock Cyl 3", "Knock Cyl 4", "Lambda", "Lambda SP", "MAP", "MAP SP", "PUT SP",
    "Torque", "Torque Req", "Inj PW DI", "Inj PW MPI", "FP DI SP", "Wastegate SP", "STFT", "Misfires", "Engine Speed", "HPFP Eff Vol",
];

/// UDS 0x22 poller in rotation; negative responses drop the DID from the rotation (also the tune check's tester,
/// `studio::tune_check`).
pub(crate) struct Tester {
    list: Vec<&'static Channel>,
    next: usize,
    pending: Option<(usize, f64)>,
    data: Vec<Vec<(f64, f64)>>,
    refused: Vec<String>,
    timeouts: usize,
}

impl Tester {
    fn new() -> Self {
        Self::with(TESTER_CHANNELS)
    }

    /// A tester reading these logger channels (`log_tools::channels` names with a DID).
    pub(crate) fn with(names: &[&str]) -> Self {
        let list: Vec<&Channel> = names.iter().filter_map(|n| by_name(n)).filter(|c| c.did.is_some()).collect();
        Self { data: vec![Vec::new(); list.len()], list, next: 0, pending: None, refused: Vec::new(), timeouts: 0 }
    }

    /// Last value read of channel `name`.
    pub(crate) fn last(&self, name: &str) -> Option<f64> {
        let k = self.list.iter().position(|c| c.name == name)?;
        self.data[k].last().map(|x| x.1)
    }

    /// Sends the next request once the previous answer is in (or 0.5 s passed); `t` = plant time.
    pub(crate) fn poll(&mut self, s: &mut StudioSession, t: f64) {
        if let Some((k, t0)) = self.pending {
            if s.uds_ecu.status.pending && t - t0 < 0.5 {
                return;
            }
            self.pending = None;
            let r: Vec<u8> = s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
            let ch = self.list[k];
            let did = ch.did.map(|d| d.0.to_be_bytes()).unwrap_or_default();
            if r.len() >= 3 && r[0] == 0x62 && r[1..3] == did {
                if let Some(v) = ch.decode(&r[3..]) {
                    self.data[k].push((t, v));
                }
            } else if r.first() == Some(&0x7F) && r.get(2) != Some(&0x78) {
                self.refused.push(format!("{} (DID {:04X}): NRC 0x{:02X}", ch.name, u16::from_be_bytes(did), r.get(2).copied().unwrap_or(0)));
                self.list.remove(k);
                self.data.remove(k);
                self.next = 0;
            } else {
                self.timeouts += 1;
            }
        }
        if self.list.is_empty() {
            return;
        }
        let k = self.next % self.list.len();
        self.next = k + 1;
        let did = self.list[k].did.unwrap().0.to_be_bytes();
        let f = s.uds_ecu.request("ReadDataByIdentifier", &[0x22, did[0], did[1]]);
        s.board.inject_can_rx(&f);
        self.pending = Some((k, t));
    }
}

type CylExtra = (&'static str, &'static str, fn(&PlantTelemetry) -> [f64; 4]);

/// Per-cylinder plant quantities recorded for the failure analysis.
const CYL_EXTRAS: &[CylExtra] = &[
    ("piston crown", "°C", |t| t.piston_crown_c),
    ("ring-land damage (Miner)", "-", |t| t.piston_damage),
    ("peak cylinder pressure", "bar", |t| t.cyl_peak_bar),
    ("knock amplitude", "bar", |t| t.knock_amp_bar),
    ("knock integral (Livengood–Wu)", "-", |t| t.knock_integral),
    ("exhaust valve", "°C", |t| t.exhaust_valve_c),
    ("cylinder λ", "-", |t| t.lambda_cyl),
    ("IMEP", "bar", |t| t.imep_bar),
    ("rod damage (Miner)", "-", |t| t.rod_damage),
    ("peak local wall pressure (incl. knock)", "bar", |t| t.combustion.p_local_bar),
    ("ring-land stress / UTS (max so far)", "-", |t| t.combustion.land_stress_max),
    ("top land / ring groove", "°C", |t| t.combustion.piston_groove_c),
    ("pre-ignitions (all, cumulative)", "-", |t| t.combustion.preign_events.map(f64::from)),
    ("LSPI events (cumulative)", "-", |t| t.combustion.lspi_events.map(f64::from)),
    ("knocking cycles (cumulative)", "-", |t| t.combustion.knock_events.map(f64::from)),
    ("spark-plug tip", "°C", |t| t.combustion.plug_tip_c),
];

/// The firmware's own controlled-reset record (SC8O30 routine `0x801F309C` writes it to standby LMU before
/// SWRSTREQ, VERIFIED-S): reason code @0xB0000038, entry id @0xB000003D (0xA2), validity guards
/// @0xB000014C/150 = 0xDEADBEEF/0x21524110, saved return addresses @0xB0000070..0xB0000097.
fn reset_record(s: &StudioSession) -> String {
    let rd = |a: u32| s.board.read_mem(a, 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]])).unwrap_or(0);
    let stack: Vec<String> = (0..10).map(|i| rd(0xB000_0070 + 4 * i)).filter(|&a| a != 0).map(|a| format!("{a:08X}")).collect();
    format!(
        "reason 0x{:X}, entry 0x{:02X}, guards {:08X}/{:08X}, return addresses [{}]",
        rd(0xB000_0038),
        rd(0xB000_003C) >> 8 & 0xFF,
        rd(0xB000_014C),
        rd(0xB000_0150),
        stack.join(" ")
    )
}

/// Fault memory through the session's tester (UDS 19 02 FF), as SAE codes with the raw record.
fn read_dtcs(s: &mut StudioSession) -> Vec<String> {
    let f = s.uds_ecu.request("ReadDTCInformation", &[0x19, 0x02, 0xFF]);
    s.board.inject_can_rx(&f);
    for _ in 0..400 {
        s.step(0.01);
        if !s.uds_ecu.status.pending {
            break;
        }
    }
    let sae = |d: &str| -> String {
        let b: Vec<u8> = (0..3).filter_map(|i| d.get(2 * i..2 * i + 2).and_then(|x| u8::from_str_radix(x, 16).ok())).collect();
        match b.as_slice() {
            [a, c, _] => format!("{}{:04X}", "PCBU".as_bytes()[(a / 64) as usize] as char, (((*a as u16) % 64) * 256) + *c as u16),
            _ => d.to_string(),
        }
    };
    // Only entries with testFailed / pending / confirmed / testFailedSinceLastClear (status & 0x2F).
    let failed = |d: &&String| d.rsplit("0x").next().and_then(|x| u8::from_str_radix(x, 16).ok()).is_some_and(|st| st & 0x2F != 0);
    let mut v: Vec<String> = s.uds_ecu.status.dtcs.iter().filter(failed).map(|d| format!("{} ({d})", sae(d))).collect();
    v.push(format!("{} entries in total", s.uds_ecu.status.dtcs.len()));
    v
}

pub fn cmd_replay(root: &Path, args: &[String]) -> Result<(), String> {
    let log_path = PathBuf::from(args.first().ok_or("usage: simemu replay <log.csv> [--image <bin>] [--profile <slug>] [--from s] [--to s] [--preroll s]")?);
    let log = LogData::load(&log_path)?;
    let image = opt(args, "--image").unwrap_or(STOCK_264H);
    let slug = opt(args, "--profile").unwrap_or(DEFAULT_PROFILE);
    if opt(args, "--bench") == Some("chassis") {
        return Err("chassis-dyno replay needs the DQ250 TCU firmware running (its SPI EEPROM dump is missing): \
                    without it no clutch is ever pressurised and the car cannot be driven on the rollers"
            .into());
    }
    let num = |k: &str, d: f64| opt(args, k).and_then(|v| v.parse::<f64>().ok()).unwrap_or(d);
    let (from, to, preroll) = (num("--from", 0.0), num("--to", log.duration()).min(log.duration()), num("--preroll", 8.0));
    let col = |n: &str| log.col(&[n]).map(|v| v.to_vec()).unwrap_or_else(|| vec![f64::NAN; log.t.len()]);
    let (l_rpm, l_pedal, l_v) = (col("Engine Speed"), col("Pedal Pos"), col("Vehicle Speed"));
    let first = |n: &str, d: f64| col(n).into_iter().find(|x| x.is_finite()).unwrap_or(d);

    // ---- session: image, plant profile, ambient and warm engine from the log ---------------------
    let wall = std::time::Instant::now();
    let mut s = StudioSession::new(root)?;
    s.load_image(image)?;
    // Stale block CRCs / ECM3 sums are verified and recomputed by `load_image` (post-programming step,
    // AGENTS.md §2.4 item 3); its notes are reported with the replay events.
    let flash_note: Option<String> = (!s.image_notes.is_empty()).then(|| s.image_notes.join("; "));
    s.tcu = None;
    s.tcu_error = Some("DQ250 TCU not emulated (SPI EEPROM dump missing)".into());
    s.cluster_tcu = None;
    // SIMEMU_BUILD / SIMEMU_ENGINE / SIMEMU_TURBO (crate::studio::bench_build) replace --profile; the session already
    // created its plant from that build.
    if crate::studio::bench_build::build_from_env()?.is_none() {
        let (ep, dp) = profile(slug)?;
        s.plant.engine_profile = ep;
        s.plant.drivetrain_profile = dp;
        s.bench_build.legacy = Some(format!("replay --profile {slug}"));
    }
    let baro = first("Ambient Press", 101.325);
    s.plant.ambient = Ambient { temp_c: first("Ambient Temp", 15.0), altitude_m: 44_330.0 * (1.0 - (baro / 101.325).powf(1.0 / 5.255)), humidity_pct: 75.0 };
    s.plant.reset();
    let (clt, oil) = (first("Coolant Temp", 90.0), first("Oil Temp", 90.0));
    s.plant.thermal = ThermalState { oil_c: oil, ..ThermalState::uniform(clt) };
    s.plant.engine.lube.bearings.iter_mut().for_each(|b| b.t_k = oil + 273.15);
    s.plant.bench = BenchMode::EngineDyno { target_rpm: 9000.0 };
    s.plant.driver.selector = Selector::P;
    s.plant.driver.ignition_on = true;
    eprintln!("replay: {} on {image} ({:?} CPU), profile {slug}; image notes: {:?}", log.source, s.board.backend(), s.image_notes);

    // ---- pre-roll: start by the firmware --------------------------------------------------------
    let mut events: Vec<String> = flash_note.into_iter().collect();
    let src = s.bench_build.legacy.clone().unwrap_or_else(|| format!("bench build {}", s.bench_build.active.name));
    events.push(format!("plant: {} | {} ({src})", s.plant.engine_profile.name, s.plant.drivetrain_profile.name));
    if let Some(c) = &s.checksums {
        let bad: Vec<String> = c.entries.iter().filter(|e| !e.ok()).map(|e| format!("{} stored 0x{:X} computed 0x{:X}", e.name, e.stored, e.computed)).collect();
        events.push(format!("image checksums at power-on: {}", if bad.is_empty() { "all valid".into() } else { bad.join("; ") }));
    }
    let mut ran: Option<f64> = None;
    for i in 0..(preroll / 0.01) as usize {
        let t = (i + 1) as f64 * 0.01;
        s.plant.driver.brake_pct = if t >= 0.5 { 60.0 } else { 0.0 };
        // The driver holds the start request until the engine runs (key held / button held), at most 6 s.
        s.plant.driver.start_button = (1.0..6.0).contains(&t) && ran.is_none_or(|t0| t - t0 < 0.3);
        s.step(0.01);
        let tel = s.plant.telemetry();
        if ran.is_none() && tel.engine_state == "running" && tel.rpm > 500.0 {
            ran = Some(t);
            events.push(format!("pre-roll {t:.2} s: engine fires, {:.0} rpm", tel.rpm));
        }
        // SIMEMU_REPLAY_TRACE=1: the start every 0.1 s (decoded outputs, plant cylinders).
        if i % 100 == 99 || (std::env::var_os("SIMEMU_REPLAY_TRACE").is_some() && i % 10 == 9) {
            let o = &s.actuators.ecu;
            let hw = s.board.hw();
            eprintln!(
                "  pre-roll {t:.1} s: resets {} {:?} | {} {:.0} rpm λ {:.2} MAP {:.0} kPa rail {:.1} MPa | starter {:?} spark {:?} inj c1 {:?} etc {:?} | cyl λ {:?} IMEP {:?}",
                hw.reset_count,
                hw.reset_log.last(),
                tel.engine_state,
                tel.rpm,
                tel.lambda_sensor,
                tel.map_kpa,
                tel.rail_mpa,
                o.starter_relay,
                o.ignition.map(|c| c.map(|c| c.spark_btdc_deg.round())),
                o.injection[0].iter().flatten().map(|p| (p.soi_btdc_deg.round(), (p.width_ms * 100.0).round() / 100.0)).collect::<Vec<_>>(),
                o.etc_duty.map(|d| (d * 100.0).round() / 100.0),
                tel.lambda_cyl.map(|x| (x * 100.0).round() / 100.0),
                tel.imep_bar.map(|x| x.round()),
            );
        }
    }
    let tel = s.plant.telemetry();
    if s.board.hw().reset_count > 0 {
        events.push(format!("firmware reset record (LMU, read-only): {}", reset_record(&s)));
    }
    events.push(format!(
        "end of pre-roll: {} at {:.0} rpm, ECU error {:?}; ECU resets {} (log {:?}, SWRSTREQ issued by blocks {:08X?}); immobiliser: {} challenges 0x010 from the ECU, {} cluster answers 0x011",
        tel.engine_state,
        tel.rpm,
        s.ecu_error,
        s.board.hw().reset_count,
        s.board.hw().reset_log,
        s.board.hw().swrst_log,
        s.immo.ecu_challenges,
        s.immo.ecu_answers
    ));
    if tel.engine_state != "running" {
        events.push(format!("ECU fault memory after the failed start (UDS 19 02 FF): {:?}", read_dtcs(&mut s)));
    }
    s.plant.driver.selector = Selector::D;

    // ---- replay ---------------------------------------------------------------------------------
    let plant_ch: Vec<&Channel> = CHANNELS.iter().filter(|c| c.plant.is_some()).collect();
    let mut sim_t = Vec::new();
    let mut sim: Vec<Vec<f64>> = vec![Vec::new(); plant_ch.len()];
    let mut extras: Vec<Vec<[f64; 4]>> = vec![Vec::new(); CYL_EXTRAS.len()];
    let mut firing_frac = Vec::new();
    let mut egt = Vec::new();
    let mut tester = Tester::new();
    let (mut stalled_since, mut diverged) = (None::<f64>, None::<f64>);
    let mut resets = s.board.hw().reset_count;
    let steps = ((to - from) / 0.01).round() as usize;
    for k in 0..steps {
        let t = from + k as f64 * 0.01;
        let (n_log, ped, v) = (interp(&log.t, &l_rpm, t), interp(&log.t, &l_pedal, t), interp(&log.t, &l_v, t));
        s.plant.driver.pedal_pct = ped.clamp(0.0, 100.0);
        s.plant.driver.brake_pct = if v < 1.0 { 60.0 } else { 0.0 };
        // AC dyno speed demand = logged speed, slew-limited (DYNO_SLEW_RPM_S) so coupling to a stopped or
        // slower engine does not step the crank speed.
        let n_now = s.plant.engine.rpm();
        // 4-quadrant AC dyno in speed mode with a finite-stiffness loop on the DMF secondary (crank keeps combustion pulsations)
        s.plant.bench = BenchMode::AcDyno { target_rpm: n_log.max(0.0).clamp(n_now - DYNO_SLEW_RPM_S * 0.01, n_now + DYNO_SLEW_RPM_S * 0.01) };
        s.step(0.01);
        tester.poll(&mut s, t + 0.01);
        let tel = s.plant.telemetry();
        sim_t.push(t + 0.01);
        for (j, c) in plant_ch.iter().enumerate() {
            sim[j].push((c.plant.unwrap())(&tel));
        }
        for (j, e) in CYL_EXTRAS.iter().enumerate() {
            extras[j].push((e.2)(&tel));
        }
        firing_frac.push(tel.firing.iter().filter(|&&f| f).count() as f64 / 4.0);
        egt.push(tel.egt_c);
        // Divergence: the firmware no longer burns fuel in the motored engine (combustion stopped).
        let burning = tel.imep_bar.iter().any(|&p| p > 0.5);
        if burning || ped < 1.0 && n_log > 1200.0 {
            stalled_since = None;
        } else if t - *stalled_since.get_or_insert(t) > 1.0 && diverged.is_none() {
            diverged = Some(t);
            events.push(format!("t = {t:.2} s: no combustion for 1 s while the log shows a firing engine (pedal {ped:.0} %, {n_log:.0} rpm) — replay diverges here"));
        }
        if let Some(e) = &s.ecu_error {
            events.push(format!("t = {t:.2} s: ECU error {e}"));
            break;
        }
        let hw = s.board.hw();
        if hw.reset_count != resets {
            resets = hw.reset_count;
            events.push(format!("t = {t:.2} s: ECU reset #{resets} {:?}, SWRSTREQ blocks {:08X?}", hw.reset_log.last(), hw.swrst_log));
        }
        if k % 100 == 99 {
            eprintln!(
                "  t {:.1} s: rpm {:.0} pedal {:.0} | sim λ {:.2} MAP {:.0} PUT {:.0} IMEP {:?} spark {:.1} | log λ {:.2} MAP {:.0}",
                t + 0.01,
                tel.rpm,
                ped,
                tel.lambda_sensor,
                tel.map_kpa,
                tel.put_kpa,
                tel.imep_bar.map(|x| x.round()),
                (plant_ch.iter().position(|c| c.name == "Ign Avg").map(|j| sim[j][k])).unwrap_or(f64::NAN),
                interp(&log.t, &col("Lambda"), t),
                interp(&log.t, &col("MAP"), t)
            );
        }
    }
    let wall_s = wall.elapsed().as_secs_f64();
    let tel = s.plant.telemetry();
    events.push(format!("end: {} at {:.0} rpm; plant failures {:?}; tester refused {:?}, {} timeouts", tel.engine_state, tel.rpm, tel.failures, tester.refused, tester.timeouts));

    // ---- comparison --------------------------------------------------------------------------
    let t1 = *sim_t.last().unwrap_or(&from);
    let mut md = format!(
        "# Replay of {}\n\nimage {image}, profile {slug}, log window {from:.1}–{to:.1} s, pre-roll {preroll:.1} s, {:.0} s wall ({:?} CPU).\n\n## Events\n\n{}\n\n## Error statistics (sim − log, sampled at the log time stamps, {from:.1}–{t1:.1} s)\n\n| channel | source | n | log mean | sim mean | bias | MAE | RMSE | max |err| | r |\n|---|---|---|---|---|---|---|---|---|---|\n",
        log.source,
        wall_s,
        s.board.backend(),
        events.iter().map(|e| format!("- {e}")).collect::<Vec<_>>().join("\n")
    );
    let mut csv = String::from("channel,source,n,log_mean,sim_mean,bias,mae,rmse,max_abs,r\n");
    let mut body = String::new();
    let spans: Vec<Span> = diverged.map(|t| Span { t0: t, t1, label: "diverged" }).into_iter().collect();
    let mut row = |name: &str, src: &str, st: crate::log_tools::report::ErrStats| {
        let _ = writeln!(md, "| {name} | {src} | {} | {} | {} | {} | {} | {} | {} | {:.2} |", st.n, fmt_num(st.log_mean), fmt_num(st.sim_mean), fmt_num(st.bias), fmt_num(st.mae), fmt_num(st.rmse), fmt_num(st.max_abs), st.r);
        let _ = writeln!(csv, "{name},{src},{},{},{},{},{},{},{},{}", st.n, st.log_mean, st.sim_mean, st.bias, st.mae, st.rmse, st.max_abs, st.r);
    };
    for c in CHANNELS {
        let ly = col(c.name);
        let mut series = vec![];
        let tl: Vec<f64> = log.t.clone();
        series.push((format!("log {}", c.name), tl, ly.clone(), "#8e8e93"));
        if let Some(j) = plant_ch.iter().position(|p| p.name == c.name) {
            row(c.name, "plant", err_stats(&log.t, &ly, &sim_t, &sim[j], from, t1));
            series.push(("sim plant".into(), sim_t.clone(), sim[j].clone(), "#0a84ff"));
        }
        if let Some(j) = tester.list.iter().position(|p| p.name == c.name) {
            let (tt, yy): (Vec<f64>, Vec<f64>) = tester.data[j].iter().copied().unzip();
            row(c.name, "ECU via UDS", err_stats(&log.t, &ly, &tt, &yy, from, t1));
            series.push(("sim ECU (UDS)".into(), tt, yy, "#ff375f"));
        }
        if series.len() > 1 {
            let ss: Vec<Series> = series.iter().map(|(lb, t, y, cl)| Series { label: lb, t, y, color: cl }).collect();
            body += &svg_plot(c.name, c.unit, &ss, &spans, (from, t1));
        }
    }
    let colors = ["#0a84ff", "#30d158", "#ff9f0a", "#ff375f"];
    for (j, (name, unit, _)) in CYL_EXTRAS.iter().enumerate() {
        let per: Vec<Vec<f64>> = (0..4).map(|c| extras[j].iter().map(|v| v[c]).collect()).collect();
        let labels = ["cyl 1", "cyl 2", "cyl 3", "cyl 4"];
        let ss: Vec<Series> = (0..4).map(|c| Series { label: labels[c], t: &sim_t, y: &per[c], color: colors[c] }).collect();
        body += &svg_plot(&format!("sim {name}"), unit, &ss, &spans, (from, t1));
        let mx: Vec<String> = per.iter().map(|v| fmt_num(v.iter().copied().filter(|x| x.is_finite()).fold(f64::MIN, f64::max))).collect();
        let _ = writeln!(md, "\nmax sim {name} per cylinder 1..4: {}", mx.join(" / "));
    }
    body += &svg_plot("sim EGT / firing fraction×1000", "°C", &[Series { label: "EGT", t: &sim_t, y: &egt, color: "#ff9f0a" }, Series { label: "firing×1000", t: &sim_t, y: &firing_frac.iter().map(|x| x * 1000.0).collect::<Vec<_>>(), color: "#8e8e93" }], &spans, (from, t1));
    let image_tag = Path::new(image).file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    let dir = out_dir(root, &format!("replay/{image_tag}"), &log_path)?;
    std::fs::write(dir.join("replay.md"), &md).map_err(|e| e.to_string())?;
    std::fs::write(dir.join("stats.csv"), &csv).map_err(|e| e.to_string())?;
    std::fs::write(dir.join("replay.html"), html_page("Log replay", &format!("<pre>{}</pre>{body}", md.replace('<', "&lt;")))).map_err(|e| e.to_string())?;
    let mut ts = String::from("t");
    plant_ch.iter().for_each(|c| ts += &format!(",{}", c.name));
    ts.push('\n');
    for (i, t) in sim_t.iter().enumerate() {
        ts += &format!("{t:.2}");
        sim.iter().for_each(|c| ts += &format!(",{:.4}", c[i]));
        ts.push('\n');
    }
    std::fs::write(dir.join("sim.csv"), ts).map_err(|e| e.to_string())?;
    println!("{md}\nwritten: {}/replay.{{md,html}}, stats.csv, sim.csv", dir.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_profile_is_cjsa_is38_dq250_awd() {
        let (e, d) = profile(DEFAULT_PROFILE).unwrap();
        assert_eq!(e.engine_code, "CJSA");
        assert_eq!(e.turbo.model, "IHI IS38");
        assert!(d.awd && d.haldex.is_some() && d.gears == 6);
        assert!(profile("nope").is_err());
    }
}
