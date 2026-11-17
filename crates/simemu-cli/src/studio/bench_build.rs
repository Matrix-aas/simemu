//! Bench build in the studio: the physical car (`simemu_physics::bench_config`) is chosen independently of the
//! firmware image. A changed build is staged and re-creates the plant at the next bench start (button, ECU power
//! cycle or cold soak); the session log records every bench start with the exact build. `/api/build` serves the
//! catalogue, presets, the user's saved builds (JSON in [`builds_dir`]), the expected physical effects and the
//! firmware ↔ build compatibility notes (never blocking).
//!
//! Scenario tests and CLI commands take the build from the environment ([`build_from_env`]):
//! `SIMEMU_BUILD=<preset id | path.json>`, then `SIMEMU_ENGINE=cjsa|cjxc|chhb` and
//! `SIMEMU_TURBO=is12|is20|is38` override the long block / turbocharger.

use crate::studio::session::StudioSession;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use simemu_damos::DamosDatabase;
use simemu_physics::bench_config::{self as bc, BenchConfig, CalFacts};
use simemu_physics::{BenchMode, VirtualPlant};
use std::path::{Path, PathBuf};

pub struct BuildState {
    /// Build the plant was created from.
    pub active: BenchConfig,
    /// Edited build waiting for the next bench start.
    pub staged: Option<BenchConfig>,
    /// One line per bench start: sim time and the exact build (JSON).
    pub log: Vec<String>,
    /// The plant was changed through the legacy engine/drivetrain selectors since the last bench start.
    pub legacy: Option<String>,
    /// Calibration facts per image file of the image list (read once).
    facts_cache: std::collections::HashMap<String, CalFacts>,
}

/// Small status embedded in `/api/state`.
#[derive(Serialize)]
pub struct BuildBrief<'a> {
    pub name: &'a str,
    pub staged: bool,
    pub legacy: Option<&'a str>,
}

impl BuildState {
    /// The environment's build, or the baseline preset.
    pub fn from_env() -> Result<Self, String> {
        Ok(Self::from_config(build_from_env()?.unwrap_or_default()))
    }

    pub fn from_config(active: BenchConfig) -> Self {
        Self { active, staged: None, log: Vec::new(), legacy: None, facts_cache: Default::default() }
    }

    pub fn brief(&self) -> BuildBrief<'_> {
        BuildBrief { name: &self.active.name, staged: self.staged.is_some(), legacy: self.legacy.as_deref() }
    }

    /// Plant of the active build (config resolved once here — nothing of it runs per step).
    pub fn new_plant(&self) -> Result<VirtualPlant, String> {
        self.new_plant_with_env(None)
    }

    pub fn new_plant_with_env(&self, env: Option<(simemu_physics::Ambient, f64, f64)>) -> Result<VirtualPlant, String> {
        let mut r = self.active.resolve()?;
        if let Some((amb, ron, eth)) = env {
            r.ambient = amb;
            r.fuel_ron = ron;
            r.pending.fuel.ethanol_mass_frac = eth;
        }
        let mut p = r.into_plant();
        if !p.drivetrain_profile.has_tcu_firmware {
            p.bench = BenchMode::EngineDyno { target_rpm: 2000.0 };
        }
        Ok(p)
    }
}

/// Gearbox without TCU firmware: the board is absent and the engine runs on the absorber.
pub fn no_tcu_msg(p: &VirtualPlant) -> String {
    format!("{} firmware not available — gearbox not emulated (bench build)", p.drivetrain_profile.tcu_code)
}

fn parse_enum<T: DeserializeOwned>(what: &str, s: &str) -> Result<T, String> {
    serde_json::from_value(Value::String(s.trim().to_lowercase())).map_err(|_| format!("{what}: unknown value {s:?}"))
}

/// A preset id or a JSON file (missing fields from the baseline).
pub fn load_build(arg: &str) -> Result<BenchConfig, String> {
    if let Some(b) = bc::preset(arg) {
        return Ok(b);
    }
    let text = std::fs::read_to_string(arg).map_err(|e| format!("build {arg}: not a preset ({}) and not readable: {e}", preset_ids()))?;
    let b: BenchConfig = serde_json::from_str(&text).map_err(|e| format!("build {arg}: {e}"))?;
    b.validate()?;
    Ok(b)
}

fn preset_ids() -> String {
    bc::PRESETS.iter().map(|p| p.0).collect::<Vec<_>>().join(", ")
}

/// `SIMEMU_BUILD`, `SIMEMU_ENGINE`, `SIMEMU_TURBO`; `None` when none is set.
pub fn build_from_env() -> Result<Option<BenchConfig>, String> {
    build_from_vars(|k| std::env::var(k).ok())
}

fn build_from_vars(get: impl Fn(&str) -> Option<String>) -> Result<Option<BenchConfig>, String> {
    let var = |k: &str| get(k).filter(|v| !v.trim().is_empty());
    let (b, e, t) = (var("SIMEMU_BUILD"), var("SIMEMU_ENGINE"), var("SIMEMU_TURBO"));
    if b.is_none() && e.is_none() && t.is_none() {
        return Ok(None);
    }
    let mut cfg = match &b {
        Some(arg) => load_build(arg)?,
        None => BenchConfig::default(),
    };
    if let Some(e) = &e {
        cfg.engine = parse_enum("SIMEMU_ENGINE", e)?;
    }
    if let Some(t) = &t {
        cfg.turbo = parse_enum("SIMEMU_TURBO", t)?;
    }
    if e.is_some() || t.is_some() {
        cfg.name = format!("{} [env: engine {} turbo {}]", cfg.name, e.as_deref().unwrap_or("-"), t.as_deref().unwrap_or("-"));
    }
    cfg.validate()?;
    Ok(Some(cfg))
}

/// User directory of saved builds: `SIMEMU_BUILDS_DIR`, else `~/.simemu/builds`.
pub fn builds_dir() -> PathBuf {
    if let Some(d) = std::env::var_os("SIMEMU_BUILDS_DIR") {
        return PathBuf::from(d);
    }
    let home = std::env::var_os("HOME").or_else(|| std::env::var_os("USERPROFILE")).unwrap_or_else(|| ".".into());
    PathBuf::from(home).join(".simemu").join("builds")
}

/// File name of a saved build: letters, digits, space, `-`, `_`, `.`; no path, no leading dot.
fn build_file(dir: &Path, name: &str) -> Result<PathBuf, String> {
    let n = name.trim();
    let ok = !n.is_empty() && n.chars().count() <= 64 && !n.starts_with('.') && n.chars().all(|c| c.is_alphanumeric() || " -_.()+".contains(c));
    if !ok {
        return Err(format!("build name {name:?}: 1–64 letters, digits, space, - _ . ( ) +; not starting with '.'"));
    }
    Ok(dir.join(format!("{n}.json")))
}

fn saved_builds(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = std::fs::read_dir(dir)
        .map(|rd| rd.flatten().filter_map(|e| e.file_name().to_str()?.strip_suffix(".json").map(str::to_string)).collect())
        .unwrap_or_default();
    v.sort();
    v
}

// ---- what the calibration was made for ----------------------------------------------------------------------

/// Part number and engine text of the CAL header (`8V0906259J`, `2.0l R4 TFSI`).
fn cal_header(cal: &[u8]) -> (String, String) {
    let head = cal.get(..0xB0).unwrap_or(cal);
    let part = head
        .split(|b| !b.is_ascii_alphanumeric())
        .filter_map(|t| std::str::from_utf8(t).ok())
        .find(|t| (10..=11).contains(&t.len()) && t.get(3..6) == Some("906"))
        .unwrap_or_default()
        .to_string();
    let label = cal.get(0x6C..0x80).map(|b| String::from_utf8_lossy(b.split(|&c| c == 0).next().unwrap_or(&[])).trim().to_string()).unwrap_or_default();
    (part, label)
}

/// DI injector static flow at 10 MPa from the calibration's injector characteristic `ip_ti_4_l` (electrical time vs
/// fuel mass and ΔP): least squares of ti = m / (q·√(ΔP/10 MPa)) + t0 over 10–50 mg and 4–15 MPa — the fit the
/// plant's injector values were taken with (`EngineProfile::ea888_18t_cjsa_is12/_is38` docs).
fn injector_fit(flash: &[u8], damos: &DamosDatabase) -> Option<f64> {
    let p = damos.get_by_id("ip_ti_4_l")?;
    let m = damos.read_matrix(p, flash, flash)?;
    let (mut sxx, mut sx, mut n, mut sxy, mut sy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (r, dp) in m.y_axis.iter().enumerate() {
        for (c, mg) in m.x_axis.iter().enumerate() {
            if (10.0..=50.0).contains(mg) && (40_000.0..=150_000.0).contains(dp) {
                let (x, y) = (mg / (dp / 100_000.0).sqrt(), m.values.get(r)?.get(c)?);
                (sxx, sx, n, sxy, sy) = (sxx + x * x, sx + x, n + 1.0, sxy + x * y, sy + y);
            }
        }
    }
    let det = sxx * n - sx * sx;
    if n < 4.0 || det.abs() < 1e-12 {
        return None;
    }
    let slope = (sxy * n - sx * sy) / det; // ms per mg at 10 MPa
    (slope > 0.0).then(|| 1.0 / slope / (simemu_physics::engine::FUEL_DENSITY_KG_M3 / 1000.0))
}

/// Calibration facts of a loaded image (header + DAMOS values).
pub fn cal_facts(flash: &[u8], damos: &DamosDatabase) -> CalFacts {
    let (part_number, engine_label) = cal_header(flash.get(0x20_0000..).unwrap_or(&[]));
    let val = |id: &str| damos.get_by_id(id).and_then(|p| damos.read_physical_value(p, flash)).filter(|v| v.is_finite() && *v > 0.0);
    CalFacts {
        part_number,
        engine_label,
        displacement_l: val("c_dpl").or_else(|| val("c_eng_vol")),
        injector_cc_s: injector_fit(flash, damos),
        n_tcha_max_sp_rpm: val("c_n_tcha_max_sp"),
    }
}

/// Facts of an image file of the list: with its DAMOS when there is one, else the CAL header only.
// ponytail: loads each image's DAMOS JSON once (≈ 0.1–0.2 s each, under the session lock on the first /api/build);
// move to a worker thread if the image list grows.
fn file_facts(root: &Path, info: &crate::studio::session::ImageInfo) -> CalFacts {
    let Ok(flash) = crate::paths::read_firmware(root, &info.file) else { return CalFacts::default() };
    match info.damos.as_ref().and_then(|d| DamosDatabase::load_from_json(&root.join(d)).ok()) {
        Some(db) => cal_facts(&flash, &db),
        None => {
            let (part_number, engine_label) = cal_header(flash.get(0x20_0000..).unwrap_or(&[]));
            CalFacts { part_number, engine_label, ..Default::default() }
        }
    }
}

// ---- bench start & API ----------------------------------------------------------------------------------------

#[derive(Deserialize)]
struct BuildRequest {
    action: String,
    #[serde(default)]
    build: Option<BenchConfig>,
    #[serde(default)]
    preset: Option<String>,
    #[serde(default)]
    name: Option<String>,
}

type RouteResult = Result<Value, (&'static str, String)>;

fn bad(e: impl Into<String>) -> (&'static str, String) {
    ("400 Bad Request", e.into())
}

/// Engine-off soak reported on a cold bench reset (12 h: engine and exhaust have reached ambient, plausible against
/// any prior hot run saved in DFlash under `ENSS 33.19` `T_ES_TCO_PLAUS`).
const COLD_SOAK_EOT_S: f64 = 12.0 * 3600.0;

impl StudioSession {
    /// Bench start: the staged build (if any) becomes the plant — re-created from scratch, bench mode, ignition state
    /// and bench supply kept; without a staged build the plant is cold-soaked (`VirtualPlant::reset`). The TCU board
    /// follows the gearbox (`reboot_tcu` = power-cycle it); the ECU keeps running unless a new stock image is auto-loaded.
    pub fn start_bench(&mut self, reboot_tcu: bool) -> Result<(), String> {
        self.start_bench_mode(false, false, reboot_tcu)
    }

    /// Unified bench start and soak/power-cycle helper:
    /// 1. Stops any active dyno sweep (restoring the prior [`BenchMode`]) and recreates/resets [`Self::plant`].
    /// 2. Resets transient driver motion inputs (`P`, pedals 0 %, start button released) while preserving `ignition_on`,
    ///    clears latched actuators, and applies either a warm soak ([`Self::warm_car`]: 90 °C engine, 70 °C gearbox,
    ///    300 s EOT) or a 12 h cold soak **before** any control unit boots.
    /// 3. Re-syncs [`MqbChassisEncoder::reset_soak`] (`Kombi_02` standing time matches `eot_clock_s`) and `ecu_link`.
    /// 4. Boots the ECU at most once (auto-restocked image or `reboot_ecu`) and then boots/updates the TCU at most once
    ///    with the final immobiliser binding and image selection.
    pub fn start_bench_mode(&mut self, warm: bool, reboot_ecu: bool, reboot_tcu: bool) -> Result<(), String> {
        if self.dyno.running {
            self.stop_dyno();
        }
        let staged = self.bench_build.staged.take();
        let mut restock = None;
        if staged.is_none() && self.bench_build.legacy.is_some() {
            // Plant set through the legacy profile selectors: keep it, cold-soaked.
            self.plant.reset();
            if !self.plant.drivetrain_profile.has_tcu_firmware
                && matches!(self.plant.bench, BenchMode::Road | BenchMode::RoadLoadDyno | BenchMode::ChassisDyno { .. })
            {
                self.plant.bench = BenchMode::EngineDyno { target_rpm: 2000.0 };
            }
        } else {
            // Fresh parts as built (worn ones included); the live ambient and fuel stay unless a new build came in.
            let keep_env = staged.is_none();
            if let Some(cfg) = staged {
                // a new engine gets its stock software (when present on disk), unless the user had chosen another image
                let old = crate::studio::session::stock_ecu_image(self.bench_build.active.engine);
                let new = crate::studio::session::stock_ecu_image(cfg.engine);
                restock = (old != new
                    && crate::studio::session::is_stock_ecu_file(&self.image_file, self.bench_build.active.engine)
                    && crate::paths::has_firmware(&self.root, new))
                .then_some(new);
                self.bench_build.active = cfg;
            }
            let p = &self.plant;
            let (driver, supply, fan, bench, ambient, ron, eth) = (
                p.driver.clone(),
                p.bench_supply_v,
                p.test_cell_fan_ms,
                p.bench,
                p.ambient,
                p.fuel_ron,
                p.fuel_ethanol,
            );
            self.plant = self.bench_build.new_plant_with_env(keep_env.then_some((ambient, ron, eth)))?;
            (self.plant.driver, self.plant.bench_supply_v, self.plant.test_cell_fan_ms) = (driver, supply, fan);
            if self.plant.drivetrain_profile.has_tcu_firmware
                || matches!(bench, BenchMode::EngineDyno { .. } | BenchMode::AcDyno { .. })
            {
                self.plant.bench = bench;
            }
            self.bench_build.legacy = None;
        }
        self.plant.driver = simemu_physics::DriverInputs {
            ignition_on: self.plant.driver.ignition_on,
            ..Default::default()
        };
        self.actuators = Default::default();
        self.warm_standby = false;
        if warm {
            self.warm_car();
        } else {
            self.eot_clock_s = COLD_SOAK_EOT_S;
            self.sync_board_eot();
            self.reset_chassis_soak();
        }
        // The new plant's clock starts at 0: re-attach the ECU sensor wiring to it (the link re-syncs board ↔ plant
        // time at the next exchange, as after an ECU power cycle). Without this a kept running ECU waits until the new
        // plant clock catches up with the old one and looks frozen.
        self.ecu_link = Default::default();
        self.ecu_link.t_off = Some(self.board.board_time_s() - self.plant.sim_time_s);

        let has_tcu_fw = self.plant.drivetrain_profile.has_tcu_firmware;
        let need_tcu_reboot = has_tcu_fw
            && (reboot_tcu
                || !self.tcu.as_ref().is_some_and(|t| t.fits(&self.plant.drivetrain_profile))
                || self.tcu_image_changed());
        if !has_tcu_fw {
            self.detach_tcu();
            self.tcu_error = Some(no_tcu_msg(&self.plant));
            self.cluster_tcu = None;
            self.tcu_booted();
        } else if need_tcu_reboot {
            // Reclaim and save the outgoing TCU's NVM before `load_image` so `bind_immo` updates `tcu_lab` without
            // booting the TCU prematurely on the old ECU image name.
            self.detach_tcu();
        }
        let ecu_res = if let Some(img) = restock {
            self.load_image(img)
        } else if reboot_ecu {
            self.power_cycle_ecu()
        } else {
            Ok(())
        };
        if need_tcu_reboot {
            self.power_cycle_tcu_fresh();
        } else if has_tcu_fw {
            if let Some(n) = self.fit_gear_set() {
                self.tcu_notes.push(n); // the new plant on the running DQ250's own gear set
            }
        }
        ecu_res?;
        let b = &self.bench_build.active;
        let line = format!("t={:.2}s bench start: {} | engine {} | drivetrain {} | build {}", self.plant.sim_time_s, b.name, self.plant.engine_profile.name, self.plant.drivetrain_profile.name, serde_json::to_string(b).unwrap_or_default());
        eprintln!("{line}");
        let label = format!("bench start: {}", b.name);
        let _ = self.ws.recorder.request(&crate::studio::recorder::RecRequest { action: "mark".into(), channels: Vec::new(), label }, self.plant.sim_time_s);
        self.bench_build.log.push(line);
        Ok(())
    }

    fn build_payload(&mut self) -> Value {
        for i in &self.images {
            if !self.bench_build.facts_cache.contains_key(&i.file) {
                let f = file_facts(&self.root, i);
                self.bench_build.facts_cache.insert(i.file.clone(), f);
            }
        }
        let st = &self.bench_build;
        let edit = st.staged.as_ref().unwrap_or(&st.active);
        let facts = cal_facts(&self.current_flash, &self.damos);
        let images: Vec<Value> = self
            .images
            .iter()
            .map(|i| {
                let f = &st.facts_cache[&i.file];
                json!({ "file": i.file, "part": f.part_number, "label": f.engine_label, "notes": bc::compatibility(&st.active, f) })
            })
            .collect();
        let turbos: Vec<Value> = bc::TURBOS
            .iter()
            .map(|t| {
                let (flow, pr) = bc::turbo_envelope(t.id);
                json!({ "spec": t, "max_flow_kg_s": flow, "max_pr": pr, "hw": bc::turbo_hardware(t.id) })
            })
            .collect();
        let resolved = edit.resolve().ok();
        json!({
            "active": st.active,
            "staged": st.staged,
            "legacy": st.legacy,
            "edit_engine": resolved.as_ref().map(|r| r.engine.name),
            "edit_drivetrain": resolved.as_ref().map(|r| r.drivetrain.name),
            "pending": resolved.as_ref().map(|r| r.pending.clone()),
            "effects": edit.effects().unwrap_or_default(),
            "presets": bc::PRESETS.iter().map(|p| json!({ "id": p.0, "ru": p.1, "en": p.2 })).collect::<Vec<_>>(),
            "saved": saved_builds(&builds_dir()),
            "builds_dir": builds_dir(),
            "catalog": {
                "engines": bc::ENGINES, "turbos": turbos, "plugs": bc::PLUGS, "vehicles": bc::VEHICLES, "wear": bc::WEAR,
                "throttles": { "gen3": simemu_physics::profiles::THROTTLE_CDA_EA888_GEN3.last(), "cjha": bc::THROTTLE_CDA_CJHA.last() },
                "stock": { "intake_mm": bc::STOCK_INTAKE_MM, "ic_core_mm": bc::STOCK_IC_CORE_MM, "ic_pipe_mm": bc::STOCK_IC_PIPE_MM, "exhaust_mm": bc::STOCK_EXHAUST_MM },
            },
            "image": { "file": self.image_file, "facts": facts, "notes": bc::compatibility(edit, &facts) },
            "images": images,
            // ponytail: reads every DQ250 file (1.5 MB) per call for its identification; cache by mtime if the list grows
            "tcu_images": crate::studio::dq250_prep::dq250_images(&self.root),
            "log": st.log,
        })
    }

    /// `/api/build`.
    pub fn build_route(&mut self, method: &str, body: &[u8]) -> RouteResult {
        if method == "POST" {
            let r: BuildRequest = serde_json::from_slice(body).map_err(|e| bad(format!("bad JSON: {e}")))?;
            let dir = builds_dir();
            let validate_tcu_img = |b: &mut BenchConfig| -> Result<(), (&'static str, String)> {
                if b.tcu_image.as_deref().is_some_and(|s| s.trim().is_empty()) {
                    b.tcu_image = None;
                }
                if let Some(img) = b.tcu_image.as_deref() {
                    crate::paths::resolve_sandboxed_firmware(&self.root, img).map_err(bad)?;
                }
                Ok(())
            };
            match r.action.as_str() {
                "stage" => {
                    let mut b = r.build.ok_or_else(|| bad("missing build"))?;
                    validate_tcu_img(&mut b)?;
                    b.resolve().map_err(bad)?;
                    self.bench_build.staged = Some(b);
                }
                "preset" => {
                    let id = r.preset.unwrap_or_default();
                    self.bench_build.staged = Some(bc::preset(&id).ok_or_else(|| bad(format!("unknown preset {id} ({})", preset_ids())))?);
                }
                "discard" => self.bench_build.staged = None,
                "save" => {
                    let name = r.name.unwrap_or_default();
                    let path = build_file(&dir, &name).map_err(bad)?;
                    let st = &self.bench_build;
                    let mut b = r.build.or_else(|| st.staged.clone()).unwrap_or_else(|| st.active.clone());
                    b.name = name.trim().to_string();
                    validate_tcu_img(&mut b)?;
                    b.validate().map_err(bad)?;
                    std::fs::create_dir_all(&dir).map_err(|e| ("500 Internal Server Error", format!("{}: {e}", dir.display())))?;
                    let text = serde_json::to_string_pretty(&b).map_err(|e| ("500 Internal Server Error", e.to_string()))?;
                    std::fs::write(&path, text).map_err(|e| ("500 Internal Server Error", format!("{}: {e}", path.display())))?;
                    self.bench_build.staged = Some(b);
                }
                "load" => {
                    let path = build_file(&dir, r.name.as_deref().unwrap_or_default()).map_err(bad)?;
                    let mut b = load_build(&path.to_string_lossy()).map_err(bad)?;
                    validate_tcu_img(&mut b)?;
                    self.bench_build.staged = Some(b);
                }
                "delete" => {
                    let path = build_file(&dir, r.name.as_deref().unwrap_or_default()).map_err(bad)?;
                    std::fs::remove_file(&path).map_err(|e| bad(format!("{}: {e}", path.display())))?;
                }
                "start" => {
                    self.bench_build.legacy = None;
                    self.start_bench_mode(false, true, true).map_err(bad)?;
                }
                a => return Err(bad(format!("unknown build action {a}"))),
            }
        }
        Ok(self.build_payload())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_and_names() {
        let mut cal = vec![0u8; 0xB0];
        let h = b"CASC8S50SC8S500\x0010360684AA------111SC800S5000000111SC800S5000000111SC800S5000000CASC8S50.DAT    8V0906259J \x002.0l R4 TFSI \x00";
        cal[..h.len()].copy_from_slice(h);
        assert_eq!(cal_header(&cal), ("8V0906259J".to_string(), "2.0l R4 TFSI".to_string()));
        let d = Path::new("/tmp");
        assert!(build_file(d, "Audi A3 8V 1.8T (stock + FMIC)").is_ok());
        for n in ["", "../x", "a/b", ".hidden", "a\\b"] {
            assert!(build_file(d, n).is_err(), "{n}");
        }
        assert_eq!(parse_enum::<bc::TurboId>("t", "IS38").unwrap(), bc::TurboId::Is38);
        assert!(load_build("a3_8v_quattro").is_ok() && load_build("/nonexistent.json").is_err());
        let vars = |b: &'static str, e: &'static str, t: &'static str| {
            move |k: &str| match k {
                "SIMEMU_BUILD" => Some(b.to_string()),
                "SIMEMU_ENGINE" => Some(e.to_string()),
                "SIMEMU_TURBO" => Some(t.to_string()),
                _ => None,
            }
        };
        assert!(build_from_vars(vars("", "", "")).unwrap().is_none());
        let c = build_from_vars(vars("a3_8v_quattro", "cjxc", "is38")).unwrap().unwrap();
        assert_eq!((c.engine, c.turbo, c.gearbox), (bc::LongBlock::Cjxc, bc::TurboId::Is38, bc::Gearbox::Dq250_0d9));
        let c = build_from_vars(vars("", "", "is20")).unwrap().unwrap();
        assert_eq!((c.engine, c.turbo), (bc::LongBlock::Cjsa, bc::TurboId::Is20));
        assert!(build_from_vars(vars("", "v8", "")).is_err() && build_from_vars(vars("nope", "", "")).is_err());
    }

    /// Bench start with images present: the staged build becomes the plant, the gearbox decides the TCU board, the
    /// session log records the build, and the loaded image is untouched (firmware independence).
    #[test]
    fn bench_start_recreates_the_plant_and_keeps_the_image() {
        let Some(root) = crate::paths::test_workspace_root() else {
            eprintln!("SKIP: images not found");
            return;
        };
        if build_from_env().ok().flatten().is_some() {
            eprintln!("SKIP: SIMEMU_BUILD/ENGINE/TURBO set");
            return;
        }
        let mut s = StudioSession::new(&root).expect("session");
        assert_eq!(s.plant.engine_profile.injector_flow_cc_s, simemu_physics::EngineProfile::ea888_18t_cjsa_is12().injector_flow_cc_s);
        assert!(s.tcu.is_some());
        assert_eq!(s.image_file, crate::paths::DEFAULT_ECU_FLASH);
        s.build_route("POST", br#"{"action":"preset","preset":"golf7r"}"#).expect("stage");
        assert!(s.plant.engine_profile.turbo.model.contains("IS12"), "staged, not applied yet");
        s.build_route("POST", br#"{"action":"start"}"#).expect("start");
        assert!(s.plant.engine_profile.turbo.model.contains("IS38"));
        // DQ250 0D9 gearbox: its TCU board (TC1766) replaces the DQ200's, the bench mode is kept.
        assert!(s.tcu.as_ref().is_some_and(|t| t.dq250().is_some()) && !matches!(s.plant.bench, BenchMode::EngineDyno { .. }), "{:?}", s.tcu_error);
        // Switching from stock CJSA to CJXC auto-restocks to CJXC's stock FRF when the user had not overridden the image.
        assert_eq!(s.image_file, crate::paths::STOCK_264K_FLASH);
        assert!(s.bench_build.log.last().is_some_and(|l| l.contains("\"engine\":\"cjxc\"") && l.contains("is38")));
        // Loading another image (1.8 SC8O40 onto 2.0 Golf 7 R) does not change the build, emits an air-model warning,
        // and a subsequent bench start preserves that user-selected image.
        let name = s.bench_build.active.name.clone();
        s.apply_preset(&crate::studio::session::PresetRequest {
            action: "load_image".into(),
            engine_slug: None,
            drivetrain_slug: None,
            fuel_ron: None,
            ambient_temp_c: None,
            altitude_m: None,
            humidity_pct: None,
            image: Some(crate::paths::DEFAULT_ECU_FLASH.into()),
        })
        .expect("image");
        assert_eq!(s.bench_build.active.name.as_str(), name.as_str());
        let v = s.build_route("GET", &[]).expect("get");
        let notes = v["image"]["notes"].as_array().unwrap();
        assert!(notes.iter().any(|n| n["level"] == "warn" && n["en"].as_str().unwrap().contains("air model")), "{notes:?}");
        // Back to a DQ200 build at a power cycle: the TCU board returns and the user-chosen image is kept.
        s.build_route("POST", br#"{"action":"preset","preset":"octavia_fl"}"#).expect("stage");
        s.apply_preset(&crate::studio::session::PresetRequest {
            action: "power_cycle".into(),
            engine_slug: None,
            drivetrain_slug: None,
            fuel_ron: None,
            ambient_temp_c: None,
            altitude_m: None,
            humidity_pct: None,
            image: None,
        })
        .expect("power cycle");
        assert_eq!(s.image_file, crate::paths::DEFAULT_ECU_FLASH);
        assert!(s.tcu.is_some() && s.plant.drivetrain_profile.has_tcu_firmware && s.bench_build.log.len() == 2);
        // A bench start (here: cold soak) restarts the plant clock at 0 while the ECU keeps running: the ECU must keep
        // executing right away, not wait until the new plant clock reaches the old one.
        s.plant.driver.ignition_on = true;
        s.step(1.0);
        s.apply_preset(&crate::studio::session::PresetRequest {
            action: "cold_soak".into(),
            engine_slug: None,
            drivetrain_slug: None,
            fuel_ron: None,
            ambient_temp_c: None,
            altitude_m: None,
            humidity_pct: None,
            image: None,
        })
        .expect("cold soak");
        let t0 = s.board.board_time_s();
        s.step(0.2);
        assert!(s.board.board_time_s() - t0 > 0.15, "ECU stalled after the bench start: {} → {}", t0, s.board.board_time_s());
        // Path traversal / outside-workspace `tcu_image` values must be rejected on `/api/build`.
        let err = s
            .build_route(
                "POST",
                br#"{"action":"stage","build":{"name":"bad","tcu_image":"../../etc/passwd.bin"}}"#,
            )
            .unwrap_err();
        assert_eq!(err.0, "400 Bad Request");
        assert!(err.1.contains("invalid firmware image name"));
    }

    fn ecu_coolant_c(s: &mut StudioSession) -> Option<f64> {
        let req = crate::studio::session::UdsRequest {
            target: "ecu".into(),
            service: "raw".into(),
            data: vec![0x22, 0xF4, 0x05],
            value: None,
            rid: None,
        };
        s.uds_request(&req).ok()?;
        for _ in 0..80 {
            s.step(0.01);
            if !s.uds_ecu.status.pending {
                break;
            }
        }
        let r: Vec<u8> = s.uds_ecu.status.response_hex.split(' ').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect();
        (r.get(..3) == Some(&[0x62, 0xF4, 0x05][..])).then(|| r.get(3).map(|&a| a as f64 - 40.0)).flatten()
    }

    struct TempDirGuard(PathBuf);
    impl Drop for TempDirGuard {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn kombi_02_standing_s(s: &StudioSession) -> f64 {
        let mut probe = simemu_bridge::MqbChassisEncoder::default();
        probe.reset_soak();
        let ci = simemu_bridge::ChassisInputs {
            terminal_15: s.plant.driver.ignition_on,
            soak_s: s.eot_clock_s,
            ..Default::default()
        };
        let slice = probe.encode(&ci);
        let d = slice.iter().find(|f| f.0 == 0x6B7).expect("Kombi_02 on first slice").2;
        ((u64::from_le_bytes(d) >> 20) & 0x1FFFF) as f64
    }

    /// Studio warm startup, `power_cycle` and `warm_soak` always boot ECU and TCU *after* preparing the plant/soak state,
    /// clear transient driver inputs (`P`, pedals 0 %), keep CAN `Kombi_02` standing time synchronized with `eot_clock_s`,
    /// and never write a virgin/unprepared DQ250 EEPROM (`written == 0`) into the state directory on startup.
    #[test]
    fn bench_resets_and_warm_startup_sync_plant_eot_can_and_ecu_boot() {
        let Some(root) = crate::paths::test_workspace_root() else {
            eprintln!("SKIP: images not found");
            return;
        };
        let nvm_guard = TempDirGuard(std::env::temp_dir().join(format!("simemu_warm_boot_test_{}", std::process::id())));
        let _ = std::fs::remove_dir_all(&nvm_guard.0);

        let mut s = StudioSession::with_build(&root, BuildState::from_config(BenchConfig::default())).expect("session");
        s.warm_car();
        s.attach_nvm_dir(nvm_guard.0.clone(), true).expect("attach_nvm_dir");
        s.plant.driver.ignition_on = true;
        assert_eq!(s.eot_clock_s, 300.0);
        assert!((kombi_02_standing_s(&s) - 300.0).abs() <= 2.0);
        let tco_warm = ecu_coolant_c(&mut s).expect("DID 0xF405 after warm startup");
        assert!((88.0..=92.0).contains(&tco_warm), "ECU must boot on 90 °C warm plant, got {tco_warm} °C");

        // Dirty motion controls before power_cycle must reset to P / 0 %, while KL15 stays on and ECU boots on cold ambient.
        s.plant.driver.selector = simemu_physics::Selector::D;
        s.plant.driver.pedal_pct = 45.0;
        s.plant.driver.brake_pct = 30.0;
        s.plant.driver.start_button = true;
        s.apply_preset(&crate::studio::session::PresetRequest {
            action: "power_cycle".into(),
            engine_slug: None,
            drivetrain_slug: None,
            fuel_ron: None,
            ambient_temp_c: None,
            altitude_m: None,
            humidity_pct: None,
            image: None,
        })
        .expect("power_cycle");
        assert!(s.plant.driver.ignition_on);
        assert_eq!(s.plant.driver.selector, simemu_physics::Selector::P);
        assert_eq!(s.plant.driver.pedal_pct, 0.0);
        assert_eq!(s.plant.driver.brake_pct, 0.0);
        assert!(!s.plant.driver.start_button);
        assert!(s.actuators.ecu.main_relay.is_none());
        assert_eq!(s.eot_clock_s, COLD_SOAK_EOT_S);
        assert!((kombi_02_standing_s(&s) - COLD_SOAK_EOT_S).abs() <= 2.0);
        let tco_cold = ecu_coolant_c(&mut s).expect("DID 0xF405 after power_cycle");
        assert!(
            (s.plant.ambient.temp_c - 2.0..=s.plant.ambient.temp_c + 2.0).contains(&tco_cold),
            "ECU must boot on cold ambient ({} °C) after power_cycle, got {tco_cold} °C",
            s.plant.ambient.temp_c
        );

        // Warm soak restores 90 °C / 70 °C / 300 s and boots ECU on the warm plant.
        s.apply_preset(&crate::studio::session::PresetRequest {
            action: "warm_soak".into(),
            engine_slug: None,
            drivetrain_slug: None,
            fuel_ron: None,
            ambient_temp_c: None,
            altitude_m: None,
            humidity_pct: None,
            image: None,
        })
        .expect("warm_soak");
        assert_eq!(s.eot_clock_s, 300.0);
        assert!((s.plant.driveline.mech.oil_c - 70.0).abs() < 1e-6);
        assert!((kombi_02_standing_s(&s) - 300.0).abs() <= 2.0);
        let tco_rewarm = ecu_coolant_c(&mut s).expect("DID 0xF405 after warm_soak");
        assert!((88.0..=92.0).contains(&tco_rewarm), "ECU must boot on 90 °C after warm_soak, got {tco_rewarm} °C");

        // A failed `load_image` preset must leave the running TCU attached rather than orphaning `s.tcu == None`.
        assert!(
            s.apply_preset(&crate::studio::session::PresetRequest {
                action: "load_image".into(),
                engine_slug: None,
                drivetrain_slug: None,
                fuel_ron: None,
                ambient_temp_c: None,
                altitude_m: None,
                humidity_pct: None,
                image: Some("nonexistent_image_404.bin".into()),
            })
            .is_err()
        );
        assert!(s.tcu.is_some(), "failed load_image must not detach running TCU");

        // Regression check for DQ250 Studio startup (`attach_nvm_dir(..., true)` with `written == 0`): must NOT persist
        // a virgin 8 KB EEPROM file to `nvm_dir` before the firmware itself writes EEPROM bytes.
        if StudioSession::dq250_image_path(&root, None).is_ok() {
            let dq_dir = TempDirGuard(std::env::temp_dir().join(format!("simemu_dq250_virgin_test_{}", std::process::id())));
            let _ = std::fs::remove_dir_all(&dq_dir.0);
            let cfg = bc::preset("a3_8v_quattro").expect("a3_8v_quattro");
            let mut s_dq = StudioSession::with_build(&root, BuildState::from_config(cfg)).expect("dq250 session");
            s_dq.warm_car();
            s_dq.attach_nvm_dir(dq_dir.0.clone(), true).expect("attach_nvm_dir dq250");
            let id = s_dq.tcu.as_ref().and_then(|t| t.nvm()).expect("dq250 nvm").0.to_string();
            assert!(
                !s_dq.tcu_nvm.file(&id).unwrap().exists(),
                "virgin DQ250 EEPROM (`written == 0`) must not be written to disk on startup"
            );
        }
    }
}
