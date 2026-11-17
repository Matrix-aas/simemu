//! Co-simulation session: SIMOS18.1 board + DQ200_G2 board + MQB CAN + physics plant.
//!
//! Every 10 ms slice: chassis frames → boards, then ten 1 ms lock-step windows in which the plant,
//! the ECU (this thread) and the TCU (worker thread) advance in parallel against the inputs latched at
//! the window start, with CAN bridged (instrument-cluster immobiliser answers included) and firmware
//! outputs decoded into plant actuators at every window end ([`StudioSession::step`]).

use crate::paths;
use crate::slcan::SlcanServer;
use crate::studio::immo_lab::{self, LabKeys};
use crate::studio::io;
use crate::studio::rt;
use crate::studio::uds::{UdsClient, UdsStatus, OBD_FUNCTIONAL_ID};
use serde::{Deserialize, Serialize};
use simemu_bridge::{ChassisInputs, MqbChassisEncoder};
use simemu_damos::{fix_checksums, verify_checksums, ChecksumReport, CuratedTunerMap, DamosDatabase, CURATED_TUNER_MAPS};
use simemu_physics::{
    Ambient, BenchMode, DriverInputs, DrivetrainProfile, EngineProfile, PlantActuators, PlantTelemetry, Selector,
    TcuOutputs, VirtualPlant,
};
use crate::studio::tcu_io::TcuBoard;
use simemu_tc1791::{CanFrame, Dq200TcuBoard, Dq250TcuBoard, Simos18Board, Wfs5ImmoPartner};
use std::collections::{HashSet, VecDeque};
use std::path::{Path, PathBuf};
use std::time::Instant;
use unicorn_tricore_sys::TriCoreReg;

pub const SLICE_S: f64 = 0.010;
/// Lock-step co-simulation window (ECU, TCU, plant, CAN) inside a slice. Ordering per window: the plant
/// advances first with the actuators decoded at the end of the previous window and records its
/// crank/cam edges with exact timestamps; the ECU board then runs over the same interval with those
/// edges, so sensor timing is exact at any speed; the TCU runs the same interval in parallel on the
/// inputs latched at the window start. The only coupling error is the actuator delay of one window
/// (1 ms = 36 °CRK at 6000 rpm), explicit in [`io::ACTUATOR_DELAY_S`]: the injectors and N276 reach the plant
/// as their edges shifted by it, the coils fire at their own discharge time when the dwell length repeats.
pub const EXCHANGE_S: f64 = 0.001;
const CAN_LOG_LEN: usize = 64;
const ECU_BOOT_INSNS: usize = 16_000_000;
/// Default factory stock ECU image (`8V0906264H__0005.bin`, SC8O40, unpacked from VAG ODIS `.frf`).
pub const S50_FLASH: &str = paths::DEFAULT_ECU_FLASH;
pub const S50_EEPROM: &str = paths::DEFAULT_ECU_EEPROM;
pub const S50_DAMOS: &str = "definitions/SC8O40.json";
const DQ200_FLASH: &str = paths::DEFAULT_DQ200_FLASH;
const DQ200_EEPROM: &str = paths::DEFAULT_DQ200_EEPROM;
/// OEM DQ250 (`0D9`) software identifier paired with 1.8 TSI (`0D9300012F`, transmits `0x0B1` `Getriebe_15`).
#[cfg(test)]
pub(crate) const DQ250_012F: &str = paths::DQ250_012F_ID;

/// Stock ECU software the studio loads for a bench engine: FRF-built factory images only (AGENTS.md §3.3).
/// CJSA/CJSB → SC8O40 264H. No FRF exists for CJX/CHHB: the 2.0 SC8S50 264K (CJHA/CJJG) is the nearest stock
/// software; the build panel shows the mismatch.
pub fn stock_ecu_image(engine: simemu_physics::bench_config::LongBlock) -> &'static str {
    match engine {
        simemu_physics::bench_config::LongBlock::Cjsa => "8V0906264H__0005.bin",
        _ => "8V0906264K__0003.bin",
    }
}

/// True when `file` is the stock ECU image for `engine` in either `.bin` (`8V0906264H__0005.bin`) or native `.frf`
/// (`FL_8V0906264H__0005.frf`) form.
pub fn is_stock_ecu_file(file: &str, engine: simemu_physics::bench_config::LongBlock) -> bool {
    let norm = |s: &str| {
        let name = Path::new(s).file_name().and_then(|n| n.to_str()).unwrap_or(s);
        let stem = name.rsplit_once('.').map_or(name, |(a, _)| a);
        stem.strip_prefix("FL_").or_else(|| stem.strip_prefix("fl_")).unwrap_or(stem).to_ascii_uppercase()
    };
    norm(file) == norm(stock_ecu_image(engine))
}
const O30_SWITCH_DEF: &str = "definitions/O30_Switch_Patch.29.33.V2.json";

/// One lock-step window for the TCU worker: sensor inputs latched at the window start, CAN frames to
/// receive first, and the board time to run to.
struct TcuWindow {
    tel: PlantTelemetry,
    driver: simemu_physics::DriverInputs,
    /// Valve coil R per channel and L ([`crate::studio::tcu_io::coil_params`]).
    coils: Option<(Vec<f64>, f64)>,
    rx: Vec<CanFrame>,
    until_s: f64,
}

/// TCU worker result of a window.
struct TcuDone {
    tx: Vec<CanFrame>,
    out: TcuOutputs,
    err: Option<String>,
    /// Wall time running the window / waiting for it.
    busy_s: f64,
    idle_s: f64,
}

/// Spin before parking at a lock-step barrier: a window takes ~1 ms of wall time, so the partner
/// usually arrives within it; parking costs a scheduler wake-up (tens of µs on macOS).
fn rt_spin() -> std::time::Duration {
    std::time::Duration::from_micros(300)
}

#[allow(clippy::large_enum_variant)]
enum TcuMsg {
    Begin(TcuBoard),
    Window(TcuWindow),
    End,
    Shutdown,
}

#[allow(clippy::large_enum_variant)]
enum TcuReply {
    Window(TcuDone),
    End(Option<TcuBoard>, f64),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TcuSlot {
    Primary,
    CellDonor,
}

/// Persistent TCU worker thread of [`StudioSession`]: avoids spawning/joining an OS thread on every
/// 10 ms slice while keeping `StudioSession::tcu` directly accessible between `step()` calls.
struct TcuWorker {
    cmd_tx: Option<rt::Producer<TcuMsg>>,
    done_rx: rt::Consumer<TcuReply>,
    handle: Option<std::thread::JoinHandle<()>>,
    lent: Option<TcuSlot>,
    in_flight: usize,
}

impl TcuWorker {
    fn new() -> Result<Self, String> {
        let (cmd_tx, cmd_rx) = rt::spsc::<TcuMsg>(4);
        let (done_tx, done_rx) = rt::spsc::<TcuReply>(4);
        let handle = std::thread::Builder::new()
            .name("tcu-worker".into())
            .spawn(move || tcu_worker_loop(cmd_rx, done_tx))
            .map_err(|e| format!("spawn TCU worker: {e}"))?;
        Ok(Self {
            cmd_tx: Some(cmd_tx),
            done_rx,
            handle: Some(handle),
            lent: None,
            in_flight: 0,
        })
    }

    #[allow(clippy::result_large_err)]
    fn push(&self, msg: TcuMsg) -> Result<(), TcuMsg> {
        match &self.cmd_tx {
            Some(tx) => tx.push(msg),
            None => Err(msg),
        }
    }

    /// Drains any pending window replies and reclaims a lent `TcuBoard` back into its owning slot
    /// (`Primary` → `tcu`, `CellDonor` → `cell_tcu`), including safely after an unwind.
    fn reclaim_all(
        &mut self,
        tcu: &mut Option<TcuBoard>,
        cell_tcu: &mut Option<TcuBoard>,
        err: &mut Option<String>,
    ) -> Option<f64> {
        let which = self.lent?;
        let mut cell_err = None;
        let (slot, err_out) = match which {
            TcuSlot::Primary => (tcu, err),
            TcuSlot::CellDonor => (cell_tcu, &mut cell_err),
        };
        while self.in_flight > 0 {
            if self.done_rx.recv_opt(rt_spin()).is_some() {
                self.in_flight -= 1;
            } else {
                self.in_flight = 0;
                self.lent = None;
                *err_out = Some("TCU worker disconnected".into());
                return None;
            }
        }
        while self.done_rx.pop().is_some() {}
        if self.push(TcuMsg::End).is_err() {
            self.lent = None;
            *err_out = Some("TCU worker disconnected".into());
            return None;
        }
        self.lent = None;
        match self.done_rx.recv_opt(rt_spin()) {
            Some(TcuReply::End(Some(t), cpu)) => {
                *slot = Some(t);
                Some(cpu)
            }
            Some(TcuReply::End(None, _)) => {
                *err_out = Some("TCU worker lost board".into());
                None
            }
            _ => {
                *err_out = Some("TCU worker disconnected".into());
                None
            }
        }
    }
}

impl Drop for TcuWorker {
    fn drop(&mut self) {
        while self.done_rx.pop().is_some() {}
        if let Some(tx) = self.cmd_tx.take() {
            let _ = tx.push(TcuMsg::Shutdown);
            drop(tx);
        }
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn tcu_worker_loop(cmd: rt::Consumer<TcuMsg>, done: rt::Producer<TcuReply>) {
    rt::set_thread_role(rt::ThreadRole::Realtime);
    let mut tcu: Option<TcuBoard> = None;
    let mut cpu0 = 0.0;
    loop {
        let i0 = Instant::now();
        let Some(msg) = cmd.recv_opt(rt_spin()) else { break };
        match msg {
            TcuMsg::Begin(t) => {
                tcu = Some(t);
                cpu0 = rt::thread_cpu_s();
            }
            TcuMsg::Window(w) => {
                let b0 = Instant::now();
                let reply = if let Some(t) = tcu.as_mut() {
                    for f in &w.rx {
                        t.inject_can_rx(f);
                    }
                    t.feed(&w.tel, &w.driver, w.coils);
                    let err = {
                        let c = t.cpu_mut();
                        c.run_for(w.until_s - c.hw().sim_time_s).err()
                    };
                    let tx = t.take_can_tx();
                    let out = t.decode();
                    let (busy_s, idle_s) = (b0.elapsed().as_secs_f64(), (b0 - i0).as_secs_f64());
                    TcuDone { tx, out, err, busy_s, idle_s }
                } else {
                    TcuDone {
                        tx: Vec::new(),
                        out: Default::default(),
                        err: Some("TCU worker missing board".into()),
                        busy_s: 0.0,
                        idle_s: 0.0,
                    }
                };
                let _ = done.push(TcuReply::Window(reply));
            }
            TcuMsg::End => {
                let _ = done.push(TcuReply::End(tcu.take(), rt::thread_cpu_s() - cpu0));
            }
            TcuMsg::Shutdown => break,
        }
    }
}
#[derive(Debug, Clone, Serialize)]
pub struct CanLogEntry {
    pub t_ms: f64,
    pub src: &'static str,
    pub id: String,
    pub data: String,
}

#[derive(Debug, Clone, Serialize, Default)]
pub struct ImmoStats {
    pub ecu_challenges: u64,
    pub ecu_answers: u64,
    pub tcu_challenges: u64,
    pub tcu_answers: u64,
    /// Engine-ECU WFS5 credentials were decrypted from its DFlash (false → placeholder keys).
    pub ecu_key_known: bool,
    /// How the immobiliser is bound (lab key set or dump data), stated at load ([`crate::studio::immo_lab`]).
    pub binding: String,
    /// Lab binding selected (false = the dump's own credentials).
    pub lab: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct BoardStatus {
    pub present: bool,
    pub sw_id: String,
    pub pc: String,
    pub region: &'static str,
    pub stm_ms: f64,
    pub irq_count: u64,
    pub warm_resets: u32,
    pub rtos_tasks_seen: usize,
    pub rtos_task_runs: u64,
    pub can_tx_total: u64,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Serialize, Default)]
pub struct FrameStats {
    pub sim_dt_s: f64,
    pub wall_s: f64,
    /// Simulated time / wall time of the last step (1.0 = real time).
    pub realtime_factor: f64,
    /// Where the wall time of the last step went (see [`CosimPerf`]).
    pub perf: CosimPerf,
    /// Wall-clock pacing of the real-time runner (absent when the UI drives `/api/step`).
    pub pacing: Option<PacingStats>,
}

/// Cost of the last [`StudioSession::step`] per component (s of wall time unless noted). The ECU and
/// the plant share the session thread (the plant step of a window produces the sensor edges the ECU
/// consumes in that window); the TCU runs in parallel on its worker thread.
#[derive(Debug, Clone, Serialize, Default)]
pub struct CosimPerf {
    pub windows: u64,
    pub plant_s: f64,
    pub ecu_s: f64,
    /// TCU firmware + I/O on the worker thread.
    pub tcu_s: f64,
    /// Session thread blocked at the end of a window waiting for the TCU (lock-step barrier).
    pub wait_tcu_s: f64,
    /// Worker blocked waiting for the next window.
    pub tcu_idle_s: f64,
    /// CAN bridge, chassis frames, workstation, dyno, telemetry latch.
    pub glue_s: f64,
    /// Longest wall time of one 1 ms window (session thread).
    pub max_window_s: f64,
    /// CPU time of the session thread and of the TCU worker (thread clocks).
    pub main_cpu_s: f64,
    pub tcu_cpu_s: f64,
}

/// Real-time runner state ([`crate::studio::rt::Pacer`]).
#[derive(Debug, Clone, Serialize, Default)]
pub struct PacingStats {
    /// Target speed (1 = real time; < 1 only when selected by the user).
    pub speed: f64,
    /// Realised simulated/wall ratio over the last 0.5 s.
    pub rtf: f64,
    /// Wall time behind schedule now (s).
    pub lag_s: f64,
    /// Schedule given up after overruns (s), and their count.
    pub slip_s: f64,
    pub overruns: u64,
}

#[derive(Debug, Clone, Serialize)]
pub struct DynoSample {
    pub rpm: f64,
    pub torque_nm: f64,
    pub power_ps: f64,
    pub map_kpa: f64,
    pub put_kpa: f64,
    pub lambda: f64,
    pub iat_c: f64,
    pub egt_c: f64,
    pub turbo_rpm: f64,
}

/// Pre-sweep stabilisation at `rpm_start` once the engine reaches `rpm_start` (s): lets boost build and the eddy-current
/// absorber speed controller settle before recording points.
const DYNO_SETTLE_S: f64 = 2.0;
/// Maximum pre-sweep wait from `start` before ramping begins even if `rpm_start` was not reached (s).
const DYNO_PRELOAD_MAX_S: f64 = 4.0;
/// Brief hold at `rpm_end` so the physical engine speed crosses the final sample point before stopping (s).
const DYNO_END_HOLD_S: f64 = 0.5;

/// Engine-dyno sweep run by the test-bench equipment: pedal held at 100 %, absorber speed target
/// settled at `rpm_start` and then ramped to `rpm_end`. The firmware does all engine control.
#[derive(Debug, Clone, Serialize)]
pub struct DynoProgram {
    pub running: bool,
    pub sweeping: bool,
    pub completed: bool,
    pub rpm_start: f64,
    pub rpm_end: f64,
    pub ramp_rpm_s: f64,
    pub target_rpm: f64,
    pub samples: Vec<DynoSample>,
    next_sample_rpm: f64,
    #[serde(skip)]
    preload_s: f64,
    #[serde(skip)]
    settled_s: f64,
    #[serde(skip)]
    end_hold_s: f64,
    #[serde(skip)]
    prev_bench: Option<BenchMode>,
    #[serde(skip)]
    prev_selector: Option<Selector>,
}

impl Default for DynoProgram {
    fn default() -> Self {
        Self {
            running: false,
            sweeping: false,
            completed: false,
            rpm_start: 1500.0,
            rpm_end: 6500.0,
            ramp_rpm_s: 300.0,
            target_rpm: 0.0,
            samples: Vec::new(),
            next_sample_rpm: 0.0,
            preload_s: 0.0,
            settled_s: 0.0,
            end_hold_s: 0.0,
            prev_bench: None,
            prev_selector: None,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ImageInfo {
    /// File name relative to the workspace root.
    pub file: String,
    /// Calibration box code at CAL+2 (e.g. `SC8S50`).
    pub box_code: String,
    /// DAMOS definition matched through `definitions/index.json`, if present locally.
    pub damos: Option<String>,
}

fn box_code(img: &[u8]) -> String {
    img.get(0x200002..0x200008).map(|b| String::from_utf8_lossy(b).into_owned()).unwrap_or_default()
}

/// Finds the DAMOS file whose index entry verifies against `code` at CAL+2 (file 0x200002).
fn damos_for(root: &Path, code: &str) -> Option<String> {
    let idx: serde_json::Value = serde_json::from_slice(&std::fs::read(root.join("definitions/index.json")).ok()?).ok()?;
    idx.as_array()?.iter().find_map(|e| {
        let v = e.get("verification")?;
        let hit = v.get("position")?.as_u64()? == 0x200002 && v.get("expected")?.as_str()? == code;
        let file = format!("definitions/{}", e.get("file")?.as_str()?);
        (hit && root.join(&file).exists()).then_some(file)
    })
}

/// Bootable SIMOS18.1 images (`.bin`, `.frf`, `.odx`) in `vault/firmwares/ecu/{stock,frf,bench,custom}/`
/// (plus `$SIMEMU_IMAGE_DIR` and workspace root), deduplicated.
pub fn list_images(root: &Path) -> Vec<ImageInfo> {
    let mut candidates = vec![S50_FLASH.to_string(), paths::STOCK_264K_FLASH.to_string()];
    for sub in [
        "vault/firmwares/ecu/stock",
        "vault/firmwares/ecu/frf",
        "vault/firmwares/ecu/bench",
        "vault/firmwares/ecu/custom",
        "vault/firmwares/ecu",
        "firmwares/ecu/stock",
        "firmwares/ecu/frf",
        "firmwares/ecu/bench",
        "firmwares/ecu/custom",
        "ecu/stock",
        "ecu/frf",
        "",
    ] {
        for (name, _) in paths::list_ecu_images_in_subdir(root, sub) {
            candidates.push(name);
        }
    }

    let mut seen_basenames = HashSet::new();
    let mut seen_paths = HashSet::new();
    candidates
        .into_iter()
        .filter_map(|file| {
            let resolved = paths::resolve_firmware(root, &file)?;
            let actual_name = resolved
                .file_name()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or(file);
            let canon = std::fs::canonicalize(&resolved).unwrap_or_else(|_| resolved.clone());
            if !seen_paths.insert(canon) || !seen_basenames.insert(actual_name.clone()) {
                return None;
            }
            let cal_head = paths::peek_ecu_cal_header(&resolved, 0x10)?;
            let code = String::from_utf8_lossy(cal_head.get(2..8)?).trim().to_string();
            if code.is_empty() {
                return None;
            }
            Some(ImageInfo {
                damos: damos_for(root, &code),
                box_code: code,
                file: actual_name,
            })
        })
        .collect()
}

#[derive(Serialize)]
pub struct StudioState<'a> {
    pub ecu: BoardStatus,
    pub tcu: BoardStatus,
    pub immo: &'a ImmoStats,
    pub uds_ecu: &'a UdsStatus,
    pub uds_tcu: &'a UdsStatus,
    pub can_recent: Vec<&'a CanLogEntry>,
    pub plant: PlantTelemetry,
    pub driver: &'a DriverInputs,
    pub bench: BenchMode,
    pub paused: bool,
    pub frame: &'a FrameStats,
    pub engine_profile: &'a EngineProfile,
    pub drivetrain_profile: &'a DrivetrainProfile,
    pub ambient: Ambient,
    pub fuel_ron: f64,
    pub drive_profile: u8,
    pub image_label: &'a str,
    pub active_preset: &'a str,
    pub curated_maps: &'static [CuratedTunerMap],
    pub dyno: &'a DynoProgram,
    /// Checksum state of the current image (the S50 dump ships with an un-CRC'd immobiliser patch in ASW1).
    pub checksums: Option<&'a ChecksumReport>,
    pub images: &'a [ImageInfo],
    pub image_file: &'a str,
    pub image_notes: &'a [String],
    /// Workstation equipment (recorder, test-cell interlock, dyno correction).
    pub workstation: crate::studio::workstation::WsStatus,
    /// Physical car on the bench (`/api/build` has the details).
    pub bench_build: crate::studio::bench_build::BuildBrief<'a>,
    /// What the TCU reports on the bus (its own frames, observation only).
    pub gearbox_bus: GearboxBus,
    /// DQ250 mechatronic preparation ([`crate::studio::dq250_prep`]) and the notes of the TCU's last power-up.
    pub tcu_prep: crate::studio::dq250_prep::PrepView<'a>,
    pub tcu_notes: &'a [String],
    /// Pre-flash check ([`crate::studio::tune_check`]): progress, verdict of the last report.
    pub tune_check: crate::studio::tune_check::CheckView,
}

/// TCU signals from its last frames on the bus (opendbc `vw_mqb.dbc`, see `drive_run`); `None` = frame not seen.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct GearboxBus {
    /// `WBA_03.WBA_eing_Gang_02` (0x394 24|4): engaged gear as the TCU reports it.
    pub gear: Option<u8>,
    /// `Getriebe_11.GE_Fahrstufe` (0x0AD 42|4): selector position as the TCU uses it (DQ200 table 0x801382BC: 5 P, 6 R,
    /// 7 N, 8 D, 9 S, 14 tiptronic).
    pub fahrstufe: Option<u8>,
    /// `Getriebe_11.GE_Zielgang` (60|4): target gear.
    pub zielgang: Option<u8>,
}

#[derive(Deserialize, Default)]
pub struct StepRequest {
    pub dt_sec: Option<f64>,
    pub pedal_pct: Option<f64>,
    pub brake_pct: Option<f64>,
    pub selector: Option<Selector>,
    pub ignition_on: Option<bool>,
    pub start_button: Option<bool>,
    pub drive_profile: Option<u8>,
    pub bench_mode: Option<String>,
    pub dyno_target_rpm: Option<f64>,
    /// Roller speed held by the chassis-dyno absorber, km/h (0 = inertia run).
    pub dyno_target_kmh: Option<f64>,
    pub paused: Option<bool>,
    /// Real-time runner speed (1 = real time, < 1 slow motion).
    pub speed: Option<f64>,
}

#[derive(Deserialize)]
pub struct WriteMapRequest {
    pub id: String,
    pub values: Vec<Vec<f64>>,
}

#[derive(Deserialize)]
pub struct PresetRequest {
    pub action: String,
    pub engine_slug: Option<String>,
    pub drivetrain_slug: Option<String>,
    pub fuel_ron: Option<f64>,
    pub ambient_temp_c: Option<f64>,
    pub altitude_m: Option<f64>,
    pub humidity_pct: Option<f64>,
    /// For `load_image`: a `file` from `StudioState.images`.
    pub image: Option<String>,
}

#[derive(Deserialize)]
pub struct DynoRequest {
    pub action: String,
    pub rpm_start: Option<f64>,
    pub rpm_end: Option<f64>,
    pub ramp_rpm_s: Option<f64>,
}

#[derive(Deserialize)]
pub struct UdsRequest {
    pub target: String,
    pub service: String,
    /// `service: "raw"`: request bytes (any length up to 4095; > 7 bytes go multi-frame), sent as a tester would.
    #[serde(default)]
    pub data: Vec<u8>,
    /// `write_coding`: the coding as the decimal number a workshop tester shows (e.g. 24 for "000024").
    #[serde(default)]
    pub value: Option<u32>,
    /// `basic_setting` / `routine_stop`: routine identifier (e.g. 0x0380).
    #[serde(default)]
    pub rid: Option<u16>,
}

/// Workshop code the bench tester writes before coding (`2E F198`, 6 bytes: the DQ250 firmware stores them
/// unchecked with the coding, EEPROM 0x1C4; the field layout of real testers is UNVERIFIED).
pub const BENCH_WORKSHOP_CODE: [u8; 6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x01];

pub struct StudioSession {
    pub root: PathBuf,
    pub board: Simos18Board,
    /// DFlash dump as read from disk (never written back).
    pub ecu_eeprom: Vec<u8>,
    /// DFlash the ECU board runs with: the dump bound to the lab key set, or the dump itself.
    pub ecu_nvm: Vec<u8>,
    /// Lab immobiliser binding (default) or the dump's own credentials (`SIMEMU_IMMO=dump`, preset `immo_dump`).
    pub immo_lab: bool,
    /// Lab key set and power class the TCU EEPROM is bound with (`None`: its dump).
    tcu_lab: Option<(LabKeys, u8)>,
    pub ecu_error: Option<String>,
    pub tcu: Option<TcuBoard>,
    tcu_worker: TcuWorker,
    pub tcu_error: Option<String>,
    /// The TCU's non-volatile memory across power cycles ([`TcuNvm`]).
    pub tcu_nvm: TcuNvm,
    /// DTC number -> SAE code table of a DQ250 TCU image (`tcu_dq250::dtc_table`); empty for the DQ200 (its decoder
    /// reads the image from disk, [`dq200_dtc_sae`]).
    tcu_dtc_table: Vec<(u16, u16)>,
    pub cluster_ecu: Wfs5ImmoPartner,
    pub cluster_tcu: Option<Wfs5ImmoPartner>,
    pub immo: ImmoStats,
    pub chassis: MqbChassisEncoder,
    tcu_chassis: MqbChassisEncoder,
    pub can_log: VecDeque<CanLogEntry>,
    /// Latest frame per (sender, identifier) seen on the bus (observation only).
    pub can_last: std::collections::BTreeMap<(&'static str, u32), CanFrame>,
    /// Tester flow-control / consecutive frames queued for the ECU and TCU (paced 1 per window / `STmin`).
    uds_fc_next: VecDeque<CanFrame>,
    uds_ecu_cooldown: usize,
    uds_tcu_fc_next: VecDeque<CanFrame>,
    uds_tcu_cooldown: usize,
    /// Optional live SLCAN-over-TCP bridge attached to the virtual vehicle CAN bus.
    pub slcan: Option<SlcanServer>,
    pub ecu_can_tx_total: u64,
    pub tcu_can_tx_total: u64,
    pub uds_ecu: UdsClient,
    pub uds_tcu: UdsClient,
    pub damos: DamosDatabase,
    pub stock_flash: Vec<u8>,
    pub current_flash: Vec<u8>,
    pub plant: VirtualPlant,
    pub actuators: PlantActuators,
    pub drive_profile: u8,
    pub paused: bool,
    pub image_label: String,
    pub active_preset: String,
    pub dyno: DynoProgram,
    pub frame: FrameStats,
    pub checksums: Option<ChecksumReport>,
    pub images: Vec<ImageInfo>,
    pub image_file: String,
    /// Post-programming completion steps applied to the loaded image (empty = loaded byte-exact).
    pub image_notes: Vec<String>,
    pub ws: crate::studio::workstation::Workstation,
    /// Plant ↔ ECU time base of the crank/cam signals.
    pub ecu_link: io::EcuSensorLink,
    /// Physical sound: probe → synthesizer thread (active only while a listener is connected).
    pub audio: simemu_acoustics::AudioRt,
    /// Bench wiring-harness faults (unplugged coil / injector connectors).
    pub harness: crate::studio::audio::Harness,
    /// Bench input: value of the board's engine-off timer (PIC12LF1822, KL30-supplied) when the ECU powers up, s —
    /// the time since the firmware last reset it (it does so after each engine start, FR ENSS 33.4
    /// LC_T_ES_HW_RST_AST), i.e. the previous run plus the engine-off duration. Applied at the next
    /// [`Self::power_cycle_ecu`].
    pub eot_clock_s: f64,
    /// A real-time runner thread steps the session (`/api/step` then only applies inputs).
    pub realtime: bool,
    /// Speed the runner paces to (1 = real time; slow motion only when the user selects it).
    pub speed: f64,
    /// Bench build (physical car), independent of the firmware image; applied at bench start.
    pub bench_build: crate::studio::bench_build::BuildState,
    /// Workshop preparation of a DQ250 mechatronic with an erased EEPROM ([`crate::studio::dq250_prep`]).
    pub tcu_prep: crate::studio::dq250_prep::Prep,
    /// What the studio did with the TCU at its last power-up (EEPROM origin, preparation, gear set), shown in the UI.
    pub tcu_notes: Vec<String>,
    /// DQ250 image file the running TCU booted from.
    pub tcu_image_path: Option<PathBuf>,
    /// DQ250 supply ([`Self::dq250_supply`]): TCU frames sent before its power-up, (frames, TCU board time) when it was
    /// last heard, terminal 15 at the last step.
    tcu_tx_boot: u64,
    tcu_heard: (u64, f64),
    tcu_kl15: bool,
    /// Terminal 15 was on since the TCU's power-up.
    tcu_woken: bool,
    /// Terminal 15 has woken the ECU at least once since its power-up.
    ecu_woken: bool,
    /// The ECU's main relay (`KL87`) is holding board power (false after Nachlauf finishes with KL15 off).
    ecu_powered: bool,
    /// ECU DFlash at its power-up or last save ([`Self::save_ecu_dflash`]); plant time of the last periodic check.
    ecu_dflash_saved: Vec<u8>,
    ecu_dflash_t: f64,
    /// `eot_clock_s` value the current `board` was booted with (used to re-boot cleanly on cold-standby KL15 wake if EOT advanced).
    ecu_boot_eot_s: f64,
    /// Pre-warmed bench standby: holds initial soak temperatures (90 °C engine, 70 °C gearbox) while the car is sitting
    /// with ignition off before the operator starts the session.
    pub(crate) warm_standby: bool,
    /// Pre-flash check of an image ([`crate::studio::tune_check`]): the running check and the last report.
    pub tune_check: crate::studio::tune_check::Check,
    /// Test-cell donor mechatronic (`DQ200_G2` `0CW300043H` at rest in `P`) wired onto the powertrain CAN harness only
    /// during `EngineDyno` / `AcDyno` runs when `self.tcu.is_none()` (e.g. `DQ381` / `DQ250 02E` builds without a gearbox
    /// TCU dump), so an AT/DCT-coded SIMOS18.1 ECU does not enter `U0101` (`COM_TCU_TOUT`) → `LV_N_MAX_SUB` (`IP_N_MAX_SUB`
    /// 5152 rpm substitute speed limit, FR p.8229). Uncoupled from the plant and diagnostic `0x7E1`.
    cell_tcu: Option<TcuBoard>,
    cell_cluster_tcu: Option<Wfs5ImmoPartner>,
    cell_tcu_unavailable: bool,
}

/// Non-volatile memory of the TCU (the DQ250's SPI EEPROM; no factory dump exists): what the firmware wrote
/// survives power cycles like a mechatronic in a car — coding, adaptations, fault memory. Kept per TCU image (its
/// identification string) in memory and, when `dir` is set (studio server: `build/studio_state/`, not tracked),
/// in `<dir>/tcu_<image id>.eeprom.bin`. Source dump files are never written.
#[derive(Default)]
pub struct TcuNvm {
    pub(crate) mem: std::collections::BTreeMap<String, Vec<u8>>,
    pub dir: Option<PathBuf>,
    /// EEPROM bytes the running board had written at the last save.
    saved_written: u64,
    /// Image whose state file is not written while its workshop preparation runs: an interrupted preparation leaves no
    /// state behind, the next start prepares again ([`crate::studio::dq250_prep`]).
    pub(crate) hold: Option<String>,
}

impl TcuNvm {
    fn path(&self, id: &str, ext: &str) -> Option<PathBuf> {
        self.dir.as_ref().map(|d| d.join(format!("tcu_{}.{ext}", safe_name(id))))
    }

    /// ECU DFlash state of ECU image file `image` (`ecu_<image>.dflash.bin`, same directory), enabled only when
    /// `SIMEMU_KEEP_ECU_DFLASH=1` so fresh bench sessions always start from the clean factory DFlash without
    /// carrying over faults from previous runs.
    pub(crate) fn ecu_file(&self, image: &str) -> Option<PathBuf> {
        if !std::env::var("SIMEMU_KEEP_ECU_DFLASH").is_ok_and(|v| v == "1") {
            return None;
        }
        self.dir.as_ref().map(|d| d.join(format!("ecu_{}.dflash.bin", safe_name(image))))
    }

    pub(crate) fn file(&self, id: &str) -> Option<PathBuf> {
        self.path(id, "eeprom.bin")
    }

    /// Record of the workshop preparation that produced the state of image `id` (`tcu_<id>.prep.json`).
    pub(crate) fn record_file(&self, id: &str) -> Option<PathBuf> {
        self.path(id, "prep.json")
    }

    /// A saved state to power image `id` up with exists: the state file (without a state directory: this session's
    /// memory) or `SIMEMU_DQ250_EEPROM`. An erased part a session powered up with is not one.
    pub fn has_state(&self, id: &str) -> bool {
        let saved = match self.file(id) {
            Some(p) => p.exists(),
            None => self.mem.contains_key(id),
        };
        saved || std::env::var("SIMEMU_DQ250_EEPROM").is_ok_and(|p| Path::new(&p).is_file())
    }

    /// Sets the EEPROM of image `id` (memory and state file).
    pub(crate) fn put(&mut self, id: &str, mem: Vec<u8>) {
        if let Some(p) = self.file(id) {
            if let Err(e) = p.parent().map_or(Ok(()), std::fs::create_dir_all).and_then(|_| std::fs::write(&p, &mem)) {
                if simemu_damos::is_ru() {
                    eprintln!("  ТКУ: EEPROM не сохранён в {}: {e}", p.display());
                } else {
                    eprintln!("  TCU: failed to save EEPROM to {}: {e}", p.display());
                }
            }
        }
        self.mem.insert(id.to_string(), mem);
    }

    /// Drops the state of image `id` (memory, state file, preparation record); `erased` = an erased part is its saved
    /// state from now on (an empty state file), otherwise as on a first start.
    pub(crate) fn reset(&mut self, id: &str, erased: bool) {
        for p in [self.file(id), self.record_file(id)].into_iter().flatten() {
            let _ = std::fs::remove_file(p);
        }
        self.mem.remove(id);
        if erased {
            self.put(id, Vec::new());
        }
    }

    /// EEPROM content to power up image `id` with: kept from the last power cycle, else the state file, else
    /// `SIMEMU_DQ250_EEPROM`, else empty (erased part).
    pub fn load(&self, id: &str) -> Vec<u8> {
        self.mem
            .get(id)
            .cloned()
            .or_else(|| self.file(id).and_then(|p| std::fs::read(p).ok()))
            .or_else(|| std::env::var("SIMEMU_DQ250_EEPROM").ok().and_then(|p| std::fs::read(p).ok()))
            .unwrap_or_default()
    }

    /// Keeps the running board's EEPROM (and writes the state file when it changed since the last save).
    pub fn keep(&mut self, tcu: Option<&TcuBoard>) {
        let Some((id, mem, written)) = tcu.and_then(TcuBoard::nvm) else { return };
        if written == 0 {
            return;
        }
        if self.mem.get(id).is_some_and(|m| m == mem) {
            self.saved_written = written;
            return;
        }
        self.mem.insert(id.to_string(), mem.to_vec());
        self.saved_written = written;
        if let Some(p) = self.file(id).filter(|_| self.hold.as_deref() != Some(id)) {
            let r = p.parent().map_or(Ok(()), std::fs::create_dir_all).and_then(|_| std::fs::write(&p, mem));
            if let Err(e) = r {
                if simemu_damos::is_ru() {
                    eprintln!("  ТКУ: EEPROM не сохранён в {}: {e}", p.display());
                } else {
                    eprintln!("  TCU: failed to save EEPROM to {}: {e}", p.display());
                }
            }
        }
    }

    /// Board wrote its EEPROM since the last save.
    fn dirty(&self, tcu: Option<&TcuBoard>) -> bool {
        tcu.and_then(TcuBoard::nvm).is_some_and(|(_, _, w)| w != self.saved_written)
    }
}

/// File-name-safe form of an image id / file name.
fn safe_name(s: &str) -> String {
    s.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '-' }).collect()
}

/// Cluster credentials when the ECU's are unknown: its answers will be rejected.
fn no_cluster_key() -> Wfs5ImmoPartner {
    Wfs5ImmoPartner::from_mqb_slots_ecu(&[0; 16], &[0; 16], 0, [0, 0], [0, 0], "")
}

/// DQ200_G2 DTCs (`19 02` records "XXXXXX status 0xYY", the VAG number in the low 16 bits) decoded with the firmware's
/// own tables (`Dq200TcuBoard::dtc_sae_code`: 350 VAG numbers at `0x80032AD0`, then their SAE codes): "U0002 (event 46)".
pub(crate) fn dq200_dtc_sae(root: &Path, dtcs: &[String]) -> Vec<String> {
    use simemu_tc1791::tcu_dq200::{DTC_EVENTS, DTC_VAG_TABLE};
    let f = paths::read_firmware(root, DQ200_FLASH).unwrap_or_default();
    let w = |i: usize| f.get(i..i + 2).map(|b| u16::from_le_bytes([b[0], b[1]]));
    let base = (DTC_VAG_TABLE - 0x8000_0000) as usize;
    dtcs.iter()
        .map(|d| {
            let vag = d.get(..6).and_then(|x| u32::from_str_radix(x, 16).ok()).map(|x| (x & 0xFFFF) as u16);
            let ev = (0..DTC_EVENTS).find(|&i| vag.is_some() && w(base + 2 * i) == vag);
            match ev.and_then(|i| w(base + 2 * (DTC_EVENTS + i)).map(|s| (i, s))) {
                Some((i, s)) => format!("{}{:04X} (event {i})", ["P", "C", "B", "U"][(s >> 14) as usize], s & 0x3FFF),
                None => "?".into(),
            }
        })
        .collect()
}

/// DQ250 DTC records ("XXXXXX status 0xYY") decoded with the image's own table (`tcu_dq250::dtc_table`): "P0805".
fn dq250_dtc_sae(table: &[(u16, u16)], dtcs: &[String]) -> Vec<String> {
    dtcs.iter()
        .map(|d| {
            let n = d.get(..6).and_then(|x| u32::from_str_radix(x, 16).ok());
            n.and_then(|n| table.iter().find(|e| e.0 as u32 == n)).map_or_else(|| "?".into(), |e| simemu_tc1791::tcu_dq250::sae_text(e.1))
        })
        .collect()
}

fn hex(d: &[u8]) -> String {
    d.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
}

fn can(id: u32, dlc: u8, data: [u8; 8]) -> CanFrame {
    CanFrame { node: 0, mo_index: 0, arb_id: id, extended: id > 0x7FF, dlc, data, timestamp_us: 0 }
}

impl StudioSession {
    /// Session on the environment's build (SIMEMU_BUILD / SIMEMU_ENGINE / SIMEMU_TURBO) or the baseline preset.
    pub fn new(root: &Path) -> Result<Self, String> {
        Self::with_build(root, crate::studio::bench_build::BuildState::from_env()?)
    }

    /// Session with the physical car `bench_build`, booting the matching factory stock FRF image by default.
    pub fn with_build(root: &Path, bench_build: crate::studio::bench_build::BuildState) -> Result<Self, String> {
        let init_file = stock_ecu_image(bench_build.active.engine);
        let chosen_req = if paths::has_firmware(root, init_file) { init_file } else { S50_FLASH };
        let chosen_path = paths::resolve_firmware_or_err(root, chosen_req)?;
        let chosen_name = chosen_path
            .file_name()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| chosen_req.to_string());
        let mut flash = paths::read_firmware_from_path(root, &chosen_path)?;
        if flash.len() == 0x40_0000 && flash[..0x1_C000].iter().all(|&b| b == 0 || b == 0xFF) {
            let (_, base) = paths::read_sboot_donor(root)?;
            flash[..0x1_C000].copy_from_slice(&base[..0x1_C000]);
        }
        let mut init_notes = simemu_damos::apply_post_programming(&mut flash)?;
        let completed = !init_notes.is_empty();
        if completed {
            init_notes.insert(
                0,
                simemu_damos::tr(
                    "Image completed after programming (source file unchanged):",
                    "Образ дозавершён после программирования (исходный файл не изменён):",
                )
                .into(),
            );
        }
        let (_, ecu_eeprom) = paths::read_dflash_or_default(root)?;
        let code = box_code(&flash);
        let damos_rel = damos_for(root, &code).unwrap_or_else(|| S50_DAMOS.to_string());
        let damos = DamosDatabase::load_from_json(&root.join(&damos_rel))?;
        let mut plant = bench_build.new_plant()?;
        plant.driver.selector = Selector::P;
        let mut ecu_link = io::EcuSensorLink::default();
        // Immobiliser: the virtual cluster and both slaves bound to one lab key set (docs/SIMOS18_WFS5_RE.md §8).
        let immo_lab = std::env::var("SIMEMU_IMMO").map_or(true, |v| v != "dump");
        let bind = immo_lab::ecu_binding(immo_lab, &flash, &ecu_eeprom);
        eprintln!("{}", bind.notes.join("\n  "));
        init_notes.extend(bind.notes);
        let mut board = Simos18Board::new(&flash, &bind.nvm)?;
        let ecu_error = io::power_on_ecu(&mut board, &mut ecu_link, &plant, ECU_BOOT_INSNS).err();

        let tcu_nvm = TcuNvm::default();
        let (tcu, tcu_error, cluster_tcu) = if plant.drivetrain_profile.has_tcu_firmware {
            Self::boot_tcu(root, &plant, bench_build.active.tcu_image.as_deref(), bind.tcu.as_ref(), &tcu_nvm)
        } else {
            (None, Some(crate::studio::bench_build::no_tcu_msg(&plant)), None)
        };
        let tcu_dtc_table = tcu.as_ref().and_then(TcuBoard::dq250).map(|t| t.dtcs.clone()).unwrap_or_default();
        let ecu_key_known = bind.cluster.is_some();
        let label_tag = bind
            .tcu
            .as_ref()
            .map(|(k, _)| {
                if simemu_damos::is_ru() {
                    format!(" · иммо: лаб. привязка {}", k.id)
                } else {
                    format!(" · immo: lab bound {}", k.id)
                }
            })
            .unwrap_or_default();
        let completed_tag = if completed {
            simemu_damos::tr(" ⚑ completed after programming", " ⚑ дозавершён после программирования")
        } else {
            ""
        };

        let mut s = Self {
            root: root.to_path_buf(),
            board,
            ecu_eeprom,
            ecu_nvm: bind.nvm,
            immo_lab,
            tcu_lab: bind.tcu,
            ecu_error,
            tcu,
            tcu_worker: TcuWorker::new()?,
            tcu_error,
            tcu_nvm,
            tcu_dtc_table,
            cluster_ecu: bind.cluster.unwrap_or_else(no_cluster_key),
            cluster_tcu,
            immo: ImmoStats { ecu_key_known, binding: bind.summary, lab: immo_lab, ..Default::default() },
            chassis: MqbChassisEncoder::default(),
            tcu_chassis: MqbChassisEncoder::default(),
            can_log: VecDeque::with_capacity(CAN_LOG_LEN),
            can_last: Default::default(),
            uds_fc_next: VecDeque::new(),
            uds_ecu_cooldown: 0,
            uds_tcu_fc_next: VecDeque::new(),
            uds_tcu_cooldown: 0,
            slcan: None,
            ecu_can_tx_total: 0,
            tcu_can_tx_total: 0,
            uds_ecu: UdsClient::new(0x7E0, 0x7E8),
            uds_tcu: UdsClient::new(0x7E1, 0x7E9),
            damos,
            stock_flash: flash.clone(),
            current_flash: flash,
            plant,
            actuators: PlantActuators::default(),
            drive_profile: 2,
            paused: false,
            image_label: format!("{code} ({chosen_name}){completed_tag}{label_tag}"),
            active_preset: "stock".to_string(),
            dyno: DynoProgram::default(),
            frame: FrameStats::default(),
            checksums: None,
            images: list_images(root),
            image_file: chosen_name,
            image_notes: init_notes,
            ws: Default::default(),
            ecu_link,
            audio: Default::default(),
            harness: Default::default(),
            eot_clock_s: 0.0,
            realtime: false,
            speed: 1.0,
            bench_build,
            tcu_prep: Default::default(),
            tcu_notes: Vec::new(),
            tcu_image_path: None,
            tcu_tx_boot: 0,
            tcu_heard: (0, 0.0),
            tcu_kl15: false,
            tcu_woken: false,
            ecu_woken: false,
            ecu_powered: true,
            ecu_dflash_saved: Vec::new(),
            ecu_dflash_t: 0.0,
            ecu_boot_eot_s: 0.0,
            warm_standby: false,
            tune_check: Default::default(),
            cell_tcu: None,
            cell_cluster_tcu: None,
            cell_tcu_unavailable: false,
        };
        s.ecu_dflash_saved = s.board.hw().dflash.to_vec();
        s.checksums = verify_checksums(&s.current_flash).ok();
        s.plant.driver.selector = Selector::P;
        s.bridge_can(0.0);
        let t0 = s.tcu.as_ref().map_or(0.0, |t| t.cpu().hw().sim_time_s);
        s.tcu_heard = (s.tcu_can_tx_total, t0);
        s.tcu_booted();
        Ok(s)
    }

    /// Boots the TCU board of the plant's gearbox (DQ200_G2 or DQ250 0D9, [`Self::boot_dq250`] with image `image`).
    pub(crate) fn boot_tcu(root: &Path, plant: &VirtualPlant, image: Option<&str>, lab: Option<&(LabKeys, u8)>, nvm: &TcuNvm) -> (Option<TcuBoard>, Option<String>, Option<Wfs5ImmoPartner>) {
        if !plant.drivetrain_profile.has_tcu_firmware {
            return (None, Some(crate::studio::bench_build::no_tcu_msg(plant)), None);
        }
        if plant.drivetrain_profile.tcu_code.starts_with("0D9") {
            return Self::boot_dq250(root, plant, image, lab, nvm);
        }
        let (t, e, p) = Self::boot_dq200(root, plant, lab);
        (t.map(TcuBoard::Dq200), e, p)
    }

    /// Resolves the DQ250 (`0D9`) image file to boot: explicit `image` (filename or OEM ASW ID substring such as
    /// `"0D9300011R"` / `"0D9300012F"`), else `SIMEMU_DQ250_IMAGE`, else the first discovered DQ250 image in
    /// `vault/firmwares/tcu/dq250/` (stock images with valid factory CBOOT CRCs preferred first).
    pub fn dq250_image_path(root: &Path, image: Option<&str>) -> Result<PathBuf, String> {
        let resolve_query = |q: &str, sandboxed: bool| -> Result<PathBuf, String> {
            if sandboxed {
                if let Ok(p) = paths::resolve_sandboxed_firmware(root, q) {
                    return Ok(p);
                }
            } else if let Some(p) = paths::resolve_firmware(root, q) {
                return Ok(p);
            }
            // Match by OEM ASW ID / part number substring (`0D9300011R`, `0D9300012F`, `4319`, `4930`, ...)
            let q_trim = q.trim();
            if !q_trim.is_empty() && q_trim.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-') {
                let q_up = q_trim.to_ascii_uppercase();
                for cand in crate::studio::dq250_prep::dq250_images(root) {
                    if cand.asw_id.to_ascii_uppercase().contains(&q_up) || cand.file.to_ascii_uppercase().contains(&q_up) {
                        return paths::resolve_sandboxed_firmware(root, &cand.file);
                    }
                }
            }
            if sandboxed {
                paths::resolve_sandboxed_firmware(root, q)
            } else {
                paths::resolve_firmware_or_err(root, q)
            }
        };
        if let Some(n) = image.filter(|s| !s.trim().is_empty()) {
            return resolve_query(n, true);
        }
        if let Ok(env_q) = std::env::var("SIMEMU_DQ250_IMAGE") {
            if !env_q.trim().is_empty() {
                return resolve_query(&env_q, false);
            }
        }
        if let Some(first) = crate::studio::dq250_prep::dq250_images(root).into_iter().next() {
            if let Some(p) = paths::resolve_firmware(root, &first.file) {
                return Ok(p);
            }
        }
        Err(if simemu_damos::is_ru() {
            format!("Прошивка ТКУ DQ250 (0D9, 1.5 МБ) не найдена в {}/vault/firmwares/tcu/dq250/", root.display())
        } else {
            format!("DQ250 (0D9, 1.5 MB) TCU firmware not found in {}/vault/firmwares/tcu/dq250/", root.display())
        })
    }

    /// Resolves the DQ250 (`0D9`) image for a given engine displacement (`engine_disp_m3`): when `image` is unset and
    /// `SIMEMU_DQ250_IMAGE` is unset, 1.8 TSI (`< 1.9 L`, `SC8O40` `8V0906264H`) prefers `0D9300012F` (`SW 4930`, OEM
    /// A3 8V 1.8 TFSI quattro, which transmits `0x0B1` `Getriebe_15` monitored by `SC8O40` for `U0101` `COM_TCU_TOUT`,
    /// `restbus_run`), falling back to [`Self::dq250_image_path`].
    pub fn dq250_image_path_for(root: &Path, engine_disp_m3: f64, image: Option<&str>) -> Result<PathBuf, String> {
        let auto_18t = image.filter(|s| !s.trim().is_empty()).is_none()
            && std::env::var("SIMEMU_DQ250_IMAGE").map_or(true, |v| v.trim().is_empty())
            && engine_disp_m3 < 1.9e-3;
        if auto_18t {
            Self::dq250_image_path(root, Some(paths::DQ250_012F_ID)).or_else(|_| Self::dq250_image_path(root, None))
        } else {
            Self::dq250_image_path(root, image)
        }
    }

    /// Boots the DQ250 0D9 (TC1766): image [`Self::dq250_image_path_for`] with the post-programming checksum completion
    /// (AGENTS.md §2.4 п.3); SPI EEPROM as the firmware left it at the last power-down ([`TcuNvm::load`]; first
    /// power-up: `SIMEMU_DQ250_EEPROM` or an erased part — no dump exists; the studio then prepares it,
    /// [`crate::studio::dq250_prep`]), bound in memory to `lab` ([`immo_lab::bind_dq250_tcu`]: only the IMMO data record,
    /// and only when it differs).
    pub(crate) fn boot_dq250(root: &Path, plant: &VirtualPlant, image: Option<&str>, lab: Option<&(LabKeys, u8)>, nvm: &TcuNvm) -> (Option<TcuBoard>, Option<String>, Option<Wfs5ImmoPartner>) {
        let path = match Self::dq250_image_path_for(root, plant.engine_profile.displacement_m3(), image) {
            Ok(p) => p,
            Err(e) => return (None, Some(e), None),
        };
        let mut flash = match std::fs::read(&path) {
            Ok(b) => b,
            Err(e) => return (None, Some(format!("{}: {e}", path.display())), None),
        };
        match simemu_damos::apply_post_programming_dq250(&mut flash) {
            Ok(notes) => notes.iter().for_each(|n| {
                if simemu_damos::is_ru() {
                    eprintln!("  ТКУ DQ250 {}: {n}", path.display());
                } else {
                    eprintln!("  TCU DQ250 {}: {n}", path.display());
                }
            }),
            Err(e) => return (None, Some(e), None),
        }
        let mut eeprom = nvm.load(&simemu_tc1791::tcu_dq250::Dq250ImageInfo::from_bytes(&flash).asw_id);
        let partner = match lab.map(|(k, b)| immo_lab::bind_dq250_tcu(&mut eeprom, k, *b)) {
            Some(Ok(p)) => {
                if simemu_damos::is_ru() {
                    eprintln!("  ТКУ DQ250: EEPROM 0x844/0x87C/0x8B4 привязан к {} (K1, B={:#04X}, ID_kombi, VIN; данные, не код)", lab.unwrap().0.id, p.power_class);
                } else {
                    eprintln!("  TCU DQ250: EEPROM 0x844/0x87C/0x8B4 bound to {} (K1, B={:#04X}, ID_kombi, VIN; data only, never code)", lab.unwrap().0.id, p.power_class);
                }
                Some(p)
            }
            Some(Err(e)) => return (None, Some(format!("DQ250 lab binding: {e}")), None),
            None => None,
        };
        match Dq250TcuBoard::new(&flash, &eeprom) {
            Ok(t) => {
                let mut d = plant.driver.clone();
                d.ignition_on = true;
                let mut tb = TcuBoard::Dq250(t);
                tb.feed(&plant.telemetry(), &d, crate::studio::tcu_io::coil_params(plant));
                (Some(tb), None, partner)
            }
            Err(e) => (None, Some(e), partner),
        }
    }

    /// Boots the DQ200 with its EEPROM dump, bound in memory to `lab` (key set, power class) when given.
    pub(crate) fn boot_dq200(root: &Path, plant: &VirtualPlant, lab: Option<&(LabKeys, u8)>) -> (Option<Dq200TcuBoard>, Option<String>, Option<Wfs5ImmoPartner>) {
        let flash = match paths::read_firmware(root, DQ200_FLASH) {
            Ok(b) => b,
            Err(e) => return (None, Some(e), None),
        };
        let mut eeprom = match paths::read_firmware(root, DQ200_EEPROM) {
            Ok(b) => b,
            Err(e) => return (None, Some(e), None),
        };
        let dump = Wfs5ImmoPartner::from_dq200_eeprom(&eeprom);
        let partner = match lab.map(|(k, b)| {
            let mut e = eeprom.clone();
            immo_lab::bind_tcu(&mut e, &flash, k, *b).map(|p| (e, p))
        }) {
            Some(Ok((e, p))) => {
                if simemu_damos::is_ru() {
                    eprintln!("  ТКУ: EEPROM 0x1E8/0x220/0x268 привязан к {} (K1, B={:#04X}, ID_kombi, VIN; данные, не код)", lab.unwrap().0.id, p.power_class);
                } else {
                    eprintln!("  TCU: EEPROM 0x1E8/0x220/0x268 bound to {} (K1, B={:#04X}, ID_kombi, VIN; data only, never code)", lab.unwrap().0.id, p.power_class);
                }
                eeprom = e;
                p
            }
            Some(Err(e)) => {
                if simemu_damos::is_ru() {
                    eprintln!("  ТКУ: лабораторная привязка не выполнена ({e}); используются данные дампа EEPROM");
                } else {
                    eprintln!("  TCU: lab binding skipped ({e}); falling back to EEPROM dump credentials");
                }
                dump
            }
            None => dump,
        };
        match Dq200TcuBoard::new(&flash, &eeprom) {
            Ok(mut t) => {
                io::feed_tcu(&mut t, &plant.telemetry());
                if let Some((r, l)) = crate::studio::tcu_io::coil_params(plant).and_then(|(r, l)| Some((<[f64; 8]>::try_from(r).ok()?, l))) {
                    t.set_coil_params(r, l);
                }
                let err = t.run_boot_trace(120, 25_000).err();
                // Clear historical fault records from the donor EEPROM via functional OBD 04 once EEPROM init finishes
                let _ = t.cpu.run_for(0.10);
                t.cpu.inject_can_rx(&can(0x7DF, 8, [0x01, 0x04, 0, 0, 0, 0, 0, 0]));
                let _ = t.cpu.run_for(0.05);
                let _ = t.cpu.take_can_tx();
                (Some(t), err, Some(partner))
            }
            Err(e) => (None, Some(e), Some(partner)),
        }
    }

    /// Attaches (or detaches) a live non-blocking SLCAN-over-TCP server to the virtual vehicle CAN bus.
    pub fn set_slcan(&mut self, server: Option<SlcanServer>) {
        self.slcan = server;
    }

    /// TCP port of the attached SLCAN server, if active.
    pub fn slcan_port(&self) -> Option<u16> {
        self.slcan.as_ref().map(SlcanServer::port)
    }

    /// Services SLCAN TCP handshakes/control commands while the co-simulation is paused.
    pub fn poll_slcan_idle(&mut self) {
        if let Some(srv) = self.slcan.as_mut() {
            srv.poll();
        }
    }

    fn broadcast_slcan(&mut self, f: &CanFrame, exclude_client: Option<u64>) {
        let now_us = (self.plant.sim_time_s * 1e6) as u64;
        if let Some(srv) = self.slcan.as_mut() {
            let mut stamped = *f;
            if stamped.timestamp_us == 0 {
                stamped.timestamp_us = now_us;
            }
            srv.broadcast(&stamped, exclude_client);
        }
    }

    fn poll_one_slcan_rx(&mut self, now_us: u64) -> Option<CanFrame> {
        let srv = self.slcan.as_mut()?;
        srv.poll();
        let (sender_id, mut ext) = srv.pop_rx()?;
        ext.timestamp_us = now_us;
        srv.broadcast(&ext, Some(sender_id));
        Some(ext)
    }

    fn log(&mut self, src: &'static str, f: &CanFrame) {
        self.can_last.insert((src, f.arb_id), *f);
        if self.can_log.len() == CAN_LOG_LEN {
            self.can_log.pop_front();
        }
        let n = (f.dlc as usize).min(8);
        self.can_log.push_back(CanLogEntry {
            t_ms: self.plant.sim_time_s * 1e3,
            src,
            id: if f.extended { format!("{:08X}", f.arb_id) } else { format!("{:03X}", f.arb_id) },
            data: hex(&f.data[..n]),
        });
    }

    /// Moves frames between the two boards and the virtual instrument cluster / tester / external SLCAN port.
    fn bridge_can(&mut self, _t: f64) {
        let tcu_tx: Vec<CanFrame> = self.tcu.as_mut().map(|t| t.take_can_tx()).unwrap_or_default();
        let to_tcu = self.bridge_can_frames(tcu_tx);
        if let Some(t) = self.tcu.as_mut() {
            for f in &to_tcu {
                t.inject_can_rx(f);
            }
        }
    }

    /// [`Self::bridge_can`] with the TCU's transmitted frames given; returns the frames for the TCU
    /// (the ECU's are injected here).
    fn bridge_can_frames(&mut self, tcu_tx: Vec<CanFrame>) -> Vec<CanFrame> {
        self.bridge_can_frames_inner(tcu_tx, false)
    }

    fn bridge_can_frames_inner(&mut self, tcu_tx: Vec<CanFrame>, donor_only: bool) -> Vec<CanFrame> {
        let now_us = (self.plant.sim_time_s * 1e6) as u64;
        let ecu_tx: Vec<CanFrame> = self.board.take_can_tx();
        self.ecu_can_tx_total += ecu_tx.len() as u64;
        if !donor_only {
            self.tcu_can_tx_total += tcu_tx.len() as u64;
        }

        // The built-in tester's flow-control / consecutive frames go out one exchange (1 ms, or STmin windows)
        // after the frame that requested them, paced at most 1 frame per window for both ECU and TCU so
        // MultiCAN message objects never overwrite unread frames (`MSGLST`).
        let mut to_ecu: Vec<CanFrame> = Vec::new();
        let mut to_tcu: Vec<CanFrame> = Vec::new();

        if self.uds_ecu_cooldown > 0 {
            self.uds_ecu_cooldown -= 1;
        } else if let Some(mut f) = self.uds_fc_next.pop_front() {
            f.timestamp_us = now_us;
            self.broadcast_slcan(&f, None);
            if f.arb_id == OBD_FUNCTIONAL_ID && !donor_only {
                to_tcu.push(f);
            }
            to_ecu.push(f);
            if !self.uds_fc_next.is_empty() {
                self.uds_ecu_cooldown = self.uds_ecu.st_min_windows().saturating_sub(1);
            }
        }

        if self.uds_tcu_cooldown > 0 {
            self.uds_tcu_cooldown -= 1;
        } else if let Some(mut f) = self.uds_tcu_fc_next.pop_front() {
            f.timestamp_us = now_us;
            self.broadcast_slcan(&f, None);
            if f.arb_id == OBD_FUNCTIONAL_ID {
                to_ecu.push(f);
            }
            if !donor_only {
                to_tcu.push(f);
            }
            if !self.uds_tcu_fc_next.is_empty() {
                self.uds_tcu_cooldown = self.uds_tcu.st_min_windows().saturating_sub(1);
            }
        }

        // External SLCAN-over-TCP client traffic: paced at most 1 frame per 1 ms exchange window into BOTH
        // ECU and TCU so external flashers/loggers never overflow MultiCAN RX message objects.
        if let Some(ext) = self.poll_one_slcan_rx(now_us) {
            self.log("EXT", &ext);
            to_ecu.push(ext);
            if !donor_only {
                to_tcu.push(ext);
            }
        }

        for f in &ecu_tx {
            let mut stamped = *f;
            if stamped.timestamp_us == 0 {
                stamped.timestamp_us = now_us;
            }
            self.log("ECU", &stamped);
            self.broadcast_slcan(&stamped, None);
            to_tcu.push(stamped);
            if f.arb_id == 0x010 && f.dlc == 8 {
                self.immo.ecu_challenges += 1;
                let mut ans = can(0x011, 8, self.cluster_ecu.answer_challenge(&f.data));
                ans.timestamp_us = now_us;
                self.broadcast_slcan(&ans, None);
                to_ecu.push(ans);
                self.immo.ecu_answers += 1;
            }
            self.uds_fc_next.extend(self.uds_ecu.on_frame(f));
        }
        if !self.uds_ecu.status.pending && self.uds_ecu.status.dtc_sae.len() != self.uds_ecu.status.dtcs.len() {
            let tbl = crate::studio::tune_check::ecu_errors(self);
            self.uds_ecu.status.dtc_sae = self
                .uds_ecu
                .status
                .dtcs
                .iter()
                .map(|d| {
                    let cus = d.get(..6).and_then(|x| u32::from_str_radix(x, 16).ok()).map(|x| (x & 0xFFFF) as u16);
                    match cus.and_then(|c| tbl.iter().find(|e| e.2 == c)) {
                        Some((nm, s, _)) => format!("{}{:04X} ({nm})", ["P", "C", "B", "U"][(s >> 14) as usize], s & 0x3FFF),
                        None => "?".into(),
                    }
                })
                .collect();
        }
        for f in &tcu_tx {
            if donor_only && f.arb_id == 0x7E9 {
                continue;
            }
            let mut stamped = *f;
            if stamped.timestamp_us == 0 {
                stamped.timestamp_us = now_us;
            }
            if !donor_only {
                self.log("TCU", &stamped);
                self.broadcast_slcan(&stamped, None);
            }
            to_ecu.push(stamped);
            self.chassis.on_tcu_frame(f.arb_id, &f.data);
            self.tcu_chassis.on_tcu_frame(f.arb_id, &f.data);
            if f.arb_id == 0x012 && f.dlc == 8 {
                if !donor_only {
                    self.immo.tcu_challenges += 1;
                }
                let partner = if donor_only { self.cell_cluster_tcu.as_ref() } else { self.cluster_tcu.as_ref() };
                if let Some(p) = partner {
                    let mut ans = can(0x013, 8, p.answer_challenge(&f.data));
                    ans.timestamp_us = now_us;
                    if !donor_only {
                        self.broadcast_slcan(&ans, None);
                        self.immo.tcu_answers += 1;
                    }
                    to_tcu.push(ans);
                }
            }
            if !donor_only {
                self.uds_tcu_fc_next.extend(self.uds_tcu.on_frame(f));
                if !self.uds_tcu.status.pending && self.uds_tcu.status.dtc_sae.len() != self.uds_tcu.status.dtcs.len() {
                    self.uds_tcu.status.dtc_sae = if self.tcu_dtc_table.is_empty() {
                        dq200_dtc_sae(&self.root, &self.uds_tcu.status.dtcs)
                    } else {
                        dq250_dtc_sae(&self.tcu_dtc_table, &self.uds_tcu.status.dtcs)
                    };
                }
            }
        }
        if let Some(srv) = self.slcan.as_mut() {
            srv.flush();
        }
        for f in &to_ecu {
            self.board.inject_can_rx(f);
        }
        to_tcu
    }

    /// Chassis CAN frames of this slice into the ECU (and the TCU if it is not on its worker); returns
    /// the TCU's frames for its worker.
    ///
    /// On an engine test cell ([`BenchMode::EngineDyno`] / [`BenchMode::AcDyno`], gearbox physically removed) once the
    /// engine runs (`rpm ≥ 550`, not cranking), the testbed restbus (`self.chassis`) feeds the engine ECU synchronous
    /// 4th-gear wheel and vehicle speed (`v = ω_eng / |i_4| · r_tyre`), as an AVL/Horiba MQB test-cell restbus does so
    /// SIMOS18.1's stopped-vehicle rev limiter (`LV_N_MAX_VST`: `C_N_MAX_DCT = 3808 rpm` below `C_VS_MIN_N_MAX_DCT =
    /// 3 km/h`, FR ENSL 32.3.2.2.1.2.13, p. 8244–8246) and MT drive-off clutch protection (`TQ_POW_MAX_DROF` below
    /// `35 km/h`, FR TQSP 72.17.2.1.3.1, p. 25777) release naturally. Meanwhile `self.tcu_chassis` feeds any powered TCU
    /// its true uncoupled state (`0 km/h` in `P`) so the gearbox never sees a wheel-vs-shaft speed mismatch.
    fn chassis_frames(&mut self) -> Vec<CanFrame> {
        let t = self.plant.telemetry();
        let d = &self.plant.driver;
        // The ESP measures wheel speeds only; its reference speed is taken here from the non-driven wheels (FWD) or
        // the slowest wheel (AWD) — on the road ≈ the body speed, on a chassis dyno what the wheels on the rollers do
        // (UNVERIFIED: the ESP's own estimator is not emulated).
        let w_phys = t.wheel_kmh.map(f64::abs);
        let awd = self.plant.drivetrain_profile.awd;
        let v_phys = if awd { w_phys.iter().copied().fold(f64::MAX, f64::min) } else { 0.5 * (w_phys[2] + w_phys[3]) };
        let backward_phys = t.vehicle_kmh < -0.05 || t.wheel_kmh_driven < -0.05;
        let on_engine_dyno = matches!(self.plant.bench, BenchMode::EngineDyno { .. } | BenchMode::AcDyno { .. });
        let dyno_coupled = on_engine_dyno && t.rpm >= 550.0 && !d.start_button && t.starter_current_a <= 0.0;
        let (w_ecu, v_ecu, backward_ecu) = if dyno_coupled {
            let ref_gear = 4.min(self.plant.drivetrain_profile.gears.max(1)) as u8;
            let i_ref = self.plant.drivetrain_profile.total_ratio(ref_gear).abs().max(1.0);
            let r_tyre = self.plant.drivetrain_profile.tyre.radius_m.max(0.2);
            let v_cell = (t.rpm * std::f64::consts::PI / 30.0 / i_ref) * r_tyre * 3.6;
            ([v_cell; 4], v_cell, false)
        } else {
            (w_phys, v_phys, backward_phys)
        };
        // WH_SensorPos_roh codes as the DQ200_G2 firmware decodes them (`ChassisInputs::selector_code`).
        let selector_code = match d.selector {
            Selector::P => 8,
            Selector::R => 7,
            Selector::N => 6,
            Selector::D => 5,
            Selector::S => 12,
            Selector::M => 10,
        };
        let (alternator_w, alternator_util_pct) = self.plant.alternator_report();
        let ci_ecu = ChassisInputs {
            wheel_kmh: w_ecu,
            vehicle_kmh: v_ecu,
            backward: backward_ecu,
            accel_ms2: t.accel_ms2,
            brake_pressure_bar: t.brake_bar,
            brake_pedal_pressed: d.brake_pct > 2.0,
            selector_code: if on_engine_dyno { 8 } else { selector_code },
            drive_profile: self.drive_profile,
            terminal_15: d.ignition_on,
            terminal_50_request: d.start_button,
            engine_running: t.rpm > 400.0,
            alternator_w,
            alternator_util_pct,
            awd,
            ambient_c: t.ambient_c,
            tyre_radius_m: self.plant.drivetrain_profile.tyre.radius_m,
            odometer_km: self.bench_build.active.mileage.engine_km,
            soak_s: self.eot_clock_s,
        };
        let ci_tcu = ChassisInputs {
            wheel_kmh: w_phys,
            vehicle_kmh: v_phys,
            backward: backward_phys,
            selector_code: if on_engine_dyno { 8 } else { selector_code },
            ..ci_ecu
        };
        let now_us = (self.plant.sim_time_s * 1e6) as u64;
        let to_can = |(id, dlc, data)| {
            let mut f = can(id, dlc, data);
            f.timestamp_us = now_us;
            f
        };
        let ecu_frames: Vec<CanFrame> = self.chassis.encode(&ci_ecu).into_iter().map(to_can).collect();
        let need_tcu_bus = self.tcu.is_some() || self.tcu_worker.lent.is_some() || on_engine_dyno;
        let tcu_frames: Vec<CanFrame> = if need_tcu_bus {
            self.tcu_chassis.encode(&ci_tcu).into_iter().map(to_can).collect()
        } else {
            Vec::new()
        };
        for f in &ecu_frames {
            self.broadcast_slcan(f, None);
            self.board.inject_can_rx(f);
        }
        if let Some(tcu) = self.tcu.as_mut() {
            for f in &tcu_frames {
                tcu.inject_can_rx(f);
            }
        } else if on_engine_dyno {
            if let Some(tcu) = self.cell_tcu.as_mut() {
                for f in &tcu_frames {
                    tcu.inject_can_rx(f);
                }
            }
        }
        tcu_frames
    }

    fn reset_cell_tcu(&mut self) {
        self.cell_tcu = None;
        self.cell_cluster_tcu = None;
        self.cell_tcu_unavailable = false;
    }

    fn ensure_cell_tcu(&mut self) {
        if self.cell_tcu.is_some() || self.cell_tcu_unavailable {
            return;
        }
        let mut cell_plant = VirtualPlant::new(
            EngineProfile::ea888_18t_cjsa_is12(),
            DrivetrainProfile::dq200_fwd(),
            self.plant.ambient,
        );
        cell_plant.driveline.case_c = 40.0;
        cell_plant.driveline.mech.oil_c = 40.0;
        let (t, _err, partner) = Self::boot_dq200(&self.root, &cell_plant, self.tcu_lab.as_ref());
        if let Some(b) = t {
            self.cell_tcu = Some(TcuBoard::Dq200(b));
            self.cell_cluster_tcu = partner;
        } else {
            self.cell_tcu_unavailable = true;
        }
    }

    fn update_dyno(&mut self, dt: f64) {
        if !self.dyno.running {
            return;
        }
        let t = self.plant.telemetry();
        if t.rpm < 300.0 {
            self.stop_dyno();
            return;
        }
        self.plant.driver.brake_pct = 0.0;
        self.plant.driver.pedal_pct = 100.0;
        if !self.dyno.sweeping {
            self.dyno.target_rpm = self.dyno.rpm_start;
            self.plant.bench = BenchMode::EngineDyno { target_rpm: self.dyno.target_rpm };
            self.dyno.preload_s += dt;
            if t.rpm >= self.dyno.rpm_start - 50.0 {
                self.dyno.settled_s += dt;
            }
            if self.dyno.settled_s >= DYNO_SETTLE_S || self.dyno.preload_s >= DYNO_PRELOAD_MAX_S {
                self.dyno.sweeping = true;
                self.dyno.next_sample_rpm = self.dyno.rpm_start;
            }
            return;
        }
        if t.rpm >= self.dyno.next_sample_rpm {
            self.dyno.samples.push(DynoSample {
                rpm: t.rpm,
                torque_nm: t.torque_brake_nm,
                power_ps: t.power_ps,
                map_kpa: t.map_kpa,
                put_kpa: t.put_kpa,
                lambda: t.lambda_sensor,
                iat_c: t.iat_c,
                egt_c: t.egt_c,
                turbo_rpm: t.turbo_rpm,
            });
            self.dyno.next_sample_rpm = ((t.rpm / 100.0).floor() + 1.0) * 100.0;
        }
        if self.dyno.target_rpm < self.dyno.rpm_end {
            self.dyno.target_rpm = (self.dyno.target_rpm + self.dyno.ramp_rpm_s * dt).min(self.dyno.rpm_end);
            self.plant.bench = BenchMode::EngineDyno { target_rpm: self.dyno.target_rpm };
        } else {
            self.dyno.end_hold_s += dt;
            self.plant.bench = BenchMode::EngineDyno { target_rpm: self.dyno.rpm_end };
            if t.rpm >= self.dyno.rpm_end || self.dyno.end_hold_s >= DYNO_END_HOLD_S {
                self.dyno.completed = true;
                self.stop_dyno();
            }
        }
    }

    pub(crate) fn stop_dyno(&mut self) {
        self.dyno.running = false;
        self.dyno.sweeping = false;
        self.plant.driver.pedal_pct = 0.0;
        let restored_bench = self.dyno.prev_bench.take();
        if let Some(b) = restored_bench {
            self.plant.bench = b;
        }
        if let Some(s) = self.dyno.prev_selector.take() {
            let uncoupled_bench = matches!(self.plant.bench, BenchMode::EngineDyno { .. } | BenchMode::AcDyno { .. });
            if uncoupled_bench || matches!(s, Selector::P | Selector::N) {
                self.plant.driver.selector = s;
            } else {
                // Keep P when returning from a high-RPM engine-dyno sweep to a coupled driveline mode so an unbraked
                // spinning engine is never slammed directly into D/S/M/R.
                self.plant.driver.selector = Selector::P;
            }
        }
    }

    /// Advances the whole co-simulation by `dt` seconds: 10 ms slices (chassis CAN frames, workstation,
    /// dyno program) of [`EXCHANGE_S`] lock-step windows. Window semantics — deterministic, independent
    /// of host threads and timing (docs/PERFORMANCE.md §3):
    /// 1. latch at the window start: plant telemetry → ECU and TCU sensor inputs; CAN frames bridged at
    ///    the end of the previous window (+ the slice's chassis frames) are queued into the boards;
    /// 2. in parallel: the TCU (worker thread) runs its firmware to the window end, while on this thread
    ///    the plant advances one window with the actuators decoded at the end of the previous window and
    ///    records its crank/cam edges with exact timestamps, then the ECU runs the same interval with
    ///    those edges (sensor timing exact at any speed);
    /// 3. barrier: the GPTA is brought up to the window end, ECU and TCU outputs are decoded for the plant's next
    ///    window ([`io::ACTUATOR_DELAY_S`]) and CAN is bridged (ECU ↔ TCU ↔ cluster/tester).
    pub fn step(&mut self, dt: f64) {
        self.tcu_worker.reclaim_all(&mut self.tcu, &mut self.cell_tcu, &mut self.tcu_error);
        self.poll_tcu_prep();
        self.tune_check.poll();
        if self.paused || !dt.is_finite() || dt <= 0.0 {
            return;
        }
        if self.warm_standby {
            if self.plant.driver.ignition_on || self.plant.driver.start_button || self.plant.engine.rpm() > 0.0 || self.plant.thermal.engine_c < 80.0 {
                self.warm_standby = false;
            } else {
                self.plant.soak(90.0);
                (self.plant.driveline.case_c, self.plant.driveline.mech.oil_c) = (70.0, 70.0);
                self.plant.battery.soc = 1.0;
            }
        }
        if self.plant.driver.ignition_on {
            if !self.ecu_powered {
                let _ = self.power_cycle_ecu();
                if self.plant.drivetrain_profile.has_tcu_firmware {
                    self.power_cycle_tcu();
                }
            } else if !self.ecu_woken && (self.eot_clock_s - self.ecu_boot_eot_s).abs() > 1.0 {
                // Cold-standby soak advanced `eot_clock_s` while `board` was waiting for KL15: re-boot cleanly on `ecu_nvm`
                // so the ECU sees the exact accumulated EOT without a `PicEot` vs STM step jump.
                let _ = self.boot_ecu();
            }
            self.ecu_woken = true;
        }
        self.dq250_supply();
        let tcu_active = self.plant.driver.ignition_on || self.ecu_woken || self.tcu_woken;
        let using_cell_tcu = tcu_active
            && self.tcu.is_none()
            && matches!(self.plant.bench, BenchMode::EngineDyno { .. } | BenchMode::AcDyno { .. });
        if using_cell_tcu {
            self.ensure_cell_tcu();
        }
        let wall = Instant::now();
        let cpu0 = rt::thread_cpu_s();
        let n = (dt / SLICE_S).round().max(1.0) as usize;
        let windows = (SLICE_S / EXCHANGE_S).round() as usize;
        let mut perf = CosimPerf::default();
        // The TCU (or the test-cell donor mechatronic on a standalone engine dyno) belongs to its worker for the
        // duration of the step when awake; frames for it travel with the window command.
        let which_slot = if using_cell_tcu { TcuSlot::CellDonor } else { TcuSlot::Primary };
        let tcu_slot = if using_cell_tcu { &mut self.cell_tcu } else { &mut self.tcu };
        let tcu_off = tcu_slot.as_ref().map_or(0.0, |t| t.cpu().hw().sim_time_s - self.plant.sim_time_s);
        let mut has_tcu = if tcu_active {
            if let Some(t) = tcu_slot.take() {
                match self.tcu_worker.push(TcuMsg::Begin(t)) {
                    Ok(()) => {
                        self.tcu_worker.lent = Some(which_slot);
                        true
                    }
                    Err(TcuMsg::Begin(t)) => {
                        *tcu_slot = Some(t);
                        false
                    }
                    Err(_) => false,
                }
            } else {
                false
            }
        } else {
            false
        };
        let mut to_tcu: Vec<CanFrame> = Vec::new();
        'slices: for _ in 0..n {
            let g0 = Instant::now();
            self.workstation_tick();
            self.update_dyno(SLICE_S);
            let ecu_active_slice = self.plant.driver.ignition_on || (self.ecu_woken && self.ecu_powered);
            if ecu_active_slice || has_tcu {
                to_tcu.extend(self.chassis_frames());
            }
            perf.glue_s += g0.elapsed().as_secs_f64();
            for _ in 0..windows {
                let w0 = Instant::now();
                let tel = self.plant.telemetry();
                if has_tcu {
                    let until_s = self.plant.sim_time_s + EXCHANGE_S + tcu_off;
                    let (w_tel, w_driver, coils) = if using_cell_tcu {
                        // Standalone DQ200_G2 J743 mechatronic sitting in the test-cell rack at rest in P (nominal
                        // charged accumulator 50 bar, 40 °C oil, clutches/forks at 0 — UNVERIFIED bench terminations).
                        let mut ct = tel.clone();
                        ct.gearbox_hydraulic_bar = 50.0;
                        ct.mechatronic_oil_c = 40.0;
                        ct.fork_mm = [0.0; 4];
                        ct.clutch_travel_mm = [0.0; 2];
                        ct.clutch_bar = [0.0; 2];
                        ct.shaft_rpm = [0.0; 2];
                        ct.output_shaft_rpm = [0.0; 2];
                        let mut cd = self.plant.driver.clone();
                        cd.selector = Selector::P;
                        (ct, cd, None)
                    } else {
                        (tel.clone(), self.plant.driver.clone(), crate::studio::tcu_io::coil_params(&self.plant))
                    };
                    if self
                        .tcu_worker
                        .push(TcuMsg::Window(TcuWindow {
                            tel: w_tel,
                            driver: w_driver,
                            coils,
                            rx: std::mem::take(&mut to_tcu),
                            until_s,
                        }))
                        .is_ok()
                    {
                        self.tcu_worker.in_flight += 1;
                    }
                }
                let ecu_active = self.plant.driver.ignition_on || (self.ecu_woken && self.ecu_powered);
                if ecu_active {
                    io::feed_ecu(&mut self.board, &tel, &self.plant.driver, self.plant.bench);
                    io::feed_lambda_sensors(&mut self.board, &self.plant.exhaust_sensors);
                }
                let p0 = Instant::now();
                self.plant.step(EXCHANGE_S, &self.actuators);
                let p1 = Instant::now();
                self.audio.feed(&mut self.plant, &self.actuators);
                let ecu_err = if ecu_active {
                    io::feed_crank_cam(&mut self.board, &mut self.ecu_link, &self.plant, EXCHANGE_S);
                    crate::studio::io_air::feed_knock(&mut self.board, &self.ecu_link, &mut self.plant);
                    io::feed_start_request(&mut self.board, &mut self.ecu_link, &self.plant);
                    let dt = self.ecu_link.board_time(self.plant.sim_time_s) - self.board.board_time_s();
                    let err = if dt > 0.0 { self.board.run_for(dt).err() } else { None };
                    self.board.sync_engine_io();
                    self.actuators.ecu = io::decode_ecu_outputs(&self.board, &self.ecu_link);
                    self.harness.apply(&mut self.actuators.ecu);
                    if !self.plant.driver.ignition_on && !self.board.power_stage_outputs().relays.iter().any(|&r| r) {
                        // Main relay opened at the end of Nachlauf: latch live PIC EOT before KL87 power is cut.
                        self.latch_board_eot();
                        self.ecu_powered = false;
                        self.actuators.ecu = Default::default();
                    }
                    err
                } else {
                    if !self.warm_standby {
                        self.eot_clock_s += EXCHANGE_S;
                        self.sync_board_eot();
                    }
                    self.ecu_link.t_off = Some(self.board.board_time_s() - self.plant.sim_time_s);
                    self.actuators.ecu = Default::default();
                    None
                };
                let e1 = Instant::now();
                let mut tcu_tx = Vec::new();
                if has_tcu && self.tcu_worker.in_flight > 0 {
                    if let Some(TcuReply::Window(d)) = self.tcu_worker.done_rx.recv_opt(rt_spin()) {
                        self.tcu_worker.in_flight -= 1;
                        perf.tcu_s += d.busy_s;
                        perf.tcu_idle_s += d.idle_s;
                        if using_cell_tcu {
                            self.actuators.tcu = Default::default();
                        } else {
                            self.actuators.tcu = d.out;
                            if let Some(e) = d.err {
                                self.tcu_error = Some(e);
                            }
                        }
                        tcu_tx = d.tx;
                    } else {
                        self.tcu_worker.in_flight = 0;
                        self.tcu_worker.lent = None;
                        has_tcu = false;
                        if !using_cell_tcu {
                            self.tcu_error = Some("TCU worker disconnected".into());
                        }
                        self.actuators.tcu = Default::default();
                    }
                } else {
                    self.actuators.tcu = Default::default();
                }
                let e2 = Instant::now();
                to_tcu = self.bridge_can_frames_inner(tcu_tx, using_cell_tcu);
                let w1 = Instant::now();
                perf.windows += 1;
                perf.plant_s += (p1 - p0).as_secs_f64();
                perf.ecu_s += (e1 - p1).as_secs_f64();
                perf.wait_tcu_s += (e2 - e1).as_secs_f64();
                perf.glue_s += (p0 - w0 + (w1 - e2)).as_secs_f64();
                perf.max_window_s = perf.max_window_s.max((w1 - w0).as_secs_f64());
                if let Some(e) = ecu_err {
                    self.ecu_error = Some(e);
                    self.actuators.awd = Default::default();
                    continue 'slices;
                }
            }
            // Haldex coupling has its own controller (another ECU, not emulated): not driven.
            self.actuators.awd = Default::default();
        }
        let tcu_cpu = self.tcu_worker.reclaim_all(&mut self.tcu, &mut self.cell_tcu, &mut self.tcu_error);
        if self.tcu_nvm.dirty(self.tcu.as_ref()) {
            self.tcu_nvm.keep(self.tcu.as_ref());
        }
        let t = self.plant.sim_time_s;
        if self.tcu_nvm.dir.is_some() && (t - self.ecu_dflash_t >= 10.0 || t < self.ecu_dflash_t) {
            self.ecu_dflash_t = t;
            self.save_ecu_dflash();
        }
        // frames bridged at the end of the last window reach the active TCU (or cell donor TCU) before its next window
        let rx_tcu = if using_cell_tcu { self.cell_tcu.as_mut() } else { self.tcu.as_mut() };
        if let Some(t) = rx_tcu {
            for f in &to_tcu {
                t.inject_can_rx(f);
            }
        }
        perf.main_cpu_s = rt::thread_cpu_s() - cpu0;
        perf.tcu_cpu_s = tcu_cpu.unwrap_or(0.0);
        let wall_s = wall.elapsed().as_secs_f64();
        let sim = n as f64 * SLICE_S;
        self.frame = FrameStats { sim_dt_s: sim, wall_s, realtime_factor: sim / wall_s.max(1e-6), perf, pacing: self.frame.pacing.take() };
    }

    /// Supply of the DQ250 board (HYP: battery-fed regulator woken by terminal 15 and held by the firmware; the hold
    /// output is not identified). With the ignition off the TCU runs its after-run (stores its EEPROM) and goes quiet on
    /// the bus (0D9300012F: within 1.5 s, VERIFIED-T in `dq250_prep`) — its supply is off from then on. The firmware does
    /// not resume by itself on the wake input once it has shut down (studio over HTTP, 2026-11-02: interrupts stopped, no
    /// frames, no UDS answer after ignition off → on, also when the ignition came back during the shutdown), so terminal
    /// 15 powers it up again (EEPROM kept): at the ignition-on edge when it has been silent ≥ 1 s, with the ignition on
    /// when it has been silent ≥ 5 s (its ASW starts transmitting ~3.1 s after power-up). Only a board that has
    /// transmitted since its power-up (an ASW that never starts is left alone).
    fn dq250_supply(&mut self) {
        let Some(t) = self.tcu.as_ref().and_then(TcuBoard::dq250).map(|t| t.cpu.hw().sim_time_s) else { return };
        let tx = self.tcu_can_tx_total;
        if tx != self.tcu_heard.0 {
            self.tcu_heard = (tx, t);
        }
        let ign = self.plant.driver.ignition_on;
        let rising = ign && !std::mem::replace(&mut self.tcu_kl15, ign);
        // powered up by the bench without terminal 15 (a power cycle with the ignition off): it shuts down by itself
        let unwoken = ign && !self.tcu_woken;
        self.tcu_woken |= ign;
        let silent_s = t - self.tcu_heard.1;
        if ign && tx > self.tcu_tx_boot && (unwoken || silent_s >= if rising { 1.0 } else { 5.0 }) {
            if simemu_damos::is_ru() {
                eprintln!("  ТКУ DQ250: молчит {silent_s:.1} с (выключилась), зажигание включено — питание платы подано заново (EEPROM сохранён)");
            } else {
                eprintln!("  TCU DQ250: silent for {silent_s:.1} s (powered down) with ignition on — board supply re-applied (EEPROM preserved)");
            }
            self.power_cycle_tcu();
        }
    }

    pub fn apply_step_request(&mut self, r: &StepRequest) {
        let d = &mut self.plant.driver;
        if let Some(p) = r.pedal_pct {
            d.pedal_pct = p.clamp(0.0, 100.0);
        }
        if let Some(b) = r.brake_pct {
            d.brake_pct = b.clamp(0.0, 100.0);
        }
        if let Some(s) = r.selector {
            d.selector = s;
            self.dyno.prev_selector = None;
        }
        if let Some(i) = r.ignition_on {
            d.ignition_on = i;
        }
        if let Some(s) = r.start_button {
            d.start_button = s;
        }
        if let Some(p) = r.drive_profile {
            self.drive_profile = p;
        }
        if let Some(p) = r.paused {
            self.paused = p;
        }
        if let Some(v) = r.speed {
            self.speed = v.clamp(0.01, 1.0);
        }
        if r.bench_mode.is_some() {
            self.dyno.prev_bench = None;
        }
        let has_tcu_fw = self.plant.drivetrain_profile.has_tcu_firmware;
        match r.bench_mode.as_deref() {
            Some("road") if has_tcu_fw => self.plant.bench = BenchMode::Road,
            Some("road_load_dyno") if has_tcu_fw => self.plant.bench = BenchMode::RoadLoadDyno,
            Some("engine_dyno") => {
                let target_rpm = r.dyno_target_rpm.unwrap_or(2000.0).clamp(500.0, 7500.0);
                self.plant.bench = BenchMode::EngineDyno { target_rpm };
            }
            Some("ac_dyno") => {
                let target_rpm = r.dyno_target_rpm.unwrap_or(2000.0).clamp(0.0, 7500.0);
                self.plant.bench = BenchMode::AcDyno { target_rpm };
            }
            Some("chassis_dyno") if has_tcu_fw => {
                let target_kmh = r.dyno_target_kmh.unwrap_or(0.0).clamp(0.0, 300.0);
                self.plant.bench = BenchMode::ChassisDyno { target_kmh };
            }
            _ => {}
        }
        if let (Some(dt), false) = (r.dt_sec, self.realtime) {
            self.step(dt.clamp(SLICE_S, 0.1));
        }
    }

    pub fn dyno_request(&mut self, r: &DynoRequest) {
        match r.action.as_str() {
            "start" => {
                if self.plant.engine.rpm() < 300.0 {
                    return;
                }
                if !self.dyno.running {
                    self.dyno.prev_bench = Some(self.plant.bench);
                    self.dyno.prev_selector = Some(self.plant.driver.selector);
                }
                self.plant.driver.brake_pct = 0.0;
                self.plant.driver.selector = Selector::P;
                let p = &mut self.dyno;
                p.rpm_start = r.rpm_start.unwrap_or(p.rpm_start).clamp(800.0, 7000.0);
                p.rpm_end = r.rpm_end.unwrap_or(p.rpm_end).clamp(p.rpm_start + 100.0, 7500.0);
                p.ramp_rpm_s = r.ramp_rpm_s.unwrap_or(p.ramp_rpm_s).clamp(50.0, 2000.0);
                p.target_rpm = p.rpm_start;
                p.next_sample_rpm = p.rpm_start;
                p.preload_s = 0.0;
                p.settled_s = 0.0;
                p.end_hold_s = 0.0;
                p.sweeping = false;
                p.completed = false;
                p.samples.clear();
                p.running = true;
                self.plant.bench = BenchMode::EngineDyno { target_rpm: p.target_rpm };
            }
            _ => {
                self.dyno.completed = false;
                self.stop_dyno();
            }
        }
    }

    /// A tester request or job on the vehicle bus ([`UdsRequest`]). Workshop jobs (VW UDS as the DQ250 0D9300012F
    /// implements it, docs/DQ250_IO_MAP.md §8, VERIFIED-T): `write_coding` = extended session, workshop code
    /// (`2E F198`, without it the firmware refuses the coding with NRC 0x24), `2E 0600` + coding, read-back;
    /// `basic_setting` = extended session, `31 01 <rid> 04 00 00`, status `22 0102`; `routine_status` = `22 0102` +
    /// `22 0105` (routine id); `routine_stop` = `31 02 <rid>`. The firmware decides everything (session,
    /// preconditions, what it stores).
    pub fn uds_request(&mut self, r: &UdsRequest) -> Result<(), String> {
        self.tcu_worker.reclaim_all(&mut self.tcu, &mut self.cell_tcu, &mut self.tcu_error);
        let (label, reqs): (String, Vec<Vec<u8>>) = match r.service.as_str() {
            "raw" if (1..=4095).contains(&r.data.len()) => (crate::studio::uds::hex(&r.data), vec![r.data.clone()]),
            "raw" => return Err("raw request: 1..4095 bytes".into()),
            // OBD service 04 on the functional address: the only clear the firmwares implement (`uds` module docs); every
            // OBD unit on the bus clears, like with a scan tool.
            "clear_dtc" => ("Clear DTCs (OBD 04, functional 0x7DF)".into(), vec![vec![0x04]]),
            // Mask 0x2F (failed now / this cycle, pending, confirmed, failed since clear), not 0xFF: SC8O40 answers at most
            // 63 records (255 bytes) and most are only "test not completed" (0x50) — the real faults did not fit (VERIFIED-T).
            "read_dtc" => ("ReadDTCInformation (reportDTCByStatusMask 0x2F)".into(), vec![vec![0x19, 0x02, 0x2F]]),
            "tester_present" => ("TesterPresent".into(), vec![vec![0x3E, 0x00]]),
            "read_coding" => ("ReadDataByIdentifier 0600 (coding)".into(), vec![vec![0x22, 0x06, 0x00]]),
            "write_coding" => {
                let c_val = r.value.ok_or("coding value missing")?;
                let c = u16::try_from(c_val).map_err(|_| format!("coding {c_val} out of range"))?.to_be_bytes();
                let wsc = [vec![0x2E, 0xF1, 0x98], BENCH_WORKSHOP_CODE.to_vec()].concat();
                (format!("Coding {c_val:06}"), vec![vec![0x10, 0x03], wsc, vec![0x2E, 0x06, 0x00, c[0], c[1]], vec![0x22, 0x06, 0x00]])
            }
            "basic_setting" => {
                let id_val = r.rid.ok_or("rid missing")?;
                let id = id_val.to_be_bytes();
                (format!("Basic setting 0x{id_val:04X}"), vec![vec![0x10, 0x03], vec![0x31, 0x01, id[0], id[1], 0x04, 0x00, 0x00], vec![0x22, 0x01, 0x02]])
            }
            "routine_status" => ("Basic setting status (22 0102, 0105)".into(), vec![vec![0x22, 0x01, 0x02], vec![0x22, 0x01, 0x05]]),
            "routine_stop" => {
                let id_val = r.rid.ok_or("rid missing")?;
                let id = id_val.to_be_bytes();
                (format!("Stop routine 0x{id_val:04X}"), vec![vec![0x31, 0x02, id[0], id[1]], vec![0x22, 0x01, 0x02]])
            }
            s => return Err(format!("unknown service {s}")),
        };
        let functional = r.service == "clear_dtc";
        let first = match r.target.as_str() {
            "ecu" => {
                self.uds_fc_next.clear();
                self.uds_ecu_cooldown = 0;
                self.uds_ecu.job_on(&label, reqs, functional)
            }
            "tcu" => {
                self.uds_tcu_fc_next.clear();
                self.uds_tcu_cooldown = 0;
                self.uds_tcu.job_on(&label, reqs, functional)
            }
            t => return Err(format!("unknown target {t}")),
        };
        if let Some(mut f) = first {
            f.timestamp_us = (self.plant.sim_time_s * 1e6) as u64;
            self.broadcast_slcan(&f, None);
            self.board.inject_can_rx(&f);
            if let Some(t) = self.tcu.as_mut() {
                t.inject_can_rx(&f);
            }
        }
        Ok(())
    }

    /// Writes edited calibration bytes into the image and the running ECU (like a flashing tool
    /// would, keeping checksums valid).
    pub fn write_map(&mut self, id: &str, values: &[Vec<f64>]) -> Result<(), String> {
        let p = self.damos.get_by_id(id).cloned().ok_or_else(|| format!("map {id} not found"))?;
        self.damos.write_matrix(&p, &mut self.current_flash, values)?;
        self.sync_cal_to_board()
    }

    /// Recomputes CAL ECM3 sum + CAL block CRC (as a flashing tool does) and loads the CAL block
    /// into the running ECU. Only the CAL region is pushed live; code blocks are never touched.
    fn sync_cal_to_board(&mut self) -> Result<(), String> {
        let mut cal_only = self.stock_flash.clone();
        cal_only[0x200000..0x280000].copy_from_slice(&self.current_flash[0x200000..0x280000]);
        fix_checksums(&mut cal_only)?;
        self.current_flash[0x200000..0x280000].copy_from_slice(&cal_only[0x200000..0x280000]);
        self.checksums = verify_checksums(&self.current_flash).ok();
        self.board.write_live_calibration(0, &self.current_flash[0x200000..0x280000])
    }

    fn scale_map(&mut self, id: &str, factor: f64, offset: f64) {
        let Some(p) = self.damos.get_by_id(id).cloned() else { return };
        if let Some(m) = self.damos.read_matrix(&p, &self.stock_flash, &self.stock_flash) {
            let vals: Vec<Vec<f64>> = m.stock_values.iter().map(|r| r.iter().map(|v| v * factor + offset).collect()).collect();
            let _ = self.damos.write_matrix(&p, &mut self.current_flash, &vals);
        }
    }

    /// Synchronises the ECU board's KL30 engine-off timer PIC (`PIC12LF1822`) to [`Self::eot_clock_s`] at the board's
    /// current simulation timestamp (`board_time_s`), avoiding an offset error when `board_time_s() > 0`.
    pub(crate) fn sync_board_eot(&mut self) {
        let now_ns = (self.board.board_time_s().max(0.0) * 1e9) as u64;
        self.board.power_stages_mut().eot.set_seconds(self.eot_clock_s, now_ns);
    }

    /// Latches [`Self::eot_clock_s`] from the ECU board's live `PIC12LF1822` counter (e.g. when Nachlauf finishes and
    /// opens KL87, or before a battery power-cycle after a live run) so subsequent off-time accumulation and the next
    /// boot continue monotonically from the exact timestamp the firmware last stored in DFlash.
    fn latch_board_eot(&mut self) {
        let now_ns = (self.board.board_time_s().max(0.0) * 1e9) as u64;
        self.eot_clock_s = self.board.power_stages_mut().eot.seconds(now_ns) as f64;
    }

    pub(crate) fn reset_chassis_soak(&mut self) {
        self.chassis.reset_soak();
        self.tcu_chassis.reset_soak();
    }

    /// The car as after a drive and a 5 min stop (initial physical state, the conditions of the warm scenarios): engine
    /// 90 °C, gearbox oil 70 °C, engine-off timer 300 s at the next ECU power-up (matched on CAN `Kombi_02`).
    pub fn warm_car(&mut self) {
        let p = &mut self.plant;
        p.soak(90.0);
        (p.driveline.case_c, p.driveline.mech.oil_c) = (70.0, 70.0);
        self.eot_clock_s = 300.0;
        self.warm_standby = true;
        self.sync_board_eot();
        self.reset_chassis_soak();
    }

    /// Powers the engine ECU off and on again with the current image and plant state (bench: battery disconnected
    /// and reconnected). The engine-off timer PIC keeps counting from KL30 (`eot_clock_s` at power-up). The DFlash stays
    /// as the firmware left it (coding, adaptations), like a real car's, and is saved (`Self::save_ecu_dflash`).
    pub fn power_cycle_ecu(&mut self) -> Result<(), String> {
        if self.ecu_woken && (self.eot_clock_s - self.ecu_boot_eot_s).abs() < 1e-6 {
            self.latch_board_eot();
        }
        self.ecu_nvm = self.board.hw().dflash.to_vec();
        self.save_ecu_dflash();
        self.boot_ecu()
    }

    /// Powers the ECU up with the current image and `ecu_nvm`.
    fn boot_ecu(&mut self) -> Result<(), String> {
        let mut board = Simos18Board::new(&self.current_flash, &self.ecu_nvm)?;
        board.power_stages_mut().eot.set_seconds(self.eot_clock_s, 0);
        self.ecu_link = Default::default();
        self.ecu_error = io::power_on_ecu(&mut board, &mut self.ecu_link, &self.plant, ECU_BOOT_INSNS).err();
        self.ecu_dflash_saved = board.hw().dflash.to_vec();
        self.board = board;
        self.ecu_boot_eot_s = self.eot_clock_s;
        self.can_last.retain(|&(src, _), _| src != "ECU");
        self.uds_ecu.abort_on_reset();
        self.uds_fc_next.clear();
        self.uds_ecu_cooldown = 0;
        self.ecu_can_tx_total = 0;
        self.immo.ecu_challenges = 0;
        self.immo.ecu_answers = 0;
        self.ecu_woken = self.plant.driver.ignition_on;
        self.ecu_powered = true;
        self.reset_cell_tcu();
        self.bridge_can(self.plant.sim_time_s * 1e3);
        Ok(())
    }

    /// Keeps the ECU DFlash of the loaded image in the state directory (`ecu_<image>.dflash.bin`, studio server) when
    /// the firmware changed it since its power-up or the last save; the next load of that image starts from it instead
    /// of the bench dump. Called at ECU power cycles, image changes and every 10 s of simulation.
    fn save_ecu_dflash(&mut self) {
        let d = &self.board.hw().dflash;
        if d[..] == self.ecu_dflash_saved[..] {
            return;
        }
        let Some(p) = self.tcu_nvm.ecu_file(&self.image_file) else { return };
        let fresh = !p.exists();
        match p.parent().map_or(Ok(()), std::fs::create_dir_all).and_then(|_| std::fs::write(&p, &d[..])) {
            Ok(()) if fresh => {
                if simemu_damos::is_ru() {
                    eprintln!("  ЭБУ: DFlash, изменённый прошивкой, сохранён в {} (следующие загрузки образа — с него)", p.display());
                } else {
                    eprintln!("  ECU: firmware-modified DFlash saved to {} (subsequent loads of this image will start from it)", p.display());
                }
            }
            Ok(()) => {}
            Err(e) => {
                if simemu_damos::is_ru() {
                    return eprintln!("  ЭБУ: DFlash не сохранён в {}: {e}", p.display());
                } else {
                    return eprintln!("  ECU: failed to save DFlash to {}: {e}", p.display());
                }
            }
        }
        self.ecu_dflash_saved = d.to_vec();
    }

    /// Immobiliser binding for the ECU image `flash` ([`immo_lab::ecu_binding`]) on the DFlash the image left in an
    /// earlier session (`file`'s state, [`Self::save_ecu_dflash`]) or else the bench dump. Returns `(notes, tcu_rebind)` so
    /// the caller can update [`Self::image_file`] and boot the ECU before rebooting the TCU.
    fn bind_immo(&mut self, flash: &[u8], file: &str) -> (Vec<String>, bool) {
        // dump mode wants the dump's own credentials: the bench dump as it is
        let saved = self.tcu_nvm.ecu_file(file).filter(|_| self.immo_lab).and_then(|p| std::fs::read(&p).ok().map(|d| (p, d)));
        let b = immo_lab::ecu_binding(self.immo_lab, flash, saved.as_ref().map_or(&self.ecu_eeprom, |s| &s.1));
        eprintln!("{}", b.notes.join("\n  "));
        self.ecu_nvm = b.nvm;
        self.immo.ecu_key_known = b.cluster.is_some();
        self.cluster_ecu = b.cluster.unwrap_or_else(no_cluster_key);
        self.immo.binding = b.summary;
        self.immo.lab = self.immo_lab;
        let rebind = self.tcu_lab != b.tcu;
        self.tcu_lab = b.tcu;
        if rebind {
            self.reset_cell_tcu();
        }
        let origin = match (&saved, simemu_damos::is_ru()) {
            (Some((p, _)), true) => format!("DFlash ЭБУ — как его оставила прошивка в прошлых сессиях studio ({}; кодирование, адаптации), не дамп стенда; удалить файл — снова дамп", p.display()),
            (Some((p, _)), false) => format!("ECU DFlash — persisted from previous Studio sessions ({}; coding, adaptations) rather than bench dump; delete file to reset to bench dump", p.display()),
            (None, true) => "DFlash ЭБУ — дамп стенда".to_string(),
            (None, false) => "ECU DFlash — bench dump".to_string(),
        };
        eprintln!("  {origin}");
        (b.notes.into_iter().chain([origin]).collect(), rebind)
    }

    /// Keeps the TCU's non-volatile memory in `dir` from now on (studio server); a DQ250 is powered up again with the
    /// EEPROM saved there by an earlier session, if any.
    pub fn set_nvm_dir(&mut self, dir: PathBuf) {
        let _ = self.attach_nvm_dir(dir, false);
    }

    /// Attaches `dir` as the persistent NVM directory; when `reboot_if_unsaved` is true (Studio server startup after
    /// [`Self::warm_car`]), units without a saved state file also boot once from clean NVM so both ECU and TCU start on
    /// the pre-warmed plant without writing virgin DQ250 EEPROMs or incomplete cold-boot DFlash markers.
    pub(crate) fn attach_nvm_dir(&mut self, dir: PathBuf, reboot_if_unsaved: bool) -> Result<(), String> {
        self.tcu_nvm.dir = Some(dir);
        let id = self.tcu.as_ref().and_then(TcuBoard::nvm).map(|(id, _, _)| id.to_string());
        let has_saved_tcu = id.as_ref().is_some_and(|id| self.tcu_nvm.file(id).is_some_and(|p| p.exists()));
        let reboot_tcu = has_saved_tcu || (reboot_if_unsaved && self.tcu.is_some());
        if reboot_tcu {
            self.detach_tcu();
        }
        // the ECU image's DFlash from an earlier session, if saved there
        let mut ecu_reloaded = false;
        if self.tcu_nvm.ecu_file(&self.image_file).is_some_and(|p| p.exists()) {
            let f = self.image_file.clone();
            match self.switch_image(&f) {
                Ok(()) => ecu_reloaded = true,
                Err(e) => {
                    if simemu_damos::is_ru() {
                        eprintln!("  ЭБУ: сохранённый DFlash не загружен: {e}");
                    } else {
                        eprintln!("  ECU: failed to load saved DFlash: {e}");
                    }
                }
            }
        }
        if !ecu_reloaded && reboot_if_unsaved {
            self.boot_ecu()?;
        }
        if let Some(id) = id.filter(|_| has_saved_tcu) {
            self.tcu_nvm.mem.remove(&id); // the saved state, not this session's fresh part
            self.power_cycle_tcu_fresh();
        } else if reboot_tcu {
            self.power_cycle_tcu_fresh();
        } else {
            self.tcu_booted(); // the running TCU's EEPROM: nothing saved for it there
        }
        Ok(())
    }

    /// Power-up from the state file (the running board's EEPROM is not kept).
    pub(crate) fn power_cycle_tcu_fresh(&mut self) {
        self.tcu_worker.reclaim_all(&mut self.tcu, &mut self.cell_tcu, &mut self.tcu_error);
        self.reset_cell_tcu();
        self.tcu_nvm.saved_written = 0;
        self.can_last.retain(|&(src, _), _| src != "TCU");
        self.uds_tcu.abort_on_reset();
        self.uds_tcu_fc_next.clear();
        self.uds_tcu_cooldown = 0;
        self.tcu_can_tx_total = 0;
        self.immo.tcu_challenges = 0;
        self.immo.tcu_answers = 0;
        if !self.plant.drivetrain_profile.has_tcu_firmware {
            self.tcu = None;
            self.tcu_error = Some(crate::studio::bench_build::no_tcu_msg(&self.plant));
            self.cluster_tcu = None;
            self.tcu_dtc_table.clear();
            (self.tcu_tx_boot, self.tcu_heard, self.tcu_kl15, self.tcu_woken) = (0, (0, 0.0), false, false);
            self.tcu_booted();
            return;
        }
        let image = self.bench_build.active.tcu_image.clone();
        let (tcu, err, partner) = Self::boot_tcu(&self.root, &self.plant, image.as_deref(), self.tcu_lab.as_ref(), &self.tcu_nvm);
        self.tcu_dtc_table = tcu.as_ref().and_then(TcuBoard::dq250).map(|t| t.dtcs.clone()).unwrap_or_default();
        self.tcu = tcu;
        self.tcu_error = err;
        self.cluster_tcu = partner;
        let t0 = self.tcu.as_ref().map_or(0.0, |t| t.cpu().hw().sim_time_s);
        let ign = self.plant.driver.ignition_on;
        (self.tcu_tx_boot, self.tcu_heard, self.tcu_kl15, self.tcu_woken) = (self.tcu_can_tx_total, (self.tcu_can_tx_total, t0), ign, ign);
        self.tcu_booted();
    }

    /// Reclaims the TCU from its worker thread, saves any firmware-written EEPROM state into [`Self::tcu_nvm`], and
    /// detaches [`Self::tcu`] while clearing stale TCU bus/UDS state.
    pub(crate) fn detach_tcu(&mut self) {
        self.tcu_worker.reclaim_all(&mut self.tcu, &mut self.cell_tcu, &mut self.tcu_error);
        self.reset_cell_tcu();
        self.tcu_nvm.keep(self.tcu.as_ref());
        self.tcu = None;
        self.can_last.retain(|&(src, _), _| src != "TCU");
        self.uds_tcu.abort_on_reset();
        self.uds_tcu_fc_next.clear();
        self.uds_tcu_cooldown = 0;
        self.tcu_can_tx_total = 0;
        self.immo.tcu_challenges = 0;
        self.immo.tcu_answers = 0;
    }

    /// The TCU's EEPROM (DQ250) is kept: the firmware finds what it wrote before ([`TcuNvm`]).
    pub fn power_cycle_tcu(&mut self) {
        self.detach_tcu();
        self.power_cycle_tcu_fresh();
    }

    /// Loads another 4 MB PFlash image (with its DAMOS) and power-cycles the ECU. Images read
    /// without the supplier bootloader (empty SBOOT) get SBOOT from the factory dump of the
    /// same hardware — SBOOT is never part of a software update.
    pub fn load_image(&mut self, file: &str) -> Result<(), String> {
        self.save_ecu_dflash(); // the outgoing image's
        self.switch_image(file)
    }

    fn resolve_image_info(&self, file: &str) -> Result<ImageInfo, String> {
        if let Some(info) = self.images.iter().find(|i| i.file == file) {
            return Ok(info.clone());
        }
        let target_path = paths::resolve_firmware_or_err(&self.root, file)?;
        let target_canon = std::fs::canonicalize(&target_path).unwrap_or_else(|_| target_path.clone());
        if let Some(info) = self.images.iter().find(|i| {
            paths::resolve_firmware(&self.root, &i.file)
                .map(|p| std::fs::canonicalize(&p).unwrap_or(p) == target_canon)
                .unwrap_or(false)
        }) {
            return Ok(info.clone());
        }
        let cal_head = paths::peek_ecu_cal_header(&target_path, 0x10)
            .ok_or_else(|| format!("{file}: invalid SIMOS18.1 image (expected 4 MB .bin or VAG .frf/.odx container)"))?;
        let code = String::from_utf8_lossy(cal_head.get(2..8).unwrap_or(&[])).trim().to_string();
        Ok(ImageInfo {
            damos: damos_for(&self.root, &code),
            box_code: code,
            file: file.to_string(),
        })
    }

    /// [`Self::load_image`] without saving the running ECU's DFlash first.
    fn switch_image(&mut self, file: &str) -> Result<(), String> {
        let info = self.resolve_image_info(file)?;
        let damos_file = info.damos.clone().ok_or_else(|| format!("no DAMOS definition for {} in definitions", info.box_code))?;
        let mut flash = paths::read_firmware(&self.root, &info.file)?;
        if flash[..0x1_C000].iter().all(|&b| b == 0 || b == 0xFF) {
            let (_, base) = paths::read_sboot_donor(&self.root)?;
            flash[..0x1_C000].copy_from_slice(&base[..0x1_C000]);
        }
        // Writes a real bootloader/flashing tool leaves after programming (block-valid markers,
        // ECM3 CAL sum) — never code bytes. Logged and shown in the UI.
        let mut notes = simemu_damos::apply_post_programming(&mut flash)?;
        let completed = !notes.is_empty();
        if completed {
            notes.insert(
                0,
                simemu_damos::tr(
                    "Image completed after programming (source file unchanged):",
                    "Образ дозавершён после программирования (исходный файл не изменён):",
                )
                .into(),
            );
        }
        let mut damos = DamosDatabase::load_from_json(&self.root.join(&damos_file))?;
        if info.box_code == "SC8O30" {
            let _ = damos.merge_from_json(&self.root.join(O30_SWITCH_DEF));
        }
        let (immo_notes, rebind_tcu) = self.bind_immo(&flash, &info.file);
        notes.extend(immo_notes);
        self.damos = damos;
        self.stock_flash = flash.clone();
        self.current_flash = flash;
        let immo_tag = self
            .tcu_lab
            .as_ref()
            .map(|(k, _)| {
                if simemu_damos::is_ru() {
                    format!(" · иммо: лаб. привязка {}", k.id)
                } else {
                    format!(" · immo: lab bound {}", k.id)
                }
            })
            .unwrap_or_default();
        let completed_tag = if completed {
            simemu_damos::tr(" ⚑ completed after programming", " ⚑ дозавершён после программирования")
        } else {
            ""
        };
        self.image_label = format!("{} ({}){completed_tag}{immo_tag}", info.box_code, info.file);
        self.image_notes = notes;
        self.image_file = info.file;
        self.tune_check.default_reference = crate::studio::tune_check::default_reference(&self.root, &self.image_file);
        self.checksums = verify_checksums(&self.current_flash).ok();
        self.active_preset = "stock".into();
        self.boot_ecu()?;
        if rebind_tcu && self.tcu.is_some() {
            self.power_cycle_tcu();
        }
        Ok(())
    }

    pub fn apply_preset(&mut self, r: &PresetRequest) -> Result<(), String> {
        match r.action.as_str() {
            "config" => {
                let mut profile_changed = false;
                if let Some(s) = &r.engine_slug {
                    self.plant.engine_profile = EngineProfile::from_slug(s).ok_or("unknown engine")?;
                    self.bench_build.legacy = Some(format!("engine profile {s}"));
                    profile_changed = true;
                }
                if let Some(s) = &r.drivetrain_slug {
                    self.plant.drivetrain_profile = DrivetrainProfile::from_slug(s).ok_or("unknown drivetrain")?;
                    self.bench_build.legacy = Some(format!("drivetrain profile {s}"));
                    profile_changed = true;
                }
                if let Some(v) = r.fuel_ron {
                    self.plant.fuel_ron = v.clamp(80.0, 110.0);
                }
                if let Some(v) = r.ambient_temp_c {
                    self.plant.ambient.temp_c = v.clamp(-40.0, 50.0);
                }
                if let Some(v) = r.altitude_m {
                    self.plant.ambient.altitude_m = v.clamp(-400.0, 5000.0);
                }
                if let Some(v) = r.humidity_pct {
                    self.plant.ambient.humidity_pct = v.clamp(0.0, 100.0);
                }
                if profile_changed {
                    self.bench_build.staged = None;
                    return self.start_bench_mode(false, true, true);
                }
                return Ok(());
            }
            // Bench start (crate::studio::bench_build): a staged build re-creates the plant, else it is cold-soaked
            // (12 h EOT); both ECU and TCU power up together on the prepared cold plant.
            "cold_soak" | "power_cycle" => return self.start_bench_mode(false, true, true),
            // Bench start with the car warm, as after a drive and a 5 min stop; ECU and TCU power up on the warm plant.
            "warm_soak" => return self.start_bench_mode(true, true, true),
            "load_image" => {
                let file = r.image.clone().ok_or("missing image")?;
                let prev_lab = self.tcu_lab.clone();
                let had_tcu = self.tcu.is_some();
                self.load_image(&file)?;
                if (!had_tcu || self.tcu_lab == prev_lab) && self.plant.drivetrain_profile.has_tcu_firmware {
                    self.power_cycle_tcu();
                }
                return Ok(());
            }
            // DQ250 mechatronic EEPROM: erased ("as a new mechatronic") / erased and prepared again (dq250_prep).
            "tcu_eeprom_erase" => return self.tcu_eeprom_action("erase"),
            "tcu_prepare" => return self.tcu_eeprom_action("prepare"),
            // Immobiliser: lab binding (default) or the dump's own credentials (works with the bench SC8S50 only).
            "immo_lab" | "immo_dump" => {
                self.immo_lab = r.action == "immo_lab";
                let file = self.image_file.clone();
                return self.load_image(&file);
            }
            "stock" if !is_stock_ecu_file(&self.image_file, self.bench_build.active.engine) => {
                self.load_image(stock_ecu_image(self.bench_build.active.engine))?;
                self.active_preset = "stock".into();
                return Ok(());
            }
            _ => {}
        }

        // Calibration macros: restore CAL from stock, then edit real maps.
        self.current_flash[0x200000..0x280000].copy_from_slice(&self.stock_flash[0x200000..0x280000]);
        match r.action.as_str() {
            "stock" => {}
            "stage1_dq200" => {
                self.scale_map("ip_put_sp", 1.21, 0.0);
                self.scale_map("ip_put_max_cap_h_diag", 1.18, 0.0);
                self.scale_map("ip_fac_pow_put_ctl_bas", 1.16, 0.0);
                self.scale_map("ip_tqi_ref_n_m_air", 1.14, 0.0);
                self.scale_map("ip_tqi_ref_max_mon", 1.25, 0.0);
                self.scale_map("ip_tq_max_pvs_lim", 1.26, 0.0);
                self.scale_map("ip_iga_bas_ivvt_vvl_port_h[0][0][0]", 1.0, 1.5);
                self.scale_map("ip_lamb_sp_knk_bas", 0.96, 0.0);
            }
            "stage2" => {
                self.scale_map("ip_put_sp", 1.35, 0.0);
                self.scale_map("ip_put_max_cap_h_diag", 1.35, 0.0);
                self.scale_map("ip_fac_pow_put_ctl_bas", 1.32, 0.0);
                self.scale_map("ip_tqi_ref_n_m_air", 1.28, 0.0);
                self.scale_map("ip_tqi_ref_max_mon", 1.50, 0.0);
                self.scale_map("ip_tq_max_pvs_lim", 1.58, 0.0);
                self.scale_map("c_n_tcha_ref_put_max_2", 1.08, 0.0);
                self.scale_map("ip_iga_bas_ivvt_vvl_port_h[0][0][0]", 1.0, 3.5);
                self.scale_map("ip_lamb_sp_knk_bas", 0.94, 0.0);
            }
            "cold_thermostat" => self.scale_map("data_thmmng.cote_thdctlsp_m_vw", 0.835, 0.0),
            "pops_bangs" => {
                self.scale_map("data_ptcom.ptcom_tidly_bpucutoff_c_vw", 1.0, 1.75);
                self.scale_map("ip_iga_min_bas_cbk_sel_port_h[0][0][0]", 1.0, -14.5);
            }
            "unlock_stationary_limiter" => self.scale_map("c_n_max_dct", 1.0, 2942.0),
            a => return Err(format!("unknown preset {a}")),
        }
        self.active_preset = r.action.clone();
        self.sync_cal_to_board()
    }

    fn board_status(&self) -> BoardStatus {
        let pc = self.board.read_reg(TriCoreReg::PC);
        BoardStatus {
            present: true,
            sw_id: self.image_label.clone(),
            pc: format!("0x{pc:08X}"),
            region: Simos18Board::classify_region(pc),
            stm_ms: self.board.hw().sim_time_s * 1e3,
            irq_count: self.board.hw().irq_dispatch_count,
            warm_resets: self.board.hw().sw_reset_count,
            rtos_tasks_seen: self.board.hw().rtos_task_counts.len(),
            rtos_task_runs: self.board.hw().rtos_task_counts.values().sum(),
            can_tx_total: self.ecu_can_tx_total,
            error: self.ecu_error.clone(),
        }
    }

    fn tcu_status(&self) -> BoardStatus {
        match &self.tcu {
            Some(t) => {
                let pc = t.cpu().read_reg(TriCoreReg::PC);
                let hw = t.cpu().hw();
                BoardStatus {
                    present: true,
                    sw_id: t.sw_id(),
                    pc: format!("0x{pc:08X}"),
                    region: t.region(pc),
                    stm_ms: hw.sim_time_s * 1e3,
                    irq_count: hw.irq_dispatch_count,
                    warm_resets: hw.sw_reset_count,
                    rtos_tasks_seen: hw.rtos_task_counts.len(),
                    rtos_task_runs: hw.rtos_task_counts.values().sum(),
                    can_tx_total: self.tcu_can_tx_total,
                    error: self.tcu_error.clone(),
                }
            }
            None => BoardStatus {
                present: false,
                sw_id: String::new(),
                pc: String::new(),
                region: "OFF",
                stm_ms: 0.0,
                irq_count: 0,
                warm_resets: 0,
                rtos_tasks_seen: 0,
                rtos_task_runs: 0,
                can_tx_total: 0,
                error: self.tcu_error.clone(),
            },
        }
    }

    pub fn state(&self) -> StudioState<'_> {
        StudioState {
            ecu: self.board_status(),
            tcu: self.tcu_status(),
            immo: &self.immo,
            uds_ecu: &self.uds_ecu.status,
            uds_tcu: &self.uds_tcu.status,
            can_recent: self.can_log.iter().rev().take(24).collect(),
            plant: self.plant.telemetry(),
            driver: &self.plant.driver,
            bench: self.plant.bench,
            paused: self.paused,
            frame: &self.frame,
            engine_profile: &self.plant.engine_profile,
            drivetrain_profile: &self.plant.drivetrain_profile,
            ambient: self.plant.ambient,
            fuel_ron: self.plant.fuel_ron,
            drive_profile: self.drive_profile,
            image_label: &self.image_label,
            active_preset: &self.active_preset,
            curated_maps: CURATED_TUNER_MAPS,
            dyno: &self.dyno,
            checksums: self.checksums.as_ref(),
            images: &self.images,
            image_file: &self.image_file,
            image_notes: &self.image_notes,
            workstation: self.ws_status(),
            bench_build: self.bench_build.brief(),
            gearbox_bus: self.gearbox_bus(),
            tcu_prep: self.tcu_prep_view(),
            tcu_notes: &self.tcu_notes,
            tune_check: self.tune_check.view(),
        }
    }

    /// Starts the pre-flash check of `image` against `reference` on the bench build ([`crate::studio::tune_check`]); the
    /// DQ250's EEPROM the firmware prepared (if this session has one for the running TCU) goes with it.
    pub fn start_tune_check(&mut self, image: &str, reference: Option<String>, mm_slot: Option<u8>) -> Result<(), String> {
        if !self.images.iter().any(|i| i.file == image) && !paths::has_sandboxed_firmware(&self.root, image) {
            return Err(format!("unknown image {image}"));
        }
        if let Some(r) = reference
            .as_deref()
            .filter(|r| !self.images.iter().any(|i| i.file == *r) && !paths::has_sandboxed_firmware(&self.root, r))
        {
            return Err(format!("unknown reference image {r}"));
        }
        let tcu_eeprom = self.tcu.as_ref().and_then(TcuBoard::nvm).map(|n| n.0.to_string()).filter(|id| self.tcu_nvm.has_state(id)).map(|id| {
            let mem = self.tcu_nvm.load(&id);
            (id, mem)
        });
        self.tune_check.start(crate::studio::tune_check::CheckRequest {
            root: self.root.clone(),
            build: self.bench_build.active.clone(),
            image: image.to_string(),
            reference,
            immo_lab: self.immo_lab,
            tcu_eeprom,
            phases: None,
            mm_slot,
        })
    }

    fn gearbox_bus(&self) -> GearboxBus {
        let alive = self.tcu_can_tx_total > self.tcu_tx_boot
            && self
                .tcu
                .as_ref()
                .is_some_and(|t| t.dq250().is_none() || t.cpu().hw().sim_time_s - self.tcu_heard.1 < 0.5);
        if !alive {
            return GearboxBus::default(); // the TCU is silent: its last frames would be stale
        }
        let sig = |id: u32, start: u32| self.can_last.get(&("TCU", id)).map(|f| ((u64::from_le_bytes(f.data) >> start) & 0xF) as u8);
        GearboxBus { gear: sig(0x394, 24), fahrstufe: sig(0x0AD, 42), zielgang: sig(0x0AD, 60) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader, Write};
    use std::net::TcpStream;
    use std::time::Duration;

    #[test]
    fn start_tune_check_blocks_traversal_and_tcu_worker_reclaims_lent_board() {
        let Some(root) = paths::test_workspace_root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let mut s = StudioSession::new(&root).expect("session");
        assert!(s.start_tune_check("../../etc/passwd.bin", None, None).is_err());
        assert!(s.start_tune_check(S50_FLASH, Some("../outside.bin".into()), None).is_err());
        assert!(s.start_tune_check("/abs/path.bin", None, None).is_err());

        // Simulate a lent TCU board with an in-flight window left over from an interrupted step:
        // `reclaim` must drain the window reply and restore `s.tcu` cleanly.
        if let Some(t) = s.tcu.take() {
            s.tcu_worker.push(TcuMsg::Begin(t)).ok();
            s.tcu_worker.lent = Some(TcuSlot::Primary);
            let until_s = s.plant.sim_time_s + EXCHANGE_S;
            let coils = crate::studio::tcu_io::coil_params(&s.plant);
            s.tcu_worker
                .push(TcuMsg::Window(TcuWindow {
                    tel: s.plant.telemetry(),
                    driver: s.plant.driver.clone(),
                    coils,
                    rx: Vec::new(),
                    until_s,
                }))
                .ok();
            s.tcu_worker.in_flight = 1;
            s.tcu_worker.reclaim_all(&mut s.tcu, &mut s.cell_tcu, &mut s.tcu_error);
            assert!(s.tcu.is_some(), "reclaim_all must recover the lent primary TcuBoard");
            assert!(s.tcu_worker.lent.is_none() && s.tcu_worker.in_flight == 0);
        }
        // Also verify CellDonor slot isolation on unwind recovery: a lent `cell_tcu` must return to `s.cell_tcu`, never `s.tcu`.
        if let Some(donor) = s.tcu.take() {
            s.cell_tcu = Some(donor);
            let t = s.cell_tcu.take().unwrap();
            s.tcu_worker.push(TcuMsg::Begin(t)).ok();
            s.tcu_worker.lent = Some(TcuSlot::CellDonor);
            s.tcu_worker.reclaim_all(&mut s.tcu, &mut s.cell_tcu, &mut s.tcu_error);
            assert!(s.tcu.is_none(), "donor reclaim must never populate primary s.tcu");
            assert!(s.cell_tcu.is_some(), "donor reclaim must restore s.cell_tcu");
            assert!(s.tcu_worker.lent.is_none());
        }
    }

    #[test]
    fn list_images_discovers_ecu_bins_and_frf_images_without_duplicates() {
        let Some(root) = paths::test_workspace_root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let imgs = list_images(&root);
        let names: Vec<&str> = imgs.iter().map(|i| i.file.as_str()).collect();
        assert!(
            names.contains(&S50_FLASH) || names.contains(&"FL_8V0906264H__0005.frf"),
            "expected stock 264H image in {names:?}"
        );
        if paths::has_firmware(&root, "FL_8V0906264H__0005.frf") {
            assert!(
                names.contains(&"FL_8V0906264H__0005.frf"),
                "expected FL_8V0906264H__0005.frf in {names:?}"
            );
        }
        let unique: HashSet<&str> = names.iter().copied().collect();
        assert_eq!(unique.len(), names.len(), "duplicate image entries in {names:?}");
    }

    #[test]
    fn native_frf_loads_and_executes_directly_without_stock_bins() {
        let Some(root) = paths::test_workspace_root() else {
            eprintln!("SKIP: workspace firmwares not found");
            return;
        };
        let Some(frf_src) = paths::resolve_firmware(&root, "FL_8V0906264H__0005.frf") else {
            eprintln!("SKIP: FL_8V0906264H__0005.frf not found");
            return;
        };

        // 1. Live StudioSession switches directly onto a VAG `.frf` container and steps co-simulation cleanly.
        let mut s = StudioSession::new(&root).expect("session");
        s.load_image("FL_8V0906264H__0005.frf").expect("load .frf directly");
        assert_eq!(s.image_file, "FL_8V0906264H__0005.frf");
        assert!(s.ecu_error.is_none(), "ECU boot error: {:?}", s.ecu_error);
        s.plant.driver.ignition_on = true;
        s.step(0.05);
        assert!(s.ecu_error.is_none(), "ECU runtime error: {:?}", s.ecu_error);
        assert!(s.ecu_can_tx_total > 0, "expected ECU CAN frames while executing unpacked .frf");

        // 2. Clean FRF-only directory (zero `ecu/stock/*.bin` AND zero `ecu/bench/*.bin` files!) resolves
        //    `DEFAULT_ECU_FLASH`, unpacks `FL_8V0906264H__0005.frf` + bundled `rom/sboot.bin` in memory,
        //    and boots into ASW (`0x8004..0x801F`) even on a blank 256 KB DFlash.
        let tmp = std::env::temp_dir().join(format!("simemu_frf_only_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(tmp.join("vault/firmwares/ecu/frf")).unwrap();
        std::fs::copy(&frf_src, tmp.join("vault/firmwares/ecu/frf/FL_8V0906264H__0005.frf")).unwrap();

        let mut flash = paths::read_firmware(&tmp, paths::DEFAULT_ECU_FLASH).expect("auto-unpack FRF with bundled rom/sboot.bin");
        let _ = simemu_damos::apply_post_programming(&mut flash).expect("post-programming");
        assert_eq!(flash.len(), 0x40_0000);
        let eeprom = vec![0u8; 0x4_0000];
        let mut board = Simos18Board::new(&flash, &eeprom).expect("board");
        let pc = board.run_boot_sequence(3_000_000).expect("boot");
        assert!(
            (0x8004_0000..0x8020_0000).contains(&pc) || (0x8088_0000..0x8090_0000).contains(&pc) || (0xC000_0000..0xC001_0000).contains(&pc),
            "expected ECU running ASW/PSPR from FRF-only workspace + bundled rom/sboot.bin, got PC=0x{pc:08X}"
        );
        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn live_slcan_bridge_streams_bus_and_paces_external_rx_one_per_window() {
        let Some(root) = paths::test_workspace_root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let mut s = StudioSession::new(&root).expect("session");
        let srv = SlcanServer::bind(0, false).expect("bind slcan");
        let port = srv.port();
        s.set_slcan(Some(srv));
        assert_eq!(s.slcan_port(), Some(port));

        let mut client = TcpStream::connect(("127.0.0.1", port)).expect("connect");
        client.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let mut reader = BufReader::new(client.try_clone().unwrap());

        // Handshake works even when session is paused via poll_slcan_idle()
        s.paused = true;
        client.write_all(b"C\rV\rS6\rO\r").unwrap();
        for _ in 0..5 {
            s.poll_slcan_idle();
            std::thread::sleep(Duration::from_millis(2));
        }
        let mut read_cr = || {
            let mut b = Vec::new();
            reader.read_until(b'\r', &mut b).unwrap();
            b
        };
        assert_eq!(read_cr(), b"\r");
        assert_eq!(read_cr(), b"V1013\r");
        assert_eq!(read_cr(), b"\r");
        assert_eq!(read_cr(), b"\r");

        // Unpause, turn ignition on, send 3 external UDS frames in a burst, and step one 10 ms slice (10 × 1 ms windows)
        s.paused = false;
        s.plant.driver.ignition_on = true;
        client
            .write_all(b"t7E08023E005555555555\rt7E18023E005555555555\r")
            .unwrap();
        client.flush().unwrap();
        std::thread::sleep(Duration::from_millis(5));

        s.step(SLICE_S);

        // External frames were logged as "EXT" and live MQB bus frames streamed out to the SLCAN client
        let ext_logged = s.can_log.iter().filter(|e| e.src == "EXT").count();
        assert_eq!(ext_logged, 2, "both external frames must enter the live bus");
        let first_bus_frame = String::from_utf8(read_cr()).unwrap();
        assert!(
            first_bus_frame.starts_with('t') && first_bus_frame.ends_with('\r'),
            "expected SLCAN bus frame, got {first_bus_frame:?}"
        );

        // Verify TCU multi-frame UDS consecutive frames queue into `uds_tcu_fc_next` (1 per window, not dumped at once)
        s.uds_request(&UdsRequest {
            target: "tcu".into(),
            service: "raw".into(),
            data: vec![0x2E, 0xF1, 0x98, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
            value: None,
            rid: None,
        })
        .unwrap();
        // Simulate TCU sending FlowControl (FS=0, BS=0, STmin=0) in one window
        let fc = CanFrame {
            node: 0,
            mo_index: 0,
            arb_id: 0x7E9,
            extended: false,
            dlc: 8,
            data: [0x30, 0, 0, 0x55, 0x55, 0x55, 0x55, 0x55],
            timestamp_us: 0,
        };
        let immediate_to_tcu = s.bridge_can_frames(vec![fc]);
        assert!(
            immediate_to_tcu.iter().all(|f| f.arb_id != 0x7E1),
            "CFs must wait in uds_tcu_fc_next rather than flooding to_tcu in the same window"
        );
        assert_eq!(s.uds_tcu_fc_next.len(), 2, "15-byte request has 2 CFs queued");
        let win1_to_tcu = s.bridge_can_frames(Vec::new());
        assert_eq!(
            win1_to_tcu.iter().filter(|f| f.arb_id == 0x7E1).count(),
            1,
            "at most 1 CF delivered to TCU per 1 ms window"
        );
        let win2_to_tcu = s.bridge_can_frames(Vec::new());
        assert_eq!(
            win2_to_tcu.iter().filter(|f| f.arb_id == 0x7E1).count(),
            1,
            "second CF delivered on the following 1 ms window"
        );
    }

    #[test]
    fn engine_dyno_sweeps_to_redline_with_and_without_dsg_while_road_park_keeps_limiter() {
        let Some(root) = crate::paths::test_workspace_root() else {
            eprintln!("SKIP: firmware image not found");
            return;
        };
        if !paths::has_firmware(&root, DQ200_FLASH) || !paths::has_firmware(&root, DQ200_EEPROM) {
            eprintln!("SKIP: {DQ200_FLASH} / {DQ200_EEPROM} not found");
            return;
        }
        let mut s = StudioSession::new(&root).expect("session");
        s.warm_car();
        s.plant.driver.ignition_on = true;
        s.plant.driver.selector = Selector::P;
        s.plant.bench = BenchMode::Road;

        // 1. Start engine in Road mode (Selector::P) and verify the real factory ~3808 rpm standstill rev limiter holds.
        let mut fired = false;
        for i in 0..450 {
            let t = (i + 1) as f64 * 0.01;
            fired |= s.plant.engine.rpm() > 600.0;
            s.plant.driver.brake_pct = if (0.2..2.5).contains(&t) { 60.0 } else { 0.0 };
            s.plant.driver.start_button = (0.5..3.0).contains(&t) && !fired;
            s.step(0.01);
        }
        assert!(s.plant.engine.rpm() > 600.0, "engine must start cleanly");

        s.plant.driver.pedal_pct = 100.0;
        let mut road_park_peak_rpm = 0.0_f64;
        for _ in 0..300 {
            s.step(0.01);
            road_park_peak_rpm = road_park_peak_rpm.max(s.plant.engine.rpm());
        }
        s.plant.driver.pedal_pct = 0.0;
        for _ in 0..100 {
            s.step(0.01);
        }
        eprintln!("Road mode in P peak RPM at WOT: {road_park_peak_rpm:.0} rpm");
        assert!(
            (3400.0..4150.0).contains(&road_park_peak_rpm),
            "parked car on the road in P must honour the factory ~3808 rpm stationary limiter, got {road_park_peak_rpm:.0} rpm"
        );

        // 2. Run a full WOT Dyno sweep (1500 -> 6500 rpm at 500 rpm/s) WITH DSG — even if the driver was holding D on the brake
        // in Road mode right before starting the sweep, `dyno_request` must clear brake line pressure (`0 bar`), switch to P,
        // and leave the car safely in P when returning to Road mode after 6500 rpm.
        s.plant.driver.brake_pct = 60.0;
        s.plant.driver.selector = Selector::D;
        s.step(0.05);
        assert!(s.plant.telemetry().brake_bar > 50.0);
        s.dyno_request(&DynoRequest {
            action: "start".into(),
            rpm_start: Some(1500.0),
            rpm_end: Some(6500.0),
            ramp_rpm_s: Some(500.0),
        });
        assert!(s.dyno.running, "dyno sweep must start on a running engine");
        assert_eq!(s.plant.driver.selector, Selector::P);
        for k in 0..1600 {
            if !s.dyno.running {
                break;
            }
            s.step(0.01);
            if k == 5 {
                assert_eq!(s.plant.telemetry().brake_bar, 0.0, "step_uncoupled must release brake line pressure during sweep");
            }
        }
        assert!(!s.dyno.running && s.dyno.completed, "dyno sweep must finish cleanly with completed=true");
        assert_eq!(s.plant.bench, BenchMode::Road, "previous bench mode must be restored after sweep");
        assert_eq!(s.plant.driver.selector, Selector::P, "coupled Road mode must remain safely in P after high-RPM sweep");
        let max_rpm_dsg = s.dyno.samples.iter().map(|p| p.rpm).fold(0.0_f64, f64::max);
        let peak_tq_dsg = s.dyno.samples.iter().map(|p| p.torque_nm).fold(0.0_f64, f64::max);
        let peak_ps_dsg = s.dyno.samples.iter().map(|p| p.power_ps).fold(0.0_f64, f64::max);
        eprintln!(
            "EngineDyno sweep WITH DSG (Selector::P): {} samples, first {:.0} rpm ({:.0} Nm), max {:.0} rpm, peak {:.1} Nm / {:.1} PS",
            s.dyno.samples.len(),
            s.dyno.samples.first().map_or(0.0, |x| x.rpm),
            s.dyno.samples.first().map_or(0.0, |x| x.torque_nm),
            max_rpm_dsg,
            peak_tq_dsg,
            peak_ps_dsg
        );
        assert!(max_rpm_dsg >= 6400.0, "EngineDyno sweep with DSG in P must pull past 3808 rpm to redline, got {max_rpm_dsg:.0} rpm");
        assert!(peak_tq_dsg >= 225.0, "1.8 TSI stock sweep must deliver full torque, got {peak_tq_dsg:.1} Nm");
        assert!(peak_ps_dsg >= 165.0, "1.8 TSI stock sweep must deliver full power, got {peak_ps_dsg:.1} PS");

        // 3. Verify DSG still shifts into D and accelerates cleanly on synchronised ChassisDyno rollers (FWD all 4 wheels rolling).
        s.plant.bench = BenchMode::ChassisDyno { target_kmh: 90.0 };
        s.plant.driver.brake_pct = 60.0;
        s.plant.driver.selector = Selector::D;
        for _ in 0..150 {
            s.step(0.01);
        }
        s.plant.driver.brake_pct = 0.0;
        s.plant.driver.pedal_pct = 80.0;
        for _ in 0..450 {
            s.step(0.01);
        }
        let tel_cd = s.plant.telemetry();
        eprintln!("ChassisDyno FWD in D: roller {:.1} km/h, wheels {:.1?} km/h, rpm {:.0}", tel_cd.roller_kmh, tel_cd.wheel_kmh, tel_cd.rpm);
        assert!(tel_cd.roller_kmh > 30.0, "DSG must drive the ChassisDyno rollers after an EngineDyno sweep, got {:.1} km/h", tel_cd.roller_kmh);
        assert!(
            (tel_cd.wheel_kmh[2] - tel_cd.roller_kmh).abs() < 2.0,
            "rear wheels on synchronised 4WD ChassisDyno must match roller speed ({:.1} vs {:.1} km/h)",
            tel_cd.wheel_kmh[2],
            tel_cd.roller_kmh
        );
        for target in ["ecu", "tcu"] {
            s.uds_request(&UdsRequest { target: target.into(), service: "raw".into(), data: vec![0x19, 0x02, 0x05], value: None, rid: None }).unwrap();
            for _ in 0..200 {
                s.step(0.01);
                if !(if target == "ecu" { &s.uds_ecu } else { &s.uds_tcu }).status.pending {
                    break;
                }
            }
            let st = if target == "ecu" { &s.uds_ecu.status } else { &s.uds_tcu.status };
            assert!(st.dtcs.is_empty(), "{target} must have no active/pending faults after EngineDyno + ChassisDyno, got {:?} = {:?}", st.dtcs, st.dtc_sae);
        }

        // 4. Verify a standalone EngineDyno build WITHOUT DSG (`Gearbox::Dq381`, `s2.tcu.is_none()` from boot)
        // also sweeps past 3808 rpm to redline at full stock power with zero active ECU faults.
        let cfg_no_dsg = simemu_physics::bench_config::BenchConfig {
            gearbox: simemu_physics::bench_config::Gearbox::Dq381,
            awd: false,
            ..Default::default()
        };
        let mut s2 = StudioSession::with_build(&root, crate::studio::bench_build::BuildState::from_config(cfg_no_dsg)).expect("no-dsg session");
        assert!(s2.tcu.is_none(), "DQ381 build has no TCU firmware");
        s2.warm_car();
        s2.plant.driver.ignition_on = true;
        let mut fired2 = false;
        for i in 0..450 {
            let t = (i + 1) as f64 * 0.01;
            fired2 |= s2.plant.engine.rpm() > 600.0;
            s2.plant.driver.brake_pct = if (0.2..2.5).contains(&t) { 60.0 } else { 0.0 };
            s2.plant.driver.start_button = (0.5..3.0).contains(&t) && !fired2;
            s2.step(0.01);
        }
        assert!(s2.plant.engine.rpm() > 600.0, "engine without DSG must start cleanly on EngineDyno");
        s2.dyno_request(&DynoRequest {
            action: "start".into(),
            rpm_start: Some(1500.0),
            rpm_end: Some(6500.0),
            ramp_rpm_s: Some(500.0),
        });
        for _ in 0..1600 {
            if !s2.dyno.running {
                break;
            }
            s2.step(0.01);
        }
        let max_rpm_no_dsg = s2.dyno.samples.iter().map(|p| p.rpm).fold(0.0_f64, f64::max);
        let peak_tq_no_dsg = s2.dyno.samples.iter().map(|p| p.torque_nm).fold(0.0_f64, f64::max);
        let peak_ps_no_dsg = s2.dyno.samples.iter().map(|p| p.power_ps).fold(0.0_f64, f64::max);
        s2.uds_request(&UdsRequest { target: "ecu".into(), service: "raw".into(), data: vec![0x19, 0x02, 0x05], value: None, rid: None }).unwrap();
        for _ in 0..200 {
            s2.step(0.01);
            if !s2.uds_ecu.status.pending {
                break;
            }
        }
        eprintln!(
            "EngineDyno sweep WITHOUT DSG (DQ381 build): {} samples, max {max_rpm_no_dsg:.0} rpm, peak {peak_tq_no_dsg:.1} Nm / {peak_ps_no_dsg:.1} PS, active DTCs {:?} = {:?}",
            s2.dyno.samples.len(),
            s2.uds_ecu.status.dtcs,
            s2.uds_ecu.status.dtc_sae,
        );
        assert!(max_rpm_no_dsg >= 6400.0, "EngineDyno without DSG must reach >= 6400 rpm, got {max_rpm_no_dsg:.0} rpm");
        assert!(peak_ps_no_dsg >= 165.0, "EngineDyno without DSG must deliver full power, got {peak_ps_no_dsg:.1} PS");
        assert!(s2.uds_ecu.status.dtcs.is_empty(), "ECU on EngineDyno without DSG must have zero active faults, got {:?} = {:?}", s2.uds_ecu.status.dtcs, s2.uds_ecu.status.dtc_sae);
    }
}
