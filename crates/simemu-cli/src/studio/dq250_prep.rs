//! Workshop preparation of a DQ250 mechatronic (docs/DQ250_IO_MAP.md §8).
//!
//! No EEPROM dump of a DQ250 exists: an image boots on an erased part — no coding, no learnt gear positions, like a
//! mechatronic from the parts shelf. A workshop codes it and runs its basic settings; the studio does the same,
//! automatically, when a DQ250 image powers up with no saved EEPROM state (`StudioSession::tcu_booted`). It runs a
//! second, headless co-simulation of the same car "in the workshop" (the bench build, the loaded ECU image, engine warm
//! 90 °C, gearbox 70 °C, on the road-load dyno in P with the brake held — the conditions of the verified
//! `dq250_workshop_scenario`) in which only a technician acts: ignition, start button and the session's UDS tester (bench
//! equipment): extended session, `2E F198` workshop code, `2E 0600` coding, ignition cycle (the TCU stores in its
//! after-run), then for each basic setting of the image's routine table ([`basic_setting_rids`]) `31 01 <rid> 04 00 00`,
//! `22 0102` until it leaves 0xC0 (running), `31 02 <rid>` (an attempt the firmware aborts, 0x60, is repeated up to
//! `BASIC_ATTEMPTS` times), the fault memory read (kept in the record; the application
//! does not support `14`), and a last ignition cycle. Every decision and every EEPROM byte is the firmware's; the studio writes
//! nothing into the EEPROM. Timing follows the firmware, not fixed sleeps: the after-run ends when the TCU falls silent on
//! the bus, a routine is ended once the forks stopped, a refused start (NRC 0x22) is repeated. The part it leaves is saved for that image (`TcuNvm`),
//! the live TCU powers up with it, and the record (`tcu_<id>.prep.json`) is shown at every later load.

use crate::studio::session::{StudioSession, UdsRequest};
use crate::studio::tcu_io::TcuBoard;
use crate::studio::uds::{routine_status_text, UdsStatus};
use serde::{Deserialize, Serialize};
use simemu_physics::bench_config::BenchConfig;
use simemu_physics::{BenchMode, Selector};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// Default workshop coding for 0D9300012F: "000024".
pub const DEFAULT_CODING_0D9300012F: u32 = 24;
/// Expected simulated duration of a preparation (progress bar only), s.
const EXPECTED_S: f64 = 110.0;
/// After ignition off the TCU's after-run stores what it learnt and then the TCU stops transmitting (bus released); its
/// supply goes once it has been silent this long, s. 0D9300012F: 504 / 394 B written within 1 s, silent from 1.5 s after
/// ignition off (VERIFIED-T, this preparation; `dq250_workshop_scenario` waits a fixed 15 s).
const AFTERRUN_QUIET_S: f64 = 1.0;
/// Longest after-run waited for, s.
const AFTERRUN_MAX_S: f64 = 15.0;
/// Attempts per basic setting. The 0x0380 clutch bite-point step waits for a steady engine torque on CAN (0D9300012F:
/// flag 0xD00043B1 of 0x80071128, CAL 0x80038384+2..+8, ±30 for 250 cycles) and aborts with code 0x0E when it does not
/// come (the 0D9300011R read on K2 after 20 s, VERIFIED-T); on the bench the idle torque swings for ≈ 1 s after each gear
/// engagement (docs/DQ250_FIRMWARE_SPEC.md §11.6), so an attempt can end like that. A technician starts it again.
const BASIC_ATTEMPTS: u32 = 3;

/// Basic settings (routines with a precondition check) of a DQ250 image, read from its `31 01` handler: the routine id
/// is compared in a chain of `mov d0, #rid; jeq d15, d0, <branch>`; a basic setting's branch calls its precondition check
/// first and refuses (NRC 0x22) unless it returns 1 (`call …; jne d2, #1, …`), the adaptation resets (0x0317,
/// 0x0384..0x03B0) start without one. The routine-start chain is the one with most such branches. 0D9300012F (chain
/// 0x8005B21E) and 0D9300011R (0x80061608): 0x0380 and 0x04D4 (VERIFIED-S). Ascending order is the order the firmware
/// needs here: 0x04D4 starts only with every gear position adapted, which is what 0x0380 does (docs/DQ250_IO_MAP.md §8).
pub fn basic_setting_rids(flash: &[u8]) -> Vec<u16> {
    let w = |a: usize| flash.get(a..a + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]));
    let mut best: Vec<u16> = Vec::new();
    let mut i = 0;
    while i + 8 <= flash.len() {
        let mut chain = Vec::new();
        let mut j = i;
        // mov d0, #imm16 (RLC 0x3B, c = d0) + jeq d15, d0, disp15 (BRR 0x5F, op2 0)
        while let (Some(m), Some(b)) = (w(j), w(j + 4)) {
            if m & 0xF000_00FF != 0x3B || b & 0x8000_FFFF != 0x0F5F {
                break;
            }
            let disp = ((b >> 16) & 0x7FFF) as i64 - if b & 0x4000_0000 != 0 { 0x8000 } else { 0 };
            chain.push((((m >> 12) & 0xFFFF) as u16, (j as i64 + 4 + 2 * disp) as usize));
            j += 8;
        }
        if chain.len() < 4 {
            i += 2;
            continue;
        }
        // branch = call disp24 (B 0x6D) + jne d2, #1, disp15 (BRC 0xDF, op2 1)
        let mut basic: Vec<u16> = chain.iter().filter(|(_, t)| w(*t).is_some_and(|c| c & 0xFF == 0x6D) && w(t + 4).is_some_and(|x| x & 0x8000_FFFF == 0x8000_12DF)).map(|c| c.0).collect();
        if basic.len() > best.len() {
            basic.sort_unstable();
            best = basic;
        }
        i = j;
    }
    best
}

/// A DQ250 image the bench build can choose ([`dq250_images`]).
#[derive(Debug, Clone, Serialize)]
pub struct Dq250Image {
    /// File name as the build stores it (resolved via `vault/firmwares/tcu/dq250/`, `$SIMEMU_IMAGE_DIR`, or root).
    pub file: String,
    /// Identification ("49300D9300012F_4930_ODKW").
    pub asw_id: String,
    /// Coding used when the build sets none; `None` = the firmware's own default (its CAL value, read on the erased part).
    pub default_coding: Option<u32>,
}

/// DQ250 images in `vault/firmwares/tcu/dq250/` (plus `$SIMEMU_IMAGE_DIR` and workspace root),
/// identified by size and OEM ASW ID (`0D9300011R`, `0D9300012F`, or any `0D9` `.bin` placed there),
/// ordered with unmodified factory stock images (valid CBOOT block CRCs) first.
pub fn dq250_images(root: &Path) -> Vec<Dq250Image> {
    use simemu_tc1791::tcu_dq250::{Dq250ImageInfo, PFLASH_READ_SIZE, PFLASH_SIZE};
    let mut v: Vec<(bool, Dq250Image)> = Vec::new();
    for sub in ["vault/firmwares/tcu/dq250", "vault/firmwares/tcu", "firmwares/tcu/dq250", "tcu/dq250", ""] {
        for (name, path) in crate::paths::list_bins_in_subdir(root, sub) {
            if !std::fs::metadata(&path).is_ok_and(|m| [PFLASH_SIZE, PFLASH_READ_SIZE].contains(&(m.len() as usize))) {
                continue;
            }
            let Ok(mut flash) = std::fs::read(&path) else { continue };
            let asw_id = Dq250ImageInfo::from_bytes(&flash).asw_id;
            if !asw_id.is_empty() && !v.iter().any(|(_, x)| x.file == name) {
                let is_stock_crc = simemu_damos::apply_post_programming_dq250(&mut flash).is_ok_and(|notes| notes.is_empty());
                v.push((is_stock_crc, Dq250Image { default_coding: default_coding(&asw_id), file: name, asw_id }));
            }
        }
    }
    v.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| a.1.asw_id.cmp(&b.1.asw_id)).then_with(|| a.1.file.cmp(&b.1.file)));
    v.into_iter().map(|(_, x)| x).collect()
}

/// Coding when the build sets none: 000024 for 0D9300012F; `None` (the firmware's CAL default) otherwise.
pub fn default_coding(asw_id: &str) -> Option<u32> {
    asw_id.contains("0D9300012F").then_some(DEFAULT_CODING_0D9300012F)
}

/// What a preparation did (saved as `tcu_<id>.prep.json` next to the EEPROM state).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PrepRecord {
    pub image_id: String,
    pub coding: Option<u32>,
    /// Where the coding came from (Russian, shown as is).
    pub coding_source: String,
    /// Basic settings run: routine id and the last `22 0102` status.
    pub routines: Vec<(u16, Option<u8>)>,
    pub ok: bool,
    /// Failed step with the module's answer (NRC / routine status).
    pub failed: Option<String>,
    /// Simulated and wall time of the preparation, s.
    pub sim_s: f64,
    pub wall_s: f64,
    /// Workshop car: ECU image and bench build.
    pub ecu_image: String,
    pub build: String,
    /// End of the preparation, Unix time.
    pub unix_s: u64,
    /// Steps and tester exchanges with their simulated time.
    pub log: Vec<String>,
}

impl PrepRecord {
    /// One line for the UI and the session log.
    pub fn summary(&self) -> String {
        let rids = self.routines.iter().map(|(r, _)| format!("0x{r:04X}")).collect::<Vec<_>>().join("/");
        let ru = simemu_damos::is_ru();
        match &self.failed {
            None if ru => format!(
                "EEPROM мехатроника подготовлен самой прошивкой: кодирование {} ({}), базовые установки {rids} выполнены (статус 0x10); \
                 данные записала прошивка, studio в EEPROM не пишет (мастерская: ЭБУ {}, {:.0} с модели за {:.0} с)",
                self.coding.map_or("—".into(), |c| format!("{c:06}")),
                self.coding_source,
                self.ecu_image,
                self.sim_s,
                self.wall_s
            ),
            None => format!(
                "Mechatronic EEPROM prepared by the firmware itself: coding {} ({}), basic settings {rids} completed (status 0x10); \
                 data written by firmware, Studio never writes EEPROM directly (workshop: ECU {}, {:.0} s sim in {:.0} s)",
                self.coding.map_or("—".into(), |c| format!("{c:06}")),
                self.coding_source,
                self.ecu_image,
                self.sim_s,
                self.wall_s
            ),
            Some(f) if ru => format!("Подготовка мехатроника не выполнена: {f}. EEPROM — как его оставила прошивка (кнопка «Подготовить заново» в «Диагностике»)"),
            Some(f) => format!("Mechatronic preparation failed: {f}. EEPROM left as written by firmware (use 'Prepare again' in Diagnostics)"),
        }
    }
}

/// Progress of a running preparation.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Progress {
    pub step: String,
    pub frac: f64,
    pub sim_s: f64,
}

struct Job {
    id: String,
    coding: Option<u32>,
    immo_lab: bool,
    handle: JoinHandle<(Option<Vec<u8>>, PrepRecord)>,
    progress: Arc<Mutex<Progress>>,
    cancel: Arc<AtomicBool>,
}

/// Session side: the running preparation and the record of the TCU's state.
#[derive(Default)]
pub struct Prep {
    /// Prepare automatically at power-up (the studio server; off in library sessions and scenarios, which set up their TCU
    /// themselves).
    pub auto: bool,
    job: Option<Job>,
    /// Record of the running TCU image's state ([`PrepRecord`]), if a preparation made it.
    pub record: Option<PrepRecord>,
    /// Records of the preparations finished in this session, per image (also kept without a state directory).
    done: std::collections::BTreeMap<String, PrepRecord>,
}

impl Prep {
    pub fn running(&self) -> bool {
        self.job.is_some()
    }

    /// Stops a running preparation (its result is dropped).
    pub(crate) fn cancel(&mut self) {
        if let Some(j) = self.job.take() {
            j.cancel.store(true, Ordering::Relaxed);
        }
    }
}

impl Drop for Prep {
    fn drop(&mut self) {
        self.cancel();
    }
}

/// `/api/state` view.
#[derive(Serialize)]
pub struct PrepView<'a> {
    pub running: bool,
    pub progress: Option<Progress>,
    pub record: Option<&'a PrepRecord>,
    /// Coding the next preparation of the running image writes (`None`: the firmware's own default).
    pub plan_coding: Option<u32>,
}

/// Inputs of a preparation (owned: it runs on its own thread).
#[derive(Clone)]
pub struct PrepRequest {
    pub root: PathBuf,
    pub build: BenchConfig,
    /// ECU image file of the workshop car (an entry of the session's image list).
    pub ecu_image: String,
    pub immo_lab: bool,
}

/// The headless workshop car.
struct Workshop<'a> {
    s: StudioSession,
    progress: &'a Mutex<Progress>,
    cancel: &'a AtomicBool,
    log: Vec<String>,
    step: String,
}

impl Workshop<'_> {
    /// A line of the record (and of the session log unless it is a tester exchange).
    fn note(&mut self, line: impl Into<String>) {
        let l = format!("{:7.2} s  {}", self.s.plant.sim_time_s, line.into());
        if !l.contains("UDS >") {
            if simemu_damos::is_ru() {
                eprintln!("  мастерская ТКУ: {l}");
            } else {
                eprintln!("  TCU workshop: {l}");
            }
        }
        self.log.push(l);
    }

    fn set_step(&mut self, step: &str) {
        self.step = step.to_string();
        self.note(format!("— {step}"));
        self.tick();
    }

    fn tick(&self) {
        if let Ok(mut p) = self.progress.lock() {
            *p = Progress { step: self.step.clone(), frac: (self.s.plant.sim_time_s / EXPECTED_S).min(0.99), sim_s: self.s.plant.sim_time_s };
        }
    }

    /// Steps the co-simulation by `secs` (as fast as it runs).
    fn run_for(&mut self, secs: f64) -> Result<(), String> {
        let mut t = 0.0;
        while t < secs - 1e-9 {
            if self.cancel.load(Ordering::Relaxed) {
                return Err(simemu_damos::tr("cancelled", "отменено").into());
            }
            let dt = (secs - t).min(0.5);
            self.s.step(dt);
            t += dt;
            self.tick();
            if let Some(e) = &self.s.tcu_error {
                return Err(if simemu_damos::is_ru() {
                    format!("ТКУ остановилась: {e}")
                } else {
                    format!("TCU halted: {e}")
                });
            }
        }
        Ok(())
    }

    /// Ignition on, start button held until the engine runs on its own (the ECU firmware drives the starter), idle 2 s.
    fn start_engine(&mut self) -> Result<(), String> {
        let d = &mut self.s.plant.driver;
        (d.ignition_on, d.selector, d.brake_pct, d.pedal_pct) = (true, Selector::P, 60.0, 0.0);
        self.run_for(1.0)?;
        self.s.plant.driver.start_button = true;
        let mut running = false;
        for _ in 0..800 {
            if self.cancel.load(Ordering::Relaxed) {
                self.s.plant.driver.start_button = false;
                return Err(simemu_damos::tr("cancelled", "отменено").into());
            }
            self.s.step(0.01);
            let t = self.s.plant.telemetry();
            if t.rpm > 600.0 && t.starter_current_a <= 0.0 {
                running = true;
                break;
            }
        }
        self.s.plant.driver.start_button = false;
        if !running {
            return Err(if simemu_damos::is_ru() {
                format!("двигатель не запустился за 8 с (стартер включает прошивка ЭБУ; ЭБУ {})", self.s.image_label)
            } else {
                format!("engine did not start within 8 s (starter controlled by ECU firmware; ECU {})", self.s.image_label)
            });
        }
        self.run_for(2.0)?;
        let rpm = self.s.plant.telemetry().rpm;
        if rpm < 400.0 {
            return Err(if simemu_damos::is_ru() {
                format!("двигатель заглох после пуска ({rpm:.0} об/мин; ЭБУ {})", self.s.image_label)
            } else {
                format!("engine stalled after start ({rpm:.0} rpm; ECU {})", self.s.image_label)
            });
        }
        self.note(if simemu_damos::is_ru() {
            format!("двигатель работает, {rpm:.0} об/мин, P, тормоз")
        } else {
            format!("engine idling at {rpm:.0} rpm, P, brake held")
        });
        Ok(())
    }

    /// One tester job (as `/api/uds` sends it), stepping until it is answered; `Err` on no answer or a negative answer.
    fn uds(&mut self, service: &str, value: Option<u32>, rid: Option<u16>, max_s: f64) -> Result<UdsStatus, String> {
        let r = UdsRequest { target: "tcu".into(), service: service.into(), data: Vec::new(), value, rid };
        self.s.uds_request(&r)?;
        let mut t = 0.0;
        while self.s.uds_tcu.status.pending && t < max_s {
            if self.cancel.load(Ordering::Relaxed) {
                return Err(simemu_damos::tr("cancelled", "отменено").into());
            }
            self.s.step(0.01);
            t += 0.01;
        }
        let st = self.s.uds_tcu.status.clone();
        for x in &st.log {
            self.note(format!("UDS > {:<24} < {:<24} {}", x.request, x.response, x.note));
        }
        if st.pending {
            return Err(if simemu_damos::is_ru() {
                format!("{}: ТКУ не ответила за {max_s} с", st.request)
            } else {
                format!("{}: TCU did not respond within {max_s} s", st.request)
            });
        }
        match st.log.iter().find(|x| x.note.starts_with("NRC")) {
            Some(x) => Err(format!("{}: {} → {}", st.request, x.request, x.note)),
            None => Ok(st),
        }
    }

    /// [`Self::uds`], repeated every second while the module answers NRC 0x22 conditionsNotCorrect (e.g. the engine speed
    /// not yet stable after the start — precondition bit 13, docs/DQ250_IO_MAP.md §8), at most 15 s.
    fn uds_retry(&mut self, service: &str, value: Option<u32>, rid: Option<u16>) -> Result<UdsStatus, String> {
        let t0 = self.s.plant.sim_time_s;
        loop {
            match self.uds(service, value, rid, 3.0) {
                Err(e) if e.contains("NRC 0x22") && self.s.plant.sim_time_s - t0 < 15.0 => self.run_for(1.0)?,
                r => return r,
            }
        }
    }

    /// Ignition off (the TCU's after-run stores its EEPROM), supply off and on, then ignition on / engine started.
    fn key_cycle(&mut self, start_engine: bool) -> Result<(), String> {
        self.s.plant.driver.ignition_on = false;
        let (w0, t0) = (self.written(), self.s.plant.sim_time_s);
        let (mut tx, mut quiet) = (self.s.tcu_can_tx_total, 0.0);
        while quiet < AFTERRUN_QUIET_S && self.s.plant.sim_time_s - t0 < AFTERRUN_MAX_S {
            self.run_for(0.25)?;
            if self.s.tcu_can_tx_total == tx {
                quiet += 0.25;
            } else {
                (tx, quiet) = (self.s.tcu_can_tx_total, 0.0);
            }
        }
        let w1 = self.written();
        let dt = self.s.plant.sim_time_s - t0;
        let wb = w1 - w0;
        self.note(if simemu_damos::is_ru() {
            format!("зажигание выключено {dt:.2} с: ТКУ записала в EEPROM {wb} Б и замолчала на шине")
        } else {
            format!("ignition off {dt:.2} s: TCU wrote {wb} B to EEPROM and went bus-silent")
        });
        self.s.power_cycle_tcu();
        if start_engine {
            self.start_engine()
        } else {
            self.s.plant.driver.ignition_on = true;
            self.run_for(2.0)
        }
    }

    fn written(&self) -> u64 {
        self.s.tcu.as_ref().and_then(TcuBoard::nvm).map_or(0, |n| n.2)
    }
}

/// Runs the workshop preparation of the build's DQ250 on an erased EEPROM ([module docs](self)); the EEPROM the firmware
/// left (`None` when a step failed) and the record.
pub fn run(req: &PrepRequest, progress: &Mutex<Progress>, cancel: &AtomicBool) -> (Option<Vec<u8>>, PrepRecord) {
    let wall = std::time::Instant::now();
    let mut rec = PrepRecord { ecu_image: req.ecu_image.clone(), build: req.build.name.clone(), ..Default::default() };
    let mut log = Vec::new();
    let r = (|| -> Result<Vec<u8>, String> {
        let ru = simemu_damos::is_ru();
        let state = crate::studio::bench_build::BuildState::from_config(req.build.clone());
        let mut s = StudioSession::with_build(&req.root, state).map_err(|e| {
            if ru { format!("сессия мастерской: {e}") } else { format!("workshop session: {e}") }
        })?;
        // The car arrives warm (engine 90 °C, ATF 70 °C after a short stop) and stands on the road-load dyno in P.
        s.warm_car();
        s.plant.bench = BenchMode::RoadLoadDyno;
        s.immo_lab = req.immo_lab;
        s.load_image(&req.ecu_image)?;
        // erased part, whatever the environment says (SIMEMU_DQ250_EEPROM)
        let id = s.tcu.as_ref().and_then(TcuBoard::nvm).map(|n| n.0.to_string()).ok_or_else(|| {
            s.tcu_error.clone().unwrap_or_else(|| simemu_damos::tr("no DQ250 board in build", "нет платы DQ250 в сборке").into())
        })?;
        rec.image_id = id.clone();
        s.tcu_nvm.mem.insert(id.clone(), Vec::new());
        s.power_cycle_tcu_fresh();
        let path = StudioSession::dq250_image_path_for(
            &req.root,
            s.plant.engine_profile.displacement_m3(),
            req.build.tcu_image.as_deref(),
        )?;
        let rids = basic_setting_rids(&std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?);
        let mut w = Workshop { s, progress, cancel, log: Vec::new(), step: String::new() };
        w.note(if ru {
            format!("ТКУ {id}, EEPROM стёрт; ЭБУ {}; сборка {}; базовые установки по таблице прошивки: {rids:04X?}", w.s.image_label, req.build.name)
        } else {
            format!("TCU {id}, EEPROM erased; ECU {}; build {}; basic settings from firmware table: {rids:04X?}", w.s.image_label, req.build.name)
        });
        let res = (|| -> Result<Vec<u8>, String> {
            w.set_step(simemu_damos::tr("engine start (P, brake)", "пуск двигателя (P, тормоз)"));
            w.start_engine()?;
            w.set_step(simemu_damos::tr("coding", "кодирование"));
            let cal = w.uds("read_coding", None, None, 3.0)?.coding;
            let (coding, source) = match (req.build.tcu_coding, default_coding(&id), cal) {
                (Some(c), _, _) => (c, simemu_damos::tr("configured in bench build", "задано в сборке стенда").to_string()),
                (None, Some(c), _) => (c, simemu_damos::tr("default for 0D9300012F image", "по умолчанию для образа 0D9300012F").to_string()),
                (None, None, Some(c)) => (c, simemu_damos::tr("firmware default (CAL, read on erased EEPROM)", "значение прошивки по умолчанию (CAL, прочитано на стёртой EEPROM)").to_string()),
                (None, None, None) => return Err(simemu_damos::tr("coding: firmware did not report default coding (22 0600)", "кодирование: прошивка не сообщила своё значение (22 0600)").into()),
            };
            (rec.coding, rec.coding_source) = (Some(coding), source);
            let st = w.uds_retry("write_coding", Some(coding), None)?;
            if st.coding != Some(coding) || st.log.len() < 4 {
                return Err(if ru {
                    format!("кодирование {coding:06} не принято: {}", st.note)
                } else {
                    format!("coding {coding:06} rejected: {}", st.note)
                });
            }
            w.set_step(simemu_damos::tr("ignition off (TCU stores EEPROM)", "выключение зажигания (ТКУ сохраняет EEPROM)"));
            w.key_cycle(true)?;
            if rids.is_empty() {
                return Err(simemu_damos::tr("no basic settings found in firmware (routine table 31 01)", "в прошивке не найдено базовых установок (таблица процедур 31 01)").into());
            }
            for &rid in &rids {
                w.set_step(&if ru { format!("базовая установка 0x{rid:04X}") } else { format!("basic setting 0x{rid:04X}") });
                let mut attempt = 1;
                let (status, secs) = loop {
                    let st = w.uds_retry("basic_setting", None, Some(rid))?;
                    let (mut status, t0) = (st.routine_status, w.s.plant.sim_time_s);
                    while status == Some(0xC0) && w.s.plant.sim_time_s - t0 < 180.0 {
                        w.run_for(1.0)?;
                        status = w.uds("routine_status", None, None, 2.0)?.routine_status;
                    }
                    let secs = w.s.plant.sim_time_s - t0;
                    w.note(if ru {
                        format!("0x{rid:04X}: статус {status:02X?} ({}) через {secs:.1} с", status.map_or("?", routine_status_text))
                    } else {
                        format!("0x{rid:04X}: status {status:02X?} ({}) in {secs:.1} s", status.map_or("?", routine_status_text))
                    });
                    if status != Some(0x60) || attempt == BASIC_ATTEMPTS {
                        break (status, secs);
                    }
                    let _ = w.uds("routine_stop", None, Some(rid), 2.0);
                    attempt += 1;
                    w.set_step(&if ru {
                        format!("базовая установка 0x{rid:04X}, попытка {attempt} (прошивка прервала предыдущую)")
                    } else {
                        format!("basic setting 0x{rid:04X}, attempt {attempt} (firmware aborted previous attempt)")
                    });
                    w.run_for(2.0)?;
                };
                rec.routines.push((rid, status));
                if status != Some(0x10) {
                    let _ = w.uds("routine_stop", None, Some(rid), 2.0);
                    return Err(if ru {
                        format!("базовая установка 0x{rid:04X}: статус {status:02X?} ({}) через {secs:.1} с (попыток: {attempt})", status.map_or("нет ответа", routine_status_text))
                    } else {
                        format!("basic setting 0x{rid:04X}: status {status:02X?} ({}) after {secs:.1} s (attempts: {attempt})", status.map_or("no response", routine_status_text))
                    });
                }
                let t1 = w.s.plant.sim_time_s;
                let mut forks = w.s.plant.telemetry().fork_mm;
                loop {
                    w.run_for(0.5)?;
                    let f = w.s.plant.telemetry().fork_mm;
                    let moved = f.iter().zip(&forks).any(|(a, b)| (a - b).abs() > 0.05);
                    forks = f;
                    if !moved || w.s.plant.sim_time_s - t1 >= 5.0 {
                        break;
                    }
                }
                let f_settle = w.s.plant.sim_time_s - t1;
                let f_mm = forks.map(|x| (x * 10.0).round() / 10.0);
                w.note(if ru {
                    format!("вилки стоят через {f_settle:.1} с: {f_mm:?} мм")
                } else {
                    format!("shift forks settled in {f_settle:.1} s: {f_mm:?} mm")
                });
                w.uds("routine_stop", None, Some(rid), 3.0)?;
            }
            w.set_step(simemu_damos::tr("fault memory", "память неисправностей"));
            let st = w.uds("read_dtc", None, None, 3.0)?;
            let fault = |d: &str| d.rsplit("0x").next().and_then(|x| u8::from_str_radix(x, 16).ok()).is_some_and(|s| s & 0x0D != 0);
            let faults: Vec<String> = st.dtcs.iter().zip(&st.dtc_sae).filter(|(d, _)| fault(d)).map(|(d, s)| format!("{s} ({d})")).collect();
            w.note(if ru {
                format!("память неисправностей ТКУ: {}", if faults.is_empty() { "без отказов".into() } else { faults.join(", ") })
            } else {
                format!("TCU fault memory: {}", if faults.is_empty() { "no active faults".into() } else { faults.join(", ") })
            });
            w.set_step(simemu_damos::tr("ignition off (TCU stores adaptations)", "выключение зажигания (ТКУ сохраняет адаптации)"));
            w.key_cycle(false)?;
            let c = w.uds("read_coding", None, None, 3.0)?.coding;
            if c != Some(coding) {
                return Err(if ru {
                    format!("после выключения зажигания кодирование {c:?}, ожидалось {coding:06}")
                } else {
                    format!("coding after key cycle is {c:?}, expected {coding:06}")
                });
            }
            w.note(if ru { format!("кодирование {coding:06} сохранено") } else { format!("coding {coding:06} persisted") });
            w.s.tcu.as_ref().and_then(TcuBoard::nvm).map(|n| n.1.to_vec()).ok_or_else(|| simemu_damos::tr("missing TCU EEPROM", "нет EEPROM ТКУ").into())
        })();
        let step = w.step.clone();
        rec.sim_s = w.s.plant.sim_time_s;
        log = std::mem::take(&mut w.log);
        res.map_err(|e| if ru { format!("шаг «{step}»: {e}") } else { format!("step '{step}': {e}") })
    })();
    rec.wall_s = wall.elapsed().as_secs_f64();
    rec.unix_s = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    rec.log = log;
    match r {
        Ok(e) => {
            rec.ok = true;
            (Some(e), rec)
        }
        Err(e) => {
            rec.failed = Some(e);
            (None, rec)
        }
    }
}

impl StudioSession {
    /// After every TCU power-up: the DQ250's gear set from its calibration, the notes about its EEPROM, and — when the
    /// image has no saved state and [`Prep::auto`] is set — the workshop preparation.
    pub(crate) fn tcu_booted(&mut self) {
        self.tcu_notes.clear();
        let Some(id) = self.tcu.as_ref().and_then(TcuBoard::nvm).map(|n| n.0.to_string()) else {
            self.tcu_prep.cancel();
            self.tcu_nvm.hold = None;
            (self.tcu_prep.record, self.tcu_image_path) = (None, None);
            return;
        };
        if self
            .tcu_prep
            .job
            .as_ref()
            .is_some_and(|j| j.id != id || j.coding != self.bench_build.active.tcu_coding || j.immo_lab != self.immo_lab)
        {
            self.tcu_prep.cancel();
            self.tcu_nvm.hold = None;
        }
        self.tcu_image_path = StudioSession::dq250_image_path_for(
            &self.root,
            self.plant.engine_profile.displacement_m3(),
            self.bench_build.active.tcu_image.as_deref(),
        )
        .ok();
        if let Some(n) = self.fit_gear_set() {
            self.tcu_notes.push(n);
        }
        if self.tcu_prep.auto && !self.tcu_nvm.has_state(&id) && !self.tcu_prep.running() {
            self.start_tcu_prep();
        }
        self.tcu_prep.record =
            self.tcu_prep.done.get(&id).cloned().or_else(|| self.tcu_nvm.record_file(&id).and_then(|p| std::fs::read(p).ok()).and_then(|b| serde_json::from_slice(&b).ok()));
        let running = self.tcu_prep.job.as_ref().is_some_and(|j| j.id == id);
        let ru = simemu_damos::is_ru();
        let note = match (&self.tcu_prep.record, running) {
            (_, true) if ru => "EEPROM мехатроника стёрт (дампа нет): идёт подготовка в мастерской — кодирование и базовые установки выполняет сама прошивка \
                                в отдельной симуляции; по окончании ТКУ перезагрузится с подготовленным EEPROM"
                .to_string(),
            (_, true) => "Mechatronic EEPROM erased (no dump): workshop preparation running — coding and basic settings are performed by the firmware itself \
                          in a background co-simulation; TCU will reboot with the prepared EEPROM when finished"
                .to_string(),
            (Some(r), _) => r.summary(),
            (None, _) if self.tcu_nvm.mem.get(&id).is_some_and(Vec::is_empty) => simemu_damos::tr(
                "Mechatronic EEPROM erased (virgin part): uncoded, basic settings not performed",
                "EEPROM мехатроника стёрт (как новый мехатроник): без кодирования и базовых установок",
            )
            .into(),
            (None, _) if self.tcu_nvm.has_state(&id) => simemu_damos::tr(
                "Mechatronic EEPROM — persisted as written by firmware in previous sessions",
                "EEPROM мехатроника — как его записала прошивка в прошлых включениях",
            )
            .into(),
            (None, _) => simemu_damos::tr(
                "Mechatronic EEPROM erased (no dump): uncoded, basic settings not performed",
                "EEPROM мехатроника стёрт (дампа нет): без кодирования и базовых установок",
            )
            .into(),
        };
        if ru {
            eprintln!("  ТКУ {id}: {note}");
        } else {
            eprintln!("  TCU {id}: {note}");
        }
        self.tcu_notes.insert(0, note);
    }

    /// The build selects another DQ250 image than the running one.
    pub(crate) fn tcu_image_changed(&self) -> bool {
        self.tcu.as_ref().and_then(TcuBoard::dq250).is_some()
            && StudioSession::dq250_image_path_for(
                &self.root,
                self.plant.engine_profile.displacement_m3(),
                self.bench_build.active.tcu_image.as_deref(),
            )
            .ok()
                != self.tcu_image_path
    }

    /// Plant gear set = the tooth table of the DQ250 calibration (`tcu_dq250::gear_set`): a mechatronic is coded for its
    /// own gearbox. Returns the note when the build's gear set was replaced.
    pub(crate) fn fit_gear_set(&mut self) -> Option<String> {
        let gs = self.tcu.as_ref()?.dq250()?.gears?;
        let p = &self.plant.drivetrain_profile;
        let d = p.clone().with_gear_set(gs.ratios, gs.finals, gs.reverse);
        if d.gear_ratios == p.gear_ratios && d.final_drives == p.final_drives && d.reverse_ratio == p.reverse_ratio {
            return None;
        }
        self.plant.drivetrain_profile = d;
        Some(if simemu_damos::is_ru() {
            format!(
                "Передаточные числа КПП — по таблице зубьев калибровки ТКУ (мехатроник стоит на своей коробке): {:?}, главные пары {:?}, R {:.3}",
                gs.ratios.map(|r| (r * 1000.0).round() / 1000.0),
                gs.finals.map(|r| (r * 1000.0).round() / 1000.0),
                gs.reverse.0
            )
        } else {
            format!(
                "Gearbox ratios matched to TCU calibration tooth table: {:?}, final drives {:?}, R {:.3}",
                gs.ratios.map(|r| (r * 1000.0).round() / 1000.0),
                gs.finals.map(|r| (r * 1000.0).round() / 1000.0),
                gs.reverse.0
            )
        })
    }

    /// Starts the workshop preparation of the running DQ250 image (one at a time); its state file is held until it ends.
    pub(crate) fn start_tcu_prep(&mut self) {
        let Some(id) = self.tcu.as_ref().and_then(TcuBoard::nvm).map(|n| n.0.to_string()) else { return };
        if self.tcu_prep.running() {
            return;
        }
        let req = PrepRequest { root: self.root.clone(), build: self.bench_build.active.clone(), ecu_image: self.image_file.clone(), immo_lab: self.immo_lab };
        let (progress, cancel) = (Arc::new(Mutex::new(Progress::default())), Arc::new(AtomicBool::new(false)));
        let (p, c) = (progress.clone(), cancel.clone());
        let spawned = std::thread::Builder::new().name("dq250-prep".into()).spawn(move || {
            crate::studio::rt::set_thread_role(crate::studio::rt::ThreadRole::Background);
            run(&req, &p, &c)
        });
        match spawned {
            Ok(handle) => {
                if simemu_damos::is_ru() {
                    eprintln!("  ТКУ {id}: EEPROM стёрт — подготовка мехатроника в мастерской (отдельная симуляция)");
                } else {
                    eprintln!("  TCU {id}: EEPROM erased — starting background workshop preparation");
                }
                let coding = self.bench_build.active.tcu_coding;
                let immo_lab = self.immo_lab;
                self.tcu_nvm.hold = Some(id.clone());
                self.tcu_prep.job = Some(Job { id, coding, immo_lab, handle, progress, cancel });
            }
            Err(e) => self.tcu_notes.push(if simemu_damos::is_ru() {
                format!("Подготовка мехатроника не запущена: {e}")
            } else {
                format!("Failed to start mechatronic preparation: {e}")
            }),
        }
    }

    /// Applies a finished preparation: its EEPROM becomes the image's state (memory and file) and the live TCU of that
    /// image powers up with it. Called every [`StudioSession::step`].
    pub(crate) fn poll_tcu_prep(&mut self) {
        if !self.tcu_prep.job.as_ref().is_some_and(|j| j.handle.is_finished()) {
            return;
        }
        let job = self.tcu_prep.job.take().expect("job");
        self.tcu_nvm.hold = None;
        let (eeprom, rec) = job.handle.join().unwrap_or_else(|_| {
            (
                None,
                PrepRecord {
                    failed: Some(simemu_damos::tr("preparation thread panicked", "поток подготовки упал").into()),
                    ..Default::default()
                },
            )
        });
        if simemu_damos::is_ru() {
            eprintln!("  ТКУ {}: {}", job.id, rec.summary());
        } else {
            eprintln!("  TCU {}: {}", job.id, rec.summary());
        }
        if let Some(p) = self.tcu_nvm.record_file(&job.id) {
            let r = p.parent().map_or(Ok(()), std::fs::create_dir_all).and_then(|_| std::fs::write(&p, serde_json::to_vec_pretty(&rec).unwrap_or_default()));
            if let Err(e) = r {
                if simemu_damos::is_ru() {
                    eprintln!("  ТКУ: запись о подготовке не сохранена в {}: {e}", p.display());
                } else {
                    eprintln!("  TCU: failed to save preparation record to {}: {e}", p.display());
                }
            }
        }
        self.tcu_prep.done.insert(job.id.clone(), rec);
        let live = self.tcu.as_ref().and_then(TcuBoard::nvm).is_some_and(|n| n.0 == job.id);
        let ok = eeprom.is_some();
        match eeprom {
            Some(e) => self.tcu_nvm.put(&job.id, e),
            None if live => {
                let m = self.tcu.as_ref().and_then(TcuBoard::nvm).map(|n| n.1.to_vec()).unwrap_or_default();
                self.tcu_nvm.put(&job.id, m);
            }
            None => {}
        }
        if live && ok {
            self.power_cycle_tcu_fresh();
        } else {
            self.tcu_booted();
        }
    }

    /// UI actions on the DQ250's EEPROM: `erase` — an erased part (no preparation); `prepare` —
    /// erased and prepared again.
    pub fn tcu_eeprom_action(&mut self, action: &str) -> Result<(), String> {
        let id = self
            .tcu
            .as_ref()
            .and_then(TcuBoard::nvm)
            .map(|n| n.0.to_string())
            .ok_or_else(|| simemu_damos::tr("no DQ250 TCU in build", "нет ТКУ DQ250 в сборке").to_string())?;
        self.tcu_prep.cancel();
        self.tcu_prep.done.remove(&id);
        self.tcu_nvm.hold = None;
        match action {
            "erase" => {
                self.tcu_nvm.reset(&id, true);
                self.power_cycle_tcu_fresh();
            }
            "prepare" => {
                self.tcu_nvm.reset(&id, false);
                self.power_cycle_tcu_fresh();
                if !self.tcu_prep.running() {
                    self.start_tcu_prep();
                    self.tcu_booted();
                }
            }
            a => return Err(format!("unknown TCU EEPROM action {a}")),
        }
        Ok(())
    }

    /// `/api/state` view of the preparation.
    pub fn tcu_prep_view(&self) -> PrepView<'_> {
        let j = self.tcu_prep.job.as_ref();
        let t = self.tcu.as_ref().and_then(TcuBoard::dq250);
        let id = t.map(|t| t.info.asw_id.as_str()).unwrap_or_default();
        PrepView {
            running: j.is_some(),
            progress: j.and_then(|j| j.progress.lock().ok().map(|p| p.clone())),
            record: self.tcu_prep.record.as_ref(),
            plan_coding: self.bench_build.active.tcu_coding.or_else(|| default_coding(id)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    /// Both DQ250 images offer the same two basic settings in their routine-start chain (VERIFIED-S).
    #[test]
    fn basic_settings_from_the_routine_table() {
        let Some(root) = root() else { return eprintln!("SKIP: images not found") };
        let imgs = dq250_images(&root);
        assert!(!imgs.is_empty(), "no DQ250 image in {}", root.display());
        for i in &imgs {
            let path = StudioSession::dq250_image_path(&root, Some(&i.file)).unwrap();
            let rids = basic_setting_rids(&std::fs::read(path).unwrap());
            println!("{} {} default coding {:?}: basic settings {rids:04X?}", i.file, i.asw_id, i.default_coding);
            assert_eq!(rids, [0x0380, 0x04D4], "{}", i.file);
        }
        assert_eq!(basic_setting_rids(&[0u8; 64]), Vec::<u16>::new());
    }

    /// Studio session as the server runs it on a 1.8 TSI + DQ250 build (warm, road-load dyno), ECU image `image`,
    /// TCU state directory `dir`, automatic preparation on.
    fn studio(root: &Path, image: &str, dir: &Path) -> StudioSession {
        let mut build = simemu_physics::bench_config::preset("cjsa_is38_dq250").expect("cjsa_is38_dq250");
        if std::env::var("SIMEMU_DQ250_IMAGE").is_err() {
            build.tcu_image = Some(crate::paths::DQ250_012F_ID.into());
        }
        let mut s = StudioSession::with_build(root, crate::studio::bench_build::BuildState::from_config(build)).expect("session");
        s.warm_car();
        s.plant.bench = BenchMode::RoadLoadDyno;
        s.load_image(image).expect("image");
        s.tcu_prep.auto = true;
        s.set_nvm_dir(dir.to_path_buf());
        s
    }

    /// The studio prepares a DQ250 by itself (`scripts/run_scenarios.sh`: `dq250_studio_prepare`, native — on Unicorn
    /// the DQ250 ASW halts once its speed sensors run, MILESTONE §2.3): ECU image `SIMEMU_IMAGE` (default stock SC8O40
    /// 264H), TCU image `SIMEMU_DQ250_IMAGE` (default 0D9300012F), empty state directory. Asserted: the erased
    /// EEPROM starts the preparation; the firmware completes it (coding accepted and kept, every basic setting of its
    /// routine table at status 0x10); the state file and the record are saved; a new session on the same directory powers
    /// up from them without preparing again; started, in D with the brake held the TCU engages 1st (`WBA_03` gear 1,
    /// 1/3 actuator in) and the car creeps once the brake is released.
    #[test]
    #[ignore = "slow: headless workshop co-simulation (ECU + DQ250 + plant) and a drive-away"]
    fn dq250_studio_prepare() {
        let Some(root) = root() else { return eprintln!("SKIP: images not found") };
        let image = std::env::var("SIMEMU_IMAGE").unwrap_or_else(|_| "8V0906264H__0005.bin".into());
        if !crate::paths::has_firmware(&root, &image) {
            return eprintln!("SKIP: {image} not found (scripts/frf_to_bin.py)");
        }
        if StudioSession::dq250_image_path(&root, None).is_err() {
            return eprintln!("SKIP: no DQ250 image");
        }
        let dir = std::env::temp_dir().join(format!("simemu_dq250_prep_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut s = studio(&root, &image, &dir);
        let id = s.tcu.as_ref().and_then(TcuBoard::nvm).map(|n| n.0.to_string()).expect("DQ250 board");
        assert!(s.tcu_prep.running(), "erased EEPROM must start the preparation: {:?}", s.tcu_notes);
        let t0 = std::time::Instant::now();
        while s.tcu_prep.running() && t0.elapsed().as_secs() < 1200 {
            std::thread::sleep(std::time::Duration::from_millis(100));
            s.poll_tcu_prep();
        }
        let rec = s.tcu_prep.record.clone().expect("record");
        for l in &rec.log {
            println!("  {l}");
        }
        println!("{}", rec.summary());
        assert!(rec.ok, "preparation failed: {:?}", rec.failed);
        assert!(!rec.routines.is_empty() && rec.routines.iter().all(|r| r.1 == Some(0x10)), "{:02X?}", rec.routines);
        assert!(s.tcu_nvm.file(&id).is_some_and(|p| p.exists()) && s.tcu_nvm.record_file(&id).is_some_and(|p| p.exists()));
        assert!(
            s.tcu_notes[0].contains("prepared by the firmware itself") || s.tcu_notes[0].contains("подготовлен самой прошивкой"),
            "{:?}",
            s.tcu_notes
        );
        drop(s);

        // next studio start on the same state directory
        let mut s = studio(&root, &image, &dir);
        println!("{:?}", s.tcu_notes);
        assert!(!s.tcu_prep.running() && s.tcu_prep.record.as_ref().is_some_and(|r| r.ok), "saved state not used: {:?}", s.tcu_notes);
        let d = &mut s.plant.driver;
        (d.ignition_on, d.selector, d.brake_pct) = (true, Selector::P, 60.0);
        s.step(1.0);
        s.plant.driver.start_button = true;
        for _ in 0..800 {
            s.step(0.01);
            let t = s.plant.telemetry();
            if t.rpm > 600.0 && t.starter_current_a <= 0.0 {
                break;
            }
        }
        s.plant.driver.start_button = false;
        s.step(5.0);
        assert!(s.plant.telemetry().rpm > 400.0, "engine not running");
        s.uds_request(&UdsRequest { target: "tcu".into(), service: "read_coding".into(), data: Vec::new(), value: None, rid: None }).unwrap();
        for _ in 0..300 {
            s.step(0.01);
        }
        assert_eq!(s.uds_tcu.status.coding, rec.coding, "coding after the restart");
        s.plant.driver.selector = Selector::D;
        let (mut first, mut creep) = (None, 0f64);
        for i in 0..1400 {
            s.plant.driver.brake_pct = if i < 1000 { 60.0 } else { 0.0 };
            s.step(0.01);
            let (tel, bus) = (s.plant.telemetry(), s.state().gearbox_bus);
            if first.is_none() && bus.gear == Some(1) && tel.fork_mm[0] >= 6.5 && tel.fork_mm[3] > -6.5 {
                first = Some(i as f64 * 0.01);
            }
            if i >= 1000 {
                creep = creep.max(tel.roller_kmh);
            }
        }
        let hw = s.tcu.as_ref().unwrap().cpu().hw();
        println!("D: 1st engaged at {first:?} s, creep {creep:.1} km/h | TCU resets {:?} traps {}", hw.reset_log, hw.trap_count);
        assert!(hw.reset_log.is_empty() && hw.trap_count == 0 && s.tcu_error.is_none(), "TCU reset/trap/error {:?}", s.tcu_error);
        assert!(first.is_some(), "D: 1st not engaged / not reported on WBA_03");
        assert!(creep > 2.0, "no creep: {creep:.1} km/h");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
