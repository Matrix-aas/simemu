//! «Проверка прошивки перед заливкой» — the bench's standard test program for an ECU image (+ the build's TCU image) on
//! the selected bench build, run headless as fast as the co-simulation goes, and a report «сток vs эта прошивка».
//!
//! Every phase is its own co-simulation of the same car (a fresh [`StudioSession`] on the bench build, warm car, the
//! image loaded with the usual post-programming steps and lab immobiliser binding, the DQ250 EEPROM the firmware prepared
//! itself). Only a driver and test-cell equipment act (AGENTS.md §2.2): ignition, start button, selector, brake, pedal,
//! the bench mode (engine dyno / road-load chassis dyno), ambient air and the fuel in the tank. The firmware decides
//! everything; the plant decides what breaks. The report reads the plant (cylinder pressure, knock, pre-ignition,
//! component temperatures, damage, failures), the firmware's own observables over UDS (spark, knock retard per cylinder,
//! λ / boost / rail set points, fault memory) and compares every quantity with a physical limit that carries its
//! source (or `UNVERIFIED`).
//!
//! Phases ([`PROGRAM`]): WOT sweep on the engine dyno, WOT pull in gears on the road-load dyno, standing launch
//! (brake + full pedal, release), the pull after a heat soak at 40 °C ambient, the launch on 92 RON fuel.
//! Verdict ([`Verdict`]): «сломает» — a part failed in the plant or a fracture limit was reached; «риск» — a limit with a
//! risk meaning was exceeded (each reason names the phase, the moment and the cylinder); «OK» otherwise.

use crate::log_tools::report::{fmt_num, html_page, svg_plot, Series, Span};
use crate::studio::session::{StudioSession, UdsRequest};
use crate::studio::tcu_io::TcuBoard;
use serde::Serialize;
use simemu_physics::bench_config::BenchConfig;
use simemu_physics::{BenchMode, Selector};
use std::fmt::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

// ================================================================================================================
// Program
// ================================================================================================================

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    /// Engine on the eddy-current dyno (gearbox removed), pedal 100 %, absorber speed ramped.
    DynoSweep,
    /// Road-load chassis dyno: D, gentle pedal to 30 km/h, then kickdown.
    Pull,
    /// Road-load chassis dyno: D, brake + full pedal, release the brake, full pedal.
    Launch,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct PhaseDef {
    pub id: &'static str,
    pub ru: &'static str,
    /// What the driver / the test cell does (Russian, shown in the report).
    pub what: &'static str,
    pub kind: Kind,
    /// Ambient air (test-cell conditioning), °C; `None` = the build's.
    pub ambient_c: Option<f64>,
    /// Fuel in the tank, RON; `None` = the build's.
    pub ron: Option<f64>,
    /// Idle in P before the run (heat soak), s.
    pub soak_s: f64,
}

pub const PROGRAM: [PhaseDef; 5] = [
    PhaseDef {
        id: "dyno",
        ru: "Моторный стенд: развёртка на полном газу",
        what: "двигатель на тормозном стенде (КПП снята), педаль 100 %, тормоз стенда ведёт обороты 1500 → 6500 об/мин со скоростью 250 об/мин/с",
        kind: Kind::DynoSweep,
        ambient_c: None,
        ron: None,
        soak_s: 0.0,
    },
    PhaseDef {
        id: "pull",
        ru: "Барабанный стенд: разгон в передачах",
        what: "D, педаль 25 % до 30 км/ч, затем педаль 100 % (kickdown) до 160 км/ч или 25 с",
        kind: Kind::Pull,
        ambient_c: None,
        ron: None,
        soak_s: 0.0,
    },
    PhaseDef {
        id: "launch",
        ru: "Старт с места (ланч)",
        what: "селектор S, тормоз 100 % и педаль 100 % 3 с, тормоз отпущен сразу, педаль 100 % до 140 км/ч или 15 с (ланч ТКУ DQ250 включается только в S/M)",
        kind: Kind::Launch,
        ambient_c: None,
        ron: None,
        soak_s: 0.0,
    },
    PhaseDef {
        id: "hot",
        ru: "Жара и прогрев на холостом ходу",
        what: "воздух в боксе 40 °C, 60 с на ХХ в P (прогрев подкапотного пространства), затем разгон как в «Разгон в передачах»",
        kind: Kind::Pull,
        ambient_c: Some(40.0),
        ron: None,
        soak_s: 60.0,
    },
    PhaseDef {
        id: "ron92",
        ru: "Топливо АИ-92",
        what: "в баке АИ-92 (RON 92), старт с места как в «Старт с места»",
        kind: Kind::Launch,
        ambient_c: None,
        ron: Some(92.0),
        soak_s: 0.0,
    },
];

// ================================================================================================================
// Limits
// ================================================================================================================

/// A physical limit the report checks: `value` in `unit`, exceeded upwards. `breaks` = reaching it is a failure.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Limit {
    pub id: &'static str,
    pub ru: &'static str,
    pub unit: &'static str,
    pub value: f64,
    pub breaks: bool,
    /// Where the value comes from (or `UNVERIFIED`).
    pub source: &'static str,
}

pub const LIMITS: &[Limit] = &[
    Limit { id: "land_fracture", ru: "Перемычка поршня: напряжение / предел прочности при её температуре", unit: "—", value: 1.0, breaks: true, source: "модель plant: σ = K·(p_лок − p_картер), K = 3 UNVERIFIED; прочность AlSi12CuNiMg M124 при 20–350 °C — Shariyat et al., LAJSS 13(6) 2016, табл. 7 (materials.rs); сплав поршня OEM не проверен" },
    Limit { id: "land_fatigue", ru: "Перемычка поршня: пульсирующий цикл выше предела выносливости", unit: "σ/UTS", value: 0.62, breaks: false, source: "вывод из данных M124 (materials.rs, LAJSS 2016 / MAHLE): предел выносливости f = σ_f/UTS ≈ 0.45–0.49 при 20–350 °C; пульсирующий цикл 0…σ по Гудману превышает его при σ/UTS > 2f/(1+f) ≈ 0.62" },
    Limit { id: "piston_damage", ru: "Накопленное повреждение перемычки за программу (Майнер)", unit: "—", value: 1e-4, breaks: false, source: "порог «ресурс < 10 000 таких программ» — выбор отчёта (UNVERIFIED); 1.0 = разрушение" },
    Limit { id: "p_max", ru: "Пиковое давление в цилиндре", unit: "бар", value: 130.0, breaks: false, source: "MAHLE (пресс-релиз 2013): у современных бензиновых двигателей пиковое давление «свыше 12 МПа»; проектный предел EA888 Gen3 не найден — 130 бар UNVERIFIED" },
    Limit { id: "knock", ru: "Амплитуда детонации (макс. амплитуда колебаний давления)", unit: "бар", value: 5.0, breaks: false, source: "Fitton & Nates, N&O Joernaal 1992, рис. 7 и табл. 1: «тяжёлая» детонация ≈ 5 бар — разрушение перемычки за 0.4 мин на 4000 об/мин (CFR, 80 мм); перенос на EA888 UNVERIFIED" },
    Limit { id: "preign", ru: "Предвоспламенения (LSPI / калильное)", unit: "соб.", value: 0.5, breaks: false, source: "Rönn et al., Prog. Energy Combust. Sci. 95 (2023) 101064: единичный цикл суперзвона (волны давления > 160 бар) может сломать перемычку; MAHLE 2017: LSPI до 300+ бар" },
    Limit { id: "crown_c", ru: "Днище поршня", unit: "°C", value: 320.0, breaks: false, source: "MAHLE 2016: турбо-GDI с форсунками охлаждения на номинале — днище 280–320 °C (верх диапазона)" },
    Limit { id: "groove_c", ru: "Канавка 1-го кольца", unit: "°C", value: 260.0, breaks: false, source: "MAHLE 2016: 1-я канавка 220–260 °C на номинале (выше — коксование масла, падение прочности)" },
    Limit { id: "exv_c", ru: "Тарелка выпускного клапана", unit: "°C", value: 850.0, breaks: false, source: "UNVERIFIED (Heywood 1988 §12.6: выпускной клапан 700–800 °C на номинале)" },
    Limit { id: "plug_c", ru: "Тепловой конус свечи", unit: "°C", value: 850.0, breaks: false, source: "NGK/Bosch техн. информация: калильное зажигание выше ≈ 850–950 °C (порог модели plant `hot_surface_c`)" },
    Limit { id: "egt_c", ru: "Газ перед турбиной (выше предела в сумме дольше 5 с)", unit: "°C", value: 980.0, breaks: false, source: "Audi eSSP 920243 «EA888 Gen 3» (11/2013), с. 53: колесо турбины из Inconel рассчитано на температуру до 980 °C (OEM); для IS38 не найден. Допуск 5 с на кратковременный выброс (удержание ланча) — UNVERIFIED; ползучесть колеса считает модель plant" },
    Limit { id: "turbo", ru: "Колесо турбины/компрессора: напряжение / прочность", unit: "—", value: 0.8, breaks: false, source: "модель plant (materials.rs AA2618 / IN713C, UNVERIFIED); 1.0 = разрыв колеса" },
    Limit { id: "lambda_lean", ru: "λ цилиндра на полной нагрузке (обеднение)", unit: "—", value: 1.05, breaks: false, source: "UNVERIFIED (бедная смесь на наддуве — рост температур и детонации)" },
    Limit { id: "rail_deficit", ru: "Рампа DI ниже уставки прошивки", unit: "МПа", value: 2.0, breaks: false, source: "UNVERIFIED (ТНВД на пределе подачи)" },
    Limit { id: "di_duty", ru: "Скважность форсунки DI", unit: "—", value: 0.42, breaks: false, source: "вывод: однократный впрыск должен уложиться от ВМТ газообмена до ≈ 60° до ВМТ сжатия ≈ 300° из 720° (UNVERIFIED)" },
    Limit { id: "clutch_torque", ru: "Момент двигателя / статическая ёмкость замкнутого сцепления (ТКУ держит полное давление, а оно проскальзывает > 100 об/мин)", unit: "—", value: 1.0, breaks: false, source: "модель plant: ёмкость z·μ·F·r по давлению в поршне (характеристика прошивки ТКУ, clutch.rs)" },
    Limit { id: "clutch_c", ru: "Поверхность сцепления", unit: "°C", value: 300.0, breaks: false, source: "модель plant clutch.rs: остекловывание мокрой накладки выше 300 °C (UNVERIFIED)" },
];

fn limit(id: &str) -> &'static Limit {
    LIMITS.iter().find(|l| l.id == id).expect("limit")
}

// ================================================================================================================
// Results
// ================================================================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Verdict {
    Ok,
    Risk,
    Breaks,
}

impl Verdict {
    pub fn ru(self) -> &'static str {
        match self {
            Verdict::Ok => "OK",
            Verdict::Risk => "риск",
            Verdict::Breaks => "сломает",
        }
    }
}

/// Why the verdict is what it is: a limit exceeded (or a failure) in a phase, at a moment, in a cylinder.
#[derive(Debug, Clone, Serialize)]
pub struct Reason {
    pub level: Verdict,
    pub phase: String,
    /// Simulated time since the start of the phase's driving program, s.
    pub t_s: f64,
    /// Cylinder 1..4 when it concerns one.
    pub cyl: Option<usize>,
    pub limit: Option<&'static str>,
    pub text: String,
}

/// Per-cylinder maxima of a phase.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Cyl {
    pub p_max_bar: f64,
    pub p_local_bar: f64,
    pub knock_max_bar: f64,
    pub knock_cycles: u32,
    pub preign: u32,
    pub lspi: u32,
    /// Cycles in which the spark lit no flame although there was fuel (plant).
    pub misfires: u32,
    pub land_stress: f64,
    pub piston_damage: f64,
    pub crown_c: f64,
    pub land_c: f64,
    pub groove_c: f64,
    pub exv_c: f64,
    pub plug_c: f64,
    pub lambda_min_load: f64,
    pub lambda_max_load: f64,
    pub di_duty: f64,
    /// Lowest knock retard the firmware reported for this cylinder (UDS `Knock Cyl n`), °.
    pub fw_knock_retard: f64,
}

/// One 0.1 s sample of a phase (plots, comparison with a log).
#[derive(Debug, Clone, Default, Serialize)]
pub struct Sample {
    pub t: f64,
    pub rpm: f64,
    pub kmh: f64,
    pub pedal: f64,
    pub brake: f64,
    pub map_kpa: f64,
    pub egt_c: f64,
    pub lambda: f64,
    pub torque_nm: f64,
    pub p_max: [f64; 4],
    pub knock: [f64; 4],
    pub land: [f64; 4],
    pub crown: [f64; 4],
    pub clutch_slip_rpm: f64,
    /// Firmware over UDS (last answer): spark, knock retard per cylinder, PUT and its set point, λ set point, IAT.
    pub fw_ign: f64,
    pub fw_knock: [f64; 4],
    pub fw_put: f64,
    pub fw_put_sp: f64,
    pub fw_lambda_sp: f64,
    pub fw_iat: f64,
    /// Firmware misfire counter `MIS_SUM_B` (UDS `Misfires`).
    pub fw_misfires: f64,
    /// Accelerator pedal as the firmware reads it, %, and its torque request, Nm (UDS `Pedal Pos`, `Torque Req`).
    pub fw_pedal: f64,
    pub fw_tq_req: f64,
    /// Throttle position as the firmware reads it, % (UDS `TPS`).
    pub fw_tps: f64,
    /// λ short-term fuel trim, % (UDS `STFT`), MPI pulse of cylinder 1, ms (`Inj PW MPI`).
    pub fw_stft: f64,
    pub fw_mpi_ms: f64,
    /// λ as the firmware measures it (upstream sensor, UDS `Lambda`).
    pub fw_lambda: f64,
    /// Turbocharger speed: the firmware's model (UDS `Turbo Speed`, krpm) and the plant's.
    pub fw_turbo_krpm: f64,
    pub turbo_krpm: f64,
    /// Plant: air and fuel (DI + MPI as delivered) per cylinder and stroke, mg (mean of the cylinders).
    pub air_mg: f64,
    pub fuel_mg: f64,
    /// In-cylinder λ (plant, at spark, liquid fuel included) and the liquid fuel films on crown + liner, mg.
    pub lambda_cyl: [f64; 4],
    pub film_mg: [f64; 4],
    /// First DI pulse of each cylinder as the firmware drove it: start, ° before firing TDC, and width, ms.
    pub di_soi_deg: [f64; 4],
    pub di_ms: [f64; 4],
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct PhaseResult {
    pub id: &'static str,
    pub ru: &'static str,
    pub what: &'static str,
    /// Set when the phase could not be run to its end (start failed, a board stopped, cancelled).
    pub error: Option<String>,
    pub sim_s: f64,
    pub cyl: [Cyl; 4],
    pub rpm_max: f64,
    pub kmh_max: f64,
    pub map_max_kpa: f64,
    pub torque_max_nm: f64,
    pub power_max_kw: f64,
    pub egt_max_c: f64,
    /// Time with the gas before the turbine above the limit `egt_c`, s.
    pub egt_over_s: f64,
    pub turbo_rpm_max: f64,
    pub turbo_stress: f64,
    pub iat_max_c: f64,
    pub rail_deficit_mpa: f64,
    pub clutch_torque_ratio: f64,
    pub clutch_surface_c: f64,
    /// Launch hold (brake + full pedal at standstill): engine speed and the firmware's spark at its end.
    pub launch_hold_rpm: Option<f64>,
    pub launch_hold_ign: Option<f64>,
    /// What the driver set up before the run (e.g. the MultiMap slot) and other remarks.
    pub notes: Vec<String>,
    /// Plant failures with the moment they appeared, s.
    pub failures: Vec<(f64, String)>,
    pub ecu_dtcs: Vec<String>,
    pub tcu_dtcs: Vec<String>,
    pub samples: Vec<Sample>,
    pub reasons: Vec<Reason>,
    pub verdict: Option<Verdict>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct ImageResult {
    pub image: String,
    pub label: String,
    pub phases: Vec<PhaseResult>,
    pub verdict: Option<Verdict>,
    pub reasons: Vec<Reason>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Report {
    pub build: String,
    pub tcu_image: Option<String>,
    pub tested: ImageResult,
    pub reference: Option<ImageResult>,
    pub verdict: Verdict,
    /// One-paragraph summary in Russian.
    pub summary: String,
    pub limits: &'static [Limit],
    pub program: &'static [PhaseDef],
    pub notes: Vec<String>,
    pub sim_s: f64,
    pub wall_s: f64,
    pub unix_s: u64,
    /// The DQ250 EEPROM the cars ran with ((image id, bytes); prepared by its firmware if the request had none).
    #[serde(skip)]
    #[cfg_attr(not(test), allow(dead_code))]
    pub tcu_eeprom: Option<(String, Vec<u8>)>,
}

// ================================================================================================================
// Running a phase
// ================================================================================================================

/// What a check needs (owned: it runs on its own threads).
#[derive(Clone)]
pub struct CheckRequest {
    pub root: PathBuf,
    pub build: BenchConfig,
    /// ECU image under test and the reference (stock) image, files as in the session's image list.
    pub image: String,
    pub reference: Option<String>,
    pub immo_lab: bool,
    /// The DQ250's EEPROM as its firmware prepared it ((image id, bytes)); `None` = prepared by the check itself.
    pub tcu_eeprom: Option<(String, Vec<u8>)>,
    /// Phases to run (`None` = [`PROGRAM`]).
    pub phases: Option<Vec<&'static str>>,
    /// MultiMap slot (1..4) the driver selects before the start when the image carries the O30 Switch Patch with pedal
    /// selection (`UI button` = 3): `None` = the slot the ECU powers up with.
    pub mm_slot: Option<u8>,
}

/// Progress of a running check: per (image, phase) the fraction done.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Progress {
    pub step: String,
    pub parts: Vec<(String, f64)>,
}

/// Firmware DIDs read in rotation during a phase (`log_tools::channels`).
const FW_DIDS: &[&str] = &["Ign Avg", "Knock Cyl 1", "Knock Cyl 2", "Knock Cyl 3", "Knock Cyl 4", "PUT", "PUT SP", "Lambda SP", "FP DI SP", "IAT", "Misfires", "Pedal Pos", "Torque Req", "TPS", "STFT", "Inj PW MPI", "Lambda", "Turbo Speed"];

struct Probe {
    r: PhaseResult,
    t0: f64,
    damage0: [f64; 4],
    events0: [[u32; 4]; 4],
    di_seen: [f64; 4],
    clutch_bar_max: [f64; 2],
    di_acc: [f64; 4],
    acc_t: f64,
    next_sample: f64,
    fw: crate::studio::log_replay::Tester,
}

impl Probe {
    fn new(def: &PhaseDef, s: &StudioSession) -> Self {
        let t = s.plant.telemetry();
        let mut r = PhaseResult { id: def.id, ru: def.ru, what: def.what, ..Default::default() };
        for c in &mut r.cyl {
            c.lambda_min_load = f64::NAN;
            c.lambda_max_load = f64::NAN;
        }
        Self {
            r,
            t0: s.plant.sim_time_s,
            damage0: t.piston_damage,
            events0: std::array::from_fn(|c| [t.combustion.knock_events[c], t.combustion.preign_events[c], t.combustion.lspi_events[c], t.combustion.misfire_events[c]]),
            di_seen: [f64::NEG_INFINITY; 4],
            clutch_bar_max: [0.0; 2],
            di_acc: [0.0; 4],
            acc_t: 0.0,
            next_sample: 0.0,
            fw: crate::studio::log_replay::Tester::with(FW_DIDS),
        }
    }

    fn fw(&self, name: &str) -> f64 {
        self.fw.last(name).unwrap_or(f64::NAN)
    }

    /// After every 10 ms step.
    fn sample(&mut self, s: &mut StudioSession, dt: f64) {
        let t = s.plant.telemetry();
        let now = s.plant.sim_time_s - self.t0;
        self.fw.poll(s, s.plant.sim_time_s);
        let fw_knock: [f64; 4] = std::array::from_fn(|c| self.fw(&format!("Knock Cyl {}", c + 1)));
        let rail_sp = self.fw("FP DI SP") / 10.0; // bar → MPa
        let r = &mut self.r;
        let load = s.plant.driver.pedal_pct > 90.0 && t.map_kpa > 150.0;
        for (c, &k) in fw_knock.iter().enumerate() {
            let y = &mut r.cyl[c];
            y.p_max_bar = y.p_max_bar.max(t.cyl_peak_bar[c]);
            y.p_local_bar = y.p_local_bar.max(t.combustion.p_local_bar[c]);
            y.knock_max_bar = y.knock_max_bar.max(t.knock_amp_bar[c]);
            y.knock_cycles = t.combustion.knock_events[c] - self.events0[c][0];
            y.preign = t.combustion.preign_events[c] - self.events0[c][1];
            y.lspi = t.combustion.lspi_events[c] - self.events0[c][2];
            y.misfires = t.combustion.misfire_events[c] - self.events0[c][3];
            y.land_stress = y.land_stress.max(t.combustion.land_stress_ratio[c]);
            y.piston_damage = t.piston_damage[c] - self.damage0[c];
            y.crown_c = y.crown_c.max(t.piston_crown_c[c]);
            y.land_c = y.land_c.max(t.combustion.piston_land_c[c]);
            y.groove_c = y.groove_c.max(t.combustion.piston_groove_c[c]);
            y.exv_c = y.exv_c.max(t.exhaust_valve_c[c]);
            y.plug_c = y.plug_c.max(t.combustion.plug_tip_c[c]);
            // burning cylinders only (a cut or misfiring one reads air)
            if load && t.fuel_mg_stroke[c] > 1.0 && t.lambda_cyl[c] < 1.5 {
                y.lambda_min_load = y.lambda_min_load.min(t.lambda_cyl[c]);
                y.lambda_max_load = y.lambda_max_load.max(t.lambda_cyl[c]);
            }
            // DI injector duty: electrical pulse widths over 0.2 s windows (each injector serves one cylinder)
            if let Some(tm) = t.ecu_outputs.timed.as_ref() {
                for d in tm.di[c].iter().flatten() {
                    if d.off_s.is_finite() && d.on_s > self.di_seen[c] {
                        self.di_seen[c] = d.on_s;
                        self.di_acc[c] += d.off_s - d.on_s;
                    }
                }
            }
            if k.is_finite() {
                y.fw_knock_retard = y.fw_knock_retard.min(k);
            }
        }
        self.acc_t += dt;
        if self.acc_t >= 0.2 {
            for c in 0..4 {
                r.cyl[c].di_duty = r.cyl[c].di_duty.max(self.di_acc[c] / self.acc_t);
            }
            (self.di_acc, self.acc_t) = ([0.0; 4], 0.0);
        }
        r.rpm_max = r.rpm_max.max(t.rpm);
        r.kmh_max = r.kmh_max.max(t.roller_kmh);
        r.map_max_kpa = r.map_max_kpa.max(t.map_kpa);
        r.torque_max_nm = r.torque_max_nm.max(t.torque_brake_nm);
        r.power_max_kw = r.power_max_kw.max(t.power_kw);
        r.egt_max_c = r.egt_max_c.max(t.egt_c);
        if t.egt_c > limit("egt_c").value {
            r.egt_over_s += dt;
        }
        r.turbo_rpm_max = r.turbo_rpm_max.max(t.turbo_rpm);
        r.turbo_stress = r.turbo_stress.max(t.turbo_stress_ratio[0].max(t.turbo_stress_ratio[1]));
        r.iat_max_c = r.iat_max_c.max(t.iat_c);
        if load && rail_sp.is_finite() {
            r.rail_deficit_mpa = r.rail_deficit_mpa.max(rail_sp - t.rail_mpa);
        }
        // the clutch that carries the drive: engine torque against its static capacity while the TCU holds it at (≥ 90 % of)
        // the highest pressure it gave it in this phase (above 5 bar: a clutch meant to be closed, not a drive-off or
        // launch slip) and it still slips more than 100 rpm at full pedal
        for k in 0..2 {
            self.clutch_bar_max[k] = self.clutch_bar_max[k].max(t.clutch_bar[k]);
        }
        if load && t.driving_gear > 0 {
            let k = (t.clutch_bar[1] > t.clutch_bar[0]) as usize;
            let held = self.clutch_bar_max[k] > 5.0 && t.clutch_bar[k] >= 0.9 * self.clutch_bar_max[k];
            if held && t.clutch_capacity_nm[k] > 1.0 && t.clutch_slip_rpm[k].abs() > 100.0 {
                r.clutch_torque_ratio = r.clutch_torque_ratio.max(t.torque_brake_nm / t.clutch_capacity_nm[k]);
            }
        }
        r.clutch_surface_c = r.clutch_surface_c.max(t.clutch_surface_c[0].max(t.clutch_surface_c[1]));
        for f in t.failures.iter().chain(&t.driveline_failures) {
            if !r.failures.iter().any(|x| &x.1 == f) {
                r.failures.push((now, f.clone()));
            }
        }
        if now >= self.next_sample {
            self.next_sample = now + 0.1;
            let k = (t.clutch_bar[1] > t.clutch_bar[0]) as usize;
            let fw = |n: &str| self.fw.last(n).unwrap_or(f64::NAN);
            self.r.samples.push(Sample {
                t: now,
                rpm: t.rpm,
                kmh: t.roller_kmh,
                pedal: s.plant.driver.pedal_pct,
                brake: s.plant.driver.brake_pct,
                map_kpa: t.map_kpa,
                egt_c: t.egt_c,
                lambda: t.lambda_sensor,
                torque_nm: t.torque_brake_nm,
                p_max: t.cyl_peak_bar,
                knock: t.knock_amp_bar,
                land: t.combustion.land_stress_ratio,
                crown: t.piston_crown_c,
                clutch_slip_rpm: t.clutch_slip_rpm[k],
                fw_ign: fw("Ign Avg"),
                fw_knock: std::array::from_fn(|c| fw(&format!("Knock Cyl {}", c + 1))),
                fw_put: fw("PUT"),
                fw_put_sp: fw("PUT SP"),
                fw_lambda_sp: fw("Lambda SP"),
                fw_iat: fw("IAT"),
                fw_misfires: fw("Misfires"),
                fw_pedal: fw("Pedal Pos"),
                fw_tq_req: fw("Torque Req"),
                fw_tps: fw("TPS"),
                fw_stft: fw("STFT"),
                fw_mpi_ms: fw("Inj PW MPI"),
                fw_lambda: fw("Lambda"),
                fw_turbo_krpm: fw("Turbo Speed"),
                turbo_krpm: t.turbo_rpm / 1e3,
                air_mg: t.air_mg_stroke,
                fuel_mg: t.fuel_mg_stroke.iter().sum::<f64>() / 4.0,
                lambda_cyl: t.lambda_cyl,
                film_mg: std::array::from_fn(|c| (s.plant.engine.cyl[c].film_kg + s.plant.engine.cyl[c].liner_film_kg) * 1e6),
                di_soi_deg: std::array::from_fn(|c| t.ecu_outputs.injection[c][0].map_or(f64::NAN, |p| p.soi_btdc_deg)),
                di_ms: std::array::from_fn(|c| t.ecu_outputs.injection[c][0].map_or(f64::NAN, |p| p.width_ms)),
            });
        }
    }
}

/// The headless car of one phase.
struct Car<'a> {
    s: StudioSession,
    cancel: &'a AtomicBool,
    progress: &'a Mutex<Progress>,
    part: usize,
    expected_s: f64,
    t_start: f64,
}

impl Car<'_> {
    fn tick(&self) {
        if let Ok(mut p) = self.progress.lock() {
            if let Some(x) = p.parts.get_mut(self.part) {
                x.1 = ((self.s.plant.sim_time_s - self.t_start) / self.expected_s).clamp(0.0, 0.99);
            }
        }
    }

    /// Steps `dt` (10 ms) with the probe; `Err` when a board stopped or the check was cancelled.
    fn step(&mut self, probe: Option<&mut Probe>, dt: f64) -> Result<(), String> {
        if self.cancel.load(Ordering::Relaxed) {
            return Err("отменено".into());
        }
        self.s.step(dt);
        if let Some(e) = self.s.ecu_error.as_ref().or(self.s.tcu_error.as_ref().filter(|_| self.s.tcu.is_some())) {
            return Err(format!("плата остановилась: {e}"));
        }
        if let Some(p) = probe {
            p.sample(&mut self.s, dt);
        }
        if ((self.s.plant.sim_time_s * 10.0).round() as u64).is_multiple_of(10) {
            self.tick();
        }
        Ok(())
    }

    fn idle(&mut self, secs: f64) -> Result<(), String> {
        let t0 = self.s.plant.sim_time_s;
        while self.s.plant.sim_time_s - t0 < secs {
            self.step(None, 0.05)?;
        }
        Ok(())
    }

    /// Ignition, start button in P with the brake until the engine runs on its own (the ECU drives the starter), 3 s idle.
    fn start(&mut self) -> Result<(), String> {
        let d = &mut self.s.plant.driver;
        (d.ignition_on, d.selector, d.brake_pct, d.pedal_pct) = (true, Selector::P, 60.0, 0.0);
        self.idle(1.0)?;
        self.s.plant.driver.start_button = true;
        let mut running = false;
        for _ in 0..800 {
            self.step(None, 0.01)?;
            let t = self.s.plant.telemetry();
            if t.rpm > 600.0 && t.starter_current_a <= 0.0 {
                running = true;
                break;
            }
        }
        self.s.plant.driver.start_button = false;
        if !running {
            return Err(format!("двигатель не запустился за 8 с (ЭБУ {})", self.s.image_label));
        }
        self.idle(3.0)?;
        if self.s.plant.telemetry().rpm < 400.0 {
            return Err("двигатель заглох после пуска".into());
        }
        Ok(())
    }

    /// MultiMap slot selection of the O30 Switch Patch with pedal selection, as its driver does it
    /// (VERIFIED-S: `UI button` 0x8087BD08 = 3, switching logic 0x8013DE98): ignition on, engine stopped, accelerator ≥ 50 %
    /// held ≈ 2 s (200 task ticks; 10 ms per tick HYP) opens the selection, every brake press (≥ 5 ticks, released in
    /// between) moves the cursor one slot on (with the engine stopped it wraps after slot 4), ≈ 3 s without input apply
    /// it and store it in NVM. The driver reads the cursor on the tachometer; the bench reads the same from the patch RAM
    /// (slot index 0xD000F801, observation only). `None` when the image has no such patch.
    fn select_slot(&mut self, slot: u8) -> Result<Option<String>, String> {
        let ui = self.s.damos.get_by_id("UI button").map(|p| self.s.current_flash.get(p.file_offset as usize).copied());
        if self.s.current_flash.get(0x20_0002..0x20_0008) != Some(b"SC8O30".as_slice()) || ui != Some(Some(3)) {
            return Ok(None);
        }
        let idx = |s: &StudioSession| s.board.read_mem(0xD000_F801, 1).ok().and_then(|b| b.first().copied());
        let d = &mut self.s.plant.driver;
        (d.ignition_on, d.selector, d.brake_pct, d.pedal_pct) = (true, Selector::P, 0.0, 0.0);
        self.idle(2.0)?;
        let before = idx(&self.s).unwrap_or(0);
        let target = slot.clamp(1, 4) - 1;
        self.s.plant.driver.pedal_pct = 100.0;
        self.idle(2.5)?;
        self.s.plant.driver.pedal_pct = 0.0;
        self.idle(0.5)?;
        let presses = (target + 4 - before.min(3)) % 4;
        for _ in 0..presses {
            self.s.plant.driver.brake_pct = 60.0;
            self.idle(0.2)?;
            self.s.plant.driver.brake_pct = 0.0;
            self.idle(0.3)?;
        }
        // ≈ 3 s without input apply the cursor (300 ticks if 10 ms) — waited generously, the task period is not traced
        self.idle(10.0)?;
        let after = idx(&self.s);
        if after != Some(target) {
            return Err(format!("слот MultiMap {slot} не выбран: было {}, после {presses} нажатий тормоза {:?}", before + 1, after.map(|x| x + 1)));
        }
        // Key off for 2 s and on again (within the ECU's after-run: it keeps running, as on the car). A bench workaround,
        // logged in the notes: started straight after a pedal hold at key-on, the ECU — the stock SC8O40 as well, the hold
        // without the patch — leaves the throttle at 11–56 % and the boost set point at ambient at full pedal (torque
        // request 600 Nm; no fault code); after a 2 s key-off it boosts to the slot's 276 kPa, after 4 s it does not.
        // The reference log drove slot 3 at 270 kPa, so this is a bench divergence; its cause is not found (MILESTONE §5).
        let off_s = 2.0;
        self.s.plant.driver.ignition_on = false;
        self.idle(off_s)?;
        self.s.plant.driver.ignition_on = true;
        self.idle(2.0)?;
        let kept = idx(&self.s);
        if kept != Some(target) {
            return Err(format!("слот MultiMap {slot} не сохранился после выключения зажигания на {off_s:.0} с: {:?}", kept.map(|x| x + 1)));
        }
        Ok(Some(format!(
            "водитель выбрал слот MultiMap {slot} (патч O30: педаль 100 % 2.5 с при неработающем двигателе, нажатий тормоза {presses}; было {}), выключил зажигание на {off_s:.0} с и включил снова (обход расхождения стенда: без этого ЭБУ после удержания педали при включении зажигания не открывает дроссель и не даёт наддув — и сток тоже; причина не найдена)",
            before + 1
        )))
    }

    /// Fault memory over the session's tester: ECU (its own error table: OBD code, name) and TCU (decoded by its table).
    fn dtcs(&mut self, target: &str) -> Vec<String> {
        let r = UdsRequest { target: target.into(), service: "read_dtc".into(), data: Vec::new(), value: None, rid: None };
        if self.s.uds_request(&r).is_err() {
            return Vec::new();
        }
        for _ in 0..300 {
            if self.cancel.load(Ordering::Relaxed) {
                break;
            }
            self.s.step(0.01);
            if !self.status(target).pending {
                break;
            }
        }
        let st = self.status(target).clone();
        // with testFailed / pending / confirmed (status bits 0, 2, 3); the rest only say "test not completed"
        let fault = |d: &str| d.rsplit("0x").next().and_then(|x| u8::from_str_radix(x, 16).ok()).is_some_and(|s| s & 0x0D != 0);
        if target == "ecu" {
            let table = ecu_errors(&self.s);
            return st.dtcs.iter().filter(|d| fault(d)).map(|d| decode_ecu_dtc(d, &table)).collect();
        }
        st.dtcs.iter().enumerate().filter(|(_, d)| fault(d)).map(|(i, d)| format!("{} ({d})", st.dtc_sae.get(i).map_or("?", |x| x.as_str()))).collect()
    }

    fn status(&self, target: &str) -> &crate::studio::uds::UdsStatus {
        if target == "ecu" {
            &self.s.uds_ecu.status
        } else {
            &self.s.uds_tcu.status
        }
    }
}

/// ECU error names in IDX_ERR order with their OBD and customer DTCs, read from the image itself: the ERRM
/// name-string table (starts at "AEB_OC_0", one string per 4-byte-aligned slot, ends with "NO_ERROR") and the u16
/// DTC tables of the calibration (`cus_cal_off` = CAL offset of `c_err_dtc_cus_aeb_oc_0` from the image's DAMOS; the
/// OBD table ends `N` bytes of class codes before it). VERIFIED-S on SC8S50: the string order equals the DAMOS
/// `c_err_dtc_cus_*` order 1076/1076; on SC8O40 (1075 errors) the tables at CAL+0x23BC6 / +0x24860 give the same OBD /
/// customer DTC as SC8S50 for 1044 / 1036 of the 1047 error names both images contain. SC8O30: 1075 errors,
/// e.g. MIS_2 = P0304, MIS_IGC_2 = P33C2, INH_INJ_MIS = P130A (VERIFIED-S).
pub(crate) fn ecu_error_table(flash: &[u8], cus_cal_off: usize) -> Vec<(String, u16, u16)> {
    let Some(mut p) = flash.windows(9).position(|w| w == b"AEB_OC_0\0") else { return Vec::new() };
    let mut names = Vec::new();
    loop {
        let e = p + flash[p..].iter().position(|&b| b == 0).unwrap_or(0);
        let s = &flash[p..e];
        if s.is_empty() || !s.iter().all(|b| b.is_ascii_alphanumeric() || *b == b'_') || s == b"NO_ERROR" {
            break;
        }
        names.push(String::from_utf8_lossy(s).into_owned());
        p = (e + 1 + 3) & !3;
    }
    let cus = 0x20_0000 + cus_cal_off;
    let Some(obd) = cus.checked_sub(3 * names.len()).map(|x| x & !1) else { return Vec::new() };
    let rd = |a: usize| flash.get(a..a + 2).map_or(0, |b| u16::from_le_bytes([b[0], b[1]]));
    names.into_iter().enumerate().map(|(i, nm)| (nm, rd(obd + 2 * i), rd(cus + 2 * i))).collect()
}

/// ECU error table of the loaded image ([`ecu_error_table`] through its DAMOS).
pub(crate) fn ecu_errors(s: &StudioSession) -> Vec<(String, u16, u16)> {
    s.damos.parameters.iter().find(|p| p.id == "c_err_dtc_cus_aeb_oc_0").map(|p| ecu_error_table(&s.current_flash, (p.cpu_address - 0x8080_0000) as usize)).unwrap_or_default()
}

/// VW ECU DTC record "00CCCC status 0xSS" (customer DTC) → "P0304 MIS_2 (клиентский 3B1E, статус 0x2F)" by the image's tables.
fn decode_ecu_dtc(d: &str, table: &[(String, u16, u16)]) -> String {
    let code = d.get(..6).and_then(|x| u32::from_str_radix(x, 16).ok()).map(|x| (x & 0xFFFF) as u16);
    let st = d.split_once("status ").map_or("", |x| x.1);
    let names: Vec<String> = table.iter().filter(|e| Some(e.2) == code).map(|e| format!("{}{:04X} {}", ["P", "C", "B", "U"][(e.1 >> 14) as usize], e.1 & 0x3FFF, e.0)).collect();
    if names.is_empty() {
        format!("{d} (нет в таблице образа)")
    } else {
        format!("{} (клиентский {:04X}, статус {st})", names.join(" / "), code.unwrap_or(0))
    }
}

/// Runs one phase of `image` on a fresh car.
fn run_phase(req: &CheckRequest, image: &str, def: &PhaseDef, progress: &Mutex<Progress>, part: usize, cancel: &AtomicBool) -> PhaseResult {
    let mut build = req.build.clone();
    if let Some(a) = def.ambient_c {
        build.ambient.temp_c = a;
    }
    if let Some(r) = def.ron {
        build.fuel.ron = r;
    }
    let mut fallback = PhaseResult { id: def.id, ru: def.ru, what: def.what, ..Default::default() };
    let s = match new_car(req, &build, image) {
        Ok(s) => s,
        Err(e) => {
            fallback.error = Some(e);
            return fallback;
        }
    };
    let expected_s = 25.0 + def.soak_s + 8.0;
    let t_start = s.plant.sim_time_s;
    let mut car = Car { s, cancel, progress, part, expected_s, t_start };
    let mut notes = Vec::new();
    let res = (|| -> Result<Probe, String> {
        if let Some(slot) = req.mm_slot {
            notes.extend(car.select_slot(slot)?);
        }
        car.start()?;
        if def.soak_s > 0.0 {
            car.idle(def.soak_s)?;
        }
        let mut p = Probe::new(def, &car.s);
        match def.kind {
            Kind::DynoSweep => dyno_sweep(&mut car, &mut p)?,
            Kind::Pull => pull(&mut car, &mut p)?,
            Kind::Launch => launch(&mut car, &mut p)?,
        }
        // lift off, stop, P — then the fault memories
        let d = &mut car.s.plant.driver;
        (d.pedal_pct, d.brake_pct) = (0.0, 40.0);
        car.s.plant.bench = BenchMode::RoadLoadDyno;
        for _ in 0..300 {
            car.step(Some(&mut p), 0.01)?;
            if car.s.plant.telemetry().roller_kmh < 0.5 {
                break;
            }
        }
        car.s.plant.driver.selector = Selector::P;
        car.idle(1.0)?;
        // after a failure the driver lets the engine idle (the firmware's misfire / cylinder-shut-off monitors need
        // engine cycles at a steady state to judge a damaged cylinder; P130A / P33C2 in the reference log were set at idle)
        if !p.r.failures.is_empty() {
            let t0 = car.s.plant.sim_time_s;
            while car.s.plant.sim_time_s - t0 < 25.0 && car.s.plant.telemetry().rpm > 300.0 {
                car.step(Some(&mut p), 0.01)?;
            }
        }
        Ok(p)
    })();
    match res {
        Ok(mut p) => {
            p.r.notes = notes;
            p.r.ecu_dtcs = car.dtcs("ecu");
            if car.s.tcu.is_some() {
                p.r.tcu_dtcs = car.dtcs("tcu");
            }
            p.r.sim_s = car.s.plant.sim_time_s - car.t_start;
            p.r
        }
        Err(e) => {
            fallback.error = Some(e);
            fallback.notes = notes;
            fallback.sim_s = car.s.plant.sim_time_s - car.t_start;
            fallback
        }
    }
}

/// A fresh warm car of the build with `image` loaded (and the DQ250's prepared EEPROM).
fn new_car(req: &CheckRequest, build: &BenchConfig, image: &str) -> Result<StudioSession, String> {
    let state = crate::studio::bench_build::BuildState::from_config(build.clone());
    let mut s = StudioSession::with_build(&req.root, state)?;
    s.warm_car();
    s.plant.bench = BenchMode::RoadLoadDyno;
    s.immo_lab = req.immo_lab;
    s.load_image(image)?;
    if let Some((id, mem)) = &req.tcu_eeprom {
        if s.tcu.as_ref().and_then(TcuBoard::nvm).is_some_and(|n| n.0 == id.as_str()) {
            s.tcu_nvm.mem.insert(id.clone(), mem.clone());
            s.power_cycle_tcu_fresh();
        }
    }
    Ok(s)
}

/// Engine dyno: absorber holds 1500 rpm, pedal 100 % for 3 s, then ramps 250 rpm/s to 6500 rpm (or as far as the
/// engine pulls it: the firmware limits speed), 2 s at the end.
fn dyno_sweep(car: &mut Car, p: &mut Probe) -> Result<(), String> {
    car.s.plant.bench = BenchMode::EngineDyno { target_rpm: 1500.0 };
    car.s.plant.driver.selector = Selector::P;
    car.s.plant.driver.brake_pct = 0.0;
    car.s.plant.driver.pedal_pct = 100.0;
    let (mut target, mut t) = (1500.0f64, 0.0);
    while t < 25.0 {
        if t > 3.0 {
            target = (target + 250.0 * 0.01).min(6500.0);
        }
        car.s.plant.bench = BenchMode::EngineDyno { target_rpm: target };
        car.step(Some(p), 0.01)?;
        t += 0.01;
    }
    car.s.plant.driver.pedal_pct = 0.0;
    car.s.plant.bench = BenchMode::EngineDyno { target_rpm: 1000.0 };
    car.idle(3.0)
}

/// Road-load dyno: D with the brake 2 s, release, 25 % pedal to 30 km/h, then 100 % to 160 km/h or 25 s.
fn pull(car: &mut Car, p: &mut Probe) -> Result<(), String> {
    let d = &mut car.s.plant.driver;
    (d.selector, d.brake_pct, d.pedal_pct) = (Selector::D, 60.0, 0.0);
    car.idle(2.0)?;
    car.s.plant.driver.brake_pct = 0.0;
    car.s.plant.driver.pedal_pct = 25.0;
    let t0 = car.s.plant.sim_time_s;
    while car.s.plant.telemetry().roller_kmh < 30.0 && car.s.plant.sim_time_s - t0 < 20.0 {
        car.step(Some(p), 0.01)?;
    }
    car.s.plant.driver.pedal_pct = 100.0;
    let t1 = car.s.plant.sim_time_s;
    while car.s.plant.telemetry().roller_kmh < 160.0 && car.s.plant.sim_time_s - t1 < 25.0 {
        car.step(Some(p), 0.01)?;
    }
    Ok(())
}

/// Road-load dyno: selector S, brake 100 % and pedal 100 % for 3 s, brake released at once, pedal 100 % to 140 km/h or
/// 15 s. Whatever the firmware does with it: the DQ250 0D9300012F/0D9300011R launch mode arms only in S or M with the brake
/// above 40 bar and then holds the engine near its launch speed by torque requests to the ECU (static RE, VERIFIED-S:
/// state machine 0xD0004175, `FUN_80093C64`; 4930: 3000 rpm at 100 % pedal); an ECU's own launch control would act here
/// too; a gearbox without a launch mode slips its clutch against the brakes.
fn launch(car: &mut Car, p: &mut Probe) -> Result<(), String> {
    let d = &mut car.s.plant.driver;
    (d.selector, d.brake_pct, d.pedal_pct) = (Selector::S, 100.0, 0.0);
    car.idle(2.0)?;
    car.s.plant.driver.pedal_pct = 100.0;
    for _ in 0..300 {
        car.step(Some(p), 0.01)?;
    }
    p.r.launch_hold_rpm = Some(car.s.plant.telemetry().rpm);
    p.r.launch_hold_ign = Some(p.fw("Ign Avg")).filter(|x| x.is_finite());
    car.s.plant.driver.brake_pct = 0.0;
    let t1 = car.s.plant.sim_time_s;
    while car.s.plant.telemetry().roller_kmh < 140.0 && car.s.plant.sim_time_s - t1 < 15.0 {
        car.step(Some(p), 0.01)?;
    }
    Ok(())
}

// ================================================================================================================
// Verdict
// ================================================================================================================

/// Time of the first sample of `p` where `f` holds (s since the phase start), else the phase end.
fn when(p: &PhaseResult, f: impl Fn(&Sample) -> bool) -> f64 {
    p.samples.iter().find(|s| f(s)).or(p.samples.last()).map_or(0.0, |s| s.t)
}

fn judge_phase(p: &mut PhaseResult) {
    let mut out = Vec::new();
    let mut add = |level: Verdict, t_s: f64, cyl: Option<usize>, lim: Option<&'static str>, text: String| {
        out.push(Reason { level, phase: p.ru.to_string(), t_s, cyl, limit: lim, text });
    };
    if let Some(e) = &p.error {
        add(Verdict::Risk, p.sim_s, None, None, format!("фаза не выполнена до конца: {e}"));
    }
    for (t, f) in &p.failures {
        add(Verdict::Breaks, *t, cyl_of(f), None, format!("отказ в физической модели: {f}"));
    }
    for c in 0..4 {
        let y = &p.cyl[c];
        let n = c + 1;
        let lf = limit("land_fracture");
        if y.land_stress >= lf.value {
            add(Verdict::Breaks, when(p, |s| s.land[c] >= lf.value), Some(n), Some(lf.id), format!("цилиндр {n}: напряжение в перемычке поршня {:.2} от предела прочности — излом за одно событие", y.land_stress));
        } else if y.land_stress > limit("land_fatigue").value {
            let l = limit("land_fatigue");
            add(Verdict::Risk, when(p, |s| s.land[c] > l.value), Some(n), Some(l.id), format!("цилиндр {n}: напряжение в перемычке {:.2} от предела прочности — выше предела выносливости, перемычка накапливает усталость", y.land_stress));
        }
        let pd = limit("piston_damage");
        if y.piston_damage > pd.value {
            add(Verdict::Risk, p.sim_s, Some(n), Some(pd.id), format!("цилиндр {n}: повреждение перемычки за фазу {:.1e} — ресурс ≈ {:.0} повторов", y.piston_damage, 1.0 / y.piston_damage));
        }
        let pm = limit("p_max");
        if y.p_max_bar > pm.value {
            add(Verdict::Risk, when(p, |s| s.p_max[c] > pm.value), Some(n), Some(pm.id), format!("цилиндр {n}: пиковое давление {:.0} бар (предел {:.0})", y.p_max_bar, pm.value));
        }
        let kn = limit("knock");
        if y.knock_max_bar > kn.value {
            add(Verdict::Risk, when(p, |s| s.knock[c] > kn.value), Some(n), Some(kn.id), format!("цилиндр {n}: детонация до {:.0} бар, циклов с детонацией {} (ретард прошивки до {:.1}°)", y.knock_max_bar, y.knock_cycles, y.fw_knock_retard));
        }
        if y.preign > 0 {
            add(Verdict::Risk, p.sim_s, Some(n), Some("preign"), format!("цилиндр {n}: предвоспламенений {} (из них LSPI {}), местное давление до {:.0} бар", y.preign, y.lspi, y.p_local_bar));
        }
        for (id, v) in [("crown_c", y.crown_c), ("groove_c", y.groove_c), ("exv_c", y.exv_c), ("plug_c", y.plug_c)] {
            let l = limit(id);
            if v > l.value {
                add(Verdict::Risk, p.sim_s, Some(n), Some(l.id), format!("цилиндр {n}: {} {:.0} °C (предел {:.0})", l.ru.to_lowercase(), v, l.value));
            }
        }
        let ll = limit("lambda_lean");
        if y.lambda_max_load > ll.value {
            add(Verdict::Risk, p.sim_s, Some(n), Some(ll.id), format!("цилиндр {n}: λ до {:.2} на полной нагрузке", y.lambda_max_load));
        }
        let dd = limit("di_duty");
        if y.di_duty > dd.value {
            add(Verdict::Risk, p.sim_s, Some(n), Some(dd.id), format!("цилиндр {n}: скважность форсунки DI {:.0} %", 100.0 * y.di_duty));
        }
    }
    let le = limit("egt_c");
    if p.egt_over_s > 5.0 {
        add(Verdict::Risk, p.sim_s, None, Some(le.id), format!("газ перед турбиной выше {:.0} °C в сумме {:.1} с (макс. {:.0} °C)", le.value, p.egt_over_s, p.egt_max_c));
    }
    for (id, v, unit) in [("turbo", p.turbo_stress, ""), ("rail_deficit", p.rail_deficit_mpa, "МПа"), ("clutch_torque", p.clutch_torque_ratio, ""), ("clutch_c", p.clutch_surface_c, "°C")] {
        let l = limit(id);
        if v > l.value {
            add(Verdict::Risk, p.sim_s, None, Some(l.id), format!("{}: {} {unit} (предел {} {unit})", l.ru, fmt_num(v), fmt_num(l.value)));
        }
    }
    p.verdict = Some(out.iter().map(|r| r.level).max().unwrap_or(Verdict::Ok));
    p.reasons = out;
}

/// Cylinder named in a plant failure text ("cylinder 4 ...").
fn cyl_of(f: &str) -> Option<usize> {
    let i = f.find("cylinder ")? + 9;
    f[i..].chars().next()?.to_digit(10).map(|d| d as usize)
}

/// Fault codes the firmware set in a phase that the reference image did not set in the same phase (by code text before
/// the bracket); without a reference every code is listed but not judged.
fn new_dtcs(p: &PhaseResult, reference: Option<&PhaseResult>) -> Vec<String> {
    let Some(q) = reference else { return Vec::new() };
    let key = |d: &String| d.split(" (").next().unwrap_or(d).to_string();
    let known: Vec<String> = q.ecu_dtcs.iter().chain(&q.tcu_dtcs).map(key).collect();
    p.ecu_dtcs.iter().chain(&p.tcu_dtcs).filter(|d| !known.contains(&key(d))).cloned().collect()
}

fn judge_image(r: &mut ImageResult, reference: Option<&ImageResult>) {
    for p in &mut r.phases {
        judge_phase(p);
        let q = reference.and_then(|x| x.phases.iter().find(|q| q.id == p.id));
        for d in new_dtcs(p, q) {
            p.reasons.push(Reason { level: Verdict::Risk, phase: p.ru.to_string(), t_s: p.sim_s, cyl: None, limit: Some("dtc"), text: format!("прошивка записала ошибку, которой нет у эталона: {d}") });
            p.verdict = Some(p.verdict.unwrap_or(Verdict::Ok).max(Verdict::Risk));
        }
    }
    r.reasons = r.phases.iter().flat_map(|p| p.reasons.iter().cloned()).collect();
    r.reasons.sort_by_key(|b| std::cmp::Reverse(b.level));
    r.verdict = Some(r.reasons.iter().map(|x| x.level).max().unwrap_or(Verdict::Ok));
}

// ================================================================================================================
// Check
// ================================================================================================================

/// Runs the whole check (both images, all phases in parallel threads). Blocking.
pub fn run(req: &CheckRequest, progress: &Mutex<Progress>, cancel: &AtomicBool) -> Report {
    let wall = std::time::Instant::now();
    let mut notes = Vec::new();
    let mut req = req.clone();
    // DQ250 without a prepared EEPROM: the firmware prepares it first (studio::dq250_prep), as the studio does
    if req.tcu_eeprom.is_none() && matches!(req.build.gearbox, simemu_physics::bench_config::Gearbox::Dq250_0d9) {
        set_step(progress, "подготовка мехатроника DQ250 (кодирование и базовые установки — прошивка ТКУ)");
        let p = Mutex::new(crate::studio::dq250_prep::Progress::default());
        let pr = crate::studio::dq250_prep::PrepRequest { root: req.root.clone(), build: req.build.clone(), ecu_image: req.reference.clone().unwrap_or(req.image.clone()), immo_lab: req.immo_lab };
        let (mem, rec) = crate::studio::dq250_prep::run(&pr, &p, cancel);
        notes.push(rec.summary());
        req.tcu_eeprom = mem.map(|m| (rec.image_id.clone(), m));
    }
    let phases: Vec<&'static PhaseDef> = PROGRAM.iter().filter(|d| req.phases.as_ref().is_none_or(|v| v.contains(&d.id))).collect();
    let images: Vec<String> = std::iter::once(req.image.clone()).chain(req.reference.clone()).collect();
    if let Ok(mut pg) = progress.lock() {
        pg.step = "прогон программы".into();
        pg.parts = images.iter().flat_map(|im| phases.iter().map(move |d| (format!("{im} · {}", d.ru), 0.0))).collect();
    }
    // all (image, phase) cars in parallel, as many at a time as half the cores (each car runs an ECU and a TCU thread)
    let jobs: Vec<(usize, &str, &PhaseDef)> = images.iter().enumerate().flat_map(|(i, im)| phases.iter().map(move |d| (i, im.as_str(), *d))).collect();
    let width = std::thread::available_parallelism().map_or(4, |n| (n.get() / 2).max(1));
    let mut done: Vec<Option<PhaseResult>> = vec![None; jobs.len()];
    for (chunk_i, chunk) in jobs.chunks(width).enumerate() {
        std::thread::scope(|sc| {
            let hs: Vec<_> = chunk
                .iter()
                .enumerate()
                .map(|(k, (_, im, d))| {
                    let part = chunk_i * width + k;
                    let req = &req;
                    sc.spawn(move || run_phase(req, im, d, progress, part, cancel))
                })
                .collect();
            for (k, h) in hs.into_iter().enumerate() {
                done[chunk_i * width + k] = h.join().ok();
            }
        });
    }
    let mut results: Vec<ImageResult> = images.iter().map(|im| ImageResult { image: im.clone(), ..Default::default() }).collect();
    for (j, (i, _, d)) in jobs.iter().enumerate() {
        let r = done[j].take().unwrap_or_else(|| PhaseResult { id: d.id, ru: d.ru, what: d.what, error: Some("поток фазы аварийно завершился".into()), ..Default::default() });
        results[*i].phases.push(r);
    }
    for r in &mut results {
        r.label = label_of(&req.root, &r.image);
    }
    if results.len() > 1 {
        judge_image(&mut results[1], None);
        let (a, b) = results.split_at_mut(1);
        judge_image(&mut a[0], Some(&b[0]));
    } else {
        judge_image(&mut results[0], None);
    }
    let reference = (results.len() > 1).then(|| results.pop()).flatten();
    let tested = results.pop().expect("tested image");
    let verdict = tested.verdict.unwrap_or(Verdict::Ok);
    let summary = summary(&tested, reference.as_ref());
    let sim_s = tested.phases.iter().chain(reference.iter().flat_map(|r| r.phases.iter())).map(|p| p.sim_s).sum();
    Report {
        build: req.build.name.clone(),
        tcu_image: req.build.tcu_image.clone(),
        tested,
        reference,
        verdict,
        summary,
        limits: LIMITS,
        program: &PROGRAM,
        notes,
        sim_s,
        wall_s: wall.elapsed().as_secs_f64(),
        unix_s: std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs()),
        tcu_eeprom: req.tcu_eeprom.clone(),
    }
}

fn set_step(progress: &Mutex<Progress>, step: &str) {
    if let Ok(mut p) = progress.lock() {
        p.step = step.into();
    }
}

fn label_of(root: &Path, file: &str) -> String {
    let code = crate::paths::read_firmware(root, file).ok().and_then(|f| f.get(0x200002..0x200008).map(|b| String::from_utf8_lossy(b).into_owned())).unwrap_or_default();
    format!("{code} ({file})")
}

/// Russian summary: verdict, the worst reasons, the comparison with the reference.
fn summary(t: &ImageResult, r: Option<&ImageResult>) -> String {
    let v = t.verdict.unwrap_or(Verdict::Ok);
    let mut s = format!("Вердикт: {} — {}.", v.ru(), t.label);
    let worst: Vec<String> = t.reasons.iter().filter(|x| x.level == v && v != Verdict::Ok).take(4).map(|x| format!("{} ({}, {:.1} с)", x.text, x.phase, x.t_s)).collect();
    if !worst.is_empty() {
        let _ = write!(s, " Причины: {}.", worst.join("; "));
    }
    if let Some(r) = r {
        let _ = write!(s, " Эталон {}: {}.", r.label, r.verdict.unwrap_or(Verdict::Ok).ru());
    }
    s
}

/// Stock image of the same part number for `image` (`vault/firmwares/ecu/{stock,frf}/`, highest index), if there is one.
pub fn default_reference(root: &Path, image: &str) -> Option<String> {
    let part_from_cal = |cal: &[u8]| cal.get(0x9E..0xA8).map(|b| String::from_utf8_lossy(b).into_owned());
    let me_path = crate::paths::resolve_firmware(root, image)?;
    let me_canon = std::fs::canonicalize(&me_path).unwrap_or(me_path.clone());
    let me = crate::paths::peek_ecu_cal_header(&me_path, 0xB0).and_then(|c| part_from_cal(&c))?;
    let mut v: Vec<String> = Vec::new();
    for sub in [
        "vault/firmwares/ecu/stock",
        "vault/firmwares/ecu/frf",
        "firmwares/ecu/stock",
        "firmwares/ecu/frf",
        "ecu/stock",
        "ecu/frf",
    ] {
        for (name, path) in crate::paths::list_ecu_images_in_subdir(root, sub) {
            let canon = std::fs::canonicalize(&path).unwrap_or_else(|_| path.clone());
            if canon == me_canon || v.contains(&name) {
                continue;
            }
            if crate::paths::peek_ecu_cal_header(&path, 0xB0)
                .and_then(|c| part_from_cal(&c))
                .is_some_and(|p| p == me)
            {
                v.push(name);
            }
        }
    }
    v.sort_by(|a, b| {
        let key_a = a.strip_prefix("FL_").or_else(|| a.strip_prefix("fl_")).unwrap_or(a);
        let key_b = b.strip_prefix("FL_").or_else(|| b.strip_prefix("fl_")).unwrap_or(b);
        key_a.cmp(key_b).then_with(|| a.ends_with(".bin").cmp(&b.ends_with(".bin")))
    });
    v.pop()
}

// ================================================================================================================
// HTML
// ================================================================================================================

fn esc(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

pub fn html(r: &Report) -> String {
    let mut b = String::new();
    let cls = |v: Verdict| match v {
        Verdict::Ok => "ok",
        Verdict::Risk => "risk",
        Verdict::Breaks => "brk",
    };
    let _ = write!(b, r#"<p class="verdict {}">{}</p><p>{}</p>"#, cls(r.verdict), esc(r.verdict.ru()), esc(&r.summary));
    let _ = write!(
        b,
        "<p>Сборка стенда: <b>{}</b>; ТКУ: {}; модель {} с за {} с.</p>",
        esc(&r.build),
        esc(r.tcu_image.as_deref().unwrap_or("по сборке")),
        fmt_num(r.sim_s),
        fmt_num(r.wall_s)
    );
    for n in &r.notes {
        let _ = write!(b, "<p class=\"note\">{}</p>", esc(n));
    }
    b.push_str("<h2>Программа</h2><table><tr><th>Фаза</th><th>Что делается</th></tr>");
    for d in r.program {
        let _ = write!(b, "<tr><td>{}</td><td>{}</td></tr>", esc(d.ru), esc(d.what));
    }
    b.push_str("</table>");
    for (title, im) in std::iter::once(("Эта прошивка", Some(&r.tested))).chain(std::iter::once(("Эталон (сток)", r.reference.as_ref()))) {
        let Some(im) = im else { continue };
        let _ = write!(b, "<h2>{title}: {} — <span class=\"{}\">{}</span></h2>", esc(&im.label), cls(im.verdict.unwrap_or(Verdict::Ok)), im.verdict.unwrap_or(Verdict::Ok).ru());
        if !im.reasons.is_empty() {
            b.push_str("<table><tr><th>Уровень</th><th>Фаза</th><th>t, с</th><th>Цил.</th><th>Причина</th></tr>");
            for x in &im.reasons {
                let _ = write!(b, "<tr class=\"{}\"><td>{}</td><td>{}</td><td>{:.1}</td><td>{}</td><td>{}</td></tr>", cls(x.level), x.level.ru(), esc(&x.phase), x.t_s, x.cyl.map_or("—".into(), |c| c.to_string()), esc(&x.text));
            }
            b.push_str("</table>");
        }
    }
    b.push_str("<h2>Сток против этой прошивки</h2>");
    b.push_str(&compare_table(r));
    b.push_str("<h2>По фазам и цилиндрам</h2>");
    for (k, p) in r.tested.phases.iter().enumerate() {
        let refp = r.reference.as_ref().and_then(|x| x.phases.get(k));
        let _ = write!(b, "<h3>{}</h3><p>{}</p>", esc(p.ru), esc(p.what));
        if let Some(e) = &p.error {
            let _ = write!(b, "<p class=\"risk\">Не выполнено до конца: {}</p>", esc(e));
        }
        b.push_str(&cyl_table(p, refp));
        let t1 = p.samples.last().map_or(1.0, |s| s.t);
        let ts: Vec<f64> = p.samples.iter().map(|s| s.t).collect();
        let col = |f: &dyn Fn(&Sample) -> f64| p.samples.iter().map(f).collect::<Vec<f64>>();
        let colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"];
        let rpm = col(&|s| s.rpm);
        let kmh = col(&|s| s.kmh * 10.0);
        let map = col(&|s| s.map_kpa);
        b.push_str(&svg_plot("Обороты, скорость ×10, MAP", "об/мин, км/ч, кПа", &[Series { label: "об/мин", t: &ts, y: &rpm, color: colors[0] }, Series { label: "км/ч ×10", t: &ts, y: &kmh, color: colors[2] }, Series { label: "MAP кПа", t: &ts, y: &map, color: colors[1] }], &[], (0.0, t1)));
        let pk: Vec<Vec<f64>> = (0..4).map(|c| col(&|s| s.p_max[c])).collect();
        let kn: Vec<Vec<f64>> = (0..4).map(|c| col(&|s| s.knock[c])).collect();
        let ld: Vec<Vec<f64>> = (0..4).map(|c| col(&|s| s.land[c])).collect();
        let names = ["цил 1", "цил 2", "цил 3", "цил 4"];
        for (title, unit, data) in [("Пиковое давление", "бар", &pk), ("Амплитуда детонации", "бар", &kn), ("Перемычка: σ / предел прочности", "—", &ld)] {
            let series: Vec<Series> = (0..4).map(|c| Series { label: names[c], t: &ts, y: &data[c], color: colors[c] }).collect();
            b.push_str(&svg_plot(title, unit, &series, &[], (0.0, t1)));
        }
        let ign = col(&|s| s.fw_ign);
        let fk: Vec<Vec<f64>> = (0..4).map(|c| col(&|s| s.fw_knock[c])).collect();
        let mut series = vec![Series { label: "УОЗ", t: &ts, y: &ign, color: "#444" }];
        series.extend((0..4).map(|c| Series { label: names[c], t: &ts, y: &fk[c], color: colors[c] }));
        b.push_str(&svg_plot("Прошивка (UDS): УОЗ и ретард детонации по цилиндрам", "°", &series, &[Span { t0: 0.0, t1: 0.0, label: "" }], (0.0, t1)));
        if !p.ecu_dtcs.is_empty() || !p.tcu_dtcs.is_empty() {
            let _ = write!(b, "<p>Ошибки ЭБУ: {}<br>Ошибки ТКУ: {}</p>", esc(&p.ecu_dtcs.join(", ")), esc(&p.tcu_dtcs.join(", ")));
        }
    }
    b.push_str("<h2>Пределы и их источники</h2><table><tr><th>Величина</th><th>Предел</th><th>Значение</th><th>Источник</th></tr>");
    for l in r.limits {
        let _ = write!(b, "<tr><td>{}</td><td>{} {}</td><td>{}</td><td>{}</td></tr>", esc(l.ru), fmt_num(l.value), esc(l.unit), if l.breaks { "сломает" } else { "риск" }, esc(l.source));
    }
    b.push_str("</table>");
    let style = "<style>.verdict{font-size:1.6em;font-weight:bold}.ok{color:#2a7a2a}.risk{color:#b07000}.brk{color:#c00000}tr.brk td{background:#fde4e4}tr.risk td{background:#fff4dc}.note{color:#555}</style>";
    html_page(&format!("Проверка прошивки — {}", r.tested.label), &format!("{style}{b}"))
}

fn compare_table(r: &Report) -> String {
    let mut b = String::from("<table><tr><th>Фаза</th><th>Величина</th><th>Сток</th><th>Эта прошивка</th></tr>");
    for (k, p) in r.tested.phases.iter().enumerate() {
        let q = r.reference.as_ref().and_then(|x| x.phases.get(k));
        let row = |name: &str, f: &dyn Fn(&PhaseResult) -> f64| format!("<tr><td>{}</td><td>{name}</td><td>{}</td><td>{}</td></tr>", esc(p.ru), q.map_or("—".into(), |q| fmt_num(f(q))), fmt_num(f(p)));
        fn mx(x: &PhaseResult, f: fn(&Cyl) -> f64) -> f64 {
            x.cyl.iter().map(f).fold(0.0, f64::max)
        }
        b += &row("макс. момент, Нм", &|x| x.torque_max_nm);
        b += &row("макс. мощность, кВт", &|x| x.power_max_kw);
        b += &row("макс. MAP, кПа", &|x| x.map_max_kpa);
        b += &row("макс. обороты", &|x| x.rpm_max);
        b += &row("пиковое давление, бар", &|x| mx(x, |c| c.p_max_bar));
        b += &row("детонация, бар", &|x| mx(x, |c| c.knock_max_bar));
        b += &row("предвоспламенения", &|x| mx(x, |c| c.preign as f64));
        b += &row("перемычка σ/предел", &|x| mx(x, |c| c.land_stress));
        b += &row("повреждение перемычки", &|x| mx(x, |c| c.piston_damage));
        b += &row("днище поршня, °C", &|x| mx(x, |c| c.crown_c));
        b += &row("газ перед турбиной, °C", &|x| x.egt_max_c);
        b += &row("момент / ёмкость сцепления", &|x| x.clutch_torque_ratio);
    }
    b.push_str("</table>");
    b
}

fn cyl_table(p: &PhaseResult, q: Option<&PhaseResult>) -> String {
    let mut b = String::from("<table><tr><th>Величина</th><th>цил 1</th><th>цил 2</th><th>цил 3</th><th>цил 4</th><th>сток (макс.)</th></tr>");
    let mut row = |name: &str, f: &dyn Fn(&Cyl) -> f64| {
        let _ = write!(b, "<tr><td>{name}</td>");
        for c in &p.cyl {
            let _ = write!(b, "<td>{}</td>", fmt_num(f(c)));
        }
        let refv = q.map(|q| q.cyl.iter().map(f).fold(f64::MIN, f64::max));
        let _ = write!(b, "<td>{}</td></tr>", refv.map_or("—".into(), fmt_num));
    };
    row("пиковое давление, бар", &|c| c.p_max_bar);
    row("местное давление (с детонацией), бар", &|c| c.p_local_bar);
    row("детонация, бар", &|c| c.knock_max_bar);
    row("циклов с детонацией", &|c| c.knock_cycles as f64);
    row("предвоспламенений", &|c| c.preign as f64);
    row("из них LSPI", &|c| c.lspi as f64);
    row("пропусков воспламенения (физика)", &|c| c.misfires as f64);
    row("перемычка σ / предел прочности", &|c| c.land_stress);
    row("повреждение перемычки (Майнер)", &|c| c.piston_damage);
    row("днище поршня, °C", &|c| c.crown_c);
    row("канавка 1-го кольца, °C", &|c| c.groove_c);
    row("выпускной клапан, °C", &|c| c.exv_c);
    row("тепловой конус свечи, °C", &|c| c.plug_c);
    row("λ на нагрузке мин", &|c| c.lambda_min_load);
    row("λ на нагрузке макс", &|c| c.lambda_max_load);
    row("скважность DI", &|c| c.di_duty);
    row("ретард детонации прошивки, °", &|c| c.fw_knock_retard);
    b.push_str("</table>");
    let _ = write!(
        b,
        "<p>Двигатель: макс. {} об/мин, {} км/ч, MAP {} кПа, газ перед турбиной {} °C, турбина {} об/мин (σ/предел {}), IAT {} °C, дефицит рампы {} МПа, момент/ёмкость сцепления {}, поверхность сцепления {} °C{}{}.</p>",
        fmt_num(p.rpm_max),
        fmt_num(p.kmh_max),
        fmt_num(p.map_max_kpa),
        fmt_num(p.egt_max_c),
        fmt_num(p.turbo_rpm_max),
        fmt_num(p.turbo_stress),
        fmt_num(p.iat_max_c),
        fmt_num(p.rail_deficit_mpa),
        fmt_num(p.clutch_torque_ratio),
        fmt_num(p.clutch_surface_c),
        p.launch_hold_rpm.map_or(String::new(), |r| format!("; на тормозе с полной педалью {} об/мин", fmt_num(r))),
        p.launch_hold_ign.map_or(String::new(), |r| format!(", УОЗ {}°", fmt_num(r))),
    );
    b
}

// ================================================================================================================
// Session side
// ================================================================================================================

struct Job {
    handle: JoinHandle<Report>,
    progress: Arc<Mutex<Progress>>,
    cancel: Arc<AtomicBool>,
}

/// The running check and the last report.
#[derive(Default)]
pub struct Check {
    job: Option<Job>,
    pub report: Option<Arc<Report>>,
    /// Stock image of the loaded image's part number ([`default_reference`]), set at image load.
    pub default_reference: Option<String>,
}

/// `/api/state` view.
#[derive(Serialize)]
pub struct CheckView {
    pub running: bool,
    pub default_reference: Option<String>,
    pub progress: Option<Progress>,
    pub verdict: Option<Verdict>,
    pub summary: Option<String>,
}

impl Check {
    pub fn view(&self) -> CheckView {
        CheckView {
            running: self.job.is_some(),
            default_reference: self.default_reference.clone(),
            progress: self.job.as_ref().and_then(|j| j.progress.lock().ok().map(|p| p.clone())),
            verdict: self.report.as_ref().map(|r| r.verdict),
            summary: self.report.as_ref().map(|r| r.summary.clone()),
        }
    }

    pub fn start(&mut self, req: CheckRequest) -> Result<(), String> {
        self.cancel();
        let progress = Arc::new(Mutex::new(Progress::default()));
        let cancel = Arc::new(AtomicBool::new(false));
        let (p, c) = (Arc::clone(&progress), Arc::clone(&cancel));
        let handle = std::thread::Builder::new()
            .name("tune-check".into())
            .spawn(move || run(&req, &p, &c))
            .map_err(|e| format!("spawn tune-check thread: {e}"))?;
        self.job = Some(Job { handle, progress, cancel });
        Ok(())
    }

    pub fn cancel(&mut self) {
        if let Some(j) = self.job.take() {
            j.cancel.store(true, Ordering::Relaxed);
        }
    }

    /// Collects a finished check.
    pub fn poll(&mut self) {
        if self.job.as_ref().is_some_and(|j| j.handle.is_finished()) {
            if let Some(j) = self.job.take() {
                if let Ok(r) = j.handle.join() {
                    self.report = Some(Arc::new(r));
                }
            }
        }
    }
}

impl Drop for Check {
    fn drop(&mut self) {
        self.cancel();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::studio::session::S50_FLASH;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    /// The verdict and the reasons follow the limits: a phase with a fractured land «сломает», one above a risk limit
    /// «риск», a clean one «OK»; the image's DTC table decodes a customer DTC.
    #[test]
    fn verdict_follows_the_limits() {
        let mut p = PhaseResult { ru: "тест", ..Default::default() };
        judge_phase(&mut p);
        assert_eq!(p.verdict, Some(Verdict::Ok));
        p.cyl[3].knock_max_bar = 7.0;
        judge_phase(&mut p);
        assert_eq!(p.verdict, Some(Verdict::Risk));
        assert_eq!(p.reasons[0].cyl, Some(4));
        p.failures.push((12.5, "cylinder 4: ring land fractured".into()));
        judge_phase(&mut p);
        assert_eq!(p.verdict, Some(Verdict::Breaks));
        assert!(p.reasons.iter().any(|r| r.level == Verdict::Breaks && r.cyl == Some(4) && (r.t_s - 12.5).abs() < 1e-9));
        let table = vec![("MIS_2".to_string(), 0x0304, 0x3B1E), ("MIS_IGC_2".to_string(), 0x33C2, 0x407E)];
        assert_eq!(decode_ecu_dtc("003B1E status 0x2F", &table), "P0304 MIS_2 (клиентский 3B1E, статус 0x2F)");
        assert!(decode_ecu_dtc("001234 status 0x08", &table).contains("нет в таблице"));
    }

    /// Stock SC8O40's own error table: cylinder misfire and network codes map to their ERRM names (VERIFIED-S).
    #[test]
    fn sc8o40_error_table_has_standard_obd_and_vag_codes() {
        let Some(root) = root() else { return };
        let Ok(fo) = crate::paths::read_firmware(&root, "8V0906264H__0005.bin") else { return };
        let Ok(dbo) = simemu_damos::DamosDatabase::load_from_json(&root.join("definitions/SC8O40.json")) else { return };
        let off_o = dbo.parameters.iter().find(|p| p.id == "c_err_dtc_cus_aeb_oc_0").map(|p| (p.cpu_address - 0x8080_0000) as usize).expect("DAMOS O40");
        let to = ecu_error_table(&fo, off_o);
        let by = |obd: u16| to.iter().filter(|e| e.1 == obd).map(|e| e.0.as_str()).collect::<Vec<_>>();
        assert_eq!(by(0x0304), ["MIS_2"]);
        assert_eq!(by(0x130A), ["INH_INJ_MIS"]);
    }

    #[test]
    #[ignore = "slow: several ECU + DQ250 + plant co-simulations"]
    fn tune_check_stock_on_bench() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let stock = "8V0906264H__0005.bin";
        let tested = std::env::var("SIMEMU_CHECK_IMAGE").unwrap_or_else(|_| stock.to_string());
        if !crate::paths::has_firmware(&root, stock) || !crate::paths::has_firmware(&root, &tested) {
            eprintln!("SKIP: {stock} or {tested} not found");
            return;
        }
        let phases: Option<Vec<&'static str>> = std::env::var("SIMEMU_CHECK_PHASES").ok().map(|v| PROGRAM.iter().map(|d| d.id).filter(|id| v.split(',').any(|x| x == *id)).collect());
        let build = simemu_physics::bench_config::preset("octavia_fl").expect("octavia_fl");
        let (p, c) = (Mutex::new(Progress::default()), AtomicBool::new(false));
        let slot = std::env::var("SIMEMU_CHECK_SLOT").ok().and_then(|v| v.parse::<u8>().ok()).unwrap_or(0);
        let reference = (tested != stock && std::env::var("SIMEMU_CHECK_REF").map_or(true, |v| v != "0")).then(|| stock.to_string());
        let req = CheckRequest { root: root.clone(), build, image: tested, reference, immo_lab: true, tcu_eeprom: None, phases, mm_slot: (slot > 0).then_some(slot) };
        let a = run(&req, &p, &c);
        let v = a.tested.verdict.unwrap_or(Verdict::Ok);
        assert!(v < Verdict::Breaks, "tune check broke on {}: {}", req.image, v.ru());
    }
}
