//! Bench / vehicle configurator: the physical car is composed from independent parts — long block, turbocharger,
//! intake, intercooler, exhaust, spark plugs, fuel, wear, gearbox, vehicle, ambient — and any firmware image runs on
//! any composition, like a real car with any software flashed. Loading an image never changes the build and the
//! build never selects an image; [`compatibility`] only reports what does not match.
//!
//! Every option is a physical parameter set with its source; nothing here is behaviour. [`BenchConfig::resolve`]
//! is a pure function run once at bench start (zero runtime cost): it builds the existing [`EngineProfile`] +
//! [`DrivetrainProfile`] from the existing constructors with struct-update (`..base`), so fields other agents add
//! to those profiles keep their defaults. The plug heat range, deposits, fuel blend and catalyst storage are consumed by the
//! plant (combustion / thermal / lambda models); quantities no plant model consumes yet stay reported in
//! [`PendingPhysics`] (knock-sensor transfer, AVS).
//!
//! Sources are cited per item (`docs/BENCH_BUILD.md` has the full table); values without data are `UNVERIFIED`.

use crate::atmosphere::Ambient;
use crate::clutch::ClutchPack;
use crate::profiles::{DrivetrainProfile, EngineProfile, GasPathHw, TurboHardware, THROTTLE_CDA_EA888_GEN3};
use crate::vehicle::BodyHw;
use crate::VirtualPlant;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// ================================================================================================================
// Choices
// ================================================================================================================

/// Long block (block, crank train, head with integrated exhaust manifold, cams) — the stock ancillaries of each are
/// in [`EngineSpec`] and can be swapped through [`Ancillaries`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LongBlock {
    /// 1.8 TSI EA888 Gen3 CJSA/CJSB, 132 kW (Octavia A7, Golf 7).
    Cjsa,
    /// 2.0 TSI EA888 Gen3 CJXC (and CJXE/CJXH/CJXG), 206–228 kW (Golf 7 R, S3 8V).
    Cjxc,
    /// 2.0 TSI EA888 Gen3 CHHB/CHHA, 162/169 kW (Golf 7 GTI / GTI Performance).
    Chhb,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TurboId {
    Is12,
    Is20,
    Is38,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntakeKind {
    /// Stock airbox with paper element and fresh-air snorkel ("штатный корпус фильтра").
    Airbox,
    /// Open zero-resistance cone filter in the engine bay ("нулевик").
    Cone,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Intake {
    pub kind: IntakeKind,
    /// Inner diameter of the duct from the filter to the compressor inlet, mm (stock [`STOCK_INTAKE_MM`]).
    pub pipe_mm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CatKind {
    /// OEM close-coupled catalyst.
    Oem,
    /// Aftermarket high-flow ("sport", 200 cpsi metal) catalyst.
    Sport,
    /// No catalyst.
    Removed,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Exhaust {
    /// Downpipe + cat-back pipe inner diameter, mm.
    pub pipe_mm: f64,
    pub cat: CatKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IcCore {
    /// OEM front-mount tube-and-fin core.
    Stock,
    /// Aftermarket front-mount bar-and-plate core, Wagner/Forge/APR class frontal area.
    Upgraded,
}

/// Intercooler and charge pipes.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Intercooler {
    pub core: IcCore,
    /// Core thickness (depth in the cooling-air direction), mm (stock [`STOCK_IC_CORE_MM`]).
    pub core_mm: f64,
    /// Charge-pipe inner diameter, mm (stock [`STOCK_IC_PIPE_MM`]).
    pub pipe_mm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SparkPlug {
    /// VW OE plug of the EA888 Gen3 (06K905601B).
    VwOe,
    /// NGK SILFER8C7ES (one heat range colder; later OE 06K905601M).
    NgkSilfer8c7es,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThrottleBody {
    /// EA888 Gen3 Ø56 mm throttle as characterised by the CJSA (264H) and CJXC (264K) calibrations — identical.
    Gen3,
    /// The characteristic of the 8V0906264K / 264F calibrations (CJHA/CJJG), 10–18 % more area above 10 °TPS.
    Cjha,
}

/// Bolt-on parts ("навесное"); `None` = the long block's own part, `Some(x)` = the part of long block `x`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Ancillaries {
    pub di_injectors: Option<LongBlock>,
    pub mpi_injectors: Option<LongBlock>,
    pub hpfp: Option<LongBlock>,
    pub intake_manifold: Option<LongBlock>,
    pub throttle: ThrottleBody,
}

impl Default for Ancillaries {
    fn default() -> Self {
        Self { di_injectors: None, mpi_injectors: None, hpfp: None, intake_manifold: None, throttle: ThrottleBody::Gen3 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Fuel {
    /// Pump octane of the base gasoline (АИ-92/95/98/100).
    pub ron: f64,
    /// Ethanol, % by volume (E10 = 10).
    pub ethanol_pct: f64,
}

/// Mileage of the engine and of each part, km; a part at 0 has the engine's mileage.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct Mileage {
    pub engine_km: f64,
    pub di_injectors_km: f64,
    pub mpi_injectors_km: f64,
    pub spark_plugs_km: f64,
    pub turbo_km: f64,
    pub hpfp_km: f64,
    pub catalyst_km: f64,
    pub clutches_km: f64,
}

impl Mileage {
    /// Mileage of a part (0 = the engine's).
    pub fn of(&self, part_km: f64) -> f64 {
        if part_km > 0.0 { part_km } else { self.engine_km.max(0.0) }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Gearbox {
    /// DQ200 0CW, 7-speed dry clutches (FWD only).
    Dq200,
    /// DQ250 02E, 6-speed wet clutches.
    Dq250_02e,
    /// DQ250 0D9 (MQB), 6-speed wet clutches.
    Dq250_0d9,
    /// DQ381 0GC, 7-speed wet clutches.
    Dq381,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Vehicle {
    /// Skoda Octavia A7 FL liftback 2017–2020.
    OctaviaA7Fl,
    /// Skoda Octavia A7 (5E) liftback 2013–2016.
    OctaviaA7,
    /// Audi A3 8V Sportback.
    A3_8v,
    /// VW Golf 7 GTI (5G) DSG.
    #[serde(alias = "bench")]
    Golf7Gti,
    /// VW Golf 7 R (5G) DSG 4MOTION.
    Golf7R,
}

/// A complete physical car on the bench.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct BenchConfig {
    pub name: String,
    pub engine: LongBlock,
    pub turbo: TurboId,
    pub intake: Intake,
    pub intercooler: Intercooler,
    pub exhaust: Exhaust,
    pub ancillaries: Ancillaries,
    pub spark_plug: SparkPlug,
    pub fuel: Fuel,
    pub mileage: Mileage,
    pub gearbox: Gearbox,
    /// Rear axle with Haldex 5 coupling.
    pub awd: bool,
    pub vehicle: Vehicle,
    /// Measured dynamic tyre radius instead of the vehicle's standard size, m.
    pub tyre_radius_m: Option<f64>,
    pub ambient: Ambient,
    /// Firmware file of the gearbox's TCU when it is emulated (DQ250 0D9: a file in the workspace root or `images/dq250/`);
    /// `None` = the studio default. Firmware, not hardware — kept with the build because the car's mechatronic carries it.
    pub tcu_image: Option<String>,
    /// Mechatronic coding a workshop writes (`2E 0600`, the decimal number a tester shows, 24 = "000024"); `None` = the
    /// image's default (studio `dq250_prep`).
    pub tcu_coding: Option<u32>,
}

impl Default for BenchConfig {
    fn default() -> Self {
        Self {
            name: PRESETS[0].2.to_string(),
            engine: LongBlock::Cjsa,
            turbo: TurboId::Is12,
            intake: Intake { kind: IntakeKind::Airbox, pipe_mm: STOCK_INTAKE_MM },
            intercooler: Intercooler { core: IcCore::Stock, core_mm: STOCK_IC_CORE_MM, pipe_mm: STOCK_IC_PIPE_MM },
            exhaust: Exhaust { pipe_mm: STOCK_EXHAUST_MM, cat: CatKind::Oem },
            ancillaries: Ancillaries::default(),
            spark_plug: SparkPlug::NgkSilfer8c7es,
            fuel: Fuel { ron: 95.0, ethanol_pct: 0.0 },
            mileage: Mileage::default(),
            gearbox: Gearbox::Dq200,
            awd: false,
            vehicle: Vehicle::OctaviaA7Fl,
            tyre_radius_m: None,
            ambient: Ambient::default(),
            tcu_image: None,
            tcu_coding: None,
        }
    }
}

// ================================================================================================================
// Catalogue data (with sources)
// ================================================================================================================

/// Long-block data sheet. Sources: `[SSP606]` Audi SSP 606 "1.8l/2.0l TFSI EA888 Gen3" (11/2012), `[SSP522]` VW SSP 522
/// "TSI 2.0 l 162/169 kW", `[WM]` Audi workshop manual Gen III, `[NEWS]` VW newsroom "The new Golf R" (2013); full table
/// with URLs in docs/BENCH_BUILD.md.
#[derive(Debug, Clone, Serialize)]
pub struct EngineSpec {
    pub id: LongBlock,
    pub name: &'static str,
    pub codes: &'static [&'static str],
    pub bore_mm: f64,
    pub stroke_mm: f64,
    pub compression_ratio: f64,
    pub rated_kw: f64,
    pub rated_nm: f64,
    /// Exhaust cam lift steps (AVS small / large), mm.
    pub exhaust_lift_mm: &'static [f64],
    pub di_injector: &'static str,
    pub mpi: &'static str,
    pub hpfp: &'static str,
    pub pistons_rods: &'static str,
    pub knock_sensors: &'static str,
    pub stock_turbo: TurboId,
    pub sources: &'static str,
}

const GEN3_MPI: &str = "port injection (EU Euro 5+/6): plastic rail, own low-pressure sensor G410, fed through the HPFP flush line \
     [SSP606 p.49]; injector flow 2.30 mg/ms from SC8S50/SC8O40 (`PortFuel::cjsa_stock`)";
const GEN3_HPFP: &str = "Hitachi 06L127025x, 4-lobe cam on the exhaust camshaft, 200 bar max [SSP606 p.49]; stroke volume = \
     SC8S50 c_mfp_max 232 mg (VERIFIED-S as the R firmware's model; the 1.8 calibrations use 150–174.5 mg — open question)";
const GEN3_KNOCK: &str = "one sensor G61 (06K905377E), below the intake manifold behind the coolant pump [SSP606, WM]; \
     position along the block UNVERIFIED; transfer function modelled in `KnockSensorPath`";

pub const ENGINES: [EngineSpec; 3] = [
    EngineSpec {
        id: LongBlock::Cjsa,
        name: "1.8 TSI EA888 Gen3 CJSA/CJSB",
        codes: &["CJSA", "CJSB"],
        bore_mm: 82.5,
        stroke_mm: 84.1,
        compression_ratio: 9.6,
        rated_kw: 132.0,
        rated_nm: 250.0,
        exhaust_lift_mm: &[6.35, 10.0],
        di_injector: "15 cm³/s [SSP606 p.59]; the 264H calibration family characterises 14.1 cm³/s @ 10 MPa (ip_ti_4_l fit, VERIFIED-S)",
        mpi: GEN3_MPI,
        hpfp: GEN3_HPFP,
        pistons_rods: "taper-face top ring, stepped 2nd ring, 2-piece oil ring; cracked rods, no small-end bush, DLC pin [SSP606 p.11]",
        knock_sensors: GEN3_KNOCK,
        stock_turbo: TurboId::Is12,
        sources: "SSP606 p.6 (CJSA 132 kW 5100–6200, 250 Nm 1250–5000; CJSB 280 Nm); AVS on the exhaust cam [SSP606], \
                  lift steps 6.35/10 mm are Gen2 values (SSP 436) taken over per SSP606 — Gen3 UNVERIFIED",
    },
    EngineSpec {
        id: LongBlock::Cjxc,
        name: "2.0 TSI EA888 Gen3 CJXC (Golf 7 R)",
        codes: &["CJXC", "CJXE", "CJXH", "CJXG", "CJXB", "CJXD"],
        bore_mm: 82.5,
        stroke_mm: 92.8,
        compression_ratio: 9.3,
        rated_kw: 221.0,
        rated_nm: 380.0,
        exhaust_lift_mm: &[6.35, 10.0],
        di_injector: "20 cm³/s, faster opening [SSP606 p.57/59], 06L906036H→L; the SC8S50/8V0906259J calibration characterises 19.0 cm³/s @ 10 MPa (VERIFIED-S)",
        mpi: GEN3_MPI,
        hpfp: GEN3_HPFP,
        pistons_rods: "9.3:1 pistons, barrel-face top ring, higher-flow piston-cooling jets, hollow Ni-alloy nitrided exhaust valves, \
                       revised head alloy and seat rings [SSP606 p.57, NEWS]; cast vs forged pistons UNVERIFIED",
        knock_sensors: GEN3_KNOCK,
        stock_turbo: TurboId::Is38,
        sources: "SSP606 p.7, NEWS (221 kW 5500–6200, 380 Nm 1800–5500, boost up to 1.2 bar)",
    },
    EngineSpec {
        id: LongBlock::Chhb,
        name: "2.0 TSI EA888 Gen3 CHHB/CHHA (Golf 7 GTI)",
        codes: &["CHHB", "CHHA"],
        bore_mm: 82.5,
        stroke_mm: 92.8,
        compression_ratio: 9.6,
        rated_kw: 162.0,
        rated_nm: 350.0,
        exhaust_lift_mm: &[6.35, 10.0],
        di_injector: "17.5 cm³/s [SSP606 p.59], 06A906036Q (retailer); at 10 MPa scaled by the fit/SSP ratio of the two calibrations we have \
                      (0.945) → 16.5 cm³/s (inferred)",
        mpi: GEN3_MPI,
        hpfp: GEN3_HPFP,
        pistons_rods: "barrel-face top ring, cracked rods, DLC pin [SSP606 p.11, SSP522]",
        knock_sensors: GEN3_KNOCK,
        stock_turbo: TurboId::Is20,
        sources: "SSP522 (CHHB 162 kW 4500–6200, 350 Nm 1500–4400; CHHA 169 kW; SIMOS 18.1; RON 98)",
    },
];

pub fn engine_spec(id: LongBlock) -> &'static EngineSpec {
    match id {
        LongBlock::Cjsa => &ENGINES[0],
        LongBlock::Cjxc => &ENGINES[1],
        LongBlock::Chhb => &ENGINES[2],
    }
}

/// Turbocharger data sheet. No compressor map of any IS-series turbo is published (IHI); the plant's elliptic speed
/// lines are fitted to what is published: wheel sizes, and flow/PR capability inferred from vendor statements.
#[derive(Debug, Clone, Serialize)]
pub struct TurboSpec {
    pub id: TurboId,
    pub name: &'static str,
    pub part_numbers: &'static str,
    /// Compressor wheel inducer / exducer, turbine wheel inlet / outlet, mm (NaN = not published).
    pub compressor_mm: [f64; 2],
    pub turbine_mm: [f64; 2],
    /// Speed limit, rpm: compressor tip speed [`TIP_SPEED_LIMIT_MS`] on this wheel. Inferred — IHI publishes none.
    pub max_speed_rpm: f64,
    pub wastegate: &'static str,
    pub bearings: &'static str,
    pub sources: &'static str,
}

/// Compressor tip speed taken as the speed limit, m/s: Garrett's limits for its IS38 drop-in replacements
/// (60 mm @ 185 krpm, 67 mm @ 165 krpm ≈ 580 m/s). Inferred.
pub const TIP_SPEED_LIMIT_MS: f64 = 580.0;
const fn limit_rpm(exducer_mm: f64) -> f64 {
    TIP_SPEED_LIMIT_MS / (PI * exducer_mm * 1e-3) * 60.0
}
const IS_BEARINGS: &str = "journal bearings, oil- and coolant-cooled bearing housing [SSP606]; restrictor size UNVERIFIED";

pub const TURBOS: [TurboSpec; 3] = [
    TurboSpec {
        id: TurboId::Is12,
        name: "IHI IS12",
        part_numbers: "06K145701J/L/M/N/S/T, 06K145713J/L, 06K145721G/Q; IHI F41CEG-SR028B (vendor)",
        compressor_mm: [f64::NAN, 49.0],
        turbine_mm: [37.0, f64::NAN],
        max_speed_rpm: limit_rpm(49.0),
        wastegate: "electric: DC motor V465 (~1 kHz PWM), Hall position sensor G581, mechanical end stops [SSP606]",
        bearings: IS_BEARINGS,
        sources: "wheel sizes not published: 49/37 mm = the plant's existing UNVERIFIED tip radii; speed line unchanged \
                  (choke 0.17 kg/s, PR₀ 2.4 @ 200 krpm, UNVERIFIED) — APR stage 1 245 hp on IS12 needs ≈ 0.19 kg/s",
    },
    TurboSpec {
        id: TurboId::Is20,
        name: "IHI IS20",
        part_numbers: "06K145702Q/R/T, 06K145722G/L, 06K145874C/K/L/M; IHI F51CEG-SR031B (vendor)",
        compressor_mm: [42.5, 54.0],
        turbine_mm: [50.4, 46.8],
        max_speed_rpm: limit_rpm(54.0),
        wastegate: "electric V465/G581 [SSP606]; actuator 06K145725S (vendor)",
        bearings: IS_BEARINGS,
        sources: "wheels: australiancar.reviews / eurosporttuning (secondary); speed line inferred: 300–330 crank hp limit → \
                  choke ≈ 0.24 kg/s at the limit speed, \"efficient to 28–30 psi\" → PR 3.1 at 0.17 kg/s (UNVERIFIED)",
    },
    TurboSpec {
        id: TurboId::Is38,
        name: "IHI IS38",
        part_numbers: "06K145702H/J/M/N, 06K145722A/H/P/T, 06K145874F/N (9VA04); facelift 06K145722N/S, 06K145874G/P (9VA05) (vendor)",
        compressor_mm: [45.2, 58.0],
        turbine_mm: [54.7, 47.4],
        max_speed_rpm: limit_rpm(58.0),
        wastegate: "electric V465/G581 [SSP606]; actuator 06K145725T (Mahle, later Hanon) (vendor/forum)",
        bearings: IS_BEARINGS,
        sources: "wheels: australiancar.reviews, mygolfmk7 (secondary; 44.9/57 per a forum seller); speed line inferred: Garrett \
                  GT2260S (+17 % flow, 485 hp) and G25-660 (+59 %, 600 hp) replacements → IS38 choke 0.27–0.33 kg/s; tuned PR ≈ 3.0 \
                  at ≈ 0.26 kg/s near the limit speed (UNVERIFIED)",
    },
];

pub fn turbo_spec(id: TurboId) -> &'static TurboSpec {
    match id {
        TurboId::Is12 => &TURBOS[0],
        TurboId::Is20 => &TURBOS[1],
        TurboId::Is38 => &TURBOS[2],
    }
}

/// Plant hardware of a turbocharger. Built from the existing profile constructors (`..base`), so fields added to
/// [`TurboHardware`] by other work keep their per-turbo defaults. The IS20/IS38 speed lines and wheel radii are the
/// sourced/inferred values of [`TURBOS`]; the IS12 keeps the existing (baseline) hardware exactly.
pub fn turbo_hardware(id: TurboId) -> TurboHardware {
    let wheels = |b: TurboHardware, s: &TurboSpec| TurboHardware {
        compressor_tip_radius_m: s.compressor_mm[1] / 2e3,
        turbine_tip_radius_m: s.turbine_mm[0] / 2e3,
        ..b
    };
    match id {
        TurboId::Is12 => EngineProfile::ea888_18t_cjsa_is12().turbo,
        // 0.24 kg/s at the limit speed (205 krpm) → 0.234 @ 200 krpm; PR 3.1 at 0.17 kg/s on the limit line → PR₀ 3.8.
        TurboId::Is20 => wheels(TurboHardware { choke_flow_ref_kg_s: 0.234, pr_zero_flow_ref: 3.8, ..EngineProfile::ea888_20t_chhb_is20().turbo }, turbo_spec(id)),
        // 0.315 kg/s at the limit speed (191 krpm) → 0.33 @ 200 krpm; PR 3.0 at 0.26 kg/s on the limit line → PR₀ 4.88.
        TurboId::Is38 => wheels(TurboHardware { choke_flow_ref_kg_s: 0.33, pr_zero_flow_ref: 4.88, ..EngineProfile::ea888_20t_cjxc_is38().turbo }, turbo_spec(id)),
    }
}

/// Choke flow and zero-flow pressure ratio on the speed-limit line of the plant's elliptic speed-line model
/// (`engine.rs`: choke ∝ N, PR − 1 ∝ N²).
pub fn turbo_envelope(id: TurboId) -> (f64, f64) {
    let (hw, spec) = (turbo_hardware(id), turbo_spec(id));
    let nr = spec.max_speed_rpm / hw.n_ref_rpm;
    (hw.choke_flow_ref_kg_s * nr, 1.0 + (hw.pr_zero_flow_ref - 1.0) * nr * nr)
}

/// Pressure ratio of the plant's elliptic speed line at shaft speed `n_rpm` and corrected flow `flow_kg_s`
/// (`engine.rs`), `None` at or past choke.
pub fn speed_line_pr(hw: &TurboHardware, n_rpm: f64, flow_kg_s: f64) -> Option<f64> {
    let nr = n_rpm / hw.n_ref_rpm;
    let x = flow_kg_s / (hw.choke_flow_ref_kg_s * nr);
    (x < 1.0).then(|| 1.0 + (hw.pr_zero_flow_ref - 1.0) * nr * nr * (1.0 - x * x).sqrt())
}

// ---- intake ---------------------------------------------------------------------------------------------------
// Flow-bench data at 28 inH2O (6.97 kPa): complete factory MQB intake 275 cfm (APR) — 552.6 cfm (034 Motorsport),
// i.e. k = 2.9·10⁵ … 0.7·10⁵ Pa/(kg/s)²; the plant's stock `AIR_FILTER_K` 1.0·10⁵ lies inside. Open intakes cut the
// restriction by ~50 % (Mishimoto, 200–600 cfm); APR's turbo inlet pipe flows 441 vs 379 cfm of the factory pipe.

/// Stock duct airbox → compressor, inner diameter, mm. UNVERIFIED (Racingline quotes 62.5 → 76 mm, for Gen4).
pub const STOCK_INTAKE_MM: f64 = 62.5;
/// Share of the stock intake loss in the airbox + element (the duct share scales ∝ D⁻⁴). UNVERIFIED.
const AIRBOX_SHARE: f64 = 0.5;
/// Air density the loss coefficients are referred to, kg/m³.
const RHO_REF: f64 = 1.2;
/// Zero-resistance cone element: ≈ 0.2 kPa at 140 g/s (K&N element flow data scaled — order of magnitude),
/// Pa/(kg/s)². UNVERIFIED.
const CONE_ELEMENT_K: f64 = 1.0e4;
/// Duct from the cone to the compressor, ζ referred to the duct dynamic pressure: ≈ 1 m of smooth pipe (Blasius
/// f ≈ 0.016 → 0.26), two bends (2 × 0.3), cone mouth (0.05). UNVERIFIED (textbook loss coefficients).
const CONE_PIPE_ZETA: f64 = 0.9;

/// Δp = k·ṁ² of a duct with loss coefficient `zeta` referred to the dynamic pressure in a pipe of diameter `d_mm`.
fn duct_k(zeta: f64, d_mm: f64) -> f64 {
    let a = PI / 4.0 * (d_mm * 1e-3).powi(2);
    zeta / (2.0 * RHO_REF * a * a)
}

/// (air filter + duct k, engine-bay share of the inlet air). The «нулевик» sits in the engine bay without a cold-air
/// feed and draws hot under-hood air (`GasPathHw::inlet_rise_k`).
fn intake_gas(i: &Intake) -> (f64, f64) {
    let k0 = crate::engine::AIR_FILTER_K;
    match i.kind {
        IntakeKind::Airbox if i.pipe_mm == STOCK_INTAKE_MM => (k0, 0.0),
        IntakeKind::Airbox => (k0 * (AIRBOX_SHARE + (1.0 - AIRBOX_SHARE) * (STOCK_INTAKE_MM / i.pipe_mm).powi(4)), 0.0),
        IntakeKind::Cone => (CONE_ELEMENT_K + duct_k(CONE_PIPE_ZETA, i.pipe_mm), 1.0),
    }
}

// ---- exhaust ---------------------------------------------------------------------------------------------------
// No measured back-pressure for any MQB exhaust was found; the stock coefficient `EXHAUST_SYSTEM_K` stays the
// plant's UNVERIFIED value and the parts scale it.

/// Stock downpipe inner diameter, mm (Integrated Engineering: "60 mm stock tube"; other vendors quote 63–63.5 mm).
pub const STOCK_EXHAUST_MM: f64 = 60.0;
/// Share of the stock back-pressure coefficient in the OEM catalyst; the rest (pipes, silencers) scales ∝ D⁻⁴ with the
/// pipe diameter. UNVERIFIED (no stock cell density / pressure-drop data found).
const CAT_SHARE: f64 = 0.45;
/// 200 cpsi metal "sport" catalyst vs OEM: "~40 % less back pressure" (HJS, forum — UNVERIFIED).
const SPORT_CAT_REL: f64 = 0.6;

fn exhaust_k(e: &Exhaust) -> f64 {
    let k0 = crate::engine::EXHAUST_SYSTEM_K;
    if e.cat == CatKind::Oem && e.pipe_mm == STOCK_EXHAUST_MM {
        return k0; // stock system exactly as calibrated
    }
    let cat = match e.cat {
        CatKind::Oem => 1.0,
        CatKind::Sport => SPORT_CAT_REL,
        CatKind::Removed => 0.0,
    };
    k0 * (CAT_SHARE * cat + (1.0 - CAT_SHARE) * (STOCK_EXHAUST_MM / e.pipe_mm).powi(4))
}

// ---- intercooler ------------------------------------------------------------------------------------------------
// APR (MQB): OEM core 24.25×16.25×1.375" (R; GTI 1.25"), effectiveness 65 % (R) / 60 % (GTI); APR bar-and-plate
// 2.25" thick, 82–83 %. Wagner Competition 200001048: 640×410×65 mm core, 70 mm connections; Forge FMINT32 +83 %
// volume, pipes 63 → 76 mm. Stock charge pipes ≈ 2.5" (Mishimoto: its 2.75" pipes are "+10 %"). No pressure-drop data
// is published for any core, so Δp follows the geometry of a cross-flow core: the charge air runs along the width
// through a flow area ∝ height × thickness, Δp ∝ width / (height·thickness)²; pipes ∝ D⁻⁴.

/// OEM core width × height × thickness, mm (APR, Golf R core).
pub const STOCK_IC_CORE: [f64; 3] = [616.0, 413.0, 35.0];
pub const STOCK_IC_CORE_MM: f64 = 35.0;
/// Aftermarket front-mount core frontal size, mm (Wagner Competition).
const UPGRADED_IC_FRONT: [f64; 2] = [640.0, 410.0];
pub const STOCK_IC_PIPE_MM: f64 = 63.5;
/// Stock charge path (core + pipes) pressure-loss coefficient, Pa/(kg/s)² (≈ 6 kPa at 0.25 kg/s), and its share in
/// the core. UNVERIFIED.
const IC_STOCK_K: f64 = 1.0e5;
const IC_CORE_SHARE: f64 = 0.5;
/// Internal air share of the core envelope and the charge-pipe length (compressor → IC → throttle), m. UNVERIFIED.
const IC_AIR_SHARE: f64 = 0.45;
const IC_PIPE_LEN_M: f64 = 2.5;

/// (effectiveness, charge-air volume m³, Δp coefficient relative to the stock path Pa/(kg/s)²). Effectiveness by
/// ε-NTU with NTU ∝ core volume (same cooling-air face velocity), anchored to the plant's stock value. UNVERIFIED.
fn intercooler_hw(ic: &Intercooler, stock_volume_m3: f64, stock_eff: f64) -> (f64, f64, f64) {
    let [w0, h0, t0] = STOCK_IC_CORE;
    if ic.core == IcCore::Stock && ic.core_mm == t0 && ic.pipe_mm == STOCK_IC_PIPE_MM {
        return (stock_eff, stock_volume_m3, 0.0); // stock path exactly as calibrated
    }
    let [w, h] = match ic.core {
        IcCore::Stock => [w0, h0],
        IcCore::Upgraded => UPGRADED_IC_FRONT,
    };
    let t = ic.core_mm;
    let vol_ratio = (w * h * t) / (w0 * h0 * t0);
    let ntu0 = -(1.0 - stock_eff).ln();
    let eff = 1.0 - (-ntu0 * vol_ratio).exp();
    let core = IC_CORE_SHARE * (w / w0) * ((h0 * t0) / (h * t)).powi(2);
    let pipes = (1.0 - IC_CORE_SHARE) * (STOCK_IC_PIPE_MM / ic.pipe_mm).powi(4);
    let dv_core = (w * h * t - w0 * h0 * t0) * 1e-9 * IC_AIR_SHARE;
    let dv_pipes = PI / 4.0 * ((ic.pipe_mm * 1e-3).powi(2) - (STOCK_IC_PIPE_MM * 1e-3).powi(2)) * IC_PIPE_LEN_M;
    (eff, stock_volume_m3 + dv_core + dv_pipes, IC_STOCK_K * (core + pipes - 1.0))
}

// ---- ancillaries («навесное») ----------------------------------------------------------------------------------

/// MPI injector static flow, mg/ms: SC8S50/8V0906259J (CJXC) `ip_fac_mff_ti_stnd_h_rng_mpi[0]` @0x80208E74 and
/// SC8O40/8V0906264H (CJSA) @0x80208DBA both 0.434 ms/(mg/stk) → 2.30 mg/ms (VERIFIED-S as the firmwares' model:
/// the same port injector on 1.8 and 2.0). CHHB: no calibration here, same assumed (UNVERIFIED). Modelled in the plant
/// via [`crate::fuel_system::PortFuel`]; also carried in [`PendingPhysics::mpi_mg_ms`].
fn mpi_mg_ms(_src: LongBlock) -> f64 {
    1.0 / 0.434
}

const INTAKE_MANIFOLD: &str = "plastic manifold with tumble flaps (N316) on every Gen3 [SSP606]; plenum volume, runner \
     lengths and sections not published, calibrations carry no manifold volume (c_vol_vim_cp = 0) — plant volume 2.6 l \
     (UNVERIFIED) for every long block";

/// 8V0906264K / 264F calibrations' `ip_ar_thr_bas` (SC8S50 layout @0x8024BE3C, SC8H83 @0x8022F300; VERIFIED-S as
/// those firmwares' characteristic of their throttle), axis as [`THROTTLE_CDA_EA888_GEN3`].
pub const THROTTLE_CDA_CJHA: [(f64, f64); 16] = [
    (0.0, 0.0671), (1.0066, 0.0957), (1.5026, 0.1171), (2.0059, 0.1431), (3.0052, 0.2030), (4.5005, 0.2754),
    (7.0024, 0.4596), (10.0003, 0.7197), (12.9982, 1.1328), (17.0027, 1.7166), (20.9998, 2.3702),
    (25.0043, 3.0050), (30.0008, 3.9992), (40.0011, 6.5213), (60.0016, 13.8643), (80.0022, 23.7063),
];

fn throttle_table(t: ThrottleBody) -> &'static [(f64, f64)] {
    match t {
        ThrottleBody::Gen3 => &THROTTLE_CDA_EA888_GEN3,
        ThrottleBody::Cjha => &THROTTLE_CDA_CJHA,
    }
}

// ---- spark plugs ---------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize)]
pub struct PlugSpec {
    pub id: SparkPlug,
    pub name: &'static str,
    /// NGK-scale heat range (higher = colder).
    pub heat_range: f64,
    /// Electrode gap as installed, m.
    pub gap_m: f64,
    pub electrodes: &'static str,
    pub sources: &'static str,
}

pub const PLUGS: [PlugSpec; 2] = [
    PlugSpec {
        id: SparkPlug::VwOe,
        name: "VW 06K905601B = NGK PLFER7A8EG",
        heat_range: 7.0,
        gap_m: 0.0008,
        electrodes: "Laser Platinum (Pt centre and ground), M14×1.25, 26.5 mm reach, 16 mm hex",
        sources: "OE fitment CJSA/CJSB/CJXC/CHHA/CHHB (vagparts.com.au); cross-reference and 0.8 mm gap (retailers) — VW gap spec UNVERIFIED",
    },
    PlugSpec {
        id: SparkPlug::NgkSilfer8c7es,
        name: "NGK SILFER8C7ES (= VW 06K905601M)",
        heat_range: 8.0,
        gap_m: 0.0007,
        electrodes: "Laser Iridium centre, platinum-tipped ground, 1 kΩ, M14×1.25, 26.5 mm reach, 16 mm hex",
        sources: "ngk.com no. 94712 (via listings); later VW OE 06K905601M → SILFER8C7ES (stmtuned); heat range one step colder",
    },
];

pub fn plug_spec(id: SparkPlug) -> &'static PlugSpec {
    match id {
        SparkPlug::VwOe => &PLUGS[0],
        SparkPlug::NgkSilfer8c7es => &PLUGS[1],
    }
}

// ---- fuel ------------------------------------------------------------------------------------------------------
// Russian AI-92/95/98: GOST 32513-2013 (RON 92/95/98, MON 83/85/88, ethanol ≤ 5 vol %). Ethanol: LHV 26.95 MJ/kg, AFR
// 9.0, latent heat 921 kJ/kg, 0.794 kg/l (DOE fuel table), RON 109 (Frontiers 2018). Gasoline latent heat 349 kJ/kg (DOE).
// Octane blends close to linearly on a molar basis (Anderson et al., SAE 2012-01-1274).

/// Ethanol: LHV MJ/kg, stoichiometric AFR, latent heat kJ/kg, density kg/m³, RON, molar mass g/mol.
const ETHANOL: (f64, f64, f64, f64, f64, f64) = (26.95, 9.0, 921.0, 794.0, 109.0, 46.07);
const GASOLINE_LATENT_KJ_KG: f64 = 349.0;
/// Mean molar mass of gasoline, g/mol. UNVERIFIED (typical 95–105).
const GASOLINE_MOLAR_G: f64 = 100.0;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct FuelProps {
    pub ron: f64,
    pub lhv_mj_kg: f64,
    pub afr_stoich: f64,
    pub latent_kj_kg: f64,
    pub ethanol_mass_frac: f64,
}

/// Blend properties. Gasoline values are the plant's own constants (`cylinder.rs`), so E0 is exactly the plant.
pub fn fuel_props(f: &Fuel) -> FuelProps {
    use crate::engine::{AFR_STOICH, FUEL_DENSITY_KG_M3, LHV_J_KG};
    let xv = (f.ethanol_pct / 100.0).clamp(0.0, 1.0);
    let w = xv * ETHANOL.3 / (xv * ETHANOL.3 + (1.0 - xv) * FUEL_DENSITY_KG_M3);
    let xm = (w / ETHANOL.5) / (w / ETHANOL.5 + (1.0 - w) / GASOLINE_MOLAR_G);
    let mix = |g: f64, e: f64| (1.0 - w) * g + w * e;
    FuelProps {
        ron: if xv == 0.0 { f.ron } else { (1.0 - xm) * f.ron + xm * ETHANOL.4 },
        lhv_mj_kg: mix(LHV_J_KG / 1e6, ETHANOL.0),
        afr_stoich: mix(AFR_STOICH, ETHANOL.1),
        latent_kj_kg: mix(GASOLINE_LATENT_KJ_KG, ETHANOL.2),
        ethanol_mass_frac: w,
    }
}

// ---- wear ------------------------------------------------------------------------------------------------------

/// Wear per 100 000 km of each part (model calibration knobs). Linear in the part's mileage ([`Mileage::of`]).
#[derive(Debug, Clone, Copy, Serialize)]
pub struct WearModel {
    /// Ring-pack leakage-area growth: blow-by loss 1.5 → 2.5 % of the indicated work from new to 300 000 km at high
    /// load (Koszalka & Krzaczek, Energies 2022, modelled PFI engine) → +67 % per 300 000 km. UNVERIFIED for EA888
    /// (VW Gen1 compression limits: 11–14 bar new, 7 bar wear limit).
    pub ring_leak: f64,
    /// Combustion-chamber deposits: equilibrium thickness, mm (no GDI data — UNVERIFIED), approached with this mileage
    /// constant, km (octane-requirement increase saturates after 5 000–15 000 miles, US 4160648 / SAE 750933).
    pub deposit_eq_mm: f64,
    pub deposit_km: f64,
    /// DI injector static-flow loss (tip coking), fraction. UNVERIFIED — GDI fouling tests report a few % over high mileage.
    pub di_flow_loss: f64,
    /// MPI injector flow loss (valve-seat deposits), fraction. UNVERIFIED.
    pub mpi_flow_loss: f64,
    /// Spark-plug gap growth of precious-metal plugs, m (VW interval 60 000 km; ≈ 0.1 mm per interval). UNVERIFIED.
    pub plug_gap_growth_m: f64,
    /// Turbocharger isentropic efficiency loss (bearing play, tip-clearance rub, fouling), absolute. UNVERIFIED.
    pub turbo_eta_loss: f64,
    /// HPFP delivered-volume loss (plunger and valve leakage), fraction. UNVERIFIED.
    pub hpfp_loss: f64,
    /// Catalyst oxygen-storage loss (thermal ageing), fraction; floor 0.2 of new. UNVERIFIED.
    pub cat_osc_loss: f64,
    /// Clutch lining wear: DQ200 dry clutch, mm per clutch (both faces; failure at 2 × 1.3 mm in `clutch.rs` — a
    /// DQ200 clutch lasting ≈ 150 000 km uses ≈ 70 %), wet DSG pack, mm of total lining. UNVERIFIED.
    pub dry_clutch_mm: f64,
    pub wet_clutch_mm: f64,
}

pub const WEAR: WearModel = WearModel {
    ring_leak: 2.0 / 9.0,
    deposit_eq_mm: 0.2,
    deposit_km: 8_000.0,
    di_flow_loss: 0.01,
    mpi_flow_loss: 0.01,
    plug_gap_growth_m: 0.15e-3,
    turbo_eta_loss: 0.015,
    hpfp_loss: 0.02,
    cat_osc_loss: 0.2,
    dry_clutch_mm: 1.2,
    wet_clutch_mm: 0.2,
};
/// Piston crown + head surface the deposits cover, cm² (EA888 Ø82.5 bore). UNVERIFIED.
const CHAMBER_AREA_CM2: f64 = 107.0;

/// Multiplier `1 − rate·km/100 000` (exactly 1 at 0 km).
fn worn(rate: f64, km: f64) -> f64 {
    1.0 - rate * km / 1e5
}

/// Geometric compression ratio with deposits of `deposit_mm` on crown and head (about 10 % of the octane-requirement
/// increase comes from this volume effect, the rest is thermal — SAE 750933; the thermal part is the deposit layer of the
/// combustion model: `Mechanics::deposit_m`).
fn cr_with_deposits(v_cyl_m3: f64, cr: f64, deposit_mm: f64) -> f64 {
    if deposit_mm <= 0.0 {
        return cr;
    }
    let vc = v_cyl_m3 / (cr - 1.0) - CHAMBER_AREA_CM2 * 1e-4 * deposit_mm * 1e-3;
    (v_cyl_m3 + vc) / vc
}

// ---- vehicles --------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize)]
pub struct VehicleSpec {
    pub id: Vehicle,
    pub name: &'static str,
    /// Running mass with driver as a FWD car with a DQ200-class gearbox, kg (published mass minus the gearbox and AWD
    /// increments where the published car has them).
    pub mass_fwd_kg: f64,
    pub cd: f64,
    pub frontal_area_m2: f64,
    pub wheelbase_m: f64,
    pub tyre: &'static str,
    /// Dynamic rolling radius r ≈ 3.05·D/2π (ETRTO-style), m.
    pub tyre_radius_m: f64,
    pub sources: &'static str,
}

pub const VEHICLES: [VehicleSpec; 5] = [
    VehicleSpec {
        id: Vehicle::OctaviaA7Fl,
        name: "Skoda Octavia A7 FL 1.8 TSI DSG 2017–2020",
        mass_fwd_kg: 1335.0,
        cd: 0.302,
        frontal_area_m2: 2.22,
        wheelbase_m: 2.680,
        tyre: "205/55 R16",
        tyre_radius_m: 0.307,
        sources: "Škoda technical data 10.01.2017 (1335 kg incl. 75 kg driver, Cd 0.302, wheelbase 2680 mm multi-link, 205/55 R16, \
                  DQ200 3.76/2.27/1.53/1.13/1.18/0.96/0.80, FD 4.438/3.227); frontal area UNVERIFIED",
    },
    VehicleSpec {
        id: Vehicle::OctaviaA7,
        name: "Skoda Octavia A7 (5E) 1.8 TSI DSG 2013–2016",
        mass_fwd_kg: 1335.0,
        cd: 0.304,
        frontal_area_m2: 2.22,
        wheelbase_m: 2.680,
        tyre: "205/55 R16",
        tyre_radius_m: 0.307,
        sources: "ADAC 235193 (EU kerb 1335 kg incl. driver, 205/55 R16); Cd auto-data.net (secondary); wheelbase as FL multi-link; \
                  frontal area UNVERIFIED",
    },
    VehicleSpec {
        id: Vehicle::A3_8v,
        name: "Audi A3 8V Sportback 1.8 TFSI quattro",
        mass_fwd_kg: 1455.0 - 25.0 - 75.0,
        cd: 0.32,
        frontal_area_m2: 2.13,
        wheelbase_m: 2.637,
        tyre: "225/45 R17",
        tyre_radius_m: 0.308,
        sources: "Audi Australia spec guide 01/2014 (1455 kg incl. 68 kg driver + 7 kg luggage, 6-speed S tronic); Cd/frontal area: \
                  S3 Sportback 0.32/2.13 m² (secondary); wheelbase UNVERIFIED",
    },
    VehicleSpec {
        id: Vehicle::Golf7Gti,
        name: "VW Golf 7 GTI 2.0 TSI DSG",
        mass_fwd_kg: 1380.0 - 25.0,
        cd: 0.30,
        frontal_area_m2: 2.19,
        wheelbase_m: 2.631,
        tyre: "225/45 R17",
        tyre_radius_m: 0.3115,
        sources: "VW Golf GTI technical data / SSP 522 (EU kerb 1380 kg incl. 75 kg driver on 6-speed DSG, 225/45 R17, wheelbase 2631 mm, \
                  frontal area 2.19 m²)",
    },
    VehicleSpec {
        id: Vehicle::Golf7R,
        name: "VW Golf 7 R (DSG, 4MOTION)",
        mass_fwd_kg: 1495.0 - 25.0 - 75.0,
        cd: 0.32,
        frontal_area_m2: 2.19,
        wheelbase_m: 2.630,
        tyre: "225/40 R18",
        tyre_radius_m: 0.309,
        sources: "VW newsroom (1495 kg 6-speed DSG, 225/40 R18, Haldex 5); wheelbase auto-motor-und-sport; Cd: R not published — \
                  S3 8V Sportback 0.32 used; frontal area: Golf VII 2.19 m² (secondary)",
    },
];

pub fn vehicle_spec(id: Vehicle) -> &'static VehicleSpec {
    match id {
        Vehicle::OctaviaA7Fl => &VEHICLES[0],
        Vehicle::OctaviaA7 => &VEHICLES[1],
        Vehicle::A3_8v => &VEHICLES[2],
        Vehicle::Golf7Gti => &VEHICLES[3],
        Vehicle::Golf7R => &VEHICLES[4],
    }
}

/// Extra mass over a DQ200 FWD car: wet DSG vs DQ200, and the AWD hardware (PTU, propshaft, Haldex rear drive), kg.
/// UNVERIFIED split (DQ250 AWD +100 kg, DQ381 AWD +125 kg over DQ200 FWD).
fn gearbox_mass_kg(g: Gearbox) -> f64 {
    match g {
        Gearbox::Dq200 => 0.0,
        Gearbox::Dq250_02e | Gearbox::Dq250_0d9 => 25.0,
        Gearbox::Dq381 => 50.0,
    }
}
const AWD_MASS_KG: f64 = 75.0;

// ================================================================================================================
// Resolution
// ================================================================================================================

/// Build quantities reported with the build. Consumed by the plant since the combustion work: plug heat range
/// (`Mechanics::spark_plug`), deposits (`Mechanics::deposit_m`, hot spots and insulation), fuel blend (`VirtualPlant::
/// fuel_ethanol` → AFR, LHV, heat of vaporisation, flame speed), catalyst oxygen storage (`lambda_sensor`); still only
/// reported: knock-sensor transfer and AVS (exhaust lift switching is not decoded from the firmware).
#[derive(Debug, Clone, Serialize)]
pub struct PendingPhysics {
    pub plug_heat_range: f64,
    pub plug_electrodes: &'static str,
    pub deposit_mm: f64,
    pub fuel: FuelProps,
    /// MPI injector static flow after wear, mg/ms.
    pub mpi_mg_ms: f64,
    pub mpi: &'static str,
    pub intake_manifold: &'static str,
    /// Catalyst oxygen-storage capacity relative to new (`lambda_sensor` OSC).
    pub cat_osc_factor: f64,
    pub knock_sensors: &'static str,
    pub exhaust_lift_mm: &'static [f64],
    pub wastegate: &'static str,
    pub turbo_bearings: &'static str,
    pub pistons_rods: &'static str,
}

#[derive(Debug, Clone)]
pub struct ResolvedBench {
    pub engine: EngineProfile,
    pub drivetrain: DrivetrainProfile,
    pub ambient: Ambient,
    pub fuel_ron: f64,
    /// Lining already worn on each clutch when installed, mm (used gearbox).
    pub clutch_wear_mm: f64,
    pub pending: PendingPhysics,
}

impl ResolvedBench {
    /// The plant with these parts installed (worn clutches included). Run once at bench start.
    pub fn into_plant(self) -> VirtualPlant {
        let w = self.clutch_wear_mm;
        let mut p = VirtualPlant::new(self.engine, self.drivetrain, self.ambient);
        p.fuel_ron = self.fuel_ron;
        p.fuel_ethanol = self.pending.fuel.ethanol_mass_frac;
        p.exhaust_sensors.hw.osc_kg *= self.pending.cat_osc_factor;
        p.exhaust_sensors.osc_kg = p.exhaust_sensors.osc_kg.min(p.exhaust_sensors.hw.osc_kg);
        if w > 0.0 {
            match (&mut p.driveline.clutches, &p.drivetrain_profile.clutch) {
                (ClutchPack::Dry(s), _) => s.wear_mm = [w; 2],
                // `w` is the wear of the whole lining stack of a pack; the pack model counts it per friction surface. (It
                // used to be stored as is: 250 000 km gave 0.50 mm per surface ≥ the 0.4 mm lining → both packs "worn
                // through", lining health 0 → a quarter of the friction from the first second, `dq250_full_drive` kickdown
                // flare at 12 bar K1 carrying ≈ 100–130 Nm.)
                (ClutchPack::Wet(s), crate::clutch::ClutchHw::Wet(h)) => s.wear_m = [0, 1].map(|k| w * 1e-3 / h[k].surfaces),
                (ClutchPack::Wet(s), _) => s.wear_m = [w * 1e-3; 2],
            }
        }
        p
    }
}

/// Static name for a profile built at bench start (profile names are `&'static str`).
// ponytail: leaks one short string per bench start (a user action); intern if builds are resolved in a hot loop.
fn leak(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

/// Long block with its own ancillaries (DI injectors, HPFP, intake manifold), before turbo and bolt-ons.
fn long_block(id: LongBlock) -> EngineProfile {
    match id {
        // CJSA: stock 1.8 geometry; its own injector is the 264H family's (fit as in `ea888_18t_cjsa_is38`).
        LongBlock::Cjsa => {
            let b = EngineProfile::ea888_18t_cjsa_is12();
            let own = EngineProfile::ea888_18t_cjsa_is38();
            EngineProfile { injector_flow_cc_s: own.injector_flow_cc_s, injector_dead_time_ms: own.injector_dead_time_ms, ..b }
        }
        LongBlock::Cjxc => EngineProfile::ea888_20t_cjxc_is38(),
        // CHHB: SSP 606 17.5 cm³/s × 0.945 (fit/SSP ratio of the 264H and 259J calibrations) = 16.5 cm³/s @ 10 MPa
        // (inferred); dead time as the R injector (UNVERIFIED).
        LongBlock::Chhb => EngineProfile { injector_flow_cc_s: 16.5, ..EngineProfile::ea888_20t_chhb_is20() },
    }
}

impl BenchConfig {
    pub fn validate(&self) -> Result<(), String> {
        let range = |v: f64, lo: f64, hi: f64, what: &str| if v.is_finite() && (lo..=hi).contains(&v) { Ok(()) } else { Err(format!("{what} {v} outside {lo}..{hi}")) };
        if self.gearbox == Gearbox::Dq200 && self.awd {
            return Err("DQ200 (0CW) exists only for front-wheel drive".into());
        }
        range(self.intake.pipe_mm, 40.0, 120.0, "intake pipe, mm")?;
        range(self.intercooler.core_mm, 20.0, 150.0, "intercooler core, mm")?;
        range(self.intercooler.pipe_mm, 40.0, 102.0, "charge pipe, mm")?;
        range(self.exhaust.pipe_mm, 45.0, 102.0, "exhaust pipe, mm")?;
        range(self.fuel.ron, 80.0, 110.0, "RON")?;
        range(self.fuel.ethanol_pct, 0.0, 100.0, "ethanol, %")?;
        let m = &self.mileage;
        for (km, what) in [(m.engine_km, "engine"), (m.di_injectors_km, "DI injectors"), (m.mpi_injectors_km, "MPI injectors"), (m.spark_plugs_km, "spark plugs"), (m.turbo_km, "turbo"), (m.hpfp_km, "HPFP"), (m.catalyst_km, "catalyst"), (m.clutches_km, "clutches")] {
            range(km, 0.0, 1.0e6, &format!("{what} mileage, km"))?;
        }
        if let Some(r) = self.tyre_radius_m {
            range(r, 0.25, 0.40, "tyre radius, m")?;
        }
        Ok(())
    }

    /// Builds the plant hardware. Pure; run once at bench start.
    pub fn resolve(&self) -> Result<ResolvedBench, String> {
        self.validate()?;
        let es = engine_spec(self.engine);
        let ts = turbo_spec(self.turbo);
        let plug = plug_spec(self.spark_plug);
        let fuel = fuel_props(&self.fuel);
        let (a, m, wr) = (&self.ancillaries, &self.mileage, &WEAR);
        let part = |p: Option<LongBlock>| long_block(p.unwrap_or(self.engine));

        let mut e = long_block(self.engine);
        let (di, mpi, hpfp, im) = (part(a.di_injectors), part(a.mpi_injectors), part(a.hpfp), part(a.intake_manifold));
        e.injector_flow_cc_s = di.injector_flow_cc_s * worn(wr.di_flow_loss, m.of(m.di_injectors_km));
        e.injector_dead_time_ms = di.injector_dead_time_ms;
        e.mpi_injector_mg_ms = mpi.mpi_injector_mg_ms * worn(wr.mpi_flow_loss, m.of(m.mpi_injectors_km));
        e.mpi_ref_dp_kpa = mpi.mpi_ref_dp_kpa;
        e.mpi_dead_time_ms = mpi.mpi_dead_time_ms;
        e.hpfp_stroke_cc = hpfp.hpfp_stroke_cc * worn(wr.hpfp_loss, m.of(m.hpfp_km));
        e.hpfp_lobes = hpfp.hpfp_lobes;
        e.hpfp_efficiency = hpfp.hpfp_efficiency;
        e.hpfp_loss = hpfp.hpfp_loss;
        e.hpfp_vcv_close_delay_ms = hpfp.hpfp_vcv_close_delay_ms;
        e.hpfp_vcv_release_ms = hpfp.hpfp_vcv_release_ms;
        e.hpfp_vcv_lock_mpa = hpfp.hpfp_vcv_lock_mpa;
        e.intake_volume_m3 = im.intake_volume_m3;
        e.throttle_cda_cm2 = throttle_table(a.throttle);

        e.turbo = turbo_hardware(self.turbo);
        let eta_loss = wr.turbo_eta_loss * m.of(m.turbo_km) / 1e5;
        e.turbo.eta_compressor -= eta_loss;
        e.turbo.eta_turbine -= eta_loss;
        e.turbo.eta_loss = eta_loss;

        let (air_filter_k, inlet_underhood_share) = intake_gas(&self.intake);
        let (eff, vol, dp_rel) = intercooler_hw(&self.intercooler, e.boost_volume_m3, e.intercooler_effectiveness);
        e.intercooler_effectiveness = eff;
        e.boost_volume_m3 = vol;
        e.gas_path = GasPathHw { air_filter_k, inlet_underhood_share, charge_air_dp_rel_k: dp_rel, exhaust_k: exhaust_k(&self.exhaust) };

        e.mech.spark_gap_m = plug.gap_m + wr.plug_gap_growth_m * m.of(m.spark_plugs_km) / 1e5;
        e.mech.spark_plug.heat_range = plug.heat_range;
        let km = m.engine_km.max(0.0);
        e.mech.ring_leak_cda_m2 *= 1.0 + wr.ring_leak * km / 1e5;
        let deposit_mm = wr.deposit_eq_mm * (1.0 - (-km / wr.deposit_km).exp());
        e.compression_ratio = cr_with_deposits(e.cylinder_volume_m3(), e.compression_ratio, deposit_mm);
        // deposits also insulate and form the hot spots of the combustion model; the cylinders spread with mileage
        e.mech.deposit_m = deposit_mm * 1e-3;
        e.mech.cyl_spread = crate::profiles::CylinderSpread::new(e.mech.cyl_spread.seed, km);
        e.slug = "build";
        e.name = leak(format!("{} + {}", es.name, ts.name));

        let mut d = match self.gearbox {
            Gearbox::Dq200 => DrivetrainProfile::dq200_fwd(),
            Gearbox::Dq250_02e => DrivetrainProfile::dq250_4wd(),
            Gearbox::Dq250_0d9 => DrivetrainProfile::dq250_0d9_awd(),
            Gearbox::Dq381 => DrivetrainProfile::dq381_4wd(),
        };
        let v = vehicle_spec(self.vehicle);
        let mass = v.mass_fwd_kg + gearbox_mass_kg(self.gearbox) + if self.awd { AWD_MASS_KG } else { 0.0 };
        let base_body = DrivetrainProfile::dq200_fwd().body;
        d.body = BodyHw {
            mass_kg: mass,
            drag_coefficient: v.cd,
            frontal_area_m2: v.frontal_area_m2,
            wheelbase_m: v.wheelbase_m,
            pitch_inertia_kgm2: 1.15 * mass,
            ..base_body
        };
        d.tyre.radius_m = self.tyre_radius_m.unwrap_or(v.tyre_radius_m);
        d.awd = self.awd;
        d.haldex = if self.awd { DrivetrainProfile::dq250_4wd().haldex } else { None };
        d.slug = "build";
        d.name = leak(format!("{} · {}{}", d.tcu_code, v.name, if self.awd { " · AWD Haldex 5" } else { " · FWD" }));
        let clutch_rate = if self.gearbox == Gearbox::Dq200 { wr.dry_clutch_mm } else { wr.wet_clutch_mm };

        let mpi_mg_ms_resolved = mpi_mg_ms(a.mpi_injectors.unwrap_or(self.engine)) * worn(wr.mpi_flow_loss, m.of(m.mpi_injectors_km));
        Ok(ResolvedBench {
            engine: e,
            drivetrain: d,
            ambient: self.ambient,
            fuel_ron: fuel.ron,
            clutch_wear_mm: clutch_rate * m.of(m.clutches_km) / 1e5,
            pending: PendingPhysics {
                plug_heat_range: plug.heat_range,
                plug_electrodes: plug.electrodes,
                deposit_mm,
                fuel,
                mpi_mg_ms: mpi_mg_ms_resolved,
                mpi: es.mpi,
                intake_manifold: INTAKE_MANIFOLD,
                cat_osc_factor: if self.exhaust.cat == CatKind::Removed { 0.0 } else { worn(wr.cat_osc_loss, m.of(m.catalyst_km)).max(0.2) },
                knock_sensors: es.knock_sensors,
                exhaust_lift_mm: es.exhaust_lift_mm,
                wastegate: ts.wastegate,
                turbo_bearings: ts.bearings,
                pistons_rods: es.pistons_rods,
            },
        })
    }

    /// Expected physical effect of each part against the same car with stock bolt-ons and new parts, at a reference
    /// operating point (0.25 kg/s air ≈ 300 PS; warm engine 90 °C at 20 °C ambient).
    pub fn effects(&self) -> Result<Vec<Effect>, String> {
        let r = self.resolve()?;
        let stock = BenchConfig {
            intake: Intake { kind: IntakeKind::Airbox, pipe_mm: STOCK_INTAKE_MM },
            intercooler: Intercooler { core: IcCore::Stock, core_mm: STOCK_IC_CORE_MM, pipe_mm: STOCK_IC_PIPE_MM },
            exhaust: Exhaust { pipe_mm: STOCK_EXHAUST_MM, cat: CatKind::Oem },
            ancillaries: Ancillaries::default(),
            spark_plug: SparkPlug::VwOe,
            fuel: Fuel { ron: self.fuel.ron, ethanol_pct: 0.0 },
            mileage: Mileage::default(),
            ..self.clone()
        }
        .resolve()?;
        let fl = 0.25;
        let (g, g0) = (&r.engine.gas_path, &stock.engine.gas_path);
        let soak = |gp: &GasPathHw, v: f64| gp.inlet_rise_k(293.15, 90.0, v);
        // Spark breakdown voltage at the spark (charge density ≈ 10 kg/m³), plant model `cylinder.rs`.
        let v_bd = |gap_m: f64| (1.0e3 + 3.0e3 * (gap_m / 7e-4) * (10.0f64 / 1.2).powf(0.9)) / 1e3;
        let (choke, pr) = turbo_envelope(self.turbo);
        let thr = |e: &EngineProfile| e.throttle_cda_cm2.last().map_or(f64::NAN, |p| p.1);
        let (fp, p, p0) = (&r.pending.fuel, &r.pending, &stock.pending);
        let (e, e0) = (&r.engine, &stock.engine);
        Ok(vec![
            Effect::new("displacement", "Рабочий объём", "Displacement", e.displacement_m3() * 1e6, e0.displacement_m3() * 1e6, "cm³"),
            Effect::new("cr", "Геом. степень сжатия (нагар)", "Geometric compression ratio (deposits)", e.compression_ratio, e0.compression_ratio, "—"),
            Effect::new("ring_leak", "Утечка через кольца (износ)", "Ring-pack leakage (wear)", e.mech.ring_leak_cda_m2 * 1e6, e0.mech.ring_leak_cda_m2 * 1e6, "mm²"),
            Effect::new("deposit", "Нагар на днище/головке", "Chamber deposits", p.deposit_mm, 0.0, "mm"),
            Effect::new("turbo_flow", "Турбина: макс. расход (линия предела)", "Turbo: max flow (limit speed line)", choke, choke, "kg/s"),
            Effect::new("turbo_pr", "Турбина: PR при нулевом расходе (линия предела)", "Turbo: zero-flow PR (limit speed line)", pr, pr, "—"),
            Effect::new("turbo_eta", "Турбина: КПД компрессора (износ)", "Turbo: compressor efficiency (wear)", e.turbo.eta_compressor, e0.turbo.eta_compressor, "—"),
            Effect::new("intake_dp", "Впуск: потери давления при 0.25 кг/с", "Intake: pressure loss at 0.25 kg/s", g.air_filter_k * fl * fl / 1e3, g0.air_filter_k * fl * fl / 1e3, "kPa"),
            Effect::new("iat_soak_idle", "Подсос горячего воздуха: стоянка, мотор прогрет", "Hot under-hood air: standing, warm engine", soak(g, 0.0), soak(g0, 0.0), "K"),
            Effect::new("iat_soak_100", "Подсос горячего воздуха: 100 км/ч", "Hot under-hood air: 100 km/h", soak(g, 27.8), soak(g0, 27.8), "K"),
            Effect::new("ic_eff", "Интеркулер: эффективность (в движении)", "Intercooler effectiveness (at speed)", e.intercooler_effectiveness, e0.intercooler_effectiveness, "—"),
            Effect::new("ic_dp", "Тракт наддува: Δp к стоку при 0.25 кг/с", "Charge-air path: Δp vs stock at 0.25 kg/s", g.charge_air_dp_rel_k * fl * fl / 1e3, 0.0, "kPa"),
            Effect::new("boost_vol", "Объём тракта наддува", "Charge-air volume", e.boost_volume_m3 * 1e3, e0.boost_volume_m3 * 1e3, "l"),
            Effect::new("p4", "Противодавление за турбиной при 0.25 кг/с", "Turbine outlet back-pressure at 0.25 kg/s", g.exhaust_k * fl * fl / 1e3, g0.exhaust_k * fl * fl / 1e3, "kPa"),
            Effect::new("cat_osc", "Катализатор: кислородная ёмкость к новому", "Catalyst oxygen storage vs new", p.cat_osc_factor, p0.cat_osc_factor, "—"),
            Effect::new("throttle", "Дроссель: Cd·A при 80°", "Throttle Cd·A at 80°", thr(e), thr(e0), "cm²"),
            Effect::new("inj", "Форсунка DI (статика при 10 МПа)", "DI injector (static at 10 MPa)", e.injector_flow_cc_s, e0.injector_flow_cc_s, "cm³/s"),
            Effect::new("mpi", "Форсунка MPI (статика)", "MPI injector (static)", p.mpi_mg_ms, p0.mpi_mg_ms, "mg/ms"),
            Effect::new("hpfp", "ТНВД: объём за ход", "HPFP stroke volume", e.hpfp_stroke_cc, e0.hpfp_stroke_cc, "cm³"),
            Effect::new("plug_gap", "Свеча: зазор (износ)", "Spark plug gap (wear)", e.mech.spark_gap_m * 1e3, e0.mech.spark_gap_m * 1e3, "mm"),
            Effect::new("spark_kv", "Пробой искрового зазора (ρ = 10 кг/м³)", "Spark breakdown (ρ = 10 kg/m³)", v_bd(e.mech.spark_gap_m), v_bd(e0.mech.spark_gap_m), "kV"),
            Effect::new("plug_heat", "Калильное число свечи (NGK)", "Plug heat range (NGK)", p.plug_heat_range, p0.plug_heat_range, "—"),
            Effect::new("ron", "Октановое число смеси (RON)", "Blend octane (RON)", fp.ron, p0.fuel.ron, "—"),
            Effect::new("lhv", "Теплота сгорания смеси", "Blend lower heating value", fp.lhv_mj_kg, p0.fuel.lhv_mj_kg, "MJ/kg"),
            Effect::new("afr", "Стехиометрия смеси", "Blend stoichiometric AFR", fp.afr_stoich, p0.fuel.afr_stoich, "—"),
            Effect::new("latent", "Теплота испарения смеси", "Blend latent heat", fp.latent_kj_kg, p0.fuel.latent_kj_kg, "kJ/kg"),
            Effect::new("clutch_wear", "Сцепления: износ накладок при установке", "Clutches: lining wear when installed", r.clutch_wear_mm, 0.0, "mm"),
            Effect::new("mass", "Масса автомобиля", "Vehicle mass", r.drivetrain.body.mass_kg, stock.drivetrain.body.mass_kg, "kg"),
            Effect::new("tyre_r", "Динамический радиус шины", "Dynamic tyre radius", r.drivetrain.tyre.radius_m * 1e3, vehicle_spec(self.vehicle).tyre_radius_m * 1e3, "mm"),
        ])
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Effect {
    pub key: &'static str,
    pub ru: &'static str,
    pub en: &'static str,
    pub value: f64,
    pub stock: f64,
    pub unit: &'static str,
}

impl Effect {
    fn new(key: &'static str, ru: &'static str, en: &'static str, value: f64, stock: f64, unit: &'static str) -> Self {
        Self { key, ru, en, value, stock, unit }
    }
}

// ================================================================================================================
// Presets
// ================================================================================================================

/// Factory stock MQB vehicle presets shown in the Studio configurator: (id, Russian name, English name).
pub const PRESETS: [(&str, &str, &str); 4] = [
    ("octavia_fl", "Skoda Octavia A7 FL 1.8 TSI сток (CJSA + IS12 + DQ200)", "Skoda Octavia A7 FL 1.8 TSI stock (CJSA + IS12 + DQ200)"),
    ("a3_8v_quattro", "Audi A3 8V 1.8 TFSI quattro сток (CJSB + IS12 + DQ250 4WD)", "Audi A3 8V 1.8 TFSI quattro stock (CJSB + IS12 + DQ250 4WD)"),
    ("golf7_gti", "VW Golf 7 GTI 2.0 TSI сток (CHHB + IS20 + DQ250)", "VW Golf 7 GTI 2.0 TSI stock (CHHB + IS20 + DQ250)"),
    ("golf7r", "VW Golf 7 R 2.0 TSI сток (CJXC + IS38 + DQ250 4WD)", "VW Golf 7 R 2.0 TSI stock (CJXC + IS38 + DQ250 4WD)"),
];

pub fn preset(id: &str) -> Option<BenchConfig> {
    let stock = BenchConfig::default();
    let ui_name = PRESETS.iter().find(|p| p.0 == id).map(|p| p.2.to_string());
    Some(match id {
        "octavia_fl" | "bench_s50" => BenchConfig {
            name: ui_name.unwrap_or_else(|| PRESETS[0].2.to_string()),
            ..stock
        },
        "a3_8v_quattro" => BenchConfig {
            name: ui_name.unwrap(),
            gearbox: Gearbox::Dq250_0d9,
            awd: true,
            vehicle: Vehicle::A3_8v,
            ..stock
        },
        "golf7_gti" => BenchConfig {
            name: ui_name.unwrap(),
            engine: LongBlock::Chhb,
            turbo: TurboId::Is20,
            spark_plug: SparkPlug::VwOe,
            gearbox: Gearbox::Dq250_0d9,
            awd: false,
            vehicle: Vehicle::Golf7Gti,
            ..stock
        },
        "golf7r" => BenchConfig {
            name: ui_name.unwrap(),
            engine: LongBlock::Cjxc,
            turbo: TurboId::Is38,
            spark_plug: SparkPlug::VwOe,
            fuel: Fuel { ron: 98.0, ethanol_pct: 0.0 },
            gearbox: Gearbox::Dq250_0d9,
            awd: true,
            vehicle: Vehicle::Golf7R,
            ..stock
        },
        // Internal hybrid test configuration (1.8 TSI + IS38 + upgraded FMIC + DQ250 AWD) for stress / E2E tests.
        "cjsa_is38_dq250" | "octavia_is38" => BenchConfig {
            name: "1.8 TSI + IHI IS38 + DQ250 4WD".to_string(),
            turbo: TurboId::Is38,
            intercooler: Intercooler { core: IcCore::Upgraded, core_mm: 60.0, pipe_mm: STOCK_IC_PIPE_MM },
            exhaust: Exhaust { pipe_mm: 76.0, cat: CatKind::Removed },
            gearbox: Gearbox::Dq250_0d9,
            awd: true,
            vehicle: Vehicle::OctaviaA7,
            tyre_radius_m: Some(0.310),
            ..stock
        },
        _ => return None,
    })
}

// ================================================================================================================
// Firmware ↔ build compatibility
// ================================================================================================================

/// What a calibration says about the hardware it was made for (read from the image by the studio).
#[derive(Debug, Clone, Default, Serialize)]
pub struct CalFacts {
    /// VW part number at CAL+0x9E (e.g. `8V0906259J`).
    pub part_number: String,
    /// Engine text at CAL+0x6C (e.g. `2.0l R4 TFSI`).
    pub engine_label: String,
    /// `c_dpl` / `c_eng_vol` (DAMOS), l.
    pub displacement_l: Option<f64>,
    /// DI injector static flow at 10 MPa from the least-squares fit of `ip_ti_4_l`, cm³/s.
    pub injector_cc_s: Option<f64>,
    /// `c_n_tcha_max_sp`, turbocharger speed protection set point, rpm.
    pub n_tcha_max_sp_rpm: Option<f64>,
}

/// Part number → engine codes and stock turbo (AGENTS.md §3.1/§3.3; ETKA not checked).
const CAL_PARTS: [(&str, &[&str], Option<TurboId>); 5] = [
    ("8V0906259J", &["CJXC", "CJXE", "CJXH"], Some(TurboId::Is38)),
    ("8V0906264H", &["CJSA", "CJSB"], Some(TurboId::Is12)),
    ("8V0906264J", &["CJSA", "CJSB"], Some(TurboId::Is12)),
    ("8V0906264K", &["CJHA", "CJJG"], None),
    ("8V0906264F", &["CJHA", "CJJG"], None),
];

#[derive(Debug, Clone, Serialize)]
pub struct CompatNote {
    /// `warn` (physics will not match what the firmware models) or `info`.
    pub level: &'static str,
    pub ru: String,
    pub en: String,
}

fn note(level: &'static str, ru: String, en: String) -> CompatNote {
    CompatNote { level, ru, en }
}

/// Mismatches between what the calibration was made for and the build on the bench. Never blocks anything.
pub fn compatibility(cfg: &BenchConfig, facts: &CalFacts) -> Vec<CompatNote> {
    let mut out = Vec::new();
    let es = engine_spec(cfg.engine);
    let Ok(r) = cfg.resolve() else { return out };
    let disp_l = r.engine.displacement_m3() * 1e3;
    let part = CAL_PARTS.iter().find(|p| facts.part_number.starts_with(p.0));
    let cal_disp = facts.displacement_l.or_else(|| facts.engine_label.split('l').next().and_then(|s| s.trim().parse().ok()));
    let codes = part.map(|p| p.1.join("/")).unwrap_or_default();
    let cal_what = if codes.is_empty() { facts.engine_label.trim().to_string() } else { format!("{codes} ({})", facts.engine_label.trim()) };
    let code_hit = part.is_some_and(|p| p.1.iter().any(|c| es.codes.contains(c)));
    match cal_disp {
        Some(d) if (d - disp_l).abs() > 0.1 => out.push(note(
            "warn",
            format!("калибровка для {cal_what}, {d:.1} л; стоит {} ({disp_l:.2} л) — модель воздуха прошивки не соответствует двигателю", es.codes[0]),
            format!("calibration for {cal_what}, {d:.1} l; the bench has {} ({disp_l:.2} l) — the firmware's air model does not match the engine", es.codes[0]),
        )),
        _ if part.is_some() && !code_hit => out.push(note(
            "info",
            format!("калибровка для {cal_what}; стоит {} — тот же объём, отличия навесного не проверены", es.codes.join("/")),
            format!("calibration for {cal_what}; the bench has {} — same displacement, ancillary differences not checked", es.codes.join("/")),
        )),
        _ => {}
    }
    if let Some(q) = facts.injector_cc_s {
        let q_hw = r.engine.injector_flow_cc_s;
        if (q_hw / q - 1.0).abs() > 0.05 {
            out.push(note(
                "warn",
                format!("впрыск: прошивка рассчитана на форсунку {q:.1} см³/с @10 МПа, стоит {q_hw:.1} — подача ×{:.2} к расчётной (смесь {}) до коррекции λ-регулированием", q_hw / q, if q_hw > q { "богаче" } else { "беднее" }),
                format!("injection: the firmware models a {q:.1} cm³/s @10 MPa injector, the bench has {q_hw:.1} — delivery ×{:.2} of the modelled (mixture {}) until λ control trims it", q_hw / q, if q_hw > q { "richer" } else { "leaner" }),
            ));
        }
    }
    if let Some(t) = part.and_then(|p| p.2) {
        let same = t == cfg.turbo;
        if !same {
            let (a, b) = (turbo_spec(t).name, turbo_spec(cfg.turbo).name);
            let p = &facts.part_number;
            out.push(note(
                "warn",
                format!("наддув: штатная калибровка {p} — под {a}, стоит {b}; если калибровку не переделали под эту турбину, модели наддува и оборотов турбины прошивки ей не соответствуют"),
                format!("boost: the stock {p} calibration is for the {a}, the bench has the {b}; unless the calibration was redone for this turbo, the firmware's boost and turbo-speed models do not match it"),
            ));
        }
    }
    if let Some(n) = facts.n_tcha_max_sp_rpm {
        let lim = turbo_spec(cfg.turbo).max_speed_rpm;
        if n > lim {
            out.push(note(
                "warn",
                format!("защита турбины: уставка прошивки {n:.0} об/мин выше предела {} ({lim:.0}) — прошивка не ограничит обороты ротора", turbo_spec(cfg.turbo).name),
                format!("turbo protection: the firmware set point {n:.0} rpm is above the {} limit ({lim:.0}) — the firmware will not limit rotor speed", turbo_spec(cfg.turbo).name),
            ));
        }
    }
    if !r.drivetrain.has_tcu_firmware {
        out.push(note(
            "info",
            format!("КПП {}: прошивки ТКУ нет — коробка не эмулируется, стенд моторный", r.drivetrain.tcu_code),
            format!("gearbox {}: no TCU firmware — the gearbox is not emulated, engine dyno only", r.drivetrain.tcu_code),
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `Debug` of a profile with name and slug set to `x` (Debug prints floats round-trip exact).
    fn dbg_e(mut e: EngineProfile) -> String {
        (e.name, e.slug) = ("x", "x");
        format!("{e:?}")
    }
    fn with(f: impl FnOnce(&mut BenchConfig)) -> ResolvedBench {
        let mut c = preset("octavia_fl").unwrap();
        f(&mut c);
        c.resolve().unwrap()
    }

    #[test]
    fn every_preset_builds() {
        for (id, _, _) in PRESETS {
            let c = preset(id).unwrap_or_else(|| panic!("{id}"));
            let r = c.resolve().unwrap_or_else(|e| panic!("{id}: {e}"));
            assert!(r.engine.displacement_m3() > 1.7e-3 && r.drivetrain.body.mass_kg > 1200.0, "{id}");
            assert!(!c.effects().unwrap().is_empty());
            r.into_plant();
        }
    }

    /// The default build is the stock Octavia A7 FL 1.8 TSI (`octavia_fl`) profile.
    #[test]
    fn default_build_matches_stock_octavia_fl_and_all_stock_presets_resolve() {
        let r = BenchConfig::default().resolve().unwrap();
        assert_eq!(r.clutch_wear_mm, 0.0);
        let mut base = EngineProfile::ea888_18t_cjsa_is12();
        base.mech.spark_plug.heat_range = plug_spec(BenchConfig::default().spark_plug).heat_range;
        assert_eq!(dbg_e(r.engine), dbg_e(base));
        assert_eq!(r.drivetrain.gear_ratios, DrivetrainProfile::dq200_fwd().gear_ratios);
        assert_eq!(r.drivetrain.final_drives, DrivetrainProfile::dq200_fwd().final_drives);
        assert_eq!(r.drivetrain.body.mass_kg, vehicle_spec(Vehicle::OctaviaA7Fl).mass_fwd_kg);
        assert_eq!(r.fuel_ron, 95.0);

        for (id, _, _) in PRESETS {
            assert!(preset(id).unwrap().resolve().is_ok(), "preset {id} must resolve");
        }
    }

    #[test]
    fn cjsa_is38_dq250_preset_resolves_dq250_awd_and_upgraded_ic() {
        let r = preset("cjsa_is38_dq250").unwrap().resolve().unwrap();
        let (d, d0) = (&r.drivetrain, DrivetrainProfile::dq250_0d9_awd());
        assert_eq!((d.gear_ratios, d.final_drives, d.tyre.radius_m, d.awd), (d0.gear_ratios, d0.final_drives, d0.tyre.radius_m, d0.awd));
        assert!(d.haldex.is_some() && (d.body.mass_kg - d0.body.mass_kg).abs() < 60.0, "{}", d.body.mass_kg);
        let base_18 = EngineProfile::ea888_18t_cjsa_is12();
        assert!((r.engine.injector_flow_cc_s / base_18.injector_flow_cc_s - 1.0).abs() < 0.03);
        assert_eq!(r.engine.gas_path.inlet_underhood_share, 0.0);
        assert!(r.engine.intercooler_effectiveness > base_18.intercooler_effectiveness);
    }

    #[test]
    fn turbo_maps_are_physically_consistent() {
        for t in TURBOS {
            let hw = turbo_hardware(t.id);
            assert!(hw.eta_compressor > 0.5 && hw.eta_compressor <= 0.8 && hw.eta_turbine <= 0.8, "{}", t.name);
            assert!(t.max_speed_rpm > 150e3 && t.max_speed_rpm < 260e3, "{}", t.name);
            // Plant speed lines (`engine.rs`): PR falls monotonically with flow up to choke; a faster line lies above
            // a slower one at every flow; efficiency on the line never exceeds the peak.
            for n in [0.5, 0.7, 0.85, 1.0].map(|k| k * t.max_speed_rpm) {
                let mut last = f64::INFINITY;
                for i in 0..200 {
                    let f = 0.002 * i as f64;
                    let Some(pr) = speed_line_pr(&hw, n, f) else { break };
                    assert!(pr > 1.0 && pr <= last, "{} {n} rpm {f} kg/s", t.name);
                    assert!(speed_line_pr(&hw, n * 1.05, f).is_some_and(|p2| p2 > pr), "{} lines cross at {n}", t.name);
                    let frac = (1.0 - (f / (hw.choke_flow_ref_kg_s * n / hw.n_ref_rpm)).powi(2)).sqrt();
                    assert!(hw.eta_compressor * (1.0 - 1.2 * (frac - 0.6).powi(2)) <= 0.8);
                    last = pr;
                }
            }
        }
        // Bigger turbo flows more; every stock turbo reaches its engine's rated airflow (≈ 3.4 g/s per kW at λ 0.85,
        // BSFC 270 g/kWh) below its limit speed.
        let flow = |t| turbo_envelope(t).0;
        assert!(flow(TurboId::Is12) < flow(TurboId::Is20) && flow(TurboId::Is20) < flow(TurboId::Is38));
        for e in ENGINES {
            let need = e.rated_kw * 270.0 / 3.6e6 * 14.7 * 0.85;
            assert!(flow(e.stock_turbo) > need, "{} needs {need} kg/s", e.name);
        }
    }

    #[test]
    fn mods_change_the_expected_quantity_in_the_right_direction() {
        let r0 = with(|_| {});
        let g0 = r0.engine.gas_path.clone();
        // «Нулевик» on a 60 mm duct: less pressure loss than the airbox, hot engine-bay air when standing.
        let cone = with(|c| c.intake = Intake { kind: IntakeKind::Cone, pipe_mm: 60.0 }).engine.gas_path;
        assert!(cone.air_filter_k < g0.air_filter_k);
        assert!(cone.inlet_rise_k(293.15, 90.0, 0.0) > 5.0);
        assert!(cone.inlet_rise_k(293.15, 90.0, 0.0) > cone.inlet_rise_k(293.15, 90.0, 30.0));
        assert_eq!(g0.inlet_rise_k(293.15, 90.0, 0.0), 0.0);
        let narrow = with(|c| c.intake = Intake { kind: IntakeKind::Cone, pipe_mm: 50.0 }).engine.gas_path;
        assert!(narrow.air_filter_k > cone.air_filter_k);
        // Wider duct behind the stock airbox: less loss, still fresh air.
        let wide = with(|c| c.intake.pipe_mm = 76.0).engine.gas_path;
        assert!(wide.air_filter_k < g0.air_filter_k && wide.inlet_underhood_share == 0.0);
        // Larger exhaust and no cat: lower back-pressure.
        let ex76 = with(|c| c.exhaust = Exhaust { pipe_mm: 76.0, cat: CatKind::Oem }).engine.gas_path;
        let decat = with(|c| c.exhaust = Exhaust { pipe_mm: 76.0, cat: CatKind::Removed });
        assert!(ex76.exhaust_k < g0.exhaust_k && decat.engine.gas_path.exhaust_k < ex76.exhaust_k);
        assert_eq!(decat.pending.cat_osc_factor, 0.0);
        // Intercooler: a thicker upgraded core cools better, holds more air and (larger charge flow area) loses less;
        // bigger charge pipes lose less, smaller ones more.
        let ic = |core, core_mm, pipe_mm| with(|c| c.intercooler = Intercooler { core, core_mm, pipe_mm }).engine;
        let ic60 = ic(IcCore::Upgraded, 60.0, STOCK_IC_PIPE_MM);
        assert!(ic60.intercooler_effectiveness > r0.engine.intercooler_effectiveness && ic60.intercooler_effectiveness < 0.95);
        assert!(ic60.boost_volume_m3 > r0.engine.boost_volume_m3 && ic60.gas_path.charge_air_dp_rel_k < 0.0);
        assert!(ic(IcCore::Upgraded, 60.0, 76.0).gas_path.charge_air_dp_rel_k < ic60.gas_path.charge_air_dp_rel_k);
        assert!(ic(IcCore::Stock, STOCK_IC_CORE_MM, 60.0).gas_path.charge_air_dp_rel_k > 0.0);
        // Ancillaries: R injectors and HPFP swap all calibrated characteristics; CJHA throttle has more area.
        let rinj = with(|c| c.ancillaries.di_injectors = Some(LongBlock::Cjxc));
        assert!(rinj.engine.injector_flow_cc_s > r0.engine.injector_flow_cc_s);
        let rhpfp = with(|c| c.ancillaries.hpfp = Some(LongBlock::Cjxc));
        let r_ref = EngineProfile::ea888_20t_cjxc_is38();
        assert_eq!(rhpfp.engine.hpfp_stroke_cc, r_ref.hpfp_stroke_cc);
        assert_eq!(rhpfp.engine.hpfp_efficiency, r_ref.hpfp_efficiency);
        assert_eq!(rhpfp.engine.hpfp_vcv_lock_mpa, r_ref.hpfp_vcv_lock_mpa);
        assert_eq!(rhpfp.engine.hpfp_vcv_close_delay_ms, r_ref.hpfp_vcv_close_delay_ms);
        let thr = with(|c| c.ancillaries.throttle = ThrottleBody::Cjha).engine.throttle_cda_cm2;
        assert!(thr.iter().zip(r0.engine.throttle_cda_cm2).all(|(a, b)| a.0 == b.0 && a.1 > b.1));
        assert_eq!(with(|c| c.ancillaries.mpi_injectors = Some(LongBlock::Cjxc)).pending.mpi_mg_ms, r0.pending.mpi_mg_ms);
        // Ethanol: higher octane, lower heating value and AFR, more latent heat.
        let e10 = fuel_props(&Fuel { ron: 92.0, ethanol_pct: 10.0 });
        let e0 = fuel_props(&Fuel { ron: 92.0, ethanol_pct: 0.0 });
        assert!(e10.ron > e0.ron && e10.ron < 97.0 && e10.lhv_mj_kg < e0.lhv_mj_kg && e10.afr_stoich < e0.afr_stoich && e10.latent_kj_kg > e0.latent_kj_kg);
        assert_eq!(with(|c| c.fuel.ron = 98.0).fuel_ron, 98.0);
        // Engine mileage: more blow-by, higher compression from deposits; every part at 0 inherits it.
        let worn_all = with(|c| c.mileage.engine_km = 200_000.0);
        let e = &worn_all.engine;
        assert!(e.mech.ring_leak_cda_m2 > r0.engine.mech.ring_leak_cda_m2 && e.compression_ratio > r0.engine.compression_ratio);
        assert!(e.injector_flow_cc_s < r0.engine.injector_flow_cc_s && e.hpfp_stroke_cc < r0.engine.hpfp_stroke_cc);
        assert!(e.mpi_injector_mg_ms < r0.engine.mpi_injector_mg_ms && (e.mpi_injector_mg_ms - worn_all.pending.mpi_mg_ms).abs() < 1e-12);
        assert!(e.mech.spark_gap_m > r0.engine.mech.spark_gap_m && e.turbo.eta_compressor < r0.engine.turbo.eta_compressor);
        assert!(worn_all.pending.mpi_mg_ms < r0.pending.mpi_mg_ms && worn_all.pending.cat_osc_factor < 1.0 && worn_all.clutch_wear_mm > 0.0);
        // A part with its own mileage: new plugs in a worn engine.
        let new_plugs = with(|c| c.mileage = Mileage { engine_km: 200_000.0, spark_plugs_km: 1.0, ..Mileage::default() });
        assert!(new_plugs.engine.mech.spark_gap_m < e.mech.spark_gap_m && new_plugs.engine.injector_flow_cc_s == e.injector_flow_cc_s);
        // Worn clutches are installed in the plant.
        let p = worn_all.into_plant();
        assert!(matches!(&p.driveline.clutches, crate::clutch::ClutchPack::Dry(s) if s.wear_mm[0] > 0.0));
        // High-mileage DQ250: 0.50 mm off each pack's lining stack at 250 000 km, linings intact.
        let mut worn_dq250 = preset("a3_8v_quattro").unwrap();
        worn_dq250.mileage.engine_km = 250_000.0;
        let p = worn_dq250.resolve().unwrap().into_plant();
        let crate::clutch::ClutchHw::Wet(h) = &p.drivetrain_profile.clutch else { panic!("wet DQ250") };
        let cond = p.driveline.clutches.condition();
        for k in 0..2 {
            assert_eq!(cond[k].0, 1.0, "lining health of K{}", k + 1);
            assert!((cond[k].2 * h[k].surfaces - 0.500).abs() < 1e-6, "stack wear K{}: {} mm", k + 1, cond[k].2 * h[k].surfaces);
        }
        // AWD adds mass and the coupling; DQ200 has no AWD.
        let awd = with(|c| (c.gearbox, c.awd) = (Gearbox::Dq250_0d9, true));
        assert!(awd.drivetrain.awd && awd.drivetrain.haldex.is_some() && awd.drivetrain.body.mass_kg > r0.drivetrain.body.mass_kg);
        let mut bad = preset("octavia_fl").unwrap();
        bad.awd = true;
        assert!(bad.resolve().is_err());
        assert!(with(|c| c.engine = LongBlock::Cjxc).engine.displacement_m3() > r0.engine.displacement_m3());
    }

    #[test]
    fn compatibility_warns_without_blocking() {
        let s50 = CalFacts { part_number: "8V0906259J".into(), engine_label: "2.0l R4 TFSI ".into(), displacement_l: Some(2.0), injector_cc_s: Some(18.97), n_tcha_max_sp_rpm: Some(165_000.0) };
        // R calibration on the Octavia A7 FL car: displacement, injector and turbo mismatch.
        let fl = preset("octavia_fl").unwrap();
        let w = compatibility(&fl, &s50);
        assert!(w.iter().any(|n| n.level == "warn" && n.ru.contains("CJXC") && n.ru.contains("CJSA") && n.ru.contains("модель воздуха")), "{w:?}");
        assert!(w.iter().any(|n| n.ru.contains("впрыск")) && w.iter().any(|n| n.ru.contains("наддув")), "{w:?}");
        assert!(fl.resolve().is_ok());
        // Same calibration on a Golf 7 R: no displacement / injector / turbo warnings.
        let r = compatibility(&preset("golf7r").unwrap(), &s50);
        assert!(r.iter().all(|n| n.level != "warn"), "{r:?}");
        // Tuned calibration with turbo protection raised to 400 000 rpm on the IS38 car.
        let tuned = CalFacts { part_number: "8V0906264H".into(), engine_label: "1.8l R4 TFSI ".into(), displacement_l: Some(1.8), injector_cc_s: Some(14.14), n_tcha_max_sp_rpm: Some(400_000.0) };
        let w = compatibility(&preset("cjsa_is38_dq250").unwrap(), &tuned);
        assert!(w.iter().any(|n| n.ru.contains("защита турбины")), "{w:?}");
        assert!(!w.iter().any(|n| n.ru.contains("модель воздуха") || n.ru.contains("впрыск")), "{w:?}");
    }
}
