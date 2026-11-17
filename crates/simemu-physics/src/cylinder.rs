//! Crank-angle-resolved quasi-dimensional cylinder.
//!
//! State: masses of fresh air, fuel vapour and combustion products, sensible internal energy U (reference 298.15 K,
//! temperature-dependent cp per species), volume from the slider crank. Between ignition and the end of the flame the
//! charge is split into an unburned zone (fresh mixture, residual gas, the end gas) and a burned zone that share one
//! pressure: each zone keeps its own temperature — the zone with the smaller heat capacity follows its own energy
//! equation (isentropic compression + wall heat + newly burned mass entering at its constant-pressure adiabatic flame
//! temperature), the other follows from the conserved total energy, and the pressure from the two equations of state.
//!
//! Per step: compressible valve flows (intake port back-flow buffer = internal EGR) carrying mean-flow, tumble and
//! turbulence into / out of the cylinder (`combustion::Turbulence`), blow-by past the rings, head-gasket leakage, DI
//! pulses (Hiroyasu–Arai penetration → piston-crown and liner films, liner fuel partly scraped into the oil), spray /
//! film evaporation with charge cooling and a turbulent mixing (stratification) state, spark (coil energy → secondary
//! voltage vs Paschen breakdown, fouled-insulator shunt) and kernel, entrainment flame (`combustion`), crevice storage,
//! thermal NO in the burned zone, Livengood–Wu end-gas integral on the unburned-zone temperature, end-gas autoignition
//! (knock, developing detonation when the end gas is large), hot-surface and oil-droplet (LSPI) pre-ignition, Hohenberg
//! wall heat (knock scrubs the boundary layer, deposits insulate), and the component nodes of `thermal` (piston crown /
//! land / groove, exhaust-valve head / seat, plug tip / ground electrode, deposit surface). Each finished cycle
//! accumulates fatigue / creep damage of the ring land (at the ring-groove temperature, single overload = fracture),
//! head-gasket bead, connecting rod and exhaust valve from the material laws in `materials`.

use crate::combustion::{self as cb, Rng, Turbulence};
use crate::crank_train::{Kinematics, Slider};
use crate::engine::{CoilCmd, Drive, InjPulse, Injector};
use crate::materials::{valve_21_4n_rupture_h, GASKET_BEAD_1_4310, PISTON_ALSI12CUNIMG, PISTON_SOLIDUS_C, ROD_36MNVS4};
use crate::profiles::Mechanics;
use serde::Serialize;
use std::f64::consts::PI;

pub const AIR: usize = 0;
pub const FUEL: usize = 1;
pub const PROD: usize = 2;
/// Specific gas constants: air, gasoline vapour (C8H18-like, 114 g/mol), stoichiometric products.
pub const R_S: [f64; 3] = [287.06, 72.8, 288.3];
pub const AFR_STOICH: f64 = 14.7;
pub const LHV_J_KG: f64 = 43.0e6;
pub const FUEL_DENSITY_KG_M3: f64 = 745.0;
const T_REF: f64 = 298.15;
const L_VAP_WATER: f64 = 2.26e6;
const T_FUEL_LIQ: f64 = 330.0;
/// Chemical energy left in CO/H2 at the frozen burned-gas composition of a stoichiometric flame (Heywood 1988 §4.9:
/// ≈ 1 %; the unburned HC are explicit — crevices, films, partial burns).
const ETA_COMB: f64 = 0.99;
/// cp of exhaust gas used by the manifold energy balance, J/(kg·K).
pub const CP_E: f64 = 1150.0;

/// Pencil coil: primary inductance, resistance, secondary capacitance (plug + lead), transfer efficiency. UNVERIFIED.
const COIL_L_H: f64 = 4e-3;
const COIL_R_OHM: f64 = 0.7;
const COIL_C_SEC_F: f64 = 50e-12;
const COIL_ETA: f64 = 0.7;
/// Laminar flame speed below which a kernel cannot survive (lean / dilution limit), m/s. UNVERIFIED.
const S_L_MIN: f64 = 0.08;
const KNOCK_DECAY_S: f64 = 0.8e-3;
/// Heat-transfer augmentation by knock: h·(1 + A/KNOCK_HEAT_PA). UNVERIFIED (×2–4 for heavy knock).
const KNOCK_HEAT_PA: f64 = 3.3e5;
/// Spray evaporation time constant at 600 K / 20 MPa, s. UNVERIFIED.
const SPRAY_TAU_S: f64 = 1.0e-3;
const NOZZLE_D_M: f64 = 1.5e-4;
/// Exhaust-valve head bending stress at 100 bar peak pressure, MPa. UNVERIFIED.
const EXV_STRESS_MPA_100BAR: f64 = 60.0;
/// MLS bead stress range when the gas force equals the fire-ring clamp, Pa. UNVERIFIED.
const BEAD_STRESS_AT_LIFTOFF: f64 = 900e6;
/// Fire-deck and liner surface temperature rise over the coolant on a firing warm engine, K. UNVERIFIED.
const HEAD_OVER_COOLANT: f64 = 80.0;
const LINER_OVER_COOLANT: f64 = 30.0;
/// The rise is carried by the gas heat flow through the wall: proportional to the running mean heat flow into the head /
/// liner of one cylinder up to these flows (≈ the plant's warm idle: 214 / 290 W at 673 rpm, 32 kPa, coolant 90 °C, test
/// `warm_start_fires_and_idles_stably_then_stops_with_compression_bounce`), the full rise above (coolant-side boiling caps
/// it, as the former constant rise did at every load); a cold-soaked or motored engine has its walls at the coolant.
/// Response time of the surface ≈ the conduction time of a few mm of aluminium / cast iron. UNVERIFIED.
const WALL_Q_REF_W: [f64; 2] = [200.0, 250.0];
const WALL_TAU_S: f64 = 2.0;
/// Liquid-fuel wall films (crown, liner): film thickness that sets the wetted area per film mass, m, and the heat per kg
/// of film the gas heat flux spends on evaporating it (vaporisation + heating to boiling), J/kg. UNVERIFIED (RIM / LIF
/// film thicknesses of impinging GDI sprays: 4–7 µm 20–60 ms after a single plume on a plate at 293 K, Dhanji et al.,
/// ILASS-Europe 2023; cold-start crown films accumulate over cycles). 20 µm out of 8–40 µm tried with the firmware in the
/// loop (`cold_start_dq250_scenario`): thinner films let the −20 °C start flood (8 µm: first run-up after 9 s of
/// cranking), thicker ones soak up the fuel of a load step on the 5 °C warm-up idle (30 µm: 1000 → 384 rpm when the DQ250
/// fills a clutch at 17 s; docs/SIMOS18_COMBUSTION.md §15).
const FILM_THICK_M: f64 = 20e-6;
const FILM_L_EFF_J_KG: f64 = 750e3;
/// Chilton–Colburn mass transfer from the wall heat-transfer coefficient: h_m·ρ = (h/c_p)·Le^(−2/3), Le ≈ 3.5 for gasoline
/// vapour in air (α/D), c_p 1100 J/(kg·K).
const FILM_HM_PER_H: f64 = 0.43 / 1100.0;
/// Share of the spray liquid that, reaching a wall, stays on it. Impact: droplets that hit the wall slower than the
/// splash threshold of a wetted wall, We_c = 1320·La^−0.18 (Bai & Gosman SAE 950283), stick and spread; faster ones
/// splash and the wall keeps on average half of them (splashed mass ratio 0.2–0.8, ibid.) — the impact speed of the
/// spray front, dS/dt at the wall = c²/(2·dist), is ≈ 25–30 m/s for the compression-stroke start pulses into the dense
/// cold charge (deposition) and 40–80 m/s for intake-stroke pulses into a 0.3–2 bar charge (splash). Wall temperature:
/// full deposition below the mid fuel's boiling point, falling to `DEPOSIT_HOT` above the heavy end's (rebound / flash;
/// Habchi 1999: no piston film above ≈ 460 K). Droplet size (SMD) UNVERIFIED; `DEPOSIT_HOT` UNVERIFIED, set so that the
/// hot crown at full load keeps about the wetting of the former model, 0.6·(1 − d/S(t)) (CJSA full-load curve,
/// `bench_tests::full_load_curve_matches_cjsa_with_knock_limited_spark`).
const DROP_SMD_M: f64 = 15e-6;
const FUEL_SURFACE_TENSION_N_M: f64 = 0.022;
const FUEL_VISCOSITY_PA_S: f64 = 5e-4; // gasoline ≈ 0.4–0.6 mPa·s at 0–20 °C
const DEPOSIT_HOT: f64 = 0.5;
/// Turbulent scalar-mixing constant C_φ (IEM: dσ²/dt = −C_φ·(ε/k)·σ²; Pope, "Turbulent Flows" 2000: C_φ ≈ 2).
const C_PHI: f64 = 2.0;
/// Share of the liner fuel film that the ring pack scrapes into the sump per cycle (the rest evaporates). UNVERIFIED.
const LINER_TO_OIL: f64 = 0.3;
/// Unoxidised share of the crevice gas that returns late in the expansion (Heywood 1988 §11.4.3: about half of the
/// crevice HC is oxidised in the cylinder).
const CREVICE_UNOX: f64 = 0.5;

/// Sensible enthalpy (ref 298.15 K) and cp of [air, fuel vapour, products], J/kg, J/(kg·K).
#[inline]
fn props(t: f64) -> ([f64; 3], [f64; 3]) {
    let ea = (-(t - 300.0) / 1100.0).exp();
    let ep = (-(t - 300.0) / 1500.0).exp();
    const EA0: f64 = 1.001_542_224; // exp((300 − 298.15)/1100)
    const EP0: f64 = 1.001_234_095; // exp((300 − 298.15)/1500)
    (
        [1305.0 * (t - T_REF) + 330_000.0 * (ea - EA0), 2400.0 * (t - T_REF), 1600.0 * (t - T_REF) + 750_000.0 * (ep - EP0)],
        [1305.0 - 300.0 * ea, 2400.0, 1600.0 - 500.0 * ep],
    )
}

/// Flow enthalpy consistent with U = Σ m·(h − R·(T − T_REF)).
#[inline]
fn flow_h(t: f64) -> [f64; 3] {
    let (h, _) = props(t);
    [h[0] + R_S[0] * T_REF, h[1] + R_S[1] * T_REF, h[2] + R_S[2] * T_REF]
}

#[inline]
fn flow_h_fuel(t: f64) -> f64 {
    2400.0 * (t - T_REF) + R_S[FUEL] * T_REF
}

#[inline]
fn internal_energy(m: &[f64; 3], t: f64) -> (f64, f64) {
    let (h, cp) = props(t);
    let mut u = 0.0;
    let mut cv = 0.0;
    for s in 0..3 {
        u += m[s] * (h[s] - R_S[s] * (t - T_REF));
        cv += m[s] * (cp[s] - R_S[s]);
    }
    (u, cv)
}

fn solve_t(m: &[f64; 3], u: f64, t0: f64) -> f64 {
    let mut t = t0.clamp(150.0, 4500.0);
    for _ in 0..4 {
        let (ui, cv) = internal_energy(m, t);
        if cv <= 1e-12 {
            return t;
        }
        let d = (u - ui) / cv;
        t = (t + d).clamp(150.0, 4500.0);
        if d.abs() < 0.05 {
            break;
        }
    }
    t
}

/// Temperature at which the sensible enthalpy per kg of composition `x` (mass fractions) equals `h`.
fn solve_h(x: &[f64; 3], h: f64, t0: f64) -> f64 {
    let mut t = t0.clamp(150.0, 4500.0);
    for _ in 0..5 {
        let (hs, cp) = props(t);
        let (hi, ci) = (x[0] * hs[0] + x[1] * hs[1] + x[2] * hs[2], x[0] * cp[0] + x[1] * cp[1] + x[2] * cp[2]);
        let d = (h - hi) / ci.max(1.0);
        t = (t + d).clamp(150.0, 4500.0);
        if d.abs() < 0.1 {
            break;
        }
    }
    t
}

#[inline]
fn mix_r(m: &[f64; 3]) -> f64 {
    let mt = m[0] + m[1] + m[2];
    if mt <= 0.0 { R_S[0] } else { (m[0] * R_S[0] + m[1] * R_S[1] + m[2] * R_S[2]) / mt }
}

#[inline]
fn mix_cp(m: &[f64; 3], t: f64) -> f64 {
    let mt = m[0] + m[1] + m[2];
    if mt <= 0.0 {
        return 1005.0;
    }
    let (_, cp) = props(t);
    (m[0] * cp[0] + m[1] * cp[1] + m[2] * cp[2]) / mt
}

/// Isentropic orifice flow (γ = 1.35, R = 287), kg/s, up → down only.
#[inline]
pub fn orifice(cda: f64, p_up: f64, t_up: f64, p_dn: f64) -> f64 {
    if cda <= 0.0 || p_up <= p_dn {
        return 0.0;
    }
    let pr = p_dn / p_up;
    let psi = if pr <= 0.5368 { 0.6761 } else { (7.714 * (pr.powf(1.4815) - pr.powf(1.7407))).max(0.0).sqrt() };
    cda * p_up / (287.0 * t_up.max(150.0)).sqrt() * psi
}

/// Signed flow a → b over `dt`, limited so that one step cannot overshoot pressure equilibrium.
#[inline]
#[allow(clippy::too_many_arguments)]
fn limited_flow(cda: f64, pa: f64, ta: f64, va: f64, pb: f64, tb: f64, vb: f64, dt: f64) -> f64 {
    if cda <= 0.0 {
        return 0.0;
    }
    let md = if pa >= pb { orifice(cda, pa, ta, pb) } else { -orifice(cda, pb, tb, pa) };
    let t = if md >= 0.0 { ta } else { tb };
    let cap = 0.5 * (pa - pb).abs() / (287.0 * t * (1.0 / va + 1.0 / vb));
    (md * dt).clamp(-cap, cap)
}

/// Local crank angle (0..720) → angle relative to firing TDC in (−360, 360].
#[inline]
pub fn atdc(phi: f64) -> f64 {
    if phi > 360.0 { phi - 720.0 } else { phi }
}

/// True when `target` (0..720) was passed going forward from `a` to `b`.
#[inline]
pub(crate) fn crossed(a: f64, b: f64, target: f64) -> bool {
    let step = (b - a).rem_euclid(720.0);
    let d = (target - a).rem_euclid(720.0);
    step > 0.0 && step < 360.0 && d > 0.0 && d <= step
}

/// Published per-cylinder results of the last completed cycle (boundary: 180 °CA after firing TDC).
#[derive(Debug, Clone, Copy, Serialize, Default)]
pub struct CycleStats {
    /// Peak mean cylinder pressure, Pa abs, and its angle, °ATDC.
    pub p_max_pa: f64,
    pub p_max_deg: f64,
    /// Peak local wall pressure (mean + knock mode amplitude), Pa.
    pub p_local_max_pa: f64,
    /// Net / gross indicated mean effective pressure and pumping MEP (positive = loss), Pa.
    pub imep_pa: f64,
    pub gross_imep_pa: f64,
    pub pmep_pa: f64,
    /// 10 / 50 / 90 % mass-fraction-burned angles, °ATDC (0 when not burned).
    pub ca10_deg: f64,
    pub ca50_deg: f64,
    pub ca90_deg: f64,
    /// Ignition angle of the cycle (spark or pre-ignition), °ATDC.
    pub ign_deg: f64,
    /// Peak knock oscillation amplitude, Pa, and knock onset, °ATDC (0 = none).
    pub knock_amp_pa: f64,
    pub knock_onset_deg: f64,
    /// Unburned mass fraction at knock onset.
    pub knock_unburned: f64,
    /// Livengood–Wu integral reached in the cycle (≥ 1 → end-gas autoignition).
    pub lw: f64,
    pub burned: bool,
    /// The spark did not light a kernel (breakdown failure, quench) although fuel was present.
    pub misfire: bool,
    /// Pre-ignition before the spark: hot surface / oil-fuel droplet (LSPI), and its angle °ATDC.
    pub preignition: bool,
    pub lspi: bool,
    pub preign_deg: f64,
    /// Trapped fresh air at IVC and fuel injected, kg.
    pub air_kg: f64,
    pub fuel_kg: f64,
    /// In-cylinder λ at spark (incl. liquid fuel).
    pub lambda: f64,
    /// RMS first-mode oscillation in the knock window (5–70 °ATDC), bar.
    pub knock_rms_bar: f64,
    /// Peak connecting-rod compression / tension, N.
    pub rod_comp_n: f64,
    pub rod_tens_n: f64,
    /// Residual (product) mass fraction at IVC.
    pub residual: f64,
    /// Turbulence intensity and laminar flame speed at ignition, m/s; stratification index at ignition (unmixed share of
    /// the fuel vapour); liquid fuel film on the crown at ignition, kg.
    pub u_prime: f64,
    pub s_l: f64,
    pub stratification: f64,
    pub film_kg: f64,
    /// Peak unburned (end-gas) and burned-zone temperatures, K.
    pub t_u_max: f64,
    pub t_b_max: f64,
    /// Engine-out NO (ppm), CO (% vol), unburned HC (ppm C1) and a soot index (rich-pocket + pool-fire fuel, mg).
    pub no_ppm: f64,
    pub co_pct: f64,
    pub hc_ppm: f64,
    pub soot_mg: f64,
    /// Ring-land peak stress over the land's UTS at its temperature.
    pub land_stress_ratio: f64,
    pub t_end_s: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Default)]
struct Acc {
    w_net: f64,
    w_gross: f64,
    p_max: f64,
    p_max_deg: f64,
    p_loc_max: f64,
    ca10: f64,
    ca50: f64,
    ca90: f64,
    ign: f64,
    knock_amp: f64,
    knock_onset: f64,
    knock_unburned: f64,
    burned: bool,
    misfire: bool,
    preign: bool,
    lspi: bool,
    preign_deg: f64,
    fuel: f64,
    air: f64,
    lambda: f64,
    ko_a2: f64,
    ko_t: f64,
    rod_c: f64,
    rod_t: f64,
    residual: f64,
    u_prime: f64,
    s_l: f64,
    strat: f64,
    film: f64,
    t_u_max: f64,
    t_b_max: f64,
    no_ppm: f64,
    co_pct: f64,
    hc_ppm: f64,
    soot: f64,
    evo_done: bool,
}

/// Inputs of one cylinder step.
pub struct CylIn {
    pub dt: f64,
    pub omega: f64,
    pub phi0: f64,
    pub phi1: f64,
    pub s: Slider,
    pub v1: f64,
    pub cda_in: f64,
    pub cda_ex: f64,
    pub in_open: bool,
    pub ex_open: bool,
    pub p_im: f64,
    pub t_im: f64,
    pub v_im: f64,
    /// Mass fractions [air, fuel, products] of the manifold gas (crankcase ventilation returns blow-by into it).
    pub x_im: [f64; 3],
    pub p_em: f64,
    pub t_em: f64,
    pub x_em: [f64; 3],
    pub v_em: f64,
    pub p_cc: f64,
    pub p_coolant: f64,
    pub t_coolant_k: f64,
    pub t_oil_k: f64,
    pub rail_pa: f64,
    pub coil: Option<CoilCmd>,
    pub pulses: [Option<InjPulse>; 3],
    /// Timed coil and DI drives ([`crate::engine::TimedOutputs`]); when present they replace `coil` / `pulses`.
    pub coil_timed: Option<[Option<Drive>; 3]>,
    pub di_timed: Option<[Option<Drive>; 3]>,
    pub battery_v: f64,
    pub ron: f64,
    /// Port (tumble) flaps closed.
    pub tumble: bool,
    /// Piston-cooling-jet effectiveness 0..1 (oil pressure, N522).
    pub jet: f64,
    pub injector_m3_s_10mpa: f64,
    pub dead_s: f64,
    pub time_s: f64,
    /// Fuel blend: stoichiometric AFR, lower heating value J/kg, heat of vaporisation J/kg, ethanol mass fraction.
    pub afr: f64,
    pub lhv: f64,
    pub latent: f64,
    pub w_eth: f64,
    /// Deposit thickness on crown/head of this cylinder, m; ring-pack leakage factor of this cylinder.
    pub deposit_m: f64,
    pub ring_factor: f64,
    /// Fuel mass fraction in the engine oil.
    pub oil_dilution: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct CylOut {
    /// Fresh air drawn from the intake manifold, kg (≥ 0).
    pub m_from_im: f64,
    /// Species to the exhaust manifold, kg (negative = drawn back), and energy (cp_e·T·dm), J.
    pub m_to_em: [f64; 3],
    pub e_to_em: f64,
    /// Blow-by to the crankcase, kg, and its species [air, fuel, products], kg.
    pub m_bb: f64,
    pub m_bb_x: [f64; 3],
    /// Heat to coolant / oil, J.
    pub q_coolant: f64,
    pub q_oil: f64,
    /// Fuel volume taken from the rail, m³.
    pub v_fuel: f64,
    /// Liquid fuel scraped from the liner into the sump, kg.
    pub fuel_to_oil: f64,
    /// Read-only for the acoustic probe: signed intake-valve flow (+ = into the cylinder), kg, and the
    /// time offsets within the step at which an injector started / stopped flowing, s.
    pub m_in_valve: f64,
    pub inj_open: Option<f64>,
    pub inj_close: Option<f64>,
    /// First-mode pressure oscillation of a knock event that started in this step, Pa (0 = none); read by the
    /// block-vibration model (`knock_sensor`).
    pub knock_pa: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct Cylinder {
    pub m: [f64; 3],
    pub u: f64,
    /// Mass-averaged temperature, K (= the single-zone temperature outside combustion).
    pub t: f64,
    pub p: f64,
    pub v: f64,
    pub spray_kg: f64,
    /// Liquid fuel film on the piston crown, kg, and on the liner, kg (totals of `film_c` / `liner_c`).
    pub film_kg: f64,
    pub liner_film_kg: f64,
    /// The same films by fuel pseudo-component (`combustion::FUEL_PSEUDO`: light / mid / heavy), kg.
    pub film_c: [f64; 3],
    pub liner_c: [f64; 3],
    pub water_kg: f64,
    bf_m: [f64; 3],
    bf_t: f64,
    in_open_prev: bool,
    /// Flame in progress (two-zone state valid).
    pub burning: bool,
    /// The coil has discharged in this cycle; dwell start of the last timed coil drive that fired.
    spark_done: bool,
    spark_on_s: f64,
    /// Burned-zone species, kg; unburned- and burned-zone temperatures, K.
    mb: [f64; 3],
    pub t_u: f64,
    pub t_b: f64,
    /// Entrained mass (burned + entrained-not-yet-burned), flame radius, charge at ignition, crevice-held unburned mass.
    m_e: f64,
    pub r_flame_m: f64,
    m0: f64,
    m_crev: f64,
    p_prev: f64,
    /// Mass fraction burned of the charge at ignition.
    pub xb: f64,
    /// Mean flow / tumble / turbulence of the charge.
    pub turb: Turbulence,
    /// Fuel vapour not yet mixed at the molecular scale, kg (stratification).
    vap_unmixed: f64,
    /// Cycle-to-cycle variation drawn at this cycle's spark: relative deviation of the turbulence intensity and of the
    /// equivalence ratio at the plug (`CombustionHw::ccv_turb_sigma` / `ccv_phi_sigma`).
    ccv_u: f64,
    ccv_phi: f64,
    pub lw: f64,
    /// Current first-mode knock oscillation envelope, Pa.
    pub knock_amp_pa: f64,
    knocked: bool,
    /// DI pulses in progress; per pulse the spray penetration coefficient (S = c·√t, Hiroyasu–Arai), the distance
    /// injector → piston at its start and the share of the wetting that lands on the liner.
    inj: Injector,
    inj_pen: [f64; 3],
    inj_dist: [f64; 3],
    inj_liner: [f64; 3],
    /// Share of the liquid reaching the wall that stays (impact regime), per pulse.
    inj_stick: [f64; 3],
    /// Piston crown centre / edge + top land / first ring groove, exhaust-valve head / seat, plug insulator tip / ground
    /// electrode, deposit surface, K.
    pub t_piston_k: f64,
    pub t_land_k: f64,
    pub t_groove_k: f64,
    pub t_exv_k: f64,
    pub t_exv_seat_k: f64,
    pub t_plug_tip_k: f64,
    pub t_plug_gnd_k: f64,
    pub t_deposit_k: f64,
    /// Plug carbon fouling 0..1.
    pub plug_fouling: f64,
    /// Running cycle-mean gas-side heat-transfer coefficient (for the deposit series resistance), W/(m²·K).
    h_mean: f64,
    /// Running mean gas heat flow into the head (fire deck) and the liner, W (their surface rise, `wall_temps`).
    pub wall_q_w: [f64; 2],
    /// Gas heat flux into the crown and the liner in the last step, W/m² (heats the fuel films on them).
    film_flux: [f64; 2],
    /// NO mass in the cylinder, kg.
    no_kg: f64,
    /// Hot-surface ignition integral; LSPI droplet: armed release angle (local °CA, < 0 = none), age, chemistry integral.
    hs_int: f64,
    lspi_at: f64,
    lspi_age: f64,
    lspi_int: f64,
    rng: Rng,
    /// Event counters since start: pre-ignitions (all), of them LSPI, knock cycles, misfires.
    pub preign_events: u32,
    pub lspi_events: u32,
    pub knock_events: u32,
    pub misfire_events: u32,
    /// Highest ring-land stress / UTS seen.
    pub land_stress_max: f64,
    /// Ring-land Miner damage (≥ 1 or crown at solidus → land fractured / crown holed).
    pub piston_damage: f64,
    pub land_failed: bool,
    /// Head-gasket bead/fire-ring damage and leak area, m².
    pub gasket_damage: f64,
    pub gasket_leak_m2: f64,
    /// Plastic rod shortening (bend), m, rod fatigue damage, fracture flag.
    pub rod_short_m: f64,
    pub rod_damage: f64,
    pub rod_broken: bool,
    /// Exhaust-valve creep life consumed (≥ 1 → head necks off).
    pub exv_creep: f64,
    /// Instantaneous blow-by, kg/s.
    pub blowby_kg_s: f64,
    acc: Acc,
    pub last: CycleStats,
}

impl Cylinder {
    pub fn new(k: &Kinematics, phi_deg: f64, p: f64, t: f64, seed: u64) -> Self {
        let v = k.volume(&k.eval(phi_deg.to_radians(), 0.0));
        let m = [p * v / (R_S[0] * t), 0.0, 0.0];
        let (u, _) = internal_energy(&m, t);
        Self {
            m,
            u,
            t,
            p,
            v,
            spray_kg: 0.0,
            film_kg: 0.0,
            liner_film_kg: 0.0,
            film_c: [0.0; 3],
            liner_c: [0.0; 3],
            water_kg: 0.0,
            bf_m: [0.0; 3],
            bf_t: t,
            in_open_prev: false,
            burning: false,
            spark_done: false,
            spark_on_s: f64::NEG_INFINITY,
            mb: [0.0; 3],
            t_u: t,
            t_b: t,
            m_e: 0.0,
            r_flame_m: 0.0,
            m0: 0.0,
            m_crev: 0.0,
            p_prev: p,
            xb: 0.0,
            turb: Turbulence::default(),
            vap_unmixed: 0.0,
            ccv_u: 0.0,
            ccv_phi: 0.0,
            lw: 0.0,
            knock_amp_pa: 0.0,
            knocked: false,
            inj: Injector::default(),
            inj_pen: [0.0; 3],
            inj_dist: [0.0; 3],
            inj_liner: [0.0; 3],
            inj_stick: [1.0; 3],
            t_piston_k: t,
            t_land_k: t,
            t_groove_k: t,
            t_exv_k: t,
            t_exv_seat_k: t,
            t_plug_tip_k: t,
            t_plug_gnd_k: t,
            t_deposit_k: t,
            plug_fouling: 0.0,
            h_mean: 300.0,
            wall_q_w: [0.0; 2],
            film_flux: [0.0; 2],
            no_kg: 0.0,
            hs_int: 0.0,
            lspi_at: -1.0,
            lspi_age: 0.0,
            lspi_int: 0.0,
            rng: Rng(seed),
            preign_events: 0,
            lspi_events: 0,
            knock_events: 0,
            misfire_events: 0,
            land_stress_max: 0.0,
            piston_damage: 0.0,
            land_failed: false,
            gasket_damage: 0.0,
            gasket_leak_m2: 0.0,
            rod_short_m: 0.0,
            rod_damage: 0.0,
            rod_broken: false,
            exv_creep: 0.0,
            blowby_kg_s: 0.0,
            acc: Acc::default(),
            last: CycleStats::default(),
        }
    }

    /// Sets every component node to a warm-engine state (test benches): piston nodes, valve, plug, deposit.
    /// Every component node at `t_k` (a stopped, soaked engine).
    pub fn set_component_temps_uniform(&mut self, t_k: f64) {
        (self.t_piston_k, self.t_land_k, self.t_groove_k, self.t_exv_k, self.t_exv_seat_k) = (t_k, t_k, t_k, t_k, t_k);
        (self.t_plug_tip_k, self.t_plug_gnd_k, self.t_deposit_k) = (t_k, t_k, t_k);
    }

    pub fn set_component_temps(&mut self, piston_k: f64, exhaust_valve_k: f64) {
        (self.t_piston_k, self.t_land_k, self.t_groove_k) = (piston_k, piston_k - 25.0, piston_k - 60.0);
        (self.t_exv_k, self.t_exv_seat_k) = (exhaust_valve_k, exhaust_valve_k - 150.0);
        (self.t_plug_tip_k, self.t_plug_gnd_k, self.t_deposit_k) = (piston_k + 150.0, piston_k + 200.0, piston_k);
    }

    /// Fire-deck and liner surface temperatures at coolant `t_coolant_k`, K (`WALL_Q_REF_W`).
    pub fn wall_temps(&self, t_coolant_k: f64) -> [f64; 2] {
        let rise = [HEAD_OVER_COOLANT, LINER_OVER_COOLANT];
        std::array::from_fn(|k| t_coolant_k + rise[k] * (self.wall_q_w[k] / WALL_Q_REF_W[k]).clamp(0.0, 1.0))
    }

    pub fn mass(&self) -> f64 {
        self.m[0] + self.m[1] + self.m[2]
    }

    /// Liquid (coolant water + crown film) volume occupying the combustion chamber, m³.
    pub fn liquid_volume(&self) -> f64 {
        self.water_kg / 1000.0 + self.film_kg / FUEL_DENSITY_KG_M3
    }

    /// Density of the unburned charge (the whole charge outside the flame), kg/m³.
    #[inline]
    fn rho_u(&self) -> f64 {
        if self.burning {
            self.p / (mix_r(&self.mu()) * self.t_u.max(150.0))
        } else {
            self.mass() / (self.v - self.liquid_volume()).max(1e-7)
        }
    }

    #[inline]
    fn mb_total(&self) -> f64 {
        self.mb[0] + self.mb[1] + self.mb[2]
    }

    #[inline]
    fn mu(&self) -> [f64; 3] {
        [(self.m[0] - self.mb[0]).max(0.0), (self.m[1] - self.mb[1]).max(0.0), (self.m[2] - self.mb[2]).max(0.0)]
    }

    /// Removes `dm` proportionally from both zones (blow-by, exhaust, back-flow).
    fn take(&mut self, dm: f64) -> [f64; 3] {
        let mt = self.mass();
        if dm <= 0.0 || mt <= 0.0 {
            return [0.0; 3];
        }
        let f = (dm / mt).min(0.95);
        let out = [self.m[0] * f, self.m[1] * f, self.m[2] * f];
        if self.burning {
            let mu = self.mu();
            let (hu, hb) = (flow_h(self.t_u), flow_h(self.t_b));
            for s in 0..3 {
                self.u -= f * (mu[s] * hu[s] + self.mb[s] * hb[s]);
                self.m[s] -= out[s];
                self.mb[s] *= 1.0 - f;
            }
            self.m_e *= 1.0 - f;
            self.m_crev *= 1.0 - f;
        } else {
            let hf = flow_h(self.t);
            for s in 0..3 {
                self.m[s] -= out[s];
                self.u -= out[s] * hf[s];
            }
        }
        self.turb.outflow(f);
        self.no_kg *= 1.0 - f;
        out
    }

    fn add(&mut self, dm: [f64; 3], t_in: f64) {
        let hf = flow_h(t_in);
        for s in 0..3 {
            self.m[s] += dm[s];
            self.u += dm[s] * hf[s];
        }
    }

    /// Port fuel drawn in with the intake charge (vapour at the charge temperature `t_k`; the port wall supplied
    /// its heat of vaporisation), counted as fuel of this cycle. Premixed with the air in the port.
    pub fn induct_port_fuel(&mut self, dm: f64, t_k: f64) {
        self.m[FUEL] += dm;
        self.u += dm * flow_h_fuel(t_k);
        self.acc.fuel += dm;
    }

    /// Records the connecting-rod axial force (compression positive) for the cycle fatigue check.
    pub fn record_rod_force(&mut self, f: f64) {
        self.acc.rod_c = self.acc.rod_c.max(f);
        self.acc.rod_t = self.acc.rod_t.max(-f);
    }

    /// Ring-pack leakage Cd·A, m²: ~25× when the land has fractured. A fatigue crack growing in the land stays closed under
    /// the gas load until the land breaks (Fitton & Nates 1992: the failure is the breakage of the land, followed by seizure
    /// or intense blow-by); the former growth 1 + 30·D² before fracture made a 97 %-damaged land leak more than a broken one,
    /// stopped the knock that was damaging it and froze the damage below 1 (with the M124 strength data).
    pub fn ring_cda(&self, mech: &Mechanics) -> f64 {
        if self.land_failed {
            25.0 * mech.ring_leak_cda_m2
        } else {
            mech.ring_leak_cda_m2
        }
    }

    /// Burns `dm` of the unburned zone (mixture as it is): its fuel up to the oxygen available becomes products and
    /// releases `ETA_COMB·LHV` per kg; the mass joins the burned zone at its constant-pressure adiabatic flame
    /// temperature. Returns the heat released, J.
    fn burn_zone(&mut self, dm: f64, i: &CylIn) -> f64 {
        let mu = self.mu();
        let m_u: f64 = mu.iter().sum();
        if dm <= 0.0 || m_u <= 0.0 {
            return 0.0;
        }
        let dm = dm.min(m_u);
        let x = [mu[0] / m_u, mu[1] / m_u, mu[2] / m_u];
        let fb = dm * x[FUEL].min(x[AIR] / i.afr);
        let mb_old = self.mb_total();
        let part = [dm * x[0] - i.afr * fb, dm * x[1] - fb, dm * x[2] + (1.0 + i.afr) * fb];
        for (mb, &p) in self.mb.iter_mut().zip(&part) {
            *mb += p;
        }
        self.m[FUEL] -= fb;
        self.m[AIR] -= i.afr * fb;
        self.m[PROD] += (1.0 + i.afr) * fb;
        let q = ETA_COMB * i.lhv * fb;
        self.u += q;
        // adiabatic flame temperature of the new burned mass (constant pressure): h_b(T_ad) = h_u(T_u) + q/dm
        let (hs, _) = props(self.t_u);
        let h_u = x[0] * hs[0] + x[1] * hs[1] + x[2] * hs[2];
        let xb = [part[0] / dm, part[1] / dm, part[2] / dm];
        let t_ad = solve_h(&xb, h_u + q / dm, self.t_b.max(1800.0));
        self.t_b = (mb_old * self.t_b + dm * t_ad) / (mb_old + dm);
        self.acc.t_b_max = self.acc.t_b_max.max(t_ad);
        q
    }

    /// Lights a flame kernel of radius `r0` at the current state (spark or pre-ignition).
    fn ignite(&mut self, r0: f64, k: &Kinematics, mech: &Mechanics, i: &CylIn, angle: f64) {
        let v_gas = (self.v - self.liquid_volume()).max(1e-7);
        self.burning = true;
        self.acc.burned = true;
        self.acc.ign = angle;
        self.mb = [0.0; 3];
        self.t_u = self.t;
        self.t_b = self.t;
        self.p_prev = self.p;
        self.m0 = self.mass();
        self.xb = 0.0;
        self.m_crev = 0.0;
        let h = v_gas / k.area;
        let (v0, _) = cb::flame_volume_area(r0, mech.combustion.plug_depth_m.min(0.5 * h), 0.5 * k.bore, h);
        let rho_u = self.mass() / v_gas;
        let dm = (rho_u * v0).min(0.05 * self.m0);
        self.burn_zone(dm, i);
        self.m_e = self.mb_total();
        self.r_flame_m = r0;
        self.update_zones(v_gas, 1.0, 0.0, 0.0);
    }

    /// Two-zone closure after all mass/energy changes of the step (see the module doc). `p_ratio` is the pressure
    /// ratio over the step used by the zone that follows its own energy equation, `q_u`/`q_b` the zones' wall heat, J.
    fn update_zones(&mut self, v_gas: f64, p_ratio: f64, q_u: f64, q_b: f64) {
        let mu = self.mu();
        let mb = self.mb;
        let m_u: f64 = mu.iter().sum();
        let m_b: f64 = mb.iter().sum();
        if m_b <= 0.0 {
            self.t = solve_t(&self.m, self.u, self.t);
            self.t_u = self.t;
            self.p = (self.mass() * mix_r(&self.m) * self.t / v_gas).max(50.0);
            return;
        }
        let (r_u, r_b) = (mix_r(&mu), mix_r(&mb));
        let (_, cv_u) = internal_energy(&mu, self.t_u);
        let (_, cv_b) = internal_energy(&mb, self.t_b);
        if m_u <= 1e-12 {
            self.t_b = solve_t(&mb, self.u, self.t_b);
            self.t_u = self.t_b;
        } else if cv_b < 0.25 * cv_u {
            // small burned zone: its temperature from its own equation, the unburned zone from the energy balance
            let cp_b = mix_cp(&mb, self.t_b);
            self.t_b = (self.t_b * p_ratio.max(1e-3).powf(r_b / cp_b) - q_b / (m_b * cp_b)).clamp(150.0, 4500.0);
            let (ub, _) = internal_energy(&mb, self.t_b);
            self.t_u = solve_t(&mu, self.u - ub, self.t_u);
        } else {
            let cp_u = mix_cp(&mu, self.t_u);
            self.t_u = (self.t_u * p_ratio.max(1e-3).powf(r_u / cp_u) - q_u / (m_u * cp_u)).clamp(150.0, 4500.0);
            let (uu, _) = internal_energy(&mu, self.t_u);
            self.t_b = solve_t(&mb, self.u - uu, self.t_b);
        }
        self.p = ((m_u * r_u * self.t_u + m_b * r_b * self.t_b) / v_gas).max(50.0);
        self.t = (m_u * self.t_u + m_b * self.t_b) / (m_u + m_b).max(1e-15);
    }

    /// Ends the flame: whatever is left unburned stays as unburned charge (HC / afterburning in the manifold).
    fn end_burn(&mut self) {
        self.burning = false;
        self.mb = [0.0; 3];
        self.m_e = 0.0;
        self.t = solve_t(&self.m, self.u, self.t);
        self.t_u = self.t;
        self.t_b = self.t;
    }

    /// Advances the cylinder from `phi0` to `phi1`.
    pub fn step(&mut self, k: &Kinematics, mech: &Mechanics, i: &CylIn) -> CylOut {
        let ch = &mech.combustion;
        let mut o = CylOut::default();
        let dt = i.dt;
        let v_gas = (self.v - self.liquid_volume()).max(1e-7);
        let rho_before = self.rho_u();
        let dphi = (i.phi1 - i.phi0 + 360.0).rem_euclid(720.0) - 360.0;
        let a = atdc(i.phi1);

        // ---- Gas exchange ---------------------------------------------------------------
        let m_tot = self.mass().max(1e-12);
        let dm_in = limited_flow(i.cda_in, i.p_im, i.t_im, i.v_im, self.p, self.t, v_gas, dt).max(-0.3 * m_tot);
        o.m_in_valve = dm_in;
        if dm_in > 0.0 {
            // intake jet through the valve curtain: mean flow and tumble (`combustion::Turbulence`)
            let rho_up = (i.p_im / (287.0 * i.t_im)).max(0.05);
            let v_jet = (dm_in / dt / (rho_up * i.cda_in.max(1e-7))).min(600.0);
            self.turb.inflow(dm_in, v_jet, ch.tumble_port[i.tumble as usize], k.bore);
            let bf = self.bf_m[0] + self.bf_m[1] + self.bf_m[2];
            let from_bf = dm_in.min(bf);
            if from_bf > 0.0 {
                let f = from_bf / bf;
                let part = [self.bf_m[0] * f, self.bf_m[1] * f, self.bf_m[2] * f];
                for (bfm, &p) in self.bf_m.iter_mut().zip(&part) {
                    *bfm -= p;
                }
                self.add(part, self.bf_t);
            }
            let fresh = dm_in - from_bf;
            // heat pick-up in the port and at the valve on the way in (`profiles::IntakeHeatHw`)
            let t_wall = i.t_coolant_k + mech.intake_heat.wall_over_coolant_k;
            let eps = mech.intake_heat.effectiveness(fresh / dt, 0.5 * (i.t_im + t_wall), i.tumble);
            let t_in = i.t_im + eps * (t_wall - i.t_im);
            self.add(i.x_im.map(|x| x * fresh), t_in);
            o.q_coolant -= fresh * 1005.0 * (t_in - i.t_im);
            o.m_from_im += fresh;
        } else if dm_in < 0.0 {
            let t_out = self.t;
            let out = self.take(-dm_in);
            let bf = self.bf_m[0] + self.bf_m[1] + self.bf_m[2];
            let dm = out[0] + out[1] + out[2];
            self.bf_t = (self.bf_t * bf + t_out * dm) / (bf + dm).max(1e-15);
            for (bfm, &os) in self.bf_m.iter_mut().zip(&out) {
                *bfm += os;
            }
        }
        let m_tot = self.mass().max(1e-12);
        let dm_ex = limited_flow(i.cda_ex, self.p, self.t, v_gas, i.p_em, i.t_em, i.v_em, dt).min(0.3 * m_tot);
        if dm_ex > 0.0 {
            let t_out = self.t;
            let out = self.take(dm_ex);
            for (em, &os) in o.m_to_em.iter_mut().zip(&out) {
                *em += os;
            }
            o.e_to_em += CP_E * t_out * dm_ex;
        } else if dm_ex < 0.0 {
            let dm = -dm_ex;
            let rho_up = (i.p_em / (288.0 * i.t_em)).max(0.05);
            let v_jet = (dm / dt / (rho_up * i.cda_ex.max(1e-7))).min(600.0);
            self.turb.inflow(dm, v_jet, 0.0, k.bore);
            let part = [i.x_em[0] * dm, i.x_em[1] * dm, i.x_em[2] * dm];
            self.add(part, i.t_em);
            for (em, &ps) in o.m_to_em.iter_mut().zip(&part) {
                *em -= ps;
            }
            o.e_to_em -= CP_E * i.t_em * dm;
        }
        // Blow-by past the rings (crankcase ≈ ambient).
        let bb = (orifice(self.ring_cda(mech) * i.ring_factor, self.p, self.t, i.p_cc) * dt).min(0.2 * self.mass());
        self.blowby_kg_s = bb / dt;
        if bb > 0.0 {
            o.m_bb_x = self.take(bb);
            o.m_bb += bb;
        }
        // Head-gasket breach: combustion gas into the coolant jacket, coolant into the cylinder.
        if self.gasket_leak_m2 > 0.0 {
            if self.p > i.p_coolant {
                let t_g = self.t;
                let dm = (orifice(0.6 * self.gasket_leak_m2, self.p, self.t, i.p_coolant) * dt).min(0.2 * self.mass());
                let out = self.take(dm);
                o.q_coolant += (out[0] + out[1] + out[2]) * 1100.0 * (t_g - i.t_coolant_k).max(0.0);
            } else {
                self.water_kg += 0.6 * self.gasket_leak_m2 * (2000.0 * (i.p_coolant - self.p)).sqrt() * dt;
            }
        }

        // ---- DI injection, spray, crown and liner films ------------------------------------
        let started = self.inj.schedule(&i.pulses, i.di_timed.as_ref(), i.phi0, i.phi1, i.omega, i.time_s, dt);
        for j in (0..3).filter(|&j| started[j]) {
            // Hiroyasu–Arai penetration S = c·√t vs distance injector → piston at the start of the pulse; the liquid gets
            // that far in cold gas and evaporates short of it in hot gas: maximum liquid penetration of a GDI spray (iso-
            // octane, 20 MPa, 0.77 ms; Yue & Reitz, ICLASS 2018 fig. 8) over S_HA(2·t_inj) is ≈ 0.88 / 0.71 / 0.58 at 400 /
            // 500 / 600 K gas (3–9 kg/m³), 1 in cold gas.
            let rho_g = (self.mass() / v_gas).max(0.3);
            let dp = (i.rail_pa - self.p).max(1e5);
            let reach = (0.88 - 0.00155 * (self.t - 400.0)).clamp(0.2, 1.0);
            self.inj_pen[j] = 2.95 * (dp / rho_g).powf(0.25) * NOZZLE_D_M.sqrt() * reach;
            self.inj_dist[j] = i.s.x + k.v_clear / k.area + 0.01;
            // with the piston far down the side-mounted injector's plumes reach the opposite liner instead
            // of the crown (EA888 Gen3 injector under the intake port, SSP 606). UNVERIFIED share.
            self.inj_liner[j] = ((i.s.x - 0.03) / 0.05).clamp(0.0, 0.5);
            let v_hit = self.inj_pen[j].powi(2) / (2.0 * self.inj_dist[j]);
            let we = FUEL_DENSITY_KG_M3 * v_hit * v_hit * DROP_SMD_M / FUEL_SURFACE_TENSION_N_M;
            let la = FUEL_DENSITY_KG_M3 * FUEL_SURFACE_TENSION_N_M * DROP_SMD_M / (FUEL_VISCOSITY_PA_S * FUEL_VISCOSITY_PA_S);
            let we_c = 1320.0 * la.powf(-0.18);
            self.inj_stick[j] = 1.0 - 0.5 * ((we - we_c) / we_c).clamp(0.0, 1.0);
        }
        if self.inj.slots.iter().any(|s| s.is_some()) {
            let q_static = i.injector_m3_s_10mpa * ((i.rail_pa - self.p).max(0.0) / 1e7).sqrt();
            let t_liner = self.wall_temps(i.t_coolant_k)[1];
            let wc = cb::fuel_components(i.w_eth);
            for j in 0..3 {
                let Some(s) = self.inj.slots[j] else { continue };
                let (a0, b0) = (s.tau(i.time_s), s.tau(i.time_s + dt));
                if a0 <= i.dead_s && i.dead_s < b0 && i.dead_s < s.w_s {
                    o.inj_open = Some(s.on_s + i.dead_s - i.time_s);
                }
                if a0 < s.w_s && s.w_s <= b0 && i.dead_s < s.w_s {
                    o.inj_close = Some(s.on_s + s.w_s - i.time_s);
                }
                let dm = FUEL_DENSITY_KG_M3 * q_static * (b0 - a0.max(i.dead_s)).max(0.0);
                // Liquid reaching the wall: the liquid keeps travelling after the end of the pulse (the maximum liquid
                // penetration above ≈ S_HA(2·t_inj)), so the share of a pulse of width t that arrives is the share injected
                // after the liquid front has reached the wall, 1 − t_arrival/(2t) = 1 − (dist/S(2t))² (S ∝ √t). Released as
                // the pulse goes: the part of the fuel delivered up to t is g(t)·(t − dead), so a pulse whose end is not known
                // yet splits exactly as the complete pulse does. Of it the wall keeps the deposited share (impact regime of the
                // pulse, wall temperature; `DEPOSIT_HOT`).
                let reach_upto = |t: f64| (1.0 - (self.inj_dist[j] / (self.inj_pen[j] * (2.0 * t).sqrt())).powi(2)).clamp(0.0, 1.0) * (t - i.dead_s).max(0.0);
                let reached = (FUEL_DENSITY_KG_M3 * q_static * (reach_upto(b0) - reach_upto(a0))).clamp(0.0, dm);
                let deposit = |t_w: f64| self.inj_stick[j] * (1.0 - (1.0 - DEPOSIT_HOT) * ((t_w - cb::FUEL_PSEUDO[1].1) / (cb::FUEL_PSEUDO[2].1 - cb::FUEL_PSEUDO[1].1)).clamp(0.0, 1.0));
                let to_liner = reached * self.inj_liner[j] * deposit(t_liner);
                let to_crown = reached * (1.0 - self.inj_liner[j]) * deposit(self.t_piston_k);
                self.spray_kg += dm - to_liner - to_crown;
                for ((lc, fc), &w_comp) in self.liner_c.iter_mut().zip(self.film_c.iter_mut()).zip(&wc) {
                    *lc += to_liner * w_comp;
                    *fc += to_crown * w_comp;
                }
                o.v_fuel += dm / FUEL_DENSITY_KG_M3;
                self.acc.fuel += dm;
                if i.time_s + dt >= s.on_s + s.w_s {
                    self.inj.slots[j] = None;
                }
            }
        }
        let latent = i.latent;
        if self.spray_kg > 0.0 {
            let rt = 600.0 / self.t.max(150.0);
            let rp = 2e7 / i.rail_pa.max(1e6);
            let tau = SPRAY_TAU_S * (rt * rt.sqrt()) * rp.sqrt().sqrt();
            let mut dm = self.spray_kg * (1.0 - (-dt / tau).exp());
            self.spray_kg -= dm;
            if self.spray_kg < 1e-24 {
                dm += self.spray_kg;
                self.spray_kg = 0.0;
            }
            let hf = flow_h_fuel(T_FUEL_LIQ) - latent;
            self.m[FUEL] += dm;
            self.u += dm * hf;
            self.vap_unmixed += dm;
        }
        // Wall films by pseudo-component (`combustion::film_evaporate`): mass transfer from the wall heat-transfer
        // coefficient, the gas heat flux into the film (last step) boiling it off. The crown's heat of vaporisation comes
        // out of the piston, the liner's out of the charge.
        let mut q_film = 0.0;
        let k_diff = self.h_mean * FILM_HM_PER_H / (FUEL_DENSITY_KG_M3 * FILM_THICK_M);
        let k_heat = |q: f64| q.max(0.0) / (FUEL_DENSITY_KG_M3 * FILM_THICK_M * FILM_L_EFF_J_KG);
        let dm: f64 = cb::film_evaporate(&mut self.film_c, self.t_piston_k, self.p, k_diff, k_heat(self.film_flux[0]), dt).iter().sum();
        if dm > 0.0 {
            self.m[FUEL] += dm;
            self.u += dm * flow_h_fuel(self.t_piston_k);
            self.vap_unmixed += dm;
            q_film = dm * latent;
        }
        let [t_head, t_liner] = self.wall_temps(i.t_coolant_k);
        let dm: f64 = cb::film_evaporate(&mut self.liner_c, t_liner, self.p, k_diff, k_heat(self.film_flux[1]), dt).iter().sum();
        if dm > 0.0 {
            self.m[FUEL] += dm;
            self.u += dm * (flow_h_fuel(t_liner) - latent);
            self.vap_unmixed += dm;
        }
        self.film_kg = self.film_c.iter().sum();
        self.liner_film_kg = self.liner_c.iter().sum();
        if self.water_kg > 0.0 && self.t > 373.0 && self.mass() > 0.0 {
            let dm = self.water_kg * (1.0 - (-dt / 5e-3).exp());
            self.water_kg -= dm;
            self.m[PROD] += dm;
            self.u += dm * (flow_h(373.0)[PROD] - L_VAP_WATER);
        }

        // ---- Turbulence and mixing ------------------------------------------------------------
        let h_ch = v_gas / k.area;
        let len = Turbulence::length(ch, h_ch, k.bore);
        let m_now = self.mass();
        let u_p = self.turb.u_prime(m_now);
        if self.vap_unmixed > 0.0 {
            let w_mix = C_PHI * (u_p / len + 1.0 / 0.2); // + slow molecular/bulk mixing floor (τ 0.2 s)
            self.vap_unmixed *= (-w_mix * dt).exp();
            if self.vap_unmixed < 1e-24 {
                self.vap_unmixed = 0.0;
            }
        }

        // ---- Cycle events, ignition, flame, pre-ignition, end-gas autoignition ------------------
        if crossed(i.phi0, i.phi1, 540.0) {
            self.spark_done = false;
            (self.ccv_u, self.ccv_phi) = (0.0, 0.0);
            self.lw = 0.0;
            self.knocked = false;
            self.hs_int = 0.0;
            self.lspi_age = 0.0;
            self.lspi_int = 0.0;
            self.lspi_at = -1.0;
            self.no_kg = 0.0;
            self.acc.evo_done = false;
        }
        // LSPI: the ring pack releases an oil / fuel droplet from the top-land crevice with a probability set by the
        // physical propensity (fuel in the oil, liner wetting, ring wear, deposit flakes); drawn once per cycle.
        let has_fuel = self.m[AIR] > 0.0 && i.afr * self.m[FUEL] / self.m[AIR] > 0.3;
        if has_fuel && crossed(i.phi0, i.phi1, 600.0) {
            let fuel = self.acc.fuel.max(self.m[FUEL]).max(1e-5);
            let lam = ch.lspi_rate
                * (1.0 + i.oil_dilution / 0.02)
                * (1.0 + 20.0 * self.liner_film_kg.max(0.0) / fuel)
                * i.ring_factor.max(0.1).sqrt()
                * (1.0 + 2.5 * i.deposit_m / 0.2e-3);
            let (u1, u2) = (self.rng.uniform(), self.rng.uniform());
            if u1 < 1.0 - (-lam).exp() {
                self.lspi_at = 620.0 + 60.0 * u2; // release 100…40 °CA before TDC
            }
        }
        if self.lspi_at > 0.0 && crossed(i.phi0, i.phi1, self.lspi_at) {
            self.lspi_age = 1e-12;
        }
        let in_window = i.phi1 >= 540.0 || a < 120.0;
        let compression = i.phi1 >= 560.0 && !i.in_open && !i.ex_open;
        if i.omega > 0.0 && !self.acc.burned && compression && has_fuel {
            // hot surface (plug nose / ground electrode / deposit): Douaud–Eyzat delay at the surface temperature
            let t_s = self.t_plug_tip_k.max(self.t_plug_gnd_k).max(self.t_deposit_k);
            if t_s > 900.0 {
                self.hs_int += dt / (ch.hot_surface_c * cb::autoignition_delay(i.ron, self.p, t_s));
            }
            // LSPI droplet: d²-law evaporation, then the cloud's own autoignition delay in the charge
            if self.lspi_age > 0.0 {
                self.lspi_age += dt;
                let t_ev = ch.lspi_drop_m * ch.lspi_drop_m / (ch.lspi_evap_k * (self.t / 700.0).max(0.3).powf(1.75));
                if self.lspi_age > t_ev {
                    self.lspi_int += dt / (ch.knock_tau_scale * cb::autoignition_delay(ch.lspi_octane, self.p, self.t));
                }
            }
            let hot = self.hs_int >= 1.0;
            if hot || self.lspi_int >= 1.0 {
                self.ignite(1.0e-3, k, mech, i, a);
                self.acc.preign = true;
                self.acc.lspi = !hot;
                self.acc.preign_deg = a;
                self.lspi_age = 0.0;
            }
        }
        // Spark: once per cycle. Timed (co-simulation): at the end of the coil's dwell as the ECU drove it. Angle command:
        // when the crank has reached the commanded angle; when it moves earlier than the crank between two looks (spark
        // advance from one cycle to the next) the discharge has already happened, so it fires at once instead of waiting
        // for an angle that has passed.
        if i.omega > 0.0 && !self.spark_done {
            let fire = match i.coil_timed {
                Some(d) => d.iter().flatten().find(|d| d.on_s > self.spark_on_s && d.off_s < i.time_s + dt).map(|d| {
                    self.spark_on_s = d.on_s;
                    CoilCmd { dwell_ms: (d.off_s - d.on_s) * 1e3, spark_btdc_deg: -a }
                }),
                None => i.coil.filter(|c| {
                    let target = -c.spark_btdc_deg;
                    let a0 = atdc(i.phi0);
                    crossed(i.phi0, i.phi1, (720.0 - c.spark_btdc_deg).rem_euclid(720.0)) || (a0 >= target && a0 < target + 90.0 && a >= a0)
                }),
            };
                if let Some(c) = fire.filter(|_| (-180.0..180.0).contains(&a)) {
                self.spark_done = true;
                if !self.burning {
                    self.spark(k, mech, &c, i, a);
                }
            }
        }
        // the flame keeps burning while the exhaust valve opens (blowdown takes burned and unburned gas in proportion,
        // `take`); it ends when it has consumed the charge or at the next intake opening
        if self.burning {
            if i.in_open {
                self.end_burn();
            } else {
                self.flame_step(k, mech, i, v_gas, dphi, a);
            }
        }
        // end gas: only while a flame is running or before any ignition of this cycle.
        // Residual burned gas above the full-load baseline (~3–8 %) chemically dilutes the end-gas (Hoepke et al.,
        // SAE 2012-01-0704), and at unboosted / throttled charge masses (< 458 mg) thermal boundary-layer cooling in
        // the peripheral squish clearance and low-pressure RO2 dissociation suppress end-gas autoignition relative to the
        // high-density boosted CFR correlation (Westbrook, Combust. Flame 121, 2000; empirical thresholds UNVERIFIED).
        if in_window && !self.knocked && (self.burning || (!self.acc.burned && has_fuel)) && self.m[FUEL] > 0.0 && self.m[AIR] > 0.0 {
            let (t_u, x_r) = if self.burning {
                let mu = self.mu();
                let m_u = (mu[0] + mu[1] + mu[2]).max(1e-12);
                (self.t_u, mu[PROD] / m_u)
            } else {
                (self.t, self.m[PROD] / self.mass().max(1e-12))
            };
            if t_u > 600.0 {
                let m_air = if self.acc.air > 0.0 {
                    self.acc.air
                } else {
                    self.m[AIR] + self.m[PROD] / (1.0 + 1.0 / i.afr.max(1.0))
                };
                let egr_dil = (50.0 * (x_r - 0.085).max(0.0)).exp()
                    * (4.58e-4 / m_air.max(1e-5)).max(1.0).powi(12)
                    * (115.0 / i.omega.max(20.0)).max(1.0).powi(4);
                self.lw += dt / (ch.knock_tau_scale * egr_dil * cb::autoignition_delay(i.ron, self.p, t_u));
                if self.lw >= 1.0 {
                    let m_tot = self.mass();
                    let x_u = if self.burning { 1.0 - self.mb_total() / m_tot } else { 1.0 };
                    let q = if self.burning {
                        let mu: f64 = self.mu().iter().sum();
                        let qq = self.burn_zone(mu, i);
                        self.end_burn();
                        qq
                    } else {
                        self.acc.burned = true;
                        self.acc.ign = a;
                        self.m0 = m_tot;
                        let qq = self.burn_zone(m_tot, i);
                        self.end_burn();
                        qq
                    };
                    if q > 0.0 {
                        // ordinary knock rings with a fraction of the constant-volume pressure jump; a large, hot end gas
                        // turns into a developing detonation whose wave approaches the full jump ("super-knock",
                        // Kalghatgi & Bradley, IJER 13 (2012) 399)
                        let g = ((x_u - 0.15) / 0.35).clamp(0.0, 1.0);
                        let g = g * g * (3.0 - 2.0 * g);
                        let amp = (ch.knock_amp_coef + (1.0 - ch.knock_amp_coef) * g) * 0.3 * q / v_gas;
                        self.knock_amp_pa = self.knock_amp_pa.max(amp);
                        o.knock_pa = amp;
                        if amp > self.acc.knock_amp {
                            self.acc.knock_amp = amp;
                            self.acc.knock_onset = a;
                            self.acc.knock_unburned = x_u;
                        }
                        self.acc.burned = true;
                    }
                    if self.xb < 0.1 {
                        self.acc.ca10 = a;
                    }
                    if self.xb < 0.5 {
                        self.acc.ca50 = a;
                    }
                    if self.xb < 0.9 {
                        self.acc.ca90 = a;
                    }
                    self.xb = 1.0;
                    self.knocked = true;
                }
            }
        }
        if self.knock_amp_pa > 0.0 {
            self.knock_amp_pa *= (-dt / KNOCK_DECAY_S).exp();
            if self.knock_amp_pa < 1e-15 {
                self.knock_amp_pa = 0.0;
            }
        }

        // ---- Wall heat transfer (Hohenberg), component nodes ------------------------------------------
        // Hohenberg, SAE 790825 (1979): h = 130·V^−0.06·p[bar]^0.8·T^−0.4·(S̄p + 1.4)^0.8. Chosen over Woschni (1967), whose
        // combustion-velocity term inflates the heat loss of low-load, low-pressure combustion: with it the plant's MBT at
        // idle sat at CA50 ≈ 16–19 °ATDC and 5–7 ° behind the firmware's reference-ignition map; Hohenberg brings both
        // back (docs/SIMOS18_COMBUSTION.md). Knock scrubs the thermal boundary layer.
        let sp = k.stroke * i.omega.abs() / PI;
        let swirl = if i.omega.abs() > 0.1 || self.burning { 1.4 } else { 0.05 };
        let h_gas = 130.0 * self.v.powf(-0.06) * ((self.p / 1e5) * (sp + swirl) / self.t.sqrt()).powf(0.8) * (1.0 + self.knock_amp_pa / KNOCK_HEAT_PA);
        self.h_mean += (h_gas - self.h_mean) * (dt / 0.05).min(1.0);
        // deposits on crown and head: series resistance δ/k on the cycle-mean film coefficient
        let dep = i.deposit_m.max(0.0);
        let f_dep = 1.0 / (1.0 + self.h_mean * dep / mech.deposit.k_w_mk);
        let a_liner = PI * k.bore * (i.s.x + k.v_clear / k.area);
        let a_head = 1.3 * k.area;
        let a_pist = 1.2 * k.area;
        // zone temperatures the walls see: area share of the burned zone ≈ its volume share
        let (t_gas_wall, x_vb, t_plug_gas) = if self.burning {
            let r_b = mix_r(&self.mb);
            let x = (self.mb_total() * r_b * self.t_b / self.p / v_gas).clamp(0.0, 1.0);
            (x * self.t_b + (1.0 - x) * self.t_u, x, self.t_b)
        } else {
            (self.t, 0.0, self.t)
        };
        let q_h = h_gas * f_dep * a_head * (t_gas_wall - t_head) * dt;
        let q_l = h_gas * a_liner * (t_gas_wall - t_liner) * dt;
        let pt = &mech.piston_thermal;
        let q_pc = h_gas * f_dep * a_pist * pt.crown_area_share * (t_gas_wall - self.t_piston_k) * dt;
        let q_pl = h_gas * f_dep * a_pist * (1.0 - pt.crown_area_share) * (t_gas_wall - self.t_land_k) * dt;
        let q_wall = q_h + q_l + q_pc + q_pl;
        self.film_flux = [q_pc / (dt * a_pist * pt.crown_area_share), q_l / (dt * a_liner)];
        for (w, q) in self.wall_q_w.iter_mut().zip([q_h, q_l]) {
            *w += (q / dt - *w) * (dt / WALL_TAU_S).min(1.0);
        }
        self.u -= q_wall;
        o.q_coolant += q_h + q_l;
        // zone shares of the wall heat (burned zone ∝ its wetted share ≈ volume share × T_b excess)
        let q_b_wall = if self.burning && t_gas_wall > 0.0 { q_wall * x_vb * self.t_b / t_gas_wall } else { 0.0 };
        let mut tp = [self.t_piston_k, self.t_land_k, self.t_groove_k];
        let (q_jet, q_ring) = pt.step(&mut tp, q_pc - q_film, q_pl, i.jet, i.t_oil_k, t_liner, dt);
        [self.t_piston_k, self.t_land_k, self.t_groove_k] = tp;
        o.q_oil += q_jet;
        o.q_coolant += q_ring;
        // Exhaust valve: face (per valve), port convection while flowing.
        let vt = &mech.exhaust_valve_thermal;
        let d_v = mech.exhaust.head_diameter_m;
        let q_face = h_gas * PI / 4.0 * d_v * d_v * (self.t - self.t_exv_k) * dt;
        let q_port = if dm_ex > 0.0 {
            let hp = vt.port_h_ref * ((dm_ex / dt) / (mech.exhaust.count * 0.05)).powf(0.8);
            hp * PI * d_v * 0.008 * (self.t - self.t_exv_k) * dt
        } else {
            0.0
        };
        let mut tv = [self.t_exv_k, self.t_exv_seat_k];
        o.q_coolant += vt.step(&mut tv, q_face, q_port, !i.ex_open, t_head, dt);
        [self.t_exv_k, self.t_exv_seat_k] = tv;
        // Spark plug (sees the kernel / burned gas around the electrodes) and the deposit surface (crown edge).
        let sph = &mech.spark_plug;
        let mut tpl = [self.t_plug_tip_k, self.t_plug_gnd_k];
        let (q_tip, q_gnd) = (h_gas * sph.tip_area_m2 * (t_plug_gas - tpl[0]) * dt, h_gas * sph.gnd_area_m2 * (t_plug_gas - tpl[1]) * dt);
        sph.step(&mut tpl, q_tip, q_gnd, t_head, dt);
        [self.t_plug_tip_k, self.t_plug_gnd_k] = tpl;
        let dh = &mech.deposit;
        let q_dep = h_gas * dh.area_m2 * (t_gas_wall - self.t_deposit_k) * dt;
        dh.step(&mut self.t_deposit_k, q_dep, self.t_land_k, dep, dt);

        // ---- Piston work and state -------------------------------------------------------
        let dv = i.v1 - self.v;
        let w_pv = self.p * dv;
        self.u -= w_pv;
        self.acc.w_net += w_pv;
        if i.phi1 >= 540.0 || i.phi1 < 180.0 {
            self.acc.w_gross += w_pv;
        }
        self.v = i.v1;
        if self.mass() < 1e-9 {
            self.m[AIR] += 1e-9;
        }
        let v_gas = (self.v - self.liquid_volume()).max(1e-7);
        if self.burning {
            // fixed point on the step's pressure ratio (2 passes: the zone exponent R/cp ≈ 0.2–0.25 contracts fast)
            let q_u_wall = q_wall - q_b_wall;
            let (t_u0, t_b0, p0) = (self.t_u, self.t_b, self.p_prev.max(50.0));
            self.update_zones(v_gas, self.p / p0, q_u_wall, q_b_wall);
            let p1 = self.p;
            (self.t_u, self.t_b) = (t_u0, t_b0);
            self.update_zones(v_gas, p1 / p0, q_u_wall, q_b_wall);
            self.p_prev = self.p;
        } else {
            self.t = solve_t(&self.m, self.u, self.t);
            self.p = (self.mass() * mix_r(&self.m) * self.t / v_gas).max(50.0);
            self.t_u = self.t;
            self.p_prev = self.p;
        }
        // compression amplification of the turbulence (rapid distortion, k ∝ ρ^⅔, valves closed) and its cascade. During
        // the flame the eddies ahead of it are compressed by the expanding burned gas: the unburned-zone density sets the
        // amplification (angular momentum of the eddies conserved, u' ∝ ρ_u^⅓ — Tabaczynski, Ferguson & Radhakrishnan,
        // SAE 770647; Wong & Hoult, SAE 790357).
        let rho_ratio = if i.in_open || i.ex_open { 1.0 } else { self.rho_u() / rho_before.max(1e-9) };
        self.turb.step(ch, dt, self.mass(), rho_ratio, v_gas / k.area, k.bore);

        // ---- Thermal NO in the burned gas ----------------------------------------------------
        let (t_no, m_no_zone) = if self.burning { (self.t_b, self.mb_total()) } else { (self.t, self.mass()) };
        if t_no > 1600.0 && m_no_zone > 0.0 && (a > -60.0 && a < 150.0) {
            let lam = if self.acc.lambda > 0.0 { self.acc.lambda } else { 1.0 };
            let rho = self.p / (288.0 * t_no);
            let y = self.no_kg / m_no_zone;
            self.no_kg += m_no_zone * cb::zeldovich_dyno_dt(t_no, self.p, rho, cb::burned_o2_mole_frac(lam, t_no), 0.72, y) * dt;
        }

        // ---- Spark-plug fouling -------------------------------------------------------------------
        let soot_src = if i.omega > 0.0 && self.last.fuel_kg > 1e-9 {
            let rich = if self.last.lambda > 0.0 { (1.0 / self.last.lambda.max(0.5) - 1.0).max(0.0) } else { 0.0 };
            2.0 * rich + 5.0 * self.last.stratification + if self.last.misfire { 10.0 } else { 0.0 }
        } else {
            0.0
        };
        self.plug_fouling = sph.foul(self.plug_fouling, self.t_plug_tip_k, soot_src, dt);

        // ---- Liner fuel: scraped into the oil by the rings on the down-stroke ------------------------
        if self.liner_film_kg > 0.0 && crossed(i.phi0, i.phi1, 180.0) {
            self.liner_c.iter_mut().for_each(|m| *m *= 1.0 - LINER_TO_OIL);
            let dm = LINER_TO_OIL * self.liner_film_kg;
            self.liner_film_kg -= dm;
            o.fuel_to_oil += dm;
        }

        // ---- Diagnostics -----------------------------------------------------------------
        if self.in_open_prev && !i.in_open && i.phi1 >= 450.0 {
            self.acc.air = self.m[AIR];
            self.acc.residual = self.m[PROD] / self.mass();
        }
        self.in_open_prev = i.in_open;
        if self.p > self.acc.p_max {
            self.acc.p_max = self.p;
            self.acc.p_max_deg = a;
        }
        if self.burning {
            self.acc.t_u_max = self.acc.t_u_max.max(self.t_u);
        }
        self.acc.p_loc_max = self.acc.p_loc_max.max(self.p + self.knock_amp_pa);
        if (5.0..70.0).contains(&a) {
            self.acc.ko_a2 += 0.5 * self.knock_amp_pa * self.knock_amp_pa * dt;
            self.acc.ko_t += dt;
        }
        if i.ex_open && a > 60.0 && a < 300.0 && !self.acc.evo_done {
            self.acc.evo_done = true;
            self.emissions_at_evo(i);
        }
        if crossed(i.phi0, i.phi1, 180.0) {
            self.finish_cycle(k, mech, i);
        }
        o
    }

    /// Entrainment flame over one step (`combustion` module doc). Returns the mass burned.
    fn flame_step(&mut self, k: &Kinematics, mech: &Mechanics, i: &CylIn, v_gas: f64, dphi: f64, a: f64) -> f64 {
        let ch = &mech.combustion;
        let dt = i.dt;
        let mu = self.mu();
        let m_u: f64 = mu.iter().sum();
        let m_b = self.mb_total();
        let m_tot = m_u + m_b;
        if m_u <= 1e-9 * m_tot.max(1e-9) {
            self.end_burn();
            return 0.0;
        }
        let r_u = mix_r(&mu);
        let v_u = (m_u * r_u * self.t_u / self.p).min(v_gas);
        let rho_u = m_u / v_u.max(1e-12);
        let h = v_gas / k.area;
        let len = Turbulence::length(ch, h, k.bore);
        // flame front: burned volume + entrained-unburned volume
        let v_b = (v_gas - v_u).max(0.0);
        let v_f = v_b + (self.m_e - m_b).max(0.0) / rho_u;
        let zp = ch.plug_depth_m.min(0.5 * h);
        let (r_f, a_f) = cb::flame_radius(v_f, self.r_flame_m, zp, 0.5 * k.bore, h);
        self.r_flame_m = r_f;
        // this cycle's deviation of the flow and mixture at the plug, fading as the front averages over more eddies
        let w_ccv = len / (len + r_f);
        let phi = if mu[AIR] > 0.0 { i.afr * mu[FUEL] / mu[AIR] } else { 3.0 } * (1.0 + self.ccv_phi * w_ccv);
        let x_r = mu[PROD] / m_u;
        let strat = if self.m[FUEL] > 0.0 { (self.vap_unmixed / self.m[FUEL]).clamp(0.0, 1.0) } else { 0.0 };
        // stratified share burns in rich pockets (φ ≈ 1.6), the rest at the mixed φ
        let sl_mix = cb::laminar_flame_speed(phi, self.t_u, self.p, x_r, i.w_eth);
        let sl = if strat > 0.0 {
            (1.0 - strat) * sl_mix + strat * cb::laminar_flame_speed(1.6, self.t_u, self.p, x_r, i.w_eth)
        } else {
            sl_mix
        };
        let u_p = self.turb.u_prime(m_tot) * (1.0 + self.ccv_u * w_ccv).max(0.1);
        let nu = cb::viscosity(self.t_u) / rho_u;
        let alpha = nu / 0.71;
        // Herweg & Maly (1992): the kernel feels only the eddies smaller than itself
        let f_dev = (1.0 - (-ch.kernel_ck * r_f / len).exp()).sqrt();
        let s_l = sl.max(1e-3);
        let s_t = s_l * (1.0 + ch.flame_c * f_dev * cb::peters_increment(u_p / s_l, len * s_l / alpha));
        let avail = (m_u - (self.m_e - m_b).max(0.0) - self.m_crev).max(0.0);
        let dm_e = (rho_u * a_f * s_t * dt).min(avail);
        self.m_e += dm_e;
        let lambda_t = (15.0 * nu * len / u_p.max(0.05)).sqrt();
        let tau_b = ch.burnup_c * lambda_t / s_l;
        let dm_b = ((self.m_e - m_b).max(0.0) * (1.0 - (-dt / tau_b).exp())).min(m_u);
        // crevices: they fill with unburned charge at wall temperature as the pressure rises
        let t_wall = self.wall_temps(i.t_coolant_k)[1];
        self.m_crev = self.m_crev.max((mech.crevice_m3 * self.p / (r_u * t_wall) * CREVICE_UNOX).min(0.2 * m_u));
        if dm_b > 0.0 {
            self.burn_zone(dm_b, i);
        }
        let xb_new = (self.mb_total() / self.m0.max(1e-12)).min(1.0);
        for (lvl, slot) in [(0.1, 0), (0.5, 1), (0.9, 2)] {
            if self.xb < lvl && xb_new >= lvl {
                match slot {
                    0 => self.acc.ca10 = a,
                    1 => self.acc.ca50 = a,
                    _ => self.acc.ca90 = a,
                }
            }
        }
        self.xb = xb_new;
        let _ = dphi;
        // the flame has consumed everything it can reach
        if avail <= 1e-4 * self.m0 && (self.m_e - self.mb_total()) <= 1e-4 * self.m0 {
            self.end_burn();
        }
        dm_b
    }

    fn spark(&mut self, k: &Kinematics, mech: &Mechanics, c: &CoilCmd, i: &CylIn, a: f64) {
        let ch = &mech.combustion;
        let dwell = (c.dwell_ms * 1e-3).max(0.0);
        let amps = i.battery_v / COIL_R_OHM * (1.0 - (-dwell * COIL_R_OHM / COIL_L_H).exp());
        let e = 0.5 * COIL_L_H * amps * amps;
        // a carbon-fouled insulator nose shunts the secondary: the voltage rise stops below breakdown
        let v_av = (2.0 * COIL_ETA * e / COIL_C_SEC_F).sqrt() * (1.0 - self.plug_fouling);
        let v_gas = (self.v - self.liquid_volume()).max(1e-7);
        let rho = self.mass() / v_gas;
        let v_bd = 1.0e3 + 3.0e3 * (mech.spark_gap_m / 7e-4) * (rho / 1.2).powf(0.9);
        let fuel_all = self.m[FUEL] + self.spray_kg;
        self.acc.lambda = if fuel_all > 0.0 { self.m[AIR] / (i.afr * fuel_all) } else { 0.0 };
        self.acc.film = self.film_kg;
        self.acc.strat = if self.m[FUEL] > 0.0 { (self.vap_unmixed / self.m[FUEL]).clamp(0.0, 1.0) } else { 0.0 };
        // A fuel-cut cycle (`self.acc.fuel <= 3e-6` kg = 3 mg, i.e. no injector pulse, only trace port/liner film
        // evaporation) is not a misfire.
        let has_fuel = self.m[AIR] > 0.0 && i.afr * self.m[FUEL] / self.m[AIR] > 0.5;
        let injected = self.acc.fuel > 3e-6;
        if v_av < v_bd || !has_fuel {
            if has_fuel && injected {
                self.acc.misfire = true;
            }
            return;
        }
        // this cycle's flow and mixture at the plug (cycle-to-cycle variation, `CombustionHw::ccv_*`)
        let (z_u, z_phi) = self.rng.normal_pair();
        (self.ccv_u, self.ccv_phi) = (ch.ccv_turb_sigma * z_u, ch.ccv_phi_sigma * z_phi);
        // kernel quench: laminar flame speed at the dilution limit or flame stretch beyond Ka_q
        let phi = i.afr * self.m[FUEL] / self.m[AIR] * (1.0 + self.ccv_phi);
        let x_r = self.m[PROD] / self.mass();
        let sl = cb::laminar_flame_speed(phi, self.t, self.p, x_r, i.w_eth);
        let h = v_gas / k.area;
        let len = Turbulence::length(ch, h, k.bore);
        let u_p = self.turb.u_prime(self.mass()) * (1.0 + self.ccv_u).max(0.1);
        // kernel: the energy delivered to the gas heats a sphere to the flame temperature (Maly & Vogel 1979)
        let e_gas = ch.spark_eta * COIL_ETA * e;
        let cp = mix_cp(&self.m, self.t);
        let r0 = (3.0 * e_gas / (4.0 * PI * rho * cp * 1800.0)).cbrt().clamp(0.3e-3, 3e-3);
        // the kernel is strained only by eddies up to its own size (Herweg & Maly 1992): Karlovitz number of that part
        let nu = cb::viscosity(self.t) / rho;
        let u_k = u_p * (1.0 - (-r0 / len).exp()).sqrt();
        let re = (u_k * len / nu).max(1.0);
        let ka = 0.157 * (u_k / sl.max(1e-3)).powi(2) / re.sqrt();
        self.acc.u_prime = u_p;
        self.acc.s_l = sl;
        if sl < S_L_MIN || ka > ch.ka_quench {
            if injected {
                self.acc.misfire = true;
            }
            return;
        }
        self.ignite(r0, k, mech, i, a);
    }

    /// Engine-out emissions of the cycle when the exhaust valve opens.
    fn emissions_at_evo(&mut self, i: &CylIn) {
        let m = self.mass().max(1e-12);
        let m_mix = 28.9e-3;
        self.acc.no_ppm = self.no_kg / m * m_mix / 30.0e-3 * 1e6;
        let lam = if self.acc.lambda > 0.0 { self.acc.lambda } else { 1.0 };
        // rich pockets of a stratified charge add CO
        self.acc.co_pct = 100.0 * (cb::co_mole_frac(lam) + 0.05 * self.acc.strat);
        let unburned_fuel = self.m[FUEL] + self.spray_kg;
        self.acc.hc_ppm = unburned_fuel / m * m_mix / 13.9e-3 * 1e6;
        self.acc.soot = 1e6 * (0.05 * self.acc.film + 0.01 * self.acc.strat * self.acc.fuel);
        let _ = i;
    }

    fn finish_cycle(&mut self, k: &Kinematics, mech: &Mechanics, i: &CylIn) {
        let a = self.acc;
        let vd = k.v_disp;
        if a.preign {
            self.preign_events += 1;
        }
        if a.lspi {
            self.lspi_events += 1;
        }
        if a.knock_amp > 1e5 {
            self.knock_events += 1;
        }
        if a.misfire {
            self.misfire_events += 1;
        }
        // Piston ring land: pulsating groove-root bending from the local wall pressure, plus the knock-mode cycles
        // (≈ f1·τ_decay / k equivalent full-amplitude cycles), at the ring-groove temperature; a single peak above the
        // UTS at that temperature fractures it (`Alloy::cycle_damage` returns 1).
        let t_land_c = self.t_groove_k.max(self.t_land_k - 20.0) - 273.15;
        if self.t_piston_k - 273.15 >= PISTON_SOLIDUS_C {
            self.land_failed = true;
        }
        let kl = mech.ring_land_stress_factor;
        let s_peak = kl * (a.p_loc_max - i.p_cc).max(0.0);
        let ratio = s_peak / PISTON_ALSI12CUNIMG.uts_pa(t_land_c).max(1.0);
        self.land_stress_max = self.land_stress_max.max(ratio);
        let mut d = PISTON_ALSI12CUNIMG.cycle_damage(s_peak, 0.0, t_land_c);
        if a.knock_amp > 0.0 {
            let f1 = 1.84 * (1.3 * 288.0 * 2400.0f64).sqrt() / (PI * k.bore);
            let n_eff = f1 * KNOCK_DECAY_S / PISTON_ALSI12CUNIMG.basquin_k;
            let sa = kl * a.knock_amp;
            d += n_eff * PISTON_ALSI12CUNIMG.cycle_damage(sa, -sa, t_land_c);
        }
        self.piston_damage = (self.piston_damage + d).min(1.0);
        if self.piston_damage >= 1.0 {
            self.land_failed = true;
        }

        self.last = CycleStats {
            p_max_pa: a.p_max,
            p_max_deg: a.p_max_deg,
            p_local_max_pa: a.p_loc_max,
            imep_pa: a.w_net / vd,
            gross_imep_pa: a.w_gross / vd,
            pmep_pa: (a.w_gross - a.w_net) / vd,
            ca10_deg: a.ca10,
            ca50_deg: a.ca50,
            ca90_deg: a.ca90,
            ign_deg: a.ign,
            knock_amp_pa: a.knock_amp,
            knock_onset_deg: a.knock_onset,
            knock_unburned: a.knock_unburned,
            lw: self.lw,
            burned: a.burned,
            misfire: a.misfire,
            preignition: a.preign,
            lspi: a.lspi,
            preign_deg: a.preign_deg,
            air_kg: a.air,
            fuel_kg: a.fuel,
            lambda: a.lambda,
            knock_rms_bar: if a.ko_t > 0.0 { (a.ko_a2 / a.ko_t).sqrt() / 1e5 } else { 0.0 },
            rod_comp_n: a.rod_c,
            rod_tens_n: a.rod_t,
            residual: a.residual,
            u_prime: a.u_prime,
            s_l: a.s_l,
            stratification: a.strat,
            film_kg: a.film,
            t_u_max: a.t_u_max,
            t_b_max: a.t_b_max,
            no_ppm: a.no_ppm,
            co_pct: a.co_pct,
            hc_ppm: a.hc_ppm,
            soot_mg: a.soot,
            land_stress_ratio: ratio,
            t_end_s: i.time_s,
        };

        // Head-gasket bead fatigue and fire-ring erosion when the gas force exceeds the clamp. The head-lift force is the
        // bore-area integral of the pressure, i.e. the bulk pressure: every acoustic knock mode is orthogonal to the
        // uniform one (its area integral vanishes, e.g. the first circumferential mode ∝ J1(kr)·cos θ), so the knock
        // ringing adds nothing to it — unlike the local wall pressure that loads the ring land above. The knocked end
        // gas still raises the bulk pressure through its own heat release.
        let a_seal = PI / 4.0 * mech.fire_ring_diameter_m.powi(2);
        let ratio = (a.p_max - i.p_coolant).max(0.0) * a_seal / mech.fire_ring_clamp_n;
        let mut dg = GASKET_BEAD_1_4310.cycle_damage(BEAD_STRESS_AT_LIFTOFF * ratio, 0.0, 150.0);
        if ratio > 1.0 {
            dg += 0.02 * (ratio - 1.0);
        }
        self.gasket_damage = (self.gasket_damage + dg).min(1.0);
        if self.gasket_damage >= 1.0 {
            self.gasket_leak_m2 = self.gasket_leak_m2.max(2e-6) + if ratio > 1.0 { 0.2e-6 * (ratio - 1.0) } else { 0.0 };
        }

        // Connecting rod: tension from inertia at gas-exchange TDC vs compression from gas.
        let ar = mech.rod_shank_area_m2;
        self.rod_damage = (self.rod_damage + ROD_36MNVS4.cycle_damage(a.rod_t / ar, -a.rod_c / ar, 120.0)).min(1.0);
        if self.rod_damage >= 1.0 {
            self.rod_broken = true;
        }

        // Exhaust valve creep (Larson–Miller) at the head node, cycle duration at the current speed.
        let cyc_s = 4.0 * PI / i.omega.abs().max(1.0);
        let sig = EXV_STRESS_MPA_100BAR * a.p_max / 100e5;
        self.exv_creep += cyc_s / 3600.0 / valve_21_4n_rupture_h(self.t_exv_k, sig.max(10.0));

        self.acc = Acc::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn energy_round_trip_and_properties() {
        let m = [1e-3, 5e-5, 2e-4];
        for &t in &[300.0, 800.0, 2200.0] {
            let (u, _) = internal_energy(&m, t);
            assert!((solve_t(&m, u, 500.0) - t).abs() < 0.1);
        }
        let (_, cp) = props(300.0);
        assert!((cp[0] - 1005.0).abs() < 1.0);
        let (_, cp) = props(2000.0);
        assert!(cp[2] > 1350.0 && cp[2] < 1500.0);
    }

    #[test]
    fn orifice_is_continuous_at_choking() {
        let a = orifice(1e-4, 1e5, 300.0, 0.5368e5 - 1.0);
        let b = orifice(1e-4, 1e5, 300.0, 0.5368e5 + 1.0);
        assert!((a - b).abs() / a < 2e-3);
        assert_eq!(orifice(1e-4, 1e5, 300.0, 1e5), 0.0);
    }

    /// Constant-pressure adiabatic flame temperature of a stoichiometric gasoline–air charge from 700 K with 10 %
    /// residual: ≈ 2550–2750 K without dissociation (the plant's products cp carries no dissociation enthalpy;
    /// equilibrium values are ≈ 2500 K — Heywood 1988 fig. 4-9).
    #[test]
    fn adiabatic_flame_temperature_is_in_the_textbook_band() {
        let (afr, x_r) = (AFR_STOICH, 0.1);
        let f = (1.0 - x_r) / (1.0 + afr);
        let xu = [afr * f, f, x_r];
        let (hs, _) = props(700.0);
        let h_u = xu[0] * hs[0] + xu[1] * hs[1] + xu[2] * hs[2];
        let xb = [0.0, 0.0, 1.0];
        let t_ad = solve_h(&xb, h_u + ETA_COMB * LHV_J_KG * f, 2000.0);
        assert!((2450.0..2800.0).contains(&t_ad), "T_ad {t_ad:.0} K");
    }
}
