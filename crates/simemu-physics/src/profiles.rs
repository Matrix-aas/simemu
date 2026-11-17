//! Physical hardware presets: engine/turbo hardware and DSG/vehicle hardware.
//!
//! Only geometry, inertias, flow capacities and component characteristics live here.
//! No calibration, no setpoints, no limits — those belong to the firmware.
//! Values marked `UNVERIFIED` are engineering approximations (no supplier maps available);
//! they are deliberately exposed as plain fields so they can be tuned against real logs.

use crate::clutch::{ClutchHw, DryClutchHw, WetClutchHw};
use crate::combustion::CombustionHw;
use crate::thermal::{DepositHw, PistonThermalHw, SparkPlugHw, ValveThermalHw};
use crate::driveline::GearTrainHw;
use crate::knock_sensor::KnockSensorHw;
use crate::wastegate::WastegateActuatorHw;
use crate::mechatronic::{Dq200HydraulicsHw, MechatronicHw, WetHydraulicsHw};
use crate::vehicle::{BodyHw, BrakeHw, HaldexHw, TyreHw};
use serde::Serialize;
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum EnginePresetId {
    /// 1.8 TSI EA888 Gen3 CJSA/CJSB + IHI IS12 (Octavia A7 FL 2019 — the car this project was built for).
    Ea888_18tCjsaIs12,
    /// Same 1.8 block with an IHI IS38 hybrid turbo and 60 mm FMIC.
    Ea888_18tIs38Hybrid,
    /// High-mileage 1.8 TSI CJSA long block + IHI IS38 with stock intercooler.
    Ea888_18tCjsaIs38,
    /// 2.0 TSI EA888 Gen3 CHHB + IHI IS20.
    Ea888_20tChhbIs20,
    /// 2.0 TSI EA888 Gen3 CJXC + IHI IS38.
    Ea888_20tCjxcIs38,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum DrivetrainPresetId {
    Dq200Fwd7Speed,
    Dq250Awd6Speed,
    Dq381Awd7Speed,
}

#[derive(Debug, Clone, Serialize)]
pub struct TurboHardware {
    pub model: &'static str,
    /// Compressor speed-line reference speed, rpm.
    pub n_ref_rpm: f64,
    /// Corrected choke flow at `n_ref_rpm`, kg/s (scales ∝ N). UNVERIFIED.
    pub choke_flow_ref_kg_s: f64,
    /// Zero-flow pressure ratio at `n_ref_rpm` (PR−1 scales ∝ N²). UNVERIFIED.
    pub pr_zero_flow_ref: f64,
    pub eta_compressor: f64,
    /// Effective flow area (Cd·A) of the turbine nozzle, m². UNVERIFIED.
    pub turbine_cda_m2: f64,
    /// Effective flow area (Cd·A) of the fully open wastegate port, m². UNVERIFIED.
    pub wastegate_cda_m2: f64,
    pub eta_turbine: f64,
    /// Turbine wheel tip radius (for the blade-speed ratio U/C), m. UNVERIFIED.
    pub turbine_tip_radius_m: f64,
    /// Rotor polar inertia (both wheels + shaft), kg·m². UNVERIFIED.
    pub inertia_kgm2: f64,
    /// Compressor wheel exducer tip radius (AA2618 wheel, hoop stress ∝ ρ·ω²·r²), m. UNVERIFIED.
    pub compressor_tip_radius_m: f64,
    /// Electric wastegate actuator + flap of this turbocharger.
    pub wg_actuator: WastegateActuatorHw,
    /// Compressor map from an engine calibration's own model of this turbocharger (`compressor_map`); `None` = the
    /// generic elliptic speed lines above (`n_ref_rpm`, `choke_flow_ref_kg_s`, `pr_zero_flow_ref`, `eta_compressor`).
    #[serde(skip)]
    pub map: Option<&'static crate::compressor_map::CompressorMap>,
    /// Efficiency lost to wear (compressor and turbine), subtracted from the table map's efficiency.
    pub eta_loss: f64,
    /// Wastegate flow over the actuator position from an engine calibration's own model (`wastegate::WastegateFlowMap`);
    /// `None` = the port orifice `wastegate_cda_m2` × the flap's curtain area alone.
    #[serde(skip)]
    pub wastegate_map: Option<&'static crate::wastegate::WastegateFlowMap>,
}

/// EA888 Gen3 throttle body (Ø56 mm): SC800S50 `ip_ar_thr_bas` "reduced area at throttle" @0x8084BE3C (axis
/// @0x8084BABA), °TPS → cm² — the flow-bench characteristic the calibration was made with (VERIFIED-S), used
/// with the isentropic orifice equation; held above 80°.
pub const THROTTLE_CDA_EA888_GEN3: [(f64, f64); 16] = [
    (0.0, 0.0599), (1.0066, 0.0849), (1.5026, 0.1100), (2.0059, 0.1341), (3.0052, 0.1904), (4.5005, 0.2646),
    (7.0024, 0.4515), (10.0003, 0.7179), (12.9982, 1.0371), (17.0027, 1.6299), (20.9998, 2.1699),
    (25.0043, 2.7502), (30.0008, 3.6603), (40.0011, 5.9697), (60.0016, 12.7503), (80.0022, 20.0003),
];

/// Electronic throttle actuator: DC motor + gear against the limp-home spring pair, referred to the throttle
/// shaft. Values UNVERIFIED (Bosch DV-E class throttle bodies: ~1.6 Ω motor, ~16:1 gear, plate full open in
/// ≈ 65 ms at 13.5 V, 15–20 % duty to hold part throttle); `max_deg` = SC800S50 `c_tps_max` 90.0 °TPS
/// @0x8080C058 (VERIFIED-S).
#[derive(Debug, Clone, Serialize)]
pub struct EtcActuator {
    /// Motor winding resistance, Ω.
    pub r_ohm: f64,
    /// Torque constant at the shaft (motor kt × gear ratio), N·m/A = back-EMF constant at the shaft, V·s/rad.
    pub k_nm_a: f64,
    /// Moving inertia at the shaft (motor rotor × ratio² dominates), kg·m².
    pub inertia_kgm2: f64,
    /// Return spring above limp-home: preload at limp-home, N·m, and rate, N·m/rad.
    pub close_preload_nm: f64,
    pub close_rate_nm_rad: f64,
    /// Limp-home (opening) spring below limp-home.
    pub open_preload_nm: f64,
    pub open_rate_nm_rad: f64,
    /// Coulomb friction (gear, shaft seals), N·m.
    pub friction_nm: f64,
    /// Upper mechanical stop, deg from the lower stop.
    pub max_deg: f64,
}

/// HPFP internal loss per pump stroke ("fuel mass offset due to internal pump behaviour", the plunger first
/// compresses the dead volume before the outlet valve opens), mg, `mg[i_fup][i_rpm]` over engine speed (rpm) and
/// rail gauge pressure (hPa).
#[derive(Debug, Clone, Serialize)]
pub struct HpfpLossMap {
    pub rpm: [f64; 8],
    pub fup_hpa: [f64; 8],
    pub mg: [[f64; 8]; 8],
}

impl HpfpLossMap {
    /// Bilinear, clamped at the edges (allocation-free).
    pub fn mg_at(&self, rpm: f64, fup_hpa: f64) -> f64 {
        let seg = |a: &[f64; 8], x: f64| {
            let i = a.partition_point(|&v| v < x).clamp(1, 7);
            (i, ((x.clamp(a[0], a[7]) - a[i - 1]) / (a[i] - a[i - 1])).clamp(0.0, 1.0))
        };
        let ((ix, fx), (iy, fy)) = (seg(&self.rpm, rpm), seg(&self.fup_hpa, fup_hpa));
        let row = |r: &[f64; 8]| r[ix - 1] + fx * (r[ix] - r[ix - 1]);
        row(&self.mg[iy - 1]) + fy * (row(&self.mg[iy]) - row(&self.mg[iy - 1]))
    }
}

/// SC800S50 (8V0906259J) `ip_mfp_ofs` @0x8083C22E (VERIFIED-S).
pub const HPFP_LOSS_SC8S50_259J: HpfpLossMap = HpfpLossMap {
    rpm: [608.0, 1024.0, 2016.0, 3008.0, 4000.0, 5024.0, 6016.0, 7008.0],
    fup_hpa: [25000.0, 50005.0, 75005.0, 100005.0, 125005.0, 150005.0, 175005.0, 200005.0],
    mg: [
        [12.99, 12.99, 20.01, 23.0, 26.01, 15.01, 10.0, 10.0],
        [23.0, 23.0, 28.0, 28.0, 32.0, 23.0, 20.01, 12.0],
        [37.01, 34.99, 36.01, 34.0, 38.0, 27.0, 26.01, 17.0],
        [49.0, 48.01, 43.0, 42.01, 43.0, 34.0, 32.0, 23.0],
        [61.99, 61.99, 53.01, 53.01, 49.0, 36.01, 34.0, 21.0],
        [75.01, 75.01, 60.0, 62.99, 55.0, 40.99, 37.01, 20.01],
        [84.99, 84.99, 64.01, 70.01, 60.0, 49.0, 39.99, 20.01],
        [89.99, 89.99, 70.01, 67.99, 60.0, 50.0, 45.0, 20.01],
    ],
};

/// SC8O40 (8V0906264H__0005) `ip_mfp_ofs` @0x8083BDDC (VERIFIED-S).
pub const HPFP_LOSS_SC8O40_264H: HpfpLossMap = HpfpLossMap {
    rpm: [608.0, 1024.0, 2016.0, 3008.0, 4000.0, 5024.0, 6016.0, 7008.0],
    fup_hpa: [25000.0, 50005.0, 75005.0, 100005.0, 125005.0, 150005.0, 175005.0, 200005.0],
    mg: [
        [2.50, 2.01, 3.01, 4.01, 2.50, 3.01, 5.00, 6.00],
        [9.01, 7.02, 8.01, 9.01, 7.50, 7.99, 11.0, 11.0],
        [12.50, 11.0, 11.0, 12.0, 11.0, 11.51, 13.01, 11.0],
        [18.02, 15.01, 14.01, 15.01, 15.01, 15.01, 16.0, 17.0],
        [21.0, 18.02, 18.5, 18.5, 18.5, 18.02, 18.02, 19.5],
        [25.01, 24.5, 24.01, 23.02, 23.0, 23.0, 23.0, 24.01],
        [31.01, 27.0, 26.01, 25.01, 25.01, 26.51, 26.01, 26.01],
        [37.01, 32.0, 31.01, 29.99, 31.01, 31.52, 32.0, 33.0],
    ],
};

/// N276 closing delay after current-on, (battery V, ms): the time the calibration advances its current-on angle by
/// to close the valve at the wanted angle (FR p.12047, "mechanical delay time between VCV current feed and VCV
/// closing"). SC800S50 259J: `ip_t_dly_vb_add` @0x808085AC (VERIFIED-S; VERIFIED-T at zero delivery: current on
/// 22.5 °CRK before plunger TDC at ≈ 1300 rpm, 14.2 V = 2.9 ms).
pub const VCV_CLOSE_DELAY_SC8S50_259J: [(f64, f64); 8] =
    [(5.9922, 5.312), (8.0234, 4.74), (10.0547, 4.172), (11.9844, 3.572), (14.0156, 3.0), (16.0469, 3.0), (18.4844, 3.0), (18.9922, 3.0)];
/// SC8O40 264H (`lc_swi_vcv_typ` = 1, another valve type): `ip_t_dly_vcv_close_add_3` 2.552 ms (idle speeds, all close
/// ranges, @0x8083C224) × `ip_fac_vb_t_dur_vcv_ctl` @0x808084BD + `ip_t_dly_vb_add` @0x80808514 (the combination of
/// FR fig. 43.15.59 — HYP until checked on a run of this image).
pub const VCV_CLOSE_DELAY_SC8O40_264H: [(f64, f64); 6] =
    [(5.9922, 3.052), (10.7656, 2.057), (12.0859, 1.657), (14.0156, 1.302), (16.0469, 1.174), (17.9766, 1.046)];
/// MPI injector dead time (opening − closing delay) vs battery voltage, (V, ms): `ip_ti_add_dly_mpi[0]`, identical in
/// SC800S50 259J @0x808092DE and SC8O40 264H @0x80809222 (VERIFIED-S).
pub const MPI_DEAD_TIME_EA888_GEN3: [(f64, f64); 6] = [(5.9922, 2.56), (9.0391, 1.552), (10.9688, 1.052), (11.9844, 0.86), (13.0, 0.7), (15.0312, 0.472)];

/// Port-fuel wall films (x–τ model, Aquino 1981 SAE 810494 / Hires & Overington 1981 SAE 810495) on two surfaces:
/// [intake-valve back (hot, fast), port wall (coolant temperature, slow)]. The fraction `x[i]` of each port-injection
/// pulse lands on film i, which evaporates into the port charge with the time constant τ_i; the airborne rest and the
/// vapour are drawn in with the intake air. τ_i scales physically from its reference state: Clausius–Clapeyron with the
/// film's surface temperature, ∝ manifold pressure (diffusion-limited evaporation: the surface mass fraction of the
/// vapour ∝ p_sat/p), ∝ (ṁ_air)^−0.8 (mass transfer of the port flow, Chilton–Colburn analogy to Dittus–Boelter).
#[derive(Debug, Clone, Serialize)]
pub struct PortFilmHw {
    /// Fractions of the injected mass deposited on [valve back, port wall] (clean valves).
    pub x: [f64; 2],
    /// Evaporation time constants at the reference state, s.
    pub tau_ref_s: [f64; 2],
    /// Film surface temperature over the coolant, K.
    pub surface_over_coolant_k: [f64; 2],
    /// Reference state: coolant °C, manifold pressure kPa, engine air flow kg/s.
    pub t_ref_c: f64,
    pub p_ref_kpa: f64,
    pub flow_ref_kg_s: f64,
    /// Effective enthalpy of vaporisation for the Clausius–Clapeyron temperature scaling of τ, J/mol.
    pub dh_vap_j_mol: f64,
    /// Intake-valve deposits: extra wall wetting (x·(1+k), τ·(1+k)); 0 = clean valves. UNVERIFIED knob.
    pub ivd_factor: f64,
    /// Port (runner) air volume per cylinder from the injector to the valve, m³ (its charge is inducted first).
    pub port_volume_m3: f64,
}

impl PortFilmHw {
    /// Evaporation time constants of the two films at coolant `t_cool_c`, manifold pressure `p_im_kpa`, engine air flow
    /// `air_kg_s`, s.
    pub fn tau_s(&self, t_cool_c: f64, p_im_kpa: f64, air_kg_s: f64) -> [f64; 2] {
        let flow = (air_kg_s.max(0.2 * self.flow_ref_kg_s) / self.flow_ref_kg_s).powf(-0.8);
        let press = (p_im_kpa / self.p_ref_kpa).clamp(0.1, 5.0);
        std::array::from_fn(|i| {
            let (t, t_ref) = (t_cool_c + 273.15 + self.surface_over_coolant_k[i], self.t_ref_c + 273.15 + self.surface_over_coolant_k[i]);
            self.tau_ref_s[i] * (1.0 + self.ivd_factor) * (self.dh_vap_j_mol / 8.314 * (1.0 / t - 1.0 / t_ref)).exp() * press * flow
        })
    }

    /// The same per fuel pseudo-component (`combustion::FUEL_PSEUDO`), s: film i's τ spread over the components by their
    /// vapour pressure at its surface relative to the mid component (Raoult with the wetted area ∝ the film's moles: each
    /// component leaves ∝ its own p_sat), normalised so that the mass-weighted mean at the reference state is
    /// `tau_ref_s` — the steady film mass there stays the firmware's x·m·τ, the light ends leave first and the heavy ends
    /// hang on, the more the colder the port.
    pub fn tau_comp_s(&self, t_cool_c: f64, p_im_kpa: f64, air_kg_s: f64) -> [[f64; 3]; 2] {
        use crate::combustion::{fuel_psat, FUEL_PSEUDO};
        let spread = |t: f64| -> [f64; 3] { std::array::from_fn(|c| fuel_psat(1, t) / fuel_psat(c, t)) };
        let tau = self.tau_s(t_cool_c, p_im_kpa, air_kg_s);
        std::array::from_fn(|i| {
            let (t, t_ref) = (t_cool_c + 273.15 + self.surface_over_coolant_k[i], self.t_ref_c + 273.15 + self.surface_over_coolant_k[i]);
            let norm: f64 = spread(t_ref).iter().zip(FUEL_PSEUDO).map(|(s, f)| f.0 * s).sum();
            spread(t).map(|s| tau[i] * s / norm)
        })
    }
}

/// EA888 Gen3 port films, anchored at warm idle (coolant 90 °C, 30–40 kPa, 700–800 rpm) to the calibrations' own
/// two-film model of this port (FMSP 41.19, FR p.11632; code `0x800B9DC6`, identical structure in 259J and 264H): film 0
/// deposition `ip_fac_mff_wf_mpi[0][1]` × `ip_fac_mff_wf_im_temp_cor` = 0.185–0.206, evaporation per cycle k0·f0 =
/// 0.80 × 0.82 = 0.66 → τ ≈ 0.15 s; film 1 0.139–0.157, 0.15 × 0.82 = 0.123 per cycle → τ ≈ 1.2 s (VERIFIED-S as the
/// firmwares' identification; docs/SIMOS18_GPTA_IO_RE.md §12.7). The earlier single film (x 0.30, τ 0.19 s) took the
/// raw map value without its 0.62 temperature factor and lacked the slow film. Surface temperatures (valve back
/// coolant + 60 K, port wall = coolant), ΔH_vap 35 kJ/mol (mid-boiling gasoline, Heywood 1988 table D.4), engine air flow
/// at the reference 3 g/s and port volume (≈ 0.25 l per cylinder) UNVERIFIED.
pub const PORT_FILM_EA888_GEN3: PortFilmHw = PortFilmHw {
    x: [0.19, 0.15],
    tau_ref_s: [0.15, 1.2],
    surface_over_coolant_k: [60.0, 0.0],
    t_ref_c: 90.0,
    p_ref_kpa: 35.0,
    flow_ref_kg_s: 3.0e-3,
    dh_vap_j_mol: 35_000.0,
    ivd_factor: 0.0,
    port_volume_m3: 0.25e-3,
};

#[derive(Debug, Clone, Serialize)]
pub struct EngineProfile {
    pub id: EnginePresetId,
    pub slug: &'static str,
    pub name: &'static str,
    pub engine_code: &'static str,
    pub cylinders: usize,
    pub bore_m: f64,
    pub stroke_m: f64,
    pub compression_ratio: f64,
    /// Crankshaft + damper + timing drive + DMF primary (crank side of the arc springs), kg·m².
    pub crank_inertia_kgm2: f64,
    /// Throttle body effective (reduced) flow area Cd·A, cm², vs plate angle from the lower stop, deg.
    pub throttle_cda_cm2: &'static [(f64, f64)],
    /// Flow area the bore/plate deposits (PCV oil mist + carbon) take from [`Self::throttle_cda_cm2`], cm²: a layer of
    /// thickness δ narrows the plate gap along the whole perimeter, ΔA ≈ π·D·δ — an additive offset, the quantity the
    /// firmware's additive throttle adaptation `AR_THR_AD_ADD` (cm², 894.07e-6 cm²/bit, FR p.13814) learns at idle.
    pub throttle_deposit_cm2: f64,
    /// Throttle plate spring rest (limp-home) opening, degrees from the lower stop.
    pub throttle_limp_home_deg: f64,
    pub etc: EtcActuator,
    pub intake_volume_m3: f64,
    /// Compressor outlet → throttle (pipes + intercooler) volume, m³.
    pub boost_volume_m3: f64,
    /// Exhaust manifold (IEM) + turbine housing volume, m³.
    pub exhaust_volume_m3: f64,
    /// DI injector: static flow at 10 MPa ΔP (rail − cylinder), cm³/s at the plant's fuel density (flow
    /// ∝ √ΔP), and the electrical time before fuel flows (opening delay − closing delay), ms.
    pub injector_flow_cc_s: f64,
    pub injector_dead_time_ms: f64,
    /// HPFP displacement per plunger stroke, cm³, and cam lobes per camshaft revolution.
    pub hpfp_stroke_cc: f64,
    pub hpfp_lobes: f64,
    pub rail_volume_cc: f64,
    /// Mechanical pressure-relief valve opening pressure, MPa.
    pub rail_prv_mpa: f64,
    /// HPFP volumetric efficiency (delivered / displaced after closing) and internal loss per stroke.
    pub hpfp_efficiency: f64,
    pub hpfp_loss: &'static HpfpLossMap,
    /// N276 (normally open) quantity-control valve: closing delay after current-on vs battery voltage, (V, ms);
    /// armature return after the current is off, ms; rail pressure above which an inlet valve closed during the
    /// up-stroke is held shut by the pump-chamber pressure until plunger TDC (self-locking), MPa.
    pub hpfp_vcv_close_delay_ms: &'static [(f64, f64)],
    pub hpfp_vcv_release_ms: f64,
    pub hpfp_vcv_lock_mpa: f64,
    /// MPI port injectors on the low-pressure supply: static flow at `mpi_ref_dp_kpa` (flow ∝ √ΔP), mg/ms, and dead
    /// time vs battery voltage, (V, ms).
    pub mpi_injector_mg_ms: f64,
    pub mpi_ref_dp_kpa: f64,
    pub mpi_dead_time_ms: &'static [(f64, f64)],
    pub port_film: PortFilmHw,
    /// Intake-manifold heat pick-up of the charge air (plenum + runners bolted to the head; the manifold air sensor reads
    /// the heated charge): wall → air UA at the reference air flow, W/K (∝ ṁ^0.8), and head flange → wall UA, W/K
    /// (`engine::manifold_air_temp_k`).
    pub intake_wall_ua_w_k: f64,
    pub intake_wall_ua_ref_kg_s: f64,
    pub intake_wall_head_ua_w_k: f64,
    pub intercooler_effectiveness: f64,
    /// Intake, charge-air and exhaust path restrictions (bolt-on parts; built by `bench_config`).
    pub gas_path: GasPathHw,
    pub turbo: TurboHardware,
    pub mech: Mechanics,
    /// Knock sensors (count, position, sensitivity) and the block's structure-borne paths to them.
    pub knock: KnockSensorHw,
    /// Port-flap position sensor output with the flaps [open, closed], V.
    pub port_flap_sensor_v: [f64; 2],
}

/// Air and exhaust path outside the long block. [`GasPathHw::stock`] is the plant as calibrated before these were
/// fields (`engine::AIR_FILTER_K`, `engine::EXHAUST_SYSTEM_K`, compressor inlet at ambient temperature);
/// `bench_config` derives the modified parts from pipe diameters and loss coefficients.
#[derive(Debug, Clone, Serialize)]
pub struct GasPathHw {
    /// Air filter + inlet duct up to the compressor: Δp = k·ṁ², Pa/(kg/s)².
    pub air_filter_k: f64,
    /// Share of the compressor inlet air drawn from the engine bay (0 = fresh-air snorkel of the stock airbox,
    /// 1 = open filter in the bay). UNVERIFIED.
    pub inlet_underhood_share: f64,
    /// Charge-air path (compressor outlet → throttle) pressure-drop coefficient relative to the stock path,
    /// Pa/(kg/s)². Compressor speed lines are referenced to PUT downstream of the stock intercooler path,
    /// so stock = 0 and a lower-loss intercooler core is negative.
    pub charge_air_dp_rel_k: f64,
    /// Exhaust after the turbine (downpipe, catalyst, silencers): turbine outlet p4 = p_amb + k·ṁ², Pa/(kg/s)².
    pub exhaust_k: f64,
}

/// Engine-bay air temperature rise over ambient as a share of (coolant − ambient) with the car standing, and the
/// ram-air speed that halves it: an open cone without a cold-air feed measured +23 K at idle and +17 K cruising over
/// ambient (strikeengine.com, non-MQB car — secondary) → 23 K / 70 K and 25 m/s / (23/17 − 1). UNVERIFIED for MQB.
pub const UNDERHOOD_RISE_FRAC: f64 = 0.33;
pub const UNDERHOOD_V_REF_MS: f64 = 70.0;

impl GasPathHw {
    pub fn stock() -> Self {
        Self { air_filter_k: crate::engine::AIR_FILTER_K, inlet_underhood_share: 0.0, charge_air_dp_rel_k: 0.0, exhaust_k: crate::engine::EXHAUST_SYSTEM_K }
    }

    /// Compressor inlet temperature rise over ambient from engine-bay air (heat soak), K. Steady state: the bay
    /// follows the coolant temperature, ram air at `v_ms` dilutes it.
    pub fn inlet_rise_k(&self, t_amb_k: f64, coolant_c: f64, v_ms: f64) -> f64 {
        let warm = (coolant_c + 273.15 - t_amb_k).max(0.0);
        self.inlet_underhood_share * UNDERHOOD_RISE_FRAC * warm / (1.0 + v_ms.abs() / UNDERHOOD_V_REF_MS)
    }
}

impl EngineProfile {
    pub fn displacement_m3(&self) -> f64 {
        std::f64::consts::PI / 4.0 * self.bore_m * self.bore_m * self.stroke_m * self.cylinders as f64
    }
    pub fn cylinder_volume_m3(&self) -> f64 {
        self.displacement_m3() / self.cylinders as f64
    }

    pub fn ea888_18t_cjsa_is12() -> Self {
        Self {
            id: EnginePresetId::Ea888_18tCjsaIs12,
            slug: "1.8t_is12",
            name: "1.8 TSI EA888 Gen3 (CJSA/CJSB) + IHI IS12",
            engine_code: "CJSA",
            cylinders: 4,
            bore_m: 0.0825,
            stroke_m: 0.0841,
            compression_ratio: 9.6,
            crank_inertia_kgm2: 0.115,
            throttle_cda_cm2: &THROTTLE_CDA_EA888_GEN3,
            // The bench car's throttle body as its ECU last learned it: the bench DFlash restores AR_THR_AD_ADD_MMV =
            // 0xFFC2 = −0.0554 cm² at power-up (UDS DID 0x13D2, VERIFIED-T; SC8S50 allows −0.100…+0.030 cm²,
            // `c_ar_thr_ad_add_min/max` @0x80809B20/1E). ≈ 31 µm of deposit on the Ø56 mm perimeter (π·D·δ, Cd 1);
            // whether it is deposit, a part tolerance or both is UNVERIFIED. A clean nominal throttle (0) under that
            // ECU needs +48 % area at idle — AMP_TPS_PLAUS / P0068 (docs/SIMOS18_COMBUSTION.md §9).
            throttle_deposit_cm2: 0.0554,
            // SC800S50 `c_tps_lih_ini` 7.0 °TPS @0x8080C050 = `c_vp_tps_sp_lih_1` 0.8333 V @0x8080C0C6 on the
            // 21 °/V sensor (VERIFIED-S: the calibrated nominal of this throttle body).
            throttle_limp_home_deg: 7.0,
            etc: EtcActuator {
                r_ohm: 1.6,
                k_nm_a: 0.10,
                inertia_kgm2: 1.2e-3,
                close_preload_nm: 0.12,
                close_rate_nm_rad: 0.05,
                open_preload_nm: 0.10,
                open_rate_nm_rad: 0.3,
                friction_nm: 0.015,
                max_deg: 90.0,
            },
            intake_volume_m3: 2.6e-3,
            boost_volume_m3: 5.0e-3,
            exhaust_volume_m3: 1.0e-3,
            // CJSA's own calibration family (SC8O40 8V0906264H; `ip_ti_4_l` identical in SC8O30):
            // least-squares fit of ti = m / (q·√(ΔP/10 MPa)) + t0 over 10–50 mg and 4–15 MPa gives q = 10.52 mg/ms at
            // 10 MPa = 14.12 cm³/s, t0 0.017 ms (max residual 0.052 ms; VERIFIED-S as the firmware's characteristic).
            // The ballistic range below ≈ 10 mg is not modelled.
            injector_flow_cc_s: 14.12,
            injector_dead_time_ms: 0.017,
            // Theoretical stroke volume = SC8O40 `c_mfp_max` 174.5 mg per pump stroke @0x808085C6 at 745 kg/m³
            // (VERIFIED-S).
            hpfp_stroke_cc: 0.2342,
            hpfp_lobes: 4.0, // UNVERIFIED (EA888 Gen3 HPFP cam believed 4-lobe)
            // High-pressure volume between pump outlet and injectors (rail + line), UNVERIFIED. 25 cm³ made one
            // pump stroke move the rail by ≈ 0.64 bar/mg, so the firmware's rail controller (P part 1.5 mg/bar,
            // `ip_fuel_mass_req_p_ctl` @0x8080189E) overshot by 2–4 MPa per stroke at idle; 50 cm³ (0.32 bar/mg)
            // gives the damped response that controller is designed for (studio::engine_run).
            rail_volume_cc: 50.0,
            rail_prv_mpa: 23.0,
            // SC8O40 pump model at idle (VERIFIED-S as the calibration's model of this pump; speed/fuel-temperature
            // dependence not modelled): `ip_fac_fuel_pump_eff_fup_n` 0.852 (608–1024 rpm) @0x8083BBD4 ×
            // `ip_fac_fuel_pump_eff_tfu_n` 0.967 (fuel ≈ 40 °C) @0x8083BC54 × `ip_fac_fuel_pump_eff_mff_sp_n` 0.996
            // @0x8083BC14.
            hpfp_efficiency: 0.820,
            hpfp_loss: &HPFP_LOSS_SC8O40_264H,
            hpfp_vcv_close_delay_ms: &VCV_CLOSE_DELAY_SC8O40_264H,
            // UNVERIFIED: return time of the armature + needle once the coil current has decayed (solenoid valves of
            // this size: a fraction of a millisecond to ≈ 1 ms).
            hpfp_vcv_release_ms: 0.5,
            // `c_fup_min_lock_vcv` 20.0 bar @0x808085BC "minimum rail pressure for self locking of the VCV" (VERIFIED-S).
            hpfp_vcv_lock_mpa: 2.0,
            // `ip_fac_mff_ti_stnd_h_rng_mpi[0]` 0.434 ms/mg (@0x80808DBA) at the pressure difference where
            // `ip_fac_ti_prs_mpi` (@0x80847EBA) is 1.0: 5004 hPa; that curve is √(5 bar/ΔP) within 3 % (VERIFIED-S).
            mpi_injector_mg_ms: 1.0 / 0.434,
            mpi_ref_dp_kpa: 500.4,
            mpi_dead_time_ms: &MPI_DEAD_TIME_EA888_GEN3,
            port_film: PORT_FILM_EA888_GEN3,
            // Warm-idle heat soak of PA6-GF30 intake manifold (ε ≈ 0.35 at 16 kg/h → series UA 1.9 W/K at 4.44 g/s;
            // flange 4 W/K, air side 3.6 W/K at 4.44 g/s; UNVERIFIED split).
            intake_wall_ua_w_k: 3.6,
            intake_wall_ua_ref_kg_s: 4.44e-3,
            intake_wall_head_ua_w_k: 4.0,
            intercooler_effectiveness: 0.72,
            gas_path: GasPathHw::stock(),
            turbo: TurboHardware {
                model: "IHI IS12",
                n_ref_rpm: 200_000.0,
                choke_flow_ref_kg_s: 0.17,
                pr_zero_flow_ref: 2.4,
                eta_compressor: 0.70,
                turbine_cda_m2: 2.6e-4,
                wastegate_cda_m2: 6.0e-4,
                eta_turbine: 0.65,
                turbine_tip_radius_m: 0.0185,
                inertia_kgm2: 2.0e-5,
                compressor_tip_radius_m: 0.0245,
                wg_actuator: WastegateActuatorHw::ea888_gen3_is12(),
                // `compressor_map::IS12_SC8O40` (the IS12 as SC8O40 models it) is not switched on: with it the plant's
                // turbine (orifice `turbine_cda_m2`, η over U/C, mean manifold pressure — UNVERIFIED) cannot spool the
                // IS12 below 2000 rpm (45 krpm, 1.08 PR at 1250 rpm WOT against the rated 250 Nm), because it passes
                // 1.3–1.6× the corrected flow of the firmware's own turbine model (`ip_flow_tur_cor_pq_tur`, N 100–120
                // krpm, PR 1.2–1.6). Needs the turbine side from the same model first (MILESTONE §5 п.12).
                map: None,
                eta_loss: 0.0,
                // The port orifice passed ≈ 1.6× the flow of the stock calibration's own wastegate model at the same
                // actuator position and gas state (bench kickdown, SC8O40 RAM; docs/SIMOS18_COMBUSTION.md §17) — the
                // turbocharger controller's correction then crossed the leak-diagnosis threshold (P0299).
                wastegate_map: Some(&crate::wastegate::IS12_SC8O40_WASTEGATE),
            },
            // 1.8 exhaust: standard lobe 9 mm / 195 ° (SC8O40 264H `c_vlft_psn_ex`, `ip_cam_op_vcp[1]`; P1 p.65 lists the
            // 10 mm exhaust lift as a 2.0 difference), LFT_1 6.35 mm / 180 ° (`AvsHw`).
            mech: Mechanics::ea888_gen3_base(0.148).with_exhaust_lobe(9.0e-3, 195.0),
            knock: KnockSensorHw::ea888_gen3(),
            // At the pin: the firmware inverts G336 through `ip_v_port_1_cor` (SC8S50 0x80201A78: 0 → 4.995 V,
            // 4.995 → 0 V) before comparing with its nominal stops `c_v_port_open_ini` 3.799 V / `c_v_port_close_ini`
            // 1.196 V (VERIFIED-S; the firmware adapts ±0.70 V), so open = 4.995 − 3.799, closed = 4.995 − 1.196.
            port_flap_sensor_v: [1.196, 3.799],
        }
    }

    pub fn ea888_18t_is38_hybrid() -> Self {
        Self {
            id: EnginePresetId::Ea888_18tIs38Hybrid,
            slug: "1.8t_is38_hybrid",
            name: "1.8 TSI CJSA + IHI IS38 + 60 mm FMIC",
            intercooler_effectiveness: 0.86,
            turbo: Self::is38(),
            ..Self::ea888_18t_cjsa_is12()
        }
    }

    /// High-mileage 1.8 TSI CJSA + IHI IS38 hybrid profile with stock intercooler and colder NGK SILFER8C7ES
    /// spark plugs (heat range 8, gap 0.7 mm — UNVERIFIED). Wear: 1.5× ring-pack leakage area and 0.2 mm
    /// combustion-chamber deposits (geometric compression 9.6 → 10.0; UNVERIFIED).
    pub fn ea888_18t_cjsa_is38() -> Self {
        let base = Self::ea888_18t_cjsa_is12();
        let mech = Mechanics {
            ring_leak_cda_m2: 1.5 * base.mech.ring_leak_cda_m2,
            spark_gap_m: 0.0007,
            spark_plug: SparkPlugHw { heat_range: 8.0, ..SparkPlugHw::ea888_oe() },
            deposit_m: 0.2e-3,
            cyl_spread: CylinderSpread::new(0x0213_2738, 250_000.0),
            ..base.mech.clone()
        };
        Self {
            id: EnginePresetId::Ea888_18tCjsaIs38,
            slug: "cjsa_is38",
            name: "1.8 TSI EA888 Gen3 (CJSA) + IHI IS38 (high-mileage)",
            compression_ratio: 10.0,
            injector_flow_cc_s: 14.12,
            injector_dead_time_ms: 0.017,
            turbo: TurboHardware { choke_flow_ref_kg_s: 0.469, pr_zero_flow_ref: 4.68, ..Self::is38() },
            mech,
            ..base
        }
    }

    pub fn ea888_20t_chhb_is20() -> Self {
        Self {
            id: EnginePresetId::Ea888_20tChhbIs20,
            slug: "2.0t_is20",
            name: "2.0 TSI EA888 Gen3 (CHHB) + IHI IS20",
            engine_code: "CHHB",
            stroke_m: 0.0928,
            turbo: TurboHardware {
                model: "IHI IS20",
                choke_flow_ref_kg_s: 0.20,
                pr_zero_flow_ref: 2.6,
                turbine_cda_m2: 4.4e-4,
                turbine_tip_radius_m: 0.0205,
                inertia_kgm2: 2.8e-5,
                compressor_tip_radius_m: 0.0255,
                map: None,
                wastegate_map: None,
                ..Self::ea888_18t_cjsa_is12().turbo
            },
            mech: Mechanics::ea888_gen3(0.144),
            ..Self::ea888_18t_cjsa_is12()
        }
    }

    pub fn ea888_20t_cjxc_is38() -> Self {
        Self {
            id: EnginePresetId::Ea888_20tCjxcIs38,
            slug: "2.0t_is38",
            name: "2.0 TSI EA888 Gen3 (CJXC) + IHI IS38",
            engine_code: "CJXC",
            stroke_m: 0.0928,
            compression_ratio: 9.3,
            // Fuel hardware of the 2.0 calibration 8V0906259J (SC800S50, `c_dpl` 2.0 l @0x80809B84; the 1.8 SC8S50
            // 264K and SC8O40 read 1.8 l), all VERIFIED-S as that calibration's characterisation of its hardware:
            // DI injector `ip_ti_4_l` @0x8084A214 fitted like the CJSA one (10–50 mg, 4–15 MPa, 60 points, max
            // residual 0.057 ms; `ip_ti_4_h` agrees at 14–21.5 MPa): q = 14.14 mg/ms = 18.98 cm³/s, t0 0.037 ms.
            injector_flow_cc_s: 18.98,
            injector_dead_time_ms: 0.037,
            // `c_mfp_max` 232 mg @0x8080865E; efficiency `ip_fac_fuel_pump_eff_fup_n` 0.898 @0x8083C026 ×
            // `_tfu_n` 0.902 (fuel ≥ 20 °C) @0x8083C0A6 × `_mff_sp_n` 0.996 @0x8083C066; loss `ip_mfp_ofs`; valve
            // type 0 (`lc_swi_vcv_typ` @0x808086F3) with the closing delay `ip_t_dly_vb_add`; self-locking above
            // `c_fup_min_lock_vcv` 25.0 bar @0x80808654.
            hpfp_stroke_cc: 0.311,
            hpfp_efficiency: 0.807,
            hpfp_loss: &HPFP_LOSS_SC8S50_259J,
            hpfp_vcv_close_delay_ms: &VCV_CLOSE_DELAY_SC8S50_259J,
            hpfp_vcv_lock_mpa: 2.5,
            turbo: Self::is38(),
            mech: Mechanics::ea888_gen3(0.144),
            ..Self::ea888_18t_cjsa_is12()
        }
    }

    /// Turbocharger hardware by slug (bench configuration: any turbo on any engine).
    pub fn turbo_from_slug(slug: &str) -> Option<TurboHardware> {
        match slug.to_lowercase().as_str() {
            "is12" => Some(Self::ea888_18t_cjsa_is12().turbo),
            "is20" => Some(Self::ea888_20t_chhb_is20().turbo),
            "is38" => Some(Self::is38()),
            _ => None,
        }
    }

    fn is38() -> TurboHardware {
        TurboHardware {
            model: "IHI IS38",
            n_ref_rpm: 200_000.0,
            choke_flow_ref_kg_s: 0.25,
            pr_zero_flow_ref: 2.9,
            eta_compressor: 0.72,
            turbine_cda_m2: 5.6e-4,
            wastegate_cda_m2: 7.0e-4,
            eta_turbine: 0.68,
            turbine_tip_radius_m: 0.0225,
            inertia_kgm2: 3.8e-5,
            compressor_tip_radius_m: 0.0270,
            wg_actuator: WastegateActuatorHw::ea888_gen3_is38(),
            map: None,
            eta_loss: 0.0,
            wastegate_map: Some(&crate::wastegate::IS12_SC8O40_WASTEGATE),
        }
    }

    pub fn from_slug(slug: &str) -> Option<Self> {
        match slug.to_lowercase().as_str() {
            "1.8t" | "1.8t_is12" | "cjsa" | "is12" => Some(Self::ea888_18t_cjsa_is12()),
            "1.8t_is38_hybrid" => Some(Self::ea888_18t_is38_hybrid()),
            "cjsa_is38" => Some(Self::ea888_18t_cjsa_is38()),
            "2.0t_is20" | "is20" | "chhb" => Some(Self::ea888_20t_chhb_is20()),
            "2.0t_is38" | "is38" | "cjxc" => Some(Self::ea888_20t_cjxc_is38()),
            _ => None,
        }
    }
}


/// Valve gear of one valve type (per valve unless stated). Cam profile: polynomial (1−u²)³ over
/// the zero-lift duration (C² continuous, zero seating velocity when the follower stays on the cam).
#[derive(Debug, Clone, Serialize)]
pub struct ValveGear {
    pub count: f64,
    pub head_diameter_m: f64,
    pub stem_diameter_m: f64,
    pub max_lift_m: f64,
    /// Zero-lift opening duration, °CA.
    pub duration_deg: f64,
    /// Flank exponent n of the (1 − u²)ⁿ profile (`valvetrain::lobe_lift`).
    pub shape_n: f64,
    /// Max-lift crank angle at phaser rest, °CA after firing TDC of the cylinder (0..720).
    pub centerline_deg: f64,
    /// Phaser authority, °CA, and its direction (−1 = advances/earlier, +1 = retards/later).
    pub phaser_range_deg: f64,
    pub phaser_dir: f64,
    /// Discharge coefficient referred to min(curtain, throat) area.
    pub cd: f64,
    /// Valve + retainer + ⅓ spring + reflected roller-finger-follower mass, kg. UNVERIFIED.
    pub moving_mass_kg: f64,
    /// Cam phaser of this camshaft (spool valve + vane rotor hydraulics).
    pub phaser: PhaserHw,
    /// Valve spring installed preload, N, and rate, N/m. UNVERIFIED (chosen for float onset ≈ 7.7–7.9 krpm).
    pub spring_preload_n: f64,
    pub spring_rate_n_m: f64,
    pub damping_ns_m: f64,
    /// Valve axis inclination to the bore axis, deg.
    pub inclination_deg: f64,
}

/// Reference lift of the calibrations' valve opening periods (`ip_cam_op_vcp`), m. HYP: VW quotes valve timing at 1 mm
/// lift; the calibrations do not state it.
pub const CAM_REF_LIFT_M: f64 = 1e-3;

/// Polynomial lobe fields (max lift, zero-lift duration, centre) from a lift `l_m`, an opening period `period_deg` at
/// [`CAM_REF_LIFT_M`], the closing angle at that lift `close_deg` (°CA after the firing TDC) and the flank exponent `n`: the
/// (1 − u²)ⁿ profile of `valvetrain::lobe_lift` reaches the reference lift at u = √(1 − (l_ref/L)^(1/n)).
pub fn lobe_from_period(l_m: f64, period_deg: f64, close_deg: f64, n: f64) -> (f64, f64, f64) {
    let u = (1.0 - (CAM_REF_LIFT_M / l_m).powf(1.0 / n)).sqrt();
    (l_m, period_deg / u, close_deg - 0.5 * period_deg)
}

/// Exhaust valvelift system (Audi AVS, two-stage exhaust lift; SC8L50 FR VVLI 78.1, p.26495–26498: NC_NR_VVL_EX = 2
/// "exhaust VVL (Avs)", electromechanical actuators, `NLC_VVL_ACR_HYD` = 0). Each cylinder's exhaust lobes sit on an
/// axially sliding cam piece carrying a standard (large, `Mechanics::exhaust`) and an LFT_1 (small) lobe for both valves.
/// Two actuators per piece push a pin into a spiral groove: the pin, once out, rides on the piece (bistable solenoid —
/// FR 78.3 p.26504) and enters the groove when its entry passes; the groove's helix shifts the piece while the lobes
/// are on the base circle; its run-out pushes the pin back, which induces the voltage the firmware evaluates (FR 78.1.12).
///
/// Calibration data (VERIFIED-S as the calibrations' model of this hardware): lifts `c_vlft_psn_ex` = [10.0, 6.35] mm
/// (SC8S50 259J @0x8080D3DC, CJXC) and [9.0, 6.35] mm (SC8O40 264H @0x8080D270, CJSA); opening periods `ip_cam_op_vcp[1]`
/// 180 ° at 6.35 mm, 215 ° at 10 mm (259J @0x8080D768) / 195 ° at 9 mm (264H @0x8080D5FC); one exhaust closing point
/// `c_cam_ind_eng[1]` = −24 °CRK for both lobes (HYP: the firmware keeps a single closing point, so both lobes close
/// together and the small one opens later). Firmware side (SC8S50, docs/SIMOS18_COMBUSTION.md §6b): the actuator current
/// runs `T_SWI_VVL_ACR` (`ip_t_swi_vvl_acr` 70 ms at ≤ 992 rpm … 17 ms at 4992 rpm × a battery factor) until the hit position
/// `c_crk_vvl_acr` = 292.1 ° after each cylinder's firing TDC (+ exhaust cam phase; FR 78.4.2.1.1), by which the pin must
/// be out; actuator dead time `ip_t_dly_swi_vvl_acr` 4.0 ms (@0x8086BF66); the recoil is expected in the window
/// `c_crk_beg_vvl_diag` / `c_crk_dur_vvl_diag` = hit + 600 … 740 ° (VERIFIED-S). The firmware's acknowledged lift follows its
/// command, not the recoil (FR p.26606).
#[derive(Debug, Clone, Serialize)]
pub struct AvsHw {
    /// LFT_1 lobe: max lift, m, zero-lift duration, °CA, and centre at phaser rest, °CA after the firing TDC.
    pub small_lift_m: f64,
    pub small_duration_deg: f64,
    pub small_centerline_deg: f64,
    /// Flank exponent of the LFT_1 lobe, (1 − u²)ⁿ (`valvetrain::lobe_lift`). FITTED 2.0 to the firmware's own charge model
    /// at its LFT_1 part-load operating points (firmware in the loop, `dyno_run::part_load_air_vs_firmware`,
    /// docs/SIMOS18_COMBUSTION.md §6.4): with the (1 − u²)³ flank of the large lobes a 6.35 mm lobe needs a 265 ° zero-lift
    /// span to reach 180 ° at 1 mm and the firmware/plant air ratio spread to 0.94–1.12; n = 2 (a fuller flank, 232 ° span)
    /// gives 0.98–1.06. The real profile is not known (UNVERIFIED).
    pub small_shape_n: f64,
    /// Groove entry passing under the pin, °CA after the firing TDC at phaser rest: after the hit position and after both
    /// lobes have closed (zero lift ≤ 379 °CA), so the piece moves on the base circle. UNVERIFIED.
    pub groove_entry_deg: f64,
    /// Crank angle over which the helix moves the piece (inside the base circle, which ends at the next opening ≈ 800 °CA),
    /// and the run-out angle that ejects the pin, °CA after the firing TDC (centre of the firmware's recoil window). UNVERIFIED.
    pub shift_deg: f64,
    pub eject_deg: f64,
    /// Actuator pin: dead time after current-on (calibration) and travel time at 13.5 V, s (UNVERIFIED: shorter than the
    /// shortest switching time the firmware allows, `ip_t_swi_vvl_acr` 17 ms at 5000 rpm, less the dead time).
    pub pin_dead_s: f64,
    pub pin_travel_s: f64,
    /// Position of the cam pieces when the plant is built (0 = standard, 1 = LFT_1): LFT_1, the state the firmware restores
    /// from NVM (STATE_ACK_VVL_RST; bench DFlash: all cylinders LFT_1, `lc_state_ack_vvl_ini` = 1 without NVM data) — the
    /// engine was stopped at idle, where both calibrations select LFT_1.
    pub initial_pos: f64,
}

impl AvsHw {
    /// EA888 Gen3 exhaust AVS with the LFT_1 lobe of both calibrations (6.35 mm, 180 °, closing at −24 ° = 336 °CA).
    pub fn ea888_gen3() -> Self {
        let n = 2.0;
        let (l, d, c) = lobe_from_period(6.35e-3, 180.0, 336.0, n);
        Self {
            small_lift_m: l,
            small_duration_deg: d,
            small_centerline_deg: c,
            small_shape_n: n,
            groove_entry_deg: 380.0,
            shift_deg: 300.0,
            eject_deg: 292.1 + 670.0,
            pin_dead_s: 4.0e-3,
            pin_travel_s: 8.0e-3,
            initial_pos: 1.0,
        }
    }
}

/// Vane cam phaser and its proportional spool valve (N205 intake / N318 exhaust).
///
/// Anchors (SC800S50 calibration, the firmware's own model of this hardware, VERIFIED-S): hold duty
/// `ip_pwm_hld_vcp` @0x8086DCF4 at a 90 °C coil = 76.3 / 60.3 / 47.6 / 41.5 % at 8.5 / 11.1 / 14.0 / 16.0 V, i.e. a
/// constant mean coil voltage ≈ 6.6 V = constant current → null current = 6.6 V / R(90 °C); adjustment speed
/// `ip_vel_ivs_act/pas_vcp` 0.9–1.8 ms/° hot (550–1100 °CRK/s), 4.5–11 ms/° at −20 °C; slower in the low oil-pressure
/// stage (`ip_fac_vel_ivs_poil_dec_ivvt` 1.2–2.2); valve response `ip_dly_act/pas_vcp` 10–50 ms. Vane-phaser
/// literature: rotor speed is set by the spool's orifice flow (∝ opening·√Δp) and slowed by viscous leakage when cold.
/// Coil resistance, spool band and ν_c are UNVERIFIED knobs fitted to those anchors.
///
/// The hold duty the firmware actually commands (FR VVTI 79.16; sum traced at 0x8017A0E8..0x8017A17A, VERIFIED-T by
/// observation probes on a 30 °C engine): `ip_pwm_hld_vcp`(coil substitute temperature, VB) + `ip_fac_pwm_hld_ofs_vcp`
/// (temperature) × `ip_pwm_hld_ofs_vcp[cam]`(rpm, load) + the adaptation table value read from NVM. For the intake
/// cam the offset map is −5…−10 % (−7 % over 1000–4000 rpm, 150–1100 mg/stk) and the bench ECU's NVM holds −3.4 %
/// (intake) / −4.7 % (exhaust; its offset map is 0) — so the real phasers hold well below the base map, which is
/// what [`Self::hold_ofs`] / [`Self::hold_adapt`] put into the plant's null (the load dependence of the offset,
/// ±1.5 %, stays inside the spool overlap and is not modelled).
#[derive(Debug, Clone, Serialize)]
pub struct PhaserHw {
    /// Solenoid resistance at 20 °C, Ω (copper, α = 0.393 %/K).
    pub coil_r20_ohm: f64,
    /// Constant-current part of the spool null (`ip_pwm_hld_vcp`, 20–130 °C), A, and the current offset for full spool
    /// travel, A.
    pub null_a: f64,
    /// Hold-duty offsets at 14 V, duty fraction: the speed/load offset at a hot coil (`ip_pwm_hld_ofs_vcp`, weighted by
    /// [`HOLD_OFS_FAC`]) and the learned adaptation (bench ECU NVM).
    pub hold_ofs: f64,
    pub hold_adapt: f64,
    pub band_a: f64,
    /// Spool overlap: fraction of the travel around the centre in which both control lands stay covered (no flow;
    /// what lets a proportional valve hold the cam against small current errors).
    pub overlap: f64,
    /// Valve response time constant (spool + magnet), s.
    pub response_s: f64,
    /// Rotor speed at full spool, 3 bar supply and 90 °C oil, °CA/s; viscous slow-down 1/(1 + (ν/ν_c)^n), normalised
    /// to 90 °C. ν_c = 60 cSt, n = 0.5: fit (in log space) of the speed ratio v(T)/v(90 °C) of `ip_vel_ivs_act_vcp[0]`
    /// @0x8086DEEC at 2016–4000 rpm (−20 °C 0.11–0.15, 0 °C 0.25–0.28, 40 °C 0.53–0.91 of the hot speed; VERIFIED-S as
    /// the firmware's model of this actuator) → 0.15 / 0.32 / 0.69. The former ν_c = 1270 cSt fitted only −20 °C and
    /// left the phaser at 0.9× its hot speed at 30 °C, 1.6–1.9× faster than the firmware's pulse pre-control assumes.
    pub rate_deg_s: f64,
    pub visc_c_cst: f64,
    pub visc_exp: f64,
    /// Drift towards rest from the mean cam friction torque with the spool centred, °CA/s.
    pub drift_deg_s: f64,
}

/// Temperature weighting of the hold-duty offset, (coil °C, factor): `ip_fac_pwm_hld_ofs_vcp[0]` (VERIFIED-S; 0.52 at
/// 24 °C traced, VERIFIED-T).
pub const HOLD_OFS_FAC: [(f64, f64); 6] = [(-20.0, 0.35), (0.0, 0.40), (40.0, 0.60), (70.0, 0.86), (90.0, 1.0), (120.0, 1.0)];

impl PhaserHw {
    pub fn ea888_gen3() -> Self {
        Self {
            coil_r20_ohm: 7.5,
            // constant-current null: the base hold map 47.6 % × 14 V at a 90 °C coil
            null_a: 0.476 * 14.0 / (7.5 * (1.0 + 0.00393 * 70.0)),
            hold_ofs: 0.0,
            hold_adapt: 0.0,
            band_a: 0.25,
            overlap: 0.2,
            response_s: 0.015,
            rate_deg_s: 800.0,
            visc_c_cst: 60.0,
            visc_exp: 0.5,
            drift_deg_s: 15.0,
        }
    }

    /// Coil resistance at `t_c`.
    pub fn coil_r(&self, t_c: f64) -> f64 {
        self.coil_r20_ohm * (1.0 + 0.00393 * (t_c - 20.0))
    }

    /// Spool null current at coil temperature `t_c`, A.
    pub fn null_current(&self, t_c: f64) -> f64 {
        self.null_a + (self.hold_ofs * crate::sensors::interp(t_c, &HOLD_OFS_FAC) + self.hold_adapt) * 14.0 / self.coil_r(t_c)
    }

    /// Rotor-speed factor of the oil temperature (1 at 90 °C).
    pub fn visc_factor(&self, oil_c: f64) -> f64 {
        let k = |t: f64| 1.0 + (crate::atmosphere::oil_viscosity_cst(t) / self.visc_c_cst).powf(self.visc_exp);
        k(90.0) / k(oil_c)
    }

    /// Test-bench helper (plant tests without an ECU): duty that holds the cam at `battery_v` and coil `t_c`.
    pub fn hold_duty(&self, battery_v: f64, t_c: f64) -> f64 {
        self.null_current(t_c) * self.coil_r(t_c) / battery_v.max(1.0)
    }
}

/// Journal-bearing geometry (short-bearing / Ocvirk model).
#[derive(Debug, Clone, Serialize)]
pub struct JournalBearing {
    pub diameter_m: f64,
    /// Effective land width (without oil groove/chamfers), m.
    pub width_m: f64,
    /// Radial clearance, m.
    pub clearance_m: f64,
}

/// Mechanical hardware of the EA888 Gen3 long block. Sources: Audi SSP 606 "2.0l TFSI EA888 Gen3"
/// (main bearing Ø48 mm, intake/exhaust valve Ø33.85/28.0 mm, intake cam adjustment 60 °CA, exhaust
/// 30 °CA, piston cooling jets switched by N522, two-stage oil pump); everything else UNVERIFIED.
#[derive(Debug, Clone, Serialize)]
pub struct Mechanics {
    pub rod_length_m: f64,
    /// Piston pin offset towards the major-thrust side, m. UNVERIFIED.
    pub pin_offset_m: f64,
    /// Piston + pin + rings + clips, kg. UNVERIFIED.
    pub piston_mass_kg: f64,
    pub rod_mass_kg: f64,
    /// Share of the rod mass lumped at the small end (reciprocating).
    pub rod_small_end_share: f64,
    /// DMF secondary + clutch input side inertia, kg·m², arc-spring rate Nm/rad, damping Nm·s/rad,
    /// free travel ±rad before the end stops.
    pub dmf_secondary_kgm2: f64,
    pub dmf_rate_nm_rad: f64,
    pub dmf_damping_nms: f64,
    pub dmf_travel_rad: f64,
    pub intake: ValveGear,
    /// Exhaust valve gear; its lobe is the standard (large) AVS lobe when [`Self::avs`] is fitted.
    pub exhaust: ValveGear,
    /// Exhaust valvelift system (AVS), `None` = one exhaust lobe.
    pub avs: Option<AvsHw>,
    /// Design minimum valve-to-piston clearance across the full phaser authority, m. UNVERIFIED.
    pub valve_piston_min_clearance_m: f64,
    pub main_bearing: JournalBearing,
    pub rod_bearing: JournalBearing,
    /// Composite surface roughness journal/shell, m.
    pub bearing_roughness_m: f64,
    /// Variable-displacement vane pump displacement per crank revolution at full eccentricity, m³.
    pub oil_pump_m3_rev: f64,
    /// Oil fill, l (filled to the MAX mark by default).
    pub oil_capacity_l: f64,
    /// Effective ring-pack leakage area of a healthy cylinder (Cd·A), m².
    pub ring_leak_cda_m2: f64,
    /// Connecting-rod shank cross-section, m², radius of gyration, m, and outer-fibre distance, m.
    pub rod_shank_area_m2: f64,
    pub rod_gyration_radius_m: f64,
    pub rod_fibre_m: f64,
    /// Head-bolt clamp load acting on one cylinder's fire ring, N, and fire-ring seal diameter, m.
    pub fire_ring_clamp_n: f64,
    pub fire_ring_diameter_m: f64,
    /// Piston thermal network (crown, land, groove nodes; oil jet, splash, ring pack).
    pub piston_thermal: PistonThermalHw,
    /// Exhaust-valve thermal nodes (head, seat).
    pub exhaust_valve_thermal: ValveThermalHw,
    /// Second-ring-land bending factor σ = K·p (groove-root tension per unit gas pressure). UNVERIFIED (design practice:
    /// ≈ 30–40 MPa at the 100–120 bar firing pressure the land is sized for).
    pub ring_land_stress_factor: f64,
    /// Spark-plug gap, m.
    pub spark_gap_m: f64,
    /// Spark plug fitted (heat range, nose/electrode thermal nodes, fouling).
    pub spark_plug: SparkPlugHw,
    /// Combustion-chamber deposit thickness on crown and head (mean over the cylinders), m, and the deposit layer.
    pub deposit_m: f64,
    pub deposit: DepositHw,
    /// Crevice volume (top-land, gasket and plug-thread crevices) per cylinder, m³: holds unburned charge at wall
    /// temperature through the flame passage (Heywood 1988 §11.4.3: 1–2 % of the clearance volume). UNVERIFIED
    /// (EA888 top land ≈ 6 mm × 0.2 mm radial gap ≈ 0.3 cm³ + gasket/thread ≈ 0.1 cm³).
    pub crevice_m3: f64,
    /// Quasi-dimensional combustion-model constants.
    pub combustion: CombustionHw,
    /// Heat pick-up of the fresh charge between the manifold air-temperature sensor and the cylinder.
    pub intake_heat: IntakeHeatHw,
    /// Cylinder-to-cylinder spread of this long block.
    pub cyl_spread: CylinderSpread,
}

/// Heat pick-up of the fresh charge on its way from the manifold (where the air-temperature sensor sits) through the
/// intake port in the head and past the intake valve into the cylinder: the inducted air leaves at
/// T_im + ε·(T_wall − T_im), ε = 1 − exp(−h·A/(ṁ·c_p)), h = `h_mult` × Dittus–Boelter (0.023·Re^0.8·Pr^0.4·k/D) on the
/// instantaneous valve mass flow ṁ (Heywood 1988 §12.3; Re-scaling of port heat transfer as in Shayler, May & Ma,
/// SAE 950682). The heat comes out of the head (coolant).
///
/// Anchor (the real engine's measured charge heating as both calibrations identify it, VERIFIED-S as data; FR INSY
/// 51.15, p.13624–13631): the warm-engine air charge depends on the coolant temperature by
/// `ip_eff_vol_fac_eff_vol_tco` × (1 + `ip_eff_vol_fac_eff_vol_tco_cor`) = +0.051 × 1.31 = +6.7 % at 45 °C vs 95 °C at
/// idle (750 rpm, 260 hPa), +5–6 % at part load, +3.6 % at 2.5 bar (SC8S50 259J @0x8084BFCC / @0x8084C044), and on
/// the manifold air temperature by only ≈ 0.6 × the density ratio (`ip_eff_vol_fac_eff_vol_tia_dif` @0x8084C1C4). A
/// heat exchanger with ε ≈ 0.4 towards a wall at about coolant temperature gives both (d ln m/dT_w = −ε/T ≈ −0.12 %/K,
/// sensitivity to T_im (1 − ε)·T_im/T ≈ 0.56). A steady fully developed pipe-flow coefficient gives only ε ≈ 0.05–0.1
/// at idle; the real port is short, curved, unsteady and ends in the valve curtain and the cylinder's intake jets, so
/// `h_mult` is FITTED to the coolant sensitivity above (not to the air-charge level): plant trapped air at 45 °C over
/// 95 °C coolant, 760 rpm, 120 mg, φ_in 15 / φ_ex 0, manifold air held at 5 °C (`bench_tests::explore_ve`, `IM_UA0`):
/// 1.021 with `h_mult` 0 (in-cylinder walls only), 1.044 with 4, 1.067 with 10 (calibration: 1.067).
#[derive(Debug, Clone, Serialize)]
pub struct IntakeHeatHw {
    /// Port hydraulic diameter with the flaps open / closed (the closed flap leaves the upper part of the port), m.
    pub port_d_m: [f64; 2],
    /// Heated length (port in the head, valve guide boss, valve tulip), m.
    pub port_len_m: f64,
    /// Effective wall temperature over the coolant (port wall ≈ coolant, valve tulip hotter), K.
    pub wall_over_coolant_k: f64,
    /// Multiplier on the Dittus–Boelter coefficient. FITTED (see above).
    pub h_mult: f64,
}

impl IntakeHeatHw {
    /// EA888 Gen3 tumble port: ≈ 30 mm hydraulic diameter, ≈ 0.10 m from the manifold flange to the valve seat incl. the
    /// two valve tulips; closed flaps ≈ 0.7 × D. UNVERIFIED geometry (no drawing).
    pub fn ea888_gen3() -> Self {
        Self { port_d_m: [0.030, 0.021], port_len_m: 0.10, wall_over_coolant_k: 10.0, h_mult: 10.0 }
    }

    /// Heat-exchanger effectiveness for a fresh-air flow of `m_kg_s` at film temperature `t_k`, flaps `closed`.
    #[inline]
    pub fn effectiveness(&self, m_kg_s: f64, t_k: f64, closed: bool) -> f64 {
        if m_kg_s <= 0.0 || self.h_mult <= 0.0 {
            return 0.0;
        }
        let d = self.port_d_m[closed as usize];
        let mu = crate::combustion::viscosity(t_k);
        let k = 0.0263 + 7.6e-5 * (t_k - 300.0);
        let re = 4.0 * m_kg_s / (PI * d * mu);
        let h = self.h_mult * 0.023 * re.powf(0.8) * 0.871 * k / d; // Pr^0.4 = 0.71^0.4
        1.0 - (-h * PI * d * self.port_len_m / (m_kg_s * 1005.0)).exp()
    }
}

/// Physically motivated cylinder-to-cylinder spread of a real engine, deterministic from `seed` (reproducible runs):
/// intake-runner flow distribution, injector static-flow tolerance, ring-pack leakage, deposit thickness, and the coolant
/// flow order along the block jacket. Spreads grow with mileage (valve/injector deposits, ring wear). All magnitudes
/// UNVERIFIED engineering values with the anchors given at [`CylinderSpread::new`].
#[derive(Debug, Clone, Serialize)]
pub struct CylinderSpread {
    /// Intake-port Cd·A factor per cylinder (manifold runner distribution, intake-valve deposits).
    pub port_flow: [f64; 4],
    /// DI and MPI injector static-flow factors.
    pub di_flow: [f64; 4],
    pub mpi_flow: [f64; 4],
    /// Ring-pack leakage factor and deposit-thickness factor.
    pub ring_leak: [f64; 4],
    pub deposit: [f64; 4],
    /// Block-jacket position where the coolant enters, cylinder pitches (0 = cylinder 1 end, 3 = cylinder 4 end); the
    /// jacket warms up along the block by the wall heat over ṁ·cp. UNVERIFIED for EA888 Gen3 (pump in the ITM module).
    pub coolant_inlet_pos: f64,
    pub seed: u64,
}

impl CylinderSpread {
    /// No spread (idealised identical cylinders).
    pub fn identical() -> Self {
        Self { port_flow: [1.0; 4], di_flow: [1.0; 4], mpi_flow: [1.0; 4], ring_leak: [1.0; 4], deposit: [1.0; 4], coolant_inlet_pos: 0.0, seed: 0 }
    }

    /// Deterministic spread for `seed` at `engine_km`. 1σ values: runner flow 1 % new + 1 %/100 000 km (Heywood 1988
    /// §7.6.2: cylinder-to-cylinder air-flow differences of a few percent in production manifolds); injector static flow
    /// 1.5 % (±3 % tolerance class of GDI/MPI injectors) + 1 %/100 000 km (tip coking); ring leakage 10 % + 20 %/100 000 km;
    /// deposits 20 % + 10 %/100 000 km. Coolant enters at the cylinder-1 end (UNVERIFIED).
    pub fn new(seed: u64, engine_km: f64) -> Self {
        let mut st = seed ^ 0x9E37_79B9_7F4A_7C15;
        let mut gauss = || {
            // Irwin–Hall(12) − 6: unit-variance, bounded ±6σ, allocation-free and platform independent
            let mut s = 0.0;
            for _ in 0..12 {
                st = st.wrapping_add(0x9E37_79B9_7F4A_7C15);
                let mut z = st;
                z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
                z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
                s += ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64;
            }
            s - 6.0
        };
        let k = engine_km.max(0.0) / 1e5;
        let mut draw = |sigma: f64| -> [f64; 4] { std::array::from_fn(|_| (1.0 + sigma * gauss()).max(0.5)) };
        Self {
            port_flow: draw(0.01 + 0.01 * k),
            di_flow: draw(0.015 + 0.01 * k),
            mpi_flow: draw(0.015 + 0.01 * k),
            ring_leak: draw(0.10 + 0.20 * k),
            deposit: draw(0.20 + 0.10 * k),
            coolant_inlet_pos: 0.0,
            seed,
        }
    }
}

impl Mechanics {
    /// Standard (large) exhaust lobe of an EA888 Gen3 with exhaust AVS: lift `l_m`, opening period at the reference lift
    /// (`ip_cam_op_vcp[1]`), closing at −24 °CRK (`c_cam_ind_eng[1]`) = 336 °CA after the firing TDC — see [`AvsHw`].
    pub fn with_exhaust_lobe(mut self, l_m: f64, period_deg: f64) -> Self {
        let n = self.exhaust.shape_n;
        (self.exhaust.max_lift_m, self.exhaust.duration_deg, self.exhaust.centerline_deg) = lobe_from_period(l_m, period_deg, 336.0, n);
        self
    }

    /// 2.0 (CJXC) long block: standard exhaust lobe 10 mm / 215 ° (SC8S50 259J, [`AvsHw`]).
    pub fn ea888_gen3(rod_length_m: f64) -> Self {
        Self::ea888_gen3_base(rod_length_m).with_exhaust_lobe(10.0e-3, 215.0)
    }

    fn ea888_gen3_base(rod_length_m: f64) -> Self {
        Self {
            rod_length_m,
            pin_offset_m: 0.0005,
            piston_mass_kg: 0.39,
            rod_mass_kg: 0.53,
            rod_small_end_share: 0.33,
            dmf_secondary_kgm2: 0.07,
            dmf_rate_nm_rad: 300.0,
            dmf_damping_nms: 1.5,
            dmf_travel_rad: 1.0,
            intake: ValveGear {
                count: 2.0,
                head_diameter_m: 0.03385,
                stem_diameter_m: 0.006,
                max_lift_m: 0.0107,
                duration_deg: 250.0,
                shape_n: 3.0,
                centerline_deg: 485.0,
                phaser_range_deg: 60.0,
                phaser_dir: -1.0,
                cd: 0.62,
                moving_mass_kg: 0.085,
                spring_preload_n: 250.0,
                spring_rate_n_m: 46_000.0,
                damping_ns_m: 6.0,
                inclination_deg: 20.5,
                phaser: PhaserHw { hold_ofs: -0.07, hold_adapt: -0.034, ..PhaserHw::ea888_gen3() },
            },
            exhaust: ValveGear {
                count: 2.0,
                head_diameter_m: 0.0280,
                stem_diameter_m: 0.006,
                max_lift_m: 0.0100,
                duration_deg: 240.0,
                shape_n: 2.0,
                centerline_deg: 245.0,
                phaser_range_deg: 30.0,
                phaser_dir: 1.0,
                cd: 0.65,
                moving_mass_kg: 0.078,
                spring_preload_n: 250.0,
                spring_rate_n_m: 46_000.0,
                damping_ns_m: 6.0,
                inclination_deg: 20.0,
                phaser: PhaserHw { hold_adapt: -0.047, ..PhaserHw::ea888_gen3() },
            },
            avs: Some(AvsHw::ea888_gen3()),
            valve_piston_min_clearance_m: 0.0015,
            main_bearing: JournalBearing { diameter_m: 0.048, width_m: 0.017, clearance_m: 20e-6 },
            rod_bearing: JournalBearing { diameter_m: 0.0478, width_m: 0.016, clearance_m: 22e-6 },
            bearing_roughness_m: 0.30e-6,
            oil_pump_m3_rev: 20e-6,
            oil_capacity_l: 4.6,
            ring_leak_cda_m2: 2.0e-7,
            rod_shank_area_m2: 190e-6,
            rod_gyration_radius_m: 0.0055,
            rod_fibre_m: 0.007,
            fire_ring_clamp_n: 105e3,
            fire_ring_diameter_m: 0.0865,
            piston_thermal: PistonThermalHw::ea888_gen3(),
            exhaust_valve_thermal: ValveThermalHw::ea888_gen3_exhaust(),
            ring_land_stress_factor: 3.0,
            spark_gap_m: 0.0007,
            spark_plug: SparkPlugHw::ea888_oe(),
            deposit_m: 0.0,
            deposit: DepositHw::ccd(),
            crevice_m3: 0.4e-6,
            combustion: CombustionHw::ea888_gen3(),
            intake_heat: IntakeHeatHw::ea888_gen3(),
            cyl_spread: CylinderSpread::new(0xEA88_8000, 0.0),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct DrivetrainProfile {
    pub id: DrivetrainPresetId,
    pub slug: &'static str,
    pub name: &'static str,
    pub tcu_code: &'static str,
    /// A TCU firmware of this gearbox exists in the project (DQ200_G2, DQ250 0D9); other gearboxes cannot be actuated.
    pub has_tcu_firmware: bool,
    pub awd: bool,
    pub gears: usize,
    /// Gear ratios 1..7 (unused entries 0). Must ultimately be read from the TCU calibration.
    pub gear_ratios: [f64; 7],
    /// Final drive used by each forward gear (two output shafts on VAG DSGs).
    pub final_drives: [f64; 7],
    pub reverse_ratio: f64,
    pub reverse_final: f64,
    /// Fork → (input shaft, gear on the + side, gear on the − side); 0 = no gear, 0xFF = reverse.
    /// Fork position-sensor sign UNVERIFIED.
    pub forks: [(usize, u8, u8); 4],
    /// Input shaft of the − side gear when it differs from the fork's (DQ250: the 6/R sleeve, 6th on input shaft 2,
    /// reverse driven from input shaft 1); `None` = the fork's shaft.
    pub fork_minus_shaft: [Option<usize>; 4],
    /// Sleeve travel: blocker-ring contact, dog-chamfer contact, torque-carrying engagement,
    /// detent at full engagement, end stop (mm).
    pub fork_sync_mm: f64,
    pub fork_dog_mm: f64,
    pub fork_engaged_mm: f64,
    pub fork_full_mm: f64,
    pub fork_travel_mm: f64,
    /// End stop of a fork side without a gear, mm: the blocker-ring position (DQ200), or a neutral detent as far out as
    /// the engaged positions (DQ250 5/N actuator: the firmware classifies its N side with the same travel thresholds as
    /// the gears, ±6.50 mm, 0D9300011R tables 0x800DBA0E / 0x8009CBB8 — VERIFIED-S; the travel itself HYP).
    pub free_side_mm: f64,
    pub clutch: ClutchHw,
    pub mechatronic: MechatronicHw,
    pub train: GearTrainHw,
    pub tyre: TyreHw,
    pub brakes: BrakeHw,
    pub body: BodyHw,
    /// AWD coupling hardware (its controller is a separate ECU).
    pub haldex: Option<HaldexHw>,
}

impl DrivetrainProfile {
    /// Signed gear ratio of the gear pair (reverse negative).
    pub fn gear_ratio(&self, gear: u8) -> f64 {
        match gear {
            0xFF => -self.reverse_ratio,
            g if g >= 1 && (g as usize) <= self.gears.min(7) => self.gear_ratios[g as usize - 1],
            _ => 0.0,
        }
    }

    /// Total ratio engine → wheel for forward gear 1..=gears, or reverse (`0xFF`).
    pub fn total_ratio(&self, gear: u8) -> f64 {
        match gear {
            0xFF => -self.reverse_ratio * self.reverse_final,
            g if g >= 1 && (g as usize) <= self.gears.min(7) => self.gear_ratios[g as usize - 1] * self.final_drives[g as usize - 1],
            _ => 0.0,
        }
    }

    /// (fork, side ±1, input shaft) carrying gear `g`.
    pub fn gear_fork(&self, g: u8) -> Option<(usize, f64, usize)> {
        if g == 0 {
            return None;
        }
        self.forks.iter().enumerate().find_map(|(f, &(s, plus, minus))| {
            if plus == g {
                Some((f, 1.0, s))
            } else if minus == g {
                Some((f, -1.0, self.fork_minus_shaft[f].unwrap_or(s)))
            } else {
                None
            }
        })
    }

    fn octavia_body(mass_kg: f64) -> BodyHw {
        BodyHw {
            mass_kg,
            drag_coefficient: 0.29,
            lift_coefficient: [0.04, 0.06],
            frontal_area_m2: 2.22,
            wheelbase_m: 2.686,
            cg_height_m: 0.55,
            front_axle_share: 0.61,
            pitch_inertia_kgm2: 1.15 * mass_kg,
            pitch_freq_hz: 1.3,
            pitch_damping_ratio: 0.3,
        }
    }

    pub fn dq200_fwd() -> Self {
        Self {
            id: DrivetrainPresetId::Dq200Fwd7Speed,
            slug: "dq200_fwd",
            name: "DSG DQ200 (0CW) 7-speed FWD, dry clutches",
            tcu_code: "0CW300043H",
            has_tcu_firmware: true,
            awd: false,
            gears: 7,
            // 0AM/0CW ratios for 1.4/1.8 TSI (workshop-manual tables). UNVERIFIED against the TCU calibration.
            gear_ratios: [3.765, 2.273, 1.531, 1.122, 1.176, 0.951, 0.795],
            final_drives: [4.438, 4.438, 4.438, 4.438, 3.227, 3.227, 3.227],
            reverse_ratio: 4.551,
            reverse_final: 3.227,
            // Fork order and sides as the DQ200_G2 firmware sees them (gear ids of the adaptation pairs
            // 0x801384E0 (+) / 0x801384DC (-): 1/3, 2/4, 5/7, R/6, VERIFIED-S); positive travel = the
            // fork sensor's positive position (`simemu_tc1791::tcu_dq200::ForkSensors`).
            forks: [(0, 1, 3), (1, 2, 4), (0, 5, 7), (1, 0xFF, 6)],
            fork_minus_shaft: [None; 4],
            // Travel anchored to the reference mechatronic adaptations (EEPROM -> RAM 0xD0003E78,
            // VERIFIED-T): synchroniser points 2.2..3.4 mm, engaged end positions 8.3..9.5 mm. The
            // split between sync contact, dog contact and torque-carrying engagement is UNVERIFIED.
            fork_sync_mm: 2.8,
            fork_dog_mm: 4.3,
            fork_engaged_mm: 6.5,
            fork_full_mm: 8.8,
            fork_travel_mm: 9.8,
            free_side_mm: 2.8,
            clutch: ClutchHw::Dry(DryClutchHw::dq200()),
            mechatronic: MechatronicHw::Dq200(Dq200HydraulicsHw::dq200()),
            train: GearTrainHw::dq200(),
            tyre: TyreHw::octavia(),
            brakes: BrakeHw::octavia(),
            body: Self::octavia_body(1385.0),
            haldex: None,
        }
    }

    /// DQ250 02E: K1 drives 1/3/5/R, K2 drives 2/4/6. 1st 45:13 = 3.462 and final 70:17 = 4.118
    /// appear in workshop-manual tables; the other ratios are the Golf V GTI set, UNVERIFIED.
    pub fn dq250_4wd() -> Self {
        Self {
            id: DrivetrainPresetId::Dq250Awd6Speed,
            slug: "dq250_4wd",
            name: "DSG DQ250 (02E) 6-speed 4WD, wet clutches",
            tcu_code: "0D9300012",
            has_tcu_firmware: false,
            awd: true,
            gears: 6,
            gear_ratios: [3.462, 2.050, 1.300, 0.902, 0.914, 0.756, 0.0],
            final_drives: [4.118, 4.118, 4.118, 4.118, 3.043, 3.043, 0.0],
            reverse_ratio: 3.987,
            reverse_final: 4.118,
            // Gear actuators as the DQ250 firmware drives and reads them (0D9300011R tables 0x800DBA0A/0E, travel sign
            // from its position classes, VERIFIED-S; docs/DQ250_IO_MAP.md §4): 1/3 and 5/N on sub-gearbox 1, 4/2 on
            // sub-gearbox 2, 6/R with 6th on input shaft 2 and reverse driven from input shaft 1.
            forks: [(0, 1, 3), (1, 4, 2), (0, 5, 0), (1, 6, 0xFF)],
            fork_minus_shaft: [None, None, None, Some(0)],
            // 5/N actuator: no travel past neutral on its N side. The firmware's synchronisation-point adaptation
            // (basic setting 0x04D4, 0D9300012F 0x8005B814 state 8) accepts actuator 2 side 1 as measured when it has
            // NOT moved (± 250 position units ≈ 0.9 mm) and requires the neutral averaged over both sides within its
            // window (state 9, CAL 0x800383F0): with a stop 8.8 mm out (former HYP) it aborted with code 9 (VERIFIED-T,
            // docs/DQ250_IO_MAP.md §8). The 0.5 mm itself is UNVERIFIED.
            free_side_mm: 0.5,
            clutch: ClutchHw::Wet([WetClutchHw::dq250_k1(), WetClutchHw::dq250_k2()]),
            mechatronic: MechatronicHw::Wet(WetHydraulicsHw::dq250()),
            train: GearTrainHw::dq250(),
            body: Self::octavia_body(1485.0),
            haldex: Some(HaldexHw::gen5()),
            ..Self::dq200_fwd()
        }
    }

    /// Reference MQB DQ250 (0D9, TCU 0D9300012F) + Haldex 5 rear axle from an Audi A3 8V quattro in an
    /// Octavia A7 body. Gear set = the TCU calibration (0D9300012F/4930 tooth table 0x80034638, read by the
    /// ratio code at 0x8008B380 the same way 0D9300011R reads 0x80030600 at 0x8009D0C8; VERIFIED-S,
    /// `simemu_tc1791::tcu_dq250::gear_set`): gears 45/13, 41/20, 39/30, 37/41, 32/35, 31/41 (the 02E set), finals
    /// 70/16 (1–4) and 70/21 (5, 6, R), reverse 33·22/(13·14); dynamic tyre radius ≈ 0.310 m (225/45 R17,
    /// UNVERIFIED). Mass: MQB 1.8/2.0 TSI DSG + AWD rear axle, propshaft and Haldex + driver ≈ 1485 kg — UNVERIFIED.
    pub fn dq250_0d9_awd() -> Self {
        Self {
            slug: "dq250_0d9_awd",
            name: "DSG DQ250 (0D9 MQB) 6-speed AWD (Haldex 5), wet clutches",
            tcu_code: "0D9300012F",
            // DQ250 0D9 firmware: 0D9300011R / 0D9300012F (simemu_tc1791::tcu_dq250).
            has_tcu_firmware: true,
            gear_ratios: [45.0 / 13.0, 41.0 / 20.0, 39.0 / 30.0, 37.0 / 41.0, 32.0 / 35.0, 31.0 / 41.0, 0.0],
            final_drives: [70.0 / 16.0, 70.0 / 16.0, 70.0 / 16.0, 70.0 / 16.0, 70.0 / 21.0, 70.0 / 21.0, 0.0],
            reverse_ratio: 33.0 * 22.0 / (13.0 * 14.0),
            reverse_final: 70.0 / 21.0,
            tyre: TyreHw { radius_m: 0.310, ..TyreHw::octavia() },
            ..Self::dq250_4wd()
        }
    }

    /// Gear set of a DQ250 TCU calibration (`simemu_tc1791::tcu_dq250::gear_set`): forward ratios and finals of gears
    /// 1..6, reverse ratio and final — the gearbox hardware that calibration is made for.
    pub fn with_gear_set(mut self, ratios: [f64; 6], finals: [f64; 6], reverse: (f64, f64)) -> Self {
        self.gear_ratios[..6].copy_from_slice(&ratios);
        self.final_drives[..6].copy_from_slice(&finals);
        (self.reverse_ratio, self.reverse_final) = reverse;
        self
    }

    /// DQ381 0GC. Ratios from a community gear-ratio calculator (Google Sheets "MK7 gear ratio
    /// calculator", DQ381 column) — UNVERIFIED; the assignment of the two final drives to gears is
    /// inferred from monotonically decreasing overall ratios.
    pub fn dq381_4wd() -> Self {
        Self {
            id: DrivetrainPresetId::Dq381Awd7Speed,
            slug: "dq381_4wd",
            name: "DSG DQ381 (0GC) 7-speed 4WD, wet clutches",
            tcu_code: "0GC300012",
            has_tcu_firmware: false,
            awd: true,
            gears: 7,
            gear_ratios: [3.400, 2.750, 1.770, 0.930, 0.710, 0.760, 0.640],
            final_drives: [4.170, 3.130, 3.130, 4.170, 4.170, 3.130, 3.130],
            reverse_ratio: 2.900,
            reverse_final: 4.170,
            forks: [(0, 1, 3), (1, 2, 4), (0, 5, 7), (1, 6, 0xFF)],
            clutch: ClutchHw::Wet([WetClutchHw::dq381_k1(), WetClutchHw::dq381_k2()]),
            mechatronic: MechatronicHw::Wet(WetHydraulicsHw::dq381()),
            train: GearTrainHw::dq381(),
            body: Self::octavia_body(1510.0),
            haldex: Some(HaldexHw::gen5()),
            ..Self::dq200_fwd()
        }
    }

    pub fn from_slug(slug: &str) -> Option<Self> {
        match slug.to_lowercase().as_str() {
            "fwd" | "dq200" | "dq200_fwd" | "cjsa" => Some(Self::dq200_fwd()),
            "4wd" | "awd" | "dq250" | "dq250_4wd" => Some(Self::dq250_4wd()),
            "dq250_0d9_awd" | "0d9" => Some(Self::dq250_0d9_awd()),
            "dq381" | "dq381_4wd" => Some(Self::dq381_4wd()),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn displacement_matches_spec() {
        let e = EngineProfile::ea888_18t_cjsa_is12();
        assert!((e.displacement_m3() * 1e6 - 1798.3).abs() < 1.0);
        let e2 = EngineProfile::ea888_20t_cjxc_is38();
        assert!((e2.displacement_m3() * 1e6 - 1984.3).abs() < 1.0);
    }

    #[test]
    fn dq200_overall_ratios_strictly_decrease() {
        let d = DrivetrainProfile::dq200_fwd();
        for g in 1..d.gears as u8 {
            assert!(d.total_ratio(g) > d.total_ratio(g + 1), "gear {g}");
        }
        assert!(d.total_ratio(0xFF) < 0.0);
    }
}
