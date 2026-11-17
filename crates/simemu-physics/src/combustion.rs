//! Quasi-dimensional SI combustion sub-models (allocation-free, evaluated every crank step of `cylinder`).
//!
//! * **Laminar flame speed** — Metghalchi & Keck, "Burning velocities of mixtures of air with methanol, isooctane
//!   and indolene at high pressure and temperature", Combust. Flame 48 (1982) 191–210 (RMFD-303 "indolene"
//!   coefficients), residual-gas dilution after Rhodes & Keck, SAE 850047 (1985): S_L = S_L0·(T/298)^α·(p/1 atm)^β·
//!   (1 − 2.06·x_r^0.77). Ethanol blends: +20 % at neat ethanol (Gülder, Combust. Flame 56, 1984; linear in mass
//!   fraction — UNVERIFIED blending rule).
//! * **Turbulence** — 0-D mean-flow / tumble / turbulence (K–T–k) cascade after Poulos & Heywood, SAE 830334 (1983)
//!   (mean flow K fed by the intake jet, dissipated into k), Morel & Keribar, SAE 850204 (1985) (compression
//!   amplification, length scale tied to the chamber height) and the tumble angular-momentum equation of Grasreiner
//!   et al., IJER 15 (2014) 805 / Dulbecco et al., SAE 2009-01-0368 (spin-up by the shrinking moment of inertia of the
//!   charge, breakdown into turbulence as the chamber flattens towards TDC).
//! * **Burn rate** — entrainment ("eddy burning") model of Blizard & Keck, SAE 740191 (1974) / Tabaczynski et al.,
//!   SAE 770647 (1977): dm_e/dt = ρ_u·A_f·S_T, dm_b/dt = (m_e − m_b)/τ_b, τ_b = λ_T/S_L (Taylor microscale);
//!   turbulent burning velocity after Peters, "The turbulent burning velocity for large-scale and small-scale
//!   turbulence", J. Fluid Mech. 384 (1999) 107 (corrugated / thin-reaction-zone regimes in one formula: S_T → S_L + 2u'
//!   for large scales, Damköhler's ∝ √(u'·L/S_L·l_F) for small ones), applied to the growing kernel with the development
//!   factor of Herweg & Maly, SAE 922243 (1992): √(1 − e^(−r_f/L)). The flame is a sphere centred on
//!   the plug, truncated by the head, piston and liner of the equivalent flat chamber (closed-form volume and area).
//! * **Kinetics** — thermal NO by the extended Zeldovich mechanism (Heywood, "Internal Combustion Engine
//!   Fundamentals", McGraw-Hill 1988, §11.2, eq. 11.8 with the rate constants of Table 11.1); rich-mixture CO/H₂ by
//!   the water-gas-shift equilibrium frozen at K = 3.5 (Heywood §4.9.2); exhaust-manifold afterburning of unburned
//!   fuel by the global single-step rate of Westbrook & Dryer, Combust. Sci. Technol. 27 (1981) 31 (n-octane).
//!
//! Every fitted constant is a field of [`CombustionHw`] (profile data) with its anchor stated there.

use serde::Serialize;
use std::f64::consts::PI;

/// Combustion-model constants of an engine (profile data). The literature defaults are the published model values;
/// the ones marked FITTED were fitted to the firmware's own maps.
///
/// Joint identification (docs/SIMOS18_COMBUSTION.md §3.3, `bench_tests::explore_combustion_grid`, 10 points 760–5500 rpm,
/// flaps open/closed, the exhaust AVS lobe the firmware selects there): `tumble_port`, `tumble_decay`, `burnup_c` and
/// `kernel_ck` together against the reference ignition `ip_iga_ref_ivvt_vvl_port_h/l[vvl]` (MBT, flap effect) and the
/// ignition-angle efficiency `ip_eff_iga_vvl_cam_h/l` (SC8S50 259J @0x8085A76C / @0x8085A8EC; the efficiency at a retard
/// fixes how much the burn slows when it moves into the expansion stroke).
#[derive(Debug, Clone, Serialize)]
pub struct CombustionHw {
    /// Multiplier on the turbulent part of Peters' burning velocity: 1 = Peters (1999) unchanged — chosen over Zimont
    /// (A = 0.52), whose u'^¾ scaling left the plant's MBT 5–10 ° behind the firmware's reference map at 3000–4500 rpm.
    pub flame_c: f64,
    /// Flame-kernel development: multiplier on r_f/L in f = √(1 − e^(−C·r_f/L)) (Herweg & Maly 1992: C = 1; their
    /// time factor √(1 − e^(−t·u'/L)) is left out — with it the late-spark burn slowed far beyond the firmware's
    /// ignition-efficiency curve `ip_eff_iga`). FITTED 0.8 (joint identification).
    pub kernel_ck: f64,
    /// Burn-up time multiplier on τ_b = λ_T/S_L (Tabaczynski 1977: 1). FITTED 0.6 (joint identification: the burn-up time
    /// is the main fixed-time part of the burn, it sets how much later the burn sits in crank degrees at high speed).
    pub burnup_c: f64,
    /// Intake-jet tumble coefficient: tumble angular momentum per (kg · m/s · bore radius) of inflow, flaps open /
    /// flaps closed (EA888 Gen3 tumble port + N316 flaps, SSP 606). FITTED: the closed/open ratio to the firmware's flap
    /// effect on the reference ignition (closed flaps 4.1–4.5 ° later at 1500–2500 rpm, `_h` vs `_l`), the level to MBT.
    pub tumble_port: [f64; 2],
    /// Mean-flow → turbulence dissipation constant c_β (Poulos & Heywood 1983: 0.3307·c_β with c_β ≈ 1).
    pub c_beta: f64,
    /// Tumble breakdown constant (rate = C·ω_T·(1 − h/h₀)² once the chamber is flatter than h₀ = `tumble_h0`·bore). FITTED
    /// 0.55 (joint identification). At 1.0 the vortex broke down within ≈ 60 ° before TDC: u'/S̄p peaked at 1.8 (flaps
    /// closed) 30 ° BTDC and fell to 0.45 at 30 ° ATDC, so a retarded burn ran in decaying turbulence and the efficiency
    /// fell 0.07–0.14 below `ip_eff_iga` at −30 °. A slower breakdown spreads it over TDC (TKE peak ≈ 20–0 ° BTDC in the
    /// 0-D/3-D tumble studies of Achuth & Mehta, IJER 2 (2001) 209 and Dulbecco et al., SAE 2009-01-0368).
    pub tumble_decay: f64,
    /// Chamber height / bore below which the tumble vortex is squeezed and breaks down. UNVERIFIED.
    pub tumble_h0: f64,
    /// Integral length scale over the instantaneous chamber height (Morel & Keribar 1985: L ≈ 0.2–0.3·h near TDC).
    pub length_scale: f64,
    /// Spark-plug electrode depth below the flat equivalent fire deck, m (EA888 central plug in the pent roof).
    pub plug_depth_m: f64,
    /// Share of the coil's secondary energy deposited in the gas (breakdown + arc + glow; Maly & Vogel, 17th Symp.
    /// (Int.) Combust. 1979: glow ≈ 30 %). UNVERIFIED.
    pub spark_eta: f64,
    /// Kernel quench: Karlovitz number above which the kernel is blown out (Abdel-Gayed & Bradley, Combust. Flame
    /// 62 (1985) 61: Ka ≈ 1.5 total-quench boundary).
    pub ka_quench: f64,
    /// Douaud–Eyzat end-gas delay multiplier (Douaud & Eyzat, SAE 780080, 1978: `τ = 17.68 ms·(ON/100)^3.402·p[atm]^−1.7·exp(3800/T)`).
    /// 1.0 = the correlation as published; checked: the plant's knock-limited spark (0.5 bar
    /// first mode, RON 95) sits −1.6…+4.5 ° above SC8O40's base ignition `ip_iga_bas_ivvt_vvl_port_l` at 1500–6000 rpm.
    pub knock_tau_scale: f64,
    /// Fraction of the constant-volume pressure jump of the autoigniting end gas that rings as the first chamber mode
    /// in ordinary knock (small end-gas mass). UNVERIFIED (gives 1–3 bar light, 10–40 bar heavy knock).
    pub knock_amp_coef: f64,
    /// Hot-surface ignition: delay multiplier on the Douaud–Eyzat delay evaluated at the surface temperature (the
    /// ignitable boundary layer is thin and loses heat to the bulk). FITTED so that surface ignition sets in at
    /// ≈ 900–950 °C surface temperature at full-load compression (NGK/Bosch: pre-ignition above ≈ 850–950 °C).
    pub hot_surface_c: f64,
    /// LSPI: oil/fuel droplet release probability per cycle at the reference state (no fuel in the oil, no liner
    /// wetting, new rings, clean chamber). UNVERIFIED (LSPI-prone engines: 10⁻⁴…10⁻³ events per cycle at 1500–2000 rpm
    /// and > 18 bar BMEP — Amann et al., SAE 2011-01-0339; Zahdeh et al., SAE 2011-01-0340).
    pub lspi_rate: f64,
    /// LSPI droplet diameter, m, and its d²-law evaporation constant at 700 K, m²/s. UNVERIFIED.
    pub lspi_drop_m: f64,
    pub lspi_evap_k: f64,
    /// Effective octane number of the evaporated oil/fuel droplet cloud (lube oil base stock + Ca detergent lower the
    /// ignition resistance — Takeuchi et al., SAE 2012-01-1615; Dahnz & Spicher, IJER 11 (2010) 485). UNVERIFIED.
    pub lspi_octane: f64,
    /// Cycle-to-cycle variation (CCV) at the spark plug: relative standard deviation of the turbulence intensity the
    /// kernel meets and of the local equivalence ratio, drawn once per cycle. The flow and mixture near the plug at
    /// ignition are the leading causes of CCV and act through the early flame development (Heywood 1988 §9.4.2; Ozdor,
    /// Dulger & Sher, SAE 940987; Galloni, Energy Convers. Manag. 50 (2009) 3209); the deviation fades as the front
    /// outgrows the integral scale L (a front of radius r averages ≈ (r/L)² independent eddies → weight L/(L + r)).
    /// UNVERIFIED magnitudes (in-cylinder LDV/PIV: cyclic velocity fluctuations at the plug of the order of the turbulence
    /// intensity itself); FITTED to the CoV of IMEP of warm SI engines (≈ 1–3 % part load, higher at idle and lean).
    pub ccv_turb_sigma: f64,
    pub ccv_phi_sigma: f64,
}

impl CombustionHw {
    pub fn ea888_gen3() -> Self {
        Self {
            flame_c: 1.0,
            kernel_ck: 0.8,
            burnup_c: 0.6,
            tumble_port: [0.20, 0.40],
            c_beta: 1.0,
            tumble_decay: 0.55,
            tumble_h0: 0.6,
            length_scale: 0.25,
            plug_depth_m: 0.004,
            spark_eta: 0.3,
            ka_quench: 4.0,
            knock_tau_scale: 1.0,
            knock_amp_coef: 0.15,
            hot_surface_c: 1.7,
            lspi_rate: 2e-4,
            lspi_drop_m: 40e-6,
            lspi_evap_k: 0.5e-6,
            lspi_octane: 40.0,
            ccv_turb_sigma: 0.5,
            ccv_phi_sigma: 0.04,
        }
    }
}

/// Gasoline as three pseudo-components for the evaporation of liquid fuel films (light / mid / heavy): mass fraction,
/// normal boiling point K, effective enthalpy of vaporisation J/mol, molar mass kg/mol. Light and mid = n-pentane and
/// iso-octane in the mass shares of the 3-component surrogate "E00" (n-pentane / iso-octane / n-undecane 36/46/18 vol %,
/// ≈ 33/47/20 wt %; Da Silva Cruz Lis et al., ICLASS 2021, table 2); heavy = 1,2,4-trimethylbenzene, the heavy aromatic
/// lump (19 wt %) of the surrogate fitted to a regular-gasoline distillation curve (Kobashi, Zama, Kuboyama, IJHMT 147
/// (2020) 119035) — a C9 aromatic boiling at 169 °C stands for the E150+ fraction better than n-undecane (196 °C, near
/// the final boiling point). ΔH: Clausius–Clapeyron through the normal boiling point and the 25 °C vapour pressure (NIST
/// WebBook: n-pentane 68.3 kPa, iso-octane 6.58 kPa, 1,2,4-TMB 0.28 kPa). Raoult vapour pressure of the blend ≈ 52 kPa at
/// 37.8 °C (EN 228 summer class A 45–60 kPa). Not a specific pump fuel.
pub const FUEL_PSEUDO: [(f64, f64, f64, f64); 3] = [(0.33, 309.2, 26.9e3, 72.15e-3), (0.48, 372.4, 33.9e3, 114.2e-3), (0.19, 442.4, 44.7e3, 120.2e-3)];

/// Pseudo-component mass fractions of the injected fuel at ethanol mass fraction `w_eth` (ethanol, boiling at 351 K, is
/// counted with the mid component).
#[inline]
pub fn fuel_components(w_eth: f64) -> [f64; 3] {
    let e = w_eth.clamp(0.0, 1.0);
    [(1.0 - e) * FUEL_PSEUDO[0].0, (1.0 - e) * FUEL_PSEUDO[1].0 + e, (1.0 - e) * FUEL_PSEUDO[2].0]
}

/// Saturation pressure of pseudo-component `c` at `t_k`, Pa.
#[inline]
pub fn fuel_psat(c: usize, t_k: f64) -> f64 {
    let (_, tb, dh, _) = FUEL_PSEUDO[c];
    101_325.0 * (dh / 8.314 * (1.0 / tb - 1.0 / t_k)).exp()
}

/// Evaporates a multi-component liquid fuel film (`m`, kg per pseudo-component, wall surface at `t_k`, gas at `p_pa`)
/// over `dt`; returns the evaporated mass per component. Three ways out, rates per unit film mass:
/// * diffusion through the gas: `k_diff`·ln(1 + B_c), Spalding transfer number B_c = p_sat,c/(p − p_sat,c) (capped at
///   100), each component on its own (Raoult with the wetted area ∝ the film's moles: the light ends go first, the heavy
///   ends stay — the slow heavy tail of gasoline films, Kobashi et al. 2020; Dhanji et al., ILASS-Europe 2023; Da Silva
///   Cruz Lis et al. 2021), and slower under pressure;
/// * boiling of a component whose boiling point at the gas pressure the wall exceeds (p_sat,c > p): the wall's heat
///   boils it off at the former single-component hot-wall rate 1/τ, τ = 5 ms·e^((420 − T)/30) (UNVERIFIED, kept: a warm
///   crown, above every component's boiling point, behaves as before; Habchi 1999: no film on a piston above ≈ 460 K);
/// * `k_heat`: the gas heat flux into the film (combustion: "pool fire") evaporating it as a whole.
pub fn film_evaporate(m: &mut [f64; 3], t_k: f64, p_pa: f64, k_diff: f64, k_heat: f64, dt: f64) -> [f64; 3] {
    if m.iter().all(|&x| x <= 0.0) {
        return [0.0; 3];
    }
    let boil = 1.0 / (5e-3 * ((420.0 - t_k) / 30.0).exp()).max(1e-4);
    std::array::from_fn(|c| {
        if m[c] <= 0.0 {
            return 0.0;
        }
        let ps = fuel_psat(c, t_k);
        let b = ps / (p_pa - ps).max(0.01 * p_pa);
        let k = k_diff * b.ln_1p() + if ps > p_pa { boil } else { 0.0 } + k_heat;
        let dm = m[c] * (1.0 - (-dt * k).exp());
        m[c] -= dm;
        if m[c] < 1e-24 {
            let rem = m[c];
            m[c] = 0.0;
            dm + rem
        } else {
            dm
        }
    })
}

/// Gasoline / ethanol blend by ethanol mass fraction `w`: (stoichiometric AFR, LHV J/kg, heat of vaporisation J/kg).
/// Gasoline = the plant's constants (`cylinder`); ethanol 9.0, 26.95 MJ/kg, 921 kJ/kg (DOE fuel properties table,
/// as `bench_config::fuel_props`); mass-weighted.
#[inline]
pub fn fuel_blend(w: f64) -> (f64, f64, f64) {
    let w = w.clamp(0.0, 1.0);
    let mix = |g: f64, e: f64| (1.0 - w) * g + w * e;
    (mix(crate::cylinder::AFR_STOICH, 9.0), mix(crate::cylinder::LHV_J_KG, 26.95e6), mix(350e3, 921e3))
}

/// Douaud–Eyzat autoignition delay, s, at octane number `on`, pressure `p` Pa, temperature `t` K.
#[inline]
pub fn autoignition_delay(on: f64, p: f64, t: f64) -> f64 {
    17.68e-3 * (on.max(1.0) / 100.0).powf(3.402) * (p / 101_325.0).max(1e-3).powf(-1.7) * (3800.0 / t).exp()
}

/// Deterministic per-cylinder random numbers (SplitMix64), uniform in [0, 1).
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct Rng(pub u64);

impl Rng {
    #[inline]
    pub fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Two independent standard normal deviates (Box–Muller), each limited to ±3σ.
    #[inline]
    pub fn normal_pair(&mut self) -> (f64, f64) {
        let (u1, u2) = (self.uniform().max(1e-300), self.uniform());
        let r = (-2.0 * u1.ln()).sqrt();
        let a = 2.0 * PI * u2;
        ((r * a.cos()).clamp(-3.0, 3.0), (r * a.sin()).clamp(-3.0, 3.0))
    }
}

// ---- laminar flame ------------------------------------------------------------------------------------------

/// Laminar burning velocity, m/s, of gasoline (RMFD-303) / ethanol blend–air at equivalence ratio `phi`, unburned
/// temperature `t_u` K, pressure `p` Pa, residual (burned-gas) mass fraction `x_r`, ethanol mass fraction `w_eth`.
#[inline]
pub fn laminar_flame_speed(phi: f64, t_u: f64, p: f64, x_r: f64, w_eth: f64) -> f64 {
    let phi = phi.clamp(0.3, 2.5);
    let sl0 = 0.2758 - 0.7834 * (phi - 1.13).powi(2);
    if sl0 <= 0.0 {
        return 0.0;
    }
    let alpha = 2.18 - 0.8 * (phi - 1.0);
    let beta = -0.16 + 0.22 * (phi - 1.0);
    let dil = (1.0 - 2.06 * x_r.max(0.0).powf(0.77)).max(0.0);
    sl0 * (t_u / 298.0).powf(alpha) * (p / 101_325.0).powf(beta) * dil * (1.0 + 0.2 * w_eth)
}

/// Peters (1999) turbulent burning-velocity increment (S_T/S_L − 1) for u'/S_L and L/l_F (l_F = a/S_L):
/// −A·(L/l_F) + √((A·L/l_F)² + a4·b3²·(u'/S_L)·(L/l_F)), A = a4·b3²/(2·b1), a4 = 0.78, b1 = 2.0, b3 = 1.0.
#[inline]
pub fn peters_increment(u_over_sl: f64, l_over_lf: f64) -> f64 {
    let a = 0.78 / 4.0 * l_over_lf;
    -a + (a * a + 0.78 * u_over_sl.max(0.0) * l_over_lf).sqrt()
}

/// Dynamic viscosity of the unburned charge, Pa·s (Sutherland, air).
#[inline]
pub fn viscosity(t: f64) -> f64 {
    1.458e-6 * t.powf(1.5) / (t + 110.4)
}

// ---- turbulence -----------------------------------------------------------------------------------------------

/// Mean-flow kinetic energy K, tumble angular momentum L_T and turbulent kinetic energy k of the charge.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct Turbulence {
    /// Mean-flow kinetic energy, J.
    pub k_mean: f64,
    /// Tumble angular momentum, kg·m²/s.
    pub l_tumble: f64,
    /// Turbulent kinetic energy, J.
    pub k_turb: f64,
}

impl Turbulence {
    /// Inflow of `dm` kg at jet velocity `v` m/s (intake valve curtain): mean-flow energy and tumble momentum.
    #[inline]
    pub fn inflow(&mut self, dm: f64, v: f64, tumble_coef: f64, bore: f64) {
        self.k_mean += 0.5 * dm * v * v;
        self.l_tumble += tumble_coef * dm * v * 0.5 * bore;
        // the jet's own shear layer: a part of its energy is turbulent on arrival (UNVERIFIED share)
        self.k_turb += 0.02 * dm * v * v;
    }

    /// Mass `frac` of the charge leaves (outflow / backflow) carrying its share of K, L_T and k.
    #[inline]
    pub fn outflow(&mut self, frac: f64) {
        let f = (1.0 - frac).clamp(0.0, 1.0);
        self.k_mean *= f;
        self.l_tumble *= f;
        self.k_turb *= f;
    }

    /// Turbulence intensity u' = √(2k/3m), m/s.
    #[inline]
    pub fn u_prime(&self, m: f64) -> f64 {
        if m <= 0.0 { 0.0 } else { (2.0 * self.k_turb / (3.0 * m)).max(0.0).sqrt() }
    }

    /// Advances by `dt`: K → k (turbulent shear), tumble breakdown → k, dissipation ε = u'³/L, rapid-distortion
    /// amplification of k by the density ratio `rho_ratio` (k ∝ ρ^⅔). `h` = instantaneous chamber height, m.
    #[inline]
    pub fn step(&mut self, hw: &CombustionHw, dt: f64, m: f64, rho_ratio: f64, h: f64, bore: f64) {
        if m <= 0.0 {
            return;
        }
        let l = (hw.length_scale * h.min(0.5 * bore)).max(5e-4);
        if rho_ratio > 0.0 && rho_ratio != 1.0 && rho_ratio.is_finite() {
            self.k_turb *= rho_ratio.powf(2.0 / 3.0);
        }
        let up = self.u_prime(m);
        // mean flow → turbulence (Poulos & Heywood 1983: P = 0.3307·c_β·(K/L)·√(k/m))
        let p_k = (0.3307 * hw.c_beta * self.k_mean / l * (self.k_turb / m).max(0.0).sqrt()).min(self.k_mean / dt);
        // tumble: angular momentum of a gas cylinder Ø bore × h about a transverse axis, I = m(3B²/4 + h²)/12
        let inertia = m * (0.75 * bore * bore + h * h) / 12.0;
        let w_t = self.l_tumble / inertia;
        let e_t = 0.5 * self.l_tumble * w_t;
        let flat = (1.0 - h / (hw.tumble_h0 * bore)).max(0.0);
        let rate = hw.tumble_decay * w_t.abs() * flat * flat;
        let de_t = e_t * (1.0 - (-2.0 * rate * dt).exp());
        // angular momentum lost with the energy (E ∝ L²) plus wall friction of the vortex (≈ 1 %/rev, UNVERIFIED)
        self.l_tumble *= (-(rate + 0.02 * w_t.abs() / (2.0 * PI)) * dt).exp();
        let eps = m * up * up * up / l;
        self.k_mean -= p_k * dt;
        self.k_turb = (self.k_turb + p_k * dt + de_t - eps * dt).max(1e-12 * m);
    }

    /// Integral length scale, m.
    #[inline]
    pub fn length(hw: &CombustionHw, h: f64, bore: f64) -> f64 {
        (hw.length_scale * h.min(0.5 * bore)).max(5e-4)
    }
}

// ---- flame geometry ---------------------------------------------------------------------------------------------

/// Sphere of radius `r` centred on the cylinder axis at depth `zp` below the fire deck of a flat cylinder of radius
/// `rc` and height `h`: enclosed volume and the free (non-wall) part of its surface. Archimedes: a spherical zone
/// between two planes has area 2πr·Δz, so the free surface is 2πr × the z-length where the sphere is inside the liner.
#[inline]
pub fn flame_volume_area(r: f64, zp: f64, rc: f64, h: f64) -> (f64, f64) {
    if r <= 0.0 {
        return (0.0, 0.0);
    }
    let (z0, z1) = ((zp - r).max(0.0), (zp + r).min(h));
    if z1 <= z0 {
        return (0.0, 0.0);
    }
    // |z − zp| < d: the cross-section is the whole bore
    let d = if r > rc { (r * r - rc * rc).sqrt() } else { 0.0 };
    // ∫ (r² − u²) du = r²u − u³/3 over u = z − zp
    let cap = |a: f64, b: f64| -> f64 { if b <= a { 0.0 } else { PI * (r * r * (b - a) - (b * b * b - a * a * a) / 3.0) } };
    let (u0, u1) = (z0 - zp, z1 - zp);
    // split [u0, u1] into |u| < d (full bore) and |u| ≥ d (spherical cap)
    let (ci0, ci1) = (u0.max(-d), u1.min(d));
    let full = if ci1 > ci0 { PI * rc * rc * (ci1 - ci0) } else { 0.0 };
    let v = full + cap(u0, u1.min(-d)) + cap(u0.max(d), u1);
    let free_len = (u1 - u0) - (ci1 - ci0).max(0.0);
    (v, 2.0 * PI * r * free_len.max(0.0))
}

/// Flame radius enclosing `v` (Newton from `r_guess`, bisection fallback) and its free area.
#[inline]
pub fn flame_radius(v: f64, r_guess: f64, zp: f64, rc: f64, h: f64) -> (f64, f64) {
    let (v_all, _) = flame_volume_area(1e3, zp, rc, h);
    if v >= v_all {
        return (((zp.max(h - zp)).powi(2) + rc * rc).sqrt(), 0.0);
    }
    let (mut lo, mut hi) = (0.0, ((zp.max(h - zp)).powi(2) + rc * rc).sqrt());
    let mut r = r_guess.clamp(1e-5, hi);
    for _ in 0..12 {
        let (vr, a) = flame_volume_area(r, zp, rc, h);
        let f = vr - v;
        if f.abs() <= 1e-6 * v.max(1e-12) {
            return (r, a);
        }
        if f > 0.0 { hi = r } else { lo = r }
        let rn = if a > 0.0 { r - f / a } else { 0.5 * (lo + hi) };
        r = if rn > lo && rn < hi { rn } else { 0.5 * (lo + hi) };
    }
    let (_, a) = flame_volume_area(r, zp, rc, h);
    (r, a)
}

// ---- kinetics -----------------------------------------------------------------------------------------------------

/// Thermal-NO formation rate in the burned zone, d(y_NO)/dt (mass fraction per s), extended Zeldovich (Heywood 1988
/// eq. 11.8 with R1/(R2+R3) ≈ 1 near stoichiometric): `d[NO]/dt = 2R1(1 − α²)/(1 + α)`, `α = [NO]/[NO]e`,
/// `R1 = k1⁺[O]e[N2]e`, k1⁺ = 7.6·10¹³·exp(−38 000/T) cm³/mol·s, `[O]e` from O2 ⇌ 2O (Heywood eq. 11.5),
/// `[NO]e` from N2 + O2 ⇌ 2NO (Kp = 20.3·exp(−21 650/T)). `x_o2`, `x_n2` are equilibrium mole fractions in the burned gas,
/// `y_no` the current NO mass fraction, `rho` the burned-gas density kg/m³.
#[inline]
pub fn zeldovich_dyno_dt(t: f64, p: f64, rho: f64, x_o2: f64, x_n2: f64, y_no: f64) -> f64 {
    if t < 1600.0 || x_o2 <= 0.0 || x_n2 <= 0.0 {
        return 0.0;
    }
    let c_tot = p / (8.314 * t) * 1e-6; // mol/cm³
    let (o2, n2) = (x_o2 * c_tot, x_n2 * c_tot);
    // [O]e = 3.6·10³·exp(−31 090/T)·[O2]e^½/(R̃T)^½, R̃ = 82.05 cm³·atm/(mol·K) (Heywood eq. 11.5)
    let o = 3.6e3 * (-31_090.0 / t).exp() * o2.sqrt() / (82.05 * t).sqrt();
    let no_e = (20.3 * (-21_650.0 / t).exp() * o2 * n2).sqrt();
    let k1 = 7.6e13 * (-38_000.0 / t).exp();
    let r1 = k1 * o * n2; // mol/cm³·s
    let no = y_no * rho / 0.030 * 1e-6; // mol/cm³
    let a = if no_e > 0.0 { no / no_e } else { 0.0 };
    let dno = 2.0 * r1 * (1.0 - a * a) / (1.0 + a); // mol/cm³·s
    dno * 1e6 * 0.030 / rho
}

/// Equilibrium O2 mole fraction of burned gas at `t` K: excess oxygen of a lean mixture plus thermal dissociation of
/// stoichiometric products (fit to equilibrium of C8H18/air at 30 bar: 0.4 % at 2400 K, 1.8 % at 2800 K — Heywood
/// 1988 fig. 3-10 class data; UNVERIFIED fit).
#[inline]
pub fn burned_o2_mole_frac(lambda: f64, t: f64) -> f64 {
    let excess = if lambda > 1.0 { 0.21 * (lambda - 1.0) / (lambda + 0.07) } else { 0.0 };
    excess + (4.99 - 25_218.0 / t.max(1000.0)).exp()
}

/// Engine-out CO mole fraction (dry-ish basis, %vol/100) from λ: rich — water-gas shift frozen at K = 3.5 (Heywood
/// §4.9.2) over the oxygen deficit; lean/stoichiometric — the kinetically frozen remainder (≈ 0.5 % at λ = 1 falling to
/// 0.1 %, Heywood fig. 11-20).
#[inline]
pub fn co_mole_frac(lambda: f64) -> f64 {
    if lambda < 1.0 {
        // oxygen deficit (as O atoms per mol of products) split between CO and H2 by K = [CO2][H2]/([CO][H2O]) = 3.5 with
        // gasoline H/C ≈ 1.87 → CO takes ≈ 0.58 of the deficit (Heywood eq. 4.29 solution, UNVERIFIED simplification)
        let deficit = 2.0 * 0.21 * (1.0 - lambda) / (1.0 + 0.07);
        0.58 * deficit + 0.005
    } else {
        0.001 + 0.004 * (-(lambda - 1.0) / 0.03).exp()
    }
}

/// Westbrook–Dryer (1981) global n-octane oxidation: fraction of the fuel vapour in a gas of temperature `t`, pressure
/// `p`, fuel / O2 mass fractions `y_f`/`y_o2` that burns within `dt` (`rate = 4.6·10¹¹·exp(−15 098/T)·[F]^0.25·[O2]^1.5`,
/// mol/cm³·s).
#[inline]
pub fn afterburn_fraction(t: f64, p: f64, y_f: f64, y_o2: f64, dt: f64) -> f64 {
    if t < 700.0 || y_f <= 0.0 || y_o2 <= 0.0 {
        return 0.0;
    }
    let rho = p / (288.0 * t);
    let f = rho * y_f / 0.114 * 1e-6;
    let o2 = rho * y_o2 / 0.032 * 1e-6;
    let rate = 4.6e11 * (-15_098.0 / t).exp() * f.powf(0.25) * o2.powf(1.5);
    (1.0 - (-rate / f * dt).exp()).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Metghalchi–Keck at their reference: indolene φ = 1, 298 K, 1 atm = 0.27 m/s; at engine conditions (650 K, 10 bar)
    /// 0.6–1.0 m/s; 20 % residual roughly halves it (Rhodes & Keck 1985).
    #[test]
    fn laminar_flame_speed_matches_metghalchi_keck() {
        let s0 = laminar_flame_speed(1.0, 298.0, 101_325.0, 0.0, 0.0);
        assert!((s0 - 0.262).abs() < 0.02, "{s0}");
        let se = laminar_flame_speed(1.0, 650.0, 10e5, 0.0, 0.0);
        assert!((0.6..1.1).contains(&se), "{se}");
        let sd = laminar_flame_speed(1.0, 650.0, 10e5, 0.2, 0.0);
        assert!((0.4..0.6).contains(&(sd / se)), "{}", sd / se);
        assert!(laminar_flame_speed(1.1, 650.0, 10e5, 0.0, 0.0) > se);
        assert!(laminar_flame_speed(0.7, 650.0, 10e5, 0.0, 0.0) < 0.6 * se);
    }

    /// Without production, turbulence decays as k(t) = k0/(1 + t/τ)², τ = 3L/u'0, with ε = u'³/L (homogeneous decay,
    /// t⁻² law for a fixed length scale).
    #[test]
    fn turbulence_decays_with_the_fixed_length_scale_law() {
        let hw = CombustionHw::ea888_gen3();
        let (m, h, b) = (5e-4, 0.04, 0.0825);
        let mut t = Turbulence { k_turb: 1.5 * m * 4.0, ..Default::default() }; // u' = 2 m/s
        let l = Turbulence::length(&hw, h, b);
        let tau = 3.0 * l / 2.0; // k' = −m·u'³/L → u' = u0/(1 + t/τ), τ = 3L/u0
        let dt = 1e-5;
        for _ in 0..(tau / dt) as usize {
            t.step(&hw, dt, m, 1.0, h, b);
        }
        let ratio = t.k_turb / (6.0 * m);
        assert!((ratio - 0.25).abs() < 0.03, "k/k0 after τ = {ratio} (expect 1/4)");
    }

    #[test]
    fn flame_geometry_is_consistent() {
        let (rc, h, zp) = (0.04125, 0.012, 0.004);
        // small sphere: full sphere
        let (v, a) = flame_volume_area(0.003, zp, rc, h);
        assert!((v - 4.0 / 3.0 * PI * 0.003f64.powi(3)).abs() < 1e-12 && (a - 4.0 * PI * 9e-6).abs() < 1e-9);
        // huge sphere: the whole chamber, no free surface
        let (v, a) = flame_volume_area(1.0, zp, rc, h);
        assert!((v - PI * rc * rc * h).abs() < 1e-10 && a == 0.0);
        // dV/dr = A
        let r = 0.02;
        let (v1, a1) = flame_volume_area(r, zp, rc, h);
        let (v2, _) = flame_volume_area(r + 1e-7, zp, rc, h);
        assert!(((v2 - v1) / 1e-7 - a1).abs() / a1 < 1e-3);
        let (rr, _) = flame_radius(v1, 0.005, zp, rc, h);
        assert!((rr - r).abs() < 1e-7);
    }

    /// Thermal NO: at 2600 K, 40 bar, stoichiometric burned gas, NO forms at ≈ 10³–10⁴ ppm/ms-class rates (Heywood
    /// fig. 11-4: ~1 ms to approach equilibrium ~ 5000 ppm); it stops below ≈ 1800 K.
    #[test]
    fn zeldovich_rates_have_the_textbook_magnitude() {
        let (t, p) = (2600.0, 40e5);
        let rho = p / (288.0 * t);
        let x_o2 = burned_o2_mole_frac(1.0, t);
        let r = zeldovich_dyno_dt(t, p, rho, x_o2, 0.72, 0.0);
        let ppm_per_ms = r * 1e-3 * 28.9 / 30.0 * 1e6;
        assert!((300.0..30_000.0).contains(&ppm_per_ms), "{ppm_per_ms} ppm/ms");
        let cold = zeldovich_dyno_dt(1800.0, p, p / (288.0 * 1800.0), burned_o2_mole_frac(1.0, 1800.0), 0.72, 0.0);
        assert!(cold * 1e-3 * 1e6 < 1.0);
    }

    /// Hot-surface ignition (the delay `hot_surface_c`·τ_DE at the surface temperature integrated over a full-load
    /// compression, 2000 rpm, 1.8 bar at IVC, ε 9.6, RON 95) sets in at a surface temperature of 850–1000 °C — the range
    /// NGK / Bosch give for plug-tip pre-ignition — and not below 800 °C.
    #[test]
    fn hot_surface_ignition_sets_in_where_the_plug_makers_put_it() {
        let hw = CombustionHw::ea888_gen3();
        let ignites = |t_s: f64| {
            let (w, eps, kappa) = (2000.0 * PI / 30.0, 9.6f64, 1.32);
            let mut int = 0.0;
            let mut a = -140.0f64;
            while a < -10.0 {
                // volume ratio from BDC with a 0.3 rod/crank ratio, isentropic compression from 1.8 bar
                let x = |d: f64| {
                    let r = d.to_radians();
                    1.0 + (eps - 1.0) / 2.0 * (1.0 - r.cos() + (1.0 - (1.0 - (0.3 * r.sin()).powi(2)).sqrt()) / 0.3)
                };
                let p = 1.8e5 * ((x(180.0) / x(a)).powf(kappa));
                int += (0.1f64.to_radians() / w) / (hw.hot_surface_c * autoignition_delay(95.0, p, t_s));
                a += 0.1;
            }
            int >= 1.0
        };
        let onset = (800..1100).find(|&c| ignites(c as f64 + 273.15)).expect("no ignition up to 1100 °C");
        assert!((850..=1000).contains(&onset), "hot-surface ignition onset {onset} °C");
    }

    /// Box–Muller pair: zero mean, unit variance (minus the ±3σ clip), uncorrelated.
    #[test]
    fn normal_pair_is_standard_normal() {
        let mut r = Rng(7);
        let v: Vec<(f64, f64)> = (0..20_000).map(|_| r.normal_pair()).collect();
        let n = v.len() as f64;
        let (ma, mb) = (v.iter().map(|x| x.0).sum::<f64>() / n, v.iter().map(|x| x.1).sum::<f64>() / n);
        let (va, cab) = (v.iter().map(|x| x.0 * x.0).sum::<f64>() / n, v.iter().map(|x| x.0 * x.1).sum::<f64>() / n);
        assert!(ma.abs() < 0.03 && mb.abs() < 0.03, "means {ma} {mb}");
        assert!((va - 0.99).abs() < 0.04, "variance {va}");
        assert!(cab.abs() < 0.03, "covariance {cab}");
    }

    #[test]
    fn afterburn_needs_heat_and_oxygen() {
        assert!(afterburn_fraction(1300.0, 2e5, 0.01, 0.03, 5e-3) > 0.5);
        assert!(afterburn_fraction(800.0, 2e5, 0.01, 0.03, 5e-3) < 0.01);
        assert_eq!(afterburn_fraction(1300.0, 2e5, 0.01, 0.0, 5e-3), 0.0);
    }

    /// Gasoline film on a cold wall distils: the light ends leave first, the heavy aromatic stays (the slow tail of the
    /// film measurements); on a wall above every component's boiling point it boils off within milliseconds, as the former
    /// single-component hot-wall film did; the gas heat flux of a flame burns it off whatever the wall.
    #[test]
    fn fuel_film_distils_on_a_cold_wall_and_boils_off_a_hot_one() {
        let run = |t_k: f64, p: f64, k_heat: f64, secs: f64| {
            let mut m = fuel_components(0.0).map(|w| w * 10e-6);
            for _ in 0..(secs / 1e-4) as usize {
                film_evaporate(&mut m, t_k, p, 3.5, k_heat, 1e-4);
            }
            m.map(|x| x / 10e-6)
        };
        // 5 °C wall, intake pressure, mass transfer of a cranking engine (h ≈ 200 W/(m²·K), 30 µm film → k_diff ≈ 3.5 /s)
        let cold = run(278.15, 0.8e5, 0.0, 0.5);
        assert!(cold[0] < 0.5 * FUEL_PSEUDO[0].0, "light ends leave: {cold:?}");
        assert!(cold[1] > 0.9 * FUEL_PSEUDO[1].0 && cold[2] > 0.995 * FUEL_PSEUDO[2].0, "mid and heavy stay: {cold:?}");
        // the same film at 20 °C loses its mid fraction faster than at 5 °C (iso-octane 5.2 vs 2.5 kPa)
        let mild = run(293.15, 0.8e5, 0.0, 0.5);
        assert!(mild[1] < cold[1], "{mild:?} vs {cold:?}");
        // 200 °C crown (full load) at 1.8 bar intake pressure: above every boiling point, gone within the intake stroke
        let hot: f64 = run(473.15, 1.8e5, 0.0, 0.01).iter().sum();
        assert!(hot < 0.02, "hot crown keeps {hot:.3}");
        // flame heat flux 1 MW/m² into a 30 µm film on a cold wall: about a quarter of it per 5 ms of burning
        let k_heat = 1e6 / (745.0 * 30e-6 * 750e3);
        let burnt = 1.0 - run(278.15, 30e5, k_heat, 5e-3).iter().sum::<f64>();
        assert!((0.15..0.35).contains(&burnt), "pool fire burnt {burnt:.2}");
    }
}
