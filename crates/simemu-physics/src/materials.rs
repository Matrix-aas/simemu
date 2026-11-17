//! Temperature-dependent material data and damage-accumulation laws.
//!
//! Components fail when the *physical* load (stress, temperature, film thickness) exceeds what the
//! material can carry at its current temperature. Cyclic loads accumulate Palmgren–Miner damage
//! from a Basquin S–N line anchored at the fatigue strength `fatigue_mpa` at `n_ref` cycles; a
//! peak stress at or above the temperature-dependent UTS is a one-cycle (static) failure. Creep is
//! accumulated with the Larson–Miller parameter. Tables are handbook-class values; every entry is
//! UNVERIFIED for the exact alloy/heat treatment used by the OEM.

/// Piecewise-linear interpolation in an ascending `(x, y)` table, clamped at both ends.
pub fn interp(t: &[(f64, f64)], x: f64) -> f64 {
    if x <= t[0].0 {
        return t[0].1;
    }
    for w in t.windows(2) {
        let ((x0, y0), (x1, y1)) = (w[0], w[1]);
        if x <= x1 {
            return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
        }
    }
    t[t.len() - 1].1
}

#[derive(Debug, Clone, Copy)]
pub struct Alloy {
    pub name: &'static str,
    pub density: f64,
    pub youngs_pa: f64,
    /// (T °C, ultimate tensile strength MPa).
    pub uts_mpa: &'static [(f64, f64)],
    /// (T °C, 0.2 % yield strength MPa).
    pub yield_mpa: &'static [(f64, f64)],
    /// (T °C, fully reversed fatigue strength MPa at `n_ref` cycles).
    pub fatigue_mpa: &'static [(f64, f64)],
    pub n_ref: f64,
    /// Basquin slope k: N = n_ref·(σ_f/σ_a)^k.
    pub basquin_k: f64,
    /// True for steels with a genuine endurance limit (no damage below σ_f).
    pub endurance_limit: bool,
}

impl Alloy {
    pub fn uts_pa(&self, t_c: f64) -> f64 {
        interp(self.uts_mpa, t_c).max(0.0) * 1e6
    }
    pub fn yield_pa(&self, t_c: f64) -> f64 {
        interp(self.yield_mpa, t_c).max(0.0) * 1e6
    }
    pub fn fatigue_pa(&self, t_c: f64) -> f64 {
        interp(self.fatigue_mpa, t_c).max(0.0) * 1e6
    }

    /// Miner damage of one load cycle between `s_max` and `s_min` (Pa, tension positive) at `t_c`.
    /// Goodman mean-stress correction for tensile mean; compressive mean is not credited.
    pub fn cycle_damage(&self, s_max: f64, s_min: f64, t_c: f64) -> f64 {
        if !s_max.is_finite() || !s_min.is_finite() || !t_c.is_finite() {
            return 0.0;
        }
        let (s_hi, s_lo) = if s_max >= s_min { (s_max, s_min) } else { (s_min, s_max) };
        let uts = self.uts_pa(t_c);
        if uts <= 0.0 || s_hi >= uts {
            return 1.0;
        }
        let sa = 0.5 * (s_hi - s_lo);
        let sm = 0.5 * (s_hi + s_lo);
        let sar = if sm > 0.0 {
            let denom = 1.0 - sm / uts;
            if denom <= 0.0 {
                return 1.0;
            }
            sa / denom
        } else {
            sa
        };
        let sf = self.fatigue_pa(t_c);
        if sar <= 0.0 || (self.endurance_limit && sar < sf) {
            return 0.0;
        }
        if sf <= 0.0 {
            return 1.0;
        }
        ((sar / sf).powf(self.basquin_k) / self.n_ref).min(1.0)
    }
}

/// Cast piston alloy AlSi12CuNiMg (MAHLE M124-class, T7). Tensile and fatigue strength and Young's modulus at 20 / 150 /
/// 250 / 350 °C: Shariyat, Fathi Sola & Jazayeri, Lat. Am. J. Solids Struct. 13(6) 2016, Table 7 (M124, T5/T7; "in a
/// close concordance with the data reported by" MAHLE GmbH 2012; <https://doi.org/10.1590/1679-78251937>, VERIFIED-DS):
/// UTS 241 / 207 / 117 / 53 MPa, fatigue strength 108 / 84 / 53 / 26 MPa, E 79 GPa at 20 °C.
/// Above 350 °C: the former handbook-class shape scaled to the 350 °C point, yield: the former table scaled to the UTS —
/// UNVERIFIED. Solidus of the eutectic AlSi12CuNiMg ≈ 530 °C.
pub const PISTON_ALSI12CUNIMG: Alloy = Alloy {
    name: "AlSi12CuNiMg (M124 T7)",
    density: 2680.0,
    youngs_pa: 79e9,
    uts_mpa: &[(20.0, 241.0), (150.0, 207.0), (250.0, 117.0), (350.0, 53.0), (400.0, 32.0), (450.0, 19.0), (530.0, 0.0)],
    yield_mpa: &[(20.0, 210.0), (150.0, 182.0), (250.0, 97.0), (350.0, 42.0), (400.0, 24.0), (450.0, 14.0), (530.0, 0.0)],
    fatigue_mpa: &[(20.0, 108.0), (150.0, 84.0), (250.0, 53.0), (350.0, 26.0), (400.0, 17.0), (450.0, 10.0), (530.0, 0.0)],
    n_ref: 5e7,
    basquin_k: 8.0,
    endurance_limit: false,
};
pub const PISTON_SOLIDUS_C: f64 = 530.0;

/// Micro-alloyed forging steel 36MnVS4 (fracture-split connecting rods). DIN EN 10267: Rm 900–1050,
/// Re ≥ 600 MPa. Fatigue (R = −1) ≈ 0.45·Rm. Temperature table UNVERIFIED.
pub const ROD_36MNVS4: Alloy = Alloy {
    name: "36MnVS4",
    density: 7850.0,
    youngs_pa: 210e9,
    uts_mpa: &[(20.0, 950.0), (200.0, 900.0), (400.0, 760.0)],
    yield_mpa: &[(20.0, 620.0), (200.0, 580.0), (400.0, 480.0)],
    fatigue_mpa: &[(20.0, 420.0), (200.0, 400.0), (400.0, 330.0)],
    n_ref: 2e6,
    basquin_k: 12.0,
    endurance_limit: true,
};

/// Valve steel (intake X45CrSi9-3 / exhaust X53CrMnNiN21-9 class). UNVERIFIED.
pub const VALVE_STEEL: Alloy = Alloy {
    name: "X45CrSi9-3 / X53CrMnNiN21-9",
    density: 7700.0,
    youngs_pa: 205e9,
    uts_mpa: &[(20.0, 1000.0), (500.0, 800.0), (700.0, 550.0), (800.0, 400.0), (900.0, 250.0), (1100.0, 60.0)],
    yield_mpa: &[(20.0, 700.0), (500.0, 550.0), (700.0, 380.0), (800.0, 280.0), (900.0, 170.0), (1100.0, 40.0)],
    fatigue_mpa: &[(20.0, 450.0), (700.0, 260.0), (900.0, 120.0)],
    n_ref: 1e7,
    basquin_k: 12.0,
    endurance_limit: true,
};

/// Forged compressor-wheel alloy AA2618-T61. RT: Rm 440, Rp0.2 370 MPa (ASM Handbook vol. 2);
/// elevated-temperature points UNVERIFIED.
pub const WHEEL_AA2618: Alloy = Alloy {
    name: "AA2618-T61",
    density: 2760.0,
    youngs_pa: 74e9,
    uts_mpa: &[(20.0, 440.0), (100.0, 420.0), (150.0, 380.0), (200.0, 310.0), (250.0, 220.0), (300.0, 130.0)],
    yield_mpa: &[(20.0, 370.0), (100.0, 350.0), (150.0, 320.0), (200.0, 260.0), (250.0, 180.0), (300.0, 100.0)],
    fatigue_mpa: &[(20.0, 125.0), (150.0, 110.0), (250.0, 80.0)],
    n_ref: 5e8,
    basquin_k: 7.0,
    endurance_limit: false,
};

/// Cast turbine-wheel superalloy Inconel 713C. UTS vs T after Special Metals / INCO data sheet
/// (approx.), UNVERIFIED.
pub const WHEEL_IN713C: Alloy = Alloy {
    name: "Inconel 713C",
    density: 7910.0,
    youngs_pa: 200e9,
    uts_mpa: &[(20.0, 850.0), (650.0, 860.0), (760.0, 820.0), (870.0, 700.0), (980.0, 500.0), (1100.0, 250.0), (1250.0, 0.0)],
    yield_mpa: &[(20.0, 740.0), (650.0, 700.0), (760.0, 690.0), (870.0, 600.0), (980.0, 400.0), (1100.0, 200.0), (1250.0, 0.0)],
    fatigue_mpa: &[(20.0, 300.0), (760.0, 250.0), (980.0, 150.0)],
    n_ref: 1e8,
    basquin_k: 10.0,
    endurance_limit: false,
};

/// MLS head-gasket bead, spring-hard stainless 1.4310 (301 FH). UNVERIFIED.
pub const GASKET_BEAD_1_4310: Alloy = Alloy {
    name: "1.4310 FH",
    density: 7900.0,
    youngs_pa: 190e9,
    uts_mpa: &[(20.0, 1300.0), (300.0, 1150.0)],
    yield_mpa: &[(20.0, 1100.0), (300.0, 950.0)],
    fatigue_mpa: &[(20.0, 500.0), (300.0, 440.0)],
    n_ref: 1e7,
    basquin_k: 10.0,
    endurance_limit: true,
};

/// Time to creep rupture (h) from a Larson–Miller line `LMP(σ) = lmp0 − slope·(σ − σ0)` with C = 20
/// (`LMP = T[K]·(20 + log10 t[h])`).
pub fn larson_miller_rupture_h(t_k: f64, sigma_mpa: f64, lmp0: f64, sigma0_mpa: f64, slope_per_mpa: f64) -> f64 {
    let lmp = lmp0 - slope_per_mpa * (sigma_mpa - sigma0_mpa);
    10f64.powf((lmp / t_k.max(300.0) - 20.0).min(30.0))
}

/// Inconel 713C: LMP ≈ 27 570 at 150 MPa (≈100 h at 980 °C), −16 per MPa. UNVERIFIED fit.
pub fn in713c_rupture_h(t_k: f64, sigma_mpa: f64) -> f64 {
    larson_miller_rupture_h(t_k, sigma_mpa, 27_570.0, 150.0, 16.0)
}

/// Exhaust-valve steel X53CrMnNiN21-9: LMP ≈ 22 730 at 140 MPa (≈100 h at 760 °C), −25 per MPa. UNVERIFIED.
pub fn valve_21_4n_rupture_h(t_k: f64, sigma_mpa: f64) -> f64 {
    larson_miller_rupture_h(t_k, sigma_mpa, 22_730.0, 140.0, 25.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn piston_alloy_softens_with_temperature_and_damage_rises() {
        let a = &PISTON_ALSI12CUNIMG;
        assert!(a.fatigue_pa(350.0) < 0.5 * a.fatigue_pa(20.0));
        let cold = a.cycle_damage(60e6, 0.0, 200.0);
        let hot = a.cycle_damage(60e6, 0.0, 380.0);
        assert!(hot > 100.0 * cold, "{cold} {hot}");
        assert_eq!(a.cycle_damage(300e6, 0.0, 20.0), 1.0);
    }

    #[test]
    fn steel_has_endurance_limit_and_creep_is_temperature_driven() {
        assert_eq!(ROD_36MNVS4.cycle_damage(300e6, -300e6, 120.0), 0.0);
        assert!(ROD_36MNVS4.cycle_damage(500e6, -500e6, 120.0) > 0.0);
        assert!(in713c_rupture_h(1373.0, 150.0) < 0.05 * in713c_rupture_h(1223.0, 150.0));
    }

    #[test]
    fn cycle_damage_is_symmetric_in_bounds_and_safe_at_uts() {
        let a = &PISTON_ALSI12CUNIMG;
        let d_fwd = a.cycle_damage(90e6, 10e6, 250.0);
        let d_rev = a.cycle_damage(10e6, 90e6, 250.0);
        assert_eq!(d_fwd, d_rev);
        assert_eq!(a.cycle_damage(0.0, 300e6, 20.0), 1.0, "reversed UTS exceed must fail statically");
        let uts = a.uts_pa(200.0);
        assert_eq!(a.cycle_damage(uts, uts, 200.0), 1.0);
        assert_eq!(a.cycle_damage(f64::NAN, 0.0, 200.0), 0.0);
    }
}
