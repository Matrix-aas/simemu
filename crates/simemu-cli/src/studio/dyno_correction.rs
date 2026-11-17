//! Dyno power/torque correction to standard reference atmospheres (spark-ignition engines).
//!
//! `corrected = observed × CF`. Inputs are the bench atmosphere: total barometric pressure,
//! water-vapour partial pressure and engine air-inlet temperature (the plant's ambient — the
//! standards measure inlet air at the air-cleaner entry, not the intake manifold).
//!
//! - SAE J1349 (JUN1990 / AUG2004), SI engines:
//!   `CF = 1.18 · (99 / Pd) · √((T + 273) / 298) − 0.18`, Pd = dry-air pressure kPa, T = °C.
//!   Reference 99 kPa dry air, 25 °C. The 1.18 / −0.18 form embeds an assumed 85 % mechanical
//!   efficiency (the correction is applied to indicated power, friction left uncorrected).
//! - DIN 70020: `CF = (1013 / p) · √((273 + t) / 293)`, p = total pressure mbar, t = °C.
//!   Reference 1013 mbar, 20 °C; humidity is ignored.
//! - ISO 1585 (= UN ECE R85), SI engines: `αa = (99 / ps)^1.2 · (T / 298)^0.6`,
//!   ps = dry-air pressure kPa, T = inlet temperature K. Reference 99 kPa dry, 298 K.
//!   A test is valid only for `0.93 ≤ αa ≤ 1.07`.
//! - EEC 80/1269 (as amended, SI engines): same factor and validity band as ISO 1585; reported
//!   separately because dyno reports name the directive explicitly.

use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct CorrectionFactor {
    pub id: &'static str,
    pub name: &'static str,
    pub cf: f64,
    pub reference: &'static str,
    pub formula: &'static str,
    /// Validity band of the factor stated by the standard, if it states one.
    pub valid: Option<[f64; 2]>,
    pub in_range: Option<bool>,
}

#[derive(Debug, Clone, Serialize)]
pub struct CorrectionReport {
    pub p_total_kpa: f64,
    pub p_vapour_kpa: f64,
    pub p_dry_kpa: f64,
    pub inlet_c: f64,
    pub factors: Vec<CorrectionFactor>,
}

pub fn correction(p_total_kpa: f64, p_vapour_kpa: f64, inlet_c: f64) -> CorrectionReport {
    let pd = (p_total_kpa - p_vapour_kpa).max(1.0);
    let tk = inlet_c + 273.15;
    let iso = (99.0 / pd).powf(1.2) * (tk / 298.0).powf(0.6);
    let band = [0.93, 1.07];
    let f = |id, name, cf: f64, reference, formula, valid: Option<[f64; 2]>| CorrectionFactor {
        id,
        name,
        cf,
        reference,
        formula,
        valid,
        in_range: valid.map(|[lo, hi]| (lo..=hi).contains(&cf)),
    };
    CorrectionReport {
        p_total_kpa,
        p_vapour_kpa,
        p_dry_kpa: pd,
        inlet_c,
        factors: vec![
            f(
                "sae_j1349",
                "SAE J1349",
                1.18 * (99.0 / pd) * ((inlet_c + 273.0) / 298.0).sqrt() - 0.18,
                "99 kPa dry air, 25 °C",
                "1.18·(99/Pd)·√((T+273)/298) − 0.18",
                None,
            ),
            f(
                "din_70020",
                "DIN 70020",
                (101.3 / p_total_kpa.max(1.0)) * ((273.0 + inlet_c) / 293.0).sqrt(),
                "1013 mbar total, 20 °C",
                "(1013/p)·√((273+t)/293)",
                None,
            ),
            f("iso_1585", "ISO 1585 / ECE R85", iso, "99 kPa dry air, 298 K", "(99/ps)^1.2·(T/298)^0.6", Some(band)),
            f("eec_80_1269", "EEC 80/1269", iso, "99 kPa dry air, 298 K", "(99/ps)^1.2·(T/298)^0.6", Some(band)),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn factors_are_unity_at_their_reference_atmospheres() {
        let cf = |r: &CorrectionReport, id: &str| r.factors.iter().find(|f| f.id == id).unwrap().cf;
        // SAE / ISO: 99 kPa dry air, 25 °C.
        let r = correction(99.0, 0.0, 25.0);
        assert!((cf(&r, "sae_j1349") - 1.0).abs() < 1e-9);
        assert!((cf(&r, "iso_1585") - 1.0).abs() < 1e-3);
        // DIN: 1013 mbar total, 20 °C (vapour ignored).
        let r = correction(101.3, 1.5, 20.0);
        assert!((cf(&r, "din_70020") - 1.0).abs() < 1e-9);
        // Thin hot air → factor > 1; the validity band is reported.
        let r = correction(85.0, 1.0, 35.0);
        assert!(cf(&r, "iso_1585") > 1.07 && r.factors[2].in_range == Some(false));
    }
}
