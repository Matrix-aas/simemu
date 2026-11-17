//! Ambient air, gas-flow and lubricant property helpers (pure functions).

/// Specific gas constant of dry air, J/(kg·K).
pub const R_AIR: f64 = 287.058;
/// Specific gas constant of water vapour, J/(kg·K).
pub const R_VAPOUR: f64 = 461.495;
/// Isentropic exponent of air.
pub const GAMMA_AIR: f64 = 1.4;
/// Isentropic exponent of exhaust gas.
pub const GAMMA_EXH: f64 = 1.33;
/// cp of air, J/(kg·K).
pub const CP_AIR: f64 = 1005.0;
/// cp of exhaust gas, J/(kg·K).
pub const CP_EXH: f64 = 1150.0;
pub const KELVIN: f64 = 273.15;

/// Ambient conditions chosen by the user of the bench.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Ambient {
    pub temp_c: f64,
    pub altitude_m: f64,
    pub humidity_pct: f64,
}

impl Default for Ambient {
    fn default() -> Self {
        // Moscow, autumn morning.
        Self { temp_c: 5.0, altitude_m: 156.0, humidity_pct: 75.0 }
    }
}

#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct AmbientState {
    pub p_kpa: f64,
    pub t_k: f64,
    /// Partial pressure of water vapour, kPa.
    pub p_vapour_kpa: f64,
    /// Humid-air density, kg/m³.
    pub rho: f64,
}

impl Ambient {
    /// ISA barometric formula with local temperature lapse, Buck (1996) saturation pressure,
    /// humid-air density from Dalton's law.
    pub fn state(&self) -> AmbientState {
        let t_c = self.temp_c.clamp(-40.0, 50.0);
        let h = self.altitude_m.clamp(-400.0, 5000.0);
        let t_k = t_c + KELVIN;
        let base = 1.0 - 0.0065 * h / (t_k + 0.0065 * h);
        let p_kpa = 101.325 * base.powf(5.25588);
        let p_sat = if t_c >= 0.0 {
            0.61121 * ((18.678 - t_c / 234.5) * (t_c / (257.14 + t_c))).exp()
        } else {
            0.61115 * ((23.036 - t_c / 333.7) * (t_c / (279.82 + t_c))).exp()
        };
        let p_v = (self.humidity_pct.clamp(0.0, 100.0) / 100.0) * p_sat;
        let p_d = p_kpa - p_v;
        let rho = (p_d * 1e3) / (R_AIR * t_k) + (p_v * 1e3) / (R_VAPOUR * t_k);
        AmbientState { p_kpa, t_k, p_vapour_kpa: p_v, rho }
    }
}

/// Isentropic orifice mass flow (kg/s) through effective area `cda` (Cd·A, m²).
/// Positive from `p_up` to `p_dn`; reverses sign when `p_dn > p_up` (using downstream temperature).
pub fn orifice_flow(cda: f64, p_up: f64, t_up: f64, p_dn: f64, t_dn: f64, gamma: f64) -> f64 {
    if cda <= 0.0 || p_up == p_dn {
        return 0.0;
    }
    let (sign, pu, tu, pd) = if p_up >= p_dn { (1.0, p_up, t_up, p_dn) } else { (-1.0, p_dn, t_dn, p_up) };
    if pu <= 0.0 {
        return 0.0;
    }
    let pr = (pd / pu).clamp(0.0, 1.0);
    let (pr_crit, psi_crit) = if gamma == GAMMA_AIR {
        (0.528_281_787_717_174_2, 0.684_731_458_263_405_7)
    } else if gamma == GAMMA_EXH {
        (0.540_374_195_362_524_3, 0.672_589_062_225_682_1)
    } else {
        let b = 2.0 / (gamma + 1.0);
        (b.powf(gamma / (gamma - 1.0)), gamma.sqrt() * b.powf((gamma + 1.0) / (2.0 * (gamma - 1.0))))
    };
    let psi = if pr <= pr_crit {
        psi_crit
    } else {
        let p_g = pr.powf(1.0 / gamma);
        (2.0 * gamma / (gamma - 1.0) * p_g * (p_g - pr)).max(0.0).sqrt()
    };
    sign * cda * pu / (R_AIR * tu.max(150.0)).sqrt() * psi
}

/// SAE 5W-40 kinematic viscosity (cSt), Vogel–Fulcher–Tammann fit:
/// 12.9 cSt @100 °C, 87 cSt @40 °C.
pub fn oil_viscosity_cst(t_c: f64) -> f64 {
    let t = t_c.clamp(-40.0, 180.0);
    0.068 * (1180.0 / (t + 125.0)).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn isa_and_density() {
        let s = Ambient { temp_c: 15.0, altitude_m: 0.0, humidity_pct: 0.0 }.state();
        assert!((s.p_kpa - 101.325).abs() < 1e-9);
        assert!((s.rho - 1.2250).abs() < 2e-3, "rho {}", s.rho);
        let m = Ambient { temp_c: 15.0, altitude_m: 156.0, humidity_pct: 0.0 }.state();
        assert!((m.p_kpa - 99.47).abs() < 0.05, "p {}", m.p_kpa);
    }

    #[test]
    fn orifice_choked_and_reverse() {
        // Choked: ṁ = CdA·p/√(RT)·0.6847
        let m = orifice_flow(1e-4, 200e3, 300.0, 50e3, 300.0, GAMMA_AIR);
        let expect = 1e-4 * 200e3 / (R_AIR * 300.0).sqrt() * 0.68473;
        assert!((m - expect).abs() / expect < 1e-3);
        let r = orifice_flow(1e-4, 50e3, 300.0, 200e3, 300.0, GAMMA_AIR);
        assert!((r + expect).abs() / expect < 1e-3);
        assert_eq!(orifice_flow(1e-4, 1e5, 300.0, 1e5, 300.0, GAMMA_AIR), 0.0);
    }

    #[test]
    fn viscosity_points() {
        assert!((oil_viscosity_cst(100.0) - 12.9).abs() < 0.3);
        assert!((oil_viscosity_cst(40.0) - 87.0).abs() < 3.0);
    }
}
