//! Exhaust oxygen sensors: gas transport from the exhaust manifold to the sensors, the upstream wide-band
//! (planar limiting-current, LSU class) cell, the catalyst's oxygen storage and the downstream binary (Nernst)
//! cell, each with its ceramic/heater thermal model. Hardware physics only: the heater power comes from the ECU's
//! PWM (`EcuOutputs::lambda_heater_duty`), the cell state goes to the ECU's sensor IC (ATIC142 on SIMOS18.1).
//!
//! Sources and evidence (labels as in docs/SIMOS18_GPTA_IO_RE.md §0):
//! * Ri(T) of both cells: the inverse of the firmware's own conversions `ip_ttip_mes_lsl` @0x8081D00E and
//!   `ip_ttip_mes_ls` @0x8081CFF8 (SC8S50; VERIFIED-S as the firmware's expectation). The linear curve reproduces
//!   the Bosch LSU 4.9 operating points (300 Ω @ 780 °C, 150 Ω @ 888 °C, 80 Ω @ 1030 °C — Bosch Motorsport
//!   data sheets 69034379 / 69070987, product page "Lambda Sensor LSU 4.9"): 303 / 148 / 88 Ω.
//! * Pump current vs λ: the inverse of the firmware's characteristic `ip_lamb_lsl_inv` @0x8081CC24 with its
//!   offset `c_icp_lsl_lamb_cor_lsl` 0.0202 mA (FR 22.10.7, p.5993: the line was taken with 20 µA reference pump
//!   current), temperature correction `ip_fac_cor_iplsl_ttip_lsl` @0x80803E84 and exhaust-pressure corrections
//!   `ip_fac_cor_iplsl_prs_ex` / `_afr_prs_ex` (factory calibrations, see `FAC_P_HPA`) at the exhaust pressure upstream
//!   of the turbine, where EA888 Gen 3 has the probe (SSP 606): a nominal sensor is one the firmware
//!   reads correctly (VERIFIED-S); sensor-to-sensor spread is not modelled (UNVERIFIED).
//! * Heater: cold resistance 3.2 Ω, 7.5 W at the 7.5 V nominal operating point (LSU 4.9 data sheet; Saab WIS
//!   "preheated oxygen sensor 3.2 ± 0.8 Ω at 20 °C") → R(T) linear (PTC platinum track), α from the two points.
//!   Heat capacity and gas conductance: UNVERIFIED knobs ([`CellHw`]), sized so that the firmware's own heater
//!   model fits: steady state at the 785 °C set-point needs ≈ 8.3 V effective at idle with 465 °C gas
//!   (`ip_v_efc_ctl_lsh_lsl` @0x8081D078), and the 12 V warm-up phase (`ip_v_efc_rise_lsh_lsl`) reaches the
//!   640 °C switch-over (`c_ttip_max_v_efc_max_lsh_lsl`) in a few seconds.
//! * Nernst EMF vs λ (binary cell, and the wide-band cell's reference-to-exhaust voltage): switching
//!   characteristic of a zirconia cell, ≈ 0.85 V rich / 0.05 V lean, steep at λ = 1 (Bosch LSF 4.2 class data
//!   sheet values; UNVERIFIED table [`EMF_TABLE`]).
//! * Transport: plug flow through the exhaust volume between the exhaust valves and the upstream sensor (mass
//!   based, so the delay follows the flow), then the cell's diffusion response. Volume and response time
//!   UNVERIFIED ([`TransportHw`]). The catalyst is an oxygen store between the two sensors (UNVERIFIED capacity).
//!
//! Real time: everything is table interpolation plus a handful of flops per 1 ms plant step, fixed-size ring
//! buffer, no allocation.

/// Interpolation in a monotonic table (x ascending), clamped at both ends.
fn interp(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    if x <= xs[0] {
        return ys[0];
    }
    for i in 1..xs.len() {
        if x <= xs[i] {
            let f = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + f * (ys[i] - ys[i - 1]);
        }
    }
    ys[ys.len() - 1]
}

/// `ip_lamb_lsl_inv` @0x8081CC24 (SC8S50): corrected pump current (mA) → 1/λ.
const IPLSL_MA: [f64; 16] = [-1.0502, -0.8768, -0.695, -0.5351, -0.4551, -0.3852, -0.3198, -0.2572, -0.196, -0.1399, -0.08, -0.02, 0.0169, 0.1061, 0.1311, 1.4];
const LAMB_INV: [f64; 16] = [1.538, 1.429, 1.333, 1.25, 1.212, 1.176, 1.143, 1.111, 1.081, 1.053, 1.026, 1.0, 0.976, 0.909, 0.889, 0.0];
/// `c_icp_lsl_lamb_cor_lsl` @0x808040C8 (mA).
const ICP_LAMB_COR_MA: f64 = 0.0202;
/// `ip_fac_cor_iplsl_ttip_lsl` @0x80803E84: tip temperature (°C) → the firmware's gain correction.
const FAC_T_C: [f64; 8] = [485.0, 585.0, 685.0, 785.0, 885.0, 985.0, 1085.0, 1185.0];
const FAC_T: [f64; 8] = [1.0927, 1.0609, 1.03, 1.0, 0.97, 0.9409, 0.9127, 0.8853];
/// `ip_fac_cor_iplsl_prs_ex` (lean) / `ip_fac_cor_iplsl_afr_prs_ex` (rich): exhaust pressure at the probe (hPa) →
/// correction. The factory calibrations agree byte for byte — SC8S50 `8V0906264K__0003` @0x80803E74 / 0x80803E5C, SC8O40
/// `264H__0005` and `264J__0003` @0x80803E6C / 0x80803E54, SC8H83 `264F__0003` @0x808039D4 / 0x808039BC
/// (VERIFIED-S); only the donor bench dump's calibration 259J (not a factory FRF image, AGENTS.md §3.1) has another pair
/// (axis to 3800 hPa, e.g. rich 0.7351 at 2000 hPa against 0.67), which the plant used before 2026-11-03.
const FAC_P_HPA: [f64; 8] = [600.074, 800.071, 1013.003, 1200.065, 1400.062, 1600.059, 2000.054, 2500.046];
const FAC_P_LEAN: [f64; 8] = [1.26, 1.12, 1.0, 0.91, 0.825, 0.76, 0.64, 0.55];
const FAC_P_RICH: [f64; 8] = [1.25, 1.11, 1.0, 0.91, 0.84, 0.78, 0.67, 0.57];

/// `ip_ttip_mes_lsl` @0x8081D00E as (T °C ascending, ln Ri): the linear sensor's Ri(T).
const LSL_T_C: [f64; 8] = [528.0, 554.0, 598.0, 671.0, 730.0, 810.0, 916.0, 1198.0];
const LSL_RI: [f64; 8] = [6000.0, 3800.0, 2200.0, 900.0, 449.0, 240.0, 124.0, 53.0];
/// `ip_ttip_mes_ls` @0x8081CFF8: the binary sensor's Ri(T) (the 300 °C / 65532 Ω end point excluded: a
/// "cold" marker, not a measurement).
const LS_T_C: [f64; 7] = [610.0, 622.0, 660.0, 720.0, 730.0, 840.0, 900.0];
const LS_RI: [f64; 7] = [2000.0, 1000.0, 650.0, 360.0, 330.0, 150.0, 110.0];

/// Zirconia cell EMF (V) vs λ, reference air vs equilibrated exhaust gas at ≈ 700 °C. UNVERIFIED (LSF 4.2 class).
pub const EMF_TABLE: ([f64; 12], [f64; 12]) = (
    [0.80, 0.90, 0.95, 0.98, 0.99, 0.995, 1.0, 1.005, 1.01, 1.02, 1.05, 1.20],
    [0.88, 0.86, 0.84, 0.82, 0.80, 0.74, 0.45, 0.16, 0.10, 0.07, 0.05, 0.03],
);

/// Ri (Ω) of a cell at `t_c` from its (T, Ri) table: log-linear between the points (Arrhenius-like), Arrhenius
/// extrapolation with the end slopes outside, capped at 1 GΩ (open circuit for every reading).
fn ri_from_table(ts: &[f64], ris: &[f64], t_c: f64) -> f64 {
    let n = ts.len();
    let (i0, i1) = if t_c < ts[0] {
        (0, 1)
    } else if t_c >= ts[n - 1] {
        (n - 2, n - 1)
    } else {
        let i = (1..n).find(|&i| t_c < ts[i]).unwrap_or(n - 1);
        (i - 1, i)
    };
    // ln Ri linear in 1/T (K): exact through both table points
    let (a, b) = (1.0 / (ts[i0] + 273.15), 1.0 / (ts[i1] + 273.15));
    let (la, lb) = (ris[i0].ln(), ris[i1].ln());
    let x = 1.0 / (t_c.max(-50.0) + 273.15);
    (la + (lb - la) * (x - a) / (b - a)).min(1e9_f64.ln()).exp()
}

/// Electrical/thermal hardware of one sensor element. UNVERIFIED values except where noted.
#[derive(Debug, Clone, Copy)]
pub struct CellHw {
    /// Heater resistance at 20 °C (Ω) and its temperature coefficient (1/K).
    pub heater_r20_ohm: f64,
    pub heater_alpha: f64,
    /// Heat capacity of the ceramic tip (J/K).
    pub heat_capacity_j_k: f64,
    /// Thermal conductance tip ↔ exhaust gas at the reference exhaust mass flow (W/K); scales with √flow
    /// (forced convection on the protection tube, Nu ∝ Re^0.5).
    pub g_gas_w_k: f64,
    pub g_ref_flow_kg_s: f64,
    /// Conductance tip ↔ sensor housing / pipe wall (W/K).
    pub g_mount_w_k: f64,
}

impl CellHw {
    /// Wide-band LSU class: 3.2 Ω cold → 7.5 Ω at 780 °C (7.5 V / 7.5 W, data sheet).
    pub fn lsu() -> Self {
        Self { heater_r20_ohm: 3.2, heater_alpha: (7.5 / 3.2 - 1.0) / 760.0, heat_capacity_j_k: 0.25, g_gas_w_k: 0.022, g_ref_flow_kg_s: 0.005, g_mount_w_k: 0.003 }
    }
    /// Binary (finger/planar LSF class) post-catalyst sensor: same heater class, set-point 730 °C.
    pub fn lsf() -> Self {
        Self { heater_r20_ohm: 3.5, heater_alpha: (7.5 / 3.5 - 1.0) / 700.0, heat_capacity_j_k: 0.3, g_gas_w_k: 0.018, g_ref_flow_kg_s: 0.005, g_mount_w_k: 0.003 }
    }
    pub fn heater_r(&self, t_c: f64) -> f64 {
        self.heater_r20_ohm * (1.0 + self.heater_alpha * (t_c - 20.0)).max(0.3)
    }
}

/// Exhaust path between the exhaust valves, the upstream sensor, the catalyst and the downstream sensor.
#[derive(Debug, Clone, Copy)]
pub struct TransportHw {
    /// Gas volume from the exhaust valves to the upstream sensor (m³), at the gas state of the probe.
    pub volume_up_m3: f64,
    /// Response time of the wide-band cell (diffusion barrier + cavity), s.
    pub tau_cell_s: f64,
    /// Oxygen storage capacity of the warm catalyst (kg O₂) and its light-off gas temperature (K).
    pub osc_kg: f64,
    pub cat_light_off_k: f64,
    /// Mixing time of the post-catalyst gas at the downstream sensor (s).
    pub tau_down_s: f64,
}

impl Default for TransportHw {
    /// UNVERIFIED: 0.8 L of transport between the engine model's manifold volume and the probe — chosen when the probe
    /// was taken to sit after the turbine; with the probe in the turbine inlet (SSP 606) the path is shorter, but no
    /// volume / delay data was found and a 0.2 L trial changed the idle λ loop enough to fail the DQ250 4319 basic
    /// setting 0x0380 (2026-11-03), so it stays; LSU response ≈ 50 ms; warm close-coupled TWC ≈ 0.3 g O₂ storage,
    /// light-off ≈ 300 °C.
    fn default() -> Self {
        Self { volume_up_m3: 0.8e-3, tau_cell_s: 0.05, osc_kg: 0.3e-3, cat_light_off_k: 573.0, tau_down_s: 0.3 }
    }
}

/// Exhaust gas leaving the manifold (input of [`ExhaustSensors::step`]). The wide-band probe of EA888 Gen 3 sits
/// upstream of the turbine (Audi SSP 606 / eSSP 920243: "oxygen sensor upstream of the turbine", G39 / GX10), and the
/// firmware corrects its pump current with the exhaust pressure there (FR 22.48.9: `PRS_EX_LSL = PRS_EX` when
/// `LC_POSN_LSL_PRE_TCHA` = 1, 22.10.6 / 22.10.7): `temp_k` / `pressure_pa` are the turbine-inlet gas, `temp_cat_k`
/// the turbine-outlet gas the catalyst and the downstream probe see.
#[derive(Debug, Clone, Copy)]
pub struct ExhaustGas {
    pub lambda: f64,
    pub mass_flow_kg_s: f64,
    pub temp_k: f64,
    pub pressure_pa: f64,
    pub temp_cat_k: f64,
}

const RING: usize = 2048;
/// Mass fraction of O₂ in air.
const O2_IN_AIR: f64 = 0.2314;
const AFR_STOICH: f64 = 14.7;
const R_EXH: f64 = 287.0;

/// One heated sensor element.
#[derive(Debug, Clone, Copy)]
pub struct Cell {
    pub hw: CellHw,
    /// Ceramic tip temperature (°C).
    pub t_c: f64,
    pub heater_w: f64,
}

impl Cell {
    fn new(hw: CellHw, t_c: f64) -> Self {
        Self { hw, t_c, heater_w: 0.0 }
    }

    /// Heater PWM `duty` from the ECU's low-side switch on the battery; gas at `gas_k` with `flow` kg/s, housing at
    /// `wall_c`.
    fn step(&mut self, duty: Option<f64>, v_batt: f64, gas_k: f64, flow: f64, wall_c: f64, dt: f64) {
        let h = self.hw;
        self.heater_w = duty.unwrap_or(0.0).clamp(0.0, 1.0) * v_batt * v_batt / h.heater_r(self.t_c);
        let g_gas = h.g_gas_w_k * (flow.max(0.0) / h.g_ref_flow_kg_s).sqrt().max(0.1);
        let q = self.heater_w - g_gas * (self.t_c - (gas_k - 273.15)) - h.g_mount_w_k * (self.t_c - wall_c);
        self.t_c += q * dt / h.heat_capacity_j_k;
    }
}

/// Wide-band cell state at its terminals (what the sensor IC sees).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct WidebandTerminals {
    /// Limiting pump current that holds the cavity at the Nernst set-point (mA, > 0 lean).
    pub ip_lim_ma: f64,
    /// Nernst voltage reference vs exhaust gas (V).
    pub e_exh_v: f64,
    /// Nernst / pump cell internal resistance (Ω).
    pub ri_ohm: f64,
    pub rp_ohm: f64,
}

/// Binary cell state at its terminals.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BinaryTerminals {
    pub e_v: f64,
    pub ri_ohm: f64,
}

#[derive(Debug, Clone)]
pub struct ExhaustSensors {
    pub hw: TransportHw,
    pub up: Cell,
    pub down: Cell,
    /// Cumulative exhaust mass (kg) and λ at the valve exit, ring buffer.
    ring_m: [f64; RING],
    ring_l: [f64; RING],
    head: usize,
    tail: usize,
    m_total: f64,
    /// λ of the gas at the upstream sensor (plug flow output) and as seen through the cell's diffusion response.
    pub lambda_up_gas: f64,
    pub lambda_up_cell: f64,
    /// Oxygen stored in the catalyst (kg) and λ at the downstream sensor.
    pub osc_kg: f64,
    pub lambda_down: f64,
    pub p_exh_pa: f64,
}

impl ExhaustSensors {
    pub fn new(ambient_c: f64) -> Self {
        Self {
            hw: TransportHw::default(),
            up: Cell::new(CellHw::lsu(), ambient_c),
            down: Cell::new(CellHw::lsf(), ambient_c),
            ring_m: [0.0; RING],
            ring_l: [10.0; RING],
            head: 0,
            tail: 0,
            m_total: 0.0,
            lambda_up_gas: 10.0,
            lambda_up_cell: 10.0,
            osc_kg: 0.5 * TransportHw::default().osc_kg,
            lambda_down: 10.0,
            p_exh_pa: 101_325.0,
        }
    }

    /// Exhaust gas at the valves → sensors, heater duties from the ECU (`None` = switch off). `wall_c`: exhaust
    /// pipe / sensor boss temperature.
    pub fn step(&mut self, gas: &ExhaustGas, heater_duty: [Option<f64>; 2], v_batt: f64, wall_c: f64, dt: f64) {
        let flow = gas.mass_flow_kg_s.max(0.0);
        // plug flow: the gas now at the sensor left the valves when the cumulative mass was m_total − m_volume
        let dm = flow * dt;
        if dm > 1e-9 {
            self.m_total += dm;
            self.head = (self.head + 1) % RING;
            if self.head == self.tail {
                self.tail = (self.tail + 1) % RING;
            }
            self.ring_m[self.head] = self.m_total;
            self.ring_l[self.head] = gas.lambda.min(10.0);
        }
        let rho = gas.pressure_pa / (R_EXH * gas.temp_k.max(250.0));
        let m_lag = self.m_total - rho * self.hw.volume_up_m3;
        while self.tail != self.head && self.ring_m[(self.tail + 1) % RING] <= m_lag {
            self.tail = (self.tail + 1) % RING;
        }
        self.lambda_up_gas = self.ring_l[self.tail];
        self.lambda_up_cell += (self.lambda_up_gas - self.lambda_up_cell) * (dt / self.hw.tau_cell_s).min(1.0);
        self.p_exh_pa = gas.pressure_pa;

        // catalyst oxygen storage: excess O₂ (lean) fills it, deficit (rich) empties it; past the limits the
        // upstream composition breaks through
        let o2_excess = flow * O2_IN_AIR * (1.0 - 1.0 / self.lambda_up_gas.max(0.3)) * AFR_STOICH / (AFR_STOICH + 1.0);
        let active = gas.temp_cat_k > self.hw.cat_light_off_k;
        let cap = if active { self.hw.osc_kg } else { 0.0 };
        let stored = self.osc_kg + o2_excess * dt;
        let (stored, lam_out) = if stored > cap {
            (cap, if o2_excess > 0.0 { self.lambda_up_gas } else { 1.0 })
        } else if stored < 0.0 {
            (0.0, if o2_excess < 0.0 { self.lambda_up_gas } else { 1.0 })
        } else {
            (stored, 1.0)
        };
        self.osc_kg = stored;
        self.lambda_down += (lam_out - self.lambda_down) * (dt / self.hw.tau_down_s).min(1.0);

        self.up.step(heater_duty[0], v_batt, gas.temp_k, flow, wall_c, dt);
        self.down.step(heater_duty[1], v_batt, gas.temp_cat_k - 50.0, flow, wall_c, dt);
    }

    /// Wide-band cell terminals.
    pub fn wideband(&self) -> WidebandTerminals {
        let lam = self.lambda_up_cell.max(0.3);
        let t = self.up.t_c;
        let ip_char = interp(&LAMB_INV_ASC.0, &LAMB_INV_ASC.1, 1.0 / lam) + ICP_LAMB_COR_MA;
        let hpa = self.p_exh_pa / 100.0;
        let f_p = if ip_char >= 0.0 { interp(&FAC_P_HPA, &FAC_P_LEAN, hpa) } else { interp(&FAC_P_HPA, &FAC_P_RICH, hpa) };
        let ri = ri_from_table(&LSL_T_C, &LSL_RI, t);
        WidebandTerminals {
            ip_lim_ma: ip_char / (interp(&FAC_T_C, &FAC_T, t) * f_p),
            e_exh_v: interp(&EMF_TABLE.0, &EMF_TABLE.1, lam),
            ri_ohm: ri,
            // UNVERIFIED: pump cell of similar geometry, about half the Nernst cell's resistance
            rp_ohm: 0.5 * ri,
        }
    }

    pub fn binary(&self) -> BinaryTerminals {
        BinaryTerminals { e_v: interp(&EMF_TABLE.0, &EMF_TABLE.1, self.lambda_down), ri_ohm: ri_from_table(&LS_T_C, &LS_RI, self.down.t_c) }
    }
}

/// `ip_lamb_lsl_inv` turned around: (1/λ ascending, pump current mA).
const LAMB_INV_ASC: ([f64; 16], [f64; 16]) = {
    let mut x = [0.0; 16];
    let mut y = [0.0; 16];
    let mut i = 0;
    while i < 16 {
        x[i] = LAMB_INV[15 - i];
        y[i] = IPLSL_MA[15 - i];
        i += 1;
    }
    (x, y)
};

#[cfg(test)]
mod tests {
    use super::*;

    fn run(s: &mut ExhaustSensors, lambda: f64, flow: f64, gas_k: f64, duty: [Option<f64>; 2], secs: f64) {
        let g = ExhaustGas { lambda, mass_flow_kg_s: flow, temp_k: gas_k, pressure_pa: 101_325.0, temp_cat_k: gas_k };
        for _ in 0..(secs / 1e-3) as usize {
            s.step(&g, duty, 14.0, 90.0, 1e-3);
        }
    }

    /// The firmware reads the nominal sensor back: its own conversion (FR 22.10.7 with all factors 1 at 785 °C /
    /// 1013 hPa, ICP_EFC cancelling the measured reference-pump share) applied to the plant's pump current.
    #[test]
    fn pump_current_reads_back_through_the_firmware_characteristic() {
        let mut s = ExhaustSensors::new(785.0);
        s.up.t_c = 785.0;
        s.p_exh_pa = 101_300.0;
        for lam in [0.75, 0.9, 0.97, 1.0, 1.03, 1.1, 1.3, 2.0] {
            s.lambda_up_cell = lam;
            let ip = s.wideband().ip_lim_ma;
            let lam_fw = 1.0 / interp(&IPLSL_MA, &LAMB_INV, ip - ICP_LAMB_COR_MA);
            assert!((lam_fw - lam).abs() < 2e-3 * lam, "λ {lam}: Ip {ip:.4} mA → firmware λ {lam_fw:.4}");
        }
        // at boost, upstream of the turbine: the pump current grows with the pressure and the firmware's FAC_COR_IPLSL_PRS
        // (rich / lean curve by the sign of the current, FR 22.10.6) takes it back
        s.p_exh_pa = 220_000.0;
        for lam in [0.75, 0.9, 1.1, 1.3] {
            s.lambda_up_cell = lam;
            let ip = s.wideband().ip_lim_ma;
            let f_p = interp(&FAC_P_HPA, if ip >= 0.0 { &FAC_P_LEAN } else { &FAC_P_RICH }, 2200.0);
            assert!(ip.abs() > 1.2 * (interp(&LAMB_INV_ASC.0, &LAMB_INV_ASC.1, 1.0 / lam) + ICP_LAMB_COR_MA).abs(), "λ {lam}: Ip {ip:.4} mA");
            let lam_fw = 1.0 / interp(&IPLSL_MA, &LAMB_INV, ip * f_p - ICP_LAMB_COR_MA);
            assert!((lam_fw - lam).abs() < 2e-3 * lam, "2.2 bar, λ {lam}: Ip {ip:.4} mA → firmware λ {lam_fw:.4}");
        }
        assert!(s.wideband().ip_lim_ma > 0.2, "lean → positive pump current");
    }

    /// Ri(T) inverts `ip_ttip_mes_lsl` and hits the LSU 4.9 data-sheet points.
    #[test]
    fn nernst_resistance_follows_the_firmware_curve_and_the_data_sheet() {
        for (t, ri) in LSL_T_C.iter().zip(LSL_RI) {
            assert!((ri_from_table(&LSL_T_C, &LSL_RI, *t) / ri - 1.0).abs() < 1e-9);
        }
        for (t, ri) in [(780.0, 300.0), (888.0, 150.0), (1030.0, 80.0)] {
            let m = ri_from_table(&LSL_T_C, &LSL_RI, t);
            assert!((m / ri - 1.0).abs() < 0.12, "{t} °C: {m:.0} Ω vs data sheet {ri} Ω");
        }
        assert!(ri_from_table(&LSL_T_C, &LSL_RI, 100.0) > 1e7, "cold ceramic is an insulator");
    }

    /// Heater: full warm-up voltage brings the tip past the firmware's 640 °C switch-over within seconds; the
    /// firmware's idle open-loop effective voltage (8.3 V at 465 °C gas) holds roughly the 785 °C set-point.
    #[test]
    fn heater_warms_up_and_holds_the_set_point_with_the_firmware_voltages() {
        let mut s = ExhaustSensors::new(90.0);
        let duty = |v: f64| Some((v / 14.0) * (v / 14.0));
        let mut t640 = None;
        for i in 0..20_000 {
            run(&mut s, 1.0, 0.0045, 738.0, [duty(12.0), duty(10.46)], 1e-3);
            if s.up.t_c > 640.0 {
                t640.get_or_insert(i as f64 * 1e-3);
            }
        }
        let t640 = t640.expect("warm-up");
        assert!((2.0..12.0).contains(&t640), "640 °C after {t640:.1} s");
        run(&mut s, 1.0, 0.0045, 738.0, [duty(8.33), None], 30.0);
        assert!((s.up.t_c - 785.0).abs() < 60.0, "steady state with the firmware's idle voltage: {:.0} °C", s.up.t_c);
        let p = s.up.heater_w;
        assert!((5.0..15.0).contains(&p), "heater power {p:.1} W");
    }

    /// Real-time budget: one plant step (1 ms) of both sensors + reading both cells' terminals. Prints the cost;
    /// fails only if it is grossly over budget (> 5 µs per step, 0.5 % of the 1 ms step).
    #[test]
    fn step_cost_is_negligible() {
        let mut s = ExhaustSensors::new(700.0);
        let g = ExhaustGas { lambda: 1.0, mass_flow_kg_s: 0.005, temp_k: 900.0, pressure_pa: 101_325.0, temp_cat_k: 900.0 };
        let n = 200_000;
        let t0 = std::time::Instant::now();
        let mut acc = 0.0;
        for i in 0..n {
            s.step(&g, [Some(0.5), Some(0.3)], 14.0, 90.0, 1e-3);
            let (w, b) = (s.wideband(), s.binary());
            acc += w.ip_lim_ma + b.e_v + (i & 1) as f64;
        }
        let ns = t0.elapsed().as_nanos() as f64 / n as f64;
        eprintln!("exhaust sensors: {ns:.0} ns per 1 ms plant step ({:.3} % of real time) [{acc:.0}]", ns / 1e4);
        assert!(ns < 5_000.0, "{ns} ns");
    }

    /// Transport delay follows the exhaust mass flow; the catalyst stores oxygen before a lean step breaks through.
    #[test]
    fn plug_flow_delay_and_catalyst_storage() {
        let mut s = ExhaustSensors::new(700.0);
        run(&mut s, 1.0, 0.005, 900.0, [None, None], 5.0); // purge the air the pipe started with
        let g = ExhaustGas { lambda: 1.2, mass_flow_kg_s: 0.005, temp_k: 900.0, pressure_pa: 101_325.0, temp_cat_k: 900.0 };
        let delay = (s.hw.volume_up_m3 * 101_325.0 / (287.0 * 900.0)) / 0.005;
        let mut t_up = None;
        let mut t_down = None;
        for i in 0..5000 {
            s.step(&g, [None, None], 14.0, 90.0, 1e-3);
            let t = i as f64 * 1e-3;
            if s.lambda_up_gas > 1.1 {
                t_up.get_or_insert(t);
            }
            if s.lambda_down > 1.1 {
                t_down.get_or_insert(t);
            }
        }
        let (t_up, t_down) = (t_up.unwrap(), t_down.unwrap());
        assert!((t_up - delay).abs() < 0.01, "plug flow {t_up:.3} s vs {delay:.3} s");
        assert!(t_down > t_up + 0.5, "oxygen storage delays the breakthrough: {t_down:.2} s");
    }
}
