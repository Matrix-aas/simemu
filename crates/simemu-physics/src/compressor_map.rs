//! Table compressor maps taken from an engine calibration's own turbocharger model — derived from the firmware's
//! model of the part, not a supplier map.
//!
//! SIMOS18.1 models the compressor (FR 10.9 "Turbocharger compressor model", ACTION_CHRG_NTcha; SC8O40 code
//! `0x800C316C`, inputs prepared at `0x800C34C4`, VERIFIED-S by disassembly):
//! * volumetric flow upstream of the compressor `FLOW_CMPR_UP` = air mass flow / inlet density (m³/s);
//! * `FAC_TIA_ABSV_CMPR_UP` = √(T_ref / T₁) (cubic fit of the root in the code), T_ref = `c_tia_absv_cha_up_ref`
//!   293.0 K (SC8O40 @0x8080388A);
//! * standardised flow `FLOW_CMPR_UP_RED` = flow · FAC, standardised speed `N_TCHA_STND` = `ip_n_tcha_stnd`(flow_red,
//!   PQ_CMPR), shaft speed `N_TCHA` = N_TCHA_STND / FAC;
//! * the efficiency of the temperature rise, ΔT = T₁·(PQ^0.4/1.4 − 1)/η, η = `ip_fac_eff_cmpr`(PQ, flow_red).
//!
//! Map storage (DAMOS "columnDir": index = x·ny + y) VERIFIED-T: the firmware's `N_TCHA_STND` read from its RAM
//! (0xD0015D12) equals the bilinear lookup below at its own flow_red / PQ (168 097 against 168 092 rpm,
//! 159 332 against 159 336 rpm on the `octavia_fl` build, `tune_check` trace 2026-11-03).
//!
//! The plant needs the inverse — flow at a pressure ratio and shaft speed: along the speed line the speed at a fixed
//! pressure ratio first falls with flow (left of the line's peak, the surge side) and then rises; the plant runs on
//! the rising branch, a speed below the branch's minimum cannot hold the pressure ratio (stall / back-flow).

/// Compressor map of the firmware's model.
#[derive(Debug)]
pub struct CompressorMap {
    pub source: &'static str,
    /// Reference temperature of the standardisation, K.
    pub t_ref_k: f64,
    /// Standardised volumetric flow axis, m³/s.
    pub q_axis: [f64; 12],
    /// Pressure-ratio axis (outlet / inlet, total).
    pub pr_axis: [f64; 12],
    /// Standardised speed, rpm: `n_std[q][pr]`.
    pub n_std: [[f64; 12]; 12],
    pub eta_pr_axis: [f64; 12],
    pub eta_q_axis: [f64; 12],
    /// Isentropic efficiency of the temperature rise: `eta[pr][q]`.
    pub eta: [[f64; 12]; 12],
}

/// Index and fraction of `x` in an ascending axis, clamped to the ends (as the firmware's map lookup).
fn locate(axis: &[f64], x: f64) -> (usize, f64) {
    let n = axis.len();
    if x <= axis[0] {
        return (0, 0.0);
    }
    if x >= axis[n - 1] {
        return (n - 2, 1.0);
    }
    let i = axis.windows(2).position(|w| x < w[1]).unwrap_or(n - 2);
    (i, (x - axis[i]) / (axis[i + 1] - axis[i]))
}

/// Lowest efficiency the plant uses: the map's corner at PR → 1 and zero flow goes down to 0.008 (no pressure rise, no
/// work), which would make the outlet temperature of the first small pressure rise meaningless. UNVERIFIED floor.
const ETA_MIN: f64 = 0.25;
/// Flow past the last axis point: extrapolated along the last segment of the speed line up to this multiple of it
/// (the firmware clamps there; a real compressor chokes). UNVERIFIED.
const Q_EXTRAP_MAX: f64 = 1.25;

impl CompressorMap {
    /// Standardised speed at standardised flow and pressure ratio (the firmware's lookup).
    pub fn n_std_at(&self, q_std: f64, pr: f64) -> f64 {
        let (i, fi) = locate(&self.q_axis, q_std);
        let (j, fj) = locate(&self.pr_axis, pr);
        let z = |a: usize, b: usize| self.n_std[a][b];
        let lo = z(i, j) + (z(i, j + 1) - z(i, j)) * fj;
        let hi = z(i + 1, j) + (z(i + 1, j + 1) - z(i + 1, j)) * fj;
        lo + (hi - lo) * fi
    }

    /// Standardised flow, m³/s, at pressure ratio `pr` (clamped to the axis) and standardised speed `n_std` on the
    /// rising (stable) branch of the speed line; `None` when the speed cannot hold this pressure ratio.
    pub fn q_std(&self, pr: f64, n_std: f64) -> Option<f64> {
        let (j, fj) = locate(&self.pr_axis, pr);
        let line: [f64; 12] = std::array::from_fn(|i| self.n_std[i][j] + (self.n_std[i][j + 1] - self.n_std[i][j]) * fj);
        let i_min = (0..12).fold(0, |m, i| if line[i] < line[m] { i } else { m });
        if n_std < line[i_min] {
            return None;
        }
        for i in i_min..11 {
            if n_std <= line[i + 1] {
                let d = line[i + 1] - line[i];
                let f = if d > 0.0 { (n_std - line[i]) / d } else { 0.0 };
                return Some(self.q_axis[i] + f * (self.q_axis[i + 1] - self.q_axis[i]));
            }
        }
        let slope = (self.q_axis[11] - self.q_axis[10]) / (line[11] - line[10]).max(1.0);
        Some((self.q_axis[11] + (n_std - line[11]) * slope).min(Q_EXTRAP_MAX * self.q_axis[11]))
    }

    /// Pressure ratio the speed holds at zero flow (the left end of its speed line), for the surge-margin readout.
    pub fn pr_zero_flow(&self, n_std: f64) -> f64 {
        let col: [f64; 12] = std::array::from_fn(|j| self.n_std[0][j]);
        let (mut j, mut f) = (0, 0.0);
        if n_std >= col[11] {
            (j, f) = (10, 1.0);
        } else if n_std > col[0] {
            j = (0..11).find(|&k| n_std <= col[k + 1]).unwrap_or(10);
            f = (n_std - col[j]) / (col[j + 1] - col[j]);
        }
        self.pr_axis[j] + f * (self.pr_axis[j + 1] - self.pr_axis[j])
    }

    /// Isentropic efficiency at pressure ratio and standardised flow.
    pub fn eta(&self, pr: f64, q_std: f64) -> f64 {
        let (i, fi) = locate(&self.eta_pr_axis, pr);
        let (j, fj) = locate(&self.eta_q_axis, q_std);
        let z = |a: usize, b: usize| self.eta[a][b];
        let lo = z(i, j) + (z(i, j + 1) - z(i, j)) * fj;
        let hi = z(i + 1, j) + (z(i + 1, j + 1) - z(i + 1, j)) * fj;
        (lo + (hi - lo) * fi).max(ETA_MIN)
    }
}

/// IHI IS12 (EA888 Gen3 1.8 CJSA) as the stock CJSA calibration SC8O40 (`images/8V0906264H__0005.bin`) models it:
/// `ip_n_tcha_stnd` @0x8081A7C4 (flow axis @0x80819606, PR axis @0x8081969C), `ip_fac_eff_cmpr` @0x80819B20 (PR axis
/// @0x80819682, flow axis @0x808196F8), `c_tia_absv_cha_up_ref` @0x8080388A (VERIFIED-S, values as stored).
pub const IS12_SC8O40: CompressorMap = CompressorMap {
    source: "SC8O40 ip_n_tcha_stnd / ip_fac_eff_cmpr (firmware model of the IS12)",
    t_ref_k: 293.005,
    q_axis: [0.000000, 0.025024, 0.040009, 0.050018, 0.065002, 0.080017, 0.095001, 0.115021, 0.125000, 0.140015, 0.154999, 0.170013],
    pr_axis: [1.000000, 1.100098, 1.300049, 1.600098, 1.750000, 1.900146, 2.050049, 2.199951, 2.400146, 2.600098, 3.000000, 3.600098],
    n_std: [
        [12841.8, 49005.1, 81213.4, 109838.9, 120843.5, 132385.3, 140618.9, 153002.9, 169781.5, 183020.0, 209491.0, 240960.7],
        [23913.6, 52966.3, 85467.5, 110345.5, 121820.1, 132434.1, 142297.4, 152081.3, 168383.8, 180493.2, 204705.8, 233850.1],
        [33337.4, 61401.4, 87493.9, 114868.2, 125463.9, 134545.9, 145025.6, 152386.5, 166784.7, 178210.4, 201062.0, 229785.2],
        [34918.2, 69909.7, 91168.2, 118109.1, 127343.7, 136956.8, 147149.7, 155767.8, 167480.5, 178259.3, 199816.9, 226556.4],
        [38964.8, 81640.6, 97515.9, 120880.1, 131359.9, 138317.9, 147204.6, 156390.4, 167211.9, 177331.5, 197955.3, 222021.5],
        [45349.1, 84985.4, 106274.4, 124322.5, 133178.7, 141583.3, 149041.7, 157708.7, 167834.5, 176757.8, 196301.3, 217926.0],
        [57287.6, 92486.6, 111340.3, 133075.0, 138256.8, 145294.2, 151532.0, 159259.0, 167150.9, 177020.3, 195806.9, 214331.1],
        [64105.2, 100982.7, 118847.7, 138201.9, 145105.0, 152771.0, 156622.3, 161908.0, 169531.2, 177410.9, 193481.4, 212164.3],
        [88439.9, 122747.8, 128649.9, 144958.5, 150244.1, 156274.4, 162292.5, 166961.7, 171868.9, 177642.8, 193621.8, 211529.5],
        [108923.3, 132452.4, 138214.1, 148999.0, 154022.2, 159472.7, 164917.0, 171057.1, 176818.8, 180529.8, 194342.0, 211602.8],
        [126385.5, 143823.2, 147308.3, 157446.3, 162506.1, 167578.1, 174731.4, 179876.7, 185137.9, 190387.0, 200891.1, 216656.5],
        [129046.6, 146313.5, 154827.9, 167535.4, 173889.2, 180249.0, 188317.9, 193389.9, 197528.1, 201654.1, 209918.2, 222314.5],
    ],
    eta_pr_axis: [1.000000, 1.199951, 1.449951, 1.699951, 1.899902, 2.000000, 2.199951, 2.399902, 2.600098, 2.800049, 3.800049, 4.000000],
    eta_q_axis: [0.000000, 0.029999, 0.050018, 0.075012, 0.100006, 0.119995, 0.140015, 0.160004, 0.174988, 0.200012, 0.225006, 0.250000],
    eta: [
        [0.007812, 0.429688, 0.582031, 0.574219, 0.519531, 0.429688, 0.191406, 0.121094, 0.121094, 0.121094, 0.121094, 0.121094],
        [0.250000, 0.628906, 0.656250, 0.656250, 0.644531, 0.625000, 0.550781, 0.402344, 0.289062, 0.289062, 0.289062, 0.289062],
        [0.300781, 0.687500, 0.738281, 0.734375, 0.714844, 0.652344, 0.585938, 0.500000, 0.433594, 0.414062, 0.414062, 0.414062],
        [0.328125, 0.703125, 0.777344, 0.808594, 0.777344, 0.726562, 0.640625, 0.601562, 0.585938, 0.558594, 0.558594, 0.558594],
        [0.347656, 0.671875, 0.769531, 0.820312, 0.820312, 0.796875, 0.714844, 0.683594, 0.683594, 0.679688, 0.679688, 0.675781],
        [0.343750, 0.640625, 0.750000, 0.828125, 0.835938, 0.828125, 0.753906, 0.718750, 0.718750, 0.714844, 0.714844, 0.710938],
        [0.339844, 0.585938, 0.707031, 0.816406, 0.859375, 0.843750, 0.820312, 0.781250, 0.781250, 0.777344, 0.777344, 0.773438],
        [0.335938, 0.542969, 0.667969, 0.804688, 0.847656, 0.851562, 0.843750, 0.832031, 0.828125, 0.816406, 0.804688, 0.796875],
        [0.332031, 0.519531, 0.628906, 0.773438, 0.824219, 0.839844, 0.835938, 0.832031, 0.828125, 0.820312, 0.816406, 0.808594],
        [0.328125, 0.496094, 0.601562, 0.742188, 0.800781, 0.828125, 0.832031, 0.828125, 0.828125, 0.824219, 0.824219, 0.820312],
        [0.304688, 0.382812, 0.468750, 0.593750, 0.691406, 0.765625, 0.820312, 0.820312, 0.820312, 0.824219, 0.828125, 0.832031],
        [0.300781, 0.359375, 0.441406, 0.562500, 0.667969, 0.753906, 0.816406, 0.820312, 0.820312, 0.824219, 0.828125, 0.832031],
    ],
};

#[cfg(test)]
mod tests {
    use super::*;

    /// The firmware's own outputs read from its RAM (`octavia_fl` build, stock SC8O40 pull): flow_red / PQ → N_TCHA_STND.
    #[test]
    fn lookup_matches_the_firmware() {
        for (q, pr, n_fw) in [(0.1333, 2.169, 168_097.0), (0.1289, 1.957, 159_332.0)] {
            let n = IS12_SC8O40.n_std_at(q, pr);
            assert!((n - n_fw).abs() < 50.0, "{q} {pr}: {n} vs firmware {n_fw}");
        }
    }

    /// The inverse returns the flow the forward lookup started from (stable branch), and a slower shaft gives less flow.
    #[test]
    fn inverse_is_consistent_and_monotonic() {
        let m = &IS12_SC8O40;
        for pr in [1.0, 1.4, 1.9, 2.2, 2.6, 3.2] {
            let mut last = 0.0;
            for k in 0..60 {
                let n = 60_000.0 + 3_000.0 * k as f64;
                let Some(q) = m.q_std(pr, n) else { continue };
                assert!(q >= last - 1e-9, "PR {pr}: flow falls with speed at {n}");
                last = q;
                if q > 0.0 && q < m.q_axis[11] {
                    assert!((m.n_std_at(q, pr) - n).abs() < 1.0, "PR {pr} n {n}: q {q} → {}", m.n_std_at(q, pr));
                }
            }
        }
        assert!(m.q_std(3.6, 150_000.0).is_none(), "150 krpm cannot hold PR 3.6");
        assert!(m.eta(2.2, 0.1) > 0.8 && m.eta(1.0, 0.0) >= ETA_MIN);
    }
}
