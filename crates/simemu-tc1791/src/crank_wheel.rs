//! 60-2 crank wheel + intake/exhaust cam Hall sensor edge generator for the SIMOS18.1 GPTA inputs.
//!
//! Angle convention: crank angle `theta` in degrees crankshaft (°CRK), continuous (not wrapped), 0 = TDC of
//! cylinder 1 (TDC0), 720° per engine cycle. Edges are produced on GPTA0 input lines (kernel 0):
//! crank IN24 (FPC0, IPS=2), cam 0 IN30 (FPC3), cam 1 IN34 (FPC5) — FPCCTRk.IPS = 2 as programmed by the
//! firmware (VERIFIED-T, doc §3.2/3.3) and the fixed SINk2 = IN(2k+24) assignment of TC1796 UM Table 24-9
//! (p.24-94, HYP for TC1791). GTC31 captures the raw IN24 falling edges (decoded GIMCRH3 = 0x80C0C3C3).
//!
//! Geometry evidence (the DAMOS SC8S50.json has no crank/cam reference *positions*: these are NC
//! constants compiled into the code, read here from the flash image):
//! * teeth = 60: ENSD init `mov d15,#0x3c` @ 0x800EA53E (VERIFIED-S); 2 missing teeth (FR p.7923, the same
//!   init stores 2 @ 0x800EA530).
//! * reference -> TDC0 = 78 °CRK: `mov.u d2,#0x4e0` (0x4E0 = 78·16, x16 °CRK) @ 0x800EA524 = NC_OFS_TDC0_REF_CRK
//!   (FR p.7924); next to it 0x480 = 72·16 (segment start 72 °CRK before TDC, FR p.7925) @ 0x800EA51C.
//!   The reference is taken as the first active (falling) edge after the gap — HYP (FR wording).
//! * cam edges, °CRK after TDC0 at reference phasing: intake table @ 0x80050554 and exhaust @ 0x80050584,
//!   8 × u16 in 1/16 °CRK, copied by 0x800EA430..0x800EA466 (VERIFIED-S): IN 46, 80, 226, 260, 406, 542, 586, 2;
//!   EX 179, 213, 359, 495, 539, 675, 719, 33 (identical to the FR baseline, p.7920-7925).
//!   Polarity (which of the 8 edges are rising) is UNVERIFIED: by default edges 0,2,4,6 are rising.
//!   Assignment cam 0 (GTC28/FPC3) = intake, cam 1 (GTC26/FPC5) = exhaust is HYP (doc §3.3).
//! * CPDD crank sensor: the signal goes low at the active edge and returns high after a fixed pulse width;
//!   forward 15..62.2 µs (DAMOS c_t_crk_sho_level_min/max = 75/311 ·0.2 µs @ 0x80805ABC/0x80805AB8), backward
//!   62.4..150 µs (c_t_crk_sho_level_back_min/max @ 0x80805AB4/0x80805AB0). Defaults 45 µs / 100 µs.

use crate::gpta::InputEdge;

/// Flash image offset of a PFlash0 address.
fn off(addr: u32) -> usize {
    (addr - 0x8000_0000) as usize
}

#[derive(Clone, Debug)]
pub struct WheelGeometry {
    pub teeth: u32,
    pub missing: u32,
    /// Angle from the reference edge to TDC0 (°CRK).
    pub ref_to_tdc0_deg: f64,
    pub cam_in_edges_deg: [f64; 8],
    pub cam_ex_edges_deg: [f64; 8],
    /// Level after edge 0 of the cam tables (UNVERIFIED).
    pub cam_edge0_rising: bool,
    pub pulse_fwd_s: f64,
    pub pulse_back_s: f64,
}

impl WheelGeometry {
    /// Values of SW 8V0906259J / SC8S50 (see module docs for the addresses they are read from).
    pub fn simos18_sc8s50() -> Self {
        Self {
            teeth: 60,
            missing: 2,
            ref_to_tdc0_deg: 78.0,
            cam_in_edges_deg: [46.0, 80.0, 226.0, 260.0, 406.0, 542.0, 586.0, 2.0],
            cam_ex_edges_deg: [179.0, 213.0, 359.0, 495.0, 539.0, 675.0, 719.0, 33.0],
            cam_edge0_rising: true,
            pulse_fwd_s: 45e-6,
            pulse_back_s: 100e-6,
        }
    }

    /// Read the geometry from a SIMOS18.1 flash image (4 MB, file 0 = 0x80000000).
    pub fn from_flash(flash: &[u8]) -> Option<Self> {
        let w32 = |a: u32| flash.get(off(a)..off(a) + 4).map(|b| u32::from_le_bytes(b.try_into().unwrap()));
        let h16 = |a: u32| flash.get(off(a)..off(a) + 2).map(|b| u16::from_le_bytes(b.try_into().unwrap()));
        // mov.u d2,#const16 (RLC, opcode 0xBB): const16 = bits [27:12]
        let movu = w32(0x800E_A524)?;
        let teeth = flash.get(off(0x800E_A53E)..off(0x800E_A53E) + 2)?;
        let missing = flash.get(off(0x800E_A530)..off(0x800E_A530) + 2)?;
        if movu & 0xFF != 0xBB || teeth[0] != 0xDA || missing[0] != 0xDA {
            return None;
        }
        let mut g = Self::simos18_sc8s50();
        g.ref_to_tdc0_deg = ((movu >> 12) & 0xFFFF) as f64 / 16.0;
        g.teeth = teeth[1] as u32;
        g.missing = missing[1] as u32;
        for i in 0..8 {
            g.cam_in_edges_deg[i] = h16(0x8005_0554 + 2 * i as u32)? as f64 / 16.0;
            g.cam_ex_edges_deg[i] = h16(0x8005_0584 + 2 * i as u32)? as f64 / 16.0;
        }
        Some(g)
    }

    fn tooth_deg(&self) -> f64 {
        360.0 / self.teeth.max(1) as f64
    }

    /// Active (falling) crank edge positions within one revolution, relative to TDC0.
    fn crank_edges(&self) -> Vec<f64> {
        (0..self.teeth.saturating_sub(self.missing))
            .map(|n| -self.ref_to_tdc0_deg + n as f64 * self.tooth_deg())
            .collect()
    }

    /// Cam level at crank angle `theta` (cam phasing `shift` °CRK, positive = later).
    pub fn cam_level(&self, edges: &[f64; 8], theta: f64, shift: f64) -> bool {
        // Last edge at or before theta within the 720° cycle decides the level.
        let mut best = (f64::MAX, false);
        for (i, &e) in edges.iter().enumerate() {
            let back = (theta - (e + shift)).rem_euclid(720.0);
            if back < best.0 {
                best = (back, (i % 2 == 0) == self.cam_edge0_rising);
            }
        }
        best.1
    }
}

/// GPTA0 input lines of the sensors.
pub const LINE_CRANK: u8 = 24;
pub const LINE_CAM0: u8 = 30;
pub const LINE_CAM1: u8 = 34;

/// Stateful edge generator: feed it the crank angle trajectory sample by sample.
pub struct CrankWheel {
    pub geo: WheelGeometry,
    pub f_gpta_hz: f64,
    t: f64,
    theta: f64,
    cam_shift: [f64; 2],
    pending: Vec<InputEdge>,
    crank_edges: Vec<f64>,
    /// Number of active crank edges generated so far.
    pub teeth_seen: u64,
}

impl CrankWheel {
    pub fn new(geo: WheelGeometry, f_gpta_hz: f64, t0_s: f64, theta0_deg: f64) -> Self {
        let crank_edges = geo.crank_edges();
        Self { geo, f_gpta_hz, t: t0_s, theta: theta0_deg, cam_shift: [0.0; 2], pending: Vec::new(), crank_edges, teeth_seen: 0 }
    }

    fn tick(&self, t: f64) -> u64 {
        if !(t.is_finite() && self.f_gpta_hz.is_finite() && t > 0.0 && self.f_gpta_hz > 0.0) {
            return 0;
        }
        (t * self.f_gpta_hz).round() as u64
    }

    fn edge(&self, t: f64, line: u8, level: bool) -> InputEdge {
        InputEdge { t: self.tick(t), kernel: 0, line, level }
    }

    /// Pin levels at the start (crank idle high, cams from the angle).
    pub fn initial_edges(&self) -> Vec<InputEdge> {
        vec![
            self.edge(self.t, LINE_CRANK, true),
            self.edge(self.t, LINE_CAM0, self.geo.cam_level(&self.geo.cam_in_edges_deg, self.theta, self.cam_shift[0])),
            self.edge(self.t, LINE_CAM1, self.geo.cam_level(&self.geo.cam_ex_edges_deg, self.theta, self.cam_shift[1])),
        ]
    }

    /// Advance to time `t1_s` at crank angle `theta1_deg` (linear in time between samples) with the given
    /// cam phase shifts (°CRK). Returns the edges with `t` in (previous sample, t1], sorted.
    pub fn advance(&mut self, t1_s: f64, theta1_deg: f64, cam_in_shift_deg: f64, cam_ex_shift_deg: f64) -> Vec<InputEdge> {
        if !(t1_s.is_finite() && theta1_deg.is_finite() && cam_in_shift_deg.is_finite() && cam_ex_shift_deg.is_finite()) {
            return Vec::new();
        }
        let (t0, th0) = (self.t, self.theta);
        let dth = theta1_deg - th0;
        let at = |th: f64| t0 + (th - th0) / dth * (t1_s - t0);
        let mut out = std::mem::take(&mut self.pending);
        if dth != 0.0 {
            let fwd = dth > 0.0;
            let (lo, hi) = if fwd { (th0, theta1_deg) } else { (theta1_deg, th0) };
            // crank teeth (period 360°)
            let pulse = if fwd { self.geo.pulse_fwd_s } else { self.geo.pulse_back_s };
            for i in 0..self.crank_edges.len() {
                for th in crossings(lo, hi, self.crank_edges[i], 360.0, fwd) {
                    let t = at(th);
                    out.push(self.edge(t, LINE_CRANK, false));
                    out.push(self.edge(t + pulse, LINE_CRANK, true));
                    self.teeth_seen += 1;
                }
            }
            // cams (period 720°); the shift is applied at the new sample (ponytail: phaser motion within one
            // sample interval is not interpolated — sample at least every few degrees when the phaser moves)
            for (c, (line, edges)) in [(LINE_CAM0, self.geo.cam_in_edges_deg), (LINE_CAM1, self.geo.cam_ex_edges_deg)].into_iter().enumerate() {
                let shift = if c == 0 { cam_in_shift_deg } else { cam_ex_shift_deg };
                for (i, &e) in edges.iter().enumerate() {
                    let rising_after = (i % 2 == 0) == self.geo.cam_edge0_rising;
                    for th in crossings(lo, hi, e + shift, 720.0, fwd) {
                        out.push(self.edge(at(th), line, if fwd { rising_after } else { !rising_after }));
                    }
                }
            }
        }
        self.cam_shift = [cam_in_shift_deg, cam_ex_shift_deg];
        self.t = t1_s;
        self.theta = theta1_deg;
        let end = self.tick(t1_s);
        out.sort_by_key(|e| e.t);
        let later = out.iter().position(|e| e.t > end).unwrap_or(out.len());
        self.pending = out.split_off(later);
        out
    }
}

/// Angles `pos + k·period` in (lo, hi] (forward) or [lo, hi) (backward), in traversal order.
fn crossings(lo: f64, hi: f64, pos: f64, period: f64, fwd: bool) -> Vec<f64> {
    if !(lo.is_finite() && hi.is_finite() && pos.is_finite() && period.is_finite() && period > 0.0 && hi >= lo) {
        return Vec::new();
    }
    let mut v = Vec::new();
    let mut th = pos + ((lo - pos) / period).floor() * period;
    while th <= hi && v.len() < 65_536 {
        if (fwd && th > lo) || (!fwd && th >= lo && th < hi) {
            v.push(th);
        }
        let next = th + period;
        if next <= th {
            break;
        }
        th = next;
    }
    if !fwd {
        v.reverse();
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sixty_minus_two_pattern_and_reference() {
        let geo = WheelGeometry::simos18_sc8s50();
        let f = 90e6;
        // 600 rpm = 3600 °/s; start just after TDC0, run 2 revolutions
        let mut w = CrankWheel::new(geo.clone(), f, 0.0, 1.0);
        let mut edges = Vec::new();
        let n = 1440;
        for i in 1..=n {
            let t = i as f64 * 0.2 / n as f64;
            edges.extend(w.advance(t, 1.0 + 720.0 * i as f64 / n as f64, 0.0, 0.0));
        }
        let falls: Vec<u64> = edges.iter().filter(|e| e.line == LINE_CRANK && !e.level).map(|e| e.t).collect();
        assert_eq!(falls.len(), 2 * 58);
        // tooth period 6° = 1/600 s / 60 -> 1.6667 ms ; gap = 3 tooth periods
        let d: Vec<u64> = falls.windows(2).map(|p| p[1] - p[0]).collect();
        let gaps: Vec<usize> = d.iter().enumerate().filter(|(_, &x)| x > 200_000).map(|(i, _)| i).collect();
        assert_eq!(gaps.len(), 2); // one gap per revolution
        assert!((d[gaps[0]] as f64 / 150_000.0 - 3.0).abs() < 0.01);
        // the first falling edge after the gap is the reference: 78° before TDC (360°) -> theta 282
        let ref_t = falls[gaps[0] + 1] as f64 / f;
        let theta = 1.0 + 3600.0 * ref_t;
        assert!((theta - 282.0).abs() < 0.01, "{theta}");
        // rising 45 us after each falling edge
        let rise = edges.iter().find(|e| e.line == LINE_CRANK && e.level && e.t > falls[0]).unwrap();
        assert!((4049..=4051).contains(&(rise.t - falls[0])));
        // 8 edges per cam and 720°
        assert_eq!(edges.iter().filter(|e| e.line == LINE_CAM0).count(), 8);
        assert_eq!(edges.iter().filter(|e| e.line == LINE_CAM1).count(), 8);
        // levels alternate on each cam
        let c0: Vec<bool> = edges.iter().filter(|e| e.line == LINE_CAM0).map(|e| e.level).collect();
        assert!(c0.windows(2).all(|p| p[0] != p[1]));
    }

    #[test]
    fn geometry_matches_flash() {
        let name = "8V0906264K__0003.bin";
        let Some(f) = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .find_map(|d| {
                std::fs::read(d.join("vault/firmwares/ecu/stock").join(name))
                    .or_else(|_| std::fs::read(d.join(name)))
                    .ok()
            })
        else {
            eprintln!("SKIP: {name} not found");
            return;
        };
        let g = WheelGeometry::from_flash(&f).expect("geometry constants");
        let d = WheelGeometry::simos18_sc8s50();
        assert_eq!((g.teeth, g.missing, g.ref_to_tdc0_deg), (d.teeth, d.missing, d.ref_to_tdc0_deg));
        assert_eq!(g.cam_in_edges_deg, d.cam_in_edges_deg);
        assert_eq!(g.cam_ex_edges_deg, d.cam_ex_edges_deg);
    }

    #[test]
    fn backward_rotation_uses_long_pulse() {
        let mut w = CrankWheel::new(WheelGeometry::simos18_sc8s50(), 90e6, 0.0, 30.0);
        let e = w.advance(0.01, 20.0, 0.0, 0.0);
        let falls: Vec<_> = e.iter().filter(|x| x.line == LINE_CRANK && !x.level).collect();
        assert_eq!(falls.len(), 1); // teeth at multiples of 6°: only 24° lies in [20°, 30°)
        let r = e.iter().find(|x| x.line == LINE_CRANK && x.level).unwrap();
        assert!((8999..=9001).contains(&(r.t - falls[0].t)));
    }
}
