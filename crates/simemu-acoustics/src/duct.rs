//! Linear 1-D duct acoustics by transfer matrices (M. L. Munjal, "Acoustics of Ducts and Mufflers", 2nd ed.,
//! Wiley 2014, ch. 2–3): state (p, q) = (acoustic pressure, volume velocity); a chain of elements from the
//! source to the open end; the open end is loaded by the radiation impedance of an unflanged pipe (H. Levine,
//! J. Schwinger, Phys. Rev. 73 (1948) 383–406) and radiates as a monopole (L. E. Kinsler et al.,
//! "Fundamentals of Acoustics", 4th ed., 2000, §7.4). Plane waves only: above the first cross-mode cut-on of a
//! chamber (f ≈ 1.84·c/(π·D), ≈ 0.9–1.5 kHz for silencer shells) the 1-D model is only indicative (Munjal
//! ch. 6). Mean-flow convection is neglected (Mach < 0.15 in the pipes); the flow enters only through the
//! linearised resistances of the catalyst monolith and the air filter.
//!
//! [`Duct::build_ir`] samples the response "pressure at 1 m from the open end per unit source volume velocity"
//! and turns it into an FIR filter (inverse FFT, raised-cosine tail).

use crate::fft::Fft;
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq)]
struct C(f64, f64);

impl C {
    const ZERO: C = C(0.0, 0.0);
    const ONE: C = C(1.0, 0.0);
    fn add(self, o: C) -> C {
        C(self.0 + o.0, self.1 + o.1)
    }
    fn mul(self, o: C) -> C {
        C(self.0 * o.0 - self.1 * o.1, self.0 * o.1 + self.1 * o.0)
    }
    fn scale(self, s: f64) -> C {
        C(self.0 * s, self.1 * s)
    }
    fn inv(self) -> C {
        let d = self.0 * self.0 + self.1 * self.1;
        if d <= 1e-30 || !d.is_finite() { C::ZERO } else { C(self.0 / d, -self.1 / d) }
    }
    fn cos(self) -> C {
        C(self.0.cos() * self.1.cosh(), -self.0.sin() * self.1.sinh())
    }
    fn sin(self) -> C {
        C(self.0.sin() * self.1.cosh(), self.0.cos() * self.1.sinh())
    }
}

type M = [C; 4];

fn mm(a: &M, b: &M) -> M {
    [a[0].mul(b[0]).add(a[1].mul(b[2])), a[0].mul(b[1]).add(a[1].mul(b[3])), a[2].mul(b[0]).add(a[3].mul(b[2])), a[2].mul(b[1]).add(a[3].mul(b[3]))]
}

/// Gas of a duct system: ratio of specific heats, specific gas constant, mean static pressure.
#[derive(Clone, Copy, Debug)]
pub struct Gas {
    pub gamma: f64,
    pub r: f64,
    pub p_pa: f64,
}

impl Gas {
    pub fn c(&self, t_k: f64) -> f64 {
        (self.gamma.max(1.0) * self.r.max(1.0) * t_k.max(50.0)).sqrt()
    }
    pub fn rho(&self, t_k: f64) -> f64 {
        self.p_pa.max(100.0) / (self.r.max(1.0) * t_k.max(50.0))
    }
}

/// Dynamic viscosity of air / exhaust gas (Sutherland), Pa·s.
pub fn viscosity(t_k: f64) -> f64 {
    let t_k = t_k.max(50.0);
    1.716e-5 * (t_k / 273.15).powf(1.5) * (273.15 + 110.4) / (t_k + 110.4)
}

#[derive(Clone, Copy, Debug)]
pub enum Element {
    /// Uniform rigid pipe: length, inner diameter, gas temperature, extra absorption of a lined/packed section
    /// (Np/m at 1 kHz, ∝ √f).
    Pipe { len: f64, dia: f64, t_k: f64, absorb: f64 },
    /// Series lumped element: resistance (Pa·s/m³) and inertance (kg/m⁴).
    Series { r: f64, m: f64 },
    /// Shunt compliance of a volume, m³.
    Volume { vol: f64, t_k: f64 },
    /// Closed side branch (annular cavity of an extended tube, quarter-wave resonator): length, area.
    Branch { len: f64, area: f64, t_k: f64 },
    /// Shunt R–C branch to ground (source impedance: turbine resistance in series with the manifold volume).
    Shunt { r: f64, c: f64 },
}

/// Wavenumber with Kirchhoff visco-thermal wall attenuation (Munjal §1.10): α = √(ων/2)/(a·c)·(1 + (γ−1)/√Pr).
fn wavenumber(g: &Gas, w: f64, t_k: f64, dia: f64, absorb: f64) -> (C, f64, f64) {
    let dia = dia.max(1e-4);
    let w = w.max(0.0);
    let c = g.c(t_k);
    let rho = g.rho(t_k);
    let nu = viscosity(t_k) / rho;
    let alpha = (w * nu / 2.0).sqrt() / (0.5 * dia * c) * (1.0 + (g.gamma.max(1.0) - 1.0) / 0.7f64.sqrt()) + absorb.max(0.0) * (w / (2.0 * PI * 1000.0)).sqrt();
    (C(w / c, -alpha), c, rho)
}

fn matrix(e: &Element, g: &Gas, w: f64) -> M {
    match *e {
        Element::Pipe { len, dia, t_k, absorb } => {
            let (len, dia) = (len.max(0.0), dia.max(1e-4));
            let (k, c, rho) = wavenumber(g, w, t_k, dia, absorb);
            let z = rho * c / (PI / 4.0 * dia * dia);
            let kl = k.scale(len);
            let (cs, sn) = (kl.cos(), kl.sin());
            [cs, C(0.0, z).mul(sn), C(0.0, 1.0 / z).mul(sn), cs]
        }
        Element::Series { r, m } => [C::ONE, C(r, w * m), C::ZERO, C::ONE],
        Element::Volume { vol, t_k } => {
            let c = g.c(t_k);
            [C::ONE, C::ZERO, C(0.0, w * vol.max(0.0) / (g.rho(t_k) * c * c)), C::ONE]
        }
        Element::Branch { len, area, t_k } => {
            let (len, area) = (len.max(0.0), area.max(1e-8));
            let dia = (4.0 * area / PI).sqrt();
            let (k, c, rho) = wavenumber(g, w, t_k, dia, 0.0);
            let kl = k.scale(len);
            // Closed end: Z_in = −j·Z0·cot(kL) → admittance j·tan(kL)/Z0.
            let tan = kl.sin().mul(kl.cos().inv());
            [C::ONE, C::ZERO, C(0.0, area / (rho * c)).mul(tan), C::ONE]
        }
        Element::Shunt { r, c } => [C::ONE, C::ZERO, C(0.0, w * c).mul(C(1.0, w * r * c).inv()), C::ONE],
    }
}

/// A duct system from a volume-velocity source to an open end that radiates to the ambient air.
pub struct Duct {
    pub elems: Vec<Element>,
    pub gas: Gas,
    pub tail_dia: f64,
    pub tail_t_k: f64,
    /// Density of the air the open end radiates into, kg/m³.
    pub rho_out: f64,
    fft: Fft,
    re: Vec<f32>,
    im: Vec<f32>,
}

impl Duct {
    /// `n_fft` = FIR length (power of two).
    pub fn new(n_fft: usize) -> Self {
        Self {
            elems: Vec::with_capacity(32),
            gas: Gas { gamma: 1.4, r: 287.0, p_pa: 101_325.0 },
            tail_dia: 0.05,
            tail_t_k: 300.0,
            rho_out: 1.2,
            fft: Fft::new(n_fft),
            re: vec![0.0; n_fft],
            im: vec![0.0; n_fft],
        }
    }

    /// Radiation impedance of the unflanged open end: Levine–Schwinger low-ka limit Z0·((ka)²/4 + j·0.6133·ka),
    /// realised as end-correction mass ‖ resistance R0·Z0 (R0 = 4·0.6133² so that Re → (ka)²/4 for ka ≪ 1 and the
    /// real part saturates at high ka).
    fn z_rad(&self, w: f64) -> C {
        let a = 0.5 * self.tail_dia.max(1e-4);
        let c = self.gas.c(self.tail_t_k);
        let z0 = self.gas.rho(self.tail_t_k) * c / (PI * a * a);
        let x = 0.6133 * w.max(0.0) / c * a;
        let r0 = 4.0 * 0.6133 * 0.6133;
        C(0.0, x * r0).mul(C(r0, x).inv()).scale(z0)
    }

    /// q_open_end / q_source at angular frequency `w`.
    fn transfer(&self, w: f64) -> C {
        let mut t: M = [C::ONE, C::ZERO, C::ZERO, C::ONE];
        for e in &self.elems {
            t = mm(&t, &matrix(e, &self.gas, w));
        }
        t[2].mul(self.z_rad(w)).add(t[3]).inv()
    }

    /// Radiated pressure at 1 m per unit source volume velocity, Pa/(m³/s): jωρ0/(4π)·q_end/q_src.
    pub fn response(&self, f: f64) -> (f64, f64) {
        let w = 2.0 * PI * f;
        let h = self.transfer(w).mul(C(0.0, w * self.rho_out / (4.0 * PI)));
        (h.0, h.1)
    }

    /// FIR of the response at sample rate `fs` into `out` (length = FFT size).
    pub fn build_ir(&mut self, fs: f64, out: &mut [f32]) {
        let n = self.fft.n;
        for k in 0..=n / 2 {
            let (re, im) = if k == 0 { (0.0, 0.0) } else { self.response(k as f64 * fs / n as f64) };
            self.re[k] = re as f32;
            self.im[k] = if k == n / 2 { 0.0 } else { im as f32 };
        }
        for k in 1..n / 2 {
            self.re[n - k] = self.re[k];
            self.im[n - k] = -self.im[k];
        }
        self.fft.inverse(&mut self.re, &mut self.im);
        let taper = n / 8;
        for (i, o) in out.iter_mut().enumerate().take(n) {
            let w = if i + taper >= n { 0.5 * (1.0 + (PI * (i + taper + 1 - n) as f64 / taper as f64).cos()) } else { 1.0 };
            *o = self.re[i] * w as f32;
        }
    }
}

/// Exhaust line downstream of the turbine. All dimensions UNVERIFIED (no drawing of the Octavia A7 1.8 TSI
/// exhaust is available here; values are typical of a C-segment 1.8 TSI line: close-coupled catalyst, front
/// pipe, middle silencer, rear silencer with extended inlet/outlet, single tailpipe).
#[derive(Clone, Copy, Debug)]
pub struct ExhaustGeometry {
    pub downpipe_len: f64,
    pub pipe_dia: f64,
    pub cat_dia: f64,
    pub cat_len: f64,
    /// Monolith channel hydraulic diameter and open frontal area (400 cpsi ≈ 1.1 mm, 0.8).
    pub cat_cell_m: f64,
    pub cat_porosity: f64,
    pub front_pipe_len: f64,
    pub mid_silencer_len: f64,
    pub mid_silencer_dia: f64,
    pub mid_pipe_len: f64,
    pub rear_silencer_len: f64,
    /// Rear silencer shell equivalent diameter (oval ≈ 0.30 × 0.16 m).
    pub rear_silencer_dia: f64,
    pub rear_ext_in: f64,
    pub rear_ext_out: f64,
    pub tail_len: f64,
    pub tail_dia: f64,
    /// Absorption of the packed silencer sections, Np/m at 1 kHz.
    pub absorb_np_m: f64,
    /// Gas cooling length downstream of the catalyst, m: T(x) = T_amb + (T_cat − T_amb)·e^(−x/L).
    pub cooling_len: f64,
}

impl Default for ExhaustGeometry {
    fn default() -> Self {
        Self {
            downpipe_len: 0.35,
            pipe_dia: 0.052,
            cat_dia: 0.118,
            cat_len: 0.11,
            cat_cell_m: 1.1e-3,
            cat_porosity: 0.8,
            front_pipe_len: 1.2,
            mid_silencer_len: 0.32,
            mid_silencer_dia: 0.12,
            mid_pipe_len: 1.35,
            rear_silencer_len: 0.42,
            rear_silencer_dia: 0.247,
            rear_ext_in: 0.10,
            rear_ext_out: 0.08,
            tail_len: 0.25,
            tail_dia: 0.05,
            absorb_np_m: 0.8,
            cooling_len: 3.0,
        }
    }
}

impl ExhaustGeometry {
    /// Fills `d` for gas at turbine outlet `t4`, catalyst `t_cat`, ambient `t_amb` (K), back pressure `p4`,
    /// mean mass flow `m_dot`, and the source impedance seen from the turbine outlet: linearised turbine +
    /// wastegate resistance `r_turb` (2·Δp/Q) in series with the manifold compliance `c_em` (V/(γ·p)).
    #[allow(clippy::too_many_arguments)]
    pub fn configure(&self, d: &mut Duct, t4: f64, t_cat: f64, t_amb: f64, p4: f64, rho_amb: f64, m_dot: f64, r_turb: f64, c_em: f64) {
        let g = self;
        let pipe_dia = g.pipe_dia.max(1e-3);
        let cat_dia = g.cat_dia.max(1e-3);
        let cat_porosity = g.cat_porosity.clamp(0.01, 1.0);
        let cat_cell_m = g.cat_cell_m.max(1e-5);
        let mid_dia = g.mid_silencer_dia.max(1e-3);
        let rear_dia = g.rear_silencer_dia.max(1e-3);
        let tail_dia = g.tail_dia.max(1e-3);
        let cooling_len = g.cooling_len.max(1e-3);
        d.gas = Gas { gamma: 1.34, r: 288.0, p_pa: p4.max(80e3) };
        d.rho_out = rho_amb.max(0.1);
        let gas = d.gas;
        let t_at = |x: f64| t_amb + (t_cat - t_amb).max(0.0) * (-x / cooling_len).exp();
        let e = &mut d.elems;
        e.clear();
        e.push(Element::Shunt { r: r_turb.max(0.0), c: c_em.max(0.0) });
        e.push(Element::Pipe { len: g.downpipe_len.max(0.0), dia: pipe_dia, t_k: t4, absorb: 0.0 });
        // Catalyst: inlet/outlet cones as short wide sections, monolith = parallel capillaries (open area
        // φ·S, Hagen–Poiseuille resistance 32μL/(φ·d_h²·S); cf. E. Dokumaci, J. Sound Vib. 182 (1995) 799–808).
        let s_cat = PI / 4.0 * cat_dia * cat_dia;
        let mono_dia = cat_dia * cat_porosity.sqrt();
        e.push(Element::Pipe { len: 0.04, dia: cat_dia, t_k: t_cat, absorb: 0.0 });
        e.push(Element::Series { r: 32.0 * viscosity(t_cat) * g.cat_len.max(0.0) / (cat_porosity * cat_cell_m * cat_cell_m * s_cat), m: 0.0 });
        e.push(Element::Pipe { len: g.cat_len.max(0.0), dia: mono_dia, t_k: t_cat, absorb: 0.0 });
        e.push(Element::Pipe { len: 0.04, dia: cat_dia, t_k: t_cat, absorb: 0.0 });
        let mut x = 0.0;
        let mut last = cat_dia;
        // Mean-flow losses at sudden area changes, linearised (Borda–Carnot expansion, contraction K = ½(1 − S2/S1);
        // Munjal §4.5): R = dΔp/dQ.
        let mut pipe = |e: &mut Vec<Element>, len: f64, dia: f64, absorb: f64, x: &mut f64| {
            let len = len.max(0.0);
            let t = t_at(*x);
            let rho = gas.rho(t);
            let q = m_dot.max(0.0) / rho;
            let (s1, s2) = (PI / 4.0 * last * last, PI / 4.0 * dia * dia);
            let r = if s2 > s1 { rho * q / s1 * (1.0 - s1 / s2).powi(2) / s1 } else { rho * q / s2 * 0.5 * (1.0 - s2 / s1) / s2 };
            if r > 0.0 && (s2 / s1 - 1.0).abs() > 0.05 {
                e.push(Element::Series { r, m: 0.0 });
            }
            last = dia;
            e.push(Element::Pipe { len, dia, t_k: t_at(*x + 0.5 * len), absorb });
            *x += len;
        };
        pipe(e, g.front_pipe_len, pipe_dia, 0.0, &mut x);
        pipe(e, g.mid_silencer_len, mid_dia, g.absorb_np_m, &mut x);
        pipe(e, g.mid_pipe_len, pipe_dia, 0.0, &mut x);
        // Rear silencer: extended-inlet / extended-outlet chamber (Munjal §3.6): annular cavities as closed branches.
        let annulus = (PI / 4.0 * (rear_dia.powi(2) - pipe_dia.powi(2))).max(1e-6);
        let (ext_in, ext_out) = (g.rear_ext_in.max(0.0), g.rear_ext_out.max(0.0));
        e.push(Element::Branch { len: ext_in, area: annulus, t_k: t_at(x) });
        pipe(e, (g.rear_silencer_len - ext_in - ext_out).max(0.01), rear_dia, g.absorb_np_m, &mut x);
        e.push(Element::Branch { len: ext_out, area: annulus, t_k: t_at(x) });
        pipe(e, g.tail_len, tail_dia, 0.0, &mut x);
        d.tail_dia = tail_dia;
        d.tail_t_k = t_at(x);
    }
}

/// Intake line upstream of the compressor (compressor face → inlet hose → air box with filter → snorkel).
/// Dimensions UNVERIFIED (typical EA888 Gen3 MQB air box ≈ 8 l).
#[derive(Clone, Copy, Debug)]
pub struct IntakeGeometry {
    pub hose_len: f64,
    pub hose_dia: f64,
    pub box_len: f64,
    pub box_dia: f64,
    pub snorkel_len: f64,
    pub snorkel_dia: f64,
    /// Laminar (Darcy) resistance of the clean filter element, Pa·s/m³.
    pub filter_r: f64,
}

impl Default for IntakeGeometry {
    fn default() -> Self {
        Self { hose_len: 0.45, hose_dia: 0.065, box_len: 0.25, box_dia: 0.20, snorkel_len: 0.30, snorkel_dia: 0.07, filter_r: 4.0e3 }
    }
}

impl IntakeGeometry {
    /// `r_filter_flow` = linearised quadratic filter loss at the operating point (2·Δp/Q), Pa·s/m³.
    pub fn configure(&self, d: &mut Duct, t_air: f64, p_amb: f64, r_filter_flow: f64) {
        let g = self;
        let t_air = t_air.max(50.0);
        let (hose_dia, box_dia, snorkel_dia) = (g.hose_dia.max(1e-3), g.box_dia.max(1e-3), g.snorkel_dia.max(1e-3));
        d.gas = Gas { gamma: 1.4, r: 287.0, p_pa: p_amb.max(100.0) };
        d.rho_out = d.gas.rho(t_air);
        let e = &mut d.elems;
        e.clear();
        e.push(Element::Pipe { len: g.hose_len.max(0.0), dia: hose_dia, t_k: t_air, absorb: 0.0 });
        e.push(Element::Pipe { len: 0.5 * g.box_len.max(0.0), dia: box_dia, t_k: t_air, absorb: 0.0 });
        e.push(Element::Series { r: (g.filter_r + r_filter_flow).max(0.0), m: 0.0 });
        e.push(Element::Pipe { len: 0.5 * g.box_len.max(0.0), dia: box_dia, t_k: t_air, absorb: 0.0 });
        e.push(Element::Pipe { len: g.snorkel_len.max(0.0), dia: snorkel_dia, t_k: t_air, absorb: 0.0 });
        d.tail_dia = snorkel_dia;
        d.tail_t_k = t_air;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Classic expansion-chamber transmission loss (Munjal eq. 3.36, Davis et al. NACA 1192):
    /// TL = 10·log10(1 + ¼(m − 1/m)²·sin²(kL)), m = area ratio. The chain reproduces it (anechoic end).
    #[test]
    fn expansion_chamber_matches_the_classical_transmission_loss() {
        let g = Gas { gamma: 1.4, r: 287.0, p_pa: 101_325.0 };
        let (d1, d2, l, t) = (0.05, 0.15, 0.3, 293.15);
        let c = g.c(t);
        let s1 = PI / 4.0 * d1 * d1;
        let z1 = g.rho(t) * c / s1;
        let m = (d2 / d1).powi(2);
        for f in [100.0, 250.0, 400.0, 600.0] {
            let w = 2.0 * PI * f;
            // The visco-thermal loss of a 0.3 m wide chamber is < 0.01 Np: negligible against the reference.
            let t_m = matrix(&Element::Pipe { len: l, dia: d2, t_k: t, absorb: 0.0 }, &g, w);
            // TL with anechoic termination: |(T11 + T12/Z1 + T21·Z1 + T22)/2|².
            let s = t_m[0].add(t_m[1].scale(1.0 / z1)).add(t_m[2].scale(z1)).add(t_m[3]).scale(0.5);
            let tl = 10.0 * (s.0 * s.0 + s.1 * s.1).log10();
            let kl = w / c * l;
            let tl_ref = 10.0 * (1.0 + 0.25 * (m - 1.0 / m).powi(2) * kl.sin().powi(2)).log10();
            assert!((tl - tl_ref).abs() < 0.3, "{f} Hz: {tl:.2} vs {tl_ref:.2} dB");
        }
    }

    /// The exhaust FIR is causal and decays inside its window (no time aliasing), and the low-frequency
    /// response follows the compact-source limit ρ0·ω·q/(4π) (rigid line, incompressible at low ka).
    #[test]
    fn exhaust_ir_decays_and_low_frequency_matches_the_monopole_limit() {
        let mut d = Duct::new(8192);
        ExhaustGeometry::default().configure(&mut d, 900.0, 800.0, 293.0, 110e3, 1.2, 0.02, 2e6, 1e-3 / (1.33 * 110e3));
        let (re, im) = d.response(2.0);
        let mono = 1.2 * 2.0 * PI * 2.0 / (4.0 * PI);
        assert!(((re * re + im * im).sqrt() / mono - 1.0).abs() < 0.05, "{re} {im} vs {mono}");
        let mut ir = vec![0.0f32; 8192];
        d.build_ir(48_000.0, &mut ir);
        let e: f32 = ir.iter().map(|x| x * x).sum();
        let tail: f32 = ir[7000..].iter().map(|x| x * x).sum();
        let head: f32 = ir[..40].iter().map(|x| x * x).sum();
        assert!(tail < 1e-4 * e, "tail energy {tail} of {e}");
        assert!(head < 1e-3 * e, "nothing before the propagation delay: {head} of {e}");
    }
}
