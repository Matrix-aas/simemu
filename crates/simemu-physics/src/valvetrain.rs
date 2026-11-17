//! Valvetrain: cam profiles, oil-driven vane phasers, roller-finger-follower valves with hydraulic
//! lash adjusters (HLA), valve float and valve–piston contact.
//!
//! Valve model (per valve, single DOF at the valve, follower mass reflected): while the follower is on
//! the cam the valve is kinematically driven, x = x_cam + e_HLA, and the cam contact force is
//! F_c = m·a_cam + F_pre + k·x + c·v + F_gas. F_c < 0 means the spring can no longer decelerate the
//! valve train: the valve separates (float) and flies free under m·ẍ = −F_pre − k·x − c·ẋ − F_gas
//! until the cam catches it or it hits the seat (restitution e). During separation the HLA plunger
//! pumps up (check valve refills) and only leaks down slowly under load, so repeated float jacks
//! the valve open — this, not an rpm threshold, is what makes the valve meet the piston at overlap TDC.
//! Contact: plastic stem bending when the impact energy exceeds the elastic bending energy of the
//! stem, permanent bend → seat leak gap.

use crate::crank_train::Slider;
use crate::materials::VALVE_STEEL;
use crate::profiles::{AvsHw, EngineProfile, PhaserHw, ValveGear};
use serde::Serialize;
use std::f64::consts::PI;

/// HLA pump-up speed at 3 bar (check-ball refill) and leak-down compliance, m/s and m/(N·s). UNVERIFIED.
const HLA_PUMP_M_S: f64 = 0.02;
const HLA_LEAK_M_NS: f64 = 4e-7;
const HLA_TRAVEL_M: f64 = 3e-3;
const SEAT_RESTITUTION: f64 = 0.4;
const PISTON_RESTITUTION: f64 = 0.3;
/// Free-stem lever between the valve guide and the head, m; seat-gap lever (guide length), m. UNVERIFIED.
const STEM_LEVER_M: f64 = 0.03;
const GUIDE_LEVER_M: f64 = 0.04;
/// Permanent bend at which the head separates from the stem (valve dropped). UNVERIFIED.
const BEND_BREAK_M: f64 = 2.5e-3;
const FREE_FLIGHT_DT: f64 = 4e-6;

/// Lift (m) and its derivatives w.r.t. crank angle (m/rad, m/rad²) at `dphi_deg` from the lobe
/// centre: L·(1−u²)³, u = Δφ/(duration/2).
#[inline]
pub fn cam_lift(g: &ValveGear, dphi_deg: f64) -> (f64, f64, f64) {
    lobe_lift(g.max_lift_m, g.duration_deg, g.shape_n, dphi_deg)
}

/// Lift and derivatives of a lobe L·(1−u²)ⁿ of lift `l` (m), zero-lift duration `duration_deg` and flank exponent `n`
/// (≥ 2; 3 = [`cam_lift`], C² at the seat; smaller = fuller flanks, shorter low-lift ramps).
#[inline]
pub fn lobe_lift(l: f64, duration_deg: f64, n: f64, dphi_deg: f64) -> (f64, f64, f64) {
    let h = (0.5 * duration_deg).to_radians();
    let u = dphi_deg.to_radians() / h;
    if u.abs() >= 1.0 {
        return (0.0, 0.0, 0.0);
    }
    let w = 1.0 - u * u;
    if n == 3.0 {
        return (l * w * w * w, -6.0 * l * u * w * w / h, l * w * (30.0 * u * u - 6.0) / (h * h));
    }
    let wn2 = if n == 2.0 { 1.0 } else { w.powf(n - 2.0) };
    (l * wn2 * w * w, -2.0 * n * l * u * wn2 * w / h, n * l * wn2 * (4.0 * (n - 1.0) * u * u - 2.0 * w) / (h * h))
}

/// Exhaust AVS cam piece of one cylinder ([`AvsHw`]) and its two actuator pins.
#[derive(Debug, Clone, Copy, Serialize, Default)]
pub struct CamPiece {
    /// Axial position: 0 = standard (large) lobes under the followers … 1 = LFT_1 (small) lobes.
    pub pos: f64,
    /// Pins [to LFT_1, to standard]: extension 0 (in) … 1 (fully out, latched by the actuator's magnet) and the time
    /// since the coil was energised, s (< 0 = not energised).
    pub pin: [f64; 2],
    pub on_s: [f64; 2],
    /// Crank angle the pin has run in its groove since the entry, °CA (< 0 = not in the groove).
    pub groove_deg: [f64; 2],
    /// Pins pushed home by a groove run-out since start (each induces the voltage pulse the firmware's switching
    /// diagnosis looks for) and the plant time of the last one, s.
    pub recoils: [u32; 2],
    pub recoil_t_s: [f64; 2],
    /// Completed switchings and pins that missed the groove entry (not fully out) since start.
    pub shifts: u32,
    pub misses: u32,
}

impl CamPiece {
    /// Advances the piece of one cylinder from local crank angle `phi0` to `phi1` (°CA, 0..720 after its firing TDC)
    /// with the actuator coils `on` [to LFT_1, to standard], `shift` = exhaust cam retard, °CA; `t_s` = time at the end.
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, hw: &AvsHw, on: [bool; 2], phi0: f64, phi1: f64, shift: f64, battery_v: f64, dt: f64, t_s: f64) {
        let entry = crate::cylinder::crossed(phi0, phi1, (hw.groove_entry_deg + shift).rem_euclid(720.0));
        let dphi = (phi1 - phi0).rem_euclid(720.0).min(360.0);
        // pin travel ∝ coil force ∝ current ∝ voltage (copper coil, magnet circuit far from saturation — UNVERIFIED)
        let travel = hw.pin_travel_s * 13.5 / battery_v.max(6.0);
        for (k, &on_k) in on.iter().enumerate() {
            self.on_s[k] = if on_k { self.on_s[k].max(0.0) + dt } else { -1.0 };
            let target = if k == 0 { 1.0 } else { 0.0 };
            if self.pos == target && self.groove_deg[k] < 0.0 {
                // the groove of this direction lies under the pin only while the piece sits at the opposite end; over a
                // piece already at its target the pin stops on the land, never latches and falls back when the current
                // ends — no shift, no recoil (UNVERIFIED geometry; the firmware's switch correction commands a direction
                // it does not know the piece is in and masks the switching diagnosis of that pulse, FR p.26701–26702)
                self.pin[k] = 0.0;
            } else if on_k && self.on_s[k] > hw.pin_dead_s {
                self.pin[k] = (self.pin[k] + dt / travel).min(1.0);
            } else if !on_k && self.pin[k] < 1.0 {
                self.pin[k] = 0.0; // not latched yet: the return spring pulls it back
            }
            if entry && self.groove_deg[k] < 0.0 {
                if self.pin[k] >= 1.0 && self.pos != target {
                    self.groove_deg[k] = 0.0;
                } else if self.pin[k] > 0.0 && self.pin[k] < 1.0 {
                    self.misses += 1; // still moving: it lands on the land and enters one camshaft turn later
                }
            }
            if self.groove_deg[k] >= 0.0 {
                let g = self.groove_deg[k] + dphi;
                self.groove_deg[k] = g;
                if g < hw.shift_deg {
                    let x = g / hw.shift_deg;
                    self.pos = if k == 0 { x } else { 1.0 - x };
                } else if self.pos != target {
                    self.pos = target;
                    self.shifts += 1;
                }
                // run-out: the groove floor rises and pushes the pin home
                if g >= hw.eject_deg - hw.groove_entry_deg {
                    (self.pin[k], self.groove_deg[k]) = (0.0, -1.0);
                    self.recoils[k] += 1;
                    self.recoil_t_s[k] = t_s;
                }
            }
        }
    }

    /// Exhaust lobe under the followers: (lift m, zero-lift duration °CA, centre °CA at phaser rest).
    #[inline]
    pub fn lobe(&self, g: &ValveGear, hw: &AvsHw) -> (f64, f64, f64, f64) {
        if self.pos < 0.5 { (g.max_lift_m, g.duration_deg, g.centerline_deg, g.shape_n) } else { (hw.small_lift_m, hw.small_duration_deg, hw.small_centerline_deg, hw.small_shape_n) }
    }
}

/// Angle from the lobe centre wrapped to (−360, 360].
#[inline]
pub fn lobe_angle(phi_local_deg: f64, centre_deg: f64) -> f64 {
    let d = (phi_local_deg - centre_deg).rem_euclid(720.0);
    if d > 360.0 { d - 720.0 } else { d }
}

/// Derived constant geometry of one valve type.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct ValveGeometry {
    /// Clearance between the closed valve head and the piston pocket at TDC, along the valve axis, m.
    pub pocket_m: f64,
    pub face_area_m2: f64,
    pub throat_area_m2: f64,
    pub cos_incl: f64,
    /// Elastic bending energy capacity of the stem, J, and the plastic-hinge force, N.
    pub u_elastic_j: f64,
    pub f_yield_n: f64,
}

impl ValveGeometry {
    fn new(g: &ValveGear, kin: &crate::crank_train::Kinematics, min_clear: f64, peak_shift_deg: f64) -> Self {
        let d = g.stem_diameter_m;
        let i = PI * d.powi(4) / 64.0;
        let m_y = VALVE_STEEL.yield_pa(150.0) * i / (0.5 * d);
        let f_y = m_y / STEM_LEVER_M;
        let defl = f_y * STEM_LEVER_M.powi(3) / (3.0 * VALVE_STEEL.youngs_pa * i);
        let cos_incl = g.inclination_deg.to_radians().cos();
        // Pocket depth = design margin + worst valve intrusion over the gas-exchange TDC with the
        // phaser at the end of its authority (maximum overlap).
        let centre = g.centerline_deg + g.phaser_dir * peak_shift_deg;
        let mut worst = f64::MIN;
        let mut phi = 250.0;
        while phi <= 470.0 {
            let (x, _, _) = cam_lift(g, lobe_angle(phi, centre));
            let s = kin.eval(phi.to_radians(), 0.0);
            worst = worst.max(x - s.x * cos_incl);
            phi += 0.25;
        }
        Self {
            pocket_m: min_clear + worst.max(0.0),
            face_area_m2: PI / 4.0 * g.head_diameter_m.powi(2),
            throat_area_m2: PI / 4.0 * ((0.9 * g.head_diameter_m).powi(2) - d * d),
            cos_incl,
            u_elastic_j: 0.5 * f_y * defl,
            f_yield_n: f_y,
        }
    }

    /// Effective flow area (Cd·A) of `n` valves at lift `x` (m).
    #[inline]
    pub fn cda(&self, g: &ValveGear, x: f64) -> f64 {
        if x <= 0.0 {
            return 0.0;
        }
        g.count * g.cd * (PI * g.head_diameter_m * x).min(self.throat_area_m2)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Default)]
pub struct Phaser {
    /// Displacement from the locked rest position, °CA (0..authority).
    pub angle_deg: f64,
    /// Spool position −1 (vent / passive) … +1 (feed / active), lagging the coil current.
    pub spool: f64,
}

impl Phaser {
    /// Vane phaser behind a proportional 4/3 spool valve (N205 / N318). The low-side switch applies the battery
    /// voltage for the PWM duty: coil current I = d·V_B / R(T_coil) (copper, coil at oil temperature); the spool
    /// balances magnet force against its spring, so it is centred (oil trapped, hold) at the null current and moves
    /// fully over ±`band_a`, lagging by the valve's response time; within the spool overlap no oil flows and the
    /// trapped oil holds the vanes. Rotor speed ∝ spool opening × √(oil supply
    /// pressure) × [`PhaserHw::visc_factor`] (orifice flow; leakage and the viscous film of a cold engine slow it); cam
    /// friction torque drifts it back towards rest. Locking pin at rest until the spool pushes it with oil pressure.
    /// `None`/unpowered = spring → passive stop (rest).
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, hw: &PhaserHw, range: f64, duty: Option<f64>, powered: bool, p_oil: f64, battery_v: f64, oil_c: f64, dt: f64) {
        let d = if powered { duty.unwrap_or(0.0).clamp(0.0, 1.0) } else { 0.0 };
        let i = d * battery_v / hw.coil_r(oil_c);
        let target = ((i - hw.null_current(oil_c)) / hw.band_a).clamp(-1.0, 1.0);
        self.spool += (target - self.spool) * (dt / hw.response_s).min(1.0);
        if self.angle_deg <= 0.0 && (p_oil < 1.0e5 || self.spool <= 0.0) {
            self.angle_deg = 0.0;
            return;
        }
        let supply = ((p_oil - 0.5e5).max(0.0) / 2.5e5).min(1.5).sqrt();
        let visc = hw.visc_factor(oil_c);
        let open = self.spool.signum() * ((self.spool.abs() - hw.overlap).max(0.0) / (1.0 - hw.overlap));
        // centred spool: oil trapped in both chamber sets → no drift (the vanes are hydraulically locked)
        let drift = if open == 0.0 { 0.0 } else { hw.drift_deg_s };
        let rate = hw.rate_deg_s * supply * visc * open - drift;
        self.angle_deg = (self.angle_deg + rate * dt).clamp(0.0, range);
    }
}

#[derive(Debug, Clone, Copy, Serialize, Default)]
pub struct Valve {
    pub x: f64,
    pub v: f64,
    /// Follower off the cam (floating).
    pub free: bool,
    /// HLA extension beyond zero lash, m.
    pub hla_m: f64,
    /// Permanent stem bend, m.
    pub bend_m: f64,
    pub broken: bool,
    /// Cam contact force, N.
    pub cam_force_n: f64,
    /// Largest valve-to-follower separation in the running / last cycle, m.
    pub float_run_m: f64,
    pub float_last_m: f64,
    pub piston_hits: u32,
    pub seat_v_max: f64,
    /// Seat impact velocity in the last step (0 = none), m/s. Read-only output for the acoustic probe.
    pub seat_hit_v: f64,
}

impl Valve {
    /// Seat leak gap of a bent valve, m (tilted head).
    #[inline]
    pub fn leak_gap(&self, g: &ValveGear) -> f64 {
        if self.broken { g.max_lift_m } else { self.bend_m * g.head_diameter_m / (2.0 * GUIDE_LEVER_M) }
    }

    /// One valve over `dt`. `cam0`/`cam1` = cam lift at the start/end of the step, (v_c, a_c) at the end.
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, g: &ValveGear, cam0: f64, cam1: f64, v_c: f64, a_c: f64, f_gas: f64, p_oil: f64, dt: f64) {
        self.seat_hit_v = 0.0;
        let m = g.moving_mass_kg;
        let force = |x: f64, v: f64| -(g.spring_preload_n + g.spring_rate_n_m * x + g.damping_ns_m * v + f_gas);
        if !self.free {
            let x = cam1 + self.hla_m;
            let fc = m * a_c + g.spring_preload_n + g.spring_rate_n_m * x + g.damping_ns_m * v_c + f_gas;
            if cam1 > 0.0 && fc < 0.0 {
                self.free = true;
                self.x = x;
                self.v = v_c;
            } else {
                self.x = x;
                self.v = v_c;
                self.cam_force_n = if x > 0.0 { fc.max(0.0) } else { 0.0 };
                self.hla_m = (self.hla_m - HLA_LEAK_M_NS * self.cam_force_n * dt).max(0.0);
                return;
            }
        }
        self.cam_force_n = 0.0;
        let n = (dt / FREE_FLIGHT_DT).ceil().clamp(1.0, 10_000.0) as usize;
        let h = dt / n as f64;
        let pump = HLA_PUMP_M_S * (p_oil.max(0.0) / 3e5).sqrt();
        for j in 0..n {
            let xc = cam0 + (cam1 - cam0) * (j + 1) as f64 / n as f64 + self.hla_m;
            self.v += force(self.x, self.v) / m * h;
            self.x += self.v * h;
            let gap = self.x - xc;
            if gap > 0.0 {
                self.float_run_m = self.float_run_m.max(gap);
                self.hla_m = (self.hla_m + (pump * h).min(gap)).min(HLA_TRAVEL_M);
            }
            let xc = cam0 + (cam1 - cam0) * (j + 1) as f64 / n as f64 + self.hla_m;
            if self.x <= xc && xc > 0.0 {
                self.free = false;
                self.x = xc;
                self.v = v_c;
                break;
            }
            if self.x <= 0.0 {
                let v_imp = -self.v;
                self.seat_v_max = self.seat_v_max.max(v_imp);
                self.seat_hit_v = self.seat_hit_v.max(v_imp);
                self.x = 0.0;
                self.v = SEAT_RESTITUTION * v_imp;
                if self.v < 0.05 {
                    self.v = 0.0;
                    self.free = false;
                    break;
                }
            }
        }
    }

    /// Piston contact check. `z` = piston-face position along the valve axis (pocket + x_p·cosα),
    /// `z_dot` its velocity. Returns true on contact.
    pub fn piston_contact(&mut self, geo: &ValveGeometry, g: &ValveGear, z: f64, z_dot: f64) -> bool {
        let gap = z - (self.x - self.bend_m);
        if gap >= 0.0 || self.broken {
            return false;
        }
        self.piston_hits += 1;
        if !self.free {
            // Driven by cam + hydraulically locked HLA: the stem must give way by the interference.
            self.bend_m += -gap;
        } else {
            let v_rel = self.v - z_dot;
            if v_rel > 0.0 {
                let e = 0.5 * g.moving_mass_kg * v_rel * v_rel * (1.0 - PISTON_RESTITUTION * PISTON_RESTITUTION);
                if e > geo.u_elastic_j {
                    self.bend_m += (e - geo.u_elastic_j) / geo.f_yield_n;
                }
                self.v = z_dot - PISTON_RESTITUTION * v_rel;
            }
            self.x = z + self.bend_m;
        }
        if self.bend_m > BEND_BREAK_M {
            self.broken = true;
        }
        true
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Valvetrain {
    /// Intake (advance) and exhaust (retard) phasers.
    pub phaser: [Phaser; 2],
    /// Per cylinder [intake, exhaust] (both valves of a pair move together).
    pub valve: [[Valve; 2]; 4],
    pub geo: [ValveGeometry; 2],
    /// Exhaust AVS cam piece per cylinder (unused without `Mechanics::avs`).
    pub avs: [CamPiece; 4],
}

impl Valvetrain {
    pub fn new(p: &EngineProfile, kin: &crate::crank_train::Kinematics) -> Self {
        let m = &p.mech;
        let pos = m.avs.as_ref().map_or(0.0, |a| a.initial_pos);
        Self {
            avs: [CamPiece { pos, on_s: [-1.0; 2], groove_deg: [-1.0; 2], ..Default::default() }; 4],
            phaser: [Phaser::default(); 2],
            valve: [[Valve::default(); 2]; 4],
            geo: [
                ValveGeometry::new(&m.intake, kin, m.valve_piston_min_clearance_m, m.intake.phaser_range_deg),
                ValveGeometry::new(&m.exhaust, kin, m.valve_piston_min_clearance_m, m.exhaust.phaser_range_deg),
            ],
        }
    }

    /// Lobe centre of valve type `k` (0 intake, 1 exhaust), °CA after the cylinder's firing TDC.
    #[inline]
    pub fn centre(&self, g: &ValveGear, k: usize) -> f64 {
        g.centerline_deg + g.phaser_dir * self.phaser[k].angle_deg
    }

    /// Active lobe of valve type `k` of cylinder `c`: (max lift m, zero-lift duration °CA, centre °CA incl. phasing, flank
    /// exponent).
    #[inline]
    pub fn lobe(&self, p: &EngineProfile, c: usize, k: usize) -> (f64, f64, f64, f64) {
        let g = if k == 0 { &p.mech.intake } else { &p.mech.exhaust };
        let (l, d, cl, n) = match (&p.mech.avs, k) {
            (Some(hw), 1) => self.avs[c].lobe(g, hw),
            _ => (g.max_lift_m, g.duration_deg, g.centerline_deg, g.shape_n),
        };
        (l, d, cl + g.phaser_dir * self.phaser[k].angle_deg, n)
    }

    /// Cam shift seen by the cam sensor, °CRK (positive = later).
    pub fn cam_shift_deg(&self, p: &EngineProfile, k: usize) -> f64 {
        let g = if k == 0 { &p.mech.intake } else { &p.mech.exhaust };
        g.phaser_dir * self.phaser[k].angle_deg
    }

    /// Advances both valves of cylinder `c` from local angle `phi0` to `phi1` (deg) and checks piston
    /// contact; `avs_on` = the cylinder's AVS actuator coils [to LFT_1, to standard]. Returns the crank torque needed to
    /// drive the cams (Nm, positive = load on the crank).
    #[allow(clippy::too_many_arguments)]
    pub fn step_cylinder(&mut self, p: &EngineProfile, c: usize, phi0: f64, phi1: f64, omega: f64, s: &Slider, p_cyl: f64, p_port: [f64; 2], p_oil: f64, avs_on: [bool; 2], battery_v: f64, dt: f64, t_s: f64) -> f64 {
        if let Some(hw) = &p.mech.avs {
            let shift = self.cam_shift_deg(p, 1);
            self.avs[c].step(hw, avs_on, phi0, phi1, shift, battery_v, dt, t_s);
        }
        let mut torque = 0.0;
        for (k, &pp) in p_port.iter().enumerate() {
            let g = if k == 0 { &p.mech.intake } else { &p.mech.exhaust };
            let (l, d, centre, n) = self.lobe(p, c, k);
            let (x1, dx1, ddx1) = lobe_lift(l, d, n, lobe_angle(phi1, centre));
            let geo = self.geo[k];
            let vl = &mut self.valve[c][k];
            let face = if vl.x < 1e-3 { geo.face_area_m2 } else { 0.3 * geo.face_area_m2 };
            let f_gas = (p_cyl - pp) * face;
            let cam_v = dx1 * omega;
            let cam_a = ddx1 * omega * omega;
            let x0 = if vl.free
                || (x1 > 0.0
                    && g.moving_mass_kg * cam_a + g.spring_preload_n + g.spring_rate_n_m * (x1 + vl.hla_m) + g.damping_ns_m * cam_v + f_gas < 0.0)
            {
                lobe_lift(l, d, n, lobe_angle(phi0, centre)).0
            } else {
                0.0
            };
            vl.step(g, x0, x1, cam_v, cam_a, f_gas, p_oil, dt);
            torque += g.count * vl.cam_force_n * dx1;
            let z = geo.pocket_m + s.x * geo.cos_incl;
            let z_dot = s.dx * omega * geo.cos_incl;
            vl.piston_contact(&geo, g, z, z_dot);
        }
        torque
    }

    /// Effective open lift of valve type `k` of cylinder `c` for gas flow (lift + bent-valve leak gap).
    #[inline]
    pub fn flow_lift(&self, p: &EngineProfile, c: usize, k: usize) -> f64 {
        let g = if k == 0 { &p.mech.intake } else { &p.mech.exhaust };
        let v = &self.valve[c][k];
        v.x.max(0.0) + v.leak_gap(g)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crank_train::Kinematics;

    fn spin(p: &EngineProfile, rpm: f64, cycles: usize) -> Valvetrain {
        let kin = Kinematics::new(p);
        let mut vt = Valvetrain::new(p, &kin);
        let w = rpm * PI / 30.0;
        let dphi: f64 = 0.5;
        let dt = dphi.to_radians() / w;
        let mut phi: f64 = 0.0;
        for _ in 0..(cycles as f64 * 720.0 / dphi) as usize {
            let p1 = (phi + dphi).rem_euclid(720.0);
            let s = kin.eval(p1.to_radians(), 0.0);
            vt.step_cylinder(p, 0, phi, p1, w, &s, 1e5, [1e5, 1e5], 3e5, [false; 2], 14.0, dt, 0.0);
            phi = p1;
        }
        vt
    }

    /// Phaser hydraulics against what the SC800S50 firmware assumes about this valve (docs/SIMOS18_GPTA_IO_RE.md
    /// §12.8): the intake hold duty it commands at 14 V and a 90 °C coil (`ip_pwm_hld_vcp` 47.6 % + offset −7 % + NVM
    /// adaptation −3.4 % = 37.2 %) keeps the cam still; full drive moves the hot engine at the 550–1100 °CRK/s of
    /// `ip_vel_ivs_act_vcp` (0.9–1.8 ms/°), a 30 °C engine at 0.4–0.7 of that and a −20 °C engine 6–9× slower; the
    /// low-pressure oil-pump stage slows it; de-energised it returns to the locked rest.
    #[test]
    fn phaser_holds_at_the_firmware_hold_duty_and_moves_at_its_speed() {
        let hw = EngineProfile::ea888_18t_cjsa_is12().mech.intake.phaser;
        let run = |ph: &mut Phaser, duty: f64, p: f64, oil_c: f64, s: f64| {
            for _ in 0..(s / 1e-3) as usize {
                ph.step(&hw, 60.0, Some(duty), true, p, 14.0, oil_c, 1e-3);
            }
        };
        let mut ph = Phaser { angle_deg: 20.0, spool: 0.0 };
        run(&mut ph, 0.476 - 0.07 - 0.034, 3e5, 90.0, 1.0);
        assert!((ph.angle_deg - 20.0).abs() < 2.0, "hold: {}", ph.angle_deg);
        let mut hot = Phaser::default();
        run(&mut hot, 0.9, 3e5, 90.0, 0.05);
        let a0 = hot.angle_deg;
        run(&mut hot, 0.9, 3e5, 90.0, 0.02);
        let v_hot = (hot.angle_deg - a0) / 0.02;
        assert!((550.0..1100.0).contains(&v_hot), "hot {v_hot} °/s");
        let mut warm = Phaser::default();
        run(&mut warm, 0.9, 3e5, 30.0, 0.05);
        let a0 = warm.angle_deg;
        run(&mut warm, 0.9, 3e5, 30.0, 0.02);
        let r30 = (warm.angle_deg - a0) / 0.02 / v_hot;
        assert!((0.4..0.7).contains(&r30), "30 °C / 90 °C speed {r30}");
        let mut cold = Phaser::default();
        run(&mut cold, 0.9, 3e5, -20.0, 0.05);
        let a0 = cold.angle_deg;
        run(&mut cold, 0.9, 3e5, -20.0, 0.1);
        let v_cold = (cold.angle_deg - a0) / 0.1;
        assert!((90.0..250.0).contains(&v_cold), "cold {v_cold} °/s");
        let mut low = Phaser::default();
        run(&mut low, 0.9, 1.5e5, 90.0, 0.05);
        let a0 = low.angle_deg;
        run(&mut low, 0.9, 1.5e5, 90.0, 0.02);
        assert!((low.angle_deg - a0) / 0.02 < v_hot / 1.2, "low oil pressure stage is slower");
        for _ in 0..1000 {
            hot.step(&hw, 60.0, None, true, 3e5, 14.0, 90.0, 1e-3);
        }
        assert_eq!(hot.angle_deg, 0.0, "de-energised: back to the locked rest");
    }

    /// AVS cam piece driven the way the firmware drives it (current for `ip_t_swi_vvl_acr` ending at the hit position
    /// 292.1 ° after the firing TDC): the piece moves to the commanded lobe on the base circle and the pin is pushed home
    /// inside the firmware's recoil window (hit + 600 … 740 °); a pulse shorter than dead time + travel moves nothing; a
    /// pin commanded towards the lobe the piece already sits on gives no recoil.
    #[test]
    fn avs_cam_piece_switches_on_the_base_circle_and_recoils_in_the_diagnosis_window() {
        let hw = EngineProfile::ea888_18t_cjsa_is12().mech.avs.unwrap();
        let run = |piece: &mut CamPiece, k: usize, t_swi_s: f64, rpm: f64| -> Vec<(f64, f64)> {
            let w = rpm * 6.0; // °CA/s
            let (dphi, mut phi, mut t, mut trace) = (0.5, 0.0f64, 0.0, Vec::new());
            let on_from = 292.1 - t_swi_s * w;
            let mut abs = 0.0f64;
            while abs < 2160.0 {
                let p1 = (phi + dphi).rem_euclid(720.0);
                abs += dphi;
                t += dphi / w;
                let on = abs >= on_from + 720.0 && abs < 292.1 + 720.0;
                let mut cmd = [false; 2];
                cmd[k] = on;
                let r = piece.recoils[k];
                piece.step(&hw, cmd, phi, p1, 0.0, 14.0, dphi / w, t);
                if piece.recoils[k] > r {
                    trace.push((abs - 720.0, piece.pos));
                }
                phi = p1;
            }
            trace
        };
        // standard → LFT_1 at 2000 rpm with the calibrated 36.5 ms
        let mut p = CamPiece { pos: 0.0, on_s: [-1.0; 2], groove_deg: [-1.0; 2], ..Default::default() };
        let rec = run(&mut p, 0, 36.5e-3, 2000.0);
        assert_eq!(p.pos, 1.0, "{p:?}");
        assert_eq!(rec.len(), 1, "{rec:?}");
        assert!((292.1 + 600.0..292.1 + 740.0).contains(&rec[0].0), "recoil at {:?} °CA after the firing TDC", rec[0].0);
        // back to standard at 5000 rpm with 17 ms
        let rec = run(&mut p, 1, 17e-3, 5000.0);
        assert_eq!((p.pos, rec.len()), (0.0, 1));
        // too short to get the pin out: nothing moves
        let rec = run(&mut p, 0, 8e-3, 2000.0);
        assert_eq!((p.pos, rec.len(), p.shifts), (0.0, 0, 2));
        // already on the standard lobe: no shift, no recoil
        let rec = run(&mut p, 1, 36.5e-3, 2000.0);
        assert_eq!((p.pos, rec.len(), p.shifts), (0.0, 0, 2));
        // that pin did not stay out: the next switch to LFT_1 is not undone by it a turn later
        assert_eq!(p.pin[1], 0.0);
        run(&mut p, 0, 36.5e-3, 2000.0);
        let rec = run(&mut p, 0, 0.0, 2000.0);
        assert_eq!((p.pos, rec.len(), p.shifts), (1.0, 0, 3));
    }

    #[test]
    fn cam_profile_derivatives_are_consistent() {
        let g = EngineProfile::ea888_18t_cjsa_is12().mech.intake;
        let h = 1e-4;
        for &d in &[-100.0, -40.0, 10.0, 70.0] {
            let (a, _, _) = cam_lift(&g, d - h);
            let (b, _, _) = cam_lift(&g, d + h);
            let (_, dx, _) = cam_lift(&g, d);
            assert!(((b - a) / (2.0 * h.to_radians()) - dx).abs() < 1e-6);
        }
        assert!((cam_lift(&g, 0.0).0 - g.max_lift_m).abs() < 1e-12);
    }

    #[test]
    fn valves_follow_the_cam_at_7000_and_float_and_pump_up_at_8500() {
        let p = EngineProfile::ea888_18t_cjsa_is12();
        let ok = spin(&p, 7000.0, 20);
        assert!(ok.valve[0].iter().all(|v| v.float_run_m < 1e-5 && v.hla_m < 1e-6), "{:?}", ok.valve[0]);
        let over = spin(&p, 8500.0, 200);
        let v = over.valve[0];
        assert!(v.iter().any(|v| v.float_run_m > 5e-5), "{:?}", v);
        assert!(v.iter().any(|v| v.hla_m > 1e-4), "HLA must pump up: {:?}", v);
    }
}
