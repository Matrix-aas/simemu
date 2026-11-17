//! Read-only acoustic probe of the engine model.
//!
//! When `enabled`, [`crate::engine::EngineState::step`] appends one [`CrankFrame`] per crank-angle step
//! (≤ 1 °CA / ≤ 50 µs, i.e. ≥ 20 kHz) with quantities the step has already computed, and one [`Impact`] per
//! discrete mechanical event (injector needle, HPFP valves, valve seating in free flight). Nothing here is
//! read back by the plant: enabling the probe changes no result (the plant tests and the board determinism
//! digests run with it disabled, the default). The consumer (`simemu-acoustics`) drains both vectors after
//! every plant step (`std::mem::take` / `drain`), so their capacity is reused.

/// Quantities at the end of one crank-angle step. `f32` is enough for acoustics.
#[derive(Debug, Clone, Copy, Default)]
pub struct CrankFrame {
    /// Plant time at the end of the step, s, and the step length, s.
    pub t_s: f64,
    pub h: f32,
    /// Crank angle in the 720° cycle (cylinder 1 firing TDC = 0) and crank speed, rad/s.
    pub crank_deg: f32,
    pub omega: f32,
    /// Gas + reciprocating-inertia torque on the crank, Nm (its reaction rolls the block on its mounts).
    pub t_gas_nm: f32,
    /// Mean cylinder pressure, Pa abs, and bulk gas temperature, K.
    pub p_cyl: [f32; 4],
    pub t_cyl: [f32; 4],
    /// First circumferential knock-mode envelope, Pa (`Cylinder::knock_amp_pa`).
    pub knock_pa: [f32; 4],
    /// Exhaust-valve mass flow (+ = cylinder → port) and intake-valve mass flow (+ = port → cylinder), kg/s.
    pub m_ex: [f32; 4],
    pub m_in: [f32; 4],
    /// Cam contact force of the intake + exhaust valve pairs of each cylinder (count × force), N.
    pub valve_n: [f32; 4],
    /// Exhaust manifold → turbine + wastegate, throttle, compressor, diverter-valve mass flows, kg/s.
    pub m_out: f32,
    pub m_thr: f32,
    pub m_c: f32,
    pub m_dv: f32,
    /// Exhaust manifold, intake manifold and boost pressures, Pa abs; manifold gas temperatures, K.
    pub p_em: f32,
    pub t_em: f32,
    pub p_im: f32,
    pub t_im: f32,
    pub p_boost: f32,
    /// Pressure downstream of the turbine (exhaust back pressure), Pa abs.
    pub p4: f32,
    /// Camshaft drive torque (timing-chain load) and HPFP drive torque at the crank, Nm.
    pub cam_nm: f32,
    pub hpfp_nm: f32,
    /// Rail pressure, Pa abs; battery voltage, V.
    pub rail_pa: f32,
    pub battery_v: f32,
    /// Turbocharger rotor speed, rad/s; compressor pressure ratio over its zero-flow (surge-side) limit.
    pub turbo_rad_s: f32,
    pub pr_c_rel: f32,
    /// Starter: armature speed rad/s, current A, relay closed, pinion driving the ring gear.
    pub starter_rad_s: f32,
    pub starter_a: f32,
    pub starter_on: bool,
    pub starter_engaged: bool,
}

/// Discrete mechanical events with the physical quantity that sets their strength.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImpactKind {
    /// Injector needle/armature reaches full lift (fuel starts flowing). `mag` = rail pressure, Pa.
    InjectorOpen,
    /// Needle seats (fuel stops). `mag` = rail − cylinder pressure, Pa.
    InjectorClose,
    /// HPFP quantity-control valve N276 closes after its pull-in delay. `mag` = supply voltage, V.
    HpfpValveClose,
    /// N276 reopens at plunger TDC (chamber pressure collapses). `mag` = rail pressure, Pa.
    HpfpValveOpen,
    /// HPFP outlet check valve opens (chamber pressure reaches the rail). `mag` = rail pressure, Pa.
    HpfpOutletOpen,
    /// Valve hits its seat in free flight (float / bounce). `mag` = impact velocity, m/s; `sub` = 0 intake, 1 exhaust.
    ValveSeat,
}

#[derive(Debug, Clone, Copy)]
pub struct Impact {
    pub t_s: f64,
    pub kind: ImpactKind,
    /// Cylinder 0..3 (0 for the pump).
    pub cyl: u8,
    pub sub: u8,
    pub mag: f32,
}

#[derive(Debug, Clone, Default)]
pub struct Probe {
    pub enabled: bool,
    pub frames: Vec<CrankFrame>,
    pub impacts: Vec<Impact>,
    /// HPFP outlet valve open in the current stroke (event detection only).
    pub hpfp_out: bool,
}

impl Probe {
    #[inline]
    pub fn impact(&mut self, t_s: f64, kind: ImpactKind, cyl: usize, sub: u8, mag: f64) {
        if self.enabled {
            self.impacts.push(Impact { t_s, kind, cyl: cyl as u8, sub, mag: mag as f32 });
        }
    }
}
