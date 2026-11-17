//! V401 hydraulic pump of the DQ200_G2 mechatronic: the brushless-motor driver chip on SSC0 / CS P3.6
//! with its bridge, the three-phase motor and the gear pump shaft, as the TC1784 sees them.
//!
//! Firmware evidence (`docs/DQ200_IO_MAP.md` §4; REA/Ghidra on the unmodified image):
//! - SPI (VERIFIED-S): SSC0, 16-bit words, LSB first (Spi channel config 0x8013C4F4, channels 0..3 =
//!   0x0F), 2 MHz (job BR 14), four jobs per cycle (0x800D94E2): reg3 = word | 3, reg6 = word | 6,
//!   reg2 = (word | 2) & !4, then a status request 0 / 1 (0x800D931A). The answer to frame N is shifted
//!   out during frame N+1; the firmware checks that register answers echo the written word in bits
//!   15:2 (reg3), 15:4 (reg6), 15:3 (reg2) (0x800D93BE), stores the status answer (0x800D95D0: groups
//!   0x254 / 0x4A8 / 0x902, bits 0, 12..15 counted as faults) and writes the read words back every
//!   cycle. Reg3 bit 14 selects the offset phase of the current-sense calibration: the firmware sets
//!   it, averages FADC channel 0 as the zero-current level, clears it and averages again (0x800D9C12,
//!   0x800D9CC8: `I[mA] = (mean - mean_offset) * 10 * (3300 * 1000 / 500) / (2 * 1023)`).
//! - Port P5 (VERIFIED-S, pin table 0x8013ABC0): P5.3 = bridge enable (set at the start 0x800DA228,
//!   cleared at the stop 0x800DA24E); P5.9 = commutation input, toggled once per commutation step by
//!   the LTCA2 LTC23 step interrupt (0x800DB5FE -> 0x800DB0BC); P5.4 set once at driver init
//!   (0x800D9258), P5.5 / P5.10 written by the start/stop sequences (meaning UNVERIFIED, not modelled).
//! - Rotation feedback (VERIFIED-S): GPTA0 GTC12 captures both edges of IN47 (CTR 0x430); the step
//!   interrupt requires one new capture per step at 40..98 % (35..65 % once synchronised) of the step
//!   interval (0x800DB6BA..0x800DB7DE, windows 0xD0008378..0xD000837E) and then commutates
//!   half the measured interval after it (0x800DB7E0). One edge per 60 deg el. step at mid-step is the
//!   floating-phase back-EMF zero crossing of a six-step drive.
//! - Motor: 4 pole pairs (`0x8013AC04[0]`: step period = f * 60 / (4 * 6 * rpm)); PWM 20 kHz (LTCA2
//!   LTC26..30, period 0x7D0 at 40.02 MHz).
//!
//! The chip itself is NOT identified (no public part matched: 16-bit LSB-first SPI with out-of-frame
//! echo, a step-advance commutation input and a zero-crossing output). Its behaviour here is the generic
//! six-step pre-driver with an external commutation clock and a floating-phase BEMF comparator as
//! described in sensorless BLDC application notes (Microchip AN970 "Using the PIC18F2431 for
//! Sensorless BLDC Motor Control"; ST AN1946 "Sensorless BLDC motor control and BEMF sampling methods";
//! Infineon TLE7183F/TLE987x data sheets for the bridge, charge pump and current-sense structure):
//! every edge of the commutation input advances the bridge one 60 deg el. state (both edges: the
//! firmware toggles once per step and expects one zero crossing per toggle), the comparator output
//! toggles at each zero crossing of the floating phase. UNVERIFIED (chip), every physical constant
//! below is UNVERIFIED and kept as a field of [`BldcPumpHw`].

use std::f64::consts::PI;

/// Physical constants of the driver, motor and gear pump. UNVERIFIED (no data sheet / SSP figures for
/// V401); sized so that the firmware's own start-up calibration (alignment 9..15 % duty for 50 ms,
/// open-loop ramp 240 -> 400 rpm in 7 steps at 15..18 % duty, CAL 0xD0008348..0xD000835A) starts the
/// rotor against the accumulator pressure, and that a 30 A fuse class motor charges the accumulator
/// 40 -> 60 bar in a few seconds (Actronics "TCU DSG7 DQ200": 40..60 bar system pressure).
#[derive(Clone, Copy, Debug)]
pub struct BldcPumpHw {
    /// Pole pairs (firmware speed scaling `0x8013AC04[0] = 4`).
    pub pole_pairs: f64,
    /// Line-to-line resistance of the conducting phase pair incl. MOSFETs and wiring, Ohm.
    pub r_ll_ohm: f64,
    /// Line-to-line inductance, H.
    pub l_ll_h: f64,
    /// Peak line-to-line back-EMF per mechanical rad/s (= torque constant of the conducting pair).
    pub ke_ll: f64,
    /// Rotor + pump gears + coupling, kg m^2.
    pub inertia_kgm2: f64,
    /// Coulomb friction (bearings, shaft seal), N m.
    pub friction_nm: f64,
    /// Comparator threshold on the floating-phase BEMF, V (hysteresis +-).
    pub zc_threshold_v: f64,
    /// Current-sense amplifier: volts per ampere of DC-link current at FADC FAIN0P (firmware scaling
    /// 0x8013ABE0 -> 0.0999 V/A) and its zero-current output.
    pub sense_v_per_a: f64,
    pub sense_zero_v: f64,
}

impl Default for BldcPumpHw {
    fn default() -> Self {
        Self {
            pole_pairs: 4.0,
            r_ll_ohm: 0.12,
            l_ll_h: 90e-6,
            ke_ll: 0.044,
            inertia_kgm2: 4.0e-5,
            friction_nm: 0.03,
            zc_threshold_v: 0.02,
            sense_v_per_a: 1000.0 / (1024.0 / 3.3 * 32.2581),
            sense_zero_v: 0.33,
        }
    }
}

/// Gear-pump load on the shaft (from the plant's pump hardware and the accumulator pressure).
#[derive(Clone, Copy, Debug, Default)]
pub struct PumpLoad {
    /// Pressure torque per pascal of outlet pressure: displacement / (2 pi eta_hm), m^3.
    pub torque_per_pa: f64,
    /// Outlet (accumulator) pressure, Pa.
    pub p_out_pa: f64,
    /// Viscous drag, N m s (scales with the fluid viscosity).
    pub visc_nm_s: f64,
}

/// Floating phase of each bridge state (A = 0, B = 120, C = 240 deg el.) for the sequence
/// A+B-, A+C-, B+C-, B+A-, C+A-, C+B- (state k is centred at 60 + 60 k deg el.).
const FLOAT_DEG: [f64; 6] = [240.0, 120.0, 0.0, 240.0, 120.0, 0.0];

/// Driver chip + motor + pump shaft.
#[derive(Clone, Debug)]
pub struct BldcPump {
    pub hw: BldcPumpHw,
    pub load: PumpLoad,
    // ---- SPI (out-of-frame echo) ----
    /// Register images by address code (2, 3, 6) and the status word.
    pub regs: [u16; 8],
    pub status: u16,
    reply: u16,
    pub frames: u64,
    // ---- pins ----
    /// P5.3 bridge enable, P5.9 commutation input level.
    pub enable: bool,
    comm: bool,
    pub comm_edges: u64,
    /// Bridge state 0..5.
    pub state: u8,
    // ---- electrical / mechanical state ----
    pub i_a: f64,
    pub omega: f64,
    /// Electrical angle, rad (0..2 pi).
    pub theta_e: f64,
    /// Accumulated shaft angle, rad.
    pub shaft_rad: f64,
    /// Comparator output = IN47 level.
    pub zc: bool,
    pub zc_edges: u64,
    /// DC-link (battery) current of the last update, A.
    pub i_dc: f64,
    /// GPTA tick of the last update.
    t: u64,
    /// IN47 edges produced by the last [`BldcPump::advance`]: (GPTA tick, level).
    pub edges: Vec<(u64, bool)>,
}

impl Default for BldcPump {
    fn default() -> Self {
        Self {
            hw: BldcPumpHw::default(),
            load: PumpLoad::default(),
            regs: [0; 8],
            status: 0,
            reply: 0,
            frames: 0,
            enable: false,
            comm: false,
            comm_edges: 0,
            state: 0,
            i_a: 0.0,
            omega: 0.0,
            theta_e: 0.0,
            shaft_rad: 0.0,
            zc: false,
            zc_edges: 0,
            i_dc: 0.0,
            t: 0,
            edges: Vec::with_capacity(8),
        }
    }
}

/// Longest integration step, s (electrical time constant L/R ~ 0.7 ms, 20 kHz PWM averaged).
const MAX_STEP_S: f64 = 25e-6;

impl BldcPump {
    /// One 16-bit SPI frame on CS P3.6: returns the answer to the previous frame and latches this one.
    pub fn xfer16(&mut self, tx: u32) -> u32 {
        let w = tx as u16;
        let out = self.reply;
        self.frames += 1;
        // address codes in the low bits (0x800D931A): ..11 reg3, .110 reg6, .010 reg2, ..0x status
        let addr = if w & 3 == 3 {
            3
        } else if w & 2 != 0 {
            (w & 7) as usize
        } else {
            0
        };
        self.reply = if addr == 0 {
            self.status
        } else {
            self.regs[addr] = w;
            w
        };
        out as u32
    }

    /// Current-sense offset phase (reg3 bit 14, see the module doc).
    pub fn sense_offset_mode(&self) -> bool {
        self.regs[3] & 0x4000 != 0
    }

    /// Pin levels from the MCU (P5.3 enable, P5.9 commutation): every commutation edge advances the
    /// bridge state once.
    pub fn set_pins(&mut self, enable: bool, comm: bool) {
        if comm != self.comm {
            self.comm = comm;
            self.comm_edges += 1;
            self.state = (self.state + 1) % 6;
        }
        self.enable = enable;
    }

    /// Floating-phase BEMF of bridge state `k`, V.
    fn e_float(&self, k: u8, theta: f64, omega: f64) -> f64 {
        self.hw.ke_ll / 3f64.sqrt() * omega * (theta - FLOAT_DEG[k as usize].to_radians()).sin()
    }

    fn idle(&self) -> bool {
        !self.enable && self.omega == 0.0 && self.i_a == 0.0
    }

    /// Integrate to GPTA tick `to` (`f_gpta` Hz) with bridge supply `v_bat` and PWM duty `duty`; the
    /// comparator edges of the interval are left in [`BldcPump::edges`].
    pub fn advance(&mut self, to: u64, f_gpta: f64, v_bat: f64, duty: f64) {
        self.edges.clear();
        let h = self.hw;
        if to <= self.t
            || !(f_gpta.is_finite()
                && f_gpta > 0.0
                && v_bat.is_finite()
                && duty.is_finite()
                && h.r_ll_ohm.is_finite()
                && h.r_ll_ohm > 0.0
                && h.l_ll_h.is_finite()
                && h.l_ll_h > 0.0
                && h.inertia_kgm2.is_finite()
                && h.inertia_kgm2 > 0.0
                && h.pole_pairs.is_finite()
                && h.pole_pairs > 0.0)
        {
            self.t = self.t.max(to);
            return;
        }
        if self.idle() {
            self.t = to;
            self.i_dc = 0.0;
            return;
        }
        let total = (to - self.t) as f64 / f_gpta;
        let n = (total / MAX_STEP_S).ceil().max(1.0);
        let dt = total / n;
        let tau = h.l_ll_h / h.r_ll_ohm;
        let decay = (-dt / tau).exp();
        let d = if self.enable { duty.clamp(0.0, 1.0) } else { 0.0 };
        let t0 = self.t;
        for s in 0..n as u64 {
            let thc = (60.0 + 60.0 * self.state as f64).to_radians();
            let c = (self.theta_e - thc).cos();
            let e_ll = h.ke_ll * self.omega * c;
            // averaged bridge voltage: PWM on = supply, off = freewheel; disabled = diodes to supply
            let v = if self.enable { d * v_bat } else { -v_bat };
            let i_inf = (v - e_ll) / h.r_ll_ohm;
            self.i_a = (i_inf + (self.i_a - i_inf) * decay).max(0.0);
            let t_e = h.ke_ll * c * self.i_a;
            let l = &self.load;
            let t_res = l.torque_per_pa * l.p_out_pa.max(0.0) + h.friction_nm;
            let w0 = self.omega;
            if w0 == 0.0 && t_e.abs() <= t_res {
                // held by the pressure on the pump gears and static friction
            } else {
                let dir = if w0 != 0.0 { w0.signum() } else { t_e.signum() };
                let w1 = w0 + dt * (t_e - dir * t_res - l.visc_nm_s * w0) / h.inertia_kgm2;
                self.omega = if w0 != 0.0 && w1.signum() != w0.signum() { 0.0 } else { w1 };
            }
            let dth = h.pole_pairs * 0.5 * (w0 + self.omega) * dt;
            let e_before = self.e_float(self.state, self.theta_e, w0);
            self.theta_e = (self.theta_e + dth).rem_euclid(2.0 * PI);
            self.shaft_rad += dth / h.pole_pairs;
            let e_after = self.e_float(self.state, self.theta_e, self.omega);
            // comparator with hysteresis; the edge is placed at the interpolated crossing
            let level = if e_after > h.zc_threshold_v {
                true
            } else if e_after < -h.zc_threshold_v {
                false
            } else {
                self.zc
            };
            if level != self.zc {
                self.zc = level;
                self.zc_edges += 1;
                let th = if level { h.zc_threshold_v } else { -h.zc_threshold_v };
                let frac = if (e_after - e_before).abs() > 1e-12 { ((th - e_before) / (e_after - e_before)).clamp(0.0, 1.0) } else { 1.0 };
                let tick = t0 + ((s as f64 + frac) * dt * f_gpta) as u64;
                self.edges.push((tick.clamp(t0 + 1, to), level));
            }
        }
        self.i_dc = d * self.i_a;
        self.t = to;
    }

    /// FADC FAIN0P voltage of the current-sense amplifier.
    pub fn sense_v(&self) -> f32 {
        let i = if self.sense_offset_mode() { 0.0 } else { self.i_dc };
        (self.hw.sense_zero_v + self.hw.sense_v_per_a * i).clamp(0.0, crate::tc1784_io::fadc::VFAREF) as f32
    }

    /// Re-anchor the time base (SoC reset) without touching the mechanics.
    pub fn reset_time(&mut self, t: u64) {
        self.t = t;
        self.edges.clear();
    }

    /// Chip power-on reset (the MCU's SoC reset also resets the driver through its enable/supply
    /// pins): registers, SPI shift register and bridge state cleared; the rotor keeps turning.
    pub fn power_on_reset(&mut self) {
        self.regs = [0; 8];
        self.reply = 0;
        self.enable = false;
        self.comm = false;
        self.state = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const F: f64 = 40.0e6;

    /// Out-of-frame echo: the firmware's checks of 0x800D93BE pass and the status answer comes back on
    /// the next frame.
    #[test]
    fn spi_answers_previous_frame_and_echoes_register_words() {
        let mut p = BldcPump::default();
        let seq = [0x001F_u32, 0x0506, 0x1032, 0x0001];
        let mut rx = [0u32; 4];
        for _ in 0..2 {
            for (k, &w) in seq.iter().enumerate() {
                rx[k] = p.xfer16(w);
            }
        }
        assert_eq!(rx[1] >> 2, 0x001F >> 2, "reg3 echo bits 15:2");
        assert_eq!(rx[2] >> 4, 0x0506 >> 4, "reg6 echo bits 15:4");
        assert_eq!(rx[3] >> 3, 0x1032 >> 3, "reg2 echo bits 15:3");
        assert_eq!(rx[0], 0, "status: no fault");
        p.xfer16(0x401F);
        assert!(p.sense_offset_mode(), "reg3 bit 14 = offset phase");
    }

    /// Self-commutated run (physics test: the test toggles the commutation input half a zero-crossing
    /// interval after each comparator edge, the rule of the firmware's capture ISR 0x800DB7E0) at full
    /// duty. Checks the motor sizing against the firmware's own expectations: one comparator edge per
    /// commutation step at mid-step, speed within 500 rpm of its 2440..2620 rpm set-point at 100 % duty
    /// under 60 bar (else 0x800EFD38 shuts the pump down), DC-link current below its 30 A limit
    /// (CAL 0x80031E98 + 0x74, `0xD000B2F8` in 10 mA).
    #[test]
    fn self_commutated_motor_meets_the_firmware_speed_and_current_limits() {
        for (bar, min_rpm) in [(60.0, 2000.0), (0.0, 2600.0)] {
            let mut p = BldcPump {
                load: PumpLoad { torque_per_pa: 0.6e-6 / (2.0 * PI * 0.85), p_out_pa: bar * 1e5, visc_nm_s: 2e-6 },
                omega: 1000.0 * PI / 30.0,
                theta_e: 30f64.to_radians(),
                ..Default::default()
            };
            let (mut t, mut comm) = (0u64, false);
            p.set_pins(true, comm);
            let (mut last_zc, mut next_comm): (Option<u64>, Option<u64>) = (None, None);
            let (mut steps, mut zcs, mut phase_sum) = (0u64, 0u64, 0.0);
            let mut step_t0 = 0u64;
            while t < (1.0 * F) as u64 {
                let to = next_comm.map_or(t + 400, |c| c.min(t + 400));
                p.advance(to, F, 13.8, 1.0);
                t = to;
                for &(te, _) in &p.edges {
                    if t > (0.5 * F) as u64 && steps > 0 {
                        zcs += 1;
                        phase_sum += (te - step_t0) as f64;
                    }
                    if let Some(z) = last_zc {
                        next_comm = Some(te + (te - z) / 2);
                    }
                    last_zc = Some(te);
                }
                if next_comm.is_some_and(|c| c <= t) {
                    comm = !comm;
                    p.set_pins(true, comm);
                    if t > (0.5 * F) as u64 {
                        steps += 1;
                    }
                    step_t0 = t;
                    next_comm = None;
                }
            }
            let rpm = p.omega * 30.0 / PI;
            let step_s = 60.0 / (rpm * 24.0);
            println!("{bar} bar: {rpm:.0} rpm, i {:.1} A, dc {:.1} A, steps {steps} zc {zcs}, mean zc phase {:.2}", p.i_a, p.i_dc, phase_sum / zcs.max(1) as f64 / F / step_s);
            assert!(rpm > min_rpm, "{bar} bar: {rpm:.0} rpm at 100 % duty");
            assert!(p.i_dc < 30.0, "{bar} bar: {:.1} A DC link", p.i_dc);
            assert!(zcs.abs_diff(steps) <= 2, "one zero crossing per step: {zcs} vs {steps}");
            let ph = phase_sum / zcs.max(1) as f64 / F / step_s;
            assert!((0.35..0.65).contains(&ph), "zero crossing at {ph:.2} of the step");
        }
    }

    /// Current sense as the firmware decodes it (0x800D9CC8): FADC code difference to the offset
    /// phase x 32.258 = DC-link current in mA.
    #[test]
    fn current_sense_matches_the_firmware_scaling() {
        let mut p = BldcPump { i_dc: 12.0, ..Default::default() };
        let code = |v: f32| (v as f64 * 1024.0 / crate::tc1784_io::fadc::VFAREF).round();
        let on = code(p.sense_v());
        p.regs[3] = 0x4000;
        let off = code(p.sense_v());
        let ma = (on - off) * 10.0 * (3300.0 * 1000.0 / 500.0) / (2.0 * 1023.0);
        assert!((ma - 12000.0).abs() < 40.0, "{ma} mA");
    }

    /// Cost of the motor model (diagnostic): ns per board update while running.
    #[test]
    #[ignore = "diagnostic"]
    fn bldc_cost() {
        let mut p = BldcPump { omega: 250.0, enable: true, ..Default::default() };
        p.load.p_out_pa = 50e5;
        let n = 1_000_000u64;
        let t0 = std::time::Instant::now();
        for k in 1..=n {
            p.advance(k * 667, F, 13.8, 0.9); // one call per 2000-instruction quantum (16.7 us)
        }
        let ns = t0.elapsed().as_nanos() as f64 / n as f64;
        println!("BldcPump::advance {ns:.1} ns/call -> {:.3} % of real time at 60k calls/s ({:.0} rpm)", ns * 60e3 / 1e9 * 100.0, p.omega * 30.0 / PI);
    }
}
