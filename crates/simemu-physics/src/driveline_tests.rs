//! Gearbox / driveline / vehicle validation on a virtual lab rig.
//!
//! Everything named `Rig`/`Driver` here is TEST EQUIPMENT: a lab technician's current-controlled
//! solenoid drivers, a scripted shift sequencer and a torque-source dyno motor in place of the
//! engine (EA888 1.8 TSI full-load curve). It never runs in the emulator, where only the real TCU
//! firmware drives the mechatronic.

use super::*;
use crate::mechatronic::{dq200 as ch, wet as wch, MechatronicHw};

const H: f64 = DT_SUB;
/// "Fully closed" for the lab's DQ200 clutch commands, Nm: the capacity the rig's 58 bar gave the previous plant
/// (K2 ≈ 355, K1 ≈ 380 Nm), so the scripts keep their torque levels.
const DRY_FULL_NM: f64 = 360.0;
const PI: f64 = std::f64::consts::PI;

/// EA888 Gen3 1.8 TSI (CJSA) full-load curve: 250 Nm 1250–5000 rpm, 132 kW 5100–6200 rpm.
fn cjsa_full_load(rpm: f64) -> f64 {
    if rpm < 1250.0 {
        (140.0 + 110.0 * (rpm - 800.0).max(0.0) / 450.0).min(250.0)
    } else if rpm <= 5000.0 {
        250.0
    } else {
        132_000.0 / (rpm.min(6500.0) * PI / 30.0)
    }
}

struct Rig {
    d: DrivetrainProfile,
    dl: DrivelineState,
    env: DriveEnv,
    out: TcuOutputs,
    /// Engine + DMF + clutch drum speed, rad/s.
    we: f64,
    je: f64,
    pedal: f64,
    t: f64,
    /// Target coil currents (A) per channel; None = driver off.
    i_target: [Option<f64>; 12],
    i_int: [f64; 12],
    pump_on: bool,
    /// Lab operator cycles V401 between 45 and 60 bar like the TCU does.
    auto_pump: bool,
    /// Dyno motor torque instead of the engine curve, Nm.
    torque_override: Option<f64>,
    /// Speed-controlled dyno motor instead of the engine, rpm.
    hold_rpm: Option<f64>,
    /// Travel set point of the lab's dry-clutch travel loop (ramped, `Rig::clutch_mm`), mm; None = not under travel control.
    x_cmd: [Option<f64>; 2],
}

impl Rig {
    fn new(d: DrivetrainProfile) -> Self {
        let dl = DrivelineState::new(&d, 25.0);
        let je = 0.145 + DrivelineState::drive_inertia(&d);
        Self {
            d,
            dl,
            env: DriveEnv { ambient_c: 25.0, coolant_c: 90.0, ..Default::default() },
            out: TcuOutputs::default(),
            we: 0.0,
            je,
            pedal: 0.0,
            t: 0.0,
            i_target: [None; 12],
            i_int: [0.0; 12],
            pump_on: false,
            auto_pump: false,
            torque_override: None,
            hold_rpm: None,
            x_cmd: [None; 2],
        }
    }
    fn warm(mut self) -> Self {
        self.dl.case_c = 60.0;
        self.dl.mech.oil_c = 60.0;
        self
    }
    fn dq200() -> Self {
        let mut r = Self::new(DrivetrainProfile::dq200_fwd()).warm();
        r.dl.mech.precharge(&r.d.mechatronic, 55e5);
        r.auto_pump = true;
        r.we = 800.0 * PI / 30.0;
        r
    }
    fn wet(d: DrivetrainProfile) -> Self {
        let mut r = Self::new(d).warm();
        r.we = 800.0 * PI / 30.0;
        r
    }
    fn is_dry(&self) -> bool {
        matches!(self.d.mechatronic, MechatronicHw::Dq200(_))
    }
    fn rpm(&self) -> f64 {
        self.we * 30.0 / PI
    }
    fn kmh(&self) -> f64 {
        self.dl.vehicle_speed_ms * 3.6
    }
    fn engaged(&self, g: u8) -> bool {
        self.dl.gear_engaged[driveline::gear_index(g).unwrap()]
    }
    fn engine_torque(&self) -> f64 {
        if let Some(t) = self.torque_override {
            return t;
        }
        let rpm = self.rpm();
        // Full-load curve is brake torque; closed throttle motors with friction + pumping.
        let drag = if rpm > 1.0 { 12.0 + 0.004 * rpm } else { 0.0 };
        let p = if rpm > 6600.0 { 0.0 } else { self.pedal.clamp(0.0, 1.0) };
        p * cjsa_full_load(rpm) - (1.0 - p) * drag
    }
    /// Current loop of the lab driver: feed-forward on nominal coil resistance + integral. The DQ200 clutch valves meter
    /// flow around a neutral current (`mechatronic::Dq200HydraulicsHw::clutch_valve_neutral_a`): their driver feeds
    /// forward on the coil's present resistance, as a current-regulated output does (the TCU's 3 kHz regulators).
    fn drive_solenoids(&mut self) {
        let shw = match &self.d.mechatronic {
            MechatronicHw::Dq200(h) => h.solenoid,
            MechatronicHw::Wet(h) => h.solenoid,
        };
        for c in 0..12 {
            let exact = self.is_dry() && (c == ch::N435_K1 || c == ch::N439_K2);
            let r_ff = if exact { self.dl.mech.solenoids[c].resistance_ohm(&shw) } else { 5.0 };
            self.out.solenoid_duty[c] = self.i_target[c].map(|i| {
                let err = i - self.dl.mech.solenoids[c].current_a;
                self.i_int[c] = (self.i_int[c] + 3.0 * err * H).clamp(-0.5, 0.5);
                (i * r_ff / self.env.battery_v + self.i_int[c]).clamp(0.0, 1.0)
            });
        }
        if self.auto_pump {
            let p = self.dl.mech.supply_pa;
            if p < 45e5 {
                self.pump_on = true;
            } else if p > 60e5 {
                self.pump_on = false;
            }
            self.out.pump_motor_duty = Some(if self.pump_on { 1.0 } else { 0.0 });
        }
    }
    fn step(&mut self) {
        self.drive_solenoids();
        let te = self.engine_torque();
        let tl = self.dl.clutch_torques(&self.d, &self.out, &self.env, self.we, te, self.je, H);
        self.we = (self.we + (te - tl) / self.je * H).max(0.0);
        if let Some(rpm) = self.hold_rpm {
            self.we = rpm * PI / 30.0;
        }
        self.dl.integrate(&self.d, &self.env, self.we, H);
        self.t += H;
    }
    fn run(&mut self, s: f64, mut each: impl FnMut(&mut Rig)) {
        for _ in 0..(s / H).round() as usize {
            each(self);
            self.step();
            assert!(self.we.is_finite() && self.dl.vehicle_speed_ms.is_finite() && self.dl.carrier_omega.is_finite() && self.dl.mech.supply_pa.is_finite(), "NaN at t={}", self.t);
        }
    }

    // ---- valve characteristics known to the lab (calibration sheets) ----
    fn supply_on(&mut self, line_bar: f64) {
        if self.is_dry() {
            // N436/N440 rising characteristic (the firmware's p -> I table, `Dq200HydraulicsHw::
            // sub_valve_curve`): 1.3 A passes the full system pressure into the sub-gearbox.
            self.i_target[ch::N436_SUB1] = Some(1.3);
            self.i_target[ch::N440_SUB2] = Some(1.3);
        } else {
            self.i_target[wch::N233_SUB1] = Some(1.1);
            self.i_target[wch::N371_SUB2] = Some(1.1);
            let MechatronicHw::Wet(h) = &self.d.mechatronic else { unreachable!() };
            self.i_target[wch::N217_MAIN] = Some(crate::mechatronic::wet_main_valve_current(h, line_bar * 1e5));
        }
    }
    fn max_clutch_bar(&self) -> f64 {
        if self.is_dry() {
            58.0
        } else {
            12.0
        }
    }
    /// Wet units: clutch pressure through the proportional pressure valve. DQ200: N435/N439 meter flow
    /// (`mechatronic::Dq200HydraulicsHw::clutch_valve_neutral_a`), so the lab can only fill (`bar` ≥ the rig maximum:
    /// valve fully open, the actuator sees the sub-gearbox pressure) or vent (`bar` ≤ 0); in between it places the
    /// actuator by travel ([`Self::clutch_mm`]).
    fn clutch_bar(&mut self, k: usize, bar: f64) {
        self.x_cmd[k] = None;
        let (chan, i) = match &self.d.mechatronic {
            MechatronicHw::Dq200(h) => {
                assert!(bar <= 0.0 || bar >= self.max_clutch_bar(), "DQ200 clutch valves meter flow: place the clutch with clutch_mm, not {bar} bar");
                ([ch::N435_K1, ch::N439_K2][k], h.clutch_valve_neutral_a + 0.47)
            }
            MechatronicHw::Wet(h) => ([wch::N215_K1, wch::N216_K2][k], crate::mechatronic::wet_clutch_valve_current(h, bar * 1e5)),
        };
        self.i_target[chan] = Some(if bar <= 0.0 { 0.0 } else { i });
    }
    /// Dry clutch actuator under travel control (mm on its travel sensor), as the DQ200 clutches are driven: P loop on the
    /// flow valve around its neutral current, ≈ 0.1 A per mm like the TCU's (docs/DQ200_IO_MAP.md §14), on a set point the
    /// lab ramps at 200 mm/s (a travel step would open the valve fully and slam the clutch shut); ≤ 0 mm = vented.
    fn clutch_mm(&mut self, k: usize, mm: f64) {
        let MechatronicHw::Dq200(h) = &self.d.mechatronic else { unreachable!("travel control is for the dry clutch") };
        let x = self.dl.mech.clutch_x_m[k] * 1e3;
        let cmd = self.x_cmd[k].unwrap_or(x);
        let cmd = cmd + (mm - cmd).clamp(-200.0 * H, 200.0 * H);
        self.x_cmd[k] = Some(cmd);
        let i = if mm <= 0.0 { 0.0 } else { (h.clutch_valve_neutral_a + 0.1 * (cmd - x)).clamp(0.0, h.clutch_valve_neutral_a + 0.47) };
        self.i_target[[ch::N435_K1, ch::N439_K2][k]] = Some(i);
    }
    /// Dry clutch `k` placed to hold `nm` (new lining) under travel control; 0 Nm = at the kiss point.
    fn clutch_nm(&mut self, k: usize, nm: f64) {
        self.clutch_mm(k, self.dry_travel_mm(k, nm.max(0.0)));
    }
    /// Travel at which dry clutch `k` holds `nm` (new lining): its kiss point plus the inverse of
    /// `clutch::DryClutchHw::torque_at` — T/a0 = u + √((u − a4)² + a2) − √(a4² + a2) solved for u.
    fn dry_travel_mm(&self, k: usize, nm: f64) -> f64 {
        let crate::clutch::ClutchHw::Dry(h) = &self.d.clutch else { unreachable!() };
        let [a0, a2, a4] = h.torque_curve[k];
        let c = nm / a0 + (a4 * a4 + a2).sqrt();
        self.dl.clutches.kiss_point_m(&self.d.clutch, k) * 1e3 + (c * c - a4 * a4 - a2) / (2.0 * (c - a4))
    }
    /// Moves sleeves towards targets (mm). Wet DQ250: one multiplexer bank at a time.
    fn forks_to(&mut self, target: [f64; 4]) {
        match &self.d.mechatronic {
            MechatronicHw::Dq200(_) => {
                for (&t_mm, (&y_mm, &v_ch)) in target.iter().zip(self.dl.fork_mm.iter().zip(&ch::FORK_VALVE)) {
                    let e = t_mm - y_mm;
                    self.i_target[v_ch] = Some((0.5 + 0.12 * e).clamp(0.0, 1.0));
                }
            }
            MechatronicHw::Wet(h) if h.proportional_gear_valves => {
                for ((dst, &t_mm), &y_mm) in self.i_target[..4].iter_mut().zip(&target).zip(&self.dl.fork_mm) {
                    let e = t_mm - y_mm;
                    *dst = Some((0.5 + 0.12 * e).clamp(0.0, 1.0));
                }
            }
            MechatronicHw::Wet(h) => {
                let (mux_on, plus_2e) = (h.mux_energised, h.plus_valves_0x2e);
                self.i_target[..4].fill(Some(0.0));
                if let Some(f) = (0..4).find(|&f| (target[f] - self.dl.fork_mm[f]).abs() > 0.4) {
                    self.i_target[wch::N92_MUX] = Some(if mux_on[f] { 1.0 } else { 0.0 });
                    let e = target[f] - self.dl.fork_mm[f];
                    let v = if (e > 0.0) == plus_2e[f] { wch::GEAR_VALVE_PLUS[f] } else { wch::GEAR_VALVE_MINUS[f] };
                    self.i_target[v] = Some(1.0);
                }
            }
        }
    }
}

#[derive(Debug, Default)]
struct ShiftLog {
    from: u8,
    duration_s: f64,
    handover_s: f64,
    min_accel: f64,
    old_clutch_at_handover: f64,
    new_clutch_at_handover: f64,
}

/// Scripted launch + upshift sequencer (test equipment), with a crude traction control on the
/// torque source like an ESP torque intervention.
struct Driver {
    gear: u8,
    max_gear: u8,
    phase: u8,
    t0: f64,
    launch_rpm: f64,
    shift_rpm: f64,
    p_launch: f64,
    log: Vec<ShiftLog>,
    cur: ShiftLog,
    pedal_cmd: f64,
}

impl Driver {
    fn new(max_gear: u8) -> Self {
        Self { gear: 1, max_gear, phase: 0, t0: 0.0, launch_rpm: 2800.0, shift_rpm: 6250.0, p_launch: 0.0, log: vec![], cur: ShiftLog::default(), pedal_cmd: 0.0 }
    }

    fn act(&mut self, r: &mut Rig) {
        r.supply_on(16.0);
        let (g, n) = (self.gear, self.gear + 1);
        let mut tgt = [0.0; 4];
        let full = r.d.fork_full_mm;
        if let Some((f, s, _)) = r.d.gear_fork(g) {
            tgt[f] = s * full;
        }
        if n <= self.max_gear {
            if let Some((f, s, _)) = r.d.gear_fork(n) {
                tgt[f] = s * full;
            }
        }
        r.forks_to(tgt);
        let kc = r.d.gear_fork(g).unwrap().2;
        let kn = r.d.gear_fork(n).map(|x| x.2).unwrap_or(1 - kc);
        let pmax = r.max_clutch_bar();
        // Traction control on the torque source.
        let v = r.dl.vehicle_speed_ms;
        let slip = (r.dl.wheel_omega[0] * r.d.tyre.radius_m - v) / v.max(2.0);
        self.pedal_cmd = (self.pedal_cmd + (0.10 - slip) * 10.0 * H).clamp(0.3, 1.0);
        match self.phase {
            // Waiting for 1st gear at standstill, idle.
            0 => {
                r.pedal = (0.15 + 0.002 * (800.0 - r.rpm())).clamp(0.0, 0.5);
                r.clutch_bar(kc, 0.0);
                r.clutch_bar(kn, 0.0);
                if r.engaged(g) && r.t > 0.5 {
                    self.phase = 1;
                    self.t0 = r.t;
                    // dry: clutch torque (Nm), the earlier 8 bar ≈ 26 Nm; wet: piston pressure (bar)
                    self.p_launch = if r.is_dry() { 26.0 } else { 1.2 };
                    self.pedal_cmd = 1.0;
                }
            }
            // Launch: slip-controlled, engine held near launch_rpm.
            // Clutch capacity modulated on wheel slip (launch control), throttle holds the engine.
            1 => {
                r.pedal = (1.0 - 0.004 * (r.rpm() - self.launch_rpm)).clamp(0.0, 1.0);
                let err = 0.10 - slip;
                if r.is_dry() {
                    // Clutch torque placed through the characteristic, as the TCU does (torque → travel, `Rig::dry_travel_mm`);
                    // the earlier pressure gains 400 bar/s and 30 bar at ≈ 8.6 Nm/bar of the previous plant.
                    self.p_launch = (self.p_launch + 3440.0 * err * H).clamp(0.0, 600.0);
                    r.clutch_nm(kc, self.p_launch + 258.0 * err);
                } else {
                    self.p_launch = (self.p_launch + 60.0 * err * H).clamp(0.0, pmax);
                    r.clutch_bar(kc, (self.p_launch + 5.0 * err).clamp(0.0, pmax));
                }
                r.clutch_bar(kn, 0.0);
                if r.dl.clutch_slip_rad_s[kc].abs() < 3.0 && r.t - self.t0 > 0.3 {
                    self.phase = 2;
                    self.pedal_cmd = r.pedal;
                }
            }
            2 => {
                r.pedal = self.pedal_cmd;
                if r.is_dry() {
                    r.clutch_nm(kc, DRY_FULL_NM)
                } else {
                    r.clutch_bar(kc, pmax)
                }
                // Pre-fill the oncoming clutch to its kiss point shortly before the shift.
                let fill = r.rpm() > self.shift_rpm - 500.0 && r.engaged(n);
                if !fill {
                    r.clutch_bar(kn, 0.0);
                } else if r.is_dry() {
                    r.clutch_nm(kn, 0.0);
                } else {
                    r.clutch_bar(kn, 1.2);
                }
                if r.rpm() > self.shift_rpm && n <= self.max_gear && r.engaged(n) {
                    self.phase = 3;
                    self.t0 = r.t;
                    self.cur = ShiftLog { from: g, min_accel: f64::MAX, ..Default::default() };
                }
            }
            // Torque phase: clutch-to-clutch handover.
            3 => {
                r.pedal = self.pedal_cmd;
                let x = ((r.t - self.t0) / 0.12).min(1.0);
                if r.is_dry() {
                    r.clutch_nm(kn, DRY_FULL_NM * x);
                    r.clutch_nm(kc, DRY_FULL_NM * (1.0 - x) * 0.6);
                } else {
                    r.clutch_bar(kn, pmax * x);
                    r.clutch_bar(kc, pmax * (1.0 - x) * 0.6);
                }
                self.cur.min_accel = self.cur.min_accel.min(r.dl.accel_ms2);
                if x >= 1.0 {
                    self.cur.handover_s = r.t - self.t0;
                    self.cur.old_clutch_at_handover = r.dl.clutch_torque_nm[kc];
                    self.cur.new_clutch_at_handover = r.dl.clutch_torque_nm[kn];
                    self.phase = 4;
                }
            }
            // Inertia phase: engine pulled down to the new gear with torque intervention.
            _ => {
                r.pedal = 0.3 * self.pedal_cmd;
                if r.is_dry() {
                    r.clutch_nm(kn, DRY_FULL_NM)
                } else {
                    r.clutch_bar(kn, pmax)
                }
                r.clutch_bar(kc, 0.0);
                self.cur.min_accel = self.cur.min_accel.min(r.dl.accel_ms2);
                if r.dl.clutch_slip_rad_s[kn].abs() < 3.0 {
                    self.cur.duration_s = r.t - self.t0;
                    self.log.push(std::mem::take(&mut self.cur));
                    self.gear = n;
                    self.phase = 2;
                }
            }
        }
    }
}

// ============================================================ DQ200 =================================

#[test]
fn standstill_without_tcu_is_inert_and_finite() {
    let mut r = Rig::new(DrivetrainProfile::dq200_fwd());
    r.run(5.0, |_| {});
    assert_eq!(r.dl.vehicle_speed_ms, 0.0);
    assert_eq!(r.dl.mech.supply_pa, 0.0);
    assert!(r.dl.gear_engaged.iter().all(|&e| !e));
    let mut p = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient { temp_c: 25.0, altitude_m: 0.0, humidity_pct: 40.0 });
    for _ in 0..100 {
        p.step(0.01, &PlantActuators::default());
    }
    let t = p.telemetry();
    assert_eq!(t.vehicle_kmh, 0.0);
    assert!(!t.tcu_outputs_connected);
    assert!(t.driveline_failures.is_empty(), "{:?}", t.driveline_failures);
}

#[test]
fn parked_on_a_grade_holds_in_p_and_rolls_back_in_n() {
    let mut r = Rig::new(DrivetrainProfile::dq200_fwd());
    r.dl.road_grade_pct = 15.0;
    r.env.selector = Selector::P;
    r.run(3.0, |_| {});
    assert!(r.dl.vehicle_speed_ms.abs() < 0.05, "P must hold: {}", r.dl.vehicle_speed_ms);
    assert!(r.dl.pawl_engaged && !r.dl.pawl_damage.broken);
    r.env.selector = Selector::N;
    r.run(2.0, |_| {});
    assert!(r.dl.vehicle_speed_ms < -1.0, "N on 15 %: must roll back, v = {}", r.dl.vehicle_speed_ms);
}

#[test]
fn dq200_pump_charges_accumulator_and_relief_caps() {
    let mut r = Rig::new(DrivetrainProfile::dq200_fwd()).warm();
    let mut t60 = None;
    let mut i_at_60 = 0.0;
    r.run(25.0, |r| {
        r.out.pump_motor_duty = Some(1.0);
        if t60.is_none() && r.dl.mech.supply_pa >= 60e5 {
            t60 = Some(r.t);
            i_at_60 = r.dl.mech.pump.current_a;
        }
    });
    let t60 = t60.expect("never reached 60 bar");
    println!("V401: 0→60 bar in {t60:.2} s, {i_at_60:.1} A at 60 bar, relief-limited {:.1} bar", r.dl.mech.supply_pa / 1e5);
    assert!((1.5..10.0).contains(&t60), "charge time {t60}");
    assert!((10.0..30.0).contains(&i_at_60), "pump current {i_at_60}");
    let p = r.dl.mech.supply_pa / 1e5;
    assert!((71.0..76.0).contains(&p), "relief valve must cap the pressure: {p}");
}

#[test]
fn dq200_leakage_is_higher_with_hot_fluid() {
    let drop = |oil_c: f64| {
        let mut r = Rig::new(DrivetrainProfile::dq200_fwd());
        r.dl.case_c = oil_c;
        r.dl.mech.oil_c = oil_c;
        r.dl.mech.precharge(&r.d.mechatronic, 60e5);
        r.run(60.0, |r| r.dl.mech.oil_c = oil_c);
        60.0 - r.dl.mech.supply_pa / 1e5
    };
    let (hot, cold) = (drop(100.0), drop(0.0));
    println!("accumulator leak-down in 60 s: {hot:.1} bar at 100 °C, {cold:.1} bar at 0 °C");
    assert!(hot > 2.0 * cold && hot > 3.0, "hot {hot} cold {cold}");
}

#[test]
fn dq200_clutch_kiss_point_and_capacity_curve() {
    // K1 under travel control (the lab's sensor loop, `Rig::clutch_mm`): capacity against the travel the actuator reached.
    let mut r = Rig::dq200();
    r.hold_rpm = Some(1000.0);
    let mut curve = vec![];
    for step in 0..=24 {
        let mm = step as f64;
        r.run(0.25, |r| {
            r.supply_on(0.0);
            r.clutch_mm(0, mm);
        });
        curve.push((mm, r.dl.mech.clutch_x_m[0] * 1e3, r.dl.clutch_capacity_nm[0], r.dl.mech.clutch_pa[0] / 1e5));
    }
    for w in curve.windows(2) {
        assert!(w[1].1 >= w[0].1 - 0.02 && w[1].2 >= w[0].2 - 2.0, "non-monotonic {:?}", w);
    }
    let kiss = r.dl.clutches.kiss_point_m(&r.d.clutch, 0) * 1e3;
    let first = curve.iter().find(|c| c.2 > 5.0).unwrap();
    println!("K1: kiss {kiss:.2} mm, torque onset at {:.2} mm ({:.1} bar); {:?}", first.1, first.3, curve.iter().map(|c| format!("{:.1} mm {:.0} Nm {:.1} bar", c.1, c.2, c.3)).collect::<Vec<_>>());
    assert!((first.1 - kiss).abs() < 2.2, "torque onset {} vs kiss {kiss}", first.1);
    assert!(curve[0].2 == 0.0 && curve[0].1 < 1.0);
    // The learned characteristic (EEPROM 0x880..0x8BF, `clutch::DryClutchHw::dq200`) at its support travels.
    for (mm, nm) in [(13.33, 279.2), (20.52, 669.2)] {
        r.run(0.5, |r| {
            r.supply_on(0.0);
            r.clutch_mm(0, mm);
        });
        let (x, cap) = (r.dl.mech.clutch_x_m[0] * 1e3, r.dl.clutch_capacity_nm[0]);
        println!("K1 at {x:.2} mm (target {mm}): {cap:.0} Nm, learned {nm} Nm");
        assert!((x - mm).abs() < 0.3 && (cap - nm).abs() < 0.07 * nm, "K1 at {x:.2} mm: {cap:.0} Nm, learned {nm}");
    }
    // Wear moves the kiss point: the firmware's adaptation has something to adapt.
    if let crate::clutch::ClutchPack::Dry(s) = &mut r.dl.clutches {
        s.wear_mm[0] = 0.3;
    }
    let kiss_worn = r.dl.clutches.kiss_point_m(&r.d.clutch, 0) * 1e3;
    assert!((kiss_worn - kiss - 1.8).abs() < 0.05, "kiss drift {kiss} → {kiss_worn}");
}

/// The basic-setting measurement of the DQ200_G2 (`0x80060DA4`, docs/DQ200_IO_MAP.md §14) repeated on the rig with K2
/// (rising curve): clutch valve held open, the sub-gearbox pressure ramped at 0.20 bar per 10 ms (`CAL 0x800204E0 + 0x24E /
/// 0x24A`) up and then down; apply = the actuator pressure where the travel comes within 0.30 mm of a point (+0x294),
/// release = where it falls 0.40 mm below it (+0x274). The pressures equal the learned apply / release curves
/// (EEPROM 0xB2C.., `clutch::DryClutchHw::actuator_bar`) within 1.5 bar on the side the ramp cannot overshoot and 2.5 bar
/// on the side the actuator lags.
#[test]
fn dq200_clutch_actuator_hysteresis_is_the_learned_basic_setting() {
    let mut r = Rig::dq200();
    r.hold_rpm = Some(1000.0);
    let crate::clutch::ClutchHw::Dry(h) = r.d.clutch.clone() else { unreachable!() };
    let MechatronicHw::Dq200(m) = r.d.mechatronic.clone() else { unreachable!() };
    // coil current that sets sub-gearbox pressure `bar` (the firmware's own p -> I table, `Dq200HydraulicsHw::sub_valve_curve`)
    let i_sub = |bar: f64| {
        let c = &m.sub_valve_curve;
        let i = c.iter().position(|p| bar * 1e5 <= p.1).unwrap_or(c.len() - 1).max(1);
        c[i - 1].0 + (c[i].0 - c[i - 1].0) * (bar * 1e5 - c[i - 1].1) / (c[i].1 - c[i - 1].1)
    };
    // above the open stop, up to the last point a 44-bar ramp reaches
    let points = &h.actuator_bar[1][2..8];
    let (mut p_apply, mut p_release) = (vec![f64::NAN; points.len()], vec![f64::NAN; points.len()]);
    let mut bar = 0.0;
    r.run(4.4, |r| {
        bar += 20.0 * H;
        r.i_target[ch::N440_SUB2] = Some(i_sub(bar));
        r.clutch_bar(1, 70.0); // K2 valve fully open: the actuator sees the sub-gearbox pressure
        let (p, x) = (r.dl.mech.clutch_pa[1] / 1e5, r.dl.mech.clutch_x_m[1] * 1e3);
        for (i, pt) in points.iter().enumerate() {
            if p_apply[i].is_nan() && (x - pt.0).abs() <= 0.30 {
                p_apply[i] = p;
            }
        }
    });
    r.run(4.4, |r| {
        bar = (bar - 20.0 * H).max(0.0);
        r.i_target[ch::N440_SUB2] = Some(i_sub(bar));
        r.clutch_bar(1, 70.0);
        let (p, x) = (r.dl.mech.clutch_pa[1] / 1e5, r.dl.mech.clutch_x_m[1] * 1e3);
        for (i, pt) in points.iter().enumerate() {
            if p_release[i].is_nan() && x <= pt.0 - 0.40 {
                p_release[i] = p;
            }
        }
    });
    for (i, &(mm, release, apply)) in points.iter().enumerate() {
        println!("K2 {mm} mm: apply {:.1} bar (learned {apply}), release {:.1} bar (learned {release})", p_apply[i], p_release[i]);
        assert!((apply - 1.5..=apply + 2.5).contains(&p_apply[i]), "K2 {mm} mm reached at {:.1} bar, learned apply {apply}", p_apply[i]);
        assert!((release - 2.5..=release + 1.5).contains(&p_release[i]), "K2 back below {mm} mm at {:.1} bar, learned release {release}", p_release[i]);
    }
}

#[test]
fn dq200_0_100_with_power_shifts_matches_octavia_1_8_tsi() {
    let mut r = Rig::dq200();
    r.env.selector = Selector::D;
    // Tyres warm from driving to the test site.
    r.dl.tyres[0].tread_c = 50.0;
    r.dl.tyres[1].tread_c = 50.0;
    let mut drv = Driver::new(4);
    let mut t_start = None;
    let mut t100 = None;
    let mut peak_slip_heat: f64 = 0.0;
    r.run(14.0, |r| {
        drv.act(r);
        if drv.phase >= 1 && t_start.is_none() {
            t_start = Some(r.t);
        }
        if t100.is_none() && r.kmh() >= 100.0 {
            t100 = Some(r.t - t_start.unwrap());
        }
        peak_slip_heat = peak_slip_heat.max(r.dl.clutches.temps().1[0]);
    });
    let t100 = t100.expect("never reached 100 km/h");
    println!("DQ200 0–100 km/h: {t100:.2} s; shifts {:#?}; K1 peak surface {peak_slip_heat:.0} °C", drv.log);
    // Skoda: Octavia III 1.8 TSI DSG 7.4 s (liftback) / 7.8 s.
    assert!((6.8..8.8).contains(&t100), "0–100 {t100}");
    let s12 = &drv.log[0];
    assert_eq!(s12.from, 1);
    assert!((0.15..0.6).contains(&s12.duration_s), "1→2 shift time {}", s12.duration_s);
    assert!(s12.old_clutch_at_handover.abs() < 0.35 * s12.new_clutch_at_handover.abs(), "handover {:?}", s12);
    assert!(s12.min_accel > -1.0, "torque hole {:?}", s12);
    assert!(r.dl.failures(&r.d).is_empty(), "{:?}", r.dl.failures(&r.d));
}

#[test]
fn dq200_launch_abuse_fades_and_burns_the_clutch() {
    let mut r = Rig::dq200();
    r.env.selector = Selector::D;
    r.env.brake_pct = 100.0;
    let mut cap_start = 0.0;
    let mut cap_hot = 0.0;
    let mut t_glaze = None;
    r.run(90.0, |r| {
        r.supply_on(0.0);
        let mut tgt = [0.0; 4];
        tgt[0] = r.d.fork_full_mm;
        r.forks_to(tgt);
        // Brake-torqued launch: driver floors it, engine held around 3500 rpm by the slipping clutch.
        r.pedal = (0.5 - 0.002 * (r.rpm() - 3500.0)).clamp(0.0, 1.0);
        // 10.8 mm ≈ 167 Nm on the new clutch: what the 26 bar of this test held on the previous plant (cushion law).
        if r.t > 1.0 {
            r.clutch_mm(0, 10.8);
        } else {
            r.clutch_bar(0, 0.0);
        }
        if (r.t - 2.0).abs() < H / 2.0 {
            cap_start = r.dl.clutch_capacity_nm[0];
        }
        if (r.t - 25.0).abs() < H / 2.0 {
            cap_hot = r.dl.clutch_capacity_nm[0];
        }
        if t_glaze.is_none() && r.dl.clutches.condition()[0].1 > 0.05 {
            t_glaze = Some(r.t);
        }
    });
    let (temps, surf) = r.dl.clutches.temps();
    let f = r.dl.failures(&r.d);
    println!("brake-stand: capacity {cap_start:.0} → {cap_hot:.0} Nm (fade), glazing from {t_glaze:?} s, bulk {:.0} °C surface {:.0} °C, failures {f:?}", temps[0], surf[0]);
    assert!(r.dl.vehicle_speed_ms.abs() < 0.2, "brakes must hold");
    assert!(cap_hot < 0.9 * cap_start, "fade {cap_start} → {cap_hot}");
    assert!(t_glaze.is_some());
    assert!(f.iter().any(|s| s.contains("K1")), "{f:?}");
}

#[test]
fn dq200_synchroniser_work_and_dog_clash_with_worn_ring() {
    // 60 km/h, all forks neutral, input shafts at rest: pre-select 3rd (normal) …
    let mut r = Rig::dq200();
    r.dl.set_rolling(&r.d, 60.0 / 3.6);
    r.env.selector = Selector::D;
    let mut t_eng = None;
    r.run(1.0, |r| {
        r.supply_on(0.0);
        r.forks_to([-r.d.fork_full_mm, 0.0, 0.0, 0.0]);
        if t_eng.is_none() && r.engaged(3) {
            t_eng = Some(r.t);
        }
    });
    let e3 = r.dl.synchro_energy_j[2];
    println!("3rd at 60 km/h: synchronised+engaged in {t_eng:?} s, cone energy {e3:.0} J, ring {:.0} °C", r.dl.synchro_c[2]);
    assert!(t_eng.is_some_and(|t| (0.05..0.6).contains(&t)), "{t_eng:?}");
    assert!((600.0..3000.0).contains(&e3), "cone energy {e3}");
    assert_eq!(r.dl.dog_health[2], 1.0);

    // … then 5th with a worn single-cone ring: the blocker no longer blocks → dogs grind.
    let mut r = Rig::dq200();
    r.dl.set_rolling(&r.d, 60.0 / 3.6);
    r.dl.synchro_energy_j[4] = r.d.train.synchro.life_j;
    r.run(1.5, |r| {
        r.supply_on(0.0);
        r.forks_to([0.0, 0.0, r.d.fork_full_mm, 0.0]);
    });
    let f = r.dl.failures(&r.d);
    println!("5th with worn ring: dog health {:.2}, engaged {}, failures {f:?}", r.dl.dog_health[4], r.engaged(5));
    assert!(r.dl.dog_health[4] < 0.9, "dog clash must damage the dogs");
    assert!(f.iter().any(|s| s.contains("gear 5 synchroniser")));
}

#[test]
fn dq200_clutch_dump_in_park_breaks_the_drivetrain() {
    let mut r = Rig::dq200();
    r.env.selector = Selector::P;
    let mut peak = 0.0f64;
    r.run(3.0, |r| {
        r.supply_on(0.0);
        let mut tgt = [0.0; 4];
        tgt[0] = r.d.fork_full_mm;
        r.forks_to(tgt);
        if r.t < 1.5 {
            r.pedal = (1.0 - 0.003 * (r.rpm() - 4500.0)).clamp(0.0, 1.0);
            r.clutch_bar(0, 0.0);
        } else {
            r.pedal = 1.0;
            r.clutch_bar(0, 58.0); // dumped
        }
        peak = peak.max(r.dl.pawl_torque_nm.abs());
    });
    let f = r.dl.failures(&r.d);
    println!("clutch dump in P at 4500 rpm: peak pawl torque {peak:.0} Nm, failures {f:?}");
    assert!(r.dl.pawl_damage.broken || r.dl.gear_damage[0].broken, "{f:?}");
}

#[test]
fn dq200_sustained_overload_fatigues_first_gear_much_faster() {
    let damage = |torque: f64| {
        let mut r = Rig::dq200();
        r.dl.force_engage(&r.d, 4);
        r.env.rollers = Some((ChassisDynoHw::default(), 60.0));
        r.dl.set_rolling(&r.d, 60.0 / 3.6);
        r.dl.rollers.speed_ms = 60.0 / 3.6;
        r.dl.vehicle_speed_ms = 0.0;
        r.we = r.dl.shaft_omega[1];
        r.torque_override = Some(torque);
        r.run(5.0, |r| {
            r.supply_on(0.0);
            r.forks_to([0.0, -r.d.fork_full_mm, 0.0, 0.0]);
            r.clutch_bar(1, 58.0);
        });
        (r.dl.gear_damage[3].fatigue, r.rpm())
    };
    let ((d250, n250), (d400, n400)) = (damage(250.0), damage(400.0));
    println!("4th gear root-bending damage in 5 s on the roller dyno: {d250:.2e} at 250 Nm ({n250:.0} rpm), {d400:.2e} at 400 Nm ({n400:.0} rpm)");
    // Stock torque is below the endurance limit of 4th; a 400 Nm tune is not. (With the clutch stiffness fitted to the
    // DQ200_G2's torque -> travel targets, `clutch::DryClutchHw::dq200`, K2 at the rig's ≈ 55 bar sub-gearbox pressure
    // holds ≈ 395 Nm and slips a little above: the absolute damage bound is 5e-5 instead of 1e-4.)
    assert!(d400 > 5e-5 && d400 > 30.0 * d250, "{d250} {d400}");
}

#[test]
fn dq200_tip_in_excites_driveline_shuffle() {
    let mut r = Rig::dq200();
    r.dl.force_engage(&r.d, 2);
    r.dl.set_rolling(&r.d, 30.0 / 3.6);
    r.we = r.dl.shaft_omega[1];
    r.torque_override = Some(0.0);
    let mut acc = vec![];
    r.run(2.0, |r| {
        r.supply_on(0.0);
        r.forks_to([0.0, r.d.fork_full_mm, 0.0, 0.0]);
        r.clutch_bar(1, 58.0);
        if r.t > 0.5 {
            r.torque_override = Some(150.0);
            acc.push(r.dl.accel_ms2);
        }
    });
    let mean = acc[acc.len() / 2..].iter().sum::<f64>() / (acc.len() / 2) as f64;
    let crossings = acc.windows(2).filter(|w| (w[0] - mean) * (w[1] - mean) < 0.0).count();
    let f = crossings as f64 / 2.0 / 1.5;
    let peak = acc.iter().cloned().fold(f64::MIN, f64::max);
    println!("tip-in 150 Nm in 2nd: shuffle ≈ {f:.1} Hz, peak {peak:.2} m/s² vs mean {mean:.2}");
    assert!((1.5..9.0).contains(&f), "shuffle frequency {f}");
    assert!(peak > 1.1 * mean, "no overshoot");
}

#[test]
fn coast_down_from_100_kmh_in_neutral() {
    let mut r = Rig::new(DrivetrainProfile::dq200_fwd()).warm();
    r.dl.set_rolling(&r.d, 100.0 / 3.6);
    r.env.selector = Selector::N;
    r.run(1.0, |_| {});
    let v1 = r.dl.vehicle_speed_ms;
    let mut t50 = 0.0;
    r.run(80.0, |r| {
        if t50 == 0.0 && r.kmh() <= 50.0 {
            t50 = r.t;
        }
    });
    // Road load at 100 km/h: aero 0.5·1.184·0.29·2.22·27.78² ≈ 294 N; rolling ≈ 1385·9.81·0.0123 ≈ 167 N
    // (+ carrier spin losses); rotating mass ≈ +47 kg → ≈ 0.33 m/s².
    let decel = 100.0 / 3.6 - v1;
    println!("coast-down: {decel:.3} m/s² at 100 km/h, 100→50 km/h in {t50:.1} s, end {:.1} km/h", r.kmh());
    assert!((0.28..0.40).contains(&decel), "decel {decel}");
    assert!((40.0..75.0).contains(&t50), "100→50 {t50}");
}

#[test]
fn moderate_braking_in_gear_does_not_lock_the_wheels() {
    let mut r = Rig::dq200();
    r.dl.force_engage(&r.d, 3);
    r.dl.set_rolling(&r.d, 50.0 / 3.6);
    r.we = r.dl.shaft_omega[0];
    r.pedal = 0.0;
    let v0 = r.dl.vehicle_speed_ms;
    r.env.brake_pct = 30.0;
    r.run(0.5, |r| {
        r.supply_on(0.0);
        r.forks_to([-r.d.fork_full_mm, 0.0, 0.0, 0.0]);
        r.clutch_bar(0, 58.0);
    });
    let decel = (v0 - r.dl.vehicle_speed_ms) / 0.5;
    let wheel_v = r.dl.wheel_omega[0] * r.d.tyre.radius_m;
    assert!(wheel_v > 0.8 * r.dl.vehicle_speed_ms, "wheels locked: {wheel_v} vs {}", r.dl.vehicle_speed_ms);
    assert!((2.5..5.5).contains(&decel), "decel {decel}");
}

#[test]
fn chassis_dyno_pull_in_third_measures_wheel_power() {
    let mut r = Rig::dq200();
    r.dl.force_engage(&r.d, 3);
    r.env.rollers = Some((ChassisDynoHw::default(), 60.0));
    r.dl.set_rolling(&r.d, 60.0 / 3.6);
    r.dl.rollers.speed_ms = 60.0 / 3.6;
    r.dl.vehicle_speed_ms = 0.0;
    r.we = r.dl.shaft_omega[0];
    r.pedal = 1.0;
    let mut p_wheel = 0.0;
    let mut p_eng = 0.0;
    r.run(4.0, |r| {
        r.supply_on(0.0);
        r.forks_to([-r.d.fork_full_mm, 0.0, 0.0, 0.0]);
        r.clutch_bar(0, 58.0);
        p_wheel = r.dl.tyres[0].force_n * r.dl.rollers.speed_ms;
        p_eng = r.engine_torque() * r.we;
    });
    let eta = p_wheel / p_eng;
    println!("roller dyno 3rd @ {:.0} km/h, {:.0} rpm: engine {:.1} kW, wheels {:.1} kW (η = {eta:.3}), gear oil {:.0} °C", r.dl.rollers.speed_ms * 3.6, r.rpm(), p_eng / 1e3, p_wheel / 1e3, r.dl.case_c);
    assert!((r.dl.rollers.speed_ms * 3.6 - 60.0).abs() < 3.0);
    assert!((0.85..0.985).contains(&eta), "η {eta}");
}

#[test]
fn realtime_cost_report() {
    let mut r = Rig::dq200();
    r.env.selector = Selector::D;
    let mut drv = Driver::new(4);
    let t0 = std::time::Instant::now();
    r.run(5.0, |r| drv.act(r));
    let per_s = t0.elapsed().as_secs_f64() / 5.0;
    println!("driveline cost: {:.1} ms per simulated second ({:.2} µs per 0.2 ms sub-step)", per_s * 1e3, per_s * 1e6 * H);
    if !cfg!(debug_assertions) {
        assert!(per_s < 0.25, "too slow for real time: {per_s}");
    }
}

// ============================================================ wet units ============================

#[test]
fn dq250_clutch_fill_kiss_and_capacity() {
    let mut r = Rig::wet(DrivetrainProfile::dq250_4wd());
    r.hold_rpm = Some(1500.0);
    let mut t_fill = None;
    r.run(1.2, |r| {
        r.supply_on(14.0);
        if r.t > 0.2 {
            r.clutch_bar(0, 5.0);
        }
        if t_fill.is_none() && r.dl.clutches.contact_force(0) > 100.0 {
            t_fill = Some(r.t - 0.2);
        }
    });
    let cap = r.dl.clutch_capacity_nm[0];
    println!("DQ250 K1: line {:.1} bar, fill {t_fill:?} s, {:.2} bar in piston, travel {:.2} mm, capacity {cap:.0} Nm at 5 bar", r.dl.mech.supply_pa / 1e5, r.dl.mech.clutch_pa[0] / 1e5, r.dl.mech.clutch_x_m[0] * 1e3);
    assert!((12.0..16.0).contains(&(r.dl.mech.supply_pa / 1e5)), "line pressure");
    assert!(t_fill.is_some_and(|t| (0.01..0.4).contains(&t)), "fill {t_fill:?}");
    // firmware characteristic (2.70 bar → 0, 39.7 Nm/bar): ≈ 91 Nm at 5 bar
    assert!((75.0..110.0).contains(&cap), "capacity {cap}");
}

/// Static capacity of K1/K2 against the characteristic the 0D9300012F firmware is calibrated for (CAL 0x8003806C:
/// zero-torque pressure +0x168, pressure at 250 Nm +0x180; centrifugal term of 0x800957A0 with k(70 °C), VERIFIED-S;
/// docs/DQ250_FIRMWARE_SPEC.md §3): below the zero-torque pressure the pack must not carry torque, above it the plant and
/// the firmware must agree within a few percent of the 250 Nm reference.
#[test]
fn dq250_clutch_matches_the_firmware_characteristic() {
    // (zero-torque bar, bar at 250 Nm, centrifugal Nm at 6000 rpm)
    let fw = [(2.700, 8.995, 1753.0 * 0.02), (2.500, 7.955, 704.0 * 0.02)];
    for (k, &(p0, p250, t_cf)) in fw.iter().enumerate() {
        for (rpm, bar) in [(800.0, p0 - 0.3), (800.0, p0 + 1.0), (800.0, 0.5 * (p0 + p250)), (800.0, p250), (6000.0, p0 - 0.5)] {
            let mut r = Rig::wet(DrivetrainProfile::dq250_4wd());
            r.dl.case_c = 70.0;
            r.dl.clutches = crate::clutch::ClutchPack::new(&r.d.clutch, 70.0); // packs at ATF temperature
            r.hold_rpm = Some(rpm);
            r.run(1.0, |r| {
                r.supply_on(14.0);
                r.dl.case_c = 70.0;
                r.clutch_bar(k, bar);
            });
            let cap = r.dl.clutch_capacity_nm[k];
            let want = ((bar - p0) * 250.0 / (p250 - p0) + t_cf * (rpm / 6000.0f64).powi(2)).max(0.0);
            println!("K{} {rpm:.0} rpm {bar:.2} bar: plant {cap:.1} Nm, firmware {want:.1} Nm (balance fill {:.2})", k + 1, r.dl.mech.balance_fill);
            assert!((cap - want).abs() < 0.06 * 250.0, "K{} at {bar:.2} bar / {rpm} rpm: plant {cap:.1} Nm vs firmware {want:.1} Nm", k + 1);
        }
    }
}

/// Open K1 on a held shaft. N218 de-energised = full clutch cooling flow (the rig default); `cooling_closed`: N218 at the
/// 1.08 A the TCU holds at cold idle (0D9300011R CAL 0x800311FC+0x242, VERIFIED-S/T) — the slide valve shuts the only oil
/// channel to the packs (VW SSP 308 p. 35/38). The TCU's own gearbox-uptake model (`GE_Aufnahmemoment`, CAL 0x800301A8
/// +0x74/+0x8C, VERIFIED-S) counts 1.5–4.5 Nm of open-clutch drag at −20..20 °C ATF; a flooded pack at cold drags an
/// order of magnitude more (SSP 308 p. 52: gear change difficulties at cold with full cooling flow).
#[test]
fn dq250_open_clutch_drag_vs_speed_and_temperature() {
    let drag = |rpm: f64, oil_c: f64, cooling_closed: bool| {
        let mut r = Rig::wet(DrivetrainProfile::dq250_4wd());
        r.dl.case_c = oil_c;
        r.dl.force_engage(&r.d, 1);
        r.env.brake_pct = 100.0;
        r.hold_rpm = Some(rpm);
        r.run(1.0, |r| {
            r.supply_on(14.0);
            if cooling_closed {
                r.i_target[wch::N218_COOL] = Some(1.08);
            }
            r.dl.case_c = oil_c;
        });
        r.dl.clutch_torque_nm[0]
    };
    let (d1k, d3k, d6k) = (drag(1000.0, 80.0, false), drag(3000.0, 80.0, false), drag(6000.0, 80.0, false));
    let (cold, cold_closed) = (drag(1000.0, 5.0, false), drag(1000.0, 5.0, true));
    println!("DQ250 K1 open drag @80 °C: {d1k:.2} / {d3k:.2} / {d6k:.2} Nm at 1000/3000/6000 rpm; 1000 rpm, 5 °C: {cold:.2} Nm flooded, {cold_closed:.2} Nm with the cooling oil shut");
    assert!(cold > 1.5 * d1k, "cold ATF must drag more");
    assert!(d6k < d1k.max(d3k), "aeration must reduce drag at high speed");
    assert!((0.2..25.0).contains(&d1k));
    assert!(cold_closed < 4.5 && cold > 4.0 * cold_closed, "cooling oil shut at 5 °C: {cold_closed:.2} Nm, flooded {cold:.2} Nm");
}

/// K1 slipping at 1000 rpm on a held shaft with the cooling oil on, just above the firmware's zero-torque pressure: cold ATF
/// may change the torque only by what the firmware corrects for — its cold map adds at most +0.25 bar (≈ 10 Nm) near zero
/// torque at 0 °C and +0.05..0.1 bar at 40 Nm (0D9300012F CAL 0x8003806C+0xAC, ATF axis −25..80 °C, VERIFIED-S).
#[test]
fn dq250_slipping_clutch_near_kiss_point_cold_vs_warm() {
    let torque = |oil_c: f64, bar: f64| {
        let mut r = Rig::wet(DrivetrainProfile::dq250_4wd());
        r.dl.case_c = oil_c;
        r.dl.clutches = crate::clutch::ClutchPack::new(&r.d.clutch, oil_c);
        r.dl.force_engage(&r.d, 1);
        r.env.brake_pct = 100.0;
        r.hold_rpm = Some(1000.0);
        r.run(3.0, |r| {
            r.supply_on(14.0);
            r.dl.case_c = oil_c;
            r.clutch_bar(0, bar);
        });
        r.dl.clutch_torque_nm[0]
    };
    let (t_0_cold, t_25_cold) = (torque(5.0, 0.0), torque(5.0, 2.5));
    assert!(t_25_cold <= t_0_cold, "pre-kiss 2.5 bar drag {t_25_cold:.1} Nm must not exceed 0 bar open drag {t_0_cold:.1} Nm");
    for bar in [3.0, 3.7] {
        let (cold, warm) = (torque(5.0, bar), torque(70.0, bar));
        println!("K1 at {bar} bar, 1000 rpm slip, cooling on: {cold:.1} Nm at 5 °C, {warm:.1} Nm at 70 °C (firmware {:.1} Nm)", (bar - 2.7) * 39.7);
        assert!((cold - warm).abs() < 15.0, "{bar} bar: cold {cold:.1} Nm vs warm {warm:.1} Nm");
    }
}

#[test]
fn dq250_balance_chamber_compensates_centrifugal_pressure() {
    // just below the firmware's zero-torque pressure (2.7 bar): only the centrifugal head can close the pack
    let contact = |bar: f64, drained: bool| {
        let mut r = Rig::wet(DrivetrainProfile::dq250_4wd());
        r.hold_rpm = Some(6000.0);
        r.run(0.6, |r| {
            r.supply_on(14.0);
            r.clutch_bar(0, bar);
            if drained {
                r.dl.mech.balance_fill = 0.0;
            }
        });
        r.dl.clutch_capacity_nm[0]
    };
    let (open, ok, drained) = (contact(0.0, true), contact(2.2, false), contact(2.2, true));
    println!("K1 at 6000 rpm: 0 bar drained {open:.1} Nm; 2.2 bar: {ok:.1} Nm with balance chamber, {drained:.1} Nm drained");
    assert!(open < 1.0, "return spring must hold the pack open at 0 bar: {open}");
    assert!(drained > ok + 10.0, "balance chamber does not compensate: {ok} vs drained {drained}");
}

#[test]
fn dq250_launch_and_power_shift_fwd() {
    let mut r = Rig::wet(DrivetrainProfile::dq250_4wd());
    r.env.selector = Selector::D;
    let mut drv = Driver::new(3);
    let mut t_start = None;
    let mut t60 = None;
    r.run(8.0, |r| {
        drv.act(r);
        if drv.phase >= 1 && t_start.is_none() {
            t_start = Some(r.t);
        }
        if t60.is_none() && r.kmh() >= 60.0 {
            t60 = Some(r.t - t_start.unwrap());
        }
    });
    println!("DQ250 (Haldex not driven → FWD) 0–60 km/h {t60:?} s; shifts {:#?}; rear torque {:.1} Nm", drv.log, r.dl.haldex.torque_nm);
    assert!(t60.is_some_and(|t| (3.0..7.0).contains(&t)), "{t60:?}");
    assert!(!drv.log.is_empty() && (0.1..0.7).contains(&drv.log[0].duration_s), "{:?}", drv.log);
    assert_eq!(r.dl.haldex.torque_nm, 0.0, "undriven Haldex must stay open");
}

#[test]
fn dq250_slip_abuse_overheats_atf_and_burns_pack() {
    let mut r = Rig::wet(DrivetrainProfile::dq250_4wd());
    r.env.brake_pct = 100.0;
    let mut p_k1 = 0.0f64;
    let mut rpm_min = f64::MAX;
    r.run(120.0, |r| {
        r.supply_on(14.0);
        r.i_target[wch::N218_COOL] = Some(1.0); // cooling valve closed (firmware gone wrong)
        r.forks_to([r.d.fork_full_mm, 0.0, 0.0, 0.0]);
        // Stall test: full throttle, clutch pressure trimmed to hold 3500 rpm.
        r.pedal = if r.t > 0.5 { 1.0 } else { 0.5 };
        if r.t > 1.0 {
            p_k1 = (p_k1 + 2e-3 * (r.rpm() - 3500.0) * H).clamp(0.0, 12.0);
            rpm_min = rpm_min.min(r.rpm());
        }
        r.clutch_bar(0, p_k1);
    });
    let (pack, surf) = r.dl.clutches.temps();
    let f = r.dl.failures(&r.d);
    println!("DQ250 slip abuse: pack {:.0} °C surface {:.0} °C, ATF {:.0} °C, ATF degradation {:.3}, failures {f:?}", pack[0], surf[0], r.dl.case_c, r.dl.mech.atf_degradation);
    assert!(rpm_min > 1500.0, "engine stalled");
    assert!(r.dl.case_c > 90.0);
    assert!(f.iter().any(|s| s.contains("K1")), "{f:?}");
}

#[test]
fn haldex_transfers_torque_only_when_its_controller_drives_the_pump() {
    let run = |duty: Option<f64>| {
        let mut r = Rig::wet(DrivetrainProfile::dq250_4wd());
        r.env.selector = Selector::D;
        r.env.awd_pump_duty = duty;
        let mut drv = Driver::new(1);
        r.run(2.5, |r| drv.act(r));
        (r.dl.haldex.torque_nm, r.kmh())
    };
    let (t_off, v_off) = run(None);
    let (t_on, v_on) = run(Some(1.0));
    println!("Haldex: off {t_off:.0} Nm / {v_off:.1} km/h, on {t_on:.0} Nm / {v_on:.1} km/h at 2.5 s");
    assert_eq!(t_off, 0.0);
    assert!(t_on.abs() > 20.0 && v_on > v_off, "AWD must add traction");
}

#[test]
fn dq381_launch_smoke() {
    let mut r = Rig::wet(DrivetrainProfile::dq381_4wd());
    r.env.selector = Selector::D;
    let mut drv = Driver::new(2);
    drv.shift_rpm = 5500.0;
    r.run(5.0, |r| drv.act(r));
    println!("DQ381 after 5 s: {:.1} km/h, gear {}, shifts {:?}", r.kmh(), drv.gear, drv.log);
    assert!(r.kmh() > 30.0 && drv.gear == 2);
}

#[test]
fn dq200_repeated_preselect_reversals_do_not_fracture_third_gear() {
    let mut r = Rig::dq200();
    r.dl.force_engage(&r.d, 4);
    r.dl.set_rolling(&r.d, 80.0 / 3.6);
    r.we = r.dl.shaft_omega[1];
    r.env.selector = Selector::D;
    // Repeatedly reverse Fork 0 (1/3) across the dog-tooth / engagement region while driving in 4th gear
    // on K2 and letting input shaft 1 spin down whenever disengaged.
    for cycle in 0..8 {
        let tgt = match cycle % 4 {
            0 => -r.d.fork_full_mm,
            1 => -5.5, // partial withdrawal inside dog splines (must stay meshed)
            2 => -r.d.fork_full_mm,
            _ => -3.5, // withdrawal past dog tips into sync zone, then re-engage
        };
        r.run(0.35, |r| {
            if cycle % 4 == 3 && !r.dl.gear_engaged[2] {
                r.dl.shaft_omega[0] *= 0.99;
            }
            r.supply_on(0.0);
            r.clutch_nm(1, 150.0);
            r.clutch_bar(0, 0.0);
            r.forks_to([tgt, -r.d.fork_full_mm, 0.0, 0.0]);
        });
    }
    r.run(0.7, |r| {
        r.supply_on(0.0);
        r.clutch_nm(1, 150.0);
        r.clutch_bar(0, 0.0);
        r.forks_to([-r.d.fork_full_mm, -r.d.fork_full_mm, 0.0, 0.0]);
    });
    assert!(!r.dl.gear_damage[2].broken, "3rd gear teeth must not fracture on pre-selection: {:?}", r.dl.failures(&r.d));
    assert_eq!(r.dl.engaged_on_shaft(&r.d, 0), Some(3));
    let expected_w0 = r.dl.carrier_omega * r.d.total_ratio(3);
    assert!((r.dl.shaft_omega[0] - expected_w0).abs() < 5.0, "shaft 1 ω={} vs expected {expected_w0}", r.dl.shaft_omega[0]);
}

#[test]
fn engine_dyno_uncouples_driveline_and_steps_mechatronic_actuators() {
    let mut p = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient::default());
    p.driveline.force_engage(&p.drivetrain_profile, 1);
    p.driveline.set_rolling(&p.drivetrain_profile, 6.54 / 3.6);
    p.driveline.mech.clutch_x_m[0] = 5.274e-3;
    p.driveline.clutches.set_travel(&p.drivetrain_profile.clutch, 0, 5.274e-3);
    p.bench = BenchMode::EngineDyno { target_rpm: 2000.0 };
    let mut act = PlantActuators::default();
    act.tcu.pump_motor_duty = Some(0.5);
    act.tcu.solenoid_duty[0] = Some(0.0);
    for _ in 0..20 {
        p.step(0.01, &act);
    }
    let t = p.telemetry();
    assert_eq!(t.vehicle_kmh, 0.0, "vehicle speed must not stay frozen at 6.54 km/h on engine dyno");
    assert_eq!(t.shaft_rpm, [0.0, 0.0]);
    assert!(t.clutch_travel_mm[0] < 0.5, "K1 must retract when TCU opens solenoid on engine dyno, got {} mm", t.clutch_travel_mm[0]);
}
