//! Plant validation on a virtual test bench. The bench operator below is TEST CODE that supplies
//! actuator commands (like a lab technician with a calibration tool and lab instruments); it is not
//! part of the product and never runs in the emulator.

use super::*;

const PI: f64 = std::f64::consts::PI;

#[derive(Clone)]
struct Operator {
    spark: f64,
    dwell_ms: f64,
    lambda: f64,
    soi_btdc: f64,
    /// Fuel share of a late DI pulse ending at `eoi_late_btdc` (the firmware's full-load mode OPP_2_S_1: 1st + late 3rd
    /// pulse), 0 = one pulse.
    late_share: f64,
    eoi_late_btdc: f64,
    put_target_kpa: f64,
    rail_target_mpa: f64,
    throttle_duty: f64,
    /// Cam phase targets (intake advance, exhaust retard), °CA.
    cams: [f64; 2],
    fuel: bool,
    ignition: bool,
    /// Port (tumble) flaps commanded closed.
    flaps_closed: bool,
    /// Port injection (MPI, the firmware's warm idle / part-load mode) instead of DI.
    mpi: bool,
}

impl Operator {
    fn wot(spark: f64, lambda: f64, put_kpa: f64) -> Self {
        Self { spark, dwell_ms: 2.0, lambda, soi_btdc: 300.0, late_share: 0.0, eoi_late_btdc: 85.0, put_target_kpa: put_kpa, rail_target_mpa: 18.0, throttle_duty: 1.0, cams: [25.0, 10.0], fuel: true, ignition: true, flaps_closed: false, mpi: false }
    }

    /// The bench's throttle position controller: `throttle_duty` sets the plate target as the fraction of the
    /// travel above (+) / below (−) limp-home; spring feed-forward from the actuator data + PD on the angle.
    fn throttle_bridge_duty(&self, p: &VirtualPlant) -> f64 {
        let (prof, e) = (&p.engine_profile, &p.engine);
        let (etc, lh) = (&prof.etc, prof.throttle_limp_home_deg);
        let d = self.throttle_duty.clamp(-1.0, 1.0);
        let target = if d >= 0.0 { lh + d * (etc.max_deg - lh) } else { lh + d * lh };
        let x = (target - lh).to_radians();
        let spring = if x > 0.0 { etc.close_preload_nm + etc.close_rate_nm_rad * x } else if x < 0.0 { -(etc.open_preload_nm - etc.open_rate_nm_rad * x) } else { 0.0 };
        let ff = spring * etc.r_ohm / etc.k_nm_a / p.battery_v;
        (ff + 0.02 * (target - e.throttle_deg) - 3e-4 * e.throttle_rate_rad_s.to_degrees()).clamp(-1.0, 1.0)
    }

    fn command(&self, p: &VirtualPlant) -> PlantActuators {
        let e = &p.engine;
        let prof = &p.engine_profile;
        let dp = (e.rail_pa - e.p_intake_pa).max(1e5);
        let q = prof.injector_flow_cc_s * 1e-6 * (dp / 1e7).sqrt();
        // Trapped air measured on the bench (cycle-resolved), speed-density estimate before the first cycle.
        let air = if e.flows.air_per_cyl_mg > 1.0 {
            e.flows.air_per_cyl_mg * 1e-6
        } else {
            0.85 * e.p_intake_pa * e.kin.v_disp / (287.0 * e.t_intake_k)
        };
        let m_f = air / (engine::AFR_STOICH * self.lambda);
        let pw = m_f / (engine::FUEL_DENSITY_KG_M3 * q) * 1000.0 + prof.injector_dead_time_ms;
        // HPFP: delivery fraction = feed-forward (one injection + the pump's internal loss per stroke over the
        // effective stroke) + P on the rail error → N276 closing angle on the plunger lift.
        let gross_mg = m_f * 1e6 + prof.hpfp_loss.mg_at(e.rpm(), (e.rail_pa - 101_325.0) / 100.0);
        let ff = gross_mg * 1e-6 / (engine::FUEL_DENSITY_KG_M3 * prof.hpfp_efficiency * prof.hpfp_stroke_cc * 1e-6);
        let frac = (ff + 0.25 * (self.rail_target_mpa - e.rail_pa / 1e6)).clamp(0.0, 1.0);
        let psi = (2.0 * frac - 1.0).clamp(-1.0, 1.0).acos() * 90.0 / PI;
        // Current on ahead of the wanted closing by the valve's pull-in delay (as the ECU does).
        let delay_deg = sensors::interp(p.battery_v, prof.hpfp_vcv_close_delay_ms) * 1e-3 * e.omega_rad_s.abs().to_degrees();
        let close = psi + engine::HPFP_TDC_DEG - 90.0 + e.vt.cam_shift_deg(prof, 1) - delay_deg;
        // Boost: P on the boost pressure → wanted wastegate flow fraction, then a position servo on the electric
        // actuator's H-bridge (+ = towards the seat).
        let open_sp = (0.5 + 0.025 * (e.p_boost_pa / 1e3 - self.put_target_kpa)).clamp(0.0, 1.0);
        let wg = e.wg.bench_servo_duty(&prof.turbo.wg_actuator, e.bench_wastegate_theta(&prof.turbo, open_sp), e.p_exhaust_pa - p.ambient_state().p_kpa * 1e3, p.battery_v);
        let mut a = PlantActuators::default();
        a.ecu.main_relay = Some(true);
        a.ecu.lpfp_duty = Some(0.8);
        // Fuel off = N276 de-energised (no delivery), as the ECU does at key-off.
        a.ecu.hpfp_close_deg = self.fuel.then_some(close);
        a.ecu.etc_duty = Some(self.throttle_bridge_duty(p));
        a.ecu.wastegate_duty = Some(wg);
        a.ecu.tumble_flaps_closed = Some(self.flaps_closed);
        for k in 0..2 {
            let err = self.cams[k] - e.vt.phaser[k].angle_deg;
            let g = if k == 0 { &prof.mech.intake } else { &prof.mech.exhaust };
            a.ecu.cam_phaser_duty[k] = Some((g.phaser.hold_duty(p.battery_v, p.thermal.oil_c) + 0.02 * err).clamp(0.0, 1.0));
        }
        for c in 0..4 {
            if self.ignition {
                a.ecu.ignition[c] = Some(CoilCmd { dwell_ms: self.dwell_ms, spark_btdc_deg: self.spark });
            }
            if self.fuel && self.mpi {
                // port injector on the low-pressure supply (flow ∝ √ΔP over the manifold), closed-valve timing
                let dp = (e.lp_fuel_pa + 101_325.0 - e.p_intake_pa).max(1e5);
                let q_mg_ms = prof.mpi_injector_mg_ms * (dp / (prof.mpi_ref_dp_kpa * 1e3)).sqrt();
                let dead = sensors::interp(p.battery_v, prof.mpi_dead_time_ms);
                a.ecu.port_injection[c][0] = Some(InjPulse { soi_btdc_deg: 370.0, width_ms: m_f * 1e6 / q_mg_ms + dead });
            } else if self.fuel && self.late_share > 0.0 {
                let w = |share: f64| (pw - prof.injector_dead_time_ms) * share + prof.injector_dead_time_ms;
                let late = w(self.late_share);
                a.ecu.injection[c][0] = Some(InjPulse { soi_btdc_deg: self.soi_btdc, width_ms: w(1.0 - self.late_share) });
                a.ecu.injection[c][2] = Some(InjPulse { soi_btdc_deg: self.eoi_late_btdc + late * e.rpm() * 6e-3, width_ms: late });
            } else if self.fuel {
                a.ecu.injection[c][0] = Some(InjPulse { soi_btdc_deg: self.soi_btdc, width_ms: pw });
            }
        }
        a
    }
}

fn plant() -> VirtualPlant {
    let mut prof = EngineProfile::ea888_18t_cjsa_is12();
    knobs(&mut prof);
    let mut p = VirtualPlant::new(prof, DrivetrainProfile::dq200_fwd(), Ambient { temp_c: 25.0, altitude_m: 0.0, humidity_pct: 40.0 });
    p.driver.ignition_on = true;
    p.driver.selector = Selector::N;
    p.thermal = ThermalState::uniform(90.0);
    p.engine.lube.bearings.iter_mut().for_each(|b| b.t_k = 363.15);
    p.engine.cyl.iter_mut().for_each(|c| {
        c.set_component_temps(480.0, 900.0);
    });
    p
}

fn dyno(rpm: f64) -> VirtualPlant {
    let mut p = plant();
    p.engine.set_speed(rpm * PI / 30.0);
    p.bench = BenchMode::EngineDyno { target_rpm: rpm };
    // The operator settings of these tests (boost of `stock_wot`) were found on a cell without a cooling fan
    // (intercooler at 0.55 × its rated effectiveness); kept that way. With the fan (`TEST_CELL_FAN_MS`) the same boost
    // traps more air: 240/249/258/248/257/248 Nm at 1250…5000 rpm, 137/143 kW at 5500/6000 (docs/SIMOS18_COMBUSTION.md §13).
    p.test_cell_fan_ms = 0.0;
    p.engine.t_boost_k = 313.0;
    p.engine.t_intake_k = 318.0;
    p.engine.rail_pa = 15e6;
    p.engine.lp_fuel_pa = 0.48e6;
    // exhaust AVS where CJSA's own set-point map puts it at full load (`ip_vlft_sp`, SC8O40 264H @0x8086B80E: LFT_1 up to
    // 2000 rpm, up to 2450 rpm above 750 mg/stk, standard from 2500 rpm); `AVS` overrides
    let lft1 = std::env::var("AVS").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(if rpm <= 2450.0 { 1.0 } else { 0.0 });
    p.engine.vt.avs.iter_mut().for_each(|a| a.pos = lft1);
    p
}

fn run(p: &mut VirtualPlant, op: &Operator, seconds: f64) {
    let frames = (seconds / 0.005).round() as usize;
    for _ in 0..frames {
        let a = op.command(p);
        p.step(0.005, &a);
        assert!(p.engine.rpm().is_finite() && p.engine.p_intake_pa.is_finite() && p.engine.cyl.iter().all(|c| c.p.is_finite()));
    }
}

/// Mean of a telemetry quantity over `seconds` of further running.
fn mean(p: &mut VirtualPlant, op: &Operator, seconds: f64, g: impl Fn(&PlantTelemetry) -> f64) -> f64 {
    let frames = (seconds / 0.005).round() as usize;
    let mut s = 0.0;
    for _ in 0..frames {
        let a = op.command(p);
        p.step(0.005, &a);
        s += g(&p.telemetry());
    }
    s / frames as f64
}

#[test]
#[ignore]
fn explore_wot_sweep() {
    let cases: Vec<(f64, f64, f64, [f64; 2])> = std::env::var("SWEEP").ok().map(|v| {
        v.split(';').map(|c| { let x: Vec<f64> = c.split(',').map(|y| y.trim().parse().unwrap()).collect(); (x[0], x[1], x[2], [x[3], x[4]]) }).collect()
    }).unwrap_or(vec![(1500.0, 4.0, 180.0, [25.0, 10.0])]);
    for (rpm, spark, put, cams) in cases {
        let mut p = dyno(rpm);
        let mut op = Operator::wot(spark, if rpm > 4500.0 { 0.86 } else { 0.9 }, put);
        op.cams = cams;
        run(&mut p, &op, 4.0);
        let tq = mean(&mut p, &op, 0.5, |t| t.torque_brake_nm);
        let t = p.telemetry();
        println!(
            "{rpm:5.0} spk {spark:4.1} cams {cams:?} tq {tq:6.1} ind {:6.1} fr {:5.1} pmp {:5.1} map {:5.1} exh {:5.1} air {:5.0} res {:.3} lam {:.2} pk {:5.1}@{:4.1} ca50 {:5.1} kn {:5.2} LW {:.2} egt {:4.0} turbo {:6.0} piston {:4.0} exv {:4.0} bfilm {:.2} ripple {:.0} fail {:?}",
            t.torque_indicated_nm, t.torque_friction_nm, t.torque_pumping_nm,
            t.map_kpa, t.exhaust_manifold_kpa, t.air_mg_stroke, t.residual_frac[0], t.lambda_sensor, t.cyl_peak_bar[0], t.cyl_peak_deg[0], t.ca50_deg[0], t.knock_amp_bar[0], t.knock_integral[0], t.egt_c, t.turbo_rpm, t.piston_crown_c[0], t.exhaust_valve_c[0],
            t.bearing_min_film_um.iter().cloned().fold(f64::MAX, f64::min), t.crank_speed_ripple_rpm, t.failures
        );
    }
}

#[test]
fn ignition_and_start_button_alone_do_nothing_without_firmware() {
    let mut p = plant();
    p.driver.start_button = true;
    for _ in 0..200 {
        p.step(0.01, &PlantActuators::default());
    }
    let t = p.telemetry();
    assert_eq!(t.engine_state, "stopped");
    assert!(!t.ecu_outputs_connected);
    assert!(p.engine.edges.is_empty());
}

/// Warm cranking with the 1.1 kW reduction starter on a 68 Ah battery (`starter` module): inrush of several
/// hundred amps at breakaway, no hang on the first compression (breakaway torque ≫ compression torque),
/// 200–300 rpm with the compression strokes modulating speed and current.
#[test]
fn starter_cranks_at_realistic_speed_and_sags_battery() {
    let mut p = plant();
    let mut a = PlantActuators::default();
    a.ecu.starter_relay = Some(true);
    let mut sum = 0.0;
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    let (mut i_peak, mut v_min, mut v_sum, mut i_sum, mut stalled_ms) = (0.0f64, f64::MAX, 0.0, 0.0, 0);
    for i in 0..2000 {
        p.step(0.001, &a);
        let t = p.telemetry();
        i_peak = i_peak.max(t.starter_current_a);
        v_min = v_min.min(t.battery_v);
        if i > 20 && t.rpm < 5.0 {
            stalled_ms += 1;
        }
        if i >= 1000 {
            sum += p.engine.rpm();
            lo = lo.min(p.engine.rpm());
            hi = hi.max(p.engine.rpm());
            v_sum += t.battery_v;
            i_sum += t.starter_current_a;
        }
    }
    let t = p.telemetry();
    let (avg, v_avg, i_avg) = (sum / 1000.0, v_sum / 1000.0, i_sum / 1000.0);
    println!(
        "cranking mean {avg:.0} rpm ({lo:.0}..{hi:.0}), {i_avg:.0} A at {v_avg:.2} V; inrush {i_peak:.0} A, {v_min:.2} V; peak {:?} bar",
        t.cyl_peak_bar
    );
    assert_eq!(t.engine_state, "cranking");
    assert!((180.0..330.0).contains(&avg), "cranking rpm {avg}");
    assert!(hi - lo > 20.0, "compression strokes must modulate the cranking speed");
    assert!(stalled_ms < 20, "starter hangs on a compression for {stalled_ms} ms");
    assert!((300.0..700.0).contains(&i_peak) && (8.0..10.5).contains(&v_min), "inrush {i_peak} A / {v_min} V");
    assert!((80.0..300.0).contains(&i_avg) && (10.0..12.3).contains(&v_avg), "cranking {i_avg} A / {v_avg} V");
    assert!(t.cyl_peak_bar.iter().all(|&b| (6.0..20.0).contains(&b)), "cranking compression {:?}", t.cyl_peak_bar);
}

/// Cylinder-1 (p, V) trace over one cycle at the current state (bench indicating system).
fn indicate(p: &mut VirtualPlant, op: &Operator) -> Vec<(f64, f64, f64)> {
    let mut v = Vec::new();
    let start = p.engine.crank_total_deg;
    while p.engine.crank_total_deg - start < 720.0 {
        let a = op.command(p);
        p.step(DT_SUB, &a);
        let c = &p.engine.cyl[0];
        v.push((crate::cylinder::atdc(p.engine.crank_deg), c.p, c.v));
    }
    v
}

#[test]
fn motoring_trace_has_sensible_peak_and_polytropic_exponent() {
    let mut p = dyno(1500.0);
    let mut op = Operator::wot(0.0, 1.0, 100.0);
    op.fuel = false;
    op.ignition = false;
    op.cams = [30.0, 0.0];
    // Motoring dyno (test equipment) holds 1500 rpm.
    for _ in 0..400 {
        let a = op.command(&p);
        p.step(0.005, &a);
        p.engine.set_speed(1500.0 * PI / 30.0);
    }
    let tr = indicate(&mut p, &op);
    let (deg, pmax, _) = tr.iter().cloned().fold((0.0, 0.0, 0.0), |m, x| if x.1 > m.1 { x } else { m });
    let near = |a: f64| *tr.iter().min_by(|x, y| (x.0 - a).abs().total_cmp(&(y.0 - a).abs())).unwrap();
    let (a1, a2) = (near(-60.0), near(-20.0));
    let n = (a2.1 / a1.1).ln() / (a1.2 / a2.2).ln();
    println!("motoring: peak {:.1} bar at {deg:.1}°, polytropic n {n:.3}", pmax / 1e5);
    assert!((15.0..25.0).contains(&(pmax / 1e5)), "peak {}", pmax / 1e5);
    assert!(deg.abs() < 3.0, "motoring peak must sit just before TDC (heat loss/blow-by): {deg}");
    assert!((1.24..1.36).contains(&n), "polytropic exponent {n}");
}

#[test]
fn warm_start_fires_and_idles_stably_then_stops_with_compression_bounce() {
    let mut p = plant();
    let mut op = Operator::wot(8.0, 1.0, 0.0);
    op.throttle_duty = 0.0;
    op.cams = [0.0, 0.0];
    op.rail_target_mpa = 5.0;
    // Crank with the starter; the operator injects and fires from the first revolution.
    let mut t_run = None;
    for i in 0..400 {
        let mut a = op.command(&p);
        a.ecu.starter_relay = Some(t_run.is_none() && i < 300);
        p.step(0.005, &a);
        if t_run.is_none() && p.engine.rpm() > 600.0 {
            t_run = Some(i as f64 * 0.005);
        }
    }
    let t_run = t_run.expect("engine must start");
    assert!(t_run < 1.5, "start took {t_run} s");
    // Idle at a fixed small throttle opening (≈ 2.8° below limp-home: 0.18 cm² of the calibrated throttle body less the
    // 0.055 cm² deposit, `EngineProfile::throttle_deposit_cm2`; the clean body was held at 1.4°, 0.10 cm²):
    // speed must settle and stay smooth.
    op.throttle_duty = -0.6;
    run(&mut p, &op, 4.0);
    let mut r = Vec::new();
    for _ in 0..400 {
        let a = op.command(&p);
        p.step(0.005, &a);
        r.push(p.engine.rpm());
    }
    let t = p.telemetry();
    let m = r.iter().sum::<f64>() / r.len() as f64;
    let sd = (r.iter().map(|x| (x - m).powi(2)).sum::<f64>() / r.len() as f64).sqrt();
    let imep = t.imep_bar;
    let im = imep.iter().sum::<f64>() / 4.0;
    let cov = (imep.iter().map(|x| (x - im).powi(2)).sum::<f64>() / 4.0).sqrt() / im;
    println!("start {t_run:.2} s; idle {m:.0} ± {sd:.1} rpm, MAP {:.1} kPa, IMEP {:?}, ripple {:.0}", t.map_kpa, imep, t.crank_speed_ripple_rpm);
    assert_eq!(t.engine_state, "running");
    assert!((600.0..1100.0).contains(&m), "idle {m}");
    assert!(sd < 30.0, "idle speed scatter {sd}");
    assert!(cov < 0.05, "IMEP spread {cov}");
    assert!((20.0..55.0).contains(&t.map_kpa), "idle MAP {}", t.map_kpa);
    // Key off: no fuel, no spark. The crank stops and rocks back on a compression stroke.
    op.fuel = false;
    op.ignition = false;
    let mut reversed = false;
    for _ in 0..800 {
        let a = op.command(&p);
        p.step(0.005, &a);
        reversed |= p.engine.omega_rad_s < 0.0 || p.engine.edges.iter().any(|e| matches!(e.source, EdgeSource::CrankTooth { forward: false, .. }));
    }
    assert_eq!(p.engine.rpm(), 0.0, "engine must come to rest");
    assert!(reversed, "compression bounce must turn the crank backwards before it stops");
}

/// Stock-like full-load operating point for the CJSA on the dyno: boost, λ (component protection
/// above 4500 rpm) and cam phasing (scavenging at low speed) as a calibration engineer would set them.
fn stock_wot(rpm: f64, spark: f64) -> Operator {
    // Boost: the knock-limited spark of the quasi-dimensional combustion (Douaud–Eyzat unscaled, within ≈ 1–2 ° of the
    // SC8O40 base-ignition map, docs/SIMOS18_COMBUSTION.md) needs more charge above 4000 rpm for the rated power.
    let put = crate::materials::interp(&[(1250.0, 205.0), (1500.0, 195.0), (2000.0, 186.0), (3000.0, 182.0), (4000.0, 186.0), (5000.0, 203.0), (6000.0, 199.0)], rpm)
        * std::env::var("PUT_SCALE").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
    let mut op = Operator::wot(spark, if rpm > 4500.0 { 0.86 } else { 0.9 }, put);
    op.cams = if rpm < 1750.0 {
        [45.0, 25.0]
    } else if rpm < 2500.0 {
        [40.0, 20.0]
    } else if rpm < 3500.0 {
        [30.0, 15.0]
    } else if rpm < 4500.0 {
        [25.0, 10.0]
    } else if rpm < 5250.0 {
        [20.0, 5.0]
    } else {
        [15.0, 5.0]
    };
    // Injection as SC8S50/SC8O40 calibrate full load up to 4000 rpm (OPP_2_S_1, docs/SIMOS18_GPTA_IO_RE.md §12.10.2): a late
    // 3rd pulse with 0.2–0.3 of the fuel at 750–1050 mg (`ip_fac_opp_2_s_1_pls_3` @0x808365A0) ending 75–100 °BTDC
    // (`ip_eoi_3_opp_2_s_1_cus` @0x8084705C); one intake-stroke pulse above.
    op.late_share = if rpm <= 4000.0 { 0.3 } else { 0.0 };
    op
}

/// Knock-limited spark search: retard from an MBT-side start until at most 5 % of the cycles of all cylinders knock with
/// a first-mode amplitude above 1 bar (trace knock as a cycle share — with cycle-to-cycle variation knock is a
/// distribution, not a level), like a calibration engineer.
fn knock_limited(p: &mut VirtualPlant, rpm: f64) -> (Operator, f64) {
    knock_limited_at(p, rpm, None)
}

/// [`knock_limited`]; with `air_mg` the operator trims the boost target so that the engine traps that air charge.
fn knock_limited_at(p: &mut VirtualPlant, rpm: f64, air_mg: Option<f64>) -> (Operator, f64) {
    let mut spark = (8.0 + 4.0 * (rpm - 1500.0) / 1000.0).clamp(4.0, 28.0);
    let mut op = stock_wot(rpm, spark);
    let mut put = op.put_target_kpa;
    let trim = |p: &VirtualPlant, put: &mut f64| {
        if let Some(target) = air_mg {
            *put = (*put * (1.0 + 0.5 * (target - p.telemetry().air_mg_stroke) / target)).clamp(100.0, 260.0);
        }
    };
    for _ in 0..5 {
        op.put_target_kpa = put;
        run(p, &op, 0.5);
        trim(p, &mut put);
    }
    loop {
        op = stock_wot(rpm, spark);
        op.put_target_kpa = put;
        run(p, &op, 0.6);
        trim(p, &mut put);
        if sample_cycles(p, &op, 15).knock_share <= 0.05 || spark < -8.0 {
            return (op, spark);
        }
        spark -= 1.5;
    }
}

/// Stock CJSA full-load air charge, mg/stroke: VCDS log L1 (stock software and hardware, 30 °C, 4th gear;
/// docs/EA888_STOCK_REFERENCE.md §3.4.2: air = 8333·Q[kg/h]/rpm, inferred), 2017…6155 rpm. Below 2000 rpm the log is
/// still spooling the turbo; the rated 250 Nm from 1250 rpm (P2 p.6) needs about the 2000-rpm charge at the 2000-rpm
/// efficiency → 820 mg (inferred).
fn stock_air_mg(rpm: f64) -> f64 {
    crate::materials::interp(
        &[(1250.0, 820.0), (2017.0, 847.0), (2302.0, 840.0), (2601.0, 785.0), (2859.0, 767.0), (3129.0, 780.0), (3390.0, 809.0), (3677.0, 827.0), (3957.0, 819.0), (4237.0, 806.0), (4536.0, 825.0), (4791.0, 830.0), (5089.0, 822.0), (5315.0, 811.0), (5550.0, 803.0), (5775.0, 747.0), (5963.0, 732.0), (6155.0, 695.0)],
        rpm,
    )
}

#[test]
fn full_load_curve_matches_cjsa_with_knock_limited_spark() {
    let mut rows = Vec::new();
    for &rpm in &[1250.0, 1500.0, 2000.0, 3000.0, 4000.0, 5000.0, 5500.0, 6000.0] {
        let mut p = dyno(rpm);
        let (op, spark) = knock_limited(&mut p, rpm);
        let tq = mean(&mut p, &op, 0.5, |t| t.torque_brake_nm);
        let t = p.telemetry();
        let kw = tq * rpm * PI / 30.0 / 1000.0;
        let pk = t.cyl_peak_bar.iter().cloned().fold(0.0, f64::max);
        let fuel_w = t.fuel_mg_stroke.iter().sum::<f64>() * 1e-6 * engine::LHV_J_KG * rpm / 120.0;
        println!(
            "{rpm:5.0} rpm: spark {spark:5.1} °BTDC  {tq:5.1} Nm  {kw:5.1} kW  air {:4.0} mg  MAP {:5.1} kPa  pmax {pk:5.1} bar  CA50 {:4.1}  knock {:.2} bar  EGT {:4.0} °C  turbo {:6.0}  η_b {:.3}  piston {:3.0} °C",
            t.air_mg_stroke, t.map_kpa, t.ca50_deg[0], t.knock_amp_bar.iter().cloned().fold(0.0, f64::max), t.egt_c, t.turbo_rpm, kw * 1e3 / fuel_w, t.piston_crown_c[0]
        );
        assert!(t.failures.is_empty(), "{:?}", t.failures);
        rows.push((rpm, tq, kw, pk, t.turbo_rpm, t.turbo_stress_ratio[0]));
    }
    // Against the published CJSA curve (250 Nm 1250–5000 rpm, 132 kW 5100–6200 rpm; P2 p.6, docs/EA888_STOCK_REFERENCE.md
    // §3.5.1). With the late full-load pulse the firmware calibrates (`stock_wot`, docs/SIMOS18_COMBUSTION.md §11):
    // 228/229/248/240/249/235 Nm at 1250…5000 rpm, 128/131 kW at 5500/6000 (was 224/224/244/231/242/235 Nm, 128/131 kW
    // with one intake-stroke pulse). 1250/1500 rpm stay ≈ 9 % low, knock-limited at a CA50 of ≈ 33 °ATDC (the plant's limit
    // ≈ 2.4 ° later than SC8O40's base ignition at 1500 rpm; open, ibid. §8). Tolerance 10 % (was 11 %).
    for &(rpm, tq, kw, pk, turbo, sr) in &rows {
        if rpm <= 5000.0 {
            assert!((tq - 250.0).abs() <= 0.10 * 250.0, "{rpm} rpm: {tq:.1} Nm");
        } else {
            assert!((kw - 132.0).abs() <= 0.07 * 132.0, "{rpm} rpm: {kw:.1} kW");
        }
        // Knock-retarded points peak low (CA50 20–39 °ATDC).
        assert!((45.0..125.0).contains(&pk), "{rpm} rpm peak pressure {pk}");
        assert!(turbo < 230_000.0 && sr < 0.8, "{rpm}: turbo {turbo} σ/UTS {sr}");
    }
    let pk_max = rows.iter().map(|r| r.3).fold(0.0, f64::max);
    assert!((60.0..125.0).contains(&pk_max), "full-load peak pressure {pk_max}");
}

/// Knock-limited spark of the stock-like calibration found by `full_load_curve_…` (1.5° margin).
fn stock_spark(rpm: f64) -> f64 {
    const T: [(f64, f64); 8] = [(1250.0, -5.0), (1500.0, -2.5), (2000.0, 2.5), (3000.0, 8.0), (4000.0, 12.0), (5000.0, 16.0), (5500.0, 18.0), (6000.0, 20.0)];
    crate::materials::interp(&T, rpm) - 1.5
}

fn report(t: &PlantTelemetry, tag: &str) {
    println!(
        "{tag}: t {:.1} rpm {:.0} tq {:.0} map {:.0} put {:.0} wg {:.0}% cool {:.0} oilT {:.0} iat {:.0} thr {:.0} kn {:.1?} ks {:.0?} crown {:.0?} land {:.0?} groove {:.0?} σ/UTS {:.2?} pdmg {:.2e} pk {:.0?} rod {:.2?} hg {:.2?} float {:.2?} hla {:.2?} bend {:.2?} film {:.2?} brgC {:.0?} oil {:.2} turbo {:.0} sr {:.2?} fail {:?}",
        t.sim_time_s, t.rpm, t.torque_brake_nm, t.map_kpa, t.put_kpa, t.wastegate_open_pct, t.coolant_c, t.oil_c, t.iat_c, t.throttle_pct, t.knock_amp_bar, t.knock_sensor_mv, t.piston_crown_c, t.combustion.piston_land_c, t.combustion.piston_groove_c, t.combustion.land_stress_ratio, t.piston_damage.iter().cloned().fold(0.0, f64::max), t.cyl_peak_bar, t.rod_bend_mm, t.head_gasket_damage,
        t.valve_float_mm, t.hla_pumpup_mm, t.valve_bend_mm, t.bearing_min_film_um, t.bearing_c, t.oil_bar, t.turbo_rpm, t.turbo_stress_ratio, t.failures
    );
}

#[test]
fn over_advanced_spark_on_boost_knocks_and_destroys_ring_lands_over_time() {
    // 15 ° past the knock limit at 1.95 bar: heavy knock (about a third of the charge autoignites as a developing
    // detonation, 30–50 bar first-mode amplitude), the knock-scrubbed boundary layer heats the ring belt to 350–400 °C
    // where AlSi12CuNiMg has lost most of its fatigue strength; the land is consumed within hours.
    // Time to fracture is not a calibrated quantity: the Miner sum grows exponentially with the land temperature and the
    // local pressure (p + knock oscillation); the test-cell conditions coolant and oil (no airflow over the radiator).
    let mut p = dyno(2000.0);
    let mut op = stock_wot(2000.0, 18.0);
    op.lambda = 0.9;
    op.put_target_kpa = 195.0;
    op.late_share = 0.0; // one intake-stroke pulse: no late-pulse charge cooling in this abusive map
    let mut dmg = Vec::new();
    let mut heavy = 0.0f64;
    for k in 0..400 {
        run(&mut p, &op, 1.0);
        p.thermal.engine_c = p.thermal.engine_c.min(95.0);
        p.thermal.oil_c = p.thermal.oil_c.min(110.0);
        let t = p.telemetry();
        heavy = heavy.max(t.knock_amp_bar.iter().cloned().fold(0.0, f64::max));
        if k == 20 {
            assert!(t.combustion.piston_land_c.iter().all(|&x| x > 300.0), "knock heats the ring belt: {:?}", t.combustion.piston_land_c);
        }
        dmg.push(t.piston_damage.iter().cloned().fold(0.0, f64::max));
        if k % 20 == 0 || !t.failures.is_empty() {
            report(&t, "knock");
        }
        if t.ring_land_failed.iter().any(|&x| x) {
            break;
        }
    }
    let t = p.telemetry();
    report(&t, "knock-end");
    assert!(heavy > 20.0, "heavy knock expected 15 ° past the knock limit: {heavy:.1} bar");
    assert!(dmg.windows(2).all(|w| w[1] >= w[0]), "damage is cumulative");
    // The ring belt soaks to 350–400 °C within ~40 s; a land broken earlier by a single heavy cycle (peak wall pressure
    // above the UTS of the hot land) has not had that time, but must have been heated by the knock.
    let fractured = t.ring_land_failed.iter().any(|&x| x);
    let land_max = t.combustion.piston_land_c.iter().cloned().fold(0.0, f64::max);
    assert!(land_max > if fractured { 300.0 } else { 350.0 }, "knock overheats the ring belt: {land_max:.0} °C");
    // Either the land has fractured already, or the worst land's Miner rate over the last 100 s gives a life of days (the
    // stock calibration's full-load pull accumulates < 1e-3 in 35 s, i.e. a life of years). With the M124 strength data
    // (`materials.rs`, LAJSS 2016 / MAHLE) 30–45 bar first-mode cycles at 350–370 °C land temperature fracture a land in
    // ≈ 50 s (2026-11-03; before, with the handbook-class table and the leak growing with damage: 4–8 days) — the order of
    // Fitton & Nates 1992, Table 1: heavy knock 0.4 min to land failure at 4000 rpm.
    let n = dmg.len();
    let rate = (dmg[n - 1] - dmg[n.saturating_sub(101)]) / (n.min(101) - 1).max(1) as f64;
    let life_s = (1.0 - dmg[n - 1]) / rate.max(1e-12);
    assert!(t.ring_land_failed.iter().any(|&x| x) || life_s < 10.0 * 24.0 * 3600.0, "severe knock must destroy a ring land: damage {:?}, life {life_s:.0} s", t.piston_damage);
}

/// Over-rev (missed shift): the valves float and the hydraulic lash adjusters pump up. With the exhaust lobes of the
/// calibrations (`AvsHw`, standard lobe 9 mm / 195 °: lower accelerations than the former 10 mm / 240 ° zero-lift profile)
/// the exhaust valves no longer float at 8800 rpm; the intake ones do and pump their HLA to its travel, but their piston
/// pocket (designed for the 60 ° intake advance) stays clear — the former valve–piston contact came from the old exhaust
/// profile. Pocket depth and HLA travel are UNVERIFIED (docs/SIMOS18_COMBUSTION.md §8).
#[test]
fn overrev_floats_valves_and_pumps_up_lifters() {
    let spin = |rpm: f64, secs: f64| {
        let mut p = dyno(rpm);
        let mut op = stock_wot(rpm, 20.0);
        op.fuel = false;
        op.ignition = false;
        op.throttle_duty = -1.0;
        op.cams = [12.5, 7.5];
        for _ in 0..(secs / 0.005) as usize {
            let a = op.command(&p);
            p.step(0.005, &a);
            p.engine.set_speed(rpm * PI / 30.0); // driven by the wheels in a missed shift / motoring dyno
        }
        p.telemetry()
    };
    let ok = spin(7000.0, 3.0);
    report(&ok, "7000");
    assert!(ok.valve_float_mm.iter().all(|&x| x < 0.01) && ok.failures.is_empty(), "7000 rpm must be safe");
    let t = spin(8800.0, 12.0);
    report(&t, "8800");
    assert!(t.valve_float_mm.iter().cloned().fold(0.0, f64::max) > 0.1, "valve float at 8800 rpm");
    assert!(t.hla_pumpup_mm.iter().cloned().fold(0.0, f64::max) > 1.0, "lifters pump up: {:?}", t.hla_pumpup_mm);
    assert!(t.failures.iter().any(|f| f.contains("valve float")), "{:?}", t.failures);
}

#[test]
fn oil_starvation_at_load_collapses_bearing_films_and_seizes_the_crank() {
    let rpm = 4000.0;
    let mut p = dyno(rpm);
    let op = stock_wot(rpm, stock_spark(rpm));
    run(&mut p, &op, 3.0);
    let healthy = p.telemetry();
    report(&healthy, "oil-ok");
    let film_ok = healthy.bearing_min_film_um.iter().cloned().fold(f64::MAX, f64::min);
    p.engine.lube.oil_l = 1.6; // leak: sump nearly empty, pickup draws air
    let mut seized_at = None;
    for k in 0..300 {
        run(&mut p, &op, 0.1);
        let t = p.telemetry();
        if k % 10 == 0 {
            report(&t, "oil-low");
        }
        if t.engine_state == "seized" {
            seized_at = Some(t.sim_time_s);
            report(&t, "seized");
            break;
        }
    }
    let t = p.telemetry();
    assert!(film_ok > 0.5, "healthy min film {film_ok} µm");
    assert!(t.oil_aeration > 0.5 && t.oil_bar < 0.5, "oil pressure must collapse: {} bar", t.oil_bar);
    assert!(seized_at.is_some(), "bearings must seize");
    assert!(t.failures.iter().any(|f| f.contains("seized")));
    assert_eq!(p.engine.rpm(), 0.0);
}

#[test]
fn massive_overboost_breaks_something_physical() {
    let rpm = 5000.0;
    let mut p = dyno(rpm);
    let mut op = stock_wot(rpm, stock_spark(rpm));
    op.put_target_kpa = 400.0; // wastegate held shut
    for k in 0..200 {
        run(&mut p, &op, 0.1);
        let t = p.telemetry();
        if k % 10 == 0 {
            report(&t, "overboost");
        }
        if !t.failures.is_empty() && k > 5 {
            report(&t, "overboost-fail");
            break;
        }
    }
    let t = p.telemetry();
    assert!(
        t.failures.iter().any(|f| f.contains("turbocharger") || f.contains("connecting rod") || f.contains("head gasket") || f.contains("ring land")),
        "{:?}",
        t.failures
    );
}

#[test]
fn stock_calibration_survives_a_long_full_load_pull() {
    let mut p = dyno(2000.0);
    let mut rpm = 2000.0;
    run(&mut p, &stock_wot(rpm, stock_spark(rpm)), 2.0);
    // 2000 → 6200 rpm in 25 s, then 10 s at rated speed.
    for i in 0..((35.0 / 0.005) as usize) {
        rpm = (2000.0 + 4200.0 * (i as f64 * 0.005 / 25.0)).min(6200.0);
        p.bench = BenchMode::EngineDyno { target_rpm: rpm };
        let op = stock_wot(rpm, stock_spark(rpm));
        let a = op.command(&p);
        p.step(0.005, &a);
    }
    let t = p.telemetry();
    report(&t, "pull");
    assert!(t.failures.is_empty(), "{:?}", t.failures);
    assert!(t.piston_damage.iter().all(|&d| d < 1e-3), "{:?}", t.piston_damage);
    assert!(t.rod_damage.iter().all(|&d| d < 1e-6) && t.head_gasket_damage.iter().all(|&d| d < 1e-4));
    assert!(t.bearing_min_film_um.iter().all(|&h| h > 0.5), "{:?}", t.bearing_min_film_um);
    assert!(t.valve_float_mm.iter().all(|&x| x < 0.01));
    assert!(t.turbo_damage < 1e-3 && t.turbo_stress_ratio[0] < 0.8);
    assert!(t.cylinder_health.iter().all(|&h| h > 0.99));
}


/// Real-time factor of the plant alone (release build): `cargo test --release perf -- --ignored --nocapture`.
#[test]
#[ignore]
fn perf_realtime_factor() {
    for &rpm in &[800.0, 3000.0, 7000.0] {
        let mut p = dyno(rpm);
        let op = stock_wot(rpm, stock_spark(rpm.min(6000.0)));
        run(&mut p, &op, 1.0);
        let t0 = std::time::Instant::now();
        run(&mut p, &op, 10.0);
        let wall = t0.elapsed().as_secs_f64();
        println!("{rpm:5.0} rpm WOT dyno: 10 s simulated in {wall:.3} s wall → {:.1}× real time", 10.0 / wall);
    }
    let mut p = plant();
    p.driver.selector = Selector::N;
    p.engine.set_speed(7000.0 * PI / 30.0);
    let mut op = stock_wot(7000.0, 26.0);
    op.throttle_duty = 0.2;
    let t0 = std::time::Instant::now();
    run(&mut p, &op, 10.0);
    println!("road mode + DQ200 driveline, ~7000 rpm part load: {:.1}× real time", 10.0 / t0.elapsed().as_secs_f64());
}

/// Overboost on a big turbo (IS38 hybrid on the stock 1.8, boost target far above stock, over-advanced spark): the turbo
/// survives; the long block runs at 110–150 bar peak pressure in heavy knock and its ring lands accumulate fatigue
/// damage, or a rod / gasket / land fails within the run. Since the re-identified burn (docs/SIMOS18_COMBUSTION.md §3.3)
/// the knock at this spark is 10–20 bar (was 27–35 bar) and the damage needs longer than this 45 s run, so a failure is
/// no longer required.
#[test]
fn overboost_on_a_big_turbo_knocks_and_loads_the_long_block() {
    // IS38 hybrid survives the boost; the long block has to carry it.
    let rpm = 3500.0;
    let mut p = dyno(rpm);
    p.engine_profile = EngineProfile::ea888_18t_is38_hybrid();
    let mut op = stock_wot(rpm, 14.0);
    op.put_target_kpa = 320.0;
    // Open-loop fuel command. The operating point is λ ≈ 0.75 on the wideband: with the pump phased to
    // its calibrated TDC (c_ang_ref_vcv = 0, previously 90° off) the rail ripple at injection leans the
    // open-loop command by ~0.03 λ, so the command is 0.72 (it was 0.82 → 0.75 measured before).
    op.lambda = 0.72;
    for k in 0..450 {
        run(&mut p, &op, 0.1);
        let t = p.telemetry();
        if k % 20 == 0 {
            report(&t, "is38");
        }
        if t.failures.iter().any(|f| f.contains("connecting rod") || f.contains("head gasket") || f.contains("ring land")) {
            report(&t, "is38-fail");
            break;
        }
    }
    let t = p.telemetry();
    assert!(!t.failures.iter().any(|f| f.contains("turbocharger")), "{:?}", t.failures);
    let failed = t.failures.iter().any(|f| f.contains("connecting rod") || f.contains("head gasket") || f.contains("ring land"));
    let max = |v: &[f64]| v.iter().cloned().fold(0.0, f64::max);
    let loaded = max(&t.cyl_peak_bar) > 110.0 && max(&t.knock_amp_bar) > 5.0 && max(&t.combustion.land_stress_ratio) > 0.4 && max(&t.piston_damage) > 0.0;
    assert!(failed || loaded, "pk {:?} knock {:?} σ/UTS {:?} damage {:?} failures {:?}", t.cyl_peak_bar, t.knock_amp_bar, t.combustion.land_stress_ratio, t.piston_damage, t.failures);
}

/// Exploration (evidence for docs/SIMOS18_GPTA_IO_RE.md §12.6): spark sweep at fixed throttle, prints the plant's MBT
/// (max indicated torque) per point, to compare with the calibration's reference ignition angle (MBT) map
/// `ip_iga_ref_ivvt_vvl_port_h`. `PTS="rpm,throttle_duty;..."`.
#[test]
#[ignore]
fn explore_mbt_vs_calibration() {
    let prof = EngineProfile::ea888_20t_cjxc_is38();
    let pts: Vec<(f64, f64)> = std::env::var("PTS").ok().map(|v| v.split(';').map(|c| { let x: Vec<f64> = c.split(',').map(|y| y.parse().unwrap()).collect(); (x[0], x[1]) }).collect()).unwrap_or(vec![(800.0, -0.66)]);
    for (rpm, thr) in pts {
        let mut best = (f64::MIN, 0.0, 0.0, 0.0);
        for k in 0..16 {
            let spark = -10.0 + 4.0 * k as f64;
            let mut p = VirtualPlant::new(prof.clone(), DrivetrainProfile::dq200_fwd(), Ambient::default());
            p.thermal = ThermalState::uniform(90.0);
            p.engine.set_speed(rpm * PI / 30.0);
            p.bench = BenchMode::EngineDyno { target_rpm: rpm };
            p.engine.rail_pa = 10e6;
            p.engine.lp_fuel_pa = 0.5e6;
            let mut op = Operator::wot(spark, 1.0, 100.0);
            op.throttle_duty = thr;
            op.cams = [0.0, 0.0];
            op.rail_target_mpa = 10.0;
            run(&mut p, &op, 2.0);
            let ind = mean(&mut p, &op, 0.5, |t| t.torque_indicated_nm);
            let t = p.telemetry();
            if std::env::var("VERB").is_ok() { println!("   spark {spark:5.1}: ind {ind:6.2} ca50 {:5.1} pmax {:5.1}@{:5.1} imep {:.2} lam {:.2} firing {:?}", t.ca50_deg[0], t.cyl_peak_bar[0], t.cyl_peak_deg[0], t.imep_bar[0], t.lambda_cyl[0], t.firing); }
            if ind > best.0 { best = (ind, spark, t.air_mg_stroke, t.ca50_deg[0]); }
        }
        println!("{rpm:5.0} rpm thr {thr:5.2}: air {:6.1} mg MBT spark {:5.1} °BTDC (CA50 {:5.1}) ind {:.1} Nm", best.2, best.1, best.3, best.0);
    }
}

/// Bench operator for the calibration comparison: holds `target_mg` of trapped air by the throttle (secant on the
/// plate target every 0.25 s), λ 1, the given cam phases and port flaps.
fn hold_air(p: &mut VirtualPlant, op: &mut Operator, target_mg: f64, seconds: f64) {
    for _ in 0..(seconds / 0.25) as usize {
        run(p, op, 0.25);
        let air = p.telemetry().air_mg_stroke.max(1.0);
        op.throttle_duty = (op.throttle_duty + 0.15 * (target_mg - air) / target_mg).clamp(-1.0, 1.0);
    }
}

fn grid_plant(engine: &str, rpm: f64) -> VirtualPlant {
    let mut prof = EngineProfile::from_slug(engine).expect("engine slug");
    if let Some(d) = std::env::var("IN_DUR").ok().and_then(|v| v.parse().ok()) {
        prof.mech.intake.duration_deg = d;
    }
    if let Some(d) = std::env::var("IN_CL").ok().and_then(|v| v.parse().ok()) {
        prof.mech.intake.centerline_deg = d;
    }
    if let Some(d) = std::env::var("CCV_U").ok().and_then(|v| v.parse().ok()) {
        prof.mech.combustion.ccv_turb_sigma = d;
    }
    if let Some(d) = std::env::var("CCV_PHI").ok().and_then(|v| v.parse().ok()) {
        prof.mech.combustion.ccv_phi_sigma = d;
    }
    if let Some(d) = std::env::var("H_MULT").ok().and_then(|v| v.parse().ok()) {
        prof.mech.intake_heat.h_mult = d;
    }
    knobs(&mut prof);
    if std::env::var("IM_UA0").is_ok() {
        // manifold air temperature independent of the coolant (isolates the port / valve heat pick-up)
        prof.intake_wall_head_ua_w_k = 0.0;
    }
    let mut p = VirtualPlant::new(prof, DrivetrainProfile::dq200_fwd(), Ambient::default());
    p.thermal = ThermalState::uniform(std::env::var("COOLANT").ok().and_then(|v| v.parse().ok()).unwrap_or(90.0));
    p.engine.lube.bearings.iter_mut().for_each(|b| b.t_k = 363.15);
    p.engine.cyl.iter_mut().for_each(|c| c.set_component_temps(470.0, 850.0));
    p.engine.set_speed(rpm * PI / 30.0);
    p.bench = BenchMode::EngineDyno { target_rpm: rpm };
    // motoring drive holds the speed exactly (the absorber alone cannot pull a misfiring point back up)
    p.motoring_rpm = Some(rpm);
    p.engine.rail_pa = 10e6;
    p.engine.lp_fuel_pa = 0.5e6;
    p.engine.t_intake_k = 318.0;
    // exhaust AVS cam pieces as assembled on the bench: `AVS=1` = LFT_1 (small lobe), 0 = standard
    if let Some(x) = std::env::var("AVS").ok().and_then(|v| v.parse::<f64>().ok()) {
        p.engine.vt.avs.iter_mut().for_each(|a| a.pos = x);
    }
    p
}

/// Combustion-model constants from the environment (re-identification sweeps; unset = profile values).
fn knobs(prof: &mut EngineProfile) {
    let knob = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<f64>().ok());
    let cb = &mut prof.mech.combustion;
    let [to, tc] = &mut cb.tumble_port;
    for (k, f) in [
        ("TUMBLE_O", to as &mut f64),
        ("TUMBLE_C", tc),
        ("TUMBLE_DECAY", &mut cb.tumble_decay),
        ("TUMBLE_H0", &mut cb.tumble_h0),
        ("LEN_SCALE", &mut cb.length_scale),
        ("KERNEL_CK", &mut cb.kernel_ck),
        ("BURNUP_C", &mut cb.burnup_c),
        ("FLAME_C", &mut cb.flame_c),
        ("C_BETA", &mut cb.c_beta),
    ] {
        if let Some(v) = knob(k) {
            *f = v;
        }
    }
}

/// Exploration / calibration evidence: plant MBT and ignition-efficiency curve on the bench at the firmware's
/// reference-ignition grid points. `ENGINE` = profile slug (default `cjxc`), `PTS = "rpm,air_mg,cam_in,cam_ex,flaps;…"`
/// (cam phases from the locked rest position as the firmware's CAM_VCP, flaps 1 = closed). Prints one CSV row per point:
/// MBT (parabola through the best three sparks), CA50 there, burn durations, u', S_L, residual, torque ratio at
/// MBT − 10/20/30/40 °.
#[test]
#[ignore]
fn explore_combustion_grid() {
    let engine = std::env::var("ENGINE").unwrap_or("cjxc".into());
    // optional 6th field: the firmware's reference ignition `ip_iga_ref_ivvt_vvl_port_h/l` at the point (°BTDC)
    let pts: Vec<[f64; 6]> = std::env::var("PTS")
        .ok()
        .map(|v| v.split(';').map(|c| { let x: Vec<f64> = c.split(',').map(|y| y.trim().parse().unwrap()).collect(); [x[0], x[1], x[2], x[3], x[4], x.get(5).copied().unwrap_or(f64::NAN)] }).collect())
        .unwrap_or(vec![[760.0, 110.0, 15.0, 0.0, 1.0, f64::NAN]]);
    // SC8S50 259J ignition-angle efficiency vs retard from the reference angle (VERIFIED-S as calibration data):
    // `ip_eff_iga_vvl_cam_h` @0x8085A76C (= `ip_eff_iga`, flaps closed) and `ip_eff_iga_vvl_cam_l` @0x8085A8EC (flaps open)
    // at −10/−20/−30/−40 °CRK (linear between the 3 ° breakpoints).
    let target = |closed: bool| if closed { [0.9551, 0.8232, 0.6040, 0.3085] } else { [0.9590, 0.8244, 0.6476, 0.3733] };
    println!("csv,engine,rpm,air_mg,cam_in,cam_ex,flaps,map_kpa,residual,mbt,ca50,ign_ca10,ca10_90,u_prime,s_l,tq_mbt,r10,r20,r30,r40,pmep_nm,fric_nm,d_mbt,e10,e20,e30,e40");
    let mut err = Vec::new();
    for [rpm, mg, cin, cex, fl, mbt_fw] in pts {
        let mut p = grid_plant(&engine, rpm);
        let mut op = Operator::wot(15.0, 1.0, 100.0);
        op.cams = [cin, cex];
        op.flaps_closed = fl > 0.5;
        op.throttle_duty = -0.3;
        op.rail_target_mpa = 10.0;
        if let Ok(r) = std::env::var("RON") {
            p.fuel_ron = r.parse().unwrap();
        }
        if mg > 500.0 {
            // boosted points: the wastegate makes the charge, the throttle trims it
            op.put_target_kpa = 100.0 * mg / 480.0;
            op.throttle_duty = 0.6;
        }
        hold_air(&mut p, &mut op, mg, 8.0);
        let mut rows: Vec<(f64, f64, PlantTelemetry)> = Vec::new();
        let mut burn: Vec<[f64; 3]> = Vec::new();
        // sparks from 48 ° before to 16 ° after the reference angle when it is given (MBT − 40 must be measured, not
        // extrapolated), else −16…60 °BTDC
        let (s0, n) = if mbt_fw.is_finite() { (4.0 * ((mbt_fw - 48.0) / 4.0).round(), 17) } else { (-16.0, 20) };
        for k in 0..n {
            let spark = s0 + 4.0 * k as f64;
            op.spark = spark;
            run(&mut p, &op, 0.8);
            // torque per trapped air (the fixed throttle lets the trapped mass drift slightly with exhaust temperature)
            let (mut last, mut ang, mut n) = ([0.0f64; 4], [0.0f64; 3], 0.0f64);
            let ind = mean(&mut p, &op, 0.4, |t| t.torque_indicated_nm / t.air_mg_stroke.max(1.0)) * mg;
            // cycle-mean burn angles of all cylinders over a further 0.2 s
            for _ in 0..40 {
                run(&mut p, &op, 0.005);
                for (c, cy) in p.engine.cyl.iter().enumerate() {
                    let s = cy.last;
                    if s.t_end_s != last[c] && s.burned && !s.misfire && s.ca90_deg != 0.0 {
                        last[c] = s.t_end_s;
                        ang = [ang[0] + s.ca10_deg, ang[1] + s.ca50_deg, ang[2] + s.ca90_deg];
                        n += 1.0;
                    }
                }
            }
            burn.push(ang.map(|a| a / n.max(1.0)));
            rows.push((spark, ind, p.telemetry()));
        }
        let (ib, _) = rows.iter().enumerate().fold((0, f64::MIN), |b, (i, r)| if r.1 > b.1 { (i, r.1) } else { b });
        let ib = ib.clamp(1, rows.len() - 2);
        let (x1, x2) = (rows[ib].0, rows[ib + 1].0);
        let (y0, y1, y2) = (rows[ib - 1].1, rows[ib].1, rows[ib + 1].1);
        let den = y0 - 2.0 * y1 + y2;
        let mbt = if den < 0.0 { x1 + 0.5 * (x2 - x1) * (y0 - y2) / den } else { x1 };
        let tq_max = y1 - 0.125 * (y0 - y2).powi(2) / den.min(-1e-9);
        let at = |d: f64| {
            let s = mbt - d;
            let i = rows.iter().position(|r| r.0 >= s).unwrap_or(rows.len() - 1).max(1);
            let (a, b) = (&rows[i - 1], &rows[i]);
            let y = a.1 + (b.1 - a.1) * (s - a.0) / (b.0 - a.0);
            y / tq_max
        };
        let t = &rows[ib].2;
        let c = &t.combustion;
        let b = burn[ib];
        let r = [at(10.0), at(20.0), at(30.0), at(40.0)];
        let tg = target(fl > 0.5);
        let e: [f64; 4] = std::array::from_fn(|k| r[k] - tg[k]);
        println!(
            "csv,{engine},{rpm:.0},{:.1},{cin:.1},{cex:.1},{fl:.0},{:.1},{:.3},{mbt:.1},{:.1},{:.1},{:.1},{:.2},{:.3},{tq_max:.1},{:.3},{:.3},{:.3},{:.3},{:.1},{:.1},{:.1},{:.3},{:.3},{:.3},{:.3}",
            t.air_mg_stroke, t.map_kpa, t.residual_frac[0], b[1], b[0] + rows[ib].0, b[2] - b[0],
            c.u_prime_ms[0], c.s_l_ms[0], r[0], r[1], r[2], r[3], t.torque_pumping_nm, t.torque_friction_nm, mbt - mbt_fw, e[0], e[1], e[2], e[3]
        );
        err.push((mbt - mbt_fw, e));
        if std::env::var("VERB").is_ok() {
            for ((s, ind, t), b) in rows.iter().zip(&burn) {
                println!("   spark {s:5.1}: ind {ind:6.2} ca10 {:5.1} ca50 {:5.1} ca90 {:5.1} pmax {:5.1} kn {:.2} misf {:?} air {:.1}", b[0], b[1], b[2], t.cyl_peak_bar[0], t.knock_amp_bar[0], t.combustion.misfire_events, t.air_mg_stroke);
            }
        }
    }
    // summary: rms MBT error (points with a firmware reference) and rms / max efficiency error
    let dm: Vec<f64> = err.iter().map(|e| e.0).filter(|x| x.is_finite()).collect();
    let ee: Vec<f64> = err.iter().flat_map(|e| e.1).collect();
    let rms = |v: &[f64]| (v.iter().map(|x| x * x).sum::<f64>() / v.len().max(1) as f64).sqrt();
    println!("summary,mbt_rms {:.2},mbt_mean {:.2},eff_rms {:.3},eff_max {:.3}", rms(&dm), dm.iter().sum::<f64>() / dm.len().max(1) as f64, rms(&ee), ee.iter().fold(0.0f64, |a, x| a.max(x.abs())));
}

/// Exploration: motored turbulence intensity over the cycle (cylinder 1), u'/S̄p at selected angles.
#[test]
#[ignore]
fn explore_turbulence_trace() {
    for &(rpm, flaps) in &[(800.0, true), (2000.0, true), (2000.0, false), (4000.0, false)] {
        let mut p = grid_plant("cjxc", rpm);
        let mut op = Operator::wot(0.0, 1.0, 100.0);
        op.fuel = false;
        op.ignition = false;
        op.cams = [15.0, 0.0];
        op.flaps_closed = flaps;
        op.throttle_duty = if rpm < 1000.0 { -0.5 } else { 0.2 };
        run(&mut p, &op, 1.0);
        let sp = 2.0 * p.engine.kin.stroke * rpm / 60.0;
        let w = rpm * PI / 30.0;
        let mut s = String::new();
        let mut marks: Vec<f64> = (0..12).map(|k| -270.0 + 30.0 * k as f64).collect();
        let mut prev = crate::cylinder::atdc(p.engine.crank_deg);
        for _ in 0..(0.2 / DT_SUB) as usize {
            let a = op.command(&p);
            p.step(DT_SUB, &a);
            let ang = crate::cylinder::atdc(p.engine.crank_deg);
            if let Some(i) = marks.iter().position(|&m| prev < m && ang >= m) {
                let c = &p.engine.cyl[0];
                let h = (c.v - 0.0) / p.engine.kin.area;
                let b = p.engine.kin.bore;
                let tr = c.turb.l_tumble / (c.mass() * (0.75 * b * b + h * h) / 12.0) / w;
                s += &format!(" {ang:4.0}:{:.2}/{tr:.2}", c.turb.u_prime(c.mass()) / sp);
                marks.remove(i);
            }
            prev = ang;
        }
        println!("{rpm:5.0} flaps {flaps}: Sp {sp:.2} m/s  u'/Sp / TR:{s}");
    }
}

/// Exploration / calibration evidence: knock-limited spark (first-mode amplitude ≤ 0.5 bar) on the CJSA dyno at the
/// stock-like full-load points for knock-delay scales `SCALES="a,b,…"`, to compare with SC8O40's base ignition map.
#[test]
#[ignore]
fn explore_knock_limit() {
    let scales: Vec<f64> = std::env::var("SCALES").ok().map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![2.0]);
    let rpms: Vec<f64> = std::env::var("RPMS").ok().map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![1500.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0]);
    for &sc in &scales {
        for &rpm in &rpms {
            let mut p = dyno(rpm);
            p.engine_profile.mech.combustion.knock_tau_scale = sc;
            // `STOCK_AIR=1`: the operator holds the stock CJSA full-load air charge instead of its boost table
            let air = std::env::var("STOCK_AIR").is_ok().then(|| stock_air_mg(rpm));
            let (op, spark) = knock_limited_at(&mut p, rpm, air);
            let tq = mean(&mut p, &op, 0.5, |t| t.torque_brake_nm);
            let t = p.telemetry();
            println!("scale {sc:.2} {rpm:5.0} rpm: KL spark {spark:5.1}  {tq:5.1} Nm  air {:5.0} mg  CA50 {:4.1}  pmax {:5.1}  Tu {:4.0} °C  land {:3.0} °C  groove {:3.0} °C  crown {:3.0} °C  plug {:3.0} °C  exv {:3.0} °C",
                t.air_mg_stroke, t.ca50_deg[0], t.cyl_peak_bar[0], t.combustion.t_end_gas_c[0], t.combustion.piston_land_c[0], t.combustion.piston_groove_c[0], t.piston_crown_c[0], t.combustion.plug_tip_c[0], t.exhaust_valve_c[0]);
        }
    }
}

/// Exploration: knock intensity vs spark advance on boost (2000 rpm), `PUT` kPa.
#[test]
#[ignore]
fn explore_knock_vs_spark() {
    let put: f64 = std::env::var("PUT").ok().and_then(|v| v.parse().ok()).unwrap_or(185.0);
    let rpm: f64 = std::env::var("RPM").ok().and_then(|v| v.parse().ok()).unwrap_or(2000.0);
    let s0: f64 = std::env::var("SPARK0").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
    for k in 0..10 {
        let spark = s0 + 4.0 * k as f64;
        let mut p = dyno(rpm);
        let mut op = stock_wot(rpm, spark);
        op.lambda = 0.9;
        op.put_target_kpa = put;
        run(&mut p, &op, std::env::var("RUN_S").ok().and_then(|v| v.parse().ok()).unwrap_or(4.0));
        let kn = mean(&mut p, &op, 0.5, |t| t.knock_amp_bar.iter().cloned().fold(0.0, f64::max));
        let t = p.telemetry();
        let c = &t.combustion;
        let l = p.engine.cyl.map(|c| c.last);
        println!(
            "spark {spark:4.0}: knock {kn:5.2} bar  x_u {:.2?}  pmax {:.0?}  p_loc {:.0?}  CA10 {:.1} CA50 {:.1} CA90 {:.1}  Tu {:.0} °C  tq {:.0}  air {:.0} mg  onset {:.0?}  ign {:.0?}  preign {:?}",
            c.knock_unburned, t.cyl_peak_bar, c.p_local_bar, c.ca10_deg[0], t.ca50_deg[0], c.ca90_deg[0], c.t_end_gas_c[0], t.torque_brake_nm, t.air_mg_stroke,
            l.map(|s| s.knock_onset_deg), l.map(|s| s.ign_deg), l.map(|s| s.preignition)
        );
    }
}

/// Exploration: trapped air vs MAP at the firmware's warm-idle cam set-points (φ_in 15, φ_ex 0, flaps closed), fired at a
/// fixed spark, to compare with the firmware's speed-density model MAF_CYL = SLOP·MAP − OFS. `ENGINE`, `RPMS`.
#[test]
#[ignore]
fn explore_ve() {
    let engine = std::env::var("ENGINE").unwrap_or("cjxc".into());
    let list = |k: &str, d: Vec<f64>| std::env::var(k).ok().map(|v| v.split(',').map(|x| x.trim().parse().unwrap()).collect()).unwrap_or(d);
    let rpms = list("RPMS", vec![760.0]);
    let mgs = list("MGS", vec![90.0, 110.0, 140.0, 180.0, 240.0]);
    let cams = list("CAMS", vec![15.0, 0.0]);
    let spark: f64 = std::env::var("SPARK").ok().and_then(|v| v.parse().ok()).unwrap_or(10.0);
    for &rpm in &rpms {
        for ci in cams.chunks(2) {
            let mut pts = Vec::new();
            for &mg in &mgs {
                let mut p = grid_plant(&engine, rpm);
                let mut op = Operator::wot(spark, 1.0, 100.0);
                op.cams = [ci[0], ci[1]];
                op.flaps_closed = std::env::var("FLAPS").map_or(true, |v| v == "1");
                op.mpi = std::env::var("MPI").is_ok_and(|v| v == "1");
                op.throttle_duty = -0.3;
                op.rail_target_mpa = 10.0;
                if let Some(put) = std::env::var("PUT").ok().and_then(|v| v.parse().ok()) {
                    (op.put_target_kpa, op.throttle_duty, op.rail_target_mpa) = (put, 0.6, 18.0);
                }
                hold_air(&mut p, &mut op, mg, 8.0);
                let t = p.telemetry();
                let ideal = t.map_kpa * 1e3 * p.engine.kin.v_disp / (287.06 * (t.iat_c + 273.15)) * 1e6;
                println!(
                    "ve,{engine},{rpm:.0},cams {:.1}/{:.1},map {:.1},air {:.1},ideal {ideal:.1},ve {:.3},res {:.3},p_exh {:.1},egt {:.0},iat {:.1},tq {:.1},bb {:.2} mg/stk",
                    ci[0], ci[1], t.map_kpa, t.air_mg_stroke, t.air_mg_stroke / ideal, t.residual_frac[0], t.exhaust_manifold_kpa, t.egt_c, t.iat_c, t.torque_indicated_nm, p.engine.flows.blowby_kg_s / (rpm / 30.0) * 1e6
                );
                pts.push((t.map_kpa * 10.0, t.air_mg_stroke));
            }
            // least-squares line MAF = SLOP·MAP[hPa] − OFS, as the firmware's model
            let n = pts.len() as f64;
            let (sx, sy) = (pts.iter().map(|q| q.0).sum::<f64>(), pts.iter().map(|q| q.1).sum::<f64>());
            let sxx = pts.iter().map(|q| q.0 * q.0).sum::<f64>();
            let sxy = pts.iter().map(|q| q.0 * q.1).sum::<f64>();
            let slop = (n * sxy - sx * sy) / (n * sxx - sx * sx);
            let ofs = (slop * sx - sy) / n;
            println!("veline,{engine},{rpm:.0},{:.1},{:.1},slop {slop:.4} mg/hPa,ofs {ofs:.1} mg,p0 {:.0} hPa", ci[0], ci[1], ofs / slop);
        }
    }
}

/// Exploration: one gas-exchange of cylinder 1 at the firmware's warm-idle point (259J run: CAM_PHA 15.7 / 3.9 °CRK,
/// port flaps deactivated), `ENGINE`, `MG` (trapped air held by the throttle), `CAMS="in,ex"`, `FLAPS`.
#[test]
#[ignore]
fn explore_intake_stroke() {
    let engine = std::env::var("ENGINE").unwrap_or("cjxc".into());
    let mg: f64 = std::env::var("MG").ok().and_then(|v| v.parse().ok()).unwrap_or(120.0);
    let cams: Vec<f64> = std::env::var("CAMS").ok().map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![15.7, 3.9]);
    let rpm: f64 = std::env::var("RPM").ok().and_then(|v| v.parse().ok()).unwrap_or(760.0);
    let spark: f64 = std::env::var("SPARK").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
    let mut p = grid_plant(&engine, rpm);
    let mut op = Operator::wot(spark, 1.0, 100.0);
    op.cams = [cams[0], cams[1]];
    op.flaps_closed = std::env::var("FLAPS").map_or(true, |v| v == "1");
    op.mpi = std::env::var("MPI").is_ok_and(|v| v == "1");
    op.throttle_duty = -0.3;
    op.rail_target_mpa = 10.0;
    hold_air(&mut p, &mut op, mg, 8.0);
    let t = p.telemetry();
    println!("map {:.1} kPa air {:.1} mg iat {:.1} °C residual {:.3} exh {:.1} kPa egt {:.0}", t.map_kpa, t.air_mg_stroke, t.iat_c, t.residual_frac[0], t.exhaust_manifold_kpa, t.egt_c);
    // wait for the start of a cycle of cylinder 1, then sample gas-exchange TDC (360) .. 640 °CA every 10 °
    while !(p.engine.crank_deg > 300.0 && p.engine.crank_deg < 340.0) {
        let a = op.command(&p);
        p.step(DT_SUB, &a);
    }
    let mut prev = p.engine.crank_deg;
    let mut next = 340.0;
    let (mut m_in, mut m_back) = (0.0, 0.0);
    while next <= 650.0 {
        let m0 = p.engine.cyl[0].m;
        let a = op.command(&p);
        p.step(DT_SUB, &a);
        let phi = p.engine.crank_deg;
        let c = &p.engine.cyl[0];
        let dm = c.m[0] - m0[0];
        if dm > 0.0 { m_in += dm } else { m_back -= dm }
        if crate::cylinder::crossed(prev, phi, next) {
            let v = &p.engine.vt.valve[0];
            println!(
                "{phi:6.1} lift in {:5.2} ex {:5.2} mm | p_cyl {:6.2} p_im {:6.2} p_em {:6.2} kPa | T {:5.0} K | air {:6.1} prod {:6.1} mg | V {:5.1} cm³",
                v[0].x * 1e3, v[1].x * 1e3, c.p / 1e3, p.engine.p_intake_pa / 1e3, p.engine.p_exhaust_pa / 1e3, c.t, c.m[0] * 1e6, c.m[2] * 1e6, c.v * 1e6
            );
            next += 10.0;
        }
        prev = phi;
    }
    println!("air in {:.1} mg, out (back to port / exhaust) {:.1} mg", m_in * 1e6, m_back * 1e6);
    println!("cycle: {:?}", p.engine.cyl[0].last);
}

/// Cycle statistics of all four cylinders over `cycles` engine cycles at a held operating point: CoV of net and gross
/// IMEP (%), mean / σ of CA50 (°), knocking-cycle share, misfires, and the first-mode knock amplitudes (bar).
struct CycleSample {
    cov_net: f64,
    cov_gross: f64,
    imep_bar: f64,
    ca50: (f64, f64),
    knock_share: f64,
    misfires: usize,
    knock_bar: Vec<f64>,
}

fn sample_cycles(p: &mut VirtualPlant, op: &Operator, cycles: usize) -> CycleSample {
    let mut last = [0.0f64; 4];
    let (mut net, mut gross, mut ca50, mut knock) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut misfires = 0;
    while net.len() < 4 * cycles {
        run(p, op, 0.01);
        for (c, cy) in p.engine.cyl.iter().enumerate() {
            let s = cy.last;
            if s.t_end_s != last[c] {
                last[c] = s.t_end_s;
                net.push(s.imep_pa / 1e5);
                gross.push(s.gross_imep_pa / 1e5);
                misfires += s.misfire as usize;
                if s.burned && !s.misfire {
                    ca50.push(s.ca50_deg);
                }
                knock.push(s.knock_amp_pa / 1e5);
            }
        }
    }
    let ms = |v: &[f64]| {
        let m = v.iter().sum::<f64>() / v.len().max(1) as f64;
        (m, (v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / v.len().max(2) as f64).sqrt())
    };
    let ((mn, sn), (mg, sg)) = (ms(&net), ms(&gross));
    CycleSample {
        cov_net: 100.0 * sn / mn.abs().max(1e-9),
        cov_gross: 100.0 * sg / mg.abs().max(1e-9),
        imep_bar: mn,
        ca50: ms(&ca50),
        knock_share: knock.iter().filter(|&&k| k > 1.0).count() as f64 / knock.len() as f64,
        misfires,
        knock_bar: knock,
    }
}

/// Exploration: cycle-to-cycle variation at `PTS = "rpm,air_mg,cam_in,cam_ex,flaps,spark,lambda;…"` (MPI below 2600 rpm,
/// as the firmware runs warm part load), `ENGINE`, `CYCLES`.
#[test]
#[ignore]
fn explore_ccv() {
    let engine = std::env::var("ENGINE").unwrap_or("cjxc".into());
    let cycles: usize = std::env::var("CYCLES").ok().and_then(|v| v.parse().ok()).unwrap_or(100);
    let pts: Vec<[f64; 7]> = std::env::var("PTS")
        .ok()
        .map(|v| v.split(';').map(|c| { let x: Vec<f64> = c.split(',').map(|y| y.trim().parse().unwrap()).collect(); [x[0], x[1], x[2], x[3], x[4], x[5], x[6]] }).collect())
        .unwrap_or(vec![[760.0, 120.0, 15.0, 4.0, 1.0, 3.0, 1.0], [2000.0, 300.0, 15.0, 0.0, 0.0, 19.0, 1.0], [2000.0, 300.0, 15.0, 0.0, 0.0, 22.0, 1.3]]);
    for [rpm, mg, ci, cx, fl, spark, lam] in pts {
        let mut p = grid_plant(&engine, rpm);
        let mut op = Operator::wot(spark, lam, 100.0);
        op.cams = [ci, cx];
        op.flaps_closed = fl > 0.5;
        op.mpi = rpm < 2600.0;
        op.throttle_duty = -0.3;
        op.rail_target_mpa = 10.0;
        hold_air(&mut p, &mut op, mg, 6.0);
        let s = sample_cycles(&mut p, &op, cycles);
        let t = p.telemetry();
        println!(
            "ccv,{engine},{rpm:.0} rpm {:.0} mg λ {lam:.2} spark {spark:.0}: IMEP {:.2} bar CoV net {:.2} % gross {:.2} % | CA50 {:.1} ± {:.2} ° | knock {:.0} % misfires {} | residual {:.3} MAP {:.1} kPa",
            t.air_mg_stroke, s.imep_bar, s.cov_net, s.cov_gross, s.ca50.0, s.ca50.1, 100.0 * s.knock_share, s.misfires, t.residual_frac[0], t.map_kpa
        );
    }
}

/// Exploration: knock with cycle-to-cycle variation on the CJSA dyno, stock-like WOT at `RPM` (default 2000), spark
/// `ADV` degrees beyond the stock knock-limited spark for each of `ADVS="a,b,…"`, fuel `RON`: share of knocking cycles
/// (first mode > 1 bar), amplitude median / 90th percentile / max, CA50 mean ± σ.
#[test]
#[ignore]
fn explore_knock_ccv() {
    let rpm: f64 = std::env::var("RPM").ok().and_then(|v| v.parse().ok()).unwrap_or(2000.0);
    let advs: Vec<f64> = std::env::var("ADVS").ok().map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![-4.0, 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0]);
    for adv in advs {
        let mut p = dyno(rpm);
        if let Ok(r) = std::env::var("RON") {
            p.fuel_ron = r.parse().unwrap();
        }
        let spark = stock_spark(rpm) + adv;
        let op = stock_wot(rpm, spark);
        run(&mut p, &op, 3.0);
        let s = sample_cycles(&mut p, &op, 50);
        let mut k = s.knock_bar.clone();
        k.sort_by(f64::total_cmp);
        let q = |f: f64| k[((k.len() - 1) as f64 * f) as usize];
        println!(
            "knock,{rpm:.0} rpm spark {spark:5.1} (KL {adv:+.0}): knocking {:4.0} % | amplitude median {:5.2} p90 {:5.2} max {:6.2} bar | CA50 {:5.1} ± {:.2} | IMEP {:.2} bar CoV {:.2} %",
            100.0 * s.knock_share, q(0.5), q(0.9), q(1.0), s.ca50.0, s.ca50.1, s.imep_bar, s.cov_net
        );
    }
}

/// Exploration: port-injected (MPI) idle on the bench — fuel from the port film model, spark `SPARK`.
#[test]
#[ignore]
fn explore_mpi_idle() {
    let spark: f64 = std::env::var("SPARK").ok().and_then(|v| v.parse().ok()).unwrap_or(10.0);
    let mut p = grid_plant("cjxc", 760.0);
    let mut op = Operator::wot(spark, 1.0, 100.0);
    op.cams = [15.0, 0.0];
    op.flaps_closed = true;
    op.throttle_duty = -0.5;
    op.rail_target_mpa = 10.0;
    hold_air(&mut p, &mut op, 120.0, 4.0);
    for k in 0..30 {
        let mut a = op.command(&p);
        let air = p.engine.flows.air_per_cyl_mg.max(50.0);
        let mg = air / 14.7;
        let w = mg * 0.434 + 0.7;
        for c in 0..4 {
            a.ecu.injection[c] = [None; 3];
            a.ecu.port_injection[c] = [Some(InjPulse { soi_btdc_deg: 370.0, width_ms: w }), None, None];
        }
        p.step(0.05, &a);
        let t = p.telemetry();
        if k % 3 == 0 {
            println!("t {:.2} firing {:?} λ {:.2?} fuel {:.1?} air {:.1} film {:.1} mg misf {:?} ign {:.1?} ca50 {:.1?}", t.sim_time_s, t.firing, t.lambda_cyl, t.fuel_mg_stroke, t.air_mg_stroke,
                p.engine.port.iter().map(|x| x.film_kg).sum::<f64>() * 1e6, t.combustion.misfire_events, t.combustion.ign_deg, t.ca50_deg);
        }
    }
}

/// Spark-plug nose temperature by heat range (NGK scale 6–9) on the CJSA dyno: at rated power (5500 rpm WOT) the OE heat
/// range 7 runs inside the self-cleaning/pre-ignition window 500–900 °C, each colder step is ≈ 50–120 K cooler (NGK / Bosch
/// technical information: ≈ 70–100 K per step), and at warm idle every plug runs below the 450–500 °C self-cleaning limit.
#[test]
fn spark_plug_tip_temperature_follows_the_heat_range() {
    let tip = |hr: f64, rpm: f64, wot: bool| {
        let mut p = dyno(rpm);
        p.engine_profile.mech.spark_plug.heat_range = hr;
        let mut op = if wot { stock_wot(rpm, stock_spark(rpm)) } else { Operator::wot(10.0, 1.0, 100.0) };
        if !wot {
            op.throttle_duty = -0.6;
            op.cams = [15.0, 0.0];
            op.flaps_closed = true;
            p.motoring_rpm = Some(rpm);
        }
        run(&mut p, &op, 15.0);
        p.telemetry().combustion.plug_tip_c[0]
    };
    let wot: Vec<f64> = [6.0, 7.0, 8.0, 9.0].iter().map(|&hr| tip(hr, 5500.0, true)).collect();
    let idle = tip(7.0, 800.0, false);
    println!("plug tip °C at 5500 rpm WOT, heat range 6..9: {wot:.0?}; idle (HR7): {idle:.0}");
    assert!((500.0..900.0).contains(&wot[1]), "HR7 at rated power {:.0} °C", wot[1]);
    assert!(wot.windows(2).all(|w| (50.0..120.0).contains(&(w[0] - w[1]))), "per heat-range step {wot:.0?}");
    assert!(idle < 500.0, "idle tip {idle:.0} °C");
}

/// LSPI: with the droplet-release probability raised (test knob, 3 % per cycle) the released oil/fuel droplets ignite
/// before the spark at low-speed full load and the early flame drives the end gas into heavy knock (local peaks well
/// above the normal maximum), while at part load the same droplets do not ignite before the spark (pressure and time
/// too low) — the event rate comes from the state, not from a script.
#[test]
fn lspi_needs_low_speed_high_load_and_ends_in_super_knock() {
    let run_case = |rpm: f64, wot: bool| {
        let mut p = dyno(rpm);
        p.engine_profile.mech.combustion.lspi_rate = 0.03;
        let mut op = if wot { stock_wot(rpm, stock_spark(rpm)) } else { Operator::wot(25.0, 1.0, 100.0) };
        if !wot {
            op.throttle_duty = -0.2;
            p.motoring_rpm = Some(rpm);
        }
        let mut p_loc_max = 0.0f64;
        for _ in 0..40 {
            run(&mut p, &op, 0.25);
            p.thermal.engine_c = p.thermal.engine_c.min(95.0);
            let t = p.telemetry();
            p_loc_max = p_loc_max.max(t.combustion.p_local_bar.iter().cloned().fold(0.0, f64::max));
        }
        let t = p.telemetry();
        (t.combustion.lspi_events.iter().sum::<u32>(), p_loc_max, t.cyl_peak_bar.iter().cloned().fold(0.0, f64::max))
    };
    let (n_low, ploc_low, pk_low) = run_case(1500.0, true);
    let (n_part, _, _) = run_case(2000.0, false);
    println!("LSPI 1500 rpm WOT: {n_low} events, peak local {ploc_low:.0} bar (mean peak {pk_low:.0}); 2000 rpm part load: {n_part} events");
    assert!(n_low >= 3, "LSPI at low-speed full load: {n_low}");
    assert!(ploc_low > 1.3 * pk_low, "pre-ignition drives the end gas into heavy knock: {ploc_low:.0} vs {pk_low:.0} bar");
    assert_eq!(n_part, 0, "no LSPI at part load");
}

/// Cold-start mixture against the firmware's own start fuel — the anchor of the wall-film model
/// (docs/SIMOS18_COMBUSTION.md §15). SC8O40 (264H) injects for its high-pressure start ST_H_3_S `ip_mff_st_h_3_s`
/// @0x8083ABD6 at ≤ 450 rpm 36.7 / 48.7 / 80.4 / 115.6 mg at TCO 20 / 5 / −10 / −20 °C, and 26.7 mg for a hot start
/// (`ip_mff_st_h_2_s` @0x8083AB46, 90 °C) — 1.4…4.3× the hot fuel into the same cylinder (VERIFIED-S): the calibration
/// counts on the cold walls keeping the difference. The plant, cranked at 200 rpm (motored) cold-soaked at each
/// temperature, three compression-stroke pulses where the firmware puts them at 5 °C (96 / 69 / 41 °BTDC, 0.4 / 0.4 /
/// 0.2 of the fuel, VERIFIED-T in `engine_start_scenario`), 14 MPa, spark at TDC: the vapour mixture at the firing TDCs of
/// cycles 3–6 has to be burnable at every temperature and stay near the hot start's (the firmware's fuel ratios). Plant
/// (2026-11-03): hot 0.92, 20 / 5 / −10 / −20 °C 0.69 / 0.70 / 0.81 / 0.93.
#[test]
fn cold_start_mixture_follows_the_firmware_start_fuel() {
    let phi_at = |t_c: f64, fuel_mg: f64| {
        let prof = EngineProfile::from_slug("cjsa").expect("cjsa");
        let mut p = VirtualPlant::new(prof, DrivetrainProfile::dq200_fwd(), Ambient { temp_c: t_c.min(25.0), altitude_m: 0.0, humidity_pct: 40.0 });
        p.thermal = ThermalState::uniform(t_c);
        if t_c > 60.0 {
            // hot restart: pistons, valves and plugs still hot from running
            p.engine.cyl.iter_mut().for_each(|c| c.set_component_temps(400.0, 600.0));
        }
        let rpm = 200.0;
        p.engine.set_speed(rpm * PI / 30.0);
        p.motoring_rpm = Some(rpm);
        p.engine.rail_pa = 14e6;
        p.engine.lp_fuel_pa = 0.5e6;
        p.driver.ignition_on = true;
        let mut op = Operator::wot(0.0, 1.0, 0.0);
        (op.throttle_duty, op.cams, op.rail_target_mpa, op.fuel) = (0.0, [0.0, 0.0], 14.0, false);
        run(&mut p, &op, 0.3);
        let (inj, dead) = (p.engine_profile.injector_flow_cc_s, p.engine_profile.injector_dead_time_ms);
        let (mut prev_v, mut falling, mut tdcs, mut phis) = ([0.0f64; 4], [false; 4], 0, Vec::new());
        for _ in 0..(6.5 * 120.0 / rpm / 5e-4) as usize {
            let mut a = op.command(&p);
            let e = &p.engine;
            let mg_ms = inj * 1e-6 * ((e.rail_pa - e.p_intake_pa).max(1e5) / 1e7).sqrt() * engine::FUEL_DENSITY_KG_M3 * 1e3;
            for inj_slot in &mut a.ecu.injection {
                *inj_slot = [(96.0, 0.4), (69.0, 0.4), (41.0, 0.2)].map(|(soi, share)| Some(InjPulse { soi_btdc_deg: soi, width_ms: fuel_mg * share / mg_ms + dead }));
            }
            p.step(5e-4, &a);
            for c in 0..4 {
                let cy = &p.engine.cyl[c];
                if falling[c] && cy.v > prev_v[c] && cy.p > 3e5 {
                    tdcs += 1;
                    if tdcs > 8 {
                        phis.push(engine::AFR_STOICH * cy.m[1] / cy.m[0]);
                    }
                }
                falling[c] = cy.v < prev_v[c];
                prev_v[c] = cy.v;
            }
        }
        let films: Vec<f64> = p.engine.cyl.iter().map(|c| (c.film_kg + c.liner_film_kg) * 1e6).collect();
        let phi = phis.iter().sum::<f64>() / phis.len().max(1) as f64;
        println!("{t_c:5.0} °C {fuel_mg:6.1} mg: vapour φ at TDC {phi:.2} ({} samples), wall films {films:.1?} mg, oil {:.1} mg", phis.len(), p.engine.oil_fuel_kg * 1e6);
        phi
    };
    let hot = phi_at(90.0, 26.7);
    let cold: Vec<(f64, f64)> = [(20.0, 36.7), (5.0, 48.7), (-10.0, 80.4), (-20.0, 115.6)].map(|(t, mg)| (t, phi_at(t, mg))).to_vec();
    // Burnable: the plant's kernel dies below φ ≈ 0.6 and above ≈ 1.7 (S_L < `S_L_MIN`). A single-component film (the
    // former model) gave 1.06 / 1.26 / 1.72 / 2.14 (hot 0.99) here: the firmware's cold fuel nearly all as vapour.
    assert!((0.85..1.15).contains(&hot), "hot start φ {hot:.2}");
    for (t, phi) in cold {
        assert!((0.6..1.4).contains(&phi), "{t} °C: vapour φ {phi:.2} at the spark");
        assert!((0.6..1.5).contains(&(phi / hot)), "{t} °C: {:.2}× the hot start's vapour φ", phi / hot);
    }
}

/// SC8O40 264H loss model (VERIFIED-S, DAMOS `SC8O40.json`): `ip_tqfr_vvl_cam_port_h[0][1][1]` @0x80263C7E — friction +
/// gas exchange of the warm engine by air (rows, mg/stk) and speed (columns, rpm), Nm (sign dropped); `ip_tqfr_add`
/// @0x802635B6 — added by oil (rows) × coolant (columns) temperature, °C.
const FW_N: [f64; 10] = [600.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0, 4000.0, 4500.0, 5000.0];
const FW_MG: [f64; 8] = [50.0, 100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 750.0];
const FW_TQFR: [[f64; 10]; 8] = [
    [15.0, 16.0, 16.0, 18.0, 19.0, 21.0, 22.0, 24.0, 26.0, 27.0],
    [15.0, 16.0, 16.0, 18.0, 19.0, 21.0, 22.0, 24.0, 26.0, 27.0],
    [13.0, 13.0, 13.0, 14.0, 15.0, 17.0, 19.0, 20.0, 23.0, 25.0],
    [10.0, 10.0, 11.0, 12.0, 14.0, 16.0, 17.0, 19.0, 22.0, 26.0],
    [9.0, 10.0, 10.0, 12.0, 12.0, 16.0, 17.0, 19.0, 23.0, 27.0],
    [9.0, 10.0, 9.0, 11.0, 13.0, 15.0, 18.0, 22.0, 26.0, 28.0],
    [9.0, 10.0, 10.0, 11.0, 13.0, 18.0, 20.0, 24.0, 28.0, 33.0],
    [9.0, 10.0, 11.0, 11.0, 13.0, 20.0, 24.0, 27.0, 32.0, 38.0],
];
const FW_T_CO: [f64; 8] = [-30.0, -20.25, -15.0, -9.75, 0.0, 20.25, 40.5, 60.0];
const FW_T_OIL: [f64; 8] = [-30.0, -20.0, -15.0, -10.0, 0.0, 20.0, 40.0, 60.0];
const FW_ADD: [[f64; 8]; 8] = [
    [50.812, 44.969, 41.875, 39.438, 36.0, 28.5, 21.969, 18.719],
    [45.406, 39.406, 36.625, 33.688, 29.5, 23.688, 17.688, 14.5],
    [44.125, 36.781, 33.219, 30.375, 25.594, 19.0, 14.281, 11.781],
    [41.656, 34.062, 30.562, 27.219, 22.156, 15.188, 10.656, 8.062],
    [36.781, 29.906, 26.219, 22.688, 17.812, 11.375, 7.5, 4.594],
    [27.156, 20.969, 17.781, 15.375, 12.062, 7.406, 4.656, 2.688],
    [17.156, 12.344, 9.875, 8.062, 5.812, 3.469, 1.719, 0.844],
    [10.25, 6.531, 5.094, 4.281, 3.094, 1.719, 0.594, 0.0],
];

/// Bilinear table lookup, clamped at the axis ends: `t[row][column]` over `ry` × `cx`.
fn bilin<const R: usize, const C: usize>(t: &[[f64; C]; R], ry: &[f64], cx: &[f64], y: f64, x: f64) -> f64 {
    let idx = |ax: &[f64], x: f64| {
        let x = x.clamp(ax[0], ax[ax.len() - 1]);
        let i = ax.windows(2).position(|w| x <= w[1]).unwrap_or(ax.len() - 2);
        (i, (x - ax[i]) / (ax[i + 1] - ax[i]))
    };
    let ((i, u), (j, v)) = (idx(ry, y), idx(cx, x));
    let r = |i: usize| t[i][j] * (1.0 - v) + t[i][j + 1] * v;
    r(i) * (1.0 - u) + r(i + 1) * u
}

/// The firmware's engine losses (friction + gas exchange + temperature correction) at `rpm`, `mg` and coolant = oil `t_c`.
fn firmware_loss_nm(rpm: f64, mg: f64, t_c: f64) -> f64 {
    bilin(&FW_TQFR, &FW_MG, &FW_N, mg, rpm) + bilin(&FW_ADD, &FW_T_OIL, &FW_T_CO, t_c, t_c)
}

/// One loss point on the dyno: held air, λ 1, the given spark; means over 1 s of [air, indicated, friction, pumping,
/// valvetrain, oil pump, HPFP, alternator, brake, absorber, crank bearings].
fn loss_point(rpm: f64, mg: f64, spark: f64, t_c: f64) -> [f64; 11] {
    let mut p = dyno(rpm);
    // absorbing and motoring (a cold engine at idle air makes less than its friction)
    p.bench = BenchMode::AcDyno { target_rpm: rpm };
    p.soak(t_c);
    let mut op = Operator::wot(spark, 1.0, 100.0);
    op.throttle_duty = 0.0;
    op.cams = [0.0, 0.0];
    op.rail_target_mpa = 10.0;
    // coolant and oil held at `t_c` (an engine at that temperature); bearing shells, liners and pistons settle to their
    // own running temperatures over the 8 s
    let pin = |p: &mut VirtualPlant| {
        p.thermal.engine_c = t_c;
        p.thermal.oil_c = t_c;
    };
    for _ in 0..32 {
        hold_air(&mut p, &mut op, mg, 0.25);
        pin(&mut p);
    }
    let mut s = [0.0f64; 11];
    for _ in 0..200 {
        let a = op.command(&p);
        p.step(0.005, &a);
        pin(&mut p);
        let (t, f) = (p.telemetry(), &p.engine.flows);
        let w = p.engine.omega_rad_s.max(1.0);
        let (alt, oil) = (p.alternator_report().0 / w, p.engine.lube.pump_power_w / w);
        let bearings = p.engine.lube.bearings.iter().map(|b| b.friction_w).sum::<f64>() / w;
        let v = [t.air_mg_stroke, f.torque_indicated_nm, f.torque_friction_nm, f.torque_pumping_nm, f.torque_valvetrain_nm, oil, f.torque_aux_nm - alt - oil, alt, t.torque_brake_nm, p.dyno_absorber_nm, bearings];
        s.iter_mut().zip(v).for_each(|(x, v)| *x += v / 200.0);
    }
    s
}

/// Diagnostic: indicated torque against spark at warm idle air (`RPM`, `MG`, default 750 rpm / 100 mg) with the throttle
/// held, normalised to the best spark — the shape the firmware's `ip_eff_iga` describes (docs/SIMOS18_COMBUSTION.md §18).
#[test]
#[ignore]
fn explore_idle_spark_efficiency() {
    let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
    let (rpm, mg) = (env("RPM", 750.0), env("MG", 100.0));
    let mut p = dyno(rpm);
    p.bench = BenchMode::AcDyno { target_rpm: rpm };
    let mut op = Operator::wot(20.0, 1.0, 100.0);
    op.throttle_duty = 0.0;
    op.cams = [0.0, 0.0];
    op.rail_target_mpa = 10.0;
    hold_air(&mut p, &mut op, mg, 4.0);
    let mut rows = Vec::new();
    for k in 0..12 {
        op.spark = 35.0 - 4.0 * k as f64;
        run(&mut p, &op, 1.0);
        let ind = mean(&mut p, &op, 1.0, |t| t.torque_indicated_nm);
        rows.push((op.spark, ind, p.telemetry().air_mg_stroke));
    }
    let best = rows.iter().map(|r| r.1).fold(f64::MIN, f64::max);
    for (spark, ind, air) in rows {
        println!("spark {spark:5.1} °BTDC: indicated {ind:5.1} Nm ({:.3} of best) air {air:5.1} mg", ind / best);
    }
}

/// Diagnostic for the loss comparison (docs/SIMOS18_COMBUSTION.md §18): everything the crank drives except the
/// alternator next to the firmware's loss model. `TEMPS=90,20,5` (coolant = oil).
#[test]
#[ignore]
fn explore_losses_vs_firmware() {
    let temps: Vec<f64> = std::env::var("TEMPS").unwrap_or("90,20,5".into()).split(',').filter_map(|x| x.parse().ok()).collect();
    for t_c in temps {
        println!("--- coolant = oil = {t_c} °C");
        for (rpm, mg, spark) in [(700.0, 105.0, 5.0), (1000.0, 150.0, 20.0), (1500.0, 200.0, 25.0), (2000.0, 280.0, 25.0), (2000.0, 500.0, 15.0), (3000.0, 400.0, 30.0), (4000.0, 600.0, 25.0)] {
            let [air, ind, fric, pump, vt, oil, hpfp, alt, brake, absorber, bearings] = loss_point(rpm, mg, spark, t_c);
            let loss = fric + pump + vt + oil + hpfp;
            let fw = firmware_loss_nm(rpm, air, t_c);
            println!(
                "{rpm:5.0} rpm {air:5.0} mg | ind {ind:6.1} | friction {fric:5.1} (bearings {bearings:4.1}) pumping {pump:5.1} valvetrain {vt:5.1} oil pump {oil:4.1} HPFP {hpfp:4.1} = {loss:5.1} Nm, firmware {fw:5.1} ({:+5.1}) | alternator {alt:4.1} | brake {brake:6.1} absorber {absorber:6.1}",
                loss - fw
            );
        }
    }
}

#[test]
fn phaser_slew_cam_edges_kl87_gating_and_extreme_boundaries() {
    use crate::sensors::EdgeSource;

    // 1. Slewing cam phasers at 3000 rpm (`n_sub > 1`): cam Hall edges must strictly alternate levels (no triple-bounce)
    //    and `SensorEdge::t_s` must be non-decreasing across every step.
    let mut p = dyno(3000.0);
    p.motoring_rpm = Some(3000.0);
    let mut act = PlantActuators::default();
    act.ecu.main_relay = Some(true);
    act.ecu.cam_phaser_duty = [Some(1.0), Some(1.0)];
    let mut last_cam: [Option<bool>; 2] = [None, None];
    let mut cam_counts = [0usize; 2];
    for _ in 0..200 {
        p.step(1e-3, &act);
        assert!(p.engine.edges.windows(2).all(|w| w[0].t_s <= w[1].t_s), "unsorted edges during phaser slew");
        for e in &p.engine.edges {
            if let EdgeSource::Cam { cam, level } = e.source {
                let c = cam as usize;
                if let Some(prev) = last_cam[c] {
                    assert_ne!(prev, level, "duplicate cam {c} edge level {level} during phaser slew");
                }
                last_cam[c] = Some(level);
                cam_counts[c] += 1;
            }
        }
    }
    assert!(cam_counts[0] >= 16 && cam_counts[1] >= 16, "{cam_counts:?}");

    // 2. KL87 (`main_relay`) supply gating on N493 rotary valve and O2 sensor heaters, plus `EcuOutputs::any_driven`.
    let mut cold = VirtualPlant::new(
        EngineProfile::ea888_18t_cjsa_is12(),
        DrivetrainProfile::dq200_fwd(),
        Ambient { temp_c: -30.0, altitude_m: 0.0, humidity_pct: 50.0 },
    );
    cold.thermal = ThermalState::uniform(-30.0);
    let mut unpowered = PlantActuators::default();
    assert!(!unpowered.ecu.any_driven());
    unpowered.ecu.rotary_valve_duty = Some(1.0);
    assert!(unpowered.ecu.any_driven());
    unpowered.ecu.lambda_heater_duty = [Some(1.0), Some(1.0)];
    let rvc0 = cold.rotary_valve.theta_deg;
    let tip0 = cold.exhaust_sensors.up.t_c;
    cold.step(0.05, &unpowered);
    assert_eq!(cold.rotary_valve.theta_deg, rvc0, "N493 must not move with KL87 off");
    assert_eq!(cold.exhaust_sensors.up.heater_w, 0.0, "LSU heater must not draw power with KL87 off");
    assert!((cold.exhaust_sensors.up.t_c - tip0).abs() < 1e-6);

    let mut powered = unpowered.clone();
    powered.ecu.main_relay = Some(true);
    cold.step(0.10, &powered);
    assert!(cold.rotary_valve.theta_deg > rvc0, "N493 moves once KL87 is on");
    assert!(cold.exhaust_sensors.up.heater_w > 1.0, "LSU heater draws power once KL87 is on");

    // 3. `motoring_rpm` from 0 rpm spins sub-step 0 immediately; stopping resets `speed_ripple_rpm` to 0.
    let mut mot = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient::default());
    mot.motoring_rpm = Some(1800.0);
    mot.step(2e-4, &PlantActuators::default());
    assert!(!mot.engine.edges.is_empty(), "first 0.2 ms sub-step at 1800 rpm must cross >2° and emit a tooth");
    mot.motoring_rpm = None;
    mot.engine.speed_ripple_rpm = 42.0;
    mot.engine.set_speed(0.0);
    mot.step(1e-3, &PlantActuators::default());
    assert_eq!(mot.telemetry().crank_speed_ripple_rpm, 0.0);

    // 4. Degenerate dt (0, negative, NaN), reverse crank rotation, and large dt (0.2 s) preserve positive/finite state.
    let t_before = cold.sim_time_s;
    cold.step(0.0, &PlantActuators::default());
    cold.step(-1.0, &PlantActuators::default());
    cold.step(f64::NAN, &PlantActuators::default());
    assert_eq!(cold.sim_time_s, t_before);

    cold.engine.set_speed(-50.0);
    cold.step(0.02, &PlantActuators::default());
    cold.step(0.20, &powered);
    let tel = cold.telemetry();
    assert!(tel.map_kpa > 0.0 && tel.map_kpa.is_finite());
    assert!(tel.put_kpa > 0.0 && tel.put_kpa.is_finite());
    assert!(tel.exhaust_manifold_kpa > 0.0 && tel.exhaust_manifold_kpa.is_finite());
    assert!(cold.engine.cyl.iter().all(|c| c.p > 0.0 && c.p.is_finite() && c.mass() > 0.0 && c.t.is_finite()));
}
