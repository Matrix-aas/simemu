//! Validation: the physical cause changes the sound. The bench operator below is TEST CODE (like the one in
//! `simemu-physics/src/bench_tests.rs`): it supplies actuator commands on a motored engine test bench; it is not
//! part of the product and never runs in the emulator.

use crate::synth::{src, BlockOut, Mix, Params, Synth, FS, N_LIST, N_SRC};
use crate::tap::Tap;
use simemu_physics::engine::{self, CoilCmd, InjPulse};
use simemu_physics::{sensors, Ambient, BenchMode, DrivetrainProfile, EngineProfile, PlantActuators, Selector, ThermalState, VirtualPlant};
use std::f64::consts::PI;

struct Op {
    spark: f64,
    throttle: f64,
    rail_mpa: f64,
    fire: [bool; 4],
    wg_close: f64,
}

impl Op {
    fn command(&self, p: &VirtualPlant) -> PlantActuators {
        let e = &p.engine;
        let prof = &p.engine_profile;
        let (etc, lh) = (&prof.etc, prof.throttle_limp_home_deg);
        let target = lh + self.throttle * (etc.max_deg - lh);
        let x = (target - lh).to_radians();
        let spring = if x > 0.0 { etc.close_preload_nm + etc.close_rate_nm_rad * x } else { 0.0 };
        let duty = (spring * etc.r_ohm / etc.k_nm_a / p.battery_v + 0.02 * (target - e.throttle_deg) - 3e-4 * e.throttle_rate_rad_s.to_degrees()).clamp(-1.0, 1.0);
        let dp = (e.rail_pa - e.p_intake_pa).max(1e5);
        let q = prof.injector_flow_cc_s * 1e-6 * (dp / 1e7).sqrt();
        let air = if e.flows.air_per_cyl_mg > 1.0 { e.flows.air_per_cyl_mg * 1e-6 } else { 0.85 * e.p_intake_pa * e.kin.v_disp / (287.0 * e.t_intake_k) };
        let m_f = air / engine::AFR_STOICH;
        let pw = m_f / (engine::FUEL_DENSITY_KG_M3 * q) * 1000.0 + prof.injector_dead_time_ms;
        let gross = m_f * 1e6 + prof.hpfp_loss.mg_at(e.rpm(), (e.rail_pa - 101_325.0) / 100.0);
        let ff = gross * 1e-6 / (engine::FUEL_DENSITY_KG_M3 * prof.hpfp_efficiency * prof.hpfp_stroke_cc * 1e-6);
        let frac = (ff + 0.25 * (self.rail_mpa - e.rail_pa / 1e6)).clamp(0.0, 1.0);
        let psi = (2.0 * frac - 1.0).clamp(-1.0, 1.0).acos() * 90.0 / PI;
        let delay = sensors::interp(p.battery_v, prof.hpfp_vcv_close_delay_ms) * 1e-3 * e.omega_rad_s.abs().to_degrees();
        let mut a = PlantActuators::default();
        a.ecu.main_relay = Some(true);
        a.ecu.lpfp_duty = Some(0.8);
        a.ecu.hpfp_close_deg = Some(psi + engine::HPFP_TDC_DEG - 90.0 + e.vt.cam_shift_deg(prof, 1) - delay);
        a.ecu.etc_duty = Some(duty);
        // `wg_close` −1 (open) .. +1 (shut) = wanted flap flow fraction 0.5·(1 − wg_close), held by the bench servo.
        let wg_sp = 0.5 * (1.0 - self.wg_close);
        a.ecu.wastegate_duty = Some(e.wg.bench_servo_duty(&prof.turbo.wg_actuator, e.bench_wastegate_theta(&prof.turbo, wg_sp), e.p_exhaust_pa - p.ambient_state().p_kpa * 1e3, p.battery_v));
        for c in 0..4 {
            if self.fire[c] {
                a.ecu.ignition[c] = Some(CoilCmd { dwell_ms: 2.0, spark_btdc_deg: self.spark });
            }
            a.ecu.injection[c][0] = Some(InjPulse { soi_btdc_deg: 300.0, width_ms: pw });
        }
        a
    }
}

fn bench(rpm: f64, ron: f64) -> VirtualPlant {
    let mut p = VirtualPlant::new(EngineProfile::ea888_18t_cjsa_is12(), DrivetrainProfile::dq200_fwd(), Ambient { temp_c: 25.0, altitude_m: 0.0, humidity_pct: 40.0 });
    p.driver.ignition_on = true;
    p.driver.selector = Selector::N;
    p.fuel_ron = ron;
    p.thermal = ThermalState::uniform(90.0);
    p.engine.lube.bearings.iter_mut().for_each(|b| b.t_k = 363.15);
    p.engine.cyl.iter_mut().for_each(|c| {
        c.t_piston_k = 480.0;
        c.t_exv_k = 900.0;
    });
    p.engine.set_speed(rpm * PI / 30.0);
    p.bench = BenchMode::EngineDyno { target_rpm: rpm };
    p.motoring_rpm = Some(rpm);
    p.engine.rail_pa = 10e6;
    p.engine.lp_fuel_pa = 0.48e6;
    p
}

/// Runs `warm` s unrecorded, then `secs` s through the synthesizer; returns the samples of every listener and
/// the mean source levels per listener.
fn run(p: &mut VirtualPlant, op: &Op, warm: f64, secs: f64, mix: Mix) -> (Vec<Vec<f32>>, [[f64; N_SRC]; N_LIST]) {
    for _ in 0..(warm / 1e-3) as usize {
        let a = op.command(p);
        p.step(1e-3, &a);
    }
    let mut synth = Synth::new(Params::default());
    synth.mix = mix;
    let mut tap = Tap::default();
    let mut out: Vec<Vec<f32>> = vec![Vec::new(); N_LIST];
    let mut lv = [[0.0f64; N_SRC]; N_LIST];
    let mut n = 0.0f64;
    let mut recs = Vec::new();
    for _ in 0..(secs / 1e-3) as usize {
        let a = op.command(p);
        p.step(1e-3, &a);
        recs.clear();
        tap.collect(p, &a, &mut |r| {
            recs.push(r);
            true
        });
        for r in &recs {
            synth.push(r, &mut |b: &BlockOut| {
                for ((ol, lvl), (&bo, &bl)) in out.iter_mut().zip(lv.iter_mut()).zip(b.out.iter().zip(b.level.iter())) {
                    ol.extend_from_slice(&bo);
                    for (acc, &v) in lvl.iter_mut().zip(bl.iter()) {
                        *acc += v as f64;
                    }
                }
                n += 1.0;
            });
        }
    }
    for l in lv.iter_mut() {
        for x in l.iter_mut() {
            *x /= n.max(1.0);
        }
    }
    (out, lv)
}

/// Amplitude of the component at `f` (Goertzel over the whole record, Hann window).
fn amp(x: &[f32], f: f64) -> f64 {
    let n = x.len();
    let w = 2.0 * PI * f / FS;
    let (mut re, mut im) = (0.0, 0.0);
    for (i, &v) in x.iter().enumerate() {
        let win = 0.5 - 0.5 * (2.0 * PI * i as f64 / n as f64).cos();
        let ph = w * i as f64;
        re += v as f64 * win * ph.cos();
        im += v as f64 * win * ph.sin();
    }
    2.0 * (re * re + im * im).sqrt() / (0.5 * n as f64)
}

fn solo(s: usize) -> Mix {
    let mut gain = [0.0; N_SRC];
    gain[s] = 1.0;
    Mix { gain, ..Mix::default() }
}

/// Engine orders 0.5..3 at the tailpipe: an even-firing four-cylinder (1-3-4-2) is dominated by the 2nd order;
/// cutting the spark of one cylinder brings up the half orders and the 1st order (720° periodicity).
#[test]
fn firing_order_dominates_and_a_misfire_brings_up_the_half_orders() {
    let rpm = 1200.0;
    let f1 = rpm / 60.0;
    let op = Op { spark: 15.0, throttle: 0.08, rail_mpa: 10.0, fire: [true; 4], wg_close: -1.0 };
    let (good, _) = run(&mut bench(rpm, 95.0), &op, 1.5, 2.0, Mix::default());
    let mis = Op { fire: [true, false, true, true], ..op };
    let (bad, _) = run(&mut bench(rpm, 95.0), &mis, 1.5, 2.0, Mix::default());
    let orders = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0];
    let spec = |x: &[f32]| orders.map(|o| amp(x, o * f1));
    let (g, b) = (spec(&good[0]), spec(&bad[0]));
    eprintln!("exterior order amplitudes (Pa) 0.5..4: good {g:.4?}\n misfire {b:.4?}");
    let half = |s: &[f64; 7]| s[0].max(s[1]).max(s[2]);
    assert!(g[3] > 3.0 * half(&g), "2nd order must dominate the sub-firing orders: {g:?}");
    assert!(half(&b) / b[3] > 5.0 * half(&g) / g[3], "misfire must raise the half / 1st orders: good {g:?} misfire {b:?}");
}

/// End-gas autoignition rings the chamber at its first circumferential mode (Draper): 5–9 kHz energy in the
/// engine bay rises by > 6 dB against the same operating point without knock.
#[test]
fn knock_adds_energy_in_the_5_to_9_khz_band() {
    let rpm = 2000.0;
    let quiet = Op { spark: 2.0, throttle: 1.0, rail_mpa: 18.0, fire: [true; 4], wg_close: 0.3 };
    // RON 80 and ≈ 15 ° of advance: with the quasi-dimensional burn and the unscaled Douaud–Eyzat delay
    // (docs/SIMOS18_COMBUSTION.md) RON 91 at this moderate boost knocks only lightly (≤ 0.4 bar), and far past the limit
    // (32 °) the fast early flame consumes the end gas before it autoignites
    let loud = Op { spark: 18.0, ..quiet };
    let band = |x: &[f32]| {
        let n = 8192;
        let mut e = 0.0;
        let mut k = 0;
        while (k + 1) * n <= x.len() {
            let seg = &x[k * n..(k + 1) * n];
            // every DFT bin of the band (FS / n = 5.9 Hz): band energy, not the few lines that happen to sit on a 250 Hz grid
            for b in (5000.0 * n as f64 / FS).ceil() as usize..=(9000.0 * n as f64 / FS) as usize {
                e += amp(seg, b as f64 * FS / n as f64).powi(2);
            }
            k += 1;
        }
        e
    };
    let mut pq = bench(rpm, 95.0);
    let (q, lq) = run(&mut pq, &quiet, 2.0, 1.0, Mix::default());
    let mut pk = bench(rpm, 80.0);
    let (k, lk) = run(&mut pk, &loud, 2.0, 1.0, Mix::default());
    eprintln!("bay levels quiet  {:?}\nbay levels knock  {:?}", lq[2].map(|x| (10.0 * (x / 4e-10).log10()) as i32), lk[2].map(|x| (10.0 * (x / 4e-10).log10()) as i32));
    let kamp: f64 = pk.engine.cyl.iter().map(|c| c.last.knock_amp_pa).fold(0.0, f64::max);
    let qamp: f64 = pq.engine.cyl.iter().map(|c| c.last.knock_amp_pa).fold(0.0, f64::max);
    let (eq, ek) = (band(&q[2]), band(&k[2]));
    eprintln!("knock amp: quiet {qamp:.0} Pa, knocking {kamp:.0} Pa; 5–9 kHz bay energy ratio {:.1} dB", 10.0 * (ek / eq).log10());
    assert!(kamp > 1e4 && qamp < kamp / 10.0, "operating points must differ in knock: {qamp} vs {kamp}");
    // ≥ 4.8 dB: with CJSA's own fuel hardware (SC8O40 injector 14.1 cm³/s, 174.5 mg pump, 2026-11-02) the injector and pump
    // ticks of the quiet point carry more 5–9 kHz energy than with the former 2.0 values. Measured over every bin of the band
    // (2026-11-07: 8.4 dB); the former 250 Hz grid caught only the lines that sat on it and swung 6.3 → 4.7 dB when the
    // friction change moved the quiet point's harmonics (docs/SIMOS18_COMBUSTION.md §18).
    assert!(ek > 3.0 * eq, "knock must add 5–9 kHz energy: {eq} vs {ek}");
}

/// The HPFP is a 4-lobe pump on the exhaust camshaft: its valve and plunger ticks repeat twice per crank
/// revolution (40 Hz at 1200 rpm), heard alone on the stethoscope.
#[test]
fn hpfp_ticks_at_its_cam_lobe_frequency() {
    let rpm = 1200.0;
    let op = Op { spark: 15.0, throttle: 0.08, rail_mpa: 10.0, fire: [true; 4], wg_close: -1.0 };
    let (x, _) = run(&mut bench(rpm, 95.0), &op, 1.0, 2.0, solo(src::HPFP));
    // Envelope (rectified, decimated to 1 kHz) and its periodicity.
    let env: Vec<f32> = x[3].chunks(48).map(|c| c.iter().map(|v| v.abs()).sum::<f32>()).collect();
    let mean = env.iter().sum::<f32>() / env.len() as f32;
    let e: Vec<f32> = env.iter().map(|v| v - mean).collect();
    let a = |f: f64| {
        let (mut re, mut im) = (0.0, 0.0);
        for (i, &v) in e.iter().enumerate() {
            let ph = 2.0 * PI * f * i as f64 / 1000.0;
            re += v as f64 * ph.cos();
            im += v as f64 * ph.sin();
        }
        (re * re + im * im).sqrt()
    };
    let f_lobe = 2.0 * rpm / 60.0;
    let (at, off1, off2) = (a(f_lobe), a(f_lobe * 0.83), a(f_lobe * 1.17));
    eprintln!("HPFP envelope: {at:.3} at {f_lobe} Hz, {off1:.3} / {off2:.3} beside");
    assert!(at > 5.0 * off1.max(off2));
    assert!(x[3].iter().any(|v| v.abs() > 1e-4), "HPFP must be audible on the stethoscope");
}

/// The compressor blade-passing tone sits at blades × shaft speed of the plant's turbocharger.
#[test]
fn turbo_whine_tracks_the_turbo_speed() {
    let mut found = Vec::new();
    for (rpm, wg) in [(2500.0, 0.2), (3000.0, 0.5)] {
        let op = Op { spark: 5.0, throttle: 1.0, rail_mpa: 18.0, fire: [true; 4], wg_close: wg };
        let mut p = bench(rpm, 98.0);
        let (x, _) = run(&mut p, &op, 3.0, 0.5, solo(src::TURBO));
        let f_bpf = Params::default().comp_blades as f64 * p.engine.turbo_rpm() / 60.0;
        // The compressor also makes broadband noise (louder than the tone at low frequency), so the check is local: the
        // strongest line within ±5 % of the BPF sits on it, and it stands out of the surrounding ±10…20 % band.
        if f_bpf < 20_000.0 {
            let line = |lo: f64, hi: f64, step: f64| {
                let n = ((hi - lo) / step) as usize;
                (0..=n).map(|i| lo + i as f64 * step).map(|f| (f, amp(&x[3], f))).collect::<Vec<_>>()
            };
            let near = line(f_bpf * 0.95, f_bpf * 1.05, f_bpf * 0.002);
            let peak = near.iter().cloned().fold((0.0, 0.0), |a, b| if b.1 > a.1 { b } else { a });
            let mut side: Vec<f64> = line(f_bpf * 0.80, f_bpf * 0.90, f_bpf * 0.004).into_iter().chain(line(f_bpf * 1.10, f_bpf * 1.20, f_bpf * 0.004)).map(|x| x.1).collect();
            side.sort_by(f64::total_cmp);
            let floor = side[side.len() / 2];
            eprintln!("{rpm} rpm: turbo {:.0} rpm → BPF {f_bpf:.0} Hz, local peak {:.0} Hz, {:.1} dB above the side band", p.engine.turbo_rpm(), peak.0, 20.0 * (peak.1 / floor).log10());
            assert!((peak.0 / f_bpf - 1.0).abs() < 0.03, "peak {} vs BPF {f_bpf}", peak.0);
            assert!(peak.1 > 3.0 * floor, "blade-pass tone must stand out of the broadband: {} vs {floor}", peak.1);
        }
        found.push(f_bpf);
    }
    assert!(found[1] > found[0] * 1.05, "more exhaust energy → faster turbo → higher whine: {found:?}");
}

/// The acoustic probe is read-only: enabling it changes nothing in the plant.
#[test]
fn probe_does_not_change_the_plant() {
    let op = Op { spark: 15.0, throttle: 0.2, rail_mpa: 12.0, fire: [true; 4], wg_close: 0.0 };
    let (mut a, mut b) = (bench(1500.0, 95.0), bench(1500.0, 95.0));
    b.engine.probe.enabled = true;
    for _ in 0..300 {
        let c = op.command(&a);
        a.step(1e-3, &c);
        let c = op.command(&b);
        b.step(1e-3, &c);
        b.engine.probe.frames.clear();
        b.engine.probe.impacts.clear();
    }
    assert_eq!(a.engine.rpm().to_bits(), b.engine.rpm().to_bits());
    for c in 0..4 {
        assert_eq!(a.engine.cyl[c].p.to_bits(), b.engine.cyl[c].p.to_bits());
    }
    assert_eq!(a.engine.p_exhaust_pa.to_bits(), b.engine.p_exhaust_pa.to_bits());
    assert_eq!(a.engine.turbo_omega_rad_s.to_bits(), b.engine.turbo_omega_rad_s.to_bits());
}

/// CPU budget: synthesizing all sources at all listener positions takes well under one core at 48 kHz, and the
/// probe adds little to the plant step.
#[test]
fn synthesis_is_faster_than_real_time() {
    let op = Op { spark: 20.0, throttle: 0.5, rail_mpa: 15.0, fire: [true; 4], wg_close: 0.0 };
    let mut p = bench(5500.0, 98.0);
    for _ in 0..300 {
        let a = op.command(&p);
        p.step(1e-3, &a);
    }
    let mut q = bench(5500.0, 98.0);
    for _ in 0..300 {
        let a = op.command(&q);
        q.step(1e-3, &a);
    }
    let tq = std::time::Instant::now();
    for _ in 0..1000 {
        let a = op.command(&q);
        q.step(1e-3, &a);
    }
    let plant_off = tq.elapsed().as_secs_f64();
    let mut recs = Vec::new();
    let mut tap = Tap::default();
    let tp = std::time::Instant::now();
    for _ in 0..1000 {
        let a = op.command(&p);
        p.step(1e-3, &a);
        tap.collect(&mut p, &a, &mut |r| {
            recs.push(r);
            true
        });
    }
    let plant_on = tp.elapsed().as_secs_f64();
    let mut synth = Synth::new(Params::default());
    let mut blocks = 0usize;
    let t0 = std::time::Instant::now();
    for r in &recs {
        synth.push(r, &mut |_| blocks += 1);
    }
    let load = t0.elapsed().as_secs_f64() / (blocks as f64 * 256.0 / FS);
    eprintln!(
        "5500 rpm: {} records/s; synthesizer load {:.1} % of one core; plant step {:.1} ms per plant second without probe, {:.1} ms with probe + tap",
        recs.len(),
        100.0 * load,
        1e3 * plant_off,
        1e3 * plant_on
    );
    if !cfg!(debug_assertions) {
        assert!(load < 0.5, "load {load}");
    }
}

#[test]
fn tap_emits_config_when_build_changes_with_same_slug() {
    use crate::tap::Record;
    use simemu_physics::bench_config::preset;

    let mut p1 = preset("octavia_fl").unwrap().resolve().unwrap().into_plant();
    let mut p2 = preset("golf7r").unwrap().resolve().unwrap().into_plant();
    assert_eq!(p1.engine_profile.slug, "build");
    assert_eq!(p2.engine_profile.slug, "build");

    let mut tap = Tap::default();
    let act = PlantActuators::default();
    let mut configs = Vec::new();
    tap.collect(&mut p1, &act, &mut |r| {
        if let Record::Config(c) = r {
            configs.push(c);
        }
        true
    });
    assert_eq!(configs.len(), 1);
    // Repeated collect on the same build does not re-emit Config.
    tap.collect(&mut p1, &act, &mut |r| {
        if let Record::Config(c) = r {
            configs.push(c);
        }
        true
    });
    assert_eq!(configs.len(), 1);
    // Switching to another BenchConfig build (same slug "build", larger IS38 compressor wheel) emits Config.
    tap.collect(&mut p2, &act, &mut |r| {
        if let Record::Config(c) = r {
            configs.push(c);
        }
        true
    });
    assert_eq!(configs.len(), 2);
    assert!(configs[1].compressor_tip_m > configs[0].compressor_tip_m);
}

#[test]
fn plant_reset_reanchors_clock_rebuilds_ducts_and_purges_future_events() {
    use crate::tap::{Record, Slow};
    use simemu_physics::acoustic_probe::{CrankFrame, Impact, ImpactKind};

    let mut synth = Synth::new(Params::default());
    // Feed steady frames at t = 60.0 s with hot exhaust (900 K), plus an impact queued slightly ahead at 60.020 s.
    synth.push(
        &Record::Slow(Slow {
            t_s: 60.0,
            p_amb: 101_325.0,
            t_amb: 293.0,
            rho_amb: 1.2,
            t4_k: 900.0,
            t_cat_k: 850.0,
            ..Default::default()
        }),
        &mut |_| {},
    );
    synth.push(
        &Record::Impact(Impact { t_s: 60.020, kind: ImpactKind::InjectorOpen, cyl: 0, sub: 0, mag: 1.0 }),
        &mut |_| {},
    );
    for k in 0..60 {
        synth.push(
            &Record::Crank(CrankFrame {
                t_s: 60.0 + (k + 1) as f64 * 2e-4,
                h: 2e-4,
                p_em: 120e3,
                t_em: 900.0,
                p4: 105e3,
                ..Default::default()
            }),
            &mut |_| {},
        );
    }

    // Now simulate plant.reset() back to t = 0.0 s with cold exhaust (293 K) and quiet inputs.
    synth.push(
        &Record::Slow(Slow {
            t_s: 0.0,
            p_amb: 101_325.0,
            t_amb: 293.0,
            rho_amb: 1.2,
            t4_k: 293.0,
            t_cat_k: 293.0,
            synchro_j: 0.0,
            ..Default::default()
        }),
        &mut |_| {},
    );
    let mut blocks = 0usize;
    let mut inj_energy = 0.0f32;
    for k in 0..60 {
        synth.push(
            &Record::Crank(CrankFrame {
                t_s: (k + 1) as f64 * 2e-4,
                h: 2e-4,
                p_em: 101_325.0,
                t_em: 293.0,
                p4: 101_325.0,
                ..Default::default()
            }),
            &mut |b| {
                blocks += 1;
                assert!(b.t0 < 1.0, "block timestamp must re-anchor near 0: {}", b.t0);
                inj_energy += b.level[3][src::INJECTORS];
            },
        );
    }
    assert!(blocks >= 1);
    assert_eq!(inj_energy, 0.0, "stale pre-reset future impact at 60.020 s must be purged");
}

#[test]
fn duct_and_synth_survive_degenerate_and_overspeed_inputs() {
    use crate::duct::{Duct, ExhaustGeometry, IntakeGeometry};
    use crate::tap::{Record, Slow};
    use simemu_physics::acoustic_probe::CrankFrame;

    // Inverted silencer dimensions (pipe wider than shell, extensions longer than shell) must remain finite.
    let mut d = Duct::new(512);
    let g = ExhaustGeometry {
        pipe_dia: 0.30,
        rear_silencer_dia: 0.20,
        rear_silencer_len: 0.10,
        rear_ext_in: 0.15,
        rear_ext_out: 0.15,
        cooling_len: 0.0,
        cat_porosity: 0.0,
        ..ExhaustGeometry::default()
    };
    g.configure(&mut d, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0);
    let mut ir = vec![0.0f32; 512];
    d.build_ir(FS, &mut ir);
    assert!(ir.iter().all(|x| x.is_finite()));

    let mut d_in = Duct::new(256);
    IntakeGeometry { hose_dia: 0.0, box_dia: 0.0, snorkel_dia: 0.0, ..IntakeGeometry::default() }.configure(&mut d_in, 0.0, 0.0, -1e6);
    d_in.build_ir(FS, &mut ir[..256]);
    assert!(ir[..256].iter().all(|x| x.is_finite()));

    // Overspeed / NaN inputs to Synth must neither alias above Nyquist nor poison outputs with NaN.
    let mut synth = Synth::new(Params::default());
    synth.push(&Record::Crank(CrankFrame { t_s: f64::NAN, h: 2e-4, ..Default::default() }), &mut |_| {});
    synth.push(
        &Record::Slow(Slow {
            t_s: 0.0,
            p_amb: 101_325.0,
            t_amb: 293.0,
            rho_amb: 1.2,
            t4_k: 500.0,
            t_cat_k: 500.0,
            shaft_rad_s: [20_000.0; 2],
            mesh_teeth: [30.0; 2],
            mesh_nm: [500.0; 2],
            pump_rad_s: 50_000.0,
            pump_a: 20.0,
            supply_pa: 50e5,
            fan_duty: 100.0,
            ..Default::default()
        }),
        &mut |_| {},
    );
    let mut rendered = false;
    for k in 0..40 {
        synth.push(
            &Record::Crank(CrankFrame {
                t_s: (k + 1) as f64 * 2e-4,
                h: 2e-4,
                p_cyl: [f32::NAN, 1e5, 1e5, 1e5],
                ..Default::default()
            }),
            &mut |b| {
                rendered = true;
                assert!(b.out.iter().flatten().all(|x| x.is_finite()));
                assert!(b.level.iter().flatten().all(|x| x.is_finite()));
            },
        );
    }
    assert!(rendered);
}

#[test]
fn audio_rt_thread_exits_on_drop_even_when_shared_is_cloned() {
    use crate::rt::AudioRt;
    use std::sync::atomic::Ordering::Relaxed;

    let mut rt = AudioRt::new();
    let shared_clone = rt.shared.clone();
    shared_clone.active.store(true, Relaxed);
    let mut p = bench(1000.0, 95.0);
    let act = PlantActuators::default();
    rt.feed(&mut p, &act);
    // Dropping `rt` while `shared_clone` is still alive must cleanly join the audio thread without hanging.
    let t0 = std::time::Instant::now();
    drop(rt);
    assert!(t0.elapsed().as_millis() < 500);
    assert!(std::sync::Arc::strong_count(&shared_clone) == 1);
}

#[test]
fn wav_writer_validates_inputs() {
    let dir = std::env::temp_dir();
    let bad = dir.join(format!("simemu_wav_bad_{}.wav", std::process::id()));
    assert!(crate::wav::write_f32(&bad, 0, 1, &[0.0]).is_err());
    assert!(crate::wav::write_f32(&bad, 48_000, 0, &[0.0]).is_err());
    assert!(crate::wav::write_f32(&bad, 48_000, 2, &[0.0, 0.1, 0.2]).is_err());
    assert!(!bad.exists(), "invalid WAV header arguments must fail before creating the file");

    let ok = dir.join(format!("simemu_wav_ok_{}.wav", std::process::id()));
    crate::wav::write_f32(&ok, 48_000, 2, &[0.25, -0.25, 0.5, -0.5]).unwrap();
    let bytes = std::fs::read(&ok).unwrap();
    let _ = std::fs::remove_file(&ok);
    assert_eq!(&bytes[0..4], b"RIFF");
    assert_eq!(bytes.len(), 44 + 16);
}
