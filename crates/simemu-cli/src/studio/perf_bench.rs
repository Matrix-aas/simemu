//! Real-time regression guard of the full co-simulation (ECU TC1791 + TCU TC1784 + plant), methodology in
//! `docs/PERFORMANCE.md`. Run on an otherwise idle machine:
//! `SIMEMU_CPU=native cargo test --release -p simemu-cli cosim_realtime_guard -- --ignored --nocapture`.
//! Scenario: warm engine, ignition on, brake, start request 1..3 s (the engine-start scenario of
//! `studio::engine_run`), `SIMEMU_BENCH_S` (default 8) simulated seconds stepped in 10 ms slices as fast
//! as possible. Fails when the realised speed (simulated / wall time) of the whole run or of the
//! engine-running part (after 4 s) is below `SIMEMU_RTF_MIN` (default `RTF_MIN` = `1.0`).

#[cfg(test)]
mod tests {
    use crate::studio::session::{CosimPerf, StudioSession, S50_FLASH};
    use std::path::PathBuf;
    use std::time::Instant;

    /// Real time. Reference machine (Apple M5 Pro, native core): RTF 1.10..1.18 measured with the machine
    /// otherwise loaded at a load average of 12..13 (docs/PERFORMANCE.md §5), so the guard has headroom
    /// only on an idle machine.
    const RTF_MIN: f64 = 1.0;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    fn add(a: &mut CosimPerf, b: &CosimPerf) {
        a.windows += b.windows;
        a.plant_s += b.plant_s;
        a.ecu_s += b.ecu_s;
        a.tcu_s += b.tcu_s;
        a.wait_tcu_s += b.wait_tcu_s;
        a.tcu_idle_s += b.tcu_idle_s;
        a.glue_s += b.glue_s;
        a.max_window_s = a.max_window_s.max(b.max_window_s);
        a.main_cpu_s += b.main_cpu_s;
        a.tcu_cpu_s += b.tcu_cpu_s;
    }

    fn report(label: &str, sim: f64, wall: f64, p: &CosimPerf) -> f64 {
        let per = |x: f64| 1e3 * x / sim; // ms of wall/CPU per simulated second
        let rtf = sim / wall;
        println!(
            "{label}: {sim:.2} s simulated in {wall:.2} s wall -> RTF {rtf:.3} | per simulated s: plant {:.0} ms, ECU {:.0} ms, glue {:.0} ms, \
             wait-for-TCU {:.0} ms (session thread); TCU {:.0} ms, TCU idle {:.0} ms (worker) | CPU: session {:.0} ms, TCU {:.0} ms | \
             max 1 ms window {:.2} ms | {} windows",
            per(p.plant_s),
            per(p.ecu_s),
            per(p.glue_s),
            per(p.wait_tcu_s),
            per(p.tcu_s),
            per(p.tcu_idle_s),
            per(p.main_cpu_s),
            per(p.tcu_cpu_s),
            p.max_window_s * 1e3,
            p.windows
        );
        rtf
    }

    #[test]
    #[ignore]
    fn cosim_realtime_guard() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        let (secs, rtf_min) = (env("SIMEMU_BENCH_S", 8.0), env("SIMEMU_RTF_MIN", RTF_MIN));
        // the session thread of the studio runner has this role too (performance cores on macOS)
        crate::studio::rt::set_thread_role(crate::studio::rt::ThreadRole::Realtime);
        let mut s = StudioSession::new(&root).expect("session");
        s.plant.soak(90.0);
        s.plant.driver.ignition_on = true;
        s.plant.driver.selector = simemu_physics::Selector::P;
        let (mut all, mut run) = (CosimPerf::default(), CosimPerf::default());
        let (mut wall_all, mut wall_run, mut sim_run) = (0.0, 0.0, 0.0);
        for i in 0..(secs / 0.01) as usize {
            let t = (i + 1) as f64 * 0.01;
            s.plant.driver.brake_pct = if t >= 0.5 { 60.0 } else { 0.0 };
            s.plant.driver.start_button = (1.0..3.0).contains(&t);
            let w0 = Instant::now();
            s.step(0.01);
            let w = w0.elapsed().as_secs_f64();
            wall_all += w;
            add(&mut all, &s.frame.perf);
            if t > 4.0 {
                wall_run += w;
                sim_run += 0.01;
                add(&mut run, &s.frame.perf);
            }
        }
        assert!(s.ecu_error.is_none() && s.tcu_error.is_none(), "{:?} {:?}", s.ecu_error, s.tcu_error);
        let rpm = s.plant.telemetry().rpm;
        let hw = s.board.hw();
        println!(
            "backend {:?}, engine {rpm:.0} rpm at the end, load average {} | ECU {} insns, {} quanta ({} cut early), {} IRQs",
            s.board.backend(),
            load_average(),
            hw.total_instructions,
            hw.quanta,
            hw.quanta_stopped_early,
            hw.irq_dispatch_count
        );
        let r_all = report("whole run (boot..start..idle)", secs, wall_all, &all);
        let r_run = if sim_run > 0.0 { report("engine running (> 4 s)", sim_run, wall_run, &run) } else { r_all };
        assert!(r_all >= rtf_min && r_run >= rtf_min, "co-simulation below real time x{rtf_min}: {r_all:.3} / {r_run:.3}");
    }

    #[test]
    #[ignore]
    fn cosim_high_rpm_profile() {
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        crate::studio::rt::set_thread_role(crate::studio::rt::ThreadRole::Realtime);
        let mut s = StudioSession::new(&root).expect("session");
        s.plant.soak(90.0);
        s.plant.driver.ignition_on = true;
        s.plant.driver.selector = simemu_physics::Selector::P;
        s.plant.bench = simemu_physics::BenchMode::RoadLoadDyno;

        // Boot + start + engage S + launch (0..6.0 s)
        for i in 0..600 {
            let t = (i + 1) as f64 * 0.01;
            s.plant.driver.brake_pct = if (0.5..4.5).contains(&t) { 60.0 } else { 0.0 };
            s.plant.driver.start_button = (1.0..3.0).contains(&t);
            if t >= 3.8 {
                s.plant.driver.selector = simemu_physics::Selector::S;
            }
            s.plant.driver.pedal_pct = if t >= 4.5 { 100.0 } else { 0.0 };
            s.step(0.01);
        }

        // Hold each RPM with ChassisDyno (90 km/h roller speed) + WOT so ECU & TCU run at full load up to 6500 RPM
        s.plant.driver.selector = simemu_physics::Selector::D;
        s.plant.bench = simemu_physics::BenchMode::ChassisDyno { target_kmh: 90.0 };
        for &target_rpm in &[1500.0, 2500.0, 3500.0, 4500.0, 5500.0, 6500.0] {
            s.plant.motoring_rpm = Some(target_rpm);
            s.plant.driver.pedal_pct = 100.0;
            for _ in 0..40 {
                s.step(0.01);
            }
            let hw0 = s.board.hw();
            let (ins0, q0, qe0, irq0, mr0, mw0, fc0, pcp0) = (
                hw0.total_instructions,
                hw0.quanta,
                hw0.quanta_stopped_early,
                hw0.irq_dispatch_count,
                hw0.mmio_read_count,
                hw0.mmio_write_count,
                hw0.fadc.conversions,
                hw0.engine_io.pcp().stats.instructions,
            );
            let mut perf = CosimPerf::default();
            let w0 = Instant::now();
            for _ in 0..100 {
                s.step(0.01);
                add(&mut perf, &s.frame.perf);
            }
            let wall = w0.elapsed().as_secs_f64();
            let tel = s.plant.telemetry();
            let hw1 = s.board.hw();
            println!(
                "=== WOT target {target_rpm:.0} (actual {:.0} rpm, {:.0} Nm, MAP {:.0} kPa) ===",
                tel.rpm, tel.torque_brake_nm, tel.map_kpa
            );
            report(&format!("  @{target_rpm:.0} rpm"), 1.0, wall, &perf);
            println!(
                "  ECU: +{} insns, +{} quanta (+{} early), +{} IRQs, +{} mmio_r, +{} mmio_w | FADC +{} conv | PCP +{} insns",
                hw1.total_instructions - ins0,
                hw1.quanta - q0,
                hw1.quanta_stopped_early - qe0,
                hw1.irq_dispatch_count - irq0,
                hw1.mmio_read_count - mr0,
                hw1.mmio_write_count - mw0,
                hw1.fadc.conversions - fc0,
                hw1.engine_io.pcp().stats.instructions - pcp0,
            );
            assert!(
                s.ecu_error.is_none() && s.tcu_error.is_none(),
                "board fault at {target_rpm:.0} rpm: ecu={:?} tcu={:?}",
                s.ecu_error,
                s.tcu_error
            );
        }
    }

    /// System load while measuring (wall-clock results are only comparable on an idle machine).
    fn load_average() -> String {
        std::process::Command::new("sysctl").args(["-n", "vm.loadavg"]).output().map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string()).unwrap_or_default()
    }
}
