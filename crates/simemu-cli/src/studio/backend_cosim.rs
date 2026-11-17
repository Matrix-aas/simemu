//! Closed-loop co-simulation (ECU + TCU + plant, `StudioSession::step`) on the Unicorn and on the
//! native CPU backend side by side: ignition on, brake, start request 1..3 s. After every 10 ms step
//! the ECU and TCU boards (instructions, interrupts, peripheral accesses, STM) and the plant
//! (telemetry, decoded ECU/TCU outputs, bit-exact `{:?}` text) are compared; the first step that
//! differs is reported and fails the test. `SIMEMU_COSIM_S` (default 6) sets the simulated time.
//! `cargo test --release -p simemu-cli cosim_backends_identical -- --ignored --nocapture`

#[cfg(test)]
mod tests {
    use crate::studio::session::{StudioSession, S50_FLASH};
    use simemu_tc1791::Simos18Board;
    use std::path::PathBuf;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    fn board(b: &Simos18Board) -> [u64; 6] {
        let hw = b.hw();
        [hw.total_instructions, hw.irq_dispatch_count, hw.trap_count, hw.mmio_read_count, hw.mmio_write_count, hw.stm_ticks()]
    }

    /// Everything the step comparison looks at.
    fn snapshot(s: &StudioSession) -> (Vec<u64>, String) {
        let mut v = board(&s.board).to_vec();
        if let Some(t) = &s.tcu {
            v.extend(board(t.cpu()));
        }
        (v, format!("{:?} | {:?} | {:?}", s.plant.telemetry(), s.actuators.ecu, s.actuators.tcu))
    }

    #[test]
    #[ignore]
    fn cosim_backends_identical() {
        if !simemu_tc1791::CpuBackend::Unicorn.is_available() {
            eprintln!("SKIP: built without `--features unicorn`");
            return;
        }
        let Some(root) = root() else {
            eprintln!("SKIP: {S50_FLASH} not found");
            return;
        };
        // The session builds its boards on the backend SIMEMU_CPU names (read at construction).
        let mut s: Vec<StudioSession> = ["unicorn", "native"]
            .iter()
            .map(|cpu| {
                std::env::set_var("SIMEMU_CPU", cpu);
                StudioSession::new(&root).expect("session")
            })
            .collect();
        std::env::remove_var("SIMEMU_CPU");
        // SIMEMU_UC_EXACT=0 + SIMEMU_COSIM_FORCE=1: show where block-granular Unicorn timing diverges
        if !s[0].board.hw().exact_timing && std::env::var_os("SIMEMU_COSIM_FORCE").is_none() {
            eprintln!("SKIP: the linked Unicorn has no exact instruction timing (stock library)");
            return;
        }
        let secs: f64 = std::env::var("SIMEMU_COSIM_S").ok().and_then(|v| v.parse().ok()).unwrap_or(6.0);
        let mut max_rpm = 0.0f64;
        for i in 0..(secs / 0.01) as usize {
            let t = (i + 1) as f64 * 0.01;
            for x in &mut s {
                x.plant.driver.ignition_on = true;
                x.plant.driver.brake_pct = if t >= 0.5 { 60.0 } else { 0.0 };
                x.plant.driver.start_button = (1.0..3.0).contains(&t);
                x.step(0.01);
            }
            let (u, n) = (snapshot(&s[0]), snapshot(&s[1]));
            assert_eq!(u.0, n.0, "boards differ at t = {t:.2} s (ECU then TCU: insns, irqs, traps, mmio r, mmio w, STM)");
            assert_eq!(u.1, n.1, "plant / outputs differ at t = {t:.2} s");
            max_rpm = max_rpm.max(s[1].plant.telemetry().rpm);
        }
        let b = board(&s[1].board);
        println!("identical for {secs} s: ECU {} insns, {} irqs; max {max_rpm:.0} rpm", b[0], b[1]);
    }
}
