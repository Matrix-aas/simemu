//! End-to-end: unmodified SC800S50 firmware <-> SSC1 + DMA + monitoring unit (no legacy hooks).
//!
//! Proves that the firmware's ECM3 processor monitoring accepts the MU: the MC walks
//! INIT -> CONF -> NORM, FS-IST and PFM run, and no ECM3 anti-bounce counter is incremented and
//! no fault reaction happens (the switch-off path check passes with the ATIC driver models, see
//! `power_stages_e2e.rs`). Skipped when the image is not present.

mod common;

use simemu_tc1791::monitor_device::MuState;
use simemu_tc1791::{CpuBackend, Simos18Board};

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;
/// ACTION_ECM3_McFaultReaction (d4 = ERR_COD_MC) and the increment path of
/// ACTION_ECM3_HandleAbc (d9 = ERR_COD_MC) in SC800S50.
const MC_FAULT_REACTION: u32 = 0x8006_D120;
const HANDLE_ABC_INC: u32 = 0x8006_D206;

#[test]
fn s50_ecm3_accepts_the_monitoring_unit() {
    for backend in CpuBackend::matrix() {
        s50_ecm3_accepts_the_monitoring_unit_on(backend);
    }
}

fn find(name: &str) -> Option<Vec<u8>> {
    common::find_image(name)
}

fn s50_ecm3_accepts_the_monitoring_unit_on(backend: CpuBackend) {
    let (Some(flash), Some(eeprom)) = (find(FACTORY), find(EEPROM)) else {
        eprintln!("skipped: {FACTORY} not found");
        return;
    };
    // Same bytes as SC800S50 at the probe sites (other versions are laid out differently).
    assert_eq!(&flash[0x6_D120..0x6_D128], &[0x20, 0x08, 0x02, 0x48, 0x49, 0xa4, 0x00, 0x0a]);
    let mut b = Simos18Board::new_on(&flash, &eeprom, backend).unwrap();
    b.set_adc_input_voltage(2, 4, 2.5).unwrap(); // supply sense (see boot_regression.rs)
    b.add_probe(MC_FAULT_REACTION).unwrap();
    b.add_probe(HANDLE_ABC_INC).unwrap();
    let (reactions, increments) = (0, 0);
    let mut norm_seen = false;
    for _ in 0..240 {
        b.run_for(0.005).unwrap();
        norm_seen |= b.hw().spi.mu.state() == MuState::Norm;
        let p = b.probe(MC_FAULT_REACTION).unwrap();
        assert_eq!(p.hits, reactions, "ECM3 fault reaction, ERR_COD_MC {:#x}", p.last_regs[4]);
        let p = b.probe(HANDLE_ABC_INC).unwrap();
        assert_eq!(p.hits, increments, "ECM3 error counter raised for code {:#x}", p.last_regs[9]);
    }
    let hw = b.hw();
    let s = hw.spi.mu.stats;
    println!("{s:?}, fault reactions {reactions}, resets {}", hw.reset_count);
    assert!(norm_seen, "MC/MU reached NORM");
    assert!(s.frames >= 10 && s.ist_correct > 0, "MoC frames exchanged and FS-IST answered");
    assert_eq!((s.faults, s.parity_errors, s.protocol_errors), (0, 0, 0));
    assert!(hw.spi.dma.ch[1].transfers > 0 && hw.spi.ssc[1].frames > 0);
    assert_eq!(hw.reset_count, 0);
}
