//! End-to-end: the level-2 "monitoring of A/D conversion" (ECM2 17.5, FR p.4939) of the unmodified SC800S50
//! firmware sees a live test voltage. The firmware ramps the duty of BSW PWM channel 0x14 (LTCA2 LTC56 quad)
//! every 40 ms; the board's RC node behind LTC59 feeds ADC0.CH14 (VP_MC_AN_DIG_MON) and the MU's own
//! converter (VP_MU_AN_DIG_MON). The alive check, the range check and the MC/MU comparison must stay
//! error-free: a stuck node (no RC path, or a PWM stuck at its boot duty) sets LV_ERR_VP_ALV_CHK_MON after
//! ten samples and the fault reaction cuts every cylinder (docs/SIMOS18_GPTA_IO_RE.md §12.3).
//! Skipped when the image is not present.

mod common;

use simemu_tc1791::{CpuBackend, Simos18Board};

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;
/// SC800S50 RAM (read only): VP_MC_AN_DIG_MON / VP_MU_AN_DIG_MON (u16, 152.588 µV/bit, ADC sync 0x8006C490),
/// LV_ERR_CONV_MON / LV_ERR_VP_ALV_CHK_MON / LV_ERR_VP_RNG_MON (A/D monitoring 0x80073C00).
const VP_MC: u32 = 0xD001_CB24;
const VP_MU: u32 = 0xD001_CB26;
const ERR_CONV: u32 = 0xD001_C033;
const ERR_ALV: u32 = 0xD001_C097;
const ERR_RNG: u32 = 0xD001_C098;
/// C_VP_ACQ_CONV_HYS_MON (0x8080FDA0) and C_VP_MAX_THD_RNG_MON (0x8080FDB4) of SC8S50.
const HYS: u16 = 0x07FD;
const RNG_MAX: u16 = 0x50A4;

#[test]
fn s50_adc_monitoring_sees_a_live_test_voltage() {
    for backend in CpuBackend::matrix() {
        run(backend);
    }
}

fn find(name: &str) -> Option<Vec<u8>> {
    common::find_image(name)
}

fn run(backend: CpuBackend) {
    let (Some(flash), Some(eeprom)) = (find(FACTORY), find(EEPROM)) else {
        eprintln!("skipped: {FACTORY} not found");
        return;
    };
    let mut b = Simos18Board::new_on(&flash, &eeprom, backend).unwrap();
    b.set_adc_input_voltage(2, 4, 2.5).unwrap(); // KL15 sense (see boot_regression.rs)
    let rd16 = |b: &Simos18Board, a: u32| b.read_mem(a, 2).map(|v| u16::from_le_bytes([v[0], v[1]])).unwrap();
    let rd8 = |b: &Simos18Board, a: u32| b.read_mem(a, 1).unwrap()[0];
    let mut seen = Vec::new();
    for _ in 0..30 {
        b.run_for(0.05).unwrap();
        assert_eq!((rd8(&b, ERR_CONV), rd8(&b, ERR_ALV), rd8(&b, ERR_RNG)), (0, 0, 0), "A/D monitoring error, VP {:?}", seen.last());
        seen.push((rd16(&b, VP_MC), rd16(&b, VP_MU)));
    }
    eprintln!("{backend:?}: VP_MC/VP_MU {seen:04X?}");
    let mut levels: Vec<u16> = seen.iter().map(|s| s.0).collect();
    levels.sort_unstable();
    levels.dedup();
    assert!(levels.len() > 10 && levels.last().unwrap() - levels[0] > 0x2000, "test voltage alive: {levels:04X?}");
    for &(mc, mu) in &seen[4..] {
        assert!(mc.abs_diff(mu) < HYS && mc <= RNG_MAX, "MC {mc:#06x} MU {mu:#06x}");
    }
    assert_eq!(b.hw().reset_count, 0);
}
