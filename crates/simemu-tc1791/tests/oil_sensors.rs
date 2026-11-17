//! End-to-end: the unmodified SC800S50 firmware decodes the oil level / temperature sender G266 (pin P3.4 = IN12,
//! `engine_io::LINE_G266`) driven by `simemu_physics::sensors::OilLevelSender`. Before the sender was emulated nothing
//! fed IN12 and the firmware set OIL_SENS_EL (DTC 0x3B42) / OIL_SENS_PLAUS (0x3B43). Only RAM is read.
//!
//! RAM (a0 = 0xD0018000, SC800S50 ENLU 27.3 at 0x801A36F0, BSW capture 0x8008DCBC / getter 0x8008DD98): the decoded
//! times are in 0.1 ms, normalised to the reference (`c_t_sig_ref_dft_loil` 100 ms / T_REF_SIG_LEN_LOIL).

mod common;

use simemu_physics::sensors::OilLevelSender;
use simemu_tc1791::engine_io::SensorSignal;
use simemu_tc1791::{CpuBackend, Simos18Board};

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;
/// BSW: high / low time of the last period (0.1 ms, from the G1.LTC40 period × duty), edge counter.
const BSW_T_H: u32 = 0xD001_1F38;
const BSW_T_L: u32 = 0xD001_1F3C;
const BSW_CTR: u32 = 0xD001_1F3E;
/// ENLU: T_REF_SIG_LEN_LOIL_TMP, T_TEMP_SIG_LOIL_TMP, T_COOL_LOIL_TMP (what the plausibility diagnosis 0x808DE214 checks)
/// and T_EDGE_FALL_LOIL (10 ms ticks since the last edge; electrical diagnosis limit 250).
const T_REF_TMP: u32 = 0xD001_2304;
const T_TEMP_TMP: u32 = 0xD001_230C;
const T_COOL_TMP: u32 = 0xD001_22FC;
const T_EDGE_FALL: u32 = 0xD000_C5AD;

#[test]
fn s50_decodes_the_oil_level_sender() {
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
    // RAM addresses above are SC800S50-specific (GetOilSens 0x8008DD98)
    assert_eq!(&flash[0x8_DD98..0x8_DD9C], &[0xB9, 0x0F, 0xFA, 0xC9]);
    let mut b = Simos18Board::new_on(&flash, &eeprom, backend).unwrap();
    b.set_adc_input_voltage(2, 4, 2.5).unwrap(); // KL15 sense (see boot_regression.rs)
    let (oil_c, level_mm, secs) = (90.0, 80.0, 4.0);
    let mut s = OilLevelSender::default();
    let mut e = s.step(0.0, true, oil_c, level_mm); // sender powered with the ECU
    e.extend(s.step(secs, true, oil_c, level_mm));
    let edges: Vec<_> = e.into_iter().map(|(t, l)| (t, SensorSignal::OilLevelSender(l))).collect();
    b.feed_sensor_edges(&edges);
    for _ in 0..(secs / 0.05) as usize {
        b.run_for(0.05).unwrap();
    }
    let h = |a: u32| u16::from_le_bytes(b.read_mem(a, 2).unwrap().try_into().unwrap()) as f64 * 0.1;
    let (r, tt, cool) = (h(T_REF_TMP), h(T_TEMP_TMP), h(T_COOL_TMP));
    let edge_fall = b.read_mem(T_EDGE_FALL, 1).unwrap()[0];
    eprintln!("{backend:?}: BSW H {} L {} ms, {} periods; T_REF {r} T_TEMP {tt} T_COOL {cool} ms, T_EDGE_FALL {edge_fall}", h(BSW_T_H), h(BSW_T_L), h(BSW_CTR) * 10.0);
    assert!((r - 100.0).abs() < 0.3, "{backend:?}: reference period {r}");
    assert!((tt - OilLevelSender::temp_ms(oil_c)).abs() < 0.3, "{backend:?}: temperature time {tt}");
    assert!((cool - OilLevelSender::cool_ms(level_mm)).abs() < 0.5, "{backend:?}: cool-off time {cool}");
    assert!(edge_fall < 120, "{backend:?}: edges keep arriving ({edge_fall} × 10 ms)");
    assert_eq!(b.hw().reset_count, 0);
}
