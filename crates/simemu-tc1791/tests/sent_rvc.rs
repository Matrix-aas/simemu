//! End-to-end: the unmodified SC800S50 firmware receives the rotary coolant valve position sensor over its SENT module
//! (`simemu_tc1791::sent`). Before the module was emulated nothing fed SENT channel 0 and the firmware set RVC_DIG_COM
//! (DTC 0x3C70). Only RAM is read. Skipped when the image is not present.

mod common;

use simemu_tc1791::sent::SentSensor;
use simemu_tc1791::{CpuBackend, Simos18Board};

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;
/// ENTE 34.22 (`0x8017DD30`): DIG_SIG_RVC (u16, from INFR GetDigRvcSens `0x800A5D9C` = D1..D3 of the SENT frame) and
/// STATE_DIAG_RVC_SENS (bit 0 sensor error = status nibble bit 0, bit 1 CRC, bit 2 communication).
const DIG_SIG_RVC: u32 = 0xD001_24B2;
const STATE_DIAG_RVC_SENS: u32 = 0xD000_C6E4;
/// SENT driver state of logical channel 0 (`*(0xD0000864)`): +0x10/+0x14 RSR copies, +0x18/+0x1C RDR copies.
const DRV_CH0: u32 = 0xD000_6CFC;

#[test]
fn s50_reads_the_rotary_valve_position_over_sent() {
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
    // RAM addresses above are SC800S50-specific (GetDigRvcSens 0x800A5D9C)
    assert_eq!(&flash[0xA_5D9C..0xA_5DA0], &[0x20, 0x08, 0x40, 0x4f]);
    let mut b = Simos18Board::new_on(&flash, &eeprom, backend).unwrap();
    b.set_adc_input_voltage(2, 4, 2.5).unwrap(); // KL15 sense (see boot_regression.rs)
    let value: u16 = 0x5A3;
    let d = [(value >> 8) as u8 & 0xF, (value >> 4) as u8 & 0xF, value as u8 & 0xF, 0xC, 0x5, 0xA];
    b.set_sent_sensor(0, Some(SentSensor { tick_ns: 3000.0, status: 0, data: d }));
    for _ in 0..30 {
        b.run_for(0.05).unwrap();
    }
    let rd = |a: u32, n: usize| b.read_mem(a, n).unwrap();
    let ch = &b.sent().ch[0];
    eprintln!("{backend:?}: frames ok {} err {}, driver {:02X?}", ch.frames_ok, ch.frames_err, rd(DRV_CH0, 0x20));
    // ~1.6 kHz frames at a 3 µs tick, all accepted, and the data ISR keeps up (RSI acknowledged)
    assert!(ch.frames_ok > 2000 && ch.frames_err == 0, "{backend:?}: {} / {}", ch.frames_ok, ch.frames_err);
    assert_eq!(ch.intstat & 1, 0, "{backend:?}: receive-success flag cleared by the firmware's ISR");
    assert_eq!(u16::from_le_bytes(rd(DIG_SIG_RVC, 2).try_into().unwrap()), value, "{backend:?}: DIG_SIG_RVC");
    assert_eq!(rd(STATE_DIAG_RVC_SENS, 1)[0], 0, "{backend:?}: no sensor/CRC/communication error");
    assert_eq!(b.hw().reset_count, 0);
}
