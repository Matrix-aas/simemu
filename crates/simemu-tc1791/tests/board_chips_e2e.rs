//! End-to-end: the unmodified SC800S50 firmware accepts the ATIC139 LS/PS function blocks (SSC1/SLSO2)
//! and the PIC12LF1822 engine-off timer (SSC0/SLSO6) of the board (docs/SIMOS18_BOARD_HW.md). Before
//! these models both answered 0: MCC_ATIC_139_LS/PS were set after the first frames and the PIC data
//! was invalid (STATE_T_ES_VLD_RD = 4). Only RAM is read. Skipped when the image is not present.

mod common;

use simemu_tc1791::{CpuBackend, Simos18Board};

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;
/// ERRM error status, 2 bytes per IDX_ERR (writer 0x800947C8: byte 1 bit 0 = reported, bit 1 = failed).
const ERR_STATUS: u32 = 0xD001_25E4;
/// IDX_ERR of MCC_ATIC_139_LS / _PS (position in the C_ERR_CLAS_* table, SC8S50.json).
const MCC_ATIC_139: [u32; 2] = [0xA6, 0xA7];
/// Driver frame states (3 = frame accepted, 2 = communication error): LS +0x1C (0x800A9C10), PS +0x18
/// (0x800A9724).
const LS_STATE: u32 = 0xD000_A118 + 0x1C;
const PS_STATE: u32 = 0xD000_B258 + 0x18;
/// PIC driver struct 0xD000B288: clock value u32 +0x00 (parser 0x800ABD40); ASW copy of the validity
/// STATE_T_ES_VLD_RD (getter 0x800ABCF0 -> 0x801A83F4).
const PIC_VALUE: u32 = 0xD000_B288;
const T_ES_VLD_RD: u32 = 0xD000_C6D3;

#[test]
fn s50_accepts_atic139_ls_ps_and_the_engine_off_timer() {
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
    // The RAM addresses above are only valid for these bytes (PIC parser 0x800ABD40, LS parser 0x800A9BE8).
    assert_eq!(&flash[0xA_BD40..0xA_BD48], &[0x85, 0xdf, 0x50, 0xf8, 0x82, 0x65, 0xd9, 0x9c]);
    assert_eq!(&flash[0xA_9BE8..0xA_9BF0], &[0xd9, 0x9d, 0x18, 0x4e, 0x09, 0xdf, 0x59, 0x08]);
    let mut b = Simos18Board::new_on(&flash, &eeprom, backend).unwrap();
    b.set_adc_input_voltage(2, 4, 2.5).unwrap(); // KL15 sense (see boot_regression.rs)
    let rd = |b: &Simos18Board, a: u32, n: usize| b.read_mem(a, n).unwrap();
    let u32_at = |b: &Simos18Board, a: u32| u32::from_le_bytes(rd(b, a, 4).try_into().unwrap());
    let sec = |b: &Simos18Board| b.hw().now_ns() as f64 * 1e-9;
    for _ in 0..20 {
        b.run_for(0.05).unwrap();
    }
    let (t0, v0) = (sec(&b), u32_at(&b, PIC_VALUE));
    for _ in 0..40 {
        b.run_for(0.05).unwrap();
    }
    let (t1, v1) = (sec(&b), u32_at(&b, PIC_VALUE));
    eprintln!("{backend:?}: PIC clock {v0} s at {t0:.2} s, {v1} s at {t1:.2} s; {:?}", b.hw().spi.ps.eot);

    for idx in MCC_ATIC_139 {
        let st = rd(&b, ERR_STATUS + 2 * idx, 2)[1];
        assert_eq!(st & 3, 1, "{backend:?}: IDX_ERR {idx:#x} reported without failure (status {st:#04x})");
    }
    assert_eq!((rd(&b, LS_STATE, 1)[0], rd(&b, PS_STATE, 1)[0]), (3, 3), "{backend:?}: LS/PS frames accepted");
    let c = &b.hw().spi.ps.atic139;
    assert!(c.ls_status > 100 && c.ps_status > 100 && c.parity_errors == 0, "{backend:?}: {c:?}");
    assert_eq!(rd(&b, T_ES_VLD_RD, 1)[0], 1, "{backend:?}: engine-off timer valid");
    let dv = v1 as f64 - v0 as f64;
    assert!((dv - (t1 - t0)).abs() <= 1.0, "{backend:?}: firmware reads a 1 Hz clock ({dv} s in {:.2} s)", t1 - t0);
    assert_eq!(b.hw().reset_count, 0);
}
