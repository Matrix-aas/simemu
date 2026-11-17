//! End-to-end: unmodified firmware <-> monitoring unit + ATIC140 (MSC) / ATIC121 / ATIC111 (SSC).
//! The switch-off path check (SOPC) must pass on its own and the power stages must end up enabled,
//! on every available SIMOS18.1 image. Images are looked up like `boot_regression.rs`; missing
//! images are skipped.

mod common;

use simemu_tc1791::atic::{Load, LoadFault, ATIC140_BRIDGED};
use simemu_tc1791::monitor_device::MuState;
use simemu_tc1791::{CpuBackend, Simos18Board};

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;

fn find(name: &str) -> Option<Vec<u8>> {
    common::find_image(name)
}

fn boot(flash: &[u8], eeprom: &[u8], secs: f64, backend: CpuBackend) -> Simos18Board {
    let mut b = Simos18Board::new_on(flash, eeprom, backend).unwrap();
    b.set_adc_input_voltage(2, 4, 2.5).unwrap(); // supply sense (see boot_regression.rs)
    for _ in 0..(secs / 0.05) as u32 {
        b.run_for(0.05).unwrap();
    }
    b
}

fn assert_enabled(name: &str, b: &Simos18Board) {
    let hw = b.hw();
    let out = b.power_stage_outputs();
    assert_eq!(hw.reset_count, 0, "{name}: resets {:?} at {:08X?}", hw.reset_log, hw.swrst_log);
    assert_eq!(hw.spi.mu.state(), MuState::Norm, "{name}");
    // MU lines high and LS_DIS0 released: injection/ignition stages enabled. MTC_DIS (throttle)
    // and EN7_8 (starter) stay under functional control of the software after SOPC (no engine
    // start requested, no throttle sensor inputs on this bench), so they are only reported.
    let l = out.lines;
    assert!(l.ndis0 && l.ndis1 && !l.ls_dis0, "{name}: disable lines {l:?}");
    assert!(out.injectors_enabled, "{name}");
    eprintln!("{name}: {l:?}");
    let ps = &hw.spi.ps;
    assert!(ps.atic140.iter().all(|c| c.dumps > 0 && c.rejected == 0), "{name}: ATIC140 dumps/rejects");
    assert!(ps.atic111.frames > 0 && ps.atic121.words > 0 && ps.atic121.parity_errors == 0, "{name}");
}

#[test]
fn s50_switch_off_path_check_passes_and_atic_diagnosis_reports_load_faults() {
    let (Some(flash), Some(eeprom)) = (find(FACTORY), find(EEPROM)) else {
        eprintln!("skipped: {FACTORY} not found");
        return;
    };
    // SC800S50 addresses below are only valid for these bytes (SOPC final check 0x80080F2C).
    assert_eq!(&flash[0x8_0F2C..0x8_0F34], &[0x02, 0x48, 0x82, 0x09, 0x91, 0x00, 0x00, 0xfb]);
    for backend in CpuBackend::matrix() {
        sopc_and_load_faults(&flash, &eeprom, backend);
    }
}

fn sopc_and_load_faults(flash: &[u8], eeprom: &[u8], backend: CpuBackend) {
    let mut b = boot(flash, eeprom, 1.0, backend);
    assert_enabled("SC800S50", &b);
    // LF_ECM3_SOPC_RESP (complement-protected, 0xB0001E14/18): all 15 paths switch off and on.
    let resp = b.read_mem(0xB000_1E14, 8).unwrap();
    let (v, c) = (u32::from_le_bytes(resp[..4].try_into().unwrap()), u32::from_le_bytes(resp[4..].try_into().unwrap()));
    assert_eq!((v, c), (0x7FFF, !0x7FFF), "SOPC result");
    // ATIC140 #1 ignition diagnosis (struct 0xD0008164: +0x24 SCB / +0x2C SCG / +0x34 OL present,
    // +0x28/+0x30/+0x38 valid; getter 0x800A9F68): healthy, then wire faults on channels 1..3.
    let rd = |b: &Simos18Board, off: u32| u32::from_le_bytes(b.read_mem(0xD000_8164 + off, 4).unwrap().try_into().unwrap());
    // The HPFP valve coil bridged between stages 25 (HS) and 24 (LS) reads open load on both when intact — the VCV
    // diagnosis' good case (FR p.12179); every other stage reports no fault.
    let vcv = ATIC140_BRIDGED[1];
    assert_eq!([rd(&b, 0x24), rd(&b, 0x2C), rd(&b, 0x34) & !vcv, rd(&b, 0x34) & vcv], [0, 0, 0, vcv], "no fault present");
    assert_eq!(rd(&b, 0x38) & 0xE, 0xE, "open-load diagnosis valid");
    for (ch, f) in [(1, LoadFault::ShortToBattery), (2, LoadFault::ShortToGround), (3, LoadFault::OpenLoad)] {
        b.set_power_stage_load(1, ch, Load { fault: f, ..Load::new(10.0, 0.01) }).unwrap();
    }
    b.run_for(0.05).unwrap();
    assert_eq!([rd(&b, 0x24) & 0xE, rd(&b, 0x2C) & 0xE, rd(&b, 0x34) & 0xE], [2, 4, 8], "faults filed per channel");
    assert_eq!(b.hw().reset_count, 0);
}

#[test]
fn all_images_pass_sopc_and_enable_the_power_stages() {
    let Some(eeprom) = find(EEPROM) else {
        eprintln!("skipped: EEPROM dump not found");
        return;
    };
    let images = [FACTORY, "8V0906264K__0003.bin", "8V0906264H__0005.bin", "8V0906264J__0003.bin", "8V0906264F__0003.bin"];
    let mut ran = 0;
    for name in images {
        let Some(flash) = find(name) else { continue };
        for backend in CpuBackend::matrix() {
            let b = boot(&flash, &eeprom, 1.0, backend);
            assert_enabled(&format!("{name} [{backend:?}]"), &b);
            ran += 1;
        }
    }
    eprintln!("{ran} images checked");
}
