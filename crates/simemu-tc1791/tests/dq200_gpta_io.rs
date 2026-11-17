//! DQ200_G2 (TC1784) on the GPTA0/LTCA2 model: timer callbacks (UDS), WFS5 challenge/answer,
//! speed-sensor and clutch-PWM captures decoded by the unmodified firmware (run with `--release`).
//!
//! Firmware RAM is only read (observation of the firmware's own decode results).

mod common;

use simemu_tc1791::tcu_dq200::{SPEED_SENSOR_TEETH, SOLENOID_PWM_CELLS};
use simemu_tc1791::{CanFrame, CpuBackend, Dq200TcuBoard, Wfs5ImmoPartner};

const FLASH: &str = common::DQ200_FLASH;
const EEPROM: &str = common::DQ200_EEPROM;

fn image(name: &str) -> Option<Vec<u8>> {
    let r = common::find_image(name);
    if r.is_none() {
        eprintln!("SKIP: {name} not found");
    }
    r
}

fn u16_at(t: &Dq200TcuBoard, a: u32) -> u16 {
    let d = &t.cpu.hw().dspr;
    let o = (a & 0xF_FFFF) as usize;
    u16::from_le_bytes([d[o], d[o + 1]])
}

#[test]
fn dq200_gpta_timers_wfs5_and_captures() {
    for backend in CpuBackend::matrix() {
        dq200_gpta_timers_wfs5_and_captures_on(backend);
    }
}

fn dq200_gpta_timers_wfs5_and_captures_on(backend: CpuBackend) {
    let (Some(f), Some(e)) = (image(FLASH), image(EEPROM)) else { return };
    let mut t = Dq200TcuBoard::new_on(&f, &e, backend).unwrap();
    let rpm = [1200.0f32, 900.0, 1500.0]; // sensors 0, 1, 2
    let duty = [simemu_tc1791::tcu_dq200::clutch_travel_duty(3.0), simemu_tc1791::tcu_dq200::clutch_travel_duty(12.0)];
    t.inject_full_mechatronic_plant(13.8, 50.0, 40.0, [0.0; 4], 3.0, 12.0, rpm[2], rpm[0], rpm[1]);
    let immo = Wfs5ImmoPartner::from_dq200_images(&f, &e);
    let (mut challenges, mut answered) = (0, 0);
    for _ in 0..150 {
        t.step_realtime_frame(0.010).unwrap();
        for fr in t.cpu.take_can_tx() {
            if fr.arb_id == 0x012 {
                challenges += 1;
                assert!(immo.verify_challenge(&fr.data), "0x012 MAC/ID_tcu mismatch: {:02X?}", fr.data);
                let ans = CanFrame { node: 0, mo_index: 0, arb_id: 0x013, extended: false, dlc: 8, data: immo.answer_challenge(&fr.data), timestamp_us: 0 };
                answered += t.cpu.inject_can_rx(&ans);
            }
        }
    }
    let live = t.read_live_state();
    println!("WFS5: {challenges} challenges, {answered} answers accepted by a receive MO, authorised {}", live.wfs5_authorized);
    assert!(challenges > 0, "WFS5 challenge 0x012 never transmitted");
    assert!(answered > 0);
    assert!(live.wfs5_authorized && challenges <= 2, "valid 0x013 not accepted (state {:X})", live.wfs5_state);

    // Speed sensors: the firmware's 0.1 rpm values (0x800C2E2A.., stored at 0xD0007536 + 4 * sensor).
    for s in 0..3 {
        let v = u16_at(&t, 0xD000_7536 + 4 * s as u32) as f64;
        println!("speed sensor {s} ({} teeth): driven {} rpm, firmware {:.1} rpm", SPEED_SENSOR_TEETH[s], rpm[s], v / 10.0);
        assert!((v / 10.0 - rpm[s] as f64).abs() < 0.01 * rpm[s] as f64, "sensor {s}: {v}");
    }
    // Clutch PWM sensors: duty * 10000 per Icu channel (0x800F1720 -> 0xD000B294).
    for k in 0..2 {
        let (a, b) = (u16_at(&t, 0xD000_B294 + 4 * k), u16_at(&t, 0xD000_B296 + 4 * k));
        println!("clutch sensor {k}: driven duty {:.4}, firmware A {a} B {b}", duty[k as usize]);
        assert!((a as f64 - 10000.0 * duty[k as usize]).abs() <= 20.0 && (a as u32 + b as u32).abs_diff(10000) <= 20);
    }

    // Timer callbacks (GPT channel 0 = GTC30 on GT1 -> 0x800EB630) run the diagnostic CAN poll.
    assert_eq!(t.uds_request(&[0x10, 0x03], 0.3).unwrap().as_deref().map(|r| r[0]), Some(0x50));
    let r = t.uds_request(&[0x19, 0x02, 0xFF], 0.5).unwrap().expect("no 19 02 answer");
    assert_eq!(&r[..2], &[0x59, 0x02]);
    for d in r[3..].chunks(4).filter(|c| c.len() == 4) {
        println!("DTC {:02X}{:02X}{:02X} status 0x{:02X}", d[0], d[1], d[2], d[3]);
    }
    let out = t.decode_outputs();
    println!("PWM outputs: {:?}", out.pwm);
    assert_eq!(out.pwm.iter().filter(|p| p.0.contains("solenoid")).count(), SOLENOID_PWM_CELLS.len());

    // DTC numbers -> SAE codes through the firmware's own event tables (CAL 0x80032AD0).
    assert_eq!(t.dtc_sae_code(0x0029FE).as_deref(), Some("P0715"));
    assert_eq!(t.dtc_sae_code(0x002A0B).as_deref(), Some("U0103"));
    // While the safety software holds its shut-off it forces the valve pins to input: no valve may
    // see its PWM and no coil current may flow, whatever the Pwm cells do.
    let live = t.read_live_state();
    println!("safety word {:08X}, errors {}", live.safety_word, live.safety_errors);
    if live.safety_word != 0x5555_5555 {
        assert!(out.solenoid_duty.iter().all(Option::is_none), "{:?}", out.solenoid_duty);
        assert!(t.coil_currents().iter().all(|&i| i == 0.0), "{:?}", t.coil_currents());
    }
}
