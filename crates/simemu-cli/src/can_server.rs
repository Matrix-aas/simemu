//! Standalone headless SIMOS18.1 SLCAN-over-TCP bench server (`simemu can-server`).
//!
//! Runs an unmodified SIMOS18.1 firmware image on a virtual bench harness over loopback SLCAN TCP
//! (`127.0.0.1:<port>`, default `28750`) with:
//! - automatic in-memory SBOOT completion for FRF/OBD reads and `apply_post_programming` checksum/marker setup,
//! - laboratory WFS5 immobiliser binding + virtual instrument cluster on `0x010` → `0x011`
//!   (`SIMEMU_CAN_CLUSTER=off` disables the cluster; `SIMEMU_IMMO=dump` uses dump credentials),
//! - bench harness supply voltages: `KL15` (`ADC2.CH4`), `KL87` (`ADC2.CH5`) and 5 V sensor references
//!   `VCC_A/B/C` (`ADC0.CH1..3`),
//! - 250 µs sub-millisecond CAN pacing (`4 × 250 µs` per 1 ms tick while RX frames are queued, injecting at
//!   most 1 RX frame per 250 µs ≈ 4000 fps = physical 500 kbps CAN wire rate) so back-to-back ISO-TP
//!   transfers run 4× faster without MultiCAN `MSGLST` overwrites,
//! - persistent multi-client / reconnect support (clients may attach and detach without stopping the ECU).

use crate::immo_lab;
use crate::paths::{self, DEFAULT_ECU_FLASH};
use crate::slcan::SlcanServer;
use simemu_tc1791::{CanFrame, Simos18Board, Wfs5ImmoPartner};
use std::fs;
use std::io::Write;
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

/// Standard regulated bench power-supply voltage (V).
const BENCH_SUPPLY_V: f64 = 13.8;
/// SIMOS18.1 KL15 / KL87 voltage-divider ratio (`FR p.23942 NC_V_IGK_BRI_RATIO`).
const KL15_DIVIDER: f64 = 5.701171875;
/// 5 V sensor reference supplies A/B/C (`ADC0.CH1..3`, 2:1 divider -> 2.5 V; `FR` `c_vcc_pvs_min/max_mon`).
const VCC_REF_V: f32 = 2.5;

const TICK_WALL: Duration = Duration::from_millis(1);
const IDLE_TICK_S: f64 = 0.001;
const SUB_QUANTA_PER_TICK: usize = 4;
const SUB_QUANTUM_S: f64 = 0.000_25;

/// Wires bench harness supply voltages onto the SIMOS18.1 ADC inputs (`KL15`, `KL87`, and `VCC_0..2`).
pub fn configure_bench_analog_inputs(board: &mut Simos18Board) -> Result<(), String> {
    let supply_sense_v = (BENCH_SUPPLY_V / KL15_DIVIDER) as f32;
    board.set_adc_input_voltage(2, 4, supply_sense_v)?; // KL15 (V_IGK)
    board.set_adc_input_voltage(2, 5, supply_sense_v)?; // KL87 (V_RLY_MAIN_MES_H_RES)
    for ch in 1..=3 {
        board.set_adc_input_voltage(0, ch, VCC_REF_V)?; // 5 V sensor reference supplies A/B/C
    }
    Ok(())
}

fn drain_board_tx(
    board: &mut Simos18Board,
    server: &mut SlcanServer,
    cluster: Option<&Wfs5ImmoPartner>,
) {
    let now_us = (board.hw().sim_time_s * 1e6) as u64;
    for mut frame in board.take_can_tx() {
        if frame.timestamp_us == 0 {
            frame.timestamp_us = now_us;
        }
        server.broadcast(&frame, None);
        if frame.arb_id == 0x010 && frame.dlc == 8 {
            if let Some(partner) = cluster {
                let answer = CanFrame {
                    node: 0,
                    mo_index: 0,
                    arb_id: 0x011,
                    extended: false,
                    dlc: 8,
                    data: partner.answer_challenge(&frame.data),
                    timestamp_us: now_us,
                };
                board.inject_can_rx(&answer);
                server.broadcast(&answer, None);
            }
        }
    }
}

/// Advances one 1 ms bench server tick: runs `4 × 250 µs` sub-quanta (at most 1 RX frame injected per
/// 250 µs) when external SLCAN frames are queued, or a single `1 ms` step when idle.
pub fn step_can_server_tick(
    board: &mut Simos18Board,
    server: &mut SlcanServer,
    cluster: Option<&Wfs5ImmoPartner>,
) -> Result<usize, String> {
    server.poll();
    let mut injected = 0usize;
    if server.has_pending_rx() {
        for _ in 0..SUB_QUANTA_PER_TICK {
            server.poll();
            if let Some((sender_id, mut rx_frame)) = server.pop_rx() {
                rx_frame.timestamp_us = (board.hw().sim_time_s * 1e6) as u64;
                server.broadcast(&rx_frame, Some(sender_id));
                board.inject_can_rx(&rx_frame);
                injected += 1;
            }
            board.run_for(SUB_QUANTUM_S)?;
            drain_board_tx(board, server, cluster);
        }
    } else {
        board.run_for(IDLE_TICK_S)?;
        drain_board_tx(board, server, cluster);
    }
    server.flush();
    Ok(injected)
}

pub fn cmd_can_server(
    root: &Path,
    port: u16,
    image: Option<&str>,
    eeprom: Option<&str>,
) -> Result<(), String> {
    let flash_path = paths::resolve_firmware_or_err(root, image.unwrap_or(DEFAULT_ECU_FLASH))?;
    let (eeprom_path, raw_eeprom) = match eeprom {
        Some(explicit) => {
            let p = paths::resolve_firmware_or_err(root, explicit)?;
            let mut b = fs::read(&p).map_err(|e| format!("{}: {e}", p.display()))?;
            if b.len() == 0x01_8000 {
                b.resize(0x04_0000, 0x00);
            }
            (p, b)
        }
        None => paths::read_dflash_or_default(root)?,
    };

    let mut flash = paths::read_firmware_from_path(root, &flash_path)?;
    if flash.len() == 0x40_0000 && flash[..0x1_C000].iter().all(|&b| b == 0 || b == 0xFF) {
        let (donor_path, donor) = paths::read_sboot_donor(root)?;
        flash[..0x1_C000].copy_from_slice(&donor[..0x1_C000]);
        if simemu_damos::is_ru() {
            eprintln!("SBOOT подставлен из {} (только в ОЗУ)", donor_path.display());
        } else {
            eprintln!("SBOOT supplied from {} (in memory only)", donor_path.display());
        }
    }
    let notes = simemu_damos::apply_post_programming(&mut flash)?;
    if !notes.is_empty() {
        if simemu_damos::is_ru() {
            eprintln!("Образ дозавершён после программирования (исходный файл не изменён): {notes:?}");
        } else {
            eprintln!("Image completed after programming (source file unchanged): {notes:?}");
        }
    }

    let (nvm, cluster) = if std::env::var("SIMEMU_CAN_CLUSTER").as_deref() == Ok("off") {
        (raw_eeprom, None)
    } else {
        let use_lab = std::env::var("SIMEMU_IMMO").as_deref() != Ok("dump");
        let binding = immo_lab::ecu_binding(use_lab, &flash, &raw_eeprom);
        for note in &binding.notes {
            eprintln!("  {note}");
        }
        (binding.nvm, binding.cluster)
    };

    let mut board = Simos18Board::new(&flash, &nvm)?;
    configure_bench_analog_inputs(&mut board)?;

    let mut server = SlcanServer::bind(port, true)?;
    println!(
        "SLCAN TCP 127.0.0.1:{}; image={}; EEPROM={}; CPU={:?}",
        server.port(),
        flash_path.display(),
        eeprom_path.display(),
        board.backend()
    );
    std::io::stdout().flush().map_err(|e| e.to_string())?;

    loop {
        let started = Instant::now();
        if let Err(e) = step_can_server_tick(&mut board, &mut server, cluster.as_ref()) {
            eprintln!("CPU error: {e}");
            return Err(e);
        }
        let elapsed = started.elapsed();
        if elapsed < TICK_WALL {
            thread::sleep(TICK_WALL - elapsed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::paths::DEFAULT_ECU_EEPROM;
    use std::net::TcpStream;

    #[test]
    fn sub_quantum_pacing_caps_rx_injection_to_four_frames_per_ms() {
        let Some(root) = paths::test_workspace_root() else {
            eprintln!("SKIP: {DEFAULT_ECU_FLASH} not found");
            return;
        };
        let flash = paths::read_firmware(&root, DEFAULT_ECU_FLASH).unwrap();
        let eeprom = paths::read_firmware(&root, DEFAULT_ECU_EEPROM).unwrap();
        let mut board = Simos18Board::new(&flash, &eeprom).unwrap();
        configure_bench_analog_inputs(&mut board).unwrap();

        // Check bench analog supply voltages (KL15, KL87, and 5V sensor refs A/B/C)
        let expected_igk = (BENCH_SUPPLY_V / KL15_DIVIDER) as f32;
        assert!((board.hw().adc.volts[2][4] - expected_igk).abs() < 1e-5);
        assert!((board.hw().adc.volts[2][5] - expected_igk).abs() < 1e-5);
        for ch in 1..=3 {
            assert!((board.hw().adc.volts[0][ch] - 2.5).abs() < 1e-5);
        }

        let mut server = SlcanServer::bind(0, false).unwrap();
        let mut client = TcpStream::connect(("127.0.0.1", server.port())).unwrap();
        // Enqueue 6 back-to-back CAN frames in a single burst
        for i in 0..6u8 {
            write!(client, "t7E08023E{:02X}5555555555\r", i).unwrap();
        }
        client.flush().unwrap();
        // Wait briefly for loopback delivery into kernel socket buffer
        for _ in 0..10 {
            server.poll();
            if server.has_pending_rx() {
                break;
            }
            thread::sleep(Duration::from_millis(2));
        }
        let t0 = board.hw().sim_time_s;
        let n1 = step_can_server_tick(&mut board, &mut server, None).unwrap();
        assert_eq!(n1, 4, "first 1 ms tick must inject at most 4 frames (1 per 250 us)");
        assert!((board.hw().sim_time_s - t0 - 0.001).abs() < 5e-5);

        let n2 = step_can_server_tick(&mut board, &mut server, None).unwrap();
        assert_eq!(n2, 2, "second 1 ms tick drains remaining 2 frames");
    }

    #[test]
    fn live_ecu_answers_wfs5_and_multiframe_uds_over_slcan() {
        use simemu_tc1791::CpuBackend;
        use std::io::Read;

        let Some(root) = paths::test_workspace_root() else {
            eprintln!("SKIP: {DEFAULT_ECU_FLASH} not found");
            return;
        };
        let flash = paths::read_firmware(&root, DEFAULT_ECU_FLASH).unwrap();
        let raw_eeprom = paths::read_firmware(&root, DEFAULT_ECU_EEPROM).unwrap();
        let binding = immo_lab::ecu_binding(true, &flash, &raw_eeprom);
        let mut board = Simos18Board::new_on(&flash, &binding.nvm, CpuBackend::Native).unwrap();
        configure_bench_analog_inputs(&mut board).unwrap();

        let mut server = SlcanServer::bind(0, false).unwrap();
        let mut client = TcpStream::connect(("127.0.0.1", server.port())).unwrap();
        client.set_nonblocking(true).unwrap();
        client.write_all(b"S6\rO\r").unwrap();
        client.flush().unwrap();

        let mut raw_rx = Vec::<u8>::new();
        let mut scratch = [0u8; 4096];
        let mut rx_payload = Vec::new();
        let mut expected_len = None;
        let mut sent_req = false;
        let mut sent_fc = false;
        let mut saw_wfs5_answer = false;

        for tick in 0..800 {
            if tick == 450 && !sent_req {
                // Send UDS 22 F1 87 (VW Spare Part Number -> multi-frame ISO-TP response)
                client.write_all(b"t7E080322F18755555555\r").unwrap();
                client.flush().unwrap();
                sent_req = true;
            }
            step_can_server_tick(&mut board, &mut server, binding.cluster.as_ref()).unwrap();
            loop {
                match client.read(&mut scratch) {
                    Ok(0) => break,
                    Ok(n) => raw_rx.extend_from_slice(&scratch[..n]),
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(e) => panic!("read error: {e}"),
                }
            }
            while let Some(pos) = raw_rx.iter().position(|&b| b == b'\r') {
                let seg_bytes: Vec<u8> = raw_rx.drain(..=pos).collect();
                let seg = std::str::from_utf8(&seg_bytes[..pos]).unwrap_or("");
                if let Ok((f, false)) = crate::slcan::parse_frame(seg) {
                    if f.arb_id == 0x011 {
                        saw_wfs5_answer = true;
                    }
                    if f.arb_id == 0x7E8 {
                        match f.data[0] >> 4 {
                            1 => {
                                let len = (((f.data[0] & 0x0F) as usize) << 8) | f.data[1] as usize;
                                expected_len = Some(len);
                                rx_payload.extend_from_slice(&f.data[2..8]);
                                if !sent_fc {
                                    client.write_all(b"t7E083000005555555555\r").unwrap();
                                    client.flush().unwrap();
                                    sent_fc = true;
                                }
                            }
                            2 => {
                                rx_payload.extend_from_slice(&f.data[1..8]);
                            }
                            _ => {}
                        }
                    }
                }
            }
            if saw_wfs5_answer && expected_len.is_some_and(|exp| rx_payload.len() >= exp) {
                rx_payload.truncate(expected_len.unwrap());
                break;
            }
        }

        assert!(saw_wfs5_answer, "SLCAN monitor must observe virtual cluster WFS5 0x011 answer");
        let exp = expected_len.expect("must receive ISO-TP FirstFrame on 0x7E8");
        assert_eq!(rx_payload.len(), exp);
        assert_eq!(&rx_payload[..3], &[0x62, 0xF1, 0x87]);
        let part_no = String::from_utf8_lossy(&rx_payload[3..]);
        assert!(
            part_no.starts_with("8V0906264H"),
            "unexpected VW spare part number from ECU: {part_no:?}"
        );
    }
}
