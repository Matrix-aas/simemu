//! Vehicle bus and external co-simulation connector hub:
//! - **MQB Rest-Bus ([`mqb`])**: Encodes drivetrain CAN frames of non-emulated vehicle modules
//!   (ESP, steering angle sensor, airbag, gateway, energy manager, instrument cluster, BCM, climate,
//!   driver door, ACC/stalk, selector lever) strictly from physical plant quantities and driver inputs
//!   (Zero-Fake-Logic: never synthesises ASR/ESP/ACC control interventions).
//! - **SIMOS18.1 CAN Descriptor Parser ([`Simos18CanDescriptor`])**: Parses the 48-byte RX and 44-byte
//!   TX CAN frame descriptor tables from `SC800S50` `PFlash0`.
//! - **BeamNG.drive UDP Link ([`BeamNgUdpBridge`])**: Non-blocking JSON-over-UDP actuator/telemetry
//!   transport with stale/duplicate packet rejection and peer-restart sequence recovery.
//!
//! Table direction note (`SC800S50`): the 44-byte table at `0x8005C038` contains the MQB IDs the engine
//! ECU is known to *transmit* (`Motor_11` `0x0A7`, `Motor_12` `0x0A8`, `Motor_20` `0x121`, `Motor_14`
//! `0x3BE`, `Motor_07` `0x640`, `Motor_Code_01` `0x641`), and the 48-byte table at `0x8005BBE4` contains
//! IDs it *receives* (`Klemmen_Status_01` `0x3C0` with DLC 4, `Kombi_01` `0x30B`, `LWI_01` `0x086`).
//! [`Simos18CanDescriptor::is_rx`] follows that verified direction.

use serde::{Deserialize, Serialize};
use simemu_physics::{PlantActuators, PlantTelemetry};
use std::io::ErrorKind;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};

pub mod mqb;
pub use mqb::{mqb_crc8_p2f, ChassisInputs, MqbChassisEncoder};

/// Maximum number of UDP datagrams drained in a single [`BeamNgUdpBridge::poll_sensors`] call.
const MAX_UDP_POLL_PACKETS: usize = 256;
const UDP_RX_BUF_LEN: usize = 65_536;

/// CAN descriptor parsed from SIMOS18.1 (`SC800S50`) `PFlash0`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Simos18CanDescriptor {
    pub index: usize,
    pub is_rx: bool,
    pub table_addr: u32,
    pub bus_id: u32,
    pub can_id: u32,
    pub dlc: u8,
    pub period_raw: u16,
    pub timeout_ticks: u16,
    pub handler_fn: u32,
    pub signal_table_ptr: u32,
    pub signal_count: u32,
}

fn is_valid_pflash_code_ptr(addr: u32) -> bool {
    if (addr & 0xDF00_0001) != 0x8000_0000 {
        return false;
    }
    let low = addr & 0x0FFF_FFFF;
    low < 0x0020_0000 || (0x0080_0000..0x00A0_0000).contains(&low)
}

fn is_valid_descriptor(bus_id: u32, can_id: u32, dlc: u8, handler_fn: u32) -> bool {
    bus_id <= 16
        && dlc <= 8
        && (can_id & !0x8000_0000) <= 0x1FFF_FFFF
        && is_valid_pflash_code_ptr(handler_fn)
}

impl Simos18CanDescriptor {
    /// `SC800S50` receive descriptor table in `PFlash0` (`0x8005BBE4`, 48-byte entries, `is_rx = true`).
    pub const RX_48_TABLE_FLASH_OFFSET: usize = 0x0005_BBE4;
    pub const RX_48_ENTRY_SIZE: usize = 0x30;
    pub const RX_48_ENTRY_COUNT: usize = 23;

    /// `SC800S50` transmit descriptor table in `PFlash0` (`0x8005C038`, 44-byte entries, `is_rx = false`).
    pub const TX_44_TABLE_FLASH_OFFSET: usize = 0x0005_C038;
    pub const TX_44_ENTRY_SIZE: usize = 0x2C;
    pub const TX_44_ENTRY_COUNT: usize = 68;

    /// Legacy alias for [`Self::RX_48_TABLE_FLASH_OFFSET`] (receive table at `0x0005BBE4`).
    pub const TX_TABLE_FLASH_OFFSET: usize = Self::RX_48_TABLE_FLASH_OFFSET;
    /// Legacy alias for [`Self::RX_48_ENTRY_SIZE`].
    pub const TX_ENTRY_SIZE: usize = Self::RX_48_ENTRY_SIZE;
    /// Legacy alias for [`Self::RX_48_ENTRY_COUNT`].
    pub const TX_ENTRY_COUNT: usize = Self::RX_48_ENTRY_COUNT;

    /// Legacy alias for [`Self::TX_44_TABLE_FLASH_OFFSET`] (transmit table at `0x0005C038`).
    pub const RX_TABLE_FLASH_OFFSET: usize = Self::TX_44_TABLE_FLASH_OFFSET;
    /// Legacy alias for [`Self::TX_44_ENTRY_SIZE`].
    pub const RX_ENTRY_SIZE: usize = Self::TX_44_ENTRY_SIZE;
    /// Legacy alias for [`Self::TX_44_ENTRY_COUNT`].
    pub const RX_ENTRY_COUNT: usize = Self::TX_44_ENTRY_COUNT;

    /// Parse both `SC800S50` descriptor tables. Field `period_raw` is UNVERIFIED (may be a buffer offset).
    pub fn parse_all_from_flash(flash: &[u8]) -> Vec<Self> {
        let mut out = Vec::with_capacity(Self::RX_48_ENTRY_COUNT + Self::TX_44_ENTRY_COUNT);

        // 1. 48-byte table (frames the ECU receives)
        for idx in 0..Self::RX_48_ENTRY_COUNT {
            let off = Self::RX_48_TABLE_FLASH_OFFSET + idx * Self::RX_48_ENTRY_SIZE;
            let Some(e) = flash.get(off..off + Self::RX_48_ENTRY_SIZE) else {
                break;
            };
            let bus_id = u32::from_le_bytes([e[4], e[5], e[6], e[7]]);
            let can_id = u32::from_le_bytes([e[8], e[9], e[10], e[11]]);
            let period_raw = u16::from_le_bytes([e[12], e[13]]);
            let timeout_ticks = u16::from_le_bytes([e[14], e[15]]);
            let dlc = e[16];
            let handler_fn = u32::from_le_bytes([e[24], e[25], e[26], e[27]]);
            let signal_table_ptr = u32::from_le_bytes([e[28], e[29], e[30], e[31]]);
            let signal_count = u32::from_le_bytes([e[32], e[33], e[34], e[35]]);

            if !is_valid_descriptor(bus_id, can_id, dlc, handler_fn) {
                continue;
            }

            out.push(Self {
                index: out.len(),
                is_rx: true,
                table_addr: 0x8000_0000 + off as u32,
                bus_id,
                can_id,
                dlc,
                period_raw,
                timeout_ticks,
                handler_fn,
                signal_table_ptr,
                signal_count,
            });
        }

        // 2. 44-byte table (frames the ECU transmits)
        for idx in 0..Self::TX_44_ENTRY_COUNT {
            let off = Self::TX_44_TABLE_FLASH_OFFSET + idx * Self::TX_44_ENTRY_SIZE;
            let Some(e) = flash.get(off..off + Self::TX_44_ENTRY_SIZE) else {
                break;
            };
            let bus_id = u32::from_le_bytes([e[0], e[1], e[2], e[3]]);
            let can_id = u32::from_le_bytes([e[4], e[5], e[6], e[7]]);
            let period_raw = u16::from_le_bytes([e[8], e[9]]);
            let timeout_ticks = u16::from_le_bytes([e[10], e[11]]);
            let dlc = e[16];
            let handler_fn = u32::from_le_bytes([e[24], e[25], e[26], e[27]]);
            let signal_table_ptr = u32::from_le_bytes([e[28], e[29], e[30], e[31]]);
            let signal_count = u32::from_le_bytes([e[32], e[33], e[34], e[35]]);

            if !is_valid_descriptor(bus_id, can_id, dlc, handler_fn) {
                continue;
            }

            out.push(Self {
                index: out.len(),
                is_rx: false,
                table_addr: 0x8000_0000 + off as u32,
                bus_id,
                can_id,
                dlc,
                period_raw,
                timeout_ticks,
                handler_fn,
                signal_table_ptr,
                signal_count,
            });
        }

        out
    }
}

/// ECU → BeamNG: decoded actuator outputs.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeamNgActuatorFrame {
    pub seq: u64,
    pub ecu_stm_ticks: u64,
    pub actuators: PlantActuators,
}

/// BeamNG → ECU: the vehicle simulated by BeamNG reported in the same physical units as the
/// built-in plant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeamNgSensorFrame {
    pub seq: u64,
    pub telemetry: serde_json::Value,
}

/// JSON-over-UDP link. No lockstep: frames carry sequence numbers; stale frames are dropped.
pub struct BeamNgUdpBridge {
    socket: UdpSocket,
    remote_addr: SocketAddr,
    rx_buf: Box<[u8]>,
    seq: u64,
    last_rx_seq: Option<u64>,
}

impl BeamNgUdpBridge {
    pub fn bind(bind_addr: &str, remote_addr: &str) -> Result<Self, String> {
        let remote = remote_addr
            .to_socket_addrs()
            .map_err(|e| format!("resolve {remote_addr}: {e}"))?
            .next()
            .ok_or_else(|| format!("resolve {remote_addr}: no socket address"))?;
        let socket = UdpSocket::bind(bind_addr).map_err(|e| format!("bind {bind_addr}: {e}"))?;
        socket.set_nonblocking(true).map_err(|e| e.to_string())?;
        Ok(Self {
            socket,
            remote_addr: remote,
            rx_buf: vec![0u8; UDP_RX_BUF_LEN].into_boxed_slice(),
            seq: 0,
            last_rx_seq: None,
        })
    }

    /// Local socket address bound by this bridge (useful when binding ephemeral port `0`).
    pub fn local_addr(&self) -> Result<SocketAddr, String> {
        self.socket.local_addr().map_err(|e| e.to_string())
    }

    /// Resets incoming sequence-number tracking so any next packet is accepted.
    pub fn reset_rx_seq(&mut self) {
        self.last_rx_seq = None;
    }

    fn send_datagram(&self, payload: &[u8]) -> Result<(), String> {
        match self.socket.send_to(payload, self.remote_addr) {
            Ok(_) => Ok(()),
            // Non-blocking UDP send buffer full or Windows ICMP Port Unreachable (`WSAECONNRESET`)
            // when the BeamNG peer is not listening yet: drop datagram cleanly without failing.
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::ConnectionReset) => {
                Ok(())
            }
            Err(e) => Err(format!("send: {e}")),
        }
    }

    pub fn send_actuators(&mut self, stm_ticks: u64, actuators: &PlantActuators) -> Result<(), String> {
        self.seq = self.seq.wrapping_add(1);
        let frame = BeamNgActuatorFrame {
            seq: self.seq,
            ecu_stm_ticks: stm_ticks,
            actuators: actuators.clone(),
        };
        let payload = serde_json::to_vec(&frame).map_err(|e| e.to_string())?;
        self.send_datagram(&payload)
    }

    /// Returns the newest pending sensor frame, dropping older/duplicate ones.
    pub fn poll_sensors(&mut self) -> Option<BeamNgSensorFrame> {
        let mut newest = None;
        for _ in 0..MAX_UDP_POLL_PACKETS {
            let len = match self.socket.recv_from(&mut self.rx_buf) {
                Ok((len, _)) => len,
                Err(e)
                    if matches!(
                        e.kind(),
                        ErrorKind::Interrupted | ErrorKind::ConnectionReset
                    ) =>
                {
                    continue;
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(_) => break,
            };
            if let Ok(f) = serde_json::from_slice::<BeamNgSensorFrame>(&self.rx_buf[..len]) {
                let accept = match self.last_rx_seq {
                    None => true,
                    Some(last) => f.seq > last || (f.seq <= 1 && last > 64),
                };
                if accept {
                    self.last_rx_seq = Some(f.seq);
                    newest = Some(f);
                }
            }
        }
        newest
    }

    /// Our own telemetry, for tools that want to compare BeamNG against the built-in plant.
    pub fn send_telemetry(&mut self, t: &PlantTelemetry) -> Result<(), String> {
        let payload = serde_json::to_vec(t).map_err(|e| e.to_string())?;
        self.send_datagram(&payload)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use std::time::Duration;

    #[test]
    fn can_descriptor_parser_validates_entries_and_handles_short_or_erased_flash() {
        assert!(Simos18CanDescriptor::parse_all_from_flash(&[0u8; 64]).is_empty());
        assert!(Simos18CanDescriptor::parse_all_from_flash(&vec![0xFFu8; 0x8_0000]).is_empty());

        let mut flash = vec![0u8; 0x7_0000];
        // Valid 48-byte RX entry at 0x5BBE4: bus 0, ID 0x3C0, DLC 4, even handler in PFlash0
        let rx_off = Simos18CanDescriptor::RX_48_TABLE_FLASH_OFFSET;
        flash[rx_off + 8..rx_off + 12].copy_from_slice(&0x3C0u32.to_le_bytes());
        flash[rx_off + 16] = 4;
        flash[rx_off + 24..rx_off + 28].copy_from_slice(&0x8005_1000u32.to_le_bytes());

        // Invalid RX entry #1: odd handler address (e.g. corrupt / Thumb-like / erased)
        let bad_off = rx_off + Simos18CanDescriptor::RX_48_ENTRY_SIZE;
        flash[bad_off + 8..bad_off + 12].copy_from_slice(&0x101u32.to_le_bytes());
        flash[bad_off + 16] = 8;
        flash[bad_off + 24..bad_off + 28].copy_from_slice(&0x8005_1001u32.to_le_bytes());

        // Invalid RX entry #2: DLC > 8
        let bad_dlc = rx_off + 2 * Simos18CanDescriptor::RX_48_ENTRY_SIZE;
        flash[bad_dlc + 8..bad_dlc + 12].copy_from_slice(&0x106u32.to_le_bytes());
        flash[bad_dlc + 16] = 9;
        flash[bad_dlc + 24..bad_dlc + 28].copy_from_slice(&0x8005_1000u32.to_le_bytes());

        // Valid 44-byte TX entry at 0x5C038: bus 0, ID 0x0A7, DLC 8, even handler in PFlash1
        let tx_off = Simos18CanDescriptor::TX_44_TABLE_FLASH_OFFSET;
        flash[tx_off + 4..tx_off + 8].copy_from_slice(&0x0A7u32.to_le_bytes());
        flash[tx_off + 16] = 8;
        flash[tx_off + 24..tx_off + 28].copy_from_slice(&0x8089_0000u32.to_le_bytes());

        let parsed = Simos18CanDescriptor::parse_all_from_flash(&flash);
        assert_eq!(parsed.len(), 2);
        assert!(parsed[0].is_rx && parsed[0].can_id == 0x3C0 && parsed[0].dlc == 4);
        assert!(!parsed[1].is_rx && parsed[1].can_id == 0x0A7 && parsed[1].dlc == 8);

        // If the real SC800S50 dump is present, verify known RX and TX descriptors parse cleanly.
        let s50_name = "8V0906264K__0003.bin";
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let bases: Vec<PathBuf> = std::env::var_os("SIMEMU_IMAGE_DIR")
            .into_iter()
            .map(PathBuf::from)
            .chain(std::iter::once(root))
            .collect();
        let found = bases.iter().find_map(|b| {
            [
                b.join(s50_name),
                b.join("vault/firmwares/ecu/stock").join(s50_name),
                b.join("firmwares/ecu/stock").join(s50_name),
            ]
            .into_iter()
            .find(|p| p.is_file())
        });
        if let Some(path) = found {
            let img = std::fs::read(path).unwrap();
            let real = Simos18CanDescriptor::parse_all_from_flash(&img);
            assert!(real.iter().any(|d| d.is_rx && d.can_id == 0x3C0 && d.dlc == 4));
            assert!(real.iter().any(|d| !d.is_rx && d.can_id == 0x0A7 && d.dlc == 8));
        }
    }

    #[test]
    fn udp_bridge_handles_seq_zero_ordering_peer_restart_and_closed_peer() {
        let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        let peer_addr = peer.local_addr().unwrap();
        let mut bridge = BeamNgUdpBridge::bind("127.0.0.1:0", &peer_addr.to_string()).unwrap();
        let bridge_addr = bridge.local_addr().unwrap();

        let send_seq = |seq: u64| {
            let f = BeamNgSensorFrame {
                seq,
                telemetry: serde_json::json!({ "seq": seq }),
            };
            let bytes = serde_json::to_vec(&f).unwrap();
            peer.send_to(&bytes, bridge_addr).unwrap();
        };

        // Initial seq == 0 must be accepted.
        send_seq(0);
        for _ in 0..20 {
            if let Some(rx) = bridge.poll_sensors() {
                assert_eq!(rx.seq, 0);
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(bridge.last_rx_seq, Some(0));

        // Out-of-order / duplicate batch: 100, 99, 100 -> keeps 100.
        send_seq(100);
        send_seq(99);
        send_seq(100);
        std::thread::sleep(Duration::from_millis(5));
        let rx = bridge.poll_sensors().expect("frame 100");
        assert_eq!(rx.seq, 100);
        assert!(bridge.poll_sensors().is_none());

        // Peer restart at seq = 1 after high sequence (> 64) re-synchronizes automatically.
        send_seq(1);
        std::thread::sleep(Duration::from_millis(5));
        let restarted = bridge.poll_sensors().expect("restarted frame 1");
        assert_eq!(restarted.seq, 1);

        // Sending to a closed UDP port must not error (WSAECONNRESET tolerated on Windows).
        drop(peer);
        bridge
            .send_actuators(42, &PlantActuators::default())
            .expect("send to closed peer must succeed cleanly");
        assert!(bridge.poll_sensors().is_none());
    }
}
