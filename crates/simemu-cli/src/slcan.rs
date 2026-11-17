//! Lawicel / SLCAN (Serial Line CAN) protocol codec and non-blocking multi-client TCP server.
//!
//! Binds strictly to `127.0.0.1` (loopback) so external tools (`python-can`, `SavvyCAN`, `SimosTools`,
//! `VW_Flash`, `can-utils`) can attach, detach, and reconnect at any time — including multiple
//! simultaneous listeners (e.g. a passive CAN monitor alongside an active UDS diagnostic / flash tool)
//! — without blocking or interrupting the emulated ECU / TCU.

use simemu_tc1791::CanFrame;
use std::collections::VecDeque;
use std::io::{ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};

pub const DEFAULT_SLCAN_PORT: u16 = 28750;

pub const SLCAN_OK_REPLY: &[u8] = b"\r";
pub const SLCAN_ERR_REPLY: &[u8] = b"\x07";
pub const SLCAN_VERSION_REPLY: &[u8] = b"V1013\r";
pub const SLCAN_VERSION_MINOR_REPLY: &[u8] = b"v1013\r";
pub const SLCAN_SERIAL_REPLY: &[u8] = b"N0001\r";
pub const SLCAN_STATUS_REPLY: &[u8] = b"F00\r";

const MAX_LINE_LEN: usize = 256;
const MAX_CLIENT_TX_BUF: usize = 256 * 1024;
const MAX_CLIENTS: usize = 32;
const MAX_RX_QUEUE: usize = 4096;
const MAX_POLL_READ_BYTES: usize = 64 * 1024;

/// Parsed host-to-adapter Lawicel / SLCAN command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SlcanCommand {
    /// Empty `\r` line used by clients to flush / sync the serial stream.
    Empty,
    /// `S0`..`S8`: select standard CAN bit-rate (10 kbps .. 1 Mbps; `S6` = 500 kbps).
    SetBitrate(u8),
    /// `sXXYY`: select custom BTR0/BTR1 bit-rate registers.
    SetBtr(u16),
    /// `Y` or `Y0`..`Y9`: Lawicel / CANable data bit-rate command.
    SetDataBitrate(Option<u8>),
    /// `O` (normal) or `L` (listen-only): open the CAN channel.
    Open { listen_only: bool },
    /// `C`: close the CAN channel.
    Close,
    /// `Z0` / `Z1`: disable / enable 4-hex-digit millisecond RX timestamps (`0000`..`EA5F`).
    SetTimestamps(bool),
    /// `X0` / `X1`: disable / enable `z\r` / `Z\r` auto-poll transmit acknowledgements.
    SetAutoPollAck(bool),
    /// `W0` / `W1`: dual / single acceptance filter mode.
    SetFilterMode(u8),
    /// `MXXXXXXXX`: 32-bit acceptance code register.
    SetAcceptanceCode(u32),
    /// `mXXXXXXXX`: 32-bit acceptance mask register.
    SetAcceptanceMask(u32),
    /// `F`: read status flags (`F00\r`).
    ReadStatus,
    /// `V` / `v`: read hardware & firmware version (`V1013\r` / `v1013\r`).
    ReadVersion { detailed: bool },
    /// `N`: read 4-character serial number (`N0001\r`).
    ReadSerial,
    /// `P` / `A`: legacy Lawicel poll single / poll all commands.
    Poll { all: bool },
    /// `t`, `T`, `r`, `R`: standard or extended data / remote (RTR) CAN frame.
    Frame { frame: CanFrame, remote: bool },
}

fn is_hex(s: &str) -> bool {
    !s.is_empty() && s.bytes().all(|b| b.is_ascii_hexdigit())
}

fn decode_hex_into(s: &str, out: &mut [u8]) -> Result<(), String> {
    if s.len() != out.len() * 2 {
        return Err("hex payload length mismatch".into());
    }
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = u8::from_str_radix(&s[i * 2..i * 2 + 2], 16)
            .map_err(|_| "invalid hex byte in SLCAN payload".to_string())?;
    }
    Ok(())
}

/// Parses a single SLCAN frame command (`t`, `T`, `r`, `R`), returning `(CanFrame, is_remote)`.
pub fn parse_frame(line: &str) -> Result<(CanFrame, bool), String> {
    let line = line.trim_end_matches(['\r', '\n']);
    let Some(&prefix) = line.as_bytes().first() else {
        return Err("empty SLCAN frame".into());
    };
    let extended = match prefix {
        b't' | b'r' => false,
        b'T' | b'R' => true,
        _ => return Err(format!("invalid frame prefix {:?}", prefix as char)),
    };
    let remote = matches!(prefix, b'r' | b'R');
    let id_len = if extended { 8 } else { 3 };
    let id_end = 1 + id_len;
    if line.len() < id_end + 1 || !line.is_ascii() {
        return Err("short SLCAN frame".into());
    }
    let arb_id = u32::from_str_radix(&line[1..id_end], 16).map_err(|_| "invalid CAN ID")?;
    let max_id = if extended { 0x1FFF_FFFF } else { 0x7FF };
    if arb_id > max_id {
        return Err("CAN ID out of range".into());
    }
    let dlc = u8::from_str_radix(&line[id_end..id_end + 1], 16).map_err(|_| "invalid DLC")?;
    if dlc > 8 {
        return Err("SLCAN DLC > 8".into());
    }
    let rest = &line[id_end + 1..];
    let mut data = [0u8; 8];
    if remote {
        if !rest.is_empty() {
            return Err("unexpected payload on RTR frame".into());
        }
    } else {
        let data_hex_len = dlc as usize * 2;
        if rest.len() != data_hex_len {
            return Err("invalid SLCAN payload length".into());
        }
        decode_hex_into(&rest[..data_hex_len], &mut data[..dlc as usize])?;
    }
    Ok((
        CanFrame {
            node: 0,
            mo_index: 0,
            arb_id,
            extended,
            dlc,
            data,
            timestamp_us: 0,
        },
        remote,
    ))
}

/// Parses one CR/LF-stripped SLCAN line from a host client.
pub fn parse_command(raw_line: &str) -> Result<SlcanCommand, String> {
    let line = raw_line.trim_end_matches(['\r', '\n']);
    if line.is_empty() {
        return Ok(SlcanCommand::Empty);
    }
    if !line.is_ascii() {
        return Err("non-ASCII SLCAN command".into());
    }
    let bytes = line.as_bytes();
    match bytes[0] {
        b't' | b'T' | b'r' | b'R' => {
            let (frame, remote) = parse_frame(line)?;
            Ok(SlcanCommand::Frame { frame, remote })
        }
        b'S' if bytes.len() == 2 && (b'0'..=b'8').contains(&bytes[1]) => {
            Ok(SlcanCommand::SetBitrate(bytes[1] - b'0'))
        }
        b's' if bytes.len() == 5 && is_hex(&line[1..]) => {
            let btr = u16::from_str_radix(&line[1..], 16).map_err(|_| "invalid BTR")?;
            Ok(SlcanCommand::SetBtr(btr))
        }
        b'Y' if bytes.len() == 1 => Ok(SlcanCommand::SetDataBitrate(None)),
        b'Y' if bytes.len() == 2 && bytes[1].is_ascii_digit() => {
            Ok(SlcanCommand::SetDataBitrate(Some(bytes[1] - b'0')))
        }
        b'O' if bytes.len() == 1 => Ok(SlcanCommand::Open { listen_only: false }),
        b'L' if bytes.len() == 1 => Ok(SlcanCommand::Open { listen_only: true }),
        b'C' if bytes.len() == 1 => Ok(SlcanCommand::Close),
        b'Z' if bytes.len() == 2 && matches!(bytes[1], b'0' | b'1') => {
            Ok(SlcanCommand::SetTimestamps(bytes[1] == b'1'))
        }
        b'X' if bytes.len() == 2 && matches!(bytes[1], b'0' | b'1') => {
            Ok(SlcanCommand::SetAutoPollAck(bytes[1] == b'1'))
        }
        b'W' if bytes.len() == 2 && matches!(bytes[1], b'0' | b'1') => {
            Ok(SlcanCommand::SetFilterMode(bytes[1] - b'0'))
        }
        b'M' if bytes.len() == 9 && is_hex(&line[1..]) => {
            let v = u32::from_str_radix(&line[1..], 16).map_err(|_| "invalid acceptance code")?;
            Ok(SlcanCommand::SetAcceptanceCode(v))
        }
        b'm' if bytes.len() == 9 && is_hex(&line[1..]) => {
            let v = u32::from_str_radix(&line[1..], 16).map_err(|_| "invalid acceptance mask")?;
            Ok(SlcanCommand::SetAcceptanceMask(v))
        }
        b'F' if bytes.len() == 1 => Ok(SlcanCommand::ReadStatus),
        b'V' if bytes.len() == 1 => Ok(SlcanCommand::ReadVersion { detailed: false }),
        b'v' if bytes.len() == 1 => Ok(SlcanCommand::ReadVersion { detailed: true }),
        b'N' if bytes.len() == 1 => Ok(SlcanCommand::ReadSerial),
        b'P' if bytes.len() == 1 => Ok(SlcanCommand::Poll { all: false }),
        b'A' if bytes.len() == 1 => Ok(SlcanCommand::Poll { all: true }),
        _ => Err(format!("unsupported SLCAN command {line:?}")),
    }
}

/// Formats a [`CanFrame`] as a Lawicel SLCAN line (`tIIIL...` or `TIIIIIIII L...`) without timestamp.
pub fn format_frame(frame: &CanFrame) -> String {
    format_frame_with_ts(frame, false)
}

/// Formats a [`CanFrame`] as a Lawicel SLCAN line, optionally appending the 4-hex-digit Lawicel
/// millisecond timestamp (`0000`..`EA5F`, wrapping every 60 000 ms).
pub fn format_frame_with_ts(frame: &CanFrame, with_timestamp: bool) -> String {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let dlc = frame.dlc.min(8) as usize;
    let mut out = String::with_capacity(if frame.extended { 32 } else { 27 });
    if frame.extended {
        use std::fmt::Write;
        let _ = write!(out, "T{:08X}{:X}", frame.arb_id & 0x1FFF_FFFF, dlc);
    } else {
        use std::fmt::Write;
        let _ = write!(out, "t{:03X}{:X}", frame.arb_id & 0x7FF, dlc);
    }
    for &b in &frame.data[..dlc] {
        out.push(HEX[(b >> 4) as usize] as char);
        out.push(HEX[(b & 0xF) as usize] as char);
    }
    if with_timestamp {
        use std::fmt::Write;
        let ts_ms = ((frame.timestamp_us / 1000) % 60_000) as u16;
        let _ = write!(out, "{ts_ms:04X}");
    }
    out.push('\r');
    out
}

struct SlcanClient {
    id: u64,
    peer: SocketAddr,
    stream: TcpStream,
    rx_buf: Vec<u8>,
    rx_overflow: bool,
    last_was_cr: bool,
    tx_buf: VecDeque<u8>,
    /// Starts `true` so raw socket tools work immediately without requiring `O\r`; `C\r` sets `false`
    /// and `O\r` / `L\r` set `true` so standard SLCAN tools (`python-can`, `SavvyCAN`) receive zero
    /// stray bus frames between `C\r` and `O\r`.
    active: bool,
    listen_only: bool,
    timestamps: bool,
    auto_poll_ack: bool,
    disconnected: bool,
}

impl SlcanClient {
    fn new(id: u64, peer: SocketAddr, stream: TcpStream) -> Self {
        Self {
            id,
            peer,
            stream,
            rx_buf: Vec::with_capacity(64),
            rx_overflow: false,
            last_was_cr: false,
            tx_buf: VecDeque::with_capacity(1024),
            active: true,
            listen_only: false,
            timestamps: false,
            auto_poll_ack: false,
            disconnected: false,
        }
    }

    fn queue_reply(&mut self, bytes: &[u8]) {
        if self.tx_buf.len() + bytes.len() > MAX_CLIENT_TX_BUF {
            self.disconnected = true;
            return;
        }
        self.tx_buf.extend(bytes);
    }

    fn flush_tx(&mut self) {
        while !self.tx_buf.is_empty() && !self.disconnected {
            let (front, _) = self.tx_buf.as_slices();
            if front.is_empty() {
                break;
            }
            match self.stream.write(front) {
                Ok(0) => {
                    self.disconnected = true;
                    break;
                }
                Ok(n) => {
                    self.tx_buf.drain(..n);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(_) => {
                    self.disconnected = true;
                    break;
                }
            }
        }
    }

    fn handle_line(&mut self, line: &str, rx_queue: &mut VecDeque<(u64, CanFrame)>) {
        match parse_command(line) {
            Ok(SlcanCommand::Empty)
            | Ok(SlcanCommand::SetBitrate(_))
            | Ok(SlcanCommand::SetBtr(_))
            | Ok(SlcanCommand::SetDataBitrate(_))
            | Ok(SlcanCommand::SetFilterMode(_))
            | Ok(SlcanCommand::SetAcceptanceCode(_))
            | Ok(SlcanCommand::SetAcceptanceMask(_))
            | Ok(SlcanCommand::Poll { all: false }) => self.queue_reply(SLCAN_OK_REPLY),
            Ok(SlcanCommand::Poll { all: true }) => self.queue_reply(b"A\r"),
            Ok(SlcanCommand::Open { listen_only }) => {
                self.active = true;
                self.listen_only = listen_only;
                self.queue_reply(SLCAN_OK_REPLY);
            }
            Ok(SlcanCommand::Close) => {
                self.active = false;
                self.listen_only = false;
                self.queue_reply(SLCAN_OK_REPLY);
            }
            Ok(SlcanCommand::SetTimestamps(on)) => {
                self.timestamps = on;
                self.queue_reply(SLCAN_OK_REPLY);
            }
            Ok(SlcanCommand::SetAutoPollAck(on)) => {
                self.auto_poll_ack = on;
                self.queue_reply(SLCAN_OK_REPLY);
            }
            Ok(SlcanCommand::ReadStatus) => self.queue_reply(SLCAN_STATUS_REPLY),
            Ok(SlcanCommand::ReadVersion { detailed: false }) => self.queue_reply(SLCAN_VERSION_REPLY),
            Ok(SlcanCommand::ReadVersion { detailed: true }) => self.queue_reply(SLCAN_VERSION_MINOR_REPLY),
            Ok(SlcanCommand::ReadSerial) => self.queue_reply(SLCAN_SERIAL_REPLY),
            Ok(SlcanCommand::Frame { frame, remote: _ }) => {
                if self.listen_only || !self.active || rx_queue.len() >= MAX_RX_QUEUE {
                    self.queue_reply(SLCAN_ERR_REPLY);
                } else {
                    if self.auto_poll_ack {
                        self.queue_reply(if frame.extended { b"Z\r" } else { b"z\r" });
                    }
                    rx_queue.push_back((self.id, frame));
                }
            }
            Err(_) => self.queue_reply(SLCAN_ERR_REPLY),
        }
    }

    fn poll_rx(&mut self, rx_queue: &mut VecDeque<(u64, CanFrame)>) {
        let mut scratch = [0u8; 2048];
        let mut total_read = 0usize;
        while total_read < MAX_POLL_READ_BYTES {
            match self.stream.read(&mut scratch) {
                Ok(0) => {
                    self.disconnected = true;
                    break;
                }
                Ok(n) => {
                    total_read += n;
                    for &b in &scratch[..n] {
                        if b == b'\n' && self.last_was_cr {
                            self.last_was_cr = false;
                            continue;
                        }
                        self.last_was_cr = b == b'\r';
                        if b == b'\r' || b == b'\n' {
                            if self.rx_overflow {
                                self.rx_buf.clear();
                                self.rx_overflow = false;
                                self.queue_reply(SLCAN_ERR_REPLY);
                            } else {
                                match std::str::from_utf8(&self.rx_buf) {
                                    Ok(s) => {
                                        let owned = s.to_string();
                                        self.rx_buf.clear();
                                        self.handle_line(&owned, rx_queue);
                                    }
                                    Err(_) => {
                                        self.rx_buf.clear();
                                        self.queue_reply(SLCAN_ERR_REPLY);
                                    }
                                }
                            }
                        } else if self.rx_buf.len() < MAX_LINE_LEN {
                            self.rx_buf.push(b);
                        } else {
                            self.rx_overflow = true;
                        }
                    }
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(_) => {
                    self.disconnected = true;
                    break;
                }
            }
        }
    }
}

/// Non-blocking multi-client SLCAN-over-TCP server bound strictly to `127.0.0.1`.
pub struct SlcanServer {
    listener: TcpListener,
    port: u16,
    clients: Vec<SlcanClient>,
    next_client_id: u64,
    rx_queue: VecDeque<(u64, CanFrame)>,
    log_events: bool,
}

impl SlcanServer {
    /// Binds a non-blocking TCP listener on `127.0.0.1:<port>` (`port == 0` picks an ephemeral port).
    pub fn bind(port: u16, log_events: bool) -> Result<Self, String> {
        let listener = TcpListener::bind(("127.0.0.1", port))
            .map_err(|e| format!("bind 127.0.0.1:{port}: {e}"))?;
        listener
            .set_nonblocking(true)
            .map_err(|e| format!("set_nonblocking 127.0.0.1:{port}: {e}"))?;
        let actual_port = listener
            .local_addr()
            .map(|a| a.port())
            .unwrap_or(port);
        Ok(Self {
            listener,
            port: actual_port,
            clients: Vec::new(),
            next_client_id: 1,
            rx_queue: VecDeque::new(),
            log_events,
        })
    }

    pub fn port(&self) -> u16 {
        self.port
    }

    pub fn client_count(&self) -> usize {
        self.clients.len()
    }

    pub fn has_pending_rx(&self) -> bool {
        !self.rx_queue.is_empty()
    }

    pub fn pop_rx(&mut self) -> Option<(u64, CanFrame)> {
        self.rx_queue.pop_front()
    }

    /// Accepts pending connections, parses incoming SLCAN commands/frames, and flushes replies.
    pub fn poll(&mut self) {
        for _ in 0..MAX_CLIENTS {
            match self.listener.accept() {
                Ok((mut stream, peer)) => {
                    if self.clients.len() >= MAX_CLIENTS {
                        let _ = stream.write_all(SLCAN_ERR_REPLY);
                        let _ = stream.shutdown(std::net::Shutdown::Both);
                        continue;
                    }
                    let _ = stream.set_nonblocking(true);
                    let _ = stream.set_nodelay(true);
                    let id = self.next_client_id;
                    self.next_client_id += 1;
                    if self.log_events {
                        eprintln!("  SLCAN client #{id} connected ({peer})");
                    }
                    self.clients.push(SlcanClient::new(id, peer, stream));
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }

        for client in &mut self.clients {
            client.poll_rx(&mut self.rx_queue);
            client.flush_tx();
        }

        self.prune_disconnected();
    }

    /// Broadcasts `frame` to all active connected SLCAN clients (except `exclude_client` so a transmitting
    /// client does not receive a self-echo while other connected monitors observe the frame).
    pub fn broadcast(&mut self, frame: &CanFrame, exclude_client: Option<u64>) {
        if self.clients.is_empty() {
            return;
        }
        let mut plain: Option<String> = None;
        let mut with_ts: Option<String> = None;
        for client in &mut self.clients {
            if !client.active || client.disconnected || exclude_client == Some(client.id) {
                continue;
            }
            let line = if client.timestamps {
                with_ts.get_or_insert_with(|| format_frame_with_ts(frame, true))
            } else {
                plain.get_or_insert_with(|| format_frame_with_ts(frame, false))
            };
            client.queue_reply(line.as_bytes());
            client.flush_tx();
        }
        self.prune_disconnected();
    }

    /// Flushes pending TX bytes across all connected clients.
    pub fn flush(&mut self) {
        for client in &mut self.clients {
            client.flush_tx();
        }
        self.prune_disconnected();
    }

    fn prune_disconnected(&mut self) {
        let log_events = self.log_events;
        self.clients.retain(|c| {
            if c.disconnected && log_events {
                eprintln!("  SLCAN client #{} disconnected ({})", c.id, c.peer);
            }
            !c.disconnected
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader};
    use std::time::Duration;

    #[test]
    fn slcan_standard_and_extended_round_trip() {
        let cmd = parse_command("t7E0802F1875555555555\r").unwrap();
        let SlcanCommand::Frame { frame, remote } = cmd else {
            panic!("expected frame");
        };
        assert!(!remote && !frame.extended);
        assert_eq!(frame.arb_id, 0x7E0);
        assert_eq!(frame.dlc, 8);
        assert_eq!(frame.data, [0x02, 0xF1, 0x87, 0x55, 0x55, 0x55, 0x55, 0x55]);
        assert_eq!(format_frame(&frame), "t7E0802F1875555555555\r");

        // Lowercase hex input + timestamp output
        let (mut ext, ext_rtr) = parse_frame("T0007e0f8301abef").unwrap();
        assert!(!ext_rtr && ext.extended);
        assert_eq!(ext.arb_id, 0x7E0F8);
        assert_eq!(ext.dlc, 3);
        assert_eq!(&ext.data[..3], &[0x01, 0xAB, 0xEF]);
        ext.timestamp_us = 1_234_567;
        assert_eq!(format_frame_with_ts(&ext, true), "T0007E0F8301ABEF04D2\r");

        // Standard and extended RTR frames
        let (rtr_std, is_rtr) = parse_frame("r1234").unwrap();
        assert!(is_rtr && !rtr_std.extended && rtr_std.arb_id == 0x123 && rtr_std.dlc == 4);
        let (rtr_ext, is_rtr_ext) = parse_frame("R18DAF1108").unwrap();
        assert!(is_rtr_ext && rtr_ext.extended && rtr_ext.arb_id == 0x18DA_F110 && rtr_ext.dlc == 8);
    }

    #[test]
    fn slcan_control_commands_and_validation() {
        assert_eq!(parse_command("").unwrap(), SlcanCommand::Empty);
        assert_eq!(parse_command("S6\r").unwrap(), SlcanCommand::SetBitrate(6));
        assert_eq!(parse_command("s001C").unwrap(), SlcanCommand::SetBtr(0x001C));
        assert_eq!(parse_command("Y").unwrap(), SlcanCommand::SetDataBitrate(None));
        assert_eq!(parse_command("Y2").unwrap(), SlcanCommand::SetDataBitrate(Some(2)));
        assert_eq!(parse_command("O").unwrap(), SlcanCommand::Open { listen_only: false });
        assert_eq!(parse_command("L").unwrap(), SlcanCommand::Open { listen_only: true });
        assert_eq!(parse_command("C").unwrap(), SlcanCommand::Close);
        assert_eq!(parse_command("Z0").unwrap(), SlcanCommand::SetTimestamps(false));
        assert_eq!(parse_command("Z1").unwrap(), SlcanCommand::SetTimestamps(true));
        assert_eq!(parse_command("X0").unwrap(), SlcanCommand::SetAutoPollAck(false));
        assert_eq!(parse_command("X1").unwrap(), SlcanCommand::SetAutoPollAck(true));
        assert_eq!(parse_command("F").unwrap(), SlcanCommand::ReadStatus);
        assert_eq!(parse_command("V").unwrap(), SlcanCommand::ReadVersion { detailed: false });
        assert_eq!(parse_command("N").unwrap(), SlcanCommand::ReadSerial);
        assert_eq!(parse_command("M00000000").unwrap(), SlcanCommand::SetAcceptanceCode(0));
        assert_eq!(parse_command("mFFFFFFFF").unwrap(), SlcanCommand::SetAcceptanceMask(0xFFFF_FFFF));

        // Invalid syntax cases must return Err (-> 0x07 BELL)
        for bad in ["X", "S9", "Z2", "t8000", "T200000000", "t7E0900", "t7E0200", "t7E01GG", "r7E021122"] {
            assert!(parse_command(bad).is_err(), "expected {bad:?} to fail");
        }
    }

    fn read_until_cr(reader: &mut BufReader<TcpStream>) -> Vec<u8> {
        let mut out = Vec::new();
        reader.read_until(b'\r', &mut out).unwrap();
        out
    }

    #[test]
    fn tcp_server_handles_handshakes_multi_client_broadcast_and_reconnects() {
        let mut srv = SlcanServer::bind(0, false).unwrap();
        let addr = ("127.0.0.1", srv.port());

        // Client 1: SavvyCAN / python-can handshake + Z1 timestamps + listen-only monitor
        let mut c1 = TcpStream::connect(addr).unwrap();
        c1.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let mut r1 = BufReader::new(c1.try_clone().unwrap());

        c1.write_all(b"\rC\rV\rN\rF\rS6\rZ1\rL\rX\r").unwrap();
        for _ in 0..5 {
            srv.poll();
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(read_until_cr(&mut r1), b"\r"); // empty \r
        assert_eq!(read_until_cr(&mut r1), b"\r"); // C
        assert_eq!(read_until_cr(&mut r1), b"V1013\r"); // V
        assert_eq!(read_until_cr(&mut r1), b"N0001\r"); // N
        assert_eq!(read_until_cr(&mut r1), b"F00\r"); // F
        assert_eq!(read_until_cr(&mut r1), b"\r"); // S6
        assert_eq!(read_until_cr(&mut r1), b"\r"); // Z1
        assert_eq!(read_until_cr(&mut r1), b"\r"); // L
        // Next byte should be 0x07 Bell for invalid "X\r", and connection remains alive!
        let mut bell = [0u8; 1];
        r1.read_exact(&mut bell).unwrap();
        assert_eq!(bell, [0x07]);

        // Listen-only client attempting to transmit gets BELL and does NOT enqueue RX
        c1.write_all(b"t7E080210035555555555\r").unwrap();
        for _ in 0..5 {
            srv.poll();
            std::thread::sleep(Duration::from_millis(5));
        }
        r1.read_exact(&mut bell).unwrap();
        assert_eq!(bell, [0x07]);
        assert!(!srv.has_pending_rx());

        // Client 2: Active UDS flasher connects simultaneously
        let mut c2 = TcpStream::connect(addr).unwrap();
        c2.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let mut r2 = BufReader::new(c2.try_clone().unwrap());
        c2.write_all(b"S6\rO\rt7E080210035555555555\r").unwrap();
        for _ in 0..5 {
            srv.poll();
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(srv.client_count(), 2);
        assert_eq!(read_until_cr(&mut r2), b"\r"); // S6
        assert_eq!(read_until_cr(&mut r2), b"\r"); // O

        let (sender_id, mut rx_frame) = srv.pop_rx().expect("queued frame from client 2");
        assert_eq!(rx_frame.arb_id, 0x7E0);
        rx_frame.timestamp_us = 250_000; // 250 ms = 0x00FA

        // Broadcast external frame excluding sender: Client 1 sees it with timestamp, Client 2 does not self-echo
        srv.broadcast(&rx_frame, Some(sender_id));
        assert_eq!(read_until_cr(&mut r1), b"t7E08021003555555555500FA\r");

        // ECU answers on 0x7E8: BOTH Client 1 (with Z1 timestamp) and Client 2 (plain Z0) receive it
        let ecu_resp = CanFrame {
            node: 0,
            mo_index: 0,
            arb_id: 0x7E8,
            extended: false,
            dlc: 8,
            data: [0x06, 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4, 0x55],
            timestamp_us: 251_000, // 251 ms = 0x00FB
        };
        srv.broadcast(&ecu_resp, None);
        assert_eq!(read_until_cr(&mut r1), b"t7E88065003003201F45500FB\r");
        assert_eq!(read_until_cr(&mut r2), b"t7E88065003003201F455\r");

        // Disconnect Client 2, verify server stays alive and Client 3 can reconnect seamlessly
        drop(r2);
        drop(c2);
        for _ in 0..5 {
            srv.poll();
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(srv.client_count(), 1);

        let mut c3 = TcpStream::connect(addr).unwrap();
        c3.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let mut r3 = BufReader::new(c3.try_clone().unwrap());
        c3.write_all(b"V\r").unwrap();
        for _ in 0..5 {
            srv.poll();
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(srv.client_count(), 2);
        assert_eq!(read_until_cr(&mut r3), b"V1013\r");
    }

    #[test]
    fn oversize_unterminated_line_and_rx_queue_cap_resist_dos() {
        let mut srv = SlcanServer::bind(0, false).unwrap();
        let addr = ("127.0.0.1", srv.port());

        let mut c = TcpStream::connect(addr).unwrap();
        c.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let mut r = BufReader::new(c.try_clone().unwrap());

        // Stream 10 KB without `\r` -> rx_buf stays bounded, returns single BELL on `\r`, and recovers
        let junk = vec![b'A'; 10 * 1024];
        c.write_all(&junk).unwrap();
        c.write_all(b"\rV\r").unwrap();
        for _ in 0..8 {
            srv.poll();
            std::thread::sleep(Duration::from_millis(2));
        }
        let mut bell = [0u8; 1];
        r.read_exact(&mut bell).unwrap();
        assert_eq!(bell, [0x07]);
        assert_eq!(read_until_cr(&mut r), b"V1013\r");

        // Flood > MAX_RX_QUEUE frames without draining -> rx_queue is strictly bounded at MAX_RX_QUEUE
        let mut burst = Vec::with_capacity((MAX_RX_QUEUE + 64) * 6);
        for _ in 0..(MAX_RX_QUEUE + 64) {
            burst.extend_from_slice(b"t1000\r");
        }
        c.write_all(&burst).unwrap();
        for _ in 0..16 {
            srv.poll();
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(srv.rx_queue.len(), MAX_RX_QUEUE);
    }
}
