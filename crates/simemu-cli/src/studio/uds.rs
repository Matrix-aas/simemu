//! Minimal ISO-TP (ISO 15765-2) + UDS client used to talk to the emulated firmware over CAN,
//! exactly like an external tester would. Fault memory, coding and adaptations live in the firmware, not here.
//!
//! A request longer than 7 bytes goes out as a first frame; its consecutive frames follow the module's flow
//! control (`FS = 0` `ContinueToSend` with `BlockSize` and `STmin`, `FS = 1` `Wait`, `FS = 2` `Overflow`).
//! A request can also go out on the OBD functional address [`OBD_FUNCTIONAL_ID`] (every OBD unit receives it,
//! each answers on its own response ID; the client waits for its unit's answer).
//!
//! Clearing the fault memory (`MILESTONE.md` §1.1): none of the firmwares has a handler for UDS `14` (SIMOS18.1:
//! application service table 0x8005AA8C with 21 entries and NRC 0x11 for the rest, SC8O40; the same table in every
//! SIMOS image; DQ250: table 0x800C4AF8; VERIFIED-S), so `14 FF FF FF` is answered `7F 14 11` by all three units. They
//! clear on OBD service `04` sent functionally (VERIFIED-T: ECU `7F 04 78` then `44`, DQ200 and DQ250 `44`); sent
//! physically the TCUs refuse it with NRC 0x22 and the ECU checks its clear-permission FID 0x46A first. A job is a list of requests sent one after another (the next one when the previous answer is complete),
//! as a workshop tester runs e.g. "extended session → workshop code → coding"; a negative answer ends the job.

use serde::Serialize;
use simemu_tc1791::CanFrame;
use std::collections::VecDeque;

/// One request/answer pair of the current job.
#[derive(Debug, Clone, Serialize, Default)]
pub struct UdsExchange {
    pub request: String,
    pub response: String,
    /// Plain-language reading of the answer ("coding 24", "routine running", "NRC 0x22 conditionsNotCorrect").
    pub note: String,
}

#[derive(Debug, Clone, Serialize, Default)]
pub struct UdsStatus {
    pub request: String,
    pub pending: bool,
    pub response_hex: String,
    pub dtcs: Vec<String>,
    /// Per `dtcs` entry: the code decoded with the module's own DTC table where the session knows it (DQ200_G2:
    /// VAG number → SAE code and event index, CAL `0x80032AD0`; its DTCs are VAG numbers, not ISO 15031-6 codes).
    #[serde(default)]
    pub dtc_sae: Vec<String>,
    pub note: String,
    /// Exchanges of the current job (a single request is a job of one).
    pub log: Vec<UdsExchange>,
    /// Coding as last read or written (`2E/22 0600`), as the decimal number a workshop tester shows.
    pub coding: Option<u32>,
    /// Basic-setting status byte as last read (`22 0102`): 0xC0 running, 0x10 completed, 0x60 failed, 0x80 aborted,
    /// 0x00 none (DQ250 0D9300012F, VERIFIED-S/T, docs/DQ250_IO_MAP.md §8).
    pub routine_status: Option<u8>,
}

/// OBD functional request ID (ISO 15765-4), received by every OBD unit of the car.
pub const OBD_FUNCTIONAL_ID: u32 = 0x7DF;

/// Decodes ISO 15765-2 `STmin` byte into microseconds (`0x00` = 0, `0x01..=0x7F` = 1..127 ms,
/// `0xF1..=0xF9` = 100..900 µs; reserved values default to the ISO maximum 127 ms).
pub fn decode_st_min_us(raw: u8) -> u32 {
    match raw {
        0x00 => 0,
        0x01..=0x7F => raw as u32 * 1_000,
        0xF1..=0xF9 => (raw - 0xF0) as u32 * 100,
        _ => 127_000,
    }
}

pub struct UdsClient {
    pub req_id: u32,
    pub resp_id: u32,
    /// CAN ID of the current job's requests: `req_id`, or [`OBD_FUNCTIONAL_ID`] for a functional job.
    tx_id: u32,
    rx: Vec<u8>,
    expected: usize,
    /// Consecutive frames of our multi-frame request remaining to be released by the module's flow control.
    tx_cf: VecDeque<[u8; 8]>,
    /// Separation time (`STmin`, µs) requested by the module's latest flow-control frame.
    tx_st_min_us: u32,
    /// Requests of the job still to send.
    queue: VecDeque<Vec<u8>>,
    current: Vec<u8>,
    pub status: UdsStatus,
}

fn frame(id: u32, data: [u8; 8]) -> CanFrame {
    CanFrame { node: 0, mo_index: 0, arb_id: id, extended: false, dlc: 8, data, timestamp_us: 0 }
}

pub fn hex(d: &[u8]) -> String {
    d.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
}

/// ISO 14229 negative response codes seen from these modules.
fn nrc_text(c: u8) -> &'static str {
    match c {
        0x10 => "generalReject",
        0x11 => "serviceNotSupported",
        0x12 => "subFunctionNotSupported",
        0x13 => "incorrectMessageLength",
        0x22 => "conditionsNotCorrect",
        0x24 => "requestSequenceError",
        0x31 => "requestOutOfRange",
        0x33 => "securityAccessDenied",
        0x35 => "invalidKey",
        0x36 => "exceededNumberOfAttempts",
        0x37 => "requiredTimeDelayNotExpired",
        0x72 => "generalProgrammingFailure",
        0x78 => "requestCorrectlyReceivedResponsePending",
        0x7E => "subFunctionNotSupportedInActiveSession",
        0x7F => "serviceNotSupportedInActiveSession",
        _ => "",
    }
}

impl UdsClient {
    pub fn new(req_id: u32, resp_id: u32) -> Self {
        Self {
            req_id,
            resp_id,
            tx_id: req_id,
            rx: Vec::new(),
            expected: 0,
            tx_cf: VecDeque::new(),
            tx_st_min_us: 0,
            queue: VecDeque::new(),
            current: Vec::new(),
            status: UdsStatus::default(),
        }
    }

    /// Separation interval in 1 ms co-simulation windows (`>= 1`) between consecutive frames released
    /// by the module's last `FlowControl`.
    pub fn st_min_windows(&self) -> usize {
        (self.tx_st_min_us.div_ceil(1_000) as usize).max(1)
    }

    /// Aborts any in-flight ISO-TP transfer or multi-step job when the target ECU/TCU resets or detaches, clearing
    /// `status.pending` so UI diagnostic pollers never wedge and leftover pre-reset requests cannot fire later.
    pub fn abort_on_reset(&mut self) {
        self.tx_id = self.req_id;
        self.rx.clear();
        self.expected = 0;
        self.tx_cf.clear();
        self.tx_st_min_us = 0;
        self.queue.clear();
        self.current.clear();
        if self.status.pending {
            self.status.pending = false;
            if self.status.note.is_empty() {
                self.status.note = simemu_damos::tr("aborted (unit reset)", "прервано (перезагрузка блока)").into();
            }
        }
    }

    /// Starts a single request (any length); returns its single or first frame (consecutive frames follow the module's
    /// flow control through [`Self::on_frame`]).
    pub fn request(&mut self, label: &str, payload: &[u8]) -> CanFrame {
        self.job(label, vec![payload.to_vec()]).expect("one request")
    }

    /// Starts a job: `reqs` are sent in order, each after the previous answer; returns the first request's frame (`None`: empty job).
    pub fn job(&mut self, label: &str, reqs: Vec<Vec<u8>>) -> Option<CanFrame> {
        self.job_on(label, reqs, false)
    }

    /// [`Self::job`] on the OBD functional address (`functional`) or the unit's physical one.
    pub fn job_on(&mut self, label: &str, reqs: Vec<Vec<u8>>, functional: bool) -> Option<CanFrame> {
        self.tx_id = if functional { OBD_FUNCTIONAL_ID } else { self.req_id };
        let (coding, routine_status) = (self.status.coding, self.status.routine_status);
        self.status = UdsStatus { request: label.to_string(), coding, routine_status, ..Default::default() };
        self.queue = reqs.into();
        self.next_request().pop()
    }

    fn next_request(&mut self) -> Vec<CanFrame> {
        let Some(mut p) = self.queue.pop_front() else { return Vec::new() };
        if p.len() > 4095 {
            p.truncate(4095);
        }
        self.rx.clear();
        self.expected = 0;
        self.tx_cf.clear();
        self.tx_st_min_us = 0;
        self.status.pending = true;
        self.status.log.push(UdsExchange { request: hex(&p), ..Default::default() });
        let mut d = [0x55u8; 8];
        if p.len() <= 7 {
            d[0] = p.len() as u8;
            d[1..1 + p.len()].copy_from_slice(&p);
        } else {
            d[0] = 0x10 | ((p.len() >> 8) as u8 & 0x0F);
            d[1] = p.len() as u8;
            d[2..8].copy_from_slice(&p[..6]);
            for (k, c) in p[6..].chunks(7).enumerate() {
                let mut cf = [0x55u8; 8];
                cf[0] = 0x20 | ((k + 1) & 0xF) as u8;
                cf[1..1 + c.len()].copy_from_slice(c);
                self.tx_cf.push_back(cf);
            }
        }
        self.current = p;
        vec![frame(self.tx_id, d)]
    }

    /// Consumes a frame from the bus; returns the frames the tester sends in reaction: flow control for a
    /// multi-frame answer, the allowed block of our consecutive frames after the module's flow control
    /// (`FS=0` `ContinueToSend` with `BlockSize`, held across `FS=1` `Wait`), or the job's next request.
    pub fn on_frame(&mut self, f: &CanFrame) -> Vec<CanFrame> {
        let dlc = (f.dlc as usize).min(8);
        if !self.status.pending || f.arb_id != self.resp_id || dlc == 0 {
            return Vec::new();
        }
        let d = &f.data[..dlc];
        match d[0] >> 4 {
            0 => {
                let n = (d[0] & 0x0F) as usize;
                if n == 0 || n > 7 || 1 + n > dlc {
                    return Vec::new();
                }
                self.rx = d[1..1 + n].to_vec();
                self.expected = n;
                self.finish()
            }
            1 => {
                if dlc < 8 {
                    return Vec::new();
                }
                let expected = (((d[0] & 0x0F) as usize) << 8) | d[1] as usize;
                if expected < 8 {
                    return Vec::new();
                }
                self.expected = expected;
                self.rx = d[2..8].to_vec();
                vec![frame(self.req_id, [0x30, 0, 0, 0x55, 0x55, 0x55, 0x55, 0x55])]
            }
            2 => {
                if self.expected == 0 || self.rx.len() >= self.expected || dlc < 2 {
                    return Vec::new();
                }
                self.rx.extend_from_slice(&d[1..dlc]);
                if self.rx.len() >= self.expected {
                    self.rx.truncate(self.expected);
                    return self.finish();
                }
                Vec::new()
            }
            3 if !self.tx_cf.is_empty() && dlc >= 3 => {
                self.tx_st_min_us = decode_st_min_us(d[2]);
                match d[0] & 0x0F {
                    // ContinueToSend: release `BlockSize` frames (or all remaining when BS == 0).
                    0 => {
                        let bs = d[1] as usize;
                        let count = if bs == 0 { self.tx_cf.len() } else { bs.min(self.tx_cf.len()) };
                        (0..count)
                            .filter_map(|_| self.tx_cf.pop_front().map(|cf| frame(self.tx_id, cf)))
                            .collect()
                    }
                    // Wait: hold remaining `tx_cf` until a subsequent `FS = 0` FlowControl arrives.
                    1 => Vec::new(),
                    // Overflow: abort the multi-frame transfer and end the job.
                    2 => {
                        self.tx_cf.clear();
                        self.queue.clear();
                        self.status.pending = false;
                        let note = "ISO-TP overflow (FC FS=2)".to_string();
                        if let Some(x) = self.status.log.last_mut() {
                            x.note = note.clone();
                        }
                        self.status.note = note;
                        Vec::new()
                    }
                    _ => Vec::new(),
                }
            }
            _ => Vec::new(),
        }
    }

    fn finish(&mut self) -> Vec<CanFrame> {
        let r = self.rx.clone();
        self.status.response_hex = hex(&r);
        let note = self.interpret(&r);
        if let Some(x) = self.status.log.last_mut() {
            x.response = hex(&r);
            x.note = note.clone();
        }
        self.status.note = note;
        if r.first() == Some(&0x7F) && r.len() >= 3 {
            if r[2] == 0x78 {
                return Vec::new(); // response pending: the answer follows
            }
            self.queue.clear(); // a refused step ends the job
        }
        self.status.pending = false;
        self.next_request()
    }

    /// Reads the answer as a tester display would.
    fn interpret(&mut self, r: &[u8]) -> String {
        let q = &self.current;
        match r {
            [0x7F, _, c, ..] => format!("NRC 0x{c:02X} {}", nrc_text(*c)),
            [0x59, 0x02, _, rest @ ..] => {
                self.status.dtcs = rest
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|c| format!("{:02X}{:02X}{:02X} status 0x{:02X}", c[0], c[1], c[2], c[3]))
                    .collect();
                self.status.dtc_sae.clear();
                format!("{} DTC(s)", self.status.dtcs.len())
            }
            [0x54, ..] | [0x44, ..] => "fault memory cleared by the firmware".into(),
            [0x50, s, ..] => format!("session 0x{s:02X}"),
            [0x62, 0x06, 0x00, c @ ..] | [0x6E, 0x06, 0x00, c @ ..] => {
                let b = if r[0] == 0x62 { c } else { q.get(3..).unwrap_or_default() };
                let c = b.get(..2).map_or(0, |b| u16::from_be_bytes([b[0], b[1]]) as u32);
                self.status.coding = Some(c);
                format!("coding {c:06}")
            }
            [0x62, 0x01, 0x02, s, ..] => {
                self.status.routine_status = Some(*s);
                format!("basic setting: {}", routine_status_text(*s))
            }
            [0x71, 0x01, hi, lo, ..] => {
                self.status.routine_status = Some(0xC0);
                format!("routine 0x{:04X} started", u16::from_be_bytes([*hi, *lo]))
            }
            [0x71, 0x02, hi, lo, ..] => format!("routine 0x{:04X} stopped", u16::from_be_bytes([*hi, *lo])),
            [0x6E, hi, lo, ..] => format!("DID 0x{:04X} written", u16::from_be_bytes([*hi, *lo])),
            _ => String::new(),
        }
    }
}

/// Basic-setting status byte (`22 0102`, DQ250 0D9300012F: written by the routine state machine 0x80070540).
pub fn routine_status_text(s: u8) -> &'static str {
    match s {
        0x00 => "none",
        0xC0 => "running",
        0x10 => "completed",
        0x40 => "completed (0x40)",
        0x60 => "failed",
        0x80 => "aborted",
        _ => "?",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn multi_frame_dtc_response_is_reassembled() {
        let mut c = UdsClient::new(0x7E0, 0x7E8);
        c.request("read", &[0x19, 0x02, 0xFF]);
        // 0x59 02 FF + 2 DTCs = 11 bytes → FF + 1 CF
        let ff = frame(0x7E8, [0x10, 11, 0x59, 0x02, 0xFF, 0x01, 0x23, 0x45]);
        assert_eq!(c.on_frame(&ff).len(), 1);
        let cf = frame(0x7E8, [0x21, 0x09, 0x06, 0x78, 0x9A, 0x08, 0x55, 0x55]);
        assert!(c.on_frame(&cf).is_empty());
        assert!(!c.status.pending);
        assert_eq!(c.status.dtcs.len(), 2);
    }

    /// A functional request goes out on 0x7DF, the answer is taken from the unit's own response ID only.
    #[test]
    fn functional_request_waits_for_its_unit() {
        let mut c = UdsClient::new(0x7E1, 0x7E9);
        let f = c.job_on("clear", vec![vec![0x04]], true).unwrap();
        assert_eq!((f.arb_id, f.data[..2].to_vec()), (OBD_FUNCTIONAL_ID, vec![0x01, 0x04]));
        assert!(c.on_frame(&frame(0x7E8, [0x01, 0x44, 0, 0, 0, 0, 0, 0])).is_empty());
        assert!(c.status.pending);
        c.on_frame(&frame(0x7E9, [0x03, 0x7F, 0x04, 0x78, 0, 0, 0, 0]));
        assert!(c.status.pending, "response pending");
        c.on_frame(&frame(0x7E9, [0x01, 0x44, 0, 0, 0, 0, 0, 0]));
        assert!(!c.status.pending);
        assert_eq!(c.status.note, "fault memory cleared by the firmware");
        assert_eq!(c.job("read", vec![vec![0x19, 0x02, 0x2F]]).unwrap().arb_id, 0x7E1, "next job physical again");
    }

    /// A 9-byte request: first frame, our consecutive frame after the module's flow control, then the job's next
    /// request after the positive answer; a negative answer ends the job.
    #[test]
    fn multi_frame_request_and_job() {
        let mut c = UdsClient::new(0x7E1, 0x7E9);
        let f = c.job("coding", vec![vec![0x2E, 0xF1, 0x98, 0, 0, 0, 0, 0, 1], vec![0x2E, 0x06, 0x00, 0x00, 0x18], vec![0x22, 0x06, 0x00]]).unwrap();
        assert_eq!(f.data, [0x10, 9, 0x2E, 0xF1, 0x98, 0, 0, 0]);
        let cf = c.on_frame(&frame(0x7E9, [0x30, 0, 0, 0, 0, 0, 0, 0]));
        assert_eq!(cf[0].data[..4], [0x21, 0, 0, 1]);
        let next = c.on_frame(&frame(0x7E9, [0x03, 0x6E, 0xF1, 0x98, 0, 0, 0, 0]));
        assert_eq!(next[0].data[..6], [0x05, 0x2E, 0x06, 0x00, 0x00, 0x18]);
        let next = c.on_frame(&frame(0x7E9, [0x03, 0x6E, 0x06, 0x00, 0, 0, 0, 0]));
        assert_eq!(c.status.coding, Some(24));
        assert_eq!(next[0].data[..4], [0x03, 0x22, 0x06, 0x00]);
        assert!(c.on_frame(&frame(0x7E9, [0x03, 0x7F, 0x22, 0x31, 0, 0, 0, 0])).is_empty());
        assert!(!c.status.pending);
        assert_eq!(c.status.log.len(), 3);
        assert_eq!(c.status.note, "NRC 0x31 requestOutOfRange");
    }

    #[test]
    fn flow_control_wait_and_block_size_gate_consecutive_frames() {
        let mut c = UdsClient::new(0x7E1, 0x7E9);
        // 48-byte request -> 6 bytes in FF + 42 bytes across 6 CFs (SN 0x21..0x26)
        let mut payload = vec![0x2E, 0xFD, 0x01];
        payload.extend(0..45u8);
        let ff = c.request("write_long_did", &payload);
        assert_eq!(ff.data[..2], [0x10, 48]);

        // FC FS=1 (Wait): holds all 6 CFs without dropping them
        assert!(c.on_frame(&frame(0x7E9, [0x31, 2, 5, 0, 0, 0, 0, 0])).is_empty());
        assert!(c.on_frame(&frame(0x7E9, [0x31, 2, 5, 0, 0, 0, 0, 0])).is_empty());
        assert!(c.status.pending);

        // FC FS=0 (ContinueToSend), BS=2, STmin=5 ms: releases exactly CF1 and CF2
        let b1 = c.on_frame(&frame(0x7E9, [0x30, 2, 5, 0, 0, 0, 0, 0]));
        assert_eq!(b1.len(), 2);
        assert_eq!(b1[0].data[0], 0x21);
        assert_eq!(b1[1].data[0], 0x22);
        assert_eq!(c.st_min_windows(), 5);

        // Another FC FS=1 (Wait) mid-transfer keeps CF3..CF6 intact
        assert!(c.on_frame(&frame(0x7E9, [0x31, 0, 0, 0, 0, 0, 0, 0])).is_empty());

        // FC FS=0, BS=3, STmin=0xF5 (500 µs -> 1 window): releases CF3, CF4, CF5
        let b2 = c.on_frame(&frame(0x7E9, [0x30, 3, 0xF5, 0, 0, 0, 0, 0]));
        assert_eq!(b2.len(), 3);
        assert_eq!(b2[0].data[0], 0x23);
        assert_eq!(b2[1].data[0], 0x24);
        assert_eq!(b2[2].data[0], 0x25);
        assert_eq!(c.st_min_windows(), 1);

        // FC FS=0, BS=0 (unlimited): releases final CF6
        let b3 = c.on_frame(&frame(0x7E9, [0x30, 0, 0, 0, 0, 0, 0, 0]));
        assert_eq!(b3.len(), 1);
        assert_eq!(b3[0].data[0], 0x26);

        // Positive response completes the job
        assert!(c.on_frame(&frame(0x7E9, [0x03, 0x6E, 0xFD, 0x01, 0x55, 0x55, 0x55, 0x55])).is_empty());
        assert!(!c.status.pending);
        assert_eq!(c.status.note, "DID 0xFD01 written");
    }

    #[test]
    fn flow_control_overflow_aborts_transfer_and_clears_job_queue() {
        let mut c = UdsClient::new(0x7E0, 0x7E8);
        let _ff = c
            .job(
                "two_step",
                vec![vec![0x2E, 0xF1, 0x98, 1, 2, 3, 4, 5, 6], vec![0x3E, 0x00]],
            )
            .unwrap();
        let out = c.on_frame(&frame(0x7E8, [0x32, 0, 0, 0, 0, 0, 0, 0]));
        assert!(out.is_empty());
        assert!(!c.status.pending);
        assert_eq!(c.status.note, "ISO-TP overflow (FC FS=2)");
        assert_eq!(c.status.log.len(), 1, "second step must not be sent after overflow");
    }

    #[test]
    fn sn_wraps_cleanly_over_more_than_fifteen_consecutive_frames() {
        let mut c = UdsClient::new(0x7E0, 0x7E8);
        let payload = vec![0x36; 6 + 17 * 7]; // FF + 17 CFs -> SN 1..15, 0, 1
        let _ff = c.request("transfer_data", &payload);
        let cfs = c.on_frame(&frame(0x7E8, [0x30, 0, 0, 0, 0, 0, 0, 0]));
        assert_eq!(cfs.len(), 17);
        assert_eq!(cfs[0].data[0], 0x21);
        assert_eq!(cfs[14].data[0], 0x2F);
        assert_eq!(cfs[15].data[0], 0x20);
        assert_eq!(cfs[16].data[0], 0x21);
    }

    #[test]
    fn st_min_decoder_handles_millis_micros_and_reserved_bytes() {
        assert_eq!(decode_st_min_us(0x00), 0);
        assert_eq!(decode_st_min_us(0x0A), 10_000);
        assert_eq!(decode_st_min_us(0x7F), 127_000);
        assert_eq!(decode_st_min_us(0xF1), 100);
        assert_eq!(decode_st_min_us(0xF9), 900);
        assert_eq!(decode_st_min_us(0x80), 127_000);
        assert_eq!(decode_st_min_us(0xFA), 127_000);
    }

    #[test]
    fn malformed_isotp_and_short_5902_frames_never_panic_or_abort_prematurely() {
        let mut c = UdsClient::new(0x7E0, 0x7E8);
        c.request("read", &[0x19, 0x02, 0x2F]);

        // Stray CF before FF, SF with SF_DL=0, SF exceeding DLC, and FF with FF_DL < 8 are ignored
        assert!(c.on_frame(&frame(0x7E8, [0x21, 1, 2, 3, 4, 5, 6, 7])).is_empty());
        assert!(c.on_frame(&frame(0x7E8, [0x00, 1, 2, 3, 4, 5, 6, 7])).is_empty());
        let mut short_dlc = frame(0x7E8, [0x05, 0x59, 0x02, 0xFF, 0, 0, 0, 0]);
        short_dlc.dlc = 3;
        assert!(c.on_frame(&short_dlc).is_empty());
        assert!(c.on_frame(&frame(0x7E8, [0x10, 5, 0x59, 0x02, 0, 0, 0, 0])).is_empty());
        assert!(c.status.pending, "malformed frames must not complete pending request");

        // 2-byte truncated `[0x59, 0x02]` response must not panic on `r[3..]`
        assert!(c.on_frame(&frame(0x7E8, [0x02, 0x59, 0x02, 0x55, 0x55, 0x55, 0x55, 0x55])).is_empty());
        assert!(!c.status.pending);
        assert!(c.status.dtcs.is_empty());
    }

    #[test]
    fn abort_on_reset_clears_pending_state_and_queued_requests() {
        let mut c = UdsClient::new(0x7E1, 0x7E9);
        c.job("coding", vec![vec![0x10, 0x03], vec![0x2E, 0x06, 0x00, 0, 0, 0x18]]);
        assert!(c.status.pending);
        c.abort_on_reset();
        assert!(!c.status.pending && !c.status.note.is_empty());
        // A later SingleFrame must not trigger the aborted second job step.
        assert!(c.on_frame(&frame(0x7E9, [0x06, 0x50, 0x03, 0, 0x32, 0x01, 0xF4, 0])).is_empty());
    }
}
