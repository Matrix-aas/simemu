//! SENT receiver module of the TC1791 (base `0xF032_1000`, 8 channels, TC1791 DS p.21 / pins SENT0..7 p.52–58;
//! `SENT0 / IN40 = P8.0` ball `F1`, DS p.52) and
//! the sensor side of the SAE J2716 link: frames are produced at the sensor's own tick and evaluated by the receiver
//! exactly as the unmodified SC800S50 programs it.
//!
//! Firmware evidence (SC800S50; REA `ev_e4b275fe…` ISRs, MMIO trace of boot = VERIFIED-T):
//! * Init: CLC = 0, FDR = 0x43FF (fSENT = fFPI = 90 MHz), SRC 0x9FC = SRPN 0x36, SRC 0x9F8 = SRPN 0x37 (TOS CPU).
//!   Logical channels (table `0x80042710` → `0x800426E0`, 16 B each): 0 → hw 0, 1 → hw 2, 2 → hw 3, all with
//!   6 data nibbles, no serial messages (entry +5 = 0). Per hw channel (block `0x100 + 0x40·ch`): RCR (+0x08) = 0,
//!   CPDR (+0x00) = 5, CFDR (+0x04) = 0x9F1 (ch 0) / 0x9D7 (ch 2/3), RCR = 0x10620, +0x1C = 0x76543210, +0x14 = 0x22,
//!   +0x2C = 0x3E1, +0x30 = 0x100000; only RCR0 bit 0 is set afterwards (only hw channel 0 runs).
//! * Data ISR `0x8012A0FC`: per channel, if bit 0 of +0x20 is set: copies word `0x080 + 4·ch` (data nibbles, D1 in
//!   bits 3:0) and +0x0C (CRC in bits 3:0, status/communication nibble in 11:8; driver read `0x800A8BC0`), then writes
//!   1 to +0x28. Error ISR `0x8012A1E8`: tests +0x20 against 0x13E0 (bits 9, 8, 7, 6, 5, 12 → driver flags
//!   0x10/8/1/4/2/0x20) and clears each bit by writing it to +0x28. Serial ISR `0x8012A2F4`: bit 11 → word +0x10
//!   (short serial message: CRC 27:24, ID 19:16, data 7:0), CRC with the J2716 table at `0x8005D3D4` (seed 5; the
//!   table equals the SAE J2716 CRC4 table, polynomial x⁴+x³+x²+1 — checked in `j2716_table_matches_the_firmware`).
//!
//! Register names follow the AURIX SENT module (successor of this IP; no TC1791 user manual is available, so every
//! name and every bit not listed above is HYP): +0x20 INTSTAT, +0x24 INTSET, +0x28 INTCLR, +0x2C INTEN (0x3E1 = RSI +
//! FRI, FDI, NNI, NVI, CRCI), +0x30 INP (interrupt node per source group, nibble k: RSI, RDI, RBI, TDI, TBI, ERRI,
//! SDI, WDI → 0x100000 = errors on node 1, everything else on node 0), +0x1C VIEW (RDR nibble pointer, identity),
//! RCR bits 11:8 = number of data nibbles (6, matches the driver table). Node n raises SRC `0x9FC − 4·n` — the
//! layout of every AUDO module (SRC0 at the top); consistent with SRPN 0x36 for data/serial and 0x37 for errors.
//! CFDR / CPDR: the nominal unit time is taken as tick = (DIV + 1) / (56 · fSENT / (PDIV + 1)) (56 ticks = the
//! calibration pulse): 3.03 µs on ch 0, 3.00 µs on ch 2/3 — HYP; frames whose calibration pulse deviates by more
//! than [`TICK_TOLERANCE`] set FRI instead of RSI (SAE J2716 allows ±20 % transmitter clock tolerance; UNVERIFIED
//! window of this receiver). The fast-channel CRC is SAE J2716 "recommended" (seed 5, data nibbles, one augmenting
//! zero nibble; RCR bit 16 HYP = this mode).
//!
//! Sensor side ([`SentSensor`]): calibration pulse 56 ticks, then status, data and CRC nibbles of 12 + value ticks,
//! no pause pulse (UNVERIFIED for the N493 position sensor). Frames are transmitted back to back and evaluated lazily
//! at their end instant (board boundaries and SENT register reads): no allocation, a few operations per frame.

use crate::memory::Tc1791HardwareState;

pub const SENT_BASE: u32 = 0xF032_1000;
/// Last module register below the two SRC registers (`0x9F8`/`0x9FC` belong to the interrupt unit).
pub const SENT_LAST: u32 = 0xF032_19F4;
pub const NCH: usize = 8;
const RDR0: u32 = 0x080;
const CH0: u32 = 0x100;
const CH_STRIDE: u32 = 0x40;
const RCR: u32 = 0x08;
const RSR: u32 = 0x0C;
const CPDR: u32 = 0x00;
const CFDR: u32 = 0x04;
const VIEW: u32 = 0x1C;
const INTSTAT: u32 = 0x20;
const INTSET: u32 = 0x24;
const INTCLR: u32 = 0x28;
const INTEN: u32 = 0x2C;
const INP: u32 = 0x30;
/// INTSTAT bits (AURIX names, HYP; the ISRs above use 0, 5..9, 11, 12).
pub const RSI: u32 = 1 << 0;
pub const RDI: u32 = 1 << 1;
pub const FRI: u32 = 1 << 5;
pub const NNI: u32 = 1 << 7;
/// Source group (INP nibble) of each INTSTAT bit: 0 RSI, 1 RDI, 2 RBI, 3 TDI, 4 TBI, 5 ERRI (bits 5..10, 12, 13),
/// 6 SDI (11), 7 WDI (13 is the watchdog error in the AURIX layout; grouped with ERRI here).
const GROUP_OF_BIT: [u8; 14] = [0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 5, 6, 5, 7];
/// Accepted deviation of the measured calibration pulse from the nominal one (UNVERIFIED, see module docs).
pub const TICK_TOLERANCE: f64 = 0.25;
/// SAE J2716 CRC4 table (polynomial x⁴+x³+x²+1).
const CRC4: [u8; 16] = [0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5];

/// SAE J2716 fast-channel CRC, "recommended" method: seed 0101, every data nibble, then one zero nibble.
pub fn j2716_crc(data: &[u8]) -> u8 {
    let c = data.iter().fold(5u8, |c, &n| CRC4[c as usize] ^ (n & 0xF));
    CRC4[c as usize]
}

/// What a SENT sensor transmits: unit time (tick), status/communication nibble and the data nibbles D1..D6.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SentSensor {
    pub tick_ns: f64,
    pub status: u8,
    pub data: [u8; 6],
}

impl SentSensor {
    /// Frame length in ticks: calibration 56, status + data + CRC nibbles 12 + value each.
    pub fn frame_ticks(&self) -> u32 {
        let nib = |n: u8| 12 + (n & 0xF) as u32;
        56 + nib(self.status) + self.data.iter().map(|&n| nib(n)).sum::<u32>() + nib(j2716_crc(&self.data))
    }
}

#[derive(Clone, Debug, Default)]
pub struct SentChannel {
    /// Sensor connected to the input pin (`None` = no sensor / not supplied: line idle, no frames).
    pub sensor: Option<SentSensor>,
    /// Frame in transmission (latched at its calibration pulse) and its start / end, board ns.
    tx: Option<SentSensor>,
    tx_start: f64,
    tx_end: f64,
    /// Receiver: frames starting before this instant are not evaluated (channel enabled mid-frame).
    armed_ns: f64,
    rdr: u32,
    rsr: u32,
    pub intstat: u32,
    /// Frames received successfully / with an error (diagnostics).
    pub frames_ok: u64,
    pub frames_err: u64,
}

#[derive(Clone, Debug, Default)]
pub struct Sent {
    pub ch: [SentChannel; NCH],
    /// Earliest `tx_end` across active channels (`0.0` when dirty, `f64::INFINITY` when no sensor is attached).
    next_end_ns: f64,
}

impl Sent {
    /// Module reset: the receivers forget their state; the sensors keep transmitting.
    pub fn soc_reset(&mut self) {
        for ch in self.ch.iter_mut() {
            *ch = SentChannel { sensor: ch.sensor, tx: ch.tx, tx_start: ch.tx_start, tx_end: ch.tx_end, frames_ok: ch.frames_ok, frames_err: ch.frames_err, ..Default::default() };
        }
        self.next_end_ns = 0.0;
    }
}

fn ch_of(off: u32) -> Option<(usize, u32)> {
    (CH0..CH0 + CH_STRIDE * NCH as u32).contains(&off).then(|| (((off - CH0) / CH_STRIDE) as usize, (off - CH0) % CH_STRIDE))
}

impl Tc1791HardwareState {
    fn sent_reg(&self, off: u32) -> u32 {
        self.seg_f000.get(SENT_BASE + off)
    }

    /// Nominal unit time of channel `c` from CPDR/CFDR (HYP decode, module docs); `None` while unconfigured.
    fn sent_nominal_tick_ns(&self, c: usize) -> Option<f64> {
        let base = CH0 + CH_STRIDE * c as u32;
        let (pdiv, div) = (self.sent_reg(base + CPDR) & 0x3FF, self.sent_reg(base + CFDR) & 0xFFF);
        (div != 0).then(|| (div + 1) as f64 * (pdiv + 1) as f64 / (56.0 * self.ffpi_hz() as f64) * 1e9)
    }

    /// Evaluates every frame that has ended by now.
    pub(crate) fn sent_update(&mut self) {
        let now = self.now_ns() as f64;
        if now < self.sent.next_end_ns {
            return;
        }
        let mut next = f64::INFINITY;
        for c in 0..NCH {
            loop {
                let ch = &mut self.sent.ch[c];
                let Some(sensor) = ch.sensor else {
                    ch.tx = None;
                    break;
                };
                if !(sensor.tick_ns.is_finite() && sensor.tick_ns >= 10.0) {
                    ch.tx = None;
                    break;
                }
                let frame_ns = sensor.frame_ticks() as f64 * sensor.tick_ns;
                if !(frame_ns.is_finite() && frame_ns > 0.0) {
                    ch.tx = None;
                    break;
                }
                let Some(tx) = ch.tx else {
                    // idle line → the sensor starts transmitting now
                    ch.tx = Some(sensor);
                    ch.tx_start = now;
                    ch.tx_end = now + frame_ns;
                    next = next.min(ch.tx_end);
                    break;
                };
                if ch.tx_end > now {
                    next = next.min(ch.tx_end);
                    break;
                }
                if now - ch.tx_end > 64.0 * frame_ns {
                    let skip = ((now - ch.tx_end) / frame_ns).floor();
                    let advance_ns = skip * frame_ns;
                    if advance_ns > 0.0 && advance_ns.is_finite() {
                        ch.tx_start += advance_ns;
                        ch.tx_end += advance_ns;
                    }
                }
                let (start, end) = (ch.tx_start, ch.tx_end);
                let mut next_end = end + frame_ns;
                if next_end <= end {
                    next_end = end + (end.abs() * f64::EPSILON).max(1.0);
                }
                ch.tx = Some(sensor);
                ch.tx_start = end;
                ch.tx_end = next_end;
                if start >= ch.armed_ns {
                    self.sent_receive(c, &tx);
                }
            }
        }
        self.sent.next_end_ns = next;
    }

    /// Receiver evaluation of one complete frame on hw channel `c`.
    fn sent_receive(&mut self, c: usize, f: &SentSensor) {
        let base = CH0 + CH_STRIDE * c as u32;
        let rcr = self.sent_reg(base + RCR);
        if rcr & 1 == 0 {
            return;
        }
        let nominal = self.sent_nominal_tick_ns(c);
        let n = ((rcr >> 8) & 0xF) as usize;
        let mut set = 0;
        if nominal.is_none_or(|t| (f.tick_ns / t - 1.0).abs() > TICK_TOLERANCE) {
            set |= FRI;
        } else if n != f.data.len() {
            set |= NNI;
        } else {
            // RDR: received data nibble i lands at the nibble position VIEW.RDNPi (identity on SC800S50).
            let view = self.sent_reg(base + VIEW);
            let mut rdr = 0u32;
            for (i, &d) in f.data.iter().enumerate() {
                rdr |= ((d & 0xF) as u32) << (4 * ((view >> (4 * i)) & 7));
            }
            // The modelled sensor always transmits the CRC of its data (no line faults are injected).
            let ch = &mut self.sent.ch[c];
            ch.rdr = rdr;
            ch.rsr = j2716_crc(&f.data) as u32 | ((f.status & 0xF) as u32) << 8;
            set |= RSI | RDI;
        }
        let ch = &mut self.sent.ch[c];
        if set & RSI != 0 {
            ch.frames_ok += 1;
        } else {
            ch.frames_err += 1;
        }
        self.sent_set_flags(c, set);
    }

    /// Sets INTSTAT bits and raises the service request of every newly set, enabled source (INP routing).
    fn sent_set_flags(&mut self, c: usize, bits: u32) {
        let base = CH0 + CH_STRIDE * c as u32;
        let (inten, inp) = (self.sent_reg(base + INTEN), self.sent_reg(base + INP));
        let ch = &mut self.sent.ch[c];
        let new = bits & !ch.intstat;
        ch.intstat |= bits;
        let mut nodes = 0u32;
        for (b, &grp) in GROUP_OF_BIT.iter().enumerate() {
            if new & inten & (1 << b) != 0 {
                nodes |= 1 << ((inp >> (4 * grp as u32)) & 0xF);
            }
        }
        for node in 0..16u32 {
            if nodes & (1 << node) != 0 && node < 2 {
                self.src_set_srr(SENT_BASE + 0x9FC - 4 * node);
            }
        }
    }

    /// Side-effect-free SENT register peek.
    pub(crate) fn sent_peek(&self, a: u32) -> Option<u32> {
        if self.variant != crate::memory::BoardVariant::Simos18Tc1791 {
            return None;
        }
        let off = a - SENT_BASE;
        if (RDR0..RDR0 + 4 * NCH as u32).contains(&off) {
            return Some(self.sent.ch[((off - RDR0) / 4) as usize].rdr);
        }
        let (c, r) = ch_of(off)?;
        match r {
            RSR | INTSTAT => {
                let ch = &self.sent.ch[c];
                Some(if r == RSR { ch.rsr } else { ch.intstat })
            }
            INTSET | INTCLR => Some(0),
            _ => None,
        }
    }

    /// SENT register read (`None` = plain register file).
    pub(crate) fn sent_read(&mut self, a: u32) -> Option<u32> {
        if self.variant != crate::memory::BoardVariant::Simos18Tc1791 {
            return None;
        }
        let off = a - SENT_BASE;
        if (RDR0..RDR0 + 4 * NCH as u32).contains(&off) {
            self.sent_update();
            return Some(self.sent.ch[((off - RDR0) / 4) as usize].rdr);
        }
        let (c, r) = ch_of(off)?;
        match r {
            RSR | INTSTAT => {
                self.sent_update();
                let ch = &self.sent.ch[c];
                Some(if r == RSR { ch.rsr } else { ch.intstat })
            }
            INTSET | INTCLR => Some(0),
            _ => None,
        }
    }

    /// SENT register write (`false` = plain register file stores it).
    pub(crate) fn sent_write(&mut self, a: u32, v: u32, m: u32) -> bool {
        if self.variant != crate::memory::BoardVariant::Simos18Tc1791 {
            return false;
        }
        let off = a - SENT_BASE;
        let Some((c, r)) = ch_of(off) else { return false };
        self.sent_update();
        match r {
            INTSET => self.sent_set_flags(c, v & m),
            INTCLR => self.sent.ch[c].intstat &= !(v & m),
            RCR => {
                let old = self.seg_f000.get(a);
                let new = (old & !m) | (v & m);
                self.seg_f000.set(a, new);
                if old & 1 == 0 && new & 1 == 1 {
                    // a receiver synchronises on the next calibration pulse
                    self.sent.ch[c].armed_ns = self.now_ns() as f64;
                }
            }
            RSR => {}
            _ => return false,
        }
        true
    }
}

impl crate::Simos18Board {
    /// Connects (or disconnects, `None`) a SAE J2716 sensor to SENT input `ch`. The next frame the sensor starts
    /// carries the new values (call it at every plant exchange with the sensor's current output).
    pub fn set_sent_sensor(&mut self, ch: usize, s: Option<SentSensor>) {
        let hw = self.hw_mut();
        if let Some(slot) = hw.sent.ch.get_mut(ch) {
            slot.sensor = s;
            hw.sent.next_end_ns = 0.0;
        }
    }

    pub fn sent(&self) -> &Sent {
        &self.hw().sent
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bitwise polynomial division (x⁴+x³+x²+1 = 0b11101), independent of the table.
    fn crc_bitwise(data: &[u8]) -> u8 {
        let mut r: u32 = 5;
        for &n in data.iter().chain(std::iter::once(&0u8)) {
            r = (r << 4) | (n as u32 & 0xF);
            for b in (4..8).rev() {
                if r & (1 << b) != 0 {
                    r ^= 0b11101 << (b - 4);
                }
            }
        }
        r as u8
    }

    #[test]
    fn j2716_crc_is_the_polynomial_division() {
        for x in 0..4096u32 {
            let d = [(x >> 8) as u8 & 0xF, (x >> 4) as u8 & 0xF, x as u8 & 0xF, 0xA, 0x3, !(x >> 8) as u8 & 0xF];
            assert_eq!(j2716_crc(&d), crc_bitwise(&d), "{d:?}");
        }
    }

    /// The firmware's own CRC table (serial-message check of the SENT driver) is the J2716 table.
    #[test]
    fn j2716_table_matches_the_firmware() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let Ok(img) = std::fs::read(root.join("vault/firmwares/ecu/stock/8V0906264K__0003.bin")) else {
            return;
        };
        let t = &img[0x5_D3D4..0x5_D3D4 + 256];
        for c in 0..16 {
            for n in 0..16 {
                assert_eq!(t[c * 16 + n], CRC4[c] ^ n as u8);
            }
        }
    }

    #[test]
    fn frame_length_counts_ticks_of_every_nibble() {
        let s = SentSensor { tick_ns: 3000.0, status: 0, data: [0; 6] };
        // 56 + status 12 + 6×12 + CRC (12 + crc)
        assert_eq!(s.frame_ticks(), 56 + 12 + 72 + 12 + j2716_crc(&[0; 6]) as u32);
    }

    #[test]
    fn invalid_or_tiny_tick_ns_does_not_hang_or_panic() {
        let mut b = crate::Simos18Board::new_variant(
            &vec![0u8; 0x40_0000],
            &vec![0u8; 0x4_0000],
            crate::memory::BoardVariant::Simos18Tc1791,
        )
        .unwrap();
        b.hw_mut().seg_f000.set(SENT_BASE + CH0 + CPDR, 0);
        b.hw_mut().seg_f000.set(SENT_BASE + CH0 + CFDR, 1); // non-zero CFDR enables nominal tick decode
        b.hw_mut().sent_write(SENT_BASE + CH0 + RCR, (6 << 8) | 1, u32::MAX); // arm ch 0, 6 nibbles
        b.set_sent_sensor(99, Some(SentSensor { tick_ns: 3000.0, status: 0, data: [1; 6] })); // OOB channel ignored

        for bad_tick in [0.0, 5.0, -100.0, f64::NAN, f64::INFINITY] {
            b.set_sent_sensor(0, Some(SentSensor { tick_ns: bad_tick, status: 0, data: [1; 6] }));
            b.hw_mut().sim_time_s += 0.1;
            b.hw_mut().sent_update();
            assert_eq!(b.sent().ch[0].frames_ok, 0, "invalid tick_ns {bad_tick} must not produce frames or loop");
        }

        // Minimum-valid 10 ns tick_ns over a 1 s jump (~650,000 frames) must fast-forward in O(1) without hanging!
        b.set_sent_sensor(0, Some(SentSensor { tick_ns: 10.0, status: 0, data: [2, 3, 4, 5, 6, 7] }));
        b.hw_mut().sent_update(); // starts frame 0 at current `now_ns`
        b.hw_mut().sim_time_s += 1.0;
        b.hw_mut().sent_update();
        assert!(
            b.sent().ch[0].frames_ok + b.sent().ch[0].frames_err >= 1,
            "O(1) fast-forward must advance past 1 s and evaluate latest frame without hanging"
        );
    }
}
