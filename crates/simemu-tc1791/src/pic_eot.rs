//! External engine-off timer of the SIMOS18.1 board: Microchip PIC12LF1822 (SOIC-8 `12LF1822 E/SN`,
//! PHOTO `06K907425B` H13, `docs/SIMOS18_BOARD_HW.md` §4; pins 2 `RA5` and 5 `RA2` are physically
//! unconnected test pads — zero wake/alarm line exists) on SSC0 / SLSO6.
//!
//! Function (VERIFIED-FR, ENSS 33.4 "Engine off duration", FR p.8334..8338): STATE_T_ES_RD /
//! STATE_T_ES_VLD_RD = "status / validity of external clock read from PIC", STATE_T_ES_TYP_INFO
//! "1: engine-off-time from PIC", "check of the SPI communication error after reset". SC8S50
//! calibration: LC_STATE_T_ES_CLC_SWI = 1 (engine-off time from the PIC, not from CAN),
//! LC_STATE_T_ES_TYPE_REQ = 0 (PIC, not ATIC), C_T_ES_HW_MAX = 172000 s ("24 h or 48 h"). Failures
//! T_ES_HW / T_ES_VLD_CHK / ... map to OBD P2610 (customer 0x59DA / 0x53AB).
//!
//! # Protocol (VERIFIED-S, unmodified SC800S50; driver struct 0xD000B288)
//! SSC0, 8 bit, MSB first, 1 MHz (CON 0x17, BR 0x2C), SSOC 0x4000 (job 2 of bus 0, table
//! 0xD0006C14), one 6-byte job per 10 ms: TX `cmd 00 00 00 00 00` (builder 0x800ABBE4, command
//! selector 0x800ABB18) in two chip-select bursts `[cmd 00] [00 00 00 00]` (job start 0x800A8CE8
//! splits it, VERIFIED-T), RX parsed by 0x800ABD40:
//! * RX0 (shifted out while the command arrives): `0x20 | version` — (RX0 & 0xE0) == 0x20 required,
//!   the low nibble is captured once (+0x25).
//! * RX2..RX4: 24-bit value, little endian (u32 +0x00, getter 0x800ABCF0).
//! * RX5: bits 7:5 = reply type, bits 4:3 = clock state (+0x1F = STATE_T_ES_RD), bits 2:0 = validity
//!   (0 -> valid (+0x20 = 1); 7 or even -> 4; odd -> 2).
//!
//! | command | reply type | firmware use |
//! |---|---|---|
//! | `0x50 \| m<<2 \| p` read clock (m = 0..3 from 0x800ABB18 cases 0..3, p = config 0x8004F426 & 3 = 1) | `0b110` (0xC0) | cyclic 0x51 |
//! | `0x90` reset clock | `0b100` (0x80) | 0x800ABA04 when the value approaches 86400·(p+1) s or the state is not 0 |
//! | `0xD0..0xD2` read diagnosis | `0b101` (0xA0), RX5 & 0x1F -> +0x21 | ASW sequence 0x801A8460.. (`0xD2`, then `0x5D`; traced only while the replies were invalid) |
//!
//! 86400·(p+1) is the driver's limit (0x800ABA48: 0x15180 · (p + 1)), i.e. p selects the 24 h / 48 h
//! range of C_T_ES_HW_MAX.
//!
//! UNVERIFIED (no firmware evidence, the PIC program is not in the dump): the version nibble (0),
//! the meaning of m, the content of the 0xDx value (clock value) and status (0), saturation at the
//! range limit, state 0 / validity 0 in all cases, the clock accuracy ([`PicEot::rate`], nominal 1.0;
//! the firmware checks it against its own clock with C_RATIO_T_ES_CHK_PLAUS), power-on value 0.
//! The PIC is supplied from KL30 (it counts while the ECU is off): its state survives an MC reset.
//! Framing: a non-zero byte starts a command (the MC only sends 0x00 as dummy, VERIFIED-T); the reply
//! bytes follow in order whatever the select bursts are (HYP: the PIC program is unknown).

/// Board wiring: SSC0, slave select SLSO6.
pub const EOT_SSC: usize = 0;
pub const EOT_SLSO: u8 = 6;
/// Reply bytes after the command byte: RX1..RX5.
pub const REPLY_LEN: usize = 5;
/// UNVERIFIED version nibble of the RX0 identification byte.
pub const VERSION: u8 = 0;
/// Byte shifted out while no reply is pending (RX0 of every job).
pub const IDLE: u8 = 0x20 | VERSION;

#[derive(Debug, Clone)]
pub struct PicEot {
    /// Clock rate relative to simulated time (calibration knob for oscillator tolerance).
    pub rate: f64,
    /// Simulated time (ns) at which the clock value was 0.
    zero_ns: f64,
    /// Range bits p of the last read command (86400·(p+1) s).
    range: u8,
    reply: [u8; REPLY_LEN],
    pos: usize,
    pub frames: u64,
    pub resets: u64,
    pub unknown_cmds: u64,
}

impl Default for PicEot {
    fn default() -> Self {
        Self { rate: 1.0, zero_ns: 0.0, range: 1, reply: [0; REPLY_LEN], pos: REPLY_LEN, frames: 0, resets: 0, unknown_cmds: 0 }
    }
}

impl PicEot {
    /// Clock value in seconds at `now_ns` (saturates at the range limit).
    pub fn seconds(&self, now_ns: u64) -> u32 {
        let s = ((now_ns as f64 - self.zero_ns) * 1e-9 * self.rate).max(0.0) as u64;
        s.min(86_400 * (self.range as u64 + 1)) as u32
    }

    /// Set the clock to `s` seconds at `now_ns` (bench: engine-off time before the ECU wakes up).
    pub fn set_seconds(&mut self, s: f64, now_ns: u64) {
        if s.is_finite() && self.rate.is_finite() && self.rate > 0.0 {
            self.zero_ns = now_ns as f64 - s.max(0.0) * 1e9 / self.rate;
        }
    }

    /// One 8-bit frame on SLSO6.
    pub fn exchange(&mut self, mosi: u8, now_ns: u64) -> u8 {
        if mosi != 0 {
            self.frames += 1;
            self.reply = self.answer(mosi, now_ns);
            self.pos = 0;
            return IDLE;
        }
        let b = self.reply.get(self.pos).copied().unwrap_or(IDLE);
        self.pos += 1;
        b
    }

    fn answer(&mut self, cmd: u8, now_ns: u64) -> [u8; REPLY_LEN] {
        let typ = match cmd & 0xF0 {
            0x50 => {
                self.range = cmd & 3;
                0b110
            }
            0x90 => {
                self.resets += 1;
                self.zero_ns = now_ns as f64;
                0b100
            }
            0xD0 => 0b101,
            _ => {
                self.unknown_cmds += 1;
                return [0; REPLY_LEN];
            }
        };
        let v = self.seconds(now_ns).to_le_bytes();
        [0, v[0], v[1], v[2], typ << 5]
    }
}

#[cfg(test)]
mod tests {
    //! The MC side is a transcription of the parser 0x800ABD40.
    use super::*;

    struct Parsed {
        version: Option<u8>,
        value: u32,
        validity: u8,
        reply_type: u8,
    }

    fn frame(p: &mut PicEot, cmd: u8, now: u64) -> Parsed {
        let rx: Vec<u8> = [cmd, 0, 0, 0, 0, 0].iter().map(|&b| p.exchange(b, now)).collect();
        let validity = match rx[5] & 7 {
            0 => 1,
            v if v == 7 || v & 1 == 0 => 4,
            _ => 2,
        };
        Parsed {
            version: (rx[0] & 0xE0 == 0x20).then_some(rx[0] & 0xF),
            value: u32::from_le_bytes([rx[2], rx[3], rx[4], 0]),
            validity,
            reply_type: rx[5] & 0xE0,
        }
    }

    #[test]
    fn counts_seconds_resets_and_answers_every_command_type() {
        let mut p = PicEot::default();
        p.set_seconds(3600.0, 0);
        let r = frame(&mut p, 0x51, 0);
        assert_eq!((r.version, r.value, r.validity, r.reply_type), (Some(0), 3600, 1, 0xC0));
        assert_eq!(frame(&mut p, 0x51, 2_500_000_000).value, 3602, "1 s per second");
        assert_eq!(frame(&mut p, 0xD2, 2_500_000_000).reply_type, 0xA0);
        assert_eq!(frame(&mut p, 0x90, 3_000_000_000).reply_type, 0x80, "reset acknowledged (0x800ABDE8)");
        assert_eq!(frame(&mut p, 0x51, 13_000_000_000).value, 10, "counts from the reset");
        p.set_seconds(1e6, 0);
        assert_eq!(frame(&mut p, 0x51, 0).value, 172_800, "48 h range with p = 1");
        assert_eq!(frame(&mut p, 0x50, 0).value, 86_400, "24 h range with p = 0");
        assert_eq!((p.resets, p.unknown_cmds), (1, 0));
    }
}
