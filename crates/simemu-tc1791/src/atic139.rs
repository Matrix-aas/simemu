//! Continental ATIC139 (Freescale/NXP `SC900697AE / A2C00704400 / ATIC139/A2`, flanked by 2× Infineon
//! `2N06L64` 60 V MOSFETs + 2× ON `MBRS360` (`B36`) 60 V Schottkys of its PS switch-mode pre-regulator;
//! PHOTO `06K907425B` H13, `docs/SIMOS18_BOARD_HW.md` §3): system ASIC of the SIMOS18.1 board on SSC1 /
//! SLSO2. The firmware talks to three function blocks of it over the same chip select:
//!
//! | job (bus 1, table 0xD0006C54) | words | frame | function | model |
//! |---|---|---|---|---|
//! | 2 | 7 | `cmd 3..9 / 0xBFEx / 0xCFFE` | monitoring unit (MU, ECM3 level 3) | [`crate::monitor_device`] |
//! | 0 | 6 | `AAAA 1xxx 1xxx 2665 1xxx 2665` | "LS": 3 channels with diagnosis | [`Atic139`] |
//! | 1 | 4 | `AAAA 00xx F001 F001` | "PS": 2 switched outputs with diagnosis | [`Atic139`] |
//!
//! Evidence that the three are one chip (docs/SIMOS18_BOARD_HW.md §3): all three jobs use SSOC
//! mask 0x04 (VERIFIED-T, job table dump); the BSW driver descriptor 0x8004CBF0 lists the MU word
//! builder 0x80081724 together with the init routines of both 0xAAAA jobs (0x801127FC, 0x80112998);
//! the DAMOS failure list has exactly two ATIC139 communication errors, MCC_ATIC_139_LS
//! (IDX_ERR 0xA6) and MCC_ATIC_139_PS (0xA7), customer DTC 0x5653 / 0x5654. Which job is "LS" and
//! which "PS" is HYP (3 diagnosed channels = low-side stages, 2 switched outputs = supply outputs).
//!
//! # Protocol (VERIFIED-S, unmodified SC800S50)
//! 16 bit, MSB first, 1 MHz (CON 0x1F, BR 0x2C), full duplex with one word latency like the MU:
//! the reply to TX word k arrives in RX word k+1. TX words carry odd parity in bit 0 (0x801F4BC4),
//! except the 0xAAAA header. The job start 0x800A8CE8 splits both 0xAAAA jobs into two chip-select
//! bursts, `[AAAA cmd] [rest]` (VERIFIED-T: SLSO2 released after word 2), and the parsers take the
//! reply to `cmd` from the first word of the second burst: the reply register survives the select
//! edge. Words are therefore routed by their command nibble, not by frame position: MU 3..9, B, C;
//! LS 1, 2; PS 0, F; header 0xAAAA (the sets are disjoint; the chip-internal decoding is HYP).
//! * Both drivers require `RX[1] == 0xAAAA` (LS 0x800A9C00, PS 0x800A9704), otherwise they mark the
//!   frame as a communication error (LS state +0x1C = 2, PS +0x18 = 2) and the MCC error is set.
//! * LS (struct 0xD000A118, TX +0, RX +0xC, parser 0x800A9BE8): replies to `TX[1]`/`TX[4]` (command with
//!   bit 3 set) are status words: bits 4:2 and 14:13 = faults, bits 10:8 = per-channel "ok" (a zero
//!   bit of an enabled channel, mask +0x25 = 7, is a fault). Reply to `TX[2]` (bit 3 clear): bits 6:4
//!   = second fault type. Reply to `TX[3]` (0x2665): low byte compared with the window of config
//!   0x8004CC5C (bytes 3/4 = 0x00/0x78): > 0x00 for three frames sets a warning flag (+0x26).
//! * PS (struct 0xD000B258, TX +0, RX +8, parser 0x800A96EC): `TX[1]` = `0xC8 | on[2:1] | x[5:4]`
//!   (setter 0x800A9A88 writes the on bits). Reply = status word with one 3-bit code per channel
//!   (bit k of fields 1:0, 5:4, 9:8); the getter 0x800A99D8 reports diagnoses 1..3 executed without
//!   fault only for the code (0, 0, 1) (field 9:8 alone). Reply to 0xF001 is kept once (+0x10, getter
//!   0x800A9B7C, no consumer found).
//!
//! UNVERIFIED (not determinable from the firmware): what the channels drive, the meaning of the
//! command data bits, the reply bits the firmware does not decode (returned as 0), the content of
//! the 0xF001 reply (0), the chip's reaction to parity errors (ignored, counted), fault behaviour
//! (always healthy: no load model, the channel assignment is unknown).

/// Header word of the LS/PS frames; echoed one word later.
pub const HEADER: u16 = 0xAAAA;
/// LS status (reply to a `1xxx` command with bit 3 set): three channels ok (bits 10:8), no fault.
pub const LS_STATUS_OK: u16 = 0x0700;
/// PS status: code (0, 0, 1) (field 9:8 alone) for both channels = all diagnoses done, no fault.
pub const PS_STATUS_OK: u16 = 0x0300;

#[derive(Debug, Clone, Default)]
pub struct Atic139 {
    /// Word prepared for the next 16 clocks (one word latency, kept across select edges).
    out: u16,
    /// Last LS / PS command words (opaque, UNVERIFIED meaning).
    pub ls_cmd: u16,
    pub ps_cmd: u16,
    /// LS / PS status words served (diagnostics).
    pub ls_status: u64,
    pub ps_status: u64,
    pub parity_errors: u64,
}

impl Atic139 {
    /// Word for the LS/PS blocks (the others go to the MU block).
    pub fn claims(w: u16) -> bool {
        w == HEADER || matches!(w >> 12, 0x0 | 0x1 | 0x2 | 0xF)
    }

    /// PS outputs commanded on (`TX[1]` bits 2:1, setter 0x800A9A88).
    pub fn ps_on(&self) -> [bool; 2] {
        [self.ps_cmd & 2 != 0, self.ps_cmd & 4 != 0]
    }

    /// One 16-bit word claimed by [`Self::claims`]: returns the reply to the previous one.
    pub fn exchange(&mut self, mosi: u16) -> u16 {
        let miso = self.out;
        self.out = self.reply(mosi);
        miso
    }

    fn reply(&mut self, w: u16) -> u16 {
        if w == HEADER {
            return HEADER;
        }
        if w.count_ones() & 1 == 0 {
            self.parity_errors += 1;
        }
        match w >> 12 {
            0x0 => {
                self.ps_cmd = w;
                self.ps_status += 1;
                PS_STATUS_OK
            }
            0x1 => {
                self.ls_cmd = w;
                if w & 8 != 0 {
                    self.ls_status += 1;
                    LS_STATUS_OK
                } else {
                    0
                }
            }
            _ => 0, // 0x2xxx (LS register read, low byte 0 = below the window) and 0xFxxx (PS, unknown content)
        }
    }
}

#[cfg(test)]
mod tests {
    //! The MC side is a transcription of the firmware parsers (addresses in the module docs).
    use super::*;

    /// Words of one job as the firmware sends them (two select bursts, the chip does not care).
    fn frame(c: &mut Atic139, tx: &[u16]) -> Vec<u16> {
        tx.iter().map(|&w| c.exchange(w)).collect()
    }

    #[test]
    fn ls_frame_decodes_as_three_healthy_channels() {
        let mut c = Atic139::default();
        let rx = frame(&mut c, &[0xAAAA, 0x177A, 0x1773, 0x2665, 0x177A, 0x2665]);
        assert_eq!(rx[1], 0xAAAA, "0x800A9C00");
        let mask = 7; // +0x25
        for s in [rx[2], rx[5]] {
            assert_eq!((s >> 2) & 7, 0, "fault bits 4:2");
            assert_eq!((s >> 13) & 3, 0, "fault bits 14:13");
            assert_eq!(!(s >> 8) & mask, 0, "ok bits 10:8 of enabled channels");
        }
        assert_eq!((rx[3] >> 4) & 7, 0, "second fault type, 0x800A9C4E");
        assert_eq!(rx[4] & 0xFF, 0, "not above the window low limit (config byte 3 = 0)");
        assert_eq!((c.ls_status, c.parity_errors), (2, 0));
    }

    #[test]
    fn ps_frame_reports_all_diagnoses_executed_without_fault() {
        let mut c = Atic139::default();
        let rx = frame(&mut c, &[0xAAAA, 0x00FE, 0xF001, 0xF001]);
        assert_eq!(rx[1], 0xAAAA, "0x800A9704");
        assert_eq!(c.ps_on(), [true, true]);
        for ch in 0..2 {
            // Per-channel code (field 1:0, 5:4, 9:8) as decoded at 0x800A9740..0x800A9850.
            let code = (rx[2] >> ch & 1, rx[2] >> (4 + ch) & 1, rx[2] >> (8 + ch) & 1);
            assert_eq!(code, (0, 0, 1), "channel {ch}: executed 1..3, no fault (getter 0x800A99D8)");
        }
    }

    #[test]
    fn words_are_routed_by_command_nibble() {
        // TX words of the MU job (monitor_device.rs) never reach the LS/PS blocks and vice versa.
        for w in [0x305E, 0x4003, 0x5A3B, 0x65C2, 0x71DC, 0x8C1E, 0x9F3E, 0xBFE1, 0xBFE4, 0xCFFE] {
            assert!(!Atic139::claims(w), "{w:04X}");
        }
        for w in [0xAAAA, 0x177A, 0x1773, 0x2665, 0x00FD, 0xF001] {
            assert!(Atic139::claims(w), "{w:04X}");
        }
        let mut c = Atic139::default();
        frame(&mut c, &[0xAAAA, 0x00FD]);
        assert_eq!(c.ps_on(), [false, true]);
    }
}
