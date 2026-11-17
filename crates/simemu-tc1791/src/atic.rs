//! Continental ATIC power-stage driver ICs of the SIMOS18.1 board, modelled as independent
//! devices behind their serial interfaces. Everything here is derived from the unmodified
//! SC800S50 firmware (addresses below; the same drivers run in SC8O30/SC8O40/SC8H83) and the
//! Funktionsrahmen (FR p.N). Points the firmware cannot reveal are marked UNVERIFIED.
//!
//! | chip     | silicon on `06K907425B` H13 (`A2C80473700-04`) | function (FR p.5679)                       | link                       | disable inputs                    |
//! |----------|------------------------------------------------|--------------------------------------------|----------------------------|-----------------------------------|
//! | ATIC140  | `A2C00757700 / ATIC140S B1` ×2 (PowerQFP-64)   | 26 low-side/ignition stages, starter EN7_8 | MSC (downstream) + MSC upstream | NDIS (MU NDIS1), LS_DIS (LS_DIS0), EN7_8 |
//! | ATIC121  | `ST A2C00057927 / ATIC121 B3` (+ `30N10L25` 100V boost, `R022` 22 mΩ, 2× `VNL10`, 2× `VNL17`) | GDI injector driver (2 banks) | SSC0 / SLSO0, 16 bit | NDIS (MU NDIS0), DIS (LS_DIS0) |
//! | ATIC111  | `ST A2C00693000` ×3 (PowerSSO-24: `T105a/90,91` J338, `T105a/86,87` N493, `T105a/88,89` V465) | H-bridge chain (throttle = channel 0) | SSC1 / SLSO3, 16 bit | EN (MU NDIS1), DIS (MTC_DIS) |
//!
//! # ATIC140 (driver 0x800AA0D8.., config builder 0x80112D60.., dump parser 0x800AA868..)
//! Downstream MSC command frames (DSC.NBC = 26 bits, DC register) carry a 4-bit code in bits 3:0:
//! * code 2 (word A): bits 7:4 = P24, 9:8 = P27, 15:10 = P18, 21:16 = P1A (0x80112F5E..F88).
//! * code 4 (word B): bits 12:4 = M (9 bit), 21:13 = !M, 25:22 = P21 (0x80112F8A..FDC, 0x800AB192).
//! * code 1 (word C): bits 7:4 = 0xA configure / 0x5 start hardware self check (0x800AB306),
//!   bit 8 = P1C, 9 = P1D, 11:10 = P7A, 13:12 = P7B, 14 = P1E, 15 = P1F (0x80112FDE..801E).
//! * bit 25 set, code >= 8: read the diagnosis dump (0x0200000B/8/E/D every 10 ms).
//!
//! Upstream frames per command = the DMA TREL the driver programs before sending it
//! (0x8008714E at 0x80113036..: A 1, B 1, C 2, read 24). The 24-byte dump (parser 0x800AA89C..,
//! 0x800AAB8C.., 0x800AAD8E..): bytes 0..3 / 4..7 / 8..11 = 26-bit channel vectors A/B/C, with the
//! configuration read back in their top bits and in bytes 12..14 exactly as the parser compares
//! them; byte 23 = disable-input states (bit 0 NDIS high, bit 1 LS_DIS high, bit 2 EN7_8 high,
//! 0x800AAD94 / 0x800AADE4); byte 14 bits 1:0 = hardware self check (bit 1 done, bit 0 failed,
//! 0x800AADB4). Data frames (SRL 16 + SRH 10 bits) carry the 26 output commands.
//!
//! # ATIC121 (driver 0x80098E00.., frame builders 0x8009958C..0x80099A9C, parsers 0x80099B44..)
//! 16-bit word: bits 15:11 address, 10:1 data, bit 0 odd parity (0x8008718C). Address 0x17 =
//! read: data = read address. Full duplex with one word latency: the response to word k is
//! shifted out during word k+1 (proven by the channel-diagnosis job 0x8009958C/0x80099EF4: the
//! diagnosis vector of channel k is taken from the reply to "read 3+k", the extension register
//! from the reply to "read 0x1C+k" only for chip 0x27). Response to a write = previous register
//! content (UNVERIFIED, no job evaluates such a reply). Response bit 12 = SPI error of the
//! previous word (0x80099BDE: bit 12 set -> only bit 12 valid). Read map:
//! * 0x0B: bit 0 disable-path fault, bit 7 stages disabled by NDIS, bit 9 disabled by DIS;
//! * 0x0C: bit 4 NDIS high, bit 5 DIS high.
//!
//! Evidence: SC8S50/SC8O40 SOPC job 0x800996BC reads 0x0B, 0x0B, 0x0C, 0, 0, 0 and the parser
//! 0x80099FBE takes reply 2 & 0x1001 | reply 3 & 0x1030 (consumer 0x8009BE84: bit 4 set = NDIS
//! enabled, bit 5 set = DIS active, bit 0 set = fault); SC8H83 reads 0x0B after writing reg 0x18
//! (TX B816 C004 C010 B816 B801 B801, parser 0x80094758 stores reply 1) and its SOPC
//! (0x8008178E) reports bit 7 set = disabled via NDIS0, bit 9 set = disabled via DIS. 0x03..0x06 channel
//! diagnosis, 0x07..0x0A channel counters, 0x1C..0x1F extended diagnosis (chip 0x27). The
//! firmware expects chip id 0x27, revision 1 (CAL 0x80041D66/67, compare 0x80099C42). Identification
//! job = init state 1 of the driver state machine (builder 0x800F7478): TX read 0x0B, write 0x18 = 2,
//! read 0x01, read 0x02, read 0x0B, read 0x00; parser 0x80099BC4 takes the chip id from the low byte of
//! the reply to "read 0x01" (RX word 3) and, for chip 0x27, the revision from the low nibble of the
//! reply to "read 0x02" (RX word 4, high nibble stored as a second field). Only a match moves the
//! driver on (state 2, configuration); a mismatch repeats the job and leaves the injector stage
//! unconfigured (VERIFIED-T: with id/revision answered at 0x00/0x01 the firmware stored id 1, rev 0 and
//! never scheduled an injection pulse). Read 0x00 closes the job as a dummy word (reply unused).
//!
//! # ATIC111 (vtable 0x8004F4A4, send 0x801F561C, parser 0x800ABE98)
//! Daisy chain of N = 3 single-channel stages, 16-bit LSB-first shift registers (SSC1 job with
//! CON.HB = 0). Frame of N+1 words per chip select: TB [header 0xAAAA, cmd ch2, cmd ch1, cmd ch0],
//! `RB [status ch2, status ch1, status ch0, header]` (the MC requires `RB[N] == 0xAAAA`). Status word:
//! bit 7 output stage enabled (SOPC, 0x800AC036), bits 15:12 diagnosis executed / 11:8 no fault
//! for four diagnoses (a word with an all-zero pair is rejected), bits 6/5/4/2 must be set in
//! normal operation, bits 1:0 = executed / no fault of a fifth diagnosis.
//! UNVERIFIED: the diagnosis assignment of the bit pairs, the meaning of the command bits other
//! than mode 3:2.

/// Fault state of an electrical load (wire harness + actuator), supplied by the vehicle model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LoadFault {
    #[default]
    None,
    OpenLoad,
    ShortToGround,
    ShortToBattery,
}

/// Electrical load at a driver output (coil, injector, solenoid, relay, motor).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Load {
    pub r_ohm: f32,
    pub l_h: f32,
    pub fault: LoadFault,
}

impl Load {
    pub const fn new(r_ohm: f32, l_h: f32) -> Self {
        Self { r_ohm, l_h, fault: LoadFault::None }
    }
    /// Steady-state current at supply `ubat` with the output switched on (0 for open load).
    pub fn current_a(&self, ubat: f32) -> f32 {
        match self.fault {
            LoadFault::OpenLoad | LoadFault::ShortToGround => 0.0,
            LoadFault::ShortToBattery => f32::INFINITY,
            LoadFault::None => ubat / self.r_ohm.max(1e-3),
        }
    }
}

#[inline]
fn odd_parity_ok(w: u16) -> bool {
    w.count_ones() & 1 == 1
}

// ---------------------------------------------------------------------------------------------
// ATIC140
// ---------------------------------------------------------------------------------------------

pub const ATIC140_CHANNELS: usize = 26;
pub const ATIC140_DUMP_LEN: usize = 24;
/// Loads wired between a high-side and a low-side stage of one ATIC140 (both coil ends on the chip), as a channel
/// mask per chip. HPFP valve N276: high side = chip 1 stage 25, low side = chip 1 stage 24 — VERIFIED-S: the VCV
/// power-stage diagnosis 0x808BB868 (FUSL 43.29, FR p.12174) reads ACTION_INFR_GetElDiagVcvHss = 0x808AD95C
/// (getter 0x800A9F68 channel id 0x119) and GetElDiagVcvLss = 0x808AD978 (id 0x118) into 0xD0001320..23.
pub const ATIC140_BRIDGED: [u32; 2] = [0, 1 << 24 | 1 << 25];

/// Hardware inputs of an ATIC140 (levels at its pins).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Atic140Pins {
    /// NDIS high = stages enabled.
    pub ndis: bool,
    /// LS_DIS high = low-side stages disabled.
    pub ls_dis: bool,
    /// EN7_8 high = starter stages 7/8 enabled.
    pub en78: bool,
}

#[derive(Debug, Clone)]
pub struct Atic140 {
    /// Raw configuration words as last received (code 2, 4, 1).
    pub word_a: u32,
    pub word_b: u32,
    pub word_c: u32,
    /// Result of the hardware self check of the disable paths is available (passed). The chip runs
    /// it at power-up and again on the start command (word C sub 0x5). UNVERIFIED power-up run:
    /// the MC treats "no result" as SOPC failure and re-requests the check only after a failed
    /// result (0x80081206 -> 0x800AB568), so a real chip must deliver one on its own.
    hwsc_done: bool,
    /// Output commands of the last downstream data frame (bit n = channel n).
    pub data: u32,
    pub loads: [Load; ATIC140_CHANNELS],
    /// Channels whose load is bridged to another stage of this chip ([`ATIC140_BRIDGED`]).
    pub bridged: u32,
    pub commands: u64,
    pub dumps: u64,
    pub rejected: u64,
}

impl Default for Atic140 {
    fn default() -> Self {
        Self::new()
    }
}

impl Atic140 {
    pub fn new() -> Self {
        // UNVERIFIED typical loads: ignition coil primaries / relay coils / solenoids.
        Self {
            word_a: 0,
            word_b: 0,
            word_c: 0,
            hwsc_done: true,
            data: 0,
            loads: [Load::new(10.0, 0.01); ATIC140_CHANNELS],
            bridged: 0,
            commands: 0,
            dumps: 0,
            rejected: 0,
        }
    }

    /// One downstream command frame (`bits` LSBs valid). Returns the upstream bytes the chip
    /// sends in reply.
    pub fn command(&mut self, w: u32, pins: Atic140Pins) -> Vec<u8> {
        self.commands += 1;
        let w = w & 0x03FF_FFFF;
        if w & (1 << 25) != 0 && w & 0xF >= 8 {
            self.dumps += 1;
            return self.dump(pins).to_vec();
        }
        match w & 0xF {
            2 => self.word_a = w,
            4 => {
                let m = (w >> 4) & 0x1FF;
                if (w >> 13) & 0x1FF != !m & 0x1FF {
                    self.rejected += 1; // value/complement mismatch: keep the old mask
                    return vec![0; 1];
                }
                self.word_b = w;
            }
            1 => {
                if (w >> 4) & 0xF == 0x5 {
                    self.hwsc_done = true; // self check of the disable paths, nothing failing
                } else {
                    self.word_c = w;
                }
                return vec![w as u8; 2]; // UNVERIFIED reply content (not evaluated by the MC)
            }
            _ => self.rejected += 1,
        }
        vec![w as u8] // UNVERIFIED reply content (not evaluated by the MC)
    }

    fn cfg6(v: u32) -> u8 {
        // bit 0 -> 4, bit 1 -> 3, bit 2 -> 2, bits 5:3 -> 7:5 (parser 0x800AA89C)
        (((v & 1) << 4) | ((v >> 1 & 1) << 3) | ((v >> 2 & 1) << 2) | ((v >> 3 & 7) << 5)) as u8
    }

    /// Per-channel diagnosis vectors (A, B, C), 26 bits each.
    /// 3-bit diagnosis code (A, B, C) per channel. The MC classifier 0x800AAB8C files
    /// (0,0,1) as "short-to-battery / short-to-ground / open-load diagnosis valid, no fault"
    /// (struct +0x28/+0x30/+0x38 set, read by the ignition diagnosis getter 0x800A9F68 as
    /// STATE_DIAG_IGN_H_RNG 0x100/0x200/0x400, FR p.12578); faults use C = 0: (1,0,0) short to
    /// battery (+0x24 = 0x001), (0,1,0) short to ground (+0x2C = 0x002), (1,1,0) open load
    /// (+0x34 = 0x004). UNVERIFIED: the chip's own naming of the codes.
    ///
    /// Bridged loads (coil between a high-side and a low-side stage, [`Self::bridged`]): the off-state open-load
    /// check of each stage expects its output pulled towards the opposite rail by the load; with the other coil end
    /// on the other (off) stage both outputs float, so an intact coil reads "open load" on both stages and a broken
    /// coil leaves each output at its own bias = "no fault". The VCV diagnosis evaluates exactly that (FR p.12179:
    /// "due to the design of ATIC 140" the good case is OC condition + OC error on both switches, all-valid-no-error
    /// on both is the open-circuit symptom). The bias mechanism itself is UNVERIFIED (no ATIC140 data sheet).
    fn vectors(&self) -> (u32, u32, u32) {
        let (mut a, mut b, mut c) = (0u32, 0u32, 0u32);
        let coil_open = |mask: u32| (0..ATIC140_CHANNELS).any(|k| mask >> k & 1 != 0 && self.loads[k].fault == LoadFault::OpenLoad);
        for (n, l) in self.loads.iter().enumerate() {
            let bit = 1 << n;
            if self.bridged & bit != 0 && matches!(l.fault, LoadFault::None | LoadFault::OpenLoad) {
                if coil_open(self.bridged) {
                    c |= bit;
                } else {
                    (a, b) = (a | bit, b | bit);
                }
                continue;
            }
            match l.fault {
                LoadFault::None => c |= bit,
                LoadFault::ShortToBattery => a |= bit,
                LoadFault::ShortToGround => b |= bit,
                LoadFault::OpenLoad => {
                    a |= bit;
                    b |= bit
                }
            }
        }
        (a, b, c)
    }

    /// The 24-byte diagnosis dump.
    pub fn dump(&self, pins: Atic140Pins) -> [u8; ATIC140_DUMP_LEN] {
        let (wa, wb, wc) = (self.word_a, self.word_b, self.word_c);
        let p24 = (wa >> 4) & 0xF;
        let p27 = (wa >> 8) & 3;
        let p18 = (wa >> 10) & 0x3F;
        let p1a = (wa >> 16) & 0x3F;
        let m = (wb >> 4) & 0x1FF;
        let (p1c, p1d) = ((wc >> 8) & 1, (wc >> 9) & 1);
        let (p7a, p7b) = ((wc >> 10) & 3, (wc >> 12) & 3);
        let (p1e, p1f) = ((wc >> 14) & 1, (wc >> 15) & 1);
        let (va, vb, vc) = self.vectors();
        let mut d = [0u8; ATIC140_DUMP_LEN];
        for (i, v) in [va, vb, vc].into_iter().enumerate() {
            d[4 * i..4 * i + 4].copy_from_slice(&(v & 0x03FF_FFFF).to_le_bytes());
        }
        d[3] |= Self::cfg6(p18);
        d[7] |= Self::cfg6(p1a);
        d[11] |= (p24 << 2 | p1c << 6 | p1d << 7) as u8;
        d[12] = ((m & 7) | ((m >> 3) & 0xF) << 4) as u8;
        d[13] = ((m >> 7) & 3 | p7a << 2 | p7b << 4 | p27 << 6) as u8;
        d[14] = ((self.hwsc_done as u32) << 1 | p1e << 6 | p1f << 7) as u8;
        d[23] = pins.ndis as u8 | (pins.ls_dis as u8) << 1 | (pins.en78 as u8) << 2;
        d
    }

    /// Output stage `n` conducting: commanded by the data frame and not disabled by a pin.
    /// UNVERIFIED: NDIS and LS_DIS gate all stages. EN7_8 (FR p.5679 "Starter#1 ATIC140 (EN7_8)",
    /// chip channels 7/8 counted from 1) gates data bits 6/7: SC800S50 drives the two starter relays
    /// through MSC1 DD bits 6 and 7 (wrappers 0x800A5B80 / 0x800A5B8C, called only by the ENSS starter
    /// relay control 0x801AC89C.. and its init 0x800ED0B6), bit 8 is a GPTA (ALTIN) data bit.
    pub fn output_on(&self, n: usize, pins: Atic140Pins) -> bool {
        let gated = !pins.ndis || pins.ls_dis || ((n == 6 || n == 7) && !pins.en78);
        n < ATIC140_CHANNELS && self.data >> n & 1 != 0 && !gated
    }
}

// ---------------------------------------------------------------------------------------------
// ATIC121
// ---------------------------------------------------------------------------------------------

pub const ATIC121_CHANNELS: usize = 4;
/// Chip identification the SC8 software family expects (CAL 0x80041D66 = 0x27, rev 1).
pub const ATIC121_CHIP_ID: u16 = 0x27;
pub const ATIC121_REVISION: u16 = 1;
pub const ATIC121_READ: u16 = 0x17;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Atic121Pins {
    /// NDIS high = enabled.
    pub ndis: bool,
    /// DIS high = disabled.
    pub dis: bool,
}

#[derive(Debug, Clone)]
pub struct Atic121 {
    /// Write registers (10-bit data).
    pub regs: [u16; 32],
    out: u16,
    pub loads: [Load; ATIC121_CHANNELS],
    pub words: u64,
    pub parity_errors: u64,
}

impl Default for Atic121 {
    fn default() -> Self {
        Self::new()
    }
}

impl Atic121 {
    pub fn new() -> Self {
        // UNVERIFIED: GDI injector coil (~1.5 Ohm, ~1 mH class).
        Self { regs: [0; 32], out: 0, loads: [Load::new(1.5, 0.001); ATIC121_CHANNELS], words: 0, parity_errors: 0 }
    }

    pub fn select(&mut self) {
        // Nothing frame-related: every word is a complete command.
    }

    fn read(&self, r: u16, pins: Atic121Pins) -> u16 {
        match r {
            // Identification job 0x800F7478 / parser 0x80099BC4 (module docs).
            0x01 => ATIC121_CHIP_ID,
            0x02 => ATIC121_REVISION,
            0x00 => 0,
            0x0B => (!pins.ndis as u16) << 7 | (pins.dis as u16) << 9,
            0x0C => (pins.ndis as u16) << 4 | (pins.dis as u16) << 5,
            // UNVERIFIED encodings: 0 = no fault / no event recorded.
            0x03..=0x06 => match self.loads[(r - 3) as usize].fault {
                LoadFault::None => 0,
                LoadFault::OpenLoad => 1 << 2,
                LoadFault::ShortToGround => 1 << 1,
                LoadFault::ShortToBattery => 1 << 0,
            },
            0x07..=0x0A | 0x1C..=0x1F => 0,
            _ => self.regs[r as usize & 31],
        }
    }

    /// One 16-bit word: returns the reply to the previous word and executes `mosi`.
    pub fn exchange(&mut self, mosi: u16, pins: Atic121Pins) -> u16 {
        self.words += 1;
        let miso = self.out;
        if !odd_parity_ok(mosi) {
            self.parity_errors += 1;
            self.out = 1 << 12;
            return miso;
        }
        let (addr, data) = ((mosi >> 11) as usize, (mosi >> 1) & 0x3FF);
        self.out = if addr == ATIC121_READ as usize {
            self.read(data, pins)
        } else {
            std::mem::replace(&mut self.regs[addr], data)
        } & 0x0FFF;
        miso
    }

    /// Injector stages enabled by the hardware disable inputs.
    pub fn enabled(&self, pins: Atic121Pins) -> bool {
        pins.ndis && !pins.dis
    }
}

// ---------------------------------------------------------------------------------------------
// ATIC111 chain
// ---------------------------------------------------------------------------------------------

pub const ATIC111_CHANNELS: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Atic111Pins {
    /// EN high = enabled.
    pub en: bool,
    /// DIS high = disabled.
    pub dis: bool,
}

#[derive(Debug, Clone)]
pub struct Atic111 {
    /// Latched command words, index = channel (= chain stage, stage 0 next to MTSR).
    pub cmd: [u16; ATIC111_CHANNELS],
    /// Shift registers of the stages.
    sr: [u16; ATIC111_CHANNELS],
    words: usize,
    pub loads: [Load; ATIC111_CHANNELS],
    pub frames: u64,
}

impl Default for Atic111 {
    fn default() -> Self {
        Self::new()
    }
}

impl Atic111 {
    pub fn new() -> Self {
        // UNVERIFIED: throttle / wastegate actuator DC motors.
        Self { cmd: [0; ATIC111_CHANNELS], sr: [0; ATIC111_CHANNELS], words: 0, loads: [Load::new(2.0, 0.002); ATIC111_CHANNELS], frames: 0 }
    }

    /// Diagnosis pairs (executed bit 12+k, no-fault bit 8+k). UNVERIFIED k: 0 short to battery,
    /// 1 short to ground, 2 open load, 3 over-temperature.
    fn status(&self, ch: usize, pins: Atic111Pins) -> u16 {
        let enabled = pins.en && !pins.dis;
        let mut ok = 0xF;
        let mut exec = 0;
        let k = match self.loads[ch].fault {
            LoadFault::None => None,
            LoadFault::ShortToBattery => Some(0),
            LoadFault::ShortToGround => Some(1),
            LoadFault::OpenLoad => Some(2),
        };
        if let Some(k) = k {
            ok &= !(1 << k);
            exec |= 1 << k;
        }
        exec << 12 | ok << 8 | (enabled as u16) << 7 | 0x40 | 0x20 | 0x10 | 0x04 | 0x01
    }

    /// Chip select edge: every stage latches its shift register as new command on release
    /// (when each stage received a complete word).
    pub fn select(&mut self, active: bool) {
        if !active && self.words >= ATIC111_CHANNELS {
            self.frames += 1;
            self.cmd = self.sr;
        }
        self.words = 0;
    }

    /// One 16-bit word in wire order (first bit = bit 15). The stages shift LSB first (the SSC
    /// job runs with CON.HB = 0; the MC reads the status bits in register numbering, 0x800ABF38)
    /// and are daisy-chained without inversion: the TB header 0xAAAA comes back as the last word.
    pub fn exchange(&mut self, mosi: u16, pins: Atic111Pins) -> u16 {
        if self.words == 0 {
            // Status is parallel-loaded into the shift registers at the first clock.
            for k in 0..ATIC111_CHANNELS {
                self.sr[k] = self.status(k, pins);
            }
        }
        self.words += 1;
        let out = self.sr[ATIC111_CHANNELS - 1];
        self.sr.copy_within(0..ATIC111_CHANNELS - 1, 1);
        self.sr[0] = mosi.reverse_bits();
        out.reverse_bits()
    }

    pub fn enabled(&self, pins: Atic111Pins) -> bool {
        pins.en && !pins.dis
    }

    /// Mode bits 3:2 of the channel command (0x801F5530 rewrites only these bits).
    pub fn mode(&self, ch: usize) -> u8 {
        self.cmd.get(ch).map_or(0, |&w| ((w >> 2) & 3) as u8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn par(w: u16) -> u16 {
        let w = w & !1;
        w | (w.count_ones() & 1 == 0) as u16
    }

    /// ATIC121 jobs exactly as the firmware sends and parses them.
    #[test]
    fn atic121_sopc_job_as_parsed_by_the_firmware() {
        let mut c = Atic121::new();
        let pins = Atic121Pins { ndis: false, dis: false };
        // Write job 0x80099750 (reg 0x18), then the SOPC status job 0x800996BC / parser 0x80099FBE
        // in both SOPC phases.
        for w in [par(0xC200), par(0xC002), par(0xB800)] {
            c.exchange(w, pins);
        }
        assert_eq!(c.regs[0x18], 1);
        for (pins, ndis, dis) in [(Atic121Pins { ndis: false, dis: false }, 0, 0), (Atic121Pins { ndis: true, dis: true }, 1, 1)] {
            let tx = [0xB816, 0xB816, par(0xB818), par(0xB800), par(0xB800), par(0xB800)];
            let rx: Vec<u16> = tx.iter().map(|&w| c.exchange(w, pins)).collect();
            let st = (rx[2] & 0x1001) | (rx[3] & 0x1030);
            assert_eq!(st, ndis << 4 | dis << 5, "bit 0 = no fault, 4 = NDIS, 5 = DIS");
            // SC8H83 job: read 0x0B, write 0x18 twice, read 0x0B, 0, 0 -> reply 1.
            let tx = [0xB816, par(0xC004), par(0xC010), 0xB816, par(0xB800), par(0xB800)];
            let rx: Vec<u16> = tx.iter().map(|&w| c.exchange(w, pins)).collect();
            assert_eq!(rx[1], (1 - ndis) << 7 | dis << 9, "bit 7 = NDIS disable, 9 = DIS disable");
        }
        // Identification job 0x800F7478 (init state 1) as parsed by 0x80099BC4: chip id = low byte of the
        // reply to "read 0x01" (RX word 3), revision = low nibble of the reply to "read 0x02" (RX word 4).
        let tx = [0xB816, par(0xC004), par(0xB802), par(0xB804), 0xB816, par(0xB800)];
        let rx: Vec<u16> = tx.iter().map(|&w| c.exchange(w, pins)).collect();
        assert_eq!((rx[3] & 0xFF, rx[4] & 0xF), (ATIC121_CHIP_ID, ATIC121_REVISION));
        // A corrupted word is answered with the SPI error flag.
        c.exchange(0xB800, pins);
        assert_eq!(c.exchange(par(0xB800), pins) & 0x1000, 0x1000);
        assert_eq!(c.parity_errors, 1);
    }

    #[test]
    fn atic111_chain_echo_status_and_command_latch() {
        let mut c = Atic111::new();
        let pins = Atic111Pins { en: true, dis: false };
        // TB words of the firmware's job (0x801F561C, buffer 0xD0010A62): header + ch2, ch1, ch0.
        // The SSC job is LSB first: TB value v appears on the wire as v.reverse_bits().
        let x = |c: &mut Atic111, w: u16, p| c.exchange(w.reverse_bits(), p).reverse_bits();
        c.select(true);
        let rx: Vec<u16> = [0xAAAA, 0x0004, 0x0008, 0x000C].iter().map(|&w| x(&mut c, w, pins)).collect();
        c.select(false);
        assert_eq!(rx[3], 0xAAAA, "header through the chain (0x800ABEFC)");
        for w in &rx[..3] {
            assert!(w & 0x1100 != 0 && w & 0x2200 != 0 && w & 0x8800 != 0, "valid word {w:04x}");
            assert_eq!(w & 0x80, 0x80, "enabled");
        }
        assert_eq!(c.cmd, [0x000C, 0x0008, 0x0004], "stage k latches TB word N-k");
        assert_eq!((c.mode(0), c.mode(2)), (3, 1));
        c.select(true);
        assert_eq!(x(&mut c, 0xAAAA, Atic111Pins { en: true, dis: true }) & 0x80, 0, "MTC_DIS");
        c.select(false);
        assert_eq!(c.cmd, [0x000C, 0x0008, 0x0004], "short frame: no latch");
        c.loads[0].fault = LoadFault::OpenLoad;
        c.select(true);
        let rx: Vec<u16> = [0xAAAA, 0, 0, 0].iter().map(|&w| x(&mut c, w, pins)).collect();
        assert_eq!((rx[2] >> 12 & 4, rx[2] >> 8 & 4), (4, 0), "ch0 open load executed and failed");
    }

    #[test]
    fn atic140_bridged_vcv_coil_reads_open_load_on_both_stages_when_intact() {
        let mut c = Atic140::new();
        c.bridged = ATIC140_BRIDGED[1];
        let code = |c: &Atic140, n: usize| {
            let (a, b, cc) = c.vectors();
            (a >> n & 1, b >> n & 1, cc >> n & 1)
        };
        // Intact N276 between HS 25 and LS 24: both off-state checks see a floating output = open-load code (1,1,0),
        // which the VCV diagnosis (FR p.12179) takes as the good case; ordinary stages report (0,0,1).
        assert_eq!((code(&c, 25), code(&c, 24), code(&c, 3)), ((1, 1, 0), (1, 1, 0), (0, 0, 1)));
        // Broken coil: each output sits at its own bias = "no fault" on both = the open-circuit symptom.
        c.loads[24].fault = LoadFault::OpenLoad;
        assert_eq!((code(&c, 25), code(&c, 24)), ((0, 0, 1), (0, 0, 1)));
        // A short is still a short.
        c.loads[24].fault = LoadFault::ShortToGround;
        assert_eq!(code(&c, 24), (0, 1, 0));
    }

    #[test]
    fn atic140_config_readback_and_disable_inputs_as_parsed_by_the_firmware() {
        let mut c = Atic140::new();
        // ATIC140 #1 of SC800S50 (MSC1 DC trace).
        let pins = Atic140Pins { ndis: false, ls_dis: false, en78: true };
        assert_eq!(c.command(0x0005_3502, pins).len(), 1);
        assert_eq!(c.command(0x003F_E004, pins).len(), 1);
        assert_eq!(c.command(0x030F_3FA1, pins).len(), 2);
        let d = c.command(0x0200_000B, pins);
        assert_eq!(d.len(), 24);
        // Parser 0x800AA89C: fields expected from the same configuration the words were built of.
        let p18 = ((d[3] as u32 >> 4) & 1) | (d[3] as u32 & 4) | ((d[3] as u32 >> 3) & 1) << 1 | ((d[3] as u32 >> 5) & 7) << 3;
        assert_eq!(p18, 0x0D);
        let m = (d[12] as u32 & 7) | ((d[12] as u32 & 0xF0) >> 1) | ((d[13] as u32 & 3) << 7);
        assert_eq!(m, 0);
        assert_eq!((d[11] >> 2) & 0xF, 0, "P24");
        assert_eq!((d[11] >> 6, d[13] >> 2 & 3, d[13] >> 4 & 3, d[13] >> 6), (3, 3, 3, 1));
        assert_eq!(d[14] & 0x20, 0);
        assert_eq!(d[23], 0b100, "NDIS low, LS_DIS low, EN7_8 high");
        // Hardware self check (0x800AB306) then the SOPC disable phase.
        c.command(0x030F_3F51, pins);
        let d = c.command(0x0200_0008, Atic140Pins { ndis: true, ls_dis: true, en78: false });
        assert_eq!(d[14] & 3, 0b10, "HWSC done, passed");
        assert_eq!(d[23], 0b011);
        c.data = 1 << 7 | 1 << 3;
        assert!(c.output_on(3, Atic140Pins { ndis: true, ls_dis: false, en78: false }));
        assert!(!c.output_on(7, Atic140Pins { ndis: true, ls_dis: false, en78: false }), "starter stage gated by EN7_8");
        assert!(!c.output_on(3, Atic140Pins { ndis: false, ls_dis: false, en78: true }));
    }
}
