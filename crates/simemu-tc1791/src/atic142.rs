//! Continental ATIC142 lambda-sensor interface ASIC of the SIMOS18.1 board (TI `A2C49811B1 / 9G15AA`, TSSOP-24
//! top-side right of ATIC121 B3, PHOTO `06K907425B` H13; FR EGCP 22.5, p.5964: "ATIC142 SPI control word ..."):
//! front end of the wide-band (linear, upstream) sensor with its own pump-current controller
//! and of the binary (downstream) sensor on its flexIO pin. SSC1 / SLSO4 (P2.4), 16 bit MSB first, 250 kHz.
//! docs/SIMOS18_BOARD_HW.md §6.
//!
//! # Protocol (VERIFIED-S, unmodified SC800S50; REA evidence ids in the doc)
//! Full duplex with one word latency: the reply to TX word k is RX word k+1 (like ATIC139). A reply carries a
//! response ID in bits 15:12 and 12 data bits, or 16 data bits for the high-resolution reads.
//! * Init (job 4, 6 words, every 10 ms until accepted, `0xC0002388` = flash `0x8012A6E4` state 0):
//!   `C000 A000 B1E8 0666 4000 4000`; accepted when the reply to `C000` has ID C and bits `0x0D0F` clear, the
//!   reply to `B1E8` is `B1E8`, and the reply to `4000` has ID 4 and bits `0x1E8` clear.
//! * Measurement frame (jobs 5 + 6 = 26 words, built by `0x80113470..` and `0x801F4C00`, mode 0 with config
//!   `0x800445D0`: length = cfg+0xC + 24, Ex00 high-resolution reads because cfg+0xD = 1):
//!   `6000 7000 4000 Cxxx Bxxx DD4x DD4x E100 1400 1600 1C00 1700 E100 1800 EA00 E500 1B00 1900 E300 2xxx 3xxx
//!   80xx 90xx Axxx F000 E100`. `1r00` reads register r (reply ID r, 12 data bits); `Er00` reads register r with
//!   high resolution (16-bit reply, no ID); `6000`/`7000` read the diagnosis vectors; `4000` reads the switch state;
//!   `C`, `B`, `2`, `3`, `8`, `9`, `A` write control words (reply ID = command nibble, checked by the handler
//!   `0x8012A786` for slots 0..4 and 18..22; slots 8..16 must answer with ID = command bits 11:8).
//! * Configuration download (job 7, 9 words): `D54c` + 8 data words for chunk c = 0..3 (32 words =
//!   C_STATE_LAMB_IC_CONF_VALUE); the reply to `D54c` must have ID D. Read-back `DD40 | (i & 7) << 3 | i >> 3`
//!   in the measurement frame; the reply must equal the stored 16-bit word (mismatch → LV_LAMB_IC_CONF_ERR).
//!
//! Register map (INFR adapters `0x800A5878` = ACTION_INFR_GetWrafMeasLin, `0x800A57B8` = GetWrafMeasFlexIO,
//! FR p.5930; scales VERIFIED-S from the adapters):
//!
//! | read | quantity (FR 22.2) | encoding |
//! |---|---|---|
//! | `E100` | IPLSL_RAW pump current | i16 & 0xFFFE, 366.39 nA/bit |
//! | `E300` | IPLSL_MMV_SEG segment-averaged pump current | as E100 |
//! | `1400` | VSLSL Nernst cell vs VG | 12-bit two's complement, 8.144 × 152.588 µV = 1.2427 mV/bit |
//! | `E500` | V_R_IT_LSL AC amplitude across the Nernst cell (Ri) | 14-bit two's complement in 15:2, 0.3107 mV/bit |
//! | `1600` | VGLSL virtual ground vs battery ground | 12-bit unsigned, 1.2427 mV/bit |
//! | `1700` | VPLSL pump cell vs VG | 12-bit signed |
//! | `1800` | VRCLSL label-resistor pin vs VG | 12-bit signed |
//! | `1900` | V_LS_SENS binary sensor cell | 12-bit signed |
//! | `EA00` | V_R_IT_LS AC amplitude across the binary cell | 14-bit in 15:2 |
//! | `1B00` | V_LAMB_IC_PIN_MPL flexIO pin | 12-bit signed |
//! | `1C00` | V_LS_GND binary sensor ground pin | 12-bit unsigned |
//! | `4000` | STATE_LAMB_IC_PROT_STR_RD switch read-back (FR p.5933 bit table) | 12 bits |
//! | `6000` / `7000` | STATE_LAMB_IC_DIAG_EL / _DIAG_LIM (latched, read-and-clear in the BSW) | 12 bits |
//!
//! Control words (ASW → BSW setters `0x801F4AF4..0x801F4B24`, `0x800A59F8`): `C` = SetWrafProt (switch request,
//! mask 0x90F/0xD0F), `B` = SetWrafActDiag, `A` = IPLSL_MAN (pump current when the controller is off), `3` =
//! IPLSL controller correction (feed-forward resistance), `2` = segment averaging length, `80xx` = Icp and Ri AC
//! range of the linear sensor, `90xx` = the same for the binary sensor.
//!
//! # Front end (UNVERIFIED: no ATIC142 data sheet; electrochemistry of a pumped-reference planar cell)
//! The plant supplies the cell state ([`LinearCell`], [`BinaryCell`]); the chip measures it:
//! * pump controller (on-chip, FR 22.5 "pumping current controller") holds the measuring cavity at the Nernst
//!   set-point [`VS_SET_V`] (450 mV, Bosch LSU 4.9 data sheet Y 258 E00 015e, the reference cell of the class) while
//!   the switches are in the measurement state and the cell conducts; then IPLSL = the cell's limiting current and
//!   VSLSL = the set-point plus the reference-pump drop. Otherwise no pump current flows and VSLSL is the open-cell
//!   Nernst voltage (reference vs exhaust) plus the drop of the reference pump current across Ri;
//! * VPLSL = VSLSL − E(reference vs exhaust) + IPLSL·Rp (cell loop: reference → cavity → pump cell → exhaust);
//! * Ri: the AC current of the coded range flows through the Nernst cell, the chip reports its amplitude; readings
//!   saturate at the converter range (±2.545 V), as a cold (high-impedance) cell does on a real ECU.

/// Board wiring: SSC1, slave select SLSO4 (P2.4, IOCR 0xA0, TC1791 DS p.33).
pub const ATIC142_SSC: usize = 1;
pub const ATIC142_SLSO: u8 = 4;

/// Nernst set-point of the pump controller (V). UNVERIFIED for ATIC142 (configured by the download words, not
/// decoded); 450 mV is the operating point of the LSU class (Bosch LSU 4.9 data sheet).
pub const VS_SET_V: f64 = 0.45;
/// Virtual ground VG of the linear sensor against battery ground (V). UNVERIFIED (VNLSL_ABS = VSLSL + VGLSL must
/// stay within 0..5 V, FR p.5991).
pub const VG_LIN_V: f64 = 2.5;
/// Binary sensor ground pin (V). UNVERIFIED (sensor ground return on the ECU ground).
pub const VG_BIN_V: f64 = 0.0;

/// LSB of the 12-bit voltage readings (V): INFR factor 0x1FD0/1000 on the 152.588 µV ASW resolution.
pub const LSB_V12: f64 = 8.144 * 5.0 / 32768.0;
/// LSB of the 14-bit AC-amplitude readings (V): INFR factor 0x7F4/1000.
pub const LSB_V14: f64 = 2.036 * 5.0 / 32768.0;
/// LSB of the pump-current readings (mA), FR p.5925 IPLSL_RAW resolution.
pub const LSB_IP_MA: f64 = 366.39e-6;

/// State of the wide-band cell at the chip pins (set by the plant glue every exchange step).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearCell {
    /// Limiting pump current that holds the cavity at the Nernst set-point (mA, > 0 lean).
    pub ip_lim_ma: f64,
    /// Nernst voltage reference electrode vs exhaust gas (V, ≈ 0.05 lean .. 0.9 rich).
    pub e_exh_v: f64,
    /// Nernst cell AC internal resistance (Ω).
    pub ri_ohm: f64,
    /// Pump cell resistance (Ω).
    pub rp_ohm: f64,
}

impl Default for LinearCell {
    /// Unplugged / cold sensor: an open circuit.
    fn default() -> Self {
        Self { ip_lim_ma: 0.0, e_exh_v: 0.0, ri_ohm: 1e9, rp_ohm: 1e9 }
    }
}

/// State of the binary (post-catalyst) cell on the flexIO pin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BinaryCell {
    /// Cell EMF (V, ≈ 0.1 lean .. 0.85 rich).
    pub e_v: f64,
    /// Cell internal resistance (Ω).
    pub ri_ohm: f64,
}

impl Default for BinaryCell {
    fn default() -> Self {
        Self { e_v: 0.0, ri_ohm: 1e9 }
    }
}

/// Ri AC current amplitude (A) per range code: the table the ASW divides by (`0x801A1752..0x801A17DE`, FR 22.6
/// p.5971; VERIFIED-S). The code reaches the chip unchanged (byte packer `0x801A11F6..0x801A120C`), so this is the
/// only mapping under which the firmware's R_IT_LSL = |V_R_IT_LSL| / ACRI is right. (FR 22.5 p.5965 describes
/// C_R_IT_RNG_LSL with another set, 30/120/480/1000/3000 µA — not what this firmware computes with.)
const ACRI_A: [f64; 8] = [0.0, 30e-6, 60e-6, 120e-6, 240e-6, 480e-6, 1000e-6, 3000e-6];
/// Ri AC current per range code of the binary sensor (flexIO) (A): the ASW's table `0x801A14C2..0x801A14FC`
/// (VERIFIED-S), equal to FR 22.5 C_R_IT_RNG_LS: 30, 120, 480, 1000, 2000, 5000 µA.
const ACRI_BIN_A: [f64; 8] = [0.0, 30e-6, 120e-6, 480e-6, 1000e-6, 2000e-6, 5000e-6, 5000e-6];
/// Reference pump current per code step (A): C_ICP_LSL_SP_MAN resolution 0.00375 mA (FR p.5965), 4-bit code
/// (VERIFIED-T: the binary sensor's `9026` carries code 6 = 22.5 µA = c_icp_ls_sp).
const ICP_STEP_A: f64 = 3.75e-6;
/// Switch read-back bits (FR p.5933, STATE_LAMB_IC_PROT_STR_RD): protection switches of the linear sensor's VIP,
/// VG and VN pins and of the binary sensor's flexIO and VG pins; start-up switches of VIP / VN.
const SW_VIP: u16 = 1 << 5;
const SW_VG_LIN: u16 = 1 << 6;
const SW_VN: u16 = 1 << 7;
const SW_FLEX: u16 = 1 << 3;
const SW_VG_BIN: u16 = 1 << 8;
const SW_STARTUP_VIP_VN: u16 = (1 << 10) | (1 << 9);
/// Pump current range of the controller (mA): the 16-bit reading range. UNVERIFIED (chip limit not decoded).
const IP_MAX_MA: f64 = 12.0;

#[derive(Clone, Debug, Default)]
pub struct Atic142 {
    /// Reply queued to be shifted out on the next word.
    out: u16,
    /// Configuration registers (C_STATE_LAMB_IC_CONF_VALUE, 32 words).
    pub conf: [u16; 32],
    /// Configuration chunk being received and the number of data words still expected.
    conf_rx: (usize, u8),
    /// Last control word per command nibble (`2`, `3`, `8`, `9`, `A`, `B`, `C`).
    pub ctl: [u16; 16],
    /// Switch state applied by the chip (FR read-back layout, 12 bits).
    pub sw: u16,
    pub lin: LinearCell,
    pub bin: BinaryCell,
    /// Words exchanged per command nibble (diagnostics).
    pub words: [u64; 16],
    pub conf_downloads: u64,
}

/// 12-bit two's complement reading of `v` volts.
fn v12(v: f64) -> u16 {
    if !v.is_finite() {
        return 0;
    }
    ((v / LSB_V12).round().clamp(-2048.0, 2047.0) as i32 & 0xFFF) as u16
}

/// 12-bit unsigned reading.
fn u12(v: f64) -> u16 {
    if !v.is_finite() {
        return 0;
    }
    (v / LSB_V12).round().clamp(0.0, 4095.0) as u16
}

/// 14-bit two's complement reading in bits 15:2.
fn v14(v: f64) -> u16 {
    if !v.is_finite() {
        return 0;
    }
    (((v / LSB_V14).round().clamp(-8192.0, 8191.0) as i32 & 0x3FFF) << 2) as u16
}

/// Pump current reading (i16, bit 0 clear).
fn ip16(ma: f64) -> u16 {
    if !ma.is_finite() {
        return 0;
    }
    ((ma / LSB_IP_MA).round().clamp(-32768.0, 32767.0) as i32 as u16) & 0xFFFE
}

impl Atic142 {
    /// Range code of the Ri AC current in a `80xx` / `90xx` control word: bits 7:5 (packer `0x801A11F6`: byte =
    /// R_IT_RNG << 5 + ICP boost << 4 + ICP code, VERIFIED-S).
    pub fn acri_code(word: u16) -> usize {
        (word as usize >> 5) & 7
    }

    /// Reference pump current (A) of a `80xx` / `90xx` control word (bits 3:0). UNVERIFIED: boost (bit 4) not
    /// modelled.
    pub fn icp_a(word: u16) -> f64 {
        (word & 0xF) as f64 * ICP_STEP_A
    }

    /// Switch pattern the chip has applied (`4000` read-back). The `Bxxx` request (switch word built by the ASW at
    /// `0x801A125E..0x801A135E` in the FR read-back layout, VERIFIED-S) takes effect at the next chip select: the
    /// init frame writes `B1E8` and must still read the switches open (`0x1E8` clear) four words later
    /// (`0x8012A774`). HYP: switching time between one select burst and the next.
    pub fn switches(&self) -> u16 {
        self.sw
    }

    /// Chip select of SLSO4 asserted: pending switch requests take effect.
    pub fn select(&mut self) {
        self.sw = self.ctl[0xB] & 0x0FFF;
    }

    /// Pump-current controller running: VIP, VG and VN connected, VIP/VN start-up switches open, and the controller
    /// enable of the control word set: `C` bit 10 = 0xD0001D36 ∧ ¬0xD0001D39 (packer `0x801A1360..0x801A13A6`; bit 0
    /// = 0xD0001D39, and the setter `0x801F50DC` clears bit 10 whenever bit 0 is set), i.e. "Ip controller on unless
    /// the constant-current mode is requested". VERIFIED-T that the firmware sets it (`C900` → `C500`/`CD00`) exactly
    /// when its sensor-ready conditions of FR 22.11.4 are met; that bit 10 is LV_IPLSL_ENA is HYP.
    pub fn pump_control_on(&self) -> bool {
        let s = self.sw;
        s & (SW_VIP | SW_VG_LIN | SW_VN) == SW_VIP | SW_VG_LIN | SW_VN && s & SW_STARTUP_VIP_VN == 0 && self.ctl[0xC] & 0x400 != 0
    }

    /// Pump current (mA) and Nernst voltage across the cell incl. the reference-pump drop (V), as the chip measures.
    fn linear_ip_vs(&self) -> (f64, f64) {
        let c = &self.lin;
        let drop = Self::icp_a(self.ctl[0x8]) * c.ri_ohm;
        // the controller settles only while the cell conducts (R_N ≪ the reference pump source impedance)
        if self.pump_control_on() && c.ri_ohm < 5_000.0 {
            (c.ip_lim_ma.clamp(-IP_MAX_MA, IP_MAX_MA), VS_SET_V + drop)
        } else {
            (0.0, c.e_exh_v + drop)
        }
    }

    fn reg(&self, r: u16) -> u16 {
        let (c, b, s) = (&self.lin, &self.bin, self.sw);
        let nernst = s & (SW_VN | SW_VG_LIN) == SW_VN | SW_VG_LIN;
        let pump = nernst && s & SW_VIP != 0;
        let flex = s & (SW_FLEX | SW_VG_BIN) == SW_FLEX | SW_VG_BIN;
        let (ip, vs) = self.linear_ip_vs();
        let v_ls = b.e_v + Self::icp_a(self.ctl[0x9]) * b.ri_ohm;
        // a disconnected pin reads 0 V against its reference (UNVERIFIED: no pin-state model of an open input)
        let on = |cond: bool, v: f64| if cond { v } else { 0.0 };
        match r {
            0x1 | 0x3 => ip16(on(pump, ip)),
            0x4 => v12(on(nernst, vs)),
            0x5 => v14(on(nernst, ACRI_A[Self::acri_code(self.ctl[0x8])] * c.ri_ohm)),
            0x6 => u12(VG_LIN_V),
            0x7 | 0x8 => v12(on(pump, vs - c.e_exh_v + ip * 1e-3 * c.rp_ohm)),
            0x9 => v12(on(flex, v_ls)),
            0xA => v14(on(flex, ACRI_BIN_A[Self::acri_code(self.ctl[0x9])] * b.ri_ohm)),
            0xB => v12(VG_BIN_V + on(flex, v_ls)),
            0xC => u12(VG_BIN_V),
            _ => 0,
        }
    }

    /// One 16-bit word on SLSO4: returns the reply to the previous one.
    pub fn exchange(&mut self, mosi: u16) -> u16 {
        let miso = self.out;
        self.out = self.reply(mosi);
        miso
    }

    fn reply(&mut self, w: u16) -> u16 {
        let nib = (w >> 12) as usize;
        self.words[nib] += 1;
        if self.conf_rx.1 > 0 {
            let (chunk, left) = self.conf_rx;
            self.conf[chunk * 8 + 8 - left as usize] = w;
            self.conf_rx.1 -= 1;
            if self.conf_rx.1 == 0 && chunk == 3 {
                self.conf_downloads += 1;
            }
            return w;
        }
        match nib {
            // configuration write header D54c / read-back DD4x
            0xD if w & 0x0FFC == 0x0540 => {
                self.conf_rx = ((w & 3) as usize, 8);
                w
            }
            0xD if w & 0x0FC0 == 0x0D40 => self.conf[(w as usize >> 3 & 7) | (w as usize & 3) << 3],
            0x1 => (w & 0x0F00) << 4 | self.reg(w >> 8 & 0xF) & 0x0FFF,
            0xE => self.reg(w >> 8 & 0xF),
            0x4 => 0x4000 | self.switches(),
            0x6 | 0x7 => w & 0xF000, // no electrical fault, no limitation (UNVERIFIED: no fault model)
            0x2 | 0x3 | 0x8 | 0x9 | 0xA | 0xB | 0xC => {
                self.ctl[nib] = w;
                w
            }
            _ => w,
        }
    }
}

#[cfg(test)]
mod tests {
    //! The MC side is a transcription of the firmware's handlers (addresses in the module docs).
    use super::*;

    fn frame(c: &mut Atic142, tx: &[u16]) -> Vec<u16> {
        tx.iter().map(|&w| c.exchange(w)).collect()
    }

    /// Init acceptance, `0x8012A712..0x8012A778`.
    #[test]
    fn init_handshake_is_accepted() {
        let mut c = Atic142::default();
        let rx = frame(&mut c, &[0xC000, 0xA000, 0xB1E8, 0x0666, 0x4000, 0x4000]);
        assert_eq!((rx[1] & 0xF000, rx[1] & 0x0D0F), (0xC000, 0));
        assert_eq!(rx[3], 0xB1E8);
        assert_eq!((rx[5] & 0xF000, rx[5] & 0x1E8), (0x4000, 0));
    }

    /// Configuration download (job 7, `0x801F4D92`) and read-back check (`0x8012A8C4`).
    #[test]
    fn configuration_downloads_and_reads_back() {
        let mut c = Atic142::default();
        let conf: Vec<u16> = (0..32).map(|i| 0xF000 ^ (i * 0x0123) as u16).collect();
        for chunk in 0..4u16 {
            let mut tx = vec![0xD540 | chunk];
            tx.extend(&conf[chunk as usize * 8..chunk as usize * 8 + 8]);
            let rx = frame(&mut c, &tx);
            assert_eq!(rx[1] >> 12, 0xD, "job 7 RX[1] ID (0x8012AA0A)");
        }
        assert_eq!(c.conf_downloads, 1);
        for i in 0..32u16 {
            let rd = 0xDD40 | (i & 7) << 3 | (i >> 3) & 3;
            let rx = frame(&mut c, &[rd, 0xF000]);
            assert_eq!(rx[1], conf[i as usize], "config word {i}");
        }
    }

    /// Real-time budget: the 26-word frame every 10 ms. Prints the cost per frame.
    #[test]
    fn frame_cost_is_negligible() {
        let mut c = Atic142 {
            lin: LinearCell { ip_lim_ma: 0.3, e_exh_v: 0.05, ri_ohm: 300.0, rp_ohm: 150.0 },
            ..Default::default()
        };
        let tx: [u16; 26] = [
            0x6000, 0x7000, 0x4000, 0xC400, 0xB9E8, 0xDD40, 0xDD48, 0xE100, 0x1400, 0x1600, 0x1C00, 0x1700, 0xE100, 0x1800, 0xEA00,
            0xE500, 0x1B00, 0x1900, 0xE300, 0x2000, 0x3000, 0x8081, 0x9026, 0xA000, 0xF000, 0xE100,
        ];
        let n = 100_000;
        let t0 = std::time::Instant::now();
        let mut acc = 0u64;
        for _ in 0..n {
            c.select();
            acc += tx.iter().map(|&w| c.exchange(w) as u64).sum::<u64>();
        }
        let ns = t0.elapsed().as_nanos() as f64 / n as f64;
        eprintln!("ATIC142: {ns:.0} ns per 26-word frame ({:.4} % of the 10 ms period) [{acc}]", ns / 1e5);
        assert!(ns < 20_000.0, "{ns} ns");
    }

    /// Measurement frame: the reply-ID checks of `0x8012A786` and the INFR scaling of the readings.
    #[test]
    fn measurement_frame_passes_the_id_checks_and_scales_like_the_firmware() {
        let mut c = Atic142 {
            lin: LinearCell { ip_lim_ma: 0.5, e_exh_v: 0.08, ri_ohm: 240.0, rp_ohm: 100.0 },
            bin: BinaryCell { e_v: 0.7, ri_ohm: 500.0 },
            ..Default::default()
        };
        let tx: [u16; 26] = [
            0x6000, 0x7000, 0x4000, 0xC400, 0xB9E8, 0xDD40, 0xDD48, 0xE100, 0x1400, 0x1600, 0x1C00, 0x1700, 0xE100, 0x1800, 0xEA00,
            0xE500, 0x1B00, 0x1900, 0xE300, 0x2000, 0x3000, 0x80C0, 0x9020, 0xA000, 0xF000, 0xE100,
        ];
        frame(&mut c, &tx); // first frame programs the switches and AC ranges
        c.select(); // they take effect at the next chip select
        let mut rx = frame(&mut c, &tx);
        rx.push(c.exchange(0));
        let tab: [usize; 24] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24];
        for i in [0, 1, 2, 3, 4, 18, 19, 20, 21, 22] {
            assert_eq!(tx[tab[i]] & 0xF000, rx[tab[i] + 1] & 0xF000, "slot {i}: ID echo");
        }
        for i in [8, 9, 10, 11, 12, 15, 16] {
            assert_eq!((tx[tab[i]] & 0x0F00) << 4, rx[tab[i] + 1] & 0xF000, "slot {i}: register ID");
        }
        let s12 = |w: u16| ((w << 4) as i16 >> 4) as f64 * LSB_V12;
        let s14 = |w: u16| (w as i16 >> 2) as f64 * LSB_V14;
        let ip = |w: u16| (w & 0xFFFE) as i16 as f64 * LSB_IP_MA;
        assert!(c.pump_control_on(), "sw {:03X} C {:04X}", c.sw, c.ctl[0xC]);
        assert!((ip(rx[8]) - 0.5).abs() < 1e-3 && (ip(rx[19]) - 0.5).abs() < 1e-3, "IPLSL_RAW / MMV_SEG");
        assert!((s12(rx[9]) - VS_SET_V).abs() < 2e-3, "VSLSL");
        assert!((s14(rx[16]) - 240.0 * 1000e-6).abs() < 1e-3, "V_R_IT_LSL = Ri · I_ac (range 6 = 1000 µA)");
        assert!((s12(rx[18]) - 0.7).abs() < 2e-3, "V_LS_SENS");
        assert_eq!(rx[3] & 0x0FFF, 0x9E8, "switch read-back = the applied B pattern (MES)");
    }
}
