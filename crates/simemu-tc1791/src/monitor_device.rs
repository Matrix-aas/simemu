//! Processor-monitoring unit ("MU", Continental ECM3 level-3 monitoring module) of the SIMOS18.1
//! board, a function block of the ATIC139 system ASIC ([`crate::atic139`] has the evidence; the
//! LS/PS blocks of the same chip answer the 0xAAAA frames on the same select line). It is an
//! independent SPI slave on SSC1 / SLSO2 that plays the question/answer game
//! with the main controller (MC = TC1791 firmware), keeps its own anti-bounce and reset counters
//! and drives the power-stage disable lines and the MC reset.
//!
//! Sources: FR = Continental SIMOS18 Funktionsrahmen ch. 18 "ECM3" (pages cited as FR p.N);
//! addresses = unmodified SC800S50 firmware (identical code relocated in SC8O30/SC8O40).
//!
//! # Transport (proven from the firmware)
//! * SSC1, 16 bit, MSB first, 1 MHz (CON 0xC01F, BR 0x2C), SLSO2 (SSOC 0x0400), one DMA job of
//!   7 words per communication cycle (TREL 7: TX ch3 from 0xD0010A54, RX ch1 to 0xD0010AD0).
//! * TX words are built in 0xD000A0FE.. by the BSW (0x80062B6C..0x80062CF8) and
//!   0x80081724 (fixed words); every TX word carries odd parity in bit 0 (0x801F4BC4 /
//!   0x8008718C XOR fold): `[cmd0, cmd1, 0xBFE1, 0xBFE4, 0xBFE7, 0xBFEB, 0xCFFE]`.
//! * Full duplex with one word latency: RX word k is the MU's answer to TX word k-1 (RX copied to
//!   0xD000A0F0 by 0x80062FD0). Bits 15:12 of every RX word = STATE_MU; ReadMuData
//!   (0x8006C736..0x8006CC10) requires words 1..6 to carry the same valid state (else MU_COM).
//!   RX bit 0 is never checked for parity by the MC.
//!
//! | TX command (bits 15:12)           | firmware encoder | MU answer in the next RX word (decoder) |
//! |-----------------------------------|------------------|------------------------------------------|
//! | 3 INIT1: `CTR_RST_MC[11:8] NR_COMP_MC[7:5]` | 0x80062B6C | `CTR_RST_MU[11:8] NR_COMP_MU[7:5] ERR_COD_MU[4:1]` (0x80062CFC) |
//! | 4 INIT2: `data[11:2]` (= 0)         | 0x80062BA0 | `~data[11:2]` (0x80062D54; MC expects 0x3FF, 0x8006C9D0) |
//! | 5 CONF1: `PFM_TMR[11:4] IGK_ACQ[3] IGK_SRC[2]` (0xA3,1,0) | 0x80062BC8 | ~data (0x80062D88; expects 0x171, 0x8006C864) |
//! | 6 CONF2: `IST32[11] IST_TMR[10:9] RST_MAX[8:6] x[5:2]` (0,2,7,0) | 0x80062BFC | ~data (0x80062DBC; expects 0x28F, 0x8006C9A6) |
//! | 7 NORM1: `WRG_INTD[11] IST_RESP[10:1]` | 0x80062C44 | `PAR_NORM1[11] DI0[10] DI1[9] ABC_IST[8:6] QUESTION[5:1]` (0x80062DF0) |
//! | 8 NORM2: `1[11] IGK[10] ~IGK[9] PFM[8:5] ~PFM[4:1]` | 0x80062C74 / 0x8006E420 | `PAR_NORM2[11] IGK[10] ~IGK[9] ABC_PFM_STATE[8:5] PFM_SYN[4:1]` (0x80062E50) |
//! | 9 DISABLE: `0xF3E / FOC_RST_MC[7] ~[6]` | 0x80062CBC | `CTR_RST_MU[11:8] FOC_RST_MU[7] ~[6] IGK[5] ERR_COD_MU[4:1]` (0x80062EB0) |
//! | 0xBFE0 read                       | 0x80081724 | ADC byte `[7:0]`; VP_ADC_MU = byte << 7 (0x80062F10, 0x8006E2FE) |
//! | 0xBFE4 read                       |            | `ABC_PFM0[3:0] ABC_PFM1[7:4] ABC_PFM2[11:8]` (0x80062F70) |
//! | 0xBFE6 read                       |            | `ABC_PFM3[3:0]` (0x80062F70: state must equal the 0xBFE4 word's) |
//! | 0xBFEA read                       |            | `CTR_MU_INJ_OFF_TMR[9:5]` (0x80063008, used by IOT 0x8006CB22) |
//! | 0xCFFE                            |            | (answer lands in RX word 0 of the next frame, unused by the MC) |
//!
//! MC-side checks reproduced by the tests: CKS values are the 10-bit complement of the data
//! received (INIT2/CONF1/CONF2, FR p.5718 "value and complement"); PAR_NORM1/2 = odd-parity bit
//! of the stored CONF2/CONF1 data (MC constants NLC_CKS_NORM_1_PAR_MU = 1, NORM_2 = 0, FR
//! p.5677; checks 0x8006C7D8 / 0x8006C92A); IGK and its complement must differ (0x8006C960,
//! MU_IGK_CPL); FS-IST answers must be `ID_RESP_MON3[question]` (0x8006D4F4: table at
//! 0x80040B20, identical in SC8O30/SC8O40/SC8H83), every question index must appear within 100
//! cycles (0x8006D2FC, MU_RND_FS_IST); after a deliberately wrong FS-IST answer the MU
//! anti-bounce counter must rise within 4 cycles and then fall (0x8006D368: MU_WRT_FS_IST /
//! MU_ABC_FS_IST); PFM: toggled answer bits must come back in the next PFM_SYN (0x8006D874,
//! MU_TOG_PFM_k), and when the MC stops toggling task k its ABC must rise within
//! `TCC_PFM_WRT_MAX = [3, 6, 3, 390]` cycles (0x8006D8EC, table 0x80040BEC) — consistent with
//! the CONF1 timer byte 0xA3 decoded MSB-first as PFM0..3 = 26 / 52 / 26 / 2990 ms (FR p.5676).
//! NDIS1 high is required in NORM, otherwise the MC keeps the power stages disabled
//! (0x8006D05C, LV_ERR_TMP_MU_MC).
//!
//! # Board dependency: switch-off path check (SOPC)
//! NDIS0/NDIS1 feed the power-stage drivers (ATIC140/121/111, FR p.5679), modelled in
//! [`crate::atic`] / [`crate::power_stages`]. The MC's SOPC (0x8006E0FC..) checks in MU_CHK that
//! the MU-driven paths are off while the MU is not in NORM, waits in MC_WAIT for
//! LV_MU_DI_1_OV = 1 (0x8006E198 -> 0x80080CF8), then requires the drivers' diagnostics to show
//! the MC paths off and the MU paths on (final check 0x80080F2C, CAL 0x80810228 = 0x7FFF).

//! # Not determinable from the MC firmware (marked UNVERIFIED below)
//! The MU is an ASIC whose internals are not in the dump. The following are modelled choices
//! that satisfy every MC-side check but are not proven: FS-IST question order (FR p.5735 only
//! says "pseudo random, each question once per cycle"), the anti-bounce maxima that trigger the
//! MU fault reaction, the IOT tick, the MU power-on configuration before CONF, the reaction to
//! parity errors. The 0xAAAA frames sharing SLSO2 belong to the LS/PS blocks ([`crate::atic139`]).

use crate::ssc::SpiSlave;

/// Board wiring: the MU is on SSC1, slave select SLSO2 (SSOC 0x0400 of the 7-word job).
pub const MU_SSC: usize = 1;
pub const MU_SLSO: u8 = 2;

/// NC_NR_MU_COMP / NC_NR_MC_COMP (FR p.5694).
pub const NR_COMP_MU: u16 = 2;
pub const NR_COMP_MC_EXPECTED: u16 = 2;

/// FS-IST answer ROM of the MU = `ID_RESP_MON3[32]` (flash 0x80040B20 in SC8S50/SC8O30/SC8O40,
/// 0x80040AC4 in SC8H83, identical contents). The MC sends `ID_RESP_MON3[i]` when its level-2'
/// result equals `ID_RESP_MON2[i]` (0x8006D530..0x8006D55A), so a working MU must expect exactly
/// these values.
pub const FSIST_ANSWERS: [u16; 32] = [
    0x0FF, 0x0C3, 0x099, 0x365, 0x03C, 0x00F, 0x330, 0x066, 0x3A9, 0x39A, 0x3FC, 0x3C0, 0x303, 0x0A5, 0x05A, 0x055,
    0x0F0, 0x0AA, 0x069, 0x000, 0x30C, 0x3A6, 0x3CF, 0x3F3, 0x36A, 0x33F, 0x0CC, 0x395, 0x033, 0x096, 0x359, 0x356,
];

/// PFM timer options (FR p.5676), in microseconds, indexed `[task][2-bit option]`.
pub const PFM_PERIOD_US: [[u64; 4]; 4] = [
    [6_500, 13_000, 26_000, 52_000],
    [13_000, 26_000, 52_000, 130_000],
    [26_000, 52_000, 130_000, 260_000],
    [260_000, 520_000, 1_495_000, 2_990_000],
];

/// ABC_PFM_MU power-on values (FR p.5697: task 3 starts at 10).
pub const PFM_ABC_INIT: [u8; 4] = [0, 0, 0, 10];

/// UNVERIFIED: anti-bounce value at which the MU performs its fault reaction (the counters are
/// 3 bit for FS-IST and 4 bit for PFM in the frame; the MC's own tests never drive them further
/// than +1, and PFM3 starts at 10).
pub const FSIST_ABC_MAX: u8 = 7;
pub const PFM_ABC_MAX: u8 = 15;

/// UNVERIFIED: power-on configuration before CONF1/CONF2 = the FR default options (PFM byte
/// 0xA7, IGK hardware acquisition from EN1; 16 questions, 26 ms, 7 resets).
pub const CONF1_DEFAULT: u16 = (0xA7 << 2) | (1 << 1);
pub const CONF2_DEFAULT: u16 = (2 << 7) | (7 << 4);

/// UNVERIFIED: injection-off timer. FR p.5752: NDIS0 drops ~850 ms after ignition off; the MC
/// treats counter values >= 26 as "about to switch off" (0x8006CB8A).
pub const IOT_TICK_NS: u64 = 850_000_000 / 27;
pub const IOT_NDIS0_OFF: u8 = 27;

/// STATE_MU codes (FR p.5717).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MuState {
    Unknown = 0x0,
    Init = 0x3,
    Conf = 0x6,
    Eolt = 0x9,
    Norm = 0xA,
    Disable = 0xF,
}

/// ERR_COD_MU (FR p.5693).
pub mod err {
    pub const NO_ERR: u8 = 0x0;
    pub const MU_COMP: u8 = 0x4;
    pub const MU_FS_IST: u8 = 0x5;
    pub const MU_PFM_0: u8 = 0x8;
    pub const MU_DI_RST: u8 = 0xE;
}

#[derive(Debug, Default, Clone, Copy)]
pub struct MuStats {
    pub frames: u64,
    pub ist_correct: u64,
    pub ist_wrong: u64,
    /// Wrong answers sent with NORM1 bit 11 set (LV_MC_IST_WRG_RESP_INTD; the wrapper 0x8006E408
    /// feeds it from LV_MC_DR_OFF = redundant switch-off request, 0x8006D108).
    pub ist_wrong_intended: u64,
    /// (question, answer) of the most recent wrong FS-IST answer.
    pub last_wrong: (u8, u16),
    pub pfm_toggles: u64,
    pub pfm_timeouts: u64,
    pub parity_errors: u64,
    pub protocol_errors: u64,
    pub faults: u64,
}

#[inline]
fn odd_parity_ok(w: u16) -> bool {
    w.count_ones() & 1 == 1
}

/// Odd-parity bit of a data field (1 when the field has an even number of ones).
#[inline]
fn odd_parity_bit(d: u16) -> u16 {
    (d.count_ones() & 1 == 0) as u16
}

#[derive(Debug, Clone)]
pub struct MonitorDevice {
    state: MuState,
    permanent_disable: bool,
    conf1: u16,
    conf2: u16,
    /// EN1 / EN2 inputs (ignition key, KL15).
    pub en1: bool,
    pub en2: bool,
    /// Voltage of the redundant analog input converted by the MU (0..5 V, 8 bit).
    pub adc_volts: f32,
    igk_mc: bool,
    ctr_rst_mc: u8,
    ctr_rst_mu: u8,
    err_cod_mu: u8,
    foc_rst_mu: bool,
    reset_request: bool,
    // FS-IST
    order: [u8; 32],
    pos: usize,
    question: u8,
    abc_ist: u8,
    lfsr: u16,
    // PFM
    pfm_sync: u8,
    pfm_abc: [u8; 4],
    pfm_deadline: [u64; 4],
    // injection-off timer
    iot_ctr: u8,
    iot_next: u64,
    // SPI shift register
    out: u16,
    frame_pos: u32,
    moc_frame: bool,
    pub stats: MuStats,
}

impl Default for MonitorDevice {
    fn default() -> Self {
        Self::new()
    }
}

impl MonitorDevice {
    /// Power-on (board supply up): INIT state, counters cleared.
    pub fn new() -> Self {
        let mut d = Self {
            state: MuState::Init,
            permanent_disable: false,
            conf1: CONF1_DEFAULT,
            conf2: CONF2_DEFAULT,
            en1: true,
            en2: true,
            adc_volts: 0.0,
            igk_mc: false,
            ctr_rst_mc: 0,
            ctr_rst_mu: 0,
            err_cod_mu: err::NO_ERR,
            foc_rst_mu: false,
            reset_request: false,
            order: [0; 32],
            pos: 0,
            question: 0,
            abc_ist: 0,
            lfsr: 0xACE1,
            pfm_sync: 0,
            pfm_abc: PFM_ABC_INIT,
            pfm_deadline: [u64::MAX; 4],
            iot_ctr: 0,
            iot_next: 0,
            out: 0,
            frame_pos: 0,
            moc_frame: false,
            stats: MuStats::default(),
        };
        d.shuffle();
        d
    }

    pub fn state(&self) -> MuState {
        self.state
    }
    pub fn ctr_rst_mu(&self) -> u8 {
        self.ctr_rst_mu
    }
    pub fn err_cod_mu(&self) -> u8 {
        self.err_cod_mu
    }
    pub fn question(&self) -> u8 {
        self.question
    }
    pub fn abc_ist(&self) -> u8 {
        self.abc_ist
    }
    pub fn pfm_abc(&self) -> [u8; 4] {
        self.pfm_abc
    }
    pub fn pfm_sync(&self) -> u8 {
        self.pfm_sync
    }

    /// NDIS0 (injection) / NDIS1 (throttle, injection, ...) disable lines: high = power stages
    /// enabled. Only in NORM without fault (FR p.5689 fault reaction; MC requirement 0x8006D05C).
    pub fn ndis0(&self) -> bool {
        self.state == MuState::Norm && self.iot_ctr < IOT_NDIS0_OFF
    }
    pub fn ndis1(&self) -> bool {
        self.state == MuState::Norm
    }

    /// The MU drives the MC reset (fault reaction or MC request via DISABLE.FOC_RST_MC). The
    /// board must reset the SoC and then call [`Self::on_mc_reset`].
    pub fn take_reset_request(&mut self) -> bool {
        std::mem::take(&mut self.reset_request)
    }

    /// The MC went through reset (the reset line is fed back to the MU, FR p.5689): the protocol
    /// restarts in INIT; reset/error counters survive.
    pub fn on_mc_reset(&mut self) {
        self.reset_request = false;
        self.foc_rst_mu = false;
        if !self.permanent_disable {
            self.state = MuState::Init;
        }
        self.abc_ist = 0;
        self.pfm_sync = 0;
        self.pfm_abc = PFM_ABC_INIT;
        self.pfm_deadline = [u64::MAX; 4];
        self.frame_pos = 0;
        self.moc_frame = false;
    }

    fn n_questions(&self) -> usize {
        if self.conf2 & (1 << 9) != 0 { 32 } else { 16 }
    }

    fn pfm_period_ns(&self, k: usize) -> u64 {
        let byte = (self.conf1 >> 2) & 0xFF;
        let opt = (byte >> (6 - 2 * k)) & 3;
        PFM_PERIOD_US[k][opt as usize] * 1000
    }

    fn rst_max(&self) -> u8 {
        ((self.conf2 >> 4) & 7) as u8
    }

    /// Ignition key as acquired by the MU (CONF1: IGK_ACQ = 1 hardware pin EN1/EN2 per IGK_SRC,
    /// IGK_ACQ = 0 the value the MC sends in NORM2).
    pub fn igk(&self) -> bool {
        if self.conf1 & 2 != 0 {
            if self.conf1 & 1 != 0 { self.en2 } else { self.en1 }
        } else {
            self.igk_mc
        }
    }

    /// UNVERIFIED question order: Fisher-Yates shuffle driven by a 16-bit Galois LFSR. Only the
    /// property checked by the MC is guaranteed: each index exactly once per cycle.
    fn shuffle(&mut self) {
        let n = self.n_questions();
        for (i, q) in self.order.iter_mut().enumerate() {
            *q = i as u8;
        }
        for i in (1..n).rev() {
            for _ in 0..16 {
                let lsb = self.lfsr & 1;
                self.lfsr >>= 1;
                if lsb != 0 {
                    self.lfsr ^= 0xB400;
                }
            }
            let j = self.lfsr as usize % (i + 1);
            self.order.swap(i, j);
        }
        self.pos = 0;
        self.question = self.order[0];
    }

    fn next_question(&mut self) {
        self.pos += 1;
        if self.pos >= self.n_questions() {
            self.shuffle();
        } else {
            self.question = self.order[self.pos];
        }
    }

    fn enter_norm(&mut self, now_ns: u64) {
        self.state = MuState::Norm;
        self.pfm_sync = 0;
        self.pfm_abc = PFM_ABC_INIT;
        for k in 0..4 {
            self.pfm_deadline[k] = now_ns + self.pfm_period_ns(k);
        }
        self.abc_ist = 0;
        self.shuffle();
        self.iot_ctr = 0;
        self.iot_next = now_ns + IOT_TICK_NS;
    }

    /// MU fault reaction (FR p.5689): disable the power stages, count, reset the MC or, after
    /// RST_MAX resets, stay in DISABLE permanently.
    fn fault(&mut self, code: u8) {
        self.stats.faults += 1;
        self.err_cod_mu = code;
        self.ctr_rst_mu = (self.ctr_rst_mu + 1).min(15);
        self.state = MuState::Disable;
        self.pfm_deadline = [u64::MAX; 4];
        if self.ctr_rst_mu > self.rst_max() {
            self.permanent_disable = true;
        } else {
            self.reset_request = true;
            self.foc_rst_mu = true;
        }
    }

    /// Advance the MU's own timers (independent oscillator) to `now_ns`.
    pub fn advance(&mut self, now_ns: u64) {
        if self.state != MuState::Norm {
            return;
        }
        for k in 0..4 {
            let period = self.pfm_period_ns(k);
            while now_ns >= self.pfm_deadline[k] {
                self.pfm_deadline[k] += period;
                self.pfm_abc[k] += 1;
                self.stats.pfm_timeouts += 1;
                if self.pfm_abc[k] >= PFM_ABC_MAX {
                    self.fault(err::MU_PFM_0 + k as u8);
                    return;
                }
            }
        }
        if self.igk() {
            self.iot_ctr = 0;
            self.iot_next = now_ns + IOT_TICK_NS;
        } else {
            while now_ns >= self.iot_next {
                self.iot_ctr = (self.iot_ctr + 1).min(31);
                self.iot_next += IOT_TICK_NS;
            }
        }
    }

    fn st(&self) -> u16 {
        (self.state as u16) << 12
    }

    /// Slave select of the MU asserted (`true`) or released.
    pub fn select(&mut self, active: bool) {
        if active {
            self.frame_pos = 0;
            self.moc_frame = false;
        }
    }

    /// One 16-bit SPI word: returns the word in the MU's output register (prepared from the
    /// previous word) and processes `mosi`.
    pub fn exchange(&mut self, mosi: u16, now_ns: u64) -> u16 {
        let miso = self.out;
        if self.frame_pos == 0 {
            self.moc_frame = matches!(mosi >> 12, 3..=9);
            if self.moc_frame {
                self.stats.frames += 1;
            }
        }
        self.frame_pos += 1;
        self.out = if self.moc_frame { self.process(mosi, now_ns) } else { 0 };
        miso
    }

    fn process(&mut self, w: u16, now_ns: u64) -> u16 {
        self.advance(now_ns);
        if !odd_parity_ok(w) {
            self.stats.parity_errors += 1; // UNVERIFIED: corrupted words are ignored
            return self.st();
        }
        let d10 = (w >> 2) & 0x3FF;
        let cks = (!d10 & 0x3FF) << 2;
        let accepting_conf = matches!(self.state, MuState::Init | MuState::Conf);
        match w >> 12 {
            0x3 => {
                self.ctr_rst_mc = ((w >> 8) & 0xF) as u8;
                if self.state == MuState::Init && (w >> 5) & 7 != NR_COMP_MC_EXPECTED {
                    self.fault(err::MU_COMP);
                }
                self.st() | (self.ctr_rst_mu as u16 & 0xF) << 8 | NR_COMP_MU << 5 | (self.err_cod_mu as u16 & 0xF) << 1
            }
            0x4 => self.st() | cks,
            0x5 | 0x6 => {
                if accepting_conf {
                    if w >> 12 == 5 {
                        self.conf1 = d10;
                    } else {
                        self.conf2 = d10;
                    }
                    self.state = MuState::Conf;
                }
                self.st() | cks
            }
            0x7 => {
                if accepting_conf {
                    self.enter_norm(now_ns);
                } else if self.state == MuState::Norm {
                    self.check_fsist_answer((w >> 1) & 0x3FF, w & 0x800 != 0);
                }
                self.st()
                    | odd_parity_bit(self.conf2) << 11
                    | (self.ndis0() as u16) << 10
                    | (self.ndis1() as u16) << 9
                    | (self.abc_ist as u16 & 7) << 6
                    | (self.question as u16 & 0x1F) << 1
            }
            0x8 => {
                let (igk, igk_cpl) = ((w >> 10) & 1, (w >> 9) & 1);
                let (pfm, pfm_cpl) = (((w >> 5) & 0xF) as u8, ((w >> 1) & 0xF) as u8);
                if igk != igk_cpl && pfm ^ pfm_cpl == 0xF {
                    self.igk_mc = igk == 1;
                    if self.state == MuState::Norm {
                        self.check_pfm_answer(pfm, now_ns);
                    }
                } else {
                    self.stats.protocol_errors += 1; // complement check failed: data not used
                }
                let state_bits = (0..4).fold(0u16, |a, k| a | ((self.pfm_abc[k] > 0) as u16) << k);
                let igk = self.igk() as u16;
                self.st() | odd_parity_bit(self.conf1) << 11 | igk << 10 | (igk ^ 1) << 9 | state_bits << 5 | (self.pfm_sync as u16) << 1
            }
            0x9 => {
                let (foc, foc_cpl) = ((w >> 7) & 1, (w >> 6) & 1);
                if foc != foc_cpl && foc == 1 && !self.permanent_disable {
                    self.reset_request = true; // MC asks the MU to reset it (LV_FOC_RST_MC)
                }
                self.state = MuState::Disable;
                self.pfm_deadline = [u64::MAX; 4];
                let f = self.foc_rst_mu as u16;
                self.st() | (self.ctr_rst_mu as u16 & 0xF) << 8 | f << 7 | (f ^ 1) << 6 | (self.igk() as u16) << 5 | (self.err_cod_mu as u16 & 0xF) << 1
            }
            0xB => {
                let data = match w & 0xFFFE {
                    0xBFE0 => (self.adc_volts.clamp(0.0, 5.0) / 5.0 * 256.0).min(255.0) as u16,
                    0xBFE4 => self.pfm_abc[0] as u16 | (self.pfm_abc[1] as u16) << 4 | (self.pfm_abc[2] as u16) << 8,
                    0xBFE6 => self.pfm_abc[3] as u16,
                    0xBFEA => (self.iot_ctr as u16) << 5,
                    _ => {
                        self.stats.protocol_errors += 1;
                        0
                    }
                };
                self.st() | (data & 0x0FFF)
            }
            0xC => self.st(),
            _ => {
                self.stats.protocol_errors += 1;
                self.st()
            }
        }
    }

    fn check_fsist_answer(&mut self, answer: u16, intended: bool) {
        let expected = FSIST_ANSWERS[self.question as usize % 32];
        if answer == expected {
            self.stats.ist_correct += 1;
            self.abc_ist = self.abc_ist.saturating_sub(1);
            if self.abc_ist == 0 {
                self.next_question();
            }
        } else {
            // Wrong answer: count and ask the same question again (FR p.5734).
            self.stats.ist_wrong += 1;
            self.stats.ist_wrong_intended += intended as u64;
            self.stats.last_wrong = (self.question, answer);
            self.abc_ist += 1;
            if self.abc_ist >= FSIST_ABC_MAX {
                self.fault(err::MU_FS_IST);
            }
        }
    }

    fn check_pfm_answer(&mut self, answer: u8, now_ns: u64) {
        for k in 0..4 {
            let bit = 1u8 << k;
            if (answer ^ self.pfm_sync) & bit != 0 {
                // Toggled within the period: restart the timer, copy MC bit, heal (FR p.5729).
                self.pfm_sync ^= bit;
                self.pfm_abc[k] = self.pfm_abc[k].saturating_sub(1);
                self.pfm_deadline[k] = now_ns + self.pfm_period_ns(k);
                self.stats.pfm_toggles += 1;
            }
        }
    }
}

/// The MU alone on its select line (unit tests; the board uses [`crate::power_stages::BoardSpiBus`]).
pub struct Simos18SpiBus<'a>(pub &'a mut MonitorDevice);

impl SpiSlave for Simos18SpiBus<'_> {
    fn select(&mut self, ssc: usize, lines: u8, _now_ns: u64) {
        if ssc == MU_SSC {
            self.0.select(lines & (1 << MU_SLSO) != 0);
        }
    }
    fn shift(&mut self, ssc: usize, lines: u8, mtsr: u32, bits: u32, now_ns: u64) -> u32 {
        if ssc == MU_SSC && lines & (1 << MU_SLSO) != 0 && bits == 16 {
            self.0.exchange(mtsr as u16, now_ns) as u32
        } else if lines == 0 {
            (1u64 << bits).wrapping_sub(1) as u32 // nobody drives MRST: pulled up
        } else {
            0
        }
    }
}

#[cfg(test)]
mod tests {
    //! The MC side below is a transcription of the firmware's encoders/decoders and checks
    //! (addresses in the module docs); it drives the MU frame by frame like the 10 ms ECM3 cycle.
    use super::*;

    fn par(w: u16) -> u16 {
        // 0x801F4BC4: force odd parity in bit 0
        let w = w & 0xFFFE;
        w | ((w.count_ones() & 1 == 0) as u16)
    }

    /// MC (firmware) state for one frame.
    #[derive(Clone, Copy, PartialEq, Debug)]
    enum Mc {
        Init,
        Conf,
        Norm,
    }

    struct McSide {
        ist_resp: u16,
        wrg: bool,
        pfm_resp: u8,
        last_q: u8,
    }

    fn tx_frame(mc: Mc, m: &McSide) -> [u16; 7] {
        let (w0, w1) = match mc {
            Mc::Init => (par(0x301E | (2 << 5)), par(0x4002)),
            Mc::Conf => (par(0x5002 | (0xA3 << 4) | (1 << 3)), par(0x6002 | (2 << 9) | (7 << 6))),
            Mc::Norm => {
                let p = m.pfm_resp as u16;
                (par((m.wrg as u16) << 11 | 7 << 12 | (m.ist_resp & 0x3FF) << 1), par(0x8800 | 1 << 10 | p << 5 | (!p & 0xF) << 1))
            }
        };
        [w0, w1, par(0xBFE0), par(0xBFE4), par(0xBFE6), par(0xBFEA), par(0xCFFE)]
    }

    fn exchange(mu: &mut MonitorDevice, tx: [u16; 7], now: u64) -> [u16; 7] {
        let mut bus = Simos18SpiBus(mu);
        bus.select(MU_SSC, 1 << MU_SLSO, now);
        let mut rx = [0u16; 7];
        for (i, w) in tx.iter().enumerate() {
            rx[i] = bus.shift(MU_SSC, 1 << MU_SLSO, *w as u32, 16, now + 16_000 * i as u64) as u16;
        }
        bus.select(MU_SSC, 0, now + 112_000);
        rx
    }

    /// ReadMuData (0x8006C736): common state of words 1..6, else MU_COM.
    fn state_of(rx: &[u16; 7]) -> Option<u16> {
        let s = rx[1] >> 12;
        (rx[1..].iter().all(|w| w >> 12 == s) && matches!(s, 3 | 6 | 0xA | 0xF)).then_some(s)
    }

    #[test]
    fn firmware_checks_accept_the_mu_through_init_conf_norm() {
        let mut mu = MonitorDevice::new();
        mu.adc_volts = 2.5;
        let mut m = McSide { ist_resp: 0xEE, wrg: false, pfm_resp: 0, last_q: 0 };
        let mut now = 0u64;
        let cycle = 10_000_000u64;

        // INIT
        let rx = exchange(&mut mu, tx_frame(Mc::Init, &m), now);
        assert_eq!(state_of(&rx), Some(3));
        assert_eq!((rx[1] >> 5) & 7, 2, "NR_COMP_MU (compatibility check 18.4.5)");
        assert_eq!((rx[2] >> 2) & 0x3FF, 0x3FF, "CKS_INI_2_MU (0x8006C9D0)");
        // CONF
        now += cycle;
        let rx = exchange(&mut mu, tx_frame(Mc::Conf, &m), now);
        assert_eq!(state_of(&rx), Some(6), "MU follows into CONF within the frame");
        assert_eq!((rx[1] >> 2) & 0x3FF, 0x171, "CKS_CONF_1_MU (0x8006C864)");
        assert_eq!((rx[2] >> 2) & 0x3FF, 0x28F, "CKS_CONF_2_MU (0x8006C9A6)");

        // NORM: run 300 cycles of the firmware's FS-IST / PFM behaviour.
        let mut seen: u32 = 0;
        let mut cycles_since_complete = 0;
        let mut toggles_seen = 0;
        for cyc in 0..300 {
            now += cycle;
            let sent_pfm = m.pfm_resp;
            let rx = exchange(&mut mu, tx_frame(Mc::Norm, &m), now);
            assert_eq!(state_of(&rx), Some(0xA), "cycle {cyc}");
            // NORM1 (0x8006C7D4..)
            assert_eq!(rx[1] >> 11 & 1, 1, "PAR_NORM1 == NLC_CKS_NORM_1_PAR_MU");
            assert_eq!(rx[1] >> 9 & 1, 1, "NDIS1 high in NORM (0x8006D078)");
            assert_eq!((rx[1] >> 6) & 7, 0, "ABC_IST_MU stays 0 with correct answers");
            let q = ((rx[1] >> 1) & 0x1F) as u8;
            // NORM2 (0x8006C922..)
            assert_eq!(rx[2] >> 11 & 1, 0, "PAR_NORM2 == NLC_CKS_NORM_2_PAR_MU");
            assert_ne!(rx[2] >> 10 & 1, rx[2] >> 9 & 1, "IGK vs complement (MU_IGK_CPL)");
            let syn = ((rx[2] >> 1) & 0xF) as u8;
            if cyc > 0 {
                // 0x8006D874: bits the MC toggled must come back
                assert_eq!(syn, sent_pfm, "MU_TOG_PFM");
                toggles_seen += 1;
            }
            assert_eq!(rx[3] & 0xFF, 128, "ADC 2.5 V -> 0x80 (VP_ADC_MU = 0x4000)");
            let abc = [rx[4] & 0xF, rx[4] >> 4 & 0xF, rx[4] >> 8 & 0xF, rx[5] & 0xF];
            assert!(abc[..3].iter().all(|&a| a == 0), "PFM0..2 healthy: {abc:?}");
            // FS-IST coverage (0x8006D26C): all 16 within 100 cycles
            seen |= 1 << q;
            cycles_since_complete += 1;
            if seen == 0xFFFF {
                seen = 0;
                cycles_since_complete = 0;
            }
            assert!(cycles_since_complete < 100, "MU_RND_FS_IST");
            // Firmware answers: ID_RESP_MON3[q] (0x8006D4F4), PFM: all tasks done -> toggle all.
            m.ist_resp = FSIST_ANSWERS[q as usize];
            m.last_q = q;
            m.pfm_resp = syn ^ 0xF;
        }
        assert!(toggles_seen > 0);
        assert_eq!(mu.abc_ist(), 0);
        assert_eq!(mu.pfm_abc(), [0, 0, 0, 0], "PFM3 healed from its power-on value 10");
        assert_eq!(mu.stats.ist_wrong, 0);
        assert!(!mu.take_reset_request());
    }

    #[test]
    fn wrong_response_tests_of_the_firmware_see_rise_and_healing() {
        let mut mu = MonitorDevice::new();
        let mut m = McSide { ist_resp: 0xEE, wrg: false, pfm_resp: 0, last_q: 0 };
        let mut now = 0;
        exchange(&mut mu, tx_frame(Mc::Init, &m), now);
        exchange(&mut mu, tx_frame(Mc::Conf, &m), now);
        let mut rx = exchange(&mut mu, tx_frame(Mc::Norm, &m), now);
        // FS-IST WRT (0x8006D368): MC sends 0xEE for the current question.
        let q = ((rx[1] >> 1) & 0x1F) as usize;
        m.ist_resp = 0xEE;
        m.wrg = true;
        now += 10_000_000;
        m.pfm_resp = ((rx[2] >> 1) & 0xF) as u8 ^ 0xF;
        rx = exchange(&mut mu, tx_frame(Mc::Norm, &m), now);
        assert_eq!((rx[1] >> 6) & 7, 1, "ABC_IST_MU rose after the deliberate wrong answer");
        assert_eq!(((rx[1] >> 1) & 0x1F) as usize, q, "same question re-sent");
        // HEAL: correct answer -> ABC falls to 0, next question.
        m.ist_resp = FSIST_ANSWERS[q];
        m.wrg = false;
        now += 10_000_000;
        m.pfm_resp = ((rx[2] >> 1) & 0xF) as u8 ^ 0xF;
        rx = exchange(&mut mu, tx_frame(Mc::Norm, &m), now);
        assert_eq!((rx[1] >> 6) & 7, 0);
        // PFM WRT (0x8006D8EC): MC stops toggling task 0 (echoes SYN) -> ABC_PFM0 must rise
        // within TCC_PFM_WRT_MAX[0] + 1 = 4 cycles (26 ms timer, 10 ms cycle).
        let mut rose_at = None;
        for c in 0..4 {
            let syn = ((rx[2] >> 1) & 0xF) as u8;
            let q = ((rx[1] >> 1) & 0x1F) as usize;
            m.ist_resp = FSIST_ANSWERS[q];
            m.pfm_resp = syn ^ 0xE; // tasks 1..3 toggle, task 0 does not
            now += 10_000_000;
            rx = exchange(&mut mu, tx_frame(Mc::Norm, &m), now);
            if rx[4] & 0xF > 0 && rose_at.is_none() {
                rose_at = Some(c);
                assert_eq!((rx[2] >> 5) & 1, 1, "LF_ABC_PFM_STATE_MU bit 0");
            }
        }
        assert!(rose_at.is_some(), "MU_WRT_PFM_0 would be raised");
        // Healing: toggling again lowers ABC_PFM0 every accepted toggle.
        let before = rx[4] & 0xF;
        let syn = ((rx[2] >> 1) & 0xF) as u8;
        m.pfm_resp = syn ^ 0xF;
        m.ist_resp = FSIST_ANSWERS[((rx[1] >> 1) & 0x1F) as usize];
        now += 10_000_000;
        rx = exchange(&mut mu, tx_frame(Mc::Norm, &m), now);
        assert_eq!(rx[4] & 0xF, before - 1);
    }

    #[test]
    fn persistent_wrong_answers_disable_and_reset_the_mc_and_the_mu_restarts_in_init() {
        let mut mu = MonitorDevice::new();
        let m = McSide { ist_resp: 0x123, wrg: false, pfm_resp: 0, last_q: 0 };
        exchange(&mut mu, tx_frame(Mc::Init, &m), 0);
        exchange(&mut mu, tx_frame(Mc::Norm, &m), 0);
        let mut rx = [0; 7];
        for i in 0..FSIST_ABC_MAX {
            rx = exchange(&mut mu, tx_frame(Mc::Norm, &m), 1000 * i as u64);
        }
        assert_eq!(state_of(&rx), Some(0xF), "MU in DISABLE");
        assert!(!mu.ndis0() && !mu.ndis1(), "power stages disabled");
        assert!(mu.take_reset_request(), "MU resets the MC");
        assert_eq!(mu.err_cod_mu(), err::MU_FS_IST);
        mu.on_mc_reset();
        let rx = exchange(&mut mu, tx_frame(Mc::Init, &m), 0);
        assert_eq!(state_of(&rx), Some(3));
        assert_eq!((rx[1] >> 8) & 0xF, 1, "CTR_RST_MU reported to the MC");
        assert_eq!((rx[1] >> 1) & 0xF, err::MU_FS_IST as u16, "ERR_COD_MU reported");
        // Incompatible MC -> MU_COMP
        let mut mu = MonitorDevice::new();
        exchange(&mut mu, [par(0x301E | 3 << 5), par(0x4002), 0, 0, 0, 0, 0], 0);
        assert!(mu.take_reset_request());
        assert_eq!(mu.err_cod_mu(), err::MU_COMP);
        // A silent MC (no PFM toggles) runs the PFM0 counter into the fault reaction.
        let mut mu = MonitorDevice::new();
        exchange(&mut mu, tx_frame(Mc::Init, &m), 0);
        exchange(&mut mu, tx_frame(Mc::Norm, &m), 0);
        mu.advance(26_000_000 * PFM_ABC_MAX as u64);
        assert_eq!(mu.state(), MuState::Disable);
        assert_eq!(mu.err_cod_mu(), err::MU_PFM_0);
    }

    #[test]
    fn other_frames_on_the_select_line_are_not_interpreted() {
        let mut mu = MonitorDevice::new();
        let rx = exchange(&mut mu, [0xAAAA, 0x177A, 0, 0, 0, 0, 0], 0);
        assert_eq!(&rx[1..], &[0; 6]);
        assert_eq!(mu.stats.frames, 0);
        assert_eq!(mu.state(), MuState::Init);
    }
}
