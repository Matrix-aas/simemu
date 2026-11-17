//! Continental/Temic DSG7 DQ200 Gen2 (`0CW300043H_0002`, Infineon TriCore `SAK-TC1784`) TCU board.
//!
//! The unmodified firmware runs on the shared engine; this board only drives electrical inputs
//! (ADC pin voltages, CAN frames, the ST95640 EEPROM on SSC0 via `tc1784_io`) and observes
//! outputs. It never writes firmware RAM, never decides gears/clutch pressures/pump state.
//!
//! Memory map of DQ200_G2 (`0CW300043H` SW `0002`, `0x2A0000` bytes):
//! - `0x8000_0000..0x8000_D000` SBOOT (`"TEMIC ACCESS OK"` @ `0x80008128`)
//! - `0x8001_0000..0x8001_FFFF` CBOOT (`"DQ200G2_CBoot_V0006"` @ `0x8001BFE8`)
//! - `0x8002_0000..0x8003_FFFF` CAL (`"0CW300043H_1602_ODEK"`); gear ratios x100 at `0x800392F2`
//!   (1..7, R = 3.76 2.27 1.53 1.12 1.18 0.95 0.80 4.17), overall ratios x100 at `0x8003932A`
//! - `0x8004_0000..0x8013_FFFF` ASW (vector table BIV = `0x80040000`)
//!
//! Clock tree from the firmware's own writes (Mcu config `0x8013B7AC`, `0x800E4A5A`): fOSC 20 MHz,
//! NDIV 47 / PDIV 1 / K2DIV 3 -> fCPU 120 MHz; CCUCON0 = 0x80000001 -> fFPI 60 MHz;
//! STM_CLC = 0x200 -> fSTM 30 MHz (the firmware's own delay code computes 30 ticks/us); OS tick
//! = 0x249F0 STM ticks = 5 ms (`0x800E19C0`).

use crate::memory::{BoardVariant, CanFrame};
use crate::{RunStats, Simos18Board};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

// ============================================================================
// Pure-Rust AES-128-ECB + CRC32_IEEE for MQB WFS5 (Immo 5) Protocol
// Verified 1:1 against `fpke` AC 1703 (`fpke-immo-wfs5.md`) & `DQ200_G2` (`0x800486B0`)
// ============================================================================

const AES_SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

const AES_RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

#[inline(always)]
fn gf_mul2(x: u8) -> u8 {
    (x << 1) ^ if (x & 0x80) != 0 { 0x1b } else { 0x00 }
}

/// Pure-Rust AES-128 single-block ECB encryption (`FIPS-197`).
pub fn aes128_ecb_encrypt(key: &[u8; 16], input: &[u8; 16]) -> [u8; 16] {
    let mut rk = [0u8; 176];
    rk[0..16].copy_from_slice(key);
    for i in 4..44 {
        let mut temp = [
            rk[(i - 1) * 4],
            rk[(i - 1) * 4 + 1],
            rk[(i - 1) * 4 + 2],
            rk[(i - 1) * 4 + 3],
        ];
        if i % 4 == 0 {
            temp = [
                AES_SBOX[temp[1] as usize] ^ AES_RCON[(i / 4) - 1],
                AES_SBOX[temp[2] as usize],
                AES_SBOX[temp[3] as usize],
                AES_SBOX[temp[0] as usize],
            ];
        }
        for j in 0..4 {
            rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ temp[j];
        }
    }

    let mut s = *input;
    for i in 0..16 {
        s[i] ^= rk[i];
    }

    for round in 1..=10 {
        for b in &mut s {
            *b = AES_SBOX[*b as usize];
        }
        // ShiftRows
        s = [
            s[0], s[5], s[10], s[15],
            s[4], s[9], s[14], s[3],
            s[8], s[13], s[2], s[7],
            s[12], s[1], s[6], s[11],
        ];
        // MixColumns (rounds 1..9)
        if round < 10 {
            for col in 0..4 {
                let c = col * 4;
                let (a0, a1, a2, a3) = (s[c], s[c + 1], s[c + 2], s[c + 3]);
                let t = a0 ^ a1 ^ a2 ^ a3;
                s[c] ^= t ^ gf_mul2(a0 ^ a1);
                s[c + 1] ^= t ^ gf_mul2(a1 ^ a2);
                s[c + 2] ^= t ^ gf_mul2(a2 ^ a3);
                s[c + 3] ^= t ^ gf_mul2(a3 ^ a0);
            }
        }
        // AddRoundKey
        let r_off = round * 16;
        for i in 0..16 {
            s[i] ^= rk[r_off + i];
        }
    }
    s
}

/// Standard IEEE 802.3 CRC-32 (`poly = 0xEDB88320`, `init = 0xFFFFFFFF`, `xorout = 0xFFFFFFFF`).
/// Matches `DQ200_G2` `0x80048044` (`table 0x801201C0`) and `fpke` AC `1703` `0x0013F128`.
pub fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

/// WFS5 (`Immo 5` MQB) Cryptographic Partner Credentials (`SIMOS18.1` Partner 0 or `DQ200_G2` Partner 1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wfs5ImmoPartner {
    pub name: String,
    pub req_can_id: u32,
    pub ans_can_id: u32,
    /// 16-byte partner key `K_n` (`CS16`)
    pub key: [u8; 16],
    /// 1-byte `Power Class` (`B` / `cfg`)
    pub power_class: u8,
    /// 2-byte `ID_kombi` (`ID_ac` of instrument cluster `fpke`)
    pub id_kombi: [u8; 2],
    /// 2-byte `ID_partner` (`ID_ecu` or `ID_tcu`)
    pub id_partner: [u8; 2],
    /// 17-char VIN extracted from EEPROM
    pub vin: String,
}

impl Wfs5ImmoPartner {
    pub const WFS5_C4: [u8; 4] = [0x9C, 0xF7, 0xFE, 0xB7];
    pub const WFS5_C7P: [u8; 7] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
    pub const WFS5_C7A: [u8; 7] = [0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E];

    /// Extract the genuine `WFS5` credentials from a `DQ200_G2` (`0CW`) 8 KB SPI EEPROM dump
    /// (`0x01E8..0x02D3` -> RAM `0xD000_0B3E..0xD000_0B73`, unpacked by `0x8004_B65E` & `0x8004_B770`).
    pub fn from_dq200_eeprom(eeprom: &[u8]) -> Self {
        let mut key = [0u8; 16];
        let mut id_kombi = [0u8; 2];
        let mut id_partner = [0u8; 2];
        let mut power_class = 0u8;
        let mut vin = String::new();

        if eeprom.len() >= 0x0260 {
            // Proven in `0x8004_B65E` (`a12 = &eeprom[0x01E8]`):
            // - `a12[0x01..0x11]` (`0x01E9..0x01F9`): 16-byte AES-128 key `K1` (`0xD000_0B48..0xD000_0B57`)
            // - `a12[0x11..0x22]` (`0x01F9..0x020A`): 17-byte ASCII VIN (`0xD000_0B5C..0xD000_0B6C`)
            // - `a12[0x23]` (`0x020B`): Power Class `B` (`0xD000_0B42`)
            // - `a12[0x25..0x27]` (`0x020D..0x020F`): 2-byte `ID_kombi` (`0xD000_0B46..0xD000_0B47`)
            // - `0x0259..0x025B`: NOT an ID. 0x8004B770 copies block-1 bytes 1..4 to 0xD0000B58..5B,
            //   which 0x8004D2E6 consumes as a 32-bit PRNG state for the next challenge
            //   (S = sum(b_i << 8i); (S + f()) * 0x6F + 0xB). `id_partner` therefore holds a stale
            //   seed; it only affects `verify_challenge` (the 0x013 answer does not use it).
            //   ID_tcu is not in the EEPROM: see `from_dq200_images`.
            key.copy_from_slice(&eeprom[0x01E9..0x01F9]);
            power_class = eeprom[0x020B];
            id_kombi.copy_from_slice(&eeprom[0x020D..0x020F]);
            id_partner.copy_from_slice(&eeprom[0x0259..0x025B]);
            let v = eeprom[0x01F9..0x020A]
                .iter()
                .copied()
                .filter(|b| (0x20..=0x7E).contains(b))
                .map(|b| b as char)
                .collect::<String>();
            if !v.is_empty() {
                vin = v;
            }
        }

        Self {
            name: "DQ200_G2_0CW (Partner 1)".to_string(),
            req_can_id: 0x012,
            ans_can_id: 0x013,
            key,
            power_class,
            id_kombi,
            id_partner,
            vin,
        }
    }

    /// Like [`Self::from_dq200_eeprom`], with `ID_tcu` derived as the firmware does (0x8004738C):
    /// low two bytes of `(u16@flash 0xC304 + 1) * (u32@flash 0xC100 + 1)` (SBOOT production data,
    /// reached through 0xD000CBE0/0xD000CBDC). VERIFIED: every 0x012 challenge the unmodified
    /// firmware sends on the bench (`0CW300043H_0002` images) carries ID_tcu = [0xC1, 0x85].
    pub fn from_dq200_images(flash: &[u8], eeprom: &[u8]) -> Self {
        let mut p = Self::from_dq200_eeprom(eeprom);
        if let (Some(a), Some(b)) = (flash.get(0xC304..0xC306), flash.get(0xC100..0xC104)) {
            let a = u16::from_le_bytes([a[0], a[1]]) as u32 + 1;
            let b = u32::from_le_bytes([b[0], b[1], b[2], b[3]]).wrapping_add(1);
            p.id_partner = (a.wrapping_mul(b) as u16).to_le_bytes();
        }
        p
    }

    /// Derive Partner 0 (`SIMOS18.1`, `0x010 -> 0x011`) from MQB vehicle master slots `A` and `B`
    /// per `fpke` AC `1703` `0x0013FD6E`: `K0 = A[0..4] || B[4..12] || A[12..16]`.
    pub fn from_mqb_slots_ecu(
        slot_a: &[u8; 16],
        slot_b: &[u8; 16],
        power_class: u8,
        id_kombi: [u8; 2],
        id_ecu: [u8; 2],
        vin: &str,
    ) -> Self {
        let mut key = [0u8; 16];
        key[0..4].copy_from_slice(&slot_a[0..4]);
        key[4..12].copy_from_slice(&slot_b[4..12]);
        key[12..16].copy_from_slice(&slot_a[12..16]);
        Self {
            name: "SIMOS18.1 (Partner 0)".to_string(),
            req_can_id: 0x010,
            ans_can_id: 0x011,
            key,
            power_class,
            id_kombi,
            id_partner: id_ecu,
            vin: vin.to_string(),
        }
    }

    /// Derive Partner 1 (`DQ200_G2`, `0x012 -> 0x013`) from MQB vehicle master slots `A` and `B`
    /// per `fpke` AC `1703` `0x0013FD6E`: `K1 = B[0..4] || A[4..7] || B[7..12] || A[12..16]`.
    pub fn from_mqb_slots_tcu(
        slot_a: &[u8; 16],
        slot_b: &[u8; 16],
        power_class: u8,
        id_kombi: [u8; 2],
        id_tcu: [u8; 2],
        vin: &str,
    ) -> Self {
        let mut key = [0u8; 16];
        key[0..4].copy_from_slice(&slot_b[0..4]);
        key[4..7].copy_from_slice(&slot_a[4..7]);
        key[7..12].copy_from_slice(&slot_b[7..12]);
        key[12..16].copy_from_slice(&slot_a[12..16]);
        Self {
            name: "DQ200_G2_0CW (Partner 1)".to_string(),
            req_can_id: 0x012,
            ans_can_id: 0x013,
            key,
            power_class,
            id_kombi,
            id_partner: id_tcu,
            vin: vin.to_string(),
        }
    }

    /// The same partner with key `K_n` and power class `B` replaced.
    pub fn with_key(mut self, key: [u8; 16], power_class: u8) -> Self {
        self.key = key;
        self.power_class = power_class;
        self
    }

    /// Verify a partner's 8-byte challenge frame (`0x010` or `0x012`):
    /// `rx = P[0..4] || c0 || c1 || (c2 ^ ID_partner[0]) || (c3 ^ ID_partner[1])`.
    pub fn verify_challenge(&self, rx: &[u8; 8]) -> bool {
        let mut block = [0u8; 16];
        block[0..4].copy_from_slice(&Self::WFS5_C4);
        block[4] = self.power_class;
        block[5..9].copy_from_slice(&rx[0..4]);
        block[9..16].copy_from_slice(&Self::WFS5_C7P);
        let enc = aes128_ecb_encrypt(&self.key, &block);
        let c = crc32_ieee(&enc).to_le_bytes();
        rx[4] == c[0]
            && rx[5] == c[1]
            && (rx[6] ^ c[2]) == self.id_partner[0]
            && (rx[7] ^ c[3]) == self.id_partner[1]
    }

    /// Compute the 8-byte `fpke` WFS5 authorization response (`0x011` or `0x013`) with `S = 0x1F`:
    /// `tx = a0 || a1 || (a2 ^ ID_kombi[0]) || (a3 ^ ID_kombi[1]) || 00 00 00 || 0x1F`,
    /// where `a = CRC32_IEEE(AES128_ECB(K_n, C4 || B || ~P || C7A))`.
    pub fn answer_challenge(&self, rx: &[u8; 8]) -> [u8; 8] {
        let mut block = [0u8; 16];
        block[0..4].copy_from_slice(&Self::WFS5_C4);
        block[4] = self.power_class;
        block[5] = !rx[0];
        block[6] = !rx[1];
        block[7] = !rx[2];
        block[8] = !rx[3];
        block[9..16].copy_from_slice(&Self::WFS5_C7A);
        let enc = aes128_ecb_encrypt(&self.key, &block);
        let a = crc32_ieee(&enc).to_le_bytes();
        [
            a[0],
            a[1],
            a[2] ^ self.id_kombi[0],
            a[3] ^ self.id_kombi[1],
            0x00,
            0x00,
            0x00,
            0x1F, // S = 0x1F: Key valid + ELV ok + Partner authorized!
        ]
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Dq200TcuImageInfo {
    pub mcu_model: &'static str,
    pub flash_size_bytes: usize,
    pub eeprom_size_bytes: usize,
    pub cboot_version: String,
    pub cal_header: String,
    pub sw_version_tag: String,
    pub asw_build_tag: String,
    pub sboot_entry_va: u32,
    pub cboot_entry_va: u32,
    pub asw_entry_va: u32,
}

impl Dq200TcuImageInfo {
    pub const EXPECTED_FLASH_SIZE: usize = 0x2A0000; // 2,752,512 bytes
    pub const EXPECTED_EEPROM_SIZE: usize = 0x2000; // 8,192 bytes

    /// Inspect a raw `DQ200_G2` (`0CW`) TriCore TC1784 Flash + EEPROM dump pair and extract hardware & SW metadata.
    pub fn inspect_files(flash_path: &Path, eeprom_path: Option<&Path>) -> Result<Self, String> {
        let flash = fs::read(flash_path)
            .map_err(|e| format!("Failed to read DQ200 Flash {}: {}", flash_path.display(), e))?;
        let eeprom_len = eeprom_path
            .and_then(|p| fs::metadata(p).ok())
            .map(|m| m.len() as usize)
            .unwrap_or(0);
        Self::from_bytes(&flash, eeprom_len)
    }

    pub fn from_bytes(flash: &[u8], eeprom_size_bytes: usize) -> Result<Self, String> {
        if flash.len() < 0x140000 {
            return Err(format!(
                "DQ200 Flash image too small: {} bytes (expected >= 0x140000)",
                flash.len()
            ));
        }

        let extract_ascii = |start: usize, len: usize| -> String {
            let Some(slice) = start.checked_add(len).and_then(|end| flash.get(start..end)) else {
                return String::new();
            };
            slice
                .iter()
                .copied()
                .filter(|b| (0x20..=0x7E).contains(b))
                .map(|b| b as char)
                .collect::<String>()
                .trim()
                .to_string()
        };

        let cboot_version = extract_ascii(0x01BFE8, 20);
        let cal_header = extract_ascii(0x020000, 48);
        let sw_version_tag = extract_ascii(0x03FFA0, 20);
        let asw_build_tag = extract_ascii(0x13FFD0, 16);

        Ok(Self {
            mcu_model: "Infineon SAK-TC1784 (TriCore 1.3.1 @ 180 MHz)",
            flash_size_bytes: flash.len(),
            eeprom_size_bytes,
            cboot_version,
            cal_header,
            sw_version_tag,
            asw_build_tag,
            sboot_entry_va: 0x8000_0000,
            cboot_entry_va: 0x8001_0000,
            asw_entry_va: 0x8004_0000,
        })
    }
}

/// Observation-only snapshot of DQ200_G2 firmware variables (read from DSPR, never written).
/// Addresses from the Getriebe_11/12 packers (`docs/DQ200_G2_REVERSE_ENGINEERING_CHECKPOINT.md`
/// §1.2-1.4: 0x800CC0CE / 0x800CC136 / 0x800CC2B0) and the drivers cited per field.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Dq200LiveState {
    /// Engaged gear `Istgang` (0xD0007D42, int8: -2 error, -1 R, 0 N, 1..7). UNVERIFIED address: on
    /// the bench in main states 2/3 the byte takes values outside -2..7 (old checkpoint doc claim).
    pub istgang: i8,
    /// Target gear `Zielgang` (0xD0007D40, int8).
    pub zielgang: i8,
    /// Low byte of 0xD00077BE. Not the selector position (old checkpoint claim): 0x800C5180 copies the
    /// i16 0xD00068F0 there. The decoded selector is 0xD0005244 (raw `WH_SensorPos_roh`) / 0xD0005245
    /// (0 P, 1 R, 2 N, 3 forward; `0x8007240A`). Kept for the UI field name.
    pub waehlhebel: u8,
    /// Emergency mode `Notlauf` (0xD00077BA, 0 = normal).
    pub notlauf: u8,
    /// Main mechatronic state machine (0xD0007EC0, switch 0x800D5BC8, jump table 0x8013A27C). VERIFIED-S
    /// transitions: 1 -> 2 when the CAN messages enabled by CAL byte +0x52 are received (0x0A7, 0x3BE,
    /// 0x0FD, 0x0B2, 0x6B7, 0x0AF by bit) and 0xD0006F75 != 2 (else 14 = shut-down); 2 -> 3/4 when the
    /// engine runs (0xD0005113 == 2) or 0xD0004D1B, -> 4 when the system pressure is sufficient
    /// (0xD0004E23); 4 -> 5 needs 0xD0007E1B == 3 and the flag at 0xD000608C/0xD0004D0E; 5 -> 8 ready.
    pub main_state: u8,
    /// G270 pressure decoded by the firmware at 0x800EDBF0 (0xD000B2B8, 0.01 bar) and its
    /// validity (0xD000B2BE: 0 below range, 1 valid, 2 above range).
    pub pressure_bar: f32,
    pub pressure_status: u8,
    /// Power-stage release word (0xD0003494, 0x21522152 = hydraulics released at 0x800EC1F4).
    pub power_stage_word: u32,
    /// WFS5 manager state (0x8004BC90, high nibble of 0xD0000BA4): 0xA = virgin/defaults (EEPROM
    /// WFS5 blocks unreadable), 9 = challenging (0x012 repeated every ~60 ms while the answer is
    /// missing or wrong), 7 = after a valid 0x013 (challenges stop).
    pub wfs5_state: u8,
    /// `wfs5_state == 7`. Evidence: reached only with a correct 0x013 (an answer with one flipped
    /// MAC bit keeps state 9). The old flag 0xD0000BDB never changes on the bench (doc error).
    pub wfs5_authorized: bool,
    /// Safety-software output state (0xD001FF58 with complement at +4; 0x80041602): 0x55555555 =
    /// outputs enabled (set by 0x8004159E with key 0xA5A5C3C3), 0x2AAAAAAA = shut-off (0x8004156C;
    /// then 0x80045AF0 forces the valve output pins to input every cycle).
    pub safety_word: u32,
    /// Safety error counter (0xD001FF7C, complement at +4): >= 22 latches the shut-off (0x800445A0).
    pub safety_errors: u32,
    /// Filtered system pressure the state machine uses (0xD0007E30, 0.01 bar; read by 0x800D4ACA).
    pub system_pressure_bar: f32,
    /// Pressure-sufficient state (0xD0004E17, 0x8006E0FA): 0 low / 1 / 2 above the temperature-
    /// dependent threshold (CAL 0x8002139A/0x800213AA: 19.6/21.3/21.3/33.0 bar at -40/-20/0/140 °C,
    /// 2 bar hysteresis 0x800213D8).
    pub pressure_ok: u8,
    /// Engine-running detection (0xD0005113, 0x8006FC94): 0 stopped .. 2 running; compares
    /// max(engine speed from CAN 0xD0007B3E, input speed 0xD0007CA8) with CAL thresholds.
    pub engine_running: u8,
}

/// Actuator-side state observable on emulated hardware (see [`Dq200TcuBoard::decode_outputs`]).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Dq200Outputs {
    /// Hydraulic power stage released by the firmware (0xD0003494 = 0x21522152, written at
    /// 0x800EC1F4 when `ADC[20] >= 0x52F`; 0x5EADDEAD = undervoltage shut-off).
    pub power_stage_released: bool,
    /// Most recent (tx, rx) SPI frames per output-side slave, keyed by chip select. Their command
    /// format is UNVERIFIED (slave devices not identified).
    pub spi_frames: Vec<(String, Vec<(u32, u32)>)>,
    /// Measured duty 0..1 of every GPTA PWM output cell: (name, duty, period s).
    pub pwm: Vec<(String, f64, f64)>,
    /// Physical commands in `simemu_physics::TcuOutputs` channel order (0 N433, 1 N434, 2 N435 K1,
    /// 3 N436, 4 N437, 5 N438, 6 N439 K2, 7 N440); `None` = PWM generator not running.
    /// Valve assignment: see [`SOLENOID_PWM_CELLS`] (VERIFIED-S).
    pub solenoid_duty: [Option<f64>; 12],
    /// Coil currents (A, mean over `tc1784_io::COIL_MEAN_S`) of the same channels from the board's
    /// coil model, which the firmware's current controllers regulate (0 while the pins are not driven).
    pub solenoid_current_a: [f64; 12],
    /// V401 pump motor PWM duty (unused: V401 is driven via ASC0/P2, see [`PUMP_PWM_CELL`]).
    pub pump_motor_duty: Option<f64>,
    /// V401 pump shaft speed of the board-level motor model (`tc1784_io::pump`), rad/s.
    pub pump_shaft_rad_s: Option<f64>,
    /// Commands that cannot be derived yet.
    pub unverified: Vec<String>,
}

/// The eight 3 kHz valve PWM generators the firmware builds in GPTA0 (Pwm driver init
/// 0x800E51D2/0x800E5244, period ISR via 0x800E48CC -> 0x800E5108): reset timer LTCk on CLK0 (fGPTA),
/// LTCk+1 period compare XR = 0x341A (period 13340 ticks = 333.3 us at 40.02 MHz), LTCk+2/LTCk+3 duty
/// compares on the two select levels (coherent update) linked to the output cell LTCk+3.
/// Pwm channel -> output cell: `tc1784_io::VALVE_PWM_CELLS` (VERIFIED-S, table 0x8013B884).
/// The valve driver groups the channels into two modules of four (config 0x8013AEF0, 0x28-byte
/// entries, +0x1C -> channel list): module 0 = Pwm ch [0, 1, 6, 5], module 1 = [2, 4, 3, 7]
/// (VERIFIED-S). Actuator i = current regulator i (valve output layer 0x800CDA02: valve-id table
/// 0x8013A0FC = [0, 1, 4, 5, 6, 7, 2, 3], type table 0x8013A104 = [0, 0, 1, 1, 1, 1, 2, 2]; 0x800E68E8
/// maps valve id -> regulator by 0x8013C9C4 = [0, 1, 6, 7, 2, 3, 4, 5]); the demand of actuator i is
/// the regulator's current set-point in 0.1 mA (`0xD0007EE8[i]` -> `0xD000A3D8 + 8 i` -> clamp 0..15000
/// in `0x800DDF88`). Function per regulator (fifth pass, VERIFIED-S by where each demand comes from,
/// `0x800D8EBA`):
/// - 0/1 (type 0): `0x80093B2C(s)` = `0xD0006090 + 8 s + 6`, the clutch control of clutch s (its +2 is
///   the target clutch travel the level-2 monitor `0x800F933E` compares with the measured travel of
///   the clutch sensor struct 0xD0007734 + 0x18 s) -> clutch valves K1 / K2;
/// - 2..5 (type 1): fork position controllers `0xD00053B4..` -> fork valves 1/3, 2/4, 5/7, R/6;
/// - 6/7 (type 2): `0x80072230(s)` = `0xD0005194[s]`, the sub-gearbox pressure function `0x80071D68`
///   (pressure request max(clutch 0xD0005180, shift 0xD000517C) ramped by CAL +0x5E/10 ms, converted
///   to current through the firmware's rising p -> I table CAL `0x80030E60`/+0x18 or its adapted copy
///   0xD000519C/0xD00051B0; the resulting pressure 0xD0005188 gates every fork movement via
///   0xD0005BE0, `0x80079854`) -> sub-gearbox pressure valves N436 / N440.
///
/// The fourth pass had types 0 and 2 the other way round (its "clutch hold current 0.53 A in state 4"
/// was the 12-bar sub-gearbox pressure ramp of `0x80071D68`: 0 -> 1200 in 50 / 10 ms = 22 mA / 10 ms).
/// Regulator -> Pwm channel (period ISR 0x800EB430): 0 -> 5, 1 -> 3, 2 -> 0, 3 -> 7, 4 -> 6, 5 -> 2,
/// 6 -> 1, 7 -> 4. `TcuOutputs` order N433 (1/3), N434 (5/7), N435 K1, N436 sub 1, N437 (2/4), N438
/// (R/6), N439 K2, N440 sub 2 -> Pwm channel:
pub const VALVE_PWM_CHANNEL_OF_SOLENOID: [usize; 8] = [0, 6, 5, 1, 7, 2, 3, 4];
/// Output cell of each `TcuOutputs` solenoid channel (derived from the two tables above).
pub const SOLENOID_PWM_CELLS: [u8; 8] = {
    let mut c = [0u8; 8];
    let mut i = 0;
    while i < 8 {
        c[i] = crate::tc1784_io::VALVE_PWM_CELLS[VALVE_PWM_CHANNEL_OF_SOLENOID[i]];
        i += 1;
    }
    c
};
/// 20 kHz PWM in LTCA2 (programmed at 0x800DACBA / 0x800DA96C: reset timer LTC26 on CLK0, period
/// compare LTC27/28 XR = 0x7D0, duty compares LTC29/30 on the two select levels; LTC30 is the last
/// cell of the chain = the bridge PWM). V401 is a sensorless brushless motor commutated by the
/// firmware through the driver chip (P5.9 toggle per step, zero-crossing feedback on IN47), modelled
/// on the board by `tc1784_io::pump` (docs/DQ200_IO_MAP.md §4).
pub const PUMP_PWM_CELL: (u8, u8) = (2, 30);
/// Gear pump of V401 as seen by the motor: pressure torque D / (2 pi eta_hm) and viscous drag. The
/// same UNVERIFIED pump the plant uses for delivery (`simemu_physics::mechatronic::Dq200HydraulicsHw::
/// dq200().pump`: 0.6 cm^3/rev, eta_hm 0.85, drag 2e-6 N m s at 0.01 Pa s; equality checked by
/// `studio::tcu_io::tests::board_pump_matches_plant_pump`).
pub const PUMP_TORQUE_PER_PA: f64 = 0.6e-6 / (2.0 * std::f64::consts::PI * 0.85);
pub const PUMP_VISC_NM_S: f64 = 2e-6;

/// GPTA0 input lines of the three active wheel-speed sensors (sensor -> (capture line, counter
/// line)), from the Icu channel table 0x8013B2D8 and the MRA routing: sensor 0 = GTC18/19 <- IN50,
/// LTC50 <- IN51; sensor 1 = GTC20/21 <- IN22, LTC49 <- IN23; sensor 2 = GTC16 <- IN20, LTC48 <-
/// IN21. Both lines of a pair carry the same sensor signal (UNVERIFIED board wiring).
pub const SPEED_SENSOR_LINES: [(u8, u8); 3] = [(50, 51), (22, 23), (20, 21)];
/// Pulses per revolution of the speed sensors (CAL 0x800238A8 + 8, read via 0x800BD12A):
/// the firmware computes speed = 0x478F3E90 * edges / (teeth * dt_GT0 / 20) in 0.1 rpm, i.e.
/// 600 * f / teeth (VERIFIED on the emulator: 0.1 rpm values at 0xD0007536/3A/3E track f exactly).
pub const SPEED_SENSOR_TEETH: [u32; 3] = [18, 36, 60];
/// Pulse widths of the direction protocol (0x800E8100: sensor 0 accepts 50 +- 20 us / 102 +- 36 us,
/// sensor 1 38 +- 14 us / 76 +- 28 us as direction +1 / -1; sensor 2 carries no direction):
/// a TLE4942-style 45 us / 90 us pulse satisfies both windows.
pub const SPEED_PULSE_S: [f64; 2] = [45e-6, 90e-6];
/// GPTA0 input lines of the clutch travel PWM sensors (sensor -> (signal A, complementary B)):
/// Icu channels 0/1 = DCM1 (FPC2 <- IN4) / DCM3 (FPC5 <- IN10), channels 2/3 = GTC7 (FPC1 <- IN2) /
/// DCM0 (FPC0 <- IN0); the firmware (0x800F1720) measures duty = 10000 * t_high / period and checks
/// 750 <= duty <= 9250 and 9500 <= A + B <= 10500 (CAL 0x80031F4C + 0x30..0x36).
pub const CLUTCH_SENSOR_LINES: [(u8, u8); 2] = [(4, 10), (2, 0)];
/// PWM frequency of the clutch travel sensors. UNVERIFIED (the firmware only evaluates the duty).
pub const CLUTCH_SENSOR_HZ: f64 = 1000.0;
/// Clutch travel sensor characteristic of the firmware (CAL `[0xD000C4A8]` = 0x80031F4C on this
/// image, VERIFIED-S): per Icu channel a two-point line duty (0.01 %) -> travel (0.01 mm) evaluated by
/// `0x80135A58` (PSPR copy 0xD40019B0, interpolation 0xD4000542) into 0xD000C4AC: channel A 1000 -> 0,
/// 9000 -> 2900; channel B 1000 -> 2900, 9000 -> 0. The clutch travel the firmware works with
/// (0xD0007E68/6A = mean of both channels, `0x80136736`) is therefore 29.00 mm over 10 % .. 90 %
/// duty. Unit 0.01 mm: HYP (consistent with the adapted values 0xD0003F00, docs/DQ200_IO_MAP.md §6.1).
pub const CLUTCH_SENSOR_CAL: u32 = 0x8003_1F4C;
/// Travel at 90 % duty of channel A, mm ([`CLUTCH_SENSOR_CAL`]).
pub const CLUTCH_TRAVEL_FULL_MM: f64 = 29.0;

/// Duty of channel A of a clutch travel sensor (G617/G618) at actuator travel `mm`: the inverse of the
/// firmware's own characteristic [`CLUTCH_SENSOR_CAL`] (channel B carries 1 - duty). Sensor datasheet
/// and mounting are unknown; the characteristic is the one the firmware is calibrated for.
pub fn clutch_travel_duty(mm: f64) -> f64 {
    let mm = if mm.is_finite() { mm } else { 0.0 };
    0.10 + 0.80 * (mm / CLUTCH_TRAVEL_FULL_MM).clamp(0.0, 1.0)
}

/// Native Infineon TC1784 (`DQ200_G2` `0CW300043H_0002`) board: the unmodified 2.625 MB PFlash
/// image runs on the shared instruction-accounted TriCore engine (BoardVariant::Dq200Tc1784);
/// the 8 KB ST95640 EEPROM sits on SSC0 (`tc1784_io`). The board only drives electrical inputs
/// (ADC pin voltages, CAN frames) and observes outputs; it never writes firmware RAM.
pub struct Dq200TcuBoard {
    pub cpu: Simos18Board,
    pub info: Dq200TcuImageInfo,
    /// Fork travel sensor characteristic of the attached mechatronic ([`ForkSensors`]).
    pub forks: ForkSensors,
}

/// Fork (gear actuator) travel sensors G487..G490: two Hall outputs (A, B) per fork, read ratiometric
/// against the 5 V sensor supply (`transfer::fork_pin_v_norm`). Characteristic VERIFIED-S from the
/// firmware: `0x800C4F9A` -> `0x800C4E48` maps each output's normalised code through a 20-point table
/// (`FUN_d4000ae0`, x = code at CAL `[0xD000CCFC] + 0x120 + 0x50 c + 0x28 k`, y = position in 0.01 mm
/// at `+ 0x260`; pointer `0xD000CCFC` = `0x80038890` on this image, VERIFIED-T). Output A spans
/// -14.00..+4.00 mm, output B -4.00..+14.00 mm, saturating outside (sigmoid Hall response); the
/// plausibility check `0x800CF27A` (P0914) needs B inside a band around f(A) (CAL `[0xD000CD14] +
/// 0x28 / + 0x50`) — true for a sensor that follows this characteristic. The board drives each output
/// with the inverse of the table (the sensor as the firmware's own calibration describes it).
/// Fork c = sensor pair c (`ADC[26]/[25]`, `[8]/[6]`, `[10]/[9]`, `[15]/[14]`, `0x8013C9B7/BB`) carries the
/// gears of the adaptation ids `0x801384E0` (+ side) / `0x801384DC` (- side): 1/3, 2/4, 5/7, R/6
/// (VERIFIED-S, ids = gears with R = 8; learned end positions `0xD0003E78`, VERIFIED-T: +8.75 (1),
/// +9.30 (2), -8.55 (3), -8.55 (4), +9.50 (5), -8.30 (6), -8.30 (7), +8.45 (R) mm). Sensor pair c ↔
/// fork c is HYP (consistent with the per-fork tables and the plausibility loop order).
#[derive(Clone, Debug)]
pub struct ForkSensors {
    /// (x = normalised code, y = 0.01 mm) per fork, output A/B, point.
    pub table: [[[(i16, i16); 20]; 2]; 4],
}

/// CAL block of the fork sensor characteristic (see [`ForkSensors`]).
pub const FORK_SENSOR_CAL: u32 = 0x8003_8890;

impl ForkSensors {
    pub fn from_flash(flash: &[u8]) -> Self {
        let rd = |a: u32| {
            let o = (a - 0x8000_0000) as usize;
            flash.get(o..o + 2).map_or(0, |b| i16::from_le_bytes([b[0], b[1]]))
        };
        let mut table = [[[(0i16, 0i16); 20]; 2]; 4];
        for (c, f) in table.iter_mut().enumerate() {
            for (k, h) in f.iter_mut().enumerate() {
                let o = FORK_SENSOR_CAL + 0x50 * c as u32 + 0x28 * k as u32;
                for (i, p) in h.iter_mut().enumerate() {
                    *p = (rd(o + 0x120 + 2 * i as u32), rd(o + 0x260 + 2 * i as u32));
                }
            }
        }
        Self { table }
    }

    /// Normalised code of output `k` of fork `c` at travel `mm`: the inverse of the firmware table
    /// inside its range; outside it the output returns towards its midpoint like the normal field
    /// component of a magnet passing a Hall element, `u / (u^2 + z^2)^(5/2)` around the flank centre
    /// with its extrema at the table ends (`u = ±z/2`). Why not a saturated output (the earlier
    /// model): the firmware's pair check (`0x800CF4FA`: `lo(A) < B < hi(A)`, 20-point band CAL
    /// `[0xD000CD14]` = `0x80038CA8` x / +0x28 lo / +0x50 hi) rejects a saturated A beyond ~8.5 mm
    /// (events 0x27..0x2A = P2832/P2837/P283C/P2841, reaction: sub-gearbox off, then the level-2
    /// check `0x800FB846` resets the TCU); with this tail every pair from -13 to +13 mm lies inside the
    /// band on all four forks, and the blend `0x80136610` (A below CAL `0x80038F78` threshold, B above
    /// its own) still reads the travel from the in-range output. Tail shape: dipole field (UNVERIFIED
    /// for this sensor — no datasheet; only its extremum spacing comes from the firmware tables).
    pub fn code(&self, c: usize, k: usize, mm: f64) -> f64 {
        let Some(t) = self.table.get(c).and_then(|f| f.get(k)) else {
            return 0.0;
        };
        let y = (if mm.is_finite() { mm } else { 0.0 }) * 100.0;
        let ((x0, y0), (x1, y1)) = ((t[0].0 as f64, t[0].1 as f64), (t[19].0 as f64, t[19].1 as f64));
        if y <= y0 || y >= y1 {
            let (half, base) = ((y1 - y0) / 2.0, (x0 + x1) / 2.0);
            if half.abs() <= 1e-9 {
                return base;
            }
            let g = |u: f64| u / (u * u + 4.0 * half * half).powf(2.5);
            return base + (x1 - base) * g(y - (y0 + y1) / 2.0) / g(half);
        }
        for w in t.windows(2) {
            let ((x0, y0), (x1, y1)) = ((w[0].0 as f64, w[0].1 as f64), (w[1].0 as f64, w[1].1 as f64));
            if y <= y1 && y1 > y0 {
                return x0 + (x1 - x0) * (y - y0) / (y1 - y0);
            }
        }
        x1
    }
}

/// Board-level electrical transfer functions, each the exact inverse of the firmware decode.
pub mod transfer {
    use crate::tc1784_io::ADC_VAREF;

    /// 12-bit ADC full-scale code (`tc1784_io` converts V -> round(V / VAREF * 4095)).
    const FULL: f64 = 4095.0;

    /// Pin voltage that the emulated 12-bit ADC converts to exactly `code`.
    pub fn code_to_pin_v(code: u32) -> f32 {
        (code.min(4095) as f64 / FULL * ADC_VAREF) as f32
    }

    /// Firmware supply scaling 0x800ED970: mV = (((5000 * gain) >> 12) * code >> 8) + 1) >> 1,
    /// gains from flash 0x8013EC4C: 0x805 for KL30 (`ADC[20]`), KL15 (`ADC[13]`), power-stage supplies
    /// (`ADC[30]`, `ADC[31]`); 0x247 for the 5 V sensor supplies (`ADC[1]`, `ADC[19]`).
    pub fn fw_supply_mv(code: u32, gain: u32) -> u32 {
        (((((5000 * gain) >> 12) * code) >> 8) + 1) >> 1
    }

    /// Board divider (input volts per ADC pin volt) implied by a gain: the firmware's mV/LSB
    /// divided by the 12-bit pin LSB VAREF/4095. gain 0x805 -> 4.0086, gain 0x247 -> 1.1373.
    pub fn divider(gain: u32) -> f64 {
        (((5000 * gain) >> 12) as f64 / 512.0) / (ADC_VAREF * 1000.0 / FULL)
    }

    /// Smallest code whose firmware-decoded value is closest to `target` for a monotonic
    /// (non-decreasing) decode: binary search (the board feeds these every 1 ms co-simulation window).
    fn invert(target: f64, decode: impl Fn(u32) -> f64) -> u32 {
        // first code at or above the target, and the first code of the value just below it
        let first = |pred: &dyn Fn(u32) -> bool| {
            let (mut lo, mut hi) = (0u32, 4096u32);
            while lo < hi {
                let m = (lo + hi) / 2;
                if pred(m) {
                    lo = m + 1;
                } else {
                    hi = m;
                }
            }
            lo
        };
        let hi = first(&|c| decode(c) < target);
        if hi == 0 {
            return 0;
        }
        let below = decode(hi - 1);
        let lo = first(&|c| decode(c) < below);
        if hi > 4095 || target - below <= decode(hi) - target {
            lo
        } else {
            hi
        }
    }

    #[cfg(test)]
    pub(crate) fn invert_linear(target: f64, decode: impl Fn(u32) -> f64) -> u32 {
        (0..=4095u32)
            .min_by(|&a, &b| (decode(a) - target).abs().partial_cmp(&(decode(b) - target).abs()).unwrap())
            .unwrap()
    }

    #[cfg(test)]
    pub(crate) fn invert_for_test(target: f64, decode: impl Fn(u32) -> f64) -> u32 {
        invert(target, decode)
    }

    /// Pin voltage for a supply rail of `volts` measured through a channel with firmware gain `gain`.
    pub fn supply_pin_v(volts: f32, gain: u32) -> f32 {
        code_to_pin_v(invert(volts as f64 * 1000.0, |c| fw_supply_mv(c, gain) as f64))
    }

    /// G270 decode at 0x800EDBF0..0x800EDC4A: `sum = [0xD000B1A0] + ADC[17]` (the term is the
    /// pressure offset of the ADC calibration block 0xD000B170, initialised to 0 at 0x800ED706
    /// and not written elsewhere); valid when 0xA3 <= sum <= 0xF5B; p = (((sum * 0x87DE -
    /// 0xCCF330) >> 13) + 1) >> 1 in 0.01 bar. I.e. `p[bar] = 17.39 * V_pin - 8.20` (0.47..4.50 V
    /// <-> 0..70 bar, ratiometric 5 V sensor on the 5 V reference).
    pub fn fw_pressure_centibar(code: u32) -> i32 {
        ((((code as i64 * 0x87DE - 0xCC_F330) >> 13) + 1) >> 1) as i32
    }

    pub fn pressure_pin_v(bar: f32) -> f32 {
        code_to_pin_v(invert(bar as f64 * 100.0, |c| fw_pressure_centibar(c) as f64))
    }

    /// Fork sensor ratiometric normalisation 0x800E66B6: `norm = min(0xE08 * ADC[x] / ADC[ref],
    /// 0xFFF)`, mV = norm * 5000 / 4095, valid 300..4700 mV. Pairs (A = 0x8013C9B9, B =
    /// 0x8013C9BD, `reference = 0x8013C9C1[pair & 1]`): fork 0 = `ADC[26]`/`ADC[25]` ref `ADC[1]`,
    /// fork 1 = `ADC[8]`/`ADC[6]` ref `ADC[19]`, fork 2 = `ADC[10]`/`ADC[9]` ref `ADC[1]`,
    /// fork 3 = `ADC[15]`/`ADC[14]` ref `ADC[19]`. Returns the pin voltage that yields `mv` after
    /// normalisation against a reference converted to `ref_code`.
    pub fn fork_pin_v(mv: f32, ref_code: u32) -> f32 {
        fork_pin_v_norm((mv as f64 * 4095.0 / 5000.0).round(), ref_code)
    }

    /// Pin voltage whose normalised code (`min(0xE08 * ADC / ADC_ref, 0xFFF)`) is closest to `norm`.
    pub fn fork_pin_v_norm(norm: f64, ref_code: u32) -> f32 {
        let code = invert(norm, |c| ((0xE08u64 * c as u64) / ref_code.max(1) as u64).min(0xFFF) as f64);
        code_to_pin_v(code)
    }
}

/// Logical ADC buffer index (`0xD000B0A4[i]`, filled by 0x800ED520/0x800ED578) -> (kernel, channel):
/// kernel 0 scans channels 0..15 into `ADC[0..15]`, kernel 1 into `ADC[16..31]` (group tables
/// 0x8013B218/0x8013B229 list channels 0..15 in order, CHCTRn.RESRSEL = n, INPCR0.DW = 12 bit).
pub const fn adc_index(i: usize) -> (u8, u8) {
    ((i / 16) as u8, (i % 16) as u8)
}

/// Fault-event tables of the CAL block: VAG fault number per event (u16 x [`DTC_EVENTS`]) followed
/// directly by the SAE code per event (same count).
pub const DTC_VAG_TABLE: u32 = 0x8003_2AD0;
pub const DTC_SAE_TABLE: u32 = DTC_VAG_TABLE + 2 * DTC_EVENTS as u32;
pub const DTC_EVENTS: usize = 350;

/// Analog rails the safety software checks while KL30 (`ADC[20]`) >= 0x595: (ADC index, pin volts).
/// Evidence (VERIFIED-S, check table 0x80046058.. used by 0x8004239C): `ADC[21]` window 1732..2387,
/// substitute 0x80B (0x8004512C); `ADC[12]` window 1704..2254, substitute 0x7BB (0x8004519E); `ADC[16]`
/// window 2540..2930, substitute 0xAAF (0x80045210). The pin voltages are the firmware's substitute
/// (nominal) codes; the rails' identity (references / 3.3 V supply) is UNVERIFIED. `ADC[16]` is not the
/// pump supply claimed by the old checkpoint doc (its window is constant, not battery-proportional).
pub const SAFETY_RAILS: [(usize, f32); 3] = [(21, 2.514), (12, 2.416), (16, 3.339)];

/// Supply measurement gains of the firmware table 0x8013EC4C.
const GAIN_SUPPLY: u32 = 0x805;
const GAIN_SENSOR_REF: u32 = 0x247;

impl Dq200TcuBoard {
    /// CPU backend from `SIMEMU_CPU` ([`crate::CpuBackend::from_env`]).
    pub fn new(flash_bytes: &[u8], eeprom_bytes: &[u8]) -> Result<Self, String> {
        Self::new_on(flash_bytes, eeprom_bytes, crate::CpuBackend::from_env())
    }

    pub fn new_on(flash_bytes: &[u8], eeprom_bytes: &[u8], backend: crate::CpuBackend) -> Result<Self, String> {
        let info = Dq200TcuImageInfo::from_bytes(flash_bytes, eeprom_bytes.len())?;
        let mut padded_flash = vec![0xFFu8; 0x400000];
        let n = flash_bytes.len().min(0x400000);
        padded_flash[..n].copy_from_slice(&flash_bytes[..n]);
        let cpu = Simos18Board::new_variant_on(&padded_flash, eeprom_bytes, BoardVariant::Dq200Tc1784, backend)?;
        let mut board = Self { cpu, info, forks: ForkSensors::from_flash(flash_bytes) };
        board.cpu.hw_mut().tc1784.pump.load = crate::tc1784_io::pump::PumpLoad {
            torque_per_pa: PUMP_TORQUE_PER_PA,
            p_out_pa: 0.0,
            visc_nm_s: PUMP_VISC_NM_S,
        };
        board.inject_mechatronic_sensors(13.8, 0.0, 25.0);
        Ok(board)
    }

    pub fn classify_region(pc: u32) -> &'static str {
        match pc {
            0x8000_0000..=0x8000_FFFF | 0xA000_0000..=0xA000_FFFF => "DQ200_SBOOT",
            0x8001_0000..=0x8001_FFFF | 0xA001_0000..=0xA001_FFFF => "DQ200_CBOOT",
            0x8002_0000..=0x8003_FFFF | 0xA002_0000..=0xA003_FFFF => "DQ200_CAL",
            0x8004_0000..=0x8013_FFFF | 0xA004_0000..=0xA013_FFFF => "DQ200_ASW",
            0xC000_0000..=0xC00F_FFFF => "DQ200_PSPR",
            0xD000_0000..=0xD40F_FFFF => "DQ200_DSPR",
            _ => "DQ200_OTHER",
        }
    }

    /// Drive ADC buffer index `i` (see [`adc_index`]) with a pin voltage.
    pub fn set_adc_pin(&mut self, i: usize, volts: f32) {
        if i >= 32 || !volts.is_finite() {
            return;
        }
        let (k, ch) = adc_index(i);
        self.cpu.hw_mut().tc1784.adc_volts[k as usize][ch as usize] = volts;
    }

    /// Speed sensor `sensor` (index of [`SPEED_SENSOR_LINES`]) at `rpm` of its target wheel: one
    /// high pulse per tooth, 45 us wide for rpm >= 0 and 90 us for rpm < 0 (direction protocol).
    /// Below one pulse per 0.5 s the sensor is treated as standing still (no edges).
    pub fn set_shaft_speed(&mut self, sensor: usize, rpm: f64) {
        let (Some(&teeth), Some(&(a, b))) = (SPEED_SENSOR_TEETH.get(sensor), SPEED_SENSOR_LINES.get(sensor)) else {
            return;
        };
        let f = if rpm.is_finite() { rpm.abs() / 60.0 * teeth as f64 } else { 0.0 };
        let w = SPEED_PULSE_S[(rpm < 0.0) as usize];
        let sig = if f >= 2.0 {
            let p = 1.0 / f;
            crate::tc1784_io::PinSignal { period_s: p, duty: (w / p).min(0.5), idle: false }
        } else {
            crate::tc1784_io::PinSignal::default()
        };
        let io = &mut self.cpu.hw_mut().tc1784;
        io.set_pin(a, sig);
        io.set_pin(b, sig);
    }

    /// Clutch travel sensor `sensor` (index of [`CLUTCH_SENSOR_LINES`]) with PWM duty `duty` (0..1)
    /// on its A output and the complementary duty on its B output.
    pub fn set_clutch_sensor_duty(&mut self, sensor: usize, duty: f64) {
        let Some(&(a, b)) = CLUTCH_SENSOR_LINES.get(sensor) else {
            return;
        };
        let duty = if duty.is_finite() { duty.clamp(0.0, 1.0) } else { 0.5 };
        let sig = |d: f64| crate::tc1784_io::PinSignal { period_s: 1.0 / CLUTCH_SENSOR_HZ, duty: d.clamp(0.0, 1.0), idle: false };
        let io = &mut self.cpu.hw_mut().tc1784;
        io.set_pin(a, sig(duty));
        io.set_pin(b, sig(1.0 - duty));
    }

    pub fn inject_mechatronic_sensors(&mut self, battery_v: f32, pressure_bar: f32, temp_c: f32) {
        self.inject_full_mechatronic_plant(battery_v, pressure_bar, temp_c, [0.0; 4], 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /// Apply the physical mechatronic state as electrical inputs of the TC1784.
    ///
    /// Verified channels (firmware decode cited in [`transfer`]): KL30 `ADC[20]`, KL15 `ADC[13]`,
    /// power-stage supplies `ADC[30]`/`ADC[31]` (all `battery_v`), 5 V sensor supplies `ADC[1]`/`ADC[19]`,
    /// G270 pressure `ADC[17]`, fork sensor pairs with their reference supply.
    ///
    /// Speed sensors through GPTA0 captures ([`Self::set_shaft_speed`]): `g182_rpm` -> sensor 2
    /// (60 teeth, no direction), `g501_rpm` -> sensor 0 (18 teeth), `g502_rpm` -> sensor 1 (36
    /// teeth); the G501/G502 order of sensors 0/1 is UNVERIFIED. Clutch travel sensors through
    /// PWM captures ([`Self::set_clutch_sensor_duty`]): K1 -> sensor 0, K2 -> sensor 1 (UNVERIFIED
    /// order) with the firmware's own characteristic [`clutch_travel_duty`] (CAL
    /// [`CLUTCH_SENSOR_CAL`], 29.00 mm over 10 .. 90 % duty).
    ///
    /// Board-side: rails `ADC[21]`/`ADC[12]`/`ADC[16]` at their nominal levels ([`SAFETY_RAILS`]), valve
    /// current senses from the coil loads (`tc1784_io::CoilLoad`, supplied with `battery_v`), the P2.1
    /// readback on `ADC[23]`/`ADC[3]` (`tc1784_io::OUTPUT_READBACKS`).
    ///
    /// Fork travel (`fork_mm`, firmware fork order 1/3, 2/4, 5/7, R/6) through the two Hall outputs per
    /// fork ([`ForkSensors`]); the V401 load pressure = `pressure_bar` (`tc1784_io::pump`).
    /// Not driven (no emulated input path yet): `temp_c` (the temperature sensor input is not
    /// identified; `ADC[12]`/`ADC[21]` are checked as rails, not as NTCs).
    #[allow(clippy::too_many_arguments)]
    pub fn inject_full_mechatronic_plant(
        &mut self,
        battery_v: f32,
        pressure_bar: f32,
        temp_c: f32,
        fork_mm: [f32; 4],
        k1_pos_mm: f32,
        k2_pos_mm: f32,
        g182_rpm: f32,
        g501_rpm: f32,
        g502_rpm: f32,
    ) {
        let _ = temp_c;
        for (sensor, rpm) in [(0, g501_rpm), (1, g502_rpm), (2, g182_rpm)] {
            self.set_shaft_speed(sensor, rpm as f64);
        }
        for (sensor, mm) in [(0, k1_pos_mm), (1, k2_pos_mm)] {
            self.set_clutch_sensor_duty(sensor, clutch_travel_duty(mm as f64));
        }
        let supply = transfer::supply_pin_v(battery_v.max(0.0), GAIN_SUPPLY);
        for i in [13, 20] {
            self.set_adc_pin(i, supply);
        }
        // ADC[30]/ADC[31]: the two power-stage supply senses behind the monitor device's switch
        // (`tc1784_io::MonitorDevice`), applied at conversion time.
        self.cpu.hw_mut().tc1784.valve_supply_pin_v = supply;
        // Board rails checked by the safety software (windows + nominal substitute values, VERIFIED-S,
        // see [`SAFETY_RAILS`]); which rail each channel carries is UNVERIFIED.
        for (i, v) in SAFETY_RAILS {
            self.set_adc_pin(i, v);
        }
        let vref = transfer::supply_pin_v(5.0, GAIN_SENSOR_REF);
        self.set_adc_pin(1, vref);
        self.set_adc_pin(19, vref);
        // ADC[24]: internal 5 V (checkpoint doc: gain 0x235, not in flash table 0x8013EC4C): UNVERIFIED.
        self.set_adc_pin(24, transfer::supply_pin_v(5.0, 0x235));
        self.set_adc_pin(17, transfer::pressure_pin_v(pressure_bar));
        let ref_code = (vref as f64 / crate::tc1784_io::ADC_VAREF * 4095.0).round() as u32;
        for (c, (a, b)) in [(26, 25), (8, 6), (10, 9), (15, 14)].into_iter().enumerate() {
            let mm = fork_mm[c] as f64;
            let va = transfer::fork_pin_v_norm(self.forks.code(c, 0, mm), ref_code);
            let vb = transfer::fork_pin_v_norm(self.forks.code(c, 1, mm), ref_code);
            self.set_adc_pin(a, va);
            self.set_adc_pin(b, vb);
        }
        // Unidentified analog inputs (0x800ED818 checks 300..4700 mV): held mid-range. UNVERIFIED.
        // ADC[23] is the P2.1 readback (`tc1784_io::OUTPUT_READBACKS`).
        for i in [11, 18, 22] {
            self.set_adc_pin(i, 2.5);
        }
        // Valve current-sense pins (0x800ED8C0, table 0x8013CF24) follow the coil loads on the PWM
        // outputs (`tc1784_io::CoilLoad`), supplied from the battery through the power-stage switch
        // of the monitor device (`tc1784_io::Tc1784Io::valve_supply_v`).
        self.cpu.hw_mut().tc1784.coil_supply_v = battery_v.max(0.0) as f64;
        // FADC inputs (`tc1784_io::FADC_RAIL_CH1_V`): channel 1 = board rail at its nominal code
        // (UNVERIFIED identity); channel 0 = pump current sense (`tc1784_io::pump::BldcPump::sense_v`).
        self.cpu.hw_mut().tc1784.fadc.volts[1][0] = crate::tc1784_io::FADC_RAIL_CH1_V;
        // V401 gear pump: outlet = accumulator pressure (G270 measures the same node, check valve
        // open while the pump delivers). FADC channel 0 follows the pump's current sense
        // (`tc1784_io::pump::BldcPump::sense_v`).
        self.cpu.hw_mut().tc1784.pump.load.p_out_pa = pressure_bar.max(0.0) as f64 * 1e5;
    }

    /// Valve coil resistance (per Pwm channel, [`crate::tc1784_io::VALVE_PWM_CELLS`] order) and
    /// inductance of the attached hydraulic unit (from the plant's coil temperatures).
    pub fn set_coil_params(&mut self, r_ohm: [f64; 8], l_h: f64) {
        for (c, r) in self.cpu.hw_mut().tc1784.coils.iter_mut().zip(r_ohm) {
            c.r_ohm = r;
            c.l_h = l_h;
        }
    }

    /// V401 pump hardware on the board (driver chip, motor, shaft): state and UNVERIFIED constants.
    pub fn pump(&self) -> &crate::tc1784_io::pump::BldcPump {
        &self.cpu.hw().tc1784.pump
    }

    pub fn pump_mut(&mut self) -> &mut crate::tc1784_io::pump::BldcPump {
        &mut self.cpu.hw_mut().tc1784.pump
    }

    /// Coil currents (A) per Pwm channel, as sensed at the last ADC conversion.
    pub fn coil_currents(&self) -> [f64; 8] {
        self.cpu.hw().tc1784.coils.map(|c| c.i_a)
    }

    /// Execute `dt_sec` of TCU time (dt * fCPU instructions; the clock tree comes from the
    /// firmware's own PLL/CCUCON0/STM_CLC writes: 120 MHz CPU, 60 MHz FPI, 30 MHz STM).
    pub fn step_realtime_frame(&mut self, dt_sec: f32) -> Result<RunStats, String> {
        self.cpu.step_realtime_frame(dt_sec)
    }

    /// SAE code of a UDS DTC (`19 02` record bytes 0..3, e.g. 0x0029FE) from the firmware's own event
    /// tables: the 350 VAG fault numbers at [`DTC_VAG_TABLE`] and the parallel SAE codes at
    /// [`DTC_SAE_TABLE`] (u16 each; 0xCxxx = Uxxxx network codes). Both tables end at the same index
    /// and are followed by other data (VERIFIED-S: 0x0029FE -> P0715 input speed sensor A, 0x002A0B ->
    /// U0103 lost communication with the gear shift module).
    pub fn dtc_sae_code(&self, dtc: u32) -> Option<String> {
        let t = self.cpu.read_mem(DTC_VAG_TABLE, 4 * DTC_EVENTS).ok()?;
        let i = (0..DTC_EVENTS).find(|&i| u16::from_le_bytes([t[2 * i], t[2 * i + 1]]) as u32 == dtc & 0xFFFF)?;
        let s = u16::from_le_bytes([t[2 * (DTC_EVENTS + i)], t[2 * (DTC_EVENTS + i) + 1]]);
        let letter = ["P", "C", "B", "U"][(s >> 14) as usize];
        Some(format!("{letter}{:04X}", s & 0x3FFF))
    }

    /// `19 02 FF` (reportDTCByStatusMask): (DTC, status, SAE code) per stored record.
    pub fn read_dtcs(&mut self) -> Result<Vec<(u32, u8, String)>, String> {
        let r = self.uds_request(&[0x19, 0x02, 0xFF], 0.5)?.ok_or("no 19 02 answer")?;
        if r.len() < 3 || r[0] != 0x59 {
            return Err(format!("19 02 -> {r:02X?}"));
        }
        Ok(r[3..]
            .as_chunks::<4>()
            .0
            .iter()
            .map(|d| {
                let dtc = u32::from_be_bytes([0, d[0], d[1], d[2]]);
                (dtc, d[3], self.dtc_sae_code(dtc).unwrap_or_else(|| "?".into()))
            })
            .collect())
    }

    /// Diagnostic register-access trace of TC1784 peripherals: accesses inside `range` are logged as
    /// (block pc, address, value, lane mask; mask 0 = read), see [`Self::io_trace`].
    pub fn set_io_trace(&mut self, range: Option<(u32, u32)>) {
        self.cpu.hw_mut().tc1784.trace_range = range;
    }

    pub fn io_trace(&self) -> &[(u32, u32, u32, u32)] {
        &self.cpu.hw().tc1784.trace_log
    }

    /// Run `max_quanta * quantum_insns` instructions (legacy entry point).
    pub fn run_boot_trace(&mut self, max_quanta: usize, quantum_insns: usize) -> Result<u32, String> {
        self.cpu.run_instructions((max_quanta * quantum_insns) as u64).map(|s| s.pc)
    }

    /// Observe firmware variables (read-only).
    pub fn read_live_state(&self) -> Dq200LiveState {
        let d = &self.cpu.hw().dspr;
        let u32_at = |o: usize| u32::from_le_bytes([d[o], d[o + 1], d[o + 2], d[o + 3]]);
        Dq200LiveState {
            istgang: d[0x7D42] as i8,
            zielgang: d[0x7D40] as i8,
            waehlhebel: d[0x77BE],
            notlauf: d[0x77BA],
            main_state: d[0x7EC0],
            pressure_bar: i16::from_le_bytes([d[0xB2B8], d[0xB2B9]]) as f32 * 0.01,
            pressure_status: d[0xB2BE],
            power_stage_word: u32_at(0x3494),
            wfs5_state: d[0x0BA4] >> 4,
            wfs5_authorized: d[0x0BA4] >> 4 == 7,
            safety_word: u32_at(0x1FF58),
            safety_errors: u32_at(0x1FF7C),
            system_pressure_bar: i16::from_le_bytes([d[0x7E30], d[0x7E31]]) as f32 * 0.01,
            pressure_ok: d[0x4E17],
            engine_running: d[0x5113],
        }
    }

    /// Actuator commands measured on the emulated output hardware (GPTA PWM output cells).
    pub fn decode_outputs(&self) -> Dq200Outputs {
        let io = &self.cpu.hw().tc1784;
        let now = io.gpta.now();
        let f = io.gpta.f_gpta_hz.max(1) as f64;
        // A PWM output exists while its chain's reset timer (cell - 3 / pump: LTC26) is configured;
        // without recent edges its duty is the static output level.
        let cell = |k: u8, c: u8, timer: u8| {
            (io.gpta.ltc_ctr(k as usize, timer as usize) & 3 == 3).then(|| match io.cell_pwm(k, true, c) {
                Some(m) => (m.duty(now), m.period as f64 / f),
                None => (io.gpta.ltc_out(k as usize, c as usize) as u8 as f64, 0.0),
            })
        };
        let mut pwm: Vec<(String, f64, f64)> = Vec::new();
        let mut solenoid_duty = [None; 12];
        // A valve sees its PWM only while the output pins drive (the safety shut-off turns them into
        // inputs: the low-side switch is then off = de-energised valve).
        let driven = self.cpu.hw().valve_pins_driven();
        for (i, &c) in SOLENOID_PWM_CELLS.iter().enumerate() {
            let m = cell(0, c, c - 3);
            solenoid_duty[i] = m.filter(|_| driven).map(|(d, _)| d);
            if let Some((d, p)) = m {
                pwm.push((format!("GPTA0 LTC{c} (Pwm ch {}, solenoid {i}{})", VALVE_PWM_CHANNEL_OF_SOLENOID[i], if driven { "" } else { ", pin not driven" }), d, p));
            }
        }
        let pump = cell(PUMP_PWM_CELL.0, PUMP_PWM_CELL.1, PUMP_PWM_CELL.1 - 4);
        if let Some((d, p)) = pump {
            pwm.push((format!("LTCA2 LTC{} (pump PWM)", PUMP_PWM_CELL.1), d, p));
        }
        Dq200Outputs {
            power_stage_released: self.read_live_state().power_stage_word == 0x2152_2152,
            spi_frames: crate::tc1784_io::SPI_DEVICES
                .iter()
                .zip(io.spi_log.iter())
                .skip(1)
                .map(|(d, l)| (d.name.to_string(), l.iter().copied().collect()))
                .collect(),
            pwm,
            solenoid_duty,
            solenoid_current_a: std::array::from_fn(|i| VALVE_PWM_CHANNEL_OF_SOLENOID.get(i).map_or(0.0, |&ch| io.coils[ch].i_mean_a)),
            pump_motor_duty: pump.map(|(d, _)| d),
            pump_shaft_rad_s: Some(io.pump.omega),
            unverified: [
                "valve module/position -> N433..N440 function: from firmware behaviour (HYP), valve characteristics UNVERIFIED",
                "V401: driver chip on CS P3.6 not identified (behaviour from the firmware), motor/pump constants UNVERIFIED",
                "SPI slaves on CS P3.7 / P2.13 (SSC0): not identified, no slave model",
                "monitor device on SSC1/P2.8: register semantics and switch rule derived from firmware checks (UNVERIFIED chip)",
            ]
            .into_iter()
            .map(String::from)
            .collect(),
        }
    }

    /// Send a UDS request to the TCU (ISO-TP on 0x7E1, node 0) and collect the response from
    /// 0x7E9, running the firmware for up to `timeout_s`. Handles single/multi-frame both ways
    /// (flow control with BS = 0, STmin = 0) and 0x78 response-pending.
    pub fn uds_request(&mut self, req: &[u8], timeout_s: f64) -> Result<Option<Vec<u8>>, String> {
        if req.len() > 4095 {
            return Err("ISO-TP payload exceeds 4095 bytes".into());
        }
        let frame = |data: [u8; 8]| CanFrame { node: 0, mo_index: 0, arb_id: 0x7E1, extended: false, dlc: 8, data, timestamp_us: 0 };
        let mut pending_tx: Vec<[u8; 8]> = Vec::new();
        if req.len() <= 7 {
            let mut d = [0u8; 8];
            d[0] = req.len() as u8;
            d[1..1 + req.len()].copy_from_slice(req);
            self.cpu.inject_can_rx(&frame(d));
        } else {
            let mut d = [0u8; 8];
            d[0] = 0x10 | ((req.len() >> 8) as u8 & 0xF);
            d[1] = req.len() as u8;
            d[2..8].copy_from_slice(&req[..6]);
            self.cpu.inject_can_rx(&frame(d));
            for (i, c) in req[6..].chunks(7).enumerate() {
                let mut d = [0u8; 8];
                d[0] = 0x20 | ((i + 1) as u8 & 0xF);
                d[1..1 + c.len()].copy_from_slice(c);
                pending_tx.push(d);
            }
        }
        let mut resp: Vec<u8> = Vec::new();
        let mut expect = 0usize;
        let mut cts = false;
        let step = 0.001;
        let mut t = 0.0;
        while t < timeout_s {
            if cts && !pending_tx.is_empty() {
                let d = pending_tx.remove(0);
                self.cpu.inject_can_rx(&frame(d));
            }
            self.cpu.run_for(step)?;
            t += step;
            for f in self.cpu.take_can_tx() {
                if f.arb_id != 0x7E9 {
                    self.cpu.hw_mut().can_tx_queue.push(f);
                    continue;
                }
                match f.data[0] >> 4 {
                    0 => {
                        let n = (f.data[0] & 0xF) as usize;
                        let r = f.data[1..1 + n.min(7)].to_vec();
                        if r.len() >= 3 && r[0] == 0x7F && r[2] == 0x78 {
                            continue; // response pending
                        }
                        return Ok(Some(r));
                    }
                    1 => {
                        expect = ((f.data[0] as usize & 0xF) << 8) | f.data[1] as usize;
                        resp = f.data[2..8].to_vec();
                        self.cpu.inject_can_rx(&frame([0x30, 0, 0, 0, 0, 0, 0, 0]));
                    }
                    2 => {
                        resp.extend_from_slice(&f.data[1..8]);
                        if resp.len() >= expect {
                            resp.truncate(expect);
                            return Ok(Some(resp));
                        }
                    }
                    3 => cts = f.data[0] & 0xF == 0,
                    _ => {}
                }
            }
        }
        Ok(None)
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use unicorn_tricore_sys::TriCoreReg;

    #[test]
    fn test_wfs5_immo_reference_vector_from_fpke_doc() {
        // Test vector from `fpke-immo-wfs5.md` §10.2:
        // A = "LAB IMMO SLOT A!", B = "LAB IMMO SLOT B!", cfg = 0x4C, ID_ac = "LA", ID_ecu = "L0"
        let slot_a = *b"LAB IMMO SLOT A!";
        let slot_b = *b"LAB IMMO SLOT B!";
        let partner = Wfs5ImmoPartner::from_mqb_slots_ecu(&slot_a, &slot_b, 0x4C, *b"LA", *b"L0", "LABEMU00000000001");
        assert_eq!(&partner.key, b"LAB IMMO SLOT A!");
        let rx_010: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x9D, 0xFA, 0x63, 0x74];
        assert!(partner.verify_challenge(&rx_010));
        assert_eq!(partner.answer_challenge(&rx_010), [0xE7, 0x7F, 0x80, 0xA4, 0x00, 0x00, 0x00, 0x1F]);
    }

    /// Binary-search inversion = exhaustive minimum search for every decode the board uses.
    #[test]
    fn transfer_inversion_matches_linear_search() {
        use super::transfer::{fw_pressure_centibar, fw_supply_mv, invert_for_test, invert_linear};
        let fork = |r: u64| move |c: u32| ((0xE08u64 * c as u64) / r).min(0xFFF) as f64;
        for t in (-200..9000).map(|x| x as f64 * 2.37) {
            for g in [0x805u32, 0x247, 0x235] {
                let d = |c| fw_supply_mv(c, g) as f64;
                assert_eq!(invert_for_test(t, d), invert_linear(t, d), "supply {g:#x} {t}");
            }
            let p = |c| fw_pressure_centibar(c) as f64;
            assert_eq!(invert_for_test(t, p), invert_linear(t, p), "pressure {t}");
            for r in [1u64, 2900, 4095] {
                assert_eq!(invert_for_test(t, fork(r)), invert_linear(t, fork(r)), "fork {r} {t}");
            }
        }
    }

    #[test]
    fn test_wfs5_dq200_eeprom_credentials() {
        let empty = Wfs5ImmoPartner::from_dq200_eeprom(&[]);
        assert_eq!(empty.power_class, 0);
        assert_eq!(empty.answer_challenge(&[0xAA, 0x55, 0x12, 0x34, 0x00, 0x00, 0x00, 0x00])[7], 0x1F);
        if let Some((f, e)) = load() {
            let p = Wfs5ImmoPartner::from_dq200_images(&f, &e);
            assert_eq!(p.power_class, 0x5C);
            assert_eq!(p.id_kombi, [0xD3, 0x2E]);
            let ans = p.answer_challenge(&[0xAA, 0x55, 0x12, 0x34, 0x00, 0x00, 0x00, 0x00]);
            assert_eq!(ans[7], 0x1F);
        }
    }

    #[test]
    fn transfer_functions_invert_firmware_decode() {
        use transfer::*;
        let code = |v: f32| (v as f64 / crate::tc1784_io::ADC_VAREF * 4095.0).round() as u32;
        // KL30 13.85 V -> firmware mV within one LSB (4.89 mV)
        let mv = fw_supply_mv(code(supply_pin_v(13.85, 0x805)), 0x805) as i32;
        assert!((mv - 13850).abs() <= 3, "{mv}");
        // the raw start/stop/power thresholds of 0x800EB970/0x800EC1F4 are 6.2/6.0/6.5 V
        assert_eq!(fw_supply_mv(0x4F5, 0x805), 6211);
        assert_eq!(fw_supply_mv(0x52F, 0x805), 6495);
        assert!((divider(0x805) - 4.0086).abs() < 1e-3);
        // G270 pressure: decode(encode(p)) within one LSB (2.1 cbar)
        for bar in [0.5f32, 20.0, 44.75, 52.0, 65.0] {
            let p = fw_pressure_centibar(code(pressure_pin_v(bar)));
            assert!((p - (bar * 100.0) as i32).abs() <= 2, "{bar} -> {p}");
        }
        // fork sensor normalisation against a 5.0 V reference
        let r = code(supply_pin_v(5.0, 0x247));
        let n = (0xE08 * code(fork_pin_v(2500.0, r)) / r).min(0xFFF);
        assert!(((n * 5000 / 4095) as i32 - 2500).abs() <= 2);
    }

    pub(super) fn load() -> Option<(Vec<u8>, Vec<u8>)> {
        let ws = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let bases: Vec<std::path::PathBuf> = std::env::var_os("SIMEMU_IMAGE_DIR")
            .into_iter()
            .flat_map(|v| std::env::split_paths(&v).collect::<Vec<_>>())
            .chain(std::iter::once(ws))
            .collect();
        for b in &bases {
            for sub in ["", "vault/firmwares/tcu/dq200", "firmwares/tcu/dq200"] {
                let d = if sub.is_empty() { b.clone() } else { b.join(sub) };
                let Ok(rd) = std::fs::read_dir(&d) else { continue };
                let mut flash_opt = None;
                let mut eep_opt = None;
                for entry in rd.flatten() {
                    let p = entry.path();
                    let Ok(m) = std::fs::metadata(&p) else { continue };
                    if m.is_file() && m.len() == 0x2A_0000 && flash_opt.is_none() {
                        flash_opt = std::fs::read(&p).ok();
                    } else if m.is_file() && m.len() == 0x2000 && eep_opt.is_none() {
                        eep_opt = std::fs::read(&p).ok();
                    }
                }
                if let (Some(f), Some(e)) = (flash_opt, eep_opt) {
                    return Some((f, e));
                }
            }
        }
        None
    }

    /// Fork sensors: the board's output codes, decoded through the firmware's own 20-point tables
    /// (linear interpolation as `FUN_d4000ae0`), give back the fork travel inside each output's
    /// unsaturated span (A: -14..+4 mm, B: -4..+14 mm).
    #[test]
    fn fork_sensor_codes_invert_the_firmware_characteristic() {
        let Some((f, _)) = load() else { return };
        let fs = ForkSensors::from_flash(&f);
        let decode = |t: &[(i16, i16); 20], x: f64| {
            let w = t.windows(2).find(|w| x <= w[1].0 as f64).unwrap_or(&t[18..20]);
            let ((x0, y0), (x1, y1)) = ((w[0].0 as f64, w[0].1 as f64), (w[1].0 as f64, w[1].1 as f64));
            (y0 + (y1 - y0) * (x - x0) / (x1 - x0)) / 100.0
        };
        for c in 0..4 {
            for (k, (lo, hi)) in [(-14.0, 4.0), (-4.0, 14.0)].into_iter().enumerate() {
                let mut mm = lo;
                while mm <= hi {
                    let back = decode(&fs.table[c][k], fs.code(c, k, mm));
                    assert!((back - mm).abs() < 1e-6, "fork {c} output {k}: {mm} mm -> {back}");
                    mm += 0.37;
                }
            }
        }
    }

    /// The two Hall outputs of every fork pass the firmware's pair check (`0x800CF4FA`: `lo(A) < B <
    /// hi(A)`, band CAL 0x80038CA8) over the whole travel incl. the end stops, which a saturated output
    /// does not (P2832.. at > 8.5 mm).
    #[test]
    fn fork_sensor_pairs_pass_the_firmware_plausibility_band() {
        let Some((f, _)) = load() else { return };
        let fs = ForkSensors::from_flash(&f);
        let rd = |a: u32| i16::from_le_bytes([f[(a - 0x8000_0000) as usize], f[(a - 0x8000_0000) as usize + 1]]) as f64;
        let band = |o: u32| -> [(f64, f64); 20] { std::array::from_fn(|i| (rd(0x8003_8CA8 + 2 * i as u32), rd(0x8003_8CA8 + o + 2 * i as u32))) };
        // 0xD4000AE0: clamped at both ends, linear in between.
        let lerp = |t: &[(f64, f64); 20], x: f64| match t.windows(2).find(|w| x < w[1].0) {
            _ if x <= t[0].0 => t[0].1,
            Some(w) => w[0].1 + (w[1].1 - w[0].1) * (x - w[0].0) / (w[1].0 - w[0].0),
            None => t[19].1,
        };
        let (lo, hi) = (band(0x28), band(0x50));
        for c in 0..4 {
            for i in -130..=130 {
                let mm = i as f64 / 10.0;
                let (a, b) = (fs.code(c, 0, mm), fs.code(c, 1, mm));
                assert!(lerp(&lo, a) < b && b < lerp(&hi, a), "fork {c} at {mm} mm: A {a:.0} B {b:.0} outside [{:.0}, {:.0}]", lerp(&lo, a), lerp(&hi, a));
            }
        }
    }

    /// Clutch travel sensors: the board's duties, decoded through the firmware's own two-point lines
    /// (CAL [`CLUTCH_SENSOR_CAL`], both channels averaged as 0x80136736 does) give back the travel.
    #[test]
    fn clutch_sensor_duty_inverts_the_firmware_characteristic() {
        let Some((f, _)) = load() else { return };
        let o = (CLUTCH_SENSOR_CAL - 0x8000_0000) as usize;
        let h = |i: usize| i16::from_le_bytes([f[o + 2 * i], f[o + 2 * i + 1]]) as f64;
        for clutch in 0..2 {
            let line = |ch: usize, d: f64| {
                let (x0, x1, y0, y1) = (h(2 * ch), h(2 * ch + 1), h(8 + 2 * ch), h(9 + 2 * ch));
                (y0 + (y1 - y0) * (d * 1e4 - x0) / (x1 - x0)) / 100.0
            };
            for mm in [0.0, 1.45, 7.75, 12.3, 24.35, 29.0] {
                let d = clutch_travel_duty(mm);
                let back = 0.5 * (line(2 * clutch, d) + line(2 * clutch + 1, 1.0 - d));
                assert!((back - mm).abs() < 1e-9, "clutch {clutch}: {mm} mm -> {back}");
            }
        }
    }

    /// Honest boot of the unmodified TCU firmware: electrical inputs only, no RAM writes.
    /// Prints milestones, transmitted CAN IDs, firmware state and the UDS 0x19 DTC report.
    #[test]
    fn dq200_honest_boot_report() {
        for backend in crate::CpuBackend::matrix() {
            honest_boot_report_on(backend);
        }
    }

    fn honest_boot_report_on(backend: crate::CpuBackend) {
        let Some((f, e)) = load() else { return };
        let mut tcu = Dq200TcuBoard::new_on(&f, &e, backend).unwrap();
        tcu.inject_full_mechatronic_plant(13.8, 50.0, 40.0, [0.0; 4], 0.0, 0.0, 780.0, 0.0, 0.0);
        let mut ids = std::collections::BTreeMap::<u32, u64>::new();
        let (mut insns, mut sim, mut wall, mut irqs) = (0u64, 0f64, 0f64, 0u64);
        let immo = Wfs5ImmoPartner::from_dq200_images(&f, &e);
        // No 0x3C0 (Klemmen_Status_01): the firmware configures no receive MO for it (MO table
        // dumped by the `dq200_probe` diagnostic with DQ_MO=1); KL15 is the ADC[13] pin.
        for _ in 0..100u32 {
            let s = tcu.step_realtime_frame(0.010).unwrap();
            insns += s.instructions;
            sim += s.sim_time_s;
            wall += s.wall_time_s;
            irqs += s.interrupts;
            for fr in tcu.cpu.take_can_tx() {
                *ids.entry(fr.arb_id | (fr.node as u32) << 30).or_default() += 1;
                if fr.arb_id == 0x012 {
                    // virtual instrument cluster (WFS5 master) answering with the TCU's own key
                    let a = CanFrame { node: 0, mo_index: 0, arb_id: 0x013, extended: false, dlc: 8, data: immo.answer_challenge(&fr.data), timestamp_us: 0 };
                    tcu.cpu.inject_can_rx(&a);
                }
            }
        }
        let pc = tcu.cpu.read_reg(TriCoreReg::PC);
        let hw = tcu.cpu.hw();
        println!("== DQ200 honest boot [{backend:?}]: {sim:.3} s TCU time, {insns} insns, {irqs} irqs, {wall:.2} s wall, PC=0x{pc:08X} ({})",
            Dq200TcuBoard::classify_region(pc));
        println!("fCPU {} Hz, resets {:?}, traps {}", hw.scu.fcpu_hz(), hw.reset_log, hw.trap_count);
        for m in &hw.milestones {
            println!("milestone {:?}", m);
        }
        let io = &hw.tc1784;
        println!(
            "SSC0 frames {}, EEPROM writes {}, DMA moves {:?} bus errors {}, ADC scans {:?}",
            io.ssc[0].frames,
            io.eeprom.writes_committed,
            io.dma.ch.iter().map(|c| c.moves).collect::<Vec<_>>(),
            io.dma.bus_errors,
            io.adc.scans
        );
        println!("CAN TX (node:id x count): {}", ids.iter().map(|(k, v)| format!("{}:0x{:03X}x{v}", k >> 30, k & 0x3FFF_FFFF)).collect::<Vec<_>>().join(" "));
        // Getriebe_11 every 10 ms: the OS keeps scheduling (a lost ICR.IE froze it after one frame).
        assert!(ids.get(&0x0AD).copied().unwrap_or(0) >= 80, "0x0AD not periodic: OS stalled");
        println!("CAN NCR {:X?} rx_dropped {}", hw.can.ncr, hw.can.rx_dropped);
        println!("live: {:?}", tcu.read_live_state());
        println!("outputs: {:?}", tcu.decode_outputs());
        println!("UDS 10 03 -> {:02X?}", tcu.uds_request(&[0x10, 0x03], 0.3).unwrap());
        match tcu.uds_request(&[0x19, 0x02, 0xFF], 0.5).unwrap() {
            Some(r) => {
                println!("UDS 19 02 FF -> {:02X?}", r);
                if r.len() >= 3 && r[0] == 0x59 {
                    for d in r[3..].chunks(4).filter(|c| c.len() == 4) {
                        println!("  DTC {:02X}{:02X}{:02X} status 0x{:02X}", d[0], d[1], d[2], d[3]);
                    }
                }
            }
            None => println!("UDS 19 02 FF -> no response"),
        }
    }

    #[test]
    fn blank_forks_oob_setters_and_oversized_uds_do_not_panic() {
        let blank = ForkSensors::from_flash(&[]);
        assert_eq!(blank.code(0, 0, 5.0), 0.0, "blank fork table must not divide 0/0 -> NaN");
        assert_eq!(blank.code(99, 99, f64::NAN), 0.0, "OOB fork/output indices must not panic");
        assert_eq!(clutch_travel_duty(f64::NAN), 0.10, "NaN clutch travel must sanitize cleanly");
    }
}

#[cfg(test)]
mod probe {
    use super::*;
    use unicorn_tricore_sys::TriCoreReg;

    /// `DQ_PIN=line:hz:duty,...` drives GPTA input lines.
    fn drive_pins(tcu: &mut Dq200TcuBoard) {
        for p in std::env::var("DQ_PIN").unwrap_or_default().split(',').filter(|s| !s.is_empty()) {
            let v: Vec<f64> = p.split(':').map(|x| x.parse().unwrap()).collect();
            tcu.cpu.hw_mut().tc1784.set_pin(v[0] as u8, crate::tc1784_io::PinSignal { period_s: 1.0 / v[1], duty: v[2], idle: false });
        }
    }

    /// Diagnostic: register-write trace (`DQ_TRACE=lo,hi`, `DQ_SECS`), first write per (pc, addr, value).
    #[test]
    #[ignore = "diagnostic"]
    fn dq200_write_trace() {
        let (f, e) = super::tests::load().expect("DQ200 dumps in vault/firmwares/tcu/dq200/");
        let mut tcu = Dq200TcuBoard::new(&f, &e).unwrap();
        let r: Vec<u32> = std::env::var("DQ_TRACE").unwrap_or("F0001800,F0002FFF".into()).split(',')
            .map(|a| u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap()).collect();
        tcu.cpu.hw_mut().tc1784.trace_range = Some((r[0], r[1]));
        drive_pins(&mut tcu);
        let secs: f64 = std::env::var("DQ_SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(0.3);
        tcu.cpu.run_for(secs).unwrap();
        let mut seen = std::collections::HashSet::new();
        let all = std::env::var("DQ_ALL").is_ok();
        for &(pc, a, v, m) in &tcu.cpu.hw().tc1784.trace_log {
            if seen.insert((pc, a, v & m)) || all {
                println!("pc {pc:08X} [{a:08X}] <- {:08X} (m {m:08X})", v);
            }
        }
        println!("total writes {}", tcu.cpu.hw().tc1784.trace_log.len());
    }

    /// Diagnostic: print RAM bytes `DQ_WATCH=addr,...` whenever one changes (1 ms sampling), with
    /// the plant stimulus of the honest boot test and `DQ_SECS` of runtime.
    #[test]
    #[ignore = "diagnostic"]
    fn dq200_watch() {
        let (f, e) = super::tests::load().expect("DQ200 dumps in vault/firmwares/tcu/dq200/");
        let mut tcu = Dq200TcuBoard::new(&f, &e).unwrap();
        // `DQ_PLANT=rpm182,rpm501,rpm502,k1mm,k2mm`
        let pl: Vec<f32> = std::env::var("DQ_PLANT").unwrap_or("780,0,0,0,0".into()).split(',').map(|x| x.parse().unwrap()).collect();
        tcu.inject_full_mechatronic_plant(13.8, 50.0, 40.0, [0.0; 4], pl[3], pl[4], pl[0], pl[1], pl[2]);
        // `addr` = byte, `addr:2` = u16, `addr:4` = u32
        let addrs: Vec<(u32, usize)> = std::env::var("DQ_WATCH").unwrap_or_default().split(',').filter(|s| !s.is_empty())
            .map(|a| {
                let (a, n) = a.split_once(':').unwrap_or((a, "1"));
                (u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap(), n.parse().unwrap())
            }).collect();
        drive_pins(&mut tcu);
        if std::env::var("DQ_NOMON").is_ok() {
            tcu.cpu.hw_mut().tc1784.monitor.attached = false;
        }
        let probes: Vec<u32> = std::env::var("DQ_PROBES").unwrap_or_default().split(',').filter(|s| !s.is_empty())
            .map(|a| u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap()).collect();
        for &p in &probes {
            tcu.cpu.add_probe(p).unwrap();
        }
        let secs: f64 = std::env::var("DQ_SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(1.0);
        let step_ms: u32 = std::env::var("DQ_STEP_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(1);
        let mut last: Vec<u32> = vec![u32::MAX; addrs.len()];
        for i in 0..(secs * 1000.0) as u32 / step_ms {
            tcu.cpu.run_for(0.001 * step_ms as f64).unwrap();
            let d = &tcu.cpu.hw().dspr;
            let cur: Vec<u32> = addrs.iter().map(|&(a, n)| {
                let o = (a & 0xF_FFFF) as usize;
                (0..n).fold(0u32, |acc, k| acc | (d[o + k] as u32) << (8 * k))
            }).collect();
            if cur != last {
                println!("{:5} ms {}", (i + 1) * step_ms, addrs.iter().zip(&cur).map(|((a, _), v)| format!("{a:X}={v:X}")).collect::<Vec<_>>().join(" "));
                last = cur;
            }
            for fr in tcu.cpu.take_can_tx() {
                if fr.arb_id == 0x012 && std::env::var("DQ_IMMO").is_ok() {
                    let p = Wfs5ImmoPartner::from_dq200_images(&f, &e);
                    let mut d = p.answer_challenge(&fr.data);
                    if std::env::var("DQ_IMMO_BAD").is_ok() {
                        d[0] ^= 1;
                    }
                    let n = tcu.cpu.inject_can_rx(&CanFrame { node: 0, mo_index: 0, arb_id: 0x013, extended: false, dlc: 8, data: d, timestamp_us: 0 });
                    println!("{:5} ms RX 0x013 {:02X?} (accepted by {n} MO)", (i + 1) * step_ms, d);
                }
                if fr.arb_id < 0x20 {
                    println!("{:5} ms TX 0x{:03X} {:02X?}", (i + 1) * step_ms, fr.arb_id, fr.data);
                }
            }
        }
        for &p in &probes {
            let pr = tcu.cpu.probe(p).unwrap();
            println!("probe 0x{p:08X} hits {} D {:X?} A {:X?}", pr.hits, &pr.last_regs[..16], &pr.last_regs[16..]);
        }
        let hw = tcu.cpu.hw();
        if std::env::var("DQ_MONLOG").is_ok() {
            for (i, &(q, r, p)) in hw.tc1784.monitor.log.iter().enumerate() {
                println!("mon {i:4}: {} reg {:2} data {:06X} | reply cnt {} reg {:2} {} data {:06X} | P3.9 {} P3.10 {}", if q & 1 == 1 { "R" } else { "W" }, (q >> 1) & 0x1F, (q >> 6) & 0xFF_FFFF,
                    r >> 30, (r >> 1) & 0x1F, if r & 1 == 1 { "R" } else { "W" }, (r >> 6) & 0xFF_FFFF, p & 1, p >> 1);
            }
        }
        println!("resets {:?} swrst pcs {:X?} monitor frames {} spi {:X?}", hw.reset_log, hw.swrst_log, hw.tc1784.monitor.frames,
            hw.tc1784.spi_log[crate::tc1784_io::SPI_MONITOR].iter().take(40).collect::<Vec<_>>());
        if std::env::var("DQ_DTC").is_ok() {
            if let Some(r) = tcu.uds_request(&[0x19, 0x02, 0xFF], 0.5).unwrap() {
                println!("DTCs: {}", r[3..].chunks(4).map(|d| format!("{:02X}{:02X}{:02X}/{:02X}", d[0], d[1], d[2], d[3])).collect::<Vec<_>>().join(" "));
            }
        }
    }

    static WRITES: std::sync::Mutex<Vec<(u32, u32, i64)>> = std::sync::Mutex::new(Vec::new());

    extern "C" fn hook_w(uc: *mut unicorn_tricore_sys::uc_engine, _t: unicorn_tricore_sys::uc_mem_type, a: u64, _s: std::ffi::c_int, v: i64, _ud: *mut std::ffi::c_void) {
        let mut pc = 0u32;
        // SAFETY: valid engine inside a callback.
        unsafe { unicorn_tricore_sys::uc_reg_read(uc, TriCoreReg::PC as std::ffi::c_int, (&mut pc as *mut u32).cast()) };
        let mut w = WRITES.lock().unwrap_or_else(|e| e.into_inner());
        if w.len() < 100_000 {
            w.push((pc, a as u32, v));
        }
    }

    /// Diagnostic: data-write watchpoint `DQ_WWATCH=lo,hi` (DSPR), prints (pc, addr, value) of the
    /// first distinct writes.
    #[test]
    #[ignore = "diagnostic"]
    fn dq200_write_watch() {
        let (f, e) = super::tests::load().expect("DQ200 dumps in vault/firmwares/tcu/dq200/");
        let mut tcu = Dq200TcuBoard::new(&f, &e).unwrap();
        tcu.inject_full_mechatronic_plant(13.8, 50.0, 40.0, [0.0; 4], 0.0, 0.0, 780.0, 0.0, 0.0);
        let r: Vec<u64> = std::env::var("DQ_WWATCH").unwrap().split(',').map(|a| u64::from_str_radix(a.trim_start_matches("0x"), 16).unwrap()).collect();
        tcu.cpu.add_hook(unicorn_tricore_sys::UC_HOOK_MEM_WRITE, hook_w as *mut std::ffi::c_void, r[0], r[1]).unwrap();
        let secs: f64 = std::env::var("DQ_SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(0.3);
        tcu.cpu.run_for(secs).unwrap();
        let mut seen = std::collections::HashSet::new();
        for &(pc, a, v) in WRITES.lock().unwrap().iter() {
            if seen.insert((pc, a, v)) {
                println!("pc {pc:08X} [{a:08X}] <- {v:X}");
            }
        }
    }

    /// Diagnostic: PC histogram + register dump (`DQ_SECS`, `DQ_REGS=addr,addr`).
    #[test]
    #[ignore = "diagnostic"]
    fn dq200_probe() {
        let (f, e) = super::tests::load().expect("DQ200 dumps in vault/firmwares/tcu/dq200/");
        let mut tcu = Dq200TcuBoard::new(&f, &e).unwrap();
        let probes: Vec<u32> = std::env::var("DQ_PROBES").unwrap_or_default().split(',').filter(|s| !s.is_empty())
            .map(|a| u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap()).collect();
        for &p in &probes {
            tcu.cpu.add_probe(p).unwrap();
        }
        let secs: f64 = std::env::var("DQ_SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(0.5);
        let mut hist = std::collections::BTreeMap::<u32, u32>::new();
        let n = (secs / 0.0001) as usize;
        for _ in 0..n {
            tcu.cpu.run_for(0.0001).unwrap();
            *hist.entry(tcu.cpu.read_reg(TriCoreReg::PC) & !0xF).or_default() += 1;
        }
        let mut v: Vec<_> = hist.into_iter().collect();
        v.sort_by_key(|x| std::cmp::Reverse(x.1));
        for (pc, c) in v.iter().take(25) {
            println!("pc 0x{pc:08X} x{c}");
        }
        let hw = tcu.cpu.hw();
        for a in std::env::var("DQ_REGS").unwrap_or_default().split(',').filter(|s| !s.is_empty()) {
            let a = u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap();
            let v = tcu.cpu.read_mem(a, 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]])).unwrap_or(0xDEAD_BEEF);
            println!("[{a:08X}] = {v:08X}");
        }
        println!("irqs {} ssc {:?} dma {:?} adc {:?}", hw.irq_dispatch_count, hw.tc1784.ssc.iter().map(|s| s.frames).collect::<Vec<_>>(),
            hw.tc1784.dma.ch.iter().map(|c| c.moves).collect::<Vec<_>>(), hw.tc1784.adc.scans);
        println!("src_active {:X?}", hw.src_active);
        println!("EEPROM READ addrs {:X?}", &hw.tc1784.eeprom.read_log[..hw.tc1784.eeprom.read_log.len().min(8)]);
        if let Ok(r) = std::env::var("DQ_LTC") {
            let (k, lo, hi) = {
                let v: Vec<u32> = r.split(',').map(|x| x.parse().unwrap()).collect();
                (v[0], v[1], v[2])
            };
            let base = if k == 2 { 0xF000_2A00 } else { 0xF000_1A00 };
            let g = &mut tcu.cpu.hw_mut().tc1784.gpta;
            for n in lo..=hi {
                println!("k{k} LTC{n:2} CTR {:08X} XR {:04X}", g.read_word(base + 8 * n).unwrap(), g.read_word(base + 8 * n + 4).unwrap());
            }
        }
        let hw = tcu.cpu.hw();
        let now = hw.tc1784.gpta.now();
        let mut cells: Vec<_> = hw.tc1784.cell_out.iter().collect();
        cells.sort_by_key(|c| *c.0);
        for ((k, c), m) in cells {
            println!("GPTA k{k} {}{:2}: edges {} period {} high {} duty {:.4} level {}", if *c >= 64 { "LTC" } else { "GTC" }, c % 64, m.edges, m.period, m.high, m.duty(now), m.level);
        }
        if std::env::var("DQ_MO").is_ok() {
            for n in 0..64usize {
                let m = &hw.can.mo[n];
                let st = hw.can.mostat(n);
                if st & 0x20 != 0 {
                    println!("MO{n:2} AR {:08X} id {:03X} AMR {:08X} FCR {:08X} STAT {st:08X} list {}", m.ar, (m.ar >> 18) & 0x7FF, m.amr, m.fcr, hw.can.mo_list[n]);
                }
            }
        }
        for (d, log) in crate::tc1784_io::SPI_DEVICES.iter().zip(hw.tc1784.spi_log.iter()) {
            println!("{}: {:X?}", d.name, log.iter().take(32).collect::<Vec<_>>());
        }
        for &p in &probes {
            let pr = tcu.cpu.probe(p).unwrap();
            println!("probe 0x{p:08X} hits {} d4={:X} d1={:X} d15={:X} a4={:X} a15={:X} a11={:X}", pr.hits, pr.last_regs[4], pr.last_regs[1], pr.last_regs[15], pr.last_regs[20], pr.last_regs[31], pr.last_regs[27]);
        }
        let mut link = tcu.cpu.read_reg(TriCoreReg::PCXI);
        for _ in 0..10 {
            if link & 0xF_FFFF == 0 { break; }
            let ea = crate::stm_icu::csa_ea(link);
            let w = |o: u32| hw.read_dspr_u32(ea + o);
            println!("  CSA {ea:08X}: PCXI={:08X} PSW={:08X} A11={:08X} UL={}", w(0), w(4), w(0xC), (link >> 22) & 1);
            link = w(0);
        }
        println!("tx {} ICR={:08X} PSW={:08X} PCXI={:08X} PC={:08X}", hw.can_tx_queue.len(), tcu.cpu.read_reg(TriCoreReg::ICR),
            tcu.cpu.read_reg(TriCoreReg::PSW), tcu.cpu.read_reg(TriCoreReg::PCXI), tcu.cpu.read_reg(TriCoreReg::PC));
    }
}

