//! SIMOS18.1 (SC800S50 / 8V0906259J) WFS5 partner-0 credentials, read from the ECU's own dumps.
//!
//! Nothing here is invented: every value comes from the ECU flash / DFlash exactly as the
//! firmware reads it (see `docs/SIMOS18_WFS5_RE.md` for the full trace):
//!
//! * IMMO NVM = ECME channels 6, 7, 8 (three redundant copies, 100 bytes each) in the DFlash
//!   FEE. Each channel is encrypted with `anvmcryptlight` (CBOOT `0x80017b2c`/`0x800176bc`/
//!   `0x8001794c`/`0x80017c64`): a Hitag2-style 48-bit stream cipher whose key is derived from
//!   the first 8 bytes of the TC1791 chip UID (DSPR `0xD0000000` at run time; the factory copy
//!   the ASW compares it with lives at `0x80014200`), IV = 16-bit "EEPROM address" = channel id,
//!   +2 per 16-bit word.
//! * Plain channel: `[0..13)` IMO state record, `[13..66)` IMO data record, `[83..100)` VIN copy.
//!   Data record (built at `0x8011c4cc`, CRC-16/CCITT-FALSE LE at `+0x33`): `[1..17)` = K0,
//!   `[0x11..0x22)` = VIN, `[0x23]` = power class B, `[0x25..0x27)` = ID_kombi.
//! * ID_ecu = majority of three u16 LE at PFlash `0x80001150` (callback `0x8011524c` ->
//!   `0xD001D19E`, used by the TX frame builder `0x80117cc8` via `0x801160d8`).
//! * FEE record trailers carry a CRC over payload, bank sequence, size and id (`FeeRecord`); the lab
//!   binding (`immo_lab.rs`) uses these helpers to write records the way the firmware does.

use simemu_tc1791::Wfs5ImmoPartner;

pub(crate) const UID_OFF: usize = 0x0001_4200; // 0x80014200: chip UID (12 bytes; cipher uses 8)
const ID_ECU_OFF: usize = 0x0000_1150; // 0x80001150: ID_ecu x3 (u16 LE)
pub(crate) const CHANNEL_LEN: usize = 100;
pub(crate) const STATE_LEN: usize = 13;
pub(crate) const DATA_LEN: usize = 53;
/// DFlash bank DF0 (`0xAF000000`, the part of the dump the board maps; bank DF1 is erased).
pub(crate) const DF0_LEN: usize = 0x1_8000;

/// CRC-16/CCITT-FALSE (`0x8011f400`): poly 0x1021, init 0xFFFF, no reflection.
pub(crate) fn crc16_ccitt(data: &[u8]) -> u16 {
    data.iter().fold(0xFFFFu16, |mut c, &b| {
        c ^= (b as u16) << 8;
        for _ in 0..8 {
            c = if c & 0x8000 != 0 { (c << 1) ^ 0x1021 } else { c << 1 };
        }
        c
    })
}

fn crc_ok(rec: &[u8]) -> bool {
    let n = rec.len() - 2;
    crc16_ccitt(&rec[..n]) == u16::from_le_bytes([rec[n], rec[n + 1]])
}

/// `anvmcryptlight` (filter tables `0x8001523c`/`0x8001524c`/`0x8001525c` as bitmasks).
pub(crate) struct AnvmCrypt {
    k: [u8; 6], // 0xB0000B3A: k[0..2] seed the state, k[2..6] ^ m = 32 key bits
    m: [u8; 4], // 0xB0000B4E
}

impl AnvmCrypt {
    const TA: u32 = 0x287B;
    const TB: u32 = 0x267D;
    const TC: u32 = 0x7956_EB79;

    /// Key schedule `0x80017b2c` over UID[0..8].
    pub(crate) fn new(u: &[u8]) -> Self {
        Self {
            k: [
                u[0] ^ u[1] ^ u[4],
                u[1] ^ u[2] ^ u[3],
                u[0] ^ u[2] ^ u[5],
                u[0] ^ u[3] ^ u[4],
                u[1] ^ u[3] ^ u[6],
                u[1] ^ u[4] ^ u[5],
            ],
            m: [
                u[4] ^ u[5] ^ u[6],
                u[2] ^ u[5] ^ u[7],
                u[4] ^ u[6] ^ u[7],
                u[0] ^ u[6] ^ u[7],
            ],
        }
    }

    /// Nonlinear filter `0x80017540`; state = 48 bits, s0 = MSB.
    fn f(s: u64) -> u32 {
        let b = |i: u32| ((s >> (47 - i)) & 1) as u32; // bit i counted from the MSB
        let n4 = |a: u32, c: u32, d: u32, e: u32| (b(a) << 3) | (b(c) << 2) | (b(d) << 1) | b(e);
        let t = |m: u32, i: u32| (m >> i) & 1;
        let i1 = n4(2, 3, 5, 6);
        let i2 = n4(8, 12, 14, 15);
        let i3 = n4(17, 21, 23, 26);
        let i4 = n4(28, 29, 31, 33);
        let i5 = n4(34, 43, 44, 46);
        let idx = (t(Self::TA, i1) << 4)
            | (t(Self::TB, i2) << 3)
            | (t(Self::TB, i3) << 2)
            | (t(Self::TB, i4) << 1)
            | t(Self::TA, i5);
        t(Self::TC, idx)
    }

    /// LFSR feedback `0x80017620` (taps = bit positions from the MSB).
    fn fb(s: u64) -> u64 {
        [0u32, 2, 3, 6, 7, 8, 16, 22, 23, 26, 30, 41, 42, 43, 46, 47]
            .iter()
            .fold(0, |a, &i| a ^ ((s >> (47 - i)) & 1))
    }

    fn shift(s: u64, bit: u64) -> u64 {
        ((s << 1) | bit) & 0xFFFF_FFFF_FFFF
    }

    /// Two keystream bytes for one 16-bit word at `addr` (`0x800176bc` + `0x8001794c`).
    fn ks(&self, addr: u16) -> [u8; 2] {
        let a = addr as u64;
        let mut s = (a << 32) | (a << 16) | ((self.k[0] as u64) << 8) | self.k[1] as u64;
        for j in 0..4 {
            let kb = self.k[2 + j] ^ self.m[j];
            for bit in (0..8).rev() {
                s = Self::shift(s, (Self::f(s) ^ ((kb as u32 >> bit) & 1)) as u64);
            }
        }
        let mut out = [0u8; 2];
        for o in &mut out {
            for bit in (0..8).rev() {
                s = Self::shift(s, Self::fb(s));
                *o |= (Self::f(s) as u8) << bit;
            }
        }
        out
    }

    /// In-place (de|en)cryption of one channel image (slice driver `0x80017c64`).
    pub(crate) fn apply(&self, channel: u16, data: &mut [u8]) {
        for (w, pair) in data.chunks_mut(2).enumerate() {
            let k = self.ks(channel.wrapping_add(2 * w as u16));
            for (b, kb) in pair.iter_mut().zip(k) {
                *b ^= kb;
            }
        }
    }
}

/// CRC-16/ARC (reflected poly 0xA001; the FEE's table at `0x80040E08`, loop `0x800870dc`).
pub(crate) fn fee_crc(init: u16, data: &[u8]) -> u16 {
    data.iter().fold(init, |mut c, &b| {
        c ^= b as u16;
        for _ in 0..8 {
            c = if c & 1 != 0 { (c >> 1) ^ 0xA001 } else { c >> 1 };
        }
        c
    })
}

/// One FEE record in a DFlash bank (ECME, `0x800f8xxx..0x800fbxxx`). Payload in 8-byte units; the first
/// byte of every 0x100 page is a page marker, not payload. 8-byte trailer (builder `0x800f9210`, parser
/// `0x800f9098`): `[~c>>12, ~c>>4, (~c<<4)|units>>8, units, id>>8, id, c>>8, c]`, id = channel<<8 | 0x46
/// (`0x800f85f8`), `c = CRC16ARC(CRC16ARC(0xA55A, payload), seed_le32 | units_le16 | id_le16)` (`0x800f8c48`,
/// `0x800f9110`); seed = bank sequence of the bank-header record (id 0xFFFE, `0x800f8bde`).
pub(crate) struct FeeRecord {
    /// Offset of the trailer in the bank.
    pub trailer: usize,
    pub id: u16,
    pub units: usize,
}

impl FeeRecord {
    /// Offsets of the payload bytes (page markers skipped).
    pub fn payload(&self) -> impl Iterator<Item = usize> {
        (self.trailer - self.units * 8..self.trailer).filter(|i| i % 0x100 != 0)
    }

    fn crc(&self, bank: &[u8], seed: u32) -> u16 {
        let p: Vec<u8> = self.payload().map(|i| bank[i]).collect();
        let mut h = seed.to_le_bytes().to_vec();
        h.extend_from_slice(&(self.units as u16).to_le_bytes());
        h.extend_from_slice(&self.id.to_le_bytes());
        fee_crc(fee_crc(0xA55A, &p), &h)
    }

    pub fn crc_ok(&self, bank: &[u8], seed: u32) -> bool {
        u16::from_be_bytes([bank[self.trailer + 6], bank[self.trailer + 7]]) == self.crc(bank, seed)
    }

    /// Recompute the trailer checksum fields after the payload was rewritten (as `0x800f9210` builds them).
    pub fn seal(&self, bank: &mut [u8], seed: u32) {
        let c = self.crc(bank, seed);
        let (n, t) = (!c, self.trailer);
        bank[t] = (bank[t] & 0xF0) | (n >> 12) as u8;
        bank[t + 1] = (n >> 4) as u8;
        bank[t + 2] = ((n & 0xF) << 4) as u8 | (bank[t + 2] & 0x0F);
        bank[t + 6..t + 8].copy_from_slice(&c.to_be_bytes());
    }
}

/// Records of a bank whose trailer is self-consistent (stored `~c` matches `c`, `0x800f9098`).
pub(crate) fn fee_records(bank: &[u8]) -> Vec<FeeRecord> {
    (8..bank.len().saturating_sub(7))
        .step_by(8)
        .filter_map(|t| {
            let tr = &bank[t..t + 8];
            let c = u16::from_be_bytes([tr[6], tr[7]]);
            let n = ((tr[0] as u16 & 0xF) << 12) | ((tr[1] as u16) << 4) | (tr[2] as u16 >> 4);
            let units = ((tr[2] as usize & 0xF) << 8) | tr[3] as usize;
            (n == !c && units > 0 && units * 8 <= t).then(|| FeeRecord { trailer: t, id: u16::from_be_bytes([tr[4], tr[5]]), units })
        })
        .collect()
}

/// Bank sequence (the CRC seed of data records) from the bank-header record (id 0xFFFE, one unit): seed =
/// raw bytes 4..8 BE; the header is checksummed over its 8 raw bytes, page marker included.
pub(crate) fn fee_bank_seed(bank: &[u8], recs: &[FeeRecord]) -> Option<u32> {
    recs.iter().filter(|r| r.id == 0xFFFE && r.units == 1).find_map(|r| {
        let raw = &bank[r.trailer - 8..r.trailer];
        let seed = u32::from_be_bytes([raw[4], raw[5], raw[6], raw[7]]);
        let mut h = seed.to_le_bytes().to_vec();
        h.extend_from_slice(&[1, 0, 0xFE, 0xFF]);
        (fee_crc(fee_crc(0xA55A, raw), &h) == u16::from_be_bytes([bank[r.trailer + 6], bank[r.trailer + 7]])).then_some(seed)
    })
}

/// A decrypted IMO channel instance (ECME channels 6/7/8) with both record CRCs valid.
pub(crate) struct ImoInstance {
    pub rec: FeeRecord,
    pub channel: u16,
    pub plain: [u8; CHANNEL_LEN],
}

impl ImoInstance {
    /// Data-record write counter (`+0x2F`), then state-record counter (`+7`): the newest copy wins.
    pub fn age(&self) -> (u32, u32) {
        let p = &self.plain;
        let ctr = |o: usize| u32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]]);
        (ctr(STATE_LEN + 0x2F), ctr(7))
    }
}

/// Every CRC-valid instance of IMO channels 6/7/8 in a DFlash bank, decrypted.
pub(crate) fn imo_instances(bank: &[u8], cipher: &AnvmCrypt) -> Vec<ImoInstance> {
    fee_records(bank)
        .into_iter()
        .filter(|r| (6..=8).contains(&(r.id >> 8)) && r.id & 0xFF == 0x46)
        .filter_map(|rec| {
            let payload: Vec<u8> = rec.payload().map(|i| bank[i]).collect();
            let mut plain = [0u8; CHANNEL_LEN];
            plain.copy_from_slice(payload.get(..CHANNEL_LEN)?);
            let channel = rec.id >> 8;
            cipher.apply(channel, &mut plain);
            let ok = crc_ok(&plain[..STATE_LEN]) && crc_ok(&plain[STATE_LEN..STATE_LEN + DATA_LEN]);
            ok.then_some(ImoInstance { rec, channel, plain })
        })
        .collect()
}

/// ID_ecu: 2-of-3 vote over the u16 LE triple at PFlash `0x80001150` (`0x8011524c`).
pub(crate) fn id_ecu(flash: &[u8]) -> Result<[u8; 2], String> {
    let id = |i: usize| u16::from_le_bytes([flash[ID_ECU_OFF + 2 * i], flash[ID_ECU_OFF + 2 * i + 1]]);
    let (a, b, c) = (id(0), id(1), id(2));
    match (a == b || a == c, b == c) {
        (true, _) => Ok(a.to_le_bytes()),
        (_, true) => Ok(b.to_le_bytes()),
        _ => Err(format!("ID_ecu triple at 0x80001150 disagrees: {a:04X} {b:04X} {c:04X}")),
    }
}

/// Engine-ECU WFS5 credentials (partner 0, `0x010 -> 0x011`) from the SIMOS18.1 DFlash dump
/// (`eeprom`, 256 KB, file offset 0 = `0xAF000000`) and the 4 MB flash image (`flash`).
pub fn simos18_wfs5_partner(eeprom: &[u8], flash: &[u8]) -> Result<Wfs5ImmoPartner, String> {
    if flash.len() < 0x40_0000 {
        return Err(format!("flash image too short: {} bytes", flash.len()));
    }
    let uid = &flash[UID_OFF..UID_OFF + 8];
    if uid.iter().all(|&b| b == 0) || uid.iter().all(|&b| b == 0xFF) {
        return Err("no chip UID at 0x80014200 (needed for the anvmcrypt key)".into());
    }
    let id_partner = id_ecu(flash)?;
    let newest = imo_instances(&eeprom[..eeprom.len().min(DF0_LEN)], &AnvmCrypt::new(uid))
        .into_iter()
        .max_by_key(ImoInstance::age)
        .ok_or("no CRC-valid IMMO record (ECME channels 6/7/8) in the DFlash dump")?;
    let d = &newest.plain[STATE_LEN..STATE_LEN + DATA_LEN];

    let mut key = [0u8; 16];
    key.copy_from_slice(&d[1..17]);
    let vin: String = d[0x11..0x22].iter().map(|&b| b as char).collect();
    if !vin.chars().all(|c| c.is_ascii_alphanumeric()) {
        return Err(format!("decrypted VIN is not ASCII: {vin:?}"));
    }
    Ok(Wfs5ImmoPartner {
        name: "SIMOS18.1 (Partner 0)".to_string(),
        req_can_id: 0x010,
        ans_can_id: 0x011,
        key,
        power_class: d[0x23],
        id_kombi: [d[0x25], d[0x26]],
        id_partner,
        vin,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dumps() -> Option<(Vec<u8>, Vec<u8>)> {
        let root = crate::paths::test_workspace_root()?;
        let e = crate::paths::read_firmware(&root, crate::paths::DEFAULT_ECU_EEPROM).ok()?;
        let f = crate::paths::read_firmware(&root, crate::paths::DEFAULT_ECU_FLASH).ok()?;
        Some((e, f))
    }

    #[test]
    fn extracts_and_matches_firmware_vectors() {
        let Some((eeprom, flash)) = dumps() else {
            eprintln!("SIMOS18 dumps not present, skipping");
            return;
        };
        let p = simos18_wfs5_partner(&eeprom, &flash).unwrap();
        assert_eq!(p.id_partner, [0xF9, 0x3B]);
        assert_eq!(p.vin.len(), 17);

        if p.power_class == 0xBA {
            // 0x010 frames transmitted by the unmodified firmware with bench DFlash (see RE doc).
            assert!(p.verify_challenge(&[0x9F, 0xD3, 0x2E, 0x28, 0x5A, 0x05, 0x34, 0xDD]));
            assert!(p.verify_challenge(&[0x6A, 0xFA, 0x02, 0x41, 0x95, 0xA7, 0x50, 0x64]));
            let mut wrong = p.clone();
            wrong.key[0] ^= 1;
            assert!(!wrong.verify_challenge(&[0x9F, 0xD3, 0x2E, 0x28, 0x5A, 0x05, 0x34, 0xDD]));
            assert_eq!(
                p.answer_challenge(&[0x6A, 0xFA, 0x02, 0x41, 0x95, 0xA7, 0x50, 0x64]),
                [0x08, 0x6A, 0xB1 ^ p.id_kombi[0], 0x2F ^ p.id_kombi[1], 0x00, 0x00, 0x00, 0x1F]
            );
        } else {
            assert_eq!(p.power_class, 0x5C);
            assert!(p.vin.starts_with("LABEMU"));
        }
    }

    #[test]
    fn bundled_rom_dflash_is_clean_valid_and_matches_lab_keys() {
        let ws = crate::paths::find_workspace_root();
        let dflash = std::fs::read(ws.join("rom/dflash.bin")).expect("rom/dflash.bin");
        let sboot = std::fs::read(ws.join("rom/sboot.bin")).expect("rom/sboot.bin");
        assert_eq!(dflash.len(), 0x04_0000);
        assert!(!dflash.windows(3).any(|w| w == b"WVW"));

        let mut flash = vec![0u8; 0x40_0000];
        flash[..sboot.len()].copy_from_slice(&sboot);
        let uid: [u8; 12] = sboot[UID_OFF..UID_OFF + 12].try_into().unwrap();
        let lab = crate::immo_lab::LabKeys::derive(&uid);

        let bank = &dflash[..DF0_LEN];
        let recs = fee_records(bank);
        let seed = fee_bank_seed(bank, &recs).expect("FEE bank header");
        assert_eq!(seed, 0x9C3);
        let bad: Vec<usize> = recs
            .iter()
            .filter(|r| r.id != 0xFFFE && !r.crc_ok(bank, seed))
            .map(|r| r.trailer)
            .collect();
        assert!(recs.len() > 600 && bad.is_empty(), "{} records, bad {bad:x?}", recs.len());

        let insts = imo_instances(bank, &AnvmCrypt::new(&uid[..8]));
        assert_eq!(insts.len(), 397);
        for inst in &insts {
            assert!(!inst.plain.windows(3).any(|w| w == b"WVW"));
            assert_eq!(&inst.plain[83..100], lab.vin.as_bytes());
            assert_eq!(&inst.plain[STATE_LEN + 1..STATE_LEN + 17], &lab.k0()[..]);
        }

        let p = simos18_wfs5_partner(&dflash, &flash).unwrap();
        assert_eq!((p.key, p.power_class, p.id_kombi, p.vin.as_str()), (lab.k0(), 0x5C, lab.id_kombi, lab.vin.as_str()));
    }

    #[test]
    fn anvmcrypt_keystream_matches_firmware() {
        // Keystream words the firmware produced for UID 44 C8 91 07 D7 43 DC 79 (traced at 0x8001794c).
        let c = AnvmCrypt::new(&[0x44, 0xC8, 0x91, 0x07, 0xD7, 0x43, 0xDC, 0x79]);
        assert_eq!(c.ks(0x0B), [0x6B, 0x96]);
        assert_eq!(c.ks(0x0D), [0xDA, 0xFA]);
        assert_eq!(c.ks(0x11), [0x2F, 0x95]);
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
    }

    #[test]
    fn fee_trailers_match_the_firmware_checksum() {
        assert_eq!(fee_crc(0, b"123456789"), 0xBB3D); // CRC-16/ARC check value (table 0x80040E08)
        let Some((eeprom, flash)) = dumps() else { return };
        let bank = &eeprom[..DF0_LEN];
        let recs = fee_records(bank);
        let seed = fee_bank_seed(bank, &recs).expect("bank header");
        assert_eq!(seed, 0x9C3);
        // Bank headers (id 0xFFFE) are checked over their raw bytes by `fee_bank_seed`.
        let bad: Vec<usize> = recs.iter().filter(|r| r.id != 0xFFFE && !r.crc_ok(bank, seed)).map(|r| r.trailer).collect();
        assert!(recs.len() > 600 && bad.is_empty(), "{} records, bad {bad:x?}", recs.len());
        let n = imo_instances(bank, &AnvmCrypt::new(&flash[UID_OFF..UID_OFF + 8])).len();
        assert_eq!(n, 397);
    }
}
