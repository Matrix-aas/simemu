//! Lab immobiliser binding (`AGENTS.md` §2.4 exception 5: automatic WFS5 binding for any loaded image
//! with generated lab keys and explicit disclosure on load).
//!
//! The virtual instrument cluster (WFS5 master) gets ONE lab key set — slots A/B, power class B, ID_kombi,
//! VIN — and both slaves are adapted to it by writing their IMMO NVM records in the firmware's own format
//! (data only, never code; the dump files on disk stay untouched, binding happens in memory at load):
//!
//! * ECU (SIMOS18.1): ECME channels 6/7/8 in DFlash bank DF0 = state record | data record | VIN copy,
//!   `anvmcryptlight` with the chip UID, IMO CRC-16 per record, FEE trailer CRC (`immo_keys.rs`). The newest
//!   FEE instance of each channel (the one the FEE reads) is rewritten with K0, B, ID_kombi, VIN and the
//!   data-record counter + 1. A real ECU would append new instances (adaptation via the IMO download path
//!   `0x80118bfa..`, decrypted with the current K, then `0x8011afd8` → NVM write callback `0x801151C4`); the
//!   records the FEE then hands to the IMO are the same.
//! * TCU (DQ200_G2): the three plain data-record copies in the ST95640 EEPROM (`0x1E8`, `0x220`, `0x268`,
//!   read by `0x8004a2ba`, newest counter wins) get K1, B, ID_kombi, VIN, counter + 1 and CRC-16.
//!
//! Key derivation (deterministic, reproducible): `block_i = AES-128(LAB_LABEL, UID[0..12] | 00 00 00 i)`;
//! slot A = block 1, slot B = block 2, block 3 gives ID_kombi (bytes 0..2), the VIN tail (2..8) and the
//! key-set id (12..16). K0/K1 are cut from A/B like the MQB cluster does (`fpke-immo-wfs5.md` §1.2). These
//! are lab keys, not the keys of any car.
//!
//! Power class B: the IMO checks the adapted B against the calibration's list `imo_idxtun01..05_c_vw`
//! (check `0x8011c414` in SC8S50, lock state 0x42 when absent); B is taken from that list of the loaded image
//! (the dump's B when it is in the list). The DQ200 never fills its list (`0x80049f70` zeroes RAM
//! `0xD0000B74`, no other writer), and an all-zero list skips the check (`0x8004b412`): it accepts any B.
//! ID_ecu/ID_tcu are production data in PFlash (`0x80001150`, DQ200 SBOOT `0xC100/0xC304`), not binding data.

use crate::immo_keys::{
    crc16_ccitt, fee_bank_seed, fee_records, imo_instances, simos18_wfs5_partner, AnvmCrypt, ImoInstance, DATA_LEN,
    DF0_LEN, STATE_LEN, UID_OFF,
};
use simemu_tc1791::tcu_dq200::aes128_ecb_encrypt;
use simemu_tc1791::Wfs5ImmoPartner;

/// AES-128 key of the lab key derivation (the "fixed lab label").
pub const LAB_LABEL: [u8; 16] = *b"SIMEMU-IMMO-LAB1";

/// One lab key set for the virtual cluster and both slaves.
#[derive(Debug, Clone, PartialEq)]
pub struct LabKeys {
    /// `LAB-xxxxxxxx`, shown in the UI and the log.
    pub id: String,
    pub slot_a: [u8; 16],
    pub slot_b: [u8; 16],
    pub id_kombi: [u8; 2],
    pub vin: String,
}

impl LabKeys {
    pub fn derive(uid: &[u8; 12]) -> Self {
        let block = |i: u8| {
            let mut x = [0u8; 16];
            x[..12].copy_from_slice(uid);
            x[15] = i;
            aes128_ecb_encrypt(&LAB_LABEL, &x)
        };
        let (slot_a, slot_b, m) = (block(1), block(2), block(3));
        let hex = |d: &[u8]| d.iter().map(|b| format!("{b:02X}")).collect::<String>();
        Self { id: format!("LAB-{}", hex(&m[12..16])), slot_a, slot_b, id_kombi: [m[0], m[1]], vin: format!("LABEMU{}", &hex(&m[2..8])[..11]) }
    }

    /// Partner key K0 of the engine ECU (`0x010 -> 0x011`).
    pub fn k0(&self) -> [u8; 16] {
        Wfs5ImmoPartner::from_mqb_slots_ecu(&self.slot_a, &self.slot_b, 0, [0; 2], [0; 2], "").key
    }

    /// Partner key K1 of the gearbox (`0x012 -> 0x013`).
    pub fn k1(&self) -> [u8; 16] {
        Wfs5ImmoPartner::from_mqb_slots_tcu(&self.slot_a, &self.slot_b, 0, [0; 2], [0; 2], "").key
    }
}

/// `imo_idxlab_c_vw` + `imo_idxtun01..05_c_vw` of the loaded calibration.
#[derive(Debug, Clone, PartialEq)]
pub struct PowerClasses {
    /// Address of `imo_idxlab_c_vw` (the list follows at +1).
    pub cal_addr: u32,
    /// `imo_idxlab_c_vw`: 3 selects C4 = `9C F7 FE B7` (`0x80117644`).
    pub variant: u8,
    pub list: [u8; 5],
}

fn va_off(va: u32) -> Option<usize> {
    let va = va & !0x2000_0000; // 0xA... = uncached alias of 0x8...
    match va {
        0x8000_0000..=0x801F_FFFF => Some((va - 0x8000_0000) as usize),
        0x8080_0000..=0x809F_FFFF => Some((va - 0x8080_0000) as usize + 0x20_0000),
        _ => None,
    }
}

/// Sign-extended `off16` of a BOL instruction (`ld.a`/`lea` with 16-bit offset), bytes 2..4.
fn bol_off16(b2: u8, b3: u8) -> i32 {
    let w = u16::from_le_bytes([b2, b3]) as u32;
    let off = (w & 0x3F) | ((w >> 12) & 0xF) << 6 | ((w >> 6) & 0x3F) << 10;
    off as u16 as i16 as i32
}

fn find(hay: &[u8], pat: &[Option<u8>]) -> Vec<usize> {
    (0..hay.len().saturating_sub(pat.len())).filter(|&i| pat.iter().enumerate().all(|(k, p)| p.is_none_or(|v| hay[i + k] == v))).collect()
}

fn sig(s: &str) -> Vec<Option<u8>> {
    s.split_whitespace().map(|t| u8::from_str_radix(t, 16).ok()).collect()
}

/// Locates the power-class list from the code: the check routine (`state ∈ {0x63, 1, 0x58, 0x0A}` → no
/// check, else B is searched in the 5-byte RAM list; SC8S50 `0x8011c414`, SC8O40 `0x801261e0`, SC8H83
/// `0x80112e9c`) names the RAM list; its initialiser (`movh.a a15,#0x8006; ld.a a15,[a15]#P; … lea a2,[a2]#list`,
/// SC8S50 `0x8011abb8`) copies it from the calibration struct whose address is the PFlash word at `0x80060000+P`.
pub fn power_classes(flash: &[u8]) -> Result<PowerClasses, String> {
    let pf0 = flash.get(..0x20_0000).ok_or("flash image too short")?;
    let check = sig("91 10 00 4d d9 44 ?? ?? da 63 14 41 91 10 00 2d d9 22 ?? ?? 14 20 3e 17 df 11 06 00 da 58 3e 13 da 0a 7e");
    let lea_a15 = sig("91 10 00 fd d9 ff");
    let ld_ptr = sig("91 60 00 f8 99 ff");
    for c in find(pf0, &check) {
        let tail = &pf0[c + check.len()..(c + check.len() + 0x20).min(pf0.len())];
        let Some(l) = find(tail, &lea_a15).first().map(|&i| c + check.len() + i + 6) else { continue };
        let (l2, l3) = (pf0[l], pf0[l + 1]);
        for init in find(pf0, &[Some(0xD9), Some(0x22), Some(l2), Some(l3)]) {
            let back = init.saturating_sub(0x20);
            for p in find(&pf0[back..init], &ld_ptr).into_iter().map(|i| back + i + 6) {
                let ptr_va = (0x8006_0000i64 + bol_off16(pf0[p], pf0[p + 1]) as i64) as u32;
                let Some(o) = va_off(ptr_va) else { continue };
                let cal = u32::from_le_bytes(flash[o..o + 4].try_into().unwrap());
                let Some(co) = va_off(cal) else { continue };
                let s = &flash[co..co + 6];
                if s[1..].iter().any(|&b| b != 0) {
                    return Ok(PowerClasses { cal_addr: cal & !0x2000_0000, variant: s[0], list: s[1..6].try_into().unwrap() });
                }
            }
        }
    }
    Err("power-class list (imo_idxtun01..05_c_vw) not found by code signature".into())
}

/// Address of an immo-off patch of the IMO state getter (`movh.a a15,#0xd001; mov.u d2,#0x58; ret` instead of
/// `ld.bu d2,[a15]state`), e.g. `Immo_v2` (`0x8011BC60` in SC8S50, `0x801252EC` in SC8O30).
/// State 0x58 = "chip UID missing / immobiliser inactive" (`docs/SIMOS18_WFS5_RE.md` §5): no challenge is sent.
pub fn immo_off_patch(flash: &[u8]) -> Option<u32> {
    let pat = sig("91 10 00 fd bb 80 05 20 00 90");
    find(&flash[..flash.len().min(0x20_0000)], &pat).first().map(|&i| 0x8000_0000 + i as u32 + 4)
}

/// Data record (53 bytes, `0x8011c4cc`): `[1..17)` K, `[0x11..0x22)` VIN, `[0x23]` B, `[0x25..0x27)` ID_kombi,
/// `[0x2F..0x33)` write counter LE, `[0x33..0x35)` CRC-16/CCITT-FALSE LE; other fields kept.
fn write_data_record(d: &mut [u8], key: &[u8; 16], k: &LabKeys, b: u8, counter: u32) {
    simemu_tc1791::tcu_dq250::write_wfs5_data_record(d, key, &k.vin, b, k.id_kombi, counter);
}

/// Adapts the ECU DFlash image (`dflash`, file offset 0 = `0xAF000000`) to the lab key set; returns the cluster's
/// partner-0 credentials read back from the bound DFlash by the same decryption the firmware uses.
pub fn bind_ecu(dflash: &mut [u8], flash: &[u8], k: &LabKeys, pc: &PowerClasses) -> Result<Wfs5ImmoPartner, String> {
    let uid = flash.get(UID_OFF..UID_OFF + 8).ok_or("flash image too short")?;
    let cipher = AnvmCrypt::new(uid);
    let bank = dflash.get_mut(..DF0_LEN).ok_or("DFlash image shorter than bank DF0")?;
    let seed = fee_bank_seed(bank, &fee_records(bank)).ok_or("no valid FEE bank header (id 0xFFFE) in DF0")?;
    let inst = imo_instances(bank, &cipher);
    let newest = inst.iter().max_by_key(|i| i.age()).ok_or("no CRC-valid IMMO record (ECME channels 6/7/8)")?;
    let dump_b = newest.plain[STATE_LEN + 0x23];
    let b = if dump_b != 0 && pc.list.contains(&dump_b) { dump_b } else { pc.list.iter().copied().find(|&x| x != 0).ok_or("empty power-class list")? };
    let mut plain = newest.plain;
    let counter = newest.age().0.wrapping_add(1);
    write_data_record(&mut plain[STATE_LEN..STATE_LEN + DATA_LEN], &k.k0(), k, b, counter);
    plain[83..100].copy_from_slice(k.vin.as_bytes()); // VIN copy for CBOOT (FR 49.2.2.5)

    // The FEE hands the IMO the last instance of each channel; identical copies written with it are rewritten too.
    let raw = |bank: &[u8], i: &ImoInstance| i.rec.payload().map(|o| bank[o]).collect::<Vec<u8>>();
    let mut targets = Vec::new();
    for ch in 6..=8u16 {
        let last = inst.iter().filter(|i| i.channel == ch).max_by_key(|i| i.rec.trailer).ok_or(format!("IMMO channel {ch} missing"))?;
        let last_raw = raw(bank, last);
        targets.extend(inst.iter().filter(|i| i.channel == ch && raw(bank, i) == last_raw));
    }
    for t in &targets {
        if !t.rec.crc_ok(bank, seed) {
            return Err(format!("FEE trailer CRC mismatch at DF0+{:#x}: record format not understood", t.rec.trailer));
        }
        let mut enc = plain;
        cipher.apply(t.channel, &mut enc);
        for (o, v) in t.rec.payload().zip(enc) {
            bank[o] = v;
        }
        t.rec.seal(bank, seed);
    }
    let p = simos18_wfs5_partner(dflash, flash)?;
    if p.key != k.k0() || p.power_class != b || p.id_kombi != k.id_kombi {
        return Err("bound DFlash does not read back the lab key set".into());
    }
    Ok(p)
}

/// DQ200 data-record copies in the ST95640 EEPROM.
pub const DQ200_DATA_COPIES: [usize; 3] = [0x1E8, 0x220, 0x268];

/// Adapts the DQ200 EEPROM image to the lab key set with power class `b`; returns the cluster's partner-1
/// credentials read back from the bound EEPROM (`ID_tcu` from the SBOOT production data, as before).
pub fn bind_tcu(eeprom: &mut [u8], dq_flash: &[u8], k: &LabKeys, b: u8) -> Result<Wfs5ImmoPartner, String> {
    let mut counter = 0u32;
    for &o in &DQ200_DATA_COPIES {
        let d = eeprom.get(o..o + DATA_LEN).ok_or("EEPROM image too short")?;
        if crc16_ccitt(&d[..0x33]) != u16::from_le_bytes([d[0x33], d[0x34]]) {
            return Err(format!("DQ200 IMMO data copy at EEPROM {o:#x}: CRC invalid"));
        }
        counter = counter.max(u32::from_le_bytes(d[0x2F..0x33].try_into().unwrap()));
    }
    for &o in &DQ200_DATA_COPIES {
        write_data_record(&mut eeprom[o..o + DATA_LEN], &k.k1(), k, b, counter.wrapping_add(1));
    }
    let p = Wfs5ImmoPartner::from_dq200_images(dq_flash, eeprom);
    if p.key != k.k1() || p.power_class != b || p.id_kombi != k.id_kombi {
        return Err("bound EEPROM does not read back the lab key set".into());
    }
    Ok(p)
}

/// Adapts a DQ250 (0D9) SPI EEPROM image to the lab key set with power class `b`: the three IMMO data-record
/// copies (`tcu_dq250::IMMO_DATA_COPIES`) get K1, B, ID_kombi, VIN, counter + 1 and CRC-16. No DQ250 EEPROM dump
/// exists: an empty image becomes an erased 8 KB part, and when no CRC-valid copy is present the record starts from
/// zeros — its bytes other than K1/VIN/B/ID_kombi/counter/CRC are not known (UNVERIFIED, they do not stop the
/// IMO from accepting the cluster, `tests/dq250_boot.rs`). Nothing else in the EEPROM is written. Returns the
/// cluster's partner-1 credentials (ID_tcu unknown: 00 00, it is only needed to verify challenges).
pub fn bind_dq250_tcu(eeprom: &mut Vec<u8>, k: &LabKeys, b: u8) -> Result<Wfs5ImmoPartner, String> {
    use simemu_tc1791::tcu_dq250::{IMMO_DATA_COPIES, IMMO_DATA_LEN};
    if eeprom.is_empty() {
        *eeprom = vec![0xFF; simemu_tc1791::tcu_dq250::EEPROM_DEFAULT_SIZE];
    }
    let valid = |d: &[u8]| crc16_ccitt(&d[..0x33]) == u16::from_le_bytes([d[0x33], d[0x34]]);
    let copies: Vec<Vec<u8>> = IMMO_DATA_COPIES
        .iter()
        .map(|&o| eeprom.get(o..o + IMMO_DATA_LEN).map(<[u8]>::to_vec).ok_or("EEPROM image too short"))
        .collect::<Result<_, _>>()?;
    let newest = copies.iter().filter(|d| valid(d)).max_by_key(|d| u32::from_le_bytes(d[0x2F..0x33].try_into().unwrap()));
    let (mut rec, counter) = match newest {
        Some(d) => (d.clone(), u32::from_le_bytes(d[0x2F..0x33].try_into().unwrap()).wrapping_add(1)),
        None => (vec![0; IMMO_DATA_LEN], 1),
    };
    // already bound to this key set (EEPROM kept from an earlier power cycle): nothing is written
    let mut same = rec.clone();
    write_data_record(&mut same, &k.k1(), k, b, counter.wrapping_sub(1));
    if newest != Some(&same) {
        write_data_record(&mut rec, &k.k1(), k, b, counter);
        for &o in &IMMO_DATA_COPIES {
            eeprom[o..o + IMMO_DATA_LEN].copy_from_slice(&rec);
        }
    }
    let mut p = Wfs5ImmoPartner::from_mqb_slots_tcu(&k.slot_a, &k.slot_b, b, k.id_kombi, [0; 2], &k.vin);
    p.name = "DQ250 0D9 (Partner 1)".into();
    Ok(p)
}

/// What the session loads into the ECU and the cluster for one image.
pub struct EcuBinding {
    /// DFlash image for the board.
    pub nvm: Vec<u8>,
    /// Cluster side of partner 0 (`None`: credentials unknown).
    pub cluster: Option<Wfs5ImmoPartner>,
    /// Lab key set and power class the TCU must be bound with (`None`: TCU keeps its dump).
    pub tcu: Option<(LabKeys, u8)>,
    /// One-line summary for the UI header / log.
    pub summary: String,
    pub notes: Vec<String>,
}

/// Binding of one ECU image: lab binding (`lab`) or the dump's own credentials (bench SC8S50 only works then).
pub fn ecu_binding(lab: bool, flash: &[u8], dflash: &[u8]) -> EcuBinding {
    let ru = simemu_damos::is_ru();
    let patch = immo_off_patch(flash).map(|a| {
        if ru {
            format!("Образ содержит патч immo-off @{a:#010X} (геттер состояния → 0x58): прошивка не вызывает приборку (0x010 не будет), записи иммо ею не проверяются. Код не восстанавливается.")
        } else {
            format!("Image contains an immo-off patch @{a:#010X} (state getter → 0x58): firmware does not challenge the cluster (no 0x010), IMMO records are bypassed. Code is left unmodified.")
        }
    });
    let dump = || {
        let p = simos18_wfs5_partner(dflash, flash);
        let summary = match &p {
            Ok(p) if ru => format!("Иммобилайзер: данные дампа DFlash (K0, B={:#04X}, ID_kombi {:02X} {:02X}) — лабораторная привязка выключена", p.power_class, p.id_kombi[0], p.id_kombi[1]),
            Ok(p) => format!("Immobiliser: DFlash dump credentials (K0, B={:#04X}, ID_kombi {:02X} {:02X}) — lab binding disabled", p.power_class, p.id_kombi[0], p.id_kombi[1]),
            Err(e) if ru => format!("Иммобилайзер: данные дампа DFlash не прочитаны ({e}) — ответы приборки будут отвергнуты"),
            Err(e) => format!("Immobiliser: DFlash dump credentials unreadable ({e}) — cluster answers will be rejected"),
        };
        EcuBinding { nvm: dflash.to_vec(), cluster: p.ok(), tcu: None, notes: vec![summary.clone()].into_iter().chain(patch.clone()).collect(), summary }
    };
    if !lab {
        return dump();
    }
    let attempt = || -> Result<EcuBinding, String> {
        let uid: [u8; 12] = flash.get(UID_OFF..UID_OFF + 12).ok_or("flash image too short")?.try_into().unwrap();
        let k = LabKeys::derive(&uid);
        let pc = power_classes(flash)?;
        let mut nvm = dflash.to_vec();
        let p = bind_ecu(&mut nvm, flash, &k, &pc)?;
        let b = p.power_class;
        let idx = pc.list.iter().position(|&x| x == b).unwrap_or(0) + 1;
        let summary = if ru {
            format!(
                "Иммобилайзер: лабораторная привязка — сгенерированы ключи {}, записаны записи DFlash/EEPROM (данные, не код); класс мощности B={b:#04X} из калибровки @{:#010X}",
                k.id,
                pc.cal_addr + idx as u32
            )
        } else {
            format!(
                "Immobiliser: lab binding — generated key set {}, bound DFlash/EEPROM records in RAM (data only, never code); power class B={b:#04X} from calibration @{:#010X}",
                k.id,
                pc.cal_addr + idx as u32
            )
        };
        let uid_hex = uid.iter().map(|b| format!("{b:02X}")).collect::<String>();
        let mut notes = if ru {
            vec![
                summary.clone(),
                format!(
                    "Ключи лабораторные (не ключи автомобиля): слоты A/B = AES-128(\"SIMEMU-IMMO-LAB1\", UID чипа {uid_hex}), ID_kombi {:02X} {:02X}, VIN {}",
                    k.id_kombi[0], k.id_kombi[1], k.vin
                ),
                format!(
                    "ЭБУ: переписаны последние экземпляры каналов ECME 6/7/8 в DFlash (anvmcrypt, CRC IMO, CRC FEE): K0, B, ID_kombi, VIN, счётчик записи; список классов калибровки {:02X?} (imo_idxtun01..05 @{:#010X}), ID_ecu {:02X} {:02X} из PFlash не меняется; файл дампа на диске не изменён",
                    pc.list,
                    pc.cal_addr + 1,
                    p.id_partner[0],
                    p.id_partner[1]
                ),
            ]
        } else {
            vec![
                summary.clone(),
                format!(
                    "Lab keys (not vehicle keys): slots A/B = AES-128(\"SIMEMU-IMMO-LAB1\", chip UID {uid_hex}), ID_kombi {:02X} {:02X}, VIN {}",
                    k.id_kombi[0], k.id_kombi[1], k.vin
                ),
                format!(
                    "ECU: rewrote latest ECME channel 6/7/8 instances in DFlash (anvmcrypt, IMO CRC, FEE CRC): K0, B, ID_kombi, VIN, write counter; calibration power classes {:02X?} (imo_idxtun01..05 @{:#010X}), PFlash ID_ecu {:02X} {:02X} unchanged; dump file on disk untouched",
                    pc.list,
                    pc.cal_addr + 1,
                    p.id_partner[0],
                    p.id_partner[1]
                ),
            ]
        };
        if pc.variant != 3 {
            notes.push(if ru {
                format!("imo_idxlab_c_vw = {} (не 3): константа C4 приборки не проверена для этого варианта", pc.variant)
            } else {
                format!("imo_idxlab_c_vw = {} (not 3): cluster constant C4 is unverified for this variant", pc.variant)
            });
        }
        notes.extend(patch.clone());
        Ok(EcuBinding { nvm, cluster: Some(p), tcu: Some((k, b)), summary, notes })
    };
    attempt().unwrap_or_else(|e| {
        let mut b = dump();
        b.notes.insert(
            0,
            if ru {
                format!("Лабораторная привязка иммобилайзера не выполнена ({e}); используются данные дампа")
            } else {
                format!("Immobiliser lab binding skipped ({e}); falling back to dump credentials")
            },
        );
        b
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::immo_keys::{fee_records, DF0_LEN};
    use std::path::PathBuf;

    const FLASH: &str = crate::paths::DEFAULT_ECU_FLASH;
    const EEPROM: &str = crate::paths::DEFAULT_ECU_EEPROM;
    const DQ_FLASH: &str = crate::paths::DEFAULT_DQ200_FLASH;
    const DQ_EEPROM: &str = crate::paths::DEFAULT_DQ200_EEPROM;

    fn root() -> Option<PathBuf> {
        crate::paths::test_workspace_root()
    }

    fn read(f: &str) -> Option<Vec<u8>> {
        let r = root()?;
        let mut v = crate::paths::read_firmware(&r, f).ok()?;
        if v.len() == 0x40_0000 && v[..0x1_C000].iter().all(|&b| b == 0 || b == 0xFF) {
            let (_, base) = crate::paths::read_sboot_donor(&r).ok()?;
            v[..0x1_C000].copy_from_slice(&base[..0x1_C000]);
        }
        Some(v)
    }

    #[test]
    fn lab_keys_are_deterministic_and_marked() {
        let uid = [0x44, 0xC8, 0x91, 0x07, 0xD7, 0x43, 0xDC, 0x79, 0x0C, 0x0C, 0x00, 0x20];
        let (a, b) = (LabKeys::derive(&uid), LabKeys::derive(&uid));
        assert_eq!(a, b);
        assert!(a.id.starts_with("LAB-") && a.vin.starts_with("LABEMU") && a.vin.len() == 17);
        assert_ne!(a.k0(), a.k1());
        let mut other = uid;
        other[11] ^= 1;
        assert_ne!(LabKeys::derive(&other).slot_a, a.slot_a);
    }

    /// Power-class lists of every image, found from code; cross-checked with the DAMOS addresses where the
    /// definition has them (SC8S50 `data_imodat.imo_idxtun01_c_vw` = 0x8087A085, SC8O30 = 0x80879401).
    #[test]
    fn power_class_lists_from_code() {
        for (f, addr, list) in [
            ("8V0906264K__0003.bin", 0x8087_A084, [0x6A, 0, 0, 0, 0]),
            ("8V0906264H__0005.bin", 0x8087_944C, [0x5C, 0, 0, 0, 0]),
            ("8V0906264J__0003.bin", 0x8087_944C, [0x5C, 0, 0, 0, 0]),
            ("8V0906264F__0003.bin", 0x8087_AEDC, [0x6A, 0, 0, 0, 0]),
        ] {
            let Some(flash) = read(f) else { continue };
            let pc = power_classes(&flash).unwrap_or_else(|e| panic!("{f}: {e}"));
            assert_eq!((pc.cal_addr, pc.variant, pc.list), (addr, 3, list), "{f}");
        }
    }

    #[test]
    fn immo_off_patch_detection() {
        let mut synthetic = vec![0xFFu8; 0x20_0000];
        assert_eq!(immo_off_patch(&synthetic), None);
        synthetic[0x11_BC5C..0x11_BC66].copy_from_slice(&[0x91, 0x10, 0x00, 0xFD, 0xBB, 0x80, 0x05, 0x20, 0x00, 0x90]);
        assert_eq!(immo_off_patch(&synthetic), Some(0x8011_BC60));
        for f in ["8V0906264H__0005.bin", "8V0906264K__0003.bin", "8V0906264F__0003.bin"] {
            if let Some(x) = read(f) {
                assert_eq!(immo_off_patch(&x), None, "{f}");
            }
        }
    }

    /// Record format / encryption round trip: the bound DFlash decrypts (firmware cipher + CRCs) to the lab
    /// key set, every FEE trailer stays valid, only IMMO payloads and their trailers changed; the DQ200 EEPROM
    /// copies read back the same set.
    #[test]
    fn binding_round_trip() {
        let (Some(flash), Some(dump)) = (read(FLASH), read(EEPROM)) else { return };
        let ecu = ecu_binding(true, &flash, &dump);
        let (k, b) = ecu.tcu.clone().unwrap_or_else(|| panic!("{}", ecu.notes.join("\n")));
        let p = ecu.cluster.unwrap();
        assert_eq!((p.key, p.power_class, p.id_kombi, p.vin.as_str()), (k.k0(), 0x5C, k.id_kombi, k.vin.as_str()));
        assert!(ecu.summary.contains(&k.id) && ecu.summary.contains("B=0x5C") && ecu.summary.contains("@0x8087944D"));
        let (bank, old) = (&ecu.nvm[..DF0_LEN], &dump[..DF0_LEN]);
        let seed = fee_bank_seed(bank, &fee_records(bank)).unwrap();
        assert!(fee_records(bank).iter().all(|r| r.id == 0xFFFE || r.crc_ok(bank, seed)));
        let changed = (0..DF0_LEN).filter(|&i| bank[i] != old[i]).count();
        assert!(changed > 0 && changed < 8 * 110, "{changed} bytes changed");
        assert_eq!(&ecu.nvm[DF0_LEN..], &dump[DF0_LEN..]);

        // Dump mode keeps the DFlash's own credentials untouched.
        let expected_dump_b = simos18_wfs5_partner(&dump, &flash).unwrap().power_class;
        let d = ecu_binding(false, &flash, &dump);
        assert_eq!(d.nvm, dump);
        assert_eq!(d.cluster.unwrap().power_class, expected_dump_b);

        // Bundled `rom/dflash.bin` also round-trips cleanly through `ecu_binding(true, ...)`:
        if let Some(ws) = root() {
            let bundled = std::fs::read(ws.join("rom/dflash.bin")).unwrap();
            let b_res = ecu_binding(true, &flash, &bundled);
            let b_changed = (0..DF0_LEN).filter(|&i| b_res.nvm[i] != bundled[i]).count();
            assert!(b_changed > 0 && b_changed < 8 * 110, "rom/dflash.bin changed {b_changed} bytes");
        }

        let (Some(dqf), Some(dqe)) = (read(DQ_FLASH), read(DQ_EEPROM)) else { return };
        let mut e = dqe.clone();
        let t = bind_tcu(&mut e, &dqf, &k, b).unwrap();
        assert_eq!((t.key, t.power_class, t.id_kombi, t.vin.as_str()), (k.k1(), b, k.id_kombi, k.vin.as_str()));
        assert_eq!(t.id_partner, Wfs5ImmoPartner::from_dq200_images(&dqf, &dqe).id_partner);
        for &o in &DQ200_DATA_COPIES {
            assert_eq!(&e[o..o + DATA_LEN], &e[0x1E8..0x1E8 + DATA_LEN]);
            assert_eq!(u32::from_le_bytes(e[o + 0x2F..o + 0x33].try_into().unwrap()), 4);
        }
        assert!(e.iter().zip(&dqe).filter(|(x, y)| x != y).count() <= 3 * DATA_LEN);
    }

    /// Board-level protocol check on every image (no plant; also images studio cannot load): the bound ECU
    /// challenges and stops after the lab cluster's answer; with K0 one bit off it keeps challenging.
    /// DQ250 (0D9): no EEPROM dump exists, so the lab binding writes the three IMMO data-record copies into an
    /// erased part; the unmodified ASW (4319 read and tuned 4930, CRC words recomputed) then sends its
    /// 0x012 challenge with the lab K1 / B and stops after the cluster's answer, while a K1 one bit off keeps it
    /// asking. Board supply input at battery level, chassis frames as on the bench. Both CPU backends via `SIMEMU_CPU`.
    #[cfg(feature = "bridge")]
    #[test]
    #[ignore]
    fn dq250_lab_binding() {
        let uid = [0x44, 0xC8, 0x91, 0x07, 0xD7, 0x43, 0xDC, 0x79, 0x0C, 0x0C, 0x00, 0x20];
        let k = LabKeys::derive(&uid);
        let (mut eeprom, mut ran) = (Vec::new(), 0);
        let p = bind_dq250_tcu(&mut eeprom, &k, 0x5C).unwrap();
        let rec = &eeprom[simemu_tc1791::tcu_dq250::IMMO_DATA_COPIES[0]..][..simemu_tc1791::tcu_dq250::IMMO_DATA_LEN];
        assert_eq!((&rec[1..17], rec[0x23], [rec[0x25], rec[0x26]]), (&k.k1()[..], 0x5C, k.id_kombi));
        let erased = eeprom.iter().enumerate().filter(|(i, _)| !simemu_tc1791::tcu_dq250::IMMO_DATA_COPIES.iter().any(|&o| (o..o + 53).contains(i)));
        assert!(erased.into_iter().all(|(_, &b)| b == 0xFF), "nothing but the IMMO records is written");
        if let Some(r) = root() {
            for img in crate::studio::dq250_prep::dq250_images(&r) {
                let Some(mut flash) = read(&img.file) else { continue };
                let f = &img.file;
                simemu_damos::apply_post_programming_dq250(&mut flash).unwrap();
                let mut counts = Vec::new();
                for flip in [0u8, 1] {
                    let mut c = p.clone();
                    c.key[0] ^= flip;
                    let mut b = simemu_tc1791::Dq250TcuBoard::new(&flash, &eeprom).unwrap();
                    let mut enc = simemu_bridge::MqbChassisEncoder::default();
                    let ci = simemu_bridge::ChassisInputs { selector_code: 8, terminal_15: true, ..Default::default() };
                    let (mut n, mut seen) = (0, 0);
                    for _ in 0..600 {
                        let mut rx = Vec::new();
                        for fr in b.can_tx()[seen..].iter().filter(|fr| fr.arb_id == 0x012) {
                            n += 1;
                            rx.push((0x013, 8, c.answer_challenge(&fr.data)));
                        }
                        seen = b.can_tx().len();
                        rx.extend(enc.encode(&ci));
                        for (id, dlc, data) in rx {
                            b.inject_can_rx(&simemu_tc1791::CanFrame { node: 1, mo_index: 0, arb_id: id, extended: false, dlc, data, timestamp_us: 0 });
                        }
                        b.run_for(0.01).unwrap();
                    }
                    counts.push(n);
                }
                eprintln!("{f}: 0x012 in 6 s: lab key {}, K1 one bit off {}", counts[0], counts[1]);
                assert!((1..=2).contains(&counts[0]) && counts[1] > 10, "{f}: {counts:?}");
                ran += 1;
            }
        }
        if ran == 0 {
            eprintln!("SKIP: no DQ250 image");
        }
    }

    /// Both CPU backends via `SIMEMU_CPU`.
    #[test]
    #[ignore]
    fn board_level_lab_binding() {
        let Some(dump) = read(EEPROM) else { return };
        for f in [FLASH, "8V0906264K__0003.bin", "8V0906264H__0005.bin", "8V0906264J__0003.bin", "8V0906264F__0003.bin"] {
            let Some(mut flash) = read(f) else { continue };
            let _ = simemu_damos::apply_post_programming(&mut flash);
            let b = ecu_binding(true, &flash, &dump);
            let cluster = b.cluster.clone().expect("lab binding");
            let golf = ecu_binding(false, &flash, &dump);
            let mut counts = Vec::new();
            // lab binding, lab binding with K0 one bit off, the dump's Golf credentials (before this change)
            for (flip, nvm, cl) in [(0u8, &b.nvm, &cluster), (1, &b.nvm, &cluster), (0, &golf.nvm, golf.cluster.as_ref().unwrap())] {
                let mut c = cl.clone();
                c.key[0] ^= flip;
                let mut board = simemu_tc1791::Simos18Board::new(&flash, nvm).unwrap();
                board.set_adc_input_voltage(2, 4, 2.5).unwrap(); // supply sense (boot_regression)
                let mut n = 0;
                for _ in 0..300 {
                    board.step_realtime_frame(0.010).unwrap();
                    for fr in board.take_can_tx().into_iter().filter(|fr| fr.arb_id == 0x010) {
                        n += 1;
                        if flip == 0 {
                            assert!(c.verify_challenge(&fr.data), "{f}: 0x010 MAC/ID_ecu mismatch {:02X?}", fr.data);
                        }
                        let a = c.answer_challenge(&fr.data);
                        board.inject_can_rx(&simemu_tc1791::CanFrame { node: 0, mo_index: 0, arb_id: 0x011, extended: false, dlc: 8, data: a, timestamp_us: 0 });
                    }
                }
                counts.push(n);
            }
            eprintln!("{f}: {} | 0x010 in 3 s: lab key {}, wrong key {}, dump (Golf B=0xBA) {}", b.summary, counts[0], counts[1], counts[2]);
            if immo_off_patch(&flash).is_some() {
                assert_eq!(counts, [0, 0, 0], "{f}");
            } else {
                assert!((1..=2).contains(&counts[0]) && counts[1] > 10, "{f}: {counts:?}");
                // With the dump's B=0xBA only the calibrations listing 0xBA challenge at all (power-class lock 0x42).
                let golf_ok = power_classes(&flash).unwrap().list.contains(&0xBA);
                assert_eq!(counts[2] > 0, golf_ok, "{f}: {counts:?}");
            }
        }
    }

    /// What one run on the virtual car shows (read-only observables).
    #[cfg(feature = "studio")]
    #[derive(Debug, Default)]
    struct Run {
        challenges: u64,
        last_challenge_s: f64,
        sparks: u32,
        injections: u32,
        max_rpm: f64,
        /// DQ200 WFS5 manager reached state 7 (valid 0x013, `Dq200LiveState::wfs5_state`).
        tcu_authorized: Option<bool>,
        /// SC8S50 only (`docs/SIMOS18_WFS5_RE.md` §5): 0x428 / 0x42A / 0x42B.
        s50_flags: Option<[u8; 3]>,
        /// DQ200 WFS5 manager states in order of appearance.
        tcu_states: Vec<u8>,
        /// Cumulative injection / spark pulses at the end of every second.
        inj_s: Vec<u32>,
        spk_s: Vec<u32>,
    }

    /// Ignition on, brake, KL50 1..6 s (the firmware runs the starter), `secs` of co-simulation.
    #[cfg(feature = "studio")]
    fn run(s: &mut crate::studio::session::StudioSession, secs: f64) -> Run {
        let mut r = Run::default();
        let c0 = s.immo.ecu_challenges;
        s.plant.driver.ignition_on = true;
        let (mut spark_end, mut inj_end) = ([0.0f64; 4], [0.0f64; 4]);
        // port (MPI) injection counts too: warm idle runs on MPI (Audi SSP 606 p.59–60, GPTA doc §12.10)
        let mut mpi_last: [Option<(f64, f64)>; 4] = [None; 4];
        for i in 0..(secs / 0.01) as usize {
            let t = (i + 1) as f64 * 0.01;
            s.plant.driver.brake_pct = if t >= 0.5 { 60.0 } else { 0.0 };
            s.plant.driver.start_button = (1.0..6.0).contains(&t);
            let n = s.immo.ecu_challenges;
            s.step(0.01);
            if s.immo.ecu_challenges != n {
                r.last_challenge_s = t;
            }
            let io = s.board.engine_io_state();
            for c in 0..4 {
                if let Some(p) = io.coil[c].filter(|p| p.end_s != spark_end[c]) {
                    spark_end[c] = p.end_s;
                    r.sparks += 1;
                }
                if let Some(p) = io.injector[c].last().filter(|p| p.end_s != inj_end[c]) {
                    inj_end[c] = p.end_s;
                    r.injections += 1;
                }
                let mpi = s.actuators.ecu.port_injection[c][0].map(|p| (p.soi_btdc_deg, p.width_ms));
                if mpi.is_some() && mpi != mpi_last[c] {
                    r.injections += 1;
                }
                mpi_last[c] = mpi;
            }
            r.max_rpm = r.max_rpm.max(s.plant.telemetry().rpm);
            if let Some(st) = s.tcu.as_ref().and_then(|t| t.dq200()).map(|t| t.read_live_state().wfs5_state) {
                if r.tcu_states.last() != Some(&st) {
                    r.tcu_states.push(st);
                }
            }
            if i % 100 == 99 {
                r.inj_s.push(r.injections);
                r.spk_s.push(r.sparks);
            }
        }
        r.challenges = s.immo.ecu_challenges - c0;
        r.tcu_authorized = r.tcu_states.contains(&7).then_some(true).or(s.tcu.as_ref().map(|_| false));
        if s.image_file == crate::studio::session::S50_FLASH {
            let m = s.board.read_mem(0xD001_0428, 4).unwrap();
            r.s50_flags = Some([m[0], m[2], m[3]]);
        }
        r
    }

    /// Every image on the virtual car with the lab binding: the ECU challenges (0x010), accepts the lab cluster's
    /// answer (stops re-challenging; SC8S50 RAM flags), ignition and injection run while cranking, the DQ200
    /// authorises. Negative control: the cluster's K0 with one bit flipped. `SIMEMU_IMMO_IMAGES=a,b` selects images.
    /// `SIMEMU_CPU=native cargo test --release -p simemu-cli lab_binding_e2e -- --ignored --nocapture`
    #[cfg(feature = "studio")]
    #[test]
    #[ignore]
    fn lab_binding_e2e() {
        use crate::studio::session::{StudioSession, S50_FLASH};
        let Some(root) = root() else { return };
        let images: Vec<String> = std::env::var("SIMEMU_IMMO_IMAGES").map(|v| v.split(',').map(String::from).collect()).unwrap_or_else(|_| {
            [S50_FLASH, "8V0906264K__0003.bin", "8V0906264H__0005.bin", "8V0906264J__0003.bin", "8V0906264F__0003.bin"]
                .map(String::from)
                .to_vec()
        });
        let mut s = StudioSession::new(&root).expect("session");
        assert!(
            !s.immo_lab || s.immo.binding.contains("lab binding") || s.immo.binding.contains("лабораторная привязка"),
            "{}",
            s.immo.binding
        );
        // Warm engine, short engine-off time (as engine_run's warm start), set before the ECU powers up.
        let warm = |s: &mut StudioSession| {
            s.plant.soak(90.0);
            s.eot_clock_s = 300.0;
        };
        for img in images.iter().filter(|f| crate::paths::has_firmware(&root, f)) {
            s.plant.reset();
            warm(&mut s);
            if let Err(e) = s.load_image(img) {
                eprintln!("{img}: studio cannot load the image ({e}); covered by board_level_lab_binding only");
                continue;
            }
            let patched = immo_off_patch(&s.current_flash).is_some();
            let secs = 8.0;
            let ok = run(&mut s, secs);
            eprintln!("{img}: {} | lab key: {ok:?}", s.immo.binding);
            // Both boards powered up again, the cluster answering with K0/K1 one bit off.
            let cycle = |s: &mut StudioSession| {
                s.plant.reset();
                warm(s);
                s.power_cycle_ecu().unwrap();
                s.power_cycle_tcu();
            };
            cycle(&mut s);
            let good = s.cluster_ecu.clone();
            s.cluster_ecu.key[0] ^= 1;
            if let Some(p) = s.cluster_tcu.as_mut() {
                p.key[0] ^= 1;
            }
            let bad = run(&mut s, secs);
            s.cluster_ecu = good;
            cycle(&mut s);
            eprintln!("{img}: wrong key: {bad:?}");
            // Firing between 5 s and the end: after the immobiliser's lock time (~3.9 s, observed) the firmware cuts
            // ignition and injection unless the cluster's answer was accepted.
            let late = |r: &Run| (r.spk_s[7] - r.spk_s[4], r.inj_s[7] - r.inj_s[4]);
            assert_ne!(bad.tcu_authorized, Some(true), "{img}: TCU accepted a wrong K1");
            if patched {
                // Immo-off code patch: no challenge at all, with any cluster answer (the binding is not checked).
                assert_eq!(ok.challenges + bad.challenges, 0, "{img}");
                continue;
            }
            assert!((1..=2).contains(&ok.challenges) && ok.last_challenge_s < 3.0, "{img}: lab-bound ECU must challenge and stop: {ok:?}");
            assert!(late(&ok).0 > 0 && late(&ok).1 > 0, "{img}: ignition/injection not released: {ok:?}");
            assert_eq!(ok.tcu_authorized, Some(true), "{img}: TCU did not authorise");
            assert!(bad.challenges > 10 && late(&bad) == (0, 0), "{img}: wrong key accepted: {bad:?}");
            if let Some(f) = ok.s50_flags {
                assert_eq!((f, bad.s50_flags.map(|b| b[0])), ([1, 1, 1], Some(0)), "{img}");
            }
        }
    }
}
