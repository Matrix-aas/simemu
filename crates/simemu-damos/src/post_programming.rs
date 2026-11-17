//! "After flashing" view of a SIMOS18 PFlash image read from a flash container or a partial read.
//!
//! When CBOOT has programmed a block successfully it writes a block-valid marker after the block
//! (outside the block CRC range), and a flashing tool that writes a modified file recomputes the block
//! CRCs and the ECM3 monitoring sums the software checks. Images assembled from containers or tuning
//! files can lack markers and carry stale checksums. This step verifies all of them and reproduces
//! exactly those writes — never code bytes — and returns a log of every change so the UI and the session
//! log show the image as "completed after programming" (checksums are
//! verified and recomputed when needed, always with an explicit notice).
//!
//! Marker evidence (SC8O30/SC8S50): SBOOT checks 0x8003FE00 == 0x90244842 and 0x8003FF00 == 0
//! (@0x80002F94, table 0x80001444); CBOOT checks 0x8013FE00/0x801FFE00/0x808FFE00 == 0x90482442 and
//! 0xA087FE00 == 0x42244890 (@0x8001D8F2, table 0x8001CEA4). All bootable images carry them.

use crate::checksum::{fix_checksums, verify_checksums};
use crate::i18n::is_ru;

/// (block name, CRC header file offset, marker file offset, marker value).
const MARKERS: [(&str, usize, usize, u32); 5] = [
    ("CBOOT", 0x1_C300, 0x3_FE00, 0x9024_4842),
    ("ASW1", 0x4_0300, 0x13_FE00, 0x9048_2442),
    ("ASW2", 0x14_0000, 0x1F_FE00, 0x9048_2442),
    ("ASW3", 0x28_0000, 0x2F_FE00, 0x9048_2442),
    ("CAL", 0x20_0300, 0x27_FE00, 0x4224_4890),
];

/// CPU address of a 4 MB image file offset (PFlash1 half lives at 0x80800000).
fn cpu(at: usize) -> u32 {
    if at >= 0x20_0000 { 0x8080_0000 + (at - 0x20_0000) as u32 } else { 0x8000_0000 + at as u32 }
}

fn rd(f: &[u8], at: usize) -> Result<u32, String> {
    let end = at
        .checked_add(4)
        .ok_or_else(|| format!("offset 0x{at:X} overflow"))?;
    let bytes: [u8; 4] = f
        .get(at..end)
        .and_then(|s| s.try_into().ok())
        .ok_or_else(|| format!("offset 0x{at:X} out of Flash bounds"))?;
    Ok(u32::from_le_bytes(bytes))
}

/// Adds missing block-valid markers for blocks that contain data and rewrites stale block CRCs / ECM3 sums.
/// Returns one log line per change (empty = image already complete). Validates checksum headers before mutating `flash`.
pub fn apply_post_programming(flash: &mut [u8]) -> Result<Vec<String>, String> {
    if flash.len() != 0x40_0000 {
        return Err("post-programming: image must be 4 MB".into());
    }
    // Validate block headers and ranges up front so malformed images never partially mutate `flash`.
    let precheck = verify_checksums(flash)?;
    if precheck.entries.is_empty() {
        return Err("post-programming: no valid block headers found in Flash image".into());
    }
    let mut log = Vec::new();
    for (name, hdr, at, value) in MARKERS {
        let ranges = rd(flash, hdr + 8)?;
        if !(1..=16).contains(&ranges) {
            continue; // no programmed block here
        }
        let cur = rd(flash, at)?;
        if cur == value {
            continue;
        }
        if cur != 0 && cur != 0xFFFF_FFFF {
            log.push(if is_ru() {
                format!("{name}: маркер @0x{:08X} содержит 0x{cur:08X} (не стёрт) — оставлен без изменений", cpu(at))
            } else {
                format!("{name}: marker @0x{:08X} holds 0x{cur:08X} (not erased) — left unchanged", cpu(at))
            });
            continue;
        }
        flash[at..at + 4].copy_from_slice(&value.to_le_bytes());
        log.push(if is_ru() {
            format!("{name}: записан маркер валидности блока 0x{value:08X} @0x{:08X}", cpu(at))
        } else {
            format!("{name}: block-valid marker 0x{value:08X} written @0x{:08X}", cpu(at))
        });
    }
    if rd(flash, 0x3_FF00)? != 0 {
        log.push(if is_ru() {
            "CBOOT: слово @0x8003FF00 не стёрто — SBOOT отвергнет CBOOT".into()
        } else {
            "CBOOT: word @0x8003FF00 is not erased — SBOOT will reject CBOOT".into()
        });
    }
    // Every block CRC and ECM3 sum is verified; stale ones are recomputed the way a flashing tool does when it
    // writes a modified file (AGENTS.md §2.4 item 3): data words only, each logged.
    let report = verify_checksums(flash)?;
    for u in &report.unrecognised {
        log.push(if is_ru() {
            format!("Контрольная сумма не проверена — раскладка таблицы в этом ПО не распознана ({u})")
        } else {
            format!("Checksum not verified — table layout in this software version is not recognised ({u})")
        });
    }
    if !report.ok() {
        for e in fix_checksums(flash)?.entries.iter().filter(|e| !e.ok()) {
            log.push(if is_ru() {
                format!(
                    "Контрольная сумма {} пересчитана @file 0x{:06X}: 0x{:X} -> 0x{:X} (как программатор при записи изменённого файла)",
                    e.name, e.offset, e.stored, e.computed
                )
            } else {
                format!(
                    "Checksum {} recomputed @file 0x{:06X}: 0x{:X} -> 0x{:X} (as a flashing tool writes a modified file)",
                    e.name, e.offset, e.stored, e.computed
                )
            });
        }
    }
    Ok(log)
}

/// DQ250 (Temic 02E/0D9, TC1766) logical-block table in CBOOT: CBOOT 0x8001113C reads the CAL block start/length
/// from 0xA00101CA/0xA00101CE and the ASW block from 0xA00101DA/0xA00101DE (records of 16 bytes at 0x800101C8).
const DQ250_BLOCK_TABLE: [(&str, usize); 2] = [("CAL", 0x1_01CA), ("ASW", 0x1_01DA)];

const DQ250_CRC_TABLE: [u32; 256] = {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = (c >> 1) ^ (0xEDB8_8320 & (c & 1).wrapping_neg());
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
};

/// CRC-32 register as CBOOT 0x80016370/0x80016330 computes it: reflected 0xEDB88320, init 0xFFFFFFFF, no final XOR.
pub fn dq250_crc_register(data: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c = (c >> 8) ^ DQ250_CRC_TABLE[((c as u8) ^ b) as usize];
    }
    c
}

/// DQ250 equivalent of [`apply_post_programming`]: CBOOT accepts a block when the CRC-32 register over the whole
/// block (CRC word included) ends at 0. The CRC word is the block's last 32-bit word (it holds the register over the
/// preceding bytes in every block of the 0D9300011R/4319 read and in the ASW of 0D9300012F/4930). A stale CRC word is
/// recomputed the way a flashing tool does when it writes a modified file (AGENTS.md §2.4 item 3): data only, logged.
/// `flash` is the PFlash image (0x178000 bytes, or a 0x180000-byte read).
pub fn apply_post_programming_dq250(flash: &mut [u8]) -> Result<Vec<String>, String> {
    if flash.len() < 0x4_0000 {
        return Err("post-programming DQ250: image too small".into());
    }
    let mut log = Vec::new();
    for (name, at) in DQ250_BLOCK_TABLE {
        let (start, len) = (rd(flash, at)?, rd(flash, at + 4)? as usize);
        let off = (start & 0x00FF_FFFF) as usize;
        // Require `off >= 0x2_0000` so a corrupt CBOOT block table can never write into SBOOT/CBOOT (`0x00000..0x1FFFF`).
        if start & 0xDF00_0000 != 0x8000_0000
            || off < 0x2_0000
            || len < 8
            || !len.is_multiple_of(4)
            || off.saturating_add(len) > flash.len()
        {
            log.push(if is_ru() {
                format!("{name}: таблица блоков CBOOT @0x{:08X} не распознана (0x{start:08X}+0x{len:X}) — CRC не проверена", 0x8000_0000 + at as u32)
            } else {
                format!("{name}: CBOOT block table @0x{:08X} not recognised (0x{start:08X}+0x{len:X}) — CRC not checked", 0x8000_0000 + at as u32)
            });
            continue;
        }
        let end = off + len;
        if dq250_crc_register(&flash[off..end]) == 0 {
            continue;
        }
        let stored = rd(flash, end - 4)?;
        let crc = dq250_crc_register(&flash[off..end - 4]);
        flash[end - 4..end].copy_from_slice(&crc.to_le_bytes());
        log.push(if is_ru() {
            format!(
                "Контрольная сумма {name} (CRC-32 блока 0x{start:08X}+0x{len:X}) пересчитана @0x{:08X}: 0x{stored:08X} -> 0x{crc:08X} (как программатор при записи изменённого файла)",
                0x8000_0000 + (end - 4) as u32
            )
        } else {
            format!(
                "Checksum {name} (CRC-32 of block 0x{start:08X}+0x{len:X}) recomputed @0x{:08X}: 0x{stored:08X} -> 0x{crc:08X} (as a flashing tool writes a modified file)",
                0x8000_0000 + (end - 4) as u32
            )
        });
    }
    Ok(log)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dq250_block_crc_fix() {
        let mut f = vec![0u8; 0x17_8000];
        for (i, b) in f[0x2_0000..].iter_mut().enumerate() {
            *b = (i * 7 % 251) as u8;
        }
        f[0x1_01CA..0x1_01D2].copy_from_slice(&[0, 0, 2, 0xA0, 0, 0, 2, 0]); // CAL 0xA0020000+0x20000
        f[0x1_01DA..0x1_01E2].copy_from_slice(&[0, 0, 4, 0xA0, 0, 0, 0x13, 0]); // ASW 0xA0040000+0x130000
        let log = apply_post_programming_dq250(&mut f).unwrap();
        assert_eq!(log.len(), 2, "{log:?}");
        assert_eq!(dq250_crc_register(&f[0x2_0000..0x4_0000]), 0);
        assert_eq!(dq250_crc_register(&f[0x4_0000..0x17_0000]), 0);
        assert!(apply_post_programming_dq250(&mut f).unwrap().is_empty(), "idempotent");
    }

    #[test]
    fn dq250_crc_table_matches_bitwise_and_refuses_cboot_overwrite() {
        fn bitwise(data: &[u8]) -> u32 {
            let mut c = 0xFFFF_FFFFu32;
            for &b in data {
                c ^= b as u32;
                for _ in 0..8 {
                    c = (c >> 1) ^ (0xEDB8_8320 & (c & 1).wrapping_neg());
                }
            }
            c
        }
        let sample: Vec<u8> = (0..1029).map(|i| (i * 37 % 253) as u8).collect();
        assert_eq!(dq250_crc_register(&sample), bitwise(&sample));

        // Corrupt table pointing into CBOOT (`0xA0010000+0x10000`) must NOT modify SBOOT/CBOOT bytes.
        let mut f = vec![0xAAu8; 0x4_0000];
        f[0x1_01CA..0x1_01D2].copy_from_slice(&[0, 0, 1, 0xA0, 0, 0, 1, 0]); // 0xA0010000+0x10000 (< 0x20000)
        f[0x1_01DA..0x1_01E2].copy_from_slice(&[0xFF; 8]);
        let before = f.clone();
        let log = apply_post_programming_dq250(&mut f).unwrap();
        assert_eq!(f, before, "SBOOT/CBOOT must never be modified");
        assert_eq!(log.len(), 2);
    }
}
