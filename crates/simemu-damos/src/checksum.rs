//! SIMOS18.1 Flash checksums, verified and fixed the way a flashing tool does it.
//!
//! Input is the full 4 MB image (`PFlash0` 0x80000000.. followed by `PFlash1` 0x80800000..).
//! Layouts below were reproduced bit-exactly on SC8S50 and SC8O30 images.
//!
//! **Block CRC32** (checked by CBOOT when a block is programmed). Each block has a header at a fixed
//! spot: `+4` CRC32 (u32 LE), `+8` range count (u32 LE, 0 = block absent from the image),
//! `+12` `count` pairs of *inclusive* `(start, end)` CPU addresses (`0x8…` or uncached `0xA…`).
//! CRC-32: poly `0x04C11DB7`, MSB-first (not reflected), init 0, no final XOR, over the ranges in order.
//!
//! **ECM3 monitoring sums** (ASW background ROM test, SC8S50 code `0x800807F4`/`0x80080824` for ASW,
//! `0x800808DC`/`0x8008090C` for CAL): `acc = 0x0123456789ABCDEF + Σ u32 LE words` (mod 2^64) over
//! *end-exclusive* ranges, compared as `acc >> 32 == ref[0]` and `acc as u32 == ref[1]` (u32 LE each).
//! - ASW: range table at ASW1+0x500, count at ASW1+0x550, reference at ASW1+0x540 (0x80040540).
//! - CAL: range table at ASW1+0x520, count at CAL+0x410, reference at CAL+0x400 (0x80800400).
//!
//! The CAL ECM3 reference lies inside the CAL CRC ranges, so ECM3 is fixed before the block CRCs.

use serde::Serialize;

const FLASH_LEN: usize = 0x40_0000;
const ECM3_SEED: u64 = 0x0123_4567_89AB_CDEF;

enum Kind {
    /// Header file offset.
    Block(usize),
    /// Range table file offset, range-count file offset.
    Ecm3(usize, usize),
}

/// `(name, file offset of the stored checksum, kind)` in fix order.
const SPECS: [(&str, usize, Kind); 8] = [
    ("ECM3 ASW", 0x4_0540, Kind::Ecm3(0x4_0500, 0x4_0550)),
    ("ECM3 CAL", 0x20_0400, Kind::Ecm3(0x4_0520, 0x20_0410)),
    ("SBOOT", 0x304, Kind::Block(0x300)),
    ("CBOOT", 0x1_C304, Kind::Block(0x1_C300)),
    ("ASW1", 0x4_0304, Kind::Block(0x4_0300)),
    ("ASW2", 0x14_0004, Kind::Block(0x14_0000)),
    ("ASW3", 0x28_0004, Kind::Block(0x28_0000)),
    ("CAL", 0x20_0304, Kind::Block(0x20_0300)),
];

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ChecksumEntry {
    pub name: &'static str,
    /// File offset of the stored checksum (ECM3: 8 bytes `hi, lo`; block: 4 bytes).
    pub offset: usize,
    pub stored: u64,
    pub computed: u64,
}

impl ChecksumEntry {
    pub fn ok(&self) -> bool {
        self.stored == self.computed
    }
}

/// Blocks whose header has zero ranges (e.g. no SBOOT in the image) are omitted.
#[derive(Debug, Clone, Default, Serialize)]
pub struct ChecksumReport {
    pub entries: Vec<ChecksumEntry>,
    /// ECM3 sums whose range table is not laid out as on SC8S50/SC8O30/SC8O40 in this software (e.g. SC8H83):
    /// neither verified nor fixed, reported instead of failing the whole image.
    pub unrecognised: Vec<String>,
}

impl ChecksumReport {
    /// Returns `true` when at least one programmable block was verified and all verified checksums match.
    pub fn ok(&self) -> bool {
        !self.entries.is_empty() && self.entries.iter().all(ChecksumEntry::ok)
    }

    pub fn get(&self, name: &str) -> Option<&ChecksumEntry> {
        self.entries.iter().find(|e| e.name == name)
    }
}

const CRC_TABLE: [u32; 256] = {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = (i as u32) << 24;
        let mut k = 0;
        while k < 8 {
            c = if c & 0x8000_0000 != 0 {
                (c << 1) ^ 0x04C1_1DB7
            } else {
                c << 1
            };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
};

fn crc32(mut crc: u32, data: &[u8]) -> u32 {
    for &b in data {
        crc = (crc << 8) ^ CRC_TABLE[((crc >> 24) as u8 ^ b) as usize];
    }
    crc
}

fn u32_at(flash: &[u8], off: usize) -> Result<u32, String> {
    let end = off
        .checked_add(4)
        .ok_or_else(|| format!("offset 0x{off:X} overflow"))?;
    let bytes: [u8; 4] = flash
        .get(off..end)
        .and_then(|s| s.try_into().ok())
        .ok_or_else(|| format!("offset 0x{off:X} out of Flash bounds"))?;
    Ok(u32::from_le_bytes(bytes))
}

/// CPU address (cached `0x8…` or uncached `0xA…`) → file offset.
fn file_off(addr: u32) -> Result<usize, String> {
    let low = (addr & 0x0FFF_FFFF) as usize;
    let off = match low {
        0..=0x1F_FFFF => Some(low),
        0x80_0000..=0x9F_FFFF => Some(low - 0x60_0000),
        _ => None,
    };
    match off {
        Some(o) if matches!(addr >> 28, 0x8 | 0xA) => Ok(o),
        _ => Err(format!("address 0x{addr:08X} is outside PFlash")),
    }
}

/// File range `[start, end)` for CPU range `[s, e)`, which must not straddle the two banks.
fn file_range(s: u32, e: u32) -> Result<(usize, usize), String> {
    let bad = || format!("bad range 0x{s:08X}..0x{e:08X}");
    if s >= e {
        return Err(bad());
    }
    let (so, eo) = (file_off(s)?, file_off(e - 1)? + 1);
    if eo < so || eo - so != (e - s) as usize {
        return Err(bad());
    }
    Ok((so, eo))
}

fn compute(
    flash: &[u8],
    name: &'static str,
    at: usize,
    kind: &Kind,
) -> Result<Option<ChecksumEntry>, String> {
    let err = |m: String| format!("{name}: {m}");
    let (stored, computed) = match *kind {
        Kind::Block(hdr) => {
            let n_raw = u32_at(flash, hdr + 8).map_err(err)?;
            if n_raw == 0 || n_raw == 0xFFFF_FFFF {
                return Ok(None);
            }
            let n = n_raw as usize;
            if n > 16 {
                return Err(err(format!("implausible range count {n}")));
            }
            let mut crc = 0;
            for i in 0..n {
                let p = hdr + 12 + 8 * i;
                let (s, e) = (u32_at(flash, p).map_err(err)?, u32_at(flash, p + 4).map_err(err)?);
                let (so, eo) = file_range(s, e.wrapping_add(1)).map_err(err)?;
                if at < eo && so < at + 4 {
                    return Err(err("CRC field lies inside its own range".into()));
                }
                crc = crc32(crc, &flash[so..eo]);
            }
            (u32_at(flash, at).map_err(err)? as u64, crc as u64)
        }
        Kind::Ecm3(table, count_at) => {
            let n = u32_at(flash, count_at).map_err(err)? as usize;
            if !(1..=4).contains(&n) {
                return Err(err(format!("implausible range count {n}")));
            }
            let mut acc = ECM3_SEED;
            for i in 0..n {
                let (s, e) = (
                    u32_at(flash, table + 8 * i).map_err(err)?,
                    u32_at(flash, table + 8 * i + 4).map_err(err)?,
                );
                let (so, eo) = file_range(s, e).map_err(err)?;
                if (s | e) & 3 != 0 || (at < eo && so < at + 8) {
                    return Err(err(format!("bad range 0x{s:08X}..0x{e:08X}")));
                }
                for w in flash[so..eo].as_chunks::<4>().0 {
                    acc = acc.wrapping_add(u32::from_le_bytes(*w) as u64);
                }
            }
            (
                ((u32_at(flash, at).map_err(err)? as u64) << 32)
                    | u32_at(flash, at + 4).map_err(err)? as u64,
                acc,
            )
        }
    };
    Ok(Some(ChecksumEntry {
        name,
        offset: at,
        stored,
        computed,
    }))
}

fn check_len(flash: &[u8]) -> Result<(), String> {
    if flash.len() != FLASH_LEN {
        return Err(format!(
            "expected a full 4 MB Flash image, got {} bytes",
            flash.len()
        ));
    }
    Ok(())
}

/// `compute`, except that an ECM3 table that is not laid out as expected is reported, not fatal.
fn compute_or_note(flash: &[u8], name: &'static str, at: usize, kind: &Kind, unrecognised: &mut Vec<String>) -> Result<Option<ChecksumEntry>, String> {
    match compute(flash, name, at, kind) {
        Err(e) if matches!(kind, Kind::Ecm3(..)) => {
            unrecognised.push(e);
            Ok(None)
        }
        r => r,
    }
}

pub fn verify_checksums(flash: &[u8]) -> Result<ChecksumReport, String> {
    check_len(flash)?;
    let (mut entries, mut unrecognised) = (Vec::new(), Vec::new());
    for (name, at, kind) in &SPECS {
        entries.extend(compute_or_note(flash, name, *at, kind, &mut unrecognised)?);
    }
    Ok(ChecksumReport { entries, unrecognised })
}

/// Recomputes and rewrites every checksum (ECM3 first, then block CRCs). The returned report lists each
/// checksum as found during the pass: `stored` is the old value and `computed` the value now in Flash,
/// so entries with `!ok()` are exactly the ones rewritten. A consistent image is left byte-identical.
pub fn fix_checksums(flash: &mut [u8]) -> Result<ChecksumReport, String> {
    check_len(flash)?;
    let (mut entries, mut unrecognised) = (Vec::new(), Vec::new());
    for (name, at, kind) in &SPECS {
        let Some(e) = compute_or_note(flash, name, *at, kind, &mut unrecognised)? else {
            continue;
        };
        if !e.ok() {
            match kind {
                Kind::Block(_) => {
                    flash[*at..*at + 4].copy_from_slice(&(e.computed as u32).to_le_bytes())
                }
                Kind::Ecm3(..) => {
                    flash[*at..*at + 4].copy_from_slice(&((e.computed >> 32) as u32).to_le_bytes());
                    flash[*at + 4..*at + 8].copy_from_slice(&(e.computed as u32).to_le_bytes());
                }
            }
        }
        entries.push(e);
    }
    if entries.is_empty() {
        return Err("no valid block headers found in Flash image".into());
    }
    if !verify_checksums(flash)?.ok() {
        return Err("checksums still inconsistent after fix (overlapping ranges?)".into());
    }
    Ok(ChecksumReport { entries, unrecognised })
}
