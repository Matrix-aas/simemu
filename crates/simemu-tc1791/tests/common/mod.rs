#![allow(dead_code)]

use std::path::PathBuf;

pub const ECU_DFLASH: &str = "simos18_dflash.bin";
pub const DQ200_FLASH: &str = "0CW300043H__0002.bin";
pub const DQ200_EEPROM: &str = "0CW300043H_eeprom.bin";
pub const DQ250_011R: &str = "0D9300011R";
pub const DQ250_012F: &str = "0D9300012F";

pub fn search_dirs() -> Vec<PathBuf> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut bases = Vec::new();
    if let Some(env_dir) = std::env::var_os("SIMEMU_IMAGE_DIR") {
        bases.extend(std::env::split_paths(&env_dir));
    }
    bases.extend([root.clone(), root.join("../../..")]);
    let subdirs = [
        "",
        "vault/firmwares/ecu/bench",
        "vault/firmwares/ecu/stock",
        "vault/firmwares/ecu/custom",
        "vault/firmwares/tcu/dq200",
        "vault/firmwares/tcu/dq250",
        "vault/firmwares",
        "firmwares/ecu/bench",
        "firmwares/ecu/stock",
        "firmwares/tcu/dq200",
        "firmwares/tcu/dq250",
        "build",
    ];
    bases
        .into_iter()
        .flat_map(|b| {
            subdirs
                .into_iter()
                .map(move |s| if s.is_empty() { b.clone() } else { b.join(s) })
        })
        .collect()
}

fn find_by_size(subdirs_hint: &[&str], sizes: &[u64]) -> Option<Vec<u8>> {
    for d in search_dirs() {
        let s = d.to_string_lossy();
        if !subdirs_hint.is_empty() && !subdirs_hint.iter().any(|h| s.replace('\\', "/").ends_with(h)) {
            continue;
        }
        let Ok(rd) = std::fs::read_dir(&d) else { continue };
        let mut paths: Vec<PathBuf> = rd
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                p.is_file()
                    && p.extension()
                        .and_then(|e| e.to_str())
                        .is_some_and(|e| e.eq_ignore_ascii_case("bin") || e.eq_ignore_ascii_case("eep"))
                    && std::fs::metadata(p).is_ok_and(|m| sizes.contains(&m.len()))
            })
            .collect();
        paths.sort();
        if let Some(first) = paths.into_iter().next() {
            if let Ok(bytes) = std::fs::read(first) {
                return Some(bytes);
            }
        }
    }
    None
}

fn find_dq250_by_tag(tag: &str) -> Option<Vec<u8>> {
    let needle = tag.as_bytes();
    for d in search_dirs() {
        let Ok(rd) = std::fs::read_dir(&d) else { continue };
        let mut paths: Vec<PathBuf> = rd
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                p.is_file()
                    && p.extension()
                        .and_then(|e| e.to_str())
                        .is_some_and(|e| e.eq_ignore_ascii_case("bin"))
                    && std::fs::metadata(p)
                        .is_ok_and(|m| matches!(m.len(), 0x17_8000 | 0x18_0000))
            })
            .collect();
        paths.sort();
        for p in paths {
            if let Ok(bytes) = std::fs::read(&p) {
                if bytes.windows(needle.len()).any(|w| w.eq_ignore_ascii_case(needle)) {
                    return Some(bytes);
                }
            }
        }
    }
    None
}

pub fn find_image(name: &str) -> Option<Vec<u8>> {
    if let Some(exact) = search_dirs()
        .into_iter()
        .map(|d| d.join(name))
        .find(|p| p.is_file())
        .and_then(|p| std::fs::read(p).ok())
    {
        return Some(exact);
    }
    match name {
        ECU_DFLASH => find_by_size(&["ecu/bench"], &[0x4_0000, 0x1_8000]).or_else(|| {
            let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
            std::fs::read(root.join("rom/dflash.bin")).ok()
        }),
        DQ200_FLASH => find_by_size(&["tcu/dq200"], &[0x2A_0000]),
        DQ200_EEPROM => find_by_size(&["tcu/dq200"], &[0x2000]),
        _ if name.starts_with("0D9300") => find_dq250_by_tag(name),
        _ => None,
    }
}
