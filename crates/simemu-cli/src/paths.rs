//! Unified firmware and workspace path resolution across `simemu-cli`.
//!
//! User-supplied firmware dumps and containers reside inside the gitignored `vault/firmwares/` hierarchy:
//! - `vault/firmwares/ecu/frf/` — official VAG ODIS `.frf` containers (unpacked natively in memory)
//! - `vault/firmwares/ecu/stock/` — clean factory SIMOS18.1 4 MB `.bin` images
//! - `vault/firmwares/ecu/bench/` — full TC1791 bench reads (Flash 4 MB SBOOT donor + DFlash/EEPROM 256 KB)
//! - `vault/firmwares/ecu/custom/` — modified / research SIMOS18.1 reads (`.bin` or `.frf`)
//! - `vault/firmwares/tcu/dq200/` — DSG7 DQ200_G2 (`0CW`) TC1784 micro + ST95640 EEPROM dumps
//! - `vault/firmwares/tcu/dq250/` — DSG6 DQ250 MQB (`0D9`) TC1766 full reads
//!
//! `$SIMEMU_IMAGE_DIR` (if set) overrides or supplements `<root>/vault/firmwares/`.

use std::collections::HashSet;
use std::env;
use std::ffi::OsStr;
use std::fs::{self, File};
use std::io::{Read, Seek, SeekFrom};
use std::path::{Component, Path, PathBuf};

/// Default SIMOS18.1 supplier BootROM (`0x00000..0x1C000`, 112 KiB) shipped in the repository for `.frf`
/// and OBD-only reads when the user has not supplied their own `SBOOT` / bench dump.
pub const DEFAULT_BUNDLED_SBOOT: &str = "rom/sboot.bin";

const EMBEDDED_DEFAULT_SBOOT: &[u8; 0x01_C000] = include_bytes!("../../../rom/sboot.bin");

/// Default scrubbed SIMOS18.1 DFlash / FEE NVM image (`0xAF000000`, 256 KiB; `DF0` = first 96 KiB)
/// shipped in the repository for out-of-the-box `.frf` / OBD runs when the user has not supplied their own bench DFlash.
pub const DEFAULT_BUNDLED_DFLASH: &str = "rom/dflash.bin";

const EMBEDDED_DEFAULT_DFLASH: &[u8; 0x04_0000] = include_bytes!("../../../rom/dflash.bin");

/// Default factory stock SIMOS18.1 Flash image (`SC8O40`, automatically falls back to `FL_8V0906264H__0005.frf`
/// when no pre-unpacked `.bin` is present on disk).
pub const DEFAULT_ECU_FLASH: &str = "8V0906264H__0005.bin";

/// Factory stock 2.0 TSI SIMOS18.1 Flash image (`SC8S50`, automatically falls back to `FL_8V0906264K__0003.frf`).
pub const STOCK_264K_FLASH: &str = "8V0906264K__0003.bin";

/// Logical default SIMOS18.1 DFlash / EEPROM role (`256 KB` or `96 KB` DF0; auto-discovered in `vault/firmwares/ecu/bench/`).
pub const DEFAULT_ECU_EEPROM: &str = "simos18_dflash.bin";

/// Logical default DQ200 Gen2 (`0CW`, TC1784) PFlash role (`0x2A0000` bytes; auto-discovered in `vault/firmwares/tcu/dq200/`).
pub const DEFAULT_DQ200_FLASH: &str = "0CW300043H__0002.bin";

/// Logical default DQ200 Gen2 (`0CW`) ST95640 SPI EEPROM role (`8 KB`; auto-discovered in `vault/firmwares/tcu/dq200/`).
pub const DEFAULT_DQ200_EEPROM: &str = "0CW300043H_eeprom.bin";

/// OEM DQ250 (`0D9`, TC1766) software identifier for 2.0 TSI (`0D9300011R` / SW 4319).
pub const DQ250_011R_ID: &str = "0D9300011R";

/// OEM DQ250 (`0D9`, TC1766) software identifier for 1.8 TSI (`0D9300012F` / SW 4930, transmits `0x0B1` `Getriebe_15`).
pub const DQ250_012F_ID: &str = "0D9300012F";

/// Standard subdirectories searched under `$SIMEMU_IMAGE_DIR` and `<root>`.
pub const FIRMWARE_SUBDIRS: &[&str] = &[
    "",
    "vault/firmwares/ecu/stock",
    "vault/firmwares/ecu/frf",
    "vault/firmwares/ecu/bench",
    "vault/firmwares/ecu/custom",
    "vault/firmwares/ecu",
    "vault/firmwares/tcu/dq200",
    "vault/firmwares/tcu/dq250",
    "vault/firmwares/tcu",
    "vault/firmwares",
    "firmwares/ecu/stock",
    "firmwares/ecu/frf",
    "firmwares/ecu/bench",
    "firmwares/ecu/custom",
    "firmwares/ecu",
    "firmwares/tcu/dq200",
    "firmwares/tcu/dq250",
    "firmwares/tcu",
    "firmwares",
    "ecu/stock",
    "ecu/frf",
    "ecu/bench",
    "ecu/custom",
    "tcu/dq200",
    "tcu/dq250",
];

/// Returns `true` if `ext` is a supported ECU firmware file extension (`.bin`, `.frf`, `.odx`).
pub fn is_ecu_image_ext(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("bin")
        || ext.eq_ignore_ascii_case("frf")
        || ext.eq_ignore_ascii_case("odx")
}

/// Candidate `.frf` container filenames when a caller requests `<stem>.bin` (e.g. `8V0906264H__0005.bin`
/// -> `["FL_8V0906264H__0005.frf", "8V0906264H__0005.frf"]`).
fn frf_fallback_names(name: &str) -> Vec<String> {
    let p = Path::new(name);
    let is_bin = p
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("bin"));
    if !is_bin {
        return Vec::new();
    }
    let Some(stem) = p.file_stem().and_then(|s| s.to_str()).filter(|s| !s.is_empty()) else {
        return Vec::new();
    };
    let lower = stem.to_ascii_lowercase();
    if lower.contains("eeprom") || lower.contains("dflash") || lower.starts_with("dq2") {
        return Vec::new();
    }
    if stem.starts_with("FL_") || stem.starts_with("fl_") {
        vec![format!("{stem}.frf")]
    } else {
        vec![format!("FL_{stem}.frf"), format!("{stem}.frf")]
    }
}

/// Ordered base directories searched for firmware images: `$SIMEMU_IMAGE_DIR` (if set), then `root`.
pub fn search_bases(root: &Path) -> Vec<PathBuf> {
    search_bases_with_env(root, env::var_os("SIMEMU_IMAGE_DIR").as_deref())
}

pub fn search_bases_with_env(root: &Path, image_dir_env: Option<&OsStr>) -> Vec<PathBuf> {
    let mut bases = Vec::with_capacity(2);
    if let Some(val) = image_dir_env.filter(|s| !s.is_empty()) {
        for p in env::split_paths(val) {
            if !p.as_os_str().is_empty() && !bases.contains(&p) {
                bases.push(p);
            }
        }
    }
    if !bases.iter().any(|b| b == root) {
        bases.push(root.to_path_buf());
    }
    bases
}

fn find_exact_in_bases(bases: &[PathBuf], target: &str) -> Option<PathBuf> {
    let req_path = Path::new(target);
    for base in bases {
        for sub in FIRMWARE_SUBDIRS {
            let candidate = if sub.is_empty() {
                base.join(target)
            } else {
                base.join(sub).join(target)
            };
            if candidate.is_file() {
                return Some(candidate);
            }
        }
        if let Some(fname) = req_path.file_name().filter(|&f| f != OsStr::new(target)) {
            for sub in FIRMWARE_SUBDIRS {
                let candidate = if sub.is_empty() {
                    base.join(fname)
                } else {
                    base.join(sub).join(fname)
                };
                if candidate.is_file() {
                    return Some(candidate);
                }
            }
        }
    }
    None
}

/// Resolves a firmware image `name` across `bases` and [`FIRMWARE_SUBDIRS`], falling back to matching
/// `FL_<stem>.frf` containers when a stock `.bin` has not been unpacked on disk.
pub fn resolve_in_bases(bases: &[PathBuf], name: &str) -> Option<PathBuf> {
    let req_path = Path::new(name);
    if req_path.is_absolute() && req_path.is_file() {
        return Some(req_path.to_path_buf());
    }
    if let Some(found) = find_exact_in_bases(bases, name) {
        return Some(found);
    }
    if req_path.is_file() {
        return Some(req_path.to_path_buf());
    }
    for alt in frf_fallback_names(name) {
        if let Some(found) = find_exact_in_bases(bases, &alt) {
            return Some(found);
        }
    }
    // Generic size/extension auto-discovery so external users can place differently named FRF/bench/DQ200 files
    // inside `vault/firmwares/**` without renaming them.
    match name {
        DEFAULT_ECU_FLASH => find_first_by_size(
            bases,
            &["vault/firmwares/ecu/stock", "firmwares/ecu/stock"],
            Some(&[0x40_0000]),
            &["bin"],
        )
        .or_else(|| {
            find_first_by_size(
                bases,
                &["vault/firmwares/ecu/frf", "firmwares/ecu/frf", "ecu/frf"],
                None,
                &["frf"],
            )
        })
        .or_else(|| {
            find_first_by_size(
                bases,
                &["vault/firmwares/ecu/custom", "firmwares/ecu/custom", "ecu/custom"],
                None,
                &["frf", "odx"],
            )
        })
        .or_else(|| {
            find_first_by_size(
                bases,
                &[
                    "vault/firmwares/ecu/custom",
                    "firmwares/ecu/custom",
                    "vault/firmwares/ecu/bench",
                    "firmwares/ecu/bench",
                ],
                Some(&[0x40_0000, 0x30_0000]),
                &["bin"],
            )
        }),
        DEFAULT_ECU_EEPROM => find_first_by_size(
            bases,
            &["vault/firmwares/ecu/bench", "firmwares/ecu/bench", "ecu/bench"],
            Some(&[0x4_0000, 0x1_8000]),
            &["bin", "eep"],
        )
        .or_else(|| {
            bases
                .iter()
                .map(|b| b.join(DEFAULT_BUNDLED_DFLASH))
                .find(|p| p.is_file())
        }),
        DEFAULT_DQ200_FLASH => find_first_by_size(
            bases,
            &["vault/firmwares/tcu/dq200", "firmwares/tcu/dq200"],
            Some(&[0x2A_0000]),
            &["bin"],
        ),
        DEFAULT_DQ200_EEPROM => find_first_by_size(
            bases,
            &["vault/firmwares/tcu/dq200", "firmwares/tcu/dq200"],
            Some(&[0x2000]),
            &["bin", "eep"],
        ),
        _ => None,
    }
}

fn find_first_by_size(
    bases: &[PathBuf],
    subdirs: &[&str],
    sizes: Option<&[u64]>,
    exts: &[&str],
) -> Option<PathBuf> {
    for base in bases {
        for sub in subdirs {
            let dir = if sub.is_empty() { base.clone() } else { base.join(sub) };
            let Ok(rd) = fs::read_dir(&dir) else { continue };
            let mut cands: Vec<PathBuf> = rd
                .flatten()
                .map(|e| e.path())
                .filter(|p| {
                    p.is_file()
                        && p.extension()
                            .and_then(|e| e.to_str())
                            .is_some_and(|e| exts.iter().any(|x| e.eq_ignore_ascii_case(x)))
                        && match sizes {
                            Some(sz) => fs::metadata(p).is_ok_and(|m| sz.contains(&m.len())),
                            None => true,
                        }
                })
                .collect();
            cands.sort();
            if let Some(first) = cands.into_iter().next() {
                return Some(first);
            }
        }
    }
    None
}

/// Returns `true` if `name` is a safe relative `.bin` / `.frf` / `.odx` firmware path without parent
/// traversal, root, or drive prefix.
pub fn is_safe_relative_firmware_name(name: &str) -> bool {
    let trimmed = name.trim();
    if trimmed.is_empty() || trimmed.contains('\0') {
        return false;
    }
    let req = Path::new(trimmed);
    let has_valid_ext = req
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(is_ecu_image_ext);
    if !has_valid_ext {
        return false;
    }
    let mut saw_normal = false;
    for comp in req.components() {
        match comp {
            Component::Normal(_) => saw_normal = true,
            Component::CurDir => {}
            Component::ParentDir | Component::RootDir | Component::Prefix(_) => return false,
        }
    }
    saw_normal
}

/// Resolves a firmware image strictly inside `bases` and [`FIRMWARE_SUBDIRS`], rejecting absolute paths,
/// `..` traversal, unsupported extensions, CWD fallbacks, and symlinks escaping outside `bases`.
pub fn resolve_sandboxed_in_bases(bases: &[PathBuf], name: &str) -> Result<PathBuf, String> {
    if !is_safe_relative_firmware_name(name) {
        return Err(format!(
            "invalid firmware image name {name:?} (must be a relative .bin/.frf/.odx path without '..')"
        ));
    }
    let trimmed = name.trim();
    let canon_bases: Vec<PathBuf> = bases
        .iter()
        .filter_map(|b| fs::canonicalize(b).ok())
        .collect();
    let verify_inside = |cand: PathBuf| -> Option<PathBuf> {
        if !cand.is_file() {
            return None;
        }
        let canon = fs::canonicalize(&cand).ok()?;
        canon_bases
            .iter()
            .any(|cb| canon.starts_with(cb))
            .then_some(cand)
    };
    let search_one = |query: &str| -> Option<PathBuf> {
        let req_path = Path::new(query);
        for base in bases {
            for sub in FIRMWARE_SUBDIRS {
                let cand = if sub.is_empty() {
                    base.join(query)
                } else {
                    base.join(sub).join(query)
                };
                if let Some(ok) = verify_inside(cand) {
                    return Some(ok);
                }
            }
            if let Some(fname) = req_path.file_name().filter(|&f| f != OsStr::new(query)) {
                for sub in FIRMWARE_SUBDIRS {
                    let cand = if sub.is_empty() {
                        base.join(fname)
                    } else {
                        base.join(sub).join(fname)
                    };
                    if let Some(ok) = verify_inside(cand) {
                        return Some(ok);
                    }
                }
            }
        }
        None
    };
    if let Some(ok) = search_one(trimmed) {
        return Ok(ok);
    }
    for alt in frf_fallback_names(trimmed) {
        if let Some(ok) = search_one(&alt) {
            return Ok(ok);
        }
    }
    Err(format!("{trimmed}: firmware file not found in workspace firmware directories"))
}

/// Resolves a firmware image strictly inside `$SIMEMU_IMAGE_DIR` / `root` firmware directories (used by HTTP APIs).
pub fn resolve_sandboxed_firmware(root: &Path, name: &str) -> Result<PathBuf, String> {
    resolve_sandboxed_in_bases(&search_bases(root), name)
}

/// Returns `true` if `name` is a valid sandboxed `.bin` / `.frf` / `.odx` image inside `$SIMEMU_IMAGE_DIR` / `root`.
pub fn has_sandboxed_firmware(root: &Path, name: &str) -> bool {
    resolve_sandboxed_firmware(root, name).is_ok()
}

/// Resolves `name` searching `$SIMEMU_IMAGE_DIR`, `<root>/vault/firmwares/**`, and `<root>/<name>`.
pub fn resolve_firmware(root: &Path, name: &str) -> Option<PathBuf> {
    resolve_in_bases(&search_bases(root), name)
}

/// Returns `true` if `name` resolves to an existing firmware file.
pub fn has_firmware(root: &Path, name: &str) -> bool {
    resolve_firmware(root, name).is_some()
}

/// Resolves `name` or returns a descriptive error listing searched locations.
pub fn resolve_firmware_or_err(root: &Path, name: &str) -> Result<PathBuf, String> {
    resolve_firmware(root, name).ok_or_else(|| {
        let ru = simemu_damos::is_ru();
        match name {
            DEFAULT_ECU_FLASH if ru => format!(
                "Прошивка ЭБУ SIMOS18.1 (.frf или .bin) не найдена (положите образ в {0}/vault/firmwares/ecu/{{frf,stock,custom}}/ или задайте $SIMEMU_IMAGE_DIR)",
                root.display()
            ),
            DEFAULT_ECU_FLASH => format!(
                "SIMOS18.1 ECU firmware (.frf or .bin) not found (place an image in {0}/vault/firmwares/ecu/{{frf,stock,custom}}/ or set $SIMEMU_IMAGE_DIR)",
                root.display()
            ),
            DEFAULT_ECU_EEPROM if ru => format!(
                "Дамп DFlash/EEPROM ЭБУ SIMOS18.1 (256 КБ / 96 КБ) не найден в {0}/vault/firmwares/ecu/bench/",
                root.display()
            ),
            DEFAULT_ECU_EEPROM => format!(
                "SIMOS18.1 ECU DFlash/EEPROM (256 KB / 96 KB) not found in {0}/vault/firmwares/ecu/bench/",
                root.display()
            ),
            DEFAULT_DQ200_FLASH if ru => format!(
                "Прошивка ТКУ DQ200_G2 (0CW, 2.625 МБ) не найдена в {0}/vault/firmwares/tcu/dq200/",
                root.display()
            ),
            DEFAULT_DQ200_FLASH => format!(
                "DQ200_G2 (0CW, 2.625 MB) TCU PFlash not found in {0}/vault/firmwares/tcu/dq200/",
                root.display()
            ),
            DEFAULT_DQ200_EEPROM if ru => format!(
                "EEPROM ТКУ DQ200_G2 (0CW, 8 КБ) не найден в {0}/vault/firmwares/tcu/dq200/",
                root.display()
            ),
            DEFAULT_DQ200_EEPROM => format!(
                "DQ200_G2 (0CW, 8 KB) TCU EEPROM not found in {0}/vault/firmwares/tcu/dq200/",
                root.display()
            ),
            _ if ru => format!(
                "{name}: файл прошивки не найден (проверены $SIMEMU_IMAGE_DIR, {0}/vault/firmwares/{{ecu/{{frf,stock,bench,custom}},tcu/{{dq200,dq250}}}} и {0})",
                root.display()
            ),
            _ => format!(
                "{name}: firmware file not found (searched $SIMEMU_IMAGE_DIR, {0}/vault/firmwares/{{ecu/{{frf,stock,bench,custom}},tcu/{{dq200,dq250}}}}, and {0})",
                root.display()
            ),
        }
    })
}

/// Reads a firmware image from `path` into memory, transparently unpacking VAG ODIS `.frf` and `.odx`
/// containers into a 4 MB SIMOS18.1 PFlash image.
///
/// **SBOOT priority**:
/// 1. If `path` is a `.bin` dump that already contains a non-empty `SBOOT` (`0x0000..0x1000`), its own
///    `SBOOT` is preserved untouched.
/// 2. Otherwise (`.frf`, `.odx`, or OBD-only `.bin`), [`read_sboot_donor`] supplies `SBOOT` from the user's
///    custom `sboot.bin` / bench dump in `vault/` if present, else the bundled default `rom/sboot.bin`.
pub fn read_firmware_from_path(root: &Path, path: &Path) -> Result<Vec<u8>, String> {
    let mut raw = fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if simemu_damos::is_frf_path(path) || simemu_damos::is_frf_bytes(&raw) {
        let donor = read_sboot_donor(root).ok().map(|(_, b)| b);
        let (img, _) = simemu_damos::unpack_frf_into_image(&raw, donor.as_deref())
            .map_err(|e| format!("{}: {e}", path.display()))?;
        return Ok(img);
    }
    if simemu_damos::is_odx_path(path) {
        let donor = read_sboot_donor(root).ok().map(|(_, b)| b);
        let xml = String::from_utf8(raw)
            .unwrap_or_else(|e| String::from_utf8_lossy(&e.into_bytes()).into_owned());
        let odx_name = path
            .file_name()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| path.display().to_string());
        let (img, _) = simemu_damos::unpack_odx_into_image(&xml, &odx_name, donor.as_deref())
            .map_err(|e| format!("{}: {e}", path.display()))?;
        return Ok(img);
    }
    if raw.len() == 0x30_0000 {
        raw.resize(0x40_0000, 0x00);
    }
    if raw.len() == 0x40_0000 && raw[..0x01_C000].iter().all(|&b| b == 0x00 || b == 0xFF) {
        if let Ok((_, donor)) = read_sboot_donor(root) {
            raw[..0x01_C000].copy_from_slice(&donor[..0x01_C000]);
        }
    }
    Ok(raw)
}

/// Resolves and reads `name` into memory (natively unpacking `.frf` / `.odx` containers when applicable).
pub fn read_firmware(root: &Path, name: &str) -> Result<Vec<u8>, String> {
    if name == DEFAULT_ECU_EEPROM {
        return read_dflash_or_default(root).map(|(_, b)| b);
    }
    let path = resolve_firmware_or_err(root, name)?;
    read_firmware_from_path(root, &path)
}

/// Fast peek of the first `prefix_len` bytes of the SIMOS18.1 `CAL` sector (`0x200000..`) from a 4 MB `.bin`,
/// `.frf`, or `.odx` file without decompressing `ASW1..3` or reading the entire 4 MB image.
pub fn peek_ecu_cal_header(path: &Path, prefix_len: usize) -> Option<Vec<u8>> {
    if prefix_len == 0 || prefix_len > 0x07_FC00 {
        return None;
    }
    if simemu_damos::is_frf_path(path) {
        let raw = fs::read(path).ok()?;
        return simemu_damos::peek_frf_cal_prefix(&raw, prefix_len).ok();
    }
    if simemu_damos::is_odx_path(path) {
        let raw = fs::read(path).ok()?;
        let xml = String::from_utf8(raw)
            .unwrap_or_else(|e| String::from_utf8_lossy(&e.into_bytes()).into_owned());
        return simemu_damos::peek_odx_cal_prefix(&xml, prefix_len).ok();
    }
    let meta = fs::metadata(path).ok()?;
    if meta.len() != 0x40_0000 && meta.len() != 0x30_0000 {
        return None;
    }
    let mut f = File::open(path).ok()?;
    f.seek(SeekFrom::Start(0x20_0000)).ok()?;
    let mut buf = vec![0u8; prefix_len];
    f.read_exact(&mut buf).ok()?;
    Some(buf)
}

fn has_valid_sboot_bytes(bytes: &[u8]) -> bool {
    matches!(bytes.len(), 0x01_C000 | 0x30_0000 | 0x40_0000)
        && bytes[..0x01_C000].iter().any(|&b| b != 0x00 && b != 0xFF)
}

/// Resolves the `SBOOT` (`0x00000..0x1C000`) image for `.frf`/`.odx` containers and OBD-only `.bin` reads
/// that do not carry their own `SBOOT` (dumps that already contain `SBOOT` never call this override).
///
/// Priority:
/// 1. User's explicit `sboot.bin` in `$SIMEMU_IMAGE_DIR` or `vault/firmwares/ecu/{bench,}/sboot.bin` / `<root>/sboot.bin`.
/// 2. User's full 4 MB / 3 MB bench read in `$SIMEMU_IMAGE_DIR` or `vault/firmwares/ecu/bench/*.bin`.
/// 3. Bundled default `<root>/rom/sboot.bin` on disk (replaceable by the user without recompiling).
/// 4. Compile-time embedded `rom/sboot.bin` fallback (when running standalone binary outside the repo).
pub fn read_sboot_donor(root: &Path) -> Result<(PathBuf, Vec<u8>), String> {
    read_sboot_donor_with_bases(&search_bases(root), root, true)
}

fn read_sboot_donor_with_bases(
    bases: &[PathBuf],
    root: &Path,
    allow_embedded_fallback: bool,
) -> Result<(PathBuf, Vec<u8>), String> {
    let mut candidates = Vec::new();

    // 1. User's explicit `sboot.bin` override in workspace / $SIMEMU_IMAGE_DIR
    for base in bases {
        for sub in [
            "vault/firmwares/ecu/bench",
            "vault/firmwares/ecu",
            "firmwares/ecu/bench",
            "firmwares/ecu",
            "ecu/bench",
            "",
        ] {
            let p = if sub.is_empty() {
                base.join("sboot.bin")
            } else {
                base.join(sub).join("sboot.bin")
            };
            if p.is_file() {
                candidates.push(p);
            }
        }
    }

    // 2. User's full bench dump in `ecu/bench/`
    for base in bases {
        for sub in ["vault/firmwares/ecu/bench", "firmwares/ecu/bench", "ecu/bench"] {
            let dir = base.join(sub);
            let Ok(rd) = fs::read_dir(&dir) else { continue };
            let mut bins: Vec<PathBuf> = rd
                .flatten()
                .map(|e| e.path())
                .filter(|p| {
                    p.is_file()
                        && p.extension()
                            .and_then(|e| e.to_str())
                            .is_some_and(|e| e.eq_ignore_ascii_case("bin"))
                })
                .collect();
            bins.sort();
            candidates.extend(bins);
        }
    }

    // 3. Bundled default `<root>/rom/sboot.bin` on disk
    let repo_rom = root.join(DEFAULT_BUNDLED_SBOOT);
    if repo_rom.is_file() {
        candidates.push(repo_rom);
    }

    for path in candidates {
        if simemu_damos::is_frf_path(&path) || simemu_damos::is_odx_path(&path) {
            continue;
        }
        let Ok(meta) = fs::metadata(&path) else { continue };
        if !matches!(meta.len(), 0x01_C000 | 0x30_0000 | 0x40_0000) {
            continue;
        }
        let bytes = fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        if has_valid_sboot_bytes(&bytes) {
            return Ok((path, bytes));
        }
    }

    // 4. Built-in embedded `rom/sboot.bin` when running standalone outside repo tree
    if allow_embedded_fallback {
        return Ok((
            PathBuf::from("rom/sboot.bin (built-in)"),
            EMBEDDED_DEFAULT_SBOOT.to_vec(),
        ));
    }
    Err("SBOOT donor not found".into())
}

/// Resolves the SIMOS18.1 DFlash / FEE NVM image (`256 KiB` at `0xAF000000`, `DF0` = first `96 KiB`).
///
/// Priority:
/// 1. User's own DFlash dump (`256 KiB` or `96 KiB`) in `$SIMEMU_IMAGE_DIR` or `vault/firmwares/ecu/bench/`.
/// 2. Bundled clean default `<root>/rom/dflash.bin` on disk (replaceable by the user without recompiling).
/// 3. Compile-time embedded `rom/dflash.bin` fallback (when running standalone binary outside the repo).
pub fn read_dflash_or_default(root: &Path) -> Result<(PathBuf, Vec<u8>), String> {
    read_dflash_with_bases(&search_bases(root), root, true)
}

fn read_dflash_with_bases(
    bases: &[PathBuf],
    root: &Path,
    allow_embedded_fallback: bool,
) -> Result<(PathBuf, Vec<u8>), String> {
    let normalize = |mut b: Vec<u8>| -> Vec<u8> {
        if b.len() == 0x01_8000 {
            b.resize(0x04_0000, 0x00);
        }
        b
    };

    // 1. User's explicit `simos18_dflash.bin` / `dflash.bin` or any 256 KB / 96 KB dump in `ecu/bench/`
    if let Some(exact) = find_exact_in_bases(bases, DEFAULT_ECU_EEPROM)
        .or_else(|| find_exact_in_bases(bases, "dflash.bin"))
        .or_else(|| {
            find_first_by_size(
                bases,
                &["vault/firmwares/ecu/bench", "firmwares/ecu/bench", "ecu/bench"],
                Some(&[0x04_0000, 0x01_8000]),
                &["bin", "eep"],
            )
        })
    {
        let bytes = fs::read(&exact).map_err(|e| format!("{}: {e}", exact.display()))?;
        if matches!(bytes.len(), 0x04_0000 | 0x01_8000) {
            return Ok((exact, normalize(bytes)));
        }
    }

    // 2. Bundled `<root>/rom/dflash.bin` on disk
    for base in bases
        .iter()
        .cloned()
        .chain(std::iter::once(root.to_path_buf()))
    {
        let p = base.join(DEFAULT_BUNDLED_DFLASH);
        if p.is_file() {
            let bytes = fs::read(&p).map_err(|e| format!("{}: {e}", p.display()))?;
            if matches!(bytes.len(), 0x04_0000 | 0x01_8000) {
                return Ok((p, normalize(bytes)));
            }
        }
    }

    // 3. Built-in embedded `rom/dflash.bin` fallback
    if allow_embedded_fallback {
        return Ok((
            PathBuf::from("rom/dflash.bin (built-in)"),
            EMBEDDED_DEFAULT_DFLASH.to_vec(),
        ));
    }
    Err("SIMOS18.1 DFlash image not found".into())
}

fn list_with_filter(root: &Path, subdir: &str, accept_ext: fn(&str) -> bool) -> Vec<(String, PathBuf)> {
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for base in search_bases(root) {
        let dir = if subdir.is_empty() { base } else { base.join(subdir) };
        let Ok(rd) = fs::read_dir(&dir) else { continue };
        let mut entries: Vec<(String, PathBuf)> = rd
            .flatten()
            .filter_map(|e| {
                let path = e.path();
                if !path.is_file() {
                    return None;
                }
                let ext_ok = path
                    .extension()
                    .and_then(|x| x.to_str())
                    .is_some_and(accept_ext);
                if !ext_ok {
                    return None;
                }
                let name = e.file_name().to_string_lossy().into_owned();
                if name.eq_ignore_ascii_case("sboot.bin")
                    || name.eq_ignore_ascii_case("dflash.bin")
                    || name.eq_ignore_ascii_case("simos18_dflash.bin")
                {
                    return None;
                }
                Some((name, path))
            })
            .collect();
        entries.sort_by(|a, b| a.0.cmp(&b.0));
        for (name, path) in entries {
            if seen.insert(name.clone()) {
                out.push((name, path));
            }
        }
    }
    out
}

/// Collects unique `.bin` file names across `<base>/<subdir>` for all search bases of `root`, sorted deterministically.
pub fn list_bins_in_subdir(root: &Path, subdir: &str) -> Vec<(String, PathBuf)> {
    list_with_filter(root, subdir, |ext| ext.eq_ignore_ascii_case("bin"))
}

/// Collects unique ECU image file names (`.bin`, `.frf`, `.odx`) across `<base>/<subdir>` for all search bases of `root`.
pub fn list_ecu_images_in_subdir(root: &Path, subdir: &str) -> Vec<(String, PathBuf)> {
    list_with_filter(root, subdir, is_ecu_image_ext)
}

fn looks_like_workspace_root(dir: &Path) -> bool {
    dir.join("vault/firmwares").is_dir()
        || (dir.join("definitions").is_dir() && dir.join("Cargo.toml").is_file())
        || (dir.join("crates/simemu-cli").is_dir() && dir.join("Cargo.toml").is_file())
        || dir.join(DEFAULT_ECU_FLASH).is_file()
        || dir.join("FL_8V0906264H__0005.frf").is_file()
}

/// Locates the repository or standalone release root from the current directory, executable directory,
/// or compile-time manifest ancestor.
pub fn find_workspace_root() -> PathBuf {
    let cwd = env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    if let Some(found) = cwd.ancestors().find(|d| looks_like_workspace_root(d)) {
        return found.to_path_buf();
    }
    if let Some(exe_dir) = env::current_exe().ok().and_then(|p| p.parent().map(PathBuf::from)) {
        if let Some(found) = exe_dir.ancestors().find(|d| looks_like_workspace_root(d)) {
            return found.to_path_buf();
        }
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .filter(|d| d.exists())
        .map(PathBuf::from)
        .unwrap_or(cwd)
}

/// Locates a workspace root for tests that require the bench SIMOS18.1 Flash image (`DEFAULT_ECU_FLASH`),
/// honoring `$SIMEMU_IMAGE_DIR` (including when running inside a git worktree whose gitignored images live
/// in `$SIMEMU_IMAGE_DIR`).
pub fn test_workspace_root() -> Option<PathBuf> {
    let manifest_ws = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .map(PathBuf::from);
    if let Some(ws) = &manifest_ws {
        if has_firmware(ws, DEFAULT_ECU_FLASH) {
            return Some(ws.clone());
        }
    }
    let mut dirs: Vec<PathBuf> = env::var_os("SIMEMU_IMAGE_DIR")
        .into_iter()
        .flat_map(|v| env::split_paths(&v).collect::<Vec<_>>())
        .collect();
    dirs.extend(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .map(PathBuf::from),
    );
    dirs.into_iter().find(|d| has_firmware(d, DEFAULT_ECU_FLASH))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_tree(tag: &str) -> PathBuf {
        let dir = env::temp_dir().join(format!("simemu_paths_{tag}_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn resolves_across_vault_subdirs_frf_fallback_and_env_override() {
        let root = temp_tree("root");
        let override_dir = temp_tree("override");

        let bench_dir = root.join("vault/firmwares/ecu/bench");
        let stock_dir = root.join("vault/firmwares/ecu/stock");
        let frf_dir = root.join("vault/firmwares/ecu/frf");
        let dq200_dir = root.join("vault/firmwares/tcu/dq200");
        let dq250_dir = root.join("vault/firmwares/tcu/dq250");
        for d in [&bench_dir, &stock_dir, &frf_dir, &dq200_dir, &dq250_dir, &override_dir] {
            fs::create_dir_all(d).unwrap();
        }

        fs::write(root.join("root_only.bin"), b"root").unwrap();
        fs::write(bench_dir.join("ecu_dump.bin"), b"ecu_root").unwrap();
        fs::write(dq200_dir.join("tcu_dump.bin"), b"tcu").unwrap();
        fs::write(stock_dir.join("8V0906264H__0005.bin"), b"frf").unwrap();
        fs::write(frf_dir.join("FL_8V0906264J__0003.frf"), b"frf_j").unwrap();
        fs::write(dq250_dir.join("dq250_read.bin"), b"dq250").unwrap();
        fs::write(override_dir.join("ecu_dump.bin"), b"ecu_override").unwrap();

        let bases_no_env = search_bases_with_env(&root, None);
        assert_eq!(
            resolve_in_bases(&bases_no_env, "root_only.bin").unwrap(),
            root.join("root_only.bin")
        );
        assert_eq!(
            resolve_in_bases(&bases_no_env, "ecu_dump.bin").unwrap(),
            bench_dir.join("ecu_dump.bin")
        );
        assert_eq!(
            resolve_in_bases(&bases_no_env, "tcu_dump.bin").unwrap(),
            dq200_dir.join("tcu_dump.bin")
        );
        assert_eq!(
            resolve_in_bases(&bases_no_env, "8V0906264H__0005.bin").unwrap(),
            stock_dir.join("8V0906264H__0005.bin")
        );
        // Requesting `8V0906264J__0003.bin` when only `FL_8V0906264J__0003.frf` exists resolves directly to the `.frf`!
        assert_eq!(
            resolve_in_bases(&bases_no_env, "8V0906264J__0003.bin").unwrap(),
            frf_dir.join("FL_8V0906264J__0003.frf")
        );
        assert_eq!(
            resolve_sandboxed_in_bases(&bases_no_env, "FL_8V0906264J__0003.frf").unwrap(),
            frf_dir.join("FL_8V0906264J__0003.frf")
        );
        assert_eq!(
            resolve_sandboxed_in_bases(&bases_no_env, "8V0906264J__0003.bin").unwrap(),
            frf_dir.join("FL_8V0906264J__0003.frf")
        );
        assert_eq!(
            resolve_in_bases(&bases_no_env, "vault/firmwares/ecu/stock/8V0906264H__0005.bin").unwrap(),
            stock_dir.join("8V0906264H__0005.bin")
        );
        assert_eq!(
            resolve_in_bases(&bases_no_env, "dq250_read.bin").unwrap(),
            dq250_dir.join("dq250_read.bin")
        );
        assert!(resolve_in_bases(&bases_no_env, "missing.bin").is_none());

        // $SIMEMU_IMAGE_DIR takes priority over root
        fs::write(override_dir.join("8V0906264K__0003.bin"), b"flat_override").unwrap();
        let bases_with_env = search_bases_with_env(&root, Some(override_dir.as_os_str()));
        assert_eq!(
            fs::read(resolve_in_bases(&bases_with_env, "ecu_dump.bin").unwrap()).unwrap(),
            b"ecu_override"
        );
        assert_eq!(
            fs::read(resolve_in_bases(&bases_with_env, "8V0906264K__0003.bin").unwrap()).unwrap(),
            b"flat_override"
        );

        let _ = fs::remove_dir_all(&root);
        let _ = fs::remove_dir_all(&override_dir);
    }

    #[test]
    fn sandboxed_resolution_blocks_traversal_absolute_and_unsupported_paths() {
        let root = temp_tree("sandbox_root");
        let outside = temp_tree("sandbox_outside");
        let bench_dir = root.join("vault/firmwares/ecu/bench");
        let frf_dir = root.join("vault/firmwares/ecu/frf");
        fs::create_dir_all(&bench_dir).unwrap();
        fs::create_dir_all(&frf_dir).unwrap();
        fs::write(bench_dir.join("valid_ecu.bin"), b"ok").unwrap();
        fs::write(frf_dir.join("FL_8V0906264H__0005.frf"), b"frf").unwrap();
        fs::write(root.join("Cargo.toml"), b"[workspace]").unwrap();
        fs::write(outside.join("secret.bin"), b"secret").unwrap();

        let bases = search_bases_with_env(&root, None);
        assert!(resolve_sandboxed_in_bases(&bases, "valid_ecu.bin").is_ok());
        assert!(resolve_sandboxed_in_bases(&bases, "FL_8V0906264H__0005.frf").is_ok());
        assert!(resolve_sandboxed_in_bases(&bases, "8V0906264H__0005.bin").is_ok());
        assert!(resolve_sandboxed_in_bases(&bases, "vault/firmwares/ecu/frf/FL_8V0906264H__0005.frf").is_ok());

        let abs_secret = outside.join("secret.bin");
        let rel_traversal = format!("../{}/secret.bin", outside.file_name().unwrap().to_string_lossy());
        for bad in [
            "",
            "Cargo.toml",
            "../secret.bin",
            "../secret.frf",
            "vault/../../secret.bin",
            rel_traversal.as_str(),
            abs_secret.to_str().unwrap(),
        ] {
            assert!(
                resolve_sandboxed_in_bases(&bases, bad).is_err(),
                "expected {bad:?} to be rejected by sandbox"
            );
        }

        let _ = fs::remove_dir_all(&root);
        let _ = fs::remove_dir_all(&outside);
    }

    #[test]
    fn workspace_root_detects_clean_clone_without_vault() {
        let ws = find_workspace_root();
        assert!(
            ws.join("definitions").is_dir() && ws.join("Cargo.toml").is_file(),
            "unexpected workspace root: {}",
            ws.display()
        );
    }

    #[test]
    fn sboot_priority_prefers_loaded_dump_then_user_sboot_then_bench_then_bundled_rom() {
        let root = temp_tree("sboot_prio");
        let bench_dir = root.join("vault/firmwares/ecu/bench");
        let custom_dir = root.join("vault/firmwares/ecu/custom");
        let rom_dir = root.join("rom");
        for d in [&bench_dir, &custom_dir, &rom_dir] {
            fs::create_dir_all(d).unwrap();
        }

        // Bundled `rom/sboot.bin` on disk (`0xCC`), user bench 4 MB dump (`0xBB`), user explicit `sboot.bin` (`0xDD`).
        fs::write(rom_dir.join("sboot.bin"), vec![0xCCu8; 0x01_C000]).unwrap();
        let mut bench_dump = vec![0x00u8; 0x40_0000];
        bench_dump[..0x01_C000].fill(0xBB);
        fs::write(bench_dir.join("user_bench.bin"), &bench_dump).unwrap();
        fs::write(bench_dir.join("sboot.bin"), vec![0xDDu8; 0x01_C000]).unwrap();

        // 1. User's loaded `.bin` (4 MB or 3 MB BSL) that ALREADY contains its own `SBOOT` (`0xAA`) MUST keep `0xAA` untouched!
        let mut full_user_dump = vec![0x11u8; 0x40_0000];
        full_user_dump[..0x01_C000].fill(0xAA);
        let full_path = custom_dir.join("my_full_bsl_read.bin");
        fs::write(&full_path, &full_user_dump).unwrap();
        let loaded_full = read_firmware_from_path(&root, &full_path).unwrap();
        assert!(loaded_full[..0x01_C000].iter().all(|&b| b == 0xAA));
        assert_eq!(loaded_full[0x01_C000], 0x11);

        let mut bsl_3mb = vec![0x22u8; 0x30_0000];
        bsl_3mb[..0x01_C000].fill(0xAE);
        let bsl_path = custom_dir.join("my_3mb_bsl.bin");
        fs::write(&bsl_path, &bsl_3mb).unwrap();
        let loaded_bsl = read_firmware_from_path(&root, &bsl_path).unwrap();
        assert_eq!(loaded_bsl.len(), 0x40_0000);
        assert!(loaded_bsl[..0x01_C000].iter().all(|&b| b == 0xAE));

        // 2. OBD-only `.bin` (empty `0xFF` SBOOT): user's explicit `sboot.bin` (`0xDD`) wins first.
        let mut obd_dump = vec![0x33u8; 0x40_0000];
        obd_dump[..0x01_C000].fill(0xFF);
        let obd_path = custom_dir.join("my_obd_read.bin");
        fs::write(&obd_path, &obd_dump).unwrap();

        let bases = search_bases_with_env(&root, None);
        let (_, d1) = read_sboot_donor_with_bases(&bases, &root, false).unwrap();
        assert!(d1[..0x01_C000].iter().all(|&b| b == 0xDD));
        let loaded_obd_1 = read_firmware_from_path(&root, &obd_path).unwrap();
        assert!(loaded_obd_1[..0x01_C000].iter().all(|&b| b == 0xDD));
        assert_eq!(loaded_obd_1[0x01_C000], 0x33);

        // 3. Without explicit `sboot.bin`, user's bench dump (`0xBB`) wins over bundled `rom/sboot.bin`.
        fs::remove_file(bench_dir.join("sboot.bin")).unwrap();
        let (_, d2) = read_sboot_donor_with_bases(&bases, &root, false).unwrap();
        assert!(d2[..0x01_C000].iter().all(|&b| b == 0xBB));

        // 4. Without any user bench dump, `<root>/rom/sboot.bin` (`0xCC`) on disk is used (user can replace it).
        fs::remove_file(bench_dir.join("user_bench.bin")).unwrap();
        let (_, d3) = read_sboot_donor_with_bases(&bases, &root, false).unwrap();
        assert!(d3[..0x01_C000].iter().all(|&b| b == 0xCC));

        // 5. Standalone binary outside repo tree: falls back to built-in `EMBEDDED_DEFAULT_SBOOT` (112 KiB).
        fs::remove_file(rom_dir.join("sboot.bin")).unwrap();
        assert!(read_sboot_donor_with_bases(&bases, &root, false).is_err());
        let (_, d4) = read_sboot_donor_with_bases(&bases, &root, true).unwrap();
        assert_eq!(d4.len(), 0x01_C000);
        assert_eq!(&d4[..], &EMBEDDED_DEFAULT_SBOOT[..]);

        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn dflash_priority_prefers_user_bench_dflash_then_bundled_rom_then_embedded() {
        let root = temp_tree("dflash_prio");
        let bench_dir = root.join("vault/firmwares/ecu/bench");
        let rom_dir = root.join("rom");
        fs::create_dir_all(&bench_dir).unwrap();
        fs::create_dir_all(&rom_dir).unwrap();

        fs::write(rom_dir.join("dflash.bin"), vec![0xCCu8; 0x04_0000]).unwrap();
        fs::write(bench_dir.join("my_bench_df0.bin"), vec![0xBBu8; 0x01_8000]).unwrap();

        let bases = search_bases_with_env(&root, None);
        // 1. User's bench DFlash (even 96 KiB DF0) wins over `rom/dflash.bin` and is zero-padded to 256 KiB.
        let (p1, d1) = read_dflash_with_bases(&bases, &root, false).unwrap();
        assert_eq!(p1, bench_dir.join("my_bench_df0.bin"));
        assert_eq!(d1.len(), 0x04_0000);
        assert!(d1[..0x01_8000].iter().all(|&b| b == 0xBB));
        assert!(d1[0x01_8000..].iter().all(|&b| b == 0x00));

        // 2. Without a user bench DFlash, `<root>/rom/dflash.bin` on disk is used.
        fs::remove_file(bench_dir.join("my_bench_df0.bin")).unwrap();
        let (p2, d2) = read_dflash_with_bases(&bases, &root, false).unwrap();
        assert_eq!(p2, root.join("rom/dflash.bin"));
        assert!(d2.iter().all(|&b| b == 0xCC));

        // 3. Standalone binary outside repo tree: falls back to built-in `EMBEDDED_DEFAULT_DFLASH`.
        fs::remove_file(rom_dir.join("dflash.bin")).unwrap();
        assert!(read_dflash_with_bases(&bases, &root, false).is_err());
        let (_, d3) = read_dflash_with_bases(&bases, &root, true).unwrap();
        assert_eq!(d3.len(), 0x04_0000);
        assert_eq!(&d3[..], &EMBEDDED_DEFAULT_DFLASH[..]);

        let _ = fs::remove_dir_all(&root);
    }
}
