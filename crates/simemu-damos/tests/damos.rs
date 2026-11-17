//! Integration and adversarial edge-case tests for `simemu-damos`.
//!
//! Real-firmware tests resolve `.bin` dumps across `$SIMEMU_IMAGE_DIR`, `<root>/vault/firmwares/ecu/**`,
//! and `<root>/` (skipping cleanly when user-supplied OEM dumps are absent).
//! Synthetic codec, rational-formula, overflow, and checksum tests always execute.

use simemu_damos::{
    apply_post_programming, fix_checksums, verify_checksums, DamosDatabase, CURATED_TUNER_MAPS,
};
use std::path::PathBuf;
use std::sync::OnceLock;

const S50_BIN: &str = "8V0906264K__0003.bin";
const STOCK_O40_BIN: &str = "8V0906264H__0005.bin";

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn read_opt(name: &str) -> Option<Vec<u8>> {
    let ws = root();
    let bases: Vec<PathBuf> = std::env::var_os("SIMEMU_IMAGE_DIR")
        .into_iter()
        .map(PathBuf::from)
        .chain(std::iter::once(ws))
        .collect();
    for base in &bases {
        for sub in [
            "",
            "vault/firmwares/ecu/bench",
            "vault/firmwares/ecu/stock",
            "vault/firmwares/ecu/custom",
            "vault/firmwares/ecu",
            "firmwares/ecu/bench",
            "firmwares/ecu/stock",
            "firmwares/ecu/custom",
        ] {
            let p = if sub.is_empty() {
                base.join(name)
            } else {
                base.join(sub).join(name)
            };
            if p.is_file() {
                return std::fs::read(p).ok();
            }
        }
    }
    None
}

fn s50() -> Option<&'static (DamosDatabase, Vec<u8>)> {
    static S50: OnceLock<Option<(DamosDatabase, Vec<u8>)>> = OnceLock::new();
    S50.get_or_init(|| {
        let flash = read_opt(S50_BIN)?;
        let db = DamosDatabase::load_from_json(&root().join("definitions/SC8S50.json"))
            .unwrap();
        Some((db, flash))
    })
    .as_ref()
}

fn changed_bytes(a: &[u8], b: &[u8]) -> Vec<usize> {
    (0..a.len()).filter(|&i| a[i] != b[i]).collect()
}

#[test]
fn full_database_read_write_roundtrip_is_lossless() {
    let Some((db, stock)) = s50() else {
        eprintln!("SKIP: {S50_BIN} not found");
        return;
    };
    assert_eq!(db.parameters.len(), 33_140);
    let mut flash = stock.clone();
    let mut nudged = stock.clone();
    let mut clamped = Vec::new();
    for p in &db.parameters {
        let m = db
            .read_matrix(p, stock, stock)
            .unwrap_or_else(|| panic!("{} unreadable", p.id));
        db.write_matrix(p, &mut flash, &m.values).unwrap();
        // Defeat the unchanged-cell shortcut so every cell goes through the encoder.
        let vals: Vec<Vec<f64>> = m
            .values
            .iter()
            .map(|r| r.iter().map(|v| v + v.abs() * 1e-12).collect())
            .collect();
        let (_, block) = db.write_matrix(p, &mut nudged, &vals).unwrap();
        let start = p.file_offset as usize + p.data_offset;
        if block[..] != stock[start..start + block.len()] {
            clamped.push(p.id.as_str());
        }
    }
    assert!(changed_bytes(stock, &flash).is_empty());
    // Only stock cells outside the definition's [min, max] re-encode differently (saturated).
    assert_eq!(
        clamped,
        [
            "data_engtoptm.engtoptm_facfil_tsensdisphi_t_vw",
            "ip_lamb_delta_max_nom_pls",
            "c_lamb_delta_max_is_pls"
        ]
    );
}

#[test]
fn axes_use_byte_data_offset() {
    let Some((db, flash)) = s50() else {
        eprintln!("SKIP: {S50_BIN} not found");
        return;
    };
    let (mut total, mut monotonic) = (0, 0);
    for p in &db.parameters {
        let m = db.read_matrix(p, flash, flash).unwrap();
        for (ax, vals) in [(&p.x_axis, &m.x_axis), (&p.y_axis, &m.y_axis)] {
            if ax.as_ref().is_some_and(|a| a.address.is_some()) {
                total += 1;
                // 30 axes have a negative factor and 4 are stored descending.
                monotonic += (vals.windows(2).all(|w| w[0] < w[1])
                    || vals.windows(2).all(|w| w[0] > w[1])) as usize;
            }
        }
    }
    assert_eq!(total, 9157);
    // Element-offset semantics (the old bug) gave only 4643 strictly increasing raw axes.
    assert_eq!(monotonic, 9109);
}

#[test]
fn float32_parameter_decodes() {
    let Some((db, flash)) = s50() else {
        eprintln!("SKIP: {S50_BIN} not found");
        return;
    };
    let p = db.get_by_id("c_prs_air_sp_hys_wide_open_thr").unwrap();
    assert!(p.is_float && p.byte_len == 4);
    let v = db.read_physical_value(p, flash).unwrap();
    assert!((v - 20.0).abs() < 0.01, "{v}");
}

#[test]
fn std_axis_parameter_reads_and_writes_past_its_axes() {
    let Some((db, stock)) = s50() else {
        eprintln!("SKIP: {S50_BIN} not found");
        return;
    };
    let p = db
        .get_by_id("data_asdleadprm.asdlead_facdrvmodopt_tqofs_m_vw.at_fnt")
        .unwrap();
    assert_eq!(p.data_offset, 36);
    let m = db.read_matrix(p, stock, stock).unwrap();
    assert_eq!(
        m.x_axis,
        [-50.0, 0.0, 200.0, 500.0, 1000.0, 2000.0, 2500.0, 3500.0]
    );
    assert_eq!(
        m.y_axis,
        [-50.0, 0.0, 200.0, 500.0, 1000.0, 2000.0, 2500.0, 3500.0]
    );
    assert_eq!(m.values[0][0], 51.0 * 0.01953125);

    let mut flash = stock.clone();
    let mut vals = m.values.clone();
    vals[0][0] += 0.01953125;
    let (cal_off, bytes) = db.write_matrix(p, &mut flash, &vals).unwrap();
    let data = p.file_offset as usize + 36;
    assert_eq!(changed_bytes(stock, &flash), [data]);
    assert_eq!(cal_off, data - 0x20_0000);
    assert_eq!(bytes.len(), 64);
}

#[test]
fn writes_round_and_saturate_to_definition_limits() {
    let Some((db, stock)) = s50() else {
        eprintln!("SKIP: {S50_BIN} not found");
        return;
    };
    // UWORD, factor 1/65536, limits 0..0.99998
    let p = db.get_by_id("c_crlc_taa_estim_fil_ext").unwrap();
    let off = p.file_offset as usize;
    let mut flash = stock.clone();
    let raw = |f: &[u8]| u16::from_le_bytes([f[off], f[off + 1]]);
    db.write_matrix(p, &mut flash, &[vec![5.0]]).unwrap();
    assert_eq!(raw(&flash), 65535);
    db.write_matrix(p, &mut flash, &[vec![-1.0]]).unwrap();
    assert_eq!(raw(&flash), 0);
    db.write_matrix(p, &mut flash, &[vec![2.5 * p.scale]])
        .unwrap();
    assert_eq!(raw(&flash), 3); // half away from zero
    assert!(db.write_matrix(p, &mut flash, &[vec![f64::NAN]]).is_err());
}

#[test]
fn duplicate_names_get_stable_unique_ids() {
    let defs = root().join("definitions");
    let mut db = DamosDatabase::load_from_json(&defs.join("8V0906264H__0003_SC8O30.json")).unwrap();
    db.merge_from_json(&defs.join("O30_Switch_Patch.29.33.V2.json"))
        .unwrap();
    // Merging a second time must also preserve globally unique IDs without collision.
    db.merge_from_json(&defs.join("O30_Switch_Patch.29.33.V2.json"))
        .unwrap();
    let mut ids: Vec<_> = db.parameters.iter().map(|p| p.id.to_lowercase()).collect();
    ids.sort();
    ids.dedup();
    assert_eq!(ids.len(), db.parameters.len());
    for (i, p) in db.parameters.iter().enumerate() {
        assert_eq!(db.by_name[&p.id.to_lowercase()], i);
    }
    // First occurrence keeps the plain name.
    assert_eq!(
        db.get_by_id("AsdDsbcCtl_ctDsbcDet_VW").unwrap().file_offset,
        2_546_649
    );
    assert_eq!(
        db.get_by_id("AsdDsbcCtl_ctDsbcDet_VW [6]")
            .unwrap()
            .file_offset,
        2_546_625
    );
}

#[test]
fn curated_maps_resolve() {
    let Some((db, flash)) = s50() else {
        eprintln!("SKIP: {S50_BIN} not found");
        return;
    };
    for c in CURATED_TUNER_MAPS {
        let p = db
            .get_by_id(c.id)
            .unwrap_or_else(|| panic!("{} missing", c.id));
        db.read_matrix(p, flash, flash).unwrap();
    }
    let p = db.get_by_id("ip_n_max_tco").unwrap();
    let m = db.read_matrix(p, flash, flash).unwrap();
    assert!(m.values[0].iter().all(|&v| v == 6816.0), "{:?}", m.values[0]);
}

#[test]
fn stock_image_checksums() {
    let Some((_, stock)) = s50() else {
        eprintln!("SKIP: {S50_BIN} not found");
        return;
    };
    // Genuine VAG FRF stock image: every stored CRC / ECM3 checksum verifies out of the box and `fix_checksums` is a no-op.
    let r = verify_checksums(stock).unwrap();
    let names: Vec<_> = r.entries.iter().map(|e| e.name).collect();
    assert_eq!(
        names,
        ["ECM3 ASW", "ECM3 CAL", "SBOOT", "CBOOT", "ASW1", "ASW2", "ASW3", "CAL"]
    );
    assert!(r.ok(), "{r:?}");
    assert_eq!(r.get("ASW1").unwrap().computed, 0x87BE_738D);
    assert_eq!(r.get("ASW3").unwrap().stored, 0x66D8_4D9E);
    let mut fixed = stock.clone();
    fix_checksums(&mut fixed).unwrap();
    assert!(changed_bytes(stock, &fixed).is_empty());

    // If a 4-byte modification is introduced in ASW1 without updating its CRC, only ASW1 fails and `fix_checksums`
    // rewrites only the stale ASW1 CRC word (`0x40304..0x40308`) idempotently.
    let mut modified = stock.clone();
    modified[0x11_BC60..0x11_BC64].copy_from_slice(&[0xBB, 0x80, 0x05, 0x20]);
    let bad: Vec<_> = verify_checksums(&modified)
        .unwrap()
        .entries
        .into_iter()
        .filter(|e| !e.ok())
        .map(|e| e.name)
        .collect();
    assert_eq!(bad, ["ASW1"]);
    let mut fixed = modified.clone();
    let rep = fix_checksums(&mut fixed).unwrap();
    assert!(!rep.get("ASW1").unwrap().ok());
    assert!(changed_bytes(&modified, &fixed)
        .iter()
        .all(|&i| (0x4_0304..0x4_0308).contains(&i)));
    let once = fixed.clone();
    assert!(fix_checksums(&mut fixed).unwrap().ok());
    assert_eq!(once, fixed);
}

#[test]
fn cal_edit_is_detected_and_fixed() {
    let Some((_, clean)) = s50() else {
        eprintln!("SKIP: {S50_BIN} not found");
        return;
    };
    // Inside the ECM3 CAL range 0x8080D9AC..0x80810230 → ECM3 CAL + CAL CRC.
    let mut f = clean.clone();
    f[0x20_D9AC] ^= 1;
    let bad: Vec<_> = verify_checksums(&f)
        .unwrap()
        .entries
        .into_iter()
        .filter(|e| !e.ok())
        .map(|e| e.name)
        .collect();
    assert_eq!(bad, ["ECM3 CAL", "CAL"]);
    fix_checksums(&mut f).unwrap();
    assert!(verify_checksums(&f).unwrap().ok());
    let changed = changed_bytes(clean, &f);
    assert!(changed.iter().all(|&i| i == 0x20_D9AC
        || (0x20_0400..0x20_0408).contains(&i)
        || (0x20_0304..0x20_0308).contains(&i)));

    // A map write outside the ECM3 range → CAL CRC only.
    let (db, _) = s50().unwrap();
    let p = db.get_by_id("ip_n_max_tco").unwrap();
    let mut f = clean.clone();
    db.write_matrix(p, &mut f, &[vec![6560.0; 6]]).unwrap();
    let bad: Vec<_> = verify_checksums(&f)
        .unwrap()
        .entries
        .into_iter()
        .filter(|e| !e.ok())
        .map(|e| e.name)
        .collect();
    assert_eq!(bad, ["CAL"]);
    fix_checksums(&mut f).unwrap();
    assert!(verify_checksums(&f).unwrap().ok());
}

#[test]
fn modified_blocks_without_sboot_are_detected_and_fixed() {
    let Some(mut f) = read_opt(STOCK_O40_BIN) else {
        eprintln!("SKIP: {STOCK_O40_BIN} not found");
        return;
    };
    // Simulate an OBD-only read (erased SBOOT) with modified ASW1 and CAL bytes and stale CRCs/ECM3.
    f[..0x1_C000].fill(0xFF);
    f[0x05_0000] ^= 0x01;
    f[0x21_0000] ^= 0x01;
    let r = verify_checksums(&f).unwrap();
    assert!(r.get("SBOOT").is_none(), "no SBOOT in an OBD-only read");
    let bad: Vec<_> = r
        .entries
        .iter()
        .filter(|e| !e.ok())
        .map(|e| e.name)
        .collect();
    assert_eq!(bad, ["ECM3 CAL", "ASW1", "CAL"]);
    let mut fixed = f.clone();
    fix_checksums(&mut fixed).unwrap();
    assert!(verify_checksums(&fixed).unwrap().ok());
}

#[test]
fn rejects_non_full_images() {
    assert!(verify_checksums(&[0u8; 0x8_0000]).is_err());
}

#[test]
fn post_programming_completes_stripped_image_and_leaves_stock_untouched() {
    let Some(mut stock) = read_opt(STOCK_O40_BIN) else {
        eprintln!("SKIP: {STOCK_O40_BIN} not found");
        return;
    };
    let before = stock.clone();
    assert!(apply_post_programming(&mut stock).unwrap().is_empty());
    assert_eq!(stock, before);

    // Simulate an unprogrammed read lacking all 5 CBOOT validity markers and carrying stale ASW1/CAL/ECM3 sums.
    let mut raw = before.clone();
    for off in [0x03_FE00usize, 0x13_FE00, 0x1F_FE00, 0x27_FE00, 0x2F_FE00] {
        raw[off..off + 4].fill(0xFF);
    }
    raw[0x05_0000] ^= 0x01;
    raw[0x21_0000] ^= 0x01;

    let log = apply_post_programming(&mut raw).unwrap();
    assert_eq!(log.iter().filter(|l| l.contains("0x90482442") || l.contains("0x90244842") || l.contains("0x42244890")).count(), 5, "{log:?}");
    assert!(log.iter().any(|l| l.contains("ECM3 CAL") && (l.contains("recomputed") || l.contains("пересчитана"))), "{log:?}");
    assert!(log.iter().any(|l| l.contains("ASW1") && (l.contains("recomputed") || l.contains("пересчитана"))), "{log:?}");
    assert!(verify_checksums(&raw).unwrap().ok());
    // idempotent
    assert!(apply_post_programming(&mut raw).unwrap().is_empty());
}

/// SC8H83 lays out its ECM3 tables differently: the image still loads, and the unverified sum is stated.
#[test]
fn post_programming_reports_unrecognised_ecm3_layout_instead_of_failing() {
    let Some(mut f) = read_opt("8V0906264F__0003.bin") else {
        eprintln!("SKIP: 8V0906264F__0003.bin not found");
        return;
    };
    let log = apply_post_programming(&mut f).expect("SC8H83 must load");
    assert!(log.iter().any(|l| (l.contains("not verified") || l.contains("не проверена")) && l.contains("ECM3 ASW")), "{log:?}");
}

#[test]
fn rational_formula_pole_crossing_and_pid_i_weight_saturation() {
    let patch_path = root().join("definitions/O30_Switch_Patch.29.33.V2.json");
    let db = DamosDatabase::load_from_json(&patch_path).unwrap();
    let pid_i = db.get_by_id("PID I weight").unwrap();
    let off = pid_i.file_offset as usize + pid_i.data_offset;

    let mut flash = vec![0u8; 0x40_0000];
    // Below min (0.0) and at +/-0.0 (the horizontal asymptote of 32/(1+X)):
    // must saturate to raw 255 (phys 0.125 = min), NEVER invert across the pole to raw 7 (phys 4.0 = max).
    for v in [-0.5, -0.0, 0.0, 0.05] {
        flash[off] = 15;
        db.write_matrix(pid_i, &mut flash, &[vec![v]]).unwrap();
        assert_eq!(flash[off], 255, "PID I weight for {v} must saturate to raw 255");
    }

    // Exact interior value: phys = 1.0 -> 32 / (1 + 31) = 1.0 -> raw 31.
    db.write_matrix(pid_i, &mut flash, &[vec![1.0]]).unwrap();
    assert_eq!(flash[off], 31);
    assert!((db.read_physical_value(pid_i, &flash).unwrap() - 1.0).abs() < 1e-12);

    // Above max (4.0): must saturate to raw 7 (32 / 8 = 4.0).
    for v in [4.0, 100.0] {
        flash[off] = 31;
        db.write_matrix(pid_i, &mut flash, &[vec![v]]).unwrap();
        assert_eq!(flash[off], 7, "PID I weight for {v} must saturate to raw 7");
    }

    // UWORD rational formula ("Slip wheel angle multiplier", 6622 / (1 + X), min 0.0, max 2.18)
    let slip = db.get_by_id("Slip wheel angle multiplier").unwrap();
    let s_off = slip.file_offset as usize + slip.data_offset;
    db.write_matrix(slip, &mut flash, &[vec![-1.0]]).unwrap();
    assert_eq!(u16::from_le_bytes([flash[s_off], flash[s_off + 1]]), 65535);
    db.write_matrix(slip, &mut flash, &[vec![10.0]]).unwrap();
    assert_eq!(u16::from_le_bytes([flash[s_off], flash[s_off + 1]]), 3037);
}

#[test]
fn rejects_corrupt_defs_overflows_erased_floats_and_non_cal_live_writes() {
    let dir = std::env::temp_dir().join(format!("simemu_damos_test_{}", std::process::id()));
    let _ = std::fs::create_dir_all(&dir);
    let write_json = |name: &str, body: &str| -> PathBuf {
        let p = dir.join(name);
        std::fs::write(&p, body).unwrap();
        p
    };

    // 1. factor == 0.0 rejected at load
    let zero_scale = write_json(
        "zero_scale.json",
        r#"{"name":"T","version":"1","baseAddress":2097152,"parameters":[{"name":"bad","address":16,"dataType":"UWORD","factor":0.0}]}"#,
    );
    assert!(DamosDatabase::load_from_json(&zero_scale).is_err());

    // 2. Degenerate rational formula (a*d == b*c -> constant function) rejected at load
    let degen = write_json(
        "degen.json",
        r#"{"name":"T","version":"1","baseAddress":2097152,"parameters":[{"name":"bad","address":16,"dataType":"UBYTE","formula":{"a":2.0,"b":4.0,"c":1.0,"d":2.0}}]}"#,
    );
    assert!(DamosDatabase::load_from_json(&degen).is_err());

    // 3. Oversized matrix (> 65536 cells) rejected at load without OOM
    let huge = write_json(
        "huge.json",
        r#"{"name":"T","version":"1","baseAddress":2097152,"parameters":[{"name":"huge","address":16,"dataType":"UWORD","rows":1000,"cols":1000}]}"#,
    );
    assert!(DamosDatabase::load_from_json(&huge).is_err());

    // 4. Truncated .bin, erased 0xFF FLOAT32 cells/axes, and non-CAL live write rejection
    let mixed = write_json(
        "mixed.json",
        r#"{
            "name":"T","version":"1","baseAddress":0,
            "parameters":[
                {"name":"flt_cal","address":2097200,"dataType":"FLOAT32","rows":2,"cols":1,
                 "xAxis":{"address":2097184,"dataType":"FLOAT32","points":2,"factor":1.0}},
                {"name":"pflash0_const","address":65536,"dataType":"UWORD","factor":1.0}
            ]
        }"#,
    );
    let db = DamosDatabase::load_from_json(&mixed).unwrap();
    let flt = db.get_by_id("flt_cal").unwrap();
    let p0 = db.get_by_id("pflash0_const").unwrap();
    assert_eq!(p0.cpu_address, 0x8001_0000);

    // Truncated flash returns None without panicking.
    assert!(db.read_matrix(flt, &[0u8; 128], &[0u8; 128]).is_none());

    // Erased 0xFF Flash has NaN FLOAT32 cells -> read_matrix returns None, write_matrix rejects NaN
    // even when matching existing NaN bits, and finite write succeeds cleanly while axis NaN falls back to 0..N.
    let mut erased = vec![0xFFu8; 0x40_0000];
    assert!(db.read_matrix(flt, &erased, &erased).is_none());
    assert!(db.write_matrix(flt, &mut erased, &[vec![f64::NAN], vec![1.0]]).is_err());
    let (cal_off, written) = db
        .write_matrix(flt, &mut erased, &[vec![12.5], vec![-3.25]])
        .unwrap();
    assert_eq!(cal_off, 2097200 - 0x20_0000);
    assert_eq!(written.len(), 8);
    let m = db.read_matrix(flt, &erased, &erased).unwrap();
    assert_eq!(m.values, vec![vec![12.5], vec![-3.25]]);
    assert_eq!(m.y_axis, vec![0.0, 1.0]); // erased FLOAT32 axis fell back to index 0, 1 (finite!)

    // Non-CAL parameter (in PFlash0 code area) can be read, but write_matrix MUST refuse to touch code!
    assert!(db.write_matrix(p0, &mut erased, &[vec![42.0]]).is_err());

    // Overflowing `data_offset = usize::MAX` on manually constructed parameter never wraps or panics.
    let mut overflow_param = flt.clone();
    overflow_param.data_offset = usize::MAX;
    assert!(db.read_physical_value(&overflow_param, &erased).is_none());
    assert!(db.read_matrix(&overflow_param, &erased, &erased).is_none());
    assert!(db
        .write_matrix(&overflow_param, &mut erased, &[vec![1.0], vec![2.0]])
        .is_err());

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn blank_and_corrupt_images_fail_checksums_without_partial_mutation() {
    let mut blank = vec![0u8; 0x40_0000];
    let rep = verify_checksums(&blank).unwrap();
    assert!(rep.entries.is_empty());
    assert!(!rep.ok(), "blank image with zero block headers must not report ok()");
    assert!(fix_checksums(&mut blank).is_err());
    assert!(apply_post_programming(&mut blank).is_err());

    // Synthetic image with 0xFF-erased SBOOT header (n = 0xFFFF_FFFF) and a valid CAL block header:
    // verify_checksums skips the 0xFF SBOOT header cleanly and checks CAL.
    let mut synth = vec![0xFFu8; 0x40_0000];
    // Clear CBOOT/ASW1/ASW2/ASW3 range counts to 0, leave SBOOT at 0xFFFF_FFFF, set 1 range in CAL.
    for hdr in [0x1_C300usize, 0x4_0300, 0x14_0000, 0x28_0000] {
        synth[hdr + 8..hdr + 12].copy_from_slice(&0u32.to_le_bytes());
    }
    let cal_hdr = 0x20_0300usize;
    synth[cal_hdr + 8..cal_hdr + 12].copy_from_slice(&1u32.to_le_bytes());
    synth[cal_hdr + 12..cal_hdr + 16].copy_from_slice(&0x8080_0400u32.to_le_bytes());
    synth[cal_hdr + 16..cal_hdr + 20].copy_from_slice(&0x8080_04FFu32.to_le_bytes());
    assert!(fix_checksums(&mut synth).unwrap().get("SBOOT").is_none());
    assert!(verify_checksums(&synth).unwrap().ok());

    // Corrupt ASW1 header range (outside PFlash) while CAL marker is erased:
    // apply_post_programming must fail BEFORE mutating the CAL marker!
    synth[0x27_FE00..0x27_FE04].copy_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
    synth[0x4_0308..0x4_030C].copy_from_slice(&1u32.to_le_bytes());
    synth[0x4_030C..0x4_0310].copy_from_slice(&0xD000_0000u32.to_le_bytes()); // RAM, invalid!
    synth[0x4_0310..0x4_0314].copy_from_slice(&0xD000_00FFu32.to_le_bytes());
    let snapshot = synth.clone();
    assert!(apply_post_programming(&mut synth).is_err());
    assert_eq!(synth, snapshot, "corrupt image must not be partially mutated");
}

#[test]
fn native_frf_unpacker_matches_all_factory_stock_bins_bit_for_bit() {
    let sboot_rom = read_opt("rom/sboot.bin").expect("bundled rom/sboot.bin must exist in repo");
    assert_eq!(sboot_rom.len(), 0x01_C000);

    for (frf_rel, stock_rel, expected_box, expected_part) in [
        (
            "vault/firmwares/ecu/frf/FL_8V0906264F__0003.frf",
            "vault/firmwares/ecu/stock/8V0906264F__0003.bin",
            "SC8H83",
            "8V0906264F",
        ),
        (
            "vault/firmwares/ecu/frf/FL_8V0906264H__0005.frf",
            "vault/firmwares/ecu/stock/8V0906264H__0005.bin",
            "SC8O40",
            "8V0906264H",
        ),
        (
            "vault/firmwares/ecu/frf/FL_8V0906264J__0003.frf",
            "vault/firmwares/ecu/stock/8V0906264J__0003.bin",
            "SC8O40",
            "8V0906264J",
        ),
        (
            "vault/firmwares/ecu/frf/FL_8V0906264K__0003.frf",
            "vault/firmwares/ecu/stock/8V0906264K__0003.bin",
            "SC8S50",
            "8V0906264K",
        ),
    ] {
        let Some(frf_bytes) = read_opt(frf_rel) else {
            eprintln!("SKIP: {frf_rel} not found");
            continue;
        };
        assert!(simemu_damos::is_frf_bytes(&frf_bytes));
        let cal_head = simemu_damos::peek_frf_cal_prefix(&frf_bytes, 0x100).unwrap();
        assert_eq!(&cal_head[2..8], expected_box.as_bytes());

        // 1. Unpack using the 112 KB bundled `rom/sboot.bin`!
        let (img_from_sboot, rep) =
            simemu_damos::unpack_frf_into_image(&frf_bytes, Some(&sboot_rom)).unwrap();
        assert_eq!(rep.box_code, expected_box);
        assert_eq!(rep.part_number, expected_part);
        assert_eq!(rep.blocks_written, ["CBOOT", "ASW1", "ASW2", "ASW3", "CAL"]);
        assert!(rep.has_sboot);
        assert!(verify_checksums(&img_from_sboot).unwrap().ok());

        // 2. Compare against factory stock `.bin` across `SBOOT`, all 5 FRF blocks, and all CBOOT `+FC00` / `+FE00` markers.
        if let Some(stock_bin) = read_opt(stock_rel) {
            for &(name, start, end) in &[
                ("SBOOT", 0x00_0000usize, 0x01_C000usize),
                ("CBOOT", 0x01_C000, 0x03_FC00),
                ("CBOOT_VALID", 0x03_FE00, 0x04_0000),
                ("ASW1", 0x04_0000, 0x13_FC04),
                ("ASW1_VALID", 0x13_FE00, 0x14_0000),
                ("ASW2", 0x14_0000, 0x1F_FC04),
                ("ASW2_VALID", 0x1F_FE00, 0x20_0000),
                ("CAL", 0x20_0000, 0x27_FC04),
                ("CAL_VALID", 0x27_FE00, 0x28_0000),
                ("ASW3", 0x28_0000, 0x2F_FC04),
                ("ASW3_VALID", 0x2F_FE00, 0x40_0000),
            ] {
                assert!(
                    img_from_sboot[start..end] == stock_bin[start..end],
                    "{frf_rel}: rom/sboot.bin unpack differed in {name} (0x{start:X}..0x{end:X})"
                );
            }
            assert_eq!(&cal_head[..0x100], &stock_bin[0x20_0000..0x20_0100]);
        }

        // 3. Donorless unpack (`None`) also succeeds with valid block CRCs, ECM3 sums, and markers.
        let (standalone, rep_no_donor) = simemu_damos::unpack_frf_into_image(&frf_bytes, None).unwrap();
        assert!(!rep_no_donor.has_sboot);
        assert!(verify_checksums(&standalone).unwrap().ok());
        let mut standalone_after = standalone.clone();
        let _ = apply_post_programming(&mut standalone_after).unwrap();
        assert_eq!(standalone_after, standalone);
    }
}
