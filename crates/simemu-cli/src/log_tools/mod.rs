//! Vehicle-log tooling: loader, channel dictionary, forensic analysis, calibration diff and HTML reports.
//! CLI: `simemu log-report <log.csv> [--image <bin>]`, `simemu cal-diff [A.bin A.json B.bin B.json]`.

pub mod caldiff;
pub mod channels;
pub mod csv_log;
pub mod forensics;
pub mod report;

use std::path::{Path, PathBuf};

pub const STOCK_264H: &str = "8V0906264H__0005.bin";
pub const STOCK_264H_DAMOS: &str = "definitions/SC8O40.json";
pub const STOCK_264K: &str = "8V0906264K__0003.bin";
pub const STOCK_264K_DAMOS: &str = "definitions/SC8S50.json";

/// Value of `--key value` in `args`.
pub fn opt<'a>(args: &'a [String], key: &str) -> Option<&'a str> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1)).map(|s| s.as_str())
}

/// Directory under `build/` for a report named after the log file.
pub fn out_dir(root: &Path, kind: &str, log: &Path) -> Result<PathBuf, String> {
    let stem = log.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "log".into());
    let d = root.join("build").join(kind).join(stem);
    std::fs::create_dir_all(&d).map_err(|e| format!("{}: {e}", d.display()))?;
    Ok(d)
}

/// `simemu log-report <log.csv> [--image <bin>] [--damos <json>]`
pub fn cmd_log_report(root: &Path, args: &[String]) -> Result<(), String> {
    let log = PathBuf::from(args.first().ok_or("usage: simemu log-report <log.csv> [--image <bin>] [--damos <json>]")?);
    let d = csv_log::LogData::load(&log)?;
    let image = opt(args, "--image").unwrap_or(STOCK_264H);
    let damos = opt(args, "--damos").unwrap_or(STOCK_264H_DAMOS);
    let cal = caldiff::Image::load(root, image, damos, &[]).map_err(|e| eprintln!("calibration not loaded ({e}): fuel-vs-command check skipped")).ok();
    let f = forensics::analyse(&d, cal.as_ref());
    let dir = out_dir(root, "log_report", &log)?;
    std::fs::write(dir.join("forensics.md"), &f.markdown).map_err(|e| e.to_string())?;
    std::fs::write(dir.join("forensics.html"), &f.html).map_err(|e| e.to_string())?;
    std::fs::write(dir.join("channels.md"), channels::dictionary_markdown(&d)).map_err(|e| e.to_string())?;
    println!("{}", f.markdown);
    println!("written: {}/forensics.{{md,html}}", dir.display());
    Ok(())
}

/// `simemu cal-diff [A.bin A.json B.bin B.json]` (default: stock `8V0906264K__0003` vs stock `8V0906264H__0005`).
pub fn cmd_cal_diff(root: &Path, args: &[String]) -> Result<(), String> {
    let a_args = [STOCK_264K, STOCK_264K_DAMOS, STOCK_264H, STOCK_264H_DAMOS];
    let p: Vec<&str> = (0..4).map(|i| args.get(i).map(|s| s.as_str()).unwrap_or(a_args[i])).collect();
    let a = caldiff::Image::load(root, p[0], p[1], &[])?;
    let b = caldiff::Image::load(root, p[2], p[3], &[])?;
    let (rows, compared, shape) = caldiff::diff(&a, &b);
    let mut md = caldiff::to_markdown(&a, &b, &rows, compared, &shape);
    md += "\n## Focus parameters, absolute values\n\n";
    md += &caldiff::focus_values(&a, &b);
    md += "\n## Pre-ignition (LSPI) configuration, absolute values\n\n";
    md += &caldiff::group_values(&a, &b, "pre-ignition / LSPI");
    let tag = |s: &str| Path::new(s).file_stem().map(|x| x.to_string_lossy().into_owned()).unwrap_or_default();
    let dir = root.join("build/caldiff");
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let base = dir.join(format!("{}_vs_{}", tag(p[0]), tag(p[2])));
    let with = |ext: &str| PathBuf::from(format!("{}.{ext}", base.display()));
    std::fs::write(with("md"), &md).map_err(|e| e.to_string())?;
    std::fs::write(with("csv"), caldiff::to_csv(&rows)).map_err(|e| e.to_string())?;
    println!("{md}");
    println!("written: {}.{{md,csv}}", base.display());
    Ok(())
}
