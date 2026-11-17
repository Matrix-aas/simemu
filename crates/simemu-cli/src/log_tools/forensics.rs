//! Forensics from a SimosTools log alone (plus, optionally, the calibration it ran on): phases, mixture,
//! spark/knock, boost, injection duty and window, fuel delivered vs commanded, gear ratios, turbo operating
//! points. Every number printed here is computed from the log and the cited calibration maps; physical
//! constants without a source are marked UNVERIFIED in the output.

use super::caldiff::{lookup, Image};
use super::report::{html_page, svg_plot, Series, Span};
use std::fmt::Write;

/// Stoichiometric air/fuel ratio of E0 gasoline (log `Eth Content` = 0). UNVERIFIED for the actual fuel.
const AFR_ST: f64 = 14.7;

pub struct Log<'a> {
    pub t: &'a [f64],
    cols: Vec<(&'static str, Vec<f64>)>,
}

impl<'a> Log<'a> {
    pub fn new(d: &'a super::csv_log::LogData) -> Self {
        let cols = super::channels::CHANNELS
            .iter()
            .map(|c| (c.name, d.col(&[c.name]).map(|v| v.to_vec()).unwrap_or_else(|| vec![f64::NAN; d.t.len()])))
            .collect();
        Self { t: &d.t, cols }
    }

    pub fn v(&self, name: &str) -> &[f64] {
        &self.cols.iter().find(|(n, _)| n.eq_ignore_ascii_case(name)).unwrap_or_else(|| panic!("channel {name}")).1
    }
}

#[derive(Debug, Clone)]
pub struct Phase {
    pub kind: &'static str,
    pub i0: usize,
    pub i1: usize,
}

fn classify(l: &Log, i: usize) -> &'static str {
    let (pedal, v, n) = (l.v("Pedal Pos")[i], l.v("Vehicle Speed")[i], l.v("Engine Speed")[i]);
    let fuel = l.v("Inj PW DI")[i] + l.v("Inj PW MPI")[i];
    if pedal >= 90.0 && v < 3.0 {
        "launch hold"
    } else if pedal >= 90.0 {
        "WOT pull"
    } else if pedal < 1.0 && n > 1200.0 && fuel <= 0.0 {
        "overrun cut-off"
    } else if pedal < 1.0 && v < 2.0 && n < 1100.0 {
        "idle"
    } else {
        "part load / transition"
    }
}

/// Contiguous segments of one class (single-sample flickers merged into the neighbour).
pub fn phases(l: &Log) -> Vec<Phase> {
    let mut k: Vec<&'static str> = (0..l.t.len()).map(|i| classify(l, i)).collect();
    for i in 1..k.len().saturating_sub(1) {
        if k[i - 1] == k[i + 1] && k[i] != k[i - 1] {
            k[i] = k[i - 1];
        }
    }
    let mut out: Vec<Phase> = Vec::new();
    for (i, &c) in k.iter().enumerate() {
        match out.last_mut() {
            Some(p) if p.kind == c => p.i1 = i,
            _ => out.push(Phase { kind: c, i0: i, i1: i }),
        }
    }
    out
}

fn stats(v: &[f64]) -> (f64, f64, f64) {
    let f: Vec<f64> = v.iter().copied().filter(|x| x.is_finite()).collect();
    if f.is_empty() {
        return (f64::NAN, f64::NAN, f64::NAN);
    }
    (f.iter().copied().fold(f64::MAX, f64::min), f.iter().sum::<f64>() / f.len() as f64, f.iter().copied().fold(f64::MIN, f64::max))
}

fn median(mut v: Vec<f64>) -> f64 {
    v.retain(|x| x.is_finite());
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// Candidate gear sets (overall ratio = gear × final drive). UNVERIFIED sources in the doc §1.
const GEARSETS: &[(&str, [f64; 6], [f64; 6])] = &[
    ("DQ250 02E (Golf V GTI set, profile dq250_4wd)", [3.462, 2.050, 1.300, 0.902, 0.914, 0.756], [4.118, 4.118, 4.118, 4.118, 3.043, 3.043]),
    ("DQ250 0D9 MQB (Golf 7 R / S3 8V set)", [2.923, 1.792, 1.143, 0.778, 0.800, 0.639], [4.769, 4.769, 4.769, 4.769, 3.444, 3.444]),
];

/// Fuel mass per injection from the firmware's own DI characteristic `ip_ti_4_h` (time = f(mass, ΔP)),
/// inverted by bisection. Returns None if the map is unavailable or its axes are not (mass, pressure).
fn di_mass_mg(cal: Option<&Image>, ti_ms: f64, dp_hpa: f64) -> Option<f64> {
    let m = cal?.map("ip_ti_4_h")?;
    let mass_on_x = m.x_unit.to_lowercase().contains("mg");
    let ti = |mg: f64| if mass_on_x { lookup(&m, mg, dp_hpa) } else { lookup(&m, dp_hpa, mg) };
    let (mut lo, mut hi) = (0.0, if mass_on_x { *m.x_axis.last()? } else { *m.y_axis.last()? } * 2.0);
    if ti_ms <= 0.0 {
        return Some(0.0);
    }
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if ti(mid) < ti_ms { lo = mid } else { hi = mid }
    }
    Some(0.5 * (lo + hi))
}

pub struct Forensics {
    pub markdown: String,
    pub html: String,
}

pub fn analyse(d: &super::csv_log::LogData, cal: Option<&Image>) -> Forensics {
    let l = Log::new(d);
    let n = l.t.len();
    let ph = phases(&l);
    let mut md = String::new();
    let rpm = l.v("Engine Speed");
    let v = l.v("Vehicle Speed");
    let gear = l.v("Gear");
    let pedal = l.v("Pedal Pos");
    let (pw_di, pw_mpi) = (l.v("Inj PW DI"), l.v("Inj PW MPI"));
    let (lam, lam_sp) = (l.v("Lambda"), l.v("Lambda SP"));
    let (map, map_sp, put, put_sp) = (l.v("MAP"), l.v("MAP SP"), l.v("PUT"), l.v("PUT SP"));
    let ign = l.v("Ign Avg");
    let knk: Vec<&[f64]> = (1..=4).map(|c| l.v(&format!("Knock Cyl {c}"))).collect();
    let (fp, fp_sp, hpfp) = (l.v("FP DI"), l.v("FP DI SP"), l.v("HPFP Eff Vol"));
    let airmass = l.v("Airmass");
    let dt_med = median(l.t.windows(2).map(|w| w[1] - w[0]).collect());

    let _ = writeln!(md, "# Log forensics: {}\n\n{} samples over {:.1} s (median step {:.3} s, {:.1} Hz). Channels found: {}/53.\n", d.source, n, d.duration(), dt_med, 1.0 / dt_med, l.cols.iter().filter(|(_, c)| c.iter().any(|x| x.is_finite())).count());

    // ---- phases ------------------------------------------------------------------------------
    let _ = writeln!(md, "## Phases\n\n| # | phase | t, s | rpm | km/h | gear (log → actual) | MAP max / SP max, kPa | λ mean / SP mean / min | ign min..max, ° | knock min cyl1..4, ° | DI / MPI PW max, ms |\n|---|---|---|---|---|---|---|---|---|---|---|");
    for (k, p) in ph.iter().enumerate().filter(|(_, p)| p.i1 > p.i0) {
        let r = p.i0..p.i1 + 1;
        let s = |x: &[f64]| stats(&x[r.clone()]);
        let gears: std::collections::BTreeSet<i64> = gear[r.clone()].iter().filter(|x| x.is_finite()).map(|x| *x as i64).collect();
        let _ = writeln!(
            md,
            "| {k} | {} | {:.1}–{:.1} | {:.0}–{:.0} | {:.0}–{:.0} | {} | {:.0} / {:.0} | {:.2} / {:.2} / {:.2} | {:.1}..{:.1} | {} | {:.2} / {:.2} |",
            p.kind,
            l.t[p.i0],
            l.t[p.i1],
            s(rpm).0,
            s(rpm).2,
            s(v).0,
            s(v).2,
            gears.iter().map(|g| format!("{g}→{}", g + 1)).collect::<Vec<_>>().join(","),
            s(map).2,
            s(map_sp).2,
            s(lam).1,
            s(lam_sp).1,
            s(lam).0,
            s(ign).0,
            s(ign).2,
            knk.iter().map(|c| format!("{:.2}", s(c).0)).collect::<Vec<_>>().join(" "),
            s(pw_di).2,
            s(pw_mpi).2
        );
    }

    // ---- gear ratios --------------------------------------------------------------------------
    let _ = writeln!(md, "\n## Gear (log `Gear` channel is 0-based) and overall ratio\n\nSamples with pedal > 20 %, speed > 10 km/h, stable gear (±1 sample). rpm per km/h → dynamic tyre radius each candidate gear set would need (consistent radius across gears = matching set).\n\n| log gear | actual | samples | rpm per km/h (median) | r_dyn needed, m: {} |\n|---|---|---|---|---|", GEARSETS.iter().map(|g| g.0).collect::<Vec<_>>().join(" / "));
    for g in 0..6 {
        let k: Vec<f64> = (1..n.saturating_sub(1))
            .filter(|&i| gear[i] == g as f64 && gear[i - 1] == gear[i] && gear[i + 1] == gear[i] && pedal[i] > 20.0 && v[i] > 10.0)
            .map(|i| rpm[i] / v[i])
            .collect();
        if k.len() < 3 {
            continue;
        }
        let kk = median(k.clone());
        // v[m/s] = ω_engine / i · r  →  r = (1/3.6) / (kk · 2π/60 / i)
        let radii: Vec<String> = GEARSETS.iter().map(|(_, gr, fd)| format!("{:.3}", (1.0 / 3.6) / (kk * std::f64::consts::PI / 30.0 / (gr[g] * fd[g])))).collect();
        let _ = writeln!(md, "| {g} | {} | {} | {:.2} | {} |", g + 1, k.len(), kk, radii.join(" / "));
    }

    // ---- injection --------------------------------------------------------------------------
    let cyc_ms: Vec<f64> = rpm.iter().map(|r| if *r > 100.0 { 120_000.0 / r } else { f64::NAN }).collect();
    let duty = |pw: &[f64]| -> Vec<f64> { pw.iter().zip(&cyc_ms).map(|(p, c)| p / c * 100.0).collect() };
    let (di_duty, mpi_duty) = (duty(pw_di), duty(pw_mpi));
    let di_deg: Vec<f64> = pw_di.iter().zip(rpm).map(|(p, r)| p * 1e-3 * r * 6.0).collect();
    let imax = |x: &[f64]| (0..n).filter(|&i| x[i].is_finite()).max_by(|&a, &b| x[a].total_cmp(&x[b])).unwrap_or(0);
    let (im, idd, ida, ipw) = (imax(&mpi_duty), imax(&di_duty), imax(&di_deg), imax(pw_mpi));
    let both = (0..n).filter(|&i| pw_di[i] > 0.0 && pw_mpi[i] > 0.0).count();
    let only_mpi = (0..n).filter(|&i| pw_di[i] <= 0.0 && pw_mpi[i] > 0.0).count();
    let only_di = (0..n).filter(|&i| pw_di[i] > 0.0 && pw_mpi[i] <= 0.0).count();
    let _ = writeln!(
        md,
        "\n## Injection duty and window\n\n`Inj PW DI` = TI_MES_4[PLS_1][0] (first DI pulse of cylinder 1), `Inj PW MPI` = TI_1_MES_MPI[0] (cylinder 1): further pulses of a split injection and the other cylinders are not in the log.\n\n- Samples with MPI only: {only_mpi}, DI only: {only_di}, both (combined mode): {both}.\n- Longest MPI pulse: {:.2} ms at {:.0} rpm, pedal {:.0} % (t = {:.1} s): cycle {:.1} ms → duty {:.1} %.\n- Highest MPI duty: {:.1} % at {:.0} rpm (PW {:.2} ms, t = {:.1} s). MPI saturation (duty ≥ ~85 % of the 720 °CA cycle) {}.\n- Highest DI duty: {:.1} % at {:.0} rpm (PW {:.2} ms, t = {:.1} s).\n- Longest DI injection in crank angle: {:.0} °CA at {:.0} rpm (PW {:.2} ms, t = {:.1} s). A single-pulse homogeneous DI injection has to fit between intake-stroke start (~360 °CA BTDC firing) and the firmware's EOI limit in compression (not logged; SimosTools 3E list has `EOI Limit`/`SOI Limit`) — UNVERIFIED window ≈ 250–300 °CA.",
        pw_mpi[ipw], rpm[ipw], pedal[ipw], l.t[ipw], cyc_ms[ipw], mpi_duty[ipw],
        mpi_duty[im], rpm[im], pw_mpi[im], l.t[im],
        if mpi_duty[im] >= 85.0 { "**reached**" } else { "not reached in this log" },
        di_duty[idd], rpm[idd], pw_di[idd], l.t[idd],
        di_deg[ida], rpm[ida], pw_di[ida], l.t[ida]
    );

    // ---- fuel delivered vs commanded --------------------------------------------------------
    let implied: Vec<f64> = (0..n).map(|i| if lam[i] > 0.3 && lam[i] < 2.0 { airmass[i] / (AFR_ST * lam[i]) } else { f64::NAN }).collect();
    let cmd_di: Vec<f64> = (0..n)
        .map(|i| if pw_di[i] > 0.0 && pw_mpi[i] <= 0.0 { di_mass_mg(cal, pw_di[i], (fp[i] * 1000.0 - map[i] * 10.0).max(0.0)).unwrap_or(f64::NAN) } else { f64::NAN })
        .collect();
    let wot_di: Vec<usize> = (0..n).filter(|&i| pedal[i] >= 90.0 && v[i] >= 3.0 && cmd_di[i].is_finite() && implied[i].is_finite()).collect();
    if cal.is_some() && !wot_di.is_empty() {
        let ratio: Vec<f64> = wot_di.iter().map(|&i| implied[i] / cmd_di[i]).collect();
        let _ = writeln!(
            md,
            "\n## Fuel delivered vs commanded (WOT, DI-only samples: {})\n\n- Implied fuel = Airmass / ({AFR_ST} · λ_measured) (air from the firmware's own airflow, λ from its wide-band sensor; AFR_st UNVERIFIED for the fuel used).\n- Commanded fuel = the firmware's DI characteristic `ip_ti_4_h` of the running calibration inverted at the logged PW and ΔP = FP DI − MAP (injection during the intake stroke).\n- implied / commanded: median {:.3}, min {:.3}, max {:.3}. Values < 1 = less fuel reached the exhaust stream than the injector map predicts (injector flow below its characteristic, or air overestimated); > 1 the opposite.",
            wot_di.len(),
            median(ratio.clone()),
            stats(&ratio).0,
            stats(&ratio).2
        );
    }

    // ---- mixture / boost / knock summaries during WOT -----------------------------------------
    let wot: Vec<usize> = (0..n).filter(|&i| pedal[i] >= 90.0).collect();
    let pick = |x: &[f64]| -> Vec<f64> { wot.iter().map(|&i| x[i]).collect() };
    let err = |a: &[f64], b: &[f64]| -> Vec<f64> { wot.iter().map(|&i| a[i] - b[i]).collect() };
    let lsp = (0..n).filter(|&i| rpm[i] < 3500.0 && map[i] > 200.0).count();
    let lsp_lam = stats(&(0..n).filter(|&i| rpm[i] < 3500.0 && map[i] > 200.0).map(|i| lam[i]).collect::<Vec<_>>());
    let _ = writeln!(
        md,
        "\n## WOT summary (pedal ≥ 90 %, {} samples)\n\n- λ − λ_SP: mean {:+.3}, min {:+.3}, max {:+.3}; λ min {:.2}.\n- MAP − MAP_SP: mean {:+.1} kPa, min {:+.1}, max {:+.1}. PUT − PUT_SP: mean {:+.1} kPa, max {:+.1}.\n- MAP max {:.1} kPa, PUT max {:.1} kPa, ambient {:.1} kPa → pressure ratio PUT/amb max {:.2}.\n- Rail: FP DI − SP min {:+.1} bar; HPFP Eff Vol ≥ 99 % in {} samples.\n- Low-speed / high-load samples (rpm < 3500 and MAP > 200 kPa — the LSPI-prone region): {} ({:.1} s), λ there {:.2}..{:.2}.\n- Knock retard (all samples): {}.\n- Turbo speed max {:.1} krpm; compressor-outlet model temperature max {:.0} °C; IAT max {:.1} °C.",
        wot.len(),
        stats(&err(lam, lam_sp)).1,
        stats(&err(lam, lam_sp)).0,
        stats(&err(lam, lam_sp)).2,
        stats(&pick(lam)).0,
        stats(&err(map, map_sp)).1,
        stats(&err(map, map_sp)).0,
        stats(&err(map, map_sp)).2,
        stats(&err(put, put_sp)).1,
        stats(&err(put, put_sp)).2,
        stats(map).2,
        stats(put).2,
        stats(l.v("Ambient Press")).1,
        stats(put).2 / stats(l.v("Ambient Press")).1,
        stats(&err(fp, fp_sp)).0,
        hpfp.iter().filter(|x| **x >= 99.0).count(),
        lsp,
        lsp as f64 * dt_med,
        lsp_lam.0,
        lsp_lam.2,
        knk.iter().enumerate().map(|(c, x)| format!("cyl {} min {:.2}° in {} samples", c + 1, stats(x).0, x.iter().filter(|y| **y < 0.0).count())).collect::<Vec<_>>().join("; "),
        stats(l.v("Turbo Speed")).2,
        stats(l.v("Turbo Air Temp")).2,
        stats(l.v("IAT")).2
    );

    // ---- HTML ------------------------------------------------------------------------------
    let spans: Vec<Span> = ph.iter().filter(|p| p.kind != "part load / transition" && p.i1 > p.i0).map(|p| Span { t0: l.t[p.i0], t1: l.t[p.i1], label: p.kind }).collect();
    let tr = (0.0, d.duration());
    let gear1: Vec<f64> = gear.iter().map(|g| g + 1.0).collect();
    type PlotSeries<'a> = (&'a str, &'a [f64], &'a str);
    let plots: Vec<(&str, &str, Vec<PlotSeries<'_>>)> = vec![
        ("Engine speed", "rpm", vec![("Engine Speed", rpm, "#0a84ff")]),
        ("Vehicle speed / pedal", "km/h, %", vec![("Vehicle Speed", v, "#30b0c7"), ("Pedal Pos", pedal, "#ff9f0a"), ("TPS", l.v("TPS"), "#8e8e93")]),
        ("Gear (actual = log + 1)", "-", vec![("gear", &gear1, "#5e5ce6")]),
        ("Manifold pressure", "kPa", vec![("MAP", map, "#0a84ff"), ("MAP SP", map_sp, "#ff375f"), ("PUT", put, "#30d158"), ("PUT SP", put_sp, "#ff9f0a")]),
        ("Lambda", "-", vec![("Lambda", lam, "#0a84ff"), ("Lambda SP", lam_sp, "#ff375f")]),
        ("Ignition", "°", vec![("Ign Avg", ign, "#0a84ff")]),
        ("Knock retard", "°", vec![("cyl 1", knk[0], "#0a84ff"), ("cyl 2", knk[1], "#30d158"), ("cyl 3", knk[2], "#ff9f0a"), ("cyl 4", knk[3], "#ff375f")]),
        ("Injection time", "ms", vec![("DI", pw_di, "#0a84ff"), ("MPI", pw_mpi, "#ff9f0a"), ("cycle time", &cyc_ms, "#8e8e93")]),
        ("Injector duty", "% of 720 °CA", vec![("DI", &di_duty, "#0a84ff"), ("MPI", &mpi_duty, "#ff9f0a")]),
        ("DI injection duration", "°CA", vec![("DI", &di_deg, "#0a84ff")]),
        ("Fuel per stroke", "mg", vec![("implied (air/λ)", &implied, "#30d158"), ("commanded DI (ip_ti_4_h)", &cmd_di, "#ff375f")]),
        ("Rail pressure", "bar", vec![("FP DI", fp, "#0a84ff"), ("FP DI SP", fp_sp, "#ff375f"), ("HPFP Eff Vol %", hpfp, "#8e8e93")]),
        ("Wastegate", "% open", vec![("Wastegate", l.v("Wastegate"), "#0a84ff"), ("Wastegate SP", l.v("Wastegate SP"), "#ff375f")]),
        ("Turbo", "krpm, °C", vec![("Turbo Speed", l.v("Turbo Speed"), "#0a84ff"), ("Turbo Air Temp", l.v("Turbo Air Temp"), "#ff9f0a"), ("IAT", l.v("IAT"), "#30d158")]),
        ("Torque", "Nm", vec![("Torque", l.v("Torque"), "#0a84ff"), ("Torque Req", l.v("Torque Req"), "#ff375f")]),
        ("Air mass", "mg/stroke", vec![("Airmass", airmass, "#0a84ff")]),
        ("Fuel trims", "%", vec![("STFT", l.v("STFT"), "#0a84ff"), ("LTFT", l.v("LTFT"), "#ff375f")]),
        ("Cams", "°", vec![("Int Cam Pos", l.v("Int Cam Pos"), "#0a84ff"), ("Exhaust Cam Pos", l.v("Exhaust Cam Pos"), "#ff375f")]),
    ];
    let mut body = String::new();
    for (title, unit, ss) in &plots {
        let series: Vec<Series> = ss.iter().map(|(lb, y, c)| Series { label: lb, t: l.t, y, color: c }).collect();
        body += &svg_plot(title, unit, &series, &spans, tr);
    }
    let html = html_page(&format!("Log forensics — {}", d.source.rsplit('/').next().unwrap_or("")), &format!("<pre>{}</pre>{body}", md.replace('<', "&lt;")));
    Forensics { markdown: md, html }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::log_tools::csv_log::LogData;

    #[test]
    fn phases_split_launch_pull_and_overrun() {
        let mut csv = String::from("Time,Engine Speed,Vehicle Speed,Pedal Pos,Inj PW DI,Inj PW MPI,Gear\n");
        for i in 0..40 {
            let (n, v, p, di) = match i {
                0..=9 => (800, 0, 0, 0.0),
                10..=19 => (3200, 0, 100, 3.0),
                20..=29 => (5000, 60, 100, 7.0),
                _ => (3000, 60, 0, 0.0),
            };
            csv += &format!("{},{n},{v},{p},{di},0,1\n", i as f64 * 0.1);
        }
        let d = LogData::parse(&csv, "t").unwrap();
        let k: Vec<&str> = phases(&Log::new(&d)).iter().map(|p| p.kind).collect();
        assert_eq!(k, ["idle", "launch hold", "WOT pull", "overrun cut-off"]);
    }
}
