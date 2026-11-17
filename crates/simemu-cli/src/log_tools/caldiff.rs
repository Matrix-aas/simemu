//! Calibration comparison of two SIMOS18 images by DAMOS name (the images may be different software
//! versions with different addresses, e.g. SC8O40 `264H__0005` vs SC8S50 `264K__0003`).
//! Only parameters present in both definitions with the same shape are compared; axes are compared too,
//! because a changed breakpoint changes the map's meaning even with equal cell values.

use simemu_damos::{DamosDatabase, DamosMatrixData};
use std::path::Path;

pub struct Image {
    pub flash: Vec<u8>,
    pub db: DamosDatabase,
    pub label: String,
}

impl Image {
    pub fn load(root: &Path, file: &str, damos: &str, extra: &[&str]) -> Result<Self, String> {
        let flash = crate::paths::read_firmware(root, file)?;
        if flash.len() != 0x40_0000 {
            return Err(format!("{file}: {} bytes, expected a 4 MB SIMOS18 image", flash.len()));
        }
        let mut db = DamosDatabase::load_from_json(&root.join(damos))?;
        for e in extra {
            db.merge_from_json(&root.join(e))?;
        }
        let code = String::from_utf8_lossy(&flash[0x200002..0x200008]).into_owned();
        Ok(Self { flash, db, label: format!("{file} ({code})") })
    }

    pub fn map(&self, name: &str) -> Option<DamosMatrixData> {
        let p = self.db.get_by_id(name)?;
        self.db.read_matrix(p, &self.flash, &self.flash)
    }
}

/// Bilinear lookup with clamping (rows ↔ y axis, columns ↔ x axis).
pub fn lookup(m: &DamosMatrixData, x: f64, y: f64) -> f64 {
    fn seg(ax: &[f64], v: f64) -> (usize, f64) {
        if ax.len() < 2 {
            return (0, 0.0);
        }
        let i = ax.windows(2).position(|w| v <= w[1]).unwrap_or(ax.len() - 2);
        let f = ((v - ax[i]) / (ax[i + 1] - ax[i])).clamp(0.0, 1.0);
        (i, if f.is_finite() { f } else { 0.0 })
    }
    let (cx, fx) = seg(&m.x_axis, x);
    let (ry, fy) = seg(&m.y_axis, y);
    if m.rows == 0 || m.cols == 0 || m.values.is_empty() {
        return 0.0;
    }
    let at = |r: usize, c: usize| {
        m.values
            .get(r.min(m.rows.saturating_sub(1)))
            .and_then(|row| row.get(c.min(m.cols.saturating_sub(1))).copied())
            .unwrap_or(0.0)
    };
    let row = |r: usize| at(r, cx) * (1.0 - fx) + at(r, cx + 1) * fx;
    row(ry) * (1.0 - fy) + row(ry + 1) * fy
}

pub struct DiffRow {
    pub name: String,
    pub title: String,
    pub unit: String,
    pub shape: (usize, usize),
    pub axes_differ: bool,
    pub cells_changed: usize,
    pub max_abs: f64,
    /// Largest change relative to the B value range (or |B| for scalars), %.
    pub max_rel_pct: f64,
    pub mean_delta: f64,
    pub a_range: (f64, f64),
    pub b_range: (f64, f64),
}

fn range(v: &[Vec<f64>]) -> (f64, f64) {
    v.iter().flatten().fold((f64::MAX, f64::MIN), |(a, b), &x| (a.min(x), b.max(x)))
}

/// Every parameter whose values or axes differ between `a` and `b` (same name, same shape).
pub fn diff(a: &Image, b: &Image) -> (Vec<DiffRow>, usize, Vec<String>) {
    let (mut rows, mut compared, mut shape_mismatch) = (Vec::new(), 0, Vec::new());
    for pa in &a.db.parameters {
        let Some(pb) = b.db.get_by_id(&pa.id) else { continue };
        let (Some(ma), Some(mb)) = (a.db.read_matrix(pa, &a.flash, &a.flash), b.db.read_matrix(pb, &b.flash, &b.flash)) else { continue };
        if (ma.rows, ma.cols) != (mb.rows, mb.cols) {
            shape_mismatch.push(pa.id.clone());
            continue;
        }
        compared += 1;
        let close = |x: f64, y: f64| (x - y).abs() <= 1e-6 * (1.0 + x.abs().max(y.abs()));
        let axes_differ = ma.x_axis.iter().zip(&mb.x_axis).chain(ma.y_axis.iter().zip(&mb.y_axis)).any(|(x, y)| !close(*x, *y));
        let d: Vec<f64> = ma.values.iter().flatten().zip(mb.values.iter().flatten()).map(|(x, y)| x - y).collect();
        let changed = ma.values.iter().flatten().zip(mb.values.iter().flatten()).filter(|(x, y)| !close(**x, **y)).count();
        if changed == 0 && !axes_differ {
            continue;
        }
        let (blo, bhi) = range(&mb.values);
        let (alo, ahi) = range(&ma.values);
        let scale = if bhi - blo > 1e-9 { bhi - blo } else { blo.abs().max(alo.abs()).max(ahi.abs()).max(1e-9) };
        let max_abs = d.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        rows.push(DiffRow {
            name: pa.id.clone(),
            title: pa.title.clone(),
            unit: pa.unit.clone(),
            shape: (ma.rows, ma.cols),
            axes_differ,
            cells_changed: changed,
            max_abs,
            max_rel_pct: 100.0 * max_abs / scale,
            mean_delta: d.iter().sum::<f64>() / d.len().max(1) as f64,
            a_range: (alo, ahi),
            b_range: (blo, bhi),
        });
    }
    (rows, compared, shape_mismatch)
}

/// Name fragments of the calibration groups that set the operating conditions relevant to a ring-land /
/// knock / thermal failure (boost, torque, mixture, spark, knock control, injection, limits, protection).
pub const FAILURE_RELEVANT: &[(&str, &str)] = &[
    ("pre-ignition / LSPI", "pre_ig|preig|pi_det|_pi_|_pi["),
    ("boost", "put_sp|put_max|fac_pow_put|tcha|pq_|wg_|bpa"),
    ("torque / load limits", "tqi_ref|tq_max|tqi_max|tqi_lim|maf_max|rl_max|m_air_max|eff_vol"),
    ("mixture / component protection", "lamb_sp|lamb_bts|lamb_fl|lamb_knk|lamb_cpp|fac_lamb|lamb_max_tq"),
    ("ignition", "iga_bas|iga_opt|iga_max|iga_min|iga_knk|iga_cor|iga_st|iga_tco|iga_tia"),
    ("knock control", "knk"),
    ("injection", "ti_4|mpi|soi|eoi|fup_sp|fup_h|split|mff_spl|cmb_mod"),
    ("rev / speed limits", "n_max|n_lim|vs_max"),
];

pub fn group_of(name: &str) -> Option<&'static str> {
    let n = name.to_lowercase();
    let n = if n.ends_with("_pi") { format!("{n}[") } else { n };
    FAILURE_RELEVANT.iter().find(|(_, pat)| pat.split('|').any(|p| n.contains(p))).map(|(g, _)| *g)
}

/// Values in A (and B where the name exists) of every parameter of `group` — the absolute configuration,
/// changed or not (e.g. whether pre-ignition detection is enabled and in which window).
pub fn group_values(a: &Image, b: &Image, group: &str) -> String {
    values_table(a, b, group, |id| group_of(id) == Some(group))
}

/// Focus parameters: the claims of the superseded forensic audit and the maps that set the failure
/// conditions, shown in absolute values (scalar, or min..max (mean) for curves/maps).
pub const FOCUS: &[&str] = &[
    "lc_iga_tra_knk_pl_ena", "c_n_max_knk_mst_sla_1", "c_n_max_knk_mst_sla_2", "ip_knks_thd_fac[0]", "ip_knks_thd_fac[1]",
    "ip_knks_thd_fac[2]", "ip_knks_thd_fac[3]", "ip_knks_gain[0]", "ip_knks_gain[1]", "ip_knks_gain[2]", "ip_knks_gain[3]",
    "c_lamb_bas_cor_min", "ip_lamb_cop_min", "ip_lamb_tur_ohp_min", "ip_lamb_fl_sp", "ip_ti_4_h", "ip_ti_4_l",
    "ip_iga_min_bas_cbk_sel_port_h[0][0][0]", "ip_iga_bas_cmb_mod_cor[5]", "ip_put_sp", "ip_put_max_cap_h_diag", "ip_pq_cha_max",
    "c_n_tcha_max_sp", "ip_tqi_ref_max_mon", "ip_soi_max_cus", "c_lf_cmb_mod_inh_red", "c_n_max_drof_mon", "c_lamb_sp_knk_pas", "c_n_max_dct", "c_n_max_bas", "c_n_max_thd_neut_gear",
];

pub fn focus_values(a: &Image, b: &Image) -> String {
    values_table(a, b, "focus", |id| FOCUS.iter().any(|f| f.eq_ignore_ascii_case(id)))
}

fn values_table(a: &Image, b: &Image, title: &str, pred: impl Fn(&str) -> bool) -> String {
    let short = |m: Option<DamosMatrixData>| -> String {
        m.map_or("—".into(), |m| {
            let v: Vec<f64> = m.values.iter().flatten().copied().collect();
            if v.len() == 1 {
                return format!("{:.3}", v[0]);
            }
            let (lo, hi) = v.iter().fold((f64::MAX, f64::MIN), |(x, y), &z| (x.min(z), y.max(z)));
            format!("{lo:.3}..{hi:.3} (mean {:.3})", v.iter().sum::<f64>() / v.len() as f64)
        })
    };
    let mut s = format!("| `{title}` parameter | A | B | title |\n|---|---|---|---|\n");
    for p in a.db.parameters.iter().filter(|p| pred(&p.id)) {
        let ma = a.db.read_matrix(p, &a.flash, &a.flash);
        let mb = b.db.get_by_id(&p.id).and_then(|q| b.db.read_matrix(q, &b.flash, &b.flash));
        s += &format!("| `{}` | {} | {} | {} |\n", p.id, short(ma), short(mb), p.title.replace('|', "/"));
    }
    s
}

pub fn to_markdown(a: &Image, b: &Image, rows: &[DiffRow], compared: usize, shape_mismatch: &[String]) -> String {
    let mut s = format!(
        "# Calibration diff\n\nA = {}\nB = {}\n\nCompared {} parameters present in both definitions with the same shape; {} differ; {} have a different shape (not compared).\nΔ = A − B. Different software versions: part of the differences are version changes, not tuning (see doc §4).\n\n",
        a.label,
        b.label,
        compared,
        rows.len(),
        shape_mismatch.len()
    );
    for (g, _) in FAILURE_RELEVANT {
        let mut v: Vec<&DiffRow> = rows.iter().filter(|r| group_of(&r.name) == Some(g)).collect();
        if v.is_empty() {
            continue;
        }
        v.sort_by(|x, y| y.max_rel_pct.total_cmp(&x.max_rel_pct));
        s += &format!("## {g} ({} changed)\n\n| name | shape | cells | max Δ | max Δ % of B range | mean Δ | A range | B range | axes | title |\n|---|---|---|---|---|---|---|---|---|---|\n", v.len());
        for r in v {
            s += &format!(
                "| `{}` | {}×{} | {} | {:.4} {} | {:.1} | {:.4} | {:.3}..{:.3} | {:.3}..{:.3} | {} | {} |\n",
                r.name, r.shape.0, r.shape.1, r.cells_changed, r.max_abs, r.unit, r.max_rel_pct, r.mean_delta, r.a_range.0, r.a_range.1, r.b_range.0, r.b_range.1, if r.axes_differ { "differ" } else { "=" }, r.title.replace('|', "/")
            );
        }
        s.push('\n');
    }
    s
}

pub fn to_csv(rows: &[DiffRow]) -> String {
    let mut s = String::from("name,group,rows,cols,cells_changed,max_abs,max_rel_pct,mean_delta,a_min,a_max,b_min,b_max,axes_differ,unit,title\n");
    for r in rows {
        s += &format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},\"{}\"\n",
            r.name,
            group_of(&r.name).unwrap_or(""),
            r.shape.0,
            r.shape.1,
            r.cells_changed,
            r.max_abs,
            r.max_rel_pct,
            r.mean_delta,
            r.a_range.0,
            r.a_range.1,
            r.b_range.0,
            r.b_range.1,
            r.axes_differ,
            r.unit,
            r.title.replace('"', "'")
        );
    }
    s
}

/// Values of the patch-only parameters (e.g. `O30_Switch_Patch`) in image A.
pub fn patch_values(a: &Image, patch_json: &Path) -> Result<String, String> {
    let p = DamosDatabase::load_from_json(patch_json)?;
    let mut s = String::from("| patch parameter | value(s) | unit |\n|---|---|---|\n");
    for q in &p.parameters {
        let Some(m) = p.read_matrix(q, &a.flash, &a.flash) else { continue };
        let vals: Vec<String> = m.values.iter().flatten().take(24).map(|v| format!("{v:.3}")).collect();
        let more = if m.rows * m.cols > 24 { " …" } else { "" };
        s += &format!("| `{}` | {}{} | {} |\n", q.id, vals.join(" "), more, q.unit);
    }
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bilinear_lookup_interpolates_and_clamps() {
        let m = DamosMatrixData {
            id: String::new(),
            title: String::new(),
            kind: String::new(),
            unit: String::new(),
            cpu_address: 0,
            rows: 2,
            cols: 2,
            x_unit: String::new(),
            y_unit: String::new(),
            x_axis: vec![0.0, 10.0],
            y_axis: vec![0.0, 1.0],
            values: vec![vec![0.0, 10.0], vec![100.0, 110.0]],
            stock_values: vec![],
        };
        assert!((lookup(&m, 5.0, 0.5) - 55.0).abs() < 1e-9);
        assert!((lookup(&m, 50.0, 2.0) - 110.0).abs() < 1e-9);
        assert_eq!(group_of("ip_put_sp"), Some("boost"));
        assert_eq!(group_of("c_knk_thd"), Some("knock control"));
        assert_eq!(group_of("ip_fac_knk_pi_fast[0]"), Some("pre-ignition / LSPI"));
        assert_eq!(group_of("c_tco_min_pi"), Some("pre-ignition / LSPI"));

        // Empty / 0-sized matrices must never underflow `rows - 1` or panic.
        let empty = DamosMatrixData {
            rows: 0,
            cols: 0,
            x_axis: vec![],
            y_axis: vec![],
            values: vec![],
            ..m
        };
        assert_eq!(lookup(&empty, 1.0, 2.0), 0.0);
    }

    /// Diagnostic print: DI injector characteristics of SC8O40 and SC8S50 at the same (mass, ΔP) points.
    #[test]
    #[ignore]
    fn compare_injector_maps() {
        let Some(root) = crate::paths::test_workspace_root() else { return };
        let Ok(a) = Image::load(&root, crate::paths::DEFAULT_ECU_FLASH, "definitions/SC8O40.json", &[]) else { return };
        let Ok(b) = Image::load(&root, crate::paths::STOCK_264K_FLASH, "definitions/SC8S50.json", &[]) else { return };
        for name in ["ip_ti_4_l", "ip_ti_4_h"] {
            let (ma, mb) = (a.map(name).unwrap(), b.map(name).unwrap());
            println!("{name}: A x [{}] {:?} y [{}] {:?}\n  B x {:?} y {:?}", ma.x_unit, ma.x_axis, ma.y_unit, ma.y_axis, mb.x_axis, mb.y_axis);
            let at = |m: &DamosMatrixData, mg: f64, hpa: f64| if m.x_unit.to_lowercase().contains("mg") { lookup(m, mg, hpa) } else { lookup(m, hpa, mg) };
            for (mg, hpa) in [(5.0, 50_000.0), (10.0, 100_000.0), (20.0, 100_000.0), (40.0, 150_000.0), (60.0, 200_000.0)] {
                println!("  {mg} mg @ {hpa} hPa: A {:.3} ms, B {:.3} ms", at(&ma, mg, hpa), at(&mb, mg, hpa));
            }
            // Least squares ti = m / (q·√(ΔP/10 MPa)) + t0 over 10–50 mg and 4–15 MPa (the profile's injector model).
            for (lbl, m) in [("A", &ma), ("B", &mb)] {
                let pts: Vec<(f64, f64)> = (0..=8).flat_map(|i| (0..=11).map(move |j| (10.0 + 5.0 * i as f64, 40_000.0 + 10_000.0 * j as f64))).map(|(mg, hpa)| (mg / (hpa / 100_000.0f64).sqrt(), at(m, mg, hpa))).collect();
                let n = pts.len() as f64;
                let (sx, sy, sxx, sxy) = pts.iter().fold((0.0, 0.0, 0.0, 0.0), |(a, b, c, d), (x, y)| (a + x, b + y, c + x * x, d + x * y));
                let k = (n * sxy - sx * sy) / (n * sxx - sx * sx);
                let t0 = (sy - k * sx) / n;
                let res = pts.iter().map(|(x, y)| (y - k * x - t0).abs()).fold(0.0, f64::max);
                println!("  fit {lbl}: q = {:.2} mg/ms at 10 MPa = {:.2} cm³/s at 745 kg/m³, t0 = {t0:.3} ms, max residual {res:.3} ms", 1.0 / k, 1.0 / k / 0.745);
            }
        }
    }
}
