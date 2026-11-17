//! Self-contained HTML reports with inline SVG plots (no plotting dependency) and error statistics.

use super::csv_log::interp;
use std::fmt::Write;

pub struct Series<'a> {
    pub label: &'a str,
    pub t: &'a [f64],
    pub y: &'a [f64],
    pub color: &'a str,
}

/// Shaded time window with a label (phases).
pub struct Span<'a> {
    pub t0: f64,
    pub t1: f64,
    pub label: &'a str,
}

const W: f64 = 960.0;
const H: f64 = 220.0;
const L: f64 = 56.0;
const R: f64 = 12.0;
const T: f64 = 22.0;
const B: f64 = 24.0;

fn esc(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// Line plot of several series on a shared time axis.
pub fn svg_plot(title: &str, unit: &str, series: &[Series], spans: &[Span], t_range: (f64, f64)) -> String {
    let (t0, t1) = t_range;
    let finite = series.iter().flat_map(|s| s.t.iter().zip(s.y).filter(|(t, y)| y.is_finite() && **t >= t0 && **t <= t1).map(|(_, y)| *y));
    let (mut lo, mut hi) = finite.fold((f64::MAX, f64::MIN), |(a, b), y| (a.min(y), b.max(y)));
    if lo > hi {
        (lo, hi) = (0.0, 1.0);
    }
    if hi - lo < 1e-9 {
        (lo, hi) = (lo - 1.0, hi + 1.0);
    }
    let pad = (hi - lo) * 0.05;
    let (lo, hi) = (lo - pad, hi + pad);
    let x = |t: f64| L + (t - t0) / (t1 - t0).max(1e-9) * (W - L - R);
    let y = |v: f64| T + (hi - v) / (hi - lo) * (H - T - B);
    let mut s = format!(r#"<svg viewBox="0 0 {W} {H}" class="plot" role="img" aria-label="{}"><text x="{L}" y="15" class="ttl">{} [{}]</text>"#, esc(title), esc(title), esc(unit));
    for sp in spans {
        let (a, b) = (x(sp.t0.max(t0)), x(sp.t1.min(t1)));
        if b > a {
            let _ = write!(s, r#"<rect x="{a:.1}" y="{T}" width="{:.1}" height="{:.1}" class="span"/><text x="{:.1}" y="{:.1}" class="spl">{}</text>"#, b - a, H - T - B, a + 2.0, T + 10.0, esc(sp.label));
        }
    }
    for k in 0..=4 {
        let v = lo + (hi - lo) * k as f64 / 4.0;
        let _ = write!(s, r#"<line x1="{L}" x2="{:.1}" y1="{:.1}" y2="{:.1}" class="grid"/><text x="{:.1}" y="{:.1}" class="ax" text-anchor="end">{}</text>"#, W - R, y(v), y(v), L - 4.0, y(v) + 4.0, fmt_num(v));
    }
    let step = nice_step((t1 - t0) / 10.0);
    let mut tt = (t0 / step).ceil() * step;
    while tt <= t1 {
        let _ = write!(s, r#"<text x="{:.1}" y="{:.1}" class="ax" text-anchor="middle">{}</text>"#, x(tt), H - 8.0, fmt_num(tt));
        tt += step;
    }
    for (i, se) in series.iter().enumerate() {
        let mut d = String::new();
        let mut pen = false;
        for (&t, &v) in se.t.iter().zip(se.y) {
            if !v.is_finite() || t < t0 || t > t1 {
                pen = false;
                continue;
            }
            let _ = write!(d, "{}{:.1},{:.1}", if pen { " L" } else { " M" }, x(t), y(v));
            pen = true;
        }
        let _ = write!(s, r#"<path d="{d}" fill="none" stroke="{}" stroke-width="1.4"/><text x="{:.1}" y="15" class="lg" fill="{}">■ {}</text>"#, se.color, W - R - 150.0 * (series.len() - i) as f64, se.color, esc(se.label));
    }
    s.push_str("</svg>");
    s
}

fn nice_step(raw: f64) -> f64 {
    let p = 10f64.powf(raw.max(1e-9).log10().floor());
    [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * p).find(|&v| v >= raw).unwrap_or(10.0 * p)
}

pub fn fmt_num(v: f64) -> String {
    if !v.is_finite() {
        "—".into()
    } else if v.abs() >= 100.0 {
        format!("{v:.0}")
    } else if v.abs() >= 10.0 {
        format!("{v:.1}")
    } else {
        format!("{v:.2}")
    }
}

pub fn html_page(title: &str, body: &str) -> String {
    format!(
        r#"<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{}</title><style>
:root{{--bg:#fff;--fg:#1d1d1f;--mut:#6e6e73;--grid:#e5e5ea;--span:rgba(255,159,10,.10)}}
@media (prefers-color-scheme:dark){{:root{{--bg:#1c1c1e;--fg:#f2f2f7;--mut:#a1a1a6;--grid:#3a3a3c;--span:rgba(255,159,10,.16)}}}}
body{{background:var(--bg);color:var(--fg);font:14px/1.45 -apple-system,system-ui,sans-serif;margin:0 auto;max-width:1000px;padding:16px}}
.plot{{width:100%;height:auto;display:block;margin:6px 0 14px}} .ttl{{font-size:12px;font-weight:600;fill:var(--fg)}}
.ax{{font-size:10px;fill:var(--mut)}} .lg{{font-size:11px}} .grid{{stroke:var(--grid)}} .span{{fill:var(--span)}} .spl{{font-size:9px;fill:var(--mut)}}
table{{border-collapse:collapse;font-size:12px;width:100%;overflow-x:auto;display:block}} td,th{{border-bottom:1px solid var(--grid);padding:3px 6px;text-align:right}} td:first-child,th:first-child{{text-align:left}}
pre{{white-space:pre-wrap;font-size:12px}}
</style></head><body><h1>{}</h1>{body}</body></html>"#,
        esc(title),
        esc(title)
    )
}

/// Error statistics of `sim` against `log`, the simulation sampled at the log's time stamps inside
/// `[t0, t1]` (both finite).
#[derive(Debug, Clone, Copy, Default)]
pub struct ErrStats {
    pub n: usize,
    pub log_mean: f64,
    pub sim_mean: f64,
    pub bias: f64,
    pub mae: f64,
    pub rmse: f64,
    pub max_abs: f64,
    pub r: f64,
}

pub fn err_stats(log_t: &[f64], log_y: &[f64], sim_t: &[f64], sim_y: &[f64], t0: f64, t1: f64) -> ErrStats {
    if sim_t.len() < 2 {
        return ErrStats::default();
    }
    let pairs: Vec<(f64, f64)> = log_t
        .iter()
        .zip(log_y)
        .filter(|(t, y)| **t >= t0 && **t <= t1 && y.is_finite())
        .map(|(&t, &y)| (y, interp(sim_t, sim_y, t)))
        .filter(|(_, s)| s.is_finite())
        .collect();
    let n = pairs.len();
    if n == 0 {
        return ErrStats::default();
    }
    let nf = n as f64;
    let (ml, ms) = (pairs.iter().map(|p| p.0).sum::<f64>() / nf, pairs.iter().map(|p| p.1).sum::<f64>() / nf);
    let e: Vec<f64> = pairs.iter().map(|(l, s)| s - l).collect();
    let (sll, sss, sls) = pairs.iter().fold((0.0, 0.0, 0.0), |(a, b, c), (l, s)| (a + (l - ml).powi(2), b + (s - ms).powi(2), c + (l - ml) * (s - ms)));
    ErrStats {
        n,
        log_mean: ml,
        sim_mean: ms,
        bias: e.iter().sum::<f64>() / nf,
        mae: e.iter().map(|x| x.abs()).sum::<f64>() / nf,
        rmse: (e.iter().map(|x| x * x).sum::<f64>() / nf).sqrt(),
        max_abs: e.iter().fold(0.0, |a, x| a.max(x.abs())),
        r: if sll > 0.0 && sss > 0.0 { sls / (sll * sss).sqrt() } else { f64::NAN },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stats_of_an_offset_copy() {
        let t: Vec<f64> = (0..11).map(|i| i as f64).collect();
        let y: Vec<f64> = t.iter().map(|x| 2.0 * x).collect();
        let s: Vec<f64> = y.iter().map(|x| x + 1.0).collect();
        let e = err_stats(&t, &y, &t, &s, 0.0, 10.0);
        assert_eq!(e.n, 11);
        assert!((e.bias - 1.0).abs() < 1e-12 && (e.rmse - 1.0).abs() < 1e-12 && (e.r - 1.0).abs() < 1e-12);
        assert!(svg_plot("x", "u", &[Series { label: "a", t: &t, y: &y, color: "#06c" }], &[], (0.0, 10.0)).contains("<path"));
    }
}
