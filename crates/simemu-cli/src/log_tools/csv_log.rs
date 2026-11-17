//! Loader for vehicle data logs exported as delimited text (SimosTools / VW_Flash / VCDS-style CSV).
//!
//! Robust to: `,` `;` or tab delimiters (decimal commas when `;`/tab), units in the header (`Name (unit)`,
//! `Name [unit]`), a time column given as clock time (`HH:MM:SS.ffffff`, midnight wrap handled), seconds or
//! milliseconds, empty / non-numeric cells (NaN), any column set and order. Time is re-based to the first row.

use std::path::Path;

#[derive(Debug, Clone)]
pub struct LogData {
    /// Channel names with the unit suffix removed (header order, time column excluded).
    pub names: Vec<String>,
    /// Unit parsed from the header (`""` if none).
    pub units: Vec<String>,
    /// Sample time, s, from the first row.
    pub t: Vec<f64>,
    /// `cols[k][i]` = channel k at sample i (NaN = missing).
    pub cols: Vec<Vec<f64>>,
    pub source: String,
}

fn split_unit(h: &str) -> (String, String) {
    let h = h.trim().trim_matches('"').trim();
    for (o, c) in [('(', ')'), ('[', ']')] {
        if let (Some(a), true) = (h.rfind(o), h.ends_with(c)) {
            return (h[..a].trim().to_string(), h[a + 1..h.len() - 1].trim().to_string());
        }
    }
    (h.to_string(), String::new())
}

fn parse_num(s: &str, decimal_comma: bool) -> f64 {
    let s = s.trim().trim_matches('"');
    let owned;
    let s = if decimal_comma {
        owned = s.replace(',', ".");
        owned.as_str()
    } else {
        s
    };
    s.parse().unwrap_or(f64::NAN)
}

/// `HH:MM:SS(.f)` or `MM:SS(.f)` → seconds; plain numbers pass through.
fn parse_time(s: &str, decimal_comma: bool) -> f64 {
    let s = s.trim().trim_matches('"');
    if s.contains(':') {
        let p: Vec<f64> = s.split(':').map(|x| parse_num(x, decimal_comma)).collect();
        return p.iter().fold(0.0, |acc, &x| acc * 60.0 + x);
    }
    parse_num(s, decimal_comma)
}

impl LogData {
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&String::from_utf8_lossy(&text), &path.display().to_string())
    }

    pub fn parse(text: &str, source: &str) -> Result<Self, String> {
        let mut lines = text.lines().map(|l| l.trim_start_matches('\u{feff}')).filter(|l| !l.trim().is_empty());
        let header = lines.next().ok_or("empty log")?;
        let delim = [';', '\t', ','].into_iter().max_by_key(|d| header.matches(*d).count()).unwrap_or(',');
        let decimal_comma = delim != ',';
        let heads: Vec<(String, String)> = header.split(delim).map(split_unit).collect();
        let tcol = heads
            .iter()
            .position(|(n, _)| matches!(n.to_lowercase().as_str(), "time" | "timestamp" | "t" | "zeit" | "время"))
            .unwrap_or(0);
        let t_scale = match heads[tcol].1.to_lowercase().as_str() {
            "ms" => 1e-3,
            "us" | "µs" => 1e-6,
            _ => 1.0,
        };
        let keep: Vec<usize> = (0..heads.len()).filter(|&k| k != tcol).collect();
        let mut t = Vec::new();
        let mut cols = vec![Vec::new(); keep.len()];
        for line in lines {
            let cells: Vec<&str> = line.split(delim).collect();
            let Some(tv) = cells.get(tcol).map(|c| parse_time(c, decimal_comma) * t_scale).filter(|x| x.is_finite()) else {
                continue;
            };
            t.push(tv);
            for (j, &k) in keep.iter().enumerate() {
                cols[j].push(cells.get(k).map_or(f64::NAN, |c| parse_num(c, decimal_comma)));
            }
        }
        if t.len() < 2 {
            return Err(format!("{source}: fewer than two samples"));
        }
        // Clock time across midnight, then re-base.
        for i in 1..t.len() {
            while t[i] < t[i - 1] - 43_200.0 {
                t[i] += 86_400.0;
            }
        }
        let t0 = t[0];
        t.iter_mut().for_each(|x| *x -= t0);
        Ok(Self {
            names: keep.iter().map(|&k| heads[k].0.clone()).collect(),
            units: keep.iter().map(|&k| heads[k].1.clone()).collect(),
            t,
            cols,
            source: source.to_string(),
        })
    }

    /// Column of the first name that matches (case-insensitive, unit suffix ignored).
    pub fn col(&self, names: &[&str]) -> Option<&[f64]> {
        names.iter().find_map(|n| self.names.iter().position(|h| h.eq_ignore_ascii_case(n))).map(|k| self.cols[k].as_slice())
    }

    pub fn duration(&self) -> f64 {
        *self.t.last().unwrap_or(&0.0)
    }
}

/// Linear interpolation of `y(t)` at `x` on a monotonic time base (clamped at the ends, NaN-tolerant
/// neighbours fall back to the nearest finite sample).
pub fn interp(t: &[f64], y: &[f64], x: f64) -> f64 {
    let i = t.partition_point(|&v| v <= x);
    match i {
        0 => y[0],
        i if i >= t.len() => y[t.len() - 1],
        i => {
            let (t0, t1, y0, y1) = (t[i - 1], t[i], y[i - 1], y[i]);
            if !y0.is_finite() || !y1.is_finite() || t1 <= t0 {
                return if y0.is_finite() { y0 } else { y1 };
            }
            y0 + (y1 - y0) * (x - t0) / (t1 - t0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_clock_time_units_delimiters_and_gaps() {
        let a = LogData::parse("Time,Engine Speed (rpm),MAP [kPa],X\n23:59:59.5,800,34.4,\n00:00:00.5,810,35,1\n", "a").unwrap();
        assert_eq!(a.names, ["Engine Speed", "MAP", "X"]);
        assert_eq!(a.units, ["rpm", "kPa", ""]);
        assert!((a.t[1] - 1.0).abs() < 1e-9, "midnight wrap {:?}", a.t);
        assert!(a.col(&["x"]).unwrap()[0].is_nan());
        assert_eq!(a.col(&["map"]).unwrap()[1], 35.0);
        let b = LogData::parse("Time (ms);Lambda\n0;0,98\n100;1,02\n", "b").unwrap();
        assert!((b.t[1] - 0.1).abs() < 1e-12 && (b.cols[0][1] - 1.02).abs() < 1e-12);
        assert!((interp(&b.t, &b.cols[0], 0.05) - 1.0).abs() < 1e-12);
    }
}
