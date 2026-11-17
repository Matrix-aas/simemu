//! Test-cell data recorder. Captures the selected channels at every 10 ms co-simulation slice
//! into a bounded ring buffer.
//!
//! Backend-side on purpose: the UI sees one snapshot per `/api/step` (≥ 50 ms of sim time, and
//! browsers throttle timers in background tabs), so UI-side recording would miss slice-level
//! transients (knock bursts, shifts, interlock trips). Values are copied verbatim from the
//! telemetry JSON; nothing is derived here.

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::VecDeque;

/// Value budget of the ring (f64 cells, ≈ 64 MB).
// ponytail: fixed in-RAM ring; stream rows to disk if captures longer than MAX_VALUES/channels slices matter
const MAX_VALUES: usize = 8_000_000;
/// One hour at the 100 Hz slice rate.
const MAX_ROWS: usize = 360_000;
const MAX_CHANNELS: usize = 1024;
/// Rows per `/api/ws/rec` response; the UI pages with `since`.
const PAGE_ROWS: usize = 20_000;

#[derive(Debug, Clone, Serialize)]
pub struct RecEvent {
    pub seq: u64,
    pub t_s: f64,
    pub label: String,
}

struct Row {
    seq: u64,
    t_s: f64,
    v: Vec<f64>,
}

#[derive(Default)]
pub struct Recorder {
    pub active: bool,
    pub channels: Vec<String>,
    rows: VecDeque<Row>,
    next_seq: u64,
    pub events: Vec<RecEvent>,
}

#[derive(Deserialize)]
pub struct RecRequest {
    pub action: String,
    #[serde(default)]
    pub channels: Vec<String>,
    #[serde(default)]
    pub label: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct RecStatus {
    pub active: bool,
    pub channels: usize,
    pub rows: usize,
    pub capacity_rows: usize,
    /// Sequence number of the newest row (monotonic across captures; `None` = empty).
    pub last_seq: Option<u64>,
    pub t_first_s: Option<f64>,
    pub t_last_s: Option<f64>,
    pub events: usize,
}

impl Recorder {
    fn capacity(&self) -> usize {
        (MAX_VALUES / self.channels.len().max(1)).min(MAX_ROWS)
    }

    pub fn status(&self) -> RecStatus {
        RecStatus {
            active: self.active,
            channels: self.channels.len(),
            rows: self.rows.len(),
            capacity_rows: self.capacity(),
            last_seq: self.rows.back().map(|r| r.seq),
            t_first_s: self.rows.front().map(|r| r.t_s),
            t_last_s: self.rows.back().map(|r| r.t_s),
            events: self.events.len(),
        }
    }

    /// Starts a new capture (the previous one is discarded).
    pub fn request(&mut self, r: &RecRequest, t_s: f64) -> Result<(), String> {
        match r.action.as_str() {
            "start" => {
                if r.channels.is_empty() || r.channels.len() > MAX_CHANNELS {
                    return Err(format!("select 1..{MAX_CHANNELS} channels"));
                }
                let next_seq = self.next_seq;
                *self = Self { active: true, channels: r.channels.clone(), next_seq, ..Default::default() };
            }
            "stop" => self.active = false,
            "mark" => {
                let label: String = r.label.chars().take(200).collect();
                self.events.push(RecEvent { seq: self.next_seq, t_s, label });
            }
            "clear" => {
                let next_seq = self.next_seq;
                *self = Self { next_seq, ..Default::default() };
            }
            a => return Err(format!("unknown recorder action {a}")),
        }
        Ok(())
    }

    pub fn capture(&mut self, t_s: f64, snapshot: &Value) {
        if !self.active {
            return;
        }
        let v = self.channels.iter().map(|p| lookup(snapshot, p)).collect();
        if self.rows.len() >= self.capacity() {
            self.rows.pop_front();
        }
        self.rows.push_back(Row { seq: self.next_seq, t_s, v });
        self.next_seq += 1;
    }

    /// Rows newer than `since` (all when `None`), at most one page. Row = `[seq, t_s, values…]`,
    /// NaN (missing / not driven) serialises as `null`.
    pub fn data(&self, since: Option<u64>) -> Value {
        let start = since.map_or(0, |s| self.rows.partition_point(|r| r.seq <= s));
        let rows: Vec<Vec<f64>> = self
            .rows
            .range(start..)
            .take(PAGE_ROWS)
            .map(|r| {
                let mut o = Vec::with_capacity(r.v.len() + 2);
                o.push(r.seq as f64);
                o.push(r.t_s);
                o.extend_from_slice(&r.v);
                o
            })
            .collect();
        let more = self.rows.len() - start > rows.len();
        json!({ "channels": self.channels, "events": self.events, "rows": rows, "more": more, "status": self.status() })
    }
}

/// Resolves `plant.fuel_mg_stroke[2]`-style paths. Booleans → 0/1; missing / null / text → NaN.
pub fn lookup(root: &Value, path: &str) -> f64 {
    let mut v = root;
    for part in path.split('.') {
        let mut it = part.split('[');
        let name = it.next().unwrap_or("");
        v = match v.get(name) {
            Some(x) => x,
            None => return f64::NAN,
        };
        for idx in it {
            v = match idx.trim_end_matches(']').parse::<usize>().ok().and_then(|i| v.get(i)) {
                Some(x) => x,
                None => return f64::NAN,
            };
        }
    }
    match v {
        Value::Number(n) => n.as_f64().unwrap_or(f64::NAN),
        Value::Bool(b) => f64::from(u8::from(*b)),
        _ => f64::NAN,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn records_pages_and_resolves_paths() {
        let snap = json!({ "plant": { "rpm": 812.5, "fuel": [1.0, 2.5], "on": true, "x": null } });
        assert_eq!(lookup(&snap, "plant.fuel[1]"), 2.5);
        assert_eq!(lookup(&snap, "plant.on"), 1.0);
        assert!(lookup(&snap, "plant.x").is_nan() && lookup(&snap, "plant.nope[3]").is_nan());

        let mut r = Recorder::default();
        let start = RecRequest { action: "start".into(), channels: vec!["plant.rpm".into(), "plant.fuel[0]".into()], label: String::new() };
        r.request(&start, 0.0).unwrap();
        for i in 0..5 {
            r.capture(i as f64 * 0.01, &snap);
        }
        let d = r.data(Some(2));
        let rows = d["rows"].as_array().unwrap();
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0][0], 3.0);
        assert_eq!(rows[0][2], 812.5);
        r.request(&RecRequest { action: "stop".into(), channels: vec![], label: String::new() }, 0.0).unwrap();
        r.capture(1.0, &snap);
        assert_eq!(r.status().rows, 5);
    }
}
