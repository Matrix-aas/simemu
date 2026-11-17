//! Workstation back end: data recorder, test-cell interlock, dyno correction factors, DAMOS
//! browsing and the CAN trace feed. All endpoints live under `/api/ws/`.

use crate::studio::dyno_correction::{correction, CorrectionReport};
use crate::studio::recorder::{RecRequest, RecStatus, Recorder};
use crate::studio::safety::{BenchSafety, SafetyLimits};
use crate::studio::session::StudioSession;
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Default)]
pub struct Workstation {
    pub recorder: Recorder,
    pub safety: BenchSafety,
}

/// Small per-step status embedded in `/api/state`.
#[derive(serde::Serialize)]
pub struct WsStatus {
    pub recorder: RecStatus,
    pub safety: BenchSafety,
    pub correction: CorrectionReport,
}

#[derive(Deserialize)]
struct SafetyRequest {
    action: String,
    limits: Option<SafetyLimits>,
}

type RouteResult = Result<Value, (&'static str, String)>;

fn bad(e: impl Into<String>) -> (&'static str, String) {
    ("400 Bad Request", e.into())
}

impl StudioSession {
    /// Bench equipment, run at the start of every 10 ms slice: the interlock (pedal lift + sweep
    /// abort while latched) and the recorder. Costs nothing while both are off.
    pub(crate) fn workstation_tick(&mut self) {
        if !self.ws.recorder.active && !self.ws.safety.limits.enabled {
            return;
        }
        let tel = self.plant.telemetry();
        if self.ws.safety.check(&tel) {
            self.stop_dyno();
        }
        if self.ws.recorder.active {
            let snap = json!({ "plant": tel, "driver": &self.plant.driver });
            self.ws.recorder.capture(tel.sim_time_s, &snap);
        }
    }

    pub(crate) fn ws_status(&self) -> WsStatus {
        let amb = self.plant.ambient.state();
        WsStatus {
            recorder: self.ws.recorder.status(),
            safety: self.ws.safety.clone(),
            correction: correction(amb.p_kpa, amb.p_vapour_kpa, self.plant.ambient.temp_c),
        }
    }
}

fn damos_item(p: &simemu_damos::DamosParameter) -> Value {
    json!({
        "id": p.id, "title": p.title, "kind": p.kind, "unit": p.unit, "category": p.category,
        "rows": p.rows, "cols": p.cols, "address": format!("0x{:08X}", p.cpu_address)
    })
}

/// Dispatches `/api/ws/*`. `q` reads a query-string parameter.
pub fn route(method: &str, path: &str, q: &dyn Fn(&str) -> String, body: &[u8], s: &mut StudioSession) -> RouteResult {
    let num = |k: &str| q(k).parse::<f64>().ok().filter(|v| v.is_finite());
    match (method, path) {
        ("POST", "/api/ws/rec") => {
            let r: RecRequest = serde_json::from_slice(body).map_err(|e| bad(format!("bad JSON: {e}")))?;
            let t = s.plant.sim_time_s;
            s.ws.recorder.request(&r, t).map_err(bad)?;
            Ok(json!(s.ws.recorder.status()))
        }
        ("GET", "/api/ws/rec") => Ok(s.ws.recorder.data(num("since").map(|v| v.max(0.0) as u64))),
        ("POST", "/api/ws/safety") => {
            let r: SafetyRequest = serde_json::from_slice(body).map_err(|e| bad(format!("bad JSON: {e}")))?;
            match r.action.as_str() {
                "set" => {
                    s.ws.safety.limits = r.limits.ok_or_else(|| bad("missing limits"))?;
                    if !s.ws.safety.limits.enabled {
                        s.ws.safety.trip = None; // disarming releases the latch
                    }
                }
                "reset" => s.ws.safety.trip = None,
                a => return Err(bad(format!("unknown safety action {a}"))),
            }
            Ok(json!(s.ws.safety))
        }
        ("GET", "/api/ws/damos/categories") => {
            let mut counts: std::collections::BTreeMap<&str, usize> = Default::default();
            for p in &s.damos.parameters {
                *counts.entry(if p.category.is_empty() { "—" } else { p.category.as_str() }).or_default() += 1;
            }
            let mut v: Vec<_> = counts.into_iter().collect();
            v.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(b.0)));
            Ok(json!({ "total": s.damos.parameters.len(), "categories": v.iter().map(|(n, c)| json!({"name": n, "count": c})).collect::<Vec<_>>() }))
        }
        ("GET", "/api/ws/damos/list") => {
            let (cat, kind, text) = (q("category"), q("kind").to_uppercase(), q("q").to_lowercase());
            let offset = num("offset").unwrap_or(0.0).max(0.0) as usize;
            let limit = num("limit").unwrap_or(200.0).clamp(1.0, 1000.0) as usize;
            let hits: Vec<_> = s
                .damos
                .parameters
                .iter()
                .filter(|p| cat.is_empty() || p.category.eq_ignore_ascii_case(&cat) || (cat == "—" && p.category.is_empty()))
                .filter(|p| kind.is_empty() || p.kind == kind)
                .filter(|p| text.is_empty() || p.id.to_lowercase().contains(&text) || p.title.to_lowercase().contains(&text))
                .collect();
            Ok(json!({ "total": hits.len(), "offset": offset, "items": hits.iter().skip(offset).take(limit).map(|p| damos_item(p)).collect::<Vec<_>>() }))
        }
        ("GET", "/api/ws/damos/meta") => {
            let id = q("id");
            s.damos.get_by_id(&id).map(|p| json!(p)).ok_or(("404 Not Found", format!("map '{id}' not found")))
        }
        ("GET", "/api/ws/can") => {
            let since = num("since_ms").unwrap_or(-1.0);
            let entries: Vec<_> = s.can_log.iter().filter(|e| e.t_ms > since).collect();
            Ok(json!({ "entries": entries, "ecu_tx_total": s.ecu_can_tx_total, "tcu_tx_total": s.tcu_can_tx_total, "sim_time_ms": s.plant.sim_time_s * 1e3 }))
        }
        _ => Err(("404 Not Found", "not found".into())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn non_finite_and_negative_query_params_fall_back_safely() {
        let Some(root) = crate::paths::test_workspace_root() else {
            eprintln!("SKIP: workspace images not found");
            return;
        };
        let mut s = StudioSession::new(&root).expect("session");
        let q = |k: &str| match k {
            "offset" => "-50".into(),
            "limit" => "NaN".into(),
            "since" => "inf".into(),
            "since_ms" => "-inf".into(),
            _ => String::new(),
        };
        let list = route("GET", "/api/ws/damos/list", &q, &[], &mut s).expect("damos list");
        assert_eq!(list["offset"], 0);
        assert_eq!(list["items"].as_array().unwrap().len(), 200);
        let rec = route("GET", "/api/ws/rec", &q, &[], &mut s).expect("rec");
        assert!(rec.is_object());
        let can = route("GET", "/api/ws/can", &q, &[], &mut s).expect("can");
        assert!(can["entries"].is_array());
    }
}
