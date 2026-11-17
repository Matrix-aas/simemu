use crate::paths;
use crate::studio::assets::studio_index_html;
use crate::studio::session::{DynoRequest, PresetRequest, StepRequest, StudioSession, UdsRequest, WriteMapRequest};
use serde::de::DeserializeOwned;
use serde::Serialize;
use std::io::{ErrorKind, Read, Write};
use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// `POST /api/check` body.
#[derive(serde::Deserialize)]
struct CheckApi {
    action: String,
    image: Option<String>,
    reference: Option<String>,
    mm_slot: Option<u8>,
}

const MAX_HTTP_HEADER_BYTES: usize = 16 * 1024;
const MAX_HTTP_BODY_BYTES: usize = 4 * 1024 * 1024;
const HTTP_READ_DEADLINE: Duration = Duration::from_secs(5);

/// No CORS headers: the API is same-origin only. Cross-site pages cannot send JSON POSTs
/// (non-simple content type triggers a preflight we never answer) and cannot read responses.
fn send_json_bytes(mut stream: TcpStream, status: &str, body: &[u8]) {
    let header = format!(
        "HTTP/1.1 {status}\r\nContent-Type: application/json; charset=utf-8\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        body.len()
    );
    let _ = stream.write_all(header.as_bytes());
    let _ = stream.write_all(body);
}

fn send_json<T: Serialize>(stream: TcpStream, status: &str, payload: &T) {
    let body = serde_json::to_vec(payload).unwrap_or_default();
    send_json_bytes(stream, status, &body);
}

fn send_bytes(mut stream: TcpStream, content_type: &str, extra_headers: &str, body: &[u8]) {
    let header = format!(
        "HTTP/1.1 200 OK\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\n{extra_headers}X-Content-Type-Options: nosniff\r\nConnection: close\r\n\r\n",
        body.len()
    );
    let _ = stream.write_all(header.as_bytes());
    let _ = stream.write_all(body);
}

fn error(stream: TcpStream, status: &str, msg: impl Into<String>) {
    send_json(stream, status, &serde_json::json!({ "error": msg.into() }));
}

fn url_decode(s: &str) -> String {
    let mut out = Vec::with_capacity(s.len());
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'%' if i + 2 < b.len() => match u8::from_str_radix(std::str::from_utf8(&b[i + 1..i + 3]).unwrap_or("zz"), 16) {
                Ok(v) => {
                    out.push(v);
                    i += 3;
                    continue;
                }
                Err(_) => out.push(b'%'),
            },
            b'+' => out.push(b' '),
            c => out.push(c),
        }
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn query_param(query: &str, key: &str) -> String {
    query
        .split('&')
        .filter_map(|kv| kv.split_once('='))
        .find(|(k, _)| *k == key)
        .map(|(_, v)| url_decode(v))
        .unwrap_or_default()
}

fn headers<'a>(head: &'a str, name: &str) -> impl Iterator<Item = &'a str> {
    let target = name.to_ascii_lowercase();
    head.lines().skip(1).filter_map(move |l| {
        let (k, v) = l.split_once(':')?;
        k.trim().eq_ignore_ascii_case(&target).then(|| v.trim())
    })
}

fn header<'a>(head: &'a str, name: &str) -> Option<&'a str> {
    headers(head, name).next()
}

/// Only accept requests addressed to the loopback listener (defeats DNS rebinding).
pub(crate) fn host_allowed(host: Option<&str>, port: u16) -> bool {
    let Some(h) = host.map(str::trim) else { return false };
    let with_port = [format!("127.0.0.1:{port}"), format!("localhost:{port}"), format!("[::1]:{port}")];
    if with_port.iter().any(|a| a.eq_ignore_ascii_case(h)) {
        return true;
    }
    port == 80 && ["127.0.0.1", "localhost", "[::1]"].iter().any(|a| a.eq_ignore_ascii_case(h))
}

/// Validates browser `Origin` header against our loopback HTTP origin (used by POST & WebSocket).
pub(crate) fn origin_allowed(origin: &str, port: u16) -> bool {
    let o = origin.trim();
    let with_port = [
        format!("http://127.0.0.1:{port}"),
        format!("http://localhost:{port}"),
        format!("http://[::1]:{port}"),
    ];
    if with_port.iter().any(|a| a.eq_ignore_ascii_case(o)) {
        return true;
    }
    port == 80
        && ["http://127.0.0.1", "http://localhost", "http://[::1]"]
            .iter()
            .any(|a| a.eq_ignore_ascii_case(o))
}

/// Checks `Sec-Fetch-Site` and `Origin` headers against cross-origin CSRF / DNS rebinding.
pub(crate) fn csrf_allowed(head: &str, port: u16) -> Result<(), &'static str> {
    if let Some(site) = header(head, "sec-fetch-site") {
        if !site.eq_ignore_ascii_case("same-origin") && !site.eq_ignore_ascii_case("none") {
            return Err("cross-origin request rejected (Sec-Fetch-Site)");
        }
    }
    if let Some(origin) = header(head, "origin") {
        if !origin_allowed(origin, port) {
            return Err("bad Origin header");
        }
    }
    Ok(())
}

fn is_json_content_type(ct: Option<&str>) -> bool {
    let Some(ct) = ct else { return false };
    let media = ct.split(';').next().unwrap_or("").trim();
    media.eq_ignore_ascii_case("application/json")
}

fn parse_content_length(head: &str) -> Result<usize, (&'static str, &'static str)> {
    if header(head, "transfer-encoding").is_some() {
        return Err(("411 Length Required", "Transfer-Encoding is not supported; provide Content-Length"));
    }
    let mut vals = headers(head, "content-length");
    let Some(first) = vals.next() else { return Ok(0) };
    if vals.any(|v| v != first) {
        return Err(("400 Bad Request", "conflicting Content-Length headers"));
    }
    let len = first
        .parse::<usize>()
        .map_err(|_| ("400 Bad Request", "invalid Content-Length header"))?;
    if len > MAX_HTTP_BODY_BYTES {
        return Err(("413 Payload Too Large", "request exceeds maximum body size"));
    }
    Ok(len)
}

pub fn handle_client(mut stream: TcpStream, session: Arc<Mutex<StudioSession>>, port: u16) {
    let deadline = Instant::now() + HTTP_READ_DEADLINE;
    let mut buf = vec![0u8; 8192];
    let mut total = 0usize;
    let mut header_end: Option<usize> = None;
    let mut content_length = 0usize;
    let mut head = String::new();

    loop {
        if let Some(h) = header_end {
            if total >= h + content_length {
                break;
            }
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return error(stream, "408 Request Timeout", "request read timed out");
        }
        let _ = stream.set_read_timeout(Some(remaining));

        if total == buf.len() {
            let max_cap = match header_end {
                None => MAX_HTTP_HEADER_BYTES + 4,
                Some(h) => h + content_length,
            };
            if buf.len() >= max_cap {
                return error(
                    stream,
                    "431 Request Header Fields Too Large",
                    "HTTP headers exceed 16 KB limit",
                );
            }
            buf.resize((buf.len() * 2).min(max_cap), 0);
        }

        match stream.read(&mut buf[total..]) {
            Ok(0) => {
                if let Some(h) = header_end {
                    if total < h + content_length {
                        return error(stream, "400 Bad Request", "incomplete request body");
                    }
                }
                break;
            }
            Err(e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock) => {
                return error(stream, "408 Request Timeout", "request read timed out");
            }
            Err(_) => return,
            Ok(n) => {
                let prev_total = total;
                total += n;
                if header_end.is_none() {
                    let scan_from = prev_total.saturating_sub(3);
                    if let Some(rel) = buf[scan_from..total].windows(4).position(|w| w == b"\r\n\r\n") {
                        let pos = scan_from + rel;
                        if pos > MAX_HTTP_HEADER_BYTES {
                            return error(
                                stream,
                                "431 Request Header Fields Too Large",
                                "HTTP headers exceed 16 KB limit",
                            );
                        }
                        let hdr_end = pos + 4;
                        header_end = Some(hdr_end);
                        head = String::from_utf8_lossy(&buf[..hdr_end]).into_owned();

                        if !host_allowed(header(&head, "host"), port) {
                            return error(stream, "403 Forbidden", "bad Host header");
                        }
                        let method_peek = head
                            .lines()
                            .next()
                            .unwrap_or("")
                            .split_whitespace()
                            .next()
                            .unwrap_or("");
                        if method_peek == "POST" {
                            if let Err(msg) = csrf_allowed(&head, port) {
                                return error(stream, "403 Forbidden", msg);
                            }
                            if !is_json_content_type(header(&head, "content-type")) {
                                return error(
                                    stream,
                                    "415 Unsupported Media Type",
                                    "POST requires application/json",
                                );
                            }
                        }
                        content_length = match parse_content_length(&head) {
                            Ok(cl) => cl,
                            Err((status, msg)) => return error(stream, status, msg),
                        };
                        let needed = hdr_end + content_length;
                        if buf.len() < needed {
                            buf.resize(needed.min(hdr_end + 65536), 0);
                        }
                    } else if total > MAX_HTTP_HEADER_BYTES {
                        return error(
                            stream,
                            "431 Request Header Fields Too Large",
                            "HTTP headers exceed 16 KB limit",
                        );
                    }
                }
            }
        }
    }
    let Some(hdr_end) = header_end else { return };
    if total < hdr_end + content_length {
        return error(stream, "400 Bad Request", "incomplete request body");
    }
    let mut first = head.lines().next().unwrap_or("").split_whitespace();
    let method = first.next().unwrap_or("");
    let full_path = first.next().unwrap_or("/");
    let (path, query) = full_path.split_once('?').unwrap_or((full_path, ""));
    let body = &buf[hdr_end..hdr_end + content_length];

    fn parse<T: DeserializeOwned>(body: &[u8]) -> Result<T, String> {
        serde_json::from_slice(body).map_err(|e| format!("bad JSON: {e}"))
    }

    let lock = || session.lock().unwrap_or_else(|e| e.into_inner());
    match (method, path) {
        ("GET", "/") | ("GET", "/index.html") => {
            send_bytes(stream, "text/html; charset=utf-8", "", studio_index_html().as_bytes());
        }
        ("GET", "/api/state") => {
            let body = {
                let s = lock();
                serde_json::to_vec(&s.state()).unwrap_or_default()
            };
            send_json_bytes(stream, "200 OK", &body);
        }
        ("GET", "/api/map") => {
            let id = query_param(query, "id");
            let m = {
                let s = lock();
                s.damos.get_by_id(&id).and_then(|p| s.damos.read_matrix(p, &s.current_flash, &s.stock_flash))
            };
            match m {
                Some(m) => send_json(stream, "200 OK", &m),
                None => error(stream, "404 Not Found", format!("map '{id}' not found")),
            }
        }
        ("GET", "/api/search") => {
            let q = query_param(query, "q");
            let hits: Vec<serde_json::Value> = {
                let s = lock();
                s.damos
                    .search(&q, 60)
                    .into_iter()
                    .map(|p| {
                        serde_json::json!({
                            "id": p.id, "title": p.title, "kind": p.kind, "unit": p.unit,
                            "rows": p.rows, "cols": p.cols, "address": format!("0x{:08X}", p.cpu_address)
                        })
                    })
                    .collect()
            };
            send_json(stream, "200 OK", &hits);
        }
        ("GET", "/api/export_bin") => {
            let mut img = lock().current_flash.clone();
            let res = simemu_damos::fix_checksums(&mut img).map(|_| img);
            match res {
                Ok(img) => send_bytes(stream, "application/octet-stream", "Content-Disposition: attachment; filename=\"simos18_edited.bin\"\r\n", &img),
                Err(e) => error(stream, "500 Internal Server Error", e),
            }
        }
        ("POST", "/api/step") => match parse::<StepRequest>(body) {
            Ok(r) => {
                let body = {
                    let mut s = lock();
                    s.apply_step_request(&r);
                    serde_json::to_vec(&s.state()).unwrap_or_default()
                };
                send_json_bytes(stream, "200 OK", &body);
            }
            Err(e) => error(stream, "400 Bad Request", e),
        },
        ("POST", "/api/map/write") => match parse::<WriteMapRequest>(body) {
            Ok(r) => {
                let res = {
                    let mut s = lock();
                    s.write_map(&r.id, &r.values).map(|()| {
                        s.active_preset = "custom".into();
                        s.damos.get_by_id(&r.id).and_then(|p| s.damos.read_matrix(p, &s.current_flash, &s.stock_flash))
                    })
                };
                match res {
                    Ok(map) => send_json(stream, "200 OK", &serde_json::json!({ "map": map })),
                    Err(e) => error(stream, "400 Bad Request", e),
                }
            }
            Err(e) => error(stream, "400 Bad Request", e),
        },
        ("POST", "/api/preset") => match parse::<PresetRequest>(body) {
            Ok(r) => {
                let res = {
                    let mut s = lock();
                    if r.action == "load_image" {
                        if let Some(img) = r.image.as_deref() {
                            if let Err(e) = paths::resolve_sandboxed_firmware(&s.root, img) {
                                return error(stream, "400 Bad Request", e);
                            }
                        }
                    }
                    s.apply_preset(&r).map(|()| serde_json::to_vec(&s.state()).unwrap_or_default())
                };
                match res {
                    Ok(body) => send_json_bytes(stream, "200 OK", &body),
                    Err(e) => error(stream, "400 Bad Request", e),
                }
            }
            Err(e) => error(stream, "400 Bad Request", e),
        },
        ("POST", "/api/dyno") => match parse::<DynoRequest>(body) {
            Ok(r) => {
                let body = {
                    let mut s = lock();
                    s.dyno_request(&r);
                    serde_json::to_vec(&s.state()).unwrap_or_default()
                };
                send_json_bytes(stream, "200 OK", &body);
            }
            Err(e) => error(stream, "400 Bad Request", e),
        },
        ("POST", "/api/uds") => match parse::<UdsRequest>(body) {
            Ok(r) => {
                let res = {
                    let mut s = lock();
                    s.uds_request(&r).map(|()| serde_json::to_vec(&s.state()).unwrap_or_default())
                };
                match res {
                    Ok(body) => send_json_bytes(stream, "200 OK", &body),
                    Err(e) => error(stream, "400 Bad Request", e),
                }
            }
            Err(e) => error(stream, "400 Bad Request", e),
        },
        ("GET", "/api/audio/ws") => {
            let sh = lock().audio.shared.clone();
            crate::studio::audio::serve_ws(stream, &head, port, sh);
        }
        ("GET", "/api/audio/info") => send_json(stream, "200 OK", &crate::studio::audio::info()),
        ("POST", "/api/audio/harness") => match parse::<crate::studio::audio::Harness>(body) {
            Ok(h) => {
                let harness = {
                    let mut s = lock();
                    s.harness = h;
                    s.harness
                };
                send_json(stream, "200 OK", &harness);
            }
            Err(e) => error(stream, "400 Bad Request", e),
        },
        ("GET", "/api/audio/harness") => {
            let harness = lock().harness;
            send_json(stream, "200 OK", &harness);
        }
        // pre-flash check (studio::tune_check): GET = state + last report, POST {action: start|cancel, image, reference,
        // mm_slot}; the report as a page or a file
        ("GET", "/api/check") => {
            let (view, report) = {
                let s = lock();
                (s.tune_check.view(), s.tune_check.report.clone())
            };
            send_json(stream, "200 OK", &serde_json::json!({ "view": view, "report": report.as_deref() }));
        }
        ("POST", "/api/check") => match parse::<CheckApi>(body) {
            Ok(r) => {
                let res = {
                    let mut s = lock();
                    match r.action.as_str() {
                        "start" => {
                            let image = r.image.unwrap_or_else(|| s.image_file.clone());
                            s.start_tune_check(&image, r.reference.filter(|x| !x.is_empty()), r.mm_slot)
                        }
                        "cancel" => {
                            s.tune_check.cancel();
                            Ok(())
                        }
                        a => Err(format!("unknown action {a}")),
                    }
                    .map(|()| s.tune_check.view())
                };
                match res {
                    Ok(v) => send_json(stream, "200 OK", &v),
                    Err(e) => error(stream, "400 Bad Request", e),
                }
            }
            Err(e) => error(stream, "400 Bad Request", e),
        },
        ("GET", "/api/check/report.html") | ("GET", "/api/check/report.json") => {
            let report = lock().tune_check.report.clone();
            match report {
                None => error(stream, "404 Not Found", "no report yet"),
                Some(r) if path.ends_with(".html") => send_bytes(stream, "text/html; charset=utf-8", "", crate::studio::tune_check::html(&r).as_bytes()),
                Some(r) => send_bytes(stream, "application/json", "Content-Disposition: attachment; filename=\"tune_check.json\"\r\n", &serde_json::to_vec_pretty(&*r).unwrap_or_default()),
            }
        }
        ("GET" | "POST", "/api/build") => {
            let res = lock().build_route(method, body);
            match res {
                Ok(v) => send_json(stream, "200 OK", &v),
                Err((status, e)) => error(stream, status, e),
            }
        }
        (_, p) if p.starts_with("/api/ws/") => {
            let r = crate::studio::workstation::route(method, p, &|k| query_param(query, k), body, &mut lock());
            match r {
                Ok(v) => send_json(stream, "200 OK", &v),
                Err((status, e)) => error(stream, status, e),
            }
        }
        _ => error(stream, "404 Not Found", "not found"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_check_rejects_rebinding_and_supports_port_80() {
        assert!(host_allowed(Some("127.0.0.1:8080"), 8080));
        assert!(host_allowed(Some("LOCALHOST:8080"), 8080));
        assert!(host_allowed(Some("[::1]:8080"), 8080));
        assert!(host_allowed(Some("localhost"), 80));
        assert!(host_allowed(Some("127.0.0.1"), 80));
        assert!(!host_allowed(Some("localhost"), 8080));
        assert!(!host_allowed(Some("evil.example:8080"), 8080));
        assert!(!host_allowed(Some("127.0.0.1:9999"), 8080));
        assert!(!host_allowed(None, 8080));
    }

    #[test]
    fn csrf_origin_content_type_and_content_length_guards() {
        assert!(origin_allowed("http://127.0.0.1:8080", 8080));
        assert!(origin_allowed("HTTP://LOCALHOST:8080", 8080));
        assert!(origin_allowed("http://localhost", 80));
        assert!(!origin_allowed("https://evil.example", 8080));
        assert!(!origin_allowed("http://127.0.0.1:9999", 8080));

        assert!(csrf_allowed("POST /api/step HTTP/1.1\r\nOrigin: http://127.0.0.1:8080\r\nSec-Fetch-Site: same-origin\r\n", 8080).is_ok());
        assert!(csrf_allowed("POST /api/step HTTP/1.1\r\nSec-Fetch-Site: cross-site\r\n", 8080).is_err());
        assert!(csrf_allowed("POST /api/step HTTP/1.1\r\nOrigin: http://evil.example\r\n", 8080).is_err());

        assert!(is_json_content_type(Some("Application/JSON; charset=utf-8")));
        assert!(!is_json_content_type(Some("text/plain")));

        assert_eq!(parse_content_length("POST / HTTP/1.1\r\nContent-Length: 42\r\n").unwrap(), 42);
        assert_eq!(
            parse_content_length("POST / HTTP/1.1\r\nContent-Length: 99999999\r\n").unwrap_err().0,
            "413 Payload Too Large"
        );
        assert_eq!(
            parse_content_length("POST / HTTP/1.1\r\nContent-Length: abc\r\n").unwrap_err().0,
            "400 Bad Request"
        );
        assert_eq!(
            parse_content_length("POST / HTTP/1.1\r\nContent-Length: 10\r\nContent-Length: 20\r\n").unwrap_err().0,
            "400 Bad Request"
        );
    }

    #[test]
    fn url_decode_handles_bad_escapes() {
        assert_eq!(url_decode("a%20b+c"), "a b c");
        assert_eq!(url_decode("%zz"), "%zz");
    }
}
