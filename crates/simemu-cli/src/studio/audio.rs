//! Studio sound: WebSocket PCM stream of the physical synthesizer (`simemu-acoustics`), wiring-harness faults
//! for diagnosis by ear, and the offline WAV renderer (`simemu render-audio`).
//!
//! Stream (`GET /api/audio/ws`, RFC 6455): server → client binary messages = little-endian f32 PCM, 48 kHz mono,
//! pressure in Pa at the selected listener; text messages ≈ 10 Hz = JSON meters/status. Client → server text
//! messages = JSON mixer `{gain:[16], cyl:[4], master, listener}`. One client at a time (a new one takes over).
//! The PCM is in plant time; the browser's AudioWorklet turns it into wall-clock audio (`sound.js`).

use crate::studio::session::{StudioSession, SLICE_S};
use simemu_acoustics::ring::Consumer;
use simemu_acoustics::{BlockOut, Mix, Params, Shared, Synth, CYL_RESOLVED, FS, LISTENER_NAMES, N_LIST, N_SRC, SRC_NAMES};
use simemu_physics::EcuOutputs;
use serde::{Deserialize, Serialize};
use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::Path;
use std::sync::atomic::Ordering::Relaxed;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Wiring-harness faults of the bench (a breakout box between the ECU and the engine loom): an unplugged
/// connector de-energises that coil / injector whatever the ECU commands. Hardware, not ECU logic.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Harness {
    #[serde(default)]
    pub coil_open: [bool; 4],
    #[serde(default)]
    pub injector_open: [bool; 4],
}

impl Harness {
    pub fn apply(&self, e: &mut EcuOutputs) {
        for c in 0..4 {
            if self.coil_open[c] {
                e.ignition[c] = None;
                e.timed.iter_mut().for_each(|t| t.coil[c] = [None; 3]);
            }
            if self.injector_open[c] {
                e.injection[c] = [None; 3];
                e.timed.iter_mut().for_each(|t| t.di[c] = [None; 3]);
            }
        }
    }
}

/// Sources, listener positions and the calibration the UI needs.
pub fn info() -> serde_json::Value {
    serde_json::json!({
        "fs": FS,
        "sources": SRC_NAMES.iter().zip(CYL_RESOLVED).map(|((k, ru, en), c)| serde_json::json!({"key": k, "ru": ru, "en": en, "cyl": c})).collect::<Vec<_>>(),
        "listeners": LISTENER_NAMES.iter().map(|(k, ru, en)| serde_json::json!({"key": k, "ru": ru, "en": en})).collect::<Vec<_>>(),
    })
}

// ---- RFC 6455 handshake helpers ------------------------------------------------------------------------

/// SHA-1 (FIPS 180-4), only for the WebSocket accept key.
fn sha1(msg: &[u8]) -> [u8; 20] {
    let mut h: [u32; 5] = [0x6745_2301, 0xEFCD_AB89, 0x98BA_DCFE, 0x1032_5476, 0xC3D2_E1F0];
    let mut m = msg.to_vec();
    m.push(0x80);
    while m.len() % 64 != 56 {
        m.push(0);
    }
    m.extend_from_slice(&((msg.len() as u64) * 8).to_be_bytes());
    for chunk in m.chunks(64) {
        let mut w = [0u32; 80];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([chunk[4 * i], chunk[4 * i + 1], chunk[4 * i + 2], chunk[4 * i + 3]]);
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let [mut a, mut b, mut c, mut d, mut e] = h;
        for (i, wi) in w.iter().enumerate() {
            let (f, k) = match i {
                0..=19 => ((b & c) | (!b & d), 0x5A82_7999),
                20..=39 => (b ^ c ^ d, 0x6ED9_EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1B_BCDC),
                _ => (b ^ c ^ d, 0xCA62_C1D6),
            };
            let t = a.rotate_left(5).wrapping_add(f).wrapping_add(e).wrapping_add(k).wrapping_add(*wi);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = t;
        }
        for (x, y) in h.iter_mut().zip([a, b, c, d, e]) {
            *x = x.wrapping_add(y);
        }
    }
    let mut out = [0u8; 20];
    for (i, x) in h.iter().enumerate() {
        out[4 * i..4 * i + 4].copy_from_slice(&x.to_be_bytes());
    }
    out
}

fn base64(b: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut s = String::new();
    for c in b.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for i in 0..4 {
            s.push(if i <= c.len() { T[(n >> (18 - 6 * i) & 63) as usize] as char } else { '=' });
        }
    }
    s
}

fn accept_key(key: &str) -> String {
    base64(&sha1(format!("{}258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key.trim()).as_bytes()))
}

fn frame(op: u8, payload: &[u8], out: &mut Vec<u8>) {
    out.clear();
    out.push(0x80 | op);
    match payload.len() {
        n if n < 126 => out.push(n as u8),
        n if n < 65536 => {
            out.push(126);
            out.extend_from_slice(&(n as u16).to_be_bytes());
        }
        n => {
            out.push(127);
            out.extend_from_slice(&(n as u64).to_be_bytes());
        }
    }
    out.extend_from_slice(payload);
}

/// Reads one client frame (must be masked per RFC 6455 §5.1). Returns (opcode, payload);
/// `None` on EOF / error / unmasked / oversize.
fn read_frame(s: &mut TcpStream) -> Option<(u8, Vec<u8>)> {
    let mut h = [0u8; 2];
    s.read_exact(&mut h).ok()?;
    if h[1] & 0x80 == 0 {
        return None;
    }
    let mut len = (h[1] & 0x7F) as u64;
    if len == 126 {
        let mut b = [0u8; 2];
        s.read_exact(&mut b).ok()?;
        len = u16::from_be_bytes(b) as u64;
    } else if len == 127 {
        let mut b = [0u8; 8];
        s.read_exact(&mut b).ok()?;
        len = u64::from_be_bytes(b);
    }
    if len > 64 * 1024 {
        return None;
    }
    let mut mask = [0u8; 4];
    s.read_exact(&mut mask).ok()?;
    let mut p = vec![0u8; len as usize];
    s.read_exact(&mut p).ok()?;
    for (i, b) in p.iter_mut().enumerate() {
        *b ^= mask[i & 3];
    }
    Some((h[0] & 0x0F, p))
}

#[derive(Deserialize)]
struct MixMsg {
    gain: Option<Vec<f32>>,
    cyl: Option<Vec<f32>>,
    master: Option<f32>,
    listener: Option<usize>,
}

fn apply_mix(sh: &Shared, m: &MixMsg) {
    let mut mix = sh.mix();
    if let Some(g) = &m.gain {
        for (d, s) in mix.gain.iter_mut().zip(g) {
            *d = s.clamp(0.0, 100.0);
        }
    }
    if let Some(c) = &m.cyl {
        for (d, s) in mix.cyl.iter_mut().zip(c) {
            *d = s.clamp(0.0, 1.0);
        }
    }
    if let Some(v) = m.master {
        mix.master = v.clamp(0.0, 100.0);
    }
    sh.set_mix(&mix);
    if let Some(l) = m.listener {
        sh.listener.store(l.min(N_LIST - 1), Relaxed);
    }
}

/// Upgrades `stream` to a WebSocket and streams the selected listener until the client leaves.
pub fn serve_ws(mut stream: TcpStream, head: &str, port: u16, sh: Arc<Shared>) {
    let hdr = |name: &str| {
        head.lines().skip(1).find_map(|l| {
            let (k, v) = l.split_once(':')?;
            k.trim().eq_ignore_ascii_case(name).then(|| v.trim().to_string())
        })
    };
    // Cross-site WebSocket hijacking: browsers always send Origin / Sec-Fetch-Site; only the Studio page itself may connect.
    if crate::studio::routes::csrf_allowed(head, port).is_err() {
        let _ = stream.write_all(b"HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        return;
    }
    let Some(key) = hdr("sec-websocket-key").filter(|_| hdr("upgrade").is_some_and(|u| u.eq_ignore_ascii_case("websocket"))) else {
        let _ = stream.write_all(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        return;
    };
    let resp = format!("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: {}\r\n\r\n", accept_key(&key));
    if stream.write_all(resp.as_bytes()).is_err() {
        return;
    }
    // Take over the PCM ring (the previous client, if any, notices the generation change and hands it back).
    let gen = sh.client_gen.fetch_add(1, Relaxed) + 1;
    let mut pcm: Option<Consumer<f32>> = None;
    for _ in 0..100 {
        pcm = sh.pcm.lock().ok().and_then(|mut g| g.take());
        if pcm.is_some() {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    let Some(mut pcm) = pcm else { return };
    let mut sink = vec![0.0f32; 1 << 14];
    while pcm.pop_slice(&mut sink) > 0 {}
    sh.active.store(true, Relaxed);

    let closed = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let pending_pong: Arc<std::sync::Mutex<Option<Vec<u8>>>> = Arc::new(std::sync::Mutex::new(None));
    if let Ok(mut rd) = stream.try_clone() {
        let (sh2, closed2, pong2) = (sh.clone(), closed.clone(), pending_pong.clone());
        let _ = rd.set_read_timeout(None);
        let _ = std::thread::Builder::new().name("audio-ws-rx".into()).spawn(move || {
            while let Some((op, p)) = read_frame(&mut rd) {
                match op {
                    0x1 => {
                        if let Ok(m) = serde_json::from_slice::<MixMsg>(&p) {
                            apply_mix(&sh2, &m);
                        }
                    }
                    0x9 if p.len() <= 125 => {
                        if let Ok(mut g) = pong2.lock() {
                            *g = Some(p);
                        }
                    }
                    0x8 => break,
                    _ => {}
                }
            }
            closed2.store(true, Relaxed);
        });
    }
    let _ = stream.set_write_timeout(Some(Duration::from_secs(2)));
    let mut bytes = Vec::with_capacity(4 * sink.len());
    let mut fr = Vec::with_capacity(4 * sink.len() + 16);
    let mut last_meta = Instant::now();
    while !closed.load(Relaxed) && sh.client_gen.load(Relaxed) == gen {
        if let Some(pong) = pending_pong.lock().ok().and_then(|mut g| g.take()) {
            frame(0xA, &pong, &mut fr);
            if stream.write_all(&fr).is_err() {
                break;
            }
        }
        let n = pcm.pop_slice(&mut sink[..4800]);
        if n > 0 {
            bytes.clear();
            for v in &sink[..n] {
                bytes.extend_from_slice(&v.to_le_bytes());
            }
            frame(0x2, &bytes, &mut fr);
            if stream.write_all(&fr).is_err() {
                break;
            }
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
        if last_meta.elapsed() >= Duration::from_millis(100) {
            last_meta = Instant::now();
            let l = sh.listener.load(Relaxed);
            let lv: Vec<f32> = sh.levels(l).iter().map(|&ms| if ms > 0.0 { (10.0 * (ms / 4e-10).log10() * 10.0).round() / 10.0 } else { -99.0 }).collect();
            let meta = serde_json::json!({
                "lv": lv, "listener": l, "sim_t": f64::from_bits(sh.sim_t.load(Relaxed)),
                "load": sh.load(), "dropped": sh.dropped.load(Relaxed), "queued": pcm.len(),
            });
            frame(0x1, meta.to_string().as_bytes(), &mut fr);
            if stream.write_all(&fr).is_err() {
                break;
            }
        }
    }
    if sh.client_gen.load(Relaxed) == gen {
        sh.active.store(false, Relaxed);
    }
    if let Ok(mut g) = sh.pcm.lock() {
        *g = Some(pcm);
    }
    let mut close = Vec::new();
    frame(0x8, &[], &mut close);
    let _ = stream.write_all(&close);
    let _ = stream.shutdown(std::net::Shutdown::Both);
}

// ---- offline renderer ----------------------------------------------------------------------------------

/// A-weighting (IEC 61672-1) of frequency `f`, linear amplitude factor.
fn a_weight(f: f64) -> f64 {
    let f2 = f * f;
    let ra = 12194.0f64.powi(2) * f2 * f2 / ((f2 + 20.6f64.powi(2)) * ((f2 + 107.7f64.powi(2)) * (f2 + 737.9f64.powi(2))).sqrt() * (f2 + 12194.0f64.powi(2)));
    ra * 10f64.powf(2.0 / 20.0)
}

/// (L_eq unweighted, L_Aeq) in dB re 20 µPa of a pressure record.
pub fn levels_db(x: &[f32]) -> (f64, f64) {
    let n = 8192;
    let fft = simemu_acoustics::fft::Fft::new(n);
    let (mut re, mut im) = (vec![0.0f32; n], vec![0.0f32; n]);
    let (mut e, mut ea, mut k) = (0.0, 0.0, 0);
    while (k + 1) * n <= x.len() {
        re.copy_from_slice(&x[k * n..(k + 1) * n]);
        im.fill(0.0);
        fft.forward(&mut re, &mut im);
        for b in 1..n / 2 {
            let p = 2.0 * (re[b] as f64 * re[b] as f64 + im[b] as f64 * im[b] as f64) / (n as f64 * n as f64);
            e += p;
            ea += p * a_weight(b as f64 * FS / n as f64).powi(2);
        }
        k += 1;
    }
    let k = k.max(1) as f64;
    (10.0 * (e / k / 4e-10).log10(), 10.0 * (ea / k / 4e-10).log10())
}

struct Scenario {
    name: &'static str,
    secs: f64,
    /// (file suffix, from s, to s)
    cuts: &'static [(&'static str, f64, f64)],
}

const SCENARIOS: [Scenario; 4] = [
    Scenario { name: "start", secs: 12.0, cuts: &[("start", 0.0, 6.0), ("idle", 6.0, 12.0)] },
    Scenario { name: "rev", secs: 12.0, cuts: &[("rev", 5.0, 12.0)] },
    Scenario { name: "misfire", secs: 12.0, cuts: &[("misfire", 5.0, 12.0)] },
    Scenario { name: "knock", secs: 11.0, cuts: &[("knock", 5.0, 11.0)] },
];

/// Driver / bench actions of a scenario at plant time `t` (the firmware does all engine control).
fn stimulus(name: &str, s: &mut StudioSession, t: f64) {
    let d = &mut s.plant.driver;
    d.brake_pct = if (0.5..6.0).contains(&t) { 60.0 } else { 0.0 };
    d.start_button = (1.0..3.0).contains(&t);
    d.pedal_pct = 0.0;
    match name {
        "rev" => {
            d.pedal_pct = if (6.0..6.5).contains(&t) {
                35.0
            } else if (8.0..8.4).contains(&t) {
                60.0
            } else if (10.0..10.3).contains(&t) {
                85.0
            } else {
                0.0
            };
        }
        "misfire" => s.harness.coil_open[1] = t >= 6.0,
        "knock" if t >= 6.0 => {
            d.pedal_pct = 100.0;
            if !matches!(s.plant.bench, simemu_physics::BenchMode::EngineDyno { .. }) {
                s.plant.bench = simemu_physics::BenchMode::EngineDyno { target_rpm: 2500.0 };
            }
        }
        _ => {}
    }
}

fn new_session(root: &Path, name: &str) -> Result<StudioSession, String> {
    let mut s = StudioSession::new(root)?;
    s.plant.soak(90.0);
    s.plant.driver.ignition_on = true;
    s.plant.driver.selector = simemu_physics::Selector::P;
    if name == "knock" {
        // Fuel in the tank: the physical knock cause (no knock-sensor input is emulated, so the ECU cannot retard).
        s.plant.fuel_ron = 80.0;
    }
    Ok(s)
}

/// `simemu render-audio [start|rev|misfire|knock|all|rtf] [out_dir]`.
pub fn cmd_render_audio(root: &Path, which: &str, out: &Path) -> Result<(), String> {
    if which == "rtf" {
        return cmd_rtf(root);
    }
    if which != "all" && !SCENARIOS.iter().any(|s| s.name == which) {
        return Err(format!(
            "unknown render-audio scenario {which:?} (expected start|rev|misfire|knock|all|rtf)"
        ));
    }
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    for sc in SCENARIOS.iter().filter(|s| which == "all" || which == s.name) {
        let mut s = new_session(root, sc.name)?;
        s.audio.shared.active.store(true, Relaxed);
        let mut rx = s.audio.take_records().ok_or("audio records already taken")?;
        let mut synth = Synth::new(Params::default());
        synth.mix = Mix::default();
        let mut pcm: Vec<Vec<f32>> = vec![Vec::new(); N_LIST];
        let mut lv: Vec<(f64, [[f32; N_SRC]; N_LIST])> = Vec::new();
        let (mut wall_sim, mut wall_syn) = (Duration::ZERO, Duration::ZERO);
        let mut events: Vec<String> = Vec::new();
        for i in 0..(sc.secs / SLICE_S).round() as usize {
            let t = i as f64 * SLICE_S;
            stimulus(sc.name, &mut s, t);
            let t0 = Instant::now();
            s.step(SLICE_S);
            wall_sim += t0.elapsed();
            let t1 = Instant::now();
            while let Some(r) = rx.pop() {
                synth.push(&r, &mut |b: &BlockOut| {
                    for (dst, src) in pcm.iter_mut().zip(&b.out) {
                        dst.extend_from_slice(src);
                    }
                    lv.push((pcm[0].len() as f64 / FS, b.level));
                });
            }
            wall_syn += t1.elapsed();
            if i % 100 == 99 {
                let tel = s.plant.telemetry();
                events.push(format!("t={:.1}s {} rpm={:.0} map={:.0}kPa knock_max={:.2}bar", t + SLICE_S, tel.engine_state, tel.rpm, tel.map_kpa, tel.knock_amp_bar.iter().cloned().fold(0.0, f64::max)));
            }
        }
        let audio_s = pcm[0].len() as f64 / FS;
        println!("== {} ({:.1} s plant time): system RTF {:.2} (sim / wall of ECU+TCU+plant+probe), synthesizer load {:.1} % of one core, dropped {}",
            sc.name, sc.secs, sc.secs / wall_sim.as_secs_f64(), 100.0 * wall_syn.as_secs_f64() / audio_s.max(1e-9), s.audio.shared.dropped.load(Relaxed));
        for e in &events {
            println!("   {e}");
        }
        for (suffix, a, b) in sc.cuts {
            for (l, (key, _, _)) in LISTENER_NAMES.iter().enumerate() {
                let x = &pcm[l];
                let (ia, ib) = (((a * FS) as usize).min(x.len()), ((b * FS) as usize).min(x.len()));
                let seg: Vec<f32> = x[ia..ib].iter().map(|v| v * WAV_SCALE).collect();
                let path = out.join(format!("{suffix}_{key}.wav"));
                simemu_acoustics::wav::write_f32(&path, FS as u32, 1, &seg).map_err(|e| format!("{}: {e}", path.display()))?;
                let (lz, la) = levels_db(&x[ia..ib]);
                println!("   {:<28} L = {lz:5.1} dB, L_A = {la:5.1} dB(A)", path.file_name().unwrap_or_default().to_string_lossy());
            }
            for (l, (key, _, _)) in LISTENER_NAMES.iter().enumerate().take(3) {
                let sel: Vec<_> = lv.iter().filter(|(t, _)| (*a..*b).contains(t)).collect();
                let n = sel.len().max(1) as f64;
                let row: Vec<String> = (0..N_SRC)
                    .map(|k| (k, sel.iter().map(|(_, x)| x[l][k] as f64).sum::<f64>() / n))
                    .filter(|&(_, e)| e > 4e-10)
                    .map(|(k, e)| format!("{}={:.0}", SRC_NAMES[k].0, 10.0 * (e / 4e-10).log10()))
                    .collect();
                println!("     {suffix} sources at {key} (dB, unweighted): {}", row.join(" "));
            }
        }
    }
    Ok(())
}

/// WAV sample value per pascal: 1.0 full scale = 10 Pa (114 dB SPL); the files are 32-bit float, never clip.
pub const WAV_SCALE: f32 = 0.1;

/// Real-time factor of the whole system without and with the sound engine (threaded, as in the studio).
fn cmd_rtf(root: &Path) -> Result<(), String> {
    let secs = 10.0;
    for with_audio in [false, true] {
        let mut s = new_session(root, "start")?;
        if with_audio {
            s.audio.shared.active.store(true, Relaxed);
        }
        let t0 = Instant::now();
        for i in 0..(secs / SLICE_S) as usize {
            stimulus("start", &mut s, i as f64 * SLICE_S);
            s.step(SLICE_S);
        }
        let wall = t0.elapsed().as_secs_f64();
        std::thread::sleep(Duration::from_millis(300));
        let sh = &s.audio.shared;
        println!(
            "{}: system RTF {:.2} over {secs} s plant time{}",
            if with_audio { "with sound (synthesizer thread)" } else { "without sound" },
            secs / wall,
            if with_audio { format!("; synthesizer load {:.1} % of one core, dropped records {}", 100.0 * sh.load(), sh.dropped.load(Relaxed)) } else { String::new() }
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn websocket_accept_key_matches_rfc6455_example() {
        assert_eq!(accept_key("dGhlIHNhbXBsZSBub25jZQ=="), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
        assert_eq!(base64(b"ab"), "YWI=");
    }

    #[test]
    fn harness_fault_cuts_only_the_unplugged_coil() {
        let mut e = EcuOutputs {
            ignition: [Some(Default::default()); 4],
            ..Default::default()
        };
        Harness { coil_open: [false, true, false, false], ..Default::default() }.apply(&mut e);
        assert!(e.ignition[0].is_some() && e.ignition[1].is_none() && e.ignition[2].is_some());
    }

    #[test]
    fn read_frame_rejects_unmasked_client_frames() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut client = TcpStream::connect(addr).unwrap();
        let (mut server, _) = listener.accept().unwrap();
        // Unmasked text frame (MASK bit 0x80 clear) -> rejected per RFC 6455 §5.1
        client.write_all(&[0x81, 0x02, b'h', b'i']).unwrap();
        assert!(read_frame(&mut server).is_none());

        // Masked text frame -> accepted and unmasked
        let mut client2 = TcpStream::connect(addr).unwrap();
        let (mut server2, _) = listener.accept().unwrap();
        client2.write_all(&[0x81, 0x82, 0xAA, 0xBB, 0xCC, 0xDD, b'h' ^ 0xAA, b'i' ^ 0xBB]).unwrap();
        assert_eq!(read_frame(&mut server2), Some((0x1, b"hi".to_vec())));
    }

    #[test]
    fn a_weighting_is_zero_db_at_1_khz() {
        assert!((20.0 * a_weight(1000.0).log10()).abs() < 0.05);
        assert!((20.0 * a_weight(100.0).log10() + 19.1).abs() < 0.2);
    }
}
