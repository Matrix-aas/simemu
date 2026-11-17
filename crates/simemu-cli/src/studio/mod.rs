pub mod assets;
pub mod audio;
mod backend_cosim;
pub mod bench_build;
pub mod dyno_correction;
pub mod drive_run;
pub mod dq250_prep;
pub mod dyno_run;
pub mod engine_run;
pub use crate::{immo_keys, immo_lab};
pub mod io;
pub mod io_air;
pub mod log_replay;
pub mod recorder;
pub mod restbus_run;
pub mod routes;
pub mod perf_bench;
pub mod safety;
pub mod rt;
pub mod session;
pub mod tcu_io;
pub mod tune_check;
pub mod uds;
pub mod workstation;

use crate::slcan::{SlcanServer, DEFAULT_SLCAN_PORT};
use crate::studio::routes::handle_client;
use crate::studio::session::StudioSession;
use std::io::Write;
use std::net::TcpListener;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

const MAX_HTTP_CONCURRENCY: usize = 32;

struct ActiveGuard(Arc<AtomicUsize>);
impl Drop for ActiveGuard {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::AcqRel);
    }
}

/// SLCAN TCP listener configuration for [`run_studio_server_opts`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StudioSlcanConfig {
    /// Default mode: attempt `127.0.0.1:28750`, continuing without error if already bound.
    DefaultIfFree(u16),
    /// Explicitly requested via `--slcan <port>` or `SIMEMU_SLCAN_PORT=<port>`: fail startup if busy.
    Explicit(u16),
    /// Disabled (`--no-slcan`, `--slcan off`, or `SIMEMU_SLCAN_PORT=off`).
    Disabled,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StudioCliOpts {
    pub http_port: u16,
    pub slcan: StudioSlcanConfig,
}

fn parse_slcan_spec(spec: &str) -> Result<StudioSlcanConfig, String> {
    let s = spec.trim();
    if s.eq_ignore_ascii_case("off") || s.eq_ignore_ascii_case("none") || s == "0" {
        return Ok(StudioSlcanConfig::Disabled);
    }
    let port = s
        .parse::<u16>()
        .map_err(|_| format!("invalid SLCAN port {spec:?} (expected 1..65535 or 'off')"))?;
    Ok(StudioSlcanConfig::Explicit(port))
}

/// Parses `simemu studio [port] [--slcan <port|off> | --no-slcan]` combined with `SIMEMU_SLCAN_PORT`.
pub fn parse_studio_args(args: &[String], env_slcan: Option<&str>) -> Result<StudioCliOpts, String> {
    let mut http_port: Option<u16> = None;
    let mut cli_slcan: Option<StudioSlcanConfig> = None;
    let mut i = 0;
    while i < args.len() {
        let arg = args[i].as_str();
        if arg == "--no-slcan" {
            cli_slcan = Some(StudioSlcanConfig::Disabled);
            i += 1;
        } else if arg == "--slcan" {
            let val = args.get(i + 1).ok_or("--slcan requires a <port> or 'off'")?;
            cli_slcan = Some(parse_slcan_spec(val)?);
            i += 2;
        } else if let Some(val) = arg.strip_prefix("--slcan=") {
            cli_slcan = Some(parse_slcan_spec(val)?);
            i += 1;
        } else if http_port.is_none() {
            http_port = Some(
                arg.parse::<u16>()
                    .map_err(|_| format!("invalid studio HTTP port {arg:?}"))?,
            );
            i += 1;
        } else {
            return Err(format!("unexpected studio argument {arg:?}"));
        }
    }
    let slcan = match cli_slcan {
        Some(cfg) => cfg,
        None => match env_slcan.filter(|s| !s.trim().is_empty()) {
            Some(v) => parse_slcan_spec(v)?,
            None => StudioSlcanConfig::DefaultIfFree(DEFAULT_SLCAN_PORT),
        },
    };
    Ok(StudioCliOpts {
        http_port: http_port.unwrap_or(8080),
        slcan,
    })
}

pub fn bind_studio_slcan(cfg: StudioSlcanConfig) -> Result<Option<SlcanServer>, String> {
    match cfg {
        StudioSlcanConfig::Disabled => Ok(None),
        StudioSlcanConfig::Explicit(port) => SlcanServer::bind(port, true).map(Some),
        StudioSlcanConfig::DefaultIfFree(port) => match SlcanServer::bind(port, true) {
            Ok(srv) => Ok(Some(srv)),
            Err(e) => {
                if simemu_damos::is_ru() {
                    eprintln!("  SLCAN: порт по умолчанию 127.0.0.1:{port} занят ({e}); работаем без SLCAN");
                } else {
                    eprintln!("  SLCAN: default port 127.0.0.1:{port} unavailable ({e}); continuing without SLCAN");
                }
                Ok(None)
            }
        },
    }
}

pub fn run_studio_server(root: &Path, port: u16) -> Result<(), String> {
    let env_slcan = std::env::var("SIMEMU_SLCAN_PORT").ok();
    let opts = parse_studio_args(&[port.to_string()], env_slcan.as_deref())?;
    run_studio_server_opts(root, opts)
}

pub fn run_studio_server_with_args(root: &Path, args: &[String]) -> Result<(), String> {
    let env_slcan = std::env::var("SIMEMU_SLCAN_PORT").ok();
    let opts = parse_studio_args(args, env_slcan.as_deref())?;
    run_studio_server_opts(root, opts)
}

pub fn run_studio_server_opts(root: &Path, opts: StudioCliOpts) -> Result<(), String> {
    let slcan_server = bind_studio_slcan(opts.slcan)?;
    let bind_addr = format!("127.0.0.1:{}", opts.http_port);
    let listener = TcpListener::bind(&bind_addr).map_err(|e| format!("bind {bind_addr}: {e}"))?;

    if simemu_damos::is_ru() {
        println!("SIMEMU Studio — загрузка прошивок SIMOS18.1 (TC1791) и DQ200_G2 (TC1784)...");
    } else {
        println!("SIMEMU Studio — booting SIMOS18.1 (TC1791) and DQ200_G2 (TC1784) firmware...");
    }
    let build = crate::studio::bench_build::build_from_env()?
        .unwrap_or_else(|| simemu_physics::bench_config::preset("octavia_fl").expect("octavia_fl"));
    let mut session = StudioSession::with_build(root, crate::studio::bench_build::BuildState::from_config(build))?;
    // a DQ250 with an erased EEPROM is prepared by its own firmware in a headless workshop run (studio::dq250_prep);
    // SIMEMU_DQ250_AUTOPREP=0 leaves it erased
    session.tcu_prep.auto = std::env::var("SIMEMU_DQ250_AUTOPREP").map_or(true, |v| v != "0");
    // start pre-warmed (90 °C coolant/oil, 70 °C gearbox, 300 s EOT) on the stock FRF software for the bench engine
    session.warm_car();
    // the ECU DFlash and TCU EEPROM (coding, adaptations) persist between studio runs, untracked; units without saved
    // state also power-cycle once so they boot on the pre-warmed plant rather than keeping their cold-boot init
    session.attach_nvm_dir(root.join("build/studio_state"), true)?;
    let slcan_port = slcan_server.as_ref().map(SlcanServer::port);
    session.set_slcan(slcan_server);

    let st = session.state();
    if simemu_damos::is_ru() {
        println!(
            "  ЭБУ: PC={} ({}) сбросов={} задач={} | ТКУ: {} {}",
            st.ecu.pc,
            st.ecu.region,
            st.ecu.warm_resets,
            st.ecu.rtos_tasks_seen,
            if st.tcu.present { st.tcu.pc.as_str() } else { "отсутствует" },
            st.tcu.error.as_deref().unwrap_or("")
        );
    } else {
        println!(
            "  ECU: PC={} ({}) resets={} tasks={} | TCU: {} {}",
            st.ecu.pc,
            st.ecu.region,
            st.ecu.warm_resets,
            st.ecu.rtos_tasks_seen,
            if st.tcu.present { st.tcu.pc.as_str() } else { "absent" },
            st.tcu.error.as_deref().unwrap_or("")
        );
    }
    drop(st);

    println!("  Studio: http://{bind_addr}");
    if let Some(p) = slcan_port {
        println!("  SLCAN TCP 127.0.0.1:{p}");
    }

    let port = opts.http_port;
    let shared = Arc::new(Mutex::new(session));
    {
        let s = Arc::clone(&shared);
        std::thread::Builder::new().name("sim-runner".into()).spawn(move || run_realtime(s)).map_err(|e| e.to_string())?;
    }
    let active_http = Arc::new(AtomicUsize::new(0));
    for mut stream in listener.incoming().flatten() {
        let cur = active_http.fetch_add(1, Ordering::AcqRel);
        if cur >= MAX_HTTP_CONCURRENCY {
            active_http.fetch_sub(1, Ordering::AcqRel);
            let _ = stream.write_all(
                b"HTTP/1.1 503 Service Unavailable\r\nRetry-After: 1\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
            );
            continue;
        }
        let guard = ActiveGuard(Arc::clone(&active_http));
        let s = Arc::clone(&shared);
        if let Err(e) = std::thread::Builder::new()
            .name("studio-http".into())
            .spawn(move || {
                let _guard = guard;
                rt::set_thread_role(rt::ThreadRole::Background);
                handle_client(stream, s, port);
            })
        {
            eprintln!("  studio: failed to spawn HTTP handler thread: {e}");
        }
    }
    Ok(())
}

/// Real-time runner: steps the session in 10 ms slices paced to the wall clock at the user-selected
/// speed ([`rt::Pacer`]: never skips simulation, reports RTF/lag/overruns in `frame.pacing`). The
/// session lock is released between slices so the UI's requests interleave.
fn run_realtime(session: Arc<Mutex<StudioSession>>) {
    use crate::studio::session::{PacingStats, SLICE_S};
    rt::set_thread_role(rt::ThreadRole::Realtime);
    let lock = || session.lock().unwrap_or_else(|e| e.into_inner());
    let mut pacer = rt::Pacer::new(1.0);
    lock().realtime = true;
    loop {
        let sleep = {
            let mut s = lock();
            if s.paused {
                s.poll_slcan_idle();
                None
            } else {
                if s.speed != pacer.speed {
                    pacer.speed = s.speed;
                    pacer.rebase();
                }
                s.step(SLICE_S);
                let d = pacer.after_step(SLICE_S, std::time::Instant::now());
                let p = &pacer;
                s.frame.pacing = Some(PacingStats { speed: p.speed, rtf: p.rtf, lag_s: p.lag_s, slip_s: p.slip_s, overruns: p.overruns });
                Some(d)
            }
        };
        match sleep {
            None => {
                pacer.rebase();
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            Some(d) if d.is_zero() => std::thread::yield_now(),
            Some(d) => std::thread::sleep(d),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn studio_cli_arg_and_env_slcan_parsing() {
        let v = |s: &[&str]| s.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        assert_eq!(
            parse_studio_args(&v(&[]), None).unwrap(),
            StudioCliOpts {
                http_port: 8080,
                slcan: StudioSlcanConfig::DefaultIfFree(DEFAULT_SLCAN_PORT),
            }
        );
        assert_eq!(
            parse_studio_args(&v(&["9090", "--slcan", "29000"]), None).unwrap(),
            StudioCliOpts {
                http_port: 9090,
                slcan: StudioSlcanConfig::Explicit(29000),
            }
        );
        assert_eq!(
            parse_studio_args(&v(&["--slcan=29001", "8088"]), Some("28755")).unwrap(),
            StudioCliOpts {
                http_port: 8088,
                slcan: StudioSlcanConfig::Explicit(29001),
            }
        );
        assert_eq!(
            parse_studio_args(&v(&[]), Some("28755")).unwrap(),
            StudioCliOpts {
                http_port: 8080,
                slcan: StudioSlcanConfig::Explicit(28755),
            }
        );
        assert_eq!(
            parse_studio_args(&v(&[]), Some("off")).unwrap(),
            StudioCliOpts {
                http_port: 8080,
                slcan: StudioSlcanConfig::Disabled,
            }
        );
        assert_eq!(
            parse_studio_args(&v(&["--no-slcan"]), Some("28755")).unwrap(),
            StudioCliOpts {
                http_port: 8080,
                slcan: StudioSlcanConfig::Disabled,
            }
        );
    }

    #[test]
    fn studio_slcan_gracefully_skips_busy_default_port_but_errors_when_explicit() {
        let blocker = TcpListener::bind("127.0.0.1:0").unwrap();
        let busy_port = blocker.local_addr().unwrap().port();

        // DefaultIfFree does NOT fail startup when the port is already occupied
        assert!(bind_studio_slcan(StudioSlcanConfig::DefaultIfFree(busy_port))
            .unwrap()
            .is_none());

        // Explicit MUST fail if the requested port is occupied
        assert!(bind_studio_slcan(StudioSlcanConfig::Explicit(busy_port)).is_err());
    }
}
