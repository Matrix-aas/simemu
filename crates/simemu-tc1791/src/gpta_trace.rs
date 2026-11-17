//! Recorder of everything a board does to its GPTA model (`SIMEMU_GPTA_TRACE=<dir>`, one file per GPTA instance):
//! the stimulus that `gpta::diff_tests::diff_replay_recorded_trace` replays through the model and the reference
//! model in lockstep. Lines (numbers hex): `D <dq250>` first, then `W <addr> <value> <mask>`, `R <addr>`,
//! `A <ticks> [<t> <kernel> <line> <level>]*` (advance with input edges), `L [<kernel> <g|l> <cell> <line>]*`
//! (pin loops).
use crate::gpta::{CellKind, InputEdge};
use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};

static FILES: AtomicU32 = AtomicU32::new(0);

pub(crate) struct GptaTrace {
    out: std::io::BufWriter<std::fs::File>,
    lines: u64,
}

impl GptaTrace {
    /// A new trace file when `SIMEMU_GPTA_TRACE` names a directory.
    pub(crate) fn open(dq250: bool) -> Option<Box<GptaTrace>> {
        let dir = std::env::var_os("SIMEMU_GPTA_TRACE")?;
        let n = FILES.fetch_add(1, Ordering::Relaxed);
        let path = std::path::Path::new(&dir).join(format!("gpta_{}_{n}.trace", std::process::id()));
        let mut out = std::io::BufWriter::new(std::fs::File::create(&path).ok()?);
        writeln!(out, "D {}", dq250 as u8).ok()?;
        Some(Box::new(GptaTrace { out, lines: 0 }))
    }

    fn line(&mut self, args: std::fmt::Arguments) {
        let _ = self.out.write_fmt(args);
        let _ = self.out.write_all(b"\n");
        self.lines += 1;
        if self.lines.is_multiple_of(4096) {
            let _ = self.out.flush();
        }
    }

    pub(crate) fn write(&mut self, a: u32, v: u32, m: u32) {
        self.line(format_args!("W {a:x} {v:x} {m:x}"));
    }

    pub(crate) fn read(&mut self, a: u32) {
        self.line(format_args!("R {a:x}"));
    }

    pub(crate) fn advance(&mut self, ticks: u64, edges: &[InputEdge]) {
        let mut s = format!("A {ticks:x}");
        for e in edges {
            s += &format!(" {:x} {} {} {}", e.t, e.kernel, e.line, e.level as u8);
        }
        self.line(format_args!("{s}"));
    }

    pub(crate) fn loops(&mut self, l: &[(u8, CellKind, u8, u8)]) {
        let mut s = "L".to_string();
        for &(k, kind, c, line) in l {
            s += &format!(" {k} {} {c} {line}", if kind == CellKind::Gtc { 'g' } else { 'l' });
        }
        self.line(format_args!("{s}"));
    }
}
