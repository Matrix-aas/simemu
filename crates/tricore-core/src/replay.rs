//! Record/replay differential harness: the native core replays an execution recorded on the
//! reference board (`simemu-tc1791`, Unicorn backend) and is compared against it.
//!
//! # Design
//!
//! The recorder (the board side, `tests/board_replay.rs`) steps the board one translation block at
//! a time (`run_instructions(1)`: one quantum boundary plus exactly one block) and emits a stream of
//! [`Event`]s in instruction-count order. The [`Replayer`] consumes them while the native core runs
//! with a [`ReplayBus`]:
//!
//! * [`Event::Read`] / [`Event::Write`] — a CPU peripheral access (or flash command store) the
//!   board logged (`Tc1791HardwareState::mmio_log`). The replay bus answers native reads with the
//!   recorded values in program order, checks native writes (address, size, value) against the
//!   recorded ones, joins accesses Unicorn split into narrower ones, and follows the ENDINIT state
//!   recorded after each access (an MTCR later in the same block depends on it).
//! * [`Event::Irq`] — interrupt entry with PIPN at an exact instruction count; the native core runs
//!   there and [`Cpu::take_interrupt`] must accept it.
//! * [`Event::AsyncTrap`] — asynchronous trap at an exact count.
//! * [`Event::Check`] — architectural state (and optionally RAM ranges) after exactly `icount`
//!   instructions.
//! * [`Event::Sync`] — copy the board state into the native core without comparing (start, board
//!   traps/resets, and resynchronisation after a divergence that a documented board/Unicorn defect
//!   explains).
//!
//! Events borrow the board's buffers (`'a`), so a long run streams in lockstep without storing a
//! trace. Board-side effects the native core cannot see (DMA writes into RAM, DFlash programming,
//! the ICU's ICR.PIPN output) and the documented Unicorn/board defects are handled by the recorder;
//! see the defect list and `classify` there.
//!
//! [`TraceOp`] streams ([`play`], [`trace_to_bytes`]) are the same execution reduced to what the
//! native core needs without the board: the regression test replays a recorded SIMOS18.1 segment,
//! and the benchmark replays a recorded 50 M-instruction run.

use crate::arch::{Arch, Trap};
use crate::bus::{Bus, BusError};
use crate::cpu::{Cpu, Exit};
use std::collections::VecDeque;

/// Architectural registers compared by the harness.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ArchState {
    pub d: [u32; 16],
    pub a: [u32; 16],
    pub pc: u32,
    pub psw: u32,
    pub pcxi: u32,
    pub fcx: u32,
    pub lcx: u32,
    pub icr: u32,
    pub isp: u32,
    pub btv: u32,
    pub biv: u32,
}

impl ArchState {
    pub fn of<B: Bus>(c: &Cpu<B>) -> Self {
        Self {
            d: c.d,
            a: c.a,
            pc: c.pc,
            psw: c.psw,
            pcxi: c.pcxi,
            fcx: c.fcx,
            lcx: c.lcx,
            icr: c.icr,
            isp: c.isp,
            btv: c.btv,
            biv: c.biv,
        }
    }

    pub fn apply<B: Bus>(&self, c: &mut Cpu<B>) {
        (c.d, c.a, c.pc, c.psw, c.pcxi, c.fcx, c.lcx, c.icr, c.isp, c.btv, c.biv) =
            (self.d, self.a, self.pc, self.psw, self.pcxi, self.fcx, self.lcx, self.icr, self.isp, self.btv, self.biv);
    }

    /// Names and values of the registers that differ (`self` = expected).
    pub fn diff(&self, other: &Self) -> Vec<(String, u32, u32)> {
        let mut v = Vec::new();
        for i in 0..16 {
            if self.d[i] != other.d[i] {
                v.push((format!("D{i}"), self.d[i], other.d[i]));
            }
            if self.a[i] != other.a[i] {
                v.push((format!("A{i}"), self.a[i], other.a[i]));
            }
        }
        for (n, x, y) in [
            ("PC", self.pc, other.pc),
            ("PSW", self.psw, other.psw),
            ("PCXI", self.pcxi, other.pcxi),
            ("FCX", self.fcx, other.fcx),
            ("LCX", self.lcx, other.lcx),
            ("ICR", self.icr, other.icr),
            ("ISP", self.isp, other.isp),
            ("BTV", self.btv, other.btv),
            ("BIV", self.biv, other.biv),
        ] {
            if x != y {
                v.push((n.to_string(), x, y));
            }
        }
        v
    }
}

/// One element of a recorded execution. RAM ranges are `(guest address, bytes)`.
#[derive(Clone, Debug)]
pub enum Event<'a> {
    /// A CPU peripheral read and the value the board returned. `endinit`: the system ENDINIT
    /// state after the access (an MTCR later in the same block depends on it).
    Read { icount: u64, addr: u32, size: u8, value: u64, endinit: bool },
    /// A CPU peripheral write (or flash command store) the board saw; the native core must
    /// perform the same write.
    Write { icount: u64, addr: u32, size: u8, value: u64, endinit: bool },
    Irq { icount: u64, pipn: u8 },
    AsyncTrap { icount: u64, trap: Trap },
    Check { icount: u64, state: ArchState, ram: &'a [(u32, &'a [u8])] },
    Sync { icount: u64, state: ArchState, ram: &'a [(u32, &'a [u8])] },
}

/// Peripheral bus fed with recorded accesses. Unicorn may split one access into narrower ones
/// (e.g. LD.D/ST.D on an MMIO segment become two 4-byte accesses); a native access is matched
/// against the recorded accesses that exactly tile its byte range, in order.
#[derive(Default)]
pub struct ReplayBus {
    /// Recorded reads: (addr, size, value, ENDINIT after the access).
    pub reads: VecDeque<(u32, u8, u64, bool)>,
    /// Peripheral writes the board performed, still to be matched by the native core.
    pub writes: VecDeque<(u32, u8, u64, bool)>,
    /// First access that did not match the recording.
    pub mismatch: Option<String>,
    /// Native reads/writes with no recorded access left.
    pub underflow: u64,
    pub endinit: bool,
}

/// Pop recorded accesses tiling `[addr, addr+size)`; returns the combined value and the ENDINIT
/// state after the last one.
fn take(q: &mut VecDeque<(u32, u8, u64, bool)>, addr: u32, size: u8) -> Result<(u64, bool), String> {
    if size > 8 {
        return Err(format!("invalid native access size {size} at {addr:#010x}"));
    }
    let (mut v, mut endinit) = (0u64, false);
    let mut done = 0u8;
    while done < size {
        match q.front() {
            Some(&(a, s, x, e))
                if s != 0
                    && a == addr.wrapping_add(done as u32)
                    && (done as u16 + s as u16) <= size as u16 =>
            {
                q.pop_front();
                let m = if s >= 8 { u64::MAX } else { (1u64 << (8 * s)) - 1 };
                v |= (x & m) << (8 * done);
                done += s;
                endinit = e;
            }
            Some(&(a, s, _, _)) => return Err(format!("board {s} bytes at {a:#010x}, native {size} bytes at {addr:#010x}")),
            None => return Err(format!("no recorded access left for native {size} bytes at {addr:#010x}")),
        }
    }
    Ok((v, endinit))
}

impl Bus for ReplayBus {
    fn read(&mut self, addr: u32, size: u8) -> Result<u64, BusError> {
        match take(&mut self.reads, addr, size) {
            Ok((v, e)) => {
                self.endinit = e;
                Ok(v)
            }
            Err(e) => {
                self.underflow += 1;
                self.mismatch.get_or_insert(format!("peripheral read: {e}"));
                Ok(0)
            }
        }
    }

    fn write(&mut self, addr: u32, size: u8, value: u64) -> Result<(), BusError> {
        let m = if size >= 8 { u64::MAX } else { (1u64 << (8 * size)) - 1 };
        match take(&mut self.writes, addr, size) {
            Ok((v, e)) if v == value & m => self.endinit = e,
            Ok((v, _)) => {
                self.mismatch.get_or_insert(format!("peripheral write {addr:#010x}/{size}: board {v:#x}, native {:#x}", value & m));
            }
            Err(e) => {
                self.underflow += 1;
                self.mismatch.get_or_insert(format!("peripheral write: {e}"));
            }
        }
        Ok(())
    }

    fn endinit(&self) -> bool {
        self.endinit
    }
}

/// A difference between the board and the native core.
#[derive(Clone, Debug)]
pub struct Divergence {
    pub icount: u64,
    pub what: String,
    /// Native state at the divergence.
    pub native: Box<ArchState>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ReplayStats {
    pub checks: u64,
    pub syncs: u64,
    pub irqs: u64,
    pub reads: u64,
    pub writes: u64,
    /// Instructions executed in intervals that ended in a passing `Check` after the previous
    /// `Check` (i.e. exactly verified).
    pub verified_insns: u64,
}

/// Consumes [`Event`]s and drives the native core.
pub struct Replayer {
    pub stats: ReplayStats,
    last_verified_from: Option<u64>,
}

impl Default for Replayer {
    fn default() -> Self {
        Self::new()
    }
}

impl Replayer {
    pub fn new() -> Self {
        Self { stats: ReplayStats::default(), last_verified_from: None }
    }

    /// Run the native core to exactly `icount` instructions (also the first half of a `Check`;
    /// lets the caller adjust board-owned state such as ICR.PIPN before comparing).
    pub fn run_to(cpu: &mut Cpu<ReplayBus>, icount: u64) -> Result<(), Divergence> {
        while cpu.icount < icount {
            match cpu.run(icount - cpu.icount) {
                Exit::Budget | Exit::Stopped(_) | Exit::Requested | Exit::Hook { .. } => {}
                Exit::FetchFault { pc } => {
                    return Err(Divergence {
                        icount: cpu.icount,
                        what: format!("native fetch fault at {pc:#010x}"),
                        native: Box::new(ArchState::of(cpu)),
                    })
                }
            }
        }
        if cpu.icount > icount {
            return Err(Divergence {
                icount: cpu.icount,
                what: format!("native core overran the target count {icount}"),
                native: Box::new(ArchState::of(cpu)),
            });
        }
        if let Some(what) = cpu.bus.mismatch.take() {
            return Err(Divergence { icount: cpu.icount, what, native: Box::new(ArchState::of(cpu)) });
        }
        Ok(())
    }

    pub fn step(&mut self, cpu: &mut Cpu<ReplayBus>, ev: &Event) -> Result<(), Divergence> {
        match ev {
            Event::Read { addr, size, value, endinit, .. } => {
                cpu.bus.reads.push_back((*addr, *size, *value, *endinit));
                self.stats.reads += 1;
            }
            Event::Write { addr, size, value, endinit, .. } => {
                cpu.bus.writes.push_back((*addr, *size, *value, *endinit));
                self.stats.writes += 1;
            }
            Event::Irq { icount, pipn } => {
                Self::run_to(cpu, *icount)?;
                cpu.set_pipn(*pipn);
                match cpu.take_interrupt(*pipn) {
                    Ok(true) => self.stats.irqs += 1,
                    r => {
                        return Err(Divergence {
                            icount: *icount,
                            what: format!("board took interrupt PIPN {pipn}, native take_interrupt -> {r:?}"),
                            native: Box::new(ArchState::of(cpu)),
                        })
                    }
                }
            }
            Event::AsyncTrap { icount, trap } => {
                Self::run_to(cpu, *icount)?;
                cpu.raise_async_trap(*trap);
            }
            Event::Check { icount, state, ram } => {
                Self::run_to(cpu, *icount)?;
                let native = Box::new(ArchState::of(cpu));
                let diff = state.diff(&native);
                if !diff.is_empty() {
                    let what = diff.iter().map(|(n, b, c)| format!("{n}: board {b:#010x} native {c:#010x}")).collect::<Vec<_>>().join(", ");
                    return Err(Divergence { icount: *icount, what, native });
                }
                for (addr, bytes) in ram.iter() {
                    if let Some(off) = first_ram_diff(cpu, *addr, bytes) {
                        let o = off as usize & !3;
                        let want = &bytes[o..(o + 16).min(bytes.len())];
                        let mut have = vec![0u8; want.len()];
                        let aligned_addr = addr.wrapping_add(o as u32);
                        let diff_addr = addr.wrapping_add(off);
                        cpu.read_bytes(aligned_addr, &mut have);
                        return Err(Divergence {
                            icount: *icount,
                            what: format!("RAM differs at {diff_addr:#010x}: board {want:02x?} native {have:02x?} (from {aligned_addr:#010x})"),
                            native: native.clone(),
                        });
                    }
                }
                if !cpu.bus.reads.is_empty() || !cpu.bus.writes.is_empty() {
                    return Err(Divergence {
                        icount: *icount,
                        what: format!(
                            "recorded peripheral accesses not performed by the native core: reads {:x?}, writes {:x?}",
                            cpu.bus.reads, cpu.bus.writes
                        ),
                        native,
                    });
                }
                self.stats.checks += 1;
                if let Some(from) = self.last_verified_from {
                    self.stats.verified_insns += icount.saturating_sub(from);
                }
                self.last_verified_from = Some(*icount);
            }
            Event::Sync { icount, state, ram } => {
                state.apply(cpu);
                for (addr, bytes) in ram.iter() {
                    for (i, w) in bytes.chunks(4096).enumerate() {
                        let a = addr.wrapping_add((i * 4096) as u32);
                        if first_ram_diff(cpu, a, w).is_some() {
                            cpu.write_bytes(a, w).expect("sync range is mapped");
                        }
                    }
                }
                cpu.icount = *icount;
                cpu.bus.reads.clear();
                cpu.bus.writes.clear();
                self.stats.syncs += 1;
                self.last_verified_from = Some(*icount);
            }
        }
        Ok(())
    }
}

/// A recorded execution reduced to what the native core needs to re-execute it without the board:
/// peripheral accesses in program order and actions at exact native instruction counts. `RunTo`
/// is only emitted before an action, so playing a trace runs the core in long free runs (this is
/// also the benchmark workload). Counts are instructions the native core executed since the trace
/// start (the board's counter differs, see the B3 defect in `tests/board_replay.rs`).
#[derive(Clone, Debug, PartialEq)]
pub enum TraceOp {
    /// Queue a recorded peripheral access (consumed in program order, see [`ReplayBus`]).
    Access { write: bool, addr: u32, size: u8, value: u64, endinit: bool },
    /// Run until exactly this many instructions have executed since the trace start.
    RunTo(u64),
    /// Take the interrupt with this PIPN now (must be accepted).
    Irq(u8),
    /// Publish the board ICU's ICR.PIPN.
    Pipn(u8),
    /// Overwrite memory: initial snapshot, board DMA writes, DFlash programming, resyncs.
    Mem(u32, Vec<u8>),
    /// Overwrite the architectural registers (start, resync after a known board defect).
    State(ArchState),
    /// Compare the architectural registers.
    Check(ArchState),
}

const ARCH_WORDS: usize = 43;

fn state_words(s: &ArchState) -> [u32; ARCH_WORDS] {
    let mut w = [0u32; ARCH_WORDS];
    w[..16].copy_from_slice(&s.d);
    w[16..32].copy_from_slice(&s.a);
    w[32..].copy_from_slice(&[s.pc, s.psw, s.pcxi, s.fcx, s.lcx, s.icr, s.isp, s.btv, s.biv, 0, 0]);
    w
}

fn words_state(w: &[u32]) -> ArchState {
    let mut s = ArchState::default();
    s.d.copy_from_slice(&w[..16]);
    s.a.copy_from_slice(&w[16..32]);
    [s.pc, s.psw, s.pcxi, s.fcx, s.lcx, s.icr, s.isp, s.btv, s.biv] = w[32..41].try_into().unwrap();
    s
}

/// Serialise a trace (little-endian, tag byte per op).
pub fn trace_to_bytes(ops: &[TraceOp]) -> Vec<u8> {
    let mut v = b"TCTRACE1".to_vec();
    for op in ops {
        match op {
            TraceOp::Access { write, addr, size, value, endinit } => {
                v.push(*write as u8 | (*endinit as u8) << 1);
                v.extend(addr.to_le_bytes());
                v.push(*size);
                v.extend(value.to_le_bytes());
            }
            TraceOp::RunTo(n) => {
                v.push(4);
                v.extend(n.to_le_bytes());
            }
            TraceOp::Irq(p) | TraceOp::Pipn(p) => v.extend([if matches!(op, TraceOp::Irq(_)) { 5 } else { 6 }, *p]),
            TraceOp::Mem(a, b) => {
                v.push(7);
                v.extend(a.to_le_bytes());
                v.extend((b.len() as u32).to_le_bytes());
                v.extend(b);
            }
            TraceOp::State(s) | TraceOp::Check(s) => {
                v.push(if matches!(op, TraceOp::State(_)) { 8 } else { 9 });
                state_words(s).iter().for_each(|w| v.extend(w.to_le_bytes()));
            }
        }
    }
    v
}

/// Inverse of [`trace_to_bytes`].
pub fn trace_from_bytes(b: &[u8]) -> Result<Vec<TraceOp>, String> {
    let mut b = b.strip_prefix(b"TCTRACE1").ok_or("not a trace")?;
    let mut ops = Vec::new();
    fn take<'a>(b: &mut &'a [u8], n: usize) -> Result<&'a [u8], String> {
        if b.len() < n {
            return Err("truncated trace".into());
        }
        let (h, t) = b.split_at(n);
        *b = t;
        Ok(h)
    }
    let u32_ = |x: &[u8]| u32::from_le_bytes(x.try_into().unwrap());
    while !b.is_empty() {
        let tag = take(&mut b, 1)?[0];
        ops.push(match tag {
            0..=3 => {
                let x = take(&mut b, 13)?;
                TraceOp::Access { write: tag & 1 != 0, endinit: tag & 2 != 0, addr: u32_(&x[..4]), size: x[4], value: u64::from_le_bytes(x[5..].try_into().unwrap()) }
            }
            4 => TraceOp::RunTo(u64::from_le_bytes(take(&mut b, 8)?.try_into().unwrap())),
            5 => TraceOp::Irq(take(&mut b, 1)?[0]),
            6 => TraceOp::Pipn(take(&mut b, 1)?[0]),
            7 => {
                let a = u32_(take(&mut b, 4)?);
                let n = u32_(take(&mut b, 4)?) as usize;
                TraceOp::Mem(a, take(&mut b, n)?.to_vec())
            }
            8 | 9 => {
                let x = take(&mut b, 4 * ARCH_WORDS)?;
                let w: Vec<u32> = x.chunks(4).map(u32_).collect();
                if tag == 8 { TraceOp::State(words_state(&w)) } else { TraceOp::Check(words_state(&w)) }
            }
            t => return Err(format!("bad trace tag {t}")),
        });
    }
    Ok(ops)
}

/// Play a trace on `cpu` (whose memory already holds the code images). Returns the number of
/// instructions executed, or the first divergence (failed `Check`, peripheral access mismatch,
/// refused interrupt).
pub fn play(cpu: &mut Cpu<ReplayBus>, ops: &[TraceOp]) -> Result<u64, Divergence> {
    let base = cpu.icount;
    for op in ops {
        match op {
            TraceOp::Access { write: false, addr, size, value, endinit } => cpu.bus.reads.push_back((*addr, *size, *value, *endinit)),
            TraceOp::Access { write: true, addr, size, value, endinit } => cpu.bus.writes.push_back((*addr, *size, *value, *endinit)),
            TraceOp::RunTo(n) => Replayer::run_to(cpu, base.saturating_add(*n))?,
            TraceOp::Irq(p) => {
                cpu.set_pipn(*p);
                if cpu.take_interrupt(*p) != Ok(true) {
                    return Err(Divergence { icount: cpu.icount.saturating_sub(base), what: format!("interrupt {p} refused"), native: Box::new(ArchState::of(cpu)) });
                }
            }
            TraceOp::Pipn(p) => cpu.set_pipn(*p),
            TraceOp::Mem(a, b) => cpu.write_bytes(*a, b).map_err(|e| Divergence {
                icount: cpu.icount.saturating_sub(base),
                what: format!("trace memory write to unmapped {e:#x}"),
                native: Box::new(ArchState::of(cpu)),
            })?,
            TraceOp::State(s) => s.apply(cpu),
            TraceOp::Check(s) => {
                let native = ArchState::of(cpu);
                let diff = s.diff(&native);
                if !diff.is_empty() {
                    let what = diff.iter().map(|(n, b, c)| format!("{n}: trace {b:#010x} native {c:#010x}")).collect::<Vec<_>>().join(", ");
                    return Err(Divergence { icount: cpu.icount.saturating_sub(base), what, native: Box::new(native) });
                }
            }
        }
    }
    Ok(cpu.icount.saturating_sub(base))
}

fn first_ram_diff<B: Bus>(cpu: &Cpu<B>, addr: u32, want: &[u8]) -> Option<u32> {
    const CHUNK: usize = 4096;
    let mut buf = [0u8; CHUNK];
    for (i, w) in want.chunks(CHUNK).enumerate() {
        let a = addr.wrapping_add((i * CHUNK) as u32);
        let have: &[u8] = match cpu.mem.host_r(a, w.len() as u32) {
            // SAFETY: host_r validated the whole range.
            Some(p) => unsafe { std::slice::from_raw_parts(p, w.len()) },
            None => {
                if !cpu.read_bytes(a, &mut buf[..w.len()]) {
                    return Some((i * CHUNK) as u32);
                }
                &buf[..w.len()]
            }
        };
        if have != w {
            let j = have.iter().zip(w).position(|(x, y)| x != y).unwrap();
            return Some((i * CHUNK + j) as u32);
        }
    }
    None
}

/// Convenience: a native TC1.6/TC1.3.1 core over a replay bus.
pub fn native(arch: Arch, mem: crate::Memory) -> Cpu<ReplayBus> {
    Cpu::new(arch, mem, ReplayBus::default())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Memory;

    #[test]
    fn sync_then_check_detects_register_divergence() {
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x1000], true);
        m.map(0xD000_0000, r, 0, 0x1000);
        let mut cpu = native(Arch::Tc16, m);
        let mut rp = Replayer::new();
        let mut s = ArchState { pc: 0xD000_0000, ..Default::default() };
        let ram = [0xAAu8; 16];
        let ranges = [(0xD000_0100u32, &ram[..])];
        rp.step(&mut cpu, &Event::Sync { icount: 100, state: s, ram: &ranges }).unwrap();
        assert_eq!(cpu.icount, 100);
        // zero instructions later the state must match exactly
        rp.step(&mut cpu, &Event::Check { icount: 100, state: s, ram: &ranges }).unwrap();
        s.d[3] = 1;
        let e = rp.step(&mut cpu, &Event::Check { icount: 100, state: s, ram: &[] }).unwrap_err();
        assert!(e.what.contains("D3"), "{}", e.what);
    }

    #[test]
    fn replay_bus_returns_recorded_values_in_order() {
        let mut b = ReplayBus::default();
        b.reads.extend([(0xF000_0010, 4, 7, true), (0xF000_0014, 2, 9, true)]);
        assert_eq!(b.read(0xF000_0010, 4), Ok(7));
        assert!(b.mismatch.is_none());
        assert_eq!(b.read(0xF000_0018, 2), Ok(0));
        assert!(b.mismatch.as_deref().unwrap().contains("0xf0000014"), "{:?}", b.mismatch);
    }

    #[test]
    fn replay_bus_joins_split_accesses_and_checks_writes() {
        let mut b = ReplayBus::default();
        b.reads.extend([(0xF000_0010, 4, 0x1111_2222, true), (0xF000_0014, 4, 0x3333_4444, true)]);
        assert_eq!(b.read(0xF000_0010, 8), Ok(0x3333_4444_1111_2222));
        b.writes.extend([(0xF000_0020, 1, 0xAB, false)]);
        b.write(0xF000_0020, 1, 0xAB).unwrap();
        assert!(b.mismatch.is_none() && b.writes.is_empty());
        assert!(!b.endinit(), "ENDINIT follows the recorded state after the write");
        b.writes.extend([(0xF000_0020, 4, 1, false)]);
        b.write(0xF000_0020, 4, 2).unwrap();
        assert!(b.mismatch.is_some());
    }

    #[test]
    fn replay_bus_rejects_zero_and_overflowing_recorded_sizes() {
        // Recorded size 0 must fail immediately instead of looping forever.
        let mut b = ReplayBus::default();
        b.reads.push_back((0xF000_0010, 0, 0xAA, true));
        assert_eq!(b.read(0xF000_0010, 4), Ok(0));
        assert!(b.mismatch.is_some());

        // Recorded size 254 after a 4-byte fragment must NOT wrap `done + s` in u8 (`4 + 254 == 2 <= 8`).
        let mut b = ReplayBus::default();
        b.reads.extend([(0xF000_0010, 4, 0x1122_3344, true), (0xF000_0014, 254, 0x55, true)]);
        assert_eq!(b.read(0xF000_0010, 8), Ok(0));
        assert!(b.mismatch.is_some());

        // Native size > 8 is rejected cleanly without shift overflow.
        let mut b = ReplayBus::default();
        b.reads.extend([(0xF000_0010, 8, 1, true), (0xF000_0018, 8, 2, true)]);
        assert_eq!(b.read(0xF000_0010, 16), Ok(0));
        assert!(b.mismatch.is_some());
    }
}
