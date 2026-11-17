//! Architectural state, guest memory access and the execution loop.

use crate::arch::{csfr, csfr_index, icr, psw, Arch, Trap, CSFR_TABLE};
use crate::bus::Bus;
use crate::cache::{BlockCache, Spans, MAX_BLOCK_INSNS};
use crate::mem::{CodeWrite, Memory};
use crate::op::{self, Flow, Insn, StopReason};

/// Why [`Cpu::run`] returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exit {
    /// Exactly `budget` instructions were executed.
    Budget,
    /// An instruction asked the caller to act before continuing (see [`StopReason`]).
    Stopped(StopReason),
    /// The bus requested a stop after a peripheral write ([`Bus::stop_requested`]).
    Requested,
    /// Instruction fetch failed at the PSE trap vector too (no executable memory at `pc`): the core
    /// cannot make progress.
    FetchFault { pc: u32 },
    /// [`Bus::hook`] marked `pc`; nothing of it has executed yet. The next `run` executes it.
    Hook { pc: u32 },
}

/// TriCore CPU: registers, CSFRs, memory, peripheral bus and the predecoded block cache.
pub struct Cpu<B: Bus> {
    /// Data registers D0..D15.
    pub d: [u32; 16],
    /// Address registers A0..A15 (A10 = SP, A11 = RA).
    pub a: [u32; 16],
    /// Address of the next instruction to execute (bit 0 always 0).
    pub pc: u32,
    pub psw: u32,
    pub pcxi: u32,
    pub fcx: u32,
    pub lcx: u32,
    pub icr: u32,
    pub isp: u32,
    pub btv: u32,
    pub biv: u32,
    pub syscon: u32,
    /// Remaining CSFRs, indexed like [`CSFR_TABLE`].
    pub csfrs: [u32; CSFR_TABLE.len()],
    /// Bit `s` = segment `s` has the peripheral-space attribute (`ldst::peripheral`).
    pub(crate) periph_mask: u16,
    /// Instructions executed since construction. An instruction that raises a synchronous trap
    /// counts as executed; trap and interrupt entry sequences do not count.
    pub icount: u64,
    pub arch: Arch,
    pub mem: Memory,
    pub bus: B,
    pub(crate) cache: BlockCache,
    /// Set by stores into translated code and by bus stop requests; checked after every instruction.
    attention: bool,
    stop_requested: bool,
    pending_code_writes: Vec<CodeWrite>,
    /// Start address of the executing block (reported to [`Bus::clock`]).
    block_pc: u32,
    /// Address of the executing instruction for the memory observers (`trace-data-stores`).
    #[cfg(feature = "trace-data-stores")]
    pub(crate) exec_pc: u32,
    /// Hooked block start already reported by [`Exit::Hook`] (odd = none).
    hook_passed: u32,
    /// Threaded execution (see [`Handler`]): instruction count at the start of the executing block,
    /// its cache index, the budget end, how many more blocks may be chained, and the exit the
    /// handlers report (instructions retired in the last block, how it ended).
    pub(crate) t_base: u64,
    pub(crate) t_blk: u32,
    pub(crate) t_end: u64,
    t_chain: u32,
    t_ret: u32,
    t_flow: Flow,
    /// [`TRAMPOLINE`] builds: the record whose handler runs next (null = none).
    t_next: *const Insn,
}

// SAFETY: `t_next` is only dereferenced during `Cpu::run`, which owns the cache it points into.
unsafe impl<B: Bus + Send> Send for Cpu<B> {}

/// Threaded-code handler: executes the instruction record and, while execution falls through,
/// tail-calls the handler of the next record of the block (direct threading, Bell 1973; one
/// indirect branch per handler instead of one shared dispatch branch, Ertl & Gregg 2003). Every
/// cached block ends with an [`end`] record. A control transfer to a block linked as a successor
/// of the current one continues there directly when the whole block fits the budget and needs no
/// hook stop ([`chain`]; block chaining as QEMU's `goto_tb`, Bellard 2005); anything else returns
/// to [`Cpu::run`] through [`Cpu::t_exit`]. Release builds turn the calls into jumps (sibling
/// calls, checked in the release disassembly; the depth without that is bounded by [`CHAIN_MAX`]
/// blocks); debug builds return the next record to a loop in [`Cpu::run`] instead ([`TRAMPOLINE`]).
pub(crate) type Handler<B> = fn(&mut Cpu<B>, &Insn);

/// Blocks chained per entry from [`Cpu::run`].
const CHAIN_MAX: u32 = 256;

/// Debug builds: no sibling calls, and `#[inline(always)]` executors give every handler a large
/// unoptimised frame, so handlers hand the next record to the loop in [`Cpu::run`].
const TRAMPOLINE: bool = cfg!(debug_assertions);

/// Run the handler of record `r` (directly, or through the [`TRAMPOLINE`] loop).
#[inline(always)]
fn call<B: Bus>(cpu: &mut Cpu<B>, r: &Insn) {
    if TRAMPOLINE {
        cpu.t_next = r;
        return;
    }
    // SAFETY: `h` was set by `Cpu::<B>::translate` (same `B`) when the block was cached.
    let f = unsafe { std::mem::transmute::<usize, Handler<B>>(r.h) };
    f(cpu, r)
}

fn handler<B: Bus, const ID: u8>(cpu: &mut Cpu<B>, i: &Insn) {
    #[cfg(feature = "trace-data-stores")]
    if !op::hot::pure(ID) {
        cpu.exec_pc = i.pc;
    }
    // id 0 (and the unused table entries): generic execution
    let generic = ID == 0 || ID as usize >= op::hot::COUNT;
    if generic || op::hot::fast(ID) {
        if generic || op::try_fast(ID, cpu, i).is_none() {
            return slow(cpu, i);
        }
        // the fast path touched neither the bus nor translated code: no attention, and nothing
        // observed the instruction count
        return next(cpu, i);
    }
    if ID == op::hot::FPU {
        // large executor out of line; pure (no memory)
        let flow = unpack(fpu_packed(cpu, i));
        return done(cpu, i, flow, true);
    }
    if !op::hot::pure(ID) {
        cpu.icount = cpu.t_base + i.idx as u64;
    }
    let flow = op::exec_id(ID, cpu, i, i.pc, i.len as u32);
    done(cpu, i, flow, op::hot::pure(ID))
}

#[inline(never)]
pub(crate) fn fpu_packed<B: Bus>(cpu: &mut Cpu<B>, i: &Insn) -> u64 {
    pack(op::exec_id(op::hot::FPU, cpu, i, i.pc, i.len as u32))
}

/// Continue after instruction `i` returned `flow` (`pure`: it cannot have set the attention flag).
#[inline(always)]
pub(crate) fn done<B: Bus>(cpu: &mut Cpu<B>, i: &Insn, flow: Flow, pure: bool) {
    match flow {
        Flow::Next if pure || !cpu.attention => next(cpu, i),
        Flow::Jump(t) if pure || !cpu.attention => chain(cpu, i.idx as u32 + 1, t),
        // re-arbitration that takes no interrupt now: carry on (the caller would only re-run)
        Flow::Stop { next, why: StopReason::IrqCheck } if !cpu.attention && !cpu.bus.irq_check(cpu.icr) => {
            chain(cpu, i.idx as u32 + 1, next)
        }
        // attention (stores into translated code, bus stop requests), traps, stops
        flow => cpu.t_flow_exit(i, flow),
    }
}

/// The block ends after `retired` instructions with a transfer to `t`: continue in the successor
/// block linked for `t` if it may run now, else return to [`Cpu::run`].
#[inline(always)]
pub(crate) fn chain<B: Bus>(cpu: &mut Cpu<B>, retired: u32, t: u32) {
    debug_assert_eq!(t & 1, 0);
    if cpu.t_chain != 0 {
        // linked successor (never a block with a hook), else the direct-mapped front (returns,
        // indirect jumps)
        let next = match cpu.cache.block(cpu.t_blk).link(t) {
            Some(l) => Some((l.idx, l.first, l.len)),
            None => cpu.cache.probe(t).and_then(|nb| {
                let b = cpu.cache.block(nb);
                (!b.hook).then_some((nb, b.first, b.len))
            }),
        };
        if let Some((nb, first, len)) = next {
            let base = cpu.t_base + retired as u64;
            // the same conditions as the loop in `run_inner`: whole block within the budget, no hook
            if cpu.t_end - base >= len as u64 {
                let first_ptr = cpu.cache.rec(first) as *const Insn;
                cpu.t_chain -= 1;
                cpu.t_base = base;
                cpu.t_blk = nb;
                cpu.block_pc = t;
                // SAFETY: `first_ptr` points into `cpu.cache.insns`, which does not change while handlers run.
                return call(cpu, unsafe { &*first_ptr });
            }
        }
    }
    cpu.t_exit(retired, Flow::Jump(t))
}

/// Continue with the record after `i`.
#[inline(always)]
pub(crate) fn next<B: Bus>(cpu: &mut Cpu<B>, i: &Insn) {
    // SAFETY: every cached block ends with an `end` record, so the next record exists.
    call(cpu, unsafe { &*(i as *const Insn).add(1) })
}

/// Any instruction through the full executor (generic ids, and the hot loads/stores whose fast
/// path does not apply: peripherals, traps, mode checks, stores into translated code, ...).
#[inline(never)]
pub(crate) fn slow<B: Bus>(cpu: &mut Cpu<B>, i: &Insn) {
    #[cfg(feature = "trace-data-stores")]
    {
        cpu.exec_pc = i.pc;
    }
    // exact count while the instruction executes (peripheral accesses read it)
    cpu.icount = cpu.t_base + i.idx as u64;
    let flow = unpack(exec_packed(cpu, i));
    done(cpu, i, flow, false)
}

/// [`op::exec_hot`] out of line, its result packed into a register ([`slow`] keeps no stack slot
/// alive across its final call, which stays a tail call).
#[inline(never)]
fn exec_packed<B: Bus>(cpu: &mut Cpu<B>, i: &Insn) -> u64 {
    pack(op::exec_hot(cpu, i, i.pc, i.len as u32))
}

#[inline(always)]
fn pack(flow: Flow) -> u64 {
    match flow {
        Flow::Next => 0,
        Flow::Jump(t) => 1 | (t as u64) << 32,
        Flow::Trap { trap, ret } => 2 | (trap.class as u64) << 8 | (trap.tin as u64) << 16 | (ret as u64) << 32,
        Flow::Stop { next, why } => {
            let w = match why {
                StopReason::IrqCheck => 0,
                StopReason::Wait => 1,
                StopReason::Debug => 2,
            };
            3 | w << 8 | (next as u64) << 32
        }
    }
}

#[inline(always)]
pub(crate) fn unpack(x: u64) -> Flow {
    let hi = (x >> 32) as u32;
    match x & 0xFF {
        0 => Flow::Next,
        1 => Flow::Jump(hi),
        2 => Flow::Trap { trap: Trap::new((x >> 8) as u8, (x >> 16) as u8), ret: hi },
        _ => Flow::Stop {
            next: hi,
            why: match (x >> 8) as u8 {
                0 => StopReason::IrqCheck,
                1 => StopReason::Wait,
                _ => StopReason::Debug,
            },
        },
    }
}

/// Record after the last instruction of a block: execution falls through to `pc`.
fn end<B: Bus>(cpu: &mut Cpu<B>, i: &Insn) {
    chain(cpu, i.idx as u32, i.pc)
}

macro_rules! handler_table {
    ($($n:literal)*) => { [$(handler::<B, $n> as Handler<B>,)*] };
}

const _: () = assert!(op::hot::COUNT <= 128);

impl<B: Bus> Cpu<B> {
    /// A core in its reset state (V1: PSW = 0B80H, ICR/PCXI/SYSCON = 0; implementation-specific
    /// registers BTV, BIV, ISP, FCX, LCX = 0 — the board sets them, e.g. TC1791 BTV = A0000100H).
    pub fn new(arch: Arch, mem: Memory, bus: B) -> Self {
        Self {
            d: [0; 16],
            a: [0; 16],
            pc: 0,
            psw: psw::RESET,
            pcxi: 0,
            fcx: 0,
            lcx: 0,
            icr: 0,
            isp: 0,
            btv: 0,
            biv: 0,
            syscon: 0,
            csfrs: [0; CSFR_TABLE.len()],
            periph_mask: match arch {
                Arch::Tc13 | Arch::Tc131 => 0xC000,
                Arch::Tc16 => 0x8000,
            },
            icount: 0,
            arch,
            mem,
            bus,
            cache: BlockCache::default(),
            attention: false,
            stop_requested: false,
            pending_code_writes: Vec::new(),
            block_pc: 0,
            #[cfg(feature = "trace-data-stores")]
            exec_pc: 0,
            hook_passed: 1,
            t_base: 0,
            t_blk: 0,
            t_end: 0,
            t_chain: 0,
            t_ret: 0,
            t_flow: Flow::Next,
            t_next: std::ptr::null(),
        }
    }

    /// Threaded-code handler of each [`op::hot`] id.
    const HANDLERS: [Handler<B>; 128] = handler_table!(
        0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
        32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63
        64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79 80 81 82 83 84 85 86 87 88 89 90 91 92 93 94 95
        96 97 98 99 100 101 102 103 104 105 106 107 108 109 110 111 112 113 114 115 116 117 118 119 120 121 122 123 124 125 126 127
    );

    /// End of a threaded block: `retired` instructions of the block completed, then `flow`.
    #[inline(always)]
    fn t_exit(&mut self, retired: u32, flow: Flow) {
        self.t_ret = retired;
        self.t_flow = flow;
    }

    /// End of a threaded block after instruction `i` returned `flow`; a fall-through ends it only
    /// when the attention flag is set (stores into translated code, bus stop requests).
    #[inline(always)]
    fn t_flow_exit(&mut self, i: &Insn, flow: Flow) {
        let flow = match flow {
            Flow::Next => Flow::Jump(i.pc.wrapping_add(i.len as u32)),
            f => f,
        };
        self.t_exit(i.idx as u32 + 1, flow);
    }

    // ---- execution ------------------------------------------------------------------------------

    /// Execute until exactly `budget` instructions have run or an instruction/the bus requests a
    /// stop. Interrupts are not taken here: the caller arbitrates between calls and uses
    /// [`Cpu::take_interrupt`] (instruction-boundary delivery).
    pub fn run(&mut self, budget: u64) -> Exit {
        let end = self.icount.saturating_add(budget);
        self.run_inner(end)
    }

    fn run_inner(&mut self, end: u64) -> Exit {
        // V1 §3.1.2 p. 3-4: PC[0] is hardwired to 0; enforce on entry so direct writes to `pub pc`
        // cannot hit the odd-address `NO_SUCC` / `hook_passed` sentinels (`1`).
        self.pc &= !1;
        let mut fetch_fault: Option<u32> = None;
        // previously executed block (successor chaining); u32::MAX = none
        let mut prev = u32::MAX;
        loop {
            if self.attention {
                // set by stores into translated code and by bus stop requests
                self.attention = false;
                for w in std::mem::take(&mut self.pending_code_writes) {
                    self.cache.invalidate(w.region, w.start, w.end);
                }
                if self.stop_requested {
                    self.stop_requested = false;
                    return Exit::Requested;
                }
            }
            if self.icount >= end {
                return Exit::Budget;
            }
            let bi = match self.cache.chained(prev, self.pc) {
                Some(i) => i,
                None => {
                    let i = match self.cache.lookup(self.pc) {
                        Some(i) => i,
                        None => match self.translate(self.pc) {
                            Some((insns, spans)) => {
                                let hook = self.bus.hook(self.pc);
                                self.cache.insert(self.pc, &insns, spans, hook)
                            }
                            None => {
                                // Program fetch synchronous error (V1 Table 6-1, class 4 TIN 1).
                                if fetch_fault == Some(self.pc) {
                                    return Exit::FetchFault { pc: self.pc };
                                }
                                let pc = self.pc;
                                self.enter_trap(Trap::PSE, pc);
                                fetch_fault = Some(self.pc);
                                prev = u32::MAX;
                                continue;
                            }
                        },
                    };
                    self.cache.link(prev, self.pc, i);
                    i
                }
            };
            fetch_fault = None;
            let blk = self.cache.block(bi);
            let (blk_start, blk_len, blk_hook) = (blk.start, blk.len as usize, blk.hook);
            let first_ptr = self.cache.first(blk) as *const Insn;
            if blk_hook {
                if self.hook_passed != blk_start {
                    self.hook_passed = blk_start;
                    return Exit::Hook { pc: blk_start };
                }
                self.hook_passed = 1;
            }
            let mut pc = blk_start;
            self.block_pc = pc;
            // the budget is checked once per block: run at most the instructions it allows
            let base = self.icount;
            if end - base >= blk_len as u64 {
                // whole block: threaded code, possibly chaining into successor blocks
                self.t_base = base;
                self.t_blk = bi;
                self.t_end = end;
                self.t_chain = CHAIN_MAX;
                // SAFETY: `first_ptr` points into `self.cache.insns`, which does not change while handlers run.
                call(self, unsafe { &*first_ptr });
                while TRAMPOLINE && !self.t_next.is_null() {
                    // SAFETY: set by `call` to a record of `cache`, which has not changed since.
                    let r = unsafe { &*std::mem::replace(&mut self.t_next, std::ptr::null()) };
                    // SAFETY: set by `translate` from `HANDLERS` of this `B`.
                    let f = unsafe { std::mem::transmute::<usize, Handler<B>>(r.h) };
                    f(self, r);
                }
                // the last block executed
                let (base, bi) = (self.t_base, self.t_blk);
                let done = self.t_ret as u64;
                match self.t_flow {
                    Flow::Jump(t) => pc = t,
                    Flow::Trap { trap, ret } => {
                        // trap entry sees the count of the trapping instruction
                        self.icount = base + done - 1;
                        self.enter_trap(trap, ret);
                        pc = self.pc;
                    }
                    Flow::Stop { next, why } => {
                        self.icount = base + done;
                        self.pc = next;
                        return Exit::Stopped(why);
                    }
                    Flow::Next => unreachable!("threaded block exits with a control transfer"),
                }
                self.icount = base + done;
                self.pc = pc;
                prev = bi;
                continue;
            }
            // the budget ends inside the block: instruction by instruction (once per quantum)
            let allowed = (end - base) as usize;
            let mut done = 0u64;
            for k in 0..allowed {
                // SAFETY: `first_ptr..first_ptr + blk_len` is valid in `self.cache.insns` (`allowed < blk_len`).
                let insn = unsafe { &*first_ptr.add(k) };
                let len = insn.len as u32;
                debug_assert_eq!((pc, len), (insn.pc, insn.len as u32));
                #[cfg(feature = "trace-data-stores")]
                {
                    self.exec_pc = insn.pc;
                }
                if !TRAMPOLINE && op::hot::fast(insn.hot) && op::try_fast(insn.hot, self, insn).is_some() {
                    done += 1;
                    pc = pc.wrapping_add(len);
                    continue;
                }
                // exact count while the instruction executes (peripheral accesses read it)
                self.icount = base + done;
                // out of line: in debug builds the inlined dispatch has a frame of ~1 MB
                let flow = unpack(exec_packed(self, insn));
                done += 1;
                match flow {
                    Flow::Next => pc = pc.wrapping_add(len),
                    Flow::Jump(t) => {
                        pc = t;
                        break;
                    }
                    Flow::Trap { trap, ret } => {
                        self.enter_trap(trap, ret);
                        pc = self.pc;
                        break;
                    }
                    Flow::Stop { next, why } => {
                        self.icount = base + done;
                        self.pc = next;
                        return Exit::Stopped(why);
                    }
                }
                if self.attention {
                    break;
                }
            }
            self.icount = base + done;
            self.pc = pc;
            prev = bi;
        }
    }

    /// Decode a block starting at `pc` (its records followed by the `end` record). `None` if not
    /// even the first instruction can be fetched.
    fn translate(&mut self, pc: u32) -> Option<(Vec<Insn>, Spans)> {
        let mut insns: Vec<Insn> = Vec::with_capacity(16);
        let mut a = pc;
        let mut spans: Vec<(crate::RegionId, u32, u32)> = Vec::with_capacity(2);
        while insns.len() < MAX_BLOCK_INSNS {
            if !insns.is_empty() && self.bus.hook(a) {
                break; // a hooked address starts its own block
            }
            let Some(lo) = self.mem.fetch16(a) else { break };
            let len = crate::fmt::insn_len(lo);
            let hi = if len == 4 {
                match self.mem.fetch16(a.wrapping_add(2)) {
                    Some(h) => h,
                    None => break,
                }
            } else {
                0
            };
            let insn = op::decode(lo, hi, a, self.arch);
            for (x, n) in [(a, 2), (a.wrapping_add(2), len - 2)] {
                if n == 0 {
                    continue;
                }
                let (r, off) = self.mem.locate(x)?;
                match spans.last_mut() {
                    Some(s) if s.0 == r && s.2 == off => s.2 += n,
                    _ => spans.push((r, off, off + n)),
                }
            }
            insns.push(insn);
            a = a.wrapping_add(len);
            if insn.op.ends_block() || spans.len() > 2 {
                break;
            }
        }
        if insns.is_empty() {
            return None;
        }
        if spans.len() > 2 {
            // keep the block within two host spans: drop the instruction that opened a third one
            insns.pop();
            spans.truncate(2);
        }
        for &(r, s, e) in &spans {
            self.mem.mark_code(r, s, e - s);
        }
        // threaded code: handlers, positions and the closing `end` record (falls through)
        for (i, insn) in insns.iter_mut().enumerate() {
            insn.idx = i as u8;
            insn.h = Self::HANDLERS[insn.hot as usize] as usize;
        }
        // whole-block superinstruction (the per-instruction handlers stay for its slow paths)
        if let Some(h) = crate::fused::select::<B>(&insns) {
            insns[0].h = h as usize;
        }
        let last = insns[insns.len() - 1];
        let next = last.pc.wrapping_add(last.len as u32);
        insns.push(Insn { op: op::Op::Invalid, pc: next, len: 0, hot: 0, idx: insns.len() as u8, h: end::<B> as Handler<B> as usize });
        let first = spans[0];
        Some((insns, [first, *spans.get(1).unwrap_or(&first)]))
    }

    /// Drop predecoded code for `[addr, addr+len)` (call after the board modifies memory behind the
    /// core's back, e.g. flash programming or a DMA into RAM code).
    pub fn invalidate_code(&mut self, addr: u32, len: u32) {
        // Intersect `[addr, addr+len)` with each 64 KB guest page's mapped span.
        // For writable RAM pages, `F_CODE` + the per-granule `r.code` bitmap let us skip pages/granules
        // that hold only data (e.g. high-rate FADC/SPI DMA buffers) in O(1) without scanning blocks.
        let end = (addr as u64 + len as u64).min(1u64 << 32);
        let mut a = addr as u64;
        while a < end {
            let page_idx = (a >> 16) as usize;
            let page_base = (page_idx as u64) << 16;
            let e = end.min(page_base + 0x1_0000);
            let lo = (a - page_base) as u32;
            let hi = (e - page_base) as u32;
            if let Some((r, rs, re, flags)) = self.mem.page_span(page_idx, lo, hi) {
                if flags & crate::mem::Memory::F_WRITABLE_FLAG != 0 {
                    if flags & crate::mem::Memory::F_CODE_FLAG != 0 {
                        if let Some(cw) = self.mem.take_code(r, rs, re - rs) {
                            self.cache.invalidate(cw.region, cw.start, cw.end);
                        }
                    }
                } else {
                    self.cache.invalidate(r, rs, re);
                }
            }
            a = e;
        }
    }

    /// Drop all predecoded code.
    pub fn flush_code_cache(&mut self) {
        self.cache.clear();
        self.pending_code_writes.clear();
        self.hook_passed = 1;
    }

    /// Number of predecoded blocks currently cached.
    pub fn cached_blocks(&self) -> usize {
        self.cache.len()
    }

    // ---- guest memory -------------------------------------------------------------------------

    /// Host pointer of the 64-byte context save area at `ea` when it is plain writable memory
    /// (stores into translated code are reported as usual).
    #[inline(always)]
    pub(crate) fn csa_host(&mut self, ea: u32) -> Option<*mut u8> {
        let (p, code) = self.mem.host_w(ea, 64)?;
        if let Some(w) = code {
            self.code_written(w);
        }
        Some(p)
    }

    #[inline(always)]
    fn code_written(&mut self, w: CodeWrite) {
        self.pending_code_writes.push(w);
        self.attention = true;
    }

    #[inline(always)]
    pub fn ld8(&mut self, ea: u32) -> Result<u8, Trap> {
        match self.mem.host_r(ea, 1) {
            // SAFETY: host_r validated the range.
            Some(p) => {
                let value = unsafe { *p };
                #[cfg(feature = "trace-data-stores")]
                self.bus.data_read(self.exec_pc, ea, 1, value as u64);
                Ok(value)
            }
            None => self.ld_slow(ea, 1).map(|v| v as u8),
        }
    }

    #[inline(always)]
    pub fn ld16(&mut self, ea: u32) -> Result<u16, Trap> {
        match self.mem.host_r(ea, 2) {
            // SAFETY: host_r validated the range.
            Some(p) => {
                let value = u16::from_le_bytes(unsafe { *(p as *const [u8; 2]) });
                #[cfg(feature = "trace-data-stores")]
                self.bus.data_read(self.exec_pc, ea, 2, value as u64);
                Ok(value)
            }
            None => self.ld_slow(ea, 2).map(|v| v as u16),
        }
    }

    #[inline(always)]
    pub fn ld32(&mut self, ea: u32) -> Result<u32, Trap> {
        match self.mem.host_r(ea, 4) {
            // SAFETY: host_r validated the range.
            Some(p) => {
                let value = u32::from_le_bytes(unsafe { *(p as *const [u8; 4]) });
                #[cfg(feature = "trace-data-stores")]
                self.bus.data_read(self.exec_pc, ea, 4, value as u64);
                Ok(value)
            }
            None => self.ld_slow(ea, 4).map(|v| v as u32),
        }
    }

    #[inline(always)]
    pub fn ld64(&mut self, ea: u32) -> Result<u64, Trap> {
        match self.mem.host_r(ea, 8) {
            // SAFETY: host_r validated the range.
            Some(p) => {
                let value = u64::from_le_bytes(unsafe { *(p as *const [u8; 8]) });
                #[cfg(feature = "trace-data-stores")]
                self.bus.data_read(self.exec_pc, ea, 8, value);
                Ok(value)
            }
            None => self.ld_slow(ea, 8),
        }
    }

    #[inline(always)]
    pub fn st8(&mut self, ea: u32, v: u8) -> Result<(), Trap> {
        self.st_n::<1>(ea, [v])
    }

    #[inline(always)]
    pub fn st16(&mut self, ea: u32, v: u16) -> Result<(), Trap> {
        self.st_n(ea, v.to_le_bytes())
    }

    #[inline(always)]
    pub fn st32(&mut self, ea: u32, v: u32) -> Result<(), Trap> {
        self.st_n(ea, v.to_le_bytes())
    }

    #[inline(always)]
    pub fn st64(&mut self, ea: u32, v: u64) -> Result<(), Trap> {
        self.st_n(ea, v.to_le_bytes())
    }

    #[inline(always)]
    fn st_n<const N: usize>(&mut self, ea: u32, bytes: [u8; N]) -> Result<(), Trap> {
        #[cfg(feature = "trace-data-stores")]
        {
            let mut v = [0u8; 8];
            v[..N].copy_from_slice(&bytes);
            self.bus.data_write(self.exec_pc, ea, N as u8, u64::from_le_bytes(v));
        }
        match self.mem.host_w(ea, N as u32) {
            Some((p, code)) => {
                // SAFETY: host_w validated the range.
                unsafe { *(p as *mut [u8; N]) = bytes };
                if let Some(w) = code {
                    self.code_written(w);
                }
                Ok(())
            }
            None => {
                let mut v = [0u8; 8];
                v[..N].copy_from_slice(&bytes);
                self.st_slow(ea, N as u8, u64::from_le_bytes(v))
            }
        }
    }

    /// Loads outside one host mapping: bytes split across mappings, else the peripheral bus.
    /// A bus error is a data access synchronous error (DSE, V1 Table 6-1 class 4 TIN 2).
    fn ld_slow(&mut self, ea: u32, size: u8) -> Result<u64, Trap> {
        if (ea & 0xFFFF) + size as u32 > 0x1_0000 {
            let mut b = [0u8; 8];
            if self.mem.read_bytes(ea, &mut b[..size as usize]) {
                let value = u64::from_le_bytes(b);
                #[cfg(feature = "trace-data-stores")]
                self.bus.data_read(self.exec_pc, ea, size, value);
                return Ok(value);
            }
        }
        self.bus.clock(self.icount, self.block_pc);
        let v = self.bus.read(ea, size).map_err(|_| Trap::DSE)?;
        if self.bus.stop_requested() {
            self.stop_requested = true;
            self.attention = true;
        }
        Ok(v)
    }

    /// Stores outside writable memory: peripherals and read-only memory (flash command cycles) go to
    /// the bus. A bus error is a data access asynchronous error (DAE, V1 Table 6-1 class 4 TIN 3).
    fn st_slow(&mut self, ea: u32, size: u8, v: u64) -> Result<(), Trap> {
        if (ea & 0xFFFF) + size as u32 > 0x1_0000
            && (0..size as u32).all(|i| self.mem.host_w(ea.wrapping_add(i), 1).is_some())
        {
            for i in 0..size as u32 {
                let (p, code) = self.mem.host_w(ea.wrapping_add(i), 1).unwrap();
                // SAFETY: host_w validated the 1-byte range.
                unsafe { *p = (v >> (8 * i)) as u8 };
                if let Some(w) = code {
                    self.code_written(w);
                }
            }
            return Ok(());
        }
        self.bus.clock(self.icount, self.block_pc);
        self.bus.write(ea, size, v).map_err(|_| Trap::DAE)?;
        if self.bus.stop_requested() {
            self.stop_requested = true;
            self.attention = true;
        }
        Ok(())
    }

    /// Debug/loader read of guest memory (no peripheral access).
    pub fn read_bytes(&self, addr: u32, out: &mut [u8]) -> bool {
        self.mem.read_bytes(addr, out)
    }

    /// Debug/loader write of guest memory, also into read-only regions; drops affected code.
    pub fn write_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), u32> {
        let mut hits = self.mem.write_bytes(addr, data)?;
        hits.sort_by_key(|w| (w.region, w.start));
        let mut cur: Option<CodeWrite> = None;
        for w in hits.into_iter().chain([CodeWrite { region: crate::RegionId(u16::MAX), start: 0, end: 0 }]) {
            match cur.as_mut() {
                Some(c) if c.region == w.region && w.start <= c.end => c.end = c.end.max(w.end),
                _ => {
                    if let Some(c) = cur.replace(w) {
                        self.cache.invalidate(c.region, c.start, c.end);
                    }
                }
            }
        }
        Ok(())
    }

    // ---- CSFRs --------------------------------------------------------------------------------

    /// MFCR semantics for CSFR offset `off` executed at `pc` (V2 MFCR p. 3-217; undefined
    /// locations read 0, V1 p. 3-18).
    pub fn csfr_read(&self, off: u16, pc: u32) -> u32 {
        match off {
            csfr::PCXI => self.pcxi,
            csfr::PSW => self.psw,
            csfr::PC => pc,
            csfr::SYSCON => self.syscon,
            csfr::BIV => self.biv,
            csfr::BTV => self.btv,
            csfr::ISP => self.isp,
            csfr::ICR => self.icr,
            csfr::FCX => self.fcx,
            csfr::LCX => self.lcx,
            _ => csfr_index(off).map_or(0, |i| self.csfrs[i]),
        }
    }

    /// MTCR semantics for CSFR offset `off` (V2 MTCR p. 3-219, V1 p. 3-1/3-18). Returns `false` when
    /// the write has no effect (undefined location, read-only register, ENDINIT protection).
    /// PC (FE08H) is not stored here: the caller turns it into a control transfer.
    pub fn csfr_write(&mut self, off: u16, v: u32) -> bool {
        match off {
            csfr::PSW => {
                self.psw = v;
                return true;
            }
            csfr::ICR => {
                self.icr = (self.icr & icr::PIPN) | (v & !icr::PIPN);
                return true;
            }
            _ => {}
        }
        let named_endinit = matches!(off, csfr::BIV | csfr::BTV | csfr::ISP);
        let def = csfr_index(off).map(|i| (i, CSFR_TABLE[i]));
        if (named_endinit || def.is_some_and(|(_, d)| d.endinit)) && self.bus.endinit() {
            return false;
        }
        match off {
            csfr::PCXI => self.pcxi = v,
            csfr::PC => return false,
            csfr::SYSCON => self.syscon = v,
            csfr::BIV => self.biv = v & !1,
            csfr::BTV => self.btv = v & !1,
            csfr::ISP => self.isp = v,
            csfr::FCX => self.fcx = v,
            csfr::LCX => self.lcx = v,
            _ => match def {
                Some((i, d)) if d.write_mask != 0 => {
                    self.csfrs[i] = (self.csfrs[i] & !d.write_mask) | (v & d.write_mask);
                    if off == 0xA000 && v & 2 != 0 {
                        // FPU_TRAP_CON.TCL: "Clears the trapped instruction (TST will be negated)",
                        // reads 0 (V1 p. 11-12)
                        self.csfrs[i] &= !1;
                    }
                    if off == csfr::PCON1 {
                        // PCON1 is implementation specific (V1 p. 8-7). TC1.6 PMI: PCINV (bit 0)
                        // and PBINV (bit 1) start a program cache / buffer invalidation and read 1
                        // until it completes; firmware polls for 0 (SIMOS 0xC00029DA). There is no
                        // cache model, so the invalidation completes immediately.
                        self.csfrs[i] &= !3;
                    }
                    if off == 0x801C {
                        self.update_periph_mask();
                    }
                }
                _ => return false,
            },
        }
        true
    }

    /// Set an implementation-specific CSFR value (reset values, read-only identification).
    pub fn csfr_set_reset(&mut self, off: u16, v: u32) {
        if let Some(i) = csfr_index(off) {
            self.csfrs[i] = v;
            if off == 0x801C {
                self.update_periph_mask();
            }
        }
    }

    #[inline]
    pub(crate) fn update_periph_mask(&mut self) {
        self.periph_mask = match self.arch {
            Arch::Tc13 | Arch::Tc131 => 0xC000,
            Arch::Tc16 => {
                let pma = self.csfrs[crate::groups::ldst::PMA0];
                ((pma >> 16) as u16 & !(pma as u16)) | 0x8000
            }
        };
    }

    /// Publish the ICU arbitration result in ICR.PIPN (V1 p. 5-14; read-only for software).
    pub fn set_pipn(&mut self, pipn: u8) {
        self.icr = (self.icr & !icr::PIPN) | ((pipn as u32) << icr::PIPN_SHIFT);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::NullBus;

    fn cpu() -> Cpu<NullBus> {
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x1_0000], true);
        m.map(0xD000_0000, r, 0, 0x1_0000);
        Cpu::new(Arch::Tc16, m, NullBus)
    }

    #[test]
    fn invalid_encoding_traps_iopc_with_exact_count() {
        // RR 0BH op2 FFH is undefined (V2 Table 3-1): IOPC trap (class 2) at BTV | 2 << 5.
        let mut c = cpu();
        c.write_bytes(0xD000_0000, &0x0FF0_000Bu32.to_le_bytes()).unwrap();
        c.btv = 0xD000_0100;
        c.fcx = 0x000D_0010; // CSA at 0xD000_0400, link word 0 -> FCX becomes 0
        c.pc = 0xD000_0000;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!(c.icount, 1);
        assert_eq!(c.pc, 0xD000_0140);
        assert_eq!(c.a[11], 0xD000_0000, "synchronous trap returns to the faulting instruction");
        assert_eq!(c.d[15], 1, "TIN");
        assert_eq!(c.run(3), Exit::Budget);
        assert_eq!(c.icount, 4, "budget is exact");
    }

    /// `Bus::hook` splits blocks and stops before the hooked instruction once per arrival;
    /// `Bus::clock` sees the exact count of the accessing instruction mid-block.
    #[test]
    fn hooks_and_exact_access_clock() {
        #[derive(Default)]
        struct B {
            clocks: Vec<(u64, u32)>,
        }
        impl Bus for B {
            fn read(&mut self, _: u32, _: u8) -> Result<u64, crate::BusError> {
                Ok(7)
            }
            fn write(&mut self, _: u32, _: u8, _: u64) -> Result<(), crate::BusError> {
                Ok(())
            }
            fn clock(&mut self, icount: u64, block_pc: u32) {
                self.clocks.push((icount, block_pc));
            }
            fn hook(&mut self, pc: u32) -> bool {
                pc == 0xD000_0002
            }
        }
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x1_0000], true);
        m.map(0xD000_0000, r, 0, 0x1_0000);
        let mut c = Cpu::new(Arch::Tc16, m, B::default());
        // nop; nop; ld.w d0, [a2]0 (peripheral); nop; nop
        c.write_bytes(0xD000_0000, &[0, 0, 0, 0, 0x19, 0x20, 0, 0, 0, 0, 0, 0]).unwrap();
        c.a[2] = 0xF000_0000;
        c.pc = 0xD000_0000;
        assert_eq!(c.run(100), Exit::Hook { pc: 0xD000_0002 });
        assert_eq!((c.icount, c.pc), (1, 0xD000_0002), "block ended before the hooked address");
        assert_eq!(c.run(3), Exit::Budget, "resumes without reporting the same arrival again");
        assert_eq!((c.icount, c.d[0]), (4, 7));
        assert_eq!(c.bus.clocks, [(2, 0xD000_0002)], "access by the 3rd instruction, in the hooked block");
        c.pc = 0xD000_0002;
        assert_eq!(c.run(1), Exit::Hook { pc: 0xD000_0002 }, "next arrival reported");
    }

    #[test]
    fn ji_uses_the_computed_address_register_target() {
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x1000], false);
        m.map(0x8000_0000, r, 0, 0x1000);
        let r = m.add_owned(vec![0; 0x2_0000], true);
        m.map(0xD000_0000, r, 0, 0x2_0000);
        let mut c = Cpu::new(Arch::Tc16, m, NullBus);
        c.write_bytes(0x8000_0000, &[0x91, 0x20, 0x00, 0xFD, 0xD9, 0xFF, 0xC0, 0x6F, 0xDC, 0x0F]).unwrap();
        c.pc = 0x8000_0000;
        assert_eq!(c.run(3), Exit::Budget);
        assert_eq!(c.a[15], 0xD001_FD80);
        assert_eq!(c.pc, 0xD001_FD80);
    }

    #[test]
    fn fsr_mask_sequence_with_zero_registers() {
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x1_0000], false);
        m.map(0x8000_0000, r, 0, 0x1_0000);
        struct Zero;
        impl Bus for Zero {
            fn read(&mut self, _: u32, _: u8) -> Result<u64, crate::BusError> { Ok(0) }
            fn write(&mut self, _: u32, _: u8, _: u64) -> Result<(), crate::BusError> { Ok(()) }
        }
        let mut c = Cpu::new(Arch::Tc16, m, Zero);
        let code = [
            0x91, 0x00, 0x80, 0xFF, 0x19, 0xF0, 0x10, 0x02,
            0xBB, 0xD0, 0x6E, 0x10, 0x8F, 0x00, 0x1F, 0x00,
            0x26, 0x10, 0x19, 0xF1, 0x10, 0x04, 0x7B, 0xD0,
            0x6E, 0x20, 0x26, 0x21, 0x02, 0x4F, 0xA6, 0x10,
        ];
        c.write_bytes(0x8000_9E54, &code).unwrap();
        c.pc = 0x8000_9E54;
        assert_eq!(c.run(7), Exit::Budget);
        assert_eq!(c.d[0], 0, "D0={:#010X} D1={:#010X} D2={:#010X} PC={:#010X}", c.d[0], c.d[1], c.d[2], c.pc);
        let ram = c.mem.add_owned(vec![0; 0x2_0000], true);
        c.mem.map(0xD000_0000, ram, 0, 0x2_0000);
        c.write_bytes(0xD001_F594, &0x8000_0000u32.to_le_bytes()).unwrap();
        let code = [
            0x91, 0x20, 0x00, 0xFD, 0x19, 0xF1, 0x54, 0x6F, 0x91, 0x00, 0x80, 0xFF,
            0xD9, 0xFF, 0x20, 0x02, 0x54, 0xFF, 0x7E, 0x14, 0xB7, 0x10, 0x01, 0x06,
            0x3C, 0x05, 0x54, 0xFF, 0x6E, 0x03, 0xB7, 0x10, 0x81, 0x07, 0x91, 0x00,
            0x80, 0xFF, 0x39, 0xFF, 0x16, 0x02, 0xD9, 0xFF, 0x20, 0x04, 0x16, 0x01,
            0x8B, 0x0F, 0x20, 0xF2, 0x8F, 0xBF, 0x00, 0xF0, 0xA6, 0xF0, 0x54, 0xFF,
            0x7E, 0x14, 0xB7, 0x10, 0x01, 0x0E, 0x3C, 0x05, 0x54, 0xFF, 0x6E, 0x03,
            0xB7, 0x10, 0x81, 0x07,
        ];
        c.write_bytes(0x8000_9E84, &code).unwrap();
        c.pc = 0x8000_9E84;
        assert_eq!(c.run(19), Exit::Budget);
        assert_eq!((c.pc, c.d[0]), (0x8000_9ED0, 0), "D0={:#010X} D1={:#010X}", c.d[0], c.d[1]);
    }

    #[test]
    fn fetch_fault_without_trap_vector_stops() {
        let mut c = cpu();
        c.pc = 0x1234_5670;
        c.btv = 0x5000_0000;
        // PSE cannot save a context (FCX = 0) -> FCU, vector BTV | 3 << 5, which is unmapped too
        assert_eq!(c.run(10), Exit::FetchFault { pc: 0x5000_0060 });
        assert_eq!(c.icount, 0);
    }

    #[test]
    fn loads_stores_and_bus_errors() {
        let mut c = cpu();
        c.st32(0xD000_0010, 0x1122_3344).unwrap();
        assert_eq!(c.ld16(0xD000_0012).unwrap(), 0x1122);
        assert_eq!(c.ld64(0xD000_0010).unwrap() & 0xFFFF_FFFF, 0x1122_3344);
        assert_eq!(c.ld32(0xF000_0000), Err(Trap::DSE));
        assert_eq!(c.st32(0xF000_0000, 1), Err(Trap::DAE));
        assert_eq!(c.ld32(0xD000_FFFE), Err(Trap::DSE), "crosses the end of the mapping");
    }

    #[test]
    #[cfg(feature = "trace-data-stores")]
    fn memory_observers_do_not_change_memory() {
        #[derive(Default)]
        struct Observer(Vec<(u32, u32, u8, u64)>, Vec<(u32, u32, u8, u64)>);
        impl Bus for Observer {
            fn read(&mut self, _: u32, _: u8) -> Result<u64, crate::BusError> { Err(crate::BusError) }
            fn write(&mut self, _: u32, _: u8, _: u64) -> Result<(), crate::BusError> { Err(crate::BusError) }
            fn data_read(&mut self, pc: u32, addr: u32, size: u8, value: u64) {
                self.1.push((pc, addr, size, value));
            }
            fn data_write(&mut self, pc: u32, addr: u32, size: u8, value: u64) {
                self.0.push((pc, addr, size, value));
            }
        }
        let mut m = Memory::new();
        let ram = m.add_owned(vec![0; 0x1000], true);
        m.map(0xD000_0000, ram, 0, 0x1000);
        let mut c = Cpu::new(Arch::Tc16, m, Observer::default());
        c.exec_pc = 0x8000_0042;
        c.st32(0xD000_0010, 0x1122_3344).unwrap();
        assert_eq!(c.bus.0, [(0x8000_0042, 0xD000_0010, 4, 0x1122_3344)]);
        assert_eq!(c.ld32(0xD000_0010).unwrap(), 0x1122_3344);
        assert_eq!(c.bus.1, [(0x8000_0042, 0xD000_0010, 4, 0x1122_3344)]);
    }

    #[test]
    fn csfr_endinit_and_undefined() {
        struct Locked;
        impl Bus for Locked {
            fn read(&mut self, _: u32, _: u8) -> Result<u64, crate::BusError> {
                Err(crate::BusError)
            }
            fn write(&mut self, _: u32, _: u8, _: u64) -> Result<(), crate::BusError> {
                Err(crate::BusError)
            }
            fn endinit(&self) -> bool {
                true
            }
        }
        let mut c = Cpu::new(Arch::Tc16, Memory::new(), Locked);
        assert!(!c.csfr_write(csfr::BTV, 0x8000_0100), "BTV is ENDINIT protected");
        assert!(c.csfr_write(csfr::ICR, 0x00FF_0105));
        assert_eq!(c.icr, 0x105, "PIPN is read-only");
        assert!(!c.csfr_write(0xFF00, 1), "GPR offsets are not CSFRs");
        assert_eq!(c.csfr_read(0x1234, 0), 0);
        assert!(c.csfr_write(0x9204, 7)); // PCON1
        assert_eq!(c.csfr_read(0x9204, 0), 4, "PCINV/PBINV complete immediately");
    }

    /// V1 §11.6 p. 11-11..11-14: FPU_TRAP_CON TST/RM/FI..FX are `rh`, FIE..FXE `rw`, TCL (bit 1) is
    /// write-only ("1: clears the trapped instruction (TST will be negated)", reads 0); FPU_TRAP_PC,
    /// FPU_TRAP_OPC and FPU_TRAP_SRC1..3 are `rh` (not writable by MTCR).
    #[test]
    fn fpu_trap_csfrs_are_hardware_owned() {
        let mut c = cpu();
        // a captured FI trap: TST, captured FI, RM = 01B, FIE enabled
        c.csfr_set_reset(0xA000, 1 << 30 | 1 << 22 | 1 << 8 | 1);
        c.csfr_set_reset(0xA004, 0xC000_0010);
        c.csfr_write(0xA000, 1 << 21 | 2); // FVE only, TCL
        assert_eq!(c.csfr_read(0xA000, 0), 1 << 30 | 1 << 21 | 1 << 8, "TST cleared, TCL reads 0, rh kept");
        c.csfr_write(0xA000, 1 | 1 << 29);
        assert_eq!(c.csfr_read(0xA000, 0) & (1 | 1 << 29), 0, "TST/FV are not software writable");
        for off in [0xA004u16, 0xA008, 0xA010, 0xA014, 0xA018] {
            let before = c.csfr_read(off, 0);
            c.csfr_write(off, 0x1234_5678);
            assert_eq!(c.csfr_read(off, 0), before, "{off:#x} is rh");
        }
    }

    #[test]
    fn odd_pc_biv_btv_are_masked_and_do_not_corrupt_sentinels() {
        #[derive(Default)]
        struct HookZero;
        impl Bus for HookZero {
            fn read(&mut self, _: u32, _: u8) -> Result<u64, crate::BusError> {
                Err(crate::BusError)
            }
            fn write(&mut self, _: u32, _: u8, _: u64) -> Result<(), crate::BusError> {
                Err(crate::BusError)
            }
            fn hook(&mut self, pc: u32) -> bool {
                pc == 0
            }
        }
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x1000], true);
        m.map(0x0000_0000, r, 0, 0x1000);
        let mut c = Cpu::new(Arch::Tc16, m, HookZero);
        // mov d0, #5 (0x82, 0x50) at address 0; set odd pc = 1.
        c.write_bytes(0, &[0x82, 0x50, 0x00, 0x00]).unwrap();
        c.pc = 1;
        assert_eq!(c.run(2), Exit::Hook { pc: 0 }, "odd PC=1 masked to 0 and triggers hook at 0");
        assert_eq!(c.run(2), Exit::Budget);
        assert_eq!((c.d[0], c.pc & 1), (5, 0));

        // Direct writes of odd BTV / BIV on public fields must still enter even trap/IRQ vectors.
        c.btv = 0x0000_0201;
        c.enter_trap(Trap::FCU, 0);
        assert_eq!(c.pc, 0x0000_0200 | (Trap::FCU.class as u32) << 5);
    }

    #[test]
    fn invalidate_code_subpage_and_cross_region_write_bytes() {
        let mut m = Memory::new();
        // Interior sub-page mapping at 0xD000_4000..0xD000_8000 (neither 0xD000_0000 nor 0xD000_FFFF is mapped).
        let sub = m.add_owned(vec![0; 0x4000], true);
        m.map(0xD000_4000, sub, 0, 0x4000);
        // Writable RAM adjacent to read-only Flash across 0x8FFF_FFF0..0x9000_0100, plus Flash alias at 0xB000_0000.
        let ram_tail = m.add_owned(vec![0; 0x10], true);
        let flash = m.add_owned(vec![0; 0x100], false);
        m.map(0x8FFF_FFF0, ram_tail, 0, 0x10);
        m.map(0x9000_0000, flash, 0, 0x100);
        m.map(0xB000_0000, flash, 0, 0x100);

        let mut c = Cpu::new(Arch::Tc16, m, NullBus);
        // 1) Cache `mov d0, #1` (0x82, 0x10) in interior sub-page, mutate via raw pointer, invalidate full 64 KB page.
        c.write_bytes(0xD000_4000, &[0x82, 0x10]).unwrap();
        c.pc = 0xD000_4000;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!(c.d[0], 1);
        let p = c.mem.host_w_plain(0xD000_4000, 2);
        assert!(p.is_none(), "F_CODE is set");
        // Mutate behind `Cpu` via `host_r` (without `host_w` consuming `take_code`), relying solely on `invalidate_code`.
        let ptr = c.mem.host_r(0xD000_4000, 2).unwrap() as *mut [u8; 2];
        unsafe { *ptr = [0x82, 0x70] }; // mov d0, #7
        c.invalidate_code(0xD000_4000, 2);
        c.pc = 0xD000_4000;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!(c.d[0], 7, "interior sub-page block invalidated");

        // 2) Cache blocks in RAM tail (0x8FFF_FFFE), Flash (0x9000_0000), and Flash alias (0xB000_0000).
        c.write_bytes(0x8FFF_FFFE, &[0x82, 0x20, 0x82, 0x30]).unwrap(); // mov d0,#2; mov d0,#3
        c.pc = 0x8FFF_FFFE;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!(c.d[0], 2);
        c.pc = 0x9000_0000;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!(c.d[0], 3);
        c.pc = 0xB000_0000;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!(c.d[0], 3);

        // Cross-region write_bytes starting in writable RAM and extending into read-only Flash.
        c.write_bytes(0x8FFF_FFFE, &[0x82, 0x40, 0x82, 0x60]).unwrap(); // mov d0,#4; mov d0,#6
        c.pc = 0x8FFF_FFFE;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!(c.d[0], 4);
        c.pc = 0x9000_0000;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!(c.d[0], 6, "read-only tail invalidated across writable->RO boundary");
        c.pc = 0xB000_0000;
        assert_eq!(c.run(1), Exit::Budget);
        assert_eq!(c.d[0], 6, "read-only alias invalidated too");
    }

    #[test]
    fn invalidate_code_skips_data_granules_and_drops_borrowed_dma_code() {
        let mut dspr = vec![0u8; 0x1000].into_boxed_slice();
        // Short 4-byte RAM block (`mov d0, #1; ji a11`) fitting inside granule 0 (`0x00..0x40`).
        dspr[0..4].copy_from_slice(&[0x82, 0x10, 0xDC, 0x0B]);
        let dspr_len = dspr.len();
        let dspr_ptr = dspr.as_mut_ptr();
        let mut m = Memory::new();
        // SAFETY: `dspr` is heap-pinned and outlives `c` in this test.
        let ram = unsafe { m.add_raw(dspr_ptr, dspr_len, true) };
        m.map(0xD000_0000, ram, 0, dspr_len);
        let flash = m.add_owned(vec![0x82, 0x71, 0xDC, 0x0B], false); // mov d1, #7; ji a11
        m.map(0x8000_0000, flash, 0, 4);

        let mut c = Cpu::new(Arch::Tc16, m, NullBus);
        c.a[11] = 0x8000_0000;
        c.pc = 0xD000_0000;
        assert_eq!(c.run(3), Exit::Budget);
        assert_eq!((c.d[0], c.d[1]), (1, 7));
        assert_eq!(c.cached_blocks(), 2);

        // Simulated DMA write to a non-code buffer (`0xD000_0100`) on the same `F_CODE` 64 KB page
        // must skip `BlockCache::invalidate` in O(1) and preserve all cached blocks.
        unsafe { std::ptr::copy_nonoverlapping([0xDEu8, 0xAD].as_ptr(), dspr_ptr.add(0x100), 2) };
        c.invalidate_code(0xD000_0100, 2);
        assert_eq!(c.cached_blocks(), 2, "data-granule DMA must not drop cached blocks");

        // Simulated DMA overwriting the executed RAM block (`0xD000_0000`) must drop that block.
        unsafe { std::ptr::copy_nonoverlapping([0x82u8, 0x50].as_ptr(), dspr_ptr, 2) }; // mov d0, #5
        c.invalidate_code(0xD000_0000, 2);
        assert_eq!(c.cached_blocks(), 1, "code-granule DMA must invalidate the RAM block");
        c.a[11] = 0x8000_0000;
        c.pc = 0xD000_0000;
        assert_eq!(c.run(2), Exit::Budget);
        assert_eq!(c.d[0], 5);
        drop(c);
        drop(dspr);
    }

    #[test]
    fn cpu_stack_footprint_stays_small() {
        // Large tables (Memory::pages, BlockCache arenas) must remain heap-allocated so `Cpu` fits
        // comfortably on default 2 MiB Windows worker stacks.
        assert!(std::mem::size_of::<Memory>() <= 64, "Memory is {} B", std::mem::size_of::<Memory>());
        assert!(std::mem::size_of::<BlockCache>() <= 256, "BlockCache is {} B", std::mem::size_of::<BlockCache>());
        assert!(std::mem::size_of::<Cpu<NullBus>>() <= 1536, "Cpu<NullBus> is {} B", std::mem::size_of::<Cpu<NullBus>>());
    }
}
