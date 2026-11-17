//! Bare-Unicorn oracle: run instruction bytes on Unicorn 2.1.x (QEMU TCG) with the same memory
//! layout as [`super::cpu_with`] and return the resulting state.
//!
//! Unicorn is a *secondary* reference: the manual is authoritative. Encodings that Unicorn
//! mistranslates are listed in [`KNOWN_WRONG`] and recognised by [`trustworthy`], which reuses the
//! decoder of the board's translator-fix layer (`simemu_tc1791::isa_fix::decode`, documented and
//! proven by unit tests in `crates/simemu-tc1791/src/isa_fix.rs`). Encodings Unicorn does not
//! implement at all come back as `Err(UC_ERR_INSN_INVALID)` / `UC_ERR_EXCEPTION`.

use super::{Setup, CODE, CODE_SIZE, DATA, DATA_SIZE, TRAP_BASE};
use std::cell::{Cell, RefCell};
use std::ffi::c_int;
use std::sync::{RwLock, RwLockReadGuard, RwLockWriteGuard};
use unicorn_tricore_sys::*;

/// Synchronises Unicorn engine lifecycle (`uc_open` / `UC_INIT` / `uc_close`) against concurrent
/// `uc_emu_start` / cache-invalidation calls across threads.
///
/// Stock `libunicorn` 2.1.x on Windows (`qemu/accel/tcg/translate-all.c`) has two `code_gen_buffer`
/// defects that surface under multithreaded differential tests:
///
/// 1. **VEH closure use-after-free race (`EXCEPTION_STACK_OVERFLOW` + `0xc0000005`)**: each engine
///    reserves a 1 GiB uncommitted `code_gen_buffer` (`MEM_RESERVE`) and prepends an RWX trampoline
///    (`uc->seh_closure`) to the process-wide Vectored Exception Handler chain via
///    `AddVectoredExceptionHandler(1, closure)`. When any thread translates into an uncommitted
///    `code_gen_buffer` chunk, `ntdll!RtlpCallVectoredHandlers` walks the global chain and invokes
///    every registered `seh_closure` *after* releasing the list SRWLock. `uc_close` ->
///    `free_code_gen_buffer` calls `RemoveVectoredExceptionHandler` (which unlinks without waiting
///    for in-flight callers) and immediately `VirtualFree`s `uc->seh_closure` and frees `uc`. An
///    in-flight VEH dispatch on another thread then faults inside the freed closure / `uc_struct`,
///    re-entering `KiUserExceptionDispatcher` -> VEH recursively until the victim thread hits its
///    2 MiB stack guard page (`"has overflowed its stack"`), followed by `0xc0000005`.
///
/// 2. **Unclamped 4 MiB commit overrun at the end of `code_gen_buffer` (`0xc0000005`)**:
///    `code_gen_buffer_handler` commits pages via `VirtualAlloc(fault_addr, 0x400000, MEM_COMMIT,
///    PAGE_EXECUTE_READWRITE)` without clamping `0x400000` to `code_gen_buffer + size - fault_addr`.
///    Any non-page-aligned fault shifts the committed horizon by +4 KiB, so the final chunk fault
///    at `end - 0x3FF000` spans 4 KiB past the `MEM_RESERVE` region, `VirtualAlloc` returns `NULL`
///    (`ERROR_INVALID_ADDRESS`), and the handler returns `EXCEPTION_CONTINUE_SEARCH` before QEMU's
///    `code_gen_highwater` can trigger `tb_flush`.
///
/// Holding a shared read guard during `uc_emu_start` / cache flushes and an exclusive write guard
/// during engine creation (`uc_open` + initial `uc_mem_map*` / `UC_INIT`, which commits the first
/// 4 MiB chunk) and `uc_close` eliminates (1), while flushing the TB region every
/// [`TB_FLUSH_PERIOD`] runs in [`Engine::emu_start`] keeps `code_gen_ptr` well inside the initial
/// 4 MiB chunk, eliminating both additional VEH commit faults and (2).
static UNICORN_VEH_LOCK: RwLock<()> = RwLock::new(());

/// Reset the engine's TCG region (`UC_CTL_TB_FLUSH`) every 64 `emu_start` calls (~1.2 MiB worst-case
/// `count = 1` TB growth < 4 MiB initial committed chunk).
const TB_FLUSH_PERIOD: u32 = 64;

fn veh_read() -> RwLockReadGuard<'static, ()> {
    UNICORN_VEH_LOCK.read().unwrap_or_else(|e| e.into_inner())
}

fn veh_write() -> RwLockWriteGuard<'static, ()> {
    UNICORN_VEH_LOCK.write().unwrap_or_else(|e| e.into_inner())
}

/// Unicorn 2.1.4 TriCore defects (see `simemu-tc1791/src/isa_fix.rs` header). Results of these
/// encodings must not be used as oracle.
pub const KNOWN_WRONG: &[&str] = &[
    "RCRW IMASK (0xD7 op2 1): position and destination register fields swapped",
    "RCRW INSERT (0xD7 op2 0): position and destination register fields swapped",
    "RRRR DEXTR (0x17 op2 4) with a != b and pos = 0: shift by 32 gives D[a] | D[b]",
    "RRPW DEXTR (0x77) with pos = 0 and a != b: same shift-by-32 defect",
    "BISR (RC 0xAD op2 0, SC 0xE0): ICR |= const9 instead of ICR.CCPN = const9",
    "DISABLE D[a] (SYS 0x0D op2 0x0F): not decoded (illegal opcode trap)",
    "MFCR of a CSFR Unicorn does not implement (PCON0..2, DCON0/2, ...): destination unchanged",
    "MTCR to a CSFR Unicorn does not implement: write dropped",
    "MTCR to ENDINIT-protected CSFRs (BIV, BTV, ISP, COMPAT, BMACON, SMACON, MIECON): always written",
    "BO LD.BU pre-increment (0x09 op2 0x11): loaded byte sign-extended",
    "MFCR/MTCR ICR (CSFR 0xFE2C) after ENABLE/DISABLE/RESTORE in the same TB: stale cached ICR; MTCR ICR writes the read-only PIPN",
    "SRO LD.H D15, [A[b]]off4 (0x8C) with off4 != 0: EA = A[b] + off4 instead of A[b] + 2 * off4",
    "RR FTOU/FTOQ31/FTOQ31Z/Q31TOF (0x4B n=1 op2 0x12/0x11/0x18/0x15): not decoded (illegal opcode trap)",
];

/// Whether Unicorn's result for this encoding may be used as a reference.
pub fn trustworthy(lo: u16, hi: u16) -> bool {
    simemu_tc1791::isa_fix::decode(lo, hi).is_none()
}

/// State after running on Unicorn.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Out {
    pub d: [u32; 16],
    pub a: [u32; 16],
    pub psw: u32,
    pub pcxi: u32,
    pub icr: u32,
    pub fcx: u32,
    pub lcx: u32,
    pub pc: u32,
    /// The DATA region after execution.
    pub data: Vec<u8>,
}

fn rd(uc: *mut uc_engine, r: TriCoreReg) -> u32 {
    let mut v = 0u32;
    // SAFETY: valid engine, register id from the enum.
    unsafe { uc_reg_read(uc, r as c_int, (&mut v as *mut u32).cast()) };
    v
}

fn wr(uc: *mut uc_engine, r: TriCoreReg, v: u32) {
    // SAFETY: as above.
    unsafe { uc_reg_write(uc, r as c_int, (&v as *const u32).cast()) };
}

/// Unicorn 2.1.4 keeps PSW.C/V/SV/AV/SAV in the separate PSW_USB_* cache (flag = bit 31, C = any
/// non-zero value) and `uc_reg_read/write(PSW)` access only the raw word. Write the cache like
/// QEMU's `psw_write` (target/tricore/helper.c) ...
fn wr_psw(uc: *mut uc_engine, v: u32) {
    wr(uc, TriCoreReg::PSW, v);
    wr(uc, TriCoreReg::PSW_USB_C, v & 1 << 31);
    wr(uc, TriCoreReg::PSW_USB_V, (v & 1 << 30) << 1);
    wr(uc, TriCoreReg::PSW_USB_SV, (v & 1 << 29) << 2);
    wr(uc, TriCoreReg::PSW_USB_AV, (v & 1 << 28) << 3);
    wr(uc, TriCoreReg::PSW_USB_SAV, (v & 1 << 27) << 4);
}

/// ... and fold it like `psw_read`, except that raw bit 24 (PSW.RM[0]) is kept (`psw_read` masks
/// with 0x06FFFFFF and drops it — a Unicorn defect of its own, see `tests/board_replay.rs` U1).
fn rd_psw(uc: *mut uc_engine) -> u32 {
    (rd(uc, TriCoreReg::PSW) & 0x07FF_FFFF)
        | ((rd(uc, TriCoreReg::PSW_USB_C) != 0) as u32) << 31
        | (rd(uc, TriCoreReg::PSW_USB_V) >> 31) << 30
        | (rd(uc, TriCoreReg::PSW_USB_SV) >> 31) << 29
        | (rd(uc, TriCoreReg::PSW_USB_AV) >> 31) << 28
        | (rd(uc, TriCoreReg::PSW_USB_SAV) >> 31) << 27
}

/// Thread-bound Unicorn engine (`CODE` and `DATA` mapped) whose lifecycle and execution are
/// synchronised with [`UNICORN_VEH_LOCK`] and whose TCG region is reset every [`TB_FLUSH_PERIOD`]
/// runs so `code_gen_ptr` stays inside the initial 4 MiB committed chunk.
pub struct Engine {
    uc: *mut uc_engine,
    emu_calls: Cell<u32>,
}

impl Engine {
    /// Open a TriCore engine with Unicorn-allocated `CODE` and `DATA` regions mapped.
    pub fn new_mapped(model: Option<c_int>) -> Result<Self, uc_err> {
        let _veh = veh_write();
        // SAFETY: `uc_open` initialises `uc` on `UC_ERR_OK`; on any mapping failure `uc_close`
        // tears it down before returning, all under the exclusive VEH lock.
        unsafe {
            let mut uc: *mut uc_engine = std::ptr::null_mut();
            let e = uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc);
            if e != UC_ERR_OK {
                return Err(e);
            }
            if let Some(m) = model {
                uc_ctl_set_cpu_model(uc, m);
            }
            for (base, size) in [(CODE, CODE_SIZE), (DATA, DATA_SIZE)] {
                let e = uc_mem_map(uc, base as u64, size as usize, UC_PROT_ALL as u32);
                if e != UC_ERR_OK {
                    uc_close(uc);
                    return Err(e);
                }
            }
            Ok(Self { uc, emu_calls: Cell::new(0) })
        }
    }

    /// Open a TriCore engine backed by caller-owned `CODE` (`CODE_SIZE`) and `DATA` (`DATA_SIZE`)
    /// buffers.
    ///
    /// # Safety
    /// `code..code + CODE_SIZE` and `data..data + DATA_SIZE` must remain valid and unmoved until
    /// this `Engine` is dropped (`uc_close`).
    unsafe fn new_ptr(model: Option<c_int>, code: *mut u8, data: *mut u8) -> Result<Self, uc_err> {
        let _veh = veh_write();
        // SAFETY: `uc_open` initialises `uc` on `UC_ERR_OK`; caller guarantees buffer lifetimes.
        unsafe {
            let mut uc: *mut uc_engine = std::ptr::null_mut();
            let e = uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc);
            if e != UC_ERR_OK {
                return Err(e);
            }
            if let Some(m) = model {
                uc_ctl_set_cpu_model(uc, m);
            }
            for (base, size, ptr) in [(CODE, CODE_SIZE, code), (DATA, DATA_SIZE, data)] {
                let e = uc_mem_map_ptr(uc, base as u64, size as usize, UC_PROT_ALL as u32, ptr.cast());
                if e != UC_ERR_OK {
                    uc_close(uc);
                    return Err(e);
                }
            }
            Ok(Self { uc, emu_calls: Cell::new(0) })
        }
    }

    #[inline]
    pub fn rd(&self, r: TriCoreReg) -> u32 {
        rd(self.uc, r)
    }

    #[inline]
    pub fn wr(&self, r: TriCoreReg, v: u32) {
        wr(self.uc, r, v);
    }

    #[inline]
    pub fn wr_psw(&self, v: u32) {
        wr_psw(self.uc, v);
    }

    #[inline]
    pub fn rd_psw(&self) -> u32 {
        rd_psw(self.uc)
    }

    #[inline]
    pub fn mem_write(&self, addr: u64, bytes: &[u8]) -> uc_err {
        if bytes.is_empty() {
            return UC_ERR_OK;
        }
        // SAFETY: `self.uc` is a live engine; `bytes` is valid for `bytes.len()` reads.
        unsafe { uc_mem_write(self.uc, addr, bytes.as_ptr().cast(), bytes.len()) }
    }

    #[inline]
    pub fn remove_cache(&self, begin: u64, end: u64) -> uc_err {
        let _veh = veh_read();
        // SAFETY: `self.uc` is a live engine, called outside emulation callbacks.
        unsafe { uc_ctl_remove_cache(self.uc, begin, end) }
    }

    #[inline]
    pub fn emu_start(&self, begin: u64, until: u64, timeout: u64, count: usize) -> uc_err {
        let _veh = veh_read();
        let n = self.emu_calls.get().wrapping_add(1);
        if n >= TB_FLUSH_PERIOD {
            self.emu_calls.set(0);
            // SAFETY: `self.uc` is live, outside emulation callbacks, under `_veh`.
            unsafe {
                let _ = uc_ctl_flush_tb(self.uc);
            }
        } else {
            self.emu_calls.set(n);
        }
        // SAFETY: `self.uc` is a live engine owned by this thread.
        unsafe { uc_emu_start(self.uc, begin, until, timeout, count) }
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        let _veh = veh_write();
        // SAFETY: `self.uc` was opened in `new_*` and is closed exactly once under the VEH write lock.
        unsafe {
            uc_close(self.uc);
        }
    }
}

/// Reusable per-thread engine for [`run_model`] so differential suites do not churn tens of
/// thousands of 1 GiB `code_gen_buffer` reservations and Windows VEH registrations.
struct CachedEngine {
    /// Declared first so `Drop` runs `uc_close` *before* freeing `code_buf` and `data_buf`.
    eng: Engine,
    code_buf: Box<[u8]>,
    data_buf: Box<[u8]>,
    init_biv: u32,
    init_isp: u32,
    init_syscon: u32,
    init_compat: u32,
}

impl CachedEngine {
    fn new(model: Option<c_int>) -> Result<Self, uc_err> {
        let mut code_buf = vec![0u8; CODE_SIZE as usize].into_boxed_slice();
        let mut data_buf = vec![0u8; DATA_SIZE as usize].into_boxed_slice();
        // SAFETY: `code_buf` and `data_buf` are heap-allocated boxed slices whose addresses do not
        // move and which drop after `eng` (field declaration order).
        let eng = unsafe { Engine::new_ptr(model, code_buf.as_mut_ptr(), data_buf.as_mut_ptr()) }?;
        let init_biv = eng.rd(TriCoreReg::BIV);
        let init_isp = eng.rd(TriCoreReg::ISP);
        let init_syscon = eng.rd(TriCoreReg::SYSCON);
        let init_compat = eng.rd(TriCoreReg::COMPAT);
        Ok(Self { eng, code_buf, data_buf, init_biv, init_isp, init_syscon, init_compat })
    }

    fn run(&mut self, code: &[u8], n: usize, s: &Setup) -> Result<Out, uc_err> {
        self.code_buf.fill(0);
        self.data_buf.fill(0);
        self.eng.mem_write(CODE as u64, code);
        for (addr, b) in &s.mem {
            self.eng.mem_write(*addr as u64, b);
        }
        self.eng.remove_cache(CODE as u64, (CODE + CODE_SIZE) as u64);
        let start = if s.pc == 0 { CODE } else { s.pc };
        if (DATA..DATA + DATA_SIZE).contains(&start) {
            self.eng.remove_cache(DATA as u64, (DATA + DATA_SIZE) as u64);
        }
        for i in 0..16 {
            self.eng.wr(TriCoreReg::d(i), s.d[i as usize]);
            self.eng.wr(TriCoreReg::a(i), s.a[i as usize]);
        }
        self.eng.wr_psw(s.psw);
        self.eng.wr(TriCoreReg::PCXI, s.pcxi);
        self.eng.wr(TriCoreReg::ICR, s.icr);
        self.eng.wr(TriCoreReg::FCX, s.fcx);
        self.eng.wr(TriCoreReg::LCX, s.lcx);
        self.eng.wr(TriCoreReg::BTV, TRAP_BASE);
        self.eng.wr(TriCoreReg::BIV, self.init_biv);
        self.eng.wr(TriCoreReg::ISP, self.init_isp);
        self.eng.wr(TriCoreReg::SYSCON, self.init_syscon);
        self.eng.wr(TriCoreReg::COMPAT, self.init_compat);
        self.eng.wr(TriCoreReg::PC, start);
        let e = self.eng.emu_start(start as u64, 0, 0, n);
        if e != UC_ERR_OK {
            return Err(e);
        }
        let mut out = Out {
            d: [0; 16],
            a: [0; 16],
            psw: self.eng.rd_psw(),
            pcxi: self.eng.rd(TriCoreReg::PCXI),
            icr: self.eng.rd(TriCoreReg::ICR),
            fcx: self.eng.rd(TriCoreReg::FCX),
            lcx: self.eng.rd(TriCoreReg::LCX),
            pc: self.eng.rd(TriCoreReg::PC),
            data: self.data_buf.to_vec(),
        };
        for i in 0..16 {
            out.d[i as usize] = self.eng.rd(TriCoreReg::d(i));
            out.a[i as usize] = self.eng.rd(TriCoreReg::a(i));
        }
        Ok(out)
    }
}

thread_local! {
    static THREAD_ENGINES: RefCell<Vec<(Option<c_int>, CachedEngine)>> = const { RefCell::new(Vec::new()) };
}

/// Execute `n` instructions of `code` (placed at CODE) on Unicorn after applying `s`.
/// `model`: `None` = Unicorn's default TriCore model (what `simemu-tc1791` uses), or one of
/// `UC_CPU_TRICORE_TC1796` (TC1.3), `UC_CPU_TRICORE_TC1797` (TC1.3.1), `UC_CPU_TRICORE_TC27X` (TC1.6.x).
pub fn run_model(code: &[u8], n: usize, s: &Setup, model: Option<c_int>) -> Result<Out, uc_err> {
    THREAD_ENGINES.with(|cell| {
        let mut list = cell.borrow_mut();
        let idx = match list.iter().position(|(m, _)| *m == model) {
            Some(i) => i,
            None => {
                let engine = CachedEngine::new(model)?;
                list.push((model, engine));
                list.len() - 1
            }
        };
        list[idx].1.run(code, n, s)
    })
}

/// [`run_model`] with Unicorn's default model.
pub fn run(code: &[u8], n: usize, s: &Setup) -> Result<Out, uc_err> {
    run_model(code, n, s, None)
}

#[cfg(all(test, feature = "unicorn"))]
mod tests {
    use super::*;

    #[test]
    fn oracle_executes_and_known_wrong_is_recognised() {
        // abs d3, d1 on Unicorn
        let mut s = Setup::default();
        s.d[1] = (-5i32) as u32;
        let o = run(&0x31C0_100Bu32.to_le_bytes(), 1, &s).unwrap();
        assert_eq!(o.d[3], 5);
        assert_eq!(o.pc, CODE + 4);
        assert!(trustworthy(0x100B, 0x31C0));
        // imask e2, #1, d0, #1 is a known Unicorn defect
        assert!(!trustworthy(0x10D7, 0x2021));
        assert_eq!(KNOWN_WRONG.len(), simemu_tc1791::isa_fix::FIX_KIND_NAMES.len());
    }

    #[test]
    fn oracle_seeds_and_reports_status_flags() {
        // add d3, d1, d2 (RR 0BH op2 00H) overflowing: V, SV, AV, SAV set; C seeded and kept
        let mut s = Setup::default();
        (s.d[1], s.d[2], s.psw) = (0x7FFF_FFFF, 1, 0x8000_0B80);
        let o = run(&0x3000_210Bu32.to_le_bytes(), 1, &s).unwrap();
        assert_eq!(o.d[3], 0x8000_0000);
        assert_eq!(o.psw >> 27, 0b11111, "PSW {:#010x}", o.psw);
    }

    #[test]
    fn cached_engine_isolates_code_and_data_across_runs() {
        // Run 1 writes DATA and executes at CODE + 0x100; Run 2 must see zeroed CODE/DATA again.
        let mut s1 = Setup::default();
        s1.a[2] = DATA + 0x40;
        s1.d[4] = 0xCAFE_BABE;
        s1.pc = CODE + 0x100;
        // st.w [a2]#0, d4 (0x0920_4289 -> bo(0x89, 0x24, 4, 2, 0))
        let stw = (0x24u32 << 22 | 2 << 12 | 4 << 8 | 0x89).to_le_bytes();
        s1.mem.push((CODE + 0x100, stw.to_vec()));
        let o1 = run(&[], 1, &s1).unwrap();
        assert_eq!(&o1.data[0x40..0x44], &0xCAFE_BABEu32.to_le_bytes());

        // Run 2 executes default NOP (0x0000) at CODE + 0x100 without s.mem: must not re-run st.w,
        // and DATA[0x40..0x44] must be 0.
        let mut s2 = Setup::default();
        s2.a[2] = DATA + 0x40;
        s2.d[4] = 0x1122_3344;
        s2.pc = CODE + 0x100;
        let o2 = run(&[], 1, &s2).unwrap();
        assert_eq!(o2.pc, CODE + 0x102, "zeroed CODE + 0x100 executes 16-bit NOP");
        assert_eq!(&o2.data[0x40..0x44], &[0, 0, 0, 0], "DATA cleared between runs");
    }

    /// Regression test for the Windows 2 MiB worker-stack crash (`EXCEPTION_STACK_OVERFLOW` +
    /// `0xc0000005 STATUS_ACCESS_VIOLATION`) when `uc_open`/`uc_close` VEH registration/removal
    /// raced with `code_gen_buffer` lazy page-commit faults in concurrent `uc_emu_start` threads.
    #[test]
    fn concurrent_engine_churn_and_cache_growth_on_2mib_stacks() {
        let insn = 0x31C0_100Bu32.to_le_bytes(); // abs d3, d1
        std::thread::scope(|s| {
            for i in 0..6 {
                std::thread::Builder::new()
                    .name(format!("uc-grow-{i}"))
                    .stack_size(2 * 1024 * 1024)
                    .spawn_scoped(s, || {
                        let eng = Engine::new_mapped(None).unwrap();
                        eng.mem_write(CODE as u64, &insn);
                        for k in 0..1_500u32 {
                            eng.wr(TriCoreReg::D1, (-(k as i32)) as u32);
                            assert_eq!(eng.remove_cache(CODE as u64, CODE as u64 + 8), UC_ERR_OK);
                            assert_eq!(eng.emu_start(CODE as u64, CODE as u64 + 4, 0, 0), UC_ERR_OK);
                            assert_eq!(eng.rd(TriCoreReg::D3), k);
                        }
                    })
                    .unwrap();
            }
            for i in 0..4 {
                std::thread::Builder::new()
                    .name(format!("uc-churn-{i}"))
                    .stack_size(2 * 1024 * 1024)
                    .spawn_scoped(s, || {
                        for k in 0..120u32 {
                            let eng = Engine::new_mapped(None).unwrap();
                            eng.mem_write(CODE as u64, &insn);
                            eng.wr(TriCoreReg::D1, (-(k as i32)) as u32);
                            assert_eq!(eng.emu_start(CODE as u64, 0, 0, 1), UC_ERR_OK);
                            assert_eq!(eng.rd(TriCoreReg::D3), k);
                            drop(eng);

                            let mut setup = Setup::default();
                            setup.d[1] = (-(k as i32)) as u32;
                            let out = run(&insn, 1, &setup).unwrap();
                            assert_eq!(out.d[3], k);
                        }
                    })
                    .unwrap();
            }
        });
    }
}
