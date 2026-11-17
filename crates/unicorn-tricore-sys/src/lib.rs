#![allow(non_camel_case_types, non_upper_case_globals, dead_code)]
//! Minimal raw FFI bindings to `libunicorn` 2.1.x (TriCore subset).

use std::ffi::{c_char, c_int, c_uint, c_void};

/// `true` when compiled with Cargo feature `link` (`--features unicorn`), linking `libunicorn` 2.1.x.
/// By default (`false`), SIMEMU builds as 100% pure Rust using `tricore-core` with zero C dependencies.
pub const LINKED: bool = cfg!(feature = "link");

#[repr(C)]
pub struct uc_engine {
    _private: [u8; 0],
}

pub type uc_hook = usize;
pub type uc_arch = c_int;
pub type uc_mode = c_int;
pub type uc_err = c_int;
pub type uc_prot = c_int;
pub type uc_mem_type = c_int;
pub type uc_hook_type = c_int;

// Architecture constants
pub const UC_ARCH_TRICORE: uc_arch = 10;

// Mode constants
pub const UC_MODE_LITTLE_ENDIAN: uc_mode = 0;

// CPU models for TriCore
pub const UC_CPU_TRICORE_TC1796: c_int = 0;
pub const UC_CPU_TRICORE_TC1797: c_int = 1;
pub const UC_CPU_TRICORE_TC27X: c_int = 2;

// Error codes (unicorn.h `uc_err`)
pub const UC_ERR_OK: uc_err = 0;
pub const UC_ERR_NOMEM: uc_err = 1;
pub const UC_ERR_ARCH: uc_err = 2;
pub const UC_ERR_HANDLE: uc_err = 3;
pub const UC_ERR_MODE: uc_err = 4;
pub const UC_ERR_VERSION: uc_err = 5;
pub const UC_ERR_READ_UNMAPPED: uc_err = 6;
pub const UC_ERR_WRITE_UNMAPPED: uc_err = 7;
pub const UC_ERR_FETCH_UNMAPPED: uc_err = 8;
pub const UC_ERR_HOOK: uc_err = 9;
pub const UC_ERR_INSN_INVALID: uc_err = 10;
pub const UC_ERR_MAP: uc_err = 11;
pub const UC_ERR_WRITE_PROT: uc_err = 12;
pub const UC_ERR_READ_PROT: uc_err = 13;
pub const UC_ERR_FETCH_PROT: uc_err = 14;
pub const UC_ERR_ARG: uc_err = 15;
pub const UC_ERR_READ_UNALIGNED: uc_err = 16;
pub const UC_ERR_WRITE_UNALIGNED: uc_err = 17;
pub const UC_ERR_FETCH_UNALIGNED: uc_err = 18;
pub const UC_ERR_HOOK_EXIST: uc_err = 19;
pub const UC_ERR_RESOURCE: uc_err = 20;
pub const UC_ERR_EXCEPTION: uc_err = 21;
pub const UC_ERR_OVERFLOW: uc_err = 22;

// Memory protection
pub const UC_PROT_NONE: uc_prot = 0;
pub const UC_PROT_READ: uc_prot = 1;
pub const UC_PROT_WRITE: uc_prot = 2;
pub const UC_PROT_EXEC: uc_prot = 4;
pub const UC_PROT_ALL: uc_prot = 7;

// Memory access types
pub const UC_MEM_READ: uc_mem_type = 16;
pub const UC_MEM_WRITE: uc_mem_type = 17;
pub const UC_MEM_FETCH: uc_mem_type = 18;
pub const UC_MEM_READ_UNMAPPED: uc_mem_type = 19;
pub const UC_MEM_WRITE_UNMAPPED: uc_mem_type = 20;
pub const UC_MEM_FETCH_UNMAPPED: uc_mem_type = 21;
pub const UC_MEM_WRITE_PROT: uc_mem_type = 22;
pub const UC_MEM_READ_PROT: uc_mem_type = 23;
pub const UC_MEM_FETCH_PROT: uc_mem_type = 24;

// Hook types
pub const UC_HOOK_INTR: uc_hook_type = 1 << 0;
pub const UC_HOOK_INSN: uc_hook_type = 1 << 1;
pub const UC_HOOK_CODE: uc_hook_type = 1 << 2;
pub const UC_HOOK_BLOCK: uc_hook_type = 1 << 3;
pub const UC_HOOK_MEM_READ_UNMAPPED: uc_hook_type = 1 << 4;
pub const UC_HOOK_MEM_WRITE_UNMAPPED: uc_hook_type = 1 << 5;
pub const UC_HOOK_MEM_FETCH_UNMAPPED: uc_hook_type = 1 << 6;
pub const UC_HOOK_MEM_READ_PROT: uc_hook_type = 1 << 7;
pub const UC_HOOK_MEM_WRITE_PROT: uc_hook_type = 1 << 8;
pub const UC_HOOK_MEM_FETCH_PROT: uc_hook_type = 1 << 9;
pub const UC_HOOK_MEM_READ: uc_hook_type = 1 << 10;
pub const UC_HOOK_MEM_WRITE: uc_hook_type = 1 << 11;
pub const UC_HOOK_MEM_FETCH: uc_hook_type = 1 << 12;
pub const UC_HOOK_TLB_FILL: uc_hook_type = 1 << 17;

// uc_ctl encoding (unicorn.h: UC_CTL_IO_WRITE = 1, UC_CTL_IO_READ = 2, nr << 26, io << 30)
pub const UC_CTL_IO_WRITE: c_int = 1;
pub const UC_CTL_IO_READ: c_int = 2;
pub const UC_CTL_UC_MODE: c_int = 0;
pub const UC_CTL_UC_PAGE_SIZE: c_int = 1;
pub const UC_CTL_CPU_MODEL: c_int = 7;
pub const UC_CTL_TB_REMOVE_CACHE: c_int = 9;
pub const UC_CTL_TB_FLUSH: c_int = 10;
pub const UC_CTL_TLB_FLUSH: c_int = 11;
pub const UC_CTL_TLB_TYPE: c_int = 12;

/// `uc_tlb_type`: TLB filled by `UC_HOOK_TLB_FILL` callbacks instead of the CPU's MMU model.
pub const UC_TLB_VIRTUAL: c_int = 1;

#[inline]
pub const fn uc_ctl_write(ctl: c_int, nr_args: c_int) -> c_int {
    ((ctl as u32) | ((nr_args as u32) << 26) | ((UC_CTL_IO_WRITE as u32) << 30)) as c_int
}

#[inline]
pub const fn uc_ctl_read(ctl: c_int, nr_args: c_int) -> c_int {
    ((ctl as u32) | ((nr_args as u32) << 26) | ((UC_CTL_IO_READ as u32) << 30)) as c_int
}

/// Result of a `UC_HOOK_TLB_FILL` callback (`uc_tlb_entry`).
#[repr(C)]
pub struct uc_tlb_entry {
    pub paddr: u64,
    pub perms: c_int,
}

pub type uc_cb_tlbevent_t = extern "C" fn(
    uc: *mut uc_engine,
    vaddr: u64,
    mem_type: uc_mem_type,
    result: *mut uc_tlb_entry,
    user_data: *mut c_void,
) -> bool;

// TriCore Registers (from tricore.h)
#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum TriCoreReg {
    Invalid = 0,
    A0 = 1,
    A1 = 2,
    A2 = 3,
    A3 = 4,
    A4 = 5,
    A5 = 6,
    A6 = 7,
    A7 = 8,
    A8 = 9,
    A9 = 10,
    A10 = 11, // SP
    A11 = 12, // RA (Return Address)
    A12 = 13,
    A13 = 14,
    A14 = 15,
    A15 = 16,
    D0 = 17,
    D1 = 18,
    D2 = 19,
    D3 = 20,
    D4 = 21,
    D5 = 22,
    D6 = 23,
    D7 = 24,
    D8 = 25,
    D9 = 26,
    D10 = 27,
    D11 = 28,
    D12 = 29,
    D13 = 30,
    D14 = 31,
    D15 = 32,
    PCXI = 33,
    PSW = 34,
    PSW_USB_C = 35,
    PSW_USB_V = 36,
    PSW_USB_SV = 37,
    PSW_USB_AV = 38,
    PSW_USB_SAV = 39,
    PC = 40,
    SYSCON = 41,
    CPU_ID = 42,
    BIV = 43,
    BTV = 44,
    ISP = 45,
    ICR = 46,
    FCX = 47,
    LCX = 48,
    COMPAT = 49,
}

impl TriCoreReg {
    /// `D[n]` for `n` in `0..16`.
    pub fn d(n: u32) -> Self {
        const D: [TriCoreReg; 16] = [
            TriCoreReg::D0, TriCoreReg::D1, TriCoreReg::D2, TriCoreReg::D3,
            TriCoreReg::D4, TriCoreReg::D5, TriCoreReg::D6, TriCoreReg::D7,
            TriCoreReg::D8, TriCoreReg::D9, TriCoreReg::D10, TriCoreReg::D11,
            TriCoreReg::D12, TriCoreReg::D13, TriCoreReg::D14, TriCoreReg::D15,
        ];
        D[(n & 15) as usize]
    }

    /// `A[n]` for `n` in `0..16`.
    pub fn a(n: u32) -> Self {
        const A: [TriCoreReg; 16] = [
            TriCoreReg::A0, TriCoreReg::A1, TriCoreReg::A2, TriCoreReg::A3,
            TriCoreReg::A4, TriCoreReg::A5, TriCoreReg::A6, TriCoreReg::A7,
            TriCoreReg::A8, TriCoreReg::A9, TriCoreReg::A10, TriCoreReg::A11,
            TriCoreReg::A12, TriCoreReg::A13, TriCoreReg::A14, TriCoreReg::A15,
        ];
        A[(n & 15) as usize]
    }
}

pub type uc_cb_hookcode_t =
    extern "C" fn(uc: *mut uc_engine, address: u64, size: u32, user_data: *mut c_void);

pub type uc_cb_hookmem_t = extern "C" fn(
    uc: *mut uc_engine,
    mem_type: uc_mem_type,
    address: u64,
    size: c_int,
    value: i64,
    user_data: *mut c_void,
);

pub type uc_cb_eventmem_t = extern "C" fn(
    uc: *mut uc_engine,
    mem_type: uc_mem_type,
    address: u64,
    size: c_int,
    value: i64,
    user_data: *mut c_void,
) -> bool;

/// `uc_mmio_map` read callback: returns the value read at `offset` (relative to the region base).
pub type uc_cb_mmio_read_t =
    extern "C" fn(uc: *mut uc_engine, offset: u64, size: c_uint, user_data: *mut c_void) -> u64;

/// `uc_mmio_map` write callback.
pub type uc_cb_mmio_write_t = extern "C" fn(
    uc: *mut uc_engine,
    offset: u64,
    size: c_uint,
    value: u64,
    user_data: *mut c_void,
);

#[cfg(feature = "link")]
mod raw_ffi {
    use super::*;

    // Unicorn 2.1.x declares memory/MMIO `size` parameters as `uint64_t` (`unicorn.h`), whereas
    // `uc_emu_start(..., size_t count)` uses `size_t`. Bind the exact C ABI (`u64`) here and
    // expose `usize`-accepting wrappers at crate root for caller ergonomics and compatibility.
    extern "C" {
        pub fn uc_mem_write(
            uc: *mut uc_engine,
            address: u64,
            bytes: *const c_void,
            size: u64,
        ) -> uc_err;
        pub fn uc_mem_read(
            uc: *mut uc_engine,
            address: u64,
            bytes: *mut c_void,
            size: u64,
        ) -> uc_err;

        pub fn uc_mem_map(uc: *mut uc_engine, address: u64, size: u64, perms: u32) -> uc_err;
        pub fn uc_mem_map_ptr(
            uc: *mut uc_engine,
            address: u64,
            size: u64,
            perms: u32,
            ptr: *mut c_void,
        ) -> uc_err;
        pub fn uc_mmio_map(
            uc: *mut uc_engine,
            address: u64,
            size: u64,
            read_cb: Option<uc_cb_mmio_read_t>,
            user_data_read: *mut c_void,
            write_cb: Option<uc_cb_mmio_write_t>,
            user_data_write: *mut c_void,
        ) -> uc_err;
        pub fn uc_mem_unmap(uc: *mut uc_engine, address: u64, size: u64) -> uc_err;
        pub fn uc_mem_protect(uc: *mut uc_engine, address: u64, size: u64, perms: u32) -> uc_err;
    }
}

#[cfg(not(feature = "link"))]
mod raw_ffi {
    use super::*;

    #[inline]
    pub unsafe fn uc_mem_write(_: *mut uc_engine, _: u64, _: *const c_void, _: u64) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_mem_read(_: *mut uc_engine, _: u64, _: *mut c_void, _: u64) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_mem_map(_: *mut uc_engine, _: u64, _: u64, _: u32) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_mem_map_ptr(_: *mut uc_engine, _: u64, _: u64, _: u32, _: *mut c_void) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_mmio_map(
        _: *mut uc_engine,
        _: u64,
        _: u64,
        _: Option<uc_cb_mmio_read_t>,
        _: *mut c_void,
        _: Option<uc_cb_mmio_write_t>,
        _: *mut c_void,
    ) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_mem_unmap(_: *mut uc_engine, _: u64, _: u64) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_mem_protect(_: *mut uc_engine, _: u64, _: u64, _: u32) -> uc_err {
        UC_ERR_ARCH
    }
}

#[cfg(feature = "link")]
extern "C" {
    pub fn uc_version(major: *mut c_uint, minor: *mut c_uint) -> c_uint;
    pub fn uc_open(arch: uc_arch, mode: uc_mode, uc: *mut *mut uc_engine) -> uc_err;
    pub fn uc_close(uc: *mut uc_engine) -> uc_err;
    pub fn uc_strerror(code: uc_err) -> *const c_char;
    pub fn uc_errno(uc: *mut uc_engine) -> uc_err;

    pub fn uc_reg_write(uc: *mut uc_engine, regid: c_int, value: *const c_void) -> uc_err;
    pub fn uc_reg_read(uc: *mut uc_engine, regid: c_int, value: *mut c_void) -> uc_err;

    pub fn uc_emu_start(
        uc: *mut uc_engine,
        begin: u64,
        until: u64,
        timeout: u64,
        count: usize,
    ) -> uc_err;
    pub fn uc_emu_stop(uc: *mut uc_engine) -> uc_err;

    pub fn uc_hook_add(
        uc: *mut uc_engine,
        hh: *mut uc_hook,
        htype: uc_hook_type,
        callback: *mut c_void,
        user_data: *mut c_void,
        begin: u64,
        end: u64,
        ...
    ) -> uc_err;
    pub fn uc_hook_del(uc: *mut uc_engine, hh: uc_hook) -> uc_err;

    pub fn uc_ctl(uc: *mut uc_engine, control: c_int, ...) -> uc_err;
}

#[cfg(not(feature = "link"))]
#[allow(clippy::missing_safety_doc)]
mod unlinked_stubs {
    use super::*;

    const UNLINKED_MSG: &[u8] = b"libunicorn not linked (rebuild with `--features unicorn`)\0";

    #[inline]
    pub unsafe fn uc_version(major: *mut c_uint, minor: *mut c_uint) -> c_uint {
        if !major.is_null() {
            *major = 0;
        }
        if !minor.is_null() {
            *minor = 0;
        }
        0
    }
    #[inline]
    pub unsafe fn uc_open(_: uc_arch, _: uc_mode, _: *mut *mut uc_engine) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_close(_: *mut uc_engine) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_strerror(_: uc_err) -> *const c_char {
        UNLINKED_MSG.as_ptr().cast()
    }
    #[inline]
    pub unsafe fn uc_errno(_: *mut uc_engine) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_reg_write(_: *mut uc_engine, _: c_int, _: *const c_void) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_reg_read(_: *mut uc_engine, _: c_int, _: *mut c_void) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_emu_start(_: *mut uc_engine, _: u64, _: u64, _: u64, _: usize) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_emu_stop(_: *mut uc_engine) -> uc_err {
        UC_ERR_ARCH
    }
    #[inline]
    pub unsafe fn uc_hook_del(_: *mut uc_engine, _: uc_hook) -> uc_err {
        UC_ERR_ARCH
    }
}

#[cfg(not(feature = "link"))]
pub use unlinked_stubs::*;

/// Register a standard (non-`UC_HOOK_INSN`) callback (`uc_hook_add` without extra variadic args).
///
/// # Safety
/// `uc` must be a valid engine handle, `hh` writable, and `callback` must match `htype`'s ABI.
#[inline]
pub unsafe fn uc_hook_add_basic(
    uc: *mut uc_engine,
    hh: *mut uc_hook,
    htype: uc_hook_type,
    callback: *mut c_void,
    user_data: *mut c_void,
    begin: u64,
    end: u64,
) -> uc_err {
    #[cfg(feature = "link")]
    {
        uc_hook_add(uc, hh, htype, callback, user_data, begin, end)
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = (uc, hh, htype, callback, user_data, begin, end);
        UC_ERR_ARCH
    }
}

/// Write `size` bytes from `bytes` to guest `address` (`uc_mem_write`).
///
/// # Safety
/// `uc` must be a valid engine handle and `bytes` must be valid for reads of `size` bytes.
#[inline]
pub unsafe fn uc_mem_write(
    uc: *mut uc_engine,
    address: u64,
    bytes: *const c_void,
    size: usize,
) -> uc_err {
    raw_ffi::uc_mem_write(uc, address, bytes, size as u64)
}

/// Read `size` bytes from guest `address` into `bytes` (`uc_mem_read`).
///
/// # Safety
/// `uc` must be a valid engine handle and `bytes` must be valid for writes of `size` bytes.
#[inline]
pub unsafe fn uc_mem_read(
    uc: *mut uc_engine,
    address: u64,
    bytes: *mut c_void,
    size: usize,
) -> uc_err {
    raw_ffi::uc_mem_read(uc, address, bytes, size as u64)
}

/// Map `size` bytes of Unicorn-allocated guest memory at `address` (`uc_mem_map`).
///
/// # Safety
/// `uc` must be a valid engine handle.
#[inline]
pub unsafe fn uc_mem_map(uc: *mut uc_engine, address: u64, size: usize, perms: u32) -> uc_err {
    raw_ffi::uc_mem_map(uc, address, size as u64, perms)
}

/// Map `size` bytes of caller-owned host memory `ptr` at `address` (`uc_mem_map_ptr`).
///
/// # Safety
/// `uc` must be a valid engine handle and `ptr` must remain valid for `size` bytes until unmapped
/// or `uc_close`.
#[inline]
pub unsafe fn uc_mem_map_ptr(
    uc: *mut uc_engine,
    address: u64,
    size: usize,
    perms: u32,
    ptr: *mut c_void,
) -> uc_err {
    raw_ffi::uc_mem_map_ptr(uc, address, size as u64, perms, ptr)
}

/// Map `size` bytes of MMIO callbacks at `address` (`uc_mmio_map`).
///
/// # Safety
/// `uc` must be a valid engine handle and user-data pointers must stay valid for their callbacks.
#[inline]
pub unsafe fn uc_mmio_map(
    uc: *mut uc_engine,
    address: u64,
    size: usize,
    read_cb: Option<uc_cb_mmio_read_t>,
    user_data_read: *mut c_void,
    write_cb: Option<uc_cb_mmio_write_t>,
    user_data_write: *mut c_void,
) -> uc_err {
    raw_ffi::uc_mmio_map(
        uc,
        address,
        size as u64,
        read_cb,
        user_data_read,
        write_cb,
        user_data_write,
    )
}

/// Unmap `size` bytes of guest memory at `address` (`uc_mem_unmap`).
///
/// # Safety
/// `uc` must be a valid engine handle.
#[inline]
pub unsafe fn uc_mem_unmap(uc: *mut uc_engine, address: u64, size: usize) -> uc_err {
    raw_ffi::uc_mem_unmap(uc, address, size as u64)
}

/// Update permissions on `size` bytes of guest memory at `address` (`uc_mem_protect`).
///
/// # Safety
/// `uc` must be a valid engine handle.
#[inline]
pub unsafe fn uc_mem_protect(uc: *mut uc_engine, address: u64, size: usize, perms: u32) -> uc_err {
    raw_ffi::uc_mem_protect(uc, address, size as u64, perms)
}

/// Invalidate all translated blocks overlapping `[begin, end)` (`uc_ctl_remove_cache`).
///
/// # Safety
/// `uc` must be a valid engine handle. Must not be called from inside an emulation callback.
pub unsafe fn uc_ctl_remove_cache(uc: *mut uc_engine, begin: u64, end: u64) -> uc_err {
    #[cfg(feature = "link")]
    {
        uc_ctl(uc, uc_ctl_write(UC_CTL_TB_REMOVE_CACHE, 2), begin, end)
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = (uc, begin, end);
        UC_ERR_ARCH
    }
}

/// Flush the entire translation-block buffer (`UC_CTL_TB_FLUSH`).
///
/// # Safety
/// `uc` must be a valid engine handle. Must not be called from inside an emulation callback.
pub unsafe fn uc_ctl_flush_tb(uc: *mut uc_engine) -> uc_err {
    #[cfg(feature = "link")]
    {
        uc_ctl(uc, uc_ctl_write(UC_CTL_TB_FLUSH, 0))
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = uc;
        UC_ERR_ARCH
    }
}

/// Inline block accounting of a SIMEMU-patched Unicorn (`scripts/build_unicorn.py`); stock
/// Unicorn rejects this control with `UC_ERR_ARG`.
pub const UC_CTL_SIMEMU_BLOCK_COUNTER: c_int = 0x5151;

/// Block classification callback: returns nonzero if the block must always call the block hook,
/// else stores its instruction count and region bit.
pub type uc_simemu_classify_t =
    extern "C" fn(user: *mut c_void, pc: u64, size: u32, n: *mut u64, bit: *mut u32) -> u32;

/// `struct uc_simemu_bc` of the patched Unicorn: every translation block starts with an inline
/// fast path that performs `*executed += n; *last_pc = pc` when the block is classified fast,
/// `*stop == 0`, `*seen & bit != 0` and `*executed + n <= *limit`; otherwise it calls the
/// `UC_HOOK_BLOCK` hook as usual.
#[repr(C)]
pub struct uc_simemu_bc {
    pub executed: *mut u64,
    pub limit: *const u64,
    pub stop: *const u8,
    pub seen: *const u16,
    pub last_pc: *mut u32,
    pub classify: uc_simemu_classify_t,
    pub user: *mut c_void,
}

/// Enable inline block accounting (patched Unicorn only; `UC_ERR_ARG` on stock builds).
///
/// # Safety
/// `uc` must be a valid engine handle; every pointer in `cfg` must stay valid while the engine
/// lives. Must not be called from inside an emulation callback.
pub unsafe fn uc_ctl_simemu_block_counter(uc: *mut uc_engine, cfg: *const uc_simemu_bc) -> uc_err {
    #[cfg(feature = "link")]
    {
        uc_ctl(uc, uc_ctl_write(UC_CTL_SIMEMU_BLOCK_COUNTER, 1), cfg)
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = (uc, cfg);
        UC_ERR_ARCH
    }
}

/// Exact instruction timing of a SIMEMU-patched Unicorn (needs [`UC_CTL_SIMEMU_BLOCK_COUNTER`]):
/// a peripheral access sees `*executed` = instructions retired before the accessing instruction,
/// that instruction ends its translation block, and a nonzero `*truncate` stored by the block hook
/// re-runs the block limited to that many instructions. Stock Unicorn rejects it (`UC_ERR_ARG`).
pub const UC_CTL_SIMEMU_EXACT: c_int = 0x5153;

/// Enable exact instruction timing (patched Unicorn only; `UC_ERR_ARG` otherwise).
///
/// # Safety
/// `uc` must be a valid engine handle with inline block accounting enabled; `truncate` must stay
/// valid while the engine lives. Must not be called from inside an emulation callback.
pub unsafe fn uc_ctl_simemu_exact(uc: *mut uc_engine, truncate: *mut u64) -> uc_err {
    #[cfg(feature = "link")]
    {
        uc_ctl(uc, uc_ctl_write(UC_CTL_SIMEMU_EXACT, 1), truncate)
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = (uc, truncate);
        UC_ERR_ARCH
    }
}

/// TriCore ISA defects fixed in a SIMEMU-patched Unicorn (`scripts/unicorn-2.1.4-simemu.patch`);
/// stock Unicorn rejects this control with `UC_ERR_ARG` (no fixes).
pub const UC_CTL_SIMEMU_ISA_FIXES: c_int = 0x5152;
/// `psw_read` keeps `PSW[24]` (`RM[0]`) instead of masking PSW with 0x06FFFFFF.
pub const SIMEMU_FIX_PSW_RM: u32 = 1;
/// `fpu_set_state` maps PSW.RM to the softfloat rounding mode (stock passes the raw field).
pub const SIMEMU_FIX_FPU_RM: u32 = 2;
/// SRO `LD.H D15, [A[b]]off4` uses `EA = A[b] + 2 * off4` (stock: `+ off4`).
pub const SIMEMU_FIX_SRO_LD_H: u32 = 4;
/// `MTCR ICR` keeps the read-only ICR.PIPN, and MFCR/MTCR ICR go through the TCG global that
/// ENABLE/DISABLE/RESTORE update (stock: raw loads/stores of a possibly stale ICR).
pub const SIMEMU_FIX_ICR: u32 = 8;
/// `UPDFL` sets `PSW[31:24] = (PSW[31:24] & ~D[a][15:8]) | (D[a][7:0] & D[a][15:8])` (stock: never
/// updates RM, clears the unmasked flags).
pub const SIMEMU_FIX_UPDFL: u32 = 16;
/// `RET` keeps `PSW[25:24]` (RM) on TC1.3.1/TC1.6 (stock restores it from the CSA).
pub const SIMEMU_FIX_RET_RM: u32 = 32;

/// The `SIMEMU_FIX_*` mask of the linked library (0 for stock Unicorn).
///
/// # Safety
/// `uc` must be a valid engine handle.
pub unsafe fn uc_ctl_simemu_isa_fixes(uc: *mut uc_engine) -> u32 {
    #[cfg(feature = "link")]
    {
        let mut m = 0u32;
        if uc_ctl(uc, uc_ctl_read(UC_CTL_SIMEMU_ISA_FIXES, 1), &mut m as *mut u32) == UC_ERR_OK {
            m
        } else {
            0
        }
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = uc;
        0
    }
}

/// Select the TLB type (`uc_ctl_tlb_mode`), e.g. [`UC_TLB_VIRTUAL`].
///
/// # Safety
/// `uc` must be a valid engine handle. Must not be called from inside an emulation callback.
pub unsafe fn uc_ctl_tlb_mode(uc: *mut uc_engine, mode: c_int) -> uc_err {
    #[cfg(feature = "link")]
    {
        uc_ctl(uc, uc_ctl_write(UC_CTL_TLB_TYPE, 1), mode)
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = (uc, mode);
        UC_ERR_ARCH
    }
}

/// Drop every TLB entry (`uc_ctl_flush_tlb`).
///
/// # Safety
/// `uc` must be a valid engine handle. Must not be called from inside an emulation callback.
pub unsafe fn uc_ctl_flush_tlb(uc: *mut uc_engine) -> uc_err {
    #[cfg(feature = "link")]
    {
        uc_ctl(uc, uc_ctl_write(UC_CTL_TLB_FLUSH, 0))
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = uc;
        UC_ERR_ARCH
    }
}

/// Target page size of the engine (`uc_ctl_get_page_size`).
///
/// # Safety
/// `uc` must be a valid engine handle.
pub unsafe fn uc_ctl_get_page_size(uc: *mut uc_engine, size: *mut u32) -> uc_err {
    #[cfg(feature = "link")]
    {
        uc_ctl(uc, uc_ctl_read(UC_CTL_UC_PAGE_SIZE, 1), size)
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = (uc, size);
        UC_ERR_ARCH
    }
}

/// Select the emulated CPU model (`uc_ctl_set_cpu_model`). Must be called before the first
/// emulation/register access.
///
/// # Safety
/// `uc` must be a valid engine handle.
pub unsafe fn uc_ctl_set_cpu_model(uc: *mut uc_engine, model: c_int) -> uc_err {
    #[cfg(feature = "link")]
    {
        uc_ctl(uc, uc_ctl_write(UC_CTL_CPU_MODEL, 1), model)
    }
    #[cfg(not(feature = "link"))]
    {
        let _ = (uc, model);
        UC_ERR_ARCH
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ctl_bit_encoding_matches_unicorn_header() {
        // UC_CTL_IO_WRITE = 1 (bit 30), UC_CTL_IO_READ = 2 (bit 31), nr_args in bits [29:26].
        assert_eq!(
            uc_ctl_write(UC_CTL_TB_REMOVE_CACHE, 2) as u32,
            0x4800_0009
        );
        assert_eq!(uc_ctl_read(UC_CTL_UC_PAGE_SIZE, 1) as u32, 0x8400_0001);
        assert_eq!(uc_ctl_write(UC_CTL_TLB_FLUSH, 0) as u32, 0x4000_000B);
    }

    #[test]
    #[cfg(feature = "link")]
    fn ffi_open_map_write_read_roundtrip() {
        use std::ptr;
        unsafe {
            let mut major = 0;
            let mut minor = 0;
            let combined = uc_version(&mut major, &mut minor);
            assert_eq!(major, 2);
            assert!(combined >= 0x0201);

            let mut uc: *mut uc_engine = ptr::null_mut();
            assert_eq!(
                uc_open(UC_ARCH_TRICORE, UC_MODE_LITTLE_ENDIAN, &mut uc),
                UC_ERR_OK
            );
            assert!(!uc.is_null());

            let mut page_size = 0u32;
            assert_eq!(uc_ctl_get_page_size(uc, &mut page_size), UC_ERR_OK);
            assert!(page_size.is_power_of_two() && page_size >= 1024);

            let base = 0xD000_0000u64;
            let map_len = (page_size as usize).max(4096);
            assert_eq!(
                uc_mem_map(uc, base, map_len, UC_PROT_ALL as u32),
                UC_ERR_OK
            );

            let payload = [0xDEu8, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78];
            assert_eq!(
                uc_mem_write(uc, base + 16, payload.as_ptr().cast(), payload.len()),
                UC_ERR_OK
            );
            let mut readback = [0u8; 8];
            assert_eq!(
                uc_mem_read(
                    uc,
                    base + 16,
                    readback.as_mut_ptr().cast(),
                    readback.len()
                ),
                UC_ERR_OK
            );
            assert_eq!(readback, payload);

            assert_eq!(
                uc_mem_protect(uc, base, map_len, UC_PROT_READ as u32),
                UC_ERR_OK
            );
            assert_eq!(uc_mem_unmap(uc, base, map_len), UC_ERR_OK);
            assert_eq!(uc_close(uc), UC_ERR_OK);
        }
    }
}
