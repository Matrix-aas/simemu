//! Guest memory: host slices mapped through a 64 KB page table, with self-modifying-code tracking.
//!
//! * A *region* is one host buffer (owned `Vec`, or a raw pointer into memory owned by the board,
//!   e.g. the `HostBuf`s of `simemu-tc1791`). Regions are read-only (flash: stores go to the
//!   [`crate::Bus`] as command cycles) or writable (RAM).
//! * A region (or a part of it) is *mapped* at one or more guest bases (cached/non-cached aliases).
//!   Every 64 KB guest page holds at most one mapping; a page may be partially mapped.
//! * Code tracking: translated blocks mark their bytes in a per-region bitmap of 64-byte granules.
//!   A store into a marked granule (through any alias) reports the granule so the CPU drops the
//!   affected blocks before the next instruction executes.

/// Handle of a registered host buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct RegionId(pub u16);

const PAGE_BITS: u32 = 16;
const PAGE_SIZE: u32 = 1 << PAGE_BITS;
pub(crate) const GRANULE_BITS: u32 = 6;

const F_MAPPED: u8 = 1;
const F_WRITABLE: u8 = 2;
const F_CODE: u8 = 4;

#[derive(Clone, Copy)]
struct Page {
    /// Host address of guest page offset 0 (may lie outside the region; only offsets in
    /// `lo..hi` are dereferenced).
    host: *mut u8,
    lo: u32,
    hi: u32,
    /// Region offset of guest page offset 0 (wrapping).
    region_off: u32,
    region: u16,
    flags: u8,
}

const EMPTY: Page = Page { host: std::ptr::null_mut(), lo: 0, hi: 0, region_off: 0, region: 0, flags: 0 };

struct Region {
    ptr: *mut u8,
    len: usize,
    writable: bool,
    _owned: Option<Box<[u8]>>,
    /// One bit per 64-byte granule that holds translated code.
    code: Vec<u64>,
    /// Guest pages that map (part of) this region.
    pages: Vec<u32>,
}

/// A store hit translated code: region and byte range (granule aligned) to invalidate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CodeWrite {
    pub region: RegionId,
    pub start: u32,
    pub end: u32,
}

pub struct Memory {
    pages: Box<[Page]>,
    regions: Vec<Region>,
}

// SAFETY: the raw pointers address buffers owned by this struct or, for `add_raw`, buffers whose
// owner guarantees they outlive it and are accessed from one thread at a time (the board moves the
// core together with its buffers).
unsafe impl Send for Memory {}

impl Default for Memory {
    fn default() -> Self {
        Self::new()
    }
}

impl Memory {
    pub fn new() -> Self {
        Self { pages: vec![EMPTY; 1 << (32 - PAGE_BITS)].into_boxed_slice(), regions: Vec::new() }
    }

    /// Register an owned buffer.
    pub fn add_owned(&mut self, data: Vec<u8>, writable: bool) -> RegionId {
        let mut b = data.into_boxed_slice();
        let (ptr, len) = (b.as_mut_ptr(), b.len());
        self.push_region(ptr, len, writable, Some(b))
    }

    /// Register a buffer owned elsewhere.
    ///
    /// # Safety
    /// `ptr..ptr+len` must stay valid (and not move) for the lifetime of this `Memory`, and must not
    /// be accessed concurrently with guest execution.
    pub unsafe fn add_raw(&mut self, ptr: *mut u8, len: usize, writable: bool) -> RegionId {
        self.push_region(ptr, len, writable, None)
    }

    fn push_region(&mut self, ptr: *mut u8, len: usize, writable: bool, owned: Option<Box<[u8]>>) -> RegionId {
        assert!(self.regions.len() < u16::MAX as usize);
        let granules = len.div_ceil(1 << GRANULE_BITS);
        self.regions.push(Region { ptr, len, writable, _owned: owned, code: vec![0; granules.div_ceil(64)], pages: Vec::new() });
        RegionId(self.regions.len() as u16 - 1)
    }

    /// Map `len` bytes of `region` starting at region offset `offset` at guest address `base`.
    /// Panics on overlap with an existing mapping in the same 64 KB page or an out-of-range slice.
    pub fn map(&mut self, base: u32, region: RegionId, offset: usize, len: usize) {
        let (ptr, rlen, writable) = {
            let r = &self.regions[region.0 as usize];
            (r.ptr, r.len, r.writable)
        };
        assert!(
            offset.checked_add(len).is_some_and(|e| e <= rlen),
            "mapping exceeds region"
        );
        assert!(len > 0 && (base as u64 + len as u64) <= 1 << 32, "mapping exceeds address space");
        let flags = F_MAPPED | if writable { F_WRITABLE } else { 0 };
        let end = base as u64 + len as u64;
        let mut a = base as u64;
        while a < end {
            let idx = (a >> PAGE_BITS) as usize;
            let page_base = (idx as u64) << PAGE_BITS;
            let lo = (a - page_base) as u32;
            let hi = (end.min(page_base + PAGE_SIZE as u64) - page_base) as u32;
            assert!(self.pages[idx].flags & F_MAPPED == 0, "page {:#x} already mapped", page_base);
            let in_off = offset + (a - base as u64) as usize; // region offset of guest address `a`
            self.pages[idx] = Page {
                host: ptr.wrapping_add(in_off).wrapping_sub(lo as usize),
                lo,
                hi,
                region_off: (in_off as u32).wrapping_sub(lo),
                region: region.0,
                flags,
            };
            self.regions[region.0 as usize].pages.push(idx as u32);
            a = page_base + PAGE_SIZE as u64;
        }
    }

    #[inline(always)]
    fn page(&self, addr: u32) -> &Page {
        // SAFETY: the table has exactly 2^16 entries.
        unsafe { self.pages.get_unchecked((addr >> PAGE_BITS) as usize) }
    }

    /// Host pointer for a `size`-byte read at `addr` (`size > 0`), if it lies inside one mapping.
    #[inline(always)]
    pub fn host_r(&self, addr: u32, size: u32) -> Option<*const u8> {
        let p = self.page(addr);
        let off = addr & (PAGE_SIZE - 1);
        if size != 0 && off >= p.lo && (off as u64) + (size as u64) <= p.hi as u64 {
            Some(p.host.wrapping_add(off as usize) as *const u8)
        } else {
            None
        }
    }

    /// Host pointer for a `size`-byte store at `addr` (`size > 0`) into writable memory, plus the
    /// code granules it overwrites (if any).
    #[inline(always)]
    pub fn host_w(&mut self, addr: u32, size: u32) -> Option<(*mut u8, Option<CodeWrite>)> {
        let p = *self.page(addr);
        let off = addr & (PAGE_SIZE - 1);
        if size == 0
            || p.flags & F_WRITABLE == 0
            || off < p.lo
            || (off as u64) + (size as u64) > p.hi as u64
        {
            return None;
        }
        let ptr = p.host.wrapping_add(off as usize);
        if p.flags & F_CODE == 0 {
            return Some((ptr, None));
        }
        let roff = p.region_off.wrapping_add(off);
        Some((ptr, self.take_code(RegionId(p.region), roff, size)))
    }

    /// Host pointer for a `size`-byte store at `addr` (`size > 0`) into writable memory of a page
    /// without translated code (`host_w` with no code write to report); `None` otherwise. No side effects.
    #[inline(always)]
    pub fn host_w_plain(&self, addr: u32, size: u32) -> Option<*mut u8> {
        let p = self.page(addr);
        let off = addr & (PAGE_SIZE - 1);
        if size != 0
            && p.flags & (F_WRITABLE | F_CODE) == F_WRITABLE
            && off >= p.lo
            && (off as u64) + (size as u64) <= p.hi as u64
        {
            Some(p.host.wrapping_add(off as usize))
        } else {
            None
        }
    }

    /// Clear and report the code granules of `region` covering `[off, off+size)`.
    pub(crate) fn take_code(&mut self, region: RegionId, off: u32, size: u32) -> Option<CodeWrite> {
        if size == 0 {
            return None;
        }
        let r = &mut self.regions[region.0 as usize];
        let g0 = off >> GRANULE_BITS;
        let g1 = (((off as u64) + (size as u64) - 1) >> GRANULE_BITS) as u32;
        let mut hit = false;
        for g in g0..=g1 {
            let (w, b) = ((g / 64) as usize, g % 64);
            if let Some(word) = r.code.get_mut(w) {
                hit |= *word & (1 << b) != 0;
                *word &= !(1 << b);
            }
        }
        let end = (((g1 as u64) + 1) << GRANULE_BITS).min(u32::MAX as u64) as u32;
        hit.then_some(CodeWrite { region, start: g0 << GRANULE_BITS, end })
    }

    /// Region and region offset backing guest address `addr`.
    #[inline]
    pub fn locate(&self, addr: u32) -> Option<(RegionId, u32)> {
        let p = self.page(addr);
        let off = addr & (PAGE_SIZE - 1);
        (p.flags & F_MAPPED != 0 && off >= p.lo && off < p.hi).then(|| (RegionId(p.region), p.region_off.wrapping_add(off)))
    }

    /// Intersect `[lo, hi)` within 64 KB guest page `idx` against its mapping, returning
    /// `(region, region_start, region_end, page_flags)`.
    pub(crate) fn page_span(&self, idx: usize, lo: u32, hi: u32) -> Option<(RegionId, u32, u32, u8)> {
        let p = self.pages.get(idx)?;
        let s = lo.max(p.lo);
        let e = hi.min(p.hi);
        if p.flags & F_MAPPED != 0 && s < e {
            let r_start = p.region_off.wrapping_add(s);
            let r_end = r_start.saturating_add(e - s);
            Some((RegionId(p.region), r_start, r_end, p.flags))
        } else {
            None
        }
    }

    pub(crate) const F_WRITABLE_FLAG: u8 = F_WRITABLE;
    pub(crate) const F_CODE_FLAG: u8 = F_CODE;

    /// Whether `addr` is mapped writable RAM (code there is tracked by granule).
    pub fn writable(&self, addr: u32) -> bool {
        let p = self.page(addr);
        let off = addr & (PAGE_SIZE - 1);
        p.flags & F_WRITABLE != 0 && off >= p.lo && off < p.hi
    }

    /// Record that `[off, off+len)` of `region` holds translated code.
    pub(crate) fn mark_code(&mut self, region: RegionId, off: u32, len: u32) {
        let r = &mut self.regions[region.0 as usize];
        if !r.writable || len == 0 {
            return; // read-only memory changes only through `invalidate` / `write_bytes`
        }
        let g0 = off >> GRANULE_BITS;
        let g1 = (((off as u64) + (len as u64) - 1) >> GRANULE_BITS) as u32;
        for g in g0..=g1 {
            if let Some(w) = r.code.get_mut((g / 64) as usize) {
                *w |= 1 << (g % 64);
            }
        }
        let end_off = (off as u64) + (len as u64);
        for &idx in &r.pages {
            let p = &mut self.pages[idx as usize];
            let s = p.region_off.wrapping_add(p.lo) as u64;
            let e = s + (p.hi - p.lo) as u64;
            if (off as u64) < e && end_off > s {
                p.flags |= F_CODE;
            }
        }
    }

    /// 16-bit instruction fetch.
    #[inline]
    pub fn fetch16(&self, addr: u32) -> Option<u16> {
        // SAFETY: host_r validated the 2-byte range.
        self.host_r(addr, 2).map(|p| unsafe { u16::from_le_bytes(*(p as *const [u8; 2])) })
    }

    /// Debug/loader read of guest memory (no bus access). False if any byte is unmapped.
    pub fn read_bytes(&self, addr: u32, out: &mut [u8]) -> bool {
        for (i, b) in out.iter_mut().enumerate() {
            match self.host_r(addr.wrapping_add(i as u32), 1) {
                // SAFETY: validated 1-byte range.
                Some(p) => *b = unsafe { *p },
                None => return false,
            }
        }
        true
    }

    /// Debug/loader write (also into read-only regions). Atomic on error: if any byte in
    /// `[addr, addr + data.len())` is unmapped, no bytes or code-tracking bits are modified.
    /// Returns the code ranges it overwrote (for both writable and read-only regions), which the
    /// caller must invalidate ([`crate::Cpu::write_bytes`] does).
    pub fn write_bytes(&mut self, addr: u32, data: &[u8]) -> Result<Vec<CodeWrite>, u32> {
        for i in 0..data.len() {
            let a = addr.wrapping_add(i as u32);
            let p = self.page(a);
            let off = a & (PAGE_SIZE - 1);
            if p.flags & F_MAPPED == 0 || off < p.lo || off >= p.hi {
                return Err(a);
            }
        }
        let mut hits: Vec<CodeWrite> = Vec::new();
        for (i, &b) in data.iter().enumerate() {
            let a = addr.wrapping_add(i as u32);
            let p = *self.page(a);
            let off = a & (PAGE_SIZE - 1);
            // SAFETY: pre-validated above as an in-range offset of a live mapping.
            unsafe { *p.host.wrapping_add(off as usize) = b };
            let reg = RegionId(p.region);
            let roff = p.region_off.wrapping_add(off);
            let cw = if p.flags & F_WRITABLE != 0 {
                self.take_code(reg, roff, 1)
            } else {
                // Read-only regions do not maintain granule bits; report exact written offsets.
                Some(CodeWrite { region: reg, start: roff, end: roff.saturating_add(1) })
            };
            if let Some(h) = cw {
                match hits.last_mut() {
                    Some(last)
                        if last.region == h.region
                            && h.start <= last.end
                            && h.end >= last.start =>
                    {
                        last.start = last.start.min(h.start);
                        last.end = last.end.max(h.end);
                    }
                    _ => hits.push(h),
                }
            }
        }
        Ok(hits)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aliases_and_partial_pages() {
        let mut m = Memory::new();
        let r = m.add_owned((0..0x4000u32).map(|i| i as u8).collect(), true);
        m.map(0xAFFF_C000, r, 0, 0x4000);
        m.map(0x8FFF_C000, r, 0, 0x4000);
        assert!(m.host_r(0xAFFF_BFFF, 1).is_none());
        let mut b = [0u8; 2];
        assert!(m.read_bytes(0x8FFF_C010, &mut b));
        assert_eq!(b, [0x10, 0x11]);
        assert!(m.host_r(0xAFFF_FFFF, 2).is_none(), "crosses the end of the mapping");
        assert_eq!(m.locate(0xAFFF_C020), Some((r, 0x20)));
    }

    #[test]
    fn code_write_is_reported_through_any_alias() {
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x8000], true);
        m.map(0xC000_0000, r, 0, 0x8000);
        m.map(0xD400_0000, r, 0, 0x8000);
        m.mark_code(r, 0x100, 8);
        assert!(m.host_w(0xC000_0200, 4).unwrap().1.is_none());
        let hit = m.host_w(0xD400_0104, 4).unwrap().1.unwrap();
        assert_eq!((hit.start, hit.end), (0x100, 0x140));
        assert!(m.host_w(0xC000_0104, 4).unwrap().1.is_none(), "granule cleared after report");
    }

    #[test]
    fn read_only_region_rejects_stores() {
        let mut m = Memory::new();
        let r = m.add_owned(vec![0; 0x1_0000], false);
        m.map(0x8000_0000, r, 0, 0x1_0000);
        assert!(m.host_w(0x8000_0000, 4).is_none());
        assert!(m.host_r(0x8000_FFFC, 4).is_some());
    }

    #[test]
    fn host_pointers_reject_zero_and_wrapping_sizes() {
        let mut m = Memory::new();
        // Unmapped EMPTY page: size 0 must NEVER return Some(null).
        assert!(m.host_r(0x1234_0000, 0).is_none());
        assert!(m.host_w(0x1234_0000, 0).is_none());
        assert!(m.host_w_plain(0x1234_0000, 0).is_none());

        let r = m.add_owned(vec![0x55; 0x1000], true);
        m.map(0xD000_0000, r, 0, 0x1000);
        assert!(m.host_r(0xD000_0000, 0).is_none());
        assert!(m.host_w(0xD000_0000, 0).is_none());
        assert!(m.host_w_plain(0xD000_0000, 0).is_none());

        // Wrapping u32 `off + size`: 0x10 + 0xFFFF_FFF4 == 4 in u32 arithmetic, must be rejected.
        assert!(m.host_r(0xD000_0010, 0xFFFF_FFF4).is_none());
        assert!(m.host_w(0xD000_0010, 0xFFFF_FFF4).is_none());
        assert!(m.host_w_plain(0xD000_0010, 0xFFFF_FFF4).is_none());
    }

    #[test]
    fn write_bytes_is_atomic_on_unmapped_tail_and_reports_ro_hits() {
        let mut m = Memory::new();
        let ram = m.add_owned(vec![0x11; 0x100], true);
        let ro = m.add_owned(vec![0x22; 0x100], false);
        m.map(0xD000_0000, ram, 0, 0x100);
        m.map(0x8000_0000, ro, 0, 0x100);
        m.mark_code(ram, 0, 16);

        // Overrunning into unmapped 0xD000_0100 must leave 0xD000_00FE..0xD000_0100 and code bits untouched.
        assert_eq!(m.write_bytes(0xD000_00FE, &[0xAA, 0xBB, 0xCC]), Err(0xD000_0100));
        let mut check = [0u8; 2];
        assert!(m.read_bytes(0xD000_00FE, &mut check));
        assert_eq!(check, [0x11, 0x11]);
        assert!(m.host_w(0xD000_0000, 4).unwrap().1.is_some(), "code granule preserved on failed write_bytes");

        // Writing into read-only flash reports coalesced CodeWrite for cache invalidation.
        let hits = m.write_bytes(0x8000_0010, &[1, 2, 3, 4]).unwrap();
        assert_eq!(hits, vec![CodeWrite { region: ro, start: 0x10, end: 0x14 }]);
    }
}
