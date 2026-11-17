//! Predecoded basic-block cache keyed by start PC.
//!
//! Blocks are decoded once and reused until the bytes they were decoded from change: a store into
//! RAM code (reported by [`crate::mem::Memory`]) or an explicit [`crate::Cpu::invalidate_code`]
//! from the board (flash programming) drops every block overlapping the written range.
//!
//! Hit path: the instructions of all blocks live in one arena (no per-block allocation, adjacent
//! blocks adjacent in memory) and every block remembers its last two successors ("chaining"), so
//! most block transitions need neither the direct-mapped front table nor the hash map.

use crate::mem::RegionId;
use crate::op::Insn;
use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

/// Upper bound on instructions per block (bounds translation latency and wasted decode work).
pub const MAX_BLOCK_INSNS: usize = 64;

/// Multiplicative hash for block start addresses (SipHash costs more than the lookup it guards).
#[derive(Default)]
pub(crate) struct PcHasher(u64);

impl Hasher for PcHasher {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, b: &[u8]) {
        for &x in b {
            self.0 = (self.0 << 8 | x as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
        }
    }
    fn write_u32(&mut self, v: u32) {
        self.0 = (v as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
}

/// Host bytes a block was decoded from: (region, start offset, end offset); at most two spans
/// (an instruction may straddle two mappings).
pub(crate) type Spans = [(RegionId, u32, u32); 2];

/// A successor of a block: its start PC, block index, first arena record and length (copies of
/// the successor's [`Block`] fields, so that following a link touches one block only).
#[derive(Clone, Copy)]
pub(crate) struct Link {
    pub pc: u32,
    pub idx: u32,
    pub first: u32,
    pub len: u32,
}

/// No successor: an odd PC never matches (instructions are halfword aligned).
const NO_SUCC: Link = Link { pc: 1, idx: 0, first: 0, len: 0 };

/// Hot part of a cached block (48 bytes); its spans live in a parallel vector.
pub(crate) struct Block {
    pub start: u32,
    /// The block's instructions are `insns[first..first + len]` of the cache arena.
    pub first: u32,
    pub len: u32,
    live: bool,
    /// [`crate::Bus::hook`] asked to stop before this block.
    pub hook: bool,
    /// Last two successors taken from this block (blocks with a hook are not linked). Every link is
    /// dropped whenever any block is invalidated, so a link always names a live block.
    succ: [Link; 2],
}

impl Block {
    /// The successor linked for `pc`.
    #[inline(always)]
    pub fn link(&self, pc: u32) -> Option<&Link> {
        if self.succ[0].pc == pc {
            Some(&self.succ[0])
        } else if self.succ[1].pc == pc {
            Some(&self.succ[1])
        } else {
            None
        }
    }
}

const FAST: usize = 1 << 16;

#[derive(Default)]
pub(crate) struct BlockCache {
    blocks: Vec<Block>,
    spans: Vec<Spans>,
    // ponytail: the arena only grows (an invalidated block's instructions stay until `clear`);
    // RAM code is rewritten rarely. Compact on invalidate if a firmware retranslates constantly.
    insns: Vec<Insn>,
    free: Vec<u32>,
    map: HashMap<u32, u32, BuildHasherDefault<PcHasher>>,
    /// Direct-mapped front: (pc, block index + 1); 0 = empty.
    fast: Vec<(u32, u32)>,
}

impl BlockCache {
    #[inline(always)]
    pub fn lookup(&mut self, pc: u32) -> Option<u32> {
        if self.fast.is_empty() {
            self.fast = vec![(0, 0); FAST];
        }
        let slot = ((pc >> 1) as usize) & (FAST - 1);
        let (tag, idx) = self.fast[slot];
        if idx != 0 && tag == pc {
            return Some(idx - 1);
        }
        let i = *self.map.get(&pc)?;
        self.fast[slot] = (pc, i + 1);
        Some(i)
    }

    /// The block starting at `pc` if it is in the direct-mapped front (read only).
    #[inline(always)]
    pub fn probe(&self, pc: u32) -> Option<u32> {
        let (tag, idx) = *self.fast.get(((pc >> 1) as usize) & (FAST - 1))?;
        (idx != 0 && tag == pc).then(|| idx - 1)
    }

    /// The block that followed `prev` last time execution continued at `pc`, if still cached.
    #[inline(always)]
    pub fn chained(&self, prev: u32, pc: u32) -> Option<u32> {
        self.blocks.get(prev as usize)?.link(pc).map(|l| l.idx)
    }

    /// Remember that execution went from block `prev` to block `idx` (starting at `pc`).
    #[inline]
    pub fn link(&mut self, prev: u32, pc: u32, idx: u32) {
        let Some(n) = self.blocks.get(idx as usize) else { return };
        if n.hook {
            return; // reached through `lookup` every time (`Cpu::run` stops before it)
        }
        let l = Link { pc, idx, first: n.first, len: n.len };
        if let Some(b) = self.blocks.get_mut(prev as usize) {
            if b.succ[0].pc != pc {
                b.succ[1] = b.succ[0];
            }
            b.succ[0] = l;
        }
    }

    /// Arena record `i` (a block's `first`, or within its records).
    #[inline(always)]
    pub fn rec(&self, i: u32) -> &Insn {
        debug_assert!((i as usize) < self.insns.len());
        // SAFETY: `first` fields name records appended by `insert`; the arena never shrinks.
        unsafe { self.insns.get_unchecked(i as usize) }
    }

    /// Block `i`: an index returned by `lookup`, `insert` or `chained` (links always name live
    /// blocks), so in range.
    #[inline(always)]
    pub fn block(&self, i: u32) -> &Block {
        debug_assert!((i as usize) < self.blocks.len());
        // SAFETY: see above; `blocks` never shrinks except in `clear`, which drops every index.
        unsafe { self.blocks.get_unchecked(i as usize) }
    }

    #[inline(always)]
    #[allow(dead_code)]
    pub fn insns(&self, b: &Block) -> &[Insn] {
        debug_assert!((b.first + b.len) as usize <= self.insns.len());
        // SAFETY: `first..first + len` was appended to the arena by `insert`, which never shrinks.
        unsafe { self.insns.get_unchecked(b.first as usize..(b.first + b.len) as usize) }
    }

    /// First threaded-code record of the block (the block's records are followed by its `end`
    /// record in the arena).
    #[inline(always)]
    pub fn first(&self, b: &Block) -> &Insn {
        debug_assert!(((b.first + b.len) as usize) < self.insns.len());
        // SAFETY: `first..=first + len` was appended to the arena by `insert`, which never shrinks.
        unsafe { self.insns.get_unchecked(b.first as usize) }
    }

    /// Cache a block: `insns` are its instruction records followed by the closing `end` record.
    pub fn insert(&mut self, start: u32, insns: &[Insn], spans: Spans, hook: bool) -> u32 {
        debug_assert!(insns.len() >= 2);
        assert!(
            self.insns
                .len()
                .checked_add(insns.len())
                .is_some_and(|n| n <= u32::MAX as usize),
            "instruction arena overflow"
        );
        let len = insns.len() as u32 - 1;
        let b = Block { start, first: self.insns.len() as u32, len, succ: [NO_SUCC; 2], live: true, hook };
        self.insns.extend_from_slice(insns);
        let i = match self.free.pop() {
            Some(i) => {
                self.blocks[i as usize] = b;
                self.spans[i as usize] = spans;
                i
            }
            None => {
                assert!(self.blocks.len() < u32::MAX as usize, "block arena overflow");
                self.blocks.push(b);
                self.spans.push(spans);
                self.blocks.len() as u32 - 1
            }
        };
        self.map.insert(start, i);
        i
    }

    /// Drop every block decoded from bytes in `[start, end)` of `region`.
    // ponytail: linear scan over all blocks; code writes are rare (RAM code copy happens before
    // translation). Add a per-region interval index if a firmware patches live code frequently.
    pub fn invalidate(&mut self, region: RegionId, start: u32, end: u32) -> usize {
        let mut n = 0;
        for (i, (b, sp)) in self.blocks.iter_mut().zip(&self.spans).enumerate() {
            if b.live && sp.iter().any(|&(r, s, e)| r == region && s < end && start < e && s < e) {
                b.live = false;
                self.map.remove(&b.start);
                self.free.push(i as u32);
                n += 1;
            }
        }
        if n > 0 {
            self.fast.iter_mut().for_each(|e| *e = (0, 0));
            self.blocks.iter_mut().for_each(|b| b.succ = [NO_SUCC; 2]);
        }
        n
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }

    pub fn len(&self) -> usize {
        self.map.len()
    }
}
