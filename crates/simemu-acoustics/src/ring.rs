//! Lock-free single-producer / single-consumer ring buffer (no allocation after construction).
//!
//! The plant thread pushes probe records, the synthesizer thread pops them; the synthesizer pushes PCM and
//! the WebSocket sender pops it. [`Producer`] and [`Consumer`] are the only handles and are not `Clone`, so
//! there is exactly one of each by construction. Lamport queue: `head` is written only by the consumer,
//! `tail` only by the producer (acquire/release pairs).

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

struct Inner<T> {
    buf: Box<[UnsafeCell<T>]>,
    mask: usize,
    head: AtomicUsize,
    tail: AtomicUsize,
}

// SAFETY: a slot is written only by the producer before the release store of `tail` and read only by the
// consumer after the acquire load of `tail` (and vice versa for `head`).
unsafe impl<T: Send> Sync for Inner<T> {}
unsafe impl<T: Send> Send for Inner<T> {}

pub struct Producer<T> {
    inner: Arc<Inner<T>>,
}
pub struct Consumer<T> {
    inner: Arc<Inner<T>>,
}

/// Ring of at least `capacity` slots (rounded up to a power of two).
pub fn ring<T: Copy + Default>(capacity: usize) -> (Producer<T>, Consumer<T>) {
    let n = capacity.next_power_of_two().max(2);
    let inner = Arc::new(Inner { buf: (0..n).map(|_| UnsafeCell::new(T::default())).collect(), mask: n - 1, head: AtomicUsize::new(0), tail: AtomicUsize::new(0) });
    (Producer { inner: inner.clone() }, Consumer { inner })
}

impl<T: Copy> Producer<T> {
    /// False (value dropped) when full: the producer never blocks.
    #[inline]
    pub fn push(&mut self, v: T) -> bool {
        let r = &self.inner;
        let t = r.tail.load(Ordering::Relaxed);
        if t.wrapping_sub(r.head.load(Ordering::Acquire)) > r.mask {
            return false;
        }
        // SAFETY: slot t is not visible to the consumer until the release store below.
        unsafe { *r.buf[t & r.mask].get() = v };
        r.tail.store(t.wrapping_add(1), Ordering::Release);
        true
    }

    /// Pushes as many of `vs` as fit, returns the count.
    pub fn push_slice(&mut self, vs: &[T]) -> usize {
        let r = &self.inner;
        let t = r.tail.load(Ordering::Relaxed);
        let free = r.mask + 1 - t.wrapping_sub(r.head.load(Ordering::Acquire));
        let n = free.min(vs.len());
        for (i, v) in vs[..n].iter().enumerate() {
            // SAFETY: as in `push`.
            unsafe { *r.buf[t.wrapping_add(i) & r.mask].get() = *v };
        }
        r.tail.store(t.wrapping_add(n), Ordering::Release);
        n
    }

    /// Free slots.
    pub fn free(&self) -> usize {
        let r = &self.inner;
        r.mask + 1 - r.tail.load(Ordering::Relaxed).wrapping_sub(r.head.load(Ordering::Acquire))
    }

    /// True once the paired [`Consumer`] has been dropped.
    pub fn is_disconnected(&self) -> bool {
        Arc::strong_count(&self.inner) == 1
    }
}

impl<T: Copy> Consumer<T> {
    #[inline]
    pub fn pop(&mut self) -> Option<T> {
        let r = &self.inner;
        let h = r.head.load(Ordering::Relaxed);
        if h == r.tail.load(Ordering::Acquire) {
            return None;
        }
        // SAFETY: slot h was published by the producer's release store of `tail`.
        let v = unsafe { *r.buf[h & r.mask].get() };
        r.head.store(h.wrapping_add(1), Ordering::Release);
        Some(v)
    }

    /// Pops up to `out.len()` values, returns the count.
    pub fn pop_slice(&mut self, out: &mut [T]) -> usize {
        let r = &self.inner;
        let h = r.head.load(Ordering::Relaxed);
        let n = r.tail.load(Ordering::Acquire).wrapping_sub(h).min(out.len());
        for (i, o) in out[..n].iter_mut().enumerate() {
            // SAFETY: as in `pop`.
            *o = unsafe { *r.buf[h.wrapping_add(i) & r.mask].get() };
        }
        r.head.store(h.wrapping_add(n), Ordering::Release);
        n
    }

    pub fn len(&self) -> usize {
        let r = &self.inner;
        r.tail.load(Ordering::Acquire).wrapping_sub(r.head.load(Ordering::Relaxed))
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// True once the paired [`Producer`] has been dropped.
    pub fn is_disconnected(&self) -> bool {
        Arc::strong_count(&self.inner) == 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spsc_across_threads_keeps_order_and_drops_when_full() {
        let (mut p, mut c) = ring::<u64>(8);
        assert!((0..8).all(|i| p.push(i)));
        assert!(!p.push(99), "full ring drops");
        assert_eq!((0..8).map(|_| c.pop().unwrap()).collect::<Vec<_>>(), (0..8).collect::<Vec<_>>());
        let (mut p, mut c) = ring::<u64>(1024);
        let h = std::thread::spawn(move || {
            let mut i = 0;
            while i < 200_000 {
                if p.push(i) {
                    i += 1;
                }
            }
        });
        let mut next = 0;
        while next < 200_000 {
            if let Some(v) = c.pop() {
                assert_eq!(v, next);
                next += 1;
            }
        }
        h.join().unwrap();
    }
}
