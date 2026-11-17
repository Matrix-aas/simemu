//! Real-time plumbing of the studio: thread QoS, per-thread CPU time, a lock-free single-producer /
//! single-consumer ring (telemetry, audio, UI streams; also the lock-step mailbox of the co-simulation
//! workers) and the wall-clock pacer with its overload policy. Methodology and numbers:
//! `docs/PERFORMANCE.md`.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread::Thread;
use std::time::{Duration, Instant};

/// Scheduling class of a studio thread.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThreadRole {
    /// Simulation threads (ECU + plant, TCU, audio synthesis): must keep the 1 ms lock-step.
    Realtime,
    /// UI streaming, recording, HTTP.
    Background,
}

/// Apply `role` to the calling thread. macOS: QoS class USER_INTERACTIVE for the simulation (the
/// scheduler keeps such threads on performance cores), UTILITY for background work
/// (`pthread_set_qos_class_self_np`, <pthread/qos.h>). Windows: HighQoS (execution-speed throttling
/// disabled, `ABOVE_NORMAL_PRIORITY_CLASS` + `THREAD_PRIORITY_HIGHEST`) so Windows 11 Thread Director
/// keeps simulation threads on P-cores even when the browser window is in the foreground.
pub fn set_thread_role(role: ThreadRole) {
    #[cfg(target_os = "macos")]
    {
        extern "C" {
            fn pthread_set_qos_class_self_np(qos: u32, relative_priority: i32) -> i32;
        }
        const QOS_CLASS_USER_INTERACTIVE: u32 = 0x21;
        const QOS_CLASS_UTILITY: u32 = 0x11;
        let class = match role {
            ThreadRole::Realtime => QOS_CLASS_USER_INTERACTIVE,
            ThreadRole::Background => QOS_CLASS_UTILITY,
        };
        // SAFETY: plain libSystem call on the current thread; failure leaves the default class.
        unsafe { pthread_set_qos_class_self_np(class, 0) };
    }
    #[cfg(windows)]
    {
        use std::ffi::c_void;
        #[repr(C)]
        struct PowerThrottlingState {
            version: u32,
            control_mask: u32,
            state_mask: u32,
        }
        #[link(name = "winmm")]
        extern "system" {
            fn timeBeginPeriod(u_period: u32) -> u32;
        }
        extern "system" {
            fn GetCurrentProcess() -> *mut c_void;
            fn GetCurrentThread() -> *mut c_void;
            fn SetPriorityClass(h_process: *mut c_void, priority_class: u32) -> i32;
            fn SetProcessInformation(
                h_process: *mut c_void,
                class: i32,
                info: *const c_void,
                size: u32,
            ) -> i32;
            fn SetThreadPriority(h_thread: *mut c_void, priority: i32) -> i32;
            fn SetThreadInformation(
                h_thread: *mut c_void,
                class: i32,
                info: *const c_void,
                size: u32,
            ) -> i32;
        }
        const ABOVE_NORMAL_PRIORITY_CLASS: u32 = 0x0000_8000;
        const THREAD_PRIORITY_HIGHEST: i32 = 2;
        const THREAD_PRIORITY_BELOW_NORMAL: i32 = -1;
        const PROCESS_POWER_THROTTLING: i32 = 4;
        const THREAD_POWER_THROTTLING: i32 = 4;
        const POWER_THROTTLING_CURRENT_VERSION: u32 = 1;
        const POWER_THROTTLING_EXECUTION_SPEED: u32 = 0x1;

        static TIMER_PERIOD: std::sync::Once = std::sync::Once::new();

        // SAFETY: pseudo-handles from GetCurrentProcess/GetCurrentThread are always valid.
        unsafe {
            let h_thread = GetCurrentThread();
            match role {
                ThreadRole::Realtime => {
                    TIMER_PERIOD.call_once(|| {
                        timeBeginPeriod(1);
                    });
                    let h_proc = GetCurrentProcess();
                    let proc_pt = PowerThrottlingState {
                        version: POWER_THROTTLING_CURRENT_VERSION,
                        control_mask: POWER_THROTTLING_EXECUTION_SPEED,
                        state_mask: 0,
                    };
                    SetPriorityClass(h_proc, ABOVE_NORMAL_PRIORITY_CLASS);
                    SetProcessInformation(
                        h_proc,
                        PROCESS_POWER_THROTTLING,
                        (&proc_pt as *const PowerThrottlingState).cast(),
                        std::mem::size_of::<PowerThrottlingState>() as u32,
                    );
                    SetThreadPriority(h_thread, THREAD_PRIORITY_HIGHEST);
                    let th_pt = PowerThrottlingState {
                        version: POWER_THROTTLING_CURRENT_VERSION,
                        control_mask: POWER_THROTTLING_EXECUTION_SPEED,
                        state_mask: 0,
                    };
                    SetThreadInformation(
                        h_thread,
                        THREAD_POWER_THROTTLING,
                        (&th_pt as *const PowerThrottlingState).cast(),
                        std::mem::size_of::<PowerThrottlingState>() as u32,
                    );
                }
                ThreadRole::Background => {
                    SetThreadPriority(h_thread, THREAD_PRIORITY_BELOW_NORMAL);
                    let th_pt = PowerThrottlingState {
                        version: POWER_THROTTLING_CURRENT_VERSION,
                        control_mask: POWER_THROTTLING_EXECUTION_SPEED,
                        state_mask: POWER_THROTTLING_EXECUTION_SPEED,
                    };
                    SetThreadInformation(
                        h_thread,
                        THREAD_POWER_THROTTLING,
                        (&th_pt as *const PowerThrottlingState).cast(),
                        std::mem::size_of::<PowerThrottlingState>() as u32,
                    );
                }
            }
        }
    }
    #[cfg(not(any(target_os = "macos", windows)))]
    let _ = role;
}

/// CPU time consumed by the calling thread (s); 0 where unsupported.
pub fn thread_cpu_s() -> f64 {
    #[cfg(unix)]
    {
        #[repr(C)]
        struct Timespec {
            tv_sec: i64,
            tv_nsec: i64,
        }
        extern "C" {
            fn clock_gettime(clk: i32, tp: *mut Timespec) -> i32;
        }
        #[cfg(target_os = "macos")]
        const CLOCK_THREAD_CPUTIME_ID: i32 = 16;
        #[cfg(not(target_os = "macos"))]
        const CLOCK_THREAD_CPUTIME_ID: i32 = 3;
        let mut ts = Timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: valid out pointer; 64-bit time_t/long on the supported (64-bit) hosts.
        if unsafe { clock_gettime(CLOCK_THREAD_CPUTIME_ID, &mut ts) } == 0 {
            return ts.tv_sec as f64 + ts.tv_nsec as f64 * 1e-9;
        }
    }
    #[cfg(windows)]
    {
        use std::ffi::c_void;
        extern "system" {
            fn GetCurrentThread() -> *mut c_void;
            fn GetThreadTimes(
                h_thread: *mut c_void,
                creation: *mut u64,
                exit: *mut u64,
                kernel: *mut u64,
                user: *mut u64,
            ) -> i32;
        }
        let (mut c, mut e, mut k, mut u) = (0u64, 0u64, 0u64, 0u64);
        // SAFETY: valid pseudo-handle and out pointers; FILETIME is two u32s (100 ns units) aligned as u64 on x86_64.
        if unsafe { GetThreadTimes(GetCurrentThread(), &mut c, &mut e, &mut k, &mut u) } != 0 {
            return (k + u) as f64 * 1e-7;
        }
    }
    0.0
}

// ---------------------------------------------------------------------------------------------
// SPSC ring
// ---------------------------------------------------------------------------------------------

struct Ring<T> {
    slots: Box<[UnsafeCell<MaybeUninit<T>>]>,
    mask: usize,
    /// Next slot to read (owned by the consumer).
    head: AtomicUsize,
    /// Next slot to write (owned by the producer).
    tail: AtomicUsize,
    /// Pushes refused because the ring was full.
    dropped: AtomicU64,
    /// Consumer parked in [`Consumer::recv`] (thread to unpark).
    waiter: std::sync::Mutex<Option<Thread>>,
    waiting: AtomicBool,
    closed: AtomicBool,
}

// SAFETY: a slot is written only by the producer before it publishes `tail` (Release) and read only
// by the consumer after observing it (Acquire), and vice versa for `head`; `T: Send` moves across.
unsafe impl<T: Send> Sync for Ring<T> {}
unsafe impl<T: Send> Send for Ring<T> {}

impl<T> Drop for Ring<T> {
    fn drop(&mut self) {
        let (mut h, t) = (*self.head.get_mut(), *self.tail.get_mut());
        while h != t {
            // SAFETY: slots in head..tail are initialised and owned by the ring.
            unsafe { self.slots[h & self.mask].get_mut().assume_init_drop() };
            h = h.wrapping_add(1);
        }
    }
}

/// Lock-free single-producer / single-consumer ring of capacity `cap` (rounded up to a power of
/// two): wait-free `push`/`pop`, plus a blocking [`Consumer::recv`] that spins briefly before parking
/// (latency of a lock-step handoff in the microsecond range).
pub fn spsc<T: Send>(cap: usize) -> (Producer<T>, Consumer<T>) {
    let n = cap.max(2).next_power_of_two();
    let ring = Arc::new(Ring {
        slots: (0..n).map(|_| UnsafeCell::new(MaybeUninit::uninit())).collect(),
        mask: n - 1,
        head: AtomicUsize::new(0),
        tail: AtomicUsize::new(0),
        dropped: AtomicU64::new(0),
        waiter: std::sync::Mutex::new(None),
        waiting: AtomicBool::new(false),
        closed: AtomicBool::new(false),
    });
    (Producer { ring: ring.clone() }, Consumer { ring })
}

pub struct Producer<T> {
    ring: Arc<Ring<T>>,
}

impl<T> Drop for Producer<T> {
    fn drop(&mut self) {
        self.ring.closed.store(true, Ordering::SeqCst);
        if let Some(th) = self.ring.waiter.lock().unwrap_or_else(|e| e.into_inner()).as_ref() {
            th.unpark();
        }
    }
}

pub struct Consumer<T> {
    ring: Arc<Ring<T>>,
}

impl<T: Send> Producer<T> {
    /// Append `v`; when the ring is full, `v` is returned and counted in [`Producer::dropped`].
    pub fn push(&self, v: T) -> Result<(), T> {
        let r = &*self.ring;
        let t = r.tail.load(Ordering::Relaxed);
        if t.wrapping_sub(r.head.load(Ordering::Acquire)) > r.mask {
            r.dropped.fetch_add(1, Ordering::Relaxed);
            return Err(v);
        }
        // SAFETY: slot `t` is free (consumer is behind `t`) and only this producer writes it.
        unsafe { (*r.slots[t & r.mask].get()).write(v) };
        r.tail.store(t.wrapping_add(1), Ordering::Release);
        // store tail / load waiting vs. store waiting / load tail (recv): no lost wake-up
        std::sync::atomic::fence(Ordering::SeqCst);
        if r.waiting.load(Ordering::SeqCst) {
            if let Some(th) = r.waiter.lock().unwrap_or_else(|e| e.into_inner()).as_ref() {
                th.unpark();
            }
        }
        Ok(())
    }

    /// Pushes refused because the ring was full.
    #[cfg_attr(not(test), allow(dead_code))] // audio / telemetry streams
    pub fn dropped(&self) -> u64 {
        self.ring.dropped.load(Ordering::Relaxed)
    }
}

impl<T: Send> Consumer<T> {
    /// Oldest element, if any.
    pub fn pop(&self) -> Option<T> {
        let r = &*self.ring;
        let h = r.head.load(Ordering::Relaxed);
        if h == r.tail.load(Ordering::Acquire) {
            return None;
        }
        // SAFETY: slot `h` was published by the producer (tail > h) and is read exactly once.
        let v = unsafe { (*r.slots[h & r.mask].get()).assume_init_read() };
        r.head.store(h.wrapping_add(1), Ordering::Release);
        Some(v)
    }

    /// Block until an element arrives or the producer is dropped: spin for `spin`, then park.
    pub fn recv_opt(&self, spin: Duration) -> Option<T> {
        let r = &*self.ring;
        let t0 = Instant::now();
        loop {
            if let Some(v) = self.pop() {
                return Some(v);
            }
            if r.closed.load(Ordering::Acquire) {
                return self.pop();
            }
            if t0.elapsed() >= spin {
                break;
            }
            std::hint::spin_loop();
        }
        *r.waiter.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::thread::current());
        loop {
            r.waiting.store(true, Ordering::SeqCst);
            std::sync::atomic::fence(Ordering::SeqCst);
            if let Some(v) = self.pop() {
                r.waiting.store(false, Ordering::SeqCst);
                return Some(v);
            }
            if r.closed.load(Ordering::SeqCst) {
                r.waiting.store(false, Ordering::SeqCst);
                return self.pop();
            }
            std::thread::park();
        }
    }

    /// Block until an element arrives: spin for `spin`, then park (woken by the producer's push).
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn recv(&self, spin: Duration) -> T {
        self.recv_opt(spin).expect("SPSC producer disconnected")
    }

    /// Elements currently queued.
    #[cfg_attr(not(test), allow(dead_code))] // audio / telemetry streams
    pub fn len(&self) -> usize {
        let r = &*self.ring;
        r.tail.load(Ordering::Acquire).wrapping_sub(r.head.load(Ordering::Relaxed))
    }

    /// Whether no elements are currently queued.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

// ---------------------------------------------------------------------------------------------
// Pacer
// ---------------------------------------------------------------------------------------------

/// Overload policy of [`Pacer`]: the simulation never skips or shortens a step; when the host falls
/// further than this behind the wall clock, the pacer re-anchors (counted as an overrun, the time is
/// added to `slip_s`) instead of running faster than real time afterwards to catch up.
pub const MAX_LAG_S: f64 = 0.1;

/// Wall-clock pacing of simulated time at `speed` × real time (1.0 = real time; < 1 slow motion,
/// only when the user selects it).
#[derive(Clone, Debug)]
pub struct Pacer {
    pub speed: f64,
    anchor_wall: Option<Instant>,
    anchor_sim: f64,
    /// Simulated time since the start.
    pub sim_s: f64,
    /// Wall time behind schedule now (s, ≥ 0).
    pub lag_s: f64,
    /// Lag given up by re-anchoring after overruns (s).
    pub slip_s: f64,
    pub overruns: u64,
    /// Realised speed over the last window (simulated / wall).
    pub rtf: f64,
    win_wall: Option<Instant>,
    win_sim: f64,
}

impl Pacer {
    pub fn new(speed: f64) -> Self {
        Self { speed, anchor_wall: None, anchor_sim: 0.0, sim_s: 0.0, lag_s: 0.0, slip_s: 0.0, overruns: 0, rtf: 0.0, win_wall: None, win_sim: 0.0 }
    }

    /// Account `sim_dt` of simulated time finished at wall time `now`; returns how long to sleep before
    /// the next step is due.
    pub fn after_step(&mut self, sim_dt: f64, now: Instant) -> Duration {
        let anchor = *self.anchor_wall.get_or_insert(now);
        self.sim_s += sim_dt;
        let due = (self.sim_s - self.anchor_sim) / self.speed.max(1e-3);
        let wall = now.duration_since(anchor).as_secs_f64();
        let ahead = due - wall;
        self.lag_s = (-ahead).max(0.0);
        if self.lag_s > MAX_LAG_S {
            self.overruns += 1;
            self.slip_s += self.lag_s;
            self.anchor_wall = Some(now);
            self.anchor_sim = self.sim_s;
        }
        let w = *self.win_wall.get_or_insert(now);
        let ws = now.duration_since(w).as_secs_f64();
        if ws >= 0.5 {
            self.rtf = (self.sim_s - self.win_sim) / ws;
            self.win_wall = Some(now);
            self.win_sim = self.sim_s;
        }
        Duration::from_secs_f64(ahead.max(0.0))
    }

    /// Forget the schedule (after a pause or a speed change): the next step starts a new anchor.
    pub fn rebase(&mut self) {
        self.anchor_wall = None;
        self.anchor_sim = self.sim_s;
        self.win_wall = None;
        self.win_sim = self.sim_s;
        self.lag_s = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spsc_fifo_full_and_cross_thread() {
        let (p, c) = spsc::<u32>(4);
        for i in 0..4 {
            p.push(i).unwrap();
        }
        assert_eq!(p.push(9), Err(9));
        assert_eq!(p.dropped(), 1);
        assert_eq!((c.pop(), c.len()), (Some(0), 3));
        let (p, c) = spsc::<u64>(8);
        let h = std::thread::spawn(move || (0..100_000u64).for_each(|i| while p.push(i).is_err() {}));
        let mut sum = 0;
        for i in 0..100_000u64 {
            let v = c.recv(Duration::from_micros(50));
            assert_eq!(v, i);
            sum += v;
        }
        h.join().unwrap();
        assert_eq!(sum, 100_000 * 99_999 / 2);
    }

    /// Lock-step handoff cost (docs/PERFORMANCE.md): round trips main → worker → main through two rings,
    /// the pattern of one co-simulation window (`cargo test --release -p simemu-cli spsc_round_trip --
    /// --ignored --nocapture`). With `work_us` of work on each side per trip, the partner usually arrives
    /// while the other spins; without work it measures the bare handoff.
    #[test]
    #[ignore]
    fn spsc_round_trip_latency() {
        for work_us in [0u64, 500] {
            let (to_w, from_m) = spsc::<u64>(2);
            let (to_m, from_w) = spsc::<u64>(2);
            let spin = Duration::from_micros(300);
            let busy = move |us: u64| {
                let t = Instant::now();
                while t.elapsed() < Duration::from_micros(us) {
                    std::hint::spin_loop();
                }
            };
            let h = std::thread::spawn(move || {
                set_thread_role(ThreadRole::Realtime);
                while let Some(v) = Some(from_m.recv(spin)).filter(|&v| v != u64::MAX) {
                    busy(work_us);
                    let _ = to_m.push(v);
                }
            });
            set_thread_role(ThreadRole::Realtime);
            let n = if work_us == 0 { 100_000 } else { 2_000 };
            let t0 = Instant::now();
            for i in 0..n {
                let _ = to_w.push(i);
                busy(work_us);
                assert_eq!(from_w.recv(spin), i);
            }
            let per = t0.elapsed().as_secs_f64() / n as f64 - work_us as f64 * 1e-6;
            let _ = to_w.push(u64::MAX);
            h.join().unwrap();
            println!("lock-step round trip with {work_us} us work per side: overhead {:.2} us", per * 1e6);
        }
    }

    #[test]
    fn pacer_sleeps_when_ahead_and_reanchors_after_overrun() {
        let t0 = Instant::now();
        let mut p = Pacer::new(1.0);
        p.after_step(0.0, t0);
        // 10 ms simulated in 4 ms: sleep 6 ms
        let s = p.after_step(0.010, t0 + Duration::from_millis(4));
        assert!((s.as_secs_f64() - 0.006).abs() < 1e-9);
        // 10 ms more done 50 ms late: no sleep, lag reported, no re-anchor yet
        let s = p.after_step(0.010, t0 + Duration::from_millis(70));
        assert_eq!(s, Duration::ZERO);
        assert!((p.lag_s - 0.050).abs() < 1e-9 && p.overruns == 0);
        // far behind: re-anchor, the lag is recorded as slip (no catch-up burst afterwards)
        p.after_step(0.010, t0 + Duration::from_millis(300));
        assert_eq!(p.overruns, 1);
        assert!((p.slip_s - 0.27).abs() < 1e-9);
        let s = p.after_step(0.010, t0 + Duration::from_millis(302));
        assert!((s.as_secs_f64() - 0.008).abs() < 1e-9);
        // slow motion 0.5x: 10 ms simulated is due after 20 ms
        let mut p = Pacer::new(0.5);
        p.after_step(0.0, t0);
        assert!((p.after_step(0.010, t0).as_secs_f64() - 0.020).abs() < 1e-9);
    }
}
