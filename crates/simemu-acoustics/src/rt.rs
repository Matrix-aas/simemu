//! Real-time wiring: plant thread → lock-free ring → synthesizer thread → lock-free PCM ring → streamer.
//!
//! The plant thread only drains the probe into the record ring ([`AudioRt::feed`]); all synthesis runs on a
//! dedicated thread that allocates nothing and takes no lock in its loop. Mixer settings and meters cross
//! threads as atomics. If the synthesizer falls behind, records are dropped at the ring (counted), the plant
//! is never blocked.

use crate::ring::{ring, Consumer, Producer};
use crate::synth::{Mix, Params, Synth, BlockOut, FS, N_LIST, N_SRC};
use crate::tap::{Record, Tap};
use simemu_physics::{PlantActuators, VirtualPlant};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering::{Acquire, Relaxed, Release}};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const REC_RING: usize = 1 << 15;
const PCM_RING: usize = 1 << 16;

fn f32a(v: f32) -> AtomicU32 {
    AtomicU32::new(v.to_bits())
}

pub struct Shared {
    /// Probe and synthesis enabled (a listener is connected or a render runs).
    pub active: AtomicBool,
    /// Listener position streamed as PCM (0 exterior, 1 cabin, 2 engine bay, 3 stethoscope).
    pub listener: AtomicUsize,
    gain: [AtomicU32; N_SRC],
    cyl: [AtomicU32; 4],
    master: AtomicU32,
    /// Mean-square pressure per listener and source of the last block, Pa² (f32 bits).
    level: [[AtomicU32; N_SRC]; N_LIST],
    /// Plant time of the last rendered block, s (f64 bits).
    pub sim_t: AtomicU64,
    pub dropped: AtomicU64,
    /// Synthesizer thread busy time and audio time produced, ns (CPU load = busy / produced).
    pub busy_ns: AtomicU64,
    pub produced_ns: AtomicU64,
    /// Streaming-client generation: a new client increments it, the previous one sees the change and leaves.
    pub client_gen: AtomicU64,
    shutdown: AtomicBool,
    /// PCM of the selected listener, Pa, 48 kHz. Taken by the one streaming client.
    pub pcm: Mutex<Option<Consumer<f32>>>,
    rec: Mutex<Option<(Consumer<Record>, Producer<f32>)>>,
}

impl Shared {
    pub fn set_mix(&self, m: &Mix) {
        for (a, v) in self.gain.iter().zip(m.gain) {
            a.store(v.to_bits(), Relaxed);
        }
        for (a, v) in self.cyl.iter().zip(m.cyl) {
            a.store(v.to_bits(), Relaxed);
        }
        self.master.store(m.master.to_bits(), Relaxed);
    }

    pub fn mix(&self) -> Mix {
        Mix {
            gain: std::array::from_fn(|i| f32::from_bits(self.gain[i].load(Relaxed))),
            cyl: std::array::from_fn(|i| f32::from_bits(self.cyl[i].load(Relaxed))),
            master: f32::from_bits(self.master.load(Relaxed)),
        }
    }

    pub fn levels(&self, listener: usize) -> [f32; N_SRC] {
        std::array::from_fn(|i| f32::from_bits(self.level[listener.min(N_LIST - 1)][i].load(Relaxed)))
    }

    /// Synthesizer CPU load as a fraction of one core at real time (busy / produced audio time).
    pub fn load(&self) -> f64 {
        self.busy_ns.load(Relaxed) as f64 / self.produced_ns.load(Relaxed).max(1) as f64
    }

    fn publish(&self, b: &BlockOut) {
        for l in 0..N_LIST {
            for s in 0..N_SRC {
                self.level[l][s].store(b.level[l][s].to_bits(), Relaxed);
            }
        }
        self.sim_t.store(b.t0.to_bits(), Relaxed);
    }
}

pub struct AudioRt {
    pub shared: Arc<Shared>,
    tap: Tap,
    tx: Producer<Record>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Default for AudioRt {
    fn default() -> Self {
        Self::new()
    }
}

impl AudioRt {
    pub fn new() -> Self {
        let (tx, rx) = ring::<Record>(REC_RING);
        let (pcm_tx, pcm_rx) = ring::<f32>(PCM_RING);
        let shared = Arc::new(Shared {
            active: AtomicBool::new(false),
            listener: AtomicUsize::new(0),
            gain: std::array::from_fn(|_| f32a(1.0)),
            cyl: std::array::from_fn(|_| f32a(1.0)),
            master: f32a(1.0),
            level: std::array::from_fn(|_| std::array::from_fn(|_| f32a(0.0))),
            sim_t: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
            busy_ns: AtomicU64::new(0),
            produced_ns: AtomicU64::new(0),
            client_gen: AtomicU64::new(0),
            shutdown: AtomicBool::new(false),
            pcm: Mutex::new(Some(pcm_rx)),
            rec: Mutex::new(Some((rx, pcm_tx))),
        });
        Self { shared, tap: Tap::default(), tx, thread: None }
    }

    /// Takes the record consumer for an offline (inline, deterministic) synthesizer; no thread is started then.
    pub fn take_records(&mut self) -> Option<Consumer<Record>> {
        self.shared.rec.lock().ok().and_then(|mut g| g.take()).map(|(rx, _)| rx)
    }

    /// Plant thread, after every `VirtualPlant::step`: moves the probe records to the synthesizer thread.
    pub fn feed(&mut self, plant: &mut VirtualPlant, act: &PlantActuators) {
        if !self.shared.active.load(Relaxed) {
            if plant.engine.probe.enabled {
                Tap::disable(plant);
            }
            return;
        }
        if self.thread.is_none() {
            if let Some((rx, pcm_tx)) = self.shared.rec.lock().ok().and_then(|mut g| g.take()) {
                let sh = self.shared.clone();
                self.thread = std::thread::Builder::new().name("simemu-audio".into()).spawn(move || run(sh, rx, pcm_tx)).ok();
            }
        }
        let tx = &mut self.tx;
        let d = self.tap.collect(plant, act, &mut |r| tx.push(r));
        if d > 0 {
            self.shared.dropped.fetch_add(d as u64, Relaxed);
        }
    }
}

impl Drop for AudioRt {
    fn drop(&mut self) {
        self.shared.shutdown.store(true, Release);
        if let Some(h) = self.thread.take() {
            h.thread().unpark();
            let _ = h.join();
        }
    }
}

/// Synthesizer thread: no allocation, no lock inside the loop.
fn run(sh: Arc<Shared>, mut rx: Consumer<Record>, mut pcm: Producer<f32>) {
    let mut synth = Synth::new(Params::default());
    let block_ns = (crate::conv::BLOCK as f64 / FS * 1e9) as u64;
    loop {
        if sh.shutdown.load(Acquire) {
            return;
        }
        if rx.is_empty() {
            if Arc::strong_count(&sh) == 1 || rx.is_disconnected() {
                return; // the session (or producer) is gone
            }
            let idle = if sh.active.load(Relaxed) { 500 } else { 20_000 };
            std::thread::park_timeout(Duration::from_micros(idle));
            continue;
        }
        synth.mix = sh.mix();
        let listener = sh.listener.load(Relaxed).min(N_LIST - 1);
        let t0 = Instant::now();
        let mut blocks = 0u64;
        for _ in 0..4096 {
            let Some(r) = rx.pop() else { break };
            synth.push(&r, &mut |b: &BlockOut| {
                pcm.push_slice(&b.out[listener]);
                sh.publish(b);
                blocks += 1;
            });
        }
        sh.busy_ns.fetch_add(t0.elapsed().as_nanos() as u64, Relaxed);
        sh.produced_ns.fetch_add(blocks * block_ns, Relaxed);
    }
}
