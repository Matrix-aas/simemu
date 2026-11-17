//! Physical sound of the virtual car: every source signal comes from the plant (`simemu-physics`) state —
//! valve and turbine mass flows, cylinder pressure, knock-mode amplitude, valve/injector/pump impacts,
//! starter, alternator, fan, clutch slip, gear meshes, mechatronic pump — and is propagated through
//! physical models of the exhaust/intake ducts, the engine and gearbox structures and the listener paths.
//! No samples, no recorded clips, no scripted events. See `docs/ACOUSTICS.md`.

pub mod conv;
pub mod duct;
pub mod fft;
pub mod ring;
pub mod rt;
pub mod synth;
pub mod tap;
pub mod wav;

pub use rt::{AudioRt, Shared};
pub use synth::{BlockOut, Mix, Params, Synth, CYL_RESOLVED, FS, LISTENER_NAMES, N_LIST, N_SRC, SRC_NAMES};
pub use tap::{Record, Tap};

#[cfg(test)]
mod tests;
