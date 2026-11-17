//! Uniformly partitioned overlap-save convolution (UPOLS: F. Wefers, "Partitioned convolution algorithms for
//! real-time auralization", PhD thesis, RWTH Aachen 2015, ch. 5; W. G. Gardner, "Efficient convolution without
//! input-output delay", J. Audio Eng. Soc. 43(3), 1995) with click-free impulse-response exchange: the block
//! after `set_ir` is computed with the old and the new filter and cross-faded. No allocation after `new`.
//! The spectral multiply-accumulate runs on split re/im `f32` slices (auto-vectorised).

use crate::fft::Fft;

pub const BLOCK: usize = 256;
const N: usize = 2 * BLOCK;
const BINS: usize = BLOCK + 1;

pub struct Convolver {
    fft: Fft,
    parts: usize,
    h_re: [Vec<f32>; 2],
    h_im: [Vec<f32>; 2],
    cur: usize,
    pending: bool,
    empty: bool,
    fdl_re: Vec<f32>,
    fdl_im: Vec<f32>,
    pos: usize,
    inbuf: [f32; N],
    a_re: [f32; N],
    a_im: [f32; N],
    s_re: [f32; N],
    s_im: [f32; N],
    old: [f32; BLOCK],
}

impl Convolver {
    /// Filter of up to `max_len` taps.
    pub fn new(max_len: usize) -> Self {
        let parts = max_len.div_ceil(BLOCK).max(1);
        let z = || vec![0.0f32; parts * BINS];
        Self {
            fft: Fft::new(N),
            parts,
            h_re: [z(), z()],
            h_im: [z(), z()],
            cur: 0,
            pending: false,
            empty: true,
            fdl_re: z(),
            fdl_im: z(),
            pos: 0,
            inbuf: [0.0; N],
            a_re: [0.0; N],
            a_im: [0.0; N],
            s_re: [0.0; N],
            s_im: [0.0; N],
            old: [0.0; BLOCK],
        }
    }

    pub fn max_len(&self) -> usize {
        self.parts * BLOCK
    }

    /// Loads a new impulse response (truncated to `max_len`); takes effect with a one-block cross-fade.
    pub fn set_ir(&mut self, ir: &[f32]) {
        let set = if self.empty { self.cur } else { 1 - self.cur };
        for p in 0..self.parts {
            self.s_re.fill(0.0);
            self.s_im.fill(0.0);
            let lo = (p * BLOCK).min(ir.len());
            let hi = ((p + 1) * BLOCK).min(ir.len());
            self.s_re[..hi - lo].copy_from_slice(&ir[lo..hi]);
            self.fft.forward(&mut self.s_re, &mut self.s_im);
            self.h_re[set][p * BINS..(p + 1) * BINS].copy_from_slice(&self.s_re[..BINS]);
            self.h_im[set][p * BINS..(p + 1) * BINS].copy_from_slice(&self.s_im[..BINS]);
        }
        self.pending = !self.empty;
        self.empty = false;
    }

    fn mac(&mut self, set: usize) -> [f32; BLOCK] {
        self.a_re[..BINS].fill(0.0);
        self.a_im[..BINS].fill(0.0);
        for p in 0..self.parts {
            let slot = (self.pos + self.parts - p) % self.parts;
            let (xr, xi) = (&self.fdl_re[slot * BINS..(slot + 1) * BINS], &self.fdl_im[slot * BINS..(slot + 1) * BINS]);
            let (hr, hi) = (&self.h_re[set][p * BINS..(p + 1) * BINS], &self.h_im[set][p * BINS..(p + 1) * BINS]);
            let (ar, ai) = (&mut self.a_re[..BINS], &mut self.a_im[..BINS]);
            for i in 0..BINS {
                ar[i] += xr[i] * hr[i] - xi[i] * hi[i];
                ai[i] += xr[i] * hi[i] + xi[i] * hr[i];
            }
        }
        // Hermitian completion (real input and filter).
        for k in 1..BLOCK {
            self.a_re[N - k] = self.a_re[k];
            self.a_im[N - k] = -self.a_im[k];
        }
        self.fft.inverse(&mut self.a_re, &mut self.a_im);
        let mut y = [0.0; BLOCK];
        y.copy_from_slice(&self.a_re[BLOCK..]);
        y
    }

    pub fn process(&mut self, x: &[f32], y: &mut [f32]) {
        assert!(x.len() >= BLOCK && y.len() >= BLOCK, "Convolver block shorter than BLOCK");
        self.inbuf.copy_within(BLOCK.., 0);
        self.inbuf[BLOCK..].copy_from_slice(&x[..BLOCK]);
        if self.empty {
            y[..BLOCK].fill(0.0);
            return;
        }
        self.s_re.copy_from_slice(&self.inbuf);
        self.s_im.fill(0.0);
        self.fft.forward(&mut self.s_re, &mut self.s_im);
        self.pos = (self.pos + 1) % self.parts;
        let p = self.pos;
        self.fdl_re[p * BINS..(p + 1) * BINS].copy_from_slice(&self.s_re[..BINS]);
        self.fdl_im[p * BINS..(p + 1) * BINS].copy_from_slice(&self.s_im[..BINS]);
        let new = self.mac(if self.pending { 1 - self.cur } else { self.cur });
        if self.pending {
            self.old = self.mac(self.cur);
            for i in 0..BLOCK {
                let w = (i as f32 + 0.5) / BLOCK as f32;
                y[i] = self.old[i] * (1.0 - w) + new[i] * w;
            }
            self.cur = 1 - self.cur;
            self.pending = false;
        } else {
            y[..BLOCK].copy_from_slice(&new);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Matches direct convolution for an impulse response longer than one partition.
    #[test]
    fn matches_direct_convolution() {
        let ir: Vec<f32> = (0..700).map(|i| ((i * 37 % 101) as f32 - 50.0) / 50.0 * (-(i as f32) / 200.0).exp()).collect();
        let x: Vec<f32> = (0..BLOCK * 6).map(|i| ((i * 13 % 29) as f32 - 14.0) / 14.0).collect();
        let mut c = Convolver::new(1024);
        c.set_ir(&ir);
        let mut y = vec![0.0; x.len()];
        for b in 0..6 {
            c.process(&x[b * BLOCK..(b + 1) * BLOCK], &mut y[b * BLOCK..(b + 1) * BLOCK]);
        }
        for n in 0..x.len() {
            let d: f32 = (0..=n.min(ir.len() - 1)).map(|k| ir[k] * x[n - k]).sum();
            assert!((d - y[n]).abs() < 2e-3, "n={n}: {d} vs {}", y[n]);
        }
    }
}
