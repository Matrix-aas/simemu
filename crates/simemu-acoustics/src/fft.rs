//! Iterative radix-2 complex FFT on split (structure-of-arrays) real/imaginary buffers. The butterflies run
//! over contiguous `f32` slices so the compiler auto-vectorises them (NEON / SSE / AVX).

pub struct Fft {
    pub n: usize,
    rev: Vec<u32>,
    /// Twiddles per stage, contiguous: stage with half-length `h` starts at offset `h − 1`.
    tw_re: Vec<f32>,
    tw_im: Vec<f32>,
}

impl Fft {
    pub fn new(n: usize) -> Self {
        assert!(n.is_power_of_two() && n >= 2);
        let bits = n.trailing_zeros();
        let rev = (0..n as u32).map(|i| i.reverse_bits() >> (32 - bits)).collect();
        let (mut tw_re, mut tw_im) = (Vec::with_capacity(n), Vec::with_capacity(n));
        let mut half = 1;
        while half < n {
            for k in 0..half {
                let a = -std::f64::consts::PI * k as f64 / half as f64;
                tw_re.push(a.cos() as f32);
                tw_im.push(a.sin() as f32);
            }
            half *= 2;
        }
        Self { n, rev, tw_re, tw_im }
    }

    /// In-place forward transform (e^{−jωt} kernel, no scaling).
    pub fn forward(&self, re: &mut [f32], im: &mut [f32]) {
        let n = self.n;
        assert!(re.len() >= n && im.len() >= n, "FFT buffer shorter than transform length");
        let (re, im) = (&mut re[..n], &mut im[..n]);
        for (i, &rev_i) in self.rev.iter().enumerate().take(n) {
            let j = rev_i as usize;
            if j > i {
                re.swap(i, j);
                im.swap(i, j);
            }
        }
        let mut half = 1;
        while half < n {
            let (wr, wi) = (&self.tw_re[half - 1..2 * half - 1], &self.tw_im[half - 1..2 * half - 1]);
            for start in (0..n).step_by(2 * half) {
                let (lo_r, hi_r) = re[start..start + 2 * half].split_at_mut(half);
                let (lo_i, hi_i) = im[start..start + 2 * half].split_at_mut(half);
                for k in 0..half {
                    let xr = hi_r[k] * wr[k] - hi_i[k] * wi[k];
                    let xi = hi_r[k] * wi[k] + hi_i[k] * wr[k];
                    hi_r[k] = lo_r[k] - xr;
                    hi_i[k] = lo_i[k] - xi;
                    lo_r[k] += xr;
                    lo_i[k] += xi;
                }
            }
            half *= 2;
        }
    }

    /// In-place inverse transform, scaled by 1/n.
    pub fn inverse(&self, re: &mut [f32], im: &mut [f32]) {
        self.forward(im, re);
        let s = 1.0 / self.n as f32;
        re[..self.n].iter_mut().for_each(|x| *x *= s);
        im[..self.n].iter_mut().for_each(|x| *x *= s);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_and_single_bin() {
        let f = Fft::new(64);
        let mut re: Vec<f32> = (0..64).map(|i| ((i * 7) % 13) as f32 - 6.0).collect();
        let mut im = vec![0.0; 64];
        let orig = re.clone();
        f.forward(&mut re, &mut im);
        f.inverse(&mut re, &mut im);
        assert!(re.iter().zip(&orig).all(|(a, b)| (a - b).abs() < 1e-4));
        let mut re: Vec<f32> = (0..64).map(|i| (2.0 * std::f32::consts::PI * 5.0 * i as f32 / 64.0).cos()).collect();
        let mut im = vec![0.0; 64];
        f.forward(&mut re, &mut im);
        assert!((re[5] - 32.0).abs() < 1e-3 && (re[59] - 32.0).abs() < 1e-3 && re[6].abs() < 1e-3);
    }
}
