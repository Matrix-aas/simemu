//! IEEE-float WAV writer (calibrated pressure, no clipping).

use std::io::{self, Write};

fn invalid(msg: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, msg)
}

/// Writes `samples` (interleaved if `channels` > 1) as 32-bit float WAV.
pub fn write_f32(path: &std::path::Path, rate: u32, channels: u16, samples: &[f32]) -> io::Result<()> {
    if rate == 0 || channels == 0 {
        return Err(invalid("WAV sample rate and channel count must be non-zero"));
    }
    if !samples.len().is_multiple_of(channels as usize) {
        return Err(invalid("WAV sample count is not a multiple of channel count"));
    }
    let data = samples
        .len()
        .checked_mul(4)
        .and_then(|n| u32::try_from(n).ok())
        .ok_or_else(|| invalid("WAV data chunk exceeds 32-bit RIFF limit"))?;
    let riff_len = 36u32.checked_add(data).ok_or_else(|| invalid("WAV RIFF size overflow"))?;
    let block_align = channels.checked_mul(4).ok_or_else(|| invalid("WAV block_align overflow"))?;
    let byte_rate = rate
        .checked_mul(block_align as u32)
        .ok_or_else(|| invalid("WAV byte_rate overflow"))?;

    let mut w = io::BufWriter::new(std::fs::File::create(path)?);
    w.write_all(b"RIFF")?;
    w.write_all(&riff_len.to_le_bytes())?;
    w.write_all(b"WAVEfmt ")?;
    w.write_all(&16u32.to_le_bytes())?;
    w.write_all(&3u16.to_le_bytes())?; // IEEE float
    w.write_all(&channels.to_le_bytes())?;
    w.write_all(&rate.to_le_bytes())?;
    w.write_all(&byte_rate.to_le_bytes())?;
    w.write_all(&block_align.to_le_bytes())?;
    w.write_all(&32u16.to_le_bytes())?;
    w.write_all(b"data")?;
    w.write_all(&data.to_le_bytes())?;
    for s in samples {
        w.write_all(&s.to_le_bytes())?;
    }
    w.flush()
}
