//! High-speed native loader and live calibration engine for Continental SIMOS18.1 DAMOS
//! (`SC8S50.json`, 33,140 parameters; `SC8O30`, `SC8O40`, `SC8O20`, `SC8H83`, and patch overlays).
//!
//! ## Black-Box Contract & Zero-Patch Guarantee
//! - **Calibration Codec ([`DamosDatabase`])**: Decodes and encodes scalar, 1D curve, and 2D/3D map
//!   parameters (`UBYTE`, `BOOLEAN`, `SBYTE`, `UWORD`, `SWORD`, `ULONG`, `SLONG`, `FLOAT32`) using
//!   linear (`raw * scale + offset`) or rational (`(a*X + b) / (c + d*X)`) conversions. Live writes
//!   ([`DamosDatabase::write_matrix`]) are strictly confined to the 512 KB `CAL` data sector
//!   (`file 0x200000..0x280000`, CPU `0x80800000..0x80880000`) and never touch code sectors.
//! - **Checksums ([`checksum`])**: Verifies and recomputes CBOOT MSB-first CRC-32 block headers and
//!   64-bit ECM3 background ROM sums in the exact order a flashing tool applies them.
//! - **Post-Programming Completion ([`post_programming`])**: Writes missing CBOOT block-valid markers
//!   and refreshes stale data checksums for container/partial reads (SIMOS18.1 and Temic DQ250),
//!   returning an explicit audit log of every modified data word.
//! - **Curated Map Catalog ([`curated`])**: Bilingual (`RU` / `EN`) reference catalog of core EA888
//!   Gen3 tuning maps cross-referenced to `Simos18 Funktionsrahmen` chapters.

pub mod checksum;
pub mod curated;
pub mod frf;
pub mod i18n;
pub mod post_programming;
pub mod types;

pub use checksum::{fix_checksums, verify_checksums, ChecksumEntry, ChecksumReport};
pub use curated::{CuratedTunerMap, CURATED_TUNER_MAPS};
pub use frf::{
    is_frf_bytes, is_frf_path, is_odx_path, peek_frf_cal_prefix, peek_odx_cal_prefix,
    unpack_frf_into_image, unpack_odx_into_image, FrfUnpackReport,
};
pub use i18n::{is_ru, lang, set_lang, tr, Lang};
pub use post_programming::{apply_post_programming, apply_post_programming_dq250, dq250_crc_register};
pub use types::{
    DamosMatrixData, DamosParameter, RationalFormula, RawDamosAxis, RawDamosFile, RawDamosParam,
};

use std::collections::HashMap;
use std::fs;
use std::path::Path;

/// Start of the 512 KB `CAL` block inside a 4 MB SIMOS18.1 PFlash image (`CPU 0x80800000`).
pub const CAL_FLASH_OFFSET: usize = 0x20_0000;
/// Length of the `CAL` block inside a 4 MB SIMOS18.1 PFlash image (512 KB).
pub const CAL_FLASH_LEN: usize = 0x08_0000;
/// Exclusive end offset of the `CAL` block (`0x280000`).
pub const CAL_FLASH_END: usize = CAL_FLASH_OFFSET + CAL_FLASH_LEN;

/// Upper bound on `rows * cols` accepted when loading untrusted DAMOS JSON definitions.
const MAX_MATRIX_CELLS: usize = 65_536;

pub struct DamosDatabase {
    pub cal_id: String,
    pub base_address: u32,
    pub parameters: Vec<DamosParameter>,
    /// Lower-cased id → index. A plain name always maps to its FIRST loaded occurrence;
    /// duplicates are reachable by their unique ids (`name [n]`).
    pub by_name: HashMap<String, usize>,
}

fn read_raw(flash: &[u8], off: usize, len: usize, signed: bool, is_float: bool) -> Option<f64> {
    if !matches!(len, 1 | 2 | 4) || (is_float && len != 4) {
        return None;
    }
    let bytes = flash.get(off..off.checked_add(len)?)?;
    let mut w = [0u8; 4];
    w[..len].copy_from_slice(bytes);
    let u = u32::from_le_bytes(w);
    let v = match (len, signed, is_float) {
        (4, _, true) => f32::from_bits(u) as f64,
        (1, true, _) => u as u8 as i8 as f64,
        (2, true, _) => u as u16 as i16 as f64,
        (4, true, _) => u as i32 as f64,
        _ => u as f64,
    };
    v.is_finite().then_some(v)
}

fn validate_formula(f: &RationalFormula) -> Result<(), &'static str> {
    if !f.a.is_finite() || !f.b.is_finite() || !f.c.is_finite() || !f.d.is_finite() {
        return Err("non-finite rational formula coefficients");
    }
    if f.c == 0.0 && f.d == 0.0 {
        return Err("zero denominator in rational formula");
    }
    let det = f.a * f.d - f.b * f.c;
    if !det.is_finite() || det == 0.0 {
        return Err("degenerate (non-invertible) rational formula");
    }
    Ok(())
}

fn to_phys(raw: f64, scale: f64, offset: f64, f: Option<&RationalFormula>) -> Option<f64> {
    let v = match f {
        Some(f) => {
            let denom = f.c + f.d * raw;
            if !denom.is_finite() || denom == 0.0 {
                return None;
            }
            (f.a * raw + f.b) / denom
        }
        None => raw * scale + offset,
    };
    v.is_finite().then_some(v)
}

fn to_raw_linear(phys: f64, scale: f64, offset: f64) -> f64 {
    (phys - offset) / scale
}

/// Evaluates the inverse of a validated [`RationalFormula`] on a pole-free interval `[lo_b, hi_b]`,
/// clamping physical inputs outside `f([lo_b, hi_b])` to the corresponding monotonic endpoint so
/// values crossing the horizontal asymptote `a / d` never invert to the wrong side of the pole.
fn to_raw_rational_on_branch(f: &RationalFormula, phys: f64, lo_b: f64, hi_b: f64) -> Option<f64> {
    let d_lo = f.c + f.d * lo_b;
    let d_hi = f.c + f.d * hi_b;
    if d_lo == 0.0 || d_hi == 0.0 || d_lo.is_sign_positive() != d_hi.is_sign_positive() {
        let denom = phys * f.d - f.a;
        if !denom.is_finite() || denom == 0.0 {
            return None;
        }
        let r = (f.b - phys * f.c) / denom;
        return r.is_finite().then_some(r);
    }
    let y_lo = (f.a * lo_b + f.b) / d_lo;
    let y_hi = (f.a * hi_b + f.b) / d_hi;
    if !y_lo.is_finite() || !y_hi.is_finite() {
        return None;
    }
    if y_lo <= y_hi {
        if phys <= y_lo {
            return Some(lo_b);
        }
        if phys >= y_hi {
            return Some(hi_b);
        }
    } else {
        if phys <= y_hi {
            return Some(hi_b);
        }
        if phys >= y_lo {
            return Some(lo_b);
        }
    }
    let denom = phys * f.d - f.a;
    if !denom.is_finite() || denom == 0.0 {
        return None;
    }
    let r = (f.b - phys * f.c) / denom;
    r.is_finite().then_some(r.clamp(lo_b, hi_b))
}

impl DamosParameter {
    fn cell_phys(&self, flash: &[u8], off: usize) -> Option<f64> {
        read_raw(flash, off, self.byte_len, self.signed, self.is_float)
            .and_then(|raw| to_phys(raw, self.scale, self.offset, self.formula.as_ref()))
    }

    fn cell_index(&self, r: usize, c: usize) -> Option<usize> {
        if self.column_dir {
            c.checked_mul(self.rows.max(1))?.checked_add(r)
        } else {
            r.checked_mul(self.cols.max(1))?.checked_add(c)
        }
    }

    /// Physical → raw cell bytes: round half away from zero, saturate to the definition's
    /// `[min, max]` and to the data type range. Only the first `byte_len` bytes are meaningful.
    fn encode_cell(&self, phys: f64) -> Result<[u8; 4], String> {
        if !phys.is_finite() {
            return Err(format!("{}: non-finite value {phys}", self.id));
        }
        if !matches!(self.byte_len, 1 | 2 | 4) || (self.is_float && self.byte_len != 4) {
            return Err(format!("{}: invalid byte width {}", self.id, self.byte_len));
        }
        if let Some(f) = &self.formula {
            validate_formula(f).map_err(|e| format!("{}: {e}", self.id))?;
        } else if !self.scale.is_finite() || self.scale == 0.0 || !self.offset.is_finite() {
            return Err(format!(
                "{}: invalid linear scaling (scale={}, offset={})",
                self.id, self.scale, self.offset
            ));
        }

        let limits = match (self.min, self.max) {
            (Some(lo), Some(hi)) if lo.is_finite() && hi.is_finite() && lo < hi => Some((lo, hi)),
            _ => None,
        };
        if self.is_float {
            let p = limits.map_or(phys, |(lo, hi)| phys.clamp(lo, hi));
            let raw = match &self.formula {
                Some(f) => {
                    let denom = p * f.d - f.a;
                    if !denom.is_finite() || denom == 0.0 {
                        return Err(format!("{}: cannot encode {phys}", self.id));
                    }
                    (f.b - p * f.c) / denom
                }
                None => to_raw_linear(p, self.scale, self.offset),
            };
            if !raw.is_finite() {
                return Err(format!("{}: cannot encode {phys}", self.id));
            }
            return Ok((raw.clamp(f32::MIN as f64, f32::MAX as f64) as f32).to_le_bytes());
        }

        let bits = 8 * self.byte_len as i32;
        let (tlo, thi) = if self.signed {
            (-(2f64.powi(bits - 1)), 2f64.powi(bits - 1) - 1.0)
        } else {
            (0.0, 2f64.powi(bits) - 1.0)
        };
        let (mut lo, mut hi) = (tlo, thi);

        let raw = if let Some(f) = &self.formula {
            if let Some((mn, mx)) = limits {
                if let (Some(a), Some(b)) = (
                    to_raw_rational_on_branch(f, mn, tlo, thi),
                    to_raw_rational_on_branch(f, mx, tlo, thi),
                ) {
                    let (l, h) = ((a.min(b) - 1e-3).ceil(), (a.max(b) + 1e-3).floor());
                    if l.is_finite() {
                        lo = lo.max(l);
                    }
                    if h.is_finite() {
                        hi = hi.min(h);
                    }
                    if lo > hi {
                        (lo, hi) = (tlo, thi);
                    }
                }
            }
            let p = limits.map_or(phys, |(mn, mx)| phys.clamp(mn, mx));
            to_raw_rational_on_branch(f, p, lo, hi)
                .ok_or_else(|| format!("{}: cannot encode {phys}", self.id))?
        } else {
            if let Some((mn, mx)) = limits {
                let (a, b) = (
                    to_raw_linear(mn, self.scale, self.offset),
                    to_raw_linear(mx, self.scale, self.offset),
                );
                // JSON limits are truncated decimals of exact raw multiples (e.g. ULONG max 0xFFFFFFFF
                // * 2^-22 = 1023.999999761581), so allow a sub-LSB tolerance before rounding inwards.
                let (l, h) = ((a.min(b) - 1e-3).ceil(), (a.max(b) + 1e-3).floor());
                if l.is_finite() {
                    lo = lo.max(l);
                }
                if h.is_finite() {
                    hi = hi.min(h);
                }
                if lo > hi {
                    (lo, hi) = (tlo, thi);
                }
            }
            let r = to_raw_linear(phys, self.scale, self.offset);
            if r.is_nan() {
                return Err(format!("{}: cannot encode {phys}", self.id));
            }
            r
        };

        // Two's complement truncation of the in-range integer gives the LE bytes for every width.
        Ok(((raw.round().clamp(lo, hi) as i64) as u32).to_le_bytes())
    }
}

impl DamosDatabase {
    /// `(byte_len, signed, is_float)` for a DAMOS data type name, `None` if unsupported.
    pub fn decode_type_spec(data_type: &str) -> Option<(usize, bool, bool)> {
        Some(match data_type {
            "UBYTE" | "BOOLEAN" => (1, false, false),
            "SBYTE" => (1, true, false),
            "UWORD" => (2, false, false),
            "SWORD" => (2, true, false),
            "ULONG" => (4, false, false),
            "SLONG" => (4, true, false),
            "FLOAT32" | "FLOAT32_IEEE" => (4, true, true),
            _ => return None,
        })
    }

    fn parse(path: &Path) -> Result<RawDamosFile, String> {
        let bytes = fs::read(path)
            .map_err(|e| format!("Failed to read DAMOS {}: {}", path.display(), e))?;
        serde_json::from_slice(&bytes)
            .map_err(|e| format!("Failed to parse DAMOS JSON {}: {}", path.display(), e))
    }

    pub fn load_from_json(path: &Path) -> Result<Self, String> {
        let raw = Self::parse(path)?;
        let mut db = Self {
            cal_id: raw.name.clone(),
            base_address: raw.base_address,
            parameters: Vec::with_capacity(raw.parameters.len()),
            by_name: HashMap::with_capacity(raw.parameters.len()),
        };
        db.ingest(raw)?;
        Ok(db)
    }

    /// Merges additional patch/MultiMap parameter definitions (e.g. `O30_Switch_Patch.29.33.V2.json`)
    /// into this `DamosDatabase`, assigning indexed aliases (`name [1]..[5]`) when multiple MultiMap slots share a name.
    pub fn merge_from_json(&mut self, path: &Path) -> Result<usize, String> {
        self.ingest(Self::parse(path)?)
    }

    /// The first occurrence of a name keeps the plain id; the n-th duplicate within a file becomes
    /// `name [n]` (a first occurrence clashing with an already loaded name becomes `name [1]`).
    fn ingest(&mut self, raw: RawDamosFile) -> Result<usize, String> {
        for p in &raw.parameters {
            let typed_axes = [&p.x_axis, &p.y_axis]
                .into_iter()
                .flatten()
                .filter(|a| a.address.is_some());
            for dt in std::iter::once(&p.data_type).chain(typed_axes.clone().map(|a| &a.data_type)) {
                if Self::decode_type_spec(dt).is_none() {
                    return Err(format!("{}: unsupported dataType '{}'", p.name, dt));
                }
            }
            for ax in typed_axes {
                if !ax.factor.is_finite() || !ax.offset.is_finite() {
                    return Err(format!("{}: non-finite axis factor/offset", p.name));
                }
            }
            if let Some(f) = &p.formula {
                validate_formula(f).map_err(|e| format!("{}: {e}", p.name))?;
            } else if !p.factor.is_finite() || p.factor == 0.0 || !p.offset.is_finite() {
                return Err(format!(
                    "{}: invalid factor/offset (factor={}, offset={})",
                    p.name, p.factor, p.offset
                ));
            }
            let rows = p.rows.unwrap_or(1).max(1);
            let cols = p.cols.unwrap_or(1).max(1);
            if rows.checked_mul(cols).is_none_or(|n| n > MAX_MATRIX_CELLS) {
                return Err(format!(
                    "{}: matrix dimensions {}x{} exceed maximum ({MAX_MATRIX_CELLS} cells)",
                    p.name, rows, cols
                ));
            }
            if raw.base_address.checked_add(p.address).is_none() {
                return Err(format!("{}: address overflow", p.name));
            }
        }

        let added = raw.parameters.len();
        let mut name_counts: HashMap<String, usize> = HashMap::new();
        for p in raw.parameters {
            let Some((byte_len, signed, is_float)) = Self::decode_type_spec(&p.data_type) else {
                return Err(format!("{}: unsupported dataType '{}'", p.name, p.data_type));
            };
            let file_offset = raw.base_address + p.address;
            let cal_offset = file_offset.saturating_sub(CAL_FLASH_OFFSET as u32);
            let cpu_address = match file_offset {
                0..=0x1F_FFFF => 0x8000_0000 + file_offset,
                0x20_0000..=0x3F_FFFF => 0x8080_0000 + (file_offset - 0x20_0000),
                _ => file_offset,
            };

            let base_key = p.name.to_lowercase();
            let count = name_counts.entry(base_key.clone()).or_insert(0);
            *count += 1;
            let unique_id = if *count == 1 && !self.by_name.contains_key(&base_key) {
                p.name.clone()
            } else {
                while self.by_name.contains_key(&format!("{base_key} [{}]", *count)) {
                    *count += 1;
                }
                format!("{} [{}]", p.name, *count)
            };

            let idx = self.parameters.len();
            self.by_name.entry(base_key).or_insert(idx);
            self.by_name.entry(unique_id.to_lowercase()).or_insert(idx);
            self.parameters.push(DamosParameter {
                id: unique_id,
                title: p.description,
                category: p.categories.first().cloned().unwrap_or_default(),
                kind: p.kind,
                data_type: p.data_type,
                unit: p.unit,
                cal_offset,
                file_offset,
                cpu_address,
                data_offset: p.data_offset,
                base_address: raw.base_address,
                byte_len,
                signed,
                is_float,
                scale: p.factor,
                offset: p.offset,
                min: p.min,
                max: p.max,
                formula: p.formula,
                rows: p.rows.unwrap_or(1).max(1),
                cols: p.cols.unwrap_or(1).max(1),
                column_dir: p.column_dir,
                x_axis: p.x_axis,
                y_axis: p.y_axis,
            });
        }
        Ok(added)
    }

    pub fn get_by_id(&self, id: &str) -> Option<&DamosParameter> {
        self.by_name
            .get(&id.to_lowercase())
            .and_then(|&idx| self.parameters.get(idx))
    }

    pub fn search(&self, query: &str, limit: usize) -> Vec<&DamosParameter> {
        let q = query.to_lowercase();
        self.parameters
            .iter()
            .filter(|p| {
                p.id.to_lowercase().contains(&q)
                    || p.title.to_lowercase().contains(&q)
                    || p.category.to_lowercase().contains(&q)
            })
            .take(limit)
            .collect()
    }

    /// Read the physical scaled value of a DAMOS parameter (first cell for maps/curves) from the 4MB Flash image.
    pub fn read_physical_value(&self, param: &DamosParameter, flash: &[u8]) -> Option<f64> {
        let off = (param.file_offset as usize).checked_add(param.data_offset)?;
        param.cell_phys(flash, off)
    }

    fn read_axis_values(
        base_address: u32,
        axis: &Option<RawDamosAxis>,
        count: usize,
        flash: &[u8],
    ) -> (String, Vec<f64>) {
        if let Some(ax) = axis {
            if let (Some(addr), Some((blen, signed, is_float))) =
                (ax.address, Self::decode_type_spec(&ax.data_type))
            {
                if let Some(base_off) = base_address
                    .checked_add(addr)
                    .and_then(|a| (a as usize).checked_add(ax.data_offset))
                {
                    let vals = (0..count)
                        .map(|i| {
                            i.checked_mul(blen)
                                .and_then(|d| base_off.checked_add(d))
                                .and_then(|off| read_raw(flash, off, blen, signed, is_float))
                                .map(|raw| raw * ax.factor + ax.offset)
                                .filter(|v| v.is_finite())
                                .unwrap_or(i as f64)
                        })
                        .collect();
                    return (ax.unit.clone(), vals);
                }
            }
        }
        ("-".to_string(), (0..count).map(|i| i as f64).collect())
    }

    /// Read full 1D/2D/3D matrix (`rows × cols`) with decoded physical X/Y axes from both current and stock Flash.
    /// Values are full precision; display rounding is the UI's job.
    pub fn read_matrix(
        &self,
        param: &DamosParameter,
        current_flash: &[u8],
        stock_flash: &[u8],
    ) -> Option<DamosMatrixData> {
        if !matches!(param.byte_len, 1 | 2 | 4) || (param.is_float && param.byte_len != 4) {
            return None;
        }
        let rows = param.rows.max(1);
        let cols = param.cols.max(1);
        let total_cells = rows.checked_mul(cols)?;
        if total_cells > MAX_MATRIX_CELLS {
            return None;
        }
        let total_bytes = total_cells.checked_mul(param.byte_len)?;
        let data_off = (param.file_offset as usize).checked_add(param.data_offset)?;
        let end_off = data_off.checked_add(total_bytes)?;
        if end_off > current_flash.len() {
            return None;
        }

        let base = param.base_address;
        // Handle 1D CURVE stored as rows x 1 where the curve axis is placed in `xAxis`
        let ((x_unit, x_axis), (y_unit, y_axis)) =
            if cols == 1 && rows > 1 && param.y_axis.is_none() && param.x_axis.is_some() {
                (
                    ("-".to_string(), vec![0.0]),
                    Self::read_axis_values(base, &param.x_axis, rows, current_flash),
                )
            } else {
                (
                    Self::read_axis_values(base, &param.x_axis, cols, current_flash),
                    Self::read_axis_values(base, &param.y_axis, rows, current_flash),
                )
            };

        let mut values = vec![vec![0.0; cols]; rows];
        let mut stock_values = vec![vec![0.0; cols]; rows];
        for r in 0..rows {
            for c in 0..cols {
                let idx = param.cell_index(r, c)?;
                let off = data_off.checked_add(idx.checked_mul(param.byte_len)?)?;
                let v_cur = param.cell_phys(current_flash, off)?;
                values[r][c] = v_cur;
                stock_values[r][c] = param.cell_phys(stock_flash, off).unwrap_or(v_cur);
            }
        }

        Some(DamosMatrixData {
            id: param.id.clone(),
            title: param.title.clone(),
            kind: param.kind.clone(),
            unit: param.unit.clone(),
            cpu_address: param.cpu_address,
            rows,
            cols,
            x_unit,
            y_unit,
            x_axis,
            y_axis,
            values,
            stock_values,
        })
    }

    /// Encode updated physical matrix values into `current_flash` and return `(cal_offset, bytes)` of the
    /// data block for live injection into `Simos18Board::write_live_calibration`.
    ///
    /// Only cells whose raw encoding differs from the current Flash are written, and a value bit-identical
    /// to what `read_matrix` returned for a cell leaves it untouched, so read → write is lossless.
    /// Nothing is written if any cell fails to encode, or if the parameter lies outside the `CAL` block
    /// (`0x200000..0x280000`, preserving Zero-Patch on code blocks).
    pub fn write_matrix(
        &self,
        param: &DamosParameter,
        current_flash: &mut [u8],
        new_values: &[Vec<f64>],
    ) -> Result<(usize, Vec<u8>), String> {
        let len = param.byte_len;
        if !matches!(len, 1 | 2 | 4) || (param.is_float && len != 4) {
            return Err(format!("{}: invalid byte width {len}", param.id));
        }
        let rows = param.rows.max(1);
        let cols = param.cols.max(1);
        if new_values.len() != rows || new_values.iter().any(|r| r.len() != cols) {
            return Err(format!(
                "Matrix dimension mismatch: expected {}x{}",
                rows, cols
            ));
        }

        let total_bytes = rows
            .checked_mul(cols)
            .and_then(|n| n.checked_mul(len))
            .ok_or_else(|| format!("{}: matrix size overflow", param.id))?;
        let start = (param.file_offset as usize)
            .checked_add(param.data_offset)
            .ok_or_else(|| format!("{}: offset overflow", param.id))?;
        let end = start
            .checked_add(total_bytes)
            .ok_or_else(|| format!("{}: end offset overflow", param.id))?;
        if end > current_flash.len() {
            return Err("Parameter out of Flash bounds".to_string());
        }
        if start < CAL_FLASH_OFFSET || end > CAL_FLASH_END {
            return Err(format!(
                "{}: file range 0x{start:06X}..0x{end:06X} lies outside CAL sector (0x{CAL_FLASH_OFFSET:06X}..0x{CAL_FLASH_END:06X})",
                param.id
            ));
        }

        let mut patches = Vec::new();
        for (r, row) in new_values.iter().enumerate() {
            for (c, &v) in row.iter().enumerate() {
                if !v.is_finite() {
                    return Err(format!("{}[{}][{}]: non-finite value", param.id, r, c));
                }
                let idx = param
                    .cell_index(r, c)
                    .ok_or_else(|| format!("{}: cell index overflow", param.id))?;
                let off = start
                    .checked_add(idx.checked_mul(len).ok_or("cell byte offset overflow")?)
                    .ok_or("cell file offset overflow")?;
                if param.cell_phys(current_flash, off).map(f64::to_bits) == Some(v.to_bits()) {
                    continue;
                }
                let bytes = param.encode_cell(v)?;
                if current_flash[off..off + len] != bytes[..len] {
                    patches.push((off, bytes));
                }
            }
        }
        for (off, bytes) in patches {
            current_flash[off..off + len].copy_from_slice(&bytes[..len]);
        }
        Ok((start - CAL_FLASH_OFFSET, current_flash[start..end].to_vec()))
    }
}
