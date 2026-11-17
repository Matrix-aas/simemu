//! Data structures for SIMOS18.1 DAMOS (`SC8S50.json`) parameters, axes, and 2D/3D matrices.

use serde::{Deserialize, Serialize};

fn default_factor() -> f64 {
    1.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RawDamosFile {
    pub name: String,
    pub version: String,
    #[serde(rename = "baseAddress")]
    pub base_address: u32,
    pub parameters: Vec<RawDamosParam>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RawDamosAxis {
    #[serde(rename = "type", default)]
    pub axis_type: String,
    #[serde(default)]
    pub points: usize,
    #[serde(default)]
    pub min: f64,
    #[serde(default)]
    pub max: f64,
    #[serde(default)]
    pub unit: String,
    #[serde(default)]
    pub address: Option<u32>,
    #[serde(rename = "dataType", default)]
    pub data_type: String,
    /// BYTE offset from `address` to the first axis point (skips the point-count word).
    #[serde(rename = "dataOffset", default)]
    pub data_offset: usize,
    #[serde(default = "default_factor")]
    pub factor: f64,
    #[serde(default)]
    pub offset: f64,
}

/// Rational conversion `phys = (a*X + b) / (c + d*X)` (XDF-derived definitions).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RationalFormula {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RawDamosParam {
    pub name: String,
    #[serde(default)]
    pub description: String,
    pub address: u32,
    /// Byte offset from `address` to the first data cell (STD_AXIS records carry their axes in front).
    #[serde(rename = "dataOffset", default)]
    pub data_offset: usize,
    #[serde(default)]
    pub min: Option<f64>,
    #[serde(default)]
    pub max: Option<f64>,
    #[serde(default)]
    pub formula: Option<RationalFormula>,
    #[serde(rename = "type", default)]
    pub kind: String,
    #[serde(rename = "dataType", default)]
    pub data_type: String,
    #[serde(default)]
    pub unit: String,
    #[serde(default = "default_factor")]
    pub factor: f64,
    #[serde(default)]
    pub offset: f64,
    #[serde(default)]
    pub categories: Vec<String>,
    #[serde(rename = "columnDir", default)]
    pub column_dir: bool,
    #[serde(default)]
    pub rows: Option<usize>,
    #[serde(default)]
    pub cols: Option<usize>,
    #[serde(rename = "xAxis", default)]
    pub x_axis: Option<RawDamosAxis>,
    #[serde(rename = "yAxis", default)]
    pub y_axis: Option<RawDamosAxis>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DamosParameter {
    pub id: String,
    pub title: String,
    pub category: String,
    pub kind: String,
    pub data_type: String,
    pub unit: String,
    /// Relative offset inside the `CAL` block (`0x00000..0x7FFFF`)
    pub cal_offset: u32,
    /// File offset inside the 4MB Flash binary (`0x200000 + cal_offset`)
    pub file_offset: u32,
    /// Native TC1791 CPU address in `PFlash1` (`0x80800000 + cal_offset`)
    pub cpu_address: u32,
    /// Byte offset from `file_offset` to the first data cell (non-zero for STD_AXIS records).
    pub data_offset: usize,
    /// `baseAddress` of the definition file this parameter came from (axis addresses are relative to it).
    pub base_address: u32,
    pub byte_len: usize,
    pub signed: bool,
    pub is_float: bool,
    pub scale: f64,
    pub offset: f64,
    /// Physical limits from the definition; writes saturate to them.
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub formula: Option<RationalFormula>,
    pub rows: usize,
    pub cols: usize,
    pub column_dir: bool,
    pub x_axis: Option<RawDamosAxis>,
    pub y_axis: Option<RawDamosAxis>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DamosMatrixData {
    pub id: String,
    pub title: String,
    pub kind: String,
    pub unit: String,
    pub cpu_address: u32,
    pub rows: usize,
    pub cols: usize,
    pub x_unit: String,
    pub y_unit: String,
    pub x_axis: Vec<f64>,
    pub y_axis: Vec<f64>,
    pub values: Vec<Vec<f64>>,
    pub stock_values: Vec<Vec<f64>>,
}
