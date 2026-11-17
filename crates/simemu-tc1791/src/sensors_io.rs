//! Board-level I/O API: analog input voltages (ADC), digital port inputs, CAN reception, memory
//! access, live calibration overlay and real-time stepping.
//!
//! Sensor physics (NTC curves, pressure transfer functions, ...) belongs to `simemu-physics`; the
//! board accepts VOLTAGES. Crankshaft/camshaft edges reach the firmware as GPTA input-pin edges
//! (`engine_io.rs`); nothing is injected into GPTA registers.

use crate::memory::CanFrame;
use crate::{RunStats, Simos18Board};
use std::ffi::c_void;
use unicorn_tricore_sys::*;

/// Named analog signal -> (ADC kernel, channel), SC800S50 evidence:
/// * BSW result buffers (VERIFIED-T, every channel driven with a distinct voltage): logical ADC id
///   `kernel << 14 | i` of the getter 0x800A70FC -> 0x800A6990 reads buffer `*(0xD00006E4 + 4·kernel)[i]`;
///   ADC0 buffer i = CH(15 − i), `ADC1 = [13, 9, 8, 7, 6, 5, 4, 3, 2, 0]`, `ADC2 = [13, 12, 11, 10, 9, 8, 7,
///   5, 4, 3, 2, 1, 0]`.
/// * Function per channel: the firmware's own OBD values (UDS 0x22 F4xx) follow exactly one channel
///   when it is moved 2.5 V -> 1.0 V (VERIFIED-T): F411 throttle ADC0.CH5, F449/F44A pedal D/E
///   ADC0.CH7/CH8, F40F intake air temperature ADC0.CH10, F40B MAP ADC0.CH11, F433 barometric pressure
///   ADC1.CH3, F423 fuel rail pressure ADC2.CH3, F405 coolant temperature ADC2.CH7; and from the getter
///   consumers (VERIFIED-S): throttle second track ADC0.CH6 (pair getter 0x800A67BE ids 0xA/0x9 -> ETC
///   0x8017FA54, c_tps_max_etc), brake light switch voltage ADC0.CH12 (id 3, getter 0x800A5600 -> DRPD
///   0x80196C58 against c_vp_thd_bls_act), KL15 (V_IGK) ADC2.CH4 (id 0x8008 -> PWSL key detection
///   0x801073DA, lc_igk_fil_rst; FR p.23942 divider NC_V_IGK_BRI_RATIO = 5.70, on-threshold 4.0 V; CAN
///   only runs above it), fuel low pressure ADC2.CH2 (id 0x800A -> 0x800D762A, ip_fup_efp); pressure upstream
///   of the throttle ADC1.CH2 (id 0x4008, getter 0x800A643C next to MAP id 4 / AMP id 0x4007 in 0x800FE934;
///   VERIFIED-T: OBD PID 0x70 boost sensor A reads 92.5 kPa at 1.5 V = ip_prs_up_thr_mes).
///
/// Not found on the ADC (not fed): charge-air temperature, oil temperature / pressure (MQB: PWM sender and
/// switches), wide-band lambda (CJ125 over SPI).
pub const UNVERIFIED_ANALOG_MAP: &[(&str, u8, u8)] = &[
    ("throttle_tps1", 0, 5),
    ("throttle_tps2", 0, 6),
    ("pedal_pvs1", 0, 7),
    ("pedal_pvs2", 0, 8),
    ("intake_air_temp", 0, 10),
    ("map", 0, 11),
    ("brake_light_switch", 0, 12),
    ("baro", 1, 3),
    ("boost_pressure", 1, 2),
    ("fuel_low_pressure", 2, 2),
    ("rail_pressure", 2, 3),
    ("kl15", 2, 4),
    ("coolant_temp", 2, 7),
    // Legacy name of ADC2.CH4 (the board's "supply sense" is the KL15 input).
    ("battery", 2, 4),
];

impl Simos18Board {
    /// Read guest memory (any mapped region, including side-effect-free peripheral register peeks).
    pub fn read_mem(&self, addr: u32, len: usize) -> Result<Vec<u8>, String> {
        if self.native.is_some() {
            return self.native_read_mem(addr, len);
        }
        if len == 0 {
            return Ok(Vec::new());
        }
        let mut buf = vec![0u8; len];
        let touches_mmio = (0..len).any(|i| crate::memory::is_mmio(addr.wrapping_add(i as u32)));
        if !touches_mmio {
            // SAFETY: valid engine; buffer valid for `len` bytes; non-MMIO range cannot invoke `&mut` MMIO C callbacks.
            let err = unsafe { uc_mem_read(self.uc, addr as u64, buf.as_mut_ptr() as *mut c_void, len) };
            if err != UC_ERR_OK {
                return Err(format!("uc_mem_read(0x{:08X}, {}) failed: {}", addr, len, Self::strerror(err)));
            }
            return Ok(buf);
        }
        let hw = self.hw();
        for (i, b) in buf.iter_mut().enumerate() {
            let a = addr.wrapping_add(i as u32);
            if crate::memory::is_mmio(a) {
                *b = hw.mmio_peek(a);
            } else {
                // SAFETY: valid engine; 1-byte non-MMIO read.
                let err = unsafe { uc_mem_read(self.uc, a as u64, (b as *mut u8).cast(), 1) };
                if err != UC_ERR_OK {
                    return Err(format!("uc_mem_read(0x{:08X}, 1) failed: {}", a, Self::strerror(err)));
                }
            }
        }
        Ok(buf)
    }

    /// Replace bytes of PFlash1 (offset relative to 0x80800000, i.e. the CAL area) — models
    /// reflashing / an emulation-RAM overlay. Cached and non-cached views share one store.
    pub fn write_live_calibration(&mut self, cal_offset: usize, data: &[u8]) -> Result<(), String> {
        let off = 0x20_0000usize
            .checked_add(cal_offset)
            .filter(|o| o.checked_add(data.len()).is_some_and(|e| e <= 0x40_0000))
            .ok_or_else(|| "Calibration offset out of PFlash1 bounds".to_string())?;
        let hw = self.hw_mut();
        hw.flash[off..off + data.len()].copy_from_slice(data);
        hw.tb_cache.fill(crate::cpu_hooks::TbCacheEntry::EMPTY);
        if let Some(c) = self.nat_mut() {
            c.invalidate_code(0x8080_0000 + cal_offset as u32, data.len() as u32);
            c.invalidate_code(0xA080_0000 + cal_offset as u32, data.len() as u32);
            return Ok(());
        }
        let start = 0x8080_0000u64 + cal_offset as u64;
        for base in [start, start + 0x2000_0000] {
            // SAFETY: outside emulation; invalidates translated code that may cover the bytes.
            unsafe { uc_ctl_remove_cache(self.uc, base, base + data.len() as u64) };
        }
        Ok(())
    }

    /// Apply an analog input voltage to ADC `kernel` (0..=2) channel `channel` (0..=15).
    pub fn set_adc_input_voltage(&mut self, kernel: u8, channel: u8, volts: f32) -> Result<(), String> {
        let slot = self
            .hw_mut()
            .adc
            .volts
            .get_mut(kernel as usize)
            .and_then(|k| k.get_mut(channel as usize))
            .ok_or_else(|| format!("no ADC{} channel {}", kernel, channel))?;
        *slot = volts;
        Ok(())
    }

    /// Drive (Some) or release (None) an external digital input level on port `port` pin `pin`.
    pub fn set_port_input(&mut self, port: usize, pin: u8, level: Option<bool>) {
        let s = &mut self.hw_mut().scu;
        if port >= s.port_ext_level.len() || pin > 15 {
            return;
        }
        let bit = 1u16 << pin;
        match level {
            Some(l) => {
                s.port_ext_driven[port] |= bit;
                if l {
                    s.port_ext_level[port] |= bit;
                } else {
                    s.port_ext_level[port] &= !bit;
                }
            }
            None => s.port_ext_driven[port] &= !bit,
        }
    }

    /// Deliver a CAN frame received on `frame.node`; returns the number of MOs updated (0/1).
    pub fn inject_can_rx(&mut self, frame: &CanFrame) -> usize {
        self.hw_mut().inject_can_rx(frame)
    }

    /// Drive a named analog input (see [`UNVERIFIED_ANALOG_MAP`]) with `volts`. Returns false for
    /// an unknown signal name.
    pub fn set_analog_signal(&mut self, name: &str, volts: f32) -> bool {
        match UNVERIFIED_ANALOG_MAP.iter().find(|(n, _, _)| *n == name) {
            Some(&(_, k, ch)) => self.set_adc_input_voltage(k, ch, volts).is_ok(),
            None => false,
        }
    }

    /// Removes and returns every CAN frame the firmware transmitted since the last call.
    pub fn take_can_tx(&mut self) -> Vec<CanFrame> {
        std::mem::take(&mut self.hw_mut().can_tx_queue)
    }

    /// Removes and returns observed data read/write watchpoint events since the last call.
    pub fn take_watch_events(&mut self) -> Vec<crate::memory::DataWatchEvent> {
        self.hw_mut().watch_events.drain(..).collect()
    }

    /// Removes and returns block-entry counts collected for configured `block_hit_ranges`.
    pub fn take_block_hits(&mut self) -> std::collections::BTreeMap<u32, u64> {
        std::mem::take(&mut self.hw_mut().block_hits)
    }

    /// Removes and returns unique call/jump edges `(from_pc, target_pc, return_pc, instructions)`.
    pub fn take_block_sequence(&mut self) -> Vec<(u32, u32, u32, u64)> {
        std::mem::take(&mut self.hw_mut().block_sequence)
    }

    /// Execute `instructions` guest instructions; returns the final PC.
    pub fn step_quantum(&mut self, instructions: usize) -> Result<u32, String> {
        self.run_instructions(instructions as u64).map(|s| s.pc)
    }

    /// Execute `dt_sec` of simulated time (dt * fCPU / CPI instructions, never skipping
    /// execution). The returned real-time factor is sim_time / wall_time and may be < 1.
    pub fn step_realtime_frame(&mut self, dt_sec: f32) -> Result<RunStats, String> {
        self.run_for(dt_sec.max(0.0) as f64)
    }
}
