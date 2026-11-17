//! Channel dictionary: logger channel → what it reads in the SIMOS18.1 firmware → what it is in the plant.
//!
//! Logger identification (VERIFIED by source): the 54-column set and its order are the default UDS-0x22
//! ("mode 22") PID list `list22d` of the SimosTools Android logger (github.com/Switchleg1/SimosTools,
//! `app/src/main/java/com/app/simostools/PIDs.kt`). Every channel is a ReadDataByIdentifier DID (VW
//! measured-value DIDs 0x1xxx..0x4xxx, OBD mode-01 PIDs mapped to 0xF4xx) decoded with the app's equation,
//! or a value the app computes (`Boost`, `Airmass`, `Fuel Trim`, `Calc HP/TQ`). The same DIDs are read
//! from the emulated ECU by the replay's tester ([`Channel::did`], [`Channel::decode`]), so both sides go
//! through the same firmware path and the same scaling.
//!
//! `fw` = the firmware variable the DID returns in SC8O30/SC8O40: RAM address VERIFIED-S from the UDS 0x22
//! table (descriptor @0x8005A960 -> 845 entries @0x800581C4, 12-byte {ptr, DID, len, mask, kind}; 0xF4xx DIDs
//! routed to the OBD mode-01 PID table @0x80050800), names VERIFIED-FR from the FR DID chapter (pp. 3967-4160;
//! FR baseline SC8L50Y0, so names are HYP for this build).
//! `plant` = the physical quantity in the
//! simulation that the channel should measure on the real car (`None` = firmware-internal only).

use simemu_physics::PlantTelemetry;

#[derive(Clone, Copy)]
pub enum Raw {
    U8,
    U16,
    I16,
}

pub type DidSpec = (u16, Raw, fn(f64) -> f64);

pub struct Channel {
    /// SimosTools channel name (= CSV header).
    pub name: &'static str,
    pub unit: &'static str,
    /// UDS DID, raw type and the SimosTools equation (`None` = computed by the app).
    pub did: Option<DidSpec>,
    pub fw: &'static str,
    pub plant: Option<fn(&PlantTelemetry) -> f64>,
    /// Plot group.
    pub group: &'static str,
}

impl Channel {
    pub fn raw_len(&self) -> usize {
        match self.did {
            Some((_, Raw::U8, _)) => 1,
            Some(_) => 2,
            None => 0,
        }
    }

    /// Physical value of a DID answer (big-endian bytes, as on the wire).
    pub fn decode(&self, b: &[u8]) -> Option<f64> {
        let (_, raw, eq) = self.did?;
        let x = match raw {
            Raw::U8 => *b.first()? as f64,
            Raw::U16 => u16::from_be_bytes([*b.first()?, *b.get(1)?]) as f64,
            Raw::I16 => i16::from_be_bytes([*b.first()?, *b.get(1)?]) as f64,
        };
        Some(eq(x))
    }
}

macro_rules! ch {
    ($n:expr, $u:expr, $did:expr, $fw:expr, $plant:expr, $g:expr) => {
        Channel { name: $n, unit: $u, did: $did, fw: $fw, plant: $plant, group: $g }
    };
}

/// Mean commanded spark advance over the four coils, °BTDC (decoded GPTA coil timing).
fn spark_avg(t: &PlantTelemetry) -> f64 {
    let v: Vec<f64> = t.ecu_outputs.ignition.iter().flatten().map(|c| c.spark_btdc_deg).collect();
    if v.is_empty() { f64::NAN } else { v.iter().sum::<f64>() / v.len() as f64 }
}

/// First DI pulse of cylinder 1 in the last completed cycle, ms (ATIC121 input G0.GTC04) - what DID 0x13A0
/// reports (TI_MES_4[PLS_1][0]).
fn di_pw(t: &PlantTelemetry) -> f64 {
    t.ecu_outputs.injection[0][0].map_or(0.0, |p| p.width_ms)
}

pub static CHANNELS: &[Channel] = &[
    ch!("Engine Speed", "rpm", Some((0xF40C, Raw::U16, |x| x / 4.0)), "N_SAE @0xD001249A via OBD PID 0x0C", Some(|t| t.rpm), "speed"),
    ch!("Vehicle Speed", "km/h", Some((0x2033, Raw::U16, |x| x / 100.0)), "VS @0xD000F618 u8 km/h, sent x100 (firmware 0.01 km/h/bit; SimosTools list22d divides by 347.947, this log is consistent with x/100, doc §2)", Some(|t| t.vehicle_kmh), "speed"),
    ch!("Gear", "-", Some((0x210F, Raw::U16, |x| x)), "GEAR @0xD000F5EB (from the TCU over CAN): sent GEAR-1 for 1..6, 8 = no gear, 10 = 7th, 7/9 = reverse/other", None, "speed"),
    ch!("Pedal Pos", "%", Some((0x1070, Raw::U16, |x| x / 10.24)), "PV_AV @0xD00120C4", None, "driver"),
    ch!("TPS", "°TPS", Some((0x20BA, Raw::I16, |x| x / 10.0)), "TPS_ETC @0xD00152EA, degTPS (not %)", Some(|t| t.throttle_pct * 0.9), "air"),
    ch!("Airflow", "kg/h", Some((0x2032, Raw::U16, |x| x)), "MAF_CYL @0xD0013EA8 (1/32 kg/h)", Some(|t| t.maf_kg_h), "air"),
    ch!("Airmass", "mg/stk", None, "app: Airflow / rpm · 8333.33", Some(|t| t.air_mg_stroke), "air"),
    ch!("MAP", "kPa", Some((0x39C0, Raw::U16, |x| x / 10.0)), "MAP @0xD0013EFC (G71)", Some(|t| t.map_kpa), "air"),
    ch!("MAP SP", "kPa", Some((0x39C1, Raw::U16, |x| x / 10.0)), "MAP_SP @0xD0013F3E", None, "air"),
    ch!("PUT", "kPa", Some((0x202A, Raw::U16, |x| x / 10.0)), "PUT_MES @0xD0014012 (G31)", Some(|t| t.put_kpa), "air"),
    ch!("PUT SP", "kPa", Some((0x2029, Raw::U16, |x| x / 10.0)), "PUT_SP @0xD0011FA2", None, "air"),
    ch!("Boost", "kPa", None, "app: MAP − ambient", Some(|t| t.map_kpa - t.baro_kpa), "air"),
    ch!("Ambient Press", "kPa", Some((0x13CA, Raw::U16, |x| x / 120.60176665439)), "AMP @0xD0013CD4", Some(|t| t.baro_kpa), "ambient"),
    ch!("Ambient Temp", "°C", Some((0x1004, Raw::I16, |x| x / 128.0)), "TAA @0xD0011BFA", Some(|t| t.ambient_c), "ambient"),
    ch!("IAT", "°C", Some((0x1001, Raw::U8, |x| x * 0.75 - 48.0)), "TIA @0xD000C269 (G42)", Some(|t| t.iat_c), "ambient"),
    ch!("Turbo Air Temp", "°C", Some((0x1041, Raw::I16, |x| x * 0.005859375 + 144.0)), "TIA_CHA_DOWN_MV @0xD0011FCE, charge air downstream of the compressor", Some(|t| t.compressor_out_c), "turbo"),
    ch!("Turbo Speed", "krpm", Some((0x1040, Raw::U16, |x| x / 163.84)), "N_TCHA @0xD0011F2C (modelled: no speed sensor on IS38)", Some(|t| t.turbo_rpm / 1000.0), "turbo"),
    ch!("Wastegate", "%", Some((0x39A2, Raw::U16, |x| 100.0 - x / 100.0)), "100 - POSN_BPA[0] @0xD0011E44 (SC8S50: @0xD0011D8A); POSN_BPA 0 % = flap seated, so the SimosTools value is 100 = closed (docs/SIMOS18_GPTA_IO_RE.md §12.6)", Some(|t| 100.0 - t.wastegate_pos_pct), "turbo"),
    ch!("Wastegate SP", "%", Some((0x39A3, Raw::U16, |x| 100.0 - x / 100.0)), "100 - POSN_SP_BPA[0] @0xD0011E50", None, "turbo"),
    ch!("Lambda", "-", Some((0x10C0, Raw::U16, |x| x / 1024.0)), "LAMB_LS_UP[0] @0xD001215E", Some(|t| t.lambda_sensor), "fuel"),
    ch!("Lambda SP", "-", Some((0xF444, Raw::U16, |x| x / 32768.0)), "LAMB_SP_SAE @0xD0014450 via OBD PID 0x44", None, "fuel"),
    ch!("STFT", "%", Some((0xF406, Raw::U8, |x| x / 1.28 - 100.0)), "FAC_LAM_LIM_SAE[1] @0xD000F261 via OBD PID 0x06 (HYP name)", None, "fuel"),
    ch!("LTFT", "%", Some((0xF456, Raw::U8, |x| x / 1.28 - 100.0)), "LAMB_DELTA_AD_LAM_ADJ_SAE[1] @0xD000F26A via PID 0x56 = secondary-O2 trim, NOT the long-term trim (that is PID 0x07)", None, "fuel"),
    ch!("Fuel Trim", "%", None, "app: STFT + LTFT", None, "fuel"),
    ch!("Inj PW DI", "ms", Some((0x13A0, Raw::U16, |x| x / 250.0)), "TI_MES_4[PLS_1][0] @0xD0013B74: cylinder 1, first pulse only", Some(di_pw), "fuel"),
    ch!("Inj PW MPI", "ms", Some((0x13AC, Raw::U16, |x| x / 250.0)), "TI_1_MES_MPI[0] @0xD001389A: cylinder 1 only", None, "fuel"),
    ch!("FP DI", "bar", Some((0x2027, Raw::U16, |x| x / 10.0)), "FUP @0xD0013724 (G247)", Some(|t| t.rail_mpa * 10.0), "fuel"),
    ch!("FP DI SP", "bar", Some((0x293B, Raw::U16, |x| x / 10.0)), "FUP_SP @0xD00136B8", None, "fuel"),
    ch!("FP MPI", "bar", Some((0x2025, Raw::U16, |x| x / 1000.0)), "FUP_EFP @0xD0011BE4 (G410)", Some(|t| t.lp_fuel_bar), "fuel"),
    ch!("FP MPI SP", "bar", Some((0x2932, Raw::U16, |x| x / 1000.0)), "FUP_EFP_SP @0xD0013684", None, "fuel"),
    ch!("HPFP Eff Vol", "%", Some((0x209A, Raw::U16, |x| x / 100.0)), "PUMP_VOL_VCV @0xD00136B4 (100 % = full stroke)", None, "fuel"),
    ch!("LPFP Duty", "%", Some((0x2028, Raw::U16, |x| x / 100.0)), "EFPPWM @0xD0013678", Some(|t| t.ecu_outputs.lpfp_duty.map_or(f64::NAN, |d| d * 100.0)), "fuel"),
    ch!("Eth Content", "%", Some((0xF452, Raw::U8, |x| x / 2.55)), "FAC_AFU_SAE @0xD000C2C4 via PID 0x52", None, "fuel"),
    ch!("Ign Avg", "°", Some((0x2004, Raw::I16, |x| x / 100.0)), "IGA_AV_MV @0xD000E6DE (0.375 deg)", Some(spark_avg), "ignition"),
    ch!("Knock Cyl 1", "°", Some((0x200A, Raw::I16, |x| x / 100.0)), "IGA_KNK_BAS_COR[0] @0xD001988E (0.375 deg)", None, "ignition"),
    ch!("Knock Cyl 2", "°", Some((0x200B, Raw::I16, |x| x / 100.0)), "IGA_KNK_BAS_COR[3] @0xD0019891", None, "ignition"),
    ch!("Knock Cyl 3", "°", Some((0x200C, Raw::I16, |x| x / 100.0)), "IGA_KNK_BAS_COR[1] @0xD001988F", None, "ignition"),
    ch!("Knock Cyl 4", "°", Some((0x200D, Raw::I16, |x| x / 100.0)), "IGA_KNK_BAS_COR[2] @0xD0019890", None, "ignition"),
    ch!("Misfires", "-", Some((0x2904, Raw::U16, |x| x)), "MIS_SUM_B @0xD00145CA", None, "ignition"),
    ch!("Torque", "Nm", Some((0x437C, Raw::I16, |x| x / 10.0)), "TQ_ENG @0xD00153E8 (firmware torque estimate)", Some(|t| t.torque_brake_nm), "torque"),
    ch!("Torque Req", "Nm", Some((0x4380, Raw::I16, |x| x / 10.0)), "TQ_DRIV_REQ @0xD00120EA (driver request)", None, "torque"),
    ch!("Calc HP", "hp", None, "app: power from vehicle acceleration (empty in this log)", None, "torque"),
    ch!("Calc TQ", "Nm", None, "app: torque from vehicle acceleration (empty in this log)", None, "torque"),
    ch!("Int Cam Pos", "°", Some((0x201E, Raw::I16, |x| 30.0 - x / 10.0)), "CAM_VCP[IN_1] @0xD001574E (offset 30 - x is the app's own)", Some(|t| t.cam_phase_deg[0]), "valvetrain"),
    ch!("Exhaust Cam Pos", "°", Some((0x201A, Raw::I16, |x| x / 10.0 - 24.0)), "CAM_VCP[EX_1] @0xD0015750 (offset x - 24 is the app's own)", Some(|t| t.cam_phase_deg[1]), "valvetrain"),
    ch!("Valve Lift Pos", "-", Some((0x167C, Raw::U8, |x| x)), "STATE_VVL_MNG @0xD000F6DF (enum)", None, "valvetrain"),
    ch!("Port Flap Pos", "-", Some((0x295C, Raw::U8, |x| x)), "STATE_EL_FAC_MVB_PORT @0xD000C326 (enum)", None, "valvetrain"),
    ch!("Coolant Temp", "°C", Some((0x11CD, Raw::U8, |x| x - 40.0)), "TCO_1_SYS @0xD000C7F0", Some(|t| t.coolant_c), "thermal"),
    ch!("Oil Temp", "°C", Some((0x202F, Raw::U16, |x| (x - 2731.4) / 10.0)), "TOIL @0xD000C5AE (byte, °C + 40; DID handler 0x801BC33C); SC8S50: the oil-temperature model, not the sender", Some(|t| t.oil_c), "thermal"),
    ch!("Cooling Fan", "%", Some((0x101E, Raw::U8, |x| x * 0.390625)), "ECF_REQ_AC_CAN @0xD0004CE6: A/C fan request over CAN, not the fan output", None, "thermal"),
    ch!("Battery Voltage", "V", Some((0x14A6, Raw::U8, |x| x * 0.1015625)), "VB @0xD000F58E", Some(|t| t.battery_v), "thermal"),
    ch!("CPU Load", "%", Some((0x14EC, Raw::U16, |x| x * 0.09765625)), "CPU_LOAD @0xD0013848", None, "ecu"),
    ch!("Cruise", "-", Some((0x203C, Raw::U16, |x| x)), "VLCSt_st_VW @0xD001B6B1", None, "ecu"),
];

pub fn by_name(name: &str) -> Option<&'static Channel> {
    CHANNELS.iter().find(|c| c.name.eq_ignore_ascii_case(name))
}

/// The dictionary as a Markdown table, with the unit and coverage found in `log` (log columns missing from
/// the dictionary are listed at the end).
pub fn dictionary_markdown(log: &super::csv_log::LogData) -> String {
    let mut s = String::from("| channel | group | log unit / dict unit | samples | DID | bytes | firmware quantity | plant counterpart |\n|---|---|---|---|---|---|---|---|\n");
    for c in CHANNELS {
        let k = log.names.iter().position(|n| n.eq_ignore_ascii_case(c.name));
        let n = k.map_or(0, |k| log.cols[k].iter().filter(|x| x.is_finite()).count());
        let lu = k.map_or("—", |k| log.units[k].as_str());
        let did = c.did.map_or("app".to_string(), |d| format!("0x{:04X}", d.0));
        s += &format!("| {} | {} | {} / {} | {} | {} | {} | {} | {} |\n", c.name, c.group, if lu.is_empty() { "-" } else { lu }, c.unit, n, did, c.raw_len(), c.fw, if c.plant.is_some() { "yes" } else { "—" });
    }
    let extra: Vec<&String> = log.names.iter().filter(|n| by_name(n).is_none()).collect();
    if !extra.is_empty() {
        s += &format!("\nLog columns not in the dictionary: {extra:?}\n");
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dictionary_covers_the_simostools_mode22_list_and_decodes_big_endian() {
        assert_eq!(CHANNELS.len(), 53);
        let k = by_name("knock cyl 1").unwrap();
        assert_eq!(k.decode(&[0xFE, 0x3E]), Some(-4.5)); // −450 / 100
        assert_eq!(by_name("Engine Speed").unwrap().decode(&[0x0C, 0x38]), Some(782.0));
        assert_eq!(by_name("IAT").unwrap().raw_len(), 1);
    }
}
