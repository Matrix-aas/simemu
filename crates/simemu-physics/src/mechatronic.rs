//! Gearbox mechatronic hydraulics driven by the TCU's power stages.
//!
//! DQ200 (0AM/0CW, J743): own hydraulic circuit with electric pump V401 and a nitrogen accumulator
//! (system pressure kept between ≈ 40 and 60 bar by the TCU switching the pump), 8 valves:
//! N433 (shift 1/3), N434 (shift 5/7), N435 (K1 actuator flow valve), N436 (sub-gearbox 1 system pressure,
//! feeds N433/N434/N435), N437 (shift 2/4), N438 (shift 6/R), N439 (K2 actuator flow valve), N440 (sub-gearbox 2,
//! feeds N437/N438/N439; [`Dq200HydraulicsHw::clutch_valve_neutral_a`]). Source: Actronics, "TCU DSG7 DQ200" (actronics.co.uk/news/tcu-dsg7-dq200)
//! and VW SSP "7-speed double clutch gearbox 0AM". The differential gear-actuator piston (system
//! pressure permanently on the annulus) is an assumption (UNVERIFIED).
//!
//! Wet units (DQ250 02E / DQ381 0GC): engine-driven pump, main-pressure regulator (N217, falling
//! characteristic = maximum pressure when de-energised), cooling-oil valve (N218), clutch pressure
//! valves (N215 K1, N216 K2), sub-gearbox safety valves (N233, N371), gear actuators via on/off
//! valves N88..N91 + multiplexer N92 (DQ250; VW SSP 308 "Direct shift gearbox 02E"); DQ381 is
//! modelled with one proportional 4/3 valve per actuator (UNVERIFIED). Pump suction aeration:
//! inlet pressure = p_atm − filter/inlet losses; delivery falls when it approaches the air-release
//! pressure (N. D. Manring, *Hydraulic Control Systems*, 2005, ch. 10).
//!
//! Nothing here decides anything: every valve position follows the coil current produced by the
//! TCU's PWM duty; `None` = driver off = de-energised hardware.

use crate::hydraulics::{self, Chamber, DcPump, DcPumpHw, Feed, Fluid, Solenoid, SolenoidHw};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

pub const SOLENOID_CHANNELS: usize = 12;

/// Hardware outputs of the TCU, at the power-stage level. `None` = output stage not driven.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TcuOutputs {
    /// PWM duty 0..1 of each solenoid driver (channel meaning per gearbox: [`solenoid_names`]).
    pub solenoid_duty: [Option<f64>; SOLENOID_CHANNELS],
    /// Electric pump motor PWM duty (DQ200 V401). Wet units have an engine-driven pump.
    pub pump_motor_duty: Option<f64>,
    /// DQ200 V401 pump shaft speed (rad/s) when the motor and its driver chip are modelled on the TCU
    /// board (`simemu_tc1791::tc1784_io::pump`): the board integrates the brushless motor against the
    /// gear-pump load at commutation resolution; the plant then only converts the shaft speed into
    /// delivery. `None` = board-less use: [`DcPump`] driven by `pump_motor_duty`.
    #[serde(default)]
    pub pump_shaft_rad_s: Option<f64>,
    /// Coil currents (A) per solenoid channel when the TCU board integrates the coils itself (DQ200:
    /// the firmware's own 3 kHz current controllers close on the board's coil model, so the spool
    /// follows that current, not a duty-based estimate). `None` = estimate from `solenoid_duty`.
    #[serde(default)]
    pub solenoid_current_a: Option<[f64; SOLENOID_CHANNELS]>,
}

impl TcuOutputs {
    pub fn any_driven(&self) -> bool {
        self.pump_motor_duty.is_some()
            || self.pump_shaft_rad_s.is_some()
            || self.solenoid_duty.iter().any(Option::is_some)
            || self.solenoid_current_a.is_some()
    }
}

/// DQ200 solenoid channels.
pub mod dq200 {
    pub const N433_SHIFT_1_3: usize = 0;
    pub const N434_SHIFT_5_7: usize = 1;
    pub const N435_K1: usize = 2;
    pub const N436_SUB1: usize = 3;
    pub const N437_SHIFT_2_4: usize = 4;
    pub const N438_SHIFT_6_R: usize = 5;
    pub const N439_K2: usize = 6;
    pub const N440_SUB2: usize = 7;
    /// Shift valve per fork in `DrivetrainProfile::forks` order (1/3, 2/4, 5/7, 6/R).
    pub const FORK_VALVE: [usize; 4] = [N433_SHIFT_1_3, N437_SHIFT_2_4, N434_SHIFT_5_7, N438_SHIFT_6_R];
}

/// Wet-unit solenoid channels (DQ250 names; DQ381 uses 0..3 as proportional actuator valves).
pub mod wet {
    pub const N88: usize = 0;
    pub const N89: usize = 1;
    pub const N90: usize = 2;
    pub const N91: usize = 3;
    pub const N92_MUX: usize = 4;
    pub const N215_K1: usize = 5;
    pub const N216_K2: usize = 6;
    pub const N217_MAIN: usize = 7;
    pub const N218_COOL: usize = 8;
    pub const N233_SUB1: usize = 9;
    pub const N371_SUB2: usize = 10;
    /// DQ250 gear-actuator valves per actuator, in the firmware's actuator order (= `DrivetrainProfile::forks`: 1/3, 4/2,
    /// 5/N, 6/R): its two chamber commands (+0x2E / +0x2C of each actuator record) go to outputs 0x800DBA06 = [0, 3, 0,
    /// 2] / 0x800DBA02 = [1, 2, 1, 3] (0D9300011R, 0x800A46E4), outputs 0..3 = shift valves A, C, B, D on P0.12..15
    /// (VERIFIED-S); A..D = N88, N90, N89, N91 is HYP (SSP 308 naming order). Direction (VERIFIED-S): the actuator controller
    /// (0x800A4680 -> `FUN_800abc54` -> `FUN_8009e184`) commands the +0x2C chamber for a target of position class 1
    /// (travel below −6.50 mm: gears 3, 2, N, R, table 0x800DBA0A) and the +0x2E chamber for class 2 (above +6.50 mm: 1, 4,
    /// 5, 6, table 0x800DBA0E). Multiplexer (HYP, [`super::WetHydraulicsHw::mux_energised`]): de-energised it connects the
    /// valves to actuators 1/3 and 6/R, energised to 4/2 and 5/N — the only grouping under which every request the
    /// firmware makes in the drive is completed (`studio::tcu_io::dq250_drive_scenario`, VERIFIED-T on 0D9300011R: in P
    /// it asks N89 + N92 for 2nd and N91 for R, in D N88 without N92 for 1st; the {1/3, 4/2} / {5/N, 6/R} grouping
    /// moves 6th or 5th instead).
    pub const GEAR_VALVE_PLUS: [usize; 4] = [N88, N91, N88, N89];
    pub const GEAR_VALVE_MINUS: [usize; 4] = [N90, N89, N90, N91];
    /// Sub-gearbox whose pressure feeds actuator f's valves: N88/N90 are supplied with sub-gearbox 1's valves (high
    /// side 1), N89/N91 with sub-gearbox 2's (high side 2) — VERIFIED-S electrically, hydraulic feed HYP.
    pub const GEAR_VALVE_SUB: [usize; 4] = [0, 1, 0, 1];
}

#[derive(Debug, Clone, Serialize)]
pub struct Dq200HydraulicsHw {
    pub pump: DcPumpHw,
    pub acc_precharge_pa: f64,
    pub acc_gas_v0_m3: f64,
    pub line_compliance_m3: f64,
    pub relief_crack_pa: f64,
    pub relief_k: f64,
    /// Laminar internal leakage Q = c·p/μ, m³.
    pub leak_m3: f64,
    /// Sub-gearbox pressure valves N436/N440: (coil current A, regulated pressure Pa), rising. The
    /// DQ200_G2 firmware drives them (current regulators 6/7) through its own p -> I table, CAL
    /// 0x80030E60 (pressure, 0.01 bar) / +0x18 (current, 0.1 mA), 12 points (VERIFIED-S); the plant
    /// valve is that table read backwards. Physical valve data UNVERIFIED (no datasheet); the firmware
    /// adapts its copy per valve (0xD000519C/0xD00051B0), e.g. 6.9 bar at 0.45 A on this unit.
    pub sub_valve_curve: [(f64, f64); 12],
    /// Clutch valves N435/N439 (firmware current regulators 0/1): 3/3 proportional flow valves between the sub-gearbox
    /// pressure (N436/N440) and the clutch actuator, a tank edge below the neutral current — VW SSP 390: they "control
    /// the oil volume". The DQ200_G2 clutch position controller (`0x80094CE4`, output `0xD0006250` → regulator set point
    /// `0x80093B2C`) is built for that (docs/DQ200_IO_MAP.md §14, VERIFIED-S): current = CAL `+0xFA` 0.53 A (descriptor
    /// `[0xD000CC88]` = 0x80031584) + map(P demand) + integral terms, the demand = travel error (0.01 mm) × Kp with Kp ∝
    /// 1/√Δp (`0x800953A0..`: 10·√(10⁷/Δp), clamped 800..1200 ‰ — the flow of an orifice ∝ √Δp), and the map (CAL +0x86
    /// demand −650..770 → +0xB2 current −0.53..+0.47 A) jumps across ±19–20 mA around the neutral: overlap compensation of
    /// a spool valve. A pressure valve would hold no position at a fixed current and needs no such map. 0.53 A is also
    /// the current the firmware holds in P/N (clutches stay where they are).
    pub clutch_valve_neutral_a: f64,
    /// Opening of the clutch valve per current above the neutral: (ΔI A, −1 tank … +1 supply), the firmware's map read
    /// backwards (demand / 770: it linearises the valve, so demand ∝ flow; HYP).
    pub clutch_valve_opening: [(f64, f64); 22],
    /// Flow coefficient of a fully open clutch-valve edge, m³/(s·√Pa). UNVERIFIED (no valve data): sized so that the
    /// firmware's P loop (Kp·√Δp = 10⁶ ‰·√Pa above, demand 770 = full edge) closes with the time constant its travel
    /// monitor expects — `0x80085E82` predicts the travel as a PT1 of the target (`0x800BD1F0`, α = 3200/10000 per 10 ms
    /// at the 0 °C the TCU reads, CAL 0x80034E7C): τ = 26 ms → k = 7.7·A/(τ·10⁶) = 9.2e-8 (≈ 3.2 mm² port, Cd 0.62).
    pub clutch_valve_k: f64,
    pub shift_valve_k: f64,
    pub gear_area_large_m2: f64,
    pub gear_area_small_m2: f64,
    pub fork_damping: f64,
    pub fork_friction_n: f64,
    pub clutch_damping: f64,
    pub solenoid: SolenoidHw,
    pub c_oil_j_k: f64,
    pub g_oil_case_w_k: f64,
}

impl Dq200HydraulicsHw {
    /// UNVERIFIED sizing: V401 ≈ 20 A / 1.8 l·min⁻¹ at 60 bar (30 A fuse), accumulator charged to
    /// 40–60 bar in a few seconds, relief at 72 bar.
    pub fn dq200() -> Self {
        Self {
            pump: DcPumpHw { r_ohm: 0.15, k_motor: 0.0325, inertia_kgm2: 4e-5, displacement_m3_rev: 0.6e-6, eta_hm: 0.85, slip_m3: 2.0e-15 },
            acc_precharge_pa: 20e5,
            acc_gas_v0_m3: 200e-6,
            line_compliance_m3: 1.0e-6,
            relief_crack_pa: 72e5,
            relief_k: 1.5e-10,
            leak_m3: 5e-16,
            sub_valve_curve: [
                (0.0, 0.0),
                (0.36, 4e5),
                (0.44, 8e5),
                (0.52, 12e5),
                (0.59, 16e5),
                (0.66, 20e5),
                (0.72, 24e5),
                (0.79, 28e5),
                (0.86, 32e5),
                (0.92, 36e5),
                (0.99, 40e5),
                (1.07, 44e5),
            ],
            clutch_valve_neutral_a: 0.53,
            // CAL 0x80031584 + 0xB2 (current, 0.1 mA) → +0x86 (demand) / 770.
            // Controller 0x80094CE4..0x80095C86 (VERIFIED-S, ev_1280258774570ac9.. / ev_67e7d88b75fd4ae0..):
            // CAL+0x58 = -205 (-20.5 mA) and CAL+0x5A = +190 (+19.0 mA) are the physical spool overlap edges
            // (also the integrator clamps CAL+0x5A..+0x78 at |err| <= 0.5 mm). In the controller's q→I table,
            // (-1, -3.0 mA) and (+2, +0.5 mA) keep noise demands |q|<=2 inside the closed overlap without jumping
            // across ±19..20.5 mA; the physical valve itself is closed (opening = 0) across the overlap and cracks
            // open at -20.5 mA / +19.0 mA.
            clutch_valve_opening: [
                (-0.53, -650.0 / 770.0),
                (-0.371, -480.0 / 770.0),
                (-0.331, -430.0 / 770.0),
                (-0.191, -200.0 / 770.0),
                (-0.121, -100.0 / 770.0),
                (-0.078, -50.0 / 770.0),
                (-0.040, -18.0 / 770.0),
                (-0.0285, -12.0 / 770.0),
                (-0.0205, -8.0 / 770.0),
                (-0.019, 0.0),
                (0.0, 0.0),
                (0.0175, 0.0),
                (0.019, 8.0 / 770.0),
                (0.027, 12.0 / 770.0),
                (0.0365, 18.0 / 770.0),
                (0.071, 50.0 / 770.0),
                (0.107, 100.0 / 770.0),
                (0.173, 200.0 / 770.0),
                (0.280, 400.0 / 770.0),
                (0.325, 490.0 / 770.0),
                (0.345, 535.0 / 770.0),
                (0.470, 1.0),
            ],
            clutch_valve_k: 9.2e-8,
            shift_valve_k: hydraulics::orifice_k(0.6e-6, 850.0),
            gear_area_large_m2: 3.14e-4,
            gear_area_small_m2: 1.57e-4,
            fork_damping: 300.0,
            fork_friction_n: 30.0,
            // Viscous drag of the clutch actuator path, N·s/m. UNVERIFIED, bounded by the basic-setting curves
            // (`clutch::DryClutchHw::actuator_bar`): the TCU ramps the pressure at 0.20 bar / 10 ms and still found K2 at
            // 7.62 mm only 1.0 bar above the flat start of its apply curve (14.0 bar at 1.84–4.73 mm), so the actuator
            // covered 2.9 mm on ≤ 1 bar of excess: c ≤ E²·A / (2·Ṗ·Δx) ≈ 270. (2000, the former value, lags 4 bar there.)
            clutch_damping: 100.0,
            // DQ200 direct-acting proportional spool valves: 50 Hz, ζ 0.8. With the physical spool overlap
            // [-19.0 mA, +17.5 mA] closed, the 150 Hz dither inside the overlap does not leak accumulator volume,
            // allowing realistic fast spool response without phase-lag limit cycles in the 2 ms (40 s⁻¹) position loop.
            solenoid: SolenoidHw { spool_hz: 50.0, spool_zeta: 0.8, ..SolenoidHw::TYPICAL },
            c_oil_j_k: 3500.0,
            g_oil_case_w_k: 4.0,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct WetHydraulicsHw {
    pub pump_displacement_m3_rev: f64,
    pub pump_eta_hm: f64,
    pub pump_slip_m3: f64,
    /// Suction filter laminar resistance, Pa per (Pa·s · m³/s). DQ250: the TCU controls its clutches with characteristics
    /// down to −25 °C ATF (0D9300012F CAL 0x8003806C+0x94, ATF axis −25..80 °C; its valve-current PI gains from −40 °C,
    /// 0x80034ADC; VERIFIED-S), so the pump must fill at idle at −25 °C (μ ≈ 2 Pa·s): ≤ 0.2 bar suction loss → 5e7.
    /// UNVERIFIED value (no filter data). The former 2e9 cavitated the pump above ≈ 1100 rpm already at 5 °C (the
    /// studio's default ambient): line pressure 0 bar under pedal, no gear engaged, P2711 / P074A stored (VERIFIED-T,
    /// studio HTTP run of the `octavia_is38` DQ250 build, 2026-11-04).
    pub suction_r: f64,
    pub suction_area_m2: f64,
    pub line_compliance_m3_pa: f64,
    pub line_p_max_pa: f64,
    pub reg_k: f64,
    pub leak_m3: f64,
    /// Clutch cooling oil at the fully open slide valve: 20 l/min at its 2 bar (VW SSP 308 p. 38). The circuit is pressure
    /// limited, so its passages pass less with viscous oil: laminar above the viscosity at `cool_laminar_c` (UNVERIFIED,
    /// no passage data). Matters at cold: the TCU still pulses the cooling at idle there (0D9300011R `FUN_80043f40`,
    /// every 1050 cycles below 20 °C ATF, CAL 0x800311FC+0x0C, VERIFIED-S/T).
    pub cool_max_m3_s: f64,
    pub cool_laminar_c: f64,
    pub clutch_valve_k: f64,
    pub clutch_valve_droop: f64,
    pub clutch_p_max_pa: f64,
    pub gear_valve_k: f64,
    pub gear_area_m2: f64,
    /// DQ381: proportional 4/3 valve per actuator; DQ250: on/off valves + multiplexer.
    pub proportional_gear_valves: bool,
    pub fork_damping: f64,
    pub fork_friction_n: f64,
    pub solenoid: SolenoidHw,
    /// Coil/spool of the clutch valves N215/N216 (the other valves: [`Self::solenoid`]). DQ250: full-scale force at 2 A,
    /// because the TCU drives them linearly up to its clutch-pressure limit of 20 bar at 10 bar/A (0D9300012F CAL
    /// 0x80033D04+0x4B8 = 20000 mbar, `I = 0.1·p`, VERIFIED-S); with the typical 1 A (spool saturating at 1.2 A) both
    /// clutches stopped at 12 bar while the TCU commanded 13.5–16 bar and K2 slipped under kickdown (VERIFIED-T,
    /// `dq250_full_drive_scenario` before 2026-11-03). Coil R/L as [`Self::solenoid`] (02E PCS valves 4.5–6.5 Ω, ATSG).
    pub clutch_solenoid: SolenoidHw,
    /// DQ250 gear-valve hydraulics (see [`wet::GEAR_VALVE_PLUS`]): the actuators the energised multiplexer N92 connects
    /// to the shift valves (the others get them de-energised; HYP), and per actuator whether its +0x2E chamber valve
    /// pushes towards + travel (VERIFIED-S: true for all).
    pub mux_energised: [bool; 4],
    pub plus_valves_0x2e: [bool; 4],
    /// Clutch valves N215/N216: set pressure per coil current, Pa/A (0 = the generic rising law with an 8 % dead band).
    /// DQ250: 10 bar/A = the TCU's own characteristic, `I[A] = 0.1·p[bar]` above 0.1 bar (0D9300011R `FUN_800c7014`,
    /// coefficients 0x800DD248 = [0, 0, 0.1, 0, 0] for both clutches, used while its adaptation is at default; VERIFIED-S).
    /// The physical valve is UNVERIFIED (no datasheet): the TCU is calibrated for this curve.
    pub clutch_valve_pa_per_a: f64,
    /// Main-pressure valve N217 linear falling: `p = line_p_max · (1 − I/i_rated)` — the TCU's own N217 model `I[A] = 1 −
    /// 0.05·p[bar]` (coefficients 0x800DD270 = [1, −0.05, 0, 0], same function; VERIFIED-S). false = generic law.
    pub main_valve_linear: bool,
    /// Main-pressure valve below the TCU's nominal model by this much (linear law only), Pa. DQ250: the firmware is made for
    /// a line that, measured at the clutch with its valve wide open, sits 0..3 bar below its nominal N217 model: its line
    /// adaptation (0D9300012F 0x800669AC states 4/5, values 0xD0003C34..3A, VERIFIED-S) requests 7.5 / 10.5 bar (CAL
    /// 0x80032BD8+0x15C) and adds clamp(request − measured clutch pressure, −0.05, +0.05 bar) per pass, limited to CAL
    /// +0x16E..+0x16C = 0..3 bar (event 0x121 at the lower, 0x148 = P0868 at the upper limit) and added to every request
    /// (`FUN_80099cf8`). On the nominal curve the plant measured 7.74 / 10.82 bar (spill droop, VERIFIED-T), the correction
    /// clamped at 0 and 0x121 stayed. 0.6 bar UNVERIFIED (no valve data), inside that window.
    pub main_valve_offset_pa: f64,
}

impl WetHydraulicsHw {
    /// DQ250 (02E). Pump size UNVERIFIED (≈ 16 cm³/rev class, line pressure up to ≈ 20 bar).
    pub fn dq250() -> Self {
        Self {
            pump_displacement_m3_rev: 16e-6,
            pump_eta_hm: 0.85,
            pump_slip_m3: 6e-14,
            suction_r: 5e7,
            suction_area_m2: 2.5e-4,
            line_compliance_m3_pa: 5e-13,
            line_p_max_pa: 20e5,
            reg_k: 1e-8,
            leak_m3: 4e-15,
            cool_max_m3_s: 20e-3 / 60.0,
            cool_laminar_c: 60.0,
            clutch_valve_k: hydraulics::orifice_k(4.0e-6, 850.0),
            clutch_valve_droop: 0.5e5 / 1.0e-4,
            // the TCU's clutch-pressure limit (0D9300012F CAL 0x80033D04+0x4B8 = 20000 mbar, VERIFIED-S)
            clutch_p_max_pa: 20e5,
            gear_valve_k: hydraulics::orifice_k(2.0e-6, 850.0),
            gear_area_m2: 3.8e-4,
            proportional_gear_valves: false,
            fork_damping: 300.0,
            fork_friction_n: 40.0,
            solenoid: SolenoidHw::TYPICAL,
            clutch_solenoid: SolenoidHw { i_rated_a: 2.0, ..SolenoidHw::TYPICAL },
            mux_energised: [false, true, true, false],
            plus_valves_0x2e: [true; 4],
            clutch_valve_pa_per_a: 10e5,
            main_valve_linear: true,
            main_valve_offset_pa: 0.6e5,
        }
    }
    pub fn dq381() -> Self {
        Self {
            pump_displacement_m3_rev: 14e-6,
            proportional_gear_valves: true,
            clutch_valve_pa_per_a: 0.0,
            main_valve_linear: false,
            clutch_p_max_pa: 16e5,
            clutch_solenoid: SolenoidHw::TYPICAL,
            main_valve_offset_pa: 0.0,
            suction_r: 2.0e9,
            ..Self::dq250()
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[allow(clippy::large_enum_variant)]
pub enum MechatronicHw {
    Dq200(Dq200HydraulicsHw),
    Wet(WetHydraulicsHw),
}

pub fn solenoid_names(hw: &MechatronicHw) -> [&'static str; SOLENOID_CHANNELS] {
    match hw {
        MechatronicHw::Dq200(_) => ["N433 shift 1/3", "N434 shift 5/7", "N435 K1", "N436 sub 1", "N437 shift 2/4", "N438 shift 6/R", "N439 K2", "N440 sub 2", "-", "-", "-", "-"],
        MechatronicHw::Wet(h) if h.proportional_gear_valves => {
            ["gear valve 1/3", "gear valve 2/4", "gear valve 5/7", "gear valve 6/R", "-", "K1 pressure", "K2 pressure", "main pressure", "cooling oil", "sub 1 safety", "sub 2 safety", "-"]
        }
        MechatronicHw::Wet(_) => ["N88", "N89", "N90", "N91", "N92 multiplexer", "N215 K1", "N216 K2", "N217 main pressure", "N218 cooling", "N233 sub 1", "N371 sub 2", "-"],
    }
}

/// Mechanical loads acting on the mechatronic's pistons (supplied by the clutch / gear-train models).
pub struct MechLoads<'a> {
    /// Force along the apply direction on clutch actuator k at travel x (m).
    pub clutch_force: &'a dyn Fn(usize, f64) -> f64,
    pub clutch_damping: [f64; 2],
    /// Squeeze-film suction that can resist plate separation (cavitation-limited), N.
    pub clutch_sep_limit_n: [f64; 2],
    /// Coulomb friction of each clutch actuator path at its present travel, N ([`crate::clutch::ClutchHw::actuator_friction_n`]).
    pub clutch_friction_n: [f64; 2],
    pub clutch_area: [f64; 2],
    pub clutch_stroke: [f64; 2],
    /// Force along +y on fork f at sleeve travel y (m) (detents, end stops not included).
    pub fork_force: &'a dyn Fn(usize, f64) -> f64,
    /// (blocked towards +, blocked towards −, Coulomb friction N) per fork.
    pub fork_stops: [(bool, bool, f64); 4],
    pub fork_y: [f64; 4],
    pub fork_shaft: [usize; 4],
}

#[derive(Debug, Clone, Serialize)]
pub struct Mechatronic {
    pub solenoids: [Solenoid; SOLENOID_CHANNELS],
    /// Accumulator (DQ200, sensor G270) or main line pressure (wet), Pa.
    pub supply_pa: f64,
    pub acc_oil_m3: f64,
    pub pump: DcPump,
    pub pump_flow_m3_s: f64,
    /// Engine-driven pump torque (wet), Nm.
    pub pump_torque_nm: f64,
    /// Fraction of geometric pump flow lost to aeration/cavitation (wet).
    pub aeration: f64,
    pub pump_health: f64,
    pub sub_pa: [f64; 2],
    pub clutch_pa: [f64; 2],
    pub clutch_x_m: [f64; 2],
    pub fork_v: [f64; 4],
    /// Force on the blocker / dog / stop from the last solve, along +y, N.
    pub fork_contact_n: [f64; 4],
    pub fork_chamber_pa: [[f64; 2]; 4],
    pub cool_flow_m3_s: [f64; 2],
    /// Supply oil taken by each consumer in the last sub-step (DQ200: clutch actuators K1/K2, forks 1..4; leakage last), m³/s.
    pub consumer_m3_s: [f64; 7],
    pub balance_fill: f64,
    /// Hydraulic fluid temperature (DQ200 mechatronic oil G510; wet: sump = gearbox case).
    pub oil_c: f64,
    /// ATF oxidation / friction-modifier depletion index 0..1 (wet).
    pub atf_degradation: f64,
}

/// Piecewise-linear `curve` (x ascending) at `x`, last segment extrapolated above the table.
fn curve_at(curve: &[(f64, f64)], x: f64) -> f64 {
    let i = curve.iter().position(|p| x <= p.0).unwrap_or(curve.len() - 1).clamp(1, curve.len() - 1);
    let ((x0, y0), (x1, y1)) = (curve[i - 1], curve[i]);
    (y0 + (y1 - y0) * (x - x0) / (x1 - x0)).max(0.0)
}

/// DQ200 clutch valve N435/N439 at coil current `i_a`: (supply, tank) edge coefficients
/// ([`Dq200HydraulicsHw::clutch_valve_opening`]), linear between the points, held at the ends.
fn clutch_valve_ports(h: &Dq200HydraulicsHw, i_a: f64) -> (f64, f64) {
    let (t, di) = (&h.clutch_valve_opening, i_a - h.clutch_valve_neutral_a);
    let o = if di <= t[0].0 {
        t[0].1
    } else {
        let i = t.iter().position(|p| di <= p.0).unwrap_or(t.len() - 1).max(1);
        let (a, b) = (t[i - 1], t[i]);
        a.1 + (b.1 - a.1) * ((di - a.0) / (b.0 - a.0)).min(1.0)
    };
    let leak = hydraulics::LEAK_FRACTION * h.clutch_valve_k;
    (leak + h.clutch_valve_k * o.max(0.0), leak + h.clutch_valve_k * (-o).max(0.0))
}

/// Wet units: coil current that sets clutch pressure `p_pa` / main pressure `p_pa` (inverse valve laws; test operators).
pub fn wet_clutch_valve_current(h: &WetHydraulicsHw, p_pa: f64) -> f64 {
    let i_rated = h.clutch_solenoid.i_rated_a;
    let s = if h.clutch_valve_pa_per_a > 0.0 { p_pa / (h.clutch_valve_pa_per_a * i_rated) } else { 0.08 + 0.92 * (p_pa / h.clutch_p_max_pa) };
    if p_pa <= 0.0 { 0.0 } else { s.clamp(0.0, 1.1) * i_rated }
}
pub fn wet_main_valve_current(h: &WetHydraulicsHw, p_pa: f64) -> f64 {
    // the lab knows the real valve: its offset from the TCU's nominal curve included
    let p = if h.main_valve_linear { p_pa + h.main_valve_offset_pa } else { p_pa };
    let r = (1.0 - p / h.line_p_max_pa).clamp(0.0, 1.0);
    h.solenoid.i_rated_a * if h.main_valve_linear { r } else { 0.08 + 0.92 * r }
}

/// Arrhenius rule of thumb: oxidation rate doubles every 10 K; ≈ 3000 h life at 100 °C. UNVERIFIED.
fn oxidation_rate(t_c: f64) -> f64 {
    2f64.powf((t_c - 100.0) / 10.0) / (3000.0 * 3600.0)
}

impl Mechatronic {
    pub fn new(hw: &MechatronicHw, t_c: f64) -> Self {
        let (supply, oil) = match hw {
            MechatronicHw::Dq200(h) => (0.0, -h.line_compliance_m3),
            MechatronicHw::Wet(_) => (0.0, 0.0),
        };
        Self {
            solenoids: [Solenoid::new(t_c); SOLENOID_CHANNELS],
            supply_pa: supply,
            acc_oil_m3: oil,
            pump: DcPump::default(),
            pump_flow_m3_s: 0.0,
            pump_torque_nm: 0.0,
            aeration: 0.0,
            pump_health: 1.0,
            sub_pa: [0.0; 2],
            clutch_pa: [0.0; 2],
            clutch_x_m: [0.0; 2],
            fork_v: [0.0; 4],
            fork_contact_n: [0.0; 4],
            fork_chamber_pa: [[0.0; 2]; 4],
            cool_flow_m3_s: [0.0; 2],
            consumer_m3_s: [0.0; 7],
            balance_fill: 0.0,
            oil_c: t_c,
            atf_degradation: 0.0,
        }
    }

    /// DQ200: charged accumulator (bench helper).
    pub fn precharge(&mut self, hw: &MechatronicHw, p_pa: f64) {
        if let MechatronicHw::Dq200(h) = hw {
            let p = if p_pa.is_finite() { p_pa.max(0.0) } else { 0.0 };
            self.acc_oil_m3 = if p <= 0.0 || h.acc_precharge_pa <= 0.0 {
                -h.line_compliance_m3
            } else if p < h.acc_precharge_pa {
                h.line_compliance_m3 * (p / h.acc_precharge_pa - 1.0)
            } else {
                // invert p = p0·(V0/(V0−V))^1.3, capped at the 95 % bladder-stop of `accumulator_pressure`
                (h.acc_gas_v0_m3 * (1.0 - (h.acc_precharge_pa / p).powf(1.0 / 1.3))).min(0.95 * h.acc_gas_v0_m3)
            };
            self.supply_pa = hydraulics::accumulator_pressure(self.acc_oil_m3, h.acc_precharge_pa, h.acc_gas_v0_m3, h.line_compliance_m3);
        }
    }

    fn spool(&self, ch: usize) -> f64 {
        self.solenoids[ch].spool
    }

    /// One sub-step. Returns heat rejected into the gearbox case, W.
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, hw: &MechatronicHw, out: &TcuOutputs, v_batt: f64, omega_engine: f64, case_c: f64, loads: &MechLoads, dt: f64) -> f64 {
        if !(dt > 0.0 && dt.is_finite()) {
            return 0.0;
        }
        let fl = Fluid::ATF;
        let shw = match hw {
            MechatronicHw::Dq200(h) => h.solenoid,
            MechatronicHw::Wet(h) => h.solenoid,
        };
        let mut coil_heat = 0.0;
        for (i, s) in self.solenoids.iter_mut().enumerate() {
            let hw_i = match hw {
                MechatronicHw::Wet(h) if i == wet::N215_K1 || i == wet::N216_K2 => h.clutch_solenoid,
                _ => shw,
            };
            coil_heat += hw_i.coil_to_oil_w_k * (s.coil_c - self.oil_c);
            s.step(&hw_i, out.solenoid_duty[i], out.solenoid_current_a.map(|c| c[i]), v_batt, self.oil_c, dt);
        }
        let mu = fl.mu(self.oil_c);
        match hw {
            MechatronicHw::Dq200(h) => {
                use dq200::*;
                let ps = self.supply_pa;
                match out.pump_shaft_rad_s {
                    Some(w) => self.pump.driven(&h.pump, w, ps, mu),
                    None => self.pump.step(&h.pump, out.pump_motor_duty, v_batt, ps, mu, dt),
                }
                self.pump_flow_m3_s = self.pump.flow_m3_s;
                for (k, ch) in [N436_SUB1, N440_SUB2].into_iter().enumerate() {
                    // Rising characteristic of the firmware's own p -> I table ([`Dq200HydraulicsHw::
                    // sub_valve_curve`]); de-energised = sub-gearbox vented (its forks and clutch unpowered).
                    self.sub_pa[k] = curve_at(&h.sub_valve_curve, self.spool(ch) * h.solenoid.i_rated_a).min(ps);
                }
                let mut cons = 0.0;
                for (k, ch) in [N435_K1, N439_K2].into_iter().enumerate() {
                    let (ks, kt) = clutch_valve_ports(h, self.spool(ch) * h.solenoid.i_rated_a);
                    let c = Chamber { area_m2: loads.clutch_area[k], dir: 1.0, feed: Feed::Valve { ps: self.sub_pa[k], ks, kt } };
                    let q = self.move_clutch(k, &[c], loads, h.clutch_damping, loads.clutch_friction_n[k], dt);
                    self.consumer_m3_s[k] = q;
                    cons += q;
                }
                for (f, &valve) in FORK_VALVE.iter().enumerate() {
                    let p_sub = self.sub_pa[loads.fork_shaft[f]];
                    let (ks, kt) = hydraulics::flow_valve_k(self.spool(valve), h.shift_valve_k, 0.04, 0.3);
                    let big = Chamber { area_m2: h.gear_area_large_m2, dir: 1.0, feed: Feed::Valve { ps: p_sub, ks, kt } };
                    let small = Chamber { area_m2: h.gear_area_small_m2, dir: -1.0, feed: Feed::Valve { ps: p_sub, ks: 50.0 * h.shift_valve_k, kt: 1e-3 * h.shift_valve_k } };
                    let q = self.move_fork(f, &[big, small], loads, h.fork_damping, dt);
                    self.consumer_m3_s[2 + f] = q;
                    cons += q;
                }
                // Accumulator: pump − consumers − leakage − relief (relief implicit).
                let leak = h.leak_m3 * ps / mu;
                self.consumer_m3_s[6] = leak;
                let stiff = hydraulics::accumulator_stiffness(self.acc_oil_m3, h.acc_precharge_pa, h.acc_gas_v0_m3, h.line_compliance_m3);
                let net = self.pump_flow_m3_s - cons - leak;
                let mut dv = net * dt;
                if ps + stiff * dv > h.relief_crack_pa {
                    dv = dt * (net - h.relief_k * (ps - h.relief_crack_pa)) / (1.0 + dt * h.relief_k * stiff);
                }
                self.acc_oil_m3 += dv;
                self.supply_pa = hydraulics::accumulator_pressure(self.acc_oil_m3, h.acc_precharge_pa, h.acc_gas_v0_m3, h.line_compliance_m3);
                // Mechatronic oil: pump losses + throttling + coils, exchanged with the case.
                let heat = (self.pump.power_w - ps * self.pump_flow_m3_s).max(0.0) + (cons + leak) * ps + coil_heat;
                let to_case = h.g_oil_case_w_k * (self.oil_c - case_c);
                self.oil_c += (heat - to_case) / h.c_oil_j_k * dt;
                to_case
            }
            MechatronicHw::Wet(h) => {
                use wet::*;
                self.oil_c = case_c;
                let w = omega_engine.max(0.0);
                let q_geo = h.pump_displacement_m3_rev * w / (2.0 * PI) * self.pump_health;
                let p_in = hydraulics::P_ATM - h.suction_r * mu * q_geo - 0.5 * fl.rho * (q_geo / h.suction_area_m2).powi(2);
                let fill = ((p_in - 0.3e5) / 0.5e5).clamp(0.0, 1.0);
                self.aeration = 1.0 - fill;
                if fill < 1.0 && q_geo > 1e-5 {
                    self.pump_health = (self.pump_health - 2e-4 * (1.0 - fill) * dt).max(0.3); // cavitation erosion
                }
                let pl = self.supply_pa;
                self.pump_flow_m3_s = (q_geo * fill - h.pump_slip_m3 * pl / mu).max(0.0);
                self.pump_torque_nm = if w > 0.0 { h.pump_displacement_m3_rev * pl / (2.0 * PI * h.pump_eta_hm) + 0.02 * mu / 0.01 } else { 0.0 };
                let p_set = if h.main_valve_linear {
                    h.line_p_max_pa * (1.0 - self.spool(N217_MAIN)).clamp(0.0, 1.0) - h.main_valve_offset_pa
                } else {
                    hydraulics::pressure_valve_setpoint(self.spool(N217_MAIN), h.line_p_max_pa, false)
                }
                .max(2.5e5);
                for (k, ch) in [N233_SUB1, N371_SUB2].into_iter().enumerate() {
                    self.sub_pa[k] = hydraulics::pressure_valve_setpoint(self.spool(ch), h.line_p_max_pa, true).min(pl);
                }
                let mut cons = 0.0;
                for (k, ch) in [N215_K1, N216_K2].into_iter().enumerate() {
                    let c = Chamber {
                        area_m2: loads.clutch_area[k],
                        dir: 1.0,
                        feed: Feed::Regulated {
                            p_set: if h.clutch_valve_pa_per_a > 0.0 {
                                (h.clutch_valve_pa_per_a * self.spool(ch) * h.clutch_solenoid.i_rated_a).min(h.clutch_p_max_pa)
                            } else {
                                hydraulics::pressure_valve_setpoint(self.spool(ch), h.clutch_p_max_pa, true)
                            },
                            ps: self.sub_pa[k],
                            k: h.clutch_valve_k,
                            droop: h.clutch_valve_droop,
                        },
                    };
                    cons += self.move_clutch(k, &[c], loads, 0.0, 0.0, dt);
                }
                let mux = self.spool(N92_MUX) > 0.5;
                for f in 0..4 {
                    let leak_k = 1e-3 * h.gear_valve_k;
                    let (fa, fb) = if h.proportional_gear_valves {
                        let p_sub = self.sub_pa[loads.fork_shaft[f]];
                        let (ks, kt) = hydraulics::flow_valve_k(self.spool(f), h.gear_valve_k, 0.04, 0.3);
                        (Feed::Valve { ps: p_sub, ks, kt }, Feed::Valve { ps: p_sub, ks: kt, kt: ks })
                    } else {
                        // DQ250: each chamber has its own 3/2 on/off valve ([`wet::GEAR_VALVE_PLUS`] / `_MINUS`); the
                        // multiplexer N92 connects the valve pair to actuators 1/2 when energised, to 0/3 when not (HYP,
                        // see [`WetHydraulicsHw::mux_energised`]). The other pair's chambers are shut in.
                        let p_sub = self.sub_pa[GEAR_VALVE_SUB[f]];
                        if mux == h.mux_energised[f] {
                            let k_of = |ch: usize| hydraulics::flow_valve_k(self.spool(ch), h.gear_valve_k, 0.0, 0.05);
                            let (vp, vm) = if h.plus_valves_0x2e[f] { (GEAR_VALVE_PLUS[f], GEAR_VALVE_MINUS[f]) } else { (GEAR_VALVE_MINUS[f], GEAR_VALVE_PLUS[f]) };
                            let (a, b) = (k_of(vp), k_of(vm));
                            (Feed::Valve { ps: p_sub, ks: a.0, kt: a.1 }, Feed::Valve { ps: p_sub, ks: b.0, kt: b.1 })
                        } else {
                            let trapped = Feed::Valve { ps: p_sub, ks: leak_k, kt: leak_k };
                            (trapped, trapped)
                        }
                    };
                    let a = Chamber { area_m2: h.gear_area_m2, dir: 1.0, feed: fa };
                    let b = Chamber { area_m2: h.gear_area_m2, dir: -1.0, feed: fb };
                    cons += self.move_fork(f, &[a, b], loads, h.fork_damping, dt);
                }
                // Line pressure node (implicit): pump − consumers − leakage − regulator spill.
                let c = h.line_compliance_m3_pa;
                let a = self.pump_flow_m3_s - cons;
                let b = h.leak_m3 / mu;
                let mut p_new = (c * pl / dt + a + h.reg_k * p_set) / (c / dt + b + h.reg_k);
                if p_new <= p_set {
                    p_new = (c * pl / dt + a) / (c / dt + b);
                }
                self.supply_pa = p_new.max(0.0);
                let spill = h.reg_k * (self.supply_pa - p_set).max(0.0);
                // Cooling valve (falling characteristic) meters the spill to the clutches.
                let open = 1.0 - ((self.spool(N218_COOL) - 0.08) / 0.92).clamp(0.0, 1.0);
                // The clutch packs get oil only through the cooling oil slide valve (VW SSP 308 p. 35/38: "the cooling oil
                // slide valve opens or closes the oil channel to the multi-plate clutches"; gear wheels are lubricated by
                // the spray pipe). The TCU closes it at cold idle (N218 ≈ 1.08 A: 0D9300011R CAL 0x800311FC+0x242 below
                // +0x23E = 10 °C ATF, VERIFIED-S/T) because a flooded open pack drags the input shafts against the
                // synchronisers (SSP 308 p. 52: full cooling flow "can result in gear change difficulties at cold
                // ambient temperatures"). docs/DQ250_FIRMWARE_SPEC.md §12.1.
                let q_cool = spill.min(h.cool_max_m3_s * open * (fl.mu(h.cool_laminar_c) / mu).min(1.0));
                self.cool_flow_m3_s = [0.5 * q_cool; 2];
                let bal_target = (spill / (1.0e-3 / 60.0)).clamp(0.0, 1.0);
                self.balance_fill += (bal_target - self.balance_fill) * (dt / 0.2).min(1.0);
                self.atf_degradation = (self.atf_degradation + oxidation_rate(self.oil_c) * dt).min(1.0);
                // Heavily oxidised fluid varnishes the spool bores: valves resting de-energised seize.
                // ponytail: threshold rule, no per-valve contamination model.
                if self.atf_degradation > 0.9 {
                    for s in self.solenoids.iter_mut().filter(|s| s.spool < 0.05) {
                        s.stuck = true;
                    }
                }
                // Hydraulic power of the pump ends as heat in the ATF.
                self.pump_torque_nm * w + coil_heat
            }
        }
    }

    /// Clutch actuator / piston: quasi-static solve and travel update. Returns supply consumption.
    fn move_clutch(&mut self, k: usize, ch: &[Chamber], loads: &MechLoads, c_base: f64, f_coulomb: f64, dt: f64) -> f64 {
        let x = self.clutch_x_m[k];
        let xmax = loads.clutch_stroke[k];
        let (c, fcav) = (loads.clutch_damping[k], loads.clutch_sep_limit_n[k]);
        let f = |v: f64| (loads.clutch_force)(k, (x + v * dt).clamp(0.0, xmax)) - if v >= 0.0 { c * v } else { (c * v).max(-fcav) };
        let (v, _) = hydraulics::solve_piston(ch, &f, c_base, f_coulomb, x >= xmax, x <= 0.0);
        self.clutch_pa[k] = ch[0].pressure(v);
        self.clutch_x_m[k] = (x + v * dt).clamp(0.0, xmax);
        (ch[0].area_m2 * v).max(0.0)
    }

    fn move_fork(&mut self, f: usize, ch: &[Chamber], loads: &MechLoads, c: f64, dt: f64) -> f64 {
        let y = loads.fork_y[f];
        let force = |v: f64| (loads.fork_force)(f, y + v * dt);
        let (stop_p, stop_n, fc) = loads.fork_stops[f];
        let (v, contact) = hydraulics::solve_piston(ch, &force, c, fc, stop_p, stop_n);
        self.fork_v[f] = v;
        self.fork_contact_n[f] = contact;
        let mut cons = 0.0;
        for (i, c) in ch.iter().enumerate() {
            let p = c.pressure(v);
            self.fork_chamber_pa[f][i] = p;
            cons += (c.dir * c.area_m2 * v).max(0.0);
        }
        cons
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn precharge_and_tcu_any_driven_handle_edge_cases() {
        let hw = MechatronicHw::Dq200(Dq200HydraulicsHw::dq200());
        let mut m = Mechatronic::new(&hw, 25.0);

        m.precharge(&hw, 0.0);
        assert_eq!(m.supply_pa, 0.0);
        assert!(m.acc_oil_m3.is_finite());

        // Below bladder precharge (10 bar < 24 bar): inverts the linear line-compliance branch cleanly.
        m.precharge(&hw, 10e5);
        assert!((m.supply_pa - 10e5).abs() < 1.0);

        // Normal operating pressure (52 bar).
        m.precharge(&hw, 52e5);
        assert!((m.supply_pa - 52e5).abs() < 1.0);

        // Above the 95 % bladder stop and NaN: stays finite and consistent with `accumulator_pressure`.
        m.precharge(&hw, 1e9);
        assert!(m.supply_pa.is_finite() && m.supply_pa > 52e5);
        m.precharge(&hw, f64::NAN);
        assert_eq!(m.supply_pa, 0.0);

        let mut out = TcuOutputs::default();
        assert!(!out.any_driven());
        out.solenoid_current_a = Some([0.0; SOLENOID_CHANNELS]);
        assert!(out.any_driven());
    }
}
