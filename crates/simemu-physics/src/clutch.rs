//! Clutch hardware: DQ200 dry double clutch and wet multi-plate clutches (DQ250, DQ381, Haldex).
//!
//! Torque law (both): T = z·μ·F·r_eff, r_eff = (r_o + r_i)/2 (uniform-wear assumption).
//! H. Naunheimer, B. Bertsche, J. Ryborz, W. Novak, *Automotive Transmissions*, 2nd ed., Springer
//! 2011, §6.3/§10 (friction clutches, DCT clutch layouts, centrifugal pressure compensation).
//!
//! Dry clutch:
//! - Travel → clamp force through lever + segment (cushion) spring, progressive then stiff.
//! - μ(T, v): positive dμ/dv for judder stability (D. Centea, H. Rahnejat, M. T. Menday, Appl. Math.
//!   Model. 25 (2001) 177–192); torque/kiss-point drift with temperature and wear
//!   (A. Myklebust, L. Eriksson, IEEE/ASME Trans. Mechatronics 20 (2015) 61–72; Myklebust, PhD
//!   thesis, Linköping 2014). Thermal expansion of plates moves the kiss point.
//! - Heat partition by thermal effusivity √(kρc) (Blok/Charron), lining ≈ 7 %.
//! - Organic facing: phenolic binder decomposition from ≈ 300 °C → glazing, > 380 °C burn-out
//!   (J. Bijwe, Polym. Compos. 18 (1997) 378–396). Wear ∝ friction energy (J. F. Archard, J. Appl.
//!   Phys. 24 (1953) 981), accelerated exponentially above 200 °C. Coefficients UNVERIFIED.
//!
//! Wet clutch:
//! - Open-clutch drag: Couette shear with aeration-limited wetted radius r_e from the radial flow
//!   balance (Y. Yuan, E. A. Liu, J. Hill, Q. Zou, J. Fluids Eng. 129 (2007) 333–337; also
//!   H. Kitabayashi, C. Y. Li, H. Hiraki, SAE 2003-01-1973; S. Iqbal, F. Al-Bender, B. Pluymers,
//!   W. Desmet, J. Fluids Eng. 135 (2013) 021104). Grooves: area fraction with groove-depth gap.
//! - Engagement: reduced Reynolds squeeze film between annular discs
//!   W = 3πμ(−ḣ)/(2h³)·[r_o⁴ − r_i⁴ − (r_o² − r_i²)²/ln(r_o/r_i)] (B. J. Hamrock, *Fundamentals of
//!   Fluid Film Lubrication*, NASA RP-1255, 1991, ch. 15) with a groove relief factor
//!   (E. J. Berger, F. Sadeghi, C. M. Krousgrill, J. Tribol. 118 (1996) 137–146; C. L. Davis,
//!   F. Sadeghi, C. M. Krousgrill, J. Tribol. 122 (2000) 110–118) and exponential asperity load
//!   (Greenwood–Williamson tail).
//! - μ(v, T, p) for paper-based linings: Stribeck-like rise with sliding speed, mild decrease with
//!   temperature and contact pressure (P. Marklund, R. Larsson, Tribol. Int. 41 (2008) 824–830;
//!   M. Ingram, H. Spikes, J. Noles, R. Watts, Tribol. Int. 43 (2010) 815–821). Aged ATF (depleted
//!   friction modifier) raises static μ → negative slope → shudder (T. Kugimiya et al., SAE 972927).
//! - Centrifugal piston pressure p(r) = p_feed + ½ρω²(r² − r_feed²), compensated by a balance
//!   chamber (Naunheimer et al. §10.4).

use crate::hydraulics::Fluid;
use serde::Serialize;
use std::f64::consts::PI;

/// Where the plate pack is in the thermal / failure sense.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct ClutchEnv {
    pub ambient_c: f64,
    pub vehicle_speed_ms: f64,
    /// Drive side (DMF secondary / clutch drum), rad/s.
    pub omega_drum: f64,
    /// Hub (input shaft) speeds, rad/s.
    pub omega_hub: [f64; 2],
    /// Fluid that wets / cools the clutch (wet units) and its degradation 0..1.
    pub oil_c: f64,
    pub atf_degradation: f64,
    /// Cooling oil flow through each wet clutch, m³/s.
    pub cool_flow_m3_s: [f64; 2],
    /// Balance-chamber fill fraction (wet), 0 = drained.
    pub balance_fill: f64,
}

// ---------------------------------------------------------------- dry ----------------------------

#[derive(Debug, Clone, Serialize)]
pub struct DryClutchHw {
    pub r_outer_m: [f64; 2],
    pub r_inner_m: [f64; 2],
    pub mu_nominal: f64,
    /// Hydraulic actuator (slave cylinder) piston in the mechatronic, m², and its travel end stop.
    pub actuator_area_m2: f64,
    pub actuator_travel_max_mm: f64,
    /// Actuator travel / pressure-plate travel (maps lining wear and plate expansion onto the kiss point). UNVERIFIED.
    pub lever_ratio: f64,
    /// Actuator travel at plate contact (new lining, 20 °C), mm.
    pub kiss_mm: [f64; 2],
    /// Torque capacity above the kiss point at the static μ of a new lining, per clutch (a0 Nm/mm, a2 mm², a4 mm):
    /// T(u) = a0·(u + √((u − a4)² + a2) − √(a4² + a2)), u = actuator travel − kiss (mm). The clamp force is
    /// T / (2·μ·r_eff), so μ fade, glazing and lining damage still act on it.
    pub torque_curve: [[f64; 3]; 2],
    /// Static force–travel curve of each clutch at its actuator (lever springs, compensation, lift-off springs and
    /// friction together), as actuator pressure: (travel mm, release bar, apply bar). The actuator moves forward
    /// above the apply pressure and back below the release pressure; the spring force is the mean of both, the
    /// Coulomb friction half the difference ([`ClutchHw::actuator_friction_n`]).
    pub actuator_bar: [[(f64, f64, f64); 9]; 2],
    /// Lining thickness per face above the rivet heads, mm.
    pub lining_mm: f64,
    /// Archard-type specific wear, m³/J (both faces together).
    pub wear_m3_per_j: f64,
    pub c_pressure_plate_j_k: [f64; 2],
    pub c_central_j_k: f64,
    pub c_disc_j_k: f64,
    pub c_housing_j_k: f64,
    /// Stack height whose expansion shortens the air gap (α = 11e-6 1/K grey iron), mm.
    pub stack_mm: f64,
    /// Central plate + pressure plates + cover (rotates with the DMF secondary).
    pub inertia_drive_kgm2: f64,
    /// Drag of a lifted-off disc (air between disc and plates), N·m per rad/s of slip. With the
    /// lift-off springs holding the plates clear there is no rubbing contact (Naunheimer et al. 2011
    /// §6.3: a disengaged dry clutch transmits only negligible residual torque); the release bearing
    /// drags the drive side against the housing, not the hub. UNVERIFIED value.
    pub open_drag_nm_s: f64,
}

impl DryClutchHw {
    /// LuK dry double clutch of the DQ200 (0AM/0CW): reference unit as learned by DQ200_G2 (`0CW300043H_0002`).
    /// Friction geometry UNVERIFIED (μ = 0.28 organic facing).
    ///
    /// Torque characteristic (docs/DQ200_IO_MAP.md §14): the firmware turns a clutch torque request into a travel
    /// target through its own clutch model (`0x800921E6` → `0x80092370`: μ scaling `0x800923AA`, characteristic
    /// `0x800505C0`/`0x80055EFA`, kiss offset `0x800924AC` → `0xD0006090`, VERIFIED-S). The model is fitted to four
    /// support points per clutch in the EEPROM (0x880..0x8BF: travel 0.0001 mm / torque 0.01 Nm; K1 4.86 / 6.75 /
    /// 13.33 / 20.52 mm at 0 / 39.2 / 279.2 / 669.2 Nm, K2 7.87 / 11.98 / 20.20 / 23.05 mm at 0 / 37.5 / 450.1 / 700 Nm,
    /// VERIFIED-S on the reference dump); its parameters are the floats at 0xD000399C (VERIFIED-T, this EEPROM) and the
    /// curve passes through the points (K2 11.98 mm: 39.9 Nm). That they describe the physical clutches (the TCU learned
    /// them on the car) is HYP. The previous plant (cushion law, kiss 7.75 / 8.36 mm) gave K1 ≈ ⅓ of this torque at
    /// a travel, K2 75–90 % above 15 mm: the clutch slipped at every travel the firmware commanded, its slip control
    /// wound the request up (342 Nm for 90 Nm of engine torque in 3rd) and the position loop hunted around it.
    ///
    /// Actuator force–travel (`actuator_bar`): the basic-setting curves from EEPROM 0xB2C..0xB8B (copy 0xB90),
    /// travel axis = 8 points to the learned end stop 0xAC0 (K1 24.35, K2 25.00 mm) with the open stop (1.45 / 1.84 mm)
    /// in front, as the controller uses them (`0x80093674`, RAM 0xD00060EC/0xD0006110/0xD0006134 with flag 0xD0006197 =
    /// 1, VERIFIED-T). Measured by `0x80060DA4` with the clutch valve held open (current override 0xD00035A0) and the
    /// sub-gearbox pressure stepped (pressure override 0xD00035A4 → `0x80071D68`, VERIFIED-S): release = the sub-gearbox
    /// pressure at which the actuator falls back below a point while the pressure ramps down, apply = the pressure at which
    /// it reaches the point while it ramps up. The curves are in the units of the firmware's sub-gearbox valve table,
    /// which the plant's N436/N440 implement (`Dq200HydraulicsHw::sub_valve_curve`). K1 is degressive and K2 rises,
    /// as LuK describes the DQ200 clutch (Schaeffler Kolloquium 2010, "Dry double clutch": "the force curve of clutch 1
    /// … has a downward slope … clutch 2 permanently slopes upward"; actuation "can only perform by means of an actuator
    /// with travel control"). The hysteresis as Coulomb friction of the actuator path is HYP.
    pub fn dq200() -> Self {
        Self {
            r_outer_m: [0.114, 0.108],
            r_inner_m: [0.077, 0.072],
            mu_nominal: 0.28,
            // UNVERIFIED (piston area: flow per mm of travel; the static force law above is in pressure).
            actuator_area_m2: 3.1e-4,
            // Mechanical stop at the travel sensor span (G617/G618: 29.00 mm over 10..90 % duty, CAL
            // 0x80031F4C). UNVERIFIED.
            actuator_travel_max_mm: 29.0,
            lever_ratio: 6.0,
            kiss_mm: [4.86, 7.87],
            torque_curve: [[31.675, 58.645, 3.8788], [49.105, 18.8915, 8.1231]],
            actuator_bar: [
                [(1.45, 5.4, 16.0), (4.31, 5.4, 16.0), (7.17, 4.8, 17.2), (10.03, 4.6, 17.8), (12.89, 6.0, 14.2), (15.75, 10.8, 18.6), (18.61, 18.2, 25.8), (21.47, 27.4, 35.6), (24.35, 38.8, 45.6)],
                [(1.84, 4.8, 14.0), (4.73, 4.8, 14.0), (7.62, 5.6, 15.0), (10.51, 6.6, 15.2), (13.40, 8.2, 16.2), (16.29, 11.6, 19.2), (19.18, 17.6, 25.2), (22.07, 27.2, 35.2), (25.00, 39.0, 45.6)],
            ],
            lining_mm: 1.3,
            wear_m3_per_j: 4.6e-14,
            c_pressure_plate_j_k: [1600.0, 1500.0],
            c_central_j_k: 2300.0,
            c_disc_j_k: 450.0,
            c_housing_j_k: 6000.0,
            stack_mm: 30.0,
            inertia_drive_kgm2: 0.075,
            open_drag_nm_s: 2e-5,
        }
    }

    fn r_eff(&self, k: usize) -> f64 {
        0.5 * (self.r_outer_m[k] + self.r_inner_m[k])
    }
    fn face_area(&self, k: usize) -> f64 {
        PI * (self.r_outer_m[k].powi(2) - self.r_inner_m[k].powi(2))
    }
    /// Torque capacity `u` mm past the kiss point at the static μ of a new lining ([`Self::torque_curve`]), Nm.
    pub fn torque_at(&self, k: usize, u: f64) -> f64 {
        let [a0, a2, a4] = self.torque_curve[k];
        if u <= 0.0 {
            0.0
        } else {
            a0 * (u + ((u - a4).powi(2) + a2).sqrt() - (a4 * a4 + a2).sqrt())
        }
    }
    /// (release, apply) actuator pressure at travel `x` mm ([`Self::actuator_bar`]), bar: linear between the points,
    /// flat below the first, the last slope beyond the last.
    pub fn actuator_bar_at(&self, k: usize, x: f64) -> (f64, f64) {
        let t = &self.actuator_bar[k];
        if x <= t[0].0 {
            return (t[0].1, t[0].2);
        }
        let i = t.iter().position(|p| x <= p.0).unwrap_or(t.len() - 1);
        let (a, b) = (t[i - 1], t[i]);
        let s = (x - a.0) / (b.0 - a.0);
        (a.1 + s * (b.1 - a.1), a.2 + s * (b.2 - a.2))
    }
}

/// Speed factor of the dry μ at zero slip (static μ = `mu_nominal` · this for a new, cool lining).
const DRY_MU_STATIC: f64 = 0.92;

#[derive(Debug, Clone, Serialize)]
pub struct DryPack {
    pub t_pressure_plate: [f64; 2],
    pub t_central: f64,
    pub t_disc: [f64; 2],
    pub t_housing: f64,
    /// Friction interface (flash) temperature, °C.
    pub t_surface: [f64; 2],
    /// Total lining thickness lost (both faces), mm.
    pub wear_mm: [f64; 2],
    pub glaze: [f64; 2],
    pub lining_health: [f64; 2],
    pub plate_health: [f64; 2],
    pub warp_mm: [f64; 2],
    pub clamp_n: [f64; 2],
    last_power_w: [f64; 2],
}

// ---------------------------------------------------------------- wet ----------------------------

#[derive(Debug, Clone, Serialize)]
pub struct WetClutchHw {
    pub r_outer_m: f64,
    pub r_inner_m: f64,
    /// Number of friction surfaces z.
    pub surfaces: f64,
    pub mu_static: f64,
    pub mu_dynamic: f64,
    pub v_stribeck_m_s: f64,
    /// Relative μ change per K around 80 °C.
    pub mu_temp_coeff: f64,
    /// μ ∝ (p/p_ref)^−c.
    pub mu_pressure_exp: f64,
    pub p_ref_pa: f64,
    pub groove_fraction: f64,
    pub groove_depth_m: f64,
    /// Squeeze-film relief by grooves (1 = plain disc).
    pub squeeze_groove_factor: f64,
    /// Total pack clearance (new), m.
    pub clearance_m: f64,
    /// Gap per surface when the pack clearance is taken up, m.
    pub h_touch_m: f64,
    /// Gap below which the plates no longer carry a uniform oil film (asperities, separator waviness/coning): the shear
    /// there belongs to the boundary friction μ, the Couette term fades out linearly towards contact, m.
    pub h_mixed_m: f64,
    pub asp_f_ref_n: f64,
    pub asp_h_ref_m: f64,
    pub asp_sigma_m: f64,
    pub piston_r_in_m: f64,
    pub piston_r_out_m: f64,
    pub feed_r_m: f64,
    pub balance_r_m: f64,
    pub return_spring_n: f64,
    pub return_rate_n_m: f64,
    pub stroke_max_m: f64,
    pub c_pack_j_k: f64,
    /// Total lining wear allowance, m, and specific wear, m³/J.
    pub lining_m: f64,
    pub wear_m3_per_j: f64,
    pub pack_length_m: f64,
    /// Inner carrier + friction plates (input-shaft side).
    pub inertia_hub_kgm2: f64,
    /// Outer drum + steels + piston (drive side).
    pub inertia_drum_kgm2: f64,
}

impl WetClutchHw {
    /// DQ250 (02E) outer clutch K1. Friction geometry UNVERIFIED (BorgWarner DualTronic class). The piston annulus, return
    /// spring and balance-chamber radius are set so that the pack has the characteristic the TCU is calibrated for
    /// (0D9300012F, CAL block 0x8003806C, VERIFIED-S; docs/DQ250_FIRMWARE_SPEC.md §3): torque 0 at 2.70 bar (+0x168), 250 Nm
    /// at 8.995 bar (+0x180/+0x18C) = 39.7 Nm/bar, static μ of a new pack at 80 °C; residual centrifugal clamp at 6000 rpm
    /// = the firmware's 0x800957A0 term k·(n/6000)², k(70 °C) ≈ 1753 × 0.02 Nm = 35 Nm ≈ 0.88 bar (balance chamber full).
    /// The previous geometry (piston 80..98 mm, 900 N spring) touched at ≈ 1.1 bar with ≈ 80–100 Nm/bar: the firmware's
    /// "open but filled" 2–2.6 bar dragged the idling engine to a stall (VERIFIED-T, fifth DQ250 pass).
    pub fn dq250_k1() -> Self {
        Self {
            r_outer_m: 0.1025,
            r_inner_m: 0.0855,
            surfaces: 8.0,
            mu_static: 0.105,
            mu_dynamic: 0.13,
            v_stribeck_m_s: 0.6,
            mu_temp_coeff: 0.0012,
            mu_pressure_exp: 0.05,
            p_ref_pa: 1.0e6,
            groove_fraction: 0.35,
            groove_depth_m: 0.3e-3,
            squeeze_groove_factor: 0.1,
            clearance_m: 1.4e-3,
            h_touch_m: 25e-6,
            // UNVERIFIED (no plate data): set equal to the open pack gap h_open = h_touch_m + clearance_m / surfaces (200 µm
            // for K1, 175 µm for K2) so that as the piston closes across free clearance below the kiss point (2.21..2.65 bar),
            // `mixed = (h_gap / h_mixed_m).min(1.0)` decreases monotonically instead of creating a 21× viscous drag spike at
            // 2.5 bar that stalled the idling engine at 5 °C (0D9300012F CAL 0x8003806C+0xAC, VERIFIED-S; docs/DQ250_FIRMWARE_SPEC.md §12.1).
            h_mixed_m: 200e-6,
            asp_f_ref_n: 6000.0,
            asp_h_ref_m: 2.0e-6,
            asp_sigma_m: 0.8e-6,
            // fitted to the firmware characteristic (above; derivation in docs/DQ250_FIRMWARE_SPEC.md §3)
            piston_r_in_m: 0.0900,
            piston_r_out_m: 0.098,
            feed_r_m: 0.0900,
            balance_r_m: 0.09368,
            return_spring_n: 1045.0,
            return_rate_n_m: 150e3,
            stroke_max_m: 2.2e-3,
            c_pack_j_k: 1100.0,
            lining_m: 0.4e-3 * 8.0,
            wear_m3_per_j: 1.0e-15,
            pack_length_m: 0.024,
            inertia_hub_kgm2: 0.012,
            inertia_drum_kgm2: 0.04,
        }
    }
    /// DQ250 inner clutch K2: fitted like K1 to the firmware's K2 characteristic — torque 0 at 2.50 bar, 250 Nm at 7.955
    /// bar = 45.8 Nm/bar, centrifugal term k(70 °C) ≈ 704 × 0.02 Nm = 14 Nm ≈ 0.31 bar at 6000 rpm (VERIFIED-S, same CAL).
    pub fn dq250_k2() -> Self {
        Self {
            r_outer_m: 0.083,
            r_inner_m: 0.066,
            surfaces: 12.0,
            clearance_m: 1.8e-3,
            h_mixed_m: 175e-6,
            asp_f_ref_n: 5200.0,
            piston_r_in_m: 0.0668,
            piston_r_out_m: 0.077,
            feed_r_m: 0.0668,
            balance_r_m: 0.06825,
            return_spring_n: 848.0,
            stroke_max_m: 2.6e-3,
            c_pack_j_k: 1000.0,
            lining_m: 0.4e-3 * 12.0,
            pack_length_m: 0.032,
            inertia_hub_kgm2: 0.008,
            inertia_drum_kgm2: 0.02,
            ..Self::dq250_k1()
        }
    }
    /// DQ381 (0GC): radially nested like DQ250, larger K1 pack. UNVERIFIED (no firmware: keeps the generic piston/spring
    /// of the earlier DQ250 model, not the DQ250 firmware fit).
    pub fn dq381_k1() -> Self {
        Self {
            surfaces: 10.0,
            h_mixed_m: 25e-6 + 1.6e-3 / 10.0,
            clearance_m: 1.6e-3,
            lining_m: 0.4e-3 * 10.0,
            pack_length_m: 0.028,
            c_pack_j_k: 1300.0,
            piston_r_in_m: 0.080,
            feed_r_m: 0.080,
            balance_r_m: 0.0805,
            return_spring_n: 900.0,
            ..Self::dq250_k1()
        }
    }
    pub fn dq381_k2() -> Self {
        Self {
            surfaces: 14.0,
            h_mixed_m: 25e-6 + 2.0e-3 / 14.0,
            clearance_m: 2.0e-3,
            lining_m: 0.4e-3 * 14.0,
            pack_length_m: 0.036,
            c_pack_j_k: 1150.0,
            piston_r_in_m: 0.055,
            feed_r_m: 0.055,
            balance_r_m: 0.0555,
            return_spring_n: 900.0,
            ..Self::dq250_k2()
        }
    }
    /// Haldex Gen 5 coupling pack. UNVERIFIED.
    pub fn haldex() -> Self {
        Self {
            r_outer_m: 0.062,
            r_inner_m: 0.045,
            h_mixed_m: 25e-6 + 1.0e-3 / 16.0,
            surfaces: 16.0,
            clearance_m: 1.0e-3,
            asp_f_ref_n: 12000.0,
            piston_r_in_m: 0.030,
            piston_r_out_m: 0.050,
            feed_r_m: 0.0,
            balance_r_m: 0.0,
            return_spring_n: 300.0,
            stroke_max_m: 1.6e-3,
            c_pack_j_k: 900.0,
            lining_m: 0.3e-3 * 16.0,
            pack_length_m: 0.03,
            inertia_hub_kgm2: 0.004,
            inertia_drum_kgm2: 0.006,
            ..Self::dq250_k1()
        }
    }

    pub fn r_eff(&self) -> f64 {
        0.5 * (self.r_outer_m + self.r_inner_m)
    }
    pub fn face_area(&self) -> f64 {
        PI * (self.r_outer_m.powi(2) - self.r_inner_m.powi(2))
    }
    pub fn piston_area(&self) -> f64 {
        PI * (self.piston_r_out_m.powi(2) - self.piston_r_in_m.powi(2))
    }
    /// Centrifugal force on a rotating annular piston fed at radius `r_feed` (oil column from
    /// r_feed outwards): F = πρω²/4·[(r_o² − r_f²)² − (r_i² − r_f²)²].
    pub fn centrifugal_force(&self, rho: f64, omega: f64, r_feed: f64) -> f64 {
        let (ro2, ri2, rf2) = (self.piston_r_out_m.powi(2), self.piston_r_in_m.powi(2), r_feed.powi(2));
        PI * rho * omega * omega / 4.0 * ((ro2 - rf2).max(0.0).powi(2) - (ri2 - rf2).max(0.0).powi(2))
    }
    fn squeeze_coeff(&self) -> f64 {
        let (ro, ri) = (self.r_outer_m, self.r_inner_m);
        1.5 * PI * (ro.powi(4) - ri.powi(4) - (ro * ro - ri * ri).powi(2) / (ro / ri).ln()) * self.squeeze_groove_factor
    }
    fn asperity_force(&self, h: f64) -> f64 {
        self.asp_f_ref_n * (-(h - self.asp_h_ref_m) / self.asp_sigma_m).exp().min(1e3)
    }
    /// Effective pack clearance after wear (`wear_m` per friction surface, as [`WetPack::wear_m`]: every surface adds
    /// its wear to the stack) and thermal expansion (steel α = 12e-6 1/K).
    fn clearance_eff(&self, wear_m: f64, t_pack_c: f64) -> f64 {
        self.clearance_m + self.surfaces * wear_m - 12e-6 * self.pack_length_m * (t_pack_c - 20.0)
    }
    /// Gap per friction surface at piston travel x.
    fn gap(&self, x: f64, wear_m: f64, t_pack_c: f64) -> f64 {
        (self.h_touch_m + (self.clearance_eff(wear_m, t_pack_c) - x) / self.surfaces).max(0.05e-6)
    }

    /// Centrifugal outflow of the whole pack wetted from r_i out to `r`: the flow that keeps that film (Yuan et al. 2007),
    /// m³/s. No outflow below 1 rad/s.
    pub fn film_outflow(&self, r: f64, h: f64, omega_c: f64, fl: &Fluid, oil_c: f64) -> f64 {
        let (h, ri) = (h.max(25e-6), self.r_inner_m);
        if omega_c.abs() < 1.0 || r <= ri {
            return 0.0;
        }
        self.surfaces * PI * h.powi(3) * fl.rho * omega_c * omega_c * (r * r - ri * ri) / (40.0 * fl.mu(oil_c) * (r / ri).ln().max(1e-9))
    }
    /// Oil in the pack's gaps (lands and grooves) wetted from r_i out to `r`, m³.
    pub fn film_volume(&self, r: f64, h: f64) -> f64 {
        (r * r - self.r_inner_m.powi(2)).max(0.0) * self.film_m3_per_m2(h)
    }
    /// Gap volume of the pack per m² of r² (z·π·(h + groove share)).
    fn film_m3_per_m2(&self, h: f64) -> f64 {
        self.surfaces * PI * (h.max(25e-6) + self.groove_fraction * self.groove_depth_m)
    }
    /// Radius out to which `oil_m3` wets the gaps.
    pub fn wetted_radius(&self, oil_m3: f64, h: f64) -> f64 {
        (self.r_inner_m.powi(2) + oil_m3.max(0.0) / self.film_m3_per_m2(h)).sqrt().min(self.r_outer_m)
    }
    /// Oil the pack holds at steady state with `q_pack` flowing through it (outflow = inflow), m³.
    pub fn steady_oil_m3(&self, q_pack: f64, omega_c: f64, h: f64, fl: &Fluid, oil_c: f64) -> f64 {
        let (ri, ro) = (self.r_inner_m, self.r_outer_m);
        if omega_c.abs() < 1.0 || self.film_outflow(ro, h, omega_c, fl, oil_c) <= q_pack {
            return self.film_volume(ro, h);
        }
        let (mut lo, mut hi) = (ri, ro);
        for _ in 0..18 {
            let m = 0.5 * (lo + hi);
            if self.film_outflow(m, h, omega_c, fl, oil_c) > q_pack.max(0.0) {
                hi = m;
            } else {
                lo = m;
            }
        }
        self.film_volume(lo, h)
    }

    /// Open-clutch / slipping viscous torque of the whole pack holding `oil_m3` in its gaps (signed with `dw`). The oil is
    /// a state ([`WetPack::oil_m3`]): it fills from the cooling flow and is flung out at [`Self::film_outflow`], so a
    /// short cooling pulse wets only the inner part of the gaps (the steady state is Yuan's balance).
    pub fn viscous_torque(&self, dw: f64, h: f64, oil_m3: f64, fl: &Fluid, oil_c: f64) -> f64 {
        let mu = fl.mu(oil_c);
        let h_gap = h;
        let h = h.max(self.h_mixed_m);
        let (ro, ri) = (self.r_outer_m, self.r_inner_m);
        let re = self.wetted_radius(oil_m3, h);
        // Below `h_mixed_m` the film is carried by asperities (mixed lubrication); its shear is part of
        // the measured boundary μ, so the separate viscous term fades out linearly.
        let mixed = (h_gap / self.h_mixed_m).min(1.0);
        let g = self.groove_fraction;
        let land = mixed * ((1.0 - g) / h + g / (h + self.groove_depth_m));
        // Aerated zone r_e..r_o: oil/air mixture with ≈ 3 % effective viscosity (Iqbal et al. 2013
        // treat it as a two-phase region; residual drag plateau at high speed).
        let r4 = re.powi(4) - ri.powi(4) + 0.03 * (ro.powi(4) - re.powi(4));
        self.surfaces * PI * mu * dw * r4 / 2.0 * land
    }

    /// Boundary friction coefficient of the paper lining.
    pub fn mu(&self, v_m_s: f64, t_c: f64, p_contact: f64, atf_deg: f64, glaze: f64, health: f64) -> f64 {
        let mu_s = self.mu_static + 0.05 * atf_deg;
        let mu_d = self.mu_dynamic - 0.01 * atf_deg;
        let mv = mu_s + (mu_d - mu_s) * (1.0 - (-v_m_s.abs() / self.v_stribeck_m_s).exp());
        let ft = (1.0 - self.mu_temp_coeff * (t_c - 80.0)).clamp(0.6, 1.15);
        let fp = (p_contact.max(1e4) / self.p_ref_pa).powf(-self.mu_pressure_exp).clamp(0.85, 1.15);
        mv * ft * fp * (1.0 - glaze) * (0.25 + 0.75 * health)
    }
}

/// Heat path of a wet pack that gets no cooling oil (carriers, drum and piston into the oil-misted clutch housing), W/K.
/// UNVERIFIED (≈ 200 W/m²K splash convection over ≈ 0.1 m² of steel). The packs get oil only through the cooling oil
/// slide valve (VW SSP 308 p. 38), so this is all that cools them while the TCU keeps the valve shut.
const PACK_HOUSING_G_W_K: f64 = 20.0;

#[derive(Debug, Clone, Serialize)]
pub struct WetPack {
    pub t_pack: [f64; 2],
    pub t_surface: [f64; 2],
    /// Lining wear per friction surface, m (the lining is gone at `lining_m / surfaces`).
    pub wear_m: [f64; 2],
    pub glaze: [f64; 2],
    pub lining_health: [f64; 2],
    /// Steel plate coning/warping, m (adds contact when open).
    pub warp_m: [f64; 2],
    pub h_m: [f64; 2],
    pub f_contact_n: [f64; 2],
    /// Oil held in each pack's gaps, m³ ([`WetClutchHw::viscous_torque`]); drained at power-up.
    pub oil_m3: [f64; 2],
}

// ---------------------------------------------------------------- common -------------------------

#[derive(Debug, Clone, Serialize)]
pub enum ClutchHw {
    Dry(DryClutchHw),
    Wet([WetClutchHw; 2]),
}

impl ClutchHw {
    /// Inertia rotating with the DMF secondary.
    pub fn drive_inertia(&self) -> f64 {
        match self {
            ClutchHw::Dry(h) => h.inertia_drive_kgm2,
            ClutchHw::Wet(h) => h[0].inertia_drum_kgm2 + h[1].inertia_drum_kgm2,
        }
    }
    pub fn hub_inertia(&self, k: usize) -> f64 {
        match self {
            ClutchHw::Dry(_) => 0.009,
            ClutchHw::Wet(h) => h[k].inertia_hub_kgm2,
        }
    }
    pub fn actuator_area(&self, k: usize) -> f64 {
        match self {
            ClutchHw::Dry(h) => h.actuator_area_m2,
            ClutchHw::Wet(h) => h[k].piston_area(),
        }
    }
    /// Film suction resisting plate separation, limited by cavitation (≈ 0.5 bar over a face), N.
    pub fn separation_limit_n(&self, k: usize) -> f64 {
        match self {
            ClutchHw::Dry(_) => 0.0,
            ClutchHw::Wet(h) => 0.5e5 * h[k].face_area(),
        }
    }
    /// Coulomb friction of the actuator path at travel `x_m`, N (dry: half the apply/release hysteresis of
    /// [`DryClutchHw::actuator_bar`]; wet pistons: none).
    pub fn actuator_friction_n(&self, k: usize, x_m: f64) -> f64 {
        match self {
            ClutchHw::Dry(h) => {
                let (release, apply) = h.actuator_bar_at(k, x_m * 1e3);
                0.5e5 * (apply - release).max(0.0) * h.actuator_area_m2
            }
            ClutchHw::Wet(_) => 0.0,
        }
    }
    /// Actuator / piston stroke end stop, m.
    pub fn stroke_max(&self, k: usize) -> f64 {
        match self {
            ClutchHw::Dry(h) => h.actuator_travel_max_mm * 1e-3,
            ClutchHw::Wet(h) => h[k].stroke_max_m,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub enum ClutchPack {
    Dry(DryPack),
    Wet(WetPack),
}

/// Organic facing μ temperature factor (fade above 150 °C, strong above 250 °C).
fn dry_fade(t: f64) -> f64 {
    let f = if t <= 150.0 {
        1.0
    } else if t <= 250.0 {
        1.0 - 0.0008 * (t - 150.0)
    } else {
        0.92 - 0.0032 * (t - 250.0)
    };
    f.max(0.3)
}

impl ClutchPack {
    pub fn new(hw: &ClutchHw, t_c: f64) -> Self {
        match hw {
            ClutchHw::Dry(_) => ClutchPack::Dry(DryPack {
                t_pressure_plate: [t_c; 2],
                t_central: t_c,
                t_disc: [t_c; 2],
                t_housing: t_c,
                t_surface: [t_c; 2],
                wear_mm: [0.0; 2],
                glaze: [0.0; 2],
                lining_health: [1.0; 2],
                plate_health: [1.0; 2],
                warp_mm: [0.0; 2],
                clamp_n: [0.0; 2],
                last_power_w: [0.0; 2],
            }),
            ClutchHw::Wet(h) => ClutchPack::Wet(WetPack {
                t_pack: [t_c; 2],
                t_surface: [t_c; 2],
                wear_m: [0.0; 2],
                glaze: [0.0; 2],
                lining_health: [1.0; 2],
                warp_m: [0.0; 2],
                h_m: [h[0].h_touch_m + h[0].clearance_m / h[0].surfaces, h[1].h_touch_m + h[1].clearance_m / h[1].surfaces],
                f_contact_n: [0.0; 2],
                oil_m3: [0.0; 2],
            }),
        }
    }

    /// Kiss point in actuator/piston travel, m (where the contact force reaches ≈ 50 N).
    pub fn kiss_point_m(&self, hw: &ClutchHw, k: usize) -> f64 {
        match (self, hw) {
            (ClutchPack::Dry(s), ClutchHw::Dry(h)) => Self::dry_kiss_mm(s, h, k) * 1e-3,
            (ClutchPack::Wet(s), ClutchHw::Wet(h)) => {
                let h = &h[k];
                let h_kiss = h.asp_h_ref_m + h.asp_sigma_m * (h.asp_f_ref_n / 50.0).ln();
                h.clearance_eff(s.wear_m[k], s.t_pack[k]) + h.surfaces * (h.h_touch_m - h_kiss)
            }
            _ => 0.0,
        }
    }

    fn dry_kiss_mm(s: &DryPack, h: &DryClutchHw, k: usize) -> f64 {
        let t_avg = 0.5 * (s.t_pressure_plate[k] + s.t_central);
        h.kiss_mm[k] + h.lever_ratio * (s.wear_mm[k] - 11e-6 * h.stack_mm * (t_avg - 20.0))
    }

    fn dry_clamp(s: &DryPack, h: &DryClutchHw, k: usize, x_mm: f64) -> f64 {
        let u = x_mm - Self::dry_kiss_mm(s, h, k);
        let f = h.torque_at(k, u) / (2.0 * h.mu_nominal * DRY_MU_STATIC * h.r_eff(k));
        // Warped pressure plate touches the disc over part of a revolution even when "open".
        let gap = (-u / h.lever_ratio).max(0.0);
        f + 1500.0 * (s.warp_mm[k] - gap).max(0.0)
    }

    /// External force on the clutch actuator / piston along the apply direction at travel `x_m`
    /// (springs, clamp reaction, centrifugal head), N.
    pub fn piston_force(&self, hw: &ClutchHw, k: usize, x_m: f64, env: &ClutchEnv, fluid: &Fluid) -> f64 {
        match (self, hw) {
            (ClutchPack::Dry(_), ClutchHw::Dry(h)) => {
                let (release, apply) = h.actuator_bar_at(k, x_m * 1e3);
                -0.5e5 * (release + apply) * h.actuator_area_m2
            }
            (ClutchPack::Wet(s), ClutchHw::Wet(h)) => {
                let h = &h[k];
                let gap = h.gap(x_m, s.wear_m[k], s.t_pack[k]);
                let f_cf = h.centrifugal_force(fluid.rho, env.omega_drum, h.feed_r_m) - env.balance_fill * h.centrifugal_force(fluid.rho, env.omega_drum, h.balance_r_m);
                -(h.return_spring_n + h.return_rate_n_m * x_m.max(0.0)) - h.asperity_force(gap) + f_cf
            }
            _ => 0.0,
        }
    }

    /// Velocity damping of the piston from the squeeze film (wet), N·s/m.
    pub fn piston_damping(&self, hw: &ClutchHw, k: usize, x_m: f64, fluid: &Fluid, oil_c: f64) -> f64 {
        match (self, hw) {
            (ClutchPack::Wet(s), ClutchHw::Wet(h)) => {
                let h = &h[k];
                let gap = h.gap(x_m, s.wear_m[k], s.t_pack[k]);
                (h.squeeze_coeff() * fluid.mu(oil_c) / (h.surfaces * gap.powi(3))).min(5e8)
            }
            _ => 0.0,
        }
    }

    /// Updates contact force from the actuator travel (start of a sub-step).
    pub fn set_travel(&mut self, hw: &ClutchHw, k: usize, x_m: f64) {
        match (self, hw) {
            (ClutchPack::Dry(s), ClutchHw::Dry(h)) => s.clamp_n[k] = Self::dry_clamp(s, h, k, x_m * 1e3),
            (ClutchPack::Wet(s), ClutchHw::Wet(h)) => {
                let hk = &h[k];
                let gap = hk.gap(x_m, s.wear_m[k], s.t_pack[k]);
                s.h_m[k] = gap;
                // Coned/warped steels rub when open.
                let warp_contact = 4e6 * (s.warp_m[k] - (gap - hk.h_touch_m).max(0.0)).max(0.0);
                s.f_contact_n[k] = hk.asperity_force(gap) + warp_contact;
            }
            _ => {}
        }
    }

    pub fn contact_force(&self, k: usize) -> f64 {
        match self {
            ClutchPack::Dry(s) => s.clamp_n[k],
            ClutchPack::Wet(s) => s.f_contact_n[k],
        }
    }

    fn dry_mu(s: &DryPack, h: &DryClutchHw, k: usize, v: f64) -> f64 {
        let f_v = DRY_MU_STATIC + (1.0 - DRY_MU_STATIC) * (1.0 - (-v.abs() / 2.0).exp());
        h.mu_nominal * f_v * dry_fade(s.t_surface[k]) * (1.0 - s.glaze[k]) * (0.35 + 0.65 * s.lining_health[k]) * (0.3 + 0.7 * s.plate_health[k])
    }

    /// Torque the clutch can hold without slipping (static friction), Nm.
    pub fn static_capacity(&self, hw: &ClutchHw, k: usize, env: &ClutchEnv) -> f64 {
        match (self, hw) {
            (ClutchPack::Dry(s), ClutchHw::Dry(h)) => 2.0 * Self::dry_mu(s, h, k, 0.0) * s.clamp_n[k] * h.r_eff(k),
            (ClutchPack::Wet(s), ClutchHw::Wet(h)) => {
                let h = &h[k];
                let p = s.f_contact_n[k] / h.face_area();
                h.surfaces * h.mu(0.0, s.t_surface[k], p, env.atf_degradation, s.glaze[k], s.lining_health[k]) * s.f_contact_n[k] * h.r_eff()
            }
            _ => 0.0,
        }
    }

    /// Transmitted torque while slipping with `dw` = drive − hub speed (signed like `dw`).
    /// `slip_angle` drives the warped-plate judder modulation.
    pub fn slip_torque(&self, hw: &ClutchHw, k: usize, dw: f64, slip_angle: f64, env: &ClutchEnv, fluid: &Fluid) -> f64 {
        let sgn = if dw > 0.0 { 1.0 } else if dw < 0.0 { -1.0 } else { 0.0 };
        match (self, hw) {
            (ClutchPack::Dry(s), ClutchHw::Dry(h)) => {
                let r = h.r_eff(k);
                let judder = 1.0 + (s.warp_mm[k] / 0.2).min(1.0) * 0.4 * slip_angle.sin();
                let t = 2.0 * Self::dry_mu(s, h, k, dw * r) * s.clamp_n[k] * r * judder;
                t * sgn + h.open_drag_nm_s * dw
            }
            (ClutchPack::Wet(s), ClutchHw::Wet(h)) => {
                let hk = &h[k];
                let r = hk.r_eff();
                let p = s.f_contact_n[k] / hk.face_area();
                let mu = hk.mu(dw * r, s.t_surface[k], p, env.atf_degradation, s.glaze[k], s.lining_health[k]);
                let visc = hk.viscous_torque(dw, s.h_m[k], s.oil_m3[k], fluid, env.oil_c);
                hk.surfaces * mu * s.f_contact_n[k] * r * sgn + visc
            }
            _ => 0.0,
        }
    }

    /// Thermal network, wear and failure for one sub-step. `p_slip` friction power per clutch, W.
    /// Returns heat rejected into the gearbox fluid / case, W.
    pub fn thermal_step(&mut self, hw: &ClutchHw, p_slip: [f64; 2], env: &ClutchEnv, fluid: &Fluid, dt: f64) -> f64 {
        match (self, hw) {
            (ClutchPack::Dry(s), ClutchHw::Dry(h)) => {
                let wd = env.omega_drum.abs().sqrt();
                let g_pp = 2.0 + 0.25 * wd;
                let mut q_cp = 0.0;
                let mut to_housing_disc = 0.0;
                for (k, &ps) in p_slip.iter().enumerate() {
                    let p = ps.max(0.0);
                    s.last_power_w[k] = p;
                    let g_disc = 0.5 + 0.05 * env.omega_hub[k].abs().sqrt();
                    let q_pp = 0.465 * p - g_pp * (s.t_pressure_plate[k] - s.t_housing);
                    s.t_pressure_plate[k] += q_pp / h.c_pressure_plate_j_k[k] * dt;
                    let q_disc_loss = g_disc * (s.t_disc[k] - s.t_housing);
                    let q_disc = 0.07 * p - q_disc_loss;
                    s.t_disc[k] += q_disc / h.c_disc_j_k * dt;
                    to_housing_disc += q_disc_loss;
                    q_cp += 0.465 * p;
                    s.t_surface[k] = 0.5 * (s.t_pressure_plate[k] + s.t_central) + 1.2e-3 * p;
                }
                let g_cp = 2.5 + 0.35 * wd;
                s.t_central += (q_cp - g_cp * (s.t_central - s.t_housing)) / h.c_central_j_k * dt;
                let to_housing = g_pp * (s.t_pressure_plate[0] + s.t_pressure_plate[1] - 2.0 * s.t_housing)
                    + g_cp * (s.t_central - s.t_housing)
                    + to_housing_disc;
                let g_amb = 12.0 + 1.5 * env.vehicle_speed_ms.abs();
                s.t_housing += (to_housing - g_amb * (s.t_housing - env.ambient_c)) / h.c_housing_j_k * dt;
                for k in 0..2 {
                    let ts = s.t_surface[k];
                    let p = s.last_power_w[k];
                    s.wear_mm[k] += h.wear_m3_per_j * p * ((ts - 200.0).max(0.0) / 60.0).exp() / h.face_area(k) * 1e3 * dt;
                    if ts > 280.0 {
                        s.glaze[k] = (s.glaze[k] + 0.004 * ((ts - 280.0) / 40.0).powi(2) * dt).min(0.6);
                    }
                    if ts > 380.0 {
                        s.lining_health[k] = (s.lining_health[k] - 0.01 * ((ts - 380.0) / 40.0).powi(2) * dt).max(0.0);
                    }
                    if s.wear_mm[k] >= 2.0 * h.lining_mm {
                        s.lining_health[k] = 0.0; // rivets on the plates
                    }
                    let t_plate = s.t_pressure_plate[k].max(s.t_central);
                    if t_plate > 330.0 {
                        s.warp_mm[k] += 1e-3 * ((t_plate - 330.0) / 50.0).powi(2) * dt;
                    }
                    if ts > 500.0 || (s.lining_health[k] == 0.0 && p > 1000.0) {
                        s.plate_health[k] = (s.plate_health[k] - 0.05 * (1.0 + ((ts - 500.0).max(0.0) / 50.0).powi(2)) * dt).max(0.0);
                    }
                }
                // Dry clutch bell housing is open to air; nothing goes into the gear oil.
                0.0
            }
            (ClutchPack::Wet(s), ClutchHw::Wet(h)) => {
                let mut to_oil = 0.0;
                for k in 0..2 {
                    let hk = &h[k];
                    let p = p_slip[k].max(0.0);
                    let mdot = env.cool_flow_m3_s[k].max(0.0) * fluid.rho;
                    // Oil in the gaps: cooling flow in, centrifugal outflow of the wetted film out (using the same
                    // `h.max(h_mixed_m)` waviness floor as `viscous_torque` so closing across free clearance does not choke outflow).
                    let (h_k, omega_c) = (s.h_m[k].max(hk.h_mixed_m), 0.5 * (env.omega_drum.abs() + env.omega_hub[k].abs()));
                    let out = hk.film_outflow(hk.wetted_radius(s.oil_m3[k], h_k), h_k, omega_c, fluid, env.oil_c);
                    s.oil_m3[k] = (s.oil_m3[k] + (env.cool_flow_m3_s[k].max(0.0) - out) * dt).clamp(0.0, hk.film_volume(hk.r_outer_m, h_k));
                    // Cooling oil through the grooves: ε–NTU with hA ≈ 150 W/K at full flow; without it the pack still loses
                    // heat through its carriers into the oil-misted clutch housing (`PACK_HOUSING_G_W_K`).
                    let g_cool = if mdot > 0.0 { mdot * fluid.cp * (1.0 - (-150.0 / (mdot * fluid.cp)).exp()) } else { 0.0 } + PACK_HOUSING_G_W_K;
                    let q_out = g_cool * (s.t_pack[k] - env.oil_c);
                    s.t_pack[k] += (p - q_out) / hk.c_pack_j_k * dt;
                    to_oil += q_out;
                    s.t_surface[k] = s.t_pack[k] + 0.6e-3 * p;
                    let ts = s.t_surface[k];
                    s.wear_m[k] += hk.wear_m3_per_j * p * ((ts - 180.0).max(0.0) / 50.0).exp() / hk.face_area() / hk.surfaces * dt;
                    if ts > 300.0 {
                        s.glaze[k] = (s.glaze[k] + 0.005 * ((ts - 300.0) / 40.0).powi(2) * dt).min(0.6);
                    }
                    if ts > 380.0 {
                        s.lining_health[k] = (s.lining_health[k] - 0.015 * ((ts - 380.0) / 40.0).powi(2) * dt).max(0.0);
                    }
                    if s.wear_m[k] * hk.surfaces >= hk.lining_m {
                        s.lining_health[k] = 0.0;
                    }
                    if s.t_pack[k] > 330.0 {
                        s.warp_m[k] += 2e-6 * ((s.t_pack[k] - 330.0) / 50.0).powi(2) * dt;
                    }
                }
                to_oil
            }
            _ => 0.0,
        }
    }

    pub fn temps(&self) -> ([f64; 2], [f64; 2]) {
        match self {
            ClutchPack::Dry(s) => ([0.5 * (s.t_pressure_plate[0] + s.t_central), 0.5 * (s.t_pressure_plate[1] + s.t_central)], s.t_surface),
            ClutchPack::Wet(s) => (s.t_pack, s.t_surface),
        }
    }

    /// (lining health, glaze, wear mm, warp mm) per clutch.
    pub fn condition(&self) -> [(f64, f64, f64, f64); 2] {
        match self {
            ClutchPack::Dry(s) => [0, 1].map(|k| (s.lining_health[k] * s.plate_health[k], s.glaze[k], s.wear_mm[k], s.warp_mm[k])),
            ClutchPack::Wet(s) => [0, 1].map(|k| (s.lining_health[k], s.glaze[k], s.wear_m[k] * 1e3, s.warp_m[k] * 1e3)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The new dry clutch at 20 °C holds what DQ200_G2 learned (EEPROM 0x880..0x8BF, `DryClutchHw::dq200`):
    /// nothing below the kiss point, the support-point torques at their travels, rising in between.
    #[test]
    fn dry_capacity_is_the_learned_characteristic() {
        let hw = ClutchHw::Dry(DryClutchHw::dq200());
        let mut p = ClutchPack::new(&hw, 20.0);
        let env = ClutchEnv { ambient_c: 20.0, vehicle_speed_ms: 0.0, omega_drum: 0.0, omega_hub: [0.0; 2], oil_c: 20.0, atf_degradation: 0.0, cool_flow_m3_s: [0.0; 2], balance_fill: 1.0 };
        let mut cap = |k: usize, mm: f64| {
            p.set_travel(&hw, k, mm * 1e-3);
            p.static_capacity(&hw, k, &env)
        };
        let support = [[(4.86, 0.0), (6.75, 39.2), (13.33, 279.2), (20.52, 669.2)], [(7.87, 0.0), (11.98, 37.5), (20.20, 450.1), (23.05, 700.0)]];
        for (k, sup) in support.iter().enumerate() {
            for &(mm, nm) in sup {
                let c = cap(k, mm);
                assert!((c - nm).abs() <= 0.07 * nm + 1e-6, "K{} {mm} mm: {c:.1} Nm, learned {nm}", k + 1);
            }
            assert_eq!(cap(k, sup[0].0 - 0.01), 0.0, "K{} open below the kiss point", k + 1);
            let mut last = 0.0;
            for i in 0..=290 {
                let c = cap(k, i as f64 * 0.1);
                assert!(c >= last, "K{} not monotonic at {} mm", k + 1, i as f64 * 0.1);
                last = c;
            }
        }
        // the actuator path: apply above release everywhere, so friction is never negative
        let ClutchHw::Dry(h) = &hw else { unreachable!() };
        for k in 0..2 {
            for i in 0..=290 {
                let (rel, app) = h.actuator_bar_at(k, i as f64 * 0.1);
                assert!(app > rel && rel > 0.0, "K{} {} mm: release {rel} apply {app}", k + 1, i as f64 * 0.1);
            }
        }
    }

    #[test]
    fn wet_open_drag_rises_then_falls_with_speed_and_is_higher_cold() {
        let h = WetClutchHw::dq250_k1();
        let fl = Fluid::ATF;
        let gap = h.h_touch_m + h.clearance_m / h.surfaces;
        let q = 1.5e-3 / 60.0; // 1.5 l/min lube through the pack
        let drag = |rpm: f64, t: f64| {
            let w = rpm * PI / 30.0;
            h.viscous_torque(w, gap, h.steady_oil_m3(q, w, gap, &fl, t), &fl, t)
        };
        let curve: Vec<(f64, f64)> = (1..=60).map(|i| (i as f64 * 100.0, drag(i as f64 * 100.0, 40.0))).collect();
        let peak = curve.iter().cloned().fold((0.0, 0.0), |a, b| if b.1 > a.1 { b } else { a });
        assert!(peak.0 > 200.0 && peak.0 < 3000.0, "drag peak at {} rpm", peak.0);
        assert!(curve[59].1 < 0.6 * peak.1, "aeration must reduce drag at high speed: {:?} {:?}", peak, curve[59]);
        assert!(drag(1000.0, 0.0) > 2.0 * drag(1000.0, 90.0));
        assert!((0.5..30.0).contains(&peak.1), "drag {}", peak.1);
    }

    #[test]
    fn wet_pack_clearance_grows_by_the_wear_of_all_surfaces() {
        let hw = ClutchHw::Wet([WetClutchHw::dq250_k1(), WetClutchHw::dq250_k2()]);
        let mut pack = ClutchPack::new(&hw, 20.0);
        let new = pack.kiss_point_m(&hw, 0);
        let ClutchPack::Wet(s) = &mut pack else { unreachable!() };
        s.wear_m[0] = 0.1e-3 / WetClutchHw::dq250_k1().surfaces; // 0.1 mm off the whole pack
        assert!((pack.kiss_point_m(&hw, 0) - new - 0.1e-3).abs() < 1e-9);
    }
}
