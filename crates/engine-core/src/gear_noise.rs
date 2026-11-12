//! A gearbox's own sound: the whine of its meshing teeth at their mesh orders and the rattle of
//! its loose gears in their backlash.
//!
//! Geometry: standard involute teeth (ISO 21771; Niemann & Winter, *Maschinenelemente* II, §21):
//! the transverse contact ratio ε_α counts how many tooth pairs share the load along the profile,
//! the overlap ratio ε_β how far a helical tooth's contact spreads along the face. The total
//! length of the contact lines — what the mesh stiffness follows — is the closed form of Maatar &
//! Velex, "An analytical expression for the time-varying contact length in perfect cylindrical
//! gears", *Mech. Mach. Theory* 31(5), 1996: the lines are parallel across the plane of action.

use crate::silencer2d::Biquad;

use crate::drivetrain::VehicleDef;
use serde::{Deserialize, Serialize};

/// Pressure angle of the standard full-depth tooth (ISO 53), rad.
const PRESSURE_ANGLE: f64 = 20.0 * std::f64::consts::PI / 180.0;

/// One gear pair: `z1` teeth driving `z2`, normal module, helix angle and face width.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mesh {
    pub z1: u32,
    pub z2: u32,
    pub module_m: f64,
    pub helix_rad: f64,
    pub face_m: f64,
}

impl Mesh {
    /// Transverse contact ratio of standard full-depth teeth (addendum = one module).
    pub fn eps_alpha(&self) -> f64 {
        let (mt, at) = self.transverse();
        let a = 0.5 * mt * f64::from(self.z1 + self.z2);
        let flank = |z: u32| {
            let r = 0.5 * mt * f64::from(z);
            let (ra, rb) = (r + self.module_m, r * at.cos());
            (ra * ra - rb * rb).max(0.0).sqrt()
        };
        (flank(self.z1) + flank(self.z2) - a * at.sin()) / (std::f64::consts::PI * mt * at.cos())
    }

    /// Transverse module and pressure angle of a helical pair (equal to the normal ones on a
    /// spur pair).
    fn transverse(&self) -> (f64, f64) {
        let c = self.helix_rad.cos();
        (self.module_m / c, (PRESSURE_ANGLE.tan() / c).atan())
    }

    /// Overlap ratio: how many axial pitches the face spans.
    pub fn eps_beta(&self) -> f64 {
        self.face_m * self.helix_rad.sin().abs() / (std::f64::consts::PI * self.module_m)
    }

    /// Base radius of gear `which` (0: the driver, 1: the driven), m.
    pub fn base_radius_m(&self, which: usize) -> f64 {
        let (mt, at) = self.transverse();
        let z = if which == 0 { self.z1 } else { self.z2 };
        0.5 * mt * f64::from(z) * at.cos()
    }
}

/// Total length of the contact lines at mesh phase `t` (in base pitches), in face widths.
///
/// Line `k` sits at `t + k` on the profile at one end of the face and slides back by `eps_beta`
/// across it; its length inside the plane of action is the share `y ∈ [0, 1]` of the face where
/// `0 ≤ t + k − eps_beta·y ≤ eps_alpha`. A spur pair (`eps_beta` = 0) just counts the pairs.
pub fn contact_length(t: f64, eps_alpha: f64, eps_beta: f64) -> f64 {
    let t = t.rem_euclid(1.0);
    let first = -(eps_alpha.ceil() as i64) - 1;
    let last = (eps_alpha + eps_beta).ceil() as i64 + 1;
    (first..=last)
        .map(|k| {
            let s = t + k as f64;
            if eps_beta <= 0.0 {
                f64::from(u8::from((0.0..=eps_alpha).contains(&s)))
            } else {
                let lo = ((s - eps_alpha) / eps_beta).max(0.0);
                let hi = (s / eps_beta).min(1.0);
                (hi - lo).max(0.0)
            }
        })
        .sum()
}

/// Tooth counts (driver, driven) for `ratio` with `sum` teeth per pair.
///
/// ESTIMATE: a car box's pairs all span one centre distance, so each keeps the same tooth sum;
/// no pinion goes under 12 teeth (undercut of a 20° tooth starts near 17, and boxes go below it
/// with profile shift).
pub fn teeth_for(ratio: f64, sum: u32) -> (u32, u32) {
    let z1 = (f64::from(sum) / (1.0 + ratio.max(0.0))).round() as u32;
    let z1 = z1.clamp(12, sum.saturating_sub(12).max(12));
    (z1, sum.saturating_sub(z1))
}

/// Tooth-contact stiffness of steel gears per metre of face for one pair in contact, N/m per m.
///
/// ISO 6336-1 (2019), Annex: the single-pair stiffness c′ of solid steel spur gears is about
/// 14 N/(mm·µm); the same figure serves a helical pair per unit contact length.
pub const SINGLE_PAIR_STIFFNESS: f64 = 14e9;

/// Damping ratio of a gear mesh.
///
/// ESTIMATE: Kahraman & Singh, "Non-linear dynamics of a spur gear pair", *J. Sound Vib.*
/// 142(1), 1990, measured mesh damping ratios between 0.03 and 0.17; the middle of that range.
pub const MESH_DAMPING: f64 = 0.07;

/// Density of the gear steel, kg/m³ (AISI 8620 / 20MnCr5 case-hardening steel, 7850).
const STEEL_DENSITY: f64 = 7850.0;

/// Surface density of the housing wall per metre of thickness, kg/m³: die-cast aluminium (2700).
const HOUSING_DENSITY: f64 = 2700.0;

/// Density of air, kg/m³ (20 °C, 1 atm).
const AIR_DENSITY: f64 = 1.2;

/// Pressure at `r_m` metres from a housing wall of thickness `wall_m` driven by `force_n`.
///
/// Cremer, Heckl & Petersson, *Structure-Borne Sound*, 3rd ed., ch. 5: below the coincidence
/// frequency a plate excited at a point radiates like a baffled monopole whose volume
/// acceleration is `F / m″`, with `m″` the plate's mass per unit area. Hence
/// `p = ρ₀·F / (2π·r·m″)` (the half-space monopole of Pierce, *Acoustics*, eq. 4-4.5).
pub fn housing_pressure_pa(force_n: f64, wall_m: f64, r_m: f64) -> f64 {
    AIR_DENSITY * force_n / (2.0 * std::f64::consts::PI * r_m * HOUSING_DENSITY * wall_m)
}

/// Side of the housing's wall panels between its ribs and flanges, m.
///
/// ESTIMATE: a passenger-car box case is ~0.3–0.4 m across, ribbed into panels of about half that.
const HOUSING_PANEL_M: f64 = 0.2;

/// Young's modulus and Poisson's ratio of die-cast aluminium (A380: 71 GPa, 0.33).
const HOUSING_E: f64 = 70e9;
const HOUSING_NU: f64 = 0.33;

/// First bending mode of a simply supported square housing panel of [`HOUSING_PANEL_M`] and
/// wall `wall_m`, Hz: `f₁₁ = (π/2)·(2/a²)·√(D/m″)`, `D = E·h³/(12(1−ν²))` (Leissa, *Vibration
/// of Plates*, NASA SP-160, 1969, §4.1).
///
/// Below it the wall is stiffness-controlled: its volume velocity under a force falls with ω²
/// against the mass law's (Cremer, Heckl & Petersson, *Structure-Borne Sound*, ch. 5), so the
/// housing radiates [`housing_pressure_pa`] only above it.
pub fn housing_first_mode_hz(wall_m: f64) -> f64 {
    let d = HOUSING_E * wall_m.powi(3) / (12.0 * (1.0 - HOUSING_NU * HOUSING_NU));
    let m2 = HOUSING_DENSITY * wall_m;
    std::f64::consts::FRAC_PI_2 * 2.0 / (HOUSING_PANEL_M * HOUSING_PANEL_M) * (d / m2).sqrt()
}

/// Static deflection of the teeth in contact along the line of action, m: `W / (c′·b·L)` with
/// `length` the contact length in face widths ([`contact_length`]).
pub fn static_error_m(load_n: f64, face_m: f64, length: f64) -> f64 {
    load_n / (SINGLE_PAIR_STIFFNESS * face_m * length)
}

/// Moment of inertia of a solid steel disc as wide as the face, on the pitch radius of `z` teeth.
fn disc_inertia_kgm2(mesh: &Mesh, z: u32) -> f64 {
    let r = 0.5 * mesh.module_m / mesh.helix_rad.cos() * f64::from(z);
    0.5 * STEEL_DENSITY * std::f64::consts::PI * r.powi(4) * mesh.face_m
}

/// One-degree-of-freedom dynamics of a gear pair along its line of action.
///
/// Welbourn, "Fundamental knowledge of gear noise", *Proc. Conf. Noise and Vibration of
/// Engines and Transmissions*, IMechE, 1979, and Kahraman & Singh 1990: the teeth are a spring
/// whose stiffness `k(t) = c′·b·L(t)` follows the contact length as the pairs in contact change
/// ([`contact_length`]), so a constant load `W` still makes the relative displacement `x` swing:
/// `m_e·ẍ + c·ẋ + k(t)·x = W`. The fluctuating force is what the housing sees.
#[derive(Debug, Clone)]
pub struct MeshDynamics {
    /// Mesh phase in base pitches.
    phase: f64,
    x: f64,
    v: f64,
    fresh: bool,
    /// Equivalent mass at the line of action, kg.
    m_eff: f64,
}

impl MeshDynamics {
    pub fn new(mesh: &Mesh) -> Self {
        let (j1, j2) = (
            disc_inertia_kgm2(mesh, mesh.z1),
            disc_inertia_kgm2(mesh, mesh.z2),
        );
        let (r1, r2) = (mesh.base_radius_m(0), mesh.base_radius_m(1));
        Self {
            phase: 0.0,
            x: 0.0,
            v: 0.0,
            fresh: true,
            m_eff: j1 * j2 / (j1 * r2 * r2 + j2 * r1 * r1),
        }
    }

    /// Forget the state (the next step starts from rest on the mean stiffness).
    pub fn reset(&mut self) {
        self.fresh = true;
    }

    /// Advance by `dt` with the driver at `omega1` rad/s transmitting `load_n` along the line of
    /// action; returns the fluctuating mesh force (the load taken off), N.
    ///
    /// A mesh frequency above the Nyquist frequency of `dt` is silent, as it would alias.
    pub fn step(&mut self, mesh: &Mesh, omega1: f64, load_n: f64, dt: f64) -> f64 {
        let rev_per_s = omega1.abs() / (2.0 * std::f64::consts::PI);
        let mesh_hz = f64::from(mesh.z1) * rev_per_s;
        if mesh_hz >= 0.5 / dt {
            self.fresh = true;
            return 0.0;
        }
        let (ea, eb) = (mesh.eps_alpha(), mesh.eps_beta());
        let c_face = SINGLE_PAIR_STIFFNESS * mesh.face_m;
        let k_mean = c_face * ea;
        if self.fresh {
            // start at rest on the mean stiffness, not from a step response
            self.x = load_n / k_mean;
            self.v = 0.0;
            self.fresh = false;
        }
        self.phase = (self.phase + mesh_hz * dt).rem_euclid(1.0);
        let k = c_face * contact_length(self.phase, ea, eb);
        let c = 2.0 * MESH_DAMPING * (k_mean * self.m_eff).sqrt();
        // the force that accelerates the mass this step (−m·a): returned from the state it acts
        // on, not the updated one, whose k·dt·v lag would leak the load's own pulsing (a firing
        // rate's 100 Hz at dt·ω ≈ −44 dB of it) into the whine as a rumble
        let force = k * self.x + c * self.v - load_n;
        self.v -= dt * force / self.m_eff;
        self.x += dt * self.v;
        force
    }
}

/// Face width assumed for the contact of a loose gear in its backlash, m.
///
/// ESTIMATE: the same 18 mm as the default gearbox face ([`GearboxDef`]); it only sets the
/// contact stiffness `c′·b` of the impact.
const LOOSE_FACE_M: f64 = 0.018;

/// Closing speed below which a touch of the flanks is a resting contact, not an impact, m/s.
///
/// ESTIMATE: 1 mm/s is a hundredth of the 0.1 m/s a flank crosses a 0.1 mm backlash at in a
/// rattling gear; slower touches radiate nothing audible.
const IMPACT_MIN_SPEED: f64 = 1e-3;

/// A loose gear free in its backlash, driven by its driver's speed.
///
/// Singh, Xie & Comparin, "Analysis of automotive neutral gear rattle", *J. Sound Vib.* 131(2),
/// 1989: an unloaded gear rides between the flanks of its driver. Oil drag (`T_d`) slows it
/// while the driver pushes it on the drive flank; when the driver decelerates faster than the
/// gear's own drag deceleration (`J·A > T_d`) the gear leaves the flank, crosses the backlash
/// and hits the other one.
#[derive(Debug, Clone)]
pub struct LooseGear {
    inertia: f64,
    base_radius: f64,
    /// Position relative to the driver along the base circle, m (drive flank at `-backlash/2`).
    x: f64,
    /// The gear's own angular speed, rad/s.
    omega: f64,
    fresh: bool,
    in_contact: bool,
    impacting: bool,
}

impl LooseGear {
    pub fn new(inertia_kgm2: f64, base_radius_m: f64) -> Self {
        Self {
            inertia: inertia_kgm2,
            base_radius: base_radius_m,
            x: 0.0,
            omega: 0.0,
            fresh: true,
            in_contact: true,
            impacting: false,
        }
    }

    /// Forget the state (the next step puts the gear on the drive flank).
    pub fn reset(&mut self) {
        self.fresh = true;
        self.in_contact = true;
        self.impacting = false;
    }

    /// Advance by `dt` with the driver at `input_omega`; returns the impact force on the teeth, N
    /// (the largest of the sub-steps; 0 between impacts and while the driver just carries the
    /// gear).
    pub fn step(&mut self, input_omega: f64, dt: f64, backlash_m: f64, drag_nm: f64) -> f64 {
        let rb = self.base_radius;
        let k = SINGLE_PAIR_STIFFNESS * LOOSE_FACE_M;
        let m_eff = self.inertia / (rb * rb);
        let c = 2.0 * MESH_DAMPING * (k * m_eff).sqrt();
        let half = 0.5 * backlash_m;
        // resting penetration of the drive flank while it carries the drag
        let carry = drag_nm / (rb * k);
        if self.fresh {
            self.x = -half - carry;
            self.omega = input_omega;
            self.fresh = false;
        }
        // keep the contact oscillation (ω_n = √(k/m)) under 0.5 rad per sub-step
        let n = ((k / m_eff).sqrt() * dt / 0.5).ceil().max(1.0);
        let h = dt / n;
        let mut peak: f64 = 0.0;
        for _ in 0..n as u32 {
            let v_rel = (self.omega - input_omega) * rb;
            let (force, depth, drive) = if self.x < -half {
                let d = -half - self.x;
                ((k * d - c * v_rel).max(0.0), d, true)
            } else if self.x > half {
                let d = self.x - half;
                (-(k * d + c * v_rel).max(0.0), d, false)
            } else {
                (0.0, 0.0, true)
            };
            let touching = force != 0.0;
            if !touching {
                self.impacting = false;
            } else if !self.in_contact && v_rel.abs() > IMPACT_MIN_SPEED {
                self.impacting = true;
            } else if drive && v_rel.abs() < IMPACT_MIN_SPEED && depth <= 1.5 * carry {
                // settled back into carrying the drag
                self.impacting = false;
            }
            self.in_contact = touching;
            if self.impacting {
                peak = peak.max(force.abs());
            }
            let drag = drag_nm * (self.omega / 0.5).clamp(-1.0, 1.0);
            self.omega += h * (rb * force - drag) / self.inertia;
            self.x += h * (self.omega - input_omega) * rb;
        }
        peak
    }
}

/// A gearbox's description (data): the gear pairs' geometry, which sets the whine's orders and
/// strength, and the loose gears' backlash and drag, which set the rattle. Every field has a
/// default (a common road-car manual box), so files without it load.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GearboxDef {
    /// Helix angle of the gear teeth, degrees (0 = straight-cut spur gears, the sequential / dog
    /// box and reverse gear's loud whine).
    ///
    /// ESTIMATE: road-car helical gears run 20-35 deg (Niemann & Winter II, ch. 21); 25 deg.
    pub helix_deg: f64,
    /// Teeth of a pair together (driver + driven).
    ///
    /// ESTIMATE: 60; every pair of a box spans one centre distance, so with a constant module
    /// they keep the same sum, and a 2.5 mm module at 60 teeth gives the ~80 mm centre distance
    /// of a compact car's box.
    pub teeth_sum: u32,
    /// Normal module, m. ESTIMATE: 2.5 mm (2-3 mm is typical of a passenger-car box).
    pub module_m: f64,
    /// Face width, m. ESTIMATE: 18 mm (15-25 mm in a passenger-car box).
    pub face_m: f64,
    /// Teeth of the final-drive pinion. ESTIMATE: 17 (a pinion under 17 teeth is undercut at 20 deg
    /// pressure angle without profile shift; the driven ring has `round(pinion x final drive)`).
    pub final_pinion_teeth: u32,
    /// Thickness of the housing wall, m. ESTIMATE: 6 mm die-cast aluminium case (4-8 mm).
    pub housing_wall_m: f64,
    /// Circumferential backlash of a gear pair, m. ESTIMATE: 0.1 mm (0.05-0.15 mm in a box).
    pub backlash_m: f64,
    /// Oil drag on a loose gear, N·m. ESTIMATE: 0.05 N·m (churning and bearing drag of one
    /// loose gear in warm gear oil is a few hundredths of a N·m).
    pub drag_nm: f64,
}

impl Default for GearboxDef {
    fn default() -> Self {
        Self {
            helix_deg: 25.0,
            teeth_sum: 60,
            module_m: 2.5e-3,
            face_m: 0.018,
            final_pinion_teeth: 17,
            housing_wall_m: 0.006,
            backlash_m: 1e-4,
            drag_nm: 0.05,
        }
    }
}

impl GearboxDef {
    /// The pair of a gear ratio: tooth counts from [`teeth_for`].
    pub fn mesh_for(&self, ratio: f64) -> Mesh {
        let (z1, z2) = teeth_for(ratio, self.teeth_sum);
        Mesh {
            z1,
            z2,
            module_m: self.module_m,
            helix_rad: self.helix_deg.to_radians(),
            face_m: self.face_m,
        }
    }

    /// The final drive's pair: the pinion and the ring `round(pinion x final drive)`.
    ///
    /// ESTIMATE: same module, helix and face as the box's pairs (a heavier final drive would
    /// be louder per unit of load, not different in order).
    pub fn final_mesh(&self, final_drive: f64) -> Mesh {
        Mesh {
            z1: self.final_pinion_teeth,
            z2: (f64::from(self.final_pinion_teeth) * final_drive)
                .round()
                .max(1.0) as u32,
            module_m: self.module_m,
            helix_rad: self.helix_deg.to_radians(),
            face_m: self.face_m,
        }
    }

    pub(crate) fn is_default(&self) -> bool {
        *self == Self::default()
    }
}

/// What the vehicle hands the engine each step so the gearbox can be heard.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveState {
    /// Engaged gear, 1-based (0 = neutral).
    pub gear: usize,
    /// Instantaneous speed of the input shaft (the crank, through a locked clutch), rad/s.
    pub input_omega: f64,
    /// Torque the input shaft transmits, N·m.
    pub input_torque_nm: f64,
    /// The clutch is locked: the input shaft follows the crank.
    pub clutch_locked: bool,
}

/// The sound of a whole gearbox: the engaged pair's and the final drive's whine, the loose
/// gears' rattle, all through the housing.
#[derive(Debug, Clone)]
pub struct GearboxNoise {
    ratios: Vec<f64>,
    final_drive: f64,
    def: GearboxDef,
    meshes: Vec<Mesh>,
    mesh_dyn: Vec<MeshDynamics>,
    final_mesh: Mesh,
    final_dyn: MeshDynamics,
    loose: Vec<LooseGear>,
    last_gear: usize,
    /// The housing wall's response ([`housing_first_mode_hz`]) and the step it was built for.
    housing: Biquad,
    housing_dt: f64,
}

impl GearboxNoise {
    pub fn new(v: &VehicleDef) -> Self {
        let meshes: Vec<Mesh> = v
            .gear_ratios
            .iter()
            .map(|&r| v.gearbox.mesh_for(r))
            .collect();
        let final_mesh = v.gearbox.final_mesh(v.final_drive);
        Self {
            ratios: v.gear_ratios.clone(),
            final_drive: v.final_drive,
            def: v.gearbox.clone(),
            mesh_dyn: meshes.iter().map(MeshDynamics::new).collect(),
            final_dyn: MeshDynamics::new(&final_mesh),
            loose: meshes
                .iter()
                .map(|m| LooseGear::new(disc_inertia_kgm2(m, m.z2), m.base_radius_m(1)))
                .collect(),
            meshes,
            final_mesh,
            last_gear: 0,
            housing: Biquad::highpass(1.0, 1.0, 2.0),
            housing_dt: 0.0,
        }
    }

    /// Advance by `dt`; returns the sound pressure at 1 m, Pa. A gear-0 or released-clutch box
    /// is silent: nothing carries torque, and the input shaft is not tied to the crank.
    pub fn step(&mut self, v: &VehicleDef, s: &DriveState, dt: f64) -> f64 {
        if v.gear_ratios != self.ratios
            || v.final_drive != self.final_drive
            || v.gearbox != self.def
        {
            *self = Self::new(v);
        }
        if !s.clutch_locked {
            self.silence();
            return 0.0;
        }
        if s.gear != self.last_gear {
            self.silence();
            self.last_gear = s.gear;
        }
        let wall = self.def.housing_wall_m;
        let mut force = 0.0;
        let engaged = s.gear.checked_sub(1).filter(|&i| i < self.ratios.len());
        if let Some(i) = engaged {
            let (mesh, ratio) = (&self.meshes[i], self.ratios[i]);
            let load = s.input_torque_nm.abs() / mesh.base_radius_m(0);
            // the signed force: the mesh's wave is zero-mean (Welbourn 1979); rectifying it would
            // radiate its mean and the load's envelope (firing pulses, shuffle) as a low rumble
            force += self.mesh_dyn[i].step(mesh, s.input_omega, load, dt);
            // the final pinion sits on the output shaft: slower by the ratio, torque higher
            let fm = &self.final_mesh;
            let load = s.input_torque_nm.abs() * ratio / fm.base_radius_m(0);
            force += self.final_dyn.step(fm, s.input_omega / ratio, load, dt);
        }
        // the loose gears of the other pairs, driven through the constant mesh at z1/z2
        if s.input_omega.abs() > 1.0 {
            for (i, g) in self.loose.iter_mut().enumerate() {
                if engaged == Some(i) {
                    continue;
                }
                let m = &self.meshes[i];
                let speed = s.input_omega * f64::from(m.z1) / f64::from(m.z2);
                force += g.step(speed, dt, self.def.backlash_m, self.def.drag_nm);
            }
        }
        if dt != self.housing_dt {
            // a Butterworth's smooth knee for the modes' sum: a ribbed case has many, not one
            // sharp resonance (ESTIMATE)
            let f1 = housing_first_mode_hz(wall).min(0.4 / dt);
            self.housing = Biquad::highpass(f1, std::f64::consts::FRAC_1_SQRT_2, 1.0 / dt);
            self.housing_dt = dt;
        }
        self.housing.process(housing_pressure_pa(force, wall, 1.0))
    }

    fn silence(&mut self) {
        self.mesh_dyn.iter_mut().for_each(MeshDynamics::reset);
        self.final_dyn.reset();
        self.loose.iter_mut().for_each(LooseGear::reset);
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use proptest::prelude::*;

    fn mean_length(eps_alpha: f64, eps_beta: f64, n: usize) -> f64 {
        (0..n)
            .map(|i| contact_length(i as f64 / n as f64, eps_alpha, eps_beta))
            .sum::<f64>()
            / n as f64
    }

    #[test]
    fn a_spur_pairs_contact_ratio_by_hand() {
        // 20/20, m = 1 mm: r_a = 11, r_b = 10·cos 20° = 9.39693; 2·√(121 − 88.30222) = 11.43640,
        // less a·sin α = 20·0.34202 = 6.84040, over π·m·cos α = 2.95213: 1.55684
        let m = Mesh {
            z1: 20,
            z2: 20,
            module_m: 1e-3,
            helix_rad: 0.0,
            face_m: 0.01,
        };
        assert!((m.eps_alpha() - 1.55684).abs() < 1e-4, "{}", m.eps_alpha());
        assert_eq!(m.eps_beta(), 0.0);
        assert!((m.base_radius_m(0) - 9.39693e-3).abs() < 1e-7);
    }

    #[test]
    fn a_helical_pairs_overlap_ratio_by_hand() {
        // b·sin β / (π·m_n) = 18·0.42262 / (π·2.5) = 0.96862
        let m = Mesh {
            z1: 26,
            z2: 34,
            module_m: 2.5e-3,
            helix_rad: 25f64.to_radians(),
            face_m: 0.018,
        };
        assert!((m.eps_beta() - 0.96862).abs() < 1e-4, "{}", m.eps_beta());
        // the transverse contact ratio is still between one and two pairs
        assert!(
            m.eps_alpha() > 1.2 && m.eps_alpha() < 1.8,
            "{}",
            m.eps_alpha()
        );
    }

    #[test]
    fn contact_length_counts_pairs_on_a_spur() {
        let two = (0..1000)
            .filter(|&i| (contact_length(i as f64 / 1000.0, 1.5, 0.0) - 2.0).abs() < 1e-9)
            .count();
        let one = (0..1000)
            .filter(|&i| (contact_length(i as f64 / 1000.0, 1.5, 0.0) - 1.0).abs() < 1e-9)
            .count();
        assert_eq!(one + two, 1000);
        assert!((two as i32 - 500).abs() <= 2, "{two}");
        assert!((mean_length(1.5, 0.0, 1000) - 1.5).abs() < 2e-3);
    }

    #[test]
    fn an_integer_overlap_holds_the_length_constant() {
        for t in [0.1, 0.37, 0.8] {
            assert!((contact_length(t, 1.5, 1.0) - 1.5).abs() < 1e-9, "{t}");
        }
    }

    #[test]
    fn tooth_counts_follow_the_ratio() {
        assert_eq!(teeth_for(1.3, 60), (26, 34));
        assert_eq!(teeth_for(0.75, 60), (34, 26));
        let (a, b) = teeth_for(3.6, 60);
        assert!(a >= 12 && b >= 12, "{a} {b}");
        let (a, b) = teeth_for(0.5, 60);
        assert_eq!((a, b), (40, 20));
    }

    /// Goertzel power of `x` at `f` Hz (sample rate `fs`), normalised by the length.
    pub(crate) fn goertzel(x: &[f64], f: f64, fs: f64) -> f64 {
        let w = 2.0 * std::f64::consts::PI * f / fs;
        let (c, mut s1, mut s2) = (2.0 * w.cos(), 0.0, 0.0);
        for &v in x {
            let s0 = v + c * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        (s1 * s1 + s2 * s2 - c * s1 * s2) / (x.len() as f64).powi(2)
    }

    /// How far under the whole `x`'s content below `hz` lies, dB: the DFT's bins up to `hz`
    /// against the whole by Parseval (Σ|X_k|² over all k = N·Σx²).
    fn under_db(x: &[f64], hz: f64, fs: f64) -> f64 {
        let n = x.len();
        let bin = |k: usize| {
            let w = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
            let (re, im) = x.iter().enumerate().fold((0.0, 0.0), |(re, im), (i, &v)| {
                (re + v * (w * i as f64).cos(), im - v * (w * i as f64).sin())
            });
            re * re + im * im
        };
        let top = (hz * n as f64 / fs) as usize;
        let low = bin(0) + 2.0 * (1..=top).map(bin).sum::<f64>();
        db(n as f64 * x.iter().map(|v| v * v).sum::<f64>(), low)
    }

    fn rms(x: &[f64]) -> f64 {
        (x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64).sqrt()
    }

    fn db(a: f64, b: f64) -> f64 {
        10.0 * (a / b).log10()
    }

    fn road_pair(helix_deg: f64) -> Mesh {
        Mesh {
            z1: 26,
            z2: 34,
            module_m: 2.5e-3,
            helix_rad: helix_deg.to_radians(),
            face_m: 0.018,
        }
    }

    /// Force of `mesh` under `load` for `secs` at 50 rev/s on the driver, sampled at 96 kHz.
    fn mesh_force(mesh: &Mesh, load: f64, secs: f64) -> Vec<f64> {
        let dt = 1.0 / 96_000.0;
        let mut d = MeshDynamics::new(mesh);
        (0..(secs / dt) as usize)
            .map(|_| d.step(mesh, 2.0 * std::f64::consts::PI * 50.0, load, dt))
            .collect()
    }

    #[test]
    fn the_housing_radiates_the_force_as_a_point_source() {
        // m'' = 2700·0.006 = 16.2 kg/m²; p = 1.2·100 / (2π·1·16.2) = 120 / 101.788 = 1.1789 Pa
        let p = housing_pressure_pa(100.0, 0.006, 1.0);
        assert!((p - 1.179).abs() < 1e-3, "{p}");
    }

    #[test]
    fn the_housings_first_mode_by_hand() {
        // D = 70e9·0.006³/(12·(1 − 0.33²)) = 15 120 000/10.6932 = 1414.0 N·m; m″ = 16.2 kg/m²;
        // √(D/m″) = 9.343; (π/2)·(2/0.04)·9.343 = 733.8 Hz. Doubling the wall doubles it
        // (h³ over h, square-rooted: ∝ h)
        assert!((housing_first_mode_hz(0.006) - 733.8).abs() < 1.0);
        assert!((housing_first_mode_hz(0.012) - 1467.6).abs() < 2.0);
    }

    #[test]
    fn the_static_error_under_load_by_hand() {
        // W / (c'·b·L) = 1000 / (14e9 · 0.02 · 1) = 3.5714 µm; with two pairs, half of it
        assert!((static_error_m(1000.0, 0.02, 1.0) - 3.5714e-6).abs() < 1e-9);
        assert!((static_error_m(1000.0, 0.02, 2.0) - 1.7857e-6).abs() < 1e-9);
    }

    #[test]
    fn a_spur_mesh_whines_at_its_mesh_frequency() {
        // 26 teeth · 50 rev/s = 1300 Hz
        let f = mesh_force(&road_pair(0.0), 3000.0, 0.5);
        let p = |hz| goertzel(&f, hz, 96_000.0);
        assert!(
            db(p(1300.0), p(1000.0)) >= 20.0,
            "{}",
            db(p(1300.0), p(1000.0))
        );
        assert!(
            db(p(1300.0), p(1700.0)) >= 20.0,
            "{}",
            db(p(1300.0), p(1700.0))
        );
    }

    #[test]
    fn helical_teeth_whine_far_less_than_spur() {
        let spur = rms(&mesh_force(&road_pair(0.0), 3000.0, 0.3));
        let helical = rms(&mesh_force(&road_pair(25.0), 3000.0, 0.3));
        assert!(spur > 1.0, "{spur}");
        assert!(
            db(spur * spur, helical * helical) >= 10.0,
            "{spur} {helical}"
        );
    }

    #[test]
    fn no_load_no_whine() {
        let f = mesh_force(&road_pair(0.0), 0.0, 0.1);
        assert!(rms(&f) < 1e-9, "{}", rms(&f));
    }

    #[test]
    fn above_nyquist_it_is_silent() {
        // 26 teeth · 400 rev/s = 10.4 kHz, above the 8 kHz Nyquist of a 16 kHz step
        let m = road_pair(0.0);
        let mut d = MeshDynamics::new(&m);
        let dt = 1.0 / 16_000.0;
        for _ in 0..2000 {
            let f = d.step(&m, 2.0 * std::f64::consts::PI * 400.0, 3000.0, dt);
            assert_eq!(f, 0.0);
        }
    }

    const DT96: f64 = 1.0 / 96_000.0;

    /// Impact force per sample of a loose gear (J = 2e-4 kg·m², r_b = 40 mm, 0.1 mm backlash,
    /// 0.05 N·m drag) whose driver turns at `100 + A/(2πf)·sin(2πft)` rad/s, `secs` long.
    fn rattle_run(accel: f64, f_hz: f64, secs: f64) -> Vec<f64> {
        let w = 2.0 * std::f64::consts::PI * f_hz;
        let mut g = LooseGear::new(2e-4, 0.04);
        (0..(secs / DT96) as usize)
            .map(|i| {
                let omega = 100.0 + accel / w * (w * i as f64 * DT96).sin();
                g.step(omega, DT96, 1e-4, 0.05)
            })
            .collect()
    }

    /// Impacts per second after the first 0.2 s: rising edges of the force.
    fn impacts_per_s(f: &[f64]) -> f64 {
        let from = (0.2 / DT96) as usize;
        let n = f[from..]
            .windows(2)
            .filter(|w| w[0] <= 0.0 && w[1] > 0.0)
            .count();
        n as f64 / (f.len() - from) as f64 / DT96
    }

    #[test]
    fn a_steady_input_does_not_rattle() {
        let f = rattle_run(0.0, 50.0, 1.0);
        assert_eq!(
            f[(0.2 / DT96) as usize..]
                .iter()
                .filter(|&&v| v > 0.0)
                .count(),
            0
        );
    }

    #[test]
    fn rattle_starts_at_the_singh_threshold() {
        // Singh: the gear leaves its flank when J·A > T_d, A* = 0.05 / 2e-4 = 250 rad/s²
        let below = impacts_per_s(&rattle_run(0.5 * 250.0, 50.0, 1.2));
        let above = impacts_per_s(&rattle_run(3.0 * 250.0, 50.0, 1.2));
        assert_eq!(below, 0.0, "{below}");
        assert!(above >= 50.0, "{above}");
    }

    fn box_vehicle() -> crate::drivetrain::VehicleDef {
        crate::drivetrain::VehicleDef {
            gear_ratios: vec![3.6, 2.1, 1.3, 1.0, 0.8],
            final_drive: 4.1,
            ..crate::drivetrain::VehicleDef::inline4()
        }
    }

    #[test]
    fn a_vehicle_file_without_a_gearbox_loads_the_default() {
        let v: crate::drivetrain::VehicleDef =
            serde_yaml_ng::from_str("name: x\nmass_kg: 1000\n").unwrap();
        assert_eq!(v.gearbox, GearboxDef::default());
        // and a default box is not written back, so preset files stay as they are
        let yaml = serde_yaml_ng::to_string(&v).unwrap();
        assert!(!yaml.contains("gearbox"), "{yaml}");
    }

    #[test]
    fn the_gearbox_round_trips_json_and_yaml() {
        let g = GearboxDef {
            helix_deg: 0.0,
            teeth_sum: 54,
            module_m: 3e-3,
            face_m: 0.022,
            final_pinion_teeth: 15,
            housing_wall_m: 0.005,
            backlash_m: 1.2e-4,
            drag_nm: 0.08,
        };
        let v = crate::drivetrain::VehicleDef {
            gearbox: g.clone(),
            ..box_vehicle()
        };
        let json = serde_json::to_string(&v).unwrap();
        let yaml = serde_yaml_ng::to_string(&v).unwrap();
        let from_json: crate::drivetrain::VehicleDef = serde_json::from_str(&json).unwrap();
        let from_yaml: crate::drivetrain::VehicleDef = serde_yaml_ng::from_str(&yaml).unwrap();
        assert_eq!(from_json.gearbox, g);
        assert_eq!(from_yaml.gearbox, g);
        assert!(serde_yaml_ng::from_str::<GearboxDef>("bogus: 1").is_err());
    }

    #[test]
    fn third_gear_whines_at_its_and_the_final_drives_orders() {
        // 3rd = ratio 1.3: teeth_for(1.3, 60) = 26/34 -> 26 · 50 rev/s = 1300 Hz at the input;
        // the output turns at 50 / 1.3 = 38.46 rev/s and the 17-tooth final pinion sits on it:
        // 17 · 38.46 = 653.8 Hz
        let v = box_vehicle();
        let mut g = GearboxNoise::new(&v);
        let s = DriveState {
            gear: 3,
            input_omega: 2.0 * std::f64::consts::PI * 50.0,
            input_torque_nm: 150.0,
            clutch_locked: true,
        };
        let dt = 1.0 / 96_000.0;
        let out: Vec<f64> = (0..48_000).map(|_| g.step(&v, &s, dt)).collect();
        let p = |hz| goertzel(&out, hz, 96_000.0);
        for (hz, label) in [(1300.0, "mesh"), (50.0 * 17.0 / 1.3, "final drive")] {
            for off in [-100.0, 100.0] {
                let margin = db(p(hz), p(hz + off));
                assert!(margin >= 20.0, "{label} vs {off}: {margin} dB");
            }
        }
    }

    #[test]
    fn a_pulsing_load_whines_only_around_the_mesh_orders() {
        // Welbourn (1979): the mesh force is k(t)·x − W, a zero-mean wave at the mesh orders; a
        // load pulsing at the firing rate (100 Hz here) only puts sidebands at mesh ± 100 Hz.
        // The lowest line is then the final drive's 653.8 − 100 = 553.8 Hz, so the box radiates
        // nothing below 200 Hz: a rectified force would put its mean and its load envelope there.
        let v = box_vehicle();
        let mut g = GearboxNoise::new(&v);
        let dt = 1.0 / 96_000.0;
        let out: Vec<f64> = (0..96_000)
            .map(|i| {
                let t = i as f64 * dt;
                let s = DriveState {
                    gear: 3,
                    input_omega: 2.0 * std::f64::consts::PI * 50.0,
                    input_torque_nm: 150.0 + 100.0 * (2.0 * std::f64::consts::PI * 100.0 * t).sin(),
                    clutch_locked: true,
                };
                g.step(&v, &s, dt)
            })
            .collect();
        let tail = &out[9_600..];
        let margin = under_db(tail, 200.0, 96_000.0);
        assert!(
            margin >= 30.0,
            "under 200 Hz only {margin} dB below the whole"
        );
    }

    #[test]
    fn a_rattling_box_radiates_no_impulse_train_below_its_housings_first_mode() {
        // the neutral rattle of the test below: impacts at the firing rate, each a one-sided
        // pulse. Below the housing's first bending mode the wall is stiffness-controlled and
        // its volume velocity falls with ω² (Cremer, Heckl & Petersson, ch. 5); 6 mm aluminium
        // over a 0.2 m panel: f₁₁ = (π/2)·(2/a²)·√(D/m″), D = 70e9·0.006³/(12·0.89) = 1416 N·m,
        // m″ = 16.2 kg/m²: 1.5708·50·9.35 = 734 Hz. Under 200 Hz the train's pulses would be
        // (200/734)² ⇒ 22.6 dB down at least; the box radiates nothing there 20 dB up.
        let v = box_vehicle();
        let w = 2.0 * std::f64::consts::PI * 27.06;
        let dt = 1.0 / 96_000.0;
        let mut g = GearboxNoise::new(&v);
        let out: Vec<f64> = (0..96_000)
            .map(|i| {
                let s = DriveState {
                    gear: 0,
                    input_omega: 85.0 + 4.25 * (w * i as f64 * dt).sin(),
                    input_torque_nm: 0.0,
                    clutch_locked: true,
                };
                g.step(&v, &s, dt)
            })
            .collect();
        let margin = under_db(&out[9_600..], 200.0, 96_000.0);
        assert!(
            margin >= 20.0,
            "under 200 Hz only {margin} dB below the whole"
        );
    }

    #[test]
    fn neutral_with_the_clutch_up_rattles_and_with_it_down_stays_quiet() {
        // 85 rad/s idle, the 4-cylinder's 2 x crank frequency (27.06 Hz) and a 5 % speed ripple
        // (ESTIMATE: idle irregularity of a 4-cylinder without dual-mass flywheel is 2-6 %):
        // A = 4.25 · 2π · 27.06 = 722 rad/s². The driven gears' thresholds T_d/J are 19..120.
        let v = box_vehicle();
        let w = 2.0 * std::f64::consts::PI * 27.06;
        let dt = 1.0 / 96_000.0;
        let run = |locked: bool| {
            let mut g = GearboxNoise::new(&v);
            (0..96_000)
                .map(|i| {
                    let s = DriveState {
                        gear: 0,
                        input_omega: 85.0 + 4.25 * (w * i as f64 * dt).sin(),
                        input_torque_nm: 0.0,
                        clutch_locked: locked,
                    };
                    g.step(&v, &s, dt)
                })
                .collect::<Vec<f64>>()
        };
        let up = run(true);
        // impacts: the sound rising through 30 % of its peak, at most one per 10 ms (a hit
        // rings the housing for a few ms; the driver's period is 37 ms)
        let tail = &up[19_200..];
        let top = tail.iter().fold(0.0f64, |m, p| m.max(p.abs()));
        let mut last = usize::MAX / 2;
        let mut edges = 0;
        for (i, w) in tail.windows(2).enumerate() {
            if w[0].abs() < 0.3 * top && w[1].abs() >= 0.3 * top && i.wrapping_sub(last) > 960 {
                edges += 1;
                last = i;
            }
        }
        assert!(edges >= 20, "{edges}");
        assert!(run(false).iter().all(|&p| p == 0.0));
    }

    #[test]
    fn overdrive_ratios_make_valid_pairs() {
        let d = GearboxDef::default();
        for ratio in [0.5, 0.8, 1.0, 1.05, 3.6] {
            let m = d.mesh_for(ratio);
            assert!(m.z1 >= 12 && m.z2 >= 12, "{ratio}: {} {}", m.z1, m.z2);
            assert!(m.eps_alpha() > 1.0, "{ratio}: {}", m.eps_alpha());
        }
    }

    #[test]
    fn a_slipping_clutch_or_gear_0_does_not_whine() {
        let v = box_vehicle();
        let mut g = GearboxNoise::new(&v);
        let dt = 1.0 / 96_000.0;
        for gear in [0, 3] {
            for locked in [false, true] {
                // a smooth input: no ripple to rattle on, so any output would be whine
                let s = DriveState {
                    gear,
                    input_omega: 300.0,
                    input_torque_nm: if locked { 0.0 } else { 150.0 },
                    clutch_locked: locked,
                };
                for _ in 0..4800 {
                    let p = g.step(&v, &s, dt);
                    assert!(
                        p.is_finite() && p == 0.0,
                        "gear {gear} locked {locked}: {p}"
                    );
                }
            }
        }
        // and with torque on a locked clutch in gear 0 there is nothing to mesh under load
        let s = DriveState {
            gear: 0,
            input_omega: 300.0,
            input_torque_nm: 150.0,
            clutch_locked: true,
        };
        assert!((0..4800).all(|_| g.step(&v, &s, dt) == 0.0));
    }

    proptest! {
        #[test]
        fn the_mean_contact_length_is_the_profile_ratio(ea in 1.1f64..2.0, eb in 0.0f64..2.5) {
            prop_assert!((mean_length(ea, eb, 2000) - ea).abs() < 0.01);
        }
    }
}
