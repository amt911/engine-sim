//! A gearbox's own sound: the whine of its meshing teeth at their mesh orders and the rattle of
//! its loose gears in their backlash.
//!
//! Geometry: standard involute teeth (ISO 21771; Niemann & Winter, *Maschinenelemente* II, §21):
//! the transverse contact ratio ε_α counts how many tooth pairs share the load along the profile,
//! the overlap ratio ε_β how far a helical tooth's contact spreads along the face. The total
//! length of the contact lines — what the mesh stiffness follows — is the closed form of Maatar &
//! Velex, "An analytical expression for the time-varying contact length in perfect cylindrical
//! gears", *Mech. Mach. Theory* 31(5), 1996: the lines are parallel across the plane of action.

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
        self.v += dt * (load_n - k * self.x - c * self.v) / self.m_eff;
        self.x += dt * self.v;
        k * self.x + c * self.v - load_n
    }
}

#[cfg(test)]
mod tests {
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

    proptest! {
        #[test]
        fn the_mean_contact_length_is_the_profile_ratio(ea in 1.1f64..2.0, eb in 0.0f64..2.5) {
            prop_assert!((mean_length(ea, eb, 2000) - ea).abs() < 0.01);
        }
    }
}
