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
        (flank(self.z1) + flank(self.z2) - a * at.sin())
            / (std::f64::consts::PI * mt * at.cos())
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
        assert!(m.eps_alpha() > 1.2 && m.eps_alpha() < 1.8, "{}", m.eps_alpha());
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

    proptest! {
        #[test]
        fn the_mean_contact_length_is_the_profile_ratio(ea in 1.1f64..2.0, eb in 0.0f64..2.5) {
            prop_assert!((mean_length(ea, eb, 2000) - ea).abs() < 0.01);
        }
    }
}
