//! The catalogue of complete breathing systems, sized to the engine
//! (`docs/superpowers/specs/2026-11-04-parts-catalogue-design.md`).
//!
//! Its two empirical numbers come from the real engines among the presets: each preset's exhaust
//! is its car's, documented where the preset is built. They are the mean gas speed in a pipe at
//! the torque peak, and the constant of the wave-tuning law.

use crate::engine::EngineDef;
use crate::exhaust::C_PRIMARY;

/// Volumetric efficiency at the torque peak. ESTIMATE: road engines fill 85–95 % of their swept
/// volume there (Heywood, *Internal Combustion Engine Fundamentals*, ch. 7); the sizing only
/// needs the flow's scale, and the same value enters the survey and the sizing, so it cancels.
pub const ETA_V: f64 = 0.9;
/// Exhaust over intake temperature. ESTIMATE: ~1000 K exhaust over 300 K air, the gas whose sound
/// speed the model's primaries carry ([`C_PRIMARY`]).
pub const T_RATIO: f64 = 1000.0 / 300.0;

/// The rpm a system is tuned for: the torque peak, a share of the redline (spec: 0.75 for road and
/// sport systems, 0.85 for race ones; Blair tunes the primaries for the torque peak).
pub fn tuned_rpm(redline_rpm: f64, race: bool) -> f64 {
    redline_rpm * if race { 0.85 } else { 0.75 }
}

/// Mean gas speed, m/s, in a pipe of `area_m2` carrying the exhaust of `cylinders` cylinders of
/// `swept_m3` each at `rpm`: `n·V_d·η_v·(N/120)·(T_exh/T_in) / A` (a four-stroke breathes once
/// every two turns).
pub fn mean_speed(cylinders: usize, swept_m3: f64, rpm: f64, area_m2: f64) -> f64 {
    cylinders as f64 * swept_m3 * ETA_V * (rpm / 120.0) * T_RATIO / area_m2.max(1e-12)
}

/// The blowdown's crank span, degrees: from the exhaust valve's opening to the next TDC
/// (`180 + EVO before BDC`), when its pulse must go out and come back as a suction.
pub fn blowdown_deg(def: &EngineDef) -> f64 {
    // the cycle starts at the firing TDC: BDC at 180°, the next TDC at 360°
    360.0 - def.exhaust_valve.open_deg
}

/// The two numbers, and their spread over the surveyed engines (min, max) for the warnings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Survey {
    /// Mean gas speed in a primary at the torque peak, m/s (median).
    pub v_primary: f64,
    pub v_primary_spread: (f64, f64),
    /// Mean gas speed in the tail, all cylinders, m/s (median).
    pub v_tail: f64,
    pub v_tail_spread: (f64, f64),
    /// The wave-tuning law `L = k·c·θ/(12·N)` (`θ` the blowdown span, `c` the primaries' sound
    /// speed, `N` rpm): the half round trip of the blowdown pulse; its form is the wave's, its
    /// constant `k` read off the real engines (median).
    pub tuning_k: f64,
    pub tuning_k_spread: (f64, f64),
}

/// The real engines surveyed: road cars whose exhausts the presets carry as built. Not the single
/// (a race pipe), the diesels, the rotaries; the K20C1's primaries feed its turbine, so they set
/// no tuning (its speeds count).
pub fn surveyed() -> Vec<(EngineDef, bool)> {
    vec![
        (EngineDef::honda_k20a(), true),
        (EngineDef::honda_k20c1(), false),
        (EngineDef::lfa_v10(), true),
        (EngineDef::subaru_ej25(), true),
        (EngineDef::flatplane_v8(), true),
        (EngineDef::crossplane_v8(), true),
    ]
}

/// Read the two numbers off the surveyed engines.
pub fn survey() -> Survey {
    let (mut primary, mut tail, mut tuning) = (Vec::new(), Vec::new(), Vec::new());
    for (d, tuned) in surveyed() {
        let n = tuned_rpm(d.redline_rpm, false);
        let swept = d.geometry.displacement();
        let x = &d.exhaust;
        primary.push(mean_speed(1, swept, n, area(x.primary_diameter_m)));
        // the tails: one per branch unless the branches merge, each with its outlets
        let branches = if x.merge.is_some() {
            1
        } else {
            x.groups(d.cylinders).len().max(1)
        };
        let tails = (branches * x.tail_outlets.max(1) as usize) as f64;
        tail.push(mean_speed(
            d.cylinders,
            swept,
            n,
            tails * area(x.tail_diameter_m),
        ));
        if tuned {
            tuning.push(x.primary_length_m * 12.0 * n / (C_PRIMARY * blowdown_deg(&d)));
        }
    }
    let (v_primary, v_primary_spread) = median_spread(&mut primary);
    let (v_tail, v_tail_spread) = median_spread(&mut tail);
    let (tuning_k, tuning_k_spread) = median_spread(&mut tuning);
    Survey {
        v_primary,
        v_primary_spread,
        v_tail,
        v_tail_spread,
        tuning_k,
        tuning_k_spread,
    }
}

/// A pipe's bore area, m².
pub fn area(diameter_m: f64) -> f64 {
    std::f64::consts::PI * 0.25 * diameter_m * diameter_m
}

/// The median (the mean of the middle two of an even count) and the range.
fn median_spread(v: &mut [f64]) -> (f64, (f64, f64)) {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n == 0 {
        return (0.0, (0.0, 0.0));
    }
    let mid = if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    };
    (mid, (v[0], v[n - 1]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_k20a_s_primary_speed_by_hand() {
        // K20A: 86 × 86 mm (0.4996 L a cylinder), redline 8400 → tuned 6300 rpm, 40 mm primary
        // (1.2566e-3 m²): 0.4996e-3 · 0.9 · 6300/120 · 3.333 / 1.2566e-3 = 62.6 m/s
        let d = EngineDef::honda_k20a();
        let n = tuned_rpm(d.redline_rpm, false);
        assert!((n - 6300.0).abs() < 1e-9);
        let swept = d.geometry.displacement();
        let a = std::f64::consts::PI * 0.25 * d.exhaust.primary_diameter_m.powi(2);
        let v = mean_speed(1, swept, n, a);
        assert!((v - 62.6).abs() < 0.5, "{v}");
        // its blowdown: the exhaust opens at 140.5° (39.5° before BDC) → 219.5°
        assert!((blowdown_deg(&d) - 219.5).abs() < 1e-9);
        // a race system tunes higher: 0.85 × 8400
        assert!((tuned_rpm(8400.0, true) - 7140.0).abs() < 1e-9);
    }

    #[test]
    fn the_survey_holds_its_engines() {
        let s = survey();
        // the tuning constant: the median of K20A 0.358, LFA 0.484, EJ25 0.323, flat-plane
        // V8 0.258, cross-plane V8 0.323 (each L·12·N/(c·θ), worked by hand) = 0.323
        assert!((s.tuning_k - 0.323).abs() < 0.005, "{}", s.tuning_k);
        assert!(s.tuning_k_spread.0 < 0.27 && s.tuning_k_spread.1 > 0.47);
        // primary speeds 57–69 m/s by hand (K20A 62.6, LFA 58.6, EJ25 56.8, flat V8 68.5,
        // cross V8 62.7, K20C1 52.5 at its 7000 rpm redline)
        assert!(s.v_primary > 55.0 && s.v_primary < 66.0, "{}", s.v_primary);
        assert!(s.v_primary_spread.0 <= s.v_primary && s.v_primary <= s.v_primary_spread.1);
        assert!(s.v_tail > 0.0 && s.v_tail_spread.0 <= s.v_tail && s.v_tail <= s.v_tail_spread.1);
    }
}
