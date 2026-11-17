//! Mechanical noise of the engine structure: what the block radiates from the impacts of its
//! moving parts (main journals thrown across their oil clearance when their load reverses,
//! pistons slapping the liner when their side force reverses), as opposed to the combustion's
//! own pressure ([`crate::structure`]).
//!
//! Its level is the measured one. At high speed a petrol engine's noise is mechanical: a
//! 1500 cc engine motored without its head and valvetrain made 88.5 dB(A) at 4000 rev/min, the
//! same as two fired ones (Raff & Perry, "Vehicle noise studies", JSV 28(3), 1973, Fig. 11).
//! Lalor, Grover & Priede ("Engine noise — mechanical origins", SAE 800402, 1980) measure three
//! petrol engines' overall noise against speed, all with the same two slopes (their eqs. 12–13:
//! a clearance-controlled branch at low speed, an inertia-controlled one above a critical speed);
//! the curve here is their 91 mm bore / 77 mm stroke engine of Fig. 7, read off the figure:
//! 75 dB(A) at 1000 rev/min, 90 at 3000, 105 at 6000 (ESTIMATE ±1 dB; the three engines spread
//! ±5 dB with no clean trend in bore — the 98 mm one is the quietest — so no bore scaling). The
//! distance is not printed: ISVR's practice of 1 m from the engine (ESTIMATE).

/// Speed where Fig. 7's two slopes meet, rev/min.
pub const CRITICAL_RPM: f64 = 3000.0;
/// Level at [`CRITICAL_RPM`], dB(A) at 1 m.
pub const CRITICAL_DBA: f64 = 90.0;
/// Slope below [`CRITICAL_RPM`], dB per decade of speed: (90 − 75)/log₁₀3.
pub const LOW_DB_PER_DECADE: f64 = 31.4;
/// Slope above it: (105 − 90)/log₁₀2.
pub const HIGH_DB_PER_DECADE: f64 = 49.8;

/// The measured petrol engine's mechanical (overall, full-load) noise at `rpm`, dB(A) at 1 m;
/// −∞ when the engine is not turning.
pub fn mechanical_dba_at_1m(rpm: f64) -> f64 {
    if rpm <= 0.0 {
        return f64::NEG_INFINITY;
    }
    let decades = (rpm / CRITICAL_RPM).log10();
    let slope = if decades < 0.0 {
        LOW_DB_PER_DECADE
    } else {
        HIGH_DB_PER_DECADE
    };
    CRITICAL_DBA + slope * decades
}

/// How much quieter the engine is at no load than at full load at 1000 rev/min, dB: Raff &
/// Perry (JSV 28(3), 1973, Fig. 1) engines A / B / C, 2.5 / 5 / 4 dB read off the figure, mean.
/// Below the critical speed the throttled combustion's smaller gas forces throw the parts across
/// their clearances less hard (§2.2: the combustion acts "upon the principal mechanism … in the
/// sub-critical regime"); above it the inertia forces rule and load does not matter.
pub const NO_LOAD_DROP_DB_AT_1000: f64 = 3.8;

/// [`mechanical_dba_at_1m`] at a part load: `load` 1 at full load (the curve), 0 at no load
/// (Raff's lower curve: the drop [`NO_LOAD_DROP_DB_AT_1000`] at 1000 rev/min, linear in log
/// speed to none at [`CRITICAL_RPM`]), in between linear in dB; clamped to 0..1.
pub fn mechanical_dba_at_1m_loaded(rpm: f64, load: f64) -> f64 {
    let full = mechanical_dba_at_1m(rpm);
    if rpm <= 0.0 || rpm >= CRITICAL_RPM {
        return full;
    }
    let drop = NO_LOAD_DROP_DB_AT_1000 * (CRITICAL_RPM / rpm).log10() / 3f64.log10();
    full - drop * (1.0 - load.clamp(0.0, 1.0))
}

/// The rms sound pressure of a level `db` re 20 µPa, Pa (0 for −∞).
pub fn rms_pa_from_dba(db: f64) -> f64 {
    20e-6 * 10f64.powf(db / 20.0)
}

/// Reciprocating mass (piston, pin, rings, the rod's small end) per bore³, kg/m³: a production
/// petrol piston set of 0.35–0.45 kg at 86 mm bore (ESTIMATE, from the prototype `e472fb2`'s
/// survey; no preset carries its piston's mass).
pub const RECIP_MASS_PER_BORE3: f64 = 600.0;

/// The reciprocating mass of a cylinder of `bore_m`, kg ([`RECIP_MASS_PER_BORE3`]).
pub fn reciprocating_mass_kg(bore_m: f64) -> f64 {
    RECIP_MASS_PER_BORE3 * bore_m.powi(3)
}

/// The force along the cylinder's axis the piston puts on the rod, N (+ towards the crank):
/// the gas's `p_gauge`·A less the reciprocating inertia m·ω²·r·(cos θ + λ cos 2θ) (the
/// slider-crank's piston acceleration to second order in λ = r/l; Heywood, *Internal
/// Combustion Engine Fundamentals*, §2.2). Its sign changes are when the main and big-end
/// journals cross their oil clearance (Raff & Perry 1973 §2.3; Lalor et al. 1980).
pub fn rod_axial_force(
    g: &crate::geometry::CylinderGeometry,
    theta: f64,
    p_gauge: f64,
    omega: f64,
    m_recip: f64,
) -> f64 {
    let r = g.crank_radius();
    let lambda = r / g.rod_m;
    let accel = omega * omega * r * (theta.cos() + lambda * (2.0 * theta).cos());
    p_gauge * g.piston_area() - m_recip * accel
}

/// The piston's thrust on the liner, N: the axial force times tan φ, φ the rod's angle
/// (sin φ = λ sin θ). It reverses at TDC and BDC and wherever the axial force does, and each
/// reversal throws the piston across its clearance: piston slap (Ungar & Ross, *Vibrations and
/// noise due to piston-slap in reciprocating machinery*, JSV 2(2), 1965; Haddad & Pullen 1974).
pub fn piston_side_force(
    g: &crate::geometry::CylinderGeometry,
    theta: f64,
    p_gauge: f64,
    omega: f64,
    m_recip: f64,
) -> f64 {
    let sin_phi = g.crank_radius() / g.rod_m * theta.sin();
    let tan_phi = sin_phi / (1.0 - sin_phi * sin_phi).max(1e-9).sqrt();
    rod_axial_force(g, theta, p_gauge, omega, m_recip) * tan_phi
}

/// The A-weighting of IEC 61672-1 as a filter at step `dt`: four zeros at 0 Hz, poles at
/// 20.6 Hz (twice), 107.7, 737.9 and 12 194 Hz (twice), each pole a first-order section by
/// the bilinear transform with its frequency pre-warped, normalized to 0 dB at 1 kHz.
#[derive(Debug, Clone)]
pub struct AWeighting {
    /// Sections (b0, b1, a1, x₁, y₁).
    sections: [(f64, f64, f64, f64, f64); 6],
    gain: f64,
}

impl AWeighting {
    const POLES_HZ: [f64; 6] = [20.6, 20.6, 107.7, 737.9, 12_194.0, 12_194.0];

    pub fn new(dt: f64) -> Self {
        let fs = 1.0 / dt;
        let mut sections = [(0.0, 0.0, 0.0, 0.0, 0.0); 6];
        for (k, &f) in Self::POLES_HZ.iter().enumerate() {
            let w = (std::f64::consts::PI * f.min(0.45 * fs) / fs).tan();
            // the four low poles carry a zero at DC each (s/(s + ω)); the two high ones are
            // low-passes (ω/(s + ω))
            let (b0, b1) = if k < 4 {
                (1.0 / (1.0 + w), -1.0 / (1.0 + w))
            } else {
                (w / (1.0 + w), w / (1.0 + w))
            };
            sections[k] = (b0, b1, (w - 1.0) / (1.0 + w), 0.0, 0.0);
        }
        let mut a = Self {
            sections,
            gain: 1.0,
        };
        a.gain = 1.0 / a.magnitude(1000.0, fs);
        a
    }

    fn magnitude(&self, f: f64, fs: f64) -> f64 {
        let z = num_complex::Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI * f / fs);
        let zi = z.inv();
        self.sections
            .iter()
            .map(|&(b0, b1, a1, _, _)| ((b0 + b1 * zi) / (1.0 + a1 * zi)).norm())
            .product()
    }

    pub fn process(&mut self, x: f64) -> f64 {
        let mut v = x;
        for (b0, b1, a1, x1, y1) in self.sections.iter_mut() {
            let y = *b0 * v + *b1 * *x1 - *a1 * *y1;
            *x1 = v;
            *y1 = y;
            v = y;
        }
        self.gain * v
    }
}

/// The A-weighted energy at 1 m of one unit impact ([`impact_pulse`] of unit peak
/// pseudo-pressure) through the block's modes at one
/// of its points and its response ([`crate::structure::StructureResponse`]), Pa²·s, at step
/// `dt`: a train of impacts at a rate R and peak a has power R·a²·this.
pub fn unit_impact_energy(dt: f64) -> f64 {
    // averaged over the points an impact may strike ([`IMPACT_POINTS`])
    let n = (0.1 / dt).ceil() as usize;
    (0..IMPACT_POINTS)
        .map(|at| {
            let mut modes = crate::structure::BlockModes::new(IMPACT_POINTS, dt);
            let mut block = crate::structure::StructureResponse::new(dt);
            let mut a = AWeighting::new(dt);
            (0..n)
                .map(|k| {
                    let p = impact_pulse(1.0, k as f64 * dt);
                    let y = a.process(block.process(modes.process_at(&[(at, p)])));
                    y * y * dt
                })
                .sum::<f64>()
        })
        .sum::<f64>()
        / IMPACT_POINTS as f64
}

/// Decay time of a piston's or a journal's impact on the block, s: Ungar & Ross (JSV 2(2),
/// 1965, eq. 1), the struck mass and the wall moving together at v = V0·e^(−t/τ), τ = M/Z, with
/// M the piston set of an 86 mm bore ([`RECIP_MASS_PER_BORE3`]·B³ = 0.3816 kg) and Z = 8√(B·ρh)
/// a cast-iron wall 5 mm thick (an infinite plate, Cremer & Heckl: 1682.7 N·s/m; ESTIMATE the
/// thickness): 0.2268 ms, its spectrum flat to 700 Hz and falling 6 dB per octave above. It
/// replaced the valve seat's 0.15 ms half-sine, flat to ~4 kHz, which put the bare engine's
/// noise in a 2–3 kHz peak 20 dB over its 500 Hz where petrol car engines measure flat from
/// 200 Hz to 3 kHz (Priede, SAE 800534, Fig. 30).
pub const IMPACT_TAU_S: f64 = 0.381_633_6 / 1_682.677_3;

/// One impact of unit `peak` at `t` s after it began ([`IMPACT_TAU_S`]); held to 12 τ.
pub fn impact_pulse(peak: f64, t: f64) -> f64 {
    if (0.0..=12.0 * IMPACT_TAU_S).contains(&t) {
        peak * (-t / IMPACT_TAU_S).exp()
    } else {
        0.0
    }
}

/// Points of the block an impact may strike, each its own voice of the block's modes
/// ([`crate::structure::BlockModes`]): a journal, a cap, a liner's thrust side. Each impact
/// lands on one drawn at random — the clearances and oil films are never the same twice, and a
/// train of identical impacts is a comb of lines, not the noise engines make (ESTIMATE: enough
/// points that no two consecutive impacts tend to share one).
pub const IMPACT_POINTS: usize = 32;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_impact_strikes_as_ungar_and_ross_say() {
        // Ungar & Ross, JSV 2(2) 1965, eqs. (1)-(2): a mass striking a structure of real input
        // impedance Z moves with it at v = V0·e^(−t/τ), τ = M/Z, a spectrum flat below 1/(2πτ)
        // and falling 6 dB per octave above. Hand values: a cast-iron wall 5 mm thick,
        // B = Eh³/(12(1−ν²)) = 110e9·1.25e-7/(12·0.9324) = 1228.9 N·m, Z = 8√(B·ρh) =
        // 8√(1228.9·36) = 1682.7 N·s/m (an infinite plate, Cremer & Heckl); the piston set of
        // an 86 mm bore at 600 kg/m³·B³: 0.3816 kg; τ = 0.3816/1682.7 = 0.2268 ms. Its
        // spectrum at 3 kHz against 100 Hz: √(1 + (2π·100·τ)²)/√(1 + (2π·3000·τ)²)
        // = 1.0101/4.3905 = 0.2301.
        assert!((IMPACT_TAU_S - 2.268e-4).abs() < 2e-6, "{IMPACT_TAU_S}");
        let dt = 1.0 / 384_000.0;
        let n = (0.01 / dt) as usize;
        let x: Vec<f64> = (0..n).map(|k| impact_pulse(1.0, k as f64 * dt)).collect();
        let mag = |f: f64| {
            let (mut re, mut im) = (0.0, 0.0);
            for (k, v) in x.iter().enumerate() {
                let ph = 2.0 * std::f64::consts::PI * f * k as f64 * dt;
                re += v * ph.cos();
                im += v * ph.sin();
            }
            re.hypot(im)
        };
        let ratio = mag(3000.0) / mag(100.0);
        assert!((ratio - 0.2301).abs() < 0.01, "{ratio}");
        assert_eq!(impact_pulse(1.0, -1e-9), 0.0);
    }

    #[test]
    fn unloaded_it_is_quieter_only_below_the_critical_speed() {
        // Raff & Perry, JSV 28(3), 1973, Fig. 1, full load minus no load read off the figure for
        // engines A / B / C: 2.5 / 5 / 4 dB at 1000 rev/min, 3.5 / 2 / 2 at 2000, ≈ 0 from 3000
        // up ("beyond a certain speed, noise level is sensibly independent of load", §2.2)
        let drop = |rpm| mechanical_dba_at_1m(rpm) - mechanical_dba_at_1m_loaded(rpm, 0.0);
        assert!(
            (2.5..=5.0).contains(&drop(1000.0)),
            "1000: {}",
            drop(1000.0)
        );
        assert!(
            (1.0..=3.5).contains(&drop(2000.0)),
            "2000: {}",
            drop(2000.0)
        );
        assert_eq!(drop(3000.0), 0.0);
        assert_eq!(drop(6000.0), 0.0);
    }

    #[test]
    fn the_load_moves_it_between_the_two_curves() {
        // full load is the curve; no load is Raff's lower one; half way, half the drop in dB
        let full = mechanical_dba_at_1m(1000.0);
        let none = mechanical_dba_at_1m_loaded(1000.0, 0.0);
        assert_eq!(mechanical_dba_at_1m_loaded(1000.0, 1.0), full);
        let half = mechanical_dba_at_1m_loaded(1000.0, 0.5);
        assert!((half - 0.5 * (full + none)).abs() < 1e-9);
        // a boosted engine is no louder than at full load, a negative load no quieter than none
        assert_eq!(mechanical_dba_at_1m_loaded(1000.0, 1.6), full);
        assert_eq!(mechanical_dba_at_1m_loaded(1000.0, -0.2), none);
    }

    #[test]
    fn the_petrol_curve_by_the_figure() {
        // Lalor, Grover & Priede, SAE 800402, Fig. 7, the 91 mm bore / 77 mm stroke petrol
        // engine, overall dB(A), read off the figure (±1 dB): 75 at 1000 rev/min, 90 at 3000,
        // 105 at 6000
        assert!((mechanical_dba_at_1m(1000.0) - 75.0).abs() < 1.0);
        assert!((mechanical_dba_at_1m(3000.0) - 90.0).abs() < 1.0);
        assert!((mechanical_dba_at_1m(6000.0) - 105.0).abs() < 1.0);
    }

    #[test]
    fn no_speed_no_noise() {
        assert_eq!(rms_pa_from_dba(mechanical_dba_at_1m(0.0)), 0.0);
        assert_eq!(rms_pa_from_dba(mechanical_dba_at_1m(-50.0)), 0.0);
    }

    #[test]
    fn decibels_to_pascals_by_hand() {
        // 94 dB re 20 µPa is 1.002 Pa rms (20e-6 · 10^(94/20) = 1.0024)
        assert!((rms_pa_from_dba(94.0) - 1.0024).abs() < 1e-4);
    }

    fn geometry() -> crate::geometry::CylinderGeometry {
        let mut g = crate::engine::EngineDef::inline4().geometry;
        g.stroke_m = 0.086;
        g.rod_m = 0.145;
        g
    }

    #[test]
    fn a_weighting_by_the_standard() {
        // IEC 61672-1 Table 3: A-weighting −19.1 dB at 100 Hz, 0.0 at 1 kHz, −2.5 at 10 kHz
        let fs = 192_000.0;
        for (f, want) in [(100.0, -19.1), (1000.0, 0.0), (10_000.0, -2.5)] {
            let mut a = AWeighting::new(1.0 / fs);
            let n = (fs * 0.5) as usize;
            let (mut e_in, mut e_out) = (0.0, 0.0);
            for k in 0..n {
                let x = (2.0 * std::f64::consts::PI * f * k as f64 / fs).sin();
                let y = a.process(x);
                if k > n / 2 {
                    e_in += x * x;
                    e_out += y * y;
                }
            }
            let db = 10.0 * (e_out / e_in).log10();
            assert!((db - want).abs() < 0.3, "{f} Hz: {db:.2} dB");
        }
    }

    #[test]
    fn the_rod_pulls_at_tdc_when_motored_by_hand() {
        // 0.4 kg reciprocating at 3000 rev/min (ω = 314.16 rad/s), r = 43 mm, l = 145 mm
        // (λ = 0.2966): at TDC the inertia pulls the rod with m·ω²·r·(1 + λ)
        // = 0.4 · 98 696 · 0.043 · 1.2966 = 2201 N, so the axial force is −2201 N
        let w = 3000.0 * std::f64::consts::PI / 30.0;
        let f = rod_axial_force(&geometry(), 0.0, 0.0, w, 0.4);
        assert!((f + 2201.0).abs() < 2.0, "{f}");
        // and 1 MPa on a 86 mm piston (5809 N) beats it: pushing
        let f = rod_axial_force(&geometry(), 0.0, 1.0e6, w, 0.4);
        assert!((f - (5808.8 - 2201.0)).abs() < 3.0, "{f}");
    }

    #[test]
    fn the_side_force_reverses_at_tdc_and_bdc() {
        // motored: the rod's angle changes sign at TDC and BDC, so does the thrust on the liner
        let w = 3000.0 * std::f64::consts::PI / 30.0;
        let g = geometry();
        let side = |deg: f64| piston_side_force(&g, deg.to_radians(), 0.0, w, 0.4);
        for at in [0.0, 180.0] {
            assert!(
                side(at - 2.0) * side(at + 2.0) < 0.0,
                "no reversal at {at}°"
            );
        }
    }

    #[test]
    fn a_piston_weighs_by_its_bore_cubed() {
        // 600 kg/m³ of bore³: an 86 mm bore's piston, pin and small end, 0.3816 kg
        assert!((reciprocating_mass_kg(0.086) - 0.3816).abs() < 1e-3);
    }
}
