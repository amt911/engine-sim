//! The tyres on the road: rolling noise of a moving car (aerodynamic noise included), as the
//! CNOSSOS-EU road source model gives it for a light vehicle (category 1).
//!
//! Commission Directive (EU) 2015/996, Annex, §2.2.2, eq. (2.2.4): the rolling noise sound power
//! in octave band i is L_WR,i = A_R,i + B_R,i·lg(v/v_ref), v_ref = 70 km/h, with A_R and B_R from
//! its Appendix F, Table F-1 (category 1), on the reference road surface, no corrections. The
//! source sits 0.05 m above the road (§2.2.1) and radiates over a rigid ground into a half space:
//! p² = W·ρc/(2πr²). Below 20 km/h CNOSSOS holds a traffic flow at its 20 km/h level; a single
//! car is followed down to rest by the same law, so a car standing still makes no rolling noise.
//!
//! Synthesis: one seeded white noise per octave band through a 4th-order Butterworth high-pass
//! at the band's lower edge and one at its upper edge (f0/√2, f0·√2; each two Bristow-Johnson
//! biquads, Q 0.5412 and 1.3066): neighbouring bands cross at −3 dB each, so their powers sum
//! flat. Each band is scaled to carry its power; the filter's noise gain is its impulse
//! response's energy, computed once.

/// Octave band centres, Hz (Table F-1).
pub const BANDS_HZ: [f64; 8] = [63.0, 125.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0];
/// A_R,i for category 1 (light motor vehicles), dB re 1 pW (Directive 2015/996, Table F-1).
pub const A_R: [f64; 8] = [79.7, 85.7, 84.5, 90.2, 97.3, 93.9, 84.1, 74.3];
/// B_R,i for category 1 (Directive 2015/996, Table F-1).
pub const B_R: [f64; 8] = [30.0, 41.5, 38.9, 25.7, 32.5, 37.2, 39.0, 40.0];
/// Reference speed of Table F-1, km/h.
const V_REF_KMH: f64 = 70.0;
/// Height of the rolling noise source above the road, m (CNOSSOS-EU §2.2.1).
pub const SOURCE_HEIGHT_M: f64 = 0.05;
/// Air at 20 °C: density, kg/m³, and speed of sound, m/s.
const RHO: f64 = 1.204;
const C: f64 = 343.0;
/// Quality factors of a 4th-order Butterworth filter's two sections: 1/(2 cos(π/8)),
/// 1/(2 cos(3π/8)).
const BUTTERWORTH_Q: [f64; 2] = [0.541_196, 1.306_563];

/// Sound power level of band `i` at `v_kmh`, dB re 1 pW (eq. 2.2.4); None at rest.
pub fn sound_power_db(i: usize, v_kmh: f64) -> Option<f64> {
    (v_kmh > 0.0).then(|| A_R[i] + B_R[i] * (v_kmh / V_REF_KMH).log10())
}

/// Rear overhang (tail pipes to the rear axle) and wheelbase of a road car, m. ESTIMATES: a
/// C-segment hatchback/saloon (0.8–1.0 m overhang, 2.6–2.7 m wheelbase).
const REAR_OVERHANG_M: f64 = 0.9;
const WHEELBASE_M: f64 = 2.6;

/// The distance at which one source of the car's whole rolling power is heard as its two axles
/// (half each, incoherent) are, for a listener `distance_m` from the tail pipes' centre,
/// `angle_deg` off the car's axis behind it, the ear `ear_height_m` above the road:
/// 1/r² = ½ (1/r_rear² + 1/r_front²).
pub fn tyre_distance_m(distance_m: f64, angle_deg: f64, ear_height_m: f64) -> f64 {
    let (across, behind) = (
        distance_m * angle_deg.to_radians().sin(),
        distance_m * angle_deg.to_radians().cos(),
    );
    let rise = ear_height_m - SOURCE_HEIGHT_M;
    let inv = |ahead: f64| 1.0 / (across * across + (behind + ahead).powi(2) + rise * rise);
    1.0 / (0.5 * (inv(REAR_OVERHANG_M) + inv(REAR_OVERHANG_M + WHEELBASE_M))).sqrt()
}

/// Rms sound pressure of a source of power `w` (W) on a rigid ground heard at `r_m`, Pa.
pub fn half_space_pa(w: f64, r_m: f64) -> f64 {
    (w * RHO * C / (2.0 * std::f64::consts::PI * r_m * r_m)).sqrt()
}

#[derive(Debug, Clone, Copy, Default)]
struct Biquad {
    b: [f64; 3],
    a: [f64; 2],
    z: [f64; 2],
}

impl Biquad {
    /// Bristow-Johnson's low-pass (`high` false) or high-pass at `fc`, quality `q`.
    fn pass(fc: f64, q: f64, fs: f64, high: bool) -> Self {
        let w0 = 2.0 * std::f64::consts::PI * fc / fs;
        let (cw, alpha) = (w0.cos(), w0.sin() / (2.0 * q));
        let a0 = 1.0 + alpha;
        let (b0, b1) = if high {
            ((1.0 + cw) / 2.0, -(1.0 + cw))
        } else {
            ((1.0 - cw) / 2.0, 1.0 - cw)
        };
        Self {
            b: [b0 / a0, b1 / a0, b0 / a0],
            a: [-2.0 * cw / a0, (1.0 - alpha) / a0],
            z: [0.0; 2],
        }
    }

    fn process(&mut self, x: f64) -> f64 {
        let y = self.b[0] * x + self.z[0];
        self.z[0] = self.b[1] * x - self.a[0] * y + self.z[1];
        self.z[1] = self.b[2] * x - self.a[1] * y;
        y
    }
}

/// The rolling noise heard by a listener, one sample at a time.
#[derive(Debug, Clone)]
pub struct RollingNoise {
    /// Per band: its filter, the gain that makes its output unit variance, its noise state.
    bands: Vec<([Biquad; 4], f64, u64)>,
    /// `leak[i][j]`: share of band j's output power that falls inside octave i.
    leak: Vec<Vec<f64>>,
    /// The speed the band powers were last solved for, km/h, and each band's power, W.
    solved: Option<(f64, Vec<f64>)>,
}

impl Biquad {
    /// |H(e^{jω})|² at `w` rad/sample.
    fn power(&self, w: f64) -> f64 {
        let (c1, s1, c2, s2) = (w.cos(), w.sin(), (2.0 * w).cos(), (2.0 * w).sin());
        let (br, bi) = (
            self.b[0] + self.b[1] * c1 + self.b[2] * c2,
            -(self.b[1] * s1 + self.b[2] * s2),
        );
        let (ar, ai) = (
            1.0 + self.a[0] * c1 + self.a[1] * c2,
            -(self.a[0] * s1 + self.a[1] * s2),
        );
        (br * br + bi * bi) / (ar * ar + ai * ai)
    }
}

/// Solve `m·x = y` (small, dense) by Gaussian elimination with partial pivoting.
fn solve(mut m: Vec<Vec<f64>>, mut y: Vec<f64>) -> Vec<f64> {
    let n = y.len();
    for k in 0..n {
        let p = (k..n)
            .max_by(|&a, &b| m[a][k].abs().total_cmp(&m[b][k].abs()))
            .unwrap_or(k);
        m.swap(k, p);
        y.swap(k, p);
        for r in k + 1..n {
            let f = m[r][k] / m[k][k];
            for c in k..n {
                m[r][c] -= f * m[k][c];
            }
            y[r] -= f * y[k];
        }
    }
    let mut x = vec![0.0; n];
    for k in (0..n).rev() {
        let s: f64 = (k + 1..n).map(|c| m[k][c] * x[c]).sum();
        x[k] = (y[k] - s) / m[k][k];
    }
    x
}

impl RollingNoise {
    /// Bands that fit under the Nyquist frequency of `sample_rate`, each with its own seed.
    pub fn new(sample_rate: f64, seed: u64) -> Self {
        let edges = |f: f64| (f / std::f64::consts::SQRT_2, f * std::f64::consts::SQRT_2);
        let bands: Vec<([Biquad; 4], f64, u64)> = BANDS_HZ
            .iter()
            .enumerate()
            .filter(|&(_, &f)| edges(f).1 < 0.5 * sample_rate)
            .map(|(i, &f)| {
                let (lo, hi) = edges(f);
                let fresh = [
                    Biquad::pass(lo, BUTTERWORTH_Q[0], sample_rate, true),
                    Biquad::pass(lo, BUTTERWORTH_Q[1], sample_rate, true),
                    Biquad::pass(hi, BUTTERWORTH_Q[0], sample_rate, false),
                    Biquad::pass(hi, BUTTERWORTH_Q[1], sample_rate, false),
                ];
                let rng = seed
                    .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                    .wrapping_add((i as u64 + 1).wrapping_mul(0xBF58_476D_1CE4_E5B9))
                    | 1;
                (fresh, 1.0, rng)
            })
            .collect();
        // each band's power density on a 1 Hz grid: its noise gain (white noise of unit
        // variance has density 2/fs per Hz, one-sided) and the share inside every octave
        let n = bands.len();
        let mut leak = vec![vec![0.0; n]; n];
        let mut out = Vec::with_capacity(n);
        for (j, (f, _, rng)) in bands.iter().enumerate() {
            let mut total = 0.0;
            let mut inside = vec![0.0; n];
            let mut hz = 0.5;
            while hz < 0.5 * sample_rate {
                let w = 2.0 * std::f64::consts::PI * hz / sample_rate;
                let d: f64 = f.iter().map(|b| b.power(w)).product();
                total += d;
                for (i, s) in inside.iter_mut().enumerate() {
                    let (lo, hi) = edges(BANDS_HZ[i]);
                    if (lo..hi).contains(&hz) {
                        *s += d;
                    }
                }
                hz += 1.0;
            }
            for i in 0..n {
                leak[i][j] = inside[i] / total;
            }
            let gain = (2.0 * total / sample_rate).sqrt();
            out.push((*f, 1.0 / gain, *rng));
        }
        Self {
            bands: out,
            leak,
            solved: None,
        }
    }

    /// Each band's output power, W, so that every octave carries its Table F-1 power at
    /// `v_kmh` once the bands' skirts are counted.
    fn solve_for(&mut self, v_kmh: f64) {
        if self
            .solved
            .as_ref()
            .is_none_or(|(v, _)| (v - v_kmh).abs() > 0.05)
        {
            let target: Vec<f64> = (0..self.bands.len())
                .map(|i| sound_power_db(i, v_kmh).map_or(0.0, |db| 1e-12 * 10f64.powf(db / 10.0)))
                .collect();
            let w = solve(self.leak.clone(), target)
                .into_iter()
                .map(|x| x.max(0.0))
                .collect();
            self.solved = Some((v_kmh, w));
        }
    }

    /// The next sample, Pa, for a car at `v_kmh` heard at `r_m` from its tyres.
    pub fn sample(&mut self, v_kmh: f64, r_m: f64) -> f64 {
        if v_kmh <= 0.0 {
            return 0.0;
        }
        self.solve_for(v_kmh);
        let Some((_, w)) = &self.solved else {
            return 0.0;
        };
        let mut p = 0.0;
        for ((f, norm, rng), &wj) in self.bands.iter_mut().zip(w) {
            let x = crate::jet_noise::white_noise(rng);
            let y = f.iter_mut().fold(x, |v, b| b.process(v)) * *norm;
            p += y * half_space_pa(wj, r_m);
        }
        p
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_power_follows_table_f1() {
        // at v_ref the power is A_R itself; at 140 km/h each band gains B_R·lg 2:
        // 1 kHz 97.3 + 32.5 × 0.30103 = 107.08 dB; 63 Hz 79.7 + 30 × 0.30103 = 88.73 dB
        assert!((sound_power_db(4, 70.0).unwrap() - 97.3).abs() < 1e-9);
        assert!((sound_power_db(4, 140.0).unwrap() - 107.08).abs() < 0.01);
        assert!((sound_power_db(0, 140.0).unwrap() - 88.73).abs() < 0.01);
        assert!(sound_power_db(4, 0.0).is_none());
    }

    #[test]
    fn a_half_space_source_reads_its_power_less_the_spreading() {
        // 1 W at 1 m over rigid ground: p² = ρc/2π = 1.204 × 343 / 6.2832 = 65.73 Pa²,
        // p = 8.107 Pa
        assert!((half_space_pa(1.0, 1.0) - 8.107).abs() < 0.01);
    }

    /// Octave band level of `x`, dB re 20 µPa: a direct DFT over the band's bins.
    fn band_db(x: &[f64], fs: f64, f0: f64) -> f64 {
        let n = x.len();
        let (lo, hi) = (f0 / std::f64::consts::SQRT_2, f0 * std::f64::consts::SQRT_2);
        let (k0, k1) = (
            (lo * n as f64 / fs).ceil() as usize,
            (hi * n as f64 / fs) as usize,
        );
        let mut e = 0.0;
        for k in k0..=k1 {
            let w = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
            let (mut re, mut im) = (0.0, 0.0);
            for (t, &v) in x.iter().enumerate() {
                re += v * (w * t as f64).cos();
                im -= v * (w * t as f64).sin();
            }
            e += 2.0 * (re * re + im * im) / (n as f64 * n as f64);
        }
        10.0 * (e / 4e-10).log10()
    }

    #[test]
    fn the_synthesis_reads_the_bands_at_the_listener() {
        // 70 km/h heard at 7.5 m: L_p = A_R − 10 lg(2π · 7.5²) + 10 lg(ρc/400)
        // = A_R − 25.48 + 0.14 dB (ρc = 413, re 400 for L_W→L_p): 1 kHz 71.96, 250 Hz 59.16,
        // 4 kHz 58.76 dB re 20 µPa. Within ±1.5 dB (finite record, filter skirts).
        let fs = 16_000.0;
        let mut r = RollingNoise::new(fs, 7);
        let x: Vec<f64> = (0..fs as usize * 3 / 2)
            .map(|_| r.sample(70.0, 7.5))
            .skip(fs as usize / 2)
            .collect();
        for (f0, want) in [(4000.0, 58.76), (1000.0, 71.96), (250.0, 59.16)] {
            let got = band_db(&x, fs, f0);
            assert!((got - want).abs() < 1.5, "{f0} Hz: {got:.2} vs {want}");
        }
    }

    #[test]
    fn the_axles_are_heard_from_where_they_roll() {
        // 3 m straight behind, ear at 1.2 m: rear axle √(3.9² + 1.15²) = 4.066 m, front axle
        // √(6.5² + 1.15²) = 6.601 m; 1/r² = ½(0.060487 + 0.022950) → r = 4.896 m
        assert!((tyre_distance_m(3.0, 0.0, 1.2) - 4.896).abs() < 0.005);
    }

    #[test]
    fn a_car_at_rest_makes_no_rolling_noise() {
        let mut r = RollingNoise::new(48_000.0, 1);
        assert!((0..4800).all(|_| r.sample(0.0, 3.0) == 0.0));
    }
}
