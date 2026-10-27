//! Flow noise of an obstruction in a duct: the random sound the turbulent wake of a junction,
//! an area step or a plate generates in the pipe ("regenerated" or "self" noise).
//!
//! Nelson & Morfey (*Aerodynamic sound production in low speed flow ducts*, JSV 79(2), 1981):
//! the wake's fluctuating drag is a dipole whose rms value in each proportional band is a
//! Strouhal-number function K(St) times the steady drag, so the sound power follows from the
//! obstruction's pressure loss. Oldham & Ukpoho (*A pressure-based technique for predicting
//! regenerated noise levels in ventilation systems*, JSV 140(2), 1990) write it from the
//! pressure-loss coefficient alone, for circular ducts and orifice-like elements, and give the
//! trend line K²(St) this module uses.

use std::f64::consts::PI;

/// Highest constriction Mach number the dipole model holds to: above it Gordon (JASA 45, 1969)
/// found a quadrupole (Nelson & Morfey §4.5). The noise's velocity stops here.
const MAX_MACH: f64 = 0.3;

/// Oldham & Ukpoho's Fig. 9 trend line (dampers and orifice plates): (St, K²(St) in dB re
/// 10⁻¹²), digitized. It agrees with Nelson & Morfey's Fig. 9 after their 2009 erratum (−6 dB).
const K2_DB: [(f64, f64); 7] = [
    (0.1, 62.0),
    (1.0, 61.0),
    (3.0, 59.0),
    (5.0, 53.0),
    (10.0, 44.0),
    (20.0, 33.0),
    (50.0, 22.0),
];

/// An obstruction as the flow sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spoiler {
    /// Open-area ratio σ (Oldham eq. 11).
    pub sigma: f64,
    /// Velocity through the constriction, U/σ (eq. 12), m/s; capped at Mach 0.3.
    pub u_c: f64,
    /// Drag coefficient C_L/(1 − σ) (eq. 9).
    pub c_d: f64,
    /// Characteristic dimension D(1 − σ)/2 of an orifice-like element in a circular duct (§3.2), m.
    pub d: f64,
}

/// The obstruction of pressure-loss coefficient `c_l` = Δp/(½ρU²) met by a flow of speed `u` (m/s)
/// in a circular duct of diameter `duct_d` (m), in gas whose speed of sound is `c`. None where
/// nothing obstructs or nothing flows.
pub fn spoiler(c_l: f64, u: f64, duct_d: f64, c: f64) -> Option<Spoiler> {
    if c_l <= 0.0 || u == 0.0 || !c_l.is_finite() || !u.is_finite() {
        return None;
    }
    let sigma = 1.0 / (c_l.sqrt() + 1.0);
    Some(Spoiler {
        sigma,
        u_c: (u.abs() / sigma).min(MAX_MACH * c),
        c_d: c_l / (1.0 - sigma),
        d: duct_d * (1.0 - sigma) / 2.0,
    })
}

/// K²(St), the ratio (not dB), log-interpolated in St on the trend line, held flat beyond it.
pub fn k2(st: f64) -> f64 {
    let db = match K2_DB.iter().position(|&(s, _)| s >= st) {
        Some(0) => K2_DB[0].1,
        None => K2_DB[K2_DB.len() - 1].1,
        Some(i) => {
            let ((s0, l0), (s1, l1)) = (K2_DB[i - 1], K2_DB[i]);
            l0 + (l1 - l0) * (st / s0).log10() / (s1 / s0).log10()
        }
    };
    10f64.powf(db / 10.0) * 1e-12
}

/// Sound power (W) generated in the third octave centred at `f_c` (Hz), both directions of the
/// duct together, in gas of density `rho` and speed of sound `c`. Below the duct's first cut-on
/// f₀ = 1.84c/(πD) plane waves: W = ρAσ⁴C_D²U_c⁴K²/(16c) (Oldham eq. 16); above it
/// W = ρπA²St²σ⁴C_D²U_c⁶K²/(24c³d²)·[1 + 3c/(8πf_cD)] (eq. 17, with its edge-mode term).
pub fn band_power(sp: &Spoiler, f_c: f64, duct_d: f64, rho: f64, c: f64) -> f64 {
    let area = PI * 0.25 * duct_d * duct_d;
    let st = f_c * sp.d / sp.u_c;
    let drag = sp.sigma.powi(4) * sp.c_d * sp.c_d;
    let f0 = 1.84 * c / (PI * duct_d);
    if f_c < f0 {
        rho * area * drag * sp.u_c.powi(4) * k2(st) / (16.0 * c)
    } else {
        rho * PI * area * area * st * st * drag * sp.u_c.powi(6) * k2(st)
            / (24.0 * c.powi(3) * sp.d * sp.d)
            * (1.0 + 3.0 * c / (8.0 * PI * f_c * duct_d))
    }
}

/// Pressure-loss coefficient of a sudden expansion from `a_small` to `a_large` (m²), on the small
/// pipe's velocity: Borda–Carnot, (1 − A₁/A₂)² (White, *Fluid Mechanics*, §6.9).
pub fn expansion_cl(a_small: f64, a_large: f64) -> f64 {
    (1.0 - a_small / a_large).powi(2)
}

/// Pressure-loss coefficient of a sudden contraction into `a_small`, on its velocity: the vena
/// contracta's C_c = 0.63 + 0.37·(A₁/A₂)³ (Weisbach), C_L = (1/C_c − 1)².
pub fn contraction_cl(a_small: f64, a_large: f64) -> f64 {
    let cc = 0.63 + 0.37 * (a_small / a_large).powi(3);
    (1.0 / cc - 1.0).powi(2)
}

/// The rate the noise is generated at, Hz: it covers the audio band (the formula is checked to
/// 10 kHz) at a fraction of a fine physics step's cost; the physics steps between hold it.
const NOISE_RATE_HZ: f64 = 48_000.0;
/// Octave bands the noise is shaped in, by their centre frequency (Hz).
const OCTAVES: [f64; 8] = [
    125.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0, 16_000.0,
];
/// Noise samples between updates of the band gains (the flow's pulses are 1–2 ms long; this is
/// ≈ 0.17 ms).
const GAIN_UPDATE: u32 = 8;
/// The K² table in third-octave steps of Strouhal number from `ST_MIN`.
const ST_MIN: f64 = 1e-3;
const ST_STEPS: usize = 3 * 21;

/// One obstruction's flow noise, synthesized: white noise through octave bandpass filters
/// whose gains give, band by band, half of [`band_power`] travelling each way along the duct.
/// The gains follow the flow ([`FlowNoise::step`] takes this step's [`Spoiler`]).
#[derive(Debug, Clone)]
pub struct FlowNoise {
    rng: u64,
    /// Physics steps per noise sample, and the count to the next.
    hold: u32,
    count: u32,
    held: f64,
    /// Noise samples until the gains are recomputed.
    until_update: u32,
    /// Per octave: biquad (b0, b2, a1, a2; b1 = 0) of a constant-peak bandpass, its state, the
    /// output's mean square for unit white input, and the gain.
    bands: [Band; OCTAVES.len()],
    /// K²(St) at St = ST_MIN·2^(j/3).
    k2_table: [f64; ST_STEPS],
    /// `leak[i][t]`: the mean square octave `i`'s filter puts into third octave `t` (centred at
    /// 125·2^((t−1)/3) Hz) for unit white input: the bands' skirts overlap, and a steep spectrum
    /// would pile the low bands' skirts onto the high ones.
    leak: [[f64; THIRDS]; OCTAVES.len()],
}

/// Third octaves the gains are fitted on: 99 Hz to 20 kHz (the octaves' own).
const THIRDS: usize = 3 * OCTAVES.len();
/// Multiplicative iterations of the gains' fit.
const FIT_ITERATIONS: usize = 4;

#[derive(Debug, Clone, Copy, Default)]
struct Band {
    b0: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    x: [f64; 2],
    y: [f64; 2],
    energy: f64,
    gain: f64,
}

impl FlowNoise {
    /// A source seeded with `seed`, stepped every `dt` seconds.
    pub fn new(seed: u64, dt: f64) -> Self {
        let hold = ((1.0 / dt) / NOISE_RATE_HZ).round().max(1.0) as u32;
        let fs = 1.0 / (dt * hold as f64);
        let mut bands = [Band::default(); OCTAVES.len()];
        for (b, &f) in bands.iter_mut().zip(&OCTAVES) {
            // RBJ bandpass, constant 0 dB peak, one octave wide (Bristow-Johnson, *Audio EQ
            // Cookbook*)
            let w0 = 2.0 * PI * f.min(0.45 * fs) / fs;
            let alpha = w0.sin() * (2f64.ln() / 2.0 * w0 / w0.sin()).sinh();
            let a0 = 1.0 + alpha;
            *b = Band {
                b0: alpha / a0,
                b2: -alpha / a0,
                a1: -2.0 * w0.cos() / a0,
                a2: (1.0 - alpha) / a0,
                ..Band::default()
            };
            // its output's mean square for unit-variance white input: the impulse response's
            // energy
            let mut probe = *b;
            b.energy = (0..16_384)
                .map(|k| probe.filter(if k == 0 { 1.0 } else { 0.0 }))
                .map(|y| y * y)
                .sum();
        }
        let mut leak = [[0.0; THIRDS]; OCTAVES.len()];
        for (row, b) in leak.iter_mut().zip(&bands) {
            for (t, v) in row.iter_mut().enumerate() {
                // ∫ |H|² over the third, times the one-sided density of unit white, 2/fs
                let fc = OCTAVES[0] * 2f64.powf((t as f64 - 1.0) / 3.0);
                let (lo, hi) = (fc * 2f64.powf(-1.0 / 6.0), fc * 2f64.powf(1.0 / 6.0));
                let n = 64;
                *v = (0..n)
                    .map(|k| {
                        let f = lo + (hi - lo) * (k as f64 + 0.5) / n as f64;
                        // the held samples' zero-order hold: sinc²(f/fs)
                        let x = PI * f / fs;
                        let hold = if hold > 1 { (x.sin() / x).powi(2) } else { 1.0 };
                        b.response(f, fs) * hold * (hi - lo) / n as f64
                    })
                    .sum::<f64>()
                    * 2.0
                    / fs;
            }
        }
        let mut k2_table = [0.0; ST_STEPS];
        for (j, v) in k2_table.iter_mut().enumerate() {
            *v = k2(ST_MIN * 2f64.powf(j as f64 / 3.0));
        }
        Self {
            rng: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            hold,
            count: 0,
            held: 0.0,
            until_update: 0,
            bands,
            k2_table,
            leak,
        }
    }

    /// This physics step's pressure wave (Pa), sent each way along the duct of diameter `duct_d`
    /// whose gas has density `rho` and speed of sound `c`, from the obstruction `sp` (None: no
    /// obstruction or no flow, silence).
    pub fn step(&mut self, sp: Option<&Spoiler>, duct_d: f64, rho: f64, c: f64) -> f64 {
        let Some(sp) = sp else {
            return 0.0;
        };
        if self.count == 0 {
            if self.until_update == 0 {
                self.update_gains(sp, duct_d, rho, c);
                self.until_update = GAIN_UPDATE;
            }
            self.until_update -= 1;
            // each band its own white noise: bands fed one input would add coherently where their
            // skirts overlap, not in power as the fit assumes
            let mut sum = 0.0;
            for i in 0..self.bands.len() {
                let w = self.white();
                let b = &mut self.bands[i];
                sum += b.gain * b.filter(w);
            }
            self.held = sum;
        }
        self.count = (self.count + 1) % self.hold;
        self.held
    }

    /// The octaves' gains: the 21 third-octave powers (eqs. 16–17), half of them each way, as a
    /// pressure mean square p² = (W/2)·ρc/A; each octave's squared gain fitted so that the bands
    /// together, skirts included, put each octave's three thirds' total where it belongs
    /// (multiplicative iterations on the leak matrix).
    fn update_gains(&mut self, sp: &Spoiler, duct_d: f64, rho: f64, c: f64) {
        let area = PI * 0.25 * duct_d * duct_d;
        let f0 = 1.84 * c / (PI * duct_d);
        let plane = rho * area * sp.sigma.powi(4) * sp.c_d * sp.c_d * sp.u_c.powi(4) / (16.0 * c);
        let to_p2 = 0.5 * rho * c / area;
        // the table index of the lowest third octave (125·2^(−1/3) Hz)
        let j0 = 3.0 * (OCTAVES[0] * 2f64.powf(-1.0 / 3.0) * sp.d / (sp.u_c * ST_MIN)).log2();
        let mut target = [0.0; THIRDS];
        for (t, v) in target.iter_mut().enumerate() {
            let f = OCTAVES[0] * 2f64.powf((t as f64 - 1.0) / 3.0);
            // eq. 17 over eq. 16 is (2πA f²/3c²)·(edge term), whatever the flow
            let modes = if f < f0 {
                1.0
            } else {
                2.0 * PI * area * f * f / (3.0 * c * c) * (1.0 + 3.0 * c / (8.0 * PI * f * duct_d))
            };
            *v = plane * lerp_table(&self.k2_table, j0 + t as f64) * modes * to_p2;
        }
        let want: [f64; OCTAVES.len()] =
            std::array::from_fn(|i| target[3 * i..3 * i + 3].iter().sum());
        let mut g2: [f64; OCTAVES.len()] =
            std::array::from_fn(|i| want[i] / self.leak[i][3 * i..3 * i + 3].iter().sum::<f64>());
        for _ in 0..FIT_ITERATIONS {
            let mut got = [0.0; OCTAVES.len()];
            for (row, &g) in self.leak.iter().zip(&g2) {
                for (o, part) in got.iter_mut().zip(row.chunks(3)) {
                    *o += g * part.iter().sum::<f64>();
                }
            }
            for ((g, w), o) in g2.iter_mut().zip(&want).zip(&got) {
                if *o > 0.0 {
                    *g *= w / o;
                }
            }
        }
        for (b, g) in self.bands.iter_mut().zip(&g2) {
            b.gain = g.sqrt();
        }
    }

    /// Unit-variance white noise: a uniform from xorshift64* (its filtered sum is Gaussian).
    fn white(&mut self) -> f64 {
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        let r = self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D);
        ((r >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 12f64.sqrt()
    }
}

impl Band {
    /// |H|² at `f` (Hz), sampled at `fs`.
    fn response(&self, f: f64, fs: f64) -> f64 {
        let w = 2.0 * PI * f / fs;
        let (c1, s1, c2, s2) = (w.cos(), w.sin(), (2.0 * w).cos(), (2.0 * w).sin());
        let (nr, ni) = (self.b0 + self.b2 * c2, -self.b2 * s2);
        let (dr, di) = (
            1.0 + self.a1 * c1 + self.a2 * c2,
            -self.a1 * s1 - self.a2 * s2,
        );
        (nr * nr + ni * ni) / (dr * dr + di * di)
    }

    fn filter(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.b2 * self.x[1] - self.a1 * self.y[0] - self.a2 * self.y[1];
        self.x = [x, self.x[0]];
        self.y = [y, self.y[0]];
        y
    }
}

/// `table` at fractional index `j`, linear, held at its ends.
fn lerp_table(table: &[f64], j: f64) -> f64 {
    if j <= 0.0 {
        return table[0];
    }
    let last = table.len() - 1;
    if j >= last as f64 {
        return table[last];
    }
    let i = j as usize;
    let f = j - i as f64;
    table[i] + (table[i + 1] - table[i]) * f
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn spoiler_from_the_pressure_loss() {
        // C_L = 4 (Oldham eq. 11): σ = 1/(√4 + 1) = 1/3; U_c = U/σ = 30 m/s (eq. 12);
        // C_D = C_L/(1 − σ) = 6 (eq. 9); d = D(1 − σ)/2 = 0.2·(2/3)/2 = 0.0667 m (§3.2)
        let sp = spoiler(4.0, 10.0, 0.2, 343.0).unwrap();
        assert_relative_eq!(sp.sigma, 1.0 / 3.0, max_relative = 1e-12);
        assert_relative_eq!(sp.u_c, 30.0, max_relative = 1e-12);
        assert_relative_eq!(sp.c_d, 6.0, max_relative = 1e-12);
        assert_relative_eq!(sp.d, 0.2 / 3.0, max_relative = 1e-12);
    }

    #[test]
    fn k2_follows_oldhams_trend_line() {
        // Fig. 9's trend line, dB re 1e-12: 61 at St 1, 44 at St 10, held flat beyond its ends;
        // at St √10, log-interpolated between (3, 59) and (5, 53): 58.381 dB
        assert_relative_eq!(k2(1.0), 10f64.powf(6.1) * 1e-12, max_relative = 1e-12);
        assert_relative_eq!(k2(10.0), 10f64.powf(4.4) * 1e-12, max_relative = 1e-12);
        assert_eq!(k2(0.01), k2(0.1));
        assert_eq!(k2(500.0), k2(50.0));
        assert_relative_eq!(
            10.0 * (k2(10f64.sqrt()) / 1e-12).log10(),
            58.381_234,
            max_relative = 1e-6
        );
    }

    /// The test spoiler: σ = 1/3, U_c = 30, C_D = 6 in a 200 mm duct.
    fn test_spoiler() -> Spoiler {
        spoiler(4.0, 10.0, 0.2, 343.0).unwrap()
    }

    #[test]
    fn band_power_below_cut_on_by_hand() {
        // f₀ = 1.84·343/(π·0.2) = 1004.5 Hz. At 250 Hz, St = 250·0.0667/30 = 0.5556, K² =
        // 61.2553 dB; eq. 16: W = ρAσ⁴C_D²U_c⁴K²/(16c) = 1.2·0.031416·(1/81)·36·810000·
        // 1.3354e-6/5488 = 3.30177e-6 W
        let w = band_power(&test_spoiler(), 250.0, 0.2, 1.2, 343.0);
        assert_relative_eq!(w, 3.301_769_6e-6, max_relative = 1e-6);
    }

    #[test]
    fn band_power_above_cut_on_by_hand() {
        // At 2000 Hz, St = 4.444, K² = 54.3834 dB; eq. 17: W = ρπA²St²σ⁴C_D²U_c⁶K²/(24c³d²)
        // ·[1 + 3c/(8πfD)] = 1.67327e-6 W
        let w = band_power(&test_spoiler(), 2000.0, 0.2, 1.2, 343.0);
        assert_relative_eq!(w, 1.673_270_5e-6, max_relative = 1e-6);
    }

    #[test]
    fn velocity_scaling() {
        // at a fixed Strouhal number, doubling the flow raises the power by 2⁴ (12.04 dB) in plane
        // waves and by 2⁶ (18.06 dB) above cut-on, less the edge-mode term's change
        let (d, rho, c) = (0.2, 1.2, 343.0);
        let a = spoiler(4.0, 10.0, d, c).unwrap();
        let b = spoiler(4.0, 20.0, d, c).unwrap();
        let below = band_power(&b, 200.0, d, rho, c) / band_power(&a, 100.0, d, rho, c);
        assert_relative_eq!(10.0 * below.log10(), 12.041, max_relative = 1e-3);
        let edge = |f: f64| 1.0 + 3.0 * c / (8.0 * std::f64::consts::PI * f * d);
        let above = band_power(&b, 4000.0, d, rho, c) / band_power(&a, 2000.0, d, rho, c);
        assert_relative_eq!(
            above,
            64.0 * edge(4000.0) / edge(2000.0),
            max_relative = 1e-9
        );
    }

    #[test]
    fn no_obstruction_no_noise() {
        assert!(spoiler(0.0, 10.0, 0.05, 600.0).is_none());
        assert!(spoiler(-1.0, 10.0, 0.05, 600.0).is_none());
        assert!(spoiler(2.0, 0.0, 0.05, 600.0).is_none());
    }

    #[test]
    fn mach_cap() {
        // the dipole model holds to a constriction Mach number of 0.3 (Nelson & Morfey §4.5,
        // after Gordon 1969): the noise's U_c stops there
        assert_eq!(spoiler(4.0, 500.0, 0.05, 600.0).unwrap().u_c, 0.3 * 600.0);
        assert!(
            band_power(
                &spoiler(4.0, 1e9, 0.05, 600.0).unwrap(),
                500.0,
                0.05,
                0.4,
                600.0
            )
            .is_finite()
        );
    }

    #[test]
    fn area_change_coefficients() {
        // Borda–Carnot: (1 − 1/4)² = 0.5625; Weisbach: C_c = 0.63 + 0.37/64 = 0.635781, so
        // C_L = (1/C_c − 1)² = 0.328178
        assert_eq!(expansion_cl(1.0, 4.0), 0.5625);
        assert_relative_eq!(contraction_cl(1.0, 4.0), 0.328_177_8, max_relative = 1e-6);
        assert_eq!(expansion_cl(1.0, 1.0), 0.0);
    }
    /// Power per third octave (W, one direction) of `x` sampled at `dt`, by averaged
    /// periodograms, at the band centred on `f`.
    fn third_octave_power(x: &[f64], dt: f64, f: f64, area: f64, rho_c: f64) -> f64 {
        // a few dozen bins in every band: long frames for the narrow low bands
        let n = if f < 1000.0 { 32_768 } else { 8192 };
        let (lo, hi) = (f * 2f64.powf(-1.0 / 6.0), f * 2f64.powf(1.0 / 6.0));
        let df = 1.0 / (n as f64 * dt);
        let w: Vec<f64> = (0..n)
            .map(|k| 0.5 - 0.5 * (2.0 * PI * k as f64 / n as f64).cos())
            .collect();
        let wsum: f64 = w.iter().map(|v| v * v).sum();
        let (mut acc, mut frames) = (0.0, 0);
        for start in (0..x.len() - n).step_by(n / 2) {
            // DFT only of the band's bins: a direct sum is enough for a test
            let (k0, k1) = ((lo / df).ceil() as usize, (hi / df).floor() as usize);
            for k in k0..=k1 {
                let (mut re, mut im) = (0.0, 0.0);
                for j in 0..n {
                    let ph = -2.0 * PI * (k * j) as f64 / n as f64;
                    let v = x[start + j] * w[j];
                    re += v * ph.cos();
                    im += v * ph.sin();
                }
                // one-sided: 2|X|²/(Σw²·N)·… → mean square in the bin
                acc += 2.0 * (re * re + im * im) / (wsum * n as f64);
            }
            frames += 1;
        }
        acc / frames as f64 * area / rho_c
    }

    #[test]
    fn held_spectrum_matches_the_formula() {
        // a spoiler held steady (σ = 1/3, U_c = 60 m/s, C_D = 6) in a 56 mm pipe of hot gas: the
        // noise sent each way carries half the predicted power in every third octave from
        // 100 Hz to 10 kHz, within ±2 dB (f₀ = 1.84·600/(π·0.056) = 6.27 kHz: both laws)
        let dt = 1.0 / 192_000.0;
        let (duct_d, rho, c) = (0.056, 0.4, 600.0);
        let area = PI * 0.25 * duct_d * duct_d;
        let sp = spoiler(4.0, 20.0, duct_d, c).unwrap();
        let mut noise = FlowNoise::new(11, dt);
        let x: Vec<f64> = (0..(4.0 / dt) as usize)
            .map(|_| noise.step(Some(&sp), duct_d, rho, c))
            .collect();
        for k in -10..=10 {
            let f = 1000.0 * 2f64.powf(k as f64 / 3.0);
            if !(100.0..=10_500.0).contains(&f) {
                continue;
            }
            let got = third_octave_power(&x, dt, f, area, rho * c);
            let want = band_power(&sp, f, duct_d, rho, c) / 2.0;
            let db = 10.0 * (got / want).log10();
            assert!(db.abs() < 2.0, "{f:.0} Hz: {db:+.2} dB");
        }
    }

    #[test]
    fn same_seed_same_noise() {
        let dt = 1.0 / 192_000.0;
        let sp = spoiler(4.0, 20.0, 0.056, 600.0).unwrap();
        let run = |seed| {
            let mut n = FlowNoise::new(seed, dt);
            (0..1000)
                .map(|_| n.step(Some(&sp), 0.056, 0.4, 600.0))
                .collect::<Vec<_>>()
        };
        assert_eq!(run(7), run(7));
        assert_ne!(run(7), run(8));
    }

    #[test]
    fn no_spoiler_is_silence() {
        let mut n = FlowNoise::new(3, 1.0 / 192_000.0);
        assert!((0..1000).all(|_| n.step(None, 0.056, 0.4, 600.0) == 0.0));
    }

    #[test]
    fn it_does_not_repeat() {
        // random, not periodic: its autocorrelation at an engine's cycle (1/50 s) is ~0
        let dt = 1.0 / 192_000.0;
        let sp = spoiler(4.0, 20.0, 0.056, 600.0).unwrap();
        let mut n = FlowNoise::new(5, dt);
        let x: Vec<f64> = (0..(2.0 / dt) as usize)
            .map(|_| n.step(Some(&sp), 0.056, 0.4, 600.0))
            .collect();
        let lag = (0.02 / dt) as usize;
        let r: f64 = x.iter().zip(&x[lag..]).map(|(a, b)| a * b).sum::<f64>()
            / x.iter().map(|a| a * a).sum::<f64>();
        assert!(r.abs() < 0.05, "{r}");
    }
}
