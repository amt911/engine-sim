//! Turbulent mixing noise of the exhaust jet at a tail pipe: the broadband "rasp" on top of the
//! engine orders. A 1D model carries the pulses, not the turbulence; measured tailpipe noise at
//! high orders and speeds is dominated by this flow noise (see `docs/FINDINGS.md`).
//!
//! Level: the jet-mixing-noise correlation of SAE ARP876 (Stone's method, as reproduced e.g. in
//! arXiv:0804.4135 eq. 4), for a single round jet at sea level, 90° to its axis:
//! OASPL = 141 + 10·log((ρⱼ/ρₐ)^w) + 10·log((V/cₐ)^7.5 / (1 + 0.01·(V/cₐ)^4.5))
//!       + 10·log(Aⱼ) − 20·log(R)   [dB re 20 µPa; Aⱼ in m², R in m]
//! Its (V/c)^7.5 is Lighthill's eighth-power law (acoustic power K·ρ·D²·V⁸/c⁵); the 141 dB
//! corresponds to K ≈ 2.5e-5, against Lighthill's ≈ 3e-5. The density exponent
//! w = 3·M^3.5/(0.6 + M^3.5) − 1 (Hoch et al., 1973) makes a slow hot jet louder than a cold one
//! (w → −1). Exhaust jets (M ≈ 0.1–0.5) sit at the low end of the correlation's data; expect
//! ±3 dB.
//!
//! Spectrum: broadband around a Strouhal number f·D/V ≈ 0.2 (the peak of subsonic jet noise at
//! 90°), here a second-order band-pass (±6 dB/octave skirts), a stand-in for the measured
//! one-third-octave shape until the ARP876 table is in hand (PAPERS.md).

use std::f64::consts::PI;

/// Speed of sound and density of the ambient air, m/s and kg/m³ (20 °C, 1 atm).
const C_AMBIENT: f64 = 343.0;
const RHO_AMBIENT: f64 = 1.204;
/// Reference sound pressure, Pa.
const P_REF: f64 = 20e-6;
/// Strouhal number of the spectrum's peak, f·D/V.
const PEAK_STROUHAL: f64 = 0.2;
/// Quality factor of the band-pass: about two octaves between its −3 dB points.
const BAND_Q: f64 = 0.7;

/// Overall sound pressure level of a round jet of exit velocity `v` (m/s, outflow > 0), exit
/// diameter `d` (m) and density `rho_jet` (kg/m³), heard at `r` metres (dB re 20 µPa).
/// No outflow, no jet: −∞.
pub fn jet_oaspl_db(v: f64, d: f64, rho_jet: f64, r: f64) -> f64 {
    if v <= 0.0 || d <= 0.0 || r <= 0.0 {
        return f64::NEG_INFINITY;
    }
    let m = v / C_AMBIENT;
    let w = 3.0 * m.powf(3.5) / (0.6 + m.powf(3.5)) - 1.0;
    let area = PI * 0.25 * d * d;
    141.0 + 10.0 * w * (rho_jet / RHO_AMBIENT).log10() + 75.0 * m.log10()
        - 10.0 * (1.0 + 0.01 * m.powf(4.5)).log10()
        + 10.0 * area.log10()
        - 20.0 * r.log10()
}

/// The jet noise of one tail pipe as a pressure signal at the listener: seeded white noise
/// through a band-pass that follows the exit velocity, scaled to the correlation's level.
#[derive(Debug, Clone)]
pub struct JetNoise {
    rng: u64,
    // band-pass state (transposed direct form II)
    s1: f64,
    s2: f64,
}

impl JetNoise {
    pub fn new(seed: u64) -> Self {
        Self {
            rng: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            s1: 0.0,
            s2: 0.0,
        }
    }

    /// Uniform white noise of unit variance (xorshift64*).
    fn white(&mut self) -> f64 {
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        let u = (self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64;
        (u - 0.5) * 12f64.sqrt()
    }

    /// One step of `dt`: the jet's sound pressure at the listener, Pa.
    pub fn step(&mut self, v: f64, d: f64, rho_jet: f64, r: f64, dt: f64) -> f64 {
        let n = self.white();
        let level = jet_oaspl_db(v, d, rho_jet, r);
        if !level.is_finite() {
            // keep the filter running down quietly
            return self.band(n * 0.0, 1_000.0, dt);
        }
        let p_rms = P_REF * 10f64.powf(level / 20.0);
        let fs = 1.0 / dt;
        let fc = (PEAK_STROUHAL * v / d).clamp(20.0, 0.2 * fs);
        // white noise through the peak-gain-1 band-pass keeps α/(1 + α) of its variance (the
        // sum of its squared impulse response, exact; π·fc/(Q·fs) only while fc ≪ fs)
        let alpha = band_alpha(fc, dt);
        let norm = ((1.0 + alpha) / alpha).sqrt();
        p_rms * norm * self.band(n, fc, dt)
    }

    /// RBJ band-pass (0 dB peak) at `fc`, Q = BAND_Q.
    fn band(&mut self, x: f64, fc: f64, dt: f64) -> f64 {
        let w0 = 2.0 * PI * fc * dt;
        let alpha = band_alpha(fc, dt);
        let a0 = 1.0 + alpha;
        let (b0, b2) = (alpha / a0, -alpha / a0);
        let (a1, a2) = (-2.0 * w0.cos() / a0, (1.0 - alpha) / a0);
        let y = b0 * x + self.s1;
        self.s1 = -a1 * y + self.s2;
        self.s2 = b2 * x - a2 * y;
        y
    }
}

/// The RBJ band-pass's α = sin(ω₀)/(2Q) (Bristow-Johnson, "Audio EQ Cookbook").
fn band_alpha(fc: f64, dt: f64) -> f64 {
    (2.0 * PI * fc * dt).sin() / (2.0 * BAND_Q)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn a_cold_jet_has_the_correlations_level() {
        // V = 100 m/s (M = 0.29155), D = 50 mm (A = 1.9635e-3 m²), ambient density, R = 7.5 m:
        // 141 + 75·log(0.29155) − 10·log(1 + 0.01·0.29155^4.5) + 10·log(1.9635e-3) − 20·log(7.5)
        // = 141 − 40.150 − 0.0002 − 27.069 − 17.501 = 56.28 dB
        assert_relative_eq!(
            jet_oaspl_db(100.0, 0.05, 1.204, 7.5),
            56.282,
            epsilon = 0.01
        );
    }

    #[test]
    fn the_level_follows_lighthills_eighth_power() {
        // twice the speed: 75·log(2) = 22.58 dB (the 0.01·M^4.5 term changes it by < 0.01 dB)
        let up = jet_oaspl_db(160.0, 0.05, 1.204, 7.5) - jet_oaspl_db(80.0, 0.05, 1.204, 7.5);
        assert_relative_eq!(up, 22.58, epsilon = 0.02);
        // and twice the distance, −6.02 dB
        let far = jet_oaspl_db(100.0, 0.05, 1.204, 15.0) - jet_oaspl_db(100.0, 0.05, 1.204, 7.5);
        assert_relative_eq!(far, -6.021, epsilon = 0.001);
    }

    #[test]
    fn a_slow_hot_jet_is_louder_than_a_cold_one() {
        // M = 0.29155: w = 3·M^3.5/(0.6 + M^3.5) − 1 = −0.93456; exhaust gas at 0.567 kg/m³
        // against 1.204: 10·w·log(0.567/1.204) = +3.056 dB
        let hot = jet_oaspl_db(100.0, 0.05, 0.567, 7.5) - jet_oaspl_db(100.0, 0.05, 1.204, 7.5);
        assert_relative_eq!(hot, 3.056, epsilon = 0.005);
    }

    #[test]
    fn no_outflow_makes_no_jet() {
        assert_eq!(jet_oaspl_db(0.0, 0.05, 1.2, 7.5), f64::NEG_INFINITY);
        assert_eq!(jet_oaspl_db(-50.0, 0.05, 1.2, 7.5), f64::NEG_INFINITY);
        let mut j = JetNoise::new(1);
        let dt = 1.0 / 192_000.0;
        assert!((0..1000).all(|_| j.step(-50.0, 0.05, 1.2, 7.5, dt) == 0.0));
        // nor does a closed pipe or a listener at the outlet itself
        assert_eq!(jet_oaspl_db(100.0, 0.0, 1.2, 7.5), f64::NEG_INFINITY);
        assert_eq!(jet_oaspl_db(100.0, 0.05, 1.2, 0.0), f64::NEG_INFINITY);
    }

    #[test]
    fn a_fast_jet_loses_a_little_to_its_mach_term() {
        // M = 2 (V = 686 m/s), cold, D = 50 mm, R = 7.5 m: 141 + 75·log(2) − 10·log(1 + 0.01·2^4.5)
        // + 10·log(1.9635e-3) − 20·log(7.5) = 141 + 22.577 − 0.886 − 27.069 − 17.501 = 118.12 dB
        assert_relative_eq!(
            jet_oaspl_db(686.0, 0.05, 1.204, 7.5),
            118.120,
            epsilon = 0.01
        );
    }

    #[test]
    fn the_noise_is_vignas_xorshift64_star() {
        // the first draws for seed 7, from the published algorithm (Vigna 2016, "An experimental
        // exploration of Marsaglia's xorshift generators, scrambled", xorshift64*: shifts 12, 25,
        // 27, multiplier 2685821657736338717) run independently, top 53 bits to [−√3, √3)
        let mut j = JetNoise::new(7);
        assert_eq!(
            [j.white(), j.white(), j.white()],
            [
                -0.336_660_087_749_421_47,
                -0.326_175_457_211_438,
                0.315_383_878_342_838_76
            ]
        );
    }

    #[test]
    fn a_thin_fast_jet_keeps_its_level_with_its_peak_past_nyquist() {
        // V = 300 m/s through D = 2 mm peaks at 0.2·V/D = 30 kHz, past the 24 kHz Nyquist of a
        // 48 kHz step: the band is held at 0.2·fs and the signal still carries the level
        let dt = 1.0 / 48_000.0;
        let mut j = JetNoise::new(3);
        let xs: Vec<f64> = (0..200_000)
            .map(|_| j.step(300.0, 0.002, 1.204, 7.5, dt))
            .skip(10_000)
            .collect();
        let rms = (xs.iter().map(|x| x * x).sum::<f64>() / xs.len() as f64).sqrt();
        let level = 20e-6 * 10f64.powf(jet_oaspl_db(300.0, 0.002, 1.204, 7.5) / 20.0);
        assert_relative_eq!(rms, level, max_relative = 0.1);
    }

    #[test]
    fn the_signal_carries_the_level_around_its_strouhal_peak() {
        // steady 100 m/s jet: rms within 5 % of 10^(56.282/20)·20 µPa = 0.013035 Pa, and most
        // of its energy near f = 0.2·V/D = 400 Hz (more within 200–800 Hz than outside)
        let dt = 1.0 / 192_000.0;
        let mut j = JetNoise::new(7);
        let xs: Vec<f64> = (0..400_000)
            .map(|_| j.step(100.0, 0.05, 1.204, 7.5, dt))
            .skip(20_000)
            .collect();
        let rms = (xs.iter().map(|x| x * x).sum::<f64>() / xs.len() as f64).sqrt();
        assert_relative_eq!(rms, 0.013_035, max_relative = 0.05);
        // energy in 200–800 Hz from the DFT power at a few bins inside and outside
        let power = |f: f64| {
            let w = 2.0 * PI * f * dt;
            let (mut re, mut im) = (0.0, 0.0);
            for (n, x) in xs.iter().enumerate() {
                re += x * (w * n as f64).cos();
                im += x * (w * n as f64).sin();
            }
            re * re + im * im
        };
        let inside: f64 = [300.0, 400.0, 550.0].iter().map(|f| power(*f)).sum();
        let outside: f64 = [50.0, 3_000.0, 6_000.0].iter().map(|f| power(*f)).sum();
        assert!(
            inside > 3.0 * outside,
            "inside {inside:.3e} outside {outside:.3e}"
        );
    }
}
