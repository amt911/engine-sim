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

/// [`jet_oaspl_db`] as an rms pressure, Pa, worked out in linear terms for the per-step path
/// (one `powf` where the decibel form takes four and five logarithms):
/// p = P_REF·10^7.05·(ρⱼ/ρₐ)^(w/2)·M^3.75/√(1 + 0.01·M^4.5)·√Aⱼ/R. No outflow: 0.
fn jet_p_rms(v: f64, d: f64, rho_jet: f64, r: f64) -> f64 {
    if v <= 0.0 || d <= 0.0 || r <= 0.0 {
        return 0.0;
    }
    // P_REF·10^(141/20)
    const P_141: f64 = P_REF * 11_220_184.543_019_63;
    let m = v / C_AMBIENT;
    let sm = m.sqrt();
    let m35 = m * m * m * sm;
    let w = 3.0 * m35 / (0.6 + m35) - 1.0;
    let area = PI * 0.25 * d * d;
    P_141 * (rho_jet / RHO_AMBIENT).powf(0.5 * w) * m * m * m * (sm * sm.sqrt())
        / (1.0 + 0.01 * m35 * m).sqrt()
        * area.sqrt()
        / r
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
        let p_rms = jet_p_rms(v, d, rho_jet, r);
        if p_rms == 0.0 {
            // nothing blows out: no jet, no sound. The band-pass runs down on silence, so the
            // next outflow starts from rest (its state is unit noise, not pascals)
            self.band(0.0, 20.0, dt);
            return 0.0;
        }
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

/// The fine-scale turbulence similarity spectrum of jet mixing noise, G(f/f_F) of Tam,
/// Golebiowski & Seiner (AIAA 96-1716, 1996, eq. 6; one shape for all their subsonic and
/// supersonic jet data, f_F its peak), as a filter whose peak follows a moving f_F: a first-order
/// high-pass at 0.1411·f_F and a second-order low-pass at 6.7866·f_F (Q 0.4094), fitted to
/// eq. (6) within 0.47 dB rms (1.5 dB at worst) from 0.02 to 50·f_F. (Eq. 6 as printed takes
/// log(f/f_F) between 10 and 30·f_F; it only joins its neighbours with log(f/10f_F), read so.)
#[derive(Debug, Clone, Default)]
pub struct FineScale {
    hp: (f64, f64),
    lp: [f64; 2],
    coef: [f64; 6],
    f_peak: f64,
}

impl FineScale {
    const HP: f64 = 0.1411;
    const LP: f64 = 6.7866;
    const Q: f64 = 0.4094;
    /// 10·log of the fitted shape at its peak (the fit's +0.105 dB offset taken out).
    const GAIN: f64 = 0.987_970_6;

    pub fn new() -> Self {
        Self::default()
    }

    /// One step of `dt` of `x` through the spectrum peaking at `f_peak` Hz.
    pub fn process(&mut self, x: f64, f_peak: f64, dt: f64) -> f64 {
        if (f_peak - self.f_peak).abs() > 0.005 * self.f_peak.max(1.0) {
            self.retune(f_peak, dt);
        }
        let [g, a1h, b0, b1, a1, a2] = self.coef;
        // first-order high-pass (bilinear): y = g·(x − x₁) − a1h·y₁
        let h = g * (x - self.hp.0) - a1h * self.hp.1;
        self.hp = (x, h);
        // second-order low-pass, transposed direct form II
        let y = b0 * h + self.lp[0];
        self.lp[0] = b1 * h - a1 * y + self.lp[1];
        self.lp[1] = b0 * h - a2 * y;
        Self::GAIN * y
    }

    fn retune(&mut self, f_peak: f64, dt: f64) {
        let nyq = 0.45 / dt;
        let w = |f: f64| (std::f64::consts::PI * f.min(nyq) * dt).tan();
        // high-pass s/(s + ω): bilinear with pre-warping
        let kh = w(Self::HP * f_peak);
        let g = 1.0 / (1.0 + kh);
        let a1h = (kh - 1.0) / (1.0 + kh);
        // low-pass 1/(1 + s/(Qω) + (s/ω)²)
        let k = w(Self::LP * f_peak);
        let norm = 1.0 / (1.0 + k / Self::Q + k * k);
        let b0 = k * k * norm;
        let b1 = 2.0 * b0;
        let a1 = 2.0 * (k * k - 1.0) * norm;
        let a2 = (1.0 - k / Self::Q + k * k) * norm;
        self.coef = [g, a1h, b0, b1, a1, a2];
        self.f_peak = f_peak;
    }
}

/// The jet through an open exhaust valve, heard inside the port: gas expanding from the
/// cylinder (`p_cyl`, `t_cyl`) to the port (`p_port`, `t_port`) through the curtain's effective
/// area `a_curtain`, isentropically, at most at c* (choked). Its turbulent mixing noise follows
/// the same correlation as a tail jet's ([`jet_oaspl_db`], ARP876), with the port's gas as the
/// medium (ρc² = γp: at one pressure the correlation needs no correction for hot gas); its
/// acoustic power W = 4πr²·p²/(ρc) goes into the primary of area `a_pipe` as plane waves (below
/// the first cut-on, 1.84c/πD), half downstream: p = √(½·W·ρc/A). The plane-wave rms pressure
/// it sends down the pipe, Pa; 0 with the valve shut or the flow reversed.
pub fn valve_jet_level(
    (p_cyl, t_cyl): (f64, f64),
    (p_port, t_port): (f64, f64),
    a_curtain: f64,
    a_pipe: f64,
) -> (f64, f64, f64) {
    use crate::gas::{CP, GAMMA, R_AIR};
    if a_curtain <= 0.0 || p_port >= p_cyl || t_cyl <= 0.0 || t_port <= 0.0 {
        return (0.0, 0.0, 0.0);
    }
    let ratio = (p_port / p_cyl).max(crate::gas::CRITICAL_PRESSURE_RATIO);
    // the step's hot path: the correlation's dB terms taken as one product (no logs), and
    // ratio^((γ−1)/γ) = ratio / ratio^(1/γ)
    let r_g = ratio.powf(1.0 / GAMMA);
    let u = (2.0 * CP * t_cyl * (1.0 - ratio / r_g)).sqrt();
    // ρ_jet / ρ_port, both ideal gas
    let density_ratio = p_cyl * r_g * t_port / (t_cyl * p_port);
    let c_a = (GAMMA * R_AIR * t_port).sqrt();
    let d = (4.0 * a_curtain / PI).sqrt();
    let m = u / c_a;
    let sqrt_m = m.sqrt();
    let m35 = m * m * m * sqrt_m;
    let w = 3.0 * m35 / (0.6 + m35) - 1.0;
    // p₁² = P_REF²·10^14.1·(ρ_j/ρ_a)^w·M^7.5/(1 + 0.01·M^4.5)·A; W = 4πr²p₁²/(ρc), half of it
    // down the pipe as a plane wave p = √(½Wρc/A_pipe): ρc cancels
    const P_REF_10_705: f64 = P_REF * 1.122_018_454_301_963_3e7;
    let p1_sq = density_ratio.powf(w) * m35 * m35 * sqrt_m / (1.0 + 0.01 * m35 * m) * a_curtain;
    (P_REF_10_705 * (2.0 * PI * p1_sq / a_pipe).sqrt(), u, d)
}

/// The noise of one exhaust valve's jet as a pressure signal in its primary: seeded white noise
/// through Tam's fine-scale spectrum ([`FineScale`]) peaking at the jet's Strouhal frequency
/// 0.2·u/d, scaled to [`valve_jet_level`].
#[derive(Debug, Clone)]
pub struct ValveJet {
    rng: u64,
    spectrum: FineScale,
}

impl ValveJet {
    /// ∫ of the fitted fine-scale shape's power over f/f_F (the filter's own, for its level).
    const SHAPE_INTEGRAL: f64 = 4.052_433_9;

    pub fn new(seed: u64) -> Self {
        Self {
            rng: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            spectrum: FineScale::new(),
        }
    }

    /// One step of `dt`: the pressure (Pa) the valve's jet sends down the primary.
    pub fn step(
        &mut self,
        cyl: (f64, f64),
        port: (f64, f64),
        a_curtain: f64,
        a_pipe: f64,
        dt: f64,
    ) -> f64 {
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        let x = (self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64;
        let white = (x - 0.5) * 12f64.sqrt();
        let (level, u, d) = valve_jet_level(cyl, port, a_curtain, a_pipe);
        if level == 0.0 {
            self.spectrum
                .process(0.0, self.spectrum.f_peak.max(1.0), dt);
            return 0.0;
        }
        // the shape (to 50·f_F) inside the band the steps resolve
        let f_peak = (PEAK_STROUHAL * u / d).min(0.05 / dt);
        // unit white noise through the shape keeps 2·f_F·∫G/fs of its variance
        let kept = (2.0 * f_peak * Self::SHAPE_INTEGRAL * dt).sqrt();
        level / kept * self.spectrum.process(white, f_peak, dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_choked_exhaust_valve_jet_by_hand() {
        // 3 bar, 1200 K in the cylinder, the port at 1 bar, 900 K: choked (1/3 < 0.528), the jet
        // at c* = √(2γRT/(γ + 1)) = 633.9 m/s with the throat's 0.8711·0.6339 = 0.5522 kg/m³,
        // into port gas of 0.38715 kg/m³ and c = 601.4 m/s: M = 1.0541, w = 1.0014. Through 4 cm²:
        // 141 + 1.5444 + 1.7163 − 0.0547 − 33.979 = 110.227 dB at 1 m (6.497 Pa), W =
        // 4π·6.497²/(0.38715·601.4) = 2.278 W, half of it down a 42 mm primary (1.3854e-3 m²):
        // √(½·2.278·232.83/1.3854e-3) = 437.5 Pa
        let a_pipe = PI * 0.25 * 0.042 * 0.042;
        let (p, u, _) = valve_jet_level((3e5, 1200.0), (1e5, 900.0), 4e-4, a_pipe);
        assert!((u - 633.9).abs() < 0.2, "{u} m/s");
        assert!((p / 437.5 - 1.0).abs() < 0.01, "{p} Pa");
        // shut, or the port above the cylinder (reversion): no jet
        assert_eq!(
            valve_jet_level((3e5, 1200.0), (1e5, 900.0), 0.0, a_pipe).0,
            0.0
        );
        assert_eq!(
            valve_jet_level((1e5, 900.0), (1.2e5, 900.0), 4e-4, a_pipe).0,
            0.0
        );
    }

    #[test]
    fn a_valve_jet_sends_its_level_down_the_pipe() {
        // the generated noise carries valve_jet_level's rms (within the variance of 0.2 s)
        let dt = 1.0 / 192_000.0;
        let a_pipe = PI * 0.25 * 0.042 * 0.042;
        let mut j = ValveJet::new(3);
        let n = (0.2 / dt) as usize;
        let cyl = (3e5, 1200.0);
        let port = (1e5, 900.0);
        let x: Vec<f64> = (0..n)
            .map(|_| j.step(cyl, port, 4e-4, a_pipe, dt))
            .collect();
        let rms = (x.iter().map(|v| v * v).sum::<f64>() / n as f64).sqrt();
        assert!((rms / 437.5 - 1.0).abs() < 0.1, "{rms} Pa");
        // and nothing while the valve is shut
        assert!((0..1000).all(|_| j.step(cyl, port, 0.0, a_pipe, dt) == 0.0));
    }

    #[test]
    fn the_fine_scale_spectrum_is_tams() {
        // a tone through the filter against eq. (6) of Tam, Golebiowski & Seiner (1996):
        // 10·log G = 0 at the peak; −5.57 dB at 0.1·f_F; −2.25 at 3·f_F; −11.8 at 10·f_F;
        // −26.6 at 30·f_F (at 0.1·f_F, the segment below 0.15: −3.5 + [11.874876 + 2.1202444·m +
        // 7.5211814·m²]·m with m = log(20·0.1/3) = −0.1761, = −3.5 − 2.067 = −5.57)
        let fs = 192_000.0;
        let f_peak = 1000.0;
        let level = |f: f64| {
            let mut g = FineScale::new();
            let n = (0.5 * fs) as usize;
            let mut peak = 0.0f64;
            for i in 0..n {
                let y = g.process((2.0 * PI * f * i as f64 / fs).sin(), f_peak, 1.0 / fs);
                if i > n / 2 {
                    peak = peak.max(y.abs());
                }
            }
            20.0 * peak.log10()
        };
        for (x, db) in [
            (0.1, -5.57),
            (1.0, 0.0),
            (3.0, -2.25),
            (10.0, -11.8),
            (30.0, -26.6),
        ] {
            let got = level(x * f_peak);
            assert!((got - db).abs() < 1.6, "{x}·f_F: {got:.2} dB, eq. 6 {db}");
        }
    }
    use approx::assert_relative_eq;

    #[test]
    fn no_outflow_makes_no_jet_noise() {
        // an idling engine's tail flow swings through zero every pulse: while nothing blows out
        // there is no jet, so no mixing noise — not the band-pass's leftover state
        let dt = 1.0 / 192_000.0;
        let mut j = JetNoise::new(7);
        for _ in 0..2_000 {
            j.step(20.0, 0.05, 0.6, 3.0, dt);
        }
        for k in 0..2_000 {
            let p = j.step(-3.0, 0.05, 0.6, 3.0, dt);
            assert_eq!(p, 0.0, "step {k}: {p} Pa with the flow reversed");
        }
        // and blowing again, it is back at the correlation's level
        let n = 20_000;
        let rms = ((0..n)
            .map(|_| j.step(20.0, 0.05, 0.6, 3.0, dt).powi(2))
            .sum::<f64>()
            / n as f64)
            .sqrt();
        let want = jet_p_rms(20.0, 0.05, 0.6, 3.0);
        assert!((rms / want - 1.0).abs() < 0.15, "{rms} vs {want}");
    }

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
    fn the_linear_level_is_the_decibel_one() {
        // the per-step pressure is the correlation's level, 20 µPa·10^(L/20), to round-off
        for (v, d, rho, r) in [
            (100.0, 0.05, 1.204, 7.5),
            (100.0, 0.05, 0.567, 7.5),
            (686.0, 0.05, 1.204, 7.5),
            (30.0, 0.08, 0.4, 3.0),
            (300.0, 0.002, 1.204, 15.0),
        ] {
            assert_relative_eq!(
                jet_p_rms(v, d, rho, r),
                20e-6 * 10f64.powf(jet_oaspl_db(v, d, rho, r) / 20.0),
                max_relative = 1e-12
            );
        }
        assert_eq!(jet_p_rms(0.0, 0.05, 1.2, 7.5), 0.0);
        assert_eq!(jet_p_rms(100.0, 0.0, 1.2, 7.5), 0.0);
        assert_eq!(jet_p_rms(100.0, 0.05, 1.2, 0.0), 0.0);
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
    fn the_band_peaks_at_its_strouhal_frequency_two_octaves_wide() {
        // fc = 400 Hz ≪ fs = 192 kHz, so the analog band-pass H = (jω/Qω₀)/(1 − ω²/ω₀² + jω/Qω₀):
        // 1 at ω₀, and an octave either side (2/0.7)/√((1 − 4)² + (2/0.7)²) = 0.6897
        let dt = 1.0 / 192_000.0;
        let gain = |f: f64| {
            let mut j = JetNoise::new(1);
            let y: Vec<f64> = (0..192_000)
                .map(|n| j.band((2.0 * PI * f * n as f64 * dt).sin(), 400.0, dt))
                .collect();
            y[96_000..].iter().fold(0.0f64, |m, v| m.max(v.abs()))
        };
        assert_relative_eq!(gain(400.0), 1.0, max_relative = 2e-3);
        assert_relative_eq!(gain(800.0), 0.6897, max_relative = 2e-3);
        assert_relative_eq!(gain(200.0), 0.6897, max_relative = 2e-3);
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
