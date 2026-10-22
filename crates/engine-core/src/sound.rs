//! Sound texture beyond the tonal pulses: turbulent flow noise, intake roar, valvetrain
//! clicks and combustion variability. Spec: `docs/superpowers/specs/2026-10-21-sound-realism.md`.
//! Deterministic (seeded), SI, pure; the engine calls it once per step.

use serde::{Deserialize, Serialize};

/// Levels of each texture source; 0 disables it, 1 is the calibrated default.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SoundDef {
    /// Broadband turbulence noise (a heuristic, off by default): at the tail pipes ∝ the
    /// outflow's dynamic pressure, and the throttle's hiss ∝ its momentum flux. Measured through
    /// the physical radiation chain it buried the engine orders (an inline-4 at full throttle
    /// went from 99 % to 55 % order energy); the tailpipe jet is to be modelled physically.
    pub flow_noise: f64,
    /// Induction sound: the throttle's plane-wave tone and the 1D intake's mouth signal.
    pub intake: f64,
    /// Valvetrain clicks at every valve closing, stronger with rpm.
    pub mechanical: f64,
    /// Cycle-to-cycle combustion variability (1 = ±1.5° burn start, ±5 % duration).
    pub combustion_jitter: f64,
    /// Wet level of the body/room reverb applied in the voice.
    pub reverb: f64,
    /// How much the voice lifts quiet passages (idle) toward the full-load level, in dB:
    /// 1 = up to +12 dB, 0 = the raw dynamics of the physics (the default: idle, overrun and
    /// full load keep their real level differences).
    pub leveling: f64,
    /// Where the listener stands: distance from the tail pipes' centre, m, and angle off the
    /// car's axis, degrees (0 = straight behind, + to the left). Off the axis, several tails
    /// arrive with different delays.
    pub listener_distance_m: f64,
    pub listener_angle_deg: f64,
}

impl Default for SoundDef {
    fn default() -> Self {
        Self {
            flow_noise: 0.0,
            intake: 1.0,
            mechanical: 1.0,
            combustion_jitter: 1.0,
            reverb: 0.15,
            leveling: 0.0,
            listener_distance_m: 3.0,
            listener_angle_deg: 30.0,
        }
    }
}

/// Burn-start jitter (degrees) and duration jitter (fraction) at `combustion_jitter` = 1.
/// Cyclic variability of SI engines is a few degrees of CA50 (Heywood, ch. 9.4).
pub const BURN_START_JITTER_DEG: f64 = 1.5;
pub const BURN_DURATION_JITTER: f64 = 0.05;

/// Density × speed of sound of hot exhaust at the outlet (≈ 0.6 kg/m³ · 500 m/s).
const RHO_C_EXHAUST: f64 = 300.0;
const RHO_EXHAUST: f64 = 0.6;
const RHO_AIR: f64 = 1.2;
const C_AIR: f64 = 343.0;
/// Fraction of the outflow dynamic pressure radiated as broadband noise (calibrated by ear
/// against the tonal level; a free parameter of this lumped model). 0.6 drowned a screaming
/// V10 in "air" (user report 2026-10-22; 53 % of the energy in engine orders at 8000 rpm);
/// 0.2 keeps the noise ~10 dB under the firing orders there.
const FLOW_NOISE_GAIN: f64 = 0.2;
const INTAKE_TONE_GAIN: f64 = 0.15;
const INTAKE_NOISE_GAIN: f64 = 0.4;
const CLICK_GAIN_PER_KRPM: f64 = 60.0;
const CLICK_DECAY_S: f64 = 0.0004;

/// One-pole band-pass (high-pass then low-pass) with state.
#[derive(Debug, Clone, Default)]
struct Band {
    hp_x: f64,
    hp_y: f64,
    lp: f64,
}

impl Band {
    fn process(&mut self, x: f64, lo_hz: f64, hi_hz: f64, dt: f64) -> f64 {
        let a = (-2.0 * std::f64::consts::PI * lo_hz * dt).exp();
        let hp = a * (self.hp_y + x - self.hp_x);
        self.hp_x = x;
        self.hp_y = hp;
        let k = 1.0 - (-2.0 * std::f64::consts::PI * hi_hz * dt).exp();
        self.lp += k * (hp - self.lp);
        self.lp
    }
}

/// Runtime state of the texture sources.
#[derive(Debug, Clone)]
pub struct Texture {
    rng: u64,
    jet: Band,
    intake: Band,
    click: Band,
    click_env: f64,
}

impl Texture {
    pub fn new(seed: u64) -> Self {
        Self {
            rng: seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1,
            jet: Band::default(),
            intake: Band::default(),
            click: Band::default(),
            click_env: 0.0,
        }
    }

    /// Uniform white noise in [-1, 1) (xorshift64*).
    pub fn white(&mut self) -> f64 {
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        let x = self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D);
        (x >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    }

    /// Jet noise for the radiated outflow signal `radiated` (= ρc·u, Pa, summed over
    /// `outlets` equal tail pipes): band-passed noise with amplitude `level`·k·½ρu² per jet
    /// (dynamic pressure; only outflow makes a jet), the jets adding incoherently (×√n).
    pub fn flow_noise(&mut self, radiated: f64, outlets: usize, level: f64, dt: f64) -> f64 {
        if level <= 0.0 {
            return 0.0;
        }
        let n_out = outlets.max(1) as f64;
        let u = (radiated / (n_out * RHO_C_EXHAUST)).max(0.0);
        let q = 0.5 * RHO_EXHAUST * u * u;
        let n = self.white();
        level * FLOW_NOISE_GAIN * n_out.sqrt() * q * self.jet.process(n, 400.0, 6_000.0, dt)
    }

    /// Induction sound from the throttle mass flow (kg/s into the plenum): the plane-wave tone
    /// c·ṁ/A_bore at `level`, plus hiss ∝ momentum flux ṁ·u / A_bore (u through the current
    /// opening) at `hiss_level`. Using the full bore keeps a nearly closed throttle (sonic,
    /// tiny ṁ) from screaming.
    pub fn intake(
        &mut self,
        throttle_flow: f64,
        throttle_area: f64,
        bore_area: f64,
        (level, hiss_level): (f64, f64),
        dt: f64,
    ) -> f64 {
        if (level <= 0.0 && hiss_level <= 0.0) || throttle_area <= 0.0 || bore_area <= 0.0 {
            return 0.0;
        }
        let m = throttle_flow.abs();
        let tone = INTAKE_TONE_GAIN * C_AIR * m / bore_area;
        let u = (m / (RHO_AIR * throttle_area)).min(C_AIR);
        let n = self.white();
        let hiss =
            INTAKE_NOISE_GAIN * m * u / bore_area * self.intake.process(n, 800.0, 8_000.0, dt);
        level * tone + hiss_level * hiss
    }

    /// Register a valve closing at `rpm`.
    pub fn valve_closed(&mut self, rpm: f64, level: f64) {
        if level > 0.0 {
            self.click_env += level * CLICK_GAIN_PER_KRPM * rpm / 1000.0;
        }
    }

    /// Valvetrain click output for this step (decaying high-passed noise bursts).
    pub fn clicks(&mut self, dt: f64) -> f64 {
        if self.click_env == 0.0 {
            return 0.0;
        }
        let n = self.white();
        let out = self.click_env * self.click.process(n, 2_000.0, 12_000.0, dt);
        self.click_env *= (-dt / CLICK_DECAY_S).exp();
        if self.click_env < 1e-4 {
            self.click_env = 0.0;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1.0 / 192_000.0;

    #[test]
    fn listener_round_trips_and_defaults_for_old_files() {
        let old: SoundDef = serde_yaml_ng::from_str("reverb: 0.2\n").unwrap();
        assert_eq!(
            (old.listener_distance_m, old.listener_angle_deg),
            (3.0, 30.0)
        );
        let s = SoundDef {
            listener_distance_m: 7.5,
            listener_angle_deg: -45.0,
            ..SoundDef::default()
        };
        let json: SoundDef = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        let yaml: SoundDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&s).unwrap()).unwrap();
        assert_eq!((json, yaml), (s.clone(), s));
    }

    #[test]
    fn leveling_round_trips_and_defaults_to_the_raw_physics() {
        let old: SoundDef = serde_yaml_ng::from_str("reverb: 0.2\n").unwrap();
        assert_eq!(old.leveling, 0.0);
        let s = SoundDef {
            leveling: 0.3,
            ..SoundDef::default()
        };
        let json: SoundDef = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        let yaml: SoundDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&s).unwrap()).unwrap();
        assert_eq!(json, s);
        assert_eq!(yaml, s);
    }

    fn rms(x: &[f64]) -> f64 {
        (x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64).sqrt()
    }

    #[test]
    fn white_noise_is_uniform_and_deterministic() {
        let mut t = Texture::new(5);
        let xs: Vec<f64> = (0..20_000).map(|_| t.white()).collect();
        assert!(xs.iter().all(|x| (-1.0..1.0).contains(x)));
        let mean = xs.iter().sum::<f64>() / xs.len() as f64;
        assert!(mean.abs() < 0.02);
        // uniform(-1, 1): rms 1/√3 = 0.577
        assert!((rms(&xs) - 0.577).abs() < 0.01, "{}", rms(&xs));
        assert_eq!(Texture::new(5).white(), xs[0]);
    }

    #[test]
    fn band_passes_the_band_and_rejects_dc_and_extremes() {
        // one-pole high-pass 400 Hz then one-pole low-pass 6 kHz, at 192 kHz
        let gain = |f: f64| {
            let mut b = Band::default();
            let mut peak: f64 = 0.0;
            for n in 0..40_000 {
                let y = b.process(
                    (2.0 * std::f64::consts::PI * f * n as f64 * DT).sin(),
                    400.0,
                    6_000.0,
                    DT,
                );
                if n > 20_000 {
                    peak = peak.max(y.abs());
                }
            }
            peak
        };
        // |H| ≈ 1/√(1+(400/f)²) · 1/√(1+(f/6000)²): 2 kHz → 0.93; 100 Hz → 0.24; 60 kHz → 0.10
        assert!((gain(2_000.0) - 0.93).abs() < 0.05, "{}", gain(2_000.0));
        assert!(gain(100.0) < 0.3, "{}", gain(100.0));
        assert!(gain(60_000.0) < 0.2, "{}", gain(60_000.0));
        let mut b = Band::default();
        let mut y = 1.0;
        for _ in 0..20_000 {
            y = b.process(1.0, 400.0, 6_000.0, DT);
        }
        assert!(y.abs() < 1e-3, "DC must be rejected: {y}");
    }

    #[test]
    fn white_noise_is_uncorrelated() {
        let mut t = Texture::new(9);
        let xs: Vec<f64> = (0..50_000).map(|_| t.white()).collect();
        let r1 = xs.windows(2).map(|w| w[0] * w[1]).sum::<f64>() / xs.len() as f64 / (1.0 / 3.0);
        assert!(r1.abs() < 0.02, "lag-1 correlation {r1}");
        // uniform(−1, 1): fourth moment 1/5
        let m4 = xs.iter().map(|x| x.powi(4)).sum::<f64>() / xs.len() as f64;
        assert!((m4 - 0.2).abs() < 0.01, "{m4}");
    }

    #[test]
    fn each_tail_pipe_is_its_own_jet() {
        // `radiated` sums the outlets. Two tails carrying 2·r are two jets at the velocity of
        // r, whose noises add incoherently: √2 × one jet, not the 4× of one jet at twice u.
        let (mut one, mut two) = (Texture::new(7), Texture::new(7));
        for _ in 0..2_000 {
            let a = one.flow_noise(6_000.0, 1, 1.0, DT);
            let b = two.flow_noise(12_000.0, 2, 1.0, DT);
            assert!((b - std::f64::consts::SQRT_2 * a).abs() <= 1e-12 * a.abs().max(1e-30));
        }
    }

    #[test]
    fn flow_noise_scales_with_dynamic_pressure_and_needs_outflow() {
        let level = |radiated: f64| {
            let mut t = Texture::new(1);
            let xs: Vec<f64> = (0..40_000)
                .map(|_| t.flow_noise(radiated, 1, 1.0, DT))
                .collect();
            rms(&xs[10_000..])
        };
        let (slow, fast) = (level(3_000.0), level(6_000.0));
        // u doubles → ½ρu² quadruples
        assert!((fast / slow - 4.0).abs() < 0.2, "{slow} → {fast}");
        assert_eq!(level(-6_000.0), 0.0, "inflow makes no jet");
        let mut t = Texture::new(1);
        assert_eq!(t.flow_noise(6_000.0, 1, 0.0, DT), 0.0, "level 0 disables");
    }

    #[test]
    fn intake_follows_throttle_flow() {
        let level = |m: f64| {
            let mut t = Texture::new(2);
            let xs: Vec<f64> = (0..40_000)
                .map(|k| {
                    t.intake(
                        m * (1.0 + 0.5 * (k as f64 * 0.01).sin()),
                        1e-3,
                        1e-3,
                        (1.0, 1.0),
                        DT,
                    )
                })
                .collect();
            rms(&xs[10_000..])
        };
        assert!(
            level(0.1) > 3.0 * level(0.01),
            "{} vs {}",
            level(0.1),
            level(0.01)
        );
        assert_eq!(level(0.0), 0.0);
        // nearly closed throttle: same tiny flow through a small opening stays quiet
        let mut t = Texture::new(2);
        let closed: Vec<f64> = (0..40_000)
            .map(|_| t.intake(0.002, 2e-6, 1e-3, (1.0, 1.0), DT))
            .collect();
        assert!(
            rms(&closed[10_000..]) < 0.1 * level(0.1),
            "{}",
            rms(&closed[10_000..])
        );
    }

    #[test]
    fn intake_tone_and_hiss_follow_the_formulas() {
        // tone = 0.15 · c · ṁ / A_bore = 0.15 · 343 · 0.05 / 1e-3 = 2572.5 Pa; the hiss is
        // zero-mean, so the mean of the output is the tone.
        let mut t = Texture::new(4);
        let xs: Vec<f64> = (0..40_000)
            .map(|_| t.intake(0.05, 1e-3, 1e-3, (1.0, 1.0), DT))
            .collect();
        let mean = xs.iter().sum::<f64>() / xs.len() as f64;
        assert!((mean / 2572.5 - 1.0).abs() < 0.02, "tone {mean}");
        // hiss ∝ ṁ·u with u = ṁ/(ρA): doubling ṁ quadruples it
        let hiss = |m: f64| {
            let mut t = Texture::new(4);
            let tone = 0.15 * 343.0 * m / 1e-3;
            let xs: Vec<f64> = (0..40_000)
                .map(|_| t.intake(m, 1e-3, 1e-3, (1.0, 1.0), DT) - tone)
                .collect();
            rms(&xs[10_000..])
        };
        assert!(
            (hiss(0.02) / hiss(0.01) - 4.0).abs() < 0.3,
            "{} {}",
            hiss(0.01),
            hiss(0.02)
        );
        // hiss ∝ ṁ·u/A_bore with u = ṁ/(ρ·A_throttle): doubling the throttle opening halves it,
        // doubling the bore halves it too.
        let hiss_at = |a_throttle: f64, a_bore: f64| {
            let mut t = Texture::new(4);
            let tone = 0.15 * 343.0 * 0.02 / a_bore;
            let xs: Vec<f64> = (0..40_000)
                .map(|_| t.intake(0.02, a_throttle, a_bore, (1.0, 1.0), DT) - tone)
                .collect();
            rms(&xs[10_000..])
        };
        let base = hiss_at(1e-3, 1e-3);
        assert!(
            (hiss_at(2e-3, 1e-3) / base - 0.5).abs() < 0.05,
            "throttle area"
        );
        assert!((hiss_at(1e-3, 2e-3) / base - 0.5).abs() < 0.05, "bore area");
        // degenerate areas give silence, not infinities
        let mut t = Texture::new(4);
        assert_eq!(t.intake(0.05, 0.0, 1e-3, (1.0, 1.0), DT), 0.0);
        assert_eq!(t.intake(0.05, 1e-3, 0.0, (1.0, 1.0), DT), 0.0);
    }

    #[test]
    fn a_valve_closing_adds_level_times_60_per_krpm() {
        let mut t = Texture::new(1);
        t.valve_closed(3_000.0, 2.0);
        assert_eq!(t.click_env, 2.0 * 60.0 * 3.0);
        t.valve_closed(1_000.0, 1.0);
        assert_eq!(t.click_env, 360.0 + 60.0);
        // one step decays by exp(−dt/0.4 ms); the burst is bounded by the envelope
        let before = t.click_env;
        let out = t.clicks(DT);
        assert!((t.click_env / before - (-DT / 0.0004f64).exp()).abs() < 1e-12);
        assert!(
            out.abs() <= before,
            "output {out} beyond its envelope {before}"
        );
        // the burst is zero-mean noise shaped by the envelope
        let mut t2 = Texture::new(8);
        t2.valve_closed(5_000.0, 1.0);
        let env0 = t2.click_env;
        let xs: Vec<f64> = (0..200).map(|_| t2.clicks(DT)).collect();
        let mean = xs.iter().sum::<f64>() / xs.len() as f64;
        assert!(
            mean.abs() < 0.05 * env0,
            "click mean {mean} vs envelope {env0}"
        );
        // below 1e-4 the envelope snaps to exactly zero
        // (1.5e-4 → 1e-4 takes ln(1.5)·0.4 ms / dt ≈ 31 steps)
        t.click_env = 1.5e-4;
        for _ in 0..40 {
            t.clicks(DT);
        }
        assert_eq!(t.click_env, 0.0);
    }

    const REFERENCE_WHITE_SEED5: [f64; 3] =
        [-0.856932398150813, 0.04843985850807053, 0.43598306058512826];

    #[test]
    fn white_matches_the_reference_xorshift64_star() {
        // Vigna's xorshift64* (shifts 12, 25, 27; multiplier 0x2545F4914F6CDD1D), seeded here as
        // seed·0xD1B54A32D192ED03 | 1; reference values from an independent Python implementation.
        let mut t = Texture::new(5);
        let got: Vec<f64> = (0..3).map(|_| t.white()).collect();
        let want = REFERENCE_WHITE_SEED5;
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-15, "{got:?}");
        }
    }

    #[test]
    fn valve_clicks_decay_and_grow_with_rpm() {
        let energy = |rpm: f64| {
            let mut t = Texture::new(3);
            t.valve_closed(rpm, 1.0);
            let xs: Vec<f64> = (0..2_000).map(|_| t.clicks(DT)).collect();
            let late: f64 = (1_500..2_000).map(|k| xs[k].abs()).sum();
            (rms(&xs), late)
        };
        let (low, _) = energy(1_000.0);
        let (high, late) = energy(8_000.0);
        assert!(high > 4.0 * low, "{low} → {high}");
        assert!(late < 1e-6, "clicks must die out (~0.4 ms): {late}");
        let mut t = Texture::new(3);
        t.valve_closed(8_000.0, 0.0);
        assert!((0..100).all(|_| t.clicks(DT) == 0.0));
    }
}
