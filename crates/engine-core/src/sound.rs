//! Sound texture beyond the tonal pulses: turbulent flow noise, intake roar, valvetrain
//! clicks and combustion variability. Spec: `docs/superpowers/specs/2026-10-21-sound-realism.md`.
//! Deterministic (seeded), SI, pure; the engine calls it once per step.

use serde::{Deserialize, Serialize};

/// Levels of each texture source; 0 disables it, 1 is the calibrated default.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SoundDef {
    /// Broadband jet/turbulence noise at the tail pipes, ∝ dynamic pressure of the outflow.
    pub flow_noise: f64,
    /// Induction sound from the throttle flow.
    pub intake: f64,
    /// Valvetrain clicks at every valve closing, stronger with rpm.
    pub mechanical: f64,
    /// Cycle-to-cycle combustion variability (1 = ±1.5° burn start, ±5 % duration).
    pub combustion_jitter: f64,
    /// Wet level of the body/room reverb applied in the voice.
    pub reverb: f64,
}

impl Default for SoundDef {
    fn default() -> Self {
        Self {
            flow_noise: 1.0,
            intake: 1.0,
            mechanical: 1.0,
            combustion_jitter: 1.0,
            reverb: 0.15,
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
/// against the tonal level; a free parameter of this lumped model).
const FLOW_NOISE_GAIN: f64 = 0.6;
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

    /// Jet noise for the radiated outflow signal `radiated` (= ρc·u, Pa): band-passed noise
    /// with amplitude `level`·k·½ρu² (dynamic pressure; only outflow makes a jet).
    pub fn flow_noise(&mut self, radiated: f64, level: f64, dt: f64) -> f64 {
        if level <= 0.0 {
            return 0.0;
        }
        let u = (radiated / RHO_C_EXHAUST).max(0.0);
        let q = 0.5 * RHO_EXHAUST * u * u;
        let n = self.white();
        level * FLOW_NOISE_GAIN * q * self.jet.process(n, 400.0, 6_000.0, dt)
    }

    /// Induction sound from the throttle mass flow (kg/s into the plenum): the plane-wave tone
    /// c·ṁ/A_bore plus hiss ∝ momentum flux ṁ·u / A_bore (u through the current opening).
    /// Using the full bore keeps a nearly closed throttle (sonic, tiny ṁ) from screaming.
    pub fn intake(
        &mut self,
        throttle_flow: f64,
        throttle_area: f64,
        bore_area: f64,
        level: f64,
        dt: f64,
    ) -> f64 {
        if level <= 0.0 || throttle_area <= 0.0 || bore_area <= 0.0 {
            return 0.0;
        }
        let m = throttle_flow.abs();
        let tone = INTAKE_TONE_GAIN * C_AIR * m / bore_area;
        let u = (m / (RHO_AIR * throttle_area)).min(C_AIR);
        let n = self.white();
        let hiss =
            INTAKE_NOISE_GAIN * m * u / bore_area * self.intake.process(n, 800.0, 8_000.0, dt);
        level * (tone + hiss)
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
    fn flow_noise_scales_with_dynamic_pressure_and_needs_outflow() {
        let level = |radiated: f64| {
            let mut t = Texture::new(1);
            let xs: Vec<f64> = (0..40_000)
                .map(|_| t.flow_noise(radiated, 1.0, DT))
                .collect();
            rms(&xs[10_000..])
        };
        let (slow, fast) = (level(3_000.0), level(6_000.0));
        // u doubles → ½ρu² quadruples
        assert!((fast / slow - 4.0).abs() < 0.2, "{slow} → {fast}");
        assert_eq!(level(-6_000.0), 0.0, "inflow makes no jet");
        let mut t = Texture::new(1);
        assert_eq!(t.flow_noise(6_000.0, 0.0, DT), 0.0, "level 0 disables");
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
                        1.0,
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
            .map(|_| t.intake(0.002, 2e-6, 1e-3, 1.0, DT))
            .collect();
        assert!(
            rms(&closed[10_000..]) < 0.1 * level(0.1),
            "{}",
            rms(&closed[10_000..])
        );
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
