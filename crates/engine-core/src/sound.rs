//! What the listener hears beyond the exhaust's pulses, as levels of physical sources: the
//! jets' mixing noise, the intake mouth, combustion variability, the room. Every source is a
//! physical model; 1 is its physical level (the earlier ear-tuned layers — a throttle tone and
//! hiss, valvetrain clicks — are gone: they put most of the background "air" in).

use serde::{Deserialize, Serialize};

/// Levels of each texture source; 0 disables it, 1 is the calibrated default.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SoundDef {
    /// Turbulent mixing noise of the jets at the tail pipes ([`crate::jet_noise`]): 1 = the
    /// jet-noise correlation's level (physical), 0 = off. It replaced a heuristic layer that,
    /// with the throttle hiss, buried the engine orders (an inline-4 at full throttle went
    /// from 99 % to 55 % order energy).
    pub flow_noise: f64,
    /// The intake mouth: a monopole of the air the engine draws (1 = physical, 0 = off).
    pub intake: f64,
    /// Valvetrain noise. No physical model yet (valve seating radiated by the block): kept so
    /// older files load; it has no effect.
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
            flow_noise: 1.0,
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
