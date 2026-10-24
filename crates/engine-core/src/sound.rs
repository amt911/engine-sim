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
    /// The block's radiated combustion noise ([`crate::structure`]): the cylinder pressure
    /// through the measured structure response of engine blocks, 1 = physical, 0 = off.
    pub block: f64,
    /// Valvetrain noise. No physical model yet (valve seating radiated by the block): kept so
    /// older files load; it has no effect.
    pub mechanical: f64,
    /// Cycle-to-cycle combustion variability: 1 = the measured spreads of Heywood §9.4.2 (each
    /// cycle's charge, mixture, residual and turbulence near the plug), 0 = every cycle alike.
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
    /// The street: a rigid ground (asphalt) under the car and the listener returns each tail's
    /// sound from its image below the ground (ISO 9613-2) — the low end adds, the highs comb.
    /// Off = free field.
    pub ground: bool,
    /// Heights above the ground of the tail pipes and of the listener's ear, m.
    pub tail_height_m: f64,
    pub listener_height_m: f64,
}

impl SoundDef {
    /// (tail height, ear height) when the ground reflects, for [`crate::listener::Listener`].
    pub fn ground_heights(&self) -> Option<(f64, f64)> {
        self.ground
            .then_some((self.tail_height_m, self.listener_height_m))
    }
}

impl Default for SoundDef {
    fn default() -> Self {
        Self {
            flow_noise: 1.0,
            intake: 1.0,
            block: 1.0,
            mechanical: 1.0,
            combustion_jitter: 1.0,
            reverb: 0.15,
            leveling: 0.0,
            listener_distance_m: 3.0,
            listener_angle_deg: 30.0,
            ground: true,
            // a road car's tail pipe and a standing adult's ear
            tail_height_m: 0.3,
            listener_height_m: 1.6,
        }
    }
}

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
    fn the_block_round_trips_and_old_files_hear_it() {
        let old: SoundDef = serde_yaml_ng::from_str("reverb: 0.2\n").unwrap();
        assert_eq!(old.block, 1.0);
        let s = SoundDef {
            block: 0.25,
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

    #[test]
    fn the_street_round_trips_and_old_files_stand_on_it() {
        let old: SoundDef = serde_yaml_ng::from_str("reverb: 0.2\n").unwrap();
        assert_eq!(old.ground_heights(), Some((0.3, 1.6)));
        let s = SoundDef {
            ground: false,
            tail_height_m: 0.45,
            listener_height_m: 1.2,
            ..SoundDef::default()
        };
        assert_eq!(s.ground_heights(), None);
        let json: SoundDef = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        let yaml: SoundDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&s).unwrap()).unwrap();
        assert_eq!((json, yaml), (s.clone(), s));
    }
}
