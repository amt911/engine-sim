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
    /// The exhaust valves' jets ([`crate::jet_noise::valve_jet_level`]): the turbulent mixing
    /// noise of each blowdown through the valve's curtain, sent down the pipes with the pulse.
    /// 1 = physical, 0 = off.
    pub valve_jet: f64,
    /// Flow noise at the exhaust's abrupt area expansions ([`crate::jet_noise::DuctJet`]):
    /// vortex shedding and shear-layer mixing where the flow separates, W = K·ρ·A·U⁴/c (Davies &
    /// Holland 1999 §4) at each site's own instantaneous flow. 1 = that law at the one
    /// calibrated K, 0 = off.
    pub duct_flow: f64,
    /// Main bearings struck across their oil clearance at each load reversal, heard through
    /// the block ([`crate::mechanical`]): 1 = the measured petrol engine's mechanical noise
    /// (Lalor et al. 1980, shared with `piston_slap`), 0 = none.
    pub main_bearings: f64,
    /// Pistons slapping the liner at each side-thrust reversal ([`crate::mechanical`]): 1 =
    /// physical, 0 = none.
    pub piston_slap: f64,
    /// The block's radiated combustion noise ([`crate::structure`]): the cylinder pressure
    /// through the measured structure response of engine blocks, 1 = physical, 0 = off.
    pub block: f64,
    /// Valvetrain noise: each valve striking its seat shakes the block (Zheng et al. 2020),
    /// heard through its structure response. 1 = physical, 0 = none.
    pub mechanical: f64,
    /// Cycle-to-cycle combustion variability: 1 = the measured spreads of Heywood §9.4.2 (each
    /// cycle's charge, mixture, residual and turbulence near the plug), 0 = every cycle alike.
    pub combustion_jitter: f64,
    /// The chamber's gas ringing after a diesel lights ([`crate::combustion::RING_MODES`]): 1 =
    /// its estimated level, 0 = off. No effect on a spark engine.
    pub chamber_ring: f64,
    /// How much the block answers each cylinder with its own voice
    /// ([`crate::structure::BlockModes`]): 0 = all alike, 1 = by its wall modes.
    pub block_modes: f64,
    /// A turbo's blow-off valve venting on a lift-off ([`crate::turbo`]): the jet of the air it
    /// lets out. 1 = physical, 0 = off.
    pub blow_off: f64,
    /// A turbo's compressor tone ([`crate::turbo::CompressorTone`]). 1 = its estimated level,
    /// 0 = off.
    pub compressor_whine: f64,
    /// Wet level of a room / car-body reverb applied in the voice; 0 (outdoors) by default.
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
    /// Off by default: the image is coherent only for a still source and a still ear, and combs
    /// the highs 20 dB deep (LFA, 6000 rpm WOT, 2–7.5 kHz) where recordings of real cars ripple
    /// 4.5 dB (60 steady segments, FINDINGS 2026-11-06); the user heard it as "enlatado" and
    /// "ecos raros". Its cepstrum peaks 43 dB over the median at the image's 0.82 ms (inline-4),
    /// 20 dB without it, as real recordings (16–25 dB, the LFA's four exterior clips).
    /// Off = free field.
    pub ground: bool,
    /// Heights above the ground of the tail pipes and of the listener's ear, m.
    pub tail_height_m: f64,
    pub listener_height_m: f64,
    /// A street around the car and the listener: the distance between its façades, m (the car
    /// in the middle). 0 = open ground, nothing but the ground reflects.
    pub street_width_m: f64,
    /// Share of the sound pressure each façade returns (ISO 9613-2, Table 4: 0.8 for building
    /// walls with windows, 1 for flat hard walls).
    pub wall_reflection: f64,
}

impl SoundDef {
    /// (tail height, ear height) when the ground reflects, for [`crate::listener::Listener`].
    /// The street the listener stands in, if any ([`crate::listener::Street`]).
    pub fn street(&self) -> Option<crate::listener::Street> {
        (self.street_width_m > 0.0).then_some(crate::listener::Street {
            width_m: self.street_width_m,
            reflection: self.wall_reflection.clamp(0.0, 1.0),
        })
    }

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
            valve_jet: 1.0,
            duct_flow: 1.0,
            main_bearings: 1.0,
            piston_slap: 1.0,
            block: 1.0,
            mechanical: 1.0,
            chamber_ring: 1.0,
            block_modes: 0.0,
            combustion_jitter: 1.0,
            blow_off: 1.0,
            compressor_whine: 1.0,
            reverb: 0.0,
            leveling: 0.0,
            listener_distance_m: 3.0,
            listener_angle_deg: 30.0,
            ground: false,
            // a road car's tail pipe and a standing adult's ear
            tail_height_m: 0.3,
            listener_height_m: 1.6,
            street_width_m: 0.0,
            wall_reflection: 0.8,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_street_round_trips_and_old_files_stand_in_the_open() {
        let d = SoundDef {
            street_width_m: 12.0,
            wall_reflection: 0.9,
            ..SoundDef::default()
        };
        let j: SoundDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
        let y: SoundDef = serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&d).unwrap()).unwrap();
        assert_eq!(j, d);
        assert_eq!(y, d);
        assert_eq!(
            d.street(),
            Some(crate::listener::Street {
                width_m: 12.0,
                reflection: 0.9
            })
        );
        let old: SoundDef = serde_yaml_ng::from_str("reverb: 0.0\n").unwrap();
        assert_eq!(old.street(), None);
    }

    #[test]
    fn an_engine_is_heard_outdoors_without_a_room() {
        // the listener stands outside (ground reflection only): a room's reverberation has no
        // place in the default, it was heard as an echo (user, 2026-10-24); a file can add one
        assert_eq!(SoundDef::default().reverb, 0.0);
    }

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
    fn the_valve_jets_round_trip_and_old_files_hear_them() {
        let old: SoundDef = serde_yaml_ng::from_str("reverb: 0.2\n").unwrap();
        assert_eq!(old.valve_jet, 1.0);
        let s = SoundDef {
            valve_jet: 0.5,
            ..SoundDef::default()
        };
        let json: SoundDef = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        let yaml: SoundDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&s).unwrap()).unwrap();
        assert_eq!((json, yaml), (s.clone(), s));
    }

    #[test]
    fn the_duct_flow_round_trips_and_old_files_hear_it() {
        let old: SoundDef = serde_yaml_ng::from_str("reverb: 0.2\n").unwrap();
        assert_eq!(old.duct_flow, 1.0);
        let s = SoundDef {
            duct_flow: 0.5,
            ..SoundDef::default()
        };
        let json: SoundDef = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        let yaml: SoundDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&s).unwrap()).unwrap();
        assert_eq!((json, yaml), (s.clone(), s));
    }

    #[test]
    fn the_impacts_round_trip_and_old_files_hear_them() {
        let old: SoundDef = serde_yaml_ng::from_str("reverb: 0.2\n").unwrap();
        assert_eq!((old.main_bearings, old.piston_slap), (1.0, 1.0));
        let s = SoundDef {
            main_bearings: 0.5,
            piston_slap: 0.25,
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
            block_modes: 0.5,
            ..SoundDef::default()
        };
        assert_eq!(old.block_modes, 0.0);
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
    fn the_street_round_trips_and_old_files_stand_in_the_free_field() {
        // the rigid ground's image combs the highs 20 dB deep at a standing listener; recordings
        // of real cars ripple 4.5 dB (FINDINGS 2026-11-06): off unless a file asks for it
        let old: SoundDef = serde_yaml_ng::from_str("reverb: 0.2\n").unwrap();
        assert_eq!(old.ground_heights(), None);
        let s = SoundDef {
            ground: true,
            tail_height_m: 0.45,
            listener_height_m: 1.2,
            ..SoundDef::default()
        };
        assert_eq!(s.ground_heights(), Some((0.45, 1.2)));
        let json: SoundDef = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        let yaml: SoundDef =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&s).unwrap()).unwrap();
        assert_eq!((json, yaml), (s.clone(), s));
    }
}
