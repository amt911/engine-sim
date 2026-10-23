//! Quality levels, as games have them: how much physics the machine can afford in real time.
//! The engine definition is never changed; a level only decides how finely it is simulated.
//! Measured on the scorecard (`sim.rs`, 2026-10-23): 3× oversampling sounds as 4×, 2× within
//! a few percent; the 1D pipes are the dear part on a ten-cylinder engine.

use engine_core::engine::{EngineDef, Fidelity};
use engine_core::exhaust::ExhaustModel;
use engine_core::fv::Scheme;
use engine_core::intake_fv::IntakeModel;

/// How finely the engine is simulated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Quality {
    /// Physics at 2× the audio rate, 0D intake and waveguide exhaust whatever the engine asks.
    Low,
    /// 2× the audio rate, the engine's own pipe models stepped with the second-order scheme.
    Medium,
    /// 3× the audio rate.
    High,
    /// 4× the audio rate (192 kHz at 48 kHz): the reference.
    Ultra,
}

impl Quality {
    /// Best first.
    pub const ALL: [Quality; 4] = [Quality::Ultra, Quality::High, Quality::Medium, Quality::Low];

    /// Physics steps per audio sample.
    pub fn substeps(self) -> usize {
        match self {
            Quality::Low | Quality::Medium => 2,
            Quality::High => 3,
            Quality::Ultra => 4,
        }
    }

    /// How finely the engine's pipes are stepped.
    pub fn fidelity(self) -> Fidelity {
        match self {
            Quality::Medium => Fidelity {
                pipe_scheme: Scheme::MusclRk2,
            },
            _ => Fidelity::default(),
        }
    }

    /// The definition as this level simulates it.
    pub fn apply(self, def: &EngineDef) -> EngineDef {
        let mut d = def.clone();
        if self == Quality::Low {
            d.intake.model = IntakeModel::ZeroD;
            d.exhaust.model = ExhaustModel::Waveguide;
        }
        d
    }

    pub fn label(self) -> &'static str {
        match self {
            Quality::Low => "Low",
            Quality::Medium => "Medium",
            Quality::High => "High",
            Quality::Ultra => "Ultra",
        }
    }

    /// One level down, if there is one.
    pub fn lower(self) -> Option<Quality> {
        match self {
            Quality::Ultra => Some(Quality::High),
            Quality::High => Some(Quality::Medium),
            Quality::Medium => Some(Quality::Low),
            Quality::Low => None,
        }
    }
}

/// The user's choice: a fixed level, or the best one this machine keeps in real time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QualityChoice {
    Auto,
    Fixed(Quality),
}

impl std::str::FromStr for QualityChoice {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        match s.to_ascii_lowercase().as_str() {
            "auto" => Ok(QualityChoice::Auto),
            "low" => Ok(QualityChoice::Fixed(Quality::Low)),
            "medium" => Ok(QualityChoice::Fixed(Quality::Medium)),
            "high" => Ok(QualityChoice::Fixed(Quality::High)),
            "ultra" => Ok(QualityChoice::Fixed(Quality::Ultra)),
            _ => Err(format!(
                "unknown quality '{s}' (auto, low, medium, high, ultra)"
            )),
        }
    }
}

/// Real-time factor a level must reach to be chosen: the UI and the OS need the rest.
pub const HEADROOM: f64 = 1.5;

/// The best level whose real-time factor (`rtf`: seconds of audio per second of work) reaches
/// [`HEADROOM`], trying the best first; Low if none does.
pub fn pick(mut rtf: impl FnMut(Quality) -> f64) -> Quality {
    Quality::ALL
        .into_iter()
        .find(|q| rtf(*q) >= HEADROOM)
        .unwrap_or(Quality::Low)
}

/// Rendering above this share of real time is falling behind (the audio buffer drains).
const OVERLOAD: f64 = 1.0;
/// Seconds of audio the governor averages its load over.
const WATCH_S: f64 = 1.0;

/// Watches the sim thread at a chosen level: when rendering keeps taking more than
/// 1/`OVERLOAD` of the audio it makes for about two seconds, the level is too dear.
#[derive(Debug, Clone)]
pub struct Governor {
    load: f64,
    rendered_s: f64,
}

impl Governor {
    pub fn new() -> Self {
        Self {
            load: 0.0,
            rendered_s: 0.0,
        }
    }

    /// One block of `audio_s` seconds took `work_s` to render. True when the level should
    /// drop (and the watch starts over).
    pub fn overloaded(&mut self, audio_s: f64, work_s: f64) -> bool {
        // the share of real time spent rendering, smoothed over about a second of audio
        let k = (audio_s / WATCH_S).min(1.0);
        self.load += (work_s / audio_s - self.load) * k;
        self.rendered_s += audio_s;
        if self.load > OVERLOAD && self.rendered_s > 2.0 * WATCH_S {
            *self = Self::new();
            return true;
        }
        false
    }
}

impl Default for Governor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_level_steps_the_physics_at_its_rate() {
        assert_eq!(Quality::Ultra.substeps(), 4);
        assert_eq!(Quality::High.substeps(), 3);
        assert_eq!(Quality::Medium.substeps(), 2);
        assert_eq!(Quality::Low.substeps(), 2);
        // Medium steps the 1D pipes with the cheap scheme; High and Ultra with the reference
        assert_eq!(Quality::Medium.fidelity().pipe_scheme, Scheme::MusclRk2);
        assert_eq!(Quality::High.fidelity(), Fidelity::default());
        assert_eq!(Quality::Ultra.fidelity(), Fidelity::default());
    }

    #[test]
    fn low_trades_the_1d_pipes_for_cheap_ones_and_the_rest_keep_them() {
        let mut def = EngineDef::lfa_v10();
        def.exhaust.model = ExhaustModel::Fv;
        assert_eq!(def.intake.model, IntakeModel::Fv);
        let low = Quality::Low.apply(&def);
        assert_eq!(low.intake.model, IntakeModel::ZeroD);
        assert_eq!(low.exhaust.model, ExhaustModel::Waveguide);
        // nothing else changes
        let mut back = low.clone();
        back.intake.model = IntakeModel::Fv;
        back.exhaust.model = ExhaustModel::Fv;
        assert_eq!(back, def);
        for q in [Quality::Medium, Quality::High, Quality::Ultra] {
            assert_eq!(q.apply(&def), def, "{q:?}");
        }
    }

    #[test]
    fn levels_step_down_to_low() {
        assert_eq!(Quality::Ultra.lower(), Some(Quality::High));
        assert_eq!(Quality::High.lower(), Some(Quality::Medium));
        assert_eq!(Quality::Medium.lower(), Some(Quality::Low));
        assert_eq!(Quality::Low.lower(), None);
    }

    #[test]
    fn the_choice_parses_from_the_command_line() {
        assert_eq!("auto".parse(), Ok(QualityChoice::Auto));
        assert_eq!("low".parse(), Ok(QualityChoice::Fixed(Quality::Low)));
        assert_eq!("Medium".parse(), Ok(QualityChoice::Fixed(Quality::Medium)));
        assert_eq!("high".parse(), Ok(QualityChoice::Fixed(Quality::High)));
        assert_eq!("ULTRA".parse(), Ok(QualityChoice::Fixed(Quality::Ultra)));
        assert!("max".parse::<QualityChoice>().is_err());
    }

    #[test]
    fn auto_picks_the_best_level_with_headroom() {
        // Ultra 1.2×, High 1.6×: High
        let rtf = |q: Quality| match q {
            Quality::Ultra => 1.2,
            Quality::High => 1.6,
            Quality::Medium => 2.4,
            Quality::Low => 5.0,
        };
        assert_eq!(pick(rtf), Quality::High);
        // the first that reaches it ends the search (no level below is measured)
        let mut tried = Vec::new();
        assert_eq!(
            pick(|q| {
                tried.push(q);
                2.0
            }),
            Quality::Ultra
        );
        assert_eq!(tried, [Quality::Ultra]);
        // exactly the headroom is enough
        assert_eq!(
            pick(|q| if q == Quality::Medium { 1.5 } else { 1.0 }),
            Quality::Medium
        );
        // a machine too slow for any: Low
        assert_eq!(pick(|_| 0.3), Quality::Low);
    }

    #[test]
    fn the_governor_drops_after_two_seconds_behind_and_not_on_a_spike() {
        let block = 128.0 / 48_000.0;
        let mut g = Governor::new();
        // comfortably ahead (half the audio time): never
        assert!((0..2_000).all(|_| !g.overloaded(block, 0.5 * block)));
        // one slow block among fast ones: no
        assert!(!g.overloaded(block, 5.0 * block));
        assert!((0..200).all(|_| !g.overloaded(block, 0.5 * block)));
        // just behind (1.1× the audio time) for long: after about two seconds of audio
        let mut g = Governor::new();
        let n = (0..2_000)
            .position(|_| g.overloaded(block, 1.1 * block))
            .unwrap();
        let t = (n + 1) as f64 * block;
        assert!((1.5..3.0).contains(&t), "dropped after {t} s");
        // and it starts over
        assert!(!g.overloaded(block, 1.1 * block));
    }
}
