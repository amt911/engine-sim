//! The simulation as the app runs it: engine physics at `SUBSTEPS` × the audio rate,
//! the DSP voice, and the data the UI draws (snapshot, pressure trace, scope).

use engine_audio::dsp::Voice;
use engine_core::engine::{Controls, CylinderView, Engine, EngineDef};

/// Physics steps per audio sample (engine-sim uses 8 fluid substeps at 10 kHz; we run
/// at the audio rate × 4, e.g. 192 kHz at 48 kHz output).
pub const SUBSTEPS: usize = 4;
pub const SCOPE_LEN: usize = 2048;
pub const IDLE_START_RPM: f64 = 900.0;

/// What the UI needs to draw one frame.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub def: EngineDef,
    pub rpm: f64,
    pub torque_nm: f64,
    pub power_w: f64,
    pub plenum_pa: f64,
    pub crank_deg: f64,
    pub cylinders: Vec<CylinderView>,
    /// Cylinder 1 pressure (Pa) for each degree of the 720° cycle, last value seen.
    pub pressure_trace: Vec<f32>,
    /// Last `SCOPE_LEN` output samples, oldest first.
    pub scope: Vec<f32>,
}

pub struct Simulation {
    engine: Engine,
    voice: Voice,
    sample_rate: f64,
    pub controls: Controls,
    trace: Vec<f32>,
    scope: Vec<f32>,
    scope_pos: usize,
}

impl Simulation {
    pub fn new(def: EngineDef, sample_rate: f64, seed: u64) -> Self {
        Self {
            voice: Voice::new(sample_rate),
            engine: Engine::new(def, IDLE_START_RPM, seed),
            sample_rate,
            controls: Controls::default(),
            trace: vec![0.0; 720],
            scope: vec![0.0; SCOPE_LEN],
            scope_pos: 0,
        }
    }
    pub fn engine(&self) -> &Engine {
        &self.engine
    }
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }
    /// Swap the engine definition, keeping the current speed. Tuning-only changes are
    /// applied in place so the engine keeps running without a click.
    pub fn replace_engine(&mut self, def: EngineDef) {
        let Some(def) = self.try_retune(def) else {
            return;
        };
        let rpm = self.engine.rpm();
        self.engine = Engine::new(def, rpm, self.engine.combustions() ^ 0x5EED);
    }
    fn try_retune(&mut self, def: EngineDef) -> Option<EngineDef> {
        if self.engine.retune(def.clone()) {
            None
        } else {
            Some(def)
        }
    }

    /// Fill `out` with mono samples.
    pub fn render(&mut self, out: &mut [f32]) {
        let dt = 1.0 / (self.sample_rate * SUBSTEPS as f64);
        for s in out.iter_mut() {
            let mut p = 0.0;
            for _ in 0..SUBSTEPS {
                p = self.engine.step(dt, &self.controls);
            }
            let c0 = self.engine.cylinder(0);
            self.trace[(c0.cycle_deg as usize).min(719)] = c0.pressure_pa as f32;
            *s = self.voice.process(p);
            self.scope[self.scope_pos] = *s;
            self.scope_pos = (self.scope_pos + 1) % SCOPE_LEN;
        }
    }
    pub fn snapshot(&self) -> Snapshot {
        let e = &self.engine;
        let mut scope = Vec::with_capacity(SCOPE_LEN);
        scope.extend_from_slice(&self.scope[self.scope_pos..]);
        scope.extend_from_slice(&self.scope[..self.scope_pos]);
        Snapshot {
            def: e.def().clone(),
            rpm: e.rpm(),
            torque_nm: e.torque_nm(),
            power_w: e.power_w(),
            plenum_pa: e.plenum_pressure_pa(),
            crank_deg: e.crank_deg(),
            cylinders: (0..e.def().cylinders).map(|i| e.cylinder(i)).collect(),
            pressure_trace: self.trace.clone(),
            scope,
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub const FS: f64 = 48_000.0;

    pub fn render_secs(sim: &mut Simulation, secs: f64) -> Vec<f32> {
        let mut out = vec![0.0; (secs * sim.sample_rate()) as usize];
        sim.render(&mut out);
        out
    }

    /// Lag (samples) of the strongest autocorrelation peak within [min, max].
    fn dominant_period(x: &[f32], min: usize, max: usize) -> usize {
        let score = |lag: usize| -> f64 {
            x.iter()
                .zip(&x[lag..])
                .map(|(a, b)| (*a as f64) * (*b as f64))
                .sum()
        };
        (min..=max)
            .max_by(|a, b| score(*a).total_cmp(&score(*b)))
            .unwrap()
    }

    #[test]
    fn render_fills_buffer_with_finite_audible_samples() {
        let mut sim = Simulation::new(EngineDef::single(), FS, 1);
        let out = render_secs(&mut sim, 1.0);
        assert_eq!(out.len(), 48_000);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.0));
        let rms = (out.iter().map(|s| (s * s) as f64).sum::<f64>() / out.len() as f64).sqrt();
        assert!(rms > 0.005, "rms {rms}");
    }

    #[test]
    fn sound_repeats_at_the_firing_frequency() {
        // Single at 3000 rpm fires 3000/120 = 25 Hz → period 1920 samples at 48 kHz.
        let mut sim = Simulation::new(EngineDef::single(), FS, 1);
        sim.controls = Controls {
            throttle: 0.5,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        render_secs(&mut sim, 0.5);
        let out = render_secs(&mut sim, 0.5);
        let p = dominant_period(&out, 1000, 3000);
        assert!((1900..=1940).contains(&p), "period {p}");
        // Inline-4 at 3000 rpm: 4 × 25 = 100 Hz → 480 samples.
        let mut sim = Simulation::new(EngineDef::inline4(), FS, 1);
        sim.controls = Controls {
            throttle: 0.5,
            hold_rpm: Some(3000.0),
            ..Controls::default()
        };
        render_secs(&mut sim, 0.5);
        let out = render_secs(&mut sim, 0.5);
        let p = dominant_period(&out, 300, 700);
        assert!((470..=490).contains(&p), "period {p}");
    }

    fn rms(x: &[f32]) -> f64 {
        (x.iter().map(|s| (s * s) as f64).sum::<f64>() / x.len() as f64).sqrt()
    }

    #[test]
    fn load_makes_it_louder_without_saturating() {
        // Regression: with the plain 0D runner pressure as source, high-rpm WOT was barely
        // louder than idle; with the valve flow as source, coasting was louder than WOT.
        // Load must raise the level (Caviezel 2026: load matters as much as rpm).
        // Known limit: a V10 at WOT sums ten smooth 0D pulses into a nearly steady flow, so
        // it is not checked here — sharp pulses need the waveguide exhaust (next milestone).
        for def in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
        ] {
            let level = |throttle: f64| {
                let mut sim = Simulation::new(def.clone(), FS, 1);
                sim.controls = Controls {
                    throttle,
                    hold_rpm: Some(6000.0),
                    ..Controls::default()
                };
                render_secs(&mut sim, 0.4);
                rms(&render_secs(&mut sim, 0.4))
            };
            let (coast, wot) = (level(0.0), level(1.0));
            if def.cylinders <= 4 {
                assert!(wot > 1.5 * coast, "{}: wot {wot} coast {coast}", def.name);
            }
            assert!(
                wot > 0.02 && wot < 0.5 && coast < 0.5,
                "{}: wot {wot} coast {coast}",
                def.name
            );
        }
    }

    #[test]
    fn snapshot_reflects_engine() {
        let mut sim = Simulation::new(EngineDef::inline4(), FS, 1);
        render_secs(&mut sim, 0.2);
        let s = sim.snapshot();
        assert_eq!(s.cylinders.len(), 4);
        assert_eq!(s.pressure_trace.len(), 720);
        assert_eq!(s.scope.len(), SCOPE_LEN);
        assert!(
            s.pressure_trace.iter().any(|p| *p > 2e5),
            "compression visible in trace"
        );
        assert!(s.rpm > 0.0);
    }

    #[test]
    fn replace_engine_keeps_speed() {
        let mut sim = Simulation::new(EngineDef::single(), FS, 1);
        sim.controls.hold_rpm = Some(2500.0);
        render_secs(&mut sim, 0.1);
        sim.controls.hold_rpm = None;
        sim.replace_engine(EngineDef::inline4());
        assert_eq!(sim.engine().def().cylinders, 4);
        assert!((sim.engine().rpm() - 2500.0).abs() < 1.0);
    }
}
