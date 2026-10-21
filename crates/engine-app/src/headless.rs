//! Headless rendering: run the simulation offline and write/check a WAV.
//! Used by `--headless`, the smoke script and PR verification.

use crate::sim::Simulation;
use engine_core::engine::{Controls, EngineDef};
use std::path::Path;

#[derive(Debug, Clone, PartialEq)]
pub struct RenderReport {
    pub samples: usize,
    pub rms: f64,
    pub peak: f64,
    pub all_finite: bool,
    pub final_rpm: f64,
}

impl RenderReport {
    /// The audio is usable: finite, in range, audible, and the engine is running.
    pub fn check(&self) -> Result<(), String> {
        if !self.all_finite {
            return Err("audio contains NaN/inf".into());
        }
        if self.peak > 1.0 {
            return Err(format!("peak {} out of [-1, 1]", self.peak));
        }
        if self.rms < 0.003 {
            return Err(format!("silent: rms {:.5}", self.rms));
        }
        if self.final_rpm < 300.0 {
            return Err(format!("engine stalled: {:.0} rpm", self.final_rpm));
        }
        Ok(())
    }
}

pub fn render(
    def: EngineDef,
    seconds: f64,
    throttle: f64,
    sample_rate: f64,
) -> (Vec<f32>, RenderReport) {
    let mut sim = Simulation::new(def, sample_rate, 1);
    sim.controls = Controls {
        throttle,
        ..Controls::default()
    };
    let mut samples = vec![0.0f32; (seconds * sample_rate) as usize];
    sim.render(&mut samples);
    let n = samples.len().max(1) as f64;
    let report = RenderReport {
        samples: samples.len(),
        rms: (samples.iter().map(|s| (*s as f64).powi(2)).sum::<f64>() / n).sqrt(),
        peak: samples.iter().fold(0.0f64, |m, s| m.max(s.abs() as f64)),
        all_finite: samples.iter().all(|s| s.is_finite()),
        final_rpm: sim.engine().rpm(),
    };
    (samples, report)
}

pub fn write_wav(path: &Path, samples: &[f32], sample_rate: u32) -> Result<(), hound::Error> {
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 32,
        sample_format: hound::SampleFormat::Float,
    };
    let mut w = hound::WavWriter::create(path, spec)?;
    for s in samples {
        w.write_sample(*s)?;
    }
    w.finalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn report(rms: f64, peak: f64, all_finite: bool, final_rpm: f64) -> RenderReport {
        RenderReport {
            samples: 100,
            rms,
            peak,
            all_finite,
            final_rpm,
        }
    }

    #[test]
    fn check_accepts_good_and_rejects_each_failure() {
        assert!(report(0.1, 0.8, true, 900.0).check().is_ok());
        assert!(report(0.0001, 0.8, true, 900.0).check().is_err(), "silent");
        assert!(
            report(0.1, 1.5, true, 900.0).check().is_err(),
            "clipping out of range"
        );
        assert!(report(0.1, 0.8, false, 900.0).check().is_err(), "NaN");
        assert!(report(0.1, 0.8, true, 50.0).check().is_err(), "stalled");
    }

    #[test]
    fn render_single_passes_check() {
        let (samples, r) = render(EngineDef::single(), 1.0, 0.2, 48_000.0);
        assert_eq!(samples.len(), 48_000);
        assert_eq!(r.samples, 48_000);
        r.check().unwrap();
    }

    #[test]
    fn wav_round_trip() {
        let dir = std::env::temp_dir().join(format!("engine-sim-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.wav");
        write_wav(&path, &[0.0, 0.5, -0.5, 1.0], 48_000).unwrap();
        let mut rd = hound::WavReader::open(&path).unwrap();
        assert_eq!(rd.spec().sample_rate, 48_000);
        let back: Vec<f32> = rd.samples::<f32>().map(|s| s.unwrap()).collect();
        assert_eq!(back, vec![0.0, 0.5, -0.5, 1.0]);
        std::fs::remove_dir_all(dir).unwrap();
    }
}
