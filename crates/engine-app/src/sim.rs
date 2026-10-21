//! The simulation as the app runs it: engine physics at `SUBSTEPS` × the audio rate,
//! the DSP voice, and the data the UI draws (snapshot, pressure trace, scope).

use engine_audio::dsp::Voice;
use engine_core::drivetrain::{Vehicle, VehicleDef};
use engine_core::engine::{Controls, CylinderView, Engine, EngineDef};
use engine_core::units::rpm_to_rad_s;

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
    /// Car speed, m/s, and engaged gear (0 = neutral).
    pub speed_m_s: f64,
    pub gear: usize,
    /// Clutch engagement, 0 (pedal down) .. 1.
    pub clutch: f64,
}

/// The driver's pedals and lever, set from the UI.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DriveInput {
    /// 0 = neutral, 1.. = forward gear.
    pub gear: usize,
    /// Clutch pedal pressed (disengaged).
    pub clutch_pedal: bool,
    /// Brake pedal 0..1.
    pub brake: f64,
}

fn voice_for(def: &EngineDef, sample_rate: f64) -> Voice {
    let mut v = Voice::new(sample_rate);
    v.reverb_mix = def.sound.reverb;
    v
}

pub struct Simulation {
    engine: Engine,
    voice: Voice,
    sample_rate: f64,
    pub controls: Controls,
    pub drive: DriveInput,
    vehicle: Vehicle,
    trace: Vec<f32>,
    trace_deg: usize,
    scope: Vec<f32>,
    scope_pos: usize,
}

impl Simulation {
    pub fn new(def: EngineDef, sample_rate: f64, seed: u64) -> Self {
        Self::with_vehicle(def, VehicleDef::default(), sample_rate, seed)
    }
    pub fn with_vehicle(def: EngineDef, vehicle: VehicleDef, sample_rate: f64, seed: u64) -> Self {
        Self {
            vehicle: Vehicle::new(vehicle),
            drive: DriveInput::default(),
            voice: voice_for(&def, sample_rate),
            engine: Engine::new(def, IDLE_START_RPM, seed),
            sample_rate,
            controls: Controls::default(),
            trace: vec![0.0; 720],
            trace_deg: 0,
            scope: vec![0.0; SCOPE_LEN],
            scope_pos: 0,
        }
    }
    #[cfg(test)]
    pub fn vehicle(&self) -> &Vehicle {
        &self.vehicle
    }
    /// Swap the car (it stops, in neutral, so the new gearing cannot jump).
    pub fn replace_vehicle(&mut self, def: VehicleDef) {
        self.vehicle = Vehicle::new(def);
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
        self.voice.reverb_mix = def.sound.reverb;
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
        self.vehicle.set_gear(self.drive.gear);
        self.vehicle
            .set_clutch(if self.drive.clutch_pedal { 0.0 } else { 1.0 });
        let mut c = self.controls;
        for s in out.iter_mut() {
            let mut p = 0.0;
            for _ in 0..SUBSTEPS {
                // The vehicle sees the crank speed and answers with the load for the next step.
                let omega = rpm_to_rad_s(self.engine.rpm());
                let load = self.vehicle.step(dt, omega, self.drive.brake);
                c.load_torque_nm = self.controls.load_torque_nm + load.torque_nm;
                c.extra_inertia_kgm2 = self.controls.extra_inertia_kgm2 + load.inertia_kgm2;
                p = self.engine.step(dt, &c);
            }
            let c0 = self.engine.cylinder(0);
            // Fill every degree passed since the last sample, so none keeps a stale value.
            let cur = (c0.cycle_deg as usize).min(719);
            let mut d = self.trace_deg;
            while d != cur {
                d = (d + 1) % 720;
                self.trace[d] = c0.pressure_pa as f32;
            }
            self.trace_deg = cur;
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
            speed_m_s: self.vehicle.speed_m_s(),
            gear: self.vehicle.gear(),
            clutch: self.vehicle.clutch(),
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

    pub fn rms(x: &[f32]) -> f64 {
        (x.iter().map(|s| (s * s) as f64).sum::<f64>() / x.len() as f64).sqrt()
    }

    /// Amplitude of engine cycle orders 1..=12 (DFT at k·rpm/120 Hz).
    pub fn orders(x: &[f32], rpm: f64) -> Vec<f64> {
        let f0 = rpm / 120.0;
        (1..=12)
            .map(|k| {
                let w = 2.0 * std::f64::consts::PI * f0 * k as f64 / FS;
                let (mut re, mut im) = (0.0, 0.0);
                for (n, s) in x.iter().enumerate() {
                    re += *s as f64 * (w * n as f64).cos();
                    im += *s as f64 * (w * n as f64).sin();
                }
                2.0 * (re * re + im * im).sqrt() / x.len() as f64
            })
            .collect()
    }

    #[test]
    fn firing_orders_dominate_and_output_is_not_saturated() {
        // Even firing: an inline-4 fires at cycle order 4 (and multiples), a V10 at order 10.
        // Odd orders below the firing order must be near zero for the I4 (all cylinders equal).
        for (def, firing) in [(EngineDef::inline4(), 4usize), (EngineDef::lfa_v10(), 10)] {
            let mut sim = Simulation::new(def.clone(), FS, 1);
            sim.controls = Controls {
                throttle: 1.0,
                hold_rpm: Some(3000.0),
                ..Controls::default()
            };
            render_secs(&mut sim, 0.4);
            let x = render_secs(&mut sim, 0.5);
            let o = orders(&x, 3000.0);
            let top = (0..12).max_by(|a, b| o[*a].total_cmp(&o[*b])).unwrap() + 1;
            assert!(
                top % firing == 0,
                "{}: strongest order {top}, orders {o:?}",
                def.name
            );
            assert!(
                o[firing - 1] > 4.0 * o[0],
                "{}: firing order not dominant over order 1: {o:?}",
                def.name
            );
            assert!(
                rms(&x) > 0.005 && rms(&x) < 0.5,
                "{}: rms {}",
                def.name,
                rms(&x)
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
    fn pressure_trace_has_no_stale_bins_at_high_rpm() {
        // Regression: at 9,000 rpm a sample advances ~1.1°, so bins were skipped and kept
        // values from earlier cycles (spikes in the plot). Every bin must be from the last cycle.
        let mut sim = Simulation::new(EngineDef::lfa_v10(), FS, 1);
        sim.controls = Controls {
            throttle: 0.0,
            hold_rpm: Some(1000.0),
            ..Controls::default()
        };
        render_secs(&mut sim, 0.3);
        sim.controls = Controls {
            throttle: 0.0,
            ignition: false,
            hold_rpm: Some(9000.0),
            ..Controls::default()
        };
        render_secs(&mut sim, 0.3);
        let trace = sim.snapshot().pressure_trace;
        // Motored at closed throttle the cylinder never exceeds ~15 bar; leftovers from
        // the fired cycles at 1,000 rpm would.
        let max = trace.iter().cloned().fold(0.0f32, f32::max);
        assert!(max < 15e5, "stale bin: {max}");
        assert!(trace.iter().all(|p| *p > 0.0));
    }

    fn driven(def: EngineDef, vehicle: VehicleDef, gear: usize, throttle: f64) -> Simulation {
        let mut sim = Simulation::with_vehicle(def, vehicle, FS, 1);
        sim.controls.throttle = throttle;
        // Pedal down in neutral, rev up, select the gear, then let the clutch out.
        sim.drive = DriveInput {
            gear: 0,
            clutch_pedal: true,
            brake: 0.0,
        };
        render_secs(&mut sim, 0.5);
        sim.drive.gear = gear;
        render_secs(&mut sim, 0.05);
        sim.drive.clutch_pedal = false;
        sim
    }

    #[test]
    fn launching_in_first_accelerates_the_car_without_stalling() {
        let mut sim = driven(EngineDef::inline4(), VehicleDef::inline4(), 1, 0.6);
        render_secs(&mut sim, 4.0);
        let s = sim.snapshot();
        assert_eq!(s.gear, 1);
        assert!(s.speed_m_s > 3.0, "speed {}", s.speed_m_s);
        assert!(s.rpm > 1000.0, "stalled at {}", s.rpm);
        assert!(sim.vehicle().locked(), "clutch should have locked by now");
        // Locked: car speed = ω·r/G (1st: 3.62·4.1 = 14.842, r = 0.316).
        let omega = s.rpm * std::f64::consts::PI / 30.0;
        assert!((s.speed_m_s - omega * 0.316 / 14.842).abs() < 0.05);
    }

    #[test]
    fn neutral_leaves_the_engine_free_and_the_car_still() {
        let mut sim = driven(EngineDef::inline4(), VehicleDef::inline4(), 0, 1.0);
        render_secs(&mut sim, 0.5);
        let s = sim.snapshot();
        assert_eq!(s.speed_m_s, 0.0);
        assert!(s.rpm > 4000.0, "free-revs: {}", s.rpm);
    }

    #[test]
    fn clutch_down_lets_the_car_coast_while_the_engine_idles() {
        let mut sim = driven(EngineDef::inline4(), VehicleDef::inline4(), 1, 0.6);
        render_secs(&mut sim, 3.0);
        let (v0, rpm0) = (sim.snapshot().speed_m_s, sim.snapshot().rpm);
        sim.controls.throttle = 0.0;
        sim.drive.clutch_pedal = true;
        render_secs(&mut sim, 1.0);
        let s = sim.snapshot();
        assert!(
            s.speed_m_s < v0 && s.speed_m_s > 0.8 * v0,
            "{v0} → {}",
            s.speed_m_s
        );
        assert!(
            s.rpm < 0.7 * rpm0,
            "engine spools down: {rpm0} -> {}",
            s.rpm
        );
    }

    #[test]
    fn braking_stops_the_car() {
        let mut sim = driven(EngineDef::inline4(), VehicleDef::inline4(), 1, 0.6);
        render_secs(&mut sim, 3.0);
        sim.controls.throttle = 0.0;
        sim.drive.clutch_pedal = true;
        sim.drive.brake = 1.0;
        // From ~15 m/s at 9.2 m/s² (full brake + road load): stopped well within 3 s.
        render_secs(&mut sim, 3.0);
        assert_eq!(sim.snapshot().speed_m_s, 0.0);
    }

    #[test]
    fn replace_vehicle_stops_in_neutral() {
        let mut sim = driven(EngineDef::inline4(), VehicleDef::inline4(), 1, 0.6);
        render_secs(&mut sim, 1.0);
        sim.replace_vehicle(VehicleDef::lfa_v10());
        let s = sim.snapshot();
        assert_eq!((s.gear, s.speed_m_s), (0, 0.0));
        assert_eq!(sim.vehicle().def().mass_kg, 1480.0);
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
#[cfg(test)]
mod probe {
    use super::tests::*;
    use super::*;

    /// Dev probe: levels and orders per operating point.
    /// `cargo test --release -p engine-app probe -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn levels() {
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
        ] {
            for (t, rpm) in [(0.0, 3000.0), (1.0, 3000.0), (0.0, 7000.0), (1.0, 7000.0)] {
                let mut sim = Simulation::new(d.clone(), FS, 1);
                sim.controls = Controls {
                    throttle: t,
                    hold_rpm: Some(rpm),
                    ..Controls::default()
                };
                render_secs(&mut sim, 0.4);
                let x = render_secs(&mut sim, 0.5);
                let o: Vec<String> = orders(&x, rpm).iter().map(|a| format!("{a:.3}")).collect();
                println!(
                    "{} t={t} {rpm}: rms {:.3} orders1-12 [{}]",
                    d.name,
                    rms(&x),
                    o.join(" ")
                );
            }
        }
    }
}
