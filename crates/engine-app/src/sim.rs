//! The simulation as the app runs it: engine physics at a [`Quality`]'s multiple of the audio rate,
//! the DSP voice, and the data the UI draws (snapshot, pressure trace, scope).

use crate::quality::Quality;
use engine_audio::dsp::{AntiAlias, Voice};
use engine_core::drivetrain::{Vehicle, VehicleDef};
use engine_core::engine::{Controls, CylinderView, Engine, EngineDef};
use engine_core::units::rpm_to_rad_s;

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
    /// The level the physics runs at.
    pub quality: Quality,
    /// Samples the audio device found missing so far: the simulation fell behind real time.
    pub dropouts: u64,
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
    /// How long the clutch takes to close, s: an automated gearbox's; 0 = a driver's release.
    pub clutch_engage_s: f64,
}

/// Just under the output's Nyquist (20.2 kHz at 48 kHz), at the level's physics rate.
fn anti_alias(sample_rate: f64, quality: Quality) -> AntiAlias {
    AntiAlias::new(0.42 * sample_rate, sample_rate * quality.substeps() as f64)
}

fn voice_for(def: &EngineDef, sample_rate: f64) -> Voice {
    let mut v = Voice::new(sample_rate);
    v.reverb_mix = def.sound.reverb;
    v.leveler.amount = def.sound.leveling;
    v
}

/// Samples rendered per pass (a few milliseconds of audio).
const RENDER_CHUNK: usize = 1024;

pub struct Simulation {
    engine: Engine,
    /// The engine as the user defined it; the physics runs `quality.apply(def)`.
    def: EngineDef,
    quality: Quality,
    voice: Voice,
    anti_alias: AntiAlias,
    sample_rate: f64,
    pub controls: Controls,
    pub drive: DriveInput,
    vehicle: Vehicle,
    trace: Vec<f32>,
    trace_deg: usize,
    scope: Vec<f32>,
    scope_pos: usize,
    /// The exhaust net's own thread, when the machine has cores to spare (`None`: it runs
    /// here, the same samples).
    exhaust_thread: Option<crate::exhaust_thread::ExhaustThread>,
    /// Half the ITB runners' own thread, when the machine has more cores still.
    runner_thread: Option<crate::exhaust_thread::RunnerThread>,
    /// One block's physics-step sound, before the anti-alias filter (kept: no allocation).
    steps: Vec<f64>,
}

impl Simulation {
    /// The reference level and the default car (tests and diagnostics).
    #[cfg(test)]
    pub fn new(def: EngineDef, sample_rate: f64, seed: u64) -> Self {
        Self::with_quality(
            def,
            VehicleDef::default(),
            sample_rate,
            seed,
            Quality::Ultra,
        )
    }
    pub fn with_quality(
        def: EngineDef,
        vehicle: VehicleDef,
        sample_rate: f64,
        seed: u64,
        quality: Quality,
    ) -> Self {
        Self {
            vehicle: Vehicle::new(vehicle),
            drive: DriveInput::default(),
            voice: voice_for(&def, sample_rate),
            anti_alias: anti_alias(sample_rate, quality),
            engine: Engine::with_fidelity(
                quality.apply(&def),
                IDLE_START_RPM,
                seed,
                quality.fidelity(),
            ),
            def,
            quality,
            sample_rate,
            controls: Controls::default(),
            trace: vec![0.0; 720],
            trace_deg: 0,
            scope: vec![0.0; SCOPE_LEN],
            scope_pos: 0,
            exhaust_thread: crate::exhaust_thread::ExhaustThread::worthwhile()
                .then(crate::exhaust_thread::ExhaustThread::spawn)
                .and_then(Result::ok),
            runner_thread: crate::exhaust_thread::RunnerThread::worthwhile()
                .then(crate::exhaust_thread::RunnerThread::spawn)
                .and_then(Result::ok),
            steps: Vec::new(),
        }
    }
    /// Turn the output limiter on (live audio) or off (an offline render, scaled afterwards).
    pub fn set_limiter(&mut self, on: bool) {
        self.voice.limit = on;
    }
    /// The clutch is locked: engine and wheels turn together (a scripted driver waits for it).
    pub fn clutch_locked(&self) -> bool {
        self.vehicle.locked()
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
        self.voice.leveler.amount = def.sound.leveling;
        self.def = def.clone();
        let Some(def) = self.try_retune(self.quality.apply(&def)) else {
            return;
        };
        let rpm = self.engine.rpm();
        self.engine = Engine::with_fidelity(
            def,
            rpm,
            self.engine.combustions() ^ 0x5EED,
            self.quality.fidelity(),
        );
    }
    /// The engine as the user defined it (the physics may run a cheaper version of it).
    pub fn def(&self) -> &EngineDef {
        &self.def
    }
    pub fn quality(&self) -> Quality {
        self.quality
    }
    /// Simulate at another level from now on, keeping the engine's speed.
    pub fn set_quality(&mut self, quality: Quality) {
        if quality == self.quality {
            return;
        }
        self.quality = quality;
        self.anti_alias = anti_alias(self.sample_rate, quality);
        let rpm = self.engine.rpm();
        self.engine = Engine::with_fidelity(
            quality.apply(&self.def),
            rpm,
            self.engine.combustions() ^ 0x5EED,
            quality.fidelity(),
        );
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
        // in chunks: the engine builds its exhaust on its first step, so a long render (headless,
        // offline) hands it to the thread from the second chunk on; and the buffer stays small
        for chunk in out.chunks_mut(RENDER_CHUNK) {
            self.render_chunk(chunk);
        }
    }

    fn render_chunk(&mut self, out: &mut [f32]) {
        let substeps = self.quality.substeps();
        let dt = 1.0 / (self.sample_rate * substeps as f64);
        self.vehicle.set_gear(self.drive.gear);
        self.vehicle.set_clutch_engage_s(self.drive.clutch_engage_s);
        self.vehicle
            .set_clutch(if self.drive.clutch_pedal { 0.0 } else { 1.0 });
        let mut c = self.controls;
        // the exhaust on its thread for this chunk, a block of steps at a time
        let detached = match &self.exhaust_thread {
            Some(w) => self.engine.detach_exhaust(|net, block| w.link(net, block)),
            None => false,
        };
        // and half the ITB runners on theirs: the same samples, no delay
        let half = self.engine.cylinders() / 2;
        let runners_away = match &self.runner_thread {
            Some(w) => self.engine.detach_runners(half, |set| w.link(set)),
            None => false,
        };
        self.steps.clear();
        for _ in 0..out.len() {
            for _ in 0..substeps {
                // The vehicle sees the crank speed and answers with the load for the next step.
                let omega = rpm_to_rad_s(self.engine.rpm());
                let load = self.vehicle.step(dt, omega, self.drive.brake);
                c.load_torque_nm = self.controls.load_torque_nm + load.torque_nm;
                c.extra_inertia_kgm2 = self.controls.extra_inertia_kgm2 + load.inertia_kgm2;
                self.steps.push(self.engine.step(dt, &c));
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
        }
        if runners_away
            && let (Some(w), Some(mut link)) =
                (&self.runner_thread, self.engine.end_detached_runners())
            && let Some(set) = w.back(&mut *link)
        {
            self.engine.attach_runners(set);
        }
        // with the exhaust away the first steps returned nothing yet (each step's sound comes
        // `2·block` steps late) and the rest comes now: as many as returned nothing
        let first = if detached {
            let late = self.engine.flush_exhaust(dt, &mut self.steps);
            if let (Some(w), Some(mut link)) = (&self.exhaust_thread, self.engine.end_detached())
                && let Some(net) = w.back(&mut *link)
            {
                self.engine.attach_exhaust(net);
            }
            late
        } else {
            0
        };
        for (s, chunk) in out.iter_mut().zip(self.steps[first..].chunks(substeps)) {
            let mut p = 0.0;
            for &x in chunk {
                p = self.anti_alias.process(x);
            }
            *s = self.voice.process(p);
            self.scope[self.scope_pos] = *s;
            self.scope_pos = (self.scope_pos + 1) % SCOPE_LEN;
        }
    }
    /// Run the exhaust net on its own thread or here: the same samples either way.
    #[cfg(test)]
    pub fn set_exhaust_thread(&mut self, on: bool) {
        // forced either way: the test is that both give the same samples, whatever the cores
        self.exhaust_thread = on
            .then(crate::exhaust_thread::ExhaustThread::spawn)
            .and_then(Result::ok);
        self.runner_thread = on
            .then(crate::exhaust_thread::RunnerThread::spawn)
            .and_then(Result::ok);
    }
    pub fn snapshot(&self) -> Snapshot {
        let e = &self.engine;
        let mut scope = Vec::with_capacity(SCOPE_LEN);
        scope.extend_from_slice(&self.scope[self.scope_pos..]);
        scope.extend_from_slice(&self.scope[..self.scope_pos]);
        Snapshot {
            def: self.def.clone(),
            quality: self.quality,
            dropouts: 0,
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

    /// Share of the signal's energy in engine-cycle orders (k · rpm/120 Hz, up to 10 kHz).
    pub fn harmonic_share(x: &[f32], rpm: f64) -> f64 {
        let f0 = rpm / 120.0;
        let total: f64 = x.iter().map(|s| (*s as f64).powi(2)).sum::<f64>() / x.len() as f64;
        let mut h = 0.0;
        let mut k = 1;
        while k as f64 * f0 < 10_000.0 {
            let w = 2.0 * std::f64::consts::PI * f0 * k as f64 / FS;
            let (mut re, mut im) = (0.0, 0.0);
            for (n, s) in x.iter().enumerate() {
                re += *s as f64 * (w * n as f64).cos();
                im += *s as f64 * (w * n as f64).sin();
            }
            let a = 2.0 * (re * re + im * im).sqrt() / x.len() as f64;
            h += 0.5 * a * a;
            k += 1;
        }
        h / total
    }

    /// Diagnostic (run by hand): a free-running drive in neutral (idle, three blips, a pull to
    /// the limiter, lift-off) rendered through the app's chain, per preset and sound layer, to
    /// `target/sound/<preset>-<mix>.wav` with the throttle trace in `throttle.csv`.
    #[test]
    fn the_exhaust_on_its_thread_gives_the_same_samples() {
        // the net on a worker thread against the net in place: byte-identical sound, through a
        // blip (the controls change between blocks, as from the UI) and several blocks
        for def in [EngineDef::lfa_v10(), EngineDef::inline4()] {
            let render = |thread: bool| {
                let mut sim = Simulation::new(def.clone(), FS, 1);
                sim.set_exhaust_thread(thread);
                let mut out = Vec::new();
                for b in 0..12 {
                    sim.controls.throttle = if (4..7).contains(&b) { 1.0 } else { 0.1 };
                    // one chunk shorter than the exhaust's lag: all of it comes at the flush
                    let mut buf = vec![0.0f32; if b == 2 { 7 } else { 2000 }];
                    sim.render(&mut buf);
                    out.extend_from_slice(&buf);
                }
                out
            };
            let (here, away) = (render(false), render(true));
            assert!(here.iter().any(|x| *x != 0.0));
            assert!(here == away, "{}: the thread changed the sound", def.name);
        }
    }

    /// `cargo test -p engine-app --release drive_render -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn drive_render() {
        use engine_core::sound::SoundDef;
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/sound");
        std::fs::create_dir_all(&dir).unwrap();
        let throttle_at = |t: f64| -> f64 {
            let blip = |t0: f64| (t0..t0 + 0.25).contains(&t);
            if blip(2.0) || blip(3.0) || blip(4.0) || (5.0..7.5).contains(&t) {
                1.0
            } else {
                0.0
            }
        };
        let block = 240;
        for (name, def) in [
            ("inline4", EngineDef::inline4()),
            ("lfa", EngineDef::lfa_v10()),
            ("flatplane", EngineDef::flatplane_v8()),
            ("crossplane", EngineDef::crossplane_v8()),
        ] {
            let quiet = SoundDef {
                flow_noise: 0.0,
                intake: 0.0,
                mechanical: 0.0,
                combustion_jitter: 0.0,
                reverb: 0.0,
                ..def.sound.clone()
            };
            for (mix, snd) in [
                ("all", def.sound.clone()),
                (
                    "physics",
                    SoundDef {
                        leveling: 0.0,
                        ..quiet
                    },
                ),
            ] {
                let mut d = def.clone();
                d.sound = snd;
                let mut sim = Simulation::new(d, FS, 1);
                let mut out = Vec::new();
                let mut rpm = Vec::new();
                let n = (11.0 * FS) as usize / block;
                for b in 0..n {
                    sim.controls.throttle = throttle_at(b as f64 * block as f64 / FS);
                    let mut buf = vec![0.0f32; block];
                    sim.render(&mut buf);
                    out.extend_from_slice(&buf);
                    rpm.push(sim.engine().rpm());
                }
                let path = dir.join(format!("{name}-{mix}.wav"));
                crate::headless::write_wav(&path, &out, FS as u32).unwrap();
                let csv: String = rpm
                    .iter()
                    .enumerate()
                    .map(|(b, r)| {
                        let t = b as f64 * block as f64 / FS;
                        format!("{t:.4},{:.0},{r:.0}\n", throttle_at(t))
                    })
                    .collect();
                std::fs::write(dir.join(format!("{name}-{mix}.csv")), csv).unwrap();
                println!("{}", path.display());
            }
        }
    }

    /// Share of the energy in engine-cycle orders below 500 Hz, 0.5–2 kHz and 2–12 kHz, of all the
    /// energy in orders.
    fn order_bands(x: &[f32], rpm: f64) -> [f64; 3] {
        let f0 = rpm / 120.0;
        let mut bands = [0.0f64; 3];
        let mut k = 1;
        while k as f64 * f0 < 12_000.0 {
            let f = k as f64 * f0;
            let w = 2.0 * std::f64::consts::PI * f / FS;
            let (mut re, mut im) = (0.0, 0.0);
            for (n, s) in x.iter().enumerate() {
                re += *s as f64 * (w * n as f64).cos();
                im += *s as f64 * (w * n as f64).sin();
            }
            bands[usize::from(f >= 500.0) + usize::from(f >= 2000.0)] += re * re + im * im;
            k += 1;
        }
        let t: f64 = bands.iter().sum();
        bands.map(|b| b / t)
    }

    /// `def` with the finite-volume exhaust.
    fn fv_exhaust(mut def: EngineDef) -> EngineDef {
        def.exhaust.model = engine_core::exhaust::ExhaustModel::Fv;
        def
    }

    /// The sound scorecard (run by hand; `scripts/scorecard.sh` compares it with the committed
    /// baseline): objective numbers per preset, as the app plays it, so every change to the sound
    /// is measured, not guessed. Steady points are held at 0.7 × redline (+ 13 rpm, so the
    /// output rate is not a multiple of the firing frequency).
    /// - `wot_dbfs`, `coast_db`: level at full throttle; coast (fuel cut) relative to it. A real
    ///   tailpipe drops by 10–20 dB on overrun.
    /// - `off_order_pct`: share of the full-throttle sound off the engine orders ("air").
    /// - `low/mid/high_pct`: order energy below 500 Hz, 0.5–2 kHz, above 2 kHz.
    /// - `blip_rpm`, `settle_rpm`: a 0.25 s blip from idle (peak), and the speed 3 s later.
    /// - `idle_jitter_rpm`: standard deviation of the idle speed over 2 s.
    ///
    /// `cargo test -p engine-app --release scorecard -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn scorecard() {
        let mut lines = vec![
            "preset,wot_dbfs,coast_db,off_order_pct,low_pct,mid_pct,high_pct,blip_rpm,settle_rpm,idle_jitter_rpm,thump_db".to_string(),
        ];
        for (name, def) in [
            ("single", EngineDef::single()),
            ("inline4", EngineDef::inline4()),
            ("lfa-v10", EngineDef::lfa_v10()),
            ("v8-flatplane", EngineDef::flatplane_v8()),
            ("v8-crossplane", EngineDef::crossplane_v8()),
            // the non-linear 1D exhaust, the model that is to become the default
            ("inline4-fv", fv_exhaust(EngineDef::inline4())),
            ("lfa-v10-fv", fv_exhaust(EngineDef::lfa_v10())),
        ] {
            let rpm = (0.7 * def.redline_rpm).round() + 13.0;
            let steady = |throttle: f64| {
                let mut sim = Simulation::new(def.clone(), FS, 1);
                sim.controls = Controls {
                    throttle,
                    hold_rpm: Some(rpm),
                    ..Controls::default()
                };
                render_secs(&mut sim, 1.0);
                render_secs(&mut sim, 1.0)
            };
            let db = |x: &[f32]| 20.0 * rms(x).max(1e-9).log10();
            let wot = steady(1.0);
            let coast = steady(0.0);
            let off = 100.0 * (1.0 - harmonic_share(&wot, rpm));
            let b = order_bands(&wot, rpm);
            // free running: idle, a blip, the fall back
            let mut sim = Simulation::new(def.clone(), FS, 1);
            render_secs(&mut sim, 4.0);
            let mut idle = Vec::new();
            for _ in 0..200 {
                render_secs(&mut sim, 0.01);
                idle.push(sim.engine().rpm());
            }
            let mean = idle.iter().sum::<f64>() / idle.len() as f64;
            let jitter =
                (idle.iter().map(|r| (r - mean).powi(2)).sum::<f64>() / idle.len() as f64).sqrt();
            sim.controls.throttle = 1.0;
            let mut peak: f64 = 0.0;
            let mut blip = Vec::new();
            for _ in 0..25 {
                blip.extend(render_secs(&mut sim, 0.01));
                peak = peak.max(sim.engine().rpm());
            }
            // the throttle's onset (first 60 ms) against the rev it starts (100–250 ms)
            let top = |x: &[f32]| x.iter().fold(0f32, |m, v| m.max(v.abs())).max(1e-9);
            let ms = |t: f64| (t * FS) as usize;
            let thump = 20.0 * (top(&blip[..ms(0.06)]) / top(&blip[ms(0.1)..ms(0.25)])).log10();
            sim.controls.throttle = 0.0;
            for _ in 0..300 {
                render_secs(&mut sim, 0.01);
                peak = peak.max(sim.engine().rpm());
            }
            lines.push(format!(
                "{name},{:.1},{:.1},{off:.1},{:.0},{:.0},{:.0},{peak:.0},{:.0},{jitter:.1},{thump:.1}",
                db(&wot),
                db(&coast) - db(&wot),
                100.0 * b[0],
                100.0 * b[1],
                100.0 * b[2],
                sim.engine().rpm(),
            ));
        }
        let text = lines.join("\n") + "\n";
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/sound");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("scorecard.csv"), &text).unwrap();
        print!("{text}");
    }

    /// Diagnostic (run by hand, prints): what the listener hears, per sound layer, as level
    /// and the share in engine-cycle orders; the rest is aperiodic ("air").
    /// `cargo test -p engine-app --release air_probe -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn air_probe() {
        use engine_core::sound::SoundDef;
        let quiet = |d: &EngineDef| SoundDef {
            flow_noise: 0.0,
            intake: 0.0,
            mechanical: 0.0,
            combustion_jitter: 0.0,
            reverb: 0.0,
            ..d.sound.clone()
        };
        for (name, def) in [
            ("inline4", EngineDef::inline4()),
            ("lfa", EngineDef::lfa_v10()),
            ("flatplane", EngineDef::flatplane_v8()),
        ] {
            for (label, throttle, rpm) in [
                ("wot", 1.0, 5000.0),
                ("wot", 1.0, 4937.0),
                ("coast", 0.0, 4937.0),
            ] {
                let mixes: [(&str, SoundDef); 6] = [
                    ("all", def.sound.clone()),
                    ("physics", quiet(&def)),
                    (
                        "+flow",
                        SoundDef {
                            flow_noise: def.sound.flow_noise,
                            ..quiet(&def)
                        },
                    ),
                    (
                        "+intake",
                        SoundDef {
                            intake: def.sound.intake,
                            ..quiet(&def)
                        },
                    ),
                    (
                        "+jitter",
                        SoundDef {
                            combustion_jitter: def.sound.combustion_jitter,
                            ..quiet(&def)
                        },
                    ),
                    (
                        "all-lev0",
                        SoundDef {
                            leveling: 0.0,
                            ..def.sound.clone()
                        },
                    ),
                ];
                let mut line = format!("{name:9} {label:5}");
                for (m, snd) in mixes {
                    let mut d = def.clone();
                    d.sound = snd;
                    let mut sim = Simulation::new(d, FS, 1);
                    sim.controls = Controls {
                        throttle,
                        hold_rpm: Some(rpm),
                        ..Controls::default()
                    };
                    render_secs(&mut sim, 1.0);
                    let x = render_secs(&mut sim, 0.5);
                    let db = 20.0 * rms(&x).max(1e-9).log10();
                    // energy split of the cycle orders: < 500 Hz, 0.5–2 kHz, > 2 kHz
                    let f0 = rpm / 120.0;
                    let mut bands = [0.0f64; 3];
                    let mut k = 1;
                    while k as f64 * f0 < 12_000.0 {
                        let f = k as f64 * f0;
                        let w = 2.0 * std::f64::consts::PI * f / FS;
                        let (mut re, mut im) = (0.0, 0.0);
                        for (n, s) in x.iter().enumerate() {
                            re += *s as f64 * (w * n as f64).cos();
                            im += *s as f64 * (w * n as f64).sin();
                        }
                        let e = re * re + im * im;
                        bands[if f < 500.0 {
                            0
                        } else if f < 2000.0 {
                            1
                        } else {
                            2
                        }] += e;
                        k += 1;
                    }
                    let t: f64 = bands.iter().sum();
                    let n = x.len() as f64;
                    let ms = x.iter().map(|s| (*s as f64).powi(2)).sum::<f64>() / n;
                    let periodic = 2.0 * t / (n * n) / ms;
                    line += &format!(
                        " | {m} {db:5.1} dB per {:3.0}% {:2.0}/{:2.0}/{:2.0}%",
                        100.0 * periodic,
                        100.0 * bands[0] / t,
                        100.0 * bands[1] / t,
                        100.0 * bands[2] / t
                    );
                }
                println!("{line}");
            }
        }
    }

    #[test]
    fn a_screaming_lfa_is_engine_orders_not_air() {
        // User report (2026-10-22): at high rpm a background "air" noise covered the engine.
        // Flow noise is texture: at 8000 rpm WOT it must stay well under the firing orders.
        let mut sim = Simulation::new(EngineDef::lfa_v10(), FS, 1);
        sim.controls = Controls {
            throttle: 1.0,
            hold_rpm: Some(8000.0),
            ..Controls::default()
        };
        render_secs(&mut sim, 1.5);
        let x = render_secs(&mut sim, 1.0);
        let share = harmonic_share(&x, 8000.0);
        assert!(share >= 0.85, "harmonic share {share}");
    }

    /// Share of the energy in cycle orders 1–3 (below the firing order of a V8) at `rpm` WOT.
    fn low_order_share(def: EngineDef, rpm: f64) -> f64 {
        let mut sim = Simulation::new(def, FS, 1);
        sim.controls = Controls {
            throttle: 1.0,
            hold_rpm: Some(rpm),
            ..Controls::default()
        };
        render_secs(&mut sim, 1.0);
        let x = render_secs(&mut sim, 1.0);
        let o = orders(&x, rpm);
        let total: f64 = o.iter().map(|a| a * a).sum();
        o[..3].iter().map(|a| a * a).sum::<f64>() / total
    }

    #[test]
    fn a_crossplane_v8_burbles_and_a_flatplane_does_not() {
        // Uneven pulses within each bank (cross plane) put energy below the firing order; a
        // flat plane's banks are even inline-4s, so orders 1–3 stay near zero. Averaged over
        // 2500–4000 rpm: the exhaust's own resonances (can, tail) shape any single speed —
        // at 3000 rpm the firing order of these preset geometries sits in a notch. Measured
        // with the packed muffler: cross 0.013, flat 0.0001.
        let speeds = [2500.0, 3000.0, 3500.0, 4000.0];
        let mean = |def: fn() -> EngineDef| {
            speeds
                .iter()
                .map(|&r| low_order_share(def(), r))
                .sum::<f64>()
                / speeds.len() as f64
        };
        let cross = mean(EngineDef::crossplane_v8);
        let flat = mean(EngineDef::flatplane_v8);
        assert!(cross > 0.005, "cross-plane low orders {cross}");
        assert!(cross > 10.0 * flat, "cross {cross} vs flat {flat}");
    }

    #[test]
    fn the_voice_follows_the_engines_leveling() {
        let mut def = EngineDef::inline4();
        def.sound.leveling = 0.4;
        let mut sim = Simulation::new(def.clone(), FS, 1);
        assert_eq!(sim.voice.leveler.amount, 0.4);
        def.sound.leveling = 0.0;
        sim.replace_engine(def);
        assert_eq!(sim.voice.leveler.amount, 0.0);
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
        let mut sim = Simulation::with_quality(def, vehicle, FS, 1, Quality::Ultra);
        sim.controls.throttle = throttle;
        // Pedal down in neutral, rev up, select the gear, then let the clutch out.
        sim.drive = DriveInput {
            gear: 0,
            clutch_pedal: true,
            brake: 0.0,
            clutch_engage_s: 0.0,
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
        let fv = |mut d: EngineDef| {
            d.exhaust.model = engine_core::exhaust::ExhaustModel::Fv;
            d.name += " (fv)";
            d
        };
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
            fv(EngineDef::inline4()),
            fv(EngineDef::lfa_v10()),
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

/// Sound diagnostics (`cargo test -p engine-app --release sound_report -- --ignored --nocapture`):
/// per preset and operating point, the output level, how much of it is harmonic (engine orders
/// up to 10 kHz) and the level with each texture source muted.
#[cfg(test)]
mod sound_report {
    use super::tests::{FS, harmonic_share, render_secs, rms};
    use super::*;

    fn level(def: &EngineDef, rpm: Option<f64>, throttle: f64) -> (Vec<f32>, f64) {
        let mut sim = Simulation::new(def.clone(), FS, 1);
        sim.controls = Controls {
            throttle,
            hold_rpm: rpm,
            ..Controls::default()
        };
        render_secs(&mut sim, 1.5);
        let x = render_secs(&mut sim, 1.0);
        let r = sim.engine().rpm();
        (x, r)
    }

    #[test]
    #[ignore]
    fn sound_report() {
        let db = |x: f64| 20.0 * x.max(1e-9).log10();
        for def in [EngineDef::inline4(), EngineDef::lfa_v10()] {
            for (what, rpm, throttle) in [
                ("idle", None, 0.0),
                ("3000 part", Some(3000.0), 0.25),
                ("8000 WOT", Some(8000.0), 1.0),
            ] {
                let (x, r) = level(&def, rpm, throttle);
                let mut line = format!(
                    "{:14} {what:10} {r:5.0} rpm  {:6.1} dBFS  harmonic {:4.0}%",
                    def.name,
                    db(rms(&x)),
                    100.0 * harmonic_share(&x, r)
                );
                for (src, f) in [
                    ("-flow", 0usize),
                    ("-intake", 1),
                    ("-mech", 2),
                    ("-jitter", 3),
                ] {
                    let mut d = def.clone();
                    let s = &mut d.sound;
                    *[
                        &mut s.flow_noise,
                        &mut s.intake,
                        &mut s.mechanical,
                        &mut s.combustion_jitter,
                    ][f] = 0.0;
                    let (y, r2) = level(&d, rpm, throttle);
                    line += &format!(
                        "  {src} {:5.1} dB/{:3.0}%",
                        db(rms(&y)),
                        100.0 * harmonic_share(&y, r2)
                    );
                }
                println!("{line}");
            }
        }
    }
}

#[cfg(test)]
mod quality_tests {
    use super::tests::{FS, render_secs, rms};
    use super::*;
    use crate::quality::Quality;
    use engine_core::intake_fv::IntakeModel;

    #[test]
    fn low_quality_runs_cheap_pipes_but_shows_the_users_engine() {
        let mut sim = Simulation::with_quality(
            EngineDef::lfa_v10(),
            VehicleDef::default(),
            FS,
            1,
            Quality::Low,
        );
        assert_eq!(sim.quality(), Quality::Low);
        assert_eq!(sim.engine().def().intake.model, IntakeModel::ZeroD);
        assert_eq!(sim.snapshot().def, EngineDef::lfa_v10());
        // and a new engine is taken at the same level, shown as given
        let mut v8 = EngineDef::crossplane_v8();
        v8.intake.model = IntakeModel::Fv;
        sim.replace_engine(v8.clone());
        assert_eq!(sim.engine().def().intake.model, IntakeModel::ZeroD);
        assert_eq!(sim.snapshot().def, v8);
        let out = render_secs(&mut sim, 0.2);
        assert!(out.iter().all(|s| s.is_finite()) && rms(&out) > 1e-4);
    }

    #[test]
    fn a_new_quality_keeps_the_engine_turning() {
        let mut sim = Simulation::new(EngineDef::lfa_v10(), FS, 1);
        assert_eq!(sim.quality(), Quality::Ultra);
        sim.controls.throttle = 0.3;
        render_secs(&mut sim, 0.5);
        let rpm = sim.engine().rpm();
        sim.set_quality(Quality::Medium);
        assert_eq!(sim.quality(), Quality::Medium);
        assert_eq!(sim.engine().def().intake.model, IntakeModel::Fv);
        assert!(
            (sim.engine().rpm() - rpm).abs() < 1.0,
            "{rpm} → {}",
            sim.engine().rpm()
        );
        let out = render_secs(&mut sim, 0.2);
        assert!(out.iter().all(|s| s.is_finite()) && rms(&out) > 1e-4);
        // the same level again changes nothing
        let rpm = sim.engine().rpm();
        let combustions = sim.engine().combustions();
        sim.set_quality(Quality::Medium);
        assert_eq!(sim.engine().combustions(), combustions);
        assert_eq!(sim.engine().rpm(), rpm);
    }
}
