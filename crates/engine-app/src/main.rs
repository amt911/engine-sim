mod app;
mod controls;
mod dyno;
mod editor;
mod files;
mod headless;
mod quality;
mod runner;
mod sim;
mod view;

use clap::Parser;
use controls::{Preset, UiState};
use std::path::PathBuf;
use std::process::ExitCode;

/// Real-time engine sound simulator.
#[derive(Debug, Parser)]
#[command(version)]
struct Cli {
    /// Built-in engine to start with.
    #[arg(long, default_value = "single", value_parser = clap::builder::PossibleValuesParser::new(Preset::ALL.map(Preset::slug)))]
    preset: String,
    /// Engine file to load (.yaml/.yml/.json); overrides --preset.
    #[arg(long)]
    engine: Option<PathBuf>,
    /// Write the selected engine to this file (.yaml/.json) and exit.
    #[arg(long)]
    save: Option<PathBuf>,
    /// Run a wide-open-throttle dyno sweep and print `rpm,torque_nm,power_hp` as CSV.
    #[arg(long)]
    dyno: bool,
    /// Render offline instead of opening the window.
    #[arg(long)]
    headless: bool,
    /// Seconds to render in headless mode.
    #[arg(long, default_value_t = 3.0)]
    seconds: f64,
    /// Throttle (0..1) in headless mode.
    #[arg(long, default_value_t = 0.2)]
    throttle: f64,
    /// Write the headless render to this WAV file.
    #[arg(long)]
    wav: Option<PathBuf>,
    /// Fail (exit 1) if the headless render is silent, out of range, NaN or stalled.
    #[arg(long)]
    check: bool,
    /// Simulation quality: auto (the best level this machine keeps in real time), low, medium,
    /// high or ultra. Headless renders offline: auto means ultra there.
    #[arg(long, default_value = "auto")]
    quality: quality::QualityChoice,
    /// Close the window after this many seconds (smoke tests).
    #[arg(long)]
    quit_after: Option<f64>,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let preset = Preset::from_slug(&cli.preset).unwrap_or(Preset::Single);
    let def = match &cli.engine {
        Some(path) => match files::load_engine(path) {
            Ok(d) => d,
            Err(e) => {
                eprintln!("{e}");
                return ExitCode::FAILURE;
            }
        },
        None => preset.def(),
    };
    if let Some(path) = &cli.save {
        return match files::save_engine(path, &def) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("{e}");
                ExitCode::FAILURE
            }
        };
    }
    if cli.dyno {
        let points = dyno::sweep(&def);
        print!("{}", dyno::to_csv(&points));
        if let Some((t, p)) = dyno::peaks(&points) {
            eprintln!(
                "{}: peak {:.0} N·m at {:.0} rpm, peak {:.0} hp at {:.0} rpm",
                def.name,
                t.torque_nm,
                t.rpm,
                p.power_hp(),
                p.rpm
            );
        }
        return ExitCode::SUCCESS;
    }
    if cli.headless {
        let name = def.name.clone();
        let level = match cli.quality {
            quality::QualityChoice::Fixed(q) => q,
            quality::QualityChoice::Auto => quality::Quality::Ultra,
        };
        let (samples, report) = headless::render(def, cli.seconds, cli.throttle, 48_000.0, level);
        println!(
            "{name}: {} samples, rms {:.4}, peak {:.3}, final {:.0} rpm",
            report.samples, report.rms, report.peak, report.final_rpm
        );
        if let Some(path) = &cli.wav
            && let Err(e) = headless::write_wav(path, &samples, 48_000)
        {
            eprintln!("cannot write {}: {e}", path.display());
            return ExitCode::FAILURE;
        }
        if cli.check
            && let Err(e) = report.check()
        {
            eprintln!("check failed: {e}");
            return ExitCode::FAILURE;
        }
        return ExitCode::SUCCESS;
    }
    let state = UiState {
        preset: cli.engine.is_none().then_some(preset),
        quality: cli.quality,
        ..UiState::with_engine(def)
    };
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 760.0])
            .with_title("engine-sim"),
        ..Default::default()
    };
    let quit_after = cli.quit_after;
    match eframe::run_native(
        "engine-sim",
        options,
        Box::new(move |cc| Ok(Box::new(app::EngineApp::new(cc, state, quit_after)))),
    ) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("window error: {e}");
            ExitCode::FAILURE
        }
    }
}
