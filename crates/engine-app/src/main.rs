mod app;
mod controls;
mod headless;
mod runner;
mod sim;
mod view;

use clap::{Parser, ValueEnum};
use controls::{Preset, UiState};
use std::path::PathBuf;
use std::process::ExitCode;

#[derive(Debug, Clone, Copy, ValueEnum)]
enum PresetArg {
    Single,
    Inline4,
    LfaV10,
}

impl From<PresetArg> for Preset {
    fn from(p: PresetArg) -> Self {
        match p {
            PresetArg::Single => Preset::Single,
            PresetArg::Inline4 => Preset::Inline4,
            PresetArg::LfaV10 => Preset::LfaV10,
        }
    }
}

/// Real-time engine sound simulator.
#[derive(Debug, Parser)]
#[command(version)]
struct Cli {
    /// Built-in engine to start with.
    #[arg(long, value_enum, default_value = "single")]
    preset: PresetArg,
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
    /// Close the window after this many seconds (smoke tests).
    #[arg(long)]
    quit_after: Option<f64>,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let preset: Preset = cli.preset.into();
    if cli.headless {
        let (samples, report) = headless::render(preset.def(), cli.seconds, cli.throttle, 48_000.0);
        println!(
            "{}: {} samples, rms {:.4}, peak {:.3}, final {:.0} rpm",
            preset.label(),
            report.samples,
            report.rms,
            report.peak,
            report.final_rpm
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
        preset,
        def: preset.def(),
        ..UiState::default()
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
