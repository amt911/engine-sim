//! The eframe application shell: wires the controls, the sim thread and the views.

use crate::controls::{UiState, controls_panel};
use crate::dyno::{self, DynoPoint};
use crate::editor::editor_panel;
use crate::files;
use crate::runner::{Command, SimHandle};
use crate::sim::{DriveInput, Snapshot};
use crate::view;
use eframe::egui;
use engine_core::engine::Controls;
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, TryRecvError, channel};
use std::time::Instant;

/// Result of a native file dialog, which runs on its own thread so the frame never blocks.
enum FileJob {
    Load(Option<PathBuf>),
    /// Path chosen, and the engine as it was when Save was clicked.
    Save(Option<PathBuf>, Box<engine_core::engine::EngineDef>),
}

enum JobPoll {
    Pending,
    Done(FileJob),
    /// The dialog thread ended without answering (e.g. no file-dialog portal).
    Lost,
}

fn poll_job(rx: &Receiver<FileJob>) -> JobPoll {
    match rx.try_recv() {
        Ok(job) => JobPoll::Done(job),
        Err(TryRecvError::Empty) => JobPoll::Pending,
        Err(TryRecvError::Disconnected) => JobPoll::Lost,
    }
}

pub struct EngineApp {
    state: UiState,
    sim: SimHandle,
    sent: Option<Controls>,
    sent_drive: Option<DriveInput>,
    dyno_job: Option<Receiver<Vec<DynoPoint>>>,
    quit_after: Option<f64>,
    started: Instant,
    file_job: Option<Receiver<FileJob>>,
}

impl EngineApp {
    pub fn new(cc: &eframe::CreationContext<'_>, state: UiState, quit_after: Option<f64>) -> Self {
        let sim = SimHandle::spawn(
            state.def.clone(),
            state.vehicle.clone(),
            state.quality,
            Some(cc.egui_ctx.clone()),
        );
        Self {
            state,
            sim,
            sent: None,
            sent_drive: None,
            dyno_job: None,
            quit_after,
            started: Instant::now(),
            file_job: None,
        }
    }
}

impl EngineApp {
    /// Start a dyno sweep on a background thread (it takes seconds), and collect a finished one.
    fn poll_dyno(&mut self, ctx: &egui::Context) {
        if std::mem::take(&mut self.state.dyno_request) && self.dyno_job.is_none() {
            let (tx, rx) = channel();
            let def = self.state.def.clone();
            let ctx = ctx.clone();
            std::thread::spawn(move || {
                let _ = tx.send(dyno::sweep(&def));
                ctx.request_repaint();
            });
            self.dyno_job = Some(rx);
        }
        if let Some(rx) = &self.dyno_job {
            match rx.try_recv() {
                Ok(curve) => {
                    self.state.dyno_curve = curve;
                    self.dyno_job = None;
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => self.dyno_job = None,
            }
        }
        self.state.dyno_running = self.dyno_job.is_some();
    }

    fn dialog(&mut self, save: bool, ctx: &egui::Context) {
        let (tx, rx) = channel();
        let ctx = ctx.clone();
        let name = format!(
            "{}.yaml",
            self.state.def.name.replace(' ', "-").to_lowercase()
        );
        let snapshot = Box::new(self.state.engine_file());
        std::thread::spawn(move || {
            let d = rfd::FileDialog::new().add_filter("Engine", &["yaml", "yml", "json"]);
            let job = if save {
                FileJob::Save(d.set_file_name(name).save_file(), snapshot)
            } else {
                FileJob::Load(d.pick_file())
            };
            let _ = tx.send(job);
            ctx.request_repaint();
        });
        self.file_job = Some(rx);
    }

    fn load(&mut self, path: &std::path::Path) {
        match files::load_engine(path) {
            Ok(def) => {
                self.state.load_engine(def);
                self.state.file_msg = Some(format!("Loaded {}", path.display()));
            }
            Err(e) => self.state.file_msg = Some(e),
        }
    }

    fn poll_files(&mut self, ui: &egui::Ui) {
        let poll = self.file_job.as_ref().map_or(JobPoll::Pending, poll_job);
        match poll {
            JobPoll::Pending => {}
            JobPoll::Lost => {
                self.file_job = None;
                self.state.file_msg = Some("The file dialog could not be opened.".into());
            }
            JobPoll::Done(job) => {
                self.file_job = None;
                match job {
                    FileJob::Load(Some(p)) => self.load(&p),
                    FileJob::Save(Some(p), def) => {
                        self.state.file_msg = Some(match files::save_engine(&p, &def) {
                            Ok(()) => format!("Saved {}", p.display()),
                            Err(e) => e,
                        });
                    }
                    FileJob::Load(None) | FileJob::Save(None, _) => {}
                }
            }
        }
        let dropped: Vec<PathBuf> = ui.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .map(|f| f.path().to_path_buf())
                .filter(|p| !p.as_os_str().is_empty())
                .collect()
        });
        if let Some(p) = dropped.first() {
            self.load(p);
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileAction {
    Load,
    Save,
}

fn file_row(ui: &mut egui::Ui, state: &UiState, busy: bool) -> Option<FileAction> {
    let mut action = None;
    ui.horizontal(|ui| {
        if ui.add_enabled(!busy, egui::Button::new("Load…")).clicked() {
            action = Some(FileAction::Load);
        }
        if ui
            .add_enabled(!busy && state.errors.is_empty(), egui::Button::new("Save…"))
            .clicked()
        {
            action = Some(FileAction::Save);
        }
    });
    ui.small("or drop a .yaml / .json file on the window");
    if let Some(m) = &state.file_msg {
        ui.small(m.as_str());
    }
    action
}

/// The whole window, from UI state and a sim snapshot. Shared by the app and the
/// off-screen renderer (`render_screens` test), so what is checked is what ships.
pub fn layout(
    ui: &mut egui::Ui,
    state: &mut UiState,
    snap: &Snapshot,
    audio: &Result<u32, String>,
    file_busy: bool,
) -> Option<FileAction> {
    let mut action = None;
    egui::Panel::left("controls")
        .resizable(true)
        .default_size(320.0)
        .show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                controls_panel(ui, state);
                ui.small("W or Up arrow: full throttle · S: starter");
                ui.separator();
                ui.heading("Editor");
                action = file_row(ui, state, file_busy);
                editor_panel(ui, state);
            });
        });
    egui::Panel::right("gauges")
        .resizable(false)
        .default_size(220.0)
        .show(ui, |ui| view::gauges(ui, snap, audio));
    egui::Panel::bottom("plots").show(ui, |ui| view::plots(ui, snap, &state.dyno_curve));
    egui::CentralPanel::default().show(ui, |ui| view::engine(ui, snap));
    action
}

impl eframe::App for EngineApp {
    // `logic` runs every frame even while the window is hidden or occluded; `ui` does not, so a
    // quit timer placed in `ui` never fired when the smoke window opened behind other windows.
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self
            .quit_after
            .is_some_and(|q| self.started.elapsed().as_secs_f64() > q)
        {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if self.quit_after.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.poll_files(ui);
        self.poll_dyno(ui.ctx());
        let snap = self.sim.snapshot();
        self.state.quality_running = Some(snap.quality);
        match layout(
            ui,
            &mut self.state,
            &snap,
            &self.sim.audio,
            self.file_job.is_some(),
        ) {
            Some(FileAction::Load) => self.dialog(false, ui.ctx()),
            Some(FileAction::Save) => self.dialog(true, ui.ctx()),
            None => {}
        }
        if self.state.engine_dirty {
            self.state.engine_dirty = false;
            self.sim
                .send(Command::Engine(Box::new(self.state.def.clone())));
        }
        if self.state.quality_dirty {
            self.state.quality_dirty = false;
            self.sim.send(Command::Quality(self.state.quality));
        }
        if self.state.vehicle_dirty {
            self.state.vehicle_dirty = false;
            self.sim
                .send(Command::Vehicle(Box::new(self.state.vehicle.clone())));
        }
        let d = self.state.drive_input();
        if self.sent_drive != Some(d) {
            self.sim.send(Command::Drive(d));
            self.sent_drive = Some(d);
        }
        let c = self.state.controls();
        if self.sent != Some(c) {
            self.sim.send(Command::Controls(c));
            self.sent = Some(c);
        }
        // No request_repaint here: the sim thread requests one per published snapshot.
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controls::Preset;
    use crate::sim::Simulation;
    use engine_core::engine::Controls;

    #[test]
    fn a_dialog_thread_that_dies_is_reported_not_pending_forever() {
        let (tx, rx) = channel::<FileJob>();
        assert!(matches!(poll_job(&rx), JobPoll::Pending));
        drop(tx);
        assert!(matches!(poll_job(&rx), JobPoll::Lost));
        let (tx, rx) = channel();
        tx.send(FileJob::Load(None)).unwrap();
        assert!(matches!(poll_job(&rx), JobPoll::Done(FileJob::Load(None))));
    }

    /// Off-screen renders of the real layout to `target/screens/*.png`, for looking at the
    /// UI without opening a window on the desktop. Needs a GPU (or lavapipe):
    /// `cargo test -p engine-app render_screens -- --ignored`.
    #[test]
    #[ignore]
    fn render_screens() {
        let out = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/screens");
        std::fs::create_dir_all(&out).unwrap();
        for (preset, throttle) in [
            (Preset::Single, 0.0),
            (Preset::Inline4, 0.4),
            (Preset::LfaV10, 1.0),
            (Preset::FlatplaneV8, 1.0),
            (Preset::CrossplaneV8, 0.3),
        ] {
            let mut sim = Simulation::with_quality(
                preset.def(),
                preset.vehicle(),
                48_000.0,
                1,
                crate::quality::Quality::Ultra,
            );
            sim.controls = Controls {
                throttle,
                ..Controls::default()
            };
            let mut buf = vec![0.0f32; 48_000];
            sim.render(&mut buf);
            let snap = sim.snapshot();
            let mut state = UiState {
                preset: Some(preset),
                throttle,
                vehicle: preset.vehicle(),
                gear: 2,
                dyno_curve: crate::dyno::sweep(&preset.def()),
                ..UiState::with_engine(preset.def())
            };
            let mut h = egui_kittest::Harness::builder()
                .with_size(egui::vec2(1400.0, 860.0))
                .wgpu()
                .build_ui(|ui| {
                    layout(ui, &mut state, &snap, &Ok(48_000), false);
                });
            h.run();
            let img = h.render().expect("render");
            img.save(out.join(format!("{}.png", preset.slug())))
                .unwrap();
        }
    }
}
