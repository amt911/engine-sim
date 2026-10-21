//! The eframe application shell: wires the controls, the sim thread and the views.

use crate::controls::{UiState, controls_panel};
use crate::editor::{editor_panel, format_firing_order};
use crate::files;
use crate::runner::{Command, SimHandle};
use crate::sim::Snapshot;
use crate::view;
use eframe::egui;
use engine_core::engine::Controls;
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, channel};
use std::time::Instant;

/// Result of a native file dialog, which runs on its own thread so the frame never blocks.
enum FileJob {
    Load(Option<PathBuf>),
    Save(Option<PathBuf>),
}

pub struct EngineApp {
    state: UiState,
    sim: SimHandle,
    sent: Option<Controls>,
    quit_after: Option<f64>,
    started: Instant,
    file_job: Option<Receiver<FileJob>>,
}

impl EngineApp {
    pub fn new(cc: &eframe::CreationContext<'_>, state: UiState, quit_after: Option<f64>) -> Self {
        let sim = SimHandle::spawn(state.def.clone(), Some(cc.egui_ctx.clone()));
        Self {
            state,
            sim,
            sent: None,
            quit_after,
            started: Instant::now(),
            file_job: None,
        }
    }
}

impl EngineApp {
    fn dialog(&mut self, save: bool, ctx: &egui::Context) {
        let (tx, rx) = channel();
        let ctx = ctx.clone();
        let name = format!(
            "{}.yaml",
            self.state.def.name.replace(' ', "-").to_lowercase()
        );
        std::thread::spawn(move || {
            let d = rfd::FileDialog::new().add_filter("Engine", &["yaml", "yml", "json"]);
            let job = if save {
                FileJob::Save(d.set_file_name(name).save_file())
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
                self.state.firing_text = format_firing_order(&def.firing_order);
                self.state.def = def;
                self.state.engine_dirty = true;
                self.state.file_msg = Some(format!("Loaded {}", path.display()));
            }
            Err(e) => self.state.file_msg = Some(e),
        }
    }

    fn poll_files(&mut self, ui: &egui::Ui) {
        if let Some(job) = self.file_job.as_ref().and_then(|rx| rx.try_recv().ok()) {
            self.file_job = None;
            match job {
                FileJob::Load(Some(p)) => self.load(&p),
                FileJob::Save(Some(p)) => {
                    self.state.file_msg = Some(match files::save_engine(&p, &self.state.def) {
                        Ok(()) => format!("Saved {}", p.display()),
                        Err(e) => e,
                    });
                }
                FileJob::Load(None) | FileJob::Save(None) => {}
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
    egui::Panel::bottom("plots").show(ui, |ui| view::plots(ui, snap));
    egui::CentralPanel::default().show(ui, |ui| view::engine(ui, snap));
    action
}

impl eframe::App for EngineApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        if self
            .quit_after
            .is_some_and(|q| self.started.elapsed().as_secs_f64() > q)
        {
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        self.poll_files(ui);
        let snap = self.sim.snapshot();
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
        ] {
            let mut sim = Simulation::new(preset.def(), 48_000.0, 1);
            sim.controls = Controls {
                throttle,
                ..Controls::default()
            };
            let mut buf = vec![0.0f32; 48_000];
            sim.render(&mut buf);
            let snap = sim.snapshot();
            let mut state = UiState {
                preset,
                def: preset.def(),
                firing_text: crate::editor::format_firing_order(&preset.def().firing_order),
                throttle,
                ..UiState::default()
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
