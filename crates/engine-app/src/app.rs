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
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        mut state: UiState,
        quit_after: Option<f64>,
    ) -> Self {
        if let Some(rs) = cc.wgpu_render_state.as_ref() {
            crate::view3d::render::install(rs);
            state.view3d.available = true;
        }
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
    egui::CentralPanel::default().show(ui, |ui| {
        crate::view3d::ui::bar(ui, &mut state.view3d);
        if state.view3d.shows_3d() {
            crate::view3d::ui::show(
                ui,
                &mut state.view3d,
                snap,
                &mut state.def,
                &mut state.engine_dirty,
            );
        } else {
            view::engine(ui, snap);
        }
    });
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
            (Preset::MazdaRenesis, 0.0),
            (Preset::Mazda13bBridgeport, 0.3),
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

    /// The engine in 3D, off-screen, for the presets of every kind (inline, V, rotary), saved to
    /// `target/screens/engine-3d-<preset>.png` and checked to hold a drawn engine. Needs a GPU
    /// (or lavapipe): `cargo test -p engine-app render_3d_screens -- --ignored`.
    #[test]
    #[ignore]
    fn render_3d_screens() {
        use egui_kittest::wgpu as kw;
        let out = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/screens");
        std::fs::create_dir_all(&out).unwrap();
        for preset in [
            Preset::Inline4,
            Preset::CrossplaneV8,
            Preset::LfaV10,
            Preset::MazdaRenesis,
            Preset::HondaK20c1,
            Preset::SubaruEj25,
        ] {
            let mut sim = Simulation::with_quality(
                preset.def(),
                preset.vehicle(),
                48_000.0,
                1,
                crate::quality::Quality::Low,
            );
            let mut buf = vec![0.0f32; 24_000];
            sim.render(&mut buf);
            let snap = sim.snapshot();
            let mut state = UiState::with_engine(preset.def());
            state.view3d.available = true;
            let rs = kw::create_render_state(
                kw::default_wgpu_setup(),
                eframe::egui_wgpu::RendererOptions::PREDICTABLE,
            );
            crate::view3d::render::install(&rs);
            let mut h = egui_kittest::Harness::builder()
                .with_size(egui::vec2(1400.0, 860.0))
                .renderer(kw::WgpuTestRenderer::from_render_state(rs))
                .build_ui(|ui| {
                    layout(ui, &mut state, &snap, &Ok(48_000), false);
                });
            h.run();
            let img = h.render().expect("render");
            img.save(out.join(format!("engine-3d-{}.png", preset.slug())))
                .unwrap();
            // the centre of the window (between the side panels, over the plots) is mostly the
            // 3D view: more than 5 % of it must differ from the background
            let (mut drawn, mut all) = (0usize, 0usize);
            let (w, hgt) = img.dimensions();
            for y in hgt / 10..hgt / 2 {
                for x in w * 3 / 10..w * 7 / 10 {
                    let p = img.get_pixel(x, y).0;
                    let d = (p[0] as i32 - 24).abs()
                        + (p[1] as i32 - 26).abs()
                        + (p[2] as i32 - 30).abs();
                    drawn += usize::from(d > 30);
                    all += 1;
                }
            }
            assert!(
                drawn * 20 > all,
                "{}: {drawn} of {all} pixels drawn",
                preset.slug()
            );
            // and opened by the cut-away layer
            drop(h);
            state.view3d.layers.cutaway = true;
            let rs = kw::create_render_state(
                kw::default_wgpu_setup(),
                eframe::egui_wgpu::RendererOptions::PREDICTABLE,
            );
            crate::view3d::render::install(&rs);
            let mut h = egui_kittest::Harness::builder()
                .with_size(egui::vec2(1400.0, 860.0))
                .renderer(kw::WgpuTestRenderer::from_render_state(rs))
                .build_ui(|ui| {
                    layout(ui, &mut state, &snap, &Ok(48_000), false);
                });
            h.run();
            let img = h.render().expect("render");
            img.save(out.join(format!("engine-3d-{}-cut.png", preset.slug())))
                .unwrap();
            // and close up, cut open, to look at the parts themselves
            drop(h);
            let piston = state.view3d.shared.lock().ok().and_then(|sh| {
                sh.instances
                    .iter()
                    .position(|i| i.part == crate::view3d::scene::Part::Piston)
                    .map(|k| (k, sh.instances[k].model.w_axis.truncate()))
            });
            if let (Some(c), Some((k, p))) = (state.view3d.camera.as_mut(), piston) {
                c.target = p;
                c.distance *= 0.3;
                // the piston clicked: its card and its arrows
                state.view3d.selected = Some((k, egui::pos2(760.0, 420.0)));
            }
            let rs = kw::create_render_state(
                kw::default_wgpu_setup(),
                eframe::egui_wgpu::RendererOptions::PREDICTABLE,
            );
            crate::view3d::render::install(&rs);
            let mut h = egui_kittest::Harness::builder()
                .with_size(egui::vec2(1400.0, 860.0))
                .renderer(kw::WgpuTestRenderer::from_render_state(rs))
                .build_ui(|ui| {
                    layout(ui, &mut state, &snap, &Ok(48_000), false);
                });
            h.run();
            let img = h.render().expect("render");
            img.save(out.join(format!("engine-3d-{}-close.png", preset.slug())))
                .unwrap();
        }
    }

    /// Clicking the 3D engine picks the part under the pointer and opens its card; Escape closes
    /// it. Needs a GPU (or lavapipe): `cargo test -p engine-app clicking_the_engine -- --ignored`.
    #[test]
    #[ignore]
    fn clicking_the_engine_opens_a_card() {
        use egui_kittest::kittest::Queryable;
        use egui_kittest::wgpu as kw;
        let preset = Preset::Inline4;
        let sim = Simulation::with_quality(
            preset.def(),
            preset.vehicle(),
            48_000.0,
            1,
            crate::quality::Quality::Low,
        );
        let snap = sim.snapshot();
        let mut state = UiState::with_engine(preset.def());
        state.view3d.available = true;
        let rs = kw::create_render_state(
            kw::default_wgpu_setup(),
            eframe::egui_wgpu::RendererOptions::PREDICTABLE,
        );
        crate::view3d::render::install(&rs);
        let mut h = egui_kittest::Harness::builder()
            .with_size(egui::vec2(1400.0, 860.0))
            .renderer(kw::WgpuTestRenderer::from_render_state(rs))
            .build_ui_state(
                |ui, state: &mut UiState| {
                    layout(ui, state, &snap, &Ok(48_000), false);
                },
                state,
            );
        h.run();
        // the middle of the view, where the camera looks: the engine
        let centre = egui::pos2(0.5 * (420.0 + 1180.0), 0.5 * (40.0 + 700.0));
        h.hover_at(centre);
        h.run();
        h.event(egui::Event::PointerButton {
            pos: centre,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: egui::Modifiers::NONE,
        });
        h.event(egui::Event::PointerButton {
            pos: centre,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: egui::Modifiers::NONE,
        });
        h.run();
        assert!(
            h.state().view3d.selected.is_some(),
            "nothing picked at the view's centre"
        );
        let titles = [
            "Cylinder",
            "Connecting rod",
            "Crankshaft",
            "Plenum",
            "Intake runner",
            "Cylinder head",
            "Block",
        ];
        assert!(
            titles.iter().any(|t| h.query_by_label(t).is_some()),
            "no card"
        );
        h.key_press(egui::Key::Escape);
        h.run();
        assert!(h.state().view3d.selected.is_none());
    }

    /// A handle dragged in the 3D view edits the engine: the clicked piston's bore arrow pulled
    /// outwards widens the bore, the engine following while it is held. A click on an arrow keeps
    /// the part selected, and a quick flick still takes the arrow pressed. Needs a GPU (or
    /// lavapipe): `cargo test -p engine-app dragging_a_handle -- --ignored`.
    #[test]
    #[ignore]
    fn dragging_a_handle_edits_the_engine() {
        use egui_kittest::wgpu as kw;
        let preset = Preset::Inline4;
        let snap = Simulation::with_quality(
            preset.def(),
            preset.vehicle(),
            48_000.0,
            1,
            crate::quality::Quality::Low,
        )
        .snapshot();
        let mut state = UiState::with_engine(preset.def());
        state.view3d.available = true;
        let bore = state.def.geometry.bore_m;
        let rs = kw::create_render_state(
            kw::default_wgpu_setup(),
            eframe::egui_wgpu::RendererOptions::PREDICTABLE,
        );
        crate::view3d::render::install(&rs);
        let mut h = egui_kittest::Harness::builder()
            .with_size(egui::vec2(1400.0, 860.0))
            .renderer(kw::WgpuTestRenderer::from_render_state(rs))
            .build_ui_state(
                |ui, state: &mut UiState| {
                    layout(ui, state, &snap, &Ok(48_000), false);
                },
                state,
            );
        h.run();
        let piston = h
            .state()
            .view3d
            .first(crate::view3d::scene::Part::Piston)
            .unwrap();
        h.state_mut().view3d.selected = Some((piston, egui::pos2(700.0, 300.0)));
        h.run();
        let (_, at) = *h
            .state()
            .view3d
            .handle_spots
            .iter()
            .find(|(f, _)| *f == crate::view3d::handles::Field::Bore)
            .expect("a bore handle on screen");
        // a click on the arrow (no move) keeps the piston selected, its arrows shown
        h.event(egui::Event::PointerButton {
            pos: at,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: egui::Modifiers::NONE,
        });
        h.run();
        h.event(egui::Event::PointerButton {
            pos: at,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: egui::Modifiers::NONE,
        });
        h.run();
        assert_eq!(
            h.state().view3d.selected.map(|s| s.0),
            Some(piston),
            "a click on an arrow is not a click on the scene"
        );
        // a quick flick: pressed on the arrow, the pointer already 25 px off its line when the
        // drag registers; the arrow pressed is still the one taken (not a turn of the camera)
        let yaw = h.state().view3d.camera.map(|c| c.yaw);
        h.hover_at(at);
        h.run();
        h.event(egui::Event::PointerButton {
            pos: at,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: egui::Modifiers::NONE,
        });
        h.run();
        h.hover_at(at + egui::vec2(25.0, 25.0));
        h.run();
        let flicked = h.state().view3d.drag.is_some();
        h.event(egui::Event::PointerButton {
            pos: at + egui::vec2(25.0, 25.0),
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: egui::Modifiers::NONE,
        });
        h.run();
        assert!(flicked, "the flick took the arrow");
        assert_eq!(
            h.state().view3d.camera.map(|c| c.yaw),
            yaw,
            "the camera did not turn"
        );
        h.state_mut().def.geometry.bore_m = bore;
        h.state_mut().view3d.selected = Some((piston, egui::pos2(700.0, 300.0)));
        h.run();
        // press on the handle, pull it 40 px along whichever way its axis points on screen
        // (both ways tried: one of them widens), release
        let press = |h: &mut egui_kittest::Harness<'_, UiState>, at: egui::Pos2, pressed| {
            h.event(egui::Event::PointerButton {
                pos: at,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: egui::Modifiers::NONE,
            });
        };
        for dir in [
            egui::vec2(40.0, 0.0),
            egui::vec2(-40.0, 0.0),
            egui::vec2(0.0, 40.0),
            egui::vec2(0.0, -40.0),
        ] {
            h.hover_at(at);
            h.run();
            press(&mut h, at, true);
            h.run();
            for k in 1..=8 {
                h.hover_at(at + dir * (k as f32 / 8.0));
                h.run();
            }
            // the engine follows the arrow before it is let go
            let held = h.state().def.geometry.bore_m;
            let dirty_held = h.state().engine_dirty;
            press(&mut h, at + dir, false);
            h.run();
            if h.state().def.geometry.bore_m > bore {
                assert!(held > bore, "changed while held: {held} vs {bore}");
                assert!(dirty_held, "the engine rebuilt while held");
                assert!(
                    h.state().view3d.selected.is_some(),
                    "the part stays selected after the drag"
                );
                break;
            }
            h.state_mut().view3d.selected = Some((piston, egui::pos2(700.0, 300.0)));
            h.run();
        }
        assert!(
            h.state().def.geometry.bore_m > bore,
            "{} vs {bore}",
            h.state().def.geometry.bore_m
        );
        assert!(h.state().engine_dirty, "the engine is rebuilt with it");
        assert!(h.state().def.geometry.bore_m <= 0.140);
    }

    #[test]
    #[ignore = "needs a GPU adapter (run with --ignored)"]
    fn typing_on_a_valves_card_edits_its_timing() {
        use egui_kittest::kittest::Queryable;
        use egui_kittest::wgpu as kw;
        let preset = Preset::Inline4;
        let snap = Simulation::with_quality(
            preset.def(),
            preset.vehicle(),
            48_000.0,
            1,
            crate::quality::Quality::Low,
        )
        .snapshot();
        let mut state = UiState::with_engine(preset.def());
        state.view3d.available = true;
        let duration = state.def.exhaust_valve.duration_deg;
        let rs = kw::create_render_state(
            kw::default_wgpu_setup(),
            eframe::egui_wgpu::RendererOptions::PREDICTABLE,
        );
        crate::view3d::render::install(&rs);
        let mut h = egui_kittest::Harness::builder()
            .with_size(egui::vec2(1400.0, 860.0))
            .renderer(kw::WgpuTestRenderer::from_render_state(rs))
            .build_ui_state(
                |ui, state: &mut UiState| {
                    layout(ui, state, &snap, &Ok(48_000), false);
                },
                state,
            );
        h.run();
        let valve = h
            .state()
            .view3d
            .first(crate::view3d::scene::Part::ExhaustValve)
            .unwrap();
        h.state_mut().view3d.selected = Some((valve, egui::pos2(700.0, 300.0)));
        h.run();
        // drag the card's duration 30 px to the right: a DragValue counts up as it is pulled
        let at = h
            .get_by_role_and_label(egui::accesskit::Role::SpinButton, "Duration")
            .rect()
            .center();
        h.hover_at(at);
        h.run();
        h.event(egui::Event::PointerButton {
            pos: at,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: egui::Modifiers::NONE,
        });
        h.run();
        for k in 1..=6 {
            h.hover_at(at + egui::vec2(5.0 * k as f32, 0.0));
            h.run();
        }
        h.event(egui::Event::PointerButton {
            pos: at + egui::vec2(30.0, 0.0),
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: egui::Modifiers::NONE,
        });
        h.run();
        let now = h.state().def.exhaust_valve.duration_deg;
        assert!(now > duration, "{now} vs {duration}");
        assert!(h.state().engine_dirty);
        assert!(h.state().view3d.selected.is_some(), "the card stays open");
    }
}
