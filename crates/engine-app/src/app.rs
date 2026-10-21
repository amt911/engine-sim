//! The eframe application shell: wires the controls, the sim thread and the views.

use crate::controls::{UiState, controls_panel};
use crate::runner::{Command, SimHandle};
use crate::view;
use eframe::egui;
use engine_core::engine::Controls;
use std::time::Instant;

pub struct EngineApp {
    state: UiState,
    sim: SimHandle,
    sent: Option<Controls>,
    quit_after: Option<f64>,
    started: Instant,
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
        }
    }
}

impl eframe::App for EngineApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        if self
            .quit_after
            .is_some_and(|q| self.started.elapsed().as_secs_f64() > q)
        {
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        egui::Panel::left("controls")
            .resizable(false)
            .default_size(230.0)
            .show(ui, |ui| {
                controls_panel(ui, &mut self.state);
                ui.separator();
                ui.small("W or Up arrow: full throttle · S: starter");
            });
        if self.state.engine_dirty {
            self.state.engine_dirty = false;
            self.sim.send(Command::Engine(self.state.def.clone()));
        }
        let c = self.state.controls();
        if self.sent != Some(c) {
            self.sim.send(Command::Controls(c));
            self.sent = Some(c);
        }
        let snap = self.sim.snapshot();
        egui::Panel::right("gauges")
            .resizable(false)
            .default_size(220.0)
            .show(ui, |ui| view::gauges(ui, &snap, &self.sim.audio));
        egui::Panel::bottom("plots").show(ui, |ui| view::plots(ui, &snap));
        egui::CentralPanel::default().show(ui, |ui| view::engine(ui, &snap));
        // No request_repaint here: the sim thread requests one per published snapshot.
    }
}
