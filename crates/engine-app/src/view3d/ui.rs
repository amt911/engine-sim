//! The 3D view in the central panel: the 2D/3D switch, the detail choice, the camera's mouse
//! input, and the frame handed to the renderer.

use super::camera::OrbitCamera;
use super::detail::{Detail, DetailGovernor};
use super::render::{Frame3d, MeshKey, Shared, target_size};
use super::scene::{self, Instance, Scene};
use crate::sim::Snapshot;
use eframe::egui;
use eframe::egui_wgpu;
use engine_core::engine::EngineDef;
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    TwoD,
    ThreeD,
}

#[derive(Debug, Clone)]
pub struct View3dState {
    pub mode: ViewMode,
    /// Whether the renderer is installed (a wgpu device exists); without it the view is 2D.
    pub available: bool,
    pub camera: Option<OrbitCamera>,
    pub governor: DetailGovernor,
    /// Bumped whenever the drawn engine changes, so the meshes and the still parts are rebuilt.
    pub def_version: u64,
    drawn: Option<EngineDef>,
    scene: Option<Scene>,
    moving: Vec<Instance>,
    shared: Arc<Mutex<Shared>>,
}

impl Default for View3dState {
    fn default() -> Self {
        Self {
            mode: ViewMode::ThreeD,
            available: false,
            camera: None,
            governor: DetailGovernor::default(),
            def_version: 0,
            drawn: None,
            scene: None,
            moving: Vec::new(),
            shared: Arc::default(),
        }
    }
}

impl View3dState {
    /// The view to draw: 3D only when asked for and a renderer exists.
    pub fn shows_3d(&self) -> bool {
        self.available && self.mode == ViewMode::ThreeD
    }

    /// Rebuild the still parts when the engine changed (compared by value: an edit, a load).
    fn sync(&mut self, def: &EngineDef) {
        if self.drawn.as_ref() != Some(def) {
            self.def_version += 1;
            let s = scene::build(def);
            self.camera = Some(OrbitCamera::framing(s.bounds.0, s.bounds.1));
            self.scene = Some(s);
            self.drawn = Some(def.clone());
        }
    }

    /// Fill the shared frame (still parts then moving ones) for the cylinders of `snap`.
    fn fill(&mut self, snap: &Snapshot, aspect: f32) {
        let Some(s) = &self.scene else { return };
        scene::moving(&snap.def, &snap.cylinders, &mut self.moving);
        if let Ok(mut sh) = self.shared.lock() {
            sh.instances.clear();
            sh.instances.extend_from_slice(&s.static_parts);
            sh.instances.extend_from_slice(&self.moving);
            if let Some(c) = &self.camera {
                sh.view_proj = c.view_proj(aspect);
            }
        }
    }
}

/// The bar over the engine view: 2D / 3D and the detail.
pub fn bar(ui: &mut egui::Ui, st: &mut View3dState) {
    ui.horizontal(|ui| {
        if !st.available {
            ui.add_enabled(false, egui::Button::new("3D"))
                .on_disabled_hover_text("No GPU device for the 3D view: showing 2D");
            return;
        }
        ui.selectable_value(&mut st.mode, ViewMode::TwoD, "2D");
        ui.selectable_value(&mut st.mode, ViewMode::ThreeD, "3D");
        if st.mode == ViewMode::ThreeD {
            let label = |d: Option<Detail>| match d {
                None => "Auto",
                Some(Detail::Low) => "Low",
                Some(Detail::Medium) => "Medium",
                Some(Detail::High) => "High",
            };
            let mut choice = st.governor.manual();
            egui::ComboBox::new("detail3d", "Detail")
                .selected_text(label(choice))
                .show_ui(ui, |ui| {
                    for d in [
                        None,
                        Some(Detail::Low),
                        Some(Detail::Medium),
                        Some(Detail::High),
                    ] {
                        ui.selectable_value(&mut choice, d, label(d));
                    }
                });
            if choice != st.governor.manual() {
                st.governor.set_manual(choice);
            }
            ui.small("drag: turn · wheel: zoom · double-click: reframe");
        }
    });
}

/// The 3D engine in the rest of the panel.
pub fn show(ui: &mut egui::Ui, st: &mut View3dState, snap: &Snapshot) {
    st.sync(&snap.def);
    let (rect, resp) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
    if let Some(c) = st.camera.as_mut() {
        let d = resp.drag_delta();
        c.drag(-d.x, d.y);
        if resp.hovered() {
            c.zoom(ui.input(|i| i.smooth_scroll_delta.y));
        }
    }
    if resp.double_clicked()
        && let Some(s) = &st.scene
    {
        st.camera = Some(OrbitCamera::framing(s.bounds.0, s.bounds.1));
    }
    st.governor.observe(1000.0 * ui.input(|i| i.unstable_dt));
    let aspect = rect.width() / rect.height().max(1.0);
    st.fill(snap, aspect);
    let e_over_r = snap
        .def
        .geometry
        .rotor
        .map_or(0.0, |r| (r.eccentricity_m / r.generating_radius_m) as f32);
    let frame = Frame3d {
        shared: st.shared.clone(),
        size_px: target_size([rect.width(), rect.height()], ui.ctx().pixels_per_point()),
        key: MeshKey {
            def_version: st.def_version,
            segments: st.governor.detail.segments(),
            e_over_r,
        },
    };
    ui.painter()
        .add(egui_wgpu::Callback::new_paint_callback(rect, frame));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_render_state_means_2d() {
        let st = View3dState::default();
        assert_eq!(st.mode, ViewMode::ThreeD);
        assert!(!st.shows_3d());
        let st = View3dState {
            available: true,
            ..View3dState::default()
        };
        assert!(st.shows_3d());
    }

    #[test]
    fn scene_rebuilds_when_the_definition_changes() {
        let mut st = View3dState::default();
        let mut d = EngineDef::inline4();
        st.sync(&d);
        assert_eq!(st.def_version, 1);
        st.sync(&d);
        assert_eq!(st.def_version, 1, "same engine: nothing rebuilt");
        d.cylinders = 6;
        st.sync(&d);
        assert_eq!(st.def_version, 2);
    }

    #[test]
    fn view_toggle_switches_to_2d_and_back() {
        use egui_kittest::{Harness, kittest::Queryable};
        let st = View3dState {
            available: true,
            ..View3dState::default()
        };
        let mut h = Harness::new_ui_state(|ui, s: &mut View3dState| bar(ui, s), st);
        h.get_by_label("2D").click();
        h.run();
        assert_eq!(h.state().mode, ViewMode::TwoD);
        assert!(
            h.query_by_label("Detail").is_none(),
            "no detail choice in 2D"
        );
        h.get_by_label("3D").click();
        h.run();
        assert_eq!(h.state().mode, ViewMode::ThreeD);
        assert!(h.query_by_label("Detail").is_some());
    }

    #[test]
    fn without_a_gpu_the_3d_button_is_off() {
        use egui_kittest::{Harness, kittest::Queryable};
        let mut h =
            Harness::new_ui_state(|ui, s: &mut View3dState| bar(ui, s), View3dState::default());
        h.run();
        assert!(h.query_by_label("2D").is_none());
        h.get_by_label("3D").click();
        h.run();
        assert!(
            !h.state().shows_3d(),
            "a click on the disabled 3D does nothing"
        );
    }
}
