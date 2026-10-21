//! The driver/controls panel: presets, ignition, starter, throttle, spark advance.
//! Logic-bearing UI, so it is tested with `egui_kittest`.

use eframe::egui;
use engine_core::engine::{Controls, EngineDef};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    Single,
    Inline4,
    LfaV10,
}

impl Preset {
    pub const ALL: [Preset; 3] = [Preset::Single, Preset::Inline4, Preset::LfaV10];
    pub fn def(self) -> EngineDef {
        match self {
            Preset::Single => EngineDef::single(),
            Preset::Inline4 => EngineDef::inline4(),
            Preset::LfaV10 => EngineDef::lfa_v10(),
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Preset::Single => "Single 500",
            Preset::Inline4 => "Inline-4 2.0",
            Preset::LfaV10 => "Lexus LFA V10",
        }
    }
}

#[derive(Debug, Clone)]
pub struct UiState {
    pub preset: Preset,
    /// The engine being edited; sent to the sim when `engine_dirty`.
    pub def: EngineDef,
    pub engine_dirty: bool,
    pub ignition: bool,
    pub throttle: f64,
    /// Set while a key holds the throttle wide open / the starter engaged.
    pub key_throttle: bool,
    pub starter: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            preset: Preset::Single,
            def: Preset::Single.def(),
            engine_dirty: false,
            ignition: true,
            throttle: 0.0,
            key_throttle: false,
            starter: false,
        }
    }
}

impl UiState {
    /// Controls the sim should run with this frame.
    pub fn controls(&self) -> Controls {
        Controls {
            throttle: if self.key_throttle {
                1.0
            } else {
                self.throttle
            },
            ignition: self.ignition,
            starter: self.starter,
            ..Controls::default()
        }
    }
}

/// Draw the panel; mutates `state`.
pub fn controls_panel(ui: &mut egui::Ui, state: &mut UiState) {
    ui.heading("Engine");
    for p in Preset::ALL {
        if ui.selectable_label(state.preset == p, p.label()).clicked() && state.preset != p {
            state.preset = p;
            state.def = p.def();
            state.engine_dirty = true;
        }
    }
    ui.separator();
    ui.heading("Driver");
    ui.checkbox(&mut state.ignition, "Ignition");
    let starter = ui.button("Starter (hold S)");
    let (w, s_key) = ui.input(|i| {
        (
            i.key_down(egui::Key::W) || i.key_down(egui::Key::ArrowUp),
            i.key_down(egui::Key::S),
        )
    });
    state.starter = starter.is_pointer_button_down_on() || s_key;
    state.key_throttle = w;
    ui.add(egui::Slider::new(&mut state.throttle, 0.0..=1.0).text("Throttle (hold W)"));
    ui.separator();
    ui.heading("Tuning");
    let r = ui.add(
        egui::Slider::new(&mut state.def.spark_advance_deg, 0.0..=50.0)
            .step_by(1.0)
            .text("Spark advance (° BTDC)"),
    );
    if r.changed() {
        state.engine_dirty = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui_kittest::Harness;
    use egui_kittest::kittest::Queryable;

    fn harness() -> Harness<'static, UiState> {
        Harness::new_ui_state(
            |ui, s: &mut UiState| controls_panel(ui, s),
            UiState::default(),
        )
    }

    #[test]
    fn ignition_checkbox_toggles_ignition() {
        let mut h = harness();
        assert!(h.state().controls().ignition);
        h.get_by_label("Ignition").click();
        h.run();
        assert!(!h.state().controls().ignition);
    }

    #[test]
    fn choosing_a_preset_loads_that_engine() {
        let mut h = harness();
        h.get_by_label("Inline-4 2.0").click();
        h.run();
        assert_eq!(h.state().preset, Preset::Inline4);
        assert_eq!(h.state().def.cylinders, 4);
        assert!(h.state().engine_dirty);
    }

    #[test]
    fn lfa_preset_is_selectable() {
        let mut h = harness();
        h.get_by_label("Lexus LFA V10").click();
        h.run();
        assert_eq!(h.state().def.cylinders, 10);
    }

    #[test]
    fn holding_w_opens_the_throttle_fully() {
        let mut h = harness();
        assert_eq!(h.state().controls().throttle, 0.0);
        h.key_down(egui::Key::W);
        h.run();
        assert_eq!(h.state().controls().throttle, 1.0);
    }

    #[test]
    fn spark_advance_slider_edits_the_engine() {
        let mut h = harness();
        let before = h.state().def.spark_advance_deg;
        let slider =
            h.get_by_role_and_label(egui::accesskit::Role::Slider, "Spark advance (° BTDC)");
        slider.focus();
        h.run();
        h.key_press(egui::Key::ArrowRight);
        h.run();
        assert!(h.state().def.spark_advance_deg > before);
        assert!(h.state().engine_dirty);
    }
}
