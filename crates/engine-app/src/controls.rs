//! The driver/controls panel: presets, ignition, starter, throttle, spark advance.
//! Logic-bearing UI, so it is tested with `egui_kittest`.

use crate::dyno::DynoPoint;
use crate::quality::{Quality, QualityChoice};
use crate::sim::DriveInput;
use eframe::egui;
use engine_core::drivetrain::VehicleDef;
use engine_core::engine::{Controls, EngineDef};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    Single,
    Inline4,
    HondaK20a,
    LfaV10,
    FlatplaneV8,
    CrossplaneV8,
}

impl Preset {
    pub const ALL: [Preset; 6] = [
        Preset::Single,
        Preset::Inline4,
        Preset::HondaK20a,
        Preset::LfaV10,
        Preset::FlatplaneV8,
        Preset::CrossplaneV8,
    ];
    pub fn def(self) -> EngineDef {
        match self {
            Preset::Single => EngineDef::single(),
            Preset::Inline4 => EngineDef::inline4(),
            Preset::HondaK20a => EngineDef::honda_k20a(),
            Preset::LfaV10 => EngineDef::lfa_v10(),
            Preset::FlatplaneV8 => EngineDef::flatplane_v8(),
            Preset::CrossplaneV8 => EngineDef::crossplane_v8(),
        }
    }
    /// The vehicle this engine comes in.
    pub fn vehicle(self) -> VehicleDef {
        self.def().vehicle
    }
    /// File/CLI name, as in `presets/<slug>.yaml` and `--preset <slug>`.
    pub fn slug(self) -> &'static str {
        match self {
            Preset::Single => "single",
            Preset::Inline4 => "inline4",
            Preset::HondaK20a => "honda-k20a",
            Preset::LfaV10 => "lfa-v10",
            Preset::FlatplaneV8 => "v8-flatplane",
            Preset::CrossplaneV8 => "v8-crossplane",
        }
    }
    pub fn from_slug(slug: &str) -> Option<Preset> {
        Preset::ALL.into_iter().find(|p| p.slug() == slug)
    }
    pub fn label(self) -> &'static str {
        match self {
            Preset::Single => "Single 500",
            Preset::Inline4 => "Inline-4 2.0",
            Preset::HondaK20a => "Honda K20A 2.0 (Type R)",
            Preset::LfaV10 => "Lexus LFA V10",
            Preset::FlatplaneV8 => "V8 4.5 flat-plane (European)",
            Preset::CrossplaneV8 => "V8 6.2 cross-plane (American)",
        }
    }
}

#[derive(Debug, Clone)]
pub struct UiState {
    /// The built-in this engine came from; `None` after loading or creating a custom one.
    pub preset: Option<Preset>,
    /// The engine being edited; sent to the sim when `engine_dirty`.
    pub def: EngineDef,
    pub engine_dirty: bool,
    pub ignition: bool,
    pub throttle: f64,
    /// Set while a key holds the throttle wide open / the starter engaged.
    pub key_throttle: bool,
    pub starter: bool,
    /// Firing order as typed in the editor ("1-3-4-2"; empty = automatic).
    pub firing_text: String,
    /// Exhaust collector groups as typed ("1-3 / 2-4"; empty = one collector).
    pub collectors_text: String,
    /// Cylinder banks as typed, same format ("1-3 / 2-4"; empty = halves).
    pub banks_text: String,
    /// Validation problems of the edited engine (shown in the editor).
    pub errors: Vec<String>,
    /// Last file load/save message.
    pub file_msg: Option<String>,
    /// The car; sent to the sim when `vehicle_dirty`.
    pub vehicle: VehicleDef,
    pub vehicle_dirty: bool,
    /// Selected gear (0 = neutral), clutch pedal and brake pedal (held keys).
    pub gear: usize,
    pub clutch_held: bool,
    pub brake_held: bool,
    /// Last dyno curve, a run in progress, and a click waiting for the app to start one.
    pub dyno_curve: Vec<DynoPoint>,
    pub dyno_running: bool,
    pub dyno_request: bool,
    /// Simulation quality, sent to the sim when `quality_dirty`, and the level it runs at.
    pub quality: QualityChoice,
    pub quality_dirty: bool,
    pub quality_running: Option<Quality>,
}

impl UiState {
    /// Edit `def`: the engine and the text fields typed from it (firing order, collectors,
    /// banks), so they cannot disagree.
    pub fn set_engine(&mut self, def: EngineDef) {
        self.firing_text = crate::editor::format_firing_order(&def.firing_order);
        self.collectors_text = crate::editor::format_collectors(&def.exhaust.collectors);
        self.banks_text = crate::editor::format_collectors(&def.layout.banks);
        self.def = def;
    }
    /// Take a whole engine file, or a preset's definition: the engine and the vehicle it
    /// carries, in neutral.
    pub fn load_engine(&mut self, def: EngineDef) {
        self.vehicle = def.vehicle.clone();
        self.vehicle_dirty = true;
        self.gear = 0;
        self.set_engine(def);
        self.preset = None;
        self.engine_dirty = true;
    }
    /// What a save writes: the engine with the vehicle in use.
    pub fn engine_file(&self) -> EngineDef {
        EngineDef {
            vehicle: self.vehicle.clone(),
            ..self.def.clone()
        }
    }
    /// Start with this engine and the vehicle it carries.
    pub fn with_engine(def: EngineDef) -> Self {
        let mut s = Self {
            vehicle: def.vehicle.clone(),
            ..Self::default()
        };
        s.set_engine(def);
        s
    }
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            preset: Some(Preset::Single),
            def: Preset::Single.def(),
            engine_dirty: false,
            ignition: true,
            throttle: 0.0,
            key_throttle: false,
            starter: false,
            firing_text: "1".into(),
            collectors_text: String::new(),
            banks_text: String::new(),
            errors: Vec::new(),
            file_msg: None,
            vehicle: Preset::Single.vehicle(),
            vehicle_dirty: false,
            gear: 0,
            clutch_held: false,
            brake_held: false,
            quality: QualityChoice::Auto,
            quality_dirty: false,
            quality_running: None,
            dyno_curve: Vec::new(),
            dyno_running: false,
            dyno_request: false,
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
    /// Gear, clutch and brake the sim should run with this frame.
    pub fn drive_input(&self) -> DriveInput {
        DriveInput {
            gear: self.gear,
            clutch_pedal: self.clutch_held,
            brake: if self.brake_held { 1.0 } else { 0.0 },
        }
    }
}

/// Number keys 1..=6 → gear.
const GEAR_KEYS: [egui::Key; 6] = [
    egui::Key::Num1,
    egui::Key::Num2,
    egui::Key::Num3,
    egui::Key::Num4,
    egui::Key::Num5,
    egui::Key::Num6,
];

/// Gear keys (E up, Q down, N neutral, 1-6) and the held clutch/brake keys. Ignored while a
/// text field has focus. Gears the car does not have are ignored.
fn drive_keys(ui: &egui::Ui, state: &mut UiState) {
    if ui.ctx().text_edit_focused() {
        state.clutch_held = false;
        state.brake_held = false;
        return;
    }
    let top = state.vehicle.gear_ratios.len();
    let (up, down, neutral, number, clutch, brake) = ui.input(|i| {
        // A fresh press only: holding a key makes the OS send `repeat: true` events.
        let fresh = |key: egui::Key| {
            i.events.iter().any(|e| {
                matches!(e, egui::Event::Key { key: k, pressed: true, repeat: false, .. } if *k == key)
            })
        };
        (
            fresh(egui::Key::E),
            fresh(egui::Key::Q),
            fresh(egui::Key::N),
            GEAR_KEYS.iter().position(|k| fresh(*k)),
            i.key_down(egui::Key::C),
            i.key_down(egui::Key::B) || i.key_down(egui::Key::ArrowDown),
        )
    });
    if up {
        state.gear = (state.gear + 1).min(top);
    }
    if down {
        state.gear = state.gear.saturating_sub(1);
    }
    if neutral {
        state.gear = 0;
    }
    if let Some(n) = number
        && n < top
    {
        state.gear = n + 1;
    }
    state.clutch_held = clutch;
    state.brake_held = brake;
}

/// Quality: Auto (the best level this machine keeps in real time) or a fixed one.
fn quality_row(ui: &mut egui::Ui, state: &mut UiState) {
    ui.horizontal(|ui| {
        ui.label("Quality");
        let mut pick = |ui: &mut egui::Ui, choice: QualityChoice, label: &str| {
            if ui
                .selectable_label(state.quality == choice, label)
                .clicked()
            {
                state.quality = choice;
                state.quality_dirty = true;
            }
        };
        pick(ui, QualityChoice::Auto, "Auto");
        for q in Quality::ALL.into_iter().rev() {
            pick(ui, QualityChoice::Fixed(q), q.label());
        }
    });
    if let Some(q) = state.quality_running {
        ui.small(format!("running: {}", q.label()));
    }
}

/// Draw the panel; mutates `state`.
pub fn controls_panel(ui: &mut egui::Ui, state: &mut UiState) {
    ui.heading("Engine");
    for p in Preset::ALL {
        if ui
            .selectable_label(state.preset == Some(p), p.label())
            .clicked()
        {
            state.load_engine(p.def());
            state.preset = Some(p);
        }
    }
    quality_row(ui, state);
    ui.separator();
    ui.heading("Driver");
    ui.checkbox(&mut state.ignition, "Ignition");
    let starter = ui.button("Starter (hold S)");
    // Keys typed into a text field (name, firing order) must not drive the engine.
    let typing = ui.ctx().text_edit_focused();
    let (w, s_key) = ui.input(|i| {
        (
            !typing && (i.key_down(egui::Key::W) || i.key_down(egui::Key::ArrowUp)),
            !typing && i.key_down(egui::Key::S),
        )
    });
    state.starter = starter.is_pointer_button_down_on() || s_key;
    state.key_throttle = w;
    ui.add(egui::Slider::new(&mut state.throttle, 0.0..=1.0).text("Throttle (hold W)"));
    drive_keys(ui, state);
    ui.horizontal(|ui| {
        ui.label("Gear");
        if ui.selectable_label(state.gear == 0, "N").clicked() {
            state.gear = 0;
        }
        for g in 1..=state.vehicle.gear_ratios.len() {
            if ui
                .selectable_label(state.gear == g, g.to_string())
                .clicked()
            {
                state.gear = g;
            }
        }
    });
    ui.label(format!(
        "{} · clutch {} · brake {}",
        state.vehicle.name,
        if state.clutch_held { "DOWN" } else { "up" },
        if state.brake_held { "ON" } else { "off" }
    ));
    ui.small("E/Q: gear up/down · N: neutral · 1-6: gear · hold C: clutch · hold B or Down: brake");
    ui.separator();
    ui.heading("Dyno");
    if ui
        .add_enabled(!state.dyno_running, egui::Button::new("Run dyno sweep"))
        .clicked()
    {
        state.dyno_request = true;
    }
    if state.dyno_running {
        ui.small("Sweeping 1,000 rpm to redline at full throttle...");
    }
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
    fn the_quality_row_picks_a_level_or_auto() {
        let mut h = harness();
        assert_eq!(h.state().quality, QualityChoice::Auto);
        h.get_by_label("Low").click();
        h.run();
        assert_eq!(h.state().quality, QualityChoice::Fixed(Quality::Low));
        assert!(h.state().quality_dirty);
        h.state_mut().quality_dirty = false;
        h.get_by_label("Auto").click();
        h.run();
        assert_eq!(h.state().quality, QualityChoice::Auto);
        assert!(h.state().quality_dirty);
        // what Auto settled on is shown
        h.state_mut().quality_running = Some(Quality::High);
        h.run();
        assert!(h.query_by_label("running: High").is_some());
    }

    #[test]
    fn choosing_a_preset_loads_that_engine() {
        let mut h = harness();
        h.get_by_label("Inline-4 2.0").click();
        h.run();
        assert_eq!(h.state().preset, Some(Preset::Inline4));
        assert_eq!(h.state().def.cylinders, 4);
        assert!(h.state().engine_dirty);
    }

    #[test]
    fn slugs_round_trip() {
        for p in Preset::ALL {
            assert_eq!(Preset::from_slug(p.slug()), Some(p));
        }
        assert_eq!(Preset::from_slug("nope"), None);
    }

    #[test]
    fn presets_work_after_loading_a_custom_engine() {
        // Regression: after loading a file the preset stayed "Single", so clicking
        // "Single 500" was ignored.
        let mut custom = Preset::Inline4.def();
        custom.name = "From a file".into();
        let state = UiState {
            preset: None,
            def: custom,
            ..UiState::default()
        };
        let mut h = Harness::new_ui_state(|ui, s: &mut UiState| controls_panel(ui, s), state);
        h.get_by_label("Single 500").click();
        h.run();
        assert_eq!(h.state().def, Preset::Single.def());
        assert_eq!(h.state().preset, Some(Preset::Single));
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
    fn holding_s_engages_the_starter() {
        let mut h = harness();
        h.key_down(egui::Key::S);
        h.run();
        assert!(h.state().controls().starter);
    }

    #[test]
    fn keys_typed_into_a_text_field_do_not_drive_the_engine() {
        let mut h = Harness::new_ui_state(
            |ui, s: &mut UiState| {
                let l = ui.label("Note");
                ui.text_edit_singleline(&mut s.firing_text)
                    .labelled_by(l.id);
                controls_panel(ui, s);
            },
            UiState::default(),
        );
        h.get_by_label("Note").focus();
        h.run();
        h.key_down(egui::Key::W);
        h.key_down(egui::Key::S);
        h.run();
        assert_eq!(h.state().controls().throttle, 0.0);
        assert!(!h.state().controls().starter);
    }

    fn press(h: &mut Harness<'static, UiState>, key: egui::Key) {
        h.key_press(key);
        h.run();
    }

    #[test]
    fn e_and_q_shift_up_and_down_within_the_gearbox() {
        let mut h = harness();
        assert_eq!(h.state().gear, 0);
        press(&mut h, egui::Key::E);
        assert_eq!(h.state().gear, 1);
        press(&mut h, egui::Key::E);
        assert_eq!(h.state().gear, 2);
        press(&mut h, egui::Key::Q);
        assert_eq!(h.state().gear, 1);
        press(&mut h, egui::Key::Q);
        press(&mut h, egui::Key::Q);
        assert_eq!(h.state().gear, 0, "Q stops at neutral");
        // The motorcycle has 6 speeds: E never goes past the top one.
        for _ in 0..9 {
            press(&mut h, egui::Key::E);
        }
        assert_eq!(h.state().gear, 6);
    }

    #[test]
    fn holding_a_gear_key_does_not_cycle_through_gears() {
        // OS auto-repeat: the key stays down and more `pressed` events arrive (egui marks
        // them `repeat: true` because the key is already down).
        let mut h = harness();
        h.key_down(egui::Key::E);
        h.run();
        assert_eq!(h.state().gear, 1);
        for _ in 0..3 {
            h.event(egui::Event::Key {
                key: egui::Key::E,
                physical_key: Some(egui::Key::E),
                pressed: true,
                repeat: true,
                modifiers: egui::Modifiers::NONE,
            });
            h.run();
        }
        assert_eq!(h.state().gear, 1);
    }

    #[test]
    fn number_keys_pick_a_gear_and_n_returns_to_neutral() {
        let mut h = harness();
        press(&mut h, egui::Key::Num3);
        assert_eq!(h.state().gear, 3);
        press(&mut h, egui::Key::N);
        assert_eq!(h.state().gear, 0);
        press(&mut h, egui::Key::Num6);
        assert_eq!(h.state().gear, 6);
    }

    #[test]
    fn a_gear_the_car_does_not_have_is_ignored() {
        // The inline-4's hatchback is a 5-speed.
        let state = UiState {
            vehicle: Preset::Inline4.vehicle(),
            ..UiState::default()
        };
        let mut h = Harness::new_ui_state(|ui, s: &mut UiState| controls_panel(ui, s), state);
        press(&mut h, egui::Key::Num5);
        assert_eq!(h.state().gear, 5);
        press(&mut h, egui::Key::Num6);
        assert_eq!(h.state().gear, 5);
        press(&mut h, egui::Key::E);
        assert_eq!(h.state().gear, 5);
    }

    #[test]
    fn holding_c_presses_the_clutch_and_b_or_down_the_brake() {
        let mut h = harness();
        assert_eq!(h.state().drive_input(), DriveInput::default());
        h.key_down(egui::Key::C);
        h.run();
        assert!(h.state().drive_input().clutch_pedal);
        h.key_up(egui::Key::C);
        h.key_down(egui::Key::B);
        h.run();
        let d = h.state().drive_input();
        assert!(!d.clutch_pedal);
        assert_eq!(d.brake, 1.0);
        h.key_up(egui::Key::B);
        h.run();
        assert_eq!(h.state().drive_input().brake, 0.0);
        h.key_down(egui::Key::ArrowDown);
        h.run();
        assert_eq!(h.state().drive_input().brake, 1.0);
    }

    #[test]
    fn gear_buttons_select_gears() {
        let mut h = harness();
        h.get_by_label("4").click();
        h.run();
        assert_eq!(h.state().gear, 4);
        h.get_by_label("N").click();
        h.run();
        assert_eq!(h.state().gear, 0);
    }

    #[test]
    fn driving_keys_are_ignored_while_typing() {
        let mut h = Harness::new_ui_state(
            |ui, s: &mut UiState| {
                let l = ui.label("Note");
                ui.text_edit_singleline(&mut s.firing_text)
                    .labelled_by(l.id);
                controls_panel(ui, s);
            },
            UiState::default(),
        );
        h.get_by_label("Note").focus();
        h.run();
        h.key_down(egui::Key::C);
        h.key_down(egui::Key::B);
        h.key_press(egui::Key::E);
        h.key_press(egui::Key::Num2);
        h.run();
        assert_eq!(h.state().gear, 0);
        assert_eq!(h.state().drive_input(), DriveInput::default());
    }

    #[test]
    fn choosing_a_preset_fits_its_car_and_resets_the_gear() {
        let mut h = harness();
        press(&mut h, egui::Key::Num2);
        h.get_by_label("Lexus LFA V10").click();
        h.run();
        assert_eq!(h.state().vehicle, VehicleDef::lfa_v10());
        assert!(h.state().vehicle_dirty);
        assert_eq!(h.state().gear, 0);
    }

    #[test]
    fn a_loaded_engine_brings_its_own_vehicle_not_the_previous_one() {
        // User report (2026-10-22): with the motorcycle selected, an imported V8 kept the
        // motorcycle's gearing and name.
        let mut s = UiState::with_engine(Preset::Single.def());
        s.preset = Some(Preset::Single);
        s.vehicle = Preset::Single.vehicle();
        s.gear = 3;
        s.load_engine(EngineDef::crossplane_v8());
        assert_eq!(
            UiState::with_engine(EngineDef::lfa_v10()).vehicle,
            VehicleDef::lfa_v10()
        );
        assert_eq!(s.preset, None);
        assert_eq!(s.vehicle, VehicleDef::crossplane_v8());
        assert!(s.vehicle_dirty && s.engine_dirty);
        assert_eq!(s.gear, 0);
        // and what it saves carries the vehicle in use
        s.vehicle = VehicleDef::lfa_v10();
        assert_eq!(s.engine_file().vehicle, VehicleDef::lfa_v10());
    }

    #[test]
    fn dyno_button_requests_a_sweep_and_is_disabled_while_running() {
        let mut h = harness();
        h.get_by_label("Run dyno sweep").click();
        h.run();
        assert!(h.state().dyno_request);
        let mut busy = harness();
        busy.state_mut().dyno_running = true;
        busy.run();
        busy.get_by_label("Run dyno sweep").click();
        busy.run();
        assert!(!busy.state().dyno_request);
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
