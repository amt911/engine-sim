//! The engine editor: every parameter of the engine definition as a slider, in the units
//! people use (mm, cc, degrees, litres). Logic-bearing UI, tested with `egui_kittest`.

use crate::controls::UiState;
use eframe::egui;
use engine_core::config::{self, MAX_CYLINDERS};
use engine_core::units::m3_to_cc;
use std::ops::RangeInclusive;

/// Slider over `value · scale` (e.g. metres shown as mm). Returns whether it changed.
fn scaled(
    ui: &mut egui::Ui,
    value: &mut f64,
    scale: f64,
    range: RangeInclusive<f64>,
    step: f64,
    text: &str,
) -> bool {
    let mut shown = *value * scale;
    let changed = ui
        .add(
            egui::Slider::new(&mut shown, range)
                .step_by(step)
                .text(text),
        )
        .changed();
    if changed {
        *value = shown / scale;
    }
    changed
}

/// Parse "1-3-4-2", "1,3,4,2" or "1 3 4 2"; empty or "auto" means automatic (empty vec).
pub fn parse_firing_order(text: &str) -> Result<Vec<usize>, String> {
    let t = text.trim();
    if t.is_empty() || t.eq_ignore_ascii_case("auto") {
        return Ok(Vec::new());
    }
    t.split(|c: char| c == '-' || c == ',' || c.is_whitespace())
        .filter(|p| !p.is_empty())
        .map(|p| {
            p.parse::<usize>()
                .map_err(|_| format!("'{p}' is not a cylinder number"))
        })
        .collect()
}

pub fn format_firing_order(order: &[usize]) -> String {
    order
        .iter()
        .map(|c| c.to_string())
        .collect::<Vec<_>>()
        .join("-")
}

/// Draw the editor; edits `state.def` and marks it dirty when the result is valid.
pub fn editor_panel(ui: &mut egui::Ui, state: &mut UiState) {
    let before = state.def.clone();
    let d = &mut state.def;
    ui.horizontal(|ui| {
        let l = ui.label("Name");
        ui.text_edit_singleline(&mut d.name).labelled_by(l.id);
    });

    egui::CollapsingHeader::new("Cylinders & geometry")
        .default_open(true)
        .show(ui, |ui| {
            let mut n = d.cylinders;
            if ui
                .add(egui::Slider::new(&mut n, 1..=MAX_CYLINDERS).text("Cylinders"))
                .changed()
            {
                d.cylinders = n;
                d.firing_order.clear();
                state.firing_text.clear();
            }
            let g = &mut d.geometry;
            scaled(ui, &mut g.bore_m, 1000.0, 40.0..=140.0, 1.0, "Bore (mm)");
            scaled(
                ui,
                &mut g.stroke_m,
                1000.0,
                30.0..=140.0,
                1.0,
                "Stroke (mm)",
            );
            scaled(ui, &mut g.rod_m, 1000.0, 60.0..=260.0, 1.0, "Rod (mm)");
            ui.add(
                egui::Slider::new(&mut g.compression_ratio, 6.0..=16.0)
                    .step_by(0.1)
                    .text("Compression ratio"),
            );
            ui.label(format!(
                "Displacement {:.0} cc",
                m3_to_cc(d.displacement_m3())
            ));
        });

    egui::CollapsingHeader::new("Firing & combustion")
        .default_open(true)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                let l = ui.label("Firing order");
                ui.add(
                    egui::TextEdit::singleline(&mut state.firing_text)
                        .hint_text("auto")
                        .desired_width(140.0),
                )
                .labelled_by(l.id);
            });
            match parse_firing_order(&state.firing_text) {
                Ok(order) => d.firing_order = order,
                Err(e) => {
                    ui.colored_label(egui::Color32::LIGHT_RED, e);
                }
            }
            ui.add(
                egui::Slider::new(&mut d.burn_duration_deg, 20.0..=90.0)
                    .step_by(1.0)
                    .text("Burn duration (°)"),
            );
        });

    egui::CollapsingHeader::new("Valves").show(ui, |ui| {
        for (name, v) in [
            ("Intake", &mut d.intake_valve),
            ("Exhaust", &mut d.exhaust_valve),
        ] {
            ui.label(name);
            ui.add(
                egui::Slider::new(&mut v.open_deg, 0.0..=719.0)
                    .step_by(1.0)
                    .text(format!("{name} opens (° of 720)")),
            );
            ui.add(
                egui::Slider::new(&mut v.duration_deg, 120.0..=360.0)
                    .step_by(1.0)
                    .text(format!("{name} duration (°)")),
            );
            scaled(
                ui,
                &mut v.max_lift_m,
                1000.0,
                3.0..=18.0,
                0.1,
                &format!("{name} lift (mm)"),
            );
            scaled(
                ui,
                &mut v.diameter_m,
                1000.0,
                15.0..=70.0,
                0.5,
                &format!("{name} valve Ø (mm)"),
            );
        }
    });

    egui::CollapsingHeader::new("Intake & exhaust").show(ui, |ui| {
        scaled(
            ui,
            &mut d.plenum_volume_m3,
            1000.0,
            0.3..=15.0,
            0.1,
            "Plenum (L)",
        );
        scaled(
            ui,
            &mut d.throttle_diameter_m,
            1000.0,
            20.0..=150.0,
            1.0,
            "Throttle Ø (mm)",
        );
        scaled(
            ui,
            &mut d.idle_throttle,
            100.0,
            0.0..=2.0,
            0.01,
            "Idle air (% of throttle)",
        );
        scaled(
            ui,
            &mut d.exhaust_runner_length_m,
            1000.0,
            100.0..=2000.0,
            10.0,
            "Exhaust runner length (mm)",
        );
        scaled(
            ui,
            &mut d.exhaust_runner_diameter_m,
            1000.0,
            20.0..=90.0,
            1.0,
            "Exhaust runner Ø (mm)",
        );
        scaled(
            ui,
            &mut d.exhaust_outlet_diameter_m,
            1000.0,
            20.0..=110.0,
            1.0,
            "Exhaust outlet Ø (mm)",
        );
    });

    egui::CollapsingHeader::new("Mechanics").show(ui, |ui| {
        ui.add(
            egui::Slider::new(&mut d.flywheel_inertia_kgm2, 0.02..=0.6)
                .step_by(0.01)
                .text("Flywheel inertia (kg·m²)"),
        );
        ui.add(
            egui::Slider::new(&mut d.redline_rpm, 2000.0..=14000.0)
                .step_by(100.0)
                .text("Redline (rpm)"),
        );
        ui.add(
            egui::Slider::new(&mut d.starter_torque_nm, 5.0..=300.0)
                .step_by(1.0)
                .text("Starter torque (N·m)"),
        );
    });

    state.errors = match config::validate(&state.def) {
        Ok(()) => Vec::new(),
        Err(config::ConfigError::Invalid(e)) => e,
        Err(e) => vec![e.to_string()],
    };
    for e in &state.errors {
        ui.colored_label(egui::Color32::LIGHT_RED, e);
    }
    if state.def != before && state.errors.is_empty() {
        state.engine_dirty = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui_kittest::Harness;
    use egui_kittest::kittest::Queryable;
    use engine_core::engine::EngineDef;

    fn harness(def: EngineDef) -> Harness<'static, UiState> {
        let state = UiState {
            firing_text: format_firing_order(&def.firing_order),
            def,
            ..UiState::default()
        };
        Harness::new_ui_state(|ui, s: &mut UiState| editor_panel(ui, s), state)
    }

    fn nudge(h: &mut Harness<'static, UiState>, label: &str) {
        h.get_by_role_and_label(egui::accesskit::Role::Slider, label)
            .focus();
        h.run();
        h.key_press(egui::Key::ArrowRight);
        h.run();
    }

    #[test]
    fn firing_order_text_round_trip() {
        assert_eq!(parse_firing_order("1-3-4-2"), Ok(vec![1, 3, 4, 2]));
        assert_eq!(parse_firing_order(" 1, 3 ,4 2 "), Ok(vec![1, 3, 4, 2]));
        assert_eq!(parse_firing_order(""), Ok(vec![]));
        assert_eq!(parse_firing_order("Auto"), Ok(vec![]));
        assert!(parse_firing_order("1-x-2").is_err());
        assert_eq!(format_firing_order(&[1, 3, 4, 2]), "1-3-4-2");
        assert_eq!(format_firing_order(&[]), "");
    }

    #[test]
    fn adding_a_cylinder_rebuilds_with_automatic_firing_order() {
        let mut h = harness(EngineDef::inline4());
        nudge(&mut h, "Cylinders");
        let s = h.state();
        assert_eq!(s.def.cylinders, 5);
        assert!(s.def.firing_order.is_empty(), "reset to automatic");
        assert!(s.engine_dirty);
    }

    #[test]
    fn bore_slider_works_in_millimetres() {
        let mut h = harness(EngineDef::single());
        nudge(&mut h, "Bore (mm)");
        assert!(
            (h.state().def.geometry.bore_m - 0.087).abs() < 1e-9,
            "{}",
            h.state().def.geometry.bore_m
        );
        assert!(h.state().engine_dirty);
    }

    #[test]
    fn invalid_firing_order_is_shown_and_not_applied() {
        let mut h = harness(EngineDef::inline4());
        let field = h.get_by_label("Firing order");
        field.focus();
        h.run();
        h.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
        h.get_by_label("Firing order").type_text("1-3-3-2");
        h.run();
        assert!(!h.state().engine_dirty, "invalid engine must not be sent");
        h.get_by_label_contains("every cylinder exactly once");
    }

    #[test]
    fn shows_displacement() {
        let h = harness(EngineDef::lfa_v10());
        h.get_by_label_contains("4805 cc");
    }

    #[test]
    fn max_cylinders_is_the_slider_limit() {
        let mut d = EngineDef::single();
        d.cylinders = MAX_CYLINDERS;
        let mut h = harness(d);
        nudge(&mut h, "Cylinders");
        assert_eq!(h.state().def.cylinders, MAX_CYLINDERS);
    }
}
