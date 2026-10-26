//! The engine editor: every parameter of the engine definition as a slider, in the units
//! people use (mm, cc, degrees, litres). Logic-bearing UI, tested with `egui_kittest`.

use crate::controls::UiState;
use eframe::egui;
use engine_core::config::{self, MAX_CYLINDERS};
use engine_core::intake_fv::IntakeModel;
use engine_core::units::m3_to_cc;
use std::ops::RangeInclusive;

/// Slider over `value · scale` (e.g. metres shown as mm). Returns whether it changed.
/// An editor slider. `Edits` clamping: what the user enters is clamped and stepped, but an
/// engine's own values stay as they are; the default (`Always`) re-steps the bound value every
/// frame, so showing an LS3 (103.25 mm bore, 1 mm step) silently made it 103 mm.
fn slider<'a, N: egui::emath::Numeric>(
    value: &'a mut N,
    range: RangeInclusive<N>,
) -> egui::Slider<'a> {
    egui::Slider::new(value, range).clamping(egui::SliderClamping::Edits)
}

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
        .add(slider(&mut shown, range).step_by(step).text(text))
        .changed();
    if changed {
        *value = shown / scale;
    }
    changed
}

/// The exhaust after each collector, segment by segment: fields, reorder, remove, add.
fn segments_editor(ui: &mut egui::Ui, x: &mut engine_core::exhaust::ExhaustDef) {
    use engine_core::exhaust::Segment;
    egui::CollapsingHeader::new("After the collector")
        .default_open(true)
        .show(ui, |ui| {
            if x.after_collector.is_empty() {
                ui.label("A can and the tail pipe above.");
                if ui.button("Design the path").clicked() {
                    x.after_collector = x.path();
                }
                return;
            }
            let mut action: Option<(usize, i8)> = None; // (index, −1 up / +1 down / 0 remove)
            let n = x.after_collector.len();
            for (k, seg) in x.after_collector.iter_mut().enumerate() {
                let tag = format!("#{}", k + 1);
                ui.push_id(k, |ui| {
                    ui.horizontal(|ui| {
                        let name = match seg {
                            Segment::Pipe { .. } => "Pipe",
                            Segment::Muffler { .. } => "Silencer",
                            Segment::Catalyst { .. } => "Catalyst",
                            Segment::Resonator { .. } => "Resonator",
                        };
                        ui.strong(format!("{tag} {name}"));
                        if ui
                            .add_enabled(k > 0, egui::Button::new(format!("Up {tag}")))
                            .clicked()
                        {
                            action = Some((k, -1));
                        }
                        if ui
                            .add_enabled(k + 1 < n, egui::Button::new(format!("Down {tag}")))
                            .clicked()
                        {
                            action = Some((k, 1));
                        }
                        if ui.button(format!("Remove {tag}")).clicked() {
                            action = Some((k, 0));
                        }
                    });
                    let mm =
                        |ui: &mut egui::Ui, v: &mut f64, r: RangeInclusive<f64>, what: &str| {
                            scaled(ui, v, 1000.0, r, 1.0, &format!("{tag} {what} (mm)"));
                        };
                    match seg {
                        Segment::Pipe {
                            length_m,
                            diameter_m,
                        } => {
                            mm(ui, length_m, 10.0..=5000.0, "length");
                            mm(ui, diameter_m, 15.0..=150.0, "Ø");
                        }
                        Segment::Muffler {
                            length_m,
                            diameter_m,
                            tube_diameter_m,
                            ..
                        } => {
                            mm(ui, length_m, 50.0..=1200.0, "length");
                            mm(ui, diameter_m, 40.0..=400.0, "shell Ø");
                            mm(
                                ui,
                                tube_diameter_m,
                                0.0..=150.0,
                                "packed tube Ø (0 = empty)",
                            );
                        }
                        Segment::Catalyst {
                            length_m,
                            diameter_m,
                            cells_per_in2,
                            ..
                        } => {
                            mm(ui, length_m, 30.0..=400.0, "length");
                            mm(ui, diameter_m, 40.0..=250.0, "Ø");
                            ui.add(
                                slider(cells_per_in2, 100.0..=1200.0)
                                    .step_by(50.0)
                                    .text(format!("{tag} cells per in²")),
                            );
                        }
                        Segment::Resonator {
                            neck_diameter_m,
                            neck_length_m,
                            volume_m3,
                        } => {
                            mm(ui, neck_diameter_m, 5.0..=100.0, "neck Ø");
                            mm(ui, neck_length_m, 5.0..=300.0, "neck length");
                            scaled(
                                ui,
                                volume_m3,
                                1000.0,
                                0.1..=30.0,
                                0.1,
                                &format!("{tag} cavity (L)"),
                            );
                        }
                    }
                });
            }
            match action {
                Some((k, 0)) => {
                    x.after_collector.remove(k);
                }
                Some((k, -1)) => x.after_collector.swap(k, k - 1),
                Some((k, _)) => x.after_collector.swap(k, k + 1),
                None => {}
            }
            let bore = x.outlet_diameter();
            ui.horizontal(|ui| {
                if ui.button("Add pipe").clicked() {
                    x.after_collector.push(Segment::Pipe {
                        length_m: 0.5,
                        diameter_m: bore,
                    });
                }
                if ui.button("Add silencer").clicked() {
                    x.after_collector.push(Segment::Muffler {
                        length_m: 0.4,
                        diameter_m: 3.0 * bore,
                        tube_diameter_m: bore,
                        packing_resistivity: 16_000.0,
                        packing_porosity: 0.95,
                    });
                }
                if ui.button("Add catalyst").clicked() {
                    x.after_collector.push(Segment::Catalyst {
                        length_m: 0.13,
                        diameter_m: 2.2 * bore,
                        cells_per_in2: 400.0,
                        wall_m: 0.000_165,
                    });
                }
                if ui.button("Add resonator").clicked() {
                    x.after_collector.push(Segment::Resonator {
                        neck_diameter_m: 0.6 * bore,
                        neck_length_m: 0.05,
                        volume_m3: 0.003,
                    });
                }
            });
        });
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

/// Collector groups: "1-3-5-7-9 / 2-4-6-8-10"; empty means one collector for all.
pub fn parse_collectors(text: &str) -> Result<Vec<Vec<usize>>, String> {
    if text.trim().is_empty() {
        return Ok(Vec::new());
    }
    text.split('/').map(parse_firing_order).collect()
}

pub fn format_collectors(groups: &[Vec<usize>]) -> String {
    groups
        .iter()
        .map(|g| format_firing_order(g))
        .collect::<Vec<_>>()
        .join(" / ")
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
                .add(slider(&mut n, 1..=MAX_CYLINDERS).text("Cylinders"))
                .changed()
            {
                // banks, bank angle, collectors and firing order follow (one coherent engine)
                d.set_cylinders(n);
                state.firing_text = format_firing_order(&d.firing_order);
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
                slider(&mut g.compression_ratio, 6.0..=16.0)
                    .step_by(0.1)
                    .text("Compression ratio"),
            );
            ui.label(format!(
                "Displacement {:.0} cc",
                m3_to_cc(d.displacement_m3())
            ));
            ui.add(
                slider(&mut d.layout.bank_angle_deg, 0.0..=180.0)
                    .step_by(1.0)
                    .text("Bank angle (°)"),
            )
            .on_hover_text("0 = inline, 60–120 = V, 180 = flat (boxer)");
            ui.add_enabled_ui(d.layout.bank_angle_deg > 0.0, |ui| {
                ui.horizontal(|ui| {
                    let l = ui.label("Banks");
                    ui.add(
                        egui::TextEdit::singleline(&mut state.banks_text)
                            .hint_text("first half / second half")
                            .desired_width(180.0),
                    )
                    .labelled_by(l.id);
                });
            })
            .response
            .on_disabled_hover_text("Inline (bank angle 0): one bank");
            if let Ok(banks) = parse_collectors(&state.banks_text) {
                d.layout.banks = banks;
            }
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
                slider(&mut d.burn_duration_deg, 20.0..=90.0)
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
                slider(&mut v.open_deg, 0.0..=719.0)
                    .step_by(1.0)
                    .text(format!("{name} opens (° of 720)")),
            );
            ui.add(
                slider(&mut v.duration_deg, 120.0..=360.0)
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
            vtec_lobe(ui, name, v);
        }
        vtec_switch(ui, &mut d.vtec);
        phaser(ui, "Intake", &mut d.intake_phaser);
        phaser(ui, "Exhaust", &mut d.exhaust_phaser);
    });

    egui::CollapsingHeader::new("Intake & exhaust").show(ui, |ui| {
        let itb = d.intake.model == IntakeModel::Fv && d.intake.individual_throttles;
        ui.add_enabled_ui(!itb, |ui| {
            scaled(
                ui,
                &mut d.plenum_volume_m3,
                1000.0,
                0.3..=15.0,
                0.1,
                "Plenum (L)",
            );
        })
        .response
        .on_disabled_hover_text("Individual throttle bodies: no common plenum");
        if itb {
            // stored as the total bore the n bodies share (√n × each)
            let share = (d.cylinders.max(1) as f64).sqrt();
            let mut each = d.throttle_diameter_m / share;
            if scaled(
                ui,
                &mut each,
                1000.0,
                15.0..=80.0,
                1.0,
                "Throttle body Ø, each (mm)",
            ) {
                d.throttle_diameter_m = each * share;
            }
        } else {
            scaled(
                ui,
                &mut d.throttle_diameter_m,
                1000.0,
                20.0..=150.0,
                1.0,
                "Throttle Ø (mm)",
            );
        }
        scaled(
            ui,
            &mut d.idle_throttle,
            100.0,
            0.0..=2.0,
            0.01,
            "Idle air (% of throttle)",
        );
        ui.add(
            egui::Slider::new(&mut d.idle_rpm, 400.0..=2000.0)
                .step_by(50.0)
                .text("Idle speed (rpm)"),
        )
        .on_hover_text("The ECU trims the idle air to hold this speed with the pedal released");
        let it = &mut d.intake;
        ui.horizontal(|ui| {
            ui.radio_value(&mut it.model, IntakeModel::ZeroD, "0D plenum");
            ui.radio_value(&mut it.model, IntakeModel::Fv, "1D runners");
        });
        if it.model == IntakeModel::Fv {
            ui.checkbox(&mut it.individual_throttles, "Individual throttle bodies");
            scaled(
                ui,
                &mut it.runner_length_m,
                1000.0,
                50.0..=1000.0,
                10.0,
                "Runner length (mm)",
            );
            scaled(
                ui,
                &mut it.runner_diameter_m,
                1000.0,
                20.0..=80.0,
                1.0,
                "Runner Ø (mm)",
            );
        }
    });

    egui::CollapsingHeader::new("Exhaust")
        .default_open(true)
        .show(ui, |ui| {
            let x = &mut d.exhaust;
            scaled(
                ui,
                &mut x.primary_length_m,
                1000.0,
                100.0..=2000.0,
                10.0,
                "Primary length (mm)",
            );
            scaled(
                ui,
                &mut x.primary_diameter_m,
                1000.0,
                20.0..=90.0,
                1.0,
                "Primary Ø (mm)",
            );
            ui.horizontal(|ui| {
                let l = ui.label("Collectors");
                ui.add(
                    egui::TextEdit::singleline(&mut state.collectors_text)
                        .hint_text("all into one")
                        .desired_width(180.0),
                )
                .labelled_by(l.id);
            });
            if let Ok(groups) = parse_collectors(&state.collectors_text) {
                x.collectors = groups;
            }
            scaled(
                ui,
                &mut x.collector_length_m,
                1000.0,
                10.0..=1500.0,
                10.0,
                "Collector length (mm)",
            );
            scaled(
                ui,
                &mut x.collector_diameter_m,
                1000.0,
                20.0..=120.0,
                1.0,
                "Collector Ø (mm)",
            );
            let (tail_length, tail_diameter) = x.tail_mut();
            scaled(
                ui,
                tail_length,
                1000.0,
                100.0..=4000.0,
                10.0,
                "Tail pipe length (mm)",
            );
            scaled(
                ui,
                tail_diameter,
                1000.0,
                20.0..=120.0,
                1.0,
                "Tail pipe Ø (mm)",
            );
            ui.add_enabled_ui(x.groups(d.cylinders).len() > 1, |ui| {
                ui.add(
                    slider(&mut x.tail_spacing_m, 0.1..=2.0)
                        .step_by(0.05)
                        .text("Tail spacing (m)"),
                );
            })
            .response
            .on_disabled_hover_text("One tail pipe: nothing to spread");
            segments_editor(ui, x);
        });

    egui::CollapsingHeader::new("Sound")
        .default_open(true)
        .show(ui, |ui| {
            let snd = &mut d.sound;
            ui.add(
                slider(&mut snd.flow_noise, 0.0..=4.0)
                    .step_by(0.05)
                    .text("Flow noise"),
            )
            .on_hover_text(
                "Turbulent jet noise at the tail pipes, from each one's exit velocity \
                 (Lighthill's eighth power). 1 = the measured jet-noise level, 0 = off.",
            );
            ui.add(
                slider(&mut snd.intake, 0.0..=4.0)
                    .step_by(0.05)
                    .text("Intake roar"),
            );
            ui.add(
                slider(&mut snd.block, 0.0..=4.0)
                    .step_by(0.05)
                    .text("Engine block"),
            )
            .on_hover_text(
                "Combustion noise radiated by the block: the cylinder pressure through the \
                 structure response measured on engines. 1 = physical, 0 = off.",
            );
            ui.add(
                slider(&mut snd.combustion_jitter, 0.0..=4.0)
                    .step_by(0.05)
                    .text("Combustion variability"),
            );
            ui.add(
                slider(&mut snd.reverb, 0.0..=1.0)
                    .step_by(0.01)
                    .text("Room reverb"),
            );
            ui.add(
                slider(&mut snd.leveling, 0.0..=1.0)
                    .step_by(0.05)
                    .text("Idle leveling"),
            )
            .on_hover_text(
                "Lifts quiet running (idle) up to +12 dB toward full load; 0 = raw dynamics",
            );
            ui.add(
                slider(&mut snd.listener_distance_m, 0.5..=30.0)
                    .step_by(0.5)
                    .text("Listener distance (m)"),
            );
            ui.add(
                slider(&mut snd.listener_angle_deg, -90.0..=90.0)
                    .step_by(5.0)
                    .text("Listener angle (°)"),
            )
            .on_hover_text(
                "0 = straight behind; off the axis, each tail pipe arrives at its own time",
            );
            ui.checkbox(&mut snd.ground, "Street (ground reflection)")
                .on_hover_text(
                    "The asphalt returns each tail's sound from below: off = free field",
                );
            ui.add_enabled(
                snd.ground,
                slider(&mut snd.tail_height_m, 0.1..=1.0)
                    .step_by(0.05)
                    .text("Tail pipe height (m)"),
            );
            ui.add_enabled(
                snd.ground,
                slider(&mut snd.listener_height_m, 0.3..=2.0)
                    .step_by(0.05)
                    .text("Ear height (m)"),
            );
            ui.add(
                slider(&mut snd.street_width_m, 0.0..=40.0)
                    .step_by(1.0)
                    .text("Street width (m)"),
            )
            .on_hover_text(
                "Façades on both sides, this far apart: each returns the car's sound from its \
                 mirror image, later and weaker. 0 = open ground",
            );
            ui.add_enabled(
                snd.street_width_m > 0.0,
                slider(&mut snd.wall_reflection, 0.0..=1.0)
                    .step_by(0.05)
                    .text("Façade reflection"),
            )
            .on_hover_text("0.8: building walls with windows; 1: flat hard walls (ISO 9613-2)");
        });

    egui::CollapsingHeader::new("Mechanics").show(ui, |ui| {
        ui.add(
            slider(&mut d.flywheel_inertia_kgm2, 0.02..=0.6)
                .step_by(0.01)
                .text("Flywheel inertia (kg·m²)"),
        );
        ui.add(
            slider(&mut d.redline_rpm, 2000.0..=14000.0)
                .step_by(100.0)
                .text("Redline (rpm)"),
        );
        ui.add(
            slider(&mut d.starter_torque_nm, 5.0..=300.0)
                .step_by(1.0)
                .text("Starter torque (N·m)"),
        );
    });

    state.errors = match config::validate(&state.def) {
        Ok(()) => Vec::new(),
        Err(config::ConfigError::Invalid(e)) => e,
        Err(e) => vec![e.to_string()],
    };
    if let Err(e) = parse_firing_order(&state.firing_text) {
        state.errors.insert(0, e);
    }
    if let Err(e) = parse_collectors(&state.collectors_text) {
        state.errors.insert(0, e);
    }
    if let Err(e) = parse_collectors(&state.banks_text) {
        state.errors.insert(0, format!("banks: {e}"));
    }
    for e in &state.errors {
        ui.colored_label(egui::Color32::LIGHT_RED, e);
    }
    if state.def != before && state.errors.is_empty() {
        state.engine_dirty = true;
    }
}

/// A valve's VTEC lobe: on adds one a little longer and higher than its base lobe (a typical
/// VTEC step: about 15 % more duration and 40 % more lift, as on the K20A), then edited.
fn vtec_lobe(ui: &mut egui::Ui, name: &str, v: &mut engine_core::valve::ValveTiming) {
    let mut on = v.high.is_some();
    if ui.checkbox(&mut on, format!("{name} VTEC lobe")).changed() {
        v.high = on.then(|| engine_core::valve::CamLobe {
            open_deg: (v.open_deg - 0.075 * v.duration_deg).rem_euclid(720.0),
            duration_deg: 1.15 * v.duration_deg,
            max_lift_m: 1.4 * v.max_lift_m,
        });
    }
    if let Some(l) = &mut v.high {
        ui.add(
            slider(&mut l.open_deg, 0.0..=719.0)
                .step_by(1.0)
                .text(format!("{name} VTEC lobe opens (° of 720)")),
        );
        ui.add(
            slider(&mut l.duration_deg, 120.0..=360.0)
                .step_by(1.0)
                .text(format!("{name} VTEC lobe duration (°)")),
        );
        scaled(
            ui,
            &mut l.max_lift_m,
            1000.0,
            3.0..=25.0,
            0.1,
            &format!("{name} VTEC lobe lift (mm)"),
        );
    }
}

/// When VTEC engages: its speed, its hysteresis and the pedal it waits for.
fn vtec_switch(ui: &mut egui::Ui, vtec: &mut Option<engine_core::valve::Vtec>) {
    let mut on = vtec.is_some();
    if ui.checkbox(&mut on, "VTEC").changed() {
        *vtec = on.then_some(engine_core::valve::Vtec {
            engage_rpm: 6000.0,
            hysteresis_rpm: 300.0,
            min_throttle: 0.3,
        });
    }
    if let Some(v) = vtec {
        ui.add(
            slider(&mut v.engage_rpm, 1000.0..=12000.0)
                .step_by(100.0)
                .text("VTEC engages (rpm)"),
        );
        ui.add(
            slider(&mut v.hysteresis_rpm, 0.0..=2000.0)
                .step_by(50.0)
                .text("VTEC hysteresis (rpm)"),
        );
        ui.add(
            slider(&mut v.min_throttle, 0.0..=1.0)
                .step_by(0.05)
                .text("VTEC needs pedal"),
        );
    }
}

/// A camshaft's phaser (VVT-i, VTC): its range, its rate and its map, point by point.
fn phaser(ui: &mut egui::Ui, name: &str, phaser: &mut Option<engine_core::valve::Phaser>) {
    let lower = name.to_lowercase();
    let mut on = phaser.is_some();
    if ui
        .checkbox(&mut on, format!("{name} cam phaser (VVT)"))
        .changed()
    {
        // a starting map in the spirit of Honda's i-VTEC (Honda R&D Tech. Review 12-2, 2000,
        // Fig. 4): little overlap at idle, more at light load, a moderate advance at full load
        *phaser = on.then(|| engine_core::valve::Phaser {
            range_deg: 50.0,
            rate_deg_s: 30.0,
            map: vec![
                engine_core::valve::PhaserPoint {
                    rpm: 1000.0,
                    full_load_deg: 0.0,
                    no_load_deg: 0.0,
                },
                engine_core::valve::PhaserPoint {
                    rpm: 3000.0,
                    full_load_deg: 25.0,
                    no_load_deg: 35.0,
                },
                engine_core::valve::PhaserPoint {
                    rpm: 7000.0,
                    full_load_deg: 15.0,
                    no_load_deg: 20.0,
                },
            ],
        });
    }
    if let Some(p) = phaser {
        ui.add(
            slider(&mut p.range_deg, 0.0..=80.0)
                .step_by(1.0)
                .text(format!("{name} phaser range (°)")),
        );
        ui.add(
            slider(&mut p.rate_deg_s, 5.0..=300.0)
                .step_by(5.0)
                .text(format!("{name} phaser rate (°/s)")),
        );
        let range = p.range_deg;
        for (k, pt) in p.map.iter_mut().enumerate() {
            ui.add(
                slider(&mut pt.rpm, 0.0..=12000.0)
                    .step_by(100.0)
                    .text(format!("{name} map point {} rpm", k + 1)),
            );
            ui.add(
                slider(&mut pt.full_load_deg, 0.0..=range)
                    .step_by(1.0)
                    .text(format!("{name} point {} advance, full load (°)", k + 1)),
            );
            ui.add(
                slider(&mut pt.no_load_deg, 0.0..=range)
                    .step_by(1.0)
                    .text(format!("{name} point {} advance, no load (°)", k + 1)),
            );
        }
        ui.horizontal(|ui| {
            if ui.button(format!("Add {lower} map point")).clicked() {
                let last = p
                    .map
                    .last()
                    .copied()
                    .unwrap_or(engine_core::valve::PhaserPoint {
                        rpm: 1000.0,
                        full_load_deg: 0.0,
                        no_load_deg: 0.0,
                    });
                p.map.push(engine_core::valve::PhaserPoint {
                    rpm: last.rpm + 1000.0,
                    ..last
                });
            }
            if p.map.len() > 1 && ui.button(format!("Remove {lower} map point")).clicked() {
                p.map.pop();
            }
        });
        // the map's speeds stay in order (it is interpolated between neighbours)
        p.map.sort_by(|a, b| a.rpm.total_cmp(&b.rpm));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui_kittest::Harness;
    use egui_kittest::kittest::Queryable;
    use engine_core::engine::EngineDef;

    fn harness(def: EngineDef) -> Harness<'static, UiState> {
        // as the app does on loading an engine: the text fields start from it
        let state = UiState::with_engine(def);
        Harness::new_ui_state(|ui, s: &mut UiState| editor_panel(ui, s), state)
    }

    fn nudge(h: &mut Harness<'static, UiState>, label: &str) {
        h.get_by_role_and_label(egui::accesskit::Role::Slider, label)
            .focus();
        h.run();
        h.key_press(egui::Key::ArrowRight);
        h.run();
    }

    fn open_valves(h: &mut Harness<'static, UiState>) {
        press(h, "Valves");
    }

    #[test]
    fn vtec_can_be_switched_on_and_its_engagement_moved() {
        let mut h = harness(EngineDef::single());
        open_valves(&mut h);
        press(&mut h, "VTEC");
        assert!(h.state().def.vtec.is_some());
        press(&mut h, "Intake VTEC lobe");
        assert!(h.state().def.intake_valve.high.is_some());
        let before = h.state().def.vtec.unwrap().engage_rpm;
        nudge(&mut h, "VTEC engages (rpm)");
        assert!(h.state().def.vtec.unwrap().engage_rpm > before);
        assert!(h.state().engine_dirty);
        // and off again
        press(&mut h, "VTEC");
        assert!(h.state().def.vtec.is_none());
    }

    #[test]
    fn a_cam_phaser_can_be_added_and_its_map_edited() {
        let mut h = harness(EngineDef::single());
        open_valves(&mut h);
        press(&mut h, "Intake cam phaser (VVT)");
        let p = h.state().def.intake_phaser.clone().expect("phaser on");
        assert!(!p.map.is_empty());
        nudge(&mut h, "Intake phaser range (°)");
        assert!(h.state().def.intake_phaser.as_ref().unwrap().range_deg > p.range_deg);
        press(&mut h, "Add intake map point");
        assert_eq!(
            h.state().def.intake_phaser.as_ref().unwrap().map.len(),
            p.map.len() + 1
        );
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
        // the even order for five, written out
        assert_eq!(s.def.firing_order, [1, 2, 4, 5, 3]);
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
    fn unparsable_firing_order_blocks_saving() {
        // Regression: "1-x" showed an error but left `errors` empty, so Save stayed enabled
        // and wrote the previous firing order.
        let mut h = harness(EngineDef::inline4());
        h.get_by_label("Firing order").focus();
        h.run();
        h.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
        h.get_by_label("Firing order").type_text("1-x");
        h.run();
        assert!(
            h.state().errors.iter().any(|e| e.contains("'x'")),
            "{:?}",
            h.state().errors
        );
    }

    #[test]
    fn collectors_text_round_trip() {
        assert_eq!(parse_collectors(""), Ok(vec![]));
        assert_eq!(
            parse_collectors("1-3 / 2-4"),
            Ok(vec![vec![1, 3], vec![2, 4]])
        );
        assert!(parse_collectors("1-3 / x").is_err());
        assert_eq!(format_collectors(&[vec![1, 3], vec![2, 4]]), "1-3 / 2-4");
        assert_eq!(format_collectors(&[]), "");
    }

    #[test]
    fn collectors_field_edits_the_exhaust() {
        let mut h = harness(EngineDef::inline4());
        h.get_by_label("Collectors").focus();
        h.run();
        h.get_by_label("Collectors").type_text("1-4 / 2-3");
        h.run();
        assert_eq!(
            h.state().def.exhaust.collectors,
            vec![vec![1, 4], vec![2, 3]]
        );
        assert!(h.state().engine_dirty);
    }

    #[test]
    fn sound_sliders_edit_the_engine() {
        let mut h = harness(EngineDef::lfa_v10());
        nudge(&mut h, "Flow noise");
        assert!(h.state().def.sound.flow_noise > 0.0);
        assert!(h.state().engine_dirty);
        nudge(&mut h, "Engine block");
        assert_eq!(h.state().def.sound.block, 1.05);
    }

    #[test]
    fn intake_model_and_runner_controls_edit_the_engine() {
        let mut h = harness(EngineDef::inline4());
        h.get_by_label("Intake & exhaust").click();
        h.run();
        h.get_by_label("1D runners").click();
        h.run();
        assert_eq!(
            h.state().def.intake.model,
            engine_core::intake_fv::IntakeModel::Fv
        );
        h.get_by_label("Individual throttle bodies").click();
        h.run();
        assert!(h.state().def.intake.individual_throttles);
        nudge(&mut h, "Runner length (mm)");
        assert!(h.state().def.intake.runner_length_m > 0.3);
        assert!(h.state().engine_dirty);
    }

    #[test]
    fn itbs_edit_each_throttle_body_and_grey_out_the_plenum() {
        // LFA: ten 45 mm throttle bodies, stored as one total bore of √10 · 45 mm.
        let mut h = harness(EngineDef::lfa_v10());
        h.get_by_label("Intake & exhaust").click();
        h.run();
        nudge(&mut h, "Throttle body Ø, each (mm)");
        let want = 0.046 * 10f64.sqrt();
        let got = h.state().def.throttle_diameter_m;
        assert!((got - want).abs() < 1e-9, "{got} vs {want}");
        // no plenum with ITBs: its slider stays put
        let v = h.state().def.plenum_volume_m3;
        nudge(&mut h, "Plenum (L)");
        assert_eq!(h.state().def.plenum_volume_m3, v);
    }

    #[test]
    fn idle_leveling_slider_edits_the_sound() {
        // the Sound section is open by default
        let mut h = harness(EngineDef::inline4());
        h.get_by_role_and_label(egui::accesskit::Role::Slider, "Idle leveling")
            .focus();
        h.run();
        h.key_press(egui::Key::ArrowRight);
        h.run();
        assert!(h.state().def.sound.leveling > 0.0);
        assert!(h.state().engine_dirty);
    }

    fn press(h: &mut Harness<'static, UiState>, label: &str) {
        h.get_by_label(label).focus();
        h.run();
        h.key_press(egui::Key::Space);
        h.run();
    }

    #[test]
    fn the_exhaust_path_is_designed_segment_by_segment() {
        use engine_core::exhaust::Segment;
        let mut h = harness(EngineDef::inline4());
        let n = h.state().def.exhaust.after_collector.len();
        assert!(n > 0, "the preset has a road exhaust");
        press(&mut h, "Add resonator");
        let path = &h.state().def.exhaust.after_collector;
        assert_eq!(path.len(), n + 1);
        assert!(matches!(path.last(), Some(Segment::Resonator { .. })));
        let volume =
            |h: &Harness<'static, UiState>| match h.state().def.exhaust.after_collector.last() {
                Some(Segment::Resonator { volume_m3, .. }) => *volume_m3,
                _ => panic!(),
            };
        let v0 = volume(&h);
        nudge(&mut h, &format!("#{} cavity (L)", n + 1));
        assert!(volume(&h) > v0);
        press(&mut h, &format!("Up #{}", n + 1));
        assert!(matches!(
            h.state().def.exhaust.after_collector[n - 1],
            Segment::Resonator { .. }
        ));
        press(&mut h, &format!("Remove #{}", n));
        assert_eq!(h.state().def.exhaust.after_collector.len(), n);
        assert!(h.state().engine_dirty);
        // an engine without a path starts one from its can and tail
        let mut h = harness(EngineDef::single());
        assert!(h.state().def.exhaust.after_collector.is_empty());
        press(&mut h, "Design the path");
        let x = &h.state().def.exhaust;
        assert!(!x.after_collector.is_empty());
        assert_eq!(x.after_collector, x.path());
    }

    #[test]
    fn the_street_width_puts_facades_around_the_car() {
        let mut h = harness(EngineDef::inline4());
        assert_eq!(h.state().def.sound.street_width_m, 0.0);
        nudge(&mut h, "Street width (m)");
        assert!(h.state().def.sound.street_width_m > 0.0);
        assert!(h.state().def.sound.street().is_some());
        assert!(h.state().engine_dirty);
    }

    #[test]
    fn the_street_switch_and_heights_edit_the_sound() {
        let mut h = harness(EngineDef::inline4());
        assert!(h.state().def.sound.ground);
        nudge(&mut h, "Ear height (m)");
        assert!(h.state().def.sound.listener_height_m > 1.6);
        h.get_by_label("Street (ground reflection)").focus();
        h.run();
        h.key_press(egui::Key::Space);
        h.run();
        assert!(!h.state().def.sound.ground);
        assert!(h.state().engine_dirty);
    }

    #[test]
    fn listener_and_tail_spacing_controls_edit_the_engine() {
        let mut h = harness(EngineDef::crossplane_v8());
        nudge(&mut h, "Listener distance (m)");
        assert!(h.state().def.sound.listener_distance_m > 3.0);
        nudge(&mut h, "Listener angle (°)");
        assert!(h.state().def.sound.listener_angle_deg > 30.0);
        // the Exhaust section is open by default
        nudge(&mut h, "Tail spacing (m)");
        assert!(h.state().def.exhaust.tail_spacing_m > 1.1);
        assert!(h.state().engine_dirty);
    }

    #[test]
    fn showing_an_engine_does_not_round_its_values() {
        // The LS3's 103.25 mm bore under a 1 mm-step slider: drawing must not snap it.
        let mut h = harness(EngineDef::crossplane_v8());
        h.run();
        h.run();
        assert_eq!(h.state().def, EngineDef::crossplane_v8());
        assert!(!h.state().engine_dirty);
    }

    #[test]
    fn bank_angle_and_banks_edit_the_layout() {
        let mut h = harness(EngineDef::inline4());
        nudge(&mut h, "Bank angle (°)");
        assert!(h.state().def.layout.bank_angle_deg > 0.0);
        h.get_by_label("Banks").focus();
        h.run();
        h.get_by_label("Banks").type_text("1-3 / 2-4");
        h.run();
        assert_eq!(h.state().def.layout.banks, vec![vec![1, 3], vec![2, 4]]);
        assert!(h.state().engine_dirty);
    }

    #[test]
    fn setting_an_engine_fills_its_text_fields() {
        let s = UiState::with_engine(EngineDef::crossplane_v8());
        assert_eq!(s.firing_text, "1-8-7-2-6-5-4-3");
        assert_eq!(s.collectors_text, "1-3-5-7 / 2-4-6-8");
        assert_eq!(s.banks_text, "1-3-5-7 / 2-4-6-8");
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
