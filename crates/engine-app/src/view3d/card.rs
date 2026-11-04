//! The floating card of a clicked part: its name and its numbers, from the engine definition
//! where the file holds them and from the part's own drawn size for pipes and cans (which are
//! drawn at their defined sizes).

use super::scene::{Instance, Part, Scene};
use engine_core::engine::EngineDef;

/// A card: a title and (label, value) rows, values with their units.
#[derive(Debug, Clone, PartialEq)]
pub struct Card {
    pub title: String,
    pub rows: Vec<(String, String)>,
}

fn mm(m: f64) -> String {
    format!("{:.1} mm", 1000.0 * m)
}

fn litres(m3: f64) -> String {
    format!("{:.2} L", 1000.0 * m3)
}

/// The card of `parts[index]` (the frame's instances: the scene's still parts first).
pub fn card(def: &EngineDef, scene: &Scene, parts: &[Instance], index: usize) -> Option<Card> {
    let part = parts.get(index)?;
    let g = def.geometry;
    let (s, _, _) = part.model.to_scale_rotation_translation();
    // a tube's drawn bore and length (unit cylinder: radius 1, height 1)
    let tube = || {
        vec![
            ("Bore".into(), mm(2.0 * s.x as f64)),
            ("Length of this run".into(), mm(s.y as f64)),
        ]
    };
    let (title, rows): (&str, Vec<(String, String)>) = match part.part.as_valve() {
        Part::Piston | Part::PistonRing | Part::Charge | Part::Liner => (
            "Cylinder",
            vec![
                ("Bore".into(), mm(g.bore_m)),
                ("Stroke".into(), mm(g.stroke_m)),
                (
                    "Displacement".into(),
                    format!("{:.0} cc", engine_core::units::m3_to_cc(g.displacement())),
                ),
                (
                    "Compression ratio".into(),
                    format!("{:.1}:1", g.compression_ratio),
                ),
            ],
        ),
        Part::Rod | Part::RodEye | Part::WristPin => (
            "Connecting rod",
            vec![
                ("Length (centres)".into(), mm(g.rod_m)),
                (
                    "Rod / stroke".into(),
                    format!("{:.2}", g.rod_m / g.stroke_m),
                ),
            ],
        ),
        Part::Crank | Part::Throw | Part::Web => (
            "Crankshaft",
            vec![
                ("Throw (half the stroke)".into(), mm(g.crank_radius())),
                ("Cylinders".into(), def.cylinders.to_string()),
            ],
        ),
        Part::IntakeValve | Part::ExhaustValve | Part::IntakeStem | Part::ExhaustStem => {
            let (name, v) = if part.part.as_valve() == Part::IntakeValve {
                ("Intake valve", &def.intake_valve)
            } else {
                ("Exhaust valve", &def.exhaust_valve)
            };
            (
                name,
                vec![
                    ("Head diameter".into(), mm(v.diameter_m)),
                    ("Lift".into(), mm(v.max_lift_m)),
                    ("Duration".into(), format!("{:.0}°", v.duration_deg)),
                    ("Opens".into(), format!("{:.0}°", v.open_deg)),
                ],
            )
        }
        Part::Pipe | Part::Joint => {
            // a primary or a collector: its whole defined length too
            let whole = scene
                .pipes
                .iter()
                .find(|p| p.instances.contains(&index))
                .map(|p| {
                    if p.pipe < def.cylinders {
                        (
                            format!("Primary of cylinder {}", p.pipe + 1),
                            def.exhaust.primary_length(p.pipe),
                        )
                    } else {
                        let k = p.pipe - def.cylinders;
                        (
                            format!("Collector {}", k + 1),
                            def.exhaust.collector_length(k),
                        )
                    }
                });
            let mut rows = tube();
            let title = match whole {
                Some((name, len)) => {
                    rows.insert(1, ("Length".into(), mm(len)));
                    return Some(Card { title: name, rows });
                }
                None => "Exhaust pipe",
            };
            (title, rows)
        }
        Part::Silencer => ("Silencer", tube()),
        Part::Catalyst => ("Catalyst", tube()),
        Part::Resonator => ("Resonator neck", tube()),
        Part::Runner => (
            "Intake runner",
            vec![
                ("Bore".into(), mm(def.intake.runner_diameter_m)),
                ("Length".into(), mm(def.intake.runner_length_m)),
            ],
        ),
        Part::Trumpet => (
            "Individual throttle",
            vec![("Runner bore".into(), mm(def.intake.runner_diameter_m))],
        ),
        Part::Plenum => (
            "Plenum",
            vec![("Volume".into(), litres(def.plenum_volume_m3))],
        ),
        Part::Throttle => (
            "Throttle",
            vec![("Bore".into(), mm(def.throttle_diameter_m))],
        ),
        Part::Airbox => (
            "Air box",
            def.intake.airbox.as_ref().map_or(Vec::new(), |a| {
                vec![
                    ("Volume".into(), litres(a.volume_m3)),
                    ("Inlet ducts".into(), a.ducts.to_string()),
                ]
            }),
        ),
        Part::Turbine | Part::Compressor | Part::Intercooler => (
            "Turbocharger",
            def.turbo.as_ref().map_or(Vec::new(), |t| {
                vec![
                    ("Compressor wheel".into(), mm(t.compressor_tip_diameter_m)),
                    (
                        "Wastegate".into(),
                        format!("{:.2} bar", t.wastegate_pa / 1e5),
                    ),
                ]
            }),
        ),
        Part::Housing
        | Part::SidePlate
        | Part::Rotor
        | Part::ApexSeal
        | Part::Eccentric
        | Part::Plug => (
            "Rotor housing",
            g.rotor.map_or(Vec::new(), |r| {
                vec![
                    ("Generating radius".into(), mm(r.generating_radius_m)),
                    ("Eccentricity".into(), mm(r.eccentricity_m)),
                    ("Width".into(), mm(r.width_m)),
                ]
            }),
        ),
        Part::Block | Part::Head => (
            if part.part == Part::Block {
                "Block"
            } else {
                "Cylinder head"
            },
            vec![
                (
                    "Layout".into(),
                    if def.layout.bank_angle_deg > 0.0 {
                        format!("{:.0}° V/flat", def.layout.bank_angle_deg)
                    } else {
                        "inline".into()
                    },
                ),
                ("Temperature".into(), "no data (not simulated)".into()),
            ],
        ),
    };
    Some(Card {
        title: title.into(),
        rows,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::view3d::scene::{build, moving};

    fn frame(def: &EngineDef) -> (Scene, Vec<Instance>) {
        let s = build(def);
        let views: Vec<_> = (0..def.cylinders)
            .map(|_| engine_core::engine::CylinderView {
                cycle_deg: 0.0,
                piston_m: 0.0,
                pressure_pa: 1e5,
                temperature_k: 300.0,
                intake_lift_m: 0.0,
                exhaust_lift_m: 0.0,
                burning: false,
            })
            .collect();
        let mut parts = s.static_parts.clone();
        let mut m = Vec::new();
        moving(def, &views, &mut m);
        parts.extend(m);
        (s, parts)
    }

    fn row<'a>(c: &'a Card, label: &str) -> &'a str {
        &c.rows.iter().find(|r| r.0 == label).unwrap().1
    }

    #[test]
    fn a_piston_shows_its_cylinders_numbers() {
        // the K20A: 86 × 86 mm, 1998 cc
        let d = EngineDef::honda_k20a();
        let (s, parts) = frame(&d);
        let k = parts.iter().position(|i| i.part == Part::Piston).unwrap();
        let c = card(&d, &s, &parts, k).unwrap();
        assert_eq!(c.title, "Cylinder");
        assert_eq!(row(&c, "Bore"), "86.0 mm");
        assert_eq!(row(&c, "Stroke"), "86.0 mm");
        assert_eq!(row(&c, "Displacement"), "500 cc");
    }

    #[test]
    fn a_primary_shows_its_whole_length_and_which_cylinder() {
        let mut d = EngineDef::inline4();
        d.exhaust.primary_length_m = 0.6;
        d.exhaust.primary_length_deviation_m = vec![0.0, 0.0, 0.05, 0.0];
        let (s, parts) = frame(&d);
        let p = &s.pipes[2];
        let c = card(&d, &s, &parts, p.instances.start).unwrap();
        assert_eq!(c.title, "Primary of cylinder 3");
        assert_eq!(row(&c, "Length"), "650.0 mm");
        assert_eq!(
            row(&c, "Bore"),
            format!("{:.1} mm", 1000.0 * d.exhaust.primary_diameter_m)
        );
    }

    #[test]
    fn the_plenum_shows_its_volume_and_parts_without_a_model_say_so() {
        let d = EngineDef::inline4();
        let (s, parts) = frame(&d);
        let k = parts.iter().position(|i| i.part == Part::Plenum).unwrap();
        let c = card(&d, &s, &parts, k).unwrap();
        assert_eq!(
            row(&c, "Volume"),
            format!("{:.2} L", 1000.0 * d.plenum_volume_m3)
        );
        let k = parts.iter().position(|i| i.part == Part::Head).unwrap();
        let c = card(&d, &s, &parts, k).unwrap();
        assert_eq!(row(&c, "Temperature"), "no data (not simulated)");
        assert_eq!(card(&d, &s, &parts, parts.len()), None);
    }
}
