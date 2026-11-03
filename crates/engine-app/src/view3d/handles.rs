//! Handles on the clicked part: drag one along its axis and the number it stands for changes
//! (bore, stroke, rod, a runner's or a primary's length, the plenum's volume), within the
//! editor's ranges. The engine takes the new value once, on release (one rebuild, as a slider).

use super::scene::{Instance, Part, Scene};
use engine_core::engine::EngineDef;
use glam::Vec3;

/// What a handle edits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    Bore,
    Stroke,
    Rod,
    RunnerLength,
    PrimaryLength,
    PlenumVolume,
}

/// A handle: where it stands, the direction it moves along, what it edits and how much the
/// value changes per metre the handle travels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Handle {
    pub at: Vec3,
    pub axis: Vec3,
    pub field: Field,
    pub per_metre: f64,
}

impl Field {
    pub fn label(self) -> &'static str {
        match self {
            Field::Bore => "Bore",
            Field::Stroke => "Stroke",
            Field::Rod => "Rod length",
            Field::RunnerLength => "Runner length",
            Field::PrimaryLength => "Primary length",
            Field::PlenumVolume => "Plenum volume",
        }
    }

    /// The editor's range for it, SI (`editor_panel`: bore 40–140 mm, stroke 30–140, rod
    /// 60–260, runners 50–1000, primaries 100–2000 mm, plenum 0.3–15 L).
    pub fn range(self) -> (f64, f64) {
        match self {
            Field::Bore => (0.040, 0.140),
            Field::Stroke => (0.030, 0.140),
            Field::Rod => (0.060, 0.260),
            Field::RunnerLength => (0.050, 1.0),
            Field::PrimaryLength => (0.100, 2.0),
            Field::PlenumVolume => (0.3e-3, 15e-3),
        }
    }

    pub fn get(self, d: &EngineDef) -> f64 {
        match self {
            Field::Bore => d.geometry.bore_m,
            Field::Stroke => d.geometry.stroke_m,
            Field::Rod => d.geometry.rod_m,
            Field::RunnerLength => d.intake.runner_length_m,
            Field::PrimaryLength => d.exhaust.primary_length_m,
            Field::PlenumVolume => d.plenum_volume_m3,
        }
    }

    /// Set it, clamped to its range.
    pub fn set(self, d: &mut EngineDef, value: f64) {
        let (lo, hi) = self.range();
        let v = if value.is_finite() {
            value.clamp(lo, hi)
        } else {
            self.get(d)
        };
        match self {
            Field::Bore => d.geometry.bore_m = v,
            Field::Stroke => d.geometry.stroke_m = v,
            Field::Rod => d.geometry.rod_m = v,
            Field::RunnerLength => d.intake.runner_length_m = v,
            Field::PrimaryLength => d.exhaust.primary_length_m = v,
            Field::PlenumVolume => d.plenum_volume_m3 = v,
        }
    }

    /// The value for display, in the editor's units.
    pub fn show(self, v: f64) -> String {
        match self {
            Field::PlenumVolume => format!("{:.2} L", 1000.0 * v),
            _ => format!("{:.1} mm", 1000.0 * v),
        }
    }
}

/// The value after dragging a handle `along_px` pixels along its on-screen axis, where one metre
/// of travel is `px_per_m` pixels: clamped to the field's range.
pub fn dragged(h: &Handle, start: f64, along_px: f32, px_per_m: f32) -> f64 {
    let (lo, hi) = h.field.range();
    let metres = if px_per_m > 0.0 {
        (along_px / px_per_m) as f64
    } else {
        0.0
    };
    (start + h.per_metre * metres).clamp(lo, hi)
}

/// The handles of `parts[selected]`.
pub fn handles(def: &EngineDef, scene: &Scene, parts: &[Instance], selected: usize) -> Vec<Handle> {
    let Some(p) = parts.get(selected) else {
        return Vec::new();
    };
    let at = |v: Vec3| p.model.transform_point3(v);
    let b = def.geometry.bore_m as f32;
    match p.part {
        Part::Piston | Part::PistonRing | Part::Charge => {
            // the piston's mesh: y up its bore from its base, x and z across: the crown's centre
            // and the bore's edge across it
            // a cylinder's moving parts go charge, piston, three rings: its piston is the nearest
            let piston = (selected.saturating_sub(4)..(selected + 2).min(parts.len()))
                .filter(|&i| parts[i].part == Part::Piston)
                .min_by_key(|&i| i.abs_diff(selected))
                .map_or(p, |i| &parts[i]);
            let up = piston.model.transform_vector3(Vec3::Y).normalize_or_zero();
            let across = piston.model.transform_vector3(Vec3::Z).normalize_or_zero();
            let crown = piston.model.transform_point3(Vec3::Y);
            vec![
                // the bore's edge moves by half the bore's change
                Handle {
                    at: crown + 0.62 * b * across,
                    axis: across,
                    field: Field::Bore,
                    per_metre: 2.0,
                },
                // the crown at the top of its stroke moves by the stroke's change
                Handle {
                    at: crown + 0.25 * b * up,
                    axis: up,
                    field: Field::Stroke,
                    per_metre: 1.0,
                },
            ]
        }
        Part::Rod | Part::RodEye => {
            // the shank's top end (the wrist pin), along the shank
            let axis = p.model.transform_vector3(Vec3::Y).normalize_or_zero();
            let top = if p.part == Part::Rod {
                at(Vec3::new(0.0, 0.5, 0.0))
            } else {
                at(Vec3::ZERO)
            };
            vec![Handle {
                at: top,
                axis,
                field: Field::Rod,
                per_metre: 1.0,
            }]
        }
        Part::Runner => {
            let axis = p.model.transform_vector3(Vec3::Y).normalize_or_zero();
            vec![Handle {
                at: at(Vec3::Y),
                axis,
                field: Field::RunnerLength,
                per_metre: 1.0,
            }]
        }
        Part::Plenum => {
            // the top face: lifting it by h adds its area × h
            let (s, _, _) = p.model.to_scale_rotation_translation();
            vec![Handle {
                at: at(Vec3::new(0.0, 0.5, 0.0)),
                axis: Vec3::Y,
                field: Field::PlenumVolume,
                per_metre: (s.x * s.z) as f64,
            }]
        }
        Part::Pipe | Part::Joint => scene
            .pipes
            .iter()
            .find(|q| q.instances.contains(&selected) && q.pipe < def.cylinders)
            .and_then(|q| q.runs.last())
            .map(|r| {
                vec![Handle {
                    at: r.to,
                    axis: (r.to - r.from).normalize_or_zero(),
                    field: Field::PrimaryLength,
                    per_metre: 1.0,
                }]
            })
            .unwrap_or_default(),
        _ => Vec::new(),
    }
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

    #[test]
    fn a_piston_has_a_bore_and_a_stroke_handle_where_they_belong() {
        // inline-4 at TDC: crown r + l + 0.35 b over the crank, cylinder 1 at x = 0
        let d = EngineDef::inline4();
        let g = d.geometry;
        let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
        let (s, parts) = frame(&d);
        let k = parts.iter().position(|i| i.part == Part::Piston).unwrap();
        let h = handles(&d, &s, &parts, k);
        assert_eq!(h.len(), 2);
        let crown = Vec3::new(0.0, r + l + 0.35 * b, 0.0);
        assert!(
            (h[0].at - (crown + Vec3::new(0.0, 0.0, 0.62 * b))).length() < 1e-5,
            "{}",
            h[0].at
        );
        assert_eq!((h[0].field, h[0].per_metre), (Field::Bore, 2.0));
        assert!(
            (h[1].at - (crown + Vec3::new(0.0, 0.25 * b, 0.0))).length() < 1e-5,
            "{}",
            h[1].at
        );
        assert_eq!(h[1].field, Field::Stroke);
    }

    #[test]
    fn dragging_changes_the_value_by_its_travel_within_the_range() {
        let h = Handle {
            at: Vec3::ZERO,
            axis: Vec3::Z,
            field: Field::Bore,
            per_metre: 2.0,
        };
        // 50 px at 1000 px/m is 5 cm of travel: the bore grows 10 cm... clamped at 140 mm
        assert_eq!(dragged(&h, 0.086, 50.0, 1000.0), 0.140);
        // 2 px: 2 mm of travel, 4 mm more bore
        // (pixels are f32: 1e-9 is their precision here)
        assert!((dragged(&h, 0.086, 2.0, 1000.0) - 0.090).abs() < 1e-9);
        assert!((dragged(&h, 0.086, -2.0, 1000.0) - 0.082).abs() < 1e-9);
        assert_eq!(dragged(&h, 0.086, 5.0, 0.0), 0.086, "no scale: no change");
    }

    #[test]
    fn the_plenum_grows_by_its_top_area_per_metre() {
        let d = EngineDef::inline4();
        let (s, parts) = frame(&d);
        let k = parts.iter().position(|i| i.part == Part::Plenum).unwrap();
        let h = handles(&d, &s, &parts, k);
        let (sc, _, _) = parts[k].model.to_scale_rotation_translation();
        assert_eq!(h[0].field, Field::PlenumVolume);
        assert!((h[0].per_metre - (sc.x * sc.z) as f64).abs() < 1e-12);
    }

    #[test]
    fn a_primarys_handle_is_at_its_end_and_others_have_none() {
        let d = EngineDef::inline4();
        let (s, parts) = frame(&d);
        let p = &s.pipes[1];
        let h = handles(&d, &s, &parts, p.instances.start);
        assert_eq!(h.len(), 1);
        assert_eq!(h[0].at, p.runs.last().unwrap().to);
        assert_eq!(h[0].field, Field::PrimaryLength);
        let head = parts.iter().position(|i| i.part == Part::Head).unwrap();
        assert!(handles(&d, &s, &parts, head).is_empty());
    }

    #[test]
    fn set_clamps_to_the_editors_range() {
        let mut d = EngineDef::inline4();
        Field::Bore.set(&mut d, 0.5);
        assert_eq!(d.geometry.bore_m, 0.140);
        Field::PlenumVolume.set(&mut d, 0.0);
        assert_eq!(d.plenum_volume_m3, 0.3e-3);
        let before = d.exhaust.primary_length_m;
        Field::PrimaryLength.set(&mut d, f64::NAN);
        assert_eq!(d.exhaust.primary_length_m, before);
        assert_eq!(Field::Stroke.show(0.0855), "85.5 mm");
        assert_eq!(Field::PlenumVolume.show(4e-3), "4.00 L");
    }

    #[test]
    fn a_charge_or_ring_finds_its_own_cylinders_piston() {
        // cylinder 2's charge and its top ring both put their handles on cylinder 2 (x = pitch)
        let d = EngineDef::inline4();
        let pitch = crate::view3d::scene::PITCH_BORES as f32 * d.geometry.bore_m as f32;
        let (s, parts) = frame(&d);
        let charges: Vec<_> = (0..parts.len())
            .filter(|&i| parts[i].part == Part::Charge)
            .collect();
        let rings: Vec<_> = (0..parts.len())
            .filter(|&i| parts[i].part == Part::PistonRing)
            .collect();
        for k in [charges[1], rings[3]] {
            let h = handles(&d, &s, &parts, k);
            assert!((h[0].at.x - pitch).abs() < 1e-5, "{k}: {}", h[0].at);
        }
    }
}
