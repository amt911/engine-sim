//! Handles on the clicked part: drag one along its axis and the number it stands for changes
//! (bore, stroke, rod, valves, a runner's or a primary's length and bore, the plenum, a
//! silencer), within the editor's ranges. The engine takes the new value once, on release (one
//! rebuild, as a slider).

use super::scene::{Instance, Part, Scene};
use engine_core::engine::EngineDef;
use engine_core::exhaust::Segment;
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
    RunnerDiameter,
    PrimaryDiameter,
    IntakeValveDiameter,
    ExhaustValveDiameter,
    IntakeLift,
    ExhaustLift,
    IntakeDuration,
    ExhaustDuration,
    IntakeOpens,
    ExhaustOpens,
    CompressionRatio,
    /// The silencer at this index of the exhaust's after-collector chain: its length…
    SilencerLength(u8),
    /// …and its shell's diameter.
    SilencerDiameter(u8),
}

/// The silencer at `i` of the after-collector chain: (length, shell diameter), if it is one.
fn can(d: &EngineDef, i: u8) -> Option<(f64, f64)> {
    match d.exhaust.after_collector.get(i as usize) {
        Some(&Segment::Muffler {
            length_m,
            diameter_m,
            ..
        }) => Some((length_m, diameter_m)),
        _ => None,
    }
}

fn can_mut(d: &mut EngineDef, i: u8) -> Option<(&mut f64, &mut f64)> {
    match d.exhaust.after_collector.get_mut(i as usize) {
        Some(Segment::Muffler {
            length_m,
            diameter_m,
            ..
        }) => Some((length_m, diameter_m)),
        _ => None,
    }
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
    /// Whether the drawn `parts[k]` is one this field reshapes or moves (what is tinted while
    /// it changes): the part kinds it changes, and for a silencer's field only its own can.
    pub fn changes(self, def: &EngineDef, parts: &[Instance], k: usize) -> bool {
        let Some(p) = parts.get(k) else { return false };
        let kind = p.part;
        match self {
            Field::Bore => matches!(
                kind,
                Part::Piston | Part::PistonRing | Part::Liner | Part::Charge
            ),
            Field::Stroke => matches!(
                kind,
                Part::Throw | Part::Crank | Part::Web | Part::Piston | Part::Charge
            ),
            Field::Rod => matches!(kind, Part::Rod | Part::RodEye),
            Field::CompressionRatio => matches!(kind, Part::Head | Part::Charge),
            Field::RunnerLength | Field::RunnerDiameter => {
                matches!(kind, Part::Runner | Part::Trumpet)
            }
            Field::PrimaryLength | Field::PrimaryDiameter => {
                matches!(kind, Part::Pipe | Part::Joint)
            }
            Field::PlenumVolume => kind == Part::Plenum,
            Field::IntakeValveDiameter => matches!(kind, Part::IntakeValve | Part::IntakeStem),
            Field::ExhaustValveDiameter => matches!(kind, Part::ExhaustValve | Part::ExhaustStem),
            Field::IntakeLift => matches!(
                kind,
                Part::IntakeValve | Part::IntakeStem | Part::IntakeSpring | Part::IntakeCam
            ),
            Field::ExhaustLift => matches!(
                kind,
                Part::ExhaustValve | Part::ExhaustStem | Part::ExhaustSpring | Part::ExhaustCam
            ),
            // timing moves the valve and turns its cam
            Field::IntakeDuration | Field::IntakeOpens => {
                matches!(kind, Part::IntakeValve | Part::IntakeStem | Part::IntakeCam)
            }
            Field::ExhaustDuration | Field::ExhaustOpens => matches!(
                kind,
                Part::ExhaustValve | Part::ExhaustStem | Part::ExhaustCam
            ),
            Field::SilencerLength(i) | Field::SilencerDiameter(i) => {
                silencer_at(def, parts, k) == Some(i)
            }
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Field::Bore => "Bore",
            Field::Stroke => "Stroke",
            Field::Rod => "Rod length",
            Field::RunnerLength => "Runner length",
            Field::PrimaryLength => "Primary length",
            Field::PlenumVolume => "Plenum volume",
            Field::RunnerDiameter => "Runner bore",
            Field::PrimaryDiameter => "Primary bore",
            Field::IntakeValveDiameter | Field::ExhaustValveDiameter => "Head diameter",
            Field::IntakeLift | Field::ExhaustLift => "Lift",
            Field::IntakeDuration | Field::ExhaustDuration => "Duration",
            Field::IntakeOpens | Field::ExhaustOpens => "Opens",
            Field::CompressionRatio => "Compression ratio",
            Field::SilencerLength(_) => "Silencer length",
            Field::SilencerDiameter(_) => "Shell diameter",
        }
    }

    /// The editor's range for it, SI (`editor_panel`: bore 40–140 mm, stroke 30–140, rod
    /// 60–260, runners 50–1000 mm long and 20–80 across, primaries 100–2000 mm long and 20–90
    /// across, plenum 0.3–15 L, valves 15–70 mm across with 3–18 mm of lift, open at 0–719° of
    /// the cycle for 120–360°, compression 6–16:1, silencers 50–1200 mm long in a 40–400 mm
    /// shell).
    pub fn range(self) -> (f64, f64) {
        match self {
            Field::Bore => (0.040, 0.140),
            Field::Stroke => (0.030, 0.140),
            Field::Rod => (0.060, 0.260),
            Field::RunnerLength => (0.050, 1.0),
            Field::PrimaryLength => (0.100, 2.0),
            Field::PlenumVolume => (0.3e-3, 15e-3),
            Field::RunnerDiameter => (0.020, 0.080),
            Field::PrimaryDiameter => (0.020, 0.090),
            Field::IntakeValveDiameter | Field::ExhaustValveDiameter => (0.015, 0.070),
            Field::IntakeLift | Field::ExhaustLift => (0.003, 0.018),
            Field::IntakeDuration | Field::ExhaustDuration => (120.0, 360.0),
            Field::IntakeOpens | Field::ExhaustOpens => (0.0, 719.0),
            Field::CompressionRatio => (6.0, 16.0),
            Field::SilencerLength(_) => (0.050, 1.2),
            Field::SilencerDiameter(_) => (0.040, 0.400),
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
            Field::RunnerDiameter => d.intake.runner_diameter_m,
            Field::PrimaryDiameter => d.exhaust.primary_diameter_m,
            Field::IntakeValveDiameter => d.intake_valve.diameter_m,
            Field::ExhaustValveDiameter => d.exhaust_valve.diameter_m,
            Field::IntakeLift => d.intake_valve.max_lift_m,
            Field::ExhaustLift => d.exhaust_valve.max_lift_m,
            Field::IntakeDuration => d.intake_valve.duration_deg,
            Field::ExhaustDuration => d.exhaust_valve.duration_deg,
            Field::IntakeOpens => d.intake_valve.open_deg,
            Field::ExhaustOpens => d.exhaust_valve.open_deg,
            Field::CompressionRatio => d.geometry.compression_ratio,
            // not a silencer (the chain was edited under it): nothing to show
            Field::SilencerLength(i) => can(d, i).map_or(f64::NAN, |c| c.0),
            Field::SilencerDiameter(i) => can(d, i).map_or(f64::NAN, |c| c.1),
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
            Field::RunnerDiameter => d.intake.runner_diameter_m = v,
            Field::PrimaryDiameter => d.exhaust.primary_diameter_m = v,
            Field::IntakeValveDiameter => d.intake_valve.diameter_m = v,
            Field::ExhaustValveDiameter => d.exhaust_valve.diameter_m = v,
            Field::IntakeLift => d.intake_valve.max_lift_m = v,
            Field::ExhaustLift => d.exhaust_valve.max_lift_m = v,
            Field::IntakeDuration => d.intake_valve.duration_deg = v,
            Field::ExhaustDuration => d.exhaust_valve.duration_deg = v,
            Field::IntakeOpens => d.intake_valve.open_deg = v,
            Field::ExhaustOpens => d.exhaust_valve.open_deg = v,
            Field::CompressionRatio => d.geometry.compression_ratio = v,
            Field::SilencerLength(i) => {
                if let Some(c) = can_mut(d, i) {
                    *c.0 = v;
                }
            }
            Field::SilencerDiameter(i) => {
                if let Some(c) = can_mut(d, i) {
                    *c.1 = v;
                }
            }
        }
    }

    /// How it is shown and typed: (editor units per SI unit, unit text, decimals).
    pub fn units(self) -> (f64, &'static str, usize) {
        match self {
            Field::PlenumVolume => (1000.0, " L", 2),
            Field::IntakeDuration
            | Field::ExhaustDuration
            | Field::IntakeOpens
            | Field::ExhaustOpens => (1.0, "°", 0),
            Field::CompressionRatio => (1.0, ":1", 1),
            _ => (1000.0, " mm", 1),
        }
    }

    /// The value for display, in the editor's units.
    pub fn show(self, v: f64) -> String {
        let (k, unit, dp) = self.units();
        format!("{:.*}{unit}", dp, k * v)
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
    match p.part.as_valve() {
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
                    axis: -up,
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
            vec![
                Handle {
                    at: at(Vec3::Y),
                    axis,
                    field: Field::RunnerLength,
                    per_metre: 1.0,
                },
                // the wall at mid-length moves by half the bore's change
                Handle {
                    at: at(Vec3::new(1.0, 0.5, 0.0)),
                    axis: p.model.transform_vector3(Vec3::X).normalize_or_zero(),
                    field: Field::RunnerDiameter,
                    per_metre: 2.0,
                },
            ]
        }
        Part::IntakeValve | Part::ExhaustValve => {
            // head and stem both stand on the valve face: its edge grows by half the diameter's
            // change, and the face goes down as the lift grows
            let (dia, lift) = if p.part.as_valve() == Part::IntakeValve {
                (Field::IntakeValveDiameter, Field::IntakeLift)
            } else {
                (Field::ExhaustValveDiameter, Field::ExhaustLift)
            };
            let face = at(Vec3::ZERO);
            let across = p.model.transform_vector3(Vec3::X).normalize_or_zero();
            let up = p.model.transform_vector3(Vec3::Y).normalize_or_zero();
            vec![
                Handle {
                    at: face + 0.2 * b * across,
                    axis: across,
                    field: dia,
                    per_metre: 2.0,
                },
                Handle {
                    at: face - 0.12 * b * up,
                    axis: -up,
                    field: lift,
                    per_metre: 1.0,
                },
            ]
        }
        Part::IntakeCam | Part::ExhaustCam => {
            // the cam turns in the bank's frame (its spring's: the cam's own turns with it):
            // the phase arrow is the tangent over the base circle. A later opening is the cam
            // turned back (half the crank's angle), towards -z, the way the cam turns for a
            // positive angle about the camshaft (x) being +y towards +z
            let (spring, field) = if p.part == Part::IntakeCam {
                (Part::IntakeSpring, Field::IntakeOpens)
            } else {
                (Part::ExhaustSpring, Field::ExhaustOpens)
            };
            let Some(frame) = (selected.saturating_sub(4)..selected)
                .rev()
                .find(|&i| parts[i].part == spring)
                .map(|i| &parts[i])
            else {
                return Vec::new();
            };
            let x = frame.model.x_axis.truncate().normalize_or_zero();
            let up = frame.model.y_axis.truncate().normalize_or_zero();
            let z = x.cross(up);
            let rho = 1.15 * super::parts::CAM_BASE_B * p.model.x_axis.truncate().length();
            if rho <= 0.0 {
                return Vec::new();
            }
            vec![Handle {
                at: p.model.w_axis.truncate() + up * rho,
                axis: -z,
                field,
                // the arrow's point goes round the cam at radius rho: s / rho rad of the cam is
                // 2 s / rho rad of the cycle
                per_metre: 360.0 / (std::f64::consts::PI * rho as f64),
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
        Part::Silencer => silencer_at(def, parts, selected).map_or(Vec::new(), |i| {
            // a straight can like a runner: its far end along it, its shell at mid-length
            vec![
                Handle {
                    at: at(Vec3::Y),
                    axis: p.model.transform_vector3(Vec3::Y).normalize_or_zero(),
                    field: Field::SilencerLength(i),
                    per_metre: 1.0,
                },
                Handle {
                    at: at(Vec3::new(1.0, 0.5, 0.0)),
                    axis: p.model.transform_vector3(Vec3::X).normalize_or_zero(),
                    field: Field::SilencerDiameter(i),
                    per_metre: 2.0,
                },
            ]
        }),
        Part::Pipe | Part::Joint => scene
            .pipes
            .iter()
            .find(|q| q.instances.contains(&selected) && q.pipe < def.cylinders)
            .and_then(|q| q.runs.last())
            .map(|r| {
                let along = (r.to - r.from).normalize_or_zero();
                // any direction square to the run: the wall at its middle
                let side = along.any_orthonormal_vector();
                let radius = 0.5 * def.exhaust.primary_diameter_m as f32;
                vec![
                    Handle {
                        at: r.to,
                        axis: along,
                        field: Field::PrimaryLength,
                        per_metre: 1.0,
                    },
                    Handle {
                        at: 0.5 * (r.from + r.to) + radius * side,
                        axis: side,
                        field: Field::PrimaryDiameter,
                        per_metre: 2.0,
                    },
                ]
            })
            .unwrap_or_default(),
        _ => Vec::new(),
    }
}

/// Give the engine `value` (SI) for `field` if the result is a valid engine and differs from
/// it; true when it took it (the engine is then rebuilt, the editor follows).
pub fn apply(edit: &mut EngineDef, field: Field, value: f64) -> bool {
    let mut next = edit.clone();
    field.set(&mut next, value);
    let take = engine_core::config::validate(&next).is_ok() && next != *edit;
    if take {
        *edit = next;
    }
    take
}

/// Which silencer of the after-collector chain `parts[k]` draws. Each branch lays that chain
/// out in order, one can per silencer (`ducts::chain`), then the merge lays its own: the
/// `j`-th can drawn is the chain's `j mod n`-th silencer, and the merge's cans (the last ones)
/// are not in it.
fn silencer_at(def: &EngineDef, parts: &[Instance], k: usize) -> Option<u8> {
    if parts.get(k)?.part != Part::Silencer {
        return None;
    }
    let is_can = |s: &Segment| matches!(s, Segment::Muffler { .. });
    let chain: Vec<usize> = (def.exhaust.after_collector.iter().enumerate())
        .filter(|(_, s)| is_can(s))
        .map(|(i, _)| i)
        .collect();
    let merged = def
        .exhaust
        .merge
        .as_ref()
        .map_or(0, |m| m.segments.iter().filter(|s| is_can(s)).count());
    let drawn = parts.iter().filter(|i| i.part == Part::Silencer).count();
    let j = parts[..k]
        .iter()
        .filter(|i| i.part == Part::Silencer)
        .count();
    if chain.is_empty() || j >= drawn.saturating_sub(merged) {
        return None;
    }
    u8::try_from(chain[j % chain.len()]).ok()
}

/// The numbers the card of `parts[k]` lets you type: its part's, and a silencer's own.
pub fn card_fields(def: &EngineDef, parts: &[Instance], k: usize) -> Vec<Field> {
    match silencer_at(def, parts, k) {
        Some(i) => vec![Field::SilencerLength(i), Field::SilencerDiameter(i)],
        None => parts
            .get(k)
            .map_or(Vec::new(), |p| editable(p.part).to_vec()),
    }
}

/// The numbers the card of a `part` lets you type, in the order it lists them.
pub fn editable(part: Part) -> &'static [Field] {
    match part.as_valve() {
        Part::Piston | Part::PistonRing | Part::Charge | Part::Liner => {
            &[Field::Bore, Field::Stroke, Field::CompressionRatio]
        }
        Part::Rod | Part::RodEye | Part::WristPin => &[Field::Rod],
        Part::IntakeValve => &[
            Field::IntakeValveDiameter,
            Field::IntakeLift,
            Field::IntakeDuration,
            Field::IntakeOpens,
        ],
        Part::ExhaustValve => &[
            Field::ExhaustValveDiameter,
            Field::ExhaustLift,
            Field::ExhaustDuration,
            Field::ExhaustOpens,
        ],
        Part::IntakeCam => &[Field::IntakeLift, Field::IntakeDuration, Field::IntakeOpens],
        Part::ExhaustCam => &[
            Field::ExhaustLift,
            Field::ExhaustDuration,
            Field::ExhaustOpens,
        ],
        Part::Runner | Part::Trumpet => &[Field::RunnerLength, Field::RunnerDiameter],
        Part::Plenum => &[Field::PlenumVolume],
        _ => &[],
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

    fn kinds(def: &EngineDef, f: Field) -> Vec<Part> {
        let (_, parts) = frame(def);
        let mut v: Vec<Part> = (0..parts.len())
            .filter(|&k| f.changes(def, &parts, k))
            .map(|k| parts[k].part)
            .collect();
        v.sort_by_key(|p| format!("{p:?}"));
        v.dedup();
        v
    }

    #[test]
    fn a_field_tints_the_parts_whose_shape_it_changes() {
        let d = EngineDef::inline4();
        assert_eq!(
            kinds(&d, Field::Bore),
            [Part::Charge, Part::Liner, Part::Piston, Part::PistonRing]
        );
        assert_eq!(kinds(&d, Field::Rod), [Part::Rod, Part::RodEye]);
        assert_eq!(kinds(&d, Field::PlenumVolume), [Part::Plenum]);
        assert_eq!(
            kinds(&d, Field::IntakeOpens),
            [Part::IntakeCam, Part::IntakeStem, Part::IntakeValve]
        );
        assert_eq!(
            kinds(&d, Field::ExhaustLift),
            [
                Part::ExhaustCam,
                Part::ExhaustSpring,
                Part::ExhaustStem,
                Part::ExhaustValve
            ]
        );
        assert_eq!(
            kinds(&d, Field::IntakeValveDiameter),
            [Part::IntakeStem, Part::IntakeValve]
        );
        assert!(kinds(&d, Field::CompressionRatio).contains(&Part::Head));
        assert!(kinds(&d, Field::PrimaryLength).contains(&Part::Pipe));
    }

    #[test]
    fn a_silencer_field_tints_only_its_own_can() {
        let d = two_cans();
        let (_, parts) = frame(&d);
        let n0 = (0..parts.len())
            .filter(|&k| Field::SilencerLength(1).changes(&d, &parts, k))
            .count();
        let n1 = (0..parts.len())
            .filter(|&k| Field::SilencerLength(3).changes(&d, &parts, k))
            .count();
        let cans = parts.iter().filter(|i| i.part == Part::Silencer).count();
        assert!(n0 >= 1 && n1 >= 1 && n0 + n1 <= cans, "{n0} {n1} of {cans}");
        assert!(n0 < cans && n1 < cans, "not every can");
    }

    #[test]
    fn a_cam_has_a_phase_arrow_across_its_lobe_that_moves_the_timing_the_right_way() {
        use crate::view3d::scene::moving;
        use engine_core::engine::CylinderView;
        let d = EngineDef::inline4();
        let (s, parts) = frame(&d);
        for (cam, field) in [
            (Part::IntakeCam, Field::IntakeOpens),
            (Part::ExhaustCam, Field::ExhaustOpens),
        ] {
            let k = parts.iter().position(|i| i.part == cam).unwrap();
            let h = handles(&d, &s, &parts, k);
            assert_eq!(h.len(), 1, "{cam:?}");
            assert_eq!(h[0].field, field);
            let b = d.geometry.bore_m as f32;
            let centre = parts[k].model.w_axis.truncate();
            let rho = 1.15 * 0.22 * b;
            // over the cam's base circle, its axis square to the camshaft (along the crank, x)
            assert!(((h[0].at - centre).length() - rho).abs() < 1e-5);
            assert!(h[0].axis.x.abs() < 1e-5 && (h[0].axis.length() - 1.0).abs() < 1e-5);
            assert!(h[0].axis.dot(h[0].at - centre).abs() < 1e-5, "a tangent");
            // the same engine 10 deg later on the cycle's timing: the cam, at a phase on its
            // base circle, has turned a half of it (5 deg), the handle's point travelling along
            // its axis, a tenth of a degree per ... per_metre degrees of timing a metre
            let mut later = d.clone();
            let cur = field.get(&later);
            field.set(&mut later, cur + 10.0);
            // (300 deg after the lobe's centre: the cam is on its base circle, turning with the
            // crank; the cylinders share the cycle angle)
            let v = if cam == Part::IntakeCam {
                &d.intake_valve
            } else {
                &d.exhaust_valve
            };
            let centre_deg = v.open_deg + 0.5 * v.duration_deg;
            let cyl: Vec<CylinderView> = (0..d.cylinders)
                .map(|_| CylinderView {
                    cycle_deg: (centre_deg + 300.0).rem_euclid(720.0),
                    piston_m: 0.0,
                    pressure_pa: 1e5,
                    temperature_k: 300.0,
                    intake_lift_m: 0.0,
                    exhaust_lift_m: 0.0,
                    burning: false,
                })
                .collect();
            let (mut a, mut c) = (Vec::new(), Vec::new());
            moving(&d, &cyl, &mut a);
            moving(&later, &cyl, &mut c);
            let at = |v: &[Instance]| v.iter().find(|i| i.part == cam).unwrap().model;
            let (m0, m1) = (at(&a), at(&c));
            let local = m0.inverse().transform_point3(h[0].at);
            let moved = m1.transform_point3(local) - h[0].at;
            // moved the way the arrow points, and by 10 deg of timing at per_metre
            let along = moved.dot(h[0].axis) as f64;
            assert!(along > 0.0, "{cam:?}: {moved}");
            assert!(
                (h[0].per_metre * along - 10.0).abs() < 0.1,
                "{cam:?}: {} deg for 10",
                h[0].per_metre * along
            );
        }
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
        assert_eq!(h.len(), 2, "its length and its bore");
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

    #[test]
    fn a_valve_has_a_diameter_and_a_lift_handle_at_its_face() {
        // inline-4, cylinder 1 at x = 0, both valves shut: the intake head's face at the deck,
        // 0.25 bore to the intake side; its diameter grows across (two edges), its lift down
        let d = EngineDef::inline4();
        let (s, parts) = frame(&d);
        let k = parts
            .iter()
            .position(|i| i.part == Part::IntakeValve)
            .unwrap();
        let h = handles(&d, &s, &parts, k);
        let face = parts[k].model.transform_point3(Vec3::ZERO);
        assert_eq!(h.len(), 2);
        assert_eq!(
            (h[0].field, h[0].per_metre),
            (Field::IntakeValveDiameter, 2.0)
        );
        assert!((h[0].at - face).length() > 0.0 && (h[0].at - face).y.abs() < 1e-6);
        assert_eq!((h[1].field, h[1].per_metre), (Field::IntakeLift, 1.0));
        assert!((h[1].axis - Vec3::NEG_Y).length() < 1e-6, "{}", h[1].axis);
        let k = parts
            .iter()
            .position(|i| i.part == Part::ExhaustValve)
            .unwrap();
        let fields: Vec<_> = handles(&d, &s, &parts, k).iter().map(|h| h.field).collect();
        assert_eq!(fields, [Field::ExhaustValveDiameter, Field::ExhaustLift]);
    }

    #[test]
    fn a_primary_and_a_runner_also_take_their_bore() {
        let d = EngineDef::inline4();
        let (s, parts) = frame(&d);
        let p = &s.pipes[0];
        let fields: Vec<_> = handles(&d, &s, &parts, p.instances.start)
            .iter()
            .map(|h| (h.field, h.per_metre))
            .collect();
        assert_eq!(
            fields,
            [(Field::PrimaryLength, 1.0), (Field::PrimaryDiameter, 2.0)]
        );
        if let Some(k) = parts.iter().position(|i| i.part == Part::Runner) {
            let fields: Vec<_> = handles(&d, &s, &parts, k).iter().map(|h| h.field).collect();
            assert_eq!(fields, [Field::RunnerLength, Field::RunnerDiameter]);
        }
    }

    #[test]
    fn the_card_edits_what_the_part_is_made_of() {
        assert_eq!(
            editable(Part::ExhaustValve),
            [
                Field::ExhaustValveDiameter,
                Field::ExhaustLift,
                Field::ExhaustDuration,
                Field::ExhaustOpens
            ]
        );
        assert_eq!(
            editable(Part::Piston),
            [Field::Bore, Field::Stroke, Field::CompressionRatio]
        );
        assert_eq!(
            editable(Part::IntakeCam),
            [Field::IntakeLift, Field::IntakeDuration, Field::IntakeOpens]
        );
        assert_eq!(editable(Part::ExhaustCam)[2], Field::ExhaustOpens);
        assert!(editable(Part::Block).is_empty());
    }

    /// Pipe, can, pipe, can behind the collectors, no merge: the cans are segments 1 and 3.
    fn two_cans() -> EngineDef {
        use engine_core::exhaust::Segment;
        let can = |length_m, diameter_m| Segment::Muffler {
            length_m,
            diameter_m,
            tube_diameter_m: 0.05,
            packing_resistivity: 16_000.0,
            packing_porosity: 0.95,
        };
        let pipe = Segment::Pipe {
            length_m: 0.3,
            diameter_m: 0.05,
        };
        let mut d = EngineDef::inline4();
        d.exhaust.after_collector = vec![pipe.clone(), can(0.35, 0.12), pipe, can(0.5, 0.2)];
        d.exhaust.merge = None;
        d
    }

    #[test]
    fn a_silencer_takes_its_length_and_its_shell() {
        let d = two_cans();
        let (s, parts) = frame(&d);
        let cans: Vec<_> = (0..parts.len())
            .filter(|&k| parts[k].part == Part::Silencer)
            .collect();
        // every branch draws both cans, in the chain's order
        assert!(!cans.is_empty() && cans.len() % 2 == 0, "{}", cans.len());
        for (j, &k) in cans.iter().enumerate() {
            let seg = if j % 2 == 0 { 1 } else { 3 };
            let fields: Vec<_> = handles(&d, &s, &parts, k)
                .iter()
                .map(|h| (h.field, h.per_metre))
                .collect();
            assert_eq!(
                fields,
                [
                    (Field::SilencerLength(seg), 1.0),
                    (Field::SilencerDiameter(seg), 2.0)
                ]
            );
            assert_eq!(
                card_fields(&d, &parts, k),
                [Field::SilencerLength(seg), Field::SilencerDiameter(seg)]
            );
        }
        // the second can's end stands 0.5 m along it from where it starts
        let h = handles(&d, &s, &parts, cans[1]);
        let start = parts[cans[1]].model.transform_point3(Vec3::ZERO);
        assert!((h[0].at.distance(start) - 0.5).abs() < 1e-5, "{}", h[0].at);
        // a piston's card is the part's list
        let piston = parts.iter().position(|i| i.part == Part::Piston).unwrap();
        assert_eq!(card_fields(&d, &parts, piston), editable(Part::Piston));
    }

    #[test]
    fn the_merges_own_can_is_not_one_of_the_chains() {
        let mut d = two_cans();
        let chain_can = d.exhaust.after_collector[1].clone();
        d.exhaust.merge = Some(engine_core::exhaust::Merge {
            segments: vec![chain_can],
            outlet_diameter_m: 0.06,
            bypass: None,
        });
        let (s, parts) = frame(&d);
        let cans: Vec<_> = (0..parts.len())
            .filter(|&k| parts[k].part == Part::Silencer)
            .collect();
        let (&last, branch) = cans.split_last().unwrap();
        assert!(handles(&d, &s, &parts, last).is_empty());
        assert!(card_fields(&d, &parts, last).is_empty());
        // the branches' cans still map to the chain
        assert_eq!(
            card_fields(&d, &parts, branch[branch.len() - 1]),
            [Field::SilencerLength(3), Field::SilencerDiameter(3)]
        );
    }

    #[test]
    fn a_silencer_field_edits_its_own_segment_only() {
        use engine_core::exhaust::Segment;
        let mut d = two_cans();
        // the editor's silencer: 50–1200 mm long, a 40–400 mm shell
        Field::SilencerLength(3).set(&mut d, 2.0);
        Field::SilencerDiameter(1).set(&mut d, 0.01);
        assert_eq!(Field::SilencerLength(3).get(&d), 1.2);
        assert_eq!(Field::SilencerDiameter(1).get(&d), 0.04);
        assert_eq!(Field::SilencerLength(1).get(&d), 0.35, "the other can kept");
        assert_eq!(Field::SilencerLength(3).show(1.2), "1200.0 mm");
        // a field pointing at a plain pipe or past the chain changes nothing
        let before = d.clone();
        Field::SilencerLength(0).set(&mut d, 0.9);
        Field::SilencerDiameter(9).set(&mut d, 0.1);
        assert_eq!(d, before);
        assert!(!apply(&mut d, Field::SilencerLength(0), 0.9));
        assert!(matches!(
            d.exhaust.after_collector[0],
            Segment::Pipe { length_m: 0.3, .. }
        ));
        assert!(apply(&mut d, Field::SilencerLength(1), 0.6));
        assert_eq!(Field::SilencerLength(1).get(&d), 0.6);
    }

    #[test]
    fn timing_and_compression_show_in_their_own_units() {
        let mut d = EngineDef::inline4();
        Field::IntakeOpens.set(&mut d, 800.0);
        assert_eq!(d.intake_valve.open_deg, 719.0);
        assert_eq!(Field::ExhaustDuration.show(248.4), "248°");
        assert_eq!(Field::CompressionRatio.show(11.04), "11.0:1");
        assert_eq!(Field::IntakeLift.show(0.0105), "10.5 mm");
    }

    #[test]
    fn an_edit_is_taken_only_when_it_changes_a_valid_engine() {
        let mut d = EngineDef::inline4();
        assert!(apply(&mut d, Field::ExhaustDuration, 250.0));
        assert_eq!(d.exhaust_valve.duration_deg, 250.0);
        assert!(!apply(&mut d, Field::ExhaustDuration, 250.0), "no change");
        // a 60 mm rod on a 140 mm stroke is shorter than the throw (70 mm): refused
        d.geometry.stroke_m = 0.140;
        let before = d.clone();
        assert!(!apply(&mut d, Field::Rod, 0.060));
        assert_eq!(d, before);
    }
}
