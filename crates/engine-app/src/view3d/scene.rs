//! The engine's parts as placed unit meshes: the still ones built once from the definition, the
//! moving ones refilled every frame from the cylinders' state. Same proportions as the 2D view
//! (`view::cylinder`): piston 0.6 bore tall with its crown 0.35 bore over the pin, head 0.35
//! bore, valves at ±0.25 bore across the bore.
//!
//! World frame: the crank along +x, cylinders 1.2 bores apart; a bank's frame is the world turned
//! about x by half the bank angle (bank 0 to −, bank 1 to +), so its cylinder axis is
//! (0, cos t, sin t). A rotary's shaft is along x too, each rotor in the y–z plane.

use engine_core::engine::{CylinderView, EngineDef};
use glam::{Mat3, Mat4, Quat, Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Part {
    Block,
    Head,
    Crank,
    Rod,
    Piston,
    IntakeValve,
    ExhaustValve,
    Housing,
    Rotor,
}

impl Part {
    /// Which unit mesh draws it.
    pub fn mesh(self) -> MeshKind {
        match self {
            Part::Block | Part::Head => MeshKind::Cuboid,
            Part::Housing => MeshKind::Housing,
            Part::Rotor => MeshKind::Rotor,
            _ => MeshKind::Cylinder,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MeshKind {
    Cylinder,
    Cuboid,
    Housing,
    Rotor,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Instance {
    pub part: Part,
    pub model: Mat4,
    pub color: [f32; 4],
}

#[derive(Debug, Clone, PartialEq)]
pub struct Scene {
    pub static_parts: Vec<Instance>,
    /// Centre and radius of a sphere holding every part in every position.
    pub bounds: (Vec3, f32),
}

/// Cylinder pitch along the crank, bores (a drawing choice: the file has no bore spacing).
pub const PITCH_BORES: f64 = 1.2;

const STEEL: [f32; 4] = [0.59, 0.61, 0.65, 1.0];
const DARK: [f32; 4] = [0.24, 0.24, 0.27, 1.0];
const LIGHT: [f32; 4] = [0.78, 0.79, 0.82, 1.0];
const INTAKE: [f32; 4] = [0.31, 0.59, 1.0, 1.0];
const EXHAUST: [f32; 4] = [1.0, 0.43, 0.31, 1.0];

/// Turn of each bank about the crank, rad.
fn bank_tilt(def: &EngineDef, bank: usize) -> f32 {
    if def.layout.bank_angle_deg <= 0.0 {
        return 0.0;
    }
    let h = (0.5 * def.layout.bank_angle_deg.clamp(0.0, 180.0)).to_radians() as f32;
    if bank == 0 { -h } else { h }
}

/// A unit cylinder (radius 1, y 0..1) stretched from `a` to `b` with `radius`.
fn rod_between(a: Vec3, b: Vec3, radius: f32) -> Mat4 {
    let d = b - a;
    let len = d.length().max(1e-9);
    let q = Quat::from_rotation_arc(Vec3::Y, d / len);
    Mat4::from_scale_rotation_translation(Vec3::new(radius, len, radius), q, a)
}

/// Rotary: maps a rotor's drawing plane (the 2D view's x, y) to the world's (z, y), the mesh's
/// extrusion axis to x.
fn rotary_frame() -> Mat3 {
    Mat3::from_cols(Vec3::Z, Vec3::X, Vec3::Y)
}

/// The still parts: block and head per bank (the block as its back wall, so the moving parts in
/// front stay visible until the cut-away layer), the main shaft; a rotary's housings.
pub fn build(def: &EngineDef) -> Scene {
    let g = def.geometry;
    let mut out = Vec::new();
    if let Some(rotor) = g.rotor {
        let (e, r, w) = (
            rotor.eccentricity_m as f32,
            rotor.generating_radius_m as f32,
            rotor.width_m as f32,
        );
        let rotors = def.cylinders.div_ceil(3).max(1);
        for k in 0..rotors {
            let x = k as f32 * 1.5 * w;
            // the housing mesh has generating radius 1 and e/R baked in: scale by R
            let m = Mat4::from_translation(Vec3::new(x - 0.05 * w, 0.0, 0.0))
                * Mat4::from_mat3(rotary_frame())
                * Mat4::from_scale(Vec3::new(r, 1.1 * w, r));
            out.push(Instance {
                part: Part::Housing,
                model: m,
                color: STEEL,
            });
        }
        let len = rotors as f32 * 1.5 * w;
        out.push(Instance {
            part: Part::Crank,
            model: rod_between(
                Vec3::new(-0.5 * w, 0.0, 0.0),
                Vec3::new(len, 0.0, 0.0),
                0.3 * e.max(0.1 * r),
            ),
            color: DARK,
        });
        let bounds = (
            Vec3::new(0.5 * len, 0.0, 0.0),
            (0.5 * len).hypot(r + e) * 1.05,
        );
        return Scene {
            static_parts: out,
            bounds,
        };
    }
    let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
    let clear_h = (g.clearance_volume() / g.piston_area()) as f32;
    let deck = r + l + 0.35 * b + clear_h;
    let pitch = PITCH_BORES as f32 * b;
    let place = def.layout.place(def.cylinders);
    let banks = place.iter().map(|p| p.0 + 1).max().unwrap_or(1);
    let cols = place.iter().map(|p| p.1 + 1).max().unwrap_or(1);
    let span = (cols as f32 - 1.0) * pitch;
    for bank in 0..banks {
        let n = place.iter().filter(|p| p.0 == bank).count();
        let len = (n.max(1) as f32 - 1.0) * pitch + b;
        let x0 = -0.5 * b;
        let rot = Mat4::from_rotation_x(bank_tilt(def, bank));
        let skirt = l - r - 0.25 * b;
        // the back wall: behind the bores, from the skirt to the deck
        let wall =
            Mat4::from_translation(Vec3::new(x0 + 0.5 * len, 0.5 * (skirt + deck), -0.65 * b))
                * Mat4::from_scale(Vec3::new(len + 0.2 * b, deck - skirt, 0.1 * b));
        out.push(Instance {
            part: Part::Block,
            model: rot * wall,
            color: STEEL,
        });
        let head = Mat4::from_translation(Vec3::new(x0 + 0.5 * len, deck + 0.175 * b, 0.0))
            * Mat4::from_scale(Vec3::new(len + 0.2 * b, 0.35 * b, 1.3 * b));
        out.push(Instance {
            part: Part::Head,
            model: rot * head,
            color: DARK,
        });
    }
    out.push(Instance {
        part: Part::Crank,
        model: rod_between(
            Vec3::new(-0.7 * b, 0.0, 0.0),
            Vec3::new(span + 0.7 * b, 0.0, 0.0),
            0.15 * b,
        ),
        color: DARK,
    });
    let reach = deck + 0.35 * b;
    let radius = (0.5 * span + 0.7 * b).hypot(reach) * 1.05;
    Scene {
        static_parts: out,
        bounds: (Vec3::new(0.5 * span, 0.0, 0.0), radius),
    }
}

/// The moving parts for the cylinders' current state, written into `out` (cleared first; it
/// keeps its capacity, so a steady frame allocates nothing).
pub fn moving(def: &EngineDef, cylinders: &[CylinderView], out: &mut Vec<Instance>) {
    out.clear();
    let g = def.geometry;
    if let Some(rotor) = g.rotor {
        let (e, r, w) = (
            rotor.eccentricity_m,
            rotor.generating_radius_m,
            rotor.width_m,
        );
        for (k, chambers) in cylinders.chunks(3).enumerate() {
            let Some(first) = chambers.first() else {
                continue;
            };
            let (c, apex) = crate::view::rotor_pose(first.cycle_deg, e, r);
            let theta = (apex[0].1 - c.1).atan2(apex[0].0 - c.0) as f32;
            let centre = rotary_frame() * Vec3::new(c.0 as f32, 0.0, c.1 as f32);
            let m = Mat4::from_translation(Vec3::new(k as f32 * 1.5 * w as f32, 0.0, 0.0) + centre)
                * Mat4::from_mat3(rotary_frame())
                * Mat4::from_rotation_y(-theta)
                * Mat4::from_scale(Vec3::new(r as f32, w as f32, r as f32));
            out.push(Instance {
                part: Part::Rotor,
                model: m,
                color: LIGHT,
            });
        }
        return;
    }
    let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
    let clear_h = (g.clearance_volume() / g.piston_area()) as f32;
    let deck = r + l + 0.35 * b + clear_h;
    let pitch = PITCH_BORES as f32 * b;
    let place = def.layout.place(def.cylinders);
    for (i, c) in cylinders.iter().enumerate() {
        let Some(&(bank, pos)) = place.get(i) else {
            break;
        };
        let rot = Mat4::from_rotation_x(bank_tilt(def, bank));
        let x = pos as f32 * pitch;
        let th = (c.cycle_deg as f32).to_radians();
        // in the bank's frame: y up the bore, z across it
        let pin = Vec3::new(x, r * th.cos(), r * th.sin());
        let wrist = Vec3::new(x, r + l - c.piston_m as f32, 0.0);
        let at = |m: Mat4| rot * m;
        out.push(Instance {
            part: Part::Piston,
            model: at(
                Mat4::from_translation(Vec3::new(x, wrist.y + 0.35 * b - 0.6 * b, 0.0))
                    * Mat4::from_scale(Vec3::new(0.49 * b, 0.6 * b, 0.49 * b)),
            ),
            color: LIGHT,
        });
        out.push(Instance {
            part: Part::Rod,
            model: at(rod_between(pin, wrist, 0.08 * b)),
            color: STEEL,
        });
        out.push(Instance {
            part: Part::Crank,
            model: at(rod_between(
                pin - Vec3::new(0.25 * b, 0.0, 0.0),
                pin + Vec3::new(0.25 * b, 0.0, 0.0),
                0.12 * b,
            )),
            color: DARK,
        });
        for (dz, lift, part, color) in [
            (-0.25, c.intake_lift_m, Part::IntakeValve, INTAKE),
            (0.25, c.exhaust_lift_m, Part::ExhaustValve, EXHAUST),
        ] {
            let face = deck - lift as f32;
            let z = dz * b;
            // the head (disc) and its stem up through the head
            out.push(Instance {
                part,
                model: at(Mat4::from_translation(Vec3::new(x, face, z))
                    * Mat4::from_scale(Vec3::new(0.17 * b, 0.04 * b, 0.17 * b))),
                color,
            });
            out.push(Instance {
                part,
                model: at(Mat4::from_translation(Vec3::new(x, face, z))
                    * Mat4::from_scale(Vec3::new(0.03 * b, 0.5 * b, 0.03 * b))),
                color,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn views(def: &EngineDef, cycle_deg: impl Fn(usize) -> f64) -> Vec<CylinderView> {
        (0..def.cylinders)
            .map(|i| {
                let d = cycle_deg(i);
                CylinderView {
                    cycle_deg: d,
                    piston_m: def.geometry.piston_position(d.to_radians()),
                    pressure_pa: 1e5,
                    temperature_k: 300.0,
                    intake_lift_m: 0.0,
                    exhaust_lift_m: 0.0,
                    burning: false,
                }
            })
            .collect()
    }

    fn of(out: &[Instance], part: Part) -> Vec<Mat4> {
        out.iter()
            .filter(|i| i.part == part)
            .map(|i| i.model)
            .collect()
    }

    #[test]
    fn inline4_cylinders_are_1_2_bores_apart() {
        let d = EngineDef::inline4();
        let b = d.geometry.bore_m as f32;
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 0.0), &mut out);
        let xs: Vec<f32> = of(&out, Part::Piston).iter().map(|m| m.w_axis.x).collect();
        assert_eq!(xs.len(), 4);
        for (k, x) in xs.iter().enumerate() {
            assert!((x - 1.2 * b * k as f32).abs() < 1e-6, "{xs:?}");
        }
    }

    #[test]
    fn a_piston_at_tdc_has_its_crown_r_plus_l_plus_0_35_bore_over_the_crank() {
        // the piston mesh's base sits 0.6 b under the crown: base y = r + l + 0.35b − 0.6b
        let d = EngineDef::inline4();
        let g = d.geometry;
        let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 0.0), &mut out);
        let y = of(&out, Part::Piston)[0].w_axis.y;
        assert!((y - (r + l - 0.25 * b)).abs() < 1e-6, "{y}");
        // at 180° it is a stroke lower
        moving(&d, &views(&d, |_| 180.0), &mut out);
        let y2 = of(&out, Part::Piston)[0].w_axis.y;
        assert!((y - y2 - 2.0 * r).abs() < 1e-6, "{y} {y2}");
    }

    #[test]
    fn v8_banks_lean_45_degrees() {
        // 90° V: bank 0's axis is (0, cos 45°, −sin 45°); its piston base at TDC lies on it
        let d = EngineDef::crossplane_v8();
        assert_eq!(d.layout.bank_angle_deg, 90.0);
        let g = d.geometry;
        let (r, l, b) = (g.crank_radius() as f32, g.rod_m as f32, g.bore_m as f32);
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 0.0), &mut out);
        let place = d.layout.place(d.cylinders);
        let pistons = of(&out, Part::Piston);
        let h = r + l - 0.25 * b;
        let s = std::f32::consts::FRAC_1_SQRT_2;
        for (k, m) in pistons.iter().enumerate() {
            let sign = if place[k].0 == 0 { -1.0 } else { 1.0 };
            assert!((m.w_axis.y - h * s).abs() < 1e-6, "{k}: {}", m.w_axis);
            assert!(
                (m.w_axis.z - sign * h * s).abs() < 1e-6,
                "{k}: {}",
                m.w_axis
            );
        }
    }

    #[test]
    fn a_rod_runs_from_its_pin_to_its_wrist() {
        // at 90° the pin is r across the bore (z = r) at the crank's height
        let d = EngineDef::inline4();
        let r = d.geometry.crank_radius() as f32;
        let mut out = Vec::new();
        moving(&d, &views(&d, |_| 90.0), &mut out);
        let rod = of(&out, Part::Rod)[0];
        let base = rod.w_axis.truncate();
        assert!((base - Vec3::new(0.0, 0.0, r)).length() < 1e-6, "{base}");
    }

    #[test]
    fn bounds_cover_every_part() {
        for d in [
            EngineDef::single(),
            EngineDef::inline4(),
            EngineDef::lfa_v10(),
            EngineDef::mazda_renesis(),
            EngineDef::crossplane_v8(),
        ] {
            let s = build(&d);
            let mut out = Vec::new();
            for deg in [0.0, 90.0, 180.0, 270.0, 540.0] {
                moving(&d, &views(&d, |_| deg), &mut out);
                for i in s.static_parts.iter().chain(&out) {
                    let p = i.model.w_axis.truncate();
                    assert!(
                        (p - s.bounds.0).length() <= s.bounds.1,
                        "{}: {p} out of {:?}",
                        d.name,
                        s.bounds
                    );
                }
            }
        }
    }

    #[test]
    fn rotary_has_housing_and_rotor_not_pistons() {
        let d = EngineDef::mazda_renesis();
        let s = build(&d);
        let mut out = Vec::new();
        moving(&d, &views(&d, |i| 240.0 * i as f64), &mut out);
        assert_eq!(of(&s.static_parts, Part::Housing).len(), 2);
        assert_eq!(of(&out, Part::Rotor).len(), 2);
        assert!(of(&out, Part::Piston).is_empty());
    }

    #[test]
    fn a_rotor_centre_rides_its_eccentric() {
        // the rotor's centre is e from the shaft (the y–z plane), wherever the shaft is
        let d = EngineDef::mazda_renesis();
        let e = d.geometry.rotor.unwrap().eccentricity_m as f32;
        let mut out = Vec::new();
        for deg in [0.0, 100.0, 333.0] {
            moving(&d, &views(&d, |_| deg), &mut out);
            let c = of(&out, Part::Rotor)[0].w_axis;
            assert!((c.y.hypot(c.z) - e).abs() < 1e-6, "{deg}: {c}");
        }
    }

    #[test]
    fn moving_does_not_reallocate() {
        // a buffer with room keeps it: same capacity, same storage (a fresh or shrunk Vec
        // would not keep 1000)
        let d = EngineDef::lfa_v10();
        let mut out = Vec::with_capacity(1000);
        let ptr = out.as_ptr();
        for deg in [0.0, 90.0] {
            moving(&d, &views(&d, |_| deg), &mut out);
            assert!(!out.is_empty());
            assert_eq!((out.capacity(), out.as_ptr()), (1000, ptr));
        }
    }
}
