//! Clicking a part: a ray from the camera through the pointer, tested against every drawn part's
//! box (its unit mesh's bounds carried by its transform), the nearest hit chosen.

use super::scene::{Instance, MeshKind};
use glam::{Mat4, Vec3};

/// The bounds of each unit mesh in its own frame ([`super::mesh`]).
pub fn local_bounds(kind: MeshKind) -> (Vec3, Vec3) {
    match kind {
        MeshKind::Cylinder | MeshKind::Liner => {
            (Vec3::new(-1.0, 0.0, -1.0), Vec3::new(1.0, 1.0, 1.0))
        }
        MeshKind::Cuboid => (Vec3::splat(-0.5), Vec3::splat(0.5)),
        MeshKind::Sphere => (Vec3::splat(-1.0), Vec3::splat(1.0)),
        // the housing's outer wall reaches 1 + e + 0.22 < 1.6 of its generating radius
        MeshKind::Housing | MeshKind::HousingRing => {
            (Vec3::new(-1.6, 0.0, -1.6), Vec3::new(1.6, 1.0, 1.6))
        }
        MeshKind::Rotor => (Vec3::new(-1.0, 0.0, -1.0), Vec3::new(1.0, 1.0, 1.0)),
        // a web: the pin end to 1.45, the counterweight to −1.1
        MeshKind::Web => (Vec3::new(-1.1, 0.0, -1.1), Vec3::new(1.1, 1.0, 1.45)),
    }
}

/// The ray (origin, unit direction) through the point `ndc` (−1..1, y up) of a view with
/// `view_proj`: from the near plane to the far one.
pub fn ray(view_proj: Mat4, ndc: (f32, f32)) -> Option<(Vec3, Vec3)> {
    let inv = view_proj.inverse();
    if !inv.is_finite() {
        return None;
    }
    let near = inv.project_point3(Vec3::new(ndc.0, ndc.1, 0.0));
    let far = inv.project_point3(Vec3::new(ndc.0, ndc.1, 1.0));
    let d = (far - near).normalize_or_zero();
    (d != Vec3::ZERO).then_some((near, d))
}

/// Distance along the ray (origin, dir) to where it enters the box (lo, hi), if it does
/// (the slab method; a ray starting inside hits at 0).
fn hit_box(origin: Vec3, dir: Vec3, lo: Vec3, hi: Vec3) -> Option<f32> {
    let inv = dir.recip();
    let (t1, t2) = ((lo - origin) * inv, (hi - origin) * inv);
    let (tmin, tmax) = (t1.min(t2).max_element(), t1.max(t2).min_element());
    (tmax >= tmin.max(0.0)).then_some(tmin.max(0.0))
}

/// The nearest part the ray hits: its index in `parts` and its distance (world units).
pub fn pick(parts: &[Instance], origin: Vec3, dir: Vec3) -> Option<(usize, f32)> {
    let mut best: Option<(usize, f32)> = None;
    for (k, p) in parts.iter().enumerate() {
        let inv = p.model.inverse();
        if !inv.is_finite() {
            continue;
        }
        // into the part's own frame; distances there are scaled, so measure the hit back in
        // the world
        let (o, d) = (inv.transform_point3(origin), inv.transform_vector3(dir));
        let (lo, hi) = local_bounds(p.part.mesh());
        if let Some(t) = hit_box(o, d, lo, hi) {
            let world = (p.model.transform_point3(o + t * d) - origin).length();
            if best.is_none_or(|(_, b)| world < b) {
                best = Some((k, world));
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::view3d::scene::Part;

    fn cube_at(c: Vec3, side: f32) -> Instance {
        Instance {
            part: Part::Block,
            model: Mat4::from_scale_rotation_translation(
                Vec3::splat(side),
                glam::Quat::IDENTITY,
                c,
            ),
            color: [1.0; 4],
        }
    }

    #[test]
    fn the_nearer_of_two_boxes_on_the_ray_is_picked() {
        // two unit cubes on the −z axis at 3 and 6 m from an eye at the origin looking −z: the
        // first is entered 2.5 m away
        let parts = [
            cube_at(Vec3::new(0.0, 0.0, -6.0), 1.0),
            cube_at(Vec3::new(0.0, 0.0, -3.0), 1.0),
        ];
        let (k, t) = pick(&parts, Vec3::ZERO, Vec3::NEG_Z).unwrap();
        assert_eq!(k, 1);
        assert!((t - 2.5).abs() < 1e-5, "{t}");
    }

    #[test]
    fn a_ray_beside_every_box_picks_nothing() {
        let parts = [cube_at(Vec3::new(0.0, 0.0, -3.0), 1.0)];
        assert_eq!(pick(&parts, Vec3::new(0.6, 0.0, 0.0), Vec3::NEG_Z), None);
        // and behind the eye is not seen
        assert_eq!(pick(&parts, Vec3::ZERO, Vec3::Z), None);
    }

    #[test]
    fn a_stretched_cylinder_is_hit_where_its_surface_is() {
        // a cylinder of radius 0.2 from y = 0 to 2 (scale 0.2, 2, 0.2) at x = 1: a ray along
        // −x at y = 1 from x = 5 enters its bounds at x = 1.2, 3.8 m away
        let p = Instance {
            part: Part::Pipe,
            model: Mat4::from_scale_rotation_translation(
                Vec3::new(0.2, 2.0, 0.2),
                glam::Quat::IDENTITY,
                Vec3::new(1.0, 0.0, 0.0),
            ),
            color: [1.0; 4],
        };
        let (_, t) = pick(&[p], Vec3::new(5.0, 1.0, 0.0), Vec3::NEG_X).unwrap();
        assert!((t - 3.8).abs() < 1e-5, "{t}");
        assert_eq!(pick(&[p], Vec3::new(5.0, 2.5, 0.0), Vec3::NEG_X), None);
    }

    #[test]
    fn the_centre_ray_points_at_the_target() {
        let cam = super::super::camera::OrbitCamera {
            target: Vec3::new(0.1, 0.2, -0.3),
            yaw: 0.7,
            pitch: 0.3,
            distance: 2.0,
        };
        let (o, d) = ray(cam.view_proj(1.5), (0.0, 0.0)).unwrap();
        let to_target = (cam.target - o).normalize();
        assert!((d - to_target).length() < 1e-4, "{d} vs {to_target}");
    }
}
