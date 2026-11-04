//! Unit meshes the scene scales and places: a capped cylinder, a cube, and prisms of the rotary
//! engine's housing (its epitrochoid) and rotor. Built once per definition, never per frame.

use std::f32::consts::TAU;

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

impl Mesh {
    /// A mesh that draws nothing: one triangle of no area (a GPU buffer is never empty).
    pub fn nothing() -> Self {
        let v = Vertex {
            pos: [0.0; 3],
            normal: [0.0, 1.0, 0.0],
        };
        Self {
            vertices: vec![v; 3],
            indices: vec![0, 1, 2],
        }
    }

    fn push(&mut self, pos: [f32; 3], normal: [f32; 3]) -> u32 {
        self.vertices.push(Vertex { pos, normal });
        (self.vertices.len() - 1) as u32
    }

    /// A flat cap (fan from `centre`) through the ring `ring` at height `y`, facing `ny`.
    fn cap(&mut self, ring: &[(f32, f32)], y: f32, ny: f32) {
        let (cx, cz) = centroid(ring);
        let c = self.push([cx, y, cz], [0.0, ny, 0.0]);
        let first = self.vertices.len() as u32;
        for &(x, z) in ring {
            self.push([x, y, z], [0.0, ny, 0.0]);
        }
        for i in 0..ring.len() as u32 - 1 {
            let (a, b) = (first + i, first + i + 1);
            // counter-clockwise seen from the side the cap faces
            if ny > 0.0 {
                self.indices.extend([c, b, a]);
            } else {
                self.indices.extend([c, a, b]);
            }
        }
    }
}

fn centroid(ring: &[(f32, f32)]) -> (f32, f32) {
    let n = ring.len().max(1) as f32;
    let (sx, sz) = ring
        .iter()
        .fold((0.0, 0.0), |(a, b), &(x, z)| (a + x, b + z));
    (sx / n, sz / n)
}

/// Radius 1 about the y axis, y from 0 to 1, capped: the side shares smooth normals
/// (2·(segments + 1) vertices, the seam doubled), each cap is a fan (segments + 2 vertices).
pub fn cylinder(segments: u32) -> Mesh {
    let s = segments.max(3);
    let ring: Vec<(f32, f32)> = (0..=s)
        .map(|i| {
            let a = TAU * i as f32 / s as f32;
            (a.cos(), a.sin())
        })
        .collect();
    let mut m = Mesh::default();
    for &(x, z) in &ring {
        m.push([x, 0.0, z], [x, 0.0, z]);
        m.push([x, 1.0, z], [x, 0.0, z]);
    }
    for i in 0..s {
        let (b0, t0, b1, t1) = (2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3);
        m.indices.extend([b0, t0, b1, b1, t0, t1]);
    }
    m.cap(&ring, 0.0, -1.0);
    m.cap(&ring, 1.0, 1.0);
    m
}

/// Side 1 centred at the origin, flat normals (4 vertices per face).
pub fn cuboid() -> Mesh {
    let mut m = Mesh::default();
    for axis in 0..3 {
        for sign in [-1.0f32, 1.0] {
            let mut n = [0.0; 3];
            n[axis] = sign;
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            let first = m.vertices.len() as u32;
            for (du, dv) in [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)] {
                let mut p = [0.0; 3];
                p[axis] = 0.5 * sign;
                p[u] = du;
                p[v] = dv;
                m.push(p, n);
            }
            let q = [first, first + 1, first + 2, first, first + 2, first + 3];
            if sign > 0.0 {
                m.indices.extend(q);
            } else {
                m.indices.extend([q[0], q[2], q[1], q[3], q[5], q[4]]);
            }
        }
    }
    m
}

/// The closed outline `ring` (x, z; first point repeated last) extruded from y = 0 to 1, with
/// one flat normal per edge and fan caps (the outline must be star-shaped about its centroid and
/// run as (cos θ, sin θ) with θ growing).
fn prism(ring: &[(f32, f32)]) -> Mesh {
    let mut m = Mesh::default();
    for w in ring.windows(2) {
        let ((x0, z0), (x1, z1)) = (w[0], w[1]);
        let (dx, dz) = (x1 - x0, z1 - z0);
        let l = (dx * dx + dz * dz).sqrt().max(1e-12);
        // outward for an outline running as (cos θ, sin θ) with θ growing, as `cylinder`'s
        let n = [dz / l, 0.0, -dx / l];
        let a = m.push([x0, 0.0, z0], n);
        let b = m.push([x0, 1.0, z0], n);
        let c = m.push([x1, 0.0, z1], n);
        let d = m.push([x1, 1.0, z1], n);
        m.indices.extend([a, b, c, c, b, d]);
    }
    m.cap(ring, 0.0, -1.0);
    m.cap(ring, 1.0, 1.0);
    m
}

/// The epitrochoid (generating radius 1, eccentricity `e_over_r`) as a closed outline
/// running as (cos θ, sin θ) with θ growing: the same curve the 2D view draws
/// ([`crate::view::housing_point`]).
fn epitrochoid(e_over_r: f32, segments: u32) -> Vec<(f32, f32)> {
    let s = segments.max(8);
    (0..=s)
        .map(|i| {
            let (x, y) =
                crate::view::housing_point(TAU as f64 * i as f64 / s as f64, e_over_r as f64, 1.0);
            (x as f32, y as f32)
        })
        .collect()
}

/// A Wankel housing's side plate: the epitrochoid extruded (generating radius 1).
pub fn housing(e_over_r: f32, segments: u32) -> Mesh {
    prism(&epitrochoid(e_over_r, segments))
}

/// A Wankel housing's rotor housing: a wall 0.22 thick round the epitrochoid bore, open through,
/// so the rotor turning inside is seen.
pub fn housing_ring(e_over_r: f32, segments: u32) -> Mesh {
    let inner = epitrochoid(e_over_r, segments);
    let outer: Vec<(f32, f32)> = inner
        .iter()
        .map(|&(x, z)| {
            let r = (x * x + z * z).sqrt().max(1e-6);
            (x + 0.22 * x / r, z + 0.22 * z / r)
        })
        .collect();
    ring_prism(&outer, &inner)
}

/// The band between two closed outlines of the same point count (outer, inner; both running as
/// (cos θ, sin θ) with θ growing), extruded from y = 0 to 1: outer wall facing out, inner wall
/// facing the hole, flat faces at both ends.
fn ring_prism(outer: &[(f32, f32)], inner: &[(f32, f32)]) -> Mesh {
    let mut m = prism_walls(outer, 1.0);
    let walls = prism_walls(inner, -1.0);
    let base = m.vertices.len() as u32;
    m.vertices.extend(walls.vertices);
    m.indices.extend(walls.indices.iter().map(|i| i + base));
    for (y, ny) in [(0.0f32, -1.0f32), (1.0, 1.0)] {
        for (o, i) in outer.windows(2).zip(inner.windows(2)) {
            let n = [0.0, ny, 0.0];
            let a = m.push([o[0].0, y, o[0].1], n);
            let b = m.push([o[1].0, y, o[1].1], n);
            let c = m.push([i[0].0, y, i[0].1], n);
            let d = m.push([i[1].0, y, i[1].1], n);
            if ny > 0.0 {
                m.indices.extend([a, c, b, b, c, d]);
            } else {
                m.indices.extend([a, b, c, b, d, c]);
            }
        }
    }
    m
}

/// The walls of an extruded closed outline (no caps); `facing` 1 = outward, −1 = inward.
fn prism_walls(ring: &[(f32, f32)], facing: f32) -> Mesh {
    let mut m = Mesh::default();
    for w in ring.windows(2) {
        let ((x0, z0), (x1, z1)) = (w[0], w[1]);
        let (dx, dz) = (x1 - x0, z1 - z0);
        let l = (dx * dx + dz * dz).sqrt().max(1e-12);
        let n = [facing * dz / l, 0.0, -facing * dx / l];
        let a = m.push([x0, 0.0, z0], n);
        let b = m.push([x0, 1.0, z0], n);
        let c = m.push([x1, 0.0, z1], n);
        let d = m.push([x1, 1.0, z1], n);
        if facing > 0.0 {
            m.indices.extend([a, b, c, c, b, d]);
        } else {
            m.indices.extend([a, c, b, c, d, b]);
        }
    }
    m
}

/// A cylinder liner cut along its axis: the back half (z ≤ 0) of a tube of outer radius 1 and
/// inner 0.92, y from 0 to 1, so the piston in it stays in view.
pub fn half_liner(segments: u32) -> Mesh {
    let s = segments.max(4);
    let arc = |r: f32| -> Vec<(f32, f32)> {
        (0..=s)
            .map(|i| {
                let a = std::f32::consts::PI * (1.0 + i as f32 / s as f32);
                (r * a.cos(), r * a.sin())
            })
            .collect()
    };
    let mut m = prism_walls(&arc(1.0), 1.0);
    let inner = prism_walls(&arc(0.92), -1.0);
    let base = m.vertices.len() as u32;
    m.vertices.extend(inner.vertices);
    m.indices.extend(inner.indices.iter().map(|i| i + base));
    m
}

/// A ball of radius 1 about the origin: `segments` round, `segments / 2` from pole to pole,
/// smooth normals (the bends between a pipe's runs).
pub fn sphere(segments: u32) -> Mesh {
    let (s, t) = (segments.max(4), (segments / 2).max(2));
    let mut m = Mesh::default();
    for j in 0..=t {
        // the poles exact: sin π in f32 is 1e-7, enough to give the last row's slivers a
        // direction
        let (sv, cv) = match j {
            0 => (0.0, 1.0),
            _ if j == t => (0.0, -1.0),
            _ => (std::f32::consts::PI * j as f32 / t as f32).sin_cos(),
        };
        for i in 0..=s {
            let u = TAU * i as f32 / s as f32;
            let p = [sv * u.cos(), cv, sv * u.sin()];
            m.push(p, p);
        }
    }
    let row = s + 1;
    for j in 0..t {
        for i in 0..s {
            let (a, b) = (j * row + i, (j + 1) * row + i);
            m.indices.extend([a, a + 1, b, b, a + 1, b + 1]);
        }
    }
    m
}

/// A crank web of a throw of unit radius: a rounded end round the pin at (0, 1) (radius 0.45)
/// and a counterweight arc of radius 1.1 under the main journal at the origin, in (x, z),
/// extruded from y = 0 to 1.
pub fn crank_web(segments: u32) -> Mesh {
    let s = segments.max(4);
    let mut ring = Vec::with_capacity(2 * s as usize + 2);
    for i in 0..=s {
        let a = std::f32::consts::PI * i as f32 / s as f32;
        ring.push((0.45 * a.cos(), 1.0 + 0.45 * a.sin()));
    }
    let (a0, a1) = (std::f32::consts::PI + 0.6, TAU - 0.6);
    for i in 0..=s {
        let a = a0 + (a1 - a0) * i as f32 / s as f32;
        ring.push((1.1 * a.cos(), 1.1 * a.sin()));
    }
    ring.push(ring[0]);
    prism(&ring)
}

/// A rotor: a Reuleaux triangle of circumradius 1 (each flank an arc about the opposite apex),
/// extruded. The real flank is the epitrochoid's inner envelope, a little flatter; drawn shape
/// only, the sim's volumes come from the true geometry.
pub fn rotor(segments_per_flank: u32) -> Mesh {
    let s = segments_per_flank.max(2);
    let apex = |k: u32| {
        let a = TAU * k as f32 / 3.0;
        (a.cos(), a.sin())
    };
    let side = 3f32.sqrt();
    let mut ring = Vec::with_capacity(3 * s as usize + 1);
    for k in 0..3 {
        let (p0, p1, c) = (apex(k), apex(k + 1), apex(k + 2));
        let a0 = (p0.1 - c.1).atan2(p0.0 - c.0);
        let mut a1 = (p1.1 - c.1).atan2(p1.0 - c.0);
        // the short way round (60°), in the outline's direction
        if (a1 - a0).abs() > std::f32::consts::PI {
            a1 -= TAU * (a1 - a0).signum();
        }
        for i in 0..s {
            let a = a0 + (a1 - a0) * i as f32 / s as f32;
            ring.push((c.0 + side * a.cos(), c.1 + side * a.sin()));
        }
    }
    ring.push(ring[0]);
    prism(&ring)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all() -> Vec<Mesh> {
        vec![
            cylinder(16),
            cuboid(),
            housing(0.15, 64),
            housing_ring(0.15, 64),
            rotor(8),
            half_liner(16),
            crank_web(8),
            sphere(12),
        ]
    }

    #[test]
    fn cylinder_has_2_rings_and_2_caps() {
        // side 2·17 = 34, caps 2·18 = 36: 70 vertices; side 16·6 + caps 2·16·3 = 192 indices
        let m = cylinder(16);
        assert_eq!(m.vertices.len(), 70);
        assert_eq!(m.indices.len(), 192);
    }

    #[test]
    fn every_normal_is_unit() {
        for m in all() {
            for v in &m.vertices {
                let n = glam::Vec3::from(v.normal).length();
                assert!((n - 1.0).abs() < 1e-5, "{n}");
            }
        }
    }

    #[test]
    fn cuboid_bounds_are_half_unit() {
        let m = cuboid();
        assert_eq!((m.vertices.len(), m.indices.len()), (24, 36));
        for v in &m.vertices {
            assert!(v.pos.iter().all(|c| c.abs() == 0.5), "{v:?}");
        }
    }

    #[test]
    fn indices_stay_in_range() {
        for m in all() {
            assert_eq!(m.indices.len() % 3, 0);
            assert!(m.indices.iter().all(|&i| (i as usize) < m.vertices.len()));
        }
    }

    #[test]
    fn housing_major_axis_is_r_plus_e() {
        // x(α) = e·cos 3α + cos α: at α = 0, 1 + e; at α = 90°, (0, 1 − e) on the minor axis
        let m = housing(0.15, 64);
        let max_x = m.vertices.iter().map(|v| v.pos[0]).fold(f32::MIN, f32::max);
        assert!((max_x - 1.15).abs() < 1e-5, "{max_x}");
        let minor = m
            .vertices
            .iter()
            .find(|v| v.pos[0].abs() < 1e-5 && v.pos[2] > 0.0);
        assert!(
            minor.is_some_and(|v| (v.pos[2] - 0.85).abs() < 1e-5),
            "{minor:?}"
        );
    }

    #[test]
    fn rotor_apexes_sit_on_the_unit_circle() {
        let m = rotor(8);
        let r = m
            .vertices
            .iter()
            .map(|v| (v.pos[0] * v.pos[0] + v.pos[2] * v.pos[2]).sqrt())
            .fold(0.0f32, f32::max);
        assert!((r - 1.0).abs() < 1e-5, "{r}");
    }

    #[test]
    fn side_normals_point_outward() {
        // every side face's normal points away from the axis (dot with its position > 0)
        for m in [housing(0.15, 64), rotor(8), cylinder(12), crank_web(8)] {
            for v in m.vertices.iter().filter(|v| v.normal[1] == 0.0) {
                let d = v.pos[0] * v.normal[0] + v.pos[2] * v.normal[2];
                assert!(d > 0.0, "{v:?}");
            }
        }
    }

    #[test]
    fn faces_wind_counter_clockwise_seen_from_their_normal() {
        // (b − a) × (c − a) points along the vertices' normal for every triangle
        for m in all() {
            for t in m.indices.chunks(3) {
                let p = |i: u32| glam::Vec3::from(m.vertices[i as usize].pos);
                let n = (p(t[1]) - p(t[0])).cross(p(t[2]) - p(t[0]));
                if n.length() < 1e-9 {
                    continue;
                }
                let vn = glam::Vec3::from(m.vertices[t[0] as usize].normal);
                assert!(n.dot(vn) > 0.0, "{t:?}");
            }
        }
    }

    #[test]
    fn the_housing_ring_is_open_round_its_epitrochoid() {
        // the bore's wall lies on the epitrochoid (1 + e on the major axis), the outside 0.22
        // further out, and nothing in between near the centre: the rotor shows
        let m = housing_ring(0.15, 64);
        let r = |v: &Vertex| (v.pos[0] * v.pos[0] + v.pos[2] * v.pos[2]).sqrt();
        let min = m.vertices.iter().map(r).fold(f32::MAX, f32::min);
        let max = m.vertices.iter().map(r).fold(0.0f32, f32::max);
        assert!((min - 0.85).abs() < 2e-3, "{min}");
        assert!((max - 1.37).abs() < 1e-4, "{max}");
    }

    #[test]
    fn the_liner_is_only_its_back_half() {
        let m = half_liner(16);
        assert!(
            m.vertices.iter().all(|v| v.pos[2] <= 1e-6),
            "a vertex in front"
        );
        // the inside faces the bore's axis
        for v in m.vertices.iter().filter(|v| {
            let r = (v.pos[0] * v.pos[0] + v.pos[2] * v.pos[2]).sqrt();
            (r - 0.92).abs() < 1e-5
        }) {
            assert!(
                v.pos[0] * v.normal[0] + v.pos[2] * v.normal[2] < 0.0,
                "{v:?}"
            );
        }
    }

    #[test]
    fn a_web_reaches_round_the_pin_and_under_the_journal() {
        let m = crank_web(8);
        let top = m.vertices.iter().map(|v| v.pos[2]).fold(f32::MIN, f32::max);
        let bottom = m.vertices.iter().map(|v| v.pos[2]).fold(f32::MAX, f32::min);
        assert!((top - 1.45).abs() < 1e-5, "{top}");
        assert!((bottom + 1.1).abs() < 1e-3, "{bottom}");
    }
}
