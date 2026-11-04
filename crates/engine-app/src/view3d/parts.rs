//! The realistic parts of `assets/3d-parts/` (Blender-made, glTF): read once, stretched to the
//! engine by the rules of that folder's README and expressed in the unit frame of the procedural
//! part each one replaces, so the instance matrices, picking, handles and cards stay as they are.
//! The shader takes normals through the inverse transpose (it divides by the scale squared), so a
//! vertex moved into a frame scaled by `S` takes its normal multiplied by `S`.

use super::mesh::{Mesh, Vertex};
use glam::{Mat4, Vec3};

/// The piston, its three rings and its wrist pin (origin: the wrist pin's centre, reference bore
/// 86 mm).
const PISTON_GLB: &[u8] = include_bytes!("../../../../assets/3d-parts/out/piston.glb");
/// The connecting rod with its cap, bolts, shells and bush (origin: the big end's centre, the
/// small end at +Y, `stretch` weights 0 at the big end to 1 at the small end).
const CONROD_GLB: &[u8] = include_bytes!("../../../../assets/3d-parts/out/conrod.glb");
/// The reference rod's length, m.
const REF_ROD: f32 = 0.143;
/// One throw of the crankshaft: its pin, both webs with their counterweights and the main
/// journals either side (origin: the crank axis at the cylinder's plane, the pin at +Y; `stretch`
/// weights 1 on the pin, 0 on the mains and the counterweight).
const CRANK_GLB: &[u8] = include_bytes!("../../../../assets/3d-parts/out/crank_throw.glb");
/// The reference throw (half the 86 mm stroke) and counterweight radius, in bores.
const REF_THROW_B: f32 = 0.5;
const COUNTERWEIGHT_B: f32 = 0.74;
/// A valve with its keepers and retainer (origin: the face's centre, closed, stem +Y; `stretch`
/// weights 0 on the head, 1 on the tip, keepers and retainer).
const VALVE_GLB: &[u8] = include_bytes!("../../../../assets/3d-parts/out/valve.glb");
/// The reference stem, face to tip, m (README) and the scene's, in bores (the head's box).
const REF_STEM: f32 = 0.0989;
const SCENE_STEM_B: f32 = 0.5;
/// The Wankel rotor with its seals, gear and bearing (real size: e 15, R 105, width 80 mm, the
/// RENESIS; its plane x–z, apex 0 on +x, centred in y: the frame of `mesh::rotor`).
const ROTOR_GLB: &[u8] = include_bytes!("../../../../assets/3d-parts/out/wankel_rotor.glb");
const REF_E: f32 = 0.015;
const REF_R: f32 = 0.105;
const REF_W: f32 = 0.080;
/// The reference engine's bore, m (README: bore 86, stroke 86, rod 143 mm).
const REF_BORE: f32 = 0.086;

/// A part as read: its vertices (in the file's frame, node transforms applied), the weight of
/// its `stretch` joint on each (0 without a skin), its morph target 0's offsets (empty without
/// one) and its triangles.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GlbPart {
    pub positions: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub stretch_w: Vec<f32>,
    pub morph0: Vec<Vec3>,
    pub indices: Vec<u32>,
}

#[derive(Debug)]
pub enum PartError {
    Gltf(gltf::Error),
    NoBinary,
    Missing(&'static str),
}

impl std::fmt::Display for PartError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PartError::Gltf(e) => write!(f, "glTF: {e}"),
            PartError::NoBinary => write!(f, "no binary chunk"),
            PartError::Missing(what) => write!(f, "no {what}"),
        }
    }
}

/// Read a `.glb`: every mesh of its default scene merged.
pub fn load(bytes: &[u8]) -> Result<GlbPart, PartError> {
    let file = gltf::Gltf::from_slice(bytes).map_err(PartError::Gltf)?;
    let blob = file.blob.as_deref().ok_or(PartError::NoBinary)?;
    let scene = file
        .document
        .default_scene()
        .or_else(|| file.document.scenes().next())
        .ok_or(PartError::Missing("scene"))?;
    let mut out = GlbPart::default();
    let mut stack: Vec<(gltf::Node, Mat4)> = scene.nodes().map(|n| (n, Mat4::IDENTITY)).collect();
    while let Some((node, parent)) = stack.pop() {
        let at = parent * Mat4::from_cols_array_2d(&node.transform().matrix());
        stack.extend(node.children().map(|c| (c, at)));
        let Some(mesh) = node.mesh() else { continue };
        // a skinned mesh ignores its node's transform (glTF 2.0 §3.7.3.3): its vertices are in
        // the model's frame at rest; its `stretch` joint carries the part's stretch weights
        let (to_model, stretch) = match node.skin() {
            Some(skin) => (
                Mat4::IDENTITY,
                skin.joints().position(|j| j.name() == Some("stretch")),
            ),
            None => (at, None),
        };
        let turn = glam::Mat3::from_mat4(to_model).inverse().transpose();
        for prim in mesh.primitives() {
            if prim.mode() != gltf::mesh::Mode::Triangles {
                continue;
            }
            let read = prim.reader(|b| match b.source() {
                gltf::buffer::Source::Bin => Some(blob),
                gltf::buffer::Source::Uri(_) => None,
            });
            let base = out.positions.len() as u32;
            let positions: Vec<Vec3> = read
                .read_positions()
                .ok_or(PartError::Missing("positions"))?
                .map(|p| to_model.transform_point3(Vec3::from(p)))
                .collect();
            let n = positions.len();
            let normals: Vec<Vec3> = read
                .read_normals()
                .ok_or(PartError::Missing("normals"))?
                .map(|v| (turn * Vec3::from(v)).normalize_or_zero())
                .collect();
            let weights: Vec<f32> = match (stretch, read.read_joints(0), read.read_weights(0)) {
                (Some(j), Some(js), Some(ws)) => js
                    .into_u16()
                    .zip(ws.into_f32())
                    .map(|(js, ws)| {
                        js.iter()
                            .zip(ws)
                            .filter(|(k, _)| usize::from(**k) == j)
                            .map(|(_, w)| w)
                            .sum()
                    })
                    .collect(),
                _ => vec![0.0; n],
            };
            let morph: Vec<Vec3> = read
                .read_morph_targets()
                .next()
                .and_then(|(p, _, _)| p)
                .map_or_else(Vec::new, |p| p.map(Vec3::from).collect());
            if normals.len() != n || weights.len() != n {
                return Err(PartError::Missing("a vertex attribute"));
            }
            if !morph.is_empty() || !out.morph0.is_empty() {
                out.morph0.resize(out.positions.len(), Vec3::ZERO);
                out.morph0.extend(
                    morph
                        .iter()
                        .copied()
                        .chain(std::iter::repeat(Vec3::ZERO))
                        .take(n),
                );
            }
            let indices: Vec<u32> = match read.read_indices() {
                Some(i) => i.into_u32().collect(),
                None => (0..n as u32).collect(),
            };
            if indices.iter().any(|&i| i as usize >= n) {
                return Err(PartError::Missing("an index in range"));
            }
            out.indices.extend(indices.iter().map(|i| base + i));
            out.positions.extend(positions);
            out.normals.extend(normals);
            out.stretch_w.extend(weights);
        }
    }
    if out.indices.is_empty() {
        return Err(PartError::Missing("triangles"));
    }
    Ok(out)
}

/// The realistic piston (with its rings and pin) in the procedural piston's unit frame: that
/// cylinder spans y 0..1 from 0.6 bore under the crown, 0.49 bore in radius; the asset's crown is
/// 0.35 bore over its origin, the wrist pin. A uniform scale with the bore cancels in that frame.
pub fn piston_unit_mesh() -> Result<Mesh, PartError> {
    let part = load(PISTON_GLB)?;
    Ok(to_unit(
        &part,
        1.0 / REF_BORE,
        Vec3::new(0.0, 0.25, 0.0),
        Vec3::new(0.49, 0.6, 0.49),
    ))
}

/// The rod asset, read once (none if unreadable: the primitives stay).
pub fn rod_asset() -> Option<&'static GlbPart> {
    static ROD: std::sync::OnceLock<Option<GlbPart>> = std::sync::OnceLock::new();
    ROD.get_or_init(|| {
        load(CONROD_GLB)
            .inspect_err(|e| {
                eprintln!("3D view: conrod.glb unreadable ({e}), drawing the primitive")
            })
            .ok()
    })
    .as_ref()
}

/// The realistic rod for `rod_over_bore` (length between centres over the bore) in the
/// procedural rod's unit frame: that box is centred between the crankpin and the wrist pin, the
/// rod's length tall, 0.14 bore wide along the crank and 0.2 bore deep (`scene::bar_between`).
/// Stretched by the README's rule `p' = s·p + w·(rod − s·0.143)·Ŷ`; in bores, the scale cancels.
pub fn rod(rod_over_bore: f32) -> Option<Mesh> {
    let mut part = rod_asset()?.clone();
    stretch_rod(&mut part, rod_over_bore);
    Some(rod_frame(&part, rod_over_bore))
}

fn stretch_rod(part: &mut GlbPart, rod_over_bore: f32) {
    // in bores: the reference rod's 0.143/0.086 stretched to `rod_over_bore` by each vertex's
    // weight (the shank's normals are left as built: a stretch along it barely turns them)
    let extra = rod_over_bore - REF_ROD / REF_BORE;
    for (p, &w) in part.positions.iter_mut().zip(&part.stretch_w) {
        *p = *p / REF_BORE + w * extra * Vec3::Y;
    }
}

fn rod_frame(part: &GlbPart, rod_over_bore: f32) -> Mesh {
    to_unit(
        part,
        1.0,
        Vec3::new(0.0, -0.5 * rod_over_bore, 0.0),
        Vec3::new(0.14, rod_over_bore, 0.2),
    )
}

/// [`piston_unit_mesh`], built once (none if the asset cannot be read: the primitive stays).
pub fn piston() -> Option<&'static Mesh> {
    static PISTON: std::sync::OnceLock<Option<Mesh>> = std::sync::OnceLock::new();
    PISTON
        .get_or_init(|| {
            piston_unit_mesh()
                .inspect_err(|e| {
                    eprintln!("3D view: piston.glb unreadable ({e}), drawing the primitive")
                })
                .ok()
        })
        .as_ref()
}

/// The crankshaft throw asset, read once (none if unreadable: the primitives stay).
pub fn throw_asset() -> Option<&'static GlbPart> {
    static THROW: std::sync::OnceLock<Option<GlbPart>> = std::sync::OnceLock::new();
    THROW
        .get_or_init(|| {
            load(CRANK_GLB)
                .inspect_err(|e| {
                    eprintln!("3D view: crank_throw.glb unreadable ({e}), drawing the primitive")
                })
                .ok()
        })
        .as_ref()
}

/// How far down the counterweight's squeeze starts, in bores: under the main journals.
const SQUEEZE_FROM_B: f32 = 0.35;

/// The realistic throw for `throw_b` (half the stroke) and `rod_b` (the rod), both in bores,
/// in the procedural crankpin's unit frame: that cylinder runs along the crank from 0.13 bore
/// before the pin to 0.13 after it, 0.13 bore in radius, its frame turned with the throw (unit
/// x = the axis × the pin's direction, y = the crank axis, z = towards the pin; `scene::moving`).
/// Stretched by the README's rule (the pin moves out by `throw − s·0.043`, the mains and the
/// counterweight stay); the counterweight shrinks towards the axis when it would reach the
/// skirt at the bottom of the stroke (README: it must stay under `rod − throw − 0.30·bore`).
pub fn throw(throw_b: f32, rod_b: f32) -> Option<Mesh> {
    let mut part = throw_asset()?.clone();
    let room = (rod_b - throw_b - 0.30).max(SQUEEZE_FROM_B + 0.01);
    let squeeze = ((room - SQUEEZE_FROM_B) / (COUNTERWEIGHT_B - SQUEEZE_FROM_B)).min(1.0);
    for (p, &w) in part.positions.iter_mut().zip(&part.stretch_w) {
        let mut q = *p / REF_BORE + w * (throw_b - REF_THROW_B) * Vec3::Y;
        if q.y < -SQUEEZE_FROM_B {
            q.y = -SQUEEZE_FROM_B + (q.y + SQUEEZE_FROM_B) * squeeze;
        }
        // into the crankpin's turned frame: (across, along the crank, towards the pin)
        *p = Vec3::new(q.z, q.x, q.y);
    }
    for n in &mut part.normals {
        *n = Vec3::new(n.z, n.x, n.y);
    }
    Some(to_unit(
        &part,
        1.0,
        Vec3::new(0.0, 0.13, -throw_b),
        Vec3::new(0.13, 0.26, 0.13),
    ))
}

/// The valve asset, read once (none if unreadable: the primitives stay).
pub fn valve_asset() -> Option<&'static GlbPart> {
    static VALVE: std::sync::OnceLock<Option<GlbPart>> = std::sync::OnceLock::new();
    VALVE
        .get_or_init(|| {
            load(VALVE_GLB)
                .inspect_err(|e| {
                    eprintln!("3D view: valve.glb unreadable ({e}), drawing the primitive")
                })
                .ok()
        })
        .as_ref()
}

/// The realistic valve in the procedural valve head's unit frame: that disc stands on the face,
/// 0.17 bore in radius (the asset's head too, README) and 0.04 bore thick. Its stem is brought to
/// the scene's 0.5 bore by the README's rule (`p' = s·p + w·(stem − s·0.0989)·Ŷ`).
pub fn valve() -> Option<Mesh> {
    let mut part = valve_asset()?.clone();
    let extra = SCENE_STEM_B - REF_STEM / REF_BORE;
    for (p, &w) in part.positions.iter_mut().zip(&part.stretch_w) {
        *p = *p / REF_BORE + w * extra * Vec3::Y;
    }
    Some(to_unit(&part, 1.0, Vec3::ZERO, Vec3::new(0.17, 0.04, 0.17)))
}

/// The realistic rotor in the procedural rotor's unit frame (0.97 R across, 0.9 width deep from
/// its face; `scene::moving`), for a rotor of `e_over_r`: none unless that is the asset's
/// (another e/R is another flank shape: the asset must be built again for it, README).
pub fn rotor(e_over_r: f32) -> Option<Mesh> {
    let reference = REF_E / REF_R;
    if (e_over_r - reference).abs() > 0.01 * reference {
        return None;
    }
    static ROTOR: std::sync::OnceLock<Option<Mesh>> = std::sync::OnceLock::new();
    ROTOR
        .get_or_init(|| {
            let part = load(ROTOR_GLB)
                .inspect_err(|e| {
                    eprintln!("3D view: wankel_rotor.glb unreadable ({e}), drawing the primitive")
                })
                .ok()?;
            // scaled with the engine's R and width, which cancel in this frame
            Some(to_unit(
                &part,
                1.0,
                Vec3::new(0.0, 0.45 * REF_W, 0.0),
                Vec3::new(0.97 * REF_R, 0.9 * REF_W, 0.97 * REF_R),
            ))
        })
        .clone()
}

/// Into a unit frame: `p ↦ (p·k + offset) / scale`, normals `n ↦ n·scale` (normalised).
fn to_unit(part: &GlbPart, k: f32, offset: Vec3, scale: Vec3) -> Mesh {
    Mesh {
        vertices: part
            .positions
            .iter()
            .zip(&part.normals)
            .map(|(&p, &n)| Vertex {
                pos: ((p * k + offset) / scale).to_array(),
                normal: (n * scale).normalize_or_zero().to_array(),
            })
            .collect(),
        indices: part.indices.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds(ps: impl Iterator<Item = Vec3>) -> (Vec3, Vec3) {
        ps.fold(
            (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
            |(lo, hi), p| (lo.min(p), hi.max(p)),
        )
    }

    #[test]
    fn the_piston_asset_loads_at_its_reference_size() {
        let p = load(PISTON_GLB).expect("piston.glb");
        assert!(!p.indices.is_empty());
        assert!(p.indices.iter().all(|&i| (i as usize) < p.positions.len()));
        assert_eq!(p.normals.len(), p.positions.len());
        assert!(p.normals.iter().all(|n| (n.length() - 1.0).abs() < 1e-3));
        let (lo, hi) = bounds(p.positions.iter().copied());
        // across: the bore, 86 mm (the rings stand at the bore)
        assert!(((hi.z - lo.z) - REF_BORE).abs() < 0.5e-3, "{lo} {hi}");
        // the crown's edge 0.35 bore over the pin (README), its centre domed 0.9 mm higher
        // (scripts/piston.py: `(0.0, CROWN + 0.9 * mm)`): 30.1 + 0.9 = 31.0 mm
        assert!((hi.y - (0.35 * REF_BORE + 0.9e-3)).abs() < 0.1e-3, "{hi}");
    }

    #[test]
    fn the_piston_sits_in_the_procedural_pistons_frame() {
        let m = piston_unit_mesh().expect("the realistic piston");
        assert_eq!(piston(), Some(&m), "built once, the same");
        let (lo, hi) = bounds(m.vertices.iter().map(|v| Vec3::from(v.pos)));
        // crown edge at y = 1 (0.35 b over the pin, the pin 0.25 b over the frame's base, 0.6 b
        // per unit), its dome 0.9 mm / (0.6·86 mm) = 0.0174 higher; radius 0.5 b over 0.49 b
        assert!((hi.y - 1.0174).abs() < 0.002, "{hi}");
        assert!((hi.z - 0.5 / 0.49).abs() < 0.01, "{hi}");
        assert!((lo.z + 0.5 / 0.49).abs() < 0.01, "{lo}");
        // the skirt: 0.65 b tall in all, so 0.30 b under the pin → (0.25 − 0.30)/0.6
        assert!((lo.y - (-0.05 / 0.6)).abs() < 0.03, "{lo}");
        // normals into the unit frame: still pointing out of the crown at the top
        let top = m
            .vertices
            .iter()
            .filter(|v| v.pos[1] > 0.999)
            .map(|v| Vec3::from(v.normal).normalize())
            .fold(Vec3::ZERO, |a, n| a + n);
        assert!(top.y > 0.0, "{top}");
    }

    /// The rod's mesh back in metres for `rod` and `bore` (the instance's scale and centre).
    fn rod_world(rod: f32, bore: f32) -> Vec<Vec3> {
        let m = super::rod(rod / bore).expect("the realistic rod");
        let (scale, centre) = (Vec3::new(0.14 * bore, rod, 0.2 * bore), Vec3::Y * 0.5 * rod);
        m.vertices
            .iter()
            .map(|v| Vec3::from(v.pos) * scale + centre)
            .collect()
    }

    #[test]
    fn the_rod_stretches_its_small_end_only() {
        let at_ref = bounds(rod_world(0.143, 0.086).into_iter());
        let longer = bounds(rod_world(0.160, 0.086).into_iter());
        // 17 mm longer between centres: the small end's top 17 mm higher, the big end's bottom
        // where it was, its width unchanged
        assert!(
            ((longer.1.y - at_ref.1.y) - 0.017).abs() < 0.2e-3,
            "{at_ref:?} {longer:?}"
        );
        assert!((longer.0.y - at_ref.0.y).abs() < 0.1e-3);
        assert!((longer.1.z - at_ref.1.z).abs() < 0.1e-3);
        // a bigger engine scales it all: bore 100 mm, rod 143·100/86 = 166.3 mm
        let big = bounds(rod_world(0.143 * 100.0 / 86.0, 0.100).into_iter());
        let k = 100.0 / 86.0;
        assert!((big.1.y - k * at_ref.1.y).abs() < 0.2e-3, "{big:?}");
        assert!((big.0.y - k * at_ref.0.y).abs() < 0.2e-3);
    }

    #[test]
    fn the_rod_asset_carries_its_stretch() {
        let p = load(CONROD_GLB).expect("conrod.glb");
        // the big end's eye (round the origin) does not stretch; the small end's (0.143 up) does
        let near = |y: f32| {
            p.positions
                .iter()
                .zip(&p.stretch_w)
                .filter(|(q, _)| (q.y - y).abs() < 0.005 && q.x.abs() < 0.01)
                .map(|(_, &w)| w)
                .collect::<Vec<_>>()
        };
        let (big, small) = (near(0.0), near(REF_ROD));
        assert!(!big.is_empty() && !small.is_empty());
        assert!(big.iter().all(|&w| w < 0.05), "{big:?}");
        assert!(small.iter().all(|&w| w > 0.95), "{small:?}");
    }

    /// The throw back in bores, in the crank's own frame (x along the crank, y towards the pin).
    fn throw_bores(throw_b: f32, rod_b: f32) -> Vec<(Vec3, f32)> {
        let m = throw(throw_b, rod_b).expect("the realistic throw");
        let w = &throw_asset().expect("crank_throw.glb").stretch_w;
        m.vertices
            .iter()
            .zip(w)
            .map(|(v, &w)| {
                let u = Vec3::from(v.pos) * Vec3::new(0.13, 0.26, 0.13);
                // unit (x, y, z) = (across, along the crank, towards the pin), from 0.13 before
                // the pin's centre at `throw_b`
                (Vec3::new(u.y - 0.13, u.z + throw_b, u.x), w)
            })
            .collect()
    }

    #[test]
    fn the_throw_reaches_the_stroke() {
        // README: the pin (weight 1) moves out by the throw's change, the mains and the
        // counterweight (weight 0) stay: 43 → 50 mm of throw on an 86 mm bore is 7/86 bore
        let long = 50.0 / 86.0;
        let (at_ref, longer) = (throw_bores(0.5, 2.0), throw_bores(long, 2.0));
        let (mut pins, mut mains) = (0, 0);
        for ((a, w), (b, _)) in at_ref.iter().zip(&longer) {
            if *w > 0.999 {
                pins += 1;
                assert!(((b.y - a.y) - (long - 0.5)).abs() < 1e-4, "{a} {b}");
                assert!((b.x - a.x).abs() < 1e-5 && (b.z - a.z).abs() < 1e-5);
            } else if *w < 1e-3 {
                mains += 1;
                assert!((*b - *a).length() < 1e-5, "{a} {b}");
            }
        }
        assert!(pins > 100 && mains > 100, "{pins} {mains}");
    }

    #[test]
    fn the_counterweight_clears_a_short_rod() {
        // the reference rod (143/86) clears it: the counterweight keeps its 0.74 bore
        let low = |rod_b| {
            throw_bores(0.5, rod_b)
                .into_iter()
                .map(|(p, _)| p.y)
                .fold(f32::MAX, f32::min)
        };
        assert!(
            (low(143.0 / 86.0) + COUNTERWEIGHT_B).abs() < 0.01,
            "{}",
            low(143.0 / 86.0)
        );
        // a 1.4-bore rod on a 0.6-bore throw leaves 1.4 − 0.6 − 0.30 = 0.5 bore under the axis
        let short = throw_bores(0.6, 1.4)
            .into_iter()
            .map(|(p, _)| p.y)
            .fold(f32::MAX, f32::min);
        assert!(short >= -0.5 - 1e-3, "{short}");
    }

    #[test]
    fn the_valve_stands_on_the_scenes_disc() {
        let m = valve().expect("the realistic valve");
        let w = &valve_asset().expect("valve.glb").stretch_w;
        let unit = Vec3::new(0.17, 0.04, 0.17);
        let bores: Vec<(Vec3, f32)> = m
            .vertices
            .iter()
            .zip(w)
            .map(|(v, &w)| (Vec3::from(v.pos) * unit, w))
            .collect();
        // the head (weight 0): 0.17 bore in radius, as the scene's disc, on the face (y ≥ 0)
        let head = bores.iter().filter(|(_, w)| *w < 1e-3);
        let (r, low) = head.fold((0.0f32, f32::MAX), |(r, l), (p, _)| {
            (r.max(p.x.hypot(p.z)), l.min(p.y))
        });
        assert!((r - 0.17).abs() < 0.005, "{r}");
        assert!(low > -0.005, "{low}");
        // the tip end (weight 1) moved down by 0.0989/0.086 − 0.5 = 0.65 bore: its top at the
        // asset's top less that
        let asset = valve_asset().unwrap();
        let top_ref = asset
            .positions
            .iter()
            .map(|p| p.y / REF_BORE)
            .fold(f32::MIN, f32::max);
        let top = bores.iter().map(|(p, _)| p.y).fold(f32::MIN, f32::max);
        let drop = REF_STEM / REF_BORE - SCENE_STEM_B;
        assert!((top - (top_ref - drop)).abs() < 1e-3, "{top} {top_ref}");
    }

    #[test]
    fn the_rotor_flank_waist_is_r_minus_2e() {
        let m = rotor(REF_E / REF_R).expect("the realistic rotor");
        // back to metres: x, z by 0.97 R; the flank's middle (60° from apex 0) stands R − 2e =
        // 75 mm from the centre (README: the true inner envelope, mid-flank 75.00 mm)
        let (mid, apex) = m.vertices.iter().fold((0.0f32, 0.0f32), |(mid, apex), v| {
            let (x, z) = (v.pos[0] * 0.97 * REF_R, v.pos[2] * 0.97 * REF_R);
            let (r, a) = (x.hypot(z), z.atan2(x).to_degrees());
            (
                if (a - 60.0).abs() < 0.5 {
                    mid.max(r)
                } else {
                    mid
                },
                if a.abs() < 0.5 { apex.max(r) } else { apex },
            )
        });
        assert!((mid - (REF_R - 2.0 * REF_E)).abs() < 0.5e-3, "{mid}");
        // the apex stays within R: the rotor fits its housing
        assert!(apex > 0.9 * REF_R && apex <= REF_R + 1e-4, "{apex}");
        // and the width spans 0..1 of the frame's 0.9 w
        let (lo, hi) = m.vertices.iter().fold((f32::MAX, f32::MIN), |(l, h), v| {
            (l.min(v.pos[1]), h.max(v.pos[1]))
        });
        assert!(lo > -0.1 && hi < 1.1, "{lo} {hi}");
        // another e/R: none (the primitive stays)
        assert!(rotor(0.16).is_none());
    }

    #[test]
    fn a_bad_asset_is_an_error() {
        assert!(load(b"not a gltf file").is_err());
    }
}
