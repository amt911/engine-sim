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

    #[test]
    fn a_bad_asset_is_an_error() {
        assert!(load(b"not a gltf file").is_err());
    }
}
