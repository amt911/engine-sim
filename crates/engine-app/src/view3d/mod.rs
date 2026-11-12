//! The engine in 3D (docs/superpowers/specs/2026-11-03-3d-view-design.md): pure scene building
//! (camera, meshes, the parts from the definition and the snapshot) and a thin wgpu renderer.

pub mod arrows;
pub mod camera;
pub mod card;
pub mod colormap;
pub mod detail;
pub mod ducts;
pub mod handles;
pub mod mesh;
pub mod parts;
pub mod pick;
pub mod render;
pub mod routing;
pub mod scene;
pub mod ui;
