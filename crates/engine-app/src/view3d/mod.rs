//! The engine in 3D (docs/superpowers/specs/2026-11-03-3d-view-design.md): pure scene building
//! (camera, meshes, the parts from the definition and the snapshot) and a thin wgpu renderer.

pub mod camera;
pub mod detail;
pub mod mesh;
pub mod render;
pub mod routing;
pub mod scene;
pub mod ui;
