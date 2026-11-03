//! An orbit camera: it circles a target point, turned by dragging and moved in and out by the
//! scroll wheel. Right-handed, y up; depth 0..1 (wgpu).

use glam::{Mat4, Vec3};

/// Radians turned per pixel dragged.
pub const RAD_PER_PX: f32 = 0.01;
/// Furthest the camera looks down or up, rad (short of the poles, where yaw degenerates).
pub const MAX_PITCH: f32 = 1.5;
/// Closest and furthest the camera stands from its target, m.
pub const MIN_DISTANCE: f32 = 0.05;
pub const MAX_DISTANCE: f32 = 20.0;
/// Vertical field of view, rad (45°), and the clip planes, m.
pub const FOV_Y: f32 = std::f32::consts::FRAC_PI_4;
pub const NEAR: f32 = 0.01;
pub const FAR: f32 = 100.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitCamera {
    pub target: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

impl OrbitCamera {
    /// Looking at a sphere of `radius` around `center` from `yaw`, a little from above, far
    /// enough that the whole sphere shows (5 % margin).
    pub fn framing(center: Vec3, radius: f32, yaw: f32) -> Self {
        Self {
            target: center,
            yaw,
            pitch: 0.3,
            distance: (1.05 * radius / (0.5 * FOV_Y).sin()).clamp(MIN_DISTANCE, MAX_DISTANCE),
        }
    }

    pub fn drag(&mut self, dx_px: f32, dy_px: f32) {
        if !(dx_px.is_finite() && dy_px.is_finite()) {
            return;
        }
        self.yaw = (self.yaw + dx_px * RAD_PER_PX) % std::f32::consts::TAU;
        self.pitch = (self.pitch + dy_px * RAD_PER_PX).clamp(-MAX_PITCH, MAX_PITCH);
    }

    /// Scroll up (positive) moves in: ×0.9 every 50 units.
    pub fn zoom(&mut self, scroll: f32) {
        if !scroll.is_finite() {
            return;
        }
        let d = self.distance * 0.9f32.powf(scroll / 50.0);
        self.distance =
            if d.is_finite() { d } else { MAX_DISTANCE }.clamp(MIN_DISTANCE, MAX_DISTANCE);
    }

    pub fn eye(&self) -> Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        self.target + self.distance * Vec3::new(cp * sy, sp, cp * cy)
    }

    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        // wgpu's clip space: z in [0, 1], y up (glam's "directx" convention)
        let proj = glam::camera::rh::proj::directx::perspective(FOV_Y, aspect.max(1e-3), NEAR, FAR);
        proj * glam::camera::rh::view::look_at_mat4(self.eye(), self.target, Vec3::Y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cam(yaw: f32, pitch: f32, distance: f32) -> OrbitCamera {
        OrbitCamera {
            target: Vec3::ZERO,
            yaw,
            pitch,
            distance,
        }
    }

    #[test]
    fn eye_of_yaw_zero_pitch_zero_is_on_plus_z() {
        let e = cam(0.0, 0.0, 3.0).eye();
        assert!((e - Vec3::new(0.0, 0.0, 3.0)).length() < 1e-6, "{e}");
    }

    #[test]
    fn drag_turns_by_rad_per_px() {
        let mut c = cam(0.0, 0.0, 3.0);
        c.drag(100.0, 0.0);
        assert!((c.yaw - 1.0).abs() < 1e-6);
        c.drag(0.0, -20.0);
        assert!((c.pitch + 0.2).abs() < 1e-6);
    }

    #[test]
    fn orbit_clamps_pitch_and_distance() {
        let mut c = cam(0.0, 0.0, 3.0);
        c.drag(0.0, 1e6);
        assert_eq!(c.pitch, 1.5);
        c.drag(f32::NAN, 3.0);
        assert_eq!(c.pitch, 1.5);
        c.zoom(1e9);
        assert_eq!(c.distance, 0.05);
        c.zoom(-1e9);
        assert_eq!(c.distance, 20.0);
        c.zoom(f32::INFINITY);
        assert_eq!(c.distance, 20.0);
        assert!(c.eye().is_finite());
    }

    #[test]
    fn zoom_is_ten_percent_per_50() {
        let mut c = cam(0.0, 0.0, 2.0);
        c.zoom(50.0);
        assert!((c.distance - 1.8).abs() < 1e-6);
    }

    #[test]
    fn view_proj_puts_the_target_at_the_centre() {
        let mut c = cam(0.7, 0.3, 2.0);
        c.target = Vec3::new(0.1, 0.2, -0.3);
        let p = c.view_proj(1.5) * c.target.extend(1.0);
        let ndc = p.truncate() / p.w;
        assert!(ndc.x.abs() < 1e-5 && ndc.y.abs() < 1e-5, "{ndc}");
        assert!(ndc.z > 0.0 && ndc.z < 1.0, "{ndc}");
    }

    #[test]
    fn framing_fits_a_sphere() {
        // 1.05·r / sin(22.5°) = 1.05 / 0.382683 = 2.74378
        let c = OrbitCamera::framing(Vec3::ONE, 1.0, 0.4);
        assert!((c.distance - 2.74378).abs() < 1e-4, "{}", c.distance);
        assert_eq!((c.target, c.yaw), (Vec3::ONE, 0.4));
    }
}
