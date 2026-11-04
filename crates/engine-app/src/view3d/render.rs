//! The wgpu side of the 3D view: unit meshes in GPU buffers, the frame's instances, an
//! off-screen colour + depth target the scene is drawn into in `prepare`, blitted into egui's
//! pass in `paint` (so egui's own pass needs no depth buffer and no matching sample count).
//!
//! The resources live in egui-wgpu's `callback_resources`, installed once with [`install`].
//! The UI hands each frame a [`Frame3d`] that only clones an `Arc`: the instances live in a
//! [`Shared`] buffer the UI refills in place, so a steady frame allocates nothing.

use super::mesh::{self, Mesh, Vertex};
use super::scene::{Instance, MeshKind};
use eframe::egui_wgpu::{self, wgpu};
use glam::{Mat4, Vec3};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use wgpu::util::DeviceExt;

/// Off-screen colour format: plain (not sRGB) like egui's target, the shader writes display
/// values.
const COLOR: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
/// The 2D view's background, (24, 26, 30).
const BACKGROUND: wgpu::Color = wgpu::Color {
    r: 24.0 / 255.0,
    g: 26.0 / 255.0,
    b: 30.0 / 255.0,
    a: 1.0,
};

/// What the meshes depend on: rebuilt only when this changes (the definition's edit count, the
/// detail's segment count, a rotary's e/R).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshKey {
    pub def_version: u64,
    pub segments: u32,
    pub e_over_r: f32,
    /// The realistic parts of `assets/3d-parts` (Medium and High) instead of the primitives.
    pub realistic: bool,
    /// The rod's length over the bore (the realistic rod is stretched to it).
    pub rod_over_bore: f32,
    /// Half the stroke over the bore (the realistic throw is stretched to it).
    pub throw_over_bore: f32,
    /// The tallest lift of the intake and exhaust lobes over the bore (the realistic cams are built
    /// for them).
    pub intake_lobe_over_bore: f32,
    pub exhaust_lobe_over_bore: f32,
}

/// The frame's scene, written by the UI thread and read by `prepare`.
#[derive(Debug, Default)]
pub struct Shared {
    pub view_proj: Mat4,
    pub instances: Vec<Instance>,
    /// The cut-away plane (normal, offset), when the layer is on.
    pub clip: Option<(Vec3, f32)>,
}

/// One frame's paint callback.
pub struct Frame3d {
    pub shared: Arc<Mutex<Shared>>,
    pub size_px: [u32; 2],
    pub key: MeshKey,
}

/// Pixel size of the off-screen target for a rect of `size_points` at `pixels_per_point`:
/// never zero on a side (wgpu rejects empty textures).
pub fn target_size(size_points: [f32; 2], pixels_per_point: f32) -> [u32; 2] {
    let px = |v: f32| {
        let p = (v * pixels_per_point).round();
        if p.is_finite() && p >= 1.0 {
            (p as u32).min(8192)
        } else {
            1
        }
    };
    [px(size_points[0]), px(size_points[1])]
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct GpuInstance {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    light: [f32; 4],
    /// The cut-away plane: fragments where n·p > d are discarded (n = 0: no cut).
    clip: [f32; 4],
}

struct GpuMesh {
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    count: u32,
}

struct Target {
    size: [u32; 2],
    color: wgpu::TextureView,
    depth: wgpu::TextureView,
    blit: wgpu::BindGroup,
}

/// Everything on the GPU side, kept between frames.
pub struct Resources {
    scene: wgpu::RenderPipeline,
    blit: wgpu::RenderPipeline,
    blit_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    globals: wgpu::Buffer,
    globals_bind: wgpu::BindGroup,
    meshes: HashMap<MeshKind, GpuMesh>,
    key: Option<MeshKey>,
    instances: Option<(wgpu::Buffer, usize)>,
    /// Instances grouped by mesh, reused every frame.
    staging: Vec<GpuInstance>,
    ranges: Vec<(MeshKind, u32, u32)>,
    target: Option<Target>,
}

/// Create the pipelines and put them where `Frame3d` finds them. `target_format` is egui's;
/// eframe's pass has one sample and no depth (the app keeps eframe's defaults).
pub fn install(rs: &egui_wgpu::RenderState) {
    let res = Resources::new(&rs.device, rs.target_format);
    rs.renderer.write().callback_resources.insert(res);
}

fn layout_entry(
    binding: u32,
    ty: wgpu::BindingType,
    vis: wgpu::ShaderStages,
) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: vis,
        ty,
        count: None,
    }
}

impl Resources {
    fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let scene_shader = device.create_shader_module(wgpu::include_wgsl!("scene.wgsl"));
        let blit_shader = device.create_shader_module(wgpu::include_wgsl!("blit.wgsl"));
        let globals_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("3d globals"),
            entries: &[layout_entry(
                0,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                wgpu::ShaderStages::VERTEX_FRAGMENT,
            )],
        });
        let globals = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("3d globals"),
            size: std::mem::size_of::<Globals>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let globals_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("3d globals"),
            layout: &globals_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: globals.as_entire_binding(),
            }],
        });
        let vertex_attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];
        let instance_attrs = wgpu::vertex_attr_array![
            2 => Float32x4, 3 => Float32x4, 4 => Float32x4, 5 => Float32x4, 6 => Float32x4
        ];
        let scene_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("3d scene"),
            bind_group_layouts: &[Some(&globals_layout)],
            ..Default::default()
        });
        let scene = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("3d scene"),
            layout: Some(&scene_layout),
            vertex: wgpu::VertexState {
                module: &scene_shader,
                entry_point: Some("vs_scene"),
                compilation_options: Default::default(),
                buffers: &[
                    Some(wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<Vertex>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &vertex_attrs,
                    }),
                    Some(wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<GpuInstance>() as u64,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &instance_attrs,
                    }),
                ],
            },
            primitive: wgpu::PrimitiveState {
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &scene_shader,
                entry_point: Some("fs_scene"),
                compilation_options: Default::default(),
                targets: &[Some(COLOR.into())],
            }),
            multiview_mask: None,
            cache: None,
        });
        let blit_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("3d blit"),
            entries: &[
                layout_entry(
                    0,
                    wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    wgpu::ShaderStages::FRAGMENT,
                ),
                layout_entry(
                    1,
                    wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    wgpu::ShaderStages::FRAGMENT,
                ),
            ],
        });
        let blit_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("3d blit"),
            bind_group_layouts: &[Some(&blit_layout)],
            ..Default::default()
        });
        let blit = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("3d blit"),
            layout: Some(&blit_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &blit_shader,
                entry_point: Some("vs_blit"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &blit_shader,
                entry_point: Some("fs_blit"),
                compilation_options: Default::default(),
                targets: &[Some(target_format.into())],
            }),
            multiview_mask: None,
            cache: None,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("3d blit"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        Self {
            scene,
            blit,
            blit_layout,
            sampler,
            globals,
            globals_bind,
            meshes: HashMap::new(),
            key: None,
            instances: None,
            staging: Vec::new(),
            ranges: Vec::new(),
            target: None,
        }
    }

    fn upload_meshes(&mut self, device: &wgpu::Device, key: MeshKey) {
        let build = |m: Mesh| GpuMesh {
            vertices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("3d mesh"),
                contents: bytemuck::cast_slice(&m.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }),
            indices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("3d mesh"),
                contents: bytemuck::cast_slice(&m.indices),
                usage: wgpu::BufferUsages::INDEX,
            }),
            count: m.indices.len() as u32,
        };
        self.meshes.clear();
        for kind in MeshKind::ALL {
            self.meshes.insert(kind, build(unit_mesh(kind, key)));
        }
        self.key = Some(key);
    }

    fn ensure_target(&mut self, device: &wgpu::Device, size: [u32; 2]) {
        if self.target.as_ref().is_some_and(|t| t.size == size) {
            return;
        }
        let texture = |format, usage, label| {
            device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some(label),
                    size: wgpu::Extent3d {
                        width: size[0],
                        height: size[1],
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&Default::default())
        };
        let color = texture(
            COLOR,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            "3d colour",
        );
        let depth = texture(DEPTH, wgpu::TextureUsages::RENDER_ATTACHMENT, "3d depth");
        let blit = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("3d blit"),
            layout: &self.blit_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&color),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
            ],
        });
        self.target = Some(Target {
            size,
            color,
            depth,
            blit,
        });
    }

    /// Sort the frame's instances by mesh into `staging` (one draw per mesh).
    fn stage(&mut self, instances: &[Instance]) {
        self.staging.clear();
        self.ranges.clear();
        for kind in MeshKind::ALL {
            let start = self.staging.len() as u32;
            for i in instances.iter().filter(|i| i.part.mesh() == kind) {
                self.staging.push(GpuInstance {
                    model: i.model.to_cols_array_2d(),
                    color: i.color,
                });
            }
            let end = self.staging.len() as u32;
            if end > start {
                self.ranges.push((kind, start, end));
            }
        }
    }

    fn upload_instances(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        let need = self.staging.len().max(1);
        if self.instances.as_ref().is_none_or(|(_, cap)| *cap < need) {
            let cap = need.next_power_of_two();
            let buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("3d instances"),
                size: (cap * std::mem::size_of::<GpuInstance>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.instances = Some((buf, cap));
        }
        if let Some((buf, _)) = &self.instances {
            queue.write_buffer(buf, 0, bytemuck::cast_slice(&self.staging));
        }
    }
}

/// The unit mesh of each kind at the key's detail (an exhaustive match: a new kind cannot be
/// left without its mesh, as the pipes' ball joints once were).
pub fn unit_mesh(kind: MeshKind, key: MeshKey) -> Mesh {
    let s = key.segments;
    match kind {
        MeshKind::Cylinder => mesh::cylinder(s),
        MeshKind::Cuboid => mesh::cuboid(),
        // the side plate and the rotor housing (with its plugs), when realistic
        MeshKind::Housing => match super::parts::side_plate(key.e_over_r) {
            Some(m) if key.realistic => m,
            _ => mesh::housing(key.e_over_r, 4 * s),
        },
        MeshKind::HousingRing => match super::parts::housing(key.e_over_r) {
            Some(m) if key.realistic => m,
            _ => mesh::housing_ring(key.e_over_r, 4 * s),
        },
        MeshKind::Plug => match super::parts::housing(key.e_over_r) {
            Some(_) if key.realistic => Mesh::nothing(),
            _ => mesh::cylinder(s),
        },
        MeshKind::Eccentric => match super::parts::eccentric(key.e_over_r) {
            Some(m) if key.realistic => m,
            _ => mesh::cylinder(s),
        },
        MeshKind::Rotor => match super::parts::rotor(key.e_over_r) {
            Some(m) if key.realistic => m,
            _ => mesh::rotor(s.div_ceil(2)),
        },
        // the realistic rotor carries its seals
        MeshKind::ApexSeal => match super::parts::rotor(key.e_over_r) {
            Some(_) if key.realistic => Mesh::nothing(),
            _ => mesh::cuboid(),
        },
        MeshKind::Liner => mesh::half_liner(s),
        // the realistic throw carries its webs
        MeshKind::Web => match super::parts::throw_asset() {
            Some(_) if key.realistic => Mesh::nothing(),
            _ => mesh::crank_web(s.div_ceil(2)),
        },
        MeshKind::Sphere => mesh::sphere(s.div_ceil(2)),
        MeshKind::Piston => match super::parts::piston() {
            Some(m) if key.realistic => m.clone(),
            _ => mesh::cylinder(s),
        },
        // the realistic piston carries its rings and pin: theirs draw nothing then
        MeshKind::PistonRing | MeshKind::WristPin => match super::parts::piston() {
            Some(_) if key.realistic => Mesh::nothing(),
            _ => mesh::cylinder(s),
        },
        MeshKind::Rod => match super::parts::rod(key.rod_over_bore) {
            Some(m) if key.realistic => m,
            _ => mesh::cuboid(),
        },
        MeshKind::Throw => match super::parts::throw(key.throw_over_bore, key.rod_over_bore) {
            Some(m) if key.realistic => m,
            _ => mesh::cylinder(s),
        },
        MeshKind::Valve => match super::parts::valve() {
            Some(m) if key.realistic => m,
            _ => mesh::cylinder(s),
        },
        // the realistic valve carries its stem
        MeshKind::ValveStem => match super::parts::valve_asset() {
            Some(_) if key.realistic => Mesh::nothing(),
            _ => mesh::cylinder(s),
        },
        // the valve train exists at the realistic detail only
        MeshKind::ValveSpring => match super::parts::spring() {
            Some(m) if key.realistic => m.clone(),
            _ => Mesh::nothing(),
        },
        MeshKind::IntakeCam => match super::parts::cam(key.intake_lobe_over_bore) {
            Some(m) if key.realistic => m,
            _ => Mesh::nothing(),
        },
        MeshKind::ExhaustCam => match super::parts::cam(key.exhaust_lobe_over_bore) {
            Some(m) if key.realistic => m,
            _ => Mesh::nothing(),
        },
        // the realistic rod carries its eyes
        MeshKind::RodEye => match super::parts::rod_asset() {
            Some(_) if key.realistic => Mesh::nothing(),
            _ => mesh::cylinder(s),
        },
    }
}

impl egui_wgpu::CallbackTrait for Frame3d {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen: &egui_wgpu::ScreenDescriptor,
        encoder: &mut wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let Some(res) = resources.get_mut::<Resources>() else {
            return Vec::new();
        };
        if res.key != Some(self.key) {
            res.upload_meshes(device, self.key);
        }
        res.ensure_target(device, self.size_px);
        let view_proj = {
            let Ok(shared) = self.shared.lock() else {
                return Vec::new();
            };
            res.stage(&shared.instances);
            (shared.view_proj, shared.clip)
        };
        let (view_proj, clip) = view_proj;
        let clip = clip.map_or([0.0; 4], |(n, d)| [n.x, n.y, n.z, d]);
        res.upload_instances(device, queue);
        let light = Vec3::new(0.4, 0.8, 0.45).normalize();
        queue.write_buffer(
            &res.globals,
            0,
            bytemuck::bytes_of(&Globals {
                view_proj: view_proj.to_cols_array_2d(),
                light: [light.x, light.y, light.z, 0.25],
                clip,
            }),
        );
        let (Some(target), Some((instances, _))) = (&res.target, &res.instances) else {
            return Vec::new();
        };
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("3d scene"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &target.color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(BACKGROUND),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &target.depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: None,
            }),
            ..Default::default()
        });
        pass.set_pipeline(&res.scene);
        pass.set_bind_group(0, &res.globals_bind, &[]);
        pass.set_vertex_buffer(1, instances.slice(..));
        for &(kind, start, end) in &res.ranges {
            if let Some(m) = res.meshes.get(&kind) {
                pass.set_vertex_buffer(0, m.vertices.slice(..));
                pass.set_index_buffer(m.indices.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.count, 0, start..end);
            }
        }
        Vec::new()
    }

    fn paint(
        &self,
        _info: eframe::egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        let Some(res) = resources.get::<Resources>() else {
            return;
        };
        let Some(target) = &res.target else { return };
        pass.set_pipeline(&res.blit);
        pass.set_bind_group(0, &target.blit, &[]);
        pass.draw(0..3, 0..1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_realistic_piston_comes_with_the_detail() {
        let key = |realistic| MeshKey {
            def_version: 1,
            segments: 12,
            e_over_r: 0.0,
            realistic,
            rod_over_bore: 143.0 / 86.0,
            throw_over_bore: 0.5,
            intake_lobe_over_bore: 0.11,
            exhaust_lobe_over_bore: 0.11,
        };
        let cylinder = mesh::cylinder(12).vertices.len();
        // Low: the primitives; Medium and High: the realistic piston, which carries its own
        // rings and pin, so theirs draw nothing (one degenerate triangle)
        assert_eq!(
            unit_mesh(MeshKind::Piston, key(false)).vertices.len(),
            cylinder
        );
        assert_eq!(
            unit_mesh(MeshKind::PistonRing, key(false)).vertices.len(),
            cylinder
        );
        assert!(unit_mesh(MeshKind::Piston, key(true)).vertices.len() > 10 * cylinder);
        assert_eq!(unit_mesh(MeshKind::Rod, key(false)), mesh::cuboid());
        assert!(unit_mesh(MeshKind::Rod, key(true)).vertices.len() > 10 * cylinder);
        assert_eq!(
            unit_mesh(MeshKind::Throw, key(false)).vertices.len(),
            cylinder
        );
        assert!(unit_mesh(MeshKind::Throw, key(true)).vertices.len() > 10 * cylinder);
        assert!(unit_mesh(MeshKind::Valve, key(true)).vertices.len() > 10 * cylinder);
        // a rotary of the asset's e/R (15/105, the RENESIS) takes the realistic rotor and its seals
        let rotary = |realistic| MeshKey {
            e_over_r: 15.0 / 105.0,
            ..key(realistic)
        };
        let primitive = unit_mesh(MeshKind::Rotor, rotary(false)).vertices.len();
        assert!(unit_mesh(MeshKind::Rotor, rotary(true)).vertices.len() > 10 * primitive);
        assert_eq!(unit_mesh(MeshKind::ApexSeal, rotary(true)).indices.len(), 3);
        assert_eq!(unit_mesh(MeshKind::Plug, rotary(true)).indices.len(), 3);
        for kind in [
            MeshKind::Housing,
            MeshKind::HousingRing,
            MeshKind::Eccentric,
        ] {
            let primitive = unit_mesh(kind, rotary(false)).vertices.len();
            assert!(
                unit_mesh(kind, rotary(true)).vertices.len() > 5 * primitive,
                "{kind:?}"
            );
        }
        assert_eq!(
            unit_mesh(MeshKind::Plug, key(true)),
            mesh::cylinder(12),
            "not a rotary"
        );
        assert_eq!(
            unit_mesh(MeshKind::ApexSeal, key(true)),
            mesh::cuboid(),
            "not a rotary"
        );
        for kind in [
            MeshKind::PistonRing,
            MeshKind::WristPin,
            MeshKind::RodEye,
            MeshKind::Web,
            MeshKind::ValveStem,
        ] {
            let m = unit_mesh(kind, key(true));
            assert_eq!(m.indices.len(), 3);
            assert!(m.vertices.iter().all(|v| v.pos == m.vertices[0].pos));
        }
    }

    #[test]
    fn every_kind_has_a_mesh() {
        let key = MeshKey {
            def_version: 1,
            segments: 12,
            e_over_r: 0.15,
            realistic: true,
            rod_over_bore: 1.6,
            throw_over_bore: 0.5,
            intake_lobe_over_bore: 0.11,
            exhaust_lobe_over_bore: 0.11,
        };
        for kind in MeshKind::ALL {
            let m = unit_mesh(kind, key);
            assert!(!m.indices.is_empty(), "{kind:?}");
        }
    }

    #[test]
    fn target_size_is_never_zero() {
        assert_eq!(target_size([0.0, 0.4], 1.0), [1, 1]);
        assert_eq!(target_size([f32::NAN, -3.0], 2.0), [1, 1]);
        // 640 × 360 points at 1.5 px/pt: 960 × 540 px
        assert_eq!(target_size([640.0, 360.0], 1.5), [960, 540]);
        assert_eq!(target_size([1e6, 10.0], 1.0), [8192, 10]);
    }
}
