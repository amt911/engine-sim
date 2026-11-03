// The 3D view's two passes: the scene (instanced unit meshes, one directional light) into an
// off-screen target, and a blit of that target into egui's pass.

struct Globals {
    view_proj: mat4x4<f32>,
    // direction towards the light (xyz), ambient share (w)
    light: vec4<f32>,
};
@group(0) @binding(0) var<uniform> g: Globals;

struct VIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) m0: vec4<f32>,
    @location(3) m1: vec4<f32>,
    @location(4) m2: vec4<f32>,
    @location(5) m3: vec4<f32>,
    @location(6) color: vec4<f32>,
};

struct VOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) n: vec3<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs_scene(v: VIn) -> VOut {
    let m = mat4x4<f32>(v.m0, v.m1, v.m2, v.m3);
    let m3 = mat3x3<f32>(v.m0.xyz, v.m1.xyz, v.m2.xyz);
    // models are translation · rotation · scale: the inverse transpose is R·S⁻¹ = M·S⁻²
    let s2 = vec3<f32>(dot(v.m0.xyz, v.m0.xyz), dot(v.m1.xyz, v.m1.xyz), dot(v.m2.xyz, v.m2.xyz));
    var o: VOut;
    o.n = m3 * (v.normal / max(s2, vec3<f32>(1e-20)));
    o.clip = g.view_proj * m * vec4<f32>(v.pos, 1.0);
    o.color = v.color;
    return o;
}

@fragment
fn fs_scene(i: VOut) -> @location(0) vec4<f32> {
    let n = normalize(i.n);
    let lambert = max(dot(n, g.light.xyz), 0.0);
    let shade = g.light.w + (1.0 - g.light.w) * lambert;
    // colours are display (sRGB-like) values and the target is not sRGB: shade in linear light
    return vec4<f32>(i.color.rgb * pow(shade, 1.0 / 2.2), 1.0);
}
