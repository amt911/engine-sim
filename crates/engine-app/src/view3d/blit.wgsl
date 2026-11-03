// Copies the 3D view's off-screen target into egui's pass.

@group(0) @binding(0) var tex: texture_2d<f32>;
@group(0) @binding(1) var samp: sampler;

struct BOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_blit(@builtin(vertex_index) k: u32) -> BOut {
    // one triangle covering the viewport
    let xy = vec2<f32>(f32((k << 1u) & 2u), f32(k & 2u));
    var o: BOut;
    o.clip = vec4<f32>(xy * 2.0 - 1.0, 0.0, 1.0);
    o.uv = vec2<f32>(xy.x, 1.0 - xy.y);
    return o;
}

@fragment
fn fs_blit(i: BOut) -> @location(0) vec4<f32> {
    return textureSample(tex, samp, i.uv);
}
