# engine-sim 3D parts (parametric, Blender-generated)

The 3D view's realistic parts. **The scripts are the source; `out/*.glb` are their build
output**, read by `crates/engine-app/src/view3d/`. Change a part by changing its script and
rebuilding it (`scripts/run.sh <part> --render`), then `python3 scripts/verify.py`; never edit a
`.glb` by hand. `renders/` (previews) and `refs/` (reference photos) are local only (git-ignored):
`run.sh --render` makes the previews again.

## Layout

```
scripts/common.py    helpers: lathe, prism, cylinders, booleans, bevel, materials, 2-joint skin,
                     glTF export, studio preview render (Cycles CPU)
scripts/piston.py    piston + 3 rings (oil ring = 2 rails + expander) + wrist pin
scripts/conrod.py    rod body (I-beam) + split cap + bolts + big-end shells + small-end bush
scripts/crankshaft.py  one throw module (repeat per cylinder) + nose (timing sprocket) + flange
scripts/valvetrain.py  valve + keepers + retainer, valve spring, cam lobe (flat follower), rocker arm
scripts/turbo.py     turbocharger: housings (compressor, centre, turbine + wastegate port), compressor
                     wheel (blades + splitters), turbine wheel, wastegate flap
scripts/wankel.py    rotor (true flank), rotor housing + plugs, side plate + stationary gear,
                     eccentric shaft; `-- --e 0.015 --R 0.105 --w 0.08` for other rotors
scripts/run.sh       ./run.sh piston [--render] | ./run.sh all --render
                     (headless Blender in a 6 GB cgroup, nice 19)
scripts/verify.py    python3 scripts/verify.py: reads the .glb like the app will (normals,
                     skins, dimensions, stretch rule); exit 1 on failure
out/*.glb            exported parts
renders/*.png        previews of every part (+ crank_i4, wankel_front/assembly, stretch demos)
refs/                the reference photos (1 piston+rod, 2 valve train, 3 crankshaft)
```

To edit by hand: open a `.glb` in Blender (File → Import → glTF), or better change the numbers in
the script and run `scripts/run.sh <part> --render`. The scripts are the source; the `.glb` are
build output.

## Conventions (match `view3d`: crank +X, cylinder +Y, metres)

- Modelled in Blender Z-up; the glTF export turns it Y-up: crank/pin axes **+X**, cylinder/rod
  axis **+Y**. Units: metres.
- Reference engine: **bore 86 mm, stroke 86 mm, rod 143 mm**. Proportions as fractions of the bore:
  crown 0.35 b over the pin (as the scene), piston 0.65 b tall, wrist pin r = 0.11 b,
  crankpin r = 0.25 b.
- Each node has `extras.engine_sim` (JSON string): `part`, `ref`, `origin`, `rule`.
- Only position + normal are needed by the current shader. Materials are glTF PBR factors
  (cast aluminium, forged steel, polished steel, dark steel, bronze) for a later PBR pass.

## Parameter rules

| file | origin | rule |
|---|---|---|
| `piston.glb` | wrist-pin centre | uniform scale `s = bore_m / 0.086` |
| `conrod.glb` | big-end centre, small end at +Y | `p' = s·p + w·(rod_m − s·0.143)·Ŷ`, `w` = `WEIGHTS_0` of joint `stretch` (0 big end, 1 small end, linear along the beam) |
| `crank_throw.glb` | axis at the cylinder plane, pin at +Y (crank angle 0) | `p' = s·p + w·(stroke_m/2 − s·0.043)·Ŷ`; pin w = 1, mains w = 0, webs w = clamp(y / 0.043): the arm stretches, the counterweight stays. Module spans ±pitch/2 (1.2 bore), so throw i sits at x = i·pitch turned about X by its crank angle |
| `crank_nose.glb`, `crank_flange.glb` | x = 0 on the end module's face; nose to −X, flange to +X | uniform `s` |
| `valve.glb` | valve face centre, valve closed, stem +Y | `p' = s·p + w·(stem_m − s·0.0989)·Ŷ` (head w 0, tip/keepers/retainer w 1); the whole valve moves −lift. Head radius 0.17 bore = the scene's disc |
| `valve_spring.glb` | its seat in the head, axis +Y | place at y = s·0.0566 in the closed valve frame (+ stem stretch); with lift scale y by (s·0.0378 − lift)/(s·0.0378) about its origin (coils close up as on a real spring) |
| `cam_lobe.glb` | camshaft +X, nose +Y at cam angle 0 | bucket (flat-follower) cam: envelope of the tappet face, convex (min radius of curvature 8.4 mm checked at build). Lift: **morph target 0**, weight = lift/(s·0.00946) − 1, exact (the profile is linear in the lift). Camshaft centre fixed at the closed valve tip + base radius; it turns at half crank speed |
| `rocker_arm.glb` | rocker-shaft axis (+X); the valve end and its adjusting screw at glTF −Z (Blender +Y) | rigid, uniform `s` (only for rocker-type heads, refs/2.png) |
| `wankel_*.glb` | shaft axis, same frame as `view3d::mesh::rotor` (rotor plane = glTF x–z = the 2D view's x–y, shaft +Y, apex 0 on +x), **real size and centred in y** | same e/R: scale (R/R_ref, w/w_ref, R/R_ref); other e/R: regenerate (the flank shape depends on it). Pose the rotor as `view::rotor_pose` (centre e·(cos 3θ, sin 3θ), turned θ), the eccentric lobe turns 3θ. Plugs at the waist (+z here = 2D +y) at x = ±0.18 R as the scene; rotor gear and stationary gear on the −y side, where the scene draws the side plate |
| `turbo_housing.glb` | shaft axis midway between the wheels; shaft +X, turbine +X, compressor −X | uniform `s = compressor_tip_diameter_m / 0.056`. The turbine housing's extras carry the ports in this frame: `inlet` (turbine inlet flange, opening up +Y), `outlet` (turbine outlet face, +X), `charge` (compressor outlet, up +Y), `eye` (compressor inlet face, −X), `hinge` (the wastegate flap's hinge) and `wheel_x` (each wheel's back face at ±`wheel_x`) |
| `turbo_compressor_wheel.glb`, `turbo_turbine_wheel.glb` | on the axis at the wheel's back face (inducer −X, exducer +X) | uniform `s`; placed at ∓`s·wheel_x`, turned about X by the wheel's angle (both on one shaft) |
| `turbo_wastegate_flap.glb` | on its hinge axis (+Z) | uniform `s`; placed at `s·hinge`, turned about +Z by the flap angle (positive opens it out of the outlet face) |

Counterweight radius is 0.74 bore: it clears the skirt at BDC with the reference rod; for a
short rod / long stroke check `s·0.0636 < rod_m − stroke_m/2 − 0.30·bore` and shrink it if not.

Skinned parts carry a standard glTF skin (joints `root`, `stretch`); the `gltf` crate reads
`JOINTS_0` / `WEIGHTS_0`. Rebuild the vertex buffer when the engine definition changes, never per
frame; per-frame motion stays a rigid `Mat4` per instance as now.

## Status (2026-11-04)

- piston: done, iterated on user feedback (curved pin-side panel, no proud strut, rounded skirt
  ends). Known nit: a faint shading smudge where the boss meets the panel (normals).
- conrod: done, stretch verified in Blender (`renders/conrod_stretched.png`); the preview
  framing crops the ends.
- crankshaft: done; webs reshaped to a fan (convex hull of pin boss, hub and counterweight)
  after user feedback. `renders/crank_i4.png` is an assembled 0-180-180-0 inline four;
  `crank_throw_long_stroke.png` shows the stroke stretch.
- Wankel: done. Rotor flank is the true inner envelope of the epitrochoid (computed; mid-flank
  75.00 mm = R − 2e exactly), not `mesh::rotor`'s Reuleaux triangle. Combustion recesses,
  apex/corner/side/oil seals, 51-tooth internal gear, bronze bearing. Housing with chromed
  trochoid, coolant and tension-bolt passages, plug bores + plugs; side plate with the
  34-tooth stationary gear; eccentric shaft. Ports not modelled (RENESIS has them in the side
  housings). `renders/wankel_front.png` matches the 2D view's pose at ψ = 0.
- Heavy meshes: wankel_rotor 36k triangles, piston 24k; fine for a few instances, decimate if a
  low-end preset needs it.
- valve train: done. Valve (tulip head, 45° seat, keeper grooves), split keepers, retainer,
  closed-and-ground spring, bucket-follower cam lobe (lift law: half-sine acceleration flank +
  constant deceleration to the nose, so a 0.11-bore lift fits a 0.22-bore base circle in ±80°
  cam and stays convex), rocker arm with adjuster. `renders/valvetrain.png`: a closed intake
  (base circle on the tip) and a fully open exhaust (nose down, spring compressed).
