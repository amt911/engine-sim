# Realistic 3D Parts in the 3D View — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the 3D view draws the user's realistic parts from `assets/3d-parts/out/*.glb` instead of the procedural cylinders and boxes. Each part is stretched to the engine's bore, stroke and rod by the README's rules.

**Architecture:**

- **New mesh kinds.** Each realistic part becomes a new `MeshKind` (`PistonGlb`, `ConrodGlb`, `CrankThrowGlb`, …). Its vertex buffer is built once per engine definition (the existing `MeshKey.def_version`): the README's stretch rule is applied, then the part is expressed in the **unit frame of the part it replaces**.
  - Vertex positions: `p_unit = S⁻¹·(p_world − t)`, where `S` and `t` are the scale and offset that `scene::moving` already gives that part's instance.
  - Normals: `n_unit = S·n`. The shader divides by `s²` (`scene.wgsl:37`), so the lighting comes out right.
  - What stays as it is: instance matrices, picking, handles and cards.
- **Loading.** The `.glb` are embedded with `include_bytes!` (a single binary) and parsed with the `gltf` crate: positions, normals, `JOINTS_0`/`WEIGHTS_0` and morph target 0.
- **Detail levels.** **Low keeps the procedural meshes** (the user's one anti-goal: bad performance); Medium and High use the realistic parts. Parts that the realistic one already contains are not drawn on their own: the rings, wrist pin and rod eye go with the piston and the rod.

**Tech Stack:** Rust, wgpu through eframe, `glam`. New crate `gltf` (MIT/Apache-2.0, the reference glTF 2.0 reader; nothing in the workspace reads glTF today).

**Spec:** `docs/superpowers/specs/2026-11-03-3d-view-design.md`. Assets and rules: `assets/3d-parts/README.md`.

## Global Constraints

- Performance budget (spec), measured with `bench_3d_frame` before/after: 60 fps on the 5850U iGPU at 1080p; GPU ≤ 4 ms/frame; UI CPU ≤ 2 ms/frame; no per-frame allocation.
- The mesh is rebuilt only when the definition changes, never per frame (README: "Rebuild the vertex buffer when the engine definition changes, never per frame").
- The frames and stretch rules are exactly the README's table; the reference engine is bore 86 mm, stroke 86 mm, rod 143 mm.
- The user sees renders (`render_screens`, off-screen) of each step before the next is built on it.

## Review Focus

1. **Short rod or long stroke.** The README's clearance rule (`s·0.0636 < rod − stroke/2 − 0.30·bore`) fails and the counterweight cuts through the skirt. Shrink it as the README says (test: the K20A and a 60 mm-rod edit stay clear).
2. **A bore far from 86 mm** (40 mm single, 140 mm diesel). The uniform scale must keep the piston's crown where the procedural one had it, so the handles and the card still point at the right place.
3. **Rotary engines.** An e/R other than the reference's means regenerating the part. Until then, keep the procedural rotor whenever e/R differs from the asset's by more than 1 %.
4. **A broken or missing `.glb`** (a bad rebuild). The view falls back to the procedural mesh and logs once; it never panics.
5. **The LFA V10 at High**, with 10 pistons and rods at 24k triangles each. Measure; if over budget, Medium uses a decimated copy.

---

### Task 1: Loader and the piston

**Files:**
- Create: `crates/engine-app/src/view3d/parts.rs` (`pub mod parts;` in `view3d/mod.rs`).
- Modify: `crates/engine-app/src/view3d/render.rs` (`unit_mesh`), `crates/engine-app/src/view3d/scene.rs` (`MeshKind`, `Part::mesh`, skipping the ring and pin instances when the realistic piston is drawn), `crates/engine-app/Cargo.toml` (`gltf`).

**Interfaces:**
- `pub struct GlbPart { pub positions: Vec<Vec3>, pub normals: Vec<Vec3>, pub stretch_w: Vec<f32>, pub morph0: Vec<Vec3>, pub indices: Vec<u32> }`
- `pub fn load(bytes: &[u8]) -> Result<GlbPart, PartError>`: merges the file's meshes in their node transforms; `stretch_w` is the weight of joint `stretch` (0 without a skin).
- `pub fn piston_unit_mesh(def: &EngineDef) -> Mesh`: scale `s = bore/0.086`, then into the procedural piston's unit frame (`scene.rs:456`, `0.49b × 0.6b × 0.49b`).

- [ ] **Step 1: Failing tests**
  - `the_piston_asset_loads_with_its_reference_size`: from `load(include_bytes!(…piston.glb))`:
    - the bounding box's x and z span equals the bore, 86 mm (±0.5 mm);
    - every normal has unit length (±1e-3);
    - every index is in range.
  - `a_bigger_bore_scales_the_piston_and_keeps_its_frame`: with bore 100 mm, the world-space box (unit mesh × the instance's S) spans 100 mm across (±0.6 mm); its crown top is where the procedural crown was (±0.5 mm).
  - `a_bad_asset_falls_back` (Review Focus 4): `load(b"nonsense")` is `Err`; `unit_mesh` for the piston then returns the procedural cylinder.
- [ ] **Step 2: Run them.** Expect FAIL.
- [ ] **Step 3: Implement.** Use the `gltf` crate's `import_slice`. Medium and High return the piston's unit mesh; Low the cylinder.
- [ ] **Step 4: PASS, then sabotage.** Drop `/S` in the unit-frame step: the frame test must fail.
- [ ] **Step 5: See it.** Run `render_screens` for the K20A and the LFA (3D) and look at the renders (never a desktop capture). Run `bench_3d_frame` before and after, and put both in the PR.
- [ ] **Step 6: Commit.** `feat(app): the realistic piston in the 3D view`.

### Task 2: The connecting rod, stretched

- `conrod_unit_mesh(def)`: `p' = s·p + w·(rod − s·0.143)·Ŷ` (README), into the procedural rod's unit frame. The rod eye is not drawn on its own at Medium/High.
- Test `the_rod_stretches_its_small_end_only`:
  - rod 160 mm: the small-end centre is 160 mm above the big end (±0.5 mm);
  - the big-end bore keeps its size (w = 0 there);
  - sabotage (w = 1 everywhere) must fail.
- Render, bench, commit `feat(app): the realistic connecting rod`.

### Task 3: The crankshaft

- `crank_throw_unit_mesh(def)`: the README's stretch rule `w = clamp(y/0.043)`, plus the counterweight check of Review Focus 1. Also the nose and flange at the ends.
- One throw per cylinder at `x = i·pitch`, turned by its crank angle (the scene's throws).
- Tests:
  - `the_throw_reaches_the_stroke`: the pin centre at stroke/2 from the axis (±0.3 mm), for stroke 86 and 100 mm;
  - `the_counterweight_clears_a_short_rod` (Review Focus 1).
- Render the 0-180-180-0 inline four, the V10 and the flat-plane V8. Commit.

### Task 4: The valve train

- Valve: stem stretch; the whole valve moves by −lift. Spring: scaled in y by its compression.
- Cam lobe: morph target 0, weight `lift/(s·0.00946) − 1`, turning at half crank speed. These are new moving instances.
- Rockers only on rocker heads (none of the presets: skip unless a preset needs it).
- Tests:
  - `the_valve_head_matches_the_scenes_disc` (radius 0.17 bore);
  - `the_spring_closes_up_with_lift`;
  - `the_cam_nose_reaches_the_lift`.
- Render, commit.

### Task 5: The Wankel

- Rotor, housing, side plate and eccentric shaft at their real size: scale (R/R_ref, w/w_ref, R/R_ref) when e/R matches, else procedural (Review Focus 3).
- Test `the_rotor_flank_waist_is_r_minus_2e`: mid-flank 75.00 mm on the reference.
- Render the Renesis and the 13B and compare with the 2D view's pose. Commit.

### Task 6: PR

- Full local gate, verify-pr and GPU kittests. The renders of every preset go in the PR (SendUserFile) for the user's eye before merging.
- Update `docs/ROADMAP.md` and `CHANGELOG.md`, then tag.
