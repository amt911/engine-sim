# 3D engine view — design (v1.0.0)

Status: draft for the user's approval (2026-11-03). Roadmap item `v1.0.0` of
[the engine-sim design](2026-10-21-engine-sim-design.md): "3D view of the engine — wgpu 3D render
of block, crank, pistons, valves, manifolds; 2D cross-section stays as an option".

## Intent

The user wants to *see* the engine they hear: the whole engine moving at its real speed, the
combustion in each cylinder, the gases flowing through intake, exhaust and turbo, the parts
coloured by temperature, with a camera they can turn — and to **change the engine graphically**,
dragging handles on the parts instead of hunting for sliders. What would make it fail even if it
works: **bad performance** (user, 2026-11-03). This project runs on modest machines (the user's
reference laptop is a Ryzen 7 5850U with integrated graphics).

## Decided with the user

- **Layout A, "everything in the scene"** (picked over a scene + fixed side panel): the 3D view
  takes the engine area of the window; editing is done by dragging handles on the parts; clicking a
  part opens a small **floating card** next to it with its exact numbers.
- **Content:** block, head, pistons, rods, crank, valves moving at the real speed; combustion in
  each cylinder; intake (air box, throttle, plenum/runners), exhaust (primaries, collectors,
  catalyst, silencers, tails) and turbo when the engine has one.
- **Layers the user toggles:** gas flow; temperature colours (with a legend); cut-away (to see
  inside).
- **Camera:** drag to orbit, wheel to zoom, click a part to select it.
- **Graphic editing of everything** the engine file holds that has a place in the scene.
- **The 2D cross-section stays** as an option (a toggle between 2D and 3D).

## Calls made by the agent (the user may overturn any)

- **Sound is untouched.** The view only shows what the simulation computes. Under load the view
  lowers its own detail; it never takes time from the sim thread or the audio.
- **Rendering inside the existing egui window with wgpu**, through eframe's wgpu paint callback
  (`egui_wgpu::CallbackTrait`): no game engine, no new window, no new heavy crate. Any new crate
  is named in the PR as the rules ask (likely only `glam` for the maths, if `egui`'s own types are
  not enough).
- **Meshes are generated from the engine definition**, procedurally (cylinders, boxes, swept
  tubes along each pipe's path), rebuilt only when the definition changes — never per frame.
  Moving parts are a few instanced meshes with per-instance transforms written each frame.
- **Temperature colours only where the simulation has a temperature.** Today that is the gas in
  each cylinder, the gas along the exhaust and intake pipes, the exhaust pipes' walls
  ([`pipe_heat`]) and the turbo's gas. The block, head and pistons have no thermal model yet (the
  cylinder wall is a fixed 400 K): they are drawn in a neutral colour with "no data" in their
  card — no invented temperatures (physics first). A lumped thermal model of piston, head and
  block is a separate later project.
- **Gas flow** is drawn from the pipes' own cells: colour by temperature (or pressure), small
  streaks moving at the local gas velocity. Each pipe sends at most 64 decimated cells per frame.
- **Combustion**: the chamber glows with the burned-gas temperature while the cylinder burns
  (`CylinderView::burning`, `temperature_k`); the spark is a short flash.
- **Editing a geometric value rebuilds the engine exactly as the editor's sliders do today**; the
  sound may have a short gap on release. While dragging, the handle shows the new value and the
  mesh previews it; the engine is rebuilt on release (so one drag = one rebuild = one undo step).
- **Cut-away** is a clip plane through the cylinders' axes, not a separate model.
- **Performance budget** (the user's one hard requirement):
  - 60 fps on the 5850U's integrated GPU at 1080p with all layers on, GPU time ≤ 4 ms/frame;
  - UI-thread CPU for the 3D view ≤ 2 ms/frame, no allocation per frame in steady state;
  - the snapshot the sim thread publishes grows by at most a few kB;
  - three detail levels (mesh segments, streak count, shadows off/on) chosen automatically from
    the measured frame time, like the sim's quality levels, and settable by hand;
  - measured by a bench (frame time on the 5850U profile and on this machine) before each phase
    merges, as the hot-loop rule requires.

## Goals, non-goals, anti-goals

- **Goals:** the content and editing above; every engine and preset the app loads (inline, V,
  flat/boxer, rotary, diesel, turbo) shown correctly; 2D still available.
- **Non-goals:** photorealism (materials stay simple: lit flat colours, the temperature layer);
  CFD inside the cylinder (the sim is 0D in the chamber, so the flame is drawn from its burn
  state, not simulated in 3D); the car body; VR.
- **Anti-goals:** a view that costs the audio a single dropout; temperatures or flows the
  simulation does not compute; a second source of truth for geometry (the scene is always derived
  from the `EngineDef`).

## Phases (each a PR from `main`, each usable on its own)

1. **Scene and camera:** procedural meshes of block, head, crank, rods, pistons, valves for every
   layout; orbit/zoom; parts moving from the snapshot; 2D/3D toggle; the frame-time bench.
2. **Intake, exhaust and turbo geometry:** pipes swept along a path laid out from the definition's
   lengths (pipes are 1D in the sim: their 3D path is a layout choice, drawn compactly and
   readable, never claimed to be the real car's routing).
3. **Layers:** temperature colours with legend; gas flow streaks; combustion glow; cut-away.
4. **Picking and floating cards:** click a part → card with its numbers (read-only first).
5. **Handles:** drag handles for the geometry (bore, stroke, rod, valve sizes and timing, pipe
   lengths and diameters, plenum and silencer volumes…); the card's numbers become editable;
   undo through the editor's existing undo.

## Verification

- Pure logic in `engine-app` (mesh generation, layout of the pipes, camera maths, picking ray,
  handle → value mapping) gets unit tests with hand-computed values, each seen failing once.
- Render: off-screen wgpu snapshots of each preset per phase (`egui_kittest` with `wgpu`, as
  `render_screens` already does) — reviewed by eye, never a desktop capture.
- Handles: `egui_kittest` drags a handle and reads the engine definition it changed.
- Performance: the frame-time bench, before/after, in each PR.
- Smoke: the GUI launch in `scripts/smoke.sh` with the 3D view on.

## Left to the builder

Shader structure, the exact mesh resolution per detail level, how pipe paths are folded to stay
compact, the colour maps (perceptually uniform, readable for colour-blind users), and how
handles look — within the budget and rules above.
