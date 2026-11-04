# Roadmap

The user's priority order, kept current: **every session reads it when it starts and updates it
when a milestone lands** (with the CHANGELOG row and the tag). Everything is physics, with every
constant cited; nothing is set by ear. Each item has its design in `docs/superpowers/specs/` and,
once planned, its plan in `docs/superpowers/plans/`; verified facts live in `docs/FACTS.md`.
Last updated: 2026-11-04.

## Done (details in `CHANGELOG.md`)

- Engine, exhaust (waveguide and finite volume), intake, drivetrain, quality levels, presets
  (single, inline-4, K20A, K20C1, LFA V10, V8s, EJ25 boxer, TDI/SDI diesels, Renesis/13B rotaries)
  — up to v0.41.
- Turbo (v0.39.0), diesel (#88).
- **3D view** (spec `2026-11-03-3d-view-design.md`), all five phases:
  - scene and camera (v0.40.0);
  - pipes (v0.42.0);
  - layers: temperature, flow, combustion, cut-away (v0.43.0);
  - click a part for its card, drag handles or type on the card (v0.44.0);
  - silencer handles (#93).
  - The editor has no undo, so the 3D view has none either.

## Now: transmissions and hybrid

Spec `2026-11-04-transmissions-hybrid-design.md`, approved 2026-11-04 with **transmission noise**
added by the user. One PR each, in this order:

1. **Gear noise** on the existing manual box (plan `2026-11-04-gear-noise.md`):
   - whine at the mesh orders (spur vs helical);
   - rattle of the unloaded gears.
2. **Sequential** (dog box, flat shift, straight-cut whine, the dogs' engagement impacts).
3. **DSG** (dual clutch).
4. **Torque-converter automatic.**
5. **CVT** (belt pitch whine).
6. **eCVT** (power split).
7. **Hybrid**, P2 parallel first:
   - permanent-magnet motor whine (electrical orders, PWM side bands);
   - a battery with its gauge;
   - the energy manager.

## Parked at the end (user, 2026-11-03): petrol idle realism

Petrol idles still sound "diesel/tractor". The user rejected these by ear, so none come back:

- piston slap;
- timing chain;
- a fitted noise floor;
- Gaussian main-bearing noise;
- road silencers on the petrol presets.

Measured leads are in the memory and FACTS notes of 2026-11-03.

## Standing processes

- **Mutation testing**, per PR, on the `engine-core` diff:
  - runs in the background and never blocks a merge;
  - the result goes to the PR as a comment, misses to a follow-up PR;
  - the user turns it off or on with `scripts/mutants-mode.sh off|on`.
- **Sound changes go to the user's ear** before anything is built on them. Send before/after
  clips, loudness-matched.

## Later (not scheduled)

- **Potato mode:** our own `--render loops` grid (rpm × load, rendered once at Reference quality),
  played back crossfaded by rpm and load. Near-zero CPU, for very slow machines.
- **F1 presets:** V12 3.5 L, V10 3.0 L, V8 2.4 L; the V6 1.6 L turbo hybrid with MGU-H.
- **Lamborghini-style tuned Helmholtz silencers** (Onorati 2006); afterfire and backfire pops.
- **Low and Medium quality closer to Ultra**, measured as a third-octave distance on held renders.
