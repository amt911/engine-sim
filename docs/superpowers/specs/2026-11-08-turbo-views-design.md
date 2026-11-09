# The turbo, seen working, in the 2D and 3D views

Status: approved by the user (2026-11-08).

## Why

The user (2026-11-08): "me gustaría que el turbo se viera también en la vista 2d y 3d". Asked what
mattered most, they chose **realistic and working**: a turbo that looks like one and shows what it
is doing (the wheels spinning with the shaft, the wastegate opening, the boost).

Today the 3D view draws a turbo as two plain cylinders (turbine, compressor) and a shaft
(`view3d/ducts.rs::turbo`) with a card of its sizes; the 2D view draws no turbo at all. The sim
already computes the turbo's state (`engine_core::turbo::TurboState`: boost, its temperature,
turbine inlet pressure, shaft rpm, blow-off flow), but the views never see it: `Snapshot` does not
carry it.

## Goals

- **3D:** a realistic turbo built from the definition's sizes, in the same pipeline as the other
  realistic parts (a parametric Blender script in `assets/3d-parts/`, its `.glb` stretched by a
  rule in its README). It is made of:
  - the compressor housing (volute, inlet, outlet);
  - the centre section (bearing housing);
  - the turbine housing (volute, inlet flange, outlet with the wastegate port);
  - the compressor wheel (its `blades` main blades, backswept, and splitters);
  - the turbine wheel;
  - the wastegate flap.
- **3D, working:**
  - the wheels turn with the shaft;
  - the wastegate flap opens as much as the sim's gate is open;
  - the cut-away layer opens the housings to show the wheels;
  - the temperature layer colours the turbine side by its gas and the compressor side by the
    charge;
  - the turbo's card shows its live values: shaft speed, boost and wastegate opening.
- **2D:** when the engine has a turbo, a turbo panel beside the cylinders. It shows:
  - both volutes seen along the shaft, the compressor on one side and the turbine on the other;
  - each wheel with its blades, turning with the shaft;
  - the gas in the volutes coloured by temperature, as the cylinders are;
  - the wastegate flap opening;
  - three readouts: shaft speed in krpm, boost in bar, and wastegate opening in %.
- **Variable geometry (TDI):** the vanes' position as a readout in both views, and the vanes as a
  ring in the 3D cut-away.

## Non-goals

- No change to the turbo's physics or its sound.
- No surge or choke display. The model leaves those out (`turbo.rs`, "Left out").
- No editing handles on the turbo in this work. The card stays read-only for the turbo, as today.

## Decisions (agent's calls, for the user to check)

- **Wheel speed on screen.** A real shaft turns at 100–200 krpm, up to 3300 revolutions a
  second. At 60 frames a second that would strobe into a random or backwards picture. So the
  wheels are drawn at **1/1000 of the shaft speed**: 150 krpm shows as 150 rpm, still clearly
  faster or slower as the turbo spools. The true speed is always in the readout.
- **Sizes.** Everything scales uniformly from the compressor wheel's tip diameter, the only size
  the definition gives (the reference part is drawn at a 56 mm tip, ESTIMATE: a K20C1-class
  wheel). The proportions come from photographs of a typical automotive turbo and are marked
  ESTIMATE in the script, as the current scrolls' 2.5 wheel diameters already are.
- **Placement.** The turbo sits where it sits now (at the collectors' end, compressor on the same
  shaft ahead of the turbine, charge pipe to the throttle). Only its look and motion change.
- **New state the views need.** `TurboState` gains three fields: `wastegate_open` (0–1, the
  gate's fraction the sim already computes), `vanes_open` (0–1, for variable geometry) and
  `turbine_inlet_t_k` (the exhaust temperature the turbine is stepped with). `Snapshot` gains
  `turbo: Option<TurboState>`. The exhaust pipe probe was tried for the turbine's gas and dropped:
  at Ultra its collectors' mean read as cold as the charge air.

## Pieces and order: two PRs, each from `main`

1. **State and the 2D panel.**
   - The new `TurboState` fields, each with a unit test.
   - `Snapshot.turbo`.
   - The 2D turbo panel.
   - Its geometry in pure functions with unit tests: the blade angles and the flap angle.
   - The off-screen screen render for the K20C1 and the TDI.
2. **The realistic 3D turbo.**
   - `assets/3d-parts/scripts/turbo.py`, its `.glb` files, and `verify.py` checks.
   - Its README rule.
   - Loading it in `view3d/parts.rs` in place of the plain cylinders, keeping the same picking
     and card.
   - Wheels and flap posed from `Snapshot.turbo`.
   - The cut-away and temperature layers.
   - Tests: the scene's instances, the wheel angle advancing with the shaft speed, and the flap
     angle following the gate.
   - The renders go to the user before building on them, as for every 3D part.

## Done when

- Running the K20C1 or the TDI, both views show the turbo:
  - the wheels speed up with the boost when you floor it;
  - the flap opens when the boost reaches its set point;
  - the readouts match the gauges.
- The naturally aspirated presets look exactly as before.
- Gates: fmt, clippy, the tests, coverage, `scripts/smoke.sh` (GUI 2D and 3D), and the verify-pr
  verdict.
