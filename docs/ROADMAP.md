# Roadmap

The user's priority order (2026-10-27). Everything is physics, with every constant cited; nothing
is set by ear. Updated as work lands. Details live in the specs (`docs/superpowers/specs/`) and
`docs/FACTS.md`.

## 1. Performance: the LFA V10 at High on a Ryzen 7 PRO 5850U laptop

- ✅ v0.26.1, v0.26.3: −19 % CPU at Low, byte-identical sound (closed-form tail exit, angle wrap).
- ✅ v0.26.5: valve chatter at 96 kHz. At Low and Medium the valves' explicit exchange went on/off from step
  to step at idle (+45 dB of 5–10 kHz hiss that Ultra lacks). Fix: backward-Euler valve flow,
  stable at any step.
- Next, measured with callgrind on the LFA at High:
  - the ITB intake runners (41 % of the cost): four runners per SIMD lane, bit-identical;
  - the catalyst and silencer networks (19 %);
  - ✅ v0.26.6: the exhaust net on its own thread, a block at a time (−15–18 %);
- Then rebuild the quality levels from what each feature measurably changes against Ultra:
  measured (FACTS 2026-10-28) the step count is what matters, and idle is where all levels
  below Ultra are far off.
- Measured dead ends: SIMD of the ITB runners (−1.7 %), CFL-sized intake steps (bias the valve
  coupling; FACTS).

## 2. Realism ("still synthetic")

Measured against real LFA recordings: a missing stationary broadband floor at 2.5–10 kHz
(12–15 dB), and cycles that repeat too exactly (FACTS, 2026-10-27).

- Duct flow noise (Nelson–Morfey, Oldham–Ukpoho): done for the collectors (branch
  `feat/duct-flow-noise`). Measured ~30 dB short, so it is not the missing source by itself.
- Stronger sources:
  - jet and valve quadrupole noise above Mach 0.3 (Gordon, U⁸);
  - flow-excited chamber and tailpipe resonance (Davies & Holland 2001).
- Gear-change "bubbling" on the throttle cut (`--render gears`, v0.26.4).
- Ange's list:
  - #1 FV exhaust at idle;
  - #2 heat transfer along the exhaust pipes (branch `feat/pipe-heat`);
  - #4 gas species, partial burns and misfire at idle.
- Breathing at high rpm: real cam lift and duration, the K20A's exhaust VTEC lobe.
- References: real recordings (local only), GT7 clips and moving-car clips from the user (Doppler).

## 3. Low and Medium sound like Ultra

Measured as a third-octave distance to Ultra on held renders (idle, part load, full load), level by
level. The idle chatter above is the first gap.

## 4. Later

- **Potato mode**: our own `--render loops` grid (rpm × load, rendered once at Reference quality)
  played back crossfaded by rpm and load, as markeasting/engine-audio does with recordings. Near-zero
  CPU, for very slow machines; transients become blends, not physics.
- F1 presets: V12 3.5 L, V10 3.0 L, V8 2.4 L; turbocharger plus MGU-H, and the V6 1.6 L hybrid.
- Transmission noise: gear whine, straight-cut race gears vs helical road gears (once the engine
  sound is right).
- Diesel (compression ignition), rotary, turbo.
- Lamborghini-style tuned Helmholtz silencers (Onorati 2006); afterfire and backfire pops.
- 2D schematic of the intake and exhaust in the UI; 3D view last.
