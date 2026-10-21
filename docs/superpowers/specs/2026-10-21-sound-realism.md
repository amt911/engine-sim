# Sound realism pass — spec (2026-10-21)

Status: approved by delegation (user: "suena muy sintético"). Research doc §"Cómo se fabrica el
sonido" (engine-sim's chain) and §"De la presión simulada a 48 kHz".

## Why it sounds synthetic

The source is a deterministic, noiseless pressure signal: every cycle identical except a ±4 %
energy jitter; no turbulence, no intake, no mechanics, heard "dry" (no body, no room).

## Additions (in the voice chain unless noted)

1. **Turbulent flow noise** (core): broadband noise at the tail outlet whose amplitude follows the
   outflow velocity (jet noise ∝ u^n; engine-sim mixes noise modulated by the flow). Band-limited
   (≈ 0.5–6 kHz). Seeded RNG → deterministic.
2. **Cycle-to-cycle variation** (core): besides the energy, jitter the burn start (±1–2°) and
   duration (±5 %) — real combustion variability (Heywood ch. 9) gives the "lumpy" texture.
3. **Intake sound** (core): induction roar from the plenum pressure fluctuation through the
   throttle (derivative of throttle mass flow), mixed at a lower level.
4. **Mechanical noise** (core/voice): valvetrain ticks (impulse at each valve closing, amplitude
   ∝ rpm) and a faint crank-order hum.
5. **Body / room** (voice): short convolution reverb (synthetic exponentially decaying noise IR
   ~120 ms, partitioned FFT not needed at this length → direct FIR at 48 kHz is too costly; use a
   small feedback-delay network: 4 delay lines + Householder matrix). Dry/wet mix.
6. **Soft saturation**: keep tanh, add gentle compression (slow AGC) so idle is not inaudible and
   WOT does not clip.

## Cases

- Same seed → identical output (determinism kept).
- Noise level rises with throttle/flow (WOT noisier than idle).
- No NaN, output in [-1, 1], realtime: sim + voice faster than 1× realtime for the LFA in release.
- Each addition has a level parameter (0 disables it), in `EngineDef.sound` (serde, defaults).
