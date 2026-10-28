# Valve flow noise on the port's scale — design

Status: **approved** by the user (2026-10-28). The user asked for a design ("diséñalo") of the turbulent noise
of the valve jets; this document is that design, for approval before any code.

## Why

"It sounds synthetic" was measured as too much of the sound repeating cycle to cycle (FACTS):

- At **idle** this was mostly a measurement artifact. `--render held:` pins the crank speed; idling
  free (`--headless --throttle 0`), the LFA repeats 0.48 / 0.42 / 0.15 / 0.02 of its 125 / 250 /
  500 / 1000 Hz octaves, the same as the real LFA (0.52 / 0.50 / 0.19 / ≈0) and a real Viper V10
  (0.38 / 0.48 / 0.12 / 0.01). The cross-plane V8 is close; the flat-plane V8 is still too
  regular (0.76 at 500 Hz, 0.41 at 1 kHz, against 0.22 and 0.02 for a real Audi RS4).
- At **load** the gap is real: in real LFA pulls at 6900–8100 rpm, 28 % of the 1–4 kHz energy sits
  on the engine orders; in renders, 76–84 % (spectral subtraction: the render lacks a floor
  15–22 dB under the real one at 2.5–8 kHz). On the road the car's inertia steadies the crank, so
  this comparison is fair.

A random source at load is missing, broadband from about 1 kHz up, and strong enough to carry
roughly 2–3 times the orders' energy in 1–4 kHz.

## What the existing sources miss

- `jet_noise::ValveJet` (Lighthill / SAE ARP876 mixing noise of the valve's jet) and Tam's
  broadband shock-associated noise (Tam 1990, eq. 3.7) both scale with the jet's own width. For a
  valve curtain that is the lift (0.5–10 mm), so at u ≈ 600 m/s the mixing peak (St = f·D/u ≈ 0.2)
  sits at 10–100 kHz and the shock-noise peak (f ≈ u_c/L_s, L_s ≈ 1.1·D_j·√(M_j² − 1)) at
  30–60 kHz. Little of either is audible, which is what the ablation showed (removing the valve
  jet changes nothing at idle).
- The audible broadband comes from the turbulence the jet leaves in the port and primary, mixed
  out on the pipe's scale (D ≈ 35–42 mm): at 100–300 m/s and St ≈ 0.2 that is 0.5–2 kHz with a
  slow roll-off above, the band that is missing.

## The model

The valve as a flow obstruction ("spoiler") in its port, the case Nelson & Morfey (1981) and
Oldham & Ukpoho (1990) treat: the wake's fluctuating drag is a dipole whose band power follows
from the obstruction's pressure loss through a Strouhal function K²(St) built on the duct's
diameter. It is already implemented and tested for the collectors on `feat/duct-flow-noise`
(`flow_noise.rs`: `Spoiler`, `k2`, `band_power`, the `FlowNoise` synthesizer), where the
collectors' small loss (~0.5 kPa) made it ~30 dB too quiet. A valve's loss is up to two orders of
magnitude larger, and the sound power grows as the cube of it.

Per cylinder and exhaust valve, every physics step:

1. The loss across the valve: Δp between the cylinder and the port, from the valve boundary's
   solution (`valve_boundary`); the port velocity u = ṁ/(ρ·A_port).
2. The spoiler: loss coefficient C_L = Δp/(½ρu²) on the port's area, Strouhal on the port
   diameter (`flow_noise::spoiler`).
3. The sound: band powers from K²(St) synthesized as a random pressure (the existing
   `FlowNoise`), injected into the primary as a wave, as `ValveJet` does now (half the power
   downstream, the other half into the cylinder, where it is lost).
4. Where the jet is choked, Nelson–Morfey is past its range (its data go to M ≈ 0.3 in the
   duct). There the flow is limited to its choked value and the jet's fully expanded Mach number
   M_j is known; Tam's shock-noise term (eq. 3.7, hot-jet factor included) is added, with D_j
   from the curtain. Most of it falls above the audible band, so it is kept only if the
   feasibility step shows it matters in band.

The intake valves get the same treatment (into the runner, out of the mouth) only if the
feasibility step shows they matter at load. They are not choked at WOT, so the loss is small.

Every constant comes from the papers: K²(St) and its fit from Oldham & Ukpoho; Tam's c̄, S₀, the
core length and convection velocity from eq. 3.7 and the text after it. Nothing is set by ear.

## Feasibility first (plan task 1, no product code)

Before building anything, with a probe on the LFA held at 7400 rpm WOT:

- log Δp, ṁ and the port gas per valve step;
- compute the predicted in-port band power (Nelson–Morfey, and Tam where choked) per third octave
  from 500 Hz to 10 kHz;
- compare with the energy needed: the render's order energy per band × (1 − s)/s, with s the real
  share (28 % at 1–4 kHz).

**Gate:** if the prediction falls more than 10 dB short of what is needed in 1–4 kHz, stop and
report back. The source is real but not the missing one, and the next candidates are the
silencer's internal flow (Davies & Holland 2001) and the tail pipe's jet at the pipe's scale.

## Acceptance

- At load: the share of 1–4 kHz energy on the orders falls from ~80 % to within ±10 points of the
  real recordings (LFA 28 %, BAC Mono 22 %). It is measured on a held render, which is fair at
  load, against the same recordings as FACTS.
- At idle: no regression on the free-running shares above. The flat-plane V8 is tracked but not
  a goal of this design.
- Torque, idle speed and the existing tests are unchanged; the new layer is
  `sound.valve_flow_noise` (default 1, 0 = off) with a JSON/YAML round trip.
- Cost: ≤ 3 % at every quality (5 s LFA headless, min of 3).
- Every quality level gets it (it is a source, not a numerical refinement).

## Non-goals

- Impulse-response convolution, room or "environment" effects: a linear filter does not change
  how much of a band repeats.
- Tuning any level by ear or per preset.
- The flat-plane V8's idle regularity (a separate question: firing evenness and CCV).

## Open decisions, with the defaults I will take unless told otherwise

- **Port diameter.** It comes from the exhaust primary's diameter (the port is not in the
  definition). An estimate, stated in the code.
- **Dipole radiation in a duct.** Below the duct's first cross-mode cut-on (~5 kHz for 40 mm hot
  gas) the dipole's power travels as plane waves, half each way (Nelson & Morfey §2). Above it,
  the plane-wave share is used anyway, as a lower bound.
- **Tam's D_j for an annular curtain.** Hydraulic diameter of the curtain (2 × lift), an
  assumption recorded in FACTS. It only matters if the feasibility step keeps Tam's term.

## References (all in `papers/`)

- Nelson, Morfey 1981, *Aerodynamic sound production in low speed flow ducts*, JSV 79(2).
- Oldham, Ukpoho 1990, *A pressure-based technique for predicting regenerated noise levels in
  ventilation systems*, JSV 140(2).
- Tam 1990, *Broadband shock-associated noise of moderately imperfectly expanded supersonic
  jets*, JSV 140(1), eq. 3.7.
- Davies & Holland 2001 (fallback candidate), Davies 1988 and 1999.

## Result of the feasibility step (2026-10-28): NO-GO

LFA held at 7400 rpm WOT, cylinder 0, 192 kHz: the exhaust valve as a spoiler in a 42 mm port
(C_L median 7.8, Δp up to 445 kPa, u_c capped at Mach 0.3 on 98 % of the open steps) generates
≈ 71 dB re 1 pW per third octave downstream, against 141–148 dB of the pulse waves the valve sends
in the same bands: **−73 dB** in 1–4 kHz where +4 dB was needed. Lifting the Mach cap would gain
20–30 dB at most. Not built. The other candidates checked the same day are in FACTS (tail-pipe
jets, crank coupling, cycle-to-cycle combustion, non-linear propagation): none closes the gap.
