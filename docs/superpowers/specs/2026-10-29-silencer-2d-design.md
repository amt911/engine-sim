# Packed silencers from a 2D model — design

Requested by the user (2026-10-29: "implementa un modelo 2d si no consume mucho"). Replaces the 1D
lumped lining (`Shunt::lining`) of a packed `Segment::Muffler` in the waveguide exhaust.

## Why

The 1D lining (low-frequency limit of a locally reacting layer, plane waves) against Selamet, Lee &
Huff, JSV 262 (2003) Fig. 7 (measured): 3.7/10.3/20.0/32.7/47.2/62.7 dB at 0.5–3 kHz vs measured
21/29/36/30/22/18 — too little below 1.5 kHz, up to 45 dB too much above (no beaming). A 2D
axisymmetric mode-matching model (Selamet, Xu, Lee & Huff, JASA 117, 2005), prototyped in
`papers/silencers/lining-study/modal2d.py`, gives 20.0/29.8/35.4/22.7/16.7/15.7 (FACTS).

## What

1. `engine-core::silencer2d`: transmission loss of a straight-through perforated dissipative
   silencer (tube r1, shell r2, length L, fibre flow resistivity R; perforate porosity 8 %, wall
   0.9 mm, holes 2.49 mm — the paper's, the defaults) in gas of (c0, ρ0): a port of modal2d.py.
   Complex Bessel functions from the `complex-bessel` crate (pure Rust, Amos Alg. 644). Fibre:
   Nice's fit at the gas's density (ρf/R scaled by ρ0/1.1555). Packing porosity is not a parameter of
   that fit and stays unused by this model.
2. Realisation: the packed muffler becomes plain pipes of the tube's bore carrying, in each
   direction, a cascade of peaking biquads whose magnitude follows −TL(f) — octave centres
   63 Hz–16 kHz, gains by least squares on the bands' interaction (Välimäki & Liski, "Accurate
   cascade graphic equalizer", IEEE SPL 24(2), 2017), two refinement passes. Minimum phase.
   Reflection at the silencer's ends neglected (straight-through, same bore in and out); the
   2D model's reflection coefficient is checked small in a test.
3. Cost: TL on ~24 frequencies per packed silencer when the net is built (target ≤ 2 s,
   measured); then 2 × 9 biquads per step per packed silencer.

## Tests (values from outside the code)

- Empty chamber (no fibre): TL zeros at kL = nπ (f = c/2L, 667 Hz for L = 0.2572 m) and the
  closed-form expansion-chamber dome 10·log10(1 + ¼(m − 1/m)²) between (Munjal ch. 3), within
  the perforate's small shift.
- Filled (R = 4896): within 6 dB of the measured 21/29/36/30/22/18 at 0.5–3 kHz.
- The biquad cascade follows a given TL curve within 1.5 dB at the octave centres.
- Guard: `the_lfa_at_full_throttle_keeps_the_real_cars_orders` stays green (sound kept).

## Left to the builder

Root-finding details (grid size, continuation across frequency), number of modes (14 chamber,
6 airway as the prototype), the sim's gas state (the tail's `C_TAIL`, `TAIL_T`).
