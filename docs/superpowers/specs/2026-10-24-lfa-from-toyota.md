# The LFA as Toyota built it — spec (2026-10-24)

Source: Okamoto, Kawamura, Tsukamoto et al., *New Toyota 4.8L V10 Petrol High Performance Engine for
Lexus LFA*, SAE 2011-01-2024 / JSAE 20119523 (`papers/okamoto2011.pdf`, local). User feedback
(2026-10-24): "el V10 suena cada vez peor"; "se parece cada vez menos al del Lexus LFA".

## What the paper says, and what the preset has

| | Toyota (paper) | Preset today |
| --- | --- | --- |
| Bore × stroke, CR | 88 × 79 mm, 12.0 (Fig. 3) | same |
| Con-rod | 143.25 mm (Fig. 3) | 140 mm |
| Rated | 412 kW / 8700, 480 N·m / 7000 (Fig. 3) | dyno test against these |
| Intake | per bank: air cleaner case → **surge tank, "400 Hz tuned by surge tank"**, an "800 Hz tuned by passage", throttles "inside the intake manifold close to the intake port" (§5.1, Fig. 18); each bank "completely separated intake and exhaust system" (§7) | ten ITB trumpets open to the atmosphere |
| Exhaust | per bank: long equal-length manifold → **front and rear three-way catalyst**, metal substrate ("low exhaust pressure … less cell number") → rear silencer (Fig. 19, §7) | two 5-into-1 collectors → straight tail pipes |
| Sound | "sound exciter" pipe from the surge tank into the cabin (§1.2, Fig. 18) | — (an in-cabin path; the listener here stands outside) |

## Phase 1 — the surge tank, in 1D (this branch)

The trumpets draw from a surge tank instead of the atmosphere, and the tank breathes through an
inlet duct.

**Why 1D (user, 2026-10-24: "pero debería ser 1d, no?").** A 0D tank is only valid well below the
tank's first standing wave. At 400 Hz the wavelength is 0.86 m, and a tank running along a bank of
five cylinders 97 mm apart (paper, Fig. 3) is ~0.49 m long. Its half-wave mode, c/2L ≈ 350–400 Hz,
is very likely the "400 Hz tuned by surge tank" itself, and a 0D tank cannot have it at all. The
"800 Hz tuned by passage" is a duct mode too.

**Model.** One tank per group of runners (the LFA: per bank):
- The tank is a quasi-1D pipe of the tank's cross-section along its length, cut into stretches
  between one junction per runner. The junctions are evenly spaced, cylinder by cylinder.
- At each junction a small 0D node joins the stretch on each side and that runner's trumpet.
  This is Benson's constant-pressure junction, the one the FV exhaust's collectors already use
  (`junction_ghost`); the runner keeps its throttle (`throttled_ghost`).
- The inlet duct (1D) joins the tank's first end and opens to the atmosphere. The tank's far end is
  closed.
- The intake sound radiates from the inlet's open end (ρc·u), not from the trumpets.
- A tank couples its runners every substep, so the runners of a tank step together; the detached
  worker stays off with tanks.

**Data:** `intake.surge_tanks`, one entry per group (runners split into equal consecutive groups, so
the LFA's 1–5 and 6–10 are its banks), each with:
- the tank's length and cross-section area;
- the inlet duct's diameter and length.

Default: none (open ITBs, as today).

**LFA values:**
- Tank length = 5 × 97 mm = 0.485 m (cylinder spacing, paper Fig. 3).
- Cross-section 0.1 × 0.1 m. **Estimate without a source:** room for the trumpets, of the runner's
  ~42 mm bore and about twice that at the mouth.
- Inlet: 70 mm × 0.2 m. **Estimate without a source.**
- The paper's 400 Hz is the check, not a fit. The test measures where the tank's resonances land.

**Acceptance:**
- In the low-frequency limit (a tank much shorter than the wavelength) the 1D tank rings at the
  Helmholtz frequency (c/2π)·√(A/(V·L)) (±3 %). This is the test written for the 0D draft, kept.
- A long tank closed at one end, excited at the closed end, shows its c/2L standing wave (±3 %).
- The LFA still breathes as Lexus published (dyno test).
- Measured and reported: where the LFA tank's modes land against the paper's 400 Hz, and the
  radiated intake spectrum against the open ITBs.
- JSON/YAML round trip of the new field.

## Phase 2 — catalysts and the rear silencer (next)

Each bank gets two catalysts (front and rear) and both banks feed a rear silencer. The paper names
the parts but gives no dimensions. They will be sized from the engine's flow (catalyst face area for
the space velocity, silencer volume against displacement), with the estimate written down as in
Phase 1.
