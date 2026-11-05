# Every noise source at its measured level, then the exhaust's own damping — design

Status: direction approved by the user 2026-11-05 ("me parece bien, siempre que sea basado en
física y suene bien de una vez por todas"). This document needs the user's approval.

## Why

Measured today (scratchpad tools now in `scripts/realism/`; `docs/agents/debugging.md` →
*Sound realism*):

- Our treble rides the firing rhythm 30–150 % (real steady recordings: 9.6 % median, p25–p75
  7.5–17.9 %); the user hears it as "ronco + ametralladora".
- Adding the duct flow noise (U⁴, branch `feat/duct-flow-u4`, built and tested, not merged)
  cannot fix it with any constant: where the texture becomes real-like the band balance is
  20–30 dB too bright ("vapor, sintético"). The flow noise itself follows our flow, which pulses
  far more than a production car's.
- **Per-source absolute levels against published measurements** (inline-4, full load, our 3 m
  listener converted to the papers' distances):

  | Source | Measured on production petrol engines | Ours |
  |---|---|---|
  | Engine structure (block + mechanical) at 1 m | 88–92 dB(A) at 3000 rpm, 98–105 at 5000–6000 (Lalor, Grover & Priede, SAE 800402, Fig. 7; Raff & Perry, JSV 28(3) 1973, Fig. 1) | ≈ 67 at 3000, ≈ 81 at 6000: ~20 dB low |
  | Exhaust orifice at 0.25 m, 45° | 104–110 dB Lin, 1000–6000 rpm (Davies & Holland, JSV 223(3) 1999, Fig. 1c) | today's inline-4 (catalyst only) ≈ 128 dB: 20 dB high; with the rule-sized road exhaust ≈ 95: 12 dB low |
  | Intake orifice | ≈ 88 dB(A) at 3000, 98 at 5000 (Raff Fig. 12b; distance not printed); the air cleaner takes ≈ 8.5 dB(A) (Raff Fig. 14) | a bare mouth: with a silenced exhaust it is 23 dB above everything at 6000 |

- At high speed petrol engine noise is mechanical: a 1500 cc engine motored without head or
  valvetrain made 88.5 dB(A) at 4000 rpm, the same as two fired ones (Raff Fig. 11). Many
  impacts per cycle make a near-stationary broadband — the "static" the user heard as an
  improvement.
- Ange's new simulator (exhaust only, no mechanical noise) measures 10–20 % modulation and a
  steep fall above 4–5 kHz: his exhaust physics (friction and heat transfer that he says
  changed the sound a lot, a turbulence model, ends that reflect the highs less) gives a steady
  treble ours does not.

## Goal

Every engine, with no per-engine knob, has each source at the level real engines of its kind
measure, and an exhaust whose damping matches the real one; then its treble texture and band
balance fall inside the real recordings' range. The user's ear gives the go/no-go.

## Non-goals, anti-goals

- No level set by ear, no per-preset constant. Every level comes from a measured curve with its
  source cited, or from physics with cited constants.
- Petrol idle stays as the user left it (parked): the mechanical sources follow their measured
  speed law and are far below the exhaust at idle; if an idle render changes audibly, that is a
  finding to report, not a goal.
- The LFA's approved sound (2026-10-31) must not get worse (its orders guard, ≤ 10.9 dB RMS).

## Part A — sources at their measured levels

A1. **Mechanical structure noise** (`crate::structure`, a mechanical excitation beside the
combustion one): main-bearing and piston-slap impacts timed by the sim's own load reversals
(gas + inertia forces on each main journal; piston side thrust changing sign), each impact a
force pulse through the existing structure response. Its level follows Lalor/Raff: the 1 m
overall dB(A) of the petrol engines' curves vs speed, two branches meeting at the critical
speed (≈ 13 dB/decade below, 50–60 above; Lalor eqs. 12–13, Raff N^5.2–5.5), scaled by
displacement as Lalor's bore series shows. Reuse the bearing prototype (`5aa0fa0`, Raff Fig. 7
low-speed level) and the piston-slap one (`e472fb2`) where they hold; replace their Gaussian
noise by impacts at the computed instants.

A2. **Intake air cleaner**: the intake mouth gets the preset's air box and filter as 1D elements
(volume + porous resistance), the box's insertion loss checked against Raff's 8.5 dB(A) and,
where available, a published air-cleaner transmission loss; presets without data get the
rule-sized box (as `road_exhaust` does for silencers).

A3. **Exhaust silencing as built**: every preset carries its real car's silencers (data where we
have it; `road_exhaust` sizing otherwise). Check first that `road_exhaust`'s insertion loss is
right: ours takes ≈ 34 dB off the orifice level where production systems take ≈ 18–20 (Raff
Fig. 15, Davies Fig. 1c vs an open pipe); find why before using it.

Acceptance A: at 1500/10 %, 3000/30 %, 6000/100 % on the inline-4 and the K20A, each source's
level within ±3 dB of its anchor (converted to the anchor's distance and weighting), measured by
`scripts/realism/levels.py`; the LFA guard holds.

## Part B — the exhaust's own damping (Ange's physics)

B1. Pipe wall friction and heat transfer on the live exhaust (the FV model has friction; heat
transfer exists as `gas_temperature`, off by default): measure each against a published case
(e.g. a pulse's decay along a heated pipe) before turning it on by default.
B2. Frequency-dependent reflection at the open ends and area changes (Ange: highs not reflected
efficiently; Levine–Schwinger end correction is in; check its |R(ka)| against the published
curve and add the junctions').
B3. Re-run the duct flow calibration (`feat/duct-flow-u4`, merged only now) with both metrics.

Acceptance B: treble modulation and band balance of every preset at the three points inside the
real p25–p75 (`scripts/realism/modulation.py --real`); the user's go/no-go on loudness-matched
clips of the inline-4, K20A and LFA; real-time cost measured, Ultra stays real time.

## Risks

- The structure-noise curves are read off figures (±2 dB) and come from a handful of engines.
- If A and B together still miss the texture, the next candidates are Ange's turbulence model
  and the papers we could not get (Kårekull 2015, Sakurai SAE 2004-01-0399): the user's option 2.
