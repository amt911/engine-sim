# Wankel rotary — design

Status: written unattended (user away, roadmap delegated 2026-10-31: "sigue el roadmap"). It is
item 5 of the user's queue (2026-10-22: "Rotary (13B/20B)"). Decisions marked *(my call)* are open
to the user's review.

## Why

A rotary sounds like nothing else: one combustion per rotor per shaft turn, no valves (ports the
rotor's flanks uncover), short fat exhaust ports straight off each chamber, a high redline. It is
the last engine family the user listed that the simulator cannot build.

## The key equivalence

A Wankel chamber's volume varies as a pure sinusoid of the eccentric shaft angle φ with period
540° (three shaft turns per rotor turn, four strokes per face): V = V_min + V_d/2·(1 − cos(2φ/3)),
V_d = 3√3·e·R·B (e eccentricity, R generating radius, B rotor width; Heywood, *ICE Fundamentals*
2nd ed., §1.8 and Fig. 1.23: RENESIS e 15, R 105, B 80 mm → 654 cm³ per chamber, which this formula
gives). With the **chamber angle ψ = ⅔φ** a chamber is a cylinder whose piston follows
r(1 − cos ψ) — a crank-slider with an infinitely long rod — on the usual 720° cycle. So:

- `CylinderGeometry` gains an optional `rotor` (e, R, B). With it, the chamber is an equivalent
  piston of the rotor flank's area √3·R·B and stroke 3e (area × stroke = V_d), and an infinite
  rod: position r(1 − cos ψ), lever arm r·sin ψ. Everything else (clearance volume from the
  compression ratio, Woschni, the flame's chamber height, the block's bore term) reads that
  equivalent piston *(my call: the rotary's long thin chamber has more wall than this; its higher
  heat loss is a known limit, Heywood §1.8)*.
- `Engine` turns the shaft; each chamber's cycle angle is ⅔ of it, and its gas torque on the shaft
  is ⅔ of p·dV/dψ. Chambers are "cylinders": a two-rotor engine has six, evenly fired 120° of ψ
  (180° of shaft) apart — exactly the 13B's rotors 180° apart with their faces 120° of rotor
  apart.
- Ports are the existing valves: open/duration in ψ, the lift curve standing for the area the
  flank uncovers. No valve train: no seat strikes, no valvetrain friction.
- Each rotor's three chambers share its exhaust port: in the exhaust net their primaries are
  short port stubs joined at once in a collector, which is the rotor's real primary pipe.

## Preset

**Mazda 13B-MSP RENESIS** (RX-8, 2003–2012): 2 × 654 cm³, e 15 mm, R 105 mm, B 80 mm, CR 10.0,
184 kW at 8200 rpm, 216 N·m at 5500 (Heywood Fig. 1.23); side intake and exhaust ports (no
overlap). Port timings, port areas, exhaust and intake lengths: ESTIMATES until a source is found.

## Non-goals

- Apex-seal leakage, the rotary's own friction model (piston friction by components is used with
  no valvetrain), peripheral ports and their overlap (a later preset: a bridge-ported 13B).
- 20B (three rotors) as its own preset: the model supports any number of rotors.

## Done when

- Geometry: V_d matches Heywood's 654 cm³; V(ψ) a pure sinusoid between V_min and V_min + V_d;
  serde round trip, old files load (no rotor).
- Engine: one combustion per rotor per shaft turn (firing frequency = shaft rpm/60 × rotors);
  every piston preset bit-identical (the rotor path is not taken).
- RENESIS dyno within 15 % of 184 kW / 216 N·m; idles; renders clips.
- Gate green, PR with manual test plan and verification; clips sent to the user.
