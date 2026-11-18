# Typical parts and typical sizes: a catalogue of real components

Status: **DRAFT for the user's approval** (written unattended, night of 2026-11-18/19). Nothing is
implemented until it is approved.

## Why

ROADMAP item 4 (user, 2026-11-04): "if you start touching parameters, they sound awful". Two
possible causes, and the work must tell them apart:

1. **The model is wrong** for sizes away from the presets it was checked on.
2. **The engine is badly sized**: a combination no real engine uses (a 30 mm primary on a 2 L
   engine at 9000 rpm, a silencer of a tenth of the swept volume), which really does sound bad.

## Goals

- **A catalogue of real parts with their real dimensions**, each with its source, that the editor
  offers in place of free numbers:
  - exhaust manifolds: road 4-1, 4-2-1, race equal-length (primary length and bore, collector);
  - silencers: road packed straight-through, reactive multi-chamber, race straight-through
    (shell, tube, length, packing), cut to the engine's swept volume as `road_exhaust` already
    does;
  - catalysts: close-coupled and underfloor (cells per in², brick size);
  - intake runners and plenums: individual throttle bodies, single throttle with plenum
    (runner length and bore, plenum volume);
  - air cleaners: rule-sized box, cone filter;
  - cams: road, sport, race (duration, lift, timing), from published cam cards.
- **"Out of range" warnings in the editor**: each sized quantity is checked against the range
  real engines use. The ranges come from the catalogue's sources and the standard rules already
  in the code (Mach index for valves, Blair's primary-length rule, ten swept volumes of silencer).
  The warning names the quantity, the real range and the source, and offers the nearest
  catalogue part.
- **A measurement that tells the two causes apart**: a set of edited engines (one per
  catalogue family, sized from it) rendered and measured with `scripts/realism/same_engine.py`
  against stock recordings of engines of the same size. If an engine built from real parts still
  sounds wrong, the model is wrong: that goes back to the physics. If only the out-of-range
  engines sound wrong, the warnings are the fix.

## Non-goals

- No new physics in this work: the catalogue only feeds existing fields.
- No online catalogue or download: the parts live in the repository as data (YAML), each with its
  source.

## Pieces (one PR each, from main)

1. **The catalogue as data** (`presets/parts/*.yaml` or a module in `engine-core`), each part
   with its source and its fields; round-trip tests; a test that every catalogue part loads and
   runs on a preset.
2. **The range checks** (`engine-core`, pure functions with hand-calculated tests): each
   quantity's real range and the warning.
3. **The editor**: a "Use a real part" picker per section and the warnings beside the fields
   (egui_kittest tests: pick a part, the fields change; an out-of-range value shows its warning).
4. **The measurement**: the edited engines rendered and compared with stock recordings
   (same_engine.py), results in `docs/FINDINGS.md`.

## Questions for the user

1. Which families matter most first (the list above is a proposal)?
2. Should an out-of-range value be only a warning, or should the editor refuse it?
3. Any parts you know well (a car you own, published cam cards) to start the catalogue with?
