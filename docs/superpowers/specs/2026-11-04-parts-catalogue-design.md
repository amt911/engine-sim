# Catalogue of complete systems, sized to the engine — design

Status: described with the user on 2026-11-04, playback approved ("me parece bien así"). Roadmap
item 4 of "Now" in `docs/ROADMAP.md`.

## Intent

The user, 2026-11-04: "if you start touching parameters, they sound awful … I don't know if it's
the simulator itself or if it's well simulated and it really sounds bad".

### The case that started it

- **What the user did:** raised the single's cylinder count to 3 or 4; it "sounded like farts";
  changing the exhaust's size "didn't change anything".
- **What it really was** (reproduced and measured, 2026-11-04): the engine kept the single's
  open race exhaust. That exhaust has no catalyst or silencer, and a 40 mm collector 5 cm long
  takes all four cylinders.
- **The measurement** (3000 rpm, 50 % throttle):

  | Engine | Spectral centroid |
  | --- | --- |
  | Single raised to 4 cylinders | 591 Hz |
  | The same with the inline-4's exhaust | 172 Hz |
  | The inline-4 preset | 163 Hz |

  Swapping the exhaust also brought the level within 0.1 dB of the inline-4.
- **The exhaust sliders do work:** 10–25 dB per band. The user did not hear them because every
  edit took ~20 s to apply then (fixed in v0.46.0).

### What it means

The physics was right. The editor let the engine change while its parts stayed sized for
another engine, and nothing said so.

### What the user wants

The user: "I picture choosing a complete exhaust system, headers, turbos, parts like that,
depending on the engine's characteristics, so it doesn't sound like a fart as it does now."

## What it is

A catalogue of **complete systems**, each chosen with one click and **sized from the engine**:

| Family | Choices (first set) |
| --- | --- |
| Exhaust system (after the headers) | road (catalyst + two silencers), sport (catalyst + one straight-through silencer), race (open) |
| Headers | 4-1, 4-2-1 (tri-Y), log manifold; per bank on a V or flat engine |
| Intake | air box + plenum, individual throttles with trumpets |
| Turbo | small (quick spool), large (high flow); none |

Each choice fills the engine definition's existing fields: `exhaust` (primaries, collectors,
after-collector chain, merge), `intake`, `plenum_volume_m3`, `throttle_diameter_m` and `turbo`.

**Sizes are computed from the engine, never copied from a preset:**

- the inputs are the displacement per cylinder, the cylinder count, the layout and the rpm the
  engine is meant for (its redline);
- the rules are published design rules, each cited in the code. These include tuned primary
  length and diameter from the exhaust valve's opening and the target rpm, collector and tail
  bore from the gas flow at that rpm, silencer volume from the displacement, plenum volume and
  runner length from the displacement and rpm, and throttle and turbo sizing from the air flow.
  Sources to read before building: Blair, *Design and Simulation of Four-Stroke Engines* (SAE,
  1999), ch. 6–7; Smith, *Scientific Design of Exhaust and Intake Systems*; Heywood, *Internal
  Combustion Engine Fundamentals* ch. 7 (intake), ch. 6 (turbocharging); Munjal, *Acoustics of
  Ducts and Mufflers* (silencer volume vs. attenuation).

**Out of range:** the editor says when a measure is outside what real engines use for this one.
For example, "40 mm collector for 2.0 L × 4: real engines use 50–63 mm". The range comes from
the same rules or from a cited survey, never from taste.

## Decided with the user

- Complete systems by family, chosen per engine (not piece by piece first).
- Sized from the engine's characteristics.
- Hand edits stay possible after choosing.

## Calls made by the agent (the user may overturn any)

- **The chosen systems are remembered.** The definition gains an optional `systems` record (which
  family and choice per system). A file without it loads as today, with no system chosen and
  nothing re-sized.
- **Changing the engine re-sizes them automatically.** This covers the cylinder count, bore,
  stroke and redline. A measure the user changed by hand after choosing is kept, and the editor
  says that the rest was re-sized and which measures were kept.
- **The choices follow the engine type.**
  - A diesel has no throttle choice.
  - A single has one primary, not a 4-1.
  - V and flat engines take their headers per bank.
  - The ranges come from the same rules, so they hold for any cylinder count.
- **A kept hand edit is still checked.** If a re-size leaves it out of range, it is flagged like
  any other measure. A "re-size all" button releases every hand edit of that system. An edit is
  recorded per measure, inside its system's choice.
- **The tuned rpm is peak torque, not the redline** (Blair tunes the primaries for the torque
  peak): 0.75 × redline for road systems, 0.85 × redline for race ones.
- **A turbo choice carries a target boost.** Small: 0.8 bar gauge from half the redline. Large:
  1.5 bar gauge from 0.75 of it. The compressor and turbine are sized for the air flow there.
  "None" removes the turbo. The intake and exhaust choices stay independent; with a turbo, the
  exhaust system starts at the turbine's outlet, as now. Compression ratio and fuelling stay out
  of scope.
- **The rules stay fixed.** If the measured check (single at 4 cylinders + road exhaust against
  the inline-4) misses its tolerance, the rules and the test are not tuned to pass. The miss is
  measured and taken to the user, as the project's physics-first rule says.
- **One PR per family**, each usable on its own. The first ships the exhaust family together
  with the `systems` record and the re-size-on-edit mechanism.
- **No format version bump.** `systems` is optional and absent from files and presets that
  have not chosen one, so the presets' YAML stays as it is.
- **No commercial parts database:** no brands or part numbers.
- **The presets keep their own measured parts.** The LFA's exhaust, the K20A's and the others
  are fitted to real cars and are not replaced. Choosing a system on a preset replaces them, as
  any edit does.
- **The warnings are shown, not forced:** an out-of-range measure is allowed and flagged.

## Goals, non-goals, anti-goals

- **Goals:**
  - any engine the editor can build gets a breathing system that fits it in one click, and
    sounds like a real engine of that size;
  - the single raised to 4 cylinders plus "road exhaust" sounds like the inline-4 preset (the
    measured case above), checked by the same centroid and level comparison;
  - every rule cited.
- **Non-goals:**
  - an optimizer that searches for the best-sounding system;
  - cams, valve timing and compression (a later family);
  - the transmission (its own spec).
- **Anti-goals:**
  - sizes set by ear or copied from one preset and scaled;
  - a re-size that silently undoes the user's hand edits;
  - a warning that blocks an edit.

## Verification

- **Each rule:** unit tests with hand values from its source.
- **Each family:** applied to the single at 1, 3 and 4 cylinders, to a V8 and to a diesel, the
  result is valid (`config::validate`) and every measure is in range.
- **The single raised to 4 cylinders + road exhaust**, rendered at 3000 rpm and 50 % throttle,
  within 50 Hz of the inline-4's centroid and 1.5 dB of its level.
- **The user's ear:** before and after clips.
- **UI:** an `egui_kittest` test chooses a system, changes the cylinder count and reads the
  re-sized measures. A hand-edited measure stays.

## Left to the builder

The exact rules and constants, from the sources above. Where the systems go in the editor (a
section per family, with the choices as buttons). How a hand edit is recorded (per measure, in
the `systems` record). The order of the families: exhaust first, as the one the user met.
