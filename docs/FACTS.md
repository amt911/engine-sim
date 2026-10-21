# FACTS — engine-sim

> This is the repo's **shared working memory between agents**. `AGENTS.md`
> § *Agent orchestration* makes it the file every agent reads in its dispatch and appends to when it
> finishes, so the next one doesn't re-derive what this one already checked.

## What goes here

A fact earns its place if **all** of these hold:

- It was **verified against the repo or the running system** — read in the code, seen in the view
  hierarchy, printed by the command. Not remembered, not inferred, not planned.
- A fresh agent starting cold would otherwise spend real time rediscovering it.
- It is a *fact*, not a decision: what the selector is, which fake exists, what that helper accepts.
  Decisions go in the spec or in `AGENTS.md`.

Not here: **gotchas that aren't deducible from the code** — those are `docs/FINDINGS.md`, and they
outlive the branch. What lives here *is* deducible from the code and merely expensive to look up
again. **This file is allowed to go stale and die with the branch**; nothing should depend on it
being complete, and nothing gets written *only* here.

## Format

Grouped by area, one line per fact, newest first inside its group. If a line needs a paragraph, it
is a finding, not a fact — move it.

```markdown
## <Area: selectors | fixtures & fakes | commands | contracts | env>

- `<name or symbol>` — <the fact, in one line>. *(verified: <how>, <YYYY-MM-DD>)*
```

`verified:` says **how you know** — `read <file>:<line>`, `ran <command>`, `maestro hierarchy`,
`curl`ed the endpoint. A line without it is a rumor.

---

## Commands that work here

_None yet._

## Drivetrain and dyno (feat/drivetrain)

- `engine_core::drivetrain::{VehicleDef, Vehicle, Load}` — `Vehicle::step(dt, omega, brake)` returns torque + reflected inertia; `Simulation` feeds them into `Controls::load_torque_nm` / `extra_inertia_kgm2` every physics substep. Vehicle presets are keyed by `Preset::vehicle()`, not stored in `EngineDef`, so `presets/*.yaml` are unchanged. *(verified: read crates/engine-app/src/{sim,controls}.rs)*
- Clutch torque is `engagement² · capacity` (linear stalled every launch in a 1300 kg car); engagement ramps 1.5 /s up, 10 /s down. *(verified: ran the launch test, engine stalled with the linear map)*
- A gear selected with the clutch up at idle stalls the engine (realistic): tests press the clutch in neutral, select the gear, then release. *(verified: sim.rs `driven()` helper)*
- Dyno (`--dyno`, release build, ~seconds): single peaks 49 N·m @2000 / 27 hp @5000; inline4 198 N·m @2500 / 112 hp @5000; LFA 516 N·m @4000 / 521 hp @8500 (published 480 N·m, 552 hp; real peak torque is at ~7000 rpm, the 0D model peaks too early). Sweep runs 1000 rpm .. below redline in 500 rpm steps. *(verified: ran `target/release/engine-sim --preset <p> --dyno`, 2026-10-21)*
- LFA top speed from ratios: 9000 rpm in 6th = 323.6 km/h vs 325 published. *(verified: hand calc + test)*
- Vehicle numbers marked ESTIMATE in `VehicleDef` doc comments (Cd·A, Crr, efficiency, clutch capacity, motorcycle and hatchback figures) were not checked against a source; LFA ratios/mass/final drive come from the task spec, tyre radius is computed from 305/30 ZR20.
