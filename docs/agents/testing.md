# Tests — stack, per-crate table, conventions

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` stays small. Its rules bind all the same:
> `AGENTS.md` lists the hard ones and says when to read this file. Edit the rule here, not a copy.

## Tests — stack, per-crate table, conventions

- **Unit tests:** `#[cfg(test)] mod tests` colocated with the code; integration tests in
  `crates/<crate>/tests/`.
- **Floats:** `approx` (`assert_relative_eq!`, `assert_abs_diff_eq!`) with an explicit tolerance and
  a comment saying where the tolerance comes from.
- **Invariants:** `proptest` — e.g. piston position stays within `[0, stroke]`, mass is conserved
  between volumes, `from_yaml(to_yaml(e)) == e`, the sim never produces NaN for any valid engine.
- **UI:** `egui_kittest` drives widgets headless (find by label, click, drag, assert the model).
- **Real binary:** `scripts/smoke.sh` (see real-environment verification).
- **Coverage:** `cargo llvm-cov`. **Excluded with reason:** `engine-app/src/main.rs` and the
  egui painting code (pixels are checked by kittest snapshots and the smoke, not by line coverage),
  `engine-audio/src/device.rs` (needs a real sound card; covered by the smoke),
  `engine-app/src/view3d/render.rs` (wgpu pipelines and passes: need a GPU device; checked by
  the `render_3d_screens` off-screen renders, the `bench_3d_frame` bench and the smoke).
  **Run under nextest's `coverage` profile** (`.config/nextest.toml`): it skips the 28
  whole-engine simulations that take ≥ 35 s in a normal run, ~15× more instrumented. With them
  the gate took 3–4 h; without them 10 min, 92.6 % lines against 94.4 % (2026-11-09) — the drop
  sits in `engine-app` (`dyno.rs`, `sim.rs`, `offline.rs`); every `engine-core` file stays
  ≥ 90 %. Those tests still run in every `cargo test --workspace`.
- **Mutation:** `cargo mutants -p engine-core` inside the cgroup (`scripts/heavy.sh`).

### Run before declaring done

| Change touches | Run before claiming success |
| --- | --- |
| `engine-core` | `cargo test -p engine-core` |
| `engine-audio` | `cargo test -p engine-audio` |
| `engine-app` UI / CLI | `cargo test -p engine-app` + `scripts/smoke.sh` |
| anything large | `cargo test --workspace && cargo clippy --workspace --all-targets -- -D warnings && scripts/smoke.sh` |

### What to test per module

| Module | What |
| --- | --- |
| `core::geometry` | crank-slider position/velocity vs closed form; TDC/BDC; Wankel chamber volume vs Yamamoto formula |
| `core::gas` | ideal gas state, orifice flow (choked/unchoked) vs textbook values, mass/energy conservation |
| `core::combustion` | Wiebe burn fraction endpoints and 50% point; misfire releases nothing |
| `core::engine` | firing order spacing, cycle work sign, stable idle without NaN |
| `core::config` | JSON/YAML round trip, defaults for missing fields, validation errors |
| `audio::dsp` | DC blocker kills DC, passes 50 Hz; filters' -3 dB point; no NaN on silence |
| `app` | editor model changes the engine definition; CLI parsing; headless render length |

### Operative conventions

- Expected numbers are computed **by hand or taken from a reference** and the source is cited next
  to them.
- A test that spawns a thread joins it (otherwise its panic is lost).
- Split a test file past ~300 LoC by aspect.
