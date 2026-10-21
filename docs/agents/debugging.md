# Debugging — keep the loop from running away

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

## Debugging — keep the loop from running away

What a bug costs is not the fix. It is the number of times you go around
`build → deploy → reach the state → observe` before you know what to fix, multiplied by what one
lap costs. Everything below attacks one of those two factors. **Each rule carries the number it
came from** — a real 15 h session — because a rule with no measured cost behind it gets deleted in
the first cleanup. Where this project hasn't measured its own, the number is marked
`<!-- pendiente de medir -->` until someone does.

### The loop is the cost

- **Measure before you ablate.** Ablation costs one lap per hypothesis and answers yes/no.
  Instrumentation costs one lap total and answers *what is actually happening*. **Measured: 28
  ablations over 1 h 42 min ruled things out and moved nothing; a single batch of probes, 13 min,
  changed the question and the bug fell on the next round.** The rule that batch produced: **if a
  pipeline completes every phase with non-empty output, the output exists** — stop asking "why
  doesn't it appear" and start asking "where does it appear". They are different questions and the
  second one is cheap.
  Here the pipeline to probe is: engine definition → physics step (crank, gas, combustion) →
  DSP chain → ring buffer → `cpal` callback; and, for the UI, sim snapshot → egui frame.
- **Budget the lap, then attack the dominant term.** Time the four phases once and write the real
  numbers into the table below; one of them dominates and the other three are noise. In the measured
  case "reach the state" was 60 s × 30 reproductions — half an hour of pure waiting — and it died to
  a shortcut nobody had bothered to write. **If a bug needs more than three reproductions, write the
  shortcut before the fourth**: a deep link, a dev-only route, an environment snapshot, a seeded
  fixture. Commit it as `scripts/repro-BUG.sh` and name it in the `docs/FINDINGS.md` entry, so
  the next person pays zero.

  | Lap phase | Command here | Measured |
  | --- | --- | --- |
  | build | `cargo build -p engine-app` | not measured yet |
  | deploy / install | — (runs from `target/`) | — |
  | reach the state | `cargo run -p engine-app -- --engine presets/single.yaml` | not measured yet |
  | observe | `cargo run -p engine-app -- --headless --seconds 3 --wav out.wav` | not measured yet |

### A finding is not a reproduction

- **Whoever reviewed read the code; they did not run it.** Reproduce a review finding yourself
  before sending anyone to fix it, and **if the implementer says they can't reproduce it, believe
  the implementer over the reviewer** — one of them has the thing running. **Measured: 1 h 25 min
  spent chasing a bug that did not exist.** This is the same reason the agentic PR pass is advisory
  and never vetoes on its own (see
  [Agentic PR verification](pr-verification.md#agentic-pr-verification-mandatory-on-every-pr)), and the reason
  `receiving-code-review` asks for verification rather than agreement.
- **A test that refuses to go red is data, not a failure.** The fourth failed attempt to pin down
  that non-existent bug is precisely what uncovered the real one, pointing the opposite way.
  Reporting "I cannot make this fail" is a result and it gets reported; covering it with a green
  test throws away the only signal the round produced.
- **Before demanding a red, ask whether the mechanism can produce one.** If another layer of the
  framework masks the effect, no amount of insisting will turn the test red, and the time goes into
  the test instead of into the bug. **Measured: over 1 h on two reds that were structurally
  impossible.** Establish that the failure is observable at that layer first; if it isn't, move the
  assertion to the layer where it is — that is what
  [Real-environment verification](real-environment-verification.md#real-environment-verification--what-no-in-process-test-can-prove)
  is for.

### Tests that cannot fail

The [mutation gate](quality-beyond-coverage.md#mutation-gate--the-60-floor-and-it-only-goes-up) already names the worst case —
an expectation recomputed with the same expression the code uses, which moves with the mutation and
agrees with it. It is not the only one. **Enumerate for this stack the assertions that are inert by
construction**, because none of them show up as a failure, a warning, or a coverage drop:

| Inert by | Looks like | Applies here |
| --- | --- | --- |
| assertions disabled at runtime | the assert keyword compiled out or off by default | `debug_assert!` vanishes in `--release` tests; use `assert!` in tests |
| a thread never joined | the assertion runs after the test already passed | a spawned sim/audio thread whose `JoinHandle` is dropped: its panic never fails the test |
| self-writing snapshots | first run records whatever happened and calls it expected | `egui_kittest` / `insta` snapshots with `UPDATE_SNAPSHOTS=1` / `--accept`: review the image or text before committing it |
| permissive mocks | the double returns the expected value by default, unasked | no mock crates here; a fake `AudioSink` that never checks what it got |
| expectation computed like the code | same expression on both sides of the equals | physics: expected values come from a hand calculation, a textbook/paper table or a closed form, never from the same formula in the test |

**Every assertion is watched failing once**, and expected values are written out by hand. This is
the same rule [Real-environment verification](real-environment-verification.md#real-environment-verification--what-no-in-process-test-can-prove)
states for on-device checks — it applies to in-process tests with no exception.

### The environment is a claim until it is measured

- **Verify the limit reaches the process doing the work** — see the check in
  [Heavy jobs run inside a memory cgroup](../../AGENTS.md#-heavy-jobs-run-inside-a-memory-cgroup-mandatory). A
  wrapper that reports success over an idle process is worse than no wrapper: it buys confidence
  and delivers nothing.
- **Environment claims get measured or they don't get made.** "That heap sounds low" produced a
  recommendation that was simply wrong. Measuring it — three runs per setting, GC pause totals, real
  peaks, not one run each — gave a **0.4% difference, below the run-to-run variance**. **No
  performance tuning lands without a before/after over more than one run**, and a difference smaller
  than the spread between runs is not a difference.

### Locate the rule before you pick a side

- **A rule that lives in one layer and isn't shared by the others fails in the wrong place.** The
  symptom surfaces where the assumption breaks, not where it is written, which is why the fix keeps
  landing in the innocent layer. Find which layer owns the rule first, then decide which side gives.
  The **contract test** row in
  [Real-environment verification](real-environment-verification.md#the-names-so-you-can-ask-for-them-by-name) is how you pin one
  down once you know it exists.
- **Replacing a component can remove capabilities in silence.** When you swap one API for another,
  enumerate what the old one did that the new one does not, and say it out loud in the PR — nothing
  will fail to compile. **An optional parameter that defaults to off is a capability that only
  exists if the caller remembers it**, which over a few months means it does not exist.
