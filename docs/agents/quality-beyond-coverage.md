# Quality beyond coverage

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

## Quality beyond coverage

**Coverage measures how much code runs, not whether it's correct.** This is especially treacherous
with AI: it tends to write the test *and* the code in one move, so if it misread the requirement, both
encode the same mistake and the test passes happily. 80% coverage with weak asserts is a false sense
of security. These gates attack that blind spot.

- **Mutation testing** *(highest priority — and the one gate with a hard number, see below)* —
  **Stryker** (JS/TS), **PITest** (Kotlin/JVM), **mutmut** / **cosmic-ray** (Python),
  **cargo-mutants** (Rust) inject deliberate bugs (`>` → `>=`, drop a line, flip a boolean) and check
  some test fails. A surviving mutant means the code is *covered but not verified*. **Concrete recipe
  that works:** scope `mutate` to a **pure compute function extracted out of the service**
  (mocked-ORM tests can't kill query-shape mutants), pick the runner per package (jest-runner vs
  vitest-runner), and set `thresholds: { high: 90, low: 80, break: 60 }` — `break` is the gate and 60
  is the floor. This is the direct antidote to AI's misleading coverage.
- **Property-based testing** *(highest priority)* — **fast-check** (JS/TS), **Hypothesis** (Python).
  Define invariants ("deserialize(serialize(x)) == x", "final price is never negative") and let the
  framework generate hundreds of cases, including the weird boundaries nobody thinks of. Catches logic
  errors that hand-picked examples miss.
- **Runtime boundary validation** — **Zod** (TS), **Pydantic** (Python) to validate everything
  crossing a boundary: API responses, forms, DB data. AI trusts types that don't hold at runtime; this
  turns those assumptions into explicit errors instead of silent failures.
- **Strict types + static analysis** — TypeScript in real `strict` mode (**including
  `noUncheckedIndexedAccess`**), type-aware ESLint, and a SAST (**Semgrep** or **CodeQL**). SAST
  matters because AI introduces vulnerabilities easily (injection, hardcoded secrets) that no
  functional test catches.
- **E2E / smoke tests** *(mandatory, not a nice-to-have)* — **Playwright** (web), **Maestro**
  (native Android/iOS — YAML flows plus `maestro hierarchy`/`maestro mcp` for discovery). Verify what
  unit tests can't: that the app *actually boots* and the full flow
  works. Code routinely passes every unit test while the app won't start or the frontend assumes an
  API contract the backend doesn't honor. This is the single highest-yield gate against
  "implemented but broken on first click" — see the hard rules in
  [E2E (Playwright) — mandatory](e2e.md#e2e-playwright--mandatory).
- **Dependency auditing** — AI invents non-existent packages ("slopsquatting") and pulls vulnerable
  versions. Use `npm ci` with a frozen lockfile, `npm audit` / Dependabot / Snyk in CI, and verify
  every new dependency actually exists and is the one you think it is.
- **Dead-code elimination** — **Knip** (JS/TS) finds unused files, exports, types and dependencies
  across the workspace (monorepo-aware; auto-detects Next/Vite and `pnpm` workspaces). Drop a
  `knip.json` at the repo root (zero-config to start: `{ "$schema": "https://unpkg.com/knip@5/schema.json" }`)
  and run `pnpm dlx knip` — or add a `"knip"` script once you want it in the loop. Pruning dead code
  shrinks the surface every session (and the AI) has to reason about and keeps `package.json` honest,
  complementing the dependency audit above. AI-written code accretes orphaned helpers and unused
  exports fast, so run it periodically on web projects.

**Process rule (worth more than any tool): don't let the AI define the acceptance criteria.** You
write or review the important test cases yourself — at least the key asserts and the requirement's
edge cases — and have the AI implement against them. That breaks the loop where the same
misunderstanding lives in both the test and the code. Mutation testing is the automated backstop for
this, but the judgment about *what the system should do* stays yours.

Priority by immediate payoff: **mutation + property-based testing first** (they hit the current blind
spot), then **runtime validation and a couple of E2E smoke tests**.

### Mutation gate — the 60% floor, and it only goes up

Coverage answers *"did any test run this line?"*. Mutation answers *"would any test have noticed if
the line were wrong?"*. A test with no assert scores 100% coverage, which is why this gate is the one
with a hard number attached.

- **Floor: 60%**, measured over the **core-logic scope** — `crates/engine-core` (geometry, thermo, combustion,
  gas flow, dynamics, config) and the pure DSP of `crates/engine-audio` — not the whole tree. Repositories, DAOs, framework glue and view code dilute the score
  into noise: a mutant inside a mocked query is not a bug anyone can write a test against. Scope
  narrow, gate hard; scope wide, gate meaningless.
- **The threshold is a ratchet.** Set it to today's real score rounded down, never under 60, and
  raise it in the same PR that raises the score. **Lowering it to make a push go through is exactly
  what the gate exists to prevent** — a score that dropped means a test stopped verifying something.
- **Not at 60 yet?** Ship the gate **advisory** (it reports, it never fails) with the current score
  and the date written next to it, and owe a PR that reaches 60 before the next feature. Advisory is
  a waypoint, not a resting place.
- **Doesn't apply to this repo?** Write that here, with the reason (no executable code; packaging-only;
  byte-matching decompilation; generated sources). An unwritten exemption gets re-litigated every few
  months; a written one does not.
- **It does not mean chasing 100%.** Equivalent mutants exist (inlined stdlib, generated glue,
  coroutine/async branches) and are annotated and left alone, not tested into submission.

Read the report before quoting a number: **SURVIVED and NO_COVERAGE mean opposite things** and the
headline percentage mixes them. A survivor is code that runs while nothing asserts on the result — a
real hole. NO_COVERAGE is code the mutation runner never reached, which is often a runner limitation
(Robolectric under PITest, for one) rather than a missing test. Split them before quoting.

The fix for a survivor is almost always the same: **assert the concrete expected value, written out
by hand**. A test that recomputes the expectation with the same expression the code uses moves with
the mutation and agrees with it — 100% line and branch coverage, zero verification.

| Stack | Tool | Gate command |
| --- | --- | --- |
| JS/TS | **Stryker** | `pnpm test:mutation` (`stryker run`, `thresholds.break: 60`) |
| Kotlin / JVM | **PITest** | `./gradlew pitestDebug -Ppitest.threshold=60` |
| Python | **mutmut** (or **cosmic-ray**) | `mutmut run` + a score check on `mutmut results` |
| Rust | **cargo-mutants** | `cargo mutants --in-place --error-percent 40` |

**Make each mutant cheap.** A mutant costs its build plus the tests that run until one fails. Here
cargo-mutants runs them under **nextest** (`.cargo/mutants.toml`), which stops at the first failure:
a caught mutant costs the test that catches it, not the suite (measured 2026-10-23: 108 s against
200 s per mutant). Only a missed mutant pays for the whole suite, so the suite's **critical path** —
its longest single test — bounds it: split a test that loops over presets into one test per preset
(same asserts, run side by side) rather than letting one test run them all in sequence.

**A mutation run is a heavy job** — it forks one JVM/worker per core and sizes nothing for you. Run it
inside the memory cgroup and cap the worker count: see
[Heavy jobs run inside a memory cgroup](../../AGENTS.md#-heavy-jobs-run-inside-a-memory-cgroup-mandatory).
