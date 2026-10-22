# Agent orchestration

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

## Agent orchestration — parallel where it's free, batched where it's yours

Delegating work to agents moves the bottleneck from typing to **scheduling**: what waits on what,
what each agent has to rediscover, and which decisions quietly stop being yours. Same convention as
[Debugging](debugging.md#debugging--keep-the-loop-from-running-away) — every rule carries the number it came
from, out of the same measured 15 h session.

- **Review is not on the critical path.** Reviewing task N and starting task N+1 are independent
  whenever they touch different files. Serialized, review is **10-15% of the wall clock** and blocks
  everything queued behind it; run in parallel it costs nothing at all. **On receiving an
  implementation report, dispatch its review and the next implementation in the same turn.**
  This is the one sanctioned exception to *"at most 1 agent at a time"* in
  [normal mode](../../AGENTS.md#mode-switch): the cap is one **implementation** agent. A review agent reads and
  reports — it writes nothing, so it cannot race the implementer.
  Here: one implementation agent plus one review agent in flight. The exclusive resource is the
  audio device (and the `target/` dir under a heavy build): at most one agent runs the GUI or plays
  audio at a time; headless tests and WAV renders need neither.
- **Keep one shared facts file.** Every fresh agent rediscovers the same things: the real selector,
  which fake already exists, what that helper actually accepts. Keep `docs/FACTS.md` in the
  workspace, have each agent append to it when it finishes, and hand it to the next one in its
  dispatch. What belongs there: **facts verified against the repo or the device**, never opinions or
  plans. It is not `docs/FINDINGS.md` and does not replace it — FINDINGS holds the durable gotcha
  that is *not* deducible from the code, FACTS holds what is perfectly deducible and merely
  expensive to rediscover, and FACTS is allowed to go stale and die with the branch.
- **Plans carry contracts, not literal code.** The agent **trusts** the code you put in the plan; if
  you never compiled it, you have written an error wearing authority. **Measured: 4 wrong code
  blocks, 15-40 min of detour each.** Write exact names, exact signatures, and "mirror the shape of
  `<X>`" — claims the agent can verify against the repo — and reserve literal code for what you have
  actually run. This is what `writing-plans` produces; keep it that way when you edit the plan by
  hand.
- **Batch the discretionary decisions.** The work that shows up along the way — a capability being
  quietly dropped, a missing script, an adjacent bug — added up to **5-6 h of 15**. Every one was
  justified on its own; deciding them as they appear is what takes them away from you. Accumulate
  them and ask **once per batch, with the estimated cost of each**. In **"modo desatendido"** there
  is nobody to ask, so the batch goes into the PR body as a list with its costs — the decision is
  still yours, it just moves to review time.
- **What never gets cut.** With the numbers on the table: review was **1.5 h of 15**, and it found a
  `create()` silently discarding fields, a 404 caused by SQL deduplication, a silent merge that
  corrupted data, a delete-and-recreate with no transaction, and several inert assertions. **Cutting
  review does not give time back; it defers it to production.** If something has to be cut, cut
  reproduction (write the shortcut — see
  [Budget the lap](debugging.md#the-loop-is-the-cost)) and cut serialization (dispatch the review in parallel).
  Never verification.

### Day one — the numbers that fill the blanks

Three measurements, taken once at the start of a project, turn every `<…>` above into something
enforceable. None of them takes more than an afternoon.

1. **The lap** — time `build → deploy → reach the state → observe` once, on a real bug if there is
   one, and write the seconds into the table in
   [The loop is the cost](debugging.md#the-loop-is-the-cost). Whichever phase dominates is the one that gets a
   shortcut script; the rest are noise and stay unoptimized.
2. **The exclusive resource** — name the thing only one agent can hold at a time (emulator, dev
   database, dev-server port, a physical device) and write it into the orchestration note above.
   Everything else parallelizes; this is the one that corrupts a run when two agents touch it.
3. **The inert assertions** — break one assertion on purpose and run the suite. Anything still green
   is inert. Then walk the table in [Tests that cannot fail](debugging.md#tests-that-cannot-fail), delete the
   rows this stack cannot produce, and name the mechanism for the ones it can.

Record all three in this file, not in a session — the point is that the next session inherits them.

## Exclusive CPU between sessions — only for jobs whose result depends on time

Two agent sessions often share one machine all night. Most work tolerates that: a build or a test suite
under load just gets slower. Some jobs don't: their **result** depends on getting CPU time when they
need it, and under load they fail or measure the wrong thing:

- real-time runs with deadlines (an emulator route driven by a pad script, an E2E with wall-clock waits);
- benchmarks, perf A/B runs, frame-pacing or latency measurements.

Only those take the machine for themselves. **Compiling, unit/integration suites, lint and coverage do
not** — they share the CPU normally (`nice`, a capped `-j`, the memory cgroup) and never take the lock.

**Why this exists.** A decomp project's full differential gate (11 real-time game routes, ~15 min) failed
three times in one evening with `WAIT … timed out` while another session's `cargo` jobs pushed the load
average to 27–32 on 24 cores. Each failure looked like a regression and cost a rerun. Run alone, every
route passed. The fix was not more retries; it was agreeing on who gets the CPU when.

**The protocol** (both sessions must agree to it — tell the other one with `SendMessage`, found through
`ListAgents`, before you start using it):

1. **One lock, one owner file.** `flock` on `/tmp/claude-<uid>/cpu-heavy.lock`; the holder writes
   `<holder pid> <session name> <session pid> <start> <what>` to `/tmp/claude-<uid>/cpu-heavy.owner` and
   clears it when done. The session pid is the agent's own process (the nearest `claude` ancestor).
2. **Short holds that end by themselves.** Hold it only for the timing-dependent job, under **30 minutes**;
   split longer ones; run the job under `timeout 1800`, so a hung job cannot keep the lock.
3. **Nobody waits forever — not even alone.** A waiter checks the owner line every 30 s:
   - the holder is **orphaned** (its process or its agent session is gone — a session that stopped, crashed
     or was closed leaves its job behind) → stop the holder and go ahead **at once**;
   - **30 minutes** have passed → it may stop the holder and go ahead (the user's rule).
   A session left alone on the machine therefore never blocks on a lock nobody will release, and a missing
   or non-numeric pid field counts as alive (an older owner line is never mistaken for an orphan).
4. **Look before you blame the code.** When a timing-dependent job fails, read `uptime` and `top` first. If
   another session's load explains it, rerun under the lock instead of debugging a non-bug.
5. **Never rewrite the lock script while a job uses it**: a shell reads its script as it runs, and a job
   that started under the old file failed with a syntax error halfway. Keep it in the repo, versioned.

```sh
# scripts/cpu-lock.sh in this repo implements this; the reference shape:
# cpu_lock.sh WHAT COMMAND... — run COMMAND holding the shared CPU lock (timing-dependent jobs only)
D=/tmp/claude-$(id -u); LOCK=$D/cpu-heavy.lock; OWNER=$D/cpu-heavy.owner
what=$1; shift
session_pid() {  # the agent session (the nearest `claude` ancestor), or the parent shell if there is none
  p=$$; while [ "$p" -gt 1 ]; do [ "$(cat /proc/$p/comm 2>/dev/null)" = claude ] && { echo $p; return; }
    p=$(awk '{print $4}' /proc/$p/stat 2>/dev/null || echo 1); done; echo $PPID
}
alive() { case "$1" in ""|*[!0-9]*) return 0;; esac; kill -0 "$1" 2>/dev/null; }
exec 9>"$LOCK"
waited=0
until flock -n 9; do
  read -r hpid _ spid _ < "$OWNER" 2>/dev/null
  if { [ -n "$hpid" ] && { ! alive "$hpid" || ! alive "$spid"; }; } || [ $waited -ge 1800 ]; then
    echo "[cpu-lock] holder orphaned or 30 min passed; stopping: $(cat "$OWNER" 2>/dev/null)" >&2
    alive "$hpid" && { kill -TERM "$hpid"; sleep 15; kill -KILL "$hpid" 2>/dev/null; }
    flock -w 120 9 && break
    echo "[cpu-lock] still held; giving up" >&2; exit 75
  fi
  sleep 30; waited=$((waited + 30))
done
echo "$$ <session> $(session_pid) $(date +%FT%T) $what" > "$OWNER"
timeout 1800 "$@"; rc=$?
: > "$OWNER"; exit $rc
```

In this repo: `scripts/cpu-lock.sh <command>` for timing jobs (benchmarks such as `fv_bench`,
real-time-factor runs); builds, tests, lint and mutation runs use `scripts/heavy.sh` (cgroup + nice
19 + `-j8`) and never take the lock.
