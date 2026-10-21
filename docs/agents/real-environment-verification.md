# Real-environment verification

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

## Real-environment verification — what no in-process test can prove

Some properties are invisible to the entire in-process suite no matter how many tests you add,
because the test runtime never restarts a process, never talks to a real server, never runs out of
disk, and never lets the scheduler cancel anything. **jsdom is not a browser, a mocked ORM is not a
database, a fake clock is not time, and an in-memory DB is not the one the user has on disk.**
Those properties need a script that drives the **real artifact on real hardware** — a real browser,
an emulator, a VM, a container, the target machine — and asserts on what is externally observable:
log lines, exit codes, HTTP responses, rows in the database, files on disk.

**Write that script, commit it, and name it here.** It must run by hand with no arguments, print a
per-phase `PASS`/`FAIL`, and exit non-zero on the first failure:
`scripts/smoke.sh` (builds the release binary, renders a few seconds of audio headless to a WAV
and asserts it is non-silent, finite and at the expected RPM; then launches the GUI for a few seconds
and asserts it did not crash).

**The run happens on real hardware or an emulator — never on a stand-in for the thing under test,**
and never on the user's daily device/workstation when the check writes state. Boot the emulator /
disposable VM / throwaway container; that is the target.

### The names, so you can ask for them by name

| Name | What it means |
| --- | --- |
| **E2E / on-device acceptance test** | Drives the real build against the real backend and asserts on observable behaviour — log lines, HTTP status, rows in the DB, files written — never on internals. The phases of the script above. |
| **Contract test** | Checks that the **client's assumptions about the server's responses** actually hold. These are exactly the assumptions no type system on the client side can see: a filter that is a strict `>` and not `>=`, a timestamp column stored with microseconds, a field the docs call optional and the server always sends. |
| **Mutation testing** (on real hardware: by hand) | Revert the fix, re-run the check, confirm it goes red, restore. Stryker / PITest / mutmut automate this for in-process code; against a device or a machine you do it manually. **A check that has never failed has not been tested.** |
| **State-invariant test** | Asserts a relationship **between two stores** that no single unit test owns — e.g. a delta watermark must never outlive the database it describes. Each store is individually correct; the pair is what breaks. |
| **Test pollution / isolation leak** | A test writing to *production* state — the installed app's storage, the dev database, the user's config directory, the real keystore. It passes, and quietly destroys data on the next run. |

### Rules that came out of real bugs, not theory

- **Prove every new check can fail before you trust it green.** Revert the fix, watch the check go
  red, restore it. This applies to unit tests written after the fact *and* to real-environment
  checks. A green you have never seen turn red is not evidence.
- **Never assert on a count you cannot predict.** A check that fails "above five rows" reports PASS
  against a deliberately broken build whenever the data happens to cluster differently — how many
  rows a bad cursor drags back depends on the dataset, not on the bug. Assert the **invariant**
  (the watermark carries milliseconds; the response is empty; the ids match), never a symptom whose
  magnitude varies with the data.
- **A watermark, cache marker or cursor must die with the data it describes.** Clearing one without
  the other is silent, permanent data loss — no crash, no log, no failing test.
- **Anything that touches machine-global state must restore it.** Device storage, the user's config
  dir, the real database, the system keystore, installed packages: save it, and restore it in the
  teardown that runs even when the test fails.
- **Run the real-environment suite the way that actually works on this machine**, not the way the
  docs say. When the canonical task hangs, deadlocks or needs a display this box doesn't have, write
  the command that works into `docs/FINDINGS.md` and use it:

  ```bash
  scripts/smoke.sh
  ```
