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
