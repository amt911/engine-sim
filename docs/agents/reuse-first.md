# Reuse first — search before you write

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

## Reuse first — search before you write

The default failure mode of an agent (and of a tired human) is to write the thing that already
exists: a second `formatPrice`, a fourth bespoke modal, a `Button` that is 90% the one in the design
system with one colour hardcoded. Nothing breaks — that is what makes it expensive. The copy drifts,
the fix lands in one of them, and the design system stops describing the product.

- **Search before writing. Every time.** Before creating a component, hook, helper, type, DTO,
  fixture or script, look for it by name *and* by behaviour (`grep -ri "format.*price"`, read
  `crates/engine-core`, `crates/engine-audio`, the UI widgets module of `crates/engine-app`). "I didn't know it existed"
  is a search you didn't run, not an excuse.
- **Extend or parameterize — don't clone.** If something is 80% right, add the prop/parameter/variant
  to it. A copy with three lines changed is two things to maintain and one of them will be forgotten.
- **Rule of three.** Two occurrences can wait. At the third, extract in the same change, not "later":
  the component into the shared UI layer, the logic into `packages/shared`.
- **Reuse across the boundary, not through it.** `web` must not import from `api` (or the reverse) to
  reuse a function. If both sides need it, it moves to `packages/shared`; if it can't move, it wasn't
  shareable.
- **Don't reuse coincidences.** Two things that look alike today but answer to different owners (an
  invoice line and a cart line) are not one thing — coupling them under one abstraction costs more
  than the duplicate. Reuse what shares a *reason to change*, not a shape.
- **Extraction includes the deletion.** Migrate the call sites and remove the old copies in the same
  PR. An abstraction that lands *next to* the copies it was meant to replace made things worse.
- **A deliberate duplicate is one sentence in the PR.** Say why the shared version didn't fit. The
  rule is not "never duplicate", it is "never duplicate by accident".

Dependencies count as existing code: before hand-rolling a date parser, a slug helper or a retry
loop, check whether something already in `package.json` does it — and never add a dependency just to
avoid writing ten lines. When one is genuinely needed, ask first (see *Working rules*).
