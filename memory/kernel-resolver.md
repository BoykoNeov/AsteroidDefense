---
name: kernel-resolver
description: Rust kernel resolver + ASTEROID_REQUIRE_KERNELS — how to run the suite so green means it ran
metadata: 
  node_type: memory
  type: project
  originSessionId: befb7b97-c9f8-4498-b2fd-b08897147d5e
  modified: 2026-07-19T20:15:15.186Z
---

**Run the Rust suite as `ASTEROID_REQUIRE_KERNELS=1 cargo test --workspace --release`.**
Done 2026-07-19 (commit 525e33a), closing the kernel-skip trap noted in
[[gdext-binding]].

`core/src/kernels.rs` is the Rust mirror of `godot/scripts/kernels.gd`:
`resolve()` takes env → conventional dirs, **both-or-nothing**; every gated site
in the workspace (core, `validation`, the gdext binding, the examples) goes
through `resolve_for_test(what)`. Real kernel dir on this machine is
`W:\temp\claude\AsteroidDefense\kernels` (de440s.bsp + pck11.pca) — note
the `AsteroidDefense` segment; earlier notes said `temp/kernels`, which is wrong.

**Why the flag is a separate thing from the resolver, and why both were needed:**
resolution cures only *"I have the kernels but didn't point at them"* — this box,
today. A fresh clone, a CI container, or a renamed directory puts the
silent-green failure straight back. `ASTEROID_REQUIRE_KERNELS` turns "nothing
resolved" into a panic naming the test that would have lied. Unset, the skip is
still green, so offline CI keeps working — that is deliberate, don't "fix" it.

**2026-09-08, the move to W: — the resolver's anchor is a *relative walk*, and
it is the thing that breaks next time.** Both mirrors find the scratch kernels
by walking up from the repo, so they encode the surrounding folder layout, not
just a path. Old layout had `temp/` *inside* `claud_projects/` beside the repo,
so one `..` reached it. The new layout does not, so the walk is now two levels:
`core/src/kernels.rs` uses `repo.join("../../temp/claude/AsteroidDefense/kernels")`
and `godot/scripts/kernels.gd` uses `res://../../../temp/claude/...` (one extra
`..` because `res://` is `<repo>/godot`, not the repo). The old sibling walk is
kept as a fallback, so an unmoved checkout still resolves. **Move the repo or
the scratch root again and both numbers are wrong** — and the failure is the
silent-green one this whole memory exists to prevent, unless REQUIRE=1 is set.

`user://kernels.cfg` is deliberately NOT read from Rust (`user://` is Godot's
per-platform app-data path; reconstructing it would be a guess that rots).

**The gate was proved by bypassing it:** rename the kernel dir away, and
REQUIRE=1 makes the suite FAIL; unset, it reproduces the original lie exactly —
*same* `81 passed` / `13 passed`, at 0.09 s / 0.00 s instead of 18.03 s /
56.38 s. Counts are indistinguishable; **the clock was always the only signal.**
That bypass is also the only way to tell a fast-but-real test from a skip — it
confirmed `tier1_field_matches_assist` genuinely runs in 0.05 s.

`ScenarioError::MissingKernelEnv` → `KernelsNotFound(String)` carrying the
searched-paths repair message.
