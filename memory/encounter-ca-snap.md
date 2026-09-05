---
name: encounter-ca-snap
description: "[C] snaps the encounter view to closest approach; argmin on the core's track, and why a sampled minimum ≠ perigee"
metadata: 
  node_type: memory
  type: project
  originSessionId: befb7b97-c9f8-4498-b2fd-b08897147d5e
  modified: 2026-07-19T20:15:34.915Z
---

**[C] in the b-plane view pauses the clock and jumps to the live track's closest
approach** (commit 98a2136, 2026-07-19). Replaces the commit→pause→scrub ritual
noted in [[gdext-binding]]: the marker only draws inside ±1.5 d of a twelve-year
campaign, and any warp step overshoots CA by ~0.53 d.

`Sim.encounter_ca_day()` = argmin over the core's encounter polyline — the same
category as the lerp `encounter.gd` already does along it, **not** a
re-derivation (GDScript still owns zero orbital mechanics). The core's bisected
CA epoch exists but is **discarded** at `deflection.rs:373,435` when reducing to
`b_plane`; plumbing it through four signatures buys sub-185 s accuracy nothing on
screen can show. Revisit only if a keyhole countdown needs the exact epoch.

`Sim.deflected_is_live(defl_empty)` is now the single rule for which track is
real — the snap and the marker MUST agree, because the deflected CA is shifted
~0.53 d off the nominal and a snap aimed at the wrong track lands off-plot and
does nothing visible.

**The lesson worth keeping: a sampled minimum is not the perigee.** The
verification check (min track range vs the core's reported perigee — one
comparison settling both frame origin and argmin correctness) FAILED first at
3259 vs 3000 km on a guessed 1% band. That was the **sampling floor**, not a
defect: at ~18 km/s the rock moves ~1700 km through the turn between 185 s
samples, so the bound is `sqrt(r_p² + (v_p·dt/2)²)` with `v_p` from vis-viva —
3432 km, and 3259 sits inside it. **Derive the tolerance from the geometry;
a guessed band fails on real physics and reads like a bug.**

Verified visually via the `_shot.gd` autoload (register in project.godot, run
non-headless, then REMOVE the autoload) — `_draw()` runs headless only for
VISIBLE nodes. It now drives `_jump_to_closest_approach()` so it tests the
shipped path.
