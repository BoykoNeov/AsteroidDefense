---
name: threat-orbit
description: "The [N] threat-orbit designer (2026-07-28) — free closed-form preview, the two walls, the gated rebuild, and the per-orbit required-Δv anchor that must never fall back on the shipping constant"
metadata: 
  node_type: memory
  type: project
  originSessionId: 2b76bc3b-5030-4061-b147-ee6d1069edd4
  modified: 2026-07-28T02:16:25.184Z
---

**The threat's heliocentric orbit is now a knob (`[N]`, 2026-07-28).** Answers the
user's ask: *"test the tractor at different orbits — some effective, some not."*
Builds on [[deflection-spectrum]]'s `[K]` bench; see [[gdext-binding]] for the
staleness traps.

## Scope — the two frozen fields are the design decision

`ImpactorConfig` has five interesting fields; **three ship as knobs** (v_rel,
approach az/el, b_offset). `impact_epoch` and `lead_years` are **frozen** because
together they *are* the mission clock — `epoch0` is GDScript's `EPOCH0_TDB`, the
origin of every drawn `t`, and `impact_epoch` is `T_IMPACT` (event schedule + both
porkchop axes). Moving them slides the campaign along the timeline instead of
changing the orbit. That freeze is what keeps a rebuild bounded.

## The preview is free because of a geometric accident

The offset is laid **perpendicular to v_rel** ⇒ the impact point **is the perigee
of the geocentric hyperbola** ⇒ `BPlaneEncounter::from_relative_state` gives
`v_inf` + incoming asymptote in closed form, and incoming helio velocity is just
`v_earth + v_inf·Ŝ`. `ImpactorConfig::preview` = microseconds vs the builder's 10 s.
`impact_offset_axis()` extracted so preview and `build_with` cannot fork.

**Measured, and the halves differ completely:** encounter geometry **exact**
(v_inf, b match the propagated nominal's own reduction to **0.001 %**); orbit is an
**estimate** (osculating at impact epoch vs vis-viva at the seed 12 yr earlier) —
**0.23 % worst** over 0.68–2.66 yr. Good enough to *label* a knob, not to *score* a
plan → the bench keeps taking `period_seconds()` from the built scenario.

## THE TWO WALLS (both reachable, closing from opposite directions)

- **Too slow:** flyby needs `v_rel > √(2μ⊕/b_offset)` — 16.3 km/s at the shipping
  3000 km vs a shipping 18. **Shrinking the offset RAISES the bar** (28.2 km/s at
  1000 km; 63.1 at 200), so pulling the hit toward Earth's centre is what falls off
  the cliff. New `ScenarioError::ImpactNotHyperbolic`.
- **Too wide:** predicted ~4790 km by holding `v_inf` fixed while
  `b = b_offset·v_rel/v_inf` grew. **WRONG — measuring said 6400.** A wider offset
  also raises `v_inf` (less well to climb), growing `b` slower *and* shrinking
  `b_capture`. They meet where they must: `b ≤ b_cap ⟺ r_perigee ≤ R⊕`, and the
  perigee **is** `b_offset`. **So the ceiling is exactly Earth's radius** and
  `b_offset` is a perigee-altitude dial wearing a b-plane name. At the clamp the
  harness measures **b/b_cap = 1 to 9 digits**.
  ⇒ the knob can never produce a miss; the `is_hit` refusal is **defensive/
  unreachable through the UI** (same relation as the bench's `holds_station`).

## THE SILENT FAILURE THIS LAYER EXISTS TO PREVENT

`REQUIRED_DV_AT_ONE_PERIOD = 0.50975` is **one rock on one orbit**. Everything else
about a rebuilt scenario keeps working, so a margin still quoting the old
requirement looks *entirely healthy*. Now: anchor is a core field, seeded free only
when the installed config `is_shipping()`, and `tractor_readout` takes
`Option<f64>` → absent = **no requirement, no margin**, and the panel says *which*
of the two absences it is (law floor vs unmeasured orbit).
**`required_dv_estimate(n)` was DELETED**, not kept — it reached for the constant,
the easy call at every future site and wrong at most of them.

**The anchor cannot be shortcut by rescaling.** 1/lead + period ratio predicts a
3.4× drop on a 2.66 yr orbit; the real requirement falls **~10×** (0.0515 vs
0.50975). **Off by 2.9×.** Both orbits share `v_inf`/`b_offset` so they need the
*same* b-plane shift — what differs is how much of it an along-track nudge buys
(approach geometry, not period). Pinned, because the shortcut fails invisibly.

## Costs + the headline

- anchor solve **28.8 s on the shipping orbit, 41–74 s on a 1.44 yr one across
  three runs** (vs ~10 s build) → separate on-demand action, shown running. **It
  scales with the period** (one period of lead on a longer orbit = a longer
  propagation), and the knobs reach past 3 yr — so the long-period orbits worth
  exploring are the slow ones to score. UI copy says **"about a minute"**,
  deliberately not a range: the first draft promised "30–60 s", the next run took
  63, the one after that 74. Live solve reproduces the recorded constant to **all
  six digits**.
- **It teaches what it was asked to:** one 200 t plan, one 6.0 yr lead —
  **margin 0.372× on the shipping orbit, 1.096× on a long-period one.** Fails and
  closes, from one knob. Kernel-free pin: a plan inside the knob ranges reaches
  **29×**, so the bench teaches success too, not only the 12.6× shortfall.

## Invalidation — results die, intentions live

Rebuild **refuses while ANY worker runs** (grid/tier2/verify/mass/tow each hold an
`Arc` of the current scenario; one landing after `poll_build` would install a
number about a replaced threat). Refusing *by name* beats a generation counter
through six result types. `poll_build` also now drops the **tow probe**.
`_invalidate_derived_views()` clears `pork_online` + grid columns + plan verdict
booleans + debounce. **Survives:** planner lead/Δv and the tractor's six knobs —
the first version re-seeded the bench every install, which would have reset the
spacecraft to 20 t whenever the orbit moved and made "change the orbit, watch the
margin" a comparison of two different tractors.

## Keys / UI

**One new action:** `[N]`. `[ENTER]` reuses `plan_commit` (apply what is dialled),
`[E]` reuses `pork_verify` ("stop estimating, go measure") — now its meaning in
three views. Knobs are a table (`Sim.THREAT_KNOBS`), 5th knob = one row. Three
bottom-centre panels now mutually exclusive via **one helper**, not three
hand-written pairs.

**A rounded literal is worse than an obviously wrong one:** the offset knob's table
placeholder read `6378.0` — R⊕ to four digits, **136.6 m short** — so the
placeholder, not the core, set the ceiling and the knob stopped just inside the
grazing boundary. Placeholders in these tables are now deliberately slack
(`9000.0`) so the core always binds.
