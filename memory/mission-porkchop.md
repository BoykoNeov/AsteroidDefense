---
name: mission-porkchop
description: "Phase-2 §8 Lambert/porkchop mission-design layer — deliverability overlay, coupled-direction impulse, real launch vehicles"
metadata: 
  node_type: memory
  type: project
  originSessionId: a4325207-628d-448a-9e1c-162d88f874e6
  modified: 2026-07-27T12:28:47.798Z
---

Phase-2 §8 "Lambert / porkchop mission design" — the deliverability layer that
turns "a Δv appears at the asteroid" into "a rocket launches, cruises, arrives,
and nudges it" (the §7/§180 honesty gap). Started 2026-07-21 after Tier-2 done
(see [[tier2-forces]]). **User chose the fuller build on both open axes** (over
the advisor's minimal-cut rec): (1) **couple the impulse DIRECTION** to the real
Lambert arrival geometry, not the along-track idealization; (2) **include real
launch vehicles** (bounded single-launch / no orbital assembly — that stays
Phase 3).

Three core modules, all committed + pushed, all tests green:

- **`core/src/lambert.rs`** (commit `ad00379`, 7/7) — universal-variable
  (Curtis Alg 5.2), single-rev short-way prograde. `r1,r2,Δt,μ → (v1,v2)`.
  Two-body is CORRECT for the planning layer (a cruise IS two-body), NOT a
  display-grade shortcut. **180° collinear → `DegenerateGeometry` (porkchop gap,
  never NaN)**. μ caller-supplied (no 2nd hardcoded μ_sun). Validation: round-trip
  vs `KeplerPropagator` (primary), an **independent published example fetched not
  recalled** (poliastro Izzo docs, ~0.02 m/s), free energy+ang-mom conic
  invariants, arrival-reaches-r2.
- **`core/src/launch_vehicle.rs`** (commit `00c203a`, 5/5) — real C3(km²/s²)→
  payload(kg) curves. **HARD GATE was provenance** (no kernel to machine-verify,
  plausible-recallable-wrong trap): every knot **fetched via `gh api` from AMAT's
  `launcher-data/*.csv`** (github.com/athulpg007/AMAT), sourced from NASA LSP
  Performance website via Girija arXiv:2310.05994. 5 vehicles (Atlas V551, Falcon
  Heavy reusable/expendable, Vulcan Centaur, Delta IV Heavy), linear interp.
  ~~**0-outside-range = infeasible** (mirrors AMAT interp1d fill_value=0)~~ —
  **CORRECTED 2026-07-27: 0 ABOVE the last knot only**; below the first knot it
  holds that knot flat (see the launch-window-map entry below).
  **FULL tables since 2026-07-27** (101/10/100/64/100 knots, `gh api`-fetched):
  the earlier ~10-point downsample shipped a doc claim of "<1% interp error" that
  had **never been measured** — measured, it was **8.9%** (Atlas V near C3=95),
  3.2% FH-reusable, 2.7% Vulcan, because the curves bend sharply near each
  vehicle's energy limit = exactly where a fast intercept lives. Transcription was
  faithful (all 11 old knots matched the full table EXACTLY); only the sampling was
  too sparse. New tests pin **row counts + strict C3 ordering**, since interp /
  monotonicity tests stay true of a subset and would not catch a re-downsample.
  Remaining caveat: delivered mass modelled AS impactor mass (Phase-3).
- **`core/src/mission.rs`** (commit `003565d`, 7/7) — the composition, **split by
  cost like the live force menu** (coupling direction ⇒ deflection check needs a
  full-field re-propagation per cell, O(N²) = hours). **Cheap grid**
  (`porkchop_grid`): pure scalar Lambert over Earth/asteroid states looked up ONCE
  per epoch (not N×M ephemeris queries); records C3, arrival |v_rel|, and the
  **along-track projection** `v_rel·v̂_ast` — a free effectiveness proxy for the
  whole point of coupling: **deliverable ≠ well-aimed**. Grid is
  **vehicle-independent**; C3→mass maps per launcher after (`cell_delivery`), so
  switching vehicles never re-solves Lambert. **On-demand verify** (`verify_cell`):
  re-propagate ONE selected cell in the full n-body field after the real VECTOR
  impulse `β·(m_sc/M)·v_rel_vec` via the existing `DeflectionScenario::evaluate`
  (already takes Vector3 — **zero new deflection code**); exact b-plane perigee.
  `required_impactor_mass` bisects mass to a target perigee with the advisor's
  **mass-cap degenerate-direction guard from day one** (v_rel⊥track → no mass
  deflects → `InfeasibleAtCap`, never a runaway bisection). Endpoints real (Earth
  ephemeris, asteroid **nominal pre-deflection** track). **8/8 tests, made
  DISCRIMINATING after an advisor review** (first cut verified wiring not
  behavior — `perigee>=0` passes even with the impulse un-applied): solver tested
  **kernel-free** (ZeroForce straight-line, like deflection.rs — solved mass
  ACTUALLY delivers its target = mis-bracket catch, monotone, InfeasibleAtCap) +
  a **cheap kernel-gated (~2 props)** real-field composition test (zero mass ⇒
  nominal HIT, delivered mass ⇒ MISS). Split algorithm-from-composition dropped
  the suite 407s→26s. Lesson: **a passing kernel-gated test that RAN (not skipped,
  see kernel-skip trap [[gdext-binding]]) can still be non-discriminating** — the
  advisor's "the test discriminates" bar is separate from the "the test ran" bar.

**2026-07-27 — multi-rev Lambert landed, AND IT EXPOSED A SHIPPED BUG.**
`lambert_universal_multirev` (N-lap transfers) needs a **different root-finder**:
inside the band `z ∈ ((2Nπ)², (2(N+1)π)²)` the TOF diverges at both edges and dips
to a minimum, so Newton walks into the wrong basin → **bracket + bisect** on a
`LowZ`/`HighZ` branch; below-minimum Δt = `NoSolutionForRevolutions` (a real
geometric gap carrying the threshold, never NaN). Validated by flying each root
through the **analytic Kepler propagator** to confirm it reaches r2 — independent
across a formulation gap (universal-variable/Stumpff vs elements/Kepler).

**THE BUG:** `T(z)` rises monotonically to ∞ as `z → 4π²`, so a single-rev root
exists for EVERY Δt — but Newton from the `z=0` seed **overshoots past the pole and
converges in the 1-rev band**. Result looks perfect (reaches r2 on time) but is a
**lapping transfer labelled direct, carrying a different C3** — the worst porkchop
failure: a plausible number in a mislabelled cell. Fixed by `SINGLE_REV_Z_MAX`
clamping the iterate; regression test is **physical** (a sub-one-rev transfer must
finish inside its own orbital period), not a peek at z, plus a long-window case for
where the clamp degenerates to pure bisection.

**Fix + feature are TWO HALVES OF ONE CHANGE** — the default grid TOF is ~3.6–3.9 yr,
inside the affected zone, so clamping ALONE would have replaced accidental lapping
transfers with the honest direct arc, which at long spans is ruinous: measured at
2.6 yr, direct **C3 = 933 km²/s²** (no launcher reaches it) vs **55 lapping**. It
would have turned the long-TOF half of the heatmap infeasible. So
`best_transfer_metrics` picks lowest-C3 across `N=0…max_revolutions` (both branches),
`porkchop_grid` takes `max_revolutions`, and **`TransferMetrics::revolutions` says
which trajectory a cell IS** (lapping the Sun = a different cruise, not just a
different number).

**2026-07-27 — THE GODOT LAUNCH-WINDOW MAP LANDED ([4]), closing the §8 view.**
Layer mirrors the Tier-2 menu exactly (proven split): `mission_core::PorkchopView`
(godot-free, worker-callable) → `Mission::begin_porkchop`/`poll_porkchop` on its
**own** mpsc channel → `Sim` under its **own** `pork_online` flag → `porkchop.gd`
(pure display). Grid built **on demand when the view opens**, never on the build
path. Keys: `[4]` open, arrows = cursor, `[L]` launcher, `[D]` metric, `[E]`
full-field verify. Harness `godot/tests/_pork_shot.gd` (autoload while running,
removed after) drives everything through `main._input()` real key events.

- **THREE kinds of empty, three fills** — `c3 = -1` no transfer at any lap
  (background) / `payload = 0` this launcher can't reach that C3 (dim floor) /
  reachable-but-poorly-aimed (dark live cell). Measured on the shipping 120×120:
  **4849 blank / 7193 unreachable / 2358 reachable** (Atlas V). Sentinels never
  NaN — one NaN flattens every ramp min/max.
- **`DEFAULT_MAX_REVOLUTIONS = 2`, decided on a MEASUREMENT** (24×24, 379 transfers,
  windows FH-expendable reaches): **N≤0 → 21 · N≤1 → 56 · N≤2 → 95 · N≤3 → 129** at
  4.8/47.8/69.0/106.8 µs/cell. Direct-only shows **under a quarter** of real missions. No
  knee exists (laps always keep opening windows) so stopping is a cost call, and the
  test prints the whole table. Shipping grid solves in **851 ms** on a worker.
- **THE VIEW FOUND A SHIPPED BUG, in the CHEAP direction**: `payload_kg` returned 0
  outside the table at **both** ends (faithful AMAT `fill_value=0` port). Wrong at
  the low end — a lower C3 is an EASIER launch. Two laps pushed cheapest cells to
  **C3 = 0.34**, under the 1.0 where 4 of 5 tables start → real flyable windows drawn
  unreachable, captioned "TOO MUCH C3 FOR THIS ROCKET", **the exact reverse**. Now:
  below first knot = hold it flat (conservative); above last knot = 0 (real ceiling).
  **`0` now means exactly ONE thing.** Lesson: *where a table starts is a sampling
  artefact, where it ends is physics* — `min_c3_km2_s2` is NOT a feasibility floor.
  **The 12×12 test grid never went below 1.0 and passed throughout**; only the
  shipping 120×120 exposed it → the check is now against the grid's **measured**
  cheapest cell, not a fixed sample. Found by the advisor pre-commit, not by tests.
- **Frontend must not invent a third rock**: `SrpParams::sub_km_rock` hides 150 m /
  2000 kg/m³ as locals, so needing a MASS meant restating them → named
  `THREAT_RADIUS_M`/`THREAT_DENSITY_KG_M3` → `threat_mass_kg()` ≈ **2.83e10 kg** +
  a drift test pinning `3/(4rρ)` to the SRP default (the `SB441_BODIES` treatment).
  `mission.rs`'s `2.0e10` is a test fixture and stays one.
- **Verdict = `|B|` vs `b_capture`** (never perigee-vs-capture, the 3C-2c bug), and
  `CellVerdict` is an **enum** (`CleanMiss`/`Encounter`/`NotHyperbolic`) so the BEST
  outcome can't share a `-1` with "not verified yet". Measured: 3956 kg through a
  C3 22.3 / **2-lap** / 2049-d window imparts **+3.06 mm/s** → |B| 6930 km inside the
  11 311 km disc = **SURFACE IMPACT**. That is the layer's honest headline — one real
  launcher delivers ~**1/65th** of the ~0.2 m/s the curve wants. Mass has no bus/
  propellant bookkeeping, so it's OPTIMISTIC → the verdict errs safe.
- **Heatmap = an ImageTexture (1 texel/cell, NEAREST), not 14 400 draw_rects/frame**
  (`_process` redraws while visible). Nearest also matters on merit: smoothing would
  invent gradients between windows never solved.
- **"Verified by picture" earned it again**: every assertion passed while the FIRST
  screenshot was a tiny corner blob — `pork` was missing from
  `main._sync_overlay_sizes`. Two more collisions were image-only (HUD mission panels
  over the heatmap → they now stand down for this view; readout col2 overlapping col1
  → now sized by `_font.get_string_size` of the longest left line, not a guessed
  char count). **All of these pass a test suite; none survives looking at the frame.**
- Godot gotcha: a NEW `class_name` isn't in `.godot/global_script_class_cache.cfg`
  until an import — `godot --headless --path godot --import` first, or every
  reference is a parse error. (It segfaults on shutdown; the cache still writes.)

**2026-07-27 — `required_impactor_mass` IS NOW ON THE FRONTEND (`[M]`), closing the
last porkchop item.** `[E]` says *does this launcher work here* and stops; `[M]`
says *how much mass would*, and the ratio is the headline as a number. 5th mpsc
channel, `Mission::begin_required_mass/poll_required_mass/required_mass` →
`Sim.pork_mass_solving` → a panel line under the verdict; `[M]` shares its keycode
with `plan_toggle` and the `pork.visible` guard wins by being earlier in main.gd's
chain (the arrows precedent).

- **My cost estimate was 10× low; the advisor's two corrections were both right and
  both invisible in the code.** Probe cost is **cell-dependent** (a probe re-flies
  arrival→encounter: **18.2 s** at a 10.8 yr lead vs **5.8 s** at 3.2 yr), and the
  *common* `InfeasibleAtCap` path is the EXPENSIVE one (walks the whole doubling
  ladder before it can say no). Naive (100 kg seed / core's `1e-4` tol / 1e9 cap)
  measured **455 s** feasible, 194 s infeasible. Shipping: **46 s** best-coupled,
  **31 s** hopeless, ~3 min slow tail. Seed killed ~11 doublings, tol ~7 bisections,
  cap stops an unreachable window climbing to 1e9. **Also corrected a shipped doc
  claim: `[E]` was documented "~1 s", it is 5.8–18.2 s** — never measured, and it is
  the unit this solve is priced in.
- **The seed had to be made SAFE before FAST — a real bug in the shipped core.**
  `required_impactor_mass` returned `seed_mass_kg` verbatim if the seed already
  cleared the target: an upper bound reported as *the requirement*. Harmless with a
  tiny seed, fatal once you seed from something meaningful to save propagations —
  the displayed physics would be a function of the seed. Now brackets **downward**
  (halve until a mass FAILS) as well as up; pinned kernel-free by seeding **100×
  high** and demanding the same answer back. Only then was seeding at
  `heaviest_deliverable_kg()` (**14 714 kg**, derived from the LSP tables) safe.
  Also took a `rel_tol` parameter (0.05 ships — the readout can't show more).
- **Cap size is a DISPLAY decision**: `100 × heaviest` = **1 471 393 kg**. Ten
  (the first instinct) sits *below* every window's requirement → every cell reads
  "over the cap" and the number the feature exists to print never appears.
  `InfeasibleAtCap` renders as **data not failure** (clean-miss-sentinel trap again).
- **`SAFE_PERIGEE_TARGET_M = 2.0e7` named once in `core::scenario`** — was a bare
  literal in `viewer/src/bin/curve.rs`, `curve.json`, and the binding's
  `required_dv_matches_curve_json` (which now asserts it). The map's *mass*
  requirement and the headline curve's *Δv* requirement must be quoted against the
  same bar or they'd look comparable and not be. **The readout NAMES the target** —
  "required mass" alone reads as *mass to miss Earth*, a smaller number. It is a
  MARGIN, not a hit test (the verdict stays |B| vs b_capture).
- **The test is the ROUND TRIP, not the return value**: fly the solved mass through
  the independent `[E]` path — **20 232 kg → 20 905 km** (clears 20 000) — AND fly
  20 % less and require it to FAIL — **16 185 kg → 16 453 km, short**. Without the
  second half, returning the cap passes everything. Both outcomes from one grid.
- **Vehicle-INDEPENDENT**: keyed `(launch, arrival)` with no vehicle index; `[L]`
  recomputes only the ratio. Keying by vehicle would re-fire 30–180 s of propagation
  on a keypress that cannot change the answer. The harness presses `[L]` and asserts
  the requirement is still current and nothing re-fired.

**OPEN / next:** direction-coupling makes the headline along-track curve one lens
(phase-sensitivity story); Lambert now DELIVERS the kinetic impactor the §5
deflection-method spectrum (gravity tractor / nuclear) would choose between.
Full HANDOFF sections: "Phase-2 mission design — 2026-07-21 session",
"The deferred leftovers, closed — 2026-07-27 session",
"The Godot launch-window map — 2026-07-27 session", and
"Required impactor mass on the map — 2026-07-27 session".
