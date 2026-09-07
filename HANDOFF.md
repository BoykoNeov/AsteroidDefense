# Asteroid Deflection Simulator — Development Handoff

This document is the starting context for continuing development in Claude Code. It captures the project vision, the architectural decisions that are **locked** (build against them, don't re-litigate them), the physics that must be correct, the validation strategy, the known hard problems, and a concrete first-tasks list.

> **Revision note (2026-06-23).** This handoff was pressure-tested and re-scoped after the first review. Locked decisions: the MVP renderer is **pure Rust** (Godot deferred to Phase 2); the MVP must deliver an **honest hit→miss flip** (not just the curve); the asteroid is integrated as a **test particle in the DE440/441 ephemeris field from Tier 1 onward**, so every tier is a pure force-term toggle (not a structural rewrite) and ASSIST is the oracle from day one. The physics target is **realism** — operationalized as the tiered force model in §5. Sections §2, §3, §5, §6, §7, §8, §10 were revised accordingly. The pre-review version is preserved in `HANDOFF.backup.md`.
>
> **Second pass (2026-06-23, same day).** A follow-up discussion resolved the remaining open questions and two previously-implicit decisions. Now locked: integrate in the **barycentric (SSB-centered) ICRF frame** in **SI units**, present heliocentric (§2, §5); **dop853 is the MVP integrator** (IAS15 is a Tier-2 long-arc upgrade) and the **clock interpolates from dop853's dense output**, not linearly (§4–§7, §10); the **pure-Rust viewer is egui** (egui_plot + painter), plotters optional (§2, §8); the headline Δv-vs-lead-time curve **fixes the impulse phase**, with phase exposed as a **separate** interactive view (§5, §7); **scenarios/fixtures are JSON** (§6); a **task-0.5 ASSIST+DE441 build / ANISE DE-position spike** gates the plan, with an explicit **fallback-to-Option-B trigger** if it stalls (§10); the MVP **soft-caps impulse magnitude** to kinetic-impactor plausibility and carries **delivery + determinism honesty caveats** in UI copy (§1, §5). The MVP perturber set stays Sun+8 planets+Moon, with the force term and ANISE loader **designed to add the 16 asteroid perturbers** at Tier 2 (§5).

---

## Where things stand — 2026-09-07

A dashboard, because §10's task list has been complete since the MVP and the
truth has lived in the dated session sections at the end of this file since.
Read this table first, then the session that owns the layer you are touching.

| Layer | Status | Where |
|---|---|---|
| Tier 1: DE440 test-particle field, dop853 + dense output, fixed-cadence clock, close-approach detector, b-plane hit test with focused capture radius | done; ASSIST oracle to 4.5e-11 | `core/src/{perturber_field,integrator,clock,close_approach,geometry}.rs` |
| The thesis: `required_dv` curve, kinetic / nuclear-standoff / gravity-tractor spectrum | done; curve slope −1.05 measured | `core/src/deflection.rs`, `forces/tractor.rs`, `viewer/` |
| Tier 2: 1PN, Yarkovsky, SRP, J2, 16 sb441 perturbers, Pluto toggle | done; per-term closed forms + Apophis vs Horizons capstone | `core/src/forces/*`, `core/tests/capstone_neo_vs_horizons.rs` |
| Mission design: Lambert (multi-rev), porkchop, launch vehicles, required impactor mass, cell verify | done | `core/src/{lambert,mission,launch_vehicle}.rs` |
| Tier 3a: covariance → b-plane Jacobian → P(impact), linearity shell | done (the synthetic rock's covariance is invented, labelled) | `core/src/uncertainty.rs` |
| **Tier 3e: real covariances** — JPL SBDB cometary elements + 8×8 matrix at their own epoch, marginalised, unit-scaled, mapped through a measured element→state Jacobian, and gated three ways against JPL's own numbers | **done 2026-09-06**; Apophis' 2029 ellipse is 18.2 km — the same size as our own dynamical error over the arc | `core/src/{sbdb,frames}.rs`, `pyref/fetch_sbdb_covariance.py`, `core/tests/fixtures/apophis.sbdb` |
| **Tier 3b: keyholes** — Öpik (ξ, ζ) frame and b-vector sign pinned, resonant circles in closed form, keyhole widths, the keyhole map, **the 3:4 keyhole flown to a return impact** | **done 2026-09-02** | `core/src/keyhole.rs`, `examples/probe_keyhole_{map,return}.rs`, `docs/keyhole_map.*` |
| **Tier 3c: keyhole targeting** — aim at any resonance on either branch, fly it, refine to the floor, and read the return in **its own** Öpik frame; the planner's `KEYHOLE` row | **done 2026-09-05** | `core/src/keyhole_target.rs`, `godot/scripts/{sim,planner}.gd` |
| **Tier 3d: P(impact) at the resonant return** — the covariance flown through *both* encounters (the chaining is in the propagation, not a matrix product), finite-difference steps re-measured on the return, the chained gain checked against the closed form to 14 % | **done 2026-09-06**; the answer is set by how well the orbit is known, not how well the impulse is aimed | `core/src/keyhole_target.rs`, `core/examples/probe_keyhole_probability.rs` |
| **Integrator convergence** — the forward tolerance swept `1e-9…1e-13` against the 12-yr campaign and the 15-yr keyhole return, crossed with the snapshot cadence; the bit-for-bit determinism gate; the flyby's amplification measured | **done 2026-09-06**; every published number is converged, but because the 1-day cadence **caps the step** — the same tolerance uncapped is 129 km off. **IAS15 retired, not deferred** (no oracle exists for it) | `core/examples/probe_integrator_convergence.rs`, `ImpactorConfig::forward_rtol` |
| **Tier 3f: keyhole doors flown** — five resonances aimed at, flown to a return impact, refined to the timing floor and both door edges bisected; the circle-crowding check in closed form | **done 2026-09-07**; the linearised *width* is conservative by ≤1.44×, but **four of the five doors do not contain their own circle** — placement is 1 to 28 half-widths off and follows no law, so the frontend's width-multiple alert became an additive 100 km band | `core/examples/probe_keyhole_placement.rs`, `core/src/keyhole_target.rs`, `godot/scripts/sim.gd` |
| Godot frontend: DE440 orrery, real NEO scenery, planner, b-plane view (with the keyhole map, `[H]`), launch-window map, Tier-2 force menu, tractor bench, threat-orbit knob | done; keyhole overlay **seen on screen 2026-09-05** and its captions budgeted | `godot/`, `godot/rust/` |
| Godot visual/perf pass: 3D world in its own viewport with 4× MSAA and phosphor persistence (peak-hold trails), per-frame position memo, cached 2D orbit traces, the `_perf.gd` frame-time harness and `run_harness.ps1` | done 2026-09-05; the 2D map went 18.9 → 7.1 ms/frame, native calls/frame 764 → 29; follow-ups in `docs/plans/2026-09-05-visuals-performance-followups.md` | `godot/scripts/main.gd`, `godot/shaders/phosphor_persist.gdshader`, `godot/tests/` |
| **Frontend speed**: the display comet flown on its own worker instead of the build worker, and every DE440 body served by one batched binding call filled on demand | **done 2026-09-06**; time to a threat solution 34.0 -> 25.2 s, native calls/frame in the 3D views 29 -> 6 | `godot/rust/src/lib.rs`, `godot/scripts/sim.gd` |
| **Frontend startup**: the DE kernel read moved onto its own worker, the boot POST given a third state to report it, and `_ready` split into timed phases | **done 2026-09-07**; `_ready` 29 -> 2 ms, and the roadmap item's "646 MB DE440" turned out to be a 32 MB file on a fragmented spinning disk | `godot/rust/src/lib.rs`, `godot/scripts/{sim,boot,solar_system,main}.gd` |
| **Frontend legibility + the b-plane's frame cost**: tags and captions placed instead of drawn where they fall, the encounter tracks as runs of polyline, a persistence control on `[I]` | **done 2026-09-07**; b-plane view 10.1 -> 9.1 ms, and a single frame-ms number caught producing two opposite wrong conclusions | `godot/scripts/{tag_layer,encounter,main,solar_system,hud}.gd`, `godot/tests/_shot.gd` |
| **The two drawn claims nobody had measured**: the b-plane view's resonant circles clipped to the viewport and tessellated to a pixel budget, and the Tier-3 ellipse's *shape* put through the ±3σ shell along its own axes | **done 2026-09-07**; the old whole-circle tessellation drew the widest resonance **59.9 px** off inside a 720 px view (now 0.113 px, and 3 points instead of 256), and the drawn ellipse holds everywhere the σ knob reaches — but the linearity scalar that was supposed to say so reads **130× too small** on the axis that matters | `godot/scripts/plot_geometry.gd`, `godot/tests/test_geometry.gd`, `core/examples/probe_tier3_drawn_shape.rs`, `core/tests/tier3_drawn_shape.rs` |
| **The ranking that was a ratio**: resonant circles ranked by kilometres from their own *door* (`margin`) instead of by keyhole *widths*, the planner's alert cut on that same row, and the note branch nobody had seen fire finally executed | **done 2026-09-07**; the width ratio divides away exactly the additive placement error the five-door batch measured — though on this rock the two rankings never actually parted company, in 2 000 random geometries or on any plan the planner can dial | `core/src/keyhole.rs`, `godot/rust/src/{mission_core,lib}.rs`, `godot/scripts/sim.gd`, `godot/tests/{test_orrery,_shot}.gd` |
| **The calibration taken outside its own domain**: the same 3:4 door flown at four deflection lead times, the lead-sweep gate that made it affordable, and the placement band resized on what the planner can actually dial | **done 2026-09-07**; the door centre moves **+19.3 km -> +210.6 km -> +467.9 km -> no door at all** across leads 4383, 900, 450 and 150 days, and **only the first of those is not dialable** - so the five doors that set `KEYHOLE_PLACEMENT_KM = 100` were every one of them flown outside the range the constant is used in | `core/examples/probe_keyhole_placement.rs`, `core/src/keyhole.rs`, `godot/rust/src/{mission_core,lib}.rs`, `godot/scripts/sim.gd` |
| **The lead was the variable, and the crowded register fired**: the 3:4 door flown at 300 d and at 200 d, the frame proposed as the mechanism and falsified, and the several-doors register searched for on real physics | **done 2026-09-07**; of lead, Δv, ξ and the angle round the circle, **only the lead orders all five flown doors** — not the impulse (the 200 d door takes a *smaller* nudge and sits *further* out, +786.0 vs +648.2 km) and not the place on the circle (the 300 d and 12 yr doors are **0.48° of arc** apart with 34× the error). Rebuilding each flight's own Öpik frame makes the spread **worse**, 767 → 1375 km. `KEYHOLE_PLACEMENT_KM` 500 → **800**, and the width claim moves from ≤1.44× to **≤2.11×** on a door that is closing | `core/examples/probe_keyhole_placement.rs`, `godot/rust/src/mission_core.rs`, `godot/scripts/sim.gd`, `godot/tests/{test_orrery,_shot}.gd` |
| **The shape number promoted out of its probe**: the per-axis linearity residual moved from two private copies into `LinearityReport::shape_residual`, with pure-math tests that know the right answer, and the blind scalar's under-reading factored | **done 2026-09-08**; behaviour identical (shipping minor 0.0007, top stop 0.5610, scalar 0.0043) and the 130x gap turns out **not** to be the 206:1 aspect ratio - it is 143 drawn half-widths of shell reach x 0.90 of the residual lying across the needle, because `shell_scale` is 0.693 of the 3sigma half-length, not equal to it. The move also flipped an eigenvector sign nothing tested - the drawn angle read **-90.26 against a published 89.736** with every ratio unchanged - now pinned in the module and folded into a half-turn at the print | `core/src/uncertainty.rs`, `core/tests/tier3_drawn_shape.rs`, `core/examples/probe_tier3_{drawn_shape,uncertainty}.rs` |
| Engineering: CI (fmt, clippy, kernel-free suite, then the physics with kernels cached), kernel fetcher, `DEVELOPING.md` | new 2026-09-02 | `.github/workflows/ci.yml`, `tools/` |

### What is next, in order

1. ~~**Real covariances from the SBDB.**~~ **DONE 2026-09-06** — and three of
   that line's clauses were wrong: the elements are **cometary** (`e, q, tp,
   node, peri, i`), the matrix is **8×8** (the non-grav parameters are estimated
   alongside the orbit and marginalise out), and a **round-trip cannot validate
   it** — the same unit convention runs both ways, so a degrees-for-radians error
   cancels exactly. Three external gates instead: JPL's published per-element σ
   against `sqrt(diag)` (exact), JPL's own Cartesian state at the covariance epoch
   (1.4 m), and a Monte Carlo in element space against `J Σ Jᵀ` (< 5 %). The
   payoff: Apophis' real covariance flown to its 2029 flyby gives a **18.2 km ×
   0.48 km** ellipse and **P = 0** at 19 571 σ — the correct answer — beside a
   **15.1 km** residual of our own dynamics over the same arc. *The two are the
   same size*, which is the batch's real finding. The invented label is retired
   for Apophis; the shipping campaign's rock is designed and keeps it. Bennu is
   deliberately left: its solution estimates SRP parameters, not `A2`, so its
   covariance describes a propagation we do not reproduce. See *Real SBDB
   covariances*.
2. ~~**P(impact) rising near a keyhole.**~~ **DONE 2026-09-06** — and it does
   not rise, at the shipping uncertainty: the return's 1σ ellipse is a needle
   131 334 km long lying in the same coordinate Δv moves, so P is 0.064 flat
   across the whole door (contrast 1.02×). Shrink the *whole* covariance 12× and
   the door appears as a 12× peak; 100× and it is a hard 0→1→0. See *P(impact) at
   the resonant return*. `uncertainty_sampling_plan` is unchanged — the return
   does not exist on the nominal, so its refusal never applied.
3. ~~**The Tier-3 ellipse on the Godot b-plane view.**~~ **DONE 2026-09-06** —
   `[U]` orders the solve on a worker (measured **34.6 s**, not the ~17 s this line
   guessed) and holds it, so the σ knob on `[Z]`/`[X]` costs nothing. Two things
   the item did not anticipate. The **frame** had to be settled first: the
   sensitivity's b-plane basis is arbitrary-but-deterministic, so every *scalar*
   is invariant under it but an **ellipse's orientation is not** — drawn in that
   frame the picture would have had the right axis lengths at a rotation nobody
   chose. The covariance is rotated into the view's pinned Öpik axes in the core,
   and the two planes agree to `1.95e-10`. And the result points somewhere: the
   1σ ellipse is **168.71 × 0.82 km lying 0.3° off `ζ̂`** — the *timing* axis,
   which is the same direction a Δv nudge moves and the same direction the
   resonant return's needle lies along. At the default zoom it is about **one
   pixel**, and the view says so rather than fattening it. See *The uncertainty
   ellipse on the b-plane*.
4. ~~**The keyhole width's order-unity slack, measured on more than one case.**~~
   **DONE 2026-09-07 — and the item was two questions wearing one name.** Five
   resonances (3:4, 5:7, 7:10, 6:5, 2:3) were flown to return impacts and had
   both door edges bisected. The *width* calibrates: the linearised door is
   conservative by at most **1.44×** once you divide out how much of the return's
   capture disc its own spatial offset `ξ₂` has already eaten — and that chord
   model is not a fit, it is `b = 11 240 ± 60 km` measured at all **ten** door
   edges. The *placement* does not calibrate, and that is the finding: the door
   centres sit 2.0, 2.2, 7.2, 19.3 and 26.8 km from their circles, which is 1.0
   to 27.8 half-widths, and **four of the five doors do not contain their own
   circle**. The offset is not a fixed distance, not a fixed number of widths,
   and not a fixed `a'` bias (`offset × |∇a'|` spans 196 to −5403). So the
   planner row cannot quote a multiple of the width, and the frontend constant
   that did (`KEYHOLE_ALERT_WIDTHS = 4.0`, which missed three of the five) is
   replaced by an additive band, `|distance| ≤ 100 km + width/2`. The 1.64
   half-widths this line called a conservative width was never a width — it was
   the placement error. See *Five keyholes flown*. **Its two loose ends closed
   2026-09-07**: the core no longer ranks circles by a width ratio (which divides
   away the very error the batch measured), and the note branch that had never
   been seen to fire is now executed by a test. See *The ranking that was a
   ratio*. **And its third loose end - "the placement error is measured at
   exactly one xi per circle; whether it varies along a circle is unknown, and
   that is what a sixth campaign should ask" - was asked and answered 2026-09-07.**
   It varies, by 24x, and the knob that moves a plan along a circle is the
   **deflection lead time**, which the planner already exposes. The five-door
   calibration turns out to have been taken at a lead the planner cannot dial.
   See *The calibration taken outside its own domain*. **And its last open half -
   the band was calibrated at 200 d and up while the slider goes down to 30 - was
   closed 2026-09-07.** Below 200 d the answer is not a bigger number: on the 3:4,
   2:3 and 5:7 the door **ceases to exist** between 150 and 200 d (all three still
   cross their circle at 125 d, and all three floor 19 447 / 25 140 / 29 341 km out
   with their timing spent), the 3:4 circle **cannot be reached at all** by 100 d
   (the 2:3 and 5:7 ended on the wrong branch there and are NOT MEASURED, not
   negative), and none of the three is reached at 75 or 50 d. The 150-to-200 d
   bracket is the 3:4's alone - it is the only circle measured on both sides of
   it. The constant stays at 800 and
   gains a stated domain floor. Three gates had to be discarded or fixed to get
   there - `screen` scores **zero hits at 200 and 300 d where the door is known**,
   the ladder aim is geometrically unusable below ~150 d (four `bracketed = false`
   walls), and `xi_sweep` was reporting a hole in the curve as its end. See *The
   band's domain floor, measured*.
5. ~~**dop853 → IAS15 crossover.**~~ **RETIRED 2026-09-06 — measured, and no second
   integrator is warranted.** Three corrections to that line. (a) The premise was
   wrong: it leaned on the 15.1 km residual vs JPL, which is *unmodelled forces*
   (planetary GR, radial `A1`), and no integrator fixes a missing force. (b) dop853
   **is** converged where it ships — the encounter perigee moves 0.07 m and the 3:4
   return's timing coordinate 365 m across four decades of tolerance, against a
   ~25 km keyhole width, so every keyhole conclusion holds. (c) But not for a good reason: the 1-day
   snapshot cadence **caps the step**, and the same tolerance uncapped is **129 km**
   off over the cruise. So the shipping accuracy is a property of the architecture,
   which is now a named knob (`ImpactorConfig::forward_rtol`) with a guard test.
   And IAS15 is retired rather than deferred because **there is no oracle for it**:
   REBOUND self-gravitates the planets (§6 says so), Horizons cannot resolve this
   class of flyby, and a resonant return is in no truth table — leaving
   dop853-at-a-tighter-tolerance as the only oracle, which is what was run. See
   *The integrator, measured instead of replaced*.
6. ~~**One contradiction inside the repo (891 km vs 786 km).**~~ **RESOLVED
   2026-09-06 — and the resolution was not the one the item proposed.** Both
   numbers are right; they are two *different shots*, 1.9e-7 m/s apart in Δv, and
   the coordinate they disagree in responds at 5.4e8 km per m/s — so a six-decimal
   Δv print is already a ±271 km smear of it. Re-running the probe and "believing
   whichever it prints", as this item suggested, would have deleted a correct
   number. The bigger find underneath: **neither was the floor.** The shipping
   12-iteration search stops 1.6e-6 m/s short of the minimum; at 20 iterations the
   floor is Δv 0.2165483096 → 1 087 km, where the timing coordinate is −26.6 km,
   i.e. essentially zero, which *strengthens* the module's spatial-floor claim.
   And the search's own reported Δv window is its iteration budget, not a door
   width. See *The floor that was a stopping distance*.
7. ~~**The synchronous kernel load in `_ready()`.**~~ **DONE 2026-09-07 — and the
   file it named was the wrong one.** `load_from` reads **`de440s.bsp`, 32 MB**, not
   the 646 MB DE440 this line claimed; the 646 MB file is the *small-body* kernel,
   which is only stat'd here and mounted on the build worker as it always was. The
   `_ready 10927 ms` observation was real but had never been split — `test_orrery.gd`
   timed all of `_ready` and one line inside it was blamed. Split now
   (`Sim.ready_phase_ms`, printed every run): warm, `load_from` is 26.6 ms of a
   29 ms `_ready` and nothing else does I/O. The seconds are the **disk** — `M:` is a
   spinning drive, and `de440s.bsp` reads at 5–13 MB/s against 35 MB/s for a large
   file on the same platter, i.e. it is fragmented; ANISE reads the whole file to
   heap, so it is a genuine 32 MB read measured at 2.4–6.4 s off the device. Not
   antivirus (a scan verdict is cached; both reads were slow). The read now runs on
   a worker (`begin_load`/`poll_load`) and the boot POST reports it — `_ready` 29 ms
   → **2 ms**, with no cold before/after claimed because a cold cache cannot be
   produced on demand. The batch's real find is the regression it caused and the
   pictures caught: `bodies_online` false at scene load left the 3D world with no
   planet nodes, which threw in `_process` every frame and silently killed every
   line below it — including the comet's span gate. See *The kernel read taken off
   the main thread*.
8. ~~**The keyhole-map circle radius is unbounded**, and **the Tier-3 ellipse is
   drawn at 1 sigma only** — the ±3σ linearity shell has never been run against
   the drawn shape.~~ **DONE 2026-09-07 — and "both are small" was half right.**
   The circle half was smaller than stated *and named the wrong layer*: the core
   is not at fault (`ResonantCircle` already refuses the degenerate case, and the
   widest circle's 9.232e6 km radius is a true answer), the **drawing** was.
   `draw_arc(cc, r, 0, TAU, 256, …)` sizes its tessellation in *angle*, so at the
   zoom-in stop that circle is 795 431 px across, one chord spans 19 500 px, and
   the line drawn through a 720 px viewport sat up to **59.9 px** from where the
   circle is. Clipping the arc to the view and tessellating to a 0.3 px budget
   gives **0.113 px** and costs **3 points instead of 256**. The offline SVG map
   never had the defect — it emits a real `<circle>` inside a clip path.
   The ellipse half was *larger* than stated, and the answer is reassuring while
   the reason it had to be asked is not: the drawn shape **is** supported at 3σ
   everywhere the σ knob reaches (worst 0.561 of the drawn half-width, at the top
   stop, extrapolating to a crossing at scale ~2e3 against a knob that stops at
   1e3) — but the linearity scalar that was supposed to say so reads **0.0043**
   where the axis that matters reads **0.561**, because it normalises against the
   *major* axis of a 205:1 needle. And the first table's scariest row was not
   curvature at all: at the small-covariance end the residual is the integrator's
   own noise, which drops a hundredfold at a tighter tolerance while real
   curvature does not move. See *The two drawn claims*. **Its leftover closed 2026-09-08**: the per-axis number lived only in a
   probe and a kernel-gated test, in two copies, while the one public verdict
   read the blind scalar - it is now `LinearityReport::shape_residual`, pinned by
   pure-math tests that know their own answer. And the 130x was **not** the
   ellipse's 205:1 aspect ratio: `shell_scale` is 0.693 of the 3sigma half-length,
   so the factor is 143 half-widths of shell reach times the 0.90 of the residual
   that lies across the needle. See *The scalar's blindness*.
9. Phase 3 — noting that its first bullet (plausible launch vehicles + payload
   mass budgets) is already built, in `core/src/launch_vehicle.rs` and the `[M]`
   readout. What is actually left there is orbital assembly, standing defence
   systems, and multi-mission campaigns.

---

## 1. What we're building

A solar-system model with 2D and 3D views, focused on **asteroid deflection and mission planning** — specifically, planning and simulating missions to deflect an Earth-bound asteroid, with different missions achieving different degrees of success.

The project has a single educational thesis it exists to demonstrate:

> **Deflecting an asteroid early — many orbits before the predicted impact — is dramatically more effective than deflecting it on final approach.** A tiny nudge applied years out beats a massive shove applied days out.

Everything in the app serves making a user *feel* this. The user should be able to attempt a last-minute deflection, watch it fail, rewind ten years, tap the asteroid once with a small impulse, and watch Earth slide safely out of the way.

**The single most important screen is a plot of required Δv vs. lead time** for a given asteroid and method. That curve *is* the thesis. Build the rest of the system to make that curve legible.

We are aiming at **realism**, not a cartoon: the dynamics that decide hit-vs-miss are modeled at ephemeris quality (see §5), validated against the same oracles professional planetary-defense tools use (see §6). The MVP turns realism *on* only as far as a synthetic teaching asteroid requires; the architecture is built so the remaining realism (GR, Yarkovsky, ephemeris perturbers, orbit-uncertainty) switches on without a rewrite.

**Two honesty caveats baked into the pedagogy, both surfaced in UI copy:**
- **Delivery.** "Tap it once, ten years out" elides the *delivery* problem. In reality an early impulse is gated by launch windows and transfer geometry (the Lambert/porkchop layer, deferred to Phase 2). Until that layer exists, the sim shows *"if you could deliver this impulse, here is what it buys you"* — not *"you can deliver it."*
- **Determinism.** The MVP shows a single deterministic track and a binary hit/miss. Real planetary defense reasons over orbit-determination *uncertainty* — an impact *probability*, not a yes/no (this is the Tier-3 layer, §5). One line of UI copy should say so (*"real tracks carry uncertainty; Tier 3 turns this single line into a probability"*), so the deterministic demo isn't mistaken for the whole story.

---

## 2. Locked architectural decisions

- **Headless deterministic simulation core is the single source of truth.** The 2D renderer, 3D renderer, and mission planner are all *consumers* of the core's state — they never own state themselves. This keeps views in sync for free and makes every scenario reproducible.
- **Determinism means same-build-same-output**, *not* bit-reproducibility across machines. A given compiled binary replays a scenario identically (so rewind/replay and saved lessons are exact). We deliberately do **not** pursue cross-platform bit-identity: adaptive integrators choose steps from floating-point error estimates, so bit-portability would pin compiler flags / math libs / FMA settings for benefit we don't need — the validation oracle (§6) compares within a *tolerance*, never bit-for-bit.
- **Fixed *cadence*, adaptive *step*.** "Fixed timestep" refers to the **clock's snapshot cadence**: the core emits state snapshots on a fixed simulation-time interval for the renderer to interpolate between. The **integrator** adaptively subdivides *between* snapshots to reach each snapshot time under an error tolerance. Fixed snapshot interval ≠ fixed integration step — these are not in tension. Never tie the simulation to Godot's/viewer's frame `delta`.
- **MVP renderer is pure Rust — `egui` is the spine** (immediate-mode shell for the controls, `egui_plot` for the Δv curve, `egui::Painter` for the top-down orbital animation; `plotters` optional later if charts need export/polish). **Godot is deferred to Phase 2.** Rationale: the thesis curve and the hit→miss animation are the entire MVP payload, and the `godot-rust` (gdext) binding is the riskiest, least-physics-bearing part of the stack. egui is the only pure-Rust option adequate at controls + chart + animation in a *single* crate; macroquad would render the animation slightly better but force a worse GUI — and game-like polish is exactly what the Phase-2 Godot frontend is for. Proving the physics behind the cheapest possible renderer de-risks the core independently and gets the "money chart" weeks earlier.
- **Language: Rust core + Rust viewer for MVP**; **Godot desktop frontend in Phase 2**, bound via `godot-rust` (gdext).
- **Desktop only.** No web export. (Godot's web export is a heavy WASM canvas with cross-origin-isolation requirements — not worth it for this.)
- **Build our own astrodynamics**, validated against established reference tools (see §6). The propagator, integrators, force model, Lambert solver, and deflection models are the heart of the project and the thing worth understanding deeply.
- **Integrate barycentric (SSB-centered) ICRF, in SI units; present heliocentric.** The integration frame is the Solar-System-barycenter ICRF — matching DE440/441 and ASSIST directly, and avoiding the non-inertial heliocentric "indirect term" footgun (§5). Use SI (m, s, kg) in the core for legibility; convert only at the ASSIST comparison boundary. Because the core is **f64 everywhere**, the f32 precision worry in §7 is a *rendering-only* problem and never touches a result (f64 spacing at 1 AU is ~15 µm, vs. ~16–18 km for f32).

### The core/consumer relationship

```
Scenario / lesson layer  ──┐
                           ▼
Data sources (JPL/ESA) ──► Simulation core (source of truth) ──► 2D viewer (MVP: pure Rust)
                           │  - composable force model        ──► 3D renderer (Phase 2: Godot)
                           │  - integrators + clock + events  ──► Mission planner
                           ▲                                         │
                           └──────────── apply Δv, re-run ◄──────────┘
```

The mission planner does not compute trajectories itself. It pushes a Δv into the core's state at a chosen time and asks the core to re-propagate. "Did this mission work?" = "run, mutate, re-run, compare miss distance (and, in Tier 3, impact probability)."

---

## 3. Tech stack — the build vs. borrow line

There is a sharp line between code worth reinventing (the lesson) and code that will ship silent, catastrophic bugs if reinvented. Respect it.

### Build (this is the project)

- Orbital-element ↔ state-vector conversions
- Kepler propagation
- Integrator hierarchy: RK4 → adaptive Dormand-Prince (DoPri/dop853) → IAS15/Gauss-Radau-style and a symplectic option (leapfrog/Verlet/WHFast-style) for long stable spans
- **Composable force model** (see §5): each acceleration term (Sun, planets, Moon, GR, J2, Yarkovsky, SRP) a separately-toggleable, separately-validated unit
- Lambert solver (intercept trajectory design)
- b-plane / target-plane geometry, gravitationally-focused miss-distance/capture computation
- Gravitational keyholes
- Deflection Δv models (kinetic, nuclear standoff, gravity tractor)
- Orbit-uncertainty → impact-probability mapping (Tier 3)

### Borrow — link & ship (bugs here are invisible until the encounter is off by seconds = thousands of km)

| Concern | Crate | License | Why not DIY |
|---|---|---|---|
| Time (TDB/TT/UTC/leap seconds) | `hifitime` | MPL 2.0 | Integer arithmetic (no float drift), validated against SPICE to 0 ns on ET↔UTC, flight-proven (Firefly Blue Ghost lunar lander). Time bugs are the classic "subtly wrong and invisible." |
| Ephemerides, frames, GM constants | `ANISE` | MPL 2.0 | Modern Rust rewrite of NAIF SPICE, validated to machine precision. Reads JPL DE440/DE441 kernels; gives ICRF/J2000 frames and μ values that exactly match JPL. **Used in the MVP for both GM constants *and* DE440/441 perturber positions** (the asteroid is a test particle in this field — see §5). Kills the μ-mismatch bug class (see §6); the kernel reader must work before first-light. |
| Linear algebra | `nalgebra` | Apache-2.0/MIT | `Vector3<f64>` etc. Use **f64 everywhere** in the physics, never f32. |

### Borrow — offline oracles only (Python `pyref/`, never linked into the shipped binary)

These generate validation fixtures. Their copyleft licenses don't constrain us because we run them offline and commit only their *output* (data, not a derivative work).

| Oracle | Regime it validates | License | Notes |
|---|---|---|---|
| `hapsira` (maintained poliastro successor) | Two-body / Kepler / Lambert | MIT-family | Analytic-precision short arcs; Vallado Lambert cases. |
| `REBOUND` (IAS15) | Integrator + encounter sensitivity (synthetic, self-consistent N-body) | GPL-3.0 | Gold-standard close-approach dynamics. Self-gravitates the planets — see §6 oracle ladder. |
| `ASSIST` (REBOUND extension) | **Full ephemeris-quality force model** (GR, Sun/Earth J2, Moon, 16 main-belt asteroid perturbers, A1/A2/A3 non-gravs) | GPL-3.0 | Test particle in the **DE441** field on IAS15, validated to ~meter level vs JPL over decades. **Its force-term list IS our realism spec.** Ships first-order **variational equations for all terms → built-in covariance mapping** (direct gift to Tier 3). `github.com/matthewholman/assist`, arXiv 2303.16246. |
| `GRSS` (Gauss-Radau Small-body Simulator) | Planetary-defense reference (impact monitoring, b-plane, keyholes, close approaches) | open-source | Purpose-built for the Tier-3 impact-probability layer; cross-check geometry/keyhole logic against it. |
| `astropy` | Frames & time cross-check | BSD | Both it and hifitime/ANISE are independently SPICE-validated. |
| `nyx` | Optional full-toolkit oracle | AGPL-3.0 | Offline only. **Never link/ship** unless the whole app goes AGPL. |

### Licensing landmine

- **Only `hifitime`, `ANISE`, `nalgebra` are linked into the shipped binary** — all permissive/MPL, safe.
- **Everything else (`nyx`, `REBOUND`, `ASSIST`, `GRSS`) lives exclusively in the offline `pyref/` fixture pipeline.** GPL/AGPL is fine there because nothing is linked into the distributed Rust and only generated *data* is committed. The one real hazard is `nyx` (AGPL, *Rust*) — easy to accidentally add to a Cargo manifest. Keep it out of every `Cargo.toml`.

---

## 4. Crate / module layout

A Cargo workspace with a clean separation so the physics is testable in complete isolation from the renderer:

```
workspace/
├── core/                  # pure simulation engine — NO renderer dependency
│   ├── state.rs           # StateVector, OrbitalElements, Epoch (hifitime), Body
│   ├── propagator.rs      # Propagator trait + Kepler/analytic impl
│   ├── integrator.rs      # Integrator trait + RK4, DoPri, IAS15-style, symplectic impls
│   ├── forces/            # composable acceleration terms (see §5)
│   │   ├── mod.rs         #   ForceModel = Σ(terms); each term toggleable + unit-tested
│   │   ├── point_mass.rs  #   arbitrary perturber list, positions from any ephemeris (Sun+planets+Moon via DE440/441/ANISE); +16 asteroids at Tier 2 (§5)
│   │   ├── relativity.rs  #   1PN (parameterized post-Newtonian) Sun term
│   │   ├── oblateness.rs  #   Earth/Sun J2
│   │   ├── yarkovsky.rs   #   diurnal + seasonal thermal recoil (transverse A2/r² form)
│   │   └── srp.rs         #   solar radiation pressure
│   ├── geometry.rs        # b-plane, gravitationally-focused capture radius, keyhole geometry
│   ├── lambert.rs         # Lambert solver for intercept design
│   ├── deflection.rs      # kinetic / nuclear-standoff / gravity-tractor Δv models
│   ├── uncertainty.rs     # covariance → b-plane → impact probability (Tier 3)
│   ├── scenario.rs        # scenario definition + (de)serialization
│   └── clock.rs           # fixed-cadence clock; sub-snapshot queries served from integrator dense output (§5), not linear interp
├── viewer/                # MVP pure-Rust renderer (egui spine: egui_plot + painter) — depends on core
├── godot/                 # Phase 2: gdext binding crate — depends on core, owns 3D rendering
├── validation/            # Rust test harness — links core ONLY, loads fixtures
└── pyref/                 # Python scripts (hapsira/REBOUND/ASSIST/GRSS) that generate fixtures
```

Key trait boundaries to define first:

- `Propagator` — given a body + an epoch, return its state. Implementations: analytic Kepler (fast, for context planets) and numerically-integrated (for the asteroid + encounter).
- `Integrator` — a swappable ODE stepper so RK4 / DoPri / IAS15 / symplectic are interchangeable. Encounter accuracy depends on choosing an adaptive high-order stepper here.
- `ForceModel` — a sum of individually-toggleable acceleration `terms`. Tiers (§5) are *which terms are enabled*, not separate code paths. Each term is unit-validated in isolation (§6).
- `Epoch` / time — wrap `hifitime`, never raw f64 seconds for absolute time.

---

## 5. The physics that must be correct

### The core mechanism (this is the thesis, mechanically)

A deflection mostly imparts an **along-track Δv**. That changes the asteroid's semi-major axis → changes its orbital period → the asteroid arrives progressively earlier/later on each subsequent orbit, and that timing error **accumulates** over many orbits. By the predicted impact date, a tiny Δv applied many orbits earlier has grown into a large along-track displacement. Required Δv to achieve a fixed miss falls roughly as **1 / (lead time)**.

> **The curve is not a clean hyperbola.** Superimposed on the 1/t trend is oscillatory structure: the sensitivity of the final miss to an impulse depends on the **true anomaly at the moment of application** (there are sweet spots near perihelion). Don't debug the wiggles as if they were a bug. **Resolved (2nd pass):** the headline curve **fixes the application phase** so it reads as a clean function of lead time (the thesis); the phase dependence (the perihelion sweet-spots) is exposed as a **separate** interactive view — a deliberate sub-lesson, not noise on the main curve.

### Hit-vs-miss is decided by the encounter, not the heliocentric arc

**Two-body Keplerian propagation is fine for drawing orbits but CANNOT decide whether the asteroid hits Earth.** Hit-vs-miss is governed by Earth's (and the Moon's) gravity during the close approach and is acutely sensitive to initial conditions. This is where the entire emotional payload lives — spend the accuracy budget here.

- **Hit criterion = gravitationally-focused capture radius**, not geometric Earth radius:
  `b_impact = R⊕ · √(1 + (v_esc / v_inf)²)`
  Earth's gravity *enlarges its own target* (factor ~1.2–2.4× for typical NEO `v_inf`). This is the correct b-plane impact test **and** a pedagogical gift. The ~100 km atmosphere height is cosmetic next to gravitational focusing.
- **Moon resolved separately during the encounter.** Lumping the Moon into the Earth-Moon barycenter shifts the gravity source by ~Earth-radius scale and corrupts the b-plane. **DE440/441 footgun:** the ephemeris natively provides the Earth-**Moon barycenter** plus a lunar offset — the geocenter is *reconstructed*. Carelessly using the EMB as "Earth's position" displaces Earth by **~4671 km** → Earth-radius-scale b-plane error. Always reconstruct the geocenter and carry the Moon as a separate perturber.
- **Integrate barycentric, not heliocentric (same class of footgun).** Integrate in the **SSB-centered ICRF** frame (matching DE440/441 and ASSIST). A Sun-centered frame is **non-inertial**: it owes an **indirect term** (the negative of the Sun's own acceleration due to the planets), and omitting it is a textbook ~planet-mass-ratio error — the same silent, encounter-corrupting class as the EMB/geocenter mistake. Integrate barycentric; transform to heliocentric only for *display*.

### Realism = a tiered, composable force model

Realism is the goal, but it's switched on in tiers so the MVP stays achievable. Each tier is a set of *enabled acceleration terms* in the composable `ForceModel` (§4) — adding a tier is flipping flags, not rewriting.

**Tier 0 — context orbits (cosmetic).** Two-body Kepler for the background planet visuals. Never used for any hit/miss decision.

**Tier 1 — MVP encounter (honest hit/miss).** The asteroid is integrated as a **test particle in the DE440/441 ephemeris field** (Sun + all planets + Moon as point-mass perturbers, positions and GM from ANISE) with an **adaptive high-order integrator — dop853 for the MVP** (8th-order Dormand-Prince: easier to get right, and its 7th-order dense output also feeds the clock's sub-snapshot interpolation; IAS15 is a Tier-2 long-arc upgrade, not needed for one encounter). Earth as a finite body via the focused capture radius above; Moon carried separately (geocenter reconstructed — see footgun above). b-plane miss geometry. The MVP asteroid is *synthetic* (no Horizons ground truth), but the perturber field is the *real* one — exactly the ASSIST setup with the non-gravitational/relativistic terms switched off. Including all 8 planets is nearly free (ephemeris lookups, not extra integrated bodies); among the giants Jupiter is the principal perturber, but note the along-track drift that drives the thesis comes from the asteroid's *own* Δa (from the Δv), not from any third body.

**Tier 2 — real-asteroid fidelity (to match Horizons).** The perturber field is *already* DE440/441 ephemeris from Tier 1, so this tier is purely **enabling additional force terms** (a config toggle, no structural change):
- **Relativistic 1PN correction** (parameterized post-Newtonian Sun term). JPL includes it; matters for low-perihelion bodies like Apophis.
- **Yarkovsky effect** — diurnal + seasonal thermal recoil; **dominates decade-scale along-track drift** of real asteroids (Bennu is the textbook case). Modeled as a transverse acceleration (A2/r² style); needs spin axis, rotation period, thermal inertia, size, density.
- **Solar radiation pressure** — small bodies and spacecraft.
- **Earth/Sun J2** (oblateness) for very close flybys and keyhole geometry.
- **Major asteroid perturbers** (the 16 ASSIST carries — Ceres/Pallas/Vesta dominate) for long-arc precision. *Planned-for since the MVP:* `point_mass.rs` takes an arbitrary perturber list and ANISE can mount a second kernel (the small-body SPK `sb441-n16.bsp` ASSIST uses alongside DE441), with GMs from ASSIST's constants — so adding these 16 is a config/data change, not a code rewrite.

This tier's term list is deliberately **ASSIST's force model** — adopt it as the spec rather than hand-deriving.

**Tier 3 — uncertainty realism (the most "real" part of planetary defense).** Real defense is probabilistic, not binary. Carry the asteroid's **orbit-determination covariance** (from JPL SBDB), map it through the dynamics to the **b-plane** (linearized via variational equations, or Monte Carlo), and report an **impact *probability*** and risk corridor — not just a miss distance. This reframes deflection success as *"drive impact probability below threshold,"* and is what makes keyholes legible (a keyhole is a tiny b-plane region whose covariance overlap sets up a resonant return). ASSIST's built-in variational equations and GRSS's impact-monitoring logic are the references here.

### Deflection methods (model as a spectrum across lead time)

- **Gravity tractor** — tiny continuous tug, needs *decades* of lead time. (Reinforces the thesis from the gentle end.)
- **Kinetic impactor** — `Δv = β · (m_spacecraft · v_relative) / M_asteroid`, where β is the momentum-enhancement factor from ejecta. DART measured **β ≈ 3.6** at Dimorphos. Expose β as a toggle (1 to ~4). Model the impulse as a **vector** at the real impact geometry; the *along-track component* is what the thesis optimizes (ties to the perihelion sweet-spot note above). **Soft-cap the impulse magnitude** to what's physically plausible for a kinetic impactor — derive Δv from spacecraft mass × relative velocity × β rather than letting the user dial an arbitrary number; when a scenario needs more, surface it honestly (*"this would take N DART-class impactors"*) instead of silently allowing an impossible nudge. Keeps the MVP honest without the full Lambert/delivery layer (§7).
- **Nuclear standoff burst** — model as **energy deposited → surface ablation → momentum → Δv**, using public scaling relations. Largest Δv, for big rocks or short notice. **Model this as deflection physics only — never weapon design.**

### Keyholes

A close pass can thread a small region (a "keyhole") that sets up a resonant *return* impact years later (this is Apophis's real history). Deflecting an asteroid *out of a keyhole* needs far less Δv than deflecting it off a direct collision — a great counterintuitive sub-lesson. Keyholes are properly a Tier-3 (covariance/b-plane) phenomenon.

---

## 6. Validation strategy

### The oracle ladder (synthetic → real)

The common mistake is validating everything against one library. The right oracle depends on the regime, and on the kind of agreement you're after: a **synthetic** asteroid (MVP) has no ground-truth *track*, so you validate the propagator **structurally** — our implementation vs. ASSIST's, same force configuration, agreement = code correctness — whereas a **real** asteroid (Phase 2) is checked against **Horizons as physical ground truth**. Either way the perturber field is the real DE440/441 ephemeris; only the asteroid's own state is invented in the MVP.

1. **Free invariants** (no external oracle) → integrator sanity. *Build first.*
2. **`hapsira` + analytic solution** → Kepler / two-body / element-state conversions, near machine precision over short arcs; Lambert via Vallado canonical cases.
3. **`REBOUND` (IAS15)** → the **integrator + encounter sensitivity** on a *synthetic, self-consistent* N-body you fully control. Use it for the free-invariant cross-checks and for studying how sensitively the b-plane responds to ICs/Δv — *not* as the trajectory oracle, since REBOUND self-gravitates the planets and won't match our ephemeris-perturber propagator over long arcs.
4. **`ASSIST`** → the trajectory oracle **from Tier 1 onward**, because our shipping propagator *is* the ASSIST configuration (test particle in the DE441 field): in Tier 1, run ASSIST with the non-grav/relativistic terms off and compare; in Tier 2, turn the matching terms on on both sides. Its force-term list defines the realism spec, and its variational equations also validate the Tier-3 covariance mapping. Cross-check keyhole/impact-monitoring geometry against **GRSS**.
5. **`astropy`** → frames & time cross-check (independently SPICE-validated, like hifitime/ANISE).
6. **JPL Horizons state vectors** → final ground truth on **real** asteroids (Apophis, Bennu, Didymos). Only meaningful once Tier 2 is on — *real-asteroid arcs will not match Horizons without GR and Yarkovsky.*

### Validate per *term* and per *propagator*, not just the sum

- **Per force term, in isolation.** A summed comparison can mask a sign error in one term. Concrete unit checks: the **GR term alone must reproduce Mercury's 42.98″/century perihelion precession** (closed-form); J2 alone reproduces nodal regression; Yarkovsky alone produces the right secular da/dt sign and magnitude.
- **Per propagator, with the right expectation.** The "free invariants" (below) mean different things for different steppers — don't assert blanket conservation:
  - **analytic Kepler** → conserves everything *by construction*. (So invariant tests on it really only exercise the **element↔state conversions**, not any integrator — don't read green here as validating an integrator.)
  - **symplectic** → energy *bounded/oscillating*, not constant.
  - **RK4 / DoPri** → energy **drifts**; assert the *error-growth rate*, not conservation. (RK4 will correctly *fail* a naive energy-conservation assertion.)

### Element↔state conversions: target the singularities explicitly

The conversions blow up at **e→0** (argument of perihelion undefined) and **i→0** (node undefined). Randomized `proptest` orbits will sail right past these and pass while the real bugs hide. The property tests **must** explicitly include near-circular and near-equatorial cases.

### Free invariants (no external oracle needed) — build first

In pure two-body, **energy, angular momentum, and the Laplace–Runge–Lenz vector are conserved**, and forward-then-backward propagation returns to the start. Wire these as `proptest` property tests over randomized orbits (plus the singular cases above) — with the per-propagator expectations above. They catch most integrator bugs before Python is even involved.

### Make it a harness, not a one-off

1. Define scenarios as data (**JSON** — it crosses the Rust↔Python `pyref/` boundary natively; RON optional later for Rust-only authoring): initial state + reference states at checkpoints.
2. Generate the reference column once with Python (`pyref/`, using the matched oracle from the ladder), commit as fixtures.
3. Rust test suite (`validation/`) loads fixtures and asserts within a **per-regime tolerance**.

### The gotcha that wastes a full day

**Pin μ, AU, frame, and time scale identically on both sides.** Most "my Rust is wrong" panics are actually one side using a Wikipedia μ and the other using JPL's. Pull the same GM and DE values through ANISE on the Rust side — and configure the Python oracle from the same constants — to kill this entire class of phantom failure.

### The gotcha that makes the whole suite lie (read this before trusting a green run)

**`cargo test` without `ASTEROID_DE_KERNEL` + `ASTEROID_PLANETARY_CONSTANTS` set silently skips every kernel-gated test and reports them as passed.** Roughly half this project's physics tests are kernel-gated. They open with `if !have_kernels() { eprintln!("skipping…"); return; }` — deliberate, so a kernel-less CI stays green (kernels are 32 MB–646 MB and are not in the repo). The trap is not the skip; it is that **the skip is invisible**:

- The `eprintln!` notice is **swallowed by cargo's output capture**, which only releases stderr for *failing* tests. A passing skip prints nothing. `--nocapture` shows it; nobody runs `--nocapture` on a green suite.
- What you see is `test result: ok. 13 passed; 0 failed`. That is indistinguishable from a real pass.
- **The runtime is the only tell.** Kernel-less: `13 passed … finished in 0.02s`. Kernels mounted: `13 passed … finished in 69.01s`. Real DE440 integration cannot happen in 20 ms. If a physics suite finishes in under a second, **it did nothing**.

This bit for real on 2026-07-17 and cost the session's whole verification story twice over: a `deflected_b_point_km` fix was "confirmed" by a test that never executed, and `frame_from_arcs_matches_frame_from` — the *only* proof that splitting `frame_from` didn't change its output — had never once run. Both were genuinely green when re-run properly, but that was luck, not verification. Note the shape of the failure: the machine **had** the kernels, sitting in the conventional directory. Only the env vars were unset.

#### Fixed 2026-07-19 — `core::kernels`, and how to run the suite now

The GDScript suites never shared this hazard: `Kernels.resolve()` (`godot/scripts/kernels.gd`) falls back from env → `user://kernels.cfg` → conventional dirs, so `test_orrery` runs real physics either way. That asymmetry was the hint at the fix. `core/src/kernels.rs` is now the Rust mirror of it, and every kernel-gated site in the workspace (core, `validation`, the gdext binding, the examples) goes through it:

```sh
ASTEROID_REQUIRE_KERNELS=1 cargo test --workspace --release   # green here MEANS it ran
```

Two distinct failures needed two distinct fixes, and this is the part worth keeping straight:

- **`kernels::resolve()`** — env → conventional dirs, both-or-nothing — cures *"I have the kernels but didn't point at them"*. That was the actual 2026-07-17 failure. Env vars are no longer needed on a machine that has the kernels in `../temp/AsteroidDefense/kernels` (or `<repo>/kernels`, or beside the exe).
- **`ASTEROID_REQUIRE_KERNELS`** turns "nothing resolved" from a silent skip into a **panic** naming the test that would have lied and every path searched. Resolution alone would have cured only *this* box *today*: a fresh clone, a CI container, or a renamed directory puts the silent-green failure straight back. Unset, the skip is still green — offline CI is preserved on purpose.

**The gate was proved by bypassing it**, not by watching it pass: with `../temp/AsteroidDefense/kernels` renamed away, `ASTEROID_REQUIRE_KERNELS=1` makes the kernel-gated tests **FAIL** loudly, and unset it reproduces the original lie exactly — *the same* `81 passed` / `13 passed`, but `0.09s` and `0.00s` instead of `18.03s` and `56.38s`. The counts are indistinguishable; the clock is the whole signal. That bypass is also what confirmed `tier1_field_matches_assist` genuinely runs in 0.05 s (it fails the moment the kernels vanish) rather than being one more silent skip.

### The gotcha with exactly the same shape, on the frontend (2026-07-27)

**Godot loads `target/debug/`, so a `--release` build leaves the frontend running old physics — with no error and no warning.** `godot/asteroid.gdextension` maps `windows.debug.x86_64` to `res://../target/debug/asteroid_gdext.dll` and `windows.release.x86_64` to the release one. The Godot *editor* and the ordinary `godot` binary are debug builds, so **they load the debug DLL** — while the entire Rust test loop (`cargo test --release`, `cargo build --release`) writes only the release one.

The failure mode is the point: a `#[func]` added to `lib.rs` and confirmed by a green release suite simply does not exist as far as GDScript is concerned. What you get is

```
SCRIPT ERROR: Invalid call. Nonexistent function 'tractor_defaults' in base 'Mission'.
```

which reads like a typo or a binding-registration problem, not like a stale artifact — and the DLL timestamps are the only tell (`target/debug` seven hours older than `target/release`). Same class as the kernel trap above: a green-looking run that is not testing what you think.

**After any Rust change, build both before touching the frontend:**

```sh
cd godot/rust && cargo build && cargo build --release
```

`class_name` is a second, independent staleness: a newly added `class_name` (e.g. `TractorPanel`) is not visible to other scripts until Godot rescans, so `main.gd` fails to parse with *"Could not find type"* while the file is plainly there. `godot --headless --editor --quit --path godot` rebuilds `global_script_class_cache.cfg`. Both of these cost an hour on 2026-07-27 and neither is discoverable from the error text.

---

`user://kernels.cfg` is deliberately *not* read by the Rust side — `user://` resolves through Godot's own per-platform app-data path, and reconstructing that in Rust to read a file the frontend wrote would be a guess that rots silently. The directory scan covers the same case, and callers that know better still pass explicit paths (`MissionCore::load_from`).

---

## 7. Known hard problems (design for these from day one)

- **Scale.** The solar system spans 8+ orders of magnitude; you cannot draw the Sun, planets, an asteroid, and a spacecraft trajectory to scale on one screen. Plan for log-compressed distance toggles, "sizes not to scale" modes, and multiple zoom regimes (whole system → Earth's neighborhood → encounter). The 2D schematic is often the *clearer* teaching tool, not a lesser one.

- **Float precision at solar-system scale — a *rendering* problem only.** At 1 AU (~1.5×10¹¹ m), **f32** spacing is ~16–18 km between representable positions → visible jitter, fatal for Earth-radius miss geometry. But the **core holds true f64 state** (f64 spacing at 1 AU is ~15 µm), so this never touches a result — it only affects how f64 world state is fed to an f32 renderer. For the pure-Rust MVP viewer (egui), work in a **recentered (floating-origin)** frame for the encounter view. In Phase 2 Godot, three complementary approaches cover different views — **decision: floating-origin first, double-precision build only as a fallback**:
  - **(a) Floating origin** *(default)* — each frame, subtract a chosen origin (Earth, during the encounter) before casting f64→f32, so the renderer only sees small numbers near zero where f32 is dense. Cheap, works with **stock Godot**, and covers the one precision-critical view (the encounter).
  - **(b) Double-precision Godot build** *(fallback only)* — compile from source with `precision=double` (Large World Coordinates); gdext must match the double-precision ABI. "Just works" with absolute coordinates but is a heavy, non-standard build to maintain — and the GPU pipeline is still f32, so you often recenter anyway. Use only if (a) proves insufficient.
  - **(c) Non-linear schematic transform** — the whole-system "not to scale" view already log-compresses distances before f32 sees them, so precision is moot there for free.

- **Time spans.** Centuries (orbital sweep) down to hours (encounter), with variable time-warp. Adaptive stepping (below) is what makes this tractable.

- **Numerical accuracy at the encounter.** Adaptive high-order integrator required. **Decision: dop853 is the MVP integrator** — at tight tolerance it is genuinely accurate for one Earth encounter plus a modest orbit count, it's easier to implement correctly than IAS15, and its dense output feeds the clock's interpolation. **IAS15 is a Tier-2 upgrade** for many-revolution long arcs (its near-symplectic edge), not a prerequisite for the MVP. Fixed-step integrators lose accuracy exactly when it matters most; never use one here. Re-confirm the dop853→IAS15 crossover empirically against REBOUND when Tier-2 long arcs arrive.

- **Relativity.** Real NEO trajectories — especially low-perihelion ones (Apophis) — do not match JPL without the 1PN Sun correction. Cheap to add as a force term; **omit it and Horizons validation silently fails.** (Tier 2.)

- **Yarkovsky thermal force.** Over decade scales this **dominates** real-asteroid trajectory uncertainty (Bennu is the textbook case). Long-arc validation against Horizons **will not match without it** — list it here so it's not discovered as a "my Rust is wrong" panic. (Tier 2.) Requires physical/spin parameters per asteroid.

- **Orbit uncertainty is the real domain, not a nicety.** Professional planetary defense reasons in **impact probability** over a covariance, not a single deterministic track. Keyholes only make sense in this frame. Design `uncertainty.rs` (covariance → b-plane → probability) as a first-class Tier-3 deliverable; ASSIST's variational equations and GRSS are the oracles. (Tier 3.)

- **Lambert + porkchop plots.** To make missions that *actually reach* the asteroid, solve Lambert's problem (departure/arrival positions + flight time → connecting orbit + launch Δv). Sweeping launch/arrival dates gives a porkchop plot. This is where the future mission/payload planning layer bolts on naturally — and it's what makes the "tap it once, years out" narrative *honest* (the impulse has to be deliverable within a launch window).

- **The thesis curve's fine structure.** Oscillation on top of 1/t (perihelion sweet spots) — see §5. **Resolved:** fix the application phase for the headline curve; expose phase as a separate view. Don't mistake the structure for a bug.

---

## 8. Phasing / roadmap

### MVP — prove the thesis (pure Rust, honest hit→miss)

- Pure-Rust 2D top-down ecliptic view (**egui**: `egui_plot` for the curve, painter for the orbital view) — **no Godot**
- A few context planets (Tier 0 Kepler) for orientation
- One **synthetic** asteroid on an Earth-collision orbit
- **Tier 1 force model**: asteroid as a test particle in the DE440/441 ephemeris field (Sun + planets + Moon via ANISE), **barycentric ICRF**, **dop853** adaptive integrator, validated against **ASSIST** (non-grav/relativistic terms off) plus REBOUND/IAS15 invariant + encounter-sensitivity checks
- b-plane geometry + **gravitationally-focused capture-radius** hit test
- Fixed-cadence clock with snapshot/interpolation; time slider / play / time-warp
- One method: kinetic impactor, parameterized by Δv (with β factor), impulse as a vector
- Apply Δv at a chosen lead time → re-propagate → **watch the hit become a miss** (Earth slides out of the way)
- **The payoff chart: required Δv vs. lead time** (headline curve fixes the impulse phase; a separate view exposes phase sensitivity)
- Soft-capped, kinetic-impactor-plausible impulse magnitudes; the **delivery** and **determinism** honesty caveats surfaced in UI copy (§1)

That MVP delivers the whole lesson *and* an honest hit→miss flip. Everything below is layering — mostly *toggling on force-model tiers* and swapping the renderer.

### Phase 2 — realism + real asteroids

- **Godot 3D view** (gdext): SubViewport composition (2D schematic/HUD over 3D, or vice versa); floating origin / double-precision as needed (§7)
- **Tier 2 force model**: enable 1PN relativity, Yarkovsky, SRP, J2, and the 16 asteroid perturbers (on top of the DE440/441 ephemeris perturber field already used in the MVP) — validated against **ASSIST**, then **Horizons** on real asteroids
- Real NEOs from the JPL Small-Body Database (§9): Apophis, Bennu, Didymos/Dimorphos
- Nuclear standoff + gravity-tractor methods — **DONE 2026-07-27**: the standoff term as an impulse sibling of the kinetic model, the **gravity tractor** as a windowed `forces/` term with its own duration solve. §5's spectrum is closed. The tractor also has a **frontend** — the `[K]` bench, six live knobs over a cheap model scored against the real field, with an on-demand full-field probe on `[E]`. The nuclear half remains core-only. And since **2026-07-28** the *rock* is dialable too: `[N]` rebuilds the campaign with the threat on a different heliocentric orbit, so the bench compares rather than merely reports — the same 200 t plan scores **0.372× on the shipping orbit and 1.096× on a long-period one**. See *The deflection spectrum, nuclear half*, *…tractor half*, *The tractor on the frontend*, and *The threat orbit became a knob*.
- Lambert / porkchop mission design (makes the impulse *deliverable*, not assumed)
- **Tier 3 uncertainty**: orbit covariance → b-plane → impact probability; keyholes; covariance ellipse shrinking with observations — **first half DONE 2026-07-28**: `core/src/uncertainty.rs` maps a 6×6 state covariance through a measured 2×6 b-plane Jacobian and integrates the result over the focused capture disc, with the linearisation it rests on probed by a deterministic ±3σ shell. The covariance is *invented and labelled as such* (the shipping rock is synthetic and has no observation arc). ~~**Still open:** keyholes and resonant returns, the ξ,ζ pinning they force~~ — **both DONE 2026-09-02**, see *Keyholes, closed*: the frame is pinned, the circles are closed-form, and the 3:4 keyhole is flown to a return impact. ~~Still open: real SBDB covariance ingestion~~ — **DONE 2026-09-06**, see *Real SBDB covariances*. Still open: the ellipse on the frontend. See *Tier 3 begins* and *Keyholes, closed*.

### Phase 3 (future)

- Plausible launch vehicles + payload mass budgets
- Orbital assembly (assemble-in-orbit when payload too big for one launch)
- Standing/ready Earth-defense systems
- Multi-mission campaigns

---

## 9. Data sources & teaching asteroids

- **JPL Horizons** — state vectors; the ground-truth reference for *real* trajectories (Phase 2 / Tier 2 onward).
- **JPL Small-Body Database** — orbital elements **and covariances** for real NEOs; the covariance feeds Tier 3.
- **JPL DE440 / DE441** (via ANISE) — planetary/lunar ephemerides. DE440 = standard span; DE441 = long span (what ASSIST uses). GM constants pulled from ANISE even in the MVP.
- **ESA NEOCC** — secondary cross-reference (and the Aegis impact-monitoring system as a Tier-3 reference).

Teaching asteroids worth seeding (Phase 2):

- **Apophis** — the perfect teaching case: famous 2029 close approach and real keyhole history; also exercises relativity (low perihelion).
- **Didymos / Dimorphos** — the DART target; gives a real, measured β for free.
- **Bennu** — well-characterized (OSIRIS-REx); the canonical Yarkovsky case.

---

## 10. First tasks for Claude Code

Re-sequenced for the pure-Rust / honest-hit-miss MVP. The encounter (ephemeris test-particle + ASSIST validation) is now **on the MVP critical path**, not a late add — which is why the task-0.5 build spike (step 2) comes first.

1. **Scaffold the Cargo workspace:** `core/` (no renderer dep), `viewer/` (pure-Rust, **egui**), `validation/`, `pyref/`. *(No `godot/` yet — Phase 2.)* Wire **ANISE + a DE440 (or DE441) kernel** loading early — the test-particle MVP needs perturber positions, not just GM constants, before first-light.
2. **Task-0.5 de-risk spike — do this before the rest of the plan leans on it.** Confirm the two pillars Option A rests on: (a) **ASSIST + DE441 actually build** offline in `pyref/` and can integrate a test particle; (b) the **ANISE DE-position reader** returns a sane reconstructed **geocenter** (not the EMB) for a known epoch. **Fallback-to-Option-B trigger:** if ASSIST won't build or the DE-position reader stalls, fall back to a self-consistent N-body MVP validated against REBOUND and revisit the ephemeris-perturber architecture at Tier 2. (Under Option A you may *demo* the hit→miss flip before ASSIST validation completes — but REBOUND cannot stand in as the trajectory oracle, since it self-gravitates the planets.)
3. **Implement `Epoch` (hifitime), `StateVector`, `OrbitalElements`, and element↔state conversions** — with `proptest` coverage that **explicitly targets e→0 and i→0** singularities (random orbits miss them).
4. **Implement the analytic Kepler propagator** behind the `Propagator` trait.
5. **Wire the free-invariant property tests** (energy / angular momentum / LRL / forward-back reversibility) **with per-propagator expectations** (analytic → machine precision; later RK4 → error-growth rate, not conservation). At this step they validate the *conversions*, nothing more — don't over-read green.
6. **Stand up one `pyref/` fixture** (propagate a known orbit via hapsira, commit reference states as JSON) and the matching Rust test in `validation/`. Pin μ/frame/time-scale identically; pull GM through ANISE on the Rust side.
7. **Build the composable `ForceModel`** (Σ of toggleable terms; `point_mass.rs` takes an arbitrary perturber list) and the integrators behind the `Integrator` trait: **RK4 first** (to exercise the invariant tests), **then dop853 as the MVP encounter integrator** (IAS15-style is a Tier-2 long-arc upgrade). Integrate in the **barycentric ICRF** frame. Then the **Tier-1 force model** — asteroid as a test particle under Sun + planets + Moon point masses, positions from DE440/441 via ANISE — **validated against ASSIST** (non-grav/relativistic terms off on both sides), with **REBOUND/IAS15** used for the free-invariant and encounter-sensitivity cross-checks. Unit-validate each GR/J2/Yarkovsky term in isolation (Mercury precession, etc.) as it's added.
8. **Implement b-plane geometry + the gravitationally-focused capture-radius hit test** — turns the encounter into a hit/miss answer and underpins the Δv-vs-lead-time curve.
9. **Build the fixed-cadence `clock`** with a snapshot API whose sub-snapshot queries are served from the integrator's **dense output** (dop853's continuous extension), not linear interpolation — linear interp visibly lies through the high-curvature encounter.
10. **`viewer/` (egui):** the Δv-vs-lead-time chart (`egui_plot`; fixed-phase headline curve + a separate phase-sensitivity view) **and** the rewind → nudge → re-propagate → "Earth slides out of the way" animation (painter), rendered in a floating-origin frame for the encounter.

At that point the engine supports the full MVP scenario. Tier-2 realism and Tier-3 uncertainty then layer on as force-model toggles + the Godot frontend (Phase 2), largely in parallel.

---

## Open questions / deferred decisions

The first review and the follow-up discussion closed every major open question (see *Resolved* below). What remains is genuinely deferred to when the relevant tier arrives:

- ~~**dop853 → IAS15 crossover (Tier 2).**~~ **Answered 2026-09-06 and the item is retired.** Measured on the 12-year campaign and the 15-year keyhole return, not against REBOUND — which §6 itself rules out as a trajectory oracle because it self-gravitates the planets. dop853 is converged at the shipping tolerance on both; what is *not* converged is the tolerance without the snapshot cadence's step cap (129 km over the cruise). The crossover question turned out to be about how the integrator is driven, not about the method. See *The integrator, measured instead of replaced*.
- **Impulse soft-cap: hard gate vs. honest readout.** Whether the MVP forbids an over-budget nudge outright or allows it with an honest *"this would take N DART-class impactors"* label — a UX call to settle in implementation (§5).
- **SBDB covariance ingestion (Tier 3).** The on-disk format/units for real-asteroid orbit-determination covariances feeding `uncertainty.rs` — deferred until Tier 3.
- ~~**b-vector sign convention + ξ,ζ decomposition (raised by step-8 b-plane geometry).**~~ **CLOSED 2026-09-02** — `B` points at the incoming asymptote (derived from the hyperbola's centre, measured 489× on a flown flyby), `ζ̂` opposes Earth's motion, `(ξ, η, ζ)` right-handed; `core/src/keyhole.rs`, see *Keyholes, closed*. The original text follows for the record. `geometry.rs` ships the b-plane hit test and the b-vector `B` with its *magnitude* pinned (`|B| = b`) and its plane pinned (`B ⊥ Ŝ`, `B ⊥ ĥ`), but its **sign** deliberately unasserted, and the Öpik/Kizner **ξ,ζ decomposition** — which needs an external reference direction (Earth's heliocentric velocity, or an ecliptic pole) — deferred to Tier 3 (`uncertainty.rs`), since that is the layer (keyholes/covariance) that actually reasons in b-plane coordinates. Nail the sign + reference frame when keyhole geometry needs it. **Phase-2 3C-2c coexists with this rather than forcing it:** the Godot b-plane view builds its *display* axes from `Ŝ` and the ecliptic pole in the binding (not core), labels them as display axes, and prints only rotation-invariant scalars (`|B|`, perigee, capture radius, `v_inf`) — so nothing on screen depends on the unpinned convention, and settling it later is still free.
- ~~**Pluto in the shipping perturber field (raised by batch-2c ASSIST validation).**~~ **CLOSED 2026-07-27 — measured at 0.6 m, shipping field stays at ten bodies.** Both halves of the blocker resolved: the missing GM was real (`pck11.pca` genuinely resolves no Pluto GM — probed, not assumed) and the DE440 header supplies one (`GM9` → 975.500 km³/s²); and the *cost* is now measured rather than extrapolated. §5's own criterion was "flip to 11 if the growing-with-lead-time cost proves to matter"; at the campaign's real ~12 yr lead Pluto moves the b-plane perigee by **0.0006 km**, two orders below the belt's sub-km floor. The batch-2c ~55 m position figure did grow, but not into anything the b-plane resolves. Pluto ships as a `Tier2Config` toggle (off by default) so the comparison stays reproducible. See *The deferred leftovers, closed*.

### Tier 2 begun — 2026-07-20 session (1PN relativity + Yarkovsky terms)

- **1PN relativity Sun term shipped and validated in isolation** (`core/src/forces/relativity.rs`). The first Tier-2 force: the PPN Schwarzschild acceleration of a test particle in the Sun's field at `β = γ = 1`, `a = μ/(c²r³)·[(4μ/r − v²)r + 4(r·v)v]` with `r, v` heliocentric. Fits the composable [`ForceModel`] sum with **zero structural change** — it is one more `.with(...)` term (§5). `c = 299 792 458` m/s exact; `μ` is a field passed in (the tests use the DE `1.327 124 400 18e20`, production must hand it the **same** ANISE-loaded `μ_sun` the point-mass Sun term uses — a second hardcoded μ would be a silent bias). Needs the Sun's full **state** (position + velocity), so it gets its own `CentralBodyState` provider rather than a `velocity_at` bolt-on to `PerturberEphemeris` (position-only); `FixedCentralBody::at_rest_origin()` keeps the isolation test kernel-free.
- **Validated by Mercury's perihelion precession, the §6 isolation check** — the term alone reproduces `Δϖ = 6πμ/(c²a(1−e²))`/orbit. Guards the advisor flagged as load-bearing, all built in from the first run: (1) the signal is compared to the closed form computed with the **same** constants, not to a literal 42.98″; (2) a **Newtonian-only control run** (1PN off) confirms the measured precession is physics, not integrator LRL drift — control ≪ signal; (3) measured by **stroboscopic** eccentricity-vector sampling (once per period) + a least-squares slope over 40 orbits, not one-orbit differencing; (4) an explicit **prograde sign** assertion (the classic `(r·v)v` sign bug's tell). Signal matches the closed form to <2% and lands in 40–46″/century; kernel-free so it actually runs (unlike a silently-skipped ANISE test). Full core suite 97 passed / 0 failed in **18.31 s** with `ASTEROID_REQUIRE_KERNELS=1` (the runtime that proves the kernel-gated half executed).
- **Yarkovsky thermal-recoil term shipped and validated in isolation** (`core/src/forces/yarkovsky.rs`). The decade-scale along-track *dominator* (§272) and the term that actually earns real-NEO Horizons validation — J2 of the Sun is negligible heliocentrically, so Yarkovsky came before it. Uses JPL Sentry's **transverse `A2` parametrization** (Farnocchia/Vokrouhlický), not a full thermophysical model: `a = A2·(r₀/r)^d·t̂` with `t̂ = ĥ×r̂` the prograde in-plane direction, `r₀ = 1 AU`, `d = 2`. `A2` carries the drift sign (`A2>0` prograde → outward `da/dt`; `A2<0` retrograde, Bennu-like → inward). Reuses the 1PN commit's `CentralBodyState` provider (heliocentric `r, v`); another `.with(...)` term, zero structural change.
- **Validated by the secular semi-major-axis drift, `⟨da/dt⟩ = 2·A2·r₀²/(n·a²(1−e²))` (d=2), the §6 isolation check.** The advisor's make-or-break was the oracle's **time weighting**: the Gauss `da/dt` integrand goes as `(1+e·cosν)³`, so a uniform-in-true-anomaly average is ~10% wrong at e≈0.2. Fixed by sampling the oracle uniformly in **mean anomaly** (= uniform in time), and cross-checked two ways — the numerical uniform-M average agrees with the closed form to <1e-4 across e=0/0.2/0.45 (`oracle_time_average_matches_the_closed_form`), and the integration-measured drift matches the **time-averaged** oracle to <1% at e=0.2 (a uniform-ν oracle would be ~10% off and fail that tolerance — the test discriminates). Same guard structure as 1PN: a circular-orbit de-risk case (e=0, no weighting ambiguity), an `A2=0` **control run** (drift ≪ signal → physics not integrator noise), an explicit prograde/retrograde **sign** pair, and an algebraic acceleration test pinning `a·r̂=0`, `|a|=A2(r₀/r)²`, and direction `ĥ×r̂` **not** `v̂` (the common wrong impl). `A2` amplified above Bennu's physical ~1e-13 m/s² for SNR (legitimate — validates form/sign/units, not magnitude — and stays linear, Δa ≪ a). Bennu numeric anchor deliberately **dropped** rather than recalled from memory (the algebraic test already guards units). Kernel-free; full core suite 104 passed / 0 in **18.57 s** under `ASTEROID_REQUIRE_KERNELS=1`.
- **Both terms now WIRED into the shipping scenario behind toggles** (`core/src/scenario.rs`). `RealFieldScenario.force` is a [`CompositeForce`], not a bare `PointMassGravity`, built by the single `compose_force(eph, &Tier2Config)` helper both `build_with` and the new measurement path share — so "GR on"/"Yarkovsky on" cannot mean two different things. `Tier2Config { relativity: bool, yarkovsky_a2: Option<f64> }` hangs off `ImpactorConfig`, **all-off by `Default`** (every downstream builder passes `ImpactorConfig::default()`, so the shipping demo is untouched). The Sun's heliocentric `r,v` for both terms comes from an ephemeris-backed `CentralBodyState` impl on `EphemerisPerturber` (mirrors the existing `GeocentricState` impl — GR/Yarkovsky and the encounter geometry read *one* Sun), and 1PN's `μ_sun` is the same `eph.gm_km3_s2(SUN_J2000)·1e9` the point-mass Sun uses (never a second hardcoded constant).
- **Verification = the fixed-seed b-plane comparison (advisor-gated), not a rebuild.** Rebuilding with terms on would back-propagate the seed through the terms-on field and reproduce the hit *by construction* → zero visible shift. So `RealFieldScenario::nominal_encounter_with(&Tier2Config)` holds the built seed fixed and re-flies it through a differently-toggled field, attributing the perigee move to the physics. `tier2_terms_leave_the_bplane_unchanged_off_and_shift_it_on` asserts **structure, never a hand-derived magnitude**: (a) all-off re-fly == the shipping perigee *bit-for-bit* (the composite-with-one-term is `0 + a_pointmass`); (b) 1PN shifts the perigee by a resolvable amount and stays a hit; (c) a **physical, un-amplified** `A2 = 1e-13 m/s²` shifts it by some nonzero finite amount. **Measured:** 1PN moves perigee **3000.0 → 2944.5 km (−55.6 km)**, still well inside the 11 311 km capture (keyhole-precision territory, the reason GR matters for planetary defence); Yarkovsky at the physical A2 moves it **5.1 km** over the ~12 yr campaign — small but real, reported honestly rather than amplified into a lie. Full core suite **105 passed / 0** in 38.98 s under `ASTEROID_REQUIRE_KERNELS=1`; the gdext binding's 16 kernel-gated tests still read cap 11 311 km / |B| 14 639 km to the digit (the all-off bit-identity, confirmed downstream).
- **Open / next:** *(all resolved — the shipping demo still defaults `tier2` off, but the live frontend toggle, SRP, the 16 `sb441` perturbers, and the Horizons capstone all landed by 2026-07-21; see **Tier 2 complete** below.)* ~~Remaining: J2 and Pluto-in-shipping.~~ **Both closed 2026-07-27** — see *The deferred leftovers, closed*.

### Tier 2 continued — 2026-07-20 session (16 sb441 asteroid perturbers enrolled as forces)

- **The 16 `sb441` main-belt bodies promoted from scenery to force perturbers** (`core/src/perturber_field.rs`). `sb441_perturber_field(&Arc<Ephemeris>)` mirrors `tier1_perturber_field`: one `PointMassGravity` over 16 `EphemerisPerturber`s reading positions for NAIF ids `2000000+number` from a **mounted `sb441-n16.bsp`**. A third `.with(...)` term on the same [`CompositeForce`] sum, zero structural change (§5) — the exact expansion `point_mass.rs` was designed for since the MVP.
- **The masses are the load-bearing half; provenance is verbatim + machine-verified, not recalled.** `sb441-n16.bsp` carries **positions only** — ASSIST joins the GMs from the DE440/441 planetary file's own `MA%04d` constants (keyed by asteroid number), so each mass is the one JPL *integrated that position with*; any other value flies a perturber whose gravity disagrees with the trajectory it traces. The 16 GMs are transcribed **verbatim from the DE440 header GROUP 1041** (`MA0001…MA0704`, au³/day², D→e) into `SB441_PERTURBER_GM_AU3_DAY2`, and were **re-read straight out of the local `linux_p1550p2650.440` binary's constant record** (CVAL array at record-2 offset 8144, AU@CVAL[10]/DENUM=440 pinning the layout) to confirm the on-disk kernel carries these exact doubles. `#[allow(clippy::excessive_precision)]` keeps the header digits, same as `DE440_EMRAT`.
- **The unit/transcription guard, and the wrinkle that made it sharper.** GMs are **not** pulled from ANISE: the shipped `pck11.pca` resolves only **6 of 16**, and to a *different, later* mass solution. Cross-checking those 6 against the hardcoded DE440 set: the three **best-determined** (Ceres, Pallas, Vesta) agree to **<1%** (Vesta to ~4 sig figs — the au³/day²→km³/s² factor is right, since a wrong factor misses by orders of magnitude); the other three (Psyche, Europa, Davida) legitimately differ by **12–72%** because DE440 *free-fit* them where pck11 has spacecraft/occultation values — which is exactly why the self-consistent DE440 set is hardcoded rather than resolved. So the test (`sb441_field_builds_and_well_determined_gms_match_pck11`) asserts <1% on the three shared determinations only, and documents why the loosely-determined bodies are *not* checked against pck11.
- **Wired behind a toggle, all-off by `Default`, fail-loud on the missing kernel.** `Tier2Config` gained `asteroid_perturbers: bool`; `compose_force` adds the belt term when set. `sb441-n16.bsp` is the **optional 646 MB kernel** (outside the both-or-nothing rule), so: `RealFieldScenario::build` mounts it when the flag is set and errors if it is absent; `build_with` requires the caller to have chained it on; and `sb441_perturber_field` **probes every body's position up front** and returns a clear error naming the missing small-body kernel rather than failing deep in the first integration step ("an incomplete field is a wrong field", applied to positions). `sb441_field_without_the_small_body_kernel_fails_loud` pins it.
- **Verification = the same fixed-seed b-plane measurement as GR/Yarkovsky, and the capstone.** `asteroid_perturbers_leave_the_bplane_unchanged_off_and_shift_it_on` builds a **Tier-1 seed** on an sb441-mounted almanac, then re-flies it with the belt on: off == baseline bit-for-bit; on shifts the perigee by a nonzero finite **measured 0.552 km** over the ~12 yr campaign (3000.0 → 2999.5 km, still well inside the 11 311 km capture) — sub-km, the residual *floor*, reported honestly not amplified. The capstone (`capstone_neo_vs_horizons.rs`) gained a **+belt column** on the Apophis-vs-Horizons residual: the belt perturbs the trajectory at every epoch (**+0.07…+0.43 km** through year 7) but at the 8-year arc end sits **within the unmodelled radial-A1 floor** (Δ −0.037 km against the ~18.6 km GR+Yk residual) — it does **not** clear that floor, exactly as the sub-km wiring result predicts. The capstone asserts only that the perturbers *act* (nonzero finite, bounded), never that they help — measure-and-report, same discipline as Yarkovsky's below-floor early years.
- **Two 16-body lists, pinned together.** `gdext`'s display-scenery `SB441_BODIES` (id, name) and core's canonical force table `SB441_PERTURBER_GM_AU3_DAY2` are two spellings of the same sixteen; `scenery_and_force_perturber_lists_agree` (kernel-free, gdext) fails at `cargo test` if either drifts. `core/examples/probe_sb441.rs` kept as the provenance sibling of `probe_perturbers`/`probe_sun_gm` (it is the probe that measured 16/16 positions vs 6/16 GMs resolve).
- **Cost, as flagged.** The 16 extra ANISE lookups per RK step make the belt-on nominal propagation the heaviest test in the suite (~110 s for the two-propagation b-plane measurement); gated and default-off, so the shipping demo is untouched. Full core suite **109 lib + 1 capstone (24 s, ran not skipped) + 12 roundtrip, 0 failed** under `ASTEROID_REQUIRE_KERNELS=1`; core clippy clean; gdext drift guard green.
- **Open / next:** ~~a frontend toggle to show the belt shift live~~ and ~~SRP~~ both landed the following session — see *Tier 2 complete* below. ~~Remaining: J2 and Pluto-in-shipping.~~ **Both closed 2026-07-27** (`J2` shifts the perigee 1.33 km — more than this whole belt; Pluto 0.6 m). The residual floor is now GR-of-the-planets + JPL's radial A1, which we do not model.

### Tier 2 complete — 2026-07-21 session (SRP + the Apophis capstone + the live force-model menu)

Three commits close out the Tier-2 force menu the §5 spec asked for; the tree is clean and pushed.

- **Solar radiation pressure shipped and validated in isolation** (`core/src/forces/srp.rs`, commit `80498f7`). The *radial* sibling of Yarkovsky's transverse recoil: `a = a₁·(r₀/r)²·r̂` with `r₀ = 1 AU`, pushing directly away from the Sun. Constructed from physical inputs — `SolarRadiationPressure::from_physical(Cr, A/m)` — rather than a bare coefficient, so the term reads in the same units a real body's data comes in. Reuses the 1PN/Yarkovsky `CentralBodyState` provider for heliocentric `r`; one more `.with(...)` term on the same [`CompositeForce`], zero structural change (§5). **Validated by the effective-μ identity** — a pure `(1/r²)` radial push away from the Sun is indistinguishable from *weakening the Sun's gravity*, so the isolation test asserts a body under Sun-gravity + SRP orbits exactly as a body under a reduced `μ_eff = μ_sun·(1 − β)` — an algebraic invariant, not a hand-tuned number. Kernel-free.
- **The Apophis capstone: our *own* integration vs JPL Horizons** (`core/tests/capstone_neo_vs_horizons.rs`, commits `15a1a09` + `b91607e`). The payoff the whole force model was built to earn (§6 real-asteroid rung): integrate Apophis in our field and diff against its Horizons `.neo` truth table, **per force term**, GR measured not asserted. Results, honest: **1PN relativity cuts the residual 5–175×** across the arc (the low-perihelion body §167 predicted would need it); **Yarkovsky at Apophis's real `A2 = −2.902e-14 au/d²` roughly halves the year-8 residual** once the signal clears the model floor; the **belt** perturbs +0.07…+0.43 km through years 1–7 but at year 8 sits *within* the unmodelled radial-A1 floor (Δ −0.037 km) — it does not clear it, exactly as the sub-km wiring result predicted. The capstone asserts direction and bound, never a hand-derived magnitude, and **fails loud** (not skip-green) when the Apophis tables are absent — a tables-but-no-Apophis run must error, per `b91607e`.
- **The live force-model menu — see the shift on screen** (`godot/`, commits `1dc0646` + `7397869`). The frontend `[P]` menu toggles each Tier-2 term (`[G]` GR / `[Y]` Yarkovsky / `[A]` asteroid belt / `[S]` SRP) and re-solves the b-plane **on demand**, reporting each term's perigee shift live. Measured, all **inward**: GR **+55.55 km**, Yarkovsky **+5.10 km**, belt **+0.55 km**, SRP **+8.36 km**. On-demand, *not* on scenario build — chaining the terms into the build path was measured at >200 s and blocked the threat solution, so it pivoted to an Arc-shared scenario (gated behind a `RealFieldScenario: Sync` bound) driven over a second mpsc worker channel. Verified two ways: the `_shot.gd` harness drives the real keys and screenshots the shifted perigee, and an FFI gate pins the per-term deltas.
- **Where the force model stands.** The Tier-2 term list (§166) was complete but for **J2** and **Pluto-in-shipping**, both of which **landed 2026-07-27** — so the list is now closed: `J2` is validated against the closed-form nodal regression and shifts the perigee 1.33 km, and Pluto ships as a toggle measured at 0.6 m (the shipping field deliberately stays at ten bodies). Everything ASSIST carries, we carry, validated per-term against the closed form or Horizons. **The deflection-method spectrum (§5) closed on 2026-07-27 with the nuclear and tractor halves, so the one remaining spec beat is Tier 3 uncertainty (§175 — covariance → b-plane → impact probability, keyholes, and where the deferred b-vector sign/ξζ convention at the open-questions list finally gets settled).**

### Phase-2 mission design — 2026-07-21 session (Lambert + porkchop + launch vehicles, core)

The §8 "makes the impulse *deliverable*, not assumed" beat — the honesty gap §7/§180 keeps flagging. **User chose the fuller build on both open axes** (over the advisor's minimal-cut recommendation): couple the impulse *direction* to the real arrival geometry (not the idealized along-track push), and include real **launch vehicles** (bounded to single-launch / no orbital assembly — that stays Phase 3). The core layer is three kernel-free-where-possible modules; the Godot porkchop heatmap view is the remaining follow-up.

- **`core/src/lambert.rs` — the two-point transfer solver** (commit `ad00379`). Universal-variable (Bate/Mueller/White; Curtis Algorithm 5.2), single-rev short-way prograde first cut. Given `r1`, `r2`, `Δt`, `μ` → departure/arrival velocities. Two-body, and that is *correct* for the planning layer (a real cruise is two-body); it is **not** a display-grade shortcut — the honest-hit/miss physics stays in the full field, Lambert only sizes/aims the delivery. The **180° collinear singularity returns `DegenerateGeometry`** (a porkchop gap, never a NaN that would poison the heatmap — the same discipline the b-plane 180° case follows). `μ` is caller-supplied (no second hardcoded `μ_sun`). **Validation ladder, all kernel-free:** round-trip vs the analytic `KeplerPropagator` across a spread of orbits/arcs (the advisor's "cheapest and strongest", validates against a propagator already at machine precision); an **independent published worked example** — poliastro's Izzo-algorithm docs, a *different* algorithm, **fetched not recalled**, agreement ~0.02 m/s (floored by the page's digit rounding); the free **energy + angular-momentum invariants** of the transfer conic across many geometries; and an arrival-state-reaches-`r2` forward check. 7/7.
- **`core/src/launch_vehicle.rs` — real C3→payload delivery curves** (commit `00c203a`). The deliverability half: given a departure `C3` (km²/s²), how much mass a real rocket lifts to it. **Provenance was the hard gate the advisor flagged** — plausible launch numbers are the recallable-but-wrong trap, and unlike the sb441 GMs there is no kernel to machine-verify against — so every knot is **fetched and cited**: transcribed from AMAT's `launcher-data/*.csv` (`github.com/athulpg007/AMAT`, machine-fetched via `gh api`), which are compiled from the **NASA LSP Performance website** via Girija arXiv:2310.05994. Five vehicles spanning the capability range (Atlas V 551, Falcon Heavy reusable/expendable, Vulcan Centaur, Delta IV Heavy), linear-interpolated with **0-outside-range = infeasible** (mirrors AMAT's `interp1d(fill_value=0)`). Two labelled caveats: knots downsampled to ~10 km²/s² (<1% vs the full table), and delivered mass modelled *as* impactor mass (Phase-3 refinement). 5/5.
- **`core/src/mission.rs` — the porkchop + on-demand verify** (commit `003565d`). The composition, split by cost exactly like the live force menu (cheap-always-on / expensive-on-demand), because coupling direction means a real deflection check needs a **full-field re-propagation per launch window** — `O(N²)` over a grid is hours. So: **the cheap grid** (`porkchop_grid`) is pure scalar Lambert over Earth/asteroid state arrays looked up *once per epoch* (not `N×M` ephemeris queries), recording `C3`, arrival `|v_rel|`, and the **along-track projection** of the impact — a free first-order *effectiveness* proxy (`v_rel·v̂_ast`) that surfaces the whole point of coupling direction: **deliverable ≠ well-aimed** (a window can carry plenty of `|Δv|` yet project poorly onto the track and barely deflect). The grid is **vehicle-independent**; `C3`→mass maps per launcher afterwards (`cell_delivery`), so switching vehicles never re-solves Lambert. **The on-demand verify** (`verify_cell`) re-propagates one selected cell in the full `n`-body field after the real *vector* impulse `β·(m_sc/M)·v_rel_vec` (via the existing `DeflectionScenario::evaluate`, which already takes a `Vector3` — zero new deflection path), reading the exact b-plane perigee. `required_impactor_mass` bisects the mass to a target perigee with the advisor's **degenerate-direction guard from day one**: a mass cap turns the `v_rel ⊥ v̂_ast` case (no deliverable mass deflects) into an honest `InfeasibleAtCap`, never a runaway bisection. Endpoints are real (Earth ephemeris, asteroid nominal *pre-deflection* trajectory); outputs labelled patched-conic planning estimates. **8/8 tests, made discriminating after an advisor review** (the first cut verified wiring, not behavior — `perigee >= 0` would pass even with the impulse un-applied): the solver is validated **kernel-free** (a `ZeroForce` straight-line pass, like `deflection.rs` tests itself — the solved mass *actually* delivers its target perigee = the mis-bracket catch, monotone in target, `InfeasibleAtCap` at a low cap), and a **cheap kernel-gated test (~2 props)** proves the real-field composition: zero impactor mass reproduces the nominal *hit* (catches wrong epoch/frame), a delivered mass flips it to a *miss* through the coupled pipeline. Splitting algorithm-from-composition dropped the suite from 407 s to 26 s. Core clippy clean.
- **Open / next:** ~~the **Godot porkchop heatmap view** (frontend) — the visualization the user is owed~~ — **landed 2026-07-27** (the `[4]` launch-window map; see *The Godot launch-window map* below). The physics/deliverability was all in core and tested; what the view added was the discovery that `payload_kg` zeroed *cheap* departures as well as unreachable ones. Then the follow-on axes: the direction-coupling makes the headline curve's along-track idealization one lens among several (a phase-sensitivity story), and Lambert now *delivers* the kinetic impactor that the §5 deflection-method spectrum (gravity tractor / nuclear) would choose between. ~~Multi-rev / long-way Lambert and exact NASA-LSP polynomial coefficients are drop-in refinements if ever wanted.~~ **Both landed 2026-07-27, and neither was a mere refinement:** multi-rev exposed that `lambert_universal` was silently returning lapping transfers labelled direct, and the full LSP tables replaced a downsample whose error had been documented as "well under 1%" and measured at 8.9%. See *The deferred leftovers, closed*.

### The deferred leftovers, closed — 2026-07-27 session (J2, Pluto, multi-rev Lambert, the full LSP tables, and the build-time item)

Five items that had been parked as "low priority", "blocked", or "drop-in if ever wanted". Working them turned up three things that were *not* known: a wrong doc claim, a wrong shipped number, and a genuine bug in a shipped solver. That is the argument for clearing a leftovers list rather than letting it age.

- **Earth's `J2` shipped and validated in isolation** (`core/src/forces/oblateness.rs`). The last Tier-2 term (§166). Deferred all through Tier 2 as "negligible heliocentrically", which is *true* — `J2` falls off as `1/r⁴` — and is exactly why it had to be **measured at the encounter** rather than argued about: essentially all of its effect is bought in the minutes the asteroid spends inside a few Earth radii. Validated by the closed-form **nodal regression** `dΩ/dt = −(3/2)·n·J2·(R_eq/p)²·cos i` to <2%, with the guard structure the 1PN/Yarkovsky terms established: a **`J2 = 0` control run** (drift ≪ signal → physics, not integrator noise), an explicit **retrograde sign pair** (`cos i < 0` must make the node *advance*), and three algebraic pins — inward over the equator, **outward over the pole at twice the magnitude**, and purely axial at the magic latitude `sin φ = 1/√5`. That last one caught a sloppy claim in my own module doc: at the magic latitude it is the bracket's `r̂` *coefficient* that vanishes, **not** `a·r̂` (`k̂` is not perpendicular to `r̂` there), so the test now pins `a × k̂ = 0` instead. Kernel-free; 8/8.
- **The spin axis is a parameter, not `ẑ`.** `J2` is defined about the body's rotation axis, and for Earth in ICRF that is *near* `ẑ` but not equal to it. Rather than assume, the term takes a [`BodyPole`] provider; the shipping wiring reads the pole ANISE rotates out of the loaded planetary constants (`Ephemeris::pole_unit_icrf`, the DCM's **third row** — `v_body = R·v_icrf`, so the body `ẑ` back in ICRF is `Rᵀẑ`). Measured by probe: exactly `ẑ` at J2000, **0.2228° off at 2040**, 0.5570° at 2100 — matching the IAU 0.557°/century model to four digits, which independently confirms the row extraction. `FixedPole` keeps the isolation tests kernel-free.
- **`J2` and `R_eq` travel as a pair, from the DE440 header.** The physics contains `J2·R_eq²`, so a `J2` from one solution used with an `R_eq` from another is a silent scale error. Both are read verbatim out of the local `linux_p1550p2650.440` constant record (`J2E = 0.00108262539`, `RE = 6378.1366` km) — the same machine-verified path the sb441 masses took. This makes `EARTH_EQUATORIAL_RADIUS_M_DE440` (6 378 136.6 m) deliberately **distinct** from the WGS-84 `geometry::EARTH_EQUATORIAL_RADIUS_M` (6 378 137.0 m): different roles, 0.4 m apart, not interchangeable.
- **Pluto: the blocker was real, and the answer is that it does not matter.** The open-questions entry parked Pluto on a missing GM — correctly: `pck11.pca` resolves **no** Pluto GM (`ID 9 not in look up table`, verified by probe, not assumed). The DE440 header has one, `GM9 = 2.175096464893358e-12` au³/day² → **975.500 km³/s²**, the Pluto+Charon *system* value as it must be for NAIF 9. Wired behind a toggle and measured on the fixed seed: over the ~12 yr campaign Pluto moves the b-plane perigee by **0.6 metres**. The §5 criterion was "flip to 11-in-shipping if the growing-with-lead-time cost proves to matter"; at the real lead time it is two orders below the belt's already-sub-km floor, so **the shipping field stays at ten bodies** — now on a measurement rather than the batch-2c extrapolation that guessed "plausibly ~km". That 0.6 m reads as signal and not integrator noise only because the terms-**off** re-fly reproduces the shipping perigee **bit-for-bit**; without that identity the number would be meaningless.
- **Measured Tier-2 perigee shifts, all terms, one campaign:** GR 55.6 km · SRP 8.36 km · **`J2` 1.33 km** · belt 0.55 km · Pluto 0.0006 km. `J2` is larger than the entire 16-body main belt. (All five are measured on the nominal *impact*; `J2`'s in-domain figure on a deflected miss is **0.12 km outward** — see *The `J2` pair* below.)
- **The `J2` number grazes a validity boundary, and measuring caught it.** The `J2` expansion is only valid *outside* `R_eq`, and this scenario's nominal is a designed **impact** — closest approach 3000 km, well inside Earth. An earlier draft of the module note called that harmless because "nothing downstream reads the sub-surface arc". Wrong: the b-plane reduction samples the state *at* closest approach and infers `v_∞` from the **point-mass** energy `v_∞² = v² − 2μ/r`, so `J2`'s potential correction there (~`J2·(R_eq/r)² ≈ 5e-3` of `μ/r`) biases it ~1% — visible as the **capture radius moving 11 311.3 → 11 389.0 km** (78 km, 0.69%) against a perigee shift of only 1.33 km. The control that names the mechanism: **1PN leaves the capture radius at 11 311.3 to the digit** (its correction there is ~1e-9 relative), so this is `J2`'s `1/r⁴` growth inside the body, not the reduction. For any *miss* geometry — every deflected trajectory, the case that actually matters — the term is in its valid domain and none of this arises. Read the 1.33 km as "of order a kilometre on a boundary-grazing geometry"; **measuring `J2` on a genuine miss geometry is the honest follow-up.** **Done 2026-07-27 — see *The `J2` pair* below: 0.12 km outward at a 3.0 R_eq perigee, and the capture-radius bias collapses 480x, which is what makes this paragraph a measurement rather than a story.** (This also explains the 11 389 vs the pinned 11 311 km: not two disagreeing code paths, one term evaluated out of domain.)
- **Multi-revolution Lambert — and the shipped bug it exposed** (`core/src/lambert.rs`). `lambert_universal_multirev` solves the `N`-lap transfer, which needs a *different root-finder*: inside the band `z ∈ ((2Nπ)², (2(N+1)π)²)` the time of flight diverges at both edges and dips to a minimum between, so Newton from any seed walks into the wrong basin or off an edge. It brackets instead — scan for the minimum, reject a `Δt` below it as `NoSolutionForRevolutions` (a real geometric gap, reported with the threshold missed, never a `NaN`), bisect on the requested `LowZ`/`HighZ` branch. Validated by flying each solution through the **analytic Kepler propagator** and confirming it reaches `r2` — an independent check across a real formulation gap (universal variables/Stumpff vs classical elements/Kepler's equation), which a wrong root fails.
- **The bug: `lambert_universal` was silently returning lapping transfers.** `T(z)` rises monotonically to infinity as `z → 4π²`, so a single-rev root exists for *every* time of flight — but a Newton step from the `z = 0` seed can overshoot straight past that pole and converge in the 1-revolution band. The result looks perfect (it reaches `r2` on time) while being a transfer that laps the Sun, carrying a different `C3`, labelled direct. In a porkchop that is the worst kind of wrong: a plausible number in a cell that is not what it says. `SINGLE_REV_Z_MAX` clamps the iterate; the regression test is **physical** (a sub-one-revolution transfer must finish inside its own orbital period) rather than a peek at `z`, and a long-window case covers the clamp's own weak spot, where it degenerates to pure bisection near the pole.
- **The fix and the feature are two halves of one change** (`core/src/mission.rs`). The default grid's times of flight are ~3.6–3.9 yr, squarely in the affected zone — so clamping *alone* would have replaced those accidental lapping transfers with the honest direct arc, which at long spans is the slow, ruinously expensive one. Measured on a 2.6 yr span: direct arc **C3 = 933 km²/s²** (no launcher reaches it) versus **55 km²/s²** lapping. The clamp alone would have turned the long-time-of-flight half of the heatmap infeasible. So `best_transfer_metrics` now selects the lowest-`C3` option across `N = 0…max_revolutions` (both branches per `N`), `porkchop_grid` takes `max_revolutions`, and `TransferMetrics::revolutions` **says which trajectory a cell actually is** — a mission that laps the Sun is a different cruise, not just a different number.
- **The full NASA-LSP tables, and a doc claim that was 9× wrong** (`core/src/launch_vehicle.rs`). Every knot of every AMAT CSV is now embedded (101/10/100/64/100 points), machine-fetched via `gh api`. The previous ~10-point downsample shipped with a note claiming its interpolation error was "well under 1%". That had never been measured; measured, it is **8.9%** for Atlas V near `C3 = 95`, 3.2% for Falcon Heavy reusable, 2.7% for Vulcan. The curves are smooth in the middle but bend sharply as a vehicle nears its energy limit — the high-`C3` region a fast intercept lives in, so the error was concentrated exactly where it mattered. The transcription itself was faithful (all 11 shipped Atlas knots match the full table *exactly*); only the sampling was too sparse. Two new tests pin the row counts and strict `C3` ordering, because interpolation/monotonicity tests all stay true of a subset and would not notice a silent re-downsample.
- **The build-time "regression" was a measurement artefact.** Recorded as "debug DLL 11 s → 34 s"; it does not reproduce. Measured, `touch core/src/lib.rs` → gdext DLL: **2.5 s** steady state, **19 s** with the rustc incremental cache deleted, ~95 s on the first build of a session (cold OS file cache on top). Deleting the 933 MB `target/debug/incremental` and immediately rebuilding twice isolates it cleanly — 19 s then 2.45 s, same code, same profile. So the variable is **cache state, not the grown core and not the `opt-level = 3` override**, and the original 34 s was almost certainly a post-edit cold-cache run. Nothing to fix; closed as unreproduced rather than left open, with the numbers recorded so the next person does not re-litigate it.
- **Grid cost, measured before the heatmap view needs it:** selecting over revolutions is **0.6 µs/cell** at `max_revolutions = 0` versus **44.7 µs/cell** at 1 and 87 µs at 2 (`core/examples/bench_porkchop_cell.rs`) — a ~70× step for the first lap, since each `N ≥ 1` is a scan-and-bisect. A 200×200 grid is 23 ms direct-only against 1.8 s allowing one lap: fine for a grid built once on a worker, not per frame. A ~2× saving is there whenever it matters (scan each band once, solve both branches from that scan). Recorded now because this project has twice been bitten by an unmeasured per-cell cost.
- **`best_transfer_metrics` ranks on `C3` alone**, which is deliverability, not aim — in tension with the module's own *deliverable ≠ well-aimed* thesis, since a cell reports the along-track projection **of its cheapest option**. `C3` is still the right primary key (it is the hard constraint: over a launcher's energy limit delivers zero mass, and zero mass deflects nothing), but the criterion and its caveat are now stated where the function explains itself rather than left implicit.
- ~~**Still open, deliberately:** the frontend `[P]` force menu measures GR/Yarkovsky/belt/SRP but **not** `J2`~~ — **closed 2026-07-27, below** (*The `J2` pair*), together with the "measure `J2` on a genuine miss geometry" follow-up two bullets above; they turned out to be one item, not two. ~~Also unchanged: the **Godot porkchop heatmap view**~~ — **landed 2026-07-27, below.**

### The `J2` pair — 2026-07-27 session (the force menu's fifth term, and `J2` measured where it is valid)

The one item the leftovers entry above left open on purpose, plus the honest follow-up it named two bullets earlier. They read as two tasks and are one: the `[P]` menu measures every term on the shipping nominal, and for `J2` alone that geometry sits **outside the term's domain**, so shipping the menu entry without the in-domain number would put a boundary-grazing figure on screen under four that are not.

- **The menu's `J2` had to be measured on the nominal, whatever its domain.** The panel forms every shift as `nominal_perigee − shifted_perigee`, one baseline for all five. Measuring `J2` on some better-behaved geometry and subtracting it from *that* baseline would difference two unrelated passes and print something that looks like a shift and is not — the exact failure class this project keeps catching. So the fifth row is measured exactly like its neighbours (fixed seed, one term swapped, `nominal_encounter_with`), and the caveat is carried by a **footnote** instead: one line, drawn only while `J2` is revealed, citing the in-domain number beside it. No third availability state — `tier2_available` stays a `>= 0.0` bool, and `J2` has no kernel dependency to be unavailable for.
- **A designed miss cannot be built; a deflected one can.** The obvious way to reach a wide perigee is a bigger `b_offset_km`, and it does not work: `RealFieldScenario::build` verifies its designed impact round-trips, so 15 000 km comes back as `perigee 1.500e7 m ≥ capture radius 7.711e6 m (not a hit)` rather than as a miss. Measured before designing anything (`core/examples/probe_miss_geometry.rs`). That leaves the deflected pass — which is also what the docs already said matters, since *every* successful deflection is one. New core entry point `RealFieldScenario::deflected_encounter_with`: same contract as `nominal_encounter_with` (seed **and impulse** fixed, only the field toggled) with both routed through one private `with_toggled_field`, so the two can never disagree about what "`J2` on" means.
- **The geometry was solved for, not guessed.** The probe solves `required_dv_along_track` for a target perigee and reports what the pass actually reaches: **0.399625 m/s** along-track one year out → perigee **19 139.2 km = 3.001 `R_eq`**, `|B|` 25 064 km against an 11 312 km capture disc — a clean miss, comfortably in domain. (It also re-taught an old lesson about cost: the first run put the impulse at the campaign start, which makes every one of ~30 bisection steps a full 12 yr flight. It was still going after 18 minutes. What is being fixed here is a *perigee*; the lead time only sets how much Δv buys it.)
- **The result, and it is not just "smaller".** On the miss geometry `J2` moves the perigee **0.1196 km outward**, where on the impact geometry the same term shows **1.3257 km inward**. Different magnitude *and* different sign — though the sign is not by itself evidence of the domain problem, since the term carries a Legendre factor in the latitude of closest approach that two different passes have no reason to share. The sufficient point is narrower and holds regardless: the menu's 1.33 km is *that geometry's* number, not "what `J2` does to a deflection", which is why it is captioned.
- **The assertion that would fail if the explanation were wrong.** "`J2` moves the perigee by a nonzero amount" is what the existing sibling test already asserts and would pass on any geometry — green, and worth nothing here. The docs' claim is *causal*: the capture-radius anomaly is `J2` evaluated deep inside the body, because the b-plane reduction infers `v_∞` from **point-mass** energy at the sampled closest approach. That correction goes as `(μ/r)·J2·(R_eq/r)²`, i.e. **`1/r³`** — note the `μ/r`, which is why it is `1/r³` and not the `1/r²` the potential term alone suggests. So the test asserts the anomaly *collapses with distance*, two ways: model-free (at least 10×) and against the predicted `1/r³` with slack for the Legendre factor, which can shrink the result but not inflate it. Measured: **0.6867 % → 0.00143 %, a 480× collapse** across a 6.4× wider perigee. A reduction that were simply biased would not care about `r`.
- **The control that names the mechanism, re-run on the new geometry:** 1PN on the *same* deflected pass leaves the capture radius at 11 311.7 km, `4.4e-7` relative. If the capture radius moved for any reason other than a term's own potential reaching into the reduction, it would move there too.
- **Five call sites, and the two that were made impossible instead of edited.** A new term touches `Tier2Shifts`, `measure_tier2_shifts`, the term table, the per-term toggle dict, and main.gd's key chain. Two of those were separate hand-kept lists that would silently half-work if they drifted — `toggle_tier2` ignores an unknown id, so a term in the table but not the dict does nothing when its key is pressed. Both now **derive from `TIER2_TERMS`**: the dict is populated in `Sim._ready`, and the shot harness reads its key/id pairs out of the same table (via `OS.find_keycode_from_string`) rather than restating them — a hand-kept second list is how a new term gets a row, a measurement and an action while nothing ever presses its key, and the check reads as coverage while being none.
- **`[O]` for Oblateness, because `[J]` was taken** (`milestone_jump`, keycode 74) — checked against every existing binding rather than assumed, and it keeps the mnemonic set G/Y/A/S/O.
- **The panel's columns are now measured, not arithmetic.** The `J2` row is the longest label in the table and cleared the old `30 × _fs × 0.60` guess by ~27 px. Last session the porkchop readout's second column overlapped its first for exactly this reason; the fix there was to size off the font's own measurement, and it is the fix here — `xstate` comes from `get_string_size` over every label in the table. The row count is derived from the table too (it was the literal `11.0`), so the box cannot end up one row short of its own contents.
- **The in-domain number is a pinned constant, not a caption.** `J2_DEFLECTED_MISS_PERIGEE_SHIFT_KM` lives in the core beside the API that measures it, reaches the panel through the binding, and is asserted against the live measurement by `earth_j2_on_a_deflected_miss_is_in_domain` — so the footnote cannot drift from the physics. The same treatment `SB441_BODIES` and `threat_mass_kg()` get.
- **Cost:** five terms instead of four, so the preview is a fifth longer in principle — **measured in the frontend at 119.8 s**, comfortably inside the harness's 240 s wait. The panel's standing "~2 MIN" is what the measurement supports, so it stays; the strings that *were* stale are the "four shifts" ones, and where the panel prints a count it now reads it from `TIER2_TERMS.size()` rather than spelling it.
- **Verified through the real key path, then in the frame.** `_tier2_shot.gd` presses `[O]` as an actual `InputEventKey` through `main._input`, so project.godot's action → main.gd → `Sim` → core all had to be right for the assert to pass — which is the check the hand-edited `project.godot` needed, that file being the one edit in this batch that no compiler sees. All five shifts came back live: **GR +55.55 · Yarkovsky +5.10 · SRP +8.36 · `J2` +1.33 · belt +0.55 km**, matching the recorded values to the digit. The picture then confirmed what no assertion covers: five rows inside a box that grew for them, the longest label in the table clearing its `ON` column, and the footnote's *0.12 km outward* sitting directly under the row that reads *+1.33 km inward* — the two numbers legible against each other, which is the entire point of adding the caveat rather than just the term.
- **The batch found one piece of coverage that was already lying.** The binding's preview test — `tier2_preview_measures_three_terms_and_leaves_belt_unavailable_unmounted` — loops over the terms it expects to be available, and that loop read `["relativity", "yarkovsky", "srp"]`. It passed, in full, **without ever touching `J2`**: the new term was measured, wired, displayed and shipped while the test named for checking exactly that quietly checked four-fifths of it. The loop now runs off a `TIER2_TERM_IDS` constant that lives beside `Tier2Shifts`, so the next term is a visibly short list rather than a green run, and the test is renamed off the count it had outgrown. Worth stating plainly because the *shape* recurs: a check written against an enumerated set stops being coverage the moment the set grows, and nothing about it turns red to say so.
- **Left open on purpose:** `deflected_encounter_with` ships as public core API with exactly one caller, the test. It earns that on the drift argument alone (it and `nominal_encounter_with` are now one code path), but nothing in the frontend uses it yet — so the `[P]` menu still answers "what does this term do to the *nominal*" and cannot answer "what does it do to **my plan**", which is the more useful question and is now one call away. Noted here rather than half-wired, the same way this menu's `J2` row was.
- **A dropped catch worth recording:** the first run of the harness was against a **stale debug DLL**. `godot --path godot` runs the project in debug and loads `target/debug/`, and only the *release* binding had been rebuilt — so the run sat there with a binding that had no `j2` term in it at all. Nothing said so; it simply produced no output. When a Godot verification run goes quiet, check which profile's DLL it actually loaded before debugging anything else.

### The Godot launch-window map — 2026-07-27 session (the §8 heatmap view, closed)

The mission layer's long-standing follow-up, open since the Lambert/porkchop core landed on 2026-07-21: the physics was built and tested, and no one could *see* it. `[4]` now opens a real porkchop over the real campaign, and the view exists to make the project's own headline honest — the planner's "spend 0.2 m/s twelve years out" assumes an impulse can be delivered, and this is the map of whether it can.

- **The layer is three files and mirrors the Tier-2 menu exactly**, because that split is already proven: `mission_core::PorkchopView` (godot-free, worker-callable) → `Mission::begin_porkchop`/`poll_porkchop` on its **own** `mpsc` channel → `Sim` state under its **own** `pork_online` flag → `porkchop.gd`, a pure-display `Control`. The grid is built **on demand** when the view opens, never on the scenario build path — the same discipline the `[P]` menu established after chaining a measurement onto the build was measured at >200 s in-game.
- **Three kinds of empty, drawn three different ways.** A cell can be blank because *no trajectory exists at any lap count* (`c3 = -1`, background), because *this launcher cannot reach that `C3`* (`payload = 0`, dim floor), or because it is a real reachable window that *projects poorly onto the track* (a dark but live cell). Collapsing any pair throws away something the operator needs — the second is the entire payoff of a vehicle-independent grid, and the third is the module's *deliverable ≠ well-aimed* thesis. Measured on the shipping 120×120 grid: **4849 blank / 7193 unreachable / 2358 reachable** for Atlas V 551, so all three states are genuinely populated and the distinction is not decorative. And the second state really does move with `[L]` — across the five launchers the same 12 042 real transfers yield **2358 / 1438 / 2358 / 2281 / 2358** reachable windows, Falcon Heavy *reusable* being the outlier because its table stops at `C3 = 64` where the others reach 96–100. Counted for **every** vehicle in the harness rather than the one on screen, since the count for whichever launcher happens to be showing cannot demonstrate that switching changes anything. Sentinels, never `NaN`: one `NaN` in a packed column poisons every min/max the ramp normalizes by and flattens the whole picture to one colour.
- **How many laps the grid allows was decided on numbers, not taste.** `DEFAULT_MAX_REVOLUTIONS` is the one free parameter that changes what an operator *sees*, since every extra lap is another family of cheaper transfers. Measured over the default campaign (24×24, 379 real transfers, windows Falcon Heavy expendable can reach): **`N≤0` → 21 · `N≤1` → 56 · `N≤2` → 95 · `N≤3` → 129**, at 4.8 / 47.8 / 69.0 / 106.8 µs per cell. A direct-only grid therefore shows **under a quarter** of the missions that exist. Shipped at **2**; the test prints the whole table, because laps keep opening windows forever (at long times of flight a tighter lapping orbit is simply cheaper) so there is no knee to find — only a cost that doubles for cruises that grow by a full solar orbit each step. The shipping grid solves in **851 ms** on a worker.
- **The frontend must not invent a third rock.** The delivered Δv divides by the threat's mass, which nothing had ever needed before — and `SrpParams::sub_km_rock` hides the body (150 m, 2000 kg/m³) as function locals, so getting a mass meant restating them. Named as `THREAT_RADIUS_M`/`THREAT_DENSITY_KG_M3` → `threat_mass_kg()` ≈ **2.83e10 kg**, with a drift test asserting `3/(4rρ)` equals `SrpParams::sub_km_rock().area_to_mass_m2_per_kg` — the same treatment `SB441_BODIES` gets. Without it the SRP toggle could model a 300 m body while the porkchop divided by some other one. (`mission.rs`'s test fixture uses `2.0e10` and stays a fixture; this is what the *shipping display* divides by.)
- **The verdict reads `|B|` against `b_capture`, and a harness assertion re-derives it.** Exactly one number in the view is not a patched-conic planning estimate: `[E]` re-flies the asteroid through the full `n`-body field with the impulse **that launcher** would deliver through **that window** (~3.2 s, its own channel again). Its outcome is an *enum* — `CleanMiss` / `Encounter{…}` / `NotHyperbolic` — not a struct of sentinels, so the best possible result cannot share a `-1` with "not verified yet". The measured example: 3956 kg through a `C3` 22.3, two-lap, 2049-day window imparts **+3.06 mm/s** along-track and leaves `|B|` at 6930 km inside the 11 311 km capture disc — **`SURFACE IMPACT`**. That is the honest headline of the whole layer: a single real launcher's kinetic impactor delivers ~1/65th of the ~0.2 m/s the deflection curve wants, and the map says so instead of implying otherwise.
- **The view found a shipped bug in `launch_vehicle.rs`, and it was in the *cheap* direction.** `payload_kg` returned `0` outside the tabulated `C3` range at **both** ends — a faithful port of AMAT's `interp1d(fill_value=0, bounds_error=False)`, and wrong at the low end, because a lower `C3` is an *easier* departure. It had been harmless while nobody asked about cheap transfers; allowing two laps pushed the grid's cheapest cells to **`C3 = 0.34 km²/s²`**, below the `1.0` where four of the five tables start, so the heatmap drew real, easily-flyable windows as unreachable and captioned them *too much C3 for this rocket* — precisely the reverse of the reason. Below the first knot now holds that knot's payload flat (conservative: the true value is a little higher); above the last knot is still `0`, which is the genuine physical ceiling. The payoff is that **`0` now means exactly one thing**, so the display renders one honest state instead of one word covering two opposite situations. Where a table *starts* is an artefact of the published data's sampling; where it *ends* is physics — treating the two ends alike was the mistake. Pinned by `payload_kg(0) > 0` for every vehicle (a bare escape trajectory is the least exotic departure there is) and, in the binding, by asserting every launcher delivers something at *the grid's own cheapest cell* — so if the grid ever stops reaching that low, the guarantee does not quietly become untested. **The 12×12 test grid never goes below 1.0 (its cheapest cell is `C3` 3.64) and passed throughout**; the 24×24 grid reaches 0.34 and the shipping 120×120 reaches **0.269**. So the check lives on a grid that demonstrably enters the regime, and it **asserts that it does** before asserting the behaviour — a below-the-table check on a grid that never goes below the table is worse than no check, because it reads as coverage. The advisor caught the whole thing pre-commit; the first version of the check was itself vacuous.
- **Delivered mass is modelled *as* impactor mass** — no bus, no propellant, no structure bookkeeping (a Phase-3 refinement, already noted in `mission.rs`). That makes every payload figure here **optimistic**, which matters for reading the headline: the `SURFACE IMPACT` verdict is conservative in the safe direction. A real spacecraft delivers less than 3956 kg of impactor through that window, not more.
- **The heatmap is a texture, not 14 400 `draw_rect`s.** `_process` queues a redraw every frame while the view is visible, and the first cut redrew every cell each time. One texel per cell with `TEXTURE_FILTER_NEAREST` gives the same crisp blocks in one draw call (and nearest matters on its own terms: a smoothed heatmap would invent gradients between windows that were never solved). The reachable-window count is likewise cached in the rebuild rather than swept per frame.
- **Verified by picture, and the picture is what found the bugs.** The `_pork_shot.gd` harness drives every step through `main._input()` with real key events — project.godot action → main.gd → `Sim` → core, not direct calls that would bypass the part most likely to be miswired — and asserts the view switch is exclusive, that `[L]` changes the delivered mass and wraps the table, and that `is_hit` agrees with `|B| ≤ b_capture`. All of that passed while the *first* screenshot showed a tiny green blob in the corner: `pork` had never been added to `main._sync_overlay_sizes`, so the Control kept its default size. Two further collisions were visible only in the image — the HUD's mission panels sitting on the heatmap (they now stand down for this view, as the 3D tag layer already does for the 2D views) and the readout's second column overlapping its first (now sized off the font's measurement of the longest left-hand line, not a guessed character count). **Every one of these passes a test suite; none of them survives looking at the frame.**

### Required impactor mass on the map — 2026-07-27 session (`[M]`, the follow-up `[E]` cannot answer)

The last thing the porkchop layer left open. `core::mission::required_impactor_mass` had been built, tested and documented since 2026-07-21, and no frontend could reach it — the map could say *this launcher fails here* and then stop, which is the least useful place to stop. `[E]` asks **does this launcher work through this window**; `[M]` asks **how much mass would**, and the ratio between them is the campaign's honest headline stated as a number instead of implied.

- **Measure-first decided every parameter, and the first estimate was 10× low.** The naive call — 100 kg seed, the core's hardcoded `1e-4` tolerance, a `1e9` cap — was measured at **455 s** on an early-arrival cell and **170 s** on a late one, with the expected-common `InfeasibleAtCap` path at 194 s / 74 s. The advisor's two corrections were both right and both invisible from the code: probe cost is **cell-dependent** (a probe re-flies from arrival to the encounter, so **18.2 s** at a 10.8 yr lead against **5.8 s** at 3.2 yr — the arrival axis spans 0.10–0.92 of a ~12 yr campaign), and the *infeasible* path is the expensive one because it walks the entire doubling ladder before it can say no. Shipping parameterisation is **46 s** for the best-coupled window and **31 s** for a hopeless one; the slow tail is an early-arrival cell far from the seed, ~3 min. Three knobs, and it is worth naming which bought what: the **seed** removed ~11 doublings, the **tolerance** ~7 bisections, and the **cap** is what stops an unreachable window climbing to 1e9 before admitting it.
- **The seed had to be made *safe* before it could be made *fast*, and that was a real bug in the shipped core.** `required_impactor_mass` returned `seed_mass_kg` verbatim when the seed already cleared the target — an upper bound reported as *the requirement*. Harmless while every caller passed a deliberately tiny seed; fatal the moment one seeds from something meaningful to save propagations, because the displayed physics would then be a function of the seed. It now brackets **downward** (halving until a mass *fails*) as well as upward, so a good seed buys speed and cannot change the answer. Pinned kernel-free by seeding **100× high** and requiring the same mass back as a low seed gives — the assertion that fails if the short-circuit ever returns. Only then was it safe to seed at `heaviest_deliverable_kg()` (**14 714 kg**, Falcon Heavy expendable at its cheapest `C3`, derived from the LSP tables rather than written down), which puts the first probe inside the range the answer lives in instead of climbing to it.
- **The cap's size is a display decision, not a solver parameter.** `100 × heaviest_deliverable` = **1 471 393 kg**, and a hundred rather than the ten first considered because ten (≈147 t) sits *below* every window's requirement in the shipping grid — every cell would read "over the cap" and the number the whole feature exists to print would never appear once. `InfeasibleAtCap` is rendered as **data, not failure** ("OVER 1 471 393 KG (100 BEST LAUNCHES) — REACHES ONLY 16 413 KM OF 20 000 KM"), because a clean-miss-shares-a-sentinel bug in a new place is still that bug.
- **One target, named once.** The requirement is solved against **`SAFE_PERIGEE_TARGET_M` = 20 000 km**, which was a bare literal duplicated across `viewer/src/bin/curve.rs`, the `curve.json` it writes, and the binding's `required_dv_matches_curve_json`. It is now a core constant those all read, so the map's *mass* requirement and the headline curve's *Δv* requirement are quoted against the same bar — two requirements measured against different bars would look comparable and would not be. Pinned by an assertion in the curve test: move the target and the recorded `curve.json` expectations fail loudly rather than silently describing a different mission. The readout **names the target in the line** ("20 232 KG TO REACH 20 000 KM PERIGEE = 7 x WHAT ATLAS V 551 DELIVERS") — "required mass" alone reads as *the mass to miss Earth*, which is a smaller number. It is a **margin, not a hit test**: the verdict question stays `|B|` vs `b_capture`, and this is a design goal in the perigee's own units, deliberately clear of the 11 311 km capture disc rather than grazing it.
- **The test is the round trip, not the return value.** HANDOFF already records the earlier catch on this exact module — *"the first cut verified wiring, not behavior; `perigee >= 0` would pass even with the impulse un-applied"* — so the binding gate takes the mass the solver reports, flies **that** mass through the independent `[E]` verify path, and requires the perigee it reaches to clear the target: **20 232 kg → 20 905 km** against a 20 000 km bar. And because *sufficient* is not *required*, it flies 20 % less and requires it to **fail**: **16 185 kg → 16 453 km, short**. Without the second half, returning the cap would pass everything. Both outcomes come from one grid — the best-coupled cell is `Feasible`, the worst-coupled late cell is `InfeasibleAtCap` — so neither is a purpose-built fixture, and the test prints its own cost so a 5× regression is visible rather than merely slow.
- **Vehicle-independent, and the harness presses `[L]` to prove it.** The requirement is a property of the window's geometry and lead, so it is keyed by `(launch, arrival)` with **no vehicle index** — cycling the launcher recomputes only the ratio beside it. That is not a nicety: keying it by vehicle would re-fire 30–180 s of propagation on a keypress that cannot change the answer. `_pork_shot.gd` drives `[M]` through the real action chain, asserts the pork guard beats the shared `plan_toggle` binding on the same key, then presses `[L]` and asserts the requirement is *still current* and no solve re-fired.
- **A shipped doc claim corrected on the way past.** `begin_cell_verify` documented `[E]` as "one propagation (~1 s)". Measured: **5.8–18.2 s**, cell-dependent. Never measured, wrong by 6–18×, and it mattered here because it is the unit the mass solve is priced in — a 10-probe solve budgeted at "~10 s" is a 3-minute one.
- **And the picture found one more, again after a fully green harness.** Every assertion passed while the frame showed the event log printing straight through the readout panel and the keys row. The console draws from `MARGIN` at full width; this view alone parks its readout in the right-hand half, so a long enough line lands on top of it. **The `[E]` verdict line already overran** — it had only ever been screenshotted *mid-typewriter*, so it read as fitting, and the longer `[M]` line is what made it unmissable. Fixed as a width **budget** rather than shorter messages (the next long message would repeat it): `PorkchopPlot.PANEL_X_FRACTION` is now one named number read by both the panel that sits there and `hud.gd`'s new `_console_width`/`_clip`. The labels were shortened too, and specifically at the *prefix* — the ratio is the end of the line and the part a reader wants, so it must not be what an ellipsis eats. Third time in this view: `_sync_overlay_sizes`, then col1/col2, now the console. **All three passed a test suite; none survived looking at the frame.**

### The deflection spectrum, nuclear half — 2026-07-27 session (§5's second method, and the source hunt that decided its coefficient)

Phase 2's checklist is down to two unstarted items; this is the first half of one of them. §5 asks for deflection modeled *as a spectrum across lead time* — gravity tractor at the gentle end, kinetic impactor in the middle, nuclear standoff at the top — and until now `deflection.rs` shipped exactly one method. The porkchop layer made the gap concrete rather than theoretical: it closed by printing `SURFACE IMPACT`, one real launcher delivering ~1/65th of what the curve wants, with nothing to compare that against.

- **Nuclear is impulse-shaped; the tractor is force-shaped — and the code already said so.** `deflection.rs`'s own `apply_impulse` doc has read *"a finite burn (gravity tractor, §5) is a Tier-2 force term, not this"* since the MVP. So the split is not a new decision: nuclear standoff lands as a sibling of `kinetic_impactor_dv` in `deflection.rs`, and the tractor will land beside `yarkovsky.rs` in `forces/` with the one thing no existing term has — a **time window**. That second half is deliberately a separate batch, because it needs a force term *and* a new solver axis (duration, not impulse), which `DeflectionScenario::required_dv` structurally cannot express.
- **Direction is chosen, and that is the real physical difference between the two methods.** A kinetic impactor's Δv points along the arrival relative velocity — the transfer picks the direction and the mission layer takes whatever along-track *projection* it gets (`impact_impulse`, and the whole reason `required_impactor_mass` must root-find). A standoff burst ablates whichever hemisphere the device is placed over, so its impulse direction is a mission choice. Which means the nuclear requirement is a **closed-form invert** of the existing along-track curve, not a second root-find: `yield_kilotonnes_for_dv` and the matching `kinetic_impactor_mass_for_dv` requote an already-solved Δv with **zero extra propagation**. Worth recording because the advisor's standing warning — *don't fork the bracket-and-bisect three ways, that's three places for the seed short-circuit bug to come back* — turned out not to bite here at all, and the reason it didn't is this geometric asymmetry rather than luck. It returns for the tractor.
- **The coefficient was fetched, not recalled, and the first two candidate sources were rejected on the numbers.** `launch_vehicle.rs` set the precedent, and a fast model misaligning the DE440 `CVAL` arrays set the cost of ignoring it. Ahrens & Harris 1992 (*Nature* 360, 429) is the canonical citation but is paywalled — its efficiency figures reach us only through secondary quotation, so it is cited as context and is **the source of no number in the code**. LLNL-PROC-485160's three-point *surface-burst* table (0.1 kt → 2.3 mm/s, 0.5 kt → 0.92 cm/s, 1 kt → 2.8 cm/s on a 1 km body) was rejected for being the wrong mechanism and for being non-monotone in Δv-per-yield (23 / 18.4 / 28 mm/s per kt) — the high yields eject 1.9 % and 7.5 % of the body, so it is not a nudge model. An early attempt to derive a coefficient from one parenthetical clause of that paper produced a value contradicting the same paper's own headline figure by ~3×, which is exactly the *"clean-looking coefficient with no provenance"* failure mode, caught before any code was written.
- **What shipped is `StandoffNuclear::DEARBORN_2007`, from a fully-specified published case.** UCRL-PROC-228569 (D. S. P. Dearborn, LLNL, 2007), the paper's own "Nudge Model": a **100 kt** device **300 m** above the surface of a **1000 m diameter, 1.05e12 kg** body deposits **11.5 kt** into the surface and settles the coalesced body at **≈6.5 mm/s**, with **99 %** of the mass still bound. Every quantity that matters is stated in one place — body, yield, height, absorbed energy, outcome — which is why this case and not another. Two details make it the right fixture rather than merely a usable one: the 300 m height is `d/R = 0.6`, which the *same* paper independently names as the **optimum height of burst**, so the coefficient is pinned at the geometry the model describes; and the paper's separately-stated ≈0.5 m/s escape speed falls out of its own mass and diameter only if 1 km is the **diameter**, which is asserted in the test — reading it as the radius would have made every Δv here wrong by ~8×. Inverting gives `C_m = 1.418e-4 N·s/J` at `η = 0.115`.
- **The honest uncertainty is stated as a spread, not hidden in a third digit.** LLNL-PROC-485160 reports a *different* run — 10 kt deposited ablating ~4000 t at "over 2 km/s" — which recomputes to `C_m = 1.91e-4 N·s/J`. The two independent series **disagree by at least 36 %**, and "at least" is load-bearing: *"over* 2 km/s" makes the 2011 value a **lower bound**, and the shipped 1.418e-4 sits *below* it, so the two point the same way and 36 % is the floor of the disagreement. An earlier draft of this section and of the doc comment claimed they "bracket rather than contradict" — they do not bracket anything, and the advisor caught it. The shipping choice is unaffected (smaller `C_m` ⇒ more yield required ⇒ errs high), only the framing was wrong. A test asserts the constant stays inside the published band — a guard on *provenance* no test of the model's arithmetic would catch, and **the only check in the suite that bounds the coefficient from above**, since the fragmentation case below bounds it only from below.
- **The validation is a case the coefficient was never fitted to.** Reproducing the Nudge Model is self-consistency, not evidence — the constant came from it. So the real test takes the *other* paper's *other* body: LLNL-PROC-485160 deposits 17 kt into a 270 m, 2.78e10 kg non-porous body and reports it **"completely fragmented" (Ke/Pe > 1)**. The model, given only the absorbed energy, independently returns **Δv = 0.363 m/s against a 0.166 m/s escape speed — a ratio of 2.19**, comfortably above escape. That is the assertion that fails if the coefficient is wrong by the order of magnitude a dimensional-plausibility check would wave through, and *"the formula looks dimensionally right"* was explicitly ruled out as an acceptance criterion. Its limits are recorded with it: the check is **one-sided** (it catches `C_m` too low, never too high) and it extrapolates across roughly **20× the surface fluence** the coefficient was fitted at, in the direction where coupling-per-joule is known to fall — so the 2.19 carries perhaps a factor of two of margin against a real systematic. Enough for the qualitative claim it tests, not enough for anything quantitative.
- **Yield is the axis; delivered mass is not.** §5 says *"model this as deflection physics only — never weapon design"*, and the mass→yield map is precisely where that bites, since yield does not scale linearly with device mass. So yield enters as an **opaque scalar** and no part of this crate converts a launch window's payload into one. A launcher's delivered mass can only ever gate whether a device of a stated class is carriable — a payload question the mission layer already answers.
- **`beta` was not reused, and that was a near miss worth naming.** Kinetic β is the ejecta momentum-enhancement factor (1–4, dimensionless, DART measured ≈3.6). Nuclear ablation efficiency is a different quantity with different units. One word covering two situations is the `payload_kg`-returns-`0`-at-both-ends bug in a new place, so the nuclear parameters are `coupling_efficiency` and `momentum_coupling_ns_per_j`, both named for what they are.
- **The model reports whether the body survives, and refuses to invent the curve between the two points that say.** Both papers are emphatic that this is a small-nudge model that stops being one as Δv approaches escape speed — 485160 states flatly that on a 100 m body (`v_esc ≈ 5 cm/s`) *"inducing a 1 cm/s speed change will almost certainly result in extensive debris ejection or fragmentation"*. That is `Δv/v_esc = 0.2`; the Nudge Model's intact case is `0.013`. Fifteen-fold apart, with **nothing published in between**. So `DisruptionRegime` has three states — `IntactDeflection` ≤ 0.013, `LikelyDisruption` ≥ 0.2, and `Uncharacterised` between them — rather than one invented threshold at a tidy midpoint, which would read as knowledge. Both anchors are asserted from the published cases, and the middle state is asserted reachable, because a three-state enum no input can land in the middle of is a two-state enum with a lie in it. This is the same discipline the J2 validity boundary got.
- **The comparison is quoted at one bar, and that single table is the point.** A second method is not a second formula; it is two answers to the *same* question. On the published 1 km body at its own 6.5 mm/s: the kinetic route needs **189 583 kg = 12.9× the heaviest single launch** any vehicle in `launch_vehicle.rs` manages (14 714 kg, Falcon Heavy expendable at its cheapest `C3`), while the nuclear route needs **one 100 kt device**, at a Δv the classifier rates `IntactDeflection`. Three methods quoted against three different bars would look comparable and would not be.
- **But that table is on Dearborn's rock, and it does not transfer — which is the batch's actual finding.** The first version of this section stopped at the paragraph above and stated the `IntactDeflection` verdict as though it were the campaign's answer. It is a property of a **1.05e12 kg** body with a 0.53 m/s escape speed. The shipping threat is a **300 m, 2.83e10 kg** body whose escape speed is **0.159 m/s**, and re-running the identical comparison there — live full-field `required_dv_along_track`, `threat_mass_kg()`, the same `SAFE_PERIGEE_TARGET_M` — inverts the conclusion:

  | lead (yr) | required Δv (m/s) | nuclear (kt) | kinetic (t) | Δv / v_esc | regime |
  |---:|---:|---:|---:|---:|:---|
  | 0.39 | 0.5878 | 243.6 | 462 | 3.705 | `LikelyDisruption` |
  | 0.79 | 0.5098 | 211.2 | 400 | 3.214 | `LikelyDisruption` |
  | 1.58 | 0.2551 | 105.7 | 200 | 1.608 | `LikelyDisruption` |
  | 3.16 | 0.1277 | 52.9 | 100 | 0.805 | `LikelyDisruption` |
  | 6.32 | 0.0662 | 27.4 | 52 | 0.417 | `LikelyDisruption` |

  **Every lead the campaign covers lands in `LikelyDisruption`** — the curve never even reaches the `Uncharacterised` band. 65.7 kt is where a burst reaches this body's escape speed; intact deflection would need Δv ≤ 0.00206 m/s, and the *easiest* point on the whole curve still asks **32× that**. So on this threat a standoff burst sized to do the job does not deflect the rock, it disperses it — which is exactly what LLNL-PROC-485160 says in words: *"[a]t a size of 100 meters ... inducing a 1 cm/s speed change will almost certainly result in extensive debris ejection or fragmentation. Fortunately, bodies of this size may be addressed by impactors."* §5 asks for the methods to be modelled as a **spectrum across lead time**; this measures where on that spectrum the campaign's own body sits instead of restating the spectrum, and the answer is that the nuclear term is the wrong tool for *this* rock and says so. The table lives in the binding (`deflection_methods_compared_at_one_bar_on_the_real_threat`) because core must not learn about the threat body, and the core-side table's doc now points at it as the one a frontend should quote. **This is the J2 pair's lesson recurring**: a per-term row has to be measured on the seed it will be displayed against, not on whichever body the literature used.
- **Kernel-free, and the suite was run with `ASTEROID_REQUIRE_KERNELS=1`.** The new term composes nothing that needs an ephemeris, so its seven tests are pure arithmetic against published numbers. The full workspace nonetheless ran under the require-kernels flag — 165 core in **111 s**, the capstone in 36 s, 23 gdext in 173 s — because the silent-skip trap has made two verification claims here vacuous before, and *runtime is the only tell*.
- **Not yet on the frontend.** This batch is core-only: no `[N]` key, no force-menu row, no panel. The porkchop view produced three bugs in two batches that every test passed and only a screenshot caught, so a display for this is its own batch with its own picture. (Keys already taken: `1`–`4`, `C`, `P`, `E`, `M`, `L`, `O`.)
- **What the tractor batch already knows, so it is not rediscovered.** Checked while scoping this half, and it is the one thing that will actually cost time: **`DeflectionScenario<'a>` holds `force: &'a dyn ForceModel` for the whole scenario lifetime** (`deflection.rs:223`, used by `deflected_trajectory`). Every solver here varies the *initial state* under a fixed field. A tractor duration solve is the opposite — each probe re-propagates under a **different force model** (the thrust window changes), not a different state. So the tractor needs an API that takes a force per probe; the `*_with(force, …)` constructors already in the file are the precedent to follow rather than an invention, but it is a real signature change and should be scoped deliberately rather than discovered mid-batch. That is also where the advisor's *"don't fork bracket-and-bisect three ways"* warning finally bites: nuclear escaped it because a chosen direction makes yield a closed-form invert, and duration has no such escape.

### The deflection spectrum, tractor half — 2026-07-27 session (§5's gentle end, and the solve axis `required_dv` structurally could not express)

Closes the deflection-method spectrum, and with it the first of Phase 2's two
remaining checklist items. §5 asks for deflection modelled *as a spectrum across
lead time* — gravity tractor at the gentle end, kinetic impactor in the middle,
nuclear standoff at the top. The nuclear half landed earlier the same day; this
is the other end, and it is a different *shape* of thing rather than a third
entry in the same list.

- **Nuclear was an impulse, the tractor is a force — and it is the first term in
  `forces/` with a time window.** Gravity and sunlight do not switch off, so every
  existing term is on for the whole integration. A tractor is a *mission*: it
  arrives, tugs, and leaves. `TowWindow` is that parameter, and it is why the
  deflection layer needed a new solve axis rather than a new coefficient.

- **There was no coefficient to source, and recognising that early saved the
  batch a repeat of the nuclear source hunt.** The standoff term needed Dearborn
  because momentum-per-joule is simulation-derived and unobtainable from first
  principles. A tractor's tow is `G·m_sc/d²` — Newton, nothing fitted. Lu & Love
  2005's quoted rate turns out to *be* that: their
  `Δv = 4.2e-3·(m/2e4 kg)·(d/100 m)^-2 m/s per year` reproduces our
  `G·m/d²·yr` to **0.30 %**, the gap being their two-significant-figure rounding
  of 4.212. So the paper was fetched for its **configuration** (20 t hovering at
  `d/r = 1.5` over a 200 m, 2 g/cm³ body) and its **cant bookkeeping**, and both
  halves get an independent published anchor: the tow rate above, and
  `T = G·M·m/(d²·cos[sin⁻¹(r/d)+φ]) = 1.052 N` against the paper's stated
  *"total thrust T = 1 N"*.

- **The cant angle is a thrust penalty, never a weaker tug — and the paper's own
  equation says so.** `T·cos[sin⁻¹(r/d)+φ] = G·M·m/d²` puts the cant on the left
  with the thrust; the gravitational attraction on the right has no `φ` in it.
  Canting makes the *engines work harder*, it does not make the *gravity weaker*,
  because gravity does not know where the nozzles point. A `cos(cant)` factor on
  the tow would look conservative while silently understating every delivered Δv
  in the project — the `payload_kg`-means-two-things defect in a new place. The
  split is enforced by signatures rather than by comment: `tow_acceleration()`
  cannot see the cant angle *or* the asteroid mass it would need, and
  `station_keeping_thrust_n()` is the one place asteroid mass legitimately enters
  the module. A test pins that widening the plume moves the thrust and leaves the
  tow bit-for-bit identical.

- **"It is just Yarkovsky with a window" is the validation asset, not the
  criticism it sounds like.** Station-keeping holds `d` fixed by construction, so
  unlike Yarkovsky and SRP the tow does *not* fade with heliocentric distance —
  which makes it exactly the `d = 0` case of the Yarkovsky `A2·(r₀/r)^d`
  parametrization. So the term needs **no new oracle**: the Gauss-planetary-
  equation machinery, with its uniform-in-mean-anomaly weighting already validated
  against a closed form, judges both. It moved out of `yarkovsky.rs`'s private
  test module into `forces/secular_oracle.rs` — one implementation, two callers,
  rather than two copies free to drift (the reason `SB441_BODIES` has a drift test).
  A `d = 0` circular closed form (`da/dt = 2·a_T/n`) was added so the tractor's
  exponent is not being exercised for the first time by the very test it judges.

- **The window edge was measured rather than assumed, and the measurement is
  sharper than the worry.** A hard on/off edge is a derivative discontinuity
  landing inside whatever sub-step the adaptive driver is taking. Measured on a
  free particle where nothing but the edges can be responsible:

  ```
                       rtol/atol 1e-9 (shipping)   rtol 1e-13 / atol 1e-6
   edges inside steps          -1.0e-4                     +6.3e-3
   edges on boundaries         -9.2e-10                    -9.2e-10
  ```

  A discontinuity does **not** defeat the error controller — it converts a
  *tolerance* into a *systematic* Δv error. Aligned to a step boundary no step
  contains a discontinuity at all and the answer is exact at any tolerance; five
  orders separate the rows at one tolerance. That decided the solver's design:
  **leave window edges free**. The `-1.0e-4` is the pessimistic end (six enormous
  steps, no gravity), it is ~1e-6 m/s on the campaign's tow, and snapping edges to
  the snapshot cadence would buy it back only by *quantizing the duration
  bisection to the cadence*. Recorded so it reads as a decision.

- **`DeflectionScenario` could not express this solve, and the fix was scoped
  before the batch rather than discovered inside it.** The type holds
  `force: &'a dyn ForceModel` for its whole lifetime; every solver on it varies
  the *initial state* under a fixed field. A tow-duration probe is the opposite —
  same state, a **different field** each time. `propagate_and_reduce(force, start,
  seed)` is the extracted body of `deflected_trajectory` that takes the field as
  an argument, and it is what made the second kind expressible.

- **`ForceSum`, not the `ForceRef` that was planned.** The intent was to box a
  borrowed base field into a `CompositeForce` alongside the tractor. That does not
  compile and the reason is worth recording: `CompositeForce` holds
  `Box<dyn ForceModel>`, which is implicitly `Box<dyn ForceModel + 'static>`, so a
  *borrowed* field cannot go in it at all — and Rust has no default lifetime
  parameters, so giving `CompositeForce` a `'a` would ripple through every use
  site. Summing two references (`ForceSum(base, tow)`) sidesteps it entirely and
  allocates nothing. It is still the decorator `forces/mod.rs` names as the reason
  `ForceModel` carries `Sync`; only its shape changed.

- **The duration solve is a *bounded* bisection, which is why the advisor's
  "don't fork bracket-and-bisect three ways" warning bit less hard than expected.**
  `required_dv` grows its impulse geometrically because there is no a-priori
  largest sensible impulse. A tow duration *has* an upper bound — the lead time
  before the encounter — so the bracket is `[0, cap]` from the outset: no seed to
  pick, no growth factor, and no expansion loop that could silently walk past the
  region where the response is monotone. What the two solvers genuinely share, the
  mapping from an encounter outcome onto the perigee scale (`NotHyperbolic` ⇒ a
  dead-centre hit, off-gate ⇒ `+∞`), was extracted to `perigee_scale` rather than
  copied, since two copies would be two places for that mapping to start reading a
  hit as an error.

- **The cap is anchored to the *nominal* encounter, and running out of it is an
  error rather than an answer.** Past the encounter, extra towing cannot move a
  flyby that has already happened, so the response flattens and a bisection on a
  flat function returns noise; a window edge inside the flyby would also perturb
  the geometry being measured. Hitting the cap raises `TowDurationCapped` carrying
  the cap *and* the perigee it reached. Returning the cap as "the required
  duration" would repeat a defect this codebase has already shipped once —
  `required_impactor_mass` handing back its seed mass verbatim when the seed
  already cleared, an upper bound reported as the requirement. A caller that
  cannot distinguish *"3.1 years is enough"* from *"12 years is not"* will read the
  second as the first, and the kernel-free suite pins the distinction with the
  same shape of test that caught it the first time.

- **Measured on the campaign's own rock, and the headline is a *different kind* of
  failure from the nuclear one.** The threat is 300 m / 2.83e10 kg at 2.00 g/cm³ —
  the same density Lu & Love assume, so the literature row and this one differ only
  in size. A 20-tonne tractor hovering at `d/r = 1.5` (225 m):

  | quantity | Lu & Love's 200 m body | this campaign's 300 m body |
  |---|---|---|
  | tow `G·m/d²` | 5.93e-11 m/s² | **2.64e-11 m/s²** (0.832 mm/s per year) |
  | station-keeping thrust | 1.05 N (paper: ~1 N) | **1.58 N** (cant 61.8°) |
  | Δv over the full 6.32 yr lead | — | **5.26 mm/s** |
  | Δv the curve requires at that lead | — | **66.2 mm/s** → **12.6× short** |

  The lead used is 8 orbital periods — the *cheapest* Δv on the whole sweep — so
  the shortfall is a **best** case, not a representative one. The nuclear term
  failed as the **wrong tool**: a burst sized for this rock exceeds its 0.159 m/s
  escape speed and disperses it, a regime change. The tractor fails as the **right
  tool at the wrong scale**: nothing about it is unphysical here, it is simply an
  order of magnitude too small, and the shortfall is a spacecraft-mass number. Of
  the three methods it is the one that comes closest to closing on its own terms.

- **And a feeble tractor does not merely fail — it deepens the hit.** Towing the
  full lead at 20 t moves the b-plane perigee from **3000.0 km to 2811.6 km**:
  188 km the *wrong* way. The nominal is a near-centre impact, so a tug this small
  walks the track *toward* Earth's centre rather than out the far side; perigee is a
  distance, so it dips toward zero before coming back up. This is asserted, not
  merely printed, because it is the concrete counter-example to *"perigee grows with
  tow duration"* — an assumption the solver is documented as **not** making. An
  earlier draft of that doc claimed monotonicity over the capped bracket; the real
  field falsified it and the doc was corrected to state what bisection actually
  needs: a **single crossing of the target level**, which holds because the dip goes
  further *below* the nominal while any sane target sits well *above* it (20 000 km
  against a 3000 km nominal).

- **The scale that does close it, and how it was reached without a third solver.**
  The tow is *exactly* linear in spacecraft mass, so the closing mass is arithmetic
  rather than a search: 20 t × 12.6 ≈ **252 t**. That is an estimate of the mass
  matching the required *Δv*, and slightly optimistic about the *perigee* bar, since
  a distributed tug arrives later on average than an impulse. So it is checked
  rather than trusted: at 2× that (504 t) the duration solve wants **3.81 yr of
  towing**, 60 % of the available lead, and the answer round-trips on the shipping
  force model — 3.81 yr reaches **20 008.8 km** against the 20 000 km bar, while
  20 % less reaches only 16 655 km. Converging within 9 km of the bar is the
  bisection working.

- **Cost was measured before a solve was wired on top of it**, which changed the
  test's design. One tow probe is **12.4 s** (a bare propagation), but one
  `required_dv_along_track` at this lead is **236.6 s** — the dv solve, not the tow
  probes, would have dominated. Since the nuclear comparison already solves this
  exact lead live and pins it against `curve.json`, the tractor test **reuses** that
  constant instead of re-paying 237 s for an identical number, and the constant was
  promoted from a local `const` inside one test to module scope so the two cannot
  drift. Whole test: **171 s**.

### The tractor on the frontend — 2026-07-27 session (`[K]`, six knobs, and the cheap model that had to be scored before it could be shown)

The tractor half above is core-only and answers one question: *does a Lu & Love
tractor deflect this rock?* (No — 12.6× short.) That is a dead end to read and an
interesting thing to **operate**, because the reason it fails is a scale rather
than a physics, and every lever that changes that scale is free to evaluate. So
`[K]` opens a bench with six live knobs — and the design work was almost entirely
in deciding *what number it is honest to print while a key is held*.

- **A live margin cannot be built from `a·T`, and the project's own note said so
  before the panel existed.** The campaign test already documented the delivered
  Δv as an *upper* bound — a tug spread over the lead arrives later on average
  than an impulse at its start, and late Δv buys less displacement. Turning that
  caveat into a live readout would have systematically flattered every
  configuration: **measured +21 %** at the one point where a real-field answer
  exists. What ships instead is the **impulsive equivalent**

  ```text
  Δv_eff = a · T · (1 − T / 2L)
  ```

  which is *not* a fitted correction — it is the same linear response `f(τ) ∝ τ`
  that produces the `1/lead` law, integrated across the tow window instead of
  evaluated at a point. One model underwrites both halves of the panel. Its
  cleanest consequence is worth stating on its own: **towing the entire lead is
  worth exactly half its delivered Δv**, which is why starting early beats towing
  hard.

- **Which way a cheap model is wrong matters more than how wrong it is.** Scored
  at the calibration point the campaign already owns (504 t towing 3.81 yr of a
  6.32 yr lead, whose real-field perigee lands on the bar), the two candidates are
  `a·T` → **1.205×** and the equivalent → **0.842×**. The shipped one reads
  **16 % low**: it calls a tractor short when the field says it just clears. That
  direction is the reason it ships rather than a tuned version — a deflection
  readout that errs toward "not enough" is safe and one that errs toward "enough"
  is not — and a test pins **both signs**, so a future edit cannot quietly flip
  the estimate to the flattering side.

- **The required-Δv law got a measured validity floor, and the test proves the
  floor rather than asserting it.** `Δv(n) ≈ Δv(1)/n` holds to **0.1 %** between
  one and two orbits and 3.9 % out at eight — but at *half* an orbit the product
  `Δv·lead` collapses to 58 % of its value, because a sub-orbital arc has not had
  time to turn a period change into along-track drift. Extrapolating there is
  **1.73× wrong**, measured. So below one orbit the panel prints no requirement
  and no margin at all — *absent*, not zero — and `required_dv_matches_curve_json`
  asserts the law really does fail below the floor, so the constant cannot be
  "tidied" away by someone who reads it as merely defensive.

- **`CURVE_JSON_DV_AT_8_PERIODS` had to leave `#[cfg(test)]`.** It was fine as a
  test constant while only tests cited it, and became the blocker the moment a
  *readout* needed the same number: the release build could not see it. Promoted
  to `REQUIRED_DV_AT_ONE_PERIOD` / `REQUIRED_DV_AT_EIGHT_PERIODS` in shipping
  code. A general shape worth remembering — a number that is "just for tests"
  stops being that the moment anything user-facing wants to quote it.

- **The wall is not the surface.** The obvious lower bound for a hover-distance
  knob is "just outside the rock", and it is wrong. The cant is `sin⁻¹(r/d) + φ`
  and the thrust divides by its cosine, so station-keeping has no solution at all
  once the cant reaches 90° — at Lu & Love's 20° plume that is

  ```text
  d/r  <  1 / cos φ  =  1.064 body radii
  ```

  a band that clears the surface, **tows perfectly well**, and cannot be flown.
  The knob's first draft bottomed at 1.02 and was reachable in three keypresses;
  the core guard correctly returns `None` there, which the panel would have
  formatted as **`0.000 N THRUST`** — reading as station-keeping being *free* at
  precisely the distance where it is impossible. Now
  `min_hover_radii_for_station_keeping` is a closed form in the core, exported
  through `tractor_defaults()` so the bound is physics rather than a literal in
  GDScript, and the readout carries a separate `holds_station` flag instead of
  inferring one from a zero. The thrust divergence approaching that floor is the
  honest answer to *"why not hover closer for a bigger `1/d²` tow?"*, so it is
  left visible rather than smoothed away.

- **The knobs are a table, not variables — chosen against the planner's
  precedent.** The planner spends an input-action *pair* per parameter, which at
  six knobs would be twelve `project.godot` actions and twelve `main.gd` branches.
  The bench borrows the porkchop's cursor idiom instead: UP/DOWN selects a row,
  LEFT/RIGHT adjusts, `[E]` measures. **One new action for six knobs**, and a
  seventh is one row in `Sim.TRACTOR_KNOBS` with no edit to `main.gd` at all. The
  harness iterates that same table, so a new knob is covered by existing.

- **A user-tweakable rock radius, without inventing a third rock.** The standing
  rule (`threat_body_matches_the_srp_default`) is that the frontend must not
  restate the threat's body, and a radius knob is exactly the edit that breaks it
  silently. Scoped structurally instead of by comment: `tractor_hover_over`
  derives its own body mass from its own radius and hands it nowhere but the
  thrust formula, while `threat_mass_kg` — the porkchop's divisor and the SRP pin
  — stays a function of a constant.
  `the_tractor_radius_knob_does_not_reach_the_shipping_rock` moves the knob 4× and
  asserts the pinned mass has not budged. The physics it exposes is the good part:
  at fixed `d/r` the tow goes as `1/r²` while the **required Δv does not move at
  all** (the threat integrates as a test particle), so a rock four times the radius
  is sixteen times harder to tug for exactly the same Δv.

- **The direction knob turned out to carry the sharpest lesson, and nothing had
  ever probed it.** Every full-field probe in the tractor work — core tests and
  frontend alike — had run *prograde*. Running the other one, measured on the
  shipping configuration:

  ```text
  PROGRADE     perigee 3000 -> 2811 km   (-188 km, DEEPER)
  RETROGRADE   perigee 3000 -> 3348 km   (+348 km, OUTWARD)
  ```

  Not a symmetric sign flip — the retrograde move is nearly **twice** as large,
  and it is the same near-centre geometry that makes perigee non-monotone in tow
  duration: the b-plane point sits ~3000 km off Earth’s centre, so one direction
  walks it *toward* the centre (perigee dips before it can come back out) and the
  other walks it straight away (perigee grows from the first day). The same 20 t
  spacecraft either worsens the impact or eases it, one keypress apart, with
  nothing about the *tow* changed. The panel opens on **prograde deliberately**:
  it is the configuration the campaign measured, it is the one that fails, and it
  is one keypress from the one that helps. Seeding on the flattering direction
  would hide the point.

- **The signed perigee shift is a first-class readout, not a ratio.** A
  margin-only panel would show a user tuning steadily "toward closing" while the
  impact deepened. `shift_m` stays signed all the way through the binding, and the
  harness asserts both directions — the inward move for prograde and the opposite
  sign for retrograde, so neither branch can rot unnoticed.

- **Three end-stops now have executed probe paths, because none of them did.**
  `duty = 0` (refused in words, not as a raw "invalid tow duration 0 s" from
  inside the window constructor), the lead knob's 11.5-orbit maximum (probes fine,
  −225 km), and the hover knob's plume wall. Each is a setting a user reaches by
  holding a key, and each was one keypress from an unexercised code path.

- **What the frontend session actually cost, and it was not the physics.** Two
  staleness traps, both silent, now written up in §6: Godot loads `target/debug/`
  while the entire Rust loop builds `--release` (a new `#[func]` simply "does not
  exist"), and a new `class_name` is invisible until the editor rescans. Plus one
  language trap worth its own line — **GDScript's `%` operator has no `%e`**. It
  does not raise; it errors once per call from inside `_draw` and puts an error
  string where a number belongs, sixty times a second, on a panel that otherwise
  looks entirely fine. The values here span 1e-11 m/s² to 1e13 kg, so it is not
  avoidable by rounding; `_sci()` formats them.

### Resolved by the 2026-07-20 session (Phase-2 3D, real bodies — Horizons NEO half)

- ~~"the Horizons per-object NEO SPKs reuse the identical read path"~~ → **they cannot; ANISE can't read them.** The plan of record (the sb441 note directly below) assumed a Horizons SPK would mount beside `sb441-n16.bsp` and read like any other body. It does not, and this was measured before any plumbing was written (`core/examples/probe_horizons.rs`, the gate that decided the whole approach): `sb441-n16.bsp` is **SPK type 2** (Chebyshev), a Horizons per-object SPK is **SPK type 21** (extended modified difference arrays), and **ANISE 0.10.3 has no type-21 evaluator** — it dispatches types 1/2/3/8/9/12/13 and returns `Type21ExtendedModifiedDifferenceArray not supported for SPK computations` for 21. No request parameter changes the type Horizons emits. "Same read path" was true of the call site and false of the decoder underneath it.
- **Chosen (advisor-gated): Horizons VECTORS → in-project sampled trajectory + cubic Hermite.** Ask Horizons for the same trajectory as *states* (position+velocity, `EPHEM_TYPE=VECTORS`, heliocentric `CENTER='500@10'`, `REF_PLANE=FRAME` ICRF, `OUT_UNITS=KM-S`) on a fixed 1-day TDB cadence, and interpolate between them. **The honesty property is preserved and it is the whole point:** the states are JPL's own relativistic solution either way, so this interpolates JPL's numbers rather than integrating our own worse ones. That distinction is exactly what separated it from the two rejected branches — (a) integrating a single state vector in our field re-litigates the deleted display-grade Kepler and hits the Tier-2 1PN trap to produce a *worse* trajectory than JPL already published; (b) implementing type 21 in a forked ANISE is variable-`MAXTRM` binary-record parsing, a real upstream contribution and a scope decision the user should own, not a default. **These NEOs are scenery, never the threat and never a deflection target.**
- **Cubic Hermite, because the table carries velocity.** The interpolant matches JPL's position *and* derivative at every node, so the drawn arc is tangent to the real trajectory, not merely near it. Accuracy is **measured, not asserted by eye** — and the measurement caught a real thing: `hermite_matches_held_out_horizons_states` decimates Apophis and reconstructs held-out samples, and the *median* converges cleanly (~12×/halving, fourth-order-ish) while the *worst case* barely moves and always lands at the **2029 Earth flyby**, whose hours-long curvature no daily table resolves. So what actually ships is measured directly in `shipped_cadence_error_across_the_2029_flyby` against a committed **hourly** fixture: **median 24 m, worst 18 885 km at the flyby** — 1.3×10⁻⁴ AU, a fraction of a pixel at orrery scale. Both flyby fixtures (`core/tests/fixtures/apophis_flyby_{1d,1h}.neo`, ~173 KB) are committed, so this one accuracy check runs on a fresh clone with **no kernels and no fetch**.
- **The data is a plain-text state table, not JSON, not a kernel.** `asteroid_core` depends on anise/hifitime/nalgebra and nothing else — serde is deliberately validation-and-viewer-only. So `.neo` files are a key/value header + one whitespace-separated state per line (floats via Python `repr`, shortest round-trip), parsed dependency-free in `core/src/horizons.rs`. Magic line `asteroid-neo-states 1`; a file that fails its declared-vs-actual sample count, frame (`SUN`/`ICRF_J2000`), or magic is a **hard error**, because a truncated download is otherwise indistinguishable from a legitimately short span. Tables live under `<kernels>/neo/*.neo`, gitignored and regenerable (`python pyref/fetch_horizons_neo.py`), absent on a fresh clone — everything works without them, the asteroids simply do not appear. Resolver + skip-loud test harness (`horizons::resolve_dir`/`load_all`/`load_all_for_test`) mirror `kernels.rs`.
- **NAIF numbering, the trap the sb441 note flagged in advance.** Horizons uses the **extended** small-body convention `20000000 + number`, so Apophis is **20099942**, verified by enumerating a fetched SPK's segment table — *not* sb441's `2000000 + number` (Ceres = 2000001). A digit apart; the wrong one is a lookup failure that looks like anything but a typo. Recorded as provenance only — the sampled read path never resolves it, since the almanac cannot answer for these objects at all.
- **The catalog now mixes provenance, and says so.** `OrreryBody` carries `Trajectory::{Integrated(Clock), Sampled(Neo)}` — the comet is *our* physics in *our* field (SSB metres), the NEOs are *JPL's* interpolated (heliocentric ICRF metres). The two frames differ by the Sun's barycentric wobble (~10⁶ km, "looks like a rendering nudge"), reconciled in the **single** `catalog_body_helio_ecl_au`. `catalog_provenance(i)` returns `"integrated"`/`"sampled"` and the frontend labels bodies with it — because a trajectory drawn beside real physics with nothing marking which is which is the exact mistake the deleted GDScript Kepler was.
- **ZERO-is-the-Sun, fifth instance.** A `.neo` table covers 2020–2070 against a clock that scrubs the DE kernel's ~300 years, so most of the range is *outside* it. `Neo::helio_state_at` returns `None` (never a zeroed vector) outside its span, per-body through `catalog_active`/`catalog_span_tdb`. `catalog_active` used to require the single `comet_online` flag — correct for one body, wrong the moment the catalog held four, since Apophis's table and the comet's arc cover different years and one flag cannot answer for both.
- **The threat is untouched, structurally.** A sampled NEO never reaches the almanac (it is a state table, not a kernel), carries no GM, and cannot enter `tier1_perturber_field`. So "mounting real asteroids cannot perturb the threat" is a guarantee, not a hope, pinned two ways: `neo_bodies_cannot_reach_the_force_model` (core, compile-time) and `real_asteroids_join_the_catalog_without_touching_the_threat` (binding) — one build, threat cap/perigee/impact read before and after the NEOs install, compared with `==` not a tolerance. Cap stayed 11 311 km, |B| 14 639 km, to the digit.
- **Orbit lines draw one lap, not fifty.** A NEO's table is decades but its orbit is ~a year, so a polyline over the whole span is dozens of precessing laps overplotted into noise (the comet escaped this only because its span *is* one authored period). `Neo::orbital_period_seconds` (vis-viva, the same "period-to-bound-the-window" move the `ephem` orbit path already makes — every drawn point is still a real state read) feeds `catalog_track_window_ecl_au`, which samples one period clamped inside the span.
- **Verified by picture.** `_shot.gd` gained `neo_1_on_arc` (scrubbed to the 2029 flyby: Apophis/Bennu/Didymos named, cyan, at 1.0–1.3 AU near Earth, one clean elliptical lap each, distinct from the amber belt) and `neo_2_past_span_gone` (2071, past the 2070 table end: all three absent, no orbit lines, nothing on the Sun). 82/82 GDScript assertions pass, including the per-body span gate and provenance checks.
- **Incidental: the debug-mount cost is fine.** The sb441 half left "`mission_online` 11 s → 34 s" open; the debug DLL now rebuilds and loads in ~20 s. Not chased further — no longer painful.
- **Open / next:** ~~these three are the §9 teaching asteroids on-screen but not yet validated against Horizons~~ — that validation **landed as the Apophis capstone** (2026-07-21, *Tier 2 complete* below): with 1PN + Yarkovsky both on the real field, our own integration of Apophis is diffed against its Horizons truth table per force term. The display asteroids stay sampled scenery; the capstone is the *integrated* validation. J2 and Pluto-in-shipping are the only Tier-2 force terms still open.

### Resolved by the 2026-07-20 session (Phase-2 3D, real bodies — sb441 half)

- ~~"the real-NEO half reads real NEOs out of `sb441-n16.bsp`"~~ → **it cannot; that file has no NEOs in it.** The plan of record rested on a factual error, caught by enumerating the kernel's SPK segment table directly rather than trusting the note. `sb441-n16.bsp` contains exactly **16 main-belt perturbers** — Ceres, Pallas, Juno, Vesta, Iris, Hygiea, Eunomia, Psyche, Euphrosyne, Europa, Cybele, Sylvia, Thisbe, Camilla, Davida, Interamnia — all Sun-centered (NAIF 10), 4 segments each, spanning 1550–2650. It is the **perturber set ASSIST integrates against**, not a target list: sub-km teaching NEOs like Apophis or Bennu would never appear in one. The §9 teaching asteroids are a *different* acquisition problem, and the split below is how it was taken.
- **Scope split (user call): sb441 now, Horizons NEOs next.** This commit builds the kernel-mounting plumbing against sb441 — real bodies on screen, zero network fetch — and the Horizons per-object NEO SPKs reuse the identical read path in a follow-up. Same end state, two commits, and the risky half (does mounting a third kernel work at all, and where does the cost land) is settled first against a file already on disk.
- **Why Horizons SPKs and not SBDB elements + integrate, for the NEOs.** Integrating a real NEO from published elements walks straight into the Tier-2 **1PN relativity** trap already flagged at §270: low-perihelion objects do not match JPL without the Sun's relativistic correction, and omitting it makes Horizons validation *silently* fail. A Horizons per-object SPK is JPL's own already-relativistically-integrated trajectory, so reading it sidesteps the question entirely — the teaching asteroids arrive correct before the force model is ready to earn them.
- ~~Where the small-body mount lives~~ → **on the existing build worker**, decided by measurement rather than taste, exactly as the comet's placement was. `sb441-n16.bsp` is 646 MB and mounting it costs **~5.7 s cold / ~272 ms warm** (release) — the gap is page-cache I/O, so a freshly launched game pays the full cost. `MissionCore::load_from` is contractually fast (~ms) and sits on the path to the first drawn frame, so mounting there would have traded a working 3-second startup for a frozen 9-second one. Per-query cost is negligible (~3.5–6 µs), so scrub reads are free once mounted.
  - The worker **cannot** mount onto the almanac it is handed: `Ephemeris::with_constants` consumes `self`, and the served `Arc<Ephemeris>` is being read by the render thread every frame. So it builds a *second* almanac from paths (`mount_small_bodies`, re-reading de440s at ~ms) and returns it inside `BuiltScenario`; `install` adopts it. The serving core never moves and is never mutated — the invariant the whole worker design exists to protect — and the scenario is served from the same field it was flown in.
  - A mount failure **warns and continues**. The mission is complete and correct without asteroids; taking the build down over scenery would trade a missing catalog for a missing threat.
- **The optional third kernel.** `KernelPair` gained `small_bodies: Option<PathBuf>` (plus `ASTEROID_SMALL_BODY_KERNEL`, and the GDScript mirror in `kernels.gd`), deliberately **outside** the both-or-nothing rule that governs `bsp` + `pca`: the file is twenty times the DE kernel and a fresh clone will not have it. Absent → `None` → no asteroids, everything else unchanged. Failing a *pair* over it would take the planets down on every machine without 646 MB to spare. Pinned by `small_body_kernel_is_optional`, and the test was checked by bypassing the resolver to fabricate a path and watching it fail.
- **ZERO-is-the-Sun, fourth instance — gated before it could ship.** These are `"ephem"` bodies on the planets' read path, so an unmounted lookup fails, and a failed heliocentric lookup drawn anyway is a body **on the Sun**. Two flags, not one: `small_bodies_armed` (a path was handed over) is *not* `small_bodies_mounted()` (the served almanac actually has it), and between those two states every lookup fails. Only the second gates a draw; `small_body_count()` also returns 0 when unmounted, so a caller that ignores the flag iterates nothing rather than sixteen bodies stacked on the Sun.
- **Verified by picture and by number.** `_shot.gd` gained a `belt_1_real_asteroids` section: all 16 report `armed=true mounted=true`, resolve at real main-belt distances (2.17–3.79 AU) with spread, non-zero node positions. The id table itself was checked by corrupting one entry (2000704 → 2000705) and watching the kernel reject it. **Not yet verified: individual visual identification** — at `vis_r` 0.020 under the green phosphor shader the sixteen are not distinguishable from the scenery belt's 1600 dust points in a wide shot. They are drawn as bodies rather than dust on purpose (that belt is a seeded RNG annulus spun rigidly; these are per-frame kernel reads), and making that distinction *legible* is open work.
- **Open: the debug-build mount cost.** `mission_online` went from ~11 s to **34 s** in the debug DLL Godot loads. The 5.7 s measurement was release; ANISE parsing 646 MB unoptimized is far slower. `profile-dev opt-level=3` is already applied to `asteroid_core` for exactly this class of problem — extending it to cover the mount path is the obvious next move if the editor loop gets painful.

### Resolved by the 2026-07-20 session (Phase-2 3D, comet)

- ~~Where a synthetic orrery body's integration runs~~ → **on the existing build worker**, handed back with the scenario. `add_synthetic_body` is inline-and-expensive by design, and the measurement is why this was not a coin flip: the display comet costs **2.0 s over 12 yr / 8.1 s over 45 yr**, against a `build_scenario` of 11.2 s — so an inline call at install would have put multi-second stalls back on the render thread the worker exists to keep free. The seed math moved into a free `seed_orrery_body(&Arc<Ephemeris>, &RealFieldScenario, …)` that the worker and `add_synthetic_body` both call, so the two paths cannot drift; `install` now takes `(BuiltScenario, Vec<OrreryBody>)` because a new scenario invalidates the old catalog anyway (the bodies were flown in the old field). Span shipped at **one orbit ≈ 22.6 yr (~4 s)** — a second lap retraces the same arc for another ~4 s of build.
- ~~Whether GDScript keeps a Kepler propagator for "cosmetic context orbits"~~ → **no; it is gone.** The comet was the last user of `_elements`/`_kepler_pos_ecl`/`solve_kepler` and the Kepler fallback branches in `pos_ecl`/`orbit_points` — all deleted, mirroring 3C-2b's deletion of the threat's f64 Kepler block. The §5 Tier-0 tier still exists as a *concept*, but nothing in the Godot frontend draws from it: every drawn body now names a real source (`ephem` / `threat` / `threat_defl` / `catalog`). The fallback that used to run Kepler now `push_error`s instead of returning `Vector3.ZERO`, because ZERO in this heliocentric frame is the Sun — silently parking an unknown body on the Sun is the failure mode this whole seam is built against.
- **The ZERO-is-the-Sun trap, third instance** — and the first one caught *before* shipping rather than after. `catalog_position_ecl_au` returns `Vector3::ZERO` outside a body's propagated span, exactly like the planets (kernel coverage) and the threat (its ~12 yr arc) before it. The comet's one-orbit arc covers under a tenth of the ~300 yr scrubbable clock, so an ungated comet would sit on the Sun for most of the timeline. Gated per-body by `Sim.catalog_active(el, t)` off `catalog_span_tdb`, and **verified by picture, not by assertion alone**: `_shot.gd` shots `comet_1_on_arc` (inbound at 4.4 AU, tagged) and `comet_2_past_span_gone` (2051 — comet absent, planets untouched, nothing on the Sun).

### Resolved by the 2026-07-17 session (Phase-2 3C-2c)

- ~~Which pair decides hit-vs-miss on the display~~ → **`b` vs `b_capture`** — the core's own `is_hit`, with the focused capture disc (1.773 R⊕ at this encounter's `v_inf` ≈ 7.63 km/s) kept as the headline bar the planner and the b-plane view both measure against. §5's two criteria are equivalent *as pairs* — `b > b_capture` (the un-focused asymptotic miss against the enlarged target) ⟺ `perigee > R⊕` (the already-focused closest approach against the solid body) — and `geometry.rs` proves it. **Mixing them is what shipped**: `sim.gd` compared a *perigee* against the *capture radius*, charging for gravitational focusing twice and demanding ~1.5× more miss than physics does. Measured on a reachable plan (0.2 m/s at one period of lead): `b` = 14 640 km clears the 11 311 km disc by real daylight, while its perigee of 9 319 km sits inside it — so the display called a working deflection `SURFACE IMPACT`. Both quantities are "miss distances" in km, which is exactly why it survived. Now pinned in the binding against `is_hit` (both pairs, on the real perturbed field — the first check the two-body equivalence survives it) and at the GDScript level on the disagreement band itself, so the mixed bar cannot come back quietly. The displayed `PROJ MISS` became `b` alongside it: a player reads it against `CAPTURE` on the next line, so it must be the same pair.

### Resolved by the 2026-06-23 review

*First pass:*
- ~~MVP renderer~~ → **pure Rust; Godot is Phase 2.** (§2, §8)
- ~~MVP planet positions: analytic Kepler vs DE440~~ → **MVP integrates the asteroid as a test particle in the DE440/441 ephemeris field (positions + GM from ANISE) from Tier 1. Tiers add force *terms*, never switch the perturber source — so ASSIST is the oracle from day one.** (§5, §6)
- ~~What "deterministic" means~~ → **same-build-same-output, not cross-machine bit-reproducibility.** (§2)
- ~~Fixed-timestep vs adaptive-integrator tension~~ → **fixed snapshot *cadence*, adaptive integration *step* between snapshots.** (§2)

*Second pass (same day):*
- ~~Pure-Rust renderer crate~~ → **egui is the spine** (egui_plot + painter); plotters optional later; macroquad's animation edge isn't worth its GUI cost (Godot covers game-like polish). (§2, §8)
- ~~Default encounter integrator~~ → **dop853 for the MVP** (sufficient, easier, dense output for the clock); IAS15 is a Tier-2 long-arc upgrade. (§5, §7)
- ~~Impulse application phase~~ → **fix it for the headline curve; expose phase as a separate view.** (§5, §7)
- ~~MVP perturber set~~ → **Sun + 8 planets + Moon, with the force term and ANISE loader designed to add the 16 asteroid perturbers at Tier 2.** (§5)
- ~~Godot precision (Phase 2)~~ → **floating-origin first; double-precision build only as a fallback.** (§7)
- ~~Integration frame (was implicit)~~ → **barycentric (SSB) ICRF, SI units, present heliocentric** — dodges the non-inertial indirect-term footgun. (§2, §5)
- ~~Float-precision worry for the core (was conflated with f32)~~ → **f64 retires it; it's a rendering-only concern.** (§2, §7)
- ~~Clock sub-snapshot interpolation~~ → **served from dop853 dense output, not linear interp.** (§4, §7, §10)
- ~~Scenario/fixture format~~ → **JSON** (crosses the Python boundary natively); RON optional for Rust-only authoring. (§6)
- Added: **task-0.5 ASSIST/DE build de-risk spike + fallback-to-B trigger** (§10); **delivery + determinism honesty caveats** in UI copy (§1); **impulse soft-cap** to kinetic-impactor plausibility (§5).

---

### The threat orbit became a knob — 2026-07-28 session (`[N]`, a closed-form preview, and the requirement that had to move with it)

The `[K]` bench above lets an operator change everything about the *tractor* and
nothing about the *rock*. The obvious next lever — "test the tractor at different
orbits, some effective, some not" — turned out to be blocked by one line:
`begin_build_scenario` hardcoded `ImpactorConfig::default()`, so the frontend
could not put the threat anywhere else at all.

- **Three knobs ship, and the two that are frozen are the interesting decision.**
  `ImpactorConfig` also carries `impact_epoch` and `lead_years`, and together they
  *are* the mission clock: `epoch0` is GDScript's `EPOCH0_TDB`, the origin every
  drawn `t` is measured from, and `impact_epoch` is `T_IMPACT`, which the event
  schedule and both porkchop axes are laid against. Moving either does not put the
  rock on a different orbit — it slides the whole campaign along the timeline and
  invalidates every date already on screen. So the knobs are approach **speed**,
  **direction** (azimuth/elevation, not three redundant vector components), and
  **impact offset**: exactly the three that change the orbit while leaving the
  calendar alone. That is what keeps a rebuild a bounded operation.

- **A rebuild costs ~10 s and can fail two ways, so it needed a preview — and the
  preview is free because of a geometric accident.** The designed impact lays its
  offset *perpendicular* to the relative velocity, which makes the impact point the
  **perigee of the geocentric hyperbola**. So the flyby can be undone
  analytically — `BPlaneEncounter::from_relative_state` gives `v_inf` and the
  incoming asymptote in closed form, and the incoming heliocentric velocity is just
  `v_earth + v_inf·S`. `ImpactorConfig::preview` is microseconds against the
  builder's ten seconds.

- **Measured against real builds, and the two halves behave completely
  differently.** The encounter geometry is **exact** — `v_inf` and `b` match the
  propagated nominal's own b-plane reduction to **0.001 %**, which is not a
  tolerance being met but the same closed form arriving at the same answer. The
  *orbit* is an estimate: osculating at the impact epoch against a build reporting
  vis-viva at the seed twelve years earlier, worst observed **0.23 %** across
  0.68–2.66 yr of period. Good enough to label a knob, not good enough to score a
  plan — so the tractor bench keeps taking `period_seconds()` from the built
  scenario, and the panel says which number it is showing.

- **The "too wide to be a hit" wall is exactly Earth's radius, and the estimate
  that said otherwise was wrong for an instructive reason.** The first prediction
  held `v_inf` fixed and grew `b = b_offset·v_rel/v_inf` until it reached the
  11 311 km capture disc, giving ~4790 km. Measuring said 6400. The error: widening
  the offset also *raises* `v_inf` (less of Earth's well to climb out of), which
  grows `b` more slowly **and shrinks `b_capture` at the same time**. They meet
  where they must — `b <= b_capture` is the same statement as `r_perigee <= R_E`,
  and the perigee *is* the offset. So `b_offset` is a perigee-altitude dial wearing
  a b-plane name, and at the knob's ceiling the harness measures `b/b_capture = 1`
  to nine digits: the widest impact that exists.

- **The other wall closes from the opposite direction, which is the
  counter-intuitive part.** The flyby exists only while `v_rel > sqrt(2*mu/b_offset)`
  — 16.3 km/s at the shipping 3000 km against a shipping 18 — and **shrinking** the
  offset *raises* that bar (28.2 km/s at 1000 km). Pulling the hit toward Earth's
  centre is what falls off the cliff. New `ScenarioError::ImpactNotHyperbolic`.

- **THE SILENT FAILURE THIS LAYER EXISTS TO PREVENT.**
  `REQUIRED_DV_AT_ONE_PERIOD = 0.50975` is a measurement of one rock on one
  heliocentric orbit. Everything else about a rebuilt scenario keeps working — the
  threat draws, the planner solves, the probe runs — so a margin still quoting the
  *previous* orbit's requirement would look entirely healthy while being a number
  about a different mission. The anchor is now a field on the core, seeded free
  only when the installed config is the shipping one, and `tractor_readout` takes
  it as `Option<f64>`: absent means the bench prints **no requirement and no
  margin**, the same absence a sub-one-period lead already produces, and says which
  of the two absences it is. `required_dv_estimate(n)` was **deleted** rather than
  kept as a convenience — it reached for the constant, which would be the easy call
  at every future call site and wrong at most of them.

- **And the anchor cannot be shortcut by rescaling.** The tempting optimisation is
  `dv ∝ 1/lead` ⇒ scale the shipping constant by the period ratio and skip 28.8 s.
  Measured on a 2.66 yr orbit against the 0.79 yr shipping one it predicts a 3.4×
  reduction; the real requirement falls **~10×**. Off by 2.9×. Both orbits share a
  `v_inf` and a `b_offset`, so they need the *same* b-plane shift — what differs is
  how much of that shift an along-track nudge buys, which is approach geometry and
  not period. Pinned by a test, because that shortcut fails invisibly: a plausible
  number, on the right order, wrong for every orbit.

- **It teaches what it was asked to.** One 200 t tractor plan, one 6.0 yr lead, two
  orbits: **margin 0.372× on the shipping orbit and 1.096× on a long-period one** —
  fails and closes, from one knob. Separately pinned kernel-free, a plan inside the
  shipping knob ranges reaches **29×**, so the bench can teach success and not only
  the 12.6× shortfall.

- **Costs, measured rather than assumed — and the frontend measured a different
  number than the Rust suite did.** The one-period anchor solve is **28.8 s on the
  shipping orbit** against ~10 s for the build, which is why it is a separate
  on-demand action rather than part of the rebuild. But the harness, on the 1.44 yr
  orbit it dials to, took **41–74 s** across three runs — because the cost
  **scales with the period**:
  a one-period lead on a longer orbit is a proportionally longer propagation, and
  the knobs reach past 3 yr. So the long-period orbits an operator goes looking for
  are exactly the slow ones to score. The UI copy therefore says **"about a
  minute"** and deliberately not a range: the first draft promised "30–60 s", the
  next run took 63, and the one after that 74. Quoting the figure measured on the
  one orbit nobody
  rebuilds *to* is an accurate number and a misleading promise. The live solve on
  the shipping orbit
  reproduces the recorded constant to all six digits, so every orbit is scored by
  the code path the validated one validated.

- **The rebuild refuses while *any* other worker runs, and that is a correctness
  fix.** The porkchop grid, the Tier-2 preview, the cell verify, the mass solve and
  the tow probe each hold an `Arc` clone of the current scenario. Start a rebuild
  underneath one and it keeps computing — correctly — about a threat that no longer
  exists, then lands *after* `poll_build` cleared the state it belonged to and
  installs itself as current. Refusing by name tells the operator what to wait for;
  a generation counter threaded through six result types costs more and says the
  same thing. `poll_build` also learned to drop the **tow probe**, which had
  survived only because a stale one used to be unreachable.

- **A rebuild must invalidate results and leave intentions alone.** Rust drops the
  grid, verdict, mass requirement, plan and probe; none of that reaches the *flags*
  GDScript gates on, so `_invalidate_derived_views()` clears `pork_online` and the
  grid columns, the plan verdict booleans and the pending debounce. What survives
  is what the operator dialled — the planner's lead/Δv, and the tractor's six
  knobs. The first version re-seeded the bench on every install, which would have
  reset the spacecraft to 20 t every time the orbit moved and quietly made "change
  the orbit, watch the margin" a comparison between two different tractors. The
  harness asserts the survival, not just the clearing.

- **One new input action for a whole panel.** `[N]` opens it; `[ENTER]` is
  `plan_commit` ("apply what is dialled") and `[E]` is `pork_verify`, which by now
  means "stop estimating and go measure it in the full field" in three views —
  solving this orbit's required Δv is exactly that. The knobs are a table
  (`Sim.THREAT_KNOBS`) like the bench's, so a fifth is one row and no `main.gd`
  edit. The three bottom-centre panels became mutually exclusive through one helper
  rather than three hand-written pairs, which is the point at which the
  hand-written version starts dropping a case.

- **A rounded literal is worse than an obviously wrong one.** The offset knob's
  table placeholder read `6378.0` — Earth's radius to four digits, and 136.6 m
  short of it — so the *placeholder* set the ceiling instead of the core's
  `6378.1366`, and the knob stopped just inside the grazing boundary rather than on
  it. The harness caught it only because the tolerance was tight enough to care.
  Placeholders in these tables are now deliberately slack (`9000.0`), so the core
  is always the binding authority. Same rule as every drawn body naming a source.

### Tier 3 begins — 2026-07-28 session (covariance → b-plane → impact probability, and the cost that turned out to be a cadence)

The last question this project could not ask: not *does this rock hit*, but
*given what is actually known about where it is, how much of that spread lands
on Earth*. `core/src/uncertainty.rs` is the first half — the covariance mapping
and the probability. Keyholes are the second and are not in this batch.

- **The sample cost was measured before the module was designed, and it was not
  what it looked like.** A perturbed re-fly costs ~9.6 s, so a 12-column
  finite-difference Jacobian is two minutes and a 1000-sample Monte Carlo is over
  three hours. But almost all of that is the **snapshot cadence**, not the physics:
  `Clock::propagate` calls `step_dense` once per snapshot, so the shipping 1-day
  cadence restarts the adaptive integrator 4 443 times across the campaign. At
  3 days a sample costs 3.2 s, at 10 days 1.1 s, at 30 days 0.50 s.

- **The absolute answer moves with the cadence; the derivative barely does — and
  the derivative is the only thing this layer consumes.** The b-plane perigee
  shifts +3 cm at 3 d, +118 m at 10 d, +13.6 km at 30 d. But a Jacobian column
  differences two runs flown at the *same* cadence, so the systematic error is
  common to both and cancels: `∂(perigee)/∂x` holds to **0.024 % at 10 days** and
  only breaks (2.65 %) at 30. Checked on three columns — a coordinate velocity
  axis, the along-track velocity, and a position component — because along-track
  carries both the largest sensitivity and the cadence's own timing error, and
  would have been where a one-axis result flattered itself. It did not: all three
  agree, and all three degrade by the *same* 2.65 % at 30 d, so the cadence error
  is a uniform scale factor rather than anything direction-dependent. Ten days
  ships, at 1.1 s a sample — which re-prices the 12-column Jacobian at 13 s and a
  1000-sample Monte Carlo at ~18 minutes.

- **`cadence_days` claimed "trades storage/step-count, not accuracy". That is
  measurably false** and the doc is corrected. Nothing that shipped is wrong — the
  shipping config uses the finest cadence on the table — but a caller coarsening
  it for speed is paying in accuracy, not just memory.

- **The cadence is a constant of the module, not a parameter.** Same shape as the
  `REQUIRED_DV_AT_ONE_PERIOD` trap the previous session built a guard against: a
  frontend that dialled cadence for display reasons would silently change every
  covariance answer while everything kept working and every number stayed
  plausible. `SAMPLE_CADENCE_DAYS`, `FD_STEP_POSITION_M` and `FD_STEP_VELOCITY_MS`
  are pinned by a test that fails if they move without a re-measurement.

- **The step sizes are per-column, because metres and m/s share no scale.**
  Richardson study at the shipping cadence: `v_along` plateaus at 1.25e-4 m/s
  (0.007 % per halving) and goes ragged with round-off below 3e-5; `r_x` is
  *still truncation-dominated* at 1e4 m (1.9 % per halving), plateaus at ~3.1e2 m
  (0.003 %), ragged below 1.6e2. What the two plateaus share is not a step size
  but a *response* — both provoke a b-plane excursion of 10–20 km.

- **THE CONSTRUCTION THIS MODULE EXISTS TO AVOID.** The obvious Jacobian —
  propagate each perturbed state, find *its own* closest approach, reduce that —
  is wrong and produces a full, plausible, entirely incorrect matrix. Closest
  approach is an argmin over a sampled polyline, so the map is **quantised**: a
  small perturbation moves the argmin by a whole sample or not at all, and the
  differences come back noisy or identically zero while the matrix still looks
  structurally fine. Every run is reduced at **one fixed epoch** instead
  (`UNCERTAINTY_REDUCTION_LEAD_SECONDS`, 12 h before the nominal CA — ~330 000 km
  out, inside the SOI, outside the well where `v_inf = √(v² − 2μ/r)` would
  cancel). Legitimate because b-plane parameters are asymptotic properties of the
  hyperbola, not of the sampling instant — and **measured**, not asserted: the
  fixed-epoch `∂r_p/∂v_along` agrees with the at-CA one to **0.025 %**, which is
  the cadence's own error and nothing more. That agreement is the load-bearing
  assertion in the kernel-gated test.

- **Impact probability is not quadrature over the disc, and the obvious version
  fails exactly where this layer lives.** A well-determined orbit puts a 10 km
  ellipse inside an 11 311 km capture disc; radial nodes spread across the disc
  then sit tens of σ apart, step over the peak, and return 0.994 for an integral
  whose answer is 1 (observed, not hypothesised — it was the first failing test).
  Whitening by the covariance's Cholesky factor turns the Gaussian into a standard
  normal and the disc into an ellipse; in polar coordinates each direction meets
  that ellipse between the roots of a quadratic and the **radial integral becomes
  analytic**, leaving one periodic 1-D integral refined by doubling. The isotropic
  centred case then reduces to `1 − exp(−R²/2σ²)` and the tests check exactly that
  across four orders of magnitude of `R/σ`.

- **The ξ,ζ convention stays deferred, and this batch proves it can.** Under any
  orthonormal change of b-plane basis — rotation *or* reflection — the mean and
  covariance transform together and the capture disc, being centred at the origin,
  is invariant; so the probability is unchanged. Pinned by a test that includes
  the reflection a rotation-only test would miss. Keyholes are what will force the
  convention, because a resonant circle sits at a specific ζ.

- **The linearity check is deterministic, and its first metric was wrong.**
  Twelve `±3σ` principal-axis extremes, not a thousand random draws — most draws
  land near the middle where linearity was never in doubt, and the extremes are
  where it bends first. But normalising each sample by *its own* flown displacement
  reported **100 % bending** on a direction whose displacement was a few metres: a
  real ratio, a meaningless one. The report now judges the worst absolute residual
  against the *shell's* scale, and the same encounter reads **0.003 %** — 12 m of
  residual against a shell reaching 351 km.

- **The rock is synthetic, so the covariance is invented and says so.** There is no
  observation arc and therefore no honest orbit-determination covariance;
  `synthetic_along_track` borrows the *shape* (NEO uncertainty is overwhelmingly
  along-track, which is why a b-plane prediction is a narrow ellipse rather than a
  disc) and labels itself as invented, the same rule every drawn body follows. A
  real covariance arrives with a real asteroid from the SBDB, in the keyhole batch.

- **`sigma_distance` means the opposite of what it looks like.** The shipping
  campaign reports **8 196 σ** from the b-plane origin and an impact probability of
  **exactly 1** — no contradiction: the origin is Earth's *centre*, the thing that
  counts as a hit is an 11 312 km disc around it, and a sub-kilometre ellipse
  thousands of ellipse-widths off centre is still deep inside that disc. The doc
  now says so; quoting the σ-distance as "how many σ from a hit" inverts the answer.

- **It teaches the thing it was built to teach.** One rock, one trajectory, one
  encounter, and the Jacobian computed once and reused — `BPlaneSensitivity` splits
  the 13-propagation half from the free half, because "the same rock, better
  observed" is the entire Tier-3 comparison and must not cost 14 s a time.
  P(impact) runs **1.000 → 0.974 → 0.785 → 0.335 → 0.104** as the along-track σ
  widens from 1e-5 to 1e-1 m/s. Nothing about the asteroid changed; only how well
  anyone knows where it is.

- **The sampling plan is threaded, not re-derived — a fix to the same failure one
  level up.** `bplane_uncertainty_checked` originally called
  `uncertainty_sampling_plan()` a second time for the shell, while differencing the
  shell's flown displacements against a mean measured at the *first* plan's epoch.
  Both scans return the same `t_reduce` today and nothing enforced it; let them
  drift and the linearity report compares two different reduction epochs and calls
  the difference nonlinearity — precisely the failure this module exists to prevent,
  reintroduced by the code that checks for it. One plan now flows through both.

- **The reduction epoch's margin was measured by breaking it.** Moving the lead and
  watching the kernel-gated test: 12 d fails at 107 %, 48 h at 6.5 %, 30 h at 2.3 %,
  26 h passes clean. So the asymptotic invariance holds out to about a day and the
  shipping 12 h has ~2.5× margin. The test also gained a direct
  `∂r_p/∂v_along`-both-ways assertion, because the perigee agreement it had been
  checking is a *proxy* for the property the Jacobian rests on — though the same
  sweep shows the perigee gate is the one that trips first here, so the derivative
  gate did not prove tighter on this scenario. It stays because it checks the claim
  itself, and a faster encounter need not preserve that ordering.

- **What is not here.** The keyhole machinery itself and the ξ,ζ pinning it forces
  (now *scoped* — see below); real SBDB covariance ingestion (published in
  equinoctial or Keplerian elements, at their own epoch, in mixed units — a
  conversion to validate by round-tripping, not by inspection); a frontend. And the
  probability sweep above only *falls*, because the shipping nominal is a designed
  hit: showing it *rise* as observations accumulate needs a nominal miss, which is
  the deflected trajectory.

### Keyholes priced before designing them — 2026-07-28

`probe_keyhole_reach` answers the question that gates the whole keyhole batch —
*which resonant returns can this rock's flyby even reach, and is the keyhole wide
enough to aim at* — for the cost of one scenario build, before any of it is built.

- **The theory is derived, not transcribed.** The flyby turns the Earth-relative
  velocity through `δ` where `tan(δ/2) = μ⊕/(b·v∞²)`; rotating `Ŝ` toward `−B̂` by
  `δ` and adding Earth's heliocentric velocity gives the outgoing orbit, so
  `a'(b, B̂)` is closed-form and `a' = (h/k)^(2/3) AU` is the resonant locus. That
  *is* the analytical resonant-return theory, reached through the deflection this
  project already models — which means it can be checked instead of trusted.
  Incidentally it settles a convention by derivation: at incoming infinity
  `v ∝ (√(e²−1)/e)·[p̂ + √(e²−1)q̂]`, which normalises to exactly `geometry.rs`'s
  `s_hat`, so `Ŝ` is the incoming **direction of motion**.

- **The round-trip gate runs first and everything is downstream of it.**
  Un-rotated, `V⊕ + v∞·Ŝ` must reproduce the rock's real pre-encounter
  heliocentric `a`. Measured: **8.669e-5** relative (0.853937 vs 0.854011 AU),
  against a predicted approximation ceiling of 1.3e-4. A frame mix-up, an
  SSB-versus-Sun-centred slip, a flipped `Ŝ`, or a vis-viva error would all show
  up here at the percent level, so this is what licenses the sweep rather than a
  claim about it. The probe exits non-zero if it fails.

- **What the approximation costs, stated before the numbers are read.** Taking the
  encounter position as Earth's own costs `η ≈ 7e-5` in heliocentric radius, so
  `δa/a ≈ 1.3e-4`; over a 7-year return that is a ~12 h timing slip, during which
  Earth moves ~1.3e6 km — **about a hundred capture radii**. So absolute `a'`
  answers "which resonances are in band" and nothing else: it is ~4 orders too
  coarse to place the return on the disc and ~7 too coarse for a 600 m
  Apophis-class keyhole. But `∂a'/∂b` **is** trustworthy, because the `r ≈ R⊕ₒᵣᵦ`
  error is common-mode across neighbouring `b` and cancels to first order in the
  derivative. That is why the derivative, not the placement, is the deliverable.

- **The band is enormous and the count is not the interesting number.** From the
  grazing `b_capture` = 11 311 km outward, over the full 2π, `a'` reaches
  **0.687 .. 1.579 AU** against an incoming 0.854 AU — because at grazing this
  encounter turns by ~63°. **135 resonances** fall in band with returns inside
  1.5–20 yr. Sweeping the whole circle also keeps the result independent of the
  b-vector sign `geometry.rs` deliberately leaves unasserted.

- **The wide keyholes are the far ones, which inverts the intuition and is the
  single most useful thing measured here.** Near-grazing resonances have the
  steepest `∂a'/∂b` and are therefore the *narrowest*: 19:10 at 1.00–1.05 capture
  radii is **17 m** wide. The wide ones sit far out where the deflection is weak —
  3:4 at 5.4–13.6 capture radii is **11.6–24.9 km**. Since the keyhole width is
  exactly the accuracy encounter-1 must be placed to, **choosing a far resonance
  moves the required propagation accuracy from tens of metres to tens of
  kilometres.** A metre-level 12-year round trip is not happening; a 12–25 km one
  plausibly is. Any keyhole batch that had started at the grazing end would have
  spent itself discovering it could not verify anything.

- **The verdict: the shipping rock hosts a usable keyhole, so no new rock is
  needed.** The **3:4** resonance (`a' = 0.825482 AU`, 4 revolutions in 3 years)
  has its locus at `b` = 60 843–153 511 km, crossed by 26 of 72 directions, with
  `|∂a'/∂b|` = 1.74e-7–3.75e-7 AU/km and a keyhole **11.6–24.9 km** wide —
  **0.069–0.148 σ_b** against the σ_b = 168.7 × 0.8 km ellipse the existing Tier-3
  map produces for the invented along-track covariance. Inverting
  `b² = r_p² + 2μ⊕r_p/v∞²` gives a target perigee of **54 385 km (8.53 R⊕)**: a
  real miss, reachable by the existing `required_dv` solver. So the deflected
  trajectory — already owed anyway, because it is the only way to show P *rise* —
  can be dialled onto the resonance, and the purpose-built threat orbit and the
  Apophis oracle both stay out of this batch.

- **The scan gate picks the winner, and it is not `ScanOptions::default()`.** The
  scenario ships `max_distance: Some(5.0e8)` — a **500 000 km** gate — and the test
  that matters is on the *perigee* a locus implies, not on its impact parameter,
  because that is what the scan reports an approach by. 3:4's locus implies
  perigees of 54 385–146 822 km and clears the gate with room to spare. **7:9,
  which has the wider keyhole (48.7–131.5 km, 0.29–0.78 σ), reaches perigee
  549 389 km and is partly invisible to the pipeline that would have to find it**;
  18:23 is worse. So 3:4 is not merely a good candidate, it is the widest keyhole
  that fits entirely inside the gate — and a keyhole nothing can detect is not a
  keyhole. The probe mirrors the shipping gate as a named constant and warns when a
  locus leaves it; censusing with `Default` (no gate at all) would have hidden
  this.

- **A width that diverges is flagged, not quoted.** `Δb = Δa'/|∂a'/∂b|` is a
  linearisation and blows up where the locus is *tangent* to a level set of `a'`.
  Five resonances (15:19, 19:24, 4:5 among them) span >100× in `|∂a'/∂b|` across
  directions and so contain such a tangency; 15:19's "1801 km" keyhole is that
  artifact, not a measurement. The probe prints a `NEAR-TANGENCY` warning on them
  and the good candidates (ratios 1.9–2.7×) are untouched by it. The keyhole-width
  tolerance itself — the `Δa'` that slides the return by one capture diameter, via
  `Δt = h·yr·1.5·Δa'/a'` and `V⊕·Δt` — is an order-unity definition, stated as one
  so it can be argued with rather than assumed.

- **The rotated branch is measured, not assumed — and the b-vector sign is now
  pinned by experiment.** The reach probe's round trip only exercises `δ = 0`; it
  cannot see a sign error in `Ŝ_out = cos δ·Ŝ − sin δ·B̂`, because `geometry.rs`
  leaves `b_vector`'s sign unasserted and a full-2π sweep is symmetric under
  `B̂ → −B̂`. A flip changes neither the band nor `|∂a'/∂b|` — it **mirrors the
  locus**, corrupting exactly the per-resonance `b_res` the batch depends on. So
  `probe_keyhole_rotation` solves the impulse for a 21 R⊕ miss (**Δv = 0.205 m/s**
  at 12 years' lead, a 190 s bracket-and-bisect), flies it, and predicts the
  post-encounter heliocentric `a` both ways against the flown truth read 45 days
  past the encounter: **`−B̂` matches to 1.518e-4 and `+B̂` misses by 7.428e-2, a
  489× separation.** The branch stands, in magnitude as well as sign, and the
  convention is now measured on a real flyby rather than adopted.

  Two things that came out of it worth keeping. The wrong branch predicted
  **0.8231 AU against the 3:4 resonance's 0.8255 AU** — a flip does not produce
  nonsense here, it produces a plausible near-hit on the very resonance being
  targeted, which is exactly why guessing was not an option. And the sign is not
  cosmetic: `−B̂` *raises* `a'` (0.854 → 0.889) while `+B̂` lowers it, and 3:4 sits
  on the lowering side — so reaching it needs a deflection of the **opposite
  sense** to the along-track impulse, not a bigger one. `required_dv` targets a
  perigee along a single direction and so controls `|b|` but not `B̂`; hitting a
  resonance needs both knobs, and that is the targeting step.

- **A dormant landmine, located and defused.** The census finds exactly **1** close
  approach inside the gate over the nominal span. `nominal_encounter_epoch` reduces
  at the *minimum-distance* approach and `uncertainty_sampling_plan` anchored
  `t_reduce` to it — so the moment the span is extended past a **deeper** second
  encounter, which is the entire point of a keyhole, that anchor silently relocates
  and every Jacobian column describes a different encounter than the caller asked
  about, **without erroring**: the matrix stays finite, symmetric, and plausible.

  `uncertainty_sampling_plan` now censuses with `find_close_approaches` and anchors
  to the **first** encounter explicitly — identical behaviour today, since first and
  minimum-distance are the same when there is one — and **refuses** if the span
  holds more than one, naming the epochs and distances it found. That is the honest
  answer rather than a cleverer guess: with two encounters in span, *which* one the
  covariance maps to is a question only the caller can answer, and a chained
  two-encounter Jacobian is not defined here yet. `nominal_encounter_epoch` keeps
  its min-distance meaning for its ~30 other callers, who do want the closest pass.
  `the_tier3_reduction_epoch_anchors_to_the_first_encounter_and_refuses_a_second`
  pins the equivalence and is deliberately a tripwire: when it fails, the message is
  "go decide which encounter the map is about," not "the plan broke."

- **One worry that dissolved on inspection.** `Clock::state_at` evaluates DOP853
  **dense output** over the integrator's own adaptive sub-steps, not linear
  interpolation between the 10-day snapshots (`clock.rs:208`, and
  `dense_subsnapshot_beats_linear_interpolation` measures the gap at >1e4×). So
  propagating *through* encounter 1 is not a correctness problem: the integrator
  densifies through the flyby on its own, and the 10-day cadence remains what it
  already was — the cadence a Jacobian's columns converged at, owed a
  re-measurement across a deep flyby but not a landmine.

### Keyholes, closed — 2026-09-02 session (the Öpik frame pinned, the circles in closed form, and the 3:4 keyhole flown to an impact)

The reach and rotation probes had priced the keyhole batch; this session built
it, and the closed form corrected one of the numbers those probes recorded.

- **The sign is derived, not only measured.** The hyperbola's centre `C = a·e·P̂`
  lies on the incoming asymptote, so the asymptote's closest point to Earth is
  `C − (C·Ŝ)Ŝ = a·e·P̂ − a·Ŝ = b·(Ŝ × ĥ)` — exactly the `B` `geometry.rs` builds.
  So `B` points from Earth's centre *at the asteroid's incoming line*, gravity
  bends toward `−B̂`, and the rotation probe's 489× measurement is the same fact
  seen from the other side. `b_vector_points_at_the_incoming_asymptote` pins it:
  every inbound sample has `r·B̂ > 0` and `r·B̂ → b` on the asymptote.

- **The ξ,ζ convention is settled by an identity, and the identity is a test.**
  `core/src/keyhole.rs`: `η̂ = Ŝ`, `ζ̂` anti-parallel to Earth's velocity projected
  on the b-plane, `ξ̂ = η̂ × ζ̂`. Under exactly those signs the rotation
  `Ŝ_out = cos δ·Ŝ − sin δ·B̂` dotted with `V̂⊕` reproduces Valsecchi's
  `cos θ' = [(b² − c²) cos θ + 2cζ sin θ]/(b² + c²)` term for term
  (`closed_form_equals_the_rotation_construction`, 200 random geometries to
  1e-12). Level sets of `a'` are then circles centred on the ζ-axis,
  `ζ_c = c sin θ/(cos θ' − cos θ)`, `R = c|sin θ'|/|cos θ' − cos θ|`, with an
  analytic gradient pinned against central differences and shown normal to the
  circle. `perigee_state_for_asymptote` inverts the reduction (the state a
  targeting step will need) and round-trips to 1e-10. **+ζ always raises `a'`**
  — bending toward Earth's motion adds heliocentric speed — which is the
  structural form of what the rotation probe found by flying.

- **The correction.** `probe_keyhole_reach` reported the 3:4 locus at
  `b = 60 843–153 511 km` and a target perigee of 54 385 km. The exact circle
  (`ζ_c = −78 704 km`, `R = 74 837 km`) runs from **3 866 km to 153 540 km**: the
  far end was right, the near end was not, because the probe bisected outward
  from `b_capture` and a ray that crosses a circle twice, or enters it inside the
  disc, showed it no sign change. So the resonance passes *through* the capture
  disc — as do all 27 resonances with `h ≤ 7` — and the nearest *miss* on it is
  the grazing point, where the keyhole is **0.18 km** wide (the far end's
  24.92 km matches the probe's 24.9). `ResonantCircle::intersections_at_radius`
  gives the grazing points; `crosses_capture_disc` says so.

- **The map, as data and as pictures.** `probe_keyhole_map` (~50 s) writes
  `docs/keyhole_map.json` — frame, capture disc, the Tier-3 ellipse rotated into
  ξ,ζ (major axis **89.7° from ξ**: along-track uncertainty is timing
  uncertainty, and the picture finally says so), 168 circles within 60 capture
  radii, and two deflections flown in the real field: **+0.2 m/s → a' 0.8899 AU
  closed form vs 0.8900 flown (1.26e-4); −0.2 m/s → 0.8233 vs 0.8233 (2.14e-5)**
  — the rotation probe's check on both ζ sides for two propagations instead of a
  190 s solve. `tools/keyhole_map_svg.py` and `keyhole_map_html.py` (standard
  library only) render it; the SVG is in the README, the HTML is interactive.

- **THE 3:4 KEYHOLE, FLOWN.** `probe_keyhole_return` (252 s) aims with the closed
  form — the circle point at the nominal's own ξ, converted to a perigee,
  `required_dv` retrograde → **0.216438 m/s** — flies the deflection, hands off
  30 d past the flyby, flies 3.6 more years, and censuses Earth approaches
  inside 0.05 AU. The aimed shot returned at **53 841 km** on 2042-12-31, about
  five capture radii: the module doc's "~100 capture radii" bound on the
  `r ≈ R⊕ₒᵣᵦ` approximation was conservative by 20×. The return miss is
  V-shaped in Δv (Earth's motion over the timing slip), so eleven golden-section
  steps found the floor: **Δv = 0.216550 m/s → 1 130 km from Earth's centre,
  2042-12-31T14:33 TDB, 3.00 yr after the 2040-01-01 flyby.** Inside the disc;
  inside Earth. The keyhole is an impact keyhole, measured. Its Δv window at the
  floor, ~1.3e-5 m/s, is ~13 km of b at the ~1e6 km/(m/s) leverage of a 12-year
  lead — the same order as the closed-form width. And the corollary the thesis
  now carries: a 0.2166 m/s nudge that turns a certain 2040 impact into a
  comfortable 23 R⊕ miss produces a certain 2042 impact instead. *Which* miss
  matters. `the_three_four_keyhole_returns_the_rock_to_earth_when_flown` re-flies
  that Δv (26 s) and asserts a return inside four capture radii at 2.9–3.1 yr.

- **The frontend draws the map, unrun.** The binding's `encounter_basis()` is now
  the core's Öpik frame (fallback to the ecliptic-pole basis, reported by
  `bplane_frame_pinned()`), `keyhole_circles(max_years)` hands GDScript the rows,
  `encounter.gd` draws circles and widths under the disc and `[H]` toggles them
  (`encounter_keyholes`, keycode 72). The binding's kernel-gated test pins that
  `ζ̂` opposes Earth's motion, that a *retrograde* plan lands on `−ζ`, and that
  3:4 is listed through the disc at ~153 500 km. **No Godot ran in this session:
  the GDScript is written against the tested binding and has not been seen on
  screen.** The `_shot.gd` autoload is the way to look, per the visual-layer
  memory.

- **Engineering, because a fresh clone could not run the physics.** The session's
  proxy blocked NAIF and nyx-space; the kernels came from the `naif-de440` PyPI
  wheel (the full `de440.bsp`, which the resolver accepts) and the ANISE
  repository's LFS media endpoint for `pck11.pca`. `tools/fetch_kernels.py`
  encodes those fallbacks with magic-byte checks; `.github/workflows/ci.yml` runs
  fmt, clippy and the kernel-free suite, then fetches, caches and runs the core
  physics with `ASTEROID_REQUIRE_KERNELS=1`; `DEVELOPING.md` holds the commands;
  the README's status ("Early — the physics core is taking shape") was three
  phases stale and is rewritten. Twenty-six files were out of `rustfmt` shape
  and were formatted in a commit of their own. **Not run here:** the three
  Horizons `.neo` tests (the JPL API was blocked too; they fail under
  `REQUIRE_KERNELS` on this box only). Baseline before the batch: 211 core tests
  green with kernels (plus the three Horizons tests that cannot run here); after: 225.

### The frontend measured and given its phosphor — 2026-09-05 session (frame times, the caches they justified, the world in its own viewport, and the keyhole map finally seen)

The brief was "improve visuals and performance". The rule this project has
always run on applied twice over: **measure first**, then change only what the
measurement names — and look at the picture, because a frame that is fast and
wrong is worse than a slow one. Everything below is verified by a number or a
PNG, and the harness that produced them ships.

- **A frame-time harness, because a per-frame cost has no other symptom.**
  `godot/tests/_perf.gd` drives every view (3D at three framings, the 2D map,
  the b-plane with and without zoom, the launch-window map, max warp) for 240
  frames each and prints wall-clock frame time, **native binding calls per
  frame** (`Sim.ffi_calls`, a new counter reset at the top of `Sim._process`),
  and draw calls; plus a microbenchmark of the raw lookups. Two lessons cost an
  hour each. `Performance.TIME_PROCESS` reported hundreds of milliseconds beside
  an 8 ms frame on this build — dropped from the report rather than explained.
  And windowed, every view sat at exactly **8.41 ms (119 fps) regardless of
  content**: the desktop compositor pins frames to the monitor even with vsync
  off, so a windowed run only shows a view *slower* than the display, and the
  CPU cost is read `--headless` (the draw paths still run for visible nodes).

- **What the numbers said.** Headless, before: the 2D map **18.9 ms/frame with
  764 native calls**, every other view 7–10 ms with 47–61 calls. The map's
  `_orbit_trace` re-walked six orbits from the ephemeris *inside `_draw`*,
  every frame, for polylines that never change — 2.7 ms per planet trace
  (measured), ~13 ms of the frame. The 3D view asked the same body positions
  two and three times a frame across the scene, the tag layer and the HUD.
  Fixes, each sized by that: **cached orbit traces** in `map2d.gd`, dropped
  on `mission_ready` (all) and `plan_changed` (the deflected arc only); a
  **per-frame memo** in `Sim.pos_ecl` keyed by body name and exact epoch,
  cleared each `_process`, with the orbit walks routed around it
  (`_lookup_ecl`) so they cannot evict the clock's answer; the impact point
  read once on install (`Sim.impact_point_ecl`) instead of twice a frame; the
  hidden 3D scene's `_process` switched off while a 2D view is up. After: the
  map **7.1 ms, 29 calls**; the 3D views 6.9–7.4 ms, 29 calls; everything else
  unchanged within the noise of a machine that was running two other projects'
  Godot tests at the time.

- **The 3D world moved into its own viewport, twice removed.** `main.gd` now
  builds `world_vp` (the 3D scene, `own_world_3d`, **4× MSAA** — every body and
  orbit is a one-pixel line and un-antialiased they crawl under camera motion)
  as a child of `persist_vp`, a 2D viewport that is **never cleared**, whose one
  ColorRect runs `shaders/phosphor_persist.gdshader`; the main viewport shows
  that as a `TextureRect` under the HUD, and the CRT shader still governs the
  whole screen. The camera rig stays in the main viewport for input ordering
  (the encounter view swallows the wheel before it) and drives a `Camera3D`
  that lives in the world by global transform (`attach_camera`). `_show_view`
  disables both render targets while a 2D view is up and clears the
  accumulation once on return. The first persistence shader was the textbook
  exponential average, `world·(1−keep) + previous·keep`, and the max-warp
  screenshot showed why that is wrong for this: a body that moves more than its
  own width per frame is drawn at `1−keep` ≈ 6 % and *vanished*, leaving tags
  pointing at nothing. It is **peak-hold with decay** now — `max(world,
  previous·keep)`, read back through `hint_screen_texture`, `keep =
  exp(−Δt/0.14 s)` — so the moving thing is at full brightness and its past
  fades behind it, and static line work is exactly its own brightness. The
  target is HDR because in 8-bit `v·0.9` rounds back to `v` below ~4/255 and
  trails never reach black. `_shot.gd` gained `trails_1_max_warp` for it.

- **The keyhole map ran for the first time, and its first picture was a
  thicket.** All 27 circles with `h ≤ 7` came out at one weight with a caption
  each, ~30 captions stacked down the ζ axis through Earth, the b-point and the
  impact mark. `encounter.gd` now brightens each circle by its keyhole width on
  a log scale (3:4 solid), and captions only the **seven widest in frame**
  (`KEYHOLE_LABELS`), the 3:4 always among them — zoom in and the budget goes
  to whatever is widest at that span. `[H]` verified as a toggle
  (`enc_7_keyholes_off`). The physics drawn is unchanged.

- **The run ritual is a script now, and it found two traps.** `godot/tests/
  run_harness.ps1 -Harness _shot.gd [-Headless]` registers the autoload after
  `Sim`, launches, restores `project.godot`, and kills **only the PID it
  started**. Trap one: Godot on this machine does not always exit on `quit()`
  — the plugin's teardown left three processes alive at 100 % CPU over the
  session, one of which **held the debug DLL** so `cargo build -p
  asteroid_gdext` failed with `Access is denied (os error 5)` while my wait
  loop reported success, and Godot ran the July binding: `Nonexistent function
  'bplane_frame_pinned' in base 'Mission'` (staleness trap 1 again, by a new
  route). Trap two: other projects' Godot processes were running on the same
  box; killing by name would have taken them — kill only your own PID.
  Non-ASCII in a `.ps1` is a parse error under Windows PowerShell 5.1 (a curly
  dash decodes to a quote), so the script is plain ASCII.

- **One test de-raced.** `test_orrery.gd` asserted the build ran off-thread by
  `polls > 1`; on the loaded machine the checks before the poll loop outlasted
  the worker, the first poll found the build landed, and a correct build
  failed as "blocking". It now asserts what the caller paid (`_ready()` under
  3 s — measured 22 ms — against a build that took 5.9 s to land), and reports
  the polls as information. 83/83.

- **Cosmetic, but honest:** body spin is per second of wall time, not per frame
  (it ran twice as fast on a 120 Hz display).

Follow-ups, sized and ordered for a smaller model, are in
`docs/plans/2026-09-05-visuals-performance-followups.md`. **Tasks 1, 2 and 7 are
done** (the startup wait, the batched position lookup, and the Tier-3 ellipse) —
though tasks 1 and 2 did not land as that document describes them, and it has now
been wrong about the code on three separate premises; read *Frontend speed* before
trusting any mechanism it states. What is left: label de-collision in the tag
layer and on the keyhole map, a persistence control, and polyline drawing for the
encounter tracks.


### Keyhole targeting, and the corollary on the panel — 2026-09-05 session (aim at any resonance, fly it, read the return in its own frame, and print how many doors away the plan is)

The keyhole batch (2026-09-02) proved a keyhole exists and flew one: the 3:4
resonant return, hit at a retrograde Δv of 0.216550 m/s, coming back *inside
Earth* three years after a flyby that missed by 153 000 km. But that shot lived
in an example, hardcoded to one resonance and one branch, and nothing on screen
said a word about it. This batch turns it into an API and puts its answer in
front of the operator.

The thesis this project exists for is *deflect early*. Its corollary is
**a miss can be worse than a hit if it is the wrong miss** — and until now the
planner could print `VERDICT: MISS - EARTH CLEAR` over a plan that had parked
the rock on a resonant circle, with nothing to say so.

#### The readout was separable from the targeting, and shipped first

The roadmap phrased this as "generalise the API, *then* add a planner readout",
which implies a dependency that is not there. The readout needs the **circle
geometry** — closed-form, already computed, already drawn — not the four-minute
aim-and-refine solve. Distance from a b-point to a circle centred at `(0, ζ_c)`
is `| |p − (0,ζ_c)| − R |`; comparing it against the keyhole width at the
nearest point costs microseconds. So the panel row shipped without waiting for
the flying.

- `core/src/keyhole.rs` gained the geometry: `ResonantCircle::closest_point_to`
  and `signed_distance` (**positive outside**), and `OpikFrame::nearest_keyhole`
  / `tightest_keyhole` over a caller-supplied census.
- **Two rankings, because they disagree.** "Nearest in kilometres" and "nearest
  in keyhole *widths*" are different questions: a wide far keyhole 200 km away is
  a likelier return than a 25 km one 50 km away, and this project already
  measured that **the wide keyholes are the far ones**. Both are returned; the
  panel headlines the widths one. A test asserts the two selectors actually
  disagree on some geometry — if they never did, one of them is dead code.
- **The census is the caller's, not one taken inside.** The readout is handed the
  same circle list the b-plane view draws, so a panel can never name a resonance
  the map beside it does not show. `Sim.KEYHOLE_MAX_YEARS` is now one constant
  for both.
- **The truncation is reported, not papered over.** `keyhole_circles` stops at 60
  capture radii (≈ 678 000 km). A plan beyond that gets
  `beyond_mapped_region`, and the panel says so instead of quoting the nearest of
  a truncated list.
- **A clean miss has no b-point at all** — the pass left the 1.3 LD scan gate, so
  there is nothing to place. The row says `UNMEASURED - PASS LEFT THE ENCOUNTER
  GATE`, and the note underneath says why that is not reassurance: *the wide
  keyholes are the far ones*, and a pass that far out flies past the widest doors
  on the map with nothing measuring it.

#### What the readout measured, and why the panel prints a number instead of a verdict

Fed the **flown** keyhole plan (Δv 0.216550 m/s at campaign start — the one that
returns inside Earth), the readout says: **20.4 km from the 3:4 circle, against a
24.9 km keyhole width. 1.64 half-widths. `inside = false`.**

So the closed form calls that plan *outside* a door the rock demonstrably flies
through. That is the order-unity slack `keyhole.rs` warned its own width
definition carries, now **measured at about 1.6×** on the one case with an
answer. The consequence is a UI decision, not a bug fix: **the panel prints "N
widths off", never a yes/no**, because a binary would say NO to a plan that comes
back. Both `keyhole.rs` and the binding test now record the number so it is a
measurement rather than an assumption.

#### The targeting API, generalised

`core/src/keyhole_target.rs` is the three steps as callable pieces:

1. **`aim_at_resonance`** — closed-form, instant. Picks the circle point at a
   held ξ on either **branch** (`CircleBranch::Minus` / `Plus`, the two ζ
   crossings) and converts it to the perigee `required_dv` targets.
   "Either ξ side" turned out to be two different questions and both are carried:
   the branch, and the sign of ξ (which is a property of the deflection
   *direction*, prograde or retrograde, and is measured rather than chosen).
2. **`fly_keyhole_shot`** — re-fly, reduce encounter 1, hand off 30 days past it,
   propagate `h + 0.6` years (the span now follows the resonance instead of the
   3:4's hardcoded 3.6) and census with a wide 0.05 AU gate.
3. **`solve_keyhole_return`** — bracket and golden-section on the return miss.

**Reachability is a returned error, not a NaN.** The probe's `√(R² − ξ²)`
survived only because the shipping geometry happened to fit; `points_at_xi`
returns `None` when the line misses the circle, and `aim_at_resonance` turns that
into `XiOffCircle` — refused in microseconds instead of poisoning a perigee, a
Δv solve and four minutes of flying.

#### The check that says whether a "physical floor" is really one

The refinement's minimum is not zero, and the module claims what remains is the
post-encounter orbit's **spatial** offset — which no timing change can remove.
That claim is unfalsifiable from the scalar return distance, so the return is now
reduced in **its own** Öpik frame (Earth's state at the *return* epoch, not
encounter 1's), where the axes mean something again: `ζ` is timing, `ξ` is the
orbit-to-orbit offset. Δv is a timing knob, so a converged search must drive
`ζ₂ → 0` and leave the floor in `|ξ₂|`.

**Measured at the flown floor: ξ₂ = 4 006 km, ζ₂ = −26.6 km — 99.3 % spatial,
`|ζ₂|/|ξ₂| = 0.007`.** Converged. A floor sitting in `ζ₂` would have been an
unconverged golden-section wearing a physics costume, and it would have looked
identical in the one number the old probe printed.

> Those two numbers read 4 014 / 891 km until 2026-09-06, and the ratio 0.222.
> That was a real flight of a *rounded* Δv at a *12-iteration* stop — see
> *The floor that was a stopping distance*. The claim is unchanged and the
> evidence for it got much stronger.

One number to keep straight while reading those: the return's **impact
parameter** is 4 006 km while its **geocentric closest approach** is 1 087 km.
The gap is gravitational focusing — the same pair `geometry.rs` documents for the
first encounter, and the same mix-up that once printed SURFACE IMPACT over a
working deflection.

#### On the panel, and the sweep that nearly drew the wrong conclusion

The planner grew two rows: the keyhole line and the caveat under it. Both come
from `Sim` formatters for the reason the miss does — the "no b-point" case is a
*success* here too, and formatting it at the draw site would re-open that trap.
GDScript still owns zero orbital mechanics; it prints what the binding returns.

The shot harness first swept ten impulses at the longest lead the planner allows
and reported the closest a player could get as **~5 000 km** from any circle —
every rung reading CLEAR. That reads as "keyholes are unreachable from the
frontend", and it is **an artifact of the step size**: as the impulse is dialled
up the b-point walks outward *past* one resonant circle after another, so
consecutive rungs bracket crossings the sweep steps over. Refining inside the
best bracket found it immediately:

> **Δv 0.930 m/s at 900 days of lead → 2.9 widths off the 3:4 circle (36 km),
> while the verdict above it reads `MISS - EARTH CLEAR` at 0.40 LD.**

That is the corollary on screen, dialable with the panel's own keys
(`godot/tests/_shot.gd` → `enc_8_planner_keyhole`). The harness keeps the
bracket-then-refine so the picture stays reachable when the scenario moves.

The alert threshold is **4 half-widths**, calibrated on the flown door rather
than picked: the one keyhole known to return sits at 1.64, so the band worth
shouting about is a small multiple of that — not a distance in kilometres, which
means different things at a 25 km door and a 25 000 km one. The row blinks on
that band, deliberately **not** on the core's `inside` flag, which would stay
silent for exactly the case it exists to warn about.

#### Which frame the readout's circles live in — asked, then measured

The readout places the **deflected** b-point on the **nominal** encounter's
circles, so the panel and the drawn map can never name different resonances.
`keyhole_target.rs` argues the opposite for a *return* — rebuild the frame,
because a different asymptote means the old axes carry no meaning — and the two
look inconsistent until the scales are compared. A resonant circle depends on
the encounter only through `c = μ⊕/v∞²` and `θ`, and a small along-track nudge
years out moves *where* the rock arrives enormously while barely changing how
fast or from what direction. A return three years later is a different
encounter; a nudged version of the same flyby is not.

That is an argument, and 20.4 km against a 24.9 km door has no room for
arguments, so it is now a measurement the binding test prints and asserts:

> `v_inf` **7 633.2** m/s nominal vs **7 635.6** m/s deflected (+3.1e-4), which
> moves the 3:4 circle **1.5 km in centre and 1.6 km in radius** — ~12 % of the
> door, ~15 % of the reported distance.

Small enough that the shared frame stands; **not** small enough to call
negligible, so the test fails if it ever reaches a quarter of the keyhole width.

#### An operational bruise worth recording

`ASTEROID_REQUIRE_KERNELS=1 cargo test --workspace --release` — the command
`DEVELOPING.md` gives — **crashed** on the way through the binding suite:
`memory allocation of 645727232 bytes failed`, then
`STATUS_STACK_BUFFER_OVERRUN`. Thirty-four kernel-gated tests each build a full
scenario (an ephemeris plus several dense-output clocks) and the default
parallelism runs them all at once. `--test-threads=2` passes (34/34, 853 s) and
is now what `DEVELOPING.md` documents. A re-run at default threads later passed,
which proves nothing: an intermittent OOM is still an OOM, and CI's runners have
less memory than this box. (CI itself is unaffected — its physics job runs only
`-p asteroid_core -p validation`.)

#### Traps recorded

- **A coarse sweep over a monotone knob is not a survey of a non-monotone
  quantity.** Distance-to-nearest-circle is a sawtooth in Δv; ten rungs sampled
  its peaks and concluded the floor was 5 000 km when it is 36 km.
- **`inside` is a linearisation, and a conservative one.** Never render it as a
  verdict; render the ratio.
- **Two b-plane points, one epoch.** `deflected_b_point_km` rescales its
  magnitude so the drawn mark can never contradict the verdict's `|B|`. That
  rescale is < 0.01 %, invisible on screen and *kilometres* at these radii — the
  same order as a keyhole width. The keyhole readout takes the **raw**
  projection; the picture keeps the rescaled one.
- **A converged search and a search stuck on its own bound are the same
  picture.** Golden-section against a wall reports a *shrinking* interval, so the
  tightness metric moves the wrong way. Report whether the minimum was ever
  bracketed; never infer it from the window.
- **A step that adds a constant is not a search that widens.** `hi = mid + (mid −
  lo)` reads like it grows and reaches only `f·(n+1)`. Doubling reaches
  `f·(2ⁿ⁺¹−1)` for the same number of flights — but only if each branch doubles
  the gap it is **extending**. Doubling the *trailing* gap grows as `2^(n/2)` and
  is indistinguishable by eye.
- **A test that asserts a formula against a literal tests nothing.** It passes
  with the code it describes deleted. Run the loop over a synthetic objective and
  measure where it ends up.
- **A parameter validated on one case is a parameter validated on zero cases.**
  Both census gates and the widening reach were constants that happened to fit
  the 3:4, and all three broke on the first other resonance tried.
- A 45-character caveat does not fit the planner's value column (the first cut
  printed `CONFIRMS` on top of the panel border). It is indented under the label
  column instead.

#### The second resonance broke it three ways — 2026-09-06

The API above was written to generalise past the 3:4 and validated *on* the 3:4,
which is not the same thing. Pointing it at **7:9** — the obvious next thing to
try, a far circle with a wide door — failed three times, and none of the three
failures announced itself as a bug. Two were constants that were only ever right
for the resonance the code grew up on.

**1. The first-encounter census gate is sized for a rock that hits.** The
shipping scan gate is 5×10⁸ m, which is generous for a threat whose whole
problem is that it arrives at Earth. A keyhole aim is the opposite: it flies
*far* out on purpose, and this project already measured that **the wide keyholes
are the far ones**. The 7:9 aim wants `b ≈ 5.2×10⁸ m`, just past the gate, so the
flight returned `NoFirstEncounter` for a flyby that was there the whole time.
`KeyholeShotOptions::widened_for_aim` now raises the gate to `max(gate, 2·b)`
before the first flight. Widening is safe for an argmin — a wider census can only
admit approaches that were being rejected, never move a minimum already inside —
and it is one-way, so a caller who set a *wider* gate keeps it.

**2. The return census gate has to grow with the return.** It was a constant
0.05 AU, wide enough that it read as paranoid. It is not a taste and it is not a
margin: the closed form misplaces `a'` by some relative δ, that becomes a period
error `ΔT/T = 1.5δ`, and over an `h`-year return the arrival slips by
`h·yr·1.5·δ` while Earth keeps moving at ~30 km/s. **The placement error is
linear in `h`**, so a constant gate is correct for exactly one resonance.
Measured: the flown 7:9 `a'` came out `9.3e-4` from the resonance — seven times
the `1.3e-4` the module doc quotes from the 3:4, and the far circles are where
that matters — which over seven years is ~3.6 days of slip, **~9.2×10⁶ km** of
Earth motion against a `0.05 AU` = `7.5×10⁶ km` window. *Every* flight in the
search reported "no return" for a return sitting just outside it. The gate is now
`h × RETURN_GATE_PER_YEAR_M`, and that constant is defined as `0.05 AU / 3` so
that at `h = 3` it reproduces the exact width the flown 3:4 result ran at — the
generalisation costs the one answer this project has nothing.

**3. A probe impulse that fails to fly must score, not abort.** The refinement
called the flight directly, so any Δv in the bracket that left no encounter
inside the gate killed the whole solve with an error instead of being scored as
"infinitely bad and move on". Golden-section handles an infinite sample fine; it
cannot handle an exception. The flight is now wrapped so an unusable Δv returns
`None` → `f64::INFINITY`, while the **aim itself** and the **final best** must
genuinely fly — scoring those as infinity would hand back a "solution" that never
happened.

#### The convergence check earned its keep on the first resonance it was pointed at

The ξ₂/ζ₂ split was added as a way to falsify the module's own claim about the
3:4 floor. On the 3:4 it confirmed it, which is the weak kind of evidence. The
strong evidence is what it does *across* a solve — aim versus refined floor:

| 3:4 | ξ₂ (spatial) | ζ₂ (timing) | ratio |
| --- | --- | --- | --- |
| closed-form aim, Δv 0.2164375000 | 3 549 km | −60 185 km | 16.96 |
| refined floor, Δv 0.2165483096 | 4 006 km | −26.6 km | **0.007** |

Timing falls by a factor of 2 261; the spatial offset moves 13 %. That is exactly
the signature "Δv is a timing knob" predicts, and it means the ratio is a real
convergence gauge rather than a number that happens to be small.

Pointed at 7:9, the same gauge says the opposite. The search stopped at a
3 924 232 km return miss made of `ξ₂ = 129 230 km` and `ζ₂ = −3 928 736 km` —
**30.4× more timing than spatial** — and the old probe printed that as "the
spatial offset, which no timing change removes", which is simply false. Δv is a
timing knob and the residual was almost all timing, so there was impulse left to
spend.

#### The gauge said "not a floor". Believing *why* took one more measurement.

The obvious reading was "the golden section ran out of iterations", and it was
wrong. Re-running 7:9 at **30** iterations instead of 12:

| 7:9 | 12 iterations | 30 iterations |
| --- | --- | --- |
| floor Δv | 0.772255 | 0.772272 |
| return miss | 3 924 232 km | 3 922 376 km |
| `ζ₂`/`ξ₂` | 30.40 | 30.43 |
| Δv window | 4.48e-5 | **5.32e-8** |

Eighteen extra flights shrank the Δv window by a factor of 842 — golden-section
doing exactly its job — and moved the answer 1 856 km out of 3.9 million, 0.05 %.
**Iterations were never the constraint.** Golden-section narrows a bracket; it
does not move one.

The real fault is in the bracketing, and it is arithmetic anyone can check:

> The widening step was `hi = mid + (mid - lo)` — **a constant step, not a
> doubling one**. Six widenings of 1 % therefore reach `aim × 1.07` and no
> further. The 7:9 aim is 0.721750; `0.721750 × 1.07 = 0.772272`. **That is the
> reported "floor", to every digit printed.**

The search converged onto the **upper wall of its own search interval**. And the
one number that should have exposed that — the Δv window — pointed the other way:
golden-section squeezing against a bound produces a *vanishing* window, which
reads as a tight, well-converged answer. A wall and a floor were indistinguishable
in every number the API returned.

The 3:4 never showed it because its aim is already right: it walks 5e-4 from aim
to floor, comfortably inside the very first ±1 % bracket, so the widening loop
never ran a single step. **The bug was unreachable from the only case the code
had ever been run on.**

Three changes:

- **The widening step doubles.** `KeyholeRefineTol::reach_fraction()` is now
  `f·(2ⁿ⁺¹ − 1)` = **127 %** of the aim at the defaults, against the old
  `f·(n + 1)` = 7 %. Same six widenings, same cost when they are not needed.
- **`KeyholeSolution::bracketed` says whether the minimum was ever actually
  enclosed** — a centre strictly finite and no worse than both ends. If it is
  false, `best` is a wall and the caller is told so instead of inferring it from
  a suspiciously small window.
- **The probe leads with it.** `!! THE SEARCH NEVER BRACKETED THE MINIMUM` prints
  above everything else, because every number underneath is a number from the
  wall.

A test pins the coincidence rather than the fix, since the coincidence is the
evidence: 7:9's walk from aim to reported floor is 6.99993 %, and the old reach
was 7.00000 %. They are not merely close — they agree to five figures, because
one *was* the other.

#### What was behind the wall

| 7:9 | on the wall | with a bracket that reaches |
| --- | --- | --- |
| floor Δv | 0.772272 (= the bound) | **0.810311** |
| return miss | 3 922 376 km | **46 608 km** |
| `ξ₂` spatial | 129 230 km | −52 998 km |
| `ζ₂` timing | −3 928 736 km | **−0 km** |
| ratio | 30.43 | **0.000** |
| bracketed | no, and it never said so | yes |

**The wall was 84× worse than the answer** — and it was reported with a Δv window
of 5.3e-8, the tightest-looking number this project has produced.

At the real floor the timing coordinate is driven to *zero*, which is exactly
what "Δv is a timing knob" predicts of a converged search, and it means the
sentence the probe prints — "the post-encounter orbit's spatial offset at the
return, which no timing change removes" — is now **earned** rather than asserted.
The 3:4 reproduces its pushed result to every digit (aim 0.216438 → 53 841 km;
floor 0.216550 → 1 130 km; resonant-return impact; 18 flights), because its
widening loop still never runs a step.

#### The fix was wrong the first time, in the same way

The first version of the doubling step read the **trailing** gap rather than the
leading one:

```rust
// right branch
let step = 2.0 * (mid - lo);   // the gap being left behind
```

which still grows — just as `2^(n/2)`. With `g = mid − lo` and `h = hi − mid`, the
shift makes `gₖ₊₁ = hₖ` and `hₖ₊₁ = 2gₖ`, so `h` only doubles every *second*
widening. The walk goes `w, 3w, 5w, 9w, 13w, 21w, 29w`: **reach 0.29, while
`reach_fraction()` said 1.27.** Reading the leading gap (`2.0 * (hi - mid)` on
the right, `2.0 * (mid - lo)` on the left) gives `w, 3w, 7w, 15w, 31w, 63w,
127w`, which is what the formula claims. Both versions look like "doubling" in
the source.

The 7:9 result above was flown on the 0.29 version and is unaffected — its floor
is 12.3 % from the aim, well inside 29 %, and `bracketed` came back true — but the
*message* would have lied: a resonance needing a 40 % walk would have been told
the bracket already reached 127 % and to look elsewhere.

**Why it survived the test that existed.** `the_widening_reach_is_geometric_not_arithmetic`
asserted `reach_fraction()` against the literal `1.27` — the formula checked
against itself. **It would pass with the loop deleted.** The widening is now a
`Bracket` type whose `widen` runs over any objective, and the test walks it
downhill with `f(dv) = −dv` (which never brackets, so every widening is spent
walking) and asserts where `hi` actually lands. Two more pin the behaviour rather
than the claim: a minimum at half the reach must be bracketed *and* enclosed by
`[lo, hi]`, one at twice it must not be; and three infinities are not a bracket.
That pair also exposed the margin — `reach_fraction()` is where **`hi`** lands,
but bracketing needs the *centre* past the minimum, and the centre reaches only
`f·(2ⁿ − 1)`, about half as far. 7:9's floor is 12 % out against a 127 % walk, so
the shipping margin is ample; the guarantee is "comfortably inside", not
"inside".

That is the third time in this batch that a number was checked against another
statement of itself instead of against the thing it describes.

**And the physics answer, which the bug had been hiding: 7:9 is not an impact
keyhole for this rock at this ξ.** A resonant return is not the same thing as an
impact; the keyhole is a short arc of the resonant circle, and 7:9's floor sits
53 000 km off it in the *spatial* coordinate. No larger along-track impulse
closes that — ξ is a property of the deflection **direction**, so reaching it
would need an out-of-plane component, not more of the same nudge. That is the
first honest negative result for a resonance other than the 3:4, and it cost 31
flights in 814 s.

### P(impact) at the resonant return — 2026-09-06 session (the covariance mapped through *both* encounters, and the finding that it is knowing, not aiming, that moves the answer)

Roadmap item 2 from the *What is next* list: *"P(impact) rising near a keyhole —
the deflected trajectory as the Tier-3 nominal, and a chained two-encounter
Jacobian."* `uncertainty_sampling_plan` had refused two encounters in span since
2026-07-28, by design, until this existed. It exists now, and the headline it
produced is not the one the roadmap entry predicted — which is the part worth
reading.

**The chaining is in the propagation, not in a matrix product.**
`core/src/keyhole_target.rs` gains `ReturnSamplingPlan` / `chained_sample` /
`return_sensitivity`. Each of the thirteen columns flies a perturbed seed from
the campaign start, through the impulse, through encounter 1, across the handoff
and on to the resonant return, and reads the b-plane there. No two-encounter
matrix is composed and no flyby is modelled — the amplification is simply in the
numbers, which is the only honest way to do it, because the amplification *is*
the keyhole. `uncertainty_sampling_plan` is untouched: its refusal fires on the
*nominal* census, where the return does not exist at all, so there was nothing to
relax and its ~30 callers keep their meaning.

**Every epoch is pinned from one nominal flight.** The module's founding lesson —
reduce at a fixed epoch, never at the sample's own closest approach, or the
argmin quantises the Jacobian into plausible noise — has more places to be sprung
with two encounters, so `ReturnSamplingPlan` carries all of them: the handoff and
both reduction epochs. Letting each sample find its own handoff would
re-introduce the argmin at the seam.

**The licence for all of it, checked before anything else was spent (2 flights,
90 s):** reducing at the fixed epoch reproduces the flown solution's encounter-1
`b` to **+0.011 %** (153 469.8 → 153 486.0 km) and the return's to **−0.109 %**
(4 111.9 → 4 107.4 km). What is compared is `b`, not the closest-approach
distance — `b` is the asymptotic quantity a reduction 12 h early is entitled to
reproduce, and the geocentric distance is smaller by focusing and is a different
number by construction.

#### The step sizes: the risk was real, the map is linear anyway

`FD_STEP_POSITION_M` / `FD_STEP_VELOCITY_MS` were measured to provoke a 10–20 km
b-plane response **at encounter 1**. Downstream of a flyby the same step is
multiplied by the flyby's gain, and here that gain is **778×**: the shipping
velocity step provokes **78 559 km** at the return, seven times the 11 310 km
capture disc the answer is integrated over. A secant across seven times the
feature, returning a finite, symmetric, entirely plausible matrix.

So `bplane_jacobian` gained a sibling taking `FdSteps`, and the plateau study
(`probe_keyhole_probability steps`, 24 flights) ran with the *return* as the
observable:

| scale | position step | ∣Δb₁∣ | ∣Δb₂∣ | column moved |
|---|---|---|---|---|
| 1.0 | 312.5 m | 72.4 km | 56 305 km | — |
| 0.1 | 31.2 m | 7.2 km | 5 630 km | 0.000 % |
| 0.01 | 3.12 m | 0.7 km | 563 km | 0.039 % |
| 0.003 | 0.94 m | 0.2 km | 169 km | 0.187 % |

**The columns do not move.** The gain is 777.5 for the position block and 777.8
for the velocity block, constant to four figures over a 300× range of step. The
state→return-b-plane map is *linear* far beyond where it needed to be, so the
shipping steps would in fact have produced the right Jacobian — but only the
measurement could say so, and the shipping run now sits at 1 % of them (response
~5 % of the disc) because there is no reason to stand on a secant that wide once
the narrower one is known to be free.

That the two blocks return the same gain to four figures is itself the physics:
position and velocity perturbations both act through the single scalar channel
the closed form describes — encounter-1 b-plane displacement → `a'` → period →
arrival time — so the gain is a property of the *encounter*, not of the
perturbation.

#### The closed form is the discriminator, because the covariance cannot be one

The covariance is invented (synthetic rock, no observation arc), so it cannot
falsify a Jacobian: any matrix maps it to *something*. The closed form can.
Differentiating `a'` at the flown b-point (`OpikFrame::gradient_semi_major_axis`),
turning that into a period error (`ΔT/T = 1.5 Δa'/a'`), an arrival slip over the
3-year return, and Earth's own 30.278 km/s of motion gives a prediction with
nothing fitted in it:

- measured `∂ζ₂/∂ζ₁` = **−777.7**
- closed form = **−905.9**
- ratio **0.858**, signs agreeing

Agreement to 14 % across a chain spanning a flyby, three years of propagation and
two independently-built Öpik frames. That is the check that says the machinery
works, and there is no other available.

**The keyhole width falls out of it — with a caveat that matters.** `keyhole_at`
defines the width as `Δa'_tol / |∇a'|`, which *is* `2·capture_radius ÷
closed-form gain`. Substituting the **measured** gain gives **29.09 km** against
the map's **24.92 km** for the 3:4 far point: the linearised width is
conservative by **1.17×**, measured differentially, so it does not inherit the
map's absolute-placement error at all. Two things it is not. It is not an
independent second result — it is the 0.858 gain ratio inverted. And it does not
retire the earlier "the flown b-point sits 1.64 half-widths from its own circle"
finding, which is a statement about *placement* and a different quantity. The
roadmap asked for more flown resonances to calibrate the width; this calibrates
it differentially from one, which is better, but the placement question stays
open.

#### The cadence does not survive the flyby

`SAMPLE_CADENCE_DAYS = 10` rests on a cancellation argument: both runs of a
central difference fly at the same cadence, so the systematic error is common and
drops out. That was measured at a **single** encounter, and here whatever
survives the differencing is multiplied by 778 on its way to the return.
Measured (13 + 13 flights):

- the nominal return `b` moves **+2.97 % (+122 km)** at 10 days — and that is the
  *mean*, which no differencing protects;
- five columns agree within 2 %, and **column 2 is 17.7 % off** — the smallest
  column, an order below its neighbours, so the one where a common-mode residue
  is largest against its own signal.

The coarse cadence is therefore **not** available here, and `ReturnSamplingPlan`
carries its cadence as a field rather than reading the module constant. "A
Jacobian is only valid at the cadence its columns converged at" has been a doc
comment since July; this is the first place it has bitten.

#### The result: at a keyhole it is knowing, not aiming, that moves the answer

The roadmap entry expected a peak — probability high inside the door, low
outside. With the shipping covariance that is flatly not what happens, for a
geometric reason.

Mapping the invented covariance through the chained Jacobian gives a 1σ ellipse
at the return of **131 334 km × 0.5 km** — a *needle*, 260 000 times longer than
it is wide, against an 11 310 km capture disc. And the needle lies along **ζ₂,
the timing coordinate**, which is the only coordinate Δv can move. Sliding Δv
slides the mean *along* the needle, which cannot change how much of the needle
crosses the disc. Across the whole door — 21 points, ±5×10⁻⁵ m/s, the mean
sweeping from 4 107 km out to 28 345 km — **P moves from 0.0628 to 0.0642, a
contrast of 1.02×.** The σ-distance sits at ~8 210 on *every* row, which is the
tell: a 22 000 km excursion barely registers because it runs along the direction
the covariance is largest in. `uncertainty.rs` already documents that signature
at the first encounter (8 196 σ with P = 1); this is the same thing one encounter
downstream.

**Shrinking the uncertainty is what moves it.** Because mapping a covariance
through a fixed Jacobian is free, the probe reports three sizes of orbit
uncertainty per flight — scale factors on the **whole** covariance — and the
three are three different physical regimes:

| Σ scale | 1σ major | vs disc | P at the floor | across the door |
|---|---|---|---|---|
| 1.0 | 131 334 km | 11.61× | 0.0642 | contrast **1.02×** (flat) |
| 1/12 | 10 944 km | 0.97× | 0.6645 | 0.055 → **0.665** → 0.055, contrast **12.1×** |
| 1/100 | 1 313 km | 0.12× | 1.0000 | 0 → **1** → 0, a hard door |

At the crossover the keyhole finally appears as a *peak*; in the deterministic
limit it is a yes/no door about 2×10⁻⁵ m/s wide, and the unrefined closed-form
aim scores exactly 0 in both of the smaller columns while scoring 0.058 in the
largest. **So the honest statement is that the impact probability at a keyhole is
set by how well the orbit is known, not by how well the impulse is aimed — for as
long as the uncertainty is larger than the door.** Steering inside the door buys
nothing until that stops being true, which is why real planetary defence spends
its money on observation arcs.

**State that conditionally or it becomes false.** It is not a fact about keyholes;
it is a fact about *this needle*, whose length is dominated by the 1 km isotropic
position σ — the most arbitrary number in an already-invented covariance, and one
`synthetic_along_track` sets independently of the velocity block by construction.
A real SBDB covariance could land in any of the three rows. What travels is the
*mechanism* (the uncertainty and the impulse push along the same coordinate, so
they cannot trade against each other), not the row this rock happens to sit in.

**One thing the sweep does not have to assume: that `J` is constant across the
door.** It is measured. A Δv change applied at the campaign start *is* a velocity
perturbation of the seed, and the step study drove exactly that: at scale 1.0 the
velocity column used a step of 1.25×10⁻⁴ m/s — larger than the entire ±5×10⁻⁵
door — and the column held to 0.003 % down to 3.75×10⁻⁷. The whole door lives
inside a range over which the Jacobian was directly measured constant, so
re-flying thirteen columns at the door's edge would buy nothing.

**Two things had to be deleted before the data could be read.**

*The narration.* The first version of the sweep closed by printing that the
probability "swings from near-certain to near-zero over a Δv window smaller than
any impulse can be controlled to" — a sentence written before the run, asserting
the expected story. The measurement contradicted it. That is the fourth time in
three sessions that a number here was found stating a claim rather than
describing a measurement, and the sweep now prints a measured contrast ratio per
covariance instead of a conclusion.

*The first σ-ladder.* It varied `σ_along` over two decades and held `σ_position`
at 1 km, and the ellipse did not move: **131 334 → 128 501 km for a 100× change**.
At the return the **position** block dominates — the Jacobian's position columns
are ~1.2×10⁵ b-plane metres per metre, so a 1 km position σ contributes ~117 000
km against the velocity block's ~24 500 km. `synthetic_along_track`'s doc says
"the along-track velocity term dominates the map", which is the claim that made
the one-block ladder look reasonable; it does not hold at the return. "How well
is the orbit known" is a statement about the whole covariance, and only scaling
all of it asks that question.

#### The needle forced a conditioning check on the probability integral

A 2.6×10⁵ aspect ratio is far outside anything `impact_probability` had been
tested at, and its whitened-polar rule is exactly the kind that returns a
plausible small number when the `θ` band where the needle enters the disc is
under-resolved — the module's founding failure mode in different clothes. So
before any number above was quoted, two kernel-free tests went in:

- `a_needle_ellipse_matches_monte_carlo` — the measured 131 334 × 0.5 km geometry
  against 4 million deterministic draws (fixed-seed xorshift + Box–Muller, no new
  dependency), at three means including one inside the disc. It agrees.
- `needle_probability_holds_until_f64_loses_the_covariance` — squeezing the minor
  axis must not move P, and it does not, out to an aspect ratio of 2.6×10⁷. One
  decade further, `(σ_long/σ_short)² = 6.9×10¹⁶` is past what f64 can hold and
  the covariance stops being positive definite: the module **refuses** there
  rather than answering, which this test now pins. `StateCovariance`'s validation
  turns out to be load-bearing rather than ceremonial.

#### Cost, and what this leaves open

The whole batch is ~110 flights at ~45 s each in release: the step study 24, the
Jacobian 13, the cadence pair 26, the gain 2, and 22 per sweep. Nothing here
belongs on a build path or a frame; `return_sensitivity` is an on-demand worker
at best and an example at least.

Still open:

- **The Tier-3 ellipse on the Godot b-plane view** (roadmap item 3) — unchanged,
  and now with a second, far stranger ellipse worth drawing.
- **Real SBDB covariances** (roadmap item 1) — and this batch sharpens why they
  matter: every number above is a statement about an *invented* needle, and the
  needle's length is the whole answer.
- **The placement half of the keyhole-width question** (the 1.64 half-widths),
  which the differential calibration does not touch.
- **`synthetic_along_track`'s "velocity dominates" doc claim**, which this
  measurement contradicts at the return and which has not been re-checked at the
  first encounter.

### Real SBDB covariances — 2026-09-06 session (JPL's own uncertainty for a real object, and the finding that our dynamics are the same size as it)

Roadmap item 1 from the *What is next* list: *"Real covariances from the SBDB
(equinoctial or Keplerian elements at their own epoch, mixed units; validate the
conversion by round-trip). This makes P(impact) real for Apophis/Bennu and
retires the 'invented' label."* Three of that sentence's clauses turned out to be
wrong, including the validation it asked for.

New: `pyref/fetch_sbdb_covariance.py`, the committed fixture
`core/tests/fixtures/apophis.sbdb` (1.8 KB), `core/src/sbdb.rs`,
`core/src/frames.rs`, and two probes — `probe_sbdb_covariance` (kernel-free) and
`probe_sbdb_apophis_2029` (the payoff).

#### What JPL actually publishes, none of which was the guess

- **Cometary elements, not equinoctial or Keplerian.** The covariance is over
  `(e, q, tp, node, peri, i)` — perihelion *distance* and time of perihelion
  *passage*, no `a` and no `M`. Four NEOs were sampled while writing this —
  Apophis, Bennu, Didymos, Eros — and all four use it; that is an observation on
  four well-observed objects, **not** a claim about the database. What is actually
  guaranteed is the refusal: the reader rejects any other element set rather than
  converting it with the wrong partial derivatives (`a_different_element_set_is_refused`).
- **The matrix is 8×8, not 6×6.** Apophis' carries the non-gravitational `A1`/`A2`
  as estimated parameters; Bennu's carries `RHO`/`AMRAT` instead. Dropping the
  trailing rows and columns *is* marginalisation for a Gaussian, so the leading
  6×6 block is the orbit's covariance with the non-grav uncertainty already
  folded in — but that is a fact worth stating, not a trim to do quietly. The
  file records what it dropped.
- **The covariance has its own epoch.** Apophis' is JD 2459215.5 (2020-12-17);
  its osculating elements are published at JD 2461200.5, **5.43 years later**.
  Moving a covariance between epochs needs a state-transition matrix we do not
  have, so everything here works at the covariance's epoch and the file carries
  the element values *there* (the API supplies them under
  `orbit.covariance.elements`).
- **Ecliptic, not ICRF**, and in mixed units: dimensionless, au, **days** (`tp`
  is a Julian date, so its variance is in days²), and **degrees**.

#### "Validate by round-trip" could not have validated anything

The roadmap line asked for a round-trip. A round-trip (elements → Cartesian →
elements) runs the same unit convention in both directions, so a consistent
degrees-for-radians error, or a `tp` scaled by the wrong number of seconds,
cancels *exactly* and the test passes. It is the quantised-argmin trap in
different clothes: a plausible answer that no structural check rejects. Three
independent gates went in instead, each able to fail on its own.

- **JPL's published per-element σ against `sqrt(diag)`.** They agree to every
  digit published — measured, ratio `1.000000` on all six. This pins the ordering
  and the native units before any physics happens, and it is enforced **at parse
  time** (`SbdbError::SigmaMismatch`), not only in a test. Free, external, and it
  catches the entire first half of the conversion.
- **JPL's own Cartesian state at the covariance epoch**, fetched from Horizons in
  *both* frames the conversion passes through and carried in the fixture.
  Reconstructing from the elements and comparing pins the element conversion;
  comparing the rotated result pins the obliquity rotation separately. Neither is
  a round-trip — the right-hand side is JPL's. Measured: **22.75 m** with the
  hardcoded `μ_sun`, and **1.4 m** when `μ` comes from the loaded DE440 kernel,
  which is the ~30 m `μ`-sensitivity the module documents, confirmed rather than
  asserted. A degrees-for-radians slip here would be ~1e10 m.
- **Monte Carlo in element space against `J Σ Jᵀ`.** 20 000 deterministic draws
  (fixed-seed xorshift + Box–Muller, no new dependency, the pattern
  `uncertainty`'s needle test already uses), agreeing with the linear map to
  better than 5 % of the ellipse — a tolerance set by the Monte Carlo's own
  `1/√N`, not by the Jacobian. This is the discriminating gate: it is the only
  one that can fail on a wrong Jacobian *and* a wrong unit scaling *and* an
  inadequate linearisation.

The finite-difference step got the treatment this crate's other steps got, and
the map being **closed form** is why it is easier: no integrator, so no noise
floor, and the only competition is truncation against round-off. Measured per
column over six decades, every column is flat to better than **4e-8** relative
across `u ∈ [1e-7, 1e-4]`, degrading to 1e-2 at `u = 1e-2` and 1e-5 at `u = 1e-9`.
`FD_RELATIVE_STEP = 1e-6` sits in the middle of that plateau. The steps are
relative to each element's own *scale of variation* — 1 for `e` and the angles,
`q` for `q`, and the **orbital period** for `tp`, which is the one that is not
the element's own magnitude.

#### The cigar is tilted, and the tilt is the physics

The fourth check is free and physical: a real NEO covariance is an along-track
cigar, so the mapped position ellipsoid's long axis should lie near the velocity.
Measured: **655 m × 38 m, aspect 17:1, long axis 9.8° off velocity** — near, but
not along, and the first instinct was that 9.8° meant a bug.

It does not. Taken one element at a time the contributions are far *larger* than
the total: `node` alone 8 379 m, `peri` alone 9 022 m, `tp` alone 1 518 m, against
a six-element answer of 655 m. The estimated element errors are strongly
correlated and **cancel by 14×** in position space, and what survives the
cancellation is the node/peri residual, which sits 7.85° off the velocity. That is
what tilts the cigar. Only the pure-timing term is exactly along-track, and the
probe measures it at **0.00°** — which doubles as an independent check on the `tp`
column of the Jacobian, since a timing error can only move a body along its own
path. The test's tolerance is 20°, argued from that measurement rather than
tightened until the number passed.

This is also the concrete reason a marginal covariance is not a list of sigmas,
which is easy to say and easier to forget.

#### The sphere-of-influence trap, which looked exactly like a dynamical error

`probe_sbdb_apophis_2029` flies the real covariance to the 2029 flyby. The first
version fixed the reduction epoch three days before closest approach — reasoning
only about the fixed-epoch requirement, which is numerical — and reported a
perigee of 33 875 km against JPL's published ~38 000 km. A 4 125 km miss on a
famous encounter reads as a broken seed or a broken field.

It was neither. Three days out puts Apophis **1.5 million km** from Earth,
outside the ~924 000 km sphere of influence, where the osculating geocentric
hyperbola is not the encounter at all and the two-body extrapolation is really
measuring the Sun. `UNCERTAINTY_REDUCTION_LEAD_SECONDS` is 12 hours and its doc
says why — *inside* the sphere, *outside* the well — and that is a **physical**
constant, not a numerical preference. Reducing there (253 000 km out) gives
**37 984 km against JPL's ~38 000 km, 16 km off**. The close approach is now
*found* on the nominal run rather than assumed from a hardcoded epoch.

The b-plane Jacobian's shipping steps were checked here rather than reused on
faith, since `uncertainty`'s own docs say that criterion does not travel: on this
encounter they do, the columns moving by at most 1.4e-4 across ×0.25…×4. That is
"the shipping steps are safe here" and **not** "the plateau was located here" —
the sweep is one-sided, `×0.25` is already the largest deviation, and nothing
above `×4` was tried. The element→state Jacobian's plateau, by contrast, really
was mapped, over six decades and per column.

#### The headline is not the probability

| quantity | measured |
|---|---|
| 1σ b-plane ellipse at the 2029 encounter | **18.2 km × 0.48 km** (aspect 38:1) |
| our own position residual vs JPL over the same arc | **15.1 km** |
| nominal crossing, in σ from the capture disc | 19 571 |
| P(impact in 2029) | **0** |

The ellipse and our own dynamical error are **the same size**. So the ellipse is
an honest statement about JPL's astrometry and says nothing about the physics we
do not model — every planet's relativity, and the radial `A1` — which displaces
the nominal by just as much. A real covariance does not make a prediction real on
its own. What it does is make the *other* error term visible, because now there is
something to compare it against. That is the actual result of this batch.

The residual is measured a year *short* of the flyby, against JPL's raw held-out
samples. Measuring it at the reduction epoch gives 14 391 km, which is not our
error at all: `horizons.rs` already documents that a 1-day state table cannot
resolve this particular hours-long flyby and measures its own interpolation error
there at 18 885 km. Reading that as a dynamical residual would have been reading
the truth table's error and calling it ours.

P = 0 at 19 571 σ is the **correct** answer — Apophis' 2029 approach is
well-determined and it misses — and this layer producing a real zero on real data
is the result, not a disappointment.

#### One obliquity in the project

The ecliptic↔ICRF rotation lived only in the Godot binding, and core now needs it
(SBDB elements are ecliptic). Rather than spell the constant out a second time —
the same trap this crate already refuses for `μ_sun`, two spellings of one number
that agree until one is edited — it moved to `core/src/frames.rs` and the
binding's three helpers delegate to it. The constant is `84381.448″`, IAU 1976 and
**not** IAU 2006's `84381.406″`, because that is the value defining SPICE's
`ECLIPJ2000`; the 42 mas difference is a ~30 km cross-track offset at 1 au.

#### What this leaves open

- **The `synthetic_along_track` path is untouched and stays.** The shipping
  campaign's rock is designed and will never have an observation arc, so its
  probability stays invented and labelled. Nothing in the keyhole or frontend
  numbers moves as a result of this batch.
- **Bennu is a follow-on, deliberately.** Its solution estimates `RHO`/`AMRAT` —
  solar radiation pressure — where Apophis estimates `A1`/`A2`. Its published
  covariance therefore describes a propagation we do not reproduce unless SRP is
  configured to match its area/mass ratio. Doing it anyway would buy a roadmap
  checkmark and a wrong number.
- **The covariance epoch is where the propagation must start.** Carrying it to an
  arbitrary epoch — the campaign start, say — needs a state-transition matrix,
  which is the same object as a 6×6 variational solve and is not built.
- **Nothing of this is on the frontend.** The Tier-3 ellipse on the Godot b-plane
  view (roadmap item 3) is still the next visible thing, and it now has two
  ellipses worth drawing rather than one.

---

### The integrator, measured instead of replaced — 2026-09-06 session (roadmap item 5 retired: dop853 is converged, and the reason is not the tolerance)

Roadmap item 5 was "dop853 → IAS15 crossover, now that 15-year multi-revolution
arcs (the keyhole returns) are in the pipeline." It is **retired, not deferred**,
and the batch that retired it is a measurement: `core/examples/probe_integrator_convergence.rs`,
plus one guard test and three doc corrections. No second integrator was written.

The pitch for the item was wrong, and worth recording as wrong: it was argued from
the previous session's finding that our own position residual against JPL over
Apophis' arc (15.1 km) is the same size as the real orbit uncertainty (18.2 km).
That residual is **unmodelled forces** — every planet's relativity, and the radial
`A1` — and this file says so where it reports it. A better integrator does nothing
for a force that is not in the model. The real customer was elsewhere: every
keyhole conclusion rests on 15-year arcs whose convergence nobody had checked.

#### The gate first, because every number below is a difference of two runs

Two builds at one tolerance reproduce the seed, the encounter perigee and the
end state **bit-for-bit**. Without that identity a tolerance delta and a
nondeterministic re-fly are indistinguishable — the same identity the Pluto
measurement leaned on to read 0.6 m as physics.

#### What the shipping tolerance actually allows, and why it never showed

`Dop853`'s error scale is `atol + rtol·|y|`. Every forward propagation ran at
`Dop853::new()`'s default `rtol = atol = 1e-9`, in SI, where a heliocentric
position component is ~`1.5e11 m` — so the controller accepts a sub-step with up
to **~150 m** of estimated local position error, and `atol` never binds on
anything. The *backward* seed design runs at `1e-12` and every force-term
isolation test at `1e-12`/`1e-13`. The forward path — the one every published
number comes out of — was the loosest thing in the crate.

Sweeping it `1e-9 … 1e-13` over the 12-year campaign, **the same tolerance is
either fine or catastrophic depending on how it is driven**:

| path over the 12.16-yr span | `|Δr|` vs the converged reference, on the cruise |
|---|---|
| one uninterrupted `step` call, rtol `1e-9` | **129 km** |
| the shipping clock path (1-day snapshots), rtol `1e-9` | **0.26 m** |

Both are the same integrator at the same tolerance. The difference is that
`Clock::propagate` restarts the adaptive controller at every snapshot, so a 1-day
snapshot **caps the step**, and the cap is what holds the error down. The
comparison is taken a year *short* of the flyby on purpose: the campaign ends 60
days past a 3 000 km Earth pass, which multiplies whatever arrives at it by a
measured **5 558×** (steady across four tolerances, which is itself the check that
it is a linear amplification and not noise). The amplified figures — 719 000 km
and 1.4 km — are those two times 5 558, and quoting them as integration error
would be quoting the flyby.

That closes on itself exactly: the clock path's `0.2582 m` cruise error × `5557.9`
= `1435 m`, which is its measured end-state column to four figures. Two
independent routes to one number.

#### The cross that decides it

If the cap is what sets accuracy, then coarsening the cadence removes the cap and
the tolerance becomes the only thing controlling the step — so the coarse-cadence
penalty `probe_tier3_cost` measured must **shrink** when the tolerance tightens.
Encounter-1 perigee, cadence × tolerance, shift against that row's own 1-day cell:

| rtol | 3 d | 10 d | 30 d | 90 d | 180 d |
|---|---|---|---|---|---|
| `1e-9` (shipping) | +0.03 m | +117.7 m | **+13 566 m** | +25 100 m | +27 360 m |
| `1e-11` | −0.02 m | −5.4 m | +54 m | +72 m | +58 m |
| `1e-13` | −0.02 m | +0.36 m | +0.51 m | +0.66 m | **+0.68 m** |

It shrinks by 40 000×. So the coarse-cadence error was step size all along, and
three things follow:

1. **At the shipping 1-day cadence the tolerance is not binding.** Tightening it
   four decades moves the perigee 0.07 m and costs 33 % more wall clock. The
   shipping value stays.
2. **The shipping accuracy is a property of the architecture, not of the
   tolerance** — a latent trap for anything that lengthens the effective step. The
   knob is now named (`ImpactorConfig::forward_rtol`, threaded through a single
   `stepper` field so the nominal, the re-flies and `propagate_free` cannot end up
   at different tolerances) and a kernel-gated test pins the conditional:
   `a_coarse_cadence_is_tolerance_bound_where_a_fine_one_is_step_capped` asserts
   the **ratio** collapses (fine-cadence sensitivity < 1/1000 of coarse), because
   the metre values are machine-specific and the separation is the physics.
3. **The efficient pairing is the opposite of the shipping one.** 180 days at
   `1e-13` reaches the 1-day answer to **0.68 m in 0.56 s**, against the shipping
   1-day/`1e-9`'s **10.9 s**. An 8th-order method wants few large accurate steps;
   we were giving it many small sloppy ones. The shipping cadence stays at 1 day
   anyway — the frontend draws the arc and needs the snapshots — but a sampling
   path that only wants a b-plane number is paying ~19× for nothing.

#### The keyhole conclusions, in the coordinates they are stated in

The load-bearing check. The 3:4 refined floor is called physics rather than an
unconverged search *because* the return's timing coordinate `ζ₂` is small beside
its spatial one `ξ₂` — and truncation over a 15-year arc lands as secular **phase**
error, which is precisely `ζ₂`. Flying the same Δv = 0.216550 m/s at each
tolerance:

| rtol | enc-1 perigee | return | `ξ₂` | `ζ₂` | `|ζ₂/ξ₂|` |
|---|---|---|---|---|---|
| `1e-9` (shipping) | 146 785.195 km | 1 141.296 km | 4 014.289 km | 890.598 km | 0.2219 |
| `1e-13` | 146 785.195 km | 1 141.339 km | 4 014.292 km | 890.963 km | 0.2219 |

`ζ₂` moves **365 m**, `ξ₂` moves 3 m, and the ratio the argument turns on is
0.2219 at every rung. Against the ~25 km closed-form keyhole width — the honest
denominator for "does this change a conclusion" — 365 m is 1.5 %. **Every keyhole
conclusion is converged at the shipping tolerance.**

> Read the *deltas* in that table, not the absolute `ζ₂`. The sweep flies a
> six-decimal Δv, and `ζ₂` responds at 5.4e8 km per m/s, so the 891 km is where
> the rounding put the shot, not the floor's timing residual (which is −26.6 km).
> The 365 m is a derivative at fixed Δv and does not care where on the curve the
> sweep stands. This absolute-vs-delta confusion is exactly what produced the
> repo contradiction below.

And the column does not converge monotonically — it scatters (−365, −349, +127,
+59 m). That is not a failure of the sweep, it is *stronger* evidence for the
finding than a monotone fall would be: at 1-day cadence the tolerance is not
binding, so what is left between rungs is uncorrelated scatter rather than
truncation. A monotone fall would have meant the tolerance was still in control.

#### Two loose ends, chased rather than asserted

- **The 118 m at the Tier-3 cadence is not a Tier-3 error.** Every cell of the
  cross is a *nominal* perigee, and Tier 3 reads **derivatives**. Re-measuring one
  central-difference column, `∂(perigee)/∂v_along` at 10 days, gives `1e-9` vs
  `1e-13` differing by **0.0225 %** — which is the 0.024 % the cancellation
  argument in `SAMPLE_CADENCE_DAYS`' doc already predicted. So the cadence bias
  never was in the derivative, the constant stays at ten days on the reasoning it
  always had, and the tolerance pairing matters here only if this layer starts
  reading an *absolute* b-plane position — drawing the ellipse on the frontend,
  which is roadmap item 3.
- **A sub-metre residual survives a tight tolerance, and it is not the scan.** At
  `1e-13` the perigee still walks monotonically with cadence and saturates
  (+0.36 m at 10 d → +0.68 m at 180 d). Refining the close-approach scan's
  sampling step 24× (6 h → 1 h → 15 min) does not move it by a **bit**, so it is
  not the argmin's resolution. What that does *not* separate is the degree-7 dense
  interpolant's own accuracy over larger accepted sub-steps from a real trajectory
  difference — finer sampling of an inaccurate interpolant does not help.
  Distinguishing them needs a re-integration to the closest-approach epoch and is
  not done. 0.68 m changes no conclusion in this project; it is recorded so it is
  not rediscovered.

#### Why IAS15 is retired rather than deferred

There is no oracle for it, and that is a stronger argument than cost. §6 nominates
REBOUND for the comparison and in the same breath says REBOUND self-gravitates the
planets and is therefore **not** a trajectory oracle — only invariants and
encounter sensitivity. Horizons cannot help either: `horizons.rs` documents that a
1-day state table cannot resolve this class of flyby (18 885 km of its own
interpolation error), and a resonant return three years past a deflection is in no
truth table anywhere. So the only available oracle for "does IAS15 beat dop853 on
*our* field" is dop853-at-a-tighter-tolerance — which is exactly what this batch
ran, and it says dop853 is converged. A second integrator would have nothing to
prove against, and unfalsifiable work is worse than no work.

The reversibility residual (forward 12 years then back) is reported and
deliberately **not** used: it falls four decades then jumps at the tightest rung,
because it round-trips the flyby twice and both cancellation and amplification act
on it. The direct comparison against a converged reference is the metric; this one
is a free invariant that happens to be a bad one here, and saying so is cheaper
than letting a reader trust it.

#### One inconsistency found — chased in the next batch

`ζ₂` reproduces here as **891 km** against `ξ₂` 4 014 km, matching the
keyhole-targeting memory entry. `core/src/keyhole_target.rs`'s own module-doc
table says **786 km** against 4 013 km for the same refined shot. Two spellings of
one measurement disagreeing inside the repo — the class of thing this project
keeps catching. **Settled 2026-09-06: neither was a spelling and neither was the
floor.** See *The floor that was a stopping distance*.

---

### The floor that was a stopping distance

*(2026-09-06 — roadmap item 6, and it did not close the way the item said it
would.)*

The item proposed re-running `probe_keyhole_return` and believing whichever
number the code printed. That would have been wrong. The re-run reproduces 786 km
exactly, because the solve is deterministic — and "believing the code" would then
have deleted 891 km as stale, when 891 km is a perfectly correct measurement of a
slightly different shot.

#### What the two numbers actually are

`ζ₂` is the return's **arrival-timing** coordinate, and Δv is a timing knob, so
`ζ₂` responds to Δv about as hard as anything in this project responds to
anything. Measured on a nine-rung ladder of flights
(`core/examples/probe_keyhole_floor.rs`):

| | response to Δv |
|---|---|
| `ζ₂` (timing) | **5.427e8 km per m/s** |
| `ξ₂` (spatial) | 4.861e6 km per m/s — **112× less** |

So a Δv printed to six decimals carries ±5e-7 m/s, which is **±271 km of `ζ₂`**.
The two published numbers differ by 105 km, which is 1.93e-7 m/s of Δv — 0.39× that
rounding. Solving each backwards:

- `ζ₂` = **786 km** is the flight at Δv `0.2165498072`
- `ζ₂` = **891 km** is the flight at Δv `0.2165500007` — i.e. the rounded
  `0.216550` that `probe_integrator_convergence` and the unit test both hard-code

Two shots 1.9e-7 m/s apart. Not two frames, not two integrator settings, not
two spellings. And the giveaway was in plain sight: a frame difference would move
*both* coordinates and could not change the scalar return distance, yet the return
distance travels with the pair too (1 130 km with one, 1 141 km with the other).
Only Δv moves `ζ₂` alone.

#### The bigger find: the published floor was never the floor

Flying the ladder past the published value found lower returns *below* it. Refined
properly — `probe_keyhole_return -- 3 4 minus retro 20`, 26 flights in 623 s:

| | Δv (m/s) | return | `ξ₂` | `ζ₂` | `|ζ₂|/|ξ₂|` |
|---|---|---|---|---|---|
| shipping default, 12 iterations | 0.2165498080 | 1 130 km | 4 013 km | 786 km | 0.196 |
| **20 iterations** | **0.2165483096** | **1 087 km** | 4 006 km | **−26.6 km** | **0.007** |

The 12-iteration search stops **1.6e-6 m/s short**. At the real minimum the timing
coordinate goes to *zero* and the residual is pure spatial offset — which is
`keyhole_target.rs`'s own thesis, now shown far more sharply than the number that
was standing in for it. Those several-hundred-km `ζ₂` values were never the orbits'
timing residual; they were **how far short of the minimum the search stopped**,
read out in a coordinate that moves 5.4e8 km per metre per second.

The ladder and the solve agree independently, which is what makes this a
measurement rather than a curve fit. Fitting a parabola through the ladder's three
lowest rungs puts the minimum at Δv 0.2165482893; the ladder's `ζ₂ = 0` crossing is
at 0.2165483587; the 20-iteration golden-section landed at 0.2165483096, between
them. The distance minimum and the timing zero are the **same point** to 7e-8 m/s —
they have to be, because near the bottom `ζ₂` moves 112× faster than `ξ₂`, so the
scalar distance is a proxy for `|ζ₂|` and minimising one minimises the other.

#### Why it stopped short, and what else that breaks

Twelve golden-section steps close a 1 % bracket by `0.618¹²`. That is
`2 · 0.01 · 0.2164375 · 0.618¹² = 1.34e-5` m/s — **exactly** the "Δv window" the
solve reported. The convergence tolerance (`rel_tol` 1e-7 of Δv ≈ 2.2e-8 m/s)
would need ~26 iterations and never binds at 12. Confirmed by running 20: the
window came back **2.86e-7 m/s**, and `1.34e-5 · 0.618⁸ = 2.85e-7`. It tracks the
iteration count to three digits.

So `KeyholeSolution::dv_window_m_s` **measures how long the search ran**, not how
wide the door is. Its doc called it "the keyhole in Δv terms: how finely the
impulse has to be controlled to stay in the door", and `probe_keyhole_return`'s
header converted it into "~13 km of b-plane — the same order as the closed form's
24.9 km far-end keyhole width". That agreement was a **coincidence of running 12
iterations**; at 20 it would have read 0.3 km and the same reasoning would have
concluded something 47× different. Both docs now say what the number is. For
contrast, the door the return actually flies through — the Δv span over which it
stays inside its own focused capture disc, 11 309 km against a 4 006 km spatial
floor — is ~±2e-5 m/s, about 3× *wider* than the 12-iteration bracket.

#### What was deliberately left alone

- **The 29.09 vs 24.92 km door-width calibration.** It comes from `∂ζ₂/∂ζ₁` along
  the flown trajectory, not from the floor's `ζ₂`, so a 1.6e-6 m/s shift in where
  the search stopped does not touch it.
- **`probe_integrator_convergence`'s `DV_FLOOR_M_S = 0.216550`.** Left rounded and
  now documented as rounded. That probe measures a *derivative* at a fixed shot
  (how much `ζ₂` moves with tolerance); moving the constant would change every
  absolute number and no delta. Only the framing "365 m out of 891 km" was an
  artefact, and it now reads against the ~25 km keyhole width instead.
- **The downstream keyhole-targeting and probability numbers** (2.9 widths off 3:4,
  0.930 m/s at 900 d, P = 0.064). A 1.6e-6 m/s shift is far below what that layer
  resolves — the probability is set by how well the *orbit* is known, and is flat
  across the whole door. Re-running them to make the digits match would be
  churn, so they stand as measured and this note says why.

#### The reusable lesson

Two of this project's recurring traps met in one place. **A search's own bracket is
not a physical width** — golden-section converges just as hard onto an iteration
limit as onto a minimum, exactly as it converges onto a wall when unbracketed (the
7:9 trap). And **a number is only as good as the digits its input was printed
with**: `ζ₂` at six-decimal Δv is a ±271 km measurement being quoted to three
significant figures. `probe_keyhole_floor.rs` exists so both are visible as
arithmetic instead of arguable.

---

### The uncertainty ellipse on the b-plane — 2026-09-06 session (roadmap item 3: the deterministic picture given a spread, and the axis it turned out to lie on)

`[U]` on the encounter view draws the Tier-3 1σ ellipse; `[Z]` / `[X]` turn a knob
that asks the same question at a better- or worse-known orbit. The layer already
existed in the core (`core/src/uncertainty.rs`, since July); what was missing was
a way to *look* at it, and looking at it is what the whole §1 determinism caveat
promises.

**Shape of the thing.** `Σ_b = J Σ Jᵀ` splits into an expensive half and a free
one: the Jacobian `J` is 13 propagations and describes the trajectory; the
covariance `Σ` costs nothing and describes how well the orbit is known. So the
solve rides a worker (an **eighth** independent channel beside the build, the
Tier-2 preview, the grid, the verify, the mass solve, the tow probe and the
anchor), lands once, and is then held — after which every press of the σ knob is
a 2×2 matrix product. Measured **34.6 s** for the solve; the knob is free.

#### The trap this was designed around, and the number that closed it

`BPlaneSensitivity` carries a b-plane frame that `uncertainty.rs` calls
*arbitrary-but-deterministic*: it seeds off whichever coordinate axis is least
aligned with `Ŝ`. Every **scalar** that module reports is invariant under that
choice, and its tests pin the invariance. **An ellipse's orientation is not a
scalar.** Taking the angle out of that frame and drawing it on the view's pinned
Öpik axes would give correct axis lengths at a rotation nobody chose — wrong in
precisely the way that looks right, on the one screen whose entire job is that the
picture and the numbers cannot disagree.

So the covariance is rotated into `(ξ̂, ζ̂)` in the core, where the frame is
already owned, and the rotation is *measured* rather than assumed:

| | measured |
|---|---|
| `‖R Rᵀ − I‖∞` | **1.95e-10** |
| nominal's out-of-plane component | **0.106 km**, against `\|B\|` 7 074 km |

The two b-planes are the same plane to a part in five billion. They are not
*identical* frames — the Öpik one is built on the closest-approach reduction and
the sensitivity's nominal on the fixed-epoch one — but the disagreement is
nowhere near enough to bend an ellipse. The worry was real and cost nothing to
satisfy, which is the good outcome: it is now asserted in a test instead of
believed.

#### What it draws, and why the orientation is the interesting part

At the shipping (synthetic) covariance:

    1σ  168.71 × 0.82 km  —  205:1  —  lying 0.3° off ζ̂
    P(impact) = 1.000000 over the 11 312 km capture disc

**The spread is essentially pure `ζ`: the timing coordinate.** That is the same
axis the keyhole work found a Δv nudge moves, and the same axis
`probe_keyhole_probability` found the resonant return's needle lying along. Three
independent measurements now say the same thing about this rock — what is
uncertain about it, and what is controllable about it, are the *same direction* —
and the ellipse is the first place it is visible rather than tabulated.

#### Three drawing decisions, each settled by a measurement rather than a preference

- **It is centred on its own mean, not on the drawn cross.** The cross is the
  closest-approach reduction; the ellipse's centre is the fixed-epoch one its
  Jacobian was differenced around. Measured **5.43 km apart — 6.6 minor axes**.
  That is invisible on screen and enormous next to the shape it would have been
  pinned to, so pairing them would have been centring one instrument's spread on
  another instrument's position. The gap is printed in the readout.
- **At the default zoom it is about one pixel, and the view says so instead of
  drawing it.** 168.71 km against a 0.15 LD half-span (~58 000 km) is ~1 px; the
  minor axis is a hundredth of that. Fattening it to something visible would be
  drawing a spread the orbit does not have. A ring marks where it is, the caption
  says `1-SIGMA < 1 PX - ZOOM IN`, and the axes are printed in kilometres
  unconditionally. **The smallness is the finding**: this crossing is known to a
  couple of hundred kilometres inside a disc eleven thousand kilometres wide —
  the same shape of result as Apophis' real 18.2 × 0.48 km ellipse.
- **The σ knob only bites upward, and that is the honest answer.** ×0.01 still
  reads P = 1.000000; ×100 is the first setting where P leaves 1, at **0.407**
  with a 16 871 km major axis. This rock is *designed* to hit, so a better-known
  orbit cannot make it miss — what improves is the sharpness of a certainty, not
  the certainty. The knob is still the layer's point: the same rock, the same
  trajectory, an impact probability that moves because of how long anyone has been
  watching.

#### What was deliberately not put on screen

- **`sigma_distance` (the Mahalanobis distance from Earth's centre).** Its own doc
  warns it is "not 'how many σ from a hit', and reading it that way inverts the
  answer" — the designed hit reads ~8 200 σ *with* P = 1. Printed beside P = 1 and
  without the capture radius in the comparison, the panel would contradict itself,
  which is the exact failure the b-plane view exists to end. `p_impact` and
  `capture_km` are printed together and never apart.
- **A 3σ ring.** `Σ_b = J Σ Jᵀ` is exact only for a linear map, and whether the
  linearisation still describes the encounter at the edge of the covariance is
  what `bplane_uncertainty_checked` measures — 25 propagations, ~28 s, not paid
  here. A 3σ ring drawn off the bare Jacobian is a shape the code will not vouch
  for, so only 1σ is drawn. **Open:** run the shell once and record whether the
  ellipse is still an ellipse out there or the truth is a banana.
- **A deflected ellipse.** The Jacobian is about the *nominal* seed, so there is
  no spread for the planned track and the legend says so outright
  (`ELLIPSE = 1-SIGMA, NOMINAL TRACK ONLY`). An ellipse on the cross beside a bare
  diamond would read as "the deflection is certain" — the opposite of the caveat
  this feature exists to honour.
- **The word "synthetic" is on the panel, and is not decoration.** The rock is
  invented, so its covariance is a shape borrowed from real NEOs
  (`synthetic_along_track`, the same three constants `probe_tier3_uncertainty`
  uses, so the panel and the probe cross-check) and not a measurement of anything.
  An ellipse drawn without that word is a claim about how well this asteroid is
  tracked, and nobody tracks it.

#### The rotation already existed, and now there is one of it

`probe_keyhole_map.rs` has computed exactly this rotation since July — same four
dot products, same orthonormality guard — to draw the ellipse into the published
b-plane map (`docs/keyhole_map.svg`). The binding was written with its own copy
before that was noticed. Two hand-rolled copies of one rotation, feeding two
pictures that must agree, with no way to check them against each other, is the
shape of every frame bug this crate has found; so it is now one method,
[`BPlaneBasis::rotation_to`], living with the arbitrariness that makes it
necessary and taking plain axis vectors so nothing new is coupled. It returns the
residual instead of gating on it, because what counts as coplanar-enough belongs
to the caller.

The payoff is a real cross-check rather than a tidiness argument. The published
map records `sigma_axes_km: [168.709999, 0.818223]` at `89.736°`; the live view
measures **168.710343 × 0.818218 at 89.737°**. Two entry points, two reductions of
the nominal 0.05 km apart, one ellipse.

#### Three faults the picture and the reviewer found that the tests did not

- **`tier3_online` was never reset on a rebuild.** The Rust side drops the
  Jacobian in `poll_build` — but the GDScript flag is set exactly once, inside
  `_poll_tier3`, which returns early unless a solve is running. So after `[N]`
  rebuilt the threat, `has_tier3()` was false while the flag stayed lit, the
  cached dictionary kept handing the **old rock's** ellipse to the view to draw on
  the **new rock's** b-plane, and `request_tier3` refused to re-solve because it
  believed one was already in hand. Three failures from one missing line, and the
  middle one is a picture quietly asserting something untrue about a different
  asteroid. Fixed in `_invalidate_derived_views`, beside the `pork_online` reset
  that exists for the identical reason. **No test could have caught it** — the
  screenshot harness never rebuilds the threat.
- **The readout was drawn on top of the HUD's event log.** Only a screenshot says
  so: the pieces sharing that screen are drawn by three different nodes, and the
  layout is not derivable from any one of them. Moved to the empty band above the
  target card, with the reason recorded next to the constant.
- **The three new keys had never been pressed.** The harness called
  `toggle_uncertainty()` and `tier3_sigma_step()` directly, so the hand-written
  action blocks in `project.godot` and the three dispatch branches in `main.gd` —
  the whole path between the footer's promise and the method — were unexercised.
  The harness now drives them through `InputMap` / `Input.parse_input_event`, the
  way a player does, and prints whether each action is registered.

[`BPlaneBasis::rotation_to`]: core/src/uncertainty.rs

#### Where it lives

- `godot/rust/src/mission_core.rs` — `Tier3View` (the held sensitivity plus the
  rotation), `Tier3Ellipse`, and the kernel-gated
  `the_tier3_ellipse_is_drawn_in_the_views_own_frame`, which is where every number
  above is printed and four of them are asserted.
- `godot/rust/src/lib.rs` — `begin_tier3` / `is_solving_tier3` / `poll_tier3` /
  `has_tier3` / `tier3_set_sigma_log10` / `tier3_ellipse`, and the invalidation:
  the sensitivity is dropped whenever a scenario is installed, because a Jacobian
  is about one rock's trajectory and `[N]` can put a different rock on a different
  orbit between one frame and the next. The σ knob is deliberately *not* reset —
  "how well is the orbit known" is a question about the layer, not a property of
  any one threat.
- `godot/scripts/sim.gd` — `request_tier3`, `_poll_tier3`, `tier3_sigma_step`.
- `godot/scripts/encounter.gd` — `_draw_uncertainty` and
  `_draw_uncertainty_readout`.
- `godot/tests/_shot.gd` — three shots (sub-pixel at the default zoom, the ellipse
  zoomed to where it is a shape, the σ knob two decades wide), driven through the
  real keybindings.
- `core/src/uncertainty.rs` — `BPlaneBasis::rotation_to`, shared with
  `core/examples/probe_keyhole_map.rs`, plus the kernel-free
  `a_rotation_between_two_frames_of_one_plane_moves_the_angle_and_not_the_axes`.

### Frontend speed — 2026-09-06 session (the startup wait cut a quarter, one binding crossing for two dozen bodies, and two plan premises that did not survive being measured)

Tasks 1 and 2 of `docs/plans/2026-09-05-visuals-performance-followups.md`. Both
landed; neither landed as written, and the reason is the same in both cases —
the plan described a mechanism, the code disagreed, and measuring first is what
found it.

#### Task 2: twenty-four lookups a frame became one crossing

`body_positions_ecl_au` takes a `PackedInt64Array` of NAIF ids and returns a
`PackedVector3Array`, one slot per id **including the misses** — a shortened
answer would slide every body past the gap onto its neighbour's position, so the
contract is pinned by a kernel-gated test that asks for eight resolvable planets
and two unresolvable ids and checks each slot against the single-body lookup at
two epochs. It makes no individual lookup faster (still ~11 µs); it removes the
traffic.

Native calls per frame, measured on this machine with the fill off and on:

| view | off | on |
|---|---|---|
| 3d_system, 3d_earth_closeup, 3d_planner_open, 3d_max_warp | 29 | 6 |
| map2d | 5 | 2 |
| encounter_keyholes, encounter_zoomed_out | 2 | 2 |
| porkchop | 0 | 0 |

The 2026-09-05 baseline row of "29 for every view" no longer describes the *off*
state, which is why the off column was re-measured rather than quoted.

**Two defects shipped in the first commit of this**, and both are worth keeping
on the record:

- **It did not parse.** `var out := mission.body_positions_ecl_au(...)` cannot
  infer a type through the untyped `mission`, so Godot refused to load the `Sim`
  autoload and the pushed commit had no working frontend at all. The Rust half
  was green and that was taken for the whole. `test_orrery.gd` catches it in one
  run — the launch dies at `_init` with *"Failed to instantiate an autoload"* —
  and running it before committing is already ground rule 4 of the plan.
- **It was eager.** Calling the fill at the end of `_process` meant every frame
  of every view paid a crossing and twenty-four lookups whether or not anything
  would ask. The proof needs no timing at all: **in the porkchop view, which asks
  for no DE440 body, `ffi/frame` went 0 → 1.** The same waste ran through the
  whole build wait, where nothing is drawn yet, competing with the build worker
  for the same almanac — time to `mission_online` was 34.0 s with the fill off and
  56.0 s and 52.1 s with it eager.

It is now triggered by the first `pos_ecl` miss for an ephemeris body **at the
live clock**, and only there: an orbit walk or a trail asking one arbitrary epoch
would otherwise drag twenty-three bodies it has no use for across the binding.
`_primed_t` claims the epoch *before* the batch call, so the short-answer error
path cannot send the remaining twenty-three bodies back in for a batch each, and
it is cleared with the memo rather than compared against the clock — a paused
clock holds `t` still while the memo underneath it keeps emptying.

**No frame-time claim is made in either direction, deliberately.** Across four
runs of identical code paths a raw Earth lookup read 12.6, 84.6, 161.9 and
68.5 µs and one Earth orbit line 2.2 to 10.1 ms, while the memo hit stayed flat
at 1.3–1.5 µs throughout. The machine drifted on an axis that has nothing to do
with this change. The crossing count is deterministic and is the only number here
worth reading — which is also what made it the right evidence for the eager-fill
defect.

#### Task 1: the comet came off the critical path, and the wait fell 34.0 s → 25.2 s

The plan proposed running the build worker's *"three independent jobs"* — threat
propagation, sb441 mount, comet flight — concurrently. Measured
(`build_phase_timings`, an `#[ignore]`d timing test, release):

| phase | cost |
|---|---|
| mount sb441 | 0.6 s (warm) |
| `BuiltScenario::build` | **11.8 s** = 10.7 s forward nominal flight + 1.1 s remainder |
| comet | 4.1 s |

Two premises fail. **The mount is not independent** — `BuiltScenario::build`
consumes the mounted almanac and the scenario keeps it, which is what the `[P]`
force menu recomposes its perturbers from; building on the unmounted almanac
would leave that menu pointing at a field the threat was never flown in. That is
structural and needs no timing to establish. **And the comet had nowhere to
overlap**: the plan wanted it to run beside the "frame + perigee scan", which is
cache reads costing milliseconds — by then both expensive propagations have
finished, because `build_with` re-flies the nominal as its own round-trip hit
check, and *that* is the 10.7 s.

So the comet moved **off** the path rather than onto a parallel branch of it. The
scenario installs with the NEO tables only (file reads), the threat comes online,
and the comet flies on a worker started at install, landing into the catalog a few
seconds later. `scenario_arc()` already existed for exactly this shape — the
Tier-2 preview worker is handed the same `Arc` — so **nothing in `core/` changed**
and no scenario crosses a thread boundary by reference.

    time to mission_online   34.0 s → 25.2 s   (−8.8 s, −26 %)

Both runs had a healthy machine gauge (raw Earth lookup 12.6 and 11.8 µs, one
Earth orbit line 2.2 and 2.1 ms), which is the check that says the pair is
comparable at all. `test_orrery.gd` now reports the other half of the trade
directly: *"the comet lights from its own worker, 3740 ms after the threat"*, and
every check the suite makes between those two moments is reachable while the
comet is still flying.

**One failure mode is deliberately relaxed rather than preserved.** On the build
worker the comet's flight was fallible with `?`, so a comet that would not fly
took the entire threat solution down with it — over scenery, and against what
`sim.gd` has documented since 3D (*"the comet is a separate body that can fail to
fly on its own"*). It now warns and leaves the catalog without a comet: the stance
the small-body mount already took, that a missing catalog beats a missing threat.
Staleness is handled by refusal rather than a generation counter, matching
`begin_rebuild_scenario`'s existing rule — `busy_worker` names the comet flight,
so a threat rebuild cannot start underneath it and inherit a comet flown in the
previous field.

**Not claimed: a cold mount time.** The test reported 0.621 s cold and 0.685 s
warm — warm *slower* than cold, which means neither was cold on a machine that had
been reading those kernels all session. The plan's 5.7 s is neither confirmed nor
refuted here, and the decision never depended on it.

#### A separate item this batch uncovered and did not fix

**`_ready()` blocks ~11 s in `mission.load_from` on a cold file cache, on the main
thread.** It surfaced as a one-off `test_orrery.gd` failure — the worker-thread
check reading `_ready 10927 ms` against its 3 s bound — on the first run after a
launch that had died at parse time and so never warmed the kernels. It passes warm
and it passed on every subsequent run. But it is **larger than Task 1's entire
ceiling**, it sits outside the worker Task 1 parallelizes, and nothing about the
threaded build touches it: this is the 646 MB DE440 read, synchronous, before the
build is even started. On a first launch after boot it is most of what the operator
waits for. It is its own item, not part of Task 1.

#### The shot harness had to be taught to wait, and that is the check that closed this out

Ground rule 2 says look at the picture for every visual task, and Task 2's own
verify line says *"pictures identical"*. Running `_shot.gd` was skipped until last,
and it was the only check that failed: the harness reaches its comet section on
`mission_online`, which **no longer means the catalog is complete**, so it
photographed an empty `comet_el`, drove `pos_ecl` into the "every drawn body names
a source" guard and then a null node. Because that happens inside an `await`
chain it did not fail — it **hung** until the runner's timeout killed it, and took
every later shot with it. The harness now waits on `_comet_pending` before that
section and skips the two comet shots with a printed FAIL rather than crashing if
the comet never lands.

With that fixed, the full run confirms the three things only a picture can:

- **The comet is drawn.** `comet_online=true … node_visible=true`, glyph and label
  out in the field. This was the real risk of the split — the comet's node is now
  built on a *second* `mission_ready`, ~8 s after the first in a windowed run, and
  `test_orrery.gd` proves the catalog holds it but draws nothing.
- **The span gate still holds.** Past the end of its arc, `node_visible=false` —
  not sitting on the Sun.
- **The threat is unchanged through the new install path.** `|B| = 14 639 km`,
  `cap = 11 311 km`, `MISS - EARTH CLEAR` — the exact pair the plan names as the
  check. Phosphor trails and the 2D traces survive the extra `mission_ready` (they
  are cleared on it, and it now fires twice per boot).

The duplicated `TRACKING`/`NO DEFLECTION PLAN` lines visible in the event log are
**not** from the second emission: `jump()` re-arms every event later than the new
clock (`sim.gd`), and the harness scrubs back to t=0 before the trails shot.

#### Two operational notes

- **The gdext suite needs `--test-threads=4`.** All 37 tests each load a 646 MB
  kernel; running them at full default parallelism exhausts the commit charge on
  this machine and the run dies with `memory allocation of 32726016 bytes failed`,
  which looks nothing like a test result and is not one.
- **A parse error in an autoload hangs a headless run indefinitely** rather than
  exiting. The engine reports *"Failed to instantiate an autoload"* on stderr and
  then sits there; with stdout redirected through a pipe nothing is flushed, so
  the run reads as "still working". One sat for an hour at 30 s of CPU. Launch
  these through `Start-Process -RedirectStandardOutput` with a timeout, and read
  the `.err` file when a run produces no output.

**Files touched:** `godot/rust/src/lib.rs` (`body_positions_ecl_au`,
`spawn_comet` / `poll_catalog` / `is_catalog_building`, `busy_worker`),
`godot/rust/src/mission_core.rs` (`MissionCore::body_positions_ecl_au`,
`adopt_orrery_body`, `build_phase_timings`, and the comet test rewritten to the
new order), `godot/scripts/sim.gd` (`_prime_ephem_positions`, `_poll_catalog`),
`godot/tests/test_orrery.gd` (84 checks now, not 83).

### The kernel read taken off the main thread — 2026-09-07 session (roadmap item 7: the wait was real, the file it was blamed on was not, and the pictures caught what the tests could not)

*What is next* item 7 said the frontend blocks `_ready` on "the 646 MB DE440 read,
synchronous, before the build is even started". The wait is real. Almost nothing
else in that sentence was.

#### The premise, checked before it was acted on

`load_from` never touches 646 MB. It reads **`de440s.bsp`, 32 MB**, plus a 37 KB
constants file, then walks a span bisection. The 646 MB file is `sb441-n16.bsp`,
the small-body kernel, and arming it is `p.is_file()` — a path stat. Its mount has
been on the build worker since the day it was added.

The `_ready 10927 ms` observation the item was built on is real, but
`test_orrery.gd` timed **all of `_ready`** and one line inside it was blamed.
Nothing had ever timed the phases separately. So they are timed now, permanently:
`Sim.ready_phase_ms` is filled every boot and printed by `test_orrery.gd`. Warm:

    font 1.4  tier2_terms 0.0  instantiate 0.1  resolve 0.6  load_from 26.6
    arm_small_bodies 0.1  epochs_and_span 0.1  build_planets 0.0  build_events 0.0
    begin_build 0.1                                          (29 ms total)

Two candidates that had to be ruled out rather than assumed away, because both do
work that can be slow on a cold machine: the `SystemFont` fallback list (1.4 ms —
Godot resolves those names against the system font set) and `Kernels.resolve()`
(0.6 ms — `_scan_dir` matches filenames with `FileAccess.file_exists`, a stat, and
never opens the 646 MB file by a side door). Neither is the story. `load_from` is
92 % of a `_ready` that does no other I/O worth naming, which is what finally makes
the original attribution *checkable* — and it names the right line for the wrong
reason.

#### Why 32 MB takes seconds: the disk, measured rather than blamed

32 MB in 11 s is ~3 MB/s, which is not a plausible cold read of a local file, so
the arithmetic itself said something else was going on. Read directly from the
device with `FILE_FLAG_NO_BUFFERING`, so the answer does not depend on what the
cache happens to hold:

| read | cost | rate |
|---|---|---|
| `de440s.bsp` (32 MB), unbuffered | 6 375 ms | 4.9 MB/s |
| `de440s.bsp` (32 MB), buffered, cache already evicted | 4 598 ms | 6.8 MB/s |
| `de440s.bsp` (32 MB), unbuffered, second pass | 2 420 ms | 12.9 MB/s |
| `sb441-n16.bsp`, first 200 MB, unbuffered | 5 656 ms | 35.4 MB/s |

`M:` is a **spinning disk** (Toshiba HDWT380, SATA, `MediaType: HDD`). The last row
is the control: the same drive streams a large file at 35 MB/s, so 5–13 MB/s on the
small one is a seek profile, i.e. `de440s.bsp` is **fragmented**. And it is a real
read of the whole file — ANISE's `Almanac::new` is `std::fs::read` (`almanac/mod.rs`),
a heap read, not an mmap, so nothing is deferred to page faults.

**It is not antivirus**, which was the first alternative worth eliminating: Defender
caches its verdict per file, so a scan would have made the *second* read fast. Both
reads were slow.

None of this is fixable in code. What is fixable is *where the wait is spent*.

#### The change

`Mission::begin_load(bsp, pca)` reads the kernels on a worker and `poll_load()`
adopts the result — a ninth channel, and the earliest one, because it runs before
there is a core at all. `load_from` stays for tests and shell runs, which have
nothing to keep responsive. Arming the small-body kernel deliberately did **not**
move onto the worker: it is a stat, and leaving it on the main side keeps its
warn-and-continue branch (a machine without the 646 MB file is a valid machine, not
a failed mission) working exactly as written instead of being smuggled back through
the channel.

    _ready   29 ms -> 2 ms

That is the whole deterministic claim. **No cold before/after is offered**, because
a cold cache cannot be produced on demand here and manufacturing one would be worse
than not having it: what can be said is that the read measured 2.4–6.4 s off this
machine's disk and that none of it now lands on the main thread. `poll_load`
reports *"still reading"*, not *"it worked"* — `is_loaded()` is the success test,
and `test_gdext.gd` pins that pair along with the refusal of a second `begin_load`
and the fact that the threaded read returns bit-identical Earth positions to the
synchronous one.

#### The state the frontend did not used to have

`bodies_online` used to be settled before the first frame. Now there is a third
state — reading — and the boot POST reports it, which is the one thing a real
power-on self-test does that a snapshot cannot: `EPHEMERIS KERNEL ... DE440S.BSP
READING ...`, retyped to `LOADED` in place when the read lands. The pending and
settled branches fill the **same four line slots** so nothing below them shifts
mid-type, and `_chars` is a budget counted across the array, so everything already
on screen stays on screen.

Two details that only appear once you look at it:

- **The line named the wrong thing.** `kernel_source` is the *directory* the
  resolver answered from, so `kernel_source.get_file()` is the word `KERNELS`.
  Tolerable while the line only ever read "... LOADED"; beside "... READING" it is
  nonsense. `Sim.kernel_file` now carries `DE440S.BSP`.
- **`field_online` is emitted after `_begin_build()`, not before.** Emitting first
  retyped the POST in the one instant where the kernels were up and the build had
  not been kicked, and printed `DEFLECTION SOLVER ... OFFLINE - NO EPHEMERIS`
  directly under `DE440S.BSP LOADED`, over `ALL SYSTEMS NOMINAL`. Kicking first
  restores exactly what the synchronous path showed.

The READING screen was photographed by **holding the landing for 25 s** and taking
the shot, because warm the state lasts about two milliseconds and is otherwise not
photographable. The hold was removed afterwards; `boot_1_post` is a permanent shot
of whatever the screen actually reaches, and it waits on the typewriter rather than
a frame count — a four-frame settle photographs the copyright banner and nothing
this screen exists to report.

#### The regression the pictures caught, and the tests did not

Both headless suites were green — 85 checks in `test_orrery.gd`, 53 in
`test_gdext.gd` — and the frontend was broken.

`SolarSystem._ready()` builds its planet nodes inside `if Sim.bodies_online:`. That
flag was *always* true by then, because the load finished inside `Sim._ready()`.
Threaded, it is false, so the world built itself with no planet nodes; then
`bodies_online` went true and `_process` hit

    Invalid access to property or key 'MERCURY' on a base object of type 'Dictionary'

on every frame from then on. **The failure is not the error.** A GDScript error in
`_process` skips the rest of the function, so every line below it stopped running —
the sixteen belt asteroids froze at the origin (which in this heliocentric view is
the Sun, the exact failure this project has shipped three times and guards
everywhere), and the comet's span gate silently stopped being applied.

That last one is how it was found. `_shot.gd` prints `comet past span: ...
node_visible=true (must be false)`, and the previous session had recorded `false`.
It was confirmed as a regression rather than a flake by stashing the GDScript
changes and re-running against the same DLL: `node_visible=false` on unmodified
scripts, `true` with the change. The errors themselves went to **stderr**, which the
windowed run's output filter was not reading — the picture and one printed flag were
the only things that told the truth.

Fixed by building the things that read the field *when the field arrives*, matching
the `mission_ready` wiring already beside it: `SolarSystem` connects `_build_planets`
to `field_online` when the field is not up at scene load, and `main.gd`'s camera
focus ring — the other `_ready`-time snapshot of `bodies_online`, which would
otherwise have offered the Sun and nothing else for the rest of the session —
rebuilds the same way, clamping the focus index rather than resetting it.

`_shot.gd` now counts planet nodes against `Sim.planets` and prints a FAIL, so the
next thing that builds too early says so in one line instead of through a frozen
belt.

#### A third consumer of the placeholders, found by auditing the ordering rather than by a test

`_ready` still calls `_build_events()`, and it moved from *after* the synchronous
load to *before* the threaded one. `_build_planets()` survives that move because it
is pure data; `_build_events()` does not. It branches on `bodies_online` and prints
`year_at(T_MIN)`/`year_at(T_MAX)`, so at scene load it now takes its **last** branch
and opens the event log with `NO EPHEMERIS KERNEL - SOLAR FIELD OFFLINE` over a
kernel that is merely still being read.

Two fixes, both matching what the rest of this batch does: a `field_loading` branch
that says `READING DE440S.BSP - STAND BY`, and a rebuild from `_poll_load` on both
outcomes. Worth noting what the failure would have looked like if only the second
half had been wrong: the log line would have carried the placeholder span
(`-3650..40000` days, i.e. 2018–2137) instead of the mounted kernel's 1849–2150 —
a plausible-looking pair of years on a time bar nobody measures, which is the same
shape as the clock assertions below and equally invisible to a screenshot.

#### The check that was quietly deleted, and put back

`test_orrery.gd` asserts `date_string() == "2028-01-01"` and `T_IMPACT ≈ 4383`
straight after `_ready()`. Threaded, those read the **placeholders** — and
`EPOCH0_TDB` defaults to `883569600.0`, which is exactly what
`default_epoch0_tdb_seconds()` returns, while `T_IMPACT` defaults to `4383.0`
against a 2.0-day tolerance. Both would have passed whether the loader landed or
not, on values the core never supplied. The test now pumps `_poll_load()` to
completion and gates on `bodies_online` before that block.

The build-worker check moved for the same reason: `ready_ms` no longer covers the
build kick, and `load_ms` is the wrong replacement because it contains the disk
read, which is seconds on a cold cache and says nothing about whether the build
blocked. It reads `ready_phase_ms["adopt_field"]` — the landing frame, which is
where the kick now lives (0.2 ms).

#### A pre-existing flaky bound, found on the way past and deliberately left alone

`test_gdext.gd` asserts `begin_build_scenario()` returns in **< 1000 ms** — the
guard that says the ~10 s build is not on the calling thread. On this machine, in
one hour, that call measured **755, 2015 and 3551 ms**. It does nothing but clone
two `Arc`s and `std::thread::spawn`; everything expensive is inside the closure.
The spread is the machine committing a thread stack while the file cache is full of
646 MB kernels from a session of repeated runs.

It is **not this batch's doing**, and that was checked rather than assumed: with the
GDScript stashed and the same DLL, the **unmodified** test failed *harder* (3551 ms)
than the modified one (2015 ms) minutes apart. Two things were also ruled out along
the way — the new threaded-read block leaves a second 32 MB almanac resident, so it
is freed (`m2 = null`) and moved to the **end** of the suite, after every wall-clock
assertion; that fixed a companion `set_plan()` failure and did not move this one.

The bound is left at 1000 ms. It has ~10x of headroom against the failure it exists
to catch (a blocking build is 10-30 s, not 3.5), and loosening a real guard on the
evidence of one thrashing afternoon is the wrong trade. Recorded so the next person
who sees it red knows to check the machine first.

#### One lever that is the operator's, not the code's

`de440s.bsp` reads at 5–13 MB/s while the same disk streams 35 MB/s. Re-copying the
file would lay it out contiguously and cut the cold read substantially. That is a
machine-local action on a machine-local file, so it is recorded here rather than
done.

#### What was not seen

One of the eighteen shots, `enc_8_planner_keyhole`, was **not** rendered with the
fix in place: the run reached the end of the keyhole sweep and its 900 s budget
expired during the golden-section refinement, on a machine that was by then taking
86–140 s just to reach the first frame. Every sweep rung printed a normal result,
and the shot is reached long after `mission_online`, so the field wiring this batch
changed cannot get at it — but it is unverified rather than verified-clean, and is
recorded that way.

**Files touched:** `godot/rust/src/lib.rs` (`field_load`, `begin_load`,
`is_loading`, `poll_load`), `godot/scripts/sim.gd` (`ready_phase_ms`, `_phase`,
`field_loading`, `kernel_file`, `_pending_kernels`, `field_online` /
`field_load_failed`, `_poll_load`), `godot/scripts/boot.gd` (the third POST state,
`_on_field_settled`, `is_typed`), `godot/scripts/solar_system.gd` and
`godot/scripts/main.gd` (build on `field_online`), `godot/tests/test_orrery.gd`,
`godot/tests/test_gdext.gd`, `godot/tests/_shot.gd`.

### The labels that were drawn where they fell, and a frame-ms number that lied twice - 2026-09-07 session (plan tasks 3, 4, 5, 6)

`docs/plans/2026-09-05-visuals-performance-followups.md` had four tasks left.
All four are done; two of them turned out to be about something other than what
the plan said, and the session's real finding is a measurement one.

#### The placement rule, and the reservation that must not blink

`tag_layer.gd` and the b-plane's caption passes both drew each label at a fixed
offset from its glyph and hoped. Sixteen belt asteroids, eight planets, the NEOs
and the threat all offset 12 px right, so at system zoom the names sat on each
other. Both now **collect, place, then paint** - the glyph never moves, because
it is the measured position and the only thing on those layers that is a claim
about where something *is*; only the text slides.

**The rule that makes it stable is that a blinked-off label still reserves its
rectangle.** "PREDICTED IMPACT" blinks at 2.2 Hz in the 3D view and 1.6 Hz on
the b-plane. A placement pass that only saw what is currently painted would
re-solve every neighbouring label twice a second, and the picture would be
correct in any single screenshot and jittering in motion - which is precisely
the class of defect a shot harness cannot catch. `belt_1_real_asteroids.png` is
the case: 2028-01-01 is twelve years before impact, so Earth is back in almost
the same place and the *invisible* impact caption is what pushes "EARTH" up a
slot. The confirmation arrived free in the Task 6 shots - the 3:4 keyhole
caption sits in the identical place in frames where the impact text is drawn and
frames where it is not.

**The b-plane task named the second-worst collision.** It asked for keyhole
captions to clear the b-point captions, which was real. But zoomed out both
marks sit almost on Earth and **the two b-point captions want the same pixels**:
`enc_4_zoomed_out` printed "PREDICTED IMPACT" and "B 0.02 LD" on top of one
another. Not crowded - unreadable. Fixed in the same pass, stepping down a line
and never sideways so a caption stays attached to the mark it names.

#### The tracks, and the two requirements that do not compose

`_draw_track` issued one `draw_line` per segment - ~1400 per track, two tracks,
every frame. The plan asked for one `draw_polyline` per track *and* the
off-frame reject kept; those cannot both hold, because a single line would have
to include the off-frame points to stay connected. What ships is **runs of
consecutive on-frame points that share a colour**, broken on either of the two
things that make a segment differ from its neighbour: it leaves the grown rect,
or `s` changes sign (which is what dims the outbound half). Same segments, same
colours, ~4 canvas commands instead of 1400.

The cache is keyed on (zoom, size) and cleared in `_fetch`. Two invariants make
that complete and **neither is visible at the cache**, so both are now written
at `_track_runs`: `center` and `ppl` are pure functions of `size` and
`_half_ld`, so the key covers everything `_plot` projects through; and the
invalidation is correct **by draw order, not by construction** - `plan_changed`
clears `_built`, not `_runs`, and what saves it is that `_draw` calls `_fetch`
at the top and `_draw_track` further down. Move that call below the tracks and
the view silently shows the previous plan.

#### The finding: one frame-ms number produced two opposite wrong conclusions

This is the part worth carrying forward.

- One post-Task-3 run read the b-plane view at **6.94 / 7.26 ms**, against ~7 for
  every other view. Conclusion drawn: the plan's premise was stale, this is no
  longer the expensive view, Task 6 has nothing to win. **Wrong.**
- The next two runs read **10.17 / 10.32** and **10.38 / 10.70**. Conclusion
  drawn: Task 4 had cost 3.3 ms/frame. A width memo was written to fix it.
  **Also wrong** - and the memo measured no better, which is the only reason it
  did not ship.

Both errors are the same error: a single run compared against a single run, and
the 6.94 was the outlier. What settled it was swapping the pre-Task-4 file back
in (`git show <commit>:<path>`) and running twice against the same DLL:
**10.16 / 10.06 and 10.10 / 10.08**. Task 4 costs ~0.2 ms. The b-plane view
really is the expensive one, at ~10.1 against ~6.9 for the 3D views, and the
plan's original 9.1/9.2 baseline was right all along.

With that baseline established honestly, Task 6 measures **10.16 / 10.06 /
10.10 / 10.08 -> 9.11 / 9.20 / 9.23 / 9.07 ms** - no overlap between the sets,
`3d_system` unchanged at 6.90-6.92, micro gauges 11.8-12.5 us throughout so the
runs are comparable at all. That is -1.0 ms, -10%, **not** the -2 ms the plan
predicted.

The procedure is now written at the end of the plan file: two runs each side,
report all four, check the micro gauges before believing a delta, and A/B by
file swap when one looks large. Also recorded there: **the harness window is
64x64**, so anything gated on "is this label on the plot" is measured in a
regime nothing like the real screen - a caption cost can be invisible in the
harness and real in the game.

#### The persistence control, and a key that would have collided silently

`[I]` cycles `[0.14, 0.35, 0.0]` s. **Not `[G]`**, which the plan proposed: `G`
is already `tier2_term_gr` in the force menu, nothing reports a duplicate
binding, and the second one simply wins. `0.0` is a real rung rather than a
limit approached - a reader who wants to know where a body *is* rather than
where it has been needs the effect gone - and it is handled as a case, since the
decay formula is a division by zero there.

The scenery belt now fades as the clock speeds up, via a `dim` uniform on
`starfield.gdshader` set **only when the warp step changes**. The belt already
had its own material instance, so the stars are untouched.

**What the belt claim rests on, precisely:** the harness prints
`belt_dim_set_for_warp=9`, which floors `dim` at 0.15. It does *not* rest on
comparing the two trail pictures - the clock runs through the two key presses
and the settle, so `trails_1` is at t=1302 d and `trails_2` at t=3218 d. They
are a fair before/after for *persistence* (ghost chains vs none) and not for the
belt's phase.

#### What this leaves

Plan tasks 3-6 are closed; task 8's four small items are not started. Two loose
ends elsewhere are now on the *What is next* list rather than buried: the
keyhole-map circle radius is unbounded, and the Tier-3 ellipse is drawn at 1
sigma with the +/-3 sigma linearity shell never run against the drawn shape.

### Five keyholes flown, and the door that is not where the map draws it - 2026-09-07 session (roadmap item 4)

Roadmap item 4 asked for the keyhole width's order-unity slack on more than one
flown resonance. The answer splits in two, and only one half is about the width.

**The width is fine. The placement is not.** Five resonances were each aimed at,
flown to a return impact, refined to their timing floor, and then had both edges
of their door found by bisection - 5 keyholes, ~45 flights each, every number
below a flown trajectory and not a formula.

| resonance | grad a' | linearised door km | flown door km | door centre, km from the circle | in half-widths |
|---|---|---|---|---|---|
| 3:4 Minus  | 2.612e1 | 24.879 | 26.919 | +19.255 |  +1.5 |
| 5:7 Minus  | 9.694e1 |  3.893 |  4.781 |  +2.020 |  +1.0 |
| 7:10 Minus | 1.382e2 |  1.924 |  2.207 | -26.750 | -27.8 |
| 6:5 Plus   | 2.425e3 |  0.183 |  0.083 |  -2.228 | -24.3 |
| 2:3 Minus  | 2.640e2 |  3.413 |  4.846 |  +7.248 |  +4.2 |

**The headline, stated so it cannot be missed: four of the five doors do not
contain their own circle.** The intervals are `[+5.796, +32.715]`, `[-0.370,
+4.411]`, `[-27.853, -25.647]`, `[-2.269, -2.186]` and `[+4.825, +9.671]` km.
Only the 5:7 straddles zero. Everywhere else the map draws the circle outside
the door it is supposed to mark, which means the map can call a point CLEAR
while a rock flown from that point returns and hits Earth. The 1.64 half-widths
recorded for the 3:4 was never a conservative width - it was the placement
error, and it reproduces here at 1.548.

**The placement error obeys none of the three laws it could have obeyed.** It is
not a fixed distance (2.0 to 26.8 km), not a fixed number of half-widths (1.0 to
27.8), and - the discriminator worth running - not a fixed bias in `a'` either:
`offset x |grad a'|` comes out 503, 196, -3697, -5403 and +1914, a 30x spread
with both signs. So there is no constant to quote, and in particular **no
multiple of the width can express it**, because the placement error does not
shrink when the door does. The 7:10's door is 2.2 km wide and sits 26.8 km from
its circle; the 6:5's is 83 m wide and sits 2.2 km off.

**Why the widths differ from the prediction, and how that was proved rather than
fitted.** The refinement drives the return's timing coordinate to zero, so the
door is the chord the return can sweep in `zeta2` at whatever spatial offset
`xi2` it is stuck with. That predicts a shrink factor `sqrt(1 - (xi2/b_cap)^2)`.
The proof is not a curve fit: **b at all ten measured edges came out 11 240 +/-
60 km** - the returns preserve `v_inf`, so every one of them has to hit a disc
of the same size, and `sqrt(xi2^2 + zeta2^2) = b_cap` at an edge *is* the chord
statement, measured on ten independent flights. Divide the factor out and the
five width ratios collapse from 0.45-1.42 to **1.00-1.44**. The one case where
the linearised door is *wider* than the flown one (the 6:5) is entirely this:
its return sits 10 040 km off axis on an 11 250 km disc, so only 45% of the
chord is available. Consequence: since `b_cap` is common to all five, the
`zeta2` width is fully determined by `xi2`, so the residual 1.00-1.44 slack
cannot live in the return geometry - it is in the `b1 -> zeta2` amplification,
i.e. in `|grad a'|` itself. **That is the calibration item 4 asked for: the
linearised width is conservative by at most 1.44x once the return's own offset
is accounted for.**

**The check that says the edge finder is honest.** This repo has shipped the
`b`-against-`R_earth`-instead-of-`b_capture` bug more than once, so: the ten
edge returns land at 6268.6 to 6370.3 km against `R_earth` = 6378 km. The edges
graze the surface, which is what a door edge is, and they do it on the
*return's* own capture disc - not encounter 1's 11 311 km, not `R_earth`. Two
independent confirmations of the same thing, neither of them planned.

**What the survey overturned on the way.** The working assumption was that short
returns (low `h`) are the reachable impact keyholes. Screening 14 circles by
flying each once says otherwise: 5:7, 7:10 and 6:5 all floor closer than the
3:4, while 4:3 and 7:5 floor at 153 448 and 1 051 352 km of spatial offset and
can never be impact keyholes for this rock at this `xi`. The column that decides
it is `xi2`, the offset the timing refinement cannot remove; `h` predicts
nothing. Only those last two deserve "cannot be an impact keyhole" - the middle
band is "`xi2` too large for the timing to close", which is a statement about
this rock, not about the resonance.

#### The frontend constant did not survive

`godot/scripts/sim.gd` alerted when a plan lay within `KEYHOLE_ALERT_WIDTHS =
4.0` half-widths of a circle. That misses three of the five flown keyholes -
7:10 at 27.8, 6:5 at 24.3, 2:3 at 4.2. It is replaced by

    alert when |distance| <= KEYHOLE_PLACEMENT_KM + width/2,   KEYHOLE_PLACEMENT_KM = 100.0

additive, because the door has a real width *and* a displaced centre and the two
errors add. **The 100 km is chosen, not measured**, and the comment says so:
every aim in this campaign was taken at one point on each circle (`xi` = 6690
km), so 26.75 km is the largest error seen at five points, not a bound over a
drawn circle. The margin is bracketed from the other side by circle crowding,
checked in closed form with no flights (`probe_keyhole_placement spacing`): at
`KEYHOLE_MAX_YEARS` = 7 the closest two circles the frontend draws sit **402 km**
apart, so a 100 km band still names one circle unambiguously. That is a real
coupling - **at 20 years the tightest pair is 3.0 km**, and this constant would
have to shrink with the horizon.

The change is visible on a plan a player can actually dial. `_shot.gd`
golden-sections the impulse for the smallest margin and now prints both rules
off the same row: `dv=0.13313 at 900 d (margin 1.5 km, d 1.845 km, door 0.718
km, old rule 5.14 half-widths vs cut 4.0) -> ** 1 KM OFF 5:8 - INSIDE THE
PLACEMENT BAND (alert=true)`. The old rule scored that plan 5.14 half-widths
against a cut of 4.0 and would have printed CLEAR. Both numbers are printed
because the first version of this claim was arithmetic done in my head and
written up as an observation - which is the failure this file has recorded
twice already.

Two smaller consequences. The panel now names the circle nearest **in
kilometres** rather than the one nearest in half-widths, because placement
dominates; the note flags the disagreement when they differ, the other way round
from before. And the label lost its third register: it no longer says "IN THE
KEYHOLE - RETURN SET UP" when a plan lands inside a drawn door, because four of
five flown doors did not contain their circle, so being inside the drawn band is
neither necessary nor sufficient. It reports a distance and whether the map can
still tell.

#### Two fixes in the core this needed

`KeyholeSolution::is_impact_return` compared the return's **already focused**
closest-approach distance against **encounter 1's** capture radius. Anything
returning between 6 378 and 11 312 km - a 5 000 km band - was reported as an
impact keyhole while being a clean miss. It now asks the return's own encounter
`is_hit()`, with a kernel-free regression test pinning the band. The shipping
3:4 answer is unchanged (1 087 km is inside Earth either way), so nothing
published moves; the survey is what would have been wrong.

`refine_keyhole_return` was split out of `solve_keyhole_return` so a caller that
already knows a bracketing aim can skip `required_dv`'s ~18 campaign re-flights.
That is what made a five-resonance survey affordable at all, and it is safe
because the aim only has to bracket and `bracketed` is reported. Every aim in
the survey landed within 75 km of its target on b-values of tens of thousands of
km, so sampling `b(dv)` once on a 28-rung ladder and inverting it cost nothing.

#### What this leaves

**[Closed 2026-09-07 - see *The ranking that was a ratio*.]** The core's
`tightest_keyhole` still selects by half-widths, which this finding
makes the wrong metric - it will systematically prefer wide circles over close
ones. The frontend now works around it by naming `nearest` instead, so nothing
on screen is wrong, but the core API answers a question that no longer matters
much. One consequence to know about before touching the panel: `keyhole_note`
prints its disagreement line when the widths-tightest circle is not the
km-nearest one, and now that the label follows kilometres **that branch is the
common case rather than the exception**. It is not visibly wrong - the harness
run printed the fallback text every time - but it is a mostly-live branch now,
not a mostly-dead one, and it will start firing as soon as the two metrics part
company. **[Closed 2026-09-07: both rows are kilometres now, the branch is
executed by `test_orrery.gd`, and the two rankings are measured never to part
company on this rock.]** Second, the placement error is measured at exactly one `xi` per circle;
whether it varies along a circle is unknown, and that is what a sixth campaign
should ask. Third, nothing here explains *why* the placement error is what it
is - it is bounded and characterised, not derived.

### The two drawn claims - 2026-09-07 session (roadmap item 8, and one of its halves named the wrong layer)

Item 8 was the last numbered entry before Phase 3, and it read as tidy-up: "the
keyhole-map circle radius is unbounded, and the Tier-3 ellipse is drawn at 1
sigma only - the +/-3 sigma linearity shell has never been run against the drawn
shape. Both are small, both are noted where they live." Neither half closed the
way that sentence expected. One was smaller than stated and lived in a different
file than the one it accused; the other was larger, and the number that was
supposed to answer it had been answering a different question all along.

The pattern is now unbroken across that whole list: item 1 had three wrong
clauses, item 4 was two questions under one name, item 5's premise was wrong,
item 6's proposed fix would have deleted a correct number, item 7 named the wrong
file. **An item's text is a pointer to where to look, never a description of what
is there.**

#### Half one: the circle radius, which was never the core's problem

The item says the radius is unbounded, and it is - but `core/src/keyhole.rs` is
right about that. `OpikFrame::resonant_circle` already refuses the degenerate
case (`denom.abs() < 1e-12`: the resonance that *is* the incoming orbit, whose
level set is a line and not a circle), and everything it returns is finite and
true. The published map (`docs/keyhole_map.json`, 168 circles) has a widest of
**9.232e6 km** - twenty-four lunar distances - for the 15:19 resonance, centred
9.229e6 km down the zeta axis. That circle **encloses Earth's centre**, so its
ring passes about **3 000 km** from Earth: it is genuinely in frame at the
deepest zoom while being enormous. That combination, not the radius alone, is
the defect.

The defect lived in `godot/scripts/encounter.gd`, in one line:

```gdscript
var segs := 96 if r < 2000.0 else 256
draw_arc(cc, r, 0.0, TAU, segs, col, 1.4 if is_three_four else 1.0)
```

`draw_arc` spreads its points around the **whole** circle, so the tessellation
was sized in *angle* while the error it commits is in *pixels*. Measured by the
new `godot/tests/test_geometry.gd` at the zoom-in stop (`_half_ld` clamps to
0.01) on a 1280x720 viewport:

| | old: 256 whole-circle segments | new: clipped, 0.3 px budget |
|---|---|---|
| radius on screen | 795 431 px | same |
| chord length | ~19 500 px | fits the window |
| **distance from the true circle** | **59.9 px** | **0.113 px** |
| points drawn | 256 | **3** |

Fifty-nine pixels on a 720 px view, on the one screen whose entire job is where a
line falls relative to a disc. And the fix is *cheaper* than the bug: the visible
window of that circle needs three points.

Two more things the rewrite settled.

- **The old cull could only see one of the two ways a circle misses the view.**
  `if cc.distance_to(center) - r > rect.size.length(): continue` catches a ring
  that passes beyond the corners. It cannot catch a ring so large that the whole
  plot sits *inside* it - and that is exactly the shape the widest resonances
  have, so those were being drawn as 256 points of nothing every frame. The new
  `circle_polyline` culls on both `d - r > vr` and `r - d > vr`.
- **The offline map never had this defect.** `tools/keyhole_map_svg.py` emits a
  real `<circle>` element inside a `<clipPath>`, so the renderer draws a true
  circle and clips it. The two pictures had been disagreeing, and only the
  interactive one was wrong.

The geometry moved to a new file, `godot/scripts/plot_geometry.gd`, and the
reason is worth recording because it cost a run to discover: **a headless
`--script` run registers no autoloads**, so a script that names `Sim` fails to
*compile* in isolation, and `encounter.gd` names `Sim` on nearly every line. The
first version of the test loaded `encounter.gd` and got `Identifier not found:
Sim` before it could call anything. Pure geometry in its own file, loaded by
`preload` rather than `class_name` (a new global class needs an editor rescan -
see the staleness traps), is testable with no kernels, no build and no window.

`test_geometry.gd` runs 17 checks in about a second. Four are worth naming: the
drawn chords meet the budget; **every in-frame point of the true circle is on the
drawn line** (clipping is only honest if it drops nothing visible - worst
0.1172 px); both culls fire; and the point cap is never what bounds the picture
(sweeping eight decades of radius, the worst case is **78 points at r = 354 px**
against a cap of 1024).

#### Half two: the ellipse's shape, and the number that could not see it

`RealFieldScenario::bplane_uncertainty_checked` has returned a `LinearityReport`
since Tier 3 began, so the item's "has never been run" was about *pointing it at
the drawn ellipse*, not about building anything. Doing that turned up the reason
it would not have helped if someone had.

**`max_relative_residual` normalises against the shell's largest flown
displacement.** On the shipping ellipse - 168.71 x 0.82 km, a 205:1 needle - that
scale *is* the major axis. So a residual big enough to be most of the 0.82 km
minor axis, i.e. big enough to mean the needle is not that thin, divides down to
a per-mil number and reads as "linear". The scalar is correct for what it was
built for (the impact probability, which the major axis dominates) and
structurally blind to the shape. Measured at the sigma knob's top stop: the
scalar says **0.0043**, the minor axis says **0.561**. A factor of **130**.

So `probe_tier3_drawn_shape` resolves the same residuals along the drawn
ellipse's own axes and sweeps the `[Z]`/`[X]` knob across its full `10^+/-3`
range, shell at 3 sigma (97 propagations, 117 s):

```text
   scale    sig_maj km   sig_min km    resid km   of major   of minor   scalar
    1e-3        0.169       0.0008      0.0302     0.0596      0.355   0.0801
    1e-2        1.687       0.0082      0.0102     0.0020      0.012   0.0029
    1e-1       16.871       0.0818      0.0190     0.0004      0.002   0.0005
     1e0      168.710       0.8182      0.0121     0.0000      0.001   0.0000
     1e1     1687.103       8.1822      0.1869     0.0000      0.006   0.0001
     1e2    16871.034      81.8218     15.4984     0.0001      0.056   0.0004
     1e3   168710.343     818.2179   1528.4543     0.0013      0.561   0.0043
```

`of minor` is the number nobody had read: the residual's component along the
minor axis, as a fraction of that axis' own 3-sigma half-width. Above 1.0 the
view would be drawing a width narrower than the error in it.

**The verdict is reassuring. The first reading of it was wrong.** The column is
U-shaped, worst at *both* ends, and the naive read - "the picture is least
trustworthy when the orbit is best known" - is backwards. Re-running at
`forward_rtol = 1e-13` instead of the shipping `1e-9` separates the two:

| | scale 1e-3 | scale 1e2 |
|---|---|---|
| residual at rtol 1e-9 | 0.0302 km | 15.4984 km |
| residual at rtol 1e-13 | **0.0003 km** | **15.3276 km** |
| `of minor` at 1e-13 | 0.355 -> **0.003** | 0.056 -> 0.056 |

A hundredfold drop at the small end and nothing at the large end. The residual is
a flat **integration-noise floor** plus a term growing as the square of the
scale; the floor is the integrator, the square-law part is real curvature, and
only the second is physics. The probe now fits and prints both
(`residual = floor + k * scale^2`) instead of quoting one number. This is the
same shape of finding as the integrator-convergence batch: a number that looks
like a property of the model turning out to be a property of how it was computed.

With the floor understood, the answer to the item: **the drawn shape is supported
everywhere the knob reaches.** At the shipping covariance the minor-axis residual
is 0.001 of the drawn 3-sigma half-width. At the top stop it is 0.561 - the drawn
width is still 1.8x the error in it - and the linear extrapolation crosses 1.0 at
scale ~2e3, outside a knob that stops at 1e3. `core/tests/tier3_drawn_shape.rs`
pins both ends (37 propagations, 60 s), and pins one more thing: that the
per-axis number stays at least 10x the scalar, because the day those two agree,
this test has stopped adding anything.

**One instinct corrected on the way.** The decomposition looked like it needed the
display rotation, for the reason `Tier3View` exists - an ellipse's orientation is
not invariant under the sensitivity's arbitrary basis. It does not: the residual
and the axis are expressed in the *same* basis, and a common rotation leaves
their dot product alone, so both ratios are invariant scalars. What genuinely
needs the rotation is the printed *angle*. The probe does it anyway, which
re-measures the two b-planes' agreement as a side effect (2.2e-16 here, against
the 1.95e-10 the ellipse batch recorded - this probe builds the Opik frame from
the sensitivity's own reduction, so the two share `S-hat` exactly).

`bplane_uncertainty_checked_many` is new and is what made a sweep affordable: one
13-propagation sensitivity and one sampling *plan* shared across n shells. The
plan sharing is not an optimisation - `sensitivity_with_plan`'s own doc warns
that two plans that drift apart make the shell difference its displacements
against a mean measured at another epoch, and report the difference as curvature.
Seven separate calls would have been seven plans.

#### What this leaves

- **Nothing changed on screen for the ellipse, deliberately.** The shell says a
  3-sigma ring would be honest to draw; it is not drawn, because at the default
  zoom the 1-sigma major axis is already about one pixel, and the readout prints
  the minor axis to 0.01 km - coarser than any residual measured here. The
  finding is that the existing picture is sound, not that it needs more on it.
- **The drawn covariance's three constants now exist in three places** - the
  binding (`TIER3_*` in `mission_core.rs`), `probe_keyhole_map`, and this
  probe/test pair - with nothing enforcing agreement, because core cannot see the
  binding. A change to what the frontend draws has to be made in all three or the
  probe silently answers about an ellipse nobody draws.
- The circle clip treats the view as its **circumscribing disc**, not its
  rectangle, so it keeps slightly more arc than needed near the corners. A
  deliberate trade: an exact rectangle clip is four line-circle intersections and
  a case analysis, to decide how much of a ring to tessellate.
- The guard test costs 60 s with kernels - the right cost for the claim, but it
  is now among the slowest in the suite.
- `cargo clippy -D warnings` reports three pre-existing `neg_cmp_op_on_partial_ord`
  lints in `core/src/sbdb.rs` (628, 736). CI does not pass `-D warnings`, so it
  stays green; nothing in this batch touches that file.

### The ranking that was a ratio - 2026-09-07 session (the two live threads the placement batch left)

The five-door placement campaign closed roadmap item 4 and left two notes about
its own aftermath. They are the same thing seen from two sides: `core` still
ranked resonant circles by **keyhole widths** (`tightest_keyhole`), which the
campaign's central finding makes the wrong key, and the planner's note line still
printed that ranking's disagreement with kilometres - a branch the batch recorded
as "a mostly-live branch now, not a mostly-dead one", i.e. one that had never
been seen to fire and was about to start.

#### Why a ratio is the wrong key, in one line

The campaign measured two things about a linearised door and got opposite
verdicts. The **width** calibrates: at most 1.44x conservative across five flown
doors. The **placement** does not: the door centres sit 2.0 to 26.8 km from their
own circles, following none of the three laws tried. So the placement error is
*additive* - a number of kilometres that lands on every circle alike - which is
exactly why the frontend's alert is `|distance| <= 100 km + width/2` and not a
multiple of anything.

Dividing an additive error by a door's width is the one operation guaranteed to
hide it. `widths_away` ranks a 200 km-wide door 300 km away (1.5 widths) ahead of
a 25 km door 60 km away (2.4 widths), when the second is the one inside reach of
a 100 km placement error and the first is nowhere near it. The doc on
`widths_away` said the opposite in as many words - *"This, not the metres, is the
honest way to rank two keyholes against each other"* - and that sentence had been
copied into four more places, including `keyhole_label`'s own docstring, which
claimed it named a resonance "in keyhole widths" while its body had already
switched to kilometres and an inline comment three lines below said so.

#### What shipped

`KeyholeProximity::margin()` is the key: `|signed_distance| - half_width`,
kilometres outside a door, negative once inside it. Same units as the placement
band, with each door's own width already taken out, so a kilometre of margin
means the same thing at a wide door as at a narrow one.
`OpikFrame::smallest_margin_keyhole` ranks by it and pairs with `nearest_keyhole`
as an edge pairs with a locus: nearest *circle* is the number to print beside the
drawn map, nearest *door edge* is the number an alert is cut on. The binding row
gains `margin_km` and the readout's second row is **renamed** `tightest` ->
`at_risk`, renamed rather than quietly redefined so that nothing reads a key
whose meaning moved under it. `sim.gd`'s `keyhole_margin_km` now *reads*
`margin_km` off the row instead of recomputing `|distance| - width/2`, because a
second copy of the formula could rank one way while the row it was handed was
chosen the other; the binding test pins that the two agree.

`tightest_keyhole` stays, with an honest doc. Deleting it would cost the only
comparison the note line has to make, and "which door is wide relative to how far
away it is" is still legible - it is just not the risk ranking.

#### The measurement that stopped this being written up as a bug fix

The obvious claim is that the old alert could miss a door: it was cut on the
*nearest circle's* margin, so a wider door slightly further out - a smaller
margin, elsewhere in the census - would not have fired it. True of the code, and
worth fixing. Not, on this rock, something anyone could have hit.

The core sweep now runs **2 000 random Öpik geometries** and, across all of them,
**the nearest circle was also the nearest door every single time**. Two rankings,
zero disagreements. The same holds on both plans the binding test flies (the
flown 3:4 keyhole: nearest 3:4 at 7.9 km outside its door, at_risk 3:4, the same
circle; the default plan: 3:5 both ways) and on the closest plan `_shot.gd` can
dial (5:8 both ways, margin 1.5 km).

That is **reported by the test, not asserted by it**. An assertion that they
*must* disagree is what the first draft contained, and it failed - which was the
useful outcome, because it forced the honest statement to be the measured one.

"Never", though, is a near-tie and not a structural fact. The runner-up came
within **9.026e-6 capture radii** of winning, against doors as wide as
**5.327e-2** in the same units - at the shipping encounter's 11 311 km capture
radius, about **0.1 km against 600 km**. The orderings are not the same ordering;
they simply never parted company in the sample. So `smallest_margin_keyhole` is
the principled key rather than a caught miss, and the code says so where it
matters rather than implying a bug was found.

**A sampling trap surfaced on the way, and it is the reason the first sweep found
nothing.** That test had been drawing its b-points from a box of +/-3 capture
radii while the census it queries reaches **60** - so it only ever asked about the
crowded near field, and keyhole widths grow with distance, which makes the far
field the only place the two rankings could plausibly differ. It is now
log-uniform in radius over [0.5, 40]. Widening it did not change the answer; not
widening it would have meant the answer was never asked for.

#### The branch nobody had seen fire

`keyhole_note`'s disagreement line was recorded in the previous batch as a branch
that would start firing and had never been observed. Per the above it still never
fires on this rock - so `test_orrery.gd` now executes it directly, by handing the
readouts a hand-built `plan_keyhole` dictionary instead of a solved plan. That is
legitimate because `plan_keyhole` is plain data the core hands over and the
readouts do nothing but format it; what is under test is the formatting rule, not
the physics that produced the numbers, and a real plan stays live underneath so
`has_plan()` and the solving / clean-miss gates are still answered by the real
thing. (It needs `_tick_plan_debounce` drained first, or the readouts correctly
answer `SOLVING...` and every check reads an empty string - which is how the
first run of them failed.)

Four checks, and the first text this branch has ever produced:

```text
PASS  one circle both ways -> the placement caveat, not a disagreement line
        (CIRCLE PLACED TO +/-100 KM - ONLY A FLOWN RETURN CONFIRMS)
PASS  a wider door further out is named by the note (20 KM FROM THE WIDER 5:8 DOOR)
PASS  the alert and the line it prints name the SAME circle - the one the alert is
        cut on (** 300 KM OFF 5:8 - INSIDE THE PLACEMENT BAND, alert=true)
PASS  a negative margin reads as inside, not as a negative distance
        (INSIDE THE 5:8 DOOR - WIDER, FURTHER OUT)
```

The third of those is a change in behaviour and not only in wording: the alerting
line used to name the nearest *circle* while the blink was cut on that same
circle's margin, so the two could not disagree - but neither of them was the row
that would fire first. Both now read `at_risk`.

`_shot.gd` prints which circle each ranking named and whether they agreed, so a
future run printing the fallback text is distinguishable from one that exercised
the branch. On the closest plan a player can dial it reports `nearest 5:8 (margin
1.5 km) | at_risk 5:8 (margin 1.5 km) -> agree`, and every other number in that
line is unchanged from the previous session (dv 0.13313 at 900 d, d 1.845 km,
door 0.718 km, old rule 5.14 half-widths) - which is the correct outcome for a
change that reorders nothing here.

#### One calibration moved out of the ranking's way

The binding test's `(1.2..2.2).contains(widths_away)` is the only place the flown
3:4 measurement lives - 1.64 half-widths from the circle, on a door this rock
demonstrably returns through, which is what says the linearised width is
conservative by about 1.6x. It was being read off *whichever row a ranking
returned*, so a physics measurement from the flown campaign was riding on a
ranking decision. It now looks the 3:4 up by name in the readout's own census
(1.636 widths, 20.4 km from a 24.9 km door). Had it not been moved, this batch
would have silently repointed it.

**Checks:** core 27/27 (`-p asteroid_core --lib keyhole`), the kernel-gated
binding test green in 51 s (a 0.02 s run means it skipped), `test_orrery.gd`
**0 FAIL** with the four new checks among the passes, `_shot.gd` windowed writing
`enc_8_planner_keyhole.png` and printing the keyhole block with no FAIL line, and
`cargo fmt --check` clean. The pass *count* is deliberately not quoted: this run
prints 89 against the 83 recorded above, the four checks here account for four of
the difference, and the rest was not chased - a number in this file is supposed to
be a measurement, and that one would not have been.


### The calibration taken outside its own domain - 2026-09-07 session (roadmap item 4's sixth campaign)

The five-door placement batch closed item 4 and wrote down what it could not
answer: every door had been flown at **one point of one circle**, so the
placement error was characterised per circle and never *along* one. The shipping
`KEYHOLE_PLACEMENT_KM = 100.0` rested on those five points and said so in its own
comment - "a CHOSEN margin, not a measured bound".

That question is now answered, and the answer moved a shipping constant by 5x.

#### The knob was not the one this looked like it needed

The obvious way to move along a resonant circle is a **sideways impulse**: the
circle spans xi, an along-track nudge barely moves xi, so add an out-of-plane
component. Priced before building, that design is bad in three separate ways. It
needs metres per second to shift xi by thousands of km (the out-of-plane leverage
is ~140x weaker); at that size it shifts `v_inf` at encounter 1, which moves the
**true** circle away from the drawn one, and that shift would land inside the
measured placement error as a frame artefact; and `fly_keyhole_shot` applies
`scalar x one unit direction`, so a fixed sideways component cannot be held while
the refinement searches the magnitude.

None of that was necessary. `MissionCore::set_plan` takes **two** arguments -
`lead_seconds` and `dv_along_track` - and `sim.gd` exposes both (`LEAD_MIN/MAX` =
30..900 d, `DV_MIN/MAX` = 0.1..300 m/s). So the set of encounter-1 b-plane points
a player can occupy is a **2-D patch, not a 1-D curve**, every resonant circle is
crossed by a whole family of `(lead, dv)` pairs at different xi, and
`KeyholeAiming.deflection_epoch` was already a field. The lead sweep needs no new
core API, no sideways impulse - and no frame confound at all, because the Opik
frame is built from the **nominal** encounter, which does not depend on the lead.

#### The gate, and what it cost to be wrong about a branch

Flying a door is ~45 flights and ~12 minutes, so the campaign was gated first:
`probe_keyhole_placement xi_sweep <h> <k> <branch> [leads...]` flies no returns
and measures no doors. It finds where the along-track curve crosses one chosen
circle at each lead and reports how far apart in xi the crossings are, against the
27 km largest placement error already measured. The three outcomes and what each
would mean were written into the stage's doc comment **before** it was run.

Its first run was wrong, and its own columns said so. A vertical line cuts a
circle **twice**, and the scan took the first sign change of `signed_distance` -
which is the near crossing at short leads but not at the 12 yr lead, where
`dv = 0.01` already starts past it. The output put a `b` of 7 859 km beside one of
153 448 km and doors 240x apart and called them four samples of one circle. Two
things made that visible rather than plausible: the door width is printed at every
crossing, and so is `b`. The near crossings sit at 7 154-7 859 km against this
encounter's 11 311 km capture radius, i.e. **inside the capture disc** - they are
impacts at the first flyby, not keyholes at all. `on_branch` now checks each
crossing against the circle's centre and scans past a wrong-branch one, logging it.

With that fixed the gate is unambiguous. On the 3:4 (circle radius 74 855 km,
centred at zeta = -78 722 km):

| lead | dv (m/s) | xi at the crossing | linearised door there |
|---|---|---|---|
| 4383 d (12 yr) | 0.2165 | +6 104 km | 24.88 km |
| 900 d | 0.9308 | +4 390 km | 24.91 km |
| 450 d | 1.2519 | -15 811 km | 24.64 km |
| 150 d | 3.2235 | -52 860 km | 21.29 km |

**58 964 km of spread**, an arc of 64 825 km on a circle of radius 74 855 - about
a third of the way round it, and 590x the whole placement band. Three of those
four leads are dialable.

**xi is not a smooth function of the lead**, and this is recorded as observed
rather than modelled. Eight leads inside the planner's range give +4390, +906,
-9310, +4952, -4379, -15811, +5537 and -52860 km at 900, 800, 700, 600, 500, 450,
300 and 150 days. It oscillates, with no law offered and none measured. Do not
read the four-row table above as a drift.

#### Three doors on one circle, and the fourth that does not exist

Each surviving lead was then flown the full way: aim, refine to the return's
timing floor, bisect both edges of the door.

| lead | dialable? | door centre from the circle | flown door | linearised / flown |
|---|---|---|---|---|
| 4383 d | **no** | +19.255 km | 26.919 km | 0.924 |
| 900 d | yes | **+210.632 km** | 27.913 km | 0.891 |
| 450 d | yes | **+467.869 km** | 20.407 km | 1.219 |
| 150 d | yes | **no door** | - | - |

Same resonance, same circle, same probe, same closed form. The placement error
moves by **24x** across the leads a player can dial. At 150 d it stops being an
impact keyhole entirely: the return floors 17 411 km out with a spatial offset
`xi2` of -23 263 km that no amount of timing can close, so far enough round the
circle the door does not merely move - it ceases to exist.

**The width survives, and that is the point of splitting the two.** Across all
eight doors now flown the linearised width is 0.89x to 1.44x the flown one. The
width/placement split the five-door batch drew is exactly right; it is only the
placement half that was under-measured, and it was under-measured in a direction
nobody had looked.

The edge-finder passes its own honesty check at every new lead: the four new edge
returns land at 6 293.6, 6 306.2, 6 308.8 and 6 334.3 km against `R_earth` =
6 378 km. Door edges graze the surface, and they do it on the **return's** own
capture disc.

#### The headline is not that it varies. It is where it was measured.

`sim.gd` sets `T_IMPACT := 4383.0` days - the campaign lead *is* the whole
timeline - and `lead_cap()` clamps to `LEAD_MAX = 900`, with `set_plan` clamping
again to `[30, <=900]`. So the 12 yr lead is **4.9x beyond the longest lead the
planner will accept**, unconditionally and for every `t`, not just for some.

Every one of the five doors that set `KEYHOLE_PLACEMENT_KM = 100.0` was flown
there. The constant was calibrated outside the domain it is consumed in, and
inside that domain it is between 2.1x and 4.7x too small. A plan at a 900 d lead
with a retrograde 0.932 m/s nudge - both ends of that dialable - sits 210.6 km
from the 3:4 circle, scores 198.2 km of margin, and **flies the rock back into
Earth three years later** while the old rule printed `CLEAR - NEAREST 3:4 IS 210
KM OFF`. That plan is now a kernel-gated binding test.

#### The crowding argument was taken at the same un-dialable point

The 100 km was bracketed from above by circle crowding: at `KEYHOLE_MAX_YEARS` =
7 the closest two drawn circles sit 402 km apart, so a 100 km band still names one
circle. That number was also measured at xi = 6 690 km - the nominal encounter's
own, which is the 12 yr lead's. `spacing` now takes an `xi=` override, and at the
xi the dialable leads actually reach:

| xi (km) | reached at | closest pair of drawn circles |
|---|---|---|
| +6 690 | 12 yr (not dialable) | 402.2 km |
| +4 390 | 900 d | **83.6 km** |
| -15 811 | 450 d | **8.3 km** |
| -52 860 | 150 d | 692.5 km |

So in parts of the map the drawn circles are closer together than the error in
where they are drawn, and no distance rule can name one resonance there.

**And the honest scope of that, which a failing assertion supplied.** The first
draft of the binding test asserted that the flown 900 d plan would find several
doors inside the band, reasoning from the 83.6 km figure. It failed: the readout
counts **1**. 83.6 km is the tightest pair *anywhere* in that xi's census, at some
other impact parameter entirely, while this plan sits at `b` = 153 722 km where
the neighbours are far apart. The crowding finding is real but narrower than it
first looked - a band honest about placement cannot separate circles *somewhere*
on the map, not everywhere - and the assertion is now the measured `>= 1` with the
count reported. This is the second batch running in which the useful outcome was
an assertion that would not pass.

#### What shipped

`OpikFrame::doors_within_band(circles, p, band)` counts every door whose margin
is inside a caller's band. The band is the caller's, not the core's: it is a
statement about how far the *map* can be trusted, which is presentation.
`MissionCore::keyhole_readout` takes it and returns `doors_in_band` alongside the
two rows, so the panel can tell the difference between naming a circle and
picking one of several.

`KEYHOLE_PLACEMENT_KM` is **500.0** - the smallest round number above the largest
placement error measured - and its doc comment is rewritten rather than appended
to, because the old caveat ("nothing here bounds the error over a whole drawn
circle") is now answered rather than still open. It says in as many words that
this is a **measured maximum over eight doors, not a bound**: three leads on one
circle and five circles at one lead is not a law, nothing here explains why the
error grows as the lead shortens, and the 150 d column says a door can vanish
rather than merely move.

The panel gains a third register. Inside the band with one door it reads as
before; inside the band with several it reads `** 300 KM OFF 7:8 - AND 2 MORE
DOORS IN THE BAND`, which is the panel declining to present one of several as the
answer. Outside the band it still reads CLEAR - the band grew, it did not become
unconditional, and an alert that is always on says nothing. The note line now says
`CIRCLE PLACED TO +/-500 KM AT THIS LEAD`, naming the lead because one figure
demonstrably does not cover the slider.

Every stage of `probe_keyhole_placement` now takes `lead=<days>`, writes one
ladder **per lead** (`keyhole_ladder_<lead>d.tsv` carrying a `# lead_days =`
header), and `Ladder::load` **refuses** a ladder flown at a different lead rather
than interpolating the wrong `b(dv)` curve - a file with no header is refused too,
since "assume it is the default" is the silent wrong-curve read the check exists
to stop. The ladder's dv span scales with the lead ratio, and its comment says
that factor is **a bracket, not a law**: the crossing dv came out 4.30x the 12 yr
value at a 4.87x shorter lead and 14.9x at a 29.2x shorter one, so the 1/lead
scaling it is borrowed from is wrong by 2x at the short end. The 3:4 cross-check
at the end of the ladder stage is now gated to the campaign's own lead, because
both numbers it compares were measured there and it read as a 6x failure at 450 d
when it was the leverage changing.

#### What a 5x wider band did to the app, measured

A band five times wider could have traded a false CLEAR for an alarm that is
always on - the failure the retired constant's own comment warned about ("if two
circles sit closer together than that band, the alert is on permanently and says
nothing"). `_shot.gd` says it did not:

```text
SHOT  planner keyhole: ** 174 KM OFF 3:5 - INSIDE THE PLACEMENT BAND (alert=true)
SHOT  keyhole sweep dv=0.05 -> CLEAR - NEAREST 3:5 IS 1,656 KM OFF
SHOT  keyhole sweep dv=0.15 -> CLEAR - NEAREST 7:11 IS 787 KM OFF
SHOT  keyhole sweep dv=0.90 -> CLEAR - NEAREST 3:4 IS 4,996 KM OFF
SHOT  keyhole sweep dv=1.40 -> CLEAR - NEAREST 3:4 IS 76,071 KM OFF
SHOT  closest a player can dial: dv=0.13313 at 900 d (margin 1.5 km, d 1.845 km,
      door 0.718 km, old rule 5.14 half-widths vs cut 4.0)
      -> ** 1 KM OFF 5:8 - INSIDE THE PLACEMENT BAND (alert=true)
SHOT  doors inside the 500 km placement band: 1
```

Three readings. **CLEAR is still reachable on plans a player dials** - the whole
impulse sweep reads it, at 787 to 76 071 km off - so the band grew without
becoming unconditional. **The closest dialable plan did not move**: `dv=0.13313 at
900 d`, 5:8, margin 1.5 km, every digit unchanged from the previous session, so
nothing recorded about it is stale. And **`doors_in_band` is 1 on real physics**,
which is why the constant's doc says in as many words that the several-doors case
is inferred from the closed-form spacing sweep and has not yet been seen on a
flown plan.

The one visible change is the default plan, which flips from CLEAR to `** 174 KM
OFF 3:5 - INSIDE THE PLACEMENT BAND`. That is the intended effect and not a
regression: 174 km is well inside the placement error now measured at dialable
leads, so the map cannot honestly call it clear. What the panel claims there is
"too close for this map to tell", which is true, rather than "safe", which was not.

**Checks:** the core's `doors_within_band` unit test (kernel-free, and it executes
the several-doors branch the flown plan does not reach), the kernel-gated binding
test `a_dialable_plan_that_returns_is_not_reported_clear` (reproduces the probe's
+210.6 km to 0.1 km and pins that the retired 100 km band would have called it
CLEAR), five new checks in `test_orrery.gd` covering the crowded register, the
single-door register, CLEAR outside the band and the constant itself, plus
`cargo fmt` and no new clippy warnings.

#### What this leaves

The placement error is now known to depend on the deflection lead, and **nothing
here says why**. Three points on one circle plus five circles at one lead is a
measurement, not a model, and the 500 km is the largest of those three - a plan at
a lead between them, or on another resonance, may be worse. The obvious next
question is whether the error is a function of the *lead* or of the *xi the lead
lands on*, which the eight-lead sweep cannot answer because xi is not monotone in
the lead: the two are entangled in every point measured so far.

Second, `doors_in_band` is measured to be 1 on the only plan flown to a return,
so the crowded register ships tested but not yet observed on real physics. The
sweep says there are places on the map where it must fire; none of them has been
flown.

Third, the 150 d column - a door that stops existing as you move round a circle -
is recorded and not explained. `xi2` is the coordinate that decides it, and how
`xi2` depends on where on the circle you enter is exactly the question the return
half of this layer has never been asked.

### The lead was the variable - 2026-09-07 session (roadmap item 4's two live threads)

The lead sweep closed item 4 and left two sentences behind it that were not
results:

> The placement error is now known to depend on the deflection lead, and
> **nothing here says why**. Three points on one circle plus five circles at one
> lead is a measurement, not a model.

> `doors_in_band` is measured to be 1 on the only plan flown to a return, so the
> crowded register ships tested but **not yet observed on real physics**.

Both are now answered, and answering them moved a shipping constant a second
time, in the same batch that made it 5x bigger last session.

#### Two proxies had to die before "the lead" meant anything

"The error depends on the lead" was never a safe sentence, because on one
resonant circle a shorter lead drags two other things with it. It slides the
plan **round the circle** (the along-track curve crosses the 3:4 at
xi = +6104 km at 12 yr and at -52 860 km at 150 d), and it forces a **bigger
impulse** for the same `b`. Every point measured before this session had all
three moving together, so "the lead" was one of three candidate labels for the
same ladder. Two flights separate them - **in sequence, not independently**: the
first rules out the position, and only then does the second rule out the
impulse.

**It is not where on the circle.** The 3:4 door flown at a **300 d** lead lands
at xi = +5531 km; the 12 yr door lands at +6103 km. That is 572 km apart on a
circle of radius 74 855 km - **0.48 degrees of arc**, the two shots at
phi = -85.80 and -85.32 degrees - and the map is wrong about one of them by
**+19.3 km** and about the other by **+648.2 km**. Same circle, same branch,
same probe, essentially the same place on it, 34x the error. The lead between
them is 14.6x.

**It is not the size of the impulse either.** That was the other live reading,
and it survived the 300 d flight (dv had gone 0.2165 -> 2.8798 as the error went
19 -> 648). It does not survive the **200 d** one. The dv a circle costs is
*not* monotone in the lead - the 200 d door floors at **2.4785 m/s**, less than
the 300 d door's 2.8798 - and its placement error is **larger**: **+786.0 km**
against +648.2. Impulse down, error up, on the same resonance. (**Not** at the
same point of it - see the next paragraph, which is the whole reason this pair
is second in the order and not first.)

**The order of those two eliminations matters and the second does not stand
alone.** The 200 d shot lands at xi = -21 412 km, so it is *not* at the same
place on the circle as the 300 d one; taken by itself that pair confounds the
impulse with the position again. It is only decisive *after* the same-phi pair
has shown the error is not a function of position alone. What survives both is
simply this: of the four candidate variables, **only the lead orders all five
flown doors.**

| ordered by | values across the five doors | monotone with the error? |
|---|---|---|
| lead | 4383, 900, 450, 300, 200 d | **yes** |
| dv | 0.2165, 0.9322, 1.2519, 2.8798, 2.4785 m/s | no |
| xi | +6103, +4386, -15812, +5531, -21412 km | no |
| phi on the circle | -85.3, -86.7, -102.2, -85.8, -106.4 deg | no |

So the ladder is indexed by the lead itself:

| lead | dialable? | dv (m/s) | xi at the door | door centre from the circle | flown door | linearised / flown |
|---|---|---|---|---|---|---|
| 4383 d | **no** | 0.2165 | +6 103 km | +19.255 km | 26.919 km | 0.924 |
| 900 d | yes | 0.9322 | +4 386 km | +210.632 km | 27.913 km | 0.891 |
| 450 d | yes | 1.2519 | -15 812 km | +467.869 km | 20.407 km | 1.219 |
| 300 d | yes | 2.8798 | +5 531 km | +648.174 km | 27.506 km | 0.904 |
| 200 d | yes | 2.4785 | -21 412 km | **+785.988 km** | **11.812 km** | **2.106** |
| 150 d | yes | - | - | no door at all | - | - |

Read it as five flown doors on one circle. It is not a law and no law is offered.

#### The mechanism that should have explained it, and does not

The best candidate was the **frame**. A resonant circle is a function of the
encounter - `c = mu_earth/v_inf^2`, the angle `theta` between the incoming
asymptote and Earth's velocity, and Earth's own heliocentric state - and the
probe, `MissionCore::keyhole_readout` and the drawn map all build that frame from
the **nominal, undeflected** encounter, then place the **deflected** point on it.
The deflection changes all three, and all three grow with the impulse. That is
the right shape for the ladder, so it was flown rather than argued
(`probe_keyhole_placement frame`, one flight per lead, no doors and no
refinement). The decision rule was written into the stage's doc comment before
it ran.

It is wrong, and the way it is wrong is informative. Rebuilding each flight's
circle in that flight's own frame does not collapse the spread; it **enlarges**
it.

| lead | door centre d0 | frame term dd | d0 + dd | of which v_inf/theta | of which timing | d0 and dd from |
|---|---|---|---|---|---|---|
| 4383 d | +19.255 km | +226.333 km | **+245.588 km** | +15.145 km | +211.187 km | one flight |
| 900 d | +210.632 km | +125.641 km | **+336.273 km** | -82.888 km | +208.529 km | one flight |
| 450 d | +467.869 km | +885.810 km | **+1353.679 km** | +721.955 km | +163.855 km | **two flights** |
| 300 d | +648.174 km | -367.904 km | **+280.270 km** | -580.596 km | +212.692 km | one flight |
| 200 d | +785.988 km | +834.626 km | **+1620.614 km** | +683.819 km | +150.807 km | one flight |

If the frame were the mechanism, the `d0 + dd` column would be **one number**
belonging to the 3:4 circle. It is not: over the four same-flight rows it spreads
**1375.0 km**, against 766.7 km for the uncorrected `d0` and a 24.9 km door. The
correction makes the answer **1.79x worse**. (The 450 d row is marked because its
`d0` is a door centre while its `dd` was read at the crossing dv, at
phi = -102.2 degrees rather than -85; since the correction contains
`-d_zeta_c*sin(phi)` that mismatch is exactly the term that swings, so the stage
now flags such rows and reports the spread with and without them. Here it changes
nothing - both spreads are 1375.0 km - but it had to be checked rather than
assumed, and on the four-row table it was the 200 d row that made the negative
unambiguous.)

**The frame is nonetheless a large effect, and one sub-term of it had never been
measured at all.** The deflected pass reaches closest approach **~1.9 h after**
the nominal impact; in 1.9 h Earth moves ~210 000 km; and that alone moves the
3:4 circle's placed point by **~+200 km**. It is +211.2, +208.5, +163.9, +212.7
and +150.8 km across the five leads - very nearly a constant, because all five
sit at nearly the same `b` (141 900 to 153 700 km) and slip by nearly the same
1.6 to 1.9 h. That is a **closed-form
reason for a band of hundreds of kilometres** arrived at without flying a single
door - and it is precisely the wrong shape to explain a ladder, because it does
not move with the lead.

It is a **sensitivity, not a correction**. The flown 12 yr door sits +19.3 km
from the nominal frame's circle and +245.5 km from its own frame's, so the map's
choice is empirically the better of the two and rebuilding the frame per plan
would make the reported number worse. It is also not free, which is why
`keyhole_readout` is left alone: it and `keyhole_circles` are bound to each other
by live assertions, and a plan-dependent frame would move the drawn circles as
the player drags the lead slider. That is a design decision to be taken
deliberately, not a side effect of an explanation task.

**The binding test that was supposed to cover this had two blind spots**, both
found by the probe and both now closed. It compared the two frames' circles by
`|dzeta_c| + |dR|` and asserted the sum was under a quarter of a door - but the
readout's number is a *signed distance*, which also moves when the axes rotate
under the b-vector, and on that very plan the circle parameters agree to well
under a door while the placed point moves **+15.1 km**, over twice the threshold
being asserted. And it built **both** frames on the nominal clock, so the +211 km
timing term - the dominant one - had never been in the test at all. The block now
walks the arrival slip (0 s, 3600 s, the measured 6927.4 s), asserts on the
placed point, and names the lead every number belongs to. It reproduces the
probe to three decimals.

#### The band moved again, and this time the unmeasured region is below it

`KEYHOLE_PLACEMENT_KM` is **800.0**. 200 days is dialable (`LEAD_MIN` = 30) and
786 km is outside the 500 km the last session shipped, so the same failure that
session's test pins - a plan the map calls CLEAR that flies the rock back into
Earth three years later - **recurred one lead down, at a constant that had just
been raised 5x to prevent it**. That plan is now a binding test of its own.

The constant's doc says in as many words that this is a measured maximum over ten
doors and **not a bound, and that the unmeasured region is BELOW the last row
rather than between rows**. The 3:4 has no door at 150 d, so its ladder stops at
200; nothing has been flown between 30 and 200 days on any resonance; and the
trend is still climbing where the measurements run out.

**The width claim moved too, and it is the first door to break it.** The
linearised width had been within 0.89x-1.44x of the flown one over nine doors.
The 200 d door is **11.812 km flown against 24.879 km drawn - 2.106x**. The
drawn width in every one of these ratios is `keyhole_at(circle, aim.target)`, i.e.
evaluated at the **aim point** (the nominal encounter's own xi = 6690 km), which
is the convention all ten doors were measured under and so the one the 0.89-2.11
range belongs to. It is not the width the panel prints, which is evaluated at the
plan's own closest point on the circle - the readout reports **24.4 km** for this
plan, giving 2.066x. The two differ because the flown shot does not sit at the
xi the aim held. That is
where the 3:4 door is closing: it does not exist at all by 150 d, and its
return's irreducible sideways offset `xi2` is already -10 300 km at 200 d. A
closing door narrows faster than the linearisation knows. The formula stays
**conservative** - it draws the door wider than it is, so the panel alerts where
it need not, which is the safe direction - but "within 1.44x" was a nine-door
statement and is retired.

#### The crowded register fired, and the search that found it had to be fixed twice

The several-doors register - the panel declining to name one resonance when the
band holds more than one door - shipped last session tested on a hand-built row
and **never observed on real physics**. `probe_keyhole_placement crowding` looked
for it: fly the whole dialable dv range at a lead in both directions, interpolate
the flown curve, scan 40 000 points of it in closed form for free, then fly the
candidate windows. The census is the **readout's** (2..=7 yr, k <= 24,
b <= 60 x capture radius), because a window among circles the frontend does not
draw would prove nothing about the frontend.

**Two things had to be fixed before its answer was worth anything.**

*The evidence that motivated the register was in the wrong metric.* The 402 /
83.6 / 8.3 km figures came from `spacing`, which measures gaps between
neighbouring circles along a line of **constant xi**. The register cuts on
`margin` - the distance from the **plan's own point** to a door - and a plan
reaches a given `b` at its own xi, not at the one the spacing sweep asked about.
This is the same class of error as ranking circles by widths instead of
kilometres, which the previous session fixed one layer up. It is also why the
first sweep found nothing at 900 d and 600 d while `spacing` implied crowding was
everywhere.

*And the first run's answer was worthless.* Ungated, it reported the register
firing at 300, 150 and 30 days - and **every plan it found sat at b between
4 593 and 9 526 km against an 11 311 km capture radius, i.e. still an impact**.
Circles crowd near Earth because that is where every resonance's circle has to
pass, so an ungated search finds its answer there every time and the answer means
nothing: a plan that has not yet turned the hit into a miss has no keyhole
question to get wrong. The stage now keeps only points with `!enc.is_hit()` and
reports how many rungs it rejected.

Gated, and at the shipping band, **the register fires on a genuine miss a player
can dial.**

| lead | Δv found (m/s) | b at the plan | miss by | doors in the band | runner-up's best margin on the whole curve |
|---|---|---|---|---|---|
| 900 d | +0.10935 | 16 938.5 km | 1.50x capture | **2** | 644.9 km |
| 600 d | +0.16756 | 16 966.5 km | 1.50x capture | **2** | 647.3 km |
| 450 d | +0.12187 | 15 123.9 km | 1.34x capture | **2** | 543.6 km |
| 300 d | +0.33958 | 17 014.1 km | 1.50x capture | **2** | 649.7 km |
| 150 d | +0.21480 | 12 383.0 km | 1.09x capture | **2** | 393.7 km |
| 30 d | +2.07657 | 11 637.5 km | **1.03x capture** | **2** | 696.3 km |

Every lead the planner allows, on a prograde nudge of a tenth of a metre per
second - `DV_MIN` is 0.1 - and at four of the six the plan is at b ~ 17 000 km,
half again the capture radius, which is a comfortable miss rather than a
technicality. The 30 d row is not: 11 637 km against an 11 311 km capture radius
is **a miss by 3 %**, and it is quoted as such.

And this is where the band width earns the register. The runner-up door's best
margin anywhere on the 900 d curve is **644.9 km** - between the retired 500 km
band and the shipping 800 km one. That is exactly why last session's search found
nothing and this one finds two doors at every lead: the register did not start
firing because the physics changed, it started firing because **the band widened
past the runner-up**. A wider band is a weaker claim about which resonance you
are near, and the panel now has to say so.

The claim is kept to what that measures: **the panel cannot name one resonance
there**. It does not say two impact keyholes exist at that point - that would
need two returns flown to Earth and four edge bisections, which is a different
question and about 50 minutes of flights. And a negative now carries a distance:
where no window exists the stage reports the runner-up door's best margin
anywhere on the curve, so "not reachable" says by how much rather than just
"no".

#### What a band 8x the original did to the app, measured

A band eight times the original 100 km could have traded a false CLEAR for an
alarm that is always on - the failure the very first version of this constant
warned about in its own comment. `_shot.gd` (windowed; it takes screenshots, and
under `--headless` it hangs at the first capture rather than failing) says it did
not:

```text
SHOT  planner keyhole: ** 174 KM OFF 3:5 - INSIDE THE PLACEMENT BAND
      | CIRCLE PLACED TO +/-800 KM AT THIS LEAD - ONLY A FLOWN RETURN CONFIRMS
SHOT  keyhole sweep dv=0.10 -> CLEAR - NEAREST 3:5 IS 1,656 KM OFF
SHOT  keyhole sweep dv=0.15 -> ** 787 KM OFF 7:11 - INSIDE THE PLACEMENT BAND
SHOT  keyhole sweep dv=0.20 -> CLEAR - NEAREST 2:3 IS 4,249 KM OFF
SHOT  keyhole sweep dv=0.90 -> CLEAR - NEAREST 3:4 IS 4,996 KM OFF
SHOT  keyhole sweep dv=1.40 -> CLEAR - NEAREST 3:4 IS 76,071 KM OFF
SHOT  closest a player can dial: dv=0.13313 at 900 d (margin 1.5 km, d 1.845 km,
      door 0.718 km, old rule 5.14 half-widths vs cut 4.0)
      -> ** 1 KM OFF 5:8 - INSIDE THE PLACEMENT BAND (alert=true)
SHOT  doors inside the 800 km placement band: 1
```

Three readings, the same three the 500 km change was measured on. **CLEAR is
still reachable across the whole dialable impulse sweep** - 1 656 to 76 071 km
off - so the band grew without becoming unconditional. **The closest plan a
player can dial has not moved**: `dv=0.13313 at 900 d`, 5:8, margin 1.5 km, every
digit unchanged across two band changes, so nothing recorded about it is stale.
And exactly **one verdict flips**, `dv=0.15`, from `CLEAR - NEAREST 7:11 IS 787 KM
OFF` to `** 787 KM OFF 7:11 - INSIDE THE PLACEMENT BAND`. That is the intended
effect: 787 km is inside the placement error now measured at a dialable lead, so
the map cannot honestly call it clear.

`doors_in_band` reads **1** on this shot's own plans, which is not a contradiction
of the crowding result above - the crowded region is at b ~ 17 000 km on a
*prograde* nudge and none of the shot's plans sits there. The register has been
seen on real physics; it has not been seen on *these* plans, and the count is
printed so the two claims stay separable.

#### What shipped

- `probe_keyhole_placement frame` - the two-term model, its decision rule written
  before the run, the three-frame split (map's / this flight's geometry on the
  map's clock / this flight's own), and a `same_flight` provenance flag on every
  row so a mixed-provenance row cannot quietly widen a spread.
- `probe_keyhole_placement crowding` - the gated search, the free 40 000-point
  scan between flown rungs, the miss gate, and the runner-up margin that makes a
  negative quantitative.
- `KEYHOLE_PLACEMENT_KM` 500 -> **800**, its doc rewritten rather than appended
  to: the ladder by lead, the two proxies killed, the frame measured and
  rejected, the metric caveat on the old crowding evidence, and the crowded
  register recorded as **measured** rather than inferred.
- A binding test `the_placement_error_follows_the_lead_and_not_the_place_on_the_circle`
  that flies both the 12 yr and the 300 d plan, pins that they land within 0.48
  degrees of arc of each other, pins the 34x error ratio, and pins that the
  retired 500 km band would have read CLEAR on the 300 d one.
- The frame block of `the_keyhole_readout_finds_the_three_four_door_the_probe_flew`
  rewritten onto the placed point and across the arrival slip.

#### Checks

Core `--lib` **262/262** with kernels required (`ASTEROID_REQUIRE_KERNELS=1`, so
nothing skipped silently). The three kernel-gated binding tests green -
`the_keyhole_readout_finds_the_three_four_door_the_probe_flew` (which now
reproduces the probe's +15.145 and +226.331 km frame terms to three decimals),
`a_dialable_plan_that_returns_is_not_reported_clear`, and the new
`the_placement_error_follows_the_lead_and_not_the_place_on_the_circle`, which
re-solves all three plans in the binding and gets +19.2, +648.2 and +786.0 km
against the probe's +19.3, +648.2 and +786.0. `test_orrery.gd` **0 failures**,
including the crowded register, the single-door register, CLEAR outside the band
and the constant itself. `_shot.gd` windowed with no FAIL line. `cargo fmt
--check` clean; no new clippy warnings (the nine on the lib are pre-existing
`neg_cmp_op_on_partial_ord`).

Two harness notes worth keeping. `test_orrery.gd` `extends SceneTree` and is run
with `godot --headless --path godot --script res://tests/test_orrery.gd`, **not**
through `run_harness.ps1` - as an autoload it fails with "does not inherit from
Node" and then sits until the timeout. And `_shot.gd` must run **windowed**; under
`--headless` it reached the first screenshot and hung.

#### What this leaves

**The mechanism is still open, and it is now a much sharper question.** The
variable is the lead. It is not where on the circle, not the impulse, and not the
frame - and those were the three things that could have been read off the
existing machinery. Whatever it is, it is something about applying the impulse
*closer to the encounter* that the closed form does not model, at fixed `b`,
fixed circle, and nearly fixed v_inf (the four flights agree on v_inf to
4e-4). The obvious next place to look is the incoming heliocentric orbit itself:
Opik's construction assumes the rock arrives on an orbit the circle was drawn
for, and a late impulse changes that orbit's own shape more per unit of b-plane
displacement than an early one does.

**The band is not a bound below 200 days**, and the app allows 30. Flying a door
on some resonance at 100 d and at 50 d would say whether 800 survives, and the
150 d result says the honest answer there may be "the door does not exist",
which is a different kind of answer and worth having.

**The three-door register is still only geometry.** Two doors has now been flown;
three has not.

### The band's domain floor, measured - 2026-09-07 session (below 200 days there is no door to be wrong about)

`KEYHOLE_PLACEMENT_KM` = 800 rested on doors flown at 200 d and up. The planner's
slider goes down to **30 d**. Its own comment said so - "a plan dialed between 30
and 200 days is outside everything that calibrated this number" - and that was the
last open half of the constant.

It is measured now, and the answer is not a bigger number. **Below 200 days there
is no door for the band to be wrong about.**

#### What was flown

Three circles - the 3:4, 2:3 and 5:7 - swept at 150, 125, 100, 75 and 50 d, then
the crossings at 125 d refined all the way to their floors. All in the **nominal**
Opik frame, which belongs to the undeflected rock, so it is the same circle at
every lead and the comparison is a comparison.

| lead | 3:4 Minus | 2:3 Minus | 5:7 Minus |
|---|---|---|---|
| 150 d | crossed, dv 3.2238, xi -52 866 km | crossed, dv 0.8831 | crossed, dv 1.5966 |
| 125 d | crossed, dv 4.4103, xi -68 990 km | crossed, dv 1.2602 | crossed, dv 2.2220 |
| 100 d | NOT REACHED - gate at dv 37.5 | wrong branch, NOT MEASURED | wrong branch, NOT MEASURED |
| 75 d | NOT REACHED - gate at dv 65.3 | NOT REACHED | NOT REACHED |
| 50 d | NOT REACHED - gate at dv 150 | NOT REACHED | NOT REACHED |

And the three doors at 125 d, the lowest lead where anything is still crossed:

| circle | floor dv | return floors at | xi2 (spatial) | zeta2 (timing) |
|---|---|---|---|---|
| 5:7 | 2.2274920 | 29 341 km | -35 526 km | **2.9 km** |
| 2:3 | 1.2610756 | 25 140 km | -31 229 km | **58.1 km** |
| 3:4 | 4.4540499 | 19 447 km | -25 380 km | **187.9 km** |

**Every one is converged, and that is what makes the negative a result.** A return
that merely lands far out could be a search that stopped early. These have spent
their timing - `zeta2` is 2.9 to 187.9 km against a `xi2` of 25 000 to 36 000 km,
shares of 0.000 to 0.007 - so the miss is the two orbits' own sideways offset, and
no impulse along this curve removes it. Resonant returns, not impact keyholes.

So the door **ceases to exist between 150 and 200 days on the 3:4** - which is the
only circle with a measurement on both sides of that gap (door at 200 d, none at
150 d). The 2:3 and the 5:7 have **no door at 125 d and were never asked above
it**; what they add is that the 3:4 is not a special case at 125 d, not a second
and third bracketing of where the door dies. The constant does not move; it gains
a stated domain floor it never had.

**What this does NOT claim.** The census has 168 circles and three were flown. "No
keyhole below 150 d" is not established and is not what the constant's doc now
says. What is established is about the circles swept, at a scan ceiling that
reproduces a known crossing - which is the amendment the decision rule took before
any result arrived, because the original wording ("no resonance yields a door")
would have needed every resonance in the census flown.

#### The control, which is the only reason the negative is worth anything

A method that finds nothing has to be shown to find something. The same
`dv=`-aimed path, run at 200 d where the door is on record, reproduces it to three
decimals: centre **+785.973 km** against the recorded +786.0, edges **+780.068**
and **+791.879** against +780.1 and +791.9, flown width 11.812 km, and the return
**HITS** at 5 514 km. The path finds a door when there is one.

#### Three gates that had to be discarded or fixed on the way

**`screen` is not a door-existence gate, and a control said so before any
conclusion rested on it.** Run at 200 d and 300 d - both leads with a known, flown
3:4 door - it reports **zero hits**, with returns at 478 709 and 576 838 km. One
unrefined shot at the ladder's aim lands half a million kilometres from Earth even
where a door exists. Its zero hits at 50-125 d were discarded rather than written
down. The stage's own footer says the column to read is `xi2`, not `hit?`.

**The ladder's aim is unusable below ~150 d, and the geometry says why.** It
matches the circle's `b` at the *nominal* xi. On the 3:4 that point is
b = 153 424 km against a circle whose largest possible `b` is 153 577 km - it is
essentially the circle's outermost point, reachable only near xi = 0. At a short
lead the curve gets to that same `b` far out in xi. Flown at 125, 100, 75 and 50 d
it returned `bracketed = false` four times, ending **214 729 to 320 646 km**
outside the circle, every one sitting at 2.27x its aim, which is exactly
`reach_fraction` - the wall, not a floor. Hence `dv=`, which aims a door from the
crossing `xi_sweep` measures.

**A bug in `xi_sweep`: a hole in the curve was being reported as its end.** The
scan stopped at the first flight that returned nothing and called it the 5e8 m
scan gate. At 50 d the retrograde curve passes **through Earth** - the ladder
reads b = 762 km at dv 1.7586 - and the flight fails there. The scan stopped at
dv 1.7787 and printed `NOT REACHED`, on a curve the ladder had already flown out
to b = 306 654 km at dv 70. That reads as a reachability finding and is a hole in
the middle of the sweep. `Ok(None)` is a terminus (past the gate the pass has
swung so wide there is no encounter left, and every larger impulse is wider);
`Err` is one bad impulse to step over, with `prev` cleared so no bracket spans the
discontinuity. With the fix the 50 d scan runs the full range to 150 m/s and still
crosses nothing - and the 100 d and 75 d rows are **unchanged**, so only 50 d had
been truncated.

**And the scan ceiling was itself a calibration.** `SWEEP_DV_HI` = 30 m/s, whose
doc says "well past what the 1/lead law needs at 150 days" - and means it. The
3:4's own aim needs 19.0 m/s at 75 d and 37.7 at 50 d, so below 150 d the ceiling
stopped the scan before the circle and printed `NOT REACHED`. Now `dvmax=` and
`rungs=`, documented as a pair because raising the span without adding rungs
coarsens the geometric spacing the sign change has to be caught in.

#### The check that gated the whole sweep

Raising the ceiling makes the rungs coarser in *ratio*, and the curve cuts every
circle **twice**. One rung pair straddling both crossings leaves the signed
distance the same sign at both ends, and the sweep prints `NOT REACHED` with a
plausible "last distance" - indistinguishable from the reachability finding this
batch is about. So the 150 d row had to reproduce the recorded crossing before any
other row was read. It does, on all three circles: the 3:4 reads dv **3.223849**,
xi **-52 865.7 km** against the recorded 3.2235 and -52 860.

#### One number that must not be banked

The 125 d 3:4 floor sits "+1 112 km from the circle", past the 800 km band. **That
is not a placement error.** Placement is the midpoint of two flown door *edges*,
and there are no edges where there is no door - it is only where the refinement
ended while chasing a return minimum.

#### What shipped

- `probe_keyhole_placement`: `dvmax=` / `rungs=` on `xi_sweep`, `dv=` on `door`
  (which prints which aim it used and what the ladder would have said), the
  gate-vs-failure fix, and `SHIPPING_BAND_KM` / `LARGEST_PLACEMENT_ERROR_KM`
  replacing a summary line that had been printing "KEYHOLE_PLACEMENT_KM = 100 km"
  through both changes of that constant.
- `core/src/keyhole_target.rs`:
  `the_three_four_door_has_ceased_to_exist_by_a_125_day_lead` - one flight at the
  refined floor, asserting the return is outside the capture disc **and** that its
  timing is spent, because the first assertion alone would pass on an unconverged
  shot that happened to land far out.
- `KEYHOLE_PLACEMENT_KM`'s doc: the constant unchanged at 800, with the ladder
  extended downward, the domain floor stated, and an explicit line that three
  circles out of 168 is not "no keyhole below 150 d".

#### What this leaves

**The mechanism behind the placement error is still open** - it was open before
this batch and this batch did not touch it. What is now known is that the question
has a bounded domain: the error only exists where a door exists, which on these
circles is 200 d and up.

**Whether other resonances have doors below 200 d is unasked.** Three circles of
168 were swept. A circle whose geometry puts its crossing at a small xi at a short
lead is the place to look, and `xi_sweep` is cheap enough to screen the whole
census before flying anything.

---

### The scalar's blindness, measured and moved into the module - 2026-09-08 session (item 8's leftover, and a factor that was never the aspect ratio)

Item 8 closed on 2026-09-07 with a finding it did not act on: the number that was
supposed to say whether the drawn uncertainty ellipse is still supported at 3 sigma
reads **130x too small** on the axis that matters. What it left behind was not a
wrong picture - nothing on screen was ever wrong - but a **shape number living in
two copies outside the module**, one in `core/examples/probe_tier3_drawn_shape.rs`
and one in `core/tests/tier3_drawn_shape.rs`, while the only public verdict
anyone can call, `LinearityReport::holds_within`, still reads the blind scalar.
A number that exists only inside the probe that discovered it is a finding, not a
guard.

#### What moved

`LinearityReport::shape_residual(&BPlaneUncertainty) -> Option<ShapeResidual>`.
It resolves the same shell residuals along the mapped ellipse's own principal
axes and returns both ratios, both axis lengths, and the major axis as a unit
vector. Four decisions inside it are worth naming.

- **A method, not a constructor argument.** `LinearityReport::new` has three call
  sites that build a report from a hand-made Jacobian with no `BPlaneUncertainty`
  anywhere - the pure-math unit tests. A constructor that demanded an ellipse
  would have forced those to fabricate one, and they exist precisely to test the
  report without the propagator.
- **One eigendecomposition.** Both copies used to take the *lengths* from
  `sigma_axes()` and the *directions* from a second `symmetric_eigen()` beside it.
  Two derivations of one fact that happened to agree.
- **`Option`, not a division.** `sigma_axes` clamps eigenvalues at zero, so a
  degenerate covariance gives an exactly zero half-width, and both old copies
  divided straight into it. `new` already guards `shell_scale > 0.0`; this now
  matches.
- **`major_hat` returned.** Without it the probe still needs its own
  eigendecomposition to print the ellipse's drawn angle, and the deduplication is
  half done. With it the probe rotates one vector.

`holds_within` is deliberately **unchanged**: its own doc argues the scalar is the
right test for the probability, which the major axis dominates, and three
assertions rest on that. `ShapeResidual::holds_within` is a sibling, not a
replacement. Nothing was plumbed to Godot - the report has never reached the
binding, and the previous batch's "nothing changed on screen for the ellipse,
deliberately" still holds.

#### The finding: 130 was never the aspect ratio

The obvious reading of "the scalar normalises against the major axis" is that it
under-reads the width by the ellipse's aspect ratio. On the shipping drawn ellipse
that would be **205:1**, and the measured factor is **130**. The gap is not
rounding, and chasing it turned up a wrong sentence in this batch's own first
draft of the docs.

`shell_scale` is **not** the major axis. It is the largest *flown* displacement
across the twelve shell points, and those points are the **state** covariance's
principal axes - six directions in the 6-D initial state, whose b-plane images do
not line up with the 2-D mapped ellipse's own axes. Measured on the shipping
covariance: the shell reaches **350.9 km** against a 3 sigma half-length of
**506.1 km**, i.e. **0.693** of it, and **143x** the 2.455 km half-width.

So the under-reading factors cleanly:

```text
  minor_ratio / scalar  =  shell_scale / (n_sigma * sigma_minor)  x  across-fraction
        130             =              143                        x      0.909
```

- The first factor is **geometry** - how many drawn half-widths the shell reaches.
  Measured **143 at both ends** of the sigma knob (0.693 and 0.695 of the half-length
  at the shipping covariance and at the top stop), because scaling the covariance
  scales the shell and the ellipse together. That is what licenses quoting a figure
  taken at the shipping scale to explain a gap measured at the top stop; the guard
  test prints it at both ends rather than leaving the invariance assumed.
- The second is **direction** - what fraction of the worst residual lies across
  the needle rather than along it. At the knob's top stop 0.90 of it does.

Neither is the aspect ratio, and the aspect ratio is the number the first draft of
`max_relative_residual`'s doc claimed. Both are now printed by the guard test
(`shipping shell reaches 350.9 km ... (0.693 of it) and a half-width of 2.455 km
(143x it)`) so the doc quotes a measured line rather than a plausible mechanism.

#### The unit test that has a right answer

The kernel-gated guard test can only pin what the real encounter happens to
produce. The three new tests in `uncertainty.rs` are pure arithmetic and one of
them knows the answer in advance: plant a residual at exactly 40 % of the drawn
3 sigma half-width, **across** the needle and **not** on the sample that sets
`shell_scale`, and the two numbers must then differ by *exactly* the aspect ratio -
because with the residual purely across and the denominator exactly the shell's
own reach, every other factor is 1. Asserted to 1e-12. The other two pin that the
ratios survive a common rotation of the b-plane basis (they are dot products in a
shared frame, so they must, while `major_hat` must rotate with it), and that a
rank-one covariance returns `None` instead of dividing by a zero width.

Together they say what the 60 s kernel test cannot: that the method computes the
thing it claims to, rather than reproducing whatever it reproduced yesterday.

#### Behaviour preserved, and how that was checked

The guard test prints the same numbers as before the move - shipping minor
**0.0007**, top stop **0.5610**, scalar **0.0043** - which is the check that
discriminates here, because a refactor that reordered an eigenvector would compile
clean and pass every pure-math test. It must be run as
`ASTEROID_REQUIRE_KERNELS=1 cargo test ...`: without it `resolve_for_test` returns
early and the test passes in 0 s having measured nothing.

One cosmetic bug caught by running the probe rather than reading it: `cargo fmt`
can **silently join a `\`-continued string literal and bake the indentation into
the string**, so a verdict line printed with eighteen spaces in the middle of it.
Reproduced on a two-line minimal file, and it is not conditioned on the joined
line fitting - a 150-character result is joined too. It happened to one of the two
such strings written in this batch and not the other (the guard test's survived),
so it is not reliably predictable either way. Write `println!` strings on one line,
and read the output after formatting rather than the source before it.

#### The one thing the guard test could not have caught

The guard test never prints an angle, so it exercises `shape_residual`'s *ratios*
and not the axis it returns. Running the probe caught what that leaves open:
`symmetric_eigen` hands back **either end** of an eigenvector, and which end is not
a property of the covariance. Moving the decomposition out of the probe flipped the
sign, and the drawn angle went **89.74 -> -90.26 degrees** against a published map
recording 89.736 - the same line, printed as a different ellipse - while every
ratio stayed identical to four decimals, because they are absolute dot products.

Two fixes, because the sign has two lives. `shape_residual` now **pins** it (first
non-zero component positive) so one covariance always yields one vector, with a
unit test that turns the same ellipse to eight angles and checks both the side and
the parallelism. And the probe **folds the printed angle into a half-turn**, because
that pinning does not survive the arbitrary rotation into the display frame - an
ellipse axis is a line, and an angle for it is only defined modulo 180 degrees.

The general lesson is the one this file keeps recording in different clothes: a
refactor that preserves every number a test prints can still change a number no
test prints. The probe is the thing that prints it.

#### What this leaves

- **Still no shape verdict on screen.** `probe_tier3_uncertainty` now prints both
  ratios and two verdicts, but the frontend prints neither, because the report
  still does not cross the binding. That is unchanged by choice, not by oversight:
  the drawn ellipse is about one pixel at the default zoom and the finding is that
  it is sound.
- **The three-place covariance duplication is untouched.** The frontend's drawn
  covariance constants still exist in `mission_core.rs`, `probe_keyhole_map` and
  this probe/test pair with nothing enforcing agreement.
- `cargo clippy` reports **ten** pre-existing `neg_cmp_op_on_partial_ord` lints
  across the crate, not the three in `sbdb.rs` the last batch recorded - the
  others are in `keyhole.rs` (3), `sbdb.rs` (4 total), `deflection.rs`,
  `keyhole_target.rs` and `probe_keyhole_floor.rs`. Nothing in this batch touches
  them, and CI does not pass `-D warnings`.
