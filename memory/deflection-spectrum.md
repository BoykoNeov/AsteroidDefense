---
name: deflection-spectrum
description: "§5's deflection-method spectrum — COMPLETE 2026-07-27 (nuclear standoff + gravity tractor, both core-only); Tier-3 uncertainty is Phase 2's last unstarted item"
metadata:
  node_type: memory
  type: project
  originSessionId: 3f14ce5f-5220-4e1b-a2f1-e0a03008a77a
  modified: 2026-07-28T01:48:13.059Z
---

**§5's method spectrum is CLOSED (2026-07-27, both halves core-only, no frontend).**
Phase 2's checklist now has **one** unstarted item: **Tier-3 uncertainty**
(covariance → b-plane → impact probability, keyholes — which also forces the
deferred b-vector sign + ξ,ζ decomposition). See [[tier2-forces]], [[mission-porkchop]].

## Nuclear half (`StandoffNuclear` in `core/src/deflection.rs`)

- **Nuclear = impulse-shaped, tractor = force-shaped**, and `apply_impulse`'s doc
  had said so since the MVP. **No second root-find**, for a geometric reason not
  luck: a standoff burst's direction is *chosen* (you place the device) while a
  kinetic impactor's is fixed by arrival `v_rel` → nuclear is a **closed-form
  invert** of `required_dv_along_track`.
- **`DEARBORN_2007`, fetched not recalled** (UCRL-PROC-228569): η = 0.115,
  `C_m = 1.418e-4 N·s/J`. Rejected sources matter — Ahrens & Harris paywalled;
  LLNL-PROC-485160's *surface*-burst table is the wrong mechanism. Uncertainty is a
  **stated spread** (485160's 1.91e-4 is ≥36% away and a *lower* bound, so we ship
  below it and err high); guarded by a **provenance** test.
- **The real finding — on OUR rock every campaign lead is `LikelyDisruption`.**
  Easiest lead (8 periods, 6.32 yr) needs 0.0662 m/s = **32×** the intact-deflection
  ceiling; 65.7 kt reaches the body's 0.159 m/s escape speed. Verdict: **wrong tool
  for this rock**. Gap bounded on **both** sides (10–100×) so a coefficient error
  can't pass more emphatically.

## Tractor half (`core/src/forces/tractor.rs` + the duration solve)

- **No coefficient to source — recognising that early saved a repeat source hunt.**
  The tow is `G·m_sc/d²`, pure Newton. Lu & Love 2005 (Nature 438, 177;
  `astro-ph/0509595`, read from the **PDF on disk** — WebFetch's summarizer refuses
  binary, don't re-fetch) quote `Δv = 4.2e-3·(m/2e4 kg)·(d/100 m)^-2 m/s/yr`, which
  **is** `G·m/d²·yr` (ours matches to **0.30%**). The paper supplies a
  **configuration** (20 t at `d/r = 1.5` over a 200 m 2 g/cm³ body) and **cant
  bookkeeping**, not a coefficient. Second anchor: `T = G·M·m/(d²·cos[sin⁻¹(r/d)+φ])
  = 1.052 N` vs the paper's stated "T = 1 N".
- **Cant is a THRUST penalty, never a weaker tug** — the paper's own equation puts
  cant on the left with `T` and has no `φ` on the gravity side. A `cos(cant)` on the
  tow would look conservative and silently understate every Δv = the
  `payload_kg`-means-two-things bug again. **Enforced by signatures**:
  `tow_acceleration()` cannot see the cant *or* the asteroid mass;
  `station_keeping_thrust_n()` is the one place asteroid mass legitimately enters.
- **"Just Yarkovsky with a window" is the VALIDATION ASSET.** Station-keeping holds
  `d` fixed → the tow doesn't fade with heliocentric distance → it is exactly the
  **`d = 0`** case of the Yarkovsky `A2·(r₀/r)^d` form. So the oracle moved out of
  `yarkovsky.rs`'s private tests into **`core/src/forces/secular_oracle.rs`** (one
  Gauss planetary equation, two callers) + a `d=0` circular closed form
  `da/dt = 2·a_T/n`.
- **THE WINDOW-EDGE MEASUREMENT** (do not re-derive): a hard on/off edge does *not*
  defeat the adaptive controller, it **converts a tolerance into a systematic Δv
  error**.
  | edges | rtol/atol 1e-9 (shipping) | rtol 1e-13/atol 1e-6 |
  |---|---|---|
  | inside steps | −1.0e-4 | +6.3e-3 |
  | on step boundaries | −9.2e-10 | −9.2e-10 |
  Boundary-aligned is exact at *any* tolerance (no step contains a discontinuity).
  **Decision: leave edges FREE** — snapping would quantize the duration bisection to
  the snapshot cadence, and 1e-4 is ~1e-6 m/s on the campaign's tow.

## The API change (was pre-scoped, and one part was wrong)

- `DeflectionScenario` holds `force: &'a dyn ForceModel` → only vary-the-*state*
  solves were expressible. Fix = **`propagate_and_reduce(force, start, seed)`**,
  the extracted body of `deflected_trajectory` taking the field as an argument.
- **`ForceSum`, NOT the planned `ForceRef`.** Boxing a borrowed field into
  `CompositeForce` **does not compile**: `Box<dyn ForceModel>` is implicitly
  `+ 'static`, and Rust has no default lifetime params so adding `'a` to
  `CompositeForce` would ripple everywhere. Summing two refs allocates nothing.
- **Bounded bisection on `[0, cap]`, not a geometric bracket** — a duration has an
  a-priori upper bound (lead time) so no seed/growth/expansion. The shared
  `NotHyperbolic`⇒hit, off-gate⇒+∞ mapping extracted to `perigee_scale` rather than
  copied (the anti-fork move).
- **`TowDurationCapped` carries the cap AND the perigee reached — never returns the
  cap.** That would repeat `required_impactor_mass` handing back its seed verbatim.
- **PERIGEE IS NOT MONOTONE IN TOW DURATION** — my first doc claimed it was and the
  real field falsified it. Bisection needs only a **single crossing of the target
  level**, which holds because the dip goes further *below* the nominal while any
  sane target sits well *above* it (20 000 km bar vs 3000 km nominal).

## The campaign headline (gdext `gravity_tractor_measured_on_the_real_threat`, 171 s)

Our rock is 300 m / 2.83e10 kg at **2.00 g/cm³ — the same density Lu & Love assume**,
so the two rows differ only in size. 20 t at `d/r = 1.5` (225 m):
tow **2.64e-11 m/s²** (0.832 mm/s/yr), station-keeping **1.578 N** (cant 61.8°),
**5.26 mm/s** delivered over the full 6.32 yr lead vs **66.2 mm/s** required =
**12.6× short** — and that lead is the *cheapest* point on the sweep, so it is a
**best** case.

- **Nuclear = wrong tool (regime change); tractor = right tool at the wrong scale
  (a spacecraft-mass number).** The tractor comes closest of the three to closing.
- **A feeble tractor DEEPENS the hit**: perigee 3000.0 → **2811.6 km**, 188 km the
  wrong way (near-centre nominal → small tug walks it toward Earth's centre).
  Asserted, not printed — it is the counter-example the non-monotonicity doc rests on.
- **Closing mass ≈ 252 t by arithmetic** (tow is exactly linear in spacecraft mass —
  no third solver). Checked not trusted: at 2× (504 t) the solve wants **3.81 yr**
  (60% of lead), round-tripping to **20 008.8 km** vs the 20 000 km bar, 20% less →
  16 655 km.
- **MEASURED COSTS** (measure-first changed the test's design): one tow probe
  **12.4 s**; one `required_dv_along_track` at 8 periods **236.6 s**. So the test
  **reuses** `CURVE_JSON_DV_AT_8_PERIODS` (promoted from a local const to module
  scope so the nuclear and tractor tests can't drift) rather than re-paying 237 s.

## The tractor frontend — `[K]` bench (2026-07-27, same day)

The nuclear half is still **core-only**. The tractor now has a panel, and the
work was all in *what is honest to print live*. See [[godot-visual-layer]].

- **`a·T` is a LIE as a live margin** (+21% measured). Ships instead:
  **`Δv_eff = a·T·(1 − T/2L)`** — not fitted, it's the *same* linear response
  `f(τ)∝τ` that gives the 1/lead law, integrated over the window. Reads **−16%**
  at the 504 t/3.81 yr calibration point → **errs SHORT, which is why it ships**.
  Test pins **both** signs so nobody flips it to the flattering side.
  Towing the whole lead is worth exactly **half** its delivered Δv.
- **`Δv(n) ≈ Δv(1)/n`, floor at 1 orbit** — 0.1% at 2 P, 3.9% at 8 P, but
  **1.73× wrong at 0.5 P** (sub-orbital arc, falloff isn't there yet). Below the
  floor the panel prints **no requirement and no margin** (absent, not zero), and
  the test *proves* the floor fails rather than asserting it.
- **`CURVE_JSON_DV_AT_8_PERIODS` had to leave `#[cfg(test)]`** → shipping
  `REQUIRED_DV_AT_{ONE_PERIOD,EIGHT_PERIODS}`. A "test-only" number stops being
  one the moment a readout quotes it.
- **THE WALL IS NOT THE SURFACE.** cant = `sin⁻¹(r/d)+φ`, thrust ÷ cos → no
  station-keeping once cant hits 90°, i.e. **`d/r < 1/cos φ` = 1.064 radii** at
  the 20° plume. That band clears the surface AND tows fine. First draft's knob
  bottomed at 1.02 (3 keypresses away); core returns `None` → panel would print
  **`0.000 N`** = station-keeping looks *free* exactly where it's impossible.
  Fix: `min_hover_radii_for_station_keeping` closed form in core + a separate
  `holds_station` flag (never infer it from a zero).
- **DIRECTION IS THE SHARPEST KNOB, and nothing had probed it** (every core+
  frontend probe was prograde until the last check). On the shipping config, one
  keypress apart: **prograde 3000→2811 km (−188, DEEPER) vs retrograde 3000→3348
  km (+348, OUTWARD)** — *not* a symmetric flip, retrograde is ~2× bigger. Same
  near-centre geometry as the non-monotonicity: b sits ~3000 km off centre, so one
  way walks it toward the centre and the other straight away. **Panel opens on
  PROGRADE deliberately** — it's the measured/documented failing case, one
  keypress from the one that works. Seeding on the flattering one hides the point.
- **Knobs are a TABLE (`Sim.TRACTOR_KNOBS`), not variables** — porkchop cursor
  idiom (UP/DOWN select, LEFT/RIGHT adjust, `[E]` measure). **One action for six
  knobs**; a 7th knob = one row, zero `main.gd` edits. Harness iterates the table.
- **Rock-radius knob scoped so it can't leak** — `tractor_hover_over` derives its
  own mass; `threat_mass_kg` (porkchop divisor + SRP pin) stays a fn of a const.
  Teaching point: tow ∝ 1/r² while **required Δv doesn't move at all** (test
  particle).
- **Three silent traps this cost an hour each on** — see [[gdext-binding]]:
  Godot loads **`target/debug/`** while the Rust loop builds `--release` (a new
  `#[func]` is "Nonexistent function"); a new `class_name` needs an editor
  rescan; and **GDScript's `%` operator has NO `%e`** — doesn't raise, errors
  once per frame from inside `_draw`. `_sci()` formats the 1e-11…1e13 range.

Keys now taken: 1–4, C, P, E, M, L, O, **K**, **N**.

## The rock itself became dialable — 2026-07-28 (see [[threat-orbit]])

`[N]` puts the threat on a **different heliocentric orbit**, which turns this
bench from "one rock that cannot be saved" into a comparison. The headline: the
**same 200 t plan over the same 6.0 yr lead scores 0.372× on the shipping orbit
and 1.096× on a long-period one** — fails and closes, from one knob.

Two things here changed *because* of that, and both are load-bearing:

- **`REQUIRED_DV_AT_ONE_PERIOD` stopped being usable as a default.** It is one
  rock on one orbit. `tractor_readout` now takes `Option<f64>`, and the
  no-anchor convenience `required_dv_estimate(n)` was **deleted** so nothing can
  quietly reach for the constant on a rebuilt orbit. The panel distinguishes the
  two absences — *below the law floor* vs *this orbit is unmeasured, press [E]*.
- **The bench's knobs survive a rebuild.** `_seed_tractor_defaults` seeds bounds
  every install and *values* only the first, or turning an orbit knob would reset
  the spacecraft to 20 t and make the comparison meaningless.
