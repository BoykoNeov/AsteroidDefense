---
name: keyhole-prediction-vs-condition
description: "2026-09-08 - the keyhole map's placement error is in its PREDICTION of the post-flyby orbit, not in the resonance CONDITION; measured on four flown 3:4 doors, and both cheap repairs (r-approximation, own frame) are dead."
metadata: 
  node_type: memory
  type: project
  originSessionId: 1f2a7c23-f07f-4eb9-a3d1-eb29becd3912
  modified: 2026-09-08T08:13:31.395Z
---

A resonant circle folds **two** claims into one locus, and until this batch the
repo had measured neither against a flown flyby:

1. **the prediction** — at a b-plane point, the flyby leaves the rock on the `a'`
   that `OpikFrame::post_encounter_semi_major_axis` says it does;
2. **the condition** — landing on `a' = a_res` is what produces the return impact.

Six sessions narrowed *where* the door sits (see [[keyhole-lead-variable]]) while
only ever re-deriving the circle. **Nobody had asked the propagator what orbit the
flyby actually produced** — `KeyholeShot::a_prime_m` is the closed form's answer,
not the flown one.

Measured by a new stage,
`W:\Claude_projects\AsteroidDefense\core\examples\probe_keyhole_placement.rs`
`outgoing` (~4 s per flown door; re-flies **recorded** door centres, so the flight
lands on the door by construction):

| lead | door offset | `a_true − a_res` | `a_true − a'_closed` |
|---|---|---|---|
| 4383 d | +19.2 km | **−14.2 km** | −33.3 km |
| 900 d | +210.6 km | **−15.7 km** | −226.6 km |
| 300 d | +648.2 km | **−13.9 km** | −664.6 km |
| 200 d | +786.0 km | **−48.8 km** | −838.6 km |

**Headline: the condition column sits at −14 to −16 km at three of the four leads
and −49 km at 200 d** — half a door width, a spread inside the measurement's own
±41 km bar — **while the prediction column IS the 19 → 786 km ladder.** (The 200 d
outlier is real, not the convention wobbling: reopening the averaging window
30 days later moves **all four rows by the same −21 km** and it stays ~33 km below
the others. The window's opening carries a systematic; comparisons between rows do
not.) So `a' = a_res` is the right target and the closed form's
`a'` is the wrong prediction of it. Everything about the *return* is thereby
excluded as a mechanism: Earth's phase at the second encounter, the h-year leg's
perturbations, any timing correction to the resonance.

**Both cheap repairs are dead, and one flight would have "confirmed" the first.**
- `r ≈ R⊕ₒᵣᵦ` (which `keyhole.rs`'s module doc had named as the cause since the
  module was written): at the **300 d** door the rock is **53 km** from Earth's
  heliocentric distance while the error is at its full 665 km, and substituting
  the rock's own distance explains **0.4 %**. At 200 d alone it looks like a law
  (right sign, and the exactly-right distance is 0.61 of the way there); the
  "fraction of the way" column across four leads reads −1.19, 3.70, 232.24, 0.61.
- Rebuilding the frame from the deflected flight's own encounter makes the
  **prediction** worse at three leads of four (−260, −353, −295, −1686 km) — the
  clean version of the `frame` stage's earlier negative, which was about the drawn
  circle and so mixed in an axis rotation.

**The observable had to be built.** An osculating heliocentric `a` after a flyby
is not convention-free: at **CA + 10 d** the rock is still inside Earth's residual
pull (~350 km-equivalent off every later sample), and even once free the value
swings ±100 km-equivalent through the revolution. What ships is the **mean over
one full post-encounter revolution** (274 d, opened at CA+30 d, 32 samples), whose
error bar — the same mean over a window starting half a revolution later — is
**41 km**. That bar is why the 12 yr row (19 km door error) is reported
INCONCLUSIVE rather than as a small answer.

**Trap:** `SETTLE_DAYS = 30` is a day count doing a distance's job — it buys ~13
Earth Hill radii only at *this* rock's ~5 km/s, and the contaminated CA+10 d sample
is at 4.4. A slower flyby would open the window nearer in and silently re-import
the contamination, so the guard test asserts the **Hill radii** at the opening
(> 10), not the days.

**How to apply:** `ASTEROID_REQUIRE_KERNELS=1 cargo run --release --example
probe_keyhole_placement -- outgoing 3 4 minus 4383 900 300 200` (~15 s + build).
Convert `a'` differences to b-plane km through `∇a'·n̂` — **exact, not a proxy**,
because the circles are the level sets of `a'`. Pinned by
`W:\Claude_projects\AsteroidDefense\core\tests\keyhole_prediction_bias.rs` (22 s,
flies the two extreme doors). Nothing shipping changed:
`KEYHOLE_PLACEMENT_KM` is still 800 — this says where the error lives, not how to
shrink it.

**CLOSED the same day by [[keyhole-baseline-not-turn]]:** the mechanism is *not*
encounter-local (the 300 d and 12 yr doors share an encounter to 1 % and differ 20× in error), and it is not any input to the closed form. Asking the same
construction on the leg **before** the encounter splits it: the baseline error (which orbit the rock arrives on) is the ladder, while the error in the *change* across the encounter is flat at ~−320 km. All four leads are still **one resonance**.

Related: [[keyhole-lead-variable]], [[keyhole-lead-sweep]], [[keyhole-placement]],
[[keyhole-reach]], [[keyhole-ranking]], [[keyhole-domain-floor]].
