---
name: keyhole-circle-on-change
description: "2026-09-08 - the keyhole circle is now placed on the CHANGE across the flyby (shipped), and the leftover placement error turned out to be a constant of semi-major axis, not of b-plane kilometres: the band is 15 000 km of a' converted per circle, so 575 km on a 3:4 and 57 km on a 2:3."
metadata: 
  node_type: memory
  type: project
  originSessionId: 4890f6aa-a34c-448c-8916-71449ffd94ee
  modified: 2026-09-08T14:31:14.958Z
---

Ships what [[keyhole-baseline-not-turn]] measured and left on the shelf, and the
gate it had to pass first moved something nobody had proposed.

**The gate did not need a door.** The baseline/turn split reads two revolution
means at whatever b-plane point a flight lands on; the recorded door centre only
enters the "what the repair buys" column. So the second resonance was flown on a
**crossing** Δv found by `xi_sweep` (50 forward flights, 313 s) instead of
bisecting a door (~45 flights *plus* a return propagation). `probe_keyhole_placement
outgoing` grew `fly=<lead>:<dv>` for exactly this, and a row flown that way says so
rather than printing a `d₀` of 0 as if it were a door. **Reusable: check whether the
claim needs the expensive observable before paying for it.**

**Headline: the leftover constant is in `a'`, not in kilometres.** The 2:3 Minus
was picked because its gradient is **10×** the 3:4's on the same encounter (263.8
vs 26.1 m/m) — the axis four leads of one resonance cannot vary (1.75 %).

| | `∇a'·n̂` (m/m) | error in the change, b-plane km | the same, km of `a'` |
|---|---|---|---|
| 3:4, four leads | 26.10 … 26.36 | +242.3 … +381.1 (spread 138.8) | +6 274 … +10 045 |
| 2:3, two leads | 254.7 … 263.8 | +25.5 … +28.1 (**spread 2.6**) | +6 485 … +7 409 |

12× apart in km — *exactly* the gradient ratio — and **overlapping** in `a'`. Same
story from the other side: the incoming measurement's own bar is ±136 b-plane km on
the 3:4 and ±14 on the 2:3, which are **±3 535 and ±3 540 km of `a'`** — the same
number from a convention that knows nothing about either resonance. So an additive
kilometre band had the wrong *shape*, not just the wrong size: 30× too generous at
a steep circle, unboundedly too tight as `∇a' → 0`.

**What shipped.**
- `OpikFrame::resonant_circle_on_change(resonance, a_in_true)` — the circle at level
  set `a_res + (a_in_predicted − a_in_true)`, **solved exactly**, not translated (a
  shifted level set has its own centre *and* radius). `ResonantCircle.a_prime_target`
  is the level set it is; `a_prime` stays the resonance's `a_res` because the width
  is a return-timing tolerance on the resonant orbit. Feeding the frame's own
  `incoming_semi_major_axis()` reproduces `resonant_circle` exactly — asserted, so
  it is a strict generalisation, not a second construction to keep in step.
- `keyhole_target::incoming_semi_major_axis_flown` — one osculating heliocentric `a`
  at the last epoch before CA where the rock is **10 Earth Hill radii** out (in Hill
  radii, *not* days: the campaign's CA − 30 d is 12.6–13 Hill here and would be
  inside Earth's grip on a slower flyby). Returns `IncomingBaseline` with epoch,
  clearance and a `settled` flag; when the arc starts closer in it gives the best
  available reading rather than refusing — refusing would drop the repair where its
  correction is largest. Read **once per plan** in `set_plan` (~30 interpolations of
  a clock already in hand), so `keyhole_readout` stays closed-form fast. No orbital
  mechanics crossed into GDScript.
- `Keyhole::placement_band(band_a) = band_a / |∇a'|`, and
  `KeyholeProximity::exposure = margin − placement_band` is **both** the alert cut
  and the ranking key (`most_exposed_keyhole` minimises what `doors_within_band`
  counts). Third ranking-key change in this file (widths → margin → exposure), each
  time because the key had stopped matching the cut. See [[keyhole-ranking]].
- Frontend: `KEYHOLE_PLACEMENT_KM = 800` (b-plane km) → **`KEYHOLE_PLACEMENT_A_KM =
  15 000`** (km of `a'`). Every row carries its own `placement_band_km` and
  `exposure_km`; the caveat quotes the row's band, never the constant. Readout also
  reports `placed_on_the_change` and `incoming_a_au` so a reader can tell which
  placement they are looking at.

**Where 15 000 comes from:** worst repaired door of the six flown, 11 024 km of `a'`,
plus the incoming bar 3 535, rounded up. A measured maximum over **two resonances,
six flights** — not a bound. Domain floor of 200 d still applies
([[keyhole-domain-floor]]).

**The constant is deliberately NOT subtracted.** All six flights sit on the same
side, 8 184 – 11 024 km of `a'` out; subtracting would put the worst door inside
~130 km. Six flights cannot set a number whose own spread is 1.35×, and trading a
known error for a badly known one is not a repair.

**Two things that had to be checked, not assumed.**
1. Every published table computed the repair as a translation along the normal. The
   shipped circle is a re-solve, and the residual is set by *shift against circle
   size*: 22 m at 12 km, 1.3 km at 393 km, 30 km at 1 433 km. Over the range the
   repair produces (≤500 km) they agree to **<5 km** vs a ±136 km bar — so the old
   tables do describe the shipped circle. Three test-tolerance forms failed before
   that one (1 km absolute, 1 % relative, `err ≤ predicted²/R`).
2. The probe's two-leg verdict printed "NOT the same — the turn owns the residual"
   on a flat 25 % rule. On the 2:3 both legs are 15–30 km against a ±14 km bar, so
   the rule was asserting more than it had measured. Third branch added:
   INDISTINGUISHABLE when the gap is under 2× the bar, and every row now prints the
   gap **and** the bar.

**An assertion had to be retired, not fixed.**
`the_keyhole_readout_finds_the_three_four_door_the_probe_flew` asserted the
keyhole-flying plan reads a *smaller* margin than the default plan. That was only
true because the old circle was drawn through that one shot's door — it encoded the
bug. Replaced by three that survive: flown plan's exposure < 0, default plan's > 0
(the retired 800 km constant would have shouted at it), and the two bands differ by
>2× because they are different circles. The 1.64-half-width historical calibration
is kept by censusing the **plain** circles alongside the repaired ones in that one
test.

**How to apply.** Gate: `cargo run --release --example probe_keyhole_placement --
xi_sweep 2 3 minus 4383 200`, then `... outgoing 2 3 minus fly=4383:0.053054,200:0.636376`.
Pinned end to end by `W:\Claude_projects\AsteroidDefense\core\tests\keyhole_prediction_bias.rs`
(shipping observable + shipping circle: two extreme 3:4 doors **20.7 km apart** vs a
**766.8 km** ladder). `godot/tests/test_orrery.gd` **extends SceneTree**, so
`run_harness.ps1` hangs on it for the full 900 s — run it directly:
`godot --headless --path W:\Claude_projects\AsteroidDefense\godot --script res://tests/test_orrery.gd`
(capture the PID; kill only that PID).

**Answered 2026-09-23 in [[keyhole-constant-not-one-number]]: not one number, band now 51 000.** Was open: the ~8 000–11 000 km-of-`a'` constant (a third resonance, or the same
two at more leads, would say whether it is one number — the 5:7, 7:10 and 6:5 have
12 yr doors but no dialable one, and their gradients 97/138/2 425 m/m span the axis
that turned out to matter); `doors_in_band` was measured against the retired 800 km
band, so the crowded-register result wants re-running; and the residual constant may
still be encounter-local, which is the one place the solar tide and the finite turn
time are live suspects.

Related: [[keyhole-baseline-not-turn]], [[keyhole-prediction-vs-condition]],
[[keyhole-placement]], [[keyhole-ranking]], [[keyhole-domain-floor]],
[[keyhole-lead-variable]], [[keyhole-targeting]], [[keyhole-reach]].
