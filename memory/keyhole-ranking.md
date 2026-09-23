---
name: keyhole-ranking
description: "2026-09-07 - the core ranked resonant circles by keyhole WIDTHS, which the placement finding makes the wrong key; replaced by margin (km outside a door). Measured 0 disagreements in 2000 geometries, so it is the principled key, not a caught bug."
metadata: 
  node_type: memory
  type: project
  originSessionId: af825b26-61f6-4964-9688-b042971d47ca
  modified: 2026-09-07T14:33:53.942Z
---

The two live threads the five-door placement batch left behind, both closed
2026-09-07. They were one thing seen twice: `core` still ranked circles by
**keyhole widths** (`OpikFrame::tightest_keyhole`), and the planner's note line
printed that ranking's disagreement with kilometres - a branch the batch recorded
as "mostly-live now, not mostly-dead" and which had never been seen to fire.

**Why a ratio is the wrong key.** The campaign measured width and placement
separately and got opposite verdicts: the linearised door is conservative by at
most 1.44x (good), but the door centres sit 2.0-26.8 km off their own circles
following no law (bad). That makes placement an *additive* error, which is why
the frontend cut is `|distance| <= 100 km + width/2`. Dividing an additive error
by a door's width is exactly what hides it - `widths_away` ranks a 200 km-wide
door 300 km away ahead of a 25 km one 60 km away.

**What shipped.** `KeyholeProximity::margin()` = `|signed_distance| - half_width`
(km outside a door, negative inside), and `OpikFrame::smallest_margin_keyhole`
ranking by it. It pairs with `nearest_keyhole` as an edge pairs with a locus:
nearest *circle* is what to print beside the drawn map, nearest *door edge* is
what an alert is cut on. Binding row gains `margin_km`; the readout's second row
is **renamed** `tightest` -> `at_risk` rather than redefined in place.
`tightest_keyhole` stays with an honest doc. GDScript `keyhole_margin_km` now
*reads* `margin_km` instead of recomputing it, so the ranking and the cut cannot
use two different formulas.

**The measurement that kept this from being written up as a bug fix.** The old
alert was cut on the nearest *circle's* margin, so a wider door further out could
in principle have gone unflagged. Measured: across **2 000 random Öpik geometries
(0 disagreements)** the nearest circle was always also the nearest door. Reported
by the test, not asserted - the first draft asserted a disagreement and failed,
which is how this got measured at all. But it is a near-tie, not structure: the
runner-up came within **9.0e-6 capture radii (~0.1 km)** of winning against doors
up to **5.3e-2 (~600 km)**. So: principled key, not a caught miss.

**Sampling trap found on the way.** That sweep had been drawing b-points from a
box of +/-3 capture radii while the census reaches 60 - it only ever asked about
the crowded near field, and keyhole widths grow with distance. Now log-uniform in
radius over [0.5, 40].

**The never-seen branch is now executed.** `test_orrery.gd` drives
`keyhole_note` / `keyhole_label` / `keyhole_alert` by assigning a hand-built
`plan_keyhole` dictionary (legitimate: it is plain data and the readouts only
format it; a real plan stays live underneath for the solving / clean-miss gates).
Four checks, including that the alerting line and the blink now name the **same**
circle because both are cut on the `at_risk` row. `_shot.gd` prints which circle
each ranking named and whether they agreed, so a fallback-text run is
distinguishable from one that exercised the branch.

**Also fixed while here:** five stale doc sites all asserting the widths ranking,
including `keyhole_label`'s docstring, which said it named a resonance "in
keyhole widths" while its body had already switched to kilometres.

Related: [[keyhole-placement]], [[keyhole-targeting]], [[gdext-binding]].
