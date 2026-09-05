---
name: keyhole-targeting
description: "Keyhole targeting as a core API (core/src/keyhole_target.rs) plus the planner's KEYHOLE row: the return read in its OWN Opik frame, the width's measured 1.6x conservatism, and the coarse-sweep trap."
metadata:
  type: project
---

Built 2026-09-05. Turns [[keyhole-reach]]'s one hardcoded flown shot into an API
and puts its answer on the planner panel. The thesis' **corollary**: *a miss can
be worse than a hit if it is the wrong miss* — the panel could print
`VERDICT: MISS - EARTH CLEAR` over a plan parked on a resonant circle and say
nothing.

**The readout was separable from the targeting, and shipping it first was
right.** The roadmap said "generalise the API, *then* add the readout", implying
a dependency that does not exist: the readout needs only the circle geometry
(closed form, already drawn), not the ~4-minute aim-and-refine solve. Distance
from a b-point to a circle centred at `(0, ζ_c)` is `| |p−(0,ζ_c)| − R |`.

**Two rankings, and they disagree.** `OpikFrame::nearest_keyhole` (closest in
km) and `tightest_keyhole` (closest in keyhole *widths*) are different
questions — a wide far keyhole 200 km away is likelier than a 25 km one 50 km
away, and the wide keyholes ARE the far ones. A test asserts they disagree on
some geometry; if they never did, one is dead code.

**Measured, and it changed the UI:** the flown keyhole plan (Δv 0.216550 m/s)
reads **20.4 km from the 3:4 circle against a 24.9 km width = 1.64 half-widths,
`inside = false`** — the closed form calls *outside* a door the rock
demonstrably returns through. That is `keyhole.rs`'s own "order-unity" slack,
now measured at **~1.6x on a sample of one**. So the panel prints "N widths off",
**never a yes/no**, and the alert band is 4 half-widths (a small multiple of a
door known to work), with the blink driven by that band and NOT by `inside`.

**The check that makes a "physical floor" falsifiable.** The refined return's
residual is claimed to be the orbits' spatial offset. Unfalsifiable from the
scalar distance, so `FlownReturn` reduces the return in **its OWN Opik frame**
(Earth's state at the *return* epoch): `ζ` = timing, `ξ` = spatial. Δv is a
timing knob, so convergence must drive `ζ₂→0`. **Measured: ξ₂ = 4 014 km,
ζ₂ = 891 km, ratio 0.222 — 78 % spatial, converged.** Keep straight: that
return's impact parameter is 4 112 km while its geocentric closest approach is
1 141 km — gravitational focusing, the same pair `geometry.rs` warns about.

**Reachability is an error, not a NaN.** `points_at_xi` returns `None` when
`|ξ| > R`; `aim_at_resonance` turns it into `XiOffCircle`, refused in
microseconds instead of poisoning a perigee, a Δv solve and four minutes of
flying. The old probe's bare `√(R²−ξ²)` survived only because the shipping
geometry happened to fit.

**Trap — a coarse sweep over a monotone knob is not a survey of a non-monotone
quantity.** Ten impulse rungs at max lead reported "closest a player can get:
~5 000 km", which reads as *keyholes are unreachable from the frontend*. Wrong:
the b-point walks outward past one circle after another, so rungs bracket
crossings they step over. Refining inside the best bracket found **Δv 0.930 m/s
at 900 d lead → 2.9 widths off 3:4 (36 km) while the verdict reads MISS - EARTH
CLEAR at 0.40 LD** — the corollary on screen, `enc_8_planner_keyhole`.

**Trap — two b-plane points at one epoch.** `deflected_b_point_km` rescales its
magnitude so the drawn mark cannot contradict the verdict's `|B|`. That rescale
is <0.01 %: invisible on screen, *kilometres* at these radii, i.e. the same
order as a keyhole width. The readout takes the **raw** projection; the picture
keeps the rescaled one.

Also: the circle census is the **caller's**, so a panel can never name a
resonance the map beside it does not draw (`Sim.KEYHOLE_MAX_YEARS`, one
constant); `keyhole_circles` truncates at 60 capture radii and a plan past that
is reported as `beyond_mapped_region` rather than matched to the nearest of a
truncated list; a clean miss has **no b-point at all** and says so, because the
wide keyholes are the far ones and such a pass flies past them unmeasured.
See [[gdext-binding]] and [[godot-visual-layer]] for the frontend rules this
follows.
