---
name: keyhole-targeting
description: "Keyhole targeting as a core API (core/src/keyhole_target.rs) plus the planner's KEYHOLE row: the return read in its OWN Opik frame, the width's measured 1.6x conservatism, and the coarse-sweep trap, and the three constants the second resonance (7:9) broke."
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

## The second resonance broke it three ways (2026-09-06)

The API was validated on the 3:4 it grew from, which is not the same as being
general. Pointing it at **7:9** — a far circle, the obvious next try — failed
three times, none of them loudly.

1. **The first-encounter scan gate is sized for a rock that hits** (5e8 m). A
   keyhole aim flies *far* out on purpose and the wide keyholes are the far
   ones; 7:9 wants `b ≈ 5.2e8 m`, so the flight said `NoFirstEncounter` for a
   flyby that was there. `widened_for_aim` now sets `max(gate, 2·b)` — one-way,
   and safe for an argmin (a wider census can only admit rejected approaches,
   never move a minimum already inside).
2. **The return census gate must scale with the return, and it is not a
   margin.** Closed-form error δ in `a'` → period error `1.5δ` → over `h` years
   the arrival slips `h·yr·1.5·δ` while Earth moves at 30 km/s: **linear in
   `h`**. Measured — 7:9's flown `a'` was `9.3e-4` off (7× the 3:4's `1.3e-4`),
   ~3.6 days ≈ **9.2e6 km** against a fixed 0.05 AU = 7.5e6 km window, so every
   flight reported "no return" for a return just outside it. Now
   `h × RETURN_GATE_PER_YEAR_M`, defined as `0.05 AU / 3` so `h = 3` reproduces
   the flown 3:4 exactly.
3. **A probe Δv that fails to fly must score `INFINITY`, not abort the solve.**
   Golden-section eats an infinite sample; it cannot eat an exception. Only the
   aim and the final best must genuinely fly.

**The ξ₂/ζ₂ gauge earned its keep across a solve, not at its end.** 3:4 aim
(Δv 0.216438): ξ₂ 3 549 / ζ₂ −60 185 km, ratio **16.96**. Refined floor
(0.216550): ξ₂ 4 013 / ζ₂ 786 km, ratio **0.196**. Timing falls 77×, spatial
barely moves — exactly what "Δv is a timing knob" predicts, so the ratio is a
real convergence gauge and not a coincidence.

**And it caught a false claim, then a real bug.** 7:9 stopped at 3 924 232 km
made of ξ₂ 129 230 / ζ₂ −3 928 736 km — **30.4× more timing than spatial** — and
the probe printed it as "the spatial offset, which no timing change removes".
Δv *is* the timing knob, so that was false.

**Trap — a wall is indistinguishable from a floor, and the Δv window lies about
which you have.** The first guess was "out of iterations". Re-running 7:9 at 30
instead of 12 shrank the Δv window 4.48e-5 → **5.32e-8** (842×, golden-section
doing its job) and moved the answer **1 856 km out of 3.9 million — 0.05 %**.
Golden-section narrows a bracket; it cannot move one. The real fault was the
widening step: `hi = mid + (mid - lo)` is **constant, not doubling**, so six 1 %
widenings reach only `aim × 1.07`, and `0.721750 × 1.07 = 0.772272` — **exactly
the reported "floor", every digit**. The search converged onto its own upper
bound, and squeezing against a bound produces a *vanishing* Δv window, i.e. it
looks like a tight answer. The 3:4 hid this completely: it walks 5e-4 from aim to
floor, inside the first ±1 % bracket, so the widening loop never ran once — **the
bug was unreachable from the only case the code had ever been run on.**

Fixed three ways: the step **doubles** (`reach_fraction()` = `f·(2ⁿ⁺¹−1)` = 127 %
of the aim vs the old 7 %, same six widenings); `KeyholeSolution::bracketed`
reports whether the minimum was ever enclosed (finite centre, no worse than both
ends), so a wall is stated rather than inferred; and the probe prints
`!! THE SEARCH NEVER BRACKETED THE MINIMUM` **above** everything, because every
number underneath comes from the wall. The test pins the *coincidence*, not the
fix — walk 6.99993 % vs old reach 7.00000 %, agreeing to five figures because one
was the other.

**What was behind the wall.** 7:9 re-flown with a bracket that reaches: floor Δv
**0.810311**, return miss **46 608 km** (was 3 922 376 — **the wall was 84× worse
than the answer**), ξ₂ −52 998 km spatial, ζ₂ **−0 km** timing, ratio **0.000**.
The timing is driven to zero, so "the spatial offset, which no timing change
removes" is finally *earned*. 3:4 reproduces its pushed result to every digit
(0.216438 → 53 841 km; 0.216550 → 1 130 km; impact; 18 flights) — its widening
loop still never runs.

**Trap — the fix was wrong the same way, and the test could not see it.** The
first doubling step read the **trailing** gap (`2.0 * (mid - lo)` on the right
branch), which still grows but as `2^(n/2)`: the walk goes `w,3w,5w,9w,13w,21w,
29w`, reach **0.29** while `reach_fraction()` promised **1.27**. Read the
*leading* gap and it is `w,3w,7w,15w,31w,63w,127w`. Both look like "doubling".
The test that existed asserted `reach_fraction()` against the literal `1.27` —
**the formula checked against itself; it would pass with the loop deleted**. The
widening is now a `Bracket` type whose `widen()` takes any objective, and the
test walks it with `f(dv) = -dv` (never brackets → every widening spent walking)
and asserts where `hi` lands; two more assert a minimum at *half* the reach is
bracketed *and* enclosed, one at twice it is not, and three infinities are not a
bracket. That pair caught the margin too: `reach_fraction()` is where **`hi`**
lands, but bracketing needs the *centre* past the minimum and the centre reaches
only `f·(2ⁿ−1)` — half as far — so the guarantee is "comfortably inside the
reach". 7:9's floor is 12 % out against a 127 % walk. **Third time this batch that a number was checked against a restatement
of itself.** The 7:9 result stands — flown on the 0.29 version, floor 12.3 % from
the aim, `bracketed` true.

**The physics the bug was hiding: 7:9 is NOT an impact keyhole for this rock at
this ξ.** A resonant return ≠ an impact; the keyhole is a short arc of the
circle, and 7:9 floors 53 000 km off it in the **spatial** coordinate. More
along-track impulse cannot close that — ξ follows the deflection **direction**,
so it would take an out-of-plane component. First honest negative result for a
resonance other than the 3:4 (31 flights, 814 s), and it means 7:9 does **not**
help calibrate the 1.6× width slack: there is no flown door to measure against.

**2026-09-06 — [[keyhole-probability]] builds on this layer** and sharpens one
number here. The keyhole width measured "1.6× conservative" was a *placement*
statement (the flown b-point sits 1.64 half-widths from its circle). Measuring
the chained `∂ζ₂/∂ζ₁` on the flown trajectory gives the width *differentially*:
**29.09 km against the map's 24.92 km, conservative by 1.17×** — immune to the
map's absolute placement error, but it does **not** replace the placement
finding. Two different quantities; don't conflate them.
