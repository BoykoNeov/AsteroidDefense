---
name: keyhole-domain-floor
description: "Below 200 d there is no keyhole door for the placement band to be wrong about - the band gains a domain floor instead of a bigger number, plus the three gates that had to be discarded or fixed to establish a negative."
metadata: 
  node_type: memory
  type: project
  originSessionId: 7ef90a4c-1125-4c7f-8b7a-17ea11dd9bd9
  modified: 2026-09-07T20:46:05.992Z
---

**2026-09-07.** `KEYHOLE_PLACEMENT_KM` = 800 was calibrated on doors flown at
200 d and up while the planner's slider goes down to 30 d. That last open half is
closed: **below 200 d the answer is not a bigger number, it is no door at all.**
The constant stays at 800 and gains a stated domain floor. See
[[keyhole-lead-variable]] for the ladder above 200 d and [[keyhole-placement]] for
the width/placement split this rests on.

Three circles (3:4, 2:3, 5:7) swept at 150/125/100/75/50 d in the **nominal** Opik
frame, so it is the same circle at every lead:

- **125 d** - all three still *cross* their circle, and all three floor **19 447 /
  25 140 / 29 341 km** out. Not impact keyholes.
- **100 d** - the 3:4 circle cannot be reached; the pass leaves the 5e8 m scan
  gate at dv 37.5 m/s first. (2:3 and 5:7 ended on the wrong branch = NOT MEASURED,
  not negative.)
- **75 d / 50 d** - none of the three reached.

**The floors are converged, which is what makes a negative a result.** Timing
`zeta2` is 2.9 / 58.1 / 187.9 km against a spatial `xi2` of 25 000-36 000 km. A
return that merely lands far out could be a search that stopped early; one whose
timing is spent cannot be. Same gauge as [[keyhole-targeting]].

**Only 3 of 168 circles were flown**, so "no keyhole below 150 d" is NOT
established and the doc says so explicitly.

## The control is the whole point

A method that finds nothing must be shown to find something. The same `dv=`-aimed
path at 200 d reproduces the recorded door to three decimals: centre **+785.973**
vs +786.0, edges +780.068 / +791.879, return **HITS**.

## Three gates discarded or fixed - do not rediscover

1. **`screen` is not a door-existence gate.** At 200 d and 300 d, where the door is
   known and flown, it scores **zero hits** (returns 478 709 / 576 838 km). One
   unrefined shot lands half a million km out even where a door exists. Read its
   `xi2` column, never `hit?`.
2. **The ladder's aim is geometrically unusable below ~150 d.** It matches the
   circle's `b` at the *nominal* xi; on the 3:4 that is b = 153 424 km against a
   circle whose largest `b` is 153 577 km, i.e. the outermost point, reachable only
   near xi = 0. Short leads reach that `b` far out in xi. Four `bracketed = false`
   runs ending 214 729-320 646 km out, each sitting at exactly 2.27x its aim =
   `reach_fraction`, the wall. Fix: `dv=` on the `door` stage, aimed from the
   `xi_sweep` crossing.
3. **`xi_sweep` reported a hole in the curve as its end.** At 50 d the retrograde
   curve passes *through Earth* (b = 762 km at dv 1.7586) and the flight fails
   there; the scan stopped and printed `NOT REACHED` on a curve the ladder had
   flown to b = 306 654 km. `Ok(None)` (past the scan gate) is a terminus; `Err` is
   one bad impulse to step over, clearing `prev` so no bracket spans the hole.

Also: `SWEEP_DV_HI` = 30 m/s is calibrated **at 150 d** and stops the scan short
below it - now `dvmax=` / `rungs=` (a pair: widening the span without adding rungs
coarsens the geometric spacing the sign change must be caught in). Because the
curve cuts a circle **twice**, a coarser scan can straddle both crossings and print
a plausible `NOT REACHED`, so **the 150 d row must reproduce the recorded crossing
(dv 3.2235, xi -52 860) before any other row is read.** It does: 3.223849 /
-52 865.7.

## One number that must not be banked

The 125 d 3:4 floor prints "+1 112 km from the circle", past the 800 km band. That
is **not** a placement error - placement is the midpoint of two flown door *edges*,
and there are no edges where there is no door.

Guard: `the_three_four_door_has_ceased_to_exist_by_a_125_day_lead` in
`M:\claud_projects\AsteroidDefense\core\src\keyhole_target.rs` - one flight, asserts
the return is outside the capture disc **and** that its timing is spent.
