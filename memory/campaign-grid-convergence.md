---
name: campaign-grid-convergence
description: "2026-10-01: the 120x120 map's cells are NOT converged for campaigns (2-4/yr 7 -> 6 on nested grids); cause = best arrival sits just inside a lap-family edge. RESOLVED same day by a continuous window search (user's pick) - see [[campaign-continuous-search]]."
metadata:
  type: project
---

Follows [[launch-campaign]] and [[impactor-mass-budget]]. HANDOFF: *Is the 120×120 map fine enough?*

**Result (FH-exp):** 120: short/7/7/7/6.. ; 477: short(0.5 %)/6/6/6/6.. ; 953: 9 (arithmetic, unflown)/6/6/6/6.. .
477->953 FAILED the pre-registered bar (yr 4 prograde +5.8 %, 1/yr flipped) - say so; it does not reach
2..10/yr because plans are retrograde-only, the 6-launch flight clears by 1.3 %, and 5 launches needs ~20 %
more push. The old "1/yr falls short at every year-start" was a 120-grid result - contradicted.
Flown at 2/yr: 120 = 7 launches, perigee 20 670 km (reproduces published); 477 = **6**,
perigee 20 268 km, keyhole 5:8 exposure +1 048 km. Both axes needed (120x477 and 477x120 still 7).
The gain is one year-2 retro window, 4 492 vs 3 948 km/launch.

**Why:** best arrival per launch date = JUST INSIDE the edge where an N-lap Lambert
family stops fitting (on the edge the branches merge at a worse C3) (`NoSolutionForRevolutions`); real geometry (angle swings 12.5° per
7.55 d, family needs +15.8 d more), conics with C3 < 200 all close < 1 km re-flown.
Steep shoulder against the edge, so sampling error ≈ how far the last sample falls short of it.

**Method traps:** (1) use n = 2^k·119+1 sizes (239, 477, 953) so grids NEST — 240 interleaves;
(2) the probe flies plans with `CAMPAIGN_FLY_RATE`, sizes via `CAMPAIGN_GRID_SIZES`;
(3) C3 ≥ ~500 multi-lap roots do NOT close under a 1 h RK4 — never feasible, unverified.
Cost: 477 grid 17.8 s in the debug DLL (≈ release; core is opt-3 in dev), 953 ~100 s.

**RESOLVED 2026-10-01** -> [[campaign-continuous-search]] (user picked option 3). Was: leave 120 (count pessimistic by 1 at 2–4/yr) / campaign on its own
477 grid (+18 s; window boxes then sit between drawn cells) / per launch date maximise over arrival up to the edge (both axes still needed). Label 1/yr "at the line" whichever ships.
