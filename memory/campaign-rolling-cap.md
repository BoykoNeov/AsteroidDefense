---
name: campaign-rolling-cap
description: "2026-10-01: shipping launch cap is now N in ANY 365.25 days (rolling), planned EXACTLY by N chains a year apart (core plan_campaign_rolling, brute-force pinned). FH: 2..12 unchanged at 6; 1/yr now FALLS SHORT (flown perigee 16 542 km at shipping seed, 18 065 at finest) on a PROGRADE plan. Estimates choose dates, flights count (proxy tracks shift only to 24 %)."
metadata:
  type: project
---

Follows [[campaign-continuous-search]], [[launch-campaign]]. HANDOFF: *A launch cap in any 12 months*.

**Rule:** at most N in any 365.25 d (half-open). Equivalent to N chains with gaps >= P (deal
sorted launch i to chain i mod N). DP per chain + split across chains; `busiest_rolling_count`
in core is the one count (binding `busiest_rolling_year` delegates). Fixed-year planner kept as
`plan_fixed_years` - its count is a LOWER BOUND on rolling (a fixed year is a rolling window).

**Estimates vs flights:** pool = search's per-date profile; shift estimated from nearest flown
window same direction + laps, scaled by key. Key tracks flown shift only to 24 % (retro
79k-98k km per m/s*yr) - the old "flat to 4 %" was six windows. Loop per rate 1..12
(`CAMPAIGN_MAX_RATE`, must be >= sim.gd CAMPAIGN_RATE_MAX): plan, fly chosen pool windows,
replan until only flown windows used - EACH DIRECTION PLANNED ALONE (else the losing direction
is ruled out on estimates), sweep repeated until a pass flies nothing. Flights parallel: measure
3.5 min -> 85 s, 40 flown.

**Result:** 2..12 per 12 mo = 6 (same plans as fixed). 1 per 12 mo FALLS SHORT both directions,
both seeds (flown perigee retro 15 862 / 16 197, PRO 16 542 / 18 065 km at 2x120 / 0.5x477);
best = prograde. Counts are flown counts for windows chosen BY KEY - another date could push harder. The "prograde is a lower bound no plan uses" claim is retired - 1/yr uses it.

**How to apply:** quote "6 at 2..12 in any 12 months; 1 in any 12 months falls short
(by a seed-dependent 8-14 %)". Converging prograde would firm up the 1/yr gap. Next: orbital assembly.
