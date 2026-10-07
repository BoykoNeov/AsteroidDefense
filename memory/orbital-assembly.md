---
name: orbital-assembly
description: "2026-10-07 Phase 3: orbital assembly built as PARKING (joining buys nothing under a linear push). Shipping = Falcon's published 26.5 t single-payload limit + storable OMS-E 315.1 s, escape burn FLOWN FINITE (8 firings, 400 km, 100 000 km apogee cap - user's pick): 1 in any 12 months FALLS SHORT 1.15 % (was at the line on an idealised burn), 2..12 stay 6. Plane drift free (pre-aimed), 28.5 deg tilt is the only limit. Plus the furthest-plan planner bug and ROLLING_SLACK_S."
metadata:
  node_type: memory
  type: project
  originSessionId: 124ce3c2-7c12-4a1d-978f-3389d87fded8
  modified: 2026-10-06T21:26:06.114Z
---

Follows [[campaign-rolling-cap]], [[launch-campaign]], [[impactor-mass-budget]]. HANDOFF: *Orbital assembly: it is parking, not joining*. Code: `core/src/orbital_assembly.rs`, `core::campaign::parked_launches` / `parked_window`, binding `CampaignCandidates::with_parking` / `direct_only`, probe `probe_orbital_assembly`.

**Premise that died at the sources:** user first chose "best date + more mass" on SpaceX's 63.8 t LEO figure; the Falcon User's Guide (2025-05-09, Table 4-1) caps ONE payload at 26 500 kg (strut PAF, "initial guide") and publishes no LEO numbers. A parked payload also needs its own departure stage (Falcon's upper stage can't wait months). User re-chose "honest default + what-ifs". Lesson: an advertised capacity is not a per-payload limit - check the user's guide's adapter table.

**Physics:** `m_out = stack * exp(-dv/(Isp g0))`, dv from 185 km circular to C3; spent stage stays on and hits. Parking carries LESS than direct FH below C3 58.0 (62 % at C3 1, 80 % at 38.4), more above. So parking wins by DATE, not mass. Storable because waits reach ~488 d (no sourced boil-off rate); OMS-E 315.1 s = Belair et al. SP2024_382 Table 2 (NTRS 20240003648); RL10B-2 465.5 s = NRC 2006 App. D (what-if only). Aestus 324 s was secondary-sourced only - not used.

**Planner, exact:** parked dates = earliest or any direct window date, + whole periods (value falls with date, so each sits as early as its chain allows); departures pruned to convex-hull vertices per date. `ROLLING_SLACK_S` = 1 ms shared by planner and `busiest_rolling_count` (a one-sided date margin broke "parked then direct one period later"). Brute force alone could NOT see the window-based dates (mutation-tested) - `a_parked_launch_can_follow_a_direct_one` does.

**Bug fixed:** unreachable plans reported the MOST-launch plan, not the furthest (more launches can force weaker dates). Direct numbers unchanged; measurement flies 34 direct windows not 40.

**Measure:** parked departures only through flown windows; per (year, direction) the best pool window by key at PARKED mass is flown too (+13 flights, 34 -> 47) - that took 1/yr from short 5 % to at the line.

**Result (FH exp):** 1/yr AT THE LINE, 9 launches (7 parked), |B| 25 739/25 955, flown perigee 19 790 km (fine seed 19 808 - retrograde, converged); 2..12 = 6. What-ifs: hydrogen 5 / 4, 63.8 t storable 3, both 2.

**Bias of known sign (review catch):** the 1/yr "at the line" is 44 km inside the 1 % band = ~7.5 m/s of extra escape burn per parked launch. Unmodelled and plausibly bigger: gravity losses (~40 min burn at 0.1 g; OMS-E max burn 1 030 s -> >= 3 burns) and the parking-orbit plane (fixed at launch, J2-regressing ~50 d cycle - NOT equal footing with direct). So: at the line on an idealised burn, likely just short in reality.

**Launcher gate (review catch):** `parked_delivery_for` - only FH expendable parks (needs a published single-payload limit AND evidence it lifts it to LEO). First cut parked every launcher; checks only ever measured FH exp. Vulcan verified: no parking, plans == direct-only.

**SUPERSEDED same day - the escape burn flown for real (`core/src/departure_burn.rs`, HANDOFF *The escape burn flown for real*):** OMS-E limits re-read at source (Belair Table 2: 26.7 kN, max firing 1 030 s, **10 starts max**). Planar two-body, tangential thrust, n-1 equal phasing firings at perigee + final solved to C3; rule = Nelder-Mead optimum to 0.1 m/s. The FINAL firing (~500-700 s, from near-escape) dominates the loss, so more firings help less and less. Loss tabulated per delivery on C3 0..100 step 1, process cache. Reviewer catches: (1) adding the loss while keeping 185 km stacks only parking's costs - 185 km decays in weeks, 400 km saves ~45 m/s, so height bracketed; (2) the apogee cap needs a stated reason (Moon tide 0.04 % at 100k km). Bracket (38 settings, `ASSEMBLY_BRACKET=1`): 185 km short everywhere; 400 km flips with firings x cap (8/100k short 25 657; 8/200k at line by 2 km; 10/400k at line 25 732). **User chose 400 km / 8 firings (2 spare) / 100 000 km -> 1/yr SHORT 1.15 %, flown perigee 19 712 km; 2..12 = 6 in every setting.** Trap: at high C3 the feasible phasing lengths are a sliver under the cap edge - a blind scan priced C3 > 95 at zero mass; now edge-first (test pins whole table). **Plane:** pre-aimed by launch time of day (departure date known at launch, J2 drift predictable) -> free; only limit is |declination| <= 28.5 deg (`parking_plane_reaches`, steeper windows not offered to parking). 7/44 flown windows steeper (to 58.4 deg), none used. NASA LSP tables' DLA assumption NOT findable (site, info summary, Girija paper) - direct side unknown. What-ifs stay impulsive, now at 400 km.

**How to apply:** quote "with parking, 1 in any 12 months falls short by ~1 % once the escape burn is flown in real firings (at the line only under looser settings); 2+ stays 6; FH expendable only". Unmodelled: station-keeping during the wait (small at 400 km, the 2 spare starts), Moon/Sun in phasing loops. Open: second-best date of a year not offered to parked launches; what-ifs reuse shipping-chosen windows. Phase 3 remaining: standing defence systems.
