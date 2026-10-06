---
name: orbital-assembly
description: "2026-10-07 Phase 3: orbital assembly built as PARKING (joining buys nothing under a linear push). Shipping = Falcon's published 26.5 t single-payload limit + storable OMS-E 315.1 s; 1 in any 12 months goes short -> AT THE LINE (9 launches, flown perigee 19 790 km, seed-independent), 2..12 stay 6. Hydrogen/63.8 t are labelled what-ifs. Plus the furthest-plan planner bug and ROLLING_SLACK_S."
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

**How to apply:** quote "with parking, 1 in any 12 months is at the line (9 launches); 2+ stays 6". Open: second-best date of a year not offered to parked launches; what-ifs reuse shipping-chosen windows. Phase 3 remaining: standing defence systems.
