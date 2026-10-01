---
name: impactor-mass-budget
description: "2026-10-01 Phase 3 payload mass budget: for a kinetic impactor the bus HITS the rock, so only burned propellant comes off (adapter is already out of the NASA LSP tables). Shipped IMPACT_MASS_FRACTION = DART 579.4/(615-14) = 0.964 in core/src/impactor_mass.rs. Moves FH count only at 4/yr (6 -> 7), all rates 1..10 measured; 5/yr holds at 6 by 0.4 %."
metadata:
  node_type: memory
  type: project
  originSessionId: 48ba07ef-f768-4c7e-b481-4e390166f41f
  modified: 2026-10-01T10:55:14.734Z
---

Phase 3's "payload mass budget" item, closed 2026-10-01. Follows [[launch-campaign]].

**The reframing (advisor caught my overstatement):** the old docs said "bus + propellant
out of delivered mass". Wrong for a kinetic impactor: bus, structure, avionics and
*unburned* propellant all hit and carry momentum. Only two things can leave:
(1) the launch adapter/separation system — **already out**: NASA LSP figures are
"separated spacecraft mass", separation system "book-kept on the launch vehicle side"
(SMEX 2007, MIDEX 2016, EVM-3 2020 ELV info summaries — the Girija paper and the elvperf
site itself do NOT say it); (2) propellant burned en route (arcs are ballistic).

**The number:** `IMPACT_MASS_FRACTION = 579.4 / (615 − 14) = 0.9641` — DART launch 615 kg
(APL Final Technical Report Oct 2023, Fig. 2), minus LICIACube ~14 kg (Dotto 2021 PSS
199:105185; released 2022-09-11), impact 579.4 ± 0.7 (Cheng 2023 Nature, *Extended Data*
Table 1). Cross-check test: DART's hydrazine dV99 55.2 m/s at MR-103G Isp 202–224 s ->
2.5–2.8 %, below the flown 3.6 % (which also includes ACS + xenon demo) but same range.
Scale-free (fixed Δv -> fixed fraction), so it carries to a 14 t impactor.

**Plumbing:** `CellDelivery` has `payload_kg` (what the rocket lifts) AND
`impact_mass_kg` (every push uses this) — deliberately two names, not a repurposed one.
`LaunchVehicle` stays the raw table. Solver seed/cap still sized off launch mass (bracket
params; answer seed-independent). `[M]` ratio and "(N LAUNCHES)" divide by impact mass.

**Effect (FH-exp, 120x120, every rate 1..10 measured with impact mass, 2/yr flown):**
short/7/7/**7**/6/6/6/6/6/6 — only **4/yr moved (6 -> 7)**. **5/yr is 6 by just 0.4 %** — the
next count a design margin would flip. 2/yr flown: perigee 20 670 km (was 21 497).

**Traps:** Python via bash heredoc on this box mangles `²` — write edit scripts to a file
under W:\temp\claude. Remaining optimism: no design margin held back from LV capability.
