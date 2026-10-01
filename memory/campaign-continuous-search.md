---
name: campaign-continuous-search
description: "2026-10-01: campaign windows now come from a continuous search (2 d launch steps, arrival zoom + lap-edge bisection, launch polish), not map cells. FH: 6 at 2..10/yr, 9 at 1/yr (AT THE LINE, flown 0.8 % clear). Retrograde converged at every seed; prograde NOT (lower bound, no plan uses it). 'At the line' = within 1 % of target |B|."
metadata:
  type: project
---

Resolves [[campaign-grid-convergence]]; follows [[launch-campaign]], [[impactor-mass-budget]].
HANDOFF: *The windows searched continuously*.

**Built:** `core::mission::TransferEvaluator` (the map cell at ANY dates; `porkchop_grid` runs
through it, kernel test pins equality) + `maximise_on_interval` (zoom, not golden - the key
has a cliff at lap edges). Binding `search_campaign_windows` / `SHIPPING_WINDOW_SEARCH`
(2 d x 120), `WindowSource::{GridCells, Continuous}`; `CampaignCandidate.v_rel_vec` (the
nearest cell holds a DIFFERENT transfer - never re-read metrics from launch/arrival_index).
Map boxes drawn at window dates. Search ~17 s on 16 threads.

**Bar (written first):** every slot >= 953 map (pass), counts = 953 at 1..10 (pass), seed
independence (retro pass, PROGRADE FAIL), flown: 2/yr 6 launches perigee 20 533 km, 1/yr 9 at
20 163 km. Control: continuous arrival on 120 rows alone still 7 at 2/yr (launch axis matters).

**Why prograde left unconverged:** winners hug lap edges (<1 h) near fixed-year starts and move
with seed (yr0 0.063-0.079); edge bisection + sampling year boundaries changed nothing at the
shipping seed. Every plan 1..10/yr is retrograde (nominal ~2 300 km retro side). Could matter
on an [N]-designed rock; rolling-year cap removes the boundary.

**"At the line":** `CAMPAIGN_AT_LINE_FRACTION = 0.01` in sim.gd (search vs 953 part 0.7 % in
flown perigee at 1/yr). Margin-based, not rate-based.

**How to apply:** quote 6 / 9 (at the line), not 7. Probes: `probe_campaign_continuous_search`
(flown, ~6 min), `probe_campaign_search_seeds` (no flights). Next: rolling-year cap (user's
item 2), then orbital assembly.
