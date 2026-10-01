---
name: launch-campaign
description: "2026-09-23 Phase 3 - multi-launch campaigns: core (chained impulses, core/src/campaign.rs) AND the [4]-map frontend ([C] panel, [Z]/[X] rate, [E] measure/fly). Cap is LAUNCHES PER YEAR (partition by Julian year from the grid's first launch date), not per grid row. FH-exp on 120x120: 7 at 2-3/yr, 6 at 4+/yr, unreachable at 1/yr."
metadata:
  node_type: memory
  type: project
  originSessionId: 2bcc0708-0589-4fa7-81f4-51b55a0d982d
  modified: 2026-09-23T17:33:11.167Z
---

Phase 3 started 2026-09-23 with multi-mission campaigns. Follows [[mission-porkchop]]'s
1/65th headline. Core and frontend both done the same day.

**Built:** `DeflectionScenario::campaign_trajectory`/`evaluate_campaign` (chained,
cadence-capped); `core/src/campaign.rs` `plan_campaign` (greedy per direction, cap
per `CampaignWindow::period`, brute-force-pinned incl. shared periods). Binding
(`godot/rust/src/mission_core.rs`): `measure_campaign_candidates` (every year's best
cell per push direction, all flown: ~18 flights, ~4 min) -> `CampaignCandidates::plan(cap)`
(free) -> `fly_campaign_plan` (one flight, ~20 s). Frontend: `[C]` on `[4]` swaps the
readout; `[E]` measures then flies (first flight auto on landing); `[Z]/[X]` rate 1..12/yr.
Harness `godot/tests/_campaign_shot.gd`.

**Why per year:** the old cap was per grid row, so 120 rows (26 d apart) allowed ~5x
the launches/yr of 24 rows. Rolling window rejected: not a partition, greedy breaks.
Anchor phase measured: 0/3/6/9 months -> 7/7/6/7 at 2/yr (within one, accepted).
Leftover grid dependence is window QUALITY (best retro shift/launch 4 094 / 3 160 /
2 218 km at 120/60/24 rows), so 24x24 still says 14 at 2/yr. 120 not proven converged.

**Numbers (FH-exp):** 7 launches at 2/yr, flown perigee 21 497 km, 0.02% off the
arithmetic, keyhole 7:11 646 km outside its band. 1/yr falls short at every year-start tried
(2 041 km short at the shipping start, 437 km at -6 months) - say "falls short", not
"unreachable". Fixed slots let a plan double the rate across a boundary: the 2/yr plan
has 4 launches in 7 months (2029-09, 2030-04); the panel prints PEAK 12 MO. Rolling-year
rule = open user decision (needs a real search, not greedy). Atlas V 551 (map default) at 2/yr: 18 launches reach 20 180 - unreachable.

**Traps:** (1) rank by `|dv| x lead`, not dv; (2) split candidates by push sign (nominal
sits ~2 300 km retrograde side); (3) always measure on the shipping 120x120 grid;
(4) the map's default launcher is Atlas V - a harness assuming FH will see "unreachable";
(5) sim.gd/porkchop.gd working copies were CRLF - edit as LF (git eol=lf normalises).

**Next:** ~~payload mass budget~~ DONE 2026-10-01 (see [[impactor-mass-budget]]: 4/yr now 7, not 6),
then orbital assembly against this baseline. HANDOFF: *Several launches against one rock*, *The
campaign on the map*.
