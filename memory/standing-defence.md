---
name: standing-defence
description: "2026-10-08 Phase 3 standing defence PART 1 of 4 (level delays only the FIRST launch - later ones assumed ready at the cap rate, i.e. an unlimited stock) (user chose all 4: delay, stored stock, waiting in orbit, detection): first launch waits for warning + sourced delay (decide 172 d PDC23; from scratch 60 mo Nuth 2018; storage 1 yr); map axes stretched 0.70/0.92 -> 0.97/0.99 because the old launch-axis END was the whole 1/yr shortfall (now 10 launches, clear); ranking gate replaced by leave-one-out of estimate_shift."
metadata:
  node_type: memory
  type: project
  originSessionId: 1de99652-5c1e-44c1-9367-8ed3039a383d
  modified: 2026-10-08T11:30:39.815Z
---

Follows [[orbital-assembly]], [[campaign-rolling-cap]], [[launch-campaign]]. HANDOFF: *Standing defence: when the first rocket can fly*. Code: `core/src/readiness.rs` (ON_THE_PAD / IN_STORAGE / FROM_SCRATCH, shipping = last), binding `measure_campaign_candidates_from(.., earliest_launch_tdb: Option<f64>, ..)`, `WindowSearch.{launch,arrival}_end_fraction`, `estimate_shift` + `EstimateRule`, probe `probe_standing_defence` (`STANDING_FIRST_YR`, `STANDING_REACH`, `STANDING_FLY`). Frontend: `[`/`]` warning, `[W]` readiness on the `[4]` campaign panel; `Mission.begin_campaign(vehicle, readiness, warning_s)`.

**Sources:** decide = PDC23 exercise discovery 2023-01-10 -> ATP 2023-07-01 (172 d); build = "at least 48 to 60 months" Nuth, Barbee & Leung JSSE 2018 doi:10.1016/j.jsse.2018.07.002 (60 ships: "at least" makes 48 optimistic); storage = "much less than a year" -> 1 yr bound; DART Phase B 2017-06-23 -> launch 2021-11-24 = 53 mo (cross-check). DSCOVR: launcher procurement was the long pole, so IN_STORAGE assumes a launcher arranged.

**The cut is a fresh search** (launch dates stepped from it, years counted from it) - a filter would lose post-cut windows in a year whose best was pre-cut (advisor). Control: cut at map start == unrestricted, assert_eq on the whole CampaignCandidates.

**Big finding: the map's launch-axis end (0.70 = 3.6 yr out, a DRAWING choice) set the orbital-assembly headline.** 1/yr plan's last launch sat at 3.71 yr. Stretched (user chose search + drawn map) -> 1/yr on the pad = **10 launches, clear 1.2 %, flown perigee 20 310 km**, last launch 2.72 yr out. 2..12 stay 6; what-ifs 5-4/3/2. Lesson: before trusting a "falls short", print how late the plan's last date is and whether it touches an axis.

**Numbers (FH, found 12 yr out):** 6/yr = 6 on pad / 6 storage / 12 from scratch; 2/yr = 6 / 7 / short; from scratch 4/yr 16, 12/yr 11. Full table (first launch 12..4 yr) in HANDOFF. Read as warning: add 0 / 1.47 / 5.47 yr. **CAVEAT (advisor, final review): a level delays only the FIRST launch; every later one is assumed ready at the cap rate.** "6 from storage at 6/yr" = all six within ~80 days = a six-impactor stock; the only sourced stock is Nuth's TWO (nuclear-capable, not kinetic). "12 from scratch" = 12 built in parallel in 60 months. Never quote these counts without that.

**Test changes, not loosening:** (1) the global proxy gate (<1.5x) read 53 % — physics: shift/key rises and falls with ARRIVAL date (orbital phase), 2.1 -> 3.3, not along-track share (~0.3 both). Planner only scales from nearby flown windows, so gate = leave-one-out of `estimate_shift`: median 0.2 %, 90th 2.2 % (bound 5 %); nearest-arrival ties nearest-launch, rule unchanged; tail = isolated windows (87-138 d from a neighbour). Counts can be too high, never too low. (2) below-table guard went vacuous on stretched 24x24 and even 120x120 (1.065 > 1.0) -> 240x240 (0.204).

**How to apply:** quote readiness and warning with every count, and the first-launch-only caveat. Parts 2-4 still open; advisor proposed re-posing part 2 as a LIMITED STOCK (S ready at storage delay, rest no earlier than the build delay) - put to the user, not yet decided. Then interceptors waiting years in orbit (station-keeping no longer small), detection. Orbital-assembly height bracket not re-run on the new axes (offered, not done). Late-arrival (<0.4 yr) estimates are poor; short counts at short warning are upper bounds.
