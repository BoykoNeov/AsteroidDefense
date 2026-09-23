---
name: keyhole-lead-sweep
description: "The keyhole placement band was calibrated at a deflection lead the planner cannot dial; at dialable leads the same door sits up to 468 km from its circle, not 19 km."
metadata: 
  node_type: memory
  type: project
  originSessionId: 1da7331a-c9bf-4ea0-9f4f-9dc206d9aeaf
  modified: 2026-09-07T16:26:25.405Z
---

**2026-09-07 — roadmap item 4's sixth campaign, CLOSED.** The five-door placement
batch ([[keyhole-placement]]) left one named question: the door-placement error
was measured at exactly one point per circle, so whether it varies *along* one was
unknown — and the shipping `KEYHOLE_PLACEMENT_KM = 100.0` rested on those five
points. It varies by **24×**, and the answer moved the constant to **500**.

> **Partly superseded 2026-09-07 by [[keyhole-lead-variable]]**: the constant is
> now **800** (a 200 d door sits +786.0 km out), the width range is **0.89–2.11×**
> rather than 0.89–1.44×, trap 2 below has been *answered* (the crowded register
> does fire, at every dialable lead, once the search asks the register's own
> question and gates on the pass being a miss), and "the lead" is now established
> as the variable rather than one of three candidate labels.

**The knob is the lead time, not a sideways impulse.** The obvious design — an
out-of-plane nudge to move ξ — needs metres per second, shifts `v_inf` at
encounter 1 (so it moves the *true* circle away from the drawn one and that
artefact lands inside the measured placement error), and can't be held fixed
because `fly_keyhole_shot` applies `scalar × one direction`. None of it was
needed: `set_plan(lead_seconds, dv_along_track)` plus `sim.gd`'s `LEAD_MIN/MAX =
30..900 d` and `DV_MIN/MAX = 0.1..300 m/s` means the reachable b-plane set is a
**2-D patch**, every circle is crossed by a family of (lead, Δv) pairs at
different ξ, `KeyholeAiming.deflection_epoch` was already a field — and the Öpik
frame is built from the *nominal* encounter, so the lead carries no frame confound.

**The headline is not that it varies — it is where it was measured.** `T_IMPACT`
is 4383 d and `lead_cap()` clamps to `LEAD_MAX = 900`, so the campaign's 12 yr
lead is **4.9× beyond anything the planner accepts**. All five calibration doors
were flown there. The same 3:4 door, flown at leads that can be dialed:

    4383 d (not dialable) +19.3 km | 900 d +210.6 km | 450 d +467.9 km | 150 d no door at all

The **width** half survived across the first eight doors (0.89–1.44× the flown
one; the 200 d door later took that to 2.11× — see [[keyhole-lead-variable]]), so
the width/placement split was right; only the placement half was under-measured.

**Three traps this batch recorded.**
1. A vertical line cuts a resonant circle **twice**. A crossing search that takes
   the first sign change gets the near branch at short leads and the far one at
   long leads — the first run compared `b` = 7 859 km against 153 448 km and doors
   240× apart as "four samples of one circle". Printing `b` and the door width at
   every crossing is what exposed it; the near crossings were inside the capture
   disc, i.e. impacts at flyby 1, not keyholes.
2. "Tightest pair anywhere in the census" is **not** the spacing near a given
   plan. An assertion that the flown 900 d plan would find several doors in the
   band failed for exactly that reason (it finds 1). Crowding is real *somewhere*
   on the map, not everywhere.
3. A ladder is a property of **one lead**. `Ladder::load` now refuses a file whose
   `# lead_days` header disagrees, and a header-less file too.

**How to apply:** gate before flying — `probe_keyhole_placement xi_sweep <h> <k>
<branch> [leads...]` costs ~100 s per lead and flies no returns. Every stage takes
`lead=<days>`. Never present ξ as a smooth function of lead: eight samples
oscillate (+4390, +906, −9310, +4952, −4379, −15811, +5537, −52860 km) with no law
measured. And when quoting the placement band, say the lead — one figure
demonstrably does not cover the slider.

Related: [[keyhole-placement]], [[keyhole-ranking]], [[keyhole-targeting]],
[[keyhole-reach]], [[gdext-binding]].
