---
name: tier3-uncertainty-view
description: The Tier-3 uncertainty ellipse on the Godot b-plane view ([U]/[Z]/[X], 2026-09-06) — the frame trap that would have drawn a correct ellipse at a meaningless angle, the measured 1.95e-10 that closed it, and the finding that the spread lies on the timing axis.
metadata:
  type: project
---

Roadmap item 3, built and measured 2026-09-06. `[U]` on the b-plane view orders a
13-propagation sensitivity solve on a worker (**34.6 s measured**, not the ~17 s
the roadmap guessed), holds it, and then maps covariances through it for free;
`[Z]` / `[X]` turn a σ knob in half-decade steps over ±3 decades.

**The trap this batch existed to avoid.** `BPlaneSensitivity` carries a b-plane
frame `uncertainty.rs` itself calls *arbitrary-but-deterministic* (it seeds off
whichever coordinate axis is least aligned with `Ŝ`). Every **scalar** that module
reports is invariant under that choice and its tests pin the invariance — but **an
ellipse's orientation is not a scalar**. Drawing an angle computed in that frame
onto the view's pinned Öpik axes gives correct axis lengths at a rotation nobody
chose: wrong in exactly the way that looks right. So the covariance is rotated
into `(ξ̂, ζ̂)` in the core, and the rotation is measured rather than assumed —
`‖R Rᵀ − I‖∞` = **1.95e-10**, out-of-plane component 0.106 km against `|B|`
7 074 km. The two frames are *not* identical (Öpik is built on the
closest-approach reduction, the sensitivity's nominal on the fixed-epoch one) but
the disagreement is nowhere near enough to bend an ellipse.

**What it points at.** 1σ **168.71 × 0.82 km, 205:1, lying 0.3° off `ζ̂`** — the
*timing* axis. That is the same direction a Δv nudge moves (see
[[keyhole-targeting]]) and the same direction the resonant return's needle lies
along (see [[keyhole-probability]]). Three independent measurements now agree that
what is *uncertain* about this rock and what is *controllable* about it are the
same direction; the ellipse is the first place it is visible rather than tabulated.

**Three drawing decisions, each settled by measuring instead of preferring.**

- **Centred on its own mean, not on the drawn cross** — they are **5.43 km apart,
  6.6 minor axes**. Pairing them would centre one instrument's spread on another
  instrument's position. The gap is printed.
- **~1 px at the default zoom, and the view says so** rather than fattening it. A
  ring plus `1-SIGMA < 1 PX - ZOOM IN`, axes always printed in km. The smallness
  *is* the finding — the same shape of result as Apophis' real 18.2 × 0.48 km
  ellipse (see [[sbdb-covariance]]).
- **The σ knob only bites upward.** ×0.01 still reads P = 1.000000; ×100 is the
  first setting where P leaves 1, at **0.407** with a 16 871 km major axis. The
  rock is designed to hit, so a better-known orbit sharpens a certainty rather
  than removing it.

**Deliberately not on screen, with the reason stated in each place:**
`sigma_distance` (on its own it reads as "how many σ from a hit" and inverts the
answer — the designed hit is ~8 200 σ *with* P = 1, so `p_impact` and `capture_km`
are printed together and never apart); a **3σ ring** (the linearity check that
would vouch for it is 25 propagations and was not paid — still open); and a
**deflected** ellipse (the Jacobian is about the nominal seed, and an ellipse on
the cross beside a bare diamond would read as "the deflection is certain"). The
panel says **synthetic** because the rock is invented and its covariance is a
shape borrowed from real NEOs, using `probe_tier3_uncertainty`'s three constants
verbatim so panel and probe cross-check.

**Invalidation:** the held sensitivity is dropped whenever a scenario is installed
— a Jacobian is about one rock's trajectory, and `[N]` can put a different rock on
a different orbit between frames. The σ knob is *not* reset; "how well is the orbit
known" is a question about the layer, not a property of a threat.

**The rotation already existed, and now there is one of it.**
`probe_keyhole_map.rs` has computed the identical four dot products — with the
same orthonormality guard — since July, to draw the ellipse into
`docs/keyhole_map.svg`. The binding had its own copy before that was noticed. It
is now one method, `BPlaneBasis::rotation_to`, in `core/src/uncertainty.rs`,
returning the residual rather than gating on it (what counts as coplanar-enough
belongs to the caller). The payoff is a cross-check, not tidiness: the published
map records `[168.709999, 0.818223]` at `89.736°`, the live view measures
**168.710343 × 0.818218 at 89.737°** — two entry points, two reductions of the
nominal 0.05 km apart, one ellipse.

**Three faults that only a picture or a reviewer could find.**

- **`tier3_online` was never reset on rebuild.** Rust drops the Jacobian in
  `poll_build`, but the GDScript flag is set *once*, inside `_poll_tier3`, which
  returns early unless a solve is running. After `[N]`, `has_tier3()` was false
  while the flag stayed lit — the cached dictionary kept handing the **old rock's**
  ellipse to the view to draw on the **new rock's** b-plane, and `request_tier3`
  refused to re-solve. Fixed in `_invalidate_derived_views` beside the
  `pork_online` reset. **No test could catch it**: the harness never rebuilds the
  threat. *The lesson is the recurring one — Rust dropping a result does not reach
  the GDScript flag most consumers actually gate on.*
- **The readout was drawn on top of the HUD's event log.** Three nodes share that
  screen and the layout is not derivable from any one of them. Only the
  screenshot said so.
- **The three new keys had never been pressed.** The harness called the methods
  directly, so the hand-written `project.godot` action blocks and the `main.gd`
  dispatch branches were unexercised while the footer promised them to a player.
  It now drives them through `InputMap` / `Input.parse_input_event`.

Evidence: `godot/rust/src/mission_core.rs` (`Tier3View`, `Tier3Ellipse`, and
`the_tier3_ellipse_is_drawn_in_the_views_own_frame`, which prints every number
above), `godot/rust/src/lib.rs` (the eighth worker channel), `godot/scripts/sim.gd`,
`godot/scripts/encounter.gd`, and three shots in `godot/tests/_shot.gd`. See also
[[tier3-uncertainty]] for the core layer this draws, and [[godot-visual-layer]].
