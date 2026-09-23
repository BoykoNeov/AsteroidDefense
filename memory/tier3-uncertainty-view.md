---
name: tier3-uncertainty-view
description: "The Tier-3 uncertainty ellipse on the Godot b-plane view — [U] to solve it on a worker, [Z]/[X] to dial sigma, the frame trap that made the rotation necessary, and the 2026-09-07 check that its drawn *shape* survives the 3-sigma shell."
metadata: 
  node_type: memory
  type: project
  originSessionId: 35de2e7e-a778-4c7a-b5b7-19795946f450
  modified: 2026-09-07T13:03:39.035Z
---

`[U]` on the b-plane view draws the orbit's 1-sigma spread; `[Z]`/`[X]` ask the
same question at a better- or worse-known orbit. Built 2026-09-06 (roadmap item
3), shape-verified 2026-09-07 (roadmap item 8). Lives in
`W:\Claude_projects\AsteroidDefense\godot\rust\src\mission_core.rs` (`Tier3View`,
`Tier3Ellipse`) and `W:\Claude_projects\AsteroidDefense\godot\scripts\encounter.gd`.

**The frame trap it was designed around.** The sensitivity's b-plane basis is
*arbitrary but deterministic* — it seeds off whichever coordinate axis is least
aligned with the incoming asymptote. Every **scalar** the uncertainty module
reports is invariant under that choice, and its tests pin the invariance — but an
**ellipse's orientation is not a scalar**. Drawn in that basis the picture would
have had the right axis lengths at a rotation nobody chose: wrong in exactly the
way that looks right. The covariance is rotated into the view's pinned Opik axes
**in the core**, once, and the two planes agree to `1.95e-10`.

**What it draws:** 1-sigma of **168.71 x 0.82 km lying 0.3 deg off zeta-hat** —
the *timing* axis, the same direction a Delta-v nudge moves and the same
direction the resonant return's needle lies along. At the default zoom that is
about **one pixel**, and the view says "1-SIGMA < 1 PX - ZOOM IN" rather than
fattening it. The solve is 13 propagations (**34.6 s** measured, not the ~17 s
the roadmap guessed), so it runs on a worker and is *held* — the sigma knob is
then free. The centre sits 5.43 km from the nominal cross because the two are
different reductions (fixed-epoch vs closest-approach), and the view prints the
gap rather than hiding it.

**2026-09-07: the drawn shape survives the 3-sigma shell — but the number that
was supposed to say so could not see it.** `LinearityReport::max_relative_residual`
normalises against the shell's largest displacement, which on a 205:1 needle *is*
the major axis, so it is structurally blind to the minor axis — the drawn
*width*. At the sigma knob's top stop the scalar reads **0.0043** while the
minor-axis residual reads **0.561**: a factor of 130. Resolved per axis
(`core/examples/probe_tier3_drawn_shape.rs`, guarded by
`core/tests/tier3_drawn_shape.rs`), the shape holds everywhere the knob reaches —
0.001 of the drawn half-width at the shipping covariance, 0.561 at the top stop,
extrapolating to a crossing at scale ~2e3 against a knob that stops at 1e3.

Two things worth carrying forward from that check:

- **The residual is a noise floor plus curvature, and only one is physics.** It
  is U-shaped in the sigma knob, and the naive read of the small end — "the
  picture is least trustworthy when the orbit is best known" — is backwards.
  Re-running at `forward_rtol = 1e-13` drops the small-covariance residual a
  hundredfold (30 m -> 0.3 m) and leaves the large-covariance one alone
  (15.50 -> 15.33 km). Same shape of finding as [[integrator-convergence]].
- **These particular ratios are frame-invariant**, contrary to first instinct:
  residual and axis are in the same basis, so a common rotation leaves their dot
  product alone. Only the printed *angle* needs the rotation. The frame trap
  above is real; it just does not reach here.

`bplane_uncertainty_checked_many` (core `scenario.rs`) shares one sensitivity
*and one sampling plan* across n shells — the plan sharing is correctness, not
speed: two plans that drift apart make the shell difference its displacements
against a mean measured at another epoch and report it as curvature.

Nothing changed on screen as a result, deliberately: a 3-sigma ring would now be
honest to draw, but 1-sigma is already ~1 px at the default zoom. See
[[tier3-uncertainty]] for the layer underneath, [[keyhole-probability]] for the
resonant-return half, and [[godot-visual-layer]] for the view it sits on.
