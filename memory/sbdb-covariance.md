---
name: sbdb-covariance
description: "Real JPL orbit covariances (2026-09-06) — the SBDB publishes cometary elements in an 8x8 matrix at its own epoch, a round-trip cannot validate the conversion, and Apophis' real 2029 ellipse turns out to be the same size as our own dynamical error."
metadata: 
  node_type: memory
  type: project
  originSessionId: 53a8ab21-f76d-4dc0-9e53-a0ad6ab55ab3
  modified: 2026-09-06T08:41:22.152Z
---

**Roadmap item 1 ("real covariances from the SBDB") closed 2026-09-06.** New:
`core/src/sbdb.rs`, `core/src/frames.rs`, `pyref/fetch_sbdb_covariance.py`, the
committed fixture `core/tests/fixtures/apophis.sbdb`, and two probes —
`probe_sbdb_covariance` (kernel-free) and `probe_sbdb_apophis_2029`.

**Three things the roadmap line got wrong about what JPL publishes.** Not
equinoctial or Keplerian — the covariance is over the **cometary** set
`(e, q, tp, node, peri, i)`, on every NEO sampled. Not 6×6 — Apophis' is **8×8**
(the non-grav `A1`/`A2` are estimated alongside the orbit; Bennu's carries
`RHO`/`AMRAT`), and dropping the trailing rows *is* marginalisation, so the
leading block is right but the fact must be recorded, not trimmed quietly. And
the covariance has **its own epoch**, 5.43 years from the element epoch for
Apophis — moving it needs a state-transition matrix we do not have, so
propagation starts where the covariance lives.

**A round-trip could not have validated this** — forward and reverse share one
unit convention, so a degrees-for-radians slip cancels exactly and the test
passes. Three external gates instead, each able to fail alone: JPL's published
per-element σ against `sqrt(diag)` (exact, and enforced *at parse time*); JPL's
own Cartesian state at the covariance epoch carried in the fixture for both
frames (residual **22.75 m** with a hardcoded μ_sun, **1.4 m** with the kernel's
— confirming the documented ~30 m μ sensitivity); and a **Monte Carlo in element
space** against `J Σ Jᵀ` (agrees to < 5 %, the tolerance set by 1/√N). See
[[tier3-uncertainty]] for the module this feeds.

**The cigar is tilted 9.8° off velocity and that is physics, not a bug.** Per
element the contributions are far *larger* than the total (`node` alone 8 379 m,
`peri` 9 022 m, `tp` 1 518 m, against 655 m for all six): the element errors are
strongly correlated and **cancel by 14×**, and the node/peri residual at 7.85° is
what survives. Only the pure-timing term is exactly along-track — measured at
**0.00°**, which doubles as an independent check on the `tp` Jacobian column.

**The trap: reducing outside Earth's sphere of influence looks exactly like a
dynamical error.** Fixing the b-plane reduction epoch 3 days before Apophis' 2029
closest approach puts it 1.5 million km out, past the ~924 000 km sphere, where
the osculating geocentric hyperbola is not the encounter — reported perigee was
**4 125 km** off JPL's published approach. `UNCERTAINTY_REDUCTION_LEAD_SECONDS`
(12 h) is a **physical** constant, not a numerical one; at that lead the answer is
37 984 km against ~38 000, i.e. 16 km.

**Headline: the real ellipse and our own error are the same size.** Apophis'
covariance flown to 2029 gives **18.2 km × 0.48 km** and **P = 0** at 19 571 σ
(the correct answer — it misses). Our own position residual against JPL over the
same arc is **15.1 km**. So the ellipse is honest about JPL's astrometry and
silent about the physics we omit (every planet's relativity, the radial `A1`),
which moves the nominal by just as much. Ingesting a real covariance did not make
a prediction real — it made the *other* error term visible. Measure that residual
a year **short** of the flyby: at the reduction epoch it reads 14 391 km, which is
the 1-day `.neo` table's own interpolation error, not ours.

**Also: there is now exactly one obliquity in the project** (`core/src/frames.rs`,
84381.448″ = SPICE `ECLIPJ2000`, not IAU 2006's 84381.406″); the Godot binding's
three helpers delegate to it — see [[gdext-binding]].

**Still open:** Bennu deliberately (its solution estimates SRP parameters, not
`A2`, so its covariance describes a propagation we do not reproduce); the
state-transition matrix that would carry a covariance to another epoch; and none
of this is on the frontend — the Tier-3 ellipse on the Godot b-plane view is
still the next visible thing ([[godot-visual-layer]]).
