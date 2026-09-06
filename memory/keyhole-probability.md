---
name: keyhole-probability
description: "Tier-3 at the resonant return (2026-09-06): the chained two-encounter Jacobian, the 778x flyby gain checked against the closed form, and the finding that at a keyhole P is set by how well the orbit is KNOWN, not how well the impulse is aimed - for as long as the uncertainty is larger than the door."
metadata:
  type: project
---

**Roadmap item 2 ("P(impact) rising near a keyhole") is DONE 2026-09-06 — and it
does not rise.** `core/src/keyhole_target.rs` gained `ReturnSamplingPlan` /
`chained_sample` / `return_sensitivity`; the driver is
`core/examples/probe_keyhole_probability.rs` (modes `check | steps | gain |
probability | cadence | sweep`). Extends [[tier3-uncertainty]] and
[[keyhole-targeting]].

**The chaining is in the propagation, not a matrix product.** Each of 13 columns
flies a perturbed seed from the campaign start through the impulse, through
encounter 1, across a **fixed** handoff and on to the return. Every epoch —
handoff and both reduction epochs — is pinned from one nominal flight, because
two encounters give the argmin-quantisation trap twice as many seams.
`uncertainty_sampling_plan`'s two-encounter refusal was never in the way: it
fires on the *nominal* census, where the return does not exist. Don't relax it.

**The headline, and it is not the expected one.** At the shipping (invented)
covariance the return's 1σ ellipse is a **needle 131 334 km × 0.5 km** against an
11 310 km capture disc, and the needle lies along ζ₂ — the timing coordinate,
which is the *only* thing Δv moves. So sliding Δv slides the mean along the
needle and cannot change the disc overlap: **P = 0.064 flat across the whole
door, contrast 1.02×** over 21 points. Scale the **whole** covariance down 12×
(needle ≈ disc) and the door appears as a 12.1× peak (0.055 → 0.665 → 0.055);
100× and it is a hard 0 → 1 → 0. **At a keyhole the probability is set by how
well the orbit is known, not by how well the impulse is aimed — for as long as
the uncertainty is larger than the door.** State it conditionally: it is a fact
about *this needle*, whose length is dominated by the 1 km isotropic position σ,
the most arbitrary number in an invented covariance. What travels is the
mechanism (uncertainty and impulse push along the same coordinate, so they cannot
trade against each other), not the row this rock sits in.

**`J` is constant across the door, measured, not assumed.** A Δv change at the
campaign start *is* a seed velocity perturbation, and the step study drove one
larger than the whole door (1.25e-4 m/s vs a ±5e-5 door) with the column holding
to 0.003 %. No need to re-fly 13 columns at the door's edge.

**Three traps this batch caught, all of the same family (a number stating a claim
instead of describing a measurement):**

1. **The probe's own closing narration** asserted P "swings from near-certain to
   near-zero across the door" — written before the run, contradicted by it.
   Deleted; the sweep now prints a measured contrast ratio per covariance.
2. **The first σ-ladder varied `σ_along` alone** and the ellipse did not move
   (131 334 → 128 501 km for a 100× change). At the return the **position** block
   dominates: position columns ~1.2e5 b-plane m/m, so 1 km of position σ gives
   ~117 000 km against the velocity block's ~24 500 km.
3. **`synthetic_along_track`'s doc claimed "the along-track velocity term
   dominates the map"** — false at *both* encounters, and its own probe's σ-ladder
   had shown so since July (ellipse 165.2 km at σ_along 1e-5, 168.7 km at 5e-5).
   Doc corrected with the measured numbers. **To ask "how well is the orbit
   known", scale both arguments.**

**The flyby gain is 778×, and it is a property of the encounter.** Position and
velocity blocks give 777.5 and 777.8 — the same to four figures — because both
act through one scalar channel: encounter-1 b-plane displacement → `a'` → period
→ arrival time.

**Finite-difference steps had to be re-measured, and the risk was real but did
not bite.** `bplane_jacobian` gained `bplane_jacobian_with_steps` + `FdSteps`.
The shipping steps provoke **78 559 km** at the return — 7 capture radii, a
secant across seven times the feature. But the plateau study says the columns do
not move (0.04 % over a 100× step range): the state→return-b-plane map is linear
far wider than needed, so the shipping steps *would* have worked. Only the
measurement could say so. Shipping run uses 1 % of them.

**The only available cross-check is the closed form, because an invented
covariance cannot falsify a Jacobian.** `gradient_semi_major_axis` → `ΔT/T = 1.5
Δa'/a'` → arrival slip over h years → Earth at 30.278 km/s predicts `∂ζ₂/∂ζ₁ =
−905.9` against the measured **−777.7**: ratio **0.858**, signs agreeing, across
a flyby and three years of propagation.

**Keyhole width, and what it does NOT settle.** `keyhole_at`'s width *is*
`2·capture_radius ÷ closed-form gain`, so substituting the measured gain gives
**29.09 km** vs the map's 24.92 km — the linearised width is conservative by
**1.17×**, measured differentially (immune to the map's placement error). This is
the 0.858 ratio inverted, **not** an independent second result, and it does
**not** retire the "flown b-point sits 1.64 half-widths from its circle"
placement finding — a different quantity, still open.

**The 10-day cadence does NOT survive the flyby.** The cancellation argument
behind `SAMPLE_CADENCE_DAYS` was measured at a single encounter; here the residue
is multiplied by 778. Measured: the nominal return `b` moves **+2.97 % (122 km)**
— that is the *mean*, which no differencing protects — and **column 2 is 17.7 %
off**. So `ReturnSamplingPlan` carries its cadence as a field and the shipping
path flies **1-day**. First time "a Jacobian is only valid at the cadence its
columns converged at" has actually bitten.

**The needle forced a conditioning check on `impact_probability`** (aspect
2.6e5, far outside anything it had been tested at). Two kernel-free tests added:
`a_needle_ellipse_matches_monte_carlo` (4M deterministic draws, fixed-seed
xorshift + Box–Muller, no new dependency) and
`needle_probability_holds_until_f64_loses_the_covariance` — P holds to aspect
2.6e7 and the module **refuses** (NotPositiveDefinite) one decade further, where
`(σ_long/σ_short)² = 6.9e16` exceeds f64. `StateCovariance`'s validation is
load-bearing, not ceremonial.

**Cost:** ~110 flights at ~45 s each in release. Never a build path or a frame.
