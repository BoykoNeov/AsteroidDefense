---
name: tier3-uncertainty
description: "Tier-3 orbit uncertainty — the covariance→b-plane→impact-probability layer, its pinned sample cadence, the quantised-argmin trap it exists to avoid, and what the keyhole batch still owes."
metadata: 
  node_type: memory
  type: project
  originSessionId: 9f278b27-03de-41f4-b45d-d40a579fd55e
  modified: 2026-07-28T07:15:03.095Z
---

`core/src/uncertainty.rs` (2026-07-28) is Phase-2's last roadmap item, first half.
It maps a 6×6 state covariance at `epoch0` through a measured 2×6 b-plane Jacobian
(`Σ_b = J Σ Jᵀ`) and integrates the result over the focused capture disc. Entry
points on `RealFieldScenario`: `bplane_sensitivity()` (13 propagations, ~14 s, the
covariance-*independent* half — reuse it), `bplane_uncertainty(cov)`, and
`bplane_uncertainty_checked(cov, n_sigma)` (adds the 12-run σ-shell, ~28 s).

**Four things that will bite anyone extending this:**

1. **Never build the Jacobian from each run's own closest approach.** CA is an
   argmin over a sampled polyline, so the map is quantised — a small perturbation
   moves the argmin a whole sample or not at all, and the columns come back noisy
   or identically zero *while the matrix still looks structurally fine*. Every run
   reduces at one fixed epoch (`UNCERTAINTY_REDUCTION_LEAD_SECONDS` = 12 h before
   nominal CA). Validated: fixed-epoch vs at-CA `∂r_p/∂v_along` agree to 0.025 %.

2. **`SAMPLE_CADENCE_DAYS` = 10 is a measurement, not a tuning knob**, and it lives
   in the module precisely so no caller can dial it. A Jacobian is only valid at
   the cadence its columns converged at. Same trap-shape as
   `REQUIRED_DV_AT_ONE_PERIOD` in [[threat-orbit]]. Steps are per-column
   (`FD_STEP_POSITION_M` = 312.5 m, `FD_STEP_VELOCITY_MS` = 1.25e-4) — metres and
   m/s share no scale. All three pinned by `cadence_is_pinned`.

3. **Impact probability is computed in *whitened* coordinates, not by quadrature
   over the disc.** Polar-on-the-disc silently returns 0.994 for an answer of 1
   whenever the ellipse is far smaller than the 11 311 km capture radius — which is
   the normal case. Whitening makes the radial integral analytic and leaves one
   periodic 1-D integral. Checked against `1 − exp(−R²/2σ²)`.

4. **`sigma_distance` is distance from Earth's *centre*, not from a hit.** The
   shipping campaign reads 8 196 σ *and* P = 1 simultaneously — no contradiction,
   the disc is 11 312 km and the ellipse is sub-km. Quote `impact_probability`.

**The ξ,ζ convention is still deferred and this batch proved it can be**: the
probability is invariant under any orthonormal b-plane basis change (rotation *and*
reflection — both tested). Keyholes are what force it, because a resonant circle
sits at a specific ζ.

**The keyhole batch is now scoped — read [[keyhole-reach]] before touching it.** It
settles which rock (the shipping one), which resonance (3:4 at 8.53 R⊕), why the
*far* keyholes are the wide ones, and the dormant landmine in
`uncertainty_sampling_plan`'s min-distance anchor that fires the moment the span
grows past a second encounter.

**Still owed by the keyhole batch:** keyholes/resonant returns, the ξ,ζ pinning,
real SBDB covariance ingestion (equinoctial/Keplerian elements, own epoch, mixed
units — validate the Cartesian conversion by round-tripping), and a frontend. Also:
the P sweep only *falls* (1.000 → 0.104 as σ_along widens) because the shipping
nominal is a designed hit; showing P *rise* as observations accumulate needs a
nominal miss, i.e. the deflected trajectory. And **re-run the σ-shell against a
real covariance**: `synthetic_along_track` is block-diagonal (no position–velocity
correlation), so its principal axes are not the ones a real correlated OD
covariance has — the shell probes twelve directions a real one would not pick, and
a real long axis may reach further into nonlinearity than the invented one does.

**Measured margins on the reduction epoch** (by moving the lead and watching the
kernel-gated test): 12 d fails at 107 %, 48 h at 6.5 %, 30 h at 2.3 %, 26 h passes.
Asymptotic invariance holds to about a day; the shipping 12 h has ~2.5× margin.

Probes: `probe_tier3_cost` (the cadence/step-size study) and
`probe_tier3_uncertainty` (three cross-checks + the σ sweep). See also
[[kernel-resolver]] — run these with `ASTEROID_REQUIRE_KERNELS=1`.

**2026-09-06 — the second half arrived, see [[keyhole-probability]].** The
covariance now maps through *both* encounters to a resonant return. Three things
that change how this file should be read:

- **`synthetic_along_track`'s "velocity dominates" claim is false** at the
  shipping numbers, at both encounters. The 1 km position σ contributes ~153 km
  of the 168.7 km first-encounter ellipse; the whole velocity block gives ≤32 km.
  This probe's own σ-ladder had shown it since July. To ask "how well is the
  orbit known", scale **both** arguments.
- **The 10-day `SAMPLE_CADENCE_DAYS` does not travel past a flyby.** Its
  cancellation argument was measured at one encounter; downstream of the 778×
  gain the nominal moves 2.97 % and the weakest column 17.7 %.
- **`impact_probability` is now checked at a 2.6e5 aspect ratio** against 4M
  Monte Carlo draws, and pinned to *refuse* past 2.6e7 where f64 loses the
  covariance. The `sigma_distance` oddity this file records (8 196 σ with P = 1)
  recurs at the return and is the *tell* for a mean moving along the covariance's
  own long axis.
