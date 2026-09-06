---
name: integrator-convergence
description: "Roadmap item 5 (dop853 → IAS15) retired by measurement 2026-09-06: dop853 IS converged where it ships, but only because the 1-day snapshot cadence caps the step — the shipping tolerance itself is 129 km loose. Plus the cadence×tolerance cross, the 5 558× flyby amplification, and why IAS15 has no oracle."
metadata: 
  node_type: memory
  type: project
  originSessionId: eba1cf66-04fe-4b97-a80d-284440441739
  modified: 2026-09-06T12:06:09.147Z
---

`core/examples/probe_integrator_convergence.rs` (2026-09-06). Roadmap item 5 —
"dop853 → IAS15 crossover" — is **retired, not deferred**, and no second integrator
was written. Four modes: `determinism` (the bit-for-bit gate, run it first),
`campaign`, `cadence`, `keyhole`.

**The pitch for the item was wrong and the correction matters.** It was argued from
[[sbdb-covariance]]'s finding that our 15.1 km residual vs JPL rivals the real
18.2 km uncertainty. That residual is **unmodelled forces** (planetary GR, radial
`A1`), not truncation — a better integrator does nothing for it. The real customer
was that every keyhole conclusion rests on 15-year arcs nobody had checked.

**The headline, and it is a trap not a triumph: the shipping accuracy is a property
of the architecture, not of the tolerance.** `Dop853`'s error scale is
`atol + rtol·|y|`, so the shipping `rtol = 1e-9` in SI allows ~150 m of local
position error per step at 1 AU (`atol` never binds). Over the 12-year campaign,
measured on the cruise a year short of the flyby:

| same integrator, same tolerance | `|Δr|` vs converged reference |
|---|---|
| one uninterrupted `step` call | **129 km** |
| the shipping clock path (1-day snapshots) | **0.26 m** |

`Clock::propagate` restarts the controller at every snapshot, so a 1-day snapshot
**caps the step** — and the cap is the whole reason the shipping numbers are good.
Anything that lengthens the effective step spends that silently.

**Always compare a year SHORT of the flyby.** The campaign span ends 60 days past a
3 000 km Earth pass, which multiplies whatever arrives at it by a measured
**5 558×** (steady across four tolerances → a linear amplification, not noise). The
end-state figures (719 000 km / 1.4 km) are the cruise figures times that. Closes on
itself: clock 0.2582 m × 5557.9 = 1435 m = the measured end-state column exactly.

**The cross that decides it** — encounter-1 perigee, shift vs that row's 1-day cell:

| rtol | 10 d | 30 d | 180 d |
|---|---|---|---|
| `1e-9` (shipping) | +117.7 m | **+13 566 m** | +27 360 m |
| `1e-13` | +0.36 m | +0.51 m | **+0.68 m** |

The coarse-cadence penalty shrinks 40 000× with tolerance, so it was step size all
along. Three consequences:

1. **At 1-day cadence the tolerance is not binding** — tightening four decades moves
   the perigee 0.07 m for +33 % wall clock. Shipping value stays `1e-9`.
2. **`ImpactorConfig::forward_rtol` is now a named knob**, threaded through a single
   `stepper` field on `RealFieldScenario` so the nominal, the re-flies and
   `propagate_free` cannot diverge in tolerance. Guard test
   `a_coarse_cadence_is_tolerance_bound_where_a_fine_one_is_step_capped` asserts the
   **ratio** collapses (fine < 1/1000 of coarse) — magnitudes are machine-specific,
   the separation is the physics. Same trap-shape as `SAMPLE_CADENCE_DAYS` in
   [[tier3-uncertainty]] and `REQUIRED_DV_AT_ONE_PERIOD` in [[threat-orbit]].
3. **The efficient pairing is the OPPOSITE of the shipping one**: 180 d at `1e-13`
   reaches the 1-day answer to 0.68 m in **0.56 s** vs shipping 1 d/`1e-9`'s
   **10.9 s**. An 8th-order method wants few large accurate steps. The shipping
   cadence still stays 1 day — the frontend draws the arc.

**Every keyhole conclusion is converged** ([[keyhole-targeting]] is safe). At fixed
Δv = 0.216550 the timing coordinate `ζ₂` moves **365 m out of 891 km** (0.04 %),
`ξ₂` 3 m out of 4 014 km, and `|ζ₂/ξ₂|` = 0.2219 at every rung. The column
**scatters rather than converging** (−365, −349, +127, +59 m) — that is *stronger*
evidence than a monotone fall, because it means the tolerance is not in control at
1-day cadence.

**The 118 m at the Tier-3 cadence is NOT a Tier-3 error.** Every cross cell is a
*nominal* perigee; Tier 3 reads derivatives. `∂(perigee)/∂v_along` at 10 days moves
**0.0225 %** between `1e-9` and `1e-13` — the 0.024 % the cancellation argument
already predicted. `SAMPLE_CADENCE_DAYS` stays 10 on the reasoning it always had.
The pairing matters only once an *absolute* b-plane position is read — i.e. drawing
the Tier-3 ellipse on the frontend (roadmap item 3).

**Why IAS15 has no oracle** (stronger than the cost argument, and the actual reason
the item is retired): HANDOFF §6 nominates REBOUND and in the same breath says it
self-gravitates the planets, so it is *not* a trajectory oracle. `horizons.rs`
documents that a 1-day table cannot resolve this class of flyby (18 885 km of its
own interpolation error), and a resonant return is in no truth table. The only
available oracle is dop853-at-tighter-tolerance — which says dop853 is converged.

**Two things recorded so they are not rediscovered:**
- **Reversibility (forward-then-back) is a bad invariant here** and is reported but
  not used: it falls four decades then jumps at the tightest rung, because it
  round-trips the flyby twice and both cancellation and amplification act on it.
- **A +0.68 m residual survives a tight tolerance** and is *not* the scan: refining
  `max_sample_dt` 24× (6 h → 15 min) does not move it by a bit. That does not
  separate the degree-7 dense interpolant's own accuracy over larger sub-steps from
  a real trajectory difference; doing so needs a re-integration to the CA epoch and
  was not done. Changes no conclusion.

**One unresolved inconsistency found:** `ζ₂` reproduces as **891 km** (matching
[[keyhole-targeting]]) while `core/src/keyhole_target.rs`'s module-doc table says
**786 km** for the same refined shot. Two spellings of one measurement disagreeing
inside the repo. Not chased.

Run with kernels — see [[kernel-resolver]].
