---
name: keyhole-reach
description: "Keyholes: the scoping that priced them (wide keyholes are the FAR ones, 3:4 on the shipping rock) and the BUILD that shipped on main 2026-09-02 as core/src/keyhole.rs."
metadata: 
  node_type: memory
  type: project
  originSessionId: 67baf875-a559-457f-a4d1-a077a13004ec
  modified: 2026-07-28T07:31:01.486Z
---

`core/examples/probe_keyhole_reach.rs` (2026-07-28) priced the keyhole batch
before any of it was designed, for one scenario build. It is the second half of
Tier 3's scoping — see [[tier3-uncertainty]].

**The theory is derived, not transcribed.** `tan(δ/2) = μ⊕/(b·v∞²)`, rotate `Ŝ`
toward `−B̂` by `δ`, add Earth's heliocentric velocity ⇒ closed-form `a'(b, B̂)`;
`a' = (h/k)^(2/3) AU` is the resonant locus. That *is* the analytical
resonant-return theory, reached through the deflection already modelled — so
**do not go looking for Valsecchi et al. 2003**; the derivation self-validates
and carries no paper-recall risk. It also settles a convention by derivation: at
incoming infinity `v ∝ (√(e²−1)/e)[p̂ + √(e²−1)q̂]` = exactly `geometry.rs`'s
`s_hat`, so **`Ŝ` is the incoming direction of motion.**

**The sign is MEASURED, not adopted** (`probe_keyhole_rotation`, ~200 s): fly a
21 R⊕ miss (Δv **0.205 m/s** at 12 yr lead, 190 s solve), predict post-encounter
`a` both ways against the flown truth. **`−B̂` matches to 1.518e-4, `+B̂` misses by
7.428e-2 — 489×.** So `Ŝ_out = cos δ·Ŝ − sin δ·B̂` with `geometry.rs`'s own
b_vector sign. Two traps it caught: the wrong branch predicted **0.8231 AU against
3:4's 0.8255 AU** — a flip gives a plausible near-hit on the target resonance, not
nonsense; and **`−B̂` *raises* `a'` while 3:4 sits on the *lowering* side**, so
reaching it needs a deflection of the **opposite sense** to along-track, not a
bigger one. `required_dv` controls `|b|` but not `B̂` — targeting needs both knobs.

**Four things worth not rediscovering:**

1. **The wide keyholes are the FAR ones — the intuition inverts.** Near-grazing
   resonances have the steepest `∂a'/∂b` and are the *narrowest* (19:10 at 1.0
   capture radii is **17 m** wide); far ones are wide (3:4 at 5.4–13.6 radii is
   **11.6–24.9 km**). Keyhole width *is* the accuracy encounter 1 must be placed
   to, so picking a far resonance moves the required propagation accuracy from
   **tens of metres to tens of km** — the difference between a verifiable round
   trip and an impossible one. Starting at the grazing end would have burned a
   batch discovering nothing could be verified.

2. **The verdict: the SHIPPING rock hosts a usable keyhole.** **3:4**,
   `a' = 0.825482 AU`, 4 revs in 3 yr, locus at `b` = 60 843–153 511 km (26/72
   directions cross), keyhole 11.6–24.9 km = **0.069–0.148 σ_b** (σ_b = 168.7 ×
   0.8 km). Target perigee to dial via `required_dv`: **54 385 km = 8.53 R⊕**, a
   real miss. So the *deflected* trajectory — owed anyway as the only way to show
   P *rise* — carries the keyhole, and the `[N]` purpose-built orbit and the
   Apophis/SBDB oracle both stay OUT of this batch. 7:9 is the margin option
   (48.7–131.5 km, 0.29–0.78 σ, 29.7 R⊕).

3. **`Δb = Δa'/|∂a'/∂b|` diverges at a tangency** — where the locus grazes a
   level set of `a'`, the quadratic term sets the width, not the linear one. Five
   resonances span >100× in `|∂a'/∂b|` across directions; 15:19's "1801 km"
   keyhole is that artifact. The probe prints `NEAR-TANGENCY` on them; good
   candidates sit at 1.9–2.7×. Same shape as every other "a number names its
   source or admits it is invented" rule here.

4. **Absolute `a'` is untrustworthy and the derivative is not.** `r ≈ R⊕ₒᵣᵦ` costs
   `δa/a ≈ 1.3e-4` ⇒ ~12 h of return timing ⇒ ~1.3e6 km of Earth motion ≈ **100
   capture radii**. So absolute `a'` answers only "which resonances are in band"
   (135 of them, band **0.687–1.579 AU** vs incoming 0.854). `∂a'/∂b` survives
   because the error is common-mode across neighbouring `b`. The round-trip gate
   (un-rotated `V⊕ + v∞·Ŝ` vs the real pre-encounter `a`) measured **8.669e-5**
   and the probe exits non-zero if it fails — it licenses every number below it.

**Landmine — FIXED, and the fix is a refusal.** `nominal_encounter_epoch` reduces
at the *minimum-distance* approach and `uncertainty_sampling_plan` anchored
`t_reduce` to it; extend the span past a **deeper** encounter 2 — the whole point
of a keyhole — and the anchor silently relocates, so every Jacobian column
describes a different encounter with **nothing erroring** (the matrix stays
finite, symmetric, plausible). `uncertainty_sampling_plan` now censuses via
`find_close_approaches` and anchors to the **first** encounter explicitly, and
**errors if the span holds >1**, naming the epochs it found — because *which*
encounter the covariance maps to is the caller's question and a chained Jacobian
isn't defined yet. `nominal_encounter_epoch` keeps min-distance for its ~30 other
callers. Tripwire test:
`the_tier3_reduction_epoch_anchors_to_the_first_encounter_and_refuses_a_second`
— when it fails the message is "decide which encounter," not "the plan broke."

**A worry that dissolved:** `Clock::state_at` is DOP853 **dense output** over the
integrator's own adaptive sub-steps, not linear interpolation between 10-day
snapshots (`clock.rs:208`; `dense_subsnapshot_beats_linear_interpolation` puts
the gap at >1e4×). Propagating *through* a flyby is therefore not a correctness
problem — the integrator densifies by itself. The 10-day cadence stays what
[[tier3-uncertainty]] says it is: the cadence a Jacobian's columns converged at,
owed a re-measurement across a deep flyby, not a landmine.

**BUILT AND MERGED to `main` 2026-09-02** (branch
`claude/project-structure-planning-6l2sin`, fast-forwarded, 5 commits, now
deleted). What the scoping above priced now exists as
`M:\claud_projects\AsteroidDefense\core\src\keyhole.rs` (~1170 lines, 14
tests) exporting `OpikFrame`, `Resonance`, `ResonantCircle`, `Keyhole`,
`perigee_state_for_asymptote`. Closed forms in it were re-derived by hand and
confirmed: `cos θ' = [(b²−c²)cos θ + 2cζ sin θ]/(b²+c²)`, circle centre
`cS/(C−T)` radius `c|sin θ'|/|C−T|`, `∂a/∂cos θ' = a²·2Vv∞/μ☉`, and the perigee
inversion `r_p = −c + √(c²+b²)`. **The ξ,ζ frame is now pinned** — `B` points
from Earth's centre toward the incoming asymptote, gravity bends toward `−B̂`,
with a no-tolerance-games test in `geometry.rs` that walks the inbound branch.
Headline flown result: **Δv 0.216550 m/s → return to 1 130 km from Earth's
centre on 2042-12-31T14:33 TDB** on the 3:4; grazing keyhole 0.18 km wide, far
end 24.92 km. Core suite 211 → 225 tests, and the whole workspace passes with
`ASTEROID_REQUIRE_KERNELS=1` — see [[kernel-resolver]].

The same merge added the repo's first **GitHub Actions CI**
(`.github/workflows/ci.yml`: fmt + clippy + tests, then a kernel-cached
physics job), `tools/fetch_kernels.py`, and `DEVELOPING.md`. **CI's first run
on main was green on both jobs** — so the ubuntu apt list is right, the cold
kernel fetch really works, and the Horizons tests pass when they have network
(they only "fail" locally for lack of it). Two non-blocking
things left open: the Godot `[H]` keyhole overlay has **never been run in the
engine** (Rust→GDScript dictionary keys were checked by hand and all match, but
nothing has been on screen), and `ResonantCircle::radius` is unbounded so the
off-screen cull in `godot/scripts/encounter.gd` can hand `draw_arc` an enormous
radius.

**Both loose ends above are now closed:** the `[H]` overlay ran and was seen
2026-09-05 (see [[godot-visual-layer]]), and the targeting layer that generalises
the one flown shot is [[keyhole-targeting]] — which also measured how much slack
the "order-unity" keyhole width carries (~1.6x, conservative). The unbounded
`ResonantCircle::radius` cull is still open.

Still owed: the keyhole-passage probability (the ellipse integrated over the
locus arc — a 2-D region, not a chord).
