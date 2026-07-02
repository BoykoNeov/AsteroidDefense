---
name: project-overview
description: "What the AsteroidDefense project is, where the spec lives, and current phase"
metadata: 
  node_type: memory
  type: project
  originSessionId: f5cc34dd-dad9-418c-9791-57031e47c59c
---

**Asteroid Deflection Simulator** — an educational planetary-defense sim whose
single thesis is: *deflecting an asteroid early is dramatically more effective
than late*; the headline screen is a Δv-vs-lead-time curve. Realism is a primary
stated goal. Headless deterministic **Rust core** = single source of truth;
**egui** pure-Rust viewer for the MVP; **Godot** (gdext) in Phase 2.

The full **locked spec** lives in `HANDOFF.md` (architecture, tiered force model,
validation oracle ladder, hard problems, task-by-task plan). `README.md` is the
public summary. Don't re-litigate decisions marked locked there.

**License:** Boyko Non-Commercial License v1.0 (BNCL-1.0) — proprietary,
non-commercial use permitted, commercial use requires separate written
permission. Relicensed from Apache-2.0 on 2026-07-01 (commit bab4853); it is no
longer OSI open source. **Public GitHub repo:** owner `BoykoNeov` (created
2026-06-23). Only hifitime/ANISE (MPL) + nalgebra (Apache/MIT) link into the
shipped binary; consuming those permissive/MPL crates stays compatible under
BNCL. GPL/AGPL oracles (REBOUND, ASSIST, GRSS, nyx) stay offline in
`pyref/` — never in any Cargo.toml.

**Current phase (as of 2026-07-02):** §10 task 7 **batch 2a DONE** (dop853
adaptive integrator — see below) on top of **batch 1 DONE** (RK4-first slice).
§10 task 6 **DONE** before it: the hapsira two-body JSON
reference fixture + rung-2 oracle test (`validation/tests/kepler_reference.rs`,
`validation/fixtures/kepler_two_body.json`, `pyref/generate_kepler_fixture.py`).
§10 task 5 before that: the free-invariant
`proptest` harness (see below). §10 task 4 before it: the analytic Kepler
propagator behind the `Propagator` trait. §10 task 3 before that:
the core physics types + element↔state map. §10 task 2 (task-0.5 de-risk spike):
**both pillars PASS, Option A confirmed, fallback trigger NOT fired.** Task 1:
Cargo workspace scaffolded — `core/` lib = `asteroid_core`, renamed to dodge the
std `core` shadow; `viewer/` egui bin; `validation/` lib; `pyref/` non-member
Python dir; core dep tree still zero egui/eframe/wgpu.

**Task 3 (§10.3) delivered** in `core/src/`: `epoch.rs` (`Epoch` newtype over
`hifitime::Epoch`, pins dynamics to **TDB**, seconds-past-J2000 handle for the
integrator), `state.rs` (`StateVector` = position+velocity `nalgebra::Vector3`,
**SI m / m·s⁻¹**, barycentric ICRF), `elements.rs` (`OrbitalElements` classical
Keplerian, **elliptical only** 0≤e<1; `to_state(μ)`/`from_state(μ)` pure geometry,
no Kepler solve — that's task 4; `ElementsError::{NonElliptical,Degenerate}`).
**Units decision:** core canonical = **SI (m, m/s)**; the km→m conversion lives
at the ANISE loader boundary (confirmed `anise::math::Vector3` *is*
`nalgebra::Vector3<f64>` — anise 0.10.3 pulls the same nalgebra 0.35, so the
boundary is a clean scalar multiply). **Singularity conventions in `from_state`**
(ω undefined at e→0, Ω at i→0/π): circular→ω=0, ν=arg-of-latitude;
equatorial→Ω=0, ω=longitude-of-periapsis (sign-flipped when retrograde h_z<0);
both→ν=true-longitude. **Tested** via `proptest` 1.11.0 (dev-dep, 2048 cases) in
`core/tests/element_state_roundtrip.rs`: the property is a **STATE** round-trip
(`S1=to_state(E); S2=to_state(from_state(S1)); S1≈S2`, **relative** tol 1e-7) —
NOT an element round-trip, because ω/Ω are gauge (undefined) exactly at the
seeded singularities; a/e/i (gauge-free) are checked directly. Strategies
**union explicit degenerate literals** (e∈{0,1e-15,…}, i∈{0,π,…}) with random
ranges + deterministic `#[test]`s for the combined corners (a random draw won't
hit e=0∧i=0 simultaneously). Bug found+fixed during dev: retrograde-equatorial
(i=π) needed the in-plane angle sign flipped; seed saved in
`*.proptest-regressions` (committed). Advisor steered the state-vs-element
framing + relative-tol + i→π seeding.

Pillar B (pure Rust, the shipped path): `core/src/ephemeris.rs` is now a **real
ANISE loader** (`Ephemeris` over `Almanac`, SSB-relative km positions), no longer
a stub. Proven via `cargo run -p asteroid_core --example spike_geocenter` +
gated test `geocenter_is_reconstructed_not_emb` (runs iff `ASTEROID_DE_KERNEL`
is set, else skips green). At 4 epochs the reconstructed **geocenter (399) ≠ EMB
(3)** by 4351–4908 km (tracks the real Earth–Moon distance; offset =
d/(EMRAT+1)), and an EMB independently rebuilt from Earth+Moon via EMRAT matches
ANISE's EMB to **0.000000 km** — proof it's the true geocenter, not a relabelled
EMB (the §5 ~4671 km footgun is provably avoided). `anise` trimmed to
`default-features = false` → drops `ureq`/network (the §10 offline invariant),
tree verified clean. Deps: anise 0.10.3, hifitime 4.3.0, nalgebra 0.35.0.

Pillar A (offline GPL oracle, `pyref/`): ASSIST 1.2.3 + rebound 4.6.0 build and
integrate a test particle in the DE field, round-trip reversible to 4.7e-4 m.
**Operational facts discovered:** (1) `assist` ships **no wheel** — compiles
from source, needs `gcc` + REBOUND headers; native Windows can't build it and
this box's WSL Ubuntu lacks pip/venv + passwordless sudo, so the oracle host is
**Docker `python:3.12-slim` + gcc** (see `pyref/Dockerfile`). (2) ASSIST's
shipped ephemeris = **DE440 planetary (`linux_p1550p2650.440`) + DE441-derived
`sb441-n16.bsp`** (the full DE441 planetary is ~2.6 GB, identical reader; task 1
allowed "DE440 or DE441"). Results + the written fallback-to-Option-B trigger
live in `pyref/SPIKE.md`. Kernels/data (~750 MB) live under
`M:\claud_projects\temp\AsteroidDefense\kernels`, git-ignored.

**Task 4 (§10.4) delivered** in `core/src/propagator.rs`: the `Propagator` trait
(`fn state_at(&self, Epoch) -> Result<StateVector, PropagatorError>`) — kept
**object-safe** on purpose (shared concrete `PropagatorError` enum, no generics /
no `Self` in return, `&self` only) so context planets (Kepler) + the future
integrated asteroid can live behind one `dyn Propagator`. `KeplerPropagator::new`
is the **validating boundary** the elements module isn't — it rejects a≤0 / μ≤0 /
e∉[0,1) (NaN-safe, fails closed) as `PropagatorError::InvalidOrbit`, since
`OrbitalElements::new` only wraps/clamps angles. The mean/eccentric-anomaly
machinery deferred from task 3 lands here as pub free fns: `eccentric_from_true`
(atan2 form), `mean_from_eccentric` (M=E−e·sinE), `true_from_eccentric`,
`solve_kepler` (Newton from E₀=M+e·sinM seed, M wrapped to [−π,π), residual tol
1e-13, 100-iter cap → `NonConvergence`). Propagation = advance M linearly
(n=√(μ/a³)), one Kepler solve per query; a,e,i,Ω,ω carried through unchanged.
**Frame caveat documented:** Kepler output is relative to the attractor of the
passed μ, NOT barycentric ICRF (§5) — Tier-0 cosmetic orbits only, never a
hit/miss decision. Tests (10 new, in-module): the discriminating anchors are
known-answer, NOT self-referential in μ — independently-computed period return
(T=2π√(a³/μ)), circular T/4 & T/2 geometry, eccentric periapsis→apoapsis, Δt=0
identity, forward-back reversibility, anomaly ν→E→M→E→ν round-trip + Kepler
residual; conservation/element-invariance included but flagged weak (by
construction per §6). Advisor confirmed the trait shape + steered toward the
non-self-referential anchors. **NOT built** (deliberate YAGNI): any numerical /
dense-output propagator machinery — the trait just doesn't preclude it.

**Task 5 (§10.5) delivered** in `validation/tests/free_invariants.rs` (the free
invariants live in the **`validation` crate**, per its own doc — rung 1 of the §6
oracle ladder; nalgebra added as a validation dev-dep). Four invariants, all
computed **from the propagated Cartesian state, never from elements** (the
non-vacuity crux — `elements_at` only advances ν, so energy-from-`a` would be a
constant *read*): specific energy `½v²−μ/r`, angular momentum `r×v`, the
eccentricity vector `(v×h−μr̂)/μ` (LRL/μ), and forward-back reversibility.
**Per-propagator expectations** via an `InvariantTolerances` struct — analytic
Kepler → machine-precision-class: conservation rel **1e-11** (energy + |h|),
eccentricity-vector **absolute** 1e-10 (relative is 0/0 at e→0), reversibility
rel **1e-7** (routes through `from_state`'s gauge fold, matching the roundtrip
test). Harness anchors t₀ to closed forms (ε=−μ/2a, |h|=√(μp), |e⃗|=e) so it's
not self-referential in μ. Reversibility **reseeds via `from_state`** (re-calling
`state_at` is a vacuous identity for an analytic map). Same unioned
singular-literal + random `proptest` strategy as the roundtrip test (512 cases) +
7 deterministic corners. **Non-vacuity proven**: corrupting `to_state`'s velocity
coeff (`e+cosν`→`cosν`) fails 5/8 tests via the energy anchor (the 3 survivors are
near-circular, where the dropped term is negligible — confirms energy is the
workhorse). **Don't over-read green** documented in the module: only the analytic
Kepler map exists, which conserves by construction, so green validates the
**conversions**, not any integrator. RK4/DoPri (→ error-growth *rate*, a
different assertion shape) and symplectic (→ bounded oscillation) deliberately
NOT built — no such propagator exists yet. Advisor steered non-vacuity, the
e→0 absolute-tolerance for LRL, the split conservation/reversibility tolerances,
and keeping the primitives as test helpers (not core API).

**Task 7 (§10.7) — batch 1 (RK4-first) delivered.** Task 7 is the biggest task
and is being done in batches; batch 1 stands up the composable force model +
swappable integrator + RK4, deferring **dop853, the ANISE-field Tier-1 model,
and ASSIST validation to later batches** (advisor-confirmed scoping). New in
`core/src/`: **`forces/mod.rs`** — `ForceModel` trait (`acceleration(&self,
Epoch, &StateVector) -> Result<Vector3, ForceError>`; fallible for ANISE later,
takes full state for 1PN/SRP/Yarkovsky later, returns *acceleration* since the
test-particle mass cancels) + `CompositeForce` (Σ of `Box<dyn ForceModel>`
terms; tiers = which terms are enabled, toggle = Vec membership; empty = free
particle; short-circuits fail-loud on first term error). `ForceError::{Singularity,
Ephemeris}`. **`forces/point_mass.rs`** — `PointMassGravity` over an arbitrary
`Vec<Perturber>` (`Σ μ_j (r_j−r)/|r_j−r|³`); perturber positions come through a
**dedicated frame-explicit `PerturberEphemeris` trait** (barycentric-ICRF SI),
**NOT** `Propagator` (whose contract is attractor-relative — conflating them is
the §5 frame footgun; advisor adjustment). `FixedPerturber` (constant, for tests
+ a fixed attractor) now; ANISE `Ephemeris` adapter later. `(μ, eph).into()`
ergonomic ctor. `|r_j−r|==0`/non-finite → fail-loud `Singularity` (real close
approaches stay finite by design). **`integrator.rs`** — object-safe `Integrator`
trait (`step(force, epoch, state, dt)`, dt may be negative) + classical fixed-step
**`Rk4`** (four stages at t, t+h/2, t+h/2, t+h — epoch threading is load-bearing
for moving perturbers) + `propagate_fixed` helper. `IntegratorError::Force`
wraps `ForceError`.

**The crux (advisor-steered): RK4 "exercises the invariant tests" via a NEW
assertion shape, NOT a loosened `assert_conserves`.** `free_invariants.rs`'s
1e-11 bounded-conservation is the analytic-map half; RK4 correctly *fails* that,
so instead `validation/tests/integrator_convergence.rs` realizes the numerical
half: (1) **fourth-order convergence** — self-calibrating, integrate a fixed arc
at N vs 2N steps, error vs the analytic `KeplerPropagator` truth drops ~16×
(order = log2(e_N/e_2N) ∈ [3.7,4.3]); no magic tol, the *ratio* is the
assertion; (2) **epoch-threading probe** — a two-body field is *autonomous* so it
can't catch a "all stages at t" bug; a non-autonomous sinusoidal forcing (closed
form) pins it via 4th-order convergence; (3) **honest drift** — RK4 energy drifts
non-vacuously (>1e-10 at coarse step) yet shrinks >8× under step halving (proves
it's a genuine integrator, not secretly conservative). **Oracle validity:** the
analytic Kepler truth is valid *only because the attractor sits at the frame
origin* (attractor-relative ≡ barycentric there). Core in-module tests also pin
RK4: constant-accel exactness + a linear-in-t exactness (cheap epoch-threading
pin, RK4 exact for ≤cubic-in-t). Updated `free_invariants.rs` module doc to
cross-reference the new file (the two are two halves of one seam — advisor caught
the now-stale "not built yet" comment). **Conscious deferral:** RK4's
*velocity-dependent* force path is unexercised (all test forces are position- or
time-only); pin it with a linear-drag closed form when the first velocity-coupled
term (1PN/SRP) lands.

**Task 7 — batch 2a (dop853) delivered.** Advisor-confirmed splitting batch 2
into **2a dop853 / 2b ANISE-field adapter / 2c ASSIST validation** (2c depends on
both, is where the force-model match subtlety lives, so it's last). New in
`core/src/integrator.rs`: **`Dop853`** — Dormand-Prince 8(5,3), the MVP encounter
integrator. Honours the **unchanged** object-safe `Integrator` trait by
sub-stepping **adaptively inside** the requested `dt` (= the "fixed snapshot
cadence, adaptive step between" architecture, §2); `step` is **pure/`&self`** (no
cross-call state, each call re-estimates its own initial step via Hairer's
algorithm → deterministic). **Coefficients transcribed from scipy's
`_ivp/dop853_coefficients.py` (v1.17.1)** into a private `dop853_tableau` mod
(only the 12 step stages; the 4 dense-output stages + `D` matrix deferred to
§10.9); **Hairer's combined 5(3) error norm** (`|h|·err5²/√((err5²+0.01·err3²)·n)`,
faithful to scipy's squared-numerator form — advisor verified). FSAL: 12 force
evals/accepted step (recompute derivative at the new point = next step's k0),
skipped on rejection. Config fields rtol/atol (default 1e-9), optional max_step,
max_substeps backstop (default 1e6) → new `IntegratorError::{StepSizeUnderflow,
MaxStepsExceeded}` (fail-loud, never spin). Backward `dt<0` + exact-endpoint
landing handled. **Verification pivoted off RK4's convergence-order test** (8th
order floors at round-off before an h⁸ slope is readable): `validation/tests/
dop853_adaptive.rs` = (1) **Kepler-oracle match** over 3.3 periods @ rtol 1e-12,
worst rel err **1.5e-11**; (2) **controller-contract** rtol sweep {1e-6,1e-9,1e-12}
→ achieved err ≈6-16×rtol, monotone in both error and force-eval count
(217→395→780); (3) **max_step cap** forces more work (395→2474 evals), stays
accurate. Core in-module tests: tableau consistency (ΣA row=C, ΣB=1, ΣE=0 —
guards transcription), poly-exactness (const + linear-in-t, the latter's velocity
bound relaxed to 1e-6 for **hifitime's ns epoch quantization** — a stage at an
irrational `C[s]·h` snaps to nearest ns ≈0.5ns, and an absolute-time-reading field
turns that into ~1e-8 vel error; NOT an integrator bug, and the epoch threading
is still pinned since a real bug gives O(tens)), reversibility, dt=0 identity,
max-substeps fail-loud, object-safety. **No CI exists** (checked `.github/`), so
the pre-existing rustfmt dirtiness in `probe_sun_gm.rs`/`kepler_reference.rs`
(older rustfmt) is un-gated; left untouched to avoid churn — new code fmt-clean.

**Next concrete step = §10 task 7 batch 2b:** the **ANISE-backed
`PerturberEphemeris` adapter** (share one `Almanac` via `Arc`; observer = **SSB**
not Sun; km→m + km³/s²→m³/s² at the boundary; μ **pulled through ANISE**, never
hardcoded) + the Tier-1 field (Sun + 8 planets + Moon). Then **batch 2c**: ASSIST
validation (`pyref/`) — ALL of GR / 16 asteroids / non-gravs **off** on ASSIST's
side to match Tier-1 Newtonian point-mass exactly; default rtol 1e-9 may need
tightening to hit ASSIST's meter bar (**re-consult advisor before 2c**). See
[[git-workflow]] for the commit/push cadence.
