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

**Current phase (as of 2026-07-02):** §10 task 7 **batch 2c DONE** (ASSIST
trajectory validation — see below), completing task 7's validation ladder, on top
of **batch 2b DONE** (ANISE-field adapter + Tier-1 perturber field), **batch 2a
DONE** (dop853 adaptive integrator), and **batch 1 DONE** (RK4-first slice).
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

**Task 7 — batch 2b (ANISE-field adapter + Tier-1 field) delivered.** New module
`core/src/perturber_field.rs` (advisor-scoped: the adapter **plus** the field
builder that consumes it; stop before ASSIST). **`EphemerisPerturber`** impls
`PerturberEphemeris` over a shared **`Arc<Ephemeris>`** + an `anise` `Frame`;
`position_at` = `position_km(frame, SSB_J2000, epoch.as_hifitime()) * KM_TO_M`,
mapping `EphemerisError → ForceError::Ephemeris`. New **`KM_TO_M = 1e3`** const,
deliberately **separate** from the existing `KM3_S2_TO_M3_S2 = 1e9` GM factor so a
position can't be scaled by the GM factor (a unit test pins `1e3³ == 1e9`). Kept
`point_mass.rs` **ANISE-free** (adapter in its own module) so its unit tests stay
kernel-independent. **`TIER1_PERTURBER_FRAMES`** = the 10 MVP bodies; frame
choices encode §5: Earth = **geocenter 399** + **Moon 301 separate** (never EMB
3), Mercury/Venus = body centers (199/299), Mars…Neptune = **barycenters** (NAIF
4–8, since de440s carries the giants only as barycenters + their moons lump into
the barycenter mass — ASSIST-convention TBC at 2c). **`tier1_perturber_field(&Arc)`**
pairs position-frame **and** GM lookup from the *same* `Frame` per body (makes the
μ↔position mass-mismatch bug unrepresentable), fails loud if any GM doesn't
resolve. Version check done: anise 0.10.3 → hifitime 4.3.0 (single copy in lock),
so `Epoch::as_hifitime()` feeds ANISE directly; `anise::math::Vector3` **is**
`nalgebra::Vector3<f64>` → km→m is a plain scalar mul. **Empirically verified vs
the real kernels** (`de440s.bsp` + `pck11.pca` under `temp/.../kernels`): new
`examples/probe_perturbers.rs` prints all 10 bodies resolving **both** position
and GM — Earth GM = 3.986004e5 km³/s² (Earth-only geocenter, **not** the ~4.035e5
EMB value → footgun provably avoided), giant **barycenter** GMs all resolve. Gated
test `tier1_field_builds_from_a_real_kernel` (needs `ASTEROID_DE_KERNEL` +
`ASTEROID_PLANETARY_CONSTANTS`, else skips green) drives it end-to-end: Sun-SSB in
the wobble band, 10 perturbers Sun-heaviest, and — the strong unit check — a test
particle at 1 AU feels **~5.9e-3 m/s² sunward** (exercises km→m, km³→m³, the 1/r²
sum). All 38 core tests + full workspace green; clippy clean; my files fmt-clean
and add zero new doc warnings (pre-existing lib.rs/propagator.rs warnings left
untouched, no CI). Docs refreshed: forces/mod.rs + point_mass.rs stale
"adapter later" forward-refs updated; README Status + layout markers.

**Task 7 — batch 2c (ASSIST trajectory validation) delivered.** Rung 3 of the §6
oracle ladder. `pyref/generate_assist_fixture.py` integrates asteroid (3666)
Holman with ASSIST set to **point-mass gravity only** (`ex.forces =
["SUN","PLANETS"]` → GR/harmonics/16-asteroids/non-gravs OFF), dumps SI states to
committed `validation/fixtures/assist_tier1.json`; `validation/tests/
assist_reference.rs` propagates the same IC with dop853 (rtol/atol **1e-12**) in
the matching field and compares. **Ran end-to-end (Docker oracle + gated Rust
test, all kernels cached locally):** worst **pos_rel 4.5e-11 / vel_rel 3.3e-11**
over 730 d, residual **growing monotonically with arc** = the GM-delta secular
drift the advisor predicted (not a structural bug). Tolerances set to 2e-10 (~4×
observed). Gated on `ASTEROID_DE_KERNEL`+`ASTEROID_PLANETARY_CONSTANTS`, skips
green offline.

**Force-model match nailed (the 2c crux, advisor-steered):** ASSIST's
point-mass term sums **11 bodies incl. PLUTO** (verified in `src/forces.c`
`order[]`), so the shipping 10-body `tier1_perturber_field` does NOT match ASSIST
exactly. The test builds an **11-body comparison field** (Tier-1 + Pluto NAIF 9)
so both sides integrate ASSIST's identical system — absorbing Pluto into tolerance
would hide the μ-slip/rotation bugs the test exists to catch. **Two real findings:**
(1) **pck11.pca carries NO Pluto GM (BODY9_GM absent)** — the comparison uses the
oracle's own DE440 Pluto GM (975.5 km³/s²); position resolves fine from de440s.bsp.
(2) **pck11 ≠ DE440-header GMs** — worst **Mercury 4.0e-6**, Uranus 1.3e-6, then
1e-8–1e-9 (Sun 5e-12); measured per-body by `anise_gm_matches_de440` (from
`gm_de440.tpc`), the residual's real floor. Dynamically Jupiter/Uranus dominate.

**OPEN DECISION surfaced to user (away → proceeded with recommended default,
re-askable):** §5 locks shipping set at 10; ASSIST (our §6 config) has 11.
**Measured Pluto-omission cost = ~55 m over 2 yr** for Holman (grows with lead
time, `pluto_omission_effect_over_arc` test). **Chose Option A: keep 10-body
shipping field, defer Pluto to Tier 2** (with the 16 asteroid perturbers + a
DE441-consistent GM source, since pck11 lacks Pluto GM). **Did NOT edit §5's
locked decision** — that's the user's call. Documented the quantified caveat in
`perturber_field.rs`. If user later wants Pluto in shipping (Option B): add
PLUTO_BARYCENTER_J2000 to `TIER1_PERTURBER_FRAMES`, resolve the pck11 Pluto-GM
gap, update the two `.len()==10` unit tests + §5.

**Infra note:** validation gained an `anise` dev-dep (default-features off, matches
core) for the Pluto Frame constant. Docker path-conv gotcha: Git Bash mangled
`-e VAR=/data`; fix = `MSYS_NO_PATHCONV=1` prefix.

**Task 8 (§10.8) — b-plane hit test DONE.** New `core/src/geometry.rs`
(`BPlaneEncounter`, `GeometryError`, `EARTH_{EQUATORIAL,MEAN}_RADIUS_M`, all
re-exported from lib). Pure, kernel-free, frame-agnostic: `from_relative_state(
r_rel, v_rel, μ⊕, R⊕)` reduces an **Earth-geocentre-relative** encounter state to
the osculating two-body-about-Earth hyperbola — `v_inf=√(2ε)`, impact parameter
`b=h/v_inf`, perigee `p/(1+e)`, eccentricity, incoming-asymptote dir
`Ŝ=(P̂+√(e²−1)Q̂)/e`, and b-vector `B=b(Ŝ×ĥ)`. **Hit test** = gravitationally-focused
capture radius `b_capture=R⊕√(1+(v_esc/v_inf)²)`, `is_hit ⇔ b≤b_capture`
(equivalent to `r_perigee≤R⊕`, tested). `focusing_factor`/`escape_speed` helpers.
Rejects bound/parabolic (`NotHyperbolic`), radial `r∥v` / zero-r (`Degenerate`),
bad μ/R⊕ (`NonPositiveParameter`). **12 in-module tests** (advisor-steered): the
discriminating one is the **perigee round-trip** (known v_inf,r_p → recovered
geometry), plus **sampling-point invariance** (same hyperbola at an off-perigee
ν=−0.7 rad → identical v_inf/b/perigee/e **and Ŝ** — this is the ONLY test that
exercises the `−v_rel·(r·v)` eccentricity-vector branch and validates Ŝ's
*direction*; all perigee tests have r·v=0 and |B|=b holds by construction
regardless of Ŝ, so without this Ŝ was unvalidated), hit⇔perigee equivalence
sweep, grazing b=b_capture, μ→0 straight-line limit, v_inf=v_esc→factor √2, NEO
focusing band 1.2–2.4. **Deliberate scope cuts** (advisor-confirmed): (a) does
NOT search for closest approach — that needs dense trajectory sampling = the
clock's job (§10.9); (b) b-vector **sign convention + Öpik/Kizner ξ,ζ
decomposition deferred to Tier 3** `uncertainty.rs` (needs an external ref dir;
keyholes are what reason in b-plane coords) — new HANDOFF open-questions entry.
**Step-9 prerequisite flagged in the module doc:** forming `v_rel` needs Earth's
*velocity*; `Ephemeris` exposes only `position_km` today but ANISE's `translate`
already returns `velocity_km_s` (discarded) — small add for the clock.

**Task 9 (§10.9) — dop853 dense output + fixed-cadence clock DONE.** Two pieces,
both advisor-steered. **(1) Dense output** in `core/src/integrator.rs`: added the
dense-output tables (`C_EXTRA`, `A_EXTRA` 3 rows, `D` 4×16) transcribed from the
same scipy v1.17.1 `dop853_coefficients.py` as the step tableau; refactored the
adaptive accept/reject loop into a shared `integrate(...)` taking an `on_accept`
callback so **plain `step` (no-op) and new `step_dense` (records segments) share
one loop** — can't drift on accept/reject or endpoint-clamping. `attempt_step` now
returns its 12 stage arrays (aliased `StageDerivs`). `step_dense` → `(final_state,
Vec<DenseSegment>)`; each **`DenseSegment`** (pub, re-exported) holds `t0,h,y0` +
7 interpolation coeffs `(fr,fv)` and evals the 7th-order continuous extension via
SciPy's reversed-Horner (alternating `·x` / `·(1−x)`). Costs **3 extra force evals
per accepted step** (the 3 extra stages K[13-15]; K[12] is the FSAL already
computed) — paid only on the dense path. **(2) `core/src/clock.rs`**: `Clock`
(+ `ClockError`) — `Clock::propagate(&Dop853, force, epoch0, state0, cadence, N)`
drives `step_dense` **cadence-by-cadence** (= §2 "fixed snapshot cadence, adaptive
step between"), storing **exact integrated snapshots** at each boundary + all dense
segments (sorted by `lo` for binary search). `snapshot(k)` = exact indexed state;
`state_at(epoch)` = **sub-snapshot query from dense output, NOT linear interp**
(fails loud `OutOfRange` outside span, never extrapolates). Signed cadence → a
**backward clock** for the rewind view. `segments()` exposes the continuous
`position(t)` the future close-approach detector root-finds on.

**The test crux the advisor flagged:** the `D` matrix is **invisible** to both the
tableau-consistency identities (they don't involve D) **and** endpoint continuity
(F[3..6] are zeroed by `·x`/`·(1−x)` at x∈{0,1}, so both step endpoints match
regardless of D). D is only exercised at **interior** points →
`dense_output_reproduces_polynomial_interior_pins_d` evals a degree-≤7 polynomial
trajectory (`a=c·tᵖ`, p=1,3) at interior x and compares to closed form (**relative**
1e-8, since the abs error floor is hifitime ns epoch-quantization scaling with tᵖ).
**Mutation-verified**: a realistic 1-digit typo in one D entry fails ONLY the
interior test while endpoints + reintegration pass — proving the interior test is
the actual D-pin. Other tests: endpoint-exactness (+ `step_dense`==`step`
bit-identical final state), non-poly reintegration cross-check (~tol not ε, §2
determinism), backward-span endpoints; clock: snapshots==direct stepping,
`state_at` at boundaries==snapshots, **dense ≫ linear-interp** on a curved 30°-arc
(≥1e4× tighter — the pedagogical thesis), out-of-range fails loud, backward clock.
58 core tests green, workspace green, clippy clean, my files fmt-clean.
**Earth-velocity surfacing NOT done** (deferred with the close-approach detector).

**Close-approach detector + Earth-velocity glue DONE** (§10.9 follow-on, the true
next increment after the clock; advisor-steered). **Two pieces.** (1) **Earth
velocity surfaced** in `ephemeris.rs`: new `state_km_s(target,observer,epoch) ->
(radius_km, velocity_km_s)` reads the `velocity_km_s` ANISE's `translate` already
returns (was discarded — field name confirmed against anise 0.10.3 source);
`position_km` now delegates to it; added `geocenter_state_ssb_km`. In
`perturber_field.rs`, `EphemerisPerturber::state_at(epoch) -> StateVector` (SI,
km→m + km/s→m/s) + an impl of the new `GeocentricState` trait, so an
`EphemerisPerturber::new(eph, EARTH_J2000)` **is** the detector's Earth source.
(2) **`core/src/close_approach.rs`** (new, ANISE-free, re-exported from lib):
root-finds the **range-rate** `f(t)=r_rel·v_rel = d/dt(½|r_rel|²)` on the clock's
dense output (asteroid) differenced against the `GeocentricState` provider (Earth).
A **`−→+` crossing brackets a range minimum** (CA); `+→−` (a max) is ignored.
Scan grid = the integrator's own sub-step boundaries (`clock.segments()`)
subdivided to `ScanOptions::max_sample_dt`; each bracket **bisected** to the CA
epoch. Returns `Vec<CloseApproach>{epoch, asteroid_ssb, earth_ssb, relative,
distance}` in epoch order; `closest_approach(...)` = the min-distance one.
**`CloseApproach::b_plane(μ,R⊕)`** feeds `geometry.rs` — the encounter pipeline is
now closed (clock → detect → relative state → b-plane hit/miss). `GeocentricState`
is blanket-impl'd for closures (kernel-free tests use a synthetic Earth-at-origin).

**Two advisor points landed on the record:** (a) **`max_sample_dt` is the one
correctness-critical knob**, made a **required physical cap** (default 6 h, NOT
`None`): DOP853 steps are Sun-error-sized so segments do NOT shrink at an Earth
approach until deep in the well, so a too-coarse grid **silently aliases away a
fast pass** (a missing entry, not an error) — documented; 6 h is marginal for a
50–70 km/s retrograde impactor, tighten then. `ScanOptions::max_distance` filters
the AU-scale synodic minima a multi-year arc produces. (b) **b-plane sampled AT
CA deliberately** — `geometry.rs:162` cautions "sample near-but-not-at CA" for
`v_inf=√(v²−2μ/r)` cancellation, but that **does not bite for Earth in f64** (even
near-parabolic keeps ~12 digits; it's *slow* passes that are worst, not fast),
and CA is where Earth most dominates → cleanest hyperbola. Reconciled in the
`b_plane` doc so the two modules aren't quietly at odds. Dense-velocity ≠ exact
d/dt(dense-pos) noted as harmless (interpolation-order; b-plane invariants are
sampling-invariant). **Tests (kernel-free, 7 new):** straight-line pass recovers
exact CA epoch + miss `b`; **moving-Earth Galilean-boost test** (Earth drifts ⟂ to
the closing velocity → same CA as rest frame ONLY if `v_earth` is subtracted right;
**mutation-verified** — flipping the subtraction's sign fails it — this is the only
test that exercises the Earth-*velocity* differencing, the named half of the
increment; advisor caught that all others used a zero-velocity Earth); receding
motion → no CA (the `+→−`-ignored check); `max_distance` filter drops a distant
pass → `closest_approach` None; **end-to-end two-body Earth hyperbola through the
clock → detector → `b_plane` recovers seeded `v_inf`/perigee** (the loop-closing
test); invalid-options rejection. Kernel-gated
`geocenter_velocity_is_earth_orbital_speed` (~29.8 km/s band) pins the velocity
surfacing against a real kernel (skips green offline — no local kernel this
session). 64 core lib tests green, full workspace green, clippy clean.

**"Surface Earth velocity" = its orbital velocity for `v_rel`** (per the geometry
step-9 note), NOT surface-rotation velocity / impact footprint — that's out of
scope for this increment (advisor confirmed the reading).

**Deflection core (task-10 Commit A) DONE** — advisor-steered core-first split of
the viewer task: build the physics the headline curve needs *before* taxing the
build with the egui/wgpu stack. New `core/src/deflection.rs` (re-exported): the
mission-planner primitive **`apply_impulse(state, Δv)`** (adds to velocity,
§4's entire physics coupling), `along_track_unit` (v̂ heading), thin
**`kinetic_impactor_dv(β,m_imp,v_rel,M_ast)`** (`|Δv|=β·(m/M)·v_rel`, DART β≈3.6;
nuclear/gravity-tractor spectrum deferred). The substance = **`DeflectionScenario`**:
propagates the nominal once, then `evaluate(epoch,Δv)` re-propagates from the
deflection epoch (samples nominal `state_at`, applies impulse, fresh `Clock` →
`closest_approach` → `b_plane`) and **`required_dv`** solves the headline curve —
geometric bracket-expand on |Δv| then bisect until the gravitationally-focused
b-plane **perigee** (NOT raw CA distance — that's the §10.8 point) clears a safe
target. `required_dv_along_track` fixes the §5/§7 fixed-phase along-track direction.

**Two conditioning facts nailed (the hard part):** (1) with a **massless** test-
Earth (Sun-only field, the codebase's convention), sampling a *hit* at its own CA
is `NotHyperbolic` unless v_rel > Earth escape speed — so the thesis test uses a
*fast* (~30 km/s, fixed-Earth gives it free) encounter + small perpendicular miss.
(2) `perigee_after` maps **`NotHyperbolic → 0`** (a dead-centre near-collision =
worst hit): a Δv sweep passes *through* a near-collision on its way to opening a
miss, so the solver must read that as "still a hit," not fail. **7 kernel-free
tests green** (72 core total): impulse/β primitives; straight-line cross-track
machinery (solver hits target perigee to 0.5%, monotone, 0-when-already-clear);
and the **thesis** — earlier along-track deflection needs less Δv (`dv_early <
0.75·dv_late`, leads 0.7 vs 0.1 period). Clippy clean.

**Carry to Commit B (advisor):** (a) the thesis test is **sub-orbital** — it pins
the leverage *direction*, NOT the multi-orbit `Δv∝1/lead` accumulation (§144);
that falloff must be visibly steeper-than-linear on the **real-field** curve or
it's a bug. (b) `NotHyperbolic→0` is a massless-Earth artifact — with real Earth
mass a near-centre pass is a genuine deep hyperbola, so that branch rarely fires
and real focusing/path-bending is still untested. (c) perigee(Δv) is non-monotone
near nominal (a dip), so the bisect can return a **conservative (non-minimal)** Δv
— safe (never under-reports) but watch **curve monotonicity/smoothness** in B.

**Kernel located (unblocks Commit B):** `de440s.bsp` + `pck11.pca` (+ `sb441-n16.bsp`
Tier-2, `linux_p1550p2650.440` ASSIST-only) live under
`M:\claud_projects\temp\AsteroidDefense\kernels`, git-ignored, downloaded by
`pyref`. ANISE reads the `.bsp`+`.pca`; wire via `ASTEROID_DE_KERNEL` /
`ASTEROID_PLANETARY_CONSTANTS`.

**Task 10 Commit B (§10.10) — egui viewer DONE (physics tested, GUI UNRUN).**
Commit `ef82cfa`. Split **B1** (headless curve gate) → **B2** (egui app); egui-free
physics in `viewer/src/scenario.rs`. **Core add** (`deflection.rs`):
`DeflectionScenario::deflected_trajectory(epoch,Δv) -> (Clock, Option<BPlaneEncounter>)`
returns the post-deflection track *and* its encounter; `evaluate` now delegates to
it (one propagation path). Equivalence test: the returned clock re-scanned == the
reported perigee, so the drawn track and the headline number can't diverge.
**`scenario.rs`**: `RealFieldScenario` back-propagates a designer DE440
Earth-impactor (default 12-yr campaign, 18 km/s v_rel → **a=0.855 AU, T=0.79 yr,
15.19 orbits of lead**); `CurvePoint`(serde)+`CurveFile`+`DEFAULT_CURVE_JSON`;
`sweep` = the headline curve; `EncounterFrame`+`nominal_hit`+`frame_from` sample
both asteroid tracks **geocentric** (Earth-at-origin) over ±1.5 d centred on impact,
reusing a prebuilt `DeflectionScenario`+nominal encounter so each nudge pays only
the short post-deflection arc — the full 12-yr nominal scan is **hoisted out**
(measured **401 s in DEBUG** when recomputed per call; release `frame_from` is
sub-second-to-few-s, cost ∝ arc length). Shared consts
`ENCOUNTER_HALF_WINDOW_SECONDS`/`ENCOUNTER_SAMPLES` used by app **and** test.
**B1 GATE PASS**: real-field sweep log-log slope **−1.049 (all) / −0.912 (tail ≥1
orbit)** → the multi-orbit `Δv∝1/lead` falloff is **REAL** (carry-item (a)
discharged). The `curve` binary writes `curve.json` (**gitignored runtime
artifact**; a fixed property of the design, computed once ~480–680 s, app loads
instantly — no live recompute, per advisor). **egui app** (`main.rs`): worker-thread
build (~12 s) + per-nudge `frame_from` on the worker, **coalesced single-in-flight**
requests, `ctx.request_repaint` from the worker; left panel = Δv log-log curve from
`curve.json` + current-lead marker; central = encounter-frame painter (Earth disc +
focused capture radius, **nominal-red spears / deflected-green slides past**); **play
sweeps only the in-view band** (~2% of the window the asteroid is on-screen —
advisor caught that a full-window sweep leaves both dots clipped ~5.9/6 s). Displayed
miss = `deflected_perigee` (exact from the b-plane, not pixel geometry). **eframe
0.35 API churn**: `App::ui(&mut Ui)` (not `update(ctx)`); panels unified to
`egui::containers::Panel::{top,left}` (SidePanel/TopBottomPanel gone, `default_size`
not `default_width`); **`egui_plot` 0.36 pairs with egui 0.35** (0.35 wanted egui
0.34 — a two-egui mismatch). `probe_prop` retargeted to time `frame_from` (the real
per-nudge cost) in release.

**HONEST STATUS: the GUI has NOT been run** — no display in the build env, so
painter / projection basis / world→screen scaling / panel layout / play animation
have **zero runtime validation**. Only the physics is tested (73 core + a
kernel-gated viewer test that pins the drawn track brackets the reported perigee;
clippy clean). Validate with a manual `cargo run -p viewer --release`
(`ASTEROID_DE_KERNEL`/`ASTEROID_PLANETARY_CONSTANTS` set) — the **play animation
especially**; advisor flagged it will likely need a tweak once seen.

**Carry forward:** (b) real-Earth-mass focusing / path-bending still **untested**
(the massless-Earth `NotHyperbolic→hit` branch rarely fires with real mass); (c) the
**1.0-orbit** curve point (Δv 0.428→0.510) is expected **phasing non-monotonicity**
(impulse phase vs encounter geometry), absorbed by the LSQ fit — not a bug, but
watch curve smoothness. **Next:** run the GUI to validate B2 visuals; then remaining
task-10 polish or the next HANDOFF task. See [[git-workflow]] for commit/push cadence.
