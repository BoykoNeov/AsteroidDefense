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

**Current phase (as of 2026-07-01):** §10 task 5 **DONE** — the free-invariant
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

**Next concrete step = §10 task 6:** stand up the first `pyref/` fixture —
propagate a known orbit via **hapsira**, commit the reference states as **JSON**,
and add the matching Rust test in `validation/` that loads it. Pin μ/frame/time-
scale identically on both sides; pull GM through ANISE on the Rust side (§6 "the
gotcha that wastes a full day"). See [[git-workflow]] for the commit/push cadence.
