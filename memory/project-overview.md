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

**Current phase (as of 2026-07-01):** §10 task 3 **DONE** — the core physics
types + element↔state map. §10 task 2 (task-0.5 de-risk spike) before it:
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

**Next concrete step = §10 task 4:** the analytic Kepler propagator behind a
`Propagator` trait (solve Kepler's equation for M↔E↔ν; this is where the
mean/eccentric-anomaly machinery deferred from task 3 lands). Then §10.5 wires
the free-invariant property tests (energy/L/LRL/reversibility) with per-propagator
expectations — analytic → machine precision. See [[git-workflow]] for the
commit/push cadence.
