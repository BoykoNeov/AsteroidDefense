# Asteroid Deflection Simulator

An educational solar-system simulator for **planetary defense** — planning and
simulating missions to deflect an Earth-bound asteroid, built around a single
thesis it exists to make you *feel*:

> **Deflecting an asteroid early — many orbits before the predicted impact — is
> dramatically more effective than deflecting it on final approach.** A tiny nudge
> applied years out beats a massive shove applied days out.

The money screen is a plot of **required Δv vs. lead time**: that curve *is* the
thesis. You should be able to attempt a last-minute deflection, watch it fail,
rewind ten years, tap the asteroid once with a small impulse, and watch Earth
slide safely out of the way.

This is built for **realism**, not a cartoon. The dynamics that decide hit-vs-miss
are modeled at ephemeris quality and validated against the same reference tools
professional planetary-defense work uses.

---

## Status

**Early — foundations landing.** The full architecture, physics, validation
strategy, and task sequence are locked in [`HANDOFF.md`](HANDOFF.md) (the
authoritative spec). The de-risk spike passed (ANISE ephemeris + oracle toolchain
build and the DE440 geocenter reconstructs correctly), and the Rust workspace is
up: epoch/state/orbital-elements with the element↔state map, an analytic Kepler
propagator behind the `Propagator` trait, and a free-invariant proptest harness
are implemented and tested (HANDOFF §10, tasks 1–5). Next up is the pyref
reference-fixture pipeline (task 6) that feeds the validation ladder. The MVP
Tier-1 encounter, viewer, and the Δv-vs-lead-time curve are still ahead.

If you're reading the code: **`HANDOFF.md` is the source of truth** for *why*
things are the way they are. This README is the summary.

---

## How it works

A **headless, deterministic Rust simulation core** is the single source of truth.
Every view and the mission planner are *consumers* of the core's state — they
never own state — so views stay in sync and every scenario is reproducible
(same build → same output).

The mission planner doesn't compute trajectories itself. It pushes a Δv into the
core's state at a chosen time and asks the core to re-propagate. *"Did this
mission work?"* = *"run, mutate, re-run, compare miss distance."*

### Physics, in tiers

Realism is switched on in composable tiers — each tier is just a set of *enabled
acceleration terms* in the force model, not a code rewrite:

- **Tier 0** — cosmetic Kepler context orbits (never used for hit/miss).
- **Tier 1 (MVP)** — the asteroid integrated as a *test particle* in the real
  JPL **DE440/441 ephemeris field** (Sun + 8 planets + Moon), in the
  barycentric ICRF frame, with an adaptive high-order integrator (dop853).
  Hit/miss is decided by a proper **b-plane** geometry with a
  **gravitationally-focused capture radius** — Earth's gravity enlarges its own
  target — not a naive geometric radius.
- **Tier 2** — real-asteroid fidelity: 1PN general relativity, the Yarkovsky
  thermal force, solar radiation pressure, J2 oblateness, and the 16 main-belt
  asteroid perturbers. (This term list is deliberately ASSIST's force model.)
- **Tier 3** — uncertainty realism: carry the orbit-determination covariance,
  map it through the dynamics to the b-plane, and report an impact *probability*
  and risk corridor — plus gravitational **keyholes**. This is what real
  planetary defense actually reasons about.

### Deflection methods

Modeled as a spectrum across lead time: **gravity tractor** (decades of lead),
**kinetic impactor** (`Δv = β·m·v / M`, with DART's measured β ≈ 3.6), and
**nuclear standoff** (energy → ablation → momentum — modeled as deflection
physics only, never weapon design).

### Two honesty caveats, surfaced in the UI

- **Delivery.** "Tap it once, ten years out" elides the *delivery* problem
  (launch windows, transfer geometry). Until the Lambert/porkchop layer exists,
  the sim shows *"if you could deliver this impulse, here's what it buys you"* —
  not *"you can deliver it."*
- **Determinism.** The MVP shows a single deterministic track and a binary
  hit/miss. Real defense reasons over uncertainty — an impact *probability*
  (that's the Tier-3 layer).

---

## Build vs. borrow

The project **builds its own astrodynamics** — propagator, integrators, force
model, Lambert solver, b-plane geometry, deflection models — because that's the
part worth understanding deeply. It **borrows** only where a reinvented bug would
be silent and catastrophic:

| Concern | Crate | License |
|---|---|---|
| Time (TDB/TT/UTC, leap seconds) | `hifitime` | MPL-2.0 |
| Ephemerides, frames, GM constants | `ANISE` | MPL-2.0 |
| Linear algebra (f64 everywhere) | `nalgebra` | Apache-2.0/MIT |

Validation reference tools — `hapsira`, `REBOUND`, `ASSIST`, `GRSS`, `astropy`,
`nyx` — run **offline only** in a Python fixture pipeline (`pyref/`). Their
copyleft licenses don't constrain this project because nothing is linked into the
shipped binary; only their *generated data* is committed as fixtures.

## Validation

Correctness is checked against an **oracle ladder** matched to the regime — free
invariants (energy / angular momentum / LRL conservation) → analytic Kepler →
REBOUND → **ASSIST** (the trajectory oracle, since the shipping propagator *is*
the ASSIST configuration) → JPL Horizons on real asteroids. Each force term is
validated *in isolation* (e.g. the GR term alone must reproduce Mercury's
42.98″/century perihelion precession), not just the sum.

---

## Planned layout

A ✅ marks what exists in the tree today; the rest is the planned target shape.

```
workspace/
├── core/        # ✅ pure simulation engine — no renderer dependency
│   │            #    (epoch, state, orbital elements, Kepler propagator, ephemeris)
│   ├── forces/  # 🔜 composable, individually-toggleable acceleration terms
│   └── ...      # 🔜 integrator, geometry, lambert, clock, ...
├── viewer/      # ✅ scaffold only — MVP pure-Rust renderer (egui) comes at task 10
├── godot/       # 🔜 Phase 2: gdext binding, 3D rendering (not yet created)
├── validation/  # ✅ Rust test harness — links core only, loads fixtures
└── pyref/       # 🔜 Python scripts that generate validation fixtures (offline)
```

## Roadmap

- **MVP** — prove the thesis in pure Rust: Tier-1 encounter, honest hit→miss
  flip, the Δv-vs-lead-time curve, kinetic impactor. (egui viewer, no Godot.)
- **Phase 2** — Godot 3D frontend; Tier-2 realism; real NEOs (Apophis, Bennu,
  Didymos/Dimorphos); nuclear + gravity-tractor methods; Lambert/porkchop
  mission design; Tier-3 uncertainty & keyholes.
- **Phase 3** — launch vehicles & payload budgets, orbital assembly, standing
  defense systems, multi-mission campaigns.

See [`HANDOFF.md`](HANDOFF.md) for the complete spec, the locked decisions, the
known hard problems, and the task-by-task plan.

---

## License

Licensed under the **Boyko Non-Commercial License v1.0 (BNCL-1.0)** — see
[`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).

Free to use, modify, and distribute for **non-commercial purposes**. Commercial
use requires separate written permission from the copyright holder.
