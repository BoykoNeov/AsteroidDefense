# Asteroid Deflection Simulator — Development Handoff

This document is the starting context for continuing development in Claude Code. It captures the project vision, the architectural decisions that are **locked** (build against them, don't re-litigate them), the physics that must be correct, the validation strategy, the known hard problems, and a concrete first-tasks list.

---

## 1. What we're building

A solar-system model with 2D and 3D views, focused on **asteroid deflection and mission planning** — specifically, planning and simulating missions to deflect an Earth-bound asteroid, with different missions achieving different degrees of success.

The project has a single educational thesis it exists to demonstrate:

> **Deflecting an asteroid early — many orbits before the predicted impact — is dramatically more effective than deflecting it on final approach.** A tiny nudge applied years out beats a massive shove applied days out.

Everything in the app serves making a user *feel* this. The user should be able to attempt a last-minute deflection, watch it fail, rewind ten years, tap the asteroid once with a small impulse, and watch Earth slide safely out of the way.

**The single most important screen is a plot of required Δv vs. lead time** for a given asteroid and method. That curve *is* the thesis. Build the rest of the system to make that curve legible.

Secondary goals (later phases): different asteroids/comets (some deflectable, some not — still a lesson), different payloads (kinetic impactor, nuclear standoff), and eventually full mission planning (plausible launch vehicles, payload mass budgets, orbital assembly, standing defense systems).

---

## 2. Locked architectural decisions

- **Headless deterministic simulation core is the single source of truth.** The 2D renderer, 3D renderer, and mission planner are all *consumers* of the core's state — they never own state themselves. This keeps 2D and 3D in sync for free and makes every scenario reproducible.
- **Language split: Rust core + Godot desktop frontend**, bound via `godot-rust` (gdext).
- **Desktop only.** No web export. (Godot's web export is a heavy WASM canvas with cross-origin-isolation requirements — not worth it for this.)
- **Build our own astrodynamics**, validated against established Python/reference tools (see §6). The propagator, integrators, Lambert solver, and deflection models are the heart of the project and the thing worth understanding deeply.
- **Determinism discipline:** the core steps on its own **fixed timestep**; Godot renders at display rate and **interpolates** between core snapshots. Never tie the simulation to Godot's frame `delta`.

### The core/consumer relationship

```
Scenario / lesson layer  ──┐
                           ▼
Data sources (JPL/ESA) ──► Simulation core (source of truth) ──► 2D renderer
                           │  - propagation engine            ──► 3D renderer
                           │  - state, clock, events          ──► Mission planner
                           ▲                                         │
                           └──────────── apply Δv, re-run ◄──────────┘
```

The mission planner does not compute trajectories itself. It pushes a Δv into the core's state at a chosen time and asks the core to re-propagate. "Did this mission work?" = "run, mutate, re-run, compare miss distance."

---

## 3. Tech stack — the build vs. borrow line

There is a sharp line between code worth reinventing (the lesson) and code that will ship silent, catastrophic bugs if reinvented. Respect it.

### Build (this is the project)

- Orbital-element ↔ state-vector conversions
- Kepler propagation
- Integrator hierarchy: RK4 → adaptive Dormand-Prince (DoPri/dop853) → a symplectic option (leapfrog/Verlet/WHFast-style) for long stable spans
- Lambert solver (intercept trajectory design)
- b-plane / target-plane geometry and miss-distance computation
- Gravitational keyholes
- Deflection Δv models (kinetic, nuclear standoff, gravity tractor)

### Borrow (bugs here are invisible until the encounter is off by seconds = thousands of km)

| Concern | Crate | Why not DIY |
|---|---|---|
| Time (TDB/TT/UTC/leap seconds) | `hifitime` | Integer arithmetic (no float drift), validated against SPICE to 0 ns on ET↔UTC, flight-proven (Firefly Blue Ghost lunar lander). Time bugs are the classic "subtly wrong and invisible." |
| Ephemerides, frames, GM constants | `ANISE` | Modern Rust rewrite of NAIF SPICE, validated against SPICE to machine precision. Reads JPL DE440 kernels; gives ICRF/J2000 frames and μ values that exactly match JPL. |
| Linear algebra | `nalgebra` | `Vector3<f64>` etc. Use **f64 everywhere** in the physics, never f32. |

### Licensing landmine

- `hifitime` and `ANISE` are **MPL 2.0** — safe to link and ship.
- `nyx` (the full toolkit) is **AGPL 3.0** — strong copyleft. Fine to use as an *offline oracle* to generate validation fixtures (not distributed), but do **not** link/ship it unless the whole app goes AGPL.

---

## 4. Crate / module layout

A Cargo workspace with a clean separation so the physics is testable in complete isolation from the renderer:

```
workspace/
├── core/                  # pure simulation engine — NO Godot dependency
│   ├── state.rs           # StateVector, OrbitalElements, Epoch (hifitime), Body
│   ├── propagator.rs      # Propagator trait + Kepler/analytic impl
│   ├── integrator.rs      # Integrator trait + RK4, DoPri, symplectic impls
│   ├── nbody.rs           # N-body force model (Sun + planets + asteroid)
│   ├── deflection.rs      # kinetic / nuclear-standoff / gravity-tractor Δv models
│   ├── geometry.rs        # b-plane, miss distance, keyhole geometry
│   ├── lambert.rs         # Lambert solver for intercept design
│   ├── scenario.rs        # scenario definition + (de)serialization
│   └── clock.rs           # fixed-timestep simulation clock + snapshot/interpolation API
├── godot/                 # gdext binding crate — depends on core, owns rendering
├── validation/            # Rust test harness — links core ONLY, loads fixtures
└── pyref/                 # Python scripts that generate reference fixtures
```

Key trait boundaries to define first:

- `Propagator` — given a body + an epoch, return its state. Implementations: analytic Kepler (fast, for context planets) and numerically-integrated (for the asteroid + encounter).
- `Integrator` — a swappable ODE stepper so RK4 / DoPri / symplectic are interchangeable. Encounter accuracy depends on choosing an adaptive high-order stepper here.
- `Epoch` / time — wrap `hifitime`, never raw f64 seconds for absolute time.

---

## 5. The physics that must be correct

### The core mechanism (this is the thesis, mechanically)

A deflection mostly imparts an **along-track Δv**. That changes the asteroid's semi-major axis → changes its orbital period → the asteroid arrives progressively earlier/later on each subsequent orbit, and that timing error **accumulates** over many orbits. By the predicted impact date, a tiny Δv applied many orbits earlier has grown into a large along-track displacement. Required Δv to achieve a fixed miss (e.g. one Earth radius) falls roughly as **1 / (lead time)**.

### Deflection methods (model as a spectrum across lead time)

- **Gravity tractor** — tiny continuous tug, needs *decades* of lead time. (Reinforces the thesis from the gentle end.)
- **Kinetic impactor** — `Δv = β · (m_spacecraft · v_relative) / M_asteroid`, where β is the momentum-enhancement factor from ejecta. DART measured **β ≈ 3.6** at Dimorphos. Expose β as a toggle (e.g. 1 to ~4).
- **Nuclear standoff burst** — model as **energy deposited → surface ablation → momentum → Δv**, using public scaling relations. Largest Δv, for big rocks or short notice. **Model this as deflection physics only — never weapon design.**

### Keyholes

A close pass can thread a small region (a "keyhole") that sets up a resonant *return* impact years later (this is Apophis's real history). Deflecting an asteroid *out of a keyhole* needs far less Δv than deflecting it off a direct collision — a great counterintuitive sub-lesson.

### Non-negotiable accuracy rule

**Two-body Keplerian propagation is fine for drawing orbits but CANNOT decide whether the asteroid hits Earth.** Hit-vs-miss is governed by Earth's gravity during the encounter and is acutely sensitive to initial conditions. The encounter must be modeled with at least Earth's gravity (patched conics) or, better, full N-body with an **adaptive timestep**. This is where the entire emotional payload lives — spend the accuracy budget here.

---

## 6. Validation strategy

### Match the oracle to the regime

The common mistake is validating everything against one library. Each regime has a different right answer:

| What you're validating | Reference oracle | What to check |
|---|---|---|
| Kepler / two-body prop | hapsira + analytic solution | Near machine precision over short arcs |
| Lambert solver | Vallado canonical test cases | Published numbers, multi-rev branches |
| Frames & time | Astropy + hifitime/ANISE | Both already SPICE-validated |
| Perturbed / N-body / encounter | REBOUND (IAS15 integrator) | Relative error growth, not absolute |
| Real asteroid trajectories | JPL Horizons state vectors | Ground truth (Apophis, Bennu, etc.) |

REBOUND is specifically the **encounter oracle** — gold standard for the close-approach dynamics that decide hit-vs-miss. Don't validate N-body against hapsira (it's two-body/patched-conic focused).

### Make it a harness, not a one-off

1. Define scenarios as data (RON or JSON): initial state + reference states at checkpoints.
2. Generate the reference column once with Python (`pyref/`), commit as fixtures.
3. Rust test suite (`validation/`) loads fixtures and asserts within a **per-regime tolerance**.

### The gotcha that wastes a full day

**Pin μ, AU, frame, and time scale identically on both sides.** Most "my Rust is wrong" panics are actually one side using a Wikipedia μ and the other using JPL's. Pull the same GM and DE values through ANISE on the Rust side to kill this entire class of phantom failure.

### Free invariants (no external oracle needed)

In pure two-body, **energy, angular momentum, and the Laplace–Runge–Lenz vector are all conserved**, and forward-then-backward propagation should return to the start. Wire these as `proptest` property tests over randomized orbits — they catch most integrator bugs before Python is even involved. Build these first.

---

## 7. Known hard problems (design for these from day one)

- **Scale.** The solar system spans 8+ orders of magnitude; you cannot draw the Sun, planets, an asteroid, and a spacecraft trajectory to scale on one screen. Plan for log-compressed distance toggles, "sizes not to scale" modes, and multiple zoom regimes (whole system → Earth's neighborhood → encounter). The 2D schematic is often the *clearer* teaching tool, not a lesser one.

- **Float precision in Godot at solar-system scale.** Standard Godot uses f32 transforms; at 1 AU (~1.5×10¹¹ m), f32 spacing is ~18 km between representable positions → visible jitter, fatal for Earth-radius miss geometry. This is a *rendering* problem only — the core holds true f64 state and Godot just draws it. Handle via: (a) **floating origin** — recenter the render frame on Earth for the encounter view so local coordinates are small; (b) optionally a **double-precision Godot build** (Large World Coordinates, `precision=double`, compiled from source); (c) the non-linear display transform you're using for schematic/wide views sidesteps raw precision anyway.

- **Time spans.** Centuries (orbital sweep) down to hours (encounter), with variable time-warp.

- **Numerical accuracy at the encounter.** Adaptive high-order integrator required (IAS15-style or dop853). Fixed-step integrators lose accuracy exactly when it matters most.

- **Lambert + porkchop plots.** To make missions that *actually reach* the asteroid, solve Lambert's problem (departure/arrival positions + flight time → connecting orbit + launch Δv). Sweeping launch/arrival dates gives a porkchop plot. This is where the future mission/payload planning layer bolts on naturally (porkchop encodes launch windows → feeds rocket/payload sizing).

---

## 8. Phasing / roadmap

### MVP — prove the thesis

- 2D top-down ecliptic view
- A few planets on Keplerian orbits (context)
- One asteroid on an Earth-collision orbit
- Time slider / play / time-warp
- One method: kinetic impactor, parameterized by Δv (with β factor)
- Apply Δv at a chosen lead time → re-propagate → show new miss distance
- **The payoff chart: required Δv vs. lead time**

That MVP already delivers the whole lesson. Everything below is layering.

### Phase 2

- 3D view (Godot SubViewport composition: 2D schematic/HUD over 3D, or vice versa)
- Real NEOs from the JPL Small-Body Database (see §9)
- Nuclear standoff + gravity-tractor methods
- Lambert / porkchop mission design
- Keyholes
- Orbit uncertainty (covariance ellipse shrinking with observations)

### Phase 3 (future)

- Plausible launch vehicles + payload mass budgets
- Orbital assembly (assemble-in-orbit when payload too big for one launch)
- Standing/ready Earth-defense systems
- Multi-mission campaigns

---

## 9. Data sources & teaching asteroids

- **JPL Horizons** — state vectors, the ground-truth reference for real trajectories.
- **JPL Small-Body Database** — orbital elements for real NEOs.
- **ESA NEOCC** — secondary cross-reference.

Teaching asteroids worth seeding:

- **Apophis** — the perfect teaching case: famous 2029 close approach and real keyhole history.
- **Didymos / Dimorphos** — the DART target; gives you a real, measured β for free.
- **Bennu** — well-characterized, OSIRIS-REx data.

---

## 10. First tasks for Claude Code

1. Scaffold the Cargo workspace: `core/` (no Godot dep), `godot/` (gdext), `validation/`, `pyref/`.
2. Implement `Epoch` (wrapping hifitime), `StateVector`, `OrbitalElements`, and the element↔state conversions.
3. Implement the analytic Kepler propagator behind the `Propagator` trait.
4. Wire the free invariant property tests (energy / angular momentum / LRL vector conservation + forward-back reversibility) with `proptest`. **Do this before any external validation** — it catches most bugs immediately.
5. Stand up one Python fixture in `pyref/` (propagate a known orbit via hapsira, commit reference states) and the matching Rust test in `validation/`. Pin μ/frame/time-scale identically.
6. Implement the RK4 integrator behind the `Integrator` trait, then a first N-body model (Sun + Earth + asteroid), validated against REBOUND.
7. Build the fixed-timestep `clock` with a snapshot + interpolation API (for Godot to render against).

At that point the engine can support the MVP scenario, and the Godot frontend work can begin in parallel.

---

## Open questions / deferred decisions

- **Scenario file format** — RON suggested (Rust-native, human-editable), but not locked.
- **MVP planet positions** — analytic Kepler (simplest) vs. DE440 via ANISE. Suggest analytic for MVP, DE440 in Phase 2.
- **Default encounter integrator** — adaptive IAS15-style vs. dop853. Decide empirically against REBOUND.
- **Godot precision approach** — start with floating origin (recenter on Earth); only compile a double-precision build if floating origin proves insufficient.
