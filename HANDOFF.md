# Asteroid Deflection Simulator — Development Handoff

This document is the starting context for continuing development in Claude Code. It captures the project vision, the architectural decisions that are **locked** (build against them, don't re-litigate them), the physics that must be correct, the validation strategy, the known hard problems, and a concrete first-tasks list.

> **Revision note (2026-06-23).** This handoff was pressure-tested and re-scoped after the first review. Locked decisions: the MVP renderer is **pure Rust** (Godot deferred to Phase 2); the MVP must deliver an **honest hit→miss flip** (not just the curve); the asteroid is integrated as a **test particle in the DE440/441 ephemeris field from Tier 1 onward**, so every tier is a pure force-term toggle (not a structural rewrite) and ASSIST is the oracle from day one. The physics target is **realism** — operationalized as the tiered force model in §5. Sections §2, §3, §5, §6, §7, §8, §10 were revised accordingly. The pre-review version is preserved in `HANDOFF.backup.md`.
>
> **Second pass (2026-06-23, same day).** A follow-up discussion resolved the remaining open questions and two previously-implicit decisions. Now locked: integrate in the **barycentric (SSB-centered) ICRF frame** in **SI units**, present heliocentric (§2, §5); **dop853 is the MVP integrator** (IAS15 is a Tier-2 long-arc upgrade) and the **clock interpolates from dop853's dense output**, not linearly (§4–§7, §10); the **pure-Rust viewer is egui** (egui_plot + painter), plotters optional (§2, §8); the headline Δv-vs-lead-time curve **fixes the impulse phase**, with phase exposed as a **separate** interactive view (§5, §7); **scenarios/fixtures are JSON** (§6); a **task-0.5 ASSIST+DE441 build / ANISE DE-position spike** gates the plan, with an explicit **fallback-to-Option-B trigger** if it stalls (§10); the MVP **soft-caps impulse magnitude** to kinetic-impactor plausibility and carries **delivery + determinism honesty caveats** in UI copy (§1, §5). The MVP perturber set stays Sun+8 planets+Moon, with the force term and ANISE loader **designed to add the 16 asteroid perturbers** at Tier 2 (§5).

---

## 1. What we're building

A solar-system model with 2D and 3D views, focused on **asteroid deflection and mission planning** — specifically, planning and simulating missions to deflect an Earth-bound asteroid, with different missions achieving different degrees of success.

The project has a single educational thesis it exists to demonstrate:

> **Deflecting an asteroid early — many orbits before the predicted impact — is dramatically more effective than deflecting it on final approach.** A tiny nudge applied years out beats a massive shove applied days out.

Everything in the app serves making a user *feel* this. The user should be able to attempt a last-minute deflection, watch it fail, rewind ten years, tap the asteroid once with a small impulse, and watch Earth slide safely out of the way.

**The single most important screen is a plot of required Δv vs. lead time** for a given asteroid and method. That curve *is* the thesis. Build the rest of the system to make that curve legible.

We are aiming at **realism**, not a cartoon: the dynamics that decide hit-vs-miss are modeled at ephemeris quality (see §5), validated against the same oracles professional planetary-defense tools use (see §6). The MVP turns realism *on* only as far as a synthetic teaching asteroid requires; the architecture is built so the remaining realism (GR, Yarkovsky, ephemeris perturbers, orbit-uncertainty) switches on without a rewrite.

**Two honesty caveats baked into the pedagogy, both surfaced in UI copy:**
- **Delivery.** "Tap it once, ten years out" elides the *delivery* problem. In reality an early impulse is gated by launch windows and transfer geometry (the Lambert/porkchop layer, deferred to Phase 2). Until that layer exists, the sim shows *"if you could deliver this impulse, here is what it buys you"* — not *"you can deliver it."*
- **Determinism.** The MVP shows a single deterministic track and a binary hit/miss. Real planetary defense reasons over orbit-determination *uncertainty* — an impact *probability*, not a yes/no (this is the Tier-3 layer, §5). One line of UI copy should say so (*"real tracks carry uncertainty; Tier 3 turns this single line into a probability"*), so the deterministic demo isn't mistaken for the whole story.

---

## 2. Locked architectural decisions

- **Headless deterministic simulation core is the single source of truth.** The 2D renderer, 3D renderer, and mission planner are all *consumers* of the core's state — they never own state themselves. This keeps views in sync for free and makes every scenario reproducible.
- **Determinism means same-build-same-output**, *not* bit-reproducibility across machines. A given compiled binary replays a scenario identically (so rewind/replay and saved lessons are exact). We deliberately do **not** pursue cross-platform bit-identity: adaptive integrators choose steps from floating-point error estimates, so bit-portability would pin compiler flags / math libs / FMA settings for benefit we don't need — the validation oracle (§6) compares within a *tolerance*, never bit-for-bit.
- **Fixed *cadence*, adaptive *step*.** "Fixed timestep" refers to the **clock's snapshot cadence**: the core emits state snapshots on a fixed simulation-time interval for the renderer to interpolate between. The **integrator** adaptively subdivides *between* snapshots to reach each snapshot time under an error tolerance. Fixed snapshot interval ≠ fixed integration step — these are not in tension. Never tie the simulation to Godot's/viewer's frame `delta`.
- **MVP renderer is pure Rust — `egui` is the spine** (immediate-mode shell for the controls, `egui_plot` for the Δv curve, `egui::Painter` for the top-down orbital animation; `plotters` optional later if charts need export/polish). **Godot is deferred to Phase 2.** Rationale: the thesis curve and the hit→miss animation are the entire MVP payload, and the `godot-rust` (gdext) binding is the riskiest, least-physics-bearing part of the stack. egui is the only pure-Rust option adequate at controls + chart + animation in a *single* crate; macroquad would render the animation slightly better but force a worse GUI — and game-like polish is exactly what the Phase-2 Godot frontend is for. Proving the physics behind the cheapest possible renderer de-risks the core independently and gets the "money chart" weeks earlier.
- **Language: Rust core + Rust viewer for MVP**; **Godot desktop frontend in Phase 2**, bound via `godot-rust` (gdext).
- **Desktop only.** No web export. (Godot's web export is a heavy WASM canvas with cross-origin-isolation requirements — not worth it for this.)
- **Build our own astrodynamics**, validated against established reference tools (see §6). The propagator, integrators, force model, Lambert solver, and deflection models are the heart of the project and the thing worth understanding deeply.
- **Integrate barycentric (SSB-centered) ICRF, in SI units; present heliocentric.** The integration frame is the Solar-System-barycenter ICRF — matching DE440/441 and ASSIST directly, and avoiding the non-inertial heliocentric "indirect term" footgun (§5). Use SI (m, s, kg) in the core for legibility; convert only at the ASSIST comparison boundary. Because the core is **f64 everywhere**, the f32 precision worry in §7 is a *rendering-only* problem and never touches a result (f64 spacing at 1 AU is ~15 µm, vs. ~16–18 km for f32).

### The core/consumer relationship

```
Scenario / lesson layer  ──┐
                           ▼
Data sources (JPL/ESA) ──► Simulation core (source of truth) ──► 2D viewer (MVP: pure Rust)
                           │  - composable force model        ──► 3D renderer (Phase 2: Godot)
                           │  - integrators + clock + events  ──► Mission planner
                           ▲                                         │
                           └──────────── apply Δv, re-run ◄──────────┘
```

The mission planner does not compute trajectories itself. It pushes a Δv into the core's state at a chosen time and asks the core to re-propagate. "Did this mission work?" = "run, mutate, re-run, compare miss distance (and, in Tier 3, impact probability)."

---

## 3. Tech stack — the build vs. borrow line

There is a sharp line between code worth reinventing (the lesson) and code that will ship silent, catastrophic bugs if reinvented. Respect it.

### Build (this is the project)

- Orbital-element ↔ state-vector conversions
- Kepler propagation
- Integrator hierarchy: RK4 → adaptive Dormand-Prince (DoPri/dop853) → IAS15/Gauss-Radau-style and a symplectic option (leapfrog/Verlet/WHFast-style) for long stable spans
- **Composable force model** (see §5): each acceleration term (Sun, planets, Moon, GR, J2, Yarkovsky, SRP) a separately-toggleable, separately-validated unit
- Lambert solver (intercept trajectory design)
- b-plane / target-plane geometry, gravitationally-focused miss-distance/capture computation
- Gravitational keyholes
- Deflection Δv models (kinetic, nuclear standoff, gravity tractor)
- Orbit-uncertainty → impact-probability mapping (Tier 3)

### Borrow — link & ship (bugs here are invisible until the encounter is off by seconds = thousands of km)

| Concern | Crate | License | Why not DIY |
|---|---|---|---|
| Time (TDB/TT/UTC/leap seconds) | `hifitime` | MPL 2.0 | Integer arithmetic (no float drift), validated against SPICE to 0 ns on ET↔UTC, flight-proven (Firefly Blue Ghost lunar lander). Time bugs are the classic "subtly wrong and invisible." |
| Ephemerides, frames, GM constants | `ANISE` | MPL 2.0 | Modern Rust rewrite of NAIF SPICE, validated to machine precision. Reads JPL DE440/DE441 kernels; gives ICRF/J2000 frames and μ values that exactly match JPL. **Used in the MVP for both GM constants *and* DE440/441 perturber positions** (the asteroid is a test particle in this field — see §5). Kills the μ-mismatch bug class (see §6); the kernel reader must work before first-light. |
| Linear algebra | `nalgebra` | Apache-2.0/MIT | `Vector3<f64>` etc. Use **f64 everywhere** in the physics, never f32. |

### Borrow — offline oracles only (Python `pyref/`, never linked into the shipped binary)

These generate validation fixtures. Their copyleft licenses don't constrain us because we run them offline and commit only their *output* (data, not a derivative work).

| Oracle | Regime it validates | License | Notes |
|---|---|---|---|
| `hapsira` (maintained poliastro successor) | Two-body / Kepler / Lambert | MIT-family | Analytic-precision short arcs; Vallado Lambert cases. |
| `REBOUND` (IAS15) | Integrator + encounter sensitivity (synthetic, self-consistent N-body) | GPL-3.0 | Gold-standard close-approach dynamics. Self-gravitates the planets — see §6 oracle ladder. |
| `ASSIST` (REBOUND extension) | **Full ephemeris-quality force model** (GR, Sun/Earth J2, Moon, 16 main-belt asteroid perturbers, A1/A2/A3 non-gravs) | GPL-3.0 | Test particle in the **DE441** field on IAS15, validated to ~meter level vs JPL over decades. **Its force-term list IS our realism spec.** Ships first-order **variational equations for all terms → built-in covariance mapping** (direct gift to Tier 3). `github.com/matthewholman/assist`, arXiv 2303.16246. |
| `GRSS` (Gauss-Radau Small-body Simulator) | Planetary-defense reference (impact monitoring, b-plane, keyholes, close approaches) | open-source | Purpose-built for the Tier-3 impact-probability layer; cross-check geometry/keyhole logic against it. |
| `astropy` | Frames & time cross-check | BSD | Both it and hifitime/ANISE are independently SPICE-validated. |
| `nyx` | Optional full-toolkit oracle | AGPL-3.0 | Offline only. **Never link/ship** unless the whole app goes AGPL. |

### Licensing landmine

- **Only `hifitime`, `ANISE`, `nalgebra` are linked into the shipped binary** — all permissive/MPL, safe.
- **Everything else (`nyx`, `REBOUND`, `ASSIST`, `GRSS`) lives exclusively in the offline `pyref/` fixture pipeline.** GPL/AGPL is fine there because nothing is linked into the distributed Rust and only generated *data* is committed. The one real hazard is `nyx` (AGPL, *Rust*) — easy to accidentally add to a Cargo manifest. Keep it out of every `Cargo.toml`.

---

## 4. Crate / module layout

A Cargo workspace with a clean separation so the physics is testable in complete isolation from the renderer:

```
workspace/
├── core/                  # pure simulation engine — NO renderer dependency
│   ├── state.rs           # StateVector, OrbitalElements, Epoch (hifitime), Body
│   ├── propagator.rs      # Propagator trait + Kepler/analytic impl
│   ├── integrator.rs      # Integrator trait + RK4, DoPri, IAS15-style, symplectic impls
│   ├── forces/            # composable acceleration terms (see §5)
│   │   ├── mod.rs         #   ForceModel = Σ(terms); each term toggleable + unit-tested
│   │   ├── point_mass.rs  #   arbitrary perturber list, positions from any ephemeris (Sun+planets+Moon via DE440/441/ANISE); +16 asteroids at Tier 2 (§5)
│   │   ├── relativity.rs  #   1PN (parameterized post-Newtonian) Sun term
│   │   ├── oblateness.rs  #   Earth/Sun J2
│   │   ├── yarkovsky.rs   #   diurnal + seasonal thermal recoil (transverse A2/r² form)
│   │   └── srp.rs         #   solar radiation pressure
│   ├── geometry.rs        # b-plane, gravitationally-focused capture radius, keyhole geometry
│   ├── lambert.rs         # Lambert solver for intercept design
│   ├── deflection.rs      # kinetic / nuclear-standoff / gravity-tractor Δv models
│   ├── uncertainty.rs     # covariance → b-plane → impact probability (Tier 3)
│   ├── scenario.rs        # scenario definition + (de)serialization
│   └── clock.rs           # fixed-cadence clock; sub-snapshot queries served from integrator dense output (§5), not linear interp
├── viewer/                # MVP pure-Rust renderer (egui spine: egui_plot + painter) — depends on core
├── godot/                 # Phase 2: gdext binding crate — depends on core, owns 3D rendering
├── validation/            # Rust test harness — links core ONLY, loads fixtures
└── pyref/                 # Python scripts (hapsira/REBOUND/ASSIST/GRSS) that generate fixtures
```

Key trait boundaries to define first:

- `Propagator` — given a body + an epoch, return its state. Implementations: analytic Kepler (fast, for context planets) and numerically-integrated (for the asteroid + encounter).
- `Integrator` — a swappable ODE stepper so RK4 / DoPri / IAS15 / symplectic are interchangeable. Encounter accuracy depends on choosing an adaptive high-order stepper here.
- `ForceModel` — a sum of individually-toggleable acceleration `terms`. Tiers (§5) are *which terms are enabled*, not separate code paths. Each term is unit-validated in isolation (§6).
- `Epoch` / time — wrap `hifitime`, never raw f64 seconds for absolute time.

---

## 5. The physics that must be correct

### The core mechanism (this is the thesis, mechanically)

A deflection mostly imparts an **along-track Δv**. That changes the asteroid's semi-major axis → changes its orbital period → the asteroid arrives progressively earlier/later on each subsequent orbit, and that timing error **accumulates** over many orbits. By the predicted impact date, a tiny Δv applied many orbits earlier has grown into a large along-track displacement. Required Δv to achieve a fixed miss falls roughly as **1 / (lead time)**.

> **The curve is not a clean hyperbola.** Superimposed on the 1/t trend is oscillatory structure: the sensitivity of the final miss to an impulse depends on the **true anomaly at the moment of application** (there are sweet spots near perihelion). Don't debug the wiggles as if they were a bug. **Resolved (2nd pass):** the headline curve **fixes the application phase** so it reads as a clean function of lead time (the thesis); the phase dependence (the perihelion sweet-spots) is exposed as a **separate** interactive view — a deliberate sub-lesson, not noise on the main curve.

### Hit-vs-miss is decided by the encounter, not the heliocentric arc

**Two-body Keplerian propagation is fine for drawing orbits but CANNOT decide whether the asteroid hits Earth.** Hit-vs-miss is governed by Earth's (and the Moon's) gravity during the close approach and is acutely sensitive to initial conditions. This is where the entire emotional payload lives — spend the accuracy budget here.

- **Hit criterion = gravitationally-focused capture radius**, not geometric Earth radius:
  `b_impact = R⊕ · √(1 + (v_esc / v_inf)²)`
  Earth's gravity *enlarges its own target* (factor ~1.2–2.4× for typical NEO `v_inf`). This is the correct b-plane impact test **and** a pedagogical gift. The ~100 km atmosphere height is cosmetic next to gravitational focusing.
- **Moon resolved separately during the encounter.** Lumping the Moon into the Earth-Moon barycenter shifts the gravity source by ~Earth-radius scale and corrupts the b-plane. **DE440/441 footgun:** the ephemeris natively provides the Earth-**Moon barycenter** plus a lunar offset — the geocenter is *reconstructed*. Carelessly using the EMB as "Earth's position" displaces Earth by **~4671 km** → Earth-radius-scale b-plane error. Always reconstruct the geocenter and carry the Moon as a separate perturber.
- **Integrate barycentric, not heliocentric (same class of footgun).** Integrate in the **SSB-centered ICRF** frame (matching DE440/441 and ASSIST). A Sun-centered frame is **non-inertial**: it owes an **indirect term** (the negative of the Sun's own acceleration due to the planets), and omitting it is a textbook ~planet-mass-ratio error — the same silent, encounter-corrupting class as the EMB/geocenter mistake. Integrate barycentric; transform to heliocentric only for *display*.

### Realism = a tiered, composable force model

Realism is the goal, but it's switched on in tiers so the MVP stays achievable. Each tier is a set of *enabled acceleration terms* in the composable `ForceModel` (§4) — adding a tier is flipping flags, not rewriting.

**Tier 0 — context orbits (cosmetic).** Two-body Kepler for the background planet visuals. Never used for any hit/miss decision.

**Tier 1 — MVP encounter (honest hit/miss).** The asteroid is integrated as a **test particle in the DE440/441 ephemeris field** (Sun + all planets + Moon as point-mass perturbers, positions and GM from ANISE) with an **adaptive high-order integrator — dop853 for the MVP** (8th-order Dormand-Prince: easier to get right, and its 7th-order dense output also feeds the clock's sub-snapshot interpolation; IAS15 is a Tier-2 long-arc upgrade, not needed for one encounter). Earth as a finite body via the focused capture radius above; Moon carried separately (geocenter reconstructed — see footgun above). b-plane miss geometry. The MVP asteroid is *synthetic* (no Horizons ground truth), but the perturber field is the *real* one — exactly the ASSIST setup with the non-gravitational/relativistic terms switched off. Including all 8 planets is nearly free (ephemeris lookups, not extra integrated bodies); among the giants Jupiter is the principal perturber, but note the along-track drift that drives the thesis comes from the asteroid's *own* Δa (from the Δv), not from any third body.

**Tier 2 — real-asteroid fidelity (to match Horizons).** The perturber field is *already* DE440/441 ephemeris from Tier 1, so this tier is purely **enabling additional force terms** (a config toggle, no structural change):
- **Relativistic 1PN correction** (parameterized post-Newtonian Sun term). JPL includes it; matters for low-perihelion bodies like Apophis.
- **Yarkovsky effect** — diurnal + seasonal thermal recoil; **dominates decade-scale along-track drift** of real asteroids (Bennu is the textbook case). Modeled as a transverse acceleration (A2/r² style); needs spin axis, rotation period, thermal inertia, size, density.
- **Solar radiation pressure** — small bodies and spacecraft.
- **Earth/Sun J2** (oblateness) for very close flybys and keyhole geometry.
- **Major asteroid perturbers** (the 16 ASSIST carries — Ceres/Pallas/Vesta dominate) for long-arc precision. *Planned-for since the MVP:* `point_mass.rs` takes an arbitrary perturber list and ANISE can mount a second kernel (the small-body SPK `sb441-n16.bsp` ASSIST uses alongside DE441), with GMs from ASSIST's constants — so adding these 16 is a config/data change, not a code rewrite.

This tier's term list is deliberately **ASSIST's force model** — adopt it as the spec rather than hand-deriving.

**Tier 3 — uncertainty realism (the most "real" part of planetary defense).** Real defense is probabilistic, not binary. Carry the asteroid's **orbit-determination covariance** (from JPL SBDB), map it through the dynamics to the **b-plane** (linearized via variational equations, or Monte Carlo), and report an **impact *probability*** and risk corridor — not just a miss distance. This reframes deflection success as *"drive impact probability below threshold,"* and is what makes keyholes legible (a keyhole is a tiny b-plane region whose covariance overlap sets up a resonant return). ASSIST's built-in variational equations and GRSS's impact-monitoring logic are the references here.

### Deflection methods (model as a spectrum across lead time)

- **Gravity tractor** — tiny continuous tug, needs *decades* of lead time. (Reinforces the thesis from the gentle end.)
- **Kinetic impactor** — `Δv = β · (m_spacecraft · v_relative) / M_asteroid`, where β is the momentum-enhancement factor from ejecta. DART measured **β ≈ 3.6** at Dimorphos. Expose β as a toggle (1 to ~4). Model the impulse as a **vector** at the real impact geometry; the *along-track component* is what the thesis optimizes (ties to the perihelion sweet-spot note above). **Soft-cap the impulse magnitude** to what's physically plausible for a kinetic impactor — derive Δv from spacecraft mass × relative velocity × β rather than letting the user dial an arbitrary number; when a scenario needs more, surface it honestly (*"this would take N DART-class impactors"*) instead of silently allowing an impossible nudge. Keeps the MVP honest without the full Lambert/delivery layer (§7).
- **Nuclear standoff burst** — model as **energy deposited → surface ablation → momentum → Δv**, using public scaling relations. Largest Δv, for big rocks or short notice. **Model this as deflection physics only — never weapon design.**

### Keyholes

A close pass can thread a small region (a "keyhole") that sets up a resonant *return* impact years later (this is Apophis's real history). Deflecting an asteroid *out of a keyhole* needs far less Δv than deflecting it off a direct collision — a great counterintuitive sub-lesson. Keyholes are properly a Tier-3 (covariance/b-plane) phenomenon.

---

## 6. Validation strategy

### The oracle ladder (synthetic → real)

The common mistake is validating everything against one library. The right oracle depends on the regime, and on the kind of agreement you're after: a **synthetic** asteroid (MVP) has no ground-truth *track*, so you validate the propagator **structurally** — our implementation vs. ASSIST's, same force configuration, agreement = code correctness — whereas a **real** asteroid (Phase 2) is checked against **Horizons as physical ground truth**. Either way the perturber field is the real DE440/441 ephemeris; only the asteroid's own state is invented in the MVP.

1. **Free invariants** (no external oracle) → integrator sanity. *Build first.*
2. **`hapsira` + analytic solution** → Kepler / two-body / element-state conversions, near machine precision over short arcs; Lambert via Vallado canonical cases.
3. **`REBOUND` (IAS15)** → the **integrator + encounter sensitivity** on a *synthetic, self-consistent* N-body you fully control. Use it for the free-invariant cross-checks and for studying how sensitively the b-plane responds to ICs/Δv — *not* as the trajectory oracle, since REBOUND self-gravitates the planets and won't match our ephemeris-perturber propagator over long arcs.
4. **`ASSIST`** → the trajectory oracle **from Tier 1 onward**, because our shipping propagator *is* the ASSIST configuration (test particle in the DE441 field): in Tier 1, run ASSIST with the non-grav/relativistic terms off and compare; in Tier 2, turn the matching terms on on both sides. Its force-term list defines the realism spec, and its variational equations also validate the Tier-3 covariance mapping. Cross-check keyhole/impact-monitoring geometry against **GRSS**.
5. **`astropy`** → frames & time cross-check (independently SPICE-validated, like hifitime/ANISE).
6. **JPL Horizons state vectors** → final ground truth on **real** asteroids (Apophis, Bennu, Didymos). Only meaningful once Tier 2 is on — *real-asteroid arcs will not match Horizons without GR and Yarkovsky.*

### Validate per *term* and per *propagator*, not just the sum

- **Per force term, in isolation.** A summed comparison can mask a sign error in one term. Concrete unit checks: the **GR term alone must reproduce Mercury's 42.98″/century perihelion precession** (closed-form); J2 alone reproduces nodal regression; Yarkovsky alone produces the right secular da/dt sign and magnitude.
- **Per propagator, with the right expectation.** The "free invariants" (below) mean different things for different steppers — don't assert blanket conservation:
  - **analytic Kepler** → conserves everything *by construction*. (So invariant tests on it really only exercise the **element↔state conversions**, not any integrator — don't read green here as validating an integrator.)
  - **symplectic** → energy *bounded/oscillating*, not constant.
  - **RK4 / DoPri** → energy **drifts**; assert the *error-growth rate*, not conservation. (RK4 will correctly *fail* a naive energy-conservation assertion.)

### Element↔state conversions: target the singularities explicitly

The conversions blow up at **e→0** (argument of perihelion undefined) and **i→0** (node undefined). Randomized `proptest` orbits will sail right past these and pass while the real bugs hide. The property tests **must** explicitly include near-circular and near-equatorial cases.

### Free invariants (no external oracle needed) — build first

In pure two-body, **energy, angular momentum, and the Laplace–Runge–Lenz vector are conserved**, and forward-then-backward propagation returns to the start. Wire these as `proptest` property tests over randomized orbits (plus the singular cases above) — with the per-propagator expectations above. They catch most integrator bugs before Python is even involved.

### Make it a harness, not a one-off

1. Define scenarios as data (**JSON** — it crosses the Rust↔Python `pyref/` boundary natively; RON optional later for Rust-only authoring): initial state + reference states at checkpoints.
2. Generate the reference column once with Python (`pyref/`, using the matched oracle from the ladder), commit as fixtures.
3. Rust test suite (`validation/`) loads fixtures and asserts within a **per-regime tolerance**.

### The gotcha that wastes a full day

**Pin μ, AU, frame, and time scale identically on both sides.** Most "my Rust is wrong" panics are actually one side using a Wikipedia μ and the other using JPL's. Pull the same GM and DE values through ANISE on the Rust side — and configure the Python oracle from the same constants — to kill this entire class of phantom failure.

---

## 7. Known hard problems (design for these from day one)

- **Scale.** The solar system spans 8+ orders of magnitude; you cannot draw the Sun, planets, an asteroid, and a spacecraft trajectory to scale on one screen. Plan for log-compressed distance toggles, "sizes not to scale" modes, and multiple zoom regimes (whole system → Earth's neighborhood → encounter). The 2D schematic is often the *clearer* teaching tool, not a lesser one.

- **Float precision at solar-system scale — a *rendering* problem only.** At 1 AU (~1.5×10¹¹ m), **f32** spacing is ~16–18 km between representable positions → visible jitter, fatal for Earth-radius miss geometry. But the **core holds true f64 state** (f64 spacing at 1 AU is ~15 µm), so this never touches a result — it only affects how f64 world state is fed to an f32 renderer. For the pure-Rust MVP viewer (egui), work in a **recentered (floating-origin)** frame for the encounter view. In Phase 2 Godot, three complementary approaches cover different views — **decision: floating-origin first, double-precision build only as a fallback**:
  - **(a) Floating origin** *(default)* — each frame, subtract a chosen origin (Earth, during the encounter) before casting f64→f32, so the renderer only sees small numbers near zero where f32 is dense. Cheap, works with **stock Godot**, and covers the one precision-critical view (the encounter).
  - **(b) Double-precision Godot build** *(fallback only)* — compile from source with `precision=double` (Large World Coordinates); gdext must match the double-precision ABI. "Just works" with absolute coordinates but is a heavy, non-standard build to maintain — and the GPU pipeline is still f32, so you often recenter anyway. Use only if (a) proves insufficient.
  - **(c) Non-linear schematic transform** — the whole-system "not to scale" view already log-compresses distances before f32 sees them, so precision is moot there for free.

- **Time spans.** Centuries (orbital sweep) down to hours (encounter), with variable time-warp. Adaptive stepping (below) is what makes this tractable.

- **Numerical accuracy at the encounter.** Adaptive high-order integrator required. **Decision: dop853 is the MVP integrator** — at tight tolerance it is genuinely accurate for one Earth encounter plus a modest orbit count, it's easier to implement correctly than IAS15, and its dense output feeds the clock's interpolation. **IAS15 is a Tier-2 upgrade** for many-revolution long arcs (its near-symplectic edge), not a prerequisite for the MVP. Fixed-step integrators lose accuracy exactly when it matters most; never use one here. Re-confirm the dop853→IAS15 crossover empirically against REBOUND when Tier-2 long arcs arrive.

- **Relativity.** Real NEO trajectories — especially low-perihelion ones (Apophis) — do not match JPL without the 1PN Sun correction. Cheap to add as a force term; **omit it and Horizons validation silently fails.** (Tier 2.)

- **Yarkovsky thermal force.** Over decade scales this **dominates** real-asteroid trajectory uncertainty (Bennu is the textbook case). Long-arc validation against Horizons **will not match without it** — list it here so it's not discovered as a "my Rust is wrong" panic. (Tier 2.) Requires physical/spin parameters per asteroid.

- **Orbit uncertainty is the real domain, not a nicety.** Professional planetary defense reasons in **impact probability** over a covariance, not a single deterministic track. Keyholes only make sense in this frame. Design `uncertainty.rs` (covariance → b-plane → probability) as a first-class Tier-3 deliverable; ASSIST's variational equations and GRSS are the oracles. (Tier 3.)

- **Lambert + porkchop plots.** To make missions that *actually reach* the asteroid, solve Lambert's problem (departure/arrival positions + flight time → connecting orbit + launch Δv). Sweeping launch/arrival dates gives a porkchop plot. This is where the future mission/payload planning layer bolts on naturally — and it's what makes the "tap it once, years out" narrative *honest* (the impulse has to be deliverable within a launch window).

- **The thesis curve's fine structure.** Oscillation on top of 1/t (perihelion sweet spots) — see §5. **Resolved:** fix the application phase for the headline curve; expose phase as a separate view. Don't mistake the structure for a bug.

---

## 8. Phasing / roadmap

### MVP — prove the thesis (pure Rust, honest hit→miss)

- Pure-Rust 2D top-down ecliptic view (**egui**: `egui_plot` for the curve, painter for the orbital view) — **no Godot**
- A few context planets (Tier 0 Kepler) for orientation
- One **synthetic** asteroid on an Earth-collision orbit
- **Tier 1 force model**: asteroid as a test particle in the DE440/441 ephemeris field (Sun + planets + Moon via ANISE), **barycentric ICRF**, **dop853** adaptive integrator, validated against **ASSIST** (non-grav/relativistic terms off) plus REBOUND/IAS15 invariant + encounter-sensitivity checks
- b-plane geometry + **gravitationally-focused capture-radius** hit test
- Fixed-cadence clock with snapshot/interpolation; time slider / play / time-warp
- One method: kinetic impactor, parameterized by Δv (with β factor), impulse as a vector
- Apply Δv at a chosen lead time → re-propagate → **watch the hit become a miss** (Earth slides out of the way)
- **The payoff chart: required Δv vs. lead time** (headline curve fixes the impulse phase; a separate view exposes phase sensitivity)
- Soft-capped, kinetic-impactor-plausible impulse magnitudes; the **delivery** and **determinism** honesty caveats surfaced in UI copy (§1)

That MVP delivers the whole lesson *and* an honest hit→miss flip. Everything below is layering — mostly *toggling on force-model tiers* and swapping the renderer.

### Phase 2 — realism + real asteroids

- **Godot 3D view** (gdext): SubViewport composition (2D schematic/HUD over 3D, or vice versa); floating origin / double-precision as needed (§7)
- **Tier 2 force model**: enable 1PN relativity, Yarkovsky, SRP, J2, and the 16 asteroid perturbers (on top of the DE440/441 ephemeris perturber field already used in the MVP) — validated against **ASSIST**, then **Horizons** on real asteroids
- Real NEOs from the JPL Small-Body Database (§9): Apophis, Bennu, Didymos/Dimorphos
- Nuclear standoff + gravity-tractor methods
- Lambert / porkchop mission design (makes the impulse *deliverable*, not assumed)
- **Tier 3 uncertainty**: orbit covariance → b-plane → impact probability; keyholes; covariance ellipse shrinking with observations

### Phase 3 (future)

- Plausible launch vehicles + payload mass budgets
- Orbital assembly (assemble-in-orbit when payload too big for one launch)
- Standing/ready Earth-defense systems
- Multi-mission campaigns

---

## 9. Data sources & teaching asteroids

- **JPL Horizons** — state vectors; the ground-truth reference for *real* trajectories (Phase 2 / Tier 2 onward).
- **JPL Small-Body Database** — orbital elements **and covariances** for real NEOs; the covariance feeds Tier 3.
- **JPL DE440 / DE441** (via ANISE) — planetary/lunar ephemerides. DE440 = standard span; DE441 = long span (what ASSIST uses). GM constants pulled from ANISE even in the MVP.
- **ESA NEOCC** — secondary cross-reference (and the Aegis impact-monitoring system as a Tier-3 reference).

Teaching asteroids worth seeding (Phase 2):

- **Apophis** — the perfect teaching case: famous 2029 close approach and real keyhole history; also exercises relativity (low perihelion).
- **Didymos / Dimorphos** — the DART target; gives a real, measured β for free.
- **Bennu** — well-characterized (OSIRIS-REx); the canonical Yarkovsky case.

---

## 10. First tasks for Claude Code

Re-sequenced for the pure-Rust / honest-hit-miss MVP. The encounter (ephemeris test-particle + ASSIST validation) is now **on the MVP critical path**, not a late add — which is why the task-0.5 build spike (step 2) comes first.

1. **Scaffold the Cargo workspace:** `core/` (no renderer dep), `viewer/` (pure-Rust, **egui**), `validation/`, `pyref/`. *(No `godot/` yet — Phase 2.)* Wire **ANISE + a DE440 (or DE441) kernel** loading early — the test-particle MVP needs perturber positions, not just GM constants, before first-light.
2. **Task-0.5 de-risk spike — do this before the rest of the plan leans on it.** Confirm the two pillars Option A rests on: (a) **ASSIST + DE441 actually build** offline in `pyref/` and can integrate a test particle; (b) the **ANISE DE-position reader** returns a sane reconstructed **geocenter** (not the EMB) for a known epoch. **Fallback-to-Option-B trigger:** if ASSIST won't build or the DE-position reader stalls, fall back to a self-consistent N-body MVP validated against REBOUND and revisit the ephemeris-perturber architecture at Tier 2. (Under Option A you may *demo* the hit→miss flip before ASSIST validation completes — but REBOUND cannot stand in as the trajectory oracle, since it self-gravitates the planets.)
3. **Implement `Epoch` (hifitime), `StateVector`, `OrbitalElements`, and element↔state conversions** — with `proptest` coverage that **explicitly targets e→0 and i→0** singularities (random orbits miss them).
4. **Implement the analytic Kepler propagator** behind the `Propagator` trait.
5. **Wire the free-invariant property tests** (energy / angular momentum / LRL / forward-back reversibility) **with per-propagator expectations** (analytic → machine precision; later RK4 → error-growth rate, not conservation). At this step they validate the *conversions*, nothing more — don't over-read green.
6. **Stand up one `pyref/` fixture** (propagate a known orbit via hapsira, commit reference states as JSON) and the matching Rust test in `validation/`. Pin μ/frame/time-scale identically; pull GM through ANISE on the Rust side.
7. **Build the composable `ForceModel`** (Σ of toggleable terms; `point_mass.rs` takes an arbitrary perturber list) and the integrators behind the `Integrator` trait: **RK4 first** (to exercise the invariant tests), **then dop853 as the MVP encounter integrator** (IAS15-style is a Tier-2 long-arc upgrade). Integrate in the **barycentric ICRF** frame. Then the **Tier-1 force model** — asteroid as a test particle under Sun + planets + Moon point masses, positions from DE440/441 via ANISE — **validated against ASSIST** (non-grav/relativistic terms off on both sides), with **REBOUND/IAS15** used for the free-invariant and encounter-sensitivity cross-checks. Unit-validate each GR/J2/Yarkovsky term in isolation (Mercury precession, etc.) as it's added.
8. **Implement b-plane geometry + the gravitationally-focused capture-radius hit test** — turns the encounter into a hit/miss answer and underpins the Δv-vs-lead-time curve.
9. **Build the fixed-cadence `clock`** with a snapshot API whose sub-snapshot queries are served from the integrator's **dense output** (dop853's continuous extension), not linear interpolation — linear interp visibly lies through the high-curvature encounter.
10. **`viewer/` (egui):** the Δv-vs-lead-time chart (`egui_plot`; fixed-phase headline curve + a separate phase-sensitivity view) **and** the rewind → nudge → re-propagate → "Earth slides out of the way" animation (painter), rendered in a floating-origin frame for the encounter.

At that point the engine supports the full MVP scenario. Tier-2 realism and Tier-3 uncertainty then layer on as force-model toggles + the Godot frontend (Phase 2), largely in parallel.

---

## Open questions / deferred decisions

The first review and the follow-up discussion closed every major open question (see *Resolved* below). What remains is genuinely deferred to when the relevant tier arrives:

- **dop853 → IAS15 crossover (Tier 2).** dop853 is the MVP integrator; the lead time / orbit count at which IAS15's near-symplectic long-arc behavior actually wins is an empirical question — measure it against REBOUND when Tier-2 long arcs arrive.
- **Impulse soft-cap: hard gate vs. honest readout.** Whether the MVP forbids an over-budget nudge outright or allows it with an honest *"this would take N DART-class impactors"* label — a UX call to settle in implementation (§5).
- **SBDB covariance ingestion (Tier 3).** The on-disk format/units for real-asteroid orbit-determination covariances feeding `uncertainty.rs` — deferred until Tier 3.
- **Pluto in the shipping perturber field (raised by batch-2c ASSIST validation).** §5 locks the MVP set at "Sun + 8 planets + Moon" (10 bodies), but ASSIST — our declared config (§6) — sums **11** point masses including Pluto. The batch-2c test measured the cost of omitting Pluto at **~55 m over 2 years** for a main-belt test particle (growing with lead time, so plausibly ~km at the decade lead times the b-plane cares about). Adding Pluto to the shipping field also needs a Pluto GM source: `pck11.pca` carries **no BODY9_GM**, so it would require a DE441-consistent constants set or a hardcoded value. **Current default (proceeded on, re-askable): keep 10 bodies and fold Pluto in at Tier 2** alongside the 16 asteroid perturbers and the fuller GM source — the natural home for the perturber-set expansion. The 2c test validates the machinery at the full 11 bodies regardless (it adds Pluto to its *comparison* field). Flip to 11-in-shipping if the growing-with-lead-time cost proves to matter for the headline curve before Tier 2.

### Resolved by the 2026-06-23 review

*First pass:*
- ~~MVP renderer~~ → **pure Rust; Godot is Phase 2.** (§2, §8)
- ~~MVP planet positions: analytic Kepler vs DE440~~ → **MVP integrates the asteroid as a test particle in the DE440/441 ephemeris field (positions + GM from ANISE) from Tier 1. Tiers add force *terms*, never switch the perturber source — so ASSIST is the oracle from day one.** (§5, §6)
- ~~What "deterministic" means~~ → **same-build-same-output, not cross-machine bit-reproducibility.** (§2)
- ~~Fixed-timestep vs adaptive-integrator tension~~ → **fixed snapshot *cadence*, adaptive integration *step* between snapshots.** (§2)

*Second pass (same day):*
- ~~Pure-Rust renderer crate~~ → **egui is the spine** (egui_plot + painter); plotters optional later; macroquad's animation edge isn't worth its GUI cost (Godot covers game-like polish). (§2, §8)
- ~~Default encounter integrator~~ → **dop853 for the MVP** (sufficient, easier, dense output for the clock); IAS15 is a Tier-2 long-arc upgrade. (§5, §7)
- ~~Impulse application phase~~ → **fix it for the headline curve; expose phase as a separate view.** (§5, §7)
- ~~MVP perturber set~~ → **Sun + 8 planets + Moon, with the force term and ANISE loader designed to add the 16 asteroid perturbers at Tier 2.** (§5)
- ~~Godot precision (Phase 2)~~ → **floating-origin first; double-precision build only as a fallback.** (§7)
- ~~Integration frame (was implicit)~~ → **barycentric (SSB) ICRF, SI units, present heliocentric** — dodges the non-inertial indirect-term footgun. (§2, §5)
- ~~Float-precision worry for the core (was conflated with f32)~~ → **f64 retires it; it's a rendering-only concern.** (§2, §7)
- ~~Clock sub-snapshot interpolation~~ → **served from dop853 dense output, not linear interp.** (§4, §7, §10)
- ~~Scenario/fixture format~~ → **JSON** (crosses the Python boundary natively); RON optional for Rust-only authoring. (§6)
- Added: **task-0.5 ASSIST/DE build de-risk spike + fallback-to-B trigger** (§10); **delivery + determinism honesty caveats** in UI copy (§1); **impulse soft-cap** to kinetic-impactor plausibility (§5).
