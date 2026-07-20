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

### The gotcha that makes the whole suite lie (read this before trusting a green run)

**`cargo test` without `ASTEROID_DE_KERNEL` + `ASTEROID_PLANETARY_CONSTANTS` set silently skips every kernel-gated test and reports them as passed.** Roughly half this project's physics tests are kernel-gated. They open with `if !have_kernels() { eprintln!("skipping…"); return; }` — deliberate, so a kernel-less CI stays green (kernels are 32 MB–646 MB and are not in the repo). The trap is not the skip; it is that **the skip is invisible**:

- The `eprintln!` notice is **swallowed by cargo's output capture**, which only releases stderr for *failing* tests. A passing skip prints nothing. `--nocapture` shows it; nobody runs `--nocapture` on a green suite.
- What you see is `test result: ok. 13 passed; 0 failed`. That is indistinguishable from a real pass.
- **The runtime is the only tell.** Kernel-less: `13 passed … finished in 0.02s`. Kernels mounted: `13 passed … finished in 69.01s`. Real DE440 integration cannot happen in 20 ms. If a physics suite finishes in under a second, **it did nothing**.

This bit for real on 2026-07-17 and cost the session's whole verification story twice over: a `deflected_b_point_km` fix was "confirmed" by a test that never executed, and `frame_from_arcs_matches_frame_from` — the *only* proof that splitting `frame_from` didn't change its output — had never once run. Both were genuinely green when re-run properly, but that was luck, not verification. Note the shape of the failure: the machine **had** the kernels, sitting in the conventional directory. Only the env vars were unset.

#### Fixed 2026-07-19 — `core::kernels`, and how to run the suite now

The GDScript suites never shared this hazard: `Kernels.resolve()` (`godot/scripts/kernels.gd`) falls back from env → `user://kernels.cfg` → conventional dirs, so `test_orrery` runs real physics either way. That asymmetry was the hint at the fix. `core/src/kernels.rs` is now the Rust mirror of it, and every kernel-gated site in the workspace (core, `validation`, the gdext binding, the examples) goes through it:

```sh
ASTEROID_REQUIRE_KERNELS=1 cargo test --workspace --release   # green here MEANS it ran
```

Two distinct failures needed two distinct fixes, and this is the part worth keeping straight:

- **`kernels::resolve()`** — env → conventional dirs, both-or-nothing — cures *"I have the kernels but didn't point at them"*. That was the actual 2026-07-17 failure. Env vars are no longer needed on a machine that has the kernels in `../temp/AsteroidDefense/kernels` (or `<repo>/kernels`, or beside the exe).
- **`ASTEROID_REQUIRE_KERNELS`** turns "nothing resolved" from a silent skip into a **panic** naming the test that would have lied and every path searched. Resolution alone would have cured only *this* box *today*: a fresh clone, a CI container, or a renamed directory puts the silent-green failure straight back. Unset, the skip is still green — offline CI is preserved on purpose.

**The gate was proved by bypassing it**, not by watching it pass: with `../temp/AsteroidDefense/kernels` renamed away, `ASTEROID_REQUIRE_KERNELS=1` makes the kernel-gated tests **FAIL** loudly, and unset it reproduces the original lie exactly — *the same* `81 passed` / `13 passed`, but `0.09s` and `0.00s` instead of `18.03s` and `56.38s`. The counts are indistinguishable; the clock is the whole signal. That bypass is also what confirmed `tier1_field_matches_assist` genuinely runs in 0.05 s (it fails the moment the kernels vanish) rather than being one more silent skip.

`user://kernels.cfg` is deliberately *not* read by the Rust side — `user://` resolves through Godot's own per-platform app-data path, and reconstructing that in Rust to read a file the frontend wrote would be a guess that rots silently. The directory scan covers the same case, and callers that know better still pass explicit paths (`MissionCore::load_from`).

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
- **b-vector sign convention + ξ,ζ decomposition (raised by step-8 b-plane geometry).** `geometry.rs` ships the b-plane hit test and the b-vector `B` with its *magnitude* pinned (`|B| = b`) and its plane pinned (`B ⊥ Ŝ`, `B ⊥ ĥ`), but its **sign** deliberately unasserted, and the Öpik/Kizner **ξ,ζ decomposition** — which needs an external reference direction (Earth's heliocentric velocity, or an ecliptic pole) — deferred to Tier 3 (`uncertainty.rs`), since that is the layer (keyholes/covariance) that actually reasons in b-plane coordinates. Nail the sign + reference frame when keyhole geometry needs it. **Phase-2 3C-2c coexists with this rather than forcing it:** the Godot b-plane view builds its *display* axes from `Ŝ` and the ecliptic pole in the binding (not core), labels them as display axes, and prints only rotation-invariant scalars (`|B|`, perigee, capture radius, `v_inf`) — so nothing on screen depends on the unpinned convention, and settling it later is still free.
- **Pluto in the shipping perturber field (raised by batch-2c ASSIST validation).** §5 locks the MVP set at "Sun + 8 planets + Moon" (10 bodies), but ASSIST — our declared config (§6) — sums **11** point masses including Pluto. The batch-2c test measured the cost of omitting Pluto at **~55 m over 2 years** for a main-belt test particle (growing with lead time, so plausibly ~km at the decade lead times the b-plane cares about). Adding Pluto to the shipping field also needs a Pluto GM source: `pck11.pca` carries **no BODY9_GM**, so it would require a DE441-consistent constants set or a hardcoded value. **Current default (proceeded on, re-askable): keep 10 bodies and fold Pluto in at Tier 2** alongside the 16 asteroid perturbers and the fuller GM source — the natural home for the perturber-set expansion. The 2c test validates the machinery at the full 11 bodies regardless (it adds Pluto to its *comparison* field). Flip to 11-in-shipping if the growing-with-lead-time cost proves to matter for the headline curve before Tier 2.

### Tier 2 begun — 2026-07-20 session (1PN relativity + Yarkovsky terms)

- **1PN relativity Sun term shipped and validated in isolation** (`core/src/forces/relativity.rs`). The first Tier-2 force: the PPN Schwarzschild acceleration of a test particle in the Sun's field at `β = γ = 1`, `a = μ/(c²r³)·[(4μ/r − v²)r + 4(r·v)v]` with `r, v` heliocentric. Fits the composable [`ForceModel`] sum with **zero structural change** — it is one more `.with(...)` term (§5). `c = 299 792 458` m/s exact; `μ` is a field passed in (the tests use the DE `1.327 124 400 18e20`, production must hand it the **same** ANISE-loaded `μ_sun` the point-mass Sun term uses — a second hardcoded μ would be a silent bias). Needs the Sun's full **state** (position + velocity), so it gets its own `CentralBodyState` provider rather than a `velocity_at` bolt-on to `PerturberEphemeris` (position-only); `FixedCentralBody::at_rest_origin()` keeps the isolation test kernel-free.
- **Validated by Mercury's perihelion precession, the §6 isolation check** — the term alone reproduces `Δϖ = 6πμ/(c²a(1−e²))`/orbit. Guards the advisor flagged as load-bearing, all built in from the first run: (1) the signal is compared to the closed form computed with the **same** constants, not to a literal 42.98″; (2) a **Newtonian-only control run** (1PN off) confirms the measured precession is physics, not integrator LRL drift — control ≪ signal; (3) measured by **stroboscopic** eccentricity-vector sampling (once per period) + a least-squares slope over 40 orbits, not one-orbit differencing; (4) an explicit **prograde sign** assertion (the classic `(r·v)v` sign bug's tell). Signal matches the closed form to <2% and lands in 40–46″/century; kernel-free so it actually runs (unlike a silently-skipped ANISE test). Full core suite 97 passed / 0 failed in **18.31 s** with `ASTEROID_REQUIRE_KERNELS=1` (the runtime that proves the kernel-gated half executed).
- **Yarkovsky thermal-recoil term shipped and validated in isolation** (`core/src/forces/yarkovsky.rs`). The decade-scale along-track *dominator* (§272) and the term that actually earns real-NEO Horizons validation — J2 of the Sun is negligible heliocentrically, so Yarkovsky came before it. Uses JPL Sentry's **transverse `A2` parametrization** (Farnocchia/Vokrouhlický), not a full thermophysical model: `a = A2·(r₀/r)^d·t̂` with `t̂ = ĥ×r̂` the prograde in-plane direction, `r₀ = 1 AU`, `d = 2`. `A2` carries the drift sign (`A2>0` prograde → outward `da/dt`; `A2<0` retrograde, Bennu-like → inward). Reuses the 1PN commit's `CentralBodyState` provider (heliocentric `r, v`); another `.with(...)` term, zero structural change.
- **Validated by the secular semi-major-axis drift, `⟨da/dt⟩ = 2·A2·r₀²/(n·a²(1−e²))` (d=2), the §6 isolation check.** The advisor's make-or-break was the oracle's **time weighting**: the Gauss `da/dt` integrand goes as `(1+e·cosν)³`, so a uniform-in-true-anomaly average is ~10% wrong at e≈0.2. Fixed by sampling the oracle uniformly in **mean anomaly** (= uniform in time), and cross-checked two ways — the numerical uniform-M average agrees with the closed form to <1e-4 across e=0/0.2/0.45 (`oracle_time_average_matches_the_closed_form`), and the integration-measured drift matches the **time-averaged** oracle to <1% at e=0.2 (a uniform-ν oracle would be ~10% off and fail that tolerance — the test discriminates). Same guard structure as 1PN: a circular-orbit de-risk case (e=0, no weighting ambiguity), an `A2=0` **control run** (drift ≪ signal → physics not integrator noise), an explicit prograde/retrograde **sign** pair, and an algebraic acceleration test pinning `a·r̂=0`, `|a|=A2(r₀/r)²`, and direction `ĥ×r̂` **not** `v̂` (the common wrong impl). `A2` amplified above Bennu's physical ~1e-13 m/s² for SNR (legitimate — validates form/sign/units, not magnitude — and stays linear, Δa ≪ a). Bennu numeric anchor deliberately **dropped** rather than recalled from memory (the algebraic test already guards units). Kernel-free; full core suite 104 passed / 0 in **18.57 s** under `ASTEROID_REQUIRE_KERNELS=1`.
- **Open / next:** both terms are validated in isolation but **not yet wired into the shipping scenario** — the deliberate next commit (§206: validate per term first). Wiring = add 1PN + Yarkovsky to the real-field force model behind toggles, then re-check the threat b-plane is *unchanged* with them off and *shifts* with them on. Then SRP, J2, the 16 asteroid perturbers as force terms, and Pluto-in-shipping — the rest of the Tier-2 body of work — before a real NEO's *own* integration can be validated against Horizons (which needs both GR and Yarkovsky to match).

### Resolved by the 2026-07-20 session (Phase-2 3D, real bodies — Horizons NEO half)

- ~~"the Horizons per-object NEO SPKs reuse the identical read path"~~ → **they cannot; ANISE can't read them.** The plan of record (the sb441 note directly below) assumed a Horizons SPK would mount beside `sb441-n16.bsp` and read like any other body. It does not, and this was measured before any plumbing was written (`core/examples/probe_horizons.rs`, the gate that decided the whole approach): `sb441-n16.bsp` is **SPK type 2** (Chebyshev), a Horizons per-object SPK is **SPK type 21** (extended modified difference arrays), and **ANISE 0.10.3 has no type-21 evaluator** — it dispatches types 1/2/3/8/9/12/13 and returns `Type21ExtendedModifiedDifferenceArray not supported for SPK computations` for 21. No request parameter changes the type Horizons emits. "Same read path" was true of the call site and false of the decoder underneath it.
- **Chosen (advisor-gated): Horizons VECTORS → in-project sampled trajectory + cubic Hermite.** Ask Horizons for the same trajectory as *states* (position+velocity, `EPHEM_TYPE=VECTORS`, heliocentric `CENTER='500@10'`, `REF_PLANE=FRAME` ICRF, `OUT_UNITS=KM-S`) on a fixed 1-day TDB cadence, and interpolate between them. **The honesty property is preserved and it is the whole point:** the states are JPL's own relativistic solution either way, so this interpolates JPL's numbers rather than integrating our own worse ones. That distinction is exactly what separated it from the two rejected branches — (a) integrating a single state vector in our field re-litigates the deleted display-grade Kepler and hits the Tier-2 1PN trap to produce a *worse* trajectory than JPL already published; (b) implementing type 21 in a forked ANISE is variable-`MAXTRM` binary-record parsing, a real upstream contribution and a scope decision the user should own, not a default. **These NEOs are scenery, never the threat and never a deflection target.**
- **Cubic Hermite, because the table carries velocity.** The interpolant matches JPL's position *and* derivative at every node, so the drawn arc is tangent to the real trajectory, not merely near it. Accuracy is **measured, not asserted by eye** — and the measurement caught a real thing: `hermite_matches_held_out_horizons_states` decimates Apophis and reconstructs held-out samples, and the *median* converges cleanly (~12×/halving, fourth-order-ish) while the *worst case* barely moves and always lands at the **2029 Earth flyby**, whose hours-long curvature no daily table resolves. So what actually ships is measured directly in `shipped_cadence_error_across_the_2029_flyby` against a committed **hourly** fixture: **median 24 m, worst 18 885 km at the flyby** — 1.3×10⁻⁴ AU, a fraction of a pixel at orrery scale. Both flyby fixtures (`core/tests/fixtures/apophis_flyby_{1d,1h}.neo`, ~173 KB) are committed, so this one accuracy check runs on a fresh clone with **no kernels and no fetch**.
- **The data is a plain-text state table, not JSON, not a kernel.** `asteroid_core` depends on anise/hifitime/nalgebra and nothing else — serde is deliberately validation-and-viewer-only. So `.neo` files are a key/value header + one whitespace-separated state per line (floats via Python `repr`, shortest round-trip), parsed dependency-free in `core/src/horizons.rs`. Magic line `asteroid-neo-states 1`; a file that fails its declared-vs-actual sample count, frame (`SUN`/`ICRF_J2000`), or magic is a **hard error**, because a truncated download is otherwise indistinguishable from a legitimately short span. Tables live under `<kernels>/neo/*.neo`, gitignored and regenerable (`python pyref/fetch_horizons_neo.py`), absent on a fresh clone — everything works without them, the asteroids simply do not appear. Resolver + skip-loud test harness (`horizons::resolve_dir`/`load_all`/`load_all_for_test`) mirror `kernels.rs`.
- **NAIF numbering, the trap the sb441 note flagged in advance.** Horizons uses the **extended** small-body convention `20000000 + number`, so Apophis is **20099942**, verified by enumerating a fetched SPK's segment table — *not* sb441's `2000000 + number` (Ceres = 2000001). A digit apart; the wrong one is a lookup failure that looks like anything but a typo. Recorded as provenance only — the sampled read path never resolves it, since the almanac cannot answer for these objects at all.
- **The catalog now mixes provenance, and says so.** `OrreryBody` carries `Trajectory::{Integrated(Clock), Sampled(Neo)}` — the comet is *our* physics in *our* field (SSB metres), the NEOs are *JPL's* interpolated (heliocentric ICRF metres). The two frames differ by the Sun's barycentric wobble (~10⁶ km, "looks like a rendering nudge"), reconciled in the **single** `catalog_body_helio_ecl_au`. `catalog_provenance(i)` returns `"integrated"`/`"sampled"` and the frontend labels bodies with it — because a trajectory drawn beside real physics with nothing marking which is which is the exact mistake the deleted GDScript Kepler was.
- **ZERO-is-the-Sun, fifth instance.** A `.neo` table covers 2020–2070 against a clock that scrubs the DE kernel's ~300 years, so most of the range is *outside* it. `Neo::helio_state_at` returns `None` (never a zeroed vector) outside its span, per-body through `catalog_active`/`catalog_span_tdb`. `catalog_active` used to require the single `comet_online` flag — correct for one body, wrong the moment the catalog held four, since Apophis's table and the comet's arc cover different years and one flag cannot answer for both.
- **The threat is untouched, structurally.** A sampled NEO never reaches the almanac (it is a state table, not a kernel), carries no GM, and cannot enter `tier1_perturber_field`. So "mounting real asteroids cannot perturb the threat" is a guarantee, not a hope, pinned two ways: `neo_bodies_cannot_reach_the_force_model` (core, compile-time) and `real_asteroids_join_the_catalog_without_touching_the_threat` (binding) — one build, threat cap/perigee/impact read before and after the NEOs install, compared with `==` not a tolerance. Cap stayed 11 311 km, |B| 14 639 km, to the digit.
- **Orbit lines draw one lap, not fifty.** A NEO's table is decades but its orbit is ~a year, so a polyline over the whole span is dozens of precessing laps overplotted into noise (the comet escaped this only because its span *is* one authored period). `Neo::orbital_period_seconds` (vis-viva, the same "period-to-bound-the-window" move the `ephem` orbit path already makes — every drawn point is still a real state read) feeds `catalog_track_window_ecl_au`, which samples one period clamped inside the span.
- **Verified by picture.** `_shot.gd` gained `neo_1_on_arc` (scrubbed to the 2029 flyby: Apophis/Bennu/Didymos named, cyan, at 1.0–1.3 AU near Earth, one clean elliptical lap each, distinct from the amber belt) and `neo_2_past_span_gone` (2071, past the 2070 table end: all three absent, no orbit lines, nothing on the Sun). 82/82 GDScript assertions pass, including the per-body span gate and provenance checks.
- **Incidental: the debug-mount cost is fine.** The sb441 half left "`mission_online` 11 s → 34 s" open; the debug DLL now rebuilds and loads in ~20 s. Not chased further — no longer painful.
- **Open / next:** these three are the §9 teaching asteroids on-screen but **not yet validated against Horizons** — that is Tier 2 (§5/§8), which needs 1PN relativity + Yarkovsky before our *own* integration of a real NEO could match JPL. The display is honest today precisely because it does not integrate them. Yarkovsky/SRP/J2 and the fuller force model remain the Tier-2 body of work; Pluto-in-shipping (below) folds in there.

### Resolved by the 2026-07-20 session (Phase-2 3D, real bodies — sb441 half)

- ~~"the real-NEO half reads real NEOs out of `sb441-n16.bsp`"~~ → **it cannot; that file has no NEOs in it.** The plan of record rested on a factual error, caught by enumerating the kernel's SPK segment table directly rather than trusting the note. `sb441-n16.bsp` contains exactly **16 main-belt perturbers** — Ceres, Pallas, Juno, Vesta, Iris, Hygiea, Eunomia, Psyche, Euphrosyne, Europa, Cybele, Sylvia, Thisbe, Camilla, Davida, Interamnia — all Sun-centered (NAIF 10), 4 segments each, spanning 1550–2650. It is the **perturber set ASSIST integrates against**, not a target list: sub-km teaching NEOs like Apophis or Bennu would never appear in one. The §9 teaching asteroids are a *different* acquisition problem, and the split below is how it was taken.
- **Scope split (user call): sb441 now, Horizons NEOs next.** This commit builds the kernel-mounting plumbing against sb441 — real bodies on screen, zero network fetch — and the Horizons per-object NEO SPKs reuse the identical read path in a follow-up. Same end state, two commits, and the risky half (does mounting a third kernel work at all, and where does the cost land) is settled first against a file already on disk.
- **Why Horizons SPKs and not SBDB elements + integrate, for the NEOs.** Integrating a real NEO from published elements walks straight into the Tier-2 **1PN relativity** trap already flagged at §270: low-perihelion objects do not match JPL without the Sun's relativistic correction, and omitting it makes Horizons validation *silently* fail. A Horizons per-object SPK is JPL's own already-relativistically-integrated trajectory, so reading it sidesteps the question entirely — the teaching asteroids arrive correct before the force model is ready to earn them.
- ~~Where the small-body mount lives~~ → **on the existing build worker**, decided by measurement rather than taste, exactly as the comet's placement was. `sb441-n16.bsp` is 646 MB and mounting it costs **~5.7 s cold / ~272 ms warm** (release) — the gap is page-cache I/O, so a freshly launched game pays the full cost. `MissionCore::load_from` is contractually fast (~ms) and sits on the path to the first drawn frame, so mounting there would have traded a working 3-second startup for a frozen 9-second one. Per-query cost is negligible (~3.5–6 µs), so scrub reads are free once mounted.
  - The worker **cannot** mount onto the almanac it is handed: `Ephemeris::with_constants` consumes `self`, and the served `Arc<Ephemeris>` is being read by the render thread every frame. So it builds a *second* almanac from paths (`mount_small_bodies`, re-reading de440s at ~ms) and returns it inside `BuiltScenario`; `install` adopts it. The serving core never moves and is never mutated — the invariant the whole worker design exists to protect — and the scenario is served from the same field it was flown in.
  - A mount failure **warns and continues**. The mission is complete and correct without asteroids; taking the build down over scenery would trade a missing catalog for a missing threat.
- **The optional third kernel.** `KernelPair` gained `small_bodies: Option<PathBuf>` (plus `ASTEROID_SMALL_BODY_KERNEL`, and the GDScript mirror in `kernels.gd`), deliberately **outside** the both-or-nothing rule that governs `bsp` + `pca`: the file is twenty times the DE kernel and a fresh clone will not have it. Absent → `None` → no asteroids, everything else unchanged. Failing a *pair* over it would take the planets down on every machine without 646 MB to spare. Pinned by `small_body_kernel_is_optional`, and the test was checked by bypassing the resolver to fabricate a path and watching it fail.
- **ZERO-is-the-Sun, fourth instance — gated before it could ship.** These are `"ephem"` bodies on the planets' read path, so an unmounted lookup fails, and a failed heliocentric lookup drawn anyway is a body **on the Sun**. Two flags, not one: `small_bodies_armed` (a path was handed over) is *not* `small_bodies_mounted()` (the served almanac actually has it), and between those two states every lookup fails. Only the second gates a draw; `small_body_count()` also returns 0 when unmounted, so a caller that ignores the flag iterates nothing rather than sixteen bodies stacked on the Sun.
- **Verified by picture and by number.** `_shot.gd` gained a `belt_1_real_asteroids` section: all 16 report `armed=true mounted=true`, resolve at real main-belt distances (2.17–3.79 AU) with spread, non-zero node positions. The id table itself was checked by corrupting one entry (2000704 → 2000705) and watching the kernel reject it. **Not yet verified: individual visual identification** — at `vis_r` 0.020 under the green phosphor shader the sixteen are not distinguishable from the scenery belt's 1600 dust points in a wide shot. They are drawn as bodies rather than dust on purpose (that belt is a seeded RNG annulus spun rigidly; these are per-frame kernel reads), and making that distinction *legible* is open work.
- **Open: the debug-build mount cost.** `mission_online` went from ~11 s to **34 s** in the debug DLL Godot loads. The 5.7 s measurement was release; ANISE parsing 646 MB unoptimized is far slower. `profile-dev opt-level=3` is already applied to `asteroid_core` for exactly this class of problem — extending it to cover the mount path is the obvious next move if the editor loop gets painful.

### Resolved by the 2026-07-20 session (Phase-2 3D, comet)

- ~~Where a synthetic orrery body's integration runs~~ → **on the existing build worker**, handed back with the scenario. `add_synthetic_body` is inline-and-expensive by design, and the measurement is why this was not a coin flip: the display comet costs **2.0 s over 12 yr / 8.1 s over 45 yr**, against a `build_scenario` of 11.2 s — so an inline call at install would have put multi-second stalls back on the render thread the worker exists to keep free. The seed math moved into a free `seed_orrery_body(&Arc<Ephemeris>, &RealFieldScenario, …)` that the worker and `add_synthetic_body` both call, so the two paths cannot drift; `install` now takes `(BuiltScenario, Vec<OrreryBody>)` because a new scenario invalidates the old catalog anyway (the bodies were flown in the old field). Span shipped at **one orbit ≈ 22.6 yr (~4 s)** — a second lap retraces the same arc for another ~4 s of build.
- ~~Whether GDScript keeps a Kepler propagator for "cosmetic context orbits"~~ → **no; it is gone.** The comet was the last user of `_elements`/`_kepler_pos_ecl`/`solve_kepler` and the Kepler fallback branches in `pos_ecl`/`orbit_points` — all deleted, mirroring 3C-2b's deletion of the threat's f64 Kepler block. The §5 Tier-0 tier still exists as a *concept*, but nothing in the Godot frontend draws from it: every drawn body now names a real source (`ephem` / `threat` / `threat_defl` / `catalog`). The fallback that used to run Kepler now `push_error`s instead of returning `Vector3.ZERO`, because ZERO in this heliocentric frame is the Sun — silently parking an unknown body on the Sun is the failure mode this whole seam is built against.
- **The ZERO-is-the-Sun trap, third instance** — and the first one caught *before* shipping rather than after. `catalog_position_ecl_au` returns `Vector3::ZERO` outside a body's propagated span, exactly like the planets (kernel coverage) and the threat (its ~12 yr arc) before it. The comet's one-orbit arc covers under a tenth of the ~300 yr scrubbable clock, so an ungated comet would sit on the Sun for most of the timeline. Gated per-body by `Sim.catalog_active(el, t)` off `catalog_span_tdb`, and **verified by picture, not by assertion alone**: `_shot.gd` shots `comet_1_on_arc` (inbound at 4.4 AU, tagged) and `comet_2_past_span_gone` (2051 — comet absent, planets untouched, nothing on the Sun).

### Resolved by the 2026-07-17 session (Phase-2 3C-2c)

- ~~Which pair decides hit-vs-miss on the display~~ → **`b` vs `b_capture`** — the core's own `is_hit`, with the focused capture disc (1.773 R⊕ at this encounter's `v_inf` ≈ 7.63 km/s) kept as the headline bar the planner and the b-plane view both measure against. §5's two criteria are equivalent *as pairs* — `b > b_capture` (the un-focused asymptotic miss against the enlarged target) ⟺ `perigee > R⊕` (the already-focused closest approach against the solid body) — and `geometry.rs` proves it. **Mixing them is what shipped**: `sim.gd` compared a *perigee* against the *capture radius*, charging for gravitational focusing twice and demanding ~1.5× more miss than physics does. Measured on a reachable plan (0.2 m/s at one period of lead): `b` = 14 640 km clears the 11 311 km disc by real daylight, while its perigee of 9 319 km sits inside it — so the display called a working deflection `SURFACE IMPACT`. Both quantities are "miss distances" in km, which is exactly why it survived. Now pinned in the binding against `is_hit` (both pairs, on the real perturbed field — the first check the two-body equivalence survives it) and at the GDScript level on the disagreement band itself, so the mixed bar cannot come back quietly. The displayed `PROJ MISS` became `b` alongside it: a player reads it against `CAPTURE` on the next line, so it must be the same pair.

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
