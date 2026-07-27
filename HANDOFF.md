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

### The gotcha with exactly the same shape, on the frontend (2026-07-27)

**Godot loads `target/debug/`, so a `--release` build leaves the frontend running old physics — with no error and no warning.** `godot/asteroid.gdextension` maps `windows.debug.x86_64` to `res://../target/debug/asteroid_gdext.dll` and `windows.release.x86_64` to the release one. The Godot *editor* and the ordinary `godot` binary are debug builds, so **they load the debug DLL** — while the entire Rust test loop (`cargo test --release`, `cargo build --release`) writes only the release one.

The failure mode is the point: a `#[func]` added to `lib.rs` and confirmed by a green release suite simply does not exist as far as GDScript is concerned. What you get is

```
SCRIPT ERROR: Invalid call. Nonexistent function 'tractor_defaults' in base 'Mission'.
```

which reads like a typo or a binding-registration problem, not like a stale artifact — and the DLL timestamps are the only tell (`target/debug` seven hours older than `target/release`). Same class as the kernel trap above: a green-looking run that is not testing what you think.

**After any Rust change, build both before touching the frontend:**

```sh
cd godot/rust && cargo build && cargo build --release
```

`class_name` is a second, independent staleness: a newly added `class_name` (e.g. `TractorPanel`) is not visible to other scripts until Godot rescans, so `main.gd` fails to parse with *"Could not find type"* while the file is plainly there. `godot --headless --editor --quit --path godot` rebuilds `global_script_class_cache.cfg`. Both of these cost an hour on 2026-07-27 and neither is discoverable from the error text.

---

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
- Nuclear standoff + gravity-tractor methods — **DONE 2026-07-27**: the standoff term as an impulse sibling of the kinetic model, the **gravity tractor** as a windowed `forces/` term with its own duration solve. §5's spectrum is closed. The tractor also has a **frontend** — the `[K]` bench, six live knobs over a cheap model scored against the real field, with an on-demand full-field probe on `[E]`. The nuclear half remains core-only. See *The deflection spectrum, nuclear half*, *…tractor half*, and *The tractor on the frontend*.
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
- ~~**Pluto in the shipping perturber field (raised by batch-2c ASSIST validation).**~~ **CLOSED 2026-07-27 — measured at 0.6 m, shipping field stays at ten bodies.** Both halves of the blocker resolved: the missing GM was real (`pck11.pca` genuinely resolves no Pluto GM — probed, not assumed) and the DE440 header supplies one (`GM9` → 975.500 km³/s²); and the *cost* is now measured rather than extrapolated. §5's own criterion was "flip to 11 if the growing-with-lead-time cost proves to matter"; at the campaign's real ~12 yr lead Pluto moves the b-plane perigee by **0.0006 km**, two orders below the belt's sub-km floor. The batch-2c ~55 m position figure did grow, but not into anything the b-plane resolves. Pluto ships as a `Tier2Config` toggle (off by default) so the comparison stays reproducible. See *The deferred leftovers, closed*.

### Tier 2 begun — 2026-07-20 session (1PN relativity + Yarkovsky terms)

- **1PN relativity Sun term shipped and validated in isolation** (`core/src/forces/relativity.rs`). The first Tier-2 force: the PPN Schwarzschild acceleration of a test particle in the Sun's field at `β = γ = 1`, `a = μ/(c²r³)·[(4μ/r − v²)r + 4(r·v)v]` with `r, v` heliocentric. Fits the composable [`ForceModel`] sum with **zero structural change** — it is one more `.with(...)` term (§5). `c = 299 792 458` m/s exact; `μ` is a field passed in (the tests use the DE `1.327 124 400 18e20`, production must hand it the **same** ANISE-loaded `μ_sun` the point-mass Sun term uses — a second hardcoded μ would be a silent bias). Needs the Sun's full **state** (position + velocity), so it gets its own `CentralBodyState` provider rather than a `velocity_at` bolt-on to `PerturberEphemeris` (position-only); `FixedCentralBody::at_rest_origin()` keeps the isolation test kernel-free.
- **Validated by Mercury's perihelion precession, the §6 isolation check** — the term alone reproduces `Δϖ = 6πμ/(c²a(1−e²))`/orbit. Guards the advisor flagged as load-bearing, all built in from the first run: (1) the signal is compared to the closed form computed with the **same** constants, not to a literal 42.98″; (2) a **Newtonian-only control run** (1PN off) confirms the measured precession is physics, not integrator LRL drift — control ≪ signal; (3) measured by **stroboscopic** eccentricity-vector sampling (once per period) + a least-squares slope over 40 orbits, not one-orbit differencing; (4) an explicit **prograde sign** assertion (the classic `(r·v)v` sign bug's tell). Signal matches the closed form to <2% and lands in 40–46″/century; kernel-free so it actually runs (unlike a silently-skipped ANISE test). Full core suite 97 passed / 0 failed in **18.31 s** with `ASTEROID_REQUIRE_KERNELS=1` (the runtime that proves the kernel-gated half executed).
- **Yarkovsky thermal-recoil term shipped and validated in isolation** (`core/src/forces/yarkovsky.rs`). The decade-scale along-track *dominator* (§272) and the term that actually earns real-NEO Horizons validation — J2 of the Sun is negligible heliocentrically, so Yarkovsky came before it. Uses JPL Sentry's **transverse `A2` parametrization** (Farnocchia/Vokrouhlický), not a full thermophysical model: `a = A2·(r₀/r)^d·t̂` with `t̂ = ĥ×r̂` the prograde in-plane direction, `r₀ = 1 AU`, `d = 2`. `A2` carries the drift sign (`A2>0` prograde → outward `da/dt`; `A2<0` retrograde, Bennu-like → inward). Reuses the 1PN commit's `CentralBodyState` provider (heliocentric `r, v`); another `.with(...)` term, zero structural change.
- **Validated by the secular semi-major-axis drift, `⟨da/dt⟩ = 2·A2·r₀²/(n·a²(1−e²))` (d=2), the §6 isolation check.** The advisor's make-or-break was the oracle's **time weighting**: the Gauss `da/dt` integrand goes as `(1+e·cosν)³`, so a uniform-in-true-anomaly average is ~10% wrong at e≈0.2. Fixed by sampling the oracle uniformly in **mean anomaly** (= uniform in time), and cross-checked two ways — the numerical uniform-M average agrees with the closed form to <1e-4 across e=0/0.2/0.45 (`oracle_time_average_matches_the_closed_form`), and the integration-measured drift matches the **time-averaged** oracle to <1% at e=0.2 (a uniform-ν oracle would be ~10% off and fail that tolerance — the test discriminates). Same guard structure as 1PN: a circular-orbit de-risk case (e=0, no weighting ambiguity), an `A2=0` **control run** (drift ≪ signal → physics not integrator noise), an explicit prograde/retrograde **sign** pair, and an algebraic acceleration test pinning `a·r̂=0`, `|a|=A2(r₀/r)²`, and direction `ĥ×r̂` **not** `v̂` (the common wrong impl). `A2` amplified above Bennu's physical ~1e-13 m/s² for SNR (legitimate — validates form/sign/units, not magnitude — and stays linear, Δa ≪ a). Bennu numeric anchor deliberately **dropped** rather than recalled from memory (the algebraic test already guards units). Kernel-free; full core suite 104 passed / 0 in **18.57 s** under `ASTEROID_REQUIRE_KERNELS=1`.
- **Both terms now WIRED into the shipping scenario behind toggles** (`core/src/scenario.rs`). `RealFieldScenario.force` is a [`CompositeForce`], not a bare `PointMassGravity`, built by the single `compose_force(eph, &Tier2Config)` helper both `build_with` and the new measurement path share — so "GR on"/"Yarkovsky on" cannot mean two different things. `Tier2Config { relativity: bool, yarkovsky_a2: Option<f64> }` hangs off `ImpactorConfig`, **all-off by `Default`** (every downstream builder passes `ImpactorConfig::default()`, so the shipping demo is untouched). The Sun's heliocentric `r,v` for both terms comes from an ephemeris-backed `CentralBodyState` impl on `EphemerisPerturber` (mirrors the existing `GeocentricState` impl — GR/Yarkovsky and the encounter geometry read *one* Sun), and 1PN's `μ_sun` is the same `eph.gm_km3_s2(SUN_J2000)·1e9` the point-mass Sun uses (never a second hardcoded constant).
- **Verification = the fixed-seed b-plane comparison (advisor-gated), not a rebuild.** Rebuilding with terms on would back-propagate the seed through the terms-on field and reproduce the hit *by construction* → zero visible shift. So `RealFieldScenario::nominal_encounter_with(&Tier2Config)` holds the built seed fixed and re-flies it through a differently-toggled field, attributing the perigee move to the physics. `tier2_terms_leave_the_bplane_unchanged_off_and_shift_it_on` asserts **structure, never a hand-derived magnitude**: (a) all-off re-fly == the shipping perigee *bit-for-bit* (the composite-with-one-term is `0 + a_pointmass`); (b) 1PN shifts the perigee by a resolvable amount and stays a hit; (c) a **physical, un-amplified** `A2 = 1e-13 m/s²` shifts it by some nonzero finite amount. **Measured:** 1PN moves perigee **3000.0 → 2944.5 km (−55.6 km)**, still well inside the 11 311 km capture (keyhole-precision territory, the reason GR matters for planetary defence); Yarkovsky at the physical A2 moves it **5.1 km** over the ~12 yr campaign — small but real, reported honestly rather than amplified into a lie. Full core suite **105 passed / 0** in 38.98 s under `ASTEROID_REQUIRE_KERNELS=1`; the gdext binding's 16 kernel-gated tests still read cap 11 311 km / |B| 14 639 km to the digit (the all-off bit-identity, confirmed downstream).
- **Open / next:** *(all resolved — the shipping demo still defaults `tier2` off, but the live frontend toggle, SRP, the 16 `sb441` perturbers, and the Horizons capstone all landed by 2026-07-21; see **Tier 2 complete** below.)* ~~Remaining: J2 and Pluto-in-shipping.~~ **Both closed 2026-07-27** — see *The deferred leftovers, closed*.

### Tier 2 continued — 2026-07-20 session (16 sb441 asteroid perturbers enrolled as forces)

- **The 16 `sb441` main-belt bodies promoted from scenery to force perturbers** (`core/src/perturber_field.rs`). `sb441_perturber_field(&Arc<Ephemeris>)` mirrors `tier1_perturber_field`: one `PointMassGravity` over 16 `EphemerisPerturber`s reading positions for NAIF ids `2000000+number` from a **mounted `sb441-n16.bsp`**. A third `.with(...)` term on the same [`CompositeForce`] sum, zero structural change (§5) — the exact expansion `point_mass.rs` was designed for since the MVP.
- **The masses are the load-bearing half; provenance is verbatim + machine-verified, not recalled.** `sb441-n16.bsp` carries **positions only** — ASSIST joins the GMs from the DE440/441 planetary file's own `MA%04d` constants (keyed by asteroid number), so each mass is the one JPL *integrated that position with*; any other value flies a perturber whose gravity disagrees with the trajectory it traces. The 16 GMs are transcribed **verbatim from the DE440 header GROUP 1041** (`MA0001…MA0704`, au³/day², D→e) into `SB441_PERTURBER_GM_AU3_DAY2`, and were **re-read straight out of the local `linux_p1550p2650.440` binary's constant record** (CVAL array at record-2 offset 8144, AU@CVAL[10]/DENUM=440 pinning the layout) to confirm the on-disk kernel carries these exact doubles. `#[allow(clippy::excessive_precision)]` keeps the header digits, same as `DE440_EMRAT`.
- **The unit/transcription guard, and the wrinkle that made it sharper.** GMs are **not** pulled from ANISE: the shipped `pck11.pca` resolves only **6 of 16**, and to a *different, later* mass solution. Cross-checking those 6 against the hardcoded DE440 set: the three **best-determined** (Ceres, Pallas, Vesta) agree to **<1%** (Vesta to ~4 sig figs — the au³/day²→km³/s² factor is right, since a wrong factor misses by orders of magnitude); the other three (Psyche, Europa, Davida) legitimately differ by **12–72%** because DE440 *free-fit* them where pck11 has spacecraft/occultation values — which is exactly why the self-consistent DE440 set is hardcoded rather than resolved. So the test (`sb441_field_builds_and_well_determined_gms_match_pck11`) asserts <1% on the three shared determinations only, and documents why the loosely-determined bodies are *not* checked against pck11.
- **Wired behind a toggle, all-off by `Default`, fail-loud on the missing kernel.** `Tier2Config` gained `asteroid_perturbers: bool`; `compose_force` adds the belt term when set. `sb441-n16.bsp` is the **optional 646 MB kernel** (outside the both-or-nothing rule), so: `RealFieldScenario::build` mounts it when the flag is set and errors if it is absent; `build_with` requires the caller to have chained it on; and `sb441_perturber_field` **probes every body's position up front** and returns a clear error naming the missing small-body kernel rather than failing deep in the first integration step ("an incomplete field is a wrong field", applied to positions). `sb441_field_without_the_small_body_kernel_fails_loud` pins it.
- **Verification = the same fixed-seed b-plane measurement as GR/Yarkovsky, and the capstone.** `asteroid_perturbers_leave_the_bplane_unchanged_off_and_shift_it_on` builds a **Tier-1 seed** on an sb441-mounted almanac, then re-flies it with the belt on: off == baseline bit-for-bit; on shifts the perigee by a nonzero finite **measured 0.552 km** over the ~12 yr campaign (3000.0 → 2999.5 km, still well inside the 11 311 km capture) — sub-km, the residual *floor*, reported honestly not amplified. The capstone (`capstone_neo_vs_horizons.rs`) gained a **+belt column** on the Apophis-vs-Horizons residual: the belt perturbs the trajectory at every epoch (**+0.07…+0.43 km** through year 7) but at the 8-year arc end sits **within the unmodelled radial-A1 floor** (Δ −0.037 km against the ~18.6 km GR+Yk residual) — it does **not** clear that floor, exactly as the sub-km wiring result predicts. The capstone asserts only that the perturbers *act* (nonzero finite, bounded), never that they help — measure-and-report, same discipline as Yarkovsky's below-floor early years.
- **Two 16-body lists, pinned together.** `gdext`'s display-scenery `SB441_BODIES` (id, name) and core's canonical force table `SB441_PERTURBER_GM_AU3_DAY2` are two spellings of the same sixteen; `scenery_and_force_perturber_lists_agree` (kernel-free, gdext) fails at `cargo test` if either drifts. `core/examples/probe_sb441.rs` kept as the provenance sibling of `probe_perturbers`/`probe_sun_gm` (it is the probe that measured 16/16 positions vs 6/16 GMs resolve).
- **Cost, as flagged.** The 16 extra ANISE lookups per RK step make the belt-on nominal propagation the heaviest test in the suite (~110 s for the two-propagation b-plane measurement); gated and default-off, so the shipping demo is untouched. Full core suite **109 lib + 1 capstone (24 s, ran not skipped) + 12 roundtrip, 0 failed** under `ASTEROID_REQUIRE_KERNELS=1`; core clippy clean; gdext drift guard green.
- **Open / next:** ~~a frontend toggle to show the belt shift live~~ and ~~SRP~~ both landed the following session — see *Tier 2 complete* below. ~~Remaining: J2 and Pluto-in-shipping.~~ **Both closed 2026-07-27** (`J2` shifts the perigee 1.33 km — more than this whole belt; Pluto 0.6 m). The residual floor is now GR-of-the-planets + JPL's radial A1, which we do not model.

### Tier 2 complete — 2026-07-21 session (SRP + the Apophis capstone + the live force-model menu)

Three commits close out the Tier-2 force menu the §5 spec asked for; the tree is clean and pushed.

- **Solar radiation pressure shipped and validated in isolation** (`core/src/forces/srp.rs`, commit `80498f7`). The *radial* sibling of Yarkovsky's transverse recoil: `a = a₁·(r₀/r)²·r̂` with `r₀ = 1 AU`, pushing directly away from the Sun. Constructed from physical inputs — `SolarRadiationPressure::from_physical(Cr, A/m)` — rather than a bare coefficient, so the term reads in the same units a real body's data comes in. Reuses the 1PN/Yarkovsky `CentralBodyState` provider for heliocentric `r`; one more `.with(...)` term on the same [`CompositeForce`], zero structural change (§5). **Validated by the effective-μ identity** — a pure `(1/r²)` radial push away from the Sun is indistinguishable from *weakening the Sun's gravity*, so the isolation test asserts a body under Sun-gravity + SRP orbits exactly as a body under a reduced `μ_eff = μ_sun·(1 − β)` — an algebraic invariant, not a hand-tuned number. Kernel-free.
- **The Apophis capstone: our *own* integration vs JPL Horizons** (`core/tests/capstone_neo_vs_horizons.rs`, commits `15a1a09` + `b91607e`). The payoff the whole force model was built to earn (§6 real-asteroid rung): integrate Apophis in our field and diff against its Horizons `.neo` truth table, **per force term**, GR measured not asserted. Results, honest: **1PN relativity cuts the residual 5–175×** across the arc (the low-perihelion body §167 predicted would need it); **Yarkovsky at Apophis's real `A2 = −2.902e-14 au/d²` roughly halves the year-8 residual** once the signal clears the model floor; the **belt** perturbs +0.07…+0.43 km through years 1–7 but at year 8 sits *within* the unmodelled radial-A1 floor (Δ −0.037 km) — it does not clear it, exactly as the sub-km wiring result predicted. The capstone asserts direction and bound, never a hand-derived magnitude, and **fails loud** (not skip-green) when the Apophis tables are absent — a tables-but-no-Apophis run must error, per `b91607e`.
- **The live force-model menu — see the shift on screen** (`godot/`, commits `1dc0646` + `7397869`). The frontend `[P]` menu toggles each Tier-2 term (`[G]` GR / `[Y]` Yarkovsky / `[A]` asteroid belt / `[S]` SRP) and re-solves the b-plane **on demand**, reporting each term's perigee shift live. Measured, all **inward**: GR **+55.55 km**, Yarkovsky **+5.10 km**, belt **+0.55 km**, SRP **+8.36 km**. On-demand, *not* on scenario build — chaining the terms into the build path was measured at >200 s and blocked the threat solution, so it pivoted to an Arc-shared scenario (gated behind a `RealFieldScenario: Sync` bound) driven over a second mpsc worker channel. Verified two ways: the `_shot.gd` harness drives the real keys and screenshots the shifted perigee, and an FFI gate pins the per-term deltas.
- **Where the force model stands.** The Tier-2 term list (§166) was complete but for **J2** and **Pluto-in-shipping**, both of which **landed 2026-07-27** — so the list is now closed: `J2` is validated against the closed-form nodal regression and shifts the perigee 1.33 km, and Pluto ships as a toggle measured at 0.6 m (the shipping field deliberately stays at ten bodies). Everything ASSIST carries, we carry, validated per-term against the closed form or Horizons. **The deflection-method spectrum (§5) closed on 2026-07-27 with the nuclear and tractor halves, so the one remaining spec beat is Tier 3 uncertainty (§175 — covariance → b-plane → impact probability, keyholes, and where the deferred b-vector sign/ξζ convention at the open-questions list finally gets settled).**

### Phase-2 mission design — 2026-07-21 session (Lambert + porkchop + launch vehicles, core)

The §8 "makes the impulse *deliverable*, not assumed" beat — the honesty gap §7/§180 keeps flagging. **User chose the fuller build on both open axes** (over the advisor's minimal-cut recommendation): couple the impulse *direction* to the real arrival geometry (not the idealized along-track push), and include real **launch vehicles** (bounded to single-launch / no orbital assembly — that stays Phase 3). The core layer is three kernel-free-where-possible modules; the Godot porkchop heatmap view is the remaining follow-up.

- **`core/src/lambert.rs` — the two-point transfer solver** (commit `ad00379`). Universal-variable (Bate/Mueller/White; Curtis Algorithm 5.2), single-rev short-way prograde first cut. Given `r1`, `r2`, `Δt`, `μ` → departure/arrival velocities. Two-body, and that is *correct* for the planning layer (a real cruise is two-body); it is **not** a display-grade shortcut — the honest-hit/miss physics stays in the full field, Lambert only sizes/aims the delivery. The **180° collinear singularity returns `DegenerateGeometry`** (a porkchop gap, never a NaN that would poison the heatmap — the same discipline the b-plane 180° case follows). `μ` is caller-supplied (no second hardcoded `μ_sun`). **Validation ladder, all kernel-free:** round-trip vs the analytic `KeplerPropagator` across a spread of orbits/arcs (the advisor's "cheapest and strongest", validates against a propagator already at machine precision); an **independent published worked example** — poliastro's Izzo-algorithm docs, a *different* algorithm, **fetched not recalled**, agreement ~0.02 m/s (floored by the page's digit rounding); the free **energy + angular-momentum invariants** of the transfer conic across many geometries; and an arrival-state-reaches-`r2` forward check. 7/7.
- **`core/src/launch_vehicle.rs` — real C3→payload delivery curves** (commit `00c203a`). The deliverability half: given a departure `C3` (km²/s²), how much mass a real rocket lifts to it. **Provenance was the hard gate the advisor flagged** — plausible launch numbers are the recallable-but-wrong trap, and unlike the sb441 GMs there is no kernel to machine-verify against — so every knot is **fetched and cited**: transcribed from AMAT's `launcher-data/*.csv` (`github.com/athulpg007/AMAT`, machine-fetched via `gh api`), which are compiled from the **NASA LSP Performance website** via Girija arXiv:2310.05994. Five vehicles spanning the capability range (Atlas V 551, Falcon Heavy reusable/expendable, Vulcan Centaur, Delta IV Heavy), linear-interpolated with **0-outside-range = infeasible** (mirrors AMAT's `interp1d(fill_value=0)`). Two labelled caveats: knots downsampled to ~10 km²/s² (<1% vs the full table), and delivered mass modelled *as* impactor mass (Phase-3 refinement). 5/5.
- **`core/src/mission.rs` — the porkchop + on-demand verify** (commit `003565d`). The composition, split by cost exactly like the live force menu (cheap-always-on / expensive-on-demand), because coupling direction means a real deflection check needs a **full-field re-propagation per launch window** — `O(N²)` over a grid is hours. So: **the cheap grid** (`porkchop_grid`) is pure scalar Lambert over Earth/asteroid state arrays looked up *once per epoch* (not `N×M` ephemeris queries), recording `C3`, arrival `|v_rel|`, and the **along-track projection** of the impact — a free first-order *effectiveness* proxy (`v_rel·v̂_ast`) that surfaces the whole point of coupling direction: **deliverable ≠ well-aimed** (a window can carry plenty of `|Δv|` yet project poorly onto the track and barely deflect). The grid is **vehicle-independent**; `C3`→mass maps per launcher afterwards (`cell_delivery`), so switching vehicles never re-solves Lambert. **The on-demand verify** (`verify_cell`) re-propagates one selected cell in the full `n`-body field after the real *vector* impulse `β·(m_sc/M)·v_rel_vec` (via the existing `DeflectionScenario::evaluate`, which already takes a `Vector3` — zero new deflection path), reading the exact b-plane perigee. `required_impactor_mass` bisects the mass to a target perigee with the advisor's **degenerate-direction guard from day one**: a mass cap turns the `v_rel ⊥ v̂_ast` case (no deliverable mass deflects) into an honest `InfeasibleAtCap`, never a runaway bisection. Endpoints are real (Earth ephemeris, asteroid nominal *pre-deflection* trajectory); outputs labelled patched-conic planning estimates. **8/8 tests, made discriminating after an advisor review** (the first cut verified wiring, not behavior — `perigee >= 0` would pass even with the impulse un-applied): the solver is validated **kernel-free** (a `ZeroForce` straight-line pass, like `deflection.rs` tests itself — the solved mass *actually* delivers its target perigee = the mis-bracket catch, monotone in target, `InfeasibleAtCap` at a low cap), and a **cheap kernel-gated test (~2 props)** proves the real-field composition: zero impactor mass reproduces the nominal *hit* (catches wrong epoch/frame), a delivered mass flips it to a *miss* through the coupled pipeline. Splitting algorithm-from-composition dropped the suite from 407 s to 26 s. Core clippy clean.
- **Open / next:** ~~the **Godot porkchop heatmap view** (frontend) — the visualization the user is owed~~ — **landed 2026-07-27** (the `[4]` launch-window map; see *The Godot launch-window map* below). The physics/deliverability was all in core and tested; what the view added was the discovery that `payload_kg` zeroed *cheap* departures as well as unreachable ones. Then the follow-on axes: the direction-coupling makes the headline curve's along-track idealization one lens among several (a phase-sensitivity story), and Lambert now *delivers* the kinetic impactor that the §5 deflection-method spectrum (gravity tractor / nuclear) would choose between. ~~Multi-rev / long-way Lambert and exact NASA-LSP polynomial coefficients are drop-in refinements if ever wanted.~~ **Both landed 2026-07-27, and neither was a mere refinement:** multi-rev exposed that `lambert_universal` was silently returning lapping transfers labelled direct, and the full LSP tables replaced a downsample whose error had been documented as "well under 1%" and measured at 8.9%. See *The deferred leftovers, closed*.

### The deferred leftovers, closed — 2026-07-27 session (J2, Pluto, multi-rev Lambert, the full LSP tables, and the build-time item)

Five items that had been parked as "low priority", "blocked", or "drop-in if ever wanted". Working them turned up three things that were *not* known: a wrong doc claim, a wrong shipped number, and a genuine bug in a shipped solver. That is the argument for clearing a leftovers list rather than letting it age.

- **Earth's `J2` shipped and validated in isolation** (`core/src/forces/oblateness.rs`). The last Tier-2 term (§166). Deferred all through Tier 2 as "negligible heliocentrically", which is *true* — `J2` falls off as `1/r⁴` — and is exactly why it had to be **measured at the encounter** rather than argued about: essentially all of its effect is bought in the minutes the asteroid spends inside a few Earth radii. Validated by the closed-form **nodal regression** `dΩ/dt = −(3/2)·n·J2·(R_eq/p)²·cos i` to <2%, with the guard structure the 1PN/Yarkovsky terms established: a **`J2 = 0` control run** (drift ≪ signal → physics, not integrator noise), an explicit **retrograde sign pair** (`cos i < 0` must make the node *advance*), and three algebraic pins — inward over the equator, **outward over the pole at twice the magnitude**, and purely axial at the magic latitude `sin φ = 1/√5`. That last one caught a sloppy claim in my own module doc: at the magic latitude it is the bracket's `r̂` *coefficient* that vanishes, **not** `a·r̂` (`k̂` is not perpendicular to `r̂` there), so the test now pins `a × k̂ = 0` instead. Kernel-free; 8/8.
- **The spin axis is a parameter, not `ẑ`.** `J2` is defined about the body's rotation axis, and for Earth in ICRF that is *near* `ẑ` but not equal to it. Rather than assume, the term takes a [`BodyPole`] provider; the shipping wiring reads the pole ANISE rotates out of the loaded planetary constants (`Ephemeris::pole_unit_icrf`, the DCM's **third row** — `v_body = R·v_icrf`, so the body `ẑ` back in ICRF is `Rᵀẑ`). Measured by probe: exactly `ẑ` at J2000, **0.2228° off at 2040**, 0.5570° at 2100 — matching the IAU 0.557°/century model to four digits, which independently confirms the row extraction. `FixedPole` keeps the isolation tests kernel-free.
- **`J2` and `R_eq` travel as a pair, from the DE440 header.** The physics contains `J2·R_eq²`, so a `J2` from one solution used with an `R_eq` from another is a silent scale error. Both are read verbatim out of the local `linux_p1550p2650.440` constant record (`J2E = 0.00108262539`, `RE = 6378.1366` km) — the same machine-verified path the sb441 masses took. This makes `EARTH_EQUATORIAL_RADIUS_M_DE440` (6 378 136.6 m) deliberately **distinct** from the WGS-84 `geometry::EARTH_EQUATORIAL_RADIUS_M` (6 378 137.0 m): different roles, 0.4 m apart, not interchangeable.
- **Pluto: the blocker was real, and the answer is that it does not matter.** The open-questions entry parked Pluto on a missing GM — correctly: `pck11.pca` resolves **no** Pluto GM (`ID 9 not in look up table`, verified by probe, not assumed). The DE440 header has one, `GM9 = 2.175096464893358e-12` au³/day² → **975.500 km³/s²**, the Pluto+Charon *system* value as it must be for NAIF 9. Wired behind a toggle and measured on the fixed seed: over the ~12 yr campaign Pluto moves the b-plane perigee by **0.6 metres**. The §5 criterion was "flip to 11-in-shipping if the growing-with-lead-time cost proves to matter"; at the real lead time it is two orders below the belt's already-sub-km floor, so **the shipping field stays at ten bodies** — now on a measurement rather than the batch-2c extrapolation that guessed "plausibly ~km". That 0.6 m reads as signal and not integrator noise only because the terms-**off** re-fly reproduces the shipping perigee **bit-for-bit**; without that identity the number would be meaningless.
- **Measured Tier-2 perigee shifts, all terms, one campaign:** GR 55.6 km · SRP 8.36 km · **`J2` 1.33 km** · belt 0.55 km · Pluto 0.0006 km. `J2` is larger than the entire 16-body main belt. (All five are measured on the nominal *impact*; `J2`'s in-domain figure on a deflected miss is **0.12 km outward** — see *The `J2` pair* below.)
- **The `J2` number grazes a validity boundary, and measuring caught it.** The `J2` expansion is only valid *outside* `R_eq`, and this scenario's nominal is a designed **impact** — closest approach 3000 km, well inside Earth. An earlier draft of the module note called that harmless because "nothing downstream reads the sub-surface arc". Wrong: the b-plane reduction samples the state *at* closest approach and infers `v_∞` from the **point-mass** energy `v_∞² = v² − 2μ/r`, so `J2`'s potential correction there (~`J2·(R_eq/r)² ≈ 5e-3` of `μ/r`) biases it ~1% — visible as the **capture radius moving 11 311.3 → 11 389.0 km** (78 km, 0.69%) against a perigee shift of only 1.33 km. The control that names the mechanism: **1PN leaves the capture radius at 11 311.3 to the digit** (its correction there is ~1e-9 relative), so this is `J2`'s `1/r⁴` growth inside the body, not the reduction. For any *miss* geometry — every deflected trajectory, the case that actually matters — the term is in its valid domain and none of this arises. Read the 1.33 km as "of order a kilometre on a boundary-grazing geometry"; **measuring `J2` on a genuine miss geometry is the honest follow-up.** **Done 2026-07-27 — see *The `J2` pair* below: 0.12 km outward at a 3.0 R_eq perigee, and the capture-radius bias collapses 480x, which is what makes this paragraph a measurement rather than a story.** (This also explains the 11 389 vs the pinned 11 311 km: not two disagreeing code paths, one term evaluated out of domain.)
- **Multi-revolution Lambert — and the shipped bug it exposed** (`core/src/lambert.rs`). `lambert_universal_multirev` solves the `N`-lap transfer, which needs a *different root-finder*: inside the band `z ∈ ((2Nπ)², (2(N+1)π)²)` the time of flight diverges at both edges and dips to a minimum between, so Newton from any seed walks into the wrong basin or off an edge. It brackets instead — scan for the minimum, reject a `Δt` below it as `NoSolutionForRevolutions` (a real geometric gap, reported with the threshold missed, never a `NaN`), bisect on the requested `LowZ`/`HighZ` branch. Validated by flying each solution through the **analytic Kepler propagator** and confirming it reaches `r2` — an independent check across a real formulation gap (universal variables/Stumpff vs classical elements/Kepler's equation), which a wrong root fails.
- **The bug: `lambert_universal` was silently returning lapping transfers.** `T(z)` rises monotonically to infinity as `z → 4π²`, so a single-rev root exists for *every* time of flight — but a Newton step from the `z = 0` seed can overshoot straight past that pole and converge in the 1-revolution band. The result looks perfect (it reaches `r2` on time) while being a transfer that laps the Sun, carrying a different `C3`, labelled direct. In a porkchop that is the worst kind of wrong: a plausible number in a cell that is not what it says. `SINGLE_REV_Z_MAX` clamps the iterate; the regression test is **physical** (a sub-one-revolution transfer must finish inside its own orbital period) rather than a peek at `z`, and a long-window case covers the clamp's own weak spot, where it degenerates to pure bisection near the pole.
- **The fix and the feature are two halves of one change** (`core/src/mission.rs`). The default grid's times of flight are ~3.6–3.9 yr, squarely in the affected zone — so clamping *alone* would have replaced those accidental lapping transfers with the honest direct arc, which at long spans is the slow, ruinously expensive one. Measured on a 2.6 yr span: direct arc **C3 = 933 km²/s²** (no launcher reaches it) versus **55 km²/s²** lapping. The clamp alone would have turned the long-time-of-flight half of the heatmap infeasible. So `best_transfer_metrics` now selects the lowest-`C3` option across `N = 0…max_revolutions` (both branches per `N`), `porkchop_grid` takes `max_revolutions`, and `TransferMetrics::revolutions` **says which trajectory a cell actually is** — a mission that laps the Sun is a different cruise, not just a different number.
- **The full NASA-LSP tables, and a doc claim that was 9× wrong** (`core/src/launch_vehicle.rs`). Every knot of every AMAT CSV is now embedded (101/10/100/64/100 points), machine-fetched via `gh api`. The previous ~10-point downsample shipped with a note claiming its interpolation error was "well under 1%". That had never been measured; measured, it is **8.9%** for Atlas V near `C3 = 95`, 3.2% for Falcon Heavy reusable, 2.7% for Vulcan. The curves are smooth in the middle but bend sharply as a vehicle nears its energy limit — the high-`C3` region a fast intercept lives in, so the error was concentrated exactly where it mattered. The transcription itself was faithful (all 11 shipped Atlas knots match the full table *exactly*); only the sampling was too sparse. Two new tests pin the row counts and strict `C3` ordering, because interpolation/monotonicity tests all stay true of a subset and would not notice a silent re-downsample.
- **The build-time "regression" was a measurement artefact.** Recorded as "debug DLL 11 s → 34 s"; it does not reproduce. Measured, `touch core/src/lib.rs` → gdext DLL: **2.5 s** steady state, **19 s** with the rustc incremental cache deleted, ~95 s on the first build of a session (cold OS file cache on top). Deleting the 933 MB `target/debug/incremental` and immediately rebuilding twice isolates it cleanly — 19 s then 2.45 s, same code, same profile. So the variable is **cache state, not the grown core and not the `opt-level = 3` override**, and the original 34 s was almost certainly a post-edit cold-cache run. Nothing to fix; closed as unreproduced rather than left open, with the numbers recorded so the next person does not re-litigate it.
- **Grid cost, measured before the heatmap view needs it:** selecting over revolutions is **0.6 µs/cell** at `max_revolutions = 0` versus **44.7 µs/cell** at 1 and 87 µs at 2 (`core/examples/bench_porkchop_cell.rs`) — a ~70× step for the first lap, since each `N ≥ 1` is a scan-and-bisect. A 200×200 grid is 23 ms direct-only against 1.8 s allowing one lap: fine for a grid built once on a worker, not per frame. A ~2× saving is there whenever it matters (scan each band once, solve both branches from that scan). Recorded now because this project has twice been bitten by an unmeasured per-cell cost.
- **`best_transfer_metrics` ranks on `C3` alone**, which is deliverability, not aim — in tension with the module's own *deliverable ≠ well-aimed* thesis, since a cell reports the along-track projection **of its cheapest option**. `C3` is still the right primary key (it is the hard constraint: over a launcher's energy limit delivers zero mass, and zero mass deflects nothing), but the criterion and its caveat are now stated where the function explains itself rather than left implicit.
- ~~**Still open, deliberately:** the frontend `[P]` force menu measures GR/Yarkovsky/belt/SRP but **not** `J2`~~ — **closed 2026-07-27, below** (*The `J2` pair*), together with the "measure `J2` on a genuine miss geometry" follow-up two bullets above; they turned out to be one item, not two. ~~Also unchanged: the **Godot porkchop heatmap view**~~ — **landed 2026-07-27, below.**

### The `J2` pair — 2026-07-27 session (the force menu's fifth term, and `J2` measured where it is valid)

The one item the leftovers entry above left open on purpose, plus the honest follow-up it named two bullets earlier. They read as two tasks and are one: the `[P]` menu measures every term on the shipping nominal, and for `J2` alone that geometry sits **outside the term's domain**, so shipping the menu entry without the in-domain number would put a boundary-grazing figure on screen under four that are not.

- **The menu's `J2` had to be measured on the nominal, whatever its domain.** The panel forms every shift as `nominal_perigee − shifted_perigee`, one baseline for all five. Measuring `J2` on some better-behaved geometry and subtracting it from *that* baseline would difference two unrelated passes and print something that looks like a shift and is not — the exact failure class this project keeps catching. So the fifth row is measured exactly like its neighbours (fixed seed, one term swapped, `nominal_encounter_with`), and the caveat is carried by a **footnote** instead: one line, drawn only while `J2` is revealed, citing the in-domain number beside it. No third availability state — `tier2_available` stays a `>= 0.0` bool, and `J2` has no kernel dependency to be unavailable for.
- **A designed miss cannot be built; a deflected one can.** The obvious way to reach a wide perigee is a bigger `b_offset_km`, and it does not work: `RealFieldScenario::build` verifies its designed impact round-trips, so 15 000 km comes back as `perigee 1.500e7 m ≥ capture radius 7.711e6 m (not a hit)` rather than as a miss. Measured before designing anything (`core/examples/probe_miss_geometry.rs`). That leaves the deflected pass — which is also what the docs already said matters, since *every* successful deflection is one. New core entry point `RealFieldScenario::deflected_encounter_with`: same contract as `nominal_encounter_with` (seed **and impulse** fixed, only the field toggled) with both routed through one private `with_toggled_field`, so the two can never disagree about what "`J2` on" means.
- **The geometry was solved for, not guessed.** The probe solves `required_dv_along_track` for a target perigee and reports what the pass actually reaches: **0.399625 m/s** along-track one year out → perigee **19 139.2 km = 3.001 `R_eq`**, `|B|` 25 064 km against an 11 312 km capture disc — a clean miss, comfortably in domain. (It also re-taught an old lesson about cost: the first run put the impulse at the campaign start, which makes every one of ~30 bisection steps a full 12 yr flight. It was still going after 18 minutes. What is being fixed here is a *perigee*; the lead time only sets how much Δv buys it.)
- **The result, and it is not just "smaller".** On the miss geometry `J2` moves the perigee **0.1196 km outward**, where on the impact geometry the same term shows **1.3257 km inward**. Different magnitude *and* different sign — though the sign is not by itself evidence of the domain problem, since the term carries a Legendre factor in the latitude of closest approach that two different passes have no reason to share. The sufficient point is narrower and holds regardless: the menu's 1.33 km is *that geometry's* number, not "what `J2` does to a deflection", which is why it is captioned.
- **The assertion that would fail if the explanation were wrong.** "`J2` moves the perigee by a nonzero amount" is what the existing sibling test already asserts and would pass on any geometry — green, and worth nothing here. The docs' claim is *causal*: the capture-radius anomaly is `J2` evaluated deep inside the body, because the b-plane reduction infers `v_∞` from **point-mass** energy at the sampled closest approach. That correction goes as `(μ/r)·J2·(R_eq/r)²`, i.e. **`1/r³`** — note the `μ/r`, which is why it is `1/r³` and not the `1/r²` the potential term alone suggests. So the test asserts the anomaly *collapses with distance*, two ways: model-free (at least 10×) and against the predicted `1/r³` with slack for the Legendre factor, which can shrink the result but not inflate it. Measured: **0.6867 % → 0.00143 %, a 480× collapse** across a 6.4× wider perigee. A reduction that were simply biased would not care about `r`.
- **The control that names the mechanism, re-run on the new geometry:** 1PN on the *same* deflected pass leaves the capture radius at 11 311.7 km, `4.4e-7` relative. If the capture radius moved for any reason other than a term's own potential reaching into the reduction, it would move there too.
- **Five call sites, and the two that were made impossible instead of edited.** A new term touches `Tier2Shifts`, `measure_tier2_shifts`, the term table, the per-term toggle dict, and main.gd's key chain. Two of those were separate hand-kept lists that would silently half-work if they drifted — `toggle_tier2` ignores an unknown id, so a term in the table but not the dict does nothing when its key is pressed. Both now **derive from `TIER2_TERMS`**: the dict is populated in `Sim._ready`, and the shot harness reads its key/id pairs out of the same table (via `OS.find_keycode_from_string`) rather than restating them — a hand-kept second list is how a new term gets a row, a measurement and an action while nothing ever presses its key, and the check reads as coverage while being none.
- **`[O]` for Oblateness, because `[J]` was taken** (`milestone_jump`, keycode 74) — checked against every existing binding rather than assumed, and it keeps the mnemonic set G/Y/A/S/O.
- **The panel's columns are now measured, not arithmetic.** The `J2` row is the longest label in the table and cleared the old `30 × _fs × 0.60` guess by ~27 px. Last session the porkchop readout's second column overlapped its first for exactly this reason; the fix there was to size off the font's own measurement, and it is the fix here — `xstate` comes from `get_string_size` over every label in the table. The row count is derived from the table too (it was the literal `11.0`), so the box cannot end up one row short of its own contents.
- **The in-domain number is a pinned constant, not a caption.** `J2_DEFLECTED_MISS_PERIGEE_SHIFT_KM` lives in the core beside the API that measures it, reaches the panel through the binding, and is asserted against the live measurement by `earth_j2_on_a_deflected_miss_is_in_domain` — so the footnote cannot drift from the physics. The same treatment `SB441_BODIES` and `threat_mass_kg()` get.
- **Cost:** five terms instead of four, so the preview is a fifth longer in principle — **measured in the frontend at 119.8 s**, comfortably inside the harness's 240 s wait. The panel's standing "~2 MIN" is what the measurement supports, so it stays; the strings that *were* stale are the "four shifts" ones, and where the panel prints a count it now reads it from `TIER2_TERMS.size()` rather than spelling it.
- **Verified through the real key path, then in the frame.** `_tier2_shot.gd` presses `[O]` as an actual `InputEventKey` through `main._input`, so project.godot's action → main.gd → `Sim` → core all had to be right for the assert to pass — which is the check the hand-edited `project.godot` needed, that file being the one edit in this batch that no compiler sees. All five shifts came back live: **GR +55.55 · Yarkovsky +5.10 · SRP +8.36 · `J2` +1.33 · belt +0.55 km**, matching the recorded values to the digit. The picture then confirmed what no assertion covers: five rows inside a box that grew for them, the longest label in the table clearing its `ON` column, and the footnote's *0.12 km outward* sitting directly under the row that reads *+1.33 km inward* — the two numbers legible against each other, which is the entire point of adding the caveat rather than just the term.
- **The batch found one piece of coverage that was already lying.** The binding's preview test — `tier2_preview_measures_three_terms_and_leaves_belt_unavailable_unmounted` — loops over the terms it expects to be available, and that loop read `["relativity", "yarkovsky", "srp"]`. It passed, in full, **without ever touching `J2`**: the new term was measured, wired, displayed and shipped while the test named for checking exactly that quietly checked four-fifths of it. The loop now runs off a `TIER2_TERM_IDS` constant that lives beside `Tier2Shifts`, so the next term is a visibly short list rather than a green run, and the test is renamed off the count it had outgrown. Worth stating plainly because the *shape* recurs: a check written against an enumerated set stops being coverage the moment the set grows, and nothing about it turns red to say so.
- **Left open on purpose:** `deflected_encounter_with` ships as public core API with exactly one caller, the test. It earns that on the drift argument alone (it and `nominal_encounter_with` are now one code path), but nothing in the frontend uses it yet — so the `[P]` menu still answers "what does this term do to the *nominal*" and cannot answer "what does it do to **my plan**", which is the more useful question and is now one call away. Noted here rather than half-wired, the same way this menu's `J2` row was.
- **A dropped catch worth recording:** the first run of the harness was against a **stale debug DLL**. `godot --path godot` runs the project in debug and loads `target/debug/`, and only the *release* binding had been rebuilt — so the run sat there with a binding that had no `j2` term in it at all. Nothing said so; it simply produced no output. When a Godot verification run goes quiet, check which profile's DLL it actually loaded before debugging anything else.

### The Godot launch-window map — 2026-07-27 session (the §8 heatmap view, closed)

The mission layer's long-standing follow-up, open since the Lambert/porkchop core landed on 2026-07-21: the physics was built and tested, and no one could *see* it. `[4]` now opens a real porkchop over the real campaign, and the view exists to make the project's own headline honest — the planner's "spend 0.2 m/s twelve years out" assumes an impulse can be delivered, and this is the map of whether it can.

- **The layer is three files and mirrors the Tier-2 menu exactly**, because that split is already proven: `mission_core::PorkchopView` (godot-free, worker-callable) → `Mission::begin_porkchop`/`poll_porkchop` on its **own** `mpsc` channel → `Sim` state under its **own** `pork_online` flag → `porkchop.gd`, a pure-display `Control`. The grid is built **on demand** when the view opens, never on the scenario build path — the same discipline the `[P]` menu established after chaining a measurement onto the build was measured at >200 s in-game.
- **Three kinds of empty, drawn three different ways.** A cell can be blank because *no trajectory exists at any lap count* (`c3 = -1`, background), because *this launcher cannot reach that `C3`* (`payload = 0`, dim floor), or because it is a real reachable window that *projects poorly onto the track* (a dark but live cell). Collapsing any pair throws away something the operator needs — the second is the entire payoff of a vehicle-independent grid, and the third is the module's *deliverable ≠ well-aimed* thesis. Measured on the shipping 120×120 grid: **4849 blank / 7193 unreachable / 2358 reachable** for Atlas V 551, so all three states are genuinely populated and the distinction is not decorative. And the second state really does move with `[L]` — across the five launchers the same 12 042 real transfers yield **2358 / 1438 / 2358 / 2281 / 2358** reachable windows, Falcon Heavy *reusable* being the outlier because its table stops at `C3 = 64` where the others reach 96–100. Counted for **every** vehicle in the harness rather than the one on screen, since the count for whichever launcher happens to be showing cannot demonstrate that switching changes anything. Sentinels, never `NaN`: one `NaN` in a packed column poisons every min/max the ramp normalizes by and flattens the whole picture to one colour.
- **How many laps the grid allows was decided on numbers, not taste.** `DEFAULT_MAX_REVOLUTIONS` is the one free parameter that changes what an operator *sees*, since every extra lap is another family of cheaper transfers. Measured over the default campaign (24×24, 379 real transfers, windows Falcon Heavy expendable can reach): **`N≤0` → 21 · `N≤1` → 56 · `N≤2` → 95 · `N≤3` → 129**, at 4.8 / 47.8 / 69.0 / 106.8 µs per cell. A direct-only grid therefore shows **under a quarter** of the missions that exist. Shipped at **2**; the test prints the whole table, because laps keep opening windows forever (at long times of flight a tighter lapping orbit is simply cheaper) so there is no knee to find — only a cost that doubles for cruises that grow by a full solar orbit each step. The shipping grid solves in **851 ms** on a worker.
- **The frontend must not invent a third rock.** The delivered Δv divides by the threat's mass, which nothing had ever needed before — and `SrpParams::sub_km_rock` hides the body (150 m, 2000 kg/m³) as function locals, so getting a mass meant restating them. Named as `THREAT_RADIUS_M`/`THREAT_DENSITY_KG_M3` → `threat_mass_kg()` ≈ **2.83e10 kg**, with a drift test asserting `3/(4rρ)` equals `SrpParams::sub_km_rock().area_to_mass_m2_per_kg` — the same treatment `SB441_BODIES` gets. Without it the SRP toggle could model a 300 m body while the porkchop divided by some other one. (`mission.rs`'s test fixture uses `2.0e10` and stays a fixture; this is what the *shipping display* divides by.)
- **The verdict reads `|B|` against `b_capture`, and a harness assertion re-derives it.** Exactly one number in the view is not a patched-conic planning estimate: `[E]` re-flies the asteroid through the full `n`-body field with the impulse **that launcher** would deliver through **that window** (~3.2 s, its own channel again). Its outcome is an *enum* — `CleanMiss` / `Encounter{…}` / `NotHyperbolic` — not a struct of sentinels, so the best possible result cannot share a `-1` with "not verified yet". The measured example: 3956 kg through a `C3` 22.3, two-lap, 2049-day window imparts **+3.06 mm/s** along-track and leaves `|B|` at 6930 km inside the 11 311 km capture disc — **`SURFACE IMPACT`**. That is the honest headline of the whole layer: a single real launcher's kinetic impactor delivers ~1/65th of the ~0.2 m/s the deflection curve wants, and the map says so instead of implying otherwise.
- **The view found a shipped bug in `launch_vehicle.rs`, and it was in the *cheap* direction.** `payload_kg` returned `0` outside the tabulated `C3` range at **both** ends — a faithful port of AMAT's `interp1d(fill_value=0, bounds_error=False)`, and wrong at the low end, because a lower `C3` is an *easier* departure. It had been harmless while nobody asked about cheap transfers; allowing two laps pushed the grid's cheapest cells to **`C3 = 0.34 km²/s²`**, below the `1.0` where four of the five tables start, so the heatmap drew real, easily-flyable windows as unreachable and captioned them *too much C3 for this rocket* — precisely the reverse of the reason. Below the first knot now holds that knot's payload flat (conservative: the true value is a little higher); above the last knot is still `0`, which is the genuine physical ceiling. The payoff is that **`0` now means exactly one thing**, so the display renders one honest state instead of one word covering two opposite situations. Where a table *starts* is an artefact of the published data's sampling; where it *ends* is physics — treating the two ends alike was the mistake. Pinned by `payload_kg(0) > 0` for every vehicle (a bare escape trajectory is the least exotic departure there is) and, in the binding, by asserting every launcher delivers something at *the grid's own cheapest cell* — so if the grid ever stops reaching that low, the guarantee does not quietly become untested. **The 12×12 test grid never goes below 1.0 (its cheapest cell is `C3` 3.64) and passed throughout**; the 24×24 grid reaches 0.34 and the shipping 120×120 reaches **0.269**. So the check lives on a grid that demonstrably enters the regime, and it **asserts that it does** before asserting the behaviour — a below-the-table check on a grid that never goes below the table is worse than no check, because it reads as coverage. The advisor caught the whole thing pre-commit; the first version of the check was itself vacuous.
- **Delivered mass is modelled *as* impactor mass** — no bus, no propellant, no structure bookkeeping (a Phase-3 refinement, already noted in `mission.rs`). That makes every payload figure here **optimistic**, which matters for reading the headline: the `SURFACE IMPACT` verdict is conservative in the safe direction. A real spacecraft delivers less than 3956 kg of impactor through that window, not more.
- **The heatmap is a texture, not 14 400 `draw_rect`s.** `_process` queues a redraw every frame while the view is visible, and the first cut redrew every cell each time. One texel per cell with `TEXTURE_FILTER_NEAREST` gives the same crisp blocks in one draw call (and nearest matters on its own terms: a smoothed heatmap would invent gradients between windows that were never solved). The reachable-window count is likewise cached in the rebuild rather than swept per frame.
- **Verified by picture, and the picture is what found the bugs.** The `_pork_shot.gd` harness drives every step through `main._input()` with real key events — project.godot action → main.gd → `Sim` → core, not direct calls that would bypass the part most likely to be miswired — and asserts the view switch is exclusive, that `[L]` changes the delivered mass and wraps the table, and that `is_hit` agrees with `|B| ≤ b_capture`. All of that passed while the *first* screenshot showed a tiny green blob in the corner: `pork` had never been added to `main._sync_overlay_sizes`, so the Control kept its default size. Two further collisions were visible only in the image — the HUD's mission panels sitting on the heatmap (they now stand down for this view, as the 3D tag layer already does for the 2D views) and the readout's second column overlapping its first (now sized off the font's measurement of the longest left-hand line, not a guessed character count). **Every one of these passes a test suite; none of them survives looking at the frame.**

### Required impactor mass on the map — 2026-07-27 session (`[M]`, the follow-up `[E]` cannot answer)

The last thing the porkchop layer left open. `core::mission::required_impactor_mass` had been built, tested and documented since 2026-07-21, and no frontend could reach it — the map could say *this launcher fails here* and then stop, which is the least useful place to stop. `[E]` asks **does this launcher work through this window**; `[M]` asks **how much mass would**, and the ratio between them is the campaign's honest headline stated as a number instead of implied.

- **Measure-first decided every parameter, and the first estimate was 10× low.** The naive call — 100 kg seed, the core's hardcoded `1e-4` tolerance, a `1e9` cap — was measured at **455 s** on an early-arrival cell and **170 s** on a late one, with the expected-common `InfeasibleAtCap` path at 194 s / 74 s. The advisor's two corrections were both right and both invisible from the code: probe cost is **cell-dependent** (a probe re-flies from arrival to the encounter, so **18.2 s** at a 10.8 yr lead against **5.8 s** at 3.2 yr — the arrival axis spans 0.10–0.92 of a ~12 yr campaign), and the *infeasible* path is the expensive one because it walks the entire doubling ladder before it can say no. Shipping parameterisation is **46 s** for the best-coupled window and **31 s** for a hopeless one; the slow tail is an early-arrival cell far from the seed, ~3 min. Three knobs, and it is worth naming which bought what: the **seed** removed ~11 doublings, the **tolerance** ~7 bisections, and the **cap** is what stops an unreachable window climbing to 1e9 before admitting it.
- **The seed had to be made *safe* before it could be made *fast*, and that was a real bug in the shipped core.** `required_impactor_mass` returned `seed_mass_kg` verbatim when the seed already cleared the target — an upper bound reported as *the requirement*. Harmless while every caller passed a deliberately tiny seed; fatal the moment one seeds from something meaningful to save propagations, because the displayed physics would then be a function of the seed. It now brackets **downward** (halving until a mass *fails*) as well as upward, so a good seed buys speed and cannot change the answer. Pinned kernel-free by seeding **100× high** and requiring the same mass back as a low seed gives — the assertion that fails if the short-circuit ever returns. Only then was it safe to seed at `heaviest_deliverable_kg()` (**14 714 kg**, Falcon Heavy expendable at its cheapest `C3`, derived from the LSP tables rather than written down), which puts the first probe inside the range the answer lives in instead of climbing to it.
- **The cap's size is a display decision, not a solver parameter.** `100 × heaviest_deliverable` = **1 471 393 kg**, and a hundred rather than the ten first considered because ten (≈147 t) sits *below* every window's requirement in the shipping grid — every cell would read "over the cap" and the number the whole feature exists to print would never appear once. `InfeasibleAtCap` is rendered as **data, not failure** ("OVER 1 471 393 KG (100 BEST LAUNCHES) — REACHES ONLY 16 413 KM OF 20 000 KM"), because a clean-miss-shares-a-sentinel bug in a new place is still that bug.
- **One target, named once.** The requirement is solved against **`SAFE_PERIGEE_TARGET_M` = 20 000 km**, which was a bare literal duplicated across `viewer/src/bin/curve.rs`, the `curve.json` it writes, and the binding's `required_dv_matches_curve_json`. It is now a core constant those all read, so the map's *mass* requirement and the headline curve's *Δv* requirement are quoted against the same bar — two requirements measured against different bars would look comparable and would not be. Pinned by an assertion in the curve test: move the target and the recorded `curve.json` expectations fail loudly rather than silently describing a different mission. The readout **names the target in the line** ("20 232 KG TO REACH 20 000 KM PERIGEE = 7 x WHAT ATLAS V 551 DELIVERS") — "required mass" alone reads as *the mass to miss Earth*, which is a smaller number. It is a **margin, not a hit test**: the verdict question stays `|B|` vs `b_capture`, and this is a design goal in the perigee's own units, deliberately clear of the 11 311 km capture disc rather than grazing it.
- **The test is the round trip, not the return value.** HANDOFF already records the earlier catch on this exact module — *"the first cut verified wiring, not behavior; `perigee >= 0` would pass even with the impulse un-applied"* — so the binding gate takes the mass the solver reports, flies **that** mass through the independent `[E]` verify path, and requires the perigee it reaches to clear the target: **20 232 kg → 20 905 km** against a 20 000 km bar. And because *sufficient* is not *required*, it flies 20 % less and requires it to **fail**: **16 185 kg → 16 453 km, short**. Without the second half, returning the cap would pass everything. Both outcomes come from one grid — the best-coupled cell is `Feasible`, the worst-coupled late cell is `InfeasibleAtCap` — so neither is a purpose-built fixture, and the test prints its own cost so a 5× regression is visible rather than merely slow.
- **Vehicle-independent, and the harness presses `[L]` to prove it.** The requirement is a property of the window's geometry and lead, so it is keyed by `(launch, arrival)` with **no vehicle index** — cycling the launcher recomputes only the ratio beside it. That is not a nicety: keying it by vehicle would re-fire 30–180 s of propagation on a keypress that cannot change the answer. `_pork_shot.gd` drives `[M]` through the real action chain, asserts the pork guard beats the shared `plan_toggle` binding on the same key, then presses `[L]` and asserts the requirement is *still current* and no solve re-fired.
- **A shipped doc claim corrected on the way past.** `begin_cell_verify` documented `[E]` as "one propagation (~1 s)". Measured: **5.8–18.2 s**, cell-dependent. Never measured, wrong by 6–18×, and it mattered here because it is the unit the mass solve is priced in — a 10-probe solve budgeted at "~10 s" is a 3-minute one.
- **And the picture found one more, again after a fully green harness.** Every assertion passed while the frame showed the event log printing straight through the readout panel and the keys row. The console draws from `MARGIN` at full width; this view alone parks its readout in the right-hand half, so a long enough line lands on top of it. **The `[E]` verdict line already overran** — it had only ever been screenshotted *mid-typewriter*, so it read as fitting, and the longer `[M]` line is what made it unmissable. Fixed as a width **budget** rather than shorter messages (the next long message would repeat it): `PorkchopPlot.PANEL_X_FRACTION` is now one named number read by both the panel that sits there and `hud.gd`'s new `_console_width`/`_clip`. The labels were shortened too, and specifically at the *prefix* — the ratio is the end of the line and the part a reader wants, so it must not be what an ellipsis eats. Third time in this view: `_sync_overlay_sizes`, then col1/col2, now the console. **All three passed a test suite; none survived looking at the frame.**

### The deflection spectrum, nuclear half — 2026-07-27 session (§5's second method, and the source hunt that decided its coefficient)

Phase 2's checklist is down to two unstarted items; this is the first half of one of them. §5 asks for deflection modeled *as a spectrum across lead time* — gravity tractor at the gentle end, kinetic impactor in the middle, nuclear standoff at the top — and until now `deflection.rs` shipped exactly one method. The porkchop layer made the gap concrete rather than theoretical: it closed by printing `SURFACE IMPACT`, one real launcher delivering ~1/65th of what the curve wants, with nothing to compare that against.

- **Nuclear is impulse-shaped; the tractor is force-shaped — and the code already said so.** `deflection.rs`'s own `apply_impulse` doc has read *"a finite burn (gravity tractor, §5) is a Tier-2 force term, not this"* since the MVP. So the split is not a new decision: nuclear standoff lands as a sibling of `kinetic_impactor_dv` in `deflection.rs`, and the tractor will land beside `yarkovsky.rs` in `forces/` with the one thing no existing term has — a **time window**. That second half is deliberately a separate batch, because it needs a force term *and* a new solver axis (duration, not impulse), which `DeflectionScenario::required_dv` structurally cannot express.
- **Direction is chosen, and that is the real physical difference between the two methods.** A kinetic impactor's Δv points along the arrival relative velocity — the transfer picks the direction and the mission layer takes whatever along-track *projection* it gets (`impact_impulse`, and the whole reason `required_impactor_mass` must root-find). A standoff burst ablates whichever hemisphere the device is placed over, so its impulse direction is a mission choice. Which means the nuclear requirement is a **closed-form invert** of the existing along-track curve, not a second root-find: `yield_kilotonnes_for_dv` and the matching `kinetic_impactor_mass_for_dv` requote an already-solved Δv with **zero extra propagation**. Worth recording because the advisor's standing warning — *don't fork the bracket-and-bisect three ways, that's three places for the seed short-circuit bug to come back* — turned out not to bite here at all, and the reason it didn't is this geometric asymmetry rather than luck. It returns for the tractor.
- **The coefficient was fetched, not recalled, and the first two candidate sources were rejected on the numbers.** `launch_vehicle.rs` set the precedent, and a fast model misaligning the DE440 `CVAL` arrays set the cost of ignoring it. Ahrens & Harris 1992 (*Nature* 360, 429) is the canonical citation but is paywalled — its efficiency figures reach us only through secondary quotation, so it is cited as context and is **the source of no number in the code**. LLNL-PROC-485160's three-point *surface-burst* table (0.1 kt → 2.3 mm/s, 0.5 kt → 0.92 cm/s, 1 kt → 2.8 cm/s on a 1 km body) was rejected for being the wrong mechanism and for being non-monotone in Δv-per-yield (23 / 18.4 / 28 mm/s per kt) — the high yields eject 1.9 % and 7.5 % of the body, so it is not a nudge model. An early attempt to derive a coefficient from one parenthetical clause of that paper produced a value contradicting the same paper's own headline figure by ~3×, which is exactly the *"clean-looking coefficient with no provenance"* failure mode, caught before any code was written.
- **What shipped is `StandoffNuclear::DEARBORN_2007`, from a fully-specified published case.** UCRL-PROC-228569 (D. S. P. Dearborn, LLNL, 2007), the paper's own "Nudge Model": a **100 kt** device **300 m** above the surface of a **1000 m diameter, 1.05e12 kg** body deposits **11.5 kt** into the surface and settles the coalesced body at **≈6.5 mm/s**, with **99 %** of the mass still bound. Every quantity that matters is stated in one place — body, yield, height, absorbed energy, outcome — which is why this case and not another. Two details make it the right fixture rather than merely a usable one: the 300 m height is `d/R = 0.6`, which the *same* paper independently names as the **optimum height of burst**, so the coefficient is pinned at the geometry the model describes; and the paper's separately-stated ≈0.5 m/s escape speed falls out of its own mass and diameter only if 1 km is the **diameter**, which is asserted in the test — reading it as the radius would have made every Δv here wrong by ~8×. Inverting gives `C_m = 1.418e-4 N·s/J` at `η = 0.115`.
- **The honest uncertainty is stated as a spread, not hidden in a third digit.** LLNL-PROC-485160 reports a *different* run — 10 kt deposited ablating ~4000 t at "over 2 km/s" — which recomputes to `C_m = 1.91e-4 N·s/J`. The two independent series **disagree by at least 36 %**, and "at least" is load-bearing: *"over* 2 km/s" makes the 2011 value a **lower bound**, and the shipped 1.418e-4 sits *below* it, so the two point the same way and 36 % is the floor of the disagreement. An earlier draft of this section and of the doc comment claimed they "bracket rather than contradict" — they do not bracket anything, and the advisor caught it. The shipping choice is unaffected (smaller `C_m` ⇒ more yield required ⇒ errs high), only the framing was wrong. A test asserts the constant stays inside the published band — a guard on *provenance* no test of the model's arithmetic would catch, and **the only check in the suite that bounds the coefficient from above**, since the fragmentation case below bounds it only from below.
- **The validation is a case the coefficient was never fitted to.** Reproducing the Nudge Model is self-consistency, not evidence — the constant came from it. So the real test takes the *other* paper's *other* body: LLNL-PROC-485160 deposits 17 kt into a 270 m, 2.78e10 kg non-porous body and reports it **"completely fragmented" (Ke/Pe > 1)**. The model, given only the absorbed energy, independently returns **Δv = 0.363 m/s against a 0.166 m/s escape speed — a ratio of 2.19**, comfortably above escape. That is the assertion that fails if the coefficient is wrong by the order of magnitude a dimensional-plausibility check would wave through, and *"the formula looks dimensionally right"* was explicitly ruled out as an acceptance criterion. Its limits are recorded with it: the check is **one-sided** (it catches `C_m` too low, never too high) and it extrapolates across roughly **20× the surface fluence** the coefficient was fitted at, in the direction where coupling-per-joule is known to fall — so the 2.19 carries perhaps a factor of two of margin against a real systematic. Enough for the qualitative claim it tests, not enough for anything quantitative.
- **Yield is the axis; delivered mass is not.** §5 says *"model this as deflection physics only — never weapon design"*, and the mass→yield map is precisely where that bites, since yield does not scale linearly with device mass. So yield enters as an **opaque scalar** and no part of this crate converts a launch window's payload into one. A launcher's delivered mass can only ever gate whether a device of a stated class is carriable — a payload question the mission layer already answers.
- **`beta` was not reused, and that was a near miss worth naming.** Kinetic β is the ejecta momentum-enhancement factor (1–4, dimensionless, DART measured ≈3.6). Nuclear ablation efficiency is a different quantity with different units. One word covering two situations is the `payload_kg`-returns-`0`-at-both-ends bug in a new place, so the nuclear parameters are `coupling_efficiency` and `momentum_coupling_ns_per_j`, both named for what they are.
- **The model reports whether the body survives, and refuses to invent the curve between the two points that say.** Both papers are emphatic that this is a small-nudge model that stops being one as Δv approaches escape speed — 485160 states flatly that on a 100 m body (`v_esc ≈ 5 cm/s`) *"inducing a 1 cm/s speed change will almost certainly result in extensive debris ejection or fragmentation"*. That is `Δv/v_esc = 0.2`; the Nudge Model's intact case is `0.013`. Fifteen-fold apart, with **nothing published in between**. So `DisruptionRegime` has three states — `IntactDeflection` ≤ 0.013, `LikelyDisruption` ≥ 0.2, and `Uncharacterised` between them — rather than one invented threshold at a tidy midpoint, which would read as knowledge. Both anchors are asserted from the published cases, and the middle state is asserted reachable, because a three-state enum no input can land in the middle of is a two-state enum with a lie in it. This is the same discipline the J2 validity boundary got.
- **The comparison is quoted at one bar, and that single table is the point.** A second method is not a second formula; it is two answers to the *same* question. On the published 1 km body at its own 6.5 mm/s: the kinetic route needs **189 583 kg = 12.9× the heaviest single launch** any vehicle in `launch_vehicle.rs` manages (14 714 kg, Falcon Heavy expendable at its cheapest `C3`), while the nuclear route needs **one 100 kt device**, at a Δv the classifier rates `IntactDeflection`. Three methods quoted against three different bars would look comparable and would not be.
- **But that table is on Dearborn's rock, and it does not transfer — which is the batch's actual finding.** The first version of this section stopped at the paragraph above and stated the `IntactDeflection` verdict as though it were the campaign's answer. It is a property of a **1.05e12 kg** body with a 0.53 m/s escape speed. The shipping threat is a **300 m, 2.83e10 kg** body whose escape speed is **0.159 m/s**, and re-running the identical comparison there — live full-field `required_dv_along_track`, `threat_mass_kg()`, the same `SAFE_PERIGEE_TARGET_M` — inverts the conclusion:

  | lead (yr) | required Δv (m/s) | nuclear (kt) | kinetic (t) | Δv / v_esc | regime |
  |---:|---:|---:|---:|---:|:---|
  | 0.39 | 0.5878 | 243.6 | 462 | 3.705 | `LikelyDisruption` |
  | 0.79 | 0.5098 | 211.2 | 400 | 3.214 | `LikelyDisruption` |
  | 1.58 | 0.2551 | 105.7 | 200 | 1.608 | `LikelyDisruption` |
  | 3.16 | 0.1277 | 52.9 | 100 | 0.805 | `LikelyDisruption` |
  | 6.32 | 0.0662 | 27.4 | 52 | 0.417 | `LikelyDisruption` |

  **Every lead the campaign covers lands in `LikelyDisruption`** — the curve never even reaches the `Uncharacterised` band. 65.7 kt is where a burst reaches this body's escape speed; intact deflection would need Δv ≤ 0.00206 m/s, and the *easiest* point on the whole curve still asks **32× that**. So on this threat a standoff burst sized to do the job does not deflect the rock, it disperses it — which is exactly what LLNL-PROC-485160 says in words: *"[a]t a size of 100 meters ... inducing a 1 cm/s speed change will almost certainly result in extensive debris ejection or fragmentation. Fortunately, bodies of this size may be addressed by impactors."* §5 asks for the methods to be modelled as a **spectrum across lead time**; this measures where on that spectrum the campaign's own body sits instead of restating the spectrum, and the answer is that the nuclear term is the wrong tool for *this* rock and says so. The table lives in the binding (`deflection_methods_compared_at_one_bar_on_the_real_threat`) because core must not learn about the threat body, and the core-side table's doc now points at it as the one a frontend should quote. **This is the J2 pair's lesson recurring**: a per-term row has to be measured on the seed it will be displayed against, not on whichever body the literature used.
- **Kernel-free, and the suite was run with `ASTEROID_REQUIRE_KERNELS=1`.** The new term composes nothing that needs an ephemeris, so its seven tests are pure arithmetic against published numbers. The full workspace nonetheless ran under the require-kernels flag — 165 core in **111 s**, the capstone in 36 s, 23 gdext in 173 s — because the silent-skip trap has made two verification claims here vacuous before, and *runtime is the only tell*.
- **Not yet on the frontend.** This batch is core-only: no `[N]` key, no force-menu row, no panel. The porkchop view produced three bugs in two batches that every test passed and only a screenshot caught, so a display for this is its own batch with its own picture. (Keys already taken: `1`–`4`, `C`, `P`, `E`, `M`, `L`, `O`.)
- **What the tractor batch already knows, so it is not rediscovered.** Checked while scoping this half, and it is the one thing that will actually cost time: **`DeflectionScenario<'a>` holds `force: &'a dyn ForceModel` for the whole scenario lifetime** (`deflection.rs:223`, used by `deflected_trajectory`). Every solver here varies the *initial state* under a fixed field. A tractor duration solve is the opposite — each probe re-propagates under a **different force model** (the thrust window changes), not a different state. So the tractor needs an API that takes a force per probe; the `*_with(force, …)` constructors already in the file are the precedent to follow rather than an invention, but it is a real signature change and should be scoped deliberately rather than discovered mid-batch. That is also where the advisor's *"don't fork bracket-and-bisect three ways"* warning finally bites: nuclear escaped it because a chosen direction makes yield a closed-form invert, and duration has no such escape.

### The deflection spectrum, tractor half — 2026-07-27 session (§5's gentle end, and the solve axis `required_dv` structurally could not express)

Closes the deflection-method spectrum, and with it the first of Phase 2's two
remaining checklist items. §5 asks for deflection modelled *as a spectrum across
lead time* — gravity tractor at the gentle end, kinetic impactor in the middle,
nuclear standoff at the top. The nuclear half landed earlier the same day; this
is the other end, and it is a different *shape* of thing rather than a third
entry in the same list.

- **Nuclear was an impulse, the tractor is a force — and it is the first term in
  `forces/` with a time window.** Gravity and sunlight do not switch off, so every
  existing term is on for the whole integration. A tractor is a *mission*: it
  arrives, tugs, and leaves. `TowWindow` is that parameter, and it is why the
  deflection layer needed a new solve axis rather than a new coefficient.

- **There was no coefficient to source, and recognising that early saved the
  batch a repeat of the nuclear source hunt.** The standoff term needed Dearborn
  because momentum-per-joule is simulation-derived and unobtainable from first
  principles. A tractor's tow is `G·m_sc/d²` — Newton, nothing fitted. Lu & Love
  2005's quoted rate turns out to *be* that: their
  `Δv = 4.2e-3·(m/2e4 kg)·(d/100 m)^-2 m/s per year` reproduces our
  `G·m/d²·yr` to **0.30 %**, the gap being their two-significant-figure rounding
  of 4.212. So the paper was fetched for its **configuration** (20 t hovering at
  `d/r = 1.5` over a 200 m, 2 g/cm³ body) and its **cant bookkeeping**, and both
  halves get an independent published anchor: the tow rate above, and
  `T = G·M·m/(d²·cos[sin⁻¹(r/d)+φ]) = 1.052 N` against the paper's stated
  *"total thrust T = 1 N"*.

- **The cant angle is a thrust penalty, never a weaker tug — and the paper's own
  equation says so.** `T·cos[sin⁻¹(r/d)+φ] = G·M·m/d²` puts the cant on the left
  with the thrust; the gravitational attraction on the right has no `φ` in it.
  Canting makes the *engines work harder*, it does not make the *gravity weaker*,
  because gravity does not know where the nozzles point. A `cos(cant)` factor on
  the tow would look conservative while silently understating every delivered Δv
  in the project — the `payload_kg`-means-two-things defect in a new place. The
  split is enforced by signatures rather than by comment: `tow_acceleration()`
  cannot see the cant angle *or* the asteroid mass it would need, and
  `station_keeping_thrust_n()` is the one place asteroid mass legitimately enters
  the module. A test pins that widening the plume moves the thrust and leaves the
  tow bit-for-bit identical.

- **"It is just Yarkovsky with a window" is the validation asset, not the
  criticism it sounds like.** Station-keeping holds `d` fixed by construction, so
  unlike Yarkovsky and SRP the tow does *not* fade with heliocentric distance —
  which makes it exactly the `d = 0` case of the Yarkovsky `A2·(r₀/r)^d`
  parametrization. So the term needs **no new oracle**: the Gauss-planetary-
  equation machinery, with its uniform-in-mean-anomaly weighting already validated
  against a closed form, judges both. It moved out of `yarkovsky.rs`'s private
  test module into `forces/secular_oracle.rs` — one implementation, two callers,
  rather than two copies free to drift (the reason `SB441_BODIES` has a drift test).
  A `d = 0` circular closed form (`da/dt = 2·a_T/n`) was added so the tractor's
  exponent is not being exercised for the first time by the very test it judges.

- **The window edge was measured rather than assumed, and the measurement is
  sharper than the worry.** A hard on/off edge is a derivative discontinuity
  landing inside whatever sub-step the adaptive driver is taking. Measured on a
  free particle where nothing but the edges can be responsible:

  ```
                       rtol/atol 1e-9 (shipping)   rtol 1e-13 / atol 1e-6
   edges inside steps          -1.0e-4                     +6.3e-3
   edges on boundaries         -9.2e-10                    -9.2e-10
  ```

  A discontinuity does **not** defeat the error controller — it converts a
  *tolerance* into a *systematic* Δv error. Aligned to a step boundary no step
  contains a discontinuity at all and the answer is exact at any tolerance; five
  orders separate the rows at one tolerance. That decided the solver's design:
  **leave window edges free**. The `-1.0e-4` is the pessimistic end (six enormous
  steps, no gravity), it is ~1e-6 m/s on the campaign's tow, and snapping edges to
  the snapshot cadence would buy it back only by *quantizing the duration
  bisection to the cadence*. Recorded so it reads as a decision.

- **`DeflectionScenario` could not express this solve, and the fix was scoped
  before the batch rather than discovered inside it.** The type holds
  `force: &'a dyn ForceModel` for its whole lifetime; every solver on it varies
  the *initial state* under a fixed field. A tow-duration probe is the opposite —
  same state, a **different field** each time. `propagate_and_reduce(force, start,
  seed)` is the extracted body of `deflected_trajectory` that takes the field as
  an argument, and it is what made the second kind expressible.

- **`ForceSum`, not the `ForceRef` that was planned.** The intent was to box a
  borrowed base field into a `CompositeForce` alongside the tractor. That does not
  compile and the reason is worth recording: `CompositeForce` holds
  `Box<dyn ForceModel>`, which is implicitly `Box<dyn ForceModel + 'static>`, so a
  *borrowed* field cannot go in it at all — and Rust has no default lifetime
  parameters, so giving `CompositeForce` a `'a` would ripple through every use
  site. Summing two references (`ForceSum(base, tow)`) sidesteps it entirely and
  allocates nothing. It is still the decorator `forces/mod.rs` names as the reason
  `ForceModel` carries `Sync`; only its shape changed.

- **The duration solve is a *bounded* bisection, which is why the advisor's
  "don't fork bracket-and-bisect three ways" warning bit less hard than expected.**
  `required_dv` grows its impulse geometrically because there is no a-priori
  largest sensible impulse. A tow duration *has* an upper bound — the lead time
  before the encounter — so the bracket is `[0, cap]` from the outset: no seed to
  pick, no growth factor, and no expansion loop that could silently walk past the
  region where the response is monotone. What the two solvers genuinely share, the
  mapping from an encounter outcome onto the perigee scale (`NotHyperbolic` ⇒ a
  dead-centre hit, off-gate ⇒ `+∞`), was extracted to `perigee_scale` rather than
  copied, since two copies would be two places for that mapping to start reading a
  hit as an error.

- **The cap is anchored to the *nominal* encounter, and running out of it is an
  error rather than an answer.** Past the encounter, extra towing cannot move a
  flyby that has already happened, so the response flattens and a bisection on a
  flat function returns noise; a window edge inside the flyby would also perturb
  the geometry being measured. Hitting the cap raises `TowDurationCapped` carrying
  the cap *and* the perigee it reached. Returning the cap as "the required
  duration" would repeat a defect this codebase has already shipped once —
  `required_impactor_mass` handing back its seed mass verbatim when the seed
  already cleared, an upper bound reported as the requirement. A caller that
  cannot distinguish *"3.1 years is enough"* from *"12 years is not"* will read the
  second as the first, and the kernel-free suite pins the distinction with the
  same shape of test that caught it the first time.

- **Measured on the campaign's own rock, and the headline is a *different kind* of
  failure from the nuclear one.** The threat is 300 m / 2.83e10 kg at 2.00 g/cm³ —
  the same density Lu & Love assume, so the literature row and this one differ only
  in size. A 20-tonne tractor hovering at `d/r = 1.5` (225 m):

  | quantity | Lu & Love's 200 m body | this campaign's 300 m body |
  |---|---|---|
  | tow `G·m/d²` | 5.93e-11 m/s² | **2.64e-11 m/s²** (0.832 mm/s per year) |
  | station-keeping thrust | 1.05 N (paper: ~1 N) | **1.58 N** (cant 61.8°) |
  | Δv over the full 6.32 yr lead | — | **5.26 mm/s** |
  | Δv the curve requires at that lead | — | **66.2 mm/s** → **12.6× short** |

  The lead used is 8 orbital periods — the *cheapest* Δv on the whole sweep — so
  the shortfall is a **best** case, not a representative one. The nuclear term
  failed as the **wrong tool**: a burst sized for this rock exceeds its 0.159 m/s
  escape speed and disperses it, a regime change. The tractor fails as the **right
  tool at the wrong scale**: nothing about it is unphysical here, it is simply an
  order of magnitude too small, and the shortfall is a spacecraft-mass number. Of
  the three methods it is the one that comes closest to closing on its own terms.

- **And a feeble tractor does not merely fail — it deepens the hit.** Towing the
  full lead at 20 t moves the b-plane perigee from **3000.0 km to 2811.6 km**:
  188 km the *wrong* way. The nominal is a near-centre impact, so a tug this small
  walks the track *toward* Earth's centre rather than out the far side; perigee is a
  distance, so it dips toward zero before coming back up. This is asserted, not
  merely printed, because it is the concrete counter-example to *"perigee grows with
  tow duration"* — an assumption the solver is documented as **not** making. An
  earlier draft of that doc claimed monotonicity over the capped bracket; the real
  field falsified it and the doc was corrected to state what bisection actually
  needs: a **single crossing of the target level**, which holds because the dip goes
  further *below* the nominal while any sane target sits well *above* it (20 000 km
  against a 3000 km nominal).

- **The scale that does close it, and how it was reached without a third solver.**
  The tow is *exactly* linear in spacecraft mass, so the closing mass is arithmetic
  rather than a search: 20 t × 12.6 ≈ **252 t**. That is an estimate of the mass
  matching the required *Δv*, and slightly optimistic about the *perigee* bar, since
  a distributed tug arrives later on average than an impulse. So it is checked
  rather than trusted: at 2× that (504 t) the duration solve wants **3.81 yr of
  towing**, 60 % of the available lead, and the answer round-trips on the shipping
  force model — 3.81 yr reaches **20 008.8 km** against the 20 000 km bar, while
  20 % less reaches only 16 655 km. Converging within 9 km of the bar is the
  bisection working.

- **Cost was measured before a solve was wired on top of it**, which changed the
  test's design. One tow probe is **12.4 s** (a bare propagation), but one
  `required_dv_along_track` at this lead is **236.6 s** — the dv solve, not the tow
  probes, would have dominated. Since the nuclear comparison already solves this
  exact lead live and pins it against `curve.json`, the tractor test **reuses** that
  constant instead of re-paying 237 s for an identical number, and the constant was
  promoted from a local `const` inside one test to module scope so the two cannot
  drift. Whole test: **171 s**.

### The tractor on the frontend — 2026-07-27 session (`[K]`, six knobs, and the cheap model that had to be scored before it could be shown)

The tractor half above is core-only and answers one question: *does a Lu & Love
tractor deflect this rock?* (No — 12.6× short.) That is a dead end to read and an
interesting thing to **operate**, because the reason it fails is a scale rather
than a physics, and every lever that changes that scale is free to evaluate. So
`[K]` opens a bench with six live knobs — and the design work was almost entirely
in deciding *what number it is honest to print while a key is held*.

- **A live margin cannot be built from `a·T`, and the project's own note said so
  before the panel existed.** The campaign test already documented the delivered
  Δv as an *upper* bound — a tug spread over the lead arrives later on average
  than an impulse at its start, and late Δv buys less displacement. Turning that
  caveat into a live readout would have systematically flattered every
  configuration: **measured +21 %** at the one point where a real-field answer
  exists. What ships instead is the **impulsive equivalent**

  ```text
  Δv_eff = a · T · (1 − T / 2L)
  ```

  which is *not* a fitted correction — it is the same linear response `f(τ) ∝ τ`
  that produces the `1/lead` law, integrated across the tow window instead of
  evaluated at a point. One model underwrites both halves of the panel. Its
  cleanest consequence is worth stating on its own: **towing the entire lead is
  worth exactly half its delivered Δv**, which is why starting early beats towing
  hard.

- **Which way a cheap model is wrong matters more than how wrong it is.** Scored
  at the calibration point the campaign already owns (504 t towing 3.81 yr of a
  6.32 yr lead, whose real-field perigee lands on the bar), the two candidates are
  `a·T` → **1.205×** and the equivalent → **0.842×**. The shipped one reads
  **16 % low**: it calls a tractor short when the field says it just clears. That
  direction is the reason it ships rather than a tuned version — a deflection
  readout that errs toward "not enough" is safe and one that errs toward "enough"
  is not — and a test pins **both signs**, so a future edit cannot quietly flip
  the estimate to the flattering side.

- **The required-Δv law got a measured validity floor, and the test proves the
  floor rather than asserting it.** `Δv(n) ≈ Δv(1)/n` holds to **0.1 %** between
  one and two orbits and 3.9 % out at eight — but at *half* an orbit the product
  `Δv·lead` collapses to 58 % of its value, because a sub-orbital arc has not had
  time to turn a period change into along-track drift. Extrapolating there is
  **1.73× wrong**, measured. So below one orbit the panel prints no requirement
  and no margin at all — *absent*, not zero — and `required_dv_matches_curve_json`
  asserts the law really does fail below the floor, so the constant cannot be
  "tidied" away by someone who reads it as merely defensive.

- **`CURVE_JSON_DV_AT_8_PERIODS` had to leave `#[cfg(test)]`.** It was fine as a
  test constant while only tests cited it, and became the blocker the moment a
  *readout* needed the same number: the release build could not see it. Promoted
  to `REQUIRED_DV_AT_ONE_PERIOD` / `REQUIRED_DV_AT_EIGHT_PERIODS` in shipping
  code. A general shape worth remembering — a number that is "just for tests"
  stops being that the moment anything user-facing wants to quote it.

- **The wall is not the surface.** The obvious lower bound for a hover-distance
  knob is "just outside the rock", and it is wrong. The cant is `sin⁻¹(r/d) + φ`
  and the thrust divides by its cosine, so station-keeping has no solution at all
  once the cant reaches 90° — at Lu & Love's 20° plume that is

  ```text
  d/r  <  1 / cos φ  =  1.064 body radii
  ```

  a band that clears the surface, **tows perfectly well**, and cannot be flown.
  The knob's first draft bottomed at 1.02 and was reachable in three keypresses;
  the core guard correctly returns `None` there, which the panel would have
  formatted as **`0.000 N THRUST`** — reading as station-keeping being *free* at
  precisely the distance where it is impossible. Now
  `min_hover_radii_for_station_keeping` is a closed form in the core, exported
  through `tractor_defaults()` so the bound is physics rather than a literal in
  GDScript, and the readout carries a separate `holds_station` flag instead of
  inferring one from a zero. The thrust divergence approaching that floor is the
  honest answer to *"why not hover closer for a bigger `1/d²` tow?"*, so it is
  left visible rather than smoothed away.

- **The knobs are a table, not variables — chosen against the planner's
  precedent.** The planner spends an input-action *pair* per parameter, which at
  six knobs would be twelve `project.godot` actions and twelve `main.gd` branches.
  The bench borrows the porkchop's cursor idiom instead: UP/DOWN selects a row,
  LEFT/RIGHT adjusts, `[E]` measures. **One new action for six knobs**, and a
  seventh is one row in `Sim.TRACTOR_KNOBS` with no edit to `main.gd` at all. The
  harness iterates that same table, so a new knob is covered by existing.

- **A user-tweakable rock radius, without inventing a third rock.** The standing
  rule (`threat_body_matches_the_srp_default`) is that the frontend must not
  restate the threat's body, and a radius knob is exactly the edit that breaks it
  silently. Scoped structurally instead of by comment: `tractor_hover_over`
  derives its own body mass from its own radius and hands it nowhere but the
  thrust formula, while `threat_mass_kg` — the porkchop's divisor and the SRP pin
  — stays a function of a constant.
  `the_tractor_radius_knob_does_not_reach_the_shipping_rock` moves the knob 4× and
  asserts the pinned mass has not budged. The physics it exposes is the good part:
  at fixed `d/r` the tow goes as `1/r²` while the **required Δv does not move at
  all** (the threat integrates as a test particle), so a rock four times the radius
  is sixteen times harder to tug for exactly the same Δv.

- **The direction knob turned out to carry the sharpest lesson, and nothing had
  ever probed it.** Every full-field probe in the tractor work — core tests and
  frontend alike — had run *prograde*. Running the other one, measured on the
  shipping configuration:

  ```text
  PROGRADE     perigee 3000 -> 2811 km   (-188 km, DEEPER)
  RETROGRADE   perigee 3000 -> 3348 km   (+348 km, OUTWARD)
  ```

  Not a symmetric sign flip — the retrograde move is nearly **twice** as large,
  and it is the same near-centre geometry that makes perigee non-monotone in tow
  duration: the b-plane point sits ~3000 km off Earth’s centre, so one direction
  walks it *toward* the centre (perigee dips before it can come back out) and the
  other walks it straight away (perigee grows from the first day). The same 20 t
  spacecraft either worsens the impact or eases it, one keypress apart, with
  nothing about the *tow* changed. The panel opens on **prograde deliberately**:
  it is the configuration the campaign measured, it is the one that fails, and it
  is one keypress from the one that helps. Seeding on the flattering direction
  would hide the point.

- **The signed perigee shift is a first-class readout, not a ratio.** A
  margin-only panel would show a user tuning steadily "toward closing" while the
  impact deepened. `shift_m` stays signed all the way through the binding, and the
  harness asserts both directions — the inward move for prograde and the opposite
  sign for retrograde, so neither branch can rot unnoticed.

- **Three end-stops now have executed probe paths, because none of them did.**
  `duty = 0` (refused in words, not as a raw "invalid tow duration 0 s" from
  inside the window constructor), the lead knob's 11.5-orbit maximum (probes fine,
  −225 km), and the hover knob's plume wall. Each is a setting a user reaches by
  holding a key, and each was one keypress from an unexercised code path.

- **What the frontend session actually cost, and it was not the physics.** Two
  staleness traps, both silent, now written up in §6: Godot loads `target/debug/`
  while the entire Rust loop builds `--release` (a new `#[func]` simply "does not
  exist"), and a new `class_name` is invisible until the editor rescans. Plus one
  language trap worth its own line — **GDScript's `%` operator has no `%e`**. It
  does not raise; it errors once per call from inside `_draw` and puts an error
  string where a number belongs, sixty times a second, on a panel that otherwise
  looks entirely fine. The values here span 1e-11 m/s² to 1e13 kg, so it is not
  avoidable by rounding; `_sci()` formats them.

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
- **Open / next:** ~~these three are the §9 teaching asteroids on-screen but not yet validated against Horizons~~ — that validation **landed as the Apophis capstone** (2026-07-21, *Tier 2 complete* below): with 1PN + Yarkovsky both on the real field, our own integration of Apophis is diffed against its Horizons truth table per force term. The display asteroids stay sampled scenery; the capstone is the *integrated* validation. J2 and Pluto-in-shipping are the only Tier-2 force terms still open.

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
