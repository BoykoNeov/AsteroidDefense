---
name: tier2-forces
description: "Tier-2 force-model work — 1PN relativity + Yarkovsky + 16 sb441 asteroid perturbers + SRP ALL DONE (isolation oracles + wired into shipping field behind Tier2Config, all-off Default) AND the CAPSTONE DONE: Apophis vs Horizons .neo — GR cuts residual 5–175x, Yarkovsky halves yr-8, belt sub-km (0.552 km). SRP = radial sibling of Yarkovsky (effective-μ test), 8.36 km b-plane shift. Live frontend force-model menu ([P]) DONE: on-demand (NOT on build — that blocks the threat solution), Arc-shared scenario, per-term shift GR+55.55/Yark+5.10/belt+0.55/SRP+8.36 km. All Tier-2 menu items COMPLETE"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0a0b19eb-ba5d-4c0d-a10e-34a497cf5af2
  modified: 2026-07-20T22:17:12.525Z
---

Tier 2 (HANDOFF §5/§6/§8) = enabling additional force terms on the existing
DE440/441 field, each a toggleable `.with(...)` on the composable
[`CompositeForce`] — no structural change. Order: 1PN relativity → Yarkovsky →
SRP → J2 → 16 asteroid perturbers → Pluto-in-shipping → validate a real NEO's
own integration vs Horizons. Started 2026-07-20.

**1PN relativity — DONE, validated in isolation (not yet wired).**
`core/src/forces/relativity.rs`: PPN Schwarzschild Sun term at β=γ=1,
`a = μ/(c²r³)·[(4μ/r − v²)r + 4(r·v)v]`, r,v heliocentric.
- `SPEED_OF_LIGHT_M_S = 299_792_458.0` (SI-exact). `μ` is a **field passed in**,
  NOT hardcoded twice — production must hand it the SAME ANISE μ_sun the
  point-mass Sun term uses, or GR vs Newtonian silently disagree on μ_sun.
- Needs the Sun's full **state** (r AND v), so it has its own `CentralBodyState`
  trait + `FixedCentralBody::at_rest_origin()` (kernel-free), instead of bolting
  `velocity_at` onto the position-only `PerturberEphemeris`. The point_mass.rs
  doc already anticipated this split.
- **Validation = Mercury perihelion precession** (§6 isolation check), the
  advisor's 4 load-bearing guards all built in: (1) compare to closed form
  `6πμ/(c²a(1−e²))`/orbit with the SAME constants, not literal 42.98″; (2) a
  **Newtonian-only control run** (GR off) proving the signal is physics not
  integrator LRL drift — control ≪ signal is THE guard; (3) **stroboscopic**
  eccentricity(LRL)-vector sampling once per period + least-squares slope over
  40 orbits, not one-orbit differencing; (4) explicit prograde-sign assert (the
  `(r·v)v` sign-bug tell). Signal <2% off closed form, lands 40–46″/century.
- 5 kernel-free tests; full core suite 97 passed / 0 failed in 18.31 s with
  `ASTEROID_REQUIRE_KERNELS=1` (the runtime tell the gated half ran — see
  [[kernel-resolver]] and the kernel-skip trap in [[gdext-binding]]).

**Yarkovsky — DONE, validated in isolation (not yet wired).**
`core/src/forces/yarkovsky.rs`: JPL Sentry transverse A2 form (NOT a
thermophysical model), `a = A2·(r0/r)^d·t̂`, t̂ = ĥ×r̂ (prograde in-plane),
r0=1AU, d=2. A2 signed: >0 prograde→outward da/dt, <0 retrograde (Bennu)→inward.
Reuses `CentralBodyState` from the 1PN commit.
- **Oracle = secular ⟨da/dt⟩ = 2·A2·r0²/(n·a²(1−e²))** (§6). Advisor's make-or-break
  was the **time weighting**: Gauss da/dt integrand ∝ (1+e·cosν)³, so a
  uniform-in-TRUE-anomaly average is ~10% wrong at e≈0.2. Fixed by sampling the
  oracle uniformly in **MEAN anomaly** (= uniform in time). Cross-checked 2 ways:
  numerical uniform-M avg vs closed form <1e-4 (e=0/0.2/0.45); integration-measured
  drift vs time-averaged oracle <1% at e=0.2 (uniform-ν would be ~10% off → the
  test discriminates).
- Guards mirror 1PN: circular e=0 de-risk case, A2=0 **control run** (drift ≪
  signal), prograde+retrograde **sign** pair, algebraic test (a·r̂=0, |a|=A2(r0/r)²,
  direction ĥ×r̂ NOT v̂ — the common wrong impl). A2 amplified above Bennu's
  physical ~1e-13 for SNR (validates form/sign/units not magnitude, stays linear).
  Bennu numeric anchor DROPPED not recalled-from-memory (algebraic test guards units).
- 7 kernel-free tests; full core suite 104 passed / 0 in 18.57 s under
  `ASTEROID_REQUIRE_KERNELS=1`.

**WIRED into the shipping scenario — DONE 2026-07-20.** `core/src/scenario.rs`:
`RealFieldScenario.force` is now a [`CompositeForce`] (was bare `PointMassGravity`),
built by ONE `compose_force(eph, &Tier2Config)` helper shared by `build_with` and
the measurement path (so "GR on" can't mean two things). `Tier2Config { relativity:
bool, yarkovsky_a2: Option<f64> }` on `ImpactorConfig`, **all-off by Default** —
every downstream builder uses `ImpactorConfig::default()`, so the shipping demo is
byte-untouched. Sun's heliocentric r,v from an ephemeris-backed `CentralBodyState`
impl on `EphemerisPerturber` (mirrors the `GeocentricState` impl); 1PN μ_sun is the
same `eph.gm_km3_s2(SUN_J2000)·1e9` the point-mass Sun uses.
- **Verification = fixed-seed b-plane comparison, NOT rebuild** (advisor's key
  call): rebuilding with terms on back-propagates the seed through the terms-on
  field → reproduces the hit by construction → ZERO shift. So
  `nominal_encounter_with(&Tier2Config)` holds the built seed fixed, re-flies only
  the forward field. Test `tier2_terms_leave_the_bplane_unchanged_off_and_shift_it_on`
  asserts STRUCTURE not magnitude: (a) all-off re-fly == shipping perigee bit-for-bit
  (`0 + a_pointmass`); (b) GR shifts resolvably + stays a hit; (c) physical
  un-amplified A2=1e-13 shifts by nonzero-finite.
- **Measured:** 1PN perigee 3000.0 → 2944.5 km (**−55.6 km**, inside 11311 km cap =
  keyhole territory); Yarkovsky @ A2=1e-13 = **5.1 km** over ~12 yr. Reported honest,
  NOT amplified (the display-grade lie the advisor gated against). Core 105 passed /
  0 in 38.98 s under `ASTEROID_REQUIRE_KERNELS=1`; gdext 16 tests still read cap
  11311 / |B| 14639 to the digit (all-off bit-identity holds downstream). See
  [[gdext-binding]].

**CAPSTONE DONE — 2026-07-20.** `core/tests/capstone_neo_vs_horizons.rs` (first
kernel+table-gated *integration* test, public-API only): seed the integrator with
**Apophis'** own JPL state at a `.neo` sample, integrate our field forward ~8 yr,
measure the heliocentric position residual against JPL's raw held-out samples for
three fields — Tier-1 / +GR / +GR+Yarkovsky. It is a **residual curve, not a
"match"** (the module doc says so in as many words; the advisor's key framing).
- **Object = Apophis, not Bennu/Didymos** (SBDB checked live, not recalled): Apophis
  carries `A2 = −2.902e−14 au/d²` (σ=1.86e−16) in *exactly* `YarkovskyA2::standard`'s
  form (R0=1 au, exp 2) → `APOPHIS_A2_SI = −5.816e−13 m/s²`. Bennu's modern
  OSIRIS-REx solution models **SRP (AMRAT), not A2**; Didymos has **no non-grav**.
  Arc is **pre-2029** (seed idx 30 ≈2020, checks to idx ~2950 ≈2028) — the 2029
  flyby (~idx 3390) is the worst case the advisor said to avoid.
- **Two silent-killer hazards, both handled:** (1) FRAME — `.neo` is heliocentric,
  integrator is SSB; seed `r_ssb=r_helio+r_sun_ssb`, compare `r_helio=r_ssb−r_sun_ssb`,
  the SAME `EphemerisPerturber(SUN).state_at` both ways so the ~10⁶ km solar wobble
  cancels. **Proven correct empirically:** +GR reaches 0.57 km residual — a wrong
  wobble/sign would leave ALL configs ~10⁶ km off, sub-km is impossible unless the
  axis is right. (2) A2 units, above. Truth is `Neo::sample` RAW held-out, never
  the Hermite interp (else it measures self-consistency).
- **Measured (de440s, this machine):** 1PN relativity is the headline — cuts the
  residual **5×–175×** at every epoch (Tier-1 100–326 km → +GR 0.5–38 km); the term
  is not just self-consistent (Mercury) but pulls a REAL NEO an order of magnitude
  toward JPL's own relativistic solution. Yarkovsky is a **secular** signal: BELOW
  the model floor yrs 1–5 (adds noise, `Yk helps by` = −0.7…−7 km), CLEARS it as its
  t²-growing along-track drift outgrows the floor, **halving the year-8 residual
  (37.8→18.6 km)**. That it only emerges over a multi-year baseline is the honest
  result — and the thing that motivates enrolling the perturbers to lower the floor.
- Discriminator = control run (term-on reduces residual vs JPL), same shape as the
  isolation oracles. Full core suite: **105 lib + 1 capstone (5.04 s, ran not
  skipped) + 12 roundtrip, all green** under `ASTEROID_REQUIRE_KERNELS=1`. The
  residual FLOOR is real physics we omit: JPL's radial A1 (5e−13 au/d², ~1σ from
  zero, unmodeled) + the 16 sb441 bodies as forces (scenery, not enrolled).

**16 sb441 PERTURBERS ENROLLED — DONE 2026-07-20.** The belt promoted from scenery
to force perturbers. `core/src/perturber_field.rs`: `sb441_perturber_field(eph)`
mirrors `tier1_perturber_field` — one `PointMassGravity` over 16 `EphemerisPerturber`s
(NAIF 2000000+number) from a **mounted `sb441-n16.bsp`**; a 3rd `.with(...)` term,
`Tier2Config.asteroid_perturbers: bool` (all-off Default). **The masses were the
whole risk** and the GM source is NOT ANISE: sb441.bsp has positions only, pck11
resolves only 6/16 (and to a LATER solution). Hardcoded `SB441_PERTURBER_GM_AU3_DAY2`
= verbatim DE440 header GROUP 1041 `MA%04d` (au³/day², keyed by asteroid number —
the masses JPL *integrated the positions with*), **machine-verified by re-reading
the local `linux_p1550p2650.440` binary's CVAL array** (record-2 offset 8144, AU@
CVAL[10]/DENUM=440 pinned the layout; a WebFetch fast-model read MISALIGNED the
GROUP-1040-names‖1041-values arrays and gave wrong values → deterministic awk/binary
parse was the fix). Guard: Ceres/Pallas/Vesta match pck11 <1% (Vesta ~4 sig figs =
the au³/day²→km³/s² factor is right); Psyche/Europa/Davida differ 12–72% because
DE440 free-fit them (that's WHY hardcode the self-consistent set, not resolve) — test
asserts <1% on the 3 shared determinations only. Fail-loud: `build` mounts sb441 when
flag set + errors if absent (optional 646 MB kernel, outside both-or-nothing);
`sb441_perturber_field` probes every position up front, clear error names the missing
kernel. **Measured b-plane shift 0.552 km** over 12 yr (3000.0→2999.5, still a hit,
sub-km FLOOR reported honest). **Capstone +belt column**: perturbs +0.07…0.43 km yr1-7
but at yr8 sits WITHIN the unmodelled radial-A1 floor (Δ −0.037 km vs 18.6 km GR+Yk) —
does NOT clear it, asserts only that perturbers ACT (measure-and-report, Yarkovsky
discipline). gdext `SB441_BODIES`↔core table pinned by `scenery_and_force_perturber_lists_agree`;
`probe_sb441.rs` kept (16/16 pos vs 6/16 GM). Core 109 lib + 1 capstone(24s,ran) + 12
roundtrip, 0 failed under REQUIRE_KERNELS; clippy clean.

**SRP — DONE 2026-07-21.** `core/src/forces/srp.rs`: solar radiation pressure as the
**radial sibling of Yarkovsky** — `a = a₁·(r₀/r)²·r̂` (+r̂ outward, d=2 fixed), Sun
**position only** (no velocity, no Poynting-Robertson), reuses `CentralBodyState`.
`from_physical(cr, area_to_mass)` → `a₁ = (Φ/c)·C_r·(A/m)` (Φ=1361 W/m² solar const,
`RADIATION_PRESSURE_1AU_PA`); stores only `a₁` so impl is one 1/r² scale.
- **Isolation test = effective-μ reduction** (advisor's key call, the SRP analog of
  Mercury/da-dt): a radial outward 1/r² force just rescales gravity →
  `μ_eff = μ_sun(1−β)`, `β = a₁·r₀²/μ_sun`. A body launched at the μ_eff circular
  speed traces an EXACT circle (radius constant) and closes after the **LONGER**
  μ_eff period (outward SRP weakens gravity — the load-bearing sign). Measured
  **geometrically** (period + radius constancy), NOT osculating elements (vis-viva
  with μ_sun on a μ_eff orbit reports spurious e≠0). SRP-off control on the same
  state falls inward + doesn't close → SRP is load-bearing not integrator noise.
  Sign test first (`a·r̂>0`). Tested at exaggerated β=0.02, ships β≈2.5e-9.
- Wired: `Tier2Config.srp: Option<SrpParams{cr, area_to_mass_m2_per_kg}>`,
  `SrpParams::sub_km_rock()` (300 m rock, C_r=1.3, A/m=2.5e-6 → β≈2.5e-9);
  `compose_force` adds `SolarRadiationPressure::from_physical` from the same
  SUN_J2000. **Measured b-plane shift 8.36 km** over 12 yr (radial → NO secular
  along-track drift, but a tiny constant accel still integrates to km at a fast
  encounter; larger than A2=1e-13's 5.1 km only by A/m choice, un-amplified). Case
  (d) added to `tier2_terms_…_shift_it_on`. 6 kernel-free srp tests.

**LIVE FRONTEND FORCE-MODEL MENU ([P]) — DONE 2026-07-21.** The GR/Yarkovsky/belt/SRP
shift toggle. `godot/scripts/tier2_panel.gd` (Tier2Panel, mirrors PlannerPanel CRT
draw); [P] opens, [G]/[Y]/[A]/[S] toggle each term to reveal its isolated perigee
shift (`nominal − shifted`, +INWARD). Per-term isolated (advisor default), numbers
precomputed so a toggle reads instantly.
- **THE PIVOT (advisor-gated, empirically forced):** first built it on-first-build
  (worker chained `.with_tier2_preview()`). WRONG — the preview runs BEFORE `install`,
  so its ~64 s (4× ~16 s propagations; **propagation dominates, scan is 0.2 s** —
  measured, so windowing the scan is dead) delayed `mission_online` = the threat
  solution + planner = core gameplay. In the real game (worker competes with render)
  time-to-threat blew past **200 s** (picture: "INTEGRATING THREAT... STAND BY" still
  up). Pivoted to **on-demand**: opening the menu kicks `begin_tier2_preview()` on a
  worker; threat solution back to ~18 s.
- **Arc-share, NOT rebuild** (advisor, gated on one compile `RealFieldScenario: Sync`
  — passes; every post-build method is `&self`, cache is `OnceLock`). `MissionCore.scenario`
  is now `Option<Arc<RealFieldScenario>>`; the preview worker holds a clone (refcount
  bump, the EXACT scenario the threat flew in) while the render thread reads it every
  frame. Free fn `measure_tier2_shifts(&scenario, mounted)` (was `with_tier2_preview`,
  removed as dead). gdext `Mission`: 2nd mpsc channel — `begin_tier2_preview`/
  `poll_tier2_preview`/`is_measuring_tier2` mirror the build channel;
  `tier2_shifted_perigee_m(term)`/`has_tier2_preview` FFI. Sim: `request_tier2_preview`
  on panel-open, `_poll_tier2_preview` each frame, honest "~2 MIN" text.
- **Belt is Option/None-not-0** (the display-grade lie in miniature): only measured
  when sb441 mounted; unmounted → -1 sentinel, panel says "UNAVAILABLE". FFI-verified
  BOTH ways: test_gdext (no sb441) → belt -1; non-headless full game (sb441 mounted) →
  belt +0.55 km. **Verified by two pictures + FFI gate**: layout shot (AWAITING branch)
  + numbers shot (all four ON: GR +55.55 / Yark +5.10 / belt +0.55 / SRP +8.36 km
  INWARD, nominal 3000/cap 11311) — `_draw` numbers branch ran no-error; belt 0.55 ==
  the capstone's recorded 0.552 to the digit. `_tier2_shot.gd` KEPT (like `_shot.gd`,
  autoload registration removed post-shot); `.godot/` gitignored so the Tier2Panel
  class-cache entry regenerates on editor import.

**Tier-2 menu COMPLETE.** Residual floor now = planets' GR + JPL's radial A1, both
unmodelled.

**2026-07-27 — the last two deferred terms CLOSED, both with numbers.**

**J2 (`core/src/forces/oblateness.rs`)** — deferred all through Tier 2 as "negligible
heliocentrically", which is TRUE (`1/r⁴` falloff) and is exactly why it had to be
**measured at the encounter**: essentially all its effect is bought in the minutes
inside a few R⊕. **Measured shift 1.33 km — bigger than the entire 16-body belt
(0.55 km).** Validated by the closed-form **nodal regression**
`dΩ/dt = −(3/2)·n·J2·(R_eq/p)²·cos i` to <2%, same guard structure as 1PN/Yarkovsky:
J2=0 **control run**, explicit **retrograde sign pair** (cos i<0 → node ADVANCES), plus
3 algebraic pins (inward over the equator, **outward over the pole at 2× the
magnitude**, purely axial at the magic latitude sinφ=1/√5). That last pin **caught my
own sloppy doc claim**: at the magic latitude the bracket's `r̂` COEFFICIENT vanishes,
NOT `a·r̂` (k̂ isn't ⊥ r̂ there) → the test pins `a × k̂ = 0`. Kernel-free, 8/8.
- **The pole is a PARAMETER, not ẑ**: `BodyPole` trait + `Ephemeris::pole_unit_icrf`
  reads ANISE's orientation DCM's **third ROW** (`v_body = R·v_icrf` ⇒ body-ẑ back in
  ICRF is `Rᵀẑ`). Probed: exactly ẑ at J2000, **0.2228° off at 2040**, 0.5570° at 2100
  — matches the IAU 0.557°/century model to 4 digits (independent confirmation the row
  extraction is right). `FixedPole` keeps isolation tests kernel-free.
- **J2 and R_eq are a PAIR** (physics carries `J2·R_eq²`; mixing solutions = silent
  scale error) — both verbatim from the DE440 header (`J2E = 0.00108262539`,
  `RE = 6378.1366` km), same machine-verified CVAL path as the sb441 GMs. Makes
  `EARTH_EQUATORIAL_RADIUS_M_DE440` (6378136.6 m) deliberately DISTINCT from WGS-84
  `geometry::EARTH_EQUATORIAL_RADIUS_M` (6378137.0 m) — different roles, not
  interchangeable.

**Pluto** — the blocker was REAL: `pck11.pca` resolves **no** Pluto GM (`ID 9 not in
look up table`, PROBED not assumed). The DE440 header has one: `GM9 =
2.175096464893358e-12` au³/day² → **975.500 km³/s²** (Pluto+Charon SYSTEM, as NAIF 9
must be). Wired as a toggle and measured: **0.0006 km (60 cm) over the 12-yr
campaign**. §5's own criterion was "flip to 11-in-shipping IF the growing-with-lead-
time cost proves to matter" — it doesn't (2 orders below the belt's sub-km floor), so
**the shipping field STAYS AT TEN BODIES**, now on a measurement instead of batch-2c's
"plausibly ~km" extrapolation. **The 0.6 m reads as signal only because the terms-OFF
re-fly reproduces the shipping perigee BIT-FOR-BIT** — without that identity the number
would be meaningless.

**All Tier-2 perigee shifts, one campaign:** GR 55.6 km · SRP 8.36 km · **J2 1.33 km**
· belt 0.55 km · Pluto 0.0006 km.

**THE J2 NUMBER GRAZES A VALIDITY BOUNDARY — measuring caught a claim I'd written.**
The J2 expansion is valid only OUTSIDE R_eq, and this scenario's nominal is a designed
**IMPACT** (closest approach 3000 km = INSIDE Earth). My first draft called that harmless
because "nothing downstream reads the sub-surface arc" — **WRONG**: the b-plane reduction
samples the state AT closest approach and infers v_inf from the **point-mass** energy
`v_inf² = v² − 2μ/r`, so J2's potential correction there (~J2·(R_eq/r)² ≈ 5e-3 of μ/r)
biases it ~1% → **capture radius 11311.3 → 11389.0 km (78 km, 0.69%)** against a perigee
shift of only 1.33 km. **The control that names the mechanism: 1PN leaves capture at
11311.3 to the digit** (its correction there is ~1e-9 relative) → it's J2's 1/r⁴ growth
inside the body, NOT the reduction. For any MISS geometry (perigee outside R_eq = every
deflected trajectory, the case that matters) the term is in-domain and none of this
arises. Read 1.33 km as "order a km on a boundary-grazing geometry"; **measuring J2 on a
real miss geometry is the follow-up**. Also explains 11389 vs the pinned 11311 km:
NOT two disagreeing code paths, one term evaluated out of domain.

**THE J2 PAIR — BOTH HALVES DONE 2026-07-27.** The "[P] menu lacks J2" leftover and the
"measure J2 on a real miss geometry" follow-up were ONE item: the menu subtracts every
shift from the SAME nominal baseline, so J2's row HAS to be measured on the impact seed
(measuring it elsewhere differences two unrelated geometries) — which means shipping the
row without the in-domain number would put a boundary-grazing figure under four that
aren't. Answer = ship the row, carry a **footnote**, not a second number in the table.

- **A designed miss is NOT buildable** — `build` verifies its impact round-trips, so
  `b_offset_km`=15000 returns `perigee 1.500e7 ≥ capture 7.711e6 (not a hit)`. The miss
  geometry has to be a **deflected** pass (which is also the case that matters).
  New core API `deflected_encounter_with(tier2, epoch, dv)`, sibling of
  `nominal_encounter_with`, both through one private `with_toggled_field`.
- **Geometry solved, not guessed** (`core/examples/probe_miss_geometry.rs`):
  **0.399625 m/s along-track 1 yr out → perigee 19139.2 km = 3.001 R_eq**, |B| 25064 vs
  capture 11312 = clean miss. COST TRAP: put the impulse at campaign start and every one
  of ~30 bisection steps is a full 12-yr flight — killed it after 18 min. A late impulse
  answers the same question (what's fixed is a PERIGEE; lead only sets the Δv price).
- **Result: J2 on the miss = 0.1196 km OUTWARD vs 1.3257 km INWARD on the impact.**
  Different magnitude AND sign — but do NOT sell the sign as proof of the domain problem
  (the Legendre factor depends on CA latitude, which two passes needn't share). The
  sufficient claim: 1.33 km is THAT geometry's number, not "what J2 does to a deflection".
- **The discriminating assertion.** "Shift is nonzero" is what the sibling test already
  asserts and passes anywhere = worthless here. The bias goes as `(μ/r)·J2·(R_eq/r)²` =
  **1/r³** (the μ/r is why it's not 1/r²), so the test asserts it COLLAPSES: model-free
  ≥10x, and against the 1/r³ prediction with slack for P₂ (which can shrink but not
  inflate it). Measured **0.6867% → 0.00143% = 480x** across a 6.4x wider perigee.
  1PN control on the same miss: capture unchanged, 4.4e-7 relative.
- `J2_DEFLECTED_MISS_PERIGEE_SHIFT_KM` (core, = −0.1196) → binding → panel footnote,
  **pinned to the live measurement by the test** so the caption can't drift (the
  SB441_BODIES treatment). Test `earth_j2_on_a_deflected_miss_is_in_domain`, ~62 s.
- **Frontend:** `[O]` for Oblateness (`[J]`=milestone_jump, keycode 74, checked not
  assumed). Five call sites; the two hand-kept lists were made to DERIVE from
  `TIER2_TERMS` — `tier2_on` populated in `Sim._ready`, and the shot harness reads its
  key/id pairs via `OS.find_keycode_from_string` (a second list is how a term gets a row,
  a measurement and an action while nothing ever presses its key). Panel columns now
  sized by `get_string_size` over the labels, not `chars × 0.6 × fs` — J2 is the longest
  row and cleared the guess by ~27 px; row count derived too. Preview now ~80 s
  (measured 119.8 s in-game on the debug DLL).
- **STALE-DLL TRAP:** `godot --path godot` runs DEBUG and loads `target/debug/`. Only the
  release binding had been rebuilt → the run went SILENT, no error. When a Godot
  verification run produces nothing, check which profile's DLL it loaded first.

HANDOFF: "The J2 pair — 2026-07-27 session".
See [[mission-porkchop]] for the same session's Lambert/launch-vehicle findings.
