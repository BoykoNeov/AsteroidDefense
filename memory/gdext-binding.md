---
name: gdext-binding
description: "Phase-2 gdext core binding — crate layout, Godot 4.7 load workflow, placeholder/tool-context and DLL-lock gotchas"
metadata: 
  node_type: memory
  type: project
  originSessionId: 997359ca-2967-40c6-87b2-33f6f77bebd0
---

**Phase-2 GDExtension core binding — Commit 1 (toolchain gate) DONE.** Exposes
the headless [[project-overview]] `asteroid_core` to the Godot frontend so the
mission planner uses real validated physics instead of the placeholder GDScript
Kepler in `godot/scripts/sim.gd`. Advisor-steered thin-slice-first: prove the
load before wiring any physics.

**Crate:** `godot/rust/` = `asteroid_gdext`, a **workspace member** (added to
root `Cargo.toml` members). `[lib] crate-type = ["cdylib"]`; deps `godot =
"0.5.4"` + `asteroid_core = { path = "../../core" }`. **Dependency flows ONE
way** (godot → core); no Godot type ever links into core (§10 invariant).
Because it's a workspace member the DLL builds to the shared root
`target/<profile>/asteroid_gdext.dll`, so `godot/asteroid.gdextension` reaches it
via `res://../target/debug/...` (`res://` = the `godot/` project dir, one level
up from `target/`). `.gdextension` keys: `entry_symbol = "gdext_rust_init"`
(gdext default), `compatibility_minimum = 4.2`, **`reloadable = true`** (see
DLL-lock below). Commit-1 surface = one `#[derive(GodotClass)] #[class(base =
RefCounted, init)]` `AsteroidCore` with `#[func] fn core_version(&self) ->
GString` returning `CORE_VERSION`.

**Forward-compat PROVEN, not just trusted:** gdext bundles **API v4.6**; it
loads at **runtime v4.7** (banner: "Initialize godot-rust (API v4.6.stable,
runtime v4.7.stable)"). The "GDExtension loads in any newer Godot as long as
runtime ≥ API version" claim holds — no custom/`api-custom` build needed for 4.7.

**Two Godot-side gotchas that cost time (both settled):**
1. **Placeholder instances in the editor.** Instantiating a **non-`#[class(tool)]`**
   GDExtension class from an **editor script** (gdai-mcp `execute_editor_script`
   runs in tool context) yields a **placeholder** — the method call returns
   `null`/NIL and logs "Cannot call GDExtension method bind '…' on placeholder
   instance." This is NOT a binding bug. Real instances only exist in **game
   runtime**. Also: a newly-added `.gdextension` needs the editor to load it at
   **startup** — a mid-session `scan()` auto-loads it but as placeholders;
   `EditorInterface.restart_editor(true)` reboots, but editor instances stay
   placeholders regardless (tool context). **Verify the binding in game context**,
   not the editor: headless `godot --headless --path godot --script
   res://tests/test_gdext.gd` (SceneTree `_init`, `AsteroidCore.new()` →
   `core_version()`), the [[godot-mcp-script-cache]] "verify the right layer"
   lesson again. Gate PASS: `core_version() = '0.1.0'`.
2. **Windows DLL lock — SOLVED by `reloadable = true`.** With reloadable set,
   Godot loads a **`~`-prefixed copy** (`target/debug/~asteroid_gdext.dll`) and
   leaves the original `asteroid_gdext.dll` unlocked, so **`cargo build` is NEVER
   blocked with the editor open** (tested: rebuild succeeded, DLL timestamp
   updated, editor running). The advisor's predicted "2nd build blocks on the
   lock" does not bite here. (Hot-reloading *new native code into a running
   scene* may still need a scene re-run, but the build itself is free.)

**Commit 2 = "full ANISE positions" (user chose the bigger scope over the
advisor's smaller slice):** real DE440 body positions feed the WHOLE display,
not just the deflection physics. Advisor flag: real planets force a real threat
(the fabricated 2031-XK hit a *fake* Earth; against real Earth the number and the
drawn asteroid would diverge — the Commit-1 "number vs picture" split). So the
Godot timeline gets unified onto core's `RealFieldScenario`. Sequenced 2A/2B/2C,
each independently verifiable.

**Commit 2A DONE (f1ab0ee):** relocated `RealFieldScenario`/`ImpactorConfig`/
`EncounterFrame`/`ScenarioError` + encounter consts from `viewer/src/scenario.rs`
into **`core/src/scenario.rs`** so viewer AND binding share one validated scenario.
**serde stays OUT of core** (workspace policy): core `sweep` returns a plain
`SweepPoint`; viewer's `scenario.rs` is now a thin shim re-exporting the core
types + keeping serde `CurvePoint`/`CurveFile`/`DEFAULT_CURVE_JSON` (wraps
SweepPoint via `From`); `curve.rs` maps at the CurveFile seam. Added
`RealFieldScenario::build_with(cfg, Arc<Ephemeris>)` + `ephemeris()` accessor so
the binding loads kernels itself. Pure relocation, 74 core tests green.

**Commit 2B DONE (5a72a51):** binding exposes the real scenario. New godot-free
**`MissionCore`** (`godot/rust/src/mission_core.rs`) — unit-testable, the `Send`
payload 2C builds off-thread — holds `Arc<Ephemeris>` + `Option<RealFieldScenario>`;
thin **`Mission`** GDScript class marshals it, adds no logic. **Two-phase:**
`load()` reads kernels (~ms → body positions live) vs `build_scenario()` = the
expensive back-prop (→ Δv solver). Split exists precisely so the fast path works
in the **debug** DLL (headless test) without paying the slow build. **Frame:**
anise 0.10.3 has NO ecliptic frame/obliquity const → `icrf_km_to_ecliptic_au`
applies SPICE ECLIPJ2000 obliquity (84381.448″) in Rust; `Frame::from_ephem_j2000(
naif_id)` builds any body frame by NAIF id (no per-planet consts). **Every #[func]
panic-free:** kernel-missing/failed-lookup → bool/ZERO/`-1`, never a panic across
FFI. crate-type gained `rlib` for `cargo test`. **Verified:** `cargo test -p
asteroid_gdext --release` (kernel-gated) — required_dv matches curve.json <2% +
body_position matches a direct ephemeris call exactly; **headless game-context**
`test_gdext.gd` — Earth |z|=0.0000 AU proves the ecliptic rotation through the
real FFI. `build_profile()` confirms **headless/editor/play load the DEBUG DLL** →
2C must force release for the scenario build or it freezes ~400 s.

**Commit 2C-1 DONE (data surface only; threading + encounter-geo deferred):**
extended `MissionCore`/`Mission` with the threat + plan reads the display needs,
all headless-verifiable, **GDScript untouched**. New MissionCore state: `nominal_clock:
Option<Clock>` (Clock is `#[derive(Clone)]`) cloned **once** in `build_scenario` from
`scenario.deflection()?.nominal()` so per-frame reads never re-propagate the ~12-yr
nominal (building a `DeflectionScenario` propagates the full nominal — deflection.rs:279);
`plan: Option<PlanState{deflection_seconds, clock, perigee: Option<f64>}>`. New reads:
`asteroid_position_ecl_au(tdb)`/`asteroid_track_ecl_au(n)` (nominal), `set_plan(lead_s,
dv_along_track)` (rebuilds ds → `along_track_unit(seed)` heading → `deflected_trajectory`;
EXPENSIVE, per-nudge not per-frame), `deflected_position_ecl_au(tdb)`/`deflected_track_ecl_au(n)`,
`has_plan`/`is_clean_miss`/`deflected_perigee_m`/`plan_deflection_tdb_seconds`.

**Frame chain (the load-bearing bit):** the integrated asteroid `Clock` stores
**SSB-relative METRES** (barycentric ICRF — perturber_field integrates in SSB frame);
`body_position_ecl_au` returns **Sun-relative** ecliptic AU. So the threat must
subtract the Sun's SSB position to share the planets' frame: `helio_km = ast_ssb_m/1e3
− sun_ssb_km`, then the existing obliquity rotation. `ssb_m_to_helio_ecl_au` does this
(sun via `position_km(SUN_J2000, SSB_J2000, epoch)`). **Advisor's decisive gate**
(`asteroid_position_coincides_with_earth_at_impact`, kernel-gated release test): the
threat *hits Earth* at impact by construction, so `asteroid_position_ecl_au(impact_tdb)
≈ body_position_ecl_au(399, impact_tdb)` within ~1e-3 AU — one assertion that exercises
the Sun-subtraction (miss it → ~1e6 km gap), m↔km (~1000×), and the rotation together.
PASSES. Two advisor traps handled: (1) `deflected_perigee==None` is the **clean-miss
SUCCESS** case (pass left the scan gate), kept distinct from no-plan — `is_clean_miss()`
bool + `deflected_perigee_m()=-1` sentinel, NOT collapsed; (2) deflected query before
`t_defl` returns the **nominal** position (no retroactive nudge) — guarded in
position + track. `deflected_surface_respects_causality_and_sentinels` test pins both.
Verified: `cargo test -p asteroid_gdext --release` 5/5 (kernel-gated); headless
`test_gdext.gd` gate still PASS (new #[func]s register, extension loads in game ctx).

**DEFERRED out of 2C-1 (advisor-sliced):**
- **Threading** — NOT headless-verifiable (would test "editor stays responsive",
  needs GUI) + a soundness risk. **Memory's "MissionCore is Send" is UNVERIFIED and
  may be stale:** `PointMassGravity` owns `Box<dyn PerturberEphemeris>`, and a boxed
  trait object is NOT `Send` unless the trait declares `: Send` (it doesn't, as read).
  Before designing threading, run the compile assert `const _: fn() = || { fn
  assert_send<T:Send>(){} assert_send::<RealFieldScenario>(); };` — if it fails, add
  `: Send` to the core `PerturberEphemeris`/`ForceModel` traits (small, principled)
  rather than a Godot-side `Thread` (murkier gdext `&mut self`-from-worker borrow rules).
- **Encounter geo-frame** (`frame_from`/`EncounterFrame` → PackedVector3Array) — its
  return frame/units depend on how `encounter.gd` builds its b-plane basis, so
  co-design it with that consumer, not blind.

**PIVOT (user redirect, 2026-07-14): orrery/sandbox superseding the old 2C-2.**
User asked for (1) selectable + **reversible** time warp (scrub back/forth), (2) load &
visualize **several** asteroids/comets, (3) a **much longer** clock showing several comet
passes. Advisor: this *invalidates* old 2C-2's "remap clock onto bounded 2028→2040 span"
— that span can't do multi-pass. So 2C-2 is dropped; work re-planned as 3A–3D. User
decisions (overriding advisor's cheaper hybrid rec): body source = **Both** (synthetic
designer catalog now, real cataloged bodies later); architecture = **full real-core
everywhere** (every drawn body from the validated core, incl. a new free-propagation mode).
Discipline kept from advisor's cost warning: each body integrated **once** into a cached
dense-output Clock at load → scrub is cheap interpolation, never per-frame re-integration.
**Note for 3D:** `temp/AsteroidDefense/kernels/sb441-n16.bsp` (JPL numbered small-body SPK)
+ `linux_p1550p2650.440` are ALREADY present → real cataloged bodies far cheaper than feared.

**3A DONE (f3052de):** `RealFieldScenario::propagate_free(epoch0, seed: StateVector,
cadence_seconds, n_snapshots) -> Result<Clock, ScenarioError>` — exposes the scenario's
**owned** Tier-1 field (`self.force`) for arbitrary free-propagation, no field rebuild.
Seed is **SSB-relative** (integration frame); caller converts ecliptic/helio elements →
ICRF first (core stays ecliptic-free). Cadence sign = direction (negative → reverse-time).
Span bounded by kernel coverage (de440s ~1849–2150). Invalid args (zero cadence, n=0) +
out-of-coverage → `ScenarioError`, never panic. Thin wrapper over `Clock::propagate`
(clock.rs:103; already supports negative cadence, backward_clock test). Kernel-gated test
`propagate_free_matches_direct_step_in_the_field`: dense sub-snapshot vs direct Dop853 step
in same field rel<1e-8 + fwd/back span bookkeeping. Core suite 75+12+1 green.

**3B DONE (0812dcf):** MissionCore orrery catalog `bodies: Vec<OrreryBody{name,kind,clock}>`.
`add_synthetic_body(name, kind, OrbitalElements /*ecliptic, SI/rad*/, epoch0, cadence_s, n)`:
`elements.to_state(mu_sun)` (helio ecliptic) → **`ecliptic_to_icrf`** (new binding-side inverse
of `icrf_km_to_ecliptic_au`, `+ε` about X, unit-agnostic → rotates pos AND vel) → `+ Sun SSB
state` (`EphemerisPerturber::new(eph, SUN_J2000).state_at(epoch0)`, SI m/SSB) → SSB seed →
`sc.propagate_free` → cached Clock. `mu_sun = eph.sun_gm_m3_s2()` (SI convenience, ephemeris.rs:150).
Requires scenario built (field lives on it); bodies clear on `build_scenario`. Core reads:
`catalog_count/name/kind(i)`, `catalog_position_ecl_au(i,tdb)`, `catalog_track_ecl_au(i,n)`,
`catalog_span_tdb(i)->(lo,hi)`. Named **`catalog_*`** (not synthetic_*) so 3D real bodies share
the API. Mission #[func]s (AU/deg/days-facing, panic-free w/ up-front orbit validation guarding
the `OrbitalElements::new` inclination debug_assert): `add_synthetic_body(name,kind,a_au,e,
incl_deg,raan_deg,argp_deg,nu_deg,epoch0_tdb,span_days,cadence_days)->i64` (idx or -1+last_error),
`catalog_count`, `catalog_name/kind(i)`, `catalog_position_ecl_au(i,tdb)`, `catalog_track_ecl_au(i,
samples)`, `catalog_span_tdb(i)->PackedFloat64Array [lo,hi]` (f64 not Vector2 — f32 loses ~64s at
TDB~1e9). Decisive test `synthetic_body_seeds_and_frames_correctly`: planar i=0 ecliptic orbit
(a) reads back at seed epoch = AUTHORED pos (seed/read exact inverses, <1e-6 AU) + (b) |z|<0.02 AU
all along ICRF-integrated track (wrong rot → ~23° tilt) + on-ellipse band + metadata. gdext 6/6;
headless gate PASS w/ catalog #[func]s registered+FFI-safe (empty-catalog add→-1, reads→ZERO/[]).

**3C-1 DONE (d876cea) — reversible/selectable time + on-screen scrub timeline** (user chose
scrub-timeline + time-first staging). All in the placeholder `Sim` clock (decoupled, low-risk,
reused when 3C-2 swaps bodies). sim.gd: `time_dir` ±1 + `reverse()`; WARP_STEPS widened to
years/sec (…365,1095,3650 d/s); wide window `T_MIN=-3650`/`T_MAX=40000` days (~1849-style ±110yr
about 2031 epoch), clamped never wrapped; `_process` fires events only while ADVANCING (reverse/
scrub un-fire silently → no log spam); `scrub_frac`/`clock_frac`; `warp_label` shows Y/S + "<<".
NEW **time_bar.gd** (`class_name TimeBar`): bottom-strip scrub timeline, phosphor-drawn, decade
ticks + IMP/LCH/INT pips + draggable playhead, `MOUSE_FILTER_STOP` + `_gui_input→Sim.scrub_frac`
(drag in strip scrubs, drag elsewhere still reaches orbit camera). project.godot: `time_reverse`=B.
main.gd: instantiate/show/layout TimeBar (bottom strip, NOT in the full-rect overlay list — has
`layout(vs)`), wire B. hud.gd: `BOTTOM_RESERVE=50` lifts console+help above the strip, added [B]REV.
**Gotcha proven:** editing project.godot while the editor is open leaves the editor's in-memory
InputMap STALE → gdai `simulate_input` rejects the new action ("not defined in project"), but the
launched game reads project.godot fresh so B works in-game; editor needs reload to inject it via MCP.
Also: a new `class_name` needs the editor global-class cache refreshed (`godot --headless --editor
--path godot --quit-after N`) before a separate headless run resolves the type. Verified via gdai
MCP play+screenshot: warp x3.0 Y/S, clock advances + events fire, scrub bar renders, playhead tracks.

**3C-2a DONE (7643a27 + 0258135 + 0f95e06) — real TDB clock + real DE440 planets, VERIFIED IN-GAME.**
Advisor reframe: slice on the **`load()`/`build_scenario()` seam**, not "clock then bodies" —
planets need only fast `load()` (no freeze, debug DLL fine, full span); threat+catalog need the
expensive build, which is what forces release DLL + freeze + bounded-span UX *all at once* → 3C-2b.

**P1 was a CONFIRMED break, not hypothetical:** `ASTEROID_DE_KERNEL` is set at **neither user nor
machine level** → NO launched game (double-click / editor Play / MCP) could find a kernel; display
would be blank. The gdext gate **passed by SKIPping** — green while the game was blind. Fixed:
`MissionCore::load_from(bsp,pca)` + `Mission.load_from` #[func]; new **`godot/scripts/kernels.gd`**
(`Kernels.resolve()`) → env → `user://kernels.cfg` → conventional dirs (`res://kernels`, beside
exe, `res://../../temp/AsteroidDefense/kernels` ← finds it on this machine). Gate now goes through
the resolver = tests the path the game takes. **Never put kernel paths in project.godot** (in git,
machine-local absolute).

**THE hazard, everywhere:** a failed/out-of-coverage lookup → `Vector3::ZERO` = **the SUN's
position** in this heliocentric frame → a broken body renders *sitting on the Sun*, NOT visibly
missing. Hence: clock clamps to real coverage; tests assert non-ZERO at **BOTH span edges**, never
mid-span only. (Sun id 10 is the one legit ZERO — never query it; it's drawn at the origin.)

**Measured (all previously folklore):**
- **de440s span = 1850–2149**, now **discovered by bisection at load** (`discover_span`, ~40 lookups)
  not hardcoded → mounting de441 (~1550–2650) isn't silently capped. `Mission.usable_span_tdb()`.
- **Mars 499 does NOT resolve** (de440s has no Mars geocentre segment) → display uses **4**
  (barycentre, within km of planet — harmless, unlike Earth's 4671 km EMB). Earth **399** never 3.
  Pinned by `display_naif_ids_resolve_across_the_whole_usable_span`.
- **`RealFieldScenario` is NOT `Send`** — compile-assert FAILS: `Box<dyn PerturberEphemeris>` has no
  `Send` bound → `Perturber`→`Vec`→`PointMassGravity`→`RealFieldScenario`. **Memory's old "MissionCore
  is Send" was WRONG.** 3C-2b's freeze is one `: Send` on the core trait from being fixable off-thread.
- **Impact epoch is a config INPUT, not solved**: `ImpactorConfig::default()` = impact **2040-01-01
  TDB**, `lead_years: 12` → epoch0 **2028-01-01**. Readable *without* the expensive build → new
  `ImpactorConfig::epoch0()` (core, used by `build_with` so no drift) + `Mission.default_impact_
  tdb_seconds()`/`default_epoch0_tdb_seconds()`. **The clock is already on the real timeline** → 3C-2b's
  threat drops on with ZERO churn.

**Why the mission layer is PARKED (user chose "park it honestly" over a standalone Kepler threat):**
NOT for the reason first assumed. A Kepler threat *would* still converge on real Earth (`_build_threat`
derives elements from `pos_ecl(earth_el, T_IMPACT)`). The real blocker: **`earth_el` has TWO consumers**
— the display AND the **f64** encounter math (`pos_ecl64`→`geo_km`→`close_approach`, + `cap_km`). Real
positions cross FFI only as **f32 `Vector3`** (~18 km @ 1 AU) → feeding them in puts an f32 floor under
the b-plane those helpers exist to protect. Keeping a private Kepler Earth instead ⇒ **two Earths**, threat
riding the invisible one. 3C-2b fixes it structurally: geometry stays *inside Rust*, only small residuals cross.

**Containment that worked:** `Sim`'s public API (`pos_ecl`/`pos3d`/`orbit_points`) UNCHANGED; what changed
is what a *body IS* — a Dict naming its `source` (`"ephem"`+`naif_id` → binding lookup; else Kepler),
dispatched in `pos_ecl`. hud/map2d/tag_layer/camera never learn the difference. `orbit_points` for ephem
bodies walks **one real period of the actual ephemeris** (not an ideal ellipse); map2d's `_orbit_trace`
rewritten off `el.m0`-poking onto `Sim.orbit_points`.

**ALSO exercise the DEGRADED path (kernels absent) — advisor caught it unrun.** Rename the kernels dir
aside + re-capture. It's the whole reason `load_from`/`kernels.gd` exist (fresh clones / other machines =
exactly where you can't watch), and it found 2 more instances of the SAME lie: `map2d._draw` had no
`bodies_online` guard (unlike tag_layer) → planets stacked on the Sun; the offline HUD panel hardcoded
"SOLAR FIELD IS LIVE". **Field and mission layer fail SEPARATELY — report each on its own evidence.**

**Verify-by-frame-capture beat headless** (gdai MCP was NOT connected — needs editor running w/ plugin).
Trick: `--script` mode does **NOT register autoloads** → main.gd's `Sim` won't compile; instead add a
temp **autoload** `Shot="*res://tests/_shot.gd"` to project.godot, run `godot --path godot --resolution
1600x900` (non-headless), `get_viewport().get_texture().get_image().save_png(...)`, then restore
project.godot. Read the PNG directly. **This caught 3 lies headless missed:** HUD "E-69001 DAYS"
countdown to a gone threat, event log "TRACKING 2031-XK P(IMPACT)=1.000", "E-4382" log stamps. Rule
learned: *empty/zeroed fields on an instrument panel read as measurements, not as absence.*
**60–90 fps on the DEBUG DLL** → 3C-2a needs no release build.

**Tests:** new `godot/tests/test_orrery.gd` (20 checks, drives `Sim._ready()` as the game does — real
2028→2040 campaign, 1849–2150 scrub, 8 planets non-ZERO at 4 epochs incl. both edges, Earth |z|=6e-5 AU,
orbit closes 1e-4 AU, dormancy). `test_sim.gd` RETAINED + running on its **own Kepler Earth** (planets are
real now) since it covers the placeholder math kept as 3C-2b's reference; both go when the real threat lands.
gdext 8/8, core 75+12+1.

**3C-2b INFRASTRUCTURE DONE (5e1a5ff, 6caa5b3, 2924734, 2198446) — GDScript wiring still ahead.**
"MEASURE FIRST" paid off: every number in the old plan was wrong, and the measurement re-sliced the work.

**Measured (release, temp/AsteroidDefense/kernels), before → after:**
`load()` 19 ms (unchanged) | `build_scenario` **19.75 s → 10.10 s** | `set_plan` **10.97 s → 0.85 s**
| `required_dv` **27.9 s → 18.3 s** (still slow — bracket+bisect; keep it OFF every live path, use the
cheap `plan_dv/miss_ld` estimate or curve.json) | `position()` 3 µs, `track(400)` 1.7 ms (both free).

**The 11 s `set_plan` was a DEFECT, not a cost** (advisor's "threat+planner is just GDScript wiring"
was refuted by this): `RealFieldScenario::deflection()` built its `DeflectionScenario` via
`DeflectionScenario::new`, which ALWAYS calls `Clock::propagate` → **re-flew the whole 12-yr nominal
cruise per call** = per planner nudge. Core's own docs already said not to; nothing enforced it. Fix:
`nominal_cache: OnceLock<Clock>` on `RealFieldScenario` (the owner of seed+force → no caller can pair a
foreign nominal) + `pub(crate) DeflectionScenario::with_nominal`; `new` = validate→propagate→delegate,
shared preconditions in `validate`. **Zero API change** → viewer+curve got it free. `build_scenario`
halved *without being touched* because `build_with` already warms the cache via its own round-trip check.
Test `nominal_is_cached_identically_and_deflection_stops_re_flying_it` pins BOTH halves: cached clock
**exactly** == fresh propagation (not "to a tolerance"), and `deflection()` <500 ms (~20× clear of both
the 10 s regression and the ms path).

**`Send` is DONE — the old "NOT Send" note was about the trait OBJECT, not the types.** `Ephemeris`/ANISE
`Almanac` were **already Send+Sync** (checked). Only `Box<dyn Trait>` erased it. So it was a bounds change,
not a restructure: `ForceModel`/`PerturberEphemeris`/`GeocentricState` are now **`: Send + Sync`**.
**`Send` ALONE IS NOT ENOUGH — don't "minimize" it back:** `ForceModel` is *decorated* (validation's
`Counting` eval-counter wraps `&'a dyn ForceModel`), and `&T: Send` requires `T: Sync`. Tried Send-only;
that test refuted it. Cost: that counter's `Cell<u64>` → `AtomicU64` (Relaxed). Compile-time assert
`a_built_scenario_and_its_ephemeris_can_cross_to_a_worker_thread` in core/scenario.rs.

**Debug-DLL speed SOLVED by profile override** (root Cargo.toml), NOT by pointing `windows.debug` at
release: `[profile.dev.package.asteroid_core] opt-level=3` + `[profile.dev.package."*"] opt-level=3`.
**Name `asteroid_core` explicitly** — it's a workspace member and `"*"` covers only non-workspace deps,
so the glob alone would leave the hot crate at opt-level 0. Measured: scenario-building test **10.70 s on
dev** vs 10.10 s release (was ~400 s). debug-assertions stay on. Binding stays debuggable, no release
compile per iteration.

**Threading shape (advisor's key catch — do NOT move the serving core):** `MissionCore` answers
`body_position_ecl_au` every frame; sending it to the worker would freeze the orrery 10 s = the exact
regression threading prevents. So: **clone `Arc<Ephemeris>` out, `BuiltScenario` back.** New
`BuiltScenario{scenario, nominal_clock, nominal_encounter}` + `BuiltScenario::build(Arc<Ephemeris>, cfg)`
(worker-side; carries everything expensive AND invariant) + `MissionCore::install(built)` +
`ephemeris_arc()`. `Mission` gained `begin_build_scenario()`/`poll_build()`/`is_building()` and
**deliberately LOST `build_scenario()`** — no blocking form reachable from GDScript. `poll_build` =
`try_recv` (frame-safe), honest about all 3 endings incl. worker death (Disconnected → error, never
poll forever). Rust-side blocking `MissionCore::build_scenario` kept `#[allow(dead_code)]` for tests/tools.

**`capture_radius_m()`/`nominal_perigee_m()` added — the honest verdict bar.** Safe = **perigee >
capture_radius**; NOT Earth's solid radius (focusing bends a geometric miss onto the surface) and NOT
`is_clean_miss` (left the scan gate — a far wider bar a safe plan needn't reach). Scanned once at build.
**Number worth knowing: capture radius = 11 311 km = 1.773 R⊕** (not ~1.18!). `cfg.v_rel_kms = 18` is the
speed **at the 3000 km impact point, deep in the well**, NOT v_inf: ε = v²/2 − μ/r = 29.13 km²/s² →
**v_inf = 7.63 km/s** → b_cap = R⊕√(1+(11.18/7.63)²) = 1.773 R⊕. Binding matches the hand derivation to
3 decimals. (Same reason the module wants v_rel ≥ ~15 km/s: escape at 3000 km is 16.3 km/s.) Test pins the
DERIVED value — a band fitted to observed output would ratify a bug.

**Verified in Godot (headless, debug DLL) — `begin_build_scenario()` returned in 16 ms while the build
took 10 388 ms** (502 polls saw it in flight); `set_plan` 877 ms; threat-Earth gap at impact **3003 km**
(= configured `b_offset_km`) inside the 11 311 km disc. gdext gate PASS (21 checks), test_orrery **19/19**
(the old "20/20" was a miscount — 19 on a clean tree too), test_sim 8/8, core **77**+12+1, gdext 9/9.
**GDScript `%e` does not exist** (like `%g`) — a `%.2e` gap printed literally in a PASSing message; report km.

**3C-2b IS DONE (865f253 binding + e3de7de GDScript) — `mission_online` IS TRUE, the threat is on screen.**
`ast_el` = the core's integrated trajectory via `asteroid_position_ecl_au`; the planner hands lead+Δv to
`Mission.set_plan` and reads a b-plane perigee back. GDScript owns **zero** orbital mechanics now.

**The gate is NOT one flag — that was the load-bearing design call** (advisor). Flipping `mission_online`
alone lights 4 subsystems and 3 would lie (comet = Kepler ellipse until the catalog; interceptor arc has no
Lambert; `encounter.gd` builds its own b-plane from the helpers this batch DELETED). So: `mission_online`
(true) + `comet_online`/`interceptor_online`/`encounter_online` (all **false**). Consumers gate on the one
that matches their real source. **`main.gd`'s `view_encounter` now gates on `encounter_online`** — it gated on
`not mission_online`, so flipping mission_online turned it into a LIVE CRASH PATH onto deleted methods.

**THE NEW ZERO-IS-THE-SUN INSTANCE (advisor caught it; it was NOT in the old handoff):** the clock clamps to
the **kernel** (~300 yr); the threat exists ~12 yr. Outside its arc `asteroid_position_ecl_au` → ZERO = **the
Sun** → an ungated threat parks on the Sun for ~96% of the scrub range. Fix: **`Mission.threat_span_tdb()`**
(new #[func], from `Clock::covered_span()` — read from the clock, never reconstructed from cfg) →
`Sim.T_THREAT_MIN/MAX` + **`Sim.threat_active(t)`**, which every drawing consumer asks before placing the
threat. `threat_range_km` returns **-1** off-arc (an ungated call returns the ~1 AU distance to the SUN and
looks entirely plausible).

**The clean-miss trap is handled AND exercised** (this was the blocking handoff item): `deflect_ok =
plan_clean_miss or perigee_km > cap_km`, decided in ONE place (`Sim._solve_plan`); every readout goes through
**`Sim.miss_label()`/`verdict_label()`/`req_dv_label()`** so no site can print the `-1` as a distance. Proof it
matters: test_orrery's 200 m/s @ 600 d case reports `>> OFF-SCALE / CLEAN MISS - THREAT RETIRED` — written the
obvious way that exact line reads `SURFACE IMPACT` at `-0.00 LD`.

**UX: user chose DEBOUNCE** (`PLAN_DEBOUNCE_S = 0.35`). Edits land instantly; the ~0.7 s solve is coalesced to
the end of a keypress burst. `plan_solving` is what stops the panel presenting the previous plan's verdict as
this one's — a pending verdict must also **not blink** (blinking is how the planner shouts IMPACT).

**Retro → NEGATIVE dv is CONFIRMED from the core, not assumed:** `MissionCore::set_plan` applies
`dv_along_track * along_track_unit(seed)` and `along_track_unit` returns **prograde v̂**, so the sign IS the
direction.

**Deleted, not ported** (as planned): `pos_ecl64`/`geo_km`/`geo_vel_kms`/`close_approach`/`elements_from_rv`
+ Sim.set_plan's Kepler math; **`test_sim.gd` retired** (its own doc said it would go). Two encounter pipelines
that must agree and can't be checked against each other is how a display starts disagreeing with its physics.

**`encounter.gd` LEFT UNTOUCHED — and that was decided empirically, not by reasoning.** I assumed its calls to
deleted `Sim.close_approach` would be a **parse error** breaking project load. **They are not:** GDScript
resolves autoload methods at RUNTIME. Proven by `godot --headless --path godot --quit-after N` loading clean.
So it is dead code, unreachable behind `encounter_online`, and 3C-2c rewrites its data path AND axis frame
against `EncounterFrame` anyway. (`--check-only --script` is USELESS for this — it doesn't register autoloads,
so every file reports "Identifier not found: Sim". **Load the project instead.**)

**HUD honesty:** target panel **dropped ECC/INC rather than fake them** — the core derives no e/i (it computes
`a` by **vis-viva only**, scenario.rs:343), and the old values were the placeholder's own constructor args
echoed back. SMA/PERIOD stay (real). Exposing real osculating elements = `OrbitalElements::from_state` exists
in core, but needs a heliocentric state + an **ICRF-vs-ecliptic inclination** decision (~23.4° apart) → clean
separate addition, deliberately not this batch. `Sim.R_E` → **6378.137** (equatorial) to match the R⊕ the core
computes its capture disc against.

**Godot testing lessons (both cost time):**
- **`_draw()` DOES run under `--headless`** — verified with a temp probe. So the HUD/planner/map2d draw paths
  ARE exercised by a plain headless run. Don't assume they aren't.
- **`solar_system` must build the threat on the new `Sim.mission_ready` signal, NOT `_ready()`** — the scenario
  is ~10 s away at scene load. The old `plan_changed → _rebuild_plan_visuals` connection lived inside the
  `if mission_online` `_ready` block, which now never runs; re-wire it there or the deflected track never
  updates on replan (advisor catch). `_line_im` now returns an empty mesh for empty points (a deflected track
  with no plan is a legitimate empty, and Godot errors on a surface closed with no vertices).

**Verified in-engine: test_orrery 44 checks / 0 fail** (drives Sim exactly as the game does: build off-thread
472 polls/14.6 s on the debug DLL, threat lands **3002 km** from Earth inside the 11 311 km disc, lookups past
the arc really return ZERO, deflected track empty until solved, debounce coalesces, solve **721 ms**). Game
runs clean headless. gdext gate PASS, core **77**+12+1, gdext **10/10**, viewer/validation green.
Pre-existing and left alone: fmt diffs in close_approach.rs / assist_reference.rs / mission_core.rs:834,
clippy `nonminimal_bool` at lib.rs:550 (`add_synthetic_body`).

**VERIFIED VISUALLY (376cc1a era, `_shot.gd` autoload + non-headless `--resolution 1600x900` → PNGs in
temp/AsteroidDefense/shots).** Advisor was right to gate the "threat on screen" claim on this: headless renders
nothing, and **`planner._draw`/`map2d._draw` run ZERO times in a passive headless run** (hidden until a
keypress), so the two panels most changed had never executed. What the pictures actually showed:
- **Loading state**: "ACQUIRING THREAT SOLUTION / INTEGRATING 12 YR OF REAL N-BODY MOTION" while the orrery is
  live and the clock runs — the threaded build doing exactly its job. Threat online after **9.8 s**.
- **Threat live**: real track drawn, panel `SMA a 0.8545 AU / PERIOD 288.5 D` (the core's vis-viva figures).
- **Planner**: `PROJ MISS 0.32 LD (124,091 KM)` / `VERDICT MISS - EARTH CLEAR` / `CAPTURE 0.029 LD (1.8 RE)`
  — 0.029 LD = 11 148 km ✔ the focused disc. Re-nudge to 200 m/s → `>> OFF-SCALE / CLEAN MISS`.
- **THE MONEY SHOT**: scrubbed to 2149 (110 yr past the arc) the threat is **GONE** — no diamond, no track,
  nothing on the Sun — panel reads `RANGE -- OUTSIDE TRACKED ARC / TRACK ARC 2028-01-01 .. 2040-03-01` while
  the planets still render. That is `threat_active` working; without it a "2031-XK <THREAT> P(IMPACT) 1.000"
  diamond would sit on the Sun for ~96% of the timeline.
- A deflected track at 0.32 LD is **sub-pixel at AU scale** (~0.0008 AU) — invisible is correct physics here,
  not a missing line. Don't "fix" it.
- Shot at frame 1 is BLACK — capture needs `await RenderingServer.frame_post_draw` *and* a few frames of
  warm-up. `_shot.gd` also needs `main.boot.dismiss()` to get past the boot overlay.

**3C-2c IS DONE (c2b2c00 core + a9041b3 binding + 2afb8e9 GDScript) — `encounter_online` TRUE, and it
FOUND A REAL BUG IN THE VERDICT.** Asking "what would the view draw?" is what surfaced it; the view was the
instrument, not the goal.

**THE VERDICT WAS WRONG — `perigee > capture_radius` is NEITHER coherent pair.** The core's `is_hit` is
**`b > b_capture`** (the UN-focused asymptotic miss vs the target enlarged for focusing), proven equivalent to
**`perigee > R⊕`** (the ALREADY-focused closest approach vs the solid body). They are equivalent *as pairs*
only: `b² = r_p² + 2μ·r_p/v_inf²`, so b ≤ b_cap ⟺ r_p ≤ R⊕. Mixing them charges for focusing **twice** →
~1.5× too strict. The old memory note ("safe = perigee > capture; NOT Earth's solid radius, focusing bends a
geometric miss onto the surface") **was the confusion itself** — that argument is exactly what b-vs-b_cap
already expresses. **Measured, reachable:** 0.2 m/s at one period lead → b=14 639 km clears the 11 311 km disc
by 2 941 km of daylight, perigee=9 319 km sits inside it → shipped code printed **SURFACE IMPACT over a
deflection that works**. Both are "miss distances in km" — why it survived. User chose **(b vs b_cap)**, keeping
1.773 R⊕ as the headline bar. `miss_ld` is now **b** too: PROJ MISS prints directly above CAPTURE and a player
reads them against each other, so they must be the same pair.

**The trap I nearly rationalized (advisor caught it):** the over-strict bar is *exactly* the bar that makes the
picture consistent if you draw the bent track against the b_cap disc — and I started reading that as evidence it
was intended. **Backwards.** The bent track legitimately enters the disc on a safe pass: b_cap is the
cross-section for the **asymptote piercing**, not for the curved path (which bottoms out at r_p < b). Fix the
drawing, not the physics. (Advisor also RETRACTED its own earlier "draw the b-point as the track's s=0
crossing" — that assumed the old straight-line Kepler tracks; the core's are n-body-bent, so their s=0 crossing
is a meaningless third quantity between r_p and b.)

**Core split (c2b2c00):** `frame_from_arcs` (samples already-flown trajectories, no propagation) + `frame_from`
(flies + delegates). Same shape as `with_nominal`. Reason: `set_plan` ALREADY flies the arc and keeps the Clock,
so `frame_from` would buy a 2nd ~0.85 s propagation against a 0.35 s debounce — the re-flying defect one level
out. **`DeflectedArc{clock, encounter, deflection_epoch}`** keeps the pair as ONE value: `frame_from`'s promise
(drawn track and its perigee from one propagation) is only as strong as the caller keeping them together once the
propagation moves out. `deflected: None` = pre-plan picture, `deflected` **EMPTY** (a zeroed track draws the
asteroid at Earth's centre = a direct hit, as the picture of "no plan"). Test: `frame_from_arcs` == `frame_from`
**bit-identical** (exact, not a tolerance — a tolerance is what waves through a physics change).

**Binding (a9041b3):** projection lives HERE (not core — display axes aren't physics; not GDScript — it owns
none). New: `deflected/nominal_impact_parameter_m`, `earth_radius_m`, `encounter_v_inf_m_s`, projected tracks,
`nominal/deflected_b_point_km`, `encounter_sample_span_tdb`. `PlanState` dropped its bare perigee for
{encounter, frame} from one `DeflectedArc`. Nominal frame built on the WORKER in `BuiltScenario` (~ms, invariant)
→ view opens on the threat instantly. **ALL-ICRF, load-bearing:** `icrf_km_to_ecliptic_au` is for the ORRERY
ONLY; tracks are geocentric ICRF and Ŝ is ICRF, so the pole is **`ecliptic_north_icrf()` = (0,−sinε,cosε)**, NOT
ecliptic (0,0,1). A mix-up wouldn't error — the plot tilts 23.4° and stays plausible. **The frame test with
teeth:** the far-field track sample's (ξ,ζ) must equal the b-point's (the asymptote pierces the b-plane AT B) —
the far sample is ~1e6 km down the s axis vs |B|~7e3 km, so an obliquity error spills ~4e5 km of depth into the
plotted plane = ~50× blowout. gdext **13/13**.

**Numbers now pinned:** v_inf **7.63 km/s**, capture **11 311 km = 1.773 R⊕**, nominal |B| **7 074 km**
(inside the disc = the hit), nominal perigee **3 003 km**, ENCOUNTER window ±1.5 d / 1400 samples (core consts,
sized for this plot).

**The picture states the physics:** nominal X inside the capture disc but **OUTSIDE Earth's circle** (|B| 0.0184
LD vs R⊕ 0.0166) — the asymptote would miss the planet and gravity reels it in, the whole §5 payload in one
frame. **`DEFAULT_HALF_LD = 0.15`** because the b-plane projection collapses the huge s component, so everything
lives within ~0.1 LD — the old 4.5 LD default would have made the disc 3 px. Screenshots DROVE the design, not
just checked it: the first draft piled PREDICTED IMPACT + b-label + track labels + ring labels on the same pixels
(a track's inbound end lands ON its own b-point, since (ξ,ζ)→B far out) → legend + radial captions + no rings
inside the capture disc.

**`_shot.gd` is KEPT now** (`godot/tests/_shot.gd`, inert unless registered as the `Shot` autoload; add to
project.godot, run `godot --path godot --resolution 1600x900`, restore). Don't delete it again — `_draw()` runs
headless but ONLY for VISIBLE nodes, and this panel is hidden until [3], so a passive headless run executes its
draw path **zero** times. That is how the old view shipped a whole phase disagreeing with its own physics.

**THE KERNEL-SKIP TRAP — a green Rust suite can be entirely hollow** (found 2026-07-17; the biggest lesson of
the batch, see the new §6 "gotcha that makes the whole suite lie" in HANDOFF.md). `cargo test` **without**
`ASTEROID_DE_KERNEL`/`ASTEROID_PLANETARY_CONSTANTS` exported silently skips every kernel-gated test (~half the
physics suite) and prints `13 passed; 0 failed`. The `eprintln!("skipping…")` guard **cannot be seen**: cargo
captures stderr and only releases it for *failing* tests. **The runtime is the only tell — 0.02 s vs 69 s
(gdext), 0.01 s vs 20 s (core). Real DE440 integration cannot happen in 20 ms.** This wasn't hypothetical: it
made *both* of this batch's verification claims vacuous — the `deflected_b_point_km` rescale was "confirmed" by a
test that never ran, and `frame_from_arcs_matches_frame_from` (the ONLY proof the `frame_from` split preserved
its output) had never executed once. Re-run with kernels: genuinely green, but by luck. **The machine HAD the
kernels** in `temp/AsteroidDefense/kernels/`; only the env was unset. GDScript is immune — `Kernels.resolve()`
does env → `user://kernels.cfg` → conventional dirs, so test_orrery always ran real physics. **Proposed, NOT
built** (advisor: unasked infra, propose don't fold in): mirror that resolver in the Rust test harness. A
`REQUIRE_KERNELS=1` panic flag is the weaker option — it relies on remembering the flag, which is the same
forgettability that caused this. Keep skip-green for genuinely kernel-less CI; close only the have-them-but-
didn't-point-at-them hole. **Verify a test can fail before believing it**: bypassing the rescale made it FAIL
(s=2.385 km vs |B|=14 639.795 → 1.33e-8 > the 1e-9 gate), which is what proved the gate had teeth.

**Deflected b-point is RESCALED, nominal is not** (`deflected_b_point_km`). Each pass has its **own** b-plane ⊥
its **own** Ŝ. The display frame is the nominal's, so the nominal's B lies in it exactly (s=0 by construction)
but the deflected B does not — a raw projection plots it at √(|B|²−s²), *inside* the |B| the panel prints. Since
the verdict IS |B| vs the capture radius, a mark off that radius could sit inside the dashed disc while the
readout says MISS. |B| is pinned; the direction is an unpinned display convention → keep the magnitude exact,
take only the bearing from the projection. Measured gap: **0.00%** (0.009° asymptote tilt — a nudge moves *where*
the rock arrives by years of leverage but barely rotates *how* it approaches). So it's a guarantee, not a fix:
right by construction beats right by a coincidence nobody re-measures. Test asserts the **plotted radius** (the
ξ/ζ actually drawn) for BOTH marks — the 3-vector norm would pass either way.

**Glyphs carry claims too.** The live marker used the b-points' diamond, which made the legend's blanket "MARKS =
ASYMPTOTE THROUGH THIS PLANE" **false** for it — the b-points are where a straight asymptote pierces the plane;
the marker is the rock's position *now*. Not cosmetic: it's the verdict bug in miniature (a picture quietly
asserting something untrue). Now a radar contact (dot-in-ring), legend names it only while on screen,
claim scoped to "X / DIAMOND". **The live-marker branch needs a committed plan + a scrub into the ±1.5 d window**
(shot 6 of `_shot.gd`) — no test and no passive run ever enters it; the campaign is 12 yr. **Pause before
scrubbing**: at warp the clock runs on through the settle frames and the first attempt landed 0.53 d past CA,
rock 1.3 LD out and off-plot (the gate working, not the branch). The payoff picture: the contact sits at perigee
0.024 LD **inside** its own b-point at 0.038 LD — b≠perigee, legible without reading a number.

## 3D (comet half) DONE 2026-07-20 — `comet_online` is true, and GDScript owns no orbital mechanics at all

**Scope was SPLIT on advisor's call** (user agreed): the comet is wiring over an API that already existed and
was already tested; **real NEOs are a different design** and are NOT done. `add_synthetic_body` *integrates
Keplerian elements* through the field, but `sb441-n16.bsp` **already contains** the ephemeris — the honest path
is reading it directly, the same `"ephem"` path the planets use extended to small-body NAIF ids, which is a new
catalog *source variant*, not more wiring. Don't bundle them.

**MEASURE-FIRST decided the placement, again.** `add_synthetic_body` is inline-on-the-main-thread by design.
Probe (release, temp kernels): `build_scenario` **11.2 s**, comet **2.0 s over 12 yr / 8.1 s over 45 yr** — both
far past the ~0.5 s an inline-after-install call could hide, so it **folds into the existing build worker**.
New free fn `seed_orrery_body(&Arc<Ephemeris>, &RealFieldScenario, …) -> OrreryBody` holds the seed math so the
worker and `add_synthetic_body` **cannot drift**; `BuiltScenario` grew `scenario_ref()`/`epoch0()` so the worker
can fly bodies through the field it just built; `install(built, bodies)` takes both because a new scenario
invalidates the old catalog anyway. Shipped span = **one orbit ≈ 22.6 yr (~4 s)**; a 2nd lap retraces the same
arc for another ~4 s.

**The comet is authored, not tuned by eye.** `display_comet` module: a=8 AU, e=0.9, i=28°, Ω=210°. Elements are
**true-anomaly only by design** (`elements.rs` has no mean-anomaly API), so wanting "perihelion near impact"
had to be converted by hand — M₀ = −2π(12.8/22.6) ≈ 2.725 rad → Kepler solve at e=0.9 → E ≈ 2.921 → **ν₀ =
176.8°** — and the derivation is spelled out in the doc comment. **Measured on the real perturbed field: 0.807
AU, +0.97 yr from impact** (vs +0.8 targeted two-body). The test re-measures it, so a careless edit to the seed
angle fails loudly instead of quietly parking the comet at aphelion for the whole campaign.

**GDScript's LAST Kepler is deleted.** The comet was the only remaining user of `_elements`/`_kepler_pos_ecl`/
`solve_kepler` and the Kepler fallback branches of `pos_ecl`/`orbit_points` — all gone (advisor predicted this
before I looked). Every drawn body now names a source: `ephem`/`threat`/`threat_defl`/`catalog`. **The fallback
now `push_error`s instead of returning ZERO** — returning ZERO for an unknown source is the trap itself.

**ZERO-is-the-Sun, 3rd instance — first one caught BEFORE shipping.** `catalog_position_ecl_au` returns
`Vector3::ZERO` out-of-span. The comet's 22.6 yr arc is <1/10 of the ~300 yr clock, so ungated it would sit on
the Sun for most of the timeline. Per-body gate `Sim.catalog_active(el, t)` off `catalog_span_tdb` (the flag is
set from **what the catalog actually holds**, not alongside `mission_online` — the 4-flags rule). test_orrery's
old `not sim.comet_online` assertion INVERTED, +8 new checks.

**Verified:** core 81 + gdext 14 (**70 s** = real physics, not a kernel skip) + validation all green;
test_orrery **0 failures**; and the two `_shot.gd` pictures that are the actual proof — `comet_1_on_arc`
(inbound 4.4 AU, tagged C/2029 K1 while the encounter plays out) and the money shot `comet_2_past_span_gone`
(2051: comet **absent**, planets untouched, nothing on the Sun). **The orbit polyline hides WITH the body** — advisor caught me
claiming it stayed drawn "matching the threat-track convention" when `_nom_orbit_line.visible = active`
means the threat hides its track too, so the claim was backwards AND unverified. The polyline is *safe*
either way (built once from the whole span, never queries the live clock, so it cannot collapse to the
Sun) — but an orbit drawn for a body the sim is not tracking still reads as a claim that it is. Lesson
repeated: don't assert a parallel to existing code without opening the existing code.

**NEXT — 3D (real-NEO half), NOT started:** `sb441-n16.bsp` (on disk) via a new SPK-backed catalog source, not
`add_synthetic_body`; teaching set is Apophis / Bennu / Didymos (HANDOFF §9). Interceptor stays cosmetic until a
Lambert solver exists — `interceptor_online` is the flag. `required_dv` remains ~18.3 s → keep OFF every live
path (`req_dv_label` is a labelled first-order estimate). Still open: osculating e/i for the HUD needs an
ICRF-vs-ecliptic inclination decision; ξ/ζ sign stays Tier-3 (3C-2c coexists with it — display axes +
rotation-invariant scalars only, so settling it later is free).
