---
name: godot-visual-layer
description: Phase-2 Godot retro-CRT visual layer built & screenshot-verified — 3D vector solar system, 2D radar plot, HUD, boot screen, mission timeline demo
metadata:
  type: project
---

Godot visual layer ("the game" surface) built 2026-07-14 and verified live via
gdai-mcp screenshots. User direction: **retro green/orange phosphor terminal
styling, but native high resolution and detail — style layer only, no
pixelation/downscale.** Godot = 32-bit visualizer + scenario surface; Rust
f64 core stays the engine (see [[godot-phase2-scaffold]]).

**Architecture (all code-built, minimal .tscn):** `main.tscn` is just a
Control + `scripts/main.gd`, which assembles at runtime:
SubViewportContainer(stretch, CRT ShaderMaterial) → SubViewport(own_world_3d)
→ [SolarSystem (Node3D), OrbitCameraRig, Map2D, TagLayer, HUD, BootScreen].
Everything (3D + HUD) renders inside the SubViewport so the CRT shader
governs the whole screen. **Gotcha: Controls parented directly to a
SubViewport do NOT size via anchors** — main.gd sizes them explicitly on
viewport.size_changed (first run had zero-size HUD/boot overlap).

**Files:** `godot/shaders/` crt.gdshader (phosphor mono-mix, sub-px scanlines,
barrel curvature, bleed, halo, noise/flicker, vignette; uniform `phosphor`
green↔amber via T key), glow_line.gdshader (spatial, EMISSION-only, `instance
uniform` line_color/energy — one shared material, per-instance
set_instance_shader_parameter), starfield.gdshader (POINT_SIZE points).
`godot/scripts/` sim.gd (autoload **Sim**), solar_system.gd, orbit_camera.gd
(drag/wheel/focus-follow), hud.gd, tag_layer.gd (unproject → upright screen
tags), map2d.gd (top-down radar: rings/sweep/orbit traces/range line),
boot.gd (typewriter POST, any-key or 5s auto dismiss), main.gd.

**Sim (display-grade placeholder, to be swapped for gdext core):** f32 Kepler
solver, J2000-ish planet elements Mercury→Jupiter; 1 AU = 10 units; ecliptic→
Godot map (x, z, −y). Threat 2031-XK a=0.855 AU **constructed from the impact
condition** (node at impact point, ω=180°, aphelion = Earth range at
T_IMPACT=1200 d) so tracks truly converge. Deflection = along-track burn at
T_INTERCEPT=1020 d modeled as Δa/a=2e-3 (display-exaggerated; HUD reports
~32 m/s, miss ~3.4 LD) with phase matched at burn → divergence is emergent.
Interceptor ATLAS-1: bezier transfer arc placeholder (→ Lambert from core
later). Comet C/2029 K1: a=8, e=0.9, GPUParticles3D anti-sunward tail
(local_coords=false; fine 0.014 quads, alpha 0.22 — big quads read as chunky
squares).

**Input via InputMap actions** (registered in ProjectSettings by editor
script, NOT raw keycodes) so gdai-mcp `simulate_input` can drive the game:
sim_pause(SPC) warp_up/down(./,) phosphor_toggle(T) view_3d(1) view_map(2)
focus_next(F) time_reset(R) milestone_jump(J → launch/intercept/impact slews,
Sim.jump marks past events consumed silently).

**Verified by screenshots:** boot→tactical 3D (green), cruise (transfer arc +
XFER bar), post-intercept (deflected vs NOMINAL TRK ghost visibly separated),
2D radar plot, amber theme, Earth close-up, comet tail. No runtime errors.

**Editor-side gotchas:** editing project.godot on disk while editor open →
editor's ProjectSettings.save() (e.g. from plugin/script) **clobbers manual
edits** — set settings via `execute_editor_script` + ProjectSettings API
instead. Parse errors "Sim not declared" appear until the autoload is
registered in the *editor's* ProjectSettings. `clear_output_logs` MCP tool
errored (harmless). Static funcs called via autoload instance warn on 4.7 —
made pos_ecl/ecl_to_godot instance methods.

> **SUPERSEDED (2026-07-17, Phase-2 3C-2c).** The section below describes the
> *placeholder* b-plane view and is kept for history. It is no longer what runs:
> `encounter.gd` now reads the core's `EncounterFrame` and owns no geometry. Every
> number below that this view computed for itself was **wrong** — it took `v_inf`
> from the closest-approach speed (3.17 km/s) rather than the hyperbolic excess
> (7.63 km/s), so its capture circle was 3.7 R⊕ where the real focused disc is
> **1.773 R⊕**. The f64 helpers it leaned on (`pos_ecl64`/`geo_km`/`geo_vel_kms`/
> `close_approach`) were deleted in 3C-2b, not ported: two encounter pipelines that
> must agree and cannot be checked against each other is how a display starts
> disagreeing with its own physics. See the `gdext-binding` memory for the current
> design — and for the verdict bug this rewrite exposed (`perigee > capture_radius`
> is neither coherent pair; the hit test is `b > b_capture`).

**Encounter/b-plane close-up view (key 3, DONE 2026-07-14, screenshot-verified) —
HISTORICAL, superseded by 3C-2c:**
`scripts/encounter.gd` (EncounterView), wired in main.gd as third view
(`view_encounter` action = KEY_3, registered via execute_editor_script since
editor was open — disk edits to project.godot get clobbered). Geocentric plot
on classic targeting axes: S = v_rel_hat at nominal CA, XI = S×N (N = ecliptic
north), ZETA = S×XI (≈south, drawn screen-down). **sim.gd grew f64 encounter
helpers** honoring the subtract-then-cast contract (GDScript scalars are f64;
only Vector3 is f32): `pos_ecl64` (PackedFloat64Array twin of pos_ecl — kept
duplicated off the hot path, keep math in sync), `geo_km` (diff in doubles,
cast small residual), `geo_vel_kms` (central diff), `close_approach` (ternary
search ±80 d of T_IMPACT). View: Earth disk (min 3 px) + dashed gravitational
capture circle b_c = R⊕√(1+(v_esc/v∞)²) ≈ 3.7 R⊕ at v∞ 3.17 km/s, LD range
rings, ±40 d track polylines (inbound bright/outbound dim, 5-d ticks),
b-vector by bisection on s=0 crossing, live asteroid diamond, encounter
solution readout, wheel zoom (0.05–30 LD half-span, _unhandled_input added
after camera rig so it wins the wheel; **wheel zoom UNTESTED** — simulate_input
does actions only). Verified pre-intercept (nominal |B| = 4 km → SURFACE
IMPACT blink) and post (B 2.34 LD diamond + nominal ghost). **Sim.miss_ld
changed:** now true CA distance from close_approach (2.3 LD), was separation
at T_IMPACT (3.4 LD) which disagreed with the plotted |B|. Gotchas: GDScript
format strings have **no %g** (prints literally — use String.num); editor logs
a stale "Identifier not found: Sim" for freshly scanned scripts, runtime clean.

**Next candidates:** gdext binding of core/ (f64→focus-residual contract),
scenario-designer UI surface, Moon + Earth-encounter zoom (moon marker on the
1 LD ring), sound (Geiger-style telemetry ticks), CRT phosphor persistence
(feedback buffer).

## Measured and given its phosphor — 2026-09-05 (frame-time harness, caches, world viewport, persistence)

**Architecture changed (main.gd):** the 3D world now renders in its own `world_vp` (own_world_3d,
**4× MSAA**) nested inside `persist_vp` (2D, `CLEAR_MODE_NEVER`, `use_hdr_2d`), whose single ColorRect runs
`shaders/phosphor_persist.gdshader`; the main viewport shows it as a TextureRect **under** the HUD/tags/2D
views, and the CRT shader still wraps everything. `OrbitCameraRig` stays in the main viewport (input
ordering — encounter.gd swallows the wheel first) and drives a `Camera3D` living in the world by global
transform (`attach_camera`); `_show_view` disables both render targets + `solar.set_process(false)` while
a 2D view is up and clears the accumulation once on return.

**Persistence is PEAK-HOLD, not an average.** First version `world·(1−keep)+prev·keep` made fast movers
vanish (drawn at 6 %) — `trails_1_max_warp.png` showed tags pointing at nothing. Shipped:
`max(world, prev·keep)` via `hint_screen_texture`, `keep = exp(−Δt/PERSIST_TAU=0.14 s)`, HDR target so
the decay reaches black. Static line work is exactly its own brightness.

**Perf method (the reusable part):** `godot/tests/_perf.gd` (autoload harness; prints frame ms,
`Sim.ffi_calls` per frame — a counter reset at the top of `Sim._process` — draw calls, plus a raw-lookup
microbench) driven by **`godot/tests/run_harness.ps1 -Harness _x.gd [-Headless]`** which registers the
autoload, runs, restores project.godot and kills only its own PID. **Run `-Headless` for CPU cost**: windowed,
the compositor pins every view to 8.41 ms/119 fps regardless of content. `Performance.TIME_PROCESS` is
garbage on this build (hundreds of ms beside an 8 ms frame) — dropped.

**What it found and what fixed it:** map2d re-walked six orbits from the ephemeris inside `_draw` every
frame (2.7 ms/planet trace; 764 native calls, 18.9 ms/frame) → cached traces (`_traces`, dropped on
mission_ready / plan_changed→deflected only) → **7.1 ms, 29 calls**. 3D layers asked the same positions
2–3× a frame (61 calls) → per-frame memo in `Sim.pos_ecl` keyed by name+exact epoch, cleared each
`_process`, orbit walks bypass it via `Sim._lookup_ecl` → 29 calls. `Sim.impact_point_ecl` read once on
install. Body spin is now per second, not per frame.

**Keyhole map `[H]` SEEN for the first time** (was "unrun"): 27 circles + 30 stacked captions = a thicket →
circle brightness by log keyhole width (3:4 solid), captions budgeted to the 7 widest in frame
(`KEYHOLE_LABELS`). Toggle verified (`enc_7_keyholes_off`).

**Traps (new):** (1) **Godot lingers after `quit()` at 100 % CPU** (plugin teardown) — it poisons timings
AND **holds the debug DLL** → `cargo build -p asteroid_gdext` fails `Access is denied (os error 5)` → Godot
runs the stale binding (`Nonexistent function 'bplane_frame_pinned'`). A `| tail` hid the failure and a
wrong epoch threshold in my wait loop let Godot launch mid-build. (2) **Other sessions' Godot processes
run on this box** (pebble_bed, River Basin) — kill only a PID you launched. (3) Non-ASCII in `.ps1` =
parse error under PS 5.1 (curly dash → quote). (4) `test_orrery`'s `polls > 1` raced a loaded machine;
now asserts `_ready()` < 3 s (measured 22 ms). 83/83.

**Follow-ups** are sized for a smaller model in `docs/plans/2026-09-05-visuals-performance-followups.md`
(parallelise mount+comet in the build worker to cut the 10–30 s startup wait; batched FFI positions; tag
de-collision; keyhole-caption vs b-caption collision; persistence key + belt smear at max warp; encounter
polylines; Tier-3 ellipse on the b-plane).

**Frontend speed — 2026-09-06 (plan tasks 1+2, neither as written).** **Startup wait 34.0 → 25.2 s
(−26%)** by flying the display comet on its OWN worker started *after* `install`, not on the build
worker. Measured first (`build_phase_timings`, `#[ignore]`d, release): mount 0.6 s warm ·
`BuiltScenario::build` 11.8 s = **10.7 s forward flight + 1.1 s remainder** · comet 4.1 s. The plan's
premises were wrong three ways — the mount is **not independent** (`build` consumes the mounted almanac
and the scenario keeps it; the `[P]` menu recomposes from it), the comet had **nowhere to overlap**
(`build_with` re-flies the nominal as its own hit check, so both propagations are done before the
"frame + scan" milliseconds), and a failed comet used to kill the whole threat solution (now warns;
`busy_worker` names the flight so a rebuild can't inherit a stale comet). `scenario_arc()` already
existed for this — **zero `core/` change**. `test_orrery.gd` now prints the trade: "the comet lights
from its own worker, 3740 ms after the threat"; 84 checks.
**Batched positions:** `body_positions_ecl_au(PackedInt64Array) -> PackedVector3Array`, one slot per id
**including misses** (a short answer slides every body onto its neighbour). ffi/frame 3D 29 → 6, map2d
5 → 2. **Shipped broken twice, both instructive:** (1) `var out := mission.…` cannot infer a type
through the untyped `mission` → autoload refused to load → **the pushed commit had no frontend at all**;
caught in one `test_orrery.gd` run. (2) Filling on a schedule (end of `_process`) made every frame pay a
crossing — proof needs no timing, **porkchop ffi/frame went 0 → 1** — and 24 lookups/frame through the
whole build wait fought the build worker: 34 s → 56 s. Now demand-driven from the first `pos_ecl` miss
**at the live clock only**; `_primed_t` claims the epoch *before* the batch, cleared with the memo (a
paused clock holds `t` still while the memo empties).
**Never claimed a frame-time result:** identical code paths read 12.6 / 84.6 / 161.9 / 68.5 µs for one
Earth lookup across four runs while the memo hit stayed 1.3–1.5 µs. Use the deterministic crossing
count; use the micro numbers as a **gauge for whether two runs are comparable at all**.
**New traps.** (a) A **parse error in an autoload hangs a headless run forever** — stderr says "Failed
to instantiate an autoload", stdout through a pipe never flushes, so it reads as "still working" (one
sat 1 h at 30 s CPU). Launch via `Start-Process -RedirectStandardOutput` with a timeout; read the `.err`.
Runner: `W:\temp\claude\AsteroidDefense\runs\run_orrery.ps1`. **`_shot.gd` hung the same way**, and it was the ONLY check that caught the comet split:
it reaches its comet section on `mission_online`, which no longer means the catalog is complete, so it
photographs an empty `comet_el` and dies inside an `await` chain — hanging instead of failing. It now
waits on `Sim._comet_pending`. **Run `_shot.gd` for any frontend change: it is the only thing that
proves a body is DRAWN** — `test_orrery.gd` proved `comet_online` flipped while the node could have
been absent. It confirmed node_visible true on arc / false past span, and the threat unchanged through
the new install path (|B|=14 639 km, cap=11 311 km). (b) The gdext suite needs
**`-- --test-threads=4`**: 37 tests × a 646 MB kernel exhausts commit and dies with `memory allocation
of 32726016 bytes failed`, which is not a test result. (c) `_ready()` blocks **~11 s in
`mission.load_from` on a COLD file cache, on the main thread** — bigger than the whole comet win, outside
the worker, ~0 warm so it only bites on the first launch after a boot. **CLOSED 2026-09-07 (item 7).**

**2026-09-07 — the kernel read went threaded, and the item's premise was wrong.** `load_from` reads
**de440s.bsp, 32 MB**, NOT "the 646 MB DE440" — the 646 MB file is sb441 and is only `is_file()`'d here.
Nothing had ever split `_ready`; it is split now and printed every run (`Sim.ready_phase_ms`): warm,
`load_from` is 26.6 of 29 ms and nothing else does I/O (font 1.4, resolve 0.6 — both ruled out, not
assumed). **The seconds are the disk, not the code:** `M:` is a spinning HDD, and `de440s.bsp` reads
4.9–12.9 MB/s unbuffered while the same platter streams a 200 MB file at 35.4 MB/s — it is fragmented.
ANISE is `std::fs::read` (heap, not mmap), so it is a real 32 MB read, 2.4–6.4 s off the device. NOT
antivirus (Defender caches its verdict; both reads were slow). **Lever that is yours, not the code's:
re-copy `de440s.bsp` to defragment it.** Fix: `Mission::begin_load`/`poll_load` (9th channel, the
earliest), `_ready` **29 → 2 ms**; no cold before/after claimed, because a cold cache cannot be made on
demand. `poll_load` false = *finished*, not *succeeded* — `is_loaded()` is the success test.

**(d) THE TRAP THIS CREATED — read before any frontend work.** `bodies_online` is **no longer settled at
scene load**. Anything that builds from the field in `_ready` must instead build on the new
`Sim.field_online` signal (`solar_system.gd` planet nodes, `main.gd` camera focus ring both had to
move). Getting it wrong is NOT a blank screen: `_process` throws `Invalid access to key 'MERCURY'` every
frame, **and a GDScript error skips the rest of the function**, so every line below it dies — the belt
froze at the origin (= the Sun) and the comet's span gate silently stopped applying. **Both headless
suites stayed green through all of it** (85 + 53 checks); only `_shot.gd`'s printed `node_visible` flag
and the picture caught it, and the errors went to **stderr**, which the windowed run's filter was not
reading. Confirmed as mine, not a flake, by `git stash`-ing the GDScript and re-running against the same
DLL. `_shot.gd` now counts planet nodes vs `Sim.planets` and prints FAIL.

**(e) A placeholder that made two tests meaningless.** `test_orrery.gd`'s clock checks run right after
`_ready`, and `EPOCH0_TDB`'s default (883569600.0) is *exactly* what the core returns, `T_IMPACT`'s
(4383.0) is inside its own 2-day tolerance — so with a threaded load both would pass whether the loader
landed or not. The pump to completion + `bodies_online` gate is what keeps them real. Same shape: the
build-worker bound now reads `ready_phase_ms["adopt_field"]` (the landing frame), because `load_ms`
contains the disk read and says nothing about whether the build blocked.

**Trap (d) has a THIRD consumer people miss:** `_build_events()` also runs in `_ready` and also reads the
placeholders (`bodies_online`, `T_MIN`, `T_MAX`) — it opened the log with "NO EPHEMERIS KERNEL" over a
kernel still being read. Rebuilt from `_poll_load` now. When auditing this, check *every* function
`_ready` calls, not just the obviously field-dependent one. **And run the CI gates before pushing:**
`cargo fmt --all -- --check` (CI's exact form; `--manifest-path` alone errors "Failed to find targets")
caught a blank line that would have turned `main` red, and `cargo build` does NOT compile `#[cfg(test)]`
code — `cargo clippy --workspace --all-targets --release` is what CI runs and what type-checks tests.

**A flaky bound that is NOT yours:** `test_gdext.gd`'s `begin_build_scenario() < 1000 ms` (a bare
`thread::spawn`) measured 755 / 2015 / 3551 ms in one hour on this machine. Verified environmental by
stashing the test and re-running the *unmodified* one against the same DLL — it failed harder. Left at
1000 ms deliberately (a blocking build would be 10–30 s, so there is 10x headroom). Check the machine
before believing it.

**Boot POST has a third state now:** `DE440S.BSP READING ...` retyped to `LOADED` in place (same four
line slots so nothing shifts; `_chars` is an array-wide budget). Emit `field_online` **after**
`_begin_build()` or the POST prints "DEFLECTION SOLVER ... OFFLINE - NO EPHEMERIS" under a loaded
kernel. Warm the READING state lasts ~2 ms, so it was photographed by temporarily holding the landing
25 s — there is no other way to see it.

## 2026-09-07 (later) - labels placed instead of dropped, and ONE frame-ms number that lied twice

Plan tasks 3/4/5/6 of `docs/plans/2026-09-05-visuals-performance-followups.md`, all closed.

**The placement rule, now used in two files** (`tag_layer.gd`, `encounter.gd`): collect -> place by
priority -> paint. The **glyph never moves** (it is the measured position); only text slides, to the
first clear offset. **A blinked-off label MUST still reserve its rectangle** - "PREDICTED IMPACT"
blinks, and a pass that only sees what is painted re-solves every neighbour twice a second: correct in
any screenshot, jittering in motion, invisible to the shot harness. `belt_1_real_asteroids` is the
case (2028-01-01 is 12 yr before impact, so Earth is back in nearly the same place and the *invisible*
caption is what moves "EARTH"). Two collisions the plan did not name: **the two b-point captions
collide with each other** zoomed out (`enc_4` printed them on the same pixels - unreadable, not
crowded), and keyhole names struck through the deflected diamond, so the **marks** are reserved too.

**THE MEASUREMENT LESSON - read before quoting any frame time here.** A single `frame ms avg` on this
box produced **two confident, opposite, wrong conclusions in one session**. (1) One run at 6.94/7.26 ms
-> "the b-plane is no longer the expensive view, Task 6 has nothing to win". Four runs say ~10.1 vs
~6.9 for the 3D views; the plan's 9.1/9.2 baseline was right. (2) Next runs at 10.2-10.7 -> "Task 4
cost 3.3 ms/frame"; a width memo was written to fix a regression that did not exist (it measured no
better and did not ship). Both errors were one run vs one run, and the 6.94 was the outlier.
**Procedure:** two runs each side and report all four (overlapping sets = no result); check the
**micro gauges** (`lookup(EARTH) raw` sat at 11.8-12.8 us across every run, which is what made a 1.0 ms
delta readable); and **A/B by file swap** - `git show <commit>:<path> > <path>`, run twice, restore -
which is what proved Task 4 innocent, the same technique that pinned the `bodies_online` regression.
**And `_perf.gd` runs in a 64x64 window**, so anything gated on "is this label on the plot" is measured
in a regime nothing like 1600x900 - a caption cost can be invisible in the harness and real in the game.

**Tracks as polylines: 10.1 -> 9.1 ms (-10%), not the -2 ms predicted.** NOT one polyline per track -
that and the off-frame reject do not compose (a single line must include the off-frame points to stay
connected). Runs of consecutive on-frame points sharing a colour, broken when the run leaves the grown
rect or `s` changes sign. Cache keyed (zoom, size), cleared in `_fetch`. Two invariants, both written
at `_track_runs` because neither is visible at the cache: `center`/`ppl` are pure functions of `size`
and `_half_ld`; and the invalidation is right **by draw order, not construction** - `plan_changed`
clears `_built`, not `_runs`, and only `_fetch` running at the top of `_draw` saves it.

**`[I]` cycles persistence `[0.14, 0.35, 0.0]` s - NOT `[G]`, which the plan asked for and which is
already `tier2_term_gr`.** Godot reports no duplicate binding; the second one just wins, so check
project.godot's keycodes before adding an action. `0.0` is a real rung handled as a case (the decay
formula divides by zero there). The belt dims with warp via a `dim` uniform on `starfield.gdshader`,
set only when the warp step changes. **The belt claim rests on the printed `belt_dim_set_for_warp=9`
(dim floors at 0.15), NOT on the two trail pictures** - the clock runs through the key presses, so they
are seconds apart and are a fair before/after for persistence only.


**2026-09-07 - `-Headless` is for `_perf.gd`, NOT for `_shot.gd`.** Running
`run_harness.ps1 -Harness _shot.gd -Headless` hangs forever after the boot line
and gets killed at the timeout with an empty stderr, which looks exactly like a
parse error and is not one: `_shot()` awaits `RenderingServer.frame_post_draw`,
and under the dummy rendering driver that signal never fires. The harness
docstring already says screenshots need a window. Run `_shot.gd`,
`_pork_shot.gd`, `_threat_shot.gd`, `_tier2_shot.gd` and `_tractor_shot.gd`
**windowed** (~15 min for `_shot.gd` end to end, and it needs
`-TimeoutSec 1500`); keep `-Headless` for `_perf.gd`.

Two shutdown messages are normal noise after the last SHOT line and are not a
failure: `Attempted to set an invalid (previously freed?) object instance into a
'TypedArray'` (x2) and `Capture not registered: 'gdaimcp'` (the editor MCP
plugin, which does not connect).


**2026-09-07 - a `--script` run registers NO autoloads, and that is a testing
lever, not just a trap.** `godot --headless --path godot --script res://tests/X.gd`
does not create the `Sim` singleton, so any script naming `Sim` fails to
*compile* in isolation - the error is `Identifier not found: Sim` at load, before
a single line runs. `encounter.gd` names `Sim` constantly, so it cannot be
exercised this way at all. The lever: pure view geometry moved into
`W:\Claude_projects\AsteroidDefense\godot\scripts\plot_geometry.gd`, which names
nothing, and `godot\tests\test_geometry.gd` measures it with **no kernels, no
scenario build and no window, in about a second**. That is the first test here
that costs nothing to run. Loaded by `preload`, not `class_name`, so a game run
needs no editor rescan.

It caught a defect no screenshot could: `draw_arc(cc, r, 0, TAU, 256, ...)`
tessellates in *angle*, so the widest resonant circle (795 431 px across at the
zoom-in stop) was drawn **59.9 px** from where the circle actually is, on a
720 px view - and it still looked like a plausible line. Clipping the arc to the
viewport and tessellating to a 0.3 px screen budget gives 0.113 px with **3
points instead of 256**. See [[keyhole-reach]].

**Watch the pipe when running Godot or cargo from a tool call.** Piping to
`tail`/`grep` buffers everything until the process exits, so a run that has
finished its work but hangs on shutdown looks like a run that printed nothing.
Redirect to a file under `W:\temp\claude` and read that instead. (A Godot run
that does hang: kill only the PID you captured - `Stop-Process -Id <pid>`.)
