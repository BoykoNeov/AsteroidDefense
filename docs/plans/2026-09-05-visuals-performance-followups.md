# Visuals and performance — follow-up plan (2026-09-05)

Written for a smaller model to execute task by task. Each task says what to
change, in which files, how to verify it, and what "done" looks like. Do them in
order unless a task says it is independent. Read `HANDOFF.md` → *The frontend
measured and given its phosphor — 2026-09-05* first; it explains the state this
plan starts from.

## Ground rules (read before touching anything)

1. **Measure before and after.** Every performance task starts and ends with:
   ```powershell
   powershell -File godot/tests/run_harness.ps1 -Harness _perf.gd -Headless
   ```
   Keep both outputs (they are also written to
   `M:\claud_projects\temp\AsteroidDefense\perf\<stamp>.txt`). The numbers to
   compare are **frame ms avg** (headless = CPU cost) and **ffi/frame** (native
   binding calls; deterministic, so it is the cleanest signal). Windowed runs are
   pinned to the monitor refresh by the compositor and cannot show a speed-up.
2. **Look at the picture for every visual task.**
   ```powershell
   powershell -File godot/tests/run_harness.ps1 -Harness _shot.gd
   ```
   writes PNGs to `M:\claud_projects\temp\AsteroidDefense\shots\`. Open the ones
   your change touches. A view that is hidden until a key is pressed executes its
   `_draw` **zero** times in a passive run, so the shot harness is the only check.
3. **After ANY Rust change:** `cargo build -p asteroid_gdext && cargo build -p
   asteroid_gdext --release`. Godot loads `target/debug/asteroid_gdext.dll`. If
   the build says `Access is denied (os error 5)`, a Godot process is holding the
   DLL: list them with
   `Get-CimInstance Win32_Process | ? { $_.Name -like 'godot*' } | select ProcessId, CommandLine`
   and kill **only** the one whose command line is this project's harness
   (`--path M:/claud_projects/AsteroidDefense/godot`), by PID. Other projects'
   Godot processes may be running; never kill by name.
4. **Run the existing checks before committing:**
   `godot --headless --path godot --script res://tests/test_orrery.gd` (83 PASS,
   0 FAIL) and, if Rust changed, `ASTEROID_REQUIRE_KERNELS=1 cargo test -p
   asteroid_gdext --release` (kernel-gated; a 0.02 s run means it skipped).
5. **Commit and push after each task** with a message that states the measured
   before/after or names the PNG that proves it.

## Baseline this plan starts from (headless, 2026-09-05)

| view | frame ms | ffi/frame |
|---|---|---|
| 3d_system | 6.9 | 29 |
| 3d_earth_closeup | 7.0 | 29 |
| 3d_planner_open | 7.4 | 29 |
| map2d | 7.1 | 29 |
| encounter_keyholes | 9.1 | 29 |
| encounter_zoomed_out | 9.2 | 29 |
| porkchop | 8.4 | 29 |
| 3d_max_warp | 7.9 | 29 |

Raw lookup cost: `body_position_ecl_au` ≈ 11 µs, threat ≈ 7 µs, one
`orbit_points(planet, 180)` ≈ 2.1 ms. Time to `mission_online` (the startup wait
for the threat solution): **25–33 s** on the debug DLL with the machine loaded;
~10 s is the documented quiet-machine figure.

---

## Task 1 — Cut the startup wait by running the worker's independent jobs in parallel

> **DONE 2026-09-06 — but not by this design; three of the claims below are wrong.**
> Measured first (`build_phase_timings`): the mount is 0.6 s warm, not 5.7 s, and it
> is **not independent** — `BuiltScenario::build` consumes the mounted almanac and
> the scenario keeps it, which is what the `[P]` force menu recomposes from, so
> step 2 would break that menu. Step 3 is wrong too: the comet cannot overlap the
> "frame + perigee scan", because that step is cache reads costing milliseconds and
> both expensive propagations have already finished — `build_with` re-flies the
> nominal as its own hit check, and that is 10.7 s of the 11.8 s. Step 5 is wrong
> about the *existing* behaviour: a failed comet flight took the whole threat
> solution down with it, it did not leave `comet_online` false. What shipped
> instead: the comet flies on its own worker started **after** the scenario is
> installed, off the critical path entirely. Time to `mission_online` 34.0 -> 25.2 s.
> See *Frontend speed* in `HANDOFF.md`.

**Why.** The threat solution takes 10 s (quiet machine) to 30 s (loaded) to
land, and until it does the planner, the b-plane view and the threat are all
offline. The build worker runs three *independent* jobs one after another:
the threat propagation (~10 s), the sb441 small-body mount (5.7 s cold, 0.3 s
warm), and the comet's free propagation (~2–4 s). Running them concurrently
saves the mount and comet time on every start.

**Files.** `godot/rust/src/mission_core.rs` (`BuiltScenario::build` and the
worker function that `Mission::begin_build_scenario` spawns — search for
`begin_build_scenario`, `mount_small_bodies`, `seed_orrery_body`), possibly
`godot/rust/src/lib.rs` if the worker closure lives there.

**Steps.**
1. Add timing first. Wrap each phase in `std::time::Instant` and `eprintln!`
   the durations (behind `if std::env::var_os("ASTEROID_BUILD_TIMING").is_some()`)
   so the split is known before anything moves. Run the game once with that
   variable set; record the three numbers.
2. The mount does not depend on the scenario (it builds a second almanac from
   paths). Start it on its own thread (`std::thread::scope`, or a second
   `std::thread::spawn` joined before `install`) at the same time the threat
   propagation starts.
3. The comet needs the scenario's force field (`seed_orrery_body(&eph,
   &scenario, …)`), so it cannot start before `RealFieldScenario::build_with`
   returns — but it does *not* need the nominal encounter frame or the perigee
   scan. Reorder so the comet flies on a second thread while the worker does the
   frame + scan, then join.
4. Keep the `Send`/`Sync` facts from the gdext memory in mind: `Arc<Ephemeris>`
   is `Send + Sync`; `RealFieldScenario` is `Sync` (there is a compile-time
   assert for it in `core/src/scenario.rs`). Borrow it across scoped threads by
   reference; do not clone it.
5. Preserve every existing failure mode: a failed mount **warns and continues**
   (a missing catalog beats a missing threat); a failed comet flight leaves
   `comet_online` false. Nothing may panic across the FFI.

**Verify.** `_perf.gd` prints `mission_online=true after N ms` — before/after on
the same machine state, twice each. `test_orrery.gd` 83/83. The kernel-gated
gdext tests green *and* taking tens of seconds (not 0.02 s).

**Done when.** Time to `mission_online` drops by roughly the mount + comet time
(expect 3–8 s), with identical threat numbers in the shots (`cap=11311 km`,
`|B|=14639 km` in `enc_2_band_miss`).

## Task 2 — One native call for all body positions

> **DONE 2026-09-06.** Native calls per frame in the 3D views 29 -> 6, map2d 5 -> 2,
> porkchop 0 -> 0. Two corrections. The baseline table above is **stale**: with the
> fill off, today's run reads map2d 5, encounter 2, porkchop 0 — not 29 everywhere.
> And step 3 as written ("after `_pos_memo.clear()`… one batch call") is what
> shipped first and it was wrong: filling on a schedule makes every frame of every
> view pay a crossing whether or not anything asks, which showed up as porkchop's
> count going 0 -> 1 and as the build wait stretching 34 -> 56 s. The fill is now
> triggered by the first `pos_ecl` miss for an ephemeris body at the live clock.
> See *Frontend speed* in `HANDOFF.md`.

**Why.** The 3D view makes 29 native calls a frame at 7–11 µs each plus
GDScript marshalling; 25 of them are "position of NAIF body X at the clock".
One call returning a `PackedVector3Array` for a `PackedInt64Array` of ids
removes ~0.25 ms/frame and most of the FFI chatter. Small win; do it after Task 1.

**Files.** `godot/rust/src/lib.rs` (new `#[func] body_positions_ecl_au(ids:
PackedInt64Array, tdb_seconds: f64) -> PackedVector3Array`), `godot/scripts/
sim.gd` (`_prime_ephem_positions()` called at the top of `_process` after the
memo clear, filling `_pos_memo` for planets + asteroids at `t`), `godot/rust/
src/mission_core.rs` if a shared helper is wanted.

**Steps.**
1. Rust: loop over ids, call the existing `body_position_ecl_au`, push
   `Vector3::ZERO` for a failed lookup (the existing contract — callers gate on
   `bodies_online`/spans). Panic-free.
2. Add a kernel-gated test asserting the batch equals the individual calls
   exactly for the eight planets at two epochs.
3. GDScript: after `_pos_memo.clear()`, if `bodies_online`, one batch call for
   `planets + asteroids` at `t`; store each in `_pos_memo[name] = [t, p]`. The
   rest of `pos_ecl` is unchanged and still serves any other epoch.
4. Rebuild **both** DLL profiles.

**Verify.** `ffi/frame` in the 3D views drops from 29 to ~5. Pictures identical.
`test_orrery.gd` green. **Done when** the count drops and the batch test passes.

## Task 3 — Tag de-collision in the 3D view

**Why.** In `belt_1_real_asteroids.png` and `trails_1_max_warp.png` labels sit on
each other ("PREDICTED IMPACT E-4383" over "Juno"/"EARTH", "2031-XK <THREAT>"
over "Apophis"). Independent of Tasks 1–2.

**Files.** `godot/scripts/tag_layer.gd`.

**Steps.**
1. Instead of drawing each tag immediately, collect `{pos, text, col, glyph,
   priority}` into an array in `_draw` (priority: threat/impact 3, planets and
   NEOs 2, belt and moon 1).
2. Sort by priority descending; keep a list of placed label `Rect2`s (use
   `_font.get_string_size`). For each label, try offsets in order
   `[(12,4), (12,-10), (-w-12,4), (12,18)]`; take the first whose rect does not
   intersect a placed rect; if none fits and priority is 1, draw the glyph but
   skip the text.
3. Keep the glyphs at their true projected positions — only the text moves.

**Verify.** Re-run `_shot.gd`; open `belt_1_real_asteroids.png`,
`neo_1_on_arc.png`, `trails_1_max_warp.png`: no text over text; every glyph
still on its body. **Done when** those three read cleanly and `ffi/frame` is
unchanged (the layer must not add lookups).

## Task 4 — Keyhole captions must not land on the b-point and impact captions

**Why.** In `enc_2_band_miss.png` the 3:4 caption sits on Earth's limb next to
"PREDICTED IMPACT", and the budgeted captions can still collide with "B 0.04 LD".

**Files.** `godot/scripts/encounter.gd` (`_draw_keyholes`, `_draw_b_points`).

**Steps.**
1. Draw order is rings → keyholes → Earth → tracks → b-points. Make
   `_draw_b_points` compute its two caption rects first (extract a
   `_caption_rects()` helper that returns them without drawing) and pass them to
   `_draw_keyholes` as reserved rects.
2. In the caption pass, a keyhole caption whose rect intersects a reserved rect
   moves to the **left** of the ζ axis (`at.x - 8 - width`) and, if still
   colliding, drops to the next budget slot.

**Verify.** `enc_2_band_miss.png`, `enc_4_zoomed_out.png`, `enc_6_live_marker_burned.png`.
**Done when** no keyhole caption touches the "B …" or "PREDICTED IMPACT" text at
the default span or the 1.2 LD span.

## Task 5 — A persistence control, and the belt at max warp

**Why.** `trails_1_max_warp.png` shows the scenery belt smearing into a solid
band at 10 yr/s (1600 points, rigid rotation). It is a legitimate phosphor
effect but it is loud, and there is no way to turn persistence off.

**Files.** `godot/scripts/main.gd` (`PERSIST_TAU` → a small ladder `[0.0, 0.14,
0.35]` cycled by a new input action), `godot/project.godot` (new action, e.g.
`persist_cycle` on `G`, keycode 71 — add it by editing the file while **no
editor is open**), `godot/scripts/hud.gd` (`_help_line` lists the key),
`godot/scripts/solar_system.gd` (belt).

**Steps.**
1. `keep = exp(-delta / tau)` with `tau = 0` meaning `keep = 0` (no trail).
2. For the belt: scale the belt's point brightness down as warp rises —
   `_belt.set_instance_shader_parameter` is not available on the star shader;
   instead give `starfield.gdshader` a `uniform float dim = 1.0` and set it on
   the belt's material from `_process` as `clampf(3650.0 / (Sim.WARP_STEPS[Sim.warp_idx] * 40.0), 0.15, 1.0)`.
   Keep the starfield's material separate (it does not move).
3. Add a `trails_2_persist_off` shot to `_shot.gd` with tau 0 at max warp.

**Verify.** Two trail shots: on (ghost chains behind planets, belt readable) and
off (no ghosts). HUD lists the key. **Done when** both pictures match that.

## Task 6 — Encounter tracks as polylines

**Why.** The b-plane view is the most expensive remaining view (9.1–9.2 ms
headless). `_draw_track` issues up to 1 400 `draw_line` calls per track per
frame with a rect test each.

**Files.** `godot/scripts/encounter.gd` (`_draw_track`).

**Steps.**
1. Split each track at the sign change of `s` (`pts[k].z`) into an inbound and
   an outbound `PackedVector2Array` built with one loop, then two
   `draw_polyline` calls (inbound full colour, outbound `col.a * 0.4`).
2. Keep the off-frame reject: skip a whole polyline only if *every* point is
   outside the grown rect (compute a bounding box while building).
3. Cache the two arrays per zoom level: rebuild only when `_half_ld` or `size`
   changed or `_built` was reset.

**Verify.** `_perf.gd` encounter rows drop (expect ~9 → ~7 ms); `enc_*` shots
identical to the eye. **Done when** both hold.

## Task 7 — The Tier-3 ellipse on the b-plane view (roadmap item, larger)

**Why.** `HANDOFF.md` → *What is next* item 4. The keyhole map already shows the
ellipse in `docs/keyhole_map.svg`; the Godot view does not. The sensitivity
solve is ~17 s, so it must be an on-demand worker like the porkchop grid.

**Files.** `godot/rust/src/mission_core.rs` and `lib.rs` (a third mpsc channel
mirroring `tier2_build`: `begin_uncertainty()`, `poll_uncertainty()`,
`is_measuring_uncertainty()`, and a reader `bplane_ellipse_km() ->
Dictionary{center_xi, center_zeta, semi_major, semi_minor, angle_from_xi_rad,
p_impact}`), `godot/scripts/sim.gd` (flags + poll + request on a key),
`godot/scripts/encounter.gd` (draw the 1σ/3σ ellipses under the b-points, label
`P(IMPACT) …`, and say "COVARIANCE INVENTED" — the shipping rock has no
observation arc), `godot/project.godot` (a key, e.g. `U`).

**Steps.**
1. Read `core/src/uncertainty.rs` (public API: the Jacobian, `impact_probability`,
   the sampling plan) and `core/examples/probe_tier3_uncertainty.rs` for the
   exact calls and the ξ,ζ rotation used by `probe_keyhole_map`.
2. Copy the `tier2_build` channel pattern exactly (begin/poll/is_measuring,
   `try_recv`, Disconnected → error).
3. Anything chained before `install` delays `mission_online`; this must be
   on-demand only.
4. Add a shot `enc_8_ellipse` to `_shot.gd` that presses the key, waits for the
   worker (≤ 60 s), and captures.

**Verify.** The ellipse's major axis is ~90° from ξ (along ζ) and the printed
`P(IMPACT)` matches `probe_tier3_uncertainty`'s figure for the nominal.
**Done when** the shot shows it and the kernel-gated binding test pins the
numbers.

## Task 8 — Small things, any order

- `hud.gd::_clip` measures strings in a loop every frame; cache the clipped
  string per (line, budget). Tiny.
- `solar_system.gd::_process` sets two instance shader parameters every frame
  (`energy` on the nominal body and the deflected line); set them only when
  `burned` changes.
- `_perf.gd`: add a windowed sanity row that asserts no view is *slower* than
  the compositor cap (frame avg > 1.2 × min across views ⇒ print `SLOW`).
- The `Performance.TIME_PROCESS` puzzle (hundreds of ms beside an 8 ms frame):
  check whether it is the monitor's units on this build; if it is real, find
  what runs in the process step that the wall clock does not see.

## Traps recorded this session (do not rediscover)

- Godot on this machine can linger after `quit()` at 100 % CPU (editor plugin
  teardown). It poisons the next measurement and holds the debug DLL. The
  runner kills only its own PID; if you launch Godot by hand, capture the PID.
- `polls > 1`-style assertions on a worker race a loaded machine. Assert what
  the caller paid instead.
- A `.ps1` with a curly dash or any non-ASCII byte fails to parse under Windows
  PowerShell 5.1 (the byte decodes to a quote character). Plain ASCII only.
- The first persistence shader was an exponential *average*; fast movers vanish
  under it. Peak-hold (`max(world, previous*keep)`) is the one that reads right.
- A memo keyed by body name must check the exact epoch; the orbit walks bypass it
  (`Sim._lookup_ecl`) on purpose.
