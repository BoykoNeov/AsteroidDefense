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

**Encounter/b-plane close-up view (key 3, DONE 2026-07-14, screenshot-verified):**
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
