---
name: godot-phase2-scaffold
description: Phase-2 Godot project scaffolded under godot/; gdai-mcp plugin verified loading in Godot 4.7
metadata:
  type: project
---

Phase 2 (Godot 3D view, HANDOFF §4/§8) kicked off 2026-07-13 by pulling the
Godot viewer forward ahead of the rest of Phase 2. State:

- **Godot 4.7.stable** installed (single-precision) at
  `C:\Users\boiko\AppData\Local\Microsoft\WinGet\Links\godot.exe`, on PATH.
- **Project root = `godot/`** (NOT the repo root). Deliberate, advisor-gated:
  repo-root-as-project-root would make Godot's filesystem scanner import
  `target/` (gigabytes), every `.rs`, and `pyref/`. `godot/` keeps `res://`
  clean. `godot/project.godot` (config_version=5, Forward+) + minimal
  `godot/main.tscn` (Node3D World + Camera3D + DirectionalLight). Both tracked.
- **`addons/` was MOVED from repo root → `godot/addons/`** (plain move; it was
  untracked/gitignored). `.gitignore` rule changed `/addons/` → `/godot/addons/`
  and added `/godot/.godot/` (editor import cache). So the plugin + Godot cache
  stay out of git — a fresh clone must re-download the plugin, and Godot will
  warn about the missing enabled plugin until it does (accepted: it's licensed
  dev tooling, matches the pre-existing ignore intent).
- **gdai-mcp plugin ("GDAI MCP" v0.3.2, gdaimcp.com) load GATE = PASS in 4.7.**
  It's a prebuilt gdextension (`compatibility_minimum=4.2`), so 4.7 was five
  minors ahead and an ABI failure was the real risk. Verified: headless editor
  import registered it (`.godot/extension_list.cfg`), no ABI/"can't open
  dynamic library" errors, editor reached "Editor layout ready", Godot
  auto-added its autoload `GDAIMCPRuntime` to project.godot, and its HTTP
  bridge **bound `127.0.0.1:3571` LISTENING** (404 on `/` — serves only MCP
  endpoints). A segfault appears on forced `--quit-after` shutdown only — a
  teardown artifact of the plugin's background HTTP thread, NOT a load failure;
  irrelevant to interactive editor use.

**MCP topology + wiring (the "add gpai mcp" step — REGISTERED 2026-07-13):**
two pieces — the in-editor gdextension/EditorPlugin (hosts HTTP on 3571) AND
`gdai_mcp_server.py` (PEP 723 inline script at
`godot/addons/gdai-mcp-plugin-godot/gdai_mcp_server.py`, deps
`mcp==1.13.0`+`httpx==0.28.1`, stdio server, run via `uv run`; NO
license/activation gate) which proxies stdio-MCP↔the 3571 HTTP. Port overridable
via env `GDAI_MCP_SERVER_PORT` (default 3571).
- **Registered** in Claude Code **local scope** (`.claude.json`, project
  M:\claud_projects\AsteroidDefense) as server `gdai-godot`:
  `claude mcp add gdai-godot --scope local -- uv run <abs path to script>`.
  Local (not project/.mcp.json) because addon is gitignored + path is
  machine-specific. `uv` present (0.11.21). Remove:
  `claude mcp remove gdai-godot -s local`.
- **To go LIVE (both required):** (1) open the editor —
  `godot -e --path M:\claud_projects\AsteroidDefense\godot` (plugin already
  enabled in project.godot → hosts :3571); the interactive editor must stay
  open. (2) `/mcp` reconnect (server added mid-session → tools not loaded until
  reconnect/restart). With editor CLOSED, `claude mcp get gdai-godot` shows
  "Connected · tools fetch failed / Request timed out" (proxy up, :3571 down) —
  EXPECTED, not a failure. Verified this exact state on registration.

**Precision decision — RESOLVED 2026-07-13: SINGLE precision (advisor-gated).**
Stay on the installed single-precision editor; §7's single+floating-origin plan
is the design of record. Double was briefly considered but rejected — it is NOT
a download (that was my error): the ONLY official double route is compiling
Godot from source (`scons ... precision=double`, ~30-60 min, ~10 GB scratch,
re-build on every version bump). Not worth it, because Godot here is only a
VIEWER of precomputed trajectories from the double-precision Rust core; it needs
no-jitter, not physics-grade precision. The discriminator is **"who does the
math," NOT heliocentric distance:** double only earns its keep when the ENGINE
does precision-sensitive computation on large coordinates. Godot never does —
physics is Rust/f64, no gameplay/collision, pure viewer that just places nodes
at coordinates we hand it. Even double-precision Godot does camera-relative
rendering internally (GPU never sees f64 in either build); double merely
automates the subtraction we already do in Rust. So the only question is what
coordinates we hand Godot, and Rust (f64, owns the frames) hands it residuals.
- **Dynamic floating origin is MANDATORY, not optional** (corrects an earlier
  wrong note that called the cruise "a coarse overview"): early deflection is
  precision-critical AND far from Earth (~1 AU, float32 ULP ≈ 18 km at absolute
  heliocentric coords). But jitter never manifests because Godot never holds a
  1-AU absolute coordinate — Rust rebases to the current view FOCUS and hands
  small residuals. Origin follows the camera target (asteroid during the
  deflection burn, Earth during the encounter); zoom into a far region → rebase
  there. Residuals are small AT THE FOCUS; far geometry (far side of a drawn
  orbit) is 18-km-quantized but sub-pixel at that zoom → invisible.
- **Implementation contract for the gdext binding (entirely Rust-side):**
  compute `(body_f64 − focus_origin_f64)` in f64 FIRST, THEN cast the residual
  to f32. Casting heliocentric f64→f32 before subtracting snaps both bodies to
  the 18 km grid → garbage; would wrongly "prove" single fails.
- **Re-usable test if precision resurfaces:** "who does the math?" Stays single
  until Godot itself does physics on absolute coordinates — never, in a viewer.
Consequence: gdext binding stays DEFAULT single (no `double-precision` feature),
editor load GATE already PASS, no from-source build. The addon's `.double` dll
is just the plugin author shipping both flavors, not evidence of a double build.
See [[project-overview]].

**Not done yet / next:** verify MCP live once (open editor + `/mcp` reconnect,
confirm tools list); then the gdext binding of `core/` at DEFAULT single
precision (do NOT make `godot/` a Cargo workspace member — a Cargo.toml /target/
there would re-poison Godot's scan; use a sibling crate or `.gdignore`'d subdir).
Binding contract: Rust rebases to view-focus origin in f64, hands Godot small
f32 residuals (see precision section). See [[git-workflow]].
