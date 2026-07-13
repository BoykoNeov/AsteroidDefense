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

**MCP topology (the "add gpai mcp next" step, NOT yet done):** two pieces — the
in-editor gdextension/EditorPlugin (hosts HTTP on 3571) AND `gdai_mcp_server.py`
(PEP 723 inline script, deps `mcp==1.13.0`+`httpx==0.28.1`, run via `uv run`;
NO license/activation gate) which proxies MCP↔the 3571 HTTP. To use: editor
open with plugin enabled + register that server in Claude's MCP config. Requires
the editor to be *running and kept open*.

**Precision decision (deferred, must be conscious before gdext binding):** the
installed editor is single-precision, matching §7's single+floating-origin plan
— correct default. But double-precision Godot is a SEPARATE editor download, and
a single editor CANNOT load a double gdextension (or vice versa). When the gdext
`core` binding lands its precision MUST match the committed editor. The `.double`
dll in the addon is the plugin author shipping both, not evidence of a double
build. See [[project-overview]].

**Not done yet / next:** wire the MCP server into Claude config; then the gdext
binding of `core/` (do NOT make `godot/` a Cargo workspace member — a Cargo.toml
/target/ there would re-poison Godot's scan; use a sibling crate or
`.gdignore`'d subdir). See [[git-workflow]].
