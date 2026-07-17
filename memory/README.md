# Project memory

These files are the **Claude Code project-memory** for this repository —
durable, structured notes the AI assistant keeps across sessions so context
isn't lost between them. They're mirrored here for transparency and version
history; the live canonical copies live outside the repo in the maintainer's
local Claude config and are kept byte-identical to what's committed.

Each file is one fact with YAML frontmatter (`name`, `description`,
`metadata.type`) and may cross-link others with `[[name]]`.

- [`MEMORY.md`](MEMORY.md) — the index: one line per memory.
- [`project-overview.md`](project-overview.md) — what the project is, where the
  locked spec lives (`../HANDOFF.md`), the license, and the current phase.
- [`git-workflow.md`](git-workflow.md) — when the assistant updates memory/docs
  and commits/pushes.
- [`godot-mcp-script-cache.md`](godot-mcp-script-cache.md) — Godot editor caches
  `.gd` resources edited externally; the running game reads disk.
- [`godot-phase2-scaffold.md`](godot-phase2-scaffold.md) — Phase-2 Godot project
  scaffold: `godot/` root, gdai-mcp wiring, the precision=single decision.
- [`godot-visual-layer.md`](godot-visual-layer.md) — the retro-CRT visual shell
  (3D orrery, 2D radar, HUD, boot); its b-plane section is superseded by the
  gdext binding, kept for history.
- [`gdext-binding.md`](gdext-binding.md) — the Phase-2 GDExtension binding of the
  Rust core into Godot: the real DE440 orrery, the deflection planner, the
  b-plane encounter view, and the traps found building them.

These are working notes, not authoritative documentation. The authoritative spec
is [`../HANDOFF.md`](../HANDOFF.md); the public summary is
[`../README.md`](../README.md).
