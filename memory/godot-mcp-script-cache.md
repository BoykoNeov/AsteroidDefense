---
name: godot-mcp-script-cache
description: Godot editor caches .gd resources edited externally; game runs read disk — verify against the right layer
metadata: 
  node_type: memory
  type: project
  originSessionId: e3b551c7-d904-4130-a1f6-917af8eb928c
---

When editing `godot/scripts/*.gd` with file tools while the Godot editor is open (gdai-mcp workflow): the **running game reads scripts from disk** (edits take effect on next play), but the **editor's in-memory GDScript resources go stale** — `load()` returns cached old source, and `GDScript.reload()` re-parses the cached `source_code`, it does NOT re-read the file.

**How to apply:** To sync the editor cache, use `execute_editor_script` with `FileAccess.get_file_as_string(path)` → `s.source_code = src` → `s.reload(true)`. Also: `main.gd:_apply_focus()` overwrites `OrbitCameraRig.distance` at startup (focus targets carry a distance), so testing camera defaults by editing `orbit_camera.gd` is a trap — change the focus-target entry instead. Keyboard-only test path: zoom is mouse-wheel-only, so temporary focus-target edits are the way to screenshot arbitrary framings via `simulate_input`.
