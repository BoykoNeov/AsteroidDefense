extends SceneTree
## Load gate for the Rust GDExtension binding (Commit 1):
##   godot --headless --path godot --script res://tests/test_gdext.gd
## Instantiates the native `AsteroidCore` class and reads `core_version()`
## back. Runs in GAME context (not the editor tool context, where a
## non-tool GDExtension class instantiates as a placeholder and the call
## returns null). A non-empty, well-formed version string proves class
## registration + the Rust<->Godot FFI + that the gdext build loaded in 4.7.

func _init() -> void:
	var fails := 0

	if not ClassDB.class_exists("AsteroidCore"):
		print("FAIL  AsteroidCore class not registered (extension not loaded)")
		quit(1)
		return

	var core = AsteroidCore.new()
	var v = core.core_version()

	if typeof(v) != TYPE_STRING:
		print("FAIL  core_version() returned type %d, expected String (placeholder?)" % typeof(v))
		fails += 1
	else:
		print("core_version() = '%s'" % v)
		if (v as String).is_empty():
			print("FAIL  core_version() is empty")
			fails += 1
		elif v == "0.1.0":
			print("PASS  version string round-tripped from Rust core (0.1.0)")
		else:
			# Non-empty but unexpected value: still proves the FFI works,
			# but flags a version drift worth noticing.
			print("PASS  FFI round-trip OK; note version '%s' != expected 0.1.0" % v)

	print("gdext load gate: %s" % ("PASS" if fails == 0 else "FAIL (%d)" % fails))
	quit(fails)
