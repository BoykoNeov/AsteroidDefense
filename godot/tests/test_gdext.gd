extends SceneTree
## Load gate + FFI checks for the Rust GDExtension binding:
##   godot --headless --path godot --script res://tests/test_gdext.gd
## (set ASTEROID_DE_KERNEL / ASTEROID_PLANETARY_CONSTANTS for the Mission part).
##
## Runs in GAME context (not the editor tool context, where a non-tool
## GDExtension class instantiates as a placeholder and calls return null).
##
## Commit 1 — AsteroidCore.core_version() round-trips a string from Rust.
## Commit 2 — Mission.load() + body_position_ecl_au(): real DE440 positions
## cross the FFI as ecliptic-AU Vector3s (fast path; no scenario build here —
## the expensive required_dv-vs-curve.json check runs release-side in
## `cargo test -p asteroid_gdext --release`). Skips the Mission part green when
## kernels are absent, like the kernel-gated Rust/validation tests.

var fails := 0

func _check(ok: bool, msg: String) -> void:
	if ok:
		print("PASS  " + msg)
	else:
		print("FAIL  " + msg)
		fails += 1

func _init() -> void:
	# --- Commit 1: the load gate ------------------------------------------
	if not ClassDB.class_exists("AsteroidCore"):
		print("FAIL  AsteroidCore not registered (extension not loaded)")
		quit(1)
		return
	var core = AsteroidCore.new()
	var v = core.core_version()
	_check(typeof(v) == TYPE_STRING and not (v as String).is_empty(),
		"core_version() round-trips a string ('%s')" % v)

	# --- Commit 2: the Mission scenario surface ---------------------------
	var m = Mission.new()
	print("build_profile = %s" % m.build_profile())
	if not m.load():
		print("SKIP  Mission.load() failed (no kernels?): %s" % m.last_error())
		print("gdext gate: %s" % ("PASS" if fails == 0 else "FAIL (%d)" % fails))
		quit(fails)
		return

	_check(m.is_loaded(), "Mission.load() succeeded, ephemeris ready")

	# Body positions come across as ecliptic-AU Vector3s. At J2000 (t=0), Earth
	# is ~0.983 AU from the Sun and (crucially) essentially in the ecliptic plane
	# — |z| would be up to ~0.4 AU if the ICRF->ecliptic obliquity rotation were
	# missing, so this pins the frame handling end-to-end through the FFI.
	var t := 0.0
	var earth: Vector3 = m.body_position_ecl_au(399, t)
	print("Earth ecl-AU = (%.4f, %.4f, %.4f), |r| = %.4f" %
		[earth.x, earth.y, earth.z, earth.length()])
	_check(earth.length() > 0.98 and earth.length() < 1.02,
		"Earth ~1 AU from Sun (|r| = %.4f)" % earth.length())
	_check(abs(earth.z) < 0.02,
		"Earth lies in the ecliptic plane (|z| = %.4f AU)" % abs(earth.z))

	var jup: Vector3 = m.body_position_ecl_au(5, t)
	_check(jup.length() > 4.9 and jup.length() < 5.5,
		"Jupiter ~5.2 AU from Sun (|r| = %.4f)" % jup.length())

	# An unresolved body / bad id returns ZERO rather than panicking across FFI.
	var bad: Vector3 = m.body_position_ecl_au(999999, t)
	_check(bad == Vector3.ZERO, "unknown NAIF id returns ZERO (no panic)")

	print("gdext gate: %s" % ("PASS" if fails == 0 else "FAIL (%d)" % fails))
	quit(fails)
