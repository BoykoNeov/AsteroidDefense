extends SceneTree
## Load gate + FFI checks for the Rust GDExtension binding:
##   godot --headless --path godot --script res://tests/test_gdext.gd
##
## Runs in GAME context (not the editor tool context, where a non-tool
## GDExtension class instantiates as a placeholder and calls return null).
##
## Commit 1 — AsteroidCore.core_version() round-trips a string from Rust.
## Commit 2 — Mission.load_from() + body_position_ecl_au(): real DE440 positions
## cross the FFI as ecliptic-AU Vector3s (fast path; no scenario build here —
## the expensive required_dv-vs-curve.json check runs release-side in
## `cargo test -p asteroid_gdext --release`).
## 3C-2a — kernels are found through Kernels.resolve(), the same resolver the
## game uses, rather than through the env vars only a developer shell has. That
## is deliberate: this gate previously passed (by skipping) on a machine where
## the launched game could not find a kernel at all, which is precisely the break
## it should have caught. Skips green only when no kernel exists anywhere.

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

	var k := Kernels.resolve()
	if not k.ok:
		print("SKIP  no kernels on this machine:\n%s" % k.error)
		print("gdext gate: %s" % ("PASS" if fails == 0 else "FAIL (%d)" % fails))
		quit(fails)
		return
	print("kernels via %s" % k.source)
	_check(m.load_from(k.bsp, k.pca),
		"Mission.load_from() succeeded (%s)" % m.last_error())
	_check(m.is_loaded(), "ephemeris ready")

	# The clock clamps to this; an epoch outside it fails every body lookup, and
	# a failed lookup returns ZERO — which is the SUN's position in this
	# heliocentric frame. So an unclamped clock does not blank the display, it
	# silently piles every planet onto the Sun. Discovered from the mounted
	# kernel (de440s ~1850-2149, de441 ~1550-2650), never hardcoded.
	var span: PackedFloat64Array = m.usable_span_tdb()
	_check(span.size() == 2 and span[0] < 0.0 and span[1] > 0.0,
		"usable_span_tdb() brackets J2000")
	if span.size() == 2:
		var yr := 365.25 * 86400.0
		print("usable span = J2000%+.1f yr .. J2000%+.1f yr (%.0f yr wide)" %
			[span[0] / yr, span[1] / yr, (span[1] - span[0]) / yr])
		_check(m.body_position_ecl_au(399, span[0]) != Vector3.ZERO
			and m.body_position_ecl_au(399, span[1]) != Vector3.ZERO,
			"Earth resolves at BOTH span edges (not just mid-span)")

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

	# Orrery catalog #[func]s are registered and FFI-safe on the fast path (no
	# scenario built here — building is the release-side cost). With no scenario,
	# the catalog is empty and add/read fail gracefully instead of panicking.
	_check(m.catalog_count() == 0, "catalog empty before any bodies added")
	var idx: int = m.add_synthetic_body("PROBE", "comet",
		2.0, 0.2, 5.0, 0.0, 0.0, 0.0, 0.0, 365.0, 5.0)
	_check(idx == -1 and not (m.last_error() as String).is_empty(),
		"add_synthetic_body before build_scenario fails with an error, no panic")
	_check(m.catalog_position_ecl_au(0, t) == Vector3.ZERO,
		"catalog_position on empty catalog returns ZERO (no panic)")
	_check(m.catalog_span_tdb(0).is_empty(),
		"catalog_span on empty catalog returns an empty array")

	# --- 3C-2b: the scenario builds on a worker, not on the main thread --------
	# The build is ~10 s of integration. It is threaded because those 10 s would
	# otherwise be 10 s of frozen display — and the display being frozen is a
	# *working* one, drawing real planets since 3C-2a. So the property under test
	# is not "it builds" (the release-side Rust tests cover that) but "it builds
	# without blocking the caller", which only Godot can answer.
	_check(not m.is_ready(), "no scenario before the build starts")
	_check(m.capture_radius_m() < 0.0, "capture radius unavailable before a build")
	# The Tier-2 preview rides the scenario build too, so before it there are no
	# shifts and every term reads the -1 "unavailable" sentinel (never 0).
	_check(not m.has_tier2_preview(), "no Tier-2 preview before a build")
	_check(m.tier2_shifted_perigee_m("relativity") < 0.0,
		"Tier-2 shift unavailable before a build (-1 sentinel, not 0)")

	var t0_begin := Time.get_ticks_msec()
	_check(m.begin_build_scenario(),
		"begin_build_scenario() started a build (%s)" % m.last_error())
	var begin_ms := Time.get_ticks_msec() - t0_begin
	# The decisive assertion: begin_ returned in milliseconds while ~10 s of work
	# is still to come. If this ever creeps up, the build has moved back onto the
	# calling thread and the boot freeze is back.
	_check(begin_ms < 1000,
		"begin_build_scenario() returned in %d ms without waiting for the build" % begin_ms)
	_check(m.is_building(), "a build is in flight")
	_check(not m.begin_build_scenario(),
		"a second concurrent build is refused rather than racing the first")

	# Poll to completion the way the frontend will, counting the polls that saw
	# work still in progress. Zero such polls would mean something blocked.
	var polls := 0
	var t0_build := Time.get_ticks_msec()
	while m.poll_build():
		polls += 1
		OS.delay_msec(20)
	var build_ms := Time.get_ticks_msec() - t0_build
	_check(polls > 0,
		"poll_build() saw the build running rather than blocking (%d polls, %d ms)"
		% [polls, build_ms])
	_check(not m.is_building(), "the build is no longer in flight once it lands")
	_check(m.is_ready(), "scenario installed after the build (%s)" % m.last_error())

	# The capture radius: the bar a deflection verdict is measured against. ~1.77 R⊕
	# here (v_inf ≈ 7.6 km/s), so ~11 300 km — see the Rust-side test for the
	# derivation. Bounds are wide enough to be a smoke test, not a duplicate of it.
	var cap: float = m.capture_radius_m()
	_check(cap > 6.4e6 and cap < 1.4e7,
		"capture radius is a focused disc (%.0f km, %.2f R-earth)" % [cap / 1000.0, cap / 6.3781e6])
	var nom_p: float = m.nominal_perigee_m()
	_check(nom_p >= 0.0 and nom_p < cap,
		"nominal perigee %.0f km falls inside the capture disc (it is a hit)" % (nom_p / 1000.0))

	# --- Tier-2 preview: the on-demand force-model shifts across the FFI --------
	# The preview is DELIBERATELY not part of the build — it is ~64 s that would
	# delay the threat solution, so it is measured on demand when the menu opens.
	# Right after the build there are no shifts yet.
	_check(not m.has_tier2_preview(),
		"no Tier-2 preview right after the build (it is off the critical path)")
	_check(m.tier2_shifted_perigee_m("relativity") < 0.0,
		"Tier-2 shift unavailable until the menu measures it (-1, not 0)")

	# Drive the measurement exactly as the frontend does: begin + poll to landing.
	# This is the ~64 s (four ~16 s propagations) the menu costs.
	_check(m.begin_tier2_preview(),
		"begin_tier2_preview() started the measurement (%s)" % m.last_error())
	_check(m.is_measuring_tier2(), "a Tier-2 measurement is in flight")
	_check(not m.begin_tier2_preview(),
		"a second concurrent Tier-2 measurement is refused rather than racing")
	var t2_polls := 0
	while m.poll_tier2_preview():
		t2_polls += 1
		OS.delay_msec(20)
	_check(t2_polls > 0,
		"poll_tier2_preview() saw the measurement running rather than blocking (%d polls)" % t2_polls)
	_check(not m.is_measuring_tier2(), "measurement no longer in flight once it lands")
	_check(m.has_tier2_preview(), "Tier-2 preview measured on demand")

	# Each always-available term must be a real perigee that actually moved off the
	# baseline; the belt must be UNAVAILABLE (this load armed no small-body kernel),
	# reported as -1, not a 0 km "does nothing".
	for term in ["relativity", "yarkovsky", "srp"]:
		var shifted: float = m.tier2_shifted_perigee_m(term)
		var shift_km: float = (nom_p - shifted) / 1000.0
		_check(shifted >= 0.0 and absf(nom_p - shifted) > 1.0,
			"Tier-2 '%s' shifted the perigee %+.2f km off baseline" % [term, shift_km])
	_check(m.tier2_shifted_perigee_m("belt") < 0.0,
		"Tier-2 belt is UNAVAILABLE without the small-body kernel (-1, not 0)")
	_check(m.tier2_shifted_perigee_m("no-such-term") < 0.0,
		"an unknown Tier-2 term is unavailable, not a silent 0")

	# The threat is on top of Earth at the impact epoch — one assertion that
	# exercises the whole threat frame chain across the FFI.
	var t_imp: float = m.impact_tdb_seconds()
	var ast: Vector3 = m.asteroid_position_ecl_au(t_imp)
	var earth_imp: Vector3 = m.body_position_ecl_au(399, t_imp)
	# Reported in km, not AU: GDScript's format has no %e/%g, so a ~1e-5 AU gap
	# formats as "0.00" — a passing check whose message says nothing.
	var gap_km: float = (ast - earth_imp).length() * 1.495978707e8
	_check(ast != Vector3.ZERO and (ast - earth_imp).length() < 1.0e-3,
		"threat coincides with Earth at impact (gap %.0f km, inside the %.0f km capture disc)"
		% [gap_km, cap / 1000.0])

	# The planner nudge: this is what the core's nominal cache bought. It re-flew
	# the whole 12-year cruise per call before (~11 s); now it re-propagates only
	# the post-deflection arc. The threshold is far above the ~1 s observed and far
	# below the regression, so it fails loudly if the cache is ever lost.
	var t0_plan := Time.get_ticks_msec()
	_check(m.set_plan(m.period_seconds(), 0.1), "set_plan() succeeded (%s)" % m.last_error())
	var plan_ms := Time.get_ticks_msec() - t0_plan
	_check(plan_ms < 5000,
		"set_plan() took %d ms (~11 000 before the nominal cache landed)" % plan_ms)
	_check(m.has_plan(), "a plan is set")
	# Exactly one of the two carries the outcome: a finite perigee XOR a clean miss.
	_check(m.is_clean_miss() != (m.deflected_perigee_m() >= 0.0),
		"clean-miss and finite-perigee are mutually exclusive with a plan set")

	print("gdext gate: %s" % ("PASS" if fails == 0 else "FAIL (%d)" % fails))
	quit(fails)
