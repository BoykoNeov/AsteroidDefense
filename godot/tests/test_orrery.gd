extends SceneTree
## Headless verification of the 3C-2a orrery: the real DE440 field behind Sim's
## unchanged public API.
##   godot --headless --path godot --script res://tests/test_orrery.gd
##
## This drives Sim exactly as the game does — _ready() resolves the kernels and
## loads the field — and then asks the questions the display asks. It is the
## closest thing to "run the game and look" that a machine can answer, and it
## covers the failure this whole slice exists to prevent: an unresolved body
## comes back as Vector3.ZERO, and ZERO is *the Sun's position* in this
## heliocentric frame, so a broken lookup does not draw as missing — it draws as
## a planet sitting on the Sun. Every assertion below that looks pedantic about
## non-ZERO is guarding that one silent failure.

var fails := 0


func _check(ok: bool, msg: String) -> void:
	if ok:
		print("PASS  " + msg)
	else:
		print("FAIL  " + msg)
		fails += 1


func _init() -> void:
	var sim = load("res://scripts/sim.gd").new()
	sim._ready()

	if not sim.bodies_online:
		print("SKIP  no ephemeris on this machine:\n%s" % sim.kernel_error)
		quit(0)
		return
	print("kernels via %s" % sim.kernel_source)

	# --- The clock is real, and anchored on the core's own campaign ---------
	# Not a fabricated 2031 epoch: these come from ImpactorConfig::default()
	# (impact 2040-01-01 TDB, lead 12 yr), read without the expensive build.
	_check(sim.date_string() == "2028-01-01",
		"t=0 is the real campaign start 2028-01-01 (got %s)" % sim.date_string())
	sim.t = sim.T_IMPACT
	_check(sim.date_string() == "2040-01-01",
		"T_IMPACT is the real impact epoch 2040-01-01 (got %s)" % sim.date_string())
	sim.t = 0.0
	_check(absf(sim.T_IMPACT - 4383.0) < 2.0,
		"campaign is ~12 yr long (T_IMPACT = %.1f d)" % sim.T_IMPACT)

	# --- The scrub range is the mounted kernel's real coverage --------------
	var span_yr: float = (sim.T_MAX - sim.T_MIN) / 365.25
	print("scrub span: %d .. %d (%.0f yr)" %
		[sim.year_at(sim.T_MIN), sim.year_at(sim.T_MAX), span_yr])
	_check(span_yr > 100.0, "scrub span is multi-century (%.0f yr)" % span_yr)
	_check(sim.T_MIN < 0.0 and sim.T_MAX > sim.T_IMPACT,
		"span brackets the whole campaign")

	# --- Every drawn planet resolves, everywhere on the clock ---------------
	# Both edges, not just mid-span: coverage runs out at the edges, and that is
	# exactly where a lookup starts failing silently onto the Sun.
	for t in [sim.T_MIN, 0.0, sim.T_IMPACT, sim.T_MAX]:
		var bad := PackedStringArray()
		for el in sim.planets:
			var p: Vector3 = sim.pos_ecl(el, t)
			if p == Vector3.ZERO or not (p.length() > 0.2 and p.length() < 31.0):
				bad.append("%s(%.2f AU)" % [el.name, p.length()])
		_check(bad.is_empty(), "all %d planets resolve at %s [%s]" %
			[sim.planets.size(), sim.year_at(t), "ok" if bad.is_empty() else ", ".join(bad)])

	# --- The bodies are the real ones, not a Kepler lookalike ---------------
	# Earth's ecliptic z is the sharp test: the ICRF->ecliptic obliquity rotation
	# is what keeps it near zero. Drop or invert that rotation and Earth swings
	# up to ~0.4 AU out of plane — the whole system visibly tilts ~23 deg.
	var earth: Vector3 = sim.pos_ecl(sim.earth_el, 0.0)
	_check(absf(earth.z) < 0.001,
		"Earth lies in the ecliptic (|z| = %.5f AU — obliquity rotation)" % absf(earth.z))
	_check(earth.length() > 0.98 and earth.length() < 1.02,
		"Earth ~1 AU (%.4f)" % earth.length())
	_check(sim.earth_el.source == "ephem" and sim.earth_el.naif_id == 399,
		"Earth is NAIF 399, the geocentre — NOT 3, the Earth-Moon barycentre "
		+ "(~4671 km away, an Earth-radius-scale b-plane error)")

	# Earth must actually move: a frozen lookup would pass every static check
	# above. Half a year apart it should be most of 2 AU away (opposite side).
	var e0: Vector3 = sim.pos_ecl(sim.earth_el, 0.0)
	var e6: Vector3 = sim.pos_ecl(sim.earth_el, 182.6)
	_check((e0 - e6).length() > 1.8,
		"Earth traverses its orbit (%.3f AU across half a year)" % (e0 - e6).length())

	# Mars is NAIF 4 (the barycentre) because de440s carries no Mars geocentre
	# segment at all — 499 does not resolve. Pinned so it is a decision, not a
	# typo someone "fixes" into a body that silently sits on the Sun.
	for el in sim.planets:
		if el.name == "MARS":
			_check(el.naif_id == 4, "Mars uses the barycentre (4); 499 is absent from de440s")

	# --- Orbit polylines are real arcs, not a fan of ZEROs ------------------
	var pts: PackedVector3Array = sim.orbit_points(sim.earth_el, 96)
	var zeros := 0
	var r_min := 1.0e9
	var r_max := 0.0
	for p in pts:
		if p == Vector3.ZERO:
			zeros += 1
		var r: float = p.length() / sim.AU
		r_min = minf(r_min, r)
		r_max = maxf(r_max, r)
	_check(pts.size() == 97 and zeros == 0,
		"Earth's orbit line is %d real points, %d collapsed to the Sun" % [pts.size(), zeros])
	_check(r_min > 0.97 and r_max < 1.02,
		"Earth's orbit line stays ~1 AU (%.3f .. %.3f)" % [r_min, r_max])
	# It closes: one period of a real orbit returns to its start.
	_check((pts[0] - pts[pts.size() - 1]).length() / sim.AU < 0.02,
		"Earth's orbit line closes after one period (gap %.4f AU)"
		% ((pts[0] - pts[pts.size() - 1]).length() / sim.AU))

	# --- The mission layer is dormant, and honest about it ------------------
	_check(not sim.mission_online, "mission layer is dormant (3C-2a)")
	_check(sim.ast_el.is_empty() and sim.comet_el.is_empty(),
		"no placeholder threat/comet is built while dormant")

	sim.free()
	print("----")
	print("%d failure(s)" % fails)
	quit(1 if fails > 0 else 0)
