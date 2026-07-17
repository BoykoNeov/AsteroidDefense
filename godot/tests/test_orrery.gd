extends SceneTree
## Headless verification of the orrery (3C-2a) and the real threat (3C-2b): the
## DE440 field and the core's integrated trajectory behind Sim's unchanged API.
##   godot --headless --path godot --script res://tests/test_orrery.gd
##
## This drives Sim exactly as the game does — _ready() resolves the kernels, loads
## the field and starts the scenario build; _poll_build drains it — and then asks
## the questions the display asks. It is the closest thing to "run the game and
## look" that a machine can answer.
##
## It covers the failure this whole slice exists to prevent: an unresolved lookup
## comes back as Vector3.ZERO, and ZERO is *the Sun's position* in this
## heliocentric frame, so a broken lookup does not draw as missing — it draws as a
## body sitting on the Sun. Every assertion below that looks pedantic about
## non-ZERO is guarding that one silent failure. 3C-2b adds a second, narrower
## instance of it: the threat exists for ~12 years of a ~300-year scrubbable
## clock, so the clock clamp does not cover it (see the span-gate block).

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

	# --- The threat builds off-thread and comes online ----------------------
	# Driven exactly as the game drives it: _ready() started the build, _process
	# polls it. ~10 s of real integration; the display stays live throughout, which
	# is the whole reason it is threaded.
	_check(sim.build_state == sim.Build.RUNNING, "_ready() started the scenario build")
	_check(not sim.mission_online, "the threat is not online before the build lands")
	_check(sim.ast_el.is_empty(), "no threat dict exists before the build lands")
	var t0 := Time.get_ticks_msec()
	var polls := 0
	while sim.build_state == sim.Build.RUNNING:
		sim._poll_build()
		polls += 1
		OS.delay_msec(20)
	_check(sim.build_state == sim.Build.READY,
		"the scenario built (%s)" % sim.build_error)
	_check(polls > 1, "the build ran on a worker, not the caller (%d polls, %d ms)"
		% [polls, Time.get_ticks_msec() - t0])
	_check(sim.mission_online, "the threat is online once the build lands")

	# The comet, interceptor and b-plane view stay dark: each is gated on the real
	# source that feeds it, and none of those exists yet. One flag for all four
	# would have lit three lies.
	_check(not sim.comet_online and not sim.interceptor_online
		and not sim.encounter_online,
		"comet/interceptor/encounter stay dormant on their own gates")

	# --- The threat is REAL: it arrives on Earth ----------------------------
	# The core integrated it backward from an impact condition, so the arc must
	# still end on the drawn Earth after a 12-year round trip through the
	# perturbed field. This is the whole scenario in one assertion.
	var ast_imp: Vector3 = sim.pos_ecl(sim.ast_el, sim.T_IMPACT)
	var earth_imp: Vector3 = sim.pos_ecl(sim.earth_el, sim.T_IMPACT)
	var gap_km: float = (ast_imp - earth_imp).length() * sim.AU_KM
	_check(ast_imp != Vector3.ZERO and gap_km < sim.cap_km,
		"the threat arrives on Earth at impact (gap %d km, inside the %d km capture disc)"
		% [int(gap_km), int(sim.cap_km)])
	_check(sim.cap_km > 6378.0,
		"the capture disc is gravitationally focused, wider than Earth (%.0f km, %.2f R_E)"
		% [sim.cap_km, sim.cap_km / sim.R_E])

	# --- The threat's span gate: the ZERO-is-the-Sun trap -------------------
	# The clock clamps to the KERNEL (~300 yr); the threat exists for ~12. Outside
	# that arc every lookup fails, and a failed lookup is ZERO — which in this
	# heliocentric frame is the SUN. So this gate is not cosmetic tidying: without
	# it the asteroid renders sitting on the Sun for most of the scrub range.
	_check(sim.threat_active(0.0) and sim.threat_active(sim.T_IMPACT),
		"the threat exists across its own campaign")
	_check(not sim.threat_active(sim.T_MAX) and not sim.threat_active(sim.T_MIN),
		"the threat does NOT exist at the kernel's span edges (%d, %d)"
		% [sim.year_at(sim.T_MIN), sim.year_at(sim.T_MAX)])
	# Prove the trap is real rather than trusting the gate: ask past the arc.
	_check(sim.mission.asteroid_position_ecl_au(sim.tdb(sim.T_MAX)) == Vector3.ZERO,
		"a lookup past the arc really does return ZERO — the Sun's position here")
	_check(sim.pos_ecl(sim.ast_el, sim.T_MAX) == Vector3.ZERO
		and sim.threat_range_km(sim.T_MAX) < 0.0,
		"Sim refuses to place or range the threat outside its arc")

	# --- The nominal track is a real arc ------------------------------------
	var trk: PackedVector3Array = sim.orbit_points(sim.ast_el, 128)
	var trk_zeros := 0
	for p in trk:
		if p == Vector3.ZERO:
			trk_zeros += 1
	_check(trk.size() > 2 and trk_zeros == 0,
		"the threat track is %d real points, %d collapsed to the Sun" % [trk.size(), trk_zeros])
	_check(sim.orbit_points(sim.ast_defl_el, 128).is_empty(),
		"the deflected track is EMPTY with no plan solved — not a zero-length one on the Sun")

	# --- The planner: edits are instant, the solve is debounced -------------
	_check(not sim.has_plan(), "no plan is solved at boot")
	sim.set_plan(180.0, 30.0, true)
	_check(sim.plan_solving, "an edit marks the verdict as pending")
	_check(sim.miss_label() == "SOLVING..." and sim.verdict_label() == "SOLVING...",
		"a pending verdict says so rather than reporting the previous plan's")
	sim._tick_plan_debounce(sim.PLAN_DEBOUNCE_S * 0.5)
	_check(sim.plan_solving and not sim.has_plan(),
		"edits inside the debounce window coalesce rather than each solving")
	var t1 := Time.get_ticks_msec()
	sim._tick_plan_debounce(sim.PLAN_DEBOUNCE_S)
	var solve_ms := Time.get_ticks_msec() - t1
	_check(not sim.plan_solving and sim.has_plan(),
		"the solve lands after the debounce (%d ms)" % solve_ms)
	# Guards the core's nominal cache from the frontend side: without it this was
	# ~11 s per keypress, which no debounce could have made usable.
	_check(solve_ms < 5000, "the solve is interactive (%d ms, ~11 000 before the cache)"
		% solve_ms)
	_check(not sim.orbit_points(sim.ast_defl_el, 128).is_empty(),
		"the deflected track exists once a plan is solved")

	# --- The verdict, and the clean-miss trap inside it ---------------------
	# `deflected_perigee_m` returns -1 for a CLEAN MISS — the best outcome — as
	# well as for no-plan. A verdict of `perigee > capture` alone therefore reads
	# the best possible deflection as a catastrophic failure. These two cases pin
	# both ends of that.
	sim.set_plan(600.0, 200.0, true)
	sim._tick_plan_debounce(1.0)
	_check(sim.deflect_ok,
		"200 m/s at 600 d lead clears Earth (miss %s, verdict %s)"
		% [sim.miss_label(), sim.verdict_label()])
	_check(not sim.miss_label().begins_with("-")
		and not sim.verdict_label().contains("IMPACT"),
		"a successful deflection never reports a negative miss or an impact verdict "
		+ "(miss %s / %s)" % [sim.miss_label(), sim.verdict_label()])

	sim.set_plan(sim.LEAD_MIN, sim.DV_MIN, true)
	sim._tick_plan_debounce(1.0)
	_check(not sim.deflect_ok and not sim.plan_clean_miss,
		"%.1f m/s at %d d lead does NOT save Earth (miss %s)"
		% [sim.DV_MIN, int(sim.LEAD_MIN), sim.miss_label()])

	# The middle band, and the readout a winning player actually sees most: a
	# FINITE perigee that still clears the capture disc. The two cases above pin
	# the sentinel ends (perigee < cap, and the -1 clean miss); this is the
	# `perigee_km > cap_km` half of the verdict, and the only case that pairs a
	# real "%.2f LD" number with EARTH CLEAR. Swept rather than hardcoded: the band
	# sits between the capture disc and the 500 000 km scan gate, and pinning one
	# tuned (dv, lead) would turn a physics change into a mystery failure here.
	var finite_safe := false
	for probe in [[60.0, 1.0], [90.0, 2.0], [120.0, 5.0], [180.0, 10.0]]:
		sim.set_plan(probe[0], probe[1], true)
		sim._tick_plan_debounce(1.0)
		if sim.deflect_ok and not sim.plan_clean_miss:
			finite_safe = true
			_check(sim.miss_label().ends_with("LD")
				and not sim.verdict_label().contains("IMPACT")
				and sim.miss_ld * sim.LD_KM > sim.cap_km,
				"%.0f m/s at %d d lead clears Earth with a FINITE miss: %s / %s"
				% [probe[1], int(probe[0]), sim.miss_label(), sim.verdict_label()])
			break
	_check(finite_safe,
		"a finite-but-safe deflection exists between the capture disc and the scan gate")

	sim.free()
	print("----")
	print("%d failure(s)" % fails)
	quit(1 if fails > 0 else 0)
