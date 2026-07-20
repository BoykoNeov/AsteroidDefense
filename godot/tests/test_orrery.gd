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

	# The b-plane view lights WITH the threat (3C-2c): its geometry is the same
	# `EncounterFrame` the scenario build produces, so it is real exactly when the
	# threat is. The comet lights too as of 3D — but on its own gate, set from what
	# the catalog actually holds, not alongside `mission_online`. The interceptor
	# stays dark: still no Lambert solver behind its cosmetic bezier. This is why
	# there are four flags and not one.
	_check(sim.encounter_online,
		"the b-plane view lights with the threat — same frame, same propagation")
	_check(sim.comet_online,
		"the comet lights from the catalog the worker flew alongside the threat")
	_check(not sim.interceptor_online,
		"the interceptor stays dormant on its own gate (no Lambert solver behind it)")

	# --- The comet is the core's integration, gated on its own span ---------
	# It rode the build worker through the same validated field as the threat, so
	# it is drawable exactly where it was flown — and nowhere else. The gate is the
	# point: outside the span the binding returns ZERO, and ZERO here is the SUN.
	_check(sim.comet_el.source == "catalog" and not sim.comet_el.has("m0"),
		"the comet is a catalog body, not a GDScript Kepler ellipse")
	var comet_span_yr: float = (sim.comet_el.t_max - sim.comet_el.t_min) / 365.25
	_check(comet_span_yr > 21.0 and comet_span_yr < 24.0,
		"the comet's arc is the ~22.6 yr orbit it was authored as (%.1f yr)" % comet_span_yr)
	_check(sim.catalog_active(sim.comet_el, sim.comet_el.t_min + 1.0),
		"the comet is active inside its propagated span")
	_check(not sim.catalog_active(sim.comet_el, sim.comet_el.t_max + 10.0),
		"the comet is NOT active past its span — where a lookup would draw it on the Sun")
	var comet_off: Vector3 = sim.pos_ecl(sim.comet_el, sim.comet_el.t_max + 10.0)
	_check(comet_off == Vector3.ZERO,
		"an out-of-span comet read is refused by the gate, not served as a position")

	# On its arc it is a real body at a real distance — and NOT at the origin,
	# which is the failure the gate exists to catch.
	var comet_on: Vector3 = sim.pos_ecl(sim.comet_el, sim.comet_el.t_min + 100.0)
	_check(comet_on != Vector3.ZERO and comet_on.length() > 0.7
			and comet_on.length() < 15.6,
		"the comet sits on its designed ellipse in-span (%.2f AU)" % comet_on.length())

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
	# The core reports -1 for a CLEAN MISS — the best outcome — as well as for
	# no-plan. A bare `|B| > capture` verdict therefore reads the best possible
	# deflection as a catastrophic failure. These two cases pin both ends of that.
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
	# FINITE |B| that still clears the capture disc. The two cases above pin
	# the sentinel ends (|B| < cap, and the -1 clean miss); this is the
	# `b_km > cap_km` half of the verdict, and the only case that pairs a
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

	# --- The band the OLD verdict got wrong --------------------------------
	# The verdict is `|B| > capture`: the un-focused asymptotic miss against the
	# target enlarged for focusing (the core's own is_hit). It used to be
	# `perigee > capture`, which is neither coherent pair and ~1.5x too strict,
	# because the perigee is ALREADY focused — it pairs with R_E, never with the
	# capture disc.
	#
	# This plan is where the two answers actually differ, so it is the one worth
	# pinning: 0.2 m/s one period out puts |B| outside the disc (a real miss, with
	# daylight to spare) while its perigee sits inside it. The old test printed
	# SURFACE IMPACT over a deflection that works.
	sim.set_plan(sim.threat_period_d(), 0.2, true)
	sim._tick_plan_debounce(1.0)
	var band_b: float = sim.miss_ld * sim.LD_KM
	var band_p: float = sim.perigee_ld(true) * sim.LD_KM
	_check(band_b > sim.cap_km and band_p < sim.cap_km,
		"0.2 m/s at one period lead sits in the disagreement band: |B| %d km > capture "
		% int(band_b) + "%d km > perigee %d km" % [int(sim.cap_km), int(band_p)])
	_check(sim.deflect_ok and not sim.verdict_label().contains("IMPACT"),
		"...and it is a MISS, not the impact the perigee-vs-capture bar called it (%s / %s)"
		% [sim.miss_label(), sim.verdict_label()])
	_check(band_p > sim.R_E,
		"the other coherent pair agrees: perigee %d km clears R_E %d km"
		% [int(band_p), int(sim.R_E)])

	# --- The b-plane view reads the core (3C-2c) ---------------------------
	var enc_nom: PackedVector3Array = sim.encounter_track(false)
	var enc_defl: PackedVector3Array = sim.encounter_track(true)
	_check(enc_nom.size() > 1000 and enc_defl.size() == enc_nom.size(),
		"both encounter tracks are densely sampled (%d / %d pts)"
		% [enc_nom.size(), enc_defl.size()])
	# s = depth along the incoming asymptote: the window straddles the b-plane.
	_check(enc_nom[0].z < 0.0 and enc_nom[enc_nom.size() - 1].z > 0.0,
		"the nominal track runs inbound (s<0) to outbound (s>0)")

	# The b-point is the operative mark, and |B| must be the number the verdict
	# reads — the picture and the panel cannot be allowed to drift apart.
	var bp: Vector3 = sim.encounter_b_point(false)
	_check(bp != Vector3.ZERO
		and absf(Vector2(bp.x, bp.y).length() / sim.LD_KM - sim.nominal_b_ld()) < 1e-3,
		"the nominal b-point's plotted distance IS |B| (%.4f LD)" % sim.nominal_b_ld())
	_check(sim.nominal_b_ld() * sim.LD_KM < sim.cap_km,
		"the nominal b-point falls INSIDE the capture disc — it is the hit (%d < %d km)"
		% [int(sim.nominal_b_ld() * sim.LD_KM), int(sim.cap_km)])
	_check(absf(sim.encounter_v_inf_kms() - 7.63) < 0.5,
		"v_inf is the hyperbolic excess ~7.63 km/s, not the config's 18 km/s at the "
		+ "impact point (got %.2f)" % sim.encounter_v_inf_kms())

	# The encounter window is the core's, centred on impact.
	var esp: PackedFloat64Array = sim.encounter_span_days()
	_check(esp.size() == 2 and absf((esp[0] + esp[1]) * 0.5 - sim.T_IMPACT) < 0.01
		and absf((esp[1] - esp[0]) - 3.0) < 0.01,
		"the encounter window is +/-1.5 d centred on impact")

	# --- The closest-approach snap lands where it claims ---------------------
	# `encounter_ca_day` argmins the core's own track. Two things must hold, and
	# ONE comparison settles both: the minimum sample range must equal the core's
	# reported perigee. If it does, the b-plane frame really is Earth-centred (so
	# |p| is geocentric distance and the argmin really is closest approach); if it
	# does not, either the track never spans perigee or the origin is not where the
	# view assumes — and in that case the snap would park the clock at a confidently
	# wrong instant, which is worse than the manual scrub it replaces.
	var ca_day: float = sim.encounter_ca_day()
	_check(not is_nan(ca_day) and absf(ca_day - sim.T_IMPACT) < 1.5,
		"the CA snap lands inside the +/-1.5 d window (CA %+.3f d)" % (ca_day - sim.T_IMPACT))
	var ca_trk: PackedVector3Array = sim.encounter_track(false)
	var min_r := INF
	for p in ca_trk:
		min_r = minf(min_r, p.length())
	var perigee_km: float = sim.perigee_ld(false) * sim.LD_KM
	# The band is DERIVED, not guessed. A sampled minimum can never beat the true
	# perigee, and it can never be worse than a half-sample-step of travel past it:
	#   ca_r_max = sqrt(r_p^2 + (v_p * dt/2)^2)
	# with v_p from vis-viva at perigee (v_inf^2 + 2*mu/r_p). Here that is ~18 km/s
	# and dt/2 ~93 s, so the rock moves ~1700 km through the turn between samples
	# and the sampled minimum legitimately reads ~3260 km against a 3000 km perigee.
	# A guessed 1% band failed on exactly that — the sampling floor, not an error.
	# The band still discriminates what it is here to discriminate: a frame whose
	# origin was not Earth's centre would miss by an Earth radius or an AU, not 8%.
	const MU_E := 398600.4418  # km^3/s^2
	var dt_half: float = (esp[1] - esp[0]) * sim.DAY_S / float(ca_trk.size() - 1) * 0.5
	var v_p: float = sqrt(pow(sim.encounter_v_inf_kms(), 2.0) + 2.0 * MU_E / perigee_km)
	var ca_r_max: float = sqrt(perigee_km * perigee_km + pow(v_p * dt_half, 2.0))
	_check(perigee_km > 0.0 and min_r >= perigee_km - 1.0 and min_r <= ca_r_max,
		"the track's minimum range IS the perigee, to the sampling floor "
		+ "(%d km in [%d, %d]) — the frame is Earth-centred and the argmin is CA"
		% [int(min_r), int(perigee_km), int(ca_r_max)])
	# Before the burn the nominal track is the live one; the snap must agree with
	# what `_draw_marker` will draw, or it parks the clock where there is no marker.
	_check(not sim.deflected_is_live(sim.encounter_track(true).is_empty()),
		"with no burn flown, the nominal track is the live one")

	# A clean miss has NO b-point. ZERO here is Earth's dead centre, so drawing it
	# unconditionally would mark the best outcome as a bullseye.
	sim.set_plan(600.0, 200.0, true)
	sim._tick_plan_debounce(1.0)
	_check(sim.plan_clean_miss and sim.encounter_b_point(true) == Vector3.ZERO,
		"a clean miss reports NO deflected b-point (ZERO = Earth's centre, not a hit)")

	# --- The CA snap on the DEFLECTED branch, which is the hard one ----------
	# Everything above ran before any burn, so it only ever exercised the nominal
	# track. The branch that actually ships once a player commits is the deflected
	# one, and its closest approach is time-shifted ~0.53 d off the nominal — which
	# is precisely why `deflected_is_live` exists as one shared rule. Left untested,
	# a broken deflected selection would keep this suite green while the marker
	# silently stopped appearing in the only situation a player reaches it in.
	# The band plan (not the clean miss above) because a clean miss leaves the
	# deflected track EMPTY — there would be nothing to argmin.
	sim.set_plan(sim.threat_period_d(), 0.2, true)
	sim._tick_plan_debounce(1.0)
	sim.try_commit()
	sim.jump(sim.T_IMPACT)
	var defl_trk: PackedVector3Array = sim.encounter_track(true)
	_check(sim.committed and sim.burned() and not defl_trk.is_empty()
		and sim.deflected_is_live(defl_trk.is_empty()),
		"after the burn is flown, the DEFLECTED track is the live one")
	var ca_defl: float = sim.encounter_ca_day()
	_check(not is_nan(ca_defl) and absf(ca_defl - sim.T_IMPACT) < 1.5,
		"the CA snap follows the deflected track into the window (CA %+.3f d)"
		% (ca_defl - sim.T_IMPACT))
	# And it is the DEFLECTED closest approach, not the nominal one wearing its
	# name: the deflected perigee is thousands of km wider, so the two minima
	# cannot coincide. Ties them to different tracks, not just to "some track".
	var min_defl := INF
	for p in defl_trk:
		min_defl = minf(min_defl, p.length())
	_check(min_defl > min_r * 1.5,
		"it minimises the deflected track, not the nominal (%d vs %d km)"
		% [int(min_defl), int(min_r)])

	sim.free()
	print("----")
	print("%d failure(s)" % fails)
	quit(1 if fails > 0 else 0)
