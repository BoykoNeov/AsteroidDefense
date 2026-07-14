extends SceneTree
## Headless checks for the mission-plan math in sim.gd:
##   godot --headless --path godot --script res://tests/test_sim.gd
## Exercises elements_from_rv round-trip, impulse -> emergent miss,
## dv linearity, lead-time growth, and the capture-radius verdict.

var fails := 0


func _check(ok: bool, msg: String) -> void:
	if ok:
		print("PASS  " + msg)
	else:
		print("FAIL  " + msg)
		fails += 1


func _init() -> void:
	var sim = load("res://scripts/sim.gd").new()
	sim._build_planets()
	sim._build_threat()

	# 1. Designer threat actually hits: nominal CA inside the capture circle.
	var ca_nom: Dictionary = sim.close_approach(sim.ast_el)
	print("nominal CA %.0f km, capture %.0f km, v_rel %.2f km/s"
		% [ca_nom.r_km, sim.cap_km, ca_nom.v_kms.length()])
	_check(ca_nom.r_km < sim.cap_km, "nominal track impacts (CA < capture)")

	# 2. elements_from_rv round-trip: rebuild the nominal orbit from its own
	#    state at t=500 d; positions must agree over the whole arc.
	var el2: Dictionary = sim.elements_from_rv(
		sim.pos_ecl64(sim.ast_el, 500.0), sim.vel_ecl64(sim.ast_el, 500.0), 500.0)
	var worst := 0.0
	for tt in [0.0, 300.0, 900.0, 1200.0]:
		var p: PackedFloat64Array = sim.pos_ecl64(sim.ast_el, tt)
		var q: PackedFloat64Array = sim.pos_ecl64(el2, tt)
		var d: float = sqrt(pow(p[0] - q[0], 2) + pow(p[1] - q[1], 2)
			+ pow(p[2] - q[2], 2)) * sim.AU_KM
		worst = maxf(worst, d)
	print("round-trip worst error %.2f km" % worst)
	_check(worst < 100.0, "rv->elements round-trip < 100 km over 1200 d")

	# 3. Impulse produces an emergent miss; ~linear in dv at fixed lead.
	sim.set_plan(180.0, 10.0, true)
	var m10: float = sim.miss_ld
	sim.set_plan(180.0, 20.0, true)
	var m20: float = sim.miss_ld
	print("lead 180: miss(10 m/s)=%.3f LD  miss(20 m/s)=%.3f LD" % [m10, m20])
	_check(m10 > 0.01, "10 m/s at 180 d lead moves the CA off the surface scale")
	_check(m20 > 1.5 * m10 and m20 < 2.5 * m10, "miss ~linear in dv (ratio %.2f)" % (m20 / m10))

	# 4. Longer lead helps at fixed dv (the thesis).
	sim.set_plan(100.0, 10.0, true)
	var m_short: float = sim.miss_ld
	sim.set_plan(600.0, 10.0, true)
	var m_long: float = sim.miss_ld
	print("dv 10 m/s: miss(lead 100)=%.3f LD  miss(lead 600)=%.3f LD" % [m_short, m_long])
	_check(m_long > 1.5 * m_short, "longer lead -> larger miss at fixed dv")

	# 5. Verdict: token nudge still impacts, big early burn clears Earth.
	sim.set_plan(60.0, 0.1, true)
	var weak_ok: bool = sim.deflect_ok
	var weak_miss: float = sim.miss_ld
	sim.set_plan(600.0, 100.0, true)
	print("weak plan miss=%.4f LD ok=%s | strong plan miss=%.2f LD ok=%s"
		% [weak_miss, weak_ok, sim.miss_ld, sim.deflect_ok])
	_check(not weak_ok, "0.1 m/s at 60 d lead is insufficient")
	_check(sim.deflect_ok and sim.miss_ld > 1.0, "100 m/s at 600 d lead clears Earth")

	# 6. Launch-window cap: lead is clamped so launch stays in the future.
	sim.t = 1100.0
	sim.set_plan(600.0, 10.0, true)
	print("t=1100: clamped lead %.0f d, launch at %.0f (t+%.0f)"
		% [sim.plan_lead_d, sim.T_LAUNCH, sim.T_LAUNCH - sim.t])
	_check(sim.T_LAUNCH >= sim.t + sim.PAD_D - 0.5, "late plan keeps launch >= now + pad")

	sim.free()
	print("----")
	print("%d failure(s)" % fails)
	quit(1 if fails > 0 else 0)
