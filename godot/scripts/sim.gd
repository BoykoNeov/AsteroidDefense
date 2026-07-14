extends Node
## Central simulation state: mission clock, time warp, Keplerian bodies,
## mission timeline events. This is DISPLAY-GRADE propagation (f32 Kepler)
## used to stand up the visual layer; it will be replaced by the Rust f64
## core (gdext binding) which hands Godot focus-relative residuals per
## HANDOFF §7. Keep the API surface (positions per body per time) stable.

signal event_logged(line: String)

const AU := 10.0                       # Godot units per AU
const AU_KM := 1.495978707e8
const LD_KM := 384400.0                # lunar distance, km

# Mission timeline (days from epoch). The impact epoch is fixed by the
# threat; launch/intercept epochs come from the operator's plan ([M]).
const T_IMPACT := 1200.0
var T_LAUNCH := T_IMPACT - 420.0
var T_INTERCEPT := T_IMPACT - 180.0

var t := 0.0                           # mission-elapsed time, days
var paused := false
var warp_idx := 3
var time_dir := 1.0                    # +1 forward, -1 reverse (run time backward)
# Selectable warp rates, days/sec — extended into years/sec so the long clock
# (decades, several comet passes) scrubs in seconds without endless key-holding.
const WARP_STEPS: Array[float] = [0.1, 0.5, 2.0, 5.0, 15.0, 45.0, 120.0, 365.0, 1095.0, 3650.0]

# Clock bounds, mission-elapsed days (epoch 2031-01-01). Wide enough to run well
# before the campaign and far past it — a ~110-year window so long-period bodies
# show several passes. The clock clamps here; it never wraps.
const T_MIN := -3650.0                  # ~10 years before epoch
const T_MAX := 40000.0                  # ~110 years after epoch

var mono_font: SystemFont

# Bodies: dictionaries with keys
#   name, a (AU), e, i, om (Omega), w (omega), m0, n (rad/day), vis_r, kind
var planets: Array[Dictionary] = []
var earth_el: Dictionary
var ast_el: Dictionary                 # nominal threat orbit
var ast_defl_el: Dictionary            # post-intercept (deflected) orbit
var comet_el: Dictionary

# Mission plan (operator-editable in the planner until launch).
const LEAD_MIN := 30.0
const LEAD_MAX := 900.0
const DV_MIN := 0.1
const DV_MAX := 300.0
const PAD_D := 2.0                     # minimum days between "now" and launch
const R_E := 6371.0                    # km
const V_ESC := 11.186                  # km/s, Earth surface escape speed

var plan_lead_d := 180.0               # intercept lead before impact epoch, days
var plan_dv_ms := 30.0                 # impulse magnitude, m/s
var plan_retro := true                 # true = retrograde (against velocity)
var committed := false                 # launch scheduled
var planner_open := false              # planner panel showing (preview tracks)

var miss_ld := 0.0                     # projected post-burn close approach, LD
var dv_ms := 0.0                       # imparted delta-v, m/s (mirrors plan)
var cap_km := 0.0                      # gravitational capture radius, km
var deflect_ok := false                # projected miss clears the capture circle

signal plan_changed

var _events: Array[Dictionary] = []


func _ready() -> void:
	mono_font = SystemFont.new()
	mono_font.font_names = PackedStringArray(
		["Consolas", "Cascadia Mono", "Courier New", "Lucida Console"])

	_build_planets()
	_build_threat()
	_build_comet()
	set_plan(plan_lead_d, plan_dv_ms, plan_retro)
	_build_events()


func _process(delta: float) -> void:
	if paused:
		return
	var prev := t
	t = clampf(t + time_dir * WARP_STEPS[warp_idx] * delta, T_MIN, T_MAX)
	# Fire an event only when the clock *advances* across it (time_dir > 0). Running
	# time backward silently un-fires the events it passes, so advancing again
	# re-plays them — no spam while reversing or scrubbing.
	for ev in _events:
		var passed: bool = t >= ev.t
		if passed and not ev.fired and t > prev:
			event_logged.emit("E%+05d  %s" % [int(ev.t - T_IMPACT), ev.msg])
		ev.fired = passed


## Flip the time direction (forward <-> reverse). Warp magnitude is unchanged.
func reverse() -> void:
	time_dir = -time_dir


## Set the warp level directly (clamped to the available steps).
func set_warp(idx: int) -> void:
	warp_idx = clampi(idx, 0, WARP_STEPS.size() - 1)


## Scrub the clock to a fraction [0,1] of the full [T_MIN, T_MAX] span. Silent
## (no event replay) — the operator is dragging, not living through the timeline.
func scrub_frac(frac: float) -> void:
	jump(T_MIN + clampf(frac, 0.0, 1.0) * (T_MAX - T_MIN))


## The clock's current position as a fraction [0,1] of [T_MIN, T_MAX].
func clock_frac() -> float:
	return clampf((t - T_MIN) / (T_MAX - T_MIN), 0.0, 1.0)


# ---------------------------------------------------------------- bodies ---

func _build_planets() -> void:
	# [name, a AU, e, i deg, Omega deg, varpi deg, L deg, vis radius]
	var raw := [
		["MERCURY", 0.3871, 0.2056, 7.005, 48.331, 77.456, 252.251, 0.045],
		["VENUS",   0.7233, 0.0068, 3.395, 76.680, 131.564, 181.980, 0.075],
		["EARTH",   1.0000, 0.0167, 0.000, 0.000, 102.937, 100.464, 0.080],
		["MARS",    1.5237, 0.0934, 1.850, 49.558, 336.060, 355.450, 0.060],
		["JUPITER", 5.2026, 0.0484, 1.303, 100.556, 14.753, 34.404, 0.180],
		["SATURN",  9.5549, 0.0539, 2.486, 113.715, 92.432, 49.954, 0.150],
		["URANUS", 19.2184, 0.0473, 0.773, 74.006, 170.954, 313.238, 0.105],
		["NEPTUNE", 30.110, 0.0086, 1.770, 131.784, 44.965, 304.880, 0.100],
	]
	for r in raw:
		var el := _elements(r[1], r[2], deg_to_rad(r[3]), deg_to_rad(r[4]),
			deg_to_rad(r[5] - r[4]), deg_to_rad(r[6] - r[5]))
		el.name = r[0]
		el.vis_r = r[7]
		el.kind = "planet"
		planets.append(el)
		if r[0] == "EARTH":
			earth_el = el


func _build_threat() -> void:
	# Designer impactor 2031-XK (matches the Rust-core scenario family:
	# a = 0.855 AU, interior orbit hitting Earth at aphelion). Elements are
	# CONSTRUCTED from the impact condition so the tracks genuinely converge:
	# ascending node at the impact point, perihelion opposite it (w = 180 deg),
	# aphelion distance = Earth's heliocentric range at T_IMPACT.
	var p_earth := pos_ecl(earth_el, T_IMPACT)      # AU, ecliptic
	var r_imp := p_earth.length()
	var theta := atan2(p_earth.y, p_earth.x)

	var a := 0.855
	var e := r_imp / a - 1.0
	var el := _elements(a, e, deg_to_rad(3.4), theta, PI, 0.0)
	# Mean anomaly must be PI (aphelion) at T_IMPACT.
	el.m0 = wrapf(PI - el.n * T_IMPACT, -PI, PI)
	el.name = "2031-XK"
	el.vis_r = 0.030
	el.kind = "asteroid"
	ast_el = el

	# Gravitational capture radius from the nominal encounter speed: inside
	# this b-plane circle, focusing bends the track onto the surface.
	var ca := close_approach(ast_el)
	cap_km = R_E * sqrt(1.0 + pow(V_ESC / ca.v_kms.length(), 2.0))


# Moon: display-only geocentric circle. The true lunar distance (0.00257 AU
# = 0.026 scene units) sits INSIDE the wireframe Earth (vis_r 0.08), so the
# orbit radius is exaggerated the same way body radii are. Never feed this
# into encounter math — miss distances in LD come from the f64 pipeline.
const MOON_VIS_R := 0.022              # scene units
const MOON_ORBIT_VIS := 0.30           # scene units around Earth
const MOON_PERIOD_D := 27.322
const MOON_INCL := deg_to_rad(5.145)


## Moon offset from Earth in scene units (prograde, slightly inclined).
func moon_local(t_days: float) -> Vector3:
	var a := TAU * t_days / MOON_PERIOD_D
	return Vector3(cos(a), 0.0, -sin(a)).rotated(
		Vector3.RIGHT, MOON_INCL) * MOON_ORBIT_VIS


func moon_pos3d(t_days: float) -> Vector3:
	return pos3d(earth_el, t_days) + moon_local(t_days)


func _build_comet() -> void:
	comet_el = _elements(8.0, 0.90, deg_to_rad(28.0), deg_to_rad(210.0),
		deg_to_rad(80.0), 0.0)
	# Put it inbound, perihelion ~T_IMPACT + 300 d.
	comet_el.m0 = wrapf(-comet_el.n * (T_IMPACT + 300.0), -PI, PI)
	comet_el.name = "C/2029 K1"
	comet_el.vis_r = 0.040
	comet_el.kind = "comet"


func _elements(a: float, e: float, i: float, om: float, w: float, m0: float) -> Dictionary:
	return {
		"name": "", "a": a, "e": e, "i": i, "om": om, "w": w,
		"m0": m0, "n": TAU / (365.25 * pow(a, 1.5)),
		"vis_r": 0.05, "kind": "body",
	}


## Default timeline: no mission on file. Impact happens unless a plan is
## committed (which swaps in the mission timeline via _rebuild_events).
func _build_events() -> void:
	_events.clear()
	var raw := [
		[1.0, "TRACKING 2031-XK - EPHEMERIS UPDATED, P(IMPACT)=1.000"],
		[20.0, "NO DEFLECTION PLAN ON FILE - [M] MISSION PLANNER"],
		[T_IMPACT - 30.0, "FINAL WARNING - IMPACT E-030 D, NO MISSION COMMITTED"],
		[T_IMPACT, "SURFACE IMPACT - NO DEFLECTION ATTEMPTED"],
	]
	for r in raw:
		_events.append({"t": r[0], "msg": r[1], "fired": r[0] <= t})


## Committed-mission timeline; outcome events follow the projected verdict.
func _rebuild_events() -> void:
	_events.clear()
	var ml := "%.2f" % miss_ld
	var raw := [
		[1.0, "TRACKING 2031-XK - EPHEMERIS UPDATED, P(IMPACT)=1.000"],
		[T_LAUNCH - 14.0, "ATLAS-1 ON PAD - LAUNCH WINDOW OPEN"],
		[T_LAUNCH, "ATLAS-1 LAUNCH - TRANSFER INJECTION NOMINAL"],
		[minf(T_LAUNCH + 30.0, T_INTERCEPT - 5.0), "ATLAS-1 CRUISE - GUIDANCE LOCK ON 2031-XK"],
		[T_INTERCEPT, "KINETIC IMPACT CONFIRMED - DV %.1f M/S %s" %
			[plan_dv_ms, "RETROGRADE" if plan_retro else "PROGRADE"]],
	]
	if deflect_ok:
		raw.append([T_INTERCEPT + 20.0, "POST-BURN SOLUTION: MISS " + ml + " LD - THREAT RETIRED"])
		raw.append([T_IMPACT, "NOMINAL IMPACT EPOCH PASSED - EARTH SAFE"])
	else:
		raw.append([T_INTERCEPT + 20.0, "POST-BURN SOLUTION: MISS " + ml + " LD - INSUFFICIENT"])
		raw.append([T_IMPACT, "SURFACE IMPACT - DEFLECTION FAILED"])
	for r in raw:
		_events.append({"t": r[0], "msg": r[1], "fired": r[0] <= t})


# --------------------------------------------------------------- mission plan ---
# The planner edits (lead, dv, direction); the deflected orbit is rebuilt
# from the actual impulse-perturbed Kepler state, so the projected miss is
# emergent — this is the placeholder stand-in for core's DeflectionScenario.

const MU := pow(TAU / 365.25, 2.0)     # AU^3/day^2, consistent with n above
const MS_TO_AUD := 86.4 / AU_KM        # 1 m/s in AU/day


func cruise_d(lead_d: float = -1.0) -> float:
	return clampf(lead_d if lead_d > 0.0 else plan_lead_d, 60.0, 240.0)


func locked() -> bool:
	return committed and t >= T_LAUNCH


func burned() -> bool:
	return committed and t >= T_INTERCEPT


## Longest lead the launch window still allows (launch >= now + PAD_D).
func lead_cap() -> float:
	var avail := T_IMPACT - t - PAD_D
	var cap: float = avail - 240.0
	if cap < 240.0:
		cap = minf(avail * 0.5, 240.0)
	if cap < 60.0:
		cap = minf(avail - 60.0, 60.0)
	return clampf(cap, 0.0, LEAD_MAX)


## Apply a mission plan. The impulse is added to the heliocentric velocity
## at the intercept epoch (f64 throughout) and the deflected element set is
## recovered from the perturbed state, so divergence and miss distance are
## genuine orbital mechanics, not a scripted offset.
func set_plan(lead_d: float, dv: float, retro: bool) -> void:
	plan_lead_d = clampf(lead_d, LEAD_MIN, maxf(lead_cap(), LEAD_MIN))
	plan_dv_ms = clampf(dv, DV_MIN, DV_MAX)
	plan_retro = retro
	T_INTERCEPT = T_IMPACT - plan_lead_d
	T_LAUNCH = T_INTERCEPT - cruise_d()
	dv_ms = plan_dv_ms

	var r := pos_ecl64(ast_el, T_INTERCEPT)
	var v := vel_ecl64(ast_el, T_INTERCEPT)
	var vlen := sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
	var dv_aud := plan_dv_ms * MS_TO_AUD * (-1.0 if plan_retro else 1.0)
	for k in 3:
		v[k] += v[k] / vlen * dv_aud
	var el := elements_from_rv(r, v, T_INTERCEPT)
	el.name = "2031-XK DEFL"
	el.vis_r = ast_el.vis_r
	el.kind = "asteroid"
	ast_defl_el = el

	miss_ld = close_approach(ast_defl_el).r_km / LD_KM
	deflect_ok = miss_ld * LD_KM > cap_km
	if committed:
		_rebuild_events()
	plan_changed.emit()


func adjust_lead(dd: float) -> void:
	if _plan_edit_blocked():
		return
	set_plan(plan_lead_d + dd, plan_dv_ms, plan_retro)


func adjust_dv(factor: float) -> void:
	if _plan_edit_blocked():
		return
	set_plan(plan_lead_d, plan_dv_ms * factor, plan_retro)


func toggle_burn_dir() -> void:
	if _plan_edit_blocked():
		return
	set_plan(plan_lead_d, plan_dv_ms, not plan_retro)


func _plan_edit_blocked() -> bool:
	if locked():
		event_logged.emit("PLAN LOCKED - INTERCEPTOR IN FLIGHT")
		return true
	return false


func try_commit() -> void:
	if committed:
		event_logged.emit("MISSION ALREADY COMMITTED")
		return
	if T_LAUNCH < t + PAD_D:
		event_logged.emit("COMMIT REFUSED - LAUNCH WINDOW CLOSED, REDUCE LEAD")
		return
	committed = true
	_rebuild_events()
	event_logged.emit("MISSION COMMITTED - LAUNCH E-%04d, INTERCEPT E-%04d" %
		[int(T_IMPACT - T_LAUNCH), int(plan_lead_d)])


## First-order estimate of the dv needed for a 1.0 LD miss at this lead
## (b-plane displacement is ~linear in dv; nominal b is ~0 by construction).
func req_dv_1ld() -> float:
	return plan_dv_ms / maxf(miss_ld, 1.0e-4)


## Heliocentric velocity, AU/day, f64 components (central difference).
## dt balances truncation vs f64 cancellation; 0.002 d keeps the derived
## element set within a few km of truth over the full mission arc.
func vel_ecl64(el: Dictionary, t_days: float) -> PackedFloat64Array:
	var dt := 0.002
	var p1 := pos_ecl64(el, t_days + dt)
	var p0 := pos_ecl64(el, t_days - dt)
	return PackedFloat64Array([
		(p1[0] - p0[0]) / (2.0 * dt),
		(p1[1] - p0[1]) / (2.0 * dt),
		(p1[2] - p0[2]) / (2.0 * dt)])


## Classical elements from a heliocentric ecliptic state (AU, AU/day) at
## epoch t_days, same dictionary shape as _elements(). Elliptic, nonzero
## inclination/eccentricity only — fine for the designer threat.
func elements_from_rv(r: PackedFloat64Array, v: PackedFloat64Array,
		t_days: float) -> Dictionary:
	var rlen := sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
	var v2 := v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
	var rv := r[0] * v[0] + r[1] * v[1] + r[2] * v[2]
	var hx := r[1] * v[2] - r[2] * v[1]
	var hy := r[2] * v[0] - r[0] * v[2]
	var hz := r[0] * v[1] - r[1] * v[0]
	var hlen := sqrt(hx * hx + hy * hy + hz * hz)

	var a := 1.0 / (2.0 / rlen - v2 / MU)
	var c := v2 - MU / rlen
	var ex := (c * r[0] - rv * v[0]) / MU
	var ey := (c * r[1] - rv * v[1]) / MU
	var ez := (c * r[2] - rv * v[2]) / MU
	var e := sqrt(ex * ex + ey * ey + ez * ez)
	var i := acos(clampf(hz / hlen, -1.0, 1.0))

	var nx := -hy                       # node vector k x h
	var ny := hx
	var nlen := sqrt(nx * nx + ny * ny)
	var om := atan2(ny, nx)
	var w := acos(clampf((nx * ex + ny * ey) / (nlen * e), -1.0, 1.0))
	if ez < 0.0:
		w = -w
	var nu := acos(clampf((ex * r[0] + ey * r[1] + ez * r[2]) / (e * rlen), -1.0, 1.0))
	if rv < 0.0:
		nu = -nu
	var ecc := 2.0 * atan2(sqrt(1.0 - e) * sin(nu * 0.5), sqrt(1.0 + e) * cos(nu * 0.5))
	var m := ecc - e * sin(ecc)
	var el := _elements(a, e, i, om, w, 0.0)
	el.m0 = wrapf(m - el.n * t_days, -PI, PI)
	return el


# ------------------------------------------------------------ propagation ---

static func solve_kepler(m: float, e: float) -> float:
	var ecc := m + e * sin(m)
	for _i in 10:
		var f := ecc - e * sin(ecc) - m
		ecc -= f / (1.0 - e * cos(ecc))
	return ecc


## Heliocentric ecliptic position, AU, at mission time t_days.
func pos_ecl(el: Dictionary, t_days: float) -> Vector3:
	var m: float = wrapf(el.m0 + el.n * t_days, -PI, PI)
	var ecc := solve_kepler(m, el.e)
	var nu := 2.0 * atan2(sqrt(1.0 + el.e) * sin(ecc * 0.5),
		sqrt(1.0 - el.e) * cos(ecc * 0.5))
	var r: float = el.a * (1.0 - el.e * cos(ecc))
	var xp := r * cos(nu)
	var yp := r * sin(nu)

	var co: float = cos(el.om)
	var so: float = sin(el.om)
	var cw: float = cos(el.w)
	var sw: float = sin(el.w)
	var ci: float = cos(el.i)
	var si: float = sin(el.i)
	return Vector3(
		(co * cw - so * sw * ci) * xp + (-co * sw - so * cw * ci) * yp,
		(so * cw + co * sw * ci) * xp + (-so * sw + co * cw * ci) * yp,
		(sw * si) * xp + (cw * si) * yp)


## Ecliptic (AU) -> Godot scene units. Ecliptic plane = XZ, north = +Y.
func ecl_to_godot(v: Vector3) -> Vector3:
	return Vector3(v.x, v.z, -v.y) * AU


## Scene-space position of a body at time t_days.
func pos3d(el: Dictionary, t_days: float) -> Vector3:
	return ecl_to_godot(pos_ecl(el, t_days))


## Full-orbit polyline in scene units (for static orbit tracks).
func orbit_points(el: Dictionary, count: int = 192) -> PackedVector3Array:
	var pts := PackedVector3Array()
	var saved: float = el.m0
	for k in count + 1:
		var m := TAU * float(k) / float(count)
		el.m0 = m
		pts.append(ecl_to_godot(pos_ecl(el, 0.0)))
	el.m0 = saved
	return pts


# ------------------------------------------------- encounter geometry (f64) ---
# GDScript scalars are 64-bit; only Vector3 truncates to f32. Geocentric
# differences near encounter (two ~1 AU vectors a few thousand km apart)
# lose ~18 km to an f32 cast, so these helpers keep every component in
# doubles and only cast the SMALL residual — the same subtract-then-cast
# contract the gdext binding will follow (HANDOFF §7).

## Heliocentric ecliptic position as 64-bit components [x, y, z], AU.
## Mirror of pos_ecl — keep the math in sync (pos_ecl stays separate to
## avoid per-call array churn on the hot orbit-trace path).
func pos_ecl64(el: Dictionary, t_days: float) -> PackedFloat64Array:
	var m: float = wrapf(el.m0 + el.n * t_days, -PI, PI)
	var ecc := solve_kepler(m, el.e)
	var nu := 2.0 * atan2(sqrt(1.0 + el.e) * sin(ecc * 0.5),
		sqrt(1.0 - el.e) * cos(ecc * 0.5))
	var r: float = el.a * (1.0 - el.e * cos(ecc))
	var xp := r * cos(nu)
	var yp := r * sin(nu)

	var co: float = cos(el.om)
	var so: float = sin(el.om)
	var cw: float = cos(el.w)
	var sw: float = sin(el.w)
	var ci: float = cos(el.i)
	var si: float = sin(el.i)
	return PackedFloat64Array([
		(co * cw - so * sw * ci) * xp + (-co * sw - so * cw * ci) * yp,
		(so * cw + co * sw * ci) * xp + (-so * sw + co * cw * ci) * yp,
		(sw * si) * xp + (cw * si) * yp])


## Geocentric position of a body, km, ecliptic axes. Subtracted in doubles
## FIRST, then cast: the residual is small, so f32 is safe.
func geo_km(el: Dictionary, t_days: float) -> Vector3:
	var p := pos_ecl64(el, t_days)
	var e := pos_ecl64(earth_el, t_days)
	return Vector3(
		(p[0] - e[0]) * AU_KM, (p[1] - e[1]) * AU_KM, (p[2] - e[2]) * AU_KM)


## Geocentric velocity, km/s, by central difference of the f64 residuals.
func geo_vel_kms(el: Dictionary, t_days: float) -> Vector3:
	var dt := 0.02                        # days
	return (geo_km(el, t_days + dt) - geo_km(el, t_days - dt)) / (2.0 * dt * 86400.0)


## Closest Earth approach of a track near the impact epoch (ternary search;
## range is unimodal inside +/-80 d of the designed encounter).
func close_approach(el: Dictionary) -> Dictionary:
	var lo := T_IMPACT - 80.0
	var hi := T_IMPACT + 80.0
	for _i in 96:
		var m1 := lo + (hi - lo) / 3.0
		var m2 := hi - (hi - lo) / 3.0
		if geo_km(el, m1).length() < geo_km(el, m2).length():
			hi = m2
		else:
			lo = m1
	var t_ca := (lo + hi) * 0.5
	return {
		"t": t_ca,
		"r_km": geo_km(el, t_ca).length(),
		"v_kms": geo_vel_kms(el, t_ca),
	}


# ------------------------------------------------------------ interceptor ---

func interceptor_phase(t_days: float) -> String:
	if not committed:
		return "STANDBY"
	if t_days < T_LAUNCH:
		return "PRELAUNCH"
	if t_days < T_INTERCEPT:
		return "CRUISE"
	return "EXPENDED"


## Cruise path: quadratic bezier Earth(T_LAUNCH) -> asteroid(T_INTERCEPT).
## Placeholder for a Lambert arc from the Rust core.
func interceptor_pos(t_days: float) -> Vector3:
	var p0 := pos3d(earth_el, T_LAUNCH)
	var p1 := pos3d(ast_el, T_INTERCEPT)
	var ctrl := (p0 + p1) * 0.5 * 0.88     # slight sunward bow (inward transfer)
	var u: float = clampf((t_days - T_LAUNCH) / (T_INTERCEPT - T_LAUNCH), 0.0, 1.0)
	return p0.lerp(ctrl, u).lerp(ctrl.lerp(p1, u), u)


func interceptor_path(count: int = 96) -> PackedVector3Array:
	var pts := PackedVector3Array()
	for k in count + 1:
		var td := T_LAUNCH + (T_INTERCEPT - T_LAUNCH) * float(k) / float(count)
		pts.append(interceptor_pos(td))
	return pts


# ------------------------------------------------------------------- misc ---

## Range Earth <-> active asteroid track, km.
func threat_range_km(t_days: float) -> float:
	var el := ast_defl_el if (committed and t_days >= T_INTERCEPT) else ast_el
	return (pos_ecl(el, t_days) - pos_ecl(earth_el, t_days)).length() * AU_KM


## Jump the mission clock; events at or before the new time are marked
## consumed silently so the console only shows live traffic.
func jump(to_days: float) -> void:
	t = clampf(to_days, T_MIN, T_MAX)
	for ev in _events:
		ev.fired = ev.t <= t


func jump_next_milestone() -> void:
	var ms := [T_LAUNCH - 10.0, T_INTERCEPT - 10.0, T_IMPACT - 20.0, T_IMPACT + 60.0] \
		if committed else [T_IMPACT - 20.0, T_IMPACT + 60.0]
	for m in ms:
		if t < m - 0.5:
			jump(m)
			event_logged.emit("CLOCK SLEW - MJD-REL %07.1f" % t)
			return
	jump(0.0)
	event_logged.emit("CLOCK SLEW - MISSION START")


func warp_label() -> String:
	var w := WARP_STEPS[warp_idx]
	var rate: String
	if w < 1.0:
		rate = "x%.1f D/S" % w
	elif w < 365.0:
		rate = "x%d D/S" % int(w)
	else:
		rate = "x%.1f Y/S" % (w / 365.25)
	return ("<< " + rate) if time_dir < 0.0 else rate


func blink(hz: float = 2.0) -> bool:
	return fmod(Time.get_ticks_msec() / 1000.0 * hz, 1.0) < 0.5


## Calendar readout: epoch 2031-01-01 + t days (display only).
func date_string() -> String:
	var days := int(t)
	var date := Time.get_date_dict_from_unix_time(1924992000 + days * 86400)
	return "%04d-%02d-%02d" % [date.year, date.month, date.day]
