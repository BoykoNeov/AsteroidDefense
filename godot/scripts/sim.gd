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

# Mission timeline (days from epoch).
const T_IMPACT := 1200.0
const T_LAUNCH := T_IMPACT - 420.0
const T_INTERCEPT := T_IMPACT - 180.0

var t := 0.0                           # mission-elapsed time, days
var paused := false
var warp_idx := 3
const WARP_STEPS: Array[float] = [0.1, 0.5, 2.0, 5.0, 15.0, 45.0, 120.0]  # days/sec

var mono_font: SystemFont

# Bodies: dictionaries with keys
#   name, a (AU), e, i, om (Omega), w (omega), m0, n (rad/day), vis_r, kind
var planets: Array[Dictionary] = []
var earth_el: Dictionary
var ast_el: Dictionary                 # nominal threat orbit
var ast_defl_el: Dictionary            # post-intercept (deflected) orbit
var comet_el: Dictionary

var miss_ld := 0.0                     # projected miss distance after burn, LD
var dv_ms := 0.0                       # imparted delta-v (display value), m/s

var _events: Array[Dictionary] = []


func _ready() -> void:
	mono_font = SystemFont.new()
	mono_font.font_names = PackedStringArray(
		["Consolas", "Cascadia Mono", "Courier New", "Lucida Console"])

	_build_planets()
	_build_threat()
	_build_comet()
	_build_events()


func _process(delta: float) -> void:
	if paused:
		return
	t += WARP_STEPS[warp_idx] * delta
	for ev in _events:
		if not ev.fired and t >= ev.t:
			ev.fired = true
			event_logged.emit("E%+05d  %s" % [int(ev.t - T_IMPACT), ev.msg])


# ---------------------------------------------------------------- bodies ---

func _build_planets() -> void:
	# [name, a AU, e, i deg, Omega deg, varpi deg, L deg, vis radius]
	var raw := [
		["MERCURY", 0.3871, 0.2056, 7.005, 48.331, 77.456, 252.251, 0.045],
		["VENUS",   0.7233, 0.0068, 3.395, 76.680, 131.564, 181.980, 0.075],
		["EARTH",   1.0000, 0.0167, 0.000, 0.000, 102.937, 100.464, 0.080],
		["MARS",    1.5237, 0.0934, 1.850, 49.558, 336.060, 355.450, 0.060],
		["JUPITER", 5.2026, 0.0484, 1.303, 100.556, 14.753, 34.404, 0.180],
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

	# Deflected orbit: retrograde along-track burn at T_INTERCEPT shrinks a,
	# same phase at the burn epoch. Divergence after the burn is emergent from
	# the period change. da/a is display-exaggerated so the split reads on a
	# solar-system-scale screen; the HUD reports the honest equivalent dv.
	var da_over_a := 2.0e-3
	var a2 := a * (1.0 - da_over_a)
	var d := ast_el.duplicate()
	d.a = a2
	d.n = TAU / (365.25 * pow(a2, 1.5))
	var m_at_burn: float = wrapf(ast_el.m0 + ast_el.n * T_INTERCEPT, -PI, PI)
	d.m0 = wrapf(m_at_burn - d.n * T_INTERCEPT, -PI, PI)
	d.name = "2031-XK DEFL"
	ast_defl_el = d

	# Projected miss + equivalent dv for the HUD.
	var sep_au := (pos_ecl(ast_defl_el, T_IMPACT) - pos_ecl(earth_el, T_IMPACT)).length()
	miss_ld = sep_au * AU_KM / LD_KM
	var v_kms := 29.78 / sqrt(a)                    # ~circular-speed scale
	dv_ms = 0.5 * da_over_a * v_kms * 1000.0


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


func _build_events() -> void:
	var ml := "%.1f" % miss_ld
	var raw := [
		[1.0, "TRACKING 2031-XK - EPHEMERIS UPDATED, P(IMPACT)=1.000"],
		[30.0, "DEFLECTION SOLUTION: KINETIC, LEAD %d D, DV %.0f M/S" % [int(T_IMPACT - T_INTERCEPT), dv_ms]],
		[T_LAUNCH - 14.0, "ATLAS-1 ON PAD - LAUNCH WINDOW OPEN"],
		[T_LAUNCH, "ATLAS-1 LAUNCH - TRANSFER INJECTION NOMINAL"],
		[T_LAUNCH + 30.0, "ATLAS-1 CRUISE - GUIDANCE LOCK ON 2031-XK"],
		[T_INTERCEPT, "KINETIC IMPACT CONFIRMED - DV APPLIED ALONG-TRACK"],
		[T_INTERCEPT + 20.0, "POST-BURN SOLUTION: MISS " + ml + " LD - THREAT RETIRED"],
		[T_IMPACT, "NOMINAL IMPACT EPOCH PASSED - EARTH SAFE"],
	]
	for r in raw:
		_events.append({"t": r[0], "msg": r[1], "fired": false})


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


# ------------------------------------------------------------ interceptor ---

func interceptor_phase(t_days: float) -> String:
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
	var el := ast_defl_el if t_days >= T_INTERCEPT else ast_el
	return (pos_ecl(el, t_days) - pos_ecl(earth_el, t_days)).length() * AU_KM


## Jump the mission clock; events at or before the new time are marked
## consumed silently so the console only shows live traffic.
func jump(to_days: float) -> void:
	t = to_days
	for ev in _events:
		ev.fired = ev.t <= t


func jump_next_milestone() -> void:
	for m in [T_LAUNCH - 10.0, T_INTERCEPT - 10.0, T_IMPACT - 20.0, T_IMPACT + 60.0]:
		if t < m - 0.5:
			jump(m)
			event_logged.emit("CLOCK SLEW - MJD-REL %07.1f" % t)
			return
	jump(0.0)
	event_logged.emit("CLOCK SLEW - MISSION START")


func warp_label() -> String:
	var w := WARP_STEPS[warp_idx]
	if w < 1.0:
		return "x%.1f D/S" % w
	return "x%d D/S" % int(w)


func blink(hz: float = 2.0) -> bool:
	return fmod(Time.get_ticks_msec() / 1000.0 * hz, 1.0) < 0.5


## Calendar readout: epoch 2031-01-01 + t days (display only).
func date_string() -> String:
	var days := int(t)
	var date := Time.get_date_dict_from_unix_time(1924992000 + days * 86400)
	return "%04d-%02d-%02d" % [date.year, date.month, date.day]
