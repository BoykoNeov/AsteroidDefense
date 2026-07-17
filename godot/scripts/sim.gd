extends Node
## Central simulation state: the mission clock, time warp, the drawn bodies, and
## the mission timeline.
##
## **The clock is real.** `t` is days from `EPOCH0_TDB`, a genuine TDB instant
## taken from the Rust core's own `ImpactorConfig::default()` — not a fabricated
## 2031 epoch. Every body position is a real DE440 ephemeris lookup through the
## gdext binding (`Mission.body_position_ecl_au`), so what is drawn is what the
## validated core says is there. The clock clamps to the *mounted kernel's*
## coverage (`Mission.usable_span_tdb()`), because outside it every lookup fails.
##
## **The public API is deliberately unchanged**: `pos_ecl(body, t)`,
## `pos3d(body, t)`, `orbit_points(body)` keep their old shapes, so the HUD, map,
## tags and camera did not have to learn where positions come from. What changed
## is what a *body* is: a Dictionary that names its source. An `"ephem"` body
## carries a `naif_id` and is looked up in the real field; a `"kepler"` body
## carries classical elements and is propagated analytically here. That dispatch
## is the whole seam — see `pos_ecl`.
##
## **The mission layer is dormant** (`mission_online == false`) as of 3C-2a. The
## threat, comet, planner, interceptor and b-plane view are switched off rather
## than left running on placeholder Kepler, because they cannot come across
## cleanly yet: their encounter math (`pos_ecl64` -> `geo_km` -> `close_approach`)
## is f64 by design, and real body positions only cross the FFI as f32 Vector3s
## (~18 km of slack at 1 AU — precisely what those helpers exist to avoid, see
## HANDOFF §7). Feeding real Earth into them would quietly drop an f32 floor into
## the b-plane; keeping a private Kepler Earth for them instead would mean two
## Earths, with the threat riding the one nobody can see. Neither is honest. In
## 3C-2b the threat comes from `asteroid_position_ecl_au` and its b-plane from the
## core, so all that math stays inside Rust and only small residuals cross — and
## it lands on the real timeline this clock already runs.

signal event_logged(line: String)

const AU := 10.0                       # Godot units per AU
const AU_KM := 1.495978707e8
const LD_KM := 384400.0                # lunar distance, km
const DAY_S := 86400.0

## Whether the deflection/threat layer is live. False until 3C-2b rebuilds it on
## the real core (see the module note). Consumers check this before drawing or
## reading anything threat-shaped; nothing fakes a number while it is false.
var mission_online := false

## The real DE440 field, via the gdext binding. Null when the extension or the
## kernels are unavailable — `bodies_online` is the flag to check, not this.
var mission = null
var bodies_online := false
var kernel_source := ""                # where the kernels were found (for the HUD)
var kernel_error := ""                 # why they were not (for the HUD)

# Mission timeline (days from EPOCH0_TDB). The impact epoch is fixed by the
# threat; launch/intercept epochs come from the operator's plan ([M]).
var T_IMPACT := 4383.0                 # overwritten from the core at _ready
var T_LAUNCH := 0.0
var T_INTERCEPT := 0.0

## Campaign start, seconds past J2000 — the real TDB instant `t = 0` means.
## Read from the core (`ImpactorConfig::default().epoch0()`, i.e. impact minus
## lead_years) so the drawn timeline cannot drift from the built one. The
## fallback is only for a kernel-less run where nothing is drawn anyway.
var EPOCH0_TDB := 883569600.0          # 2028-01-01 TDB, the core's default

var t := 0.0                           # mission-elapsed time, days
var paused := false
var warp_idx := 3
var time_dir := 1.0                    # +1 forward, -1 reverse (run time backward)
# Selectable warp rates, days/sec — extended into years/sec so the long clock
# (decades, several comet passes) scrubs in seconds without endless key-holding.
const WARP_STEPS: Array[float] = [0.1, 0.5, 2.0, 5.0, 15.0, 45.0, 120.0, 365.0, 1095.0, 3650.0]

# Clock bounds, days from EPOCH0_TDB — set at _ready from the mounted kernel's
# usable span (de440s ~1850-2149, de441 ~1550-2650), never hardcoded. The clock
# clamps here; it never wraps. This clamp is not cosmetic: past the coverage edge
# every lookup fails and a failed lookup returns ZERO, which in this heliocentric
# frame is the SUN's position — so an unclamped clock would not blank the
# display, it would silently collapse every planet onto the Sun.
var T_MIN := -3650.0
var T_MAX := 40000.0

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

	_load_field()
	_build_planets()
	_build_events()


## Bring up the real DE440 field: find the kernels, load them, and adopt the
## core's own campaign epochs and the mounted kernel's coverage window.
##
## Everything here is allowed to fail without taking the app down — a missing
## extension or kernel leaves `bodies_online` false and `kernel_error` set, and
## the HUD says so. What must NOT happen is a silent fallback to fabricated
## bodies: this build draws the real field or admits it cannot.
func _load_field() -> void:
	if not ClassDB.class_exists("Mission"):
		kernel_error = "GDExtension not loaded (build it: cargo build -p asteroid_gdext --release)"
		return
	mission = ClassDB.instantiate("Mission")

	var k := Kernels.resolve()
	if not k.ok:
		kernel_error = k.error
		return
	if not mission.load_from(k.bsp, k.pca):
		kernel_error = "kernel load failed (%s): %s" % [k.source, mission.last_error()]
		return

	kernel_source = k.source
	bodies_online = true

	# Anchor the clock on the core's real campaign, read cheaply — the impact
	# epoch is a config input, not something the expensive build solves for.
	EPOCH0_TDB = mission.default_epoch0_tdb_seconds()
	T_IMPACT = (mission.default_impact_tdb_seconds() - EPOCH0_TDB) / DAY_S

	# Clamp to what the mounted kernel actually serves, not to a guess.
	var span: PackedFloat64Array = mission.usable_span_tdb()
	if span.size() == 2:
		T_MIN = (span[0] - EPOCH0_TDB) / DAY_S
		T_MAX = (span[1] - EPOCH0_TDB) / DAY_S


## Seconds past J2000 for a mission-elapsed time in days — the frame every
## binding call speaks. `t` is a display convenience; this is the real instant.
func tdb(t_days: float = INF) -> float:
	var d := t if is_inf(t_days) else t_days
	return EPOCH0_TDB + d * DAY_S


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
			event_logged.emit(_stamp(ev.t) + "  " + ev.msg)
		ev.fired = passed


## Console timestamp. "E-nnnn" is days-to-impact — meaningful only when there is
## an impact being tracked; with the mission layer dormant it stamped orrery
## messages with a countdown to a threat that is not on screen. Then it is just
## the date.
func _stamp(t_days: float) -> String:
	if mission_online:
		return "E%+05d" % int(t_days - T_IMPACT)
	return date_string(t_days)


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

## The drawn planets, sourced from the real DE440 field by NAIF id.
##
## Two ids are not the obvious ones, and both are pinned by a test in the binding
## (`display_naif_ids_resolve_across_the_whole_usable_span`):
##
##   EARTH is **399**, never 3. Id 3 is the Earth-Moon *barycentre*, ~4671 km
##   from the geocentre — an Earth-radius-scale error, the HANDOFF §5 footgun.
##
##   MARS is **4** (its barycentre), because de440s carries no Mars geocentre
##   segment at all — 499 simply does not resolve. Harmless here, unlike Earth's
##   case: Mars's moons are negligible, so its barycentre sits within a few km of
##   the planet. The outer planets are barycentres for the same reason and are
##   likewise fine at AU display scale.
##
## `a_au` is nominal, used only for display decisions (orbit-line detail, one
## period's worth of sampling) — never for a position, which is always a lookup.
func _build_planets() -> void:
	# [name, NAIF id, nominal a AU, vis radius]
	var raw := [
		["MERCURY", 199, 0.3871, 0.045],
		["VENUS",   299, 0.7233, 0.075],
		["EARTH",   399, 1.0000, 0.080],
		["MARS",      4, 1.5237, 0.060],
		["JUPITER",   5, 5.2026, 0.180],
		["SATURN",    6, 9.5549, 0.150],
		["URANUS",    7, 19.2184, 0.105],
		["NEPTUNE",   8, 30.110, 0.100],
	]
	for r in raw:
		var body := {
			"name": r[0], "source": "ephem", "naif_id": r[1],
			"a": r[2], "vis_r": r[3], "kind": "planet",
		}
		planets.append(body)
		if r[0] == "EARTH":
			earth_el = body


# --- DORMANT until 3C-2b -----------------------------------------------------
# Nothing below is called while `mission_online` is false: _ready() no longer
# builds the threat, the comet or a plan, so `ast_el` / `comet_el` /
# `ast_defl_el` stay empty and every consumer gates on `mission_online`.
#
# Kept rather than deleted because it is the working reference for what 3C-2b has
# to reproduce against the real core (the threat is *constructed from the impact
# condition* so the tracks genuinely converge — that idea survives; only its
# source changes to asteroid_position_ecl_au + a core-computed b-plane).
#
# Why it cannot simply be pointed at real Earth today: the elements below derive
# from `pos_ecl(earth_el, T_IMPACT)`, so against real Earth the threat *would*
# still converge on the drawn planet. The blocker is downstream — `close_approach`
# (via pos_ecl64/geo_km) is deliberately f64, while real positions cross the FFI
# only as f32 Vector3 (~18 km of slack at 1 AU). Wiring real Earth into that chain
# would put an f32 floor under the b-plane and under the capture radius computed
# right here — exactly the error those helpers exist to exclude (HANDOFF §7).

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
##
## While the mission layer is dormant the threat events are NOT scheduled: a
## console announcing "TRACKING 2031-XK - P(IMPACT)=1.000" over a display with no
## threat on it is the loudest lie on the screen, and the event log is the one
## surface a player reads as ground truth.
func _build_events() -> void:
	_events.clear()
	var raw := []
	if mission_online:
		raw = [
			[1.0, "TRACKING 2031-XK - EPHEMERIS UPDATED, P(IMPACT)=1.000"],
			[20.0, "NO DEFLECTION PLAN ON FILE - [M] MISSION PLANNER"],
			[T_IMPACT - 30.0, "FINAL WARNING - IMPACT E-030 D, NO MISSION COMMITTED"],
			[T_IMPACT, "SURFACE IMPACT - NO DEFLECTION ATTEMPTED"],
		]
	elif bodies_online:
		raw = [
			[1.0, "DE440 EPHEMERIS MOUNTED - %d - %d" % [year_at(T_MIN), year_at(T_MAX)]],
			[2.0, "SOLAR FIELD LIVE - %d BODIES - DRAG TIMELINE TO SCRUB" % planets.size()],
			[3.0, "THREAT DB EMPTY - MISSION LAYER REBUILDING ON f64 CORE"],
		]
	else:
		raw = [[1.0, "NO EPHEMERIS KERNEL - SOLAR FIELD OFFLINE"]]
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
##
## The dispatch seam: an `"ephem"` body is a real DE440 lookup through the
## binding; anything else is propagated analytically below. Consumers call this
## exactly as they always did and never learn which happened.
##
## An out-of-coverage or unresolved lookup comes back as ZERO — which in this
## heliocentric frame is *the Sun's position*, not an obviously broken value. The
## clock clamp to `[T_MIN, T_MAX]` is what keeps that from being reachable; the
## binding-side test pins every drawn id across the whole span so it stays that
## way.
func pos_ecl(el: Dictionary, t_days: float) -> Vector3:
	if el.get("source", "") == "ephem":
		if not bodies_online:
			return Vector3.ZERO
		return mission.body_position_ecl_au(el.naif_id, tdb(t_days))
	return _kepler_pos_ecl(el, t_days)


## Analytic two-body position, AU — HANDOFF §5 Tier 0 (cosmetic context orbits,
## never a hit/miss decision). Retained for bodies that have no ephemeris source.
func _kepler_pos_ecl(el: Dictionary, t_days: float) -> Vector3:
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
##
## For a real body this walks one orbital period of the *actual* ephemeris rather
## than drawing an idealised ellipse — so what is drawn is the orbit the core
## flies, wobbles and all. Sampled once at build; planetary orbits do not visibly
## precess over a display session, so it need not follow the clock.
##
## The period comes from the nominal `a` (Kepler's third law) purely to know how
## far to sample; the points themselves are all real lookups.
func orbit_points(el: Dictionary, count: int = 192) -> PackedVector3Array:
	var pts := PackedVector3Array()
	if el.get("source", "") == "ephem":
		if not bodies_online:
			return pts
		var period_d: float = 365.25 * pow(float(el.a), 1.5)
		# Sample from the campaign epoch, and clamp into coverage so a long-period
		# outer planet near a span edge yields a short arc rather than a fan of
		# ZEROs collapsing onto the Sun.
		var t0: float = clampf(0.0, T_MIN, T_MAX)
		var t1: float = clampf(t0 + period_d, T_MIN, T_MAX)
		for k in count + 1:
			var td: float = t0 + (t1 - t0) * float(k) / float(count)
			pts.append(ecl_to_godot(pos_ecl(el, td)))
		return pts

	var saved: float = el.m0
	for k in count + 1:
		var m := TAU * float(k) / float(count)
		el.m0 = m
		pts.append(ecl_to_godot(_kepler_pos_ecl(el, 0.0)))
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


## Unix seconds at the J2000 epoch (2000-01-01T12:00:00). The ~69 s TT-vs-UTC
## offset is knowingly ignored: this drives a YYYY-MM-DD readout, where a minute
## of slop cannot show. Anything that needs real time scales uses hifitime inside
## the core, which is exactly why that lives there and not here.
const J2000_UNIX := 946728000.0


## Calendar readout — a real date now, derived from the real TDB instant rather
## than counting days from a made-up epoch. Defaults to the current clock.
func date_string(t_days: float = INF) -> String:
	var date := Time.get_date_dict_from_unix_time(int(J2000_UNIX + tdb(t_days)))
	return "%04d-%02d-%02d" % [date.year, date.month, date.day]


## Calendar year at a mission time, for axis labels.
func year_at(t_days: float) -> int:
	return Time.get_date_dict_from_unix_time(int(J2000_UNIX + tdb(t_days))).year
