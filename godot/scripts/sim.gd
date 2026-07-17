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
## **The threat is real** as of 3C-2b. `ast_el` is no longer a Kepler ellipse
## fitted to an impact point: it is the core's integrated trajectory through the
## real perturbed field, sampled through `asteroid_position_ecl_au`. The planner
## no longer models its own impulse — `set_plan` hands lead time and Δv to the
## core, which re-propagates and returns a b-plane perigee. All the f64 encounter
## math now lives in Rust, where it always belonged; only positions cross the FFI
## as f32, and only after the core has subtracted (HANDOFF §7).
##
## **The mission layer is not one switch.** Each piece is gated on the real source
## that feeds it, and stays dark until that source exists — see the flags below.
## The threat is online; the comet, the interceptor and the b-plane view are not,
## and the difference is not cosmetic. A single `mission_online` flag would light
## all four at once, and three of them would be lying.

signal event_logged(line: String)

## The real threat is built and drawable. Consumers that draw it build their nodes
## here rather than at `_ready`: the scenario takes ~10 s on a worker thread, so at
## scene load there is genuinely nothing to draw yet.
signal mission_ready

const AU := 10.0                       # Godot units per AU
const AU_KM := 1.495978707e8
const LD_KM := 384400.0                # lunar distance, km
const DAY_S := 86400.0

## Whether the threat and planner are live: true once the core's scenario has
## finished building and installed (see `_poll_build`). Consumers check this
## before drawing or reading anything threat-shaped; nothing fakes a number while
## it is false.
var mission_online := false

## The comet is dormant until it comes from the core's orrery catalog (3C-2c).
## It was a Kepler ellipse; drawing it beside a real integrated threat would put
## two different physics on one screen with nothing marking which is which.
var comet_online := false

## The interceptor is dormant. Its cruise path is a cosmetic bezier with no
## Lambert solver behind it — the one piece of this display that was never
## physics. It stays off until it is, rather than drawing a spacecraft on a
## trajectory no solver produced.
var interceptor_online := false

## The b-plane view is live (3C-2c): `encounter.gd` reads the core's
## `EncounterFrame` — the same propagation the planner's verdict comes from — so
## the geometry on screen and the trajectory behind it are one thing. Set with
## `mission_online`, since the frame is built with the scenario.
##
## It stays a separate flag because the four sources are still separate: the comet
## and the interceptor have no core behind them yet, and lighting them from here
## would be the lie this design exists to prevent.
var encounter_online := false

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

## The window the threat exists over, days from EPOCH0_TDB — read from the core
## (`threat_span_tdb`), never reconstructed here.
##
## This is a *second* coverage gate, and the clock clamp does not do its job. The
## clock is clamped to the mounted kernel (~300 years); the threat is propagated
## over ~12. Outside those 12 years `asteroid_position_ecl_au` fails, and a failed
## lookup returns ZERO — which in this heliocentric frame is **the Sun**. So an
## ungated threat does not disappear when you scrub off its arc: it sits on the
## Sun for ~96% of the timeline. Consumers ask `threat_active()`, never the clock.
var T_THREAT_MIN := 0.0
var T_THREAT_MAX := 0.0

## Scenario build state. The build is ~10 s of real integration on a worker
## thread; the display keeps running (and the planets keep moving) throughout.
enum Build { IDLE, RUNNING, READY, FAILED }
var build_state := Build.IDLE
var build_error := ""

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
## Earth's radius, km — the disc the encounter view draws, and the display divisor
## for "capture = N R_E". Read from the core at `_install_threat` (it is the very
## `earth_radius` the capture disc was computed against), so the display and the
## physics cannot use different Earths. The literal is only a pre-build fallback.
var R_E := 6378.137

var plan_lead_d := 180.0               # intercept lead before impact epoch, days
var plan_dv_ms := 30.0                 # impulse magnitude, m/s
var plan_retro := true                 # true = retrograde (against velocity)
var committed := false                 # launch scheduled
var planner_open := false              # planner panel showing (preview tracks)

## The projected miss, LD — the deflected pass's **b-plane impact parameter**, not
## its perigee. See `_solve_plan` for why that distinction is the verdict.
var miss_ld := 0.0
var dv_ms := 0.0                       # imparted delta-v, m/s (mirrors plan)
var cap_km := 0.0                      # gravitational capture radius, km (from the core)
var deflect_ok := false                # projected |B| clears the capture disc

## A clean miss: the deflected pass left the core's scan gate entirely. This is
## the BEST outcome, and it has no finite perigee — the core reports -1. Never
## read `miss_ld` without checking this first; see `miss_label`.
var plan_clean_miss := false

## A solve is pending or running, so `miss_ld`/`deflect_ok` describe the PREVIOUS
## plan, not the one on screen. Readouts must say so rather than assert a stale
## verdict as current.
var plan_solving := false

## Debounce for the core solve. Each solve re-propagates the post-impulse arc
## (~0.9 s, down from ~11 s before the core's nominal cache) — fast enough to feel
## live, far too slow to run per keypress while an operator holds an arrow key.
## So edits land instantly and the solve is coalesced to the end of the burst.
const PLAN_DEBOUNCE_S := 0.35
var _plan_dirty := false
var _plan_timer := 0.0

signal plan_changed

var _events: Array[Dictionary] = []


func _ready() -> void:
	mono_font = SystemFont.new()
	mono_font.font_names = PackedStringArray(
		["Consolas", "Cascadia Mono", "Courier New", "Lucida Console"])

	_load_field()
	_build_planets()
	_build_events()
	_begin_build()


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
	# Both of these run while paused: a paused clock does not mean a paused build,
	# and an operator who pauses mid-edit still wants their verdict solved.
	_poll_build()
	_tick_plan_debounce(delta)

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


# ------------------------------------------------------------ the threat ---

## Start the scenario build on a worker thread.
##
## This is ~10 s of real integration through the perturbed field. It is threaded
## because the alternative is 10 s of frozen display — and since 3C-2a that
## display is a *working* one, drawing real planets on a real clock. Freezing it
## to build the threat would break the thing that already works to add the thing
## that doesn't yet.
func _begin_build() -> void:
	if not bodies_online:
		return
	if not mission.begin_build_scenario():
		build_state = Build.FAILED
		build_error = mission.last_error()
		return
	build_state = Build.RUNNING


## Drain the build. `poll_build()` is true while the worker is still running; it
## installs the scenario into the core on the frame it lands.
func _poll_build() -> void:
	if build_state != Build.RUNNING:
		return
	if mission.poll_build():
		return
	if not mission.is_ready():
		build_state = Build.FAILED
		build_error = mission.last_error()
		event_logged.emit(_stamp(t) + "  THREAT SOLUTION FAILED - " + build_error)
		return
	build_state = Build.READY
	_install_threat()


## Adopt the built scenario: the threat becomes drawable and the planner opens.
##
## `ast_el` / `ast_defl_el` stay Dictionaries with the same shape every consumer
## already reads — only `source` changes, and `pos_ecl` / `orbit_points` dispatch
## on it. Nothing downstream learns that the ellipse became an integration.
func _install_threat() -> void:
	ast_el = {
		"name": "2031-XK", "source": "threat", "kind": "asteroid",
		"a": mission.semi_major_axis_m() / (AU_KM * 1000.0), "vis_r": 0.030,
	}
	ast_defl_el = {
		"name": "2031-XK DEFL", "source": "threat_defl", "kind": "asteroid",
		"a": ast_el.a, "vis_r": ast_el.vis_r,
	}

	# The window the threat exists over — the ZERO-is-the-Sun gate (see
	# T_THREAT_MIN). From the core, so it cannot drift from what a lookup answers.
	var s: PackedFloat64Array = mission.threat_span_tdb()
	if s.size() == 2:
		T_THREAT_MIN = (s[0] - EPOCH0_TDB) / DAY_S
		T_THREAT_MAX = (s[1] - EPOCH0_TDB) / DAY_S

	# The capture radius: the bar the verdict is measured against, and the real
	# one — Earth's focusing widens it to ~1.77 R_E at this encounter speed, so a
	# "miss" inside it is a hit that Earth reels in. It is the bar for the *impact
	# parameter* specifically (see `_solve_plan`), and R_E is the Earth it was
	# computed against — both read from the core rather than kept in step by hand.
	cap_km = mission.capture_radius_m() / 1000.0
	R_E = mission.earth_radius_m() / 1000.0

	mission_online = true
	# The b-plane frame is built with the scenario, so the close-up is live the
	# moment the threat is. The comet and interceptor flags stay dark: still no core.
	encounter_online = true
	_build_events()
	mission_ready.emit()
	event_logged.emit(_stamp(t) + "  THREAT SOLUTION ACQUIRED - 2031-XK TRACKING")


## Whether the threat exists at a mission time. False outside the propagated span
## — where a lookup would return ZERO and draw the asteroid on the Sun. Every
## consumer that draws the threat asks this first.
func threat_active(t_days: float = INF) -> bool:
	if not mission_online:
		return false
	var d := t if is_inf(t_days) else t_days
	return d >= T_THREAT_MIN and d <= T_THREAT_MAX


## The threat's tracked arc as dates, for a readout that has to explain why there
## is nothing to show at the current clock.
func threat_arc_label() -> String:
	if not mission_online:
		return "--"
	return "%s .. %s" % [date_string(T_THREAT_MIN), date_string(T_THREAT_MAX)]


## The threat's heliocentric period, days. The core's figure (vis-viva on the
## integrated seed), not a mean motion this layer keeps its own copy of.
func threat_period_d() -> float:
	if not mission_online:
		return 0.0
	return mission.period_seconds() / DAY_S


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
## The threat events are scheduled only once the threat is real. The event log is
## the one surface a player reads as ground truth, so a console announcing
## "TRACKING 2031-XK - P(IMPACT)=1.000" over a display with no threat on it would
## be the loudest lie on the screen.
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
			[3.0, "INTEGRATING THREAT TRAJECTORY - REAL FIELD, STAND BY"],
		]
	else:
		raw = [[1.0, "NO EPHEMERIS KERNEL - SOLAR FIELD OFFLINE"]]
	for r in raw:
		_events.append({"t": r[0], "msg": r[1], "fired": r[0] <= t})


## Committed-mission timeline; outcome events follow the projected verdict.
## The miss goes through `miss_label` like every other readout — a clean miss has
## no number to print here either.
func _rebuild_events() -> void:
	_events.clear()
	var ml := miss_label()
	var raw := [
		[1.0, "TRACKING 2031-XK - EPHEMERIS UPDATED, P(IMPACT)=1.000"],
		[T_LAUNCH - 14.0, "ATLAS-1 ON PAD - LAUNCH WINDOW OPEN"],
		[T_LAUNCH, "ATLAS-1 LAUNCH - TRANSFER INJECTION NOMINAL"],
		[minf(T_LAUNCH + 30.0, T_INTERCEPT - 5.0), "ATLAS-1 CRUISE - GUIDANCE LOCK ON 2031-XK"],
		[T_INTERCEPT, "KINETIC IMPACT CONFIRMED - DV %.1f M/S %s" %
			[plan_dv_ms, "RETROGRADE" if plan_retro else "PROGRADE"]],
	]
	if deflect_ok:
		raw.append([T_INTERCEPT + 20.0, "POST-BURN SOLUTION: MISS " + ml + " - THREAT RETIRED"])
		raw.append([T_IMPACT, "NOMINAL IMPACT EPOCH PASSED - EARTH SAFE"])
	else:
		raw.append([T_INTERCEPT + 20.0, "POST-BURN SOLUTION: MISS " + ml + " - INSUFFICIENT"])
		raw.append([T_IMPACT, "SURFACE IMPACT - DEFLECTION FAILED"])
	for r in raw:
		_events.append({"t": r[0], "msg": r[1], "fired": r[0] <= t})


# --------------------------------------------------------------- mission plan ---
# The planner edits (lead, dv, direction); the core does the physics. This layer
# marshals a plan in and a verdict out, and owns no orbital mechanics at all.


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


## Apply a mission plan. The edit lands now; the physics is debounced.
##
## The core owns the deflection: `_solve_plan` hands it a lead time and a signed
## along-track impulse, and it re-propagates the post-impulse arc through the real
## perturbed field and reduces the encounter to a b-plane perigee. This function
## deliberately computes no orbital mechanics — the chain it used to run
## (elements_from_rv -> close_approach) is gone, not ported.
func set_plan(lead_d: float, dv: float, retro: bool) -> void:
	plan_lead_d = clampf(lead_d, LEAD_MIN, maxf(lead_cap(), LEAD_MIN))
	plan_dv_ms = clampf(dv, DV_MIN, DV_MAX)
	plan_retro = retro
	T_INTERCEPT = T_IMPACT - plan_lead_d
	T_LAUNCH = T_INTERCEPT - cruise_d()
	dv_ms = plan_dv_ms

	# The numbers under the operator's fingers move immediately; the verdict
	# follows the solve. `plan_solving` is what stops the panel presenting the
	# previous plan's verdict as this one's during the gap.
	_plan_dirty = true
	_plan_timer = PLAN_DEBOUNCE_S
	plan_solving = true
	plan_changed.emit()


## Coalesce a burst of plan edits into one solve. An operator holding an arrow key
## emits an edit per frame; each solve is ~0.9 s of integration, so solving per
## edit would queue minutes of work to answer a question already superseded.
func _tick_plan_debounce(delta: float) -> void:
	if not _plan_dirty:
		return
	_plan_timer -= delta
	if _plan_timer <= 0.0:
		_solve_plan()


## Hand the plan to the core and read the verdict back. Blocks for ~0.9 s — this
## is the hitch the debounce exists to ration.
func _solve_plan() -> void:
	_plan_dirty = false
	plan_solving = false
	if not mission_online:
		return

	# Retrograde is a NEGATIVE along-track impulse. Not a convention chosen here:
	# the core applies `dv * along_track_unit(state)`, and that unit vector is
	# prograde by construction, so the sign is the direction.
	var dv_signed := plan_dv_ms * (-1.0 if plan_retro else 1.0)
	if not mission.set_plan(plan_lead_d * DAY_S, dv_signed):
		plan_clean_miss = false
		miss_ld = 0.0
		deflect_ok = false
		event_logged.emit(_stamp(t) + "  PLAN SOLVE FAILED - " + str(mission.last_error()))
		plan_changed.emit()
		return

	plan_clean_miss = mission.is_clean_miss()
	# The **impact parameter**, not the perigee — see below.
	var b_km: float = mission.deflected_impact_parameter_m() / 1000.0
	miss_ld = b_km / LD_KM

	# THE verdict, and the one place it is decided.
	#
	# Safe is `b > cap_km`: the b-plane impact parameter against the focused
	# capture radius. That is the core's own `is_hit`, and the pairing matters.
	# There are two coherent criteria and they are equivalent:
	#
	#     b > b_capture        the UN-focused asymptotic miss, against a target
	#                          enlarged to account for focusing
	#     perigee > R_E        the ALREADY-focused closest approach, against
	#                          Earth's actual solid body
	#
	# This used to read `perigee_km > cap_km`, which is neither pair: it charges
	# for gravitational focusing twice and demands ~1.5x more miss than physics
	# does. Measured on a plan a player can dial in (0.2 m/s, one period of lead):
	# b = 14,640 km clears the 11,311 km disc — a miss, by 2,941 km of real
	# daylight — while its perigee of 9,319 km sits inside that disc, so the old
	# test printed SURFACE IMPACT over a deflection that works. The two numbers are
	# both "miss distances" in km, which is exactly why the mix-up survived.
	#
	# The clean-miss check still comes first, for the older trap: a clean miss
	# reports -1, so the *success* case shares "no plan"'s sentinel, and a bare
	# `b_km > cap_km` would read the best possible outcome as a catastrophic
	# failure at a negative miss distance.
	deflect_ok = plan_clean_miss or b_km > cap_km
	if committed:
		_rebuild_events()
	plan_changed.emit()


## Whether the core holds a solved plan. Not the same as "the operator opened the
## planner": the deflected track does not exist until the core has propagated it,
## and sampling it before then is what draws a body on the Sun.
func has_plan() -> bool:
	return mission_online and mission.has_plan()


## The projected miss, formatted — the single place `miss_ld` becomes text.
## `with_km` adds the grouped kilometre figure for the planner's wide column.
##
## This is the impact parameter, which is what makes it comparable to `cap_km`
## printed beside it (see `_solve_plan`): a player reads those two numbers against
## each other, so they must be the pair the verdict actually compares.
##
## Three panels print this. A clean miss carries no finite |B| (the core reports
## -1), so it must never reach a "%.2f LD": centralising the formatting is what
## stops one of the three sites quietly printing "-0.01 LD" as a real miss.
func miss_label(with_km: bool = false) -> String:
	if plan_solving:
		return "SOLVING..."
	if not has_plan():
		return "NO SOLUTION"
	if plan_clean_miss:
		return ">> OFF-SCALE (LEFT THE ENCOUNTER)"
	var s := "%.2f LD" % miss_ld
	if with_km:
		s += "  (%s KM)" % group_num(int(miss_ld * LD_KM))
	return s


## Thousands-separated integer, for readouts where a raw 1234567 is unreadable.
func group_num(v: int) -> String:
	var s := str(v)
	var out := ""
	while s.length() > 3:
		out = "," + s.right(3) + out
		s = s.left(s.length() - 3)
	return s + out


## The verdict, formatted. Same contract as `miss_label`: clean miss first.
func verdict_label() -> String:
	if plan_solving:
		return "SOLVING..."
	if not has_plan():
		return "NO SOLUTION ON FILE"
	if plan_clean_miss:
		return "CLEAN MISS - THREAT RETIRED"
	if deflect_ok:
		return "MISS - EARTH CLEAR"
	return "SURFACE IMPACT - INSUFFICIENT"


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


## First-order estimate of the Δv needed for a 1.0 LD miss at this lead (b-plane
## displacement is ~linear in Δv; nominal b is ~0 by construction), formatted.
##
## An *estimate* on purpose. The core can solve this exactly — `required_dv`
## brackets and bisects on the real perigee — but takes ~18 s to do it, which is
## not a readout that can sit next to a live planner. So this stays a labelled
## first-order guess rather than a number pretending to be the solve.
func req_dv_label() -> String:
	if plan_solving or not has_plan():
		return "--"
	# A clean miss is already past 1 LD by an unmeasured margin, and its -1
	# perigee would divide into a garbage requirement.
	if plan_clean_miss:
		return "ACHIEVED"
	var req := plan_dv_ms / maxf(miss_ld, 1.0e-4)
	return ">999 M/S" if req > 999.0 else "%.1f M/S" % req


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
	match el.get("source", ""):
		"ephem":
			if not bodies_online:
				return Vector3.ZERO
			return mission.body_position_ecl_au(el.naif_id, tdb(t_days))
		"threat":
			if not threat_active(t_days):
				return Vector3.ZERO
			return mission.asteroid_position_ecl_au(tdb(t_days))
		"threat_defl":
			# No plan means no deflected arc to sample — not a zero-length one.
			if not has_plan() or not threat_active(t_days):
				return Vector3.ZERO
			return mission.deflected_position_ecl_au(tdb(t_days))
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

	# The threat's track is the core's own integration, sampled span-wide by the
	# binding — not one period of an ellipse. It is an open arc from campaign start
	# to impact, not a closed orbit, which is the point: that arc ends on Earth.
	var src: String = el.get("source", "")
	if src == "threat" or src == "threat_defl":
		if not mission_online:
			return pts
		if src == "threat_defl" and not has_plan():
			return pts
		var track: PackedVector3Array = mission.asteroid_track_ecl_au(count) \
			if src == "threat" else mission.deflected_track_ecl_au(count)
		for p in track:
			pts.append(ecl_to_godot(p))
		return pts

	if src == "ephem":
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


# ---------------------------------------------- encounter (the b-plane view) ---
# The close-up reads the core's `EncounterFrame` through here. As everywhere else,
# this layer marshals and owns no geometry: points arrive already projected into
# the core's b-plane display frame — `(xi, zeta, s)` km, `s` being depth along the
# incoming asymptote — because the asymptote lives in the core and choosing the
# frame is the only judgement involved.


## An encounter track as `(xi, zeta, s)` km per sample, uniformly spaced over
## `encounter_span_days()`.
##
## The nominal exists the moment the threat does — it is the incoming impact, and
## it needs no plan. The deflected one is **empty** until the core has solved a
## plan: empty, not zero-length, because a zeroed track would draw the asteroid
## straight through Earth's centre and call it a deflection.
func encounter_track(deflected: bool) -> PackedVector3Array:
	if not encounter_online:
		return PackedVector3Array()
	return mission.encounter_deflected_track_km() if deflected \
		else mission.encounter_nominal_track_km()


## Where a pass's incoming asymptote pierces the b-plane — `(xi, zeta, s)` km, at
## distance |B| from Earth's centre. **This is the point the verdict is about.**
##
## `Vector3.ZERO` means there is no such point (no plan, or a clean miss that left
## the encounter). ZERO is Earth's dead centre in this frame — a perfect hit — so
## callers must check rather than draw it.
func encounter_b_point(deflected: bool) -> Vector3:
	if not encounter_online:
		return Vector3.ZERO
	if deflected and (not has_plan() or plan_clean_miss):
		return Vector3.ZERO
	return mission.deflected_b_point_km() if deflected else mission.nominal_b_point_km()


## The encounter window as `[first, last]` mission days — the arc the tracks cover
## (the core's ±1.5 d around impact). Empty when the mission layer is dormant.
func encounter_span_days() -> PackedFloat64Array:
	var out := PackedFloat64Array()
	if not encounter_online:
		return out
	var s: PackedFloat64Array = mission.encounter_sample_span_tdb()
	if s.size() == 2:
		out.push_back((s[0] - EPOCH0_TDB) / DAY_S)
		out.push_back((s[1] - EPOCH0_TDB) / DAY_S)
	return out


## The encounter's hyperbolic excess speed, km/s. Not the 18 km/s the config names
## — that is the speed at the impact point, deep in Earth's well; stripped of the
## well it is ~7.63 km/s, and that is what sets the capture disc at 1.77 R_E.
func encounter_v_inf_kms() -> float:
	if not encounter_online:
		return 0.0
	return mission.encounter_v_inf_m_s() / 1000.0


## The nominal pass's |B|, LD — the hit being undone, inside the capture disc by
## construction. (The deflected pass's |B| is `miss_ld`; see `miss_label`.)
func nominal_b_ld() -> float:
	if not encounter_online:
		return 0.0
	return mission.nominal_impact_parameter_m() / 1000.0 / LD_KM


## A pass's actual closest approach to Earth's centre, LD — reported alongside |B|
## because "how close did it really come" is a fair question, but it is **not** the
## verdict: the perigee is already focused, so it pairs with R_E, never with
## `cap_km`. Negative when there is no such pass. See `_solve_plan`.
func perigee_ld(deflected: bool) -> float:
	if not encounter_online:
		return -1.0
	var m: float = mission.deflected_perigee_m() if deflected else mission.nominal_perigee_m()
	return -1.0 if m < 0.0 else m / 1000.0 / LD_KM


# ------------------------------------------------- encounter geometry (f64) ---
# DELETED in 3C-2b, deliberately not ported.
#
# `pos_ecl64` / `geo_km` / `geo_vel_kms` / `close_approach` / `elements_from_rv`
# existed to keep the encounter in doubles while GDScript's Vector3 truncates to
# f32 (~18 km of slack at 1 AU, HANDOFF §7). That was the right call for a
# placeholder that had to do its own physics. It is the wrong call now: the core
# does this properly — a real close-approach root-find on dense output, and a
# b-plane reduction with gravitational focusing that a ternary search on range
# cannot express.
#
# Keeping them "for reference" would mean two encounter pipelines that must agree
# and cannot be checked against each other, which is how a display quietly starts
# disagreeing with its own physics. The core is the reference. GDScript gets
# thinner: it marshals a plan in and a verdict out.
#
# The f32 boundary is still real and still respected — the core subtracts in f64
# and only the small geocentric residual crosses (see `Mission::set_plan` and the
# 3C-2c `EncounterFrame` work).


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

## Range Earth <-> active asteroid track, km. Negative when the threat does not
## exist at this time — callers must not print a range to a body that is not
## there (an ungated call would return the distance to the SUN, ~1 AU, and look
## entirely plausible).
##
## Display-grade only: this is an f32 difference of two ~1 AU vectors, so it
## carries ~18 km of slack (HANDOFF §7). Fine for a "RANGE 12,450,000 KM" readout,
## never for a hit/miss call — that is `deflect_ok`, from the core's b-plane.
func threat_range_km(t_days: float) -> float:
	if not threat_active(t_days):
		return -1.0
	# Decided by the *queried* time, not the clock: this answers "where was it at
	# t_days", and a caller asking about a past epoch must not get the track chosen
	# by where the clock happens to sit now.
	var el := ast_defl_el if (committed and has_plan() and t_days >= T_INTERCEPT) else ast_el
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
