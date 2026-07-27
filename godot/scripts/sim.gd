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

## The comet is live (3D): it comes from the core's orrery catalog, integrated on
## the build worker in the same validated Tier-1 field as the threat. It was a
## Kepler ellipse authored in this file, which put two different physics on one
## screen with nothing marking which is which.
##
## Set from what the catalog actually holds after the build lands (see
## `_install_catalog`) — never alongside `mission_online`, because the comet is a
## separate body that can fail to fly on its own. It is *synthetic and labelled as
## such*: a designed orbit, honestly propagated, not a real object.
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
## A small-body kernel was found and handed to the core. NOT the same as mounted:
## the mount happens on the build worker, and `Mission.small_bodies_mounted()` is
## the only flag an asteroid draw may gate on. This one is for the HUD, so a
## machine without the 646 MB file can say so instead of showing nothing.
var small_bodies_armed := false
## The main-belt asteroids from sb441, as `"ephem"` bodies. Empty until a build
## lands with the kernel mounted — see `_install_asteroids`.
var asteroids: Array = []
## The real near-Earth asteroids from JPL Horizons state tables, as `"catalog"`
## bodies. Empty until a build lands with tables on disk — see `_install_catalog`.
##
## **A third provenance on one screen.** The belt asteroids above are `"ephem"`:
## a kernel contains them and ANISE reads them. The comet is `"catalog"` and
## integrated: our physics, our field. These are `"catalog"` and *sampled*: JPL's
## trajectory, interpolated between JPL's own states, because Horizons ships these
## objects as SPK type 21 and ANISE cannot evaluate it. Each carries a
## `provenance` field from the core rather than this layer assuming one — the
## standing rule being that nothing is drawn beside real physics unlabelled.
var neos: Array = []

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

## Tier-2 physics menu ([P]): the five force terms whose b-plane shift the core
## measured once at build time (`Mission.has_tier2_preview`). Each entry is an
## on/off toggle — flipped by [G]/[Y]/[A]/[S]/[O] — and when ON the panel reveals
## that term's isolated perigee shift. Off by default so switching a term on is the
## act that *shows* its contribution (the "toggle to show the shift" the panel is
## for). The numbers are precomputed and fixed per scenario, so a toggle reads
## instantly. The five terms in panel order: [menu key, core id, display name]. The
## id is the string the core keys `tier2_shifted_perigee_m` on, so this table is the
## single place the frontend names them.
##
## `J2` carries a `*` because it is the one term whose *validity* depends on the
## geometry it was measured on: the shipping nominal is a designed impact 3000 km
## from Earth's centre, inside the body, and the J2 expansion holds only outside
## `R_eq`. It is measured on that seed anyway — every shift here is subtracted from
## the same nominal baseline, so measuring one term somewhere else would print a
## difference between two unrelated geometries. The in-domain companion reaches the
## panel's footnote through `j2_miss_shift_km` instead (see `Tier2Panel`).
const TIER2_TERMS := [
	["G", "relativity", "GR (1PN RELATIVITY)"],
	["Y", "yarkovsky", "YARKOVSKY (A2)"],
	["A", "belt", "MAIN-BELT (16x SB441)"],
	["S", "srp", "SOLAR RAD. PRESSURE"],
	["O", "j2", "EARTH J2 (OBLATENESS) *"],
]
var tier2_ready := false                # the core has measured the five shifts
var tier2_measuring := false            # the on-demand ~2 min measurement is running
var tier2_panel_open := false           # the physics panel is showing
## Per-term reveal state, keyed by core id — **populated from `TIER2_TERMS` in
## `_ready`**, never written out a second time. `toggle_tier2` ignores an unknown
## key, so a term listed in the table but missing here would silently do nothing
## when its key was pressed; deriving the dict is what makes that impossible.
var tier2_on := {}
## The nominal (un-deflected) b-plane perigee, km — the baseline every Tier-2 shift
## is measured against (`shift = nominal − shifted`). Read from the core at
## `_install_threat`, same source as `cap_km`, so the menu and the encounter view
## quote one perigee.
var nom_perigee_km := 0.0

## Porkchop / deliverability layer ([4], HANDOFF §8). The launch-window map: for
## every (launch, arrival) pair, the transfer that reaches the asteroid and what a
## real launcher can put on it. Built on demand from the core's `PorkchopView`,
## once per scenario, on a worker — ~45 us/cell, so this grid is ~0.6 s of work
## that must never touch the render thread.
##
## `pork_online` is **its own flag**, set from `Mission.has_porkchop()` and not
## alongside `mission_online`. A threat solution does not imply a grid: the two
## land seconds apart, and a view that gated on the wrong one would draw an empty
## heatmap as if it were a measured result.
const PORK_LAUNCH_SAMPLES := 120
const PORK_ARRIVAL_SAMPLES := 120

## Heatmap metrics, cycled by [D]. Each is [id, label, unit, legend title] — the
## short title exists because the colour key is a narrow column and a truncated
## label there ("LAUNCH ENERG") reads as a rendering fault.
const PORK_METRICS := [
	["c3", "LAUNCH ENERGY C3", "KM2/S2", "C3"],
	["dv", "DELIVERED ALONG-TRACK DV", "MM/S", "DELTA-V"],
]

var pork_online := false               # a built grid is readable
var pork_building := false             # the ~0.6 s worker is running
var pork_rows := 0                     # launch epochs
var pork_cols := 0                     # arrival epochs
var pork_launch_tdb := PackedFloat64Array()
var pork_arrival_tdb := PackedFloat64Array()
## Departure C3 per cell, km^2/s^2 — **-1 marks a cell with no transfer at all**.
## The single authority on emptiness; the other columns carry ordinary zeros in
## blank cells, and a third state (a real transfer this launcher cannot reach,
## `pork_payload == 0`) must stay distinct from both.
var pork_c3 := PackedFloat64Array()
var pork_along := PackedFloat64Array()   # signed along-track projection, m/s
var pork_revs := PackedInt32Array()      # complete solar laps; -1 where blank
var pork_payload := PackedFloat64Array() # deliverable mass for pork_vehicle, kg
var pork_dv := PackedFloat64Array()      # delivered along-track dv, m/s (signed)
var pork_vehicle := 0                    # index into the core's launcher table
var pork_metric := 0                     # index into PORK_METRICS
var pork_i := 0                          # cursor: launch index
var pork_j := 0                          # cursor: arrival index
var pork_verifying := false              # the on-demand full-field verify is running
## The on-demand required-impactor-mass solve is running. Its own flag beside
## `pork_verifying` because they are separate workers answering separate questions,
## and the panel shows both lines at once — one running must not blank the other.
var pork_mass_solving := false

signal porkchop_changed

## ---------------------------------------------------------------- tractor ---
##
## The gravity-tractor bench: six knobs and a live scoring of what they buy.
##
## **The knobs are a table, not a set of variables**, and that is the whole design.
## The planner spends a dedicated action pair per parameter (`plan_lead_up` /
## `plan_lead_down`, `plan_dv_up` / `plan_dv_down`, …), which is fine at two
## parameters and is twelve input actions and twelve `main.gd` branches at six.
## Here UP/DOWN picks a row and LEFT/RIGHT adjusts it — the porkchop's cursor
## idiom — so **adding a parameter is one row in `TRACTOR_KNOBS` and nothing
## else**. No input action, no key, no branch, no plumbing.
##
## Each row is [id, label, unit, min, max, step_factor, is_multiplicative]. A
## multiplicative knob steps by ×/÷ (mass and hover distance span decades and are
## unusable on a linear step); an additive one steps by ±.
const TRACTOR_KNOBS := [
	["mass", "SPACECRAFT MASS", "T", 1.0, 4000.0, 1.5, true],
	# The hover minimum here is a PLACEHOLDER and is overridden from the core in
	# `_seed_tractor_defaults`. The real floor is `1/cos(plume)` ~ 1.064 radii, not
	# the surface at 1.0: between them the spacecraft is outside the body, tows
	# perfectly well, and has no station-keeping solution at all. A hand-written
	# bound "just above the surface" lands squarely inside that band.
	["hover", "HOVER DISTANCE", "R", 1.0, 8.0, 1.15, true],
	["radius", "ROCK RADIUS", "M", 15.0, 800.0, 1.25, true],
	["lead", "TOW START", "ORB", 0.25, 11.5, 0.25, false],
	["duty", "TOW DURATION", "PCT", 0.0, 100.0, 5.0, false],
	["dir", "TOW DIRECTION", "", 0.0, 1.0, 1.0, false],
]

## Live knob values, keyed by the ids above. Seeded from the core in `_ready` so
## the panel opens on the configuration the campaign actually measured (Lu &
## Love's 20 t at d/r = 1.5 over this threat) rather than on numbers restated
## here — the frontend names a source for this exactly as it does for a drawn body.
var tractor := {
	"mass": 20.0,        # tonnes
	"hover": 1.5,        # body radii  (overwritten from the core)
	"radius": 150.0,     # m           (overwritten from the core)
	"lead": 8.0,         # orbital periods — the campaign's cheapest lead
	"duty": 100.0,       # percent of the lead spent towing
	"dir": 0.0,          # 0 prograde, 1 retrograde
}
var tractor_row := 0                   # which knob the cursor is on
var tractor_panel_open := false
var tractor_probing := false           # the on-demand ~12 s full-field probe is running
## The lead below which the required-Δv law does not hold, in orbital periods —
## read from the core, never assumed. The panel refuses to print a requirement
## below it rather than printing one that is the wrong shape.
var tractor_law_min_periods := 1.0
var tractor_target_perigee_m := 0.0
## The closest hover the plume geometry permits, in body radii — read from the
## core, never assumed. See the note on the `hover` row of `TRACTOR_KNOBS`.
var tractor_hover_min := 1.0

signal tractor_changed

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

	for term: Array in TIER2_TERMS:
		tier2_on[term[1]] = false

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

	# Arm the small-body kernel if this machine has one. Records a path only — the
	# ~5.7 s mount happens on the build worker, so the load stays fast and the
	# asteroids appear when the build lands. Absent (or unreadable) is fine: the
	# mission is complete without them, so this warns rather than failing the load.
	if not k.small_bodies.is_empty():
		if mission.set_small_body_kernel(k.small_bodies):
			small_bodies_armed = true
		else:
			push_warning("small-body kernel not armed: %s" % mission.last_error())

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
	# These all run while paused: a paused clock does not mean a paused build, an
	# operator who pauses mid-edit still wants their verdict solved, and the Tier-2
	# measurement (kicked from the menu) must land whatever the clock is doing.
	_poll_build()
	_poll_tier2_preview()
	_poll_porkchop()
	_poll_cell_verify()
	_poll_required_mass()
	_poll_tow_probe()
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
	nom_perigee_km = mission.nominal_perigee_m() / 1000.0
	# The Tier-2 shifts are NOT measured with the build — that ~64 s would delay this
	# very threat solution. They are measured on demand when the operator opens the
	# force-model menu (`request_tier2_preview`), so `tier2_ready` starts false.
	tier2_ready = false
	tier2_measuring = false

	mission_online = true
	# The tractor bench opens on the core's own configuration, not on numbers
	# restated in GDScript — seeded here because the shipping hover distance is a
	# core constant and this is the first moment it is readable.
	_seed_tractor_defaults()
	# The b-plane frame is built with the scenario, so the close-up is live the
	# moment the threat is.
	encounter_online = true
	# The comet rode the same worker, in the same field — so it lands here too, and
	# `comet_online` is set by what the catalog actually contains, not by assuming
	# the worker did what it was asked. The interceptor flag stays dark: still no
	# Lambert solver behind it.
	_install_catalog()
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


## Adopt the orrery catalog the build worker flew alongside the threat.
##
## The comet used to be a Kepler ellipse authored here — the last piece of orbital
## mechanics in GDScript, and the same mistake the threat's f64 block was: two
## different physics on one screen with nothing marking which is which. Now it is
## the core's integration in the core's validated field, and this layer only names
## an index and a colour.
func _install_catalog() -> void:
	_install_asteroids()
	comet_el = {}
	comet_online = false
	neos.clear()
	for i in mission.catalog_count():
		var s: PackedFloat64Array = mission.catalog_span_tdb(i)
		if s.size() != 2:
			continue
		var el := {
			"name": mission.catalog_name(i), "source": "catalog",
			"kind": mission.catalog_kind(i), "catalog_index": i,
			# Named by the core, not inferred here: "integrated" is our physics in
			# our field, "sampled" is JPL's read from a Horizons table. The HUD
			# shows it, because two provenances drawn identically is the mistake
			# the deleted GDScript Kepler was.
			"provenance": mission.catalog_provenance(i),
			# Days from EPOCH0_TDB — the ZERO-is-the-Sun gate, per body, read from
			# the core rather than reconstructed from the span we asked for.
			"t_min": (s[0] - EPOCH0_TDB) / DAY_S,
			"t_max": (s[1] - EPOCH0_TDB) / DAY_S,
		}
		match el.kind:
			"comet":
				if comet_online:
					continue  # one comet is scenery; several would be a catalog bug
				el["vis_r"] = 0.040
				comet_el = el
				comet_online = true
			"asteroid":
				# Smaller than the comet and than the belt blobs: these are the
				# real NEOs, and they are physically the smallest things drawn.
				el["vis_r"] = 0.032
				neos.append(el)
			_:
				push_error("_install_catalog: catalog body '%s' has unknown kind '%s'"
					% [el.name, el.kind])


## Adopt the main-belt asteroids the build worker's mount made reachable.
##
## These are `"ephem"` bodies, not `"catalog"` ones — the same read path as the
## planets, because `sb441-n16.bsp` *contains* their trajectories. Nothing is
## integrated for them and nothing here approximates them; the distinction is the
## whole reason this is a kernel mount rather than sixteen more synthetic bodies.
##
## Gated on `small_bodies_mounted()`, which is the served core's answer, not on
## `small_bodies_armed`, which only says a path was handed over. Between those two
## states every lookup here fails — and a failed ephem lookup drawn anyway is a
## body sitting on the Sun, the failure this project has shipped three times.
func _install_asteroids() -> void:
	asteroids.clear()
	if not bodies_online or not mission.small_bodies_mounted():
		return
	for i in mission.small_body_count():
		asteroids.append({
			"name": mission.small_body_name(i),
			"source": "ephem",
			"naif_id": mission.small_body_id(i),
			# Nominal only — display decisions (orbit-line sampling), never a
			# position. Main belt to within what a 0.5 AU-wide ring needs.
			# vis_r has to beat the scenery: the belt's dust points are ~1 px and
			# these must read as bodies, not as brighter dust. Sized between the
			# comet (0.040) and a small planet.
			"a": 2.7, "vis_r": 0.045, "kind": "asteroid",
		})


## Whether a catalog body exists at a mission time — the per-body twin of
## `threat_active`, and mandatory for the same reason: outside its propagated span
## `catalog_position_ecl_au` returns ZERO, and ZERO here is the Sun. A comet that
## silently parks on the Sun for the two thirds of the clock it does not cover is
## exactly the failure this gate exists to prevent.
## Gates **per body**, not on a global flag. It used to require `comet_online`,
## which was correct while the catalog held exactly one body and became wrong the
## moment it held four: Apophis's table and the comet's arc cover different years,
## and one flag cannot answer for both. An offline body has no `source` (its
## dictionary is empty), so it still answers false here.
func catalog_active(el: Dictionary, t_days: float = INF) -> bool:
	if el.get("source", "") != "catalog":
		return false
	var d := t if is_inf(t_days) else t_days
	return d >= float(el.t_min) and d <= float(el.t_max)


## The comet's tracked arc as dates, for a readout that has to explain why there is
## nothing to show at the current clock.
func comet_arc_label() -> String:
	if not comet_online:
		return "--"
	return "%s .. %s" % [date_string(comet_el.t_min), date_string(comet_el.t_max)]


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


## Kick off the on-demand Tier-2 shift measurement — the ~2 min (five ~16 s
## propagations) that fills the force-model menu. Called when the panel opens.
## Off the build critical path by design: the threat solution never waits on it.
## A no-op if the threat is not up yet, the shifts are already measured, or a
## measurement is already running.
func request_tier2_preview() -> void:
	if not mission_online or tier2_ready or tier2_measuring:
		return
	if mission.begin_tier2_preview():
		tier2_measuring = true
		event_logged.emit(_stamp(t) + "  MEASURING TIER-2 FORCE SHIFTS - ~2 MIN, STAND BY")


## Pump the on-demand measurement each frame; adopt the shifts when the worker
## lands them. Mirrors `_poll_build`.
func _poll_tier2_preview() -> void:
	if not tier2_measuring:
		return
	# poll_tier2_preview returns true while running, false once landed.
	if not mission.poll_tier2_preview():
		tier2_measuring = false
		tier2_ready = mission.has_tier2_preview()
		if tier2_ready:
			event_logged.emit(_stamp(t) + "  TIER-2 FORCE SHIFTS READY - GR/YARK/BELT/SRP/J2")
		else:
			event_logged.emit(_stamp(t) + "  TIER-2 MEASUREMENT FAILED - " + str(mission.last_error()))


## Flip one Tier-2 term's reveal state. `term` is a core id
## ("relativity"/"yarkovsky"/"belt"/"srp"/"j2"). Switching a term ON that the core
## could not measure (the belt with no small-body kernel mounted) is called out
## rather than silently revealing nothing.
func toggle_tier2(term: String) -> void:
	if not tier2_on.has(term):
		return
	tier2_on[term] = not tier2_on[term]
	if tier2_on[term] and not tier2_available(term):
		event_logged.emit("%s SHIFT UNAVAILABLE - NO SMALL-BODY KERNEL" % term.to_upper())


## Whether the core has a measured shift for this term. False for the belt when the
## small-body kernel was never mounted — the shift is genuinely unknown there, and
## the panel must say so rather than draw a 0 km that reads as "does nothing". The
## core returns its -1 sentinel for that case; a real perigee is never negative.
func tier2_available(term: String) -> bool:
	if not mission_online or not tier2_ready:
		return false
	return mission.tier2_shifted_perigee_m(term) >= 0.0


## The isolated b-plane perigee SHIFT for one term, km — `nominal − shifted`.
## Signed: positive means the term pulls the perigee inward (closer to Earth's
## centre), negative means it eases the pass outward. `NAN` when the term is
## unavailable, so callers gate on `tier2_available` before formatting.
func tier2_shift_km(term: String) -> float:
	if not tier2_available(term):
		return NAN
	return nom_perigee_km - mission.tier2_shifted_perigee_m(term) / 1000.0


## The J2 perigee shift measured on a genuine **miss** geometry, km — the in-domain
## companion to the menu's own J2 entry, which (like every other term) is measured
## on the nominal impact, 3000 km from Earth's centre and therefore *inside* the
## body where the J2 expansion does not hold.
##
## Read from the core rather than written here, so the footnote and the physics
## cannot drift: the constant is pinned to the core's own measurement by a test.
## `NAN` before the extension loads, so the panel can omit the note rather than
## print a zero that would read as "J2 does nothing out there".
func j2_miss_shift_km() -> float:
	if mission == null:
		return NAN
	return mission.j2_miss_geometry_shift_km()


# ------------------------------------------------------- porkchop / delivery ---

## Kick off the launch-window grid on a worker. Called when the heatmap view
## opens — on demand, like the Tier-2 menu, and for the same reason: it is real
## work (~0.6 s of Lambert solves) that the threat solution must never wait on.
## A no-op if there is no threat yet, a grid already exists, or one is building.
func request_porkchop() -> void:
	if not mission_online or pork_online or pork_building:
		return
	if mission.begin_porkchop(PORK_LAUNCH_SAMPLES, PORK_ARRIVAL_SAMPLES):
		pork_building = true
		event_logged.emit(_stamp(t) + "  SOLVING LAUNCH-WINDOW GRID - %dx%d TRANSFERS" %
			[PORK_LAUNCH_SAMPLES, PORK_ARRIVAL_SAMPLES])


## Pump the grid worker; pull the columns in when it lands. Mirrors
## `_poll_tier2_preview`.
func _poll_porkchop() -> void:
	if not pork_building:
		return
	# poll_porkchop returns true while running, false once landed.
	if mission.poll_porkchop():
		return
	pork_building = false
	pork_online = mission.has_porkchop()
	if not pork_online:
		event_logged.emit(_stamp(t) + "  LAUNCH-WINDOW GRID FAILED - " + str(mission.last_error()))
		return
	_fetch_porkchop()
	event_logged.emit(_stamp(t) + "  LAUNCH-WINDOW GRID READY - %d OF %d WINDOWS REACHABLE" %
		[pork_feasible_count(), pork_rows * pork_cols])


## Pull every grid column from the core into local caches.
##
## Read **once** per grid (and per launcher change), never per frame: each call
## marshals ~14 000 doubles across the FFI boundary. Nothing here is computed —
## these are projections of cells the core already solved.
func _fetch_porkchop() -> void:
	pork_rows = mission.porkchop_launch_count()
	pork_cols = mission.porkchop_arrival_count()
	pork_launch_tdb = mission.porkchop_launch_tdb()
	pork_arrival_tdb = mission.porkchop_arrival_tdb()
	pork_c3 = mission.porkchop_c3()
	pork_along = mission.porkchop_along_track()
	pork_revs = mission.porkchop_revolutions()
	_fetch_porkchop_vehicle()
	pork_i = clampi(pork_i, 0, maxi(pork_rows - 1, 0))
	pork_j = clampi(pork_j, 0, maxi(pork_cols - 1, 0))
	porkchop_changed.emit()


## Re-read only the two vehicle-dependent columns. This is the payoff of the
## core's vehicle-independent grid: switching launcher re-maps C3 to mass, it
## never re-solves a single Lambert arc.
func _fetch_porkchop_vehicle() -> void:
	pork_payload = mission.porkchop_payload_kg(pork_vehicle)
	pork_dv = mission.porkchop_along_track_dv(pork_vehicle)


## Row-major index of a cell, or -1 if out of range.
func pork_index(i: int, j: int) -> int:
	if i < 0 or j < 0 or i >= pork_rows or j >= pork_cols:
		return -1
	return i * pork_cols + j


## Whether a cell holds **no transfer at any allowed revolution count** — the
## grid's own blank, read from the one column entitled to say so.
func pork_blank(i: int, j: int) -> bool:
	var k := pork_index(i, j)
	return k < 0 or pork_c3[k] < 0.0


## Whether a cell holds a real transfer that the *current launcher* can reach.
## Distinct from `pork_blank`: this one is a fact about the rocket, and it changes
## under [V] while the trajectory underneath does not.
func pork_reachable(i: int, j: int) -> bool:
	var k := pork_index(i, j)
	return k >= 0 and pork_c3[k] >= 0.0 and pork_payload[k] > 0.0


func pork_feasible_count() -> int:
	var n := 0
	for k in range(pork_c3.size()):
		if pork_c3[k] >= 0.0 and pork_payload[k] > 0.0:
			n += 1
	return n


## The cursor cell's full readout row, or an empty Dictionary for a blank cell.
## One core call, so the panel can never assemble a row out of two cells.
func pork_cell() -> Dictionary:
	if not pork_online:
		return {}
	return mission.porkchop_cell(pork_i, pork_j, pork_vehicle)


func move_pork_cursor(di: int, dj: int) -> void:
	if not pork_online:
		return
	pork_i = clampi(pork_i + di, 0, pork_rows - 1)
	pork_j = clampi(pork_j + dj, 0, pork_cols - 1)


func cycle_pork_vehicle() -> void:
	if not mission_online:
		return
	pork_vehicle = (pork_vehicle + 1) % maxi(mission.vehicle_count(), 1)
	if pork_online:
		_fetch_porkchop_vehicle()
		porkchop_changed.emit()
	event_logged.emit("LAUNCHER: " + str(mission.vehicle_name(pork_vehicle)).to_upper())


func cycle_pork_metric() -> void:
	pork_metric = (pork_metric + 1) % PORK_METRICS.size()
	porkchop_changed.emit()


func pork_vehicle_name() -> String:
	if not mission_online:
		return "--"
	return str(mission.vehicle_name(pork_vehicle)).to_upper()


func pork_vehicle_max_c3() -> float:
	if not mission_online:
		return 0.0
	return mission.vehicle_max_c3(pork_vehicle)


## Fire the on-demand full-field verify of the cursor cell — one real n-body
## propagation with the impulse **this launcher** would actually deliver through
## **this window**. Everything above it is a patched-conic planning estimate; this
## is the only number in the view that the honest physics produced.
func request_cell_verify() -> void:
	if not pork_online or pork_verifying:
		return
	if pork_blank(pork_i, pork_j):
		event_logged.emit("NO TRANSFER IN THAT WINDOW - NOTHING TO VERIFY")
		return
	if not pork_reachable(pork_i, pork_j):
		event_logged.emit("%s DELIVERS NO MASS AT THAT C3 - NOTHING TO VERIFY" % pork_vehicle_name())
		return
	if mission.begin_cell_verify(pork_i, pork_j, pork_vehicle):
		pork_verifying = true
		event_logged.emit(_stamp(t) + "  VERIFYING WINDOW IN FULL N-BODY FIELD - STAND BY")
	else:
		event_logged.emit("VERIFY REFUSED - " + str(mission.last_error()))


func _poll_cell_verify() -> void:
	if not pork_verifying:
		return
	if mission.poll_cell_verify():
		return
	pork_verifying = false
	var v := pork_verdict()
	if v.is_empty():
		event_logged.emit(_stamp(t) + "  WINDOW VERIFY FAILED - " + str(mission.last_error()))
	else:
		event_logged.emit(_stamp(t) + "  WINDOW VERIFY: " + pork_verdict_label())
	porkchop_changed.emit()


## The last full-field verdict, or `{}` if none has been computed for this grid.
## Carries the cell it belongs to, so the view can tell "the cursor's verdict"
## from "a verdict for a window I have since moved off".
func pork_verdict() -> Dictionary:
	if not pork_online:
		return {}
	return mission.cell_verdict()


## Whether the last verdict describes the cell the cursor is on right now.
func pork_verdict_is_current() -> bool:
	var v := pork_verdict()
	if v.is_empty():
		return false
	return int(v.launch_index) == pork_i and int(v.arrival_index) == pork_j \
		and int(v.vehicle) == pork_vehicle


## The verdict in one line.
##
## **The comparison is |B| against the capture radius** — the pair the core's own
## `is_hit` uses, and exactly the `is_hit` flag it hands back. The perigee is shown
## beside it because "how close did it come" is what a reader means, but it pairs
## with Earth's solid radius, not with the capture disc. Mixing the pairs is the
## bug this project already shipped once: it reads plausible and is ~1.5x too
## strict, failing plans that physics calls safe.
func pork_verdict_label() -> String:
	var v := pork_verdict()
	if v.is_empty():
		return "NOT VERIFIED"
	match str(v.outcome):
		"clean_miss":
			# The best outcome there is — the deflected pass never came back for a
			# close approach at all. It has no b-plane numbers because there is no
			# encounter to reduce, not because they went missing.
			return "CLEAN MISS - NO EARTH ENCOUNTER"
		"not_hyperbolic":
			return "DEAD-CENTRE CAPTURE - NO B-PLANE SOLUTION"
		"encounter":
			var b: float = float(v.impact_parameter_m) / 1000.0
			var cap: float = float(v.capture_radius_m) / 1000.0
			var rp: float = float(v.perigee_m) / 1000.0
			var word: String = "SURFACE IMPACT" if bool(v.is_hit) else "MISS"
			return "%s - |B| %s KM vs CAPTURE %s KM (PERIGEE %s KM)" % [
				word, group_num(int(b)), group_num(int(cap)), group_num(int(rp))]
	return "UNKNOWN VERDICT"


# ------------------------------------------ required impactor mass ([M]) ---
#
# [E] asks "does THIS launcher work through THIS window", and when the answer is no
# it stops there. [M] asks the follow-up an operator actually needs: how much mass
# WOULD work. The two compose into the campaign's honest headline — a real rocket
# delivers a small fraction of what the window wants, and the fraction is the story.
#
# **Vehicle-independent**, so it survives an [L] press: the requirement is a property
# of the window's geometry and lead, and only the ratio against the payload moves.


## Fire the on-demand required-mass solve for the cursor cell.
##
## Far slower than [E] — several full-field propagations rather than one, measured
## on the shipping grid at **46 s** for the best-coupled window and **31 s** for a
## hopeless one, with early-arrival windows the slow tail (~3 min: a probe re-flies
## the whole remaining cruise, 18 s at ten years out against 6 s at three). The panel
## says so while it runs; there is no way to make this cheap that does not make it a
## different number.
func request_required_mass() -> void:
	if not pork_online or pork_mass_solving:
		return
	# A window with no transfer has no geometry to solve against. Note this is the
	# ONLY gate: unlike [E], a window this launcher cannot reach is still worth
	# solving — "what would it take" is exactly the question an unreachable cell
	# raises, and refusing it would answer only where the answer is least needed.
	if pork_blank(pork_i, pork_j):
		event_logged.emit("NO TRANSFER IN THAT WINDOW - NOTHING TO SOLVE")
		return
	if mission.begin_required_mass(pork_i, pork_j):
		pork_mass_solving = true
		event_logged.emit(_stamp(t) + "  SOLVING REQUIRED IMPACTOR MASS - THIS TAKES A MINUTE")
	else:
		event_logged.emit("MASS SOLVE REFUSED - " + str(mission.last_error()))


func _poll_required_mass() -> void:
	if not pork_mass_solving:
		return
	if mission.poll_required_mass():
		return
	pork_mass_solving = false
	var m := pork_required_mass()
	if m.is_empty():
		event_logged.emit(_stamp(t) + "  MASS SOLVE FAILED - " + str(mission.last_error()))
	else:
		# "NEEDS", not "REQUIRED MASS:" — the log column is only as wide as the
		# readout panel's left edge in this view, and the ratio at the end of the
		# label is the part a reader wants, so the prefix is where the characters
		# come from rather than the payload. `hud.gd` clips as a backstop.
		event_logged.emit(_stamp(t) + "  NEEDS " + pork_required_mass_label())
	porkchop_changed.emit()


# ----------------------------------------------------------------- tractor ---

## Adopt the core's tractor configuration as the panel's starting point.
##
## Called once the threat solution lands, because the shipping hover distance and
## the law's validity floor are *core* facts. Restating them here would be the
## bug `tractor_defaults()` exists to prevent — the panel and the physics quietly
## describing different tractors.
func _seed_tractor_defaults() -> void:
	if not mission_online:
		return
	var d: Dictionary = mission.tractor_defaults()
	tractor.hover = float(d.hover_radii)
	tractor.radius = float(d.rock_radius_m)
	tractor_law_min_periods = float(d.law_min_periods)
	tractor_target_perigee_m = float(d.target_perigee_m)
	tractor_hover_min = float(d.min_hover_radii)
	# Seeding the bound is not enough — the *current* value must be pulled inside
	# it too, or a default that predates the bound sits below it untouched until
	# the knob is first turned.
	tractor.hover = maxf(tractor.hover, tractor_hover_min)


## Lead time in seconds — the knob is in orbital periods, because that is the
## unit the required-Δv law is stated in and the unit the campaign quotes leads
## in ("eight orbits"). Days would be a number with no physics attached to it.
func tractor_lead_s() -> float:
	return tractor.lead * period_s()


func tractor_duration_s() -> float:
	return tractor_lead_s() * tractor.duty * 0.01


## The threat's heliocentric period in seconds, or 0 before the solution lands.
func period_s() -> float:
	return mission.period_seconds() if mission_online else 0.0


## Everything the panel prints that costs nothing — one call, one dictionary.
## `{}` before the threat solution exists.
func tractor_readout() -> Dictionary:
	if not mission_online:
		return {}
	return mission.tractor_readout(
		tractor.mass * 1000.0, tractor.hover, tractor.radius,
		tractor_lead_s(), tractor_duration_s(), tractor.dir > 0.5)


func move_tractor_cursor(d: int) -> void:
	tractor_row = wrapi(tractor_row + d, 0, TRACTOR_KNOBS.size())
	tractor_changed.emit()


## Step the selected knob. `dir` is -1 or +1.
##
## Multiplicative knobs step by ×/÷ because they span decades: spacecraft mass
## runs from a smallsat to a battleship and hover distance from grazing to
## useless, and a linear step that is sensible at one end is invisible at the
## other. Additive knobs are the ones with a natural unit interval.
func adjust_tractor(dir: int) -> void:
	var knob: Array = TRACTOR_KNOBS[tractor_row]
	var id: String = knob[0]
	var lo: float = knob[3]
	var hi: float = knob[4]
	var step: float = knob[5]
	var mult: bool = knob[6]
	var v: float = tractor[id]
	if id == "dir":
		# Not a magnitude — a toggle. Both directions of the key flip it, so the
		# row behaves the way every other row does under LEFT/RIGHT.
		v = 1.0 - v
	elif mult:
		v = v * step if dir > 0 else v / step
	else:
		v = v + step * dir
	# The hover row's floor comes from the core, not from the table: it is a
	# physics bound (`1/cos(plume)`) and the table's own literal is a placeholder.
	if id == "hover":
		lo = maxf(lo, tractor_hover_min)
	tractor[id] = clampf(v, lo, hi)
	tractor_changed.emit()


## Fire the on-demand full-field probe: one real n-body propagation with the tow
## term switched on for exactly this window.
##
## **The only honest number in the panel.** Everything above it is the cheap
## model — good to a stated band, and deliberately biased toward "not enough" —
## while this is the perigee the shipping force model actually produces. Costs
## ~12 s at the campaign's longest lead, which is why it is a key press and not a
## live readout.
func request_tow_probe() -> void:
	if not mission_online or tractor_probing:
		return
	# Both refusals are reachable by turning a knob to a documented end stop, so
	# they are answered here in words rather than as a raw error from inside the
	# window constructor twelve seconds later.
	if tractor_duration_s() <= 0.0:
		event_logged.emit("NO TOW TO PROBE - TOW DURATION IS ZERO")
		return
	if not bool(tractor_readout().get("holds_station", false)):
		event_logged.emit("NO STATION-KEEPING SOLUTION - RAISE HOVER DISTANCE")
		return
	if mission.begin_tow_probe(
			tractor.mass * 1000.0, tractor.hover, tractor.radius,
			tractor_lead_s(), tractor_duration_s(), tractor.dir > 0.5):
		tractor_probing = true
		event_logged.emit(_stamp(t) + "  TOWING IN FULL N-BODY FIELD - STAND BY")
	else:
		event_logged.emit("TOW PROBE REFUSED - " + str(mission.last_error()))


func _poll_tow_probe() -> void:
	if not tractor_probing:
		return
	if mission.poll_tow_probe():
		return
	tractor_probing = false
	var p := tractor_probe()
	if p.is_empty():
		event_logged.emit(_stamp(t) + "  TOW PROBE FAILED - " + str(mission.last_error()))
	else:
		event_logged.emit(_stamp(t) + "  TOW RESULT: " + tractor_probe_label())
	tractor_changed.emit()


## The last full-field probe, or `{}` if none has been run.
func tractor_probe() -> Dictionary:
	if not mission_online:
		return {}
	return mission.tow_probe()


## Whether that probe describes the knobs as they stand right now.
##
## Same staleness discipline the porkchop's verdict gets, and it matters more
## here: every knob is continuous, so a probe is one keypress away from being
## about a configuration the operator has left. A perigee shown as current when
## it is not would be a measured number lying.
func tractor_probe_is_current() -> bool:
	var p := tractor_probe()
	if p.is_empty():
		return false
	return is_equal_approx(float(p.spacecraft_mass_kg), tractor.mass * 1000.0) \
		and is_equal_approx(float(p.hover_radii), tractor.hover) \
		and is_equal_approx(float(p.rock_radius_m), tractor.radius) \
		and is_equal_approx(float(p.lead_seconds), tractor_lead_s()) \
		and is_equal_approx(float(p.duration_seconds), tractor_duration_s()) \
		and bool(p.retrograde) == (tractor.dir > 0.5)


## The probe in one line — **perigee and the signed shift, never just the ratio**.
##
## The sign is the finding. A tractor too weak to carry the b-plane point past
## Earth walks it *toward* the centre first, so a feeble tow makes the impact
## deeper: the campaign measures the shipping 20 t tractor moving perigee 3000.0
## -> 2811.6 km. A user tuning a knob and watching only a margin creep upward
## would read that as progress. Printing the direction is what stops the panel
## teaching the opposite of the lesson.
func tractor_probe_label() -> String:
	var p := tractor_probe()
	if p.is_empty():
		return "NO PROBE"
	if bool(p.clean_miss):
		return "CLEAN MISS - THREAT RETIRED"
	var per := float(p.perigee_m) / 1000.0
	var shift := float(p.shift_m) / 1000.0
	var arrow := "DEEPER" if shift < 0.0 else "OUTWARD"
	return "PERIGEE %s KM (%+.1f KM %s)" % [group_num(int(per)), shift, arrow]


## Whether the probed perigee clears the campaign's safe bar — the same
## `SAFE_PERIGEE_TARGET_M` the headline Δv curve and the launch-window map's
## required mass are quoted against, so the three numbers compose.
func tractor_probe_clears() -> bool:
	var p := tractor_probe()
	if p.is_empty():
		return false
	return bool(p.clean_miss) or float(p.perigee_m) >= tractor_target_perigee_m


## The last required-mass result, or `{}` if none has been solved for this grid.
func pork_required_mass() -> Dictionary:
	if not pork_online:
		return {}
	return mission.required_mass()


## Whether the last requirement describes the window the cursor is on.
##
## No vehicle in the comparison — deliberately. The requirement does not depend on
## the launcher, so cycling [L] must not grey out a number that is still true; only
## the ratio beside it changes.
func pork_required_mass_is_current() -> bool:
	var m := pork_required_mass()
	if m.is_empty():
		return false
	return int(m.launch_index) == pork_i and int(m.arrival_index) == pork_j


## The requirement in one line, **naming the target it was solved against**.
##
## That naming is not decoration. "REQUIRED MASS 157 T" alone reads as the mass
## needed to miss Earth, which is a smaller number; this is the mass to reach the
## campaign's 20 000 km safe perigee, the same bar the headline Delta-v curve is
## quoted against. Two requirements measured against different bars would look
## comparable and would not be.
##
## `infeasible_at_cap` is **data, not failure** — the honest state of a window no
## plausible launch fleet can save — so it prints what it reached, not an error.
func pork_required_mass_label() -> String:
	var m := pork_required_mass()
	if m.is_empty():
		return "NOT SOLVED"
	var tgt: float = float(m.target_perigee_m) / 1000.0
	match str(m.outcome):
		"feasible":
			var kg: float = float(m.impactor_mass_kg)
			# The ratio divides by the payload at **the solved window**, read off the
			# requirement itself — never at the cursor. The solve takes 46-180 s and
			# the operator is free to move during it, so the two are routinely
			# different cells by the time this formats. The panel happens to be safe
			# (it prints only when the two agree), but `_poll_required_mass` logs this
			# line unconditionally on arrival, and a row assembled out of two windows
			# is exactly the failure this view keeps its verdict pairing to avoid.
			return "%s KG FOR A %s KM PERIGEE%s" % [
				group_num(int(kg)), group_num(int(tgt)),
				_mass_ratio_suffix(kg, int(m.launch_index), int(m.arrival_index))]
		"infeasible_at_cap":
			var cap: float = float(m.mass_cap_kg)
			var got: float = float(m.perigee_reached_m) / 1000.0
			return "OVER %s KG (%d LAUNCHES) - GETS %s OF %s KM" % [
				group_num(int(cap)), int(round(cap / maxf(mission.heaviest_deliverable_kg(), 1.0))),
				group_num(int(got)), group_num(int(tgt))]
	return "UNKNOWN REQUIREMENT"


## " = 100 x WHAT THIS LAUNCHER DELIVERS" for the window at `(i, j)` — **the cell
## the requirement was solved for**, which the caller passes explicitly rather than
## letting this reach for the cursor. The ratio is the point of the whole readout;
## it is left off rather than faked when the selected rocket delivers nothing there.
func _mass_ratio_suffix(required_kg: float, i: int, j: int) -> String:
	var k := pork_index(i, j)
	if k < 0 or pork_payload[k] <= 0.0:
		return ""
	return " - %sx %s" % [
		group_num(int(round(required_kg / pork_payload[k]))), pork_vehicle_name()]


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
		"catalog":
			# Orrery scenery, flown in the same field on the build worker. Gated on
			# its own propagated span for the same reason the threat is: outside it
			# the binding returns ZERO, which here is the Sun.
			if not catalog_active(el, t_days):
				return Vector3.ZERO
			return mission.catalog_position_ecl_au(el.catalog_index, tdb(t_days))
	# Every drawn body now names a real source. Reaching here is a bug, and it must
	# not present as one: ZERO is the Sun in this frame, so say so out loud rather
	# than quietly parking the body at the origin.
	push_error("pos_ecl: body '%s' has no known source '%s'" % [
		el.get("name", "?"), el.get("source", "")])
	return Vector3.ZERO


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

	if src == "catalog":
		# Any catalog body, not just the comet — gated on the body having an index
		# rather than on a per-body online flag, since the mission may carry a
		# comet, real NEOs, both or neither.
		if not mission_online or not el.has("catalog_index"):
			return pts
		var idx: int = el.catalog_index
		var period_s: float = mission.catalog_orbit_period_seconds(idx)
		var span: PackedFloat64Array = mission.catalog_span_tdb(idx)
		# One orbital period, not the whole table. A NEO's states cover ~50 years
		# but its orbit is ~1 year, so the full span is dozens of precessing laps
		# drawn over each other; the binding returns the period for exactly this.
		# The comet reports its whole 22.6-yr span as "one period" and is unchanged.
		# Sampled via the binding's windowed track — the same arc `pos_ecl` walks,
		# so the drawn orbit and the moving body cannot disagree.
		if period_s > 0.0 and span.size() == 2 and (span[1] - span[0]) > period_s:
			# Anchor the lap at the body's active midpoint, clamped inside the span.
			var mid: float = 0.5 * (span[0] + span[1])
			var t0: float = clampf(mid - 0.5 * period_s, span[0], span[1] - period_s)
			for p in mission.catalog_track_window_ecl_au(idx, t0, t0 + period_s, count):
				pts.append(ecl_to_godot(p))
		else:
			for p in mission.catalog_track_ecl_au(idx, count):
				pts.append(ecl_to_godot(p))
		return pts

	push_error("orbit_points: body '%s' has no known source '%s'" % [
		el.get("name", "?"), src])
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


## Whether the *deflected* track is the one that is real right now.
##
## The single rule for "which pass is happening", because two places need it and
## they must not drift: the encounter view draws the live marker on this track,
## and `encounter_ca_day` snaps the clock to that same track's closest approach.
## Disagreement between them is not a cosmetic bug — the deflected closest
## approach is time-shifted about half a day off the nominal one, so a snap
## computed on the wrong track lands outside the ±1.5 d window and the marker
## silently fails to appear, which is the exact complaint this snap exists to fix.
##
## Before the burn the deflected track has not happened yet; after it, an empty
## deflected track means the core has no plan to draw.
func deflected_is_live(deflected_track_empty: bool) -> bool:
	return burned() and not deflected_track_empty


## The mission day of the active track's closest approach to Earth — the one
## instant the live marker is guaranteed to be on-plot and at its most
## interesting (the rock sits at perigee, *inside* its own b-point).
##
## NAN when there is no track to search. Callers jump to it; see `main.gd`.
##
## This is an argmin over a polyline the core produced, in the core's own
## Earth-centred b-plane display frame — the same category of operation as the
## interpolation `encounter.gd` already does along that polyline, and emphatically
## not a re-derivation of the encounter. GDScript still owns zero orbital
## mechanics here. Resolution is the sample spacing (~185 s over the ±1.5 d
## window), which is below the `CA %+.2f D` readout's own 864 s precision, so
## plumbing the core's bisected CA epoch out through four signatures would buy
## accuracy nothing on screen can show.
##
## `test_encounter_ca` pins the result against the core's reported perigee: if
## the minimum sample range matches, the frame really is Earth-centred and this
## really is the closest approach.
func encounter_ca_day() -> float:
	var span := encounter_span_days()
	if span.size() != 2:
		return NAN
	var trk := encounter_track(deflected_is_live(encounter_track(true).is_empty()))
	if trk.size() < 2:
		return NAN
	var best := 0
	var best_r: float = trk[0].length()
	for i in range(1, trk.size()):
		var r: float = trk[i].length()
		if r < best_r:
			best_r = r
			best = i
	# Samples are uniform over the span — the inverse of the marker's own mapping.
	return span[0] + (span[1] - span[0]) * float(best) / float(trk.size() - 1)


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
