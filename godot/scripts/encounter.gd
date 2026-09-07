class_name EncounterView
extends Control
## Earth-encounter close-up: the classic b-plane targeting picture, read from the
## core.
##
## Looking straight down the incoming asymptote, Earth at the origin. What decides
## the outcome is where each pass's asymptote **pierces this plane** — the b-vector
## B — measured against the gravitationally-focused capture disc. |B| > b_capture
## is a miss. That is the core's own hit test and the same pair the planner's
## verdict reads (`Sim._solve_plan`), so this view and that panel cannot disagree.
##
## The disc is the whole pedagogical payload (HANDOFF §5): Earth's gravity makes
## Earth a bigger target than Earth. At this encounter's v_inf ≈ 7.63 km/s the
## collision cross-section is 1.77 R_E, so a pass aimed to clear the planet by half
## an Earth radius is still reeled in. The tracks are drawn as context, and they
## are the same physics seen from the side: they visibly bend toward the origin.
##
## **A track may cross the capture disc on a perfectly safe pass, and that is not a
## contradiction.** The disc is the cross-section for the *asymptote's* piercing
## point, not for the curved path, which bottoms out at its perigee — a smaller
## number than |B|. The asymptote is what the disc judges; the track is scenery.
##
## This view owns no geometry. Every point arrives from the core already projected
## into its b-plane display frame — (xi, zeta, s) km, s being depth along the
## asymptote — because the asymptote lives in the core and picking the frame is the
## only judgement involved. What used to be here is gone rather than ported: a
## Kepler `close_approach`, its own R_E and V_ESC, and a v_inf taken from the
## closest-approach speed instead of the hyperbolic excess. That last one is why
## the old view drew a 3.7 R_E disc where the real one is 1.77 — it was reading
## 3.17 km/s where the encounter's v_inf is 7.63.
##
## The axes are the core's **pinned Öpik frame** since the keyhole batch
## (core/src/keyhole.rs): xi across Earth's heliocentric motion, zeta against it,
## B's sign pointing at the incoming asymptote. That is what lets this view draw
## the **keyhole map** — the resonant-return circles, each the locus of b-plane
## points whose flyby brings the rock back h years later — because those circles
## are centred on the zeta axis, and it is why a signed zeta now means something:
## "later" is down. `[H]` toggles the circles. If the core could not pin the
## frame (`Sim.bplane_frame_pinned()` false) the axes fall back to a display
## basis and the readout says so; no circles are drawn then.

## Plot half-span at open, lunar distances. Sized for the capture disc (0.029 LD),
## not for the LD-scale rings the heliocentric views use: this is the one frame
## where the whole story happens inside a tenth of a lunar distance. The b-plane
## projection cooperates — it drops the huge `s` component, so an inbound track
## that reaches ~10^6 km down-range still sits within ~|B| of the origin here.
const DEFAULT_HALF_LD := 0.15
const MARGIN := 18.0
## Where the Tier-3 readout block starts, px from the top.
##
## An empty band: below the view header and above the HUD's target card, which
## begins around y = 260. Chosen from a screenshot rather than from the layout
## code, because the pieces that share this screen are drawn by three different
## nodes and only the picture knows where they all land.
const UNCERTAINTY_READOUT_Y := 108.0

var _font: Font
var _fs := 13
var _half_ld := DEFAULT_HALF_LD

# Cached core reads. The tracks are ~1400 points each and never change unless the
# plan does, so they are pulled on `plan_changed` / first draw rather than per
# frame. `_built` false means "re-read from the core", not "recompute" — there is
# nothing to compute here.
var _built := false
var _nom := PackedVector3Array()
var _defl := PackedVector3Array()
var _b_nom := Vector3.ZERO
var _b_defl := Vector3.ZERO
var _span := PackedFloat64Array()
## The keyhole map: resonant-return circles with returns inside
## Sim.KEYHOLE_MAX_YEARS, read from the core (closed-form; see
## Sim.keyhole_circles). Drawn when `_keyholes` is on and the frame is pinned.
##
## The horizon lives on Sim because the planner's keyhole readout has to use the
## same one: naming a resonance in the panel that this view does not draw would
## be a picture and a number disagreeing, which is the failure the whole b-plane
## view exists to end.
## How many circles get a caption at once (the widest in frame; see _draw_keyholes).
const KEYHOLE_LABELS := 7
var _circles: Array = []
var _keyholes := true

## The Tier-3 uncertainty overlay: the 1-sigma b-plane ellipse the orbit's own
## covariance implies, and the fraction of it that lands on the capture disc.
##
## Off by default and **paid for on demand** — the sensitivity behind it is 13
## propagations, ~17 s on a worker — so [U] both opens it and orders it. Unlike
## the keyhole map, whose circles are closed-form and free, this one cannot be
## drawn the frame it is asked for, and the view says so while it waits rather
## than showing an empty overlay that reads as "no uncertainty".
##
## Everything drawn here belongs to the **nominal** track. The Jacobian is taken
## about the undeflected seed, so there is no deflected ellipse and the legend has
## to say so: a spread on the cross with a bare diamond beside it would read as
## "the deflection is certain", which is the opposite of §1's whole caveat.
var _uncertainty := false

## Whether the live-asteroid contact went on screen this frame. Set by
## _draw_marker, read by _draw_legend a few calls later — draw order, not cached
## state, so it cannot go stale.
var _marker_live := false

## Projected track segments, ready to draw. See `_track_runs`.
var _runs := {}
var _runs_key := ""


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_preset(Control.PRESET_FULL_RECT)
	_font = Sim.mono_font
	# The deflected track and its b-point only exist once the core has solved a
	# plan, and they change on every re-solve.
	Sim.plan_changed.connect(func() -> void: _built = false)
	# The threat itself is ~10 s away at scene load; nothing exists to read before
	# the scenario lands.
	Sim.mission_ready.connect(func() -> void: _built = false)
	# The ellipse is not part of `_fetch`: it is refetched by Sim on the events that
	# can change it (the solve landing, the sigma knob), and read from there. A
	# `_built = false` here would re-pull the tracks every time the knob moved.
	Sim.tier3_changed.connect(func() -> void: queue_redraw())


func _process(_delta: float) -> void:
	if visible:
		queue_redraw()


func _unhandled_input(event: InputEvent) -> void:
	# Wheel zooms the plot span; swallow it so the 3D rig underneath doesn't zoom
	# too. (Added after the camera rig, so we see it first.)
	if not visible:
		return
	if event is InputEventMouseButton and event.pressed:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP:
			_half_ld = clampf(_half_ld * 0.8, 0.01, 30.0)
			get_viewport().set_input_as_handled()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_half_ld = clampf(_half_ld * 1.25, 0.01, 30.0)
			get_viewport().set_input_as_handled()


## Pull the encounter from the core. No geometry, only marshalling — every one of
## these is a cached read of work the core already did.
func _fetch() -> void:
	_nom = Sim.encounter_track(false)
	_defl = Sim.encounter_track(true)
	_b_nom = Sim.encounter_b_point(false)
	_b_defl = Sim.encounter_b_point(true)
	_span = Sim.encounter_span_days()
	_circles = Sim.keyhole_circles(Sim.KEYHOLE_MAX_YEARS) if Sim.bplane_frame_pinned() else []
	# The projected tracks are built from `_nom`/`_defl`, which are assigned here
	# and nowhere else — so this is the whole of their invalidation.
	_runs.clear()
	_built = true


## [H]: show or hide the resonant-return circles.
func toggle_keyholes() -> void:
	_keyholes = not _keyholes
	Sim.event_logged.emit("KEYHOLE MAP %s" % ("ON" if _keyholes else "OFF"))


## [U]: show or hide the Tier-3 uncertainty ellipse, ordering the solve the first
## time it is asked for. Sim's request is a no-op once solved or in flight, so
## toggling repeatedly costs nothing.
func toggle_uncertainty() -> void:
	_uncertainty = not _uncertainty
	if _uncertainty:
		Sim.request_tier3()
	Sim.event_logged.emit("UNCERTAINTY OVERLAY %s" % ("ON" if _uncertainty else "OFF"))


# ------------------------------------------------------------------ draw ---

func _draw() -> void:
	var w := size.x
	var h := size.y
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.004, 0.006, 0.005), true)

	var bright := Color(1, 1, 1)
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.42, 0.42, 0.42)
	var faint := Color(0.18, 0.18, 0.18)

	# Nothing to draw until the core has a threat. Say so rather than presenting an
	# empty grid as a measured encounter — a blank instrument reads as "clear".
	if not Sim.encounter_online:
		_centered("ENCOUNTER SOLUTION NOT ACQUIRED", Vector2(w * 0.5, h * 0.5), dim, _fs)
		_centered("INTEGRATING 12 YR OF REAL N-BODY MOTION",
			Vector2(w * 0.5, h * 0.5 + 18.0), faint, _fs - 2)
		return
	if not _built:
		_fetch()

	var center := Vector2(w * 0.5, h * 0.5)
	var ppl := minf(w, h) * 0.5 / _half_ld * 0.92      # px per lunar distance

	# Targeting axes — a display frame, and labelled as one (see the class doc).
	draw_line(Vector2(0, center.y), Vector2(w, center.y), faint, 1.0)
	draw_line(Vector2(center.x, 0), Vector2(center.x, h), faint, 1.0)

	# The captions this frame's b-points will print, decided before anything is
	# drawn: the keyhole pass runs first (circles are context and belong under the
	# marks) and must know which rectangles are already spoken for.
	var caps := _b_captions(center, ppl)
	var reserved: Array[Rect2] = _draw_rings(center, ppl, w, h, faint, dim)
	for c: Dictionary in caps:
		reserved.append(c["rect"])
		# The mark itself, not just its caption. A keyhole name drawn through the
		# diamond is the same defect as one drawn through "B 0.04 LD" — the glyph
		# is the answer and the name beside it is context.
		reserved.append(Rect2(c["mark"] - Vector2(9, 9), Vector2(18, 18)))
	if _keyholes:
		_draw_keyholes(center, ppl, dim, faint, reserved)
	_draw_earth_and_disc(center, ppl, bright, mid, dim)

	# Tracks: context. Dim, and behind the b-points that actually decide things.
	_draw_track(_nom, center, ppl, Color(0.5, 0.5, 0.5), "nom")
	if not _defl.is_empty():
		# Faint while a solve is pending: this is still the previous plan's arc.
		var a: float = 0.3 if Sim.plan_solving else 0.75
		_draw_track(_defl, center, ppl, Color(0.62, 0.62, 0.62, a), "defl")

	# Top-left, well above the encounter solution — NOT under it. The first
	# screenshot of this overlay had it printed below the verdict, where the HUD's
	# event log lands on top of it and both became unreadable. That column belongs
	# to the HUD from ~0.7 h down; this block gets the empty band above the target
	# card instead.
	_draw_uncertainty_readout(MARGIN, UNCERTAINTY_READOUT_Y, _fs + 5.0, mid, dim)
	_draw_b_points(center, ppl, bright, mid, dim, caps)
	# Over the b-points, under the live marker: the ellipse is about where the
	# nominal crossing *is not* pinned down, so it has to be readable against the
	# cross it surrounds.
	if _uncertainty:
		_draw_uncertainty(center, ppl, bright, mid, dim)
	_draw_marker(center, ppl, bright, mid)
	_draw_legend(w, mid, dim)

	# Header.
	_centered("EARTH ENCOUNTER - B-PLANE VIEW", Vector2(w * 0.5, 40), dim, _fs)
	_readout(w, h, mid, bright, dim)

	var foot := "SPAN +/-%s LD   [WHEEL] ZOOM   [H] KEYHOLES %s   [U] UNCERTAINTY %s%s" % [
		String.num(_half_ld, 3), "ON" if _keyholes else "OFF",
		"ON" if _uncertainty else "OFF",
		"   [Z]/[X] SIGMA" if _uncertainty and Sim.tier3_online else ""]
	_centered(foot, Vector2(w * 0.5, h - MARGIN - 4), dim, _fs - 2)


## Range rings in lunar distances, auto-selected for the current zoom so the
## close-up (0.15 LD) and a wide view (10 LD) both get sensible labels.
##
## Nothing is drawn inside the capture disc. A ring there would be a second
## distance reference inside the only one that decides anything, and its label
## would land on Earth — the clutter that made the first draft unreadable.
## Returns the rectangles its labels occupy, so the keyhole captions drawn after
## it can steer clear of them.
func _draw_rings(center: Vector2, ppl: float, w: float, h: float,
		faint: Color, dim: Color) -> Array[Rect2]:
	var floor_px: float = Sim.cap_km / Sim.LD_KM * ppl + 10.0
	var rects: Array[Rect2] = []
	for r: float in [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0]:
		var rp: float = r * ppl
		if rp < floor_px or rp > minf(w, h) * 0.52:
			continue
		draw_arc(center, rp, 0, TAU, 160, faint, 1.0)
		var lbl := String.num(r, 2) + " LD" + (" - LUNAR DIST" if r == 1.0 else "")
		var at := center + Vector2(rp * 0.7071 + 5, -rp * 0.7071 - 4)
		draw_string(_font, at, lbl, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)
		rects.append(_text_rect(at, lbl, _fs - 2))
	return rects


## The keyhole map: one circle per resonant return, centred on the zeta axis at
## (0, center_zeta) with radius R, both from the core. Every point on a circle is
## a flyby that leaves the rock on an orbit meeting Earth again h years later; the
## thin band around it that lands the return on the capture disc is the keyhole,
## and its width is printed at whichever crossing of the zeta axis is on the plot.
##
## Most circles pass *through* the disc — part of every listed resonance is an
## impact — and the far ends of the wide ones sit well outside the default span,
## so zoom out with the wheel to see 3:4 whole. Circles are context, drawn under
## the disc and the marks: they never decide anything on this screen.
## `reserved` is text already on the plot — the range-ring labels and the two
## b-point captions. A keyhole name is context; the b-point captions are the
## comparison this whole view exists to make, so the names give way, never the
## other way round.
func _draw_keyholes(center: Vector2, ppl: float, dim: Color, faint: Color,
		reserved: Array[Rect2]) -> void:
	if _circles.is_empty():
		return
	var rect := Rect2(Vector2.ZERO, size).grow(-MARGIN)

	# First pass: the circles, every one. Brightness follows keyhole width, so the
	# eye lands on the resonances that matter — a 25 km keyhole is a real target
	# and a 0.03 km one is a hairline — and the 3:4 (the one the project flew) is
	# drawn solid. The first time this map ran, all ~40 circles came out at one
	# weight with a caption each, and the picture was a thicket: the widths went
	# unread because nothing said which of forty lines to read first.
	var widest := 0.0
	for c: Dictionary in _circles:
		widest = maxf(widest, maxf(float(c["far_width_km"]), float(c["near_width_km"])))
	var labelled: Array = []                # [y_at, x_at, text, colour, width]
	for c: Dictionary in _circles:
		var cc: Vector2 = _plot(center, ppl, Vector3(0.0, float(c["center_zeta_km"]), 0.0))
		var r: float = float(c["radius_km"]) / Sim.LD_KM * ppl
		# Entirely off-frame: the nearest point of the circle is beyond the corners.
		if cc.distance_to(center) - r > rect.size.length():
			continue
		var is_three_four: bool = int(c["h"]) == 3 and int(c["k"]) == 4
		# Which crossing of the zeta axis is on the plot decides where the caption
		# goes and which width it quotes; neither on-plot means no caption.
		var far := _plot(center, ppl, Vector3(float(c["far_xi_km"]), float(c["far_zeta_km"]), 0.0))
		var near := _plot(center, ppl,
			Vector3(float(c["near_xi_km"]), float(c["near_zeta_km"]), 0.0))
		var at := Vector2.INF
		var width_km := 0.0
		if rect.has_point(far):
			at = far
			width_km = float(c["far_width_km"])
		elif rect.has_point(near):
			at = near
			width_km = float(c["near_width_km"])
		# Width on a log scale from the widest down to a thousandth of it.
		var w_rel: float = clampf(1.0 + log(maxf(width_km, widest * 1e-3) / widest) / log(1000.0), 0.0, 1.0)
		var col: Color
		if is_three_four:
			col = Color(dim, 0.95)
		else:
			var g: float = lerpf(faint.r * 1.15, dim.r, w_rel)
			col = Color(g, g, g, lerpf(0.45, 0.9, w_rel))
		var segs := 96 if r < 2000.0 else 256
		draw_arc(cc, r, 0.0, TAU, segs, col, 1.4 if is_three_four else 1.0)
		if at != Vector2.INF:
			var txt := "%d:%d  KEYHOLE %s KM" % [int(c["h"]), int(c["k"]), String.num(width_km, 2)]
			labelled.append([at.y, at.x, txt, col, width_km + (1e9 if is_three_four else 0.0)])

	# Second pass: captions for the widest few only, the 3:4 always among them.
	# Every circle used to get one; at the default span that is ~30 captions
	# stacked down the zeta axis through Earth, the b-point and the impact mark.
	# The rest are on the plot uncaptioned — zoom in and they get their turn, since
	# the budget is spent on whatever is widest among the circles in frame.
	labelled.sort_custom(func(a: Array, b: Array) -> bool: return a[4] > b[4])
	var budget: int = mini(labelled.size(), KEYHOLE_LABELS)
	var placed: Array[float] = []
	var fs: int = _fs - 3
	for i in budget:
		var e: Array = labelled[i]
		var y: float = e[0]
		for py: float in placed:
			if absf(y - py) < _fs + 2.0:
				y = py + _fs + 2.0
		# Right of the crossing, then left of it, then one row further down. Left
		# first because these circles are centred on the zeta axis and the b-point
		# captions all run rightward from marks on or near it, so the far side is
		# usually clear; the row step is the fallback for when it is not.
		var tw: float = _font.get_string_size(
			e[2], HORIZONTAL_ALIGNMENT_LEFT, -1, fs).x
		var at := Vector2(e[1] + 8.0, y + 4.0)
		for cand: Vector2 in [at, Vector2(e[1] - 8.0 - tw, y + 4.0),
				Vector2(e[1] + 8.0, y + 4.0 + _fs + 2.0)]:
			at = cand
			if not _overlaps(_text_rect(at, e[2], fs), reserved):
				break
		placed.append(at.y - 4.0)
		draw_string(_font, at, e[2], HORIZONTAL_ALIGNMENT_LEFT, -1, fs, e[3])


## What the marks mean, stated once in a corner instead of on top of them. The
## b-plane projection collapses the huge along-asymptote distance, so the inbound
## end of a track lands right on that pass's b-point — label them in place and every
## caption in the picture stacks on the same few pixels.
func _draw_legend(w: float, mid: Color, dim: Color) -> void:
	var x := w - 250.0
	var y := 470.0
	var lh := _fs + 4.0
	_cross(Vector2(x + 6, y - 4), 4.0, mid)
	draw_string(_font, Vector2(x + 18, y), "NOMINAL - THE IMPACT",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, mid)
	var row := 1.0
	if not _defl.is_empty():
		_diamond(Vector2(x + 6, y + row * lh - 4), 4.0, mid)
		draw_string(_font, Vector2(x + 18, y + row * lh), "DEFLECTED - PLANNED",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, mid)
		row += 1.0
	# Only while the rock is actually on the plot. A legend entry for a glyph that
	# is not on screen — which is the case for all but 3 days of a 12-year campaign
	# — teaches the reader to look for something that is not there.
	if _marker_live:
		_contact(Vector2(x + 6, y + row * lh - 4), 4.0, mid)
		draw_string(_font, Vector2(x + 18, y + row * lh), "2031-XK - POSITION NOW",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, mid)
		row += 1.0
	# "MARKS" means the diamonds and the cross. Saying it unqualified while the
	# ringed dot is on screen would make the legend describe it wrongly: the rock is
	# a position, not an asymptote crossing.
	draw_string(_font, Vector2(x, y + row * lh),
		"X / DIAMOND = ASYMPTOTE THRU THIS PLANE",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)
	draw_string(_font, Vector2(x, y + (row + 1.0) * lh),
		"LINES = TRACK (BENDS - CONTEXT ONLY)",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)
	var extra := 2.0
	if _keyholes and not _circles.is_empty():
		draw_string(_font, Vector2(x, y + (row + extra) * lh),
			"CIRCLES = RESONANT RETURNS H:K (KEYHOLE MAP)",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)
		extra += 1.0
	# Named as the nominal's, always — an ellipse on the cross beside a bare diamond
	# is a picture saying the deflection is certain, which is the exact thing §1's
	# determinism caveat exists to deny.
	if _uncertainty and Sim.tier3_online:
		draw_string(_font, Vector2(x, y + (row + extra) * lh),
			"ELLIPSE = 1-SIGMA, NOMINAL TRACK ONLY",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)


## Earth, and the disc that is the point of the whole view.
func _draw_earth_and_disc(center: Vector2, ppl: float, bright: Color, mid: Color,
		dim: Color) -> void:
	var re_px := maxf(Sim.R_E / Sim.LD_KM * ppl, 2.0)
	draw_circle(center, re_px, Color(0.09, 0.09, 0.09))
	draw_arc(center, re_px, 0, TAU, 96, bright, 1.4)
	if re_px > 6.0:
		draw_string(_font, center + Vector2(re_px + 7, 4), "EARTH",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)

	# The capture disc: the bar every verdict is measured against. Drawn dashed
	# because it is not a thing, it is a threshold.
	var cap_px: float = Sim.cap_km / Sim.LD_KM * ppl
	if cap_px > re_px + 3.0:
		_dashed_circle(center, cap_px, dim)
		draw_string(_font, center + Vector2(cap_px * 0.7071 + 5, cap_px * 0.7071 + 12),
			"CAPTURE %.2f RE" % (Sim.cap_km / Sim.R_E),
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)


## The b-points — where each asymptote pierces this plane. These are the operative
## marks: their distance from the centre IS the miss the verdict compares.
func _draw_b_points(center: Vector2, ppl: float, bright: Color, mid: Color,
		dim: Color, caps: Array) -> void:
	# Nominal: inside the disc, by construction. This is the hit — and it stays the
	# prediction until the burn actually happens, not merely until a plan is drawn
	# up. A solved plan sitting beside a blinking PREDICTED IMPACT is the whole
	# comparison this view exists to make.
	if _b_nom != Vector3.ZERO:
		var p := _clamp_to_plot(_plot(center, ppl, _b_nom))
		if Sim.burned():
			_cross(p, 5.0, dim)
		elif Sim.blink(1.6):
			_cross(p, 8.0, bright)
			_paint_caption(_cap(caps, "nom"), bright)

	# Deflected: ZERO means there is no such point — no plan, or a clean miss that
	# left the encounter entirely. Both must draw nothing; ZERO is Earth's centre.
	if _b_defl == Vector3.ZERO:
		return
	var pd := _clamp_to_plot(_plot(center, ppl, _b_defl))
	# While a solve is pending this mark still belongs to the PREVIOUS plan, so it
	# is drawn faint: the operator has moved on and this has not caught up. The
	# label says so too (`miss_label` reports SOLVING...), but a confidently-drawn
	# diamond in the wrong place is the part a player would believe.
	var solving: bool = Sim.plan_solving
	_dashed_line(center, pd, Color(dim, dim.a * (0.4 if solving else 1.0)), 6.0, 5.0)
	_diamond(pd, 6.0, Color(bright, 0.35 if solving else 1.0))
	_paint_caption(_cap(caps, "defl"), Color(bright, 0.5 if solving else 1.0))


## Where this frame's two b-point captions go, and what they say — decided in one
## place, before anything is drawn.
##
## Two reasons it is not simply done at the draw site. `_draw_keyholes` runs
## *first* (circles are context and belong under the marks) and needs these
## rectangles to avoid them, so they have to exist before the b-points are painted.
## And the two captions collide with **each other**: zoomed out, both marks sit
## almost on Earth and both captions want the same pixels — `enc_4_zoomed_out`
## printed "PREDICTED IMPACT" and "B 0.02 LD" on top of one another, which is not
## a crowded label but an unreadable one.
##
## The impact caption blinks. Its rectangle is returned on every frame regardless
## and `_paint_caption` is what the blink gates: a reservation that comes and goes
## twice a second would make every keyhole caption jump between two placements.
func _b_captions(center: Vector2, ppl: float) -> Array:
	var out: Array = []
	# Only when there is a caption to place: once burned, the nominal mark is a
	# dim unlabelled cross, and its rectangle must not be held against anything.
	if _b_nom != Vector3.ZERO and not Sim.burned():
		var raw := _plot(center, ppl, _b_nom)
		out.append(_caption("nom", _clamp_to_plot(raw), center,
			"PREDICTED IMPACT" + (" >" if not _on_plot(raw) else ""), out))
	if _b_defl != Vector3.ZERO:
		var raw_d := _plot(center, ppl, _b_defl)
		# Through Sim's formatter, like every other site that prints a miss.
		out.append(_caption("defl", _clamp_to_plot(raw_d), center,
			"B " + Sim.miss_label() + (" >" if not _on_plot(raw_d) else ""), out))
	return out


## One caption, placed clear of the ones already in `taken`.
##
## `_label_at` puts it radially outward from Earth; if that lands on a caption
## already placed, it steps down by a line until it does not. Stepping *down* and
## never sideways keeps the caption attached to the mark it names — the b-points
## are what this view is about, and a caption that has wandered off is worse than
## one sitting a line lower.
func _caption(kind: String, p: Vector2, center: Vector2, text: String,
		taken: Array) -> Dictionary:
	var at := _label_at(p, center, text)
	var lh: float = _fs + 3.0
	for _step in 4:
		var r := _text_rect(at, text, _fs)
		var clear := true
		for other: Dictionary in taken:
			if r.intersects(other["rect"]):
				clear = false
				break
		if clear:
			break
		at.y += lh
	return {"kind": kind, "text": text, "at": at, "mark": p,
		"rect": _text_rect(at, text, _fs)}


func _cap(caps: Array, kind: String) -> Dictionary:
	for c: Dictionary in caps:
		if c["kind"] == kind:
			return c
	return {}


func _paint_caption(c: Dictionary, col: Color) -> void:
	if c.is_empty():
		return
	draw_string(_font, c["at"], c["text"], HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)


## The live asteroid, when the clock is actually inside the encounter window.
##
## The window is ±1.5 days; the campaign is twelve years. So this is absent almost
## always, and it must be *absent* — the same gate `threat_active` applies to the
## orrery. Clamping the clock onto the nearest end of the track instead would park
## a marker at the frame edge and call it the asteroid's position.
##
## Drawn as a ringed dot, NOT the diamond the b-points use, because it is a
## different kind of thing: the b-points are where an asymptote pierces this plane,
## while this is the rock's actual position right now. Sharing a glyph would make
## the legend's "MARKS = ASYMPTOTE THROUGH THIS PLANE" a lie for one of them — and
## a picture quietly asserting something untrue is the failure this view exists to
## end. The difference is visible on screen too: at closest approach the rock sits
## at its perigee, *inside* its own b-point, which is the entire lesson of the
## verdict bug this view exposed.
func _draw_marker(center: Vector2, ppl: float, bright: Color, mid: Color) -> void:
	_marker_live = false
	if _span.size() != 2 or _nom.is_empty():
		return
	var t: float = Sim.t
	if t < _span[0] or t > _span[1]:
		return
	# The track that is real at this moment: the deflected one only after the burn.
	# Through Sim's rule, not a local copy of it — `Sim.encounter_ca_day` snaps the
	# clock using the same call, and a snap aimed at the other track lands off-plot.
	var trk: PackedVector3Array = _defl if Sim.deflected_is_live(_defl.is_empty()) else _nom
	# Samples are uniform over the span, so the clock maps straight to an index.
	# This interpolates a polyline the core produced — a drawing operation, not a
	# propagation; at ~185 s spacing the segments are far below a pixel here.
	var frac: float = (t - _span[0]) / (_span[1] - _span[0])
	var x := frac * float(trk.size() - 1)
	var i := clampi(int(x), 0, trk.size() - 2)
	var p3: Vector3 = trk[i].lerp(trk[i + 1], x - float(i))
	var p := _plot(center, ppl, p3)
	if not _on_plot(p):
		return
	_marker_live = true
	_contact(p, 4.0, bright if Sim.blink(2.2) else mid)
	draw_string(_font, p + Vector2(11, 5), "2031-XK  CA %+.2f D" % (t - Sim.T_IMPACT),
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)


## A track's (xi, zeta) shadow. Full brightness inbound (s < 0), dimmed once past
## the b-plane — the s component the projection carries, put to work.
##
## Unlabelled by design; see `_draw_legend`.
func _draw_track(pts: PackedVector3Array, center: Vector2, ppl: float,
		col: Color, id: String) -> void:
	var out_col := Color(col.r, col.g, col.b, col.a * 0.4)
	for run: Array in _track_runs(pts, center, ppl, id):
		draw_polyline(run[0], out_col if run[1] else col, 1.1)


## A track's drawable segments, grouped into the longest runs that share a colour
## and are on frame, and remembered until the zoom, the size or the tracks change.
##
## One `draw_polyline` per run rather than one `draw_line` per segment: a ~1400
## point track was 1400 canvas commands per track per frame, and this view redraws
## every frame because things blink and the clock runs.
##
## **Not one polyline per track.** The two requirements the plan stated together do
## not compose that way — a single line would have to include the off-frame points
## to stay connected, which is the reject undone. A run breaks on either of the two
## things that make a segment different from its neighbour: it leaves the grown
## rect, or it crosses the b-plane (`s` changes sign, which is what dims the
## outbound half). Same segments, same colours, ~4 commands instead of 1400.
##
## Cached on (zoom, size) and dropped whenever `_fetch` re-reads the tracks — which
## is the complete set of things that can change the geometry, since `_nom` and
## `_defl` are assigned nowhere else.
func _track_runs(pts: PackedVector3Array, center: Vector2, ppl: float,
		id: String) -> Array:
	# Keyed by which track it is, not by the points themselves: a
	# PackedVector3Array as a dictionary key would hash ~1400 vectors on every
	# lookup, which is the per-frame cost this is here to remove.
	var key := "%.9f|%.0f|%.0f" % [_half_ld, size.x, size.y]
	if _runs_key != key:
		_runs_key = key
		_runs.clear()
	var hit: Variant = _runs.get(id)
	if hit != null:
		return hit
	var runs: Array = []
	if pts.size() >= 2:
		var rect := Rect2(Vector2.ZERO, size).grow(400.0)
		var prev := _plot(center, ppl, pts[0])
		var cur_pts := PackedVector2Array()
		var cur_out := false
		for k in range(1, pts.size()):
			var cur := _plot(center, ppl, pts[k])
			var out: bool = pts[k].z > 0.0
			# Cheap reject: at this zoom most of a track is far off-frame.
			if rect.has_point(cur) or rect.has_point(prev):
				if cur_pts.is_empty() or out != cur_out:
					if cur_pts.size() >= 2:
						runs.append([cur_pts, cur_out])
					cur_pts = PackedVector2Array([prev])
					cur_out = out
				cur_pts.append(cur)
			elif cur_pts.size() >= 2:
				runs.append([cur_pts, cur_out])
				cur_pts = PackedVector2Array()
			else:
				cur_pts = PackedVector2Array()
			prev = cur
		if cur_pts.size() >= 2:
			runs.append([cur_pts, cur_out])
	_runs[id] = runs
	return runs


## The encounter solution. Only quantities the core actually pins: |B| and the
## capture radius (the pair the verdict compares), the perigee (labelled as the
## separate thing it is), v_inf, and — now that the core has settled it — which
## frame the axes are.
func _readout(w: float, h: float, mid: Color, bright: Color, dim: Color) -> void:
	var lh := _fs + 5.0
	var ry := h * 0.56
	draw_string(_font, Vector2(MARGIN, ry), "-- ENCOUNTER SOLUTION " + "-".repeat(12),
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, Color(0.25, 0.25, 0.25))

	var p_nom: float = Sim.perigee_ld(false)
	var lines := [
		["FRAME", "OPIK XI/ZETA - ZETA OPPOSES EARTH MOTION" if Sim.bplane_frame_pinned()
			else "GEOCENTRIC B-PLANE (DISPLAY AXES)"],
		["V-INF", "%.2f KM/S" % Sim.encounter_v_inf_kms()],
		["NOM |B|", "%.4f LD  (PERIGEE %s)" %
			[Sim.nominal_b_ld(), "%.4f LD" % p_nom if p_nom >= 0.0 else "--"]],
		["DEFL |B|", Sim.miss_label(true)],
		["CAPTURE", "%.4f LD  (%.2f RE)" % [Sim.cap_km / Sim.LD_KM, Sim.cap_km / Sim.R_E]],
	]
	for k in lines.size():
		draw_string(_font, Vector2(MARGIN, ry + (k + 1) * lh),
			"%-9s %s" % [lines[k][0], lines[k][1]],
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)

	# The verdict, from Sim — never re-derived here. Two panels deciding hit-vs-miss
	# from the same numbers is how they start disagreeing.
	var vy := ry + (lines.size() + 1) * lh
	var steady: bool = Sim.deflect_ok or Sim.plan_solving or not Sim.has_plan()
	if steady or Sim.blink(1.4):
		draw_string(_font, Vector2(MARGIN, vy), "SOLUTION: " + Sim.verdict_label(),
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, bright)
	# Why the disc is bigger than the planet, said once, where it is being used.
	draw_string(_font, Vector2(MARGIN, vy + lh),
		"HIT WHEN |B| <= CAPTURE - EARTH'S GRAVITY WIDENS ITS OWN TARGET",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)


## The Tier-3 uncertainty ellipse, in the same `(xi, zeta)` km this whole view is
## drawn in — the core rotates it there, because an ellipse's *orientation* is the
## one thing about it that is not invariant under the b-plane frame the
## sensitivity happens to be solved in.
##
## **The ellipse is normally far smaller than a pixel, and that is the finding,
## not a bug.** At the default span (0.15 LD, ~58 000 km across) a ~170 km
## 1-sigma major axis is about a pixel and the minor axis is a hundredth of one.
## Fattening it to something visible would be drawing a spread the orbit does not
## have; instead the axes are printed in kilometres unconditionally and the
## overlay says outright when the shape is below the resolution of the screen.
## Zooming in with the wheel is what makes it appear, which teaches the right
## thing: this rock's crossing is known to a few hundred kilometres inside a
## capture disc eleven thousand kilometres wide.
##
## Centred on the sensitivity's **own** mean, which is not the cross the view
## draws. Both are honest reductions of one hyperbola — the cross at closest
## approach, the ellipse at the fixed epoch its Jacobian was differenced around —
## and pairing them would be centring one instrument's spread on another
## instrument's position. **Measured: they are 5.43 km apart, against a 0.82 km
## minor axis** — so that is not a rounding, it is six and a half ellipse-widths,
## and the readout prints it rather than leaving the reader to assume the cross is
## the centre.
func _draw_uncertainty(center: Vector2, ppl: float, bright: Color, mid: Color,
		dim: Color) -> void:
	if Sim.tier3_solving:
		_centered("SOLVING B-PLANE SENSITIVITY - 13 PROPAGATIONS",
			Vector2(size.x * 0.5, 60.0), dim, _fs - 2)
		return
	if not Sim.tier3_online or Sim.tier3.is_empty():
		return

	var e: Dictionary = Sim.tier3
	var mean := Vector3(float(e["mean_xi_km"]), float(e["mean_zeta_km"]), 0.0)
	var at := _plot(center, ppl, mean)
	var major_px: float = float(e["major_km"]) / Sim.LD_KM * ppl
	var minor_px: float = float(e["minor_km"]) / Sim.LD_KM * ppl
	var a: float = float(e["angle_rad"])

	# Below a pixel there is no shape to draw. A ring marks where it is and the
	# caption says why nothing is inside it, rather than leaving the operator to
	# conclude the overlay is broken.
	if major_px < 1.5:
		draw_arc(at, 5.0, 0.0, TAU, 24, Color(mid, 0.7), 1.0)
		draw_string(_font, at + Vector2(9, -6), "1-SIGMA < 1 PX - ZOOM IN",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 3, dim)
		return

	# The ellipse. Built in screen space from the two semi-axes: xi is +x and zeta
	# is +y here (see `_plot`), so the core's angle-from-xi is the screen angle too.
	var u := Vector2(cos(a), sin(a))
	var v := Vector2(-u.y, u.x)
	var pts := PackedVector2Array()
	for k in 65:
		var th: float = TAU * float(k) / 64.0
		pts.push_back(at + u * (major_px * cos(th)) + v * (minor_px * sin(th)))
	draw_polyline(pts, Color(bright, 0.85), 1.2)
	# Captioned at the end of the major axis, always. This ellipse is 205:1, so at
	# any zoom where it fits on screen it is a *line* — and an unlabelled short line
	# in a picture full of tracks and circles reads as an artifact, not as the thing
	# the overlay was opened to see. The label goes on the end rather than beside
	# the centre because the centre is inside the capture disc's fill.
	# Under the LOWER end of the needle, not through `_label_at`. That helper places
	# a caption radially outward from Earth, which is right for a mark on its own
	# and wrong here: the ellipse's centre and the nominal cross are 5.43 km apart —
	# a fifth of a pixel — so both captions would be pushed to the same spot and
	# the first screenshot of this had them printed on top of each other. The
	# b-point labels go radially out; this one goes down.
	var tip_a := at + u * major_px
	var tip_b := at - u * major_px
	var tip: Vector2 = tip_a if tip_a.y > tip_b.y else tip_b
	var txt := "1-SIGMA %s KM" % String.num(float(e["major_km"]), 1)
	draw_string(_font, _clamp_to_plot(tip + Vector2(8, 14)), txt,
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 3, Color(mid, 0.9))
	# The centre, only once it is distinguishable from the ellipse around it.
	if major_px > 8.0:
		draw_line(at + Vector2(-3, 0), at + Vector2(3, 0), Color(mid, 0.8), 1.0)
		draw_line(at + Vector2(0, -3), at + Vector2(0, 3), Color(mid, 0.8), 1.0)


## The uncertainty readout — printed whenever the overlay is on, whether or not
## the ellipse is large enough to see, because the numbers are the part that
## survives the zoom level.
##
## `P(IMPACT)` and the capture radius are printed **together and never apart**.
## The Mahalanobis distance the core can also report is deliberately absent: on
## its own it reads as "how many sigma from a hit" and inverts the answer — the
## designed hit sits thousands of ellipse-widths from Earth's centre and still has
## P = 1, because all of that is inside an 11 000 km disc.
func _draw_uncertainty_readout(x: float, y: float, lh: float, mid: Color,
		dim: Color) -> float:
	if not _uncertainty:
		return y
	draw_string(_font, Vector2(x, y), "-- ORBIT UNCERTAINTY (TIER 3) " + "-".repeat(6),
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, Color(0.25, 0.25, 0.25))
	var row := y + lh
	if Sim.tier3_solving:
		draw_string(_font, Vector2(x, row), "%-9s %s" % ["STATUS", "SOLVING - 13 PROPAGATIONS"],
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)
		return row + lh
	if not Sim.tier3_online or Sim.tier3.is_empty():
		draw_string(_font, Vector2(x, row), "%-9s %s" % ["STATUS", "NOT SOLVED - PRESS [U]"],
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, dim)
		return row + lh

	var e: Dictionary = Sim.tier3
	var lines := [
		# The word "synthetic" is not decoration. This rock is invented, so its
		# covariance is a shape borrowed from real NEOs and not a measurement of
		# anything; an ellipse drawn without saying so is a claim about how well this
		# asteroid is tracked, and nobody tracks it.
		["COVAR", "SYNTHETIC - ALONG-TRACK 20:1 (INVENTED ROCK)"],
		["1-SIGMA", "%s x %s KM AT %s DEG FROM XI" % [
			String.num(float(e["major_km"]), 1), String.num(float(e["minor_km"]), 2),
			String.num(rad_to_deg(float(e["angle_rad"])), 1)]],
		["P(IMPACT)", "%s  OVER THE %s KM CAPTURE DISC" % [
			String.num(float(e["p_impact"]), 6), String.num(float(e["capture_km"]), 0)]],
		["KNOWN TO", "x%s OF NOMINAL   [Z] BETTER / [X] WORSE" %
			String.num(float(e["sigma_scale"]), 4)],
		["SCOPE", "NOMINAL TRACK ONLY - NO DEFLECTED SPREAD"],
	]
	# Where the ellipse sits relative to the cross. Printed because the two are
	# different reductions of one hyperbola and the gap between them is several
	# minor axes wide — small on screen, and not small compared to the shape it is
	# next to. Omitted when the core could not report it (no nominal b-point).
	if e.has("mean_gap_km"):
		lines.append(["CENTRE", "%s KM FROM THE NOMINAL CROSS (FIXED-EPOCH REDUCTION)" %
			String.num(float(e["mean_gap_km"]), 2)])
	for k in lines.size():
		draw_string(_font, Vector2(x, row + k * lh), "%-9s %s" % [lines[k][0], lines[k][1]],
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)
	return row + lines.size() * lh


# --------------------------------------------------------------- helpers ---

## (xi, zeta, s) km -> screen. Only xi/zeta place the point; s is depth into the
## picture, and the view uses it for shading, not position.
func _plot(center: Vector2, ppl: float, p: Vector3) -> Vector2:
	return center + Vector2(p.x, p.y) / Sim.LD_KM * ppl


## Place a mark's caption radially outward from Earth, so captions separate the way
## the marks do instead of collecting in the crowded middle. Kept on-screen.
func _label_at(p: Vector2, center: Vector2, text: String) -> Vector2:
	var dir := (p - center).normalized() if p.distance_to(center) > 1.0 else Vector2.RIGHT
	var tw := _font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs).x
	var at := p + dir * 14.0 + Vector2(0, -5)
	# Flip a right-side caption to the left of its mark rather than off the frame.
	if at.x + tw > size.x - MARGIN:
		at.x = p.x - tw - 14.0
	return at.clamp(Vector2(MARGIN, 60.0), size - Vector2(tw + MARGIN, 60.0))


## The box a `draw_string` at `at` will fill. `draw_string` takes a baseline-left
## origin, so the box starts an ascent above it — get that wrong and every
## reservation sits a line away from the text it is reserving for.
func _text_rect(at: Vector2, text: String, fs: int) -> Rect2:
	return Rect2(at.x, at.y - _font.get_ascent(fs),
		_font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, fs).x,
		_font.get_height(fs))


func _overlaps(r: Rect2, rects: Array[Rect2]) -> bool:
	for other: Rect2 in rects:
		if r.intersects(other):
			return true
	return false


func _on_plot(p: Vector2) -> bool:
	return Rect2(Vector2.ZERO, size).grow(-MARGIN).has_point(p)


func _clamp_to_plot(p: Vector2) -> Vector2:
	var r := Rect2(Vector2.ZERO, size).grow(-MARGIN - 6.0)
	return Vector2(clampf(p.x, r.position.x, r.end.x), clampf(p.y, r.position.y, r.end.y))


func _centered(s: String, at: Vector2, col: Color, fs: int) -> void:
	var tw := _font.get_string_size(s, HORIZONTAL_ALIGNMENT_LEFT, -1, fs).x
	draw_string(_font, Vector2(at.x - tw * 0.5, at.y), s,
		HORIZONTAL_ALIGNMENT_LEFT, -1, fs, col)


func _dashed_circle(c: Vector2, r: float, col: Color) -> void:
	for k in range(0, 48, 2):
		draw_arc(c, r, TAU * k / 48.0, TAU * (k + 1) / 48.0, 6, col, 1.0)


func _dashed_line(a: Vector2, b: Vector2, col: Color, dash: float, gap: float) -> void:
	var dir := b - a
	var len_ := dir.length()
	if len_ < 0.001:
		return
	dir /= len_
	var d := 0.0
	while d < len_:
		var e := minf(d + dash, len_)
		draw_line(a + dir * d, a + dir * e, col, 1.0)
		d = e + gap


func _diamond(p: Vector2, r: float, col: Color) -> void:
	draw_polyline(PackedVector2Array([
		p + Vector2(0, -r), p + Vector2(r, 0), p + Vector2(0, r),
		p + Vector2(-r, 0), p + Vector2(0, -r)]), col, 1.2)


## A radar contact: filled dot inside a ring. Reserved for the live asteroid, so
## "a thing that is there right now" never wears the same glyph as "where a line
## crosses this plane" (see _draw_marker).
func _contact(p: Vector2, r: float, col: Color) -> void:
	draw_circle(p, r * 0.55, col)
	draw_arc(p, r * 1.7, 0.0, TAU, 20, col, 1.2)


func _cross(p: Vector2, r: float, col: Color) -> void:
	draw_line(p + Vector2(-r, -r), p + Vector2(r, r), col, 1.5)
	draw_line(p + Vector2(-r, r), p + Vector2(r, -r), col, 1.5)
