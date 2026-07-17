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
## The axes are a **display** frame and are labelled as one. The core deliberately
## leaves the Öpik/Kizner xi/zeta decomposition and B's sign unpinned (a Tier-3
## keyhole question), so nothing here prints signed components under those names —
## only the rotation-invariant scalars this view is entitled to.

## Plot half-span at open, lunar distances. Sized for the capture disc (0.029 LD),
## not for the LD-scale rings the heliocentric views use: this is the one frame
## where the whole story happens inside a tenth of a lunar distance. The b-plane
## projection cooperates — it drops the huge `s` component, so an inbound track
## that reaches ~10^6 km down-range still sits within ~|B| of the origin here.
const DEFAULT_HALF_LD := 0.15
const MARGIN := 18.0

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
	_built = true


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

	_draw_rings(center, ppl, w, h, faint, dim)
	_draw_earth_and_disc(center, ppl, bright, mid, dim)

	# Tracks: context. Dim, and behind the b-points that actually decide things.
	_draw_track(_nom, center, ppl, Color(0.5, 0.5, 0.5))
	if not _defl.is_empty():
		# Faint while a solve is pending: this is still the previous plan's arc.
		var a: float = 0.3 if Sim.plan_solving else 0.75
		_draw_track(_defl, center, ppl, Color(0.62, 0.62, 0.62, a))

	_draw_b_points(center, ppl, bright, mid, dim)
	_draw_marker(center, ppl, bright, mid)
	_draw_legend(w, mid, dim)

	# Header.
	_centered("EARTH ENCOUNTER - B-PLANE VIEW", Vector2(w * 0.5, 40), dim, _fs)
	_readout(w, h, mid, bright, dim)

	var foot := "SPAN +/-%s LD   [WHEEL] ZOOM" % String.num(_half_ld, 3)
	_centered(foot, Vector2(w * 0.5, h - MARGIN - 4), dim, _fs - 2)


## Range rings in lunar distances, auto-selected for the current zoom so the
## close-up (0.15 LD) and a wide view (10 LD) both get sensible labels.
##
## Nothing is drawn inside the capture disc. A ring there would be a second
## distance reference inside the only one that decides anything, and its label
## would land on Earth — the clutter that made the first draft unreadable.
func _draw_rings(center: Vector2, ppl: float, w: float, h: float,
		faint: Color, dim: Color) -> void:
	var floor_px: float = Sim.cap_km / Sim.LD_KM * ppl + 10.0
	for r: float in [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0]:
		var rp: float = r * ppl
		if rp < floor_px or rp > minf(w, h) * 0.52:
			continue
		draw_arc(center, rp, 0, TAU, 160, faint, 1.0)
		var lbl := String.num(r, 2) + " LD" + (" - LUNAR DIST" if r == 1.0 else "")
		draw_string(_font, center + Vector2(rp * 0.7071 + 5, -rp * 0.7071 - 4),
			lbl, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)


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
	if not _defl.is_empty():
		_diamond(Vector2(x + 6, y + lh - 4), 4.0, mid)
		draw_string(_font, Vector2(x + 18, y + lh), "DEFLECTED - PLANNED",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, mid)
	draw_string(_font, Vector2(x, y + 2 * lh),
		"MARKS = ASYMPTOTE THROUGH THIS PLANE",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)
	draw_string(_font, Vector2(x, y + 3 * lh),
		"LINES = TRACK (BENDS - CONTEXT ONLY)",
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
		dim: Color) -> void:
	# Nominal: inside the disc, by construction. This is the hit — and it stays the
	# prediction until the burn actually happens, not merely until a plan is drawn
	# up. A solved plan sitting beside a blinking PREDICTED IMPACT is the whole
	# comparison this view exists to make.
	if _b_nom != Vector3.ZERO:
		var p := _plot(center, ppl, _b_nom)
		var clamped: bool = not _on_plot(p)
		p = _clamp_to_plot(p)
		if Sim.burned():
			_cross(p, 5.0, dim)
		elif Sim.blink(1.6):
			_cross(p, 8.0, bright)
			draw_string(_font, _label_at(p, center, "PREDICTED IMPACT"),
				"PREDICTED IMPACT" + (" >" if clamped else ""),
				HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, bright)

	# Deflected: ZERO means there is no such point — no plan, or a clean miss that
	# left the encounter entirely. Both must draw nothing; ZERO is Earth's centre.
	if _b_defl == Vector3.ZERO:
		return
	var pd := _plot(center, ppl, _b_defl)
	var off: bool = not _on_plot(pd)
	pd = _clamp_to_plot(pd)
	# While a solve is pending this mark still belongs to the PREVIOUS plan, so it
	# is drawn faint: the operator has moved on and this has not caught up. The
	# label says so too (`miss_label` reports SOLVING...), but a confidently-drawn
	# diamond in the wrong place is the part a player would believe.
	var solving: bool = Sim.plan_solving
	_dashed_line(center, pd, Color(dim, dim.a * (0.4 if solving else 1.0)), 6.0, 5.0)
	_diamond(pd, 6.0, Color(bright, 0.35 if solving else 1.0))
	# Through Sim's formatter, like every other site that prints a miss.
	var txt := "B " + Sim.miss_label() + (" >" if off else "")
	draw_string(_font, _label_at(pd, center, txt), txt,
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, Color(bright, 0.5 if solving else 1.0))


## The live asteroid, when the clock is actually inside the encounter window.
##
## The window is ±1.5 days; the campaign is twelve years. So this is absent almost
## always, and it must be *absent* — the same gate `threat_active` applies to the
## orrery. Clamping the clock onto the nearest end of the track instead would park
## a marker at the frame edge and call it the asteroid's position.
func _draw_marker(center: Vector2, ppl: float, bright: Color, mid: Color) -> void:
	if _span.size() != 2 or _nom.is_empty():
		return
	var t: float = Sim.t
	if t < _span[0] or t > _span[1]:
		return
	# The track that is real at this moment: the deflected one only after the burn.
	var trk: PackedVector3Array = _defl if (Sim.burned() and not _defl.is_empty()) else _nom
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
	_diamond(p, 7.0, bright if Sim.blink(2.2) else mid)
	draw_string(_font, p + Vector2(11, 5), "2031-XK  CA %+.2f D" % (t - Sim.T_IMPACT),
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)


## A track's (xi, zeta) shadow. Full brightness inbound (s < 0), dimmed once past
## the b-plane — the s component the projection carries, put to work.
##
## Unlabelled by design; see `_draw_legend`.
func _draw_track(pts: PackedVector3Array, center: Vector2, ppl: float,
		col: Color) -> void:
	if pts.size() < 2:
		return
	var out_col := Color(col.r, col.g, col.b, col.a * 0.4)
	var prev := _plot(center, ppl, pts[0])
	var rect := Rect2(Vector2.ZERO, size).grow(400.0)
	for k in range(1, pts.size()):
		var cur := _plot(center, ppl, pts[k])
		# Cheap reject: at this zoom most of a track is far off-frame.
		if rect.has_point(cur) or rect.has_point(prev):
			draw_line(prev, cur, col if pts[k].z <= 0.0 else out_col, 1.1)
		prev = cur


## The encounter solution. Only quantities the core actually pins: |B| and the
## capture radius (the pair the verdict compares), the perigee (labelled as the
## separate thing it is), and v_inf. No signed xi/zeta — the core has not settled
## that convention, so printing components under those names would be inventing one.
func _readout(w: float, h: float, mid: Color, bright: Color, dim: Color) -> void:
	var lh := _fs + 5.0
	var ry := h * 0.56
	draw_string(_font, Vector2(MARGIN, ry), "-- ENCOUNTER SOLUTION " + "-".repeat(12),
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, Color(0.25, 0.25, 0.25))

	var p_nom: float = Sim.perigee_ld(false)
	var lines := [
		["FRAME", "GEOCENTRIC B-PLANE (DISPLAY AXES)"],
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


func _cross(p: Vector2, r: float, col: Color) -> void:
	draw_line(p + Vector2(-r, -r), p + Vector2(r, r), col, 1.5)
	draw_line(p + Vector2(-r, r), p + Vector2(r, -r), col, 1.5)
