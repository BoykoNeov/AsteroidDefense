class_name EncounterView
extends Control
## Earth-encounter close-up: geocentric plot in the encounter b-plane.
## Axes are the classic targeting axes built from the nominal inbound
## relative velocity: S = v_rel normalized, XI = S x N (N = ecliptic
## north), ZETA = S x XI (points roughly south; drawn screen-down).
## Display-grade like the rest of Sim: S is the CA-epoch relative velocity
## rather than the hyperbolic asymptote, and tracks are heliocentric Kepler
## differences (no Earth-gravity bend) — the capture circle carries the
## gravitational-focusing story instead. Mouse wheel zooms.

const WINDOW_D := 40.0                 # track half-window around CA, days
const SAMPLES := 320
const V_ESC := 11.186                  # Earth surface escape speed, km/s
const R_E := 6371.0                    # km
const MARGIN := 18.0

var _font: Font
var _fs := 13
var _half_ld := 4.5                    # plot half-span, lunar distances
var _built := false

var _s_hat: Vector3
var _xi_hat: Vector3
var _ze_hat: Vector3
var _v_inf := 0.0
var _cap_km := 0.0                     # gravitational capture radius
var _nom := {}
var _defl := {}


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_preset(Control.PRESET_FULL_RECT)
	_font = Sim.mono_font
	# Deflected track and b-point depend on the mission plan.
	Sim.plan_changed.connect(func() -> void: _built = false)


func _process(_delta: float) -> void:
	if visible:
		queue_redraw()


func _unhandled_input(event: InputEvent) -> void:
	# Wheel zooms the plot span; swallow it so the 3D rig underneath
	# doesn't zoom too. (Added after the camera rig, so we see it first.)
	if not visible:
		return
	if event is InputEventMouseButton and event.pressed:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP:
			_half_ld = clampf(_half_ld * 0.8, 0.05, 30.0)
			get_viewport().set_input_as_handled()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_half_ld = clampf(_half_ld * 1.25, 0.05, 30.0)
			get_viewport().set_input_as_handled()


# ------------------------------------------------------------- solutions ---

## Solve the b-plane geometry for both tracks. Requires the mission layer: every
## input here is threat-shaped, and while it is dormant `ast_el` is empty. main.gd
## refuses to show this view in that state; the guard makes the dependency
## explicit rather than incidental, since an empty dict would fail deep inside
## close_approach with nothing pointing back here.
func _build() -> void:
	if not Sim.mission_online:
		return
	var ca_n: Dictionary = Sim.close_approach(Sim.ast_el)
	var ca_d: Dictionary = Sim.close_approach(Sim.ast_defl_el)
	var v: Vector3 = ca_n.v_kms
	_s_hat = v.normalized()
	_xi_hat = _s_hat.cross(Vector3(0, 0, 1)).normalized()
	_ze_hat = _s_hat.cross(_xi_hat)
	_v_inf = v.length()
	_cap_km = R_E * sqrt(1.0 + pow(V_ESC / _v_inf, 2.0))
	_nom = _solve_track(Sim.ast_el, ca_n)
	_defl = _solve_track(Sim.ast_defl_el, ca_d)
	_built = true


## Project geocentric km -> (xi, zeta, s) km.
func _project(g: Vector3) -> Vector3:
	return Vector3(g.dot(_xi_hat), g.dot(_ze_hat), g.dot(_s_hat))


func _solve_track(el: Dictionary, ca: Dictionary) -> Dictionary:
	var t0: float = ca.t - WINDOW_D
	var dt := 2.0 * WINDOW_D / SAMPLES
	var pts := PackedVector3Array()
	for k in SAMPLES + 1:
		pts.append(_project(Sim.geo_km(el, t0 + k * dt)))
	# B-vector: bisect the b-plane crossing (s = 0) between samples.
	var b := Vector2.ZERO
	var found := false
	for k in SAMPLES:
		if pts[k].z <= 0.0 and pts[k + 1].z > 0.0:
			var lo := t0 + k * dt
			var hi := lo + dt
			for _i in 48:
				var m := (lo + hi) * 0.5
				if _project(Sim.geo_km(el, m)).z <= 0.0:
					lo = m
				else:
					hi = m
			var p := _project(Sim.geo_km(el, (lo + hi) * 0.5))
			b = Vector2(p.x, p.y)
			found = true
			break
	return {"ca": ca, "t0": t0, "dt": dt, "pts": pts, "b": b, "b_found": found}


# ------------------------------------------------------------------ draw ---

func _draw() -> void:
	if not _built:
		_build()
	var w := size.x
	var h := size.y
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.004, 0.006, 0.005), true)
	var center := Vector2(w * 0.5, h * 0.5)
	var ppl := minf(w, h) * 0.5 / _half_ld * 0.92      # px per lunar distance
	var bright := Color(1, 1, 1)
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.42, 0.42, 0.42)
	var faint := Color(0.18, 0.18, 0.18)
	var t: float = Sim.t
	var post: bool = Sim.burned()
	var preview: bool = not post and (Sim.planner_open or Sim.committed)

	# Targeting axes.
	draw_line(Vector2(0, center.y), Vector2(w, center.y), faint, 1.0)
	draw_line(Vector2(center.x, 0), Vector2(center.x, h), faint, 1.0)
	draw_string(_font, Vector2(w - MARGIN - 34, center.y - 6), "+XI",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)
	draw_string(_font, Vector2(center.x + 6, h - MARGIN - 30), "+ZETA",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)

	# Range rings, lunar distances.
	for r: float in [0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0]:
		var rp: float = r * ppl
		if rp < 26.0 or rp > minf(w, h) * 0.52:
			continue
		draw_arc(center, rp, 0, TAU, 160, faint, 1.0)
		var lbl := String.num(r) + " LD" + (" - LUNAR DIST" if r == 1.0 else "")
		draw_string(_font, center + Vector2(rp * 0.7071 + 5, -rp * 0.7071 - 4),
			lbl, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)

	# Earth + gravitational capture cross-section.
	var re_px := maxf(R_E / Sim.LD_KM * ppl, 3.0)
	draw_circle(center, re_px, Color(0.09, 0.09, 0.09))
	draw_arc(center, re_px, 0, TAU, 96, bright, 1.4)
	draw_string(_font, center + Vector2(re_px + 7, 4), "EARTH",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)
	var cap_px := _cap_km / Sim.LD_KM * ppl
	if cap_px > re_px + 5.0:
		_dashed_circle(center, cap_px, dim)
		draw_string(_font, center + Vector2(cap_px * 0.7071 + 5, cap_px * 0.7071 + 12),
			"CAPTURE %.1f RE" % (_cap_km / R_E),
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)

	# Tracks: nominal always; deflected once the burn has happened, or as a
	# planned-solution preview while the planner is open / plan committed.
	_draw_track(_nom, center, ppl, dim if post else mid,
		"NOMINAL TRK" if post else "2031-XK INBOUND")
	if post:
		_draw_track(_defl, center, ppl, mid, "2031-XK DEFLECTED")
	elif preview:
		_draw_track(_defl, center, ppl, Color(1, 1, 1, 0.5), "PLANNED TRK")
		if _defl.b_found:
			var bplan: Vector2 = center + _defl.b / Sim.LD_KM * ppl
			_dashed_line(center, bplan, dim, 6.0, 5.0)
			_diamond(bplan, 6.0, mid)
			draw_string(_font, bplan + Vector2(10, -6),
				"PLAN B %.2f LD" % (_defl.b.length() / Sim.LD_KM),
				HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, mid)

	# B-vector markers.
	if _nom.b_found:
		var bp: Vector2 = center + _nom.b / Sim.LD_KM * ppl
		if post:
			_cross(bp, 5.0, dim)
		elif Sim.blink(1.6):
			_cross(bp, 8.0, bright)
			draw_string(_font, bp + Vector2(14, -16), "PREDICTED IMPACT",
				HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, bright)
	if post and _defl.b_found:
		var bd: Vector2 = center + _defl.b / Sim.LD_KM * ppl
		_dashed_line(center, bd, dim, 6.0, 5.0)
		_diamond(bd, 6.0, bright)
		draw_string(_font, bd + Vector2(10, -6),
			"B %.2f LD" % (_defl.b.length() / Sim.LD_KM),
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, bright)

	# Live asteroid marker when inside the sampled window.
	var trk: Dictionary = _defl if post else _nom
	var el_act: Dictionary = Sim.ast_defl_el if post else Sim.ast_el
	var ca_t: float = trk.ca.t
	if absf(t - ca_t) <= WINDOW_D:
		var p := _project(Sim.geo_km(el_act, t))
		var pp: Vector2 = center + Vector2(p.x, p.y) / Sim.LD_KM * ppl
		if Rect2(Vector2.ZERO, size).grow(20.0).has_point(pp):
			_diamond(pp, 7.0, bright if Sim.blink(2.2) else mid)
			draw_string(_font, pp + Vector2(11, 5),
				"2031-XK  CA %+.1f D" % (t - ca_t),
				HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)

	# Header.
	var hdr := "EARTH ENCOUNTER - B-PLANE VIEW"
	var hw := _font.get_string_size(hdr, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs).x
	draw_string(_font, Vector2(w * 0.5 - hw * 0.5, 40), hdr,
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, dim)

	# Encounter-solution readout (active track).
	var b_km: float = trk.b.length() if trk.b_found else trk.ca.r_km
	var lh := _fs + 5.0
	var ry := h * 0.56
	draw_string(_font, Vector2(MARGIN, ry), "-- ENCOUNTER SOLUTION " + "-".repeat(12),
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, Color(0.25, 0.25, 0.25))
	var lines := [
		["FRAME", "GEOCENTRIC XI/ZETA"],
		["V-REL", "%.2f KM/S" % _v_inf],
		["CA", "E%+.1f D" % (ca_t - Sim.T_IMPACT)],
		["B-XI", "%+.3f LD" % (trk.b.x / Sim.LD_KM)],
		["B-ZETA", "%+.3f LD" % (trk.b.y / Sim.LD_KM)],
		["|B|", "%.3f LD (%d KM)" % [b_km / Sim.LD_KM, int(b_km)]],
		["CAPTURE", "%.3f LD (%.1f RE)" % [_cap_km / Sim.LD_KM, _cap_km / R_E]],
	]
	for k in lines.size():
		draw_string(_font, Vector2(MARGIN, ry + (k + 1) * lh),
			"%-8s %s" % [lines[k][0], lines[k][1]],
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)
	var vy := ry + (lines.size() + 1) * lh
	if b_km < _cap_km:
		if Sim.blink(1.4):
			draw_string(_font, Vector2(MARGIN, vy), "SOLUTION: SURFACE IMPACT",
				HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, bright)
	else:
		draw_string(_font, Vector2(MARGIN, vy), "SOLUTION: MISS - EARTH CLEAR",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, bright)

	# Footer scale/help.
	var foot := "SPAN +/-%.1f LD   [WHEEL] ZOOM" % _half_ld
	var fw := _font.get_string_size(foot, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2).x
	draw_string(_font, Vector2(w * 0.5 - fw * 0.5, h - MARGIN - 4), foot,
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)


## Track polyline: full brightness inbound (s < 0), dimmed after the
## b-plane crossing; day ticks every 5 d.
func _draw_track(trk: Dictionary, center: Vector2, ppl: float, col: Color,
		label: String) -> void:
	var pts: PackedVector3Array = trk.pts
	var out_col := Color(col.r, col.g, col.b, col.a * 0.45)
	var scr := PackedVector2Array()
	scr.resize(pts.size())
	for k in pts.size():
		scr[k] = center + Vector2(pts[k].x, pts[k].y) / Sim.LD_KM * ppl
	for k in pts.size() - 1:
		draw_line(scr[k], scr[k + 1], col if pts[k].z <= 0.0 else out_col, 1.2)
	# Day ticks (every 20 samples = 5 d at the default sampling).
	for k in range(0, pts.size() - 1, 20):
		var d := (scr[k + 1] - scr[k]).normalized()
		var n := Vector2(-d.y, d.x) * 3.0
		draw_line(scr[k] - n, scr[k] + n, col, 1.0)
	# Label near the inbound end, kept on-screen.
	var lp := scr[0].clamp(Vector2(MARGIN, 60.0), size - Vector2(190.0, 60.0))
	draw_string(_font, lp + Vector2(8, -6), label,
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, col)


# --------------------------------------------------------------- helpers ---

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
