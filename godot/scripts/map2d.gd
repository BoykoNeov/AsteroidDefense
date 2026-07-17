class_name Map2D
extends Control
## Top-down 2D tactical plot: range rings, orbit traces, body markers,
## Earth-threat range line, radar sweep. High-detail vector drawing —
## the retro feel comes from style, not resolution.

var _font: Font
var _fs := 13
var _sweep := 0.0


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_preset(Control.PRESET_FULL_RECT)
	_font = Sim.mono_font


func _process(delta: float) -> void:
	if visible:
		_sweep = fmod(_sweep + delta * 0.7, TAU)
		queue_redraw()


## Ecliptic AU -> screen px. Top-down view from ecliptic north.
func _to_screen(ecl: Vector3, center: Vector2, s: float) -> Vector2:
	return center + Vector2(ecl.x, -ecl.y) * s


func _draw() -> void:
	var w := size.x
	var h := size.y
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.004, 0.006, 0.005), true)
	var center := Vector2(w * 0.5, h * 0.5)
	var s := minf(w, h) * 0.5 / 1.85          # px per AU, plot to 1.85 AU
	var bright := Color(1, 1, 1)
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.40, 0.40, 0.40)
	var faint := Color(0.18, 0.18, 0.18)
	var t: float = Sim.t

	# Range rings + labels.
	for ring in range(1, 8):
		var r := 0.25 * ring
		draw_arc(center, r * s, 0, TAU, 128, faint, 1.0)
		if ring % 2 == 0:
			draw_string(_font, center + Vector2(r * s + 4, -3), "%.1f AU" % r,
				HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)
	# Spokes.
	for k in 12:
		var a := TAU * k / 12.0
		var d := Vector2(cos(a), sin(a))
		draw_line(center + d * 0.25 * s, center + d * 1.85 * s, faint, 1.0)

	# Radar sweep: fading trail of arc segments behind the beam.
	for seg in 28:
		var a0 := _sweep - seg * 0.035
		var alpha := 0.10 * (1.0 - seg / 28.0)
		draw_line(center, center + Vector2(cos(a0), sin(a0)) * 1.85 * s,
			Color(1, 1, 1, alpha), 2.0)

	# Without a field there are no positions to plot, and drawing anyway would be
	# the worst version of it: pos_ecl returns ZERO, _to_screen(ZERO) is the plot
	# centre, so every planet marker and label would stack neatly on the Sun and
	# read as a real (catastrophic) plot rather than as missing data.
	if not Sim.bodies_online:
		draw_string(_font, center + Vector2(-96, 40), "NO EPHEMERIS - FIELD OFFLINE",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, mid)
		return

	# Orbit traces.
	for el in Sim.planets:
		if el.a > 2.0:
			continue                          # Jupiter off-plot
		_orbit_trace(el, center, s, dim if el.name == "EARTH" else faint)
	if Sim.mission_online:
		_orbit_trace(Sim.ast_el, center, s, mid)
		if Sim.burned():
			_orbit_trace(Sim.ast_defl_el, center, s, dim, true)
		elif Sim.planner_open or Sim.committed:
			_orbit_trace(Sim.ast_defl_el, center, s, faint, true)

	# Sun.
	draw_circle(center, 4.0, bright)
	draw_string(_font, center + Vector2(8, 4), "SOL",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, dim)

	# Planets.
	for el in Sim.planets:
		if el.a > 2.0:
			continue
		var p := _to_screen(Sim.pos_ecl(el, t), center, s)
		draw_arc(p, 5.0, 0, TAU, 24, mid, 1.2)
		draw_string(_font, p + Vector2(9, 4), el.name,
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1,
			mid if el.name == "EARTH" else dim)

	# Threat, interceptor and the predicted-impact marker are dormant until 3C-2b
	# rebuilds them on the real core (see the Sim module note).
	if Sim.mission_online:
		_draw_threat(center, s, t, bright, mid, dim, faint)

	# Plot header.
	draw_string(_font, Vector2(w * 0.5 - 120, 40),
		"HELIOCENTRIC PLOT - ECLIPTIC N", HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, dim)


## Threat marker, its range line to Earth, the interceptor and the predicted
## impact cross — everything that depends on the mission layer, lifted out of
## _draw so the dormant case is one guarded call rather than five scattered ones.
func _draw_threat(center: Vector2, s: float, t: float, bright: Color,
		mid: Color, dim: Color, faint: Color) -> void:
	var burned: bool = Sim.burned()
	var p_e := _to_screen(Sim.pos_ecl(Sim.earth_el, t), center, s)
	var el_act: Dictionary = Sim.ast_defl_el if burned else Sim.ast_el
	var p_a := _to_screen(Sim.pos_ecl(el_act, t), center, s)
	_dashed_line(p_e, p_a, dim, 6.0, 5.0)
	var mid_pt := (p_e + p_a) * 0.5
	draw_string(_font, mid_pt + Vector2(8, -4),
		"RNG %.3f AU" % (Sim.threat_range_km(t) / Sim.AU_KM),
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, mid)

	var tcol := bright if (burned or Sim.blink(1.4)) else mid
	_diamond(p_a, 6.0, tcol)
	draw_string(_font, p_a + Vector2(10, 4),
		"2031-XK" + ("" if burned else " <THREAT>"),
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, tcol)
	if burned:
		var p_n := _to_screen(Sim.pos_ecl(Sim.ast_el, t), center, s)
		_diamond(p_n, 5.0, faint)
		draw_string(_font, p_n + Vector2(9, 12), "NOMINAL",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 2, dim)

	if Sim.interceptor_phase(t) == "CRUISE":
		var p_i := _map_pos(Sim.interceptor_pos(t), center, s)
		draw_line(p_i + Vector2(-6, 0), p_i + Vector2(6, 0), bright, 1.2)
		draw_line(p_i + Vector2(0, -6), p_i + Vector2(0, 6), bright, 1.2)
		draw_string(_font, p_i + Vector2(9, 4), "ATLAS-1",
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs - 1, bright)

	if not burned and Sim.blink(2.2):
		var p_x := _to_screen(Sim.pos_ecl(Sim.earth_el, Sim.T_IMPACT), center, s)
		draw_line(p_x + Vector2(-7, -7), p_x + Vector2(7, 7), bright, 1.5)
		draw_line(p_x + Vector2(-7, 7), p_x + Vector2(7, -7), bright, 1.5)


func _map_pos(scene_pos: Vector3, center: Vector2, s: float) -> Vector2:
	# Scene units -> ecliptic AU -> screen.
	var ecl := Vector3(scene_pos.x, -scene_pos.z, scene_pos.y) / Sim.AU
	return _to_screen(ecl, center, s)


## Orbit polyline for any body. Delegates the sampling to Sim.orbit_points so a
## real ephemeris body traces its ACTUAL orbit (walked from the field) and a
## Kepler body its analytic one — this used to poke `el.m0` directly, which only
## a Kepler body has.
func _orbit_trace(el: Dictionary, center: Vector2, s: float, col: Color,
		dashed: bool = false) -> void:
	var pts := PackedVector2Array()
	for p in Sim.orbit_points(el, 180):
		pts.append(_map_pos(p, center, s))
	if pts.size() < 2:
		return
	if dashed:
		for k in range(0, pts.size() - 1, 2):
			draw_line(pts[k], pts[k + 1], col, 1.0)
	else:
		draw_polyline(pts, col, 1.0)


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
