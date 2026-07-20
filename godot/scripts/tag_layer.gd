class_name TagLayer
extends Control
## Projects 3D bodies to screen space and draws retro tracking tags:
## box/diamond markers, designation text, predicted-impact X. Text stays
## upright and pixel-crisp (authentic vector-display annotation style).

var camera_rig: OrbitCameraRig

var _font: Font
var _fs := 13


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_preset(Control.PRESET_FULL_RECT)
	_font = Sim.mono_font


func _process(_delta: float) -> void:
	queue_redraw()


func _draw() -> void:
	if camera_rig == null or camera_rig.camera == null:
		return
	if not Sim.bodies_online:
		return                            # nothing is drawn, so nothing to tag
	var cam := camera_rig.camera
	var t: float = Sim.t
	var mid := Color(0.75, 0.75, 0.75)
	var dim := Color(0.45, 0.45, 0.45)
	var bright := Color(1, 1, 1)

	_tag_box(cam, Vector3.ZERO, "SOL", dim)
	for el in Sim.planets:
		_tag_box(cam, Sim.pos3d(el, t), el.name, mid if el.name == "EARTH" else dim)
	# The sixteen real main-belt bodies. Tagged for the same reason the planets are:
	# an untagged blob among 1600 scenery dust points is indistinguishable from the
	# scenery, and the entire point of mounting a kernel rather than scattering an
	# RNG annulus is that these sixteen are *real*. A name is what carries that.
	#
	# Only when zoomed out far enough to see the belt at all — at close zoom they
	# are off-screen or piled on each other, and sixteen overlapping labels is worse
	# than none. `Sim.asteroids` is empty unless the kernel mounted, so there is no
	# unmounted state in which this labels anything.
	if camera_rig.distance > 18.0:
		for el in Sim.asteroids:
			_tag_box(cam, Sim.pos3d(el, t), el.name, dim)

	# Moon tag only at close zoom — at system scale it overlaps EARTH's.
	if camera_rig.distance < 10.0:
		_tag_box(cam, Sim.moon_pos3d(t), "MOON", dim)

	# A tag is a claim that something is *there*. Outside the threat's propagated
	# span there is no threat to point at — and a lookup would return ZERO, so an
	# ungated tag would confidently label the Sun "2031-XK <THREAT>".
	if not Sim.threat_active(t):
		return

	var burned: bool = Sim.burned() and Sim.has_plan()
	if burned:
		_tag_diamond(cam, Sim.pos3d(Sim.ast_el, t), "NOMINAL TRK", dim)
		_tag_diamond(cam, Sim.pos3d(Sim.ast_defl_el, t), "2031-XK", bright)
	else:
		var col := bright if Sim.blink(1.4) else mid
		_tag_diamond(cam, Sim.pos3d(Sim.ast_el, t), "2031-XK <THREAT>", col)

	# Only while it is on its arc — a tag at ZERO would label the Sun.
	if Sim.catalog_active(Sim.comet_el, t):
		_tag_diamond(cam, Sim.pos3d(Sim.comet_el, t), Sim.comet_el.name, dim)

	# The real NEOs, named — and the name is the point, exactly as it was for the
	# sixteen belt asteroids: an unlabelled blob among the scenery is
	# indistinguishable from scenery, and these are the only real near-Earth
	# objects on the screen. Same per-body span gate as the draw above.
	for el in Sim.neos:
		if Sim.catalog_active(el, t):
			_tag_diamond(cam, Sim.pos3d(el, t), el.name, bright)

	if Sim.interceptor_online and Sim.interceptor_phase(t) == "CRUISE":
		_tag_cross(cam, Sim.interceptor_pos(t), "ATLAS-1", bright)

	# Predicted impact point: Earth's position at the impact epoch.
	if not burned:
		var p_imp: Vector3 = Sim.pos3d(Sim.earth_el, Sim.T_IMPACT)
		if Sim.blink(2.2):
			_tag_x(cam, p_imp, "PREDICTED IMPACT E-%04d" % int(maxf(0.0, Sim.T_IMPACT - t)), bright)


# ------------------------------------------------------------------ markers ---

func _project(cam: Camera3D, world: Vector3) -> Variant:
	if cam.is_position_behind(world):
		return null
	var sp := cam.unproject_position(world)
	if sp.x < -50 or sp.y < -50 or sp.x > size.x + 50 or sp.y > size.y + 50:
		return null
	return sp


func _label(sp: Vector2, text: String, col: Color) -> void:
	draw_string(_font, sp + Vector2(12, 4), text,
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)


func _tag_box(cam: Camera3D, world: Vector3, text: String, col: Color) -> void:
	var sp = _project(cam, world)
	if sp == null:
		return
	draw_rect(Rect2(sp - Vector2(5, 5), Vector2(10, 10)), col, false, 1.2)
	_label(sp, text, col)


func _tag_diamond(cam: Camera3D, world: Vector3, text: String, col: Color) -> void:
	var sp = _project(cam, world)
	if sp == null:
		return
	var r := 7.0
	var pts := PackedVector2Array([
		sp + Vector2(0, -r), sp + Vector2(r, 0),
		sp + Vector2(0, r), sp + Vector2(-r, 0), sp + Vector2(0, -r)])
	draw_polyline(pts, col, 1.2)
	_label(sp, text, col)


func _tag_cross(cam: Camera3D, world: Vector3, text: String, col: Color) -> void:
	var sp = _project(cam, world)
	if sp == null:
		return
	var r := 7.0
	draw_line(sp + Vector2(-r, 0), sp + Vector2(r, 0), col, 1.2)
	draw_line(sp + Vector2(0, -r), sp + Vector2(0, r), col, 1.2)
	_label(sp, text, col)


func _tag_x(cam: Camera3D, world: Vector3, text: String, col: Color) -> void:
	var sp = _project(cam, world)
	if sp == null:
		return
	var r := 8.0
	draw_line(sp + Vector2(-r, -r), sp + Vector2(r, r), col, 1.5)
	draw_line(sp + Vector2(-r, r), sp + Vector2(r, -r), col, 1.5)
	_label(sp, text, col)
