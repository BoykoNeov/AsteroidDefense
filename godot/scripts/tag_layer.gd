class_name TagLayer
extends Control
## Projects 3D bodies to screen space and draws retro tracking tags:
## box/diamond markers, designation text, predicted-impact X. Text stays
## upright and pixel-crisp (authentic vector-display annotation style).
##
## **Tags are collected, then placed, then drawn** — not drawn where they fall.
## Sixteen belt asteroids, eight planets, the Moon, the NEOs and the threat all
## label themselves 12 px right of their glyph, and at system zoom that put
## "PREDICTED IMPACT E-4383" through "EARTH" and "2031-XK <THREAT>" through
## "Apophis". The glyph never moves — it is the measured position and the only
## thing on this layer that is a claim about where something *is*. Only the text
## slides, to the first offset that is clear.

var camera_rig: OrbitCameraRig

var _font: Font
var _fs := 13

## Label offsets from the glyph, tried in order. `x < 0` means "place the text's
## right edge this far left of the glyph", i.e. the label flips to the other side;
## the width is not known until the string is measured, so it cannot be baked in.
const OFFSETS: Array[Vector2] = [
	Vector2(12, 4), Vector2(12, -10), Vector2(-12, 4), Vector2(12, 18)]

## Collected this frame: [priority, screen_pos, text, colour, glyph, show].
## Rebuilt every `_draw`; never read outside one.
var _tags: Array = []


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
	_tags.clear()
	_collect(camera_rig.camera, Sim.t)
	_paint(_place())


## Everything that wants a tag this frame, in draw order.
##
## Nothing here draws. A tag whose *text* is suppressed this frame (the impact
## mark blinks) is still collected with `show = false`, because its rectangle must
## be reserved regardless: if the label pass only saw what is currently visible,
## every neighbouring label would re-solve its offset twice a second and the whole
## layer would jitter. Blink decides what is painted, never what is placed.
func _collect(cam: Camera3D, t: float) -> void:
	var mid := Color(0.75, 0.75, 0.75)
	var dim := Color(0.45, 0.45, 0.45)
	var bright := Color(1, 1, 1)

	_add(cam, Vector3.ZERO, "SOL", dim, "box", 2)
	for el in Sim.planets:
		_add(cam, Sim.pos3d(el, t), el.name,
			mid if el.name == "EARTH" else dim, "box", 2)
	# The sixteen real main-belt bodies. Tagged for the same reason the planets are:
	# an untagged blob among 1600 scenery dust points is indistinguishable from the
	# scenery, and the entire point of mounting a kernel rather than scattering an
	# RNG annulus is that these sixteen are *real*. A name is what carries that.
	#
	# Only when zoomed out far enough to see the belt at all — at close zoom they
	# are off-screen or piled on each other, and sixteen overlapping labels is worse
	# than none. `Sim.asteroids` is empty unless the kernel mounted, so there is no
	# unmounted state in which this labels anything.
	#
	# Priority 1: these are the tags that give way. Sixteen of them arrive in one
	# clump and they are the least load-bearing thing on the screen, so when the
	# clump has no room the glyph stays and the name goes.
	if camera_rig.distance > 18.0:
		for el in Sim.asteroids:
			_add(cam, Sim.pos3d(el, t), el.name, dim, "box", 1)

	# Moon tag only at close zoom — at system scale it overlaps EARTH's.
	if camera_rig.distance < 10.0:
		_add(cam, Sim.moon_pos3d(t), "MOON", dim, "box", 1)

	# A tag is a claim that something is *there*. Outside the threat's propagated
	# span there is no threat to point at — and a lookup would return ZERO, so an
	# ungated tag would confidently label the Sun "2031-XK <THREAT>".
	if not Sim.threat_active(t):
		return

	var burned: bool = Sim.burned() and Sim.has_plan()
	if burned:
		_add(cam, Sim.pos3d(Sim.ast_el, t), "NOMINAL TRK", dim, "diamond", 3)
		_add(cam, Sim.pos3d(Sim.ast_defl_el, t), "2031-XK", bright, "diamond", 3)
	else:
		var col := bright if Sim.blink(1.4) else mid
		_add(cam, Sim.pos3d(Sim.ast_el, t), "2031-XK <THREAT>", col, "diamond", 3)

	# Only while it is on its arc — a tag at ZERO would label the Sun.
	if Sim.catalog_active(Sim.comet_el, t):
		_add(cam, Sim.pos3d(Sim.comet_el, t), Sim.comet_el.name, dim, "diamond", 2)

	# The real NEOs, named — and the name is the point, exactly as it was for the
	# sixteen belt asteroids: an unlabelled blob among the scenery is
	# indistinguishable from scenery, and these are the only real near-Earth
	# objects on the screen. Same per-body span gate as the draw above.
	for el in Sim.neos:
		if Sim.catalog_active(el, t):
			_add(cam, Sim.pos3d(el, t), el.name, bright, "diamond", 2)

	if Sim.interceptor_online and Sim.interceptor_phase(t) == "CRUISE":
		_add(cam, Sim.interceptor_pos(t), "ATLAS-1", bright, "cross", 3)

	# Predicted impact point: Earth's position at the impact epoch — a constant of
	# the threat solution, read from Sim rather than looked up every frame.
	if not burned:
		var p_imp: Vector3 = Sim.ecl_to_godot(Sim.impact_point_ecl)
		_add(cam, p_imp, "PREDICTED IMPACT E-%04d" % int(maxf(0.0, Sim.T_IMPACT - t)),
			bright, "x", 3, Sim.blink(2.2))


## Where each collected label goes, as an array of `Rect2` parallel to `_tags`.
## An empty rect means "no room — draw the glyph, drop the text", which only ever
## happens to a priority-1 tag.
##
## Highest priority first, so the threat and the impact mark keep the canonical
## offset and the scenery works around them rather than the other way round.
func _place() -> Array:
	var order: Array = []
	for i in _tags.size():
		order.append(i)
	# Priority descending, collection order within a priority. `sort_custom` is not
	# a stable sort, so the index is part of the key rather than left to chance —
	# an unstable tiebreak means labels swap places between frames and flicker.
	order.sort_custom(func(a: int, b: int) -> bool:
		if _tags[a][0] != _tags[b][0]:
			return _tags[a][0] > _tags[b][0]
		return a < b)

	var rects: Array = []
	rects.resize(_tags.size())
	var placed: Array[Rect2] = []
	var h: float = _font.get_height(_fs)
	var ascent: float = _font.get_ascent(_fs)
	for i: int in order:
		var e: Array = _tags[i]
		var sp: Vector2 = e[1]
		var w: float = _font.get_string_size(
			e[2], HORIZONTAL_ALIGNMENT_LEFT, -1, _fs).x
		var chosen := Rect2()
		for off: Vector2 in OFFSETS:
			var at := Vector2(sp.x + off.x if off.x > 0.0 else sp.x - w + off.x,
				sp.y + off.y)
			var r := Rect2(at.x, at.y - ascent, w, h)
			var clear := true
			for p: Rect2 in placed:
				if r.intersects(p):
					clear = false
					break
			if clear:
				chosen = r
				break
		# Nothing fits. A low-priority name gives way (empty rect = glyph only);
		# anything else keeps the canonical offset and overlaps, because dropping
		# the threat's or the impact point's name is worse than crowding.
		if chosen.size == Vector2.ZERO and e[0] > 1:
			chosen = Rect2(sp.x + OFFSETS[0].x, sp.y + OFFSETS[0].y - ascent, w, h)
		if chosen.size != Vector2.ZERO:
			placed.append(chosen)
		rects[i] = chosen
	return rects


## Glyphs at their true projected positions, labels at the rects `_place` chose.
## Collection order, so the stacking is exactly what it was before tags moved.
func _paint(rects: Array) -> void:
	var ascent: float = _font.get_ascent(_fs)
	for i in _tags.size():
		var e: Array = _tags[i]
		if not e[5]:
			continue                       # blinked off this frame
		var sp: Vector2 = e[1]
		var col: Color = e[3]
		match e[4]:
			"box":
				draw_rect(Rect2(sp - Vector2(5, 5), Vector2(10, 10)), col, false, 1.2)
			"diamond":
				var r := 7.0
				draw_polyline(PackedVector2Array([
					sp + Vector2(0, -r), sp + Vector2(r, 0),
					sp + Vector2(0, r), sp + Vector2(-r, 0), sp + Vector2(0, -r)]),
					col, 1.2)
			"cross":
				draw_line(sp + Vector2(-7, 0), sp + Vector2(7, 0), col, 1.2)
				draw_line(sp + Vector2(0, -7), sp + Vector2(0, 7), col, 1.2)
			"x":
				var r := 8.0
				draw_line(sp + Vector2(-r, -r), sp + Vector2(r, r), col, 1.5)
				draw_line(sp + Vector2(-r, r), sp + Vector2(r, -r), col, 1.5)
		var rect: Rect2 = rects[i]
		if rect.size == Vector2.ZERO:
			continue                       # crowded out; the glyph still marks it
		draw_string(_font, Vector2(rect.position.x, rect.position.y + ascent), e[2],
			HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)


# ------------------------------------------------------------------ markers ---

func _project(cam: Camera3D, world: Vector3) -> Variant:
	if cam.is_position_behind(world):
		return null
	var sp := cam.unproject_position(world)
	if sp.x < -50 or sp.y < -50 or sp.x > size.x + 50 or sp.y > size.y + 50:
		return null
	return sp


func _add(cam: Camera3D, world: Vector3, text: String, col: Color, glyph: String,
		priority: int, show: bool = true) -> void:
	var sp = _project(cam, world)
	if sp == null:
		return
	_tags.append([priority, sp, text, col, glyph, show])
