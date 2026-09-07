extends SceneTree
## Headless checks on the view geometry that has no scene in it — today, the
## clipped tessellation of a resonant-return circle.
##   godot --headless --path godot --script res://tests/test_geometry.gd
##
## Needs no kernels, no build, no window: `PlotGeometry.circle_polyline` is a
## pure function of a centre, a radius and a rectangle, and that is exactly why
## it — and the file it lives in — exist apart from `encounter.gd`. A `--script`
## run registers no autoloads, so a script that names `Sim` cannot even be
## compiled here; `encounter.gd` names it constantly.
##
## The defect these checks were written for is invisible to every other harness — a circle drawn with the wrong tessellation is still
## a line on the screen in roughly the right place, so `_shot.gd` shows a picture
## that looks fine, and nothing in `test_orrery.gd` reads pixels. The only way to
## catch it is to measure the drawn points against the circle they stand for.
##
## The case that matters is not a made-up one. `docs/keyhole_map.json` — the
## published map this view draws — contains a 15:19 resonance whose circle has a
## radius of 9.232e6 km and a centre 9.229e6 km down the zeta axis, so the ring
## passes about 3 000 km from Earth. It is on screen at the deepest zoom while
## being enormous, which is the combination the old whole-circle tessellation
## could not survive.

var fails := 0

## The published 15:19 circle, km — the widest on the shipping map.
const WIDE_RADIUS_KM := 9.232e6
const WIDE_CENTER_ZETA_KM := 9.229e6
## A lunar distance in km, as `Sim.LD_KM` has it.
const LD_KM := 384400.0
## The zoom-in stop (`_half_ld` clamps to 0.01) and a 1280x720 viewport, which is
## what the shot harness renders at.
const HALF_LD_MIN := 0.01
const VIEW_W := 1280.0
const VIEW_H := 720.0
const MARGIN := 18.0


func _check(ok: bool, msg: String) -> void:
	if ok:
		print("PASS  " + msg)
	else:
		print("FAIL  " + msg)
		fails += 1


## Largest distance from `pts` (read as a polyline) to the circle it stands for.
## Measured at the chord midpoints, where a chord is furthest from its arc.
func _max_bow(pts: PackedVector2Array, cc: Vector2, r: float) -> float:
	var worst := 0.0
	for i in pts.size() - 1:
		var mid: Vector2 = (pts[i] + pts[i + 1]) * 0.5
		worst = maxf(worst, absf(r - mid.distance_to(cc)))
	return worst


## Distance from `p` to the polyline `pts`.
func _dist_to_polyline(p: Vector2, pts: PackedVector2Array) -> float:
	var best := INF
	for i in pts.size() - 1:
		best = minf(best, p.distance_to(Geometry2D.get_closest_point_to_segment(
			p, pts[i], pts[i + 1])))
	return best


func _init() -> void:
	var ev := load("res://scripts/plot_geometry.gd")
	var rect := Rect2(Vector2.ZERO, Vector2(VIEW_W, VIEW_H)).grow(-MARGIN)
	var center := Vector2(VIEW_W, VIEW_H) * 0.5
	var ppl: float = minf(VIEW_W, VIEW_H) * 0.5 / HALF_LD_MIN * 0.92

	# The wide circle in screen pixels, at the deepest zoom. Zeta is +y down in
	# this view's plot transform; the sign does not matter to any check here.
	var r: float = WIDE_RADIUS_KM / LD_KM * ppl
	var cc := center + Vector2(0.0, WIDE_CENTER_ZETA_KM / LD_KM * ppl)
	print("wide circle at the zoom stop: r = %.0f px, centre %.0f px from the view"
		% [r, cc.distance_to(center)])

	var poly: PackedVector2Array = ev.circle_polyline(cc, r, rect)
	_check(poly.size() >= 2, "the wide circle is drawn at all (%d points)" % poly.size())
	_check(poly.size() <= ev.CIRCLE_MAX_POINTS, "point count is bounded (%d <= %d)"
		% [poly.size(), ev.CIRCLE_MAX_POINTS])

	# --- 1. The drawn line is where the circle is -----------------------------
	var bow := _max_bow(poly, cc, r)
	# What the old code drew: 256 segments spread around the whole circle, so the
	# bow is fixed by the radius alone. This is the number the fix exists for, and
	# it is measured here rather than quoted from a comment.
	var old_bow: float = r * (1.0 - cos(TAU / 512.0))
	print("bow from the true circle: %.3f px clipped, %.1f px as 256 whole-circle segments"
		% [bow, old_bow])
	_check(bow <= ev.CIRCLE_CHORD_PX * 1.05,
		"the drawn chords stay within the %.2f px budget (%.4f px)"
		% [ev.CIRCLE_CHORD_PX, bow])
	_check(old_bow > 10.0,
		"and the old whole-circle tessellation did not (%.1f px, on a %d px view)"
		% [old_bow, int(VIEW_H)])

	# --- 2. Nothing visible is left undrawn -----------------------------------
	# Clipping is only honest if the arc it keeps covers everything in frame. Walk
	# the true circle finely through the window and require every sample that
	# lands in the rect to be on the polyline.
	var missed := 0.0
	var theta0: float = (rect.get_center() - cc).angle()
	var sweep: float = 4.0 * rect.size.length() / r      # generously past the view
	var steps := 4000
	for i in steps + 1:
		var a: float = theta0 - sweep * 0.5 + sweep * float(i) / float(steps)
		var p: Vector2 = cc + Vector2(cos(a), sin(a)) * r
		if rect.has_point(p):
			missed = maxf(missed, _dist_to_polyline(p, poly))
	_check(missed <= ev.CIRCLE_CHORD_PX * 1.05,
		"every in-frame point of the circle is on the drawn line (worst %.4f px)" % missed)

	# --- 3. Culling, both ways ------------------------------------------------
	# Far away: the ring is nowhere near the view.
	_check(ev.circle_polyline(center + Vector2(0.0, 50000.0), 100.0, rect).is_empty(),
		"a ring that passes nowhere near the view is not drawn")
	# The case the old cull could not see: a radius so large that the whole view
	# sits inside the ring. The old test (`dist - r > diagonal`) is very negative
	# here, so it drew — 256 points of nothing, every frame.
	_check(ev.circle_polyline(center + Vector2(0.0, 1.0e7), 1.0e7 - 5000.0, rect).is_empty(),
		"a ring the view sits wholly inside is not drawn either")
	# And a ring that does cross the view is kept.
	_check(not ev.circle_polyline(center, 200.0, rect).is_empty(),
		"a ring through the view is drawn")

	# --- 4. A circle inside the view is a closed loop -------------------------
	var small: PackedVector2Array = ev.circle_polyline(center, 40.0, rect)
	_check(small.size() >= 8 and small[0].distance_to(small[small.size() - 1]) < 0.01,
		"a circle that fits in the view closes on itself (%d points)" % small.size())
	_check(_max_bow(small, center, 40.0) <= ev.CIRCLE_CHORD_PX * 1.05,
		"and meets the same budget")

	# --- 5. The point cap is never what bounds the picture --------------------
	# The cap exists so a pathological radius costs a bounded frame. It should
	# never be the thing deciding a real circle's tessellation, and the claim is
	# worth measuring rather than believing: sweep eight decades of radius with the
	# ring passing close to the view centre, which is how these circles actually
	# sit, and report the worst.
	var worst_n := 0
	var worst_r := 0.0
	for i in 400:
		var rr: float = pow(10.0, -1.0 + 8.0 * float(i) / 399.0)
		var n: int = ev.circle_polyline(center + Vector2(0.0, rr * 0.999), rr, rect).size()
		if n > worst_n:
			worst_n = n
			worst_r = rr
	print("worst point count over radii 1e-1..1e7 px: %d at r = %.0f px" % [worst_n, worst_r])
	_check(worst_n < ev.CIRCLE_MAX_POINTS,
		"the point cap is never reached (%d < %d)" % [worst_n, ev.CIRCLE_MAX_POINTS])

	# --- 6. Degenerate inputs are refused, not drawn --------------------------
	_check(ev.circle_polyline(center, 0.0, rect).is_empty(), "a zero radius draws nothing")
	_check(ev.circle_polyline(center, -5.0, rect).is_empty(), "a negative radius draws nothing")
	_check(ev.circle_polyline(center, INF, rect).is_empty(), "an infinite radius draws nothing")
	_check(ev.circle_polyline(Vector2(NAN, 0.0), 100.0, rect).is_empty(),
		"a NaN centre draws nothing")

	print("----")
	print("%d failure(s)" % fails)
	quit(1 if fails > 0 else 0)
