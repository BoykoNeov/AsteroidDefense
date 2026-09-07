extends RefCounted
## Screen-space geometry for the 2D plots — pure functions of pixels, with no
## reference to `Sim`, the scene, or anything the core owns.
##
## It is a separate file for a reason that is about testing, not tidiness. A
## headless `--script` run registers no autoloads, so any script that names `Sim`
## fails to *compile* in isolation and cannot be exercised outside a full game
## boot. `encounter.gd` names `Sim` on nearly every line. Putting the geometry
## here lets `tests/test_geometry.gd` load and measure it with no kernels, no
## build and no window — which is the only way the defect below gets caught,
## because a mis-tessellated circle still draws as a plausible line and no
## screenshot harness reads pixels.
##
## Loaded by `preload`, not by `class_name`: a new global class needs an editor
## rescan before a game run can see it, and that has cost this project a session
## before.

## Screen-space accuracy budget for a drawn circle, px: no drawn chord may bow
## further than this from the circle it stands in for.
##
## Circles here are drawn in *screen* space, so their tessellation has to be
## sized there too, and `draw_arc(cc, r, 0, TAU, 256, ...)` sizes it in *angle* —
## 256 segments spread around the whole circle however big the circle is. That is
## fine at the default span and wrong at the zoom-in stop, because the radii the
## b-plane view draws are unbounded in the only sense a picture cares about: the
## widest resonance on the shipping map (15:19) has a radius of 9.232e6 km,
## twenty-four lunar distances, and its ring passes about 3 000 km from Earth. So
## it is *in frame* at the deepest zoom while being 795 431 px across, one of its
## 256 chords spans 19 500 px, and the straight line drawn through the 720 px
## viewport sits up to 59.9 px from where the circle actually is — a twelfth of
## the screen, on the one view whose entire job is where a line falls relative to
## a disc. Both numbers are measured by `test_geometry.gd`, not asserted here.
##
## The core was never the problem: `ResonantCircle` refuses the degenerate case
## and a 9.232e6 km radius is a true answer. The drawing was. Clipping the arc to
## the view and tessellating to this budget is both right and *cheaper* — the
## visible window of that same circle needs three points, not 256.
const CIRCLE_CHORD_PX := 0.3
## Hard cap on points per circle, so a pathological radius costs a bounded frame
## rather than a hung one. Nothing comes near it: sweeping eight decades of
## radius with the ring through the view, the worst case measured is **78 points,
## at a radius of 354 px** — the cap is thirteen times that.
const CIRCLE_MAX_POINTS := 1024


## The part of the circle `(cc, r)` that can reach `rect`, as a polyline whose
## chords bow no more than [constant CIRCLE_CHORD_PX] from the true circle.
## Empty when no part of it can — which is two cases, not one.
##
## **The view is treated as its circumscribing disc**, not as the rectangle. That
## is deliberately loose — it keeps slightly more arc than strictly needed near
## the corners — because an exact rectangle clip is four line-circle
## intersections and a case analysis, to answer a question whose only consumer is
## "how much of this ring do I tessellate". The looseness costs points, and the
## point count is capped anyway.
##
## The window comes from the law of cosines on the triangle (circle centre, view
## centre, a point where the two circles cross): with `d` the distance between
## the centres, `vr` the view radius and `r` the circle's,
## `cos(half-window) = (d*d + r*r - vr*vr) / (2*d*r)`. Out of range clamps to a
## full circle, which is the right answer when the circle fits inside the view.
static func circle_polyline(cc: Vector2, r: float, rect: Rect2) -> PackedVector2Array:
	var pts := PackedVector2Array()
	if not is_finite(r) or r <= 0.0 or not is_finite(cc.x) or not is_finite(cc.y):
		return pts
	var vc := rect.get_center()
	var vr: float = rect.size.length() * 0.5
	var d: float = cc.distance_to(vc)
	# Two ways to miss the view, and the old cull in `encounter.gd` could only see
	# the first: the ring passes too far away, or the ring is so large that the
	# whole view sits inside it. The second is the shape the widest resonances
	# have, which is exactly why they were being drawn as 256 points of nothing.
	if d - r > vr or r - d > vr:
		return pts

	var half := PI
	var theta0 := 0.0
	if d > 1.0e-9:
		half = acos(clampf((d * d + r * r - vr * vr) / (2.0 * d * r), -1.0, 1.0))
		theta0 = (vc - cc).angle()

	# A chord subtending `da` on radius `r` bows `r * (1 - cos(da / 2))` from its
	# arc. Invert that for the budget. A circle smaller than the budget is drawn
	# as a couple of points and nobody can tell the difference.
	var step := TAU
	if r > CIRCLE_CHORD_PX:
		step = 2.0 * acos(clampf(1.0 - CIRCLE_CHORD_PX / r, -1.0, 1.0))
	var n: int = clampi(int(ceil(2.0 * half / maxf(step, 1.0e-9))), 2, CIRCLE_MAX_POINTS - 1)

	var da: float = 2.0 * half / float(n)
	var a0: float = theta0 - half
	pts.resize(n + 1)
	for i in n + 1:
		var a: float = a0 + da * float(i)
		pts[i] = cc + Vector2(cos(a), sin(a)) * r
	return pts
