class_name PlannerPanel
extends Control
## Mission-planner overlay ([M]): edit intercept lead time and impulse
## magnitude/direction, watch the projected miss update live, commit the
## launch. Pure display — key handling lives in main.gd, state in Sim.

const W := 500.0
const MARGIN := 12.0

var _font: Font
var _fs := 15


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	visible = false
	_font = Sim.mono_font


func _process(_delta: float) -> void:
	if visible:
		queue_redraw()


func _draw() -> void:
	var lh := _fs + 6.0
	var rows := 14.0                         # drawn text rows, incl. separators
	var ph := rows * lh + 2.0 * MARGIN + 4.0
	var origin := Vector2(size.x * 0.5 - W * 0.5, size.y - ph - 60.0)
	var bright := Color(1, 1, 1)
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.42, 0.42, 0.42)
	var faint := Color(0.25, 0.25, 0.25)

	# Opaque backing so scene linework doesn't bleed through the text.
	var rect := Rect2(origin, Vector2(W, ph))
	draw_rect(rect, Color(0, 0, 0, 0.88), true)
	draw_rect(rect, mid, false, 1.2)
	var x := origin.x + MARGIN
	var xv := x + 12.0 * _fs * 0.62          # value column
	var y := origin.y + MARGIN + _fs

	_t(Vector2(x, y), "MISSION PLANNER - KINETIC INTERCEPT", bright)
	y += lh
	_t(Vector2(x, y), "-".repeat(56), faint)
	y += lh

	var lead := Sim.plan_lead_d
	_t(Vector2(x, y), "LEAD TIME", dim)
	_t(Vector2(xv, y), "< %4d D > BEFORE IMPACT" % int(lead), mid)
	_t_r(Vector2(origin.x + W - MARGIN, y), "[LEFT/RIGHT]", dim)
	y += lh
	_t(Vector2(x, y), "INTERCEPT", dim)
	_t(Vector2(xv, y), "E-%04d  MJD-REL %06.0f" % [int(lead), Sim.T_INTERCEPT], mid)
	y += lh
	_t(Vector2(x, y), "DV IMPULSE", dim)
	_t(Vector2(xv, y), "- %6.1f M/S + %s" %
		[Sim.plan_dv_ms, "RETROGRADE" if Sim.plan_retro else "PROGRADE"], mid)
	_t_r(Vector2(origin.x + W - MARGIN, y), "[-/=] [V]", dim)
	y += lh
	_t(Vector2(x, y), "LAUNCH", dim)
	_t(Vector2(xv, y), "E-%04d  CRUISE %d D" %
		[int(Sim.T_IMPACT - Sim.T_LAUNCH), int(Sim.cruise_d())], mid)
	y += lh
	_t(Vector2(x, y), "-".repeat(56), faint)
	y += lh

	# Miss and verdict both come from Sim's formatters, never from `miss_ld`
	# directly: a clean miss has no finite perigee to print (the core reports -1),
	# and it is the SUCCESS case. Formatting it here would re-open that trap.
	_t(Vector2(x, y), "PROJ MISS", dim)
	_t(Vector2(xv, y), Sim.miss_label(true), bright)
	y += lh
	_t(Vector2(x, y), "CAPTURE", dim)
	_t(Vector2(xv, y), "%.3f LD RADIUS (%.1f RE)" %
		[Sim.cap_km / Sim.LD_KM, Sim.cap_km / Sim.R_E], mid)
	y += lh
	_t(Vector2(x, y), "VERDICT", dim)
	# Blinking is how this panel shouts IMPACT, so only a real failure blinks. A
	# pending solve has nothing to shout about yet, and a clean miss is good news.
	var steady: bool = Sim.deflect_ok or Sim.plan_solving or not Sim.has_plan()
	if steady or Sim.blink(1.4):
		_t(Vector2(xv, y), Sim.verdict_label(), bright)
	y += lh
	_t(Vector2(x, y), "REQ DV EST", dim)
	_t(Vector2(xv, y), Sim.req_dv_label() + " FOR 1.0 LD MISS", mid)
	y += lh
	_t(Vector2(x, y), "-".repeat(56), faint)
	y += lh

	_t(Vector2(x, y), "STATUS", dim)
	_t(Vector2(xv, y), _status_line(), bright if Sim.blink(1.8) or Sim.committed else mid)
	y += lh
	_t(Vector2(x, y), "[ENTER] COMMIT LAUNCH   [M] CLOSE", dim)


func _status_line() -> String:
	if Sim.burned():
		return "EXPENDED - BURN COMPLETE"
	if Sim.locked():
		return "LOCKED - INTERCEPTOR IN FLIGHT"
	if Sim.committed:
		return "COMMITTED - LAUNCH E-%04d" % int(Sim.T_IMPACT - Sim.T_LAUNCH)
	if Sim.T_LAUNCH < Sim.t + Sim.PAD_D:
		return "WINDOW CLOSED - REDUCE LEAD"
	return "DRAFT - NOT COMMITTED"


func _t(pos: Vector2, s: String, col: Color) -> void:
	draw_string(_font, pos, s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)


func _t_r(pos: Vector2, s: String, col: Color) -> void:
	var sw := _font.get_string_size(s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs).x
	draw_string(_font, pos - Vector2(sw, 0), s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)
