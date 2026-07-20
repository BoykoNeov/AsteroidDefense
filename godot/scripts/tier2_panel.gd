class_name Tier2Panel
extends Control
## Tier-2 force-model menu ([P]): switch the four non-Tier-1 perturbations on and
## off and read how far each one moves the predicted b-plane perigee.
##
## The numbers are not computed here and they are not live-integrated on a keypress:
## the core measured all four **once** at build time (`Mission.with_tier2_preview`),
## each by re-flying the fixed shipping seed through one term in isolation — the
## honest "how much does this piece of physics move the impact" measurement (HANDOFF
## §5/§6), never a rebuild that would reproduce the hit by construction. So a toggle
## just reveals a cached, real number; "live" means instant, not recomputed.
##
## Pure display — key handling lives in main.gd, state in Sim. Mirrors PlannerPanel.

const W := 560.0
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
	var rows := 11.0
	var ph := rows * lh + 2.0 * MARGIN + 4.0
	var origin := Vector2(size.x * 0.5 - W * 0.5, size.y - ph - 60.0)
	var bright := Color(1, 1, 1)
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.42, 0.42, 0.42)
	var faint := Color(0.25, 0.25, 0.25)
	var on_col := Color(0.55, 1.0, 0.6)      # a term switched on

	var rect := Rect2(origin, Vector2(W, ph))
	draw_rect(rect, Color(0, 0, 0, 0.88), true)
	draw_rect(rect, mid, false, 1.2)
	var x := origin.x + MARGIN
	var xstate := x + 30.0 * _fs * 0.60       # ON/off column
	var xval := x + 36.0 * _fs * 0.60         # shift-value column
	var y := origin.y + MARGIN + _fs

	_t(Vector2(x, y), "TIER-2 FORCE MODEL - PERTURBATION MENU", bright)
	y += lh
	_t(Vector2(x, y), "-".repeat(62), faint)
	y += lh

	# The shifts are measured on demand (opening this menu kicks it). Until they
	# land, say what is happening plainly rather than draw four zeroes — a zero here
	# would read as "no effect".
	if not Sim.tier2_ready:
		var msg := ""
		if not Sim.mission_online:
			msg = "AWAITING THREAT SOLUTION"
		elif Sim.tier2_measuring:
			msg = "MEASURING FOUR FORCE-MODEL SHIFTS (~2 MIN) ..."
		else:
			msg = "PRESS [P] AGAIN TO MEASURE FORCE-MODEL SHIFTS"
		_t(Vector2(x, y), msg, dim)
		y += 1.5 * lh
		_t(Vector2(x, y), "EACH RE-FLIES THE THREAT WITH ONE TERM ADDED", faint)
		y += 1.5 * lh
		_t(Vector2(x, y), "[P] CLOSE", dim)
		return

	_t(Vector2(x, y), "SHIFT = HOW FAR THIS TERM MOVES THE NOMINAL PERIGEE", dim)
	y += lh

	for term in Sim.TIER2_TERMS:
		var key: String = term[0]
		var id: String = term[1]
		var name: String = term[2]
		var is_on: bool = Sim.tier2_on[id]
		_t(Vector2(x, y), "[%s] %s" % [key, name], mid if is_on else dim)
		_t(Vector2(xstate, y), "ON" if is_on else "off", on_col if is_on else faint)
		_t(Vector2(xval, y), _value_text(id, is_on), on_col if is_on else faint)
		y += lh

	y += lh * 0.35
	_t(Vector2(x, y), "-".repeat(62), faint)
	y += lh
	_t(Vector2(x, y), "NOMINAL PERIGEE", dim)
	_t(Vector2(xstate, y), "%8.1f KM  (CAPTURE %.0f KM)" %
		[Sim.nom_perigee_km, Sim.cap_km], mid)
	y += lh
	_t(Vector2(x, y), "[G/Y/A/S] TOGGLE TERM        [P] CLOSE", dim)


## The right-hand cell for one term: its measured shift when switched on, or a
## prompt/unavailable note otherwise.
func _value_text(id: String, is_on: bool) -> String:
	if not is_on:
		return "-- toggle to reveal --"
	if not Sim.tier2_available(id):
		# Only the belt reaches here, and only with no small-body kernel: the shift
		# is genuinely unknown, NOT zero.
		return "UNAVAILABLE (NO SB441 KERNEL)"
	var s := Sim.tier2_shift_km(id)
	var dir := "INWARD" if s > 0.0 else "OUTWARD"
	return "PERIGEE %+.2f KM  %s" % [s, dir]


func _t(pos: Vector2, s: String, col: Color) -> void:
	draw_string(_font, pos, s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)
