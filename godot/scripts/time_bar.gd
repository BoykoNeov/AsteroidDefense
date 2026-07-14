class_name TimeBar
extends Control
## On-screen scrub timeline: a bottom strip showing the whole mission clock span
## [Sim.T_MIN, Sim.T_MAX] with a draggable playhead, mission-event ticks, and a
## warp/direction readout. Drawn white/gray — the CRT shader maps it to phosphor.
##
## It occupies only a thin strip at the bottom and uses MOUSE_FILTER_STOP, so a
## drag inside it scrubs the clock while a drag anywhere else still reaches the
## orbit camera. Dragging calls Sim.scrub_frac(); it does not fight the warp.

const STRIP_H := 46.0                  # height of the bar, px
const PAD := 24.0                      # left/right inset of the track, px
const TRACK_Y := 30.0                  # track baseline within the strip, px

var _font: Font
var _fs := 13
var _dragging := false


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP
	_font = Sim.mono_font


## Called by the assembler on viewport resize: dock to a full-width bottom strip.
func layout(viewport_size: Vector2) -> void:
	position = Vector2(0.0, viewport_size.y - STRIP_H)
	size = Vector2(viewport_size.x, STRIP_H)


func _process(_delta: float) -> void:
	queue_redraw()


func _gui_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		_dragging = event.pressed
		if event.pressed:
			_scrub_to_x(event.position.x)
		accept_event()
	elif event is InputEventMouseMotion and _dragging:
		_scrub_to_x(event.position.x)
		accept_event()


func _scrub_to_x(local_x: float) -> void:
	var track_w := size.x - 2.0 * PAD
	Sim.scrub_frac((local_x - PAD) / maxf(track_w, 1.0))


func _draw() -> void:
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.42, 0.42, 0.42)
	var faint := Color(0.25, 0.25, 0.25)
	var bright := Color(1, 1, 1)

	var x0 := PAD
	var x1 := size.x - PAD
	var track_w := x1 - x0
	var span: float = Sim.T_MAX - Sim.T_MIN

	# Baseline track.
	draw_line(Vector2(x0, TRACK_Y), Vector2(x1, TRACK_Y), dim, 1.5)

	# Year ticks (epoch 2031). A tick every ~decade keeps the long span readable.
	var step_days := 3652.5             # 10 years
	var d: float = ceil(Sim.T_MIN / step_days) * step_days
	while d <= Sim.T_MAX:
		var tx: float = x0 + (d - Sim.T_MIN) / span * track_w
		draw_line(Vector2(tx, TRACK_Y - 4.0), Vector2(tx, TRACK_Y + 4.0), faint, 1.0)
		var yr := 2031 + int(round(d / 365.25))
		_text_c(Vector2(tx, TRACK_Y + 18.0), str(yr), faint)
		d += step_days

	# Mission-event markers (launch / intercept / impact) as labelled pips.
	_marker(Sim.T_IMPACT, x0, track_w, span, "IMP", mid)
	if Sim.committed:
		_marker(Sim.T_LAUNCH, x0, track_w, span, "LCH", dim)
		_marker(Sim.T_INTERCEPT, x0, track_w, span, "INT", dim)

	# Playhead at the current clock fraction.
	var px: float = x0 + Sim.clock_frac() * track_w
	draw_line(Vector2(px, TRACK_Y - 12.0), Vector2(px, TRACK_Y + 12.0), bright, 2.0)
	draw_rect(Rect2(px - 4.0, TRACK_Y - 14.0, 8.0, 6.0), bright, true)

	# Left readout: WARP + direction (blinks HOLD when paused).
	var status: String = ("** HOLD **" if Sim.paused else ("WARP " + Sim.warp_label()))
	if not (Sim.paused and not Sim.blink(2.0)):
		_text(Vector2(x0, 14.0), status, mid)
	# Right readout: current date + a scrub hint.
	_text_r(Vector2(x1, 14.0), Sim.date_string() + "   [B]REV  DRAG TO SCRUB", dim)


# ------------------------------------------------------------------ helpers ---

func _marker(t_days: float, x0: float, track_w: float, span: float,
		tag: String, col: Color) -> void:
	if t_days < Sim.T_MIN or t_days > Sim.T_MAX:
		return
	var mx: float = x0 + (t_days - Sim.T_MIN) / span * track_w
	draw_line(Vector2(mx, TRACK_Y - 8.0), Vector2(mx, TRACK_Y + 8.0), col, 1.5)
	_text_c(Vector2(mx, TRACK_Y - 12.0), tag, col)


func _text(pos: Vector2, s: String, col: Color) -> void:
	draw_string(_font, pos, s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)


func _text_r(pos: Vector2, s: String, col: Color) -> void:
	var sw := _font.get_string_size(s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs).x
	draw_string(_font, pos - Vector2(sw, 0), s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)


func _text_c(pos: Vector2, s: String, col: Color) -> void:
	var sw := _font.get_string_size(s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs).x
	draw_string(_font, pos - Vector2(sw * 0.5, 0), s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)
