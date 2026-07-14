class_name BootScreen
extends Control
## Boot/POST typewriter overlay shown at startup. Dismissed by any key
## (handled in main.gd) or auto-fades after the sequence completes.

signal finished

const LINES := [
	"ASTEROID DEFENSE COMMAND - PDC/OS v2.6",
	"COPYRIGHT (C) 2031 PLANETARY DEFENSE COORDINATION OFFICE",
	"",
	"MEMORY TEST ................ 65536 KB OK",
	"EPHEMERIS KERNEL ........... DE440S LOADED",
	"PROPAGATOR ................. DOP853 F64 [RUST CORE]",
	"B-PLANE TARGETING .......... ONLINE",
	"CLOSE-APPROACH SCANNER ..... ONLINE",
	"DEFLECTION SOLVER .......... ONLINE",
	"MISSION PLANNER ............ READY - KEY [M]",
	"",
	"THREAT DB .................. 1 OBJECT(S) FLAGGED",
	"> 2031-XK  P(IMPACT)=1.000  EPOCH E-1200 D",
	"",
	"ALL SYSTEMS NOMINAL",
	"",
	"PRESS ANY KEY TO ENGAGE TACTICAL DISPLAY",
]

var _chars := 0.0
var _done := false
var _idle := 0.0
var _font: Font
var _fs := 16


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP
	set_anchors_preset(Control.PRESET_FULL_RECT)
	_font = Sim.mono_font


func _process(delta: float) -> void:
	if not _done:
		_chars += delta * 220.0
		if _chars >= float(_total_chars()):
			_done = true
	else:
		_idle += delta
		if _idle > 5.0:
			dismiss()
			return
	queue_redraw()


func dismiss() -> void:
	finished.emit()
	queue_free()


func _total_chars() -> int:
	var n := 0
	for l in LINES:
		n += l.length() + 1
	return n


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, size), Color(0, 0, 0), true)
	var budget := int(_chars)
	var y := size.y * 0.18
	var x := size.x * 0.14
	var lh := _fs + 8.0
	for l in LINES:
		if budget <= 0:
			break
		var line: String = l.substr(0, mini(l.length(), budget))
		budget -= l.length() + 1
		var col := Color(1, 1, 1) if (l.begins_with(">") or l.begins_with("ALL") or l.begins_with("PRESS")) \
			else Color(0.75, 0.75, 0.75)
		draw_string(_font, Vector2(x, y), line, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)
		y += lh
	if Sim.blink(2.5):
		draw_string(_font, Vector2(x, y), "_", HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, Color(1, 1, 1))
