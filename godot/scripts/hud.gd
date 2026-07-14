class_name HUD
extends Control
## Retro terminal HUD: title block, mission clock, target/interceptor data
## panels, console log with typewriter reveal, help line. Drawn white/gray —
## the CRT shader maps everything to the phosphor palette.

const MARGIN := 18.0
const PANEL_W := 340.0

var camera_rig: OrbitCameraRig
var view_name := "TACTICAL 3D"

var _console: Array[String] = []
var _reveal := 0.0                   # chars revealed of the newest line
var _font: Font
var _fs := 15


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_preset(Control.PRESET_FULL_RECT)
	_font = Sim.mono_font
	Sim.event_logged.connect(_on_event)
	_console.append("CHANNEL OPEN - PDC TACTICAL UPLINK 7741.3")


func _on_event(line: String) -> void:
	_console.append(line)
	_reveal = 0.0
	if _console.size() > 8:
		_console = _console.slice(_console.size() - 8)


func _process(delta: float) -> void:
	_reveal += delta * 90.0          # typewriter: chars/sec
	queue_redraw()


func _draw() -> void:
	var w := size.x
	var h := size.y
	var lh := _fs + 5.0
	var bright := Color(1, 1, 1)
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.42, 0.42, 0.42)
	var faint := Color(0.25, 0.25, 0.25)

	# ---- title block (top-left) ----
	var y := MARGIN + lh
	_text(Vector2(MARGIN, y), "ASTEROID DEFENSE COMMAND", bright); y += lh
	_text(Vector2(MARGIN, y), "PDC/OS v2.6 - " + view_name, dim); y += lh * 1.4

	# ---- mission clock (top-right) ----
	var days_to := Sim.T_IMPACT - Sim.t
	var clock := ("E-%04d DAYS" % int(ceil(days_to))) if days_to >= 0.0 \
		else ("E+%04d DAYS" % int(-days_to))
	var cx := w - MARGIN
	_text_r(Vector2(cx, MARGIN + lh), clock, bright, _fs + 6)
	_text_r(Vector2(cx, MARGIN + lh * 2.3), "MJD-REL %07.1f  %s" % [Sim.t, Sim.date_string()], mid)
	var warp_str: String = "WARP " + Sim.warp_label()
	if Sim.paused:
		if Sim.blink(2.0):
			_text_r(Vector2(cx, MARGIN + lh * 3.5), "** HOLD **", bright)
	else:
		_text_r(Vector2(cx, MARGIN + lh * 3.5), warp_str, mid)

	# ---- target panel (left) ----
	var py := h * 0.30
	_panel(Rect2(MARGIN, py, PANEL_W, 9.4 * lh + 14.0), "TRK 001 - TARGET", bright)
	var ty := py + lh + 8.0
	var burned: bool = Sim.burned()
	var el: Dictionary = Sim.ast_defl_el if burned else Sim.ast_el
	var rng_km: float = Sim.threat_range_km(Sim.t)
	var lines := [
		["DESIG", "2031-XK" + ("  [DEFLECTED]" if burned else "")],
		["CLASS", "ATEN-TYPE / KINETIC-VIABLE"],
		["SMA   a", "%.4f AU" % el.a],
		["ECC   e", "%.4f" % el.e],
		["INC   i", "%.2f DEG" % rad_to_deg(el.i)],
		["PERIOD", "%.1f D" % (TAU / el.n)],
		["RANGE", _fmt_km(rng_km)],
	]
	for ln in lines:
		_text(Vector2(MARGIN + 12, ty), "%-8s %s" % [ln[0], ln[1]], mid)
		ty += lh
	if burned and Sim.deflect_ok:
		_text(Vector2(MARGIN + 12, ty), "P(IMPACT) 0.000", bright); ty += lh
		_text(Vector2(MARGIN + 12, ty), "PROJ MISS %.2f LD" % Sim.miss_ld, bright)
	elif burned:
		if Sim.blink(1.4):
			_text(Vector2(MARGIN + 12, ty), "P(IMPACT) 1.000 ", bright)
			_text(Vector2(MARGIN + 12 + 17 * _fs * 0.6, ty), "<INSUFFICIENT>", bright)
		ty += lh
		_text(Vector2(MARGIN + 12, ty), "PROJ MISS %.2f LD" % Sim.miss_ld, mid)
	else:
		if Sim.blink(1.4):
			_text(Vector2(MARGIN + 12, ty), "P(IMPACT) 1.000 ", bright)
			_text(Vector2(MARGIN + 12 + 17 * _fs * 0.6, ty), "<THREAT>", bright)
		ty += lh
		_text(Vector2(MARGIN + 12, ty), ("IMPACT E-%d D" % int(days_to)) if days_to >= 0.0
			else "IMPACT OCCURRED E+%d D" % int(-days_to), mid)

	# ---- interceptor panel (right) ----
	var ph := 7.4 * lh + 14.0
	var px := w - MARGIN - PANEL_W
	_panel(Rect2(px, py, PANEL_W, ph), "ATLAS-1 - INTERCEPTOR", bright)
	var iy := py + lh + 8.0
	var phase: String = Sim.interceptor_phase(Sim.t)
	var ilines := []
	match phase:
		"STANDBY":
			ilines = [
				["STATUS", "STANDBY - NO MISSION"],
				["THREAT", "P(IMPACT) 1.000 UNMITIGATED"],
				["PLAN", "AWAITING DEFLECTION PLAN"],
				["ACTION", "[M] OPEN MISSION PLANNER"],
			]
		"PRELAUNCH":
			ilines = [
				["STATUS", "PRELAUNCH / PAD"],
				["WINDOW", "E-%04d D" % int(Sim.T_IMPACT - Sim.T_LAUNCH)],
				["PLAN", "KINETIC IMPACTOR"],
				["BETA", "3.6 (EST)"],
				["DV EQ", "%.1f M/S AT INTERCEPT" % Sim.dv_ms],
				["LEAD", "%d D BEFORE EPOCH" % int(Sim.T_IMPACT - Sim.T_INTERCEPT)],
			]
		"CRUISE":
			var frac: float = clampf((Sim.t - Sim.T_LAUNCH) / (Sim.T_INTERCEPT - Sim.T_LAUNCH), 0.0, 1.0)
			ilines = [
				["STATUS", "CRUISE - GUIDANCE LOCK"],
				["XFER", _bar(frac) + " %3d%%" % int(frac * 100.0)],
				["TTI", "%d D" % int(Sim.T_INTERCEPT - Sim.t)],
				["BETA", "3.6 (EST)"],
				["DV EQ", "%.1f M/S AT INTERCEPT" % Sim.dv_ms],
				["LEAD", "%d D BEFORE EPOCH" % int(Sim.T_IMPACT - Sim.T_INTERCEPT)],
			]
		_:
			ilines = [
				["STATUS", "EXPENDED - IMPACT GOOD"],
				["RESULT", "DV %.1f M/S %s" % [Sim.dv_ms,
					"RETROGRADE" if Sim.plan_retro else "PROGRADE"]],
				["MISS", "%.2f LD PROJECTED" % Sim.miss_ld],
				["ASSESS", "DEFLECTION SUCCESSFUL" if Sim.deflect_ok
					else "INSUFFICIENT - IMPACT"],
			]
	for ln in ilines:
		_text(Vector2(px + 12, iy), "%-8s %s" % [ln[0], ln[1]], mid)
		iy += lh

	# ---- console (bottom-left) ----
	var rows: int = _console.size()
	var cy := h - MARGIN - (rows + 1) * lh
	_text(Vector2(MARGIN, cy - 4), "-- EVENT LOG " + "-".repeat(38), faint)
	for k in rows:
		var line := _console[k]
		if k == rows - 1:
			line = line.substr(0, int(_reveal))
		_text(Vector2(MARGIN, cy + (k + 1) * lh), line, mid if k < rows - 1 else bright)
	# blinking prompt cursor
	var last_w := _font.get_string_size(_console[rows - 1].substr(0, int(_reveal)),
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs).x
	if Sim.blink(2.5):
		_text(Vector2(MARGIN + last_w + 4, cy + rows * lh), "_", bright)

	# ---- help line (bottom-right) ----
	_text_r(Vector2(w - MARGIN, h - MARGIN),
		"[SPC]HOLD [,/.]WARP [J]JUMP [M]PLAN [F]FOCUS:%s [1]3D [2]MAP [3]ENC [T]PHOSPHOR" % camera_rig.focus_name,
		dim, _fs - 2)

	# ---- frame corners (screen bezel ticks) ----
	var tick := 26.0
	for corner in [Vector2(6, 6), Vector2(w - 6, 6), Vector2(6, h - 6), Vector2(w - 6, h - 6)]:
		var sx: float = 1.0 if corner.x < w * 0.5 else -1.0
		var sy: float = 1.0 if corner.y < h * 0.5 else -1.0
		draw_line(corner, corner + Vector2(tick * sx, 0), faint, 1.5)
		draw_line(corner, corner + Vector2(0, tick * sy), faint, 1.5)


# ------------------------------------------------------------------ helpers ---

func _text(pos: Vector2, s: String, col: Color, fs: int = -1) -> void:
	draw_string(_font, pos, s, HORIZONTAL_ALIGNMENT_LEFT, -1,
		fs if fs > 0 else _fs, col)


func _text_r(pos: Vector2, s: String, col: Color, fs: int = -1) -> void:
	var f := fs if fs > 0 else _fs
	var sw := _font.get_string_size(s, HORIZONTAL_ALIGNMENT_LEFT, -1, f).x
	draw_string(_font, pos - Vector2(sw, 0), s, HORIZONTAL_ALIGNMENT_LEFT, -1, f, col)


func _panel(rect: Rect2, title: String, col: Color) -> void:
	var dim := Color(0.35, 0.35, 0.35)
	draw_rect(rect, dim, false, 1.2)
	# notched title
	var tw := _font.get_string_size(" " + title + " ",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs).x
	draw_rect(Rect2(rect.position.x + 8, rect.position.y - _fs * 0.7, tw, _fs * 1.2),
		Color(0, 0, 0), true)
	draw_string(_font, Vector2(rect.position.x + 12, rect.position.y + _fs * 0.35),
		title, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)


func _bar(frac: float, width: int = 12) -> String:
	var fill := int(round(frac * width))
	return "[" + "=".repeat(fill) + " ".repeat(width - fill) + "]"


func _fmt_km(km: float) -> String:
	if km > 1.0e7:
		return "%.3f AU" % (km / Sim.AU_KM)
	return "%s KM" % _group(int(km))


func _group(v: int) -> String:
	var s := str(v)
	var out := ""
	while s.length() > 3:
		out = "," + s.right(3) + out
		s = s.left(s.length() - 3)
	return s + out
