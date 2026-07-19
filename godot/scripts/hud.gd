class_name HUD
extends Control
## Retro terminal HUD: title block, mission clock, target/interceptor data
## panels, console log with typewriter reveal, help line. Drawn white/gray —
## the CRT shader maps everything to the phosphor palette.

const MARGIN := 18.0
const PANEL_W := 340.0
const BOTTOM_RESERVE := 50.0         # clearance for the TimeBar scrub strip

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
	# An "E-nnnn DAYS" countdown only means something when there is a threat to
	# count down to. With the mission layer dormant it read "E-69001 DAYS" at the
	# far end of the scrub — a countdown to an impact that is not being drawn.
	# Show the epoch instead: that is what the clock is now.
	var days_to := Sim.T_IMPACT - Sim.t
	var clock := Sim.date_string()
	if Sim.mission_online:
		clock = ("E-%04d DAYS" % int(ceil(days_to))) if days_to >= 0.0 \
			else ("E+%04d DAYS" % int(-days_to))
	var cx := w - MARGIN
	_text_r(Vector2(cx, MARGIN + lh), clock, bright, _fs + 6)
	_text_r(Vector2(cx, MARGIN + lh * 2.3), "TDB EPOCH  MJD-REL %+08.1f D" % Sim.t
		if not Sim.mission_online
		else "MJD-REL %07.1f  %s" % [Sim.t, Sim.date_string()], mid)
	var warp_str: String = "WARP " + Sim.warp_label()
	if Sim.paused:
		if Sim.blink(2.0):
			_text_r(Vector2(cx, MARGIN + lh * 3.5), "** HOLD **", bright)
	else:
		_text_r(Vector2(cx, MARGIN + lh * 3.5), warp_str, mid)

	# ---- target panel (left) ----
	var py := h * 0.30
	if not Sim.mission_online:
		_offline_panels(Rect2(MARGIN, py, PANEL_W, 9.4 * lh + 14.0), lh, bright, mid, dim)
		_console_block(w, h, lh, mid, bright, faint)
		_help_line(w, h, dim)
		_bezel(w, h, faint)
		return
	_panel(Rect2(MARGIN, py, PANEL_W, 7.4 * lh + 14.0), "TRK 001 - TARGET", bright)
	var ty := py + lh + 8.0
	var burned: bool = Sim.burned() and Sim.has_plan()
	var active: bool = Sim.threat_active(Sim.t)
	var rng_km: float = Sim.threat_range_km(Sim.t)
	# SMA and PERIOD are the core's, measured off the integrated seed by vis-viva
	# — not designer inputs echoed back. ECC and INC are gone rather than faked:
	# the core derives no eccentricity or inclination, and the old panel's values
	# were the placeholder's own constructor arguments. A number the physics never
	# computed has no business on a panel labelled TARGET.
	var lines := [
		["DESIG", "2031-XK" + ("  [DEFLECTED]" if burned else "")],
		["CLASS", "ATEN-TYPE / KINETIC-VIABLE"],
		["SMA   a", "%.4f AU" % Sim.ast_el.a],
		["PERIOD", "%.1f D" % Sim.threat_period_d()],
		["RANGE", _fmt_km(rng_km) if active else "-- OUTSIDE TRACKED ARC"],
	]
	for ln in lines:
		_text(Vector2(MARGIN + 12, ty), "%-8s %s" % [ln[0], ln[1]], mid)
		ty += lh
	if not active:
		# The clock is off the threat's ~12-year arc. There is no object to assign
		# an impact probability to, so the panel says where the arc is instead.
		_text(Vector2(MARGIN + 12, ty), "TRACK ARC %s" % Sim.threat_arc_label(), mid)
	elif burned and Sim.deflect_ok:
		_text(Vector2(MARGIN + 12, ty), "P(IMPACT) 0.000", bright); ty += lh
		_text(Vector2(MARGIN + 12, ty), "PROJ MISS " + Sim.miss_label(), bright)
	elif burned:
		if Sim.blink(1.4):
			_text(Vector2(MARGIN + 12, ty), "P(IMPACT) 1.000 ", bright)
			_text(Vector2(MARGIN + 12 + 17 * _fs * 0.6, ty), "<INSUFFICIENT>", bright)
		ty += lh
		_text(Vector2(MARGIN + 12, ty), "PROJ MISS " + Sim.miss_label(), mid)
	else:
		if Sim.blink(1.4):
			_text(Vector2(MARGIN + 12, ty), "P(IMPACT) 1.000 ", bright)
			_text(Vector2(MARGIN + 12 + 17 * _fs * 0.6, ty), "<THREAT>", bright)
		ty += lh
		_text(Vector2(MARGIN + 12, ty), ("IMPACT E-%d D" % int(days_to)) if days_to >= 0.0
			else "IMPACT OCCURRED E+%d D" % int(-days_to), mid)

	# ---- interceptor panel (right) ----
	# The interceptor itself is dormant (no Lambert solver behind its arc), but the
	# panel stays: it is where the plan is read back, and the plan is real. The
	# phases below describe the *plan's* timeline, which the core solves against —
	# not a spacecraft being flown.
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
				["MISS", Sim.miss_label() + " PROJECTED"],
				["ASSESS", Sim.verdict_label()],
			]
	for ln in ilines:
		_text(Vector2(px + 12, iy), "%-8s %s" % [ln[0], ln[1]], mid)
		iy += lh

	_console_block(w, h, lh, mid, bright, faint)
	_help_line(w, h, dim)
	_bezel(w, h, faint)


# ------------------------------------------------------------------ panels ---

## What the left/right panels say before the threat exists — during the ~10 s
## boot integration, or after a failed build.
##
## Deliberately states the situation instead of showing zeroed-out target and
## interceptor readouts: a panel reading "SMA 0.0000 AU / P(IMPACT) 0.000" looks
## like a live instrument reporting a dead threat, which is a lie. Empty fields
## are not neutral on an instrument panel — they read as measurements.
func _offline_panels(rect: Rect2, lh: float, bright: Color, mid: Color, dim: Color) -> void:
	_panel(rect, "TRK 001 - TARGET", bright)
	var ty := rect.position.y + lh + 8.0
	var x := rect.position.x + 12
	# This panel is now mostly a *loading* state, not a permanent one: the threat
	# is ~10 s of real integration away at boot. It must not read like a dead
	# subsystem while the core is working, nor claim to be working after a failure.
	var lines: Array[String] = []
	match Sim.build_state:
		Sim.Build.RUNNING:
			lines = [
				"ACQUIRING THREAT SOLUTION",
				"",
				"INTEGRATING 12 YR OF REAL",
				"N-BODY MOTION THROUGH THE",
				"DE440 FIELD. STAND BY...",
				"",
			]
		Sim.Build.FAILED:
			lines = [
				"*** NO THREAT SOLUTION ***",
				"",
				"THE TRAJECTORY BUILD FAILED.",
				"SEE THE CONSOLE FOR THE",
				"REASON. NO THREAT IS DRAWN.",
				"",
			]
		_:
			lines = [
				"MISSION LAYER OFFLINE",
				"",
				"NO EPHEMERIS, SO NO FIELD",
				"TO INTEGRATE A THREAT",
				"THROUGH.",
				"",
			]
	# The field is a separate subsystem from the mission layer and fails
	# separately. This panel used to state "SOLAR FIELD IS LIVE" unconditionally,
	# which was flatly false on a machine with no kernel — the same lie as the
	# threat countdown, one panel over. Report what is actually up.
	if Sim.bodies_online:
		lines.append("SOLAR FIELD IS LIVE:")
		lines.append("REAL DE440 EPHEMERIS.")
	else:
		lines.append("SOLAR FIELD OFFLINE:")
		lines.append("NO EPHEMERIS KERNEL FOUND.")
		lines.append("SEE BOOT LOG FOR PATHS.")
	for ln in lines:
		_text(Vector2(x, ty), ln, mid if ln.begins_with("REAL") or ln.begins_with("SOLAR")
			or ln.begins_with("NO ") else dim)
		ty += lh
	if Sim.blink(1.4):
		var tag := "-- DEGRADED --"
		if Sim.build_state == Sim.Build.RUNNING:
			tag = "-- COMPUTING THREAT TRAJECTORY --"
		_text(Vector2(x, ty), tag, bright)


func _console_block(w: float, h: float, lh: float, mid: Color, bright: Color,
		faint: Color) -> void:
	var rows: int = _console.size()
	var cy := h - MARGIN - BOTTOM_RESERVE - (rows + 1) * lh
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


func _help_line(w: float, h: float, dim: Color) -> void:
	var line := "[SPC]HOLD [,/.]WARP [B]REV [J]JUMP [M]PLAN [F]FOCUS:%s" % camera_rig.focus_name
	# [C] only binds while the encounter view is up, so it is only advertised
	# there — a key listed everywhere that works in one place is its own small lie,
	# and an unlisted key in the one view that needs it is why the closest-approach
	# marker was effectively unreachable in the first place.
	if view_name == "ENCOUNTER B-PLANE":
		line += " [C]CLOSEST APPR"
	line += " [1]3D [2]MAP [3]ENC [T]PHOSPHOR"
	_text_r(Vector2(w - MARGIN, h - MARGIN - BOTTOM_RESERVE), line, dim, _fs - 2)


## Screen bezel ticks at the four frame corners.
func _bezel(w: float, h: float, faint: Color) -> void:
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
