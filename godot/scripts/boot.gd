class_name BootScreen
extends Control
## Boot/POST typewriter overlay shown at startup. Dismissed by any key
## (handled in main.gd) or auto-fades after the sequence completes.
##
## The POST reports the machine's ACTUAL state rather than a fixed script. It
## used to assert "EPHEMERIS KERNEL ... DE440S LOADED" and "DEFLECTION SOLVER
## ... ONLINE" unconditionally, which since 3C-2a can be plainly untrue — and a
## power-on self-test that always passes is the one piece of set dressing that
## must not be dressing. When the kernels cannot be found this is where the
## operator is told, in full, including how to fix it.

signal finished

var _lines: Array[String] = []
var _chars := 0.0
var _done := false
var _idle := 0.0
var _font: Font
var _fs := 16


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP
	set_anchors_preset(Control.PRESET_FULL_RECT)
	_font = Sim.mono_font
	_lines = _post_lines()


## The POST, built from what actually came up.
func _post_lines() -> Array[String]:
	var out: Array[String] = [
		"ASTEROID DEFENSE COMMAND - PDC/OS v2.6",
		"COPYRIGHT (C) 2031 PLANETARY DEFENSE COORDINATION OFFICE",
		"",
		"MEMORY TEST ................ 65536 KB OK",
	]

	if Sim.bodies_online:
		out.append("EPHEMERIS KERNEL ........... %s LOADED" %
			Sim.kernel_source.get_file().to_upper())
		out.append("PROPAGATOR ................. DOP853 F64 [RUST CORE]")
		out.append("EPHEMERIS SPAN ............. %d - %d" %
			[Sim.year_at(Sim.T_MIN), Sim.year_at(Sim.T_MAX)])
		out.append("SOLAR FIELD ................ %d BODIES - REAL DE440" %
			Sim.planets.size())
	else:
		out.append("EPHEMERIS KERNEL ........... *** NOT FOUND ***")
		out.append("SOLAR FIELD ................ OFFLINE - NO BODIES DRAWN")
		out.append("")
		for l in Sim.kernel_error.split("\n"):
			out.append("! " + l)

	out.append("")
	# This is a snapshot: the POST is typed once at _ready, while the threat is
	# still integrating on its worker thread (~10 s). So it reports the build as
	# RUNNING rather than claiming either "online" or "offline" — both would be
	# false at the moment this text is written.
	match Sim.build_state:
		Sim.Build.READY:
			out.append("DEFLECTION SOLVER .......... ONLINE [RUST CORE]")
			out.append("MISSION PLANNER ............ READY - KEY [M]")
			out.append("THREAT DB .................. 1 OBJECT(S) FLAGGED")
		Sim.Build.RUNNING:
			out.append("DEFLECTION SOLVER .......... ONLINE [RUST CORE]")
			out.append("THREAT TRAJECTORY .......... INTEGRATING REAL FIELD...")
			out.append("MISSION PLANNER ............ ARMS ON SOLUTION - KEY [M]")
		Sim.Build.FAILED:
			out.append("DEFLECTION SOLVER .......... *** SOLUTION FAILED ***")
			out.append("MISSION PLANNER ............ OFFLINE")
			for l in Sim.build_error.split("\n"):
				out.append("! " + l)
		_:
			out.append("DEFLECTION SOLVER .......... OFFLINE - NO EPHEMERIS")
			out.append("MISSION PLANNER ............ OFFLINE")

	# The b-plane view is a separate subsystem and separately dormant (3C-2c) —
	# reporting it green because the solver is up is exactly the kind of blanket
	# self-test this POST exists to not be.
	out.append("B-PLANE TARGETING .......... OFFLINE - REBUILDING ON CORE")

	out.append("")
	if not Sim.bodies_online:
		out.append("*** DEGRADED - NO EPHEMERIS ***")
	elif Sim.build_state == Sim.Build.FAILED:
		out.append("*** DEGRADED - ORRERY ONLINE, NO THREAT SOLUTION ***")
	else:
		out.append("ALL SYSTEMS NOMINAL")
	out.append("")
	out.append("PRESS ANY KEY TO ENGAGE TACTICAL DISPLAY")
	return out


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
	for l in _lines:
		n += l.length() + 1
	return n


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, size), Color(0, 0, 0), true)
	var budget := int(_chars)
	var y := size.y * 0.14
	var x := size.x * 0.14
	var lh := _fs + 8.0
	for l in _lines:
		if budget <= 0:
			break
		var line: String = l.substr(0, mini(l.length(), budget))
		budget -= l.length() + 1
		# Failures ("!"/"***") and headlines read bright; the rest is routine.
		var col := Color(0.75, 0.75, 0.75)
		if l.begins_with("!") or l.contains("***"):
			col = Color(1, 1, 1)
		elif l.begins_with(">") or l.begins_with("ALL") or l.begins_with("PRESS") \
				or l.begins_with("ORRERY"):
			col = Color(1, 1, 1)
		draw_string(_font, Vector2(x, y), line, HORIZONTAL_ALIGNMENT_LEFT, -1,
			_fs - 3 if l.begins_with("!") else _fs, col)
		y += lh
	if Sim.blink(2.5):
		draw_string(_font, Vector2(x, y), "_", HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, Color(1, 1, 1))
