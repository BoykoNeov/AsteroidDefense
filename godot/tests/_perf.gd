extends Node
## Frame-time harness — measures what each view costs per frame, in numbers.
##
## Registered as an autoload (`Perf="*res://tests/_perf.gd"`) and driven NON-headless
## with vsync off so the frame rate is the work, not the monitor:
##   godot --path godot --resolution 1600x900 --disable-vsync
## then removed from project.godot again. Not part of the shipping game.
##
## Why it exists: every view here redraws every frame (`queue_redraw` in `_process`,
## because things blink and the clock runs), so a per-frame cost hides in plain sight
## — nothing errors, the picture is right, and the fan spins. The only way to know a
## view is cheap is to measure it, and the only way to know a change helped is to
## measure it again. Numbers this prints, per view, over a fixed frame budget:
##
##   process ms   — main-thread script+engine time per frame (Performance.TIME_PROCESS)
##   fps          — frames per second with vsync off (the whole frame, GPU included)
##   ffi/frame    — native binding calls Sim made per frame (`Sim.ffi_calls`)
##   draw calls   — RenderingServer draw calls per frame
##
## Output goes to stdout and to `W:/temp/claude/AsteroidDefense/perf/<stamp>.txt`
## so two runs can be diffed.

const OUT_DIR := "W:/temp/claude/AsteroidDefense/perf"
const FRAMES := 240

var _report: Array[String] = []


func _ready() -> void:
	call_deferred("_run")


func _run() -> void:
	await get_tree().process_frame
	var main := get_tree().root.get_node_or_null("Main")
	if main == null:
		for c in get_tree().root.get_children():
			if c.get_script() != null and c.has_method("_apply_focus"):
				main = c
				break
	if main == null:
		print("PERF  FAIL: no Main node")
		get_tree().quit(1)
		return

	DirAccess.make_dir_recursive_absolute(OUT_DIR)
	main.boot.dismiss()
	await _settle(6)

	var t0 := Time.get_ticks_msec()
	while not Sim.mission_online and Time.get_ticks_msec() - t0 < 90000:
		await get_tree().process_frame
	_say("PERF  mission_online=%s after %d ms  (build wait, not a frame cost)"
		% [Sim.mission_online, Time.get_ticks_msec() - t0])
	_say("PERF  window %s  vsync=%s  headless=%s  frames/view=%d" % [
		get_viewport().size, DisplayServer.window_get_vsync_mode(),
		DisplayServer.get_name() == "headless", FRAMES])
	_microbench()

	# The clock runs at a mid warp so every body moves every frame — a paused clock
	# would let a lazy view look cheap.
	Sim.paused = false
	Sim.jump(0.0)
	Sim.warp_idx = 4

	# 1. 3D tactical, system scale (the default screen).
	main._show_view(null)
	main.tags.visible = true
	main.hud.view_name = "TACTICAL 3D"
	await _settle(10)
	await _measure("3d_system")

	# 2. 3D tactical, Earth close-up (moon + wire Earth fill the frame).
	main._focus_idx = 3            # SUN, MERCURY, VENUS, EARTH
	main._apply_focus()
	await _settle(30)
	await _measure("3d_earth_closeup")
	main._focus_idx = 0
	main._apply_focus()

	# 3. 3D with the planner open (deflected preview track drawn).
	Sim.set_plan(Sim.threat_period_d(), 0.2, true)
	Sim._tick_plan_debounce(1.0)
	main.planner.visible = true
	Sim.planner_open = true
	await _settle(10)
	await _measure("3d_planner_open")
	main.planner.visible = false
	Sim.planner_open = false

	# 4. The 2D heliocentric plot.
	main._show_view(main.map2d)
	main.hud.view_name = "HELIO PLOT 2D"
	await _settle(10)
	await _measure("map2d")

	# 5. The b-plane view with the keyhole map on.
	main._show_view(main.enc)
	main.hud.view_name = "ENCOUNTER B-PLANE"
	main.enc._keyholes = true
	await _settle(10)
	await _measure("encounter_keyholes")
	main.enc._half_ld = 1.2
	await _settle(5)
	await _measure("encounter_zoomed_out")
	main.enc._half_ld = 0.15

	# 6. The launch-window map (grid solved on demand first).
	main._show_view(main.pork)
	main.hud.view_name = "LAUNCH WINDOWS"
	Sim.request_porkchop()
	var t1 := Time.get_ticks_msec()
	while not Sim.pork_online and Time.get_ticks_msec() - t1 < 60000:
		await get_tree().process_frame
	await _settle(10)
	await _measure("porkchop")

	# 7. Back to 3D, clock at max warp (bodies sweep whole orbits per second).
	main._show_view(null)
	main.tags.visible = true
	main.hud.view_name = "TACTICAL 3D"
	Sim.warp_idx = Sim.WARP_STEPS.size() - 1
	Sim.jump(0.0)
	await _settle(10)
	await _measure("3d_max_warp")

	var stamp := Time.get_datetime_string_from_system().replace(":", "-")
	var f := FileAccess.open("%s/%s.txt" % [OUT_DIR, stamp], FileAccess.WRITE)
	if f != null:
		for line in _report:
			f.store_line(line)
		f.close()
		print("PERF  wrote %s/%s.txt" % [OUT_DIR, stamp])
	get_tree().quit(0)


## Average the monitors over FRAMES frames and report one line.
func _measure(label: String) -> void:
	var frame_max := 0.0
	var draw_sum := 0.0
	var ffi_sum := 0
	var wall0 := Time.get_ticks_usec()
	var last := wall0
	for _i in FRAMES:
		await get_tree().process_frame
		var now := Time.get_ticks_usec()
		frame_max = maxf(frame_max, float(now - last) / 1000.0)
		last = now
		draw_sum += Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME)
		ffi_sum += Sim.ffi_calls
	var wall_s := float(Time.get_ticks_usec() - wall0) / 1.0e6
	var n := float(FRAMES)
	# Frame time is the wall clock between consecutive frames — the number a player
	# feels. Windowed, it is pinned to the monitor's refresh by the desktop
	# compositor even with vsync off, so a windowed run only shows a view that is
	# *slower* than the display; run `--headless` (the draw paths still execute for
	# visible nodes) to see the CPU cost of each view uncapped.
	# `Performance.TIME_PROCESS` was reported here once and dropped: it returned
	# hundreds of milliseconds beside an 8 ms frame, so whatever it measures on this
	# build it is not the process step.
	_say("PERF  %-22s frame %6.2f ms avg %7.2f max | %6.1f fps | ffi/frame %5d | draw calls %5d"
		% [label, 1000.0 * wall_s / n, frame_max, n / wall_s,
			int(round(ffi_sum / n)), int(round(draw_sum / n))])


## Time the per-frame lookups the views are built from, in isolation.
##
## Frame times bundle everything; these say what one call costs, so a change to
## `Sim.pos3d` or `Sim.orbit_points` can be priced without guessing which view moved.
func _microbench() -> void:
	var t: float = Sim.t
	var reps := 500
	var us := func(f: Callable, n: int) -> float:
		var t0 := Time.get_ticks_usec()
		for _i in n:
			f.call()
		return float(Time.get_ticks_usec() - t0) / float(n)
	# Raw lookups (the native call plus marshalling), then the memoized front door
	# asked for the clock's epoch — which after the first call is a memo hit.
	_say("PERF  micro  lookup(EARTH)  raw        %8.1f us" % us.call(func(): Sim._lookup_ecl(Sim.earth_el, t), reps))
	_say("PERF  micro  lookup(THREAT) raw        %8.1f us" % us.call(func(): Sim._lookup_ecl(Sim.ast_el, t), reps))
	if Sim.comet_online:
		_say("PERF  micro  lookup(COMET)  raw        %8.1f us" % us.call(func(): Sim._lookup_ecl(Sim.comet_el, t), reps))
	_say("PERF  micro  pos3d(EARTH)   memo hit   %8.1f us" % us.call(func(): Sim.pos3d(Sim.earth_el, t), reps))
	_say("PERF  micro  threat_range_km memo hit  %8.1f us" % us.call(func(): Sim.threat_range_km(t), reps))
	_say("PERF  micro  orbit_points(EARTH,180)  %8.1f us" % us.call(func(): Sim.orbit_points(Sim.earth_el, 180), 20))
	_say("PERF  micro  orbit_points(THREAT,180) %8.1f us" % us.call(func(): Sim.orbit_points(Sim.ast_el, 180), 20))
	_say("PERF  micro  orbit_points(MARS,384)   %8.1f us" % us.call(func(): Sim.orbit_points(Sim.planets[3], 384), 20))


func _settle(frames: int) -> void:
	for _i in frames:
		await get_tree().process_frame


func _say(line: String) -> void:
	print(line)
	_report.append(line)
