extends Node
## Temporary launch-window-map harness — the only thing that runs PorkchopPlot._draw().
##   godot --path godot --resolution 1600x900   (non-headless → a real PNG)
##   godot --headless --path godot              (_draw still runs for VISIBLE nodes)
## Registered as an autoload while running, removed afterwards. Not shipped.
##
## Why this exists: `_draw` runs under `--headless`, but **only for visible nodes**,
## and the porkchop view is hidden until [4]. So its cell loop, its ramp, its
## readout formatting and its verdict line never execute in a passive run — the
## exact gap that let a wrong panel ship a whole phase elsewhere in this project.
##
## Every step goes through `main._input()` with a real key event, so the chain
## being tested is project.godot action → main.gd handler → Sim → core, not a
## direct call that would bypass the part most likely to be miswired.

const OUT := "M:/claud_projects/temp/AsteroidDefense/shots"


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
		print("PORKSHOT  FAIL: no Main node")
		get_tree().quit(1)
		return

	DirAccess.make_dir_recursive_absolute(OUT)
	main.boot.dismiss()
	await _settle(6)

	var t0 := Time.get_ticks_msec()
	while not Sim.mission_online and Time.get_ticks_msec() - t0 < 300000:
		await get_tree().process_frame
	print("PORKSHOT  mission_online=%s after %d ms" % [Sim.mission_online, Time.get_ticks_msec() - t0])
	if not Sim.mission_online:
		print("PORKSHOT  FAIL: no threat solution")
		get_tree().quit(1)
		return

	# [4] must switch the view AND kick the grid. Both, from one key.
	assert(not main.pork.visible, "the map starts hidden")
	main._input(_key(KEY_4))
	print("PORKSHOT  [4] -> visible=%s building=%s (expect true/true)"
		% [main.pork.visible, Sim.pork_building])
	assert(main.pork.visible, "[4] must open the launch-window map via the action")
	assert(Sim.pork_building, "[4] must kick the grid solve")
	# The view switch must be exclusive — two stacked overlays paint over each other.
	assert(not main.enc.visible and not main.map2d.visible, "[4] must hide the other views")

	var t1 := Time.get_ticks_msec()
	while Sim.pork_building and Time.get_ticks_msec() - t1 < 180000:
		await get_tree().process_frame
	print("PORKSHOT  pork_online=%s grid=%dx%d after %d ms"
		% [Sim.pork_online, Sim.pork_rows, Sim.pork_cols, Time.get_ticks_msec() - t1])
	assert(Sim.pork_online, "the grid must land: " + str(Sim.mission.last_error()))
	assert(Sim.pork_rows == Sim.PORK_LAUNCH_SAMPLES and Sim.pork_cols == Sim.PORK_ARRIVAL_SAMPLES,
		"the grid came back a different size than requested")

	# The three blanks, counted — the claim the whole picture rests on. If
	# "unreachable" were zero the dim floor would be invisible and the view would
	# be lying by omission rather than by drawing.
	var blank := 0
	var unreach := 0
	var reach := 0
	for i in range(Sim.pork_rows):
		for j in range(Sim.pork_cols):
			if Sim.pork_blank(i, j):
				blank += 1
			elif Sim.pork_reachable(i, j):
				reach += 1
			else:
				unreach += 1
	print("PORKSHOT  cells: %d blank / %d unreachable / %d reachable (%s)"
		% [blank, unreach, reach, Sim.pork_vehicle_name()])
	assert(reach > 0, "no reachable window at all — the heatmap would be empty")
	assert(unreach > 0, "no unreachable-but-real window — the dim floor never draws")
	assert(blank > 0, "no blank cell — the below-diagonal gap is missing")

	# Per-launcher reach at shipping resolution, and the grid's cheapest cell.
	# This is where `payload_kg`'s flat-hold below the table shows: Atlas V and
	# Delta IV are tabulated from C3 0.0 and -9.24 so they were never affected,
	# while the three that start at 1.0 used to lose every cell below it. Counted
	# for every vehicle rather than just the selected one, because the count for
	# the one launcher that happens to be showing cannot demonstrate that.
	var cheapest := INF
	for k in range(Sim.pork_c3.size()):
		if Sim.pork_c3[k] >= 0.0:
			cheapest = minf(cheapest, Sim.pork_c3[k])
	print("PORKSHOT  cheapest cell C3 = %.3f km2/s2" % cheapest)
	for v in range(Sim.mission.vehicle_count()):
		var pay: PackedFloat64Array = Sim.mission.porkchop_payload_kg(v)
		var n := 0
		for k in range(Sim.pork_c3.size()):
			if Sim.pork_c3[k] >= 0.0 and pay[k] > 0.0:
				n += 1
		print("PORKSHOT    %-28s ceiling C3 %6.1f  reaches %5d windows"
			% [str(Sim.mission.vehicle_name(v)).to_upper(),
				Sim.mission.vehicle_max_c3(v), n])
		assert(n > 0, "a launcher that reaches no window at all")

	# Park the cursor on the best-coupled reachable window, so the readout and the
	# verify below describe a cell worth describing.
	var best := -1.0
	for i in range(Sim.pork_rows):
		for j in range(Sim.pork_cols):
			if not Sim.pork_reachable(i, j):
				continue
			var v: float = absf(Sim.pork_dv[i * Sim.pork_cols + j])
			if v > best:
				best = v
				Sim.pork_i = i
				Sim.pork_j = j
	var cell := Sim.pork_cell()
	print("PORKSHOT  cursor (%d,%d): C3 %.2f km2/s2  N=%d  TOF %.0f d  %.0f kg  dv %+.5f mm/s"
		% [Sim.pork_i, Sim.pork_j, float(cell.c3_km2_s2), int(cell.revolutions),
			float(cell.tof_days), float(cell.payload_kg),
			float(cell.along_track_dv_ms) * 1000.0])
	assert(not cell.is_empty(), "the cursor cell must have a readout row")

	# [L] cycles the launcher and must actually change the delivered mass — the
	# vehicle-independent grid's whole payoff, driven through the real key.
	var before_name := Sim.pork_vehicle_name()
	var before_kg := float(cell.payload_kg)
	main._input(_key(KEY_L))
	await _settle(2)
	var after := Sim.pork_cell()
	print("PORKSHOT  [L] %s (%.0f kg) -> %s (%.0f kg)"
		% [before_name, before_kg, Sim.pork_vehicle_name(), float(after.payload_kg)])
	assert(Sim.pork_vehicle_name() != before_name, "[L] must change the launcher")
	main._input(_key(KEY_L))
	main._input(_key(KEY_L))
	main._input(_key(KEY_L))
	main._input(_key(KEY_L))
	await _settle(2)
	assert(Sim.pork_vehicle_name() == before_name, "[L] must wrap back round the table")

	# [D] cycles the metric; the ramp must rebuild rather than keep C3 bounds.
	main._input(_key(KEY_D))
	await _settle(3)
	print("PORKSHOT  [D] -> metric %s" % str(Sim.PORK_METRICS[Sim.pork_metric][1]))
	await _shot("porkchop_delivered_dv")
	main._input(_key(KEY_D))
	await _settle(3)
	print("PORKSHOT  [D] -> metric %s" % str(Sim.PORK_METRICS[Sim.pork_metric][1]))

	# [E] fires the one number in the view that is not a patched-conic estimate.
	main._input(_key(KEY_E))
	print("PORKSHOT  [E] -> verifying=%s" % Sim.pork_verifying)
	assert(Sim.pork_verifying, "[E] must fire the full-field verify")
	var t2 := Time.get_ticks_msec()
	while Sim.pork_verifying and Time.get_ticks_msec() - t2 < 180000:
		await get_tree().process_frame
	print("PORKSHOT  verdict after %d ms: %s" % [Time.get_ticks_msec() - t2, Sim.pork_verdict_label()])
	var v := Sim.pork_verdict()
	assert(not v.is_empty(), "the verify must land a verdict: " + str(Sim.mission.last_error()))
	assert(Sim.pork_verdict_is_current(), "the verdict must be labelled with the cursor's cell")
	# The verdict is |B| vs the capture radius — the coherent pair, and the same
	# comparison the core's own is_hit makes. Re-derived here so a frontend that
	# started reading the perigee against the capture disc fails loudly.
	if str(v.outcome) == "encounter":
		var b: float = float(v.impact_parameter_m)
		var cap: float = float(v.capture_radius_m)
		print("PORKSHOT  |B| %.0f m vs capture %.0f m -> is_hit=%s (perigee %.0f m vs R_E %.0f m)"
			% [b, cap, bool(v.is_hit), float(v.perigee_m), float(v.earth_radius_m)])
		assert(bool(v.is_hit) == (b <= cap),
			"is_hit disagrees with |B| vs capture — the frontend is reading the wrong pair")

	await _settle(4)
	await _shot("porkchop_c3_verified")
	get_tree().quit(0)


## A pressed key event for `keycode`, fed to main._input() so the real action
## handlers (project.godot → main.gd → Sim) run rather than being bypassed.
func _key(keycode: int) -> InputEventKey:
	var ev := InputEventKey.new()
	ev.keycode = keycode
	ev.pressed = true
	return ev


func _settle(frames: int) -> void:
	for _i in frames:
		await get_tree().process_frame


func _shot(name: String) -> void:
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var path := "%s/%s.png" % [OUT, name]
	img.save_png(path)
	print("PORKSHOT  wrote %s" % path)
