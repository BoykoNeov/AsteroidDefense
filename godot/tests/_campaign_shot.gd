extends Node
## Temporary harness for the launch campaign on the launch-window map ([4] then [C]).
##   powershell -File godot/tests/run_harness.ps1 -Harness _campaign_shot.gd
## Registered as an autoload while running, removed afterwards. Not shipped.
##
## The campaign panel only draws while the map is up AND [C] has swapped it in, so
## a passive run never executes a line of it. Every step goes through `main._input()`
## with a real key event: project.godot action -> main.gd guard -> Sim -> core.
##
## What it pins, beyond "it drew":
## - the cap is per YEAR: no year of the plan carries more launches than the rate;
## - the rate knob is free (no solve fires on [Z]/[X]) and greys the flight line;
## - 1/yr is an answer ("FALLS SHORT"), not a failure;
## - [L] makes the held campaign stale instead of showing it as this rocket's;
## - [M] does not open the planner or start a mass solve while the panel is up.

const OUT := "W:/temp/claude/AsteroidDefense/shots"


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
		print("CAMPSHOT  FAIL: no Main node")
		get_tree().quit(1)
		return
	DirAccess.make_dir_recursive_absolute(OUT)
	main.boot.dismiss()
	await _settle(6)

	var t0 := Time.get_ticks_msec()
	while not Sim.mission_online and Time.get_ticks_msec() - t0 < 300000:
		await get_tree().process_frame
	assert(Sim.mission_online, "no threat solution")
	main._input(_key(KEY_4))
	var t1 := Time.get_ticks_msec()
	while Sim.pork_building and Time.get_ticks_msec() - t1 < 180000:
		await get_tree().process_frame
	assert(Sim.pork_online, "the grid must land: " + str(Sim.mission.last_error()))
	# The headline launcher. The map opens on the first row of the table (Atlas V
	# 551), which cannot reach the target at 2/yr at all - an answer the panel must
	# also render, but not the one the flight checks below need.
	for _k in Sim.mission.vehicle_count():
		if Sim.pork_vehicle_name().contains("FALCON HEAVY") and Sim.pork_vehicle_name().contains("EXP"):
			break
		main._input(_key(KEY_L))
	await _settle(2)
	assert(Sim.pork_vehicle_name().contains("FALCON HEAVY"), "no Falcon Heavy on the table")
	print("CAMPSHOT  grid %dx%d, launcher %s" % [Sim.pork_rows, Sim.pork_cols, Sim.pork_vehicle_name()])

	main._input(_key(KEY_C))
	await _settle(3)
	assert(Sim.pork_campaign_open, "[C] must open the campaign panel on the map")
	await _shot("campaign_unsolved")

	main._input(_key(KEY_E))
	assert(Sim.pork_campaign_solving, "[E] on the campaign panel must start the measurement")
	assert(not Sim.pork_verifying, "[E] fired the cell verify instead of the campaign")
	await _settle(3)
	await _shot("campaign_solving")
	var t2 := Time.get_ticks_msec()
	while Sim.pork_campaign_solving and Time.get_ticks_msec() - t2 < 900000:
		await get_tree().process_frame
	print("CAMPSHOT  measured in %d ms: %s" % [Time.get_ticks_msec() - t2, Sim.campaign_count_label()])
	assert(Sim.pork_campaign_is_current_vehicle(), "the campaign must land: " + str(Sim.mission.last_error()))
	print("CAMPSHOT  %s" % Sim.campaign_windows_label())
	_assert_per_year_cap()

	# The first flight fires on its own when the measurement lands.
	assert(Sim.pork_campaign_flying, "the plan must be flown automatically on landing")
	var t3 := Time.get_ticks_msec()
	while Sim.pork_campaign_flying and Time.get_ticks_msec() - t3 < 300000:
		await get_tree().process_frame
	print("CAMPSHOT  flown in %d ms: %s" % [Time.get_ticks_msec() - t3, Sim.campaign_flight_label()])
	print("CAMPSHOT  %s" % Sim.campaign_keyhole_label())
	assert(Sim.pork_campaign_flight_is_current(), "the flight must describe the plan on screen")
	var f := Sim.pork_campaign_flight()
	assert(str(f.outcome) != "encounter" or not bool(f.is_hit), "the planned campaign still hits")
	await _settle(4)
	await _shot("campaign_flown")

	# The rate knob: free, and it greys the flight.
	main._input(_key(KEY_Z))
	await _settle(2)
	assert(Sim.pork_campaign_rate == 1, "[Z] must step the rate down")
	assert(not Sim.pork_campaign_solving and not Sim.pork_campaign_flying, "[Z] fired a solve")
	assert(not Sim.pork_campaign_flight_is_current(), "a flight at 2/yr shown as current at 1/yr")
	print("CAMPSHOT  1/yr: %s" % Sim.campaign_count_label())
	_assert_per_year_cap()
	await _settle(3)
	await _shot("campaign_rate_1")
	for _k in 3:
		main._input(_key(KEY_X))
	await _settle(2)
	print("CAMPSHOT  %d/yr: %s" % [Sim.pork_campaign_rate, Sim.campaign_count_label()])
	_assert_per_year_cap()
	main._input(_key(KEY_Z))
	main._input(_key(KEY_Z))
	await _settle(2)
	assert(Sim.pork_campaign_rate == 2, "back to 2/yr")
	assert(Sim.pork_campaign_flight_is_current(), "back on the flown rate, the flight is current again")

	# [M] is inert while the campaign panel is up.
	main._input(_key(KEY_M))
	await _settle(2)
	assert(not main.planner.visible, "[M] opened the planner over the campaign panel")
	assert(not Sim.pork_mass_solving, "[M] started a mass solve the campaign panel does not show")

	# [L] makes the held campaign another launcher's.
	main._input(_key(KEY_L))
	await _settle(3)
	assert(not Sim.pork_campaign_is_current_vehicle(), "[L] left the campaign labelled as this rocket's")
	await _shot("campaign_other_launcher")
	get_tree().quit(0)


## No year carries more launches than the dialled rate — the whole redefinition.
func _assert_per_year_cap() -> void:
	var per: Dictionary = {}
	for w: Dictionary in Sim.pork_campaign.windows:
		per[int(w.period)] = int(per.get(int(w.period), 0)) + int(w.launches)
	for p in per:
		assert(int(per[p]) <= Sim.pork_campaign_rate,
			"year %d carries %d launches at a cap of %d/yr" % [p, per[p], Sim.pork_campaign_rate])


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
	print("CAMPSHOT  wrote %s" % path)
