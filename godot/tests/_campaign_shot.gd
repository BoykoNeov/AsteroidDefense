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
## - the cap is per ANY 12 MONTHS: no 365.25 days of the plan carry more launches
##   than the rate (the core's own count, `busiest_rolling_year`);
## - the rate knob is free (no solve fires on [Z]/[X]) and greys the flight line;
## - the readiness line: the panel opens on the shipping level (FROM SCRATCH, the
##   rock found when the scenario starts), [W] and [ / ] are free to dial (no solve
##   fires), and a held campaign measured at another setting is not shown as this
##   one's. The ON THE PAD level reproduces every count from before 2026-10-08's
##   standing-defence batch, so the checks below that pin those run on it;
## - 1/yr on the pad is 10 LAUNCHES since the map's axes were stretched (it fell
##   short by ~1 % while launches stopped 3.6 yr before impact; its last launch is
##   now 2.72 yr out). Launches park in orbit and leave on a later date, the escape
##   burn flown finite (8 firings from 400 km). The what-if row says it falls short
##   with no parking;
## - FROM SCRATCH, found 12 yr out (first launch 6.53 yr before impact), 2/yr FALLS
##   SHORT and 6/yr needs 12 LAUNCHES, against 6 on the pad;
## - the parking line names the height, stack, engine and firings, and the plan
##   really parks;
## - the window boxes sit at their own dates, between the map's cells (look at the
##   shot: the continuous search finds dates no cell has);
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
	print("CAMPSHOT  %s" % Sim.campaign_readiness_label())
	assert(Sim.campaign_readiness_label().contains("FROM SCRATCH"),
		"the panel must open on the shipping readiness level")
	await _shot("campaign_unsolved")

	# The readiness knobs are free: dialling them fires nothing.
	main._input(_key(KEY_BRACKETRIGHT))
	await _settle(1)
	assert(Sim.campaign_warning_yr() > Sim.T_IMPACT * Sim.DAY_S / Sim.YEAR_S, "[ ] ] must lengthen the warning")
	main._input(_key(KEY_BRACKETLEFT))
	await _settle(1)
	assert(absf(Sim.campaign_warning_yr() - Sim.T_IMPACT * Sim.DAY_S / Sim.YEAR_S) < 1e-9, "[ [ ] must step it back")
	# [W] from the shipping level (the last) wraps to ON THE PAD (the first).
	main._input(_key(KEY_W))
	await _settle(1)
	assert(not Sim.pork_campaign_solving, "a readiness knob fired a solve")
	assert(Sim.campaign_readiness_label().contains("ON THE PAD"), "[W] must step to ON THE PAD")

	main._input(_key(KEY_E))
	assert(Sim.pork_campaign_solving, "[E] on the campaign panel must start the measurement")
	assert(not Sim.pork_verifying, "[E] fired the cell verify instead of the campaign")
	await _settle(3)
	await _shot("campaign_solving")
	var t2 := Time.get_ticks_msec()
	while Sim.pork_campaign_solving and Time.get_ticks_msec() - t2 < 900000:
		await get_tree().process_frame
	print("CAMPSHOT  measured in %d ms: %s" % [Time.get_ticks_msec() - t2, Sim.campaign_count_label()])
	assert(Sim.pork_campaign_is_current(), "the campaign must land: " + str(Sim.mission.last_error()))
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
	print("CAMPSHOT  %s" % Sim.campaign_windows_label())
	print("CAMPSHOT  %s" % Sim.campaign_parking_label())
	print("CAMPSHOT  %s" % Sim.campaign_what_if_label())
	assert(Sim.campaign_count_label().begins_with("10 LAUNCHES"),
		"1 in any 12 months on the pad is 10 launches once late launch dates are on the map")
	assert(Sim.campaign_what_if_label().contains("NO PARKING SHORT"),
		"with no parking, 1 in any 12 months falls short")
	assert(Sim.campaign_parking_label().contains("LONGEST WAIT"), "the 1/yr plan parks")
	assert(Sim.campaign_parking_label().contains("400 KM") and Sim.campaign_parking_label().contains("8 BURNS"),
		"the parking line names the height and the firings")
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

	# FROM SCRATCH: the same rock, the first launch 5.5 yr later.
	main._input(_key(KEY_W))
	main._input(_key(KEY_W))
	await _settle(2)
	assert(Sim.campaign_readiness_label().contains("FROM SCRATCH"), "[W] twice must come back round")
	assert(not Sim.pork_campaign_is_current(), "a campaign measured on the pad shown as FROM SCRATCH's")
	await _shot("campaign_scratch_unsolved")
	main._input(_key(KEY_E))
	assert(Sim.pork_campaign_solving, "[E] must measure the new readiness")
	var t4 := Time.get_ticks_msec()
	while Sim.pork_campaign_solving and Time.get_ticks_msec() - t4 < 900000:
		await get_tree().process_frame
	assert(Sim.pork_campaign_is_current(), "the FROM SCRATCH campaign must land: " + str(Sim.mission.last_error()))
	print("CAMPSHOT  from scratch, measured in %d ms: %s" % [Time.get_ticks_msec() - t4, Sim.campaign_readiness_label()])
	print("CAMPSHOT  from scratch %d/yr: %s" % [Sim.pork_campaign_rate, Sim.campaign_count_label()])
	assert(Sim.campaign_count_label().begins_with("FALLS SHORT"), "built from scratch, 2 a year must fall short")
	assert(float(Sim.pork_campaign.first_launch_tdb) > Sim.pork_launch_tdb[0], "the first launch must have moved")
	_assert_per_year_cap()
	# The landing flies the plan if there is one; at 2/yr there is not.
	var t5 := Time.get_ticks_msec()
	while Sim.pork_campaign_flying and Time.get_ticks_msec() - t5 < 300000:
		await get_tree().process_frame
	for _k in 4:
		main._input(_key(KEY_X))
	await _settle(2)
	print("CAMPSHOT  from scratch %d/yr: %s" % [Sim.pork_campaign_rate, Sim.campaign_count_label()])
	assert(Sim.campaign_count_label().begins_with("12 LAUNCHES"), "built from scratch, 6 a year needs 12")
	_assert_per_year_cap()
	await _shot("campaign_scratch_6")
	for _k in 4:
		main._input(_key(KEY_Z))
	await _settle(1)

	# [L] makes the held campaign another launcher's.
	main._input(_key(KEY_L))
	await _settle(3)
	assert(not Sim.pork_campaign_is_current(), "[L] left the campaign labelled as this rocket's")
	await _shot("campaign_other_launcher")
	get_tree().quit(0)


## No 365.25 days carry more launches than the dialled rate — the whole rule.
## Counted here from the windows' own launch dates, not read off the core's tally,
## so a planner that broke the cap and a tally that hid it cannot agree by accident.
## Parked launches go up whole periods apart, which floating point only rounds to,
## so a launch within 1 ms of a window's far end counts as outside it - the core's
## own `ROLLING_SLACK_S`, restated.
func _assert_per_year_cap() -> void:
	var dated: Array = []
	for w: Dictionary in Sim.pork_campaign.windows:
		if int(w.launches) > 0:
			dated.append([float(w.launch_tdb), int(w.launches)])
	var year_s := 365.25 * 86400.0
	for a in dated:
		var n := 0
		for b in dated:
			if float(b[0]) >= float(a[0]) and float(b[0]) < float(a[0]) + year_s - 1.0e-3:
				n += int(b[1])
		assert(n <= Sim.pork_campaign_rate,
			"%d launches inside 12 months at a cap of %d" % [n, Sim.pork_campaign_rate])
	assert(int(Sim.pork_campaign.get("busiest_rolling_year", 0)) <= Sim.pork_campaign_rate,
		"the core's own busiest-12-months tally exceeds the rate")


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
