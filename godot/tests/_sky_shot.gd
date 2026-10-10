extends Node
## Temporary sky-screen harness — the only thing that runs SkyView._draw().
##   powershell -File godot/tests/run_harness.ps1 -Harness _sky_shot.gd
## Registered as an autoload while running, removed afterwards. Not shipped.
##
## The sky screen is hidden until [5], so in a passive run its drawing, its key
## handling and its panel never execute. Every step here goes through
## `main._input()` with a real key event, so the chain tested is project.godot
## action -> main.gd -> SkyView / Sim -> core, not a direct call that would skip
## the part most likely to be miswired (the [B] key in particular, which means
## time-reverse everywhere else).

const OUT := "W:/temp/claude/sky-obs/shots"


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
		print("SKYSHOT  FAIL: no Main node")
		get_tree().quit(1)
		return
	DirAccess.make_dir_recursive_absolute(OUT)
	main.boot.dismiss()
	await _settle(6)

	var t0 := Time.get_ticks_msec()
	while not Sim.bodies_online and Time.get_ticks_msec() - t0 < 120000:
		await get_tree().process_frame
	print("SKYSHOT  bodies_online=%s after %d ms" % [Sim.bodies_online, Time.get_ticks_msec() - t0])
	if not Sim.bodies_online:
		print("SKYSHOT  FAIL: kernel field never came up")
		get_tree().quit(1)
		return

	main._input(_key(KEY_5))
	print("SKYSHOT  [5] -> visible=%s building=%s (expect true/true)" % [main.sky.visible, Sim.sky_building])
	assert(main.sky.visible, "[5] must open the sky screen")
	assert(not main.enc.visible and not main.pork.visible and not main.map2d.visible, "[5] must hide the other views")

	var t1 := Time.get_ticks_msec()
	while Sim.sky_building and Time.get_ticks_msec() - t1 < 60000:
		await get_tree().process_frame
	print("SKYSHOT  sky_online=%s after %d ms, %d shots, error '%s'" %
		[Sim.sky_online, Time.get_ticks_msec() - t1, Sim.sky_shots.size(), Sim.sky_error])
	if not Sim.sky_online:
		print("SKYSHOT  FAIL: no observing run")
		get_tree().quit(1)
		return

	# [B] must toggle the blink here, NOT reverse the clock.
	var dir_before: float = Sim.time_dir
	var blink_before: bool = main.sky.blinking
	main._input(_key(KEY_B))
	print("SKYSHOT  [B] -> blinking %s -> %s" % [blink_before, main.sky.blinking])
	assert(main.sky.blinking != blink_before, "[B] must toggle the blink on the sky screen")
	print("SKYSHOT  clock direction %s -> %s (expect unchanged)" % [dir_before, Sim.time_dir])
	assert(Sim.time_dir == dir_before, "[B] must not reverse the clock on the sky screen")

	# Shot 1 of night 1, telescope field, names on.
	main.sky.blinking = false
	main.sky.shot_idx = 0
	await _settle(3)
	await _shot("sky_telescope_n1s1")
	main._input(_key(KEY_RIGHT))
	await _settle(3)
	await _shot("sky_telescope_n1s2")
	print("SKYSHOT  [RIGHT] -> shot %d" % main.sky.shot_idx)

	# Stacked: the night's three asteroid images over one frame of stars.
	main._input(_key(KEY_O))
	await _settle(3)
	await _shot("sky_stacked_n1")
	print("SKYSHOT  [O] -> stacked %s" % main.sky.stacked)

	# Zoomed in 4x, still stacked: the three trails and the jump between them.
	main._input(_key(KEY_UP))
	main._input(_key(KEY_UP))
	await _settle(3)
	await _shot("sky_stacked_zoom4")
	print("SKYSHOT  [UP][UP] -> zoom %.0fx" % main.sky.ZOOMS[main.sky.zoom_idx])

	# The finder chart with names, then without.
	for _i in 4:
		main._input(_key(KEY_DOWN))
	await _settle(3)
	await _shot("sky_finder_names")
	main._input(_key(KEY_L))
	await _settle(3)
	await _shot("sky_finder_no_names")
	print("SKYSHOT  finder zoom_idx=%d labels=%s" % [main.sky.zoom_idx, main.sky.labels])

	# Night 3 in the telescope field.
	main._input(_key(KEY_L))
	main._input(_key(KEY_UP))
	main.sky.stacked = false
	main.sky.shot_idx = Sim.sky_shots.size() - 1
	await _settle(3)
	await _shot("sky_telescope_n3s3")

	# The trial orbit: [M] opens it on the rough starting guess, degrees out.
	main.sky.zoom_idx = 1
	main.sky.shot_idx = 4
	main._input(_key(KEY_M))
	await _settle(3)
	print("SKYSHOT  [M] -> trial open %s, start RMS %.1f arcsec" % [Sim.sky_trial_open, Sim.sky_trial_rms])
	assert(Sim.sky_trial_open, "[M] must open the trial orbit on the sky screen")
	assert(not main.planner.visible, "[M] must not open the mission planner here")
	assert(Sim.sky_trial_rms > 3600.0, "the starting guess should be degrees out")
	await _shot("sky_trial_start")
	main.sky.zoom_idx = 0
	await _settle(3)
	await _shot("sky_trial_start_finder")
	# [H]: the hint names a knob and a direction, and following it helps.
	var before: float = Sim.sky_trial_rms
	main._input(_key(KEY_H))
	print("SKYSHOT  [H] -> '%s' (cursor %d)" % [Sim.sky_trial_hint_text, Sim.sky_trial_cursor])
	var hint_sign := 1 if Sim.sky_trial_hint_text.begins_with("HINT: RAISE") else -1
	main._input(_key(KEY_RIGHT if hint_sign > 0 else KEY_LEFT))
	print("SKYSHOT  followed the hint: RMS %.1f -> %.1f arcsec" % [before, Sim.sky_trial_rms])
	assert(Sim.sky_trial_rms < before, "following the hint must shrink the miss")
	await _settle(3)
	await _shot("sky_trial_after_hint")
	# [E]: JPL's orbit. The ghosts land on the asteroid to within the measuring error.
	main._input(_key(KEY_E))
	print("SKYSHOT  [E] -> JPL orbit, RMS %.3f arcsec" % Sim.sky_trial_rms)
	assert(Sim.sky_trial_rms < 1.0, "JPL's orbit must land within the measuring error")
	main.sky.zoom_idx = 3
	main.sky.stacked = true
	await _settle(3)
	await _shot("sky_trial_jpl_zoom4_stacked")
	# Turning a knob makes it the player's orbit again: [E] then reveals JPL's
	# afresh rather than throwing the edit away for the pre-reveal guess.
	main._input(_key(KEY_RIGHT))
	main._input(_key(KEY_E))
	print("SKYSHOT  knob turned, [E] again -> RMS %.3f arcsec" % Sim.sky_trial_rms)
	assert(Sim.sky_trial_rms < 1.0, "[E] after an edit must reveal JPL's orbit again")
	# [R] restarts from the rough guess.
	main._input(_key(KEY_R))
	print("SKYSHOT  [R] -> RMS %.1f arcsec (back to the start)" % Sim.sky_trial_rms)
	assert(Sim.sky_trial_rms > 3600.0, "[R] must restart the guess")

	print("SKYSHOT  PASS")
	get_tree().quit(0)


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
	print("SKYSHOT  wrote %s" % path)
