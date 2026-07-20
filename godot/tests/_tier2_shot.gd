extends Node
## Temporary Tier-2 panel harness — the only thing that runs Tier2Panel._draw().
##   godot --path godot --resolution 1600x900   (non-headless → a real PNG)
##   godot --headless --path godot              (_draw still runs for VISIBLE nodes
##                                                → verifies the numbers branch, blank img)
## Registered as an autoload while running, removed afterwards. Not shipped.
##
## Why: _draw runs under --headless but ONLY for visible nodes, and the panel is
## hidden until [P]. So the numbers branch (the TIER2_TERMS loop + _value_text
## formatting) never executes in a passive run — exactly the gap that let a wrong
## panel ship a whole phase elsewhere in this project.

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
		print("TIER2SHOT  FAIL: no Main node")
		get_tree().quit(1)
		return

	DirAccess.make_dir_recursive_absolute(OUT)
	main.boot.dismiss()
	await _settle(6)

	# Wait for the (fast) build → threat solution.
	var t0 := Time.get_ticks_msec()
	while not Sim.mission_online and Time.get_ticks_msec() - t0 < 180000:
		await get_tree().process_frame
	print("TIER2SHOT  mission_online=%s after %d ms" % [Sim.mission_online, Time.get_ticks_msec() - t0])

	# Open the panel (kicks the on-demand ~2 min measurement) and wait for it.
	main.enc.visible = false
	main.map2d.visible = false
	main.planner.visible = false
	main.tier2_panel.visible = true
	Sim.tier2_panel_open = true
	Sim.request_tier2_preview()
	var t1 := Time.get_ticks_msec()
	while not Sim.tier2_ready and Time.get_ticks_msec() - t1 < 240000:
		await get_tree().process_frame
	print("TIER2SHOT  tier2_ready=%s after %d ms" % [Sim.tier2_ready, Time.get_ticks_msec() - t1])

	# Switch every term on — the numbers branch of _draw.
	for term in ["relativity", "yarkovsky", "belt", "srp"]:
		Sim.tier2_on[term] = true
	await _settle(6)   # _draw runs here for the now-visible panel

	# Echo exactly what the panel formats, so the console and the picture can be
	# read against each other (the "verified by picture was overstated" guard).
	for t in Sim.TIER2_TERMS:
		var id: String = t[1]
		var avail: bool = Sim.tier2_available(id)
		var shift: float = Sim.tier2_shift_km(id)
		print("TIER2SHOT  %-11s avail=%s shift=%s km"
			% [id, avail, ("N/A" if is_nan(shift) else "%+.2f" % shift)])
	print("TIER2SHOT  nominal_perigee=%.1f km capture=%.0f km" % [Sim.nom_perigee_km, Sim.cap_km])

	await _shot("tier2_menu_all_on")
	get_tree().quit(0)


func _settle(frames: int) -> void:
	for _i in frames:
		await get_tree().process_frame


func _shot(name: String) -> void:
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var path := "%s/%s.png" % [OUT, name]
	img.save_png(path)
	print("TIER2SHOT  wrote %s" % path)
