extends Node
## Temporary screenshot harness — the only thing that actually runs the panels.
##
## Registered as an autoload (`Shot="*res://tests/_shot.gd"`) and driven NON-headless:
##   godot --path godot --resolution 1600x900
## then removed from project.godot again. It is not part of the shipping game.
##
## Why it exists: `_draw()` does run under `--headless`, but **only for VISIBLE
## nodes**. The encounter view is hidden until [3] is pressed, so a passive headless
## run executes its draw path exactly zero times — which is how the previous b-plane
## view shipped for a whole phase while disagreeing with its own physics. A picture
## is the only check that the thing a player looks at is the thing the core says.
##
## Two gotchas, both learned the hard way: a shot taken at frame 1 is BLACK (capture
## needs `await RenderingServer.frame_post_draw` plus a few frames of warm-up), and
## the boot overlay covers everything until dismissed.

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
		print("SHOT  FAIL: no Main node")
		get_tree().quit(1)
		return

	DirAccess.make_dir_recursive_absolute(OUT)
	main.boot.dismiss()
	await _settle(6)

	# The threat is ~10 s of real integration away. Nothing to draw until it lands.
	var t0 := Time.get_ticks_msec()
	while not Sim.mission_online and Time.get_ticks_msec() - t0 < 60000:
		await get_tree().process_frame
	print("SHOT  mission_online=%s after %d ms" % [Sim.mission_online, Time.get_ticks_msec() - t0])

	# Show the b-plane view: exactly what [3] does, without an InputMap round-trip.
	main.enc.visible = true
	main.map2d.visible = false
	main.tags.visible = false
	main.hud.view_name = "ENCOUNTER B-PLANE"

	# 1. No plan: the incoming impact, and nothing pretending to be a deflection.
	await _settle(4)
	await _shot("enc_1_no_plan")
	print("SHOT  no plan: b_defl=%s (ZERO expected), defl track %d pts"
		% [Sim.encounter_b_point(true), Sim.encounter_track(true).size()])

	# 2. A plan in the band the old verdict got wrong: |B| outside the disc, its
	#    perigee inside. This must read MISS.
	Sim.set_plan(Sim.threat_period_d(), 0.2, true)
	Sim._tick_plan_debounce(1.0)
	await _settle(4)
	await _shot("enc_2_band_miss")
	print("SHOT  band: |B|=%d km cap=%d km perigee=%d km verdict=%s"
		% [int(Sim.miss_ld * Sim.LD_KM), int(Sim.cap_km),
			int(Sim.perigee_ld(true) * Sim.LD_KM), Sim.verdict_label()])

	# 3. An insufficient nudge: the b-point stays inside the disc. The hit.
	Sim.set_plan(30.0, 0.1, true)
	Sim._tick_plan_debounce(1.0)
	await _settle(4)
	await _shot("enc_3_impact")
	print("SHOT  weak: |B|=%d km verdict=%s"
		% [int(Sim.miss_ld * Sim.LD_KM), Sim.verdict_label()])

	# 4. Zoomed out, so the tracks and their bend are visible around the disc.
	main.enc._half_ld = 1.2
	await _settle(3)
	await _shot("enc_4_zoomed_out")
	main.enc._half_ld = 0.15

	# 5. THE LIVE MARKER, which only draws when the clock is inside the ±1.5 d
	#    window — i.e. never, in any other shot or test here, since the campaign is
	#    twelve years long. Commit first (the launch window has to still be open at
	#    t=0), then scrub to just before impact: that also lights the BURNED state,
	#    where the nominal cross goes dim and the deflected track becomes the live
	#    one. Both branches are unreachable from a passive run.
	Sim.set_plan(Sim.threat_period_d(), 0.2, true)
	Sim._tick_plan_debounce(1.0)
	Sim.try_commit()
	# PAUSE before scrubbing: at warp the clock runs on through the settle frames,
	# and the encounter is over in hours. The first attempt landed 0.53 d past
	# closest approach, by which point the rock is ~1.3 LD out and off-plot — the
	# gate behaving correctly, but not the branch being checked.
	Sim.paused = true
	Sim.jump(Sim.T_IMPACT)
	await _settle(4)
	await _shot("enc_6_live_marker_burned")
	print("SHOT  marker: t=%.3f d (impact %.3f), committed=%s burned=%s, window=%s"
		% [Sim.t, Sim.T_IMPACT, Sim.committed, Sim.burned(), Sim.encounter_span_days()])
	Sim.paused = false
	Sim.jump(0.0)
	await _settle(2)

	# 7. The planner beside it — the two panels must agree, and this is the pair a
	#    player reads against each other.
	main.enc.visible = false
	main.planner.visible = true
	Sim.planner_open = true
	Sim.set_plan(Sim.threat_period_d(), 0.2, true)
	Sim._tick_plan_debounce(1.0)
	await _settle(4)
	await _shot("enc_5_planner_agrees")

	get_tree().quit(0)


func _settle(frames: int) -> void:
	for _i in frames:
		await get_tree().process_frame


func _shot(name: String) -> void:
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var path := "%s/%s.png" % [OUT, name]
	img.save_png(path)
	print("SHOT  wrote %s" % path)
