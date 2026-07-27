extends Node
## Temporary gravity-tractor bench harness — the only thing that runs
## TractorPanel._draw().
##   godot --path godot --resolution 1600x900   (non-headless → a real PNG)
##   godot --headless --path godot              (_draw still runs for VISIBLE nodes
##                                                → verifies the numbers branch, blank img)
## Registered as an autoload while running, removed afterwards. Not shipped.
##
## Why, same as `_tier2_shot.gd`: `_draw` runs under `--headless` but ONLY for
## visible nodes, and this panel is hidden until [K]. So its numbers branch never
## executes in a passive run — which for a panel with six formatters and three
## "absent, not zero" branches is a lot of code no other check reaches.
##
## What it is really guarding, beyond "does it draw":
##
## 1. **Every knob steps through its REAL key path** — the arrow actions, read out
##    of `Sim.TRACTOR_KNOBS` rather than restated here. A hand-kept second list is
##    how a newly added knob gets a panel row and a `Sim` entry while nothing ever
##    presses its key; the check would read as coverage and would not be.
## 2. **The margin is formed from the impulsive-equivalent, never the delivered
##    Δv.** Those two differ by up to 2× and both are on screen, so a panel that
##    picked the wrong one would look entirely plausible.
## 3. **The requirement disappears below the law's validity floor**, rather than
##    being extrapolated into a regime where it is 1.73× wrong.
## 4. **A probe stops being labelled current the moment a knob moves.**

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
		print("TRACTORSHOT  FAIL: no Main node")
		get_tree().quit(1)
		return

	DirAccess.make_dir_recursive_absolute(OUT)
	main.boot.dismiss()
	await _settle(6)

	var t0 := Time.get_ticks_msec()
	while not Sim.mission_online and Time.get_ticks_msec() - t0 < 180000:
		await get_tree().process_frame
	print("TRACTORSHOT  mission_online=%s after %d ms" % [Sim.mission_online, Time.get_ticks_msec() - t0])

	main.enc.visible = false
	main.map2d.visible = false
	main.planner.visible = false

	# The defaults must have come from the CORE, not from the literals in Sim.
	# A panel that silently kept its own 1.5 would agree with the physics today
	# and drift the first time the core's hover distance moved.
	print("TRACTORSHOT  seeded hover=%.3f R radius=%.1f m law_min=%.2f orb target=%.0f km"
		% [Sim.tractor.hover, Sim.tractor.radius, Sim.tractor_law_min_periods,
			Sim.tractor_target_perigee_m / 1000.0])
	var seeded_hover: float = Sim.tractor.hover
	assert(Sim.tractor_law_min_periods > 0.0, "the law floor must be seeded from the core")
	assert(Sim.tractor_target_perigee_m > 0.0, "the safe-perigee bar must be seeded from the core")

	# Drive the REAL toggle path: a key event through main._input(), so
	# project.godot action -> main.gd handler -> Sim executes.
	assert(not main.tractor_panel.visible, "panel starts hidden")
	main._input(_key(KEY_K))
	print("TRACTORSHOT  [K] -> panel.visible=%s (expect true)" % main.tractor_panel.visible)
	assert(main.tractor_panel.visible, "[K] must open the bench via the action")

	# --- every knob, through its real key -------------------------------------
	# Read out of TRACTOR_KNOBS so a seventh knob is covered by existing.
	for i in Sim.TRACTOR_KNOBS.size():
		var knob: Array = Sim.TRACTOR_KNOBS[i]
		var id: String = knob[0]
		# Park the cursor on this row using the real UP/DOWN actions. Bounded:
		# an unguarded `while` around a synchronous key press has no `await` in
		# it, so if the action ever stops moving the cursor it does not fail, it
		# freezes the engine with no output at all — which is how this harness
		# first "ran" for ten minutes and said nothing.
		var hops := 0
		while Sim.tractor_row != i and hops <= Sim.TRACTOR_KNOBS.size():
			main._input(_key(KEY_DOWN))
			hops += 1
		assert(Sim.tractor_row == i,
			"[DOWN] must reach row %d — is the cursor action wired?" % i)
		# Step the knob and step it back. Which direction is tried FIRST depends on
		# where the knob sits: `duty` ships pinned at its maximum, so a
		# [RIGHT]-only check records "100 -> 100" and passes without ever proving
		# that row is wired. Whichever way it can move, it must.
		var before: float = Sim.tractor[id]
		var out_key: int = KEY_LEFT if before >= float(knob[4]) else KEY_RIGHT
		var back_key: int = KEY_RIGHT if out_key == KEY_LEFT else KEY_LEFT
		main._input(_key(out_key))
		var after: float = Sim.tractor[id]
		print("TRACTORSHOT  row %d %-14s %s %.4f -> %.4f"
			% [i, id, "[LEFT]" if out_key == KEY_LEFT else "[RIGHT]", before, after])
		assert(after != before,
			"knob %s did not move in either direction (is the cursor row wired?)" % id)
		main._input(_key(back_key))
		assert(is_equal_approx(Sim.tractor[id], before),
			"stepping knob %s back must restore it" % id)

	# --- the margin must come from the equivalent, not the delivered ----------
	# Both are drawn, they differ by up to 2x, and picking the wrong one is
	# invisible on screen. So the relationship is asserted rather than eyeballed.
	var r := Sim.tractor_readout()
	# `%s` on a float, not `%e` — GDScript has no `%e`, and a bad format does not
	# merely print wrong: it ABORTS the coroutine. That is what made an earlier run
	# of this harness hang for ten minutes with no output, because `_run` never
	# reached `get_tree().quit()`.
	print("TRACTORSHOT  delivered=%s  equivalent=%s  required=%s  margin=%s"
		% [r.delivered_dv_m_s, r.equivalent_dv_m_s,
			r.required_dv_m_s if r.has("required_dv_m_s") else "ABSENT",
			("%.4f" % r.margin) if r.has("margin") else "ABSENT"])
	assert(float(r.equivalent_dv_m_s) < float(r.delivered_dv_m_s),
		"the impulsive equivalent must sit below the delivered upper bound")

	# The formatter itself, on the magnitudes it actually receives. This exists
	# because the first version used `is_zero_approx` — whose epsilon is 1e-5 —
	# and so rendered every tow acceleration (~1e-11) as a flat "0 M/S2", beside a
	# correct mm/s/yr figure that made it look plausible. Every assertion in this
	# harness passed; only the screenshot showed it. Values are checked through
	# the panel's own helper so the check cannot drift from what is drawn.
	var tow_txt: String = main.tractor_panel._sci(float(r.tow_accel_m_s2), 3)
	print("TRACTORSHOT  tow accel %s formats as '%s'" % [r.tow_accel_m_s2, tow_txt])
	assert(float(r.tow_accel_m_s2) > 0.0, "the shipping configuration must tow")
	assert(tow_txt != "0" and tow_txt.contains("E"),
		"a ~1e-11 tow must format as scientific notation, not '%s'" % tow_txt)
	assert(main.tractor_panel._sci(0.0, 3) == "0", "an actual zero still prints as 0")
	if r.has("margin"):
		var from_equivalent := float(r.equivalent_dv_m_s) / float(r.required_dv_m_s)
		assert(is_equal_approx(float(r.margin), from_equivalent),
			"the margin must be formed from the impulsive equivalent, not the delivered Δv")

	# --- the plume wall, reached the way a user reaches it -------------------
	# Three [LEFT] presses from the seeded 1.5 lands at ~0.99 radii unclamped —
	# inside the body. The band that matters is narrower and less obvious: from
	# the surface up to 1/cos(20 deg) = 1.064 the spacecraft is OUTSIDE the rock
	# and still has no station-keeping solution. The knob must stop at the plume
	# wall, not at the surface, and it is only reachable by pressing LEFT — which
	# the sweep above never does, because it steps every knob outward first.
	var hpark := 0
	while Sim.tractor_row != 1 and hpark <= Sim.TRACTOR_KNOBS.size():
		main._input(_key(KEY_DOWN))
		hpark += 1
	assert(Sim.tractor_row == 1, "could not park the cursor on the hover row")
	for _i in 20:
		main._input(_key(KEY_LEFT))
	var tight := Sim.tractor_readout()
	print("TRACTORSHOT  hover floored at %.4f R (core min %.4f) holds_station=%s thrust=%.1f N"
		% [Sim.tractor.hover, Sim.tractor_hover_min, tight.holds_station, tight.thrust_n])
	assert(Sim.tractor.hover >= Sim.tractor_hover_min,
		"the hover knob must clamp at the core's plume wall, not below it")
	assert(Sim.tractor_hover_min > 1.0,
		"the plume wall must sit ABOVE the surface — a floor of 1.0 is the bug")
	assert(bool(tight.holds_station),
		"at the clamped floor a station-keeping solution must still exist")
	assert(float(tight.thrust_n) > 0.0,
		"and the thrust there must be a real number, never the 0.000 N a missing \
		 value formats to")
	# The wall is a wall because the thrust diverges into it. That divergence is
	# the honest answer to "why not just hover closer for a bigger 1/d^2 tow?".
	var wide := Sim.tractor_hover_min * 1.5
	var before_h: float = Sim.tractor.hover
	Sim.tractor.hover = wide
	var loose := Sim.tractor_readout()
	Sim.tractor.hover = before_h
	print("TRACTORSHOT  thrust at floor %.1f N vs at %.3f R %.1f N"
		% [tight.thrust_n, wide, loose.thrust_n])
	assert(float(tight.thrust_n) > 3.0 * float(loose.thrust_n),
		"station-keeping thrust must blow up approaching the plume wall")
	await _settle(4)
	await _shot("tractor_plume_wall")

	# Back to the shipping hover distance — set, not stepped. The hover knob is
	# multiplicative (x1.15), so from the floor at 1.0642 the reachable values are
	# 1.224, 1.407, 1.618... and 1.5 is not among them. Stepping back "until >=
	# 1.5" lands on 1.618, which tows (1.5/1.618)^2 = 0.86x as hard and quietly
	# makes the probe below describe a configuration the campaign never measured.
	# Restoring it exactly is what lets the two layers be compared at all.
	Sim.tractor.hover = seeded_hover

	# --- the validity floor, driven from the panel ---------------------------
	# Wind the lead below one orbit and the requirement must VANISH. This is the
	# branch that keeps a 1.73x-wrong number off the screen, and it is only
	# reachable by actually turning the knob.
	var park := 0
	while Sim.tractor_row != 3 and park <= Sim.TRACTOR_KNOBS.size():
		main._input(_key(KEY_DOWN))
		park += 1
	assert(Sim.tractor_row == 3, "could not park the cursor on the lead row")
	var guard := 0
	while Sim.tractor.lead >= Sim.tractor_law_min_periods and guard < 200:
		main._input(_key(KEY_LEFT))
		guard += 1
	var low := Sim.tractor_readout()
	print("TRACTORSHOT  lead=%.2f orb -> required=%s margin=%s (expect both ABSENT)"
		% [Sim.tractor.lead,
			"PRESENT" if low.has("required_dv_m_s") else "ABSENT",
			"PRESENT" if low.has("margin") else "ABSENT"])
	assert(not low.has("required_dv_m_s"),
		"below the law's floor the requirement must be absent, not extrapolated")
	assert(not low.has("margin"), "no requirement means no margin")
	await _settle(4)
	await _shot("tractor_below_law_floor")

	# Wind it back up to the campaign's own lead and probe for real.
	while Sim.tractor.lead < 8.0 and guard < 400:
		main._input(_key(KEY_RIGHT))
		guard += 1

	# --- the on-demand full-field probe, through [E] --------------------------
	main._input(_key(KEY_E))
	print("TRACTORSHOT  [E] -> probing=%s (expect true)" % Sim.tractor_probing)
	assert(Sim.tractor_probing, "[E] must fire the full-field probe")
	var t1 := Time.get_ticks_msec()
	while Sim.tractor_probing and Time.get_ticks_msec() - t1 < 240000:
		await get_tree().process_frame
	var p := Sim.tractor_probe()
	print("TRACTORSHOT  probe after %d ms: %s" % [Time.get_ticks_msec() - t1, Sim.tractor_probe_label()])
	assert(not p.is_empty(), "the probe must land a result")
	assert(Sim.tractor_probe_is_current(), "a fresh probe must be current for the live knobs")
	# THE finding, on the real field: a 20 t tractor moves this perigee the WRONG
	# WAY. If this ever comes back positive at the shipping configuration the
	# panel's whole "DEEPER" branch has gone untested and the campaign's headline
	# has moved.
	print("TRACTORSHOT  nominal=%.1f km  towed=%.1f km  shift=%+.1f km  clears=%s"
		% [float(p.nominal_perigee_m) / 1000.0, float(p.perigee_m) / 1000.0,
			float(p.shift_m) / 1000.0, Sim.tractor_probe_clears()])
	assert(float(p.shift_m) < 0.0,
		"the shipping 20 t configuration must still move perigee INWARD - if this 		 goes positive the panel's DEEPER branch is untested and the campaign 		 headline has moved")
	# The knobs are now exactly the campaign's: 20 t, d/r = 1.5, 150 m rock, 8
	# orbits, towing the whole lead. So this is a genuine CROSS-LAYER check —
	# `gravity_tractor_measured_on_the_real_threat` measures -188.4 km in Rust, and
	# the frontend must reproduce it, not merely produce something negative. A band
	# rather than an equality because the two reach the perigee by different call
	# paths (a solve versus a single probe).
	var shift_km := float(p.shift_m) / 1000.0
	assert(shift_km > -220.0 and shift_km < -150.0,
		"the frontend probe should reproduce the campaign's -188.4 km inward move; 		 got %+.1f km" % shift_km)

	# --- and staleness: one keypress must un-current a measured number --------
	main._input(_key(KEY_RIGHT))
	print("TRACTORSHOT  after one [RIGHT]: probe_is_current=%s (expect false)"
		% Sim.tractor_probe_is_current())
	assert(not Sim.tractor_probe_is_current(),
		"moving a knob must stop the probe being labelled current")
	main._input(_key(KEY_LEFT))
	assert(Sim.tractor_probe_is_current(), "stepping back must restore it")

	# --- the direction knob's PROBE path, which nothing had ever run ----------
	# Every probe above — here and in the Rust suite — is prograde. The core has
	# `retrograde_tug_is_the_exact_negative_of_prograde`, so flipping this knob
	# should flip the sign of the b-plane displacement, and on a near-centre
	# nominal that is the difference between deepening the impact and easing it.
	# A knob that changes the answer this much cannot ship with its probe path
	# unexecuted.
	var prograde_shift: float = float(p.shift_m)
	var dpark := 0
	while Sim.tractor_row != 5 and dpark <= Sim.TRACTOR_KNOBS.size():
		main._input(_key(KEY_DOWN))
		dpark += 1
	assert(Sim.tractor_row == 5, "could not park the cursor on the direction row")
	main._input(_key(KEY_LEFT))
	assert(Sim.tractor.dir > 0.5, "[LEFT] must flip the direction knob to retrograde")
	main._input(_key(KEY_E))
	assert(Sim.tractor_probing, "[E] must fire a probe for the flipped direction")
	var t2 := Time.get_ticks_msec()
	while Sim.tractor_probing and Time.get_ticks_msec() - t2 < 240000:
		await get_tree().process_frame
	var pr := Sim.tractor_probe()
	assert(not pr.is_empty(), "the retrograde probe must land a result")
	print("TRACTORSHOT  RETROGRADE: %s" % Sim.tractor_probe_label())
	print("TRACTORSHOT  prograde shift %+.1f km vs retrograde %+.1f km"
		% [prograde_shift / 1000.0, float(pr.shift_m) / 1000.0])
	assert(float(pr.shift_m) * prograde_shift < 0.0,
		"retrograde must move the perigee the OTHER way — got %+.1f km against a \
		 prograde %+.1f km" % [float(pr.shift_m) / 1000.0, prograde_shift / 1000.0])
	await _settle(4)
	await _shot("tractor_retrograde")
	main._input(_key(KEY_RIGHT))   # back to prograde

	# --- the two knob end stops whose refusals had never executed -------------
	# `duty = 0` is a legal setting and would otherwise reach `TowWindow` as a
	# zero-length window; the guard answers in words instead. Cheap: no
	# propagation happens, which is the point of checking it.
	var upark := 0
	while Sim.tractor_row != 4 and upark <= Sim.TRACTOR_KNOBS.size():
		main._input(_key(KEY_DOWN))
		upark += 1
	for _j in 30:
		main._input(_key(KEY_LEFT))
	assert(is_zero_approx(Sim.tractor.duty), "the duty knob must reach zero")
	main._input(_key(KEY_E))
	print("TRACTORSHOT  [E] at duty=0 -> probing=%s (expect false)" % Sim.tractor_probing)
	assert(not Sim.tractor_probing, "a zero-duration tow must be refused, not propagated")
	for _j in 30:
		main._input(_key(KEY_RIGHT))

	# And the far end of the lead knob: 11.5 orbits is inside the campaign's 12 yr
	# of seeded lead, but only just, and nothing had probed there.
	var lpark := 0
	while Sim.tractor_row != 3 and lpark <= Sim.TRACTOR_KNOBS.size():
		main._input(_key(KEY_DOWN))
		lpark += 1
	for _j in 60:
		main._input(_key(KEY_RIGHT))
	print("TRACTORSHOT  lead knob at max = %.2f orb (%.2f yr)"
		% [Sim.tractor.lead, Sim.tractor_lead_s() / (365.25 * 86400.0)])
	main._input(_key(KEY_E))
	if Sim.tractor_probing:
		var t3 := Time.get_ticks_msec()
		while Sim.tractor_probing and Time.get_ticks_msec() - t3 < 240000:
			await get_tree().process_frame
		var pl := Sim.tractor_probe()
		assert(not pl.is_empty(),
			"a probe at the lead knob's maximum must land, not fall off the \
			 propagated span")
		print("TRACTORSHOT  at max lead: %s" % Sim.tractor_probe_label())
	else:
		# A refusal here is acceptable ONLY if it is a named one; silence would
		# mean the knob offers a setting that quietly does nothing.
		print("TRACTORSHOT  at max lead the probe was refused: %s" % Sim.mission.last_error())
		assert(str(Sim.mission.last_error()) != "",
			"a refused probe must say why")

	await _settle(6)
	await _shot("tractor_bench")
	get_tree().quit(0)


## A pressed key event for `keycode`, to feed main._input() so the real action
## handlers (project.godot -> main.gd -> Sim) run rather than being bypassed.
func _key(keycode: int) -> InputEventKey:
	var ev := InputEventKey.new()
	ev.keycode = keycode
	ev.pressed = true
	return ev


func _settle(frames: int) -> void:
	for _i in frames:
		await get_tree().process_frame


## Save a frame — **and do nothing at all under `--headless`.**
##
## `RenderingServer.frame_post_draw` never fires with the dummy display driver, so
## awaiting it parks the coroutine forever: no error, no output, an apparent hang.
## `_tier2_shot.gd` has the same await and gets away with it only because its
## single capture is the last statement before `quit()` — everything it asserts
## has already printed. This harness takes three, with assertions after them, so
## it has to be honest about which runs produce pictures.
##
## The headless run is still worth doing and is the one wired into the loop: every
## assertion, formatter and `_draw` numbers-branch executes. The PNGs come from
## `godot --path godot --resolution 1600x900`.
func _shot(name: String) -> void:
	if DisplayServer.get_name() == "headless":
		print("TRACTORSHOT  (headless: skipping capture of %s)" % name)
		return
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var path := "%s/%s.png" % [OUT, name]
	img.save_png(path)
	print("TRACTORSHOT  wrote %s" % path)
