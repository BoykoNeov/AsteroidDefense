extends Node
## Temporary threat-orbit designer harness — the only thing that runs
## ThreatPanel._draw().
##   godot --path godot --resolution 1600x900   (non-headless → real PNGs)
##   godot --headless --path godot              (_draw still runs for VISIBLE nodes
##                                                → verifies the numbers branch)
## Registered as an autoload while running, removed afterwards. Not shipped.
##
## Same reason as `_tractor_shot.gd`: `_draw` runs under `--headless` but only for
## VISIBLE nodes, and this panel is hidden until [N].
##
## # This harness DOES drive the rebuild, and that was a choice
##
## The rebuild is ~10 s and the anchor solve 41–74 s on the orbit this dials to,
## so covering them costs about a minute of bounded waiting. The alternative —
## cover only the free preview and the refusals, and verify the rest by picture —
## was tempting and rejected: the rebuild is the only path that exercises
## **invalidation**, and invalidation
## failures are invisible by construction. A launch-window map left lit over a
## replaced threat, or a tractor margin still quoting the previous orbit's
## requirement, both look completely healthy on screen. A picture would not catch
## either.
##
## The waits are bounded `while` loops with an `await` inside, per the hazard this
## project already hit once: an unguarded `while` around synchronous key presses
## freezes the engine with no output at all.
##
## What it guards beyond "does it draw":
##
## 1. **Both walls, and both refusals.** They close from opposite directions —
##    too slow for the offset, and too wide to be a hit — and each must be
##    refused in microseconds by the preview rather than after a 10 s build.
## 2. **The preview's two numbers are not the two knobs.** `v_inf` is not the
##    speed knob (that one is measured deep in Earth's well) and the b-plane miss
##    is not the offset knob (focusing more than doubles it). A panel that
##    conflated either would look entirely reasonable.
## 3. **A rebuild invalidates everything downstream of the old orbit** — and
##    leaves the operator's tractor knobs alone, which is the opposite mistake.
## 4. **The required-Δv anchor goes absent on a rebuilt orbit and comes back
##    from [E]**, rather than silently continuing to quote the shipping constant.

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
		print("THREATSHOT  FAIL: no Main node")
		get_tree().quit(1)
		return

	DirAccess.make_dir_recursive_absolute(OUT)
	main.boot.dismiss()
	await _settle(6)

	var t0 := Time.get_ticks_msec()
	while not Sim.mission_online and Time.get_ticks_msec() - t0 < 180000:
		await get_tree().process_frame
	print("THREATSHOT  mission_online=%s after %d ms"
		% [Sim.mission_online, Time.get_ticks_msec() - t0])
	assert(Sim.mission_online, "no threat solution — nothing below can run")

	main.enc.visible = false
	main.map2d.visible = false
	main.planner.visible = false

	# The knobs must have been read back off the CORE's installed config, not
	# restated here or in Sim. `ImpactorConfig::default()`'s direction is
	# (0.6, -0.7, 0.2), which normalizes to az ~ -49.4 deg, el ~ +12.2 deg.
	print("THREATSHOT  seeded v_rel=%.1f km/s az=%+.1f el=%+.1f offset=%.0f km (max %.0f)"
		% [Sim.threat_knobs.v_rel, Sim.threat_knobs.az, Sim.threat_knobs.el,
			Sim.threat_knobs.offset, Sim.threat_offset_max])
	assert(Sim.threat_knobs_are_installed(),
		"the panel must open on the orbit that is actually installed")
	assert(Sim.threat_offset_max > 6000.0 and Sim.threat_offset_max < 6500.0,
		"the offset ceiling must be Earth's radius from the core, got %.0f"
			% Sim.threat_offset_max)
	# The shipping orbit's requirement is known for free — it is a recorded
	# constant, not a solve. If this is false at boot every margin in the tractor
	# bench has silently gone absent.
	assert(Sim.threat_anchor_known(), "the shipping orbit must arrive with its anchor seeded")

	assert(not main.threat_panel.visible, "panel starts hidden")
	main._input(_key(KEY_N))
	print("THREATSHOT  [N] -> panel.visible=%s (expect true)" % main.threat_panel.visible)
	assert(main.threat_panel.visible, "[N] must open the designer via the action")

	# --- the three bottom-centre panels are mutually exclusive -----------------
	# All three draw at the same origin and all three claim the arrows through one
	# elif chain, so two open at once means one silently stops responding.
	main._input(_key(KEY_K))
	print("THREATSHOT  [K] with designer up -> threat=%s tractor=%s (expect false/true)"
		% [main.threat_panel.visible, main.tractor_panel.visible])
	assert(not main.threat_panel.visible and main.tractor_panel.visible,
		"opening the bench must close the designer")
	main._input(_key(KEY_N))
	assert(main.threat_panel.visible and not main.tractor_panel.visible,
		"and opening the designer must close the bench")

	# --- the free preview: neither number is the knob it resembles -------------
	var p := Sim.threat_preview()
	assert(bool(p.ok), "the shipping geometry must preview cleanly")
	var v_inf_kms := float(p.v_inf_m_s) / 1000.0
	var b_km := float(p.impact_parameter_m) / 1000.0
	print("THREATSHOT  preview: v_inf=%.3f km/s (knob %.1f)  b=%.0f km (offset %.0f, capture %.0f)  T=%.3f yr"
		% [v_inf_kms, Sim.threat_knobs.v_rel, b_km, Sim.threat_knobs.offset,
			float(p.capture_radius_m) / 1000.0,
			float(p.period_seconds) / (365.25 * 86400.0)])
	assert(bool(p.is_hit), "the shipping geometry must still be a designed impact")
	# v_inf is what is left after climbing out of Earth's well; the knob is the
	# speed at the impact point, deep inside it. Conflating them is how the
	# capture radius gets computed from the wrong speed.
	assert(v_inf_kms < 0.6 * Sim.threat_knobs.v_rel,
		"v_inf must be well below the impact-point speed, got %.3f vs %.1f"
			% [v_inf_kms, Sim.threat_knobs.v_rel])
	# And the b-plane miss is NOT the offset: focusing widens it past 2x.
	assert(b_km > 2.0 * Sim.threat_knobs.offset,
		"the asymptote must miss by more than twice the aim point, got %.0f vs %.0f"
			% [b_km, Sim.threat_knobs.offset])

	# --- every knob, through its real key --------------------------------------
	# Read out of THREAT_KNOBS so a fifth knob is covered by existing.
	for i in Sim.THREAT_KNOBS.size():
		var knob: Array = Sim.THREAT_KNOBS[i]
		var id: String = knob[0]
		var hops := 0
		while Sim.threat_row != i and hops <= Sim.THREAT_KNOBS.size():
			main._input(_key(KEY_DOWN))
			hops += 1
		assert(Sim.threat_row == i, "[DOWN] must reach row %d — is the cursor wired?" % i)
		var before: float = Sim.threat_knobs[id]
		# Same lesson the bench's duty knob taught: step whichever way can move.
		var out_key: int = KEY_LEFT if before >= float(knob[4]) else KEY_RIGHT
		var back_key: int = KEY_RIGHT if out_key == KEY_LEFT else KEY_LEFT
		main._input(_key(out_key))
		var after: float = Sim.threat_knobs[id]
		print("THREATSHOT  row %d %-8s %s %+.3f -> %+.3f"
			% [i, id, "[LEFT]" if out_key == KEY_LEFT else "[RIGHT]", before, after])
		assert(after != before, "knob %s did not move in either direction" % id)
		main._input(_key(back_key))
		assert(is_equal_approx(Sim.threat_knobs[id], before),
			"stepping knob %s back must restore it" % id)

	# --- WALL 2: the offset ceiling is EXACTLY the grazing boundary -------------
	# The offset is laid perpendicular to the relative velocity, so it IS the
	# geocentric perigee — and `b <= b_capture` is the same statement as
	# `perigee <= R_E`. Winding the knob to its ceiling therefore lands precisely
	# on `b == b_capture`: the widest impact that exists.
	#
	# That equality is a far stronger check than `is_hit` would be. `b` and
	# `b_capture` are computed by different formulas from different quantities
	# (`h/v_inf` against `R_E*sqrt(1+(v_esc/v_inf)^2)`), and they meet here only
	# because the derivation is right. A wrong `v_inf` moves both and `is_hit`
	# would still flip somewhere plausible-looking.
	var opark := 0
	while Sim.threat_row != 3 and opark <= Sim.THREAT_KNOBS.size():
		main._input(_key(KEY_DOWN))
		opark += 1
	assert(Sim.threat_row == 3, "could not park the cursor on the offset row")
	for _i in 80:
		main._input(_key(KEY_RIGHT))
	var wide := Sim.threat_preview()
	var wb := float(wide.impact_parameter_m)
	var wc := float(wide.capture_radius_m)
	print("THREATSHOT  offset clamped at %.0f km (core max %.0f) -> is_hit=%s b=%.1f capture=%.1f (b/cap %.6f)"
		% [Sim.threat_knobs.offset, Sim.threat_offset_max, wide.get("is_hit"),
			wb / 1000.0, wc / 1000.0, wb / wc])
	assert(Sim.threat_knobs.offset <= Sim.threat_offset_max + 1e-6,
		"the offset knob must clamp at Earth's radius")
	# Tight on purpose. At 1e-5 this passed while the knob was clamping 136.6 m
	# short of Earth's radius, because a placeholder in the knob table read 6378.0
	# and was the binding bound instead of the core's 6378.1366. A loose tolerance
	# here hides exactly the class of bug the "never restate a core constant" rule
	# exists to prevent.
	assert(absf(wb / wc - 1.0) < 1e-9,
		"at the ceiling the asymptote must graze the capture disc exactly \
		 (b/capture = %.10f) — that equality IS `perigee == R_E`, so a miss means \
		 something other than the core is setting the knob's bound" % (wb / wc))
	# So the knob cannot produce a miss at all, which is the intended protection:
	# this panel designs an *impactor*, and a geometry that misses Earth leaves
	# the campaign nothing to deflect. The binding's `is_hit` refusal is therefore
	# unreachable from the UI — the same relationship the tractor bench's
	# `holds_station` branch has to its hover clamp. It is still exercised, by
	# calling past the clamp the way any future caller could.
	assert(bool(wide.is_hit), "the ceiling is the last hitting offset, not the first missing one")
	var past: bool = Sim.mission.begin_rebuild_scenario(
		Sim.threat_knobs.v_rel, Sim.threat_knobs.az, Sim.threat_knobs.el,
		Sim.threat_offset_max + 500.0)
	print("THREATSHOT  binding called 500 km past the clamp -> started=%s err='%s'"
		% [past, Sim.mission.last_error()])
	assert(not past, "a geometry that misses Earth must be refused, not built")
	assert(str(Sim.mission.last_error()).contains("capture"),
		"and the refusal must name the disc it missed, not fail generically")
	await _settle(4)
	await _shot("threat_offset_ceiling")

	# --- WALL 1: too slow for the offset ---------------------------------------
	# The counter-intuitive one: SHRINKING the offset raises the escape speed that
	# must be cleared, so pulling the hit toward Earth's centre is what falls off
	# the hyperbolic cliff.
	for _i in 80:
		main._input(_key(KEY_LEFT))
	var tight := Sim.threat_preview()
	print("THREATSHOT  offset floored at %.0f km -> ok=%s err='%s'"
		% [Sim.threat_knobs.offset, tight.get("ok"), tight.get("error", "")])
	assert(not bool(tight.ok),
		"at a small offset the shipping speed must stop being a flyby at all")
	main._input(_key(KEY_ENTER))
	assert(not Sim.threat_rebuilding, "a non-hyperbolic geometry must be refused too")
	await _settle(4)
	await _shot("threat_not_a_flyby")
	# Back to a legal offset.
	for _i in 80:
		main._input(_key(KEY_RIGHT))
	for _i in 34:
		main._input(_key(KEY_LEFT))
	assert(bool(Sim.threat_preview().get("is_hit", false)), "must be back to a designed hit")

	# --- move the rock to a long-period orbit, USING the preview as the oracle --
	# Which is what an operator does: turn azimuth and watch the period. Bounded,
	# and it targets a *measured* property rather than a magic angle.
	var shipping_period: float = Sim.mission.period_seconds()
	var apark := 0
	while Sim.threat_row != 1 and apark <= Sim.THREAT_KNOBS.size():
		main._input(_key(KEY_DOWN))
		apark += 1
	assert(Sim.threat_row == 1, "could not park the cursor on the azimuth row")
	var turns := 0
	while turns < 90:
		var pv := Sim.threat_preview()
		if bool(pv.get("ok", false)) and bool(pv.get("is_hit", false)) \
				and float(pv.period_seconds) > 1.8 * shipping_period:
			break
		main._input(_key(KEY_LEFT))
		turns += 1
	var target := Sim.threat_preview()
	print("THREATSHOT  after %d [LEFT] on azimuth: az=%+.1f -> T=%.3f yr (shipping %.3f)"
		% [turns, Sim.threat_knobs.az, float(target.period_seconds) / (365.25 * 86400.0),
			shipping_period / (365.25 * 86400.0)])
	assert(float(target.period_seconds) > 1.8 * shipping_period,
		"the azimuth knob must be able to reach a much longer period")
	assert(not Sim.threat_knobs_are_installed(),
		"dialled knobs that differ from the build must read as not installed")
	await _settle(4)
	await _shot("threat_designer")

	# --- state that must SURVIVE the rebuild, recorded first -------------------
	# The tractor bench is a separate experiment. A rebuild that reset the
	# operator's spacecraft to 20 t would make "change the orbit, watch the margin
	# move" a comparison between two different tractors.
	var park_mass := 0
	while Sim.tractor_row != 0 and park_mass <= Sim.TRACTOR_KNOBS.size():
		Sim.move_tractor_cursor(1)
		park_mass += 1
	for _i in 6:
		Sim.adjust_tractor(1)
	var tractor_mass_before: float = Sim.tractor.mass
	var tractor_hover_before: float = Sim.tractor.hover
	print("THREATSHOT  tractor set to %.1f t at %.3f R before the rebuild"
		% [tractor_mass_before, tractor_hover_before])
	assert(tractor_mass_before > 20.0, "the mass knob must actually have moved")

	# --- and state that must DIE with it ---------------------------------------
	Sim.request_porkchop()
	var tg := Time.get_ticks_msec()
	while Sim.pork_building and Time.get_ticks_msec() - tg < 120000:
		await get_tree().process_frame
	print("THREATSHOT  launch-window map built: pork_online=%s (%d x %d)"
		% [Sim.pork_online, Sim.pork_rows, Sim.pork_cols])
	assert(Sim.pork_online, "need a built grid to prove a rebuild invalidates it")

	# --- THE REBUILD ------------------------------------------------------------
	main._input(_key(KEY_ENTER))
	print("THREATSHOT  [ENTER] -> rebuilding=%s (expect true)" % Sim.threat_rebuilding)
	assert(Sim.threat_rebuilding, "[ENTER] must start the rebuild")
	var t1 := Time.get_ticks_msec()
	while Sim.threat_rebuilding and Time.get_ticks_msec() - t1 < 240000:
		await get_tree().process_frame
	print("THREATSHOT  rebuild landed after %d ms" % (Time.get_ticks_msec() - t1))
	assert(not Sim.threat_rebuilding, "the rebuild must finish inside the timeout")
	assert(Sim.mission_online, "the threat must still be online after a rebuild")

	var new_period: float = Sim.mission.period_seconds()
	print("THREATSHOT  installed period %.3f yr (was %.3f)"
		% [new_period / (365.25 * 86400.0), shipping_period / (365.25 * 86400.0)])
	assert(new_period > 1.5 * shipping_period,
		"the rebuild must actually have moved the threat's orbit")
	assert(Sim.threat_knobs_are_installed(),
		"after a successful rebuild the knobs must read as installed")
	# The closed-form preview predicted this period before anything was built. It
	# is an estimate (~0.2%), so this is a band, not an equality — but a wide miss
	# would mean the panel had been labelling knobs with the wrong orbit.
	var pred_err: float = absf(float(target.period_seconds) - new_period) / new_period
	print("THREATSHOT  preview predicted %.4f yr, build produced %.4f yr (%.3f pct)"
		% [float(target.period_seconds) / (365.25 * 86400.0),
			new_period / (365.25 * 86400.0), 100.0 * pred_err])
	assert(pred_err < 0.02,
		"the free preview must have predicted the built period to within a couple of \
		 percent; was off by %.2f pct" % (100.0 * pred_err))

	# Invalidation: everything solved against the old orbit is gone.
	print("THREATSHOT  after rebuild: pork_online=%s probe_empty=%s anchor_known=%s"
		% [Sim.pork_online, Sim.tractor_probe().is_empty(), Sim.threat_anchor_known()])
	assert(not Sim.pork_online,
		"a rebuilt threat must take the launch-window map with it")
	assert(Sim.pork_rows == 0 and Sim.pork_c3.is_empty(),
		"and the grid's columns, not merely its flag")
	assert(Sim.tractor_probe().is_empty(),
		"a tow probe is a perigee on the OLD trajectory and must not survive")
	assert(not Sim.mission.has_plan(), "the plan belonged to the old orbit")

	# The anchor is the one that would fail silently: everything above is visibly
	# blank, while a stale margin looks exactly like a live one.
	assert(not Sim.threat_anchor_known(),
		"a rebuilt orbit must NOT inherit the shipping orbit's required Δv")
	var stale := Sim.tractor_readout()
	print("THREATSHOT  tractor on the new orbit: required=%s margin=%s (expect both ABSENT)"
		% ["PRESENT" if stale.has("required_dv_m_s") else "ABSENT",
			"PRESENT" if stale.has("margin") else "ABSENT"])
	assert(not stale.has("required_dv_m_s") and not stale.has("margin"),
		"with no anchor the bench must print no requirement and no margin")

	# …and the operator's bench survived untouched.
	print("THREATSHOT  tractor after rebuild: %.1f t at %.3f R (was %.1f t at %.3f R)"
		% [Sim.tractor.mass, Sim.tractor.hover, tractor_mass_before, tractor_hover_before])
	assert(is_equal_approx(Sim.tractor.mass, tractor_mass_before)
			and is_equal_approx(Sim.tractor.hover, tractor_hover_before),
		"a rebuild must not reset the operator's tractor knobs — the bench and the \
		 orbit are separate experiments")
	await _settle(4)
	await _shot("threat_rebuilt_unmeasured")

	# --- [E]: solve this orbit's requirement, and the margin comes back --------
	main._input(_key(KEY_E))
	print("THREATSHOT  [E] -> anchor_solving=%s (expect true)" % Sim.threat_anchor_solving)
	assert(Sim.threat_anchor_solving, "[E] must fire the anchor solve")
	var t2 := Time.get_ticks_msec()
	while Sim.threat_anchor_solving and Time.get_ticks_msec() - t2 < 300000:
		await get_tree().process_frame
	print("THREATSHOT  anchor solved after %d ms: %.6f m/s at one orbit"
		% [Time.get_ticks_msec() - t2, Sim.mission.required_dv_anchor()])
	assert(Sim.threat_anchor_known(), "the anchor solve must land")
	var anchor: float = Sim.mission.required_dv_anchor()
	assert(anchor > 0.0, "a requirement of zero is not a requirement")
	# It must be this ORBIT's requirement, not the shipping one restated.
	assert(absf(anchor - 0.50975) / 0.50975 > 0.2,
		"a long-period orbit must want a materially different Δv than the shipping \
		 0.50975 m/s; got %.6f" % anchor)

	var scored := Sim.tractor_readout()
	print("THREATSHOT  tractor now: required=%s margin=%s"
		% [scored.get("required_dv_m_s", "ABSENT"),
			("%.4f" % scored.margin) if scored.has("margin") else "ABSENT"])
	assert(scored.has("required_dv_m_s") and scored.has("margin"),
		"with an anchor the bench must print a requirement and a margin again")
	await _settle(6)
	await _shot("threat_rebuilt_measured")
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


## Save a frame — and do nothing under `--headless`, where
## `RenderingServer.frame_post_draw` never fires and awaiting it parks the
## coroutine forever with no error and no output. See `_tractor_shot.gd`.
func _shot(name: String) -> void:
	if DisplayServer.get_name() == "headless":
		print("THREATSHOT  (headless: skipping capture of %s)" % name)
		return
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var path := "%s/%s.png" % [OUT, name]
	img.save_png(path)
	print("THREATSHOT  wrote %s" % path)
