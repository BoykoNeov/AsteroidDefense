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

	# 0. THE POST ITSELF, before it is dismissed. It reports the machine's actual
	#    state, and since the kernel read moved onto a worker it has a third state
	#    to report: the read is typed as READING and rewritten when it lands. Warm
	#    the read is ~2 ms, so what this captures is the settled screen — the
	#    READING branch was photographed separately, by holding the landing, because
	#    two milliseconds is not photographable.
	# Wait for the typewriter, not a frame count: the POST types at 220 chars/s and
	# the ephemeris lines are a second in, so a four-frame settle photographs the
	# copyright banner and nothing that this screen is here to report.
	var tb := Time.get_ticks_msec()
	while is_instance_valid(main.boot) and not main.boot.is_typed() 			and Time.get_ticks_msec() - tb < 8000:
		await get_tree().process_frame
	await _settle(2)
	print("SHOT  boot: field_loading=%s bodies_online=%s at %d ms"
		% [Sim.field_loading, Sim.bodies_online, Time.get_ticks_msec()])
	await _shot("boot_1_post")

	main.boot.dismiss()
	await _settle(6)

	# The threat is ~10 s of real integration away. Nothing to draw until it lands.
	var t0 := Time.get_ticks_msec()
	while not Sim.mission_online and Time.get_ticks_msec() - t0 < 60000:
		await get_tree().process_frame
	print("SHOT  mission_online=%s after %d ms" % [Sim.mission_online, Time.get_ticks_msec() - t0])

	# 0a. THE SIXTEEN REAL ASTEROIDS (3D). The mount happens on the build worker, so
	#     this can only be checked after mission_online — and it must be checked in a
	#     picture, because "the nodes exist" and "the nodes are somewhere real" are
	#     different claims and only the second one matters. The specific failure being
	#     looked for: a body at ZERO, which in this heliocentric view is the Sun.
	# The 3D world builds its planet nodes from the field, which since the kernel
	# read went threaded can arrive *after* this scene does. When that wiring is
	# missing the symptom is not a blank screen: `_process` throws a missing-key
	# error on the first frame `bodies_online` is true and every line below it in
	# that function stops running, so bodies freeze at the origin and the comet's
	# span gate silently stops being applied. Count the nodes.
	if main.solar.body_nodes.size() != Sim.planets.size():
		print("SHOT  FAIL: %d planet nodes for %d planets - the world was built before the field landed"
			% [main.solar.body_nodes.size(), Sim.planets.size()])
	print("SHOT  small_bodies armed=%s mounted=%s count=%d"
		% [Sim.small_bodies_armed, Sim.mission.small_bodies_mounted(), Sim.asteroids.size()])
	Sim.paused = true
	Sim.jump(0.0)
	await _settle(4)
	await _shot("belt_1_real_asteroids")
	for i in mini(Sim.asteroids.size(), 16):
		var el: Dictionary = Sim.asteroids[i]
		var p: Vector3 = Sim.pos_ecl(el, Sim.t)
		print("SHOT  %-12s naif=%d r=%.3f AU node=%s"
			% [el.name, el.naif_id, p.length(),
				main.solar._asteroid_nodes[i].position])
	Sim.paused = false

	# 0b. THE REAL NEOs (3D) — Apophis, Bennu, Didymos, on JPL's own trajectory. The
	#     shot pair the span gate exists for, and the point of the whole commit: at an
	#     epoch inside their tables they are named bodies out in the field, and past
	#     the table's end they are GONE — not on the Sun, which is what ZERO draws as
	#     here. The on-arc shot is scrubbed to the 2029 Apophis flyby, when the real
	#     object is genuinely near Earth. No passive run scrubs to a named year.
	print("SHOT  neos=%d" % Sim.neos.size())
	if Sim.neos.size() > 0:
		var apophis: Dictionary = Sim.neos[0]
		for el in Sim.neos:
			if str(el.name).contains("Apophis"):
				apophis = el
		Sim.paused = true
		# 2029-04-13, the flyby — ~471 days past the 2028-01-01 epoch.
		Sim.jump(468.0)
		await _settle(4)
		await _shot("neo_1_on_arc")
		for el in Sim.neos:
			var p: Vector3 = Sim.pos_ecl(el, Sim.t)
			print("SHOT  %-14s prov=%s active=%s r=%.3f AU"
				% [el.name, el.provenance, Sim.catalog_active(el, Sim.t), p.length()])

		Sim.jump(apophis.t_max + 400.0)        # past the end of the table
		await _settle(4)
		await _shot("neo_2_past_span_gone")
		print("SHOT  apophis past span: t=%.0f d active=%s (must be false — ZERO is the Sun)"
			% [Sim.t, Sim.catalog_active(apophis, Sim.t)])
		Sim.jump(0.0)
		Sim.paused = false
		await _settle(2)

	# 0. THE COMET (3D), in the 3D solar view — the shot pair the span gate exists
	#    for. On its arc it must be a body out in the field; past the end of its
	#    propagated orbit it must be GONE, and specifically not sitting on the Sun,
	#    which is what ZERO draws as in this heliocentric frame. Nothing else here
	#    checks that, and no passive run scrubs past 22.6 years.
	#    It also has to be WAITED for. `mission_online` no longer means the catalog
	#    is complete: the comet flies on its own worker started at install, so every
	#    shot above this point was taken while it was still in the air. Without this
	#    wait the harness photographs an empty `comet_el`, drives `pos_ecl` into the
	#    "no known source" guard and then a null node — and, because that happens
	#    inside an `await` chain, the run does not fail, it HANGS until the runner
	#    kills it, taking every later shot with it.
	var t_comet := Time.get_ticks_msec()
	while Sim._comet_pending and Time.get_ticks_msec() - t_comet < 120000:
		await get_tree().process_frame
	print("SHOT  comet_online=%s arc=%s (landed %d ms after the threat)"
		% [Sim.comet_online, Sim.comet_arc_label(), Time.get_ticks_msec() - t_comet])
	if not Sim.comet_online:
		# Say so and carry on rather than crash: the comet is scenery, and the
		# encounter and planner shots below are the ones that check physics.
		print("SHOT  FAIL: the comet never reached the catalog - skipping its two shots")
	else:
		Sim.paused = true
		Sim.jump(Sim.T_IMPACT)                 # perihelion falls near the impact epoch
		await _settle(4)
		await _shot("comet_1_on_arc")
		var p_on: Vector3 = Sim.pos_ecl(Sim.comet_el, Sim.t)
		print("SHOT  comet on arc: t=%.0f d active=%s r=%.2f AU node_visible=%s"
			% [Sim.t, Sim.catalog_active(Sim.comet_el, Sim.t), p_on.length(),
				main.solar.comet_node.visible])

		Sim.jump(Sim.comet_el.t_max + 400.0)   # past the end of the propagated orbit
		await _settle(4)
		await _shot("comet_2_past_span_gone")
		print("SHOT  comet past span: t=%.0f d active=%s node_visible=%s (must be false — ZERO is the Sun)"
			% [Sim.t, Sim.catalog_active(Sim.comet_el, Sim.t), main.solar.comet_node.visible])
		Sim.jump(0.0)
		Sim.paused = false
		await _settle(2)

	# 0c. PHOSPHOR PERSISTENCE, which only shows while things move: at the top warp
	#     step every inner body sweeps degrees of orbit per frame, so each should
	#     trail a fading arc behind it, while the HUD text and tags (drawn above
	#     the persisted layer) stay sharp. A paused shot cannot show this, and no
	#     other shot here runs the clock.
	Sim.paused = false
	Sim.jump(0.0)
	Sim.warp_idx = Sim.WARP_STEPS.size() - 1
	await _settle(40)
	await _shot("trails_1_max_warp")
	print("SHOT  trails: warp=%s t=%.0f d (bodies should streak, text should not)"
		% [Sim.warp_label(), Sim.t])

	# 0d. THE SAME FRAME WITH PERSISTENCE OFF. The pair is the check: the first
	#     shot must show ghost chains behind the planets and the second must show
	#     none, at the same warp and the same clock. Driven through the InputMap
	#     rather than by setting `persist_idx`, because what is being checked is
	#     the promise the HUD makes to a player - a hand-written action block in
	#     project.godot and a dispatch branch in main.gd sit between [I] and the
	#     variable, and setting the variable would test neither.
	for _i in main.PERSIST_TAUS.size() - 1:
		await _press("persist_cycle")
	await _settle(40)
	await _shot("trails_2_persist_off")
	print("SHOT  persist: idx=%d tau=%.2f belt_dim_set_for_warp=%d (0.00 = off)"
		% [main.persist_idx, main.PERSIST_TAUS[main.persist_idx], Sim.warp_idx])
	await _press("persist_cycle")     # back to the default rung for later shots
	Sim.warp_idx = 3
	Sim.jump(0.0)
	Sim.paused = false

	# Show the b-plane view: exactly what [3] does, without an InputMap round-trip.
	main._show_view(main.enc)
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
	# This used to hand-code the ritual — pause, then jump to T_IMPACT — because
	# reaching the marker took both steps and a warp step overshoots closest
	# approach by 0.53 d, by which point the rock is ~1.3 LD out and off-plot (the
	# gate behaving correctly, but not the branch being checked). That ritual is
	# now the [C] key, so the harness drives *that* instead: it verifies the shipped
	# path rather than a private re-implementation of it, and if the snap ever stops
	# landing in the window, this shot goes empty and says so.
	main._jump_to_closest_approach()
	await _settle(4)
	await _shot("enc_6_live_marker_burned")
	print("SHOT  marker: t=%.3f d (impact %.3f, CA %+.3f d), committed=%s burned=%s, window=%s"
		% [Sim.t, Sim.T_IMPACT, Sim.t - Sim.T_IMPACT, Sim.committed, Sim.burned(),
			Sim.encounter_span_days()])
	print("SHOT  marker live on screen: %s" % main.enc._marker_live)
	Sim.paused = false
	Sim.jump(0.0)
	await _settle(2)

	# 6b. The same view with the keyhole map off — [H] — so the circles are shown to
	#     be a toggle and not baked into the plot.
	main.enc.toggle_keyholes()
	await _settle(3)
	await _shot("enc_7_keyholes_off")
	main.enc.toggle_keyholes()

	# 6c. THE TIER-3 UNCERTAINTY OVERLAY — [U]. Three pictures, because one would
	#     not be honest about any of them.
	#
	#     The overlay is the first thing on this screen that cannot be drawn the
	#     frame it is asked for: the sensitivity behind it is 13 propagations. So
	#     the first shot is deliberately taken at the DEFAULT zoom, where the 1σ
	#     ellipse is ~1 px across and the view says so instead of drawing it. That
	#     is the finding, not a failure — the crossing of this rock is known to a
	#     couple of hundred kilometres inside a capture disc eleven thousand
	#     kilometres wide, and a shape fattened up to be visible would be a lie
	#     about how well the orbit is known.
	#     The keys are exercised through the InputMap, not by calling the methods:
	#     the footer promises [U] and [Z]/[X] to a player, and three hand-written
	#     action blocks in project.godot plus three dispatch branches in main.gd sit
	#     between that promise and the method. Calling the method directly would
	#     test everything except the part that was typed by hand.
	main._show_view(main.enc)
	for a in ["encounter_uncertainty", "encounter_sigma_down", "encounter_sigma_up"]:
		print("SHOT  action %s registered=%s events=%d" % [a, InputMap.has_action(a),
			InputMap.action_get_events(a).size() if InputMap.has_action(a) else 0])
	await _press("encounter_uncertainty")
	var tu0 := Time.get_ticks_msec()
	while Sim.tier3_solving and Time.get_ticks_msec() - tu0 < 180000:
		await get_tree().process_frame
	print("SHOT  tier3_online=%s after %d ms" % [Sim.tier3_online, Time.get_ticks_msec() - tu0])
	print("SHOT  tier3 ellipse: %s" % Sim.tier3)
	await _settle(4)
	await _shot("enc_9_uncertainty_subpixel")

	#     Zoomed to where the ellipse is a shape. It comes out lying almost exactly
	#     along ζ̂ — the TIMING axis — which is the same coordinate the keyhole work
	#     found a Δv nudge moves. Worth a picture: it is the reason the core rotates
	#     the covariance into this view's frame instead of reporting an angle in the
	#     arbitrary frame the sensitivity was solved in, where it would have pointed
	#     nowhere in particular.
	main.enc._half_ld = 0.025
	await _settle(3)
	await _shot("enc_10_uncertainty_ellipse")

	#     And the σ knob, the layer's actual lesson: the same rock, the same
	#     trajectory, a spread that grows purely because the orbit is assumed less
	#     well observed. Two decades wider is where P finally leaves 1.
	main.enc._half_ld = 0.15
	for _k in 4:
		await _press("encounter_sigma_up")
	await _settle(3)
	await _shot("enc_11_uncertainty_sigma_x100")
	print("SHOT  tier3 at x100: major %s km, P %s"
		% [Sim.tier3.get("major_km", 0.0), Sim.tier3.get("p_impact", 0.0)])
	for _k in 4:
		await _press("encounter_sigma_down")
	await _press("encounter_uncertainty")

	# 7. The planner beside it — the two panels must agree, and this is the pair a
	#    player reads against each other.
	main._show_view(null)
	main.planner.visible = true
	Sim.planner_open = true
	Sim.set_plan(Sim.threat_period_d(), 0.2, true)
	Sim._tick_plan_debounce(1.0)
	await _settle(4)
	await _shot("enc_5_planner_agrees")
	print("SHOT  planner keyhole: %s | %s (alert=%s)"
		% [Sim.keyhole_label(), Sim.keyhole_note(), Sim.keyhole_alert()])

	# 8. The keyhole row with something to say. The default plan is hundreds of
	#    widths from anything, which only ever exercises the CLEAR wording - and
	#    the whole point of the row is the other case.
	#
	#    Measured 2026-09-05: a coarse sweep at the longest lead the planner
	#    allows never gets closer than ~5,000 km to a circle, which reads as "a
	#    player cannot reach a keyhole". That conclusion is an artifact of the
	#    step size. As the impulse is dialled the b-point walks OUTWARD past one
	#    resonant circle after another, so consecutive rungs bracket crossings;
	#    the coarse sweep just steps over them. So: sweep to bracket, then refine
	#    inside the best bracket, and shoot what a player would actually see with
	#    their finger on the key.
	var rungs := [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.6, 0.9, 1.4]
	var best_dv := 0.0
	var best_w := INF
	var best_i := -1
	for i in rungs.size():
		var w := await _keyhole_margin_at(rungs[i])
		print("SHOT  keyhole sweep dv=%.2f -> %s" % [rungs[i], Sim.keyhole_label()])
		if w < best_w:
			best_w = w
			best_dv = rungs[i]
			best_i = i
	# Golden-section inside the neighbours of the best rung. Non-smooth where the
	# nearest circle changes identity, which is fine: any minimum it finds is a
	# real place a player can dial to, and that is all this picture claims.
	if best_i >= 0:
		var lo: float = rungs[maxi(best_i - 1, 0)]
		var hi: float = rungs[mini(best_i + 1, rungs.size() - 1)]
		var phi := 0.5 * (sqrt(5.0) - 1.0)
		var x1 := hi - phi * (hi - lo)
		var x2 := lo + phi * (hi - lo)
		var f1 := await _keyhole_margin_at(x1)
		var f2 := await _keyhole_margin_at(x2)
		for _i in 10:
			if f1 < f2:
				hi = x2
				x2 = x1
				f2 = f1
				x1 = hi - phi * (hi - lo)
				f1 = await _keyhole_margin_at(x1)
			else:
				lo = x1
				x1 = x2
				f1 = f2
				x2 = lo + phi * (hi - lo)
				f2 = await _keyhole_margin_at(x2)
		best_dv = x1 if f1 < f2 else x2
		best_w = minf(f1, f2)
		await _keyhole_margin_at(best_dv)
		await _settle(4)
		await _shot("enc_8_planner_keyhole")
		# `widths_away` is printed beside the margin on purpose: it is the rule
		# that shipped until 2026-09-07 (alert when <= 4.0 half-widths), and the
		# claim that the additive band catches plans that rule called CLEAR is
		# only worth making if both numbers come off the same plan.
		var row: Dictionary = Sim.plan_keyhole.get("at_risk", {})
		var near: Dictionary = Sim.plan_keyhole.get("nearest", {})
		print("SHOT  closest a player can dial: dv=%.5f at %d d (margin %.1f km, d %.3f km, door %.3f km, old rule %.2f half-widths vs cut 4.0) -> %s (alert=%s)"
			% [best_dv, int(Sim.plan_lead_d), best_w,
				absf(row.get("distance_km", INF)), float(row.get("width_km", 0.0)),
				float(row.get("widths_away", INF)),
				Sim.keyhole_label(), Sim.keyhole_alert()])
		# Which circle each ranking names, printed off the same plan. The note's
		# disagreement branch only ever fires when these two differ, so a run in
		# which they agree is a run that never exercised it — say so here rather
		# than reading the fallback text as evidence the branch works.
		print("SHOT  rankings: nearest %d:%d (margin %.1f km) | at_risk %d:%d (margin %.1f km) -> %s"
			% [int(near.get("h", 0)), int(near.get("k", 0)),
				Sim.keyhole_margin_km(near),
				int(row.get("h", 0)), int(row.get("k", 0)),
				Sim.keyhole_margin_km(row),
				"NO ROWS" if near.is_empty() or row.is_empty()
					else ("DISAGREE" if int(near.get("h", 0)) != int(row.get("h", 0))
						or int(near.get("k", 0)) != int(row.get("k", 0)) else "agree")])
		print("SHOT  keyhole note: %s" % Sim.keyhole_note())

	get_tree().quit(0)


## Solve one plan at the longest allowed lead and report how far outside the
## nearest resonant return's door it lands, kilometres. INF when there is
## nothing to measure against (no b-point, no map). Minimising the same
## quantity the panel is cut on keeps the picture and the wording agreeing —
## which is why this reads the `at_risk` row and not the `nearest` one: the
## alert is cut on the smallest margin in the whole census, so a search that
## minimised the nearest circle's margin would be optimising a different number
## from the one it is trying to make the panel print.
func _keyhole_margin_at(dv: float) -> float:
	Sim.set_plan(Sim.LEAD_MAX, dv, true)
	Sim._tick_plan_debounce(1.0)
	await _settle(1)
	return Sim.keyhole_margin_km(Sim.plan_keyhole.get("at_risk", {}))


## Press one mapped action the way a player does — through `_unhandled_input`, so
## the binding in project.godot and the dispatch branch in main.gd are both on the
## path. `Input.parse_input_event` is the only way to reach that from a script.
func _press(action: String) -> void:
	var events: Array[InputEvent] = InputMap.action_get_events(action)
	if events.is_empty():
		print("SHOT  FAIL: action %s has no binding" % action)
		return
	var down: InputEvent = events[0].duplicate()
	down.pressed = true
	Input.parse_input_event(down)
	await _settle(2)
	var up: InputEvent = events[0].duplicate()
	up.pressed = false
	Input.parse_input_event(up)
	await _settle(1)


func _settle(frames: int) -> void:
	for _i in frames:
		await get_tree().process_frame


func _shot(name: String) -> void:
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var path := "%s/%s.png" % [OUT, name]
	img.save_png(path)
	print("SHOT  wrote %s" % path)
