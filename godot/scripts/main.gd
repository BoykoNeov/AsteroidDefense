extends Control
## Root assembler: SubViewport (3D world + HUD layers) shown through the
## CRT phosphor shader. Owns global hotkeys, phosphor theme toggle, and
## the 3D/2D view switch. Native-resolution rendering throughout — retro
## comes from the styling layer, never from downscaling.

const CRT_SHADER := preload("res://shaders/crt.gdshader")
const PERSIST_SHADER := preload("res://shaders/phosphor_persist.gdshader")

const PHOSPHOR_GREEN := Color(0.25, 1.0, 0.45)
const PHOSPHOR_AMBER := Color(1.0, 0.62, 0.13)
## Phosphor persistence time constants, seconds: a trail left by a moving body
## fades to 1/e of its brightness in this long. Wall time, not frames, so the
## look does not depend on the refresh rate.
##
## A ladder rather than one number, cycled by `[I]`, because persistence is the
## one effect on this screen that can actively hide the thing it decorates. At
## the top warp step the scenery belt sweeps into a solid band and the inner
## planets into rings; that is a true picture of 10 yr/s and it is also a wall.
## **0.0 means off** — `keep = 0`, nothing held from the previous frame — and it
## is a real entry, not a limit approached: a reader who wants to know where a
## body *is* rather than where it has been needs the effect gone, not shortened.
const PERSIST_TAUS: Array[float] = [0.14, 0.35, 0.0]

var crt_mat: ShaderMaterial
var viewport: SubViewport
## The 3D world, rendered in its own viewport (see `_ready`). Only the 3D line
## work goes through here; every Control is in `viewport` above it.
var world_vp: SubViewport
## Accumulates `world_vp` frames into the phosphor-persistence image.
var persist_vp: SubViewport
var persist_mat: ShaderMaterial
var _persist_rect: ColorRect
## Shows the persisted world inside the main viewport, under the HUD.
var world_view: TextureRect
var solar: SolarSystem
var rig: OrbitCameraRig
var hud: HUD
var tags: TagLayer
var map2d: Map2D
var enc: EncounterView
var pork: PorkchopPlot
var planner: PlannerPanel
var tier2_panel: Tier2Panel
var tractor_panel: TractorPanel
var threat_panel: ThreatPanel
var boot: BootScreen
var time_bar: TimeBar

var _green := true
## Index into PERSIST_TAUS. Public so the shot harness can photograph the ends
## of the ladder without an InputMap round-trip it does not need.
var persist_idx := 0
var _focus_idx := 0
var _focus_targets: Array = []       # [name, getter, distance]


func _ready() -> void:
	set_anchors_preset(Control.PRESET_FULL_RECT)

	var container := SubViewportContainer.new()
	container.set_anchors_preset(Control.PRESET_FULL_RECT)
	container.stretch = true
	crt_mat = ShaderMaterial.new()
	crt_mat.shader = CRT_SHADER
	crt_mat.set_shader_parameter("phosphor", PHOSPHOR_GREEN)
	container.material = crt_mat
	add_child(container)

	viewport = SubViewport.new()
	viewport.handle_input_locally = true
	# Nothing 3D renders here any more — the world has its own viewport below —
	# so the empty 3D pass is switched off. The camera rig (a Node3D) still lives
	# here for input ordering; it needs a tree, not a renderer.
	viewport.disable_3d = true
	container.add_child(viewport)

	# --- The 3D world, twice removed --------------------------------------
	# The world renders into `world_vp`, which feeds `persist_vp`, which the main
	# viewport shows as a texture under the HUD. Two things the shared viewport
	# could not give:
	#   - 4x MSAA on the line work. Every body and orbit here is a one-pixel line,
	#     and un-antialiased they crawl as the camera moves. The HUD text stays
	#     crisp because it is not in this viewport.
	#   - phosphor persistence: a moving body leaves a trail that decays in
	#     PERSIST_TAU. Text and tags must not smear, so they are drawn above it.
	# `world_vp` is a child of `persist_vp` so it renders first each frame; the
	# persistence shader then reads this frame's world, not last frame's.
	persist_vp = SubViewport.new()
	persist_vp.name = "Persist"
	persist_vp.disable_3d = true
	# Never cleared: the image IS the accumulation. HDR (16-bit float) so the decay
	# reaches black — in 8-bit a value times 0.9 rounds back to itself below ~4/255
	# and every trail leaves a permanent faint ghost.
	persist_vp.render_target_clear_mode = SubViewport.CLEAR_MODE_NEVER
	persist_vp.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	persist_vp.use_hdr_2d = true
	viewport.add_child(persist_vp)

	world_vp = SubViewport.new()
	world_vp.name = "World"
	world_vp.own_world_3d = true
	world_vp.msaa_3d = Viewport.MSAA_4X
	world_vp.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	persist_vp.add_child(world_vp)

	solar = SolarSystem.new()
	solar.name = "SolarSystem"
	world_vp.add_child(solar)

	var cam := Camera3D.new()
	cam.name = "Camera"
	world_vp.add_child(cam)
	cam.current = true

	_persist_rect = ColorRect.new()
	_persist_rect.name = "PersistBlend"
	persist_mat = ShaderMaterial.new()
	persist_mat.shader = PERSIST_SHADER
	persist_mat.set_shader_parameter("world_tex", world_vp.get_texture())
	_persist_rect.material = persist_mat
	persist_vp.add_child(_persist_rect)

	world_view = TextureRect.new()
	world_view.name = "WorldView"
	world_view.texture = persist_vp.get_texture()
	world_view.mouse_filter = Control.MOUSE_FILTER_IGNORE
	viewport.add_child(world_view)

	# The rig stays in the main viewport (input ordering — see orbit_camera.gd)
	# and drives the world's camera by global transform.
	rig = OrbitCameraRig.new()
	rig.name = "CameraRig"
	rig.attach_camera(cam)
	viewport.add_child(rig)

	map2d = Map2D.new()
	map2d.name = "Map2D"
	map2d.visible = false
	viewport.add_child(map2d)

	enc = EncounterView.new()
	enc.name = "Encounter"
	enc.visible = false
	viewport.add_child(enc)

	pork = PorkchopPlot.new()
	pork.name = "Porkchop"
	pork.visible = false
	viewport.add_child(pork)

	tags = TagLayer.new()
	tags.name = "Tags"
	tags.camera_rig = rig
	tags.visible = false
	viewport.add_child(tags)

	hud = HUD.new()
	hud.name = "HUD"
	hud.camera_rig = rig
	hud.visible = false
	viewport.add_child(hud)

	time_bar = TimeBar.new()
	time_bar.name = "TimeBar"
	time_bar.visible = false
	viewport.add_child(time_bar)

	planner = PlannerPanel.new()
	planner.name = "Planner"
	viewport.add_child(planner)

	tier2_panel = Tier2Panel.new()
	tier2_panel.name = "Tier2Panel"
	viewport.add_child(tier2_panel)

	tractor_panel = TractorPanel.new()
	tractor_panel.name = "TractorPanel"
	viewport.add_child(tractor_panel)

	threat_panel = ThreatPanel.new()
	threat_panel.name = "ThreatPanel"
	viewport.add_child(threat_panel)

	boot = BootScreen.new()
	boot.name = "Boot"
	boot.finished.connect(func() -> void:
		hud.visible = true
		time_bar.visible = true
		tags.visible = not (map2d.visible or enc.visible or pork.visible))
	viewport.add_child(boot)

	# Controls parented directly to a SubViewport don't inherit its size via
	# anchors — size them explicitly and track viewport resizes.
	viewport.size_changed.connect(_sync_overlay_sizes)
	_sync_overlay_sizes.call_deferred()

	# The Sun is always focusable (it is the frame origin, not a lookup). Every
	# other target is a real ephemeris body, so the list is built only when the
	# field is up; the threat/comet/interceptor targets return in 3C-2b with the
	# bodies themselves.
	_build_focus_targets()
	if not Sim.bodies_online:
		# Built once more when the threaded kernel read lands: at scene load the
		# field may not be up yet, and a focus list frozen at that moment offers the
		# Sun and nothing else for the rest of the session.
		Sim.field_online.connect(_build_focus_targets)
	_apply_focus()


func _process(delta: float) -> void:
	# Frame-rate independent decay: keep^(frames per second) is a fixed fraction
	# per second whatever the refresh rate. tau 0 is the off rung and must be
	# handled as a case, not by the formula — exp(-delta/0) is a division by zero.
	var tau: float = PERSIST_TAUS[persist_idx]
	persist_mat.set_shader_parameter("keep", 0.0 if tau <= 0.0 else exp(-delta / tau))


func _input(event: InputEvent) -> void:
	var is_press: bool = (event is InputEventKey or event is InputEventAction) \
		and event.is_pressed() and not event.is_echo()
	if not is_press:
		return
	if is_instance_valid(boot) and boot.is_inside_tree():
		boot.dismiss()
		get_viewport().set_input_as_handled()
		return
	if event.is_action_pressed("sim_pause"):
		Sim.paused = not Sim.paused
	elif event.is_action_pressed("warp_up"):
		Sim.warp_idx = mini(Sim.warp_idx + 1, Sim.WARP_STEPS.size() - 1)
	elif event.is_action_pressed("warp_down"):
		Sim.warp_idx = maxi(Sim.warp_idx - 1, 0)
	elif event.is_action_pressed("time_reverse"):
		Sim.reverse()
	elif event.is_action_pressed("phosphor_toggle"):
		_green = not _green
		crt_mat.set_shader_parameter("phosphor",
			PHOSPHOR_GREEN if _green else PHOSPHOR_AMBER)
	elif event.is_action_pressed("persist_cycle"):
		persist_idx = (persist_idx + 1) % PERSIST_TAUS.size()
		# No clear needed on the way to OFF: the persistence pass is a full-rect
		# ColorRect and `max(world, previous * 0)` is `world`, so every pixel is
		# overwritten on the next frame anyway.
		var tau: float = PERSIST_TAUS[persist_idx]
		Sim.event_logged.emit("PHOSPHOR PERSISTENCE %s" %
			("OFF" if tau <= 0.0 else "%.2f S" % tau))
	elif event.is_action_pressed("view_3d"):
		_show_view(null)
		tags.visible = true
		hud.view_name = "TACTICAL 3D"
	elif event.is_action_pressed("view_map"):
		_show_view(map2d)
		hud.view_name = "HELIO PLOT 2D"
	elif event.is_action_pressed("view_encounter"):
		# Gated on `encounter_online`, NOT `mission_online`. They light together now
		# (3C-2c put the b-plane on the core's `EncounterFrame`), but they are not
		# the same claim: this one says the encounter geometry exists, and it does
		# not until the scenario build lands ~10 s in.
		if not Sim.encounter_online:
			Sim.event_logged.emit("ENCOUNTER VIEW OFFLINE - AWAITING THREAT SOLUTION")
		else:
			_show_view(enc)
			hud.view_name = "ENCOUNTER B-PLANE"
	elif event.is_action_pressed("view_porkchop"):
		# Gated on `mission_online`, not on `pork_online`: the grid does not exist
		# until this view asks for it, so gating on the grid would make the key a
		# no-op that could never open the thing that builds it.
		if not Sim.mission_online:
			Sim.event_logged.emit("LAUNCH-WINDOW MAP OFFLINE - AWAITING THREAT SOLUTION")
		else:
			_show_view(pork)
			hud.view_name = "LAUNCH WINDOWS"
			# Opening the map is what pays for the grid — on demand, off the build
			# path, exactly like the Tier-2 menu. A no-op once it is solved.
			Sim.request_porkchop()
	elif enc.visible and event.is_action_pressed("encounter_ca_jump"):
		_jump_to_closest_approach()
	elif enc.visible and event.is_action_pressed("encounter_keyholes"):
		enc.toggle_keyholes()
	elif enc.visible and event.is_action_pressed("encounter_uncertainty"):
		enc.toggle_uncertainty()
	# The sigma knob, guarded on the b-plane view like the two above. [Z]/[X] rather
	# than the arrows because the arrows are already spoken for three times over
	# (planner lead, launch-window cursor, tractor knobs) and a fourth claimant
	# resolved by guard order is how a keypress starts meaning different things
	# depending on which panel happens to be open.
	elif enc.visible and event.is_action_pressed("encounter_sigma_down"):
		Sim.tier3_sigma_step(-Sim.TIER3_SIGMA_STEP)
	elif enc.visible and event.is_action_pressed("encounter_sigma_up"):
		Sim.tier3_sigma_step(Sim.TIER3_SIGMA_STEP)
	# The porkchop's cursor keys are checked BEFORE the planner's, and both are
	# guarded on their own view being up. LEFT/RIGHT are shared with the planner's
	# lead adjust, so whichever guard matches first in this chain wins — when the
	# launch-window map is open, the arrows drive it.
	elif pork.visible and event.is_action_pressed("pork_cursor_up"):
		Sim.move_pork_cursor(-1, 0)
	elif pork.visible and event.is_action_pressed("pork_cursor_down"):
		Sim.move_pork_cursor(1, 0)
	elif pork.visible and event.is_action_pressed("pork_cursor_left"):
		Sim.move_pork_cursor(0, -1)
	elif pork.visible and event.is_action_pressed("pork_cursor_right"):
		Sim.move_pork_cursor(0, 1)
	elif pork.visible and event.is_action_pressed("pork_vehicle"):
		Sim.cycle_pork_vehicle()
	elif pork.visible and event.is_action_pressed("pork_metric"):
		Sim.cycle_pork_metric()
	elif pork.visible and event.is_action_pressed("pork_verify"):
		Sim.request_cell_verify()
	# [M] is shared with the planner toggle, and resolved the same way the arrows
	# are: this guard is earlier in the chain, so while the launch-window map is up
	# [M] solves the window's required mass rather than opening the planner.
	elif pork.visible and event.is_action_pressed("pork_required_mass"):
		Sim.request_required_mass()
	# The tractor bench claims the arrows and [E] while it is up, on the same
	# first-match-wins principle the porkchop's cursor keys already use. It sits
	# BEFORE the planner's lead/dv adjust for the same reason the map does: the
	# panel that is open is the one the arrows belong to.
	#
	# **One new input action for six knobs.** UP/DOWN picks a row and LEFT/RIGHT
	# adjusts it, so a seventh knob is one row in `Sim.TRACTOR_KNOBS` and no edit
	# here at all — where the planner's key-pair-per-parameter would have cost
	# twelve actions and twelve branches.
	elif tractor_panel.visible and event.is_action_pressed("pork_cursor_up"):
		Sim.move_tractor_cursor(-1)
	elif tractor_panel.visible and event.is_action_pressed("pork_cursor_down"):
		Sim.move_tractor_cursor(1)
	elif tractor_panel.visible and event.is_action_pressed("pork_cursor_left"):
		Sim.adjust_tractor(-1)
	elif tractor_panel.visible and event.is_action_pressed("pork_cursor_right"):
		Sim.adjust_tractor(1)
	elif tractor_panel.visible and event.is_action_pressed("pork_verify"):
		# [E] means the same thing in both views — "stop estimating and go measure
		# it in the full field" — so it is deliberately the same key.
		Sim.request_tow_probe()
	# The threat-orbit designer claims the arrows on the same terms as the bench,
	# and it reuses two keys rather than minting new ones. [ENTER] is `plan_commit`
	# ("apply what is dialled") and [E] is `pork_verify`, which by now means "stop
	# estimating and go measure it" in three views — solving this orbit's required
	# Δv is exactly that. A fifth knob is one row in `Sim.THREAT_KNOBS`.
	elif threat_panel.visible and event.is_action_pressed("pork_cursor_up"):
		Sim.move_threat_cursor(-1)
	elif threat_panel.visible and event.is_action_pressed("pork_cursor_down"):
		Sim.move_threat_cursor(1)
	elif threat_panel.visible and event.is_action_pressed("pork_cursor_left"):
		Sim.adjust_threat(-1)
	elif threat_panel.visible and event.is_action_pressed("pork_cursor_right"):
		Sim.adjust_threat(1)
	elif threat_panel.visible and event.is_action_pressed("plan_commit"):
		Sim.request_threat_rebuild()
	elif threat_panel.visible and event.is_action_pressed("pork_verify"):
		Sim.request_anchor_solve()
	elif event.is_action_pressed("focus_next"):
		_focus_idx = (_focus_idx + 1) % _focus_targets.size()
		_apply_focus()
	elif event.is_action_pressed("milestone_jump"):
		Sim.jump_next_milestone()
	elif event.is_action_pressed("time_reset"):
		Sim.jump(0.0)
	elif event.is_action_pressed("plan_toggle"):
		if not Sim.mission_online:
			Sim.event_logged.emit("MISSION PLANNER OFFLINE - REBUILDING ON REAL CORE")
		else:
			planner.visible = not planner.visible
			Sim.planner_open = planner.visible
			# Symmetric to [K]/[N]: same origin, same arrow keys, and here the
			# other two would keep the arrows because their guards sit earlier in
			# the chain.
			if planner.visible:
				_close_other_bottom_panels(planner)
	elif planner.visible and event.is_action_pressed("plan_lead_up"):
		Sim.adjust_lead(10.0)
	elif planner.visible and event.is_action_pressed("plan_lead_down"):
		Sim.adjust_lead(-10.0)
	elif planner.visible and event.is_action_pressed("plan_dv_up"):
		Sim.adjust_dv(1.25)
	elif planner.visible and event.is_action_pressed("plan_dv_down"):
		Sim.adjust_dv(0.8)
	elif planner.visible and event.is_action_pressed("plan_dir"):
		Sim.toggle_burn_dir()
	elif planner.visible and event.is_action_pressed("plan_commit"):
		Sim.try_commit()
	elif event.is_action_pressed("tier2_toggle"):
		# Gated on `mission_online` for the same reason the planner is: the five
		# shifts ride the scenario build, so there is nothing to show until the
		# threat solution lands.
		if not Sim.mission_online:
			Sim.event_logged.emit("FORCE-MODEL MENU OFFLINE - AWAITING THREAT SOLUTION")
		else:
			tier2_panel.visible = not tier2_panel.visible
			Sim.tier2_panel_open = tier2_panel.visible
			# Opening the menu is what pays for the ~2 min measurement — on demand,
			# off the build path. A no-op once the shifts are cached.
			if tier2_panel.visible:
				Sim.request_tier2_preview()
	elif tier2_panel.visible and event.is_action_pressed("tier2_term_gr"):
		Sim.toggle_tier2("relativity")
	elif tier2_panel.visible and event.is_action_pressed("tier2_term_yark"):
		Sim.toggle_tier2("yarkovsky")
	elif tier2_panel.visible and event.is_action_pressed("tier2_term_belt"):
		Sim.toggle_tier2("belt")
	elif tier2_panel.visible and event.is_action_pressed("tier2_term_srp"):
		Sim.toggle_tier2("srp")
	elif tier2_panel.visible and event.is_action_pressed("tier2_term_j2"):
		Sim.toggle_tier2("j2")
	elif event.is_action_pressed("tractor_toggle"):
		# Gated on `mission_online` like the planner and the force menu: every
		# number in the bench is measured against the threat solution, and an
		# empty one would show a tractor tugging nothing.
		#
		# Nothing is paid for on open, unlike the Tier-2 menu and the launch-window
		# map. The whole panel is arithmetic until [E] is pressed, which is the
		# property that lets a user sweep six knobs freely.
		if not Sim.mission_online:
			Sim.event_logged.emit("GRAVITY TRACTOR OFFLINE - AWAITING THREAT SOLUTION")
		else:
			tractor_panel.visible = not tractor_panel.visible
			Sim.tractor_panel_open = tractor_panel.visible
			if tractor_panel.visible:
				_close_other_bottom_panels(tractor_panel)
	elif event.is_action_pressed("threat_toggle"):
		# Gated like the bench and the planner. Free to open and free to turn:
		# the preview is closed form, and nothing is spent until [ENTER] or [E].
		if not Sim.mission_online:
			Sim.event_logged.emit("THREAT DESIGNER OFFLINE - AWAITING THREAT SOLUTION")
		else:
			threat_panel.visible = not threat_panel.visible
			Sim.threat_panel_open = threat_panel.visible
			if threat_panel.visible:
				_close_other_bottom_panels(threat_panel)


## Close every bottom-centre overlay except `keep`.
##
## **Not tidiness.** All three draw at the same origin, and all three claim the
## arrow keys through guards in one `elif` chain — so two open at once means the
## later one's keys silently stop responding with nothing on screen to say why.
## It was a two-way check between the planner and the bench; the threat designer
## made it a third case, which is the point at which writing it out by hand at
## each toggle starts dropping a pair. The porkchop avoids all of this by being a
## full-frame view.
func _close_other_bottom_panels(keep: Control) -> void:
	if planner != keep and planner.visible:
		planner.visible = false
		Sim.planner_open = false
	if tractor_panel != keep and tractor_panel.visible:
		tractor_panel.visible = false
		Sim.tractor_panel_open = false
	if threat_panel != keep and threat_panel.visible:
		threat_panel.visible = false
		Sim.threat_panel_open = false


## Show exactly one of the full-frame overlay views (or `null` for the 3D world),
## hiding the others.
##
## Centralized because the views are mutually exclusive and the list keeps growing:
## with each switch setting every sibling by hand, adding the fourth view meant
## editing three unrelated branches, and forgetting one leaves two overlays stacked
## — the second silently painting over the first.
func _show_view(which: Control) -> void:
	for v: Control in [map2d, enc, pork]:
		v.visible = (v == which)
	var world_shown: bool = which == null
	tags.visible = world_shown
	# The 2D views paint an opaque frame, so the 3D world under them is invisible
	# work: stop rendering it (and accumulating it) while one is up. On return the
	# accumulation is cleared once, so the world does not fade in from a picture
	# that is minutes stale.
	world_view.visible = world_shown
	var mode := SubViewport.UPDATE_ALWAYS if world_shown else SubViewport.UPDATE_DISABLED
	world_vp.render_target_update_mode = mode
	persist_vp.render_target_update_mode = mode
	# Nor does the hidden scene need its ~30 body lookups a frame; it catches up on
	# the first frame back, since every position is a function of the clock.
	solar.set_process(world_shown)
	if world_shown:
		persist_vp.render_target_clear_mode = SubViewport.CLEAR_MODE_ONCE


## Park the clock on the live asteroid at its closest approach — the one moment
## the encounter view's radar contact is on screen.
##
## Reaching that by hand was a three-step ritual: pause, then scrub into a ±1.5 d
## window inside a twelve-year campaign, because any warp step overshoots closest
## approach by ~0.53 d and the marker is correctly absent everywhere else. The
## view's most informative instant was effectively unreachable, so this makes it
## one key.
##
## **Pausing is not a convenience here, it is the point.** Jumping while the clock
## runs walks straight back out of the window at the next warp step, which looks
## exactly like the marker failing to appear.
func _jump_to_closest_approach() -> void:
	var day := Sim.encounter_ca_day()
	if is_nan(day):
		Sim.event_logged.emit("NO ENCOUNTER TRACK - CANNOT SLEW TO CLOSEST APPROACH")
		return
	Sim.paused = true
	Sim.jump(day)
	Sim.event_logged.emit("CLOCK HOLD AT CLOSEST APPROACH - CA %+.2f D" % (day - Sim.T_IMPACT))


## The camera's focus ring: the Sun, then every ephemeris body.
##
## Rebuildable rather than built inline, because the field it reads can arrive
## after this scene. `_focus_idx` is clamped rather than reset — the operator may
## have moved the ring before the planets landed, and snapping their camera back
## to the Sun at that moment would look like the app losing its place.
func _build_focus_targets() -> void:
	_focus_targets = [["SUN", func() -> Vector3: return Vector3.ZERO, 32.0]]
	if Sim.bodies_online:
		for el in Sim.planets:
			var body: Dictionary = el
			var dist: float = 3.0 if body.name == "EARTH" else maxf(2.0, float(body.vis_r) * 24.0)
			_focus_targets.append([body.name,
				func() -> Vector3: return Sim.pos3d(body, Sim.t), dist])
	_focus_idx = clampi(_focus_idx, 0, _focus_targets.size() - 1)


func _apply_focus() -> void:
	var f: Array = _focus_targets[_focus_idx]
	rig.set_focus(f[0], f[1], f[2])


func _sync_overlay_sizes() -> void:
	var vs := Vector2(viewport.size)
	# The world and persistence viewports match the main one pixel for pixel, so
	# `Camera3D.unproject_position` (the tag layer) lands where the HUD expects.
	world_vp.size = viewport.size
	persist_vp.size = viewport.size
	for c: Control in [world_view, _persist_rect, map2d, enc, pork, tags, hud, planner,
			tier2_panel, tractor_panel, threat_panel, boot]:
		if is_instance_valid(c):
			c.position = Vector2.ZERO
			c.size = vs
	# The scrub bar is a bottom strip, not a full-rect overlay.
	if is_instance_valid(time_bar):
		time_bar.layout(vs)
