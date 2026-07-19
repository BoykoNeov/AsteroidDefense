extends Control
## Root assembler: SubViewport (3D world + HUD layers) shown through the
## CRT phosphor shader. Owns global hotkeys, phosphor theme toggle, and
## the 3D/2D view switch. Native-resolution rendering throughout — retro
## comes from the styling layer, never from downscaling.

const CRT_SHADER := preload("res://shaders/crt.gdshader")

const PHOSPHOR_GREEN := Color(0.25, 1.0, 0.45)
const PHOSPHOR_AMBER := Color(1.0, 0.62, 0.13)

var crt_mat: ShaderMaterial
var viewport: SubViewport
var solar: SolarSystem
var rig: OrbitCameraRig
var hud: HUD
var tags: TagLayer
var map2d: Map2D
var enc: EncounterView
var planner: PlannerPanel
var boot: BootScreen
var time_bar: TimeBar

var _green := true
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
	viewport.own_world_3d = true
	viewport.handle_input_locally = true
	container.add_child(viewport)

	solar = SolarSystem.new()
	solar.name = "SolarSystem"
	viewport.add_child(solar)

	rig = OrbitCameraRig.new()
	rig.name = "CameraRig"
	viewport.add_child(rig)
	rig.camera.current = true

	map2d = Map2D.new()
	map2d.name = "Map2D"
	map2d.visible = false
	viewport.add_child(map2d)

	enc = EncounterView.new()
	enc.name = "Encounter"
	enc.visible = false
	viewport.add_child(enc)

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

	boot = BootScreen.new()
	boot.name = "Boot"
	boot.finished.connect(func() -> void:
		hud.visible = true
		time_bar.visible = true
		tags.visible = not (map2d.visible or enc.visible))
	viewport.add_child(boot)

	# Controls parented directly to a SubViewport don't inherit its size via
	# anchors — size them explicitly and track viewport resizes.
	viewport.size_changed.connect(_sync_overlay_sizes)
	_sync_overlay_sizes.call_deferred()

	# The Sun is always focusable (it is the frame origin, not a lookup). Every
	# other target is a real ephemeris body, so the list is built only when the
	# field is up; the threat/comet/interceptor targets return in 3C-2b with the
	# bodies themselves.
	_focus_targets = [["SUN", func() -> Vector3: return Vector3.ZERO, 32.0]]
	if Sim.bodies_online:
		for el in Sim.planets:
			var body: Dictionary = el
			var dist: float = 3.0 if body.name == "EARTH" else maxf(2.0, float(body.vis_r) * 24.0)
			_focus_targets.append([body.name,
				func() -> Vector3: return Sim.pos3d(body, Sim.t), dist])
	_apply_focus()


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
	elif event.is_action_pressed("view_3d"):
		map2d.visible = false
		enc.visible = false
		tags.visible = true
		hud.view_name = "TACTICAL 3D"
	elif event.is_action_pressed("view_map"):
		map2d.visible = true
		enc.visible = false
		tags.visible = false
		hud.view_name = "HELIO PLOT 2D"
	elif event.is_action_pressed("view_encounter"):
		# Gated on `encounter_online`, NOT `mission_online`. They light together now
		# (3C-2c put the b-plane on the core's `EncounterFrame`), but they are not
		# the same claim: this one says the encounter geometry exists, and it does
		# not until the scenario build lands ~10 s in.
		if not Sim.encounter_online:
			Sim.event_logged.emit("ENCOUNTER VIEW OFFLINE - AWAITING THREAT SOLUTION")
		else:
			enc.visible = true
			map2d.visible = false
			tags.visible = false
			hud.view_name = "ENCOUNTER B-PLANE"
	elif enc.visible and event.is_action_pressed("encounter_ca_jump"):
		_jump_to_closest_approach()
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


func _apply_focus() -> void:
	var f: Array = _focus_targets[_focus_idx]
	rig.set_focus(f[0], f[1], f[2])


func _sync_overlay_sizes() -> void:
	var vs := Vector2(viewport.size)
	for c: Control in [map2d, enc, tags, hud, planner, boot]:
		if is_instance_valid(c):
			c.position = Vector2.ZERO
			c.size = vs
	# The scrub bar is a bottom strip, not a full-rect overlay.
	if is_instance_valid(time_bar):
		time_bar.layout(vs)
