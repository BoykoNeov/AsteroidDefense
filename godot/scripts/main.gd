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
var boot: BootScreen

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

	boot = BootScreen.new()
	boot.name = "Boot"
	boot.finished.connect(func() -> void:
		hud.visible = true
		tags.visible = not (map2d.visible or enc.visible))
	viewport.add_child(boot)

	# Controls parented directly to a SubViewport don't inherit its size via
	# anchors — size them explicitly and track viewport resizes.
	viewport.size_changed.connect(_sync_overlay_sizes)
	_sync_overlay_sizes.call_deferred()

	_focus_targets = [
		["SUN", func() -> Vector3: return Vector3.ZERO, 32.0],
		["EARTH", func() -> Vector3: return Sim.pos3d(Sim.earth_el, Sim.t), 3.0],
		["2031-XK", func() -> Vector3: return Sim.pos3d(
			Sim.ast_defl_el if Sim.t >= Sim.T_INTERCEPT else Sim.ast_el, Sim.t), 1.6],
		["C/2029K1", func() -> Vector3: return Sim.pos3d(Sim.comet_el, Sim.t), 2.4],
		["ATLAS-1", func() -> Vector3:
			return Sim.interceptor_pos(Sim.t) if Sim.interceptor_phase(Sim.t) == "CRUISE" \
				else Sim.pos3d(Sim.earth_el, Sim.t), 1.6],
	]
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
		enc.visible = true
		map2d.visible = false
		tags.visible = false
		hud.view_name = "ENCOUNTER B-PLANE"
	elif event.is_action_pressed("focus_next"):
		_focus_idx = (_focus_idx + 1) % _focus_targets.size()
		_apply_focus()
	elif event.is_action_pressed("milestone_jump"):
		Sim.jump_next_milestone()
	elif event.is_action_pressed("time_reset"):
		Sim.jump(0.0)


func _apply_focus() -> void:
	var f: Array = _focus_targets[_focus_idx]
	rig.set_focus(f[0], f[1], f[2])


func _sync_overlay_sizes() -> void:
	var vs := Vector2(viewport.size)
	for c: Control in [map2d, enc, tags, hud, boot]:
		if is_instance_valid(c):
			c.position = Vector2.ZERO
			c.size = vs
