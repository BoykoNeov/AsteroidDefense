class_name OrbitCameraRig
extends Node3D
## Orbit camera: drag to rotate, wheel to zoom, smoothly tracks the current
## focus target. Focus cycling is driven from main.gd.
##
## The rig and its camera live in **different viewports**. The rig sits in the
## main (HUD) viewport so it keeps receiving `_unhandled_input` after the 2D
## overlays have had their turn — the encounter view swallows the wheel before it
## gets here, and that ordering is what lets one wheel zoom two different things.
## The camera is a child of the 3D world viewport, because a Camera3D can only
## render the world it is in, and the world moved into its own viewport for
## anti-aliasing and phosphor persistence (see main.gd). So the camera is driven
## by global transform, not by being this node's child: `attach_camera` hands it
## over, and `_process` places it every frame.

var camera: Camera3D
var yaw := 0.6
var pitch := 0.55
var distance := 32.0
var focus_getter: Callable          # -> Vector3 (scene units)
var focus_name := "SUN"

var _dragging := false


func _ready() -> void:
	focus_getter = func() -> Vector3: return Vector3.ZERO


## Take ownership of a camera that already lives in the world viewport.
func attach_camera(cam: Camera3D) -> void:
	camera = cam
	camera.near = 0.005
	camera.far = 5000.0
	camera.fov = 55.0


func _process(delta: float) -> void:
	if camera == null:
		return
	var target: Vector3 = focus_getter.call()
	position = position.lerp(target, minf(1.0, delta * 6.0))
	pitch = clampf(pitch, -1.45, 1.45)
	distance = clampf(distance, 0.4, 900.0)
	var dir := Vector3(
		cos(pitch) * sin(yaw), sin(pitch), cos(pitch) * cos(yaw))
	# Global placement: the camera is not a child of this node.
	camera.look_at_from_position(position + dir * distance, position, Vector3.UP)


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_LEFT or mb.button_index == MOUSE_BUTTON_RIGHT:
			_dragging = mb.pressed
		elif mb.button_index == MOUSE_BUTTON_WHEEL_UP and mb.pressed:
			distance *= 0.88
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			distance *= 1.14
	elif event is InputEventMouseMotion and _dragging:
		var mm := event as InputEventMouseMotion
		yaw -= mm.relative.x * 0.006
		pitch += mm.relative.y * 0.006


func set_focus(name_: String, getter: Callable, dist: float = -1.0) -> void:
	focus_name = name_
	focus_getter = getter
	if dist > 0.0:
		distance = dist
