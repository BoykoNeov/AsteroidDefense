class_name OrbitCameraRig
extends Node3D
## Orbit camera: drag to rotate, wheel to zoom, smoothly tracks the current
## focus target. Focus cycling is driven from main.gd.

var camera: Camera3D
var yaw := 0.6
var pitch := 0.55
var distance := 32.0
var focus_getter: Callable          # -> Vector3 (scene units)
var focus_name := "SUN"

var _dragging := false


func _ready() -> void:
	camera = Camera3D.new()
	camera.near = 0.005
	camera.far = 3000.0
	camera.fov = 55.0
	add_child(camera)
	focus_getter = func() -> Vector3: return Vector3.ZERO


func _process(delta: float) -> void:
	var target: Vector3 = focus_getter.call()
	position = position.lerp(target, minf(1.0, delta * 6.0))
	pitch = clampf(pitch, -1.45, 1.45)
	distance = clampf(distance, 0.4, 400.0)
	var dir := Vector3(
		cos(pitch) * sin(yaw), sin(pitch), cos(pitch) * cos(yaw))
	camera.position = dir * distance
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
