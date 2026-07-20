class_name SolarSystem
extends Node3D
## Builds and animates the 3D vector-display solar system: starfield,
## ecliptic grid, sun, wireframe planets, threat asteroid (nominal +
## deflected tracks), comet with anti-sunward tail, interceptor and its
## transfer arc. All geometry is emissive line work feeding the env glow.

const LINE_SHADER := preload("res://shaders/glow_line.gdshader")
const STAR_SHADER := preload("res://shaders/starfield.gdshader")

var _line_mat: ShaderMaterial

var sun_node: Node3D
var body_nodes := {}                  # name -> MeshInstance3D (wireframe)
var moon_node: MeshInstance3D
var _belt: MeshInstance3D
## The sixteen real asteroids, index-aligned with `Sim.asteroids`. Kept as an Array
## rather than a name dict because the pairing with that list is what the per-frame
## lookup uses, and a name-keyed miss would be a silent no-draw.
var _asteroid_nodes: Array[MeshInstance3D] = []
var ast_nominal: MeshInstance3D
var ast_deflected: MeshInstance3D
var comet_node: MeshInstance3D
var comet_tail: GPUParticles3D
var interceptor: MeshInstance3D
var intercept_flash: MeshInstance3D
var _nom_orbit_line: MeshInstance3D
var _defl_orbit_line: MeshInstance3D
var _comet_orbit_line: MeshInstance3D
var _intercept_path_line: MeshInstance3D


func _ready() -> void:
	_line_mat = ShaderMaterial.new()
	_line_mat.shader = LINE_SHADER

	_build_environment()
	_build_starfield()
	_build_grid()
	_build_sun()
	if Sim.bodies_online:
		_build_planets()
	_build_belt()
	# The threat cannot be built here: the scenario is ~10 s of integration on a
	# worker thread and does not exist yet at scene load. `mission_ready` fires when
	# it lands. The comet and interceptor stay dormant until they come from the core
	# (3C-2c); their nodes are never created, so nothing half-drawn can leak in.
	Sim.mission_ready.connect(_on_mission_ready)
	if Sim.mission_online:
		_on_mission_ready()                # already built (rebuilt scene, autoload up)


## The threat landed: build its nodes and start tracking plan changes.
func _on_mission_ready() -> void:
	_build_threat()
	_build_asteroids()
	if Sim.comet_online:
		_build_comet()
	if Sim.interceptor_online:
		_build_interceptor()
	# The deflected orbit is re-sampled from the core on every solve, so it must
	# follow `plan_changed` — the line drawn at build time is only the first plan.
	Sim.plan_changed.connect(_rebuild_plan_visuals)
	_rebuild_plan_visuals()


func _process(_delta: float) -> void:
	var t: float = Sim.t
	if Sim.bodies_online:
		for el in Sim.planets:
			body_nodes[el.name].position = Sim.pos3d(el, t)
		moon_node.position = Sim.moon_local(t)
		# The sixteen real ones, in the same loop and by the same lookup as the
		# planets — `Sim.asteroids` is empty unless the kernel actually mounted, so
		# there is no unmounted state in which this draws anything.
		for i in _asteroid_nodes.size():
			_asteroid_nodes[i].position = Sim.pos3d(Sim.asteroids[i], t)
	# Rigid rotation at the belt's mean motion (~2.7 AU, T ~ 4.4 yr) —
	# Kepler shear across the annulus is invisible at display speeds. This is the
	# *scenery* belt, not the sixteen bodies above.
	_belt.rotation.y = TAU * t / (4.4 * 365.25)

	if not Sim.mission_online:
		return

	# The threat exists only over its propagated span (~12 yr of a ~300 yr
	# scrubbable clock). Outside it a lookup returns ZERO, which here is the SUN —
	# so this is a hide, not a cosmetic fade: an ungated marker parks on the Sun.
	var active: bool = Sim.threat_active(t)
	ast_nominal.visible = active
	_nom_orbit_line.visible = active
	if active:
		ast_nominal.position = Sim.pos3d(Sim.ast_el, t)
		ast_nominal.rotate_y(0.01)
		ast_nominal.rotate_x(0.004)

	# The deflected body needs a *solved* plan, not just a committed one: its track
	# is the core's post-impulse arc and does not exist until the solve lands.
	var burned: bool = Sim.burned() and Sim.has_plan()
	ast_deflected.visible = burned and active
	# Deflected orbit: bright once real, dim dashed preview while planning.
	var preview: bool = not burned and Sim.has_plan() and (Sim.planner_open or Sim.committed)
	_defl_orbit_line.visible = burned or preview
	_defl_orbit_line.set_instance_shader_parameter("energy", 1.2 if burned else 0.45)
	# Nominal track fades to a dim ghost once the real object is deflected.
	ast_nominal.set_instance_shader_parameter("energy", 0.5 if burned else 2.2)
	if ast_deflected.visible:
		ast_deflected.position = Sim.pos3d(Sim.ast_defl_el, t)
		ast_deflected.rotate_y(0.01)
		ast_deflected.rotate_x(0.004)

	# Hidden outside its propagated span, not drawn at ZERO — which in this
	# heliocentric frame would put the comet on the Sun for most of the clock its
	# one-orbit arc does not cover. The track hides with the body, exactly as the
	# threat's does above: the polyline itself is safe (built once from the whole
	# span, so it never queries the live clock and cannot collapse to the Sun), but
	# an orbit drawn for a body the sim is not tracking still reads as a claim that
	# it is.
	if Sim.comet_online:
		comet_node.visible = Sim.catalog_active(Sim.comet_el, t)
		_comet_orbit_line.visible = comet_node.visible
		if comet_node.visible:
			comet_node.position = Sim.pos3d(Sim.comet_el, t)
			comet_node.rotate_y(0.006)
			_update_comet_tail()

	if not Sim.interceptor_online:
		return

	var phase: String = Sim.interceptor_phase(t)
	interceptor.visible = phase == "CRUISE"
	_intercept_path_line.visible = phase == "CRUISE" or phase == "EXPENDED" \
		or (Sim.planner_open and not burned)
	_intercept_path_line.set_instance_shader_parameter(
		"energy", 0.8 if phase == "CRUISE" or phase == "EXPENDED" else 0.35)
	if phase == "CRUISE":
		interceptor.position = Sim.interceptor_pos(t)
	# Brief expanding flash at the intercept point.
	var since_hit: float = t - Sim.T_INTERCEPT
	intercept_flash.visible = since_hit >= 0.0 and since_hit < 25.0
	if intercept_flash.visible:
		var s: float = 1.0 + since_hit * 0.35
		intercept_flash.scale = Vector3.ONE * s
		intercept_flash.set_instance_shader_parameter(
			"energy", maxf(0.0, 3.0 * (1.0 - since_hit / 25.0)))


# ------------------------------------------------------------ construction ---

func _build_environment() -> void:
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.004, 0.006, 0.005)
	env.glow_enabled = true
	env.glow_intensity = 0.7
	env.glow_bloom = 0.05
	env.glow_hdr_threshold = 0.9
	env.glow_blend_mode = Environment.GLOW_BLEND_MODE_ADDITIVE
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)


func _build_starfield() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = 20310714
	var verts := PackedVector3Array()
	var cols := PackedColorArray()
	for _i in 900:
		var v := Vector3(rng.randfn(), rng.randfn(), rng.randfn()).normalized() * 1800.0
		verts.append(v)
		var b := rng.randf_range(0.12, 0.55)
		cols.append(Color(b, b, b))
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_COLOR] = cols
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)
	var mat := ShaderMaterial.new()
	mat.shader = STAR_SHADER
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.material_override = mat
	mi.name = "Starfield"
	add_child(mi)


func _build_grid() -> void:
	# Polar ecliptic grid: range rings every 0.25 AU + 12 spokes, very dim.
	var pts := PackedVector3Array()
	for ring in range(1, 8):
		var r := 0.25 * ring * Sim.AU
		var prev := Vector3(r, 0, 0)
		for k in range(1, 97):
			var a := TAU * k / 96.0
			var p := Vector3(cos(a) * r, 0, sin(a) * r)
			pts.append(prev)
			pts.append(p)
			prev = p
	for s in 12:
		var a := TAU * s / 12.0
		var dir := Vector3(cos(a), 0, sin(a))
		pts.append(dir * 0.25 * Sim.AU)
		pts.append(dir * 1.75 * Sim.AU)
	var mi := _line_mesh(pts, Mesh.PRIMITIVE_LINES)
	mi.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	mi.set_instance_shader_parameter("energy", 0.10)
	mi.name = "EclipticGrid"
	add_child(mi)

	# Sparse outer rings so the far planets sit on a readable scale.
	var opts := PackedVector3Array()
	for r_au in [5.0, 10.0, 20.0, 30.0]:
		var r: float = r_au * Sim.AU
		var prev := Vector3(r, 0, 0)
		for k in range(1, 193):
			var a := TAU * k / 192.0
			var p := Vector3(cos(a) * r, 0, sin(a) * r)
			opts.append(prev)
			opts.append(p)
			prev = p
	var omi := _line_mesh(opts, Mesh.PRIMITIVE_LINES)
	omi.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	omi.set_instance_shader_parameter("energy", 0.05)
	omi.name = "OuterGrid"
	add_child(omi)


func _build_sun() -> void:
	sun_node = Node3D.new()
	sun_node.name = "Sun"
	add_child(sun_node)

	var wire := _wire_sphere(0.5, 12, 8)
	wire.set_instance_shader_parameter("line_color", Color(1.0, 0.95, 0.8))
	wire.set_instance_shader_parameter("energy", 2.6)
	sun_node.add_child(wire)

	# Soft radial glow billboard behind the wireframe.
	var grad := Gradient.new()
	grad.set_color(0, Color(1.0, 0.98, 0.9, 0.55))
	grad.set_color(1, Color(1.0, 0.9, 0.7, 0.0))
	var gtex := GradientTexture2D.new()
	gtex.gradient = grad
	gtex.fill = GradientTexture2D.FILL_RADIAL
	gtex.fill_from = Vector2(0.5, 0.5)
	gtex.fill_to = Vector2(0.5, 0.0)
	gtex.width = 256
	gtex.height = 256
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.blend_mode = BaseMaterial3D.BLEND_MODE_ADD
	mat.billboard_mode = BaseMaterial3D.BILLBOARD_ENABLED
	mat.albedo_texture = gtex
	var quad := QuadMesh.new()
	quad.size = Vector2(3.5, 3.5)
	var mi := MeshInstance3D.new()
	mi.mesh = quad
	mi.material_override = mat
	sun_node.add_child(mi)


func _build_planets() -> void:
	for el in Sim.planets:
		var pts: PackedVector3Array = Sim.orbit_points(el, 192 if el.a < 6.0 else 384)
		var orbit := _line_mesh(pts, Mesh.PRIMITIVE_LINE_STRIP)
		var bright := 0.55 if el.name == "EARTH" else 0.28
		orbit.set_instance_shader_parameter("line_color", Color(1, 1, 1))
		orbit.set_instance_shader_parameter("energy", bright)
		orbit.name = el.name + "Orbit"
		add_child(orbit)

		var body := _wire_sphere(el.vis_r, 10, 6)
		body.set_instance_shader_parameter("line_color", Color(1, 1, 1))
		body.set_instance_shader_parameter("energy", 1.8 if el.name == "EARTH" else 1.2)
		body.name = el.name
		add_child(body)
		body_nodes[el.name] = body

		if el.name == "SATURN":
			_add_saturn_rings(body, el.vis_r)
		elif el.name == "EARTH":
			_add_moon(body)


## Three concentric ring circles tilted to Saturn's obliquity, parented to
## the planet so they track it. Gap between 2nd and 3rd reads as Cassini.
func _add_saturn_rings(body: MeshInstance3D, vis_r: float) -> void:
	var pts := PackedVector3Array()
	var tilt := Basis(Vector3.RIGHT, deg_to_rad(26.7))
	for f in [1.45, 1.75, 2.15]:
		var r: float = vis_r * f
		var prev := tilt * Vector3(r, 0, 0)
		for k in range(1, 65):
			var a := TAU * k / 64.0
			var p := tilt * Vector3(cos(a) * r, 0, sin(a) * r)
			pts.append(prev)
			pts.append(p)
			prev = p
	var rings := _line_mesh(pts, Mesh.PRIMITIVE_LINES)
	rings.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	rings.set_instance_shader_parameter("energy", 0.9)
	rings.name = "SaturnRings"
	body.add_child(rings)


## Moon + its orbit circle, parented to Earth (display-exaggerated radius,
## see Sim.MOON_ORBIT_VIS). Local position driven from _process.
func _add_moon(earth_body: MeshInstance3D) -> void:
	var pts := PackedVector3Array()
	for k in 97:
		pts.append(Sim.moon_local(Sim.MOON_PERIOD_D * k / 96.0))
	var orbit := _line_mesh(_dash(pts, 2, 2), Mesh.PRIMITIVE_LINES)
	orbit.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	orbit.set_instance_shader_parameter("energy", 0.30)
	orbit.name = "MoonOrbit"
	earth_body.add_child(orbit)

	moon_node = _wire_sphere(Sim.MOON_VIS_R, 8, 4)
	moon_node.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	moon_node.set_instance_shader_parameter("energy", 1.1)
	moon_node.name = "Moon"
	earth_body.add_child(moon_node)


## Main asteroid belt: dim point scatter 2.1-3.3 AU with slight vertical
## dispersion. Reuses the starfield point shader; rotated rigidly in
## _process at the belt's mean orbital rate.
## The sixteen real main-belt asteroids from sb441-n16.bsp, once mounted.
##
## Worth being precise about what changed, because they share the screen with
## `_build_belt`'s 1600 dust points: that belt is **scenery** — a seeded RNG
## annulus spun rigidly at a mean motion. These sixteen are positions read from a
## JPL kernel every frame, the same call the planets make. Same view, two very
## different epistemic statuses, so they are drawn as bodies (amber wire blobs
## with tags) rather than as more dust.
##
## Empty until the build lands with the kernel mounted; `Sim.asteroids` is the gate
## and it is empty when the mount did not happen.
func _build_asteroids() -> void:
	for el in Sim.asteroids:
		var node := _wire_blob(el.vis_r, 3.0)
		node.set_instance_shader_parameter("line_color", Color(1.0, 0.72, 0.25))
		# Brighter than the belt dust (0.05–0.20) by a wide margin — the dust is
		# scenery and these are measurements, and the screen has to say which.
		node.set_instance_shader_parameter("energy", 1.6)
		node.name = "Asteroid_%s" % el.name
		add_child(node)
		_asteroid_nodes.append(node)


func _build_belt() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = 27021801
	var verts := PackedVector3Array()
	var cols := PackedColorArray()
	for _i in 1600:
		var r := rng.randf_range(2.1, 3.3) * Sim.AU
		var a := rng.randf() * TAU
		var y := rng.randfn() * 0.03 * r
		verts.append(Vector3(cos(a) * r, y, sin(a) * r))
		var b := rng.randf_range(0.05, 0.20)
		cols.append(Color(b, b, b))
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_COLOR] = cols
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)
	var mat := ShaderMaterial.new()
	mat.shader = STAR_SHADER
	_belt = MeshInstance3D.new()
	_belt.mesh = mesh
	_belt.material_override = mat
	_belt.name = "AsteroidBelt"
	add_child(_belt)


## Build the threat's nodes from the core's integrated trajectory. Called on
## `mission_ready`, never at scene load — before that there is no trajectory.
func _build_threat() -> void:
	# Nominal (impact) orbit: bright solid track. Sampled once from the core; this
	# is the real integrated arc through the perturbed field, and it is an open
	# curve ending on Earth rather than a closed ellipse.
	_nom_orbit_line = _line_mesh(
		Sim.orbit_points(Sim.ast_el, 512), Mesh.PRIMITIVE_LINE_STRIP)
	_nom_orbit_line.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	_nom_orbit_line.set_instance_shader_parameter("energy", 0.9)
	_nom_orbit_line.name = "ThreatOrbit"
	add_child(_nom_orbit_line)

	# Deflected orbit: dashed, appears after the burn.
	_defl_orbit_line = _line_mesh(
		_dash(Sim.orbit_points(Sim.ast_defl_el, 384)), Mesh.PRIMITIVE_LINES)
	_defl_orbit_line.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	_defl_orbit_line.set_instance_shader_parameter("energy", 1.2)
	_defl_orbit_line.visible = false
	_defl_orbit_line.name = "DeflectedOrbit"
	add_child(_defl_orbit_line)

	ast_nominal = _wire_blob(Sim.ast_el.vis_r, 1.0)
	ast_nominal.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	ast_nominal.set_instance_shader_parameter("energy", 2.2)
	ast_nominal.name = "Threat"
	add_child(ast_nominal)

	ast_deflected = _wire_blob(Sim.ast_el.vis_r, 2.0)
	ast_deflected.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	ast_deflected.set_instance_shader_parameter("energy", 2.4)
	ast_deflected.visible = false
	ast_deflected.name = "ThreatDeflected"
	add_child(ast_deflected)


func _build_comet() -> void:
	_comet_orbit_line = _line_mesh(_dash(Sim.orbit_points(Sim.comet_el, 512), 3, 3),
		Mesh.PRIMITIVE_LINES)
	_comet_orbit_line.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	_comet_orbit_line.set_instance_shader_parameter("energy", 0.22)
	_comet_orbit_line.name = "CometOrbit"
	add_child(_comet_orbit_line)

	comet_node = _wire_blob(Sim.comet_el.vis_r, 3.0)
	comet_node.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	comet_node.set_instance_shader_parameter("energy", 1.8)
	comet_node.name = "Comet"
	add_child(comet_node)

	comet_tail = GPUParticles3D.new()
	comet_tail.amount = 1400
	comet_tail.lifetime = 3.5
	comet_tail.local_coords = false
	var pm := ParticleProcessMaterial.new()
	pm.direction = Vector3(1, 0, 0)
	pm.spread = 6.0
	pm.initial_velocity_min = 1.2
	pm.initial_velocity_max = 2.6
	pm.gravity = Vector3.ZERO
	pm.scale_min = 0.25
	pm.scale_max = 0.7
	var ramp := Gradient.new()
	ramp.set_color(0, Color(0.85, 0.85, 0.85, 0.22))
	ramp.set_color(1, Color(0.5, 0.5, 0.5, 0.0))
	var ramp_tex := GradientTexture1D.new()
	ramp_tex.gradient = ramp
	pm.color_ramp = ramp_tex
	comet_tail.process_material = pm
	var pmesh := QuadMesh.new()
	pmesh.size = Vector2(0.014, 0.014)
	var pmat := StandardMaterial3D.new()
	pmat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	pmat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	pmat.blend_mode = BaseMaterial3D.BLEND_MODE_ADD
	pmat.billboard_mode = BaseMaterial3D.BILLBOARD_ENABLED
	pmat.vertex_color_use_as_albedo = true
	pmesh.material = pmat
	comet_tail.draw_pass_1 = pmesh
	comet_node.add_child(comet_tail)


func _build_interceptor() -> void:
	_intercept_path_line = _line_mesh(_dash(Sim.interceptor_path(), 2, 2),
		Mesh.PRIMITIVE_LINES)
	_intercept_path_line.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	_intercept_path_line.set_instance_shader_parameter("energy", 0.8)
	_intercept_path_line.visible = false
	_intercept_path_line.name = "TransferArc"
	add_child(_intercept_path_line)

	# Small 3-axis cross marker.
	var s := 0.06
	var pts := PackedVector3Array([
		Vector3(-s, 0, 0), Vector3(s, 0, 0),
		Vector3(0, -s, 0), Vector3(0, s, 0),
		Vector3(0, 0, -s), Vector3(0, 0, s),
	])
	interceptor = _line_mesh(pts, Mesh.PRIMITIVE_LINES)
	interceptor.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	interceptor.set_instance_shader_parameter("energy", 2.8)
	interceptor.visible = false
	interceptor.name = "Interceptor"
	add_child(interceptor)

	intercept_flash = _wire_sphere(0.09, 8, 5)
	intercept_flash.set_instance_shader_parameter("line_color", Color(1, 1, 1))
	intercept_flash.set_instance_shader_parameter("energy", 3.0)
	intercept_flash.visible = false
	intercept_flash.position = Sim.pos3d(Sim.ast_el, Sim.T_INTERCEPT)
	intercept_flash.name = "InterceptFlash"
	add_child(intercept_flash)


## Plan-dependent geometry: deflected orbit, transfer arc, flash position.
## Rebuilt whenever the mission plan changes (Sim.plan_changed).
func _rebuild_plan_visuals() -> void:
	# Re-sampled from the core on every solve: the deflected arc is a different
	# integration per plan, not a redraw of the same one.
	_defl_orbit_line.mesh = _line_im(
		_dash(Sim.orbit_points(Sim.ast_defl_el, 384)), Mesh.PRIMITIVE_LINES)
	if not Sim.interceptor_online:
		return
	_intercept_path_line.mesh = _line_im(
		_dash(Sim.interceptor_path(), 2, 2), Mesh.PRIMITIVE_LINES)
	intercept_flash.position = Sim.pos3d(Sim.ast_el, Sim.T_INTERCEPT)


func _update_comet_tail() -> void:
	var anti_sun := comet_node.position.normalized()
	var pm: ParticleProcessMaterial = comet_tail.process_material
	pm.direction = anti_sun
	# Tail length scales with solar proximity.
	var r_au := comet_node.position.length() / Sim.AU
	var strength: float = clampf(2.5 / maxf(r_au, 0.3), 0.4, 5.0)
	pm.initial_velocity_min = strength * 0.5
	pm.initial_velocity_max = strength


# ------------------------------------------------------------------ meshes ---

func _line_im(pts: PackedVector3Array, primitive: int) -> ImmediateMesh:
	var im := ImmediateMesh.new()
	# An empty point list is a legitimate state, not a mistake: the deflected track
	# does not exist until the core has solved a plan. Godot errors on a surface
	# closed with no vertices, so empty in means an empty mesh out.
	if pts.is_empty():
		return im
	im.surface_begin(primitive)
	for p in pts:
		im.surface_add_vertex(p)
	im.surface_end()
	return im


func _line_mesh(pts: PackedVector3Array, primitive: int) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.mesh = _line_im(pts, primitive)
	mi.material_override = _line_mat
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	return mi


## Lat/long wireframe sphere as a single LINES mesh.
func _wire_sphere(radius: float, meridians: int, parallels: int) -> MeshInstance3D:
	var pts := PackedVector3Array()
	for p in range(1, parallels):
		var phi := PI * p / parallels - PI * 0.5
		var r := cos(phi) * radius
		var y := sin(phi) * radius
		var prev := Vector3(r, y, 0)
		for k in range(1, 49):
			var a := TAU * k / 48.0
			var pt := Vector3(cos(a) * r, y, sin(a) * r)
			pts.append(prev)
			pts.append(pt)
			prev = pt
	for mgd in meridians:
		var a := TAU * mgd / meridians
		var prev2 := Vector3(0, -radius, 0)
		for k in range(1, 33):
			var phi := PI * k / 32.0 - PI * 0.5
			var pt := Vector3(cos(a) * cos(phi) * radius, sin(phi) * radius,
				sin(a) * cos(phi) * radius)
			pts.append(prev2)
			pts.append(pt)
			prev2 = pt
	return _line_mesh(pts, Mesh.PRIMITIVE_LINES)


## Irregular wireframe blob (asteroid/comet nucleus): sphere wireframe with a
## deterministic lumpy radial perturbation (consistent across shared verts).
func _wire_blob(radius: float, seed_phase: float) -> MeshInstance3D:
	var pts := PackedVector3Array()
	var lump := func(d: Vector3) -> float:
		return radius * (1.0 + 0.30 * sin(4.1 * d.x + seed_phase)
			* cos(3.3 * d.y - seed_phase) + 0.18 * sin(5.7 * d.z + 2.0 * seed_phase))
	for p in range(1, 6):
		var phi := PI * p / 6.0 - PI * 0.5
		var prev := Vector3.ZERO
		for k in 33:
			var a := TAU * k / 32.0
			var d := Vector3(cos(a) * cos(phi), sin(phi), sin(a) * cos(phi))
			var pt: Vector3 = d * lump.call(d)
			if k > 0:
				pts.append(prev)
				pts.append(pt)
			prev = pt
	for mgd in 8:
		var a := TAU * mgd / 8.0
		var prev2 := Vector3.ZERO
		for k in 17:
			var phi := PI * k / 16.0 - PI * 0.5
			var d := Vector3(cos(a) * cos(phi), sin(phi), sin(a) * cos(phi))
			var pt: Vector3 = d * lump.call(d)
			if k > 0:
				pts.append(prev2)
				pts.append(pt)
			prev2 = pt
	return _line_mesh(pts, Mesh.PRIMITIVE_LINES)


## Convert a polyline into dashed LINES pairs (keep n, skip m).
func _dash(pts: PackedVector3Array, keep: int = 2, skip: int = 2) -> PackedVector3Array:
	var out := PackedVector3Array()
	var period := keep + skip
	for k in pts.size() - 1:
		if k % period < keep:
			out.append(pts[k])
			out.append(pts[k + 1])
	return out
