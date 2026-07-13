// Shared emissive material for all vector-display geometry (orbit lines,
// wireframe bodies, grids). Per-instance color/energy via instance uniforms
// so one material serves every line in the scene. Emission > 1.0 feeds the
// WorldEnvironment glow for the phosphor bloom.
shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_disabled, shadows_disabled, specular_disabled;

instance uniform vec3 line_color : source_color = vec3(0.9, 0.9, 0.9);
instance uniform float energy = 1.4;

void fragment() {
	ALBEDO = vec3(0.0);
	EMISSION = line_color * energy;
	ROUGHNESS = 1.0;
}
