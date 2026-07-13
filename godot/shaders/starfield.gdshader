// Point-sprite starfield on a far sphere. Per-vertex color carries the
// star's brightness (set once at build time).
shader_type spatial;
render_mode blend_add, depth_draw_never, cull_disabled, shadows_disabled, specular_disabled, unshaded;

uniform float point_px = 2.0;

void vertex() {
	POINT_SIZE = point_px;
}

void fragment() {
	ALBEDO = COLOR.rgb;
}
