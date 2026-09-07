// Point-sprite starfield on a far sphere. Per-vertex color carries the
// star's brightness (set once at build time).
shader_type spatial;
render_mode blend_add, depth_draw_never, cull_disabled, shadows_disabled, specular_disabled, unshaded;

uniform float point_px = 2.0;
// Global brightness scale. 1.0 for the starfield, which never moves; the
// scenery belt turns this down as the warp rises, because 1600 points sweeping
// a rigid annulus at 10 yr/s smear into one solid band under the phosphor
// persistence and swallow the labelled bodies in front of them.
uniform float dim = 1.0;

void vertex() {
	POINT_SIZE = point_px;
}

void fragment() {
	ALBEDO = COLOR.rgb * dim;
}
