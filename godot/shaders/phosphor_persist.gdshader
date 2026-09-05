// Phosphor persistence for the 3D world layer.
//
// Drawn as a full-rect ColorRect inside a SubViewport whose render target is
// NEVER cleared, so the target holds last frame's result when this runs. Each
// pixel becomes `max(world_now, previous * keep)`: whatever is on screen right
// now is drawn at full brightness, and whatever was there before fades by
// `keep` per frame until something brighter replaces it. A body that moves
// leaves a trail; anything static is exactly its own brightness, because
// `max(w, w * keep) == w` — nothing is dimmed, nothing accumulates.
//
// **Not an average.** The first version blended `world * (1 - keep) +
// previous * keep`, which is the textbook exponential trail and wrong here: a
// body that moves more than its own width per frame is only ever drawn at
// `1 - keep` of its brightness, so at the top warp step every planet faded to
// a few percent and the screen showed tags pointing at nothing. Peak-hold
// keeps the moving thing bright and lets its past decay behind it.
//
// `keep` is set per frame from the frame time (main.gd) so the trail length is
// a time, not a frame count. Only the 3D world goes through this: text, tags
// and the 2D plots sit above it and never persist.
//
// `previous` is read from the viewport's own back buffer (hint_screen_texture
// — Godot copies the target before this item draws), and blending is disabled
// so the result overwrites. The target viewport is HDR (16-bit float): in
// 8-bit a value times 0.9 rounds back to itself below ~4/255 and trails never
// reach black, leaving permanent faint ghosts.
shader_type canvas_item;
render_mode blend_disabled, unshaded;

uniform sampler2D world_tex : filter_linear;
uniform sampler2D previous : hint_screen_texture, filter_nearest;
uniform float keep : hint_range(0.0, 1.0) = 0.9;

void fragment() {
	vec3 w = texture(world_tex, UV).rgb;
	vec3 p = texture(previous, SCREEN_UV).rgb * keep;
	COLOR = vec4(max(w, p), 1.0);
}
