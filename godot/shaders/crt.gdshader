// Retro phosphor-CRT post-process. Applied to the SubViewportContainer that
// shows the whole game (3D world + HUD), so everything on screen obeys the
// monochrome phosphor palette. Renders at native resolution — the retro feel
// comes from tint/scanlines/glow, never from downscaling or pixelation.
shader_type canvas_item;

uniform vec3 phosphor : source_color = vec3(0.25, 1.0, 0.45);
uniform float mono_mix : hint_range(0.0, 1.0) = 1.0;
uniform float curvature : hint_range(0.0, 0.5) = 0.07;
uniform float scan_intensity : hint_range(0.0, 1.0) = 0.13;
uniform float vignette_strength : hint_range(0.0, 1.0) = 0.32;
uniform float noise_amount : hint_range(0.0, 0.2) = 0.03;
uniform float flicker_amount : hint_range(0.0, 0.2) = 0.015;
uniform float bleed_px : hint_range(0.0, 4.0) = 1.0;
uniform float halo_strength : hint_range(0.0, 1.0) = 0.35;
uniform float brightness : hint_range(0.5, 2.0) = 1.18;

float hash12(vec2 p) {
	vec3 p3 = fract(vec3(p.xyx) * 0.1031);
	p3 += dot(p3, p3.yzx + 33.33);
	return fract((p3.x + p3.y) * p3.z);
}

void fragment() {
	// Gentle barrel curvature.
	vec2 cc = UV - 0.5;
	float r2 = dot(cc, cc);
	vec2 uv = UV + cc * r2 * curvature;

	vec3 col = vec3(0.0);
	if (uv.x >= 0.0 && uv.x <= 1.0 && uv.y >= 0.0 && uv.y <= 1.0) {
		vec2 px = TEXTURE_PIXEL_SIZE;

		// Slight horizontal bleed (electron-beam smear), sub-pixel scale.
		vec3 c = texture(TEXTURE, uv).rgb * 0.72;
		c += texture(TEXTURE, uv + vec2(px.x * bleed_px, 0.0)).rgb * 0.14;
		c += texture(TEXTURE, uv - vec2(px.x * bleed_px, 0.0)).rgb * 0.14;

		// Soft halo so HUD text/lines get a phosphor glow (3D already has env glow).
		vec3 halo = texture(TEXTURE, uv + vec2(0.0,  px.y * 2.0)).rgb
		          + texture(TEXTURE, uv - vec2(0.0,  px.y * 2.0)).rgb
		          + texture(TEXTURE, uv + vec2(px.x * 2.0, 0.0)).rgb
		          + texture(TEXTURE, uv - vec2(px.x * 2.0, 0.0)).rgb;
		c += halo * 0.25 * halo_strength * 0.30;

		// Monochrome phosphor mapping: luminance -> tone curve -> tint.
		float lum = dot(c, vec3(0.299, 0.587, 0.114));
		lum = pow(lum, 0.82);
		col = mix(c, phosphor * lum, mono_mix);

		// Fine scanlines locked to output pixel rows (crisp at any resolution).
		float row = uv.y / px.y;
		float scan = 0.5 + 0.5 * sin(row * 3.14159265);
		col *= 1.0 - scan_intensity * scan;

		// Slow rolling brightness band.
		col *= 1.0 + 0.025 * sin(6.2831853 * (uv.y * 1.3 - TIME * 0.06));

		// Per-pixel noise grain + global flicker.
		float n = hash12(uv * vec2(1913.0, 1021.0) + vec2(TIME * 61.3, TIME * 12.7));
		col *= 1.0 + noise_amount * (n - 0.5) * 2.0;
		col *= 1.0 + flicker_amount * (hash12(vec2(TIME * 7.0, 0.37)) - 0.5) * 2.0;

		// Vignette toward the tube corners.
		col *= mix(1.0, 1.0 - vignette_strength, smoothstep(0.12, 0.55, r2));

		col *= brightness;
	}
	COLOR = vec4(col, 1.0);
}
