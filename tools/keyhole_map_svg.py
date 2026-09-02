#!/usr/bin/env python3
"""Render the keyhole map JSON (`probe_keyhole_map`) as a static SVG.

No dependencies beyond the standard library, so it runs anywhere the JSON does.
Two panels at two scales, because the picture has structure at both: the full
map out to ~14 capture radii, where the far ends of the wide circles are, and a
close-up of the disc, where every circle threads through and the grazing points
sit. The uncertainty ellipse is sub-kilometre and invisible at either scale, so
it gets a third, tiny panel of its own at the nominal b-point.

Usage::

    python tools/keyhole_map_svg.py docs/keyhole_map.json docs/keyhole_map.svg
"""

from __future__ import annotations

import json
import math
import sys

# The project's instrument palette: phosphor on near-black.
GROUND = "#07100c"
GRID = "#16241c"
INK = "#c9d6cc"
MUTED = "#6f8276"
FAINT = "#3a4a41"
PHOSPHOR = "#5ce09b"  # the encounter's own marks
AMBER = "#f2b950"  # flown deflected passes
DISC = "#e8f0ea"
# Resonant circles keyed by return time, one hue, light -> dark with years.
CIRCLE_RAMP = ["#9fe8ff", "#6fcbef", "#4aa8d4", "#3286b3", "#25678f", "#1c4d6d"]

FONT = "'IBM Plex Mono', 'DejaVu Sans Mono', Menlo, monospace"


def circle_colour(h: int) -> str:
    if h <= 2:
        return CIRCLE_RAMP[0]
    if h <= 3:
        return CIRCLE_RAMP[1]
    if h <= 5:
        return CIRCLE_RAMP[2]
    if h <= 7:
        return CIRCLE_RAMP[3]
    if h <= 12:
        return CIRCLE_RAMP[4]
    return CIRCLE_RAMP[5]


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Panel:
    """A square b-plane panel: (ξ, ζ) km -> pixels, ξ right, ζ up."""

    def __init__(self, x0: float, y0: float, size: float, half_km: float):
        self.x0, self.y0, self.size, self.half = x0, y0, size, half_km
        self.cx = x0 + size / 2
        self.cy = y0 + size / 2
        self.ppk = size / 2 / half_km

    def px(self, xi: float, zeta: float) -> tuple[float, float]:
        return self.cx + xi * self.ppk, self.cy - zeta * self.ppk

    def clip_id(self) -> str:
        return f"clip{int(self.x0)}_{int(self.y0)}"


def nice_rings(half_km: float) -> list[float]:
    step = 10 ** math.floor(math.log10(half_km))
    if half_km / step < 2:
        step /= 5
    elif half_km / step < 5:
        step /= 2
    rings = []
    r = step
    while r < half_km * 1.05:
        rings.append(r)
        r += step
    return rings


def render_panel(out: list[str], p: Panel, d: dict, title: str, label_circles: bool,
                 max_years: int | None) -> None:
    enc = d["encounter"]
    cap = enc["capture_radius_km"]
    re_ = enc["earth_radius_km"]
    cid = p.clip_id()
    out.append(f'<clipPath id="{cid}"><rect x="{p.x0}" y="{p.y0}" width="{p.size}" '
               f'height="{p.size}"/></clipPath>')
    out.append(f'<rect x="{p.x0}" y="{p.y0}" width="{p.size}" height="{p.size}" '
               f'fill="{GROUND}" stroke="{FAINT}" stroke-width="1"/>')
    out.append(f'<g clip-path="url(#{cid})">')
    # Axes and rings.
    out.append(f'<line x1="{p.x0}" y1="{p.cy}" x2="{p.x0 + p.size}" y2="{p.cy}" '
               f'stroke="{GRID}" stroke-width="1"/>')
    out.append(f'<line x1="{p.cx}" y1="{p.y0}" x2="{p.cx}" y2="{p.y0 + p.size}" '
               f'stroke="{GRID}" stroke-width="1"/>')
    for r in nice_rings(p.half):
        rp = r * p.ppk
        if rp < cap * p.ppk + 8:
            continue
        out.append(f'<circle cx="{p.cx}" cy="{p.cy}" r="{rp:.1f}" fill="none" '
                   f'stroke="{GRID}" stroke-width="1"/>')
        lx, ly = p.cx + rp * 0.7071 + 4, p.cy - rp * 0.7071 - 4
        out.append(f'<text x="{lx:.1f}" y="{ly:.1f}" fill="{MUTED}" font-size="10" '
                   f'font-family="{FONT}">{r:g} km</text>')

    # Resonant circles.
    circles = [c for c in d["circles"] if max_years is None or c["h"] <= max_years]
    for c in sorted(circles, key=lambda c: -c["radius_km"]):
        cx, cy = p.px(0.0, c["center_zeta_km"])
        rp = c["radius_km"] * p.ppk
        col = circle_colour(c["h"])
        width = 1.6 if c["h"] <= 7 else 0.8
        alpha = 0.95 if c["h"] <= 7 else 0.45
        out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{rp:.1f}" fill="none" '
                   f'stroke="{col}" stroke-width="{width}" stroke-opacity="{alpha}"/>')
    # Labels along the +ξ side of the far crossing of the ζ axis, for the
    # circles that matter and fit.
    if label_circles:
        placed: list[tuple[float, float]] = []
        for i, c in enumerate(sorted(circles, key=lambda c: c["farthest"]["point_km"][1])):
            if c["h"] > 7:
                continue
            # Label at the far crossing of the ζ axis when it is on the panel,
            # else at the near one; alternate sides so a stack of circles whose
            # far ends coincide reads as a column on each side.
            far = c["farthest"]["point_km"]
            at = far if abs(far[1]) < p.half * 0.95 else c["nearest"]["point_km"]
            fx, fy = p.px(at[0], at[1])
            side = 1 if i % 2 == 0 else -1
            fx += side * 8
            while any(abs(fy - y) < 11 and abs(fx - x) < 40 for x, y in placed):
                fy += 11
            placed.append((fx, fy))
            bold = ' font-weight="700"' if (c["h"], c["k"]) == (3, 4) else ""
            anchor = "start" if side > 0 else "end"
            out.append(f'<text x="{fx:.1f}" y="{fy + 4:.1f}" fill="{circle_colour(c["h"])}" '
                       f'font-size="10" font-family="{FONT}" text-anchor="{anchor}"{bold}>'
                       f'{c["h"]}:{c["k"]}</text>')

    # Earth and the capture disc.
    out.append(f'<circle cx="{p.cx}" cy="{p.cy}" r="{cap * p.ppk:.1f}" fill="{DISC}" '
               f'fill-opacity="0.06" stroke="{DISC}" stroke-width="1" stroke-dasharray="4 3"/>')
    out.append(f'<circle cx="{p.cx}" cy="{p.cy}" r="{max(re_ * p.ppk, 1.5):.1f}" fill="#121c16" '
               f'stroke="{DISC}" stroke-width="1.2"/>')
    if cap * p.ppk > 30:
        lx, ly = p.cx + cap * p.ppk * 0.7071 + 5, p.cy + cap * p.ppk * 0.7071 + 12
        out.append(f'<text x="{lx:.1f}" y="{ly:.1f}" fill="{MUTED}" font-size="10" '
                   f'font-family="{FONT}">capture {cap / re_:.2f} R⊕</text>')

    # Grazing points on the close-up: where each circle enters the disc.
    if p.half < 5 * cap:
        for c in circles:
            g = c.get("grazing")
            if not g:
                continue
            gx, gy = p.px(*g["point_km"])
            out.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="2.2" fill="{circle_colour(c["h"])}" '
                       f'stroke="{GROUND}" stroke-width="1"/>')
            out.append(f'<circle cx="{2 * p.cx - gx:.1f}" cy="{gy:.1f}" r="2.2" '
                       f'fill="{circle_colour(c["h"])}" stroke="{GROUND}" stroke-width="1"/>')

    # The nominal and the flown deflections.
    nx, ny = p.px(*d["nominal"]["point_km"])
    out.append(f'<line x1="{nx - 5}" y1="{ny - 5}" x2="{nx + 5}" y2="{ny + 5}" stroke="{PHOSPHOR}" '
               f'stroke-width="1.6"/>')
    out.append(f'<line x1="{nx - 5}" y1="{ny + 5}" x2="{nx + 5}" y2="{ny - 5}" stroke="{PHOSPHOR}" '
               f'stroke-width="1.6"/>')
    if p.half < 5 * cap:
        out.append(f'<text x="{nx + 9:.1f}" y="{ny - 6:.1f}" fill="{PHOSPHOR}" font-size="10" '
                   f'font-family="{FONT}">nominal · impact</text>')
    for df in d["deflected"]:
        x, y = p.px(*df["point_km"])
        if not (p.x0 <= x <= p.x0 + p.size and p.y0 <= y <= p.y0 + p.size):
            continue
        pts = f"{x},{y - 6} {x + 6},{y} {x},{y + 6} {x - 6},{y}"
        out.append(f'<polygon points="{pts}" fill="{GROUND}" stroke="{AMBER}" stroke-width="1.5"/>')
        out.append(f'<text x="{x + 10:.1f}" y="{y + 4:.1f}" fill="{AMBER}" font-size="10" '
                   f'font-family="{FONT}">{df["label"]} {df["dv_m_s"]:+.1f} m/s · '
                   f'a′ {df["a_closed_form_au"]:.4f} AU</text>')
    out.append("</g>")
    out.append(f'<text x="{p.x0 + 8}" y="{p.y0 + 16}" fill="{INK}" font-size="12" '
               f'font-family="{FONT}" font-weight="600">{esc(title)}</text>')
    out.append(f'<text x="{p.x0 + p.size - 8}" y="{p.y0 + p.size - 8}" fill="{MUTED}" '
               f'font-size="10" font-family="{FONT}" text-anchor="end">ξ → · ζ ↑ · ±{p.half:g} km</text>')


def render_ellipse(out: list[str], x0: float, y0: float, size: float, d: dict) -> None:
    e = d["ellipse"]
    sa, sb = e["sigma_axes_km"]
    ang = e["major_axis_deg_from_xi"]
    half = 3.2 * sa
    p = Panel(x0, y0, size, half)
    out.append(f'<rect x="{x0}" y="{y0}" width="{size}" height="{size}" fill="{GROUND}" '
               f'stroke="{FAINT}"/>')
    out.append(f'<line x1="{x0}" y1="{p.cy}" x2="{x0 + size}" y2="{p.cy}" stroke="{GRID}"/>')
    out.append(f'<line x1="{p.cx}" y1="{y0}" x2="{p.cx}" y2="{y0 + size}" stroke="{GRID}"/>')
    for n, alpha in ((3, 0.35), (1, 0.9)):
        out.append(f'<ellipse cx="{p.cx}" cy="{p.cy}" rx="{n * sa * p.ppk:.1f}" '
                   f'ry="{max(n * sb * p.ppk, 0.8):.1f}" transform="rotate({-ang:.2f} {p.cx} {p.cy})" '
                   f'fill="{PHOSPHOR}" fill-opacity="{0.08 * n}" stroke="{PHOSPHOR}" '
                   f'stroke-opacity="{alpha}" stroke-width="1"/>')
    out.append(f'<text x="{x0 + 8}" y="{y0 + 16}" fill="{INK}" font-size="12" font-family="{FONT}" '
               f'font-weight="600">uncertainty at the nominal</text>')
    lines = [
        f"σ {sa:.1f} × {sb:.2f} km (1σ, 3σ)",
        f"major axis {ang:.0f}° from ξ — along ζ, timing",
        f"P(impact) {e['impact_probability']:.3f}",
        "invented covariance, labelled as such",
    ]
    for i, s in enumerate(lines):
        out.append(f'<text x="{x0 + 8}" y="{y0 + size - 8 - (len(lines) - 1 - i) * 13}" '
                   f'fill="{MUTED}" font-size="10" font-family="{FONT}">{esc(s)}</text>')


def render(d: dict) -> str:
    W, H = 1260, 760
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'viewBox="0 0 {W} {H}" font-family="{FONT}">',
           f'<rect width="{W}" height="{H}" fill="{GROUND}"/>']
    enc = d["encounter"]
    cap = enc["capture_radius_km"]
    out.append(f'<text x="24" y="34" fill="{INK}" font-size="18" font-weight="600">'
               f'Keyhole map — b-plane of the {esc(d["encounter_epoch_tdb"][:10])} encounter</text>')
    out.append(f'<text x="24" y="54" fill="{MUTED}" font-size="11">Öpik frame (ξ, ζ), pinned in '
               f'core/src/keyhole.rs · v∞ {enc["v_inf_km_s"]:.2f} km/s · θ {d["frame"]["theta_deg"]:.1f}° · '
               f'c {d["frame"]["c_km"]:.0f} km · capture {cap:.0f} km · '
               f'round trip {enc["round_trip_rel"]:.1e}</text>')

    big = Panel(24, 72, 640, 14.0 * cap)
    render_panel(out, big, d, "full map · resonant-return circles, h ≤ 7 yr labelled", True, None)
    zoom = Panel(690, 72, 320, 2.6 * cap)
    render_panel(out, zoom, d, "the disc · where the circles enter", False, 7)
    render_ellipse(out, 690, 416, 320, d)

    # Legend / readout column.
    x, y = 1036, 84
    rows = [
        (PHOSPHOR, "×", "nominal b-point (the impact)"),
        (AMBER, "◇", "flown −/+0.2 m/s at 12 yr"),
        (DISC, "◌", "capture disc, dashed"),
    ]
    for col, glyph, text in rows:
        out.append(f'<text x="{x}" y="{y}" fill="{col}" font-size="12">{glyph}</text>')
        out.append(f'<text x="{x + 16}" y="{y}" fill="{INK}" font-size="10">{esc(text)}</text>')
        y += 16
    y += 6
    out.append(f'<text x="{x}" y="{y}" fill="{INK}" font-size="10">circles by return time</text>')
    y += 14
    for (lo, hi), col in zip(((2, 2), (3, 3), (4, 5), (6, 7), (8, 12), (13, 20)), CIRCLE_RAMP):
        out.append(f'<rect x="{x}" y="{y - 8}" width="14" height="8" fill="{col}"/>')
        label = f"{lo} yr" if lo == hi else f"{lo}–{hi} yr"
        out.append(f'<text x="{x + 20}" y="{y}" fill="{MUTED}" font-size="10">{label}</text>')
        y += 14
    y += 10
    out.append(f'<text x="{x}" y="{y}" fill="{INK}" font-size="10">keyholes, h ≤ 7</text>')
    y += 6
    out.append(f'<text x="{x}" y="{y + 12}" fill="{MUTED}" font-size="9">h:k   a′ AU   graze→far km</text>')
    y += 24
    for c in d["circles"]:
        if c["h"] > 7:
            continue
        g = c.get("grazing")
        near = g["width_km"] if g else c["nearest"]["width_km"]
        far = c["farthest"]["width_km"]
        out.append(f'<text x="{x}" y="{y}" fill="{circle_colour(c["h"])}" font-size="9">'
                   f'{c["h"]}:{c["k"]:<3d} {c["a_prime_au"]:.4f}  {near:6.2f} → {far:6.2f}</text>')
        y += 12
        if y > H - 40:
            break
    out.append(f'<text x="24" y="{H - 14}" fill="{MUTED}" font-size="10">'
               f'{len(d["circles"])} resonant circles within 60 capture radii, 2–20 yr returns. '
               f'Closed-form placement is good to ~1e-4 in a′ (~100 capture radii at the return); '
               f'the widths are gradients and are trustworthy; every return claim needs the propagator.</text>')
    out.append("</svg>")
    return "\n".join(out)


def main() -> None:
    src = sys.argv[1] if len(sys.argv) > 1 else "docs/keyhole_map.json"
    dst = sys.argv[2] if len(sys.argv) > 2 else "docs/keyhole_map.svg"
    with open(src, encoding="utf-8") as f:
        d = json.load(f)
    svg = render(d)
    with open(dst, "w", encoding="utf-8") as f:
        f.write(svg)
    print(f"wrote {dst} ({len(svg)} bytes)")


if __name__ == "__main__":
    main()
