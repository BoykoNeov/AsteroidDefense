class_name SkyView
extends Control
## The sky-observation screen ([5]) — a real asteroid photographed against real
## stars, the way it is first found: several shots of one patch of sky, blinked.
##
## docs/plans/2026-10-10-sky-observation-screen.md, step 4. Apophis from Mt.
## Lemmon Survey across its March 2021 approach: three nights, three 60 s
## exposures 20 minutes apart each night. Blink them and the stars stay put while
## one faint dot jumps — that is the asteroid, and the jump is its motion.
##
## **What is real and what is a game choice, as the screen says it:**
## - real: every star (Tycho-2), the asteroid's position, brightness and motion
##   (matched to JPL Horizons), the dark-sky window each night, and the size of
##   the error the asteroid is drawn with (Mt. Lemmon's published 0.31"/0.28").
## - chosen: the 30' field and the pointing — the telescope is aimed at the
##   asteroid's true position, which no real observer of a new object can do.
## - shown as it is: the asteroid (V ~15.6) is **fainter than every catalogue
##   star in the frame**, because Tycho-2 stops near V 12. It is not brightened.
##
## Orientation is the astronomer's: north up, **east to the left** (the sky seen
## from below, not a map seen from above).
##
## Pure display: every position arrives from the core through `Sim`. Key
## handling lives in main.gd.

const TOP_RESERVE := 96.0
## The event console's block along the bottom-left stays clear: at 900 px high
## its header sits near y 680 (seen in `_sky_shot.gd`'s first picture, where a
## 150 px reserve ran the frame through it).
const BOTTOM_RESERVE := 240.0
const PANEL_W := 420.0
const MARGIN := 32.0
## Seconds each shot stays up while blinking — about the rate a blink comparator
## was flicked by hand.
const BLINK_S := 0.7
## Zoom steps. 0 is the wide finder chart (naked-eye stars and names around the
## pointing); 1.. are the telescope field at 1x, 2x, 4x.
const ZOOMS: Array[float] = [0.0, 1.0, 2.0, 4.0]
## Faintest star the size ramp is drawn against, per mode.
const TELESCOPE_MAG := 12.0
const FINDER_MAG := 6.0

var _font: Font
var _fs := 13

## Which shot is up (index into Sim.sky_shots).
var shot_idx := 0
## Blinking through the current night's shots.
var blinking := true
## All of the current night's asteroid images over one frame of stars.
var stacked := false
## Star names drawn.
var labels := true
var zoom_idx := 1
var _blink_t := 0.0
## The info panel's running text cursor (see `_line`).
var _px := 0.0
var _py := 0.0


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_preset(Control.PRESET_FULL_RECT)
	_font = Sim.mono_font
	Sim.sky_changed.connect(func() -> void: shot_idx = clampi(shot_idx, 0, maxi(Sim.sky_shots.size() - 1, 0)))
	# At scene load the kernel field may not be up; the run is asked for when the
	# screen is opened, and again when the field lands if the screen is up then.
	Sim.field_online.connect(func() -> void:
		if visible:
			Sim.request_sky_run())


func _process(delta: float) -> void:
	if not visible:
		return
	if blinking and Sim.sky_online and not stacked and Sim.sky_shots.size() > 0:
		_blink_t += delta
		if _blink_t >= BLINK_S:
			_blink_t = 0.0
			_step_within_night(1)
	queue_redraw()


# ------------------------------------------------------------------ keys ---

## Previous/next shot across the whole run. Stops the blink: stepping by hand is
## how a shot is inspected.
func step_shot(d: int) -> void:
	if Sim.sky_shots.is_empty():
		return
	blinking = false
	shot_idx = posmod(shot_idx + d, Sim.sky_shots.size())


func zoom(d: int) -> void:
	zoom_idx = clampi(zoom_idx + d, 0, ZOOMS.size() - 1)


func toggle_blink() -> void:
	blinking = not blinking
	if blinking:
		stacked = false
	_blink_t = 0.0


func toggle_stack() -> void:
	stacked = not stacked
	if stacked:
		blinking = false


func toggle_labels() -> void:
	labels = not labels


func _step_within_night(d: int) -> void:
	var night := _night_of(shot_idx)
	var members := _night_members(night)
	if members.is_empty():
		return
	var k := members.find(shot_idx)
	shot_idx = members[posmod(k + d, members.size())]


func _night_of(i: int) -> int:
	return int((Sim.sky_shots[i].info as Dictionary).get("night", 0))


func _night_members(night: int) -> Array[int]:
	var out: Array[int] = []
	for i in range(Sim.sky_shots.size()):
		if _night_of(i) == night:
			out.append(i)
	return out


# ------------------------------------------------------------------ draw ---

func _draw() -> void:
	var w := size.x
	var h := size.y
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.004, 0.006, 0.005), true)
	var bright := Color(1, 1, 1)
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.42, 0.42, 0.42)
	var faint := Color(0.18, 0.18, 0.18)

	if not Sim.sky_online:
		var msg := "PLANNING THE OBSERVING RUN ..." if Sim.sky_building else "OBSERVING RUN OFFLINE"
		_centered(msg, Vector2(w * 0.5, h * 0.5), dim, _fs)
		if Sim.sky_error != "":
			_centered(Sim.sky_error.to_upper(), Vector2(w * 0.5, h * 0.5 + 20.0), faint, _fs - 2)
		return

	var side := minf(w - PANEL_W - 3.0 * MARGIN, h - TOP_RESERVE - BOTTOM_RESERVE)
	var frame := Rect2(Vector2(MARGIN, TOP_RESERVE), Vector2(side, side))
	var shot: Dictionary = Sim.sky_shots[shot_idx]
	var info: Dictionary = shot.info
	var night := int(info.get("night", 0))
	var finder := ZOOMS[zoom_idx] == 0.0
	var half_arcsec: float = float(Sim.sky_info.half_width_arcsec)
	# Arcseconds from the frame centre to its edge, in this mode.
	var reach: float = float(Sim.sky_info.finder_radius_deg) * 3600.0 if finder \
		else half_arcsec / ZOOMS[zoom_idx]
	var px_per_arcsec := side * 0.5 / reach

	draw_rect(frame, Color(0.0, 0.0, 0.0), true)
	draw_rect(frame, faint, false, 1.0)
	if finder:
		var fs: Dictionary = Sim.sky_finders[night]
		_draw_stars(frame, px_per_arcsec, fs.stars, fs.labels, FINDER_MAG, mid, dim)
		# The telescope's field, inside the finder chart.
		var c := frame.get_center()
		var hp := half_arcsec * px_per_arcsec
		draw_rect(Rect2(c - Vector2(hp, hp), Vector2(2.0 * hp, 2.0 * hp)), bright, false, 1.0)
		_t(c + Vector2(hp + 4.0, -hp + 10.0), "TELESCOPE FIELD", mid, _fs - 3)
	else:
		_draw_stars(frame, px_per_arcsec, shot.stars, shot.labels, TELESCOPE_MAG, mid, dim)
		if stacked:
			var n := 1
			for i in _night_members(night):
				_draw_rock(frame, px_per_arcsec, Sim.sky_shots[i].rock, str(n), mid)
				n += 1
		else:
			_draw_rock(frame, px_per_arcsec, shot.rock, "", mid)
	_draw_compass(frame, dim)
	_draw_scale_bar(frame, px_per_arcsec, finder, dim)
	_draw_panel(Vector2(frame.end.x + 2.0 * MARGIN, TOP_RESERVE), shot, finder, bright, mid, dim, faint)


## Stars as discs sized by brightness, brightest largest. Flat [xi, eta, V, ref].
func _draw_stars(frame: Rect2, k: float, stars: PackedFloat64Array, names: PackedStringArray,
		mag_limit: float, mid: Color, dim: Color) -> void:
	var n := stars.size() / 4
	for j in range(n):
		var p := _to_screen(frame, k, stars[4 * j], stars[4 * j + 1])
		if not frame.has_point(p):
			continue
		var v := stars[4 * j + 2]
		var r := clampf(1.0 + 0.8 * (mag_limit - v), 1.0, 7.0)
		var a := clampf(0.45 + 0.08 * (mag_limit - v), 0.45, 1.0)
		draw_circle(p, r, Color(1, 1, 1, a))
		if labels and j < names.size() and names[j] != "":
			_t(p + Vector2(r + 3.0, 4.0), names[j], dim if v > 3.0 else mid, _fs - 3)


## The asteroid as drawn: its trail across the exposure. Drawn at the brightness
## the frame actually gives it — dimmer than the faintest star — not brightened.
func _draw_rock(frame: Rect2, k: float, rock: PackedFloat64Array, tag: String, mid: Color) -> void:
	if rock.size() < 7:
		return
	var a := _to_screen(frame, k, rock[2], rock[3])
	var b := _to_screen(frame, k, rock[4], rock[5])
	if not (frame.has_point(a) or frame.has_point(b)):
		return
	var col := Color(1, 1, 1, 0.42)
	if a.distance_to(b) < 1.5:
		draw_circle((a + b) * 0.5, 1.0, col)
	else:
		draw_line(a, b, col, 1.6)
	if tag != "":
		_t((a + b) * 0.5 + Vector2(5.0, -5.0), tag, mid, _fs - 3)


## North up, east LEFT: a positive xi (east) is drawn to the left of centre.
func _to_screen(frame: Rect2, k: float, xi: float, eta: float) -> Vector2:
	return frame.get_center() + Vector2(-xi * k, -eta * k)


func _draw_compass(frame: Rect2, dim: Color) -> void:
	var o := frame.position + Vector2(frame.size.x - 34.0, 40.0)
	draw_line(o, o + Vector2(0, -22), dim, 1.0)
	draw_line(o, o + Vector2(-22, 0), dim, 1.0)
	_t(o + Vector2(-4, -26), "N", dim, _fs - 3)
	_t(o + Vector2(-34, 4), "E", dim, _fs - 3)


func _draw_scale_bar(frame: Rect2, k: float, finder: bool, dim: Color) -> void:
	var arcsec := 3600.0 if finder else 60.0
	var label := "1 DEG" if finder else "1 ARCMIN"
	if not finder and ZOOMS[zoom_idx] >= 4.0:
		arcsec = 10.0
		label = "10 ARCSEC"
	var len_px := arcsec * k
	var y := frame.end.y - 14.0
	var x := frame.position.x + 14.0
	draw_line(Vector2(x, y), Vector2(x + len_px, y), dim, 1.0)
	_t(Vector2(x, y - 5.0), label, dim, _fs - 3)


func _draw_panel(at: Vector2, shot: Dictionary, finder: bool, bright: Color, mid: Color,
		dim: Color, faint: Color) -> void:
	var info: Dictionary = shot.info
	var night := int(info.get("night", 0))
	var nights := int(Sim.sky_info.get("nights", 0))
	var per_night := _night_members(night).size()
	_px = at.x
	_py = at.y
	_line("%s FROM %s" % [Sim.sky_info.target, Sim.sky_info.site], bright)
	_line("NIGHT %d OF %d   SHOT %d OF %d%s" % [night + 1, nights,
		int(info.get("index_in_night", 0)) + 1, per_night,
		"   BLINKING" if blinking else ("   STACKED" if stacked else "")], mid)
	_line("%s   EXPOSURE %d S" % [info.get("utc", ""), int(info.get("exposure_s", 0))], mid)
	_py += 6.0
	_line("BRIGHTNESS   V %.2f" % float(info.v_mag), mid)
	var faintest := _faintest_star(shot.stars)
	if float(info.v_mag) > faintest:
		_line("  FAINTER THAN EVERY STAR IN THIS FRAME", dim, _fs - 2)
		_line("  (THE STAR CATALOGUE STOPS NEAR V 12)", dim, _fs - 2)
	_line("DISTANCE     %.3f AU" % float(info.delta_au), mid)
	_line("MOTION       %.1f\"/MIN ACROSS THE SKY" % float(info.rate_arcsec_min), mid)
	_line("ALTITUDE     %.0f DEG   SUN %.0f DEG" % [float(info.alt_deg), float(info.sun_alt_deg)], mid)
	_line("POINTING     RA %.3f  DEC %+.3f" % [float(info.pointing_ra_deg), float(info.pointing_dec_deg)], mid)
	_py += 6.0
	if finder:
		_line("FINDER CHART: NAKED-EYE STARS (V<6)", dim, _fs - 2)
		_line("WITHIN %d DEG OF THE POINTING" % int(Sim.sky_info.finder_radius_deg), dim, _fs - 2)
	else:
		_line("FIELD 30' x 30' (ZOOM %dx)" % int(ZOOMS[zoom_idx]), dim, _fs - 2)
		_line("HINT: BLINK THE SHOTS - THE STARS STAY PUT,", dim, _fs - 2)
		_line("ONE FAINT DOT JUMPS. THAT IS THE ASTEROID.", dim, _fs - 2)
	_py += 6.0
	_line("GAME CHOICES: THE FIELD SIZE, AND THE POINTING -", faint, _fs - 3)
	_line("AIMED AT THE TRUE POSITION, WHICH NO OBSERVER", faint, _fs - 3)
	_line("OF A NEW OBJECT CAN DO.", faint, _fs - 3)
	_line("ASTEROID DRAWN WITH MT. LEMMON'S MEASURING ERROR", faint, _fs - 3)
	_line("(%.2f\" / %.2f\", VERES ET AL. 2017)" % [float(Sim.sky_info.sigma_ra_arcsec),
		float(Sim.sky_info.sigma_dec_arcsec)], faint, _fs - 3)
	_line("STARS: TYCHO-2 (HOG ET AL. 2000)  NAMES: CDS IV/27A", faint, _fs - 3)
	_line("POSITIONS: JPL HORIZONS (APOPHIS), DE440", faint, _fs - 3)
	_py += 6.0
	_line("[</>]SHOT [UP/DN]ZOOM [B]BLINK [O]STACK [L]NAMES", dim, _fs - 2)


## One panel line at the running cursor, which then moves down.
func _line(s: String, col: Color, fs: int = -1) -> void:
	var size_px := _fs if fs < 0 else fs
	_t(Vector2(_px, _py), s, col, size_px)
	_py += float(size_px) + 6.0


func _faintest_star(stars: PackedFloat64Array) -> float:
	var f := -INF
	for j in range(stars.size() / 4):
		f = maxf(f, stars[4 * j + 2])
	return f


func _t(pos: Vector2, s: String, col: Color, fs: int = -1) -> void:
	draw_string(_font, pos, s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs if fs < 0 else fs, col)


func _centered(s: String, at: Vector2, col: Color, fs: int) -> void:
	var wdt := _font.get_string_size(s, HORIZONTAL_ALIGNMENT_LEFT, -1, fs).x
	draw_string(_font, at - Vector2(wdt * 0.5, 0.0), s, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, col)
