class_name TractorPanel
extends Control
## Gravity-tractor bench ([K]): six knobs, a live scoring of what they buy, and
## an on-demand full-field probe. Pure display — key handling lives in main.gd,
## state and physics calls in Sim.
##
## # What this view is for
##
## The tractor is the *gentle* end of the deflection spectrum, and on the
## campaign's own rock it does not close: a Lu & Love-class 20 t spacecraft
## towing for the entire 6.32 yr lead delivers about a twelfth of the Δv the
## curve asks for. That single result is a dead end to read and an interesting
## one to *operate*, because the reason it fails is a scale, not a physics — and
## the levers that change the scale are all cheap to evaluate:
##
##     spacecraft mass   linear          the obvious one, and the least interesting
##     hover distance    1/d^2           bounded below by the PLUME WALL, not the surface
##     rock radius       1/r^2           at fixed d/r; the required Δv does NOT move
##     tow start         lead^2          delivered rises with lead, required falls as 1/lead
##     tow duration      -               with diminishing returns: late Δv buys less
##
## The lead row is the one worth finding: because the requirement falls as
## `1/lead` while the delivered Δv rises *with* lead, tractor effectiveness goes
## as **lead squared**. Doubling the warning time is four times the tractor.
##
## # The direction row is the sharpest lesson in the panel
##
## Measured on the shipping configuration, one keypress apart:
##
##     PROGRADE     perigee 3000 -> 2811 km   (-188 km, DEEPER)
##     RETROGRADE   perigee 3000 -> 3348 km   (+348 km, OUTWARD)
##
## Not a symmetric sign flip — the retrograde move is nearly twice as large. The
## nominal is a near-centre hit, so the b-plane point sits ~3000 km off Earth's
## centre: tugging one way walks it *toward* the centre (perigee dips before it
## can ever come back out), tugging the other walks it straight away (perigee
## grows from the first day). The same 20-tonne spacecraft therefore either makes
## the impact worse or makes it better, and nothing about the *tow* changed —
## only which side of the planet the rock is being nudged toward.
##
## **The panel opens on PROGRADE deliberately.** It is the configuration the
## campaign measured and documented, it is the one that fails, and it is one
## keypress from the one that helps. Seeding on the flattering direction would
## hide the whole point.
##
## # Two numbers, and only one of them is measured
##
## Everything above the rule is arithmetic — free, and answerable while a key is
## held. The perigee below it is a real n-body propagation costing ~12 s, fired by
## [E]. The panel keeps them visually apart and greys the probe the instant a knob
## moves off it, because the failure mode here is not a wrong number, it is a
## *stale measured* number sitting where a live one appears to be.

const W := 560.0
const MARGIN := 12.0

var _font: Font
var _fs := 15


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	visible = false
	_font = Sim.mono_font


func _process(_delta: float) -> void:
	if visible:
		queue_redraw()


func _draw() -> void:
	var lh := _fs + 6.0
	var rows := 19.0
	var ph := rows * lh + 2.0 * MARGIN + 4.0
	var origin := Vector2(size.x * 0.5 - W * 0.5, size.y - ph - 60.0)
	var bright := Color(1, 1, 1)
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.42, 0.42, 0.42)
	var faint := Color(0.25, 0.25, 0.25)

	var rect := Rect2(origin, Vector2(W, ph))
	draw_rect(rect, Color(0, 0, 0, 0.88), true)
	draw_rect(rect, mid, false, 1.2)
	var x := origin.x + MARGIN
	var xv := x + 15.0 * _fs * 0.62
	var y := origin.y + MARGIN + _fs

	_t(Vector2(x, y), "GRAVITY TRACTOR - STATION-KEEPING TOW", bright)
	y += lh
	_t(Vector2(x, y), "-".repeat(62), faint)
	y += lh

	var r := Sim.tractor_readout()

	# ---- the knobs -------------------------------------------------------
	# Drawn straight off Sim.TRACTOR_KNOBS so a new parameter appears here by
	# existing, with no edit to this function.
	for i in Sim.TRACTOR_KNOBS.size():
		var knob: Array = Sim.TRACTOR_KNOBS[i]
		var selected: bool = i == Sim.tractor_row
		var label_col: Color = bright if selected else dim
		var value_col: Color = bright if selected else mid
		_t(Vector2(x, y), ("> " if selected else "  ") + str(knob[1]), label_col)
		_t(Vector2(xv, y), _knob_value(str(knob[0]), r), value_col)
		y += lh

	_t(Vector2(x, y), "-".repeat(62), faint)
	y += lh

	if r.is_empty():
		_t(Vector2(x, y), "AWAITING THREAT SOLUTION", mid)
		return

	# ---- what the configuration is ---------------------------------------
	# The tow and the thrust are the two halves of Lu & Love's bookkeeping and
	# they pull in opposite directions: hovering closer tugs harder AND costs
	# more thrust, and the cant angle is why. Shown together so the trade is
	# visible rather than discovered.
	var flyable: bool = bool(r.flyable)
	if not flyable:
		# The geometric wall. `1/d^2` alone never reveals it — the arithmetic is
		# perfectly happy inside the asteroid — so the panel has to say it.
		_t(Vector2(x, y), "GEOMETRY", dim)
		_t(Vector2(xv, y), "SPACECRAFT INSIDE THE BODY - RAISE HOVER", bright)
		return
	_t(Vector2(x, y), "TOW ACCEL", dim)
	_t(Vector2(xv, y), "%s M/S2   (%.3f MM/S/YR)" %
		[_sci(float(r.tow_accel_m_s2), 3),
			float(r.tow_accel_m_s2) * 365.25 * 86400.0 * 1000.0], mid)
	y += lh
	_t(Vector2(x, y), "STATION-KEEP", dim)
	# `holds_station` is its own flag, never `thrust_n == 0`. Between the surface
	# and 1/cos(plume) the cant has passed 90 deg and no thrust holds the
	# spacecraft there — while the tow above is perfectly real. So the row must
	# say "impossible" rather than print the 0.000 N a missing value formats to,
	# which would read as station-keeping being FREE at exactly the distance where
	# it cannot be done at all.
	if bool(r.holds_station):
		# The thrust spans 1 N at a comfortable hover to ~3e15 N a hair above the
		# plume wall, because `cos(cant)` is heading for zero there. `%.3f` would
		# render that as a 16-digit run of characters that reads as a rendering
		# fault rather than as the divergence it is.
		var t_n := float(r.thrust_n)
		var t_txt: String = ("%.3f" % t_n) if t_n < 1.0e6 else _sci(t_n, 2)
		_t(Vector2(xv, y), "%s N THRUST  CANT %.1f DEG" % [t_txt, r.cant_deg], mid)
	else:
		# DEFENSIVE, and honestly unreached through this UI: the hover knob clamps
		# at exactly `1/cos(plume)`, where the cant is 90.0 deg but its cosine is
		# still a hair above zero, so `holds_station` is true at the tightest
		# setting a user can select. The core's own test exercises the `None` path
		# directly at 1.02 radii. This branch exists so that a future knob bound,
		# a wider plume, or a caller that sets the value outside the clamp cannot
		# turn a missing thrust into a printed 0.000 N.
		_t(Vector2(xv, y), "NO SOLUTION - CANT PAST 90 DEG (TOW IS REAL)", bright)
	y += lh
	_t(Vector2(x, y), "ROCK MASS", dim)
	_t(Vector2(xv, y), "%s KG" % _sci(float(r.rock_mass_kg), 3), mid)
	y += lh
	_t(Vector2(x, y), "-".repeat(62), faint)
	y += lh

	# ---- the estimate ----------------------------------------------------
	# DELIVERED is `a*T` and is labelled a bound, not a result. It overstates
	# what a tow is worth by up to 2x (measured +21% on the one configuration
	# with a real-field answer) because a tug spread over the lead arrives later
	# on average than an impulse at its start. EFFECTIVE is the impulsive
	# equivalent, and it is the one the margin is formed from.
	_t(Vector2(x, y), "DELIVERED", dim)
	_t(Vector2(xv, y), "%.4f MM/S  (UPPER BOUND)" %
		(float(r.delivered_dv_m_s) * 1000.0), mid)
	y += lh
	_t(Vector2(x, y), "EFFECTIVE", dim)
	_t(Vector2(xv, y), "%.4f MM/S  IMPULSE-EQUIVALENT" %
		(float(r.equivalent_dv_m_s) * 1000.0), mid)
	y += lh
	_t(Vector2(x, y), "REQUIRED", dim)
	if r.has("required_dv_m_s"):
		_t(Vector2(xv, y), "%.4f MM/S  (EST, 1/LEAD LAW)" %
			(float(r.required_dv_m_s) * 1000.0), mid)
	else:
		# Below one orbit the `1/lead` law is not imprecise, it is the wrong
		# shape — measured 1.73x wrong at half a period. Declining to print is
		# the honest option; a number here would be the display-grade lie.
		_t(Vector2(xv, y), "LEAD BELOW %.2f ORB - LAW DOES NOT HOLD" %
			Sim.tractor_law_min_periods, dim)
	y += lh
	_t(Vector2(x, y), "MARGIN", dim)
	_t(Vector2(xv, y), _margin_label(r), bright)
	y += lh
	_t(Vector2(x, y), "-".repeat(62), faint)
	y += lh

	# ---- and the one measured number -------------------------------------
	_t(Vector2(x, y), "FULL FIELD", dim)
	var current: bool = Sim.tractor_probe_is_current()
	if Sim.tractor_probing:
		_t(Vector2(xv, y), "TOWING - FULL N-BODY PROPAGATION..." if Sim.blink(1.6) else "", bright)
	elif Sim.tractor_probe().is_empty():
		_t(Vector2(xv, y), "NOT PROBED", dim)
	else:
		# A probe for knobs the operator has moved off is dimmed, never hidden:
		# it is still a true measurement, just not of what is on screen now.
		_t(Vector2(xv, y), Sim.tractor_probe_label(), bright if current else faint)
	y += lh
	if not current and not Sim.tractor_probe().is_empty() and not Sim.tractor_probing:
		_t(Vector2(xv, y), "(FOR EARLIER SETTINGS - RE-PROBE)", faint)
	y += lh
	# The adjust hint lives here rather than right-aligned on the selected row.
	# It was on the row, and the hover value at the plume wall is long enough to
	# run straight into it — the two strings overprinted into "MINEFTPEUMETWALL",
	# which looks like a font fault rather than two labels sharing pixels.
	_t(Vector2(x, y), "[LEFT/RIGHT] ADJUST  [UP/DOWN] SELECT  [E] PROBE  [K] CLOSE", dim)


## One knob's value, formatted in its own unit. The `r` readout is passed in so
## the lead row can show what the lead means in years without a second call.
func _knob_value(id: String, r: Dictionary) -> String:
	match id:
		"mass":
			return "< %7.1f T >" % Sim.tractor.mass
		"hover":
			# Both the ratio and the metres: the ratio is the physics (the tow
			# goes as 1/d^2 at fixed r) and the metres are what a reader pictures.
			# At the floor, say so. A knob that stops moving with no explanation
			# looks broken, and here the reason it stops is the interesting part:
			# the plume wall at 1/cos(phi), not the asteroid's surface.
			var at_floor: bool = Sim.tractor.hover <= Sim.tractor_hover_min + 1e-6
			return "< %6.3f R >  (%.0f M FROM CENTRE)%s" % \
				[Sim.tractor.hover, Sim.tractor.hover * Sim.tractor.radius,
					"  MIN (PLUME WALL)" if at_floor else ""]
		"radius":
			return "< %6.0f M >  (%.0f M ACROSS)" % \
				[Sim.tractor.radius, 2.0 * Sim.tractor.radius]
		"lead":
			var yr := Sim.tractor_lead_s() / (365.25 * 86400.0)
			return "< %6.2f ORB >  (%.2f YR)" % [Sim.tractor.lead, yr]
		"duty":
			var yr2 := Sim.tractor_duration_s() / (365.25 * 86400.0)
			# "PCT", not "%%": GDScript's `%` operator has no escape for a literal
			# percent sign. The malformed format does not raise — it errors once
			# per frame from inside `_draw` and puts an error string on the panel
			# where a number belongs, which reads as a physics fault.
			return "< %5.0f PCT >  (%.2f YR OF TOWING)" % [Sim.tractor.duty, yr2]
		"dir":
			return "< %s >" % ("RETROGRADE" if Sim.tractor.dir > 0.5 else "PROGRADE")
	return "--"


## The margin, formatted — and never formed from the delivered Δv.
##
## Absent below the law's validity floor rather than guessed: a margin computed
## from a requirement that does not apply is worse than no margin, because it
## looks exactly like one that does.
func _margin_label(r: Dictionary) -> String:
	if not r.has("margin"):
		return "-- (NO VALID REQUIREMENT AT THIS LEAD)"
	var m := float(r.margin)
	if m >= 1.0:
		return "%.2fx REQUIRED - CLOSES (EST)" % m
	if m <= 0.0:
		return "NO TOW"
	return "%.3fx REQUIRED - SHORT BY %.1fx (EST)" % [m, 1.0 / m]


## Scientific notation, because **GDScript's `%` operator has no `%e`**.
##
## And it does not raise on one either: `"%.3e" % x` fails, returns nothing
## useful, and logs a script error — so a `_draw` written with it printed an error
## string where a number belonged, sixty times a second, while the panel itself
## still looked broadly plausible. The values here span 1e-11 m/s^2 to 1e13 kg, so
## plain `%f` is not an option and this is not avoidable by rounding.
func _sci(v: float, digits: int) -> String:
	# `v == 0.0`, NOT `is_zero_approx(v)`. Godot's epsilon there is 1e-5, and every
	# tow acceleration on this panel is ~1e-11 — so the approximate test called the
	# most important number in the view "zero" and drew `TOW ACCEL 0 M/S2` beside a
	# perfectly correct 0.832 mm/s/yr. Nothing in the headless run noticed; the
	# screenshot did. A helper for small magnitudes must not carry a
	# large-magnitude notion of small.
	if not is_finite(v) or v == 0.0:
		return "0"
	var e := int(floor(log(absf(v)) / log(10.0)))
	var mant := v / pow(10.0, e)
	return ("%." + str(digits) + "fE%+03d") % [mant, e]


func _t(pos: Vector2, s: String, col: Color) -> void:
	draw_string(_font, pos, s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)

