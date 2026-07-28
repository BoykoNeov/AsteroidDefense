class_name ThreatPanel
extends Control
## Threat-orbit designer ([T]): four knobs that put the rock on a different
## heliocentric orbit, a free closed-form preview of what they produce, and the
## two expensive operations that make it real. Pure display — key handling lives
## in main.gd, state and physics calls in Sim.
##
## # Why this panel is split in two halves
##
## Every other panel in this project is either free (the tractor bench's
## arithmetic) or fires one on-demand solve ([E], [M]). This one is both, because
## a threat orbit costs ~10 s to build and can be refused for two different
## reasons. A panel with no preview would be four knobs whose only feedback is a
## ten-second wait followed by an error message.
##
## So: turning a knob re-previews in **closed form** — exact for the encounter
## geometry, 0.23% for the orbit — and [R] spends the rebuild. The preview finds
## both walls in microseconds that the builder finds in ten seconds.
##
## # The two walls, which close from opposite directions
##
##     TOO SLOW     v_rel must clear sqrt(2*mu/offset) -- Earth escape AT THE
##                  OFFSET. 16.3 km/s at 3000 km against a shipping 18, and
##                  SHRINKING the offset raises the bar (28.2 km/s at 1000 km).
##                  Pulling the hit toward Earth's centre is what falls off.
##
##     NOT A HIT    the offset IS the geocentric perigee, because it is laid
##                  perpendicular to the relative velocity. So its ceiling is
##                  exactly Earth's radius -- and the b-plane miss it produces
##                  is offset * v_rel / v_inf, over twice as large (7077 km for
##                  the shipping 3000).
##
## # What the operator is meant to learn here
##
## That the orbit is not scenery. Measured across two orbits this panel can
## reach: the same 200 t tractor plan over the same 6.0 yr lead scores **0.372x
## on the shipping orbit and 1.096x on a long-period one** — fails and closes,
## from one knob. The requirement itself moves ~10x, and notably *not* by the
## 3.4x the period ratio alone would predict, which is why [A] solves rather than
## scales.
##
## # The required-Δv anchor has three states, and the middle one is new
##
##     SHIPPING     seeded free from a recorded constant; the margin prints
##     UNMEASURED   after a rebuild -- no requirement, no margin, and [E] fixes it
##     SOLVING      about a minute, shown running because a still panel that long
##                  reads as a hang
##
## Measured 28.8 s (shipping orbit), then 41 / 63 / 74 s on a 1.44 yr one across
## three runs. The copy says "about a minute" rather than a range, because the cost
## **scales with the period** — one period of lead on a longer orbit is a longer
## propagation — and the knobs reach past 3 yr. A stated range would be a promise
## the solver can already exceed; the first draft said "30-60 s" and a run took 63.
##
## # Keys
##
## One new input action ([N] to open). The two operations reuse keys whose meaning
## already matches rather than minting more: [ENTER] is `plan_commit` — apply what
## is dialled — and [E] is `pork_verify`, which already means "stop estimating and
## go measure it in the full field" in both the launch-window map and the tractor
## bench. Solving this orbit's required Δv is exactly that.
##
## The middle state is the one this panel exists to make visible. Everything else
## about a rebuilt scenario works, so a margin that kept quoting the *previous*
## orbit's requirement would look entirely healthy — and be a number about a
## different mission.

const W := 620.0
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
	var rows := 18.0
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
	var xv := x + 17.0 * _fs * 0.62
	var y := origin.y + MARGIN + _fs

	_t(Vector2(x, y), "THREAT ORBIT - IMPACTOR DESIGN", bright)
	y += lh
	_t(Vector2(x, y), "-".repeat(68), faint)
	y += lh

	if not Sim.mission_online:
		_t(Vector2(x, y), "AWAITING THREAT SOLUTION", mid)
		return

	# ---- the knobs -------------------------------------------------------
	# Drawn straight off Sim.THREAT_KNOBS, same as the tractor bench: a fifth
	# knob appears here by existing, with no edit to this function.
	for i in Sim.THREAT_KNOBS.size():
		var knob: Array = Sim.THREAT_KNOBS[i]
		var selected: bool = i == Sim.threat_row
		_t(Vector2(x, y), ("> " if selected else "  ") + str(knob[1]),
			bright if selected else dim)
		_t(Vector2(xv, y), _knob_value(str(knob[0])), bright if selected else mid)
		y += lh

	_t(Vector2(x, y), "-".repeat(68), faint)
	y += lh

	# ---- the free preview ------------------------------------------------
	var p := Sim.threat_preview()
	if not bool(p.get("ok", false)):
		# The hyperbolic wall. There is genuinely no v_inf and no b-plane here, so
		# unlike a miss there are no numbers to show — only the reason and the way
		# out, which the core's message names.
		_t(Vector2(x, y), "GEOMETRY", dim)
		_t(Vector2(xv, y), "NOT A FLYBY - RAISE SPEED OR OFFSET", bright)
		y += lh
		# **Wrapped, and the tail preferred over the head.** A `.left(46)` here cut
		# the message at "...is not a hyperbolic flyby: rel" — throwing away the one
		# number that says how far the knob has to move ("Earth escape there is
		# 63.135 km/s"). The core writes that clause specifically for this panel, so
		# truncating it away made the detailed message pointless. There is a whole
		# panel of vertical room below this branch; use it.
		for line in _wrap(_detail(str(p.get("error", ""))), 62):
			_t(Vector2(x + 16.0, y), line, faint)
			y += lh
		return

	_t(Vector2(x, y), "V-INFINITY", dim)
	# The distinction this row exists for: the knob is the speed at the impact
	# point, deep in Earth's well. This is what is left after climbing out, and it
	# is what sets the capture disc below.
	_t(Vector2(xv, y), "%.3f KM/S  (KNOB IS %.1f AT THE IMPACT POINT)" %
		[float(p.v_inf_m_s) / 1000.0, Sim.threat_knobs.v_rel], mid)
	y += lh

	_t(Vector2(x, y), "B-PLANE MISS", dim)
	var b_km := float(p.impact_parameter_m) / 1000.0
	var cap_km := float(p.capture_radius_m) / 1000.0
	var is_hit: bool = bool(p.is_hit)
	# `b` against `b_capture` — the pair the core's own hit test uses. The offset
	# knob is NOT this number and printing them together is the point: focusing
	# widens the asymptote's miss to over twice the aim point.
	_t(Vector2(xv, y), "%.0f KM  VS CAPTURE %.0f KM  %s" %
		[b_km, cap_km, "[HIT]" if is_hit else "[MISSES EARTH]"],
		mid if is_hit else bright)
	y += lh

	_t(Vector2(x, y), "ORBIT", dim)
	var yr := float(p.period_seconds) / (365.25 * 86400.0)
	_t(Vector2(xv, y), "A %.3f AU  E %.3f  I %.1f DEG  T %.3f YR" %
		[float(p.semi_major_axis_m) / (Sim.AU_KM * 1000.0),
			float(p.eccentricity), float(p.inclination_deg), yr], mid)
	y += lh
	# The one caveat that must travel with the orbit line. Everything above it is
	# exact; this line is osculating at the impact epoch against a build that
	# reports vis-viva at the seed twelve years earlier.
	_t(Vector2(xv, y), "(CLOSED-FORM ESTIMATE, ~0.2 PCT - REBUILD TO MEASURE)", faint)
	y += lh
	_t(Vector2(x, y), "-".repeat(68), faint)
	y += lh

	# ---- what is actually installed --------------------------------------
	_t(Vector2(x, y), "INSTALLED", dim)
	if Sim.threat_rebuilding:
		_t(Vector2(xv, y), "REBUILDING CAMPAIGN - BACK-PROPAGATING SEED..."
			if Sim.blink(1.6) else "", bright)
	elif not is_hit:
		# DEFENSIVE, and unreachable through this UI — the same relationship the
		# bench's `holds_station` branch has to its hover clamp. `is_hit` is
		# `perigee <= R_E`, the perigee IS the offset, and the offset knob clamps
		# at exactly Earth's radius: at the ceiling `b == b_capture` to nine
		# digits, so the knob's last position is the widest impact there is and
		# there is no next one. The branch stays because the *binding* accepts any
		# offset, and a caller past the clamp must not get a ten-second build that
		# ends in a raw error.
		_t(Vector2(xv, y), "CANNOT REBUILD - NO IMPACT TO DEFLECT", bright)
	elif Sim.threat_knobs_are_installed():
		_t(Vector2(xv, y), "THESE KNOBS - THREAT IS ON THIS ORBIT", mid)
	else:
		# The staleness marker, and it is read off the *core*, not off a flag set
		# when [ENTER] was pressed — so a rebuild that failed leaves this showing
		# "not applied" rather than claiming success.
		_t(Vector2(xv, y), "DIFFERENT ORBIT - [ENTER] TO REBUILD (~10 S)", bright)
	y += lh

	# ---- and the requirement that has to move with the orbit -------------
	_t(Vector2(x, y), "REQUIRED DV", dim)
	if Sim.threat_anchor_solving:
		_t(Vector2(xv, y), "SOLVING FOR THIS ORBIT - ABOUT A MINUTE..."
			if Sim.blink(1.6) else "", bright)
	elif Sim.threat_anchor_known():
		# **Say whose requirement this is.** Every row above describes the orbit on
		# the knobs; this one describes the orbit that is *installed*, and while
		# those differ the two sit adjacent with nothing marking the change of
		# subject. A reader who takes 0.5098 as the dialled orbit's requirement has
		# been misled by layout alone — which is the same class of error as the
		# margin quoting a replaced orbit, just committed by the panel instead of
		# by the physics.
		var whose := "" if Sim.threat_knobs_are_installed() else "  (INSTALLED)"
		_t(Vector2(xv, y), "%.4f M/S AT ONE ORBIT OF LEAD%s" %
			[Sim.mission.required_dv_anchor(), whose], mid)
	else:
		# The state this panel exists to make visible. Not an error and not a
		# failure — simply a question nobody has paid to answer yet, and the
		# tractor bench's margin is absent for exactly this window.
		_t(Vector2(xv, y), "UNMEASURED ON THIS ORBIT - [E] TO SOLVE (~1 MIN)", bright)
	y += lh
	_t(Vector2(x, y), "-".repeat(68), faint)
	y += lh
	_t(Vector2(x, y),
		"[LEFT/RIGHT] ADJUST  [UP/DOWN] SELECT  [ENTER] REBUILD  [E] SOLVE REQ DV  [N] CLOSE", dim)


## One knob's value, formatted in its own unit.
func _knob_value(id: String) -> String:
	match id:
		"v_rel":
			return "< %6.1f KM/S >" % Sim.threat_knobs.v_rel
		"az":
			return "< %+7.1f DEG >  (IN THE ECLIPTIC)" % Sim.threat_knobs.az
		"el":
			return "< %+7.1f DEG >  (OUT OF THE ECLIPTIC)" % Sim.threat_knobs.el
		"offset":
			# Say what it really is. It reads like a b-plane number and is not one:
			# the offset is perpendicular to the relative velocity, so it IS the
			# geocentric perigee — which is why its ceiling is Earth's radius and
			# not something larger, and why the b-plane miss above is bigger.
			# At the ceiling the asymptote grazes the capture disc exactly
			# (b == b_capture), so this is the widest impact that exists — worth
			# saying, because a knob that stops with no explanation looks broken
			# and here the reason it stops is the interesting part.
			var at_max: bool = Sim.threat_knobs.offset >= Sim.threat_offset_max - 1e-6
			return "< %6.0f KM >  (= PERIGEE)%s" % \
				[Sim.threat_knobs.offset,
					"  MAX (EARTH RADIUS - GRAZING HIT)" if at_max else ""]
	return "--"


## The operator-facing half of a core error message.
##
## These read "<what went wrong> — <what to do about it>", and the second half is
## the one written for a reader at a knob. Falls back to the whole string when
## there is no separator, so a message from somewhere else is never swallowed.
func _detail(msg: String) -> String:
	var i := msg.rfind(" — ")
	return msg.substr(i + 3) if i >= 0 else msg


## Break `s` into lines of at most `width` characters, on spaces. The font is
## monospace, so a character count is a width.
func _wrap(s: String, width: int) -> Array[String]:
	var out: Array[String] = []
	var line := ""
	for word in s.split(" ", false):
		if line.is_empty():
			line = word
		elif line.length() + 1 + word.length() <= width:
			line += " " + word
		else:
			out.append(line)
			line = word
	if not line.is_empty():
		out.append(line)
	return out


func _t(pos: Vector2, s: String, col: Color) -> void:
	draw_string(_font, pos, s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs, col)
