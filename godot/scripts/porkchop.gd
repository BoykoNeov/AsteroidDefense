class_name PorkchopPlot
extends Control
## The launch-window map ([4]) — the classic porkchop plot, read from the core.
##
## Every cell is one (launch date, arrival date) pair, and its brightness is what
## that window costs or delivers. This is the view that makes the campaign's
## central claim *honest*: the planner's "spend 0.2 m/s twelve years out" assumes
## an impulse can be delivered, and here is the map of whether it can — which
## launcher, through which window, carrying how much mass.
##
## **Three kinds of empty, drawn three different ways**, because collapsing any
## pair throws away something the operator needs:
##
## - **no transfer at all** (background) — no trajectory connects those two dates
##   at any allowed lap count. Below the diagonal this is simply "arrival before
##   launch"; elsewhere it is a Lambert gap.
## - **unreachable** (dim hatch) — a real trajectory exists, but *this launcher*
##   cannot make that launch energy. Press [L] and the same trajectory may become
##   reachable. That is the whole payoff of a vehicle-independent grid, and it is
##   invisible if this is drawn like the background.
## - **reachable** (bright) — a mission exists here.
##
## **Everything on this plot is a patched-conic planning estimate.** The Lambert
## arc sizes and aims the delivery; it does not decide hit or miss. Exactly two
## lines in this view come from the real n-body field, and both are labelled: the
## [E] verdict (does this launcher clear the capture disc through this window) and
## the [M] requirement (what mass would). They are the question and its follow-up —
## [E] can say no, only [M] can say by how much.
##
## Pure display: it owns no orbital mechanics. The grid columns, the cursor
## readout and the verdict all arrive from `Sim`, which marshals them from the
## core. Key handling lives in main.gd.

const MARGIN := 64.0
## Clearance above the plot for the HUD's header/clock block.
const TOP_RESERVE := 96.0
## Width of the colour-key column down the plot's right edge.
const LEGEND_W := 168.0
## Height reserved at the bottom for the cursor readout.
const PANEL_H := 238.0
## Where the cursor readout's left edge sits, as a fraction of the view width.
##
## Read by **two** files: this one places the panel here, and `hud.gd` budgets the
## event-log column against it — the log runs the full width in every other view,
## and in this one the panel is sitting in the half it would otherwise use. Named
## once because the alternative is the same literal in two places and a console line
## printing over the readout the first time a message gets long, which is exactly
## what happened.
const PANEL_X_FRACTION := 0.44

var _font: Font
var _fs := 13

# The heatmap, baked into a texture one cell per texel, plus the ramp bounds and
# the reachable-cell count. All rebuilt only when the grid or the launcher changes
# (`porkchop_changed`) — never per frame.
#
# A texture rather than a `draw_rect` per cell: the shipping grid is 120x120, and
# 14 400 draw calls *every frame* (`_process` queues a redraw while visible) is a
# real cost for a picture that changes only on a keypress. One texel per cell with
# nearest filtering gives the same crisp blocks for one draw call.
var _built := false
var _tex: ImageTexture
var _lo := 0.0
var _hi := 1.0
var _reachable := 0


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_preset(Control.PRESET_FULL_RECT)
	# Cells are data, not artwork: a smoothed heatmap would invent gradients between
	# windows that were never solved.
	texture_filter = CanvasItem.TEXTURE_FILTER_NEAREST
	_font = Sim.mono_font
	Sim.porkchop_changed.connect(func() -> void: _built = false)
	Sim.mission_ready.connect(func() -> void: _built = false)


func _process(_delta: float) -> void:
	if visible:
		queue_redraw()


# ------------------------------------------------------------------ ramp ---

## The scalar this cell is coloured by, or NAN for a cell with no transfer.
##
## Metric "dv" reads the **magnitude** of the delivered along-track Δv: the sign
## says which way the push moves the semi-major axis (negative = a retrograde,
## orbit-shrinking push), and that is a real lever rather than bad aim, so it
## belongs in the readout and not in the brightness.
func _metric_of(k: int) -> float:
	if Sim.pork_c3[k] < 0.0:
		return NAN
	if Sim.PORK_METRICS[Sim.pork_metric][0] == "c3":
		return Sim.pork_c3[k]
	return absf(Sim.pork_dv[k])


## Recompute the colour ramp's bounds over the cells that have a transfer.
##
## The C3 ramp is pinned to the **launcher's own tabulated ceiling**, not to the
## grid's maximum: the question a reader is asking is "can this rocket fly it",
## and a ramp normalized by the most absurd cell in the grid would paint every
## flyable window the same near-black. The Δv ramp has no such external bar, so it
## normalizes to the grid.
func _rebuild_ramp() -> void:
	if Sim.PORK_METRICS[Sim.pork_metric][0] == "c3":
		_lo = 0.0
		_hi = maxf(Sim.pork_vehicle_max_c3(), 1.0)
		return
	_lo = 0.0
	_hi = 0.0
	for k in range(Sim.pork_c3.size()):
		var v := _metric_of(k)
		if not is_nan(v):
			_hi = maxf(_hi, v)
	if _hi <= 0.0:
		_hi = 1.0


## Bake the grid into a texture: one texel per window, in row-major order, so the
## image's x is arrival and its y is launch — the plot's own orientation.
##
## The three cell states get three distinguishable fills, and that separation is
## the point of the picture (see the class note). "Unreachable" in particular must
## not read as background: the shape of what *this launcher* can fly is only
## legible against the shape of what exists at all.
func _rebuild() -> void:
	_built = true
	_rebuild_ramp()
	var img := Image.create(Sim.pork_cols, Sim.pork_rows, false, Image.FORMAT_RGB8)
	var no_transfer := Color(0.012, 0.016, 0.014)
	var unreachable := Color(0.075, 0.075, 0.075)
	_reachable = 0
	for i in range(Sim.pork_rows):
		for j in range(Sim.pork_cols):
			var k := i * Sim.pork_cols + j
			var col := no_transfer
			if Sim.pork_c3[k] >= 0.0:
				if Sim.pork_payload[k] > 0.0:
					var s := _shade(_metric_of(k))
					col = Color(s, s, s)
					_reachable += 1
				else:
					col = unreachable
			img.set_pixel(j, i, col)
	_tex = ImageTexture.create_from_image(img)


## Brightness for a metric value: bright = good (cheap C3 / large Δv).
func _shade(v: float) -> float:
	var f := clampf((v - _lo) / (_hi - _lo), 0.0, 1.0)
	if Sim.PORK_METRICS[Sim.pork_metric][0] == "c3":
		f = 1.0 - f          # cheap is bright
	# A gamma bend: most of the interesting structure is at the cheap end, and a
	# linear ramp buries it.
	return pow(f, 0.6)


# ------------------------------------------------------------------ draw ---

func _draw() -> void:
	var w := size.x
	var h := size.y
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.004, 0.006, 0.005), true)

	var bright := Color(1, 1, 1)
	var mid := Color(0.72, 0.72, 0.72)
	var dim := Color(0.42, 0.42, 0.42)
	var faint := Color(0.18, 0.18, 0.18)

	# Nothing measured yet is said plainly. A blank grid presented as a result
	# would read as "no windows exist", which is the opposite of "not solved".
	if not Sim.mission_online:
		_centered("LAUNCH-WINDOW MAP OFFLINE", Vector2(w * 0.5, h * 0.5), dim, _fs)
		_centered("AWAITING THREAT SOLUTION", Vector2(w * 0.5, h * 0.5 + 18.0), faint, _fs - 2)
		return
	if not Sim.pork_online:
		var msg := "PRESS [4] AGAIN TO SOLVE THE LAUNCH-WINDOW GRID"
		if Sim.pork_building:
			msg = "SOLVING %d x %d LAMBERT TRANSFERS ..." % \
				[Sim.PORK_LAUNCH_SAMPLES, Sim.PORK_ARRIVAL_SAMPLES]
		_centered(msg, Vector2(w * 0.5, h * 0.5), dim, _fs)
		return
	if not _built:
		_rebuild()

	# Laid out around the HUD chrome that stays up in this view: the clock block
	# along the top, and the event console down the bottom-left. The plot takes the
	# upper band, the colour key its right margin, and the cursor readout the
	# bottom-*right* — beside the console rather than under the plot, which is
	# where it would land on top of it.
	var plot := Rect2(Vector2(MARGIN, TOP_RESERVE), Vector2(w - MARGIN - LEGEND_W, h * 0.56))
	_draw_cells(plot)
	_draw_axes(plot, mid, dim, faint)
	_draw_cursor(plot, bright)
	_draw_legend(plot, mid, dim, faint)
	_draw_panel(Vector2(w * PANEL_X_FRACTION, h - PANEL_H), bright, mid, dim, faint)


## The heatmap proper: x = arrival date, y = launch date (the two axes the core
## hands us). Time of flight is then the diagonal distance and reads straight off
## the plot, and the "arrival at or before launch" region falls out as the blank
## wedge — the orientation the porkchop literature uses.
func _draw_cells(plot: Rect2) -> void:
	if _tex != null:
		draw_texture_rect(_tex, plot, false)


func _draw_axes(plot: Rect2, mid: Color, dim: Color, faint: Color) -> void:
	draw_rect(plot, faint, false, 1.0)
	# Four date ticks per axis, read from the core's own epoch arrays.
	for n in range(4):
		var f := n / 3.0
		var x := plot.position.x + plot.size.x * f
		var y := plot.position.y + plot.size.y * f
		var aj := int(round(f * (Sim.pork_cols - 1)))
		var li := int(round(f * (Sim.pork_rows - 1)))
		draw_line(Vector2(x, plot.end.y), Vector2(x, plot.end.y + 5.0), faint, 1.0)
		draw_line(Vector2(plot.position.x - 5.0, y), Vector2(plot.position.x, y), faint, 1.0)
		_t(Vector2(x - 26.0, plot.end.y + 17.0), _date_of(Sim.pork_arrival_tdb, aj), dim, _fs - 3)
		_t(Vector2(4.0, y + 4.0), _date_of(Sim.pork_launch_tdb, li), dim, _fs - 3)
	_t(Vector2(plot.position.x, plot.position.y - 12.0), "ARRIVAL / INTERCEPT DATE ->", mid, _fs - 2)
	_t(Vector2(4.0, plot.position.y - 12.0), "LAUNCH", mid, _fs - 2)


## Crosshair on the selected cell, drawn as lines to the axes so its dates are
## readable without hunting for the marker.
func _draw_cursor(plot: Rect2, bright: Color) -> void:
	var cw := plot.size.x / float(Sim.pork_cols)
	var ch := plot.size.y / float(Sim.pork_rows)
	var p := plot.position + Vector2(Sim.pork_j * cw, Sim.pork_i * ch)
	var r := Rect2(p - Vector2(1, 1), Vector2(cw + 2.0, ch + 2.0))
	draw_rect(r, bright, false, 1.4)
	var c := p + Vector2(cw, ch) * 0.5
	draw_line(Vector2(plot.position.x, c.y), Vector2(r.position.x, c.y), Color(0.5, 0.5, 0.5), 1.0)
	draw_line(Vector2(c.x, r.end.y), Vector2(c.x, plot.end.y), Color(0.5, 0.5, 0.5), 1.0)


## The colour key, plus the two non-ramp states spelled out. A ramp alone would
## leave a reader to guess whether a dark cell is expensive or impossible.
func _draw_legend(plot: Rect2, mid: Color, dim: Color, faint: Color) -> void:
	var x := plot.end.x + 16.0
	# Pushed well below the plot's top edge: the HUD's clock/warp block occupies
	# the top-right corner in every view, and a key drawn level with the plot lands
	# straight on it.
	var y := plot.position.y + 52.0
	var bar_h := 96.0
	var metric: Array = Sim.PORK_METRICS[Sim.pork_metric]
	var lo_text := "0"
	var hi_text := "%.0f" % _hi
	if str(metric[0]) != "c3":
		hi_text = "%.2f" % (_hi * 1000.0)      # m/s -> mm/s, the readout's unit

	_t(Vector2(x, y - 26.0), str(metric[3]), mid, _fs - 2)
	_t(Vector2(x, y - 14.0), str(metric[2]), faint, _fs - 4)
	for n in range(24):
		var f := n / 23.0
		var s := pow(f, 0.6)
		draw_rect(Rect2(Vector2(x, y + bar_h - f * bar_h), Vector2(14.0, bar_h / 24.0 + 1.0)),
			Color(s, s, s), true)
	draw_rect(Rect2(Vector2(x, y), Vector2(14.0, bar_h)), faint, false, 1.0)
	# Bright end is "good" in both metrics, so the numbers are what differ: cheap
	# C3 at the top, large delivered Δv at the top.
	_t(Vector2(x + 19.0, y + 8.0), hi_text if str(metric[0]) != "c3" else lo_text, dim, _fs - 3)
	_t(Vector2(x + 19.0, y + bar_h), lo_text if str(metric[0]) != "c3" else hi_text, dim, _fs - 3)
	if str(metric[0]) == "c3":
		_t(Vector2(x, y + bar_h + 16.0), "VEHICLE", faint, _fs - 4)
		_t(Vector2(x, y + bar_h + 26.0), "CEILING", faint, _fs - 4)

	y += bar_h + 48.0
	draw_rect(Rect2(Vector2(x, y), Vector2(14.0, 10.0)), Color(0.075, 0.075, 0.075), true)
	draw_rect(Rect2(Vector2(x, y), Vector2(14.0, 10.0)), faint, false, 1.0)
	_t(Vector2(x, y + 22.0), "TOO MUCH", faint, _fs - 4)
	_t(Vector2(x, y + 32.0), "C3 FOR", faint, _fs - 4)
	_t(Vector2(x, y + 42.0), "THIS ROCKET", faint, _fs - 4)
	y += 62.0
	draw_rect(Rect2(Vector2(x, y), Vector2(14.0, 10.0)), faint, false, 1.0)
	_t(Vector2(x, y + 22.0), "NO TRANSFER", faint, _fs - 4)
	_t(Vector2(x, y + 32.0), "AT ANY LAP", faint, _fs - 4)


## The cursor cell's numbers, and the one line that is not a planning estimate.
func _draw_panel(origin: Vector2, bright: Color, mid: Color, dim: Color, faint: Color) -> void:
	var lh := _fs + 5.0
	var x := origin.x
	var y := origin.y
	# The second column clears the *longest* left-hand line (the launch/arrive/cruise
	# row, ~56 characters), not a guessed 30. Sized off the font rather than a
	# literal, so a font-size change cannot silently overlap the two columns again.
	var col2 := x + _font.get_string_size(
		"LAUNCH  0000-00-00   ARRIVE  0000-00-00   CRUISE 0000 D  ",
		HORIZONTAL_ALIGNMENT_LEFT, -1, _fs).x
	var cell := Sim.pork_cell()

	_t(Vector2(x, y), "LAUNCHER  %s" % Sim.pork_vehicle_name(), bright)
	# `_reachable` is counted in `_rebuild`, not recounted here: this runs every
	# frame, and a 14 400-cell sweep per frame for one readout number is exactly
	# the kind of cost this project has been bitten by twice.
	_t(Vector2(col2, y), "WINDOWS REACHABLE  %d / %d" %
		[_reachable, Sim.pork_rows * Sim.pork_cols], mid)
	y += lh

	if cell.is_empty():
		_t(Vector2(x, y), "NO TRANSFER IN THIS WINDOW", dim)
		y += lh
		_t(Vector2(x, y), "ARRIVAL AT OR BEFORE LAUNCH, OR NO LAMBERT ARC AT ANY LAP COUNT", faint,
			_fs - 2)
		y += lh * 1.4
		_draw_keys(Vector2(x, y), dim)
		return

	var lap: int = int(cell.revolutions)
	var lap_text: String = "DIRECT ARC" if lap == 0 else ("%d SOLAR LAP%s EN ROUTE" %
		[lap, "" if lap == 1 else "S"])
	_t(Vector2(x, y), "LAUNCH  %s   ARRIVE  %s   CRUISE %.0f D" % [
		Sim.date_string((float(cell.launch_tdb) - Sim.EPOCH0_TDB) / Sim.DAY_S),
		Sim.date_string((float(cell.arrival_tdb) - Sim.EPOCH0_TDB) / Sim.DAY_S),
		float(cell.tof_days)], mid)
	_t(Vector2(col2, y), lap_text, mid if lap == 0 else bright)
	y += lh

	var pay: float = float(cell.payload_kg)
	_t(Vector2(x, y), "C3 %7.2f KM2/S2   IMPACT SPEED %5.2f KM/S" %
		[float(cell.c3_km2_s2), float(cell.arrival_v_rel_ms) / 1000.0], mid)
	if pay > 0.0:
		_t(Vector2(col2, y), "DELIVERS %s KG" % Sim.group_num(int(pay)), bright)
	else:
		_t(Vector2(col2, y), "DELIVERS NOTHING (ABOVE %s CEILING)" %
			("C3 %.0f" % Sim.pork_vehicle_max_c3()), dim)
	y += lh

	# The along-track projection is the effectiveness proxy — and it is SIGNED.
	# A negative projection is a retrograde push that shrinks the orbit: a real
	# lever, aimed the other way, not a poorly aimed one.
	var proj: float = float(cell.along_track_proj_ms)
	var dv: float = float(cell.along_track_dv_ms)
	var dir_word: String = "PROGRADE" if proj >= 0.0 else "RETROGRADE"
	_t(Vector2(x, y), "ALONG-TRACK PROJECTION %+7.0f M/S  %s" % [proj, dir_word], mid)
	if pay > 0.0:
		_t(Vector2(col2, y), "IMPARTS %+.4f MM/S" % (dv * 1000.0), mid)
	y += lh

	# The verdict line. Everything above is patched-conic; this is the only number
	# in the view the full n-body field produced, and it says so.
	var verdict_col := dim
	var verdict := "[E] VERIFY THIS WINDOW IN THE FULL N-BODY FIELD"
	if Sim.pork_verifying:
		verdict = "RE-FLYING 12 YR IN THE FULL FIELD ..."
	elif Sim.pork_verdict_is_current():
		verdict = "FULL-FIELD VERDICT:  " + Sim.pork_verdict_label()
		verdict_col = bright
	elif not Sim.pork_verdict().is_empty():
		# A verdict exists but belongs to a different cell. Saying so beats either
		# hiding it or — far worse — showing it beside the cell it is not about.
		verdict = "[E] VERIFY (LAST VERDICT WAS FOR ANOTHER WINDOW)"
	y += lh * 0.3
	_t(Vector2(x, y), verdict, verdict_col)
	y += lh

	# The follow-up [E] cannot answer. When [E] says this launcher fails, the next
	# question is *by how much* — and the answer is a ratio against the payload on
	# the line above, which is the campaign's honest headline in one number.
	#
	# Same full n-body field as the verdict, so it sits under the same banner. It is
	# also **vehicle-independent**, which is why it stays legible after [L]: the
	# window's requirement does not move, only the ratio does.
	var mass_col := dim
	var mass := "[M] SOLVE THE IMPACTOR MASS THIS WINDOW WOULD NEED"
	if Sim.pork_mass_solving:
		mass = "BRACKETING IMPACTOR MASS IN THE FULL FIELD - UP TO ~3 MIN ..."
	elif Sim.pork_required_mass_is_current():
		mass = "NEEDS  " + Sim.pork_required_mass_label()
		mass_col = bright
	elif not Sim.pork_required_mass().is_empty():
		mass = "[M] SOLVE (LAST REQUIREMENT WAS FOR ANOTHER WINDOW)"
	_t(Vector2(x, y), mass, mass_col)
	y += lh * 1.3
	_draw_keys(Vector2(x, y), dim)


func _draw_keys(pos: Vector2, dim: Color) -> void:
	_t(pos, "[ARROWS] SELECT WINDOW  [L] LAUNCHER  [D] METRIC  [E] VERIFY  [M] REQUIRED MASS  [1] BACK",
		dim, _fs - 2)
	_t(pos + Vector2(0.0, _fs + 4.0),
		"PATCHED-CONIC PLANNING ESTIMATES - ONLY THE [E] AND [M] LINES ARE THE REAL FIELD",
		Color(0.30, 0.30, 0.30), _fs - 3)


func _date_of(axis: PackedFloat64Array, idx: int) -> String:
	if idx < 0 or idx >= axis.size():
		return ""
	return Sim.date_string((axis[idx] - Sim.EPOCH0_TDB) / Sim.DAY_S)


func _t(pos: Vector2, s: String, col: Color, fs: int = -1) -> void:
	draw_string(_font, pos, s, HORIZONTAL_ALIGNMENT_LEFT, -1, _fs if fs < 0 else fs, col)


func _centered(s: String, at: Vector2, col: Color, fs: int) -> void:
	var wdt := _font.get_string_size(s, HORIZONTAL_ALIGNMENT_LEFT, -1, fs).x
	draw_string(_font, at - Vector2(wdt * 0.5, 0.0), s, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, col)
