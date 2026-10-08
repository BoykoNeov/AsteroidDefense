# Is the leftover placement error one number in a'? — decision rule, written 2026-09-23 BEFORE any flight

Observable: per flight, the error in the CHANGE of a across the encounter, "out by, in km of a'"
(nominal frame), from `probe_keyhole_placement outgoing h k branch fly=...`, converted with THAT
row's own printed grad a'.

Existing six rows (3:4 x4 leads, 2:3 x2): +6 274 .. +11 024 km of a', all positive.
Incoming-bar self-check: ~±3 535..3 540 km of a' on both resonances.

New rows: 5:7 Minus, 7:10 Minus, 6:5 Plus, at leads 4383, 900, 200 d (crossing flights from xi_sweep).

Outcomes:
- ONE NUMBER: every new row same sign, inside ~+6 000..+11 000 ± 3 540 -> the constant is
  resonance-independent over grad a' 26..2 425 m/m; subtraction becomes defensible (proposal, not auto-ship).
- SCALES WITH THE TURN: the 6:5 (the only a-RAISING flyby) comes out opposite sign -> the error follows
  the turn (finite turn time / solar tide suspects), NOT subtractable as a constant.
- NOT ONE NUMBER IN a': same sign but magnitude orders with gradient -> the units are still wrong.
- 6:5 alone deviating does not isolate gradient: it also changes branch (Plus) and sign of Δa.
  5:7 / 7:10 (Minus, 97 / 138 m/m) test "one number" cleanly.
- Self-check fails (incoming bar far from ~±3 540 km of a' on a new resonance) -> that row is unreadable.
- A crossing NOT REACHED at 200 d is recorded as a result only after raising dvmax=/rungs=.
- The a_true − a_res column on crossing rows is NOT a measurement (search residual 0.5 km × grad).

## Addendum, written 2026-09-23 after the 5:7 / 7:10 / 6:5 rows and BEFORE the separating flights

Result so far: 5:7 (+5 595..+6 938) and 7:10 (+5 629..+6 514) sit inside the six-flight cluster;
6:5 Plus is SAME sign in a' but +15 933 / +16 821 / +44 237 (self-check passes: bar 3 420..3 640).
6:5 differs on four axes at once: gradient (2 425), branch (Plus), direction (raises a), and b (22.6k km
= 2.0 capture radii, vs 40k..153k for the rest). Census: W:\temp\claude\keyhole_constant\census.log.

Separating pair (each at 4383 and 200 d):
- 7:8 Plus  — RAISES a, FAR (b 84 093 km, 7.4 capture radii), grad -119.5 (mirror of 7:10 Minus in b and grad)
- 5:8 Minus — LOWERS a, CLOSE (b 24 593 km, 2.2 capture radii), grad 486.7 (matches 6:5 in b)

"Large" = above 11 000 + 3 540 = 14 540 km of a'. "Normal" = inside ~6 000..11 000 ± 3 540.
- 7:8 normal AND 5:8 large  -> CLOSENESS drives it (encounter-local); widen only for close passes.
- 7:8 large  AND 5:8 normal -> DIRECTION / BRANCH drives it; widen for a-raising circles.
- both large               -> both matter or gradient (5:8 grad 487 is 2x the 2:3); report, blanket raise.
- both normal              -> the 6:5 is special beyond these axes; fly 7:6 Plus (its near-twin: raises,
                              b 24 191, grad -2 068) before deciding anything.
