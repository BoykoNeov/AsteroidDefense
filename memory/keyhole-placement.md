---
name: keyhole-placement
description: Five keyholes flown 2026-09-07 - the linearised width is right to 1.44x but four of five doors do not contain their own circle, so no multiple of the width can express placement; the frontend alert is now an additive km band.
metadata:
  type: project
---

Roadmap item 4 (calibrate the keyhole width's order-unity slack on more than
one flown resonance) is **DONE 2026-09-07**, and it turned out to be two
questions wearing one name. Probe:
`W:\Claude_projects\AsteroidDefense\core\examples\probe_keyhole_placement.rs`,
four stages - `ladder` (sample b(dv) once, 28 rungs), `screen` (census + one
flight per candidate), `door` (refine to the floor, then bisect both edges),
`spacing` (closed-form circle crowding, no flights). Work dir
`W:\temp\claude\keyhole_placement`.

**The width calibrates; the placement does not.** Five resonances flown to
return impacts, both door edges bisected:

| res | linearised door km | flown door km | centre offset km | half-widths |
|---|---|---|---|---|
| 3:4 Minus  | 24.879 | 26.919 | +19.255 |  +1.5 |
| 5:7 Minus  |  3.893 |  4.781 |  +2.020 |  +1.0 |
| 7:10 Minus |  1.924 |  2.207 | -26.750 | -27.8 |
| 6:5 Plus   |  0.183 |  0.083 |  -2.228 | -24.3 |
| 2:3 Minus  |  3.413 |  4.846 |  +7.248 |  +4.2 |

**Headline: four of the five doors do not contain their own circle.** Only the
5:7's interval straddles zero. So the map can print CLEAR for a point whose
flown return hits Earth. The 1.64 half-widths long recorded for the 3:4 was
never a conservative *width* - it was the *placement* error (reproduces at
1.548).

**Three candidate laws, all dead.** The offset is not a fixed distance (2.0 to
26.8 km), not a fixed number of half-widths (1.0 to 27.8), and not a fixed `a'`
bias: `offset x |grad a'|` = 503, 196, -3697, -5403, +1914 (30x spread, both
signs). The placement error does **not** shrink when the door does, which is
exactly why no width multiple can express it.

**The width's residual spread IS explained, and by a measured mechanism not a
fit.** The refinement zeroes the return's timing coordinate, so the door is the
chord swept in `zeta2` at fixed spatial offset `xi2`: shrink factor
`sqrt(1 - (xi2/b_cap)^2)`. The proof is that **b at all ten door edges came out
11 240 +/- 60 km** - `sqrt(xi2^2+zeta2^2) = b_cap` at an edge *is* the chord
statement, on ten independent flights, and `v_inf` survives the resonance so
every return faces the same size disc. Dividing it out collapses the width
ratios from 0.45-1.42 to **1.00-1.44**. The 6:5 (the only case where the
linearised door is *wider* than flown) is entirely this: 10 040 km off axis on
an 11 250 km disc leaves 45% of the chord. Since `b_cap` is common, the residual
1.00-1.44 must live in the `b1 -> zeta2` amplification, i.e. in `|grad a'|`.

**The unplanned honesty check.** The ten edge returns land at 6268.6-6370.3 km
against R_earth 6378 km - they graze the surface, on the *return's own* capture
disc. Given this repo's history of pairing `b` against `R_earth` or against the
wrong encounter's `b_capture`, that is worth keeping. See [[gdext-binding]].

**Two core fixes this needed.** `KeyholeSolution::is_impact_return` was
comparing the return's *already focused* distance against **encounter 1's**
capture radius, so any return between 6 378 and 11 312 km was called an impact
keyhole while being a clean miss - now asks the return's own `is_hit()`, with a
kernel-free regression test. And `refine_keyhole_return` was split out of
`solve_keyhole_return` so a caller with a bracketing aim skips `required_dv`'s
~18 campaign re-flights; that is what made a 5-resonance survey affordable.

**Short returns are NOT the reachable keyholes.** The screen overturned that:
5:7, 7:10 and 6:5 all floor closer than the 3:4, while 4:3 and 7:5 floor at
153 448 and 1 051 352 km. The deciding column is `xi2`, not `h`. Only those last
two deserve "cannot be an impact keyhole"; the rest is "`xi2` too large for the
timing to close", a statement about this rock.

Related: [[keyhole-targeting]], [[keyhole-reach]], [[keyhole-probability]].
