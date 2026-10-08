# Does the 800 km placement band survive below 200 days? — measured 2026-09-07

Decision rule (written before the first flight): `2026-09-07-keyhole-lead-floor-rule.md`, same folder.

## What was flown

All on the **nominal** Öpik frame of the undeflected rock, so every circle below is
the *same* circle at every lead. Retrograde along-track nudges. Probe:
`M:\claud_projects\AsteroidDefense\core\examples\probe_keyhole_placement.rs`.

## 1. Where the reachable curve meets each circle (`xi_sweep`, free of returns)

Scan ceiling raised to 150 m/s with 44 rungs — the shipping 30 m/s default is
calibrated at 150 d and stops short below it.

**Control: the 150 d row reproduces the recorded crossing on all three circles.**
3:4 reads Δv **3.223849**, ξ **−52 865.7 km** against the recorded 3.2235 /
−52 860. The widened scan is therefore readable.

| lead | 3:4 Minus | 2:3 Minus | 5:7 Minus |
|---|---|---|---|
| 150 d | crossed, Δv 3.2238, ξ −52 866 km | crossed, Δv 0.8831, ξ −9 596 km | crossed, Δv 1.5966, ξ −22 770 km |
| 125 d | crossed, Δv 4.4103, ξ −68 990 km | crossed, Δv 1.2602, ξ −14 913 km | crossed, Δv 2.2220, ξ −31 413 km |
| 100 d | NOT REACHED — pass left the 5e8 m scan gate at Δv 37.52 | wrong branch — NOT MEASURED | wrong branch — NOT MEASURED |
| 75 d | NOT REACHED — gate at Δv 65.31 | NOT REACHED — gate at Δv 65.31 | NOT REACHED — gate at Δv 65.31 |
| 50 d | NOT REACHED — gate at Δv 150.0 | NOT REACHED — gate at Δv 150.0 | NOT REACHED — gate at Δv 150.0 |

A gate exit is a genuine terminus: past it the pass has swung so wide there is no
encounter left to reduce, and every larger impulse is wider still. The 100 d rows
for the 2:3 and 5:7 are **not measured**, not negative.

## 2. The doors at 125 d, aimed from those crossings

Every one brackets, and every one has spent its timing — which is what makes the
floor a floor rather than a wall.

| circle | aim Δv | floor Δv | return floors at | ξ₂ (irreducible) | ζ₂ (timing) |
|---|---|---|---|---|---|
| 5:7 | 2.221983 | 2.2274920 | 29 341 km | −35 526 km | **2.9 km** |
| 2:3 | 1.260162 | 1.2610756 | 25 140 km | −31 229 km | **58.1 km** |
| 3:4 | 4.410255 | 4.4540499 | 19 447 km | −25 380 km | **187.9 km** |

None is an impact keyhole. Each is a resonant return that misses Earth by tens of
thousands of kilometres, and the miss is *spatial* — ξ₂ is what timing cannot
remove, and ζ₂ has gone to essentially zero in all three.

**Not a placement measurement.** The 3:4 floor sits "+1 112 km from the circle",
past the 800 km band, and that must not be read as a placement error: placement is
the midpoint of two flown door **edges**, and there are no edges where there is no
door. It is only where the refinement ended while chasing a return minimum.

## 3. Two gates that had to be discarded or fixed on the way

**The `screen` stage is not a door-existence gate.** Control at 200 d and 300 d,
where the 3:4 door is known and was flown: **zero hits**, returns at 478 709 and
576 838 km. A single unrefined shot lands half a million km from Earth even where
a door exists. Its zero hits at 50–125 d were discarded, not written down.

**The ladder's aim is unusable below ~150 d.** It matches the circle's `b` at the
*nominal* ξ; on the 3:4 that point is b = 153 424 km against a circle whose largest
possible b is 153 577 km, i.e. essentially the outermost point, reachable only near
ξ = 0. At a short lead the curve reaches that b far out in ξ. Flown at 125/100/75/50 d
it gave `bracketed = false` four times, ending 214 729 to 320 646 km outside the
circle. Hence the new `dv=` override, aimed from the measured crossing.

**A bug in `xi_sweep`, found and fixed.** It stopped at the first flight that
returned nothing and called it the scan gate. At 50 d the retrograde curve passes
*through Earth* — the ladder reads b = 762 km at Δv 1.7586 — and the flight fails
there. The scan stopped at Δv 1.7787 and reported `NOT REACHED` on a curve the
ladder had already flown out to b = 306 654 km at Δv 70. A gate exit (`Ok(None)`)
is a terminus; a failed flight (`Err`) is a hole to step over. Now distinguished,
and `prev` is cleared across a hole so no bracket spans the discontinuity. With the
fix the 50 d scan runs the full range to 150 m/s and still crosses nothing — the
100 d and 75 d rows are unchanged, so only 50 d was truncated.

## 4. Where the band's domain floor sits

- 200 d: the 3:4 door **exists**, centre +786.0 km from its circle. (recorded)
- 150 d: the 3:4 **has no door** — floors 17 411 km out, ξ₂ −23 263 km. (recorded)
- 125 d: 3:4, 2:3 and 5:7 all crossed, **none is an impact keyhole**. (this batch)
- 100 d: the 3:4 circle **cannot be reached** at all. (this batch)
- 75 d, 50 d: none of the three circles reached. (this batch)

So on these circles the door ceases to exist between 150 and 200 days, and below
that the question the band answers does not arise for them.
