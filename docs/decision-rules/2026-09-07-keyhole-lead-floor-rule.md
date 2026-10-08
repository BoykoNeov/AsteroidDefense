# Does the 800 km placement band survive below 200 days? — decision rule

Written 2026-09-07, BEFORE the first flight, so the outcome cannot be chosen
after the numbers are in. This is the same discipline `xi_sweep`'s doc comment
used, and it is what made that stage's first (wrong) run legible.

## The question

`Sim.KEYHOLE_PLACEMENT_KM = 800.0` is how far the app admits a drawn resonant
circle may sit from the door the rock actually flies through. It was calibrated
on doors flown at leads of **200 d and up** (12 yr, 900, 450, 300, 200). The
planner lets a player dial **30 d**. Nothing has ever been flown between 30 and
200 d except the 150 d shot, which found **no door on the 3:4** — a result about
one circle at one point, not about short leads.

800 is `786.0` rounded up: **one** measurement decides it.

## The leads

50, 75, 100, 125 d. Four, not two, because the crossing point ξ is measured to
oscillate with lead (+4390, +906, −9310, +4952, −4379, −15811, +5537, −52860 km
at 900…150 d) with no law offered. A single lead lands at one arbitrary point of
one circle, so it can neither confirm nor deny anything about its neighbours.

## The gate before the flights

Per lead: `ladder` (writes b(Δv) once), then `screen` (~15 s per candidate, no
door bisection). Fly `door` only where the screen shows a return that **HITs**
with a spatial offset ξ₂ small enough that timing can plausibly close it.

Two things read off the screen BEFORE any door is called a result:

- **Δv against the app's ceiling.** `DV_MAX = 300 m/s`. A door needing more than
  that is a real measurement of a place the player cannot go — a different
  answer, not a violation of the band.
- **Aim residual against the door width.** Widths so far are 20–28 km. If the
  ladder's interpolation residual is a large fraction of that, the ladder is not
  bracketing and `door` will burn flights or return `bracketed = false`.

## The outcomes, decided now

1. **A door is flown at some lead < 200 d and its centre sits ≤ 800 km from the
   circle.** The band holds. It gains rows in its doc and, for the first time, a
   **stated domain floor**: the lowest lead at which a door has actually been
   flown. No constant moves.

2. **A door is flown at some lead < 200 d and its centre sits > 800 km out.**
   The constant moves a third time, to that measurement rounded up the way 786.0
   became 800. Its doc is **rewritten**, not appended to. A kernel-gated binding
   test re-solves the new plan and pins the number, as the 300 d/200 d ones do.
   `_shot.gd` re-run to say what the wider band did to the app's own verdicts.

3. **No swept circle is crossed by the reachable curve below some lead.** Then
   the answer is a **reachability floor, not a bigger constant**. The constant
   stays at 800 and its doc states the floor.

   **Amended before any result arrived** (the original wording was "no resonance
   yields a door at any of the four leads", which is not establishable without
   flying every resonance in the census). What this campaign can actually claim is
   a statement about the circles it swept, and only at a scan ceiling that
   reproduces a known crossing: below lead X, no crossing exists on any of the
   circles swept, at a `dvmax=` that reproduces the recorded 150 d control. A
   reachability statement about swept circles is defensible; "no door anywhere" is
   not, and must not be written down.

4. **`bracketed = false`.** This is **NOT MEASURED**. It is not outcome 3. The
   stage says so itself. It means raise `max_widenings` or fix the aim, and it
   may not be written down as "no door".

## The sanity check every flown door must pass

The eight doors flown so far put their edge returns at 6 293.6–6 334.3 km against
`R_earth = 6 378 km` — edges graze the surface, on the return's own capture disc.
A new door whose edges do not graze is a suspect **edge-finder**, not a physics
result, and is chased before it is written down.

## The control that gates the whole sweep

Raising the scan ceiling to cover short leads (`dvmax=`) makes the rungs coarser
in *ratio*, and the curve cuts every circle **twice**. If one rung pair straddles
both crossings the signed distance has the same sign at both ends and the sweep
reports `NOT REACHED` — silently, with a plausible "last distance". That failure
looks exactly like the reachability finding outcome 3 is about.

So the 3:4 sweep's **150 d row must reproduce the recorded crossing** (Δv 3.2235,
ξ −52 860 km) before any other row in that file is read. If it does not, the
widening broke the scan and every `NOT REACHED` below it is uninterpretable.

## Telling a real floor from a wall

`bracketed = true` is necessary and not sufficient. A floor is only a floor if:

- its Δv is not sitting at ~2.27× the aim (`reach_fraction` — the wall all four
  ladder-aimed runs hit) nor at ~0; and
- `ζ₂` has gone to near-zero (the 3:4 floor read −26.6 km after 20 iterations)
  while `ξ₂` carries the irreducible part. A floor with a large `ζ₂` is an
  unfinished search, not a door.

## Work dirs

One per lead, so nothing shares a log or a ladder:
`W:/temp/claude/keyhole_lead_floor/lead<NN>`.
