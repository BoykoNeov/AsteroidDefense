---
name: keyhole-lead-variable
description: "The keyhole placement error is indexed by the deflection LEAD, not by the impulse or the place on the circle, and rebuilding the Opik frame makes it worse; band 500 -> 800 km and the crowded register finally fired."
metadata: 
  node_type: memory
  type: project
  originSessionId: 180ce3eb-a055-4d41-a5b6-e4f1d7d5b4d5
  modified: 2026-09-07T18:20:59.371Z
---

**2026-09-07 — the two live threads [[keyhole-lead-sweep]] left, both closed.**
That batch said "the placement error depends on the lead and **nothing here says
why**", and "the several-doors register ships tested but **not yet observed on
real physics**". Answering them moved `KEYHOLE_PLACEMENT_KM` a second time in two
sessions, **500 → 800**.

**"It depends on the lead" was never safe, because a shorter lead drags two other
things with it** — it slides the plan round the circle, and it forces a bigger
impulse. Every earlier point had all three moving together. Two flights separated
them, and **the order of the two eliminations matters**:

1. **Not the place on the circle.** The 3:4 door at **300 d** lands at
   ξ = +5531 km; the 12 yr door at +6103 km — **0.48° of arc apart** on a
   74 855 km circle — and the errors are **+19.3 km vs +648.2 km**. 34×.
2. **Not the impulse.** The **200 d** door floors at a *smaller* Δv than the
   300 d one (2.4785 vs 2.8798 m/s — **the Δv a circle costs is not monotone in
   the lead**) and sits *further* out, **+786.0 km**. Impulse down, error up.
   This pair alone is not decisive (the 200 d shot is at ξ = −21 412 km, so
   position is confounded again); it only works after step 1.

What survives: **of lead, Δv, ξ and the angle round the circle, only the lead
orders all five flown doors** — 4383/900/450/300/200 d giving
19.3/210.6/467.9/648.2/786.0 km. The other three are each non-monotone somewhere.
**The mechanism is still unknown** and the question is now much sharper: something
about applying the impulse *closer to the encounter*, at fixed b, fixed circle and
v∞ agreeing to 4e-4.

**The frame was the obvious mechanism and it is wrong.** The map builds the Öpik
frame from the **nominal, undeflected** encounter and places the **deflected**
point on it, so δv∞, δθ and the arrival-time slip are all unmodelled and all grow
with the impulse — the right shape. `probe_keyhole_placement frame` rebuilt each
flight's circle in its own frame: the spread across leads goes **767 → 1375 km**,
i.e. **1.79× worse**. Not a correction. But the frame *is* large — the deflected
pass reaches closest approach **~1.9 h late**, Earth moves 210 000 km, and that
alone moves the placed point **~200 km at every lead** (a near-constant, which is
why it sizes the band and cannot explain the ladder).

**Two blind spots in the binding test that was meant to cover this**, both now
closed: it compared circle *parameters* (`|δζ_c|+|δR|`) rather than the **placed
point** (which also moves when the axes rotate — +15.1 km on that very plan,
double the threshold it asserted), and it built **both** frames on the nominal
clock, so the dominant +211 km timing term had never been tested at all.

**The crowded register fires — at every dialable lead — and two fixes were needed
before the answer was worth anything.**
- *The old evidence was in the wrong metric.* `spacing`'s 402/83.6/8.3 km are gaps
  between circles at **constant ξ**; the register cuts on `margin` from the
  **plan's own point**, and a plan reaches a given b at its own ξ. Same class of
  error as [[keyhole-ranking]]'s widths-vs-km.
- *The first run's answer was worthless.* Ungated it "fired" at b = 4 593–9 526 km
  against an 11 311 km capture radius — **still impacts**. Circles crowd near
  Earth because every circle passes there. **Always gate on `!enc.is_hit()`.**

Gated, at the shipping 800 km band, all six leads (900…30 d) fire on a genuine
miss with 2 doors, mostly at b ≈ 17 000 km (1.50× capture; the 30 d one is a miss
by 3 % and is quoted as such). **And it fires because the band widened past the
runner-up, not because the physics changed**: the runner-up door's best margin on
the 900 d curve is **644.9 km**, between the retired 500 and the shipping 800.

**The width claim moved too.** 0.89–1.44× held for nine doors; the 200 d door is
**11.8 km flown against 24.9 drawn — 2.106×**, the drawn width taken at the
probe's **aim point** (nominal ξ = 6690 km), which is the convention all ten
ratios use; at the plan's own closest point — the width the panel prints — it is
24.4 km and 2.07×. That is where the 3:4 door is
*closing* (none at 150 d; its return's irreducible ξ₂ is already −10 300 km at
200 d). Still conservative (draws wider than real, errs safe), but "within 1.44×"
is retired.

**How to apply:** `probe_keyhole_placement frame` (1 flight/lead, no doors) and
`crowding lead=<d> band=<km>` (~80 flights, ~2 min). Never quote the band without
the lead. **800 is a measured maximum over ten doors and NOT a bound — and the
unmeasured region is BELOW the last row** (`LEAD_MIN` = 30, nothing flown between
30 and 200 d on any resonance, trend still climbing).

Related: [[keyhole-lead-sweep]], [[keyhole-placement]], [[keyhole-ranking]],
[[keyhole-targeting]], [[gdext-binding]].
