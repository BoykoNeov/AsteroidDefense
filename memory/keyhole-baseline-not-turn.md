---
name: keyhole-baseline-not-turn
description: "2026-09-08 - the keyhole map's placement ladder is in its BASELINE (which orbit the deflected rock arrives on), not in the turn; drawing the circle on the CHANGE in a would collapse a 767 km ladder to a 103 km spread. Measured then SHIPPED 2026-09-08 — see keyhole-circle-on-change."
metadata: 
  node_type: memory
  type: project
  originSessionId: 316511a0-4544-463f-9872-0959f486ec4b
  modified: 2026-09-08T10:24:51.177Z
---

Follow-on to [[keyhole-prediction-vs-condition]], which showed the resonance
**condition** is sound and the **prediction** of `a'` carries the whole
19 → 786 km door-placement ladder. This session found where inside the prediction.

**The two suspects that batch left open are excluded by a number already on
record.** They were the solar tide across the flyby and the finite time the turn
takes — both *encounter-local*. But the 300 d and 12 yr doors sit **0.48° of arc
apart on the same circle**: same `c`, same `b` to under 1 %, same `v∞` to 4e-4,
i.e. the same encounter — and their errors are −33 and −665 km, a factor of 20.
Nothing encounter-local can do that. (They are still live for the *constant*
below.) The general lesson: before flying a new candidate, check whether two rows
already in the file share the thing the candidate depends on.

**Every input to the closed form is innocent too.** Swapped one at a time into the
nominal frame (each rebuilt through `OpikFrame::new`, never by poking fields, so
the axes and `θ` follow the swapped ingredient; and each re-*projects* the
b-vector, since the b-vector is physical and (ξ, ζ) are coordinates):

| lead | bias | `Ŝ` alone | `v∞` alone | `r⊕` alone | `V⊕` alone | sum | all four |
|---|---|---|---|---|---|---|---|
| 4383 d | −33.3 | +95.0 | −79.9 | −9.8 | +208.4 | +213.8 | +226.7 |
| 900 d | −226.6 | +2.7 | −85.8 | −9.8 | +205.7 | +112.8 | +126.0 |
| 300 d | −664.6 | −479.5 | −103.5 | −9.9 | +209.5 | −383.3 | −370.2 |
| 200 d | −838.6 | +732.1 | −38.3 | −9.2 | +152.9 | +837.6 | +847.9 |

None orders by lead; the sum reproduces the all-at-once column to 11 %, so the
terms are **additive** and this was never a hidden cancellation. Two side checks
that had to pass first: `∇a'·n̂` is flat (25.90–26.36 m/m, 1.75 %), so the ladder is
a real `a'` error and not the unit conversion every km-equivalent is scaled by;
and the out-of-plane component `project()` silently drops (21–149 km) is worth
**0.0 km**, being second order.

**What splits it: ask the SAME construction on the leg before the encounter.**
`OpikFrame::incoming_semi_major_axis` is the identical arithmetic with the
incoming asymptote. That separates a **baseline** (which orbit the rock arrives
on) from a **turn** (what the flyby does to it), and only the turn is anything the
encounter owns:

| lead | baseline err | outgoing err | **the CHANGE** |
|---|---|---|---|
| 4383 d | +292.8 | −33.3 | **−326.1** |
| 900 d | +83.8 | −226.6 | **−310.4** |
| 300 d | −422.3 | −664.6 | **−242.3** |
| 200 d | −457.5 | −838.6 | **−381.1** |

**Headline: the two absolutes are 750 and 805 km apart across the extreme doors;
the change is 55 km apart, inside the ±136 km bar.** The flyby is predicted right
to a constant; the ladder is the construction misjudging which orbit the
*deflected* rock arrives on, and it misjudges it more the later the impulse.

**The `r ≈ R⊕ₒᵣᵦ` substitution was real all along — on the other leg.** In the
flight's *own* frame it accounts for **98.8 %** of the baseline error at 200 d
(−1456.9 of −1474.8 km), leaving +72/+67/+72/−18 km at the four leads. It shifts
incoming and outgoing by nearly the same amount and so **cancels in the change** —
which is exactly why substituting it in the outgoing prediction explained 0.4 %.
A correct attribution of a term that does not move the answer.

**The repair, measured and deliberately NOT shipped.** Place the circle where
`a' − a_in = a_res − a_in_true`, with `a_in_true` from the flight the planner has
already propagated — along the normal that is a pure shift, no re-solve. Door
offsets go **+19 / +211 / +648 / +786 (a 767 km ladder) → +312 / +294 / +226 /
+329 (103 km spread)**. A single osculating sample at CA − 30 d, which costs
nothing, is exactly as flat (102 km) at a +380 offset instead of +290. Not shipped
because it is one resonance and four leads, the 103 km residual sits *at* the
incoming measurement's own ±136 km noise floor, and the offset itself would have
to be pinned first. `KEYHOLE_PLACEMENT_KM` is still 800.

**The incoming observable** mirrors the outgoing one exactly — window **closes** at
CA − 30 d, one full revolution **backward** (`Clock` documents a negative cadence),
32 samples, half-revolution shift as the bar, same Hill-radii gate. At short leads
the backward arc runs past the impulse epoch onto a continuation the rock never
flew: **deliberate and correct**, because the observable wanted is the orbit the
rock *is on* at CA − 30 d, not its history. Its bar is ±136 km, **three times** the
outgoing leg's ±41 on the same rock and convention — unexplained.

**How to apply:** `ASTEROID_REQUIRE_KERNELS=1 cargo run --release --example
probe_keyhole_placement -- outgoing 3 4 minus 4383 900 300 200` (~15 s + build).
Pinned by `W:\Claude_projects\AsteroidDefense\core\tests\keyhole_prediction_bias.rs`
(20 s) — which asserts the finding as a **contrast** (doors 767 km apart, baseline
750, outgoing 805, change 55, repaired 16.6), because either half alone would pass
on a run where the whole measurement had gone flat.

**Trap (tooling):** the Bash tool truncates a command near ~120 lines, so a long
heredoc dies with `unexpected EOF while looking for matching`. Write long files
with the Write tool, not `cat <<EOF`. It also collapses `\\` to `\` in tool input,
which breaks Python patch scripts — use `chr(10)` instead of newline escapes.

**Shipped 2026-09-08 in [[keyhole-circle-on-change]]** — with the −320 km constant
re-read as a constant of *semi-major axis* rather than of b-plane distance, which
changed the band's units as well as its size.

**Still open:** the −320 km constant (encounter-local candidates are live again
here); the ±136 km incoming bar; and it is still **one resonance**.

Related: [[keyhole-prediction-vs-condition]], [[keyhole-lead-variable]],
[[keyhole-placement]], [[keyhole-reach]], [[keyhole-ranking]],
[[keyhole-domain-floor]], [[keyhole-lead-sweep]].
