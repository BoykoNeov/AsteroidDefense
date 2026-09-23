---
name: keyhole-constant-not-one-number
description: "2026-09-23, roadmap item 9 CLOSED - the leftover keyhole placement error is NOT one number in a' (2 831..46 806 km over 21 flights, 8 resonances, one sign); close orbit-raising circles (6:5, 7:6) are 2-5x the rest, so the band went UP 15 000 -> 51 000 km of a' (user chose blanket raise, offered as 48 000; 51 000 after the snapshot-convention conversion); crowded register NOT reachable on the shipped circles (~660 000 needed)."
metadata:
  node_type: memory
  type: project
  originSessionId: 1c459d94-811d-459c-9305-c4fe1e1612b8
  modified: 2026-09-23T14:01:05.572Z
---

Follows [[keyhole-circle-on-change]], which left "is the ~8 000-11 000 km-of-a'
constant one number?" open.

**Answer: no.** Error in the change across the flyby, CA-30 d convention (probe
prints revolution mean; add each row's printed snapshot offset, ~+2 400 on every
row since all flights arrive on the same orbit):
- lowering circles 3:4, 5:7, 7:10, 2:3, 5:8: 7 983 .. 14 185
- 7:8 Plus (raises, far, b 84k): 2 831 / 7 106 - the smallest
- 7:6 Plus / 6:5 Plus (raise, close b ~23k, grad ~2 400): up to 40 507 / 46 806 at 200 d
Same sign on all 21: the map always predicts the post-flyby orbit too LARGE (Plus
rows print minus in b-plane km only because grad·n̂ < 0). Separating pair
pre-registered: 5:8 (close, lowers) normal, 7:8 (raises, far) small - neither
factor alone. Mechanism open; encounter-local suspects more live.

**Shipped:** `KEYHOLE_PLACEMENT_A_KM` 15 000 -> **51 000** (worst 46 806 + ~3 540
bar). At 15 000 a 6:5 plan could read CLEAR inside its error. **User chose the
blanket raise** over a close-raising-only exception and over documenting only; cost
is the 3:4 warning zone ~575 -> ~1 950 km. Five close raising circles (5:4, 4:3,
7:5, 3:2, 5:3 Plus) never flown - where 51 000 is least supported.

**Crowded register:** `stage_crowding` had been passing b-plane `band=` km into
`doors_within_band` (metres of a' since 2026-09-08) and censusing plain circles.
Fixed (on-change census per rung's own arriving a, `band_a=`, `band=` refused).
Result: NOT reachable at 900/300/200 d - needs ~660 000 km of a'. Old "fires at
every lead" was the wrong unit.

**Traps:** `outgoing`'s fixed 500 d post-flyby arc panicked on the 6:5's 438 d
orbit (now period-sized). New `census` stage lists branch/b/gradient/raises per
circle - use it to pick circles. Never `cd` in Bash (hook blocks the whole command,
nothing runs). Don't edit the probe while a background `cargo run` loop uses it.

**How to apply:** band calibrations must be converted to the CA-30 d single-sample
convention before comparing with the shipped constant. Full write-up: HANDOFF.md
*The constant is not one number*. Logs `W:\temp\claude\keyhole_constant\`.
