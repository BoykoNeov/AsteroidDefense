---
name: sky-observation
description: "2026-10-10 roadmap's last item (detection), plan docs/plans/2026-10-10-sky-observation-screen.md: the [5] sky screen - Apophis from Mt. Lemmon (G96) against Tycho-2, steps 1-5 DONE (position, stars, shots, screen, guess-and-compare); next step 6 measuring the shots, then Gauss, then least-squares fit feeding Tier-3. Trap: GDScript % has no %g and the shot harness still PASSES."
metadata:
  node_type: memory
  type: project
  originSessionId: 99be4291-672a-4c13-a099-aa5a8ab645b9
  modified: 2026-10-10T12:25:41.863Z
---

The sky-observation screen is standing defence part 4 (detection) — see
[[standing-defence]]. Plan and progress table:
`docs/plans/2026-10-10-sky-observation-screen.md`.

**State 2026-10-10:** steps 1–5 done and committed (sky position vs Horizons,
Tycho-2 catalogue, simulated telescope shots with Vereš 2017 G96 errors, the
`[5]` screen, and the `[M]` trial-orbit dials with ghost markers / `[H]` hint /
`[E]` JPL's orbit / `[R]` restart). Next: step 6 (player measures the shots:
click → plate solution), 7 (Gauss three-shot orbit, gated on a textbook worked
example), 8 (least-squares fit whose covariance feeds [[tier3-uncertainty]]),
9 (the made-up threat).

Step 5 numbers: start guess 12.5° out; JPL's orbit leaves 0.37″ (game seed) /
0.42″ (Rust test seed) — the measuring error alone; two-body ghosts are honest
over the run's ±1.5 days (0.008″) but not over weeks (13.6″ at ±30 d), which
matters once step 8 fits longer arcs.

**Trap (cost a whole step's panel):** GDScript's `%` operator has no `%g`; one
bad specifier makes the WHOLE line print as the raw format string, logged only
as "String formatting error: unsupported format character". `_sky_shot.gd`
asserts on `Sim` numbers, not drawn text, so it passed while all six dial rows
were garbage. **How to apply:** after every harness run, read its ERROR lines
(run_harness.ps1 prints them from the `.err` log) and look at the PNGs in
`W:\temp\claude\sky-obs\shots`; use `String.num(x, n)` for compact numbers.
Run the harness through `cmd //v:on //c "start /belownormal /b /wait powershell
-NoProfile -File ...run_harness.ps1 ... & exit !errorlevel!"` (below-normal rule).
Related: [[godot-visual-layer]], [[gdext-binding]].
