# The sky-observation screen — plan (2026-10-10)

The player sees the asteroid move across real background stars over several
telescope shots, measures where it is in each one, and works out its orbit from
those positions, three ways: by guessing and comparing, with the classic
three-shot method (Gauss), and with a best fit that also gives the uncertainty.
Teaching text and optional hints run through all of it.

This is the start of the roadmap's last open item, **detection**.

## Decisions taken (user, 2026-10-10)

| question | answer |
|---|---|
| which asteroid | **both, Apophis first** — JPL publishes Apophis' sky positions, so the sky maths is checked against an outside source before the made-up threat uses it. The threat's "real" shots come from its hidden true orbit, labelled as such |
| background stars | **Tycho-2, whole sky** (CDS I/259, 2.5 M stars, ~100 MB compressed), fetched like the kernels and gitignored — **plus real star names, switchable** |
| where it sits | **a standalone screen first** (key `[5]`); hooking it into "when is the threat found" comes later |
| how far fitting goes | **all three, in order**: guess-and-compare → Gauss three-shot orbit → least-squares best fit whose covariance feeds the existing Tier-3 ellipse (the old roadmap line "covariance ellipse shrinking with observations", never built) |

## Rules this plan inherits

- **Core computes, GDScript draws.** Every position on the screen comes out of
  `asteroid_core`; GDScript owns zero orbital mechanics (HANDOFF, Godot visual layer).
- **The Horizons `.neo` tables are display-only** — a test enforces they never
  reach the force model. They may supply Apophis' *true* sky positions; any orbit
  the player fits is propagated by the core.
- **A round trip is not an oracle.** Fitting our own synthetic shots and getting
  our own orbit back proves the code is self-consistent, nothing more (the SBDB
  batch learned this). Every step below names an outside check.
- **No aberration on the rock that the stars do not have.** Catalog positions are
  *astrometric* (no annual aberration, no light deflection). The rock's
  position is computed the same way: light-time corrected, nothing else. That way
  rock and stars sit in one frame, and Horizons' "astrometric RA/Dec" quantity is
  the matching oracle.

## Steps

Each step ends with a check that could fail, then commit and push.

### 1. Where the rock appears in the sky (core) — `core/src/astrometry.rs`

- `astrometric_radec(observer_ssb, target_ssb_fn, t_obs)` → RA, Dec, distance,
  with the **light-time iteration** (the rock is seen where it was when the light
  left it).
- Observer = geocentre first. **Measure** geocentre vs a real site (parallax) at
  ordinary distances and at the 2029 flyby before deciding whether a site is
  needed: at 38 000 km the two differ by degrees; at 0.1 au by arcseconds.
- **Gate:** Horizons observer table, quantity 1 (astrometric RA/Dec), centre
  `500@399`, for Apophis on a spread of dates including the 2029 approach;
  fetched by a new `pyref/` script into a committed fixture. Tolerance comes from
  the measured `.neo` interpolation error, converted to arcseconds — not picked.

### 2. The star catalog — `tools/fetch_tycho2.py` + `core/src/star_catalog.rs`

- Fetch CDS I/259 `tyc2.dat.*` and pack it into a compact binary sorted into
  sky zones (RA/Dec, proper motion, BT/VT, HIP number), in
  `kernels/stars/` (gitignored), found by the same resolver path as the kernels.
- Proper motion applied to the shot's date (Tycho-2 positions are J2000 mean
  positions with proper motions).
- **Names:** the IAU WGSN list of proper names (keyed by HIP) and Bayer /
  Flamsteed designations (Yale Bright Star Catalogue, via HD↔HIP). Licences
  checked and recorded before shipping anything; fetched, not committed.
- A named star is rare in a telescope-sized view, so the screen needs a **wide
  zoom** (constellation scale) where names matter, down to the telescope view.
- **Gate:** a handful of known stars (Vega, Sirius, Polaris) at their published
  J2000 positions; cone-search star counts against VizieR for one field.

### 3. The shots (core) — `core/src/sky_shot.rs`

- A shot = time, field centre, field size, pixel scale, exposure. Gnomonic
  (tangent-plane) projection of the stars and the rock.
- Rock brightness from the H,G magnitude system (H and G sourced per object).
- Measurement noise on the rock's position: a **sourced** typical astrometric
  error, labelled; a seeded random generator so shots are reproducible.
- Field size and spacing between shots chosen from the rock's real motion rate
  (arcsec per minute), measured, not guessed.

### 4. The screen (Godot) — `[5]`

- Shots shown **blinking** (the classic way to spot a mover) or **stacked**;
  zoom from wide to telescope view; star names on a toggle; credit line for the
  catalog. Uses the existing panel-exclusivity helper and the HUD key budget.
- Visual check through a `_shot.gd`-style headless capture — the only honest
  look at a view that is hidden until a key is pressed.

### 5. Guess and compare

- The player dials a trial orbit (a, e, i, node, perihelion argument, mean
  anomaly). Ghost markers show where that orbit puts the rock in each shot, with
  the miss in arcseconds per shot and overall.
- **Before choosing** two-body or full-field prediction for the ghosts, measure
  the gap between them over a typical arc against the measurement noise.

### 6. Measuring the shots — manual, semi-automatic, automatic

- **Manual:** the player clicks the rock; reference stars turn the click into
  RA/Dec (a linear plate solution — the real astronomer's step, and a teaching
  moment).
- **Semi-automatic:** the click is snapped to the rock's centre.
- **Automatic:** the game measures it.

### 7. The three-shot orbit (Gauss)

- Gauss's method from three measured positions, with step-by-step hints.
- Its eighth-degree equation can have **more than one** physical root: all are
  shown, and the player picks with the help of a fourth shot. Never chosen
  silently.
- **Gate:** a textbook worked example with published numbers (Curtis, or
  Vallado), reproduced to its printed digits.

### 8. The best fit and its uncertainty

- Least-squares differential correction over all shots, starting from the Gauss
  orbit or the player's guess, with the covariance it produces.
- Feeds the existing Tier-3 machinery: the b-plane ellipse and impact
  probability, now *earned* from observations rather than invented.
- **Gate:** chi-square consistency over many noisy realisations (the scatter of
  fitted orbits matches the claimed covariance), and if available real MPC
  astrometry for Apophis fitted and compared with JPL's SBDB orbit.

### 9. The made-up threat

- Same machinery pointed at the synthetic rock, its "real" shots generated from
  its hidden true orbit and labelled as such.

### Later — the detection hook

The date the threat is "found" (standing defence, part 1) comes out of the
shots instead of being dialled.
