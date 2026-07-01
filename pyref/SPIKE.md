# Task-0.5 de-risk spike — results & fallback-to-Option-B trigger

**Date:** 2026-07-01 · **Gates:** everything downstream in HANDOFF §10 (the
ephemeris-test-particle MVP + ASSIST-as-oracle architecture, "Option A").

The spike confirms the two pillars Option A rests on, and — crucially — writes
down, *in advance*, the objective condition under which we abandon Option A for
Option B. That trigger is the real deliverable: it stops us sinking weeks into
an architecture whose foundation quietly failed.

---

## Pillar B — ANISE DE-position reader returns a reconstructed geocenter

**Status: PASS.** (Pure Rust, runs natively on Windows — the shipped path.)

- Reader: `core/src/ephemeris.rs` (`Ephemeris` over an ANISE `Almanac`).
- Runnable proof: `cargo run -p asteroid_core --example spike_geocenter -- <de440s.bsp>`.
- Gated test: `geocenter_is_reconstructed_not_emb` (runs when `ASTEROID_DE_KERNEL`
  is set; skips green otherwise so CI stays offline).
- Kernel: `de440s.bsp` (DE440, 1849–2150 span, 32 MB) from NAIF.

What it proves, at four epochs across the span:

| Check | Result |
|---|---|
| geocenter (Earth 399) vs SSB **≠** EMB (3) vs SSB | offset **4351–4908 km** (never ~0) |
| offset tracks the *real* Earth–Moon distance | yes: 4351 km @ 358 Mm, 4908 km @ 404 Mm |
| Earth–Moon distance physical | 358 000–404 000 km |
| EMB reconstructed from Earth+Moon via EMRAT vs ANISE's EMB | residual **0.000000 km** |

The zero EMRAT residual is the strong result: ANISE's Earth-399 segment is the
genuine geocenter, self-consistent with the EMB and Moon to floating-point — not
a relabelled EMB. The §5 "~4671 km EMB footgun" is provably avoided.

Bonus: with `anise` at `default-features = false` the core's dependency tree has
**no `ureq`/network and no egui** (verified via `cargo tree`) — the §10 offline
and no-UI-in-core invariants hold.

---

## Pillar A — ASSIST builds offline and integrates a test particle in the DE field

**Status: PASS** — with DE**440** planetary (`linux_p1550p2650.440`) + DE**441**-
consistent asteroids (`sb441-n16.bsp`). (Docker `python:3.12-slim` + `gcc`,
2026-07-01.)

> Naming precision: ASSIST's *planetary* ephemeris here is DE440 truncated to
> 1550–2650; only the 16-asteroid set is DE441-derived. This is ASSIST's own
> shipped pairing, the C reader is identical for the full DE441 planetary file
> (~2.6 GB, 1550–2650 vs -13000..17000 span), and HANDOFF §10 task 1 explicitly
> allowed "DE440 or DE441" — so the substitution is deliberate, not a shortfall.

- Script: `pyref/spike_assist_de441.py`.
- Built: `rebound` 4.6.0 (assist pins rebound < 5) + `assist` 1.2.3, compiled
  from the assist sdist against REBOUND's headers.
- Perturber data (downloaded once, git-ignored): `linux_p1550p2650.440` (DE440
  planetary, 1550–2650, 102 MB) + `sb441-n16.bsp` (DE441-consistent 16
  asteroids, 646 MB) — ASSIST's shipped set and the exact field the Tier-1 Rust
  core mirrors as a test particle.
- Test particle: asteroid (3666) Holman, barycentric equatorial ICRF, at
  JD 2458849.5 (2020-01-01). Integrated forward 100 d (moved ~0.85 AU, so the
  DE force model is actually acting), then back to t0.
- **Round-trip position error: 4.665e-04 m (0.47 mm)** — well under the 1 m
  reversibility bar. ASSIST builds, loads the DE441 field, and integrates.

### Platform finding (important for the pyref pipeline)

- `rebound` 5.0.0 installs from a **wheel on both native Windows and Linux**.
- `assist` 1.2.3 has **no Windows wheel and fails to build natively on Windows**
  (`failed-wheel-build-for-install`) — upstream targets Linux/macOS only.
- WSL Ubuntu here lacks `python3-venv`/`pip` and passwordless `sudo`, so the
  chosen, reproducible oracle host is **Docker `python:3.12-slim`** (no sudo,
  pulls its own toolchain, bind-mounts the repo + a data cache).

Canonical invocation (from repo root, Windows):

```
docker run --rm \
  -v "M:/claud_projects/AsteroidDefense/pyref:/pyref:ro" \
  -v "<data-cache-dir>:/data" -e ASSIST_DATA_DIR=/data \
  python:3.12-slim \
  bash -c "apt-get update -qq && apt-get install -y -qq gcc \
           && pip install numpy rebound assist \
           && python /pyref/spike_assist_de441.py"
```

(`gcc` is required: `assist` ships no wheel and compiles from source. `slim`
lacks a compiler, so it is added in the container — no host toolchain or sudo
needed. Data files persist in the mounted `/data` cache across runs.)

For a version-controlled env, `pyref/Dockerfile` bakes the same setup:
`docker build -t asteroid-pyref pyref/`, then run with the two mounts above.

---

## Fallback-to-Option-B trigger (decision rule, written in advance)

**Fall back to Option B if ANY of these is true:**

1. **ASSIST cannot be built/imported** on any available host (native, WSL, or
   Docker) within the spike's effort budget — i.e. `import assist` never
   succeeds.
2. **ASSIST cannot integrate a test particle** in the DE ephemeris field: the
   `assist.Extras` sim fails to load the ephemeris, fails to step, the particle
   does not move, or a forward+back round trip is not reversible to < 1 m.
3. **The ANISE DE-position reader stalls**: it cannot return a geocenter
   *distinct* from the EMB for a known epoch (returns the EMB, i.e. offset ≈ 0,
   errors out, or fails EMRAT self-consistency).

**Option B (the fallback):** ship a **self-consistent N-body MVP** (Sun +
planets + Moon as *gravitating* bodies integrated alongside the asteroid),
validated against **REBOUND**, and **revisit the ephemeris-perturber
architecture at Tier 2**. Caveat carried from §10: under Option A you may *demo*
the hit→miss flip before ASSIST validation completes, but **REBOUND cannot be
the trajectory oracle** — it self-gravitates the planets, so it is not the same
dynamical system as a test particle in a fixed DE field.

**Current verdict:** Pillar B **PASS**, Pillar A **PASS**. Trigger **NOT
fired** → **proceed on Option A** (ephemeris test-particle MVP with ASSIST as
the trajectory oracle from Tier 1). The downstream §10 plan (steps 3–10) is
unblocked.
