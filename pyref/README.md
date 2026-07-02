# `pyref/` — offline reference-oracle pipeline (NOT shipped, NOT a Cargo member)

This directory holds **Python** scripts that generate validation fixtures and
run reference oracles (`hapsira`, `REBOUND`, `ASSIST`, later `GRSS`) for the
Rust `core`. It is deliberately **outside the Cargo workspace** and must never
appear in any `Cargo.toml`.

## License firewall (read before adding anything)

`REBOUND`, `ASSIST`, `nyx`, and `GRSS` are **GPL/AGPL**. They are fine *here*
because:

- nothing in `pyref/` is linked into the distributed Rust binary, and
- only the **generated data** (JSON fixtures) is committed and consumed by
  `validation/` — data is not a derivative work of the generator.

Only `hifitime`, `ANISE`, and `nalgebra` (permissive / MPL) ever link into the
shipped binary. The one real hazard is `nyx` (AGPL, *Rust*) — never add it to a
manifest. See HANDOFF §3.

## Platform note

REBOUND and ASSIST are C extensions targeting **Linux/macOS**; upstream does
not support native Windows builds. `assist` ships **no wheel at all** — it
compiles from source and needs a C toolchain (`gcc`) plus REBOUND's headers.
On this Windows dev box the oracle pipeline therefore runs in a **Docker
`python:3.12-slim` container with `gcc` added** (WSL Ubuntu here lacks
pip/venv and passwordless sudo). See `SPIKE.md` for the exact, confirmed
invocation. The shipped Rust core is unaffected (it never touches this dir).

## Contents

- `requirements.txt` — pinned **REBOUND/ASSIST** oracle deps (Linux-only C build).
- `spike_assist_de441.py` — task-0.5 de-risk spike (pillar a): build ASSIST +
  the DE441-consistent ephemeris and integrate a test particle.
- `SPIKE.md` — task-0.5 spike results + the fallback-to-Option-B trigger.
- `requirements-hapsira.txt` — pinned **hapsira** deps, kept separate from the
  ASSIST deps so a two-body fixture regen does not need the ASSIST toolchain.
- `generate_kepler_fixture.py` — §10.6 fixture generator: propagates a known
  orbit with hapsira and writes `../validation/fixtures/kepler_two_body.json`
  (the reference column for `KeplerPropagator`). μ is pinned to ANISE's Sun GM
  (see `core/examples/probe_sun_gm.rs`); frame convention and elapsed-seconds
  time are pinned identically on both sides. See the module docstring for the
  Docker invocation.

## Fixtures (the committed output)

Generated JSON lives in `../validation/fixtures/` and **is** committed (the
`.gitignore` excludes kernels and `.pca`/`.bsp` data, but keeps `*.json`
fixtures). Data is not a derivative work of the GPL/AGPL oracles, so committing
the numbers is fine (see the license firewall above). Each fixture records its
own provenance (oracle version, μ source) in a `provenance` block.
