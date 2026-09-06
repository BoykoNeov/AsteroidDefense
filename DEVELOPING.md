# Developing

The commands that build, test, probe and render this repository, in one place.
`HANDOFF.md` says *why*; this says *how*. Everything here was run on a fresh
Linux clone on 2026-09-02; timings are from that machine (release profile).

## Toolchain

- Rust stable (1.94 at the time of writing), `rustfmt`, `clippy`.
- Python 3.10+ for the tools (standard library only) and `pyref/` (its own
  pinned requirements, Docker for the GPL oracles — see `pyref/README.md`).
- Godot 4.7 only for the frontend; the `godot/rust` binding builds and tests
  without an engine.

## The kernels, and the trap they guard against

The physics runs in the real JPL DE440 field. The core resolves a **pair** — a
DE ephemeris `.bsp` and ANISE's `pck11.pca` — from `kernels/` beside the repo
(or `ASTEROID_DE_KERNEL` + `ASTEROID_PLANETARY_CONSTANTS`). Without them every
physics test **skips and prints green**; `core/src/kernels.rs` explains how that
once made two verification claims vacuous. So:

```sh
python tools/fetch_kernels.py --neo        # DE440 + pck11 (+ Horizons NEO tables)
ASTEROID_REQUIRE_KERNELS=1 cargo test --workspace --release -- --test-threads=2
```

Green there means the physics actually **ran**. The `--test-threads=2` is not
optional on a normal desktop: the binding's 34 kernel-gated tests each build a
full scenario — an ephemeris plus several dense-output clocks — and running them
all at once exhausted memory on a 32 GB box (`memory allocation of 645727232
bytes failed`, then `STATUS_STACK_BUFFER_OVERRUN`). Two at a time passes.

`fetch_kernels.py` tries NAIF first and falls back to the `naif-de440` PyPI
wheel (a proxy that blocks NAIF usually lets PyPI through) and to the ANISE
repository's LFS copy of `pck11.pca`; it verifies magic bytes so an error page
can never pass as a kernel. `--neo` adds the three Horizons state tables the
real-asteroid scenery reads (needs `ssd.jpl.nasa.gov`). The optional 646 MB
`sb441-n16.bsp` (the 16 main-belt perturbers) is not fetched; drop it in
`kernels/` and the build worker mounts it.

## Build and test

```sh
cargo build --workspace --release
cargo test --workspace --release                        # kernel-free: physics skips green
ASTEROID_REQUIRE_KERNELS=1 cargo test --workspace --release -- --test-threads=2  # the real suite
cargo fmt --all -- --check && cargo clippy --workspace --all-targets --release
```

Costs with kernels: `asteroid_core` ~135 s (235 tests), the `godot/rust`
binding ~14 min at `--test-threads=2` (34 tests, most solving real plans), the
`validation` crate seconds. `--release` is not optional — the integration is
~40× slower unoptimised (the workspace already sets `opt-level = 3` for the core
in dev builds for the same reason).

CI (`.github/workflows/ci.yml`) runs fmt, clippy and the kernel-free suite on
every push, then fetches and caches the kernels and runs the core physics with
`ASTEROID_REQUIRE_KERNELS=1`.

## Probes

`core/examples/probe_*.rs` are measurement programs, not tests: each answers
one question about the shipping scenario and prints what would falsify it. The
ones the current physics rests on:

| probe | asks | cost |
|---|---|---|
| `probe_tier3_cost` | what a perturbed re-fly costs at each cadence | minutes |
| `probe_tier3_uncertainty` | the Jacobian cross-checks, the ellipse, P(impact), the ±3σ shell | ~30 s |
| `probe_keyhole_reach` | which resonances the flyby can reach at all (direction sweep) | ~30 s |
| `probe_keyhole_rotation` | the b-vector sign, measured on a flown flyby | ~200 s |
| `probe_keyhole_map` | the whole b-plane keyhole map as JSON, with two flown checks | ~50 s |
| `probe_keyhole_return` | **fly** the 3:4 keyhole and find the return's floor | ~4 min |
| `probe_keyhole_probability` | Tier 3 **at the return**: the chained two-encounter Jacobian and P(impact). Modes `check` (2 flights) / `steps` / `gain` / `probability` / `cadence` / `sweep` | 2 min – 20 min per mode |
| `probe_integrator_convergence` | is `dop853` **converged** on these arcs, and is the limit the tolerance, the snapshot cadence, or the method? Modes `determinism` (the bit-for-bit gate — run it first) / `campaign` / `cadence` / `keyhole` | 30 s – 6 min per mode |

```sh
cargo run -p asteroid_core --release --example probe_keyhole_map -- docs/keyhole_map.json
```

## Rendering the keyhole map

```sh
python tools/keyhole_map_svg.py  docs/keyhole_map.json docs/keyhole_map.svg   # static, README
python tools/keyhole_map_html.py docs/keyhole_map.json docs/keyhole_map.html  # interactive
```

Both are standard-library Python over the probe's JSON; regenerate them after
the probe whenever the scenario or the frame changes. `docs/` is the committed
output.

## The viewers

- **egui (MVP):** `cargo run -p viewer --release` (needs the kernels; the
  Δv-vs-lead curve is cached in `curve.json`, built once by
  `cargo run -p viewer --release --bin curve`, ~8–11 min).
- **Godot (Phase 2):** build the binding with `cargo build -p asteroid_gdext`
  (debug — that is the DLL the editor loads; the core is optimised in dev
  builds), open `godot/` in Godot 4.7. Keys are listed on the HUD; the b-plane
  view is `[3]`, `[H]` toggles the keyhole map on it, `[C]` snaps to closest
  approach. Headless checks: `godot --headless --path godot --script
  res://tests/test_orrery.gd` and the `_shot.gd` autoloads for screenshots (the
  visual layer is only verifiable by looking — see `memory/gdext-binding.md`).

## The Godot harnesses: pictures and frame times

`godot/tests/_*.gd` are autoload harnesses: each drives the running game the way
a player would and either saves PNGs (`_shot.gd`, `_pork_shot.gd`, …) or prints
frame times (`_perf.gd`). They run only when registered as an autoload, and the
screenshot ones need a window. `godot/tests/run_harness.ps1` does the whole
ritual — registers the autoload after `Sim`, launches, waits, restores
`project.godot`, and kills **only the PID it started** if Godot lingers:

```powershell
powershell -File godot/tests/run_harness.ps1 -Harness _shot.gd            # PNGs → temp/AsteroidDefense/shots
powershell -File godot/tests/run_harness.ps1 -Harness _perf.gd -Headless  # CPU cost per view, uncapped
```

Read `_perf.gd`'s output as: **ffi/frame** is the number of native binding calls
the frame made (deterministic, the number to watch), **frame ms** is the wall
clock. Run it `-Headless` for CPU cost — windowed, the desktop compositor pins
the frame to the monitor's refresh even with vsync off, so a windowed run only
shows a view that is *slower* than the display. The 2026-09-05 numbers, headless,
before → after the per-frame caches: the 2D map **18.9 → 7.1 ms** and
**764 → 29** native calls a frame; the 3D views 61 → 29 calls.

Two traps the runner exists for. (1) Godot does not always exit on `quit()` here
— the editor plugin's teardown can leave the process alive at 100 % CPU, which
poisons the next measurement *and holds the debug DLL*, so the next
`cargo build -p asteroid_gdext` fails with `Access is denied (os error 5)` and
Godot keeps loading the stale binding (`Nonexistent function 'X' in base
'Mission'`). (2) Other projects' Godot processes may be running on the same
machine; the runner never touches them — kill only a PID you started.

## Layout

```
core/         asteroid_core — the physics (see core/src/lib.rs for the module map)
core/examples the probes
validation/   oracle-ladder tests over committed pyref fixtures
pyref/        offline fixture generators (GPL oracles, never linked)
viewer/       egui MVP viewer + the curve cache builder
godot/        Godot 4.7 project; godot/rust is the gdext binding (→ core, one way)
tools/        kernel fetcher, keyhole-map renderers
docs/         generated keyhole map (JSON, SVG, HTML)
memory/       the assistant's project memory, mirrored for transparency
```
