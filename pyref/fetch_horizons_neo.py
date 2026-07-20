#!/usr/bin/env python3
"""Fetch real near-Earth asteroid state tables from JPL Horizons.

**Why a state table and not an SPK.** The obvious path — ask Horizons for a
binary SPK and mount it beside ``sb441-n16.bsp``, the same read path the sixteen
main-belt perturbers already use — does not work, and this was measured rather
than assumed (``core/examples/probe_horizons.rs``):

* ``sb441-n16.bsp`` is **SPK type 2** (Chebyshev position).
* A Horizons per-object SPK is **SPK type 21** (extended modified difference
  arrays). There is no request parameter that changes this.
* ANISE 0.10.3 evaluates SPK types 1, 2, 3, 8, 9, 12 and 13, and returns
  ``Type21ExtendedModifiedDifferenceArray not supported for SPK computations``
  for type 21.

So the almanac cannot read these objects at all, and "same read path" was true
of the call site but false of the decoder underneath it.

**What this does instead, and why it is still honest.** It asks Horizons for the
same trajectory as *states* — position and velocity, sampled on a fixed TDB
cadence — and the consumer (``core/src/horizons.rs``) interpolates between them.
The numbers are JPL's own relativistic solution either way; the only thing we add
is interpolation *between* JPL's samples, whose error is numerical, bounded by
the cadence, and directly measurable against held-out samples.

That is categorically different from integrating a real asteroid ourselves,
which this project deliberately does not do: that would replace JPL's solution
with a worse one of our own (no 1PN relativity, no Yarkovsky — see HANDOFF §5
Tier 2), and it is the same mistake as the display-grade Kepler that was deleted
from GDScript. Interpolating JPL's states is not that.

**Frame.** Heliocentric (``CENTER='500@10'``), ICRF/J2000 equatorial
(``REF_PLANE=FRAME``), km and km/s — which is exactly the frame the core's
``icrf_km_to_ecliptic_au`` expects, so the read path needs no extra rotation.

Usage::

    python pyref/fetch_horizons_neo.py                 # all three, default span
    python pyref/fetch_horizons_neo.py --out DIR       # explicit destination
    python pyref/fetch_horizons_neo.py 99942           # just Apophis

Output lands in ``<kernels-dir>/neo/<slug>.neo`` and is **gitignored**, like the
kernels themselves — a few MB per object, regenerable, and absent on a fresh
clone. Everything works without it; the asteroids simply do not appear.

**Why plain text and not JSON.** ``asteroid_core`` depends on anise, hifitime and
nalgebra and nothing else — serde is deliberately validation-and-viewer-only (see
the workspace ``Cargo.toml``). Adding a JSON parser to the physics crate to read a
display data file would cross that line for no benefit, so the format is a
key/value header followed by one whitespace-separated state per line: ~40 lines of
dependency-free parsing on the Rust side, greppable and diffable on this one.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import urllib.parse
import urllib.request

API = "https://ssd.jpl.nasa.gov/api/horizons.api"

# The teaching asteroids named in HANDOFF §9. `number` is the IAU minor-planet
# number; `naif_id` is the id the object answers to in SPK space and is recorded
# here for provenance only — nothing in the sampled read path looks it up.
#
# Note the numbering: Horizons uses the **extended** small-body convention,
# 20000000 + number, so Apophis is 20099942. `sb441-n16.bsp` uses the older
# 2000000 + number for its numbered perturbers (Ceres is 2000001). The two are a
# digit apart and picking the wrong one produces a lookup failure that looks like
# anything but a typo. Confirmed by enumerating a fetched SPK's segment table.
TARGETS = [
    {"number": 99942, "slug": "apophis", "name": "99942 Apophis", "naif_id": 20099942},
    {"number": 101955, "slug": "bennu", "name": "101955 Bennu", "naif_id": 20101955},
    {"number": 65803, "slug": "didymos", "name": "65803 Didymos", "naif_id": 20065803},
]

# Default span. Wide enough to cover the campaign clock with room either side,
# narrow enough that a 1-day cadence stays a few MB per object. The consumer
# gates every query on the span it actually finds in the file, so widening or
# narrowing this changes when the body is visible and nothing else.
DEFAULT_START = "2020-01-01"
DEFAULT_STOP = "2070-01-01"

# Sampling cadence. Fixed and uniform in TDB, which is what lets the consumer
# store `t0 + i*step` instead of a per-sample epoch list. One day is a compromise
# validated by the held-out-sample test in `core/src/horizons.rs` — do not change
# it without re-running that test, since it sets the interpolation error.
STEP = "1d"
STEP_SECONDS = 86_400.0

# Horizons truncates very long tables, so ask in chunks and stitch. Ten years at
# a one-day cadence is ~3653 rows, comfortably inside any limit.
CHUNK_YEARS = 10

# Julian date of J2000.0 — the epoch the core counts TDB seconds from.
JD_J2000 = 2451545.0
SECONDS_PER_DAY = 86_400.0

# First line of every output file. The reader rejects anything else outright, so a
# file from a different tool — or from a future format revision — fails at the
# first line instead of being read as a plausible-looking trajectory.
FORMAT_MAGIC = "asteroid-neo-states"
FORMAT_VERSION = 1


def step_to_seconds(step: str) -> float:
    """`"1d"` / `"1h"` / `"30m"` as seconds. Rejects anything else rather than
    guessing, because a misread cadence shears the whole trajectory in time."""
    units = {"d": 86_400.0, "h": 3600.0, "m": 60.0}
    unit = step[-1:].lower()
    if unit not in units or not step[:-1].isdigit():
        raise SystemExit(f"unsupported --step {step!r}; use e.g. 1d, 1h, 30m")
    return int(step[:-1]) * units[unit]


def fetch_chunk(number: int, start: str, stop: str, step: str) -> str:
    """The raw Horizons result text for one target over one time chunk."""
    query = {
        "format": "json",
        # Trailing semicolon is what marks this a *small-body* lookup rather
        # than a major-body id; without it 99942 is not found.
        "COMMAND": f"'{number};'",
        "EPHEM_TYPE": "VECTORS",
        "CENTER": "'500@10'",  # Sun body centre
        "REF_PLANE": "FRAME",  # ICRF equatorial, not ecliptic
        "REF_SYSTEM": "ICRF",
        "VEC_TABLE": "2",  # position + velocity
        "OUT_UNITS": "KM-S",
        "CSV_FORMAT": "YES",
        "VEC_LABELS": "NO",
        "OBJ_DATA": "NO",
        "START_TIME": start,
        "STOP_TIME": stop,
        "STEP_SIZE": step,
    }
    url = f"{API}?{urllib.parse.urlencode(query)}"
    with urllib.request.urlopen(url, timeout=300) as response:
        payload = json.load(response)
    if "result" not in payload:
        raise SystemExit(f"Horizons returned no result for {number}: {payload}")
    return payload["result"]


def parse_rows(result: str) -> list[tuple[float, list[float]]]:
    """`(jd_tdb, [x, y, z, vx, vy, vz])` for every row between $$SOE and $$EOE."""
    try:
        body = result[result.index("$$SOE") + 5 : result.index("$$EOE")]
    except ValueError as exc:
        raise SystemExit(f"no data block in Horizons reply:\n{result[:800]}") from exc
    rows = []
    for line in body.splitlines():
        parts = [p.strip() for p in line.split(",") if p.strip()]
        if len(parts) < 8:
            continue
        # jd, calendar-date, then the six state components.
        rows.append((float(parts[0]), [float(p) for p in parts[2:8]]))
    return rows


def chunk_bounds(start: str, stop: str) -> list[tuple[str, str]]:
    """`(start, stop)` pairs covering `start..stop`, none longer than CHUNK_YEARS.

    Horizons truncates very long tables, so a fifty-year one-day request has to
    be split. A span inside a single year (the fine-cadence validation fixtures)
    is one chunk — the year-stepping loop would produce none at all.
    """
    start_year, stop_year = int(start[:4]), int(stop[:4])
    if stop_year <= start_year:
        return [(start, stop)]
    bounds = []
    for year in range(start_year, stop_year, CHUNK_YEARS):
        bounds.append(
            (
                start if year == start_year else f"{year}-01-01",
                f"{min(year + CHUNK_YEARS, stop_year)}-01-01",
            )
        )
    # The loop lands on whole years; carry the caller's real end date.
    bounds[-1] = (bounds[-1][0], stop)
    return bounds


def fetch_target(target: dict, start: str, stop: str, step: str, step_seconds: float) -> dict:
    """Every chunk for one target, stitched into one uniformly sampled table."""
    rows: list[tuple[float, list[float]]] = []
    for chunk_start, chunk_stop in chunk_bounds(start, stop):
        print(f"  {target['slug']}: {chunk_start} .. {chunk_stop} @ {step}", flush=True)
        chunk = parse_rows(fetch_chunk(target["number"], chunk_start, chunk_stop, step))
        # Chunk boundaries are inclusive at both ends, so the first row of each
        # chunk after the first repeats the previous chunk's last row. Dropping
        # it here is what keeps the cadence uniform — and uniform cadence is the
        # file format's one structural assumption.
        if rows and chunk and chunk[0][0] == rows[-1][0]:
            chunk = chunk[1:]
        rows.extend(chunk)

    if len(rows) < 4:
        raise SystemExit(f"{target['slug']}: only {len(rows)} samples, refusing to write")

    # Verify the uniform-cadence assumption instead of trusting it: the consumer
    # addresses samples as t0 + i*step, so a single ragged step would silently
    # shear the whole trajectory in time.
    for (jd_a, _), (jd_b, _) in zip(rows, rows[1:]):
        gap = (jd_b - jd_a) * SECONDS_PER_DAY
        if abs(gap - step_seconds) > 1e-3:
            raise SystemExit(
                f"{target['slug']}: non-uniform cadence — {gap} s between "
                f"JD {jd_a} and {jd_b}, expected {step_seconds} s"
            )

    header = {
        "name": target["name"],
        "designation": str(target["number"]),
        "naif_id": target["naif_id"],
        "source": "JPL-Horizons-VECTORS",
        "center": "SUN",
        "frame": "ICRF_J2000",
        "units": "km km/s",
        "t0_tdb_seconds": repr((rows[0][0] - JD_J2000) * SECONDS_PER_DAY),
        "step_seconds": repr(step_seconds),
        "n_samples": len(rows),
    }
    return {"header": header, "rows": rows}


def render(table: dict) -> str:
    """The on-disk text form: key/value header, `states`, then one row per line.

    Floats go through ``repr`` (shortest round-trip) rather than a fixed format,
    so a parse on the Rust side recovers the exact double Horizons sent — a
    truncated decimal here would be an invented trajectory there.
    """
    lines = [f"{FORMAT_MAGIC} {FORMAT_VERSION}"]
    lines += [f"{key} {value}" for key, value in table["header"].items()]
    lines.append("states")
    lines += [" ".join(repr(component) for component in state) for _, state in table["rows"]]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("numbers", nargs="*", type=int, help="IAU numbers (default: all)")
    parser.add_argument("--out", type=pathlib.Path, help="destination directory")
    parser.add_argument("--start", default=DEFAULT_START)
    parser.add_argument("--stop", default=DEFAULT_STOP)
    parser.add_argument(
        "--step",
        default=STEP,
        help="Horizons STEP_SIZE, e.g. 1d or 1h (default: %(default)s). "
        "The fine-cadence validation fixtures are fetched with 1h.",
    )
    parser.add_argument(
        "--suffix",
        default="",
        help="appended to the output filename, e.g. --suffix _flyby_1h",
    )
    args = parser.parse_args()

    step_seconds = step_to_seconds(args.step)

    out = args.out or pathlib.Path(__file__).resolve().parents[1].parent / (
        "temp/AsteroidDefense/kernels/neo"
    )
    out.mkdir(parents=True, exist_ok=True)

    targets = TARGETS
    if args.numbers:
        targets = [t for t in TARGETS if t["number"] in args.numbers]
        if not targets:
            raise SystemExit(f"no known target among {args.numbers}")

    for target in targets:
        print(f"{target['name']}:", flush=True)
        table = fetch_target(target, args.start, args.stop, args.step, step_seconds)
        path = out / f"{target['slug']}{args.suffix}.neo"
        path.write_text(render(table), encoding="utf-8")
        n_samples = table["header"]["n_samples"]
        span_years = n_samples * step_seconds / (365.25 * SECONDS_PER_DAY)
        print(
            f"  wrote {path} - {n_samples} samples "
            f"(~{span_years:.0f} yr), {path.stat().st_size / 1e6:.1f} MB\n",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
