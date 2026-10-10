#!/usr/bin/env python3
"""Fetch JPL Horizons' own sky positions for Apophis — the oracle for
``core/src/astrometry.rs``.

**What is checked, and what is deliberately not.** The core turns a target's
barycentric trajectory and Earth's DE440 position into a right ascension and
declination as seen from the geocentre. The only thing worth checking is *that
step*: the light-time iteration and the direction it produces. So the fixture
carries, for each date,

* Horizons' **astrometric** RA/Dec (quantity 1) and range (quantity 20) from the
  geocentre (``500@399``) — "compensated for down-leg light-time delay
  aberration", nothing else: no annual aberration, no light deflection. That is
  the same convention as a star catalog's positions, which is why the screen
  uses it (see ``docs/plans/2026-10-10-sky-observation-screen.md``);
* Apophis' **barycentric** state (``500@0``, ICRF) at the *same* instant, from
  the same Horizons solution.

The test feeds the second into the core and compares with the first. Using
Horizons' own state rather than the shipped ``.neo`` table keeps interpolation
error out of the gate: the light-time step needs the target at ``t - tau`` with
``tau`` up to ~15 minutes, and the test steps back from the state at ``t``
along a parabola under the Sun's pull (a straight line is ~2 km off at that
``tau`` — the Sun pulls ~6 mm/s^2 at 1 au — which was measured as a false
~1 mas disagreement before it was fixed).

**Times.** Both tables are requested in TT (observer tables do not offer TDB).
The test converts with hifitime.

Usage::

    python pyref/fetch_horizons_astrometry.py   # writes the committed fixture

The output is small and committed: ``core/tests/fixtures/apophis_astrometry.txt``.
"""

from __future__ import annotations

import json
import pathlib
import urllib.parse
import urllib.request

API = "https://ssd.jpl.nasa.gov/api/horizons.api"
OUT = pathlib.Path(__file__).resolve().parent.parent / "core" / "tests" / "fixtures" / "apophis_astrometry.txt"

# Julian dates, TT. A spread of ordinary geometry (near and far, both sides of
# the Sun, high and low declination) plus the 2029 flyby, where the rock moves
# tens of degrees in hours and the light time is a fraction of a second.
EPOCHS_JD_TT = [
    2459280.5,  # 2021-03-06, the 0.11 au approach
    2459450.5,  # 2021-08-23, far side
    2459700.5,  # 2022-04-30
    2460000.5,  # 2023-02-25
    2460400.5,  # 2024-03-31
    2460800.5,  # 2025-05-04
    2461200.5,  # 2026-06-08
    2461600.5,  # 2027-07-13
    2462000.5,  # 2028-08-16
    2462239.75,  # 2029-04-13 06:00
    2462240.25,  # 2029-04-13 18:00
    2462240.40625,  # 2029-04-13 21:45, near closest approach
    2462240.4375,  # 2029-04-13 22:30
    2462240.5,  # 2029-04-14 00:00
    2462242.5,  # 2029-04-16
    2464500.5,  # 2035-07-25
]

COMMON = {
    "format": "json",
    "COMMAND": "'99942;'",  # trailing ';' = small-body lookup
    "CSV_FORMAT": "YES",
    "OBJ_DATA": "NO",
    "TLIST_TYPE": "JD",
    "TIME_TYPE": "TT",
}


def query(params: dict) -> str:
    url = f"{API}?{urllib.parse.urlencode({**COMMON, **params})}"
    with urllib.request.urlopen(url, timeout=300) as response:
        payload = json.load(response)
    if "result" not in payload:
        raise SystemExit(f"Horizons returned no result: {payload}")
    return payload["result"]


def rows(result: str) -> list[list[str]]:
    body = result[result.index("$$SOE") + 5 : result.index("$$EOE")]
    return [[p.strip() for p in line.split(",")] for line in body.splitlines() if line.strip()]


def main() -> None:
    tlist = " ".join(f"'{jd}'" for jd in EPOCHS_JD_TT)

    observer = rows(
        query(
            {
                "EPHEM_TYPE": "OBSERVER",
                "CENTER": "'500@399'",  # geocentre
                "QUANTITIES": "'1,20'",
                "ANG_FORMAT": "DEG",
                "EXTRA_PREC": "YES",
                "TLIST": tlist,
            }
        )
    )
    vectors = rows(
        query(
            {
                "EPHEM_TYPE": "VECTORS",
                "CENTER": "'500@0'",  # solar-system barycentre
                "REF_PLANE": "FRAME",
                "REF_SYSTEM": "ICRF",
                "VEC_TABLE": "2",
                "OUT_UNITS": "KM-S",
                "VEC_LABELS": "NO",
                "TLIST": tlist,
            }
        )
    )
    if len(observer) != len(EPOCHS_JD_TT) or len(vectors) != len(EPOCHS_JD_TT):
        raise SystemExit(f"expected {len(EPOCHS_JD_TT)} rows, got {len(observer)} / {len(vectors)}")

    lines = [
        "# asteroid-astrometry-oracle 1",
        "# 99942 Apophis, JPL Horizons, fetched by pyref/fetch_horizons_astrometry.py",
        "# observer 500@399 (geocentre), quantity 1 = astrometric RA/Dec (ICRF, light-time only)",
        "# state 500@0 (SSB), ICRF, km and km/s, at the same TT instant",
        "# jd_tt ra_deg dec_deg delta_au x y z vx vy vz",
    ]
    for jd, obs, vec in zip(EPOCHS_JD_TT, observer, vectors):
        if abs(float(vec[0]) - jd) > 1e-9:
            raise SystemExit(f"vector row {vec[0]} is not epoch {jd}")
        ra, dec, delta = obs[3], obs[4], obs[5]
        state = vec[2:8]
        lines.append(" ".join([f"{jd:.6f}", ra, dec, delta, *state]))

    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {OUT} ({len(EPOCHS_JD_TT)} epochs)")


if __name__ == "__main__":
    main()
