#!/usr/bin/env python3
"""Fetch Horizons' hour-by-hour view of Apophis from Mt. Lemmon across its March
2021 approach — the oracle for ``core/src/sky_shot.rs``.

The screen's first scenario photographs Apophis from Mt. Lemmon Survey (G96)
over several nights around its 2021-03-06 approach (0.11 au). Each thing the
shot generator computes for itself has a Horizons column to be checked against:

* position: quantity 1 (astrometric RA/Dec) — the rock as the shot draws it,
  read from the shipped ``.neo`` table rather than Horizons' own state, so this
  also checks the table's interpolation in the window the game uses;
* motion: quantity 3 (RA·cos Dec and Dec rates, arcsec/hour);
* brightness: quantity 9 (APmag, the IAU H-G magnitude, airless);
* darkness: the solar-presence flag (``*`` day, ``C``/``N``/``A`` twilight,
  blank night) and quantity 4's elevation, for the "can we observe now" test.

Usage::

    python pyref/fetch_horizons_night_sky.py

Writes the small committed fixture
``core/tests/fixtures/apophis_2021_g96.txt``.
"""

from __future__ import annotations

import json
import pathlib
import urllib.parse
import urllib.request

API = "https://ssd.jpl.nasa.gov/api/horizons.api"
OUT = pathlib.Path(__file__).resolve().parent.parent / "core" / "tests" / "fixtures" / "apophis_2021_g96.txt"


def main() -> None:
    query = {
        "format": "json",
        "COMMAND": "'99942;'",
        "EPHEM_TYPE": "OBSERVER",
        "CENTER": "'G96@399'",
        "QUANTITIES": "'1,3,4,9,20'",
        "ANG_FORMAT": "DEG",
        "EXTRA_PREC": "YES",
        "CSV_FORMAT": "YES",
        "OBJ_DATA": "YES",
        "CAL_FORMAT": "JD",
        "TIME_TYPE": "TT",
        "START_TIME": "2021-03-01 00:00",
        "STOP_TIME": "2021-03-12 00:00",
        "STEP_SIZE": "1h",
    }
    with urllib.request.urlopen(f"{API}?{urllib.parse.urlencode(query)}", timeout=300) as r:
        result = json.load(r)["result"]

    header = result[result.index("Asteroid physical parameters") :].splitlines()[2]
    body = result[result.index("$$SOE") + 5 : result.index("$$EOE")]
    lines = [
        "# asteroid-night-sky-oracle 1",
        "# 99942 Apophis from G96 (Mt. Lemmon Survey), JPL Horizons, pyref/fetch_horizons_night_sky.py",
        f"# physical: {header.strip()}",
        "# jd_tt sun_flag(-=night) moon_flag(-=none) ra_deg dec_deg dra_cosd_arcsec_h ddec_arcsec_h az_deg el_deg apmag delta_au",
    ]
    for row in body.splitlines():
        p = [x.strip() for x in row.split(",")]
        if len(p) < 13:
            continue
        sun = p[1] or "-"
        moon = p[2] or "-"
        lines.append(" ".join([p[0], sun, moon, p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[11]]))
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {OUT} ({len(lines) - 4} rows)")


if __name__ == "__main__":
    main()
