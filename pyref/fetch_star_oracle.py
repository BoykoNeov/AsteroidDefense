#!/usr/bin/env python3
"""Fetch the outside answers ``core/tests/star_catalog_vs_cds.rs`` checks against.

Two independent services, neither of which shares code with the packer
(``tools/fetch_tycho2.py``) or the reader (``core/src/star_catalog.rs``):

* **VizieR cone searches** on Tycho-2 (main catalogue and supplement 1): the
  Tycho identifiers CDS itself finds within a radius of three centres — an
  ordinary field (the Pleiades), one straddling RA 0h, and one over the north
  celestial pole. The reader's cell zoning, RA wrap and pole handling either
  find the same stars or they do not.
* **SIMBAD** positions (ICRS, epoch J2000) for four named stars — the
  Hipparcos-based positions the names are supposed to point at.

Usage::

    python pyref/fetch_star_oracle.py   # writes core/tests/fixtures/star_oracle.txt
"""

from __future__ import annotations

import csv
import io
import pathlib
import urllib.parse
import urllib.request

OUT = pathlib.Path(__file__).resolve().parent.parent / "core" / "tests" / "fixtures" / "star_oracle.txt"
VIZIER = "https://vizier.cds.unistra.fr/viz-bin/asu-tsv"
SIMBAD_TAP = "https://simbad.cds.unistra.fr/simbad/sim-tap/sync"

# (label, RA deg, Dec deg, radius deg)
CONES = [
    ("pleiades", 56.75, 24.1167, 0.5),
    ("ra0h", 0.1, 10.0, 0.6),
    ("pole", 0.0, 90.0, 0.4),
]
NAMED = ["Vega", "Sirius", "Polaris", "Betelgeuse"]


def get(url: str) -> str:
    with urllib.request.urlopen(url, timeout=300) as r:
        return r.read().decode("utf-8")


def cone(source: str, ra: float, dec: float, radius: float) -> list[str]:
    query = {
        "-source": source,
        "-c": f"{ra} {dec:+}",
        "-c.eq": "J2000",
        "-c.rd": f"{radius}",
        "-out": "TYC1,TYC2,TYC3",
        "-out.max": "unlimited",
    }
    text = get(f"{VIZIER}?{urllib.parse.urlencode(query)}")
    ids = []
    for line in text.splitlines():
        parts = line.split("\t")
        if len(parts) == 3 and all(p.strip().isdigit() for p in parts):
            ids.append("-".join(p.strip() for p in parts))
    return ids


def simbad(name: str) -> tuple[float, float]:
    adql = (
        "SELECT ra, dec FROM basic JOIN ident ON ident.oidref = basic.oid "
        f"WHERE ident.id = 'NAME {name}' OR ident.id = '{name}'"
    )
    query = {"request": "doQuery", "lang": "adql", "format": "csv", "query": adql}
    rows = list(csv.reader(io.StringIO(get(f"{SIMBAD_TAP}?{urllib.parse.urlencode(query)}"))))
    if len(rows) < 2:
        raise SystemExit(f"SIMBAD has no position for {name}")
    return float(rows[1][0]), float(rows[1][1])


def main() -> None:
    lines = [
        "# asteroid-star-oracle 1",
        "# cone <label> <ra_deg> <dec_deg> <radius_deg> then the Tycho ids VizieR finds (I/259 tyc2 + suppl_1)",
        "# star <name> <ra_deg> <dec_deg>   SIMBAD, ICRS, epoch J2000",
    ]
    for label, ra, dec, radius in CONES:
        ids = cone("I/259/tyc2", ra, dec, radius) + cone("I/259/suppl_1", ra, dec, radius)
        print(f"{label}: {len(ids)} stars")
        lines.append(f"cone {label} {ra} {dec} {radius} {' '.join(sorted(ids))}")
    for name in NAMED:
        ra, dec = simbad(name)
        print(f"{name}: {ra} {dec}")
        lines.append(f"star {name} {ra!r} {dec!r}")
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
