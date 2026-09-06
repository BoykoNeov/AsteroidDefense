#!/usr/bin/env python3
"""Fetch a real asteroid's orbit-determination covariance from the JPL SBDB.

Everything in ``core/src/uncertainty.rs`` maps a 6x6 covariance on the asteroid's
Cartesian state through the dynamics to a b-plane ellipse and an impact
probability. Until now the only covariance available was
``StateCovariance::synthetic_along_track`` — *invented*, and labelled as such,
because the shipping threat is a designed rock with no observation arc. This
script fetches the real thing for a real object, so the pipeline can be driven by
a number that came from astrometry instead of from taste.

**What the SBDB actually publishes, which is not what you would guess.**

* The covariance is in the **cometary** element set ``COM``:
  ``(e, q, tp, node, peri, i)`` — eccentricity, perihelion distance, time of
  perihelion passage, longitude of ascending node, argument of perihelion,
  inclination. Not Keplerian ``(a, e, i, Om, w, M)`` and not equinoctial. Every
  object sampled while writing this (Apophis, Bennu, Didymos, Eros) used ``COM``.
* The units are **mixed**: dimensionless, au, **days** (``tp`` is a Julian date,
  so its variance is in days squared), and **degrees** for all three angles.
* The matrix is often **larger than 6x6**. Apophis' is 8x8 — the two
  non-gravitational acceleration parameters ``A1``/``A2`` are solved for
  alongside the orbit and appear as extra rows and columns. Bennu's carries
  ``RHO``/``AMRAT`` instead. Dropping those trailing rows/columns is exactly
  marginalisation for a Gaussian, so the leading 6x6 block is the correct
  covariance *of the orbit alone*, with the non-grav uncertainty already folded
  in. This script records which labels it dropped rather than silently trimming.
* The covariance has **its own epoch**, which is generally *not* the orbit's
  osculating-element epoch. Apophis' covariance is at JD 2459215.5 (2020-12-17)
  while its elements are published at JD 2461200.5 (2026-06-01). Moving a
  covariance between epochs needs a state-transition matrix; the honest thing is
  to start the propagation where the covariance lives, so this file carries the
  element values **at the covariance epoch** (the API supplies them, under
  ``orbit.covariance.elements``) and never the ones at the orbit epoch.

**The free external gate.** JPL publishes a per-element 1-sigma alongside the
matrix, and it equals ``sqrt(diag)`` exactly. That is a check on parsing,
ordering and units that costs nothing and needs no physics, so this script
asserts it before writing and the Rust reader asserts it again on load. A
round-trip test could not catch a consistent unit error; this can.

**The second gate is a truth state.** A covariance is useless without knowing
that our reconstruction of the *mean* state from those elements is right, and a
degrees-vs-radians slip in the angles would sail through any elements->state->
elements round-trip. So the file also carries JPL's own Cartesian state at the
covariance epoch, fetched from Horizons in **both** frames the conversion passes
through — heliocentric ecliptic (what the elements are referred to) and
heliocentric ICRF (what the integrator runs in). The Rust side reconstructs the
state from the elements and compares against both, which pins the element
conversion and the obliquity rotation separately, with no kernels involved.

**Why plain text and not JSON.** Same reason as ``fetch_horizons_neo.py``:
``asteroid_core`` depends on anise, hifitime and nalgebra and nothing else, and
adding a JSON parser to the physics crate to read a ~2 KB data file would cross
that line for no benefit. The format is a key/value header followed by a
``covariance`` block of six rows.

Unlike the ``.neo`` tables, these files are **committed** — a couple of
kilobytes each, and the tests that read them are deliberately kernel-free.

Usage::

    python pyref/fetch_sbdb_covariance.py                 # Apophis, into fixtures
    python pyref/fetch_sbdb_covariance.py 101955          # a different object
    python pyref/fetch_sbdb_covariance.py --out DIR
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import urllib.parse
import urllib.request

SBDB_API = "https://ssd-api.jpl.nasa.gov/sbdb.api"
HORIZONS_API = "https://ssd.jpl.nasa.gov/api/horizons.api"

# The element set this reader understands, in the order the API delivers it. A
# different set is not a variation to accommodate on the fly — it is a different
# set of partial derivatives on the Rust side — so an object publishing one is
# refused here rather than written into a file that would be read hopefully.
COM_LABELS = ["e", "q", "tp", "node", "peri", "i"]

# Units of each COM element as the API delivers them. `tp` is a Julian date, so
# its unit is days and its variance is days squared.
COM_UNITS = ["none", "au", "d", "deg", "deg", "deg"]

# Targets known to this script. `naif_id` is Horizons' extended small-body
# numbering (20000000 + number) and is provenance only, as in the .neo tables.
TARGETS = {
    99942: {"slug": "apophis", "naif_id": 20099942},
    101955: {"slug": "bennu", "naif_id": 20101955},
    65803: {"slug": "didymos", "naif_id": 20065803},
    433: {"slug": "eros", "naif_id": 20000433},
}

# First line of every output file; anything else is refused at line one.
FORMAT_MAGIC = "asteroid-sbdb-covariance"
FORMAT_VERSION = 1

# How far `sqrt(diag)` may differ from the published sigma before this is treated
# as a parse error rather than as rounding. The published sigmas are given to
# ~4 significant figures for the non-grav parameters and to full precision for
# the six orbital elements, so this is loose enough for the former and far
# tighter than any real ordering or unit mistake.
SIGMA_RTOL = 1.0e-3


def fetch_json(url: str, what: str) -> dict:
    """A JSON reply, or a hard exit. A proxy's HTML error page fails the decode
    rather than being carried forward as an empty-looking record."""
    try:
        with urllib.request.urlopen(url, timeout=120) as response:
            raw = response.read()
    except Exception as exc:  # noqa: BLE001 - any transport failure is fatal here
        raise SystemExit(f"{what}: request failed: {exc!r}") from exc
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        head = raw[:200].decode("utf-8", "replace")
        raise SystemExit(f"{what}: reply is not JSON (proxy page?): {head!r}") from exc


def fetch_sbdb(number: int) -> dict:
    """The SBDB record for one numbered object, with the covariance matrix."""
    query = {"sstr": str(number), "cov": "mat", "full-prec": "true"}
    url = f"{SBDB_API}?{urllib.parse.urlencode(query)}"
    payload = fetch_json(url, f"SBDB {number}")
    if "orbit" not in payload or "object" not in payload:
        raise SystemExit(f"SBDB {number}: no orbit/object block: {list(payload)}")
    if not payload["orbit"].get("covariance"):
        raise SystemExit(
            f"SBDB {number}: no covariance published (an object with too short "
            f"an observation arc has none — this script has nothing to fetch)"
        )
    return payload


def fetch_horizons_state(number: int, jd_tdb: float, ref_plane: str) -> list[float]:
    """JPL's own heliocentric state at one epoch, km and km/s.

    `ref_plane` is Horizons' own spelling: `ECLIPTIC` for the frame the SBDB
    elements are referred to, `FRAME` for ICRF equatorial.
    """
    query = {
        "format": "json",
        # The trailing semicolon marks a small-body lookup; without it the
        # number is searched for as a major body and not found.
        "COMMAND": f"'{number};'",
        "EPHEM_TYPE": "VECTORS",
        "CENTER": "'500@10'",  # Sun body centre
        "REF_PLANE": ref_plane,
        "REF_SYSTEM": "ICRF",
        "VEC_TABLE": "2",  # position + velocity
        "OUT_UNITS": "KM-S",
        "CSV_FORMAT": "YES",
        "VEC_LABELS": "NO",
        "OBJ_DATA": "NO",
        "TLIST": f"'{jd_tdb!r}'",
        "TLIST_TYPE": "JD",
    }
    url = f"{HORIZONS_API}?{urllib.parse.urlencode(query)}"
    payload = fetch_json(url, f"Horizons {number} @ {jd_tdb}")
    result = payload.get("result")
    if not result:
        raise SystemExit(f"Horizons {number}: no result block: {list(payload)}")
    try:
        body = result[result.index("$$SOE") + 5 : result.index("$$EOE")]
    except ValueError as exc:
        raise SystemExit(f"Horizons {number}: no data block:\n{result[:800]}") from exc
    rows = []
    for line in body.splitlines():
        parts = [p.strip() for p in line.split(",") if p.strip()]
        if len(parts) >= 8:
            rows.append((float(parts[0]), [float(p) for p in parts[2:8]]))
    if len(rows) != 1:
        raise SystemExit(f"Horizons {number}: expected one row, got {len(rows)}")
    got_jd, state = rows[0]
    # Horizons rounds the echoed JD; a whole-second slip would be a metres-level
    # error in the truth state and must not pass silently.
    if abs(got_jd - jd_tdb) > 1.0e-6:
        raise SystemExit(
            f"Horizons {number}: asked for JD {jd_tdb} and got {got_jd} "
            f"({abs(got_jd - jd_tdb) * 86400.0:.3f} s off)"
        )
    return state


def extract(payload: dict, number: int) -> dict:
    """Validate the SBDB record and reduce it to what the file carries."""
    orbit = payload["orbit"]
    cov = orbit["covariance"]

    labels = cov.get("labels")
    data = cov.get("data")
    elements = cov.get("elements")
    if not labels or not data or not elements:
        raise SystemExit(f"SBDB {number}: covariance block missing labels/data/elements")

    if labels[:6] != COM_LABELS:
        raise SystemExit(
            f"SBDB {number}: covariance is in element set {labels[:6]}, not the "
            f"cometary set {COM_LABELS} this pipeline converts. Refusing rather "
            f"than writing a file whose partial derivatives would be wrong."
        )
    if [e["label"] for e in elements] != COM_LABELS:
        raise SystemExit(
            f"SBDB {number}: covariance.elements are "
            f"{[e['label'] for e in elements]}, not {COM_LABELS}"
        )
    n = len(data)
    if n < 6 or any(len(row) != n for row in data):
        raise SystemExit(f"SBDB {number}: covariance data is not square ({n} rows)")

    matrix = [[float(x) for x in row] for row in data]

    # Gate 1: the published per-element sigma must equal sqrt(diag). This is the
    # check that catches a wrong ordering or a units misread, and it is free.
    for i, element in enumerate(elements):
        want = float(element["sigma"])
        got = math.sqrt(matrix[i][i])
        if want <= 0.0 or abs(got - want) > SIGMA_RTOL * want:
            raise SystemExit(
                f"SBDB {number}: sqrt(diag) for {element['label']} is {got!r} but "
                f"the published sigma is {want!r} — the matrix is not what the "
                f"element block describes"
            )

    # The matrix arrives exactly symmetric (measured: worst asymmetry 0.0 on
    # Apophis). Assert it rather than symmetrising, so a future API change that
    # starts returning an upper triangle is a loud failure and not a quiet one.
    scale = max(abs(matrix[i][j]) for i in range(n) for j in range(n))
    worst = max(abs(matrix[i][j] - matrix[j][i]) for i in range(n) for j in range(n))
    if worst > 1.0e-12 * scale:
        raise SystemExit(
            f"SBDB {number}: covariance is not symmetric as delivered "
            f"(worst {worst!r} against scale {scale!r})"
        )

    cov_epoch = float(cov["epoch"])
    return {
        "number": number,
        "fullname": payload["object"].get("fullname", str(number)).strip(),
        "orbit_id": orbit.get("orbit_id", "?"),
        "soln_date": orbit.get("soln_date", "?"),
        "producer": orbit.get("producer", "?"),
        "sb_used": orbit.get("sb_used", "?"),
        "pe_used": orbit.get("pe_used", "?"),
        "equinox": orbit.get("equinox", "?"),
        "first_obs": orbit.get("first_obs", "?"),
        "last_obs": orbit.get("last_obs", "?"),
        "n_obs_used": orbit.get("n_obs_used", "?"),
        "cov_epoch": cov_epoch,
        "orbit_epoch": float(orbit["epoch"]),
        "marginalized": labels[6:],
        "values": [float(e["value"]) for e in elements],
        "sigmas": [float(e["sigma"]) for e in elements],
        "matrix6": [row[:6] for row in matrix[:6]],
    }


def render(record: dict, target: dict, truth_ecl: list[float], truth_icrf: list[float]) -> str:
    """The on-disk text. Full `repr` precision throughout: these are the inputs to
    a covariance, and a rounded matrix stops being positive definite."""
    out: list[str] = [f"{FORMAT_MAGIC} {FORMAT_VERSION}"]

    def kv(key: str, value: object) -> None:
        out.append(f"{key} {value}")

    kv("name", record["fullname"])
    kv("designation", record["number"])
    kv("naif_id", target["naif_id"])
    kv("source", "JPL-SBDB")
    kv("orbit_id", record["orbit_id"])
    kv("soln_date", record["soln_date"])
    kv("producer", record["producer"])
    kv("sb_used", record["sb_used"])
    kv("pe_used", record["pe_used"])
    kv("equinox", record["equinox"])
    kv("obs_arc", f"{record['first_obs']} {record['last_obs']} {record['n_obs_used']}")
    kv("element_set", "COM")
    kv("center", "SUN")
    kv("frame", "ECLIPJ2000")
    kv("labels", " ".join(COM_LABELS))
    kv("units", " ".join(COM_UNITS))
    kv("marginalized", " ".join(record["marginalized"]) if record["marginalized"] else "none")
    kv("cov_epoch_jd_tdb", repr(record["cov_epoch"]))
    kv("orbit_epoch_jd_tdb", repr(record["orbit_epoch"]))
    kv("elements", " ".join(repr(v) for v in record["values"]))
    kv("sigmas", " ".join(repr(v) for v in record["sigmas"]))
    kv("truth_units", "km km/s")
    kv("truth_state_ecliptic", " ".join(repr(v) for v in truth_ecl))
    kv("truth_state_icrf", " ".join(repr(v) for v in truth_icrf))
    out.append("covariance")
    for row in record["matrix6"]:
        out.append(" ".join(repr(v) for v in row))
    return "\n".join(out) + "\n"


def main(argv: list[str] | None = None) -> int:
    default_out = pathlib.Path(__file__).resolve().parent.parent / "core" / "tests" / "fixtures"
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "numbers",
        nargs="*",
        type=int,
        default=[99942],
        help="IAU minor-planet numbers (default: 99942 Apophis)",
    )
    parser.add_argument("--out", type=pathlib.Path, default=default_out, help="destination dir")
    args = parser.parse_args(argv)

    args.out.mkdir(parents=True, exist_ok=True)
    for number in args.numbers:
        target = TARGETS.get(number, {"slug": str(number), "naif_id": 20000000 + number})
        print(f"{number}: SBDB covariance", flush=True)
        record = extract(fetch_sbdb(number), number)
        jd = record["cov_epoch"]
        print(f"  covariance epoch JD {jd} TDB (orbit epoch {record['orbit_epoch']})", flush=True)
        if record["marginalized"]:
            print(f"  marginalizing over {' '.join(record['marginalized'])}", flush=True)
        print("  Horizons truth state, ecliptic", flush=True)
        truth_ecl = fetch_horizons_state(number, jd, "ECLIPTIC")
        print("  Horizons truth state, ICRF", flush=True)
        truth_icrf = fetch_horizons_state(number, jd, "FRAME")

        path = args.out / f"{target['slug']}.sbdb"
        path.write_text(render(record, target, truth_ecl, truth_icrf), encoding="ascii")
        print(f"  wrote {path} ({path.stat().st_size} bytes)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
