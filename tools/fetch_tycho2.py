#!/usr/bin/env python3
"""Fetch the Tycho-2 star catalogue and star names, and pack them for the core.

The sky-observation screen draws real background stars
(``docs/plans/2026-10-10-sky-observation-screen.md``). They come from:

* **Tycho-2** (Høg et al. 2000, A&A 355, L27; CDS catalogue I/259): 2 539 913
  stars, positions on the ICRS at epoch J2000 with proper motions, complete to
  V ≈ 11.5. Plus **supplement 1** (17 588 Hipparcos/Tycho-1 stars missing from
  the main file — this is where most of the very brightest stars live), whose
  positions are at J1991.25 and are carried to J2000 here.
* **Star names**: the HD-DM-GC-HR-HIP-Bayer-Flamsteed Cross Index (Kostjuk
  2002; CDS catalogue IV/27A) — Bayer letters, Flamsteed numbers and
  constellations for 3 690 stars, and proper names for 345 of them (the first
  name the table lists for a star is taken).

All from the CDS archive at Strasbourg, which asks that use be acknowledged;
the screen shows the credit line. The packed files land in
``kernels/stars/`` beside the DE kernels and are **gitignored**, like them —
~60 MB, regenerable, absent on a fresh clone.

**The packed format** (``tycho2.stars``, little-endian), read by
``core/src/star_catalog.rs``::

    8 bytes   magic b"ASTSTARS"
    u32       format version (1)
    u32       number of stars
    f32       the largest proper motion in the file, mas/yr (for query padding)
    u32 x 64801  offsets: stars in 1°x1° cell c are records [off[c], off[c+1]),
                 c = floor(dec + 90) * 360 + floor(ra), J2000 position, dec 90
                 folded into the top row
    24-byte records, sorted by cell:
        u32 ra   J2000, units of 1e-7 deg
        i32 dec  J2000, units of 1e-7 deg
        f32 pm_ra_cosdec  mas/yr (0 when the catalogue gives none)
        f32 pm_dec        mas/yr
        i16 v    V magnitude, millimag (see ``v_mag``)
        u16 tyc1, u16 tyc2, u8 tyc3   the Tycho identifier "TYC tyc1-tyc2-tyc3"
        u8  flags  bit0 supplement-1 star, bit1 no proper motion known

``names.txt``: ``tyc1-tyc2-tyc3|bayer|flamsteed|constellation|proper name``,
one per named star, joined through the Hipparcos number.

Usage::

    python tools/fetch_tycho2.py               # into <repo>/kernels/stars
    python tools/fetch_tycho2.py --dest DIR
    python tools/fetch_tycho2.py --keep-raw    # keep the downloaded .gz files
"""

from __future__ import annotations

import argparse
import gzip
import math
import pathlib
import shutil
import struct
import sys
import urllib.request

REPO = pathlib.Path(__file__).resolve().parent.parent
TYC2 = "https://cdsarc.cds.unistra.fr/ftp/I/259/"
XIDX = "https://cdsarc.cds.unistra.fr/ftp/IV/27A/"
TYC2_PARTS = [f"tyc2.dat.{i:02d}.gz" for i in range(20)]

MAGIC = b"ASTSTARS"
VERSION = 1
CELLS = 360 * 180
RECORD = struct.Struct("<IiffhHHBB")
assert RECORD.size == 24

# Supplement-1 positions are at J1991.25 (ReadMe I/259); carry them to J2000.
SUPPL_EPOCH = 1991.25
MAS_PER_DEG = 3_600_000.0


def log(msg: str) -> None:
    print(f"[fetch_tycho2] {msg}", flush=True)


def download(url: str, path: pathlib.Path) -> None:
    if path.exists() and path.stat().st_size > 0:
        return
    log(f"downloading {url}")
    tmp = path.with_suffix(path.suffix + ".part")
    with urllib.request.urlopen(url, timeout=600) as r, open(tmp, "wb") as f:
        shutil.copyfileobj(r, f)
    tmp.replace(path)


def field(line: str, a: int, b: int) -> str:
    """Bytes a..b (1-based, inclusive) of a CDS fixed-width record."""
    return line[a - 1 : b].strip()


def opt_float(s: str) -> float | None:
    return float(s) if s else None


def v_mag(bt: float | None, vt: float | None) -> float | None:
    """Johnson V from Tycho BT/VT: V = VT − 0.090 (BT − VT) (Tycho-2 guide /
    ESA 1997 vol. 1 §1.3). One band alone is used as it stands."""
    if bt is not None and vt is not None:
        return vt - 0.090 * (bt - vt)
    return vt if vt is not None else bt


def carry(ra: float, dec: float, pmra: float, pmde: float, years: float) -> tuple[float, float]:
    """Move a position by its proper motion (mas/yr, pmra already × cos dec) on the
    sphere, along the local east/north directions."""
    r, d = math.radians(ra), math.radians(dec)
    p = (math.cos(d) * math.cos(r), math.cos(d) * math.sin(r), math.sin(d))
    east = (-math.sin(r), math.cos(r), 0.0)
    north = (-math.sin(d) * math.cos(r), -math.sin(d) * math.sin(r), math.cos(d))
    k = math.radians(1.0 / MAS_PER_DEG) * years
    v = [p[i] + k * (pmra * east[i] + pmde * north[i]) for i in range(3)]
    n = math.sqrt(sum(x * x for x in v))
    v = [x / n for x in v]
    return math.degrees(math.atan2(v[1], v[0])) % 360.0, math.degrees(math.asin(v[2]))


def cell_of(ra: float, dec: float) -> int:
    row = min(int(math.floor(dec + 90.0)), 179)
    col = min(int(math.floor(ra)), 359)
    return row * 360 + col


def read_main(path: pathlib.Path, out: list, hip_of: dict) -> None:
    with gzip.open(path, "rt", encoding="ascii") as f:
        for line in f:
            tyc = (int(field(line, 1, 4)), int(field(line, 6, 10)), int(field(line, 12, 12)))
            pflag = line[13]
            bt, vt = opt_float(field(line, 111, 116)), opt_float(field(line, 124, 129))
            v = v_mag(bt, vt)
            if v is None:
                continue
            if pflag == "X":
                # No mean position: the observed Tycho-2 position (epoch ~1991)
                # with no proper motion known — kept, and flagged. 109 445 stars,
                # Polaris among them (379 mas from its J2000 position); the
                # reader never uses a flagged star as a measuring reference.
                ra, dec = float(field(line, 153, 164)), float(field(line, 166, 177))
                pmra = pmde = 0.0
                flags = 2
            else:
                ra, dec = float(field(line, 16, 27)), float(field(line, 29, 40))
                pmra = opt_float(field(line, 42, 48)) or 0.0
                pmde = opt_float(field(line, 50, 56)) or 0.0
                flags = 0
            hip = field(line, 143, 148)
            if hip:
                hip_of.setdefault(int(hip), tyc)
            out.append((ra, dec, pmra, pmde, v, tyc, flags))


def read_suppl(path: pathlib.Path, out: list, hip_of: dict) -> None:
    with gzip.open(path, "rt", encoding="ascii") as f:
        for line in f:
            tyc = (int(field(line, 1, 4)), int(field(line, 6, 10)), int(field(line, 12, 12)))
            ra, dec = float(field(line, 16, 27)), float(field(line, 29, 40))
            pmra, pmde = opt_float(field(line, 42, 48)), opt_float(field(line, 50, 56))
            flags = 1
            if pmra is None or pmde is None:
                pmra = pmde = 0.0
                flags |= 2
            else:
                ra, dec = carry(ra, dec, pmra, pmde, 2000.0 - SUPPL_EPOCH)
            bt, vt = opt_float(field(line, 84, 89)), opt_float(field(line, 97, 102))
            v = v_mag(bt, vt)
            if v is None:
                continue
            hip = field(line, 116, 121)
            if hip:
                hip_of.setdefault(int(hip), tyc)
            out.append((ra, dec, pmra, pmde, v, tyc, flags))


def write_stars(stars: list, path: pathlib.Path) -> None:
    stars.sort(key=lambda s: cell_of(s[0], s[1]))
    counts = [0] * CELLS
    for s in stars:
        counts[cell_of(s[0], s[1])] += 1
    offsets = [0]
    for c in counts:
        offsets.append(offsets[-1] + c)
    max_pm = max(math.hypot(s[2], s[3]) for s in stars)
    tmp = path.with_suffix(".part")
    with open(tmp, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<IIf", VERSION, len(stars), max_pm))
        f.write(struct.pack(f"<{CELLS + 1}I", *offsets))
        for ra, dec, pmra, pmde, v, tyc, flags in stars:
            f.write(
                RECORD.pack(
                    round(ra * 1e7) % 3_600_000_000,
                    round(dec * 1e7),
                    pmra,
                    pmde,
                    max(-32768, min(32767, round(v * 1000))),
                    tyc[0],
                    tyc[1],
                    tyc[2],
                    flags,
                )
            )
    tmp.replace(path)
    log(f"wrote {path} ({len(stars)} stars, max proper motion {max_pm:.0f} mas/yr)")


def write_names(raw: pathlib.Path, hip_of: dict, path: pathlib.Path) -> None:
    # catalog.dat: HD -> (HIP, Flamsteed, Bayer, constellation).
    by_hd: dict[int, tuple[int, str, str, str]] = {}
    for line in (raw / "catalog.dat").read_text(encoding="latin-1").splitlines():
        hip = field(line, 32, 37)
        if not hip:
            continue
        by_hd[int(field(line, 1, 6))] = (int(hip), field(line, 65, 67), field(line, 69, 73), field(line, 75, 77))
    # table3.dat: HD -> proper names, first listed wins.
    proper: dict[int, str] = {}
    for line in (raw / "table3.dat").read_text(encoding="latin-1").splitlines():
        hd = int(field(line, 1, 6))
        proper.setdefault(hd, field(line, 22, 76))
    lines = []
    for hd, (hip, fl, bayer, cst) in sorted(by_hd.items()):
        tyc = hip_of.get(hip)
        if tyc is None:
            continue
        name = proper.get(hd, "").replace("|", "/")
        lines.append(f"{tyc[0]}-{tyc[1]}-{tyc[2]}|{bayer}|{fl}|{cst}|{name}")
    path.write_text("# asteroid-star-names 1\n" + "\n".join(lines) + "\n", encoding="utf-8")
    log(f"wrote {path} ({len(lines)} named stars)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--dest", type=pathlib.Path, default=REPO / "kernels" / "stars")
    ap.add_argument("--keep-raw", action="store_true", help="keep the downloaded catalogue files")
    args = ap.parse_args()
    dest: pathlib.Path = args.dest
    raw = dest / "raw"
    raw.mkdir(parents=True, exist_ok=True)

    for name in TYC2_PARTS + ["suppl_1.dat.gz"]:
        download(TYC2 + name, raw / name)
    for name in ["catalog.dat", "table3.dat"]:
        download(XIDX + name, raw / name)

    stars: list = []
    hip_of: dict = {}
    for name in TYC2_PARTS:
        log(f"reading {name}")
        read_main(raw / name, stars, hip_of)
    read_suppl(raw / "suppl_1.dat.gz", stars, hip_of)
    write_stars(stars, dest / "tycho2.stars")
    write_names(raw, hip_of, dest / "names.txt")

    if not args.keep_raw:
        shutil.rmtree(raw)
        log("removed the raw downloads (use --keep-raw to keep them)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
