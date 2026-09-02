#!/usr/bin/env python3
"""Fetch the JPL kernels the physics suite needs into `kernels/`.

The core resolves a **pair** — a DE ephemeris `.bsp` and ANISE's planetary
constants `pck11.pca` — from `kernels/` beside the repo (see
`core/src/kernels.rs`). They are hundreds of megabytes at most, never committed,
and a fresh clone has neither, which is the "everything skipped and printed
green" trap that module exists to make loud. This script is the other half of
the fix: one command that puts the pair where the resolver looks.

Sources, tried in order, because the obvious one is not always reachable:

* DE ephemeris: `de440s.bsp` (32 MB, 1849–2150) from NAIF; failing that, the
  full `de440.bsp` (114 MB, 1550–2650) unpacked from the `naif-de440` PyPI wheel
  — a proxy that blocks naif.jpl.nasa.gov often lets pypi.org through, and the
  resolver accepts either filename.
* `pck11.pca`: the ANISE repository's copy via GitHub's LFS media endpoint;
  failing that, the nyx-space public-data host.

Optional: `--neo` also runs `pyref/fetch_horizons_neo.py` for the real-asteroid
state tables (`kernels/neo/*.neo`, ~1 MB) that the Horizons scenery and its
tests read. The 646 MB `sb441-n16.bsp` small-body kernel is not fetched here; it
is optional to the core and the Godot build worker mounts it only if present.

Every file is checked by its magic bytes before it is kept, so a proxy's HTML
error page can never be mistaken for a kernel.

Usage::

    python tools/fetch_kernels.py            # into <repo>/kernels
    python tools/fetch_kernels.py --neo      # plus the Horizons NEO tables
    python tools/fetch_kernels.py --dest DIR
"""

from __future__ import annotations

import argparse
import io
import pathlib
import subprocess
import sys
import urllib.error
import urllib.request
import zipfile

REPO = pathlib.Path(__file__).resolve().parent.parent

DE440S_URL = "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440s.bsp"
NAIF_DE440_WHEEL = "naif-de440"
PCK11_URLS = [
    "https://media.githubusercontent.com/media/nyx-space/anise/master/data/pck11.pca",
    "https://public-data.nyx-space.com/anise/v0.5/pck11.pca",
]


def log(msg: str) -> None:
    print(f"[fetch_kernels] {msg}", flush=True)


def download(url: str, timeout: int = 600) -> bytes | None:
    log(f"GET {url}")
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read()
    except (urllib.error.URLError, OSError, ValueError) as e:
        log(f"  failed: {e}")
        return None


def is_spk(data: bytes) -> bool:
    return data[:8] in (b"DAF/SPK ", b"DAF/SPK\x00")


def is_anise(data: bytes) -> bool:
    return b"ANISE" in data[:64]


def fetch_de(dest: pathlib.Path) -> bool:
    for name in ("de440s.bsp", "de440.bsp", "de441.bsp"):
        p = dest / name
        if p.is_file() and is_spk(p.read_bytes()[:8] + b"\0" * 8):
            log(f"{name} already present")
            return True
    data = download(DE440S_URL)
    if data and is_spk(data):
        (dest / "de440s.bsp").write_bytes(data)
        log(f"wrote de440s.bsp ({len(data) / 1e6:.1f} MB)")
        return True
    log("NAIF unreachable or served a non-SPK; trying the naif-de440 PyPI wheel")
    try:
        subprocess.run(
            [sys.executable, "-m", "pip", "download", NAIF_DE440_WHEEL, "--no-deps",
             "-d", str(dest), "-q"],
            check=True,
        )
    except (subprocess.CalledProcessError, OSError) as e:
        log(f"  pip download failed: {e}")
        return False
    wheels = sorted(dest.glob("naif_de440-*.whl"))
    if not wheels:
        log("  no wheel landed")
        return False
    with zipfile.ZipFile(wheels[-1]) as z:
        members = [m for m in z.namelist() if m.endswith("de440.bsp")]
        if not members:
            log("  wheel holds no de440.bsp")
            return False
        data = z.read(members[0])
    for w in wheels:
        w.unlink()
    if not is_spk(data):
        log("  wheel payload is not an SPK")
        return False
    (dest / "de440.bsp").write_bytes(data)
    log(f"wrote de440.bsp ({len(data) / 1e6:.1f} MB, full DE440 span)")
    return True


def fetch_pck(dest: pathlib.Path) -> bool:
    p = dest / "pck11.pca"
    if p.is_file() and is_anise(p.read_bytes()[:64]):
        log("pck11.pca already present")
        return True
    for url in PCK11_URLS:
        data = download(url, timeout=120)
        if data and is_anise(data):
            p.write_bytes(data)
            log(f"wrote pck11.pca ({len(data)} bytes)")
            return True
        if data:
            log("  not an ANISE file (an LFS pointer or an error page); trying the next source")
    return False


def fetch_neo(dest: pathlib.Path) -> bool:
    script = REPO / "pyref" / "fetch_horizons_neo.py"
    out = dest / "neo"
    out.mkdir(parents=True, exist_ok=True)
    log(f"running {script.relative_to(REPO)} --out {out}")
    try:
        subprocess.run([sys.executable, str(script), "--out", str(out)], check=True)
    except (subprocess.CalledProcessError, OSError) as e:
        log(f"  Horizons fetch failed: {e}")
        return False
    got = list(out.glob("*.neo"))
    log(f"{len(got)} .neo tables in {out}")
    return bool(got)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--dest", type=pathlib.Path, default=REPO / "kernels")
    ap.add_argument("--neo", action="store_true", help="also fetch the Horizons NEO tables")
    args = ap.parse_args()
    dest: pathlib.Path = args.dest
    dest.mkdir(parents=True, exist_ok=True)

    ok_de = fetch_de(dest)
    ok_pck = fetch_pck(dest)
    ok_neo = fetch_neo(dest) if args.neo else True

    if ok_de and ok_pck:
        log(f"kernel pair ready in {dest}")
        log("run:  ASTEROID_REQUIRE_KERNELS=1 cargo test --workspace --release")
    else:
        log("the pair is incomplete — the physics suite will SKIP and print green unless "
            "ASTEROID_REQUIRE_KERNELS=1 is set")
    return 0 if (ok_de and ok_pck and ok_neo) else 1


if __name__ == "__main__":
    sys.exit(main())
