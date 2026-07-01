#!/usr/bin/env python3
"""Task-0.5 de-risk spike, pillar (a): confirm ASSIST + the DE441-consistent
ephemeris build offline and can integrate a test particle in the DE field.

ASSIST's shipped configuration pairs the DE440 *planetary* file
(`linux_p1550p2650.440`, 1550-2650 span) with the DE441-consistent 16-asteroid
small-body file (`sb441-n16.bsp`) — this is the exact perturber field the Tier-1
Rust core mirrors as a test particle. The planetary reader is identical for the
full DE441 file; only the span/size differ.

What this proves for the spike:
  1. `rebound` and `assist` import (i.e. the C extensions built).
  2. an `assist.Ephem` loads the DE ephemeris.
  3. an `assist.Extras`-driven REBOUND sim integrates a test particle forward,
  4. and back — round-trip position error stays tiny (the integrator is sane).

Run under Linux/WSL (rebound/assist do not support native Windows):
    python3 -m venv .venv && . .venv/bin/activate
    pip install -r requirements.txt
    python3 spike_assist_de441.py

Data files are downloaded once to $ASSIST_DATA_DIR (default:
/mnt/m/claud_projects/temp/AsteroidDefense/kernels) and are git-ignored.
"""

import os
import sys
import urllib.request

DATA_DIR = os.environ.get(
    "ASSIST_DATA_DIR", "/mnt/m/claud_projects/temp/AsteroidDefense/kernels"
)

FILES = {
    "linux_p1550p2650.440": "https://ssd.jpl.nasa.gov/ftp/eph/planets/Linux/de440/linux_p1550p2650.440",
    "sb441-n16.bsp": "https://ssd.jpl.nasa.gov/ftp/eph/small_bodies/asteroids_de441/sb441-n16.bsp",
}

# Asteroid (3666) Holman, barycentric equatorial ICRF, AU and AU/day, at
# JD 2458849.5 (2020-01-01 TDB) — ASSIST's canonical example initial condition.
JD_REF = 2451545.0  # ASSIST default reference epoch (J2000)
T0 = 2458849.5 - JD_REF  # sim time is days since jd_ref
HOLMAN = dict(
    x=3.3388753502614090e00,
    y=-9.1765182678903168e-01,
    z=-5.0385906775843303e-01,
    vx=2.8056633343957200e-03,
    vy=7.5504086883996016e-03,
    vz=2.9800282074358684e-03,
)


def fail(msg):
    print(f"\nSPIKE PILLAR A (ASSIST + DE441): FAIL\n  {msg}")
    sys.exit(1)


def ensure_data():
    os.makedirs(DATA_DIR, exist_ok=True)
    paths = {}
    for name, url in FILES.items():
        path = os.path.join(DATA_DIR, name)
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            print(f"downloading {name} -> {path}")
            urllib.request.urlretrieve(url, path)
        print(f"  {name}: {os.path.getsize(path) / 1e6:.1f} MB")
        paths[name] = path
    return paths


def main():
    print(f"python {sys.version.split()[0]} on {sys.platform}")
    try:
        import rebound
        import assist
    except Exception as e:  # noqa: BLE001
        fail(f"import failed (C extension did not build?): {e!r}")
    print(f"rebound {rebound.__version__}, assist {assist.__version__}")

    paths = ensure_data()
    ephem = assist.Ephem(paths["linux_p1550p2650.440"], paths["sb441-n16.bsp"])
    print(f"ephem loaded; jd_ref = {ephem.jd_ref}")

    sim = rebound.Simulation()
    sim.t = T0
    sim.add(**HOLMAN)
    assist.Extras(sim, ephem)  # attaches the DE441 force model to the sim

    p0 = sim.particles[0].xyz
    print(f"t0={T0:.1f} d  r0={p0}")

    # Integrate forward 100 days, then back to t0.
    sim.integrate(T0 + 100.0)
    p1 = sim.particles[0].xyz
    print(f"t1={sim.t:.1f} d  r1={p1}")

    sim.integrate(T0)
    p2 = sim.particles[0].xyz
    # Round-trip position error in metres (1 AU = 1.495978707e11 m).
    au_m = 1.495978707e11
    err_m = (
        sum((a - b) ** 2 for a, b in zip(p0, p2)) ** 0.5
    ) * au_m
    print(f"round-trip back to t0  r2={p2}")
    print(f"round-trip position error: {err_m:.3e} m")

    moved_au = sum((a - b) ** 2 for a, b in zip(p0, p1)) ** 0.5
    if moved_au < 1e-4:
        fail(f"test particle barely moved ({moved_au:.2e} AU) — integration suspect")
    if err_m > 1.0:
        fail(f"round-trip error {err_m:.3e} m too large — integrator not reversible")

    print("\nSPIKE PILLAR A (ASSIST + DE441): PASS")
    print("  rebound+assist built, DE ephemeris loaded, test particle integrated")
    print(f"  forward+back over 100 d, round-trip error {err_m:.3e} m")


if __name__ == "__main__":
    main()
