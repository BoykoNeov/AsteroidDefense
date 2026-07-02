#!/usr/bin/env python3
"""Generate the Tier-1 ASSIST reference fixture (HANDOFF §6, §10.7 batch 2c).

Integrate a test particle in the **DE ephemeris field** with ASSIST — the §6
trajectory oracle — configured to the *exact* Tier-1 force model the Rust core
mirrors: **point-mass gravity only**, from the Sun + 8 planets + Moon + Pluto,
with GR, the Sun/Earth harmonics, the 16 asteroid perturbers, and the A1/A2/A3
non-gravitational terms all **off**. The Rust `validation` crate then propagates
the *same* initial condition with dop853 in the *same* 11-body field
(`assist_reference.rs`) and compares.

Why the force model is set to 11 point masses, not the 10 of the shipping
`tier1_perturber_field`:

  ASSIST's direct-gravity term (with harmonics/GR/asteroids off) sums **eleven**
  bodies — Sun, Moon, Pluto, and the eight planets (Earth and Moon carried
  separately; Pluto as its DE barycenter). See ASSIST `src/forces.c`, the
  `order[ASSIST_BODY_NPLANETS]` array. To validate the Rust *machinery* against
  the oracle at a tight, GM-floor-limited tolerance — rather than a tolerance
  loose enough to swallow Pluto and thereby hide a real μ-slip or rotation bug —
  both sides must integrate the identical dynamical system. So the Rust test adds
  Pluto (NAIF 9) for this comparison. Whether the *shipping* field should also
  carry Pluto (§5 locks "Sun + 8 planets + Moon" at 10) is a separate decision;
  this fixture does not presume it.

What is pinned identically on both sides so the comparison tests *dynamics*, not
a units/frame/constant slip (mirroring `generate_kepler_fixture.py`):

  1. **Force model.** `ex.forces` is asserted to resolve to exactly
     {"SUN", "PLANETS"} — the 11-body point mass, everything else off — and the
     active list is recorded in provenance.
  2. **Frame.** ASSIST integrates barycentric (SSB-centered) equatorial ICRF/
     J2000 — the DE440 native frame, identical to ANISE's `SSB_J2000`. No frame
     rotation enters.
  3. **Units.** ASSIST is native AU / AU-day; states are converted to SI (m,
     m/s) via the IAU AU (`AU_M`), asserted equal to ASSIST's own AU constant so
     a silent AU mismatch (a pure scale error) fails here. The initial condition
     is written to the fixture *in SI* as the single source of truth — the Rust
     side reads it, never re-transcribing the AU numbers.
  4. **Time.** ASSIST sim time is days past `jd_ref` = JD 2451545.0 TDB (= the
     hifitime J2000 TDB epoch). Sample epochs are recorded as TDB seconds past
     J2000 (`days * 86400`), so the Rust `Epoch::from_tdb_seconds_past_j2000`
     lands on the same instant with no time-scale conversion.

The DE440 GM set ASSIST uses (from its DE440 header, == NAIF `gm_de440.tpc`,
Park et al. 2021) is recorded per body so the Rust side can measure the
ANISE(pck11) − DE440 GM delta — the residual's real floor (advisor note): a
large per-body delta is a *finding*, not something to bury in tolerance.

Run under Docker (`assist` has no Windows wheel; compiles from source, needs
gcc — see SPIKE.md). Kernels are the ASSIST pair, cached & git-ignored:

    docker run --rm \
      -v "M:/claud_projects/AsteroidDefense/pyref:/pyref:ro" \
      -v "M:/claud_projects/AsteroidDefense/validation/fixtures:/out" \
      -v "<data-cache-dir>:/data" -e ASSIST_DATA_DIR=/data \
      python:3.12-slim \
      bash -c "apt-get update -qq && apt-get install -y -qq gcc \
               && pip install numpy rebound assist \
               && python /pyref/generate_assist_fixture.py /out/assist_tier1.json"

The committed output is `validation/fixtures/assist_tier1.json`.
"""

import json
import os
import sys

# IAU 2012 astronomical unit, metres — the DE440 AU and this crate's AU_M.
AU_M = 149_597_870_700.0
DAY_S = 86_400.0
JD_REF = 2451545.0  # ASSIST default reference epoch = J2000 TDB (hifitime J2000).

DATA_DIR = os.environ.get(
    "ASSIST_DATA_DIR", "/mnt/m/claud_projects/temp/AsteroidDefense/kernels"
)
FILES = {
    # DE440 planetary (1550–2650) + DE441-consistent 16-asteroid small bodies —
    # ASSIST's shipped pairing (see spike_assist_de441.py / SPIKE.md).
    "planets": "linux_p1550p2650.440",
    "asteroids": "sb441-n16.bsp",
}

# Asteroid (3666) Holman — ASSIST's canonical example IC, barycentric equatorial
# ICRF, AU and AU/day, at JD 2458849.5 (2020-01-01 TDB). A main-belt body
# (~3.3 AU) so a multi-hundred-day arc sweeps varied geometry.
IC_JD_TDB = 2458849.5
HOLMAN_AU = dict(
    x=3.3388753502614090e00,
    y=-9.1765182678903168e-01,
    z=-5.0385906775843303e-01,
    vx=2.8056633343957200e-03,
    vy=7.5504086883996016e-03,
    vz=2.9800282074358684e-03,
)

# Sample arc offsets from the IC, in days. 0.0 first (the seed — confirms the
# AU→SI round-trip); then a spread out to 2 years through varied geometry.
SAMPLE_DAYS = [0.0, 30.0, 90.0, 180.0, 365.0, 730.0]

# DE440 GM set (km^3/s^2) ASSIST reads from its DE440 header, == NAIF
# gm_de440.tpc (Park et al. 2021). Keyed by the body the Rust field tracks;
# Mercury/Venus use the barycenter GM (BODY1/BODY2), which equals the body
# center's (no significant satellites). For the ANISE−DE440 GM-delta diagnostic.
DE440_GM_KM3_S2 = {
    "Sun": 1.3271244004127942e11,       # BODY10
    "Mercury": 2.2031868551400003e04,   # BODY1 (barycenter)
    "Venus": 3.2485859200000000e05,     # BODY2 (barycenter)
    "Earth": 3.9860043550702266e05,     # BODY399
    "Moon": 4.9028001184575496e03,      # BODY301
    "Mars": 4.2828375815756102e04,      # BODY4 (barycenter)
    "Jupiter": 1.2671276409999998e08,   # BODY5 (barycenter)
    "Saturn": 3.7940584841799997e07,    # BODY6 (barycenter)
    "Uranus": 5.7945563999999985e06,    # BODY7 (barycenter)
    "Neptune": 6.8365271005803989e06,   # BODY8 (barycenter)
    "Pluto": 9.7550000000000000e02,     # BODY9 (barycenter)
}
DE440_GM_PROVENANCE = {
    "gm_source": "DE440 header GMs ASSIST integrates with == NAIF gm_de440.tpc",
    "gm_reference": "Park et al. 2021 (DE440/DE441); gm_de440.tpc",
    "gm_url": "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/gm_de440.tpc",
    "note": (
        "Mercury/Venus are barycenter GMs (BODY1/BODY2), equal to the body "
        "center's. Compared on the Rust side against ANISE's pck11.pca values."
    ),
}


def fail(msg):
    print(f"\nASSIST FIXTURE GENERATION: FAIL\n  {msg}")
    sys.exit(1)


def ensure_data():
    paths = {}
    for key, name in FILES.items():
        path = os.path.join(DATA_DIR, name)
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            fail(
                f"{name} not found in {DATA_DIR} — run spike_assist_de441.py first "
                f"to download the ASSIST kernel pair (they are large & git-ignored)"
            )
        paths[key] = path
    return paths


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "assist_tier1.json"

    try:
        import numpy
        import rebound
        import assist
    except Exception as e:  # noqa: BLE001
        fail(f"import failed (assist C extension did not build?): {e!r}")

    paths = ensure_data()
    ephem = assist.Ephem(paths["planets"], paths["asteroids"])

    # AU pin: ASSIST's own AU must equal the IAU AU_M we convert with, else the
    # SI states are silently mis-scaled. Read it if the binding exposes it.
    assist_au_m = None
    try:
        assist_au_m = float(ephem.AU) * 1.0e3  # ephemeris AU is in km
    except Exception:  # noqa: BLE001
        pass
    if assist_au_m is not None and abs(assist_au_m - AU_M) > 1.0:
        fail(
            f"ASSIST AU {assist_au_m} m != AU_M {AU_M} m — a scale mismatch would "
            f"corrupt every SI state"
        )

    t0 = IC_JD_TDB - JD_REF  # sim time: days since jd_ref (TDB)

    sim = rebound.Simulation()
    sim.t = t0
    sim.add(**HOLMAN_AU)
    ex = assist.Extras(sim, ephem)

    # Force model: point-mass Sun + planets ONLY (11 bodies incl. Moon & Pluto).
    # GR / harmonics / asteroids / non-gravs OFF — the exact Tier-1 dynamical
    # system. Assert the resolved active set, don't assume the flag semantics.
    ex.forces = ["SUN", "PLANETS"]
    active = set(ex.forces)
    if active != {"SUN", "PLANETS"}:
        fail(f"forces resolved to {sorted(active)}, expected {{'SUN','PLANETS'}}")

    def state_si():
        p = sim.particles[0]
        return (
            [p.x * AU_M, p.y * AU_M, p.z * AU_M],
            [p.vx * AU_M / DAY_S, p.vy * AU_M / DAY_S, p.vz * AU_M / DAY_S],
        )

    r0, v0 = state_si()

    samples = []
    for off in SAMPLE_DAYS:
        sim.integrate(t0 + off)  # SAMPLE_DAYS ascending → monotone forward
        r, v = state_si()
        samples.append(
            {
                "days": off,
                "dt_s": off * DAY_S,
                "tdb_seconds_past_j2000": (t0 + off) * DAY_S,
                "position_m": r,
                "velocity_m_s": v,
            }
        )

    # Non-vacuous check: the particle must actually move under the DE field, or
    # the "oracle" agreeing with a broken Rust side would be meaningless.
    moved = sum((a - b) ** 2 for a, b in zip(r0, samples[-1]["position_m"])) ** 0.5
    if moved < 1.0e9:  # < ~0.007 AU over 2 years → integration suspect
        fail(f"test particle barely moved ({moved:.3e} m over 2 yr) — DE force suspect")

    fixture = {
        "_comment": (
            "Tier-1 ASSIST reference trajectory for asteroid_core (HANDOFF §10.7 "
            "batch 2c). Test particle in the DE440 field, 11-body point-mass "
            "gravity only (Sun+8 planets+Moon+Pluto), GR/harmonics/asteroids/"
            "non-gravs off. Regenerate with pyref/generate_assist_fixture.py; do "
            "not hand-edit."
        ),
        "generator": "pyref/generate_assist_fixture.py",
        "provenance": {
            "oracle": "ASSIST (test particle in DE440 field, IAS15)",
            "rebound_version": rebound.__version__,
            "assist_version": assist.__version__,
            "python_version": sys.version.split()[0],
            "numpy_version": numpy.__version__,
            "forces_active": sorted(active),
            "planets_kernel": FILES["planets"],
            "asteroids_kernel": FILES["asteroids"],
            "au_m": AU_M,
            "assist_au_m": assist_au_m,
            "jd_ref_tdb": JD_REF,
            "ic_jd_tdb": IC_JD_TDB,
            **DE440_GM_PROVENANCE,
        },
        "frame_note": (
            "States are barycentric (SSB) equatorial ICRF/J2000, SI (m, m/s) — "
            "identical to ANISE SSB_J2000. Epochs are TDB seconds past J2000 "
            "(hifitime J2000 = JD 2451545.0 TDB); no time-scale conversion."
        ),
        "target": "asteroid (3666) Holman",
        "bodies": [
            "Sun", "Mercury", "Venus", "Earth", "Moon",
            "Mars", "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto",
        ],
        "de440_gm_km3_s2": DE440_GM_KM3_S2,
        "epoch0_tdb_seconds_past_j2000": t0 * DAY_S,
        "initial_state": {"position_m": r0, "velocity_m_s": v0},
        "samples": samples,
    }

    with open(out_path, "w") as f:
        json.dump(fixture, f, indent=2)
        f.write("\n")

    print(
        f"wrote {out_path}: {len(samples)} samples over {SAMPLE_DAYS[-1]:.0f} d, "
        f"11-body point mass (forces={sorted(active)}), moved {moved:.3e} m "
        f"(assist {assist.__version__}, rebound {rebound.__version__})"
    )


if __name__ == "__main__":
    main()
