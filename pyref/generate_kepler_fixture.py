#!/usr/bin/env python3
"""Generate the §10.6 two-body reference fixture (HANDOFF §6, §10.6).

Propagate a known heliocentric orbit with **hapsira** (analytic two-body
propagator) and write the reference states as JSON, for the Rust `validation`
crate to check `asteroid_core`'s analytic `KeplerPropagator` against an
independent oracle.

Why this fixture is trustworthy — the three things pinned identically on both
sides so the comparison tests *propagation*, not a units/frame/constant slip:

  1. **μ.** The Sun's gravitational parameter is baked from the value ANISE
     resolves from `pck11.pca` (`core/examples/probe_sun_gm.rs`), NOT hapsira's
     default `Sun.k` (a different, IAU-nominal value). A custom `Body` forces
     hapsira to use it; the generator then self-asserts the resulting period
     equals 2π√(a³/μ) for that μ, so a silent override failure fails loudly here
     instead of shipping a fixture built on the wrong constant. The Rust side
     pulls the same μ through ANISE and the gated test `sun_gm_matches_fixture`
     asserts it equals `mu_m3_s2` below.
  2. **Frame convention.** Classical elements → Cartesian is the standard 3-1-3
     perifocal→inertial rotation on both sides, so the comparison is independent
     of which *physical* frame the orbit is called (the `dt_s == 0` sample guards
     this: its position depends only on a, e, ν and the rotation convention).
  3. **Time.** States are sampled by *elapsed seconds* (`TimeDelta`), never an
     absolute epoch, so no TT/TDB/UTC scale conversion enters — two-body motion
     only cares about Δt in SI seconds.

Run under Docker (hapsira needs astropy<7; native Windows Python here is 3.14,
which numba does not yet support):

    docker run --rm \
      -v "M:/claud_projects/AsteroidDefense/pyref:/pyref:ro" \
      -v "M:/claud_projects/AsteroidDefense/validation/fixtures:/out" \
      python:3.12-slim \
      bash -c "pip install -r /pyref/requirements-hapsira.txt \
               && python /pyref/generate_kepler_fixture.py /out/kepler_two_body.json"

The committed output is `validation/fixtures/kepler_two_body.json`.
"""

import json
import math
import sys

from astropy import units as u
from astropy.time import TimeDelta
from hapsira.bodies import Body
from hapsira.twobody import Orbit

import astropy
import hapsira
import numpy

# Sun GM as ANISE resolves it from pck11.pca (frame Sun J2000), in SI m^3/s^2.
# Provenance: `cargo run -p asteroid_core --example probe_sun_gm -- pck11.pca`.
# This is the DE440-consistent value, distinct from hapsira's default Sun.k.
MU_SUN_M3_S2 = 1.32712440041939370e20

MU_PROVENANCE = {
    "mu_source": "ANISE Sun GM, frame SUN_J2000, resolved from pck11.pca",
    "mu_pca_file": "pck11.pca",
    "mu_pca_url": "http://public-data.nyxspace.com/anise/v0.10/pck11.pca",
    "anise_version": "0.10.3",
    "probe": "core/examples/probe_sun_gm.rs",
}

# Sample offsets from the reference epoch, as fractions of the orbital period.
# 0.0 first (guards frame convention + that hapsira used our μ); a full period
# (return-to-start); a large non-integer multiple (12.7 — the sensitive μ-pin
# discriminator: phase error grows as n·Δt, and it exercises the many-period
# wrap in solve_kepler); and a negative offset (backward propagation).
PERIOD_FRACTIONS = [0.0, 0.125, 0.25, 0.5, 0.75, 1.0, 12.7, -0.25]

# Two generic orbits — inclined and non-equatorial so neither hits the
# circular/equatorial gauge singularities (those are task-5's proptest job, not
# this fixture's). The second stresses the Kepler solve at higher eccentricity.
ORBITS = [
    {
        "label": "moderate-e inclined (a=2.5 AU, e=0.4)",
        "a": 2.5 * u.au,
        "ecc": 0.4 * u.one,
        "inc": 15.0 * u.deg,
        "raan": 60.0 * u.deg,
        "argp": 40.0 * u.deg,
        "nu": 10.0 * u.deg,
    },
    {
        "label": "high-e inclined (a=1.8 AU, e=0.7)",
        "a": 1.8 * u.au,
        "ecc": 0.7 * u.one,
        "inc": 33.0 * u.deg,
        "raan": 200.0 * u.deg,
        "argp": 130.0 * u.deg,
        "nu": 250.0 * u.deg,
    },
]


def fail(msg):
    print(f"\nFIXTURE GENERATION: FAIL\n  {msg}")
    sys.exit(1)


def build_orbit(spec, attractor):
    return Orbit.from_classical(
        attractor,
        spec["a"],
        spec["ecc"],
        spec["inc"],
        spec["raan"],
        spec["argp"],
        spec["nu"],
    )


def orbit_record(spec, attractor):
    orb = build_orbit(spec, attractor)

    a_m = spec["a"].to(u.m).value
    period_s = orb.period.to(u.s).value

    # Self-check: hapsira must have used OUR μ. If the custom-body override
    # silently failed, the period would follow hapsira's default Sun.k instead.
    period_analytic = 2.0 * math.pi * math.sqrt(a_m**3 / MU_SUN_M3_S2)
    rel = abs(period_s - period_analytic) / period_analytic
    if rel > 1e-12:
        fail(
            f"period mismatch for {spec['label']!r}: hapsira {period_s} vs "
            f"analytic {period_analytic} (rel {rel:.2e}) — μ override did not take"
        )

    samples = []
    for frac in PERIOD_FRACTIONS:
        dt_s = frac * period_s
        state = orb.propagate(TimeDelta(dt_s * u.s))
        r = state.r.to(u.m).value
        v = state.v.to(u.m / u.s).value
        samples.append(
            {
                "period_fraction": frac,
                "dt_s": dt_s,
                "position_m": [float(r[0]), float(r[1]), float(r[2])],
                "velocity_m_s": [float(v[0]), float(v[1]), float(v[2])],
            }
        )

    return {
        "label": spec["label"],
        "elements": {
            "a_m": a_m,
            "ecc": float(spec["ecc"].value),
            "inc_rad": spec["inc"].to(u.rad).value,
            "raan_rad": spec["raan"].to(u.rad).value,
            "argp_rad": spec["argp"].to(u.rad).value,
            "nu_rad": spec["nu"].to(u.rad).value,
        },
        "period_s": period_s,
        "samples": samples,
    }


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "kepler_two_body.json"

    attractor = Body(parent=None, k=MU_SUN_M3_S2 * u.m**3 / u.s**2, name="RefSun")
    # Guard the constant itself before we lean on it.
    k_used = attractor.k.to(u.m**3 / u.s**2).value
    if k_used != MU_SUN_M3_S2:
        fail(f"attractor.k {k_used} != intended μ {MU_SUN_M3_S2}")

    fixture = {
        "_comment": (
            "Two-body reference states from hapsira for asteroid_core's analytic "
            "KeplerPropagator (HANDOFF §10.6). Regenerate with "
            "pyref/generate_kepler_fixture.py; do not hand-edit."
        ),
        "generator": "pyref/generate_kepler_fixture.py",
        "provenance": {
            **MU_PROVENANCE,
            "propagator": "hapsira Orbit.propagate (analytic two-body)",
            "python_version": sys.version.split()[0],
            "hapsira_version": hapsira.__version__,
            "astropy_version": astropy.__version__,
            "numpy_version": numpy.__version__,
        },
        "frame_note": (
            "States are heliocentric, SI (m, m/s). Classical elements → Cartesian "
            "uses the standard 3-1-3 perifocal→inertial rotation on both sides, so "
            "the comparison is frame-label independent; angles in radians, Δt in "
            "elapsed seconds (no absolute epoch → no time-scale conversion)."
        ),
        "mu_m3_s2": MU_SUN_M3_S2,
        "orbits": [orbit_record(spec, attractor) for spec in ORBITS],
    }

    with open(out_path, "w") as f:
        json.dump(fixture, f, indent=2)
        f.write("\n")

    n = sum(len(o["samples"]) for o in fixture["orbits"])
    print(
        f"wrote {out_path}: {len(fixture['orbits'])} orbits, {n} samples, "
        f"μ={MU_SUN_M3_S2:.6e} m^3/s^2 (hapsira {hapsira.__version__}, "
        f"astropy {astropy.__version__})"
    )


if __name__ == "__main__":
    main()
