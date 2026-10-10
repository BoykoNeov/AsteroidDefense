"""Worst-case plane cost for a stack parked at 600 km, 28.5 deg, before the rock was
found, leaving through each flown window - the offline check made before
`core/src/station_keeping.rs` was built (HANDOFF, *Standing defence, part 3*).

Input: the `OW,...` rows the probe prints,
    ASTEROID_REQUIRE_KERNELS=1 cargo test -p asteroid_gdext --release --lib         probe_orbit_windows -- --ignored --nocapture > orbit_windows.txt
    python tools/standing_plane_cost.py orbit_windows.txt

The apogee turn here uses the bare angle (inclination + |declination|); the Rust
version turns about the loop's apsides, which needs more (sin phi = sin psi /
sin theta_inf), so it reads ~750 m/s where this reads ~650 for the earliest windows.
"""
import math, sys

MU = 3.986004356e14
RE = 6378137.0
J2 = 1.08263e-3
INC = math.radians(28.5)
H0 = 600e3
H_LO, H_HI = 400e3, 1000e3
APO_CAP = 1.0e8  # 100 000 km, the shipping phasing-loop cap
G0 = 9.80665


def rate(h):  # nodal regression magnitude, rad/s
    a = RE + h
    n = math.sqrt(MU / a**3)
    return 1.5 * n * J2 * (RE / a) ** 2 * math.cos(INC)


def hohmann(h1, h2):
    r1, r2 = RE + h1, RE + h2
    at = (r1 + r2) / 2
    v1, v2 = math.sqrt(MU / r1), math.sqrt(MU / r2)
    vp, va = math.sqrt(MU * (2 / r1 - 1 / at)), math.sqrt(MU * (2 / r2 - 1 / at))
    return abs(vp - v1) + abs(v2 - va)


def escape_dv(h, c3):  # impulsive, m/s; c3 km2/s2
    r = RE + h
    return math.sqrt(2 * MU / r + c3 * 1e6) - math.sqrt(MU / r)


def worst_node_shift(dec):
    """Largest node shift an unknown starting node can need, rad, drifting either way."""
    s = math.sin(abs(dec)) / math.sin(INC)
    if s > 1:
        return None  # the plane can never contain it
    th = math.asin(s)
    beta = math.atan2(math.cos(INC) * math.sin(th), math.cos(th))
    return math.pi / 2 + abs(beta)


def dir_cost(shift, T, c3, leave_from_hold, up):
    """Cheapest dv to add `shift` rad of node by holding above (up) or below H0."""
    hs = [H0 + k * 1e3 for k in range(1, int((H_HI - H0) / 1e3) + 1)] if up else \
         [H0 - k * 1e3 for k in range(1, int((H0 - H_LO) / 1e3) + 1)]
    if shift <= 0:
        return (0.0, H0)
    for h in hs:  # cost grows with |h - H0|: the first that reaches is the cheapest
        if abs(rate(h) - rate(H0)) * T >= shift:
            dv = hohmann(H0, h) + (escape_dv(h, c3) - escape_dv(H0, c3) if leave_from_hold else hohmann(h, H0))
            return (dv, h)
    return None


def hold_cost(dec, T, c3, leave_from_hold):
    """Worst case over the unknown starting node: the larger gap between the two
    aligned nodes is L = pi + 2|beta|; a node x into it can shift x one way or L - x
    the other, and the unlucky node is where the cheaper of the two is dearest."""
    need = worst_node_shift(dec)
    if need is None:
        return None
    L = 2 * need
    inf = (float("inf"), None)
    worst = None
    n = 400
    for k in range(n + 1):
        x = L * k / n
        a = dir_cost(x, T, c3, leave_from_hold, True) or inf
        b = dir_cost(L - x, T, c3, leave_from_hold, False) or inf
        m = a if a[0] <= b[0] else b
        if worst is None or m[0] > worst[0]:
            worst = m
    return None if worst[0] == float("inf") else worst


def apogee_turn_cost(dec):
    """Turn the plane at the apogee of the last phasing loop (perigee 600 km, apogee
    100 000 km): worst-case tilt between the asymptote and the plane is inc + |dec|."""
    psi = min(INC + abs(math.radians(dec)), math.pi / 2)
    rp, ra = RE + H0, APO_CAP
    a = (rp + ra) / 2
    va = math.sqrt(MU * (2 / ra - 1 / a))
    return 2 * va * math.sin(psi / 2)


rows = [l.strip().split(",") for l in open(sys.argv[1]) if l.startswith("OW,")]
print("warn  days  yr_out  dec   C3   |shift|km  hold-return  hold-leave  apogee-turn  best  %mass")
for r in rows:
    w, days, yout, dec, c3, sh = float(r[1]), float(r[2]), float(r[3]), float(r[4]), float(r[5]), float(r[6])
    T = days * 86400.0
    decr = math.radians(dec)
    hr = hold_cost(decr, T, c3, False)
    hl = hold_cost(decr, T, c3, True)
    ap = apogee_turn_cost(dec)
    if abs(dec) > 28.5:
        print(f"{w:4.0f} {days:6.0f} {yout:6.2f} {dec:6.1f} {c3:5.1f} {sh:9.0f}   (steeper than the orbit: unreachable)")
        continue
    cands = [x[0] for x in (hr, hl) if x] + [ap]
    best = min(cands)
    frac = 1 - math.exp(-best / (310 * G0))
    f = lambda x: f"{x[0]:6.0f}@{x[1]/1e3:4.0f}" if x else "   infeas  "
    print(f"{w:4.0f} {days:6.0f} {yout:6.2f} {dec:6.1f} {c3:5.1f} {sh:9.0f}  {f(hr)}  {f(hl)}  {ap:8.0f}  {best:6.0f} {100*frac:5.1f}")
