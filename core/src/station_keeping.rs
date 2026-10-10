//! `station_keeping` — what waiting in a low orbit costs (HANDOFF §8, Phase 3's
//! standing defence, part 3: interceptors waiting in orbit).
//!
//! A parked stack ([`crate::orbital_assembly`]) waits in a circular orbit for its
//! departure date. Until this module the wait was free. It is not, in two ways:
//!
//! - **Drag.** The thin air at a few hundred kilometres slows the stack, and the
//!   height it loses has to be bought back with propellant. Small over weeks, a few
//!   per cent of the stack over a decade at 400 km, nearly nothing from 600 km up.
//! - **The plane** (stacks parked before the rock was found, only). A parked launch
//!   that goes up *after* the rock is found knows its departure date, so its launch
//!   time is chosen to have the plane turned into place on that day - free (the
//!   orbital-assembly module doc). A stack that went up years before cannot have
//!   known, so on the day the rock is found its plane is wherever it happens to be,
//!   and it has to be swung into line before the departure - at a cost.
//!
//! Both are paid out of the stack, at the station-keeping thrusters' efficiency,
//! before the escape burn.
//!
//! # The air: NASA's design table, the cycle's median of the day-side maximum
//! *Natural Orbital Environment Guidelines for Use in Aerospace Vehicle
//! Development*, NASA TM-4527 (B. J. Anderson, ed., R. E. Smith, comp.; MSFC, June
//! 1994; NTRS 19940031668),
//! Table C-1: "Median value of global maximum densities for altitude and F10.7 bin"
//! (pages C-1 to C-6), from the MSFC/MET model with its short-period statistics.
//! [`DENSITY_TABLE`] carries three of its columns from 350 to 1 000 km - Bin 1
//! (F10.7 66-102, a quiet sun), Bin 5 (210-246, an active one) and **All (66-246),
//! the shipping column**: a stack waiting years sees the whole solar cycle. Every
//! row was read off the page images, not the text layer (whose 550 km "All" median
//! reads 7.105E-12 - another row's number; the page says 4.442E-13).
//!
//! **It overstates the drag, on purpose.** The table is the *global* maximum - the
//! density at the hottest point of the day-side bulge - while an orbit spends half
//! its time on the night side, where it is several times thinner. So the cost here
//! is an upper bound in the direction that counts against waiting in orbit.
//!
//! # The stack: the fairing it rode up in, at any attitude
//! Nothing published gives a 26.5 t storable stack's shape. What bounds it is the
//! fairing it was launched in: Falcon's extended fairing, "the same diameter as the
//! standard faring (5.2 m, 17.2 ft) and an overall height of 18.7 m" (*Falcon User's
//! Guide*, SpaceX, 2025-05-09, §4.1.2). Anything that fits presents at most the
//! cylinder's area, and a body tumbling at random presents on average a quarter of
//! its surface (Cauchy's theorem for a convex body): [`FAIRING_TUMBLING_AREA_M2`],
//! about 87 m². That is the shipping area - a second upper bound, since a real stack
//! is mostly ~18 m³ of propellant and would hold a low-drag attitude. The
//! [`FAIRING_NOSE_ON_AREA_M2`] (the fairing's circle, ~21 m²) is the bracket.
//! The drag coefficient is [`DRAG_COEFFICIENT`] = 2.2: "estimated at 2.2, using the
//! default from NASA's Debris Analysis Software (DAS) version 3.1" - W. S.
//! Shambaugh, *Doing Battle with the Sun*, 4S Symposium 2024, arXiv:2406.08342, §3.
//!
//! # The thrusters: Orion's auxiliary engines
//! The departure engine (the OMS-E) is rated for ten starts and the escape uses
//! eight, so it cannot make up drag for years. Orion's service module carries eight
//! auxiliary engines for exactly this kind of work, on the same propellant: "Thrust
//! 489 N … Specific Impulse 310 seconds … Max Firing Time 2000 seconds … Number of
//! Burns 40" (Belair et al., SP2024_382, Table 3 - the paper the OMS-E is cited
//! from). [`ORION_AUX`].

use crate::impactor_mass::propellant_fraction;

/// Air density, kg/m³, by altitude: `(km, quiet sun, whole cycle, active sun)` -
/// NASA TM-4527 Table C-1, the medians of the global maximum density in F10.7 Bin 1
/// (66-102), All (66-246) and Bin 5 (210-246). Read off the page images.
pub const DENSITY_TABLE: [(f64, f64, f64, f64); 16] = [
    (350.0, 5.786e-12, 1.095e-11, 2.602e-11),
    (375.0, 3.479e-12, 7.008e-12, 1.826e-11),
    (400.0, 2.128e-12, 4.567e-12, 1.302e-11),
    (425.0, 1.321e-12, 3.019e-12, 9.405e-12),
    (450.0, 8.298e-13, 2.021e-12, 6.873e-12),
    (475.0, 5.271e-13, 1.366e-12, 5.073e-12),
    (500.0, 3.384e-13, 9.322e-13, 3.777e-12),
    (550.0, 1.445e-13, 4.442e-13, 2.134e-12),
    (600.0, 6.534e-14, 2.181e-13, 1.243e-12),
    (650.0, 3.179e-14, 1.110e-13, 7.370e-13),
    (700.0, 1.696e-14, 5.872e-14, 4.445e-13),
    (750.0, 1.011e-14, 3.269e-14, 2.724e-13),
    (800.0, 6.631e-15, 1.931e-14, 1.700e-13),
    (850.0, 4.735e-15, 1.223e-14, 1.079e-13),
    (900.0, 3.576e-15, 8.284e-15, 6.967e-14),
    (950.0, 2.804e-15, 5.977e-15, 4.609e-14),
];

/// The table's top row, kept apart so [`DENSITY_TABLE`] stays a plain array.
const DENSITY_1000_KM: (f64, f64, f64, f64) = (1000.0, 2.251e-15, 4.535e-15, 3.122e-14);

/// Which of the table's columns: how active the sun is over the wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolarActivity {
    /// F10.7 66-102 (Bin 1).
    Quiet,
    /// F10.7 66-246, the whole cycle (All) - the shipping column.
    WholeCycle,
    /// F10.7 210-246 (Bin 5).
    Active,
}

/// The table's density at `altitude_m`, kg/m³ - interpolated log-linearly between
/// rows (density falls close to exponentially with height). `None` outside the
/// table's 350-1 000 km: nothing here is extrapolated.
pub fn density_kg_m3(altitude_m: f64, activity: SolarActivity) -> Option<f64> {
    let km = altitude_m / 1_000.0;
    let pick = |r: &(f64, f64, f64, f64)| match activity {
        SolarActivity::Quiet => r.1,
        SolarActivity::WholeCycle => r.2,
        SolarActivity::Active => r.3,
    };
    let rows: Vec<(f64, f64, f64, f64)> = DENSITY_TABLE
        .iter()
        .copied()
        .chain(std::iter::once(DENSITY_1000_KM))
        .collect();
    if !(rows[0].0..=rows[rows.len() - 1].0).contains(&km) {
        return None;
    }
    let j = rows.partition_point(|r| r.0 <= km).clamp(1, rows.len() - 1);
    let (lo, hi) = (&rows[j - 1], &rows[j]);
    let f = (km - lo.0) / (hi.0 - lo.0);
    // A row, exactly - not through ln and back.
    if f == 0.0 {
        return Some(pick(lo));
    }
    if f == 1.0 {
        return Some(pick(hi));
    }
    Some((pick(lo).ln() + f * (pick(hi).ln() - pick(lo).ln())).exp())
}

/// The drag coefficient: 2.2, the NASA Debris Assessment Software default.
pub const DRAG_COEFFICIENT: f64 = 2.2;

/// Falcon's extended fairing, m: "the same diameter as the standard faring (5.2 m
/// …) and an overall height of 18.7 m" (*Falcon User's Guide*, 2025-05-09).
pub const FAIRING_DIAMETER_M: f64 = 5.2;
/// See [`FAIRING_DIAMETER_M`].
pub const FAIRING_HEIGHT_M: f64 = 18.7;

/// The mean area a body filling the fairing presents tumbling at random, m²: a
/// quarter of the cylinder's surface (Cauchy). The shipping area - an upper bound.
pub const FAIRING_TUMBLING_AREA_M2: f64 = std::f64::consts::PI
    * (FAIRING_DIAMETER_M / 2.0)
    * (FAIRING_DIAMETER_M / 2.0 + FAIRING_HEIGHT_M)
    / 2.0;

/// The fairing's circle, m²: the same body flown nose-first. The bracket.
pub const FAIRING_NOSE_ON_AREA_M2: f64 =
    std::f64::consts::PI * (FAIRING_DIAMETER_M / 2.0) * (FAIRING_DIAMETER_M / 2.0);

/// Station-keeping thrusters, as far as the cost of waiting needs them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thrusters {
    /// Display name.
    pub name: &'static str,
    /// Vacuum specific impulse, s.
    pub isp_s: f64,
    /// Thrust of one engine, N.
    pub thrust_n: f64,
    /// Engines fitted.
    pub count: u32,
    /// Rated number of burns.
    pub burns: u32,
}

/// Orion's auxiliary engines (Aerojet Rocketdyne R-4D-11, MMH / MON-3) - Belair et
/// al., SP2024_382, Table 3: 489 N, 310 s, 40 burns; eight fitted (§4.1).
pub const ORION_AUX: Thrusters = Thrusters {
    name: "R-4D-11 auxiliary",
    isp_s: 310.0,
    thrust_n: 489.0,
    count: 8,
    burns: 40,
};

/// How the drag on a waiting stack is priced.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DragModel {
    /// The table column.
    pub activity: SolarActivity,
    /// The area the stack presents, m².
    pub area_m2: f64,
    /// The drag coefficient.
    pub cd: f64,
}

/// The shipping drag: the whole cycle, the fairing tumbling - both upper bounds.
pub const SHIPPING_DRAG: DragModel = DragModel {
    activity: SolarActivity::WholeCycle,
    area_m2: FAIRING_TUMBLING_AREA_M2,
    cd: DRAG_COEFFICIENT,
};

/// The bracket: the same air, the stack flown nose-first.
pub const NOSE_ON_DRAG: DragModel = DragModel {
    area_m2: FAIRING_NOSE_ON_AREA_M2,
    ..SHIPPING_DRAG
};

impl DragModel {
    /// The Δv drag takes per second at `altitude_m` from a circular orbit, m/s², on
    /// a stack of `mass_kg`: `½ ρ v² Cd A / m`. Above the table's 1 000 km the
    /// density is taken at 1 000 km - an upper bound, since the air only thins
    /// higher up (it is ~0.02 m/s a year there already). `None` below its 350 km.
    pub fn dv_rate_m_s2(&self, altitude_m: f64, mass_kg: f64) -> Option<f64> {
        let rho = density_kg_m3(altitude_m.min(DENSITY_1000_KM.0 * 1e3), self.activity)?;
        let r = crate::geometry::EARTH_EQUATORIAL_RADIUS_M + altitude_m;
        let v2 = crate::orbital_assembly::EARTH_GM_M3_S2 / r;
        Some(0.5 * rho * v2 * self.cd * self.area_m2 / mass_kg)
    }

    /// The fraction of a `mass_kg` stack left after `wait_s` of making up drag at
    /// `altitude_m` with `thrusters`. Priced at the starting mass throughout - the
    /// stack gets lighter as it burns and the drag acceleration rises with it, but
    /// over the years here the stack loses a few per cent, so the error is a few
    /// per cent *of* a few per cent. `None` outside the density table.
    pub fn mass_left(
        &self,
        altitude_m: f64,
        mass_kg: f64,
        wait_s: f64,
        thrusters: &Thrusters,
    ) -> Option<f64> {
        let a = self.dv_rate_m_s2(altitude_m, mass_kg)?;
        Some(1.0 - propellant_fraction(a * wait_s.max(0.0), thrusters.isp_s))
    }
}

/// What waiting in a parking orbit costs: the drag, made up with these thrusters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waiting {
    /// How the drag is priced.
    pub drag: DragModel,
    /// What makes it up.
    pub thrusters: Thrusters,
}

/// The shipping wait: [`SHIPPING_DRAG`] made up by [`ORION_AUX`].
pub const SHIPPING_WAITING: Waiting = Waiting {
    drag: SHIPPING_DRAG,
    thrusters: ORION_AUX,
};

impl Waiting {
    /// What is left of a `mass_kg` stack after `t` seconds at `altitude_m` is
    /// `e^(−k t)`; this is `k`, per second. `None` outside the density table.
    pub fn decay_per_s(&self, altitude_m: f64, mass_kg: f64) -> Option<f64> {
        let a = self.drag.dv_rate_m_s2(altitude_m, mass_kg)?;
        Some(a / (self.thrusters.isp_s * crate::impactor_mass::G0_M_S2))
    }
}

// --- The plane of a stack parked before the rock was found -------------------

/// How fast a circular orbit's plane turns about Earth's axis, rad/s (magnitude;
/// it regresses, westward for a prograde orbit): `3/2 · n · J2 · (R/a)² · cos i`,
/// with DE440's `J2` and the radius it is defined against.
pub fn node_rate_rad_s(altitude_m: f64, inclination_rad: f64) -> f64 {
    use crate::forces::oblateness::{EARTH_EQUATORIAL_RADIUS_M_DE440, EARTH_J2_DE440};
    let a = crate::geometry::EARTH_EQUATORIAL_RADIUS_M + altitude_m;
    let n = (crate::orbital_assembly::EARTH_GM_M3_S2 / a.powi(3)).sqrt();
    1.5 * n * EARTH_J2_DE440 * (EARTH_EQUATORIAL_RADIUS_M_DE440 / a).powi(2) * inclination_rad.cos()
}

/// The largest gap, rad, between the two node angles at which an orbit tilted
/// `inclination_rad` contains a departure of declination `declination_rad` - so
/// an unknown starting node is at most this far from one of them, counted the
/// long way round the gap (`π + 2|β|`; `2π` when the departure grazes the orbit's
/// top, where the two angles meet). `None` when the departure is steeper than the
/// orbit and no node contains it.
///
/// An orbit with node `Ω` contains the directions `û(θ)` whose declination is
/// `asin(sin i · sin θ)`; for a declination `δ` that is `sin θ = sin δ / sin i`,
/// at `θ₁` and `π - θ₁`, whose right ascensions sit `β = atan2(cos i · sin θ₁,
/// cos θ₁)` and `π - β` past the node. The two nodes are `π - 2β` apart one way and
/// `π + 2β` the other.
pub fn node_gap_rad(declination_rad: f64, inclination_rad: f64) -> Option<f64> {
    let s = declination_rad.abs().sin() / inclination_rad.sin();
    if !(s <= 1.0) {
        return None;
    }
    let th = s.asin();
    let beta = (inclination_rad.cos() * th.sin()).atan2(th.cos());
    Some(std::f64::consts::PI + 2.0 * beta.abs())
}

/// Heights a stack may hold at while it swings its plane, m: no lower than
/// [`crate::orbital_assembly::PARKING_ALTITUDE_M`] (the lowest the project lets a
/// stack wait at) and no higher than 1 000 km (the top of the density table, and
/// the edge of the inner radiation belt the orbital-assembly bracket stopped at).
pub const HOLD_ALTITUDE_RANGE_M: (f64, f64) = (crate::orbital_assembly::PARKING_ALTITUDE_M, 1.0e6);

/// A two-burn transfer between circular orbits of altitudes `h1_m` and `h2_m`, m/s.
pub fn hohmann_dv_m_s(h1_m: f64, h2_m: f64) -> f64 {
    let mu = crate::orbital_assembly::EARTH_GM_M3_S2;
    let r1 = crate::geometry::EARTH_EQUATORIAL_RADIUS_M + h1_m;
    let r2 = crate::geometry::EARTH_EQUATORIAL_RADIUS_M + h2_m;
    let at = 0.5 * (r1 + r2);
    let (v1, v2) = ((mu / r1).sqrt(), (mu / r2).sqrt());
    let (vp, va) = (
        (mu * (2.0 / r1 - 1.0 / at)).sqrt(),
        (mu * (2.0 / r2 - 1.0 / at)).sqrt(),
    );
    (vp - v1).abs() + (v2 - va).abs()
}

/// The cheapest way to turn the node `shift_rad` further than it turns by itself
/// over `hold_s`, holding above (`raise`) or below the home altitude `home_m` and
/// coming back: `(Δv m/s, hold altitude m)`, or `None` if no height inside
/// [`HOLD_ALTITUDE_RANGE_M`] turns it that far in the time.
///
/// Holding the whole time at the nearest height that does it is the cheapest way:
/// the node gained is the rate change times the time held, and both the rate change
/// and the transfer cost grow with the height change. Drag at the hold height is
/// charged too (a stack lowered toward 400 km pays it).
pub fn node_hold_dv_m_s(
    home_m: f64,
    inclination_rad: f64,
    shift_rad: f64,
    hold_s: f64,
    raise: bool,
    drag: &DragModel,
    mass_kg: f64,
) -> Option<(f64, f64)> {
    if shift_rad <= 0.0 {
        return Some((0.0, home_m));
    }
    if !(hold_s > 0.0) {
        return None;
    }
    let r0 = node_rate_rad_s(home_m, inclination_rad);
    // Higher turns slower, lower faster.
    let want = if raise {
        r0 - shift_rad / hold_s
    } else {
        r0 + shift_rad / hold_s
    };
    if !(want > 0.0) {
        return None;
    }
    // rate ∝ a^-3.5: invert for the radius.
    let a0 = crate::geometry::EARTH_EQUATORIAL_RADIUS_M + home_m;
    let a = a0 * (r0 / want).powf(1.0 / 3.5);
    let h = a - crate::geometry::EARTH_EQUATORIAL_RADIUS_M;
    if !(HOLD_ALTITUDE_RANGE_M.0..=HOLD_ALTITUDE_RANGE_M.1).contains(&h) {
        return None;
    }
    let drag_dv = drag.dv_rate_m_s2(h, mass_kg)? * hold_s;
    Some((2.0 * hohmann_dv_m_s(home_m, h) + drag_dv, h))
}

/// The worst case over an unknown starting node of swinging the plane into line by
/// holding at another height ([`node_hold_dv_m_s`]): `(Δv m/s, hold altitude m)`,
/// or `None` where some starting node cannot be brought round in `hold_s`.
///
/// A node `x` into the larger gap `L` ([`node_gap_rad`]) can be turned `x` one way
/// (lowered, faster) or `L - x` the other (raised, slower). The two directions do
/// not cost the same per radian, so the unlucky node is not mid-gap but where the
/// cheaper of the two is dearest - found by bisection on the crossing.
pub fn worst_node_hold_dv_m_s(
    home_m: f64,
    inclination_rad: f64,
    declination_rad: f64,
    hold_s: f64,
    drag: &DragModel,
    mass_kg: f64,
) -> Option<(f64, f64)> {
    let l = node_gap_rad(declination_rad, inclination_rad)?;
    let cost = |x: f64, raise: bool| {
        node_hold_dv_m_s(home_m, inclination_rad, x, hold_s, raise, drag, mass_kg)
            .unwrap_or((f64::INFINITY, f64::NAN))
    };
    // At x the options are lowering by x and raising by L - x; lowering's cost grows
    // with x, raising's falls. The worst node is where they cross.
    let pick = |x: f64| {
        let (lo, hi) = (cost(x, false), cost(l - x, true));
        if lo.0 <= hi.0 {
            lo
        } else {
            hi
        }
    };
    let (mut a, mut b) = (0.0_f64, l);
    for _ in 0..100 {
        let m = 0.5 * (a + b);
        if cost(m, false).0 < cost(l - m, true).0 {
            a = m;
        } else {
            b = m;
        }
    }
    let w = pick(0.5 * (a + b));
    w.0.is_finite().then_some(w)
}

/// The other way to bring the plane round: turn it at the far end of a high
/// elliptical loop from `home_m` out to `apogee_r_m` (from Earth's centre), where
/// the stack moves slowly, then come back down to leave. The worst case over the
/// starting node: the departure can sit `inclination + |declination|` off the
/// plane, and the turn is made about the loop's line of apsides, which keeps the
/// escape hyperbola's perigee in place - so the plane has to turn by `φ` with
/// `sin φ = sin ψ / sin θ∞`, `θ∞` the angle from perigee to the asymptote
/// (`cos θ∞ = −1/e`) - an estimate, not a flown manoeuvre. Δv m/s, two-body.
pub fn worst_apogee_turn_dv_m_s(
    home_m: f64,
    apogee_r_m: f64,
    inclination_rad: f64,
    declination_rad: f64,
    c3_km2_s2: f64,
) -> f64 {
    let mu = crate::orbital_assembly::EARTH_GM_M3_S2;
    let rp = crate::geometry::EARTH_EQUATORIAL_RADIUS_M + home_m;
    let a = 0.5 * (rp + apogee_r_m);
    let va = (mu * (2.0 / apogee_r_m - 1.0 / a)).sqrt();
    let psi = (inclination_rad + declination_rad.abs()).min(std::f64::consts::FRAC_PI_2);
    let e = 1.0 + rp * c3_km2_s2.max(0.0) * 1.0e6 / mu;
    let theta_inf = (-1.0 / e).acos();
    let phi = (psi.sin() / theta_inf.sin()).min(1.0).asin();
    2.0 * va * (0.5 * phi).sin()
}

// --- A stack parked years before the rock was found -------------------------

/// The height a standing stack waits at for years, m: 600 km, the user's choice
/// (2026-10-10). Drag there is ~1.5 m/s a year on the shipping (upper-bound) area,
/// against ~31 at 400 km; it sits below the radiation belts, and Falcon Heavy lifts
/// far more than the 26.5 t stack to low orbit (the lift to 600 km is not itself
/// sourced). The post-warning parked launches stay at 400 km.
pub const STANDING_ALTITUDE_M: f64 = 600_000.0;

/// The grid a stack that leaves *from* its holding height holds at, m: each such
/// height flies its own escape-loss table (about a tenth of a second, then cached),
/// so the heights are 50 km steps from 400 to 1 000 km, rounded away from home -
/// further from home turns the plane at least as fast, so the rounding never makes
/// a shift look possible that is not, and costs a little more than it has to.
pub const HOLD_GRID_M: f64 = 50_000.0;

/// A stack launched to a parking orbit before anyone knew the rock was there, and
/// waiting ([`crate::readiness::IN_ORBIT`]). What it can deliver through a window is
/// the parked stack's ([`crate::orbital_assembly::ParkedDelivery`]) less three
/// costs, all made up by the station-keeping thrusters before the escape:
///
/// 1. **Drag** for the whole time in orbit - the years before the rock was found,
///    the decision, and the wait for the departure.
/// 2. **The plane.** Its node is wherever it has drifted to, so the departure is
///    priced for the **worst** starting node (the user's choice, 2026-10-10): the
///    cheapest of three ways to bring it round -
///    - hold at another height until the node has turned into place, then come back
///      ([`worst_node_hold_dv_m_s`]);
///    - hold there and leave from there - more node per metre of height when going
///      up, and a smaller escape burn ([`HOLD_GRID_M`]);
///    - turn the plane at the far end of a high loop ([`worst_apogee_turn_dv_m_s`]),
///      dear but always there, which is what is left in the first weeks.
///
///    The worst node is taken for each way separately and the best of those kept -
///    which can only be dearer than the worst node of the best way (a lower bound
///    on the mass).
/// 3. **The escape**, as for any parked launch, from the height it leaves from.
///
/// A departure steeper than the orbit's tilt is not offered (as for a parked
/// launch, [`crate::orbital_assembly::parking_plane_reaches`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StandingStack {
    /// The parked stack, at the height it waits at, with its [`Waiting`] cost.
    pub delivery: crate::orbital_assembly::ParkedDelivery,
    /// The orbit's tilt, rad.
    pub inclination_rad: f64,
}

/// The shipping standing stack: the shipping parked stack at
/// [`STANDING_ALTITUDE_M`], tilted like any Cape launch.
pub const SHIPPING_STANDING_STACK: StandingStack = StandingStack {
    delivery: crate::orbital_assembly::ParkedDelivery {
        parking_altitude_m: STANDING_ALTITUDE_M,
        ..crate::orbital_assembly::SHIPPING_PARKED_DELIVERY
    },
    inclination_rad: crate::orbital_assembly::PARKING_INCLINATION_DEG * std::f64::consts::PI
        / 180.0,
};

/// How a standing stack brought its plane round, for a readout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneMethod {
    /// Held at another height and came back.
    HoldAndReturn,
    /// Held at another height and left from there.
    HoldAndLeave,
    /// Turned the plane at the far end of a high loop.
    ApogeeTurn,
}

/// What a standing stack delivers through one window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StandingDelivery {
    /// The mass that leaves Earth toward the rock, kg (the separated mass).
    pub separated_kg: f64,
    /// Station-keeping Δv spent before the escape, m/s: drag plus the plane.
    pub station_keeping_dv_m_s: f64,
    /// How the plane was brought round.
    pub method: PlaneMethod,
    /// The height it left from, m.
    pub leave_altitude_m: f64,
}

impl StandingStack {
    /// What leaves Earth through a window of characteristic energy `c3_km2_s2` and
    /// departure declination `declination_rad`, for a stack that has been in orbit
    /// `in_orbit_s` before the go-ahead (the years before the rock was found, plus
    /// the decision) and leaves `hold_s` after it. `None` when the departure is
    /// steeper than the orbit, or no way reaches it.
    pub fn deliver(
        &self,
        c3_km2_s2: f64,
        declination_rad: f64,
        in_orbit_s: f64,
        hold_s: f64,
    ) -> Option<StandingDelivery> {
        let d = &self.delivery;
        let w = d.waiting?;
        let m0 = d.stack_kg;
        let home = d.parking_altitude_m;
        let i = self.inclination_rad;
        if declination_rad.abs() > i || !c3_km2_s2.is_finite() {
            return None;
        }
        let hold = hold_s.max(0.0);
        let g0isp = w.thrusters.isp_s * crate::impactor_mass::G0_M_S2;
        let kept = |dv: f64| (-dv / g0isp).exp();
        // Drag at home until the go-ahead.
        let before = w.drag.dv_rate_m_s2(home, m0)? * in_orbit_s.max(0.0);
        // The escape from height `h`, as a fraction of the stack.
        let escape_from = |h: f64| {
            let at = crate::orbital_assembly::ParkedDelivery {
                parking_altitude_m: h,
                ..*d
            };
            at.separated_mass_kg(c3_km2_s2) / m0
        };
        let esc_home = escape_from(home);
        let mut options: Vec<(f64, f64, PlaneMethod, f64)> = Vec::new();

        // 1. Turn the plane at the far end of a high loop: drag at home throughout.
        if let crate::orbital_assembly::EscapeBurn::Finite { apogee_cap_m, .. } = d.escape {
            let dv = before
                + w.drag.dv_rate_m_s2(home, m0)? * hold
                + worst_apogee_turn_dv_m_s(home, apogee_cap_m, i, declination_rad, c3_km2_s2);
            options.push((kept(dv) * esc_home, dv, PlaneMethod::ApogeeTurn, home));
        }
        // 2. Hold and come back: the worst node, continuous heights.
        if let Some((dv, _)) = worst_node_hold_dv_m_s(home, i, declination_rad, hold, &w.drag, m0) {
            let dv = before + dv;
            options.push((kept(dv) * esc_home, dv, PlaneMethod::HoldAndReturn, home));
        }
        // 3. Hold and leave from there, on the height grid: the worst node, by the
        // same crossing as `worst_node_hold_dv_m_s` but on the mass that leaves.
        if let Some(gap) = node_gap_rad(declination_rad, i) {
            let leave = |x: f64, raise: bool| -> Option<(f64, f64, f64)> {
                if x <= 0.0 {
                    return Some((esc_home, 0.0, home));
                }
                let (_, h) = node_hold_dv_m_s(home, i, x, hold, raise, &w.drag, m0)?;
                let q = (h / HOLD_GRID_M).abs();
                let hq = if raise { q.ceil() } else { q.floor() } * HOLD_GRID_M;
                if !(HOLD_ALTITUDE_RANGE_M.0..=HOLD_ALTITUDE_RANGE_M.1).contains(&hq) {
                    return None;
                }
                let dv = hohmann_dv_m_s(home, hq) + w.drag.dv_rate_m_s2(hq, m0)? * hold;
                Some((kept(dv) * escape_from(hq), dv, hq))
            };
            let mass = |o: Option<(f64, f64, f64)>| o.map_or(0.0, |v| v.0);
            // A node `x` into the gap: lowered by `x`, or raised by `gap - x`; the
            // luckier of the two. Lowering's mass falls with `x`, raising's grows.
            let (mut a, mut b) = (0.0_f64, gap);
            for _ in 0..60 {
                let m = 0.5 * (a + b);
                if mass(leave(m, false)) > mass(leave(gap - m, true)) {
                    a = m;
                } else {
                    b = m;
                }
            }
            // The worst of the two bracket ends, each at its luckier direction.
            let at = |x: f64| {
                let (lo, hi) = (leave(x, false), leave(gap - x, true));
                if mass(lo) >= mass(hi) {
                    lo
                } else {
                    hi
                }
            };
            let (pa, pb) = (at(a), at(b));
            let worst = if mass(pa) <= mass(pb) { pa } else { pb };
            if let Some((m, dv, hq)) = worst {
                let m = m * kept(before);
                if m > 0.0 {
                    options.push((m, before + dv, PlaneMethod::HoldAndLeave, hq));
                }
            }
        }
        let best = options
            .into_iter()
            .filter(|o| o.0 > 0.0)
            .max_by(|a, b| a.0.total_cmp(&b.0))?;
        Some(StandingDelivery {
            separated_kg: best.0 * m0,
            station_keeping_dv_m_s: best.1,
            method: best.2,
            leave_altitude_m: best.3,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const YEAR: f64 = crate::readiness::YEAR_S;
    const STACK: f64 = crate::orbital_assembly::FALCON_SINGLE_PAYLOAD_LIMIT_KG;

    /// The table reads back at its own rows, interpolates between them, falls with
    /// height in every column, and refuses to extrapolate.
    #[test]
    fn the_density_table_is_read_not_invented() {
        assert_eq!(
            density_kg_m3(400e3, SolarActivity::WholeCycle),
            Some(4.567e-12)
        );
        assert_eq!(
            density_kg_m3(550e3, SolarActivity::WholeCycle),
            Some(4.442e-13)
        );
        assert_eq!(
            density_kg_m3(1000e3, SolarActivity::Active),
            Some(3.122e-14)
        );
        let mid = density_kg_m3(412.5e3, SolarActivity::WholeCycle).unwrap();
        assert!(
            (mid - (4.567e-12_f64 * 3.019e-12).sqrt()).abs() < 1e-18,
            "{mid}"
        );
        for act in [
            SolarActivity::Quiet,
            SolarActivity::WholeCycle,
            SolarActivity::Active,
        ] {
            let mut last = f64::INFINITY;
            for k in 0..=65 {
                let rho = density_kg_m3(350e3 + 10e3 * k as f64, act).unwrap();
                assert!(rho < last, "{act:?} at {} km", 350 + 10 * k);
                last = rho;
            }
        }
        assert_eq!(density_kg_m3(349e3, SolarActivity::WholeCycle), None);
        assert_eq!(density_kg_m3(1001e3, SolarActivity::WholeCycle), None);
        // A quiet sun is thinner than the cycle, an active one thicker, at every row.
        for r in DENSITY_TABLE {
            assert!(r.1 < r.2 && r.2 < r.3, "{} km", r.0);
        }
    }

    /// The drag cost at the sizes the session measured by hand: ~31 m/s a year at
    /// 400 km on the tumbling fairing, ~1.5 at 600 km, a factor of ~4 less nose-on.
    #[test]
    fn the_drag_is_the_size_the_sources_give() {
        let at = |h: f64, d: &DragModel| d.dv_rate_m_s2(h, STACK).unwrap() * YEAR;
        let (a400, a600) = (at(400e3, &SHIPPING_DRAG), at(600e3, &SHIPPING_DRAG));
        assert!((28.0..34.0).contains(&a400), "{a400}");
        assert!((1.2..1.8).contains(&a600), "{a600}");
        let ratio = FAIRING_TUMBLING_AREA_M2 / FAIRING_NOSE_ON_AREA_M2;
        assert!((at(400e3, &NOSE_ON_DRAG) * ratio - a400).abs() < 1e-9);
        assert!((86.0..88.0).contains(&FAIRING_TUMBLING_AREA_M2));
        // A decade at 400 km costs several per cent of the stack; at 600 km, a few
        // tenths of one.
        let left = |h| {
            SHIPPING_DRAG
                .mass_left(h, STACK, 10.0 * YEAR, &ORION_AUX)
                .unwrap()
        };
        assert!((0.88..0.92).contains(&left(400e3)), "{}", left(400e3));
        assert!(left(600e3) > 0.99, "{}", left(600e3));
    }

    /// J2 regression at the sizes in every textbook: ~7.1 deg a day at 400 km and
    /// 28.5 deg, slower higher up.
    #[test]
    fn the_plane_turns_at_the_textbook_rate() {
        let i = 28.5_f64.to_radians();
        let d = |h: f64| node_rate_rad_s(h, i).to_degrees() * 86_400.0;
        assert!((d(400e3) - 7.08).abs() < 0.02, "{}", d(400e3));
        assert!((d(600e3) - 6.39).abs() < 0.02, "{}", d(600e3));
        assert!(d(1000e3) < d(600e3));
    }

    /// The two aligned nodes: half a turn apart for a departure on the equator, and
    /// meeting (a full turn the long way) when it grazes the orbit's top; nothing
    /// for a steeper one.
    #[test]
    fn the_node_gap_runs_from_half_a_turn_to_a_whole_one() {
        use std::f64::consts::PI;
        let i = 28.5_f64.to_radians();
        assert!((node_gap_rad(0.0, i).unwrap() - PI).abs() < 1e-12);
        assert!((node_gap_rad(i, i).unwrap() - 2.0 * PI).abs() < 1e-6);
        let mid = node_gap_rad(15_f64.to_radians(), i).unwrap();
        assert!(mid > PI && mid < 2.0 * PI);
        assert_eq!(node_gap_rad(30_f64.to_radians(), i), None);
        // Symmetric in the sign of the declination.
        assert_eq!(node_gap_rad(-0.2, i), node_gap_rad(0.2, i));
    }

    /// Holding at another height: free for no shift, dearer for a bigger one or a
    /// shorter time, impossible when no height in the range turns it far enough, and
    /// the worst case sits between the two one-way costs of the half gap.
    #[test]
    fn swinging_the_plane_costs_what_the_rates_say() {
        let i = 28.5_f64.to_radians();
        let h0 = 600e3;
        let d = &SHIPPING_DRAG;
        assert_eq!(
            node_hold_dv_m_s(h0, i, 0.0, YEAR, true, d, STACK),
            Some((0.0, h0))
        );
        let one = |s: f64, t: f64, up| node_hold_dv_m_s(h0, i, s, t, up, d, STACK);
        let a = one(1.0, YEAR, true).unwrap().0;
        assert!(one(2.0, YEAR, true).unwrap().0 > a);
        assert!(one(1.0, 0.5 * YEAR, true).unwrap().0 > a);
        // A year at 1 000 km gains (6.39 - 5.26) deg/d * 365 = ~410 deg; 2 pi needs
        // more time than a month.
        assert!(one(std::f64::consts::TAU, 30.0 * 86_400.0, true).is_none());
        assert!(one(std::f64::consts::TAU, 30.0 * 86_400.0, false).is_none());
        let (w, h) = worst_node_hold_dv_m_s(h0, i, 0.0, YEAR, d, STACK).unwrap();
        let half = std::f64::consts::FRAC_PI_2;
        let (lo, hi) = (
            one(half, YEAR, false).unwrap().0,
            one(half, YEAR, true).unwrap().0,
        );
        assert!(
            w >= lo.min(hi) - 1e-6 && w <= lo.max(hi) + 1e-6,
            "{w} vs {lo}, {hi}"
        );
        assert!((HOLD_ALTITUDE_RANGE_M.0..=HOLD_ALTITUDE_RANGE_M.1).contains(&h));
        // Two weeks is too short for the worst node on the equator.
        assert!(worst_node_hold_dv_m_s(h0, i, 0.0, 14.0 * 86_400.0, d, STACK).is_none());
    }

    /// The apogee turn: a few hundred m/s from a 100 000 km loop, more for a steeper
    /// departure, and it does not depend on the time available.
    #[test]
    fn turning_at_apogee_is_dear_but_always_there() {
        let i = 28.5_f64.to_radians();
        let cap = crate::orbital_assembly::SHIPPING_APOGEE_CAP_M;
        let flat = worst_apogee_turn_dv_m_s(600e3, cap, i, 0.0, 40.0);
        let steep = worst_apogee_turn_dv_m_s(600e3, cap, i, 25_f64.to_radians(), 40.0);
        assert!((250.0..450.0).contains(&flat), "{flat}");
        assert!(steep > flat);
    }

    /// The standing stack, at the sizes the offline calculation gave (the session's
    /// `phasing.py` on the flown windows): a window two months after the go-ahead
    /// only by the apogee turn, ~750 m/s; one ~half a year out by holding, a few
    /// per cent; a decade of waiting at 600 km costs well under one per cent; a
    /// steeper departure than the orbit is not offered.
    #[test]
    fn a_standing_stack_pays_for_its_plane_and_its_years() {
        let s = SHIPPING_STANDING_STACK;
        assert_eq!(s.delivery.parking_altitude_m, 600e3);
        let day = 86_400.0;
        let direct = s.delivery.separated_mass_kg(75.1);
        let early = s
            .deliver(75.1, (-24.9_f64).to_radians(), 0.0, 58.0 * day)
            .unwrap();
        assert_eq!(early.method, PlaneMethod::ApogeeTurn);
        // ~650 m/s offline on the bare angle; the turn about the loop's apsides has
        // to go further (sin φ = sin ψ / sin θ∞, 63 deg for this 53 deg) - ~750.
        assert!(
            (650.0..800.0).contains(&early.station_keeping_dv_m_s),
            "{early:?}"
        );
        let later = s
            .deliver(16.1, (-1.7_f64).to_radians(), 0.0, 156.0 * day)
            .unwrap();
        assert_ne!(later.method, PlaneMethod::ApogeeTurn, "{later:?}");
        assert!(
            (50.0..220.0).contains(&later.station_keeping_dv_m_s),
            "{later:?}"
        );
        // A decade before the warning: drag at 600 km is ~15 m/s on the upper-bound area.
        let fresh = s.deliver(16.1, 0.1, 0.0, 400.0 * day).unwrap();
        let old = s.deliver(16.1, 0.1, 10.0 * YEAR, 400.0 * day).unwrap();
        let lost = 1.0 - old.separated_kg / fresh.separated_kg;
        assert!((0.002..0.01).contains(&lost), "{lost}");
        assert!(fresh.separated_kg < s.delivery.separated_mass_kg(16.1));
        assert!(early.separated_kg < direct);
        assert!(s
            .deliver(16.1, 30_f64.to_radians(), 0.0, 400.0 * day)
            .is_none());
        // More time to swing the plane is never dearer.
        let mut last = 0.0;
        for days in [100.0, 200.0, 400.0, 800.0, 1600.0] {
            let m = s
                .deliver(30.0, 0.2, 5.0 * YEAR, days * day)
                .unwrap()
                .separated_kg;
            assert!(m >= last - 1e-6, "{days} d: {m} after {last}");
            last = m;
        }
    }

    /// The Hohmann helper is the textbook one: zero for no change, symmetric.
    #[test]
    fn hohmann_is_symmetric_and_zero_at_home() {
        assert_eq!(hohmann_dv_m_s(600e3, 600e3), 0.0);
        let (up, down) = (hohmann_dv_m_s(600e3, 800e3), hohmann_dv_m_s(800e3, 600e3));
        assert!((up - down).abs() < 1e-9);
        assert!((100.0..115.0).contains(&up), "{up}");
    }
}
