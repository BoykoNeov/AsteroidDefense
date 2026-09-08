//! Tier 3, second half: the Öpik b-plane frame **pinned**, the closed-form
//! post-encounter orbit, resonant-return circles, and keyhole widths
//! (HANDOFF §5 *Keyholes*, §7, and the open question this module closes).
//!
//! `uncertainty.rs` answers *how much of the orbit's spread lands on Earth*.
//! This module answers the question that only makes sense inside that one:
//! *where on the b-plane does a miss set up a return* — a **keyhole**. A flyby
//! that misses Earth still turns the asteroid's heliocentric velocity, and if the
//! turned orbit's period is commensurable with Earth's (`k` revolutions in `h`
//! years) the two meet again at the same place `h` years later. The set of
//! b-plane points that produce one such resonance is a **circle**, and the
//! keyhole is the thin band around it that returns *onto the capture disc*.
//!
//! # The theory, derived from the deflection this project already models
//!
//! The flyby turns the Earth-relative velocity through `δ`, `tan(δ/2) = c/b`
//! with `c = μ⊕/v∞²`, bending it *toward Earth* — from `Ŝ` toward `−B̂`. Adding
//! Earth's heliocentric velocity back gives the outgoing heliocentric orbit:
//!
//! ```text
//!   Ŝ_out = cos δ · Ŝ − sin δ · B̂            v_out = V⊕ + v∞ · Ŝ_out
//! ```
//!
//! That is already everything. In the Öpik frame below, with `θ` the angle
//! between `Ŝ` and `V⊕`, the outgoing angle `θ'` obeys **Valsecchi et al. (2003)**
//! eq. for the post-encounter geometry,
//!
//! ```text
//!   cos θ' = [ (b² − c²) cos θ + 2 c ζ sin θ ] / (b² + c²)         b² = ξ² + ζ²
//! ```
//!
//! and this module proves the two are the *same statement*: substituting
//! `cos δ = (b² − c²)/(b² + c²)`, `sin δ = 2bc/(b² + c²)` and `B̂·ζ̂ = ζ/b` into
//! `Ŝ_out · V̂⊕` reproduces the formula term for term. The tests pin that identity
//! numerically on random geometries, which is what licenses using the analytic
//! form for circles and gradients while the rotation form stays the physical
//! meaning.
//!
//! Given `cos θ'`, vis-viva at Earth's heliocentric position gives the outgoing
//! semi-major axis `a'`. A level set of `a'` is a level set of `cos θ'`, which the
//! formula above makes a **circle centred on the ζ-axis**:
//!
//! ```text
//!   centre  ζ_c = c · sin θ / (cos θ' − cos θ)
//!   radius  R   = c · |sin θ'| / |cos θ' − cos θ|
//! ```
//!
//! — the *resonant circle* of a resonance `a' = (h/k)^(2/3) AU`.
//!
//! # The frame, and why it had to be pinned here
//!
//! `geometry.rs` left the b-vector's sign and the ξ,ζ decomposition deliberately
//! unpinned, and `uncertainty.rs` proved it could afford to (the impact
//! probability is invariant under any orthonormal change of b-plane basis). A
//! resonant circle is not: its centre sits at a specific `ζ`, so the convention
//! has to be settled, and it is settled **by derivation and by measurement**, not
//! adopted:
//!
//! - **`B` points from Earth's centre to the incoming asymptote** — the side the
//!   asteroid arrives on. Derived from the hyperbola's centre `C = a·e·P̂` lying on
//!   the asymptote (`geometry.rs` now pins this with a test), and *measured* on a
//!   flown flyby by `probe_keyhole_rotation`: `−B̂` predicts the outgoing `a` to
//!   `1.5e-4`, `+B̂` misses by `7.4e-2` — a 489× separation.
//! - **`η̂ = Ŝ`** — the incoming direction of motion.
//! - **`ζ̂` is anti-parallel to the projection of Earth's heliocentric velocity
//!   onto the b-plane.** This is exactly the sign under which Valsecchi's
//!   `+2cζ sin θ` term comes out with a plus, which the identity test checks.
//!   Physically, `ζ` is the *timing* coordinate: moving along `ζ` is the asteroid
//!   arriving earlier or later against Earth's motion.
//! - **`ξ̂ = η̂ × ζ̂`**, so `(ξ̂, η̂, ζ̂)` is right-handed. `ξ` is (to first order)
//!   the minimum orbit intersection distance — how far the two orbits miss each
//!   other in space, which no timing change can fix.
//!
//! # What the closed form is good for, stated before it is used
//!
//! Taking the encounter position as Earth's own costs `η ≈ 7e-5` in heliocentric
//! radius, hence `δa'/a' ≈ 1.3e-4`: over a 7-year return that is a ~12 h slip,
//! about a hundred capture radii of return placement. So the *absolute* `a'` says
//! which resonances are reachable and where their circles lie to ~1e-4, and
//! nothing finer. The **gradient** `∂a'/∂(ξ,ζ)` is trustworthy, because that
//! error is common-mode across neighbouring points and cancels to first order —
//! and the gradient is what a keyhole width is. Every quantitative claim about a
//! *return encounter* still has to come from the propagator; this module is the
//! map that says where to fly.
//!
//! **The size of that paragraph is right, its named cause is not, and the error
//! is in the wrong half of the construction** — measured 2026-09-08 against four
//! flown 3:4 doors by `probe_keyhole_placement outgoing`, which reads the
//! semi-major axis the propagator actually produces (a mean over one full
//! revolution, ±41 km-equivalent going out and ±136 coming in) rather than the one
//! this module predicts. Three findings, in the order they were established.
//!
//! **The resonance condition is sound.** A flown door centre leaves the rock 14 to
//! 49 km-equivalent from `a_res` at every lead — half a door width, and −14 to
//! −16 km at three of the four — while the door's distance from its circle runs
//! 19 → 786 km. So `a' = a_res` is the right target and this module's `a'` is the
//! wrong prediction of it.
//!
//! **Every input to the prediction is innocent.** Swapped one at a time into the
//! nominal frame — the flown asymptote `Ŝ`, the flown `v∞`, Earth's position and
//! Earth's velocity at the flight's own encounter — each moves the answer, none of
//! them orders by lead, and their sum is not the bias either (they are additive to
//! within 10 %, so this is not a cancellation). The `r ≈ R⊕ₒᵣᵦ` substitution named
//! above is dead as the cause: at the 300 d door the rock is **53 km** from
//! Earth's heliocentric distance — a relative `3.6e-7`, some **200× below** the
//! `7e-5` this paragraph charges — while the error is at its full `1.4e-4`, and
//! substituting the rock's own distance moves the circle 2.8 km against the 665 km
//! wanted. Nor is it the component the nominal projection drops when the flown
//! b-vector tips out of the nominal plane: that is second order, and 149 km out of
//! plane costs **0.0 km**. Nor is it the frame — rebuilding `c`, `θ` and Earth's
//! state from the deflected flight's own encounter makes the prediction worse at
//! three of the four leads.
//!
//! **The error is in the baseline, not in the turn.** Asking the same construction
//! on the leg *before* the encounter splits it, because
//! [`OpikFrame::incoming_semi_major_axis`] is the identical arithmetic with the
//! incoming asymptote in place of the outgoing one. The baseline error — which
//! orbit the rock arrives on — runs **+293, +84, −422, −458** km-equivalent across
//! deflection leads of 4383, 900, 300 and 200 days; the outgoing error runs −33,
//! −227, −665, −839; and **their difference, the error in the change across the
//! encounter and the only part the flyby itself owns, is flat at −326, −310, −242,
//! −381**, a 55 km spread at the two extremes against 750 and 805 km in the two
//! absolutes, and inside the ±136 km bar of the measurement. In the flight's own
//! frame it is flatter still: −372, −365, −374 at three of the four leads.
//!
//! **The repair, and it ships.** Place the circle where `a' − a_in` equals
//! `a_res − a_in_true`, taking `a_in_true` from the flight the planner has already
//! propagated, and the door offsets go from `+19 / +211 / +648 / +786` — a 767 km
//! ladder — to `+312 / +294 / +226 / +329`, a spread of 103 km about a constant. A
//! single osculating sample instead of a 32-sample revolution mean is exactly as
//! flat (spread 102 km, at a +380 km offset instead) and costs nothing, because the
//! planner already holds that state; that is what
//! [`OpikFrame::resonant_circle_on_change`] takes, fed by
//! [`incoming_semi_major_axis_flown`](crate::keyhole_target::incoming_semi_major_axis_flown).
//! Pinned by `core/tests/keyhole_prediction_bias.rs`.
//!
//! # The unit the placement error is a constant in
//!
//! Everything above is the 3:4, whose `|∇a'|` moves 1.75 % across the four leads —
//! so "a constant number of b-plane kilometres" and "a constant amount of `a'`" are
//! the same statement there, and the cheaper one was assumed. A **2:3 flown at two
//! leads** separates them, because its gradient is ten times steeper on the same
//! encounter (263.8 against 26.1 m/m):
//!
//! ```text
//!            error in the change    in b-plane km    in km of a'
//!   3:4 × 4                          +242 … +381    +6 274 … +10 045
//!   2:3 × 2                          + 25 … + 28    +6 485 … + 7 409
//! ```
//!
//! Twelve times apart in kilometres — the gradient ratio — and overlapping in `a'`.
//! So the closed form misplaces a circle by a constant amount of **semi-major
//! axis**, and the b-plane distance that comes to is whatever the local geometry
//! makes it. [`Keyhole::placement_band`] is that conversion, and it is why the
//! frontend's band moved from 800 b-plane km to 15 000 km of `a'` — which is
//! 575 km on the 3:4 and 57 km on the 2:3.
//!
//! **The constant itself is not subtracted.** All six flights sit on the same side,
//! 8 184 to 11 024 km of `a'` out on the shipping single-sample convention, so a
//! further correction is visible in the data. Six flights over two resonances
//! cannot set it — the spread is 1.35× — and trading a known error for a badly
//! known one is not a repair. That is the next thing to measure.
//!
//! # The keyhole width, as a definition rather than a claim
//!
//! A `Δa'` shifts the period by `ΔT/T = 1.5·Δa'/a'`; after the `k` revolutions of
//! an `h`-year return the arrival slips by `Δt = h·yr·1.5·Δa'/a'`, during which
//! Earth moves `V⊕·Δt`. Calling one capture *diameter* the tolerance gives the
//! `Δa'` that still returns onto the disc, and dividing by `|∇a'|` gives the width
//! of the band around the circle. Order-unity, stated so it can be argued with.
//! The measured consequence that motivated this module survives it: near-grazing
//! resonances have the steepest gradient and the *narrowest* keyholes; the wide
//! ones are far out where the deflection is weak.
//!
//! **How much order-unity slack, measured.** The one keyhole this project has
//! flown ([`keyhole_target`](crate::keyhole_target), the 3:4 at Δv 0.216550 m/s)
//! sits **20.4 km** from its circle against a **24.9 km** width — 1.64 *half*-
//! widths, i.e. outside the band by this definition — and it returns *inside
//! Earth* three years later. So the width here is conservative by about 1.6× on
//! the one case with an answer. Quote it as "how many keyhole widths away", never
//! as a yes/no: the binary would say no to a plan that comes back.
//!
//! (That 0.216550 is the 12-iteration stop; the sharper floor is `0.2165483096`.
//! The 1.6× calibration is unaffected — it comes from the flown trajectory's
//! `∂ζ₂/∂ζ₁`, not from where the search stopped.)

use nalgebra::{Vector2, Vector3};

use crate::geometry::BPlaneEncounter;

/// One astronomical unit, metres (IAU 2012) — the unit resonances are named in.
pub const AU_M: f64 = 1.495_978_707e11;

/// One Julian year, seconds — the unit `h` counts in an `h:k` resonance.
pub const JULIAN_YEAR_S: f64 = 365.25 * 86_400.0;

/// Why an Öpik frame could not be built.
#[derive(Debug, Clone, PartialEq)]
pub enum KeyholeError {
    /// Some input was not finite.
    NonFinite,
    /// Earth's heliocentric velocity is (anti-)parallel to the incoming
    /// asymptote, so it has no projection onto the b-plane and `ζ̂` is undefined.
    /// Physically impossible for a real encounter (the asteroid would have to
    /// approach exactly along Earth's motion), so this is a caller bug, not a case.
    DegenerateFrame {
        /// `|sin θ|` — how far from parallel the two directions were.
        sin_theta: f64,
    },
    /// `μ☉`, `|V⊕|` or `|r⊕|` was not positive.
    NonPositiveParameter,
}

impl std::fmt::Display for KeyholeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KeyholeError::NonFinite => write!(f, "non-finite input to the Öpik frame"),
            KeyholeError::DegenerateFrame { sin_theta } => write!(
                f,
                "Earth's velocity is parallel to the incoming asymptote (sin θ = {sin_theta:.3e}): \
                 the ζ axis is undefined"
            ),
            KeyholeError::NonPositiveParameter => {
                write!(f, "μ☉, |V⊕| and |r⊕| must all be positive")
            }
        }
    }
}

impl std::error::Error for KeyholeError {}

/// An `h:k` mean-motion resonance with Earth: the asteroid completes `k`
/// revolutions in the `h` years Earth takes to complete `h` — so they meet again
/// `h` years after the encounter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Resonance {
    /// Years until the return: Earth's revolutions.
    pub h: u32,
    /// The asteroid's revolutions in that time.
    pub k: u32,
}

impl Resonance {
    /// The resonant heliocentric semi-major axis, metres: `(h/k)^(2/3) AU`.
    pub fn semi_major_axis_m(&self) -> f64 {
        (self.h as f64 / self.k as f64).powf(2.0 / 3.0) * AU_M
    }

    /// The resonant orbital period, seconds: `h/k` Julian years.
    pub fn period_seconds(&self) -> f64 {
        self.h as f64 / self.k as f64 * JULIAN_YEAR_S
    }

    /// Every coprime `h:k` with `h` in `years` and `k ≤ max_k`, sorted by
    /// semi-major axis. Non-coprime pairs name the same orbit as their reduced
    /// form and are dropped rather than double-counted.
    pub fn census(years: std::ops::RangeInclusive<u32>, max_k: u32) -> Vec<Resonance> {
        let mut out: Vec<Resonance> = Vec::new();
        for h in years {
            for k in 1..=max_k {
                if gcd(h, k) == 1 {
                    out.push(Resonance { h, k });
                }
            }
        }
        out.sort_by(|a, b| {
            a.semi_major_axis_m()
                .partial_cmp(&b.semi_major_axis_m())
                .expect("finite")
        });
        out
    }
}

impl std::fmt::Display for Resonance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}:{}", self.h, self.k)
    }
}

fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

/// The Öpik b-plane frame of one encounter, with everything needed to read the
/// post-encounter heliocentric orbit off a b-plane point in closed form.
///
/// Build it with [`OpikFrame::new`] from the encounter's b-plane reduction and
/// Earth's heliocentric state at the encounter. Coordinates are `(ξ, ζ)` metres
/// in the b-plane; see the module doc for the sign conventions and what pins them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpikFrame {
    /// `η̂ = Ŝ`, the incoming asymptote — the direction of motion far before the
    /// encounter. The b-plane is perpendicular to it.
    pub eta_hat: Vector3<f64>,
    /// `ξ̂ = η̂ × ζ̂`: the in-plane axis perpendicular to Earth's velocity. To
    /// first order, the minimum orbit intersection distance.
    pub xi_hat: Vector3<f64>,
    /// `ζ̂`: anti-parallel to the projection of Earth's heliocentric velocity on
    /// the b-plane. The timing coordinate.
    pub zeta_hat: Vector3<f64>,
    /// Hyperbolic excess speed `v∞`, m/s.
    pub v_inf: f64,
    /// Earth's `μ⊕`, m³/s².
    pub mu_earth: f64,
    /// The Sun's `μ☉`, m³/s² — the outgoing orbit is heliocentric.
    pub mu_sun: f64,
    /// The gravitationally-focused capture radius at this `v∞`, metres — the
    /// disc a return has to land on, which sizes the keyhole tolerance.
    pub capture_radius: f64,
    /// Earth's heliocentric position at the encounter, metres. The encounter
    /// position is taken to be Earth's own (the `r ≈ R⊕ₒᵣᵦ` approximation).
    pub r_earth: Vector3<f64>,
    /// Earth's heliocentric velocity at the encounter, m/s.
    pub v_earth: Vector3<f64>,
    /// `cos θ`, `θ` the angle between `Ŝ` and `V⊕`.
    pub cos_theta: f64,
    /// `sin θ > 0` (the frame is only defined when it is).
    pub sin_theta: f64,
}

/// Which of the two places a vertical line `ξ = const` crosses a resonant circle.
///
/// Named by the coordinate rather than by "near"/"far", because which one is
/// nearer Earth depends on the sign of the circle's centre `ζ_c` and a name that
/// silently flips with the geometry is a name that gets used wrongly. Use
/// [`ResonantCircle::points_at_xi`] and compare `norm()` when the question really
/// is "which is closer to Earth".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CircleBranch {
    /// `ζ = ζ_c − √(R² − ξ²)` — the lower-ζ crossing (arriving later against
    /// Earth's motion, since `ζ̂` opposes it).
    Minus,
    /// `ζ = ζ_c + √(R² − ξ²)` — the upper-ζ crossing.
    Plus,
}

/// A resonant-return circle: the locus of b-plane points whose flyby leaves the
/// asteroid on the `h:k` resonant orbit. Centred on the ζ-axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResonantCircle {
    /// Which resonance.
    pub resonance: Resonance,
    /// The resonant semi-major axis, metres — the `a'` the *resonance* asks for.
    pub a_prime: f64,
    /// The `a'` level set this circle actually **is**, metres.
    ///
    /// Equal to [`a_prime`](Self::a_prime) for [`OpikFrame::resonant_circle`], and
    /// that is the only case there was until 2026-09-08. It differs for
    /// [`OpikFrame::resonant_circle_on_change`], which draws the locus where the
    /// closed form's *change* across the encounter is right rather than its
    /// absolute `a'` — the two are the same claim only if the construction knows
    /// which orbit the rock arrives on, and it does not (module doc).
    ///
    /// Carried rather than recomputed because every scalar below — `cos_theta_out`,
    /// the centre, the radius — belongs to this number, and a reader who
    /// re-derives `a'` from `cos_theta_out` must land on the same value or the
    /// struct is lying to them. The keyhole *width* is sized from `a_prime`
    /// instead: the width is a return-timing tolerance, and the return happens on
    /// the resonant orbit, not on the level set the map drew to reach it. (They
    /// differ by ~1e-4, so this is a statement about which number means what, not
    /// a numerical correction.)
    pub a_prime_target: f64,
    /// The outgoing `cos θ'` that produces
    /// [`a_prime_target`](Self::a_prime_target).
    pub cos_theta_out: f64,
    /// Centre `ζ_c`, metres (`ξ_c = 0` always).
    pub center_zeta: f64,
    /// Radius, metres.
    pub radius: f64,
}

impl ResonantCircle {
    /// A point on the circle, parameterised by angle from the `+ξ` direction.
    pub fn point(&self, angle: f64) -> Vector2<f64> {
        Vector2::new(
            self.radius * angle.cos(),
            self.center_zeta + self.radius * angle.sin(),
        )
    }

    /// The point of the circle closest to Earth's centre — on the ζ-axis, at the
    /// smallest `|b|` this resonance can be reached at. If the circle encloses
    /// the origin the closest point is the origin itself, and the resonance is
    /// reachable in every direction; the returned point is then the origin.
    pub fn nearest_point(&self) -> Vector2<f64> {
        if self.encloses_origin() {
            return Vector2::zeros();
        }
        let sign = self.center_zeta.signum();
        Vector2::new(0.0, self.center_zeta - sign * self.radius)
    }

    /// The point of the circle farthest from Earth's centre, on the ζ-axis.
    pub fn farthest_point(&self) -> Vector2<f64> {
        let sign = if self.center_zeta == 0.0 {
            1.0
        } else {
            self.center_zeta.signum()
        };
        Vector2::new(0.0, self.center_zeta + sign * self.radius)
    }

    /// Whether Earth's centre lies inside the circle.
    pub fn encloses_origin(&self) -> bool {
        self.center_zeta.abs() < self.radius
    }

    /// The range of impact parameters `|b|` the circle spans: `(min, max)`.
    pub fn b_range(&self) -> (f64, f64) {
        (
            (self.center_zeta.abs() - self.radius).max(0.0),
            self.center_zeta.abs() + self.radius,
        )
    }

    /// Whether any part of the circle lies inside the capture disc — i.e. part
    /// of the "keyhole" is an impact already and not a return.
    ///
    /// This is the common case, not the exception, and it is what the reach
    /// probe's outward bisection could not see: on the shipping rock the 3:4
    /// circle runs from `b = 3 866 km` (deep inside the 11 311 km disc) out to
    /// `153 577 km`, so the part of the locus nearest Earth is an impact, and the
    /// nearest *miss* on it is the grazing point
    /// ([`intersections_at_radius`](Self::intersections_at_radius) at the capture
    /// radius) — not the 60 843 km the probe reported, which was the first
    /// crossing its sweep could bracket.
    pub fn crosses_capture_disc(&self, capture_radius: f64) -> bool {
        self.b_range().0 < capture_radius
    }

    /// Where the circle meets the ring `|b| = radius`: the two points `(+ξ, ζ)`
    /// and `(−ξ, ζ)`, mirror images in ξ, or `None` if the ring misses the circle
    /// (or the circle is centred on the origin, where the two are either the same
    /// circle or disjoint). At `radius = b_capture` these are the grazing points —
    /// the nearest-to-Earth places on the resonance that are still a miss.
    pub fn intersections_at_radius(&self, radius: f64) -> Option<(Vector2<f64>, Vector2<f64>)> {
        let d = self.center_zeta;
        if d.abs() < 1e-9 * self.radius.max(1.0) {
            return None;
        }
        let zeta = (radius * radius - self.radius * self.radius + d * d) / (2.0 * d);
        let xi2 = radius * radius - zeta * zeta;
        if xi2 < 0.0 {
            return None;
        }
        let xi = xi2.sqrt();
        Some((Vector2::new(xi, zeta), Vector2::new(-xi, zeta)))
    }

    /// Where the vertical line `ξ = xi` crosses this circle: `(minus, plus)`,
    /// the lower-ζ point first. `None` when the line misses the circle
    /// (`|ξ| > radius`) — **the reachability answer**, and the reason this is not
    /// a bare `√(R² − ξ²)` at a call site: a resonance whose circle is narrower
    /// than the ξ a deflection naturally lands on cannot be aimed at from that ξ
    /// at all, and a NaN would say so only by poisoning everything downstream.
    ///
    /// At `|ξ| = radius` the two coincide (the tangent point).
    pub fn points_at_xi(&self, xi: f64) -> Option<(Vector2<f64>, Vector2<f64>)> {
        let disc = self.radius * self.radius - xi * xi;
        if !(disc >= 0.0) {
            return None;
        }
        let root = disc.sqrt();
        Some((
            Vector2::new(xi, self.center_zeta - root),
            Vector2::new(xi, self.center_zeta + root),
        ))
    }

    /// One of the two [`points_at_xi`](Self::points_at_xi), chosen by branch.
    pub fn point_at_xi(&self, xi: f64, branch: CircleBranch) -> Option<Vector2<f64>> {
        self.points_at_xi(xi).map(|(minus, plus)| match branch {
            CircleBranch::Minus => minus,
            CircleBranch::Plus => plus,
        })
    }

    /// The point of the circle nearest an arbitrary b-plane point — the radial
    /// projection of `p` onto the circle from its centre `(0, ζ_c)`.
    ///
    /// `None` only when `p` *is* the centre, where every point of the circle is
    /// equidistant and "nearest" has no answer. A caller wanting a number there
    /// wants [`Self::signed_distance`], which is `−radius` and well defined.
    pub fn closest_point_to(&self, p: Vector2<f64>) -> Option<Vector2<f64>> {
        let centre = Vector2::new(0.0, self.center_zeta);
        let d = p - centre;
        let n = d.norm();
        if !(n > 0.0) {
            return None;
        }
        Some(centre + d * (self.radius / n))
    }

    /// How far `p` is from the circle, metres: **positive outside** the circle,
    /// negative inside it, zero on it. `|p − (0, ζ_c)| − radius`.
    ///
    /// This is the raw distance to the *locus*, not a statement about the return.
    /// Whether that distance is small enough to matter is the keyhole width, and
    /// the width is only meaningful where the gradient was evaluated — see
    /// [`OpikFrame::nearest_keyhole`], which pairs the two.
    pub fn signed_distance(&self, p: Vector2<f64>) -> f64 {
        (p - Vector2::new(0.0, self.center_zeta)).norm() - self.radius
    }
}

/// The linearised keyhole at one point of a resonant circle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Keyhole {
    /// Where on the circle, `(ξ, ζ)` metres.
    pub at: Vector2<f64>,
    /// `∇a'` there, metres of `a'` per metre of b-plane displacement.
    pub gradient: Vector2<f64>,
    /// The `Δa'` that still returns onto the capture disc, metres.
    pub semi_major_axis_tolerance: f64,
    /// The keyhole's full width across the circle, metres: `Δa'_tol / |∇a'|`.
    pub width: f64,
}

impl Keyhole {
    /// How far from this circle the map's own placement error can reach, metres
    /// of b-plane, given a band `band_a` expressed in **metres of `a'`**.
    ///
    /// `band_a / |∇a'|`, and the unit is the whole point. Until 2026-09-08 the
    /// frontend carried this band as a fixed number of b-plane kilometres
    /// (`KEYHOLE_PLACEMENT_KM`, latterly 800), because every door that had ever
    /// been flown was on the 3:4 and its gradient moves 1.75 % across the leads —
    /// so a constant in `a'` and a constant in kilometres are the same statement
    /// there and the cheaper one was chosen. A 2:3 flown at two leads separates
    /// them: across a **ten times** different gradient the error is +25.5 and
    /// +28.1 b-plane km against the 3:4's +242 to +381 — a 12× move, the gradient
    /// ratio — while in `a'` it is 6 485 and 7 409 km against 6 274 to 10 045, which
    /// overlaps. The error is a constant of the *construction*, and the b-plane
    /// distance it corresponds to is whatever the local gradient makes it.
    ///
    /// So a band in kilometres is 30× too generous at a steep circle and, at a
    /// near-tangency one, not generous enough by any factor at all: as `∇a' → 0`
    /// this diverges, and it diverges together with [`width`](Self::width), which
    /// is the same limit and is already documented as the linearisation's artifact.
    /// Both come back non-finite, and
    /// [`keyhole_proximities`](OpikFrame::keyhole_proximities) filters such circles
    /// out before a caller can rank on either.
    ///
    /// Negative or non-finite `band_a` gives `0.0`: a caller that has no band is
    /// asking for the door alone, not for an infinite one.
    pub fn placement_band(&self, band_a: f64) -> f64 {
        if !(band_a > 0.0) {
            return 0.0;
        }
        band_a / self.gradient.norm()
    }
}

/// How close a given b-plane point came to one resonant return's keyhole — the
/// answer to *"where does this plan leave the rock, in keyhole terms?"*.
///
/// The distance is to the **circle**, in the b-plane, in metres. That is a map
/// coordinate and not a prediction of a return: `keyhole.rs`'s module doc puts
/// the closed form's absolute placement error at `δa'/a' ≈ 1.3e-4`, which over
/// an `h`-year return is hours of arrival slip — a million kilometres of Earth's
/// motion against a keyhole tens of kilometres wide. Only a flown return says
/// what actually happens. What the map *is* good for is the gradient, and the
/// gradient is what [`Keyhole::width`] is, so the comparison of `distance`
/// against `half_width` is self-consistent even where the absolute placement is
/// not: it says how many keyhole widths of aiming error this plan carries.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeyholeProximity {
    /// The resonance whose circle this is.
    pub circle: ResonantCircle,
    /// The point on that circle nearest the queried b-point, `(ξ, ζ)` metres.
    pub closest_point: Vector2<f64>,
    /// Distance from the queried point to the circle, metres: **positive
    /// outside** the circle, negative inside it.
    pub signed_distance: f64,
    /// The linearised keyhole evaluated **at `closest_point`** — the place the
    /// plan would have to move to, so the width quoted is the width of the door
    /// it is aiming at rather than of some other point of the same circle.
    pub keyhole: Keyhole,
}

impl KeyholeProximity {
    /// Half the keyhole's full width — the tolerance `signed_distance` is
    /// measured against. Infinite at a near-tangency circle (radius → 0, where
    /// `∇a' → 0`), which [`OpikFrame::keyhole_at`] returns as infinity on
    /// purpose rather than clamping.
    pub fn half_width(&self) -> f64 {
        0.5 * self.keyhole.width
    }

    /// Whether the queried point lies inside the keyhole band — i.e. the plan,
    /// as aimed, sets up this resonant return. See the type doc before quoting
    /// this as a prediction: it is the map's answer, not a flown one.
    pub fn inside(&self) -> bool {
        self.signed_distance.abs() <= self.half_width()
    }

    /// `|signed_distance| / half_width` — how many keyhole half-widths of aiming
    /// error the plan carries against **this** circle: `< 1` is inside the door,
    /// `10` is ten doors away.
    ///
    /// **Do not rank two circles by it.** That is what this doc claimed until
    /// 2026-09-07, and `probe_keyhole_placement`'s five flown doors falsified it.
    /// The *width* calibrates — the linearised door is conservative by at most
    /// 1.44× — but the *placement* error is 2.0 to 26.8 km, is not proportional
    /// to the width, and follows none of the three laws tried, so it behaves as
    /// an additive unknown that lands on every circle alike. Divided by a wide
    /// door that unknown vanishes; divided by a narrow one it dominates. The
    /// ratio therefore ranks a 200 km-wide door 300 km away ahead of a 25 km one
    /// 60 km away, when the second is the one within reach of the placement
    /// error. Rank by [`margin`](Self::margin) instead.
    pub fn widths_away(&self) -> f64 {
        self.signed_distance.abs() / self.half_width()
    }

    /// How far **outside its own door** the queried point lies, metres:
    /// `|signed_distance| − half_width`. Negative inside the door, zero on its
    /// edge.
    ///
    /// The quantity to rank circles by and to cut an alert on, because the
    /// closed form's placement error is additive (see
    /// [`widths_away`](Self::widths_away)): a kilometre of margin means the same
    /// thing at a wide door as at a narrow one, and each door's own width has
    /// already been taken out. `−∞` at a near-tangency circle, whose width is
    /// infinite — those never reach a caller, being filtered out of
    /// [`keyhole_proximities`](OpikFrame::keyhole_proximities).
    pub fn margin(&self) -> f64 {
        self.signed_distance.abs() - self.half_width()
    }

    /// How far outside **everything the map could be wrong about** the queried
    /// point lies, metres: [`margin`](Self::margin) less this circle's own
    /// placement band, with `band_a` in metres of `a'` (see
    /// [`Keyhole::placement_band`]). Negative means the door is within reach of the
    /// map's error and the plan cannot be called clear of it.
    ///
    /// **This is the ranking key and the alert cut, and they are the same
    /// expression on purpose.** The 2026-09-07 session replaced a width *ratio*
    /// with [`margin`](Self::margin) for exactly this reason — the row shown and
    /// the row the alert fires on have to be the same row — and the band's move
    /// from kilometres to `a'` moves the key with it. Ranking by `margin` while
    /// cutting on a per-circle band would name a shallow-gradient circle 300 km
    /// away ahead of a steep one 40 km away, when the second is the one whose band
    /// is 40 km wide.
    pub fn exposure(&self, band_a: f64) -> f64 {
        self.margin() - self.keyhole.placement_band(band_a)
    }
}

impl OpikFrame {
    /// Build the frame from an encounter's b-plane reduction and Earth's
    /// heliocentric state at that encounter (metres, m/s, any inertial frame
    /// shared with the encounter's `Ŝ` — the core's ICRF).
    ///
    /// `enc` may be the closest-approach reduction or the fixed-epoch one
    /// `uncertainty.rs` uses; the asymptote is the same to the invariance of the
    /// hyperbola. Use the *same* reduction the covariance was mapped with when
    /// the two are to be drawn together.
    pub fn new(
        enc: &BPlaneEncounter,
        r_earth_helio: Vector3<f64>,
        v_earth_helio: Vector3<f64>,
        mu_sun: f64,
    ) -> Result<Self, KeyholeError> {
        let finite = enc.s_hat.iter().all(|c| c.is_finite())
            && r_earth_helio.iter().all(|c| c.is_finite())
            && v_earth_helio.iter().all(|c| c.is_finite())
            && enc.v_inf.is_finite()
            && enc.mu.is_finite()
            && enc.capture_radius.is_finite()
            && mu_sun.is_finite();
        if !finite {
            return Err(KeyholeError::NonFinite);
        }
        let v_mag = v_earth_helio.norm();
        if !(mu_sun > 0.0 && v_mag > 0.0 && r_earth_helio.norm() > 0.0 && enc.v_inf > 0.0) {
            return Err(KeyholeError::NonPositiveParameter);
        }
        let eta_hat = enc.s_hat.normalize();
        let v_hat = v_earth_helio / v_mag;
        let cos_theta = eta_hat.dot(&v_hat);
        // The projection of V̂⊕ onto the b-plane; ζ̂ points the other way.
        let w = v_hat - cos_theta * eta_hat;
        let sin_theta = w.norm();
        if sin_theta < 1.0e-9 {
            return Err(KeyholeError::DegenerateFrame { sin_theta });
        }
        let zeta_hat = -w / sin_theta;
        let xi_hat = eta_hat.cross(&zeta_hat);
        Ok(Self {
            eta_hat,
            xi_hat,
            zeta_hat,
            v_inf: enc.v_inf,
            mu_earth: enc.mu,
            mu_sun,
            capture_radius: enc.capture_radius,
            r_earth: r_earth_helio,
            v_earth: v_earth_helio,
            cos_theta,
            sin_theta,
        })
    }

    /// `c = μ⊕/v∞²`, metres — the impact parameter at which the flyby turns the
    /// velocity through 90°; the scale of every circle below.
    pub fn c(&self) -> f64 {
        self.mu_earth / (self.v_inf * self.v_inf)
    }

    /// `θ`, radians — the angle between the incoming asymptote and Earth's
    /// heliocentric velocity.
    pub fn theta(&self) -> f64 {
        self.sin_theta.atan2(self.cos_theta)
    }

    /// A b-plane 3-vector expressed in this frame: `(ξ, ζ)` metres.
    pub fn project(&self, b_vector: &Vector3<f64>) -> Vector2<f64> {
        Vector2::new(b_vector.dot(&self.xi_hat), b_vector.dot(&self.zeta_hat))
    }

    /// `(ξ, ζ)` back to a 3-vector in the encounter's inertial frame.
    pub fn unproject(&self, p: Vector2<f64>) -> Vector3<f64> {
        p.x * self.xi_hat + p.y * self.zeta_hat
    }

    /// The perigee radius of the flyby hyperbola with impact parameter `b`:
    /// `r_p = −c + √(c² + b²)`, the inverse of
    /// [`impact_parameter_for_perigee`](Self::impact_parameter_for_perigee).
    ///
    /// Exposed because the b-plane is where a keyhole lives and a perigee is what
    /// a deflection solver targets (`DeflectionScenario::required_dv`), so this
    /// conversion sits on every path from "aim at this circle" to "here is the
    /// Δv". Doing it by hand at each call site is how the two drift apart.
    pub fn perigee_for_impact_parameter(&self, b: f64) -> f64 {
        let c = self.c();
        -c + (c * c + b * b).sqrt()
    }

    /// `b² = r_p² + 2 c r_p` — gravitational focusing, the other direction.
    pub fn impact_parameter_for_perigee(&self, r_p: f64) -> f64 {
        (r_p * r_p + 2.0 * self.c() * r_p).max(0.0).sqrt()
    }

    /// The flyby turn angle `δ` at impact parameter `b`: `tan(δ/2) = c/b`.
    pub fn turn_angle(&self, b: f64) -> f64 {
        2.0 * (self.c() / b).atan()
    }

    /// The outgoing asymptote for an incoming b-vector, by the rotation the
    /// module doc derives: `cos δ · Ŝ − sin δ · B̂`. The physical statement; the
    /// closed forms below are proved against it.
    pub fn outgoing_asymptote(&self, b_vector: &Vector3<f64>) -> Vector3<f64> {
        let b = b_vector.norm();
        let b_hat = b_vector / b;
        let delta = self.turn_angle(b);
        delta.cos() * self.eta_hat - delta.sin() * b_hat
    }

    /// Heliocentric semi-major axis of the orbit the asteroid is *on* as it
    /// arrives: `V⊕ + v∞·Ŝ` at Earth's position. Compare this against the real
    /// pre-encounter orbit and the frame is validated end to end (the reach
    /// probe's round-trip gate, `8.7e-5` on the shipping rock).
    ///
    /// **This is also where the keyhole map's placement error lives** (measured
    /// 2026-09-08). That `8.7e-5` gate is taken on the *nominal* rock, which is the
    /// rock the frame is built from; on the **deflected** flights that actually fly
    /// a door, the same number is wrong by `+293`, `+84`, `−422` and `−458` b-plane
    /// km-equivalent at deflection leads of 4383, 900, 300 and 200 days. That
    /// ladder — and not anything the flyby does — is what puts a flown door 19 to
    /// 786 km from its circle: the error in the *change* across the encounter is
    /// flat at about −320 km at every lead. See the module doc and
    /// `core/tests/keyhole_prediction_bias.rs`.
    pub fn incoming_semi_major_axis(&self) -> f64 {
        self.semi_major_axis_of(&(self.v_earth + self.v_inf * self.eta_hat))
    }

    /// Post-encounter semi-major axis by the rotation construction, metres.
    /// Negative means the flyby ejected the asteroid onto a hyperbolic orbit.
    pub fn post_encounter_semi_major_axis_by_rotation(&self, b_vector: &Vector3<f64>) -> f64 {
        let s_out = self.outgoing_asymptote(b_vector);
        self.semi_major_axis_of(&(self.v_earth + self.v_inf * s_out))
    }

    /// Valsecchi's `cos θ'` at a b-plane point — the closed form.
    pub fn cos_theta_out(&self, p: Vector2<f64>) -> f64 {
        let c = self.c();
        let b2 = p.norm_squared();
        ((b2 - c * c) * self.cos_theta + 2.0 * c * p.y * self.sin_theta) / (b2 + c * c)
    }

    /// Post-encounter semi-major axis at a b-plane point, metres, in closed form.
    /// Identical to [`post_encounter_semi_major_axis_by_rotation`] — the tests
    /// pin it — but cheap enough to sweep and differentiable analytically.
    ///
    /// [`post_encounter_semi_major_axis_by_rotation`]: Self::post_encounter_semi_major_axis_by_rotation
    pub fn post_encounter_semi_major_axis(&self, p: Vector2<f64>) -> f64 {
        self.semi_major_axis_for_cos_theta_out(self.cos_theta_out(p))
    }

    /// The `cos θ'` that lands the asteroid on semi-major axis `a`, or `None` if
    /// no flyby can (`|cos θ'| > 1`): the resonance is out of this encounter's
    /// reach at any `b`.
    pub fn cos_theta_out_for_semi_major_axis(&self, a: f64) -> Option<f64> {
        if !(a.is_finite() && a > 0.0) {
            return None;
        }
        let r = self.r_earth.norm();
        let v2_out = self.mu_sun * (2.0 / r - 1.0 / a);
        let v = self.v_earth.norm();
        let u = self.v_inf;
        let cos = (v2_out - v * v - u * u) / (2.0 * v * u);
        (cos.is_finite() && cos.abs() <= 1.0).then_some(cos)
    }

    /// `∇a'` at a b-plane point: `(∂a'/∂ξ, ∂a'/∂ζ)`, dimensionless (metres per
    /// metre). Analytic, by the chain rule through `cos θ'`; the tests pin it
    /// against central differences. This is the one output the `r ≈ R⊕ₒᵣᵦ`
    /// approximation does *not* corrupt to first order.
    pub fn gradient_semi_major_axis(&self, p: Vector2<f64>) -> Vector2<f64> {
        let c = self.c();
        let b2 = p.norm_squared();
        let n = (b2 - c * c) * self.cos_theta + 2.0 * c * p.y * self.sin_theta;
        let d = b2 + c * c;
        let dn_dxi = 2.0 * p.x * self.cos_theta;
        let dn_dzeta = 2.0 * p.y * self.cos_theta + 2.0 * c * self.sin_theta;
        let dd_dxi = 2.0 * p.x;
        let dd_dzeta = 2.0 * p.y;
        let dcos_dxi = (dn_dxi * d - n * dd_dxi) / (d * d);
        let dcos_dzeta = (dn_dzeta * d - n * dd_dzeta) / (d * d);
        // a = 1/(2/r − v²/μ) ⇒ da = a²·d(v²)/μ, and d(v²) = 2·V⊕·v∞·d(cos θ').
        let a = self.semi_major_axis_for_cos_theta_out(n / d);
        let da_dcos = a * a * 2.0 * self.v_earth.norm() * self.v_inf / self.mu_sun;
        Vector2::new(da_dcos * dcos_dxi, da_dcos * dcos_dzeta)
    }

    /// The resonant circle of `resonance`, or `None` if this encounter cannot
    /// reach it (no `cos θ'` produces that `a'`), or if the resonance *is* the
    /// incoming orbit (`cos θ' = cos θ`), whose level set is the ξ-axis line
    /// rather than a circle.
    pub fn resonant_circle(&self, resonance: Resonance) -> Option<ResonantCircle> {
        self.circle_at_level_set(resonance, resonance.semi_major_axis_m())
    }

    /// The circle of constant `a' = target`, labelled with `resonance`. The one
    /// place a `ResonantCircle` is built, so the plain and the change-placed
    /// constructors cannot drift apart in the geometry; they differ only in which
    /// level set they ask for and in what `a_prime` is then said to mean.
    fn circle_at_level_set(&self, resonance: Resonance, target: f64) -> Option<ResonantCircle> {
        let cos_out = self.cos_theta_out_for_semi_major_axis(target)?;
        let denom = cos_out - self.cos_theta;
        if denom.abs() < 1.0e-12 {
            return None;
        }
        let c = self.c();
        let sin_out = (1.0 - cos_out * cos_out).max(0.0).sqrt();
        Some(ResonantCircle {
            resonance,
            a_prime: target,
            a_prime_target: target,
            cos_theta_out: cos_out,
            center_zeta: c * self.sin_theta / denom,
            radius: c * sin_out / denom.abs(),
        })
    }

    /// The same circle placed on the **change** the flyby makes to the orbit
    /// instead of on the absolute `a'` — the 2026-09-08 repair, and the one
    /// correction measured so far that makes the map's placement error *smaller*.
    ///
    /// [`resonant_circle`](Self::resonant_circle) draws the locus where this
    /// frame's absolute prediction `a'` equals `a_res`. That prediction carries a
    /// ladder: on four flown 3:4 doors it is wrong by −33, −227, −665 and −839
    /// b-plane km-equivalent as the deflection lead shortens from 12 yr to 200 d.
    /// The *change* across the encounter is not — it is out by a constant, +326,
    /// +310, +242, +381 km-equivalent on the same four, and +28, +26 on two 2:3
    /// crossings at a gradient ten times different. So the honest locus is the one
    /// where the change is right:
    ///
    /// ```text
    ///   a'(ξ, ζ) − a_in_predicted  =  a_res − a_in_true
    /// ```
    ///
    /// which is the ordinary circle of the shifted target
    /// `a_res + (a_in_predicted − a_in_true)`. `a_in_predicted` is this frame's own
    /// [`incoming_semi_major_axis`](Self::incoming_semi_major_axis) — the same
    /// arithmetic, so the two errors are like for like and the shift is exactly
    /// the part that cancels.
    ///
    /// `a_incoming_true` is the heliocentric semi-major axis the **deflected** rock
    /// is on as it arrives, metres, read off the flown trajectory rather than
    /// predicted — see
    /// [`incoming_semi_major_axis_flown`](crate::keyhole_target::incoming_semi_major_axis_flown),
    /// which is the observable this was calibrated with. Passing this frame's own
    /// `incoming_semi_major_axis()` returns exactly
    /// [`resonant_circle`](Self::resonant_circle), which is the sense in which the
    /// repair is a strict generalisation.
    ///
    /// **What it does not fix.** The change is predicted out by a constant of
    /// roughly 8 200 to 9 700 km *of `a'`* (the range across two resonances and six
    /// flights), and that constant is **not** subtracted here: six flights cannot
    /// set it. What the repair buys is that the residual is a constant in `a'`
    /// rather than a ladder in the lead — see [`Keyhole::placement_band`], which is
    /// how a caller is meant to allow for it.
    ///
    /// `None` on the same terms as [`resonant_circle`](Self::resonant_circle), plus
    /// a non-finite or non-positive `a_incoming_true`.
    pub fn resonant_circle_on_change(
        &self,
        resonance: Resonance,
        a_incoming_true: f64,
    ) -> Option<ResonantCircle> {
        if !(a_incoming_true.is_finite() && a_incoming_true > 0.0) {
            return None;
        }
        let a_res = resonance.semi_major_axis_m();
        let shift = self.incoming_semi_major_axis() - a_incoming_true;
        let mut circle = self.circle_at_level_set(resonance, a_res + shift)?;
        circle.a_prime = a_res;
        Some(circle)
    }

    /// [`resonant_circles`](Self::resonant_circles), with every circle placed on
    /// the change ([`resonant_circle_on_change`](Self::resonant_circle_on_change)).
    ///
    /// The census is taken on the *resonances*, so it names exactly the same set as
    /// the unrepaired call at the same arguments; only where each circle is drawn
    /// moves. A caller that draws one and reads the other would put a plan on the
    /// wrong side of a door, which is why this exists as a pair rather than as a
    /// flag on the loop.
    pub fn resonant_circles_on_change(
        &self,
        years: std::ops::RangeInclusive<u32>,
        max_k: u32,
        b_max: f64,
        a_incoming_true: f64,
    ) -> Vec<ResonantCircle> {
        Resonance::census(years, max_k)
            .into_iter()
            .filter_map(|r| self.resonant_circle_on_change(r, a_incoming_true))
            .filter(|c| c.b_range().0 <= b_max)
            .collect()
    }

    /// Every resonance in `years` with `k ≤ max_k` whose circle exists and comes
    /// within `b_max` of Earth's centre, sorted by `a'`. The census the reach
    /// probe ran by sweeping 72 directions, done exactly.
    pub fn resonant_circles(
        &self,
        years: std::ops::RangeInclusive<u32>,
        max_k: u32,
        b_max: f64,
    ) -> Vec<ResonantCircle> {
        Resonance::census(years, max_k)
            .into_iter()
            .filter_map(|r| self.resonant_circle(r))
            .filter(|c| c.b_range().0 <= b_max)
            .collect()
    }

    /// The `Δa'` that still returns onto the capture disc for this resonance:
    /// one capture diameter of Earth's motion over the `h`-year return, converted
    /// through `ΔT/T = 1.5·Δa'/a'`. See the module doc — a definition.
    pub fn semi_major_axis_tolerance(&self, circle: &ResonantCircle) -> f64 {
        let h_years = circle.resonance.h as f64;
        2.0 * self.capture_radius * circle.a_prime
            / (self.v_earth.norm() * h_years * JULIAN_YEAR_S * 1.5)
    }

    /// The linearised keyhole at `at` on `circle` — width `Δa'_tol / |∇a'|`.
    /// `at` should lie on the circle (use [`ResonantCircle::point`],
    /// [`nearest_point`](ResonantCircle::nearest_point),
    /// [`farthest_point`](ResonantCircle::farthest_point) or
    /// [`intersections_at_radius`](ResonantCircle::intersections_at_radius)); the
    /// gradient is evaluated where asked and the width is meaningful only there.
    ///
    /// A resonance at the very edge of reach (`cos θ' = ±1`) has a circle of
    /// zero radius: a single point where `∇a' = 0` and the width is `+∞` — the
    /// linearisation's tangency artifact the reach probe flagged as
    /// `NEAR-TANGENCY`. It is returned as infinity rather than clamped, so a
    /// caller sees the artifact for what it is.
    pub fn keyhole_at(&self, circle: &ResonantCircle, at: Vector2<f64>) -> Keyhole {
        let gradient = self.gradient_semi_major_axis(at);
        let tolerance = self.semi_major_axis_tolerance(circle);
        Keyhole {
            at,
            gradient,
            semi_major_axis_tolerance: tolerance,
            width: tolerance / gradient.norm(),
        }
    }

    /// Where a b-plane point stands against each of `circles` — one
    /// [`KeyholeProximity`] per circle, in the order given.
    ///
    /// `circles` is the caller's own mapped set (typically
    /// [`resonant_circles`](Self::resonant_circles)), *not* a census taken here,
    /// so a readout and the map it is read beside can never disagree about which
    /// resonances were considered — and so a caller can say "nothing in the
    /// mapped region" rather than quoting the nearest of a truncated list.
    ///
    /// Circles whose keyhole width is not finite are **skipped**: those are the
    /// near-tangency artifacts (radius → 0, `∇a' → 0`) the module doc flags, and
    /// an infinite width would rank ahead of every real keyhole while meaning
    /// nothing. They stay visible through `resonant_circles`; they are only
    /// unrankable.
    pub fn keyhole_proximities(
        &self,
        circles: &[ResonantCircle],
        p: Vector2<f64>,
    ) -> Vec<KeyholeProximity> {
        circles
            .iter()
            .filter_map(|c| {
                let closest = c.closest_point_to(p).unwrap_or_else(|| c.point(0.0));
                let keyhole = self.keyhole_at(c, closest);
                keyhole.width.is_finite().then_some(KeyholeProximity {
                    circle: *c,
                    closest_point: closest,
                    signed_distance: c.signed_distance(p),
                    keyhole,
                })
            })
            .collect()
    }

    /// The circle of `circles` whose locus passes closest to `p` in **metres**.
    /// The one to quote as "this plan lands N km from the h:k circle".
    pub fn nearest_keyhole(
        &self,
        circles: &[ResonantCircle],
        p: Vector2<f64>,
    ) -> Option<KeyholeProximity> {
        self.keyhole_proximities(circles, p)
            .into_iter()
            .min_by(|a, b| {
                a.signed_distance
                    .abs()
                    .partial_cmp(&b.signed_distance.abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }

    /// The circle of `circles` the point is nearest **in keyhole widths**.
    ///
    /// Kept because "which door is wide relative to how far away it is" is a
    /// legible quantity and because its disagreement with
    /// [`smallest_margin_keyhole`](Self::smallest_margin_keyhole) is itself
    /// informative — but it is **not** the risk ranking, and this doc used to say
    /// it was. See [`KeyholeProximity::widths_away`] for why a ratio is the wrong
    /// key once the placement error is known to be additive.
    pub fn tightest_keyhole(
        &self,
        circles: &[ResonantCircle],
        p: Vector2<f64>,
    ) -> Option<KeyholeProximity> {
        self.keyhole_proximities(circles, p)
            .into_iter()
            .min_by(|a, b| {
                a.widths_away()
                    .partial_cmp(&b.widths_away())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }

    /// The circle of `circles` whose **door** the point is nearest — the minimum
    /// of [`KeyholeProximity::margin`], i.e. the fewest kilometres from being
    /// inside a keyhole.
    ///
    /// This is the risk ranking, and the one to cut an alert on. It pairs with
    /// [`nearest_keyhole`](Self::nearest_keyhole) as an edge pairs with a locus:
    /// `nearest_keyhole` is the closest *circle*, the number to print beside the
    /// drawn map; this is the closest *door edge*, the number to compare against
    /// the placement band. The two name the same circle unless a wider door
    /// slightly further out beats the nearest circle's own.
    pub fn smallest_margin_keyhole(
        &self,
        circles: &[ResonantCircle],
        p: Vector2<f64>,
    ) -> Option<KeyholeProximity> {
        self.keyhole_proximities(circles, p)
            .into_iter()
            .min_by(|a, b| {
                a.margin()
                    .partial_cmp(&b.margin())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }

    /// The circle of `circles` the point is nearest **once the map's own placement
    /// error is allowed for** — the minimum of [`KeyholeProximity::exposure`],
    /// with `band_a` in metres of `a'`.
    ///
    /// This is the row to show and the row to cut the alert on, and it supersedes
    /// [`smallest_margin_keyhole`](Self::smallest_margin_keyhole) for that job.
    /// The two agree whenever every candidate circle has a similar gradient, which
    /// is every case the repo had flown before 2026-09-08 and is why the
    /// distinction had not come up; they part company exactly where the band does
    /// its work, between a wide shallow circle and a narrow steep one.
    pub fn most_exposed_keyhole(
        &self,
        circles: &[ResonantCircle],
        p: Vector2<f64>,
        band_a: f64,
    ) -> Option<KeyholeProximity> {
        self.keyhole_proximities(circles, p)
            .into_iter()
            .min_by(|x, y| {
                x.exposure(band_a)
                    .partial_cmp(&y.exposure(band_a))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }

    /// How many of `circles` have a **door** within `band` metres of `p` — the
    /// count of [`KeyholeProximity::margin`] at or under `band`.
    ///
    /// The question this answers is not "how close is the nearest door" but
    /// **"can a distance rule name one resonance at all here?"**, and it exists
    /// because the answer turned out to be no in the region the frontend
    /// actually works in.
    ///
    /// The placement band a caller passes is the closed form's own error in
    /// where it draws a circle. `probe_keyhole_placement` measured that error on
    /// the 3:4 at three deflection lead times and got 19.3 km at the campaign's
    /// 12 yr lead, **210.6 km at 900 days and 467.9 km at 450 days** — while the
    /// closest neighbouring pair of drawn circles at those same b-plane points is
    /// 402.2, **83.6 and 8.3 km** apart. So at the leads a mission planner can
    /// dial, the band is between 2.5× and 56× the spacing, and more than one door
    /// falls inside it. A caller that prints only the nearest one is not wrong
    /// about that one; it is silently answering a question whose answer is not
    /// unique, and this is how it finds out.
    ///
    /// `band_a` is the caller's, not the core's: it is a statement about how much
    /// the *map* can be trusted, which is a presentation decision. Non-finite or
    /// negative bands count nothing.
    ///
    /// **The unit changed on 2026-09-08**: `band_a` is metres of `a'`, and each
    /// circle converts it to its own b-plane reach through its own gradient (see
    /// [`Keyhole::placement_band`]). The kilometre figures in the paragraph above
    /// are the 3:4's, and stand as written because that is the circle they were
    /// measured on; on a circle ten times steeper the same band is a tenth as wide,
    /// which is the point of the change.
    pub fn doors_within_band(
        &self,
        circles: &[ResonantCircle],
        p: Vector2<f64>,
        band_a: f64,
    ) -> usize {
        if !(band_a >= 0.0) {
            return 0;
        }
        self.keyhole_proximities(circles, p)
            .into_iter()
            .filter(|k| k.exposure(band_a) <= 0.0)
            .count()
    }

    /// Vis-viva at Earth's heliocentric position for a heliocentric velocity.
    fn semi_major_axis_of(&self, v_helio: &Vector3<f64>) -> f64 {
        1.0 / (2.0 / self.r_earth.norm() - v_helio.norm_squared() / self.mu_sun)
    }

    fn semi_major_axis_for_cos_theta_out(&self, cos_out: f64) -> f64 {
        let v = self.v_earth.norm();
        let u = self.v_inf;
        let v2_out = v * v + u * u + 2.0 * v * u * cos_out;
        1.0 / (2.0 / self.r_earth.norm() - v2_out / self.mu_sun)
    }
}

/// The Earth-relative state **at perigee** of the hyperbola with excess speed
/// `v_inf`, impact parameter `b`, incoming asymptote `s_hat` and b-vector
/// direction `b_hat` (unit, `⊥ s_hat`) — the inverse of
/// [`BPlaneEncounter::from_relative_state`].
///
/// `geometry.rs` derives `Ŝ = (P̂ + √(e²−1)·Q̂)/e` and `B̂ = (√(e²−1)·P̂ − Q̂)/e`
/// from the perifocal axes; this inverts that pair: `P̂ = (Ŝ + √(e²−1)·B̂)/e`,
/// `Q̂ = (√(e²−1)·Ŝ − B̂)/e`. Feeding the result back through
/// `from_relative_state` reproduces `v_inf`, `b`, `Ŝ` and `B` to round-off, which
/// the tests pin — and which is what a targeting step will need to turn a chosen
/// b-plane point into a state to aim a propagation at.
pub fn perigee_state_for_asymptote(
    v_inf: f64,
    b: f64,
    s_hat: Vector3<f64>,
    b_hat: Vector3<f64>,
    mu: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let c = mu / (v_inf * v_inf);
    let r_p = -c + (c * c + b * b).sqrt();
    let e = 1.0 + r_p * v_inf * v_inf / mu;
    let root = (e * e - 1.0).sqrt();
    let s = s_hat.normalize();
    let bh = b_hat.normalize();
    let p_hat = (s + root * bh) / e;
    let q_hat = (root * s - bh) / e;
    let v_p = b * v_inf / r_p; // h = b·v∞ = r_p·v_p
    (r_p * p_hat, v_p * q_hat)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::EARTH_EQUATORIAL_RADIUS_M;

    const MU_EARTH: f64 = 3.986_004_356e14;
    const MU_SUN: f64 = 1.327_124_400_41e20;
    const R_EARTH: f64 = EARTH_EQUATORIAL_RADIUS_M;

    /// `doors_within_band` counts every door in the band, not just the nearest —
    /// and the several-doors branch is executed here rather than left to a plan
    /// that happens to find one.
    ///
    /// The branch exists because the placement error the frontend must allow for
    /// (measured at 210.6 km at a 900 day lead, 467.9 km at 450) is larger than
    /// the tightest spacing between drawn circles at the same b-plane points
    /// (83.6 km and 8.3 km). Where those two coincide, a distance rule cannot name
    /// one resonance. The `a_dialable_plan_that_returns_is_not_reported_clear`
    /// binding test found the flown 900 d plan sits somewhere the circles happen
    /// to be far apart, so it counts exactly one — which is why the crowded case
    /// needs its own test instead of being assumed to arise on its own.
    ///
    /// Kernel-free: it sweeps the census a real frame produces and asks for a
    /// band wide enough to hold several of its doors, then checks the count
    /// against the same proximities the ranking is computed from.
    #[test]
    fn doors_within_band_counts_every_door_not_just_the_nearest() {
        let mut seed = 0x5eed_face_u64;
        let (f, _) = random_frame(&mut seed);
        let circles = f.resonant_circles(2..=20, 24, 60.0 * f.capture_radius);
        assert!(
            circles.len() > 3,
            "this geometry has {} circles; the test needs a census to count within",
            circles.len()
        );
        let p = Vector2::new(0.3 * f.capture_radius, -2.0 * f.capture_radius);
        // The band is in metres of `a'` (2026-09-08), so the quantity to sort is
        // not the margin but **the band at which each door comes onto the edge** —
        // `margin × |∇a'|`, each circle converting through its own gradient. Doing
        // it any other way would test the old units through the new call.
        let mut margins: Vec<f64> = f
            .keyhole_proximities(&circles, p)
            .iter()
            .map(|k| k.margin() * k.keyhole.gradient.norm())
            .collect();
        margins.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!(margins.len() >= 3, "need at least three doors to rank");

        // A band under the smallest margin holds nothing; one just over the third
        // smallest holds exactly three. Both edges come from the proximities the
        // ranking itself uses, so this cannot pass by counting a different census.
        let just_under = margins[0] - 1.0;
        let just_over_third = 0.5 * (margins[2] + margins[3.min(margins.len() - 1)]);
        assert_eq!(
            f.doors_within_band(&circles, p, just_under.max(0.0)),
            0,
            "a band tighter than every margin must count nothing"
        );
        assert_eq!(
            f.doors_within_band(&circles, p, margins[0]),
            1,
            "a band at exactly the smallest margin counts the one door on its edge"
        );
        let n = f.doors_within_band(&circles, p, just_over_third);
        assert!(
            n >= 3,
            "a band past the third-smallest margin must count at least three doors; \
             counted {n} with bands {:.1?} km of a'",
            margins.iter().take(5).map(|m| m / 1e3).collect::<Vec<_>>()
        );

        // A nonsense band is not a wide band.
        assert_eq!(f.doors_within_band(&circles, p, -1.0), 0);
        assert_eq!(f.doors_within_band(&circles, p, f64::NAN), 0);
    }

    /// Passing the frame's own predicted incoming orbit back in must reproduce the
    /// plain circle **exactly**. That is what makes the repair a strict
    /// generalisation rather than a second construction to keep in step: if the two
    /// ever disagree here, the shifted-target algebra is wrong.
    #[test]
    fn the_change_placed_circle_is_the_plain_one_when_the_baseline_is_the_predicted_one() {
        let mut seed = 0x1dea_0908_u64;
        for _ in 0..40 {
            let (f, _) = random_frame(&mut seed);
            for r in Resonance::census(2..=9, 12) {
                let (Some(plain), Some(same)) = (
                    f.resonant_circle(r),
                    f.resonant_circle_on_change(r, f.incoming_semi_major_axis()),
                ) else {
                    continue;
                };
                assert_eq!(plain, same, "{r} circle moved with a zero baseline shift");
            }
        }
    }

    /// A bad baseline is refused rather than propagated: the repair takes a
    /// *measured* number, and a measurement that failed must not silently draw a
    /// circle somewhere.
    #[test]
    fn the_change_placed_circle_refuses_a_nonsense_baseline() {
        let mut seed = 0x0908_1dea_u64;
        let (f, _) = random_frame(&mut seed);
        let r = Resonance { h: 3, k: 4 };
        for bad in [f64::NAN, f64::INFINITY, 0.0, -1.0] {
            assert!(
                f.resonant_circle_on_change(r, bad).is_none(),
                "a baseline of {bad} produced a circle"
            );
        }
    }

    /// **The exact circle against the linearised shift.** The measurement campaign
    /// only ever computed the repair as a translation of the drawn circle along its
    /// own normal, because that is all a table of door offsets needs. What ships
    /// solves for a *different circle* — the level set of a shifted `a'`, with its
    /// own centre and its own radius. If the two disagreed, either the published
    /// tables or the shipping code would be wrong, and this is the only place that
    /// can say which.
    ///
    /// The shift asked for is 10 000 km of `a'`, the size the campaign measured
    /// (8 200 to 9 700 km across six flights), and the claim is an absolute one over
    /// the b-plane distances that shift actually produces — out to 500 km, against
    /// repaired doors sitting 226 to 418 km from their circles: **under 5 km**,
    /// where the measurement that set the correction carries a ±136 km bar. So the
    /// published tables, computed as a pure normal shift, describe the circle this
    /// code draws.
    ///
    /// They cannot agree exactly, and the bound is stated over a range rather than
    /// everywhere because of *why*: a level set of a different `a'` has a different
    /// radius, so the exact circle is a translation **plus** a change of curvature.
    /// The residual is second order but not in the shift alone — it is set by the
    /// shift against the circle's own size, so it is 22 m on a 12 km shift, 1.3 km
    /// on a 393 km one and 30 km once the shift reaches 1 433 km, three times
    /// further than anything the repair has been measured at. Reading the
    /// correction out there would need that term; nothing does, and a caller who
    /// starts to would find this test the place that says so.
    #[test]
    fn the_exact_change_placed_circle_matches_the_linearised_normal_shift() {
        let mut seed = 0xc17c_1e08_u64;
        let shift_a = 1.0e7; // metres of a'
        let mut checked = 0;
        let mut worst: f64 = 0.0;
        for _ in 0..60 {
            let (f, _) = random_frame(&mut seed);
            for r in Resonance::census(2..=9, 12) {
                let Some(plain) = f.resonant_circle(r) else {
                    continue;
                };
                // A baseline *smaller* than predicted shifts the target up.
                let Some(moved) =
                    f.resonant_circle_on_change(r, f.incoming_semi_major_axis() - shift_a)
                else {
                    continue;
                };
                // Ask at points on the plain circle, where the normal is the radial
                // direction and ∇a' lies along it.
                for angle in [0.4, 2.1, 4.7] {
                    let at = plain.point(angle);
                    let n_hat = (at - Vector2::new(0.0, plain.center_zeta)).normalize();
                    let grad_n = f.gradient_semi_major_axis(at).dot(&n_hat);
                    if !(grad_n.abs() > 1.0e-6) {
                        continue;
                    }
                    // The level set moves out by shift/∇, leaving the point that far
                    // inside the new circle.
                    let predicted = -shift_a / grad_n;
                    let exact = moved.signed_distance(at);
                    // Second order in the shift, so the comparison is made over
                    // the range the repair produces and not beyond it — see the doc.
                    if predicted.abs() > 500.0e3 {
                        continue;
                    }
                    let err = (exact - predicted).abs();
                    checked += 1;
                    worst = worst.max(err);
                    assert!(
                        err < 5.0e3,
                        "{r} at angle {angle}: exact {:.3} km vs linearised {:.3} km \
                         ({:.2} % of the shift)",
                        exact / 1e3,
                        predicted / 1e3,
                        100.0 * err / predicted.abs()
                    );
                }
            }
        }
        assert!(
            checked > 100,
            "only {checked} comparisons ran; the filters ate the test"
        );
        assert!(
            worst < 5.0e3,
            "worst disagreement {:.3} km over {checked} comparisons",
            worst / 1e3
        );
    }

    /// The band divides by the gradient, and it goes non-finite exactly where the
    /// keyhole width does — the near-tangency artifact both share. A caller must
    /// never see one finite and the other not, because the alert is cut on their
    /// difference.
    #[test]
    fn the_placement_band_is_the_band_in_a_over_the_gradient() {
        let mut seed = 0xba7d_0908_u64;
        let (f, _) = random_frame(&mut seed);
        let circles = f.resonant_circles(2..=12, 24, 60.0 * f.capture_radius);
        assert!(!circles.is_empty());
        let band_a = 1.5e7;
        for c in &circles {
            let at = c.point(1.0);
            let k = f.keyhole_at(c, at);
            let expected = band_a / k.gradient.norm();
            assert!(
                (k.placement_band(band_a) - expected).abs() <= 1e-9 * expected.abs().max(1.0),
                "band {} vs {expected}",
                k.placement_band(band_a)
            );
            assert_eq!(
                k.placement_band(band_a).is_finite(),
                k.width.is_finite(),
                "the band and the width must be finite together"
            );
            // No band is not an infinite band.
            assert_eq!(k.placement_band(0.0), 0.0);
            assert_eq!(k.placement_band(-1.0), 0.0);
            assert_eq!(k.placement_band(f64::NAN), 0.0);
        }
    }

    /// `exposure` is `margin` less that circle's own band, and
    /// `most_exposed_keyhole` minimises exactly the quantity `doors_within_band`
    /// cuts on. A ranking that does not match its own alert is how the width-ratio
    /// bug of 2026-09-07 happened.
    #[test]
    fn the_ranking_key_and_the_alert_cut_are_the_same_expression() {
        let mut seed = 0xa1e7_0908_u64;
        let (f, _) = random_frame(&mut seed);
        let circles = f.resonant_circles(2..=20, 24, 60.0 * f.capture_radius);
        assert!(circles.len() > 3);
        let p = Vector2::new(0.3 * f.capture_radius, -2.0 * f.capture_radius);
        let band_a = 1.5e7;
        let mut exposures: Vec<f64> = f
            .keyhole_proximities(&circles, p)
            .iter()
            .map(|k| {
                assert!(
                    (k.exposure(band_a) - (k.margin() - k.keyhole.placement_band(band_a))).abs()
                        < 1e-9,
                    "exposure is not margin less band"
                );
                k.exposure(band_a)
            })
            .collect();
        exposures.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let best = f
            .most_exposed_keyhole(&circles, p, band_a)
            .expect("a ranked circle");
        assert!(
            (best.exposure(band_a) - exposures[0]).abs() < 1e-9,
            "the ranked circle is not the minimum-exposure one"
        );
        let n_negative = exposures.iter().filter(|e| **e <= 0.0).count();
        assert_eq!(
            f.doors_within_band(&circles, p, band_a),
            n_negative,
            "the count and the ranking disagree about how many doors are in reach"
        );
    }

    /// A deterministic pseudo-random stream (LCG) — enough to spread geometries
    /// around without a dev-dependency.
    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((*seed >> 11) as f64) / ((1u64 << 53) as f64)
    }

    fn unit(seed: &mut u64) -> Vector3<f64> {
        loop {
            let v = Vector3::new(
                2.0 * lcg(seed) - 1.0,
                2.0 * lcg(seed) - 1.0,
                2.0 * lcg(seed) - 1.0,
            );
            let n = v.norm();
            if n > 0.2 && n < 1.0 {
                return v / n;
            }
        }
    }

    /// Earth on a circular 1 AU orbit at a random phase, plus an encounter with
    /// random `Ŝ`, `B̂` and `v_inf` in the NEO band.
    fn random_frame(seed: &mut u64) -> (OpikFrame, BPlaneEncounter) {
        let phase = std::f64::consts::TAU * lcg(seed);
        let r_e = AU_M * Vector3::new(phase.cos(), phase.sin(), 0.0);
        let v_circ = (MU_SUN / AU_M).sqrt();
        let v_e = v_circ * Vector3::new(-phase.sin(), phase.cos(), 0.0);
        let s_hat = unit(seed);
        let mut b_hat = unit(seed);
        b_hat -= b_hat.dot(&s_hat) * s_hat;
        let b_hat = b_hat.normalize();
        let v_inf = 3_000.0 + 15_000.0 * lcg(seed);
        let b = R_EARTH * (1.5 + 30.0 * lcg(seed));
        let (r, v) = perigee_state_for_asymptote(v_inf, b, s_hat, b_hat, MU_EARTH);
        let enc = BPlaneEncounter::from_relative_state(r, v, MU_EARTH, R_EARTH).unwrap();
        let frame = OpikFrame::new(&enc, r_e, v_e, MU_SUN).unwrap();
        (frame, enc)
    }

    #[test]
    fn perigee_state_inverts_the_bplane_reduction() {
        let mut seed = 7;
        for _ in 0..50 {
            let s_hat = unit(&mut seed);
            let mut b_hat = unit(&mut seed);
            b_hat -= b_hat.dot(&s_hat) * s_hat;
            let b_hat = b_hat.normalize();
            let v_inf = 2_000.0 + 20_000.0 * lcg(&mut seed);
            let b = R_EARTH * (1.0 + 50.0 * lcg(&mut seed));
            let (r, v) = perigee_state_for_asymptote(v_inf, b, s_hat, b_hat, MU_EARTH);
            let enc = BPlaneEncounter::from_relative_state(r, v, MU_EARTH, R_EARTH).unwrap();
            assert!((enc.v_inf - v_inf).abs() / v_inf < 1e-10, "v_inf");
            assert!((enc.impact_parameter - b).abs() / b < 1e-10, "b");
            assert!((enc.s_hat - s_hat).norm() < 1e-10, "Ŝ");
            assert!(
                (enc.b_vector / b - b_hat).norm() < 1e-10,
                "B̂ {:?} vs {:?}",
                enc.b_vector / b,
                b_hat
            );
            // And it really is the perigee: r ⊥ v.
            assert!(r.dot(&v).abs() < 1e-6 * r.norm() * v.norm());
        }
    }

    #[test]
    fn frame_is_orthonormal_right_handed_and_zeta_opposes_earths_motion() {
        let mut seed = 11;
        for _ in 0..50 {
            let (f, enc) = random_frame(&mut seed);
            for (a, b) in [
                (f.xi_hat, f.eta_hat),
                (f.eta_hat, f.zeta_hat),
                (f.zeta_hat, f.xi_hat),
            ] {
                assert!((a.norm() - 1.0).abs() < 1e-12);
                assert!(a.dot(&b).abs() < 1e-12);
            }
            // Right-handed (ξ, η, ζ).
            assert!((f.xi_hat.cross(&f.eta_hat) - f.zeta_hat).norm() < 1e-12);
            assert!((f.eta_hat - enc.s_hat).norm() < 1e-12);
            // ζ̂ points against Earth's motion; ξ̂ is perpendicular to it.
            assert!(f.zeta_hat.dot(&f.v_earth) < 0.0);
            assert!(f.xi_hat.dot(&f.v_earth).abs() < 1e-9 * f.v_earth.norm());
            assert!(f.sin_theta > 0.0);
            assert!((f.cos_theta * f.cos_theta + f.sin_theta * f.sin_theta - 1.0).abs() < 1e-12);
            // Project/unproject round-trips a b-plane vector.
            let p = f.project(&enc.b_vector);
            assert!((f.unproject(p) - enc.b_vector).norm() < 1e-6 * enc.impact_parameter);
            assert!((p.norm() - enc.impact_parameter).abs() / enc.impact_parameter < 1e-12);
        }
    }

    #[test]
    fn a_parallel_velocity_is_refused_not_guessed() {
        let s_hat = Vector3::new(0.0, 1.0, 0.0);
        let b_hat = Vector3::new(1.0, 0.0, 0.0);
        let (r, v) = perigee_state_for_asymptote(8_000.0, 5.0 * R_EARTH, s_hat, b_hat, MU_EARTH);
        let enc = BPlaneEncounter::from_relative_state(r, v, MU_EARTH, R_EARTH).unwrap();
        let v_e = 29_780.0 * s_hat; // Earth moving exactly along Ŝ
        match OpikFrame::new(&enc, Vector3::new(AU_M, 0.0, 0.0), v_e, MU_SUN) {
            Err(KeyholeError::DegenerateFrame { .. }) => {}
            other => panic!("expected DegenerateFrame, got {other:?}"),
        }
        assert_eq!(
            OpikFrame::new(&enc, Vector3::new(AU_M, 0.0, 0.0), Vector3::zeros(), MU_SUN),
            Err(KeyholeError::NonPositiveParameter)
        );
    }

    /// THE identity this module rests on: Valsecchi's closed-form `cos θ'` is the
    /// rotation `Ŝ_out = cos δ·Ŝ − sin δ·B̂` dotted with `V̂⊕`, under exactly the
    /// sign conventions the frame pins. A flipped `ζ̂`, a `+B̂` bend, or a wrong
    /// `δ` all break this at the percent level.
    #[test]
    fn closed_form_equals_the_rotation_construction() {
        let mut seed = 23;
        for _ in 0..200 {
            let (f, _) = random_frame(&mut seed);
            // Arbitrary b-plane points, not only the encounter's own B.
            let p = Vector2::new(
                (2.0 * lcg(&mut seed) - 1.0) * 40.0 * R_EARTH,
                (2.0 * lcg(&mut seed) - 1.0) * 40.0 * R_EARTH,
            );
            if p.norm() < 0.5 * R_EARTH {
                continue;
            }
            let b_vec = f.unproject(p);
            let by_rotation = f.post_encounter_semi_major_axis_by_rotation(&b_vec);
            let closed = f.post_encounter_semi_major_axis(p);
            assert!(
                (by_rotation - closed).abs() <= 1e-9 * by_rotation.abs().max(AU_M),
                "rotation {by_rotation:.6e} vs closed form {closed:.6e} at {p:?}"
            );
            // And cos θ' itself, which is the sharper check (a' can hide a sign
            // error behind a large |a'|).
            let s_out = f.outgoing_asymptote(&b_vec);
            let cos_rot = s_out.dot(&f.v_earth) / f.v_earth.norm();
            assert!(
                (cos_rot - f.cos_theta_out(p)).abs() < 1e-12,
                "cos θ' rotation {cos_rot} vs closed form {}",
                f.cos_theta_out(p)
            );
        }
    }

    #[test]
    fn a_wide_pass_leaves_the_orbit_alone_and_a_close_one_does_not() {
        let mut seed = 5;
        let (f, enc) = random_frame(&mut seed);
        let a_in = f.incoming_semi_major_axis();
        let b_hat = enc.b_vector / enc.impact_parameter;
        // b = 10⁶ capture radii: the turn is ~1e-6 rad and a' ≈ a to ~1e-5.
        let far = f.post_encounter_semi_major_axis_by_rotation(&(1.0e6 * f.capture_radius * b_hat));
        assert!(
            (far - a_in).abs() / a_in.abs() < 1e-4,
            "far {far} vs {a_in}"
        );
        // At c the turn is 90°: a' must differ from a by a lot.
        let close = f.post_encounter_semi_major_axis_by_rotation(&(f.c() * b_hat));
        assert!(
            (close - a_in).abs() / a_in.abs() > 1e-2,
            "close {close} vs {a_in}"
        );
        assert!((f.turn_angle(f.c()) - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn a_prime_is_mirror_symmetric_in_xi() {
        let mut seed = 31;
        for _ in 0..50 {
            let (f, _) = random_frame(&mut seed);
            let xi = (lcg(&mut seed) + 0.1) * 20.0 * R_EARTH;
            let zeta = (2.0 * lcg(&mut seed) - 1.0) * 20.0 * R_EARTH;
            let a = f.post_encounter_semi_major_axis(Vector2::new(xi, zeta));
            let m = f.post_encounter_semi_major_axis(Vector2::new(-xi, zeta));
            assert!((a - m).abs() <= 1e-12 * a.abs());
        }
    }

    /// Every point of a resonant circle really lands on the resonant `a'`, and
    /// the level set is *only* the circle: step off it and `a'` moves.
    #[test]
    fn resonant_circles_are_level_sets_of_a_prime() {
        let mut seed = 42;
        let mut circles_checked = 0;
        for _ in 0..100 {
            let (f, _) = random_frame(&mut seed);
            for circle in f.resonant_circles(1..=20, 24, 200.0 * f.capture_radius) {
                if circle.radius < 1.0e-3 * f.capture_radius {
                    // The reach limit: a point, not a circle, with ∇a' = 0.
                    continue;
                }
                circles_checked += 1;
                for i in 0..16 {
                    let p = circle.point(std::f64::consts::TAU * i as f64 / 16.0);
                    if p.norm() < 1e-3 * f.capture_radius {
                        continue; // through the origin: a' is undefined there
                    }
                    let a = f.post_encounter_semi_major_axis(p);
                    assert!(
                        (a - circle.a_prime).abs() <= 1e-8 * circle.a_prime,
                        "{} at {p:?}: a' {a:.9e} vs resonant {:.9e}",
                        circle.resonance,
                        circle.a_prime
                    );
                    // Off the circle, along the gradient, a' changes.
                    let g = f.gradient_semi_major_axis(p);
                    let step = 1.0e-2 * p.norm() * g / g.norm();
                    let off = f.post_encounter_semi_major_axis(p + step);
                    assert!(
                        (off - circle.a_prime).abs() > 1e-10 * circle.a_prime,
                        "{} at {p:?}: a' did not move off the circle (|∇a'| = {:.3e})",
                        circle.resonance,
                        g.norm()
                    );
                }
                // Centre on the ζ-axis, by construction of `point`; check the
                // reported b-range against the geometry.
                let (lo, hi) = circle.b_range();
                assert!(lo <= circle.nearest_point().norm() + 1e-6);
                assert!((hi - circle.farthest_point().norm()).abs() < 1e-6);
            }
        }
        assert!(
            circles_checked > 100,
            "only {circles_checked} circles found"
        );
    }

    #[test]
    fn the_analytic_gradient_matches_central_differences_and_is_normal_to_the_circle() {
        let mut seed = 77;
        for _ in 0..100 {
            let (f, _) = random_frame(&mut seed);
            let p = Vector2::new(
                (2.0 * lcg(&mut seed) - 1.0) * 30.0 * R_EARTH,
                (2.0 * lcg(&mut seed) - 1.0) * 30.0 * R_EARTH,
            );
            if p.norm() < 1.0 * R_EARTH {
                continue;
            }
            let g = f.gradient_semi_major_axis(p);
            let h = 1.0e-4 * p.norm();
            let fd = Vector2::new(
                (f.post_encounter_semi_major_axis(p + Vector2::new(h, 0.0))
                    - f.post_encounter_semi_major_axis(p - Vector2::new(h, 0.0)))
                    / (2.0 * h),
                (f.post_encounter_semi_major_axis(p + Vector2::new(0.0, h))
                    - f.post_encounter_semi_major_axis(p - Vector2::new(0.0, h)))
                    / (2.0 * h),
            );
            assert!(
                (g - fd).norm() <= 1e-5 * g.norm().max(1e-30),
                "analytic {g:?} vs central difference {fd:?}"
            );
            // Normal to the level set: parallel to the radius vector from the
            // circle centre through p. The circle through p is the one for a'(p).
            let a = f.post_encounter_semi_major_axis(p);
            if a <= 0.0 {
                continue;
            }
            let cos_out = f.cos_theta_out(p);
            let denom = cos_out - f.cos_theta;
            if denom.abs() < 1e-6 {
                continue;
            }
            let center = Vector2::new(0.0, f.c() * f.sin_theta / denom);
            let radial = (p - center).normalize();
            let cross = g.x * radial.y - g.y * radial.x;
            assert!(
                cross.abs() <= 1e-6 * g.norm(),
                "gradient {g:?} not radial about {center:?} at {p:?}"
            );
        }
    }

    /// The measured inversion that motivated the module: on one resonance the
    /// keyhole is narrowest at the near (steep) end and widest at the far end.
    #[test]
    fn keyholes_are_narrow_near_earth_and_wide_far_out() {
        let mut seed = 99;
        let mut compared = 0;
        for _ in 0..100 {
            let (f, _) = random_frame(&mut seed);
            for circle in f.resonant_circles(1..=20, 24, 100.0 * f.capture_radius) {
                if circle.encloses_origin() || circle.radius < 1.0e-3 * f.capture_radius {
                    continue;
                }
                let near = f.keyhole_at(&circle, circle.nearest_point());
                let far = f.keyhole_at(&circle, circle.farthest_point());
                assert!(near.width > 0.0 && far.width > 0.0);
                assert!(
                    near.width <= far.width * (1.0 + 1e-9),
                    "{}: near {:.3e} m wider than far {:.3e} m",
                    circle.resonance,
                    near.width,
                    far.width
                );
                assert!(
                    (near.width * near.gradient.norm() - near.semi_major_axis_tolerance).abs()
                        <= 1e-9 * near.semi_major_axis_tolerance
                );
                compared += 1;
            }
        }
        assert!(compared > 50);
    }

    #[test]
    fn distance_to_a_circle_is_signed_and_its_closest_point_is_on_the_circle() {
        let mut seed = 4242;
        let mut checked = 0;
        for _ in 0..40 {
            let (f, _) = random_frame(&mut seed);
            for circle in f.resonant_circles(1..=20, 24, 100.0 * f.capture_radius) {
                if circle.radius < 1.0e-3 * f.capture_radius {
                    continue;
                }
                let centre = Vector2::new(0.0, circle.center_zeta);
                // A point on the circle: distance 0, and its own closest point.
                let angle = std::f64::consts::TAU * lcg(&mut seed);
                let on = circle.point(angle);
                assert!(
                    circle.signed_distance(on).abs() <= 1e-6 * circle.radius,
                    "{}: a point of the circle is {:.3e} m off it",
                    circle.resonance,
                    circle.signed_distance(on)
                );
                assert!((circle.closest_point_to(on).unwrap() - on).norm() <= 1e-6 * circle.radius);
                // Pushed out along the radius by d: distance +d. Pulled in: −d.
                let out = centre + (on - centre) * 1.25;
                let din = centre + (on - centre) * 0.75;
                assert!(
                    (circle.signed_distance(out) - 0.25 * circle.radius).abs()
                        <= 1e-6 * circle.radius,
                    "outside must be positive"
                );
                assert!(
                    (circle.signed_distance(din) + 0.25 * circle.radius).abs()
                        <= 1e-6 * circle.radius,
                    "inside must be negative"
                );
                // The closest point is the same one from either side, and it is
                // the nearest point of the circle to a brute-force sweep.
                for q in [out, din] {
                    let cp = circle.closest_point_to(q).unwrap();
                    assert!((cp - on).norm() <= 1e-6 * circle.radius);
                    let mut best = f64::INFINITY;
                    for i in 0..2000 {
                        let a = std::f64::consts::TAU * (i as f64) / 2000.0;
                        best = best.min((circle.point(a) - q).norm());
                    }
                    assert!(
                        (cp - q).norm() <= best + 1e-3 * circle.radius,
                        "{}: analytic {:.6e} beaten by sweep {:.6e}",
                        circle.resonance,
                        (cp - q).norm(),
                        best
                    );
                }
                // The centre itself: no nearest point, but a defined distance.
                assert!(circle.closest_point_to(centre).is_none());
                assert!((circle.signed_distance(centre) + circle.radius).abs() < 1e-9);
                checked += 1;
            }
        }
        assert!(checked > 50, "only {checked} circles exercised");
    }

    /// The readout the planner prints: how far a b-point is from the nearest
    /// resonant return, and how that ranks against the keyhole's own width.
    /// Ranked two ways on purpose — the nearest circle in kilometres is not
    /// always the one the point is most likely to be *in*.
    #[test]
    fn the_nearest_keyhole_is_the_nearest_circle_and_widths_rank_differently() {
        let mut seed = 20260905;
        let mut saw_disagreement = false;
        let mut margin_disagreements = 0usize;
        let mut queries = 0usize;
        // How much better the smallest-margin circle's margin is than the nearest
        // circle's, in capture radii — 0 whenever the two are the same circle.
        let mut worst_gap: f64 = 0.0;
        // And the number that says whether "they never disagree" is structure or
        // luck: how close the runner-up ever came to beating the nearest circle's
        // margin, in capture radii. A swap is exactly this going negative.
        let mut closest_call = f64::INFINITY;
        // The scale to read that against: the widest half-width anywhere, same
        // units. A door far narrower than the gaps between circles cannot reorder
        // them, which is the structural reason to expect no disagreement at all.
        let mut widest_door: f64 = 0.0;
        // 2 000 geometries, not the 60 this swept until 2026-09-07. The count is
        // load-bearing for what the printed line below is allowed to mean: at 60 a
        // rare disagreement is indistinguishable from none, and the whole reason
        // the line is printed rather than asserted is that "never" is a claim about
        // the census, not about the sample size. Costs ~0.4 s.
        for _ in 0..2000 {
            let (f, _) = random_frame(&mut seed);
            let circles = f.resonant_circles(1..=20, 24, 60.0 * f.capture_radius);
            if circles.len() < 2 {
                continue;
            }
            // A b-point somewhere in the mapped region — log-uniform in radius
            // across [0.5, 40] capture radii, because the census runs out to 60 and
            // sampling a box of ±3 (which this did until 2026-09-07) asks only about
            // the crowded near field. Keyhole widths grow with distance, so which
            // circle wins a ranking is a question about the far field.
            let r_q = f.capture_radius * 0.5 * (80.0f64).powf(lcg(&mut seed));
            let th = std::f64::consts::TAU * lcg(&mut seed);
            let p = Vector2::new(r_q * th.cos(), r_q * th.sin());
            let all = f.keyhole_proximities(&circles, p);
            assert!(all.iter().all(|k| k.keyhole.width.is_finite()));
            let near = f.nearest_keyhole(&circles, p).expect("a nearest circle");
            let tight = f.tightest_keyhole(&circles, p).expect("a tightest keyhole");
            let risk = f
                .smallest_margin_keyhole(&circles, p)
                .expect("a smallest-margin circle");
            // Each selector really is the minimum of its own key.
            for k in &all {
                assert!(near.signed_distance.abs() <= k.signed_distance.abs() * (1.0 + 1e-12));
                assert!(tight.widths_away() <= k.widths_away() * (1.0 + 1e-12));
                // Margins go negative inside a door, so the tolerance has to be
                // additive on the scale of the circles, not a relative slack on a
                // quantity whose sign changes.
                assert!(risk.margin() <= k.margin() + 1e-6 * f.capture_radius);
            }
            // `margin` is the definition the frontend cuts on, spelled out once
            // more here so a change to it has to be made in two places.
            for k in &all {
                assert!(
                    (k.margin() - (k.signed_distance.abs() - 0.5 * k.keyhole.width)).abs()
                        <= 1e-9 * f.capture_radius
                );
                assert!(k.inside() == (k.margin() <= 0.0));
            }
            // The proximity is self-consistent: the closest point is on its
            // circle, at exactly |signed_distance| from the query.
            for k in &all {
                assert!(k.circle.signed_distance(k.closest_point).abs() <= 1e-6 * k.circle.radius);
                assert!(
                    ((k.closest_point - p).norm() - k.signed_distance.abs()).abs()
                        <= 1e-6 * k.circle.radius.max(p.norm())
                );
                assert_eq!(k.keyhole.at, k.closest_point);
                assert!(k.inside() == (k.widths_away() <= 1.0));
            }
            if near.circle.resonance != tight.circle.resonance {
                saw_disagreement = true;
            }
            queries += 1;
            for k in &all {
                widest_door = widest_door.max(k.half_width() / f.capture_radius);
                if k.circle.resonance != near.circle.resonance {
                    closest_call =
                        closest_call.min((k.margin() - near.margin()) / f.capture_radius);
                }
            }
            if near.circle.resonance != risk.circle.resonance {
                margin_disagreements += 1;
                worst_gap = worst_gap.max((near.margin() - risk.margin()) / f.capture_radius);
            }
        }
        assert!(
            saw_disagreement,
            "kilometres and keyhole widths never disagreed — then one selector is redundant"
        );
        // Reported, not asserted. Whether the nearest circle is also the one whose
        // door the point is nearest is a fact about how the widths grow across a
        // census, and it is allowed to come out "always" — `smallest_margin_keyhole`
        // is the principled key either way, being the one the placement band is
        // additive in. What would not be honest is claiming a disagreement without
        // having watched for one.
        println!(
            "margin vs kilometres: {margin_disagreements} of {queries} queries named a \
             different circle; the runner-up came within {closest_call:.3e} capture \
             radii of winning (worst actual gain {worst_gap:.3}), against a widest \
             door anywhere of {widest_door:.3e}"
        );
    }

    #[test]
    fn census_is_coprime_and_sorted() {
        let all = Resonance::census(1..=6, 8);
        assert!(all.iter().all(|r| gcd(r.h, r.k) == 1));
        assert!(all
            .windows(2)
            .all(|w| w[0].semi_major_axis_m() <= w[1].semi_major_axis_m()));
        assert!(all.contains(&Resonance { h: 3, k: 4 }));
        assert!(!all.contains(&Resonance { h: 2, k: 4 }));
        let r = Resonance { h: 3, k: 4 };
        assert!((r.semi_major_axis_m() / AU_M - 0.825_481_812).abs() < 1e-8);
        assert!((r.period_seconds() / JULIAN_YEAR_S - 0.75).abs() < 1e-15);
        assert_eq!(format!("{r}"), "3:4");
    }

    /// An unreachable resonance is `None`, not a circle with a NaN in it.
    #[test]
    fn out_of_reach_resonances_are_none() {
        let mut seed = 3;
        let (f, _) = random_frame(&mut seed);
        // 1:100 is a 0.046 AU orbit no Earth flyby can produce.
        assert!(f.resonant_circle(Resonance { h: 1, k: 100 }).is_none());
        assert!(f.cos_theta_out_for_semi_major_axis(-1.0).is_none());
        assert!(f.cos_theta_out_for_semi_major_axis(f64::NAN).is_none());
    }

    /// Kernel-gated: the shipping rock's frame is validated end to end — the
    /// un-rotated reconstruction reproduces its real pre-encounter orbit at the
    /// `r ≈ R⊕ₒᵣᵦ` ceiling, the 3:4 resonance the reach probe picked is present
    /// with the locus and keyhole it reported, and the rotated branch's sign is
    /// consistent with the flown measurement (`−B̂` raises `a'` on this rock).
    #[test]
    fn the_shipping_rock_reaches_the_three_four_resonance_where_the_probe_said() {
        use crate::scenario::{ImpactorConfig, RealFieldScenario};
        use anise::constants::frames::{EARTH_J2000, SSB_J2000, SUN_J2000};

        let Some(_) = crate::kernels::resolve_for_test("the shipping-rock keyhole census") else {
            return;
        };
        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("build");
        let eph = sc.ephemeris().clone();
        let ds = sc.deflection().expect("deflection");
        let enc = sc.nominal_hit(&ds).expect("nominal hit");
        let t_ca = ds
            .nominal_encounter_epoch()
            .expect("epoch")
            .expect("an encounter");
        let (r_km, v_km) = eph
            .state_km_s(EARTH_J2000, SUN_J2000, t_ca.as_hifitime())
            .expect("Earth heliocentric state");
        let mu_sun = eph.sun_gm_m3_s2().expect("sun GM");
        let f = OpikFrame::new(&enc, r_km * 1e3, v_km * 1e3, mu_sun).expect("frame");

        // Round trip against the real pre-encounter heliocentric orbit.
        let t_pre = t_ca.shifted_by_seconds(-30.0 * 86_400.0);
        let pre = ds.nominal().state_at(t_pre).expect("pre state");
        let (rs_km, vs_km) = eph
            .state_km_s(SUN_J2000, SSB_J2000, t_pre.as_hifitime())
            .expect("Sun state");
        let r_h = pre.position - rs_km * 1e3;
        let v_h = pre.velocity - vs_km * 1e3;
        let a_actual = 1.0 / (2.0 / r_h.norm() - v_h.norm_squared() / mu_sun);
        let rel = (f.incoming_semi_major_axis() - a_actual).abs() / a_actual;
        assert!(
            rel < 1.0e-3,
            "round trip {rel:.3e}: a frame, sign or vis-viva bug, not the approximation"
        );

        // The 3:4 resonance. The reach probe reported its locus at b = 60 843–
        // 153 511 km from a 72-direction sweep that bisected *outward from the
        // capture radius* — so a ray that crossed the circle twice, or entered it
        // inside the disc, was invisible to it. The exact circle says the far end
        // was right and the near end was not: the locus reaches b = 3 866 km, well
        // inside the 11 311 km disc. Part of the 3:4 resonance is an impact, and
        // the nearest miss on it is the grazing point.
        let circle = f
            .resonant_circle(Resonance { h: 3, k: 4 })
            .expect("3:4 in reach");
        let (lo, hi) = circle.b_range();
        println!(
            "3:4 circle: ζ_c {:.0} km, R {:.0} km, b {:.0}..{:.0} km, capture {:.0} km",
            circle.center_zeta / 1e3,
            circle.radius / 1e3,
            lo / 1e3,
            hi / 1e3,
            f.capture_radius / 1e3
        );
        assert!(
            (3.0e6..=5.0e6).contains(&lo) && (150.0e6..=158.0e6).contains(&hi),
            "3:4 locus b = {:.0}..{:.0} km",
            lo / 1e3,
            hi / 1e3
        );
        assert!(circle.crosses_capture_disc(f.capture_radius));
        assert!(!circle.encloses_origin());
        let (graze, graze_mirror) = circle
            .intersections_at_radius(f.capture_radius)
            .expect("the circle crosses the disc edge");
        assert!((graze.norm() - f.capture_radius).abs() < 1e-3);
        assert!((graze_mirror.x + graze.x).abs() < 1e-6 && graze_mirror.y == graze.y);
        let at_graze = f.keyhole_at(&circle, graze);
        let far = f.keyhole_at(&circle, circle.farthest_point());
        println!(
            "3:4 keyhole width: {:.2} km at grazing, {:.2} km at the far end",
            at_graze.width / 1e3,
            far.width / 1e3
        );
        // The far end matches the probe's 24.9 km; the grazing point is steeper
        // than anything the probe reached and correspondingly narrower.
        assert!(
            at_graze.width < 12.0e3 && (18.0e3..=30.0e3).contains(&far.width),
            "3:4 keyhole {:.1} km at grazing, {:.1} km far (probe far end: 24.9)",
            at_graze.width / 1e3,
            far.width / 1e3
        );

        // The sign, structurally. Bending toward Earth is bending toward −ζ̂,
        // i.e. toward Earth's own motion, which adds heliocentric speed: so +ζ
        // *raises* a' and −ζ lowers it, for any encounter. The 3:4 resonance is
        // below the incoming 0.854 AU, so its circle must sit on the −ζ side —
        // which is what probe_keyhole_rotation found the hard way: its along-track
        // nudge landed on the raising side (0.854 → 0.889 AU), and reaching 3:4
        // needs the opposite sense of deflection, not a larger one.
        let a_in = f.incoming_semi_major_axis();
        let up = f.post_encounter_semi_major_axis(Vector2::new(0.0, 21.0 * R_EARTH));
        let down = f.post_encounter_semi_major_axis(Vector2::new(0.0, -21.0 * R_EARTH));
        assert!(
            up > a_in && down < a_in,
            "+ζ must raise a': {up} / {a_in} / {down}"
        );
        assert!(circle.a_prime < a_in);
        assert!(
            circle.center_zeta < 0.0,
            "3:4 must lie on the lowering side"
        );
        println!(
            "nominal b-point in the Öpik frame: ξ {:.0} km, ζ {:.0} km",
            f.project(&enc.b_vector).x / 1e3,
            f.project(&enc.b_vector).y / 1e3
        );
    }
    /// Kernel-gated, ~15 s. **The 3:4 keyhole, flown.** `probe_keyhole_return`
    /// aimed the shipping rock at the 3:4 circle with the closed form (retrograde
    /// along-track Δv 0.216438 m/s at the campaign start, return 53 841 km) and
    /// refined the Δv against the propagator to the return's floor:
    /// **0.216550 m/s → 1 130 km from Earth's centre on 2042-12-31**, three years
    /// after the 2040-01-01 flyby, inside the capture disc and inside Earth.
    ///
    /// That Δv is the **12-iteration** stop, rounded to six decimals — a real
    /// flight, but 1.6e-6 m/s short of the actual minimum (`0.2165483096`
    /// → 1 087 km, measured 2026-09-06 at 20 iterations). It is kept here on
    /// purpose: this test's job is that *a* flown Δv still returns inside Earth,
    /// and re-pinning it to the sharper floor would cost a re-fly for no claim.
    /// Never round it further — `ζ₂` moves 5.4e8 km per m/s. See
    /// [`keyhole_target`](crate::keyhole_target). This
    /// re-flies that one Δv — no solve, one deflected 12-year arc plus 3.6 years
    /// onward — and asserts the return is where the probe measured it, so the
    /// flown keyhole is a regression test rather than a session note.
    ///
    /// The tolerance is a few capture radii, deliberately loose: the return miss
    /// is a V-shaped function of Δv whose bottom the search reaches to ~1e-5 m/s
    /// at the default iteration budget (that is the budget, **not** a flat floor —
    /// see [`KeyholeSolution::dv_window_m_s`](crate::keyhole_target::KeyholeSolution::dv_window_m_s)),
    /// and an integrator
    /// or cadence change that moves the floor by that much still lands a return
    /// within thousands of kilometres, which is the claim. What would fail this
    /// is a frame or sign regression, which moves the return by ~1e6 km.
    #[test]
    fn the_three_four_keyhole_returns_the_rock_to_earth_when_flown() {
        use crate::close_approach::{closest_approach, find_close_approaches, ScanOptions};
        use crate::perturber_field::EphemerisPerturber;
        use crate::scenario::{ImpactorConfig, RealFieldScenario};
        use anise::constants::frames::EARTH_J2000;

        let Some(_) = crate::kernels::resolve_for_test("the flown 3:4 keyhole") else {
            return;
        };
        const KEYHOLE_DV_RETROGRADE_M_S: f64 = 0.216_550;
        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("build");
        let earth = EphemerisPerturber::new(sc.ephemeris().clone(), EARTH_J2000);
        let ds = sc.deflection().expect("deflection");
        let nominal = sc.nominal_hit(&ds).expect("nominal hit");
        let epoch0 = sc.epoch0();
        let seed = ds.nominal().state_at(epoch0).expect("seed");
        let retro = -crate::deflection::along_track_unit(seed).expect("along-track");
        let (clock, _) = ds
            .deflected_trajectory(epoch0, KEYHOLE_DV_RETROGRADE_M_S * retro)
            .expect("deflected trajectory");
        let scan = ScanOptions {
            max_sample_dt: 6.0 * 3600.0,
            time_tol_seconds: 1.0e-3,
            max_distance: Some(5.0e8),
        };
        let ca1 = closest_approach(&clock, &earth, scan)
            .expect("scan")
            .expect("encounter 1 inside the gate");
        let enc1 = ca1
            .b_plane(nominal.mu, nominal.earth_radius)
            .expect("reduce");
        assert!(!enc1.is_hit(), "encounter 1 must be a miss");
        assert!(
            (150.0e6..=157.0e6).contains(&enc1.impact_parameter),
            "encounter-1 b {:.0} km is off the 3:4 far end",
            enc1.impact_parameter / 1e3
        );
        let t_hand = ca1.epoch.shifted_by_seconds(30.0 * 86_400.0);
        let hand = clock.state_at(t_hand).expect("hand-off state");
        let onward = sc
            .propagate_free(t_hand, hand, 86_400.0, (3.6 * 365.25) as u32)
            .expect("fly on");
        let returns = find_close_approaches(
            &onward,
            &earth,
            ScanOptions {
                max_sample_dt: 6.0 * 3600.0,
                time_tol_seconds: 1.0e-3,
                max_distance: Some(0.05 * AU_M),
            },
        )
        .expect("return census");
        let best = returns
            .iter()
            .min_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap())
            .expect("a return inside 0.05 AU");
        let years_after = (best.epoch.tdb_seconds_past_j2000()
            - ca1.epoch.tdb_seconds_past_j2000())
            / JULIAN_YEAR_S;
        println!(
            "3:4 return: {:.0} km at {} ({:.3} yr after encounter 1)",
            best.distance / 1e3,
            best.epoch.as_hifitime(),
            years_after
        );
        assert!(
            (2.9..=3.1).contains(&years_after),
            "the return must come {years_after:.3} ≈ 3 yr later"
        );
        assert!(
            best.distance < 4.0 * nominal.capture_radius,
            "return miss {:.0} km — the flown keyhole has moved",
            best.distance / 1e3
        );
    }
}
