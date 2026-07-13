//! `geometry` — b-plane encounter geometry and the gravitationally-focused
//! capture-radius hit test (HANDOFF §5, §10.8).
//!
//! This is where the heliocentric arc becomes a **hit-or-miss answer**. Hit-vs-miss
//! is *not* decided by the Sun-centred orbit; it is decided by Earth's gravity
//! during the close approach, and it is acutely sensitive to initial conditions
//! (HANDOFF §5). This module reduces one close approach to the two invariants that
//! settle it: the **b-plane impact parameter** `b` (the perpendicular miss of the
//! incoming asymptote from Earth's centre) and the **gravitationally-focused
//! capture radius** `b_capture` — Earth's gravity enlarges its own target, so the
//! collision cross-section is a disc of radius `b_capture > R⊕`, not `R⊕`.
//!
//! ```text
//!   b_capture = R⊕ · √(1 + (v_esc / v_inf)²)          v_esc = √(2μ⊕ / R⊕)
//!   hit  ⇔  b ≤ b_capture   ⇔   r_perigee ≤ R⊕
//! ```
//!
//! The factor `√(1 + (v_esc/v_inf)²)` is ~1.2–2.4 for typical NEO `v_inf`
//! (HANDOFF §5) — the pedagogical payload of the whole encounter.
//!
//! # Frame-agnostic, but the input must be **Earth-relative and near-encounter**
//! [`BPlaneEncounter::from_relative_state`] takes the asteroid's state **relative
//! to Earth's geocentre** (position m, velocity m/s), the two vectors in a common
//! inertial frame (the core's barycentric ICRF is fine — only the *relative* state
//! enters, and the b-plane basis is built from it). Everything computed here is an
//! osculating **two-body-about-Earth** invariant, so mathematically it does not
//! matter *where* on the hyperbola the state is sampled.
//!
//! It matters **physically**, though: these quantities only describe a real
//! encounter when Earth actually dominates the dynamics — i.e. the state must be
//! sampled **inside Earth's sphere of influence / Hill sphere, near closest
//! approach** (Hill radius ≈ 1.5e9 m ≈ 0.01 AU; SOI ≈ 9.2e8 m). Feed a state from
//! 0.5 AU out and you get a perfectly self-consistent "Earth hyperbola" that is
//! meaningless — the body is really on a heliocentric arc there. Producing a
//! valid near-encounter relative state (find closest approach on the propagated
//! trajectory, difference against the reconstructed geocentre) is the **caller's**
//! job and belongs to the clock / close-approach detector (§10.9), which samples
//! the trajectory densely enough to bracket the minimum. This module deliberately
//! does *not* search for closest approach.
//!
//! ## Step-9 prerequisite (noted here so it is not a surprise)
//! Forming `v_rel` needs Earth's **velocity**, and [`crate::ephemeris::Ephemeris`]
//! currently exposes only `position_km`. ANISE's `translate` already returns
//! `velocity_km_s` alongside the radius — it is simply discarded today; surfacing
//! it is a small add when the close-approach detector lands (§10.9).
//!
//! # Scope (§10.8)
//! The hit test and the scalar b-plane geometry (`v_inf`, `b`, perigee, capture
//! radius, the incoming-asymptote direction `Ŝ`, and the b-vector `B`). The full
//! Öpik / Kizner **ξ,ζ decomposition** of `B` — which needs an external reference
//! direction (Earth's heliocentric velocity, or an ecliptic pole) and is what
//! keyhole/covariance work reasons in — is deferred to Tier 3 (`uncertainty.rs`).
//! `B` is provided here as a 3-vector with pinned invariants (`|B| = b`, `B ⊥ Ŝ`,
//! `B ⊥ ĥ`); its *sign convention* is left to nail down when keyholes need it.

use nalgebra::Vector3;

/// Earth's equatorial radius (WGS-84), metres — the larger, conservative choice
/// for the solid-body target radius `R⊕`. The ~100 km atmosphere is cosmetic next
/// to gravitational focusing (HANDOFF §5); pick whichever radius the scenario
/// wants and pass it explicitly — the geometry does not assume one.
pub const EARTH_EQUATORIAL_RADIUS_M: f64 = 6_378_137.0;

/// Earth's mean (volumetric) radius, metres — the alternative `R⊕` when a single
/// spherical radius is wanted rather than the equatorial bulge.
pub const EARTH_MEAN_RADIUS_M: f64 = 6_371_000.0;

/// Why a b-plane encounter could not be built from a relative state.
#[derive(Debug, Clone, PartialEq)]
pub enum GeometryError {
    /// The relative state is bound or parabolic about Earth (specific orbital
    /// energy `ε ≤ 0`), so there is no hyperbolic flyby and no `v_inf`. A genuine
    /// NEO encounter is always hyperbolic about Earth; a non-positive energy means
    /// the state was sampled somewhere it does not describe an Earth encounter
    /// (see the module note on sampling inside the SOI).
    NotHyperbolic {
        /// The offending specific orbital energy `ε = v²/2 − μ/r` (m²/s²), `≤ 0`.
        specific_energy: f64,
    },
    /// The state is degenerate for b-plane geometry: zero radius, or (near-)zero
    /// specific angular momentum (a radial fall, `r ∥ v`) for which the b-plane
    /// basis and the b-vector are undefined. Rejected rather than returning a
    /// `NaN` geometry.
    Degenerate,
    /// `μ` or `R⊕` was not a finite positive number.
    NonPositiveParameter,
}

impl std::fmt::Display for GeometryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GeometryError::NotHyperbolic { specific_energy } => write!(
                f,
                "relative state is not hyperbolic about Earth (ε = {specific_energy:.6e} m²/s² ≤ 0)"
            ),
            GeometryError::Degenerate => write!(
                f,
                "degenerate encounter state (zero radius or radial motion: r ∥ v)"
            ),
            GeometryError::NonPositiveParameter => {
                write!(f, "μ and R⊕ must both be finite and positive")
            }
        }
    }
}

impl std::error::Error for GeometryError {}

/// The b-plane geometry of one close approach, as an osculating two-body hyperbola
/// about Earth. Build it with [`BPlaneEncounter::from_relative_state`].
///
/// All fields are in SI (metres, m/s, m³/s²) and expressed in the same inertial
/// frame as the input relative state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BPlaneEncounter {
    /// Hyperbolic excess speed `v_inf = √(2ε)`, m/s — the approach speed "at
    /// infinity" that sets the strength of gravitational focusing.
    pub v_inf: f64,
    /// b-plane impact parameter `b = h / v_inf`, metres — the perpendicular
    /// distance of the incoming asymptote from Earth's centre. This is the
    /// **miss distance** the hit test compares against the capture radius.
    pub impact_parameter: f64,
    /// Perigee distance `r_p = p/(1+e)` of the osculating hyperbola, metres — the
    /// closest the asteroid's centre would pass Earth's centre under Earth's
    /// gravity alone. `r_p ≤ R⊕` is the equivalent-and-cross-checked hit criterion.
    pub perigee: f64,
    /// Gravitationally-focused capture radius `b_capture`, metres — the radius of
    /// Earth's effective collision disc in the b-plane (HANDOFF §5).
    pub capture_radius: f64,
    /// Orbital eccentricity `e > 1` of the flyby hyperbola.
    pub eccentricity: f64,
    /// The solid-body target radius `R⊕` (m) the capture radius was built from.
    pub earth_radius: f64,
    /// Earth's gravitational parameter `μ⊕` (m³/s²) used.
    pub mu: f64,
    /// Incoming-asymptote direction `Ŝ` (unit) — the direction of `v_inf`, i.e.
    /// the way the body travels far *before* the encounter. The b-plane is the
    /// plane through Earth's centre perpendicular to `Ŝ`.
    pub s_hat: Vector3<f64>,
    /// The b-vector `B` (metres): magnitude equals [`impact_parameter`], lies in
    /// the b-plane (`B·Ŝ = 0`) and in the orbital plane (`B·ĥ = 0`), per
    /// `B = b·(Ŝ × ĥ)`. Sign convention unpinned pending keyhole work (§10.8 doc).
    ///
    /// [`impact_parameter`]: BPlaneEncounter::impact_parameter
    pub b_vector: Vector3<f64>,
}

impl BPlaneEncounter {
    /// Reduce an Earth-relative encounter state to its b-plane geometry.
    ///
    /// `r_rel`/`v_rel` are the asteroid's position (m) and velocity (m/s)
    /// **relative to Earth's geocentre**, sampled **near closest approach inside
    /// Earth's SOI** (see the module note — the caller owns that). `mu` is Earth's
    /// `μ⊕` (m³/s², pull it through ANISE) and `earth_radius` is the target `R⊕`
    /// (e.g. [`EARTH_EQUATORIAL_RADIUS_M`]).
    ///
    /// Returns [`GeometryError::NotHyperbolic`] if the state is bound/parabolic
    /// about Earth, [`GeometryError::Degenerate`] for a zero-radius or radial
    /// (`r ∥ v`) state, and [`GeometryError::NonPositiveParameter`] for a
    /// non-finite/non-positive `μ` or `R⊕`.
    ///
    /// Numerical note: `v_inf = √(v² − 2μ/r)` is a difference of comparable
    /// squares deep in Earth's well, so it loses significant digits if the state
    /// is sampled almost exactly at the perigee of a very fast pass. Sampling at
    /// SOI-scale range (where `v ≈ v_inf`) keeps it well-conditioned — another
    /// reason the caller samples near-but-not-at closest approach.
    pub fn from_relative_state(
        r_rel: Vector3<f64>,
        v_rel: Vector3<f64>,
        mu: f64,
        earth_radius: f64,
    ) -> Result<Self, GeometryError> {
        if !(mu.is_finite() && mu > 0.0 && earth_radius.is_finite() && earth_radius > 0.0) {
            return Err(GeometryError::NonPositiveParameter);
        }

        let r = r_rel.norm();
        if r == 0.0 {
            return Err(GeometryError::Degenerate);
        }
        let v2 = v_rel.norm_squared();

        // Specific orbital energy about Earth; ε > 0 ⇔ hyperbolic flyby.
        let energy = 0.5 * v2 - mu / r;
        // Reject ε ≤ 0 (bound/parabolic) and any non-finite ε (a NaN from a bad
        // input state) — the `is_finite && > 0` form also keeps clippy's
        // partial-ord lint happy versus a bare `!(energy > 0.0)`.
        if !(energy.is_finite() && energy > 0.0) {
            return Err(GeometryError::NotHyperbolic {
                specific_energy: energy,
            });
        }
        let v_inf = (2.0 * energy).sqrt();

        // Specific angular momentum. Zero ⇒ radial fall ⇒ no b-plane basis.
        let h_vec = r_rel.cross(&v_rel);
        let h = h_vec.norm();
        if h == 0.0 {
            return Err(GeometryError::Degenerate);
        }
        let h_hat = h_vec / h;

        // Eccentricity vector (points to periapsis); e > 1 for ε > 0.
        let e_vec = (r_rel * (v2 - mu / r) - v_rel * r_rel.dot(&v_rel)) / mu;
        let ecc = e_vec.norm();
        let p_hat = e_vec / ecc; // periapsis direction P̂
        let q_hat = h_hat.cross(&p_hat); // Q̂, 90° ahead of periapsis in-plane

        // b-plane impact parameter and hyperbola perigee (p = h²/μ, r_p = p/(1+e)).
        let impact_parameter = h / v_inf;
        let perigee = (h * h / mu) / (1.0 + ecc);

        // Gravitationally-focused capture radius:
        //   b_capture² = R⊕²·(1 + (v_esc/v_inf)²) = R⊕² + 2μR⊕/v_inf².
        let capture_radius =
            (earth_radius * earth_radius + 2.0 * mu * earth_radius / (v_inf * v_inf)).sqrt();

        // Incoming-asymptote direction Ŝ = (P̂ + √(e²−1)·Q̂)/e (perifocal velocity
        // direction at the incoming asymptote ν = −ν_∞, cos ν_∞ = −1/e). The
        // `.max(0.0)` guards a round-off-negative (e²−1) at the parabolic edge.
        let sqrt_e2m1 = (ecc * ecc - 1.0).max(0.0).sqrt();
        let s_hat = (p_hat + sqrt_e2m1 * q_hat) / ecc;

        // b-vector: |B| = b, B ⊥ Ŝ and B ⊥ ĥ. Ŝ × ĥ = (√(e²−1)P̂ − Q̂)/e is a
        // unit in-plane vector perpendicular to Ŝ, so B = b·(Ŝ × ĥ).
        let b_vector = impact_parameter * s_hat.cross(&h_hat);

        Ok(Self {
            v_inf,
            impact_parameter,
            perigee,
            capture_radius,
            eccentricity: ecc,
            earth_radius,
            mu,
            s_hat,
            b_vector,
        })
    }

    /// The hit verdict: `true` when the b-plane miss is within the focused capture
    /// radius (`b ≤ b_capture`). Equivalent by construction to `r_perigee ≤ R⊕`
    /// (the tests pin that equivalence).
    pub fn is_hit(&self) -> bool {
        self.impact_parameter <= self.capture_radius
    }

    /// Gravitational focusing factor `b_capture / R⊕ = √(1 + (v_esc/v_inf)²)` (≥ 1)
    /// — how much larger than its solid disc Earth's gravity makes the target.
    pub fn focusing_factor(&self) -> f64 {
        self.capture_radius / self.earth_radius
    }

    /// Earth's surface escape speed `v_esc = √(2μ⊕/R⊕)`, m/s.
    pub fn escape_speed(&self) -> f64 {
        (2.0 * self.mu / self.earth_radius).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Earth GM in SI (m³/s²), DE440-consistent — a fixed literal for these
    /// kernel-free geometry tests (the shipping code pulls μ through ANISE).
    const MU_EARTH: f64 = 3.986_004_356e14;
    const R_EARTH: f64 = EARTH_EQUATORIAL_RADIUS_M;

    /// Build the relative state at perigee of a hyperbola with the given `v_inf`
    /// and perigee distance `r_p`: position `r_p·x̂`, velocity perpendicular on
    /// `+ŷ` with the vis-viva perigee speed `v_p = √(v_inf² + 2μ/r_p)`.
    fn state_at_perigee(v_inf: f64, r_p: f64) -> (Vector3<f64>, Vector3<f64>) {
        let v_p = (v_inf * v_inf + 2.0 * MU_EARTH / r_p).sqrt();
        (Vector3::new(r_p, 0.0, 0.0), Vector3::new(0.0, v_p, 0.0))
    }

    /// The discriminating round-trip: from a state built at a *known* `v_inf` and
    /// `r_p`, the recovered `v_inf`, perigee, impact parameter, and eccentricity
    /// must match the closed-form values. This pins the whole chain (energy → h →
    /// e → b) and would catch a wrong-but-self-consistent formula that the
    /// invariant checks alone miss.
    #[test]
    fn perigee_state_round_trips_to_known_geometry() {
        let v_inf = 8_000.0; // 8 km/s — a brisk NEO flyby
        let r_p = 2.0 * R_EARTH; // clean miss, perigee at 2 R⊕
        let (r_rel, v_rel) = state_at_perigee(v_inf, r_p);
        let v_p = v_rel.norm();

        let enc = BPlaneEncounter::from_relative_state(r_rel, v_rel, MU_EARTH, R_EARTH).unwrap();

        assert!(
            (enc.v_inf - v_inf).abs() / v_inf < 1e-12,
            "v_inf {}",
            enc.v_inf
        );
        assert!(
            (enc.perigee - r_p).abs() / r_p < 1e-12,
            "r_p {}",
            enc.perigee
        );
        // b = h/v_inf = r_p·v_p/v_inf (angular momentum at perigee is r_p·v_p).
        let b_expected = r_p * v_p / v_inf;
        assert!(
            (enc.impact_parameter - b_expected).abs() / b_expected < 1e-12,
            "b {} vs {}",
            enc.impact_parameter,
            b_expected
        );
        // e = 1 + r_p·v_inf²/μ for a hyperbola specified by (r_p, v_inf).
        let e_expected = 1.0 + r_p * v_inf * v_inf / MU_EARTH;
        assert!(
            (enc.eccentricity - e_expected).abs() / e_expected < 1e-12,
            "e {} vs {}",
            enc.eccentricity,
            e_expected
        );
    }

    /// Sampling-point invariance — the module's central documented claim ("it does
    /// not matter *where* on the hyperbola the state is sampled"). Build the *same*
    /// hyperbola (perigee on +x̂, motion +ŷ) at an **inbound off-perigee** true
    /// anomaly (`r·v ≠ 0`), and assert every recovered quantity — `v_inf`, `b`,
    /// `perigee`, `eccentricity`, **and the asymptote direction `Ŝ`** — matches the
    /// perigee-sampled encounter. This is the test that actually exercises the
    /// `−v_rel·(r·v)` branch of the eccentricity vector and validates `Ŝ`'s
    /// *direction* (the perigee tests leave both unpinned: at perigee `r·v = 0`, and
    /// `|B| = b` / `B ⊥ Ŝ` hold by construction regardless of where `Ŝ` points).
    #[test]
    fn geometry_is_invariant_to_the_sampling_point_on_the_hyperbola() {
        let v_inf = 8_000.0;
        let r_p = 2.0 * R_EARTH;
        let (rp_r, rp_v) = state_at_perigee(v_inf, r_p);
        let reference =
            BPlaneEncounter::from_relative_state(rp_r, rp_v, MU_EARTH, R_EARTH).unwrap();

        // Same hyperbola in the perifocal frame P̂=x̂, Q̂=ŷ (so ĥ=+ẑ), sampled at an
        // inbound true anomaly ν = −0.7 rad (r·v < 0). e and p follow from (r_p, v_inf).
        let e = 1.0 + r_p * v_inf * v_inf / MU_EARTH;
        let p = r_p * (1.0 + e); // r_p = p/(1+e)
        let nu = -0.7_f64;
        let (sin_nu, cos_nu) = nu.sin_cos();
        let r = p / (1.0 + e * cos_nu);
        let pos = Vector3::new(r * cos_nu, r * sin_nu, 0.0);
        let sqrt_mu_p = (MU_EARTH / p).sqrt();
        let vel = Vector3::new(-sin_nu, e + cos_nu, 0.0) * sqrt_mu_p;
        // Sanity: this really is an off-perigee (r·v ≠ 0), inbound (r·v < 0) sample.
        assert!(
            pos.dot(&vel) < -1.0,
            "expected an inbound off-perigee sample"
        );

        let enc = BPlaneEncounter::from_relative_state(pos, vel, MU_EARTH, R_EARTH).unwrap();

        assert!(
            (enc.v_inf - reference.v_inf).abs() / reference.v_inf < 1e-9,
            "v_inf"
        );
        assert!(
            (enc.impact_parameter - reference.impact_parameter).abs() / reference.impact_parameter
                < 1e-9,
            "b"
        );
        assert!(
            (enc.perigee - reference.perigee).abs() / reference.perigee < 1e-9,
            "perigee"
        );
        assert!(
            (enc.eccentricity - reference.eccentricity).abs() / reference.eccentricity < 1e-9,
            "e"
        );
        // The direction-sensitive check: Ŝ must agree with the perigee-sampled Ŝ,
        // not merely be some unit vector. A dropped/flipped e_vec term would send Ŝ
        // elsewhere and blow this up.
        assert!(
            (enc.s_hat - reference.s_hat).norm() < 1e-9,
            "Ŝ {:?} vs reference {:?}",
            enc.s_hat,
            reference.s_hat
        );
    }

    /// The load-bearing equivalence (advisor-flagged): `b ≤ b_capture` is exactly
    /// `r_perigee ≤ R⊕`. Sweep perigee from inside to outside R⊕ and assert the
    /// two criteria never disagree — including a near-grazing case at the boundary.
    #[test]
    fn hit_criterion_matches_perigee_inside_earth() {
        // Straddle R⊕ closely on both sides but avoid the exact boundary — at
        // frac == 1.0 the two criteria are independently-rounded booleans at the
        // grazing point (a round-off coin-flip); `grazing_perigee_…` pins that
        // point properly with a relative tolerance instead.
        let v_inf = 5_000.0;
        for frac in [0.5, 0.9, 0.999, 1.001, 1.1, 3.0] {
            let r_p = frac * R_EARTH;
            let (r_rel, v_rel) = state_at_perigee(v_inf, r_p);
            let enc =
                BPlaneEncounter::from_relative_state(r_rel, v_rel, MU_EARTH, R_EARTH).unwrap();
            let by_perigee = enc.perigee <= R_EARTH;
            assert_eq!(
                enc.is_hit(),
                by_perigee,
                "frac {frac}: is_hit={} but perigee {} vs R⊕ {} (b {} vs b_capture {})",
                enc.is_hit(),
                enc.perigee,
                R_EARTH,
                enc.impact_parameter,
                enc.capture_radius
            );
        }
    }

    /// A grazing pass — perigee exactly at R⊕ — must give `b = b_capture` to
    /// round-off, the exact boundary of the capture disc.
    #[test]
    fn grazing_perigee_gives_impact_parameter_equal_to_capture_radius() {
        let v_inf = 7_000.0;
        let (r_rel, v_rel) = state_at_perigee(v_inf, R_EARTH);
        let enc = BPlaneEncounter::from_relative_state(r_rel, v_rel, MU_EARTH, R_EARTH).unwrap();
        assert!(
            (enc.impact_parameter - enc.capture_radius).abs() / enc.capture_radius < 1e-12,
            "grazing: b {} vs b_capture {}",
            enc.impact_parameter,
            enc.capture_radius
        );
    }

    /// `v_inf = v_esc ⇒ b_capture = R⊕·√2` (focusing factor √2). Choose the
    /// approach speed to equal Earth's surface escape speed and check the disc.
    #[test]
    fn capture_radius_at_v_inf_equal_v_esc_is_r_earth_root_two() {
        let v_esc = (2.0 * MU_EARTH / R_EARTH).sqrt();
        // Any clean-miss perigee; capture radius depends only on v_inf and R⊕.
        let (r_rel, v_rel) = state_at_perigee(v_esc, 5.0 * R_EARTH);
        let enc = BPlaneEncounter::from_relative_state(r_rel, v_rel, MU_EARTH, R_EARTH).unwrap();
        assert!((enc.v_inf - v_esc).abs() / v_esc < 1e-12);
        assert!(
            (enc.focusing_factor() - std::f64::consts::SQRT_2).abs() < 1e-9,
            "focusing factor {} ≠ √2",
            enc.focusing_factor()
        );
        assert!((enc.capture_radius - R_EARTH * std::f64::consts::SQRT_2).abs() / R_EARTH < 1e-9);
    }

    /// The straight-line / weak-gravity limit: as `μ → 0` the capture radius
    /// collapses to `R⊕` and the impact parameter collapses to the geometric
    /// perpendicular miss of the straight-line path `b = |r × v|/|v|`. Confirms
    /// the focusing is genuinely a gravity effect and vanishes without it.
    #[test]
    fn weak_gravity_limit_recovers_straight_line_geometry() {
        // Off-axis inbound state, evaluated with a tiny μ so gravity barely bends.
        let r_rel = Vector3::new(1.0e9, 3.0e8, 0.0);
        let v_rel = Vector3::new(-9_000.0, 0.0, 0.0);
        let mu_tiny = 1.0e3; // ~11 orders below Earth's μ: effectively field-free
        let enc = BPlaneEncounter::from_relative_state(r_rel, v_rel, mu_tiny, R_EARTH).unwrap();

        let straight_line_miss = r_rel.cross(&v_rel).norm() / v_rel.norm();
        assert!(
            (enc.impact_parameter - straight_line_miss).abs() / straight_line_miss < 1e-6,
            "b {} vs straight-line {}",
            enc.impact_parameter,
            straight_line_miss
        );
        assert!(
            (enc.capture_radius - R_EARTH).abs() / R_EARTH < 1e-6,
            "capture radius {} ≠ R⊕ in the weak-gravity limit",
            enc.capture_radius
        );
        assert!((enc.focusing_factor() - 1.0).abs() < 1e-6);
    }

    /// The b-vector invariants that are actually pinned (§10.8): magnitude equals
    /// the impact parameter, and it is perpendicular to both the asymptote `Ŝ` and
    /// the angular-momentum axis `ĥ`. The *sign* is intentionally not asserted.
    #[test]
    fn b_vector_has_pinned_magnitude_and_is_in_the_b_plane() {
        // A generic 3-D inbound state (out of any coordinate plane).
        let r_rel = Vector3::new(4.0e8, -2.0e8, 1.5e8);
        let v_rel = Vector3::new(-6_000.0, 4_000.0, -1_000.0);
        let enc = BPlaneEncounter::from_relative_state(r_rel, v_rel, MU_EARTH, R_EARTH).unwrap();

        assert!((enc.s_hat.norm() - 1.0).abs() < 1e-12, "Ŝ not unit");
        assert!(
            (enc.b_vector.norm() - enc.impact_parameter).abs() / enc.impact_parameter < 1e-12,
            "|B| {} ≠ b {}",
            enc.b_vector.norm(),
            enc.impact_parameter
        );
        // Perpendicular to the asymptote and to the orbit normal (h = r × v).
        let h_hat = r_rel.cross(&v_rel).normalize();
        assert!(enc.b_vector.dot(&enc.s_hat).abs() / enc.impact_parameter < 1e-12);
        assert!(enc.b_vector.dot(&h_hat).abs() / enc.impact_parameter < 1e-12);
    }

    #[test]
    fn bound_state_is_rejected_as_not_hyperbolic() {
        // A circular-ish bound state about Earth: speed below escape, r ⟂ v.
        let r = 1.0e8;
        let v_circ = (MU_EARTH / r).sqrt(); // bound (ε < 0)
        let r_rel = Vector3::new(r, 0.0, 0.0);
        let v_rel = Vector3::new(0.0, v_circ, 0.0);
        match BPlaneEncounter::from_relative_state(r_rel, v_rel, MU_EARTH, R_EARTH) {
            Err(GeometryError::NotHyperbolic { specific_energy }) => {
                assert!(specific_energy < 0.0)
            }
            other => panic!("expected NotHyperbolic, got {other:?}"),
        }
    }

    #[test]
    fn radial_state_is_rejected_as_degenerate() {
        // r ∥ v ⇒ zero angular momentum ⇒ no b-plane basis. Hyperbolic in energy
        // (fast radial infall) so it passes the energy gate and must be caught by
        // the angular-momentum guard.
        let r_rel = Vector3::new(1.0e8, 0.0, 0.0);
        let v_rel = Vector3::new(20_000.0, 0.0, 0.0); // along r̂, ε > 0
        assert_eq!(
            BPlaneEncounter::from_relative_state(r_rel, v_rel, MU_EARTH, R_EARTH),
            Err(GeometryError::Degenerate)
        );
    }

    #[test]
    fn non_positive_parameters_are_rejected() {
        let r_rel = Vector3::new(1.0e8, 0.0, 0.0);
        let v_rel = Vector3::new(0.0, 20_000.0, 0.0);
        for (mu, radius) in [
            (0.0, R_EARTH),
            (-1.0, R_EARTH),
            (MU_EARTH, 0.0),
            (MU_EARTH, -1.0),
        ] {
            assert_eq!(
                BPlaneEncounter::from_relative_state(r_rel, v_rel, mu, radius),
                Err(GeometryError::NonPositiveParameter)
            );
        }
    }

    /// A concrete focusing sanity check against the HANDOFF §5 range: a typical
    /// NEO `v_inf` (a few km/s) enlarges Earth's target by the quoted ~1.2–2.4×.
    #[test]
    fn focusing_factor_sits_in_the_handoff_range_for_typical_neo() {
        // v_inf ≈ 5 km/s is mid-range for NEO close approaches; v_esc⊕ ≈ 11.2 km/s.
        let (r_rel, v_rel) = state_at_perigee(5_000.0, 4.0 * R_EARTH);
        let enc = BPlaneEncounter::from_relative_state(r_rel, v_rel, MU_EARTH, R_EARTH).unwrap();
        let f = enc.focusing_factor();
        assert!(
            (1.2..=2.6).contains(&f),
            "focusing factor {f} out of NEO band"
        );
    }
}
