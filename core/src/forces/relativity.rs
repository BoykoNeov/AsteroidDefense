//! Relativistic 1PN (first post-Newtonian) Sun term — the first Tier-2 force
//! (HANDOFF §5, §6, §10).
//!
//! Real near-Earth asteroids do not match JPL without the Sun's relativistic
//! correction; it matters most for low-perihelion bodies like Apophis, and
//! **omitting it makes Horizons validation silently fail** (HANDOFF §270). This
//! term is the parameterized-post-Newtonian Schwarzschild acceleration of a test
//! particle in the field of a single central mass (the Sun), taken at the general
//! relativity point `β = γ = 1`:
//!
//! ```text
//! a_1PN = μ / (c² r³) · [ (4 μ/r − v²) r  +  4 (r·v) v ]
//! ```
//!
//! where `r` and `v` are the body's position and velocity **relative to the Sun**
//! (heliocentric), `r = |r|`, `v = |v|`, `μ = GM_sun`, and `c` is the speed of
//! light. This is the standard 1PN two-body form (e.g. Moyer; the `2(β+γ)μ/r −
//! γv²` / `2(1+γ)(r·v)v` PPN expression with `β = γ = 1`). Its signature
//! observable is the **anomalistic apsidal precession**
//!
//! ```text
//! Δϖ = 6π μ / (c² a (1 − e²))   radians per orbit
//! ```
//!
//! which for Mercury is the textbook 42.98″/century — the isolation check the
//! test below reproduces (HANDOFF §6, "the GR term alone must reproduce Mercury's
//! 42.98″/century perihelion precession").
//!
//! # Frame and the central-body state
//! Like every term, this returns an acceleration in the **barycentric (SSB) ICRF**
//! frame (HANDOFF §5). But the *physics* is heliocentric — the formula is written
//! in the Sun's rest frame — so the term needs the Sun's full **state** (position
//! *and* velocity) at the epoch to form the relative `r` and `v`. That is a
//! stronger contract than [`super::point_mass::PerturberEphemeris`] (position
//! only), so 1PN gets its own [`CentralBodyState`] provider rather than growing a
//! `velocity_at` sibling onto the position trait: [`FixedCentralBody`] (constant,
//! for the kernel-free Mercury test and a Sun pinned at the origin) or, at wiring
//! time, an ANISE-backed provider over the real DE440/441 Sun.
//!
//! # μ is passed in, never hardcoded
//! The term carries `μ` as a field, exactly as [`super::point_mass::Perturber`]
//! does. Production must hand it the **same** `μ_sun` the point-mass Sun term
//! uses (the ANISE-loaded GM); GR and Newtonian gravity disagreeing on `μ_sun`
//! would be a silent bias. The tests pass the literal DE constant.

use super::{ForceError, ForceModel};
use crate::epoch::Epoch;
use crate::state::StateVector;
use nalgebra::Vector3;

/// Speed of light in vacuum, m/s — the SI-exact defining value.
pub const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

/// Source of the central body's full **state** (position and velocity) at an
/// epoch, in the barycentric (SSB) ICRF frame, SI units (m, m/s).
///
/// This is the velocity-carrying sibling of
/// [`super::point_mass::PerturberEphemeris`]: the 1PN term forms the body's
/// motion *relative to the Sun*, so it needs the Sun's velocity, not just its
/// position. `Send + Sync` for the same thread-mobility reason as
/// [`ForceModel`](crate::forces::ForceModel) — the term lives inside a force
/// field that must leave the render thread for the scenario build.
pub trait CentralBodyState: Send + Sync {
    /// State of the central body at `epoch` (barycentric ICRF, SI).
    fn state_at(&self, epoch: Epoch) -> Result<StateVector, ForceError>;
}

/// A central body whose state never changes — a fixed point at rest in the frame.
///
/// The workhorse of the isolation test: `FixedCentralBody::at_rest_origin()` puts
/// the Sun at the frame origin with zero velocity, so the body's barycentric
/// state *is* its heliocentric state and the Mercury precession check needs no
/// ephemeris (matching [`super::point_mass::FixedPerturber`]'s role for the
/// two-body Newtonian tests).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedCentralBody {
    state: StateVector,
}

impl FixedCentralBody {
    /// A central body pinned at `state` (barycentric ICRF, SI).
    pub fn new(state: StateVector) -> Self {
        Self { state }
    }

    /// The Sun at the frame origin, at rest.
    pub fn at_rest_origin() -> Self {
        Self {
            state: StateVector::from_components(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        }
    }
}

impl CentralBodyState for FixedCentralBody {
    fn state_at(&self, _epoch: Epoch) -> Result<StateVector, ForceError> {
        Ok(self.state)
    }
}

/// The 1PN (first post-Newtonian) relativistic acceleration from a single central
/// mass, at the general-relativity PPN point `β = γ = 1` (HANDOFF §5).
///
/// Holds the central body's `μ` (m³/s²) and a [`CentralBodyState`] source for its
/// motion. Like the Newtonian point-mass term the integrated body is a test
/// particle — its own mass cancels — so this is an acceleration, and the central
/// body follows its own ephemeris (no back-reaction).
pub struct Relativity1PN {
    /// Gravitational parameter `μ = GM` of the central body, SI (m³/s²).
    mu: f64,
    /// Where the central body is — and how fast — at any epoch.
    central: Box<dyn CentralBodyState>,
}

impl Relativity1PN {
    /// Build the term from the central body's `μ` and a state source.
    pub fn new(mu: f64, central: impl CentralBodyState + 'static) -> Self {
        Self {
            mu,
            central: Box::new(central),
        }
    }

    /// The Sun pinned at the frame origin at rest, with gravitational parameter
    /// `mu` — the kernel-free configuration the Mercury isolation test uses.
    pub fn sun_at_origin(mu: f64) -> Self {
        Self::new(mu, FixedCentralBody::at_rest_origin())
    }
}

impl ForceModel for Relativity1PN {
    fn acceleration(&self, epoch: Epoch, state: &StateVector) -> Result<Vector3<f64>, ForceError> {
        let sun = self.central.state_at(epoch)?;
        // Motion relative to the central body — the frame the 1PN formula is written in.
        let r = state.position - sun.position;
        let v = state.velocity - sun.velocity;

        let r_norm = r.norm();
        // A body coincident with (or a NaN separation from) the central mass is a
        // degenerate configuration, not a physical flyby — fail loud rather than
        // emit a non-finite acceleration. Mirrors the point-mass singularity guard;
        // there is one perturber here, so the index is 0.
        if r_norm == 0.0 || !r_norm.is_finite() {
            return Err(ForceError::Singularity {
                perturber_index: 0,
                separation: r_norm,
            });
        }

        let c2 = SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S;
        let r_dot_v = r.dot(&v);
        let v2 = v.dot(&v);

        // a = μ/(c² r³) · [ (4μ/r − v²) r + 4 (r·v) v ]
        let bracket = (4.0 * self.mu / r_norm - v2) * r + 4.0 * r_dot_v * v;
        Ok(self.mu / (c2 * r_norm * r_norm * r_norm) * bracket)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forces::point_mass::{FixedPerturber, PointMassGravity};
    use crate::forces::CompositeForce;
    use crate::integrator::{Dop853, Integrator};

    /// GM of the Sun, DE440/441 value (m³/s²) — the same literal the propagator,
    /// clock and deflection modules use. Production passes the ANISE-loaded GM;
    /// the kernel-free tests pass this so both terms share one `μ_sun`.
    const MU_SUN: f64 = 1.327_124_400_18e20;
    /// Astronomical unit (m), matching the rest of the crate.
    const AU: f64 = 1.495_978_707e11;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    /// With the body's velocity perpendicular to its radius (`r·v = 0`), the whole
    /// velocity cross-term drops and the 1PN acceleration is purely radial:
    /// `a = μ/(c² r²) · (4μ/r − v²) r̂`. A configuration whose closed form is
    /// unambiguous, so it pins the radial magnitude and sign without re-deriving
    /// the code's own expression.
    #[test]
    fn perpendicular_velocity_gives_a_clean_radial_term() {
        let mu = MU_SUN;
        let big_r = 0.4 * AU;
        let speed = 40_000.0; // m/s, order of an inner-planet speed
        let s = StateVector::from_components(big_r, 0.0, 0.0, 0.0, speed, 0.0);
        let term = Relativity1PN::sun_at_origin(mu);
        let a = term.acceleration(epoch0(), &s).unwrap();

        let c2 = SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S;
        let expected_x = mu / (c2 * big_r * big_r) * (4.0 * mu / big_r - speed * speed);
        assert!(
            (a.x - expected_x).abs() < 1e-20 * expected_x.abs().max(1e-20) + 1e-25,
            "radial component: a.x = {}, expected {expected_x}",
            a.x
        );
        assert!(a.y.abs() < 1e-25 && a.z.abs() < 1e-25, "should be purely radial: {a:?}");
        // The correction is a tiny inward pull (4μ/r ≫ v² here, so the bracket is
        // positive and a points along +x̂ = outward)… sanity: it is small.
        assert!(a.norm() < 1e-6, "1PN correction should be a small acceleration: {}", a.norm());
    }

    /// A configuration with `r·v ≠ 0` exercises the `4(r·v)v` cross-term and its
    /// sign. Expected value written out with explicit scalar arithmetic (an
    /// independent transcription of the formula, not a call back into the code).
    #[test]
    fn oblique_velocity_exercises_the_cross_term() {
        let mu = MU_SUN;
        let rx = 0.5 * AU;
        let (vx, vy) = (12_000.0, 34_000.0);
        let s = StateVector::from_components(rx, 0.0, 0.0, vx, vy, 0.0);
        let a = Relativity1PN::sun_at_origin(mu)
            .acceleration(epoch0(), &s)
            .unwrap();

        let c2 = SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S;
        let r_norm = rx;
        let v2 = vx * vx + vy * vy;
        let r_dot_v = rx * vx;
        let pref = mu / (c2 * r_norm * r_norm * r_norm);
        let ex = pref * ((4.0 * mu / r_norm - v2) * rx + 4.0 * r_dot_v * vx);
        let ey = pref * (4.0 * r_dot_v * vy);
        assert!((a.x - ex).abs() < 1e-22 + 1e-12 * ex.abs(), "a.x={} expected {ex}", a.x);
        assert!((a.y - ey).abs() < 1e-22 + 1e-12 * ey.abs(), "a.y={} expected {ey}", a.y);
        assert!(a.z.abs() < 1e-25, "planar motion stays planar: {a:?}");
        // The cross-term is non-zero here — guards against it being dropped.
        assert!(ey.abs() > 0.0);
    }

    /// A body sitting exactly on the central mass is a singularity, not a flyby.
    #[test]
    fn coincident_with_the_sun_fails_loud() {
        let term = Relativity1PN::sun_at_origin(MU_SUN);
        let s = StateVector::from_components(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        match term.acceleration(epoch0(), &s) {
            Err(ForceError::Singularity {
                perturber_index,
                separation,
            }) => {
                assert_eq!(perturber_index, 0);
                assert_eq!(separation, 0.0);
            }
            other => panic!("expected Singularity, got {other:?}"),
        }
    }

    /// The eccentricity (Laplace–Runge–Lenz) vector `e = (v×h)/μ − r̂`, whose
    /// direction is the perihelion direction. Under pure Newtonian gravity it is
    /// conserved; the 1PN term rotates it, and that rotation *is* the precession.
    fn eccentricity_vector(state: &StateVector, mu: f64) -> Vector3<f64> {
        let r = state.position;
        let v = state.velocity;
        let h = r.cross(&v);
        v.cross(&h) / mu - r.normalize()
    }

    /// Least-squares slope of `y` against integer index `0..n` — the per-orbit
    /// precession rate from the stroboscopic angle samples.
    fn slope_per_step(ys: &[f64]) -> f64 {
        let n = ys.len() as f64;
        let mean_x = (n - 1.0) / 2.0;
        let mean_y = ys.iter().sum::<f64>() / n;
        let mut num = 0.0;
        let mut den = 0.0;
        for (i, &y) in ys.iter().enumerate() {
            let dx = i as f64 - mean_x;
            num += dx * (y - mean_y);
            den += dx * dx;
        }
        num / den
    }

    /// Integrate Mercury and measure the apsidal precession of its eccentricity
    /// vector by **stroboscopic sampling** — record `atan2(e_y, e_x)` once per
    /// orbital period, so the intra-orbit wiggle drops out and a straight-line fit
    /// gives the secular rate. Returns (measured rad/orbit, closed-form rad/orbit).
    ///
    /// `with_gr` toggles the 1PN term. With it **off** this is the control run the
    /// whole test hinges on: it must return ~0, proving the measured signal is
    /// physics and not integrator drift in the LRL vector.
    fn measure_precession(with_gr: bool) -> (f64, f64) {
        // Real Mercury, placed in the xy-plane at perihelion on +x with a purely
        // +y (prograde) velocity — so the perihelion angle starts at 0 and the
        // eccentricity vector's angle is atan2(e_y, e_x), cleanly unwrappable.
        let a = 0.387_098_1 * AU;
        let e = 0.205_630;
        let r_peri = a * (1.0 - e);
        // vis-viva at perihelion: v² = μ(2/r − 1/a); direction +y.
        let v_peri = (MU_SUN * (2.0 / r_peri - 1.0 / a)).sqrt();
        let mut state = StateVector::from_components(r_peri, 0.0, 0.0, 0.0, v_peri, 0.0);

        let period = std::f64::consts::TAU * (a * a * a / MU_SUN).sqrt();

        // Newtonian Sun always; add the 1PN term only for the signal run. Both use
        // the same μ_sun and the same Sun-at-origin, so heliocentric = barycentric.
        let mut model = CompositeForce::new().with(Box::new(PointMassGravity::new(vec![(
            MU_SUN,
            FixedPerturber::at_origin(),
        )
            .into()])));
        if with_gr {
            model = model.with(Box::new(Relativity1PN::sun_at_origin(MU_SUN)));
        }

        // dop853 at tight tolerance: the per-orbit signal is ~5e-7 rad, so the
        // integrator's own LRL drift must sit well below that (the control run
        // measures exactly this).
        let stepper = Dop853::new().with_tolerances(1e-13, 1e-6);

        let n_orbits = 40;
        let mut angles = Vec::with_capacity(n_orbits + 1);
        let mut epoch = epoch0();
        // Sample at t = 0, T, 2T, … : stroboscopic, so the intra-orbit oscillation
        // of the eccentricity-vector direction is sampled at the same phase and
        // cancels out of the fit.
        let ev0 = eccentricity_vector(&state, MU_SUN);
        angles.push(ev0.y.atan2(ev0.x));
        for _ in 0..n_orbits {
            state = stepper.step(&model, epoch, &state, period).unwrap();
            epoch = epoch.shifted_by_seconds(period);
            let ev = eccentricity_vector(&state, MU_SUN);
            angles.push(ev.y.atan2(ev.x));
        }

        // Angles are tiny (≤ ~2e-5 rad total), no wrapping to worry about.
        let measured = slope_per_step(&angles); // rad per orbit
        let closed_form = 6.0 * std::f64::consts::PI * MU_SUN
            / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S * a * (1.0 - e * e));
        (measured, closed_form)
    }

    #[test]
    fn mercury_perihelion_precesses_at_the_1pn_rate() {
        let (measured, closed_form) = measure_precession(true);

        // 1. Prograde: the perihelion advances (positive), not regresses. A sign
        //    error in the (r·v)v term flips this.
        assert!(
            measured > 0.0,
            "precession must be prograde, got {measured} rad/orbit"
        );

        // 2. The signal matches the closed form computed with the SAME constants.
        //    Tolerance covers the stroboscopic sampling + integrator residual.
        let rel_err = (measured - closed_form).abs() / closed_form;
        assert!(
            rel_err < 0.02,
            "measured {measured} rad/orbit vs closed-form {closed_form} (rel err {rel_err:.4})"
        );

        // 3. Human-facing sanity: scaled to arcsec/century it lands near the
        //    textbook 42.98″ (loose — this is a readability check, not the physics
        //    assertion, which is #2 against our own constants).
        let a = 0.387_098_1 * AU;
        let period = std::f64::consts::TAU * (a * a * a / MU_SUN).sqrt();
        let seconds_per_century = 100.0 * 365.25 * 86_400.0;
        let orbits_per_century = seconds_per_century / period;
        let arcsec_per_century =
            measured * orbits_per_century * (180.0 / std::f64::consts::PI) * 3600.0;
        assert!(
            (40.0..46.0).contains(&arcsec_per_century),
            "≈42.98″/century expected, got {arcsec_per_century:.3}″"
        );
    }

    #[test]
    fn newtonian_control_shows_no_precession() {
        // The guard that gives the signal test meaning: the identical integration
        // with the 1PN term OFF must show a precession that is a small fraction of
        // the term-on signal — otherwise a loose tolerance would be "measuring"
        // pure integrator drift.
        let (control, _) = measure_precession(false);
        let (signal, _) = measure_precession(true);
        assert!(
            control.abs() < 0.02 * signal.abs(),
            "control (GR off) precession {control} rad/orbit must be ≪ signal {signal}"
        );
    }
}
