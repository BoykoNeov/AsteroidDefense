//! `Propagator` — the "state at an epoch" boundary, plus the analytic Kepler impl.
//!
//! A [`Propagator`] answers one question: *given an epoch, where is the body?*
//! (HANDOFF §4, §10.4). It is the seam between the two ways the simulation gets a
//! state — the fast **analytic Kepler** map used for context planets, and the
//! numerically-integrated propagator for the asteroid + encounter (a later task).
//! The trait is deliberately **object-safe** (no generics, no `Self` in the
//! return, `&self` only) so heterogeneous propagators can be queried uniformly
//! through `dyn Propagator` — Kepler for the background planets, the integrator
//! for the asteroid — behind one interface.
//!
//! # Frame
//! The Kepler propagator is a **two-body** map about a single attractor. Its
//! output state is expressed **relative to that attractor** (the body whose `μ`
//! was supplied at construction) — *not* the barycentric-ICRF frame the core
//! integrates in (HANDOFF §5). It is the right tool for cosmetic Tier-0 context
//! orbits; it is *never* used to decide hit-vs-miss (that is the integrated
//! encounter's job).
//!
//! # The anomaly machinery (deferred from §10.3)
//! [`crate::elements`] works in the true anomaly `ν` directly and does no
//! time evolution. Time evolution is uniform in the **mean anomaly** `M`, so
//! advancing an orbit means `ν → E → M`, step `M` linearly in time, then invert
//! `M → E → ν` by solving **Kepler's equation** `M = E − e·sin E`. That
//! machinery — the `ν↔E↔M` conversions and the Newton solver — lives here, as
//! the free functions [`eccentric_from_true`], [`mean_from_eccentric`],
//! [`true_from_eccentric`], and [`solve_kepler`].

use crate::elements::OrbitalElements;
use crate::epoch::Epoch;
use crate::state::StateVector;
use std::f64::consts::{PI, TAU};

/// Failure modes of propagation.
///
/// A single concrete enum (rather than a per-impl associated type) keeps
/// [`Propagator`] object-safe. The Kepler impl only ever produces
/// [`PropagatorError::InvalidOrbit`] (at construction) and, in principle,
/// [`PropagatorError::NonConvergence`] (never seen for `e < 1` in practice);
/// later propagators reuse the enum.
#[derive(Debug, Clone, PartialEq)]
pub enum PropagatorError {
    /// The orbit is not a bound ellipse the Kepler map can propagate: the
    /// semi-major axis is non-positive, `μ` is non-positive, or the eccentricity
    /// is outside `[0, 1)`.
    InvalidOrbit {
        semi_major_axis: f64,
        eccentricity: f64,
        mu: f64,
    },
    /// Kepler's equation failed to converge within the iteration cap. Not
    /// expected for `e < 1`; surfaced rather than silently returning a bad root.
    NonConvergence {
        mean_anomaly: f64,
        eccentricity: f64,
        residual: f64,
    },
}

impl std::fmt::Display for PropagatorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PropagatorError::InvalidOrbit {
                semi_major_axis,
                eccentricity,
                mu,
            } => write!(
                f,
                "invalid orbit for Kepler propagation (a = {semi_major_axis:.6e}, e = {eccentricity:.6}, μ = {mu:.6e}); need a > 0, μ > 0, 0 ≤ e < 1"
            ),
            PropagatorError::NonConvergence {
                mean_anomaly,
                eccentricity,
                residual,
            } => write!(
                f,
                "Kepler's equation did not converge (M = {mean_anomaly:.6}, e = {eccentricity:.6}, residual = {residual:.3e})"
            ),
        }
    }
}

impl std::error::Error for PropagatorError {}

/// The state-at-an-epoch boundary (HANDOFF §4). Object-safe by design: query any
/// implementation through `&dyn Propagator`.
pub trait Propagator {
    /// State of the body at `epoch`, in the propagator's own frame (for
    /// [`KeplerPropagator`], relative to the attractor whose `μ` it holds).
    fn state_at(&self, epoch: Epoch) -> Result<StateVector, PropagatorError>;
}

/// Analytic two-body (Kepler) propagator about a single attractor of
/// gravitational parameter `μ`.
///
/// Constructed from a reference orbit ([`OrbitalElements`]) valid at a reference
/// [`Epoch`]; [`state_at`](Propagator::state_at) evaluates the orbit at any other
/// epoch by advancing the mean anomaly. Because a fixed ellipse conserves
/// `a, e, i, Ω, ω` by construction, only the anomaly changes with time — the
/// propagator precomputes the mean motion `n = √(μ/a³)` and the reference mean
/// anomaly `M₀` once, so each query is a single Kepler solve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeplerPropagator {
    mu: f64,
    epoch0: Epoch,
    elements0: OrbitalElements,
    /// Mean motion `n = √(μ/a³)`, rad/s.
    mean_motion: f64,
    /// Mean anomaly at `epoch0`, rad.
    mean_anomaly0: f64,
}

impl KeplerPropagator {
    /// Build a Kepler propagator for `elements` (valid at `epoch`) about an
    /// attractor of gravitational parameter `mu` (SI, m³/s²).
    ///
    /// This is the validating boundary: [`OrbitalElements::new`] does *not* check
    /// that the orbit is a bound ellipse (it only wraps/clamps angles), so the
    /// precondition `a > 0`, `μ > 0`, `0 ≤ e < 1` is enforced here and reported
    /// as [`PropagatorError::InvalidOrbit`] rather than producing `NaN` downstream.
    pub fn new(elements: OrbitalElements, mu: f64, epoch: Epoch) -> Result<Self, PropagatorError> {
        let a = elements.semi_major_axis;
        let e = elements.eccentricity;
        // Positive predicate (not a negated `>`) so NaN inputs fail closed:
        // `a > 0.0` / `.contains` are all false for NaN, so `valid` is false.
        let valid =
            a.is_finite() && a > 0.0 && mu.is_finite() && mu > 0.0 && (0.0..1.0).contains(&e);
        if !valid {
            return Err(PropagatorError::InvalidOrbit {
                semi_major_axis: a,
                eccentricity: e,
                mu,
            });
        }

        let mean_motion = (mu / (a * a * a)).sqrt();
        let ecc_anomaly0 = eccentric_from_true(elements.true_anomaly, e);
        let mean_anomaly0 = mean_from_eccentric(ecc_anomaly0, e);

        Ok(Self {
            mu,
            epoch0: epoch,
            elements0: elements,
            mean_motion,
            mean_anomaly0,
        })
    }

    /// Gravitational parameter of the attractor, m³/s².
    pub fn mu(&self) -> f64 {
        self.mu
    }

    /// Reference epoch the propagator was seeded at.
    pub fn epoch(&self) -> Epoch {
        self.epoch0
    }

    /// Reference elements the propagator was seeded with.
    pub fn elements(&self) -> OrbitalElements {
        self.elements0
    }

    /// Mean motion `n = √(μ/a³)`, rad/s.
    pub fn mean_motion(&self) -> f64 {
        self.mean_motion
    }

    /// Orbital period `T = 2π/n`, seconds.
    pub fn period(&self) -> f64 {
        TAU / self.mean_motion
    }

    /// Elements at `epoch`: the reference orbit with its true anomaly advanced.
    /// Only `ν` changes; `a, e, i, Ω, ω` are carried through unchanged.
    pub fn elements_at(&self, epoch: Epoch) -> Result<OrbitalElements, PropagatorError> {
        let dt = epoch.tdb_seconds_past_j2000() - self.epoch0.tdb_seconds_past_j2000();
        let e = self.elements0.eccentricity;

        let mean_anomaly = self.mean_anomaly0 + self.mean_motion * dt;
        let ecc_anomaly = solve_kepler(mean_anomaly, e)?;
        let true_anomaly = true_from_eccentric(ecc_anomaly, e);

        Ok(OrbitalElements::new(
            self.elements0.semi_major_axis,
            e,
            self.elements0.inclination,
            self.elements0.raan,
            self.elements0.arg_periapsis,
            true_anomaly,
        ))
    }
}

impl Propagator for KeplerPropagator {
    fn state_at(&self, epoch: Epoch) -> Result<StateVector, PropagatorError> {
        Ok(self.elements_at(epoch)?.to_state(self.mu))
    }
}

// --- Anomaly machinery (deferred from §10.3) ---------------------------------

/// Max Newton iterations before [`solve_kepler`] declares non-convergence. For
/// `e < 1` the iteration converges quadratically in well under 10 steps from the
/// seed below; this cap is a safety backstop, not an operating point.
const KEPLER_MAX_ITERS: u32 = 100;

/// Convergence tolerance on the Kepler residual `|E − e·sin E − M|`, radians.
const KEPLER_TOL: f64 = 1e-13;

/// Eccentric anomaly `E` from true anomaly `ν`:
/// `E = atan2(√(1−e²)·sin ν, e + cos ν)`. The `atan2` form is quadrant-correct
/// with no half-angle `tan` blow-up. Result in `(−π, π]`.
pub fn eccentric_from_true(true_anomaly: f64, eccentricity: f64) -> f64 {
    let (s, c) = true_anomaly.sin_cos();
    let beta = (1.0 - eccentricity * eccentricity).sqrt();
    (beta * s).atan2(eccentricity + c)
}

/// Mean anomaly `M` from eccentric anomaly `E` via Kepler's equation directly:
/// `M = E − e·sin E`.
pub fn mean_from_eccentric(eccentric_anomaly: f64, eccentricity: f64) -> f64 {
    eccentric_anomaly - eccentricity * eccentric_anomaly.sin()
}

/// True anomaly `ν` from eccentric anomaly `E`:
/// `ν = atan2(√(1−e²)·sin E, cos E − e)`, wrapped to `[0, 2π)` to match the
/// [`OrbitalElements`] angle convention.
pub fn true_from_eccentric(eccentric_anomaly: f64, eccentricity: f64) -> f64 {
    let (s, c) = eccentric_anomaly.sin_cos();
    let beta = (1.0 - eccentricity * eccentricity).sqrt();
    wrap_2pi((beta * s).atan2(c - eccentricity))
}

/// Solve Kepler's equation `M = E − e·sin E` for the eccentric anomaly `E`.
///
/// `mean_anomaly` may be any real (it is wrapped to `[−π, π)` first, which keeps
/// the solve well-conditioned over many periods); `eccentricity ∈ [0, 1)`.
/// Newton–Raphson from the `E₀ = M + e·sin M` seed — the derivative
/// `1 − e·cos E ≥ 1 − e > 0` never vanishes, so the step is always well-defined.
/// Returns the root in `[−π, π)`, or [`PropagatorError::NonConvergence`] if the
/// residual stays above [`KEPLER_TOL`] after [`KEPLER_MAX_ITERS`].
///
/// **Near-parabolic caveat.** This plain seed converges poorly for very high
/// eccentricity (`e ≳ 0.99`) near periapsis passage — the classic pathological
/// region. It fails loudly with [`PropagatorError::NonConvergence`] rather than
/// returning a bad root; when Phase-2 brings real high-`e` NEOs, the fix is a
/// better initial guess (e.g. Danby's cubic seed), not a larger iteration cap.
pub fn solve_kepler(mean_anomaly: f64, eccentricity: f64) -> Result<f64, PropagatorError> {
    let m = wrap_pi(mean_anomaly);
    let mut ecc_anomaly = m + eccentricity * m.sin();

    let mut residual = kepler_residual(ecc_anomaly, eccentricity, m);
    let mut iters = 0;
    while residual.abs() >= KEPLER_TOL && iters < KEPLER_MAX_ITERS {
        let derivative = 1.0 - eccentricity * ecc_anomaly.cos();
        ecc_anomaly -= residual / derivative;
        residual = kepler_residual(ecc_anomaly, eccentricity, m);
        iters += 1;
    }

    if residual.abs() < KEPLER_TOL {
        Ok(ecc_anomaly)
    } else {
        Err(PropagatorError::NonConvergence {
            mean_anomaly: m,
            eccentricity,
            residual,
        })
    }
}

/// Kepler residual `f(E) = E − e·sin E − M` whose root is the eccentric anomaly.
fn kepler_residual(eccentric_anomaly: f64, eccentricity: f64, mean_anomaly: f64) -> f64 {
    eccentric_anomaly - eccentricity * eccentric_anomaly.sin() - mean_anomaly
}

/// Wrap an angle to `[0, 2π)`.
fn wrap_2pi(angle: f64) -> f64 {
    angle.rem_euclid(TAU)
}

/// Wrap an angle to `[−π, π)`.
fn wrap_pi(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    /// Sun gravitational parameter, SI (m³/s²) — the same representative
    /// heliocentric μ used by the element↔state round-trip tests.
    const MU_SUN: f64 = 1.327_124_400_18e20;
    /// 1 AU in metres, a representative semi-major axis.
    const AU: f64 = 1.495_978_707e11;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    /// Relative agreement between two states (position and velocity).
    fn state_rel_err(a: StateVector, b: StateVector) -> f64 {
        let p = (a.position - b.position).norm() / b.position.norm();
        let v = (a.velocity - b.velocity).norm() / b.velocity.norm();
        p.max(v)
    }

    // --- The anomaly machinery, directly ------------------------------------

    #[test]
    fn kepler_residual_is_zero_at_the_root() {
        // Solve, then confirm E − e·sin E == M to tolerance across e and M.
        for &e in &[0.0, 1e-9, 0.1, 0.5, 0.9, 0.95] {
            for k in 0..32 {
                let m = -PI + (k as f64) * (TAU / 32.0);
                let ecc = solve_kepler(m, e).expect("converges for e < 1");
                let res = (ecc - e * ecc.sin() - wrap_pi(m)).abs();
                assert!(res < 1e-12, "residual {res:.3e} for e={e}, M={m}");
            }
        }
    }

    #[test]
    fn anomaly_round_trip_nu_e_m_e_nu() {
        // ν → E → M → (solve) E → ν must return the original ν, including e→0
        // and across the full [0, 2π).
        for &e in &[0.0, 1e-12, 1e-6, 0.3, 0.7, 0.9] {
            for k in 0..64 {
                let nu = (k as f64) * (TAU / 64.0);
                let ecc = eccentric_from_true(nu, e);
                let m = mean_from_eccentric(ecc, e);
                let ecc2 = solve_kepler(m, e).unwrap();
                let nu2 = true_from_eccentric(ecc2, e);
                // Compare as points on the circle to avoid a 0/2π wrap false-fail.
                let d = (nu2 - nu).rem_euclid(TAU);
                let d = d.min(TAU - d);
                assert!(d < 1e-10, "Δν {d:.3e} for e={e}, ν={nu}");
            }
        }
    }

    #[test]
    fn solve_kepler_wraps_many_periods() {
        // A mean anomaly many turns out must land on the same root as its
        // principal value.
        let e = 0.6;
        let base = 0.734_f64;
        let root = solve_kepler(base, e).unwrap();
        let wrapped = solve_kepler(base + 17.0 * TAU, e).unwrap();
        assert!((root - wrapped).abs() < 1e-12);
    }

    // --- The propagator: known-answer anchors (not self-referential in μ) ----

    #[test]
    fn zero_dt_reproduces_the_seed_state() {
        let elems = OrbitalElements::new(1.3 * AU, 0.4, 0.5, 1.0, 2.0, 3.0);
        let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();
        let at0 = prop.state_at(epoch0()).unwrap();
        let seed = elems.to_state(MU_SUN);
        assert!(state_rel_err(at0, seed) < 1e-12);
    }

    #[test]
    fn period_computed_independently_returns_to_start() {
        // T is computed from a and μ *independently* of the propagator's own
        // mean motion, so this pins n = √(μ/a³) in physical units — a μ/unit
        // slip that the state round-trip would cancel shows up here.
        let a = 1.3 * AU;
        let elems = OrbitalElements::new(a, 0.4, 0.5, 1.0, 2.0, 3.0);
        let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();

        let period = TAU * (a * a * a / MU_SUN).sqrt();
        let start = prop.state_at(epoch0()).unwrap();
        let after_one_period = prop.state_at(epoch0().shifted_by_seconds(period)).unwrap();
        assert!(
            state_rel_err(after_one_period, start) < 1e-9,
            "period return err {:.3e}",
            state_rel_err(after_one_period, start)
        );
        // The propagator's own period must equal the independent value.
        assert!((prop.period() - period).abs() / period < 1e-12);
    }

    #[test]
    fn circular_quarter_and_half_period_hit_known_geometry() {
        // Circular equatorial orbit starting at ν=0 → r=(a,0,0), v=(0,√(μ/a),0).
        // After T/4 the body is at (0,a,0); after T/2 at (−a,0,0). Pure geometry,
        // no round-trip cancellation.
        let a = AU;
        let v_circ = (MU_SUN / a).sqrt();
        let elems = OrbitalElements::new(a, 0.0, 0.0, 0.0, 0.0, 0.0);
        let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();
        let t = prop.period();

        let quarter = prop.state_at(epoch0().shifted_by_seconds(t / 4.0)).unwrap();
        assert!((quarter.position - Vector3::new(0.0, a, 0.0)).norm() / a < 1e-9);
        assert!((quarter.velocity - Vector3::new(-v_circ, 0.0, 0.0)).norm() / v_circ < 1e-9);

        let half = prop.state_at(epoch0().shifted_by_seconds(t / 2.0)).unwrap();
        assert!((half.position - Vector3::new(-a, 0.0, 0.0)).norm() / a < 1e-9);
        assert!((half.velocity - Vector3::new(0.0, -v_circ, 0.0)).norm() / v_circ < 1e-9);
    }

    #[test]
    fn eccentric_half_period_goes_periapsis_to_apoapsis() {
        // Equatorial ellipse (argp=0) starting at periapsis (ν=0, r=a(1−e) on +x).
        // After T/2 the body is at apoapsis: r=a(1+e) on −x. Anchors the anomaly
        // machinery under eccentricity against a closed-form target.
        let a = 1.5 * AU;
        let e = 0.35;
        let elems = OrbitalElements::new(a, e, 0.0, 0.0, 0.0, 0.0);
        let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();

        let peri = prop.state_at(epoch0()).unwrap();
        assert!((peri.position - Vector3::new(a * (1.0 - e), 0.0, 0.0)).norm() / a < 1e-12);

        let apo = prop
            .state_at(epoch0().shifted_by_seconds(prop.period() / 2.0))
            .unwrap();
        assert!(
            (apo.position - Vector3::new(-a * (1.0 + e), 0.0, 0.0)).norm() / a < 1e-9,
            "apoapsis pos {:?}",
            apo.position
        );
    }

    #[test]
    fn forward_then_back_is_reversible() {
        // Propagate forward by Δt, reseed a fresh propagator from that state, then
        // propagate back by Δt; must recover the original state.
        let elems = OrbitalElements::new(1.2 * AU, 0.5, 0.7, 0.3, 1.4, 0.2);
        let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();
        let dt = 1.234e7; // ~5 months
        let mid_epoch = epoch0().shifted_by_seconds(dt);

        let start = prop.state_at(epoch0()).unwrap();
        let mid = prop.state_at(mid_epoch).unwrap();

        let mid_elems = OrbitalElements::from_state(mid, MU_SUN).unwrap();
        let reseeded = KeplerPropagator::new(mid_elems, MU_SUN, mid_epoch).unwrap();
        let back = reseeded.state_at(epoch0()).unwrap();

        assert!(
            state_rel_err(back, start) < 1e-9,
            "reversibility err {:.3e}",
            state_rel_err(back, start)
        );
    }

    // --- Weak-but-cheap invariant: shape elements are conserved --------------

    #[test]
    fn shape_elements_are_conserved_by_construction() {
        // a, e, i, Ω, ω do not evolve under two-body motion — conserved by
        // construction (HANDOFF §6), so this proves little about time evolution;
        // included as a cheap guard that only ν moves.
        let elems = OrbitalElements::new(1.1 * AU, 0.42, 0.9, 2.1, 0.8, 0.0);
        let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();
        let later = prop
            .elements_at(epoch0().shifted_by_seconds(9.9e6))
            .unwrap();
        assert!(
            (later.semi_major_axis - elems.semi_major_axis).abs() / elems.semi_major_axis < 1e-12
        );
        assert!((later.eccentricity - elems.eccentricity).abs() < 1e-12);
        assert!((later.inclination - elems.inclination).abs() < 1e-12);
        assert!((later.raan - elems.raan).abs() < 1e-12);
        assert!((later.arg_periapsis - elems.arg_periapsis).abs() < 1e-12);
    }

    // --- Constructor validation + object safety ------------------------------

    #[test]
    fn rejects_non_elliptical_and_degenerate_orbits() {
        let bad_e = OrbitalElements::new(AU, 1.0, 0.1, 0.0, 0.0, 0.0);
        assert!(matches!(
            KeplerPropagator::new(bad_e, MU_SUN, epoch0()),
            Err(PropagatorError::InvalidOrbit { .. })
        ));

        // OrbitalElements::new does not reject a ≤ 0, so the propagator must.
        let bad_a = OrbitalElements::new(-AU, 0.1, 0.1, 0.0, 0.0, 0.0);
        assert!(matches!(
            KeplerPropagator::new(bad_a, MU_SUN, epoch0()),
            Err(PropagatorError::InvalidOrbit { .. })
        ));

        let bad_mu = OrbitalElements::new(AU, 0.1, 0.1, 0.0, 0.0, 0.0);
        assert!(matches!(
            KeplerPropagator::new(bad_mu, 0.0, epoch0()),
            Err(PropagatorError::InvalidOrbit { .. })
        ));
    }

    #[test]
    fn is_object_safe() {
        // Compile-time proof the trait can be used as `dyn Propagator` — the whole
        // point of the shared error enum and generic-free signature.
        let elems = OrbitalElements::new(AU, 0.1, 0.1, 0.0, 0.0, 0.0);
        let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();
        let dynamic: &dyn Propagator = &prop;
        assert!(dynamic.state_at(epoch0()).is_ok());
    }
}
