//! `Integrator` — a swappable ODE stepper for the equation of motion (HANDOFF §4).
//!
//! The integrated body obeys a second-order ODE, `r̈ = a(t, r, ṙ)`, with the
//! acceleration supplied by a [`ForceModel`](crate::forces::ForceModel). An
//! [`Integrator`] advances a [`StateVector`] `(r, ṙ)` by one step of `Δt`; the
//! trait exists so RK4 / dop853 / (Tier-2) IAS15 / symplectic steppers are
//! interchangeable behind one interface (§4, §5). It is deliberately
//! **object-safe** — no generics, no `Self` in the signature — matching the
//! [`Propagator`](crate::propagator::Propagator) convention, so a stepper can be
//! chosen at run time via `&dyn Integrator`.
//!
//! # This task (§10.7): RK4 first
//! [`Rk4`] is the classical fixed-step fourth-order Runge–Kutta method. It comes
//! first specifically to **exercise the free-invariant tests**: unlike the
//! analytic Kepler map (which conserves by construction), a numerical stepper
//! *drifts*, so it is what forces the invariant harness to grow its
//! error-growth-rate / convergence-order assertion shape (HANDOFF §6, §10.5). The
//! MVP encounter integrator is **dop853** (adaptive, 8th-order, dense output for
//! the clock); it lands in a later batch. RK4 is not used for the encounter.
//!
//! # Frame
//! The stepper is frame-agnostic — it advances whatever frame the force model and
//! seed state are expressed in. The core integrates in **barycentric ICRF, SI**
//! (HANDOFF §5); the force model enforces that, the integrator just steps.

use crate::epoch::Epoch;
use crate::forces::{ForceError, ForceModel};
use crate::state::StateVector;

/// Failure modes of a step. Currently a step can only fail because the force
/// model failed to evaluate; the enum is kept distinct (rather than surfacing
/// [`ForceError`] directly) so adaptive steppers can later report their own
/// modes (e.g. a step-size floor) without a breaking change.
#[derive(Debug, Clone, PartialEq)]
pub enum IntegratorError {
    /// The force model could not be evaluated at a stage of the step.
    Force(ForceError),
}

impl std::fmt::Display for IntegratorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IntegratorError::Force(e) => write!(f, "integrator force evaluation failed: {e}"),
        }
    }
}

impl std::error::Error for IntegratorError {}

impl From<ForceError> for IntegratorError {
    fn from(e: ForceError) -> Self {
        IntegratorError::Force(e)
    }
}

/// A single-step ODE integrator for `r̈ = a(t, r, ṙ)` (HANDOFF §4). Object-safe.
pub trait Integrator {
    /// Advance `state` (at `epoch`) by `dt` seconds under `force`, returning the
    /// state at `epoch + dt`. `dt` may be negative (backward integration).
    fn step(
        &self,
        force: &dyn ForceModel,
        epoch: Epoch,
        state: &StateVector,
        dt: f64,
    ) -> Result<StateVector, IntegratorError>;
}

/// Classical fixed-step fourth-order Runge–Kutta (RK4).
///
/// Applied to the first-order system `y = (r, ṙ)`, `ẏ = (ṙ, a(t, r, ṙ))`. The
/// four stages evaluate the force at `t`, `t + Δt/2` (twice), and `t + Δt`, so a
/// **time-varying** field (moving perturbers) is sampled at the correct
/// sub-step epochs — carrying the epoch through each stage is load-bearing, not
/// cosmetic (a stepper that evaluated every stage at `t` would silently drop to
/// first order on a non-autonomous field). Global error is `O(Δt⁴)`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Rk4;

impl Integrator for Rk4 {
    fn step(
        &self,
        force: &dyn ForceModel,
        epoch: Epoch,
        state: &StateVector,
        dt: f64,
    ) -> Result<StateVector, IntegratorError> {
        let h = dt;
        let r = state.position;
        let v = state.velocity;

        let epoch_mid = epoch.shifted_by_seconds(0.5 * h);
        let epoch_end = epoch.shifted_by_seconds(h);

        // Stage 1 — slope at the start.
        let a1 = force.acceleration(epoch, state)?;

        // Stage 2 — slope at the midpoint, using the stage-1 slope.
        let s2 = StateVector::new(r + 0.5 * h * v, v + 0.5 * h * a1);
        let a2 = force.acceleration(epoch_mid, &s2)?;

        // Stage 3 — slope at the midpoint, using the stage-2 slope.
        let s3 = StateVector::new(r + 0.5 * h * s2.velocity, v + 0.5 * h * a2);
        let a3 = force.acceleration(epoch_mid, &s3)?;

        // Stage 4 — slope at the endpoint, using the stage-3 slope.
        let s4 = StateVector::new(r + h * s3.velocity, v + h * a3);
        let a4 = force.acceleration(epoch_end, &s4)?;

        // Weighted average of the four slopes (the position-derivatives are the
        // stage velocities; the velocity-derivatives are the stage accelerations).
        let dr = (v + 2.0 * s2.velocity + 2.0 * s3.velocity + s4.velocity) / 6.0;
        let dv = (a1 + 2.0 * a2 + 2.0 * a3 + a4) / 6.0;

        Ok(StateVector::new(r + h * dr, v + h * dv))
    }
}

/// Advance `state0` (at `epoch0`) by `total_dt` seconds in `n_steps` equal
/// fixed steps of `total_dt / n_steps`, returning the final state.
///
/// A convenience over [`Integrator::step`] for fixed-cadence propagation (and the
/// convergence-order tests, which compare the same arc at `N` vs `2N` steps).
/// `n_steps` must be at least 1.
pub fn propagate_fixed(
    integrator: &dyn Integrator,
    force: &dyn ForceModel,
    epoch0: Epoch,
    state0: StateVector,
    total_dt: f64,
    n_steps: u32,
) -> Result<StateVector, IntegratorError> {
    assert!(n_steps >= 1, "propagate_fixed needs at least one step");
    let h = total_dt / (n_steps as f64);
    let mut state = state0;
    let mut epoch = epoch0;
    for _ in 0..n_steps {
        state = integrator.step(force, epoch, &state, h)?;
        epoch = epoch.shifted_by_seconds(h);
    }
    Ok(state)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forces::point_mass::{FixedPerturber, PointMassGravity};
    use nalgebra::Vector3;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    /// A spatially-uniform, constant-in-time acceleration `g` (a test-only force).
    /// RK4 integrates `r̈ = g` exactly, so it pins the stage-weighting arithmetic.
    struct UniformField {
        g: Vector3<f64>,
    }
    impl ForceModel for UniformField {
        fn acceleration(
            &self,
            _epoch: Epoch,
            _state: &StateVector,
        ) -> Result<Vector3<f64>, ForceError> {
            Ok(self.g)
        }
    }

    /// A spatially-uniform acceleration that is **linear in time**: `a(t) = c·t`
    /// with `t` measured in seconds past J2000. RK4 integrates a system whose
    /// acceleration is a cubic-or-lower polynomial in `t` *exactly*, so a nonzero
    /// residual here can only come from evaluating a stage at the wrong epoch —
    /// this is the cheap, self-contained epoch-threading probe.
    struct LinearInTimeField {
        c: Vector3<f64>,
    }
    impl ForceModel for LinearInTimeField {
        fn acceleration(
            &self,
            epoch: Epoch,
            _state: &StateVector,
        ) -> Result<Vector3<f64>, ForceError> {
            Ok(self.c * epoch.tdb_seconds_past_j2000())
        }
    }

    #[test]
    fn constant_acceleration_is_integrated_exactly() {
        let g = Vector3::new(0.3, -1.2, 0.7);
        let field = UniformField { g };
        let r0 = Vector3::new(1.0, 2.0, -3.0);
        let v0 = Vector3::new(-0.5, 0.4, 0.9);
        let s0 = StateVector::new(r0, v0);
        let t = 10.0;

        let end = propagate_fixed(&Rk4, &field, epoch0(), s0, t, 7).unwrap();
        // Closed form: r = r0 + v0 t + ½ g t², v = v0 + g t.
        let r_exact = r0 + v0 * t + 0.5 * g * t * t;
        let v_exact = v0 + g * t;
        assert!(
            (end.position - r_exact).norm() < 1e-9,
            "pos {:?}",
            end.position
        );
        assert!(
            (end.velocity - v_exact).norm() < 1e-12,
            "vel {:?}",
            end.velocity
        );
    }

    #[test]
    fn linear_in_time_acceleration_pins_epoch_threading() {
        // a(t) = c t  ⇒  v(t) = v0 + ½ c t²,  r(t) = r0 + v0 t + (1/6) c t³
        // (integrating from t=0). RK4 is exact for this cubic-in-t trajectory, so
        // any epoch-threading bug (all stages at t) breaks the match immediately.
        let c = Vector3::new(2.0, -1.0, 0.5);
        let field = LinearInTimeField { c };
        let r0 = Vector3::new(0.0, 0.0, 0.0);
        let v0 = Vector3::new(1.0, 2.0, 3.0);
        let s0 = StateVector::new(r0, v0);
        let t = 6.0;

        let end = propagate_fixed(&Rk4, &field, epoch0(), s0, t, 5).unwrap();
        let r_exact = r0 + v0 * t + c * (t * t * t) / 6.0;
        let v_exact = v0 + 0.5 * c * t * t;
        assert!(
            (end.position - r_exact).norm() < 1e-6,
            "pos {:?} vs {:?}",
            end.position,
            r_exact
        );
        assert!(
            (end.velocity - v_exact).norm() < 1e-9,
            "vel {:?} vs {:?}",
            end.velocity,
            v_exact
        );
    }

    #[test]
    fn a_single_step_forward_then_back_returns_to_start() {
        // Reversibility of the stepper itself on a two-body field: step +h, then
        // step −h from there, recover the seed (RK4 is not time-symmetric, so this
        // holds to truncation order, not exactly — hence the modest tolerance).
        let mu = 1.327_124_400_18e20;
        let field = PointMassGravity::new(vec![(mu, FixedPerturber::at_origin()).into()]);
        let au = 1.495_978_707e11;
        let s0 = StateVector::from_components(au, 0.0, 0.0, 0.0, (mu / au).sqrt(), 0.0);
        let h = 3600.0; // one hour

        let fwd = Rk4.step(&field, epoch0(), &s0, h).unwrap();
        let back = Rk4
            .step(&field, epoch0().shifted_by_seconds(h), &fwd, -h)
            .unwrap();
        assert!(
            (back.position - s0.position).norm() / au < 1e-10,
            "pos err {:.3e}",
            (back.position - s0.position).norm() / au
        );
    }

    #[test]
    fn object_safe_as_dyn_integrator() {
        let field = PointMassGravity::new(vec![(1.0, FixedPerturber::at_origin()).into()]);
        let dynamic: &dyn Integrator = &Rk4;
        let s = StateVector::from_components(1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(dynamic.step(&field, epoch0(), &s, 0.01).is_ok());
    }
}
