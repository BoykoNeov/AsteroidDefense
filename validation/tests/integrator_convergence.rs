//! RK4 numerical-integrator validation (HANDOFF §6, §10.5, §10.7).
//!
//! This is the **RK4-first** half of the free-invariant seam. `free_invariants.rs`
//! asserts *bounded conservation at machine precision* — the right shape for the
//! analytic Kepler map, which conserves by construction. A numerical stepper does
//! **not** conserve; threading RK4 through that harness would just make it fail.
//! The correct assertion is a **different shape**, not a looser tolerance
//! (HANDOFF §10.5), and this file realizes it:
//!
//! 1. **Fourth-order convergence** — the self-calibrating primary. Integrate a
//!    fixed arc at `N` and `2N` steps; the error against the analytic
//!    `KeplerPropagator` truth must fall ~16× (measured order `log2(e_N/e_2N) ≈
//!    4`). No magic tolerance — the *ratio* is the assertion.
//! 2. **Epoch threading under a genuinely time-varying field** — a two-body field
//!    with the attractor at the origin is *autonomous*, so it cannot distinguish
//!    correct sub-step epoch evaluation from a bug that evaluates every stage at
//!    `t`. A non-autonomous sinusoidal forcing (with a closed-form solution)
//!    pins that, again via fourth-order convergence.
//! 3. **Honest drift** — RK4 energy genuinely drifts (non-vacuously nonzero), yet
//!    the drift shrinks ~16× under step halving. This is the companion to
//!    "don't over-read green": it proves the stepper is *not* secretly conserving.
//!
//! # Oracle validity (HANDOFF §5)
//! The analytic `KeplerPropagator` is a valid truth here **only because the
//! attractor sits at the frame origin** — the propagator's state is
//! attractor-relative, and it coincides with the barycentric integration frame
//! precisely when the attractor is at the origin. Seed = `elements.to_state(μ)`;
//! truth = `KeplerPropagator(elements, μ, t0).state_at(t0 + arc)`.

use asteroid_core::forces::point_mass::{FixedPerturber, PointMassGravity};
use asteroid_core::forces::{ForceError, ForceModel};
use asteroid_core::{
    propagate_fixed, Epoch, KeplerPropagator, OrbitalElements, Propagator, Rk4, StateVector,
};
use nalgebra::Vector3;

/// Sun gravitational parameter, SI (m³/s²) — the representative heliocentric μ
/// used across the core's tests.
const MU_SUN: f64 = 1.327_124_400_18e20;
/// 1 AU in metres.
const AU: f64 = 1.495_978_707e11;

fn epoch0() -> Epoch {
    Epoch::from_tdb_seconds_past_j2000(0.0)
}

/// A two-body field: one attractor of parameter `mu` fixed at the frame origin.
fn two_body_field(mu: f64) -> PointMassGravity {
    PointMassGravity::new(vec![(mu, FixedPerturber::at_origin()).into()])
}

/// Specific orbital energy `ε = ½|v|² − μ/|r|`, computed from the Cartesian state
/// (never from elements — so it actually reflects the integrated trajectory).
fn specific_energy(state: StateVector, mu: f64) -> f64 {
    0.5 * state.velocity.dot(&state.velocity) - mu / state.position.norm()
}

/// Estimated convergence order from two error levels at step counts `n` and `2n`:
/// `order = log2(e_n / e_2n)`. For a fourth-order method in the
/// truncation-dominated regime this is ≈ 4.
fn convergence_order(err_n: f64, err_2n: f64) -> f64 {
    (err_n / err_2n).log2()
}

// --- 1. Fourth-order convergence on the two-body field -----------------------

/// RK4 position error against the analytic Kepler truth, integrating `elements`
/// (about `mu`) forward over `arc` seconds in `n` fixed steps. Returns the
/// absolute position error norm (metres); scale cancels in the order ratio.
fn rk4_position_error(elements: OrbitalElements, mu: f64, arc: f64, n: u32) -> f64 {
    let field = two_body_field(mu);
    let seed = elements.to_state(mu);
    let numeric = propagate_fixed(&Rk4, &field, epoch0(), seed, arc, n).expect("integrates");

    let truth = KeplerPropagator::new(elements, mu, epoch0())
        .expect("valid orbit")
        .state_at(epoch0().shifted_by_seconds(arc))
        .expect("propagates");

    (numeric.position - truth.position).norm()
}

#[test]
fn two_body_rk4_is_fourth_order() {
    // A mildly eccentric heliocentric orbit; integrate a quarter period so the
    // step counts below sit in the truncation-dominated regime (errors ~1e-6…1e-7,
    // far above the ~1e-13 round-off floor that would drag the measured order down).
    let elements = OrbitalElements::new(1.2 * AU, 0.2, 0.4, 1.0, 2.0, 0.3);
    let period = KeplerPropagator::new(elements, MU_SUN, epoch0())
        .unwrap()
        .period();
    let arc = period / 4.0;

    let n = 60;
    let e_n = rk4_position_error(elements, MU_SUN, arc, n);
    let e_2n = rk4_position_error(elements, MU_SUN, arc, 2 * n);
    let order = convergence_order(e_n, e_2n);

    assert!(
        (3.7..=4.3).contains(&order),
        "expected 4th-order convergence, measured order {order:.3} (e_{n} = {e_n:.3e}, e_{} = {e_2n:.3e})",
        2 * n
    );
}

// --- 2. Epoch threading under a genuinely time-varying field -----------------

/// A spatially-uniform acceleration `a(t) = c·sin(Ω t)` (test-only), with `t`
/// measured in seconds past J2000. Its closed-form trajectory is non-polynomial
/// in `t`, so RK4 is *not* exact — the error converges at fourth order only if
/// each stage is sampled at the correct sub-step epoch (a bug that evaluated all
/// stages at `t` would drop the order).
struct SinForcing {
    c: Vector3<f64>,
    omega: f64,
}

impl ForceModel for SinForcing {
    fn acceleration(&self, epoch: Epoch, _state: &StateVector) -> Result<Vector3<f64>, ForceError> {
        let t = epoch.tdb_seconds_past_j2000();
        Ok(self.c * (self.omega * t).sin())
    }
}

/// Closed-form solution of `r̈ = c·sin(Ω t)` from `t = 0`:
/// `v(t) = v0 + c·(1 − cos Ω t)/Ω`,
/// `r(t) = r0 + v0·t + c·(Ω t − sin Ω t)/Ω²`.
fn sin_forcing_exact(
    r0: Vector3<f64>,
    v0: Vector3<f64>,
    c: Vector3<f64>,
    omega: f64,
    t: f64,
) -> StateVector {
    let v = v0 + c * (1.0 - (omega * t).cos()) / omega;
    let r = r0 + v0 * t + c * (omega * t - (omega * t).sin()) / (omega * omega);
    StateVector::new(r, v)
}

fn sin_forcing_error(omega: f64, arc: f64, n: u32) -> f64 {
    let c = Vector3::new(2.0, -1.0, 0.5);
    let r0 = Vector3::new(10.0, -3.0, 4.0);
    let v0 = Vector3::new(-1.0, 2.0, 0.3);
    let field = SinForcing { c, omega };

    let numeric = propagate_fixed(&Rk4, &field, epoch0(), StateVector::new(r0, v0), arc, n)
        .expect("integrates");
    let exact = sin_forcing_exact(r0, v0, c, omega, arc);
    (numeric.position - exact.position).norm()
}

#[test]
fn time_varying_field_rk4_is_fourth_order() {
    // Ω·arc ≈ 2 (a fraction of a forcing period); step counts keep Ω·h small so
    // the error is truncation-dominated.
    let omega = 1.0;
    let arc = 2.0;
    let n = 20;

    let e_n = sin_forcing_error(omega, arc, n);
    let e_2n = sin_forcing_error(omega, arc, 2 * n);
    let order = convergence_order(e_n, e_2n);

    assert!(
        (3.7..=4.3).contains(&order),
        "epoch-threaded time-varying field: expected 4th order, measured {order:.3} \
         (e_{n} = {e_n:.3e}, e_{} = {e_2n:.3e})",
        2 * n
    );
}

// --- 3. Honest drift: RK4 does not conserve, and the drift shrinks with h -----

/// Relative specific-energy drift over one full period, integrated in `n` steps.
fn rk4_energy_drift_rel(elements: OrbitalElements, mu: f64, n: u32) -> f64 {
    let field = two_body_field(mu);
    let seed = elements.to_state(mu);
    let period = KeplerPropagator::new(elements, mu, epoch0())
        .unwrap()
        .period();

    let energy0 = specific_energy(seed, mu);
    let end = propagate_fixed(&Rk4, &field, epoch0(), seed, period, n).expect("integrates");
    let energy1 = specific_energy(end, mu);
    (energy1 - energy0).abs() / energy0.abs()
}

#[test]
fn rk4_energy_drifts_but_shrinks_with_step() {
    // The point of RK4-first (HANDOFF §10.5): a numerical stepper is NOT a
    // conservative map. This would FAIL free_invariants.rs's 1e-11 bound — which
    // is exactly why that harness is not reused. Here we assert the honest shape:
    //   (a) the drift is genuinely nonzero (non-vacuous — the stepper really
    //       integrates, it does not secretly conserve), and
    //   (b) halving the step shrinks it toward zero (consistent with 4th order).
    let elements = OrbitalElements::new(1.3 * AU, 0.3, 0.5, 0.7, 1.5, 0.0);

    let n = 200;
    let drift_coarse = rk4_energy_drift_rel(elements, MU_SUN, n);
    let drift_fine = rk4_energy_drift_rel(elements, MU_SUN, 2 * n);

    // (a) Non-vacuous: a coarse step drifts well above round-off — proof it is a
    //     genuine integrator, not a conservative map masquerading as one.
    assert!(
        drift_coarse > 1e-10,
        "expected a genuine (nonzero) energy drift at n={n}, got {drift_coarse:.3e}"
    );
    // (b) Convergent: halving the step cuts the drift by a large factor (4th-order
    //     global energy error → ~16×; assert >8× for round-off headroom).
    assert!(
        drift_coarse / drift_fine > 8.0,
        "energy drift should shrink with step: coarse {drift_coarse:.3e}, fine {drift_fine:.3e} \
         (ratio {:.1})",
        drift_coarse / drift_fine
    );
}
