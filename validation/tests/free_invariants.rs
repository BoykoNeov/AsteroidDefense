//! Free-invariant property tests (HANDOFF §6 "free invariants", §10.5).
//!
//! In pure two-body motion four things hold with no external oracle needed:
//!
//! - **specific energy** `ε = ½|v|² − μ/|r|` is constant (`= −μ/2a`);
//! - **angular momentum** `h = r × v` is constant (vector, `|h| = √(μp)`);
//! - **the Laplace–Runge–Lenz vector** is constant — carried here as the
//!   **eccentricity vector** `e⃗ = (v × h − μ r̂)/μ`, which points at periapsis
//!   and has magnitude `e`;
//! - **forward-then-back propagation** returns to the start.
//!
//! These are wired as `proptest` properties over randomized orbits *unioned with
//! the `e→0` / `i→0` / `i→π` singular literals* (random draws never sample the
//! measure-zero singularities — same rationale as `core/tests/element_state_roundtrip.rs`).
//!
//! # Don't over-read green (HANDOFF §6, §10.5)
//! The only propagator that exists at this task is the **analytic Kepler** map,
//! which conserves all of the above **by construction** — a fixed ellipse
//! advanced only in true anomaly `ν`. So a green run here validates the
//! **conversions** (`to_state`, and for reversibility also `from_state`),
//! **not any integrator**. The non-vacuity comes from computing every invariant
//! from the *propagated Cartesian state* `to_state(ν)` — never from the elements,
//! which carry `a, e, i, Ω, ω` through byte-identical and would make e.g. an
//! energy-from-`a` check a constant *read*. Energy is the workhorse: it couples
//! `r` and `v` through vis-viva, so it catches a wrong velocity coefficient in
//! `to_state`.
//!
//! # Per-propagator expectations (the harness seam)
//! [`InvariantTolerances`] parameterizes the assertion so future propagators plug
//! in with their *own* expectation (HANDOFF §6 "validate per propagator"):
//! - **analytic Kepler** (here) → bounded drift at **machine precision**;
//! - **RK4** (§10.7) → energy *drifts*; the right assertion is on the
//!   error-*growth rate* / convergence order, **not** bounded conservation — a
//!   different assertion shape, not just a looser tolerance. This is the other
//!   half of the same seam and now lives in `integrator_convergence.rs`
//!   (fourth-order convergence against the analytic Kepler truth + honest,
//!   step-shrinking drift), which is why it is **not** reused via this harness;
//! - **dop853 / symplectic** (later) → adaptive error control / energy
//!   *bounded-oscillating*, not constant — deferred with their propagators.

use asteroid_core::{Epoch, KeplerPropagator, OrbitalElements, Propagator, StateVector};
use nalgebra::Vector3;
use proptest::prelude::*;
use std::f64::consts::PI;

/// Sun gravitational parameter, SI (m³/s²) — a representative heliocentric μ,
/// matching the value used across the core's tests.
const MU_SUN: f64 = 1.327_124_400_18e20;

/// Number of sample points across one orbital period at which conservation is
/// checked (excludes the reference sample at `t₀`).
const CONSERVATION_SAMPLES: u32 = 12;

// --- Invariant primitives, computed from the Cartesian state -----------------
// Kept as local test helpers (not core public API): they must derive purely from
// `state.position` / `state.velocity` so the test exercises `to_state`'s
// ν-dependence rather than reading a carried-through element.

/// Specific orbital energy `ε = ½|v|² − μ/|r|` (J/kg). Constant `= −μ/2a` for a
/// bound orbit; the strongest single invariant (couples r and v via vis-viva).
fn specific_energy(state: StateVector, mu: f64) -> f64 {
    0.5 * state.velocity.dot(&state.velocity) - mu / state.position.norm()
}

/// Specific angular momentum `h = r × v` (m²/s), constant in magnitude and
/// direction for two-body motion (`|h| = √(μp)`).
fn angular_momentum(state: StateVector) -> Vector3<f64> {
    state.position.cross(&state.velocity)
}

/// Eccentricity vector `e⃗ = (v × h − μ r̂)/μ` — the LRL vector scaled by `μ`.
/// Points toward periapsis; `|e⃗| = e`. Carried in this (dimensionless, O(1))
/// form so the `e → 0` case has a finite reference for an **absolute** drift
/// comparison (a relative one would be 0/0 at a circular orbit).
fn eccentricity_vector(state: StateVector, mu: f64) -> Vector3<f64> {
    let r_vec = state.position;
    let v_vec = state.velocity;
    let h_vec = r_vec.cross(&v_vec);
    (v_vec.cross(&h_vec)) / mu - r_vec / r_vec.norm()
}

/// Relative agreement between two states (max of the position and velocity
/// relative errors) — the reversibility metric.
fn state_rel_err(a: StateVector, b: StateVector) -> f64 {
    let p = (a.position - b.position).norm() / b.position.norm();
    let v = (a.velocity - b.velocity).norm() / b.velocity.norm();
    p.max(v)
}

// --- Per-propagator expectations ---------------------------------------------

/// Tolerances a given propagator is expected to meet on the free invariants.
///
/// Analytic Kepler conserves by construction, so its tolerances are
/// machine-precision-class (see [`analytic_kepler`]). Numerical integrators will
/// need a *different assertion shape* (error-growth rate, or bounded
/// oscillation), not merely different numbers — that is why only the analytic
/// expectation is realized here.
#[derive(Debug, Clone, Copy)]
struct InvariantTolerances {
    /// Relative drift allowed on specific energy and `|h|` across the arc, and on
    /// the closed-form anchors (`ε = −μ/2a`, `|h| = √(μp)`).
    conservation_rel: f64,
    /// **Absolute** drift allowed on the eccentricity vector and on its
    /// closed-form anchor (`|e⃗| = e`) — absolute because `e⃗ → 0` as `e → 0`.
    ecc_vector_abs: f64,
    /// Relative error allowed on forward→back reversibility. Looser than
    /// conservation because it routes through `from_state`'s gauge fold at the
    /// near-circular / near-equatorial singularities (same 1e-7 the element↔state
    /// round-trip test settled on).
    reversibility_rel: f64,
}

impl InvariantTolerances {
    /// The analytic Kepler expectation: conserves everything by construction, so
    /// the conservation bounds are machine-precision-class. `1e-11` (not `1e-13`)
    /// leaves headroom for the ~40× term cancellation in `½v² − μ/r` at the
    /// high-eccentricity, near-periapsis samples.
    fn analytic_kepler() -> Self {
        Self {
            conservation_rel: 1e-11,
            ecc_vector_abs: 1e-10,
            reversibility_rel: 1e-7,
        }
    }
}

// --- The harness -------------------------------------------------------------

/// Assert the conservation invariants hold across one orbital period for any
/// `Propagator`, against the given per-propagator [`InvariantTolerances`].
///
/// Also anchors the `t₀` invariants to their **closed forms** (`ε = −μ/2a`,
/// `|h| = √(μp)`, `|e⃗| = e`) so the check is not merely self-referential in μ —
/// a units/μ slip that a drift-only comparison would cancel is caught here.
fn assert_conserves(
    prop: &dyn Propagator,
    reference: &OrbitalElements,
    mu: f64,
    epoch0: Epoch,
    period: f64,
    tol: InvariantTolerances,
) {
    let s0 = prop.state_at(epoch0).expect("reference state");
    let energy0 = specific_energy(s0, mu);
    let h0 = angular_momentum(s0);
    let evec0 = eccentricity_vector(s0, mu);

    // Closed-form anchors at t₀ (absolute pin on the μ-coupling and units).
    let a = reference.semi_major_axis;
    let p = reference.semi_latus_rectum();
    let energy_closed = -mu / (2.0 * a);
    let h_closed = (mu * p).sqrt();
    assert!(
        (energy0 - energy_closed).abs() / energy_closed.abs() < tol.conservation_rel,
        "energy anchor: {energy0:.6e} vs −μ/2a = {energy_closed:.6e}  ({reference:?})"
    );
    assert!(
        (h0.norm() - h_closed).abs() / h_closed < tol.conservation_rel,
        "|h| anchor: {:.6e} vs √(μp) = {h_closed:.6e}  ({reference:?})",
        h0.norm()
    );
    assert!(
        (evec0.norm() - reference.eccentricity).abs() < tol.ecc_vector_abs,
        "|e⃗| anchor: {:.6e} vs e = {:.6e}  ({reference:?})",
        evec0.norm(),
        reference.eccentricity
    );

    // Conservation: sample across one period, compare drift from the t₀ value.
    for k in 1..=CONSERVATION_SAMPLES {
        let dt = period * (k as f64) / (CONSERVATION_SAMPLES as f64);
        let s = prop
            .state_at(epoch0.shifted_by_seconds(dt))
            .expect("propagated state");

        let energy = specific_energy(s, mu);
        assert!(
            (energy - energy0).abs() / energy0.abs() < tol.conservation_rel,
            "energy drift {:.3e} at k={k}  ({reference:?})",
            (energy - energy0).abs() / energy0.abs()
        );

        let h = angular_momentum(s);
        assert!(
            (h - h0).norm() / h0.norm() < tol.conservation_rel,
            "h drift {:.3e} at k={k}  ({reference:?})",
            (h - h0).norm() / h0.norm()
        );

        let evec = eccentricity_vector(s, mu);
        assert!(
            (evec - evec0).norm() < tol.ecc_vector_abs,
            "e⃗ drift {:.3e} at k={k}  ({reference:?})",
            (evec - evec0).norm()
        );
    }
}

/// Assert forward-then-back reversibility for the analytic Kepler propagator:
/// propagate to a mid epoch, reseed a fresh propagator from that **state** (this
/// is what routes the test through `from_state`), propagate back, and require the
/// original state. Re-calling `state_at(t₀)` on the same instance would be a
/// vacuous identity — the reseed is the point.
fn assert_reversible(reference: OrbitalElements, mu: f64, epoch0: Epoch, tol: InvariantTolerances) {
    let prop = KeplerPropagator::new(reference, mu, epoch0).expect("valid orbit");
    let period = prop.period();
    let start = prop.state_at(epoch0).expect("start state");

    // A sub-period shift and a multi-period shift (exercises the mean-anomaly
    // wrap through several revolutions).
    for &frac in &[0.37_f64, 3.2_f64] {
        let dt = frac * period;
        let mid_epoch = epoch0.shifted_by_seconds(dt);
        let mid = prop.state_at(mid_epoch).expect("mid state");

        let mid_elems = OrbitalElements::from_state(mid, mu).expect("mid is elliptical");
        let reseeded = KeplerPropagator::new(mid_elems, mu, mid_epoch).expect("reseed");
        let back = reseeded.state_at(epoch0).expect("back state");

        assert!(
            state_rel_err(back, start) < tol.reversibility_rel,
            "reversibility err {:.3e} (frac {frac})  ({reference:?})",
            state_rel_err(back, start)
        );
    }
}

/// Full free-invariant check for one orbit: conservation + reversibility under
/// the analytic Kepler expectation.
fn assert_free_invariants(elements: OrbitalElements) {
    let epoch0 = Epoch::from_tdb_seconds_past_j2000(0.0);
    let mu = MU_SUN;
    let tol = InvariantTolerances::analytic_kepler();

    let prop = KeplerPropagator::new(elements, mu, epoch0).expect("valid orbit");
    assert_conserves(&prop, &elements, mu, epoch0, prop.period(), tol);
    assert_reversible(elements, mu, epoch0, tol);
}

// --- Deterministic corners: the combined singularities -----------------------

#[test]
fn general_orbit_holds_invariants() {
    assert_free_invariants(OrbitalElements::new(1.5e11, 0.3, 0.4, 1.0, 2.0, 3.0));
}

#[test]
fn high_eccentricity_holds_invariants() {
    // e = 0.9: the ~cancellation-heavy energy case near periapsis/apoapsis.
    assert_free_invariants(OrbitalElements::new(1.5e11, 0.9, 0.6, 1.0, 2.0, 0.0));
}

#[test]
fn circular_inclined_holds_invariants() {
    // e = 0: eccentricity vector → 0, exercising the absolute-tolerance path.
    assert_free_invariants(OrbitalElements::new(1.5e11, 0.0, 0.9, 1.2, 0.0, 2.5));
}

#[test]
fn elliptical_equatorial_holds_invariants() {
    assert_free_invariants(OrbitalElements::new(1.5e11, 0.3, 0.0, 0.0, 1.1, 2.0));
}

#[test]
fn circular_equatorial_holds_invariants() {
    // e = 0 AND i = 0: both gauge angles undefined.
    assert_free_invariants(OrbitalElements::new(1.5e11, 0.0, 0.0, 0.0, 0.0, 1.7));
}

#[test]
fn retrograde_equatorial_holds_invariants() {
    // i = π: retrograde equatorial (h points −ẑ).
    assert_free_invariants(OrbitalElements::new(1.5e11, 0.3, PI, 0.0, 1.1, 2.0));
}

#[test]
fn circular_retrograde_equatorial_holds_invariants() {
    assert_free_invariants(OrbitalElements::new(1.5e11, 0.0, PI, 0.0, 0.0, 1.7));
}

// --- Property tests: unioned degenerate literals + random ranges -------------
// Same union strategy as core/tests/element_state_roundtrip.rs — random draws
// never hit the measure-zero singularities, so seed them explicitly.

/// Eccentricity: explicit near-circular literals unioned with a broad range
/// (kept below 0.95 so the orbit stays a well-conditioned bound ellipse).
fn ecc_strategy() -> impl Strategy<Value = f64> {
    prop_oneof![
        Just(0.0),
        Just(1e-15),
        Just(1e-9),
        Just(1e-6),
        0.0f64..0.95f64,
    ]
}

/// Inclination: explicit equatorial/polar/retrograde literals unioned with the
/// full `[0, π]` range.
fn incl_strategy() -> impl Strategy<Value = f64> {
    prop_oneof![
        Just(0.0),
        Just(1e-9),
        Just(1e-6),
        Just(PI / 2.0),
        Just(PI - 1e-9),
        Just(PI),
        0.0f64..PI,
    ]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    #[test]
    fn free_invariants_across_element_space(
        a in 5e10f64..8e11f64,
        e in ecc_strategy(),
        i in incl_strategy(),
        raan in 0.0f64..std::f64::consts::TAU,
        argp in 0.0f64..std::f64::consts::TAU,
        nu in 0.0f64..std::f64::consts::TAU,
    ) {
        assert_free_invariants(OrbitalElements::new(a, e, i, raan, argp, nu));
    }
}
