//! Element↔state round-trip tests (HANDOFF §10.3, §10.5-preview).
//!
//! The property under test is a **state** round-trip, not an element one:
//!
//! ```text
//!   S1 = to_state(E)
//!   E2 = from_state(S1)     // E2 may differ from E in the gauge angles (ω, Ω)
//!   S2 = to_state(E2)
//!   assert S1 ≈ S2          // the physical invariant round-trips even when
//!                           // the classical angles are undefined
//! ```
//!
//! This is the correct formulation at the `e → 0` / `i → 0` / `i → π`
//! singularities the task demands we cover: `ω` is undefined for a circular
//! orbit and `Ω` for an equatorial one, so asserting `E2 ≈ E` on those angles
//! would spuriously fail *exactly* where coverage matters. The Cartesian state
//! stays well-conditioned there, so we assert on it. The always-defined shape
//! elements `a, e, i` *are* checked to round-trip.
//!
//! Tolerances are **relative** — with SI units `a ~ 1.5e11 m` and
//! `v ~ 3e4 m/s`, an absolute threshold is meaningless.
//!
//! proptest never samples the measure-zero singular values by chance, so the
//! strategies **union explicit degenerate literals** (`e ∈ {0, 1e-15, …}`,
//! `i ∈ {0, π, …}`) with the random ranges, and the deterministic `#[test]`s
//! below pin the *combined* corners (e=0 ∧ i=0, e=0 ∧ i=π, …) that even a
//! unioned random draw won't hit simultaneously.

use asteroid_core::{OrbitalElements, StateVector};
use proptest::prelude::*;
use std::f64::consts::PI;

/// Sun gravitational parameter, SI (m³/s²) — a representative heliocentric μ.
const MU_SUN: f64 = 1.327_124_400_18e20;

/// Relative tolerance for the state round-trip and the a/e/i checks. Comfortably
/// above the ~1e-13 relative error the exact branches achieve and the ~1e-8
/// gauge-fold error at the very smallest seeded eccentricities.
const REL_TOL: f64 = 1e-7;

/// Assert `S1 = to_state(E)` and `S2 = to_state(from_state(S1))` agree to
/// relative tolerance, and that the well-defined `a, e, i` survive the round
/// trip. Returns the recovered elements for further inspection.
fn assert_state_roundtrips(elements: OrbitalElements) -> OrbitalElements {
    let s1 = elements.to_state(MU_SUN);
    let recovered = OrbitalElements::from_state(s1, MU_SUN)
        .unwrap_or_else(|e| panic!("from_state failed for {elements:?}: {e}"));
    let s2 = recovered.to_state(MU_SUN);

    let pos_res = (s2.position - s1.position).norm() / s1.position.norm();
    let vel_res = (s2.velocity - s1.velocity).norm() / s1.velocity.norm();
    assert!(
        pos_res < REL_TOL,
        "position round-trip {pos_res:.3e} exceeds {REL_TOL:.0e}\n  in : {elements:?}\n  out: {recovered:?}"
    );
    assert!(
        vel_res < REL_TOL,
        "velocity round-trip {vel_res:.3e} exceeds {REL_TOL:.0e}\n  in : {elements:?}\n  out: {recovered:?}"
    );

    // Shape elements are gauge-free and must round-trip.
    let a_res = (recovered.semi_major_axis - elements.semi_major_axis).abs()
        / elements.semi_major_axis.abs();
    assert!(
        a_res < REL_TOL,
        "a: {a_res:.3e}  {elements:?} -> {recovered:?}"
    );
    assert!(
        (recovered.eccentricity - elements.eccentricity).abs() < REL_TOL,
        "e: {elements:?} -> {recovered:?}"
    );
    assert!(
        (recovered.inclination - elements.inclination).abs() < REL_TOL,
        "i: {elements:?} -> {recovered:?}"
    );

    recovered
}

// --- Deterministic corner cases: the combined singularities ------------------

#[test]
fn general_orbit_roundtrips() {
    assert_state_roundtrips(OrbitalElements::new(1.5e11, 0.3, 0.4, 1.0, 2.0, 3.0));
}

#[test]
fn circular_inclined_roundtrips() {
    // e = 0, i ≠ 0: ω undefined → folded into ν (argument of latitude).
    assert_state_roundtrips(OrbitalElements::new(1.5e11, 0.0, 0.9, 1.2, 0.0, 2.5));
}

#[test]
fn elliptical_equatorial_roundtrips() {
    // i = 0, e ≠ 0: Ω undefined → folded into ω (longitude of periapsis).
    assert_state_roundtrips(OrbitalElements::new(1.5e11, 0.3, 0.0, 0.0, 1.1, 2.0));
}

#[test]
fn circular_equatorial_roundtrips() {
    // e = 0 AND i = 0: both Ω and ω undefined → ν = true longitude.
    assert_state_roundtrips(OrbitalElements::new(1.5e11, 0.0, 0.0, 0.0, 0.0, 1.7));
}

#[test]
fn elliptical_retrograde_equatorial_roundtrips() {
    // i = π, e ≠ 0: retrograde equatorial — a singularity the task implies but
    // does not name.
    assert_state_roundtrips(OrbitalElements::new(1.5e11, 0.3, PI, 0.0, 1.1, 2.0));
}

#[test]
fn circular_retrograde_equatorial_roundtrips() {
    // i = π AND e = 0: the combined retrograde-equatorial-circular corner.
    assert_state_roundtrips(OrbitalElements::new(1.5e11, 0.0, PI, 0.0, 0.0, 1.7));
}

#[test]
fn near_but_not_at_singularity_uses_exact_branch() {
    // e and i just above the folding threshold: the standard branch must stay
    // consistent here (this is where a too-large tolerance would hide a bug).
    assert_state_roundtrips(OrbitalElements::new(1.5e11, 1e-8, 1e-8, 1.0, 2.0, 3.0));
}

#[test]
fn from_state_rejects_hyperbolic() {
    // A state well above escape speed at 1 AU: e ≥ 1, out of scope.
    let r = 1.5e11;
    let v_escape = (2.0 * MU_SUN / r).sqrt();
    let state = StateVector::from_components(r, 0.0, 0.0, 0.0, 1.5 * v_escape, 0.0);
    match OrbitalElements::from_state(state, MU_SUN) {
        Err(asteroid_core::ElementsError::NonElliptical { eccentricity }) => {
            assert!(eccentricity >= 1.0, "expected e ≥ 1, got {eccentricity}");
        }
        other => panic!("expected NonElliptical, got {other:?}"),
    }
}

#[test]
fn from_state_rejects_degenerate() {
    // Radial (zero angular momentum) state.
    let state = StateVector::from_components(1.5e11, 0.0, 0.0, 100.0, 0.0, 0.0);
    assert_eq!(
        OrbitalElements::from_state(state, MU_SUN),
        Err(asteroid_core::ElementsError::Degenerate)
    );
}

// --- Property tests: unioned degenerate literals + random ranges -------------

/// Eccentricity: explicit near-circular literals unioned with a broad range.
fn ecc_strategy() -> impl Strategy<Value = f64> {
    prop_oneof![
        Just(0.0),
        Just(1e-15),
        Just(1e-12),
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
        Just(1e-12),
        Just(1e-9),
        Just(1e-6),
        Just(PI / 2.0),
        Just(PI - 1e-9),
        Just(PI - 1e-12),
        Just(PI),
        0.0f64..PI,
    ]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2048))]

    #[test]
    fn state_roundtrips_across_element_space(
        a in 5e10f64..8e11f64,
        e in ecc_strategy(),
        i in incl_strategy(),
        raan in 0.0f64..std::f64::consts::TAU,
        argp in 0.0f64..std::f64::consts::TAU,
        nu in 0.0f64..std::f64::consts::TAU,
    ) {
        assert_state_roundtrips(OrbitalElements::new(a, e, i, raan, argp, nu));
    }
}
