//! §10.6 reference-fixture check: `asteroid_core`'s analytic `KeplerPropagator`
//! against hapsira, an independent two-body oracle.
//!
//! This is rung 2 of the §6 oracle ladder (the free-invariant proptests in
//! `free_invariants.rs` are rung 1). Where the invariant tests only confirm the
//! propagator conserves what a two-body map must conserve — green there is
//! consistent with a wrong-but-self-consistent map — this test pins the actual
//! numbers against a *separate* implementation, so a systematic error (a bad
//! rotation, a μ slip, a sign) that conserves energy would still be caught.
//!
//! What makes the comparison honest is that μ, the frame convention, and time
//! are pinned identically on both sides (see the fixture's `frame_note` /
//! `provenance` and `pyref/generate_kepler_fixture.py`):
//!   * **μ** is ANISE's Sun GM, baked into the generator and re-derived here by
//!     [`sun_gm_matches_fixture`] (gated on a local `.pca`);
//!   * **frame** is the standard 3-1-3 element→Cartesian map on both sides — the
//!     `dt_s == 0` sample isolates that convention (its state depends only on
//!     a, e, ν), so a mismatch there flags a convention slip before any
//!     propagation is blamed;
//!   * **time** is elapsed seconds, so no time-scale conversion enters.

use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::{Epoch, KeplerPropagator, OrbitalElements, Propagator};
use nalgebra::Vector3;
use serde::Deserialize;

const FIXTURE: &str = include_str!("../fixtures/kepler_two_body.json");

/// hapsira is an analytic two-body propagator, so agreement is limited only by
/// float round-off through two independent element→state paths, not by
/// integration error. Observed worst case is ~2.8e-13 (see the printed report,
/// dominated by the 12.7-period sample where phase round-off accumulates); this
/// bound sits a few× above it, leaving margin for libm/BLAS differences across
/// platforms while staying ~1000× tighter than the house 1e-9 — tight enough
/// that a real regression (a wrong rotation or a μ slip) blows through it.
const PROPAGATED_TOL: f64 = 1e-12;

/// The `dt = 0` sample tests only the element→Cartesian convention (no time
/// evolution), so it agrees essentially to round-off (observed ~1e-15) and is
/// held tighter than the propagated samples.
const SEED_TOL: f64 = 1e-13;

/// μ is written into the fixture as the exact value ANISE resolves, and this
/// crate re-derives it the same way, so the pin is effectively bit-exact; the
/// tolerance only absorbs the km³/s²→m³/s² multiply's last ulp.
const MU_TOL: f64 = 1e-12;

#[derive(Deserialize)]
struct Fixture {
    mu_m3_s2: f64,
    orbits: Vec<OrbitFixture>,
}

#[derive(Deserialize)]
struct OrbitFixture {
    label: String,
    elements: Elements,
    samples: Vec<Sample>,
}

#[derive(Deserialize)]
struct Elements {
    a_m: f64,
    ecc: f64,
    inc_rad: f64,
    raan_rad: f64,
    argp_rad: f64,
    nu_rad: f64,
}

#[derive(Deserialize)]
struct Sample {
    period_fraction: f64,
    dt_s: f64,
    position_m: [f64; 3],
    velocity_m_s: [f64; 3],
}

fn load() -> Fixture {
    serde_json::from_str(FIXTURE).expect("fixture parses")
}

/// Relative error between a computed and a reference vector, `‖c − r‖ / ‖r‖`.
fn rel_err(computed: Vector3<f64>, reference: [f64; 3]) -> f64 {
    let r = Vector3::new(reference[0], reference[1], reference[2]);
    (computed - r).norm() / r.norm()
}

#[test]
fn kepler_propagator_matches_hapsira() {
    let fixture = load();
    let mu = fixture.mu_m3_s2;
    // Reference epoch is arbitrary for a two-body map — only Δt matters — so
    // anchor at J2000 and shift by each sample's elapsed seconds.
    let epoch0 = Epoch::from_tdb_seconds_past_j2000(0.0);

    let mut worst_prop = 0.0_f64;
    let mut worst_seed = 0.0_f64;
    for orbit in &fixture.orbits {
        let e = &orbit.elements;
        let elems = OrbitalElements::new(
            e.a_m, e.ecc, e.inc_rad, e.raan_rad, e.argp_rad, e.nu_rad,
        );
        let prop = KeplerPropagator::new(elems, mu, epoch0)
            .unwrap_or_else(|err| panic!("propagator for {:?}: {err}", orbit.label));

        for s in &orbit.samples {
            let state = prop
                .state_at(epoch0.shifted_by_seconds(s.dt_s))
                .unwrap_or_else(|err| {
                    panic!("state_at {:?} dt={}: {err}", orbit.label, s.dt_s)
                });

            let pos_err = rel_err(state.position, s.position_m);
            let vel_err = rel_err(state.velocity, s.velocity_m_s);
            let tol = if s.dt_s == 0.0 { SEED_TOL } else { PROPAGATED_TOL };

            assert!(
                pos_err < tol && vel_err < tol,
                "{}: at {}P (dt={:.3e}s) pos_err {:.3e}, vel_err {:.3e} exceed tol {:.0e}",
                orbit.label, s.period_fraction, s.dt_s, pos_err, vel_err, tol
            );
            if s.dt_s == 0.0 {
                worst_seed = worst_seed.max(pos_err).max(vel_err);
            } else {
                worst_prop = worst_prop.max(pos_err).max(vel_err);
            }
        }
    }
    // Surfaced so a future tightening of the tolerances is grounded in the
    // observed residuals, not guessed (run with --nocapture).
    println!("worst rel err vs hapsira: seed {worst_seed:.3e}, propagated {worst_prop:.3e}");
}

/// Gated cross-check that the fixture's μ really is what ANISE resolves for the
/// Sun (HANDOFF §6, "pull GM through ANISE") — the pin the whole comparison
/// rests on. Runs only when `ASTEROID_PLANETARY_CONSTANTS` points at a local
/// `.pca` (e.g. `pck11.pca`); skips green otherwise so CI stays offline, mirroring
/// the DE-kernel-gated ephemeris test in `core`.
#[test]
fn sun_gm_matches_fixture() {
    let Ok(pca) = std::env::var("ASTEROID_PLANETARY_CONSTANTS") else {
        eprintln!("ASTEROID_PLANETARY_CONSTANTS unset — skipping Sun GM pin check");
        return;
    };
    let fixture = load();
    let eph = Ephemeris::load(&pca).expect("load planetary constants");
    let anise_mu = eph.sun_gm_m3_s2().expect("resolve Sun GM");

    let rel = (anise_mu - fixture.mu_m3_s2).abs() / fixture.mu_m3_s2;
    assert!(
        rel < MU_TOL,
        "fixture μ {:.12e} != ANISE Sun GM {:.12e} (rel {:.3e}) — the fixture was \
         generated for a different μ; regenerate it or re-probe with probe_sun_gm",
        fixture.mu_m3_s2, anise_mu, rel
    );
}
