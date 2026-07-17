//! §10.7 (dop853 batch): the adaptive [`Dop853`] encounter integrator against
//! the analytic Kepler oracle, plus a *controller-contract* check.
//!
//! Why this shape (HANDOFF §6 oracle ladder, and the batch-2a design note): an
//! 8th-order method reaches round-off long before an `h⁸` convergence-order slope
//! can be read, so the N-vs-2N order test that validated RK4
//! (`integrator_convergence.rs`) gives no clean signal here. The verification
//! target therefore shifts:
//!
//!   1. **Oracle match.** A test particle in a single point-mass field at the
//!      frame origin *is* a two-body problem, so [`KeplerPropagator`] is an exact
//!      oracle. Integrating a multi-period arc with `Dop853` and matching it pins
//!      the numbers — a wrong coefficient or a dropped order shows up as a gross
//!      miss, not a conserved-but-wrong drift.
//!   2. **Controller contract.** The genuinely *new* behaviour in this batch is
//!      the adaptive step controller. Sweeping `rtol` and asserting that (a) the
//!      achieved error honours the requested tolerance within a modest factor and
//!      (b) tightening `rtol` monotonically tightens the error *and* costs more
//!      force evaluations tests the machinery RK4 never had.
//!
//! The field attractor sits at the origin, so the barycentric integration frame
//! and the propagator's attractor-relative frame coincide (HANDOFF §5) and the
//! comparison needs no frame transform. μ is pinned identically on both sides.

use asteroid_core::forces::point_mass::{FixedPerturber, PointMassGravity};
use asteroid_core::forces::{ForceError, ForceModel};
use asteroid_core::{
    Dop853, Epoch, Integrator, KeplerPropagator, OrbitalElements, Propagator, StateVector,
};
use nalgebra::Vector3;
use std::sync::atomic::{AtomicU64, Ordering};

/// Heliocentric Sun μ (m³/s²), the ANISE-resolved value the fixtures pin against
/// (`pyref/generate_kepler_fixture.py`). Only its being *identical* on both sides
/// matters here, but using the real value keeps the orbit physical.
const MU_SUN: f64 = 1.327_124_400_419_393_7e20;

/// A `ForceModel` that counts how many times its acceleration is evaluated, so a
/// test can compare the *work* the adaptive controller does at different `rtol`.
/// An atomic gives the interior mutability `acceleration(&self, …)` needs while
/// keeping the counter `Sync`, which `ForceModel` requires of its implementors so
/// that a force field can cross to a worker thread (see the trait's note). This is
/// a decorator — it holds another model by reference — which is precisely the shape
/// that needs the bound. `Relaxed` is right: nothing synchronises on this count,
/// and only its total is read, after the propagation is over.
struct Counting<'a> {
    inner: &'a dyn ForceModel,
    evals: AtomicU64,
}

impl<'a> Counting<'a> {
    fn new(inner: &'a dyn ForceModel) -> Self {
        Self {
            inner,
            evals: AtomicU64::new(0),
        }
    }
    fn count(&self) -> u64 {
        self.evals.load(Ordering::Relaxed)
    }
}

impl ForceModel for Counting<'_> {
    fn acceleration(&self, epoch: Epoch, state: &StateVector) -> Result<Vector3<f64>, ForceError> {
        self.evals.fetch_add(1, Ordering::Relaxed);
        self.inner.acceleration(epoch, state)
    }
}

/// The test orbit: a moderate-eccentricity inclined heliocentric ellipse. Its
/// period anchors the propagation spans below.
fn test_orbit() -> (OrbitalElements, f64) {
    let a = 1.5 * 1.495_978_707e11; // 1.5 AU
    let elems = OrbitalElements::new(a, 0.2, 20.0_f64.to_radians(), 1.1, 0.7, 0.3);
    let period = std::f64::consts::TAU * (a * a * a / MU_SUN).sqrt();
    (elems, period)
}

/// Build the two-body field (attractor at the origin) and the Kepler oracle for
/// the test orbit, sharing μ and the seed epoch.
fn setup(epoch0: Epoch) -> (PointMassGravity, KeplerPropagator, StateVector) {
    let (elems, _) = test_orbit();
    let field = PointMassGravity::new(vec![(MU_SUN, FixedPerturber::at_origin()).into()]);
    let oracle = KeplerPropagator::new(elems, MU_SUN, epoch0).expect("kepler oracle");
    let seed = oracle.state_at(epoch0).expect("seed state");
    (field, oracle, seed)
}

fn rel_err(computed: StateVector, reference: StateVector) -> f64 {
    (computed.position - reference.position).norm() / reference.position.norm()
}

#[test]
fn dop853_tracks_the_two_body_oracle_over_many_periods() {
    let epoch0 = Epoch::from_tdb_seconds_past_j2000(0.0);
    let (field, oracle, seed) = setup(epoch0);
    let (_, period) = test_orbit();
    let dop = Dop853::new().with_tolerances(1e-12, 1e-6);

    // Sample across ~3.3 periods, including a non-integer multiple where the
    // orbital phase is most sensitive to accumulated error.
    let fractions = [0.37, 1.0, 2.5, 3.3];
    let mut worst = 0.0_f64;
    for f in fractions {
        let span = f * period;
        let got = dop.step(&field, epoch0, &seed, span).expect("dop step");
        let truth = oracle
            .state_at(epoch0.shifted_by_seconds(span))
            .expect("oracle state");
        let e = rel_err(got, truth);
        worst = worst.max(e);
        assert!(e < 1e-9, "at {f}P: rel err {e:.3e} vs Kepler oracle");
    }
    println!("dop853 vs Kepler oracle: worst rel err {worst:.3e} over 3.3 periods @ rtol 1e-12");
}

#[test]
fn dop853_max_step_caps_the_substep() {
    // A `max_step` far below what the controller would otherwise choose must force
    // more, smaller sub-steps — so the capped run does strictly more work — while
    // still landing on the oracle. Confirms the cap is honoured, not ignored.
    let epoch0 = Epoch::from_tdb_seconds_past_j2000(0.0);
    let (field, oracle, seed) = setup(epoch0);
    let (_, period) = test_orbit();
    let span = period;
    let truth = oracle
        .state_at(epoch0.shifted_by_seconds(span))
        .expect("oracle state");

    let free = Counting::new(&field);
    let got_free = Dop853::new()
        .with_tolerances(1e-9, 1e-6)
        .step(&free, epoch0, &seed, span)
        .expect("uncapped step");

    let capped_field = Counting::new(&field);
    // Cap at 1/200 of a period: below the controller's natural step here.
    let got_capped = Dop853::new()
        .with_tolerances(1e-9, 1e-6)
        .with_max_step(span / 200.0)
        .step(&capped_field, epoch0, &seed, span)
        .expect("capped step");

    println!(
        "max_step cap: uncapped {} evals, capped {} evals",
        free.count(),
        capped_field.count()
    );
    assert!(
        capped_field.count() > free.count(),
        "cap did not increase work: uncapped {}, capped {}",
        free.count(),
        capped_field.count()
    );
    // Both remain accurate against the oracle.
    assert!(rel_err(got_free, truth) < 1e-7, "uncapped drifted");
    assert!(rel_err(got_capped, truth) < 1e-7, "capped drifted");
}

#[test]
fn dop853_controller_honours_and_ranks_its_tolerance() {
    let epoch0 = Epoch::from_tdb_seconds_past_j2000(0.0);
    let (field, oracle, seed) = setup(epoch0);
    let (_, period) = test_orbit();
    let span = period; // one full revolution
    let truth = oracle
        .state_at(epoch0.shifted_by_seconds(span))
        .expect("oracle state");

    // Increasingly strict relative tolerances.
    let rtols = [1e-6_f64, 1e-9, 1e-12];
    let mut errors = Vec::new();
    let mut evals = Vec::new();
    for &rtol in &rtols {
        let counting = Counting::new(&field);
        let dop = Dop853::new().with_tolerances(rtol, rtol * 1e3);
        let got = dop.step(&counting, epoch0, &seed, span).expect("dop step");
        let e = rel_err(got, truth);
        println!(
            "rtol {rtol:.0e}: achieved rel err {e:.3e}, {} force evals",
            counting.count()
        );
        // (a) The controller honours its contract: achieved error is within a
        // modest factor of the requested tolerance (global error over a full
        // revolution is a small multiple of the per-step local tolerance).
        assert!(
            e < 1e4 * rtol,
            "rtol {rtol:.0e}: achieved {e:.3e} exceeds 1e4×rtol"
        );
        errors.push(e);
        evals.push(counting.count());
    }

    // (b) Tightening rtol must monotonically shrink the achieved error and raise
    // the work done — the defining behaviour of a working adaptive controller.
    for i in 1..rtols.len() {
        assert!(
            errors[i] < errors[i - 1],
            "error not monotone: rtol {:.0e} gave {:.3e}, rtol {:.0e} gave {:.3e}",
            rtols[i - 1],
            errors[i - 1],
            rtols[i],
            errors[i]
        );
        assert!(
            evals[i] > evals[i - 1],
            "work not monotone: rtol {:.0e} used {}, rtol {:.0e} used {}",
            rtols[i - 1],
            evals[i - 1],
            rtols[i],
            evals[i]
        );
    }
}
