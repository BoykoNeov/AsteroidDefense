//! Fly Apophis' **real** orbit-determination covariance through *our* field to
//! its 2029 Earth flyby, and read the b-plane ellipse and impact probability
//! that come out.
//!
//! This is what the SBDB ingestion was for. Everything the Tier-3 layer has said
//! until now — the ellipse, the σ-distance, the probability — rested on
//! `StateCovariance::synthetic_along_track`, an invented covariance for a
//! designed rock. Here every input is JPL's: the elements, the covariance, the
//! non-gravitational `A2`, and the epoch.
//!
//! # The headline, and it is not the probability
//!
//! Measured: the 1σ b-plane ellipse is **18.2 km × 0.48 km**, and our own
//! position residual against JPL over the same arc is **15.1 km**. They are the
//! same size. So the ellipse is an honest statement about JPL's astrometry and
//! says nothing about the physics we do not model — every planet's relativity,
//! and the radial `A1` — which displaces the nominal by just as much. A real
//! covariance does not make a prediction real on its own; it makes the *other*
//! error term visible, because now there is something to compare it against.
//!
//! The probability itself is 0: the nominal crossing sits 19 571 σ outside the
//! capture disc. That is the correct answer — Apophis' 2029 approach is
//! well-determined and it misses — and it is worth saying plainly that this
//! layer producing a real *zero* is the result, not a disappointment.
//!
//! # What would falsify each block
//!
//!   1. The nominal encounter: Apophis' 2029 approach is famously ~38 000 km from
//!      Earth's centre. A perigee far from that means the seed or the field is
//!      wrong, and nothing downstream is worth reading. Measured here: 37 984 km.
//!   2. The b-plane Jacobian's step plateau, re-measured. `uncertainty`'s shipping
//!      steps were tuned for the campaign's own encounter and its docs say the
//!      criterion does not travel; this checks rather than assumes. (It travels:
//!      the columns move by 1e-4 across a 16× range of step.)
//!   3. The ellipse and the probability.
//!   4. Our own dynamical error, measured a year short of the flyby — the number
//!      block 3 has to be read against.
//!
//! # The trap this probe walked into first
//!
//! The reduction epoch was initially fixed three days before closest approach,
//! which puts Apophis 1.5 million km out — **outside** Earth's ~924 000 km sphere
//! of influence. The osculating geocentric hyperbola is not the encounter there,
//! and the reported perigee came out 4 125 km from JPL's published approach. That
//! looked exactly like a dynamical error and was not one. Reducing at the
//! shipping 12-hour lead (253 000 km, inside the sphere) brought it to 16 km.
//! `UNCERTAINTY_REDUCTION_LEAD_SECONDS` is a physical constant, not a numerical
//! preference, and this is what its doc means.
//!
//! Requires kernels; block 4 additionally needs the Apophis `.neo` table, and is
//! skipped with a message when it is absent.
//!
//!   cargo run -p asteroid_core --release --example probe_sbdb_apophis_2029

use std::sync::Arc;

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::forces::relativity::Relativity1PN;
use asteroid_core::forces::yarkovsky::YarkovskyA2;
use asteroid_core::{
    bplane_jacobian_with_steps, closest_approach, tier1_perturber_field, BPlaneBasis,
    BPlaneEncounter, BPlaneUncertainty, Clock, CompositeForce, Dop853, EphemerisPerturber, Epoch,
    FdSteps, SbdbOrbit, ScanOptions, StateVector, UncertaintyError,
    UNCERTAINTY_REDUCTION_LEAD_SECONDS,
};
use nalgebra::Vector2;

/// One AU in metres, and seconds per day — the `A2` unit conversion.
const AU_M: f64 = 1.495_978_707e11;
const DAY_S: f64 = 86_400.0;

/// Apophis' JPL transverse non-gravitational parameter in the term's units
/// (m/s² at 1 AU). SBDB publishes `A2 = −2.902e−14 au/d²`; this is the same
/// constant the Tier-2 capstone uses, and it must be, because the covariance
/// being flown here was fitted *with* it.
const APOPHIS_A2_SI: f64 = -2.902e-14 * AU_M / (DAY_S * DAY_S);

/// Snapshot cadence for the long arc, seconds. Ten days, matching
/// `uncertainty::SAMPLE_CADENCE_DAYS`: a Jacobian is only valid at the cadence
/// its columns converged at, and every run here — nominal and perturbed — uses
/// this one, so the systematic part cancels out of the differences.
const CADENCE_S: f64 = 10.0 * DAY_S;

/// TDB seconds past J2000 of an epoch comfortably after the 2029 encounter — the
/// end of the finding run. 2029-05-13.
const FIND_UNTIL_TDB: f64 = 926_596_000.0;

fn main() {
    let Some(k) = asteroid_core::kernels::resolve() else {
        eprintln!("no kernels found — see `python tools/fetch_kernels.py`");
        std::process::exit(1);
    };
    let (bsp, pca) = k.as_strs();
    let eph = Arc::new(
        Ephemeris::load(bsp)
            .and_then(|e| e.with_constants(pca))
            .expect("load DE pair"),
    );
    let mu_sun = eph.gm_km3_s2(SUN_J2000).expect("sun gm") * 1e9;

    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/apophis.sbdb");
    let orbit = SbdbOrbit::read(path).expect("read the committed Apophis record");

    let epoch0 = orbit.covariance_epoch();
    let sun0 = EphemerisPerturber::new(Arc::clone(&eph), SUN_J2000)
        .state_at(epoch0)
        .expect("sun state");
    let helio = orbit.state_icrf(mu_sun).expect("elements convert");
    let seed = StateVector::new(
        helio.position + sun0.position,
        helio.velocity + sun0.velocity,
    );

    println!("== {} ==", orbit.name());
    println!(
        "  JPL orbit {} ({}), arc {}",
        orbit.orbit_id(),
        orbit.solution_date(),
        orbit.observation_arc()
    );
    println!(
        "  seeded at the covariance epoch, {:.0} s past J2000",
        epoch0.tdb_seconds_past_j2000()
    );
    let residual_vs_jpl = (helio.position - orbit.truth_state_icrf().position).norm();
    println!("  seed reproduces JPL's own state to {residual_vs_jpl:.1} m");

    // The field the covariance was fitted with, as far as we model it: DE440
    // point masses, the Sun's 1PN relativity, and Apophis' own transverse A2.
    let field = || {
        CompositeForce::new()
            .with(Box::new(tier1_perturber_field(&eph).unwrap()))
            .with(Box::new(Relativity1PN::new(
                mu_sun,
                EphemerisPerturber::new(Arc::clone(&eph), SUN_J2000),
            )))
            .with(Box::new(YarkovskyA2::standard(
                APOPHIS_A2_SI,
                EphemerisPerturber::new(Arc::clone(&eph), SUN_J2000),
            )))
    };
    let force = field();
    let dop = Dop853::new().with_tolerances(1e-12, 1e-6);
    let earth = EphemerisPerturber::new(Arc::clone(&eph), EARTH_J2000);

    // Find closest approach on the NOMINAL run only, then freeze the reduction
    // epoch 12 hours ahead of it — `UNCERTAINTY_REDUCTION_LEAD_SECONDS`, and the
    // lead matters for a physical reason, not a numerical one. Reduce too early
    // and the asteroid is outside Earth's ~924 000 km sphere of influence, where
    // the osculating geocentric hyperbola is not the encounter at all: a first
    // pass here reduced three days out (1.5 million km) and reported a perigee
    // 4 125 km from JPL's published approach, which is the Sun's pull on the
    // two-body extrapolation, not a dynamical error.
    let find_span = FIND_UNTIL_TDB - epoch0.tdb_seconds_past_j2000();
    let find_snaps = (find_span / CADENCE_S).ceil() as u32 + 1;
    let finding = Clock::propagate(&dop, &force, epoch0, seed, CADENCE_S, find_snaps)
        .expect("nominal flight past the encounter");
    let ca = closest_approach(
        &finding,
        &earth,
        ScanOptions {
            max_sample_dt: 3600.0,
            time_tol_seconds: 1.0e-3,
            max_distance: Some(5.0e9),
        },
    )
    .expect("scan")
    .expect("Apophis has a 2029 close approach");
    println!(
        "  closest approach found at {:.0} s past J2000, {:.1} km geocentric",
        ca.epoch.tdb_seconds_past_j2000(),
        ca.distance / 1000.0
    );
    let t_reduce = ca
        .epoch
        .shifted_by_seconds(-UNCERTAINTY_REDUCTION_LEAD_SECONDS);
    let span = t_reduce.tdb_seconds_past_j2000() - epoch0.tdb_seconds_past_j2000();
    let n_snap = (span / CADENCE_S).ceil() as u32 + 1;
    println!(
        "  arc to the reduction epoch: {:.2} yr, {n_snap} snapshots at {:.0} d",
        span / (365.25 * DAY_S),
        CADENCE_S / DAY_S
    );
    let mu_earth = eph.gm_km3_s2(EARTH_J2000).expect("earth gm") * 1e9;
    let reduce = |s: StateVector| -> Result<BPlaneEncounter, UncertaintyError> {
        let clock = Clock::propagate(&dop, &force, epoch0, s, CADENCE_S, n_snap).map_err(|e| {
            UncertaintyError::SampleFailed {
                column: None,
                message: e.to_string(),
            }
        })?;
        let st = clock
            .state_at(t_reduce)
            .map_err(|e| UncertaintyError::SampleFailed {
                column: None,
                message: e.to_string(),
            })?;
        let e = earth
            .state_at(t_reduce)
            .map_err(|e| UncertaintyError::SampleFailed {
                column: None,
                message: e.to_string(),
            })?;
        BPlaneEncounter::from_relative_state(
            st.position - e.position,
            st.velocity - e.velocity,
            mu_earth,
            asteroid_core::EARTH_MEAN_RADIUS_M,
        )
        .map_err(|g| UncertaintyError::SampleFailed {
            column: None,
            message: g.to_string(),
        })
    };

    println!("\n-- 1. the nominal 2029 encounter, in our field --");
    let t = std::time::Instant::now();
    let nominal = reduce(seed).expect("nominal reduces");
    println!("  one flight: {:.1} s", t.elapsed().as_secs_f64());
    println!(
        "  |B| = {:.1} km, perigee = {:.1} km from Earth's centre, v_inf = {:.3} km/s",
        nominal.impact_parameter / 1000.0,
        nominal.perigee / 1000.0,
        nominal.v_inf / 1000.0
    );
    println!(
        "  capture radius (focused) = {:.1} km — the disc that would count as an impact",
        nominal.capture_radius / 1000.0
    );
    println!(
        "  JPL's published 2029 approach is ~38 000 km geocentric; ours is \
         {:.0} km, i.e. {:+.0} km",
        nominal.perigee / 1000.0,
        nominal.perigee / 1000.0 - 38_000.0
    );

    println!("\n-- 2. the b-plane Jacobian's step plateau, re-measured here --");
    println!("     (uncertainty.rs's shipping steps were measured on a different");
    println!("      encounter; its own docs say that criterion does not travel)");
    let basis = BPlaneBasis::from_encounter(&nominal);
    let mut columns = Vec::new();
    for factor in [4.0, 2.0, 1.0, 0.5, 0.25] {
        let steps = FdSteps::default().scaled(factor);
        let project = |s: StateVector| -> Result<Vector2<f64>, UncertaintyError> {
            reduce(s).map(|e| basis.project(&e))
        };
        let j = bplane_jacobian_with_steps(seed, steps, project).expect("jacobian");
        println!(
            "  ×{factor:<5} steps ({:8.1} m, {:.2e} m/s): |∂b/∂v_x| = {:.4e} m per m/s",
            steps.position_m,
            steps.velocity_ms,
            j.column(3).norm()
        );
        columns.push((factor, j));
    }
    let reference = columns
        .iter()
        .find(|(f, _)| *f == 1.0)
        .map(|(_, j)| *j)
        .expect("the shipping step is in the sweep");
    for (factor, j) in &columns {
        let mut worst = 0.0_f64;
        for c in 0..6 {
            let denom = reference.column(c).norm().max(1e-30);
            worst = worst.max((j.column(c) - reference.column(c)).norm() / denom);
        }
        println!("  ×{factor:<5} differs from the shipping step by {worst:.2e} relative");
    }

    println!("\n-- 3. the real covariance, mapped --");
    let mapped = orbit.state_covariance_icrf(mu_sun).expect("maps");
    let pos_sigma = (0..3)
        .map(|i| mapped.covariance.matrix()[(i, i)].sqrt())
        .fold(0.0_f64, f64::max);
    println!("  at the covariance epoch: largest position 1σ = {pos_sigma:.1} m");
    let bp = BPlaneUncertainty::from_jacobian(&reference, &mapped.covariance, &nominal, basis);
    let (long, short) = bp.sigma_axes();
    println!(
        "  at the 2029 b-plane: 1σ ellipse {:.1} km × {:.3} km (aspect {:.0}:1)",
        long / 1000.0,
        short / 1000.0,
        long / short.max(f64::MIN_POSITIVE)
    );
    match bp.sigma_distance() {
        Some(d) => println!("  the b-plane origin sits {d:.1} σ from the nominal crossing"),
        None => println!("  σ-distance undefined (degenerate ellipse)"),
    }
    match bp.impact_probability() {
        Ok(p) => println!("  P(impact in 2029) = {p:.3e}"),
        Err(e) => println!("  P(impact) refused: {e}"),
    }

    println!("\n-- 4. and what our own dynamics cost --");
    println!("     (measured a year BEFORE the flyby, against JPL's raw held-out");
    println!("      samples. Not at the reduction epoch: a 1-day state table cannot");
    println!("      resolve an hours-long flyby, and `horizons.rs` measures its own");
    println!("      interpolation error there at 18 885 km. Reading that as our");
    println!("      dynamical error would be reading the table's error, not ours.)");
    let (bodies, _errors) = asteroid_core::horizons::load_all();
    match bodies.iter().find(|n| n.designation() == "99942") {
        Some(neo) => {
            // The last raw sample at least a year short of closest approach, and
            // the raw sample rather than the Hermite interpolation — otherwise
            // this measures our agreement with our own interpolator.
            let cutoff = ca.epoch.tdb_seconds_past_j2000() - 365.0 * DAY_S;
            let idx = (0..neo.len()).rfind(|&i| neo.sample_epoch_tdb(i) <= cutoff);
            match idx.and_then(|i| neo.sample(i).map(|s| (i, s))) {
                Some((i, jpl)) => {
                    let t_check = Epoch::from_tdb_seconds_past_j2000(neo.sample_epoch_tdb(i));
                    let check_span = neo.sample_epoch_tdb(i) - epoch0.tdb_seconds_past_j2000();
                    let snaps = (check_span / CADENCE_S).ceil() as u32 + 1;
                    let clock = Clock::propagate(&dop, &force, epoch0, seed, CADENCE_S, snaps)
                        .expect("re-fly to the check epoch");
                    let ours = clock.state_at(t_check).expect("state");
                    let sun_r = EphemerisPerturber::new(Arc::clone(&eph), SUN_J2000)
                        .state_at(t_check)
                        .expect("sun state");
                    let d = ((ours.position - sun_r.position) - jpl.position).norm();
                    println!(
                        "  {:.2} yr of flight: our position is {:.1} km from JPL's own",
                        check_span / (365.25 * DAY_S),
                        d / 1000.0
                    );
                    println!(
                        "  the 1σ b-plane ellipse is {:.1} km long, and that residual is \
                         {:.1} km.",
                        long / 1000.0,
                        d / 1000.0
                    );
                    println!(
                        "  They are the same size. The ellipse is an honest statement \
                         about JPL's astrometry\n  and says nothing about the physics we \
                         do not model — every planet's relativity and\n  the radial A1 — \
                         which moves the nominal by just as much."
                    );
                }
                None => println!("  the .neo table has no sample a year before the flyby"),
            }
        }
        None => println!("  no apophis.neo table — run `python tools/fetch_kernels.py --neo`"),
    }

    println!(
        "\n  Every number above traces to JPL: the elements, the covariance, the \
         A2, the epoch.\n  The shipping campaign's own probability stays invented, \
         because its rock is."
    );
}
