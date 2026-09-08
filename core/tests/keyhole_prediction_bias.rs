//! The guard on *which half* of the keyhole map is wrong — roadmap item 4's
//! mechanism thread.
//!
//! A resonant circle folds two claims into one locus, and until 2026-09-08 the
//! repo had measured neither of them against a flown flyby:
//!
//! 1. **the prediction** — that at a b-plane point `p`, the flyby leaves the rock
//!    on the `a'` [`OpikFrame::post_encounter_semi_major_axis`] says it does;
//! 2. **the condition** — that landing on `a' = a_res` is what produces a
//!    resonant return impact `h` years later.
//!
//! Five doors flown at five deflection leads sit 19 to 786 km from their own
//! circles, and six sessions of work narrowed *where* that error is without ever
//! asking *which claim* it belongs to. `probe_keyhole_placement outgoing` asked,
//! by measuring the semi-major axis the propagator actually produces. Four leads
//! on the 3:4, with the `a'` difference converted to a b-plane distance through
//! `∇a'·n̂` — exact rather than a proxy, because the circles *are* the level sets
//! of `a'`, so `∇a'` is normal to them:
//!
//! ```text
//!   lead     door d₀     a_true − a_res     a_true − a'_closed
//!   4383 d   + 19.2 km       −14.2 km            − 33.3 km
//!    900 d   +210.6 km       −15.7 km            −226.6 km
//!    300 d   +648.2 km       −13.9 km            −664.6 km
//!    200 d   +786.0 km       −48.8 km            −838.6 km
//! ```
//!
//! The condition column is **flat at about −15 km** — half a door width, inside
//! the ±41 km error bar of the measurement itself — while the door error runs
//! 19 → 786 km. So the resonance condition is sound at every lead, and the whole
//! ladder lives in the closed form's prediction of the outgoing orbit.
//!
//! Both candidate repairs were run on the same flights and both are dead. The
//! `r ≈ R⊕ₒᵣᵦ` substitution (which `keyhole.rs`'s module doc names as the source
//! of the absolute error) does not order the bias: at the 300 d door the rock is
//! **53 km** from Earth's heliocentric distance while the bias is at its full
//! 665 km. And rebuilding the frame from the deflected flight's own encounter,
//! rather than the nominal one the map uses, makes the prediction **worse** at
//! three of the four leads (−260, −353, −295, −1686 km).
//!
//! What this pins is the split, not a mechanism: the condition holds, the
//! prediction carries the ladder. Requires kernels.

use asteroid_core::{
    aim_at_resonance, along_track_unit, closest_approach, CircleBranch, EphemerisPerturber,
    ImpactorConfig, OpikFrame, RealFieldScenario, Resonance, ScanOptions,
};
use nalgebra::Vector2;

use anise::constants::frames::{EARTH_J2000, SSB_J2000, SUN_J2000};

/// The two extreme doors of the flown 3:4 ladder: `(lead days, Δv m/s, recorded
/// door centre m)`. Both are `same_flight` rows — the Δv is the door's own
/// refined floor, so `d₀` and everything read beside it come from one flight.
/// Mirrored from `FLOWN_34_BY_LEAD` in `core/examples/probe_keyhole_placement.rs`.
const DOORS: &[(f64, f64, f64)] = &[
    (4383.0, 0.216_548_269_9, 19.255e3),
    (200.0, 2.478_471_262_1, 785.988e3),
];

/// How far the flown outgoing orbit may sit from the exact resonance, in b-plane
/// kilometres, before "the condition holds" stops being true. Measured −14.2 and
/// −48.8 km; the limit is set at twice the worse of those, and it must stay well
/// under the 786 km door error it is being contrasted with.
const CONDITION_LIMIT_KM: f64 = 100.0;

/// How big the prediction bias must be at the 200 d door for the contrast to
/// mean anything. Measured 838.6 km — this asks only that it stay several times
/// [`CONDITION_LIMIT_KM`], which is the whole claim.
const BIAS_FLOOR_KM: f64 = 500.0;

/// Where the revolution-mean window opens, days past closest approach. Earlier
/// than this the rock is still inside Earth's residual pull: the CA+10 d sample
/// is ~350 km-equivalent away from every later one.
const SETTLE_DAYS: f64 = 30.0;

/// How many points the revolution mean averages. The osculating value swings
/// ±100 km-equivalent through a revolution; averaging over exactly one kills that
/// periodic term and takes the measurement's own error bar to ~41 km.
const MEAN_SAMPLES: usize = 32;

#[test]
fn the_placement_error_is_in_the_prediction_and_not_in_the_resonance_condition() {
    if asteroid_core::kernels::resolve_for_test("keyhole prediction-bias guard").is_none() {
        return;
    }
    let resonance = Resonance { h: 3, k: 4 };
    let scenario = RealFieldScenario::build(&ImpactorConfig::default()).expect("build");
    let eph = scenario.ephemeris().clone();
    let mu_sun = eph.sun_gm_m3_s2().expect("sun GM");
    let ds = scenario.deflection().expect("deflection");
    let nominal = scenario.nominal_hit(&ds).expect("nominal hit");
    let t_ca = ds
        .nominal_encounter_epoch()
        .expect("epoch")
        .expect("an encounter");
    let (r_km, v_km) = eph
        .state_km_s(EARTH_J2000, SUN_J2000, t_ca.as_hifitime())
        .expect("Earth state");
    let frame = OpikFrame::new(&nominal, r_km * 1e3, v_km * 1e3, mu_sun).expect("frame");
    let a_res = resonance.semi_major_axis_m();
    let circle = frame.resonant_circle(resonance).expect("3:4 in reach");

    let xi = frame.project(&nominal.b_vector).x;
    let aim = aim_at_resonance(&frame, resonance, xi, CircleBranch::Minus).expect("aim");
    let sign = if aim.target.y < 0.0 { -1.0 } else { 1.0 };

    let impact = scenario.impact_epoch();
    let default_lead_days =
        (impact.tdb_seconds_past_j2000() - scenario.epoch0().tdb_seconds_past_j2000()) / 86_400.0;
    let scan = ScanOptions {
        max_sample_dt: 6.0 * 3600.0,
        time_tol_seconds: 1.0e-3,
        max_distance: Some(5.0e8),
    };
    let earth = EphemerisPerturber::new(eph.clone(), EARTH_J2000);

    for &(lead_days, dv, recorded) in DOORS {
        // The campaign's own lead is `epoch0()` exactly, not `impact − 4383 d`:
        // the two differ by hours, and hours are tens of kilometres here.
        let epoch = if (lead_days - default_lead_days).abs() < 1.0 {
            scenario.epoch0()
        } else {
            impact.shifted_by_seconds(-lead_days * 86_400.0)
        };
        let seed = ds.nominal().state_at(epoch).expect("seed");
        let dir = along_track_unit(seed).expect("along-track") * sign;
        let (clock, _) = ds
            .deflected_trajectory(epoch, dir * dv)
            .expect("deflected trajectory");
        let ca = closest_approach(&clock, &earth, scan)
            .expect("scan")
            .expect("an encounter");
        let enc = ca
            .b_plane(frame.mu_earth, nominal.earth_radius)
            .expect("b-plane");
        let p = frame.project(&enc.b_vector);
        let d0 = circle.signed_distance(p);

        let n_hat = (p - Vector2::new(0.0, circle.center_zeta)).normalize();
        let grad_n = frame.gradient_semi_major_axis(p).dot(&n_hat);
        let a_closed = frame.post_encounter_semi_major_axis(p);

        // The flown flight reproduces the recorded door centre. Without this the
        // rest of the row is about some other plan.
        assert!(
            (d0 - recorded).abs() < 1.0e3,
            "lead {lead_days} d: flew to d₀ {:.3} km, not the recorded {:.3} km",
            d0 / 1e3,
            recorded / 1e3
        );
        // The conversion is arithmetic, not a measurement: the circle is the level
        // set, so the map's own a' error *is* d₀. If this fails nothing else on the
        // row may be read.
        let identity = (a_closed - a_res) / grad_n;
        assert!(
            (identity - d0).abs() < 5.0e3,
            "lead {lead_days} d: (a'_closed − a_res)/∇a'·n̂ = {:.3} km against d₀ {:.3} km — \
             the gradient conversion is broken",
            identity / 1e3,
            d0 / 1e3
        );

        // The observable: the mean over one full post-encounter revolution. The
        // osculating value alone wobbles by more than the 200 d signal.
        let t10 = ca.epoch.shifted_by_seconds(10.0 * 86_400.0);
        let seed10 = clock.state_at(t10).expect("state 10 d after CA");
        let onward = scenario
            .propagate_free(t10, seed10, 10.0 * 86_400.0, 50)
            .expect("post-encounter arc");
        let period = std::f64::consts::TAU * (a_closed.powi(3) / mu_sun).sqrt();
        let t_open = ca.epoch.shifted_by_seconds(SETTLE_DAYS * 86_400.0);
        let a_true = (0..MEAN_SAMPLES)
            .map(|i| {
                let t = t_open.shifted_by_seconds(period * i as f64 / MEAN_SAMPLES as f64);
                let st = onward.state_at(t).expect("post-encounter state");
                let (rs_km, vs_km) = eph
                    .state_km_s(SUN_J2000, SSB_J2000, t.as_hifitime())
                    .expect("Sun state");
                let r_h = st.position - rs_km * 1e3;
                let v_h = st.velocity - vs_km * 1e3;
                1.0 / (2.0 / r_h.norm() - v_h.norm_squared() / mu_sun)
            })
            .sum::<f64>()
            / MEAN_SAMPLES as f64;

        let condition_km = (a_true - a_res) / grad_n / 1e3;
        let bias_km = (a_true - a_closed) / grad_n / 1e3;
        println!(
            "lead {lead_days:6.0} d: door d₀ {:+8.1} km | condition (a_true − a_res) {condition_km:+8.1} km | \
             prediction bias (a_true − a'_closed) {bias_km:+8.1} km",
            d0 / 1e3
        );

        assert!(
            condition_km.abs() < CONDITION_LIMIT_KM,
            "lead {lead_days} d: the flown outgoing orbit is {condition_km:.1} km-equivalent from \
             the exact resonance — the return condition is not `a = a_res` after all"
        );
        if lead_days < 300.0 {
            assert!(
                bias_km.abs() > BIAS_FLOOR_KM,
                "lead {lead_days} d: prediction bias {bias_km:.1} km — the contrast this test \
                 exists to pin has gone"
            );
        }
    }
}
