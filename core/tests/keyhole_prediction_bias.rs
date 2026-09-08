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
//! The condition column sits at **−14 to −16 km at three of the four leads** and
//! −49 km at 200 d — half a door width, and a spread inside the ±41 km error bar
//! of the measurement itself — while the door error runs 19 → 786 km. So the
//! resonance condition is sound at every lead, and the whole ladder lives in the
//! closed form's prediction of the outgoing orbit.
//!
//! The 200 d row's −49 is real rather than the convention wobbling: reopening the
//! averaging window 30 days later moves all four rows by the same ~−21 km, and it
//! stays ~33 km below the other three. The window's opening carries a systematic;
//! comparisons between rows do not.
//!
//! Both candidate repairs were run on the same flights and both are dead. The
//! `r ≈ R⊕ₒᵣᵦ` substitution (which `keyhole.rs`'s module doc names as the source
//! of the absolute error) does not order the bias: at the 300 d door the rock is
//! **53 km** from Earth's heliocentric distance while the bias is at its full
//! 665 km. And rebuilding the frame from the deflected flight's own encounter,
//! rather than the nominal one the map uses, makes the prediction **worse** at
//! three of the four leads (−260, −353, −295, −1686 km).
//!
//! # Where the ladder actually is (added 2026-09-08, the same day)
//!
//! Splitting the prediction from the condition left "the prediction is wrong" as
//! a whole. It is not a whole. The construction makes the same claim twice —
//! `OpikFrame::incoming_semi_major_axis` is the identical arithmetic with the
//! incoming asymptote in place of the outgoing one — so asking it on **both legs
//! of the same flight** separates *which orbit the rock arrives on* from *what the
//! flyby does to it*:
//!
//! ```text
//!   lead     door d₀    baseline err    outgoing err    THE CHANGE    door on the change
//!   4383 d   + 19.2 km    +292.8 km       − 33.3 km      −326.1 km        +311.9 km
//!    200 d   +786.0 km    −457.4 km       −838.6 km      −381.2 km        +328.6 km
//!   apart      766.8         750.2           805.2          55.1              16.6
//! ```
//!
//! The two absolute errors run a ladder 750 and 805 km wide across the two extreme
//! doors. The error in the **change**, which is the only part the encounter itself
//! owns, is 55 km apart — inside the ±136 km bar of the incoming measurement. So
//! the flyby is predicted correctly to a constant, and the whole ladder is the
//! construction misjudging the orbit the deflected rock arrives on. (The four-lead
//! table in `probe_keyhole_placement outgoing` fills in 900 d and 300 d, where the
//! change reads −310.4 and −242.3 km.)
//!
//! The right-hand column is why that matters: draw the circle where `a' − a_in`
//! equals `a_res − a_in_true` and the same two doors land 16.6 km apart instead of
//! 766.8. That repair is **measured, not shipped** — one resonance, four leads, and
//! a residual spread at the measurement's own noise floor.
//!
//! What this pins is the whole decomposition: the condition holds, the turn is a
//! constant, and the baseline carries the ladder. Requires kernels.

use asteroid_core::{
    aim_at_resonance, along_track_unit, closest_approach, CircleBranch, EphemerisPerturber, Epoch,
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
///
/// **A day count is the wrong unit for that and is used anyway**, because the
/// probe's arc is sampled on a day cadence. What the physics cares about is how
/// far out of Earth's grip the reading is taken, and 30 days buys that only at
/// *this* rock's ~5 km/s: a slower flyby would open the window nearer in and
/// quietly re-import the contamination. So the test asserts the distance rather
/// than trusting the days — see [`SETTLE_HILL_RADII`].
const SETTLE_DAYS: f64 = 30.0;

/// How far out of Earth's grip the window must open, in Earth Hill radii. The
/// contaminated CA+10 d sample sits at 4.4; these flights reach ~13 by
/// [`SETTLE_DAYS`]. The gate is set at 10 — below the measured value, above the
/// one known to be wrong — so it fails loudly on an encounter slow enough to need
/// a longer wait instead of returning a number that looks like the others.
const SETTLE_HILL_RADII: f64 = 10.0;

/// Earth's Hill radius, metres — 0.01 AU to two figures. A scale for the gate
/// above, nothing else.
const EARTH_HILL_RADIUS_M: f64 = 1.5e9;

/// How many points the revolution mean averages. The osculating value swings
/// ±100 km-equivalent through a revolution; averaging over exactly one kills that
/// periodic term and takes the measurement's own error bar to ~41 km.
const MEAN_SAMPLES: usize = 32;
/// How far apart the error in the **change** of semi-major axis may be, in
/// b-plane kilometres, at the two extreme doors before "the flyby's own error is
/// a constant" stops being true. Measured −326.1 km at 4383 d against −381.1 km
/// at 200 d, i.e. **55.0 km apart**, while the two *absolute* errors are 805 km
/// apart. The limit is set at 150 — comfortably under a third of the contrast it
/// is being read against, and above the ±136 km bar of the measurement.
const TURN_SPREAD_LIMIT_KM: f64 = 150.0;

/// How far apart the *absolute* errors must stay for that contrast to mean
/// anything. Measured **750 km** on the incoming leg and **805 km** on the
/// outgoing one, against a 55 km spread in the change. If this floor is ever not
/// met the two doors no longer bracket a ladder and nothing above is a contrast.
const ABSOLUTE_SPREAD_FLOOR_KM: f64 = 500.0;

/// The incoming revolution mean's own error bar, in b-plane kilometres — the same
/// mean taken over a revolution closing half a revolution earlier. Measured 133.6
/// to 136.5 km across the four flown doors, **three times** the outgoing leg's
/// ±41 km. That is why the ~100 km residual left after the repair is reported as
/// *at* this measurement's noise floor rather than as a resolved number.
const INCOMING_BAR_LIMIT_KM: f64 = 250.0;

/// One row of the two-door contrast, in b-plane kilometres throughout.
struct Row {
    lead_days: f64,
    /// The door's distance from the circle the map draws today.
    d0: f64,
    /// `a_in_true − a_in_closed`: how wrong the construction is about the orbit
    /// the rock arrives on.
    incoming: f64,
    /// `a_out_true − a'_closed`: how wrong it is about the orbit the flyby leaves.
    outgoing: f64,
    /// The difference of those two — the error in the *change* across the
    /// encounter, which is the only part the flyby itself owns.
    turn: f64,
    /// Where the door would sit if the circle were placed on that change instead
    /// of on the absolute `a'`.
    repaired: f64,
}

#[test]
fn the_condition_holds_the_ladder_is_in_the_baseline_and_the_turn_is_a_constant() {
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
    let mut rows: Vec<Row> = Vec::new();

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
        let hill_radii = (onward
            .state_at(t_open)
            .expect("state at the window")
            .position
            - earth.state_at(t_open).expect("Earth").position)
            .norm()
            / EARTH_HILL_RADIUS_M;
        assert!(
            hill_radii > SETTLE_HILL_RADII,
            "lead {lead_days} d: the mean's window opens {hill_radii:.1} Hill radii out, under the \
             {SETTLE_HILL_RADII} this reading needs — SETTLE_DAYS is calibrated to a v∞ this \
             encounter does not have, and the answer would carry Earth's residual pull"
        );
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

        // ---- The same construction, asked on the leg BEFORE the encounter ----
        //
        // `incoming_semi_major_axis` is the identical arithmetic — `V⊕ + v∞·Ŝ`,
        // vis-viva at Earth's heliocentric distance — with the incoming asymptote
        // in place of the outgoing one. Asking it on both legs of the same flight
        // splits the prediction error into a *baseline* (which orbit the rock
        // arrives on) and a *turn* (what the flyby does to it), and only the
        // second is anything the encounter owns.
        //
        // The observable mirrors the outgoing one exactly: the window **closes**
        // at CA − `SETTLE_DAYS` and runs one full revolution backward, the same
        // `MEAN_SAMPLES` points, the same half-revolution shift as its error bar,
        // and the same Hill-radii gate. Mirrored on purpose, so that the
        // difference of the two means is a difference of like for like.
        //
        // At the 200 d door that backward arc runs past the impulse epoch, onto a
        // continuation the rock never flew. That is deliberate and it is the right
        // observable: what is wanted is the orbit the rock **is on** as it
        // arrives, which is a property of its state at CA − 30 d and not of how it
        // got there.
        let a_in_closed = frame.incoming_semi_major_axis();
        let t_in = ca.epoch.shifted_by_seconds(-SETTLE_DAYS * 86_400.0);
        let seed_in = clock.state_at(t_in).expect("state before CA");
        let hill_in = (seed_in.position - earth.state_at(t_in).expect("Earth").position).norm()
            / EARTH_HILL_RADIUS_M;
        assert!(
            hill_in > SETTLE_HILL_RADII,
            "lead {lead_days} d: the incoming mean's window closes {hill_in:.1} Hill radii out, under the {SETTLE_HILL_RADII} this reading needs"
        );
        let back = scenario
            .propagate_free(t_in, seed_in, -10.0 * 86_400.0, 50)
            .expect("pre-encounter arc");
        // The period comes from the *incoming* orbit: a window sized by the
        // outgoing revolution is not a revolution of the orbit being averaged.
        let period_in = std::f64::consts::TAU * (a_in_closed.powi(3) / mu_sun).sqrt();
        let mean_in = |close: Epoch| {
            (0..MEAN_SAMPLES)
                .map(|i| {
                    let t = close.shifted_by_seconds(-period_in * i as f64 / MEAN_SAMPLES as f64);
                    let st = back.state_at(t).expect("pre-encounter state");
                    let (rs_km, vs_km) = eph
                        .state_km_s(SUN_J2000, SSB_J2000, t.as_hifitime())
                        .expect("Sun state");
                    let r_h = st.position - rs_km * 1e3;
                    let v_h = st.velocity - vs_km * 1e3;
                    1.0 / (2.0 / r_h.norm() - v_h.norm_squared() / mu_sun)
                })
                .sum::<f64>()
                / MEAN_SAMPLES as f64
        };
        let a_in_true = mean_in(t_in);
        let in_bar_km = (a_in_true - mean_in(t_in.shifted_by_seconds(-period_in / 2.0))).abs()
            / grad_n.abs()
            / 1e3;
        assert!(
            in_bar_km < INCOMING_BAR_LIMIT_KM,
            "lead {lead_days} d: the incoming revolution mean's own error bar is {in_bar_km:.1} km, over the {INCOMING_BAR_LIMIT_KM} that licenses reading it — the window is not one revolution or the arc has not settled"
        );
        let incoming_km = (a_in_true - a_in_closed) / grad_n / 1e3;
        let turn_km = bias_km - incoming_km;
        // Placing the circle on the change is a pure shift of it along the normal
        // by the baseline error, so the repaired door needs no re-solve.
        let repaired_km = d0 / 1e3 + incoming_km;
        println!(
            "lead {lead_days:6.0} d: baseline (a_in_true − a_in_closed) {incoming_km:+8.1} km ±{in_bar_km:.0} | turn (the change) {turn_km:+8.1} km | door on the change {repaired_km:+8.1} km"
        );
        rows.push(Row {
            lead_days,
            d0: d0 / 1e3,
            incoming: incoming_km,
            outgoing: bias_km,
            turn: turn_km,
            repaired: repaired_km,
        });
    }

    // ---- The two-door contrast, which is what this file now pins ----
    //
    // The absolute errors run a ladder; the error in the change across the
    // encounter does not. Asserted as a *contrast* rather than as two separate
    // tolerances, because either half alone would pass on a run where the whole
    // measurement had gone flat.
    assert_eq!(rows.len(), DOORS.len(), "a door failed to fly");
    let (lo, hi) = (&rows[0], &rows[1]);
    let apart = |a: f64, b: f64| (a - b).abs();
    println!(
        "\n{:.0} d vs {:.0} d: door {:.1} km apart | baseline {:.1} km apart | outgoing {:.1} km apart | THE CHANGE {:.1} km apart | repaired door {:.1} km apart",
        lo.lead_days,
        hi.lead_days,
        apart(lo.d0, hi.d0),
        apart(lo.incoming, hi.incoming),
        apart(lo.outgoing, hi.outgoing),
        apart(lo.turn, hi.turn),
        apart(lo.repaired, hi.repaired)
    );
    assert!(
        apart(lo.d0, hi.d0) > ABSOLUTE_SPREAD_FLOOR_KM,
        "the two doors are only {:.1} km apart — there is no ladder here to explain",
        apart(lo.d0, hi.d0)
    );
    assert!(
        apart(lo.incoming, hi.incoming) > ABSOLUTE_SPREAD_FLOOR_KM,
        "the baseline error is only {:.1} km apart at the two extremes — it is not what carries the ladder after all",
        apart(lo.incoming, hi.incoming)
    );
    assert!(
        apart(lo.outgoing, hi.outgoing) > ABSOLUTE_SPREAD_FLOOR_KM,
        "the outgoing error is only {:.1} km apart at the two extremes",
        apart(lo.outgoing, hi.outgoing)
    );
    assert!(
        apart(lo.turn, hi.turn) < TURN_SPREAD_LIMIT_KM,
        "the error in the *change* across the encounter is {:.1} km apart at the two extremes, over the {TURN_SPREAD_LIMIT_KM} that makes it a constant — the flyby itself is carrying part of the ladder after all",
        apart(lo.turn, hi.turn)
    );
    assert!(
        apart(lo.repaired, hi.repaired) < TURN_SPREAD_LIMIT_KM,
        "drawing the circle on the change leaves the two doors {:.1} km apart — the repair does not flatten the ladder",
        apart(lo.repaired, hi.repaired)
    );
}
