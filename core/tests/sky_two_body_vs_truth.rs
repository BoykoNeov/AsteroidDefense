//! How wrong is a **two-body** orbit on the sky? The question step 5 of the
//! sky-observation plan asks before choosing how the player's trial orbit is
//! predicted (`docs/plans/2026-10-10-sky-observation-screen.md`).
//!
//! A trial orbit is six numbers — heliocentric Kepler elements — and the
//! cheapest prediction from them ignores every planet. Apophis in March 2021 is
//! 0.11 au from Earth, so Earth's pull is the largest thing that leaves out. The
//! test takes JPL's own state of Apophis at the middle of the observing run,
//! turns it into the osculating ellipse (the two-body orbit that matches it
//! exactly at that instant), and measures how far that ellipse's sky position
//! drifts from JPL's as the time from the matching instant grows — against the
//! 0.3″ the shots are measured to.
//!
//! **Measured 2026-10-10:** 0.008″ over the screen's ±1.5 days (40× under the
//! measurement error), 0.045″ at ±3 days, **1.0″ at ±10 days and 13.6″ at ±30
//! days**. So a two-body trial orbit is honest for the three-night run the screen
//! shows, and is *not* for an arc of weeks: there Earth's pull is already
//! several measurement errors, and a fit over such an arc must use the field.
//!
//! The ellipse is the *best possible* two-body orbit only at its own epoch; a
//! fitted one could absorb some of the drift. So this is an upper bound on what
//! two-body prediction costs a fit — and an exact statement of what it costs a
//! player who dials in the true osculating elements.

use asteroid_core::astrometry::{separation_rad, RAD_TO_ARCSEC};
use asteroid_core::earth_orientation::MT_LEMMON;
use asteroid_core::elements::OrbitalElements;
use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::epoch::Epoch;
use asteroid_core::frames::icrf_to_ecliptic;
use asteroid_core::propagator::KeplerPropagator;
use asteroid_core::sky_shot::{sight, Target, APOPHIS_G, APOPHIS_H};
use asteroid_core::state::StateVector;

#[test]
fn a_two_body_orbit_drifts_from_apophis_on_the_sky() {
    let what = "two-body vs truth on the sky";
    let Some(k) = asteroid_core::kernels::resolve_for_test(what) else {
        return;
    };
    let (bsp, pca) = k.as_strs();
    let eph = Ephemeris::load(bsp)
        .and_then(|e| e.with_constants(pca))
        .expect("load DE pair");
    let bodies = asteroid_core::horizons::load_all_for_test(what);
    if bodies.is_empty() {
        return;
    }
    let apophis = bodies
        .iter()
        .find(|n| n.designation() == "99942")
        .expect("apophis.neo");
    let mu_sun = eph.sun_gm_m3_s2().unwrap();

    let truth = Target {
        name: "truth".into(),
        h: APOPHIS_H,
        g: APOPHIS_G,
        ssb_m: Box::new(|t| {
            let helio = apophis.helio_state_at(t)?.position;
            let sun = eph
                .sun_ssb_km(Epoch::from_tdb_seconds_past_j2000(t).as_hifitime())
                .ok()?;
            Some(helio + sun * 1000.0)
        }),
    };

    // The middle of the screen's run: 2021-03-06 06:00 UTC (night 2).
    let mid = Epoch::from_utc_gregorian(2021, 3, 6, 6, 0, 0);
    let s = apophis
        .helio_state_at(mid.tdb_seconds_past_j2000())
        .unwrap();
    let ecl = StateVector::new(icrf_to_ecliptic(s.position), icrf_to_ecliptic(s.velocity));
    let el = OrbitalElements::from_state(ecl, mu_sun).unwrap();
    println!(
        "  osculating at 2021-03-06: a {:.5} au  e {:.5}  i {:.4}°",
        el.semi_major_axis / 1.495_978_707e11,
        el.eccentricity,
        el.inclination.to_degrees()
    );
    let kepler = KeplerPropagator::new(el, mu_sun, mid).unwrap();
    let model = Target::two_body("two-body", APOPHIS_H, APOPHIS_G, kepler, &eph);

    println!("  days from match   drift on the sky");
    let mut drift = Vec::new();
    for &days in &[0.0, 0.5, 1.0, 1.5, 3.0, 10.0, 30.0] {
        let mut worst: f64 = 0.0;
        for sign in [-1.0, 1.0] {
            let e = mid.shifted_by_seconds(sign * days * 86_400.0);
            let a = sight(&eph, &MT_LEMMON, e, &truth).unwrap().sky.unit_icrf;
            let b = sight(&eph, &MT_LEMMON, e, &model).unwrap().sky.unit_icrf;
            worst = worst.max(separation_rad(&a, &b) * RAD_TO_ARCSEC);
        }
        println!("  {days:>6.1}            {worst:>10.3}″");
        drift.push((days, worst));
    }
    // At the matching instant the two agree to the state round trip.
    assert!(drift[0].1 < 1e-3, "{}", drift[0].1);
    // The verdict the step needs, stated as what was measured on 2026-10-10.
    let three_nights = drift.iter().find(|d| d.0 == 1.5).unwrap().1;
    let month = drift.iter().find(|d| d.0 == 30.0).unwrap().1;
    println!("  over the screen's 3 nights: {three_nights:.3}″; over ±30 days: {month:.1}″");
    // Guards on the measured verdict, each with a factor-of-few margin: the run's
    // arc must stay far under the 0.3″ error, and a month must stay far over it
    // (if it ever does not, the two-body choice needs re-measuring, not trusting).
    assert!(three_nights < 0.05, "{three_nights}");
    assert!(month > 3.0, "{month}");
}
