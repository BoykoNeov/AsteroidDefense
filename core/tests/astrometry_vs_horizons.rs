//! The sky-position gate: `asteroid_core::astrometry` against JPL Horizons' own
//! astrometric RA/Dec for Apophis, from the geocentre, on sixteen dates — ten of
//! ordinary geometry (0.11 to 1.9 au) and six across the 2029 flyby (down to
//! 38 000 km, where the rock crosses tens of degrees an hour).
//!
//! The fixture (`fixtures/apophis_astrometry.txt`, from
//! `pyref/fetch_horizons_astrometry.py`) carries Horizons' barycentric state of
//! Apophis at each instant, so this checks the light-time step and the direction
//! it produces — not any table's interpolation. Between `t − τ` and `t` the
//! target is stepped back from that state along a parabola under the Sun's pull
//! (`p₀ + v₀Δt + ½a₀Δt²`, `a₀` from DE440's Sun). The error left is the next
//! order, jerk·τ³/6 ≈ 0.1 m at τ ≈ 840 s, plus the planets' pull that the
//! parabola leaves out (≈ 0.01 m away from Earth, 2 mm at the flyby).
//!
//! **A straight line is not good enough, and that was measured.** The first
//! version stepped linearly on an estimate of the Sun's pull that was 1 000×
//! too small (6 µm/s² written for 6 mm/s²). Its error, `a τ²/2`, is ~2 km at
//! τ ≈ 840 s: the gate read 0.5–1.1 mas off Horizons at every distant date and
//! up to 3.5 km in range — a test artefact that looked exactly like a physics
//! disagreement.
//!
//! What remains is Earth itself: our DE440 geocentre against Horizons' (DE441
//! for planets), plus the TT→TDB conversion. The tolerance is set from the
//! worst case of those, converted to arcseconds at each date's own range — see
//! `TOLERANCE_M`.

use asteroid_core::astrometry::{self, RAD_TO_ARCSEC};
use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::epoch::Epoch;
use hifitime::{Epoch as HEpoch, TimeScale};
use nalgebra::Vector3;

const FIXTURE: &str = include_str!("fixtures/apophis_astrometry.txt");
const AU_KM: f64 = 149_597_870.7;

/// The position error budget at the target, metres, that the angular tolerance
/// is made from (`TOLERANCE_M / range`).
///
/// The parts: DE440 vs DE441 geocentre over 2021-2035 (tens of centimetres to
/// metres), the parabolic step above (≤ 0.2 m), and hifitime's TT→TDB against
/// Horizons' (microseconds × 30 km/s ≈ 0.1 m). 50 m covers all three with a
/// tenfold margin and still means **0.07 mas at 1 au and 0.27″ at the flyby's
/// closest** — a real check either way. The measured residual is printed so the
/// margin can be read off rather than trusted: on 2026-10-10 the worst date used
/// **0.058** of it (≈ 3 m; 0.001–0.002 mas at every distant date, ≤ 0.085 mas
/// across the flyby), and the range agreed to 0.16 m or better.
const TOLERANCE_M: f64 = 50.0;

struct Row {
    jd_tt: f64,
    ra_deg: f64,
    dec_deg: f64,
    delta_au: f64,
    pos_km: Vector3<f64>,
    vel_km_s: Vector3<f64>,
    g96_ra_deg: f64,
    g96_dec_deg: f64,
    g96_delta_au: f64,
}

fn rows() -> Vec<Row> {
    let mut lines = FIXTURE.lines();
    assert_eq!(
        lines.next(),
        Some("# asteroid-astrometry-oracle 2"),
        "fixture header"
    );
    lines
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| {
            let f: Vec<f64> = l
                .split_whitespace()
                .map(|x| x.parse().expect("number"))
                .collect();
            assert_eq!(f.len(), 13, "row: {l}");
            Row {
                jd_tt: f[0],
                ra_deg: f[1],
                dec_deg: f[2],
                delta_au: f[3],
                pos_km: Vector3::new(f[4], f[5], f[6]),
                vel_km_s: Vector3::new(f[7], f[8], f[9]),
                g96_ra_deg: f[10],
                g96_dec_deg: f[11],
                g96_delta_au: f[12],
            }
        })
        .collect()
}

#[test]
fn apophis_matches_horizons_astrometric_radec() {
    let Some(k) = asteroid_core::kernels::resolve_for_test("astrometry vs Horizons") else {
        return;
    };
    let (bsp, pca) = k.as_strs();
    let eph = Ephemeris::load(bsp)
        .and_then(|e| e.with_constants(pca))
        .expect("load DE pair");

    let mu_sun = eph.sun_gm_m3_s2().expect("sun gm");
    let rows = rows();
    assert_eq!(rows.len(), 16);
    println!("  date (JD TT)      range au    miss mas   tol mas   range diff m");
    let mut worst_ratio: f64 = 0.0;
    let mut failures = Vec::new();
    for r in &rows {
        let epoch = Epoch::from(HEpoch::from_jde_in_time_scale(r.jd_tt, TimeScale::TT));
        let t0 = epoch.tdb_seconds_past_j2000();
        let (p0, v0) = (r.pos_km * 1000.0, r.vel_km_s * 1000.0);
        let sun_m = eph.sun_ssb_km(epoch.as_hifitime()).unwrap() * 1000.0;
        let to_sun = sun_m - p0;
        let a0 = to_sun * (mu_sun / to_sun.norm().powi(3));
        let s = astrometry::geocentric(&eph, epoch, |t| {
            let dt = t - t0;
            Some(p0 + v0 * dt + a0 * (0.5 * dt * dt))
        })
        .expect("sky position");

        let horizons = astrometry::unit_of(r.ra_deg.to_radians(), r.dec_deg.to_radians());
        let miss_mas = astrometry::separation_rad(&s.unit_icrf, &horizons) * RAD_TO_ARCSEC * 1e3;
        let range_m = r.delta_au * AU_KM * 1000.0;
        let tol_mas = TOLERANCE_M / range_m * RAD_TO_ARCSEC * 1e3;
        // Horizons prints delta to 14 decimals of an au (1.5 m), so the range is
        // compared at the same tolerance as the direction.
        let range_diff_m = s.range_m - range_m;
        let earth_m = eph.geocenter_ssb_km(epoch.as_hifitime()).unwrap() * 1000.0;
        let elong = astrometry::separation_rad(&s.unit_icrf, &(sun_m - earth_m)).to_degrees();
        println!(
            "  {:.5}  {:>10.6}  {:>10.4}  {:>8.3}  {:>10.2}  elong {elong:6.1}",
            r.jd_tt, r.delta_au, miss_mas, tol_mas, range_diff_m
        );
        worst_ratio = worst_ratio.max(miss_mas / tol_mas);
        if miss_mas >= tol_mas || range_diff_m.abs() >= TOLERANCE_M {
            failures.push(format!(
                "JD {}: {miss_mas:.4} mas (tolerance {tol_mas:.4}), range {range_diff_m:.2} m",
                r.jd_tt
            ));
        }
    }
    println!("  worst miss = {:.3} of the tolerance", worst_ratio);
    assert!(
        failures.is_empty(),
        "off Horizons:
{}",
        failures.join(
            "
"
        )
    );
}

/// The light-time correction is not optional: without it the rock lands far from
/// Horizons' position, by its own motion across the light time. A check that the
/// gate above would catch a missing correction rather than pass it by accident.
#[test]
fn dropping_light_time_misses_horizons_by_arcseconds() {
    let Some(k) = asteroid_core::kernels::resolve_for_test("astrometry light-time control") else {
        return;
    };
    let (bsp, pca) = k.as_strs();
    let eph = Ephemeris::load(bsp)
        .and_then(|e| e.with_constants(pca))
        .expect("load DE pair");
    let r = &rows()[0]; // 2021-03-06, 0.11 au
    let epoch = Epoch::from(HEpoch::from_jde_in_time_scale(r.jd_tt, TimeScale::TT));
    let earth_m = eph.geocenter_ssb_km(epoch.as_hifitime()).unwrap() * 1000.0;
    let geometric = (r.pos_km * 1000.0 - earth_m).normalize();
    let horizons = astrometry::unit_of(r.ra_deg.to_radians(), r.dec_deg.to_radians());
    let miss_arcsec = astrometry::separation_rad(&geometric, &horizons) * RAD_TO_ARCSEC;
    println!("  no light time: {miss_arcsec:.2} arcsec off Horizons");
    assert!(miss_arcsec > 1.0, "{miss_arcsec} arcsec");
}

/// The same sixteen dates from Mt. Lemmon (G96). This measures the site chain in
/// `earth_orientation.rs` against Horizons' full IERS model. The miss is quoted
/// as the **site displacement that would cause it** (`miss × range`), because
/// that is what the simplified chain gets wrong and it does not depend on how
/// far away the rock is.
///
/// The budget, from the terms the chain leaves out: UT1 − UTC ≤ 0.9 s, which is
/// ≤ 354 m of site motion at Mt. Lemmon's 393 m/s; polar motion ≤ 0.5″, ≤ 16 m;
/// truncated nutation and frame bias, ≤ 1 m. So 400 m.
///
/// Measured 2026-10-10: **2 to 48 m** on every date (2–28 m on the 2021-22 dates
/// inside Horizons' measured Earth-orientation data), i.e. 0.01–0.17 mas at
/// ordinary distances and ≤ 0.26″ at the flyby's closest. The last column says
/// why the site is not optional: geocentre and Mt. Lemmon differ by **3–78″**
/// at 0.1–1.9 au and by up to **9°** at the flyby.
#[test]
fn apophis_from_mt_lemmon_matches_horizons() {
    let Some(k) = asteroid_core::kernels::resolve_for_test("topocentric astrometry vs Horizons")
    else {
        return;
    };
    let (bsp, pca) = k.as_strs();
    let eph = Ephemeris::load(bsp)
        .and_then(|e| e.with_constants(pca))
        .expect("load DE pair");
    let mu_sun = eph.sun_gm_m3_s2().expect("sun gm");
    const SITE_TOLERANCE_M: f64 = 400.0;

    println!("  date (JD TT)      range au   miss mas   site-equiv m   geocentric-vs-site arcsec");
    let mut failures = Vec::new();
    for r in &rows() {
        let epoch = Epoch::from(HEpoch::from_jde_in_time_scale(r.jd_tt, TimeScale::TT));
        let t0 = epoch.tdb_seconds_past_j2000();
        let (p0, v0) = (r.pos_km * 1000.0, r.vel_km_s * 1000.0);
        let sun_m = eph.sun_ssb_km(epoch.as_hifitime()).unwrap() * 1000.0;
        let to_sun = sun_m - p0;
        let a0 = to_sun * (mu_sun / to_sun.norm().powi(3));
        let target = |t: f64| {
            let dt = t - t0;
            Some(p0 + v0 * dt + a0 * (0.5 * dt * dt))
        };
        let s = astrometry::topocentric(&eph, &astrometry_site(), epoch, target).unwrap();
        let geo = astrometry::geocentric(&eph, epoch, target).unwrap();

        let horizons = astrometry::unit_of(r.g96_ra_deg.to_radians(), r.g96_dec_deg.to_radians());
        let miss = astrometry::separation_rad(&s.unit_icrf, &horizons);
        let range_m = r.g96_delta_au * AU_KM * 1000.0;
        let site_equiv_m = miss * range_m;
        let parallax = astrometry::separation_rad(&s.unit_icrf, &geo.unit_icrf) * RAD_TO_ARCSEC;
        println!(
            "  {:.5}  {:>10.6}  {:>9.3}  {:>12.1}  {:>12.2}",
            r.jd_tt,
            r.g96_delta_au,
            miss * RAD_TO_ARCSEC * 1e3,
            site_equiv_m,
            parallax
        );
        if site_equiv_m >= SITE_TOLERANCE_M {
            failures.push(format!(
                "JD {}: site-equivalent {site_equiv_m:.1} m",
                r.jd_tt
            ));
        }
    }
    assert!(
        failures.is_empty(),
        "off Horizons:
{}",
        failures.join(
            "
"
        )
    );
}

fn astrometry_site() -> asteroid_core::earth_orientation::Site {
    asteroid_core::earth_orientation::MT_LEMMON
}
