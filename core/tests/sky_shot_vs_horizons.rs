//! The shot generator's gate: what `asteroid_core::sky_shot` computes about
//! Apophis seen from Mt. Lemmon across its March 2021 approach, against JPL
//! Horizons hour by hour (`fixtures/apophis_2021_g96.txt`, from
//! `pyref/fetch_horizons_night_sky.py`) — position, rate of motion, brightness,
//! altitude and darkness — plus the shot itself: stars in the field, the rock
//! where the projection says, and its drawn error the size the source says.
//!
//! The asteroid is read from the shipped Horizons `.neo` table (the same source
//! the game draws "real" shots from), so the position check also covers that
//! table's interpolation in this window.

use std::f64::consts::PI;

use asteroid_core::astrometry::{separation_rad, unit_of, RAD_TO_ARCSEC};
use asteroid_core::earth_orientation::MT_LEMMON;
use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::epoch::Epoch;
use asteroid_core::horizons::Neo;
use asteroid_core::sky_shot::{
    self, plan_run, shot_seed, sight, take_shot, NightPlan, ShotSpec, Target, G96_SIGMA_DEC_ARCSEC,
    G96_SIGMA_RA_ARCSEC,
};
use asteroid_core::star_catalog::{self, StarCatalog};
use hifitime::{Epoch as HEpoch, TimeScale};

const FIXTURE: &str = include_str!("fixtures/apophis_2021_g96.txt");

/// Apophis' H and G as Horizons states them in the fixture's header line —
/// read from there, not typed, so the two cannot drift apart.
fn h_and_g() -> (f64, f64) {
    let line = FIXTURE
        .lines()
        .find(|l| l.starts_with("# physical:"))
        .unwrap();
    let num = |key: &str| -> f64 {
        let rest = &line[line.find(key).unwrap() + key.len()..];
        rest.split_whitespace().next().unwrap().parse().unwrap()
    };
    (num("H="), num("G="))
}

#[test]
fn the_shipped_h_and_g_are_horizons_own() {
    assert_eq!(h_and_g(), (sky_shot::APOPHIS_H, sky_shot::APOPHIS_G));
}

struct Row {
    jd_tt: f64,
    sun: char,
    ra: f64,
    dec: f64,
    dra_cosd: f64,
    ddec: f64,
    el: f64,
    apmag: f64,
}

fn rows() -> Vec<Row> {
    FIXTURE
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(|l| {
            let f: Vec<&str> = l.split_whitespace().collect();
            let n = |i: usize| f[i].parse::<f64>().unwrap();
            Row {
                jd_tt: n(0),
                sun: f[1].chars().next().unwrap(),
                ra: n(3),
                dec: n(4),
                dra_cosd: n(5),
                ddec: n(6),
                el: n(8),
                apmag: n(9),
            }
        })
        .collect()
}

fn epoch_tt(jd: f64) -> Epoch {
    Epoch::from(HEpoch::from_jde_in_time_scale(jd, TimeScale::TT))
}

struct World {
    eph: Ephemeris,
    apophis: Neo,
}

fn world(what: &str) -> Option<World> {
    let k = asteroid_core::kernels::resolve_for_test(what)?;
    let (bsp, pca) = k.as_strs();
    let eph = Ephemeris::load(bsp)
        .and_then(|e| e.with_constants(pca))
        .expect("load DE pair");
    let bodies = asteroid_core::horizons::load_all_for_test(what);
    if bodies.is_empty() {
        return None;
    }
    let apophis = bodies
        .into_iter()
        .find(|n| n.designation() == "99942")
        .expect("apophis.neo is one of the shipped tables");
    Some(World { eph, apophis })
}

fn target(w: &World) -> Target<'_> {
    let (h, g) = h_and_g();
    Target {
        name: "99942 Apophis".into(),
        h,
        g,
        ssb_m: Box::new(move |t| {
            let helio = w.apophis.helio_state_at(t)?.position;
            let sun = w
                .eph
                .sun_ssb_km(Epoch::from_tdb_seconds_past_j2000(t).as_hifitime())
                .ok()?;
            Some(helio + sun * 1000.0)
        }),
    }
}

/// Position, motion, brightness and altitude, every hour for eleven days.
///
/// Bounds, each from what is left out rather than picked:
/// - **position 5 mas**: the site model is 2–48 m off Horizons
///   (`astrometry_vs_horizons`) and the `.neo` table's 1-day Hermite
///   interpolation a median 24 m; ~70 m at 0.113 au is 0.85 mas.
/// - **rates 0.5″/h**: ours is a ±5 min finite difference of astrometric
///   positions, Horizons' the analytic rate of apparent ones; the difference
///   in the rate of aberration is ~0.02″/h.
/// - **magnitude 0.002**: the same H-G formula; Horizons prints 3 decimals.
/// - **elevation 0.02°**: Horizons' is "airless apparent" — it includes
///   annual aberration (≤ 20.5″), nutation and polar motion that our geometric
///   altitude leaves out; together < 0.01°.
///
/// Measured 2026-10-10, worst of 265 hours: position 0.88 mas, rate 0.28″/h,
/// magnitude 0.0006, elevation 0.006°; all 110 Horizons-night hours dark here.
#[test]
fn apophis_from_mt_lemmon_matches_horizons_hour_by_hour() {
    let Some(w) = world("sky shot vs Horizons") else {
        return;
    };
    let t = target(&w);
    let rows = rows();
    assert_eq!(rows.len(), 265);
    let (mut worst_pos, mut worst_rate, mut worst_mag, mut worst_el) = (0f64, 0f64, 0f64, 0f64);
    let mut dark_checked = 0;
    for r in &rows {
        let e = epoch_tt(r.jd_tt);
        let s = sight(&w.eph, &MT_LEMMON, e, &t).unwrap();

        let horizons = unit_of(r.ra.to_radians(), r.dec.to_radians());
        let pos_mas = separation_rad(&s.sky.unit_icrf, &horizons) * RAD_TO_ARCSEC * 1e3;

        // Rates, arcsec/hour, from ±5 minutes.
        let dt = 300.0;
        let a = sight(&w.eph, &MT_LEMMON, e.shifted_by_seconds(-dt), &t)
            .unwrap()
            .sky;
        let b = sight(&w.eph, &MT_LEMMON, e.shifted_by_seconds(dt), &t)
            .unwrap()
            .sky;
        let mut dra = b.ra_rad - a.ra_rad;
        if dra > PI {
            dra -= 2.0 * PI;
        } else if dra < -PI {
            dra += 2.0 * PI;
        }
        let per_h = 3600.0 / (2.0 * dt) * RAD_TO_ARCSEC;
        let rate_ra = dra * s.sky.dec_rad.cos() * per_h;
        let rate_dec = (b.dec_rad - a.dec_rad) * per_h;
        let rate_err = (rate_ra - r.dra_cosd).hypot(rate_dec - r.ddec);

        let mag_err = (s.v_mag - r.apmag).abs();
        let el_err = (s.target_alt_rad.to_degrees() - r.el).abs();

        worst_pos = worst_pos.max(pos_mas);
        worst_rate = worst_rate.max(rate_err);
        worst_mag = worst_mag.max(mag_err);
        worst_el = worst_el.max(el_err);

        // Darkness: Horizons' flag against our Sun altitude, away from the
        // flag boundaries (its `*` uses the refracted upper limb).
        let sun_deg = s.sun_alt_rad.to_degrees();
        match r.sun {
            '-' => {
                assert!(
                    sun_deg < -17.0,
                    "JD {}: Horizons night, our Sun at {sun_deg:.2}°",
                    r.jd_tt
                );
                dark_checked += 1;
            }
            '*' => assert!(
                sun_deg > -1.5,
                "JD {}: Horizons day, our Sun at {sun_deg:.2}°",
                r.jd_tt
            ),
            'N' => assert!(
                (-13.0..=-5.0).contains(&sun_deg),
                "JD {}: Horizons nautical twilight, our Sun at {sun_deg:.2}°",
                r.jd_tt
            ),
            other => panic!("unexpected solar flag {other}"),
        }
        if sun_deg < -19.0 {
            assert!(s.dark());
        }
    }
    println!(
        "  worst over 265 hours: position {worst_pos:.3} mas, rate {worst_rate:.3}″/h, \
         magnitude {worst_mag:.4}, elevation {worst_el:.4}°  ({dark_checked} dark hours)"
    );
    assert!(worst_pos < 5.0, "position {worst_pos} mas");
    assert!(worst_rate < 0.5, "rate {worst_rate} arcsec/h");
    assert!(worst_mag < 0.002, "magnitude {worst_mag}");
    assert!(worst_el < 0.02, "elevation {worst_el} deg");
}

/// A run of real shots: three nights of three, 20 minutes apart, 60 s
/// exposures. Every shot dark and the rock above 30°; the rock where the
/// projection of its sky position says; the drawn error the published size; the
/// trail the length its rate says.
///
/// Measured 2026-10-10: 6, 8 and 11 Tycho-2 stars (V ≤ 12) in the 0.5° field
/// on the three nights, every one a usable reference; the rock at V 15.56–15.62
/// is **fainter than every star in its field** — Tycho-2 stops at V ≈ 11.5, and
/// that is shown as it is, not brightened. Trails 3.57–3.62″; the drawn error's
/// RMS over 400 seeds 0.307″ / 0.294″ against the published 0.31″ / 0.28″.
#[test]
fn a_run_of_shots_is_dark_high_and_honest() {
    let Some(w) = world("sky shot run") else {
        return;
    };
    let Some(cat_path) = star_catalog::resolve_for_test("sky shot run (stars)") else {
        return;
    };
    let cat = StarCatalog::open(cat_path).unwrap();
    let t = target(&w);
    let plan = NightPlan {
        nights: 3,
        shots_per_night: 3,
        spacing_s: 1200.0,
        exposure_s: 60.0,
        min_alt_deg: 30.0,
        gap_days: 0,
    };
    let start = epoch_tt(2459278.5); // 2021-03-05 00:00 TT
    let nights = plan_run(&w.eph, &MT_LEMMON, &t, start, &plan).unwrap();
    assert_eq!(nights.len(), 3);

    let mut shots = Vec::new();
    for (n, night) in nights.iter().enumerate() {
        let pointing = sight(&w.eph, &MT_LEMMON, night[0], &t)
            .unwrap()
            .sky
            .unit_icrf;
        for (k, &epoch) in night.iter().enumerate() {
            let spec = ShotSpec {
                epoch_mid: epoch,
                exposure_s: plan.exposure_s,
                pointing,
                half_width_arcsec: 900.0,
                star_mag_limit: 12.0,
            };
            let shot = take_shot(
                &w.eph,
                &cat,
                &MT_LEMMON,
                &t,
                &spec,
                shot_seed(2021, n * 10 + k),
            )
            .unwrap();
            assert!(
                shot.sighting.observable(30.0),
                "night {n} shot {k} not observable"
            );
            shots.push(shot);
        }
    }
    for (i, shot) in shots.iter().enumerate() {
        let rock = shot
            .rock
            .expect("the rock stays in a 0.5° field through a night");
        let (x, y) = rock.true_mid;
        let back = shot.plane.deproject(x, y);
        let miss = separation_rad(&back, &shot.sighting.sky.unit_icrf) * RAD_TO_ARCSEC;
        assert!(miss < 1e-6, "shot {i}: {miss} arcsec");
        let trail =
            (rock.drawn_end.0 - rock.drawn_start.0).hypot(rock.drawn_end.1 - rock.drawn_start.1);
        println!(
            "  shot {i}: {} stars ({} references), V {:.2}, trail {trail:.2}″, error ({:+.2}, {:+.2})″",
            shot.stars.len(),
            shot.stars.iter().filter(|s| s.reference).count(),
            rock.v_mag,
            rock.drawn_mid.0 - x,
            rock.drawn_mid.1 - y
        );
        assert!(
            shot.stars.iter().filter(|s| s.reference).count() >= 3,
            "too few references to measure against"
        );
        // ~3.6″/min at this approach (Horizons rate quantity, ~215″/h).
        assert!((2.5..=4.5).contains(&trail), "trail {trail}");
    }

    // The drawn error, over many seeds of one shot, has the published size.
    let spec = shots[0].spec;
    let n = 400;
    let (mut sx, mut sy) = (0.0, 0.0);
    for seed in 0..n {
        let s = take_shot(&w.eph, &cat, &MT_LEMMON, &t, &spec, shot_seed(99, seed)).unwrap();
        let r = s.rock.unwrap();
        sx += (r.drawn_mid.0 - r.true_mid.0).powi(2);
        sy += (r.drawn_mid.1 - r.true_mid.1).powi(2);
    }
    let (rx, ry) = ((sx / n as f64).sqrt(), (sy / n as f64).sqrt());
    println!("  drawn error RMS over {n} seeds: {rx:.3}″ RA, {ry:.3}″ Dec");
    // RMS of n normals: relative standard error 1/sqrt(2n) = 3.5 %; allow 4 of them.
    assert!((rx / G96_SIGMA_RA_ARCSEC - 1.0).abs() < 0.14, "{rx}");
    assert!((ry / G96_SIGMA_DEC_ARCSEC - 1.0).abs() < 0.14, "{ry}");
}
