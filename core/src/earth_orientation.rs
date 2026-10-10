//! Where an observatory on the ground is, in the ICRF — for topocentric sky
//! positions.
//!
//! # Why the site matters
//!
//! Seen from an asteroid, Earth's radius spans `R⊕ / Δ`: **8.8″ at 1 au and 88″
//! at 0.1 au**. A real astrometric measurement is good to a few tenths of an
//! arcsecond, so a sky position computed from Earth's *centre* is wrong by tens
//! to hundreds of measurement errors for any shot taken from the ground — and
//! it is wrong with a daily rhythm (the site rides around the axis), which is a
//! real thing an orbit fitter has to model. So the screen's shots come from a
//! named site, and the site's ICRF position comes from here.
//!
//! # The model, and what it leaves out
//!
//! `r_GCRS = Pᵀ · Nᵀ · R₃(−GAST) · r_ITRS`, the classical equinox-based chain:
//!
//! - **Earth rotation angle** from UT1, taken as UTC (hifitime). UT1 − UTC is
//!   kept under 0.9 s by leap seconds but is not predictable years ahead; 0.9 s
//!   is ~350 m of site motion at Mt. Lemmon. That is the largest term left out,
//!   and it is left out on purpose: the screen's dates run years into the
//!   future, where no Earth-orientation data exists (JPL's own runs out in
//!   January 2027 and is a prediction after that).
//! - **GMST** from the ERA by the IAU 2006 polynomial; **GAST** adds the
//!   equation of the equinoxes `Δψ cos ε`.
//! - **Precession**: IAU 2006 `ζ_A, z_A, θ_A` (Capitaine et al. 2003, as adopted
//!   in IERS Conventions 2010, eq. 5.40), to `t³`.
//! - **Nutation**: the 18 largest terms of the IAU 1980 series (Meeus,
//!   *Astronomical Algorithms*, 2nd ed., table 22.A), leaving out terms below
//!   0.006″ — a few milliarcseconds of axis, centimetres of site.
//! - Left out entirely: polar motion (≤ 0.5″, ~15 m of site) and the ICRS frame
//!   bias (23 mas, < 1 m).
//!
//! `core/tests/astrometry_vs_horizons.rs` measures the whole chain against
//! Horizons' topocentric positions from Mt. Lemmon, which use the full IERS
//! model.

use nalgebra::{Matrix3, Vector3};

use crate::epoch::Epoch;

const ARCSEC: f64 = std::f64::consts::PI / (180.0 * 3600.0);

/// A fixed site on the ground, in Earth-fixed cylindrical coordinates (the form
/// both the MPC observatory list and Horizons give).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Site {
    /// Short label, e.g. the MPC observatory code.
    pub code: &'static str,
    /// Human name.
    pub name: &'static str,
    /// East longitude, radians.
    pub east_lon_rad: f64,
    /// Distance from Earth's spin axis, metres.
    pub dxy_m: f64,
    /// Height above the equatorial plane, metres (north positive).
    pub dz_m: f64,
}

/// Mt. Lemmon Survey (MPC code G96), part of the Catalina Sky Survey — one of
/// the most productive near-Earth-asteroid discovery telescopes. Coordinates
/// from JPL Horizons' site header for `G96@399` (fetched 2026-10-10):
/// E-lon 249.2113°, Dxy 5390.20576 km, Dz 3403.44132 km, ITRF93.
pub const MT_LEMMON: Site = Site {
    code: "G96",
    name: "Mt. Lemmon Survey",
    east_lon_rad: 249.2113 * std::f64::consts::PI / 180.0,
    dxy_m: 5_390_205.76,
    dz_m: 3_403_441.32,
};

impl Site {
    /// The site in the Earth-fixed (ITRS) frame, metres.
    pub fn itrs_m(&self) -> Vector3<f64> {
        let (s, c) = self.east_lon_rad.sin_cos();
        Vector3::new(self.dxy_m * c, self.dxy_m * s, self.dz_m)
    }

    /// The site relative to the geocentre, ICRF (GCRS) axes, metres, at `epoch`.
    pub fn gcrs_m(&self, epoch: Epoch) -> Vector3<f64> {
        itrs_to_gcrs(epoch) * self.itrs_m()
    }
}

/// The rotation taking Earth-fixed vectors to ICRF-aligned geocentric ones at
/// `epoch` (the module doc lists what it includes).
pub fn itrs_to_gcrs(epoch: Epoch) -> Matrix3<f64> {
    let h = epoch.as_hifitime();
    // Julian centuries of TT since J2000 — the argument of every series here.
    let t = (h.to_jde_tt_days() - 2_451_545.0) / 36_525.0;
    // UT1 taken as UTC (see the module doc for what that costs).
    let du = h.to_jde_utc_days() - 2_451_545.0;

    let era = std::f64::consts::TAU * (0.779_057_273_264 + 1.002_737_811_911_354_5 * du);
    let gmst = era
        + (0.014_506 + 4_612.156_534 * t + 1.391_581_7 * t * t
            - 0.000_000_44 * t.powi(3)
            - 0.000_029_956 * t.powi(4))
            * ARCSEC;

    let eps_a =
        (84_381.406 - 46.836_769 * t - 0.000_183_1 * t * t + 0.002_003_40 * t.powi(3)) * ARCSEC;
    let (dpsi, deps) = nutation(t);
    let gast = gmst + dpsi * eps_a.cos();

    let zeta =
        (2.650_545 + 2_306.083_227 * t + 0.298_849_9 * t * t + 0.018_018_28 * t.powi(3)) * ARCSEC;
    let z =
        (-2.650_545 + 2_306.077_181 * t + 1.092_734_8 * t * t + 0.018_268_37 * t.powi(3)) * ARCSEC;
    let theta = (2_004.191_903 * t - 0.429_493_4 * t * t - 0.041_822_64 * t.powi(3)) * ARCSEC;

    // Mean-of-J2000 -> mean-of-date, and mean-of-date -> true-of-date, as passive
    // rotations; their transposes carry true-of-date back to the ICRF axes.
    let precession = r3(-z) * r2(theta) * r3(-zeta);
    let nutation = r1(-(eps_a + deps)) * r3(-dpsi) * r1(eps_a);
    precession.transpose() * nutation.transpose() * r3(-gast)
}

/// One IAU 1980 nutation term: multipliers of (D, M, M′, F, Ω), then
/// Δψ = (a + b t) sin, Δε = (c + d t) cos, in 0.0001″.
type NutationTerm = (i8, i8, i8, i8, i8, f64, f64, f64, f64);

/// Nutation in longitude and obliquity, radians: the 18 largest IAU 1980 terms.
fn nutation(t: f64) -> (f64, f64) {
    let deg = std::f64::consts::PI / 180.0;
    // Delaunay-style arguments, Meeus ch. 22.
    let d = (297.850_36 + 445_267.111_480 * t - 0.001_914_2 * t * t + t.powi(3) / 189_474.0) * deg;
    let m = (357.527_72 + 35_999.050_340 * t - 0.000_160_3 * t * t - t.powi(3) / 300_000.0) * deg;
    let mp = (134.962_98 + 477_198.867_398 * t + 0.008_697_2 * t * t + t.powi(3) / 56_250.0) * deg;
    let f = (93.271_91 + 483_202.017_538 * t - 0.003_682_5 * t * t + t.powi(3) / 327_270.0) * deg;
    let om = (125.044_52 - 1_934.136_261 * t + 0.002_070_8 * t * t + t.powi(3) / 450_000.0) * deg;

    // (D, M, M', F, Ω), Δψ = (a + b t) sin(arg), Δε = (c + d t) cos(arg), in 0.0001″.
    #[rustfmt::skip]
    const TERMS: [NutationTerm; 18] = [
        ( 0,  0,  0, 0, 1, -171_996.0, -174.2, 92_025.0,  8.9),
        (-2,  0,  0, 2, 2,  -13_187.0,   -1.6,  5_736.0, -3.1),
        ( 0,  0,  0, 2, 2,   -2_274.0,   -0.2,    977.0, -0.5),
        ( 0,  0,  0, 0, 2,    2_062.0,    0.2,   -895.0,  0.5),
        ( 0,  1,  0, 0, 0,    1_426.0,   -3.4,     54.0, -0.1),
        ( 0,  0,  1, 0, 0,      712.0,    0.1,     -7.0,  0.0),
        (-2,  1,  0, 2, 2,     -517.0,    1.2,    224.0, -0.6),
        ( 0,  0,  0, 2, 1,     -386.0,   -0.4,    200.0,  0.0),
        ( 0,  0,  1, 2, 2,     -301.0,    0.0,    129.0, -0.1),
        (-2, -1,  0, 2, 2,      217.0,   -0.5,    -95.0,  0.3),
        (-2,  0,  1, 0, 0,     -158.0,    0.0,      0.0,  0.0),
        (-2,  0,  0, 2, 1,      129.0,    0.1,    -70.0,  0.0),
        ( 0,  0, -1, 2, 2,      123.0,    0.0,    -53.0,  0.0),
        ( 2,  0,  0, 0, 0,       63.0,    0.0,      0.0,  0.0),
        ( 0,  0,  1, 0, 1,       63.0,    0.1,    -33.0,  0.0),
        ( 2,  0, -1, 2, 2,      -59.0,    0.0,     26.0,  0.0),
        ( 0,  0, -1, 0, 1,      -58.0,   -0.1,     32.0,  0.0),
        ( 0,  0,  1, 2, 1,      -51.0,    0.0,     27.0,  0.0),
    ];
    let (mut dpsi, mut deps) = (0.0, 0.0);
    for &(cd, cm, cmp, cf, com, a, b, c, dd) in &TERMS {
        let arg = cd as f64 * d + cm as f64 * m + cmp as f64 * mp + cf as f64 * f + com as f64 * om;
        dpsi += (a + b * t) * arg.sin();
        deps += (c + dd * t) * arg.cos();
    }
    (dpsi * 1e-4 * ARCSEC, deps * 1e-4 * ARCSEC)
}

/// Passive rotations about the x, y and z axes.
fn r1(a: f64) -> Matrix3<f64> {
    let (s, c) = a.sin_cos();
    Matrix3::new(1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c)
}
fn r2(a: f64) -> Matrix3<f64> {
    let (s, c) = a.sin_cos();
    Matrix3::new(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c)
}
fn r3(a: f64) -> Matrix3<f64> {
    let (s, c) = a.sin_cos();
    Matrix3::new(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tt_minus_utc_s(epoch: Epoch) -> f64 {
        let h = epoch.as_hifitime();
        (h.to_jde_tt_days() - h.to_jde_utc_days()) * 86_400.0
    }

    #[test]
    fn the_rotation_is_orthonormal() {
        let m = itrs_to_gcrs(Epoch::from_tdb_gregorian(2029, 4, 13, 21, 45, 0, 0));
        assert!((m * m.transpose() - Matrix3::identity()).norm() < 1e-14);
        assert!((m.determinant() - 1.0).abs() < 1e-14);
    }

    /// At J2000 precession is (nearly) zero, so the Greenwich meridian's ICRF
    /// right ascension is the **mean** sidereal time — not the apparent: the
    /// equation of the equinoxes that GAST adds is undone by `Nᵀ` carrying the
    /// meridian back to the mean equator. GMST at 2000-01-01 12:00 UT1 is the
    /// published 18h 41m 50.54841s = 280.460 618 37° (Astronomical Almanac,
    /// IAU 1982 definition). An error in GAST, `Nᵀ` or their signs shows up here
    /// as the ~13″ equation of the equinoxes.
    #[test]
    fn greenwich_sits_at_the_published_sidereal_time_at_j2000() {
        let epoch = Epoch::from(hifitime::Epoch::from_gregorian_utc(2000, 1, 1, 12, 0, 0, 0));
        let x = itrs_to_gcrs(epoch) * Vector3::new(1.0, 0.0, 0.0);
        let ra_deg = x.y.atan2(x.x).to_degrees().rem_euclid(360.0);
        let off_arcsec = (ra_deg - 280.460_618_37) * 3600.0;
        // Measured 0.0147″: the IAU 2006 GMST's constant 0.014506″ term.
        assert!(off_arcsec.abs() < 0.05, "{off_arcsec} arcsec");
    }

    #[test]
    fn tt_minus_utc_is_the_leap_second_count_plus_32_184() {
        let epoch = Epoch::from(hifitime::Epoch::from_gregorian_utc(
            2026, 10, 10, 0, 0, 0, 0,
        ));
        assert!((tt_minus_utc_s(epoch) - 69.184).abs() < 1e-3);
    }
}
