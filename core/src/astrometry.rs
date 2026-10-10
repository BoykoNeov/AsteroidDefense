//! Where a body appears in the sky — astrometric right ascension and declination.
//!
//! The first step of the sky-observation screen
//! (`docs/plans/2026-10-10-sky-observation-screen.md`): turn a target's
//! barycentric trajectory and an observer's barycentric position into the
//! direction a telescope would record, in the same frame as a star catalog.
//!
//! # Astrometric, and why nothing more
//!
//! A star catalog (Tycho-2 here) gives **astrometric** positions: ICRF
//! directions with no annual aberration and no gravitational light deflection —
//! those are applied by an observer to *every* object in the field alike, so
//! they cancel when a moving body is measured against the stars around it. The
//! target here gets exactly one correction, the one that does *not* cancel:
//! **light time**. The rock is seen where it was when the light left it, and the
//! stars are effectively infinitely far, so this is the only term that moves the
//! rock relative to its background.
//!
//! JPL Horizons publishes the same quantity ("quantity 1, astrometric RA/Dec …
//! compensated for down-leg light-time delay aberration"), which is what makes it
//! the oracle: `core/tests/astrometry_vs_horizons.rs` reproduces Horizons'
//! own numbers for Apophis on sixteen dates, six of them across the 2029 flyby.
//! Adding aberration here would put the rock up to 20″ away from where it sits
//! among the catalog stars.
//!
//! # The light-time iteration
//!
//! `τ = |r_target(t − τ) − r_observer(t)| / c`, solved by fixed-point iteration
//! from `τ = 0`. Each pass shrinks the error by about `v_radial / c` (≈ 10⁻⁴ for
//! a near-Earth asteroid), so it converges to a nanosecond in three or four
//! passes. The observer is held at the reception time — it is the target's light
//! that travels, not the observer's.

use nalgebra::Vector3;

use crate::earth_orientation::Site;
use crate::ephemeris::{Ephemeris, EphemerisError};
use crate::epoch::Epoch;
use crate::forces::relativity::SPEED_OF_LIGHT_M_S;

/// The iteration stops once one pass moves the light time by less than this.
/// A nanosecond is ~30 µm of the target's motion — far below anything drawn or
/// measured.
const LIGHT_TIME_TOLERANCE_S: f64 = 1e-9;

/// A bound on passes, so a target function that disagrees with itself cannot spin
/// forever. Convergence takes three or four (see the module doc); twenty failing
/// means the target is moving at a sizeable fraction of `c`, i.e. the input is
/// wrong.
const LIGHT_TIME_MAX_ITERATIONS: usize = 20;

/// Where a target appears from an observer at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkyPosition {
    /// Right ascension, radians, in `[0, 2π)`.
    pub ra_rad: f64,
    /// Declination, radians, in `[−π/2, π/2]`.
    pub dec_rad: f64,
    /// Observer → target distance at the light-emission instant, metres (what
    /// Horizons calls the light-time-aberrated range, "delta").
    pub range_m: f64,
    /// The light time `τ`, seconds: the target is seen as it was at `t − τ`.
    pub light_time_s: f64,
    /// The same direction as a unit vector, ICRF.
    pub unit_icrf: Vector3<f64>,
}

/// Why a sky position could not be formed.
#[derive(Debug, Clone, PartialEq)]
pub enum AstrometryError {
    /// The target function had no position at this TDB instant (outside its
    /// table, say). Reported, never replaced by a zero vector — a failed lookup
    /// drawn at the origin is the Sun.
    TargetUnavailable { tdb_seconds: f64 },
    /// The observer and target coincide, so there is no direction.
    ZeroRange,
    /// The light-time iteration did not settle in
    /// [`LIGHT_TIME_MAX_ITERATIONS`] passes.
    NotConverged { last_step_s: f64 },
    /// The observer's position could not be read from the ephemeris.
    Ephemeris(String),
}

impl std::fmt::Display for AstrometryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TargetUnavailable { tdb_seconds } => {
                write!(f, "target has no position at TDB {tdb_seconds} s")
            }
            Self::ZeroRange => write!(f, "observer and target coincide"),
            Self::NotConverged { last_step_s } => {
                write!(
                    f,
                    "light-time iteration did not converge (last step {last_step_s} s)"
                )
            }
            Self::Ephemeris(e) => write!(f, "observer position: {e}"),
        }
    }
}

impl std::error::Error for AstrometryError {}

impl From<EphemerisError> for AstrometryError {
    fn from(e: EphemerisError) -> Self {
        Self::Ephemeris(e.to_string())
    }
}

/// The astrometric position of a target seen from `observer_ssb_m` at
/// `t_obs_tdb_s` (TDB seconds past J2000).
///
/// `target_ssb_m(t)` returns the target's **barycentric** ICRF position in metres
/// at TDB `t`, or `None` where it has none. It is called at the retarded instants
/// `t_obs − τ` only.
pub fn astrometric<F>(
    observer_ssb_m: Vector3<f64>,
    t_obs_tdb_s: f64,
    mut target_ssb_m: F,
) -> Result<SkyPosition, AstrometryError>
where
    F: FnMut(f64) -> Option<Vector3<f64>>,
{
    let mut tau = 0.0;
    let mut last_step = f64::INFINITY;
    for _ in 0..LIGHT_TIME_MAX_ITERATIONS {
        let t_emit = t_obs_tdb_s - tau;
        let target = target_ssb_m(t_emit).ok_or(AstrometryError::TargetUnavailable {
            tdb_seconds: t_emit,
        })?;
        let line = target - observer_ssb_m;
        let range = line.norm();
        if range == 0.0 {
            return Err(AstrometryError::ZeroRange);
        }
        let next = range / SPEED_OF_LIGHT_M_S;
        last_step = next - tau;
        tau = next;
        if last_step.abs() < LIGHT_TIME_TOLERANCE_S {
            let unit = line / range;
            let (ra_rad, dec_rad) = radec_of(&unit);
            return Ok(SkyPosition {
                ra_rad,
                dec_rad,
                range_m: range,
                light_time_s: tau,
                unit_icrf: unit,
            });
        }
    }
    Err(AstrometryError::NotConverged {
        last_step_s: last_step,
    })
}

/// [`astrometric`] from the **geocentre** (DE440's reconstructed Earth, not the
/// Earth–Moon barycentre) at `epoch`.
pub fn geocentric<F>(
    eph: &Ephemeris,
    epoch: Epoch,
    target_ssb_m: F,
) -> Result<SkyPosition, AstrometryError>
where
    F: FnMut(f64) -> Option<Vector3<f64>>,
{
    let earth_m = eph.geocenter_ssb_km(epoch.as_hifitime())? * 1000.0;
    astrometric(earth_m, epoch.tdb_seconds_past_j2000(), target_ssb_m)
}

/// [`astrometric`] from an observatory on the ground — the geocentre plus the
/// site's position from [`earth_orientation`](crate::earth_orientation). This is
/// what a telescope records; [`geocentric`] differs from it by up to `R⊕/Δ`
/// (8.8″ at 1 au, 88″ at 0.1 au).
pub fn topocentric<F>(
    eph: &Ephemeris,
    site: &Site,
    epoch: Epoch,
    target_ssb_m: F,
) -> Result<SkyPosition, AstrometryError>
where
    F: FnMut(f64) -> Option<Vector3<f64>>,
{
    let earth_m = eph.geocenter_ssb_km(epoch.as_hifitime())? * 1000.0;
    astrometric(
        earth_m + site.gcrs_m(epoch),
        epoch.tdb_seconds_past_j2000(),
        target_ssb_m,
    )
}

/// Right ascension in `[0, 2π)` and declination of a direction. The vector need
/// not be unit length; it must not be zero.
pub fn radec_of(v: &Vector3<f64>) -> (f64, f64) {
    let ra = v.y.atan2(v.x).rem_euclid(std::f64::consts::TAU);
    let dec = v.z.atan2(v.x.hypot(v.y));
    (ra, dec)
}

/// The unit vector pointing at (`ra_rad`, `dec_rad`).
pub fn unit_of(ra_rad: f64, dec_rad: f64) -> Vector3<f64> {
    let (sr, cr) = ra_rad.sin_cos();
    let (sd, cd) = dec_rad.sin_cos();
    Vector3::new(cd * cr, cd * sr, sd)
}

/// The angle between two directions, radians. Uses `atan2(|a×b|, a·b)`, which
/// keeps full precision at the sub-arcsecond separations the screen measures
/// (`acos` of a dot product loses it there).
pub fn separation_rad(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    a.cross(b).norm().atan2(a.dot(b))
}

/// Radians to arcseconds.
pub const RAD_TO_ARCSEC: f64 = 180.0 * 3600.0 / std::f64::consts::PI;

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn ra_dec_and_unit_vectors_are_inverses() {
        for &(ra, dec) in &[(0.0, 0.0), (1.0, 0.5), (3.0, -1.2), (6.2, 1.5), (PI, -0.3)] {
            let (r, d) = radec_of(&unit_of(ra, dec));
            assert!(
                (r - ra).abs() < 1e-14 && (d - dec).abs() < 1e-14,
                "{ra},{dec} -> {r},{d}"
            );
        }
        // The pole: RA is undefined, the declination is not.
        let (_, d) = radec_of(&Vector3::new(0.0, 0.0, 2.0));
        assert_eq!(d, FRAC_PI_2);
    }

    #[test]
    fn separation_keeps_precision_at_milliarcseconds() {
        let a = unit_of(1.0, 0.3);
        let tiny = 1e-3 / RAD_TO_ARCSEC; // 1 mas
        let b = unit_of(1.0, 0.3 + tiny);
        let sep = separation_rad(&a, &b);
        assert!(
            (sep - tiny).abs() / tiny < 1e-6,
            "{} mas",
            sep * RAD_TO_ARCSEC * 1e3
        );
    }

    /// A target at rest: the light time is just distance over c.
    #[test]
    fn a_target_at_rest_is_seen_at_distance_over_c() {
        let obs = Vector3::new(1.0e11, 0.0, 0.0);
        let target = Vector3::new(1.0e11, 3.0e10, 4.0e10);
        let s = astrometric(obs, 0.0, |_| Some(target)).unwrap();
        assert!((s.range_m - 5.0e10).abs() < 1e-3);
        assert!((s.light_time_s - 5.0e10 / SPEED_OF_LIGHT_M_S).abs() < 1e-12);
        assert!((s.unit_icrf - Vector3::new(0.0, 0.6, 0.8)).norm() < 1e-15);
    }

    /// A target in uniform motion has a closed-form light time: with the
    /// observer at the origin, `|p + v(−τ)| = cτ` is a quadratic in `τ`. The
    /// iteration must land on its positive root — an oracle independent of the
    /// iteration itself.
    #[test]
    fn uniform_motion_matches_the_closed_form_light_time() {
        let p = Vector3::new(2.0e10, -1.0e10, 5.0e9); // position at reception time
        let v = Vector3::new(3.0e4, 2.0e4, -1.5e4); // 39 km/s, fast for a NEO
        let c = SPEED_OF_LIGHT_M_S;
        // |p − vτ|² = c²τ²  →  (c² − v²)τ² + 2(p·v)τ − p² = 0
        let (a, b, cc) = (c * c - v.dot(&v), 2.0 * p.dot(&v), -p.dot(&p));
        let tau = (-b + (b * b - 4.0 * a * cc).sqrt()) / (2.0 * a);

        let t_obs = 1000.0;
        let s = astrometric(Vector3::zeros(), t_obs, |t| Some(p + v * (t - t_obs))).unwrap();
        assert!(
            (s.light_time_s - tau).abs() < 1e-8,
            "{} vs {}",
            s.light_time_s,
            tau
        );
        let expected = (p - v * tau).normalize();
        let miss = separation_rad(&s.unit_icrf, &expected) * RAD_TO_ARCSEC;
        assert!(miss < 1e-6, "{miss} arcsec");
    }

    #[test]
    fn a_missing_target_is_reported_not_zeroed() {
        let err = astrometric(Vector3::zeros(), 5.0, |_| None).unwrap_err();
        assert_eq!(err, AstrometryError::TargetUnavailable { tdb_seconds: 5.0 });
        let err = astrometric(Vector3::zeros(), 5.0, |_| Some(Vector3::zeros())).unwrap_err();
        assert_eq!(err, AstrometryError::ZeroRange);
    }
}
