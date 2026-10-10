//! Telescope shots — the asteroid against real stars, as a camera records it.
//!
//! Step 3 of the sky-observation screen
//! (`docs/plans/2026-10-10-sky-observation-screen.md`). A shot is one exposure
//! from a ground site: the catalogue stars in the field, carried to the date by
//! their proper motions ([`star_catalog`](crate::star_catalog)), and the
//! asteroid at its topocentric astrometric position
//! ([`astrometry::topocentric`](crate::astrometry::topocentric)) — both
//! projected onto the flat image plane the way a camera's optics project the
//! sky (the gnomonic, or tangent-plane, projection).
//!
//! # What is real here, and what is chosen
//!
//! - **Real:** every star (Tycho-2), the asteroid's position (matched to JPL
//!   Horizons to milliarcseconds from Mt. Lemmon), its brightness (the IAU H-G
//!   magnitude, matched to Horizons' APmag), the darkness of the sky and the
//!   asteroid's altitude (matched to Horizons' solar-presence flag and
//!   elevation), and the **size of the measurement error**: Mt. Lemmon's
//!   astrometric residuals, 0.31″ in RA and 0.28″ in Dec (Vereš et al. 2017,
//!   *Icarus* 296, 139, table 1 — 18.6 million G96 detections).
//! - **Chosen, and labelled so on the screen:** the field size and pixel scale
//!   (not Mt. Lemmon's camera), and the **pointing** — the telescope is aimed at
//!   the asteroid's true position at each night's first shot, which no real
//!   observer can do for an object whose orbit they are still working out.
//!
//! # The error is in the picture, not in the measurement
//!
//! The asteroid is *drawn* at its true position plus one seeded random offset
//! per shot ([`RockImage::drawn_mid`]). Whoever then measures the picture — the
//! player clicking, the snap-to-centre help, or the automatic measurer — reads
//! the same offset image, so the error enters once, the way it does in a real
//! frame. The offset is reproducible: the same run and shot always give the
//! same picture.
//!
//! # Time is mid-exposure
//!
//! The asteroid moves during an exposure — at its March 2021 approach about
//! 3.6″ a minute, so a 60 s exposure smears it into a 3.6″ trail. A measured
//! position belongs to the **middle** of the exposure; [`ShotSpec::epoch_mid`]
//! is that instant and the trail runs half an exposure either side of it.
//!
//! # The stars are not perfect either
//!
//! Tycho-2 positions are good to ~60 mas at J2000 and its proper motions to
//! ~2.5 mas/yr (CDS I/259 ReadMe), so a reference star in 2021 is off by about
//! 0.1″ — under the asteroid's own 0.3″ error, so it does not set the result,
//! but it is not zero. Stars with no proper motion at all (Tycho-2 flag `X`)
//! are drawn but never offered as references ([`ShotStar::reference`]).

use nalgebra::Vector3;

use crate::astrometry::{self, AstrometryError, SkyPosition, RAD_TO_ARCSEC};
use crate::earth_orientation::Site;
use crate::ephemeris::Ephemeris;
use crate::epoch::Epoch;
use crate::rng::NormalRng;
use crate::star_catalog::{StarCatalog, StarCatalogError, TycId};

/// Mt. Lemmon Survey's astrometric error, arcseconds, RA·cos Dec and Dec
/// (Vereš et al. 2017, *Icarus* 296, 139, table 1).
pub const G96_SIGMA_RA_ARCSEC: f64 = 0.31;
/// See [`G96_SIGMA_RA_ARCSEC`].
pub const G96_SIGMA_DEC_ARCSEC: f64 = 0.28;

/// The Sun's centre this far below the horizon is astronomical night — the sky
/// is as dark as it gets.
pub const ASTRONOMICAL_NIGHT_DEG: f64 = -18.0;

const AU_M: f64 = 149_597_870_700.0;

/// A target the telescope can be pointed at: its barycentric ICRF position in
/// metres at a TDB instant, and its H-G magnitude parameters.
pub struct Target<'a> {
    pub name: String,
    /// Absolute magnitude.
    pub h: f64,
    /// Slope parameter.
    pub g: f64,
    /// Barycentric ICRF position, metres, at TDB seconds past J2000.
    pub ssb_m: Box<dyn Fn(f64) -> Option<Vector3<f64>> + 'a>,
}

/// Apparent magnitude in the IAU H-G system (Bowell et al. 1989, in *Asteroids
/// II*): `H + 5 log₁₀(r Δ) − 2.5 log₁₀((1−G)Φ₁ + GΦ₂)`, with
/// `Φᵢ = exp(−Aᵢ tan(α/2)^Bᵢ)`, `A = (3.33, 1.87)`, `B = (0.63, 1.22)`. The
/// form Horizons' APmag uses.
pub fn hg_magnitude(h: f64, g: f64, r_au: f64, delta_au: f64, phase_rad: f64) -> f64 {
    let t = (phase_rad / 2.0).tan();
    let phi1 = (-3.33 * t.powf(0.63)).exp();
    let phi2 = (-1.87 * t.powf(1.22)).exp();
    h + 5.0 * (r_au * delta_au).log10() - 2.5 * ((1.0 - g) * phi1 + g * phi2).log10()
}

/// One look at the target from a site: where it is, how bright, and whether it
/// can be seen.
#[derive(Debug, Clone, Copy)]
pub struct Sighting {
    pub epoch: Epoch,
    /// Topocentric astrometric position.
    pub sky: SkyPosition,
    /// Sun–target distance, au, at the light-emission instant.
    pub r_au: f64,
    /// Observer–target distance, au.
    pub delta_au: f64,
    /// Sun–target–observer angle, radians.
    pub phase_rad: f64,
    /// H-G apparent magnitude (airless).
    pub v_mag: f64,
    /// Altitude of the Sun's centre above the site's horizon, radians
    /// (geometric, no refraction).
    pub sun_alt_rad: f64,
    /// Altitude of the target, radians (geometric, no refraction).
    pub target_alt_rad: f64,
}

impl Sighting {
    /// Astronomical night: the Sun more than 18° below the horizon.
    pub fn dark(&self) -> bool {
        self.sun_alt_rad.to_degrees() < ASTRONOMICAL_NIGHT_DEG
    }

    /// Dark, and the target at least `min_alt_deg` up.
    pub fn observable(&self, min_alt_deg: f64) -> bool {
        self.dark() && self.target_alt_rad.to_degrees() >= min_alt_deg
    }
}

/// Look at `target` from `site` at `epoch`.
pub fn sight(
    eph: &Ephemeris,
    site: &Site,
    epoch: Epoch,
    target: &Target,
) -> Result<Sighting, AstrometryError> {
    let sky = astrometry::topocentric(eph, site, epoch, |t| (target.ssb_m)(t))?;
    let t_emit = epoch.tdb_seconds_past_j2000() - sky.light_time_s;
    let rock = (target.ssb_m)(t_emit).ok_or(AstrometryError::TargetUnavailable {
        tdb_seconds: t_emit,
    })?;
    let emit = Epoch::from_tdb_seconds_past_j2000(t_emit);
    let sun_emit = eph.sun_ssb_km(emit.as_hifitime())? * 1000.0;
    let earth = eph.geocenter_ssb_km(epoch.as_hifitime())? * 1000.0;
    let observer = earth + site.gcrs_m(epoch);

    let to_sun = sun_emit - rock;
    let to_obs = observer - rock;
    let r_au = to_sun.norm() / AU_M;
    let delta_au = sky.range_m / AU_M;
    let phase_rad = astrometry::separation_rad(&to_sun, &to_obs);
    let v_mag = hg_magnitude(target.h, target.g, r_au, delta_au, phase_rad);

    let zenith = site.zenith_gcrs(epoch);
    let sun_now = eph.sun_ssb_km(epoch.as_hifitime())? * 1000.0;
    let sun_alt_rad = (sun_now - observer).normalize().dot(&zenith).asin();
    let target_alt_rad = sky.unit_icrf.dot(&zenith).asin();
    Ok(Sighting {
        epoch,
        sky,
        r_au,
        delta_au,
        phase_rad,
        v_mag,
        sun_alt_rad,
        target_alt_rad,
    })
}

/// The camera's image plane: the gnomonic projection about a pointing
/// direction. `ξ` runs east (increasing RA), `η` north; both in arcseconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TangentPlane {
    pub center: Vector3<f64>,
    pub east: Vector3<f64>,
    pub north: Vector3<f64>,
}

impl TangentPlane {
    /// The plane touching the sky at `center` (any length, not along the pole
    /// exactly — at the pole "east" is undefined and an arbitrary one is used).
    pub fn new(center: &Vector3<f64>) -> Self {
        let c = center.normalize();
        let east = Vector3::new(-c.y, c.x, 0.0);
        let east = if east.norm() < 1e-12 {
            Vector3::new(0.0, 1.0, 0.0)
        } else {
            east.normalize()
        };
        let north = c.cross(&east);
        Self {
            center: c,
            east,
            north,
        }
    }

    /// `(ξ, η)` in arcseconds of a direction, or `None` if it is behind the
    /// plane (more than 90° from the centre).
    pub fn project(&self, p: &Vector3<f64>) -> Option<(f64, f64)> {
        let d = p.dot(&self.center);
        if d <= 0.0 {
            return None;
        }
        Some((
            p.dot(&self.east) / d * RAD_TO_ARCSEC,
            p.dot(&self.north) / d * RAD_TO_ARCSEC,
        ))
    }

    /// The unit direction at `(ξ, η)` arcseconds — the inverse of
    /// [`project`](Self::project).
    pub fn deproject(&self, xi_arcsec: f64, eta_arcsec: f64) -> Vector3<f64> {
        (self.center
            + self.east * (xi_arcsec / RAD_TO_ARCSEC)
            + self.north * (eta_arcsec / RAD_TO_ARCSEC))
            .normalize()
    }
}

/// What one exposure is asked to be.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShotSpec {
    /// The middle of the exposure — the instant a measured position belongs to.
    pub epoch_mid: Epoch,
    pub exposure_s: f64,
    /// Where the telescope points (the field centre).
    pub pointing: Vector3<f64>,
    /// Half the side of the square field, arcseconds.
    pub half_width_arcsec: f64,
    /// Faintest star drawn.
    pub star_mag_limit: f32,
}

/// A star in a shot.
#[derive(Debug, Clone, PartialEq)]
pub struct ShotStar {
    pub id: TycId,
    pub xi_arcsec: f64,
    pub eta_arcsec: f64,
    pub v_mag: f32,
    /// Fit to measure against: it has a proper motion, so its position is
    /// good to ~0.1″ at the shot's date. Stars without one are drawn only.
    pub reference: bool,
    /// Its name, if it has one (`Vega`, `β Ori`, `21 And`).
    pub label: Option<String>,
}

/// The asteroid's image in a shot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RockImage {
    /// The true position at mid-exposure, `(ξ, η)` arcseconds — kept for
    /// scoring, never shown before the player has measured.
    pub true_mid: (f64, f64),
    /// Where the picture puts it: the true position plus this shot's seeded
    /// measurement error.
    pub drawn_mid: (f64, f64),
    /// The drawn trail's ends, half an exposure either side (same offset).
    pub drawn_start: (f64, f64),
    pub drawn_end: (f64, f64),
    pub v_mag: f64,
}

/// One exposure.
#[derive(Debug, Clone)]
pub struct Shot {
    pub spec: ShotSpec,
    pub plane: TangentPlane,
    pub stars: Vec<ShotStar>,
    /// `None` when the asteroid is outside the field.
    pub rock: Option<RockImage>,
    pub sighting: Sighting,
}

/// Why a shot could not be made.
#[derive(Debug)]
pub enum ShotError {
    Astrometry(AstrometryError),
    Stars(StarCatalogError),
}

impl std::fmt::Display for ShotError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Astrometry(e) => write!(f, "{e}"),
            Self::Stars(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for ShotError {}

impl From<AstrometryError> for ShotError {
    fn from(e: AstrometryError) -> Self {
        Self::Astrometry(e)
    }
}
impl From<StarCatalogError> for ShotError {
    fn from(e: StarCatalogError) -> Self {
        Self::Stars(e)
    }
}

/// The seed for one shot of one run: the run's seed mixed with the shot's index,
/// so shots differ from each other and a run is reproducible as a whole.
pub fn shot_seed(run_seed: u64, shot_index: usize) -> u64 {
    // SplitMix64's finaliser, to spread neighbouring indices apart.
    let mut z = run_seed ^ (shot_index as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Make one shot. `seed` fixes its measurement error (see [`shot_seed`]).
pub fn take_shot(
    eph: &Ephemeris,
    catalog: &StarCatalog,
    site: &Site,
    target: &Target,
    spec: &ShotSpec,
    seed: u64,
) -> Result<Shot, ShotError> {
    let plane = TangentPlane::new(&spec.pointing);
    let t_mid = spec.epoch_mid.tdb_seconds_past_j2000();
    let hw = spec.half_width_arcsec;
    let inside = |(x, y): (f64, f64)| x.abs() <= hw && y.abs() <= hw;

    let radius = hw * std::f64::consts::SQRT_2 / RAD_TO_ARCSEC;
    let mut stars = Vec::new();
    for s in catalog.cone(&plane.center, radius, spec.star_mag_limit, t_mid)? {
        let Some(xy) = plane.project(&s.unit_icrf) else {
            continue;
        };
        if !inside(xy) {
            continue;
        }
        stars.push(ShotStar {
            id: s.id,
            xi_arcsec: xy.0,
            eta_arcsec: xy.1,
            v_mag: s.v_mag,
            reference: !s.no_proper_motion,
            label: catalog
                .name(s.id)
                .map(|n| n.label())
                .filter(|l| !l.is_empty()),
        });
    }

    let sighting = sight(eph, site, spec.epoch_mid, target)?;
    let half = spec.exposure_s / 2.0;
    let at = |dt: f64| -> Result<Option<(f64, f64)>, AstrometryError> {
        let e = spec.epoch_mid.shifted_by_seconds(dt);
        let s = astrometry::topocentric(eph, site, e, |t| (target.ssb_m)(t))?;
        Ok(plane.project(&s.unit_icrf))
    };
    let rock = match plane.project(&sighting.sky.unit_icrf) {
        Some(mid) if inside(mid) => {
            let mut rng = NormalRng::new(seed);
            let off = (
                rng.normal() * G96_SIGMA_RA_ARCSEC,
                rng.normal() * G96_SIGMA_DEC_ARCSEC,
            );
            let shift = |p: (f64, f64)| (p.0 + off.0, p.1 + off.1);
            let start = at(-half)?.unwrap_or(mid);
            let end = at(half)?.unwrap_or(mid);
            Some(RockImage {
                true_mid: mid,
                drawn_mid: shift(mid),
                drawn_start: shift(start),
                drawn_end: shift(end),
                v_mag: sighting.v_mag,
            })
        }
        _ => None,
    };
    Ok(Shot {
        spec: *spec,
        plane,
        stars,
        rock,
        sighting,
    })
}

/// How a run of shots is laid out across nights.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NightPlan {
    /// Nights with shots.
    pub nights: usize,
    pub shots_per_night: usize,
    /// Between consecutive shots in a night, seconds.
    pub spacing_s: f64,
    pub exposure_s: f64,
    /// The target must be at least this high for every shot.
    pub min_alt_deg: f64,
    /// Skip this many days between observed nights (0 = consecutive nights).
    pub gap_days: usize,
}

/// The mid-exposure instants of a run: for each night from `start` on, the
/// sequence of shots placed in that night's observable window (dark, target
/// high enough), centred on when the target is highest. A night with no window
/// long enough is skipped; the search gives up after `nights × (gap + 1) + 10`
/// days. The window's edges are found on a 5-minute grid, so a sequence can sit
/// up to 5 minutes inside where a finer search would put its ends.
pub fn plan_run(
    eph: &Ephemeris,
    site: &Site,
    target: &Target,
    start: Epoch,
    plan: &NightPlan,
) -> Result<Vec<Vec<Epoch>>, AstrometryError> {
    const STEP_S: f64 = 300.0;
    let span = (plan.shots_per_night.saturating_sub(1)) as f64 * plan.spacing_s + plan.exposure_s;
    let max_days = plan.nights * (plan.gap_days + 1) + 10;
    let mut nights = Vec::new();
    let mut day = 0;
    while nights.len() < plan.nights && day < max_days {
        let day_start = start.shifted_by_seconds(day as f64 * 86_400.0);
        // Observable samples across this day, and the longest unbroken run.
        let mut best: Option<(usize, usize)> = None;
        let mut run_start: Option<usize> = None;
        let steps = (86_400.0 / STEP_S) as usize;
        let mut alts = Vec::with_capacity(steps + 1);
        for i in 0..=steps {
            let s = sight(
                eph,
                site,
                day_start.shifted_by_seconds(i as f64 * STEP_S),
                target,
            )?;
            alts.push(s.target_alt_rad);
            if s.observable(plan.min_alt_deg) {
                run_start.get_or_insert(i);
            } else if let Some(a) = run_start.take() {
                if best.is_none_or(|(x, y)| i - a > y - x) {
                    best = Some((a, i));
                }
            }
        }
        if let Some(a) = run_start {
            if best.is_none_or(|(x, y)| steps + 1 - a > y - x) {
                best = Some((a, steps + 1));
            }
        }
        if let Some((a, b)) = best {
            let window = (b - a - 1) as f64 * STEP_S;
            if window >= span {
                // Centre on the highest sample, then clamp into the window.
                let peak = (a..b)
                    .max_by(|&i, &j| alts[i].total_cmp(&alts[j]))
                    .unwrap_or(a);
                let lo = a as f64 * STEP_S + span / 2.0;
                let hi = (b - 1) as f64 * STEP_S - span / 2.0;
                let mid = (peak as f64 * STEP_S).clamp(lo, hi);
                let first = mid - span / 2.0 + plan.exposure_s / 2.0;
                let shots = (0..plan.shots_per_night)
                    .map(|k| day_start.shifted_by_seconds(first + k as f64 * plan.spacing_s))
                    .collect();
                nights.push(shots);
                day += plan.gap_days;
            }
        }
        day += 1;
    }
    Ok(nights)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::astrometry::{separation_rad, unit_of};

    #[test]
    fn projection_round_trips_and_runs_east_and_north() {
        let plane = TangentPlane::new(&unit_of(1.2, 0.4));
        for &(x, y) in &[(0.0, 0.0), (900.0, -300.0), (-1800.0, 1800.0)] {
            let p = plane.deproject(x, y);
            let (x2, y2) = plane.project(&p).unwrap();
            assert!((x - x2).abs() < 1e-9 && (y - y2).abs() < 1e-9);
        }
        // A small step in RA moves +ξ, a small step in Dec moves +η.
        let (x, _) = plane.project(&unit_of(1.2 + 1e-5, 0.4)).unwrap();
        let (_, y) = plane.project(&unit_of(1.2, 0.4 + 1e-5)).unwrap();
        assert!(x > 0.0 && y > 0.0);
        // Near the centre the projection is the angle itself.
        let p = unit_of(1.2, 0.4 + 10.0 / RAD_TO_ARCSEC);
        let (_, y) = plane.project(&p).unwrap();
        let sep = separation_rad(&p, &plane.center) * RAD_TO_ARCSEC;
        assert!((y - sep).abs() < 1e-6, "{y} vs {sep}");
    }

    /// Bowell's own check value: at zero phase both Φ are 1, so the magnitude
    /// is `H + 5 log₁₀(rΔ)` whatever G is.
    #[test]
    fn hg_at_zero_phase_is_distance_only() {
        let m = hg_magnitude(19.09, 0.24, 1.1, 0.12, 0.0);
        assert!((m - (19.09 + 5.0 * (1.1f64 * 0.12).log10())).abs() < 1e-12);
        // And fainter at larger phase.
        assert!(hg_magnitude(19.09, 0.24, 1.1, 0.12, 0.5) > m);
    }

    #[test]
    fn shot_seeds_differ_and_repeat() {
        assert_eq!(shot_seed(7, 3), shot_seed(7, 3));
        assert_ne!(shot_seed(7, 3), shot_seed(7, 4));
        assert_ne!(shot_seed(7, 3), shot_seed(8, 3));
    }
}
