//! The sky-observation screen's data — built on a worker, godot-free so it is
//! unit-testable without a running Godot (the same split as `mission_core`).
//!
//! One observing run of Apophis from Mt. Lemmon across its March 2021 approach
//! (`docs/plans/2026-10-10-sky-observation-screen.md`, step 4): the shots of
//! [`asteroid_core::sky_shot`], plus for each night a wide **finder chart** —
//! the naked-eye stars around the pointing, with their names — because a 0.5°
//! telescope field almost never contains a star with a name, and "where in the
//! sky is this?" is the first thing an observer asks.
//!
//! Everything drawn comes from here; GDScript only places it on the screen.

use asteroid_core::astrometry::{separation_rad, RAD_TO_ARCSEC};
use asteroid_core::earth_orientation::MT_LEMMON;
use asteroid_core::elements::OrbitalElements;
use asteroid_core::frames::icrf_to_ecliptic;
use asteroid_core::propagator::{
    eccentric_from_true, mean_from_eccentric, solve_kepler, true_from_eccentric, KeplerPropagator,
};
use asteroid_core::state::StateVector;
use nalgebra::Vector3;
use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::epoch::Epoch;
use asteroid_core::horizons::{self, Neo};
use asteroid_core::sky_shot::{
    plan_run, shot_seed, sight, take_shot, NightPlan, RockImage, ShotSpec, TangentPlane, Target,
    APOPHIS_G, APOPHIS_H, G96_SIGMA_DEC_ARCSEC, G96_SIGMA_RA_ARCSEC,
};
use asteroid_core::star_catalog::{self, StarCatalog};

/// How the run is laid out. The defaults are the Apophis 2021 scenario; the
/// numbers marked *chosen* are game choices, not Mt. Lemmon's.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkyRunConfig {
    /// Seed for the shots' measurement errors.
    pub seed: u64,
    /// The search for the first night starts here (TDB s past J2000).
    pub start_tdb_s: f64,
    pub plan: NightPlan,
    /// Half the telescope field's side, arcseconds (*chosen*: 15′, a 0.5° field).
    pub half_width_arcsec: f64,
    /// Faintest star in a shot.
    pub star_mag_limit: f32,
    /// The finder chart's radius, degrees, and faintest star.
    pub finder_radius_deg: f64,
    pub finder_mag_limit: f32,
}

impl SkyRunConfig {
    /// Apophis, March 2021: three consecutive nights from 2021-03-05, three
    /// 60 s exposures 20 minutes apart each night, the rock at least 30° up.
    pub fn apophis_2021(seed: u64) -> Self {
        let start = Epoch::from_utc_gregorian(2021, 3, 5, 0, 0, 0);
        Self {
            seed,
            start_tdb_s: start.tdb_seconds_past_j2000(),
            plan: NightPlan {
                nights: 3,
                shots_per_night: 3,
                spacing_s: 1200.0,
                exposure_s: 60.0,
                min_alt_deg: 30.0,
                gap_days: 0,
            },
            half_width_arcsec: 900.0,
            star_mag_limit: 12.0,
            finder_radius_deg: 12.0,
            finder_mag_limit: 6.0,
        }
    }
}

/// A star as drawn: image-plane position and what to call it.
#[derive(Debug, Clone, PartialEq)]
pub struct SkyStarView {
    pub xi_arcsec: f64,
    pub eta_arcsec: f64,
    pub v_mag: f32,
    /// Usable to measure against (has a proper motion).
    pub reference: bool,
    /// Its name, or empty.
    pub label: String,
}

/// One shot, ready to draw.
#[derive(Debug, Clone)]
pub struct SkyShotView {
    pub night: usize,
    pub index_in_night: usize,
    /// Mid-exposure, TDB seconds past J2000.
    pub epoch_tdb_s: f64,
    /// Mid-exposure as a UTC calendar string.
    pub utc: String,
    pub exposure_s: f64,
    pub stars: Vec<SkyStarView>,
    /// `None` if the rock is outside the field.
    pub rock: Option<RockImage>,
    pub v_mag: f64,
    pub alt_deg: f64,
    pub sun_alt_deg: f64,
    pub delta_au: f64,
    /// The rock's speed across the sky, arcseconds per minute.
    pub rate_arcsec_min: f64,
    pub pointing_ra_deg: f64,
    pub pointing_dec_deg: f64,
    /// The pointing as a unit vector — the image plane the ghosts are drawn in.
    pub pointing_icrf: Vector3<f64>,
}

/// A night's wide finder chart, on the same image plane as that night's shots.
#[derive(Debug, Clone)]
pub struct SkyFinderView {
    pub stars: Vec<SkyStarView>,
}

/// A whole run.
#[derive(Debug, Clone)]
pub struct SkyRunView {
    pub target: String,
    pub site: String,
    pub half_width_arcsec: f64,
    pub finder_radius_deg: f64,
    pub sigma_ra_arcsec: f64,
    pub sigma_dec_arcsec: f64,
    pub shots: Vec<SkyShotView>,
    pub finders: Vec<SkyFinderView>,
    /// The epoch the trial orbit's elements are stated at — the run's middle
    /// shot — TDB seconds and as UTC text.
    pub element_epoch_tdb_s: f64,
    pub element_epoch_utc: String,
    /// JPL's own orbit at that epoch as trial elements: the osculating ellipse of
    /// the Horizons state (what [E] reveals).
    pub truth: TrialElements,
    /// Where the player starts: [`truth`](Self::truth) knocked off by
    /// [`START_OFFSET`], so the first ghost is degrees away but the right part of
    /// the sky.
    pub start: TrialElements,
    /// The Sun's GM the two-body orbits use (DE440's), m³/s².
    pub mu_sun: f64,
}

/// A trial orbit as the screen dials it: `[a (au), e, i, node, perihelion
/// argument, mean anomaly]`, angles in degrees, heliocentric, ecliptic J2000.
pub type TrialElements = [f64; 6];

/// How far the starting guess is from JPL's orbit, element by element. A
/// teaching choice (labelled on the screen as "a rough guess"): every element
/// is wrong, by amounts an astronomer's first look could plausibly be off.
pub const START_OFFSET: TrialElements = [0.05, -0.03, 1.5, 4.0, -6.0, 3.0];

const AU_M: f64 = 149_597_870_700.0;

/// What one trial orbit predicts for one shot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ghost {
    /// On the shot's image plane, arcseconds; NaN when the prediction is more
    /// than 90° away (behind the plane).
    pub xi_arcsec: f64,
    pub eta_arcsec: f64,
    /// Ghost minus the asteroid's drawn image, on the plane (NaN if either is
    /// missing).
    pub dxi_arcsec: f64,
    pub deta_arcsec: f64,
    /// The angle on the sky between the ghost and the drawn asteroid — the miss.
    pub sep_arcsec: f64,
    /// Which way the ghost lies from the asteroid, degrees, north through east.
    pub pa_deg: f64,
}

/// Turn trial elements into the core's element set (mean anomaly → true).
pub fn trial_to_elements(el: &TrialElements) -> Result<OrbitalElements, String> {
    let [a_au, e, i, node, peri, m] = *el;
    if !(a_au > 0.0 && (0.0..1.0).contains(&e) && (0.0..=180.0).contains(&i)) {
        return Err(format!("a {a_au} au, e {e}, i {i} deg is not an ellipse"));
    }
    let ecc = solve_kepler(m.to_radians(), e).map_err(|x| x.to_string())?;
    let nu = true_from_eccentric(ecc, e);
    Ok(OrbitalElements::new(
        a_au * AU_M,
        e,
        i.to_radians(),
        node.to_radians(),
        peri.to_radians(),
        nu,
    ))
}

/// The core's element set as trial elements (true anomaly → mean).
pub fn elements_to_trial(el: &OrbitalElements) -> TrialElements {
    let ecc = eccentric_from_true(el.true_anomaly, el.eccentricity);
    let m = mean_from_eccentric(ecc, el.eccentricity).rem_euclid(std::f64::consts::TAU);
    [
        el.semi_major_axis / AU_M,
        el.eccentricity,
        el.inclination.to_degrees(),
        el.raan.to_degrees(),
        el.arg_periapsis.to_degrees(),
        m.to_degrees(),
    ]
}

/// Keep a trial element inside its legal range, the way the screen's knobs do.
fn clamp_trial(k: usize, v: f64) -> f64 {
    match k {
        0 => v.max(0.05),
        1 => v.clamp(0.0, 0.99),
        2 => v.clamp(0.0, 180.0),
        _ => v.rem_euclid(360.0),
    }
}

impl SkyRunView {
    /// What `el` predicts for every shot, against the drawn asteroid.
    pub fn ghosts(&self, eph: &Ephemeris, el: &TrialElements) -> Result<Vec<Ghost>, String> {
        let elements = trial_to_elements(el)?;
        let epoch = Epoch::from_tdb_seconds_past_j2000(self.element_epoch_tdb_s);
        let kepler = KeplerPropagator::new(elements, self.mu_sun, epoch).map_err(|e| e.to_string())?;
        let target = Target::two_body("TRIAL", APOPHIS_H, APOPHIS_G, kepler, eph);
        let mut out = Vec::with_capacity(self.shots.len());
        for shot in &self.shots {
            let at = Epoch::from_tdb_seconds_past_j2000(shot.epoch_tdb_s);
            let ghost = sight(eph, &MT_LEMMON, at, &target)
                .map_err(|e| e.to_string())?
                .sky
                .unit_icrf;
            let plane = TangentPlane::new(&shot.pointing_icrf);
            let (xi, eta) = plane.project(&ghost).unwrap_or((f64::NAN, f64::NAN));
            let (mut dxi, mut deta, mut sep, mut pa) = (f64::NAN, f64::NAN, f64::NAN, f64::NAN);
            if let Some(r) = shot.rock {
                let rock = plane.deproject(r.drawn_mid.0, r.drawn_mid.1);
                dxi = xi - r.drawn_mid.0;
                deta = eta - r.drawn_mid.1;
                sep = separation_rad(&rock, &ghost) * RAD_TO_ARCSEC;
                // Position angle at the asteroid: north through east.
                let east = Vector3::new(-rock.y, rock.x, 0.0).normalize();
                let north = rock.cross(&east);
                let d = ghost - rock;
                pa = d.dot(&east).atan2(d.dot(&north)).to_degrees().rem_euclid(360.0);
            }
            out.push(Ghost {
                xi_arcsec: xi,
                eta_arcsec: eta,
                dxi_arcsec: dxi,
                deta_arcsec: deta,
                sep_arcsec: sep,
                pa_deg: pa,
            });
        }
        Ok(out)
    }

    /// The root-mean-square miss over every shot with an asteroid in it.
    pub fn rms_arcsec(ghosts: &[Ghost]) -> f64 {
        let seps: Vec<f64> = ghosts.iter().map(|g| g.sep_arcsec).filter(|s| s.is_finite()).collect();
        if seps.is_empty() {
            return f64::NAN;
        }
        (seps.iter().map(|s| s * s).sum::<f64>() / seps.len() as f64).sqrt()
    }

    /// The hint: of the twelve single steps (each element, up or down, by its
    /// step in `steps`), the one that shrinks the RMS miss most —
    /// `(element, ±1, rms after)`, or `None` if no single step helps.
    pub fn hint(
        &self,
        eph: &Ephemeris,
        el: &TrialElements,
        steps: &TrialElements,
    ) -> Result<Option<(usize, f64, f64)>, String> {
        let now = Self::rms_arcsec(&self.ghosts(eph, el)?);
        let mut best: Option<(usize, f64, f64)> = None;
        for k in 0..6 {
            for sign in [-1.0, 1.0] {
                let mut tried = *el;
                tried[k] = clamp_trial(k, tried[k] + sign * steps[k]);
                let Ok(g) = self.ghosts(eph, &tried) else { continue };
                let rms = Self::rms_arcsec(&g);
                if rms < now && best.is_none_or(|b| rms < b.2) {
                    best = Some((k, sign, rms));
                }
            }
        }
        Ok(best)
    }
}

impl SkyRunView {
    /// Build the Apophis run. Needs the DE field (passed in), the shipped
    /// `apophis.neo` table and the Tycho-2 catalogue (both resolved here); a
    /// missing one is an error message for the screen, never a panic.
    pub fn build_apophis(eph: &Ephemeris, cfg: &SkyRunConfig) -> Result<Self, String> {
        let (bodies, _) = horizons::load_all();
        let apophis: Neo = bodies
            .into_iter()
            .find(|n| n.designation() == "99942")
            .ok_or_else(|| {
                format!(
                    "no Apophis table (kernels/neo/apophis.neo). {}",
                    horizons::not_found_message()
                )
            })?;
        let cat_path = star_catalog::resolve().ok_or_else(|| {
            "no star catalogue - run `python tools/fetch_tycho2.py` (writes kernels/stars/)"
                .to_string()
        })?;
        let catalog = StarCatalog::open(&cat_path).map_err(|e| e.to_string())?;

        let target = Target {
            name: "99942 APOPHIS".into(),
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

        let start = Epoch::from_tdb_seconds_past_j2000(cfg.start_tdb_s);
        let nights =
            plan_run(eph, &MT_LEMMON, &target, start, &cfg.plan).map_err(|e| e.to_string())?;
        if nights.is_empty() {
            return Err("no observable night found for this run".into());
        }

        let label_of = |id| catalog.name(id).map(|n| n.label()).unwrap_or_default();
        let mut shots = Vec::new();
        let mut finders = Vec::new();
        for (n, night) in nights.iter().enumerate() {
            // The game's shortcut: the telescope is aimed at the true position at
            // the night's first shot (labelled on the screen).
            let first = sight(eph, &MT_LEMMON, night[0], &target).map_err(|e| e.to_string())?;
            let pointing = first.sky.unit_icrf;
            let plane = TangentPlane::new(&pointing);

            let finder_stars = catalog
                .cone(
                    &pointing,
                    cfg.finder_radius_deg.to_radians(),
                    cfg.finder_mag_limit,
                    night[0].tdb_seconds_past_j2000(),
                )
                .map_err(|e| e.to_string())?
                .into_iter()
                .filter_map(|s| {
                    let (x, y) = plane.project(&s.unit_icrf)?;
                    Some(SkyStarView {
                        xi_arcsec: x,
                        eta_arcsec: y,
                        v_mag: s.v_mag,
                        reference: !s.no_proper_motion,
                        label: label_of(s.id),
                    })
                })
                .collect();
            finders.push(SkyFinderView {
                stars: finder_stars,
            });

            for (k, &epoch) in night.iter().enumerate() {
                let spec = ShotSpec {
                    epoch_mid: epoch,
                    exposure_s: cfg.plan.exposure_s,
                    pointing,
                    half_width_arcsec: cfg.half_width_arcsec,
                    star_mag_limit: cfg.star_mag_limit,
                };
                let seed = shot_seed(cfg.seed, n * 100 + k);
                let shot = take_shot(eph, &catalog, &MT_LEMMON, &target, &spec, seed)
                    .map_err(|e| e.to_string())?;
                let rate = shot
                    .rock
                    .map(|r| {
                        let (a, b) = (r.drawn_start, r.drawn_end);
                        (b.0 - a.0).hypot(b.1 - a.1) / cfg.plan.exposure_s * 60.0
                    })
                    .unwrap_or(f64::NAN);
                let utc = epoch.utc_string();
                let s = &shot.sighting;
                shots.push(SkyShotView {
                    night: n,
                    index_in_night: k,
                    epoch_tdb_s: epoch.tdb_seconds_past_j2000(),
                    utc,
                    exposure_s: cfg.plan.exposure_s,
                    stars: shot
                        .stars
                        .iter()
                        .map(|st| SkyStarView {
                            xi_arcsec: st.xi_arcsec,
                            eta_arcsec: st.eta_arcsec,
                            v_mag: st.v_mag,
                            reference: st.reference,
                            label: st.label.clone().unwrap_or_default(),
                        })
                        .collect(),
                    rock: shot.rock,
                    v_mag: s.v_mag,
                    alt_deg: s.target_alt_rad.to_degrees(),
                    sun_alt_deg: s.sun_alt_rad.to_degrees(),
                    delta_au: s.delta_au,
                    rate_arcsec_min: rate,
                    pointing_ra_deg: first.sky.ra_rad.to_degrees(),
                    pointing_dec_deg: first.sky.dec_rad.to_degrees(),
                    pointing_icrf: pointing,
                });
            }
        }
        // The trial orbit's epoch is the run's middle shot; JPL's osculating
        // ellipse there is the truth [E] reveals.
        let element_epoch_tdb_s = shots[shots.len() / 2].epoch_tdb_s;
        let element_epoch = Epoch::from_tdb_seconds_past_j2000(element_epoch_tdb_s);
        let mu_sun = eph.sun_gm_m3_s2().map_err(|e| e.to_string())?;
        let helio = apophis
            .helio_state_at(element_epoch_tdb_s)
            .ok_or("Apophis' table does not cover the element epoch")?;
        let ecl = StateVector::new(icrf_to_ecliptic(helio.position), icrf_to_ecliptic(helio.velocity));
        let truth = elements_to_trial(&OrbitalElements::from_state(ecl, mu_sun).map_err(|e| e.to_string())?);
        let mut start = truth;
        for k in 0..6 {
            start[k] = clamp_trial(k, truth[k] + START_OFFSET[k]);
        }

        Ok(Self {
            element_epoch_tdb_s,
            element_epoch_utc: element_epoch.utc_string(),
            truth,
            start,
            mu_sun,
            target: target.name.clone(),
            site: format!("{} ({})", MT_LEMMON.name.to_uppercase(), MT_LEMMON.code),
            half_width_arcsec: cfg.half_width_arcsec,
            finder_radius_deg: cfg.finder_radius_deg,
            sigma_ra_arcsec: G96_SIGMA_RA_ARCSEC,
            sigma_dec_arcsec: G96_SIGMA_DEC_ARCSEC,
            shots,
            finders,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The run the screen shows: nine shots over three nights, every one with
    /// the rock in the field and at least three reference stars, and finder
    /// charts that actually name something.
    #[test]
    fn the_apophis_run_builds_and_is_drawable() {
        let Some(k) = asteroid_core::kernels::resolve_for_test("sky run view") else {
            return;
        };
        if star_catalog::resolve_for_test("sky run view (stars)").is_none() {
            return;
        }
        if horizons::load_all_for_test("sky run view (apophis.neo)").is_empty() {
            return;
        }
        let (bsp, pca) = k.as_strs();
        let eph = Ephemeris::load(bsp)
            .and_then(|e| e.with_constants(pca))
            .unwrap();
        let view = SkyRunView::build_apophis(&eph, &SkyRunConfig::apophis_2021(1)).unwrap();
        assert_eq!(view.shots.len(), 9);
        assert_eq!(view.finders.len(), 3);
        for s in &view.shots {
            assert!(
                s.rock.is_some(),
                "night {} shot {}: rock outside the field",
                s.night,
                s.index_in_night
            );
            assert!(s.stars.iter().filter(|x| x.reference).count() >= 3);
            assert!(s.alt_deg >= 30.0 && s.sun_alt_deg < -18.0);
            assert!(
                (2.5..4.5).contains(&s.rate_arcsec_min),
                "{}",
                s.rate_arcsec_min
            );
            assert!(s.utc.starts_with("2021-03-0"), "{}", s.utc);
        }
        for (night, f) in view.finders.iter().enumerate() {
            let named = f.stars.iter().filter(|s| !s.label.is_empty()).count();
            println!(
                "  night {night}: finder {} stars, {named} named",
                f.stars.len()
            );
            assert!(
                named >= 3,
                "night {night}: only {named} named stars in the finder"
            );
        }
        // JPL's orbit lands the ghosts on the drawn asteroid to within the drawn
        // error: 9 shots of a 0.31"/0.28" offset give an RMS near 0.42", and the
        // two-body drift over the run is 0.008" (sky_two_body_vs_truth).
        let truth = view.ghosts(&eph, &view.truth).unwrap();
        let rms_truth = SkyRunView::rms_arcsec(&truth);
        println!("  JPL's orbit: RMS miss {rms_truth:.3}\"");
        assert!((0.15..0.8).contains(&rms_truth), "{rms_truth}");
        // The start is a rough guess: degrees away.
        let start = SkyRunView::rms_arcsec(&view.ghosts(&eph, &view.start).unwrap());
        println!("  starting guess: RMS miss {:.2} deg", start / 3600.0);
        assert!(start > 3600.0, "{start}");
        // The hint finds a step that helps, and following it really does.
        let steps = [0.01, 0.01, 0.1, 1.0, 1.0, 1.0];
        let (k, sign, after) = view.hint(&eph, &view.start, &steps).unwrap().expect("a step helps");
        let mut moved = view.start;
        moved[k] += sign * steps[k];
        let check = SkyRunView::rms_arcsec(&view.ghosts(&eph, &moved).unwrap());
        assert!(after < start && (check - after).abs() < 1e-9);
        // Elements survive the round trip through the core's element set.
        let back = elements_to_trial(&trial_to_elements(&view.truth).unwrap());
        for k in 0..6 {
            assert!((back[k] - view.truth[k]).abs() < 1e-9, "element {k}");
        }

        // Same seed, same picture.
        let again = SkyRunView::build_apophis(&eph, &SkyRunConfig::apophis_2021(1)).unwrap();
        assert_eq!(view.shots[4].rock, again.shots[4].rock);
    }
}
