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

use asteroid_core::earth_orientation::MT_LEMMON;
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
                });
            }
        }
        Ok(Self {
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
        // Same seed, same picture.
        let again = SkyRunView::build_apophis(&eph, &SkyRunConfig::apophis_2021(1)).unwrap();
        assert_eq!(view.shots[4].rock, again.shots[4].rock);
    }
}
