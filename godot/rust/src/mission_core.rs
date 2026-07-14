//! `MissionCore` — the godot-free heart of the binding.
//!
//! Holds the loaded DE440 ephemeris and (once built) the [`RealFieldScenario`],
//! and answers the two questions the Godot frontend asks: *where is body N at
//! epoch t* (for the solar-system display) and *how much along-track Δv clears
//! the threat at this lead* (the headline number + planner). It deals only in
//! plain Rust / nalgebra types, so it is unit-testable with `cargo test` — no
//! running Godot — and it is the `Send` payload the 2C worker thread will build
//! off Godot's main thread. The thin [`crate::Mission`] class marshals these to
//! Godot types and never adds logic of its own.
//!
//! **Two-phase, on purpose.** [`load`](MissionCore::load) reads the kernels
//! (~ms) and immediately enables body-position queries; [`build_scenario`](
//! MissionCore::build_scenario) runs the expensive multi-year back-propagation
//! that the deflection solver needs. Splitting them lets the display show the
//! real planets the instant the kernel is in, while the scenario builds behind
//! it — and lets the fast path be exercised without paying the slow one.
//!
//! **Frame:** the core works in ICRF (equatorial J2000); the display draws in
//! the **ecliptic** plane. [`icrf_km_to_ecliptic_au`] applies the fixed J2000
//! obliquity rotation (SPICE's `ECLIPJ2000` value, 84381.448″) so the returned
//! positions sit in the ecliptic — skipping it would tilt the whole system ~23°.

use std::sync::Arc;

use anise::constants::frames::SUN_J2000;
use anise::prelude::Frame;
use nalgebra::Vector3;

use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::scenario::{ImpactorConfig, RealFieldScenario, ScenarioError};
use asteroid_core::{DvSolveTol, Epoch};

/// Kilometres per astronomical unit — the display scale positions cross into.
const AU_KM: f64 = 1.495_978_707e8;
/// Mean obliquity of the ecliptic at J2000, arcseconds — the exact value that
/// defines SPICE's `ECLIPJ2000` frame, so our ecliptic matches the kernel's.
const OBLIQUITY_ARCSEC: f64 = 84_381.448;

/// Rotate an ICRF (equatorial-J2000) position in **km** into ecliptic-J2000 and
/// scale to **AU**. A rotation by the mean obliquity about the shared X axis
/// (vernal equinox): the ecliptic north pole sits at ICRF `(0, −sinε, cosε)`.
pub fn icrf_km_to_ecliptic_au(v_km: Vector3<f64>) -> Vector3<f64> {
    let eps = OBLIQUITY_ARCSEC / 3600.0 * std::f64::consts::PI / 180.0;
    let (s, c) = eps.sin_cos();
    Vector3::new(
        v_km.x / AU_KM,
        (c * v_km.y + s * v_km.z) / AU_KM,
        (-s * v_km.y + c * v_km.z) / AU_KM,
    )
}

/// The loaded mission: always an ephemeris, optionally a built scenario.
pub struct MissionCore {
    ephemeris: Arc<Ephemeris>,
    scenario: Option<RealFieldScenario>,
}

impl MissionCore {
    /// Read the DE440 kernels named by `ASTEROID_DE_KERNEL` (the `.bsp`) and
    /// `ASTEROID_PLANETARY_CONSTANTS` (the `.pca`) and hold them. Fast (~ms):
    /// enables [`body_position_ecl_au`](Self::body_position_ecl_au) immediately;
    /// the scenario is built separately. The env-var convention matches the core
    /// tests and the `curve`/viewer binaries.
    pub fn load() -> Result<Self, ScenarioError> {
        let bsp = std::env::var("ASTEROID_DE_KERNEL")
            .map_err(|_| ScenarioError::MissingKernelEnv("ASTEROID_DE_KERNEL"))?;
        let pca = std::env::var("ASTEROID_PLANETARY_CONSTANTS")
            .map_err(|_| ScenarioError::MissingKernelEnv("ASTEROID_PLANETARY_CONSTANTS"))?;
        let eph = Ephemeris::load(&bsp)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
            .with_constants(&pca)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        Ok(Self {
            ephemeris: Arc::new(eph),
            scenario: None,
        })
    }

    /// Build the designer impactor + campaign over the already-loaded ephemeris
    /// (the **expensive** multi-year back-propagation). Enables the deflection
    /// solver ([`required_dv_along_track`](Self::required_dv_along_track)).
    pub fn build_scenario(&mut self, cfg: &ImpactorConfig) -> Result<(), ScenarioError> {
        self.scenario = Some(RealFieldScenario::build_with(
            cfg,
            Arc::clone(&self.ephemeris),
        )?);
        Ok(())
    }

    /// Whether the (expensive) scenario has been built.
    pub fn has_scenario(&self) -> bool {
        self.scenario.is_some()
    }

    /// Heliocentric **ecliptic-J2000** position of NAIF body `naif_id` at
    /// `tdb_seconds` past J2000, in **AU**. `None` if the ephemeris cannot
    /// resolve the body/epoch (out of the kernel span, unknown id). Available as
    /// soon as [`load`](Self::load) succeeds — no scenario required.
    pub fn body_position_ecl_au(&self, naif_id: i32, tdb_seconds: f64) -> Option<Vector3<f64>> {
        let frame = Frame::from_ephem_j2000(naif_id);
        let epoch = Epoch::from_tdb_seconds_past_j2000(tdb_seconds);
        self.ephemeris
            .position_km(frame, SUN_J2000, epoch.as_hifitime())
            .ok()
            .map(icrf_km_to_ecliptic_au)
    }

    /// The minimum along-track Δv (m/s) that lifts the b-plane perigee to
    /// `target_perigee_m` when applied `lead_seconds` before impact — one point
    /// of the headline curve. Errors if the scenario is not built yet.
    pub fn required_dv_along_track(
        &self,
        lead_seconds: f64,
        target_perigee_m: f64,
    ) -> Result<f64, ScenarioError> {
        let sc = self
            .scenario
            .as_ref()
            .ok_or_else(|| ScenarioError::NominalNotAHit("scenario not built".into()))?;
        let ds = sc.deflection()?;
        let deflection_epoch = sc.impact_epoch().shifted_by_seconds(-lead_seconds);
        Ok(ds.required_dv_along_track(deflection_epoch, target_perigee_m, DvSolveTol::default())?)
    }

    /// Heliocentric semi-major axis of the threat, m (0 if no scenario).
    pub fn semi_major_axis_m(&self) -> f64 {
        self.scenario.as_ref().map_or(0.0, |s| s.semi_major_axis_m)
    }

    /// Heliocentric orbital period of the threat, seconds (0 if no scenario).
    pub fn period_seconds(&self) -> f64 {
        self.scenario.as_ref().map_or(0.0, |s| s.period_seconds)
    }

    /// Impact epoch, seconds past J2000 (0 if no scenario).
    pub fn impact_tdb_seconds(&self) -> f64 {
        self.scenario
            .as_ref()
            .map_or(0.0, |s| s.impact_epoch().tdb_seconds_past_j2000())
    }

    /// Campaign-start epoch, seconds past J2000 (0 if no scenario).
    pub fn epoch0_tdb_seconds(&self) -> f64 {
        self.scenario
            .as_ref()
            .map_or(0.0, |s| s.epoch0().tdb_seconds_past_j2000())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn have_kernels() -> bool {
        std::env::var("ASTEROID_DE_KERNEL").is_ok()
            && std::env::var("ASTEROID_PLANETARY_CONSTANTS").is_ok()
    }

    /// The obliquity rotation is a pure rotation: it preserves length and leaves
    /// a vector in the equatorial plane (z_eq = 0) with its z-component still
    /// zero only along the shared X axis. Concretely, the ecliptic north pole
    /// `(0,0,1)` AU·AU_KM in ecliptic came from ICRF `(0,−sinε,cosε)`. Kernel-free.
    #[test]
    fn obliquity_rotation_is_orthonormal_about_x() {
        // A point on the ICRF x-axis is unchanged in y/z.
        let on_x = icrf_km_to_ecliptic_au(Vector3::new(AU_KM, 0.0, 0.0));
        assert!((on_x.x - 1.0).abs() < 1e-12);
        assert!(on_x.y.abs() < 1e-12 && on_x.z.abs() < 1e-12);

        // Length preserved (rotation), checked on an oblique vector.
        let v = Vector3::new(0.3 * AU_KM, -0.7 * AU_KM, 0.5 * AU_KM);
        let r = icrf_km_to_ecliptic_au(v);
        assert!((r.norm() - v.norm() / AU_KM).abs() < 1e-12);

        // The ICRF celestial pole (0,0,1) tilts to ecliptic latitude 90°−ε: its
        // ecliptic y is +sinε, z is +cosε (pole leans toward +Y in ecliptic).
        let pole = icrf_km_to_ecliptic_au(Vector3::new(0.0, 0.0, AU_KM));
        let eps = OBLIQUITY_ARCSEC / 3600.0 * std::f64::consts::PI / 180.0;
        assert!((pole.x).abs() < 1e-12);
        assert!((pole.y - eps.sin()).abs() < 1e-12);
        assert!((pole.z - eps.cos()).abs() < 1e-12);
    }

    /// Kernel-gated (release-run for speed). Loads the real DE440 kernels and
    /// checks the body-position path against physics + a *direct* ephemeris call:
    /// Earth ~1 AU from the Sun and essentially in the ecliptic plane (|z| ≪ 1),
    /// which it would NOT be (|z| up to ~0.4 AU) if the obliquity rotation were
    /// dropped — so this pins the rotation end-to-end. Skips green offline.
    #[test]
    fn body_positions_match_direct_ephemeris_and_lie_in_ecliptic() {
        if !have_kernels() {
            eprintln!("skipping body_positions_*: no DE kernel");
            return;
        }
        let mc = MissionCore::load().expect("load kernels");
        // 2035-01-01 TDB, comfortably inside the de440s span and the campaign.
        let t = Epoch::from_tdb_gregorian(2035, 1, 1, 0, 0, 0, 0).tdb_seconds_past_j2000();

        let earth = mc.body_position_ecl_au(399, t).expect("earth position");
        assert!(
            (0.98..=1.02).contains(&earth.norm()),
            "Earth heliocentric distance {:.4} AU not ~1 AU",
            earth.norm()
        );
        assert!(
            earth.z.abs() < 0.02,
            "Earth ecliptic z {:.4} AU too large — obliquity rotation likely wrong/missing",
            earth.z
        );

        // Direct ephemeris call, rotated by the same helper, must match exactly.
        let direct = mc
            .ephemeris
            .position_km(
                Frame::from_ephem_j2000(399),
                SUN_J2000,
                Epoch::from_tdb_seconds_past_j2000(t).as_hifitime(),
            )
            .expect("direct earth position");
        let direct_ecl = icrf_km_to_ecliptic_au(direct);
        assert!(
            (earth - direct_ecl).norm() < 1e-12,
            "body_position_ecl_au disagrees with a direct ephemeris call"
        );

        // Jupiter (barycenter, NAIF 5) is ~5.2 AU — a second, well-separated body.
        let jup = mc.body_position_ecl_au(5, t).expect("jupiter position");
        assert!(
            (4.9..=5.5).contains(&jup.norm()),
            "Jupiter heliocentric distance {:.3} AU not ~5.2 AU",
            jup.norm()
        );
    }

    /// Kernel-gated (release-run). Builds the default scenario and checks the
    /// binding's `required_dv_along_track` reproduces the cached `curve.json`
    /// points for the same fixed config — proving the deflection path is wired
    /// right, not just that it runs. Values are the deterministic output of
    /// `ImpactorConfig::default()`; if that config changes, regenerate curve.json
    /// and update these. Skips green offline.
    #[test]
    fn required_dv_matches_curve_json() {
        if !have_kernels() {
            eprintln!("skipping required_dv_matches_curve_json: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        let target = 2.0e7; // curve.json target_perigee_m
                            // (lead_seconds, required_dv) pairs straight from curve.json.
        let cases = [
            (12_464_104.312150536_f64, 0.587_75_f64), // 0.5 period
            (24_928_208.624301072, 0.509_75),         // 1.0 period
            (49_856_417.248602144, 0.255_125),        // 2.0 periods
        ];
        for (lead, expected) in cases {
            let dv = mc.required_dv_along_track(lead, target).expect("dv solve");
            let rel = (dv - expected).abs() / expected;
            assert!(
                rel < 0.02,
                "lead {lead:.0}s: dv {dv:.5} vs curve.json {expected:.5} (rel {rel:.3})"
            );
        }
    }
}
