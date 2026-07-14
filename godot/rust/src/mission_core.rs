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

use anise::constants::frames::{SSB_J2000, SUN_J2000};
use anise::prelude::Frame;
use nalgebra::Vector3;

use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::scenario::{ImpactorConfig, RealFieldScenario, ScenarioError};
use asteroid_core::{along_track_unit, Clock, DvSolveTol, Epoch};

/// Kilometres per astronomical unit — the display scale positions cross into.
const AU_KM: f64 = 1.495_978_707e8;
/// Metres per kilometre — the integrated `Clock` stores SSB positions in metres,
/// but [`icrf_km_to_ecliptic_au`] takes kilometres, so we scale down first.
const M_PER_KM: f64 = 1.0e3;
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

/// A committed deflection plan and its precomputed result.
///
/// `perigee == None` is the **clean-miss success case** — the deflected pass left
/// the scan gate, i.e. the miss is so wide it is off any sensible frame, which is
/// exactly what the player wants. It must stay distinct from "no plan set" (that
/// is `MissionCore::plan == None`), so the planner does not read the *best*
/// deflection as a failure.
struct PlanState {
    /// The deflection epoch, seconds past J2000 — before this the impulse has not
    /// happened, so deflected queries fall back to the nominal track.
    deflection_seconds: f64,
    /// The post-impulse arc: a `Clock` covering `[deflection_epoch, span_end]`.
    clock: Clock,
    /// The deflected b-plane perigee (miss distance), m, or `None` for a clean
    /// miss that left the scan gate (see the struct note).
    perigee: Option<f64>,
}

/// The loaded mission: always an ephemeris, optionally a built scenario, and —
/// once built — the cached nominal trajectory and (optionally) a deflection plan.
pub struct MissionCore {
    ephemeris: Arc<Ephemeris>,
    scenario: Option<RealFieldScenario>,
    /// The nominal (un-deflected) trajectory, cloned **once** at build time so
    /// per-frame position/track reads are cheap `Clock` queries. Rebuilding a
    /// `DeflectionScenario` re-propagates the whole multi-year nominal
    /// (`deflection.rs`), so we never do that on a display read.
    nominal_clock: Option<Clock>,
    /// The current deflection plan, recomputed only on [`set_plan`](Self::set_plan)
    /// and read cheaply thereafter.
    plan: Option<PlanState>,
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
            nominal_clock: None,
            plan: None,
        })
    }

    /// Build the designer impactor + campaign over the already-loaded ephemeris
    /// (the **expensive** multi-year back-propagation). Enables the deflection
    /// solver ([`required_dv_along_track`](Self::required_dv_along_track)).
    pub fn build_scenario(&mut self, cfg: &ImpactorConfig) -> Result<(), ScenarioError> {
        let scenario = RealFieldScenario::build_with(cfg, Arc::clone(&self.ephemeris))?;
        // Cache the nominal trajectory once so display reads never re-propagate it.
        // (`deflection()` builds a fresh `DeflectionScenario`, which propagates the
        // full nominal; we pay that here at build time, not per frame.)
        let nominal_clock = scenario.deflection()?.nominal().clone();
        self.scenario = Some(scenario);
        self.nominal_clock = Some(nominal_clock);
        self.plan = None; // a new scenario invalidates any prior plan
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

    /// Heliocentric **ecliptic-J2000 AU** from an SSB-relative position in
    /// **metres** (the frame the integrated asteroid `Clock` stores), at `epoch`.
    ///
    /// Subtracts the Sun's SSB position first, so the result lands in the *same*
    /// frame [`body_position_ecl_au`](Self::body_position_ecl_au) puts the planets
    /// in (Sun-relative ecliptic AU); dropping that subtraction would offset the
    /// asteroid from its own drawn orbit by the Sun's barycentric wobble (~1e6 km).
    /// `icrf_km_to_ecliptic_au` wants kilometres, so the metres are scaled down
    /// before the rotation. `None` if the Sun position cannot be resolved.
    fn ssb_m_to_helio_ecl_au(&self, ssb_m: Vector3<f64>, epoch: Epoch) -> Option<Vector3<f64>> {
        let sun_km = self
            .ephemeris
            .position_km(SUN_J2000, SSB_J2000, epoch.as_hifitime())
            .ok()?;
        let helio_km = ssb_m / M_PER_KM - sun_km;
        Some(icrf_km_to_ecliptic_au(helio_km))
    }

    /// Nominal (un-deflected) threat position, heliocentric **ecliptic AU**, at
    /// `tdb` seconds past J2000 — the asteroid on the solar-system display, in the
    /// same frame as [`body_position_ecl_au`](Self::body_position_ecl_au). `None`
    /// before the scenario is built or for an epoch outside the propagated span.
    pub fn asteroid_position_ecl_au(&self, tdb: f64) -> Option<Vector3<f64>> {
        let clock = self.nominal_clock.as_ref()?;
        let epoch = Epoch::from_tdb_seconds_past_j2000(tdb);
        let st = clock.state_at(epoch).ok()?;
        self.ssb_m_to_helio_ecl_au(st.position, epoch)
    }

    /// Deflected threat position, heliocentric **ecliptic AU**, at `tdb`.
    ///
    /// Before the plan's deflection epoch the impulse has not been applied, so
    /// this returns the nominal position — otherwise the nudge would appear to act
    /// retroactively. After it, the post-impulse arc. `None` if no plan is set or
    /// the epoch is out of span.
    pub fn deflected_position_ecl_au(&self, tdb: f64) -> Option<Vector3<f64>> {
        let plan = self.plan.as_ref()?;
        let epoch = Epoch::from_tdb_seconds_past_j2000(tdb);
        let st = if tdb < plan.deflection_seconds {
            self.nominal_clock.as_ref()?.state_at(epoch).ok()?
        } else {
            plan.clock.state_at(epoch).ok()?
        };
        self.ssb_m_to_helio_ecl_au(st.position, epoch)
    }

    /// Sample an SSB-position function over `[t0, t1]` at `n` (≥ 2) uniform epochs
    /// and map each into heliocentric ecliptic AU — the shared body of the track
    /// samplers below. Points whose lookup fails are dropped; within a propagated
    /// span (the only way these are called) none do, so the polyline stays whole.
    fn track_ecl_au<F>(&self, n: usize, t0: f64, t1: f64, ssb_at: F) -> Vec<Vector3<f64>>
    where
        F: Fn(Epoch) -> Option<Vector3<f64>>,
    {
        let n = n.max(2);
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let frac = i as f64 / (n - 1) as f64;
            let t = t0 + (t1 - t0) * frac;
            let epoch = Epoch::from_tdb_seconds_past_j2000(t);
            if let Some(au) = ssb_at(epoch).and_then(|p| self.ssb_m_to_helio_ecl_au(p, epoch)) {
                out.push(au);
            }
        }
        out
    }

    /// The nominal threat arc from campaign start to impact as `n` heliocentric
    /// ecliptic-AU points (the orbit polyline). The caller samples this **once**;
    /// the reads are cheap but it walks the whole span. Empty if no scenario.
    pub fn asteroid_track_ecl_au(&self, n: usize) -> Vec<Vector3<f64>> {
        let (Some(clock), Some(sc)) = (self.nominal_clock.as_ref(), self.scenario.as_ref()) else {
            return Vec::new();
        };
        let t0 = sc.epoch0().tdb_seconds_past_j2000();
        let t1 = sc.impact_epoch().tdb_seconds_past_j2000();
        self.track_ecl_au(n, t0, t1, |e| clock.state_at(e).ok().map(|s| s.position))
    }

    /// The deflected threat arc from campaign start to impact as `n` heliocentric
    /// ecliptic-AU points: the nominal track up to the deflection epoch, the
    /// post-impulse arc after it (same guard as
    /// [`deflected_position_ecl_au`](Self::deflected_position_ecl_au)). Empty if
    /// no plan is set.
    pub fn deflected_track_ecl_au(&self, n: usize) -> Vec<Vector3<f64>> {
        let (Some(nom), Some(sc), Some(plan)) = (
            self.nominal_clock.as_ref(),
            self.scenario.as_ref(),
            self.plan.as_ref(),
        ) else {
            return Vec::new();
        };
        let t0 = sc.epoch0().tdb_seconds_past_j2000();
        let t1 = sc.impact_epoch().tdb_seconds_past_j2000();
        self.track_ecl_au(n, t0, t1, |e| {
            let clk = if e.tdb_seconds_past_j2000() < plan.deflection_seconds {
                nom
            } else {
                &plan.clock
            };
            clk.state_at(e).ok().map(|s| s.position)
        })
    }

    /// Commit a deflection plan: an **along-track** impulse of `dv_along_track`
    /// (m/s) applied `lead_seconds` before impact. Recomputes and caches the
    /// deflected arc and its b-plane perigee.
    ///
    /// **Expensive** — it rebuilds the `DeflectionScenario` (re-propagating the
    /// nominal) to find the along-track heading and re-propagate the deflected
    /// arc. Call on a plan change, never per frame. Read the result cheaply via
    /// [`deflected_perigee_m`](Self::deflected_perigee_m) /
    /// [`is_clean_miss`](Self::is_clean_miss) and the deflected position/track.
    pub fn set_plan(
        &mut self,
        lead_seconds: f64,
        dv_along_track: f64,
    ) -> Result<(), ScenarioError> {
        let sc = self
            .scenario
            .as_ref()
            .ok_or_else(|| ScenarioError::NominalNotAHit("scenario not built".into()))?;
        let deflection_epoch = sc.impact_epoch().shifted_by_seconds(-lead_seconds);
        let ds = sc.deflection()?;
        let seed = ds
            .nominal()
            .state_at(deflection_epoch)
            .map_err(|e| ScenarioError::Integration(e.to_string()))?;
        let dir = along_track_unit(seed).ok_or_else(|| {
            ScenarioError::Integration("nominal has zero velocity; no along-track heading".into())
        })?;
        let (clock, enc) = ds.deflected_trajectory(deflection_epoch, dv_along_track * dir)?;
        self.plan = Some(PlanState {
            deflection_seconds: deflection_epoch.tdb_seconds_past_j2000(),
            clock,
            perigee: enc.map(|e| e.perigee),
        });
        Ok(())
    }

    /// Whether a deflection plan is currently set.
    pub fn has_plan(&self) -> bool {
        self.plan.is_some()
    }

    /// Whether the current plan's deflected pass left the scan gate — a clean,
    /// wide miss (the **success** case), distinct from "no plan" / "solve failed".
    pub fn is_clean_miss(&self) -> bool {
        self.plan.as_ref().is_some_and(|p| p.perigee.is_none())
    }

    /// The deflected b-plane perigee (miss distance), m — `None` if no plan is set
    /// **or** the pass is a clean miss (use [`is_clean_miss`](Self::is_clean_miss)
    /// to tell those two apart).
    pub fn deflected_perigee_m(&self) -> Option<f64> {
        self.plan.as_ref().and_then(|p| p.perigee)
    }

    /// The current plan's deflection epoch, seconds past J2000 (`None` if no plan).
    pub fn plan_deflection_tdb_seconds(&self) -> Option<f64> {
        self.plan.as_ref().map(|p| p.deflection_seconds)
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

    /// Kernel-gated (release-run). The single most decisive frame check: the
    /// threat *hits Earth* at the impact epoch by construction, so its heliocentric
    /// ecliptic-AU position there must coincide with Earth's to within the
    /// conditioned b-offset (~3000 km ≈ 2e-5 AU) plus round-trip integration error.
    /// This one assertion exercises the whole threat-frame chain end-to-end — the
    /// SSB→heliocentric subtraction, the m→km scaling, and the obliquity rotation:
    /// a missing Sun subtraction shows as a ~1e6 km gap, a m-vs-km slip as ~1000×.
    /// Far sharper than "distance ≈ a". Also pins the track sampler's length.
    #[test]
    fn asteroid_position_coincides_with_earth_at_impact() {
        if !have_kernels() {
            eprintln!("skipping asteroid_position_coincides_with_earth_at_impact: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        let t_impact = mc.impact_tdb_seconds();
        let ast = mc
            .asteroid_position_ecl_au(t_impact)
            .expect("asteroid position at impact");
        let earth = mc
            .body_position_ecl_au(399, t_impact)
            .expect("earth position at impact");

        // Sane heliocentric band first — a wholly wrong frame (barycentric, or
        // km/m confusion) lands far outside this.
        assert!(
            (0.3..=3.0).contains(&ast.norm()),
            "threat heliocentric distance {:.4} AU is not in a sane band",
            ast.norm()
        );
        // The decisive coincidence: at impact the asteroid is on top of Earth.
        let gap_au = (ast - earth).norm();
        assert!(
            gap_au < 1.0e-3,
            "threat-Earth gap at impact {gap_au:.3e} AU too large — frame chain wrong \
             (Sun subtraction / km-vs-m / obliquity)"
        );

        // The track sampler returns exactly n points (no silent drops in-span).
        let track = mc.asteroid_track_ecl_au(200);
        assert_eq!(
            track.len(),
            200,
            "nominal track should be a full n-point line"
        );
        assert!(
            track.iter().all(|p| (0.2..=4.0).contains(&p.norm())),
            "every track point should sit at a plausible heliocentric distance"
        );
    }

    /// Kernel-gated (release-run). The deflected surface obeys causality and the
    /// success-sentinel contract: before the deflection epoch the deflected
    /// position equals the nominal (the impulse has not acted yet); at impact it
    /// has moved; and exactly one of `is_clean_miss` / `deflected_perigee_m`
    /// carries the result (never both, never neither once a plan is set).
    #[test]
    fn deflected_surface_respects_causality_and_sentinels() {
        if !have_kernels() {
            eprintln!("skipping deflected_surface_respects_causality_and_sentinels: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        assert!(!mc.has_plan(), "no plan before set_plan");
        assert_eq!(mc.deflected_perigee_m(), None);
        assert!(!mc.is_clean_miss());

        // A modest along-track nudge one heliocentric period before impact.
        let lead = mc.period_seconds();
        mc.set_plan(lead, 0.1).expect("set_plan succeeds");
        assert!(mc.has_plan());

        let t_defl = mc.plan_deflection_tdb_seconds().expect("plan epoch");

        // Before the deflection epoch: deflected == nominal (no retroactive nudge).
        let t_before = t_defl - 1.0e6;
        let nom_before = mc
            .asteroid_position_ecl_au(t_before)
            .expect("nominal before defl");
        let defl_before = mc
            .deflected_position_ecl_au(t_before)
            .expect("deflected before defl");
        assert!(
            (nom_before - defl_before).norm() < 1.0e-12,
            "deflected position before the deflection epoch must equal the nominal"
        );

        // At impact: the deflected track has moved off the nominal.
        let t_impact = mc.impact_tdb_seconds();
        let nom_impact = mc
            .asteroid_position_ecl_au(t_impact)
            .expect("nominal at impact");
        let defl_impact = mc
            .deflected_position_ecl_au(t_impact)
            .expect("deflected at impact");
        assert!(
            (nom_impact - defl_impact).norm() > 1.0e-9,
            "a 0.1 m/s nudge one period out should visibly move the impact-epoch position"
        );

        // Sentinel contract: with a plan set, exactly one of the two reads the
        // outcome — a finite perigee XOR a clean (off-gate) miss.
        assert_ne!(
            mc.is_clean_miss(),
            mc.deflected_perigee_m().is_some(),
            "clean-miss and finite-perigee must be mutually exclusive with a plan set"
        );

        // The deflected track is a full n-point line too.
        assert_eq!(mc.deflected_track_ecl_au(150).len(), 150);
    }
}
