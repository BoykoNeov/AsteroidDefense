//! Designer Earth-impactor over the real DE440 field, and the Δv-vs-lead-time
//! sweep across it (HANDOFF §1, §4, §5, §10 task 10).
//!
//! This module turns the crate's kernel-free deflection machinery into a
//! *concrete mission*: an asteroid that genuinely strikes Earth years from now
//! under the full Tier-1 ephemeris field, and the headline curve — how small an
//! along-track nudge still turns that strike into a safe miss, as a function of
//! how early it is applied.
//!
//! It lives in `core` (not a renderer crate) so **both** the egui viewer and the
//! Godot gdext binding drive the *same* validated scenario — one source of truth
//! for the drawn tracks and the headline numbers. It is deliberately **serde-free**
//! (the workspace keeps serde out of core); [`sweep`](RealFieldScenario::sweep)
//! returns plain [`SweepPoint`]s that a renderer wraps in its own serialisable
//! form when it needs to cache the curve to disk.
//!
//! # Why a *designer* impactor (back-propagation)
//! You cannot pick heliocentric elements and hope they hit Earth in 2040. So we
//! run the encounter geometry backward: fix the impact — Earth's ephemeris state
//! at a chosen epoch, plus a hyperbolic relative velocity and a small
//! perpendicular offset (a *conditioned* hit, perigee inside the capture radius)
//! — and integrate that state **backward** to the campaign start. Forward
//! propagation from the resulting seed then reproduces the impact by
//! construction (to the integrator tolerance, which [`RealFieldScenario::build`]
//! verifies by asserting the nominal encounter still reads as a hit).
//!
//! # Why `v_rel ≥ ~15 km/s`
//! The b-plane reduction needs a hyperbolic relative orbit at closest approach.
//! With the *real* massive Earth in the field the encounter is a genuine
//! hyperbola whenever the relative speed clears Earth escape (~11.2 km/s at the
//! surface); seeding well above that keeps every probe along the whole Δv curve
//! cleanly hyperbolic, sidestepping the massless-Earth `NotHyperbolic` edge the
//! core solver only folds in as a fallback.

use std::error::Error;
use std::fmt;
use std::sync::Arc;

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use nalgebra::Vector3;

use crate::ephemeris::{Ephemeris, KM3_S2_TO_M3_S2};
use crate::forces::point_mass::PointMassGravity;
use crate::geometry::BPlaneEncounter;
use crate::perturber_field::{tier1_perturber_field, EphemerisPerturber};
use crate::{
    geometry, Clock, DeflectionError, DeflectionScenario, Dop853, DvSolveTol, Epoch, Integrator,
    ScanOptions, StateVector,
};

/// Metres per kilometre — the km→m scale the DE440 states cross into SI on.
const KM_TO_M: f64 = 1.0e3;
/// Seconds in a Julian year (365.25 d), for lead-time bookkeeping.
const SECONDS_PER_YEAR: f64 = 365.25 * 86_400.0;

/// Half-width of the encounter window the animation samples, seconds. ±1.5 days
/// brackets the fast (18 km/s) pass with room for a modestly time-shifted
/// deflected closest approach. Shared by renderers and their tests so a test
/// exercises the resolution the app actually renders.
pub const ENCOUNTER_HALF_WINDOW_SECONDS: f64 = 1.5 * 86_400.0;
/// Samples across the encounter window — dense enough that the track is smooth
/// through the tight turn near closest approach.
pub const ENCOUNTER_SAMPLES: usize = 1_400;

/// The knobs that define a designer impactor and the campaign around it.
///
/// [`Default`] is a ~12-year, multi-revolution campaign: a fast (18 km/s
/// relative) hyperbolic strike in 2040, seeded far enough back that the headline
/// curve spans several heliocentric orbits — the regime where the `Δv ∝ 1/lead`
/// falloff actually appears (a single sub-orbital arc cannot show it).
#[derive(Debug, Clone, Copy)]
pub struct ImpactorConfig {
    /// The impact epoch — where the asteroid meets Earth. The campaign runs from
    /// `impact_epoch − lead_years` up to here (plus a margin).
    pub impact_epoch: Epoch,
    /// Lead time of the campaign start before impact, Julian years. The seed is
    /// the impact state integrated backward this far.
    pub lead_years: f64,
    /// Relative speed at impact, km/s. Keep ≥ ~15 so every encounter along the
    /// curve is cleanly hyperbolic (see the module note).
    pub v_rel_kms: f64,
    /// Direction of the relative velocity at impact (need not be unit; it is
    /// normalized). Sets the heliocentric orbit the seed lands on, so it also
    /// governs the orbital period — [`RealFieldScenario::build`] reports the
    /// resulting `a`/`T` so a choice that is unbound or barely sub-orbital shows.
    pub v_rel_dir: Vector3<f64>,
    /// Perpendicular offset of the asteroid from Earth's centre at impact, km —
    /// a *conditioned* hit (inside the capture radius, above dead-centre so the
    /// b-plane geometry is well posed).
    pub b_offset_km: f64,
    /// Snapshot cadence of the propagated clock, days. Dense output serves
    /// sub-cadence queries, so this trades storage/step-count, not accuracy.
    pub cadence_days: f64,
    /// How far past the impact epoch to propagate, days — a margin so a deflected
    /// (time-shifted) pass still lands inside the span.
    pub span_margin_days: f64,
    /// Relative tolerance for the **backward** seed integration. Tight, because
    /// this fixes how faithfully the forward pass reproduces the designed impact.
    pub back_rtol: f64,
}

impl ImpactorConfig {
    /// The campaign-start epoch this config implies: `impact_epoch − lead_years`.
    ///
    /// Both inputs are *given*, so this is knowable without
    /// [`RealFieldScenario::build`] — worth having separately, because building
    /// costs a multi-year back-propagation while a caller that only needs to
    /// place the campaign on a timeline (the Godot frontend's clock) needs no
    /// trajectory at all. `build_with` calls this too, so the two can never
    /// disagree about when the campaign starts.
    pub fn epoch0(&self) -> Epoch {
        self.impact_epoch
            .shifted_by_seconds(-self.lead_years * SECONDS_PER_YEAR)
    }
}

impl Default for ImpactorConfig {
    fn default() -> Self {
        Self {
            // 2040-01-01 TDB.
            impact_epoch: Epoch::from_tdb_gregorian(2040, 1, 1, 0, 0, 0, 0),
            lead_years: 12.0,
            v_rel_kms: 18.0,
            // A generic oblique approach; the builder reports the orbit it yields.
            v_rel_dir: Vector3::new(0.6, -0.7, 0.2),
            b_offset_km: 3_000.0,
            cadence_days: 1.0,
            span_margin_days: 60.0,
            back_rtol: 1.0e-12,
        }
    }
}

/// Everything downstream failure mode of building/sweeping a scenario, unified so
/// every consumer (binary, egui app, gdext binding) surfaces one error type.
#[derive(Debug)]
pub enum ScenarioError {
    /// A required kernel-path env var was unset (`ASTEROID_DE_KERNEL` /
    /// `ASTEROID_PLANETARY_CONSTANTS`).
    MissingKernelEnv(&'static str),
    /// Loading the DE kernel or its planetary constants failed.
    Ephemeris(String),
    /// A backward/forward integration failed.
    Integration(String),
    /// The chosen geometry put the seed on an unbound (or degenerate)
    /// heliocentric orbit — no finite period, so "lead in orbits" is undefined.
    /// Carries the computed semi-major axis (m; ≤ 0 or non-finite).
    UnboundOrbit(f64),
    /// The forward pass did not reproduce a hit — the designed impact did not
    /// round-trip through back-then-forward integration at the chosen tolerance.
    NominalNotAHit(String),
    /// A deflection evaluation/solve failed.
    Deflection(DeflectionError),
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ScenarioError::MissingKernelEnv(v) => {
                write!(
                    f,
                    "environment variable {v} is not set (kernel path required)"
                )
            }
            ScenarioError::Ephemeris(m) => write!(f, "ephemeris load failed: {m}"),
            ScenarioError::Integration(m) => write!(f, "integration failed: {m}"),
            ScenarioError::UnboundOrbit(a) => write!(
                f,
                "seed is on a non-bound heliocentric orbit (a = {a:.3e} m); \
                 choose a smaller v_rel or a different direction"
            ),
            ScenarioError::NominalNotAHit(m) => {
                write!(f, "designed impact did not round-trip: {m}")
            }
            ScenarioError::Deflection(e) => write!(f, "deflection solve failed: {e}"),
        }
    }
}

impl Error for ScenarioError {}

impl From<DeflectionError> for ScenarioError {
    fn from(e: DeflectionError) -> Self {
        ScenarioError::Deflection(e)
    }
}

/// A built, ready-to-sweep impact scenario over the real DE440 field.
///
/// Owns the loaded ephemeris (shared into the field and the Earth-state source),
/// the Tier-1 force model, the seed, and the campaign geometry. [`Self::sweep`]
/// then reads off the headline Δv-vs-lead curve. The struct owns the borrowables
/// so a [`DeflectionScenario`] can borrow them per call ([`Self::deflection`]).
pub struct RealFieldScenario {
    /// The DE440 almanac (kept alive; the field and Earth source hold `Arc`s).
    ephemeris: Arc<Ephemeris>,
    force: PointMassGravity,
    earth: EphemerisPerturber,
    mu_earth: f64,
    earth_radius: f64,
    scan: ScanOptions,

    epoch0: Epoch,
    seed: StateVector,
    impact_epoch: Epoch,
    cadence_seconds: f64,
    n_snapshots: u32,

    /// Heliocentric semi-major axis of the seed, m (> 0; bound).
    pub semi_major_axis_m: f64,
    /// Heliocentric orbital period of the seed, seconds.
    pub period_seconds: f64,
}

/// One point of the headline curve: the minimum along-track Δv that raises the
/// b-plane perigee to the safe target, applied `lead_seconds` before impact.
///
/// Plain data (no serde — core stays serde-free); a renderer that caches the
/// curve wraps this in its own serialisable form.
#[derive(Debug, Clone, Copy)]
pub struct SweepPoint {
    /// Lead time before impact, seconds.
    pub lead_seconds: f64,
    /// Lead time expressed in heliocentric orbital periods.
    pub lead_periods: f64,
    /// Minimum along-track Δv to clear the target perigee, m/s.
    pub required_dv: f64,
}

/// The encounter drawn in Earth's frame: both asteroid tracks sampled *relative
/// to Earth's geocentre* over a window centred on the nominal impact, plus the
/// b-plane numbers that annotate them.
///
/// This is the "Earth slides out of the way" picture (HANDOFF §1, §10.10) at the
/// only scale where the miss is visible: the heliocentric orbit is ~1.3e8 km but
/// the safe miss is ~2e4 km, so a single frame cannot hold both the deflection
/// point and the pass. Here Earth sits at the origin (a disc of `earth_radius`,
/// with the focused `capture_radius` as the collision cross-section); the nominal
/// track spears that disc (`nominal_perigee ≤ capture_radius`, the hit) and the
/// deflected track clears it. The displayed miss is `deflected_perigee` — the
/// *same* validated b-plane number the curve solver uses, taken from the one
/// propagation that produced the drawn deflected track, so the visual cannot
/// silently disagree with the physics.
#[derive(Debug, Clone)]
pub struct EncounterFrame {
    /// Sample epochs, seconds past J2000 (shared by both tracks, ascending).
    pub sample_seconds: Vec<f64>,
    /// Nominal (un-deflected) asteroid position relative to Earth's geocentre, m.
    pub nominal: Vec<Vector3<f64>>,
    /// Deflected asteroid position relative to Earth's geocentre, m — the impulse
    /// is applied at `deflection_epoch`; window samples all lie after it.
    pub deflected: Vec<Vector3<f64>>,
    /// Earth's solid-body radius, m (the disc to draw).
    pub earth_radius: f64,
    /// Gravitationally-focused capture radius at the nominal encounter, m — the
    /// effective collision cross-section; a perigee inside it is a hit.
    pub capture_radius: f64,
    /// Nominal b-plane perigee, m (≤ `capture_radius`: the hit being undone).
    pub nominal_perigee: f64,
    /// Deflected b-plane perigee, m, or `None` if the deflected pass left the scan
    /// gate entirely (a miss so wide it is off any sensible frame).
    pub deflected_perigee: Option<f64>,
}

impl RealFieldScenario {
    /// Load the DE440 field, design the impactor per `cfg`, back-propagate the
    /// seed, and verify the nominal reproduces a hit.
    ///
    /// Kernel paths come from `ASTEROID_DE_KERNEL` (the `.bsp`) and
    /// `ASTEROID_PLANETARY_CONSTANTS` (the `.pca`), matching the core test/env
    /// convention.
    pub fn build(cfg: &ImpactorConfig) -> Result<Self, ScenarioError> {
        let bsp = std::env::var("ASTEROID_DE_KERNEL")
            .map_err(|_| ScenarioError::MissingKernelEnv("ASTEROID_DE_KERNEL"))?;
        let pca = std::env::var("ASTEROID_PLANETARY_CONSTANTS")
            .map_err(|_| ScenarioError::MissingKernelEnv("ASTEROID_PLANETARY_CONSTANTS"))?;

        let eph = Ephemeris::load(&bsp)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
            .with_constants(&pca)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        Self::build_with(cfg, Arc::new(eph))
    }

    /// Build from an already-loaded ephemeris (the kernels resolved elsewhere).
    /// [`build`](Self::build) is the env-var convenience over this; a binding that
    /// loads the kernel itself (surfacing its own error to the UI) calls here.
    pub fn build_with(cfg: &ImpactorConfig, eph: Arc<Ephemeris>) -> Result<Self, ScenarioError> {
        let force =
            tier1_perturber_field(&eph).map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        let earth = EphemerisPerturber::new(Arc::clone(&eph), EARTH_J2000);
        let sun = EphemerisPerturber::new(Arc::clone(&eph), SUN_J2000);

        let mu_earth = eph
            .gm_km3_s2(EARTH_J2000)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
            * KM3_S2_TO_M3_S2;
        let mu_sun = eph
            .gm_km3_s2(SUN_J2000)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
            * KM3_S2_TO_M3_S2;
        let earth_radius = geometry::EARTH_EQUATORIAL_RADIUS_M;

        // --- Design the impact state (§ module note) --------------------------
        let earth_impact = earth
            .state_at(cfg.impact_epoch)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        let vdir = cfg.v_rel_dir.normalize();
        // A unit vector perpendicular to the relative velocity, for the offset.
        let perp = {
            let seed_axis = if vdir.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::y()
            };
            let p = vdir.cross(&seed_axis);
            p / p.norm()
        };
        let impact_pos = earth_impact.position + cfg.b_offset_km * KM_TO_M * perp;
        let impact_vel = earth_impact.velocity + cfg.v_rel_kms * KM_TO_M * vdir;
        let impact_state = StateVector::new(impact_pos, impact_vel);

        // --- Back-propagate to the campaign start with a tight tolerance ------
        let lead_seconds = cfg.lead_years * SECONDS_PER_YEAR;
        let epoch0 = cfg.epoch0();
        let back = Dop853::new().with_rtol(cfg.back_rtol);
        let seed = back
            .step(&force, cfg.impact_epoch, &impact_state, -lead_seconds)
            .map_err(|e| ScenarioError::Integration(e.to_string()))?;

        // --- Heliocentric a, T of the seed (vis-viva, Sun-relative) -----------
        let sun0 = sun
            .state_at(epoch0)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        let r = (seed.position - sun0.position).norm();
        let v2 = (seed.velocity - sun0.velocity).norm_squared();
        let a = 1.0 / (2.0 / r - v2 / mu_sun);
        if !(a.is_finite() && a > 0.0) {
            return Err(ScenarioError::UnboundOrbit(a));
        }
        let period = std::f64::consts::TAU * (a * a * a / mu_sun).sqrt();

        let cadence_seconds = cfg.cadence_days * 86_400.0;
        let total_span = lead_seconds + cfg.span_margin_days * 86_400.0;
        let n_snapshots = (total_span / cadence_seconds).ceil().max(1.0) as u32;

        let scan = ScanOptions {
            max_sample_dt: 6.0 * 3600.0,
            time_tol_seconds: 1.0e-3,
            max_distance: Some(5.0e8),
        };

        let scenario = Self {
            ephemeris: eph,
            force,
            earth,
            mu_earth,
            earth_radius,
            scan,
            epoch0,
            seed,
            impact_epoch: cfg.impact_epoch,
            cadence_seconds,
            n_snapshots,
            semi_major_axis_m: a,
            period_seconds: period,
        };

        // --- Verify the round-trip: the nominal must still read as a hit ------
        let ds = scenario.deflection()?;
        match ds.nominal_encounter()? {
            Some(enc) if enc.is_hit() => {}
            Some(enc) => {
                return Err(ScenarioError::NominalNotAHit(format!(
                    "perigee {:.3e} m ≥ capture radius {:.3e} m (not a hit)",
                    enc.perigee, enc.capture_radius
                )))
            }
            None => {
                return Err(ScenarioError::NominalNotAHit(
                    "no close approach inside the scan gate".into(),
                ))
            }
        }

        Ok(scenario)
    }

    /// The loaded ephemeris this scenario owns — shared (`Arc`) so a binding can
    /// serve body positions for the display from the *same* kernel the physics
    /// runs on, with no second load.
    pub fn ephemeris(&self) -> &Arc<Ephemeris> {
        &self.ephemeris
    }

    /// Build a [`DeflectionScenario`] borrowing this scenario's owned field and
    /// Earth-state source — the object the Δv solver runs on.
    pub fn deflection(&self) -> Result<DeflectionScenario<'_>, DeflectionError> {
        DeflectionScenario::new(
            Dop853::new(),
            &self.force,
            &self.earth,
            self.epoch0,
            self.seed,
            self.cadence_seconds,
            self.n_snapshots,
            self.scan,
            self.mu_earth,
            self.earth_radius,
        )
    }

    /// The campaign-start epoch (the seed's epoch).
    pub fn epoch0(&self) -> Epoch {
        self.epoch0
    }

    /// The impact epoch.
    pub fn impact_epoch(&self) -> Epoch {
        self.impact_epoch
    }

    /// Free-propagate an arbitrary seed state through this scenario's validated
    /// Tier-1 field into a dense-output [`Clock`] over
    /// `[epoch0, epoch0 + n_snapshots·cadence_seconds]`.
    ///
    /// This is the orrery / sandbox propagation path (HANDOFF §7): any body — a
    /// synthetic designer comet, a what-if trajectory — flies in the *same* DE440
    /// field the deflection physics runs on, so the drawn multi-body scene and the
    /// mission share one force model (no second field build, one source of truth).
    ///
    /// The seed is **SSB-relative** — the integration frame, barycentric ICRF in
    /// SI (metres, m/s) — matching what the nominal [`Clock`] stores; a caller
    /// holding heliocentric or ecliptic elements converts into that frame first
    /// (element→state about the Sun, rotate to ICRF, add the Sun's SSB state).
    ///
    /// `cadence_seconds`'s **sign sets the direction**: a negative cadence
    /// reconstructs the past for a reverse-time view, and the dense output serves
    /// cheap sub-cadence scrub queries either way ([`Clock::state_at`]).
    ///
    /// The span is bounded by the loaded kernel's coverage: the field looks up
    /// planet positions at every step, so a span reaching outside DE440 fails with
    /// [`ScenarioError::Integration`] rather than extrapolating. Invalid arguments
    /// (a zero/non-finite cadence, or `n_snapshots == 0`) also return that error
    /// instead of panicking, so a binding can surface them as a status.
    pub fn propagate_free(
        &self,
        epoch0: Epoch,
        seed: StateVector,
        cadence_seconds: f64,
        n_snapshots: u32,
    ) -> Result<Clock, ScenarioError> {
        if !(cadence_seconds.is_finite() && cadence_seconds != 0.0) {
            return Err(ScenarioError::Integration(
                "propagate_free cadence must be finite and non-zero".into(),
            ));
        }
        if n_snapshots < 1 {
            return Err(ScenarioError::Integration(
                "propagate_free needs at least one snapshot".into(),
            ));
        }
        Clock::propagate(
            &Dop853::new(),
            &self.force,
            epoch0,
            seed,
            cadence_seconds,
            n_snapshots,
        )
        .map_err(|e| ScenarioError::Integration(e.to_string()))
    }

    /// Sweep the headline curve: for each lead in `leads_periods` (units of the
    /// heliocentric period), solve the minimum along-track Δv that lifts the
    /// b-plane perigee to `target_perigee_m`.
    ///
    /// A lead that would fall before the campaign start (`> lead_years` worth of
    /// periods) is clamped to the start epoch. Each point is an independent
    /// bracket+bisect solve, so this is the expensive call — the whole reason a
    /// renderer times it before deciding on-thread vs. background.
    pub fn sweep(
        &self,
        leads_periods: &[f64],
        target_perigee_m: f64,
    ) -> Result<Vec<SweepPoint>, ScenarioError> {
        let ds = self.deflection()?;
        let tol = DvSolveTol::default();
        let t_impact = self.impact_epoch.tdb_seconds_past_j2000();
        let t0 = self.epoch0.tdb_seconds_past_j2000();

        let mut points = Vec::with_capacity(leads_periods.len());
        for &lp in leads_periods {
            let mut lead_seconds = lp * self.period_seconds;
            // Clamp a lead that would precede the campaign start.
            let earliest_lead = t_impact - t0;
            if lead_seconds > earliest_lead {
                lead_seconds = earliest_lead;
            }
            let deflection_epoch = self.impact_epoch.shifted_by_seconds(-lead_seconds);
            let dv = ds.required_dv_along_track(deflection_epoch, target_perigee_m, tol)?;
            points.push(SweepPoint {
                lead_seconds,
                lead_periods: lead_seconds / self.period_seconds,
                required_dv: dv,
            });
        }
        Ok(points)
    }

    /// Sample the encounter in Earth's frame for the animation: both asteroid
    /// tracks relative to Earth's geocentre over a `±half_window_seconds` window
    /// centred on the impact epoch, with `n_samples` points, after an along-track
    /// (or arbitrary) impulse `delta_v` applied at `deflection_epoch`.
    ///
    /// This convenience builds a fresh [`DeflectionScenario`] and recomputes the
    /// nominal encounter — a full-nominal propagation and a full-span scan — so it
    /// costs seconds. A renderer must not pay that per nudge: it builds one
    /// [`DeflectionScenario`] and one [`nominal_hit`](Self::nominal_hit) up front,
    /// then calls [`frame_from`](Self::frame_from) per nudge, which re-propagates
    /// only the short post-deflection arc. Use this wrapper for one-off frames
    /// (tests, tooling); use `frame_from` in the animation loop.
    pub fn encounter_frame(
        &self,
        deflection_epoch: Epoch,
        delta_v: Vector3<f64>,
        half_window_seconds: f64,
        n_samples: usize,
    ) -> Result<EncounterFrame, ScenarioError> {
        let ds = self.deflection()?;
        let nominal_enc = self.nominal_hit(&ds)?;
        self.frame_from(
            &ds,
            nominal_enc,
            deflection_epoch,
            delta_v,
            half_window_seconds,
            n_samples,
        )
    }

    /// The nominal Earth encounter — the hit being undone — for a built `ds`. It
    /// scans the full nominal span, so the animation loop computes it **once** (the
    /// nominal never changes) and passes it to [`frame_from`](Self::frame_from)
    /// each nudge rather than re-scanning.
    pub fn nominal_hit(
        &self,
        ds: &DeflectionScenario<'_>,
    ) -> Result<BPlaneEncounter, ScenarioError> {
        ds.nominal_encounter()?.ok_or_else(|| {
            ScenarioError::NominalNotAHit("no nominal close approach inside the scan gate".into())
        })
    }

    /// Sample the encounter frame using an already-built [`DeflectionScenario`] and
    /// its precomputed [`nominal_hit`](Self::nominal_hit), so the expensive
    /// full-nominal propagation and scan happen once and each nudge pays only
    /// [`DeflectionScenario::deflected_trajectory`]'s short post-deflection arc —
    /// the sub-second per-nudge cost. `ds` must be one this scenario produced (via
    /// [`deflection`](Self::deflection)); it shares this scenario's Earth source, so
    /// the geocentric transform below is consistent with the b-plane it reports.
    ///
    /// The deflected track and its reported perigee come from the *same*
    /// propagation, so what the animation draws and what
    /// [`EncounterFrame::deflected_perigee`] annotates cannot diverge.
    pub fn frame_from(
        &self,
        ds: &DeflectionScenario<'_>,
        nominal_enc: BPlaneEncounter,
        deflection_epoch: Epoch,
        delta_v: Vector3<f64>,
        half_window_seconds: f64,
        n_samples: usize,
    ) -> Result<EncounterFrame, ScenarioError> {
        // One propagation gives both the deflected track (the clock) and its
        // b-plane perigee, so the drawing and the number agree by construction.
        let (clock, deflected_enc) = ds.deflected_trajectory(deflection_epoch, delta_v)?;

        let n = n_samples.max(2);
        let center = self.impact_epoch.tdb_seconds_past_j2000();
        let t_defl = deflection_epoch.tdb_seconds_past_j2000();

        let mut sample_seconds = Vec::with_capacity(n);
        let mut nominal = Vec::with_capacity(n);
        let mut deflected = Vec::with_capacity(n);

        for i in 0..n {
            let frac = i as f64 / (n - 1) as f64;
            let t = center - half_window_seconds + 2.0 * half_window_seconds * frac;
            let epoch = Epoch::from_tdb_seconds_past_j2000(t);

            let earth_pos = self
                .earth
                .state_at(epoch)
                .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
                .position;

            let ast_nom = ds
                .nominal()
                .state_at(epoch)
                .map_err(|e| ScenarioError::Integration(e.to_string()))?
                .position;

            // Before the deflection epoch the asteroid is still on the nominal
            // track (the impulse has not happened yet); after it, read the
            // post-deflection clock. For the animation's near-impact window this
            // is always the post-deflection branch, but the guard keeps the helper
            // honest for a window that reaches back across the nudge.
            let ast_defl = if t < t_defl {
                ast_nom
            } else {
                clock
                    .state_at(epoch)
                    .map_err(|e| ScenarioError::Integration(e.to_string()))?
                    .position
            };

            sample_seconds.push(t);
            nominal.push(ast_nom - earth_pos);
            deflected.push(ast_defl - earth_pos);
        }

        Ok(EncounterFrame {
            sample_seconds,
            nominal,
            deflected,
            earth_radius: self.earth_radius,
            capture_radius: nominal_enc.capture_radius,
            nominal_perigee: nominal_enc.perigee,
            deflected_perigee: deflected_enc.map(|e| e.perigee),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deflection::along_track_unit;

    /// 1 AU in metres — for expressing tolerances as a fraction of an AU.
    const AU_M: f64 = 1.495_978_707e11;

    /// Least distance of a geocentric track from Earth's centre over the window.
    fn min_range(track: &[Vector3<f64>]) -> f64 {
        track.iter().map(|p| p.norm()).fold(f64::INFINITY, f64::min)
    }

    /// `propagate_free` must fly the seed through the scenario's *own* Tier-1
    /// field: a sub-cadence [`Clock::state_at`] query has to agree with a direct
    /// `Dop853` step over the same interval in that same field, to the integrator
    /// tolerance. This pins that the orrery path (a) uses the validated field (not
    /// a fresh/empty one), (b) hands the dense output back correctly, and (c)
    /// serves an arbitrary sub-snapshot epoch, not just cadence boundaries.
    ///
    /// Kernel-gated: needs the DE440 `.bsp`/`.pca` via `ASTEROID_DE_KERNEL` /
    /// `ASTEROID_PLANETARY_CONSTANTS`; skips (does not fail) when they are unset.
    #[test]
    fn propagate_free_matches_direct_step_in_the_field() {
        if std::env::var("ASTEROID_DE_KERNEL").is_err()
            || std::env::var("ASTEROID_PLANETARY_CONSTANTS").is_err()
        {
            eprintln!("skipping propagate_free_matches_direct_step_in_the_field: no DE kernel");
            return;
        }

        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("scenario builds");

        // A bound heliocentric seed at ~2 AU, built in the SSB (integration) frame:
        // Sun's barycentric state plus a circular-ish offset. mu_sun from the same
        // kernel keeps the seed physically sensible.
        let epoch0 = Epoch::from_tdb_gregorian(2030, 1, 1, 0, 0, 0, 0);
        let sun = EphemerisPerturber::new(Arc::clone(sc.ephemeris()), SUN_J2000);
        let sun0 = sun.state_at(epoch0).expect("sun state");
        let mu_sun = sc.ephemeris().gm_km3_s2(SUN_J2000).expect("sun GM") * KM3_S2_TO_M3_S2;
        let r = 2.0 * AU_M;
        let v_circ = (mu_sun / r).sqrt();
        let seed = StateVector::new(
            sun0.position + Vector3::new(r, 0.0, 0.0),
            sun0.velocity + Vector3::new(0.0, v_circ, 0.0),
        );

        let cadence = 5.0 * 86_400.0; // 5-day snapshots
        let n = 24;
        let clock = sc
            .propagate_free(epoch0, seed, cadence, n)
            .expect("free propagation");

        // The clock covers [epoch0, epoch0 + 24·5 d]; span and direction check.
        let (lo, hi) = clock.covered_span();
        let t0 = epoch0.tdb_seconds_past_j2000();
        assert!(
            (lo - t0).abs() < 1e-6,
            "span should start at the seed epoch"
        );
        assert!(
            (hi - (t0 + cadence * n as f64)).abs() < 1.0,
            "span should end n·cadence forward"
        );

        // A deliberately off-boundary sub-snapshot epoch (37.3 d in, between the
        // 7th and 8th snapshots): dense output vs a direct step to that epoch.
        let dt = 37.3 * 86_400.0;
        let direct = Dop853::new()
            .step(&sc.force, epoch0, &seed, dt)
            .expect("direct step");
        let dense = clock
            .state_at(epoch0.shifted_by_seconds(dt))
            .expect("sub-snapshot query");
        let rel = (dense.position - direct.position).norm() / AU_M;
        assert!(
            rel < 1e-8,
            "free-prop dense query diverges from a direct step in the same field: rel {rel:.3e}"
        );

        // Backward propagation reconstructs the past: a negative cadence covers
        // [epoch0 − n·cadence, epoch0], the reverse-time view relies on.
        let back = sc
            .propagate_free(epoch0, seed, -cadence, 6)
            .expect("backward free propagation");
        let (blo, bhi) = back.covered_span();
        assert!(
            (bhi - t0).abs() < 1e-6,
            "backward span ends at the seed epoch"
        );
        assert!(
            (blo - (t0 - cadence * 6.0)).abs() < 1.0,
            "backward span reaches n·cadence into the past"
        );

        // Invalid arguments surface as an error, never a panic (the FFI contract).
        assert!(sc.propagate_free(epoch0, seed, 0.0, 4).is_err());
        assert!(sc.propagate_free(epoch0, seed, cadence, 0).is_err());
    }

    /// The displayed encounter must equal the validated physics: the geocentric
    /// track the animation walks reaches its closest approach at the very b-plane
    /// perigee the solver reports — so the picture cannot show a hit the numbers
    /// call a miss (or vice-versa). We anchor on the **nominal** track because its
    /// closest approach is the impact epoch by construction, i.e. exactly the
    /// window centre, so fine uniform sampling resolves it. The deflected track is
    /// only required to never appear *closer* than its reported perigee (no visual
    /// lie), since a large nudge can shift its closest approach partly out of a
    /// window centred on the nominal impact.
    ///
    /// Kernel-gated: needs the DE440 `.bsp`/`.pca` via `ASTEROID_DE_KERNEL` /
    /// `ASTEROID_PLANETARY_CONSTANTS`. Skips (does not fail) when they are unset,
    /// matching the `curve`/`probe_prop` binaries — the kernel-free physics is
    /// pinned in the crate's own unit tests.
    #[test]
    fn encounter_frame_track_agrees_with_reported_perigee() {
        if std::env::var("ASTEROID_DE_KERNEL").is_err()
            || std::env::var("ASTEROID_PLANETARY_CONSTANTS").is_err()
        {
            eprintln!("skipping encounter_frame_track_agrees_with_reported_perigee: no DE kernel");
            return;
        }

        let cfg = ImpactorConfig::default();
        let sc = RealFieldScenario::build(&cfg).expect("scenario builds");

        // A modest along-track nudge one period before impact — the arc is short
        // (sub-second prop) and the deflected pass stays an encounter (larger, but
        // still finite, perigee) rather than escaping the scan gate.
        let deflection_epoch = sc.impact_epoch().shifted_by_seconds(-sc.period_seconds);
        let ds = sc.deflection().expect("deflection scenario");
        let seed = ds
            .nominal()
            .state_at(deflection_epoch)
            .expect("nominal state");
        let dir = along_track_unit(seed).expect("nominal has a heading");
        let dv = 0.2 * dir; // 0.2 m/s

        // Use the *app's* window and sample count so the test covers the exact
        // resolution the viewer renders (not a finer one that would hide any
        // sampling gap the user actually sees).
        let half_window = ENCOUNTER_HALF_WINDOW_SECONDS;
        let n = ENCOUNTER_SAMPLES;
        let frame = sc
            .encounter_frame(deflection_epoch, dv, half_window, n)
            .expect("encounter frame");

        assert_eq!(frame.nominal.len(), n);
        assert_eq!(frame.deflected.len(), n);
        assert_eq!(frame.sample_seconds.len(), n);
        assert!(
            frame.sample_seconds.windows(2).all(|w| w[1] > w[0]),
            "sample epochs must be strictly ascending"
        );
        assert!(
            frame.capture_radius >= frame.earth_radius && frame.earth_radius > 0.0,
            "capture radius ≥ Earth radius > 0"
        );

        // The nominal is the hit being undone.
        assert!(
            frame.nominal_perigee < frame.capture_radius,
            "nominal perigee {:.3e} m must be inside the capture radius {:.3e} m (a hit)",
            frame.nominal_perigee,
            frame.capture_radius
        );

        // Max geocentric range moved between two adjacent samples, a hard bound on
        // how far the sampled minimum can sit above the continuous perigee.
        let spacing = 2.0 * half_window / (n as f64 - 1.0);
        let slack = cfg.v_rel_kms * KM_TO_M * spacing;

        // The nominal track's closest sample brackets its reported perigee: never
        // inside it (that would be the visual lying about the miss), and no more
        // than one sample-step's worth of range above it.
        let nom_min = min_range(&frame.nominal);
        assert!(
            nom_min >= frame.nominal_perigee - 1.0,
            "nominal track dips below its reported perigee: min {:.3e} < perigee {:.3e}",
            nom_min,
            frame.nominal_perigee
        );
        assert!(
            nom_min <= frame.nominal_perigee + slack,
            "nominal track never reaches its reported perigee: min {:.3e} > perigee {:.3e} + slack {:.3e}",
            nom_min,
            frame.nominal_perigee,
            slack
        );

        // The deflected pass must still be an encounter for this small nudge, and
        // its track must never appear closer than the reported deflected perigee.
        let defl_perigee = frame
            .deflected_perigee
            .expect("a 0.2 m/s nudge should still leave a finite-perigee encounter");
        let defl_min = min_range(&frame.deflected);
        assert!(
            defl_min >= defl_perigee - 1.0,
            "deflected track dips below its reported perigee: min {:.3e} < perigee {:.3e}",
            defl_min,
            defl_perigee
        );
    }
}
