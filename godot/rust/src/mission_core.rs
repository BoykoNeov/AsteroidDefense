//! `MissionCore` — the godot-free heart of the binding.
//!
//! Holds the loaded DE440 ephemeris and (once built) the [`RealFieldScenario`],
//! and answers the two questions the Godot frontend asks: *where is body N at
//! epoch t* (for the solar-system display) and *how much along-track Δv clears
//! the threat at this lead* (the headline number + planner). It deals only in
//! plain Rust / nalgebra types, so it is unit-testable with `cargo test` — no
//! running Godot. The thin [`crate::Mission`] class marshals these to Godot types
//! and never adds logic of its own.
//!
//! **`RealFieldScenario` is `Send`** — the core traits (`ForceModel`,
//! `PerturberEphemeris`, `GeocentricState`) carry `Send + Sync` bounds, pinned by
//! a compile-time assertion in `core::scenario`. A built scenario can therefore be
//! produced on a worker thread and moved here, which is the only way the ~10 s
//! build does not freeze Godot's main thread. (This note previously said the
//! opposite; the bounds were added once the measurement showed the build was far
//! too slow to run inline.)
//!
//! What must **not** move to a worker is *this* struct: it serves planet positions
//! (`body_position_ecl_au`) every frame from `load()`, which is ~19 ms and live
//! immediately. Sending it away for the duration of a build would freeze the
//! orrery for those 10 s — the very regression threading exists to prevent. The
//! split that follows from it: clone the `Arc<Ephemeris>`, build a scenario
//! off-thread from that clone, and hand the finished scenario back to this
//! (still-serving) core to install.
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
use asteroid_core::{
    along_track_unit, Clock, DvSolveTol, EphemerisPerturber, Epoch, OrbitalElements, StateVector,
};

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

/// Rotate an **ecliptic-J2000** vector into ICRF (equatorial-J2000) — the exact
/// inverse of the rotation in [`icrf_km_to_ecliptic_au`] (a `+ε` about X vs the
/// forward `−ε`), and **unit-agnostic**: it rotates a vector, so it maps both a
/// position and a velocity. The synthetic-body seed path needs it: a designer
/// orbit is authored with its inclination referred to the *ecliptic* (the plane
/// the display and a human designer think in), but the integrator runs in ICRF,
/// so the element→state result is rotated here before it is seeded.
fn ecliptic_to_icrf(v: Vector3<f64>) -> Vector3<f64> {
    let eps = OBLIQUITY_ARCSEC / 3600.0 * std::f64::consts::PI / 180.0;
    let (s, c) = eps.sin_cos();
    Vector3::new(v.x, c * v.y - s * v.z, s * v.y + c * v.z)
}

/// Discover the loaded kernel's usable coverage window by bisecting on whether
/// Earth resolves — `(lo, hi)` seconds past J2000, inset by [`SPAN_MARGIN_S`].
///
/// Bisection rather than a hardcoded date pair because the mounted kernel decides
/// the answer: de440s covers ~1850–2149, de441 ~1550–2650, and hardcoding the
/// short span would silently cap a user who mounted the long one. Bisection
/// rather than reading the SPK segment headers because coverage is only *useful*
/// where a full geocenter lookup succeeds (SSB→EMB→Earth — all three segments),
/// which is exactly what this probes; a segment table can advertise a span the
/// dereferencing chain cannot actually serve.
///
/// ~40 lookups at ~µs each, once per load. Errors only if the kernel serves no
/// epoch at all (a wrong or corrupt file), which is worth failing loudly on.
fn discover_span(eph: &Ephemeris) -> Result<(f64, f64), ScenarioError> {
    let resolves = |t: f64| -> bool {
        eph.position_km(
            Frame::from_ephem_j2000(399),
            SUN_J2000,
            Epoch::from_tdb_seconds_past_j2000(t).as_hifitime(),
        )
        .is_ok()
    };

    // A kernel that serves nothing anywhere is a load failure, not an empty span.
    if !resolves(0.0) {
        return Err(ScenarioError::Ephemeris(
            "kernel resolves no Earth position at J2000 — wrong or corrupt file?".into(),
        ));
    }

    // Walk each edge in from a bracket known to be outside coverage. J2000 is
    // inside (checked above), so each bisection is well-posed.
    let mut lo = (PROBE_LO_S, 0.0); // (fails, works)
    while lo.1 - lo.0 > SPAN_MARGIN_S {
        let mid = 0.5 * (lo.0 + lo.1);
        if resolves(mid) {
            lo.1 = mid
        } else {
            lo.0 = mid
        }
    }
    let mut hi = (0.0, PROBE_HI_S); // (works, fails)
    while hi.1 - hi.0 > SPAN_MARGIN_S {
        let mid = 0.5 * (hi.0 + hi.1);
        if resolves(mid) {
            hi.0 = mid
        } else {
            hi.1 = mid
        }
    }
    Ok((lo.1 + SPAN_MARGIN_S, hi.0 - SPAN_MARGIN_S))
}

/// One body in the orrery catalog: a pre-integrated, dense-output trajectory the
/// display scrubs over. The `Clock` is built **once** (at [`MissionCore::
/// add_synthetic_body`]) in the same validated Tier-1 field as the threat, so a
/// scrub query is a cheap dense-output evaluation, never a re-integration.
struct OrreryBody {
    /// Display label (e.g. `"C/2029 K1"`).
    name: String,
    /// Coarse class the frontend styles on (`"asteroid"`, `"comet"`, …).
    kind: String,
    /// The pre-integrated trajectory in SSB metres (the integration frame).
    clock: Clock,
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

/// A safety margin pulled in from each discovered coverage edge, seconds (1 day).
/// The bisection lands within a day of the true edge; insetting by that much
/// guarantees the reported span is *inside* coverage rather than straddling it,
/// so a clock clamped to this span never asks for an epoch the kernel lacks.
const SPAN_MARGIN_S: f64 = 86_400.0;

/// Bisection bounds for span discovery, seconds past J2000 — years ~1000 and
/// ~3000, comfortably outside any DE kernel's coverage (de440s ≈ 1850–2149;
/// de441 ≈ 1550–2650), so the true edges are always bracketed.
const PROBE_LO_S: f64 = -31_557_600_000.0;
const PROBE_HI_S: f64 = 31_557_600_000.0;

/// The loaded mission: always an ephemeris, optionally a built scenario, and —
/// once built — the cached nominal trajectory and (optionally) a deflection plan.
pub struct MissionCore {
    ephemeris: Arc<Ephemeris>,
    /// The kernel's usable coverage window, `(lo, hi)` seconds past J2000,
    /// discovered by bisection at load (see [`discover_span`]) rather than
    /// hardcoded — the shipped kernel may be de440s (~1850–2149) or the long-span
    /// de441 (~1550–2650), and the frontend clamps its clock to whatever is
    /// actually mounted.
    span: (f64, f64),
    scenario: Option<RealFieldScenario>,
    /// The nominal (un-deflected) trajectory, cloned **once** at build time so
    /// per-frame position/track reads are cheap `Clock` queries. Rebuilding a
    /// `DeflectionScenario` re-propagates the whole multi-year nominal
    /// (`deflection.rs`), so we never do that on a display read.
    nominal_clock: Option<Clock>,
    /// The current deflection plan, recomputed only on [`set_plan`](Self::set_plan)
    /// and read cheaply thereafter.
    plan: Option<PlanState>,
    /// The orrery catalog: extra bodies (synthetic designer comets/asteroids now,
    /// real cataloged bodies later) each pre-integrated into a dense-output `Clock`
    /// at add time, so the multi-body display scrubs cheaply. Independent of the
    /// threat/plan; indexed by insertion order.
    bodies: Vec<OrreryBody>,
}

impl MissionCore {
    /// Read the DE440 kernels named by `ASTEROID_DE_KERNEL` (the `.bsp`) and
    /// `ASTEROID_PLANETARY_CONSTANTS` (the `.pca`) and hold them. The env-var
    /// convention matches the core tests and the `curve`/viewer binaries — all of
    /// which run from a developer shell.
    ///
    /// **A launched Godot game generally has neither variable set** (they are not
    /// persisted at user or machine level), so the frontend resolves paths itself
    /// and calls [`load_from`](Self::load_from). This stays as the shell/test
    /// entry point.
    pub fn load() -> Result<Self, ScenarioError> {
        let bsp = std::env::var("ASTEROID_DE_KERNEL")
            .map_err(|_| ScenarioError::MissingKernelEnv("ASTEROID_DE_KERNEL"))?;
        let pca = std::env::var("ASTEROID_PLANETARY_CONSTANTS")
            .map_err(|_| ScenarioError::MissingKernelEnv("ASTEROID_PLANETARY_CONSTANTS"))?;
        Self::load_from(&bsp, &pca)
    }

    /// Read the DE kernels at two explicit paths — the entry point for any caller
    /// that resolves paths itself rather than through the environment (the Godot
    /// frontend, which cannot rely on env vars reaching a double-clicked game).
    ///
    /// Fast (~ms plus a short span bisection): enables
    /// [`body_position_ecl_au`](Self::body_position_ecl_au) immediately; the
    /// scenario is built separately.
    pub fn load_from(bsp: &str, pca: &str) -> Result<Self, ScenarioError> {
        let eph = Ephemeris::load(bsp)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
            .with_constants(pca)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        let span = discover_span(&eph)?;
        Ok(Self {
            ephemeris: Arc::new(eph),
            span,
            scenario: None,
            nominal_clock: None,
            plan: None,
            bodies: Vec::new(),
        })
    }

    /// The loaded kernel's usable coverage window, `(lo, hi)` seconds past J2000.
    /// The frontend clamps its clock to this — outside it every body lookup fails,
    /// and a failed lookup is indistinguishable from "at the Sun" downstream.
    pub fn usable_span_tdb(&self) -> (f64, f64) {
        self.span
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
        self.bodies.clear(); // …and any orrery bodies flown in the old field
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

    // --- Orrery catalog (the multi-body, long-span, scrubbable display) --------

    /// Add a **synthetic designer body** to the orrery catalog and return its
    /// index. The orbit is given by classical Keplerian `elements` referred to the
    /// **ecliptic** (the plane the display and a human designer reason in), valid
    /// at `epoch0`; the body is then integrated **once** through the scenario's
    /// validated Tier-1 field into a dense-output [`Clock`] spanning
    /// `n_snapshots · cadence_seconds` from `epoch0` (sign of the cadence sets the
    /// direction — a forward span for a body seeded at the display's start epoch).
    ///
    /// Requires [`build_scenario`](Self::build_scenario) first (the field lives on
    /// the scenario). The seed is built in the integration frame: element→state
    /// about the Sun (heliocentric, ecliptic), rotate ecliptic→ICRF, add the Sun's
    /// SSB state — the exact inverse of the read path, so a query back at `epoch0`
    /// recovers the authored position.
    ///
    /// **Cost:** one N-body integration over the whole span (seconds for a
    /// multi-decade comet). Call at load, not per frame; reads are cheap after.
    pub fn add_synthetic_body(
        &mut self,
        name: &str,
        kind: &str,
        elements: OrbitalElements,
        epoch0: Epoch,
        cadence_seconds: f64,
        n_snapshots: u32,
    ) -> Result<usize, ScenarioError> {
        let sc = self
            .scenario
            .as_ref()
            .ok_or_else(|| ScenarioError::NominalNotAHit("scenario not built".into()))?;
        let mu_sun = self
            .ephemeris
            .sun_gm_m3_s2()
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;

        // Heliocentric ecliptic state from the elements, rotated into the ICRF
        // integration frame, then lifted to SSB by adding the Sun's barycentric
        // state — the seed the field integrates.
        let helio_ecl = elements.to_state(mu_sun);
        let helio_icrf = StateVector::new(
            ecliptic_to_icrf(helio_ecl.position),
            ecliptic_to_icrf(helio_ecl.velocity),
        );
        let sun_ssb = EphemerisPerturber::new(Arc::clone(&self.ephemeris), SUN_J2000)
            .state_at(epoch0)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        let seed = StateVector::new(
            helio_icrf.position + sun_ssb.position,
            helio_icrf.velocity + sun_ssb.velocity,
        );

        let clock = sc.propagate_free(epoch0, seed, cadence_seconds, n_snapshots)?;
        self.bodies.push(OrreryBody {
            name: name.to_string(),
            kind: kind.to_string(),
            clock,
        });
        Ok(self.bodies.len() - 1)
    }

    /// Number of bodies in the orrery catalog.
    pub fn catalog_count(&self) -> usize {
        self.bodies.len()
    }

    /// Display label of catalog body `i` (`None` if out of range).
    pub fn catalog_name(&self, i: usize) -> Option<&str> {
        self.bodies.get(i).map(|b| b.name.as_str())
    }

    /// Coarse class of catalog body `i` (`"asteroid"`/`"comet"`/…; `None` if OOR).
    pub fn catalog_kind(&self, i: usize) -> Option<&str> {
        self.bodies.get(i).map(|b| b.kind.as_str())
    }

    /// The propagated span of catalog body `i` as `(lo, hi)` seconds past J2000 —
    /// the frontend clamps/hides the body outside this (the reverse/long scrub
    /// exposes bodies with a bounded arc). `None` if `i` is out of range.
    pub fn catalog_span_tdb(&self, i: usize) -> Option<(f64, f64)> {
        self.bodies.get(i).map(|b| b.clock.covered_span())
    }

    /// Position of catalog body `i` at `tdb`, heliocentric **ecliptic AU** — the
    /// same display frame as the planets and the threat. `None` if `i` is out of
    /// range or `tdb` is outside the body's propagated span (the frontend uses
    /// [`catalog_span_tdb`](Self::catalog_span_tdb) to know which).
    pub fn catalog_position_ecl_au(&self, i: usize, tdb: f64) -> Option<Vector3<f64>> {
        let b = self.bodies.get(i)?;
        let epoch = Epoch::from_tdb_seconds_past_j2000(tdb);
        let st = b.clock.state_at(epoch).ok()?;
        self.ssb_m_to_helio_ecl_au(st.position, epoch)
    }

    /// Catalog body `i`'s trajectory as `n` heliocentric ecliptic-AU points across
    /// its whole propagated span — the orbit polyline. Sampled **once**. Empty if
    /// `i` is out of range.
    pub fn catalog_track_ecl_au(&self, i: usize, n: usize) -> Vec<Vector3<f64>> {
        let Some(b) = self.bodies.get(i) else {
            return Vec::new();
        };
        let (t0, t1) = b.clock.covered_span();
        self.track_ecl_au(n, t0, t1, |e| b.clock.state_at(e).ok().map(|s| s.position))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn have_kernels() -> bool {
        std::env::var("ASTEROID_DE_KERNEL").is_ok()
            && std::env::var("ASTEROID_PLANETARY_CONSTANTS").is_ok()
    }

    /// Metres per AU — for authoring synthetic-body semi-major axes in SI.
    const AU_M: f64 = AU_KM * M_PER_KM;

    /// 2035-01-01 TDB — comfortably inside the de440s span; the synthetic-body
    /// seed epoch for the catalog tests.
    fn epoch_2035() -> Epoch {
        Epoch::from_tdb_gregorian(2035, 1, 1, 0, 0, 0, 0)
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

    /// Kernel-gated. Every NAIF id the orrery display draws must resolve at
    /// **both edges** of the usable span, not just mid-span — a failed lookup
    /// returns `None`, which the binding maps to `Vector3::ZERO`, and ZERO is the
    /// *Sun's* position in this heliocentric frame. So a body that falls out of
    /// coverage does not render as visibly broken; it renders silently sitting on
    /// the Sun. This test is what stands between that and a shipped display.
    ///
    /// Two id choices are pinned here because the obvious guess is wrong:
    /// **Earth is 399, never 3** (3 is the Earth–Moon barycenter — the ~4671 km
    /// footgun of HANDOFF §5), and **Mars is 4, not 499** (de440s carries no Mars
    /// *geocenter* segment at all; the barycenter is all there is, and it sits
    /// within a few km of the planet, so it is harmless here — unlike Earth's).
    #[test]
    fn display_naif_ids_resolve_across_the_whole_usable_span() {
        if !have_kernels() {
            eprintln!("skipping display_naif_ids_*: no DE kernel");
            return;
        }
        let mc = MissionCore::load().expect("load kernels");
        let (span_lo, span_hi) = mc.usable_span_tdb();

        // The exact id list the orrery draws, with the heliocentric distance band
        // each must land in (AU) anywhere in the span. Bands are wide enough for
        // the real eccentric excursion, tight enough to catch a wrong body.
        let bodies: [(i32, &str, f64, f64); 8] = [
            (199, "MERCURY", 0.30, 0.48),
            (299, "VENUS", 0.71, 0.74),
            (399, "EARTH", 0.98, 1.02),
            (4, "MARS", 1.38, 1.68),
            (5, "JUPITER", 4.94, 5.46),
            (6, "SATURN", 8.99, 10.10),
            (7, "URANUS", 18.28, 20.10),
            (8, "NEPTUNE", 29.79, 30.33),
        ];

        for t in [span_lo, 0.0, span_hi] {
            for (id, name, lo, hi) in bodies {
                let p = mc.body_position_ecl_au(id, t).unwrap_or_else(|| {
                    panic!(
                        "{name} (NAIF {id}) does not resolve at TDB {t:.0} — it would render \
                         silently at the Sun, not visibly missing"
                    )
                });
                assert!(
                    (lo..=hi).contains(&p.norm()),
                    "{name} (NAIF {id}) at TDB {t:.0}: {:.3} AU outside [{lo}, {hi}]",
                    p.norm()
                );
                assert_ne!(p, Vector3::zeros(), "{name} returned the Sun's position");
            }
        }

        // Mars has no geocenter segment in de440s — pinned so a future "tidy-up"
        // to 499 (matching Earth's 399) fails loudly here instead of silently at
        // the Sun. If a mounted kernel ever gains it, prefer it and update this.
        assert!(
            mc.body_position_ecl_au(499, 0.0).is_none(),
            "this kernel resolves Mars 499 — prefer the geocenter over the \
             barycenter in the display and update this test"
        );
    }

    /// Kernel-gated. The discovered span must be genuinely usable at both edges
    /// and genuinely exhausted just outside them — the property the frontend's
    /// clock clamp relies on. Asserts the *shape* (a sane multi-century window
    /// bracketing J2000), not hardcoded dates, since the mounted kernel decides
    /// them: de440s ≈ 1850–2149, de441 ≈ 1550–2650.
    #[test]
    fn discovered_span_is_usable_inside_and_exhausted_outside() {
        if !have_kernels() {
            eprintln!("skipping discovered_span_*: no DE kernel");
            return;
        }
        let mc = MissionCore::load().expect("load kernels");
        let (lo, hi) = mc.usable_span_tdb();
        let year = 365.25 * 86_400.0;

        assert!(lo < 0.0 && hi > 0.0, "span should bracket J2000");
        assert!(
            (hi - lo) / year > 100.0,
            "span {:.0} yr implausibly short for a DE kernel",
            (hi - lo) / year
        );
        // Inside at both edges…
        assert!(mc.body_position_ecl_au(399, lo).is_some(), "span lo unusable");
        assert!(mc.body_position_ecl_au(399, hi).is_some(), "span hi unusable");
        // …and exhausted a year out, so the span is the real edge, not a guess
        // that happens to be conservative by decades.
        assert!(
            mc.body_position_ecl_au(399, lo - year).is_none(),
            "a year below the span still resolves — discovery under-reports coverage"
        );
        assert!(
            mc.body_position_ecl_au(399, hi + year).is_none(),
            "a year above the span still resolves — discovery under-reports coverage"
        );
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

    /// Kernel-gated (release-run). The orrery seed path is correct end-to-end. A
    /// synthetic body authored with **ecliptic** elements and **zero inclination**
    /// must (a) read back at its seed epoch as the *authored* heliocentric position
    /// — proving the ecliptic→ICRF→+Sun seed is the exact inverse of the read path
    /// — and (b) stay in the ecliptic plane (|z| ≈ 0) all along its integrated
    /// track, which it would NOT if the ecliptic↔ICRF rotation were wrong (a ~23°
    /// tilt would lift z by up to ~0.4·r). Also checks the orbit is physically on
    /// its designed ellipse (distance in `[a(1−e), a(1+e)]`) and the metadata.
    #[test]
    fn synthetic_body_seeds_and_frames_correctly() {
        if !have_kernels() {
            eprintln!("skipping synthetic_body_seeds_and_frames_correctly: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        // Adding a body before a scenario is built is an error, not a panic.
        // (Re-checked here since build already ran; use a fresh core for the guard.)
        let mut unbuilt = MissionCore::load().expect("load kernels");
        let planar = OrbitalElements::new(2.0 * AU_M, 0.2, 0.0, 0.0, 0.0, 0.0);
        assert!(unbuilt
            .add_synthetic_body("X", "asteroid", planar, epoch_2035(), 5.0 * 86_400.0, 4)
            .is_err());

        let a_m = 2.0 * AU_M;
        let e = 0.2;
        let elements = OrbitalElements::new(a_m, e, 0.0, 0.0, 0.0, 0.0); // ecliptic, planar
        let epoch0 = epoch_2035();
        let epoch0_tdb = epoch0.tdb_seconds_past_j2000();
        let cadence = 5.0 * 86_400.0;
        let n = 146; // ~2 years — most of one orbit (T = 2^1.5 ≈ 2.83 yr)

        // The authored heliocentric ecliptic position, in AU, for the round-trip.
        let mu_sun = mc.ephemeris.sun_gm_m3_s2().expect("sun GM");
        let expected_ecl_au = elements.to_state(mu_sun).position / AU_M;

        let idx = mc
            .add_synthetic_body("TEST-COMET", "comet", elements, epoch0, cadence, n)
            .expect("add synthetic body");
        assert_eq!(idx, 0);
        assert_eq!(mc.catalog_count(), 1);
        assert_eq!(mc.catalog_name(idx), Some("TEST-COMET"));
        assert_eq!(mc.catalog_kind(idx), Some("comet"));

        // (a) Seed round-trip: at epoch0 the read recovers the authored position.
        let at0 = mc
            .catalog_position_ecl_au(idx, epoch0_tdb)
            .expect("position at seed epoch");
        assert!(
            (at0 - expected_ecl_au).norm() < 1e-6,
            "seed round-trip off by {:.3e} AU — ecliptic↔ICRF seed/read not inverse",
            (at0 - expected_ecl_au).norm()
        );

        // Span covers [epoch0, epoch0 + n·cadence]; used to clamp/hide the body.
        let (lo, hi) = mc.catalog_span_tdb(idx).expect("span");
        assert!((lo - epoch0_tdb).abs() < 1.0);
        assert!((hi - (epoch0_tdb + cadence * n as f64)).abs() < 1.0);

        // (b) Planarity + on-ellipse across the whole track.
        let track = mc.catalog_track_ecl_au(idx, 200);
        assert_eq!(track.len(), 200, "track should be a full n-point line");
        for p in &track {
            assert!(
                p.z.abs() < 0.02,
                "planar (i=0) ecliptic orbit lifted to |z| = {:.4} AU — rotation wrong",
                p.z.abs()
            );
            assert!(
                (1.55..=2.45).contains(&p.norm()),
                "distance {:.4} AU outside the designed ellipse [a(1−e), a(1+e)]",
                p.norm()
            );
        }

        // Out-of-range index and out-of-span epoch both return None (no panic).
        assert!(mc.catalog_position_ecl_au(9, epoch0_tdb).is_none());
        assert!(mc
            .catalog_position_ecl_au(idx, epoch0_tdb - 1.0e9)
            .is_none());
    }
}
